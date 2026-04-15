#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

/* ================= RADIO CONFIG ================= */
#define ESPNOW_CHANNEL 6
#define SEND_INTERVAL_MS 20

// Start with broadcast for bench testing. For racing, paste the receiver MAC.
uint8_t RECEIVER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ================= TRANSMITTER PIN CONFIG ================= */
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ADS1115_ADDR 0x48

#define SPI_SCK_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define DISPLAY_CS_PIN 15
#define DISPLAY_DC_PIN 16
#define DISPLAY_RST_PIN 17

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6

/* ================= DISPLAY CONFIG ================= */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define DISPLAY_ROTATION 0

/* ================= PACKET FORMAT ================= */
#define RC_PACKET_VERSION 1
#define BTN_KILL 0x01
#define BTN_LIGHTS 0x02
#define BTN_AUX1 0x04

struct __attribute__((packed)) ControlPacket {
  uint8_t version;
  uint16_t sequence;
  int16_t throttle;     // -1000 reverse, 0 stop, +1000 forward
  int16_t steering;     // -1000 left, 0 center, +1000 right
  uint16_t speedLimit;  // 0..1000
  int16_t aux1;         // raw mapped joystick axis
  int16_t aux2;         // raw mapped joystick axis
  uint8_t mode;         // 1..3
  uint8_t buttons;      // BTN_* mask
  uint16_t crc;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t version;
  uint16_t sequenceAck;
  uint16_t batteryMv;
  int16_t leftMotor;
  int16_t rightMotor;
  uint8_t linkState;
  uint16_t packetAgeMs;
  uint16_t crc;
};

static_assert(sizeof(ControlPacket) <= 250, "ESP-NOW v1 packet must stay <= 250 bytes");

Adafruit_ADS1115 ads;
Adafruit_ST7789 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

struct AxisCal {
  int16_t minRaw;
  int16_t centerRaw;
  int16_t maxRaw;
  bool invert;
  int16_t deadbandRaw;
};

// Replace these after running tests/transmitter_pot_test.
AxisCal axisCal[4] = {
  {4100, 13200, 22400, true,  80}, // A0 throttle
  {4100, 13200, 22400, false, 80}, // A1 steering
  {4100, 13200, 22400, false, 80}, // A2 aux / speed trim if needed
  {4100, 13200, 22400, false, 80}  // A3 mode select
};

class AxisFilter {
public:
  int16_t push(int16_t value) {
    total -= samples[index];
    samples[index] = value;
    total += value;
    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) count++;
    return total / count;
  }

private:
  static const uint8_t FILTER_SIZE = 5;
  int16_t samples[FILTER_SIZE] = {0, 0, 0, 0, 0};
  int32_t total = 0;
  uint8_t index = 0;
  uint8_t count = 0;
};

AxisFilter filters[4];

volatile bool sendReady = true;
volatile bool lastSendOk = false;
volatile bool telemetryReady = false;
TelemetryPacket latestTelemetry = {};

uint16_t sequenceNumber = 0;
uint16_t speedLimit = 700;
bool killLatched = true;
bool lightsEnabled = true;
uint32_t lastSendMs = 0;
uint32_t lastScreenMs = 0;
uint32_t lastSerialMs = 0;

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

uint16_t controlCrc(const ControlPacket &packet) {
  ControlPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

uint16_t telemetryCrc(const TelemetryPacket &packet) {
  TelemetryPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

bool isBroadcastMac(const uint8_t *mac) {
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] != 0xFF) return false;
  }
  return true;
}

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

#if ESP_IDF_VERSION_MAJOR > 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 5)
void onDataSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendReady = true;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onTelemetryRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
void onTelemetryRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len != sizeof(TelemetryPacket)) return;

  TelemetryPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.version != RC_PACKET_VERSION) return;
  if (telemetryCrc(packet) != packet.crc) return;

  latestTelemetry = packet;
  telemetryReady = true;
}

int16_t splitMapAxis(int16_t raw, const AxisCal &cal) {
  int32_t value;

  if (abs(raw - cal.centerRaw) <= cal.deadbandRaw) {
    value = 0;
  } else if (raw > cal.centerRaw) {
    int32_t span = max<int32_t>(1, cal.maxRaw - cal.centerRaw - cal.deadbandRaw);
    value = (int32_t)(raw - cal.centerRaw - cal.deadbandRaw) * 1000L / span;
  } else {
    int32_t span = max<int32_t>(1, cal.centerRaw - cal.minRaw - cal.deadbandRaw);
    value = -(int32_t)(cal.centerRaw - raw - cal.deadbandRaw) * 1000L / span;
  }

  value = constrain(value, -1000, 1000);
  if (cal.invert) value = -value;
  return (int16_t)value;
}

int16_t readAxis(uint8_t channel) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  int16_t filtered = filters[channel].push(raw);
  return splitMapAxis(filtered, axisCal[channel]);
}

void updateEncoder() {
  static int lastA = HIGH;
  static bool lastButton = HIGH;
  static uint32_t lastButtonChange = 0;

  int a = digitalRead(ENCODER_A_PIN);
  if (a != lastA && a == LOW) {
    if (digitalRead(ENCODER_B_PIN) == HIGH) speedLimit += 25;
    else speedLimit -= 25;
    speedLimit = constrain(speedLimit, 150, 1000);
  }
  lastA = a;

  bool button = digitalRead(ENCODER_SW_PIN);
  if (button != lastButton && millis() - lastButtonChange > 40) {
    lastButtonChange = millis();
    lastButton = button;
    if (button == LOW) {
      killLatched = !killLatched;
    }
  }
}

uint8_t modeFromAxis(int16_t axis) {
  if (axis < -333) return 1;
  if (axis > 333) return 3;
  return 2;
}

ControlPacket buildPacket() {
  ControlPacket packet = {};
  packet.version = RC_PACKET_VERSION;
  packet.sequence = ++sequenceNumber;
  packet.throttle = readAxis(0);
  packet.steering = readAxis(1);
  packet.aux1 = readAxis(2);
  packet.aux2 = readAxis(3);
  packet.speedLimit = speedLimit;
  packet.mode = modeFromAxis(packet.aux2);
  packet.buttons = 0;
  if (killLatched) packet.buttons |= BTN_KILL;
  if (lightsEnabled) packet.buttons |= BTN_LIGHTS;
  packet.crc = controlCrc(packet);
  return packet;
}

void drawScreen(const ControlPacket &packet) {
  if (millis() - lastScreenMs < 100) return;
  lastScreenMs = millis();

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.print("RC TX");

  tft.setCursor(48, 0);
  tft.setTextColor(killLatched ? ST77XX_RED : ST77XX_GREEN);
  tft.print(killLatched ? "KILL" : "ARMED");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 18);
  tft.print("THR ");
  tft.print(packet.throttle);
  tft.setCursor(0, 32);
  tft.print("STR ");
  tft.print(packet.steering);
  tft.setCursor(0, 46);
  tft.print("LIM ");
  tft.print(packet.speedLimit / 10);
  tft.print("%");
  tft.setCursor(72, 46);
  tft.print("M");
  tft.print(packet.mode);

  tft.setCursor(0, 66);
  tft.setTextColor(lastSendOk ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.print(lastSendOk ? "TX OK" : "TX WAIT");

  if (telemetryReady) {
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("RX ");
    tft.print(latestTelemetry.packetAgeMs);
    tft.print("ms");

    tft.setCursor(0, 94);
    tft.print("L ");
    tft.print(latestTelemetry.leftMotor);
    tft.setCursor(64, 94);
    tft.print("R ");
    tft.print(latestTelemetry.rightMotor);
  } else {
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_RED);
    tft.print("NO RX TEL");
  }
}

void printStatus(const ControlPacket &packet) {
  if (millis() - lastSerialMs < 500) return;
  lastSerialMs = millis();

  Serial.printf("seq=%u thr=%d steer=%d limit=%u mode=%u kill=%u tx=%s",
                packet.sequence, packet.throttle, packet.steering,
                packet.speedLimit, packet.mode, killLatched,
                lastSendOk ? "ok" : "pending/fail");
  if (telemetryReady) {
    Serial.printf(" rxAge=%ums left=%d right=%d batt=%umV",
                  latestTelemetry.packetAgeMs,
                  latestTelemetry.leftMotor,
                  latestTelemetry.rightMotor,
                  latestTelemetry.batteryMv);
  }
  Serial.println();
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onTelemetryRecv);

  if (!addPeer(RECEIVER_MAC)) {
    Serial.println("Receiver peer add failed");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ADS1115 not found. Check SDA/SCL, power, and ADDR to GND.");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tft.init(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  tft.setRotation(DISPLAY_ROTATION);
  tft.fillScreen(ST77XX_BLACK);

  setupEspNow();

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Receiver peer: ");
  printMac(RECEIVER_MAC);
  Serial.println(isBroadcastMac(RECEIVER_MAC) ? " (broadcast)" : " (direct)");
  Serial.println("Encoder button toggles kill. Start with wheels off ground.");
}

void loop() {
  updateEncoder();
  ControlPacket packet = buildPacket();

  if (sendReady && millis() - lastSendMs >= SEND_INTERVAL_MS) {
    sendReady = false;
    lastSendMs = millis();
    esp_err_t result = esp_now_send(RECEIVER_MAC, (const uint8_t *)&packet, sizeof(packet));
    if (result != ESP_OK) {
      sendReady = true;
      lastSendOk = false;
    }
  }

  drawScreen(packet);
  printStatus(packet);
}
