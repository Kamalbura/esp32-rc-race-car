#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_NeoPixel.h>

/* ================= RADIO CONFIG ================= */
#define ESPNOW_CHANNEL 6
#define SEND_INTERVAL_MS 10
#define TELEMETRY_STALE_MS 500

uint8_t RECEIVER_MAC[6] = {0xB4, 0x3A, 0x45, 0x3F, 0xA4, 0xE8};

const uint8_t ESPNOW_PMK[16] = {
  0x52, 0x43, 0x52, 0x41, 0x43, 0x45, 0x50, 0x4D,
  0x4B, 0x32, 0x30, 0x32, 0x36, 0x30, 0x34, 0x15
};

const uint8_t ESPNOW_LMK[16] = {
  0x42, 0x54, 0x53, 0x45, 0x53, 0x50, 0x4E, 0x4F,
  0x57, 0x52, 0x41, 0x57, 0x41, 0x44, 0x43, 0x31
};

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
#define BOOT_BUTTON_PIN 0

/* ================= DISPLAY CONFIG ================= */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define DISPLAY_ROTATION 0

/* ================= STATUS RGB LED ================= */
#define BUILTIN_RGB_PIN 48
#define BUILTIN_RGB_COUNT 1
#define BUILTIN_RGB_BRIGHTNESS 24

/* ================= PACKET FORMAT ================= */
#define RC_PACKET_VERSION 1
#define BTN_KILL 0x01
#define BTN_LIGHTS 0x02
#define BTN_AUX1 0x04
#define BTN_CALIBRATION BTN_AUX1

#define BUTTON_DEBOUNCE_MS 40
#define BUTTON_LONG_PRESS_MS 700
#define BUTTON_MULTI_CLICK_MS 700
#define ENCODER_STEP_DEBOUNCE_MS 3

#define STEERING_AXIS_CH 0
#define THROTTLE_AXIS_CH 1
#define AUX1_AXIS_CH 2
#define AUX2_AXIS_CH 3

struct __attribute__((packed)) ControlPacket {
  uint8_t version;
  uint16_t sequence;
  int16_t throttle;     // -1000 reverse, 0 stop, +1000 forward
  int16_t steering;     // -1000 left, 0 center, +1000 right
  uint16_t speedLimit;  // 0..255
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
Adafruit_ST7735 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);
Adafruit_NeoPixel statusPixel(BUILTIN_RGB_COUNT, BUILTIN_RGB_PIN, NEO_GRB + NEO_KHZ800);

struct AxisCal {
  int16_t minRaw;
  int16_t centerRaw;
  int16_t maxRaw;
  bool invert;
  int16_t deadbandRaw;
};

// Current bench mapping:
// A0 = joystick horizontal (steering), A1 = joystick vertical (throttle).
// The previous build had these swapped, which caused:
// forward->left, right->reverse, left->forward, back->right.
// These provisional min/max values should still be replaced by a clean full capture.
// Calibrated from guided_calibration_test @ 2026-04-16.
// Center values measured at rest:  A0≈21220  A1≈20560  A2≈21090  A3≈20440.
// Deadband raised to 500 (~3.6%) to eliminate center creep from ADC noise.
AxisCal axisCal[4] = {
  {4000, 21220, 32000, true,  500},  // A0 steering  (inverted to match physical direction)
  {4000, 20560, 32000, true,  500},  // A1 throttle  (inverted to match physical direction)
  {4000, 21090, 32000, false, 500},  // A2 aux / speed trim if needed
  {4000, 20440, 32000, false, 500}   // A3 mode select
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
  static const uint8_t FILTER_SIZE = 3;
  int16_t samples[FILTER_SIZE] = {0, 0, 0};
  int32_t total = 0;
  uint8_t index = 0;
  uint8_t count = 0;
};

AxisFilter filters[4];

volatile bool sendReady = true;
volatile bool lastSendOk = false;

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryPacket latestTelemetry = {};
bool telemetryReady = false;
uint32_t lastTelemetryRxMs = 0;

uint16_t sequenceNumber = 0;
uint16_t speedLimit = 128;
bool killLatched = true;
bool lightsEnabled = true;
bool encoderSwitchDown = false;
bool bootButtonDown = false;
bool calibrationMode = false;
uint32_t encoderButtonPressCount = 0;
bool actionButtonDown = false;
bool longPressHandled = false;
uint32_t buttonPressStartMs = 0;
uint8_t shortPressCount = 0;
uint32_t shortPressDeadlineMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastScreenMs = 0;
uint32_t lastSerialMs = 0;

/* ================= DUAL-CORE DISPLAY OFFLOAD ================= */
// Display + serial print run on Core 0 so the control loop on Core 1
// is never blocked by slow SPI writes (~15ms per frame).
ControlPacket lastBuiltPacket = {};
portMUX_TYPE displayMux = portMUX_INITIALIZER_UNLOCKED;

/* ================= ADC READ OPTIMIZATION ================= */
// Throttle + steering are read every cycle (~2.3ms for 2 channels).
// Aux channels are read every 5th cycle to save ~2.3ms on 80% of iterations.
int16_t cachedAux1 = 0;
int16_t cachedAux2 = 0;
uint8_t auxReadCounter = 0;

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
  peer.encrypt = true;
  peer.ifidx = WIFI_IF_STA;
  memcpy(peer.lmk, ESPNOW_LMK, sizeof(ESPNOW_LMK));

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

  portENTER_CRITICAL(&telemetryMux);
  latestTelemetry = packet;
  telemetryReady = true;
  lastTelemetryRxMs = millis();
  portEXIT_CRITICAL(&telemetryMux);
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
  static bool initialized = false;
  static uint8_t lastClk = HIGH;
  static bool rawButtonState = false;
  static bool lastStableButton = false;
  static uint32_t lastBounceMs = 0;
  static uint32_t lastEncoderStepMs = 0;

  uint8_t clk = digitalRead(ENCODER_A_PIN);
  uint8_t dt = digitalRead(ENCODER_B_PIN);

  if (!initialized) {
    lastClk = clk;
    initialized = true;
  } else if (clk != lastClk) {
    uint32_t now = millis();
    if (clk == LOW && now - lastEncoderStepMs >= ENCODER_STEP_DEBOUNCE_MS) {
      int delta = (dt != clk) ? 5 : -5;
      speedLimit = constrain(speedLimit + delta, 0, 255);
      lastEncoderStepMs = now;
      Serial.printf("speed_limit=%u clk=%u dt=%u\n", speedLimit, clk, dt);
    }
    lastClk = clk;
  }

  bool encoderButton = digitalRead(ENCODER_SW_PIN);
  bool bootButton = digitalRead(BOOT_BUTTON_PIN);
  encoderSwitchDown = (encoderButton == LOW);
  bootButtonDown = (bootButton == LOW);
  bool button = encoderSwitchDown || bootButtonDown;

  if (button != rawButtonState) {
    rawButtonState = button;
    lastBounceMs = millis();
  }

  if (millis() - lastBounceMs >= BUTTON_DEBOUNCE_MS && button != lastStableButton) {
    lastStableButton = button;

    if (button) {
      actionButtonDown = true;
      longPressHandled = false;
      buttonPressStartMs = millis();
    } else {
      actionButtonDown = false;
      if (!longPressHandled) {
        encoderButtonPressCount++;
        shortPressCount++;
        shortPressDeadlineMs = millis() + BUTTON_MULTI_CLICK_MS;
        Serial.printf("short press count=%lu shortGroup=%u encSw=%u boot=%u\n",
                      encoderButtonPressCount,
                      shortPressCount,
                      encoderSwitchDown ? 1 : 0,
                      bootButtonDown ? 1 : 0);
      }
    }
  }

  if (actionButtonDown && !longPressHandled && millis() - buttonPressStartMs >= BUTTON_LONG_PRESS_MS) {
    killLatched = !killLatched;
    longPressHandled = true;
    shortPressCount = 0;
    shortPressDeadlineMs = 0;
    Serial.printf("long press kill=%u encSw=%u boot=%u\n",
                  killLatched,
                  encoderSwitchDown ? 1 : 0,
                  bootButtonDown ? 1 : 0);
  }

  if (!actionButtonDown && shortPressCount > 0 && millis() >= shortPressDeadlineMs) {
    if (shortPressCount == 1) {
      lightsEnabled = !lightsEnabled;
      Serial.printf("lights=%u via single press\n", lightsEnabled ? 1 : 0);
    } else if (shortPressCount >= 3) {
      calibrationMode = !calibrationMode;
      Serial.printf("calibration_mode=%u via %u short presses\n",
                    calibrationMode ? 1 : 0,
                    shortPressCount);
    }
    shortPressCount = 0;
    shortPressDeadlineMs = 0;
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

  // Always read throttle + steering at full rate (2 ADC reads ~2.3ms).
  packet.throttle = readAxis(THROTTLE_AXIS_CH);
  packet.steering = readAxis(STEERING_AXIS_CH);

  // Read aux channels at 1/5 rate (~20 Hz still, saves ~2.3ms on 80% of loops).
  if (++auxReadCounter >= 5) {
    auxReadCounter = 0;
    cachedAux1 = readAxis(AUX1_AXIS_CH);
    cachedAux2 = readAxis(AUX2_AXIS_CH);
  }
  packet.aux1 = cachedAux1;
  packet.aux2 = cachedAux2;

  packet.speedLimit = speedLimit;
  packet.mode = modeFromAxis(packet.aux2);
  packet.buttons = 0;
  if (killLatched) packet.buttons |= BTN_KILL;
  if (lightsEnabled) packet.buttons |= BTN_LIGHTS;
  if (calibrationMode) packet.buttons |= BTN_CALIBRATION;
  packet.crc = controlCrc(packet);
  return packet;
}

void drawScreen(const ControlPacket &packet) {
  if (millis() - lastScreenMs < 100) return;
  lastScreenMs = millis();

  // Read telemetry under mutex to avoid torn reads
  TelemetryPacket telCopy;
  bool telFresh;
  portENTER_CRITICAL(&telemetryMux);
  telCopy = latestTelemetry;
  telFresh = telemetryReady && (millis() - lastTelemetryRxMs < TELEMETRY_STALE_MS);
  portEXIT_CRITICAL(&telemetryMux);

  // Clear only the value regions instead of full screen to eliminate flicker.
  // Row layout: 0, 18, 32, 46, 66, 80, 94, 108 — each row is ~12px tall.
  tft.fillRect(0, 0, 128, 14, ST77XX_BLACK);     // header row
  tft.fillRect(0, 18, 128, 12, ST77XX_BLACK);    // THR
  tft.fillRect(0, 32, 128, 12, ST77XX_BLACK);    // STR
  tft.fillRect(0, 46, 128, 12, ST77XX_BLACK);    // SPD + mode
  tft.fillRect(0, 66, 128, 12, ST77XX_BLACK);    // drive/cal + TX status
  tft.fillRect(0, 80, 128, 12, ST77XX_BLACK);    // RX telemetry row 1
  tft.fillRect(0, 94, 128, 12, ST77XX_BLACK);    // RX telemetry row 2
  tft.fillRect(0, 108, 128, 20, ST77XX_BLACK);   // SW + lights row

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
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
  tft.print("SPD ");
  tft.print(packet.speedLimit);
  tft.print("/255");
  tft.setCursor(92, 46);
  tft.print("M");
  tft.print(packet.mode);

  tft.setCursor(0, 66);
  tft.setTextColor(calibrationMode ? ST77XX_CYAN : ST77XX_WHITE);
  tft.print(calibrationMode ? "CAL SAFE" : "DRIVE");

  tft.setCursor(72, 66);
  tft.setTextColor(lastSendOk ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.print(lastSendOk ? "TX OK" : "WAIT");

  if (telFresh) {
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("RX ");
    tft.print(telCopy.packetAgeMs);
    tft.print("ms");

    tft.setCursor(0, 94);
    tft.print("L ");
    tft.print(telCopy.leftMotor);
    tft.setCursor(64, 94);
    tft.print("R ");
    tft.print(telCopy.rightMotor);
  } else {
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_RED);
    tft.print("NO RX TEL");
  }

  tft.setTextColor(encoderSwitchDown ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setCursor(0, 108);
  tft.print("SW ");
  tft.print(encoderSwitchDown ? "DOWN " : "UP   ");
  tft.print(bootButtonDown ? " B" : " -");
  tft.print(encoderButtonPressCount);
  tft.setCursor(84, 108);
  tft.setTextColor(lightsEnabled ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print(lightsEnabled ? "LT" : "lt");
}

void setStatusPixel(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0, statusPixel.Color(r, g, b));
  statusPixel.show();
}

void updateStatusLed(const ControlPacket &packet) {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 80) return;
  lastUpdate = millis();

  bool telFresh;
  portENTER_CRITICAL(&telemetryMux);
  telFresh = telemetryReady && (millis() - lastTelemetryRxMs < TELEMETRY_STALE_MS);
  portEXIT_CRITICAL(&telemetryMux);

  if (packet.buttons & BTN_KILL) {
    uint8_t pulse = (millis() / 200) % 2 ? 80 : 8;
    setStatusPixel(pulse, 0, 0);
  } else if (!lastSendOk) {
    setStatusPixel(80, 45, 0);
  } else if (telFresh) {
    setStatusPixel(0, 70, 0);
  } else {
    setStatusPixel(0, 0, 60);
  }
}

void printStatus(const ControlPacket &packet) {
  if (millis() - lastSerialMs < 1000) return;
  lastSerialMs = millis();

  TelemetryPacket telCopy;
  bool telFresh;
  uint32_t telAgeMs;
  portENTER_CRITICAL(&telemetryMux);
  telCopy = latestTelemetry;
  telFresh = telemetryReady && (millis() - lastTelemetryRxMs < TELEMETRY_STALE_MS);
  telAgeMs = millis() - lastTelemetryRxMs;
  portEXIT_CRITICAL(&telemetryMux);

  Serial.printf("seq=%u thr=%d steer=%d limit=%u mode=%u kill=%u lights=%u tx=%s",
                packet.sequence, packet.throttle, packet.steering,
                packet.speedLimit, packet.mode, killLatched, lightsEnabled,
                lastSendOk ? "ok" : "pending/fail");
  Serial.printf(" cal=%u encSw=%u boot=%u swCount=%lu",
                calibrationMode ? 1 : 0,
                encoderSwitchDown ? 1 : 0,
                bootButtonDown ? 1 : 0,
                encoderButtonPressCount);
  if (telFresh) {
    Serial.printf(" rxAge=%ums left=%d right=%d batt=%umV",
                  telCopy.packetAgeMs,
                  telCopy.leftMotor,
                  telCopy.rightMotor,
                  telCopy.batteryMv);
  } else {
    Serial.printf(" telStale=%lums", telAgeMs);
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

  esp_now_set_pmk(ESPNOW_PMK);
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
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  statusPixel.begin();
  statusPixel.setBrightness(BUILTIN_RGB_BRIGHTNESS);
  setStatusPixel(0, 0, 20);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ADS1115 not found. Check SDA/SCL, power, and ADDR to GND.");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tft.initR(INITR_144GREENTAB);
  tft.setRotation(DISPLAY_ROTATION);
  tft.fillScreen(ST77XX_BLACK);

  setupEspNow();

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Receiver peer: ");
  printMac(RECEIVER_MAC);
  Serial.println(" (secure direct)");
  Serial.println("Long press encoder/BOOT toggles kill. Single press toggles lights.");
  Serial.println("Triple short press toggles calibration mode.");

  // Launch display task on Core 0 so SPI writes never block the control loop.
  xTaskCreatePinnedToCore(displayTask, "disp", 4096, NULL, 1, NULL, 0);
}

// Display + serial task on Core 0. Runs independently from control loop.
void displayTask(void *param) {
  (void)param;
  for (;;) {
    ControlPacket pkt;
    portENTER_CRITICAL(&displayMux);
    pkt = lastBuiltPacket;
    portEXIT_CRITICAL(&displayMux);

    drawScreen(pkt);
    printStatus(pkt);

    vTaskDelay(pdMS_TO_TICKS(50));  // ~20 fps check rate, drawScreen rate-limits to 10 fps
  }
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

  // Share latest packet with Core 0 display task (fast ~20 byte copy)
  portENTER_CRITICAL(&displayMux);
  lastBuiltPacket = packet;
  portEXIT_CRITICAL(&displayMux);

  // Status LED stays on Core 1 for immediate visual feedback
  updateStatusLed(packet);
}
