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

#define ESPNOW_CHANNEL 6
#define SEND_INTERVAL_MS 50
#define ACK_TIMEOUT_MS 250

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ADS1115_ADDR 0x48

#define SPI_SCK_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define DISPLAY_CS_PIN 15
#define DISPLAY_DC_PIN 16
#define DISPLAY_RST_PIN 17
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 128
#define DISPLAY_ROTATION 0

#define BUILTIN_RGB_PIN 48
#define BUILTIN_RGB_COUNT 1
#define BUILTIN_RGB_BRIGHTNESS 24

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6

#define TEST_MAGIC 0x52434144UL  // "RCAD"
#define TEST_VERSION 1

// COM4 receiver MAC from esptool read_mac: B4:3A:45:3F:A4:E8.
uint8_t RECEIVER_MAC[6] = {0xB4, 0x3A, 0x45, 0x3F, 0xA4, 0xE8};

// ESP-NOW encrypted unicast uses a 16-byte PMK and a 16-byte LMK.
// Change both keys before competition use and keep the same values on receiver.
const uint8_t ESPNOW_PMK[16] = {
  0x52, 0x43, 0x52, 0x41, 0x43, 0x45, 0x50, 0x4D,
  0x4B, 0x32, 0x30, 0x32, 0x36, 0x30, 0x34, 0x15
};

const uint8_t ESPNOW_LMK[16] = {
  0x42, 0x54, 0x53, 0x45, 0x53, 0x50, 0x4E, 0x4F,
  0x57, 0x52, 0x41, 0x57, 0x41, 0x44, 0x43, 0x31
};

struct __attribute__((packed)) RawAdcPacket {
  uint32_t magic;
  uint8_t version;
  uint16_t sequence;
  uint32_t txUptimeMs;
  int16_t adcRaw[4];
  uint8_t encoderBits;
  uint16_t crc;
};

struct __attribute__((packed)) AckPacket {
  uint32_t magic;
  uint8_t version;
  uint16_t sequenceAck;
  uint32_t rxUptimeMs;
  uint32_t rxPacketCount;
  int16_t echoedAdcRaw[4];
  uint16_t crc;
};

Adafruit_ADS1115 ads;
Adafruit_ST7735 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);
Adafruit_NeoPixel statusPixel(BUILTIN_RGB_COUNT, BUILTIN_RGB_PIN, NEO_GRB + NEO_KHZ800);

portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;
AckPacket latestAck = {};
bool ackValid = false;

volatile bool sendReady = true;
volatile bool lastSendOk = false;

uint16_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSerialMs = 0;
uint32_t sentPackets = 0;
uint32_t sendErrors = 0;
volatile uint32_t ackPackets = 0;
uint32_t ackTimeouts = 0;
volatile uint16_t waitingAckSequence = 0;
uint32_t waitingAckSinceMs = 0;
volatile bool waitingForAck = false;

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

uint16_t rawPacketCrc(const RawAdcPacket &packet) {
  RawAdcPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

uint16_t ackPacketCrc(const AckPacket &packet) {
  AckPacket copy = packet;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

uint8_t readEncoderBits() {
  uint8_t bits = 0;
  if (digitalRead(ENCODER_A_PIN) == LOW) bits |= 0x01;
  if (digitalRead(ENCODER_B_PIN) == LOW) bits |= 0x02;
  if (digitalRead(ENCODER_SW_PIN) == LOW) bits |= 0x04;
  return bits;
}

RawAdcPacket buildPacket() {
  RawAdcPacket packet = {};
  packet.magic = TEST_MAGIC;
  packet.version = TEST_VERSION;
  packet.sequence = ++sequenceNumber;
  packet.txUptimeMs = millis();
  for (uint8_t i = 0; i < 4; i++) {
    packet.adcRaw[i] = ads.readADC_SingleEnded(i);
  }
  packet.encoderBits = readEncoderBits();
  packet.crc = rawPacketCrc(packet);
  return packet;
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
void onSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendReady = true;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len != sizeof(AckPacket)) return;

  AckPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != TEST_MAGIC || packet.version != TEST_VERSION) return;
  if (ackPacketCrc(packet) != packet.crc) return;

  portENTER_CRITICAL(&ackMux);
  latestAck = packet;
  ackValid = true;
  portEXIT_CRITICAL(&ackMux);

  ackPackets++;
  if (waitingForAck && packet.sequenceAck == waitingAckSequence) {
    waitingForAck = false;
  }
}

void drawDisplay(const RawAdcPacket &packet) {
  if (millis() - lastDisplayMs < 100) return;
  lastDisplayMs = millis();

  AckPacket ackCopy;
  bool hasAck;
  portENTER_CRITICAL(&ackMux);
  ackCopy = latestAck;
  hasAck = ackValid;
  portEXIT_CRITICAL(&ackMux);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.print("RAW ADC SECURE");

  tft.setCursor(0, 16);
  tft.print("A0 ");
  tft.print(packet.adcRaw[0]);
  tft.setCursor(64, 16);
  tft.print("A1 ");
  tft.print(packet.adcRaw[1]);

  tft.setCursor(0, 28);
  tft.print("A2 ");
  tft.print(packet.adcRaw[2]);
  tft.setCursor(64, 28);
  tft.print("A3 ");
  tft.print(packet.adcRaw[3]);

  tft.setCursor(0, 46);
  tft.print("SEQ ");
  tft.print(packet.sequence);

  tft.setCursor(0, 58);
  tft.setTextColor(lastSendOk ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.print(lastSendOk ? "SEND OK" : "SEND WAIT");

  tft.setCursor(0, 70);
  tft.setTextColor(hasAck ? ST77XX_CYAN : ST77XX_RED);
  if (hasAck) {
    tft.print("ACK ");
    tft.print(ackCopy.sequenceAck);
    tft.print(" R");
    tft.print(ackCopy.rxPacketCount);
  } else {
    tft.print("NO ACK");
  }

  tft.setCursor(0, 82);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("ACKS ");
  tft.print(ackPackets);
  tft.print(" L");
  tft.print(ackTimeouts);

  tft.setCursor(0, 94);
  tft.print("ERR ");
  tft.print(sendErrors);
  tft.print(" ENC ");
  tft.print(packet.encoderBits, HEX);

  tft.setCursor(0, 112);
  tft.print("PEER A4:E8");
}

void printSerial(const RawAdcPacket &packet) {
  if (millis() - lastSerialMs < 250) return;
  lastSerialMs = millis();

  AckPacket ackCopy;
  bool hasAck;
  portENTER_CRITICAL(&ackMux);
  ackCopy = latestAck;
  hasAck = ackValid;
  portEXIT_CRITICAL(&ackMux);

  Serial.printf("seq=%u raw A0=%d A1=%d A2=%d A3=%d enc=0x%02X send=%s sent=%lu errors=%lu",
                packet.sequence,
                packet.adcRaw[0], packet.adcRaw[1], packet.adcRaw[2], packet.adcRaw[3],
                packet.encoderBits,
                lastSendOk ? "ok" : "pending/fail",
                sentPackets,
                sendErrors);
  if (hasAck) {
    Serial.printf(" ackSeq=%u rxPackets=%lu ackCount=%lu ackTimeouts=%lu echo=%d,%d,%d,%d",
                  ackCopy.sequenceAck,
                  ackCopy.rxPacketCount,
                  ackPackets,
                  ackTimeouts,
                  ackCopy.echoedAdcRaw[0], ackCopy.echoedAdcRaw[1],
                  ackCopy.echoedAdcRaw[2], ackCopy.echoedAdcRaw[3]);
  }
  Serial.println();
}

void setStatusPixel(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0, statusPixel.Color(r, g, b));
  statusPixel.show();
}

void updateStatusLed() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 80) return;
  lastUpdate = millis();

  if (!lastSendOk && sentPackets > 0) {
    setStatusPixel(80, 0, 0);
  } else if (waitingForAck) {
    setStatusPixel(80, 45, 0);
  } else if (ackValid && lastSendOk) {
    setStatusPixel(0, 70, 0);
  } else {
    setStatusPixel(0, 0, 60);
  }
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
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  if (!addPeer(RECEIVER_MAC)) {
    Serial.println("Encrypted receiver peer add failed");
    while (true) delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

  statusPixel.begin();
  statusPixel.setBrightness(BUILTIN_RGB_BRIGHTNESS);
  setStatusPixel(0, 0, 20);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ADS1115 not found. Check SDA/SCL and ADDR to GND.");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tft.initR(INITR_144GREENTAB);
  tft.setRotation(DISPLAY_ROTATION);
  tft.fillScreen(ST77XX_BLACK);

  setupEspNow();

  Serial.print("ESP-NOW raw ADC transmitter MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Secure unicast peer: B4:3A:45:3F:A4:E8");
  Serial.println("Sending uncalibrated ADS1115 A0..A3 raw values with encrypted bidirectional ACK.");
}

void loop() {
  static RawAdcPacket lastPacket = {};

  if (sendReady && millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    sendReady = false;
    lastPacket = buildPacket();

    esp_err_t result = esp_now_send(RECEIVER_MAC, (const uint8_t *)&lastPacket, sizeof(lastPacket));
    if (result == ESP_OK) {
      sentPackets++;
      waitingForAck = true;
      waitingAckSequence = lastPacket.sequence;
      waitingAckSinceMs = millis();
    } else {
      sendReady = true;
      lastSendOk = false;
      sendErrors++;
      Serial.printf("esp_now_send error: %d\n", result);
    }
  }

  if (waitingForAck && millis() - waitingAckSinceMs > ACK_TIMEOUT_MS) {
    waitingForAck = false;
    ackTimeouts++;
  }

  drawDisplay(lastPacket);
  updateStatusLed();
  printSerial(lastPacket);
}
