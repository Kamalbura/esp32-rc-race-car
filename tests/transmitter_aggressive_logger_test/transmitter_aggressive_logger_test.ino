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
#define ESPNOW_SEND_INTERVAL_MS 50
#define SERIAL_SAMPLE_INTERVAL_MS 10
#define DISPLAY_INTERVAL_MS 150

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ADS1115_ADDR 0x48

#define SPI_SCK_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define DISPLAY_CS_PIN 15
#define DISPLAY_DC_PIN 16
#define DISPLAY_RST_PIN 17
#define DISPLAY_ROTATION 0

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6

#define BUILTIN_RGB_PIN 48
#define BUILTIN_RGB_COUNT 1
#define BUILTIN_RGB_BRIGHTNESS 18

#define TEST_MAGIC 0x52434144UL
#define TEST_VERSION 1

#include "secrets.h"


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

uint16_t sequenceNumber = 0;
uint32_t lastEspNowMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t sentPackets = 0;
uint32_t sendErrors = 0;
uint32_t ackPackets = 0;
int16_t encoderDelta = 0;
bool lastSendOk = false;

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

void setStatusPixel(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0, statusPixel.Color(r, g, b));
  statusPixel.show();
}

uint8_t readEncoderBits() {
  uint8_t bits = 0;
  if (digitalRead(ENCODER_A_PIN) == LOW) bits |= 0x01;
  if (digitalRead(ENCODER_B_PIN) == LOW) bits |= 0x02;
  if (digitalRead(ENCODER_SW_PIN) == LOW) bits |= 0x04;
  return bits;
}

void updateEncoderDelta() {
  static int lastA = HIGH;
  int a = digitalRead(ENCODER_A_PIN);
  if (a != lastA && a == LOW) {
    encoderDelta += digitalRead(ENCODER_B_PIN) == HIGH ? 1 : -1;
  }
  lastA = a;
}

void readRaw(int16_t raw[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    raw[i] = ads.readADC_SingleEnded(i);
  }
}

RawAdcPacket buildPacket(const int16_t raw[4], uint8_t encoderBits) {
  RawAdcPacket packet = {};
  packet.magic = TEST_MAGIC;
  packet.version = TEST_VERSION;
  packet.sequence = ++sequenceNumber;
  packet.txUptimeMs = millis();
  for (uint8_t i = 0; i < 4; i++) {
    packet.adcRaw[i] = raw[i];
  }
  packet.encoderBits = encoderBits;
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
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR,ESP_NOW_INIT_FAILED");
    while (true) delay(1000);
  }
  esp_now_set_pmk(ESPNOW_PMK);
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  if (!addPeer(RECEIVER_MAC)) {
    Serial.println("ERR,ENCRYPTED_RECEIVER_PEER_FAILED");
    while (true) delay(1000);
  }
}

void drawDisplay(const int16_t raw[4], uint8_t encoderBits) {
  if (millis() - lastDisplayMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayMs = millis();

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(0, 0);
  tft.print("AGG TX LOGGER");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 18);
  tft.print("A0 ");
  tft.print(raw[0]);
  tft.setCursor(64, 18);
  tft.print("A1 ");
  tft.print(raw[1]);

  tft.setCursor(0, 32);
  tft.print("A2 ");
  tft.print(raw[2]);
  tft.setCursor(64, 32);
  tft.print("A3 ");
  tft.print(raw[3]);

  tft.setCursor(0, 52);
  tft.print("ENC ");
  tft.print(encoderBits, HEX);
  tft.print(" D ");
  tft.print(encoderDelta);

  tft.setCursor(0, 70);
  tft.setTextColor(lastSendOk ? ST77XX_GREEN : ST77XX_RED);
  tft.print(lastSendOk ? "ESPNOW OK" : "ESPNOW WAIT");

  tft.setTextColor(ackValid ? ST77XX_CYAN : ST77XX_YELLOW);
  tft.setCursor(0, 86);
  tft.print("ACK ");
  tft.print(ackPackets);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 108);
  tft.print("USB logs DATA");
}

void printData(uint32_t nowMs, const int16_t raw[4], uint8_t encoderBits) {
  if (nowMs - lastSerialMs < SERIAL_SAMPLE_INTERVAL_MS) return;
  lastSerialMs = nowMs;

  Serial.printf("DATA,%lu,%u,%d,%d,%d,%d,%u,%d,%lu,%lu,%lu\n",
                nowMs, sequenceNumber,
                raw[0], raw[1], raw[2], raw[3],
                encoderBits, encoderDelta,
                sentPackets, sendErrors, ackPackets);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

  statusPixel.begin();
  statusPixel.setBrightness(BUILTIN_RGB_BRIGHTNESS);
  setStatusPixel(0, 0, 40);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ERR,ADS1115_NOT_FOUND");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tft.initR(INITR_144GREENTAB);
  tft.setRotation(DISPLAY_ROTATION);
  tft.fillScreen(ST77XX_BLACK);

  setupEspNow();

  Serial.println("AGG_TX_READY");
  Serial.println("CSV,device_ms,seq,A0,A1,A2,A3,enc_bits,enc_delta,sent,errors,ack");
  Serial.print("TX_MAC,");
  Serial.println(WiFi.macAddress());
}

void loop() {
  updateEncoderDelta();

  int16_t raw[4];
  readRaw(raw);
  uint8_t encoderBits = readEncoderBits();
  uint32_t nowMs = millis();

  if (nowMs - lastEspNowMs >= ESPNOW_SEND_INTERVAL_MS) {
    lastEspNowMs = nowMs;
    RawAdcPacket packet = buildPacket(raw, encoderBits);
    esp_err_t result = esp_now_send(RECEIVER_MAC, (const uint8_t *)&packet, sizeof(packet));
    if (result == ESP_OK) sentPackets++;
    else sendErrors++;
  }

  printData(nowMs, raw, encoderBits);
  drawDisplay(raw, encoderBits);

  if (lastSendOk && ackValid) setStatusPixel(0, 45, 0);
  else if (sendErrors > 0) setStatusPixel(60, 0, 0);
  else setStatusPixel(0, 0, 45);
}
