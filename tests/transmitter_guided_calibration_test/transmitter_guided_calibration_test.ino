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
#define BUILTIN_RGB_BRIGHTNESS 24

#define TEST_MAGIC 0x52434144UL
#define TEST_VERSION 1

uint8_t RECEIVER_MAC[6] = {0xB4, 0x3A, 0x45, 0x3F, 0xA4, 0xE8};

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

struct StepStats {
  const char *name;
  int16_t minRaw[4];
  int16_t maxRaw[4];
  int64_t sumRaw[4];
  uint32_t samples;
  int16_t encoderDelta;
  uint8_t lastEncoderBits;
};

struct StepDef {
  const char *name;
  const char *line1;
  const char *line2;
  bool switchCaptureOk;
};

const StepDef steps[] = {
  {"both_center_rest", "Both sticks", "center / release", true},
  {"both_full_up", "Both sticks", "full UP", true},
  {"both_full_down", "Both sticks", "full DOWN", true},
  {"both_full_left", "Both sticks", "full LEFT", true},
  {"both_full_right", "Both sticks", "full RIGHT", true},
  {"left_center_rest", "Left stick", "center only", true},
  {"left_full_up", "Left stick", "full UP", true},
  {"left_full_down", "Left stick", "full DOWN", true},
  {"left_full_left", "Left stick", "full LEFT", true},
  {"left_full_right", "Left stick", "full RIGHT", true},
  {"right_center_rest", "Right stick", "center only", true},
  {"right_full_up", "Right stick", "full UP", true},
  {"right_full_down", "Right stick", "full DOWN", true},
  {"right_full_left", "Right stick", "full LEFT", true},
  {"right_full_right", "Right stick", "full RIGHT", true},
  {"encoder_idle", "Encoder", "do not touch", true},
  {"encoder_one_right", "Encoder", "turn 1 click right", true},
  {"encoder_two_right", "Encoder", "turn 2 clicks right", true},
  {"encoder_many_right", "Encoder", "turn many right", true},
  {"encoder_one_left", "Encoder", "turn 1 click left", true},
  {"encoder_two_left", "Encoder", "turn 2 clicks left", true},
  {"encoder_many_left", "Encoder", "turn many left", true},
  {"encoder_button_hold", "Hold enc button", "Codex sends c", false}
};

const uint8_t STEP_COUNT = sizeof(steps) / sizeof(steps[0]);

Adafruit_ADS1115 ads;
Adafruit_ST7735 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);
Adafruit_NeoPixel statusPixel(BUILTIN_RGB_COUNT, BUILTIN_RGB_PIN, NEO_GRB + NEO_KHZ800);

StepStats stats[STEP_COUNT];

portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;
AckPacket latestAck = {};
bool ackValid = false;

uint8_t currentStep = 0;
bool finished = false;
uint16_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastDrawMs = 0;
uint32_t sentPackets = 0;
uint32_t sendErrors = 0;
uint32_t ackPackets = 0;
int16_t stepEncoderDelta = 0;

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

void resetStepStats(uint8_t index) {
  stats[index].name = steps[index].name;
  stats[index].samples = 0;
  stats[index].encoderDelta = 0;
  stats[index].lastEncoderBits = 0;
  for (uint8_t i = 0; i < 4; i++) {
    stats[index].minRaw[i] = 32767;
    stats[index].maxRaw[i] = -32768;
    stats[index].sumRaw[i] = 0;
  }
  stepEncoderDelta = 0;
}

void printStepInstruction() {
  Serial.print("STEP ");
  Serial.print(currentStep + 1);
  Serial.print("/");
  Serial.print(STEP_COUNT);
  Serial.print(": ");
  Serial.print(steps[currentStep].line1);
  Serial.print(" - ");
  Serial.println(steps[currentStep].line2);
  if (steps[currentStep].switchCaptureOk) {
    Serial.println("Hold steady, then press encoder switch or send serial 'c' to capture.");
  } else {
    Serial.println("Hold this state. Capture must be serial 'c' so the button value is measured cleanly.");
  }
  Serial.println("Send serial 'r' to reset only the current step samples.");
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
    stepEncoderDelta += digitalRead(ENCODER_B_PIN) == HIGH ? 1 : -1;
  }
  lastA = a;
}

bool capturePressed() {
  static bool lastButton = HIGH;
  static uint32_t lastChange = 0;
  bool button = digitalRead(ENCODER_SW_PIN);
  bool pressed = false;
  if (button != lastButton && millis() - lastChange > 60) {
    lastChange = millis();
    lastButton = button;
    if (button == LOW) pressed = true;
  }
  return pressed;
}

void addSample(const int16_t raw[4], uint8_t encoderBits) {
  StepStats &s = stats[currentStep];
  if (s.samples == 0) {
    for (uint8_t i = 0; i < 4; i++) {
      s.minRaw[i] = raw[i];
      s.maxRaw[i] = raw[i];
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    s.minRaw[i] = min(s.minRaw[i], raw[i]);
    s.maxRaw[i] = max(s.maxRaw[i], raw[i]);
    s.sumRaw[i] += raw[i];
  }
  s.samples++;
  s.encoderDelta = stepEncoderDelta;
  s.lastEncoderBits = encoderBits;
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

void drawPrompt(const int16_t raw[4]) {
  if (millis() - lastDrawMs < 100) return;
  lastDrawMs = millis();

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(0, 0);
  tft.print("CAL ");
  tft.print(currentStep + 1);
  tft.print("/");
  tft.print(STEP_COUNT);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 14);
  tft.print(steps[currentStep].line1);
  tft.setCursor(0, 26);
  tft.print(steps[currentStep].line2);

  tft.setCursor(0, 42);
  tft.print("A0 ");
  tft.print(raw[0]);
  tft.setCursor(64, 42);
  tft.print("A1 ");
  tft.print(raw[1]);

  tft.setCursor(0, 54);
  tft.print("A2 ");
  tft.print(raw[2]);
  tft.setCursor(64, 54);
  tft.print("A3 ");
  tft.print(raw[3]);

  tft.setCursor(0, 72);
  tft.print("Samples ");
  tft.print(stats[currentStep].samples);

  tft.setCursor(0, 86);
  tft.print("Enc ");
  tft.print(stepEncoderDelta);
  tft.print(" Ack ");
  tft.print(ackPackets);

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(0, 110);
  tft.print(steps[currentStep].switchCaptureOk ? "SW or serial c" : "serial c only");
}

void printLive(const int16_t raw[4], uint8_t encoderBits) {
  if (millis() - lastPrintMs < 250) return;
  lastPrintMs = millis();
  Serial.printf("step=%u/%u name=%s A0=%d A1=%d A2=%d A3=%d encBits=0x%02X encDelta=%d samples=%lu sent=%lu errors=%lu ack=%lu\n",
                currentStep + 1, STEP_COUNT, steps[currentStep].name,
                raw[0], raw[1], raw[2], raw[3],
                encoderBits, stepEncoderDelta, stats[currentStep].samples,
                sentPackets, sendErrors, ackPackets);
}

void printStepSummary(uint8_t index) {
  StepStats &s = stats[index];
  Serial.printf("CAPTURED %s samples=%lu encDelta=%d encBits=0x%02X\n",
                s.name, s.samples, s.encoderDelta, s.lastEncoderBits);
  for (uint8_t i = 0; i < 4; i++) {
    float avg = s.samples == 0 ? 0.0f : (float)s.sumRaw[i] / s.samples;
    Serial.printf("  A%u min=%d max=%d avg=%.1f span=%d\n",
                  i, s.minRaw[i], s.maxRaw[i], avg, s.maxRaw[i] - s.minRaw[i]);
  }
}

void printFinalJson() {
  Serial.println("CAL_JSON_BEGIN");
  Serial.println("{");
  Serial.println("  \"source\": \"transmitter_guided_calibration_test\",");
  Serial.println("  \"boardRole\": \"COM3 transmitter\",");
  Serial.println("  \"display\": {\"driver\": \"ST7735\", \"size\": \"128x128\", \"rotation\": 0},");
  Serial.println("  \"ads1115\": {\"address\": \"0x48\", \"sda\": 8, \"scl\": 9},");
  Serial.println("  \"steps\": [");
  for (uint8_t i = 0; i < STEP_COUNT; i++) {
    StepStats &s = stats[i];
    Serial.println("    {");
    Serial.printf("      \"name\": \"%s\",\n", s.name);
    Serial.printf("      \"samples\": %lu,\n", s.samples);
    Serial.printf("      \"encoderDelta\": %d,\n", s.encoderDelta);
    Serial.printf("      \"encoderBits\": %u,\n", s.lastEncoderBits);
    Serial.println("      \"channels\": [");
    for (uint8_t ch = 0; ch < 4; ch++) {
      float avg = s.samples == 0 ? 0.0f : (float)s.sumRaw[ch] / s.samples;
      Serial.printf("        {\"channel\": \"A%u\", \"min\": %d, \"max\": %d, \"avg\": %.1f, \"span\": %d}%s\n",
                    ch, s.minRaw[ch], s.maxRaw[ch], avg, s.maxRaw[ch] - s.minRaw[ch],
                    ch == 3 ? "" : ",");
    }
    Serial.println("      ]");
    Serial.print("    }");
    Serial.println(i == STEP_COUNT - 1 ? "" : ",");
  }
  Serial.println("  ]");
  Serial.println("}");
  Serial.println("CAL_JSON_END");
}

void finishStep() {
  printStepSummary(currentStep);
  currentStep++;
  if (currentStep >= STEP_COUNT) {
    finished = true;
    setStatusPixel(0, 70, 0);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.print("CAL DONE");
    tft.setCursor(0, 18);
    tft.print("Read Serial JSON");
    printFinalJson();
  } else {
    resetStepStats(currentStep);
    printStepInstruction();
  }
}

bool serialCaptureRequested() {
  bool requested = false;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      requested = true;
    } else if (c == 'r' || c == 'R') {
      resetStepStats(currentStep);
      Serial.print("RESET STEP: ");
      Serial.println(steps[currentStep].name);
      printStepInstruction();
    }
  }
  return requested;
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
  setStatusPixel(0, 0, 50);

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
  for (uint8_t i = 0; i < STEP_COUNT; i++) resetStepStats(i);

  Serial.println("Guided transmitter calibration started.");
  Serial.println("For best results, move to the requested state, send 'r', hold steady for 1-2 seconds, then send 'c'.");
  Serial.println("The encoder switch may capture normal steps, but serial 'c' is cleaner and required for button-hold.");
  printStepInstruction();
}

void loop() {
  if (finished) return;

  updateEncoderDelta();

  int16_t raw[4];
  for (uint8_t i = 0; i < 4; i++) {
    raw[i] = ads.readADC_SingleEnded(i);
  }
  uint8_t encoderBits = readEncoderBits();
  addSample(raw, encoderBits);

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    RawAdcPacket packet = buildPacket(raw, encoderBits);
    esp_err_t result = esp_now_send(RECEIVER_MAC, (const uint8_t *)&packet, sizeof(packet));
    if (result == ESP_OK) sentPackets++;
    else sendErrors++;
  }

  drawPrompt(raw);
  printLive(raw, encoderBits);

  bool switchCapture = steps[currentStep].switchCaptureOk && capturePressed();
  bool serialCapture = serialCaptureRequested();
  if (switchCapture || serialCapture) {
    setStatusPixel(0, 60, 0);
    finishStep();
    delay(350);
    if (!finished) setStatusPixel(0, 0, 50);
  }
}
