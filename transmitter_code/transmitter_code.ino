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
#include <Preferences.h>

/* ================= RADIO CONFIG ================= */
#define ESPNOW_CHANNEL 6
#define SEND_INTERVAL_MS 10
#define SEND_CALLBACK_TIMEOUT_MS 50
#define TELEMETRY_STALE_MS 500

// ESP-NOW peer MAC + PMK/LMK live in secrets.h (gitignored).
// Copy secrets.example.h -> secrets.h in this folder and set your values.
#include "secrets.h"

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
#define CALIBRATION_ENTRY_CLICKS 4
#define FULL_SPEED_LIMIT 255
#define CAL_CAPTURE_SAMPLES 80
#define CAL_CAPTURE_DELAY_MS 2
#define CAL_STORE_NS "txcal"
#define CAL_STORE_KEY "axis_v1"
#define CAL_STORE_MAGIC 0x54584341UL  // "TXCA"
#define CAL_STORE_VERSION 1

#define STEERING_AXIS_CH 0
#define THROTTLE_AXIS_CH 1
#define AUX1_AXIS_CH 2
#define AUX2_AXIS_CH 3
#define SWAP_STEERING_LEFT_RIGHT 0

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

enum CalStep : uint8_t {
  CAL_CENTER = 0,
  CAL_BOTH_UP = 1,
  CAL_BOTH_DOWN = 2,
  CAL_BOTH_LEFT = 3,
  CAL_BOTH_RIGHT = 4,
  CAL_DONE = 5
};

struct StoredAxisCal {
  int16_t minRaw;
  int16_t centerRaw;
  int16_t maxRaw;
  int16_t deadbandRaw;
  uint8_t invert;
  uint8_t reserved[3];
};

struct StoredCalibration {
  uint32_t magic;
  uint16_t version;
  uint16_t crc;
  StoredAxisCal axis[4];
};

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
  {4000, 21220, 32000, false, 500},  // A0 steering
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
bool killLatched = true;
bool lightsEnabled = true;
bool encoderSwitchDown = false;
bool bootButtonDown = false;
uint32_t encoderButtonPressCount = 0;
bool actionButtonDown = false;
bool longPressHandled = false;
uint32_t buttonPressStartMs = 0;
uint8_t shortPressCount = 0;
uint32_t shortPressDeadlineMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastScreenMs = 0;
uint32_t lastSerialMs = 0;

bool runtimeCalActive = false;
uint8_t runtimeCalStep = CAL_CENTER;
int16_t calSamples[CAL_DONE][4] = {};

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

uint16_t calibrationStorageCrc(const StoredCalibration &stored) {
  StoredCalibration copy = stored;
  copy.crc = 0;
  return crc16Ccitt((const uint8_t *)&copy, sizeof(copy));
}

StoredCalibration buildStoredCalibration() {
  StoredCalibration stored = {};
  stored.magic = CAL_STORE_MAGIC;
  stored.version = CAL_STORE_VERSION;
  for (uint8_t i = 0; i < 4; i++) {
    stored.axis[i].minRaw = axisCal[i].minRaw;
    stored.axis[i].centerRaw = axisCal[i].centerRaw;
    stored.axis[i].maxRaw = axisCal[i].maxRaw;
    stored.axis[i].deadbandRaw = axisCal[i].deadbandRaw;
    stored.axis[i].invert = axisCal[i].invert ? 1 : 0;
  }
  stored.crc = calibrationStorageCrc(stored);
  return stored;
}

void applyStoredCalibration(const StoredCalibration &stored) {
  for (uint8_t i = 0; i < 4; i++) {
    axisCal[i].minRaw = stored.axis[i].minRaw;
    axisCal[i].centerRaw = stored.axis[i].centerRaw;
    axisCal[i].maxRaw = stored.axis[i].maxRaw;
    axisCal[i].deadbandRaw = stored.axis[i].deadbandRaw;
    axisCal[i].invert = stored.axis[i].invert != 0;
  }
}

bool saveCalibrationToNvs() {
  StoredCalibration stored = buildStoredCalibration();
  Preferences prefs;
  if (!prefs.begin(CAL_STORE_NS, false)) {
    Serial.println("cal save failed: nvs open");
    return false;
  }
  size_t written = prefs.putBytes(CAL_STORE_KEY, &stored, sizeof(stored));
  prefs.end();
  bool ok = written == sizeof(stored);
  Serial.println(ok ? "cal saved to nvs" : "cal save failed: short write");
  return ok;
}

bool loadCalibrationFromNvs() {
  Preferences prefs;
  if (!prefs.begin(CAL_STORE_NS, true)) {
    Serial.println("cal load skipped: nvs open");
    return false;
  }

  StoredCalibration stored = {};
  size_t read = prefs.getBytes(CAL_STORE_KEY, &stored, sizeof(stored));
  prefs.end();
  if (read != sizeof(stored)) {
    Serial.println("cal load: no stored calibration");
    return false;
  }
  if (stored.magic != CAL_STORE_MAGIC || stored.version != CAL_STORE_VERSION) {
    Serial.println("cal load: header mismatch");
    return false;
  }
  if (calibrationStorageCrc(stored) != stored.crc) {
    Serial.println("cal load: crc mismatch");
    return false;
  }

  applyStoredCalibration(stored);
  Serial.println("cal loaded from nvs");
  return true;
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

const char *calStepText(uint8_t step) {
  switch (step) {
    case CAL_CENTER: return "Center all sticks";
    case CAL_BOTH_UP: return "Both sticks UP";
    case CAL_BOTH_DOWN: return "Both sticks DOWN";
    case CAL_BOTH_LEFT: return "Both sticks LEFT";
    case CAL_BOTH_RIGHT: return "Both sticks RIGHT";
    default: return "Done";
  }
}

void captureCalibrationStep(uint8_t step) {
  int32_t sum[4] = {0, 0, 0, 0};
  for (uint8_t n = 0; n < CAL_CAPTURE_SAMPLES; n++) {
    for (uint8_t ch = 0; ch < 4; ch++) {
      sum[ch] += ads.readADC_SingleEnded(ch);
    }
    delay(CAL_CAPTURE_DELAY_MS);
  }
  for (uint8_t ch = 0; ch < 4; ch++) {
    calSamples[step][ch] = (int16_t)(sum[ch] / CAL_CAPTURE_SAMPLES);
  }
}

void normalizeAxisBounds(AxisCal &cal) {
  if (cal.minRaw > cal.centerRaw - 200) cal.minRaw = cal.centerRaw - 200;
  if (cal.maxRaw < cal.centerRaw + 200) cal.maxRaw = cal.centerRaw + 200;
}

void applyRuntimeCalibration() {
  axisCal[STEERING_AXIS_CH].centerRaw = calSamples[CAL_CENTER][STEERING_AXIS_CH];
  axisCal[THROTTLE_AXIS_CH].centerRaw = calSamples[CAL_CENTER][THROTTLE_AXIS_CH];
  axisCal[AUX1_AXIS_CH].centerRaw = calSamples[CAL_CENTER][AUX1_AXIS_CH];
  axisCal[AUX2_AXIS_CH].centerRaw = calSamples[CAL_CENTER][AUX2_AXIS_CH];

  int16_t steerLeft = calSamples[CAL_BOTH_LEFT][STEERING_AXIS_CH];
  int16_t steerRight = calSamples[CAL_BOTH_RIGHT][STEERING_AXIS_CH];
  axisCal[STEERING_AXIS_CH].minRaw = min(steerLeft, steerRight);
  axisCal[STEERING_AXIS_CH].maxRaw = max(steerLeft, steerRight);
  axisCal[STEERING_AXIS_CH].invert = (steerRight < axisCal[STEERING_AXIS_CH].centerRaw);

  int16_t thrUp = calSamples[CAL_BOTH_UP][THROTTLE_AXIS_CH];
  int16_t thrDown = calSamples[CAL_BOTH_DOWN][THROTTLE_AXIS_CH];
  axisCal[THROTTLE_AXIS_CH].minRaw = min(thrUp, thrDown);
  axisCal[THROTTLE_AXIS_CH].maxRaw = max(thrUp, thrDown);
  axisCal[THROTTLE_AXIS_CH].invert = (thrUp < axisCal[THROTTLE_AXIS_CH].centerRaw);

  axisCal[AUX1_AXIS_CH].minRaw = min(calSamples[CAL_BOTH_LEFT][AUX1_AXIS_CH], calSamples[CAL_BOTH_RIGHT][AUX1_AXIS_CH]);
  axisCal[AUX1_AXIS_CH].maxRaw = max(calSamples[CAL_BOTH_LEFT][AUX1_AXIS_CH], calSamples[CAL_BOTH_RIGHT][AUX1_AXIS_CH]);
  axisCal[AUX2_AXIS_CH].minRaw = min(calSamples[CAL_BOTH_UP][AUX2_AXIS_CH], calSamples[CAL_BOTH_DOWN][AUX2_AXIS_CH]);
  axisCal[AUX2_AXIS_CH].maxRaw = max(calSamples[CAL_BOTH_UP][AUX2_AXIS_CH], calSamples[CAL_BOTH_DOWN][AUX2_AXIS_CH]);

  normalizeAxisBounds(axisCal[STEERING_AXIS_CH]);
  normalizeAxisBounds(axisCal[THROTTLE_AXIS_CH]);
  normalizeAxisBounds(axisCal[AUX1_AXIS_CH]);
  normalizeAxisBounds(axisCal[AUX2_AXIS_CH]);

  for (uint8_t i = 0; i < 4; i++) {
    filters[i] = AxisFilter();
  }

  saveCalibrationToNvs();

  Serial.printf("cal applied: steer ctr=%d min=%d max=%d inv=%u, thr ctr=%d min=%d max=%d inv=%u\n",
                axisCal[STEERING_AXIS_CH].centerRaw,
                axisCal[STEERING_AXIS_CH].minRaw,
                axisCal[STEERING_AXIS_CH].maxRaw,
                axisCal[STEERING_AXIS_CH].invert ? 1 : 0,
                axisCal[THROTTLE_AXIS_CH].centerRaw,
                axisCal[THROTTLE_AXIS_CH].minRaw,
                axisCal[THROTTLE_AXIS_CH].maxRaw,
                axisCal[THROTTLE_AXIS_CH].invert ? 1 : 0);
}

void startRuntimeCalibration() {
  runtimeCalActive = true;
  runtimeCalStep = CAL_CENTER;
  killLatched = true;
  shortPressCount = 0;
  shortPressDeadlineMs = 0;
  Serial.println("calibration start: press BOOT/encoder to capture each step");
}

void captureNextCalibrationStep() {
  if (!runtimeCalActive) return;
  if (runtimeCalStep >= CAL_DONE) return;

  captureCalibrationStep(runtimeCalStep);
  Serial.printf("cal step %u captured: %s\n", runtimeCalStep + 1, calStepText(runtimeCalStep));
  runtimeCalStep++;

  if (runtimeCalStep >= CAL_DONE) {
    applyRuntimeCalibration();
    runtimeCalActive = false;
    Serial.println("calibration complete");
  }
}

void updateEncoder() {
  static bool rawButtonState = false;
  static bool lastStableButton = false;
  static uint32_t lastBounceMs = 0;

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
        if (runtimeCalActive) {
          captureNextCalibrationStep();
        } else {
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
    } else if (shortPressCount >= CALIBRATION_ENTRY_CLICKS) {
      startRuntimeCalibration();
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
  int16_t steering = readAxis(STEERING_AXIS_CH);
#if SWAP_STEERING_LEFT_RIGHT
  steering = -steering;
#endif
  packet.steering = steering;

  // Read aux channels at 1/5 rate (~20 Hz still, saves ~2.3ms on 80% of loops).
  if (++auxReadCounter >= 5) {
    auxReadCounter = 0;
    cachedAux1 = readAxis(AUX1_AXIS_CH);
    cachedAux2 = readAxis(AUX2_AXIS_CH);
  }
  packet.aux1 = cachedAux1;
  packet.aux2 = cachedAux2;

  packet.speedLimit = FULL_SPEED_LIMIT;
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

  // Read telemetry under mutex to avoid torn reads
  TelemetryPacket telCopy;
  bool telFresh;
  portENTER_CRITICAL(&telemetryMux);
  telCopy = latestTelemetry;
  telFresh = telemetryReady && (millis() - lastTelemetryRxMs < TELEMETRY_STALE_MS);
  portEXIT_CRITICAL(&telemetryMux);

  if (runtimeCalActive) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(0, 0);
    tft.print("CALIBRATION");
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 16);
    tft.print("Step ");
    tft.print(runtimeCalStep + 1);
    tft.print("/");
    tft.print((uint8_t)CAL_DONE);
    tft.setCursor(0, 30);
    tft.print(calStepText(runtimeCalStep));
    tft.setCursor(0, 48);
    tft.print("Press BOOT/SW");
    tft.setCursor(0, 60);
    tft.print("to capture");
    tft.setCursor(0, 84);
    tft.print("THR ");
    tft.print(packet.throttle);
    tft.setCursor(64, 84);
    tft.print("STR ");
    tft.print(packet.steering);
    tft.setCursor(0, 96);
    tft.print("AUX1 ");
    tft.print(packet.aux1);
    tft.setCursor(64, 96);
    tft.print("AUX2 ");
    tft.print(packet.aux2);
    tft.setCursor(0, 114);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Hold position steady");
    return;
  }

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
  tft.print("SPD FULL");
  tft.setCursor(92, 46);
  tft.print("M");
  tft.print(packet.mode);

  tft.setCursor(0, 66);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("RACE");

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
  Serial.printf(" calRun=%u step=%u encSw=%u boot=%u swCount=%lu",
                runtimeCalActive ? 1 : 0,
                runtimeCalActive ? runtimeCalStep + 1 : 0,
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
  loadCalibrationFromNvs();

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
  Serial.println("Four short presses starts calibration wizard (center/up/down/left/right).");
  Serial.println("Race mode sends full speed limit; joystick tilt controls speed.");

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
  // Poll buttons/encoder every iteration so the UI stays responsive.
  updateEncoder();

  // Hold the last built packet so the display task + status LED always have
  // valid data between sends. ADC reads now happen only on the send cadence.
  static ControlPacket packet = {};

  uint32_t now = millis();

  // Send-callback watchdog: esp_now_send clears sendReady until onDataSent
  // fires. If that callback is ever lost under RF stress, don't latch the
  // send loop off forever — recover after a bounded wait.
  if (!sendReady && now - lastSendMs > SEND_CALLBACK_TIMEOUT_MS) {
    sendReady = true;
    lastSendOk = false;
  }

  if (sendReady && now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    // Build (and therefore read the ADS1115) only at the 100 Hz send rate,
    // not every loop iteration. This drops I2C/ADC work ~4x.
    packet = buildPacket();

    sendReady = false;
    esp_err_t result = esp_now_send(RECEIVER_MAC, (const uint8_t *)&packet, sizeof(packet));
    if (result != ESP_OK) {
      sendReady = true;
      lastSendOk = false;
    }

    // Share latest packet with Core 0 display task (fast ~20 byte copy)
    portENTER_CRITICAL(&displayMux);
    lastBuiltPacket = packet;
    portEXIT_CRITICAL(&displayMux);
  }

  // Status LED stays on Core 1 for immediate visual feedback (self rate-limited)
  updateStatusLed(packet);

  // Yield ~1 ms so the Core 1 IDLE task can run. Without this the loop spins
  // at 100% and the SoC never light-sleeps between sends, which kept the
  // transmitter warm. Still ~10 iterations per 10 ms send window, so encoder
  // and button polling stay responsive.
  vTaskDelay(1);
}
