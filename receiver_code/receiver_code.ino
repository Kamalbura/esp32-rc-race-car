#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_arduino_version.h>
#include <Adafruit_NeoPixel.h>

/* ================= RADIO CONFIG ================= */
#define ESPNOW_CHANNEL 6
#define FAILSAFE_MS 120
#define RC_PACKET_VERSION 1

// ESP-NOW peer MAC + PMK/LMK live in secrets.h (gitignored).
// Copy secrets.example.h -> secrets.h in this folder and set your values.
#include "secrets.h"

/* ================= BTS MOTOR PINS ================= */
struct BTS {
  uint8_t lpwm;
  uint8_t rpwm;
  uint8_t len;
  uint8_t ren;
  uint8_t chL;
  uint8_t chR;
};

// Same BTS connections from the previous working sketch.
BTS leftMotor  = {15, 16, 17, 18, 0, 1};
BTS rightMotor = { 9, 10, 11, 12, 2, 3};

const bool LEFT_MOTOR_INVERT = true;
const bool RIGHT_MOTOR_INVERT = false;

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_BITS 8
#define MOTOR_DEADBAND 0.03f
#define MOTOR_ACCEL_PWM_PER_SEC 650.0f
#define MOTOR_DECEL_PWM_PER_SEC 1000.0f
#define MOTOR_REVERSE_BRAKE_PWM_PER_SEC 1500.0f
#define CALIBRATION_SPEED_SCALE 0.5f

// Steering behavior:
// - More steering authority at low throttle for tighter low-speed turns.
// - Reduced steering authority at high throttle for stability.
// - Time-based slew for smooth turn engagement (less snap).
#define STEERING_GAIN_LOW_SPEED 1.10f
#define STEERING_GAIN_HIGH_SPEED 0.45f
#define STEERING_ENGAGE_RATE_PER_SEC 1.80f
#define STEERING_RELEASE_RATE_PER_SEC 3.50f
#define STRAIGHT_ASSIST_STEER_WINDOW 0.10f
#define STALL_GUARD_THROTTLE_MIN 0.30f
#define STALL_GUARD_MIN_PWM 55

/* ================= LIGHTS ================= */
#define PIXEL_PIN 8
#define NUM_PIXELS 16
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ================= BUZZER ================= */
#define BUZZER_PIN 37
#define BUZZER_ACTIVE_HIGH 1

/* ================= OPTIONAL BATTERY TELEMETRY ================= */
const int BATTERY_ADC_PIN = -1;       // Set to an ADC pin if you add a divider.
const float BATTERY_DIVIDER_RATIO = 1.0f;

#define BTN_KILL 0x01
#define BTN_LIGHTS 0x02
#define BTN_AUX1 0x04
#define BTN_CALIBRATION BTN_AUX1

struct __attribute__((packed)) ControlPacket {
  uint8_t version;
  uint16_t sequence;
  int16_t throttle;
  int16_t steering;
  uint16_t speedLimit;
  int16_t aux1;
  int16_t aux2;
  uint8_t mode;
  uint8_t buttons;
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

portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket latestPacket = {};
uint32_t lastPacketMs = 0;
bool packetValid = false;

uint8_t controllerMac[6] = {0};
bool controllerKnown = false;

int16_t lastLeftCommand = 0;
int16_t lastRightCommand = 0;
float currentLeftPwm = 0.0f;
float currentRightPwm = 0.0f;
uint32_t lastMotorRampMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSerialMs = 0;
volatile uint32_t telemetrySendErrors = 0;
float filteredSteering = 0.0f;
uint32_t lastSteeringFilterMs = 0;

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

void printMac(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

void buzzerWrite(bool on) {
  digitalWrite(BUZZER_PIN, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

void buzz(uint16_t onMs, uint16_t offMs = 0) {
  buzzerWrite(true);
  delay(onMs);
  buzzerWrite(false);
  if (offMs > 0) delay(offMs);
}

void startupPattern() {
  strip.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 80));
    strip.show();
    delay(20);
  }
  buzz(70, 60);
  buzz(70, 60);
  for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 80, 0));
  strip.show();
  buzz(140, 0);
  delay(120);
  strip.clear();
  strip.show();
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

void setupControllerPeer() {
  memcpy(controllerMac, TRANSMITTER_MAC, sizeof(controllerMac));
  controllerKnown = addPeer(controllerMac);
  Serial.print("Controller peer: ");
  printMac(controllerMac);
  Serial.println(controllerKnown ? " secure OK" : " secure add failed");
}

void pwmAttach(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, channel);
#else
  ledcSetup(channel, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void drive(BTS &motor, int speed, bool invert) {
  speed = constrain(speed, -255, 255);
  if (invert) speed = -speed;

  if (speed == 0) {
    pwmWrite(motor.lpwm, motor.chL, 0);
    pwmWrite(motor.rpwm, motor.chR, 0);
  } else if (speed > 0) {
    pwmWrite(motor.rpwm, motor.chR, 0);
    pwmWrite(motor.lpwm, motor.chL, speed);
  } else {
    pwmWrite(motor.lpwm, motor.chL, 0);
    pwmWrite(motor.rpwm, motor.chR, -speed);
  }
}

void stopMotors() {
  drive(leftMotor, 0, LEFT_MOTOR_INVERT);
  drive(rightMotor, 0, RIGHT_MOTOR_INVERT);
  currentLeftPwm = 0;
  currentRightPwm = 0;
  lastLeftCommand = 0;
  lastRightCommand = 0;
}

float rampPwm(float current, float target, float dt) {
  if (fabs(current) < 1.0f) current = 0;
  if (fabs(target) < 1.0f) target = 0;

  float rate = (fabs(target) > fabs(current)) ? MOTOR_ACCEL_PWM_PER_SEC : MOTOR_DECEL_PWM_PER_SEC;
  if (current != 0 && target != 0 && ((current > 0) != (target > 0))) {
    target = 0;
    rate = MOTOR_REVERSE_BRAKE_PWM_PER_SEC;
  }

  float maxStep = rate * dt;
  float diff = target - current;
  if (fabs(diff) <= maxStep) return target;
  return current + (diff > 0 ? maxStep : -maxStep);
}

void updateMotorOutputs(int targetLeft, int targetRight) {
  uint32_t now = millis();
  float dt = lastMotorRampMs == 0 ? 0.02f : (now - lastMotorRampMs) / 1000.0f;
  lastMotorRampMs = now;
  if (dt > 0.1f) dt = 0.02f;

  currentLeftPwm = rampPwm(currentLeftPwm, constrain(targetLeft, -255, 255), dt);
  currentRightPwm = rampPwm(currentRightPwm, constrain(targetRight, -255, 255), dt);

  lastLeftCommand = (int16_t)lroundf(currentLeftPwm);
  lastRightCommand = (int16_t)lroundf(currentRightPwm);
  drive(leftMotor, lastLeftCommand, LEFT_MOTOR_INVERT);
  drive(rightMotor, lastRightCommand, RIGHT_MOTOR_INVERT);
}

float applySteeringRateLimit(float targetSteering, float throttleAbs) {
  uint32_t now = millis();
  float dt = lastSteeringFilterMs == 0 ? 0.02f : (now - lastSteeringFilterMs) / 1000.0f;
  lastSteeringFilterMs = now;
  if (dt > 0.1f) dt = 0.02f;

  // Steering command growth is slower than release; high speed slows engagement a bit more.
  bool engaging = fabs(targetSteering) > fabs(filteredSteering);
  float engageRate = STEERING_ENGAGE_RATE_PER_SEC * (1.0f - 0.35f * throttleAbs);
  if (engageRate < 1.0f) engageRate = 1.0f;
  float rate = engaging ? engageRate : STEERING_RELEASE_RATE_PER_SEC;

  float maxStep = rate * dt;
  float delta = targetSteering - filteredSteering;
  if (fabs(delta) <= maxStep) {
    filteredSteering = targetSteering;
  } else {
    filteredSteering += (delta > 0.0f ? maxStep : -maxStep);
  }
  return filteredSteering;
}

void updateLights(float steering, bool linkOk, bool kill, bool lightsOn, bool calibrationMode) {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();

  const uint8_t frontArc[4] = {14, 15, 0, 1};
  const uint8_t rightArc[4] = {2, 3, 4, 5};
  const uint8_t rearArc[4] = {6, 7, 8, 9};
  const uint8_t leftArc[4] = {10, 11, 12, 13};

  strip.clear();
  if (!linkOk) {
    uint8_t r = (millis() / 120) % 2 ? 180 : 20;
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(r, 0, 0));
  } else if (kill) {
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(70, 0, 0));
  } else {
    if (lightsOn) {
      for (uint8_t i = 0; i < 4; i++) {
        strip.setPixelColor(frontArc[i], strip.Color(150, 150, 150));
        strip.setPixelColor(rearArc[i], strip.Color(60, 0, 0));
      }
    }
    if (calibrationMode) {
      uint8_t blue = (millis() / 180) % 2 ? 90 : 25;
      for (int i = 0; i < NUM_PIXELS; i++) {
        uint32_t existing = strip.getPixelColor(i);
        uint8_t r = (existing >> 16) & 0xFF;
        uint8_t g = (existing >> 8) & 0xFF;
        strip.setPixelColor(i, strip.Color(r / 2, g / 2, blue));
      }
    }
    if (steering > 0.15f) {
      for (uint8_t i = 0; i < 4; i++) strip.setPixelColor(rightArc[i], strip.Color(255, 60, 0));
    } else if (steering < -0.15f) {
      for (uint8_t i = 0; i < 4; i++) strip.setPixelColor(leftArc[i], strip.Color(255, 60, 0));
    }
  }
  strip.show();
}

uint16_t readBatteryMv() {
  if (BATTERY_ADC_PIN < 0) return 0;
  return (uint16_t)(analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_DIVIDER_RATIO);
}

void sendTelemetry(const ControlPacket &packet, bool linkOk) {
  if (!controllerKnown || millis() - lastTelemetryMs < 100) return;
  lastTelemetryMs = millis();

  TelemetryPacket telemetry = {};
  telemetry.version = RC_PACKET_VERSION;
  telemetry.sequenceAck = packet.sequence;
  telemetry.batteryMv = readBatteryMv();
  telemetry.leftMotor = lastLeftCommand;
  telemetry.rightMotor = lastRightCommand;
  telemetry.linkState = linkOk ? 1 : 0;
  telemetry.packetAgeMs = (uint16_t)min<uint32_t>(65535, millis() - lastPacketMs);
  telemetry.crc = telemetryCrc(telemetry);

  esp_now_send(controllerMac, (const uint8_t *)&telemetry, sizeof(telemetry));
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onControlRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onControlRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (!sameMac(mac, TRANSMITTER_MAC)) return;
  if (len != sizeof(ControlPacket)) return;

  ControlPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.version != RC_PACKET_VERSION) return;
  if (controlCrc(packet) != packet.crc) return;

  portENTER_CRITICAL(&packetMux);
  latestPacket = packet;
  lastPacketMs = millis();
  packetValid = true;
  portEXIT_CRITICAL(&packetMux);
}

#if ESP_IDF_VERSION_MAJOR > 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 5)
void onTelemetrySent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
#else
void onTelemetrySent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  if (status != ESP_NOW_SEND_SUCCESS) telemetrySendErrors++;
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
  esp_now_register_recv_cb(onControlRecv);
  esp_now_register_send_cb(onTelemetrySent);
}

void setupMotors() {
  uint8_t pins[] = {
    leftMotor.lpwm, leftMotor.rpwm, leftMotor.len, leftMotor.ren,
    rightMotor.lpwm, rightMotor.rpwm, rightMotor.len, rightMotor.ren
  };

  for (uint8_t pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  pwmAttach(leftMotor.lpwm, leftMotor.chL);
  pwmAttach(leftMotor.rpwm, leftMotor.chR);
  pwmAttach(rightMotor.lpwm, rightMotor.chL);
  pwmAttach(rightMotor.rpwm, rightMotor.chR);
  stopMotors();

  digitalWrite(leftMotor.len, HIGH);
  digitalWrite(leftMotor.ren, HIGH);
  digitalWrite(rightMotor.len, HIGH);
  digitalWrite(rightMotor.ren, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
  setupMotors();
  strip.begin();
  strip.setBrightness(100);
  strip.show();
  setupEspNow();
  setupControllerPeer();
  startupPattern();

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Locked transmitter MAC: ");
  printMac(TRANSMITTER_MAC);
  Serial.println();
  Serial.println("Waiting for ESP-NOW control packets...");
}

void loop() {
  ControlPacket packet;
  bool hasPacket;
  uint32_t ageMs;

  portENTER_CRITICAL(&packetMux);
  packet = latestPacket;
  hasPacket = packetValid;
  ageMs = millis() - lastPacketMs;
  portEXIT_CRITICAL(&packetMux);

  bool linkOk = hasPacket && ageMs <= FAILSAFE_MS;
  bool kill = !linkOk || (packet.buttons & BTN_KILL);
  bool lightsOn = packet.buttons & BTN_LIGHTS;
  bool calibrationMode = packet.buttons & BTN_CALIBRATION;
  uint16_t requestedSpeedLimit = packet.speedLimit;
  uint16_t effectiveSpeedLimit = requestedSpeedLimit;

  if (kill) {
    stopMotors();
    filteredSteering = 0.0f;
    lastSteeringFilterMs = 0;
  } else {
    float throttle = packet.throttle / 1000.0f;
    float steering = packet.steering / 1000.0f;
    float limit = constrain(packet.speedLimit / 255.0f, 0.0f, 1.0f);
    // In calibration mode, scale the full knob range into a safe half range.
    // This avoids the old behavior where speed saturated above mid-knob.
    if (calibrationMode) limit *= CALIBRATION_SPEED_SCALE;
    effectiveSpeedLimit = (uint16_t)lroundf(limit * 255.0f);

    if (fabs(throttle) < MOTOR_DEADBAND) throttle = 0;
    if (fabs(steering) < MOTOR_DEADBAND) steering = 0;

    // Speed-aware steering: sharp at low speed, stable at high speed.
    float throttleAbs = fabs(throttle);
    float speedFactor = throttleAbs * throttleAbs;  // smooth nonlinear blend
    float steerGain = STEERING_GAIN_LOW_SPEED +
                      (STEERING_GAIN_HIGH_SPEED - STEERING_GAIN_LOW_SPEED) * speedFactor;
    float steerTarget = constrain(steering * steerGain, -1.0f, 1.0f);
    steering = applySteeringRateLimit(steerTarget, throttleAbs);

    // Keep straight reverse/forward stable when stick is near-center on steering.
    // This avoids one-motor crawl caused by small steering bias + wheel stiction.
    bool straightAssist = (throttleAbs >= STALL_GUARD_THROTTLE_MIN) &&
                          (fabs(steering) <= STRAIGHT_ASSIST_STEER_WINDOW);
    if (straightAssist) {
      steering = 0.0f;
    }

    float left = throttle + steering;
    float right = throttle - steering;
    float maxMag = max(fabs(left), fabs(right));
    if (maxMag > 1.0f) {
      left /= maxMag;
      right /= maxMag;
    }

    int targetLeft = (int)(left * limit * 255.0f);
    int targetRight = (int)(right * limit * 255.0f);
    if (straightAssist) {
      int minPwm = (int)lroundf(STALL_GUARD_MIN_PWM * limit);
      if (minPwm < 20) minPwm = 20;
      if (abs(targetLeft) < minPwm) targetLeft = (targetLeft >= 0) ? minPwm : -minPwm;
      if (abs(targetRight) < minPwm) targetRight = (targetRight >= 0) ? minPwm : -minPwm;
    }
    updateMotorOutputs(targetLeft, targetRight);
  }

  updateLights(packet.steering / 1000.0f, linkOk, kill, lightsOn, calibrationMode);
  sendTelemetry(packet, linkOk);

  if (millis() - lastSerialMs > 500) {
    lastSerialMs = millis();
    Serial.printf("link=%u age=%lu kill=%u cal=%u thr=%d steer=%d left=%d right=%d spdReq=%u spdEff=%u mode=%u telErr=%lu\n",
                  linkOk, ageMs, kill, calibrationMode ? 1 : 0, packet.throttle, packet.steering,
                  lastLeftCommand, lastRightCommand, requestedSpeedLimit, effectiveSpeedLimit, packet.mode,
                  telemetrySendErrors);
  }
}
