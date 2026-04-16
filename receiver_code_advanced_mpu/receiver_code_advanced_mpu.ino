#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

/* ================= RADIO CONFIG ================= */
#define ESPNOW_CHANNEL 6
#define FAILSAFE_MS 120
#define RC_PACKET_VERSION 1

uint8_t TRANSMITTER_MAC[6] = {0xB4, 0x3A, 0x45, 0x3F, 0x46, 0xBC};

const uint8_t ESPNOW_PMK[16] = {
  0x52, 0x43, 0x52, 0x41, 0x43, 0x45, 0x50, 0x4D,
  0x4B, 0x32, 0x30, 0x32, 0x36, 0x30, 0x34, 0x15
};

const uint8_t ESPNOW_LMK[16] = {
  0x42, 0x54, 0x53, 0x45, 0x53, 0x50, 0x4E, 0x4F,
  0x57, 0x52, 0x41, 0x57, 0x41, 0x44, 0x43, 0x31
};

/* ================= MPU6050 CONFIG ================= */
#define MPU_SDA_PIN 1
#define MPU_SCL_PIN 2
#define ENABLE_YAW_ASSIST 0
#define MAX_TARGET_YAW_DPS 220.0f
#define YAW_ASSIST_GAIN 0.003f
#define TURN_TEST_MIN_PWM 90
#define TURN_TEST_MAX_PWM 170
#define TURN_TEST_TIMEOUT_MS 5000
#define TURN_TEST_TOLERANCE_DEG 4.0f
#define TURN_TEST_SETTLE_DPS 18.0f

Adafruit_MPU6050 mpu;
bool mpuReady = false;
float gyroZOffsetDps = 0.0f;
float yawRateDps = 0.0f;

/* ================= BTS MOTOR PINS ================= */
struct BTS {
  uint8_t lpwm;
  uint8_t rpwm;
  uint8_t len;
  uint8_t ren;
  uint8_t chL;
  uint8_t chR;
};

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

/* ================= LIGHTS ================= */
#define PIXEL_PIN 8
#define NUM_PIXELS 16
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ================= BUZZER ================= */
#define BUZZER_PIN 37
#define BUZZER_ACTIVE_HIGH 1

const int BATTERY_ADC_PIN = -1;
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
uint32_t lastTelemetryMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastMpuMs = 0;
uint32_t lastMotorRampMs = 0;
bool turnTestActive = false;
volatile uint32_t telemetrySendErrors = 0;

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

void startupPattern(bool mpuOk) {
  strip.clear();
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 80));
    strip.show();
    delay(20);
  }
  buzz(70, 60);
  buzz(70, 60);
  if (mpuOk) {
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 80, 0));
    strip.show();
    buzz(140, 0);
  } else {
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(80, 40, 0));
    strip.show();
    buzz(60, 40);
    buzz(60, 0);
  }
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

void setupMpu() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found. Advanced receiver will run without yaw assist.");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("Keep car still: calibrating gyro Z...");
  float sum = 0.0f;
  for (int i = 0; i < 300; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z * 57.2957795f;
    delay(5);
  }
  gyroZOffsetDps = sum / 300.0f;
  mpuReady = true;
  Serial.printf("MPU6050 ready. Gyro Z offset %.2f dps\n", gyroZOffsetDps);
}

void updateMpu() {
  if (!mpuReady || millis() - lastMpuMs < 5) return;
  lastMpuMs = millis();

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  yawRateDps = gyro.gyro.z * 57.2957795f - gyroZOffsetDps;
}

void recalibrateGyro() {
  if (!mpuReady) return;
  stopMotors();
  delay(150);
  Serial.println("Recalibrating gyro. Keep car still.");
  float sum = 0.0f;
  for (int i = 0; i < 250; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z * 57.2957795f;
    delay(4);
  }
  gyroZOffsetDps = sum / 250.0f;
  yawRateDps = 0.0f;
  Serial.printf("New gyro Z offset %.2f dps\n", gyroZOffsetDps);
  buzz(50, 50);
  buzz(120, 0);
}

bool executeTurnDegrees(float targetDeg) {
  if (!mpuReady) {
    Serial.println("Turn test unavailable: MPU6050 not ready.");
    buzz(50, 50);
    buzz(50, 0);
    return false;
  }

  stopMotors();
  delay(120);
  updateMpu();
  turnTestActive = true;
  float integratedYawDeg = 0.0f;
  uint32_t startMs = millis();
  uint32_t prevMs = startMs;

  Serial.printf("Turn test start: target %.1f deg\n", targetDeg);
  buzz(70, 40);

  while (millis() - startMs < TURN_TEST_TIMEOUT_MS) {
    updateMpu();
    uint32_t now = millis();
    float dt = (now - prevMs) / 1000.0f;
    if (dt > 0.001f) {
      integratedYawDeg += yawRateDps * dt;
      prevMs = now;
    }

    float remaining = targetDeg - integratedYawDeg;
    if (fabs(remaining) <= TURN_TEST_TOLERANCE_DEG && fabs(yawRateDps) <= TURN_TEST_SETTLE_DPS) {
      stopMotors();
      turnTestActive = false;
      Serial.printf("Turn complete: yaw=%.1f deg\n", integratedYawDeg);
      buzz(120, 0);
      return true;
    }

    float mag = constrain(fabs(remaining) / 90.0f, 0.0f, 1.0f);
    int pwm = (int)lroundf(TURN_TEST_MIN_PWM + (TURN_TEST_MAX_PWM - TURN_TEST_MIN_PWM) * mag);
    int signedPwm = remaining > 0 ? pwm : -pwm;
    updateMotorOutputs(-signedPwm, signedPwm);
    delay(5);
  }

  stopMotors();
  turnTestActive = false;
  Serial.printf("Turn timeout: yaw=%.1f deg target=%.1f deg\n", integratedYawDeg, targetDeg);
  buzz(50, 50);
  buzz(50, 50);
  buzz(50, 0);
  return false;
}

void printSerialHelp() {
  Serial.println("Receiver serial commands:");
  Serial.println("  h = help");
  Serial.println("  b = buzzer test");
  Serial.println("  c = recalibrate gyro (car still)");
  Serial.println("  i = print MPU idle snapshot");
  Serial.println("  l = turn left 90 deg");
  Serial.println("  p = RGB ring test");
  Serial.println("  r = turn right 90 deg");
  Serial.println("  s = stop motors");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = (char)tolower(Serial.read());
    switch (cmd) {
      case 'h':
        printSerialHelp();
        break;
      case 'b':
        buzz(80, 60);
        buzz(120, 0);
        break;
      case 'c':
        recalibrateGyro();
        break;
      case 'i':
        Serial.printf("mpuReady=%u gyroZOffset=%.2f yawRate=%.2f dps\n",
                      mpuReady ? 1 : 0,
                      gyroZOffsetDps,
                      yawRateDps);
        break;
      case 'l':
        executeTurnDegrees(90.0f);
        break;
      case 'p':
        strip.clear();
        strip.setPixelColor(14, strip.Color(180, 180, 180));
        strip.setPixelColor(15, strip.Color(180, 180, 180));
        strip.setPixelColor(0, strip.Color(180, 180, 180));
        strip.setPixelColor(1, strip.Color(180, 180, 180));
        strip.show();
        delay(250);
        strip.clear();
        strip.setPixelColor(2, strip.Color(255, 90, 0));
        strip.setPixelColor(3, strip.Color(255, 90, 0));
        strip.setPixelColor(4, strip.Color(255, 90, 0));
        strip.setPixelColor(5, strip.Color(255, 90, 0));
        strip.show();
        delay(250);
        strip.clear();
        strip.setPixelColor(6, strip.Color(180, 0, 0));
        strip.setPixelColor(7, strip.Color(180, 0, 0));
        strip.setPixelColor(8, strip.Color(180, 0, 0));
        strip.setPixelColor(9, strip.Color(180, 0, 0));
        strip.show();
        delay(250);
        strip.clear();
        strip.setPixelColor(10, strip.Color(255, 90, 0));
        strip.setPixelColor(11, strip.Color(255, 90, 0));
        strip.setPixelColor(12, strip.Color(255, 90, 0));
        strip.setPixelColor(13, strip.Color(255, 90, 0));
        strip.show();
        delay(250);
        strip.clear();
        strip.show();
        break;
      case 'r':
        executeTurnDegrees(-90.0f);
        break;
      case 's':
        stopMotors();
        Serial.println("Motors stopped.");
        break;
      default:
        break;
    }
  }
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
  setupMpu();
  setupEspNow();
  setupControllerPeer();
  startupPattern(mpuReady);

  Serial.print("Advanced receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Locked transmitter MAC: ");
  printMac(TRANSMITTER_MAC);
  Serial.println();
  Serial.println("MPU yaw axis: +Z up, positive gyro Z = left turn.");
  Serial.println("Board mounting now uses +Y to the front and +X sideways.");
  Serial.println("Waiting for ESP-NOW control packets...");
  printSerialHelp();
}

void loop() {
  handleSerialCommands();
  updateMpu();

  ControlPacket packet;
  bool hasPacket;
  uint32_t ageMs;

  portENTER_CRITICAL(&packetMux);
  packet = latestPacket;
  hasPacket = packetValid;
  ageMs = millis() - lastPacketMs;
  portEXIT_CRITICAL(&packetMux);

  bool linkOk = hasPacket && ageMs <= FAILSAFE_MS;
  bool kill = turnTestActive || !linkOk || (packet.buttons & BTN_KILL);
  bool lightsOn = packet.buttons & BTN_LIGHTS;
  bool calibrationMode = packet.buttons & BTN_CALIBRATION;
  uint16_t requestedSpeedLimit = packet.speedLimit;
  uint16_t effectiveSpeedLimit = requestedSpeedLimit;

  if (kill) {
    stopMotors();
  } else {
    float throttle = packet.throttle / 1000.0f;
    float steering = packet.steering / 1000.0f;
    float limit = constrain(packet.speedLimit / 255.0f, 0.0f, 1.0f);
    if (calibrationMode) limit = min(limit, 0.5f);
    effectiveSpeedLimit = (uint16_t)lroundf(limit * 255.0f);

    if (fabs(throttle) < MOTOR_DEADBAND) throttle = 0;
    if (fabs(steering) < MOTOR_DEADBAND) steering = 0;

#if ENABLE_YAW_ASSIST
    if (mpuReady && fabs(throttle) > 0.10f && fabs(steering) > 0.05f) {
      float targetYawDps = steering * MAX_TARGET_YAW_DPS;
      float yawError = targetYawDps - yawRateDps;
      steering += constrain(yawError * YAW_ASSIST_GAIN, -0.20f, 0.20f);
      steering = constrain(steering, -1.0f, 1.0f);
    }
#endif

    float left = throttle + steering;
    float right = throttle - steering;
    float maxMag = max(fabs(left), fabs(right));
    if (maxMag > 1.0f) {
      left /= maxMag;
      right /= maxMag;
    }

    int targetLeft = (int)(left * limit * 255.0f);
    int targetRight = (int)(right * limit * 255.0f);
    updateMotorOutputs(targetLeft, targetRight);
  }

  updateLights(packet.steering / 1000.0f, linkOk, kill, lightsOn, calibrationMode);
  sendTelemetry(packet, linkOk);

  if (millis() - lastSerialMs > 500) {
    lastSerialMs = millis();
    Serial.printf("link=%u age=%lu kill=%u cal=%u thr=%d steer=%d spdReq=%u spdEff=%u left=%d right=%d yaw=%.1f assist=%u telErr=%lu\n",
                  linkOk, ageMs, kill, calibrationMode ? 1 : 0,
                  packet.throttle, packet.steering, requestedSpeedLimit, effectiveSpeedLimit,
                  lastLeftCommand, lastRightCommand, yawRateDps, ENABLE_YAW_ASSIST,
                  telemetrySendErrors);
  }
}
