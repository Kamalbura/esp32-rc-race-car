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

/* ================= MPU6050 CONFIG ================= */
#define MPU_SDA_PIN 1
#define MPU_SCL_PIN 2
#define ENABLE_YAW_ASSIST 0
#define MAX_TARGET_YAW_DPS 220.0f
#define YAW_ASSIST_GAIN 0.003f

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
BTS rightMotor = { 9, 36, 11, 12, 2, 3};

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_BITS 8
#define MOTOR_DEADBAND 0.03f

/* ================= LIGHTS ================= */
#define PIXEL_PIN 8
#define NUM_PIXELS 48
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int BATTERY_ADC_PIN = -1;
const float BATTERY_DIVIDER_RATIO = 1.0f;

#define BTN_KILL 0x01
#define BTN_LIGHTS 0x02
#define BTN_AUX1 0x04

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
uint8_t pendingControllerMac[6] = {0};
bool controllerKnown = false;
volatile bool pendingPeer = false;

int16_t lastLeftCommand = 0;
int16_t lastRightCommand = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastMpuMs = 0;

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

void rememberControllerIfNeeded() {
  if (!pendingPeer) return;

  uint8_t mac[6];
  portENTER_CRITICAL(&packetMux);
  memcpy(mac, pendingControllerMac, 6);
  pendingPeer = false;
  portEXIT_CRITICAL(&packetMux);

  if (!controllerKnown || !sameMac(mac, controllerMac)) {
    memcpy(controllerMac, mac, 6);
    controllerKnown = addPeer(controllerMac);
    Serial.print("Controller learned: ");
    printMac(controllerMac);
    Serial.println(controllerKnown ? " peer OK" : " peer add failed");
  }
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

void drive(BTS &motor, int speed) {
  speed = constrain(speed, -255, 255);

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
  drive(leftMotor, 0);
  drive(rightMotor, 0);
  lastLeftCommand = 0;
  lastRightCommand = 0;
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

void updateLights(float steering, bool linkOk, bool kill, bool lightsOn) {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();

  strip.clear();
  if (!linkOk) {
    uint8_t r = (millis() / 120) % 2 ? 180 : 20;
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(r, 0, 0));
  } else if (kill) {
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(70, 0, 0));
  } else if (lightsOn) {
    for (int i = 16; i < 32; i++) strip.setPixelColor(i, strip.Color(150, 150, 150));
    if (steering > 0.15f) {
      for (int i = 0; i < 16; i++) strip.setPixelColor(i, strip.Color(255, 60, 0));
    } else if (steering < -0.15f) {
      for (int i = 32; i < 48; i++) strip.setPixelColor(i, strip.Color(255, 60, 0));
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
  if (len != sizeof(ControlPacket)) return;

  ControlPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.version != RC_PACKET_VERSION) return;
  if (controlCrc(packet) != packet.crc) return;

  portENTER_CRITICAL(&packetMux);
  latestPacket = packet;
  lastPacketMs = millis();
  packetValid = true;
  memcpy(pendingControllerMac, mac, 6);
  pendingPeer = true;
  portEXIT_CRITICAL(&packetMux);
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

  esp_now_register_recv_cb(onControlRecv);
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

  setupMotors();
  strip.begin();
  strip.setBrightness(100);
  strip.show();
  setupMpu();
  setupEspNow();

  Serial.print("Advanced receiver MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Waiting for ESP-NOW control packets...");
}

void loop() {
  rememberControllerIfNeeded();
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
  bool kill = !linkOk || (packet.buttons & BTN_KILL);
  bool lightsOn = packet.buttons & BTN_LIGHTS;

  if (kill) {
    stopMotors();
  } else {
    float throttle = packet.throttle / 1000.0f;
    float steering = packet.steering / 1000.0f;
    float limit = constrain(packet.speedLimit / 1000.0f, 0.0f, 1.0f);

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

    lastLeftCommand = (int16_t)(left * limit * 255.0f);
    lastRightCommand = (int16_t)(right * limit * 255.0f);
    drive(leftMotor, lastLeftCommand);
    drive(rightMotor, lastRightCommand);
  }

  updateLights(packet.steering / 1000.0f, linkOk, kill, lightsOn);
  sendTelemetry(packet, linkOk);

  if (millis() - lastSerialMs > 500) {
    lastSerialMs = millis();
    Serial.printf("link=%u age=%lu kill=%u thr=%d steer=%d left=%d right=%d yaw=%.1f assist=%u\n",
                  linkOk, ageMs, kill, packet.throttle, packet.steering,
                  lastLeftCommand, lastRightCommand, yawRateDps, ENABLE_YAW_ASSIST);
  }
}
