#include <Arduino.h>
#include <esp_arduino_version.h>

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

int testSpeed = 80;
bool autoRamp = false;

void printPinout() {
  Serial.println("Receiver BTS pinout:");
  Serial.printf("  Left  LPWM=%u RPWM=%u L_EN=%u R_EN=%u\n",
                leftMotor.lpwm, leftMotor.rpwm, leftMotor.len, leftMotor.ren);
  Serial.printf("  Right LPWM=%u RPWM=%u L_EN=%u R_EN=%u\n",
                rightMotor.lpwm, rightMotor.rpwm, rightMotor.len, rightMotor.ren);
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

void driveBoth(int left, int right) {
  drive(leftMotor, left, LEFT_MOTOR_INVERT);
  drive(rightMotor, right, RIGHT_MOTOR_INVERT);
  Serial.printf("left=%d right=%d\n", left, right);
}

void stopMotors() {
  driveBoth(0, 0);
}

void setupMotorPins() {
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

void printHelp() {
  Serial.println();
  Serial.println("BTS motor test commands:");
  Serial.println("  w forward");
  Serial.println("  s reverse");
  Serial.println("  a spin left");
  Serial.println("  d spin right");
  Serial.println("  i left motor forward");
  Serial.println("  k left motor reverse");
  Serial.println("  o right motor forward");
  Serial.println("  l right motor reverse");
  Serial.println("  0 stop");
  Serial.println("  + speed up");
  Serial.println("  - speed down");
  Serial.println("  r toggle slow auto ramp");
  Serial.println("  p print pinout");
  Serial.println("Keep wheels off the ground.");
  Serial.println();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') driveBoth(testSpeed, testSpeed);
    else if (c == 's') driveBoth(-testSpeed, -testSpeed);
    else if (c == 'a') driveBoth(testSpeed, -testSpeed);
    else if (c == 'd') driveBoth(-testSpeed, testSpeed);
    else if (c == 'i') driveBoth(testSpeed, 0);
    else if (c == 'k') driveBoth(-testSpeed, 0);
    else if (c == 'o') driveBoth(0, testSpeed);
    else if (c == 'l') driveBoth(0, -testSpeed);
    else if (c == '0') {
      autoRamp = false;
      stopMotors();
    } else if (c == '+') {
      testSpeed = constrain(testSpeed + 10, 0, 255);
      Serial.printf("testSpeed=%d\n", testSpeed);
    } else if (c == '-') {
      testSpeed = constrain(testSpeed - 10, 0, 255);
      Serial.printf("testSpeed=%d\n", testSpeed);
    } else if (c == 'r') {
      autoRamp = !autoRamp;
      Serial.println(autoRamp ? "auto ramp on" : "auto ramp off");
      if (!autoRamp) stopMotors();
    } else if (c == 'p') {
      printPinout();
    } else if (c == 'h' || c == '?') {
      printHelp();
    }
  }
}

void runAutoRamp() {
  if (!autoRamp) return;

  static uint32_t lastStep = 0;
  static int speed = 0;
  static int dir = 1;

  if (millis() - lastStep < 80) return;
  lastStep = millis();

  speed += dir * 5;
  if (speed >= testSpeed || speed <= -testSpeed) dir = -dir;
  driveBoth(speed, speed);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  setupMotorPins();
  printPinout();
  printHelp();
}

void loop() {
  handleSerial();
  runAutoRamp();
}
