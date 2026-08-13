#include <Arduino.h>
#include <FastLED.h>

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// Motor pins
constexpr uint8_t MOTOR_L_FWD = 26;
constexpr uint8_t MOTOR_L_REV = 27;
constexpr uint8_t MOTOR_R_FWD = 23;
constexpr uint8_t MOTOR_R_REV = 25;
constexpr uint8_t MOTOR_BL_FWD = 18;
constexpr uint8_t MOTOR_BL_REV = 19;
constexpr uint8_t MOTOR_BR_FWD = 5;
constexpr uint8_t MOTOR_BR_REV = 17;

// LED strip
constexpr uint8_t LED_PIN = 13;
constexpr uint8_t NUM_LEDS = 12;
constexpr uint8_t LED_BRIGHTNESS = 60;
CRGB leds[NUM_LEDS];

// PWM settings
constexpr uint16_t PWM_FREQ = 5000;
constexpr uint8_t PWM_RESOLUTION = 8;

struct MotorPwmPin {
  uint8_t pin;
  uint8_t channel;
};

MotorPwmPin leftForward = {MOTOR_L_FWD, 0};
MotorPwmPin leftReverse = {MOTOR_L_REV, 1};
MotorPwmPin rightForward = {MOTOR_R_FWD, 2};
MotorPwmPin rightReverse = {MOTOR_R_REV, 3};
MotorPwmPin backLeftForward = {MOTOR_BL_FWD, 4};
MotorPwmPin backLeftReverse = {MOTOR_BL_REV, 5};
MotorPwmPin backRightForward = {MOTOR_BR_FWD, 6};
MotorPwmPin backRightReverse = {MOTOR_BR_REV, 7};

// Control tuning
constexpr int MAX_DRIVE_SPEED = 255;
constexpr int DIGITAL_DRIVE_SPEED = 220;
constexpr int DIGITAL_TURN_SPEED = 180;
constexpr int RAMP_STEP = 12;
constexpr unsigned long DIRECTION_CHANGE_DELAY = 300;
constexpr unsigned long DEBUG_INTERVAL = 300;
constexpr unsigned long LED_MODE_DEBOUNCE = 250;

int ledMode = 3;
bool failsafeActive = true;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int actualLeftSpeed = 0;
int actualRightSpeed = 0;
int currentDirection = 0;
unsigned long lastDirectionChange = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastDebugPrint = 0;
unsigned long lastModeChange = 0;

bool lastStartPressed = false;
bool lastSelectPressed = false;

void configurePwmPin(const MotorPwmPin &motor) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(motor.pin, PWM_FREQ, PWM_RESOLUTION, motor.channel);
#else
  ledcSetup(motor.channel, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(motor.pin, motor.channel);
#endif
}

void writePwm(const MotorPwmPin &motor, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motor.pin, duty);
#else
  ledcWrite(motor.channel, duty);
#endif
}

int moveTowards(int currentValue, int targetValue, int stepSize) {
  if (currentValue < targetValue) {
    return min(currentValue + stepSize, targetValue);
  }
  if (currentValue > targetValue) {
    return max(currentValue - stepSize, targetValue);
  }
  return currentValue;
}

void writeMotorPair(const MotorPwmPin &forwardMotor, const MotorPwmPin &reverseMotor, int speedValue) {
  speedValue = constrain(speedValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

  if (speedValue >= 0) {
    writePwm(forwardMotor, speedValue);
    writePwm(reverseMotor, 0);
  } else {
    writePwm(forwardMotor, 0);
    writePwm(reverseMotor, -speedValue);
  }
}

void applyRawMotorSpeeds(int leftSpeed, int rightSpeed) {
  writeMotorPair(leftForward, leftReverse, leftSpeed);
  writeMotorPair(backLeftForward, backLeftReverse, leftSpeed);
  writeMotorPair(rightForward, rightReverse, rightSpeed);
  writeMotorPair(backRightForward, backRightReverse, rightSpeed);
}

void stopAllMotors() {
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  actualLeftSpeed = 0;
  actualRightSpeed = 0;
  currentDirection = 0;
  applyRawMotorSpeeds(0, 0);
}

void setDriveMix(int throttleValue, int steeringValue) {
  throttleValue = constrain(throttleValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  steeringValue = constrain(steeringValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

  targetLeftSpeed = constrain(throttleValue + steeringValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  targetRightSpeed = constrain(throttleValue - steeringValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
}

void updateDriveOutputs() {
  int requestedDirection = 0;
  if (targetLeftSpeed > 0 || targetRightSpeed > 0) {
    requestedDirection = 1;
  } else if (targetLeftSpeed < 0 || targetRightSpeed < 0) {
    requestedDirection = -1;
  }

  if (currentDirection != 0 &&
      requestedDirection != 0 &&
      requestedDirection != currentDirection &&
      millis() - lastDirectionChange < DIRECTION_CHANGE_DELAY) {
    actualLeftSpeed = moveTowards(actualLeftSpeed, 0, RAMP_STEP * 2);
    actualRightSpeed = moveTowards(actualRightSpeed, 0, RAMP_STEP * 2);
    applyRawMotorSpeeds(actualLeftSpeed, actualRightSpeed);
    return;
  }

  if (requestedDirection != currentDirection) {
    currentDirection = requestedDirection;
    lastDirectionChange = millis();
  }

  actualLeftSpeed = moveTowards(actualLeftSpeed, targetLeftSpeed, RAMP_STEP);
  actualRightSpeed = moveTowards(actualRightSpeed, targetRightSpeed, RAMP_STEP);
  applyRawMotorSpeeds(actualLeftSpeed, actualRightSpeed);
}

void showFailsafeBlink() {
  static bool ledState = false;
  static unsigned long lastBlinkTime = 0;

  if (millis() - lastBlinkTime >= 180) {
    lastBlinkTime = millis();
    ledState = !ledState;
    fill_solid(leds, NUM_LEDS, ledState ? CRGB::Red : CRGB::Black);
    FastLED.show();
  }
}

void updateLEDs() {
  if (failsafeActive) {
    showFailsafeBlink();
    return;
  }

  switch (ledMode) {
    case 0:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      break;

    case 1: {
      int speedValue = max(abs(actualLeftSpeed), abs(actualRightSpeed));
      CRGB color = CRGB(speedValue, 255 - speedValue, 0);
      fill_solid(leds, NUM_LEDS, color);
      break;
    }

    case 2: {
      static uint8_t startIndex = 0;
      if (millis() - lastLedUpdate >= 20) {
        lastLedUpdate = millis();
        startIndex++;
        fill_rainbow(leds, NUM_LEDS, startIndex, 255 / NUM_LEDS);
      }
      break;
    }

    case 3:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      if (actualLeftSpeed > 20 || actualRightSpeed > 20) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
        leds[11] = CRGB::Blue;
      } else if (actualLeftSpeed < -20 || actualRightSpeed < -20) {
        leds[5] = CRGB::Red;
        leds[6] = CRGB::Red;
      }

      if (actualLeftSpeed > actualRightSpeed + 25) {
        leds[9] = CRGB::Yellow;
        leds[10] = CRGB::Yellow;
      } else if (actualRightSpeed > actualLeftSpeed + 25) {
        leds[2] = CRGB::Yellow;
        leds[3] = CRGB::Yellow;
      }
      break;

    case 4: {
      static uint8_t pos = 0;
      if (millis() - lastLedUpdate >= 50) {
        lastLedUpdate = millis();
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        leds[pos] = CRGB::Purple;
        pos = (pos + 1) % NUM_LEDS;
      }
      break;
    }

    default:
      ledMode = 0;
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      break;
  }

  FastLED.show();
}

void handleGamePad() {
  Dabble.processInput();

  bool appConnected = Dabble.isAppConnected();
  failsafeActive = !appConnected;

  if (!appConnected) {
    stopAllMotors();
    return;
  }

  bool startPressed = GamePad.isStartPressed();
  bool selectPressed = GamePad.isSelectPressed();

  if (startPressed && !lastStartPressed && millis() - lastModeChange >= LED_MODE_DEBOUNCE) {
    ledMode = (ledMode + 1) % 5;
    lastModeChange = millis();
  }

  if (selectPressed && !lastSelectPressed) {
    ledMode = 0;
  }

  lastStartPressed = startPressed;
  lastSelectPressed = selectPressed;

  int throttleValue = 0;
  int steeringValue = 0;

  bool upPressed = GamePad.isUpPressed();
  bool downPressed = GamePad.isDownPressed();
  bool leftPressed = GamePad.isLeftPressed();
  bool rightPressed = GamePad.isRightPressed();

  if (upPressed || downPressed || leftPressed || rightPressed) {
    if (upPressed) {
      throttleValue += DIGITAL_DRIVE_SPEED;
    }
    if (downPressed) {
      throttleValue -= DIGITAL_DRIVE_SPEED;
    }
    if (leftPressed) {
      steeringValue -= DIGITAL_TURN_SPEED;
    }
    if (rightPressed) {
      steeringValue += DIGITAL_TURN_SPEED;
    }

    if (!upPressed && !downPressed) {
      steeringValue = constrain(steeringValue, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    }
  } else {
    float joystickX = GamePad.getXaxisData();
    float joystickY = GamePad.getYaxisData();

    throttleValue = int(joystickY * 36.0f);
    steeringValue = int(joystickX * 36.0f);

    if (abs(throttleValue) < 20) {
      throttleValue = 0;
    }
    if (abs(steeringValue) < 20) {
      steeringValue = 0;
    }
  }

  if (GamePad.isTrianglePressed()) {
    ledMode = 1;
  } else if (GamePad.isCirclePressed()) {
    ledMode = 2;
  } else if (GamePad.isSquarePressed()) {
    ledMode = 3;
  } else if (GamePad.isCrossPressed()) {
    setDriveMix(0, 0);
    return;
  }

  setDriveMix(throttleValue, steeringValue);
}

void printDebugInfo() {
  if (millis() - lastDebugPrint < DEBUG_INTERVAL) {
    return;
  }

  lastDebugPrint = millis();
  Serial.print("Connected: ");
  Serial.print(Dabble.isAppConnected() ? "yes" : "no");
  Serial.print(" | LED mode: ");
  Serial.print(ledMode);
  Serial.print(" | Target L/R: ");
  Serial.print(targetLeftSpeed);
  Serial.print(" / ");
  Serial.print(targetRightSpeed);
  Serial.print(" | Actual L/R: ");
  Serial.print(actualLeftSpeed);
  Serial.print(" / ");
  Serial.println(actualRightSpeed);
}

void runStartupAnimation() {
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(40);
  }
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  configurePwmPin(leftForward);
  configurePwmPin(leftReverse);
  configurePwmPin(rightForward);
  configurePwmPin(rightReverse);
  configurePwmPin(backLeftForward);
  configurePwmPin(backLeftReverse);
  configurePwmPin(backRightForward);
  configurePwmPin(backRightReverse);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  runStartupAnimation();

  stopAllMotors();

  Dabble.begin("CyberTruckESP32");

  Serial.println("Cyber Truck Dabble control ready");
  Serial.println("Use Dabble GamePad over BLE");
  Serial.println("Start = next LED mode | Select = lights off");
  Serial.println("Triangle = speed LEDs | Circle = rainbow | Square = direction | Cross = stop");
}

void loop() {
  handleGamePad();
  updateDriveOutputs();
  updateLEDs();
  printDebugInfo();
  delay(10);
}
