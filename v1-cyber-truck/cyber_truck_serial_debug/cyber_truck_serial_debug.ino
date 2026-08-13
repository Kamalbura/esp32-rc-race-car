#include <Arduino.h>
#include <FastLED.h>

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

constexpr int MAX_DRIVE_SPEED = 255;
constexpr int RAMP_STEP = 12;
constexpr unsigned long DIRECTION_CHANGE_DELAY = 300;
constexpr unsigned long STATUS_INTERVAL = 1000;

enum LightOverrideMode {
  LIGHT_AUTO = -1,
  LIGHT_OFF = 0,
  LIGHT_LEFT,
  LIGHT_RIGHT,
  LIGHT_FORWARD,
  LIGHT_REVERSE,
  LIGHT_HAZARD,
  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_BLUE,
  LIGHT_WHITE
};

int ledMode = 3;
LightOverrideMode lightOverride = LIGHT_AUTO;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int actualLeftSpeed = 0;
int actualRightSpeed = 0;
int currentDirection = 0;
int defaultSpeed = 180;
unsigned long lastDirectionChange = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastStatusPrint = 0;
String serialBuffer;

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

void applyLightOverride() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  switch (lightOverride) {
    case LIGHT_OFF:
      break;

    case LIGHT_LEFT:
      leds[9] = CRGB::Yellow;
      leds[10] = CRGB::Yellow;
      break;

    case LIGHT_RIGHT:
      leds[2] = CRGB::Yellow;
      leds[3] = CRGB::Yellow;
      break;

    case LIGHT_FORWARD:
      leds[0] = CRGB::Blue;
      leds[1] = CRGB::Blue;
      leds[11] = CRGB::Blue;
      break;

    case LIGHT_REVERSE:
      leds[5] = CRGB::Red;
      leds[6] = CRGB::Red;
      break;

    case LIGHT_HAZARD: {
      bool onState = ((millis() / 250) % 2) == 0;
      if (onState) {
        fill_solid(leds, NUM_LEDS, CRGB::Orange);
      }
      break;
    }

    case LIGHT_RED:
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      break;

    case LIGHT_GREEN:
      fill_solid(leds, NUM_LEDS, CRGB::Green);
      break;

    case LIGHT_BLUE:
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
      break;

    case LIGHT_WHITE:
      fill_solid(leds, NUM_LEDS, CRGB::White);
      break;

    default:
      break;
  }
}

void updateLEDs() {
  if (lightOverride != LIGHT_AUTO) {
    applyLightOverride();
    FastLED.show();
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

int parseSpeedValue(const String &argument, int fallbackSpeed) {
  if (argument.length() == 0) {
    return fallbackSpeed;
  }
  return constrain(argument.toInt(), 0, MAX_DRIVE_SPEED);
}

void printStatus() {
  Serial.print("LED mode: ");
  Serial.print(ledMode);
  Serial.print(" | Light override: ");
  Serial.print(int(lightOverride));
  Serial.print(" | Default speed: ");
  Serial.print(defaultSpeed);
  Serial.print(" | Target L/R: ");
  Serial.print(targetLeftSpeed);
  Serial.print(" / ");
  Serial.print(targetRightSpeed);
  Serial.print(" | Actual L/R: ");
  Serial.print(actualLeftSpeed);
  Serial.print(" / ");
  Serial.println(actualRightSpeed);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  f [speed]           forward");
  Serial.println("  b [speed]           backward");
  Serial.println("  l [speed]           spin left");
  Serial.println("  r [speed]           spin right");
  Serial.println("  s                   stop");
  Serial.println("  drive <t> <s>       throttle and steering, -255..255");
  Serial.println("  speed <0-255>       set default speed");
  Serial.println("  mode <0-4>          LED auto mode");
  Serial.println("  lights auto         return to LED auto mode");
  Serial.println("  lights off          LEDs off");
  Serial.println("  lights left|right|forward|reverse|hazard");
  Serial.println("  lights red|green|blue|white");
  Serial.println("  status              print current state");
  Serial.println("  help                show commands");
}

void handleLightsCommand(String argument) {
  argument.trim();
  argument.toLowerCase();

  if (argument == "auto") {
    lightOverride = LIGHT_AUTO;
  } else if (argument == "off") {
    lightOverride = LIGHT_OFF;
  } else if (argument == "left" || argument == "l") {
    lightOverride = LIGHT_LEFT;
  } else if (argument == "right" || argument == "r") {
    lightOverride = LIGHT_RIGHT;
  } else if (argument == "forward" || argument == "f") {
    lightOverride = LIGHT_FORWARD;
  } else if (argument == "reverse" || argument == "b" || argument == "back") {
    lightOverride = LIGHT_REVERSE;
  } else if (argument == "hazard" || argument == "h") {
    lightOverride = LIGHT_HAZARD;
  } else if (argument == "red") {
    lightOverride = LIGHT_RED;
  } else if (argument == "green") {
    lightOverride = LIGHT_GREEN;
  } else if (argument == "blue") {
    lightOverride = LIGHT_BLUE;
  } else if (argument == "white") {
    lightOverride = LIGHT_WHITE;
  } else {
    Serial.println("Unknown lights command");
    return;
  }

  Serial.print("Light override set to: ");
  Serial.println(argument);
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  int separatorIndex = command.indexOf(' ');
  String keyword = separatorIndex == -1 ? command : command.substring(0, separatorIndex);
  String argument = separatorIndex == -1 ? "" : command.substring(separatorIndex + 1);
  argument.trim();

  if (keyword == "f" || keyword == "forward") {
    setDriveMix(parseSpeedValue(argument, defaultSpeed), 0);
  } else if (keyword == "b" || keyword == "back" || keyword == "reverse") {
    setDriveMix(-parseSpeedValue(argument, defaultSpeed), 0);
  } else if (keyword == "l" || keyword == "left") {
    setDriveMix(0, -parseSpeedValue(argument, defaultSpeed));
  } else if (keyword == "r" || keyword == "right") {
    setDriveMix(0, parseSpeedValue(argument, defaultSpeed));
  } else if (keyword == "s" || keyword == "stop") {
    setDriveMix(0, 0);
  } else if (keyword == "speed") {
    defaultSpeed = parseSpeedValue(argument, defaultSpeed);
    Serial.print("Default speed set to ");
    Serial.println(defaultSpeed);
  } else if (keyword == "mode") {
    ledMode = constrain(argument.toInt(), 0, 4);
    lightOverride = LIGHT_AUTO;
    Serial.print("LED auto mode set to ");
    Serial.println(ledMode);
  } else if (keyword == "drive") {
    int splitIndex = argument.indexOf(' ');
    if (splitIndex == -1) {
      Serial.println("Usage: drive <throttle> <steering>");
      return;
    }
    int throttleValue = constrain(argument.substring(0, splitIndex).toInt(), -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    int steeringValue = constrain(argument.substring(splitIndex + 1).toInt(), -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    setDriveMix(throttleValue, steeringValue);
  } else if (keyword == "lights") {
    handleLightsCommand(argument);
  } else if (keyword == "status") {
    printStatus();
  } else if (keyword == "help") {
    printHelp();
  } else {
    Serial.println("Unknown command");
    printHelp();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incomingChar = char(Serial.read());

    if (incomingChar == '\r') {
      continue;
    }

    if (incomingChar == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
      continue;
    }

    if (serialBuffer.length() < 96) {
      serialBuffer += incomingChar;
    }
  }
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

  Serial.println("Cyber Truck serial debug ready");
  printHelp();
}

void loop() {
  readSerialCommands();
  updateDriveOutputs();
  updateLEDs();

  if (millis() - lastStatusPrint >= STATUS_INTERVAL) {
    lastStatusPrint = millis();
    printStatus();
  }

  delay(10);
}
