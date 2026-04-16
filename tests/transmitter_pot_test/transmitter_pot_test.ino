#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ADS1115_ADDR 0x48

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6
#define BOOT_BUTTON_PIN 0

Adafruit_ADS1115 ads;

int16_t minRaw[4] = {32767, 32767, 32767, 32767};
int16_t maxRaw[4] = {-32768, -32768, -32768, -32768};
int16_t centerRaw[4] = {0, 0, 0, 0};

struct MappingStep {
  const char *name;
  const char *prompt;
};

struct MappingResult {
  bool valid;
  uint8_t channel;
  int8_t direction;
  int16_t average[4];
  int32_t delta[4];
};

const MappingStep mappingSteps[] = {
  {"left_up", "Move LEFT stick UP and hold, then press BOOT/encoder button or send n"},
  {"left_down", "Move LEFT stick DOWN and hold, then press BOOT/encoder button or send n"},
  {"left_left", "Move LEFT stick LEFT and hold, then press BOOT/encoder button or send n"},
  {"left_right", "Move LEFT stick RIGHT and hold, then press BOOT/encoder button or send n"},
  {"right_up", "Move RIGHT stick UP and hold, then press BOOT/encoder button or send n"},
  {"right_down", "Move RIGHT stick DOWN and hold, then press BOOT/encoder button or send n"},
  {"right_left", "Move RIGHT stick LEFT and hold, then press BOOT/encoder button or send n"},
  {"right_right", "Move RIGHT stick RIGHT and hold, then press BOOT/encoder button or send n"},
};

const uint8_t MAPPING_STEP_COUNT = sizeof(mappingSteps) / sizeof(mappingSteps[0]);
const uint8_t CAPTURE_SAMPLES = 120;
const uint16_t CAPTURE_DELAY_MS = 4;
const int32_t MOVEMENT_THRESHOLD = 900;

MappingResult mappingResults[MAPPING_STEP_COUNT] = {};
bool mappingActive = false;
uint8_t mappingStepIndex = 0;

void resetMinMax() {
  for (uint8_t i = 0; i < 4; i++) {
    minRaw[i] = 32767;
    maxRaw[i] = -32768;
  }
}

void captureCenter() {
  for (uint8_t i = 0; i < 4; i++) {
    int32_t sum = 0;
    for (uint8_t n = 0; n < 20; n++) {
      sum += ads.readADC_SingleEnded(i);
      delay(2);
    }
    centerRaw[i] = sum / 20;
  }
  Serial.println("Center captured.");
}

int16_t splitMapAxis(int16_t raw, int16_t minVal, int16_t centerVal, int16_t maxVal) {
  const int16_t deadband = 80;
  int32_t value;

  if (abs(raw - centerVal) <= deadband) {
    value = 0;
  } else if (raw > centerVal) {
    int32_t span = max<int32_t>(1, maxVal - centerVal - deadband);
    value = (int32_t)(raw - centerVal - deadband) * 1000L / span;
  } else {
    int32_t span = max<int32_t>(1, centerVal - minVal - deadband);
    value = -(int32_t)(centerVal - raw - deadband) * 1000L / span;
  }

  return constrain(value, -1000, 1000);
}

const char *polarityText(int8_t direction) {
  if (direction > 0) return "POS";
  if (direction < 0) return "NEG";
  return "CENTER";
}

bool captureTriggerPressed() {
  static bool lastPressed = false;
  static uint32_t lastEdgeMs = 0;

  bool pressed = (digitalRead(ENCODER_SW_PIN) == LOW) || (digitalRead(BOOT_BUTTON_PIN) == LOW);
  bool triggered = false;

  if (pressed != lastPressed && millis() - lastEdgeMs > 80) {
    lastEdgeMs = millis();
    if (pressed) triggered = true;
    lastPressed = pressed;
  }
  return triggered;
}

void printMappingPrompt() {
  if (!mappingActive || mappingStepIndex >= MAPPING_STEP_COUNT) return;
  Serial.println();
  Serial.print("MAP STEP ");
  Serial.print(mappingStepIndex + 1);
  Serial.print("/");
  Serial.print(MAPPING_STEP_COUNT);
  Serial.print(": ");
  Serial.println(mappingSteps[mappingStepIndex].name);
  Serial.println(mappingSteps[mappingStepIndex].prompt);
  Serial.println("Send x to cancel mapping mode.");
}

MappingResult captureMappingResult() {
  MappingResult result = {};
  int64_t sum[4] = {0, 0, 0, 0};

  for (uint8_t sample = 0; sample < CAPTURE_SAMPLES; sample++) {
    for (uint8_t i = 0; i < 4; i++) {
      sum[i] += ads.readADC_SingleEnded(i);
    }
    delay(CAPTURE_DELAY_MS);
  }

  int32_t bestAbsDelta = 0;
  for (uint8_t i = 0; i < 4; i++) {
    result.average[i] = sum[i] / CAPTURE_SAMPLES;
    result.delta[i] = result.average[i] - centerRaw[i];
    int32_t absDelta = abs(result.delta[i]);
    if (absDelta > bestAbsDelta) {
      bestAbsDelta = absDelta;
      result.channel = i;
    }
  }

  result.valid = bestAbsDelta >= MOVEMENT_THRESHOLD;
  if (result.valid) {
    result.direction = result.delta[result.channel] >= 0 ? 1 : -1;
  }
  return result;
}

void printOneMappingResult(const char *label, const MappingResult &result) {
  Serial.print(label);
  Serial.print(": ");
  if (!result.valid) {
    Serial.println("no strong movement detected");
    return;
  }

  Serial.print("A");
  Serial.print(result.channel);
  Serial.print(" ");
  Serial.print(polarityText(result.direction));
  Serial.print("  delta=[");
  for (uint8_t i = 0; i < 4; i++) {
    if (i) Serial.print(", ");
    Serial.print(result.delta[i]);
  }
  Serial.println("]");
}

void printAxisPairSummary(const char *label,
                          const MappingResult &first,
                          const char *firstDir,
                          const MappingResult &second,
                          const char *secondDir) {
  Serial.print(label);
  Serial.print(": ");

  if (!first.valid || !second.valid) {
    Serial.println("incomplete");
    return;
  }

  if (first.channel != second.channel) {
    Serial.print("mismatch -> ");
    Serial.print(firstDir);
    Serial.print(" uses A");
    Serial.print(first.channel);
    Serial.print(", ");
    Serial.print(secondDir);
    Serial.print(" uses A");
    Serial.println(second.channel);
    return;
  }

  if (first.direction == second.direction) {
    Serial.print("same polarity on both directions -> check wiring on A");
    Serial.println(first.channel);
    return;
  }

  Serial.print("A");
  Serial.print(first.channel);
  Serial.print("  ");
  Serial.print(firstDir);
  Serial.print("=");
  Serial.print(polarityText(first.direction));
  Serial.print("  ");
  Serial.print(secondDir);
  Serial.print("=");
  Serial.println(polarityText(second.direction));
}

void printMappingSummary() {
  Serial.println();
  Serial.println("Joystick direction mapping summary:");
  for (uint8_t i = 0; i < MAPPING_STEP_COUNT; i++) {
    printOneMappingResult(mappingSteps[i].name, mappingResults[i]);
  }

  Serial.println();
  Serial.println("Suggested stick axes:");
  printAxisPairSummary("Left stick vertical", mappingResults[0], "UP", mappingResults[1], "DOWN");
  printAxisPairSummary("Left stick horizontal", mappingResults[2], "LEFT", mappingResults[3], "RIGHT");
  printAxisPairSummary("Right stick vertical", mappingResults[4], "UP", mappingResults[5], "DOWN");
  printAxisPairSummary("Right stick horizontal", mappingResults[6], "LEFT", mappingResults[7], "RIGHT");
  Serial.println();
}

void startMappingMode() {
  memset(mappingResults, 0, sizeof(mappingResults));
  mappingActive = true;
  mappingStepIndex = 0;
  captureCenter();
  Serial.println();
  Serial.println("Joystick mapping mode started.");
  Serial.println("Keep sticks centered while center capture runs.");
  printMappingPrompt();
}

void finishMappingStep() {
  MappingResult result = captureMappingResult();
  mappingResults[mappingStepIndex] = result;
  printOneMappingResult(mappingSteps[mappingStepIndex].name, result);

  mappingStepIndex++;
  if (mappingStepIndex >= MAPPING_STEP_COUNT) {
    mappingActive = false;
    Serial.println("Mapping sequence finished.");
    printMappingSummary();
  } else {
    printMappingPrompt();
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Transmitter ADS1115 / potentiometer test");
  Serial.println("  c = capture joystick centers");
  Serial.println("  g = start guided joystick mapping");
  Serial.println("  n = capture current guided mapping step");
  Serial.println("  r = reset min/max tracking");
  Serial.println("  x = cancel guided mapping");
  Serial.println("Move each joystick fully, then copy min/center/max into transmitter_code axisCal[].");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ADS1115 not found. Check SDA/SCL and tie ADDR to GND for address 0x48.");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  captureCenter();
  resetMinMax();
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') captureCenter();
    else if (c == 'g') startMappingMode();
    else if (c == 'n') {
      if (mappingActive) finishMappingStep();
    }
    else if (c == 'r') {
      resetMinMax();
      Serial.println("Min/max reset.");
    } else if (c == 'x') {
      mappingActive = false;
      Serial.println("Mapping mode cancelled.");
    } else if (c == 'h' || c == '?') {
      printHelp();
    }
  }

  if (mappingActive && captureTriggerPressed()) {
    finishMappingStep();
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  Serial.print("ADS ");
  for (uint8_t i = 0; i < 4; i++) {
    int16_t raw = ads.readADC_SingleEnded(i);
    minRaw[i] = min(minRaw[i], raw);
    maxRaw[i] = max(maxRaw[i], raw);
    int16_t mapped = splitMapAxis(raw, minRaw[i], centerRaw[i], maxRaw[i]);

    Serial.printf("A%u raw=%6d min=%6d ctr=%6d max=%6d map=%5d  ",
                  i, raw, minRaw[i], centerRaw[i], maxRaw[i], mapped);
  }

  bool swPressed = digitalRead(ENCODER_SW_PIN) == LOW;
  bool bootPressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
  Serial.printf("ENC A=%u B=%u SWraw=%u SW=%s BOOTraw=%u BOOT=%s\n",
                digitalRead(ENCODER_A_PIN),
                digitalRead(ENCODER_B_PIN),
                digitalRead(ENCODER_SW_PIN),
                swPressed ? "PRESSED" : "RELEASED",
                digitalRead(BOOT_BUTTON_PIN),
                bootPressed ? "PRESSED" : "RELEASED");

  if (mappingActive) {
    Serial.printf("ACTIVE MAP STEP: %s\n", mappingSteps[mappingStepIndex].name);
  }
}
