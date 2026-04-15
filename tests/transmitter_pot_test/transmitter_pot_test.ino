#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ADS1115_ADDR 0x48

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define ENCODER_SW_PIN 6

Adafruit_ADS1115 ads;

int16_t minRaw[4] = {32767, 32767, 32767, 32767};
int16_t maxRaw[4] = {-32768, -32768, -32768, -32768};
int16_t centerRaw[4] = {0, 0, 0, 0};

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

void printHelp() {
  Serial.println();
  Serial.println("Transmitter ADS1115 / potentiometer test");
  Serial.println("  c = capture joystick centers");
  Serial.println("  r = reset min/max tracking");
  Serial.println("Move each joystick fully, then copy min/center/max into transmitter_code axisCal[].");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

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
    else if (c == 'r') {
      resetMinMax();
      Serial.println("Min/max reset.");
    } else if (c == 'h' || c == '?') {
      printHelp();
    }
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

  Serial.printf("ENC A=%u B=%u SW=%u\n",
                digitalRead(ENCODER_A_PIN),
                digitalRead(ENCODER_B_PIN),
                digitalRead(ENCODER_SW_PIN));
}
