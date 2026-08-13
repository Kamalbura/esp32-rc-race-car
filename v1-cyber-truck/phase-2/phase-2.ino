// RC Car with NeoPixel LEDs + MPU6050 (optional)
// Channels: CH1-CH6 on pins 36, 39, 34, 35, 32, 33
// Motor control: PWM on pins 25, 26, 27, 18 | Enable: 19, 23
// NeoPixel: Pin 13 (12 LEDs total)

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 13
#define LED_COUNT 12
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Channel pins
int ch1 = 36, ch2 = 39, ch3 = 34, ch4 = 35, ch5 = 32, ch6 = 33;

// Motor PWM pins
int mL1 = 25, mL2 = 26, mR1 = 27, mR2 = 18;
int enL = 19, enR = 23;

Adafruit_MPU6050 mpu;
bool mpuAvailable = false;

float kalmanX = 0, kalmanY = 0, compX = 0, compY = 0, hybridX = 0, hybridY = 0;
float lastTime = 0;

unsigned long blinkTime = 0;
bool blinkState = false;

unsigned long lastDirectionChange = 0;
int prevDirection = 0; // 0 = stop, 1 = forward, -1 = reverse
#define DIRECTION_CHANGE_DELAY 300 // ms to wait before changing direction
#define SIGNAL_TIMEOUT 500 // ms to wait before considering signal lost
unsigned long lastValidSignal = 0;

int readPWM(int pin) {
  uint32_t duration = pulseIn(pin, HIGH, 25000);
  if (duration > 0) {
    lastValidSignal = millis(); // Update the last valid signal time
    return constrain(duration, 1000, 2000);
  }
  return 1500; // Center/neutral position if no signal
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);

  pinMode(mL1, OUTPUT);
  pinMode(mL2, OUTPUT);
  pinMode(mR1, OUTPUT);
  pinMode(mR2, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);

  analogWrite(enL, 0);
  analogWrite(enR, 0);

  if (mpu.begin()) {
    mpuAvailable = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }
}

void updateMPU() {
  if (!mpuAvailable) return;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  float accX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;
  float gyroX = g.gyro.x * 180 / PI;
  float gyroY = g.gyro.y * 180 / PI;

  kalmanX += dt * (gyroX - 0.05 * (kalmanX - accX));
  kalmanY += dt * (gyroY - 0.05 * (kalmanY - accY));

  compX = 0.98 * (compX + gyroX * dt) + 0.02 * accX;
  compY = 0.98 * (compY + gyroY * dt) + 0.02 * accY;

  hybridX = (kalmanX + compX) / 2;
  hybridY = (kalmanY + compY) / 2;
}

void setMotors(int l1, int l2, int r1, int r2, int speed) {
  int newDirection = 0;
  if (l1 == 1 || r1 == 1) newDirection = 1;      // forward
  else if (l2 == 1 || r2 == 1) newDirection = -1; // reverse
  
  // Prevent sudden direction changes
  if (prevDirection != 0 && newDirection != 0 && 
      prevDirection != newDirection && 
      millis() - lastDirectionChange < DIRECTION_CHANGE_DELAY) {
    // Force stop for a moment when changing directions
    digitalWrite(mL1, 0);
    digitalWrite(mL2, 0);
    digitalWrite(mR1, 0);
    digitalWrite(mR2, 0);
    analogWrite(enL, 0);
    analogWrite(enR, 0);
    return;
  }
  
  // If direction is changing, record the time
  if (prevDirection != newDirection) {
    lastDirectionChange = millis();
    prevDirection = newDirection;
    
    // Briefly stop motors before changing direction
    if (prevDirection != 0 && newDirection != 0 && prevDirection != newDirection) {
      digitalWrite(mL1, 0);
      digitalWrite(mL2, 0);
      digitalWrite(mR1, 0);
      digitalWrite(mR2, 0);
      analogWrite(enL, 0);
      analogWrite(enR, 0);
      delay(50); // Brief delay for motors to stop
    }
  }
  
  // Apply gradual speed changes
  static int currentSpeed = 0;
  if (speed > currentSpeed) currentSpeed = min(speed, currentSpeed + 10);
  else if (speed < currentSpeed) currentSpeed = max(speed, currentSpeed - 10);
  
  // Now set the motor states
  digitalWrite(mL1, l1);
  digitalWrite(mL2, l2);
  digitalWrite(mR1, r1);
  digitalWrite(mR2, r2);
  analogWrite(enL, currentSpeed);
  analogWrite(enR, currentSpeed);
}

void updateLEDs(int ch1Val, int ch2Val, int ch5Val, int ch6Val) {
  strip.clear();

  bool hazard = ch5Val > 1500;
  bool turnAssist = ch6Val > 1500;

  if (hazard) {
    if (millis() - blinkTime > 300) {
      blinkState = !blinkState;
      blinkTime = millis();
    }
    if (blinkState) strip.fill(strip.Color(255, 140, 0));

  } else if (ch2Val > 1510) {
    // Forward
    strip.fill(strip.Color(255, 255, 255));
    if (ch1Val < 1490) for (int i = 0; i < 6; i++) strip.setPixelColor(i, strip.Color(255, 140, 0));
    else if (ch1Val > 1510) for (int i = 6; i < 12; i++) strip.setPixelColor(i, strip.Color(255, 140, 0));

  } else if (ch2Val < 1490) {
    // Reverse
    strip.fill(strip.Color(0, 0, 255));
    if (ch1Val < 1490) for (int i = 0; i < 6; i++) strip.setPixelColor(i, strip.Color(255, 140, 0));
    else if (ch1Val > 1510) for (int i = 6; i < 12; i++) strip.setPixelColor(i, strip.Color(255, 140, 0));

  } else if (abs(hybridX) > 20 && mpuAvailable) {
    for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
  }

  strip.show();
}

void loop() {
  int val1 = readPWM(ch1);
  int val2 = readPWM(ch2);
  int val3 = readPWM(ch3);
  int val5 = readPWM(ch5);
  int val6 = readPWM(ch6);

  updateMPU();
  updateLEDs(val1, val2, val5, val6);
  
  // Check for signal loss
  if (millis() - lastValidSignal > SIGNAL_TIMEOUT) {
    // Signal lost - implement failsafe
    setMotors(0, 0, 0, 0, 0);
    
    // Flash warning LEDs
    if (millis() - blinkTime > 100) {
      blinkState = !blinkState;
      blinkTime = millis();
      strip.clear();
      if (blinkState) strip.fill(strip.Color(255, 0, 0));
      strip.show();
    }
    return; // Don't process further until signal returns
  }

  int throttle = map(val3, 1000, 2000, 0, 255);

  if (val2 > 1510) {
    // Forward
    if (val1 < 1490) {
      // Left turn
      if (val6 > 1500) setMotors(0, 1, 1, 0, throttle); // Aggressive
      else setMotors(0, 0, 1, 0, throttle);
    } else if (val1 > 1510) {
      if (val6 > 1500) setMotors(1, 0, 0, 1, throttle);
      else setMotors(1, 0, 0, 0, throttle);
    } else setMotors(1, 0, 1, 0, throttle);

  } else if (val2 < 1490) {
    // Backward
    if (val1 < 1490) {
      if (val6 > 1500) setMotors(1, 0, 0, 1, throttle);
      else setMotors(0, 0, 0, 1, throttle);
    } else if (val1 > 1510) {
      if (val6 > 1500) setMotors(0, 1, 1, 0, throttle);
      else setMotors(0, 1, 0, 0, throttle);
    } else setMotors(0, 1, 0, 1, throttle);

  } else {
    setMotors(0, 0, 0, 0, 0);
  }
}
