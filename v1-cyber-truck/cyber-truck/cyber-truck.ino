#include <FastLED.h>

// Pin Definitions
// Receiver input pins
const int RX_PIN_THROTTLE = 36;  // Channel 1
const int RX_PIN_STEERING = 39;  // Channel 2
const int RX_PIN_AUX1 = 24;      // Channel 3
const int RX_PIN_AUX2 = 32;      // Channel 4 (changed from 25 to avoid conflict)
const int RX_PIN_AUX3 = 33;      // Channel 5

// Motor driver pins (BTS drivers)
// Each driver controls 2 motors (4 motors total)
const int MOTOR_L_FWD = 26;  // Left side forward
const int MOTOR_L_REV = 27;  // Left side reverse
const int MOTOR_R_FWD = 23;  // Right side forward
const int MOTOR_R_REV = 25;  // Right side reverse
const int MOTOR_BL_FWD = 18; // Back-left forward
const int MOTOR_BL_REV = 19; // Back-left reverse
// Additional pins for back-right motors
const int MOTOR_BR_FWD = 5;  // Back-right forward
const int MOTOR_BR_REV = 17; // Back-right reverse

// LED strip
#define LED_PIN     13
#define NUM_LEDS    12
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// RC signal parameters
#define RC_MIN 1000
#define RC_MAX 2000
#define RC_MID 1500
#define RC_DEADBAND 50

// Variables to store RC pulse widths
volatile unsigned long throttleStart = 0;
volatile unsigned long steeringStart = 0;
volatile unsigned long aux1Start = 0;
volatile unsigned long aux2Start = 0;
volatile unsigned long aux3Start = 0;

volatile int throttleWidth = 0;
volatile int steeringWidth = 0;
volatile int aux1Width = 0;
volatile int aux2Width = 0;
volatile int aux3Width = 0;

// Motor speed variables
int leftSpeed = 0;
int rightSpeed = 0;
int backLeftSpeed = 0;
int backRightSpeed = 0;

// LED mode
int ledMode = 0;
unsigned long lastLedUpdate = 0;

// Failsafe
unsigned long lastValidSignal = 0;
const unsigned long failsafeTimeout = 500; // ms

void setup() {
  Serial.begin(115200);
  
  // Initialize receiver pins as inputs
  pinMode(RX_PIN_THROTTLE, INPUT);
  pinMode(RX_PIN_STEERING, INPUT);
  pinMode(RX_PIN_AUX1, INPUT);
  pinMode(RX_PIN_AUX2, INPUT);
  pinMode(RX_PIN_AUX3, INPUT);
  
  // Initialize motor control pins as outputs
  pinMode(MOTOR_L_FWD, OUTPUT);
  pinMode(MOTOR_L_REV, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT);
  pinMode(MOTOR_R_REV, OUTPUT);
  pinMode(MOTOR_BL_FWD, OUTPUT);
  pinMode(MOTOR_BL_REV, OUTPUT);
  pinMode(MOTOR_BR_FWD, OUTPUT);
  pinMode(MOTOR_BR_REV, OUTPUT);
  
  // Initialize LED strip
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  
  // Set up interrupts for RC input
  attachInterrupt(digitalPinToInterrupt(RX_PIN_THROTTLE), handleThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_PIN_STEERING), handleSteering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_PIN_AUX1), handleAux1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_PIN_AUX2), handleAux2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_PIN_AUX3), handleAux3, CHANGE);
  
  // Initialize with motors off
  stopAllMotors();
  
  // Startup LED sequence
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(50);
  }
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("Cyber Truck RC initialization complete!");
}

void loop() {
  // Check failsafe
  if (millis() - lastValidSignal > failsafeTimeout) {
    stopAllMotors();
    blinkLEDs(CRGB::Red); // Indicate failsafe activation
    return;
  }
  
  // Process RC inputs
  processInputs();
  
  // Control motors based on processed inputs
  controlMotors();
  
  // Update LEDs based on current mode and status
  updateLEDs();
  
  // Print debug info occasionally
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) {
    printDebugInfo();
    lastDebugTime = millis();
  }
}

// Interrupt handlers for RC inputs
void handleThrottle() {
  if (digitalRead(RX_PIN_THROTTLE) == HIGH) {
    throttleStart = micros();
  } else {
    throttleWidth = micros() - throttleStart;
    lastValidSignal = millis();
  }
}

void handleSteering() {
  if (digitalRead(RX_PIN_STEERING) == HIGH) {
    steeringStart = micros();
  } else {
    steeringWidth = micros() - steeringStart;
    lastValidSignal = millis();
  }
}

void handleAux1() {
  if (digitalRead(RX_PIN_AUX1) == HIGH) {
    aux1Start = micros();
  } else {
    aux1Width = micros() - aux1Start;
    lastValidSignal = millis();
  }
}

void handleAux2() {
  if (digitalRead(RX_PIN_AUX2) == HIGH) {
    aux2Start = micros();
  } else {
    aux2Width = micros() - aux2Start;
    lastValidSignal = millis();
  }
}

void handleAux3() {
  if (digitalRead(RX_PIN_AUX3) == HIGH) {
    aux3Start = micros();
  } else {
    aux3Width = micros() - aux3Start;
    lastValidSignal = millis();
  }
}

// Process RC inputs to calculate motor speeds
void processInputs() {
  // Normalize throttle to -255 to 255
  int throttle = map(constrain(throttleWidth, RC_MIN, RC_MAX), RC_MIN, RC_MAX, -255, 255);
  if (abs(throttle) < RC_DEADBAND) throttle = 0;
  
  // Normalize steering to -255 to 255
  int steering = map(constrain(steeringWidth, RC_MIN, RC_MAX), RC_MIN, RC_MAX, -255, 255);
  if (abs(steering) < RC_DEADBAND) steering = 0;
  
  // Tank-style steering: differential control
  leftSpeed = throttle + steering;
  rightSpeed = throttle - steering;
  backLeftSpeed = throttle + steering;
  backRightSpeed = throttle - steering;
  
  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  backLeftSpeed = constrain(backLeftSpeed, -255, 255);
  backRightSpeed = constrain(backRightSpeed, -255, 255);
  
  // AUX1 for speed limiting if needed
  int speedLimit = map(constrain(aux1Width, RC_MIN, RC_MAX), RC_MIN, RC_MAX, 50, 100);
  
  // Apply speed limit percentage
  leftSpeed = leftSpeed * speedLimit / 100;
  rightSpeed = rightSpeed * speedLimit / 100;
  backLeftSpeed = backLeftSpeed * speedLimit / 100;
  backRightSpeed = backRightSpeed * speedLimit / 100;
  
  // AUX2 controls LED mode
  ledMode = map(constrain(aux2Width, RC_MIN, RC_MAX), RC_MIN, RC_MAX, 0, 4);
}

// Apply calculated speeds to motors
void controlMotors() {
  // Front Left Motor
  if (leftSpeed > 0) {
    analogWrite(MOTOR_L_FWD, leftSpeed);
    analogWrite(MOTOR_L_REV, 0);
  } else {
    analogWrite(MOTOR_L_FWD, 0);
    analogWrite(MOTOR_L_REV, -leftSpeed);
  }
  
  // Front Right Motor
  if (rightSpeed > 0) {
    analogWrite(MOTOR_R_FWD, rightSpeed);
    analogWrite(MOTOR_R_REV, 0);
  } else {
    analogWrite(MOTOR_R_FWD, 0);
    analogWrite(MOTOR_R_REV, -rightSpeed);
  }
  
  // Back Left Motor
  if (backLeftSpeed > 0) {
    analogWrite(MOTOR_BL_FWD, backLeftSpeed);
    analogWrite(MOTOR_BL_REV, 0);
  } else {
    analogWrite(MOTOR_BL_FWD, 0);
    analogWrite(MOTOR_BL_REV, -backLeftSpeed);
  }
  
  // Back Right Motor
  if (backRightSpeed > 0) {
    analogWrite(MOTOR_BR_FWD, backRightSpeed);
    analogWrite(MOTOR_BR_REV, 0);
  } else {
    analogWrite(MOTOR_BR_FWD, 0);
    analogWrite(MOTOR_BR_REV, -backRightSpeed);
  }
}

// Stop all motors immediately
void stopAllMotors() {
  digitalWrite(MOTOR_L_FWD, LOW);
  digitalWrite(MOTOR_L_REV, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);
  digitalWrite(MOTOR_R_REV, LOW);
  digitalWrite(MOTOR_BL_FWD, LOW);
  digitalWrite(MOTOR_BL_REV, LOW);
  digitalWrite(MOTOR_BR_FWD, LOW);
  digitalWrite(MOTOR_BR_REV, LOW);
}

// Update LED effects based on current mode
void updateLEDs() {
  switch (ledMode) {
    case 0: // All off
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      break;
      
    case 1: // Speed indicator - color based on throttle
      {
        int speed = max(abs(leftSpeed), abs(rightSpeed));
        CRGB color = CRGB(speed, 255-speed, 0);
        fill_solid(leds, NUM_LEDS, color);
      }
      break;
      
    case 2: // Rainbow effect
      {
        static uint8_t startIndex = 0;
        startIndex = startIndex + 1;
        fill_rainbow(leds, NUM_LEDS, startIndex, 255 / NUM_LEDS);
      }
      break;
      
    case 3: // Direction indicators
      {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        if (throttleWidth > RC_MID + RC_DEADBAND) {
          // Forward - Blue front LEDs
          leds[0] = CRGB::Blue;
          leds[1] = CRGB::Blue;
          leds[11] = CRGB::Blue;
        } else if (throttleWidth < RC_MID - RC_DEADBAND) {
          // Reverse - Red back LEDs
          leds[5] = CRGB::Red;
          leds[6] = CRGB::Red;
        }
        
        if (steeringWidth > RC_MID + RC_DEADBAND) {
          // Right turn - Yellow right LEDs
          leds[2] = CRGB::Yellow;
          leds[3] = CRGB::Yellow;
        } else if (steeringWidth < RC_MID - RC_DEADBAND) {
          // Left turn - Yellow left LEDs
          leds[9] = CRGB::Yellow;
          leds[10] = CRGB::Yellow;
        }
      }
      break;
      
    case 4: // Chase effect
      {
        static uint8_t pos = 0;
        if (millis() - lastLedUpdate > 50) {
          lastLedUpdate = millis();
          for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
          }
          leds[pos] = CRGB::Purple;
          pos = (pos + 1) % NUM_LEDS;
        }
      }
      break;
  }
  
  FastLED.show();
}

// Blink all LEDs with specified color (for alerts)
void blinkLEDs(CRGB color) {
  static boolean ledState = false;
  static unsigned long lastBlinkTime = 0;
  
  if (millis() - lastBlinkTime > 200) {
    lastBlinkTime = millis();
    ledState = !ledState;
    
    fill_solid(leds, NUM_LEDS, ledState ? color : CRGB::Black);
    FastLED.show();
  }
}

// Print debug information to Serial
void printDebugInfo() {
  Serial.println("RC Values: ");
  Serial.print("Throttle: "); Serial.print(throttleWidth);
  Serial.print(" Steering: "); Serial.print(steeringWidth);
  Serial.print(" AUX1: "); Serial.print(aux1Width);
  Serial.print(" AUX2: "); Serial.print(aux2Width);
  Serial.print(" AUX3: "); Serial.println(aux3Width);
  
  Serial.println("Motor Values: ");
  Serial.print("Left: "); Serial.print(leftSpeed);
  Serial.print(" Right: "); Serial.print(rightSpeed);
  Serial.print(" Back L: "); Serial.print(backLeftSpeed);
  Serial.print(" Back R: "); Serial.println(backRightSpeed);
  
  Serial.print("LED Mode: "); Serial.println(ledMode);
  Serial.println();
}
