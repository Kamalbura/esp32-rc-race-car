#include <Arduino.h>
#include <FastLED.h>

// Pin definitions
// RC receiver pins
#define CH1_PIN 36  // Steering (connect to RC receiver CH1)
#define CH3_PIN 34  // Throttle (connect to RC receiver CH3)
#define CH5_PIN 32  // AUX1 - Mode control (optional)

// BTS7960 driver pins (2 drivers, one for left motors, one for right)
#define LEFT_RPWM 25  // Left motors forward PWM
#define LEFT_LPWM 26  // Left motors reverse PWM
#define RIGHT_RPWM 27  // Right motors forward PWM
#define RIGHT_LPWM 18  // Right motors reverse PWM

// LED strip configuration
#define LED_PIN     13
#define NUM_LEDS    12
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// RC signal variables
unsigned long ch1_start = 0;
unsigned long ch3_start = 0;
unsigned long ch5_start = 0;
int ch1_pwm = 1500;  // steering (neutral value)
int ch3_pwm = 1500;  // throttle (neutral value)
int ch5_pwm = 1500;  // aux channel (neutral value)
volatile bool ch1_signal_present = false;
volatile bool ch3_signal_present = false;
volatile bool ch5_signal_present = false;
unsigned long last_signal_time = 0;
const unsigned long FAILSAFE_DELAY = 500; // failsafe timeout in ms

// Motor control variables
int leftSpeed = 0;   // Range: -255 to 255
int rightSpeed = 0;  // Range: -255 to 255
int prevLeftSpeed = 0;
int prevRightSpeed = 0;
int currentDirection = 0; // 0 = stop, 1 = forward, -1 = reverse
unsigned long lastDirectionChange = 0;
#define DIRECTION_CHANGE_DELAY 300 // ms to wait before changing direction

// PWM setup
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
const int LEFT_MOTOR_FORWARD_CHANNEL = 0;
const int LEFT_MOTOR_REVERSE_CHANNEL = 1;
const int RIGHT_MOTOR_FORWARD_CHANNEL = 2;
const int RIGHT_MOTOR_REVERSE_CHANNEL = 3;

// RC Signal Constants
const int RC_MIN = 1000;  // minimum RC pulse width in µs
const int RC_MAX = 2000;  // maximum RC pulse width in µs
const int RC_MID = 1500;  // middle/neutral RC pulse width in µs
const int RC_DEADZONE = 50; // deadzone around mid position

// LED variables
int ledMode = 0;
unsigned long lastLedUpdate = 0;
bool failsafeActive = false;

// ISR for RC channel 1 (steering)
void IRAM_ATTR ch1_interrupt() {
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1_start = micros();
  } else {
    int pulseWidth = micros() - ch1_start;
    // Validate pulse width
    if (pulseWidth >= 800 && pulseWidth <= 2200) {
      ch1_pwm = pulseWidth;
      ch1_signal_present = true;
      last_signal_time = millis();
    }
  }
}

// ISR for RC channel 3 (throttle)
void IRAM_ATTR ch3_interrupt() {
  if (digitalRead(CH3_PIN) == HIGH) {
    ch3_start = micros();
  } else {
    int pulseWidth = micros() - ch3_start;
    // Validate pulse width
    if (pulseWidth >= 800 && pulseWidth <= 2200) {
      ch3_pwm = pulseWidth;
      ch3_signal_present = true;
      last_signal_time = millis();
    }
  }
}

// ISR for RC channel 5 (aux)
void IRAM_ATTR ch5_interrupt() {
  if (digitalRead(CH5_PIN) == HIGH) {
    ch5_start = micros();
  } else {
    int pulseWidth = micros() - ch5_start;
    // Validate pulse width
    if (pulseWidth >= 800 && pulseWidth <= 2200) {
      ch5_pwm = pulseWidth;
      ch5_signal_present = true;
      last_signal_time = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configure RC receiver pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  
  // Attach interrupts for RC signals
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), ch3_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5_PIN), ch5_interrupt, CHANGE);

  // Configure PWM for motor drivers
  ledcSetup(LEFT_MOTOR_FORWARD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LEFT_MOTOR_REVERSE_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_FORWARD_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_MOTOR_REVERSE_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach PWM channels to GPIO pins
  ledcAttachPin(LEFT_RPWM, LEFT_MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(LEFT_LPWM, LEFT_MOTOR_REVERSE_CHANNEL);
  ledcAttachPin(RIGHT_RPWM, RIGHT_MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(RIGHT_LPWM, RIGHT_MOTOR_REVERSE_CHANNEL);
  
  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  
  // Startup animation
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(40);
  }
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println("ESP32 RC Car System Initialized");
}

// Map RC value to motor speed with deadzone
int mapWithDeadzone(int value, int inMin, int inMid, int inMax, int outMin, int outMax) {
  // Apply deadzone around center
  if (abs(value - inMid) < RC_DEADZONE) {
    return 0;
  }
  
  if (value < inMid) {
    return map(value, inMin, inMid - RC_DEADZONE, outMin, 0);
  } else {
    return map(value, inMid + RC_DEADZONE, inMax, 0, outMax);
  }
}

// Set motor speed and direction with smooth acceleration/deceleration
void setMotorSpeed(int targetLeftSpeed, int targetRightSpeed) {
  // Constrain values
  targetLeftSpeed = constrain(targetLeftSpeed, -255, 255);
  targetRightSpeed = constrain(targetRightSpeed, -255, 255);
  
  // Determine current direction
  int newDirection = 0;
  if (targetLeftSpeed > 0 || targetRightSpeed > 0) newDirection = 1;      // forward
  else if (targetLeftSpeed < 0 || targetRightSpeed < 0) newDirection = -1; // reverse
  
  // Prevent sudden direction changes
  if (currentDirection != 0 && newDirection != 0 && 
      currentDirection != newDirection && 
      millis() - lastDirectionChange < DIRECTION_CHANGE_DELAY) {
    // Stop motors when trying to change direction too quickly
    ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, 0);
    ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, 0);
    return;
  }
  
  // If direction is changing, record the time
  if (currentDirection != newDirection) {
    lastDirectionChange = millis();
    currentDirection = newDirection;
    
    // Briefly stop motors before changing direction
    if (currentDirection != 0 && newDirection != 0 && currentDirection != newDirection) {
      ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, 0);
      ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, 0);
      ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, 0);
      ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, 0);
      delay(50); // Brief delay for motors to stop
    }
  }
  
  // Gradual acceleration/deceleration
  int actualLeftSpeed, actualRightSpeed;
  
  // Smooth changes to left motor speed
  if (targetLeftSpeed > prevLeftSpeed) {
    actualLeftSpeed = min(targetLeftSpeed, prevLeftSpeed + 10);
  } else if (targetLeftSpeed < prevLeftSpeed) {
    actualLeftSpeed = max(targetLeftSpeed, prevLeftSpeed - 10);
  } else {
    actualLeftSpeed = targetLeftSpeed;
  }
  
  // Smooth changes to right motor speed
  if (targetRightSpeed > prevRightSpeed) {
    actualRightSpeed = min(targetRightSpeed, prevRightSpeed + 10);
  } else if (targetRightSpeed < prevRightSpeed) {
    actualRightSpeed = max(targetRightSpeed, prevRightSpeed - 10);
  } else {
    actualRightSpeed = targetRightSpeed;
  }
  
  // Set left motors
  if (actualLeftSpeed >= 0) {
    ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, actualLeftSpeed);
    ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, 0);
  } else {
    ledcWrite(LEFT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(LEFT_MOTOR_REVERSE_CHANNEL, -actualLeftSpeed);
  }
  
  // Set right motors
  if (actualRightSpeed >= 0) {
    ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, actualRightSpeed);
    ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, 0);
  } else {
    ledcWrite(RIGHT_MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(RIGHT_MOTOR_REVERSE_CHANNEL, -actualRightSpeed);
  }
  
  // Save current speeds for next iteration
  prevLeftSpeed = actualLeftSpeed;
  prevRightSpeed = actualRightSpeed;
}

// Update LED effects based on current mode and vehicle state
void updateLEDs() {
  // Handle failsafe state
  if (failsafeActive) {
    static boolean ledState = false;
    static unsigned long lastBlinkTime = 0;
    
    if (millis() - lastBlinkTime > 150) {
      lastBlinkTime = millis();
      ledState = !ledState;
      
      fill_solid(leds, NUM_LEDS, ledState ? CRGB::Red : CRGB::Black);
      FastLED.show();
    }
    return;
  }
  
  // Determine LED mode based on AUX channel
  ledMode = map(constrain(ch5_pwm, RC_MIN, RC_MAX), RC_MIN, RC_MAX, 0, 4);
  
  switch (ledMode) {
    case 0: // All off
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      break;
      
    case 1: // Speed indicator - color based on throttle
      {
        int speed = max(abs(leftSpeed), abs(rightSpeed));
        CRGB color = CRGB(speed, 255-speed, 0); // Red to Green based on speed
        fill_solid(leds, NUM_LEDS, color);
      }
      break;
      
    case 2: // Rainbow effect
      {
        static uint8_t startIndex = 0;
        if (millis() - lastLedUpdate > 20) {
          lastLedUpdate = millis();
          startIndex = startIndex + 1;
          fill_rainbow(leds, NUM_LEDS, startIndex, 255 / NUM_LEDS);
        }
      }
      break;
      
    case 3: // Direction indicators
      {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        if (ch3_pwm > RC_MID + RC_DEADZONE) {
          // Forward - Blue front LEDs
          leds[0] = CRGB::Blue;
          leds[1] = CRGB::Blue;
          leds[11] = CRGB::Blue;
        } else if (ch3_pwm < RC_MID - RC_DEADZONE) {
          // Reverse - Red back LEDs
          leds[5] = CRGB::Red;
          leds[6] = CRGB::Red;
        }
        
        if (ch1_pwm > RC_MID + RC_DEADZONE) {
          // Right turn - Yellow right LEDs
          leds[2] = CRGB::Yellow;
          leds[3] = CRGB::Yellow;
        } else if (ch1_pwm < RC_MID - RC_DEADZONE) {
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

void loop() {
  // Check if signal is present (failsafe)
  if (millis() - last_signal_time > FAILSAFE_DELAY) {
    // Signal lost - activate failsafe
    setMotorSpeed(0, 0);
    failsafeActive = true;
    Serial.println("Failsafe activated - stopping motors");
    updateLEDs();
    delay(50);
    return;
  } else {
    failsafeActive = false;
  }
  
  // Validate RC signals
  ch1_pwm = constrain(ch1_pwm, RC_MIN, RC_MAX);
  ch3_pwm = constrain(ch3_pwm, RC_MIN, RC_MAX);
  
  // Calculate base throttle (-255 to 255)
  int throttle = mapWithDeadzone(ch3_pwm, RC_MIN, RC_MID, RC_MAX, -255, 255);
  
  // Calculate steering (-100% to 100%)
  float steering = map(ch1_pwm, RC_MIN, RC_MAX, -100, 100) / 100.0;
  
  // Apply enhanced differential steering logic
  if (throttle > 0) { // Moving forward
    if (steering > 0) { // Turning right
      leftSpeed = throttle;
      rightSpeed = throttle * (1 - abs(steering) * 0.9); // Slightly stronger turning effect
    } else { // Turning left
      leftSpeed = throttle * (1 - abs(steering) * 0.9);
      rightSpeed = throttle;
    }
  } else if (throttle < 0) { // Moving in reverse
    if (steering > 0) { // Turning right
      leftSpeed = throttle;
      rightSpeed = throttle * (1 - abs(steering) * 0.9);
    } else { // Turning left
      leftSpeed = throttle * (1 - abs(steering) * 0.9);
      rightSpeed = throttle;
    }
  } else { // No throttle input
    // Spin in place if only steering is provided with enough force
    if (abs(steering) > 0.3) {
      leftSpeed = 150 * steering;
      rightSpeed = -150 * steering;
    } else {
      leftSpeed = 0;
      rightSpeed = 0;
    }
  }
  
  // Apply motor speeds with smooth acceleration/deceleration
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Update LEDs based on vehicle state
  updateLEDs();
  
  // Debug output (every 250ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    Serial.print("CH1 (Steering): ");
    Serial.print(ch1_pwm);
    Serial.print(" | CH3 (Throttle): ");
    Serial.print(ch3_pwm);
    Serial.print(" | CH5 (AUX): ");
    Serial.print(ch5_pwm);
    Serial.print(" | Left Motors: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Motors: ");
    Serial.println(rightSpeed);
    
    lastPrint = millis();
  }
  
  delay(10); // Small delay for stability
}
