#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

/* ================= SYSTEM CONFIG ================= */
#define IBUS_RX_PIN       13
#define PIXEL_PIN         8
#define NUM_PIXELS        48
#define SERVO_FREQ        50
#define SERVO_PERIOD_US   20000
#define SERVO_COUNT       4        // BASE, ELBOW, ARM, TIP only
#define PWM_RESOLUTION    14
#define PWM_MAX_DUTY      16383.0f
#define FAILSAFE_MS       50
#define INPUT_DEADBAND    10
#define MOTOR_DEADBAND    0.03f    // 3% dead zone prevents motor hum at center

/* ---- STARTUP SAFETY — prevents current spike on power-on ---- */
#define STARTUP_RAMP_SPEED   15.0f  // deg/sec during initial correction (very slow)
#define STARTUP_STAGGER_MS   300    // ms delay between each servo powering on
#define IBUS_WAIT_MS         2000   // max ms to wait for receiver before fallback
#define NVS_SAVE_INTERVAL_MS 3000   // save positions to flash every 3 seconds

/* ================= SERVO TYPES (needed by NVS & ISR) ================= */
enum class ServoType { BASE, ARM };
enum class BaseState { MOVING, SETTLING, DETACHED };

struct ServoAxis {
  uint8_t pwmCh, gpio;
  ServoType type;
  float cur, tgt, speed;
  float minD, maxD;
  uint16_t minUs, maxUs;
  bool enabled;
  BaseState bState;
  uint32_t settleStart;
  bool startupDone;       // true once initial ramp finished
  float startupSpeed;     // slow speed used only during power-on correction
};

// Array defined below after hardware objects — declared here for NVS functions
extern ServoAxis servos[SERVO_COUNT];

/* ================= HARDWARE OBJECTS (before first function for Arduino prototypes) ================= */
struct BTS { uint8_t lpwm, rpwm, len, ren; };
BTS motorA = {15, 16, 17, 18};
BTS motorB = {9, 36, 11, 12};  // rpwm was 10 (damaged), now 37

/* ================= NVS POSITION MEMORY ================= */
// Saves servo positions to ESP32 flash so on next boot
// the code knows WHERE the servos physically are.
// Without this, a displaced arm causes a violent current-spike jump.
//
// NVS flash has ~100k write cycles per sector. At 3s interval
// that's ~300k seconds = 83 hours continuous before wear concern.
// We only write when positions actually change (>1° movement)
// which greatly extends flash life in practice.
Preferences prefs;
const char* servoKeys[SERVO_COUNT] = {"s0", "s1", "s2", "s3"};
uint32_t lastNvsSave = 0;
bool nvsPositionsDirty = false;
float lastSavedPos[SERVO_COUNT] = {90, 90, 90, 90};

void loadPositionsFromNVS() {
  prefs.begin("servo", true); // read-only
  for (int i = 0; i < SERVO_COUNT; i++) {
    float saved = prefs.getFloat(servoKeys[i], -1.0f);
    if (saved >= 0.0f && saved <= 180.0f) {
      servos[i].cur = saved;
      servos[i].tgt = saved;
      lastSavedPos[i] = saved;
      Serial.printf("NVS: Servo %d loaded %.1f deg\n", i, saved);
    } else {
      Serial.printf("NVS: Servo %d no saved pos, default %.1f deg\n", i, servos[i].cur);
    }
  }
  prefs.end();
}

void savePositionsToNVS() {
  prefs.begin("servo", false); // read-write
  for (int i = 0; i < SERVO_COUNT; i++) {
    prefs.putFloat(servoKeys[i], servos[i].cur);
    lastSavedPos[i] = servos[i].cur;
  }
  prefs.end();
  nvsPositionsDirty = false;
}

// Mark dirty only when positions actually moved significantly (>1°)
void nvsCheckDirty() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (fabs(servos[i].cur - lastSavedPos[i]) > 1.0f) {
      nvsPositionsDirty = true;
      return;
    }
  }
}

// Call periodically — only writes if positions changed and interval elapsed
void nvsSaveIfNeeded() {
  if (nvsPositionsDirty && (millis() - lastNvsSave >= NVS_SAVE_INTERVAL_MS)) {
    savePositionsToNVS();
    lastNvsSave = millis();
  }
}

/* ================= HARDWARE OBJECTS ================= */
Adafruit_NeoPixel strip(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ================= ROBUST IBUS ================= */
class RobustIBUS {
  HardwareSerial* s;
  uint16_t ch[14];
  uint32_t lastFrame;
  bool lost;
  enum { SYNC, LEN, DATA, CSL, CSH } state;
  uint8_t buf[32], idx;
  uint16_t calc, rx_cs;

public:
  RobustIBUS() : lastFrame(0), lost(true), state(SYNC) {
    for (int i = 0; i < 14; i++) ch[i] = 1500;
  }
  void begin(HardwareSerial& ser, int rxPin) {
    s = &ser; s->begin(115200, SERIAL_8N1, rxPin, -1);
  }
  void handle() {
    while (s->available()) process(s->read());
    if (millis() - lastFrame > FAILSAFE_MS) lost = true;
  }
  uint16_t read(uint8_t c) {
    uint16_t val = (lost || c >= 14) ? 1500 : ch[c];
    if (abs((int)val - 1500) < INPUT_DEADBAND) return 1500;
    return val;
  }
  bool isSafe() { return !lost; }
private:
  void process(uint8_t b) {
    switch (state) {
      case SYNC: if (b == 0x20) { calc = 0xFFFF - 0x20; state = LEN; } break;
      case LEN:  if (b == 0x40) { calc -= 0x40; idx = 0; state = DATA; } else state = SYNC; break;
      case DATA: if (idx < 28) { buf[idx++] = b; calc -= b; } if (idx >= 28) state = CSL; break;
      case CSL:  rx_cs = b; state = CSH; break;
      case CSH:  rx_cs |= (b << 8);
                 if (calc == rx_cs) {
                   for (int i = 0; i < 14; i++) ch[i] = buf[i * 2] | (buf[i * 2 + 1] << 8);
                   lastFrame = millis(); lost = false;
                 }
                 state = SYNC; break;
    }
  }
};
RobustIBUS rx;

/* ================= SERVO ENGINE ================= */
enum class StartupPhase { WAIT_IBUS, STAGGER_ON, RAMPING, READY };

// 4 servos: BASE(0), ELBOW(1), ARM(2), TIP(3)
// ALL start enabled=false, startupDone=false
// TIP gets the slowest startup speed (10 deg/s) since it carries payload
ServoAxis servos[SERVO_COUNT] = {
  { 4,  4, ServoType::BASE, 90, 90, 35,   0, 180, 500, 2500, false, BaseState::DETACHED, 0, false, STARTUP_RAMP_SPEED },
  { 5,  5, ServoType::ARM,  90, 90, 40,  20, 160, 500, 2500, false, BaseState::MOVING,   0, false, STARTUP_RAMP_SPEED },
  { 6,  6, ServoType::ARM,  90, 90, 40,  30, 170, 500, 2500, false, BaseState::MOVING,   0, false, STARTUP_RAMP_SPEED },
  { 7,  7, ServoType::ARM,  90, 90, 25,  40, 140, 500, 2500, false, BaseState::MOVING,   0, false, 10.0f },
};

hw_timer_t* timer = nullptr;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Startup state (accessed from loop only, no ISR access needed)
StartupPhase startupPhase = StartupPhase::WAIT_IBUS;
uint8_t staggerIndex = 0;
uint32_t staggerTimer = 0;
uint32_t ibusWaitStart = 0;

uint32_t degToDuty(const ServoAxis& s, float d) {
  d = constrain(d, s.minD, s.maxD);
  float us = s.minUs + ((d - s.minD) / (s.maxD - s.minD)) * (s.maxUs - s.minUs);
  return (uint32_t)((us * PWM_MAX_DUTY) / SERVO_PERIOD_US);
}

/* ================= TIMER ISR — SMOOTH RAMP MOTION ================= */
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&mux);
  static uint32_t lastUs = 0;
  uint32_t now = micros();
  float dt = (lastUs == 0) ? 0.02f : (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.02f;

  for (int i = 0; i < SERVO_COUNT; i++) {
    ServoAxis& s = servos[i];
    if (!s.enabled) continue;

    float diff = s.tgt - s.cur;
    // Use slow startup speed until initial correction is done
    float spd = s.startupDone ? s.speed : s.startupSpeed;
    float step = spd * dt;

    if (s.type == ServoType::BASE) {
      if (s.bState == BaseState::MOVING) {
        if (fabs(diff) > 0.2f) {
          s.cur += constrain(diff, -step, step);
          ledcWrite(s.pwmCh, degToDuty(s, s.cur));
        } else {
          s.bState = BaseState::SETTLING;
          s.settleStart = now; // raw microseconds — no division in ISR
        }
      } else if (s.bState == BaseState::SETTLING) {
        if (now - s.settleStart >= 200000) { // 200ms in µs
          ledcWrite(s.pwmCh, 0);
          s.bState = BaseState::DETACHED;
        }
      }
    } else {
      if (fabs(diff) > 0.1f) {
        s.cur += constrain(diff, -step, step);
        ledcWrite(s.pwmCh, degToDuty(s, s.cur));
      }
    }

    // Mark startup done when servo reaches its initial target
    if (!s.startupDone && fabs(diff) < 0.5f) {
      s.startupDone = true;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

/* ================= MOTOR DRIVE (MUTEX PROTECTED) ================= */
void drive(BTS& m, int spd, uint8_t chL, uint8_t chR) {
  spd = constrain(spd, -255, 255);
  portENTER_CRITICAL(&mux);
  if (spd == 0) {
    ledcWrite(chL, 0); ledcWrite(chR, 0);
  } else if (spd > 0) {
    ledcWrite(chR, 0); ledcWrite(chL, spd);
  } else {
    ledcWrite(chL, 0); ledcWrite(chR, -spd);
  }
  portEXIT_CRITICAL(&mux);
}

/* ================= VISUALS ================= */
void updateLEDs(float yaw, int mode, bool kill) {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();

  strip.clear();
  if (startupPhase != StartupPhase::READY) {
    // Startup: slow blue pulse so operator knows system is initializing
    uint8_t b = (uint8_t)(50.0f + 50.0f * sin(millis() / 300.0f));
    for (int i = 0; i < 48; i++) strip.setPixelColor(i, strip.Color(0, 0, b));
  } else if (!rx.isSafe()) {
    uint8_t r = (uint8_t)(100.0f + sin(millis() / 200.0f) * 100.0f);
    for (int i = 0; i < 48; i++) strip.setPixelColor(i, strip.Color(r, 0, 0));
  } else if (kill) {
    for (int i = 0; i < 48; i++) strip.setPixelColor(i, strip.Color(50, 0, 0));
  } else {
    uint32_t modeCol = (mode == 1) ? strip.Color(0, 100, 100) :
                        (mode == 2) ? strip.Color(100, 0, 100) :
                                      strip.Color(0, 100, 0);
    for (int i = 16; i < 32; i++) strip.setPixelColor(i, strip.Color(150, 150, 150));
    if (yaw > 0.1f) for (int i = 0; i < 16; i++) strip.setPixelColor(i, strip.Color(255, 60, 0));
    else if (yaw < -0.1f) for (int i = 32; i < 48; i++) strip.setPixelColor(i, strip.Color(255, 60, 0));
    strip.setPixelColor(0, modeCol); strip.setPixelColor(47, modeCol);
  }
  strip.show();
}

/* ================= STARTUP STATE MACHINE ================= */
void handleStartup() {
  switch (startupPhase) {

    case StartupPhase::WAIT_IBUS:
      if (rx.isSafe()) {
        // Receiver is live — set TARGETS from current stick positions
        // cur was already loaded from NVS (= last known physical position)
        // So: cur ≈ physical position, tgt = where sticks want to go
        // The ISR will SLOWLY ramp from cur → tgt at startup speed
        float valA = (rx.read(4) - 1000.0f) / 1000.0f * 180.0f;
        float valB = (rx.read(5) - 1000.0f) / 1000.0f * 180.0f;
        valA = constrain(valA, 0.0f, 180.0f);
        valB = constrain(valB, 0.0f, 180.0f);
        int mode = rx.read(8) < 1300 ? 1 : rx.read(8) < 1700 ? 2 : 3;

        portENTER_CRITICAL(&mux);
        // Set ONLY targets from sticks — cur stays at NVS-loaded position
        // This means ISR will slowly ramp from saved position to stick target
        if (mode == 1) {
          servos[0].tgt = valA;
          servos[3].tgt = constrain(40.0f + (valB / 180.0f) * 100.0f, 40.0f, 140.0f);
        } else if (mode == 2) {
          servos[1].tgt = constrain(20.0f + (valA / 180.0f) * 140.0f, 20.0f, 160.0f);
          servos[2].tgt = constrain(30.0f + (valB / 180.0f) * 140.0f, 30.0f, 170.0f);
        }
        // Servos not in the active mode: tgt = cur (NVS value), so zero movement
        portEXIT_CRITICAL(&mux);

        staggerIndex = 0;
        staggerTimer = millis();
        startupPhase = StartupPhase::STAGGER_ON;
        Serial.println("IBUS OK — staggered servo power-on starting");

      } else if (millis() - ibusWaitStart > IBUS_WAIT_MS) {
        // No receiver — power on at NVS positions (cur = tgt = saved)
        // Servos won't move, they just hold where they are
        staggerIndex = 0;
        staggerTimer = millis();
        startupPhase = StartupPhase::STAGGER_ON;
        Serial.println("WARN: No IBUS — powering servos at last saved positions");
      }
      break;

    case StartupPhase::STAGGER_ON:
      // Power on ONE servo at a time with 300ms gap to spread current draw
      if (millis() - staggerTimer >= STARTUP_STAGGER_MS) {
        if (staggerIndex < SERVO_COUNT) {
          portENTER_CRITICAL(&mux);
          ServoAxis& s = servos[staggerIndex];
          s.enabled = true;
          if (s.type == ServoType::BASE) s.bState = BaseState::MOVING;
          // Write the servo's current position first — no jump
          ledcWrite(s.pwmCh, degToDuty(s, s.cur));
          portEXIT_CRITICAL(&mux);

          Serial.printf("Servo %d ON (cur=%.1f tgt=%.1f)\n", staggerIndex, s.cur, s.tgt);
          staggerIndex++;
          staggerTimer = millis();
        } else {
          startupPhase = StartupPhase::RAMPING;
          staggerTimer = millis();
          Serial.println("All servos on — ramping at safe speed");
        }
      }
      break;

    case StartupPhase::RAMPING: {
      // Wait for all servos to finish their slow initial correction
      bool allDone = true;
      for (int i = 0; i < SERVO_COUNT; i++) {
        if (!servos[i].startupDone) { allDone = false; break; }
      }
      if (allDone || (millis() - staggerTimer > 5000)) {
        portENTER_CRITICAL(&mux);
        for (int i = 0; i < SERVO_COUNT; i++) servos[i].startupDone = true;
        portEXIT_CRITICAL(&mux);
        startupPhase = StartupPhase::READY;
        Serial.println("SYSTEM READY — full speed control active");
      }
      break;
    }

    case StartupPhase::READY:
      break;
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  // 1. ALL pins LOW immediately — no garbage pulses on boot
  uint8_t motorPins[] = {motorA.lpwm, motorA.rpwm, motorB.lpwm, motorB.rpwm,
                         motorA.len, motorA.ren, motorB.len, motorB.ren};
  for (uint8_t p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  for (int i = 0; i < SERVO_COUNT; i++) { pinMode(servos[i].gpio, OUTPUT); digitalWrite(servos[i].gpio, LOW); }

  // 2. Motor LEDC channels (0-3)
  ledcSetup(0, 20000, 8); ledcAttachPin(motorA.lpwm, 0);
  ledcSetup(1, 20000, 8); ledcAttachPin(motorA.rpwm, 1);
  ledcSetup(2, 20000, 8); ledcAttachPin(motorB.lpwm, 2);
  ledcSetup(3, 20000, 8); ledcAttachPin(motorB.rpwm, 3);

  // 3. Servo LEDC channels — setup but ZERO duty (servos unpowered at boot!)
  for (auto& s : servos) {
    ledcSetup(s.pwmCh, SERVO_FREQ, PWM_RESOLUTION);
    ledcAttachPin(s.gpio, s.pwmCh);
    ledcWrite(s.pwmCh, 0);   // NO PWM = servo completely unpowered
    s.enabled = false;        // ISR won't touch this servo
  }

  // 3b. Load last-known servo positions from flash
  //     This is THE KEY to safe startup: cur = last physical position
  //     So when we power a servo on, the first PWM matches where it IS
  loadPositionsFromNVS();

  // 4. NeoPixel
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  // 5. Timer ISR for smooth servo motion
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, SERVO_PERIOD_US, true);
  timerAlarmEnable(timer);

  // 6. iBUS receiver
  rx.begin(Serial1, IBUS_RX_PIN);

  // 7. Enable BTS drivers AFTER all LEDC channels are zeroed
  digitalWrite(motorA.len, HIGH); digitalWrite(motorA.ren, HIGH);
  digitalWrite(motorB.len, HIGH); digitalWrite(motorB.ren, HIGH);

  // 8. Record startup time
  ibusWaitStart = millis();
  Serial.println("BOOT: Waiting for iBUS before arming servos...");
}

/* ================= LOOP ================= */
void loop() {
  rx.handle();

  // Startup sequence — motors disabled, servos powering on safely
  if (startupPhase != StartupPhase::READY) {
    handleStartup();
    drive(motorA, 0, 0, 1);
    drive(motorB, 0, 2, 3);
    updateLEDs(0, 0, false);
    return;
  }

  // ================= NORMAL OPERATION =================
  bool kill   = rx.read(9) > 1700;
  bool clutch = rx.read(6) > 1700;
  int mode    = rx.read(8) < 1300 ? 1 : rx.read(8) < 1700 ? 2 : 3;

  static bool wasDead = false;  // track kill/failsafe transitions

  if (kill || !rx.isSafe()) {
    // KILL: stop motors, disable servos but preserve position
    drive(motorA, 0, 0, 1);
    drive(motorB, 0, 2, 3);
    portENTER_CRITICAL(&mux);
    for (auto& s : servos) s.enabled = false;
    portEXIT_CRITICAL(&mux);
    // Save positions ONCE on kill entry — not every loop iteration
    if (!wasDead) { savePositionsToNVS(); wasDead = true; }

  } else {
    wasDead = false;
    // ---- MOTORS (tank drive with proper deadband + normalized mixing) ----
    float fwd   = map(rx.read(1), 1000, 2000, -1000, 1000) / 1000.0f;
    float yaw   = map(rx.read(0), 1000, 2000, -1000, 1000) / 1000.0f;
    float limit = map(rx.read(2), 1000, 2000, 0, 1000) / 1000.0f;

    if (fabs(fwd) < MOTOR_DEADBAND) fwd = 0;
    if (fabs(yaw) < MOTOR_DEADBAND) yaw = 0;

    // Normalize mix so full stick + full turn doesn't clip at 255
    float left  = fwd + yaw;
    float right = fwd - yaw;
    float maxMag = fmax(fabs(left), fabs(right));
    if (maxMag > 1.0f) { left /= maxMag; right /= maxMag; }

    drive(motorA, (int)(left  * limit * 255.0f), 0, 1);
    drive(motorB, (int)(right * limit * 255.0f), 2, 3);

    // ---- SERVOS ----
    portENTER_CRITICAL(&mux);
    for (auto& s : servos) s.enabled = true;
    portEXIT_CRITICAL(&mux);

    if (!clutch) {
      // Float math for smooth fractional-degree targets (no integer map())
      float raw4 = rx.read(4);
      float raw5 = rx.read(5);
      float valA = constrain((raw4 - 1000.0f) / 1000.0f * 180.0f, 0.0f, 180.0f);
      float valB = constrain((raw5 - 1000.0f) / 1000.0f * 180.0f, 0.0f, 180.0f);

      portENTER_CRITICAL(&mux);
      if (mode == 1) {
        // VR-A → Base rotation, VR-B → Tip
        if (fabs(valA - servos[0].tgt) > 1.5f) {
          servos[0].tgt = valA;
          if (servos[0].bState == BaseState::DETACHED) servos[0].bState = BaseState::MOVING;
        }
        servos[3].tgt = constrain(40.0f + (valB / 180.0f) * 100.0f, 40.0f, 140.0f);
      } else if (mode == 2) {
        // VR-A → Elbow, VR-B → Arm
        servos[1].tgt = constrain(20.0f + (valA / 180.0f) * 140.0f, 20.0f, 160.0f);
        servos[2].tgt = constrain(30.0f + (valB / 180.0f) * 140.0f, 30.0f, 170.0f);
      }
      // Mode 3: reserved (no AUX servos)
      portEXIT_CRITICAL(&mux);
    }
  }

  // Periodically save servo positions to flash (every 3s if actually moved)
  nvsCheckDirty();
  nvsSaveIfNeeded();

  updateLEDs(map(rx.read(0), 1000, 2000, -1000, 1000) / 1000.0f, mode, kill);
}