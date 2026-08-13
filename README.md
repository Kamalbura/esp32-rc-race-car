# ESP32-S3 RC Race Car (Cyber Truck V2)

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)
[![ESP32-S3](https://img.shields.io/badge/ESP32--S3-×2-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-009D93.svg)](https://www.arduino.cc/)
[![ESP-NOW](https://img.shields.io/badge/ESP--NOW-encrypted%20@%20100Hz-orange.svg)](docs/PROTOCOL.md)

![RC Race Car](./media/test-ride-thumb.jpg)

## 📌 Overview

The second generation of the Cyber Truck build. Same chassis, rebuilt drivetrain and a completely new radio system: instead of reading a commercial RC receiver, this version uses a **purpose-built handheld transmitter** talking to the car over encrypted ESP-NOW. The project demonstrates embedded systems programming, custom radio protocol design, closed-loop motor control, IMU sensor fusion, and real-time dual-core scheduling.

**Watch our project in action:**
[![Test Drive Video](./media/test-ride-thumb.jpg)](./media/test-ride.mp4)

## 🔄 What Changed From V1

| Subsystem | V1 — Cyber Truck | V2 — Race Car |
|-----------|------------------|---------------|
| Radio link | Commercial 6-channel RC receiver | Custom encrypted ESP-NOW peer link |
| Controller | ESP32 dev board | ESP32-S3 ×2 (transmitter + receiver) |
| Motors | Johnson 1000RPM (×4) | 555 geared 500RPM |
| Battery | 7.4V–11.1V LiPo | Upgraded 3S pack |
| Telemetry | None | Bidirectional with ACK + sequence counters |
| Failsafe | Receiver signal loss | 120 ms link timeout, CRC-validated |
| Stabilisation | None | MPU6050 yaw-rate assist |
| Steering | Fixed differential | Speed-dependent authority with slew limiting |

V1 firmware is preserved under [`v1-cyber-truck/`](v1-cyber-truck/).

### Why the motors got slower

V1 ran four Johnson 1000RPM motors. On paper the faster setup — in practice the wrong figure to optimise. What reaches the ground is **torque at the wheel**, not motor RPM. Traction is limited by the tyre, so past the point where it breaks loose extra RPM becomes wheelspin instead of acceleration, and a heavier pack raises the torque needed to move off from rest, which is exactly where a high-RPM low-torque motor stalls.

V2 runs **555-frame geared motors at 500RPM** — half the free speed, considerably more torque per wheel, chosen by working the torque budget per tyre. That decision has a fingerprint in the firmware: `STALL_GUARD_MIN_PWM = 55` exists because these motors will not reliably start under load below roughly 20% duty.

## 🎮 Key Features

- **Custom Radio Link**: Encrypted ESP-NOW unicast between two fixed MACs at 100 Hz, replacing the commercial receiver entirely
- **Packet Integrity**: CRC-16/CCITT on every frame — a corrupt packet is treated as silence and feeds the failsafe rather than glitching the controls
- **Bidirectional Telemetry**: The car reports battery voltage, effective motor PWM and link state back to the transmitter, with sequence acknowledgement for packet-loss measurement
- **Speed-Dependent Steering**: Steering authority scales from 1.10 at low speed to 0.45 at high speed, with asymmetric slew limiting so turn-in is smoothed but recovery stays quick
- **Gyroscopic Assist**: MPU6050 yaw-rate feedback with startup offset calibration and an automated turn test
- **Dual-Core Scheduling**: Control loop and SPI display rendering split across both ESP32-S3 cores so screen updates never disturb the 100 Hz send interval
- **Failsafe Protection**: 120 ms link timeout stops the motors immediately, bypassing the acceleration ramp
- **Guided Calibration**: On-device and host-driven workflows for measuring axis endpoints, centres and deadbands
- **Modular Architecture**: Separate sketches per role, with per-subsystem test sketches

## 👥 Team

This project was created by:

| Team Member | Role | Contribution |
|-------------|------|--------------|
| **Kamal Bura** | Hardware Engineer | V2 electronics, firmware and radio protocol; V1 hardware design and integration |
| **Vighnesh** | Software Developer | V1 control algorithms and motor driver interface |
| **Karthekeya** | Design Engineer | V1 chassis design and LED system implementation |

## 🔧 Hardware Components

### Transmitter

| Component | Specification | Purpose |
|-----------|---------------|---------|
| Microcontroller | ESP32-S3 | Handheld controller unit |
| ADC | ADS1115 (16-bit, I²C) | Analogue control axes |
| Display | ST7735 128×128 SPI | Live telemetry and status |
| Input | Rotary encoder with switch | Speed limit and mode gestures |
| Status LED | Onboard WS2812 (GPIO48) | Link state indication |
| Storage | NVS via Preferences | Persistent calibration |

### Receiver (on the car)

| Component | Specification | Purpose |
|-----------|---------------|---------|
| Microcontroller | ESP32-S3 | Car control unit |
| Motor Drivers | BTS7960 H-Bridge (×2) | High-current motor control |
| Motors | 555 geared, 500RPM | Vehicle propulsion |
| IMU | MPU6050 (I²C) | Yaw-rate assist |
| Power Supply | 3S LiPo via XT60 | System power |
| LED System | WS2812B Addressable RGB | Visual feedback |
| Chassis | Custom Design (shared with V1) | Vehicle structure |

## 🔌 Wiring Diagram

### Transmitter — ESP32-S3

#### ADS1115 (control axes)
- SDA → ESP32-S3 pin 8
- SCL → ESP32-S3 pin 9 (400 kHz)
- ADDR → GND (address `0x48`)

#### ST7735 Display
- SCK → ESP32-S3 pin 12
- MOSI → ESP32-S3 pin 11
- MISO → ESP32-S3 pin 13 (reserved)
- CS → ESP32-S3 pin 15
- DC → ESP32-S3 pin 16
- RST → ESP32-S3 pin 17

#### Rotary Encoder
- A/CLK → ESP32-S3 pin 4
- B/DT → ESP32-S3 pin 5
- SW → ESP32-S3 pin 6
- BOOT fallback button → ESP32-S3 pin 0

### Receiver — ESP32-S3

#### Left Motors Driver (BTS7960)
- LPWM → ESP32-S3 pin 15 (LEDC channel 0)
- RPWM → ESP32-S3 pin 16 (LEDC channel 1)
- L_EN → ESP32-S3 pin 17
- R_EN → ESP32-S3 pin 18

#### Right Motors Driver (BTS7960)
- LPWM → ESP32-S3 pin 9 (LEDC channel 2)
- RPWM → ESP32-S3 pin 10 (LEDC channel 3)
- L_EN → ESP32-S3 pin 11
- R_EN → ESP32-S3 pin 12

#### MPU6050
- SDA → ESP32-S3 pin 1
- SCL → ESP32-S3 pin 2
- Mounting: +X forward, +Y left, +Z up (positive gyro Z is a left turn)

![Chassis internals](./media/hardware.jpg)

## 📊 System Architecture & Signal Flow

### Core Components Interaction
```
┌──────────────────────────────────────┐        ┌──────────────────────────────────────┐
│   TRANSMITTER (ESP32-S3)             │        │   RECEIVER (ESP32-S3, on the car)    │
│                                      │        │                                      │
│  ┌────────────┐   ┌───────────────┐  │        │  ┌───────────────────────────────┐   │
│  │ ADS1115    │──▶│ Core 1        │  │        │  │ Packet Validation             │   │
│  │ 4 axes     │   │ Control loop  │  │ ESP-NOW│  │ - CRC-16/CCITT check          │   │
│  └────────────┘   │ 100 Hz send   │──┼───────▶│  │ - version + sequence          │   │
│  ┌────────────┐   └───────────────┘  │ AES    │  └───────────────┬───────────────┘   │
│  │ Encoder    │──▶                   │encrypt │                  │                   │
│  │ + buttons  │   ┌───────────────┐  │        │  ┌───────────────▼───────────────┐   │
│  └────────────┘   │ Core 0        │  │◀───────┼──│ Steering + Motion Model       │   │
│  ┌────────────┐◀──│ Display task  │  │telemetry│ │ - speed-dependent gain        │   │
│  │ ST7735     │   │ SPI render    │  │        │  │ - slew limiting               │   │
│  │ 128×128    │   └───────────────┘  │        │  │ - accel/decel/brake ramps     │   │
│  └────────────┘                      │        │  └───────────────┬───────────────┘   │
└──────────────────────────────────────┘        │                  │                   │
                                                │  ┌───────────────▼───────────────┐   │
                            ┌───────────────────┼──│ MPU6050 Yaw Assist (optional) │   │
                            │                   │  └───────────────┬───────────────┘   │
                            │                   │                  │                   │
                            │                   └──────────────────┼───────────────────┘
                            │                                      │
                 ┌──────────▼──────────┐              ┌────────────▼────────────┐
                 │ Failsafe Monitor    │              │ BTS7960 ×2              │
                 │ 120 ms timeout      │─────STOP────▶│ Left + Right Motors     │
                 └─────────────────────┘              └────────────┬────────────┘
                                                                   │
                                                      ┌────────────▼────────────┐
                                                      │ 555 Geared Motors ×4    │
                                                      │ + WS2812B RGB LEDs      │
                                                      └─────────────────────────┘
```

### Packet Processing Flow
```
                        ┌──────────────────────────────────────────┐
                        │            TRANSMITTER                   │
┌─────────────┐         │  ┌────────────────────────────────────┐  │
│ Joystick    │──A0/A1──┼─▶│ ADS1115 read (16-bit)              │  │
│ axes        │──A2/A3──┼─▶│                                    │  │
└─────────────┘         │  └──────────────┬─────────────────────┘  │
                        │                 │                        │
                        │  ┌──────────────▼─────────────────────┐  │
                        │  │ Calibration mapping                │  │
                        │  │ (min/centre/max, invert, deadband) │  │
                        │  └──────────────┬─────────────────────┘  │
                        │                 │                        │
                        │  ┌──────────────▼─────────────────────┐  │
                        │  │ Build ControlPacket                │  │
                        │  │ throttle/steering ±1000, buttons,  │  │
                        │  │ speedLimit, sequence, CRC-16       │  │
                        │  └──────────────┬─────────────────────┘  │
                        └─────────────────┼────────────────────────┘
                                          │ encrypted unicast, 10 ms
                        ┌─────────────────▼────────────────────────┐
                        │            RECEIVER                      │
                        │  ┌────────────────────────────────────┐  │
                        │  │ CRC + version validation           │  │
                        │  │ (bad CRC → treated as silence)     │  │
                        │  └──────────────┬─────────────────────┘  │
                        │                 │                        │
                        │  ┌──────────────▼─────────────────────┐  │
                        │  │ Steering model                     │  │
                        │  │ gain(throttle) + rate limit        │  │
                        │  └──────────────┬─────────────────────┘  │
                        │                 │                        │
                        │  ┌──────────────▼─────────────────────┐  │
                        │  │ Motor ramping + stall guard        │  │
                        │  │ accel 650 / decel 1000 / rev 1500  │  │
                        │  └──────────────┬─────────────────────┘  │
                        │                 │                        │
                        │  ┌──────────────▼─────────────────────┐  │
                        │  │ LEDC PWM 20 kHz, 8-bit → BTS7960   │  │
                        │  └────────────────────────────────────┘  │
                        │                 │ TelemetryPacket        │
                        └─────────────────┼────────────────────────┘
                                          ▼ back to transmitter
```

## 📡 Radio Protocol

| Parameter | Value |
|-----------|-------|
| Channel | 6 |
| Send period | 10 ms (100 Hz) |
| Receiver failsafe | 120 ms |
| Telemetry stale threshold | 500 ms |
| Encryption | ESP-NOW PMK + LMK (16 bytes each) |
| Integrity | CRC-16/CCITT, polynomial `0x1021`, init `0xFFFF` |
| Addressing | Unicast to fixed MAC — no broadcast, no discovery |

The failsafe timeout is deliberately twelve send periods: ordinary jitter never trips it, but a genuine link loss is caught in under a fifth of a second. Full details in [docs/PROTOCOL.md](docs/PROTOCOL.md).

## 💡 LED Lighting System

Carried over from V1 and driven by the receiver.

- **LED Type**: WS2812B Addressable RGB LEDs
- **Number of LEDs**: 16
- **Power**: 5V from regulated source

Lighting is toggled from the transmitter via a single short press, with the failsafe state driving its own warning pattern.

## 🧠 Control Architecture

### Motion profile

| Setting | Value | Reasoning |
|---------|-------|-----------|
| `MOTOR_PWM_FREQ` | 20 kHz | Above audible range — no drivetrain whine |
| `MOTOR_ACCEL_PWM_PER_SEC` | 650 | Protects the gearbox, limits inrush |
| `MOTOR_DECEL_PWM_PER_SEC` | 1000 | Stopping may be more aggressive than starting |
| `MOTOR_REVERSE_BRAKE_PWM_PER_SEC` | 1500 | Reversing against rotation is the worst case |
| `STALL_GUARD_MIN_PWM` | 55 | Below this the geared motors will not start under load |

Kill and failsafe bypass all three ramps and cut to zero immediately.

### Steering model

| Setting | Value | Effect |
|---------|-------|--------|
| `STEERING_GAIN_LOW_SPEED` | 1.10 | Full authority for tight low-speed turns |
| `STEERING_GAIN_HIGH_SPEED` | 0.45 | Reduced authority at speed for stability |
| `STEERING_ENGAGE_RATE_PER_SEC` | 1.80 | Turn-in, scaled down further by throttle |
| `STEERING_RELEASE_RATE_PER_SEC` | 3.50 | Faster release than engagement |

Engagement is deliberately slower than release — a stick flick cannot snap the car sideways, but straightening out stays responsive. See [docs/TUNING.md](docs/TUNING.md).

### ESP32-S3 Processing Workflow

1. **Signal Acquisition**
   - Read four axes from the ADS1115 over I²C
   - Apply calibration mapping to normalised ±1000 values

2. **Control Logic Processing**
   - Apply deadband filtering to remove stick noise
   - Build the control packet with sequence number and CRC
   - Transmit encrypted unicast every 10 ms

3. **Receiver Processing**
   - Validate CRC and packet version
   - Apply speed-dependent steering gain and slew limiting
   - Ramp motor outputs, apply speed limit and stall guard

4. **Safety Monitoring**
   - Track time since the last valid packet
   - Stop immediately past 120 ms or on kill
   - Return telemetry with sequence acknowledgement

## 🚀 Getting Started

### Prerequisites
- Arduino IDE (1.8.13 or later)
- ESP32 board package installed in Arduino IDE
- Required libraries:
  - Adafruit ADS1X15
  - Adafruit GFX
  - Adafruit ST7735
  - Adafruit NeoPixel
  - Adafruit MPU6050
  - Adafruit Unified Sensor

### Installation

1. Clone this repository:
```bash
git clone https://github.com/Kamalbura/esp32-rc-race-car.git
```

2. Create the pairing configuration for each sketch you intend to flash:
```bash
cp transmitter_code/secrets.example.h           transmitter_code/secrets.h
cp receiver_code_advanced_mpu/secrets.example.h receiver_code_advanced_mpu/secrets.h
```

3. Fill in each `secrets.h` with the peer MAC address and the shared PMK/LMK. Find a board's MAC with `WiFi.macAddress()` after `WiFi.mode(WIFI_STA)`. **Both ends must use identical keys and the same channel**, or the link will not come up.

4. Open the Arduino IDE, select an ESP32-S3 board, and upload `receiver_code_advanced_mpu` to the car and `transmitter_code` to the controller.

5. Open both Serial Monitors at `115200` and confirm the receiver reports link activity.

> **Test with the wheels off the ground first.**

## 📝 Usage Instructions

1. Power on the receiver, then the transmitter
2. Confirm link state on the transmitter display
3. Use the controls:
   - Left axis: forward/reverse throttle
   - Right axis: left/right steering
   - Encoder rotation: speed limit, 0–255
   - Long press: toggle kill
   - Single short press: toggle lights
   - Triple short press: toggle calibration mode

`GPIO0` (BOOT) doubles as a runtime fallback button — do not hold it during reset or power-up. In calibration mode the full knob range is scaled into half output.

## 🔬 Technical Achievements

- **Custom Radio Protocol**: Designed a packet format, integrity scheme and failsafe policy from scratch rather than consuming PWM from an off-the-shelf receiver
- **Link-Layer Integrity**: CRC-16/CCITT with corrupt frames folded into the failsafe path instead of being acted on
- **Closed-Loop Telemetry**: Sequence acknowledgement makes packet loss measurable rather than merely felt
- **Torque-Led Drivetrain Design**: Selected motors from a torque budget per tyre instead of headline RPM, and carried that constraint into the firmware
- **Real-Time Dual-Core Scheduling**: Isolated SPI rendering from the control loop so display work cannot jitter the send interval
- **Empirical Calibration**: Built both guided and host-driven capture tooling, including deliberate overshoot sampling to size the deadband correctly
- **Sensor Fusion**: Gyro-offset-calibrated yaw-rate assist with an automated turn test for per-surface tuning

## 🎥 Project Demo

[![Demo Video](./media/test-ride-thumb.jpg)](./media/demo.mp4)

The demo includes:
- Different steering modes
- LED lighting system demonstrations
- Performance on various terrains
- Failsafe feature in action

## 📚 Documentation

| Document | Contents |
|----------|----------|
| [docs/HARDWARE.md](docs/HARDWARE.md) | Both generations' BOMs, complete pinouts, the motor decision, known caveats |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Packet structs, CRC, encryption, failsafe timing, dual-core split |
| [docs/TUNING.md](docs/TUNING.md) | Motion profile, steering model, yaw assist, retuning procedure |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Guided and host-driven capture workflows, current axis values |

## 🔮 Future Enhancements

Delivered since V1:

- [x] Add MPU6050 for gyroscopic stabilization
- [x] Add telemetry data transmission

Still open:

- [ ] Enable yaw assist by default once tuned across surfaces
- [ ] Battery voltage cutoff and low-cell warning on the transmitter display
- [ ] Log telemetry to SD for post-run analysis
- [ ] Develop autonomous navigation capabilities
- [ ] Web interface for configuration settings

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Authors

V2 by Kamal Bura. V1 (Cyber Truck) by Kamal Bura, Vighnesh, and Karthekeya.

## 🙏 Acknowledgments

- ESP32 Community for their excellent documentation
- Espressif for the ESP-NOW and TinyUSB stacks
- Adafruit library developers
- Arduino community for library support
- Our instructors and classmates for valuable feedback

---

*Created as part of an embedded systems engineering project, this repository demonstrates practical application of microcontroller programming, custom radio protocol design, PWM motor control, and real-time systems design.*
