# RC Race Car — ESP32-S3

[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Link](https://img.shields.io/badge/Link-ESP--NOW%20encrypted-orange.svg)](docs/PROTOCOL.md)

A radio-control car where **both ends of the radio link are custom hardware** —
a handheld ESP32-S3 transmitter and an ESP32-S3 receiver on the car, paired
over encrypted ESP-NOW at 100 Hz.

![Hardware](media/hardware.jpg)

One chassis, two generations. Generation 1 was built around a commercial RC
receiver. Generation 2 kept the frame and replaced the motors, the battery and
the entire radio link.

---

## Two generations

| | **V1 — Cyber Truck** | **V2 — Race Car** (current) |
|---|---|---|
| Radio | Commercial 6-channel RC receiver | Custom encrypted ESP-NOW peer link |
| Controller | ESP32 dev board | ESP32-S3 ×2 |
| Motors | Johnson 1000 RPM ×4 | 555 geared, 500 RPM |
| Telemetry | None | Bidirectional, ACK + sequence counters |
| Failsafe | Receiver signal loss | 120 ms link timeout, CRC-checked |
| IMU | Optional, unused | MPU6050 with yaw-rate assist |
| Code | [`v1-cyber-truck/`](v1-cyber-truck/) | this repository root |

The motor change is the one worth explaining. V1's four Johnson 1000 RPM motors
looked like the faster setup, but RPM is the wrong number to optimise on a
heavy chassis with grippy tyres — what matters at the ground is torque per
wheel. V2 runs 555-frame geared motors at half the free speed and considerably
more torque, chosen by working the torque budget per tyre rather than chasing
top speed. Full reasoning in [docs/HARDWARE.md](docs/HARDWARE.md).

**Video:** [test ride](media/test-ride.mp4) · [demo](media/demo.mp4)

---

## Documentation

| Document | Contents |
|---|---|
| [docs/HARDWARE.md](docs/HARDWARE.md) | Both generations' bills of materials, full pinouts, why the motors changed, known caveats |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | ESP-NOW packet formats, CRC-16, encryption, failsafe timing, dual-core split |
| [docs/TUNING.md](docs/TUNING.md) | Motion profile, V7 steering model, yaw assist, retuning procedure |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Guided and aggressive calibration workflows, current axis values |

---

## Repository layout

```
transmitter_code/            V2 handheld transmitter (current)
receiver_code/               V2 car receiver
receiver_code_advanced_mpu/  V2 receiver with MPU6050 yaw assist
legacy_flysky_ibus/          FlySky iBUS receiver, kept for reference
v1-cyber-truck/              Generation 1 firmware
tests/                       Per-subsystem test sketches
tools/                       Host-side calibration capture
calibration/                 Axis observations and a reference capture run
docs/                        Detailed documentation
media/                       Photos and video
```

---

## Setup

Each sketch folder needs a `secrets.h` holding the peer MAC and the shared
ESP-NOW keys. It is gitignored — copy the example and fill it in:

```bash
cp transmitter_code/secrets.h.example transmitter_code/secrets.h
cp receiver_code_advanced_mpu/secrets.h.example receiver_code_advanced_mpu/secrets.h
# repeat for any test sketch you intend to flash
```

Find a board's MAC with `WiFi.macAddress()` after `WiFi.mode(WIFI_STA)`. Both
ends need **identical** PMK and LMK and the same `ESPNOW_CHANNEL`, or the link
will not come up.

### Bring-up

1. Flash `receiver_code_advanced_mpu` to the receiver board.
2. Flash `transmitter_code` to the transmitter board.
3. Open both serial monitors at `115200`.
4. Confirm the receiver reports link activity.
5. **Test with the wheels off the ground first.**

### Libraries

`Adafruit ADS1X15` · `Adafruit GFX` · `Adafruit ST7735` · `Adafruit NeoPixel` ·
`Adafruit MPU6050` · `Adafruit Unified Sensor`

---

## Transmitter controls

| Input | Action |
|---|---|
| Encoder rotate | Speed limit, 0–255 |
| Encoder or BOOT — long press | Toggle kill |
| Encoder or BOOT — single short press | Toggle lights |
| Encoder or BOOT — triple short press | Toggle calibration mode |

`GPIO0` (BOOT) is a runtime fallback button — do not hold it during reset or
power-up. In calibration mode the full knob range is scaled into half output.

---

## Link behaviour

- Encrypted ESP-NOW unicast between two fixed MACs
- 10 ms send period (100 Hz), 120 ms receiver failsafe
- CRC-16/CCITT on every packet; a bad CRC is treated as silence
- Transmitter control and display loops split across both cores
- Motor inversion corrected in software (`LEFT_MOTOR_INVERT = true`)
- Receiver reports both requested (`spdReq`) and effective (`spdEff`) speed

---

## Status

All main and test sketches compile cleanly. Flash usage roughly 55 % for
`transmitter_code` and `receiver_code_advanced_mpu`, 53 % for `receiver_code`.

Known risks are tracked in [docs/HARDWARE.md](docs/HARDWARE.md#known-hardware-caveats).

---

## Credits

Generation 1 (Cyber Truck) was built with **Vighnesh** (control algorithms and
motor driver interface) and **Karthekeya** (chassis design and LED system).
Generation 2 electronics and firmware by **Kamal Bura**.
