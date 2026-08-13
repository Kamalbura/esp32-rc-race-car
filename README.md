# RC Race Car — ESP32-S3 ESP-NOW (V7)

A complete radio-control system where **both ends are custom hardware**: a handheld
ESP32-S3 transmitter and an ESP32-S3 receiver on the car, linked by encrypted
ESP-NOW rather than a commercial RC receiver.
The codebase is split into Arduino IDE sketch folders. Open the folder that has
the same name as the `.ino` file inside it.

## Main Sketches

- `transmitter_code/transmitter_code.ino`
- `receiver_code/receiver_code.ino`
- `receiver_code_advanced_mpu/receiver_code_advanced_mpu.ino`
- `legacy_flysky_ibus/legacy_flysky_ibus.ino` (reference only)

## Test Sketches

- `tests/espnow_transmitter_test/espnow_transmitter_test.ino`
- `tests/espnow_receiver_test/espnow_receiver_test.ino`
- `tests/bts_motor_test/bts_motor_test.ino`
- `tests/transmitter_pot_test/transmitter_pot_test.ino`
- `tests/spi_display_128x128_test/spi_display_128x128_test.ino`
- `tests/transmitter_guided_calibration_test/transmitter_guided_calibration_test.ino`
- `tests/transmitter_aggressive_logger_test/transmitter_aggressive_logger_test.ino`

## Link Behaviour

- Secure direct ESP-NOW unicast between fixed COM3 and COM4 MAC addresses
- Transmitter send period: `10 ms` (about 100 Hz)
- Receiver failsafe: `120 ms`
- Transmitter control loop and display loop split across both ESP32-S3 cores
- Motor direction correction in software:
  - `LEFT_MOTOR_INVERT = true`
  - `RIGHT_MOTOR_INVERT = false`
- Speed limit is `0..255` end-to-end
- Receiver logs show both requested and effective speed:
  - `spdReq`
  - `spdEff` (after calibration scaling)

## Pairing and Setup

Each sketch folder needs a `secrets.h` holding the peer MAC and the shared keys.
It is gitignored; copy the example and fill it in:

```bash
cp transmitter_code/secrets.h.example transmitter_code/secrets.h
cp receiver_code_advanced_mpu/secrets.h.example receiver_code_advanced_mpu/secrets.h
# repeat for any test sketch you intend to flash
```

| Board | Role | Port |
| --- | --- | --- |
| ESP32-S3 transmitter | Handheld controller | `COM3` |
| ESP32-S3 receiver | Car controller | `COM4` |

Find a board's MAC by calling `WiFi.macAddress()` after `WiFi.mode(WIFI_STA)`.
Both ends must use identical PMK and LMK, and the same `ESPNOW_CHANNEL`.

## Quick Bring-Up

1. Flash `receiver_code_advanced_mpu` to COM4.
2. Flash `transmitter_code` to COM3.
3. Open both Serial Monitors at `115200`.
4. Confirm receiver prints link activity.
5. Test controls with wheels off the ground first.

## Transmitter Controls

- Encoder or BOOT long press: toggle kill
- Encoder or BOOT single short press: toggle lights
- Encoder or BOOT triple short press: toggle calibration mode
- Encoder rotation: speed limit `0..255`

Notes:
- BOOT (`GPIO0`) is used as a runtime fallback button.
- Do not hold BOOT during reset or power-up.
- In calibration mode, full speed knob range is scaled into safe half output range.

## Transmitter Axis Mapping

- `A0` -> steering axis
- `A1` -> throttle axis
- `A2` -> aux1
- `A3` -> aux2 / mode source

Current calibration values in code are tuned from recent guided captures:

| Channel | min | center | max | invert | deadband |
| --- | --- | --- | --- | --- | --- |
| `A0` steering | 4000 | 21220 | 32000 | `true` | 500 |
| `A1` throttle | 4000 | 20560 | 32000 | `true` | 500 |
| `A2` aux1 | 4000 | 21090 | 32000 | `false` | 500 |
| `A3` aux2/mode | 4000 | 20440 | 32000 | `false` | 500 |

## Dual-Core Design (Transmitter)

- Core 1:
  - read controls
  - build and send ESP-NOW packets
  - update status LED
- Core 0:
  - display task (`displayTask`)
  - serial status printing

This keeps SPI display work from blocking the control send loop.

## Transmitter Pinout

| Function | ESP32-S3 GPIO | Notes |
| --- | --- | --- |
| ADS1115 SDA | `8` | I2C |
| ADS1115 SCL | `9` | I2C @ 400 kHz |
| ADS1115 ADDR | `GND` | I2C address `0x48` |
| ST7735 SCK | `12` | SPI |
| ST7735 MOSI | `11` | SPI |
| ST7735 MISO | `13` | Reserved in code |
| ST7735 CS | `15` | Display chip select |
| ST7735 DC | `16` | Display D/C |
| ST7735 RST | `17` | Display reset |
| Encoder A/CLK | `4` | Input pullup |
| Encoder B/DT | `5` | Input pullup |
| Encoder SW | `6` | Input pullup |
| BOOT fallback button | `0` | Input pullup |
| Built-in RGB LED | `48` | WS2812 style on common S3 boards |

Display is currently configured as `128x128` ST7735 with `DISPLAY_ROTATION 0`.

## Receiver Pinout

| Function | ESP32-S3 GPIO | Notes |
| --- | --- | --- |
| Left BTS LPWM | `15` | PWM channel 0 |
| Left BTS RPWM | `16` | PWM channel 1 |
| Left BTS L_EN | `17` | HIGH after init |
| Left BTS R_EN | `18` | HIGH after init |
| Right BTS LPWM | `9` | PWM channel 2 |
| Right BTS RPWM | `10` | PWM channel 3 |
| Right BTS L_EN | `11` | HIGH after init |
| Right BTS R_EN | `12` | HIGH after init |
| 16-LED ring DIN | `8` | NeoPixel GRB |
| Buzzer SIG | `37` | Board-dependent safety note below |
| MPU6050 SDA (advanced) | `1` | I2C |
| MPU6050 SCL (advanced) | `2` | I2C |

## MPU Mounting Assumption

Current advanced receiver code assumes:

| MPU axis | Car direction |
| --- | --- |
| `+Y` | Front |
| `+X` | Right side |
| `+Z` | Up |

Positive gyro Z means left yaw.

## Advanced Receiver Serial Commands

Available on `receiver_code_advanced_mpu` at `115200`:

| Command | Action |
| --- | --- |
| `h` | Help |
| `b` | Buzzer test |
| `c` | Recalibrate gyro Z offset |
| `i` | MPU idle snapshot |
| `l` | Left 90 degree turn test |
| `p` | RGB ring test |
| `r` | Right 90 degree turn test |
| `s` | Stop motors |

Periodic receiver line includes:

- `thr`, `steer`
- `spdReq`, `spdEff`
- `left`, `right`
- `yaw`
- `telErr`

## BTS-Only Motor Test

Use `tests/bts_motor_test` for isolated BTS checks.

Commands:

- `w` forward
- `s` reverse
- `a` spin left
- `d` spin right
- `i` left motor forward
- `k` left motor reverse
- `o` right motor forward
- `l` right motor reverse
- `+` or `-` speed
- `r` auto ramp
- `0` stop
- `p` print BTS pinout
- `h` help

## Transmitter ADC and Mapping Test

Use `tests/transmitter_pot_test` to inspect raw values and map joystick movement.

Commands:

- `c` capture center
- `r` reset min/max
- `g` start guided direction mapping
- `n` capture current mapping step
- `x` cancel guided mapping
- `h` help

Guided mapping steps:

- left up, left down, left left, left right
- right up, right down, right left, right right

It prints detected dominant channel and polarity per movement.

## ESP-NOW Raw Link Test

Use `tests/espnow_transmitter_test` with `tests/espnow_receiver_test` to validate:

- raw ADS values
- secure unicast send/receive
- ACK return path
- sequence and missed packet counters

## V7 Steering Model

Steering authority scales with throttle, and the command is slew-limited rather
than applied instantly:

| Setting | Value | Effect |
| --- | --- | --- |
| `STEERING_GAIN_LOW_SPEED` | `1.10` | More authority for tight low-speed turns |
| `STEERING_GAIN_HIGH_SPEED` | `0.45` | Reduced authority at speed for stability |
| `STEERING_ENGAGE_RATE_PER_SEC` | `1.80` | Turn-in rate, scaled down further by throttle |
| `STEERING_RELEASE_RATE_PER_SEC` | `3.50` | Faster release than engagement, so recovery is quick |
| `STRAIGHT_ASSIST_STEER_WINDOW` | `0.10` | Deadband treated as straight-ahead |
| `STALL_GUARD_THROTTLE_MIN` | `0.30` | Below this throttle, apply the floor PWM |
| `STALL_GUARD_MIN_PWM` | `55` | Minimum duty that actually turns the motors |

Engagement is deliberately slower than release: turn-in is smoothed to avoid
snap, while straightening out stays responsive.

## Drive Safety

Receiver ramp settings:

| Setting | Value |
| --- | --- |
| `MOTOR_ACCEL_PWM_PER_SEC` | `650` |
| `MOTOR_DECEL_PWM_PER_SEC` | `1000` |
| `MOTOR_REVERSE_BRAKE_PWM_PER_SEC` | `1500` |

Kill and failsafe still stop immediately.

## Aggressive Bug Audit Notes

Current status from compile audit:

- All main sketches compile cleanly.
- All listed test sketches compile cleanly.

Known risks to keep in mind:

1. `GPIO37` buzzer is board-variant dependent on some ESP32-S3 modules.
2. BOOT button (`GPIO0`) is used at runtime, so avoid holding during reset.
3. ADS1115 center can drift with wiring noise. Re-run guided calibration when hardware changes.
4. Encoder electrical noise can affect speed stepping. Current code includes debounced step logic.

## Validation Snapshot

Latest compile sizes (approx):

- `transmitter_code`: ~55%
- `receiver_code_advanced_mpu`: ~55%
- `receiver_code`: ~53%
- tests: all compile without errors

This README tracks V7 behaviour as committed in this repository.
