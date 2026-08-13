# Hardware

The same chassis across two generations. Generation 1 ("Cyber Truck") used a
commercial RC receiver; generation 2 replaced the entire radio link, the motors
and the battery while keeping the frame.

---

## Generation comparison

| | V1 — Cyber Truck | V2 — Race Car (current) |
|---|---|---|
| Controller | ESP32 dev board | ESP32-S3 ×2 (transmitter + receiver) |
| Radio link | Commercial 6-channel RC receiver | Custom encrypted ESP-NOW, peer-to-peer |
| Control input | Off-the-shelf RC transmitter | Purpose-built handheld transmitter |
| Motors | Johnson 1000 RPM ×4 | **555 geared, 500 RPM** |
| Motor drivers | BTS7960 ×2 | BTS7960 ×2 |
| Battery | 7.4–11.1 V LiPo | Upgraded pack |
| IMU | None (optional MPU6050 in `phase-2`) | MPU6050, yaw-rate assist |
| Feedback | WS2812B strip | WS2812B strip + on-board 128×128 display |
| Telemetry | None | Bidirectional, ACK + sequence counters |

---

## Why the motors changed

Generation 1 ran four Johnson 1000 RPM motors. On paper that is the faster
setup, but RPM is the wrong figure to optimise for on a heavy chassis with
grippy tyres — what reaches the ground is torque at the wheel, and a high-RPM
motor with a short gear reduction gives away exactly that.

Generation 2 uses **555-frame geared motors rated 500 RPM**. Half the free speed,
substantially more torque per wheel. The choice came from working the torque
budget per tyre rather than chasing top speed:

- Traction is limited by the tyre, not the motor. Beyond the point where the
  tyre breaks loose, extra RPM converts into wheelspin rather than acceleration.
- A heavier pack and chassis raise the torque needed to start moving from rest.
  High-RPM/low-torque motors stall or draw heavy current at exactly that moment.
- Lower free speed makes the drivetrain far easier to control at low duty
  cycles, which is what the V2 steering model and stall guard depend on.

The `STALL_GUARD_MIN_PWM` value of 55 in the receiver exists because of this
change: below roughly 20 % duty the geared motors do not reliably start turning
under load, so any commanded motion below that floor is raised to it.

---

## V2 — Transmitter pinout (ESP32-S3)

| Function | GPIO | Notes |
|---|---|---|
| ADS1115 SDA | `8` | I²C |
| ADS1115 SCL | `9` | I²C @ 400 kHz |
| ADS1115 ADDR | `GND` | Address `0x48` |
| ST7735 SCK | `12` | SPI |
| ST7735 MOSI | `11` | SPI |
| ST7735 MISO | `13` | Reserved |
| ST7735 CS | `15` | |
| ST7735 DC | `16` | |
| ST7735 RST | `17` | |
| Encoder A / CLK | `4` | Input pullup |
| Encoder B / DT | `5` | Input pullup |
| Encoder SW | `6` | Input pullup |
| BOOT fallback button | `0` | Do not hold during reset |
| Built-in RGB LED | `48` | WS2812 |

A 16-bit ADS1115 is used for the control axes rather than the ESP32's internal
ADC, whose non-linearity and noise floor are poor enough to be felt in a
steering channel.

## V2 — Receiver pinout (ESP32-S3)

| Function | GPIO | LEDC channel |
|---|---|---|
| Left BTS LPWM | `15` | 0 |
| Left BTS RPWM | `16` | 1 |
| Left BTS L_EN | `17` | — |
| Left BTS R_EN | `18` | — |
| Right BTS LPWM | `9` | 2 |
| Right BTS RPWM | `10` | 3 |
| Right BTS L_EN | `11` | — |
| Right BTS R_EN | `12` | — |
| MPU6050 SDA | `1` | I²C |
| MPU6050 SCL | `2` | I²C |

Motor PWM runs at **20 kHz, 8-bit** — above audible range, so the drivetrain
does not whine. `LEFT_MOTOR_INVERT` is `true` and `RIGHT_MOTOR_INVERT` is
`false`, correcting the mirrored mounting in software instead of by swapping
wires.

### MPU6050 mounting

- `+X` toward the front of the car
- `+Y` toward the left side
- `+Z` upward
- Positive gyro Z is a left-turn yaw

---

## V1 — Cyber Truck pinout (ESP32)

| Function | GPIO |
|---|---|
| RC channels CH1–CH6 | `36`, `39`, `34`, `35`, `32`, `33` |
| Motor PWM | `25`, `26`, `27`, `18` |
| Motor enable | `19`, `23` |
| NeoPixel (12 LEDs) | `13` |

V1 read the receiver's PWM output directly with interrupt timing, which is why
its channel pins are all input-only ADC-capable GPIOs.

---

## Known hardware caveats

1. `GPIO37` as a buzzer is board-variant dependent on some ESP32-S3 modules.
2. `GPIO0` (BOOT) is used as a runtime button. Holding it during power-up or
   reset drops the board into bootloader mode.
3. ADS1115 centre readings drift with wiring noise. Re-run the guided
   calibration whenever the harness is disturbed — see [CALIBRATION.md](CALIBRATION.md).
4. Encoder contacts are electrically noisy; the firmware debounces in software
   rather than relying on RC filtering.
