# RC Race Car ESP32-S3 ESP-NOW Version

This repo is now split into Arduino IDE sketch folders. Open the folder that has
the same name as the `.ino` file inside it.

## Main Sketches

- `transmitter_code/transmitter_code.ino`: ESP32-S3 handheld transmitter. Reads
  the ADS1115 joystick inputs, rotary encoder, optional ST7789 SPI display, then
  sends control packets over ESP-NOW.
- `receiver_code/receiver_code.ino`: basic car receiver. Receives ESP-NOW
  packets and drives the two BTS motor modules with the same tank-mix logic from
  the old FlySky/iBUS sketch.
- `receiver_code_advanced_mpu/receiver_code_advanced_mpu.ino`: advanced receiver.
  Same BTS/ESP-NOW base plus MPU6050 telemetry and optional yaw-rate turn assist.
- `legacy_flysky_ibus/legacy_flysky_ibus.ino`: copy of the original working
  sketch for reference.

## Test Sketches

- `tests/espnow_transmitter_test/espnow_transmitter_test.ino`
- `tests/espnow_receiver_test/espnow_receiver_test.ino`
- `tests/bts_motor_test/bts_motor_test.ino`
- `tests/transmitter_pot_test/transmitter_pot_test.ino`

## ESP-NOW Raw ADC Test

Use this before the full car receiver code. It proves the transmitter ADC,
display, ESP-NOW send path, ESP-NOW receive path, return ACK path, and encrypted
unicast pairing.

1. Upload `tests/espnow_receiver_test/espnow_receiver_test.ino` to the car-side
   ESP32-S3. In the current bench setup this is `COM4`.
2. Open Serial Monitor at `115200`. The receiver prints its MAC address and then
   waits for packets.
3. Upload `tests/espnow_transmitter_test/espnow_transmitter_test.ino` to the
   handheld transmitter ESP32-S3. In the current bench setup this is `COM3`.
4. Open Serial Monitor at `115200` on the transmitter, or watch the SPI display.
5. Move each joystick/potentiometer connected to ADS1115 `A0`, `A1`, `A2`, and
   `A3`.

The transmitter sends uncalibrated raw ADS1115 values. No deadband, center
capture, inversion, or split mapping is applied in this test. The test uses
encrypted ESP-NOW unicast, not broadcast.

| Packet Field | Meaning |
| --- | --- |
| `adcRaw[0]` | Direct raw ADS1115 `A0` value |
| `adcRaw[1]` | Direct raw ADS1115 `A1` value |
| `adcRaw[2]` | Direct raw ADS1115 `A2` value |
| `adcRaw[3]` | Direct raw ADS1115 `A3` value |
| `encoderBits` bit `0` | Encoder `A` pin state |
| `encoderBits` bit `1` | Encoder `B` pin state |
| `encoderBits` bit `2` | Encoder switch state |
| `sequence` | Incrementing transmitter packet number |

The receiver prints the four raw channels and sends an ACK back to the
transmitter. The transmitter display shows the latest raw values, send state,
last ACK sequence, and receiver packet count.

Link-quality fields to watch:

| Field | Good Value | Meaning |
| --- | --- | --- |
| Transmitter `errors` | `0` | `esp_now_send()` accepted the packet for sending |
| Transmitter `ackTimeouts` | `0` or very low | Receiver ACK arrived before timeout |
| Receiver `bad` | `0` | Packet size, magic, version, and CRC are valid |
| Receiver `ignored` | `0` | No packets accepted from unknown MAC addresses |
| Receiver `missed` | `0` or very low | No sequence gaps in received transmitter packets |
| Receiver `rxAge` | Usually below `100 ms` | Time since receiver last got a valid packet |

Current hardcoded secure test peers:

| Board | Port | MAC |
| --- | --- | --- |
| Raw ADC transmitter | `COM3` | `B4:3A:45:3F:46:BC` |
| Raw ADC receiver | `COM4` | `B4:3A:45:3F:A4:E8` |

The secure test sketches use a shared 16-byte PMK and 16-byte LMK. ESP-NOW
broadcast packets are not encrypted, so these tests use only the hardcoded
unicast peer MAC. Change the PMK and LMK before competition use.

The transmitter test display is configured for a `128 x 128` SPI screen with
small text and `DISPLAY_ROTATION 0`. If the screen is still upside down on your
physical mount, change `DISPLAY_ROTATION` in `tests/espnow_transmitter_test` to
`1`, `2`, or `3`.

## Important Setup Notes

1. Upload `tests/espnow_receiver_test` to the car ESP32-S3 first and open Serial
   Monitor. It prints the receiver MAC address.
2. Paste that MAC into `RECEIVER_MAC` in `transmitter_code.ino` for direct
   unicast. Leaving it as `FF:FF:FF:FF:FF:FF` uses broadcast for first tests.
3. Run `tests/transmitter_pot_test` and move each joystick fully. Copy the real
   min/center/max values into `axisCal[]` in `transmitter_code.ino`.
4. Run `tests/bts_motor_test` with the wheels off the ground before using the
   full receiver.

## ESP-NOW Settings

| Setting | Value | Where |
| --- | --- | --- |
| ESP-NOW channel | `6` | All ESP-NOW sketches |
| Packet rate | `20 ms` / about `50 Hz` | `transmitter_code` |
| Receiver failsafe | `120 ms` | `receiver_code`, `receiver_code_advanced_mpu` |
| First-link mode | Broadcast MAC `FF:FF:FF:FF:FF:FF` | `transmitter_code` |

For racing, use direct unicast instead of broadcast. Upload the receiver first,
read its MAC address from Serial Monitor, then put that MAC in `RECEIVER_MAC` in
`transmitter_code/transmitter_code.ino`.

## Transmitter ESP32-S3 Pinout

This is the handheld controller board. These pins are used by
`transmitter_code/transmitter_code.ino` and
`tests/transmitter_pot_test/transmitter_pot_test.ino`.

| Function | Module Pin | ESP32-S3 GPIO | Notes |
| --- | --- | --- | --- |
| I2C data | ADS1115 `SDA` | `GPIO 8` | Shared I2C bus for joystick ADC |
| I2C clock | ADS1115 `SCL` | `GPIO 9` | Runs at `400 kHz` |
| ADC address | ADS1115 `ADDR` | `GND` | Locks ADS1115 address to `0x48` |
| ADC input 0 | ADS1115 `A0` | Joystick axis wire | Throttle in current code |
| ADC input 1 | ADS1115 `A1` | Joystick axis wire | Steering in current code |
| ADC input 2 | ADS1115 `A2` | Joystick/pot axis wire | Aux input |
| ADC input 3 | ADS1115 `A3` | Joystick/pot axis wire | Mode input |
| SPI clock | Display `SCK` / `SCL` | `GPIO 12` | SPI display clock |
| SPI data out | Display `MOSI` / `SDA` | `GPIO 11` | ESP32-S3 to display |
| SPI data in | Display `MISO` | `GPIO 13` | Not used by most ST7789 displays, reserved in code |
| Display select | Display `CS` | `GPIO 15` | ST7789 chip select |
| Display data/command | Display `DC` | `GPIO 16` | ST7789 command/data pin |
| Display reset | Display `RST` / `RES` | `GPIO 17` | ST7789 reset pin |
| Encoder A | Rotary encoder `CLK` / `A` | `GPIO 4` | Uses internal pullup |
| Encoder B | Rotary encoder `DT` / `B` | `GPIO 5` | Uses internal pullup |
| Encoder switch | Rotary encoder `SW` | `GPIO 6` | Press toggles kill/armed |
| Logic power | ADS1115 `VDD`, encoder `+` | `3V3` | Keep ADC signals at ESP32-safe voltage |
| Display power | Display `VCC` | `5V` or `3V3` | Match your display module requirement |
| Ground | All module `GND` pins | `GND` | All grounds must be common |

The transmitter code assumes a `128 x 128` SPI display using the Adafruit
ST7789-compatible driver path. If your exact display is an ST7735 or ILI9341,
keep the same wiring idea but change the display library/object in the
transmitter sketch.

## Receiver ESP32-S3 Pinout

This is the car board. These pins are used by `receiver_code/receiver_code.ino`,
`receiver_code_advanced_mpu/receiver_code_advanced_mpu.ino`, and
`tests/bts_motor_test/bts_motor_test.ino`.

| Function | Module Pin | ESP32-S3 GPIO | Notes |
| --- | --- | --- | --- |
| Left motor forward PWM | Left BTS `LPWM` | `GPIO 15` | PWM channel `0`, `20 kHz`, 8-bit |
| Left motor reverse PWM | Left BTS `RPWM` | `GPIO 16` | PWM channel `1`, `20 kHz`, 8-bit |
| Left driver left enable | Left BTS `L_EN` | `GPIO 17` | Set `HIGH` after PWM starts at zero |
| Left driver right enable | Left BTS `R_EN` | `GPIO 18` | Set `HIGH` after PWM starts at zero |
| Right motor forward PWM | Right BTS `LPWM` | `GPIO 9` | PWM channel `2`, `20 kHz`, 8-bit |
| Right motor reverse PWM | Right BTS `RPWM` | `GPIO 36` | PWM channel `3`, `20 kHz`, 8-bit |
| Right driver left enable | Right BTS `L_EN` | `GPIO 11` | Set `HIGH` after PWM starts at zero |
| Right driver right enable | Right BTS `R_EN` | `GPIO 12` | Set `HIGH` after PWM starts at zero |
| LED signal | NeoPixel / LED module `DIN` | `GPIO 8` | `48` LEDs in the current code |
| Battery sense | Voltage divider output | Not assigned | `BATTERY_ADC_PIN = -1` disables it |
| Ground | BTS logic `GND`, LED `GND`, ESP `GND` | `GND` | Must be tied to motor battery negative |

### Advanced Receiver Only

| Function | Module Pin | ESP32-S3 GPIO | Notes |
| --- | --- | --- | --- |
| MPU6050 I2C data | MPU6050 `SDA` | `GPIO 1` | Advanced receiver only |
| MPU6050 I2C clock | MPU6050 `SCL` | `GPIO 2` | Advanced receiver only |
| MPU6050 power | MPU6050 `VCC` | `3V3` or module-safe `5V` | Use what your breakout supports |
| MPU6050 ground | MPU6050 `GND` | `GND` | Common ground |

## BTS Motor Driver Wiring

Each BTS driver controls one motor. The ESP32-S3 controls only the low-current
logic pins. Motor power must go directly to the BTS high-current terminals.

| BTS Pin | Connect To | Notes |
| --- | --- | --- |
| `B+` / `VCC Motor` | Main motor battery positive | High-current path |
| `B-` / `GND Motor` | Main motor battery negative | Must also connect to ESP32-S3 `GND` |
| `M+` | Motor terminal 1 | Swap with `M-` if direction is backward |
| `M-` | Motor terminal 2 | Swap with `M+` if direction is backward |
| `VCC` / logic `5V` | Logic supply for BTS module | Most BTS7960 modules expect `5V` logic power |
| `GND` / logic ground | ESP32-S3 `GND` | Required for PWM signal reference |
| `LPWM` | ESP32-S3 PWM pin | Forward side in this code |
| `RPWM` | ESP32-S3 PWM pin | Reverse side in this code |
| `L_EN` | ESP32-S3 enable pin | Code drives it `HIGH` |
| `R_EN` | ESP32-S3 enable pin | Code drives it `HIGH` |

Bench-test one BTS at a time with `tests/bts_motor_test` and the wheels off the
ground. Start at low speed. If a motor spins opposite to the expected direction,
swap that motor's `M+` and `M-` wires or swap that motor's direction in code.

## Validation Notes

- The BTS pin map is identical in `receiver_code`, `receiver_code_advanced_mpu`,
  and `tests/bts_motor_test`.
- The transmitter ADS1115 and encoder pin map is identical in `transmitter_code`
  and `tests/transmitter_pot_test`.
- The receiver and transmitter are separate ESP32-S3 boards, so reused GPIO
  numbers between them are not conflicts.
- `GPIO 36` is used because it was already in the previous working sketch. Make
  sure your exact ESP32-S3 dev module exposes `GPIO 36` and does not reserve it
  for onboard flash, PSRAM, or another board feature.
- ESP-NOW does not need any extra wire between transmitter and receiver.
