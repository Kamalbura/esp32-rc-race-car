# V1 — Cyber Truck

Generation 1 firmware, kept for reference. This ran on the same chassis as the
current V2 build, before the motors, battery and radio link were replaced.

Superseded by the V2 code at the repository root. See the
[generation comparison](../README.md#two-generations).

## What V1 was

- ESP32 dev board reading a **commercial 6-channel RC receiver** directly
- Interrupt-timed PWM decode on channel pins `36`, `39`, `34`, `35`, `32`, `33`
- Differential steering across BTS7960 drivers on `25`, `26`, `27`, `18`
  with enables on `19`, `23`
- Johnson 1000 RPM motors ×4
- WS2812B strip on `13`
- Failsafe on receiver signal loss

## Sketches

| Sketch | Purpose |
|---|---|
| `cyber-truck/` | Main build |
| `cyber_truck_dabble/` | Dabble app control over Bluetooth |
| `cyber_truck_serial_debug/` | Serial instrumentation for channel decode |
| `esp32_rc_car/` | Earlier RC car variant |
| `phase-2/` | NeoPixel work plus an optional MPU6050 — the first step toward the IMU work that became V2's yaw assist |

## Why it was replaced

A commercial receiver hands you PWM channels and nothing else — no return path,
no packet integrity, and no way to distinguish a lost frame from a centred
stick. V2 replaces it with a purpose-built transmitter and an encrypted ESP-NOW
link carrying structured packets, CRC checking and bidirectional telemetry.
See [docs/PROTOCOL.md](../docs/PROTOCOL.md).
