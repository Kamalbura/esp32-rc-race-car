# Contributing to Vector-RC

Thanks for helping improve Vector-RC. This is real hardware that drives motors, so the
bar is: **it compiles in CI, and anything that changes runtime behavior is bench-tested
with the wheels off the ground before it's trusted.**

## Development setup

You can use the Arduino IDE or `arduino-cli`. CI uses `arduino-cli`, so matching it locally
avoids surprises.

1. Install the ESP32 core (Arduino boards manager URL:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`).
   Target board / FQBN: `esp32:esp32:esp32s3`.
2. Install libraries: Adafruit ADS1X15, Adafruit GFX, Adafruit ST7735/ST7789, Adafruit
   NeoPixel, Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO.
3. **Secrets:** in every sketch folder you build, copy `secrets.example.h` to `secrets.h`
   and fill in your board MACs and ESP-NOW keys. `secrets.h` is gitignored — never commit it.
   Both boards must share identical PMK + LMK.

Compile one sketch:

```
arduino-cli compile --fqbn esp32:esp32:esp32s3 transmitter_code
```

## Repo conventions

- **Each sketch is a folder** whose name matches its `.ino`. That's how Arduino and CI find it.
- **Keep the packet format compatible.** `ControlPacket` / `TelemetryPacket` are shared by both
  boards and versioned (`RC_PACKET_VERSION`). If you must change a struct, bump the version and
  update both sides in the same PR — and call it out as a breaking change.
- **Match the surrounding style** (2-space indent, existing naming, comment density). Don't
  reformat files you aren't otherwise changing.
- **Don't claim capabilities the code doesn't have** in docs, comments, or commit messages
  (e.g. long-range mode, RSSI, working battery telemetry, yaw-assist). Make it true first.

## Firmware changes

- **Isolate behavior changes** into focused commits so they can be bench-tested and bisected
  independently. Don't bundle an unrelated refactor with a control-loop change.
- **Say what needs a physical retest.** If a change alters timing, motor output, or radio
  behavior, note in the PR that it needs a bench run and what to look for.
- Kill and the 120 ms failsafe must always stop the motors immediately — don't add a code path
  that can bypass them.

## Pull requests

- CI (`.github/workflows/build.yml`) compiles every standard sketch for ESP32-S3 and must pass.
- Describe what you changed, why, and — for firmware — how you verified it on hardware.

## Safety

Always test with the wheels off the ground first. Arm only after confirming kill and failsafe
behavior on your bench.
