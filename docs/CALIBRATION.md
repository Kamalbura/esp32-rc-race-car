# Calibration

The transmitter reads four analogue axes through a 16-bit ADS1115. Raw counts
mean nothing until the endpoints, centre and deadband are known for each
channel — and those move whenever the harness is disturbed. Two tools exist for
establishing them.

---

## Current values

Tuned from recent guided captures:

| Channel | min | centre | max | invert | deadband |
|---|---|---|---|---|---|
| `A0` steering | 4000 | 21220 | 32000 | `true` | 500 |
| `A1` throttle | 4000 | 20560 | 32000 | `true` | 500 |
| `A2` aux1 | 4000 | 21090 | 32000 | `false` | 500 |
| `A3` aux2 / mode | 4000 | 20440 | 32000 | `false` | 500 |

Note the centres are not at the midpoint of the range and differ per channel by
several hundred counts. That asymmetry is exactly why calibration is measured
rather than assumed.

---

## Method 1 — guided calibration (on-device)

`tests/transmitter_guided_calibration_test` walks through a sequence of held
positions and captures a sample set at each.

1. Flash the receiver test to the receiver board first, so link state is visible.
2. Flash the guided calibration test to the transmitter.
3. Open the serial monitor at `115200`.
4. For each prompt, move to the requested position and **hold steady**.
5. Send `r` to reset the current step's samples, hold for 1–2 seconds, then
   send `c` to capture.
6. On completion the sketch prints a block delimited by `CAL_JSON_BEGIN` and
   `CAL_JSON_END`.
7. Paste that block into `calibration/transmitter_observations.hjson`, or
   replace the file with it.

Capture with serial `c` rather than the encoder switch. The encoder works for
ordinary steps, but the button-hold steps need the switch left untouched so its
value is measured cleanly — pressing it to trigger the capture would record the
wrong state.

---

## Method 2 — aggressive capture (host-driven)

`tools/aggressive_transmitter_capture.py` drives
`tests/transmitter_aggressive_logger_test` from the host over serial and logs
every sample to CSV and JSON under `calibration/logs/`.

Where the guided method samples *static* held positions, this one deliberately
captures **dynamic** behaviour — the same movement performed slowly, at normal
speed, and flicked fast:

```
center_rest              Release both joysticks
both_slow_up_center      Move both slowly full UP, return to centre
both_normal_up_center    Same at normal speed
both_fast_up_center      Flick fast, return to centre
...repeated for DOWN, LEFT, RIGHT
```

The point is overshoot. A stick flicked to its stop and released does not
settle instantly, and a deadband tuned only on static readings will let that
ringing through as real control input. Sampling the fast cases shows how much
margin the deadband actually needs.

Run it with:

```bash
python tools/aggressive_transmitter_capture.py --port COM3
```

A reference capture is committed at
`calibration/logs/transmitter_aggressive_20260416_031819.csv` (with its `.json`
summary) so the analysis is reproducible without the hardware. Further captures
are gitignored.

---

## When to recalibrate

- Any time the ADS1115 wiring or the joystick harness is disturbed
- After replacing a joystick or potentiometer
- If the car creeps with the sticks centred — the centre has drifted
- If full deflection no longer reaches ±1000 in the control packet

ADS1115 centre readings drift with wiring noise, so treat calibration as
routine maintenance rather than a one-off setup step.

---

## Receiver-side calibration

Two things are calibrated on the receiver rather than the transmitter:

**Gyro Z offset** — measured automatically at startup with the car stationary.
Keep the car still through boot; an offset captured while moving will make it
drift under yaw assist.

**Turn test** — see [TUNING.md](TUNING.md). Sweeps motor PWM between 90 and 170
to find the duty that actually rotates the chassis on the current surface.
