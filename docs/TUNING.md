# Tuning — drive, steering and yaw assist

Every constant here was arrived at by driving the car, not by simulation. The
reasoning is recorded so the values can be re-derived if the hardware changes.

---

## Motion profile

| Constant | Value | Meaning |
|---|---|---|
| `MOTOR_PWM_FREQ` | 20000 Hz | Above audible range — no drivetrain whine |
| `MOTOR_PWM_BITS` | 8 | 0–255 duty, matching the speed-limit range end to end |
| `MOTOR_DEADBAND` | 0.03 | Stick noise below 3 % is treated as centre |
| `MOTOR_ACCEL_PWM_PER_SEC` | 650 | Ramp up |
| `MOTOR_DECEL_PWM_PER_SEC` | 1000 | Ramp down |
| `MOTOR_REVERSE_BRAKE_PWM_PER_SEC` | 1500 | Crossing zero into reverse |
| `CALIBRATION_SPEED_SCALE` | 0.5 | Calibration mode halves output |

The three ramp rates are deliberately ordered
`accel < decel < reverse-brake`. Accelerating slowly protects the gearbox and
limits inrush current. Stopping is allowed to be more aggressive than starting,
because a slow stop is a safety problem. Direction reversal is fastest of all —
the H-bridge is being asked to reverse against rotation, and lingering in that
state is the worst case for both the driver and the motor.

**Kill and failsafe ignore all three.** They cut to zero immediately.

---

## Steering model (V7)

Steering used to be applied directly: stick position mapped straight to a
differential. That is fine at walking pace and unpleasant at speed — the same
stick deflection that gives a usable turn at low speed will spin the car when
it is moving.

Two mechanisms fix it.

### Speed-dependent authority

| Constant | Value |
|---|---|
| `STEERING_GAIN_LOW_SPEED` | 1.10 |
| `STEERING_GAIN_HIGH_SPEED` | 0.45 |

Steering gain is interpolated between these by throttle magnitude. Full
authority — slightly over unity — is available for tight manoeuvring near
standstill, falling to under half at speed where the car needs stability more
than it needs rotation.

### Asymmetric slew limiting

| Constant | Value |
|---|---|
| `STEERING_ENGAGE_RATE_PER_SEC` | 1.80 |
| `STEERING_RELEASE_RATE_PER_SEC` | 3.50 |

`applySteeringRateLimit()` rate-limits the steering command rather than
applying it instantly, and **engagement is slower than release**:

```c
bool engaging = fabs(targetSteering) > fabs(filteredSteering);
float engageRate = STEERING_ENGAGE_RATE_PER_SEC * (1.0f - 0.35f * throttleAbs);
if (engageRate < 1.0f) engageRate = 1.0f;
float rate = engaging ? engageRate : STEERING_RELEASE_RATE_PER_SEC;
```

Turn-in is smoothed so a stick flick cannot snap the car sideways, and
engagement slows by a further 35 % at full throttle. Straightening out is
nearly twice as fast, because recovering from a slide must never be the slow
direction. The engage rate is floored at 1.0/s so the car never becomes
unresponsive.

`dt` is clamped to 20 ms if a loop iteration overruns, so a scheduling hiccup
cannot produce one enormous steering step.

### Straight assist and stall guard

| Constant | Value | Purpose |
|---|---|---|
| `STRAIGHT_ASSIST_STEER_WINDOW` | 0.10 | Inside this band, treat as straight |
| `STALL_GUARD_THROTTLE_MIN` | 0.30 | Below this throttle, apply the floor |
| `STALL_GUARD_MIN_PWM` | 55 | Minimum duty that actually turns the wheels |

The stall guard is a direct consequence of the motor change described in
[HARDWARE.md](HARDWARE.md). The 555 geared motors do not reliably start under
load below roughly 20 % duty; without a floor, small throttle inputs produce
current draw and heat but no motion.

---

## Yaw assist (MPU6050)

Present in `receiver_code_advanced_mpu` and **disabled by default**
(`ENABLE_YAW_ASSIST 0`).

| Constant | Value |
|---|---|
| `MAX_TARGET_YAW_DPS` | 220.0 |
| `YAW_ASSIST_GAIN` | 0.003 |

Steering input is interpreted as a *requested yaw rate* rather than a raw
differential, and the measured gyro Z rate is fed back with a deliberately
small gain. The intent is to hold a commanded rate of rotation on surfaces
where one wheel has less traction than the other.

Gyro Z offset is measured at startup with the car stationary and subtracted
from every reading — an uncalibrated MPU6050 has a bias large enough to make
the car creep on its own.

### Automated turn test

| Constant | Value |
|---|---|
| `TURN_TEST_MIN_PWM` | 90 |
| `TURN_TEST_MAX_PWM` | 170 |
| `TURN_TEST_TIMEOUT_MS` | 5000 |
| `TURN_TEST_TOLERANCE_DEG` | 4.0 |
| `TURN_TEST_SETTLE_DPS` | 18.0 |

Commands a rotation, then uses gyro integration to decide whether the car
reached the target heading within 4°, treating the manoeuvre as complete once
the yaw rate falls below 18 °/s. Sweeping PWM between 90 and 170 finds the duty
that actually rotates the chassis on a given surface — useful because that
threshold changes between carpet, concrete and grass.

---

## Retuning after hardware changes

1. Re-run the guided calibration ([CALIBRATION.md](CALIBRATION.md)) — axis
   centres shift whenever the harness is touched.
2. Find the new stall floor: raise throttle slowly until the wheels turn
   reliably under the car's own weight, and set `STALL_GUARD_MIN_PWM` there.
3. Re-check the ramp rates. Heavier packs or higher-torque motors want a lower
   `MOTOR_ACCEL_PWM_PER_SEC`.
4. Re-run the turn test on the surface you actually drive on.
