# Radio protocol

Generation 2 replaces the commercial RC receiver with a direct ESP-NOW link
between two ESP32-S3 boards. This document describes the wire format and the
link behaviour.

---

## Why ESP-NOW

A commercial RC receiver gives you PWM channels and nothing else: no return
path, no packet integrity, no way to tell a lost frame from a centred stick.
ESP-NOW is connectionless — no association, no DHCP, no TCP handshake — so
latency is low and predictable, while still allowing a structured payload and
a telemetry return path.

Constraints it imposes:

- 250 byte maximum payload. Both packet structs are `static_assert`-ed against it.
- Both peers must sit on the same Wi-Fi channel (`ESPNOW_CHANNEL 6` here).
- Peers are addressed by MAC. There is no discovery, so both MACs are compiled
  in via `secrets.h`.

---

## Encryption

ESP-NOW encryption is enabled with a Primary Master Key and a Local Master Key:

| Key | Length | Role |
|---|---|---|
| PMK | 16 bytes | Encrypts the LMK |
| LMK | 16 bytes | Encrypts payloads for this specific peer |

Both ends must carry identical keys. They live in a gitignored `secrets.h` in
each sketch folder — see [`secrets.h.example`](../transmitter_code/secrets.h.example).
Transmission is **unicast to a fixed MAC**, not broadcast, so an unpaired
listener sees encrypted frames addressed elsewhere.

---

## Control packet — transmitter to receiver

Sent every **10 ms (100 Hz)**.

```c
struct __attribute__((packed)) ControlPacket {
  uint8_t  version;      // RC_PACKET_VERSION, currently 1
  uint16_t sequence;     // increments per packet, wraps at 65535
  int16_t  throttle;     // -1000 reverse .. 0 stop .. +1000 forward
  int16_t  steering;     // -1000 left .. 0 centre .. +1000 right
  uint16_t speedLimit;   // 0..255, set by the encoder
  int16_t  aux1;         // mapped auxiliary axis
  int16_t  aux2;         // mapped auxiliary axis / mode source
  uint8_t  mode;         // 1..3
  uint8_t  buttons;      // bit mask, see below
  uint16_t crc;          // CRC-16/CCITT over all preceding bytes
};
```

Button mask:

| Bit | Constant | Meaning |
|---|---|---|
| `0x01` | `BTN_KILL` | Kill latched — motors forced to stop |
| `0x02` | `BTN_LIGHTS` | Lighting on |
| `0x04` | `BTN_AUX1` / `BTN_CALIBRATION` | Calibration mode |

Throttle and steering are transmitted as ±1000 integers rather than floats:
fixed-point keeps the packet small and avoids any float representation
mismatch between the two boards.

---

## Telemetry packet — receiver to transmitter

```c
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  version;
  uint16_t sequenceAck;  // last control sequence the receiver acted on
  uint16_t batteryMv;    // pack voltage in millivolts
  int16_t  leftMotor;    // effective PWM after ramping and limits
  int16_t  rightMotor;
  uint8_t  linkState;
  uint16_t packetAgeMs;  // age of the newest control packet
  uint16_t crc;
};
```

`sequenceAck` echoing the control `sequence` is what makes packet loss
measurable: the transmitter compares what it sent against what was
acknowledged, and reports missed counts. `leftMotor` and `rightMotor` report
the **effective** PWM after ramping, speed limit and calibration scaling, so
the display distinguishes requested (`spdReq`) from actual (`spdEff`).

---

## Integrity — CRC-16/CCITT

Every packet carries a CRC-16/CCITT (polynomial `0x1021`, initial value
`0xFFFF`) over all bytes preceding the field. A packet whose CRC does not match
is discarded without updating the last-packet timestamp, so corruption is
treated exactly like silence and feeds the failsafe rather than producing a
glitched control input.

ESP-NOW already has a frame CRC beneath this. The application-level CRC guards
against corruption introduced above the radio — a struct packing mismatch or a
version skew between the two firmwares.

---

## Failsafe

| Parameter | Value |
|---|---|
| Control send period | 10 ms |
| Receiver failsafe timeout | **120 ms** |
| Telemetry stale threshold (transmitter) | 500 ms |

If the receiver goes 120 ms without a valid packet — twelve consecutive
missed sends — it stops the motors immediately. The timeout is deliberately a
multiple of the send period so that ordinary jitter never trips it, while a
genuine link loss is caught in under a fifth of a second.

Kill and failsafe both bypass the acceleration ramp entirely. Ramping exists to
protect the drivetrain during normal driving; when the link drops or the
operator hits kill, the correct behaviour is to stop now.

---

## Timing and core allocation

The transmitter splits work across both ESP32-S3 cores:

| Core | Responsibility |
|---|---|
| Core 1 | Read ADS1115 axes, build and send the control packet, update status LED |
| Core 0 | `displayTask` — SPI display rendering, serial status output |

SPI writes to the ST7735 are slow enough to disturb a 100 Hz control loop if
run inline. Isolating rendering on the other core keeps the send interval
stable regardless of what is on screen.
