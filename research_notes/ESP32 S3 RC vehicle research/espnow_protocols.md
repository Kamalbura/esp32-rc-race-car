# ESP-NOW protocols and RC vehicle safety patterns

Research date: 2026-11-14. Firecrawl was not available in this environment; sources were collected with available web fetch/search tools and direct GitHub/HTTP retrieval. Local comparison is against `docs/PROTOCOL.md`, `receiver_code/receiver_code.ino`, and `transmitter_code/transmitter_code.ino` in this repository.

## Authentication, encryption, framing, and replay

### Takeaway
Use ESP-NOW encrypted unicast as the confidentiality/authentication boundary, but keep application framing, session state, and freshness checks. CCMP does not replace an application-level sequence/session policy, and ESP-NOW delivery success is only a MAC-layer result.

### Cited Findings
- ESP-NOW is connectionless and protects action frames with CCMP; PMK and per-peer LMK are each 16 bytes. PMK encrypts LMKs; an LMK is required for encrypted unicast, while multicast encryption is unsupported. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#security)
- Current Espressif documentation states a maximum of 20 peers, with encrypted peers no more than 17 and a default of 7; the build-time `CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM` controls the encrypted-peer limit. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#add-paired-device)
- ESP-NOW v1.0 payloads are limited to 250 bytes; v2.0 devices support 1470 bytes, but a v1-compatible design should remain at or below 250 bytes. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#frame-format)
- Espressif explicitly says send-callback success means MAC-layer receipt, not guaranteed application receipt; applications needing delivery confirmation should send an application ACK and retransmit on timeout. A sequence number can be used to drop duplicates. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#send-esp-now-data)
- Espressif's official example adds type/state/sequence/CRC/magic as application “extras,” parses CRC, and uses a queue between Wi-Fi callbacks and a lower-priority task. [Espressif official `wifi/espnow` example](https://github.com/espressif/esp-idf/tree/master/examples/wifi/espnow) · [example source](https://github.com/espressif/esp-idf/blob/master/examples/wifi/espnow/main/espnow_example_main.c)
- `RomanLut/hx_espnow_rc` is a credible RC-oriented implementation with 50 packets/s, bidirectional telemetry, CRC-protected telemetry, RSSI/A1/A2 reporting, and a documented one-second failsafe. [GitHub README](https://github.com/RomanLut/hx_espnow_rc)
- `OPAMAN123/2A-Trust` demonstrates shared-token plus monotonic-sequence replay checks, but its own README says the token is not cryptographically strong, firmware-visible, and unsuitable for production. It is useful as a teaching reference, not as a replacement for LMK/CCMP. [GitHub README](https://github.com/OPAMAN123/2A-Trust)

### Inferences
- The V2 fixed-MAC, encrypted-unicast design is materially stronger than broadcast or a plaintext shared token. CRC is still useful for rejecting struct/version mistakes, but it is not authentication.
- The 16-bit V2 sequence is adequate for a 100 Hz link when the active session is bounded by the 120 ms failsafe; nevertheless, explicitly define wrap behavior and reject stale/duplicate packets within a session. A sequence comparison is ambiguous if an outage permits a jump of 32,768 or more values.
- Never add a token or CRC as a claimed security upgrade. If keys are extracted from firmware, CCMP trust is lost; use provisioning/rotation or a stronger application AEAD only if the threat model requires it.

### Gaps
- Espressif documents CCMP and frame random values (for relay-attack resistance), but does not present ESP-NOW as a complete application replay/session protocol. The application must define reboot, counter persistence, and acceptance windows.

## Session/boot handling, watchdogs, callbacks, channel, sleep, and telemetry

### Takeaway
A safe vehicle needs two independent controls: a fast command-age failsafe and a firmware/task watchdog. Keep Wi-Fi callbacks bounded and queue work; pin both peers to the same channel and disable sleep for a low-latency always-on RC link unless a wake-window design is deliberately implemented.

### Cited Findings
- ESP-NOW peers must use the local channel (or channel 0 for current channel); encrypted unicast reception requires the receiver to add the peer and use the same LMK. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#add-paired-device)
- Espressif says send and receive callbacks execute in the high-priority Wi-Fi task; lengthy work must be posted to a queue. It also warns that too-short send intervals can disorder callbacks and recommends sending the next frame after the prior send callback returns. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#send-esp-now-data) · [receiving guidance](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#receiving-esp-now-data)
- Power saving is station-only and requires configuring both an ESP-NOW RX wake window and the connectionless-module wake interval; the default wake window keeps RX on. [Espressif ESP-NOW API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html#config-esp-now-power-saving-parameter)
- Espressif's official example allocates/copies receive data, queues callback events, and performs parsing and peer work in `example_espnow_task`, rather than in the Wi-Fi callback. [Official example source](https://github.com/espressif/esp-idf/blob/master/examples/wifi/espnow/main/espnow_example_main.c)

### Inferences
- Use a receiver command watchdog (for example 100–150 ms at a 10 ms control period) that forces hardware PWM/enable pins safe, plus an independent task watchdog that resets a wedged MCU. A link-age timeout cannot detect a dead main task if the last output remains energized.
- ACK telemetry should carry the highest accepted control sequence, session ID, receiver link state, command age, and optionally a receiver telemetry sequence. Treat `ESP_NOW_SEND_SUCCESS` as transport evidence only; use the echoed control sequence to measure loss and application freshness.
- For a 100 Hz drive link, the current `WIFI_PS_NONE`/`WiFi.setSleep(false)` choice is conservative and appropriate. Do not enable modem sleep without measuring worst-case command age and configuring wake windows on both devices.

### Gaps
- No single official timing value exists for an RC failsafe; 120 ms is an engineering choice that must be validated against drivetrain stopping distance, RF loss bursts, and the vehicle's maximum speed. A bench/drive test should record packet age, callback latency, and stop distance.

## Direct comparison with repository V2 and prioritized recommendations

### Takeaway
V2 already has strong fundamentals: encrypted unicast, fixed channel, PMK/LMK, packed bounded packets, CRC, session ID, sequence checks, kill bit, 120 ms receiver failsafe, telemetry ACK, and transmitter send-callback timeout. The largest safety gaps are callback work/state mutation, a session-persistence edge case, and the possibility that the transmitter's callback timeout permits late/out-of-order callback state to affect the send gate.

### Cited Findings
- V2's documented packet sizes and behavior are in [`docs/PROTOCOL.md`](../../docs/PROTOCOL.md): 10 ms controls, 120 ms receiver failsafe, 500 ms telemetry stale threshold, CRC-16/CCITT, session ID, sequence ACK, and kill bypassing the motor ramp.
- `receiver_code.ino` checks source MAC, exact packet length, version, CRC, session transition, safe kill/centered start, and modular sequence freshness in `onControlRecv`; it sets `lastPacketMs` only after acceptance ([source](../../receiver_code/receiver_code.ino)).
- `receiver_code.ino` sets Wi-Fi sleep off, `WIFI_PS_NONE`, and channel 6; it sends telemetry every 100 ms and includes the last acted-on control sequence ([source](../../receiver_code/receiver_code.ino)).
- `transmitter_code.ino` gates sends on `sendReady`, sends every 10 ms, has a 50 ms callback recovery timeout, rejects stale telemetry sequences, disables Wi-Fi sleep, and uses channel 6 ([source](../../transmitter_code/transmitter_code.ino)).

### Inferences
1. **Fix initial session persistence.** When `!haveSession`, the receiver accepts a safe packet and sets `activeSessionId`, but unlike the session-change branch it does not set `pendingSessionId/sessionPersistPending`. Persist the first accepted session too, and reject `sessionId == 0`. Keep the safe-kill + centered-stick gate for every new session.
2. **Move receive validation out of the Wi-Fi callback.** The current callback does MAC/length/version/CRC checks, `millis()`, a critical-section state transition, and packet copy. Replace this with a bounded copy into a preallocated queue/ring buffer (including source MAC and RSSI if needed); validate and commit state in the receiver task. Never call Preferences, motor/PWM functions, `Serial`, `delay`, or display code from callbacks.
3. **Make send completion generation-aware.** The 50 ms transmitter timeout intentionally recovers a lost callback, but a late callback can then update `lastSendOk/sendReady` for an earlier transmission. Add a send generation/sequence token, or use a one-outstanding-frame queue whose timeout records the transaction as expired and ignores its late callback. Keep the next-send interval above the documented minimum and instrument callback latency.
4. **Separate transport ACK from control ACK.** Retain `sequenceAck`, but add a telemetry sequence and counters for accepted, duplicate, stale-session, CRC, MAC, and send failures. Display “MAC send success” separately from “receiver acted on sequence N.” Do not retransmit old throttle commands blindly; if retransmission is used, only retransmit the newest command and apply a bounded deadline.
5. **Harden reboot and rollover tests.** Test receiver power-cycle with an old delayed packet, transmitter reboot while receiver is linked, repeated safe-start packets, sequence wrap, duplicate packets, CRC faults, channel mismatch, and telemetry loss. Require kill/center before accepting a new session and ensure NVS failure leaves the vehicle stopped rather than silently weakening policy.
6. **Protect the high-speed control loop.** Keep display/SPI, NeoPixel `show()`, serial logging, Preferences writes, and sensor work out of the 10 ms send path or bound them with measured budgets. On the receiver, evaluate command age and force the motor enable/PWM safe state before any cosmetic or telemetry work. Add an independent hardware emergency-stop/enable path where practical.
7. **Add an MCU task watchdog and startup inhibit.** Feed the watchdog only from the healthy control task; on reset, keep motor enables low until initialization, peer/channel setup, and a valid safe-start packet complete. Log reset reason and the maximum observed loop/callback duration.
8. **Validate RF/channel assumptions in deployment.** Channel 6 must match on both boards; avoid concurrently associating to an AP on another channel. Keep sleep disabled for this latency target, or implement and test explicit wake windows. Consider fixed peer PHY rate only after measuring packet age and range; long-range PHY settings trade throughput/latency for range and must be validated on the actual vehicle.

### Gaps
- This repository has no radio-loss/sequence-wrap/fault-injection test harness, no visible task-watchdog configuration in the reviewed V2 sketches, and no measured stop-distance or callback-latency dataset. Those measurements are required before selecting a tighter failsafe than 120 ms or claiming high-speed safety.
- The official API documents encrypted-peer limits as platform/configuration dependent (default 7, maximum 17 in the cited current documentation); verify the exact Arduino-ESP32/ESP-IDF version used by the ESP32-S3 build before relying on a limit.
