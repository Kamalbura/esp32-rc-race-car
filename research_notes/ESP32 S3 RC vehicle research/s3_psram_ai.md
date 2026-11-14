# ESP32-S3 RC vehicle AI research

## Practical applications and maintained examples

### Takeaway
ESP32-S3 is a credible companion-compute MCU for an RC vehicle when deterministic motor control is kept separate from optional computation. This project has no camera or microphone in its connected hardware. The relevant Espressif capabilities here are optimized DSP/Kalman filtering, bounded sensor logging, and future small classifiers; camera and audio capabilities are out of scope.

### Cited Findings
- Espressif's [ESP-NN](https://github.com/espressif/esp-nn) provides optimized neural-network kernels; ESP32-S3 uses assembly implementations that exploit its vector instructions. Espressif's published measurements report INT8 person detection at about 54 ms per invoke on a 240 MHz ESP32-S3 with ESP-NN (versus about 2,300 ms with reference kernels), with the tensor arena in SPIRAM.
- The same [ESP-NN performance table](https://github.com/espressif/esp-nn) reports approximately 729 ms for MobileNetV3 Small (224x224x3) and 1,253 ms for YOLO11n (192x192x3) on ESP32-S3 using optimized kernels and PSRAM; dual-core figures are approximately 690 ms and 1,053 ms. These are inference times, not complete camera-to-actuator latency.
- [ESP-DSP](https://github.com/espressif/esp-dsp) is Espressif's official component. It includes optimized vector math, FFT, FIR/IIR, matrix, and Kalman APIs, with ESP32-S3 optimizations and examples. The [Kalman example](https://github.com/espressif/esp-dsp/tree/master/examples/kalman) contains a 13-state IMU EKF example (`ekf_imu13states_main.cpp`) suitable as a starting point for gyro/accelerometer attitude or yaw-rate fusion, although its input is simulated and must be replaced with a real IMU driver.
- Espressif's maintained [esp-tflite-micro](https://github.com/espressif/esp-tflite-micro) provides ESP-IDF examples including `hello_world`, `micro_speech`, and `person_detection`; its component integration can use ESP-NN optimized kernels. This is appropriate for small INT8 classifiers, command recognition, visual wake-word/person detection, and anomaly models—not for high-rate autonomous driving perception.
- [ESP-SR](https://github.com/espressif/esp-sr) and [ESP-Skainet examples](https://github.com/espressif/esp-skainet/tree/master/examples) provide WakeNet, audio front end, and speech-command examples for ESP32-S3. The [ESP-SR benchmark](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/benchmark/README.html) lists representative WakeNet resource use: roughly 16–20 KB internal RAM plus 324–523 KB PSRAM for newer quantized WakeNet9/10 configurations, with about 3.0–7.1 ms per 32-ms frame (model/version/configuration dependent).
- The official [esp32-camera](https://github.com/espressif/esp32-camera) driver supports ESP32-S3 camera sensors and recommends JPEG capture for performance. Its README notes that PSRAM is needed for high resolutions or non-JPEG formats above CIF; `fb_count` and frame-buffer location directly trade latency and memory.
- [esp-who](https://github.com/espressif/esp-who) contains maintained ESP32-S3 object-detection examples, including [`examples/object_detect`](https://github.com/espressif/esp-who/tree/master/examples/object_detect) with pedestrian, face, cat, and dog detection models and S3 board/BSP configurations.
- For audio/sensor expansion, [esp-iot-solution](https://github.com/espressif/esp-iot-solution) provides Espressif-maintained drivers and examples for sensors, audio, displays, and actuators. Use ESP-IDF I2S/I2C/SPI peripherals for microphones, IMUs, ToF/range sensors, and displays, and use [ESP-DSP's Kalman example](https://github.com/espressif/esp-dsp/blob/master/examples/kalman/main/ekf_imu13states_main.cpp) for the fusion algorithm.

### Inferences
- Good applications for this hardware are: (1) timestamped MPU6050 history and calibration capture; (2) IMU + command telemetry fusion for yaw-rate assist, slip/stall detection, and smoother display telemetry; (3) bounded local data logging and post-run tuning; and (4) small non-safety anomaly or classification experiments.
- Avoid using camera inference as the sole collision-avoidance or stop mechanism. A watchdog, radio timeout, current/stall guard, and obstacle sensor should be independent and able to force motor PWM to zero.

### Gaps
- ESP-NN benchmark values are model/configuration measurements, not guarantees for a particular carrier board, camera driver, Wi-Fi load, or Arduino build. Benchmark the complete pipeline on the actual board.
- The requested “remote” and “car” board names were not supplied. The capacity comparison below treats them as ESP32-S3 module configurations N8R2 (8 MB flash/2 MB PSRAM) and N16R8 (16 MB flash/8 MB PSRAM); verify module markings and actual usable partition sizes.

## Memory placement and real-time safety

### Takeaway
PSRAM is a large, cached working store, not deterministic real-time RAM. Keep the motor failsafe/PWM path, interrupt handlers, small queues, and time-critical sensor state in internal IRAM/DRAM; place model arenas, camera buffers, audio buffers, and logs in PSRAM only when their latency is acceptable.

### Cited Findings
- [ESP-IDF external-RAM documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/external-ram.html) states that ESP32-S3 has only a few hundred KB of internal RAM and maps external PSRAM for heap/data use. PSRAM access is cache-mediated.
- [ESP-IDF flash/PSRAM configuration](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/flash_psram_config.html) documents that flash and PSRAM share the MSPI/cache address space and that mapping is constrained by the virtual memory map. Enabling PSRAM does not make it equivalent to internal SRAM.
- [ESP-IDF memory-types guidance](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html) and [IRAM-safe interrupt guidance](https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/system/intr_alloc.html) require IRAM-safe interrupt code and internal data when an ISR must run with cache disabled. Constants used by such code must not unexpectedly reside in flash/PSRAM.
- [ESP-IDF SPI-flash concurrency guidance](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_flash/spi_flash_concurrency.html) explains that flash operations can suspend cache access; code/data needed during that interval must be internal. The practical consequence is to avoid OTA/flash writes during a driving-critical section, or guarantee the failsafe path is entirely internal.
- The [ESP-IDF XIP-from-PSRAM example](https://github.com/espressif/esp-idf/tree/master/examples/system/xip_from_psram) demonstrates optional placement of instruction/rodata in PSRAM, but this changes cache and latency assumptions and should not be used for a hard real-time safety path without measurement.

### Inferences
- Suggested placement for this RC vehicle: IRAM/DRAM for PWM update/failsafe, radio timeout bookkeeping, watchdog service, the latest MPU6050 sample, filter state, and latest safety command; internal DMA-capable buffers for peripherals that require them; PSRAM for a bounded IMU ring buffer, calibration captures, telemetry history, diagnostic logs, and optional model arenas; flash for firmware and model files loaded at startup.
- Use capability-aware allocation (`MALLOC_CAP_INTERNAL`, `MALLOC_CAP_DMA`, and `MALLOC_CAP_SPIRAM`) and inspect the map/heap at runtime. Do not pass PSRAM pointers to an ISR that may run with cache disabled.
- Partition work: a high-priority control task (for example 100–500 Hz) computes a bounded motor command from the latest validated inputs; camera/audio/AI tasks publish timestamped observations into queues. AI may drop frames rather than block the control task.

### Gaps
- Exact internal-RAM availability depends on ESP-IDF version, Wi-Fi/BLE, camera driver, and enabled components. Measure free internal heap and stack high-water marks after all features are enabled.

## 2 MB PSRAM/8 MB flash remote versus 8 MB PSRAM/16 MB flash car

### Takeaway
The smaller N8R2 remote can comfortably run radio/control UI, display, telemetry, light DSP, and one small TFLite Micro or WakeNet workload, but should not be the camera/AI hub. The larger N16R8 car board can host camera buffers plus WakeNet/audio and a modest INT8 vision model, but still cannot provide low-latency, high-frame-rate modern object detection; its safety controller must remain independent.

### Cited Findings
- ESP32-S3 module suffix conventions are documented by Espressif's [module and hardware documentation](https://www.espressif.com/en/products/socs/esp32-s3). In this note, N8R2/N16R8 are shorthand for the user-provided 8/2 and 16/8 MB flash/PSRAM configurations; board vendors may reserve different partition space.
- A raw RGB565 frame costs width x height x 2 bytes: QVGA (320x240) is about 150 KiB and VGA (640x480) about 600 KiB before additional frame buffers, DMA descriptors, conversion tensors, and JPEG decode workspace. This arithmetic explains why JPEG + one frame buffer is the safer starting point on 2 MB PSRAM; the camera driver's [PSRAM/fb_count guidance](https://github.com/espressif/esp32-camera) should govern the final configuration.
- A 32-ms mono 16-kHz int16 audio frame is 1,024 bytes; seconds of buffering and multi-channel AFE state are therefore inexpensive compared with camera frames, while WakeNet model working memory is documented at hundreds of KB in [ESP-SR benchmarks](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/benchmark/README.html).
- Espressif's [ESP-NN measurements](https://github.com/espressif/esp-nn) show that even optimized YOLO11n on S3 is around 1.25 seconds per invoke in the cited setup, while person detection is around 54 ms. This bounds realistic use more strongly than raw PSRAM size.

### Inferences
- **N8R2 remote (8 MB flash, 2 MB PSRAM):** reserve internal RAM for control and display drivers; use PSRAM for a small audio ring, telemetry history, and one compact tensor arena. Practical features: encrypted ESP-NOW, joystick/display, IMU yaw-rate assist, Kalman/complementary filtering, FFT diagnostics, WakeNet9/10-style voice trigger, or a small command/classification model. Use low-resolution JPEG snapshots only as an occasional diagnostic; do not run continuous camera inference while maintaining the transmitter UI/radio deadline. Keep model assets and OTA slots conservative because 8 MB flash is quickly consumed by dual OTA partitions, filesystem, and model blobs.
- **N16R8 car (16 MB flash, 8 MB PSRAM):** use PSRAM for one or two JPEG frame buffers, audio buffers, a larger tensor arena, and model scratch space; use 16 MB flash for a larger application plus model partition and filesystem/OTA headroom. Practical features: low-resolution camera obstacle/person warning, WakeNet plus command recognition, IMU + wheel/command fusion, and local logging. Run camera inference at a deliberately low rate (for example 1–5 Hz depending on measured model and capture latency), while the motor safety loop runs independently at a fixed deadline.
- The extra 6 MB PSRAM is mainly capacity and buffering headroom, not a proportional speed increase: PSRAM remains cached/shared-bus memory, and model compute is dominated by kernel operations and memory traffic. The extra 8 MB flash expands model/OTA/storage options but does not make flash execution deterministic.
- For the current project hardware, keep MPU6050 sampling and the motor command/failsafe path on the receiver; send fused telemetry or AI advisories over the existing authenticated ESP-NOW link. A remote-side voice/UI command should be treated as an ordinary input subject to the receiver's timeout, sequence, and stall guards.

### Gaps
- No board schematic, module marking, partition table, camera sensor, microphone count, or exact model is available here, so capacity numbers are planning bounds rather than a certification. Validate with `heap_caps_get_free_size`, stack high-water marks, frame-buffer allocation logs, and worst-case end-to-end control latency.

## Research limitations and design checklist

### Takeaway
Firecrawl was not available in this environment. Research used available web-search/web-fetch tools and direct official Espressif documentation/GitHub pages; URLs are recorded so the claims can be rechecked against the current upstream revisions.

### Cited Findings
- Official primary sources used: [ESP-IDF external RAM](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/external-ram.html), [flash/PSRAM configuration](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/flash_psram_config.html), [memory types](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/memory-types.html), [ESP-NN](https://github.com/espressif/esp-nn), [ESP-DSP](https://github.com/espressif/esp-dsp), [TFLite Micro](https://github.com/espressif/esp-tflite-micro), [ESP-SR](https://github.com/espressif/esp-sr), [esp32-camera](https://github.com/espressif/esp32-camera), and [esp-who](https://github.com/espressif/esp-who).

### Inferences
- Recommended implementation order: (1) preserve and measure radio/motor failsafe; (2) add IMU fusion and logging; (3) add camera/audio capture with bounded queues; (4) add one model at a time; (5) measure worst-case latency under Wi-Fi, display, logging, and flash/OTA conditions before allowing any AI output to influence speed.
- Safety rule: AI can request “slow/stop,” but only an internal, deterministic safety path should decide whether motors are allowed to run.

### Gaps
- Firecrawl-specific extraction and cross-source crawling could not be performed because Firecrawl is unavailable. Some upstream README benchmark values and model memory figures can change with releases; pin component versions and rerun measurements for the production firmware.
