# Rules for AI agents working in firmware-cpp

## Project
The C++ port of the PING firmware: aim the device, press the button, get the target's GPS
coordinate. ESP32-S3-DevKitC-1-N8R8 with a BNO085 IMU (SPI), NEO-M9N GPS (UART), TF03 LiDAR
(UART) and an SH1106 OLED (I²C). The Rust firmware in `../src` is the behavioural reference
and has run on the hardware; do not edit it. `../firmware-zig` is another session's port; do
not edit it either.

Goal: firmware that cannot fail because of a driver. Every peripheral error is reported and
recovered from at runtime; nothing in a sensor path aborts the chip.

## Hard rules
- All work goes to `NotMaxwell/PINGCapstone` (remote `origin`), on the session's branch.
  Never push to `main`; never open PRs unless the user asks.
- Runtime is ESP-IDF's FreeRTOS: one task per sensor, plain blocking calls with timeouts.
  No bare-metal loops, no async executors, no extra RTOS layers.
- Toolchain is pinned: ESP-IDF v6.1.1, built with esp-clang (`IDF_TOOLCHAIN=clang`); host
  tests use the system clang through `CMakePresets.json`. GCC (`IDF_TOOLCHAIN=gcc`) must keep
  building too.
- Only ESP-IDF's current drivers (`esp_driver_*`: `i2c_master`, `spi_master`, `uart`,
  `gpio`, `usb_serial_jtag`). No legacy `driver/i2c.h`, no Arduino layers.
- Drivers: manufacturer code wherever it exists, so sensor dropouts seen on the Rust port can
  be pinned on hardware rather than our code. CEVA's `sh2` (vendored in `components/sh2`,
  unchanged) for the BNO085, over a port of CEVA's reference SPI HAL that keeps its states
  and timings (deviations are listed in `shtp_spi.cpp`); our own UBX code for the NEO-M9N; our own TF03 code; u8g2 for the display. Add a
  third-party library only when the user agrees.
- No `ESP_ERROR_CHECK` (or anything else that aborts) after start-up: check every
  `esp_err_t`, log it with `esp_err_to_name`, and retry or re-initialise. `ESP_ERROR_CHECK`
  is acceptable only in `app_main` for setup that cannot fail on a good board.
- No exceptions, no RTTI-dependent code, no heap allocation in steady-state loops.
  Errors are `std::expected<T, Error>`.
- Comment only what the code does not make obvious: why a choice was made, a non-obvious
  invariant, a deliberate deviation from the Rust firmware. No narration comments.
- Report honestly: failing builds/tests are stated with output; skipped steps are named.
  Nothing here has been on hardware unless the user says so.
- Ask the user when a decision is genuinely theirs (pins, hardware, deleting things,
  pushing anywhere new). Otherwise pick the sensible default, say so, and keep going.

## Layout
- `components/ping/include/ping/settings.hpp` — ESP-IDF-free settings (mirrors `src/constants.rs`).
- `components/ping/include/ping/constants.hpp` — pin map and buses; includes settings.
- Hardware-free modules (host tested): `geo`, `mag_cal`, `tf03_protocol`, `ubx`, `shtp_spi`.
- Hardware modules: `imu` (sh2 + `bno085_hal`), `gps`, `tf03`, `display`, `rtos`.
- `main/apps/*.cpp` — one application per file, each with its own `app_main`; pick with
  `-DPING_APP=<name>`.
- `test/host/test_*.cpp` — GoogleTest, picked up automatically.

## Build and verify
See `BUILD.md` for the full list. The short version:
- `cmake --preset host && cmake --build --preset host && ctest --preset host` — host tests.
- `. $IDF_PATH/export.sh && IDF_TOOLCHAIN=clang idf.py -B build/main build` — firmware.
- Build every app (`main gps_demo imu_demo lidar_demo lcd_demo`) before saying it builds.
- Scratch files go in `.scratch/` (git-ignored), never `/tmp`.

## Hardware-free code
- Must not include ESP-IDF or FreeRTOS headers; test it on the host.
- Bus access goes through a small interface so the protocol logic can be tested with a fake.

## Subagents and cost
- The lead agent manages: defines interfaces, delegates modules, integrates, verifies.
- Use the cheapest model that can do the job: Haiku for boilerplate, Sonnet for ordinary
  module work, Opus for hard algorithmic or driver work, Fable only for high-reasoning tasks.
- Give each subagent one owned file set and a way to verify alone; do not have two agents
  edit the same file. Keep prompts and reports short; do not re-read what is already known.
