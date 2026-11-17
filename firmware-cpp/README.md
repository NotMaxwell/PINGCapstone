# PING firmware: C++ port

An ESP-IDF version of the Rust firmware in `../src`, with the same main
program, peripheral demos and pin map. It runs on ESP-IDF's FreeRTOS: one task
per sensor, each with blocking calls that time out. The Rust firmware is still
the reference and the one that has run on hardware.

**Build, flash, test and debug commands are in [BUILD.md](BUILD.md).**
Rules for AI agents working here are in [CLAUDE.md](CLAUDE.md).

## Driver choices

The aim is that no driver can be the reason the device does not work. So the
drivers are either the chip vendor's own reference code or a port of the Rust
drivers that have run on this hardware, built only on ESP-IDF's current
(`esp_driver_*`) APIs. Every error is reported and recovered from at runtime.

| Part | Source | Written here |
|------|--------|--------------|
| BNO085 IMU (SPI, SH-2) | CEVA's [sh2](https://github.com/ceva-dsp/sh2) reference driver v1.4.0, vendored unchanged in `components/sh2` | `shtp_spi.cpp`: the SPI transport, matching the Rust driver's INT/WAKE/RST handling, tested against a fake bus. `bno085_hal.cpp`: that transport on ESP-IDF's SPI master. `imu.cpp`: bring-up, reports, events |
| NEO-M9N GPS (UBX) | none | `ubx.cpp`: UBX framing, CFG-VALSET, NAV-PVT/MON-VER/ACK decoding, host tested. `gps.cpp`: the receiver on ESP-IDF's UART driver |
| TF03 LiDAR | none exists | `tf03_protocol.cpp`, `tf03.cpp`, ported from `src/tf03` |
| SH1106/SSD1306 OLED | [u8g2](https://github.com/olikraus/u8g2) 2.37.1: controllers, fonts | `display.cpp`: the `i2c_master` callback, bus recovery, text layout |
| Compass | none | `mag_cal.cpp`, ported from `src/mag_cal.rs`: hard/soft iron fitting, start-up hard iron calibration, heading from the game rotation vector |
| Target geometry | none | `geo.cpp`, ported from `src/geo.rs` |

u8g2 is the only library fetched at build time (pinned in
`components/ping/idf_component.yml`, locked in `dependencies.lock`).

What changed from the previous version of this port, which used the
esp32_BNO08x wrapper and SparkFun's u-blox library:

- **BNO085 WAKE pin.** Now pulsed low before every write and waited on, as the
  Rust driver and CEVA's SPI notes do. esp32_BNO08x only held it high.
- **GPIO ISR service.** Installed once and shared; the IMU's INT handler no
  longer fights other users for it.
- **Compass.** The bearing comes from the game rotation vector and the ESP32's
  own magnetometer correction (`mag_cal`), as in the Rust firmware, instead of
  the hub's rotation vector. It includes `kHeadingCorrectionDeg` and the hard
  iron calibration found during handling after power up.
- **Fallback position, display task, compass and GPS logging**, as in the
  Rust main program.
- **Failure handling.** No `ESP_ERROR_CHECK` after start-up. Sensor tasks
  retry bring-up forever, restart a sensor that goes silent, flush a UART that
  overflowed, clock a stuck I²C bus free, and feed the task watchdog, so a
  driver call that hangs anyway restarts the chip.

## Layout

```
CMakeLists.txt          ESP-IDF project; also the host tests with -DPING_HOST_TESTS=ON
CMakePresets.json       one preset per application / toolchain / host test build
sdkconfig.defaults      ESP32-S3 N8R8, 1 kHz tick, USB-Serial-JTAG console, task watchdog
sdkconfig.debug         layered on top by the main-debug preset
main/apps/              one file per application, each with its own app_main
  main.cpp              the PING program: button press -> target coordinate
  gps_demo.cpp  imu_demo.cpp  lidar_demo.cpp  lcd_demo.cpp
components/ping/        everything the applications share
  include/ping/constants.hpp   pin map and buses (mirrors src/constants.rs)
  include/ping/settings.hpp    rates, aiming, calibration, timing (no ESP-IDF)
components/sh2/         CEVA's SH-2 driver, vendored
test/host/              GoogleTest suite for the parts that touch no hardware
.clangd .vscode/ .nvim.lua     editor setup, see BUILD.md
```

## Differences from the Rust firmware

- **Logging.** `ESP_LOG` on the USB serial port instead of RTT. The target
  lines share that port.
- **Display offset.** u8g2 knows where the SH1106's glass sits, so
  `kLcdController` replaces the Rust `LCD_COLUMN_OFFSET`.
- **BNO085 over I²C.** Not implemented: the board is wired for SPI.
- **Button.** Polled every 5 ms rather than woken by an edge interrupt.
- **Wi-Fi.** Left out until there is something to use it for.
- **Calibration tools.** `imu_cal`, `heading_cal`, `compass_diag` and
  `gps_diag` are not ported yet; use the Rust binaries to measure
  `kMagCalibration` and `kHeadingCorrectionDeg`.

None of this has been run on hardware yet. Every application builds with
esp-clang and GCC, and the host tests pass.
