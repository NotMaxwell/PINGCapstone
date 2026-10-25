# PING firmware: C++ port

An ESP-IDF version of the Rust firmware in `../src`, with the same main
program, the same four peripheral demos and the same pin map. It exists to see
what the project looks like in C++ on ESP-IDF. The Rust firmware is still the
real one.

Unlike the first attempt at this port, it writes as little as possible itself.
The libraries listed below do the drivers' heavy lifting, and ESP-IDF's own
drivers stand in for the hand-made HAL layer.

## What comes from where

| Part | Source | Written here |
|------|--------|--------------|
| BNO085 IMU (SPI, SH-2) | [esp32_BNO08x](https://github.com/myles-parfeniuk/esp32_BNO08x), which wraps CEVA's own [sh2](https://github.com/ceva-dsp/sh2) reference driver | `imu.cpp`: bring-up on our pins and the rotation-vector hand-off |
| NEO-M9N GPS (UBX) | [SparkFun u-blox GNSS v4](https://components.espressif.com/components/sparkfun/sparkfun_u-blox_gnss_v4), a native ESP-IDF component | `gps.cpp`: the CFG-VALSET we want and a plain `Solution` |
| SH1106/SSD1306 OLED | [u8g2](https://github.com/olikraus/u8g2): controllers, fonts, drawing | `display.cpp`: the `i2c_master` callback and the four-line text layout |
| TF03 LiDAR | none exists | `tf03_protocol.cpp` and `tf03.cpp`, ported from `src/tf03` |
| Target geometry | none needed | `geo.cpp`, ported from `src/geo.rs` |
| UART, SPI, I²C, GPIO, timing | ESP-IDF drivers and FreeRTOS | nothing |

The ESP-IDF component manager fetches the libraries on the first build
(`components/ping/idf_component.yml`). `dependencies.lock` pins exactly what
was fetched.

## Layout

```
CMakeLists.txt          ESP-IDF project
sdkconfig.defaults      ESP32-S3 N8R8, 1 kHz tick, USB-Serial-JTAG console
main/apps/              one file per application, each with its own app_main
  main.cpp              the PING program: button press -> target coordinate
  gps_demo.cpp  imu_demo.cpp  lidar_demo.cpp  lcd_demo.cpp
components/ping/        everything the applications share
  include/ping/constants.hpp   pin map and fixed settings (mirrors src/constants.rs)
test/host/              GoogleTest suite for the parts that touch no hardware
```

## Building and flashing

Needs ESP-IDF 6.0 or later; it has been built with 6.1.

```sh
. ~/Dev/Work/esp-idf/export.sh

# The main program
idf.py -B build/main flash monitor

# A demo: main, gps_demo, imu_demo, lidar_demo or lcd_demo
idf.py -B build/imu_demo -DPING_APP=imu_demo flash monitor
```

Each application gets its own build directory, so switching between them does
not rebuild everything. Logs, and the main program's `target #n: ...` lines,
come out of the USB port the board is flashed through.

## Host tests

```sh
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host
```

GoogleTest is fetched by CMake; nothing needs installing.

## Differences from the Rust firmware

- **Logging.** Diagnostics go through `ESP_LOG` on the USB serial port instead
  of RTT. The target lines share that port.
- **Display offset.** u8g2's SH1106 driver knows where that controller's glass
  sits, so `kLcdController` replaces the Rust `LCD_COLUMN_OFFSET`.
- **BNO085 WAKE pin.** esp32_BNO08x does not drive PS0/WAKE. The port holds it
  high so the hub comes up in SPI mode and leaves it there. The Rust driver
  pulses it low before each write. If writes to the hub turn out unreliable on
  hardware, look here first.
- **BNO085 over I²C.** Not available: esp32_BNO08x is SPI only, and the board
  is wired for SPI.
- **Button.** Polled every 5 ms rather than woken by an edge interrupt.
  esp32_BNO08x installs the shared GPIO ISR service itself, and polling keeps
  the two from contending over it.
- **Wi-Fi.** The Rust main program starts the Wi-Fi controller for a future
  web page but does not use it. This port leaves Wi-Fi out until there is
  something to use it for.

None of this has been run on hardware yet. Every application builds, and the
host tests pass.
