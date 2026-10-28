# PING firmware: Zig port

A Zig version of the C++ port in `../firmware-cpp` (itself a port of the Rust
firmware in `../src`), with the same main program, the same four peripheral
demos and the same pin map. It runs on ESP-IDF and its FreeRTOS, and uses the
vendors' own C drivers where they exist. The Rust firmware is still the real
one.

## How Zig gets onto an ESP32-S3

Upstream Zig has no Xtensa backend, so it cannot produce ESP32-S3 machine code
itself. Instead, `zig build firmware` compiles one application with Zig's **C
backend** into a single C file, `ping.c`, and ESP-IDF's GCC compiles that
alongside everything else. `idf.py build` runs the Zig step for you
(`main/CMakeLists.txt`). Plain Zig 0.17 from mise is enough; no Espressif fork
of Zig or LLVM is involved. `build.zig`'s header comment explains why the C is
generated for `arm-freestanding-eabi`.

ESP-IDF, FreeRTOS and the C drivers are called directly through `translate-c`
of the headers in `src/c/`. There is no Zig HAL in between; `src/rtos.zig` and
`src/gpio.zig` are thin helpers over ESP-IDF. The CMake side asks GCC for its
include directories and macros (including the picolibc headers ESP-IDF 6 uses)
so the headers are translated exactly as GCC sees them.

## What comes from where

| Part | Source | Written here |
|------|--------|--------------|
| BNO085 IMU (SPI, SH-2) | CEVA's [sh2](https://github.com/ceva-dsp/sh2) v1.4.0, vendored unchanged in `components/sh2` | `imu.zig`: the `sh2_Hal_t` over ESP-IDF SPI and the INT/RST/WAKE pins |
| NEO-M9N GPS (UBX) | u-blox [ubxlib](https://github.com/u-blox/ubxlib) v1.5.0, only its UBX protocol file and GNSS headers, vendored in `components/ubxlib_ubx` | `ubx.zig` (CFG-VALSET, NAV-PVT, MON-VER, ACK) and `gps.zig` (UART) |
| SH1106/SSD1306 OLED | [u8g2](https://github.com/olikraus/u8g2): controllers, fonts, drawing | `display.zig`: the `i2c_master` callback and the four-line text layout |
| TF03 LiDAR | none exists | `tf03_protocol.zig` and `tf03.zig`, ported from `src/tf03` |
| Target geometry | none needed | `geo.zig`, ported from `src/geo.rs` |
| UART, SPI, I²C, GPIO, timing | ESP-IDF drivers and FreeRTOS | nothing |

The component manager fetches u8g2 on the first build
(`main/idf_component.yml`); `dependencies.lock` pins it.

## Layout

```
build.zig               zig build test / zig build firmware
CMakeLists.txt          ESP-IDF project
sdkconfig.defaults      ESP32-S3 N8R8, 1 kHz tick, USB-Serial-JTAG console
src/apps/               one file per application, each exporting app_main
  main.zig              the PING program: button press -> target coordinate
  gps_demo.zig  imu_demo.zig  lidar_demo.zig  lcd_demo.zig
src/                    everything the applications share (the `ping` module)
  constants.zig         pin map and fixed settings (mirrors src/constants.rs)
src/c/                  C headers translated into the idf, sh2, ubx and u8g2 modules
main/                   the ESP-IDF component: builds ping.c, plus small C helpers
  idf_config.c          ESP-IDF config structs with bitfields, which translate-c cannot fill in
  int128.c              128-bit integer helpers the 32-bit libgcc lacks (Zig's float formatting uses them)
components/             the vendored sh2 and ubxlib sources
```

## Building and flashing

Needs Zig 0.17.0 (`mise install` in the repo root) and ESP-IDF 6.x; it has
been built with 6.1.

```sh
. ~/Dev/Work/esp-idf/export.sh

# The main program
idf.py -B build/main flash monitor

# A demo: main, gps_demo, imu_demo, lidar_demo or lcd_demo
idf.py -B build/imu_demo -DPING_APP=imu_demo flash monitor
```

Each application gets its own build directory. Logs, and the main program's
`target #n: ...` lines, come out of the USB port the board is flashed through.

## Host tests

```sh
zig build test
```

Runs the geometry, TF03 protocol and UBX tests on the host. The UBX tests run
against ubxlib's C, built for the host.

## Differences from the other ports

- **Logging.** Zig's `std.log` goes through `ESP_LOG` on the USB serial port
  instead of RTT, as in the C++ port.
- **Display offset.** As in the C++ port, u8g2 knows where the SH1106's glass
  sits, so `lcd_controller` replaces the Rust `LCD_COLUMN_OFFSET`.
- **BNO085 WAKE pin.** Unlike the C++ port, and like the Rust driver, the IMU
  driver pulses WAKE (PS0) low before each write.
- **BNO085 over I²C.** Not available; the board is wired for SPI.
- **Button.** Polled every 5 ms, as in the C++ port.
- **Wi-Fi.** Left out, as in the C++ port.
- **Warnings.** `ping.c` is compiled with `-w`: the C backend's output trips
  ESP-IDF's `-Werror` set in harmless ways, and the Zig compiler has already
  checked the code.

None of this has been run on hardware yet. Every application builds, and the
host tests pass.
