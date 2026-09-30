# PING firmware, C++

A C++23 port of the Rust firmware in `../src`, for the ESP32-S3 on ESP-IDF
and FreeRTOS, built by CMake with Espressif's LLVM/Clang.

Aim the device at something, press the button, and it prints that thing's
GPS coordinate on the USB serial port:

```
target #1: 41.2572453, -95.9355353, alt 359.3 m | range 123.45 m, bearing 313.7 deg, elevation 13.7 deg | from 41.2565000, -95.9345000, alt 330.0 m, +/-1.8 m, 14 satellites | heading +/-2.1 deg, compass calibration high
target #2: none, GPS has no usable fix (no fix, 3 satellites)
```

Each sensor runs in its own FreeRTOS task and keeps its latest reading. A
press takes the three readings and walks the LiDAR's range out from the GPS
position along the direction the IMU says the LiDAR points. The Wi-Fi web
server the Rust firmware was preparing for is not ported yet.

## Sensor drivers

Each sensor uses its maker's own host library where one exists. The libraries
are copied into `vendor/` rather than fetched by a package manager, and each
has a `README.md` saying where it came from and what, if anything, was
changed.

| Sensor | Driver | Where |
|---|---|---|
| BNO085 IMU (I²C) | CEVA's **SH-2** library v1.4.0, the reference driver for the BNO08x family. We add the ESP-IDF HAL it asks for: an `sh2_Hal_t` over the `i2c_master` driver plus the INT pin. | `vendor/sh2`, `components/bno085` |
| NEO-M9N GPS (UART) | u-blox's **ubxlib** v1.5.0, GNSS part only. It handles the UART, UBX framing, `CFG-VALSET` and NAV-PVT decoding. u-blox has discontinued ubxlib (Nov 2024), but it is mature. | `vendor/ubxlib`, `components/neo_m9n` |
| TF03 LiDAR (UART) | Benewake publishes no driver, only the protocol, so this one is ours: the Rust driver's logic on ESP-IDF's UART driver. Its parser is checked at compile time. | `components/tf03` |

Everything underneath is ESP-IDF's own drivers (`i2c_master`, `uart`,
`gpio`, `esp_timer`); there is no hand-rolled HAL.

## Layout

```
CMakeLists.txt        ESP-IDF project; defaults to ESP32-S3 + Clang
CMakePresets.json     cmake --preset esp32s3 (Clang) / esp32s3-gcc
sdkconfig.defaults    240 MHz, console on USB, 1 kHz tick, -Os
cmake/ping_cxx.cmake  -std=c++23 and warnings for our components
main/                 the application
  config.hpp            pins, rates, boresight, declination, timeouts
  sensors.cpp           one task per sensor, with restart on silence
  sighting.cpp          the button press: readings -> coordinate
  main.cpp              wiring and the button loop
components/
  bno085/             SH-2 HAL + C++ wrapper
  neo_m9n/            ubxlib wrapper
  tf03/               TF03 driver and protocol
  geo/                range + orientation -> coordinate (no ESP-IDF)
vendor/
  sh2/                CEVA SH-2, unmodified
  ubxlib/             u-blox ubxlib, GNSS subset, unmodified + stubs
test/host/            host build of geo and the TF03 protocol checks
```

## Setting up

You need ESP-IDF v6.1 with the ESP32-S3 GCC toolchain (for its C and C++
libraries) and Espressif's Clang:

```sh
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
cd ~/esp/esp-idf
./install.sh esp32s3
python tools/idf_tools.py install esp-clang
. ./export.sh      # every new shell: sets IDF_PATH and puts the tools on PATH
```

Upstream Clang can't be used here: Xtensa support is only in Espressif's
fork. ESP-IDF picks the fork when `IDF_TOOLCHAIN=clang`, which this project
sets by default.

## Building

```sh
cd firmware-cpp
cmake --preset esp32s3            # configure: Ninja, ESP32-S3, Clang
cmake --build --preset esp32s3    # build/ping.bin
```

Plain `cmake -B build -G Ninja` works too, since `CMakeLists.txt` defaults
the target and toolchain itself, and so does `idf.py build`. The `esp32s3-gcc`
preset builds the same code with GCC, into `build-gcc/`, for comparison; both
build with no warnings. (Today GCC's image is smaller, 282 KB against Clang's
355 KB.) `build/compile_commands.json` is exported for clangd.

Settings go in `sdkconfig.defaults`. Each preset generates its full
`sdkconfig` inside its own build directory, so the two toolchains never share
one; `cmake --build --preset esp32s3 --target menuconfig` edits it
interactively.

## Flashing and watching

The board's USB port (the one labelled USB, not UART, on most dev boards) is
both the flashing port and the console:

```sh
export ESPPORT=/dev/ttyACM0        # macOS: /dev/cu.usbmodem*, Windows: COMx
cmake --build --preset esp32s3 --target flash
cmake --build --preset esp32s3 --target monitor    # Ctrl-] to quit
```

or `idf.py -p $ESPPORT flash monitor`. Result lines and the diagnostics share
the console; result lines start with `target #`.

## Wiring

Unchanged from the Rust firmware; all of it lives in `main/config.hpp`.

| Sensor | ESP32-S3 | Sensor pin |
|---|---|---|
| NEO-M9N (UART2, 38400) | GPIO15 (TX) | RX1 |
| | GPIO16 (RX) | TX1 |
| BNO085 (I2C0, 400 kHz, address 0x4A) | GPIO8 | SDA |
| | GPIO9 | SCL |
| | GPIO10 | INT (pulled up) |
| TF03 (UART1, 115200) | GPIO17 (TX) | RxD, blue |
| | GPIO18 (RX) | TxD, brown |
| Button | GPIO0 | BOOT button, or a switch to ground |

Before trusting the bearings, set `kLidarBoresight` to how the LiDAR is
mounted against the IMU and `kMagneticDeclinationDeg` to where the device is
used.

## Host tests

The geometry and the TF03 parser have no hardware in them, so they also
build on a desktop with any C++23 compiler:

```sh
cmake -S test/host -B build-host -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build build-host && ctest --test-dir build-host --output-on-failure
```

`geo_test` checks the aim from known orientations, and checks that the
projection stays within 6 mm of the exact straight line up to 60° latitude
and within 3 cm at 85°. The TF03 checks are `static_assert`s in
`components/tf03/src/protocol_checks.cpp`, so they also run in every
firmware build.

## Notes on the C++

- **C++23, strictly.** ESP-IDF 6.1 compiles C++ as `-std=gnu++26` by
  default. Our components override that with `-std=c++23`, the newest
  standard the toolchain supports in full; ESP-IDF's own code keeps its
  default.
- **Errors are values.** Exceptions and RTTI stay off, as ESP-IDF ships them.
  Every driver call returns `std::expected`, and the tasks compose them with
  `and_then`/`transform`, much as the Rust used `Result` and `?`.
- **Blocking tasks instead of async.** Where the Rust awaited, each sensor
  task blocks: on a UART read, on ubxlib, or on the IMU's INT line through a
  GPIO interrupt and a semaphore. FreeRTOS schedules the rest meanwhile.
- **No `std::format` or `std::print`.** They pull in ~270 KB of libstdc++,
  which took the image from 355 KB to 743 KB of the 1 MB app partition and
  would leave no room for the web server. Output uses ESP-IDF's `ESP_LOGx`
  and a `snprintf` wrapper marked `[[gnu::format]]`, so format strings are
  still checked at compile time.

## Not yet ported

- The Wi-Fi web server, which the Rust firmware only initialised.
- The single-sensor demo binaries (`gps_demo`, `imu_demo`, `lidar_demo`).
- None of this has run on hardware yet. It builds cleanly, but the I²C
  timing against a real BNO085 in particular needs checking on the bench.
