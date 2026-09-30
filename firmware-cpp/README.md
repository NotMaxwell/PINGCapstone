# PING firmware, C++

A C++23 port of the Rust firmware in `../src` for the ESP32-S3. It runs on
ESP-IDF and FreeRTOS, uses the Arduino core for ESP32 as its hardware layer,
and is built by CMake with Espressif's LLVM/Clang.

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

## The hardware layer, and the drivers on it

This firmware writes none of its own hardware access. The hardware layer is
the **Arduino core for ESP32** (Espressif's `arduino-esp32`), used as an
ESP-IDF component. Its `Wire`, `HardwareSerial`, `pinMode`, `digitalRead`
and `attachInterrupt` sit on ESP-IDF's drivers and FreeRTOS. Each sensor is
driven by a library that already targets that API:

| Sensor | Library | Talks over |
|---|---|---|
| BNO085 IMU | **Adafruit BNO08x**, which bundles the SH-2 driver from the chip's maker and implements its HAL | `Wire` (I²C) |
| NEO-M9N GPS | **SparkFun u-blox GNSS v3**, from the maker of the GPS-15733 board used here | `Serial2` |
| TF03 LiDAR | **TFMini-Plus** (Bud Ryerson). Benewake ships no driver, and the TF03 speaks the TFMini-Plus's protocol. | `Serial1` |

All of this lives in `vendor/`, copied in rather than fetched by a package
manager, with no upstream file modified. `vendor/README.md` lists versions,
commits and licenses. **TFMini-Plus has no license**; read the note there
before publishing.

## Layout

```
CMakeLists.txt        ESP-IDF project; defaults to ESP32-S3 + Clang
CMakePresets.json     cmake --preset esp32s3 (Clang) / esp32s3-gcc
sdkconfig.defaults    240 MHz, console on USB, 1 kHz tick, Arduino libraries
cmake/ping_cxx.cmake  -std=c++23 and warnings for our code
main/                 the application
  config.hpp            pins, rates, boresight, declination, timeouts
  sensors.cpp           one task per sensor, on the vendored libraries
  sighting.cpp          the button press: readings -> coordinate
  main.cpp              Arduino start-up, the tasks, the button loop
components/geo/       range + orientation -> coordinate (plain C++)
vendor/               Arduino core and sensor libraries, unmodified
test/host/            host build of the geometry
```

## Setting up

The Arduino core's current stable release, 3.3, is built on **ESP-IDF
v5.5**, so that is the release this project needs. Install it with the
ESP32-S3 GCC toolchain (for its C and C++ libraries) and Espressif's Clang:

```sh
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v5.5
cd ~/esp/esp-idf-v5.5
./install.sh esp32s3
python tools/idf_tools.py install esp-clang
. ./export.sh      # every new shell: sets IDF_PATH and puts the tools on PATH
```

Upstream Clang can't be used here: Xtensa support is only in Espressif's
fork. ESP-IDF picks the fork when `IDF_TOOLCHAIN=clang`, which this project
sets by default. ESP-IDF 5.5 still calls its Clang support experimental and
says so when you configure; the `esp32s3-gcc` preset is there for when that
matters.

## Building

```sh
cd firmware-cpp
cmake --preset esp32s3            # configure: Ninja, ESP32-S3, Clang
cmake --build --preset esp32s3    # build/ping.bin
```

Plain `cmake -B build -G Ninja` works too, since `CMakeLists.txt` defaults
the target and toolchain itself, and so does `idf.py build`. The
`esp32s3-gcc` preset builds the same code with GCC, into `build-gcc/`. Both
build with no warnings. The images come out at 407 KB with Clang and 353 KB
with GCC, of the 1 MB app partition. `build/compile_commands.json` is
exported for clangd.

Settings go in `sdkconfig.defaults`. Each preset generates its full
`sdkconfig` inside its own build directory, so the two toolchains never share
one; `cmake --build --preset esp32s3 --target menuconfig` edits it
interactively. The Arduino core only builds the libraries switched on there
(`CONFIG_ARDUINO_SELECTIVE_*`): `Wire` and `SPI` today. The web server will
need `Network`, `WiFi` and `WebServer` back.

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
| NEO-M9N (`Serial2`, 38400) | GPIO15 (TX) | RX1 |
| | GPIO16 (RX) | TX1 |
| BNO085 (`Wire`, 400 kHz, address 0x4A) | GPIO8 | SDA |
| | GPIO9 | SCL |
| | GPIO10 | INT (pulled up) |
| TF03 (`Serial1`, 115200) | GPIO17 (TX) | RxD, blue |
| | GPIO18 (RX) | TxD, brown |
| Button | GPIO0 | BOOT button, or a switch to ground |

Before trusting the bearings, set `kLidarBoresight` to how the LiDAR is
mounted against the IMU and `kMagneticDeclinationDeg` to where the device is
used.

## Host tests

The geometry has no hardware in it, so it also builds on a desktop with any
C++23 compiler:

```sh
cmake -S test/host -B build-host -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build build-host && ctest --test-dir build-host --output-on-failure
```

`geo_test` checks the aim from known orientations, and checks that the
projection stays within 6 mm of the exact straight line up to 60° latitude
and within 3 cm at 85°.

## Notes

- **C++23, strictly.** ESP-IDF 5.5 compiles C++ as `gnu++2b`, C++23 with GNU
  extensions. Our code builds as ISO `-std=c++23`, and the vendored code
  keeps ESP-IDF's default.
- **FreeRTOS tasks, not `setup()`/`loop()`.** Arduino's autostart is off.
  `app_main` calls `initArduino()` and starts one task per sensor, as the
  Rust started one Embassy task per sensor.
- **What the tasks still decide for themselves.** They wait for the IMU's INT
  line before asking the library for reports. They only call the TF03
  library once a whole frame has arrived, because its `getData()`
  busy-waits. And they apply the TF03's own out-of-range (18000 cm) and weak
  return (strength below 40) rules, which the TFMini-Plus library doesn't
  know about.
- **Recovery, as in the Rust.** A GPS silent for 3 s is configured again, in
  case a power glitch put it back to factory NMEA output. An IMU silent for
  1 s, or one that reset itself, is started again.
- **No `std::format`.** It pulls in about 270 KB of libstdc++, too much next
  to a future web server in a 1 MB partition. Output uses `ESP_LOGx` and an
  `snprintf` wrapper marked `[[gnu::format]]`, so format strings are still
  checked at compile time.
- **Build tweaks for the vendored code** live in `CMakeLists.txt`, not in
  upstream's files. They link `esp_netif` to the core, which upstream gets
  through a registry package, and switch off the handful of warnings Clang
  raises in the libraries.

## Not yet done

- The Wi-Fi web server, which the Rust firmware only initialised.
- The single-sensor demo binaries (`gps_demo`, `imu_demo`, `lidar_demo`).
- None of this has run on hardware yet. It builds cleanly with both
  toolchains, but it needs a first run on the bench.
