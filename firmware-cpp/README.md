# P.I.N.G. sensor node — C++ / ROS 2 firmware

A C++ port of the Rust/embassy firmware in the repository root, for the
ESP32-S3-DevKitC-1-N8R8. It reads the three sensors on the Team 13 order form
and publishes them to ROS 2 (Jazzy) through micro-ROS:

| Sensor | Bus | Driver | ROS 2 topic | Message |
|---|---|---|---|---|
| Adafruit BNO085 (4754) | I²C @ 400 kHz | Adafruit_BNO08x → CEVA SH-2 | `/imu/data` | `sensor_msgs/Imu` |
| Benewake TF03-100 | UART1 @ 115200 | TFMPlus | `/lidar/range` | `sensor_msgs/Range` |
| SparkFun NEO-M9N | UART2 @ 38400 | SparkFun u-blox GNSS v3 | `/gps/fix` | `sensor_msgs/NavSatFix` |

Nothing below the drivers is hand-written. The Arduino core, running as an
ESP-IDF component, supplies I²C, UART and GPIO, and each sensor is driven by
its vendor's library (or the de-facto standard one where the vendor ships
none).

## Layout

```
firmware-cpp/
├── CMakeLists.txt            ESP-IDF project
├── sdkconfig.defaults        board, PSRAM, Arduino and micro-ROS settings
├── partitions.csv            4 MB app partition (micro-ROS + Arduino need > 1 MB)
├── main/
│   ├── idf_component.yml     arduino-esp32 + micro-ROS, from the ESP Component Registry
│   ├── config.hpp            pin map, rates, topic and frame names
│   ├── imu.cpp lidar.cpp gnss.cpp   one FreeRTOS task per sensor
│   ├── ros_node.cpp          micro-ROS node, publishers, agent reconnect, time sync
│   └── main.cpp              app_main: start Arduino, start tasks
├── components/arduino_libs/  builds the libraries in external/ as one IDF component
└── external/                 vendor libraries, pinned as git submodules
```

## Dependencies

Dependency management is the **ESP-IDF Component Manager**, not Conan or
vcpkg. Neither Conan nor vcpkg supports the Xtensa ESP32-S3 target, and the
component manager already resolves, downloads and lock-files (`dependencies.lock`)
everything ESP-IDF builds. The Arduino libraries aren't in the registry (only
Adafruit_BusIO ships an IDF `CMakeLists.txt`), so they are git submodules pinned to
release commits.

| Library | Version | Source | License |
|---|---|---|---|
| arduino-esp32 | ~3.3.12 | ESP Component Registry `espressif/arduino-esp32` | LGPL-2.1 |
| micro_ros_espidf_component | ^24 (Jazzy) | ESP Component Registry `micro-ros/micro_ros_espidf_component` | Apache-2.0 |
| Adafruit_BNO08x | 1.2.7 | submodule | BSD-3-Clause; bundled CEVA SH-2 is Apache-2.0 + CEVA NOTICE |
| Adafruit_BusIO | 1.17.4 | submodule | MIT |
| Adafruit_Sensor | 1.1.15 | submodule | Apache-2.0 |
| SparkFun_u-blox_GNSS_v3 | 3.1.15 | submodule | MIT |
| TFMini-Plus (TFMPlus) | 1.5.0 | submodule | **no license file** — see below |

**About the TF03 driver.** Benewake publishes manuals and example sketches but
no library. TFMPlus is the widely used Benewake frame parser, and the TF03
sends the same 9-byte frame. `lidar.cpp` covers the two ways the TF03
differs: bytes 6–7 are reserved rather than temperature, and "out of range"
is the 18000 cm sentinel rather than `0xFFFF`. The repo has no license file,
so ask your instructor whether it's fine to use. If it isn't, the frame is
simple enough to parse in about 40 lines.

## Building

Requires ESP-IDF **v5.5** (arduino-esp32 3.3 needs ≥ 5.3; micro-ROS is tested on 5.2–6.0).
The code is compiled as **C++26** (`-std=gnu++26`, GCC 14.2 in IDF 5.5).

```sh
git submodule update --init --recursive
cd firmware-cpp

# micro-ROS builds its libraries with colcon, inside the IDF Python env:
. $IDF_PATH/export.sh
pip install catkin_pkg colcon-common-extensions lark empy==3.3.4

idf.py set-target esp32s3
idf.py menuconfig   # micro-ROS Settings → Wi-Fi SSID/password, agent IP and port
idf.py build flash monitor
```

Or with no local IDF install:

```sh
docker run --rm -it -v "$PWD/..":/project -w /project/firmware-cpp \
  --device=/dev/ttyUSB0 espressif/idf:release-v5.5 bash -lc \
  'pip install catkin_pkg colcon-common-extensions lark empy==3.3.4 && idf.py set-target esp32s3 build'
```

After changing `colcon.meta` or the micro-ROS settings, run `idf.py clean-microros`.

## Running with ROS 2

Start the agent on the machine whose IP you configured:

```sh
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

Then, from any ROS 2 Jazzy shell on the same network:

```sh
ros2 topic list
ros2 topic echo /imu/data
ros2 topic hz /lidar/range
```

All three publishers use best-effort QoS, so subscribe with
`--qos-reliability best_effort` if a tool defaults to reliable.
Timestamps are synchronised to the agent's clock at connect time and every 60 s.
If the agent goes away, the node tears its session down and reconnects on
its own.

## Wiring (ESP32-S3-DevKitC-1-N8R8)

| Signal | GPIO | Notes |
|---|---|---|
| BNO085 SDA / SCL | 8 / 9 | Same as the Rust `imu_demo`; STEMMA QT board has pull-ups |
| BNO085 RST | 11 | Active low; lets the firmware recover a wedged hub |
| TF03 TX → ESP RX | 18 | UART1. TF03 UART is 3.3 V LVTTL, so no level shifter is needed |
| TF03 RX ← ESP TX | 17 | Power the TF03 from the 12 V rail (see its manual for range) |
| NEO-M9N TX → ESP RX | 16 | UART2, 38400 baud by default |
| NEO-M9N RX ← ESP TX | 15 | |

Pins avoided: 26–32 (flash), 33–37 (octal PSRAM on the N8R8), 0/3/45/46
(strapping), 19/20 (native USB), 43/44 (console UART).

## Known risks to check on the bench

- **BNO085 over I²C on ESP32.** Adafruit have a long-standing issue with the
  BNO08x on the *original* ESP32's I²C controller. The S3's controller is
  different and is generally reported to work, but test it first. If it's
  flaky, `Adafruit_BNO08x::begin_SPI` is the fallback.
- **TF03 firmware without signal strength.** Early TF03 firmware reports
  strength as 0, and the `strength >= 40` validity check would then reject
  every frame. The firmware version is logged at boot.
- **Orientation covariance.** The BNO085 reports a single heading-accuracy
  value, so it is used for all three axes. That overstates roll/pitch
  uncertainty but is never over-confident.
