# Vendored code

Third-party code, copied in rather than fetched by a package manager. Each
directory is an ESP-IDF component; `../CMakeLists.txt` adds them all through
`EXTRA_COMPONENT_DIRS`.

**No upstream file is modified.** The only files here that are ours are the
`CMakeLists.txt` of the four libraries that ship none for ESP-IDF, marked
below. Two build tweaks that would otherwise need upstream edits live in
`../CMakeLists.txt` instead:

- linking `esp_netif` to the core, which upstream gets from the `mdns`
  registry component;
- switching off the few warnings Clang (and, in one file, GCC) raises in this
  code.

| Directory | Upstream | Version | Commit | License | Ours |
|---|---|---|---|---|---|
| `arduino-esp32` | [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) | 3.3.12 | `94afccf` | LGPL-2.1 | — |
| `Adafruit_BNO08x` | [adafruit/Adafruit_BNO08x](https://github.com/adafruit/Adafruit_BNO08x) | 1.2.7 | `525d050` | BSD-3-Clause (bundled SH-2 driver: Apache-2.0) | `CMakeLists.txt` |
| `Adafruit_BusIO` | [adafruit/Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO) | 1.17.4 | `3b83642` | MIT | — |
| `Adafruit_Sensor` | [adafruit/Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor) | 1.1.15 | `0a9127a` | Apache-2.0 | `CMakeLists.txt` |
| `SparkFun_u-blox_GNSS_v3` | [sparkfun/SparkFun_u-blox_GNSS_v3](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3) | 3.1.15 | `8b3ba1e` | MIT | `CMakeLists.txt` |
| `TFMini-Plus` | [budryerson/TFMini-Plus](https://github.com/budryerson/TFMini-Plus) | 1.5.0 | `90f1fcd` | **none stated** — see below | `CMakeLists.txt` |

## What each is for

- **arduino-esp32** is the Arduino core for ESP32, used as an ESP-IDF
  component. It is the HAL the three sensor libraries are written against:
  `Wire`, `HardwareSerial`, `pinMode`/`digitalRead`/`attachInterrupt`,
  `millis`/`delay`. Only part of it is here: `CMakeLists.txt`,
  `Kconfig.projbuild`, `cores/esp32`, `variants/esp32s3`, and the `Wire` and
  `SPI` libraries. `sdkconfig.defaults` turns on the core's selective
  compilation so that only those two libraries are built. Version 3.3 is
  the current stable line and is built on ESP-IDF v5.5, which is why the
  project uses that release.
- **Adafruit_BNO08x** drives the BNO085. It bundles the SH-2 host driver
  from Hillcrest Laboratories (now part of CEVA), the BNO08x's maker, and
  implements its HAL on `Wire`. It needs **Adafruit_BusIO** (I2C/SPI device
  wrappers) and **Adafruit_Sensor** (the Unified Sensor base class).
- **SparkFun_u-blox_GNSS_v3** drives the NEO-M9N on SparkFun's own GPS-15733
  board: `CFG-VALSET` configuration and `UBX-NAV-PVT` parsing over
  `HardwareSerial`.
- **TFMini-Plus** drives the TF03. Benewake publishes no driver of its own.
  The TF03 shares the TFMini-Plus's UART protocol: the same 9-byte `59 59`
  data frame, and the same command IDs for version, reset, frame rate,
  trigger, baud rate, output and save.

## TFMini-Plus has no license

Neither the repository nor its source files state a license, which by
default means all rights reserved. Before this code goes anywhere public,
ask the author (Bud Ryerson) for permission or a license, or replace it. The
LiDAR task only uses `begin()`, `sendCommand(GET_FIRMWARE_VERSION)`,
`getData()` and the `version` and `status` fields, so a replacement is
small. The public-domain
[opensensinglab/tfmini](https://github.com/opensensinglab/tfmini) is not a
drop-in: it sends the original TFMini's configuration commands on start-up.

## Updating

Copy the new release's files over the same paths, and nothing else. If the
build then fails, fix it from `../CMakeLists.txt` or a component
`CMakeLists.txt` of ours rather than by editing upstream's files.
