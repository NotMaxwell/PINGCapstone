# u-blox ubxlib (vendored, GNSS subset)

u-blox's own host library for its modules. The firmware uses its GNSS API to
talk to the NEO-M9N over a UART: opening the device, configuring it with
`UBX-CFG-VALSET`, receiving `UBX-NAV-PVT` and decoding it.

| | |
|---|---|
| Upstream | <https://github.com/u-blox/ubxlib> |
| Version  | `v1.5.0`, commit `0209dcf03ebe4d54a6b601f9f7a2741dbf56c785` |
| License  | Apache-2.0 (`ubxlib/LICENSE`) |
| Status   | u-blox discontinued ubxlib in November 2024 and archived the repository. It is mature and still published, but will get no further fixes. |

## What is here

`ubxlib/` holds upstream files, byte for byte, laid out as in the upstream
repository. It is a subset: just the sources `CMakeLists.txt` compiles, and
just the headers those sources and the firmware include (found from the
compiler's dependency output). Upstream's tests, examples, docs, other
platform ports and the cellular, Wi-Fi, BLE, MQTT, HTTP, socket and security
modules are left out. Some of those modules' headers remain because the
device and GNSS code include them.

## What is ours

- `CMakeLists.txt` registers the component with ESP-IDF. It lists sources
  explicitly instead of using upstream's `port/ubxlib.cmake`, which builds
  every "base" module whatever features are selected.
- `port/u_port_unsupported.c` replaces three files of upstream's ESP-IDF
  port and stubs two functions the GNSS code expects other modules to
  provide:
  - `u_port_i2c.c` uses ESP-IDF's legacy I2C driver. ESP-IDF aborts at boot
    if that is linked next to the current `i2c_master` driver, which the
    BNO085 uses.
  - `u_port_spi.c` and `u_port_ppp.c` are unused, and the latter would pull
    in lwIP.
  - `uCellAtClientHandleGet()` and `uShortRangeAtClientHandleGet()` are
    defined by the cellular and short-range modules. Upstream never builds
    GNSS without them on ESP-IDF, and a receiver on its own UART never
    needs them.

No upstream file is modified.

## Updating

Copy the new release over `ubxlib/`, keeping only what is here now. If the
build then fails to link, the linker names what is missing; add the file
that defines it to `CMakeLists.txt`. To find headers that are no longer
needed, list what the compiler used with `ninja -C build -t deps`.
