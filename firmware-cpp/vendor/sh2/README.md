# CEVA SH-2 driver (vendored)

The BNO085's own host driver, from its maker. It implements the SHTP
transport and the SH-2 command set: resets, product IDs, sensor
configuration, tare, calibration, and decoding of every sensor report.

| | |
|---|---|
| Upstream | <https://github.com/ceva-dsp/sh2> |
| Version  | `v1.4.0`, commit `b514b1e2586ddc195e553dac89fc94c637b25298` |
| License  | Apache-2.0 (see the file headers and `ceva-sh2/NOTICE.txt`) |
| Local changes | None. `ceva-sh2/` is a straight copy of the repository's files. |

`CMakeLists.txt` is ours and only registers the sources with ESP-IDF. The
platform half of the driver, the `sh2_Hal_t` that moves bytes over I²C, is
in `components/bno085`.

## Updating

Copy the new release's `*.c`, `*.h`, `README.md` and `NOTICE.txt` over
`ceva-sh2/`, update the version above, and add any new `.c` files to
`CMakeLists.txt`.
