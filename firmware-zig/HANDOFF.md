# Zig port: handoff notes (2026-10-23)

Status of `firmware-zig/`, the Zig port of the C++ port in `../firmware-cpp/`
(itself a port of the Rust firmware in `../src`). Written so a fresh session
can pick it up. Delete this file once the port is finished.

## The brief (from the user)

- Port `firmware-cpp` to Zig, **Zig 0.17.0** (installed via mise; `../mise.toml`).
- Use the **vendors' C drivers** and keep **FreeRTOS** (ESP-IDF's).
- If an existing ESP32 HAL for Zig exists, use it; then write Zig.
- Don't touch `firmware-cpp/`; it was another session's work.

## Repo state: read this first

- Nothing is committed. `firmware-cpp/`, `firmware-zig/` and `mise.toml` are
  all **untracked**. The checkout is on `main` (685cd0f). It was on
  `cpp-port-libraries` earlier, and the index still has staged deletions of the
  old `firmware-cpp/components/ping_bno085`, `ping_hal` and `test/host`
  files from the C++ port's first attempt.
- A "Git branch cleanup" session was running at the same time. Make sure
  nothing runs `git clean` or a hard reset before both ports are committed.
  Ask the user before committing; on `main`, branch first.

## Chip and toolchain decisions

- Target: ESP32-S3 (Xtensa), ESP-IDF 6.x at `~/Dev/Work/esp-idf`
  (`dependencies.lock` in firmware-cpp shows 6.2.0).
- Upstream Zig has no Xtensa backend. Options considered:
  - kassane/zig-espressif-bootstrap (Zig 0.17.0 fork plus Espressif LLVM) with
    kassane/zig-esp-idf-sample's `imports/` wrappers as the HAL. Researched;
    it works by translate-c'ing a large hand-patched stub header and is
    marked experimental.
  - **Chosen (already in `build.zig`)**: Zig's **C backend**. Each app is
    compiled to one C file (`ofmt = .c`, target `arm-freestanding-eabi`,
    same C ABI as GCC's Xtensa ABI; the generated C asserts sizes and
    alignments). ESP-IDF's GCC compiles it. C headers are translated for
    Xtensa (`cpu esp32`). Plain mise Zig 0.17 is enough; no fork needed.
    `build.zig`'s header comment explains why.
  - So the sample's HAL is **not** used. ESP-IDF is called directly via
    translate-c of `src/c/idf.h`, with thin Zig helpers in `src/rtos.zig`.
    Tell the user about this difference from the original plan.

## Drivers: where each comes from

| Part | Source | Status |
|------|--------|--------|
| BNO085 | CEVA `sh2` v1.4.0 C, vendored unchanged in `components/sh2` | vendored; **`src/imu.zig` (sh2_Hal_t over ESP-IDF SPI + INT/RST/WAKE) not written** |
| NEO-M9N | ubxlib v1.5.0, only `u_ubx_protocol.c` and GNSS key/NAV-PVT headers, vendored in `components/ubxlib_ubx` (ubxlib is archived; the rest of it isn't used) | `src/ubx.zig` written (CFG-VALSET, NAV-PVT, MON-VER, ACK); **`src/gps.zig` (UART driver) not written** |
| SH1106 OLED | u8g2 2.37.1 via `main/idf_component.yml` | **`src/display.zig` not written** (mirror `firmware-cpp/.../display.cpp`; needs `U8X8_WITH_USER_PTR` on u8g2's lib as in the C++ CMake) |
| TF03 | none exists; hand-port | `src/tf03_protocol.zig`, `src/tf03.zig` written |
| Geometry | hand-port | `src/geo.zig` written |

The user was asked to choose the GPS driver (ubxlib vs UBX written in Zig vs
SparkFun through a C shim) and closed the question without answering. The
existing code already uses ubxlib's protocol files, so the GPS driver is in
fact settled; confirm with the user if in doubt.

## Files

Done (not yet compiled for the target):
- `build.zig`: `zig build test` (host tests for geo, tf03_protocol, and ubx
  built against ubxlib's C) and `zig build firmware -Dapp=<app>
  -Didf-cflags=<rsp file>`, which emits `zig-out/ping.c` and
  `zig-out/include/zig.h`.
- `build.zig.zon`, `CMakeLists.txt` (top-level), `sdkconfig.defaults`, `.gitignore`.
- `main/idf_config.{c,h}`: C helpers for `uart_config_t`,
  `i2c_master_bus_config_t` and `i2c_device_config_t`. They have bitfields,
  so translate-c makes them opaque.
- `src/c/{idf,sh2,ubx,u8g2}.h`: translate-c roots, becoming modules `idf`, `sh2`, `ubx`, `u8g2`.
- `src/constants.zig`, `geo.zig`, `rtos.zig` (ticks/delay/now/spawn/Mutex/
  EspError/check/Text/logFn/panic), `tf03_protocol.zig`, `tf03.zig`, `ubx.zig`.

Missing:
1. `build.zig.zon` needs `.fingerprint = 0x2cc748a2994cfd08,`. `zig build test`
   fails on this before running anything.
2. `src/ping.zig`: root of the `ping` module that `build.zig` imports. It
   should re-export constants, geo, rtos, tf03, ubx, gps, imu and display.
3. `src/imu.zig`, `src/gps.zig`, `src/display.zig` (see the table).
4. `src/apps/{main,gps_demo,imu_demo,lidar_demo,lcd_demo}.zig`: each exports
   `app_main`. Port them from `firmware-cpp/main/apps/*.cpp`.
5. `main/CMakeLists.txt`: `idf_component_register` (`idf_config.c` plus the
   generated `ping.c`, REQUIRES the drivers, `sh2`, `ubxlib_ubx`, `u8g2`).
   Write a response file of include dirs and defines with `file(GENERATE)`
   from the component's `INCLUDE_DIRECTORIES` / `COMPILE_DEFINITIONS`, then run
   `zig build firmware -Dapp=${PING_APP} -Didf-cflags=<file>` as a custom
   command producing `ping.c`. Mirror PING_APP selection from firmware-cpp.
6. `README.md` like `firmware-cpp/README.md`; then remove this file.
7. Verify: `zig build test`, then `. ~/Dev/Work/esp-idf/export.sh &&
   idf.py -B build/main build` for every app.

## References

- C++ source of truth: `firmware-cpp/components/ping/{include/ping,src}`,
  `firmware-cpp/main/apps`, `firmware-cpp/test/host`.
- Memory notes: `cpp-port-uses-libraries`, `tf03-driver-in-repo`,
  `bno085-driver-in-repo`, `keep-main-rs-clear`.
- Reference clones (temporary, may be gone): `/tmp/zigport-ref.FE2q/{sample,ubxlib,sh2}`.
