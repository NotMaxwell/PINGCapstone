# PING

Aim the device at something, press the button, and it reports that thing's GPS
coordinate.

An ESP32-S3 reads three sensors and combines them:

- a **NEO-M9N** GPS gives the device's own position,
- a **BNO085** IMU gives its orientation, and so the compass bearing and
  elevation the LiDAR points along,
- a **TF03** LiDAR gives the distance to whatever it points at.

On a press, the firmware walks the LiDAR's range out from the GPS position
along that direction, on the WGS-84 ellipsoid. It prints the target's latitude,
longitude and altitude on the USB serial port, and shows them on a 128×64 OLED.

This repository holds three versions of the firmware:

| Directory | Language and runtime | Status |
|-----------|----------------------|--------|
| `src/` (this README) | **Rust**, `no_std`, esp-hal + Embassy | The main firmware; runs on the hardware |
| [`firmware-cpp/`](firmware-cpp/README.md) | C++23 on ESP-IDF / FreeRTOS, manufacturer drivers | Port used to check whether sensor dropouts are hardware or code |
| `firmware-zig/` | Zig on ESP-IDF | Unfinished port; see its `HANDOFF.md` |

## Hardware

ESP32-S3-DevKitC-1-N8R8 (8 MB flash, 8 MB octal PSRAM).

| Device | Bus | Pins (GPIO) |
|--------|-----|-------------|
| Trigger button (to GND) | input, pull-up | 4 |
| NEO-M9N GPS | UART2, 38400 8N1 | TX 15 → RXD, RX 16 ← TXD |
| TF03 LiDAR (5–24 V UART/CAN version) | UART1, 115200 8N1 | TX 17 → blue, RX 18 ← brown |
| BNO085 IMU (Adafruit breakout) | SPI2 mode 3, 1 MHz | CS 10, MOSI 11, SCK 12, MISO 13, INT 14, RST 9, PS0/WAKE 8 |
| SH1106 OLED, 0x3C | I2C0, 400 kHz | SDA 1, SCL 2 |

The TF03 runs from the fused battery line; the rest from the board's 3V3.
[`src/constants.rs`](src/constants.rs) is the authoritative pin map, with power,
the pins kept clear on purpose, and the TF03 cable colours. Change a pin there
and every binary follows.

## Code layout

```
src/
  lib.rs            the ping_capstone library shared by every binary
  constants.rs      pin map and every fixed setting (calibration, timing, aiming)
  bno085.rs, bno085/  BNO085 driver: SHTP over SPI or I2C, SH-2 reports
  neo_m9n.rs        NEO-M9N driver: UBX configuration and NAV-PVT
  tf03.rs, tf03/    TF03 driver and its wire protocol
  ssd1306.rs        SSD1306/SH1106 text display
  geo.rs            range + bearing + elevation -> coordinate
  mag_cal.rs        magnetometer calibration and the compass heading
  bin/              one binary per program (below)
docs/
  heading-calibration.md   measuring the heading correction, and troubleshooting
  tf03-test-plan.md        bench procedure for a TF03 that shows no distance
```

### Binaries

| Binary | What it does |
|--------|--------------|
| `ping_capstone` | The PING program: button press → target coordinate |
| `gps_demo` | GPS on its own: position, accuracy, satellites |
| `gps_diag` | Finds out why the GPS has no fix: line sniffing, MON-VER, configuration, antenna, RF |
| `imu_demo` | IMU on its own: heading, elevation, roll, compass trust |
| `imu_cal` | Wipes the IMU's calibration, walks through six faces, fits the hard/soft iron correction for `MAG_CALIBRATION` |
| `heading_cal` | Measures `HEADING_CORRECTION_DEG` against known bearings |
| `compass_diag` | Finds out why the heading is wrong: field strength and dip against Earth's |
| `lidar_demo` | LiDAR on its own: distance and signal strength |
| `lcd_demo` | Display wiring check, from the wires up |

Diagnostics go out over RTT (through the debug probe connection); the main
program's target lines also go to the USB serial port.

## Toolchain

| Tool | Why |
|------|-----|
| [`espup`](https://github.com/esp-rs/espup) | Installs the `esp` Rust toolchain (Xtensa support), which `rust-toolchain.toml` selects |
| [`probe-rs`](https://probe.rs) | Flashes, prints RTT and debugs over the S3's built-in USB-JTAG; it is the cargo runner in `.cargo/config.toml` |
| Rust ≥ 1.88 (from `espup`) | Edition 2024 |

```sh
cargo install espup --locked
espup install                 # then source the export file it prints, e.g. . ~/export-esp.sh
cargo install probe-rs-tools --locked
```

On Linux, install probe-rs's udev rules (see probe.rs, "Probe setup") so the
board can be opened without root.

## Build, flash, run

Plug the board in by its **USB** port (the S3's own USB-JTAG, not the UART
bridge). The target (`xtensa-esp32s3-none-elf`) and `build-std` are set in
`.cargo/config.toml`, so plain cargo commands work:

```sh
cargo build --release                         # every binary
cargo run --release --bin ping_capstone       # flash the main program and stream RTT
cargo run --release --bin imu_demo            # or any binary from the table
```

`cargo run` hands the ELF to `probe-rs run`, which flashes it, resets the chip
and prints RTT output and, on a panic, a stack trace. Ctrl-C stops it; the
board keeps running.

To read the main program's target lines without probe-rs, open the board's USB
serial port (`/dev/ttyACM*`, `/dev/cu.usbmodem*` or a COM port) in any serial
monitor.

## Debugging

`probe-rs` is also a GDB-less debugger with a VS Code extension
(**probe-rs-debugger**) and a DAP server for other editors (`probe-rs dap-server`).
Release builds keep full debug info (`debug = 2` in `Cargo.toml`), at
`opt-level = "s"`. For stepping, a debug build is easier on the eyes:

```sh
cargo build --bin imu_demo     # dev profile, still opt-level "s"
probe-rs attach --chip esp32s3 target/xtensa-esp32s3-none-elf/debug/imu_demo
```

## Lints and checks

```sh
cargo clippy --release --all-features
cargo fmt --check
```

`.clippy.toml` caps stack frames at 1 KiB; the binaries forbid `mem::forget`
on esp-hal types.

## Calibrating

Before trusting the bearings:

1. Set `LIDAR_BORESIGHT` in `src/constants.rs` to how the LiDAR is mounted
   against the IMU.
2. Run `imu_cal` and paste its `MAG_CALIBRATION` into `src/constants.rs`.
3. Run `heading_cal` and paste its `HEADING_CORRECTION_DEG`.

[`docs/heading-calibration.md`](docs/heading-calibration.md) is the full
procedure, including how to tell when a single correction is the wrong fix.
Without a calibration the device still reports a heading, using the hard iron
it works out during handling after power up, but marks it unreliable until then.

## Known issues

- **IMU and LiDAR dropouts.** Both sensors sometimes stop communicating. The
  C++ port in [`firmware-cpp/`](firmware-cpp/README.md) uses the manufacturers'
  own driver code (CEVA's SH-2 driver and reference SPI HAL for the BNO085)
  and logs wire-level counters when a sensor goes quiet. Run it on the same
  hardware to tell a sensor or wiring fault from a driver fault. For the TF03,
  [`docs/tf03-test-plan.md`](docs/tf03-test-plan.md) works through the link
  step by step.
- **Wi-Fi** is initialised in the main program but not yet used; it is for a
  future web page that serves the coordinates.
