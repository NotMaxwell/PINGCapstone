# Building, flashing and debugging firmware-cpp

Everything goes through CMake. ESP-IDF's `idf.py` is a front end to CMake and
reads the presets in `CMakePresets.json`, so each application and toolchain
is one preset name:

| Preset       | What                                                      | Build dir          |
|--------------|-----------------------------------------------------------|--------------------|
| `default`    | main program, esp-clang                                   | `build/main`       |
| `gps_demo`   | GPS demo, esp-clang                                       | `build/gps_demo`   |
| `imu_demo`   | IMU demo, esp-clang                                       | `build/imu_demo`   |
| `lidar_demo` | LiDAR demo, esp-clang                                     | `build/lidar_demo` |
| `lcd_demo`   | display demo, esp-clang                                   | `build/lcd_demo`   |
| `main-debug` | main program, `-Og`, GDB stub on panic, no watchdog panic | `build/main-debug` |
| `main-gcc`   | main program, Espressif GCC (fallback toolchain)          | `build/main-gcc`   |
| `host`       | unit tests on your computer, system clang                 | `build/host`       |
| `host-asan`  | the same with AddressSanitizer and UBSan                  | `build/host-asan`  |

Each preset keeps its own `sdkconfig` inside its build directory, so switching
between them never rebuilds the others.

## Tools

| Tool | Version | Why |
|------|---------|-----|
| ESP-IDF | **v6.1.1** (pinned) | Espressif's SDK: FreeRTOS, drivers, build system, flashing |
| esp-clang | 21.1.3 (installed by ESP-IDF) | Firmware compiler (`IDF_TOOLCHAIN=clang`) |
| xtensa-esp-elf GCC | 15.2 (installed by ESP-IDF) | Required by ESP-IDF even with clang (libraries, linker); also the `main-gcc` preset |
| esp-clangd | 21.1.3 | Language server that understands the Xtensa target |
| OpenOCD (esp32) | v0.12.0-esp32 | GDB server over the S3's built-in USB-JTAG |
| xtensa-esp32s3-elf-gdb | 17.1 | Debugger; speaks DAP natively |
| CMake ≥ 3.25, Ninja | | Build |
| clang + libc++ (host tests) | clang 18+; libc++ only needed before clang 19 | Host unit tests |

### Quick setup

The setup scripts install everything below for your OS, and are safe to run again:

```sh
./setup.sh                    # Linux (apt, dnf, pacman, zypper) or macOS (Homebrew)
./setup.sh --no-system        # without sudo: ESP-IDF only, system packages are up to you
```

```powershell
powershell -ExecutionPolicy Bypass -File setup.ps1    # Windows, no administrator rights needed
```

On Linux the script also installs OpenOCD's udev rule and adds you to the
serial-port groups (log out and in afterwards). On Windows it installs Git and
Python with `winget --scope user` if they are missing, and llvm-mingw for the
host tests. If your organisation blocks programs in your user folder, it says
so at the end; see "On a locked-down PC" below.

### Installing by hand

Linux and macOS (Windows: `install.ps1` instead of `install.sh`):

```sh
mkdir -p ~/esp && cd ~/esp
git clone --depth 1 --branch v6.1.1 --recursive --shallow-submodules \
    https://github.com/espressif/esp-idf.git esp-idf-v6.1.1
cd esp-idf-v6.1.1
./install.sh esp32s3
python3 tools/idf_tools.py install esp-clang openocd-esp32
```

Every shell that builds, flashes or debugs needs ESP-IDF's environment first:

```sh
. ~/esp/esp-idf-v6.1.1/export.sh
```

That puts `idf.py`, esp-clang, esp-clangd (as `clangd`), OpenOCD and the
Xtensa GDB on `PATH`. Start your editor from that shell.

On Linux, give yourself the USB port once: `sudo usermod -aG dialout,plugdev $USER`
and install Espressif's udev rule for OpenOCD
(`sudo cp $OPENOCD_SCRIPTS/../contrib/60-openocd.rules /etc/udev/rules.d/`), then log out and in.

For the host tests on Ubuntu/Debian with clang 18: `sudo apt install clang ninja-build cmake libc++-dev libc++abi-dev`.

## Build

```sh
cd firmware-cpp
idf.py build                              # "default" preset: main program, esp-clang
idf.py --preset imu_demo build            # a demo
idf.py --preset main-gcc build            # same program with GCC
```

The first build fetches u8g2 (the only library not in this repository) into
`managed_components/` and writes `dependencies.lock`; commit that file.

## Flash and monitor

Plug the board in by the USB port marked **USB** (the S3's own USB-Serial-JTAG,
not the UART bridge). Logs and the `target #n: ...` lines come out of the same port.

```sh
idf.py flash monitor                      # main program
idf.py --preset lidar_demo flash monitor
idf.py -p /dev/ttyACM0 flash monitor      # pick the port if there are several
```

Leave the monitor with `Ctrl-]`. If flashing cannot connect, hold **BOOT**, tap
**RESET**, release **BOOT**, and flash again.

Other useful targets: `idf.py --preset <p> menuconfig`, `size`, `size-components`,
`erase-flash`, `fullclean`.

## Host tests

No board and no ESP-IDF needed:

```sh
cmake --preset host && cmake --build --preset host && ctest --preset host
cmake --preset host-asan && cmake --build --preset host-asan && ctest --preset host-asan
./build/host/test/host/ping_host_tests --gtest_filter='Ubx*'   # one suite
```

They cover everything that does not touch hardware: the TF03 and UBX wire
protocols, the BNO085 SPI transport (against a fake bus), the geometry and the
compass maths.

## Debug on the board

The ESP32-S3 debugs over the same USB cable through its built-in JTAG: no probe.

```sh
idf.py --preset main-debug flash          # once: optimisation that keeps variables
idf.py --preset main-debug openocd        # terminal 1: GDB server on :3333
idf.py --preset main-debug gdb            # terminal 2: GDB, halted at app_main
```

Or by hand: `openocd -f board/esp32s3-builtin.cfg` and
`xtensa-esp32s3-elf-gdb -ex 'target extended-remote :3333' -ex 'mon reset halt'
-ex 'thbreak app_main' -ex c build/main-debug/ping_capstone.elf`.

Useful in GDB: `info threads` (one per FreeRTOS task), `thread apply all bt`,
`mon reset halt`. The ESP32-S3 has two hardware breakpoints per core plus
flash breakpoints that OpenOCD manages; a few at a time works best.

A crash on a release build prints a backtrace in `idf.py monitor`, which
decodes the addresses against the ELF. The `main-debug` preset instead stops in
the GDB stub on a panic, where `idf.py monitor` drops straight into GDB.

## Editors

Both editors use clangd for code intelligence. It reads `.clangd`, which points
firmware sources at `build/main/compile_commands.json` and the host tests at
`build/host/compile_commands.json`, so **build `default` and `host` once**
before opening the editor. Use Espressif's clangd (on `PATH` after `export.sh`):
upstream clangd does not know the Xtensa target.

### VS Code

Open `firmware-cpp/` from a shell that sourced `export.sh` (`code firmware-cpp`).
Install the recommended extensions (clangd, C/C++ for its debugger, CMake Tools).

- **Build:** `Ctrl+Shift+B` → pick a preset. Other tasks (`Terminal → Run Task`):
  flash + monitor, monitor, flash debug build, menuconfig, openocd, host tests.
- **Debug:** run "firmware: flash debug build", then `F5` with
  "ESP32-S3: debug main (OpenOCD)". It starts OpenOCD, resets the chip and
  stops at `app_main`. "Host tests (gdb)" debugs the unit tests.

Espressif's own "ESP-IDF" extension also works with this project if you prefer
its UI; it is not required.

### Neovim (0.11+)

`firmware-cpp/.nvim.lua` is a project config. Turn on `vim.o.exrc = true` in
your `init.lua`, start `nvim` inside `firmware-cpp/` from an exported shell, and
allow the file when asked. It gives you:

- clangd on C/C++ files (built-in `vim.lsp`, no plugin needed),
- `:PingBuild [preset]`, `:PingFlash [preset]`, `:PingMonitor [preset]`,
  `:PingTest [host|host-asan]`, `:PingOpenOCD`,
- with [nvim-dap](https://github.com/mfussenegger/nvim-dap): the configurations
  from `.vscode/launch.json` through the cpptools adapter
  (`:MasonInstall cpptools`), and without cpptools a native GDB-DAP
  configuration, "ESP32-S3: attach via GDB-DAP". For that one run
  `:PingOpenOCD`, start the configuration, then type `monitor reset halt` in
  the DAP REPL.

## On a locked-down PC (no programs may be installed or run)

Everything except editing can be done from a web browser.

1. **Build** happens on GitHub: `.github/workflows/firmware-cpp.yml` builds every
   preset and runs the host tests on each push that touches `firmware-cpp/`.
   On GitHub, open **Actions → firmware-cpp →** the latest run, and download the
   artifact for the application you want (`main`, `imu_demo`, ...). It holds
   `bootloader.bin`, `partition-table.bin`, `ping_capstone.bin`, the `.elf` for
   decoding crashes, and `FLASH-OFFSETS.txt`. To build without pushing a change,
   use **Run workflow** on that page.
2. **Flash** from Chrome or Edge (they speak USB serial natively) at
   <https://espressif.github.io/esptool-js/>: plug the board in by its **USB**
   port, **Connect**, pick the port, then add the three files at these offsets
   and **Program**:

   | File | Offset |
   |------|--------|
   | `bootloader.bin` | `0x0` |
   | `partition-table.bin` | `0x8000` |
   | `ping_capstone.bin` | `0x10000` |

   If it cannot connect, hold **BOOT**, tap **RESET**, release **BOOT**, and try again.
3. **Monitor**: the same page's **Console** tab (115200 baud is fine; the USB
   port ignores the rate) shows the logs and the `target #n: ...` lines. Press
   **RESET** on the board after flashing.
4. **Crashes**: copy the `Backtrace:` line from the console; it is decoded
   against the `.elf` from the same artifact with
   `xtensa-esp32s3-elf-addr2line -pfiaC -e ping_capstone.elf <addresses>` on any
   machine that has the toolchain (or ask Claude, which has one).

JTAG debugging needs OpenOCD and GDB, which are programs, so it is not
available this way.

## When something goes wrong

| Symptom | Likely cause |
|---------|--------------|
| `IDF_PATH is not set` | `export.sh` not sourced in this shell |
| clangd: `unknown target triple 'xtensa'` | upstream clangd running; start the editor from an exported shell |
| host configure: `No C++ standard library with std::expected` | clang < 19 without libc++; install `libc++-dev libc++abi-dev` |
| `idf.py` picks the wrong build | pass `--preset`; `idf.py` without one uses `default` |
| OpenOCD `LIBUSB_ERROR_ACCESS` | udev rule / `plugdev` group missing (Linux) |
| board resets every 10 s with `task_wdt` | a sensor task stopped feeding the watchdog: a driver hung; the log above the panic names the task |
