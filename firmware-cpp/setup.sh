#!/usr/bin/env bash
# Installs everything firmware-cpp needs on Linux or macOS: ESP-IDF (pinned),
# its esp-clang, GCC, OpenOCD and GDB, and the host-test tools (clang, libc++
# where needed, CMake, Ninja). Windows: run setup.ps1 instead.
#
#   ./setup.sh                  # everything
#   ./setup.sh --no-system      # skip the package manager (no sudo); ESP-IDF only
#   ./setup.sh --idf-dir DIR    # where to put ESP-IDF (default ~/esp/esp-idf-v6.1.1)
#
# Safe to run again: anything already in place is left alone.
set -euo pipefail

IDF_VERSION="v6.1.1"
IDF_DIR="${HOME}/esp/esp-idf-${IDF_VERSION}"
SYSTEM=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-system) SYSTEM=0 ;;
        --idf-dir) IDF_DIR="$2"; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
    shift
done

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }

sudo_cmd() {
    if [[ $EUID -eq 0 ]]; then "$@"; elif have sudo; then sudo "$@"; else
        echo "need root for: $* (re-run as root, or with --no-system after installing it yourself)" >&2
        return 1
    fi
}

# --- System packages ---------------------------------------------------------
# ESP-IDF's own prerequisites plus the host-test tools.
install_system() {
    local os
    os="$(uname -s)"
    if [[ "$os" == "Darwin" ]]; then
        if ! have brew; then
            echo "Homebrew is needed on macOS: https://brew.sh" >&2
            exit 1
        fi
        say "Installing packages with Homebrew"
        # Apple clang ships libc++ with std::expected, so no extra C++ library.
        brew install git cmake ninja dfu-util python libusb ccache
        return
    fi

    if have apt-get; then
        say "Installing packages with apt"
        sudo_cmd apt-get update
        sudo_cmd apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv \
            cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
            clang libc++-dev libc++abi-dev
    elif have dnf; then
        say "Installing packages with dnf"
        sudo_cmd dnf install -y git wget flex bison gperf python3 python3-pip cmake ninja-build \
            ccache libffi-devel openssl-devel dfu-util libusb1 clang libcxx-devel libcxxabi-devel
    elif have pacman; then
        say "Installing packages with pacman"
        sudo_cmd pacman -S --needed --noconfirm git wget flex bison gperf python python-pip cmake \
            ninja ccache libffi openssl dfu-util libusb clang libc++
    elif have zypper; then
        say "Installing packages with zypper"
        sudo_cmd zypper install -y git wget flex bison gperf python3 python3-pip cmake ninja ccache \
            libffi-devel libopenssl-devel dfu-util libusb-1_0-0 clang libc++-devel
    else
        echo "No supported package manager (apt, dnf, pacman, zypper). Install git, python3," \
             "cmake, ninja, clang and libc++ yourself, then run with --no-system." >&2
        exit 1
    fi
}

# --- ESP-IDF -----------------------------------------------------------------
install_idf() {
    if [[ -d "$IDF_DIR/.git" ]]; then
        local current
        current="$(git -C "$IDF_DIR" describe --tags --exact-match 2>/dev/null || echo unknown)"
        if [[ "$current" != "$IDF_VERSION" ]]; then
            echo "$IDF_DIR holds ESP-IDF $current, not $IDF_VERSION. Move it aside or pass --idf-dir." >&2
            exit 1
        fi
        say "ESP-IDF $IDF_VERSION already at $IDF_DIR"
    else
        say "Cloning ESP-IDF $IDF_VERSION into $IDF_DIR"
        mkdir -p "$(dirname "$IDF_DIR")"
        git clone --depth 1 --branch "$IDF_VERSION" --recursive --shallow-submodules \
            https://github.com/espressif/esp-idf.git "$IDF_DIR"
    fi

    say "Installing the ESP32-S3 toolchain (GCC, GDB, OpenOCD, Python env)"
    "$IDF_DIR/install.sh" esp32s3

    say "Installing esp-clang and esp-clangd"
    # idf_tools.py needs the Python environment install.sh just created.
    # shellcheck disable=SC1091
    . "$IDF_DIR/export.sh" >/dev/null
    python "$IDF_DIR/tools/idf_tools.py" install esp-clang openocd-esp32
}

# --- Linux USB access --------------------------------------------------------
setup_usb() {
    [[ "$(uname -s)" == "Linux" && $SYSTEM -eq 1 ]] || return 0
    # shellcheck disable=SC1091
    . "$IDF_DIR/export.sh" >/dev/null
    local rules
    rules="$(dirname "$(dirname "$OPENOCD_SCRIPTS")")/contrib/60-openocd.rules"
    if [[ -f "$rules" && -d /etc/udev/rules.d && ! -f /etc/udev/rules.d/60-openocd.rules ]]; then
        say "Installing OpenOCD's udev rule (USB-JTAG without root)"
        sudo_cmd cp "$rules" /etc/udev/rules.d/
        sudo_cmd udevadm control --reload-rules || true
        sudo_cmd udevadm trigger || true
    fi
    local user="${SUDO_USER:-${USER:-$(id -un)}}"
    for group in dialout plugdev uucp; do
        if getent group "$group" >/dev/null && ! id -nG "$user" | grep -qw "$group"; then
            say "Adding $user to $group (serial port access; log out and in afterwards)"
            sudo_cmd usermod -aG "$group" "$user"
        fi
    done
}

# --- Check -------------------------------------------------------------------
check() {
    say "Checking"
    # shellcheck disable=SC1091
    . "$IDF_DIR/export.sh" >/dev/null
    local ok=1
    for tool in idf.py clang clangd xtensa-esp32s3-elf-gcc xtensa-esp32s3-elf-gdb openocd cmake ninja; do
        if have "$tool"; then printf '  %-24s %s\n' "$tool" "$(command -v "$tool")"
        else printf '  %-24s MISSING\n' "$tool"; ok=0; fi
    done
    # The host tests need a clang that is not ESP-IDF's (which only targets
    # Espressif chips); CMakeLists.txt skips ESP-IDF's when it looks.
    local host_cxx=""
    local dir
    IFS=: read -r -a dirs <<<"$PATH"
    for dir in "${dirs[@]}"; do
        [[ "$dir" == *.espressif* ]] && continue
        if [[ -x "$dir/clang++" ]]; then host_cxx="$dir/clang++"; break; fi
    done
    if [[ -n "$host_cxx" ]]; then printf '  %-24s %s\n' "host clang++" "$host_cxx"
    else printf '  %-24s MISSING (needed for the host tests)\n' "host clang++"; ok=0; fi

    if [[ $ok -eq 0 ]]; then
        echo "Some tools are missing; see the output above." >&2
        exit 1
    fi
}

[[ $SYSTEM -eq 1 ]] && install_system
install_idf
setup_usb
check

cat <<EOF

Done. In every new shell, before building:

    . "$IDF_DIR/export.sh"

Then, from firmware-cpp/:

    idf.py build                                                     # main program (esp-clang)
    idf.py --preset imu_demo flash monitor                           # a demo on the board
    cmake --preset host && cmake --build --preset host && ctest --preset host   # host tests

BUILD.md has the rest (editors, debugging).
EOF
