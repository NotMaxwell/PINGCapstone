# Installs everything firmware-cpp needs on Windows, without administrator
# rights: ESP-IDF (pinned) with esp-clang, GCC, OpenOCD and GDB, and
# llvm-mingw (clang + libc++) for the host tests. Everything goes under your
# user folder; PATH changes are made for your account only.
#
#   powershell -ExecutionPolicy Bypass -File setup.ps1
#   powershell -ExecutionPolicy Bypass -File setup.ps1 -IdfDir D:\esp\idf -SkipHostTools
#
# Safe to run again: anything already in place is left alone.
#
# Not covered, because it needs administrator rights: the WinUSB driver for
# JTAG debugging (idf-env driver install --espressif). Flashing and the serial
# monitor need no driver on Windows 10/11.
#
# If your organisation blocks programs from running in your user folder
# (application control), the tools install but cannot start; this script
# detects that at the end. BUILD.md, "On a locked-down PC", has the
# browser-only route.

param(
    [string]$IdfVersion = "v6.1.1",
    [string]$IdfDir = (Join-Path $HOME "esp\esp-idf-v6.1.1"),
    [string]$ToolsDir = (Join-Path $HOME "tools"),
    [switch]$SkipHostTools
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"   # Invoke-WebRequest is very slow with the progress bar

function Say($text) { Write-Host "`n==> $text" -ForegroundColor Cyan }
function Have($name) { [bool](Get-Command $name -ErrorAction SilentlyContinue) }

function Add-UserPath($dir) {
    $current = [Environment]::GetEnvironmentVariable("Path", "User")
    $parts = @($current -split ";" | Where-Object { $_ })
    if ($parts -notcontains $dir) {
        [Environment]::SetEnvironmentVariable("Path", (@($dir) + $parts) -join ";", "User")
        Write-Host "  added to your PATH: $dir"
    }
    if (($env:Path -split ";") -notcontains $dir) { $env:Path = "$dir;$env:Path" }
}

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "User") + ";" +
                [Environment]::GetEnvironmentVariable("Path", "Machine")
}

# --- Git and Python ----------------------------------------------------------
# ESP-IDF's installer needs both. winget's --scope user installs need no admin.
function Ensure-Prerequisite($command, $wingetId, $what) {
    if (Have $command) { return }
    if (-not (Have "winget")) {
        throw "$what is missing and winget is not available. Install $what for your user " +
              "(the 'just me' option) and run this again."
    }
    Say "Installing $what for your user (winget)"
    winget install --id $wingetId --scope user --silent --accept-package-agreements --accept-source-agreements
    Refresh-Path
    if (-not (Have $command)) {
        throw "$what installed but '$command' is not on PATH yet. Open a new PowerShell and run this again."
    }
}

Ensure-Prerequisite "git" "Git.Git" "Git"
Ensure-Prerequisite "python" "Python.Python.3.12" "Python 3"

# --- ESP-IDF -----------------------------------------------------------------
if (Test-Path (Join-Path $IdfDir ".git")) {
    $current = (& git -C $IdfDir describe --tags --exact-match 2>$null)
    if ($current -ne $IdfVersion) {
        throw "$IdfDir holds ESP-IDF '$current', not $IdfVersion. Move it aside or pass -IdfDir."
    }
    Say "ESP-IDF $IdfVersion already at $IdfDir"
} else {
    Say "Cloning ESP-IDF $IdfVersion into $IdfDir"
    New-Item -ItemType Directory -Force -Path (Split-Path $IdfDir) | Out-Null
    git clone --depth 1 --branch $IdfVersion --recursive --shallow-submodules `
        https://github.com/espressif/esp-idf.git $IdfDir
    if ($LASTEXITCODE -ne 0) { throw "git clone failed" }
}

Say "Installing the ESP32-S3 toolchain (GCC, GDB, OpenOCD, Python env)"
& (Join-Path $IdfDir "install.ps1") esp32s3
if ($LASTEXITCODE -ne 0) { throw "ESP-IDF install.ps1 failed" }

Say "Installing esp-clang and esp-clangd"
. (Join-Path $IdfDir "export.ps1") | Out-Null
python (Join-Path $IdfDir "tools\idf_tools.py") install esp-clang openocd-esp32
if ($LASTEXITCODE -ne 0) {
    Write-Warning ("idf_tools.py reported a problem. If it says 'Access is denied', your " +
                   "organisation blocks programs in your user folder; see the end of this script's output.")
}

# --- Host-test compiler ------------------------------------------------------
# llvm-mingw: a portable clang that already uses libc++, so std::expected
# works, and needs no Visual Studio. CMake and Ninja come from ESP-IDF.
if (-not $SkipHostTools) {
    $llvm = Join-Path $ToolsDir "llvm-mingw"
    if (Test-Path (Join-Path $llvm "bin\clang++.exe")) {
        Say "llvm-mingw already at $llvm"
    } else {
        Say "Downloading llvm-mingw (host-test compiler)"
        $release = Invoke-RestMethod "https://api.github.com/repos/mstorsjo/llvm-mingw/releases/latest" `
            -Headers @{ "User-Agent" = "ping-firmware-setup" }
        $asset = $release.assets | Where-Object { $_.name -match "ucrt-x86_64\.zip$" } | Select-Object -First 1
        if (-not $asset) { throw "No ucrt-x86_64 zip in llvm-mingw $($release.tag_name)" }
        New-Item -ItemType Directory -Force -Path $ToolsDir | Out-Null
        $zip = Join-Path $env:TEMP $asset.name
        Invoke-WebRequest $asset.browser_download_url -OutFile $zip
        $staging = Join-Path $ToolsDir "llvm-mingw-unpack"
        Expand-Archive $zip -DestinationPath $staging -Force
        # The zip holds one versioned folder; keep a stable name so PATH survives upgrades.
        Move-Item (Get-ChildItem $staging -Directory | Select-Object -First 1).FullName $llvm
        Remove-Item $staging, $zip -Recurse -Force
    }
    Add-UserPath (Join-Path $llvm "bin")
}

# --- Check -------------------------------------------------------------------
Say "Checking that each tool runs"
$blocked = @()
$missing = @()
$checks = [ordered]@{
    "xtensa-esp32s3-elf-gcc" = "--version"
    "clang"                  = "--version"
    "xtensa-esp32s3-elf-gdb" = "--version"
    "openocd"                = "--version"
    "cmake"                  = "--version"
    "ninja"                  = "--version"
}
if (-not $SkipHostTools) { $checks["$ToolsDir\llvm-mingw\bin\clang++.exe"] = "--version" }

foreach ($tool in $checks.Keys) {
    $cmd = Get-Command $tool -ErrorAction SilentlyContinue
    if (-not $cmd) { $missing += $tool; Write-Host ("  {0,-26} MISSING" -f $tool); continue }
    try {
        & $cmd.Source $checks[$tool] *> $null
        Write-Host ("  {0,-26} ok" -f $tool)
    } catch {
        if ($_.Exception.Message -match "denied|blocked|policy") { $blocked += $tool } else { $missing += $tool }
        Write-Host ("  {0,-26} FAILED: {1}" -f $tool, $_.Exception.Message)
    }
}

if ($blocked.Count -gt 0) {
    Write-Host @"

Windows refused to run: $($blocked -join ', ').
That is your organisation's application control, not a broken install. Either
ask IT to allow $HOME\.espressif\tools and $ToolsDir, or use the browser-only
route in BUILD.md ("On a locked-down PC"): GitHub Actions builds the firmware,
and Chrome/Edge flash it.
"@ -ForegroundColor Yellow
    exit 1
}
if ($missing.Count -gt 0) {
    Write-Host "`nMissing: $($missing -join ', '). See the output above." -ForegroundColor Yellow
    exit 1
}

Write-Host @"

Done. In every new PowerShell, before building:

    . $IdfDir\export.ps1

Then, from firmware-cpp\:

    idf.py build                                   # main program (esp-clang)
    idf.py --preset imu_demo -p COM5 flash monitor # a demo on the board (your COM port)
    cmake --preset host; cmake --build --preset host; ctest --preset host   # host tests

BUILD.md has the rest (editors, debugging).
"@ -ForegroundColor Green
