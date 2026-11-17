#!/bin/bash
# One-time setup of the simulator in WSL (Ubuntu-24.04) or on macOS
# (Homebrew; Apple Silicon or Intel): build tools, VEXos
# (downloaded from VEX by fetch_firmware.sh), QEMU
# v11.1.2 with the VEX device models under ~/re/qemu, the Arm GNU
# arm-none-eabi toolchain under ~/re/arm-gnu (builds the test programs), and
# the Python venv for MuJoCo physics. Idempotent: rerun it after pulling qemu-vex/ changes.
# Run from Windows:  wsl -d Ubuntu-24.04 -- bash <repo>/scripts/setup.sh
# On macOS:          bash scripts/setup.sh
# Env: SKIP_APT=1 skips the apt / brew step (no sudo), SKIP_MUJOCO=1 MuJoCo.
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
. "$V5SIM_ROOT/scripts/platform.sh"
PATH=$(v5_path); export PATH      # platform.sh: no slow Windows PATH; Homebrew on macOS
QEMU_TAG=v11.1.2
# The arm-none-eabi toolchain for this machine: Arm's GNU toolchain build,
# and xPack's build of the same GCC 14.2 (GitHub releases) where Arm has no
# build here or its download server refuses (it answers 503 to some
# networks). Both pinned by SHA-256.
ARM_VER=14.2.rel1
XPACK_VER=14.2.1-1.1
ARM_HOST="" ARM_SHA256=""
case $V5SIM_OS-$(uname -m) in
  macos-arm64)  XPACK_HOST=darwin-arm64
                XPACK_SHA256=@XPACK_SHA_DARWIN_ARM64@ ;;
  macos-x86_64) XPACK_HOST=darwin-x64
                XPACK_SHA256=@XPACK_SHA_DARWIN_X64@ ;;
  *-x86_64)     ARM_HOST=x86_64
                ARM_SHA256=62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823
                XPACK_HOST=linux-x64
                XPACK_SHA256=@XPACK_SHA_LINUX_X64@ ;;
  *) echo "no arm-none-eabi toolchain build for $(uname -sm) here" >&2; exit 1 ;;
esac

case $V5SIM_ROOT in *[[:space:]]*)
  echo "the repo path has a space in it ($V5SIM_ROOT); the scripts need a path without spaces" >&2
  exit 1 ;;
esac

if [ "$V5SIM_OS" = macos ]; then
  xcode-select -p >/dev/null 2>&1 || {
    echo "needs Apple's Command Line Tools: run  xcode-select --install  then rerun this" >&2; exit 1; }
  command -v brew >/dev/null || {
    echo "needs Homebrew (https://brew.sh): install it, then rerun this" >&2; exit 1; }
  if [ "${SKIP_APT:-0}" != 1 ]; then
    echo "== build tools (Homebrew)"
    brew install ninja meson pkg-config glib pixman sdl2 python@3.12 git
  fi
  # Homebrew's Python: macOS's own python3 is too old to rely on. v5_path
  # puts it first for every script.
  PATH=$(v5_path); export PATH
  PY=$(brew --prefix python@3.12)/bin/python3.12
elif [ "${SKIP_APT:-0}" != 1 ]; then
  echo "== build tools (sudo)"
  sudo apt-get update -q
  sudo apt-get install -y -q ninja-build meson pkg-config gcc libglib2.0-dev libpixman-1-dev \
    libsdl2-dev python3-venv python3-serial flex bison git curl
fi
PY=${PY:-python3}

echo "== VEXos firmware -> extracted/"
bash "$V5SIM_ROOT/scripts/fetch_firmware.sh"

if [ ! -d ~/re/qemu/.git ]; then
  echo "== QEMU $QEMU_TAG -> ~/re/qemu"
  mkdir -p ~/re
  git clone -q --depth 1 --branch "$QEMU_TAG" https://gitlab.com/qemu-project/qemu.git ~/re/qemu
fi
echo "== VEX device models + build (first build takes several minutes)"
bash "$V5SIM_ROOT/scripts/qemu_install_vex.sh"

# fetch URL FILE SHA256: download (unless there already) and check it
fetch() {
  [ -f "$2" ] && sha256_ok "$2" "$3" && return 0
  rm -f "$2"
  curl -fsSL --retry 3 -o "$2" "$1" && sha256_ok "$2" "$3" && return 0
  echo "  couldn't get a matching $(basename "$2")" >&2
  rm -f "$2"
  return 1
}
if [ ! -x ~/re/arm-gnu/bin/arm-none-eabi-gcc ]; then
  echo "== arm-none-eabi GCC 14.2 -> ~/re/arm-gnu (programs/test-programs/build.sh, sdk_test.sh)"
  mkdir -p ~/re/dl
  tarball=""
  if [ -n "$ARM_HOST" ]; then
    f=~/re/dl/arm-gnu-toolchain-$ARM_VER-$ARM_HOST-arm-none-eabi.tar.xz
    fetch "https://developer.arm.com/-/media/Files/downloads/gnu/$ARM_VER/binrel/$(basename "$f")" \
      "$f" "$ARM_SHA256" && tarball=$f
  fi
  if [ -z "$tarball" ]; then
    f=~/re/dl/xpack-arm-none-eabi-gcc-$XPACK_VER-$XPACK_HOST.tar.gz
    fetch "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v$XPACK_VER/$(basename "$f")" \
      "$f" "${V5SIM_XPACK_SHA256:-$XPACK_SHA256}" && tarball=$f
  fi
  [ -n "$tarball" ] || { echo "no arm-none-eabi toolchain: check the network and rerun" >&2; exit 1; }
  rm -rf ~/re/arm-gnu && mkdir ~/re/arm-gnu
  tar -xf "$tarball" -C ~/re/arm-gnu --strip-components=1
fi
~/re/arm-gnu/bin/arm-none-eabi-gcc --version | head -1

# The robot sim's Python (v5brain.sh uses ~/venvs/sim when it's there): on
# macOS always, for Homebrew's Python and pyserial; elsewhere for MuJoCo.
if [ "$V5SIM_OS" = macos ] || [ "${SKIP_MUJOCO:-0}" != 1 ]; then
  if [ ! -x ~/venvs/sim/bin/python ]; then
    echo "== Python venv -> ~/venvs/sim"
    "$PY" -m venv --system-site-packages ~/venvs/sim
  fi
  pkgs="pytest"
  [ "$V5SIM_OS" = macos ] && pkgs="$pkgs pyserial"
  [ "${SKIP_MUJOCO:-0}" != 1 ] && pkgs="$pkgs mujoco"
  # pytest runs the unit tests (cd sim && ~/venvs/sim/bin/python -m pytest tests)
  ~/venvs/sim/bin/pip install -q $pkgs
fi

mkdir -p ~/re/qemu-runs
if [ "$V5SIM_OS" = macos ]; then
  echo "SETUP_DONE: run ./v5sim from the repo (e.g. ./v5sim competition)"
else
  echo "SETUP_DONE: run V5Sim.cmd from Windows, or scripts/v5brain.sh in WSL"
fi
