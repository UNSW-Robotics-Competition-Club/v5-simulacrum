#!/bin/bash
# One-time setup of the simulator in WSL (Ubuntu-24.04): build tools, VEXos
# (downloaded from VEX by fetch_firmware.sh), QEMU
# v11.1.2 with the VEX device models under ~/re/qemu, the Arm GNU
# arm-none-eabi toolchain under ~/re/arm-gnu (builds the test programs), and
# the Python venv for MuJoCo physics. Idempotent: rerun it after pulling qemu-vex/ changes.
# Run from Windows:  wsl -d Ubuntu-24.04 -- bash <repo>/scripts/setup.sh
# Env: SKIP_APT=1 skips the apt step (no sudo), SKIP_MUJOCO=1 the venv.
set -euo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin   # see qemu_build.sh
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
QEMU_TAG=v11.1.2
ARM_VER=14.2.rel1
ARM_SHA256=62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823

case $V5SIM_ROOT in *[[:space:]]*)
  echo "the repo path has a space in it ($V5SIM_ROOT); the scripts need a path without spaces" >&2
  exit 1 ;;
esac

if [ "${SKIP_APT:-0}" != 1 ]; then
  echo "== build tools (sudo)"
  sudo apt-get update -q
  sudo apt-get install -y -q ninja-build meson pkg-config gcc libglib2.0-dev libpixman-1-dev \
    libsdl2-dev python3-venv python3-serial flex bison git curl
fi

echo "== VEXos firmware -> extracted/"
bash "$V5SIM_ROOT/scripts/fetch_firmware.sh"

if [ ! -d ~/re/qemu/.git ]; then
  echo "== QEMU $QEMU_TAG -> ~/re/qemu"
  mkdir -p ~/re
  git clone -q --depth 1 --branch "$QEMU_TAG" https://gitlab.com/qemu-project/qemu.git ~/re/qemu
fi
echo "== VEX device models + build (first build takes several minutes)"
bash "$V5SIM_ROOT/scripts/qemu_install_vex.sh"

if [ ! -x ~/re/arm-gnu/bin/arm-none-eabi-gcc ]; then
  echo "== Arm GNU $ARM_VER arm-none-eabi -> ~/re/arm-gnu (programs/test-programs/build.sh, sdk_test.sh)"
  mkdir -p ~/re/dl
  tarball=~/re/dl/arm-gnu-toolchain-$ARM_VER-x86_64-arm-none-eabi.tar.xz
  [ -f "$tarball" ] || curl -fsSL -o "$tarball" \
    "https://developer.arm.com/-/media/Files/downloads/gnu/$ARM_VER/binrel/$(basename "$tarball")"
  echo "$ARM_SHA256  $tarball" | sha256sum -c -
  rm -rf ~/re/arm-gnu && mkdir ~/re/arm-gnu
  tar -xJf "$tarball" -C ~/re/arm-gnu --strip-components=1
fi
~/re/arm-gnu/bin/arm-none-eabi-gcc --version | head -1

if [ "${SKIP_MUJOCO:-0}" != 1 ]; then
  if [ ! -x ~/venvs/sim/bin/python ]; then
    echo "== MuJoCo venv -> ~/venvs/sim"
    python3 -m venv --system-site-packages ~/venvs/sim
  fi
  # pytest runs the unit tests (cd sim && ~/venvs/sim/bin/python -m pytest tests)
  ~/venvs/sim/bin/python -c "import mujoco, pytest" 2>/dev/null     || ~/venvs/sim/bin/pip install -q mujoco pytest
fi

mkdir -p ~/re/qemu-runs
echo "SETUP_DONE: run V5Sim.cmd from Windows, or scripts/v5brain.sh in WSL"
