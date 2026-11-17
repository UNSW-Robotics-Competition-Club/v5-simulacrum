#!/bin/bash
# Configure and build QEMU (arm-softmmu only) under ~/re/qemu.
# Run from Windows: wsl -d Ubuntu-24.04 -- bash <repo>/scripts/qemu_build.sh
set -euo pipefail
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/platform.sh"
PATH=$(v5_path); export PATH     # platform.sh: no slow Windows PATH; Homebrew on macOS
cd ~/re/qemu
pyopt=()
if [ "$V5SIM_OS" = macos ]; then
  # QEMU's configure makes a venv that sees the Python's own site-packages,
  # and its checks crash on a broken package there (seen with Homebrew's
  # Python next to a python.org install). Give it a clean venv to start from.
  [ -x ~/re/qemu-py/bin/python3 ] || python3 -m venv ~/re/qemu-py
  pyopt=(--python="$HOME/re/qemu-py/bin/python3")
fi
if [ ! -f build/build.ninja ]; then
  ./configure --target-list=arm-softmmu --enable-sdl --disable-werror \
    --disable-docs --disable-tools --prefix="$HOME/re/qemu-install" ${pyopt[@]+"${pyopt[@]}"}
fi
ninja -C build -j"$(ncpu)" qemu-system-arm
ls -l build/qemu-system-arm
build/qemu-system-arm -machine help | grep -i zynq
