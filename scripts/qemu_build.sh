#!/bin/bash
# Configure and build QEMU (arm-softmmu only) under ~/re/qemu.
# Run from Windows: wsl -d Ubuntu-24.04 -- bash <repo>/scripts/qemu_build.sh
set -euo pipefail
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/platform.sh"
PATH=$(v5_path); export PATH     # platform.sh: no slow Windows PATH; Homebrew on macOS
cd ~/re/qemu
if [ ! -f build/build.ninja ]; then
  ./configure --target-list=arm-softmmu --enable-sdl --disable-werror \
    --disable-docs --disable-tools --prefix="$HOME/re/qemu-install"
fi
ninja -C build -j"$(ncpu)" qemu-system-arm
ls -l build/qemu-system-arm
build/qemu-system-arm -machine help | grep -i zynq
