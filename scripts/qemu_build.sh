#!/bin/bash
# Configure and build QEMU (arm-softmmu only) under ~/re/qemu.
# Run from Windows: wsl -d Ubuntu-24.04 -- bash <repo>/scripts/qemu_build.sh
set -euo pipefail
# WSL appends the Windows PATH; a slow /mnt/c entry made every gcc helper
# lookup take ~40 s. The build only needs Linux tools.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
cd ~/re/qemu
if [ ! -f build/build.ninja ]; then
  ./configure --target-list=arm-softmmu --enable-sdl --disable-werror \
    --disable-docs --disable-tools --prefix="$HOME/re/qemu-install"
fi
ninja -C build -j"$(nproc)" qemu-system-arm
ls -l build/qemu-system-arm
build/qemu-system-arm -machine help | grep -i zynq
