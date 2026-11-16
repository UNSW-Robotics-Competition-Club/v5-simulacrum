#!/bin/bash
# Symlink qemu-vex/ sources into ~/re/qemu and register them (idempotent),
# then rebuild. Run: wsl -d Ubuntu-24.04 -- bash /mnt/c/.../scripts/qemu_install_vex.sh
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
REPO=$V5SIM_ROOT
Q=~/re/qemu
cd "$Q"

SRCS=(hw/arm/vex_v5.c hw/i2c/cadence_i2c.c hw/misc/vex_v5_nxp.c hw/display/logicvc.c hw/gpio/xlnx_axi_gpio.c hw/misc/xlnx_axi_mutex.c hw/input/vex_v5_touch.c hw/misc/vex_v5_nxp_uart.c hw/char/vex_v5_smartport.c hw/usb/vex_v5_usbdev.c hw/arm/vex_motor.c hw/misc/lpc82x.c hw/misc/lpc82x_timers.c hw/misc/lpc82x_link.c hw/misc/lpc82x_plant.c)
HDRS=(hw/misc/lpc82x.h)
for f in "${SRCS[@]}" "${HDRS[@]}"; do
  ln -sf "$REPO/qemu-vex/$f" "$Q/$f"
done

# Patches to upstream files (skip ones already applied).
for p in "$REPO"/qemu-vex/patches/*.patch; do
  if git apply --check "$p" 2>/dev/null; then
    git apply "$p" && echo "applied $(basename "$p")"
  elif ! git apply --reverse --check "$p" 2>/dev/null; then
    echo "patch does not apply: $p" >&2; exit 1
  fi
done

# meson paths are relative to hw/arm/
files=""
for f in "${SRCS[@]}"; do files+="'../${f#hw/}', "; done
# Rewrite only on change: touching meson.build makes ninja rerun the whole
# meson configure (minutes on a cold WSL).
line="arm_common_ss.add(when: 'CONFIG_VEX_V5', if_true: files(${files%, }))"
if ! grep -qxF "$line" hw/arm/meson.build; then
  sed -i '/CONFIG_VEX_V5/d' hw/arm/meson.build
  echo "$line" >> hw/arm/meson.build
fi

if ! grep -q '^config VEX_V5' hw/arm/Kconfig; then
  printf '\nconfig VEX_V5\n    bool\n    default y\n    depends on TCG && ARM\n    select ZYNQ\n    select I2C\n' >> hw/arm/Kconfig
fi

bash "$REPO/scripts/qemu_build.sh"
build/qemu-system-arm -machine help | grep -i vex
