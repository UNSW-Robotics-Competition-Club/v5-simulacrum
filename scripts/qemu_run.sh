#!/bin/bash
# Boot VEXos on the vex-v5 machine, headless, FSBL skipped (CPU0 starts at
# MainLoop). The FSBL image is still loaded at 0: CPU1 uses its dispatch.
# Usage: qemu_run.sh [seconds] [extra qemu args...]
# Env:   PROGRAMS="n=p.bin" install programs in SPIFFS (see qspi_with_programs.sh)
#        USERBIN=prog.bin  load a user program at 0x03800000 instead of p5
#        LAUNCH_MS=N       start it on CPU1 N ms after reset (-M launch-ms)
#        QEMU_EXTRA="..."  extra qemu args (e.g. -trace ...)
# Logs unimplemented/guest-error accesses to ~/re/qemu-runs/m1.log
set -uo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
REPO=$V5SIM_ROOT
B=$REPO/extracted/boot
QEMU=~/re/qemu/build/qemu-system-arm
SECS=${1:-10}; shift || true
mkdir -p ~/re/qemu-runs
LOG=~/re/qemu-runs/m1.log
USERBIN=${USERBIN:-$B/p5_system_0_ps_03800000.bin}
QSPI=$(bash "$REPO/scripts/qspi_with_programs.sh")

timeout "$SECS" "$QEMU" -M "vex-v5,launch-ms=${LAUNCH_MS:-0}" -smp 2 -m 512M -nographic \
  -device loader,file=$B/p0_FSBL_ps_00000000.bin,addr=0x0,force-raw=on \
  -device loader,file=$B/p2_MainLoop_ps_01000000.bin,addr=0x01000000,force-raw=on \
  -device loader,file=$B/p3_system_0_ps_03400000.bin,addr=0x03400000,force-raw=on \
  -device loader,file=$B/p4_system_0_ps_037fc000.bin,addr=0x037fc000,force-raw=on \
  -device loader,file="$USERBIN",addr=0x03800000,force-raw=on \
  -drive if=mtd,index=8,format=raw,file="$QSPI" \
  -device loader,addr=0x01000000,cpu-num=0 \
  -d unimp,guest_errors -D "$LOG" ${QEMU_EXTRA:-} "$@"
echo "exit=$? log=$LOG lines=$(wc -l < "$LOG")"
