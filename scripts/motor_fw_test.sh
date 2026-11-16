#!/bin/bash
# End to end with the real motor firmware (notes/motor-emulation.md): the
# VEXcode motor program (programs/test-programs/vexcode_motor55) drives a
# motor on port 7 that runs DEVFRM02 / DEVFRM05 in a vex-motor QEMU
# (--motor-fw 7). It should spin at ~100 rpm, then settle at 720 degrees.
#   motor_fw_test.sh [green|5.5w ...]     (default: both)
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
BIN=$REPO/programs/test-programs/vexcode_motor55/build/vexcode_motor55.bin
[ -f "$BIN" ] || { echo "build it first (Git Bash): bash programs/test-programs/vexcode_build.sh programs/test-programs/vexcode_motor55" >&2; exit 1; }
for cart in "${@:-green 5.5w}"; do
  for c in $cart; do
    echo "== motor $c, real firmware"
    bash "$REPO/scripts/v5brain.sh" --headless --usb --run "$BIN" \
      --robot "--motor 7:$c --motor-fw 7 --status 1" > $R/v5brain.out 2>&1 < /dev/null &
    e2e_started "" 1
    python3 -I - <<'PY' > $R/motor_fw_user.txt &
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 6:
    try:
        out += os.read(fd, 4096)
    except BlockingIOError:
        time.sleep(0.05)
print(out.decode(errors="replace"))
PY
    wait %2 2>/dev/null
    python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
    wait
    grep -a "v5brain\]\|tmo\|abort\|Traceback\|Error" $R/v5brain.out
    grep -a "spin 100" $R/motor_fw_user.txt | tail -1
    grep -a "at 720\|done" $R/motor_fw_user.txt | tail -1
    grep -a -o "p7\[[^|]*" $ROBOT_LOG | tail -1
  done
done
