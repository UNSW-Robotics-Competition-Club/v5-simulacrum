#!/bin/bash
# End to end: a 5.5 W motor (type 5) on port 7 with VEXcode (programs/
# vexcode_motor55). The brain should report motor type 1 (5.5 W), the motor
# spin at ~100 rpm, then settle at 720 degrees. Output via the USB user port.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$REPO/programs/test-programs/vexcode_motor55/build/vexcode_motor55.bin" \
  --robot "--motor 7:5.5w --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 1
python3 -I - <<'PY' > $R/motor55_user.txt &
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
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a "spin 100" $R/motor55_user.txt | tail -1
grep -a "at 720\|done" $R/motor55_user.txt | tail -1
grep -a -o "p7\[[^|]*" $ROBOT_LOG | tail -1
