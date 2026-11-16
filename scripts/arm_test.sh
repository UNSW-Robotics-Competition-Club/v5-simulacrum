#!/bin/bash
# End to end: a loaded arm (sim/v5sim/mechanism.py) under VEXcode's motor
# control (programs/test-programs/vexcode_arm). The motor must hold the arm level against
# gravity (about 0.9 N.m at the motor: ~0.78 A on a red motor, position 150),
# and the arm must fall back to its -30 degree stop when let go (motor ~0).
export PATH=/usr/local/bin:/usr/bin:/bin
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$REPO/programs/test-programs/vexcode_arm/build/vexcode_arm.bin" \
  --robot "--motor 8:red --arm 8:5:12:1.5:-30:-30:90 --status 0.5" > $R/arm.out 2>&1 < /dev/null &
brain=$!
sleep 3
python3 -I - <<'PY' > $R/arm_user.txt
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 9:
    try:
        out += os.read(fd, 4096)
    except BlockingIOError:
        time.sleep(0.05)
print(out.decode(errors="replace"))
PY
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit >/dev/null 2>&1
wait $brain
grep -a "^hold" $R/arm_user.txt | tail -1
grep -a "^done" $R/arm_user.txt | tail -1
