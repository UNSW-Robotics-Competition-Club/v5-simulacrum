#!/bin/bash
# End to end: legacy 3-wire gyro, pot, line tracker, light sensor and
# accelerometer on the brain's ports (programs/test-programs/vexcode_adi_extra). The gyro
# should read ~0 after calibrating and ~90 after the right turn (robot
# heading in the status line), the pot ~100 deg with its motor at 100.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$REPO/programs/test-programs/vexcode_adi_extra/build/vexcode_adi_extra.bin" \
  --robot "--drive 1:2:green --motor 7:green --adi 22:A=gyro,B=pot:7,C=line,D=light,E=accel --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
python3 -I - <<'PY' > $R/adi_extra_user.txt &
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 12:
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
grep -a "^cal" $R/adi_extra_user.txt | tail -1
grep -a "^turn" $R/adi_extra_user.txt | tail -1
grep -a "^done" $R/adi_extra_user.txt | tail -1
grep -a "pose" $ROBOT_LOG | tail -1 | sed 's/ | /\n/g' | grep -a "pose\|adi"
