#!/bin/bash
# End to end: the brain's USB port with real PROS CLI (pros-cli in
# ~/venvs/pros, see SIMULATOR.md). Boots with --usb and the drivetrain sim,
# then over $TTY_SYS:
#   1. pros v5 status                      -> VEXos 1.1.5-0
#   2. pros upload --after run (slot 4)    -> VEXcode drive program; robot drives
#   3. pros upload --after run (slot 3)    -> printf program; its output and
#      echoed input come back on $TTY_USER
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
PROS=${PROS:-$HOME/venvs/pros/bin/pros}
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
sleep ${1:-22}
"$PROS" v5 status $TTY_SYS 2>&1 | grep -a "System version"
"$PROS" upload --target v5 --slot 4 --after run --name drive --no-compress-bin \
  "$REPO/programs/test-programs/vexcode_drive/build/vexcode_drive.bin" $TTY_SYS 2>&1 | tail -1
sleep 6
grep -a "pose" $ROBOT_LOG | tail -1 | cut -c1-40
"$PROS" upload --target v5 --slot 3 --after run --name print \
  "$REPO/programs/test-programs/vexcode_print/build/vexcode_print.bin" $TTY_SYS 2>&1 | tail -1
python3 -I - <<'PY'
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0, sent = b"", time.time(), False
while time.time() - t0 < 3:
    try:
        out += os.read(fd, 4096)
    except BlockingIOError:
        time.sleep(0.05)
    if not sent and time.time() - t0 > 1.5:
        os.write(fd, b"ok")
        sent = True
lines = out.decode(errors="replace").splitlines()
print("user port:", " | ".join(lines[:3]), "...", " | ".join(l for l in lines if l.startswith("echo")))
PY
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
