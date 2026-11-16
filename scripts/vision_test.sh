#!/bin/bash
# End to end: legacy Vision Sensor (type 11) on port 11 with a red colour
# signature (programs/test-programs/vexcode_vision). The program should see the red block
# left of centre, turn left until it's centred (x ~158 of 316) and stop;
# the blue block is not reported. Output via the USB user port.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT; P=$REPO/programs/test-programs/vexcode_vision
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$P/build/vexcode_vision.bin" \
  --robot "--drive 1:2:green --vision 11 --blocks $P/blocks.json --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
python3 -I - <<'PY' > $R/vision_user.txt &
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 10:
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
grep -a "^start" $R/vision_user.txt | tail -1
grep -a "^done" $R/vision_user.txt | tail -1
grep -a "vision p" $ROBOT_LOG | tail -1 | sed 's/ | /\n/g' | grep -a "pose\|vision"
