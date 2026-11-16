#!/bin/bash
# End to end: AI Vision AprilTags and colour codes (programs/test-programs/vexcode_aitags).
# Tag 3 on the far wall should be seen centred and upright (angle 0), the
# red-blue code as one box spanning both blocks; after the right turn the
# tag's centre should move left in the image. Output via the USB user port.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT; P=$REPO/programs/test-programs/vexcode_aitags
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$P/build/vexcode_aitags.bin" \
  --robot "--drive 1:2:green --aivision 10 --tags $P/tags.json --blocks $P/blocks.json --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
python3 -I - <<'PY' > $R/aitags_user.txt &
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
grep -a "^ahead tags" $R/aitags_user.txt | tail -1
grep -a "^ahead codes" $R/aitags_user.txt | tail -1
grep -a "^turned tags" $R/aitags_user.txt | tail -1
grep -a "aivision p" $ROBOT_LOG | tail -1 | sed 's/ | /\n/g' | grep -a "pose\|aivision"
