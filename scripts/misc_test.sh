#!/bin/bash
# End to end: Electromagnet 3, Light Tower 4, CTE pneumatics 5, LED 6 with
# VEXcode (programs/test-programs/vexcode_misc). The brain should report types 17/27/30/3,
# the magnet's power and current, the tower's colours, the cylinder states
# and counters, and the LED colour. Then the field view's /input holds the
# tower button and swipes up, up, left at the optical sensor on 8: the
# program should count 2 ups and 1 left. Output via the USB user port.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --run "$REPO/programs/test-programs/vexcode_misc/build/vexcode_misc.bin" \
  --robot "--magnet 3 --tower 4 --pneumatic 5 --led 6 --optical 8 --view $VIEW_PORT --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
python3 -I - <<'PY' > $R/misc_user.txt &
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
post() { curl -s -H 'Content-Type: application/json' -d "$1" localhost:$VIEW_PORT/input; }
sleep 8
post '{"tower": true}'
for g in up up left; do sleep 0.5; post "{\"gesture\": \"$g\"}"; done
wait %2 2>/dev/null
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a "^idle" $R/misc_user.txt | tail -1
grep -a "^on" $R/misc_user.txt | tail -1
grep -a "^done" $R/misc_user.txt | tail -1
grep -a "magnet p" $ROBOT_LOG | tail -1 | sed 's/ | /\n/g' | grep -a "magnet\|tower\|pneumatic\|led p"
