#!/bin/bash
# Two sim brains: TESTER (instance 8, runs programs/test-programs/radiolink) cabled via
# scripts/port_bridge.py to the ROBOT brain (instance 9, motors on 1-3).
# Tester port 1 <-> robot port 21. Proves a brain can drive the radio uplink.
export PATH=/usr/local/bin:/usr/bin:/bin
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
bash "$REPO/programs/test-programs/build.sh" radiolink >/dev/null 2>&1 || { echo build failed; exit 1; }

bash "$REPO/scripts/v5brain.sh" --id 9 --headless \
  --robot "--motor 1:green --motor 2:blue --motor 3:5.5w" > $R/v5b_robot.out 2>&1 < /dev/null &
bash "$REPO/scripts/v5brain.sh" --id 8 --headless --usb \
  --run "$REPO/programs/test-programs/radiolink/radiolink.bin" > $R/v5b_tester.out 2>&1 < /dev/null &
# bring the bridge up immediately; it retries connect until both sockets exist
python3 "$REPO/scripts/port_bridge.py" /tmp/v5sim8/v5port1.sock /tmp/v5sim9/v5port21.sock \
  > $R/bridge.out 2>&1 &
BR=$!
sleep ${1:-26}
echo "=== bridge:"; head -2 $R/bridge.out
echo "=== tester user port (last distinct lines):"
python3 -I - /tmp/v5sim8/ttyV5user <<'PY'
import os, sys, time, tty
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK); tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 5:
    try: out += os.read(fd, 4096)
    except BlockingIOError: time.sleep(0.05)
lines=[l for l in out.decode(errors="replace").splitlines() if l.strip()]
for l in lines[-6:]: print("  ", l)
PY
kill $BR 2>/dev/null
python3 -I "$REPO/scripts/v5ctl.py" /tmp/v5sim8/v5qmp.sock quit >/dev/null 2>&1
python3 -I "$REPO/scripts/v5ctl.py" /tmp/v5sim9/v5qmp.sock quit >/dev/null 2>&1
wait 2>/dev/null
