#!/bin/bash
# End to end: the brain's front button and the device dashboard. Runs the
# hello program (it drives a motor on port 1), reads /devices.json, then
# holds the button 1.5 s through the field view's /input: the program must
# stop (motor back to 0 V). A tap of the B key through QEMU's keyboard path
# then goes Back on the brain screen (the screen changes).
#   button_test.sh
export PATH=/usr/local/bin:/usr/bin:/bin
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
IN=http://127.0.0.1:$VIEW_PORT
bash "$REPO/scripts/v5brain.sh" --headless --run "$REPO/programs/test-programs/hello/hello.bin" \
  --robot "--motor 1:green --view $VIEW_PORT --status 0.5" > $R/v5brain.out 2>&1 < /dev/null &
sleep 6
curl -s $IN/devices.json | python3 -c '
import json, sys
m = next(d for d in json.load(sys.stdin)["devices"] if d["kind"] == "motor")
print("dashboard: port", m["port"], m["link"], m["velocity_rpm"], "rpm", m["voltage_v"], "V mode", m["mode"])
'
curl -s -o /dev/null -w "page %{http_code}\n" $IN/dashboard
grep -a "p1\[DATA\]" $ROBOT_LOG | tail -1 | cut -c1-40
curl -s -H 'Content-Type: application/json' -d '{"brain_button": 1.5}' $IN/input
sleep 3
echo "after hold: $(grep -a "p1\[" $ROBOT_LOG | tail -1 | cut -c1-40)"
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/button_before.ppm"
python3 -I - <<'PY'
import os
import sys, time
sys.path.insert(0, os.environ["V5SIM_ROOT"] + "/scripts")
import v5ctl
q = v5ctl.Qmp(os.environ["QMP"])
key = lambda down: q.cmd("input-send-event", events=[
    {"type": "key", "data": {"down": down, "key": {"type": "qcode", "data": "b"}}}])
key(True); time.sleep(0.3); key(False); time.sleep(1.0)
q.cmd("screendump", filename=os.path.expanduser("~/re/qemu-runs") + "/button_after.ppm")
PY
cmp -s $R/button_before.ppm $R/button_after.ppm && echo "B key: screen unchanged" || echo "B key: screen changed"
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a -i "brain button\|Traceback" $ROBOT_LOG | head -3
