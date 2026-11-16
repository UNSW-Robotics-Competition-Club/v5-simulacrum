#!/bin/bash
# End to end: the ui-test preset's competition control (--competition field)
# with the vexcode_ui starter template. The brain starts disabled on a field
# controller; a tap on the brain screen picks the "Turn" routine; then the
# field-control page's START runs a timed match, and Stop + Manual driver
# end it. Expect: the radio's status a9 (field, disabled) -> 91 (match
# autonomous) -> a1 (driver), /field.json reporting the match mid-run, the
# robot still until autonomous, then turning in place (the heading changes,
# the position barely does).
# Needs the program built first (Git Bash):
#   bash programs/test-programs/vexcode_build.sh programs/test-programs/vexcode_ui
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
IN=http://127.0.0.1:$VIEW_PORT/input
PROG=$REPO/programs/test-programs/vexcode_ui/build/vexcode_ui.bin
[ -f "$PROG" ] || { echo "build $PROG first (see the header of $0)" >&2; exit 1; }
post() {
  curl -s -X POST -H "Content-Type: application/json" --data "$1" "$IN" > /dev/null
}
bash "$REPO/scripts/v5brain.sh" --headless --run "$PROG" \
  --robot "--drive 1,2,3:4,5,6:blue --imu 10 --radio 11 --view $VIEW_PORT --competition field --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 4                                   # pre_auton: IMU calibration
# "Turn" is the second button: x 170..320, y 100..180 below the 32 px banner
python3 -I "$REPO/scripts/v5ctl.py" $QMP "tap:245,172" wait:1 "shot:$R/ui_selected.ppm"
sleep 2
post '{"field": "match"}'                          # START on the Match tab
sleep 3
curl -s "http://127.0.0.1:$VIEW_PORT/field.json" | cut -c1-160; echo
sleep 2
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/ui_after_auton.ppm"
post '{"field": "stop", "competition": "DRIVER"}'  # Manual: DRIVER
sleep 2
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -ao "radio p11\[DATA\] [A-Z]* status [0-9a-f]*" $ROBOT_LOG | uniq
grep -a "pose" $ROBOT_LOG | awk 'NR % 4 == 1' | cut -c1-60 | tail -12
