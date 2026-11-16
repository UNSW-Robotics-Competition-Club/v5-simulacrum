#!/bin/bash
# End to end: the VEXcode competition template (programs/test-programs/vexcode_comp) with
# the controller over a VEXnet radio on port 11 instead of a tether, the
# match run as field control (--match 14:6:8). Same expectations as
# comp_test: still while disabled, straight in autonomous, turning in
# driver control, stopped after; the radio's status line shows the field
# byte (a9, 91, 99, a1, a9) and the seconds counter.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
IN=http://127.0.0.1:$VIEW_PORT/input
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_comp/build/vexcode_comp.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --imu 10 --radio 11 --view $VIEW_PORT --match 14:6:8 --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
sleep 9
curl -s -X POST -H "Content-Type: application/json" \
  --data '{"axes": {"Axis1": 40}}' "$IN"
sleep 7
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/radio_auton.ppm"
sleep 9
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/radio_driver.ppm"
sleep 8
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/radio_end.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a "match:" $ROBOT_LOG
grep -a "pose" $ROBOT_LOG | awk '$1+0 >= 9' | cut -c1-48
grep -a "radio p11" $ROBOT_LOG | sed 's/.*radio p11/radio p11/' | cut -c1-60 | awk '$5 != last {print; last = $5}'
