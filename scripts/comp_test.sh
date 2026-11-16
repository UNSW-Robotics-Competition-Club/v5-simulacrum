#!/bin/bash
# End to end: the VEXcode competition template (programs/test-programs/vexcode_comp) through
# a short timed match from the sim (--match 14:6:8). A turn stick is held from
# before the match starts, so the robot must: stay still while disabled, drive
# straight in autonomous (the SDK zeroes the sticks then), turn in place in
# driver control, and stop when the match ends.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
IN=http://127.0.0.1:$VIEW_PORT/input
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_comp/build/vexcode_comp.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --imu 10 --controller 11 --view $VIEW_PORT --match 14:6:8 --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
sleep 9
curl -s -X POST -H "Content-Type: application/json" \
  --data '{"axes": {"Axis1": 40}}' "$IN"
sleep 7
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/comp_auton.ppm"
sleep 9
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/comp_driver.ppm"
sleep 8
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/comp_end.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a "match:" $ROBOT_LOG
grep -a "pose" $ROBOT_LOG | awk '$1+0 >= 9' | cut -c1-48
