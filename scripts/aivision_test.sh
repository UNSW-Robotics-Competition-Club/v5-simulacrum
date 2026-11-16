#!/bin/bash
# End to end: AI Vision Sensor (port 10) with a colour signature. The VEXcode
# program (programs/test-programs/vexcode_aivision) turns until the red block of
# blocks.json is centred in the camera image, then drives at it with the
# intake running. Expect the robot to turn left (+heading) and end holding
# [red], with the blue block untouched.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_aivision/build/vexcode_aivision.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --intake 7 --aivision 10 --blocks $REPO/programs/test-programs/vexcode_aivision/blocks.json --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 11
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/aivision.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a -o "pose x=[^|]*\|aivision p10[^|]*\|blocks [0-9]* held \[[^]]*\]" $ROBOT_LOG | paste - - - | uniq | tail -6
