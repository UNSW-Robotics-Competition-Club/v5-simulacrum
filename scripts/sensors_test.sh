#!/bin/bash
# End to end: Optical (port 8) and GPS (port 9) sensors with a game object
# (programs/test-programs/vexcode_sensors/blocks.json, one blue block 0.7 m ahead). The
# VEXcode program drives with its intake running until the optical sensor
# sees an object; the screen should read "saw blue", and the GPS line should
# match the robot's pose (x in mm, heading 90 = facing +x).
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_sensors/build/vexcode_sensors.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --intake 7 --optical 8 --gps 9 --blocks $REPO/programs/test-programs/vexcode_sensors/blocks.json --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 9
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/sensors.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a -o "pose x=[^|]*\|optical p8[^|]*\|gps p9[^|]*\|blocks [0-9]* held \[[^]]*\]" $ROBOT_LOG | paste - - - - | uniq | tail -4
