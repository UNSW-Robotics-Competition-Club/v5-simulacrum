#!/bin/bash
# End to end: VEXcode 3-wire devices (programs/test-programs/vexcode_adi) on the brain's
# built-in ports (smart port 22): bumper on A, pneumatic on C, legacy encoder
# tracking wheel on E/F, ultrasonic on G/H. The robot should drive to the
# wall (x = 1.555 m), fire the piston (C=1), and back up one tracking-wheel
# turn (about 0.22 m).
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_adi/build/vexcode_adi.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --adi 22:A=bumper,C=piston,E=encoder,G=sonar --trace 22 --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 11
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/adi.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a -m3 "p22 < 50\|p22 < 51" $ROBOT_LOG | cut -c1-80
grep -a "pose" $ROBOT_LOG | tail -6 | cut -c1-44
grep -a -o "adi p22[^|]*" $ROBOT_LOG | tail -1
