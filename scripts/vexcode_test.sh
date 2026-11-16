#!/bin/bash
# End to end with real student code: build programs/test-programs/vexcode_drive with
# VEXcode's toolchain (programs/test-programs/vexcode_build.sh, Git Bash), then run it here
# on a simulated 6-motor drivetrain; screenshots ~/re/qemu-runs/vc{1,2}.ppm.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash $REPO/scripts/v5brain.sh --headless --run $REPO/programs/test-programs/vexcode_drive/build/vexcode_drive.bin --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 3; python3 -I $REPO/scripts/v5ctl.py $QMP shot:$R/vc1.ppm
sleep 8; python3 -I $REPO/scripts/v5ctl.py $QMP shot:$R/vc2.ppm quit
wait
grep -a "v5brain\]\|tmo\|abort\|Data abort" $R/v5brain.out
cut -c1-75 $ROBOT_LOG | tail -16
