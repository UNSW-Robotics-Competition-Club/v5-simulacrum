#!/bin/bash
# End to end: VEXcode inertial turn-to-90 on the simulated drivetrain + IMU.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash $REPO/scripts/v5brain.sh --headless --run $REPO/programs/test-programs/vexcode_imu/build/vexcode_imu.bin --robot "--drive 1,2,3:4,5,6:blue --imu 10 --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 17; python3 -I $REPO/scripts/v5ctl.py $QMP shot:$R/imu1.ppm quit; wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out; cut -c1-60 $ROBOT_LOG | tail -14; grep -a -o "imu p10.*" $ROBOT_LOG | tail -3
