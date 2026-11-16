#!/bin/bash
# End to end: VEXcode driver control (programs/test-programs/vexcode_driver) on a simulated
# robot with a tethered controller, IMU, tracking wheel and distance sensor.
# The script posts stick input like a driver: forward 2 s, turn 1 s, stop.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
IN=http://127.0.0.1:$VIEW_PORT/input
post() { curl -s -X POST -H "Content-Type: application/json" --data "$1" "$IN"; }
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_driver/build/vexcode_driver.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --imu 10 --controller 11 --rotation 12:track --distance 13 --view $VIEW_PORT --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 7
post '{"axes": {"Axis3": 127}, "buttons": ["A"]}'
sleep 2
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/drv1.ppm"
post '{"axes": {"Axis1": 76}}'
sleep 1
post '{"axes": {}}'
sleep 2
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/drv2.ppm" quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a "pose" $ROBOT_LOG | tail -8 | cut -c1-70
grep -a -o "controller p11.*" $ROBOT_LOG | tail -1
grep -a -o "distance p13[^|]*\|rotation p12[^|]*" $ROBOT_LOG | tail -2
