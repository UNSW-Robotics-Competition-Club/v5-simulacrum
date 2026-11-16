#!/bin/bash
# Smoke test: boot headless, start a program with --run, attach a motor sim
# on port 1, screenshot after SECS. Usage: smoke_run.sh [prog.bin] [secs]
# Out: ~/re/qemu-runs/{smoke.ppm,v5brain.out,robot.txt}
R=~/re/qemu-runs
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
PROG=${1:-$REPO/programs/test-programs/hello/hello.bin}
SECS=${2:-9}     # after the program starts
bash "$REPO/scripts/v5brain.sh" --headless --run "$PROG" \
  --robot "--motor 1:green --status 2" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" "$SECS"
python3 -I "$REPO/scripts/v5ctl.py" $QMP "shot:$R/smoke.ppm" quit
wait
grep -a "v5brain\]\|time to boot\|tmo" $R/v5brain.out
tail -2 $ROBOT_LOG | cut -c1-80
