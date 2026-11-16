#!/bin/bash
# End to end: game objects (programs/test-programs/vexcode_intake/blocks.json). The VEXcode
# program drives through a line of three blocks with its intake (motor 7)
# running, turns around and ejects them; a fourth block beside the path is
# plowed aside. Expect "held [red blue red]" after collecting, then the
# three blocks back on the field in the demo goal (goals.json): score red
# 2*3 + 10 (control) = 16, blue 3.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_intake/build/vexcode_intake.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --intake 7:4 --blocks $REPO/programs/test-programs/vexcode_intake/blocks.json --goals $REPO/programs/test-programs/vexcode_intake/goals.json --view $VIEW_PORT --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
e2e_still 2 40
curl -s http://127.0.0.1:$VIEW_PORT/state.json > $R/intake_state.json
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a "v5brain\]\|tmo\|abort" $R/v5brain.out
grep -a -o "pose x=[^|]*\|blocks [0-9]* held \[[^]]*\]\|score red [0-9]* blue [0-9]*" $ROBOT_LOG | paste - - - | uniq | tail -8
python3 -I -c "import json,sys; s=json.load(open(sys.argv[1])); print('field:', [(round(x,2),round(y,2),c) for x,y,c in s['blocks']], 'held', s['held'], 'score', s['score'])" $R/intake_state.json
