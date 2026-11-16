#!/bin/bash
# End to end: MuJoCo field elements and an opponent. The intake program
# (programs/test-programs/vexcode_intake) drives forward along +x from the field centre;
# a braked opponent stands across its path at sim x = 0.75 m (RW 0, 29.5 in)
# and a post (opponent_obstacles.json) beside it. Expect the robot to stop
# at the opponent's face (x <= 0.75 - 0.457 = 0.29 m, plus a ram's shove)
# and the opponent to hold within a few cm; the field view lists both.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
PY=${V5SIM_PYTHON:-$HOME/venvs/sim/bin/python}
bash "$REPO/scripts/v5brain.sh" --headless \
  --run "$REPO/programs/test-programs/vexcode_intake/build/vexcode_intake.bin" \
  --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --physics mujoco --opponent 0:29.5:0:braked --obstacles $REPO/scripts/opponent_obstacles.json --view $VIEW_PORT --status 1" \
  > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
e2e_still 2 40
curl -s http://127.0.0.1:$VIEW_PORT/state.json > $R/opponent_state.json
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a "v5brain\]\|tmo\|abort\|error" $R/v5brain.out
"$PY" -I - "$ROBOT_LOG" "$R/opponent_state.json" <<'PY'
import json, re, sys
xs = [float(m) for m in re.findall(r"pose x=([-+][0-9.]+)", open(sys.argv[1], errors="replace").read())]
s = json.load(open(sys.argv[2]))
(ox, oy, oh), = s["opponents"]
print(f"robot max x {max(xs):+.3f} m, opponent at x {ox:+.3f} y {oy:+.3f}, "
      f"{len(s['obstacles'])} obstacle(s)")
ok = 0.2 < max(xs) < 0.29 + 0.08 and abs(ox - 0.75) < 0.08 and len(s["obstacles"]) == 1
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
PY
