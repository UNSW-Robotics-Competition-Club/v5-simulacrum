#!/bin/bash
# End to end: a PROS hot/cold program installed with --run/--link (as PROS
# uploads it: the cold package a library at 0x03800000, the hot package a
# program at 0x07800000 linked to it) starts on the brain.
#   V5SIM_HOTCOLD=<a built PROS project>  (its bin/hot.package.bin and
#   bin/cold.package.bin; PROS 4 builds them by default)
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
P=${V5SIM_HOTCOLD:-}
if [ -z "$P" ] || [ ! -f "$P/bin/hot.package.bin" ] || [ ! -f "$P/bin/cold.package.bin" ]; then
  echo "SKIP: set V5SIM_HOTCOLD to a built PROS project (bin/hot.package.bin, bin/cold.package.bin)"
  exit 0
fi
# --run takes no spaces in paths: copy the packages
D=$(mktemp -d /tmp/v5hotcold.XXXXXX)
cp "$P/bin/hot.package.bin" "$P/bin/cold.package.bin" "$D/"
: > "${CONSOLE_LOG:=$R/console${V5SIM_ID#0}.txt}"
bash "$REPO/scripts/v5brain.sh" --headless --run "$D/hot.package.bin" --link "$D/cold.package.bin" --slot 3 \
  --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 4
img=$R/qspi_programs${V5SIM_ID#0}.img
python3 -I "$REPO/scripts/mkspiffs_vex.py" --list "$img" | grep -E "^(user/slot_3|pros/)"
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
rm -rf "$D"
grep -a "Start 3rd party code\|load fail\|error" "$R/console${V5SIM_ID#0}.txt" | head -3
