#!/bin/bash
# Boot the brain headless and run a v5ctl.py script against it.
# Usage: v5session.sh "<v5ctl cmds>" [extra qemu args]
# e.g.   v5session.sh "wait:15 shot:/tmp/a.ppm tap:400,252 wait:2 shot:/tmp/b.ppm quit"
CMDS=$1; shift || true
SOCK=/tmp/v5qmp.sock; rm -f $SOCK
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
bash "$V5SIM_ROOT/scripts/qemu_run.sh" 300 -qmp unix:$SOCK,server,nowait "$@" > ~/re/qemu-runs/uart.txt 2>&1 &
for _ in $(seq 50); do [ -S $SOCK ] && break; sleep 0.1; done
# shellcheck disable=SC2086
python3 -I "$V5SIM_ROOT/scripts/v5ctl.py" $SOCK $CMDS
wait
