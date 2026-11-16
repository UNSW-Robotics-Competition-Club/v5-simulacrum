#!/bin/bash
# End to end: headless brain + simulated robot (drivetrain on 1-3/4-6,
# a free motor on 8). Opens Device Info, then port 8's motor page.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; SIM=$V5SIM_ROOT/sim
ports=(); for n in 1 2 3 4 5 6 8; do rm -f /tmp/v5port$n.sock
  ports+=(-chardev "socket,id=port$n,path=/tmp/v5port$n.sock,server=on,wait=off"); done
bash "$V5SIM_ROOT/scripts/v5session.sh" \
  "${SESSION:-wait:20 tap:180,95 wait:3 shot:$R/r1.ppm ${CMDS:-} quit}" "${ports[@]}" > $R/session.txt 2>&1 &
cd $SIM && python3 -m v5sim.robot --motor 8:green --drive 1,2,3:4,5,6:blue \
  --seconds ${SECS:-30} --status 2 ${ROBOT_ARGS:-} > $R/robot.txt 2>&1
wait
tail -4 $R/robot.txt; grep -a "Found dev\|Motor" $R/uart.txt | head
