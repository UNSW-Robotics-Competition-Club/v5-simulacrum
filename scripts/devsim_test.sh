#!/bin/bash
# Boot with port 1 on a socket, run devsim.py against it, screenshot Devices.
R=~/re/qemu-runs; P=/tmp/v5port1.sock; rm -f $P
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
bash "$V5SIM_ROOT/scripts/v5session.sh" \
  "wait:18 tap:180,95 wait:6 shot:$R/d1.ppm quit" \
  -chardev socket,id=port1,path=$P,server=on,wait=off > $R/session.txt 2>&1 &
for _ in $(seq 100); do [ -S $P ] && break; sleep 0.1; done
sleep 14
python3 -I "$V5SIM_ROOT/scripts/devsim.py" $P 8 > $R/devsim.txt 2>&1
wait
head -8 $R/devsim.txt; echo ...; grep -c "brain>" $R/devsim.txt; grep -a "Found dev\|port" $R/uart.txt | head
