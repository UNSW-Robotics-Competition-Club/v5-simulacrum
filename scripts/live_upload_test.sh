#!/bin/bash
# End to end: Run on Sim without a restart. Boots with no USB host side at
# all (no --usb, no --usbip: as on a Mac or Windows without the USB driver),
# then installs and starts programs on the running brain through
# v5usb_share.py's upload socket with v5upload.py, twice, and checks each
# one started (VEXos logs RemoteStartUserProgram; hello drives motor 1)
# while QEMU kept running (the same process throughout).
#   programs/test-programs/hello (programs/test-programs/build.sh hello)
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
PROG=$REPO/programs/test-programs/hello/hello.bin
if [ ! -f "$PROG" ]; then
  echo "SKIP: build it first: bash programs/test-programs/build.sh hello"
  exit 0
fi
CONSOLE=$R/console${V5SIM_ID#0}.txt
: > "$R/v5brain.out"
bash "$REPO/scripts/v5brain.sh" --headless \
  --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_log "$R/v5brain.out" "upload .*v5usb-host.sock" 30 >/dev/null || { echo "FAIL: no upload socket"; exit 1; }
SOCK=$(grep -ao "upload [^,]*v5usb-host.sock" "$R/v5brain.out" | tail -1 | cut -d' ' -f2)
sleep ${1:-20}                                   # VEXos boots before it takes uploads
qemu_pid() { pgrep -f "qemu-system-arm.*$QMP" | head -1; }
before=$(qemu_pid)
starts() { grep -ac "RemoteStartUserProgram" "$CONSOLE"; }
n0=$(starts)
for slot in 2 3; do
  t0=$(date +%s.%N)
  python3 -I "$REPO/scripts/v5upload.py" "$SOCK" "$PROG" --slot $slot --name hello$slot | tail -1
  printf 'slot %s: uploaded and started in %.1f s\n' $slot "$(echo "$(date +%s.%N) - $t0" | bc)"
  sleep 1
  python3 -I - "$VIEW_PORT" <<'PY'
import json, sys, urllib.request
def motors(x):
    if isinstance(x, dict):
        if x.get("kind") == "motor":
            yield x
        for v in x.values():
            yield from motors(v)
    elif isinstance(x, list):
        for v in x:
            yield from motors(v)
snap = json.load(urllib.request.urlopen(f"http://localhost:{sys.argv[1]}/devices.json", timeout=5))
m1 = next(m for m in motors(snap) if m["port"] == 1)
print(f"  motor 1: {m1['voltage_v']:+.1f} V" + ("" if abs(m1["voltage_v"]) > 1 else "  FAIL: not driven"))
PY
done
expected=2
# optional: a PROS hot/cold pair (V5SIM_HOTCOLD=<a built PROS project>),
# twice: the second upload skips the cold package the brain already has
if [ -n "${V5SIM_HOTCOLD:-}" ] && [ -f "$V5SIM_HOTCOLD/bin/hot.package.bin" ]; then
  for n in 1 2; do
    t0=$(date +%s.%N)
    python3 -I "$REPO/scripts/v5upload.py" "$SOCK" "$V5SIM_HOTCOLD/bin/hot.package.bin" \
      --link "$V5SIM_HOTCOLD/bin/cold.package.bin" --slot 4 | tail -1
    printf 'hot/cold %s: %.1f s\n' $n "$(echo "$(date +%s.%N) - $t0" | bc)"
    sleep 2
  done
  grep -ac "Start 3rd party code" "$CONSOLE" | sed 's/^/PROS starts: /'
  expected=$((expected + 2))
fi
# optional: a VEXcode Python program with VEX's VM (V5SIM_PYTHON_VM, else the
# VEXcode extension's newest)
VM=${V5SIM_PYTHON_VM:-}
if [ -z "$VM" ] && command -v cmd.exe >/dev/null; then
  g=$(wslpath -u "$(cd / && cmd.exe /c "echo %APPDATA%" 2>/dev/null | tr -d '\r')")/Code/User/globalStorage
  VM=$(ls -d "$g"/vexrobotics.vexcode/sdk/python/V5/V5_*/vexv5/vm/python_vm.bin 2>/dev/null | sort -V | tail -1)
fi
if [ -n "$VM" ] && [ -f "$VM" ]; then
  t0=$(date +%s.%N)
  python3 -I "$REPO/scripts/v5upload.py" "$SOCK" "$REPO/programs/test-programs/vexcode_python/main.py" \
    --link "$VM" --slot 5 | tail -1
  printf 'python: %.1f s\n' "$(echo "$(date +%s.%N) - $t0" | bc)"
  sleep 2
  expected=$((expected + 1))
fi
n1=$(starts)
after=$(qemu_pid)
[ "$before" = "$after" ] && [ -n "$before" ] && echo "same QEMU process: $before" || echo "FAIL: QEMU restarted ($before -> $after)"
[ $((n1 - n0)) -ge $expected ] && echo "programs started: $((n1 - n0))" || echo "FAIL: started $((n1 - n0)) programs, expected $expected"
python3 -I "$REPO/scripts/mkspiffs_vex.py" --list "$R/qspi_programs${V5SIM_ID#0}.img" 2>/dev/null | grep -E "^user/slot_[23]" || true
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
