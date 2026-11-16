#!/bin/bash
# End to end: VEXcode's own uploader (vexcom, the Linux x64 build shipped in
# the VEXcode VS Code extension) over --usb:
#   1. vexcom --json                    brain info
#   2. vexcom --python                  install VEX's MicroPython VM (once)
#   3. vexcom --write main.py --run     a VEXcode Python program drives the
#                                       robot; its print() output comes back
#                                       on the user port
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
EXT=$(wslpath "$(cmd.exe /c "echo %USERPROFILE%" 2>/dev/null | tr -d '\r')")/.vscode/extensions
V=~/re/tools/vexcom
if [ ! -x "$V" ]; then
  src=$(ls -d $EXT/vexrobotics.vexcode-*/resources/tools/vexcom/linux-x64/vexcom | sort | tail -1)
  mkdir -p ~/re/tools && cp "$src" "$V" && chmod +x "$V"
fi
bash "$REPO/scripts/v5brain.sh" --headless --usb \
  --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started
timeout 30 "$V" --json $TTY_SYS 2>&1 | grep -a '"vexos"'
if ! timeout 30 "$V" --json $TTY_SYS 2>&1 | grep -q '"binfile": "python_vm.bin"'; then
  # the port goes first: with --python vexcom otherwise ignores it
  timeout 120 "$V" $TTY_SYS --python > /dev/null 2>&1
fi
python3 -I - <<'PY' > $R/vexcom_user.txt &
import os, time, tty
fd = os.open(os.environ["TTY_USER"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
out, t0 = b"", time.time()
while time.time() - t0 < 12:
    try:
        out += os.read(fd, 4096)
    except BlockingIOError:
        time.sleep(0.05)
print(out.decode(errors="replace"))
PY
timeout 90 "$V" --slot 7 --write "$REPO/programs/test-programs/vexcode_python/main.py" --run $TTY_SYS 2>&1 | tail -1
wait %2 2>/dev/null
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
grep -a "MicroPython\|python\|forward\|turn" $R/vexcom_user.txt
grep -a "pose" $ROBOT_LOG | tail -1 | cut -c1-40
