#!/bin/bash
# End to end: VEXcode's own uploader (vexcom, the Linux x64 build the
# VEXcode VS Code extension downloads with its tools) over --usb:
#   1. vexcom --json                    brain info
#   2. vexcom --python                  install VEX's MicroPython VM (once)
#   3. vexcom --write main.py --run     a VEXcode Python program drives the
#                                       robot; its print() output comes back
#                                       on the user port
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
# A Windows folder (%NAME%) as a WSL path. e2e_all.sh's PATH has no cmd.exe.
win_dir() {
  local cmd; cmd=$(command -v cmd.exe || echo /mnt/c/Windows/System32/cmd.exe)
  [ -x "$cmd" ] && wslpath -u "$(cd / && "$cmd" /c "echo %$1%" 2>/dev/null | tr -d '\r')"
}
V=~/re/tools/vexcom
if [ ! -x "$V" ]; then
  # VEXcode 0.8+ keeps its tools in globalStorage, older versions in the extension
  src=""
  for g in "$(win_dir APPDATA)/Code/User/globalStorage/vexrobotics.vexcode/tools/vexcom/*/linux-x64/vexcom" \
           "$(win_dir USERPROFILE)/.vscode/extensions/vexrobotics.vexcode-*/resources/tools/vexcom/linux-x64/vexcom"; do
    src=$(ls -d $g 2>/dev/null | sort -V | tail -1)
    [ -n "$src" ] && break
  done
  if [ -z "$src" ]; then
    echo "SKIP: no vexcom. Install the VEXcode VS Code extension and build a project once (it downloads vexcom)."
    exit 0
  fi
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
