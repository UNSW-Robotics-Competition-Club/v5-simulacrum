#!/bin/bash
# End to end: a VEXcode Python program installed with --run main.py --link
# python_vm.bin (as vexcom uploads it: VEX's MicroPython VM a library at
# 0x03800000, the source at 0x07000000 linked to it) runs and drives.
# The VM is VEX's: the VEXcode VS Code extension keeps it in its SDK
# (globalStorage/vexrobotics.vexcode/sdk/python/V5/V5_*/vexv5/vm/), or set
# V5SIM_PYTHON_VM.
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
R=~/re/qemu-runs; REPO=$V5SIM_ROOT
. "$REPO/scripts/e2e_env.sh"
# A Windows folder (%NAME%) as a WSL path. e2e_all.sh's PATH has no cmd.exe.
win_dir() {
  local cmd; cmd=$(command -v cmd.exe || echo /mnt/c/Windows/System32/cmd.exe)
  [ -x "$cmd" ] && wslpath -u "$(cd / && "$cmd" /c "echo %$1%" 2>/dev/null | tr -d '\r')"
}
VM=${V5SIM_PYTHON_VM:-}
if [ -z "$VM" ]; then
  for g in "$(win_dir APPDATA 2>/dev/null)/Code/User/globalStorage" \
           "$HOME/Library/Application Support/Code/User/globalStorage" \
           "$HOME/.config/Code/User/globalStorage"; do
    VM=$(ls -d "$g"/vexrobotics.vexcode/sdk/python/V5/V5_*/vexv5/vm/python_vm.bin 2>/dev/null | sort -V | tail -1)
    [ -n "$VM" ] && break
  done
fi
if [ -z "$VM" ] || [ ! -f "$VM" ]; then
  echo "SKIP: no python_vm.bin. Install the VEXcode VS Code extension and open a V5 Python project once (it downloads the SDK)."
  exit 0
fi
# --run takes no spaces in paths: copy them
D=$(mktemp -d /tmp/v5pyrun.XXXXXX)
cp "$VM" "$D/python_vm.bin"
cp "$REPO/programs/test-programs/vexcode_python/main.py" "$D/main.py"
bash "$REPO/scripts/v5brain.sh" --headless --run "$D/main.py" --link "$D/python_vm.bin" --slot 4 \
  --robot "--drive 1,2,3:4,5,6:blue --status 1" > $R/v5brain.out 2>&1 < /dev/null &
e2e_started "" 6
python3 -I "$REPO/scripts/mkspiffs_vex.py" --list "$R/qspi_programs${V5SIM_ID#0}.img" | grep -E "^(user/main|vxvm/)"
grep -a "pose" $ROBOT_LOG | tail -1 | cut -c1-40
python3 -I "$REPO/scripts/v5ctl.py" $QMP quit
wait
rm -rf "$D"
grep -a "run python code\|Traceback\|Error" "$R/console${V5SIM_ID#0}.txt" | head -3
