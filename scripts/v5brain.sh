#!/bin/bash
# Emulated V5 brain (stock VEXos 1.1.5.0 on QEMU -M vex-v5).
#
#   v5brain.sh [--id N|auto] [--run prog.bin] [--slot N] [--robot "ARGS"]
#              [--headless] [--run-after SECS] [--usb] [-- extra qemu args]
#
#   --run FILE      install FILE (a V5 user program .bin, 'XVX5' header) in
#                   the brain's program storage and start it once VEXos is
#                   up: Programs > slot > Run, the same path a user takes
#   --slot N        program slot 1..8 for --run (default 1)
#   --robot "ARGS"  also start the device/physics sim, e.g.
#                   --robot "--drive 1,2,3:4,5,6:blue --motor 8:green"
#   --headless      no window; console on stdout, screenshots via QMP
#   --run-after S   seconds to wait before starting the Run taps (env
#                   V5SIM_RUN_AFTER, default 0.5). The taps wait for VEXos's
#                   Home screen and each screen they open, so a program
#                   starts about 2.5 s after launch
#   --usb           bridge the brain's USB serial ports to ttys like a real
#                   brain on a PC: /tmp/ttyV5sys (system: pros upload/terminal
#                   --port /tmp/ttyV5sys) and /tmp/ttyV5user (program stdio)
#   --usbip         instead, serve the brain's USB as a USB/IP device on
#                   localhost:3300+N (scripts/v5usb_usbip.py): on Windows,
#                   `usbip attach -r localhost -b 1-1` (usbip-win2) gives it
#                   real COM ports that VEXcode and PROS find as a V5 brain.
#                   With usbip-win2 installed this script attaches it (and
#                   detaches it on exit) by running Windows' usbip.exe
#   --id N|auto     which simulator instance (env V5SIM_ID; default 0), so
#                   several can run at once. 0 uses the paths below; N >= 1
#                   puts every socket and tty in /tmp/v5sim<N>/, the field
#                   view on port 8765+N, the log in ~/re/qemu-runs/robot<N>.txt
#                   and its own flash image. auto = the lowest free one. A
#                   busy instance refuses to start rather than take over.
#
# Window: click = touch. Console on the terminal (Ctrl-A X quits).
# QMP on /tmp/v5qmp.sock (scripts/v5ctl.py). Smart ports 1..22 on
# /tmp/v5port<N>.sock. Env PROGRAMS="name=file.bin[@slot] ..." installs
# more programs (slots 0-based there).
# Env V5SIM_ROBOT_EXTRA="ARGS" appends robot-sim options to --robot, e.g.
# "--physics mujoco" to rerun any e2e check on MuJoCo physics.
#
# From Windows:
#   wsl -d Ubuntu-24.04 -- bash <repo>/scripts/v5brain.sh --run /mnt/c/.../prog.bin
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
. "$V5SIM_ROOT/scripts/platform.sh"
# macOS: Homebrew's python3 (the helpers) even when started from an app
# with a minimal PATH (VS Code from the Dock)
[ "$V5SIM_OS" = macos ] && PATH="$(v5_path):$PATH" && export PATH
REPO=$V5SIM_ROOT
# Where things are; the VS Code extension's bundled simulator sets all three.
# V5SIM_FIRMWARE_DIR: the unpacked VEXos (scripts/fetch_firmware.sh),
# V5SIM_QEMU: qemu-system-arm with the VEX models, V5SIM_RUNS: logs and the
# brain's flash images.
export V5SIM_FIRMWARE_DIR=${V5SIM_FIRMWARE_DIR:-$REPO/extracted}
B=$V5SIM_FIRMWARE_DIR/boot
QEMU=${V5SIM_QEMU:-~/re/qemu/build/qemu-system-arm}
export V5SIM_RUNS=${V5SIM_RUNS:-~/re/qemu-runs}
RUNS=$V5SIM_RUNS
MAX_ID=9

run_file="" slot=1 robot_args="" headless=0 run_after=${V5SIM_RUN_AFTER:-0.5} usb=0 id=${V5SIM_ID:-0}
while [ $# -gt 0 ]; do
  case $1 in
    --id) id=$2; shift 2 ;;
    --run) run_file=$2; shift 2 ;;
    --slot) slot=$2; shift 2 ;;
    --robot) robot_args=$2; shift 2 ;;
    --headless) headless=1; shift ;;
    --run-after) run_after=$2; shift 2 ;;
    --usb) usb=1; shift ;;
    --usbip) usb=ip; shift ;;
    --) shift; break ;;
    -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done
if [ -n "$run_file" ]; then
  [ -f "$run_file" ] || { echo "no such file: $run_file" >&2; exit 2; }
  case $run_file in *[[:space:]]* | *[*?]*)   # PROGRAMS is a space-separated list
    echo "--run: the path can't contain spaces or wildcards: $run_file" >&2; exit 2 ;;
  esac
  [ "$slot" -ge 1 ] && [ "$slot" -le 8 ] || { echo "--slot must be 1..8" >&2; exit 2; }
  name=$(basename "$run_file" .bin | tr -c 'A-Za-z0-9_\n' '_' | cut -c1-20)
  PROGRAMS="${PROGRAMS:-} $name=$run_file@$((slot - 1))"
fi
export PROGRAMS="${PROGRAMS:-}"

# Claim an instance: hold /tmp/v5sim<N>.lock for as long as this script runs.
mkdir -p "$RUNS"
claim() {
  local dir=/tmp
  [ "$1" = 0 ] || dir=/tmp/v5sim$1
  exec 9>"/tmp/v5sim$1.lock"
  lock_fd 9 || return 1
  # a brain started without the lock (an older copy of this script) still
  # listens on its sockets. (Not a connect test: QMP takes one client at
  # a time, so a busy brain doesn't answer.)
  if unix_listening "$dir/v5qmp.sock" || unix_listening "$dir/v5port1.sock"; then
    exec 9>&-
    return 1
  fi
}
if [ "$id" = auto ]; then
  # V5SIM_AUTO_FIRST=1: automation leaves instance 0 to people
  for n in $(seq "${V5SIM_AUTO_FIRST:-0}" $MAX_ID); do claim "$n" && { id=$n; break; }; done
  [ "$id" != auto ] || { echo "every simulator instance 0..$MAX_ID is busy" >&2; exit 1; }
else
  case $id in ''|*[!0-9]*) echo "--id must be 0..$MAX_ID or auto" >&2; exit 2 ;; esac
  [ "$id" -le $MAX_ID ] || { echo "--id must be 0..$MAX_ID or auto" >&2; exit 2; }
  claim "$id" || { echo "simulator instance $id is already running; use --id auto" >&2; exit 1; }
fi
if [ "$id" = 0 ]; then
  T=/tmp ROBOT_LOG=$RUNS/robot.txt CONSOLE_LOG=$RUNS/console.txt
else
  T=/tmp/v5sim$id ROBOT_LOG=$RUNS/robot$id.txt CONSOLE_LOG=$RUNS/console$id.txt
  mkdir -p "$T"
  # its own flash: uploads persist per instance, like separate brains
  if [ ! -f "$RUNS/qspi$id.img" ]; then
    cp "$(PROGRAMS='' bash "$REPO/scripts/qspi_with_programs.sh")" "$RUNS/qspi$id.img"
  fi
  export QSPI=$RUNS/qspi$id.img QSPI_OUT=$RUNS/qspi_programs$id.img
fi
QMP=$T/v5qmp.sock
VIEW=$((8765 + id))
# Window watchdog (Windows `scripts/v5watch.cmd`): record our instance id under
# the window's token so, if that window is closed or killed (which does NOT run
# our traps), the watchdog can stop us gracefully and detach USB/IP cleanly.
WIN_TOKEN=${V5SIM_WINDOW_TOKEN:-}
[ -n "$WIN_TOKEN" ] && echo "$id" > "$RUNS/win-$WIN_TOKEN"
QSPI=$(bash "$REPO/scripts/qspi_with_programs.sh")
USERBIN=${USERBIN:-$B/p5_system_0_ps_03800000.bin}

ports=()
for n in $(seq 1 22); do   # 22 = the built-in 3-wire ports (ADI)
  rm -f $T/v5port$n.sock
  ports+=(-chardev "socket,id=port$n,path=$T/v5port$n.sock,server=on,wait=off")
done
# USB0's two CDC ports (system, user): unix sockets, ttys with --usb
for u in sys user; do
  rm -f $T/v5usb-$u.sock
  ports+=(-chardev "socket,id=usb$u,path=$T/v5usb-$u.sock,server=on,wait=off")
done
rm -f "$QMP" "$T/v5qmp-robot.sock" "$T/v5qmp-screen.sock"

# A window shows the screen inside a picture of the brain, with its front
# button clickable (V5SIM_SKIN=file.ppm overrides, also headless; "" = none).
skin_file="$REPO/sim/skins/v5-brain.ppm"
[ -f "$skin_file" ] || python3 "$REPO/sim/skins/make_brain_skin.py" "$skin_file" >/dev/null
skin=${V5SIM_SKIN-$skin_file}
if [ "$headless" = 1 ]; then
  skin=${V5SIM_SKIN-}
  display=(-display none)
else
  display=(-display sdl,show-cursor=on)
fi

# The Windows usbip port holding this instance's brain ("" if none), found by
# its server (usbip://localhost:<port>/), not a saved number another instance
# may have taken since.
usbip_vport() {
  [ -x "${USBIP_EXE:-}" ] || return 0
  { "$USBIP_EXE" port 2>/dev/null || true; } | tr -d '\r' |
    awk -v want="usbip://localhost:$USBIP_PORT/" '/^Port [0-9]+:/ { p = $2 + 0 } index($0, want) { print p; exit }' || true
}
helpers=()
cleanup() {
  # set -e is on here too: nothing below may end the trap before the helpers
  # are killed (a leftover USB/IP server would hold the port for the next run)
  set +e
  [ -n "${WIN_TOKEN:-}" ] && rm -f "$RUNS/win-$WIN_TOKEN"
  # Take the brain off Windows' USB first, while its USB/IP server still
  # answers: tearing a port down after the server is gone can bugcheck
  # usbip-win2, and the driver starts re-attaching it.
  if [ "$usb" = ip ]; then
    local v
    v=$(usbip_vport)
    [ -n "$v" ] && "$USBIP_EXE" detach -p "$v" > /dev/null 2>&1 || true
    rm -f "${T:-/tmp}/usbip-vport"
  fi
  for p in "${helpers[@]}"; do kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT
# Closing the brain window (or `kill`) sends TERM/HUP; exit so the EXIT trap
# runs and the USB/IP brain is detached from Windows instead of left stale.
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

wait_for_sock() { for _ in $(seq 100); do [ -S "$1" ] && return 0; sleep 0.1; done; return 1; }

# The robot sim's Python: V5SIM_PYTHON, else the ~/venvs/sim venv (it has
# MuJoCo for --physics mujoco), else the system python3.
sim_python=${V5SIM_PYTHON:-python3}
[ -z "${V5SIM_PYTHON:-}" ] && [ -x ~/venvs/sim/bin/python ] && sim_python=~/venvs/sim/bin/python
[ -n "$robot_args" ] && robot_args="$robot_args ${V5SIM_ROBOT_EXTRA:-}"
# the robot sim's own QMP socket: the field view presses the brain's button
# and the battery model reports to it, without waiting on a script's QMP
ROBOT_QMP=$T/v5qmp-robot.sock
[ -n "$robot_args" ] && robot_args="$robot_args --qmp $ROBOT_QMP"
# and a third for the field view's /screen page (frames + touch), which holds
# its connection open: the brain's screen in a browser or VS Code
[ -n "$robot_args" ] && robot_args="$robot_args --screen-qmp $T/v5qmp-screen.sock"
# the instance's field view (the last --view wins over a preset's); another
# instance's robot also gets its own sockets
[ -n "$robot_args" ] && robot_args="$robot_args --view $VIEW"
if [ -n "$robot_args" ] && [ "$id" != 0 ]; then
  robot_args="$robot_args --sock $T/v5port{port}.sock"
fi
if [ -n "$robot_args" ]; then
  (wait_for_sock $T/v5port1.sock
   # its warnings and errors (bad options, two devices on a port) also
   # show on this console, not only in the log
   cd "$REPO/sim" && exec env PATH=/usr/bin:/bin "$sim_python" -m v5sim.robot $robot_args \
     > "$ROBOT_LOG" 2> >(tee -a "$ROBOT_LOG" | while IFS= read -r l; do printf '[robot] %s\n' "$l"; done >&2)) &
  helpers+=($!)
fi
# usbipd-win, if installed, owns the standard USB/IP port 3240 on Windows, so
# the Windows usbip.exe client reaches it instead of our WSL server (and fails
# with "Device not found by bus id"). Base the port away from 3240; override
# with V5SIM_USBIP_BASE if 3300+ is taken too.
USBIP_PORT=$(( ${V5SIM_USBIP_BASE:-3300} + id ))
USBIP_EXE="/mnt/c/Program Files/USBip/usbip.exe"
usbip_vport=""                     # the Windows port it attached to, for detach
if [ "$usb" = ip ]; then
  (wait_for_sock $T/v5usb-sys.sock &&
   exec python3 -I "$REPO/scripts/v5usb_usbip.py" $T/v5usb-sys.sock $T/v5usb-user.sock \
     --port $USBIP_PORT --serial V5SIM$id >&2) &
  helpers+=($!)
  if [ -x "$USBIP_EXE" ]; then
    rm -f "$T/usbip-vport"
    (for _ in $(seq 100); do
       tcp_listening "$USBIP_PORT" && break; sleep 0.1
     done
     # --once: attach a single time. Without it usbip-win2 keeps retrying and
     # stacks a fresh COM-port pair on every reconnect, so stale dead brains
     # pile up on Windows' USB (the "no/duplicate COM ports" confusion).
     out=$("$USBIP_EXE" -t $USBIP_PORT attach -r localhost -b 1-1 --once 2>&1)
     echo "[v5brain] Windows: $(echo "$out" | tr -d '\r')" >&2
     echo "$out" | grep -o "port [0-9]*" | grep -o "[0-9]*" > "$T/usbip-vport") &
    helpers+=($!)
  fi
fi
if [ "$usb" = 1 ]; then
  (wait_for_sock $T/v5usb-sys.sock &&
   exec python3 -I "$REPO/scripts/v5usb_pty.py" \
     $T/v5usb-sys.sock=$T/ttyV5sys $T/v5usb-user.sock=$T/ttyV5user >&2) &
  helpers+=($!)
fi
if [ -n "$run_file" ]; then
  (wait_for_sock "$QMP" && sleep "$run_after" &&
   python3 -I "$REPO/scripts/v5ctl.py" "$QMP" "run:$slot" &&
   echo "[v5brain] started slot $slot ($name)" >&2) &
  helpers+=($!)
fi

info="instance $id: QMP $QMP, log $CONSOLE_LOG"
[ -n "$robot_args" ] && info="$info, field view http://localhost:$VIEW/, brain screen http://localhost:$VIEW/screen"
[ "$usb" = 1 ] && info="$info, USB $T/ttyV5sys $T/ttyV5user"
[ "$usb" = ip ] && info="$info, USB/IP localhost:$USBIP_PORT busid 1-1"
echo "[v5brain] $info" >&2
# The brain's serial console goes to stdio (this window, if any) and is also
# logged to $CONSOLE_LOG so `v5sim log` can follow it live without a window.
# A hung VEXos can print in a loop (megabytes a second), so the log is capped:
# past V5SIM_LOG_MAX bytes it keeps its last 256 KB. QEMU appends to it
# (logappend=on, emptied here first): trimming a file QEMU wrote at its own
# offset would only leave a sparse file that keeps growing.
: > "$CONSOLE_LOG"
LOG_MAX=${V5SIM_LOG_MAX:-20000000}
(while sleep 2; do
   size=$(stat -c %s "$CONSOLE_LOG" 2>/dev/null || echo 0)
   if [ "$size" -gt "$LOG_MAX" ]; then
     keep=$(tail -c 262144 "$CONSOLE_LOG")
     printf '[v5brain] console log passed %s bytes: earlier lines dropped\n%s' "$LOG_MAX" "$keep" > "$CONSOLE_LOG"
   fi
 done) > /dev/null 2>&1 &    # not on our stdout: its sleep would hold the caller's pipe open
helpers+=($!)
"$QEMU" -M "vex-v5,launch-ms=${LAUNCH_MS:-0}${skin:+,skin=$skin}" -smp 2 -m 512M "${display[@]}" \
  -chardev "stdio,id=cons,mux=on,logfile=$CONSOLE_LOG,logappend=on" \
  -serial chardev:cons -mon chardev=cons,mode=readline \
  -qmp unix:$QMP,server,nowait \
  -qmp unix:$T/v5qmp-robot.sock,server,nowait \
  -qmp unix:$T/v5qmp-screen.sock,server,nowait \
  -device loader,file=$B/p0_FSBL_ps_00000000.bin,addr=0x0,force-raw=on \
  -device loader,file=$B/p2_MainLoop_ps_01000000.bin,addr=0x01000000,force-raw=on \
  -device loader,file=$B/p3_system_0_ps_03400000.bin,addr=0x03400000,force-raw=on \
  -device loader,file=$B/p4_system_0_ps_037fc000.bin,addr=0x037fc000,force-raw=on \
  -device loader,file="$USERBIN",addr=0x03800000,force-raw=on \
  -drive if=mtd,index=8,format=raw,file="$QSPI" \
  -device loader,addr=0x01000000,cpu-num=0 \
  "${ports[@]}" "$@" && rc=0 || rc=$?
# Say why the brain went away: a closed window, `v5sim stop` and Ctrl-A X all
# exit 0 silently, which otherwise reads like a crash in the robot sim's log.
if [ "$rc" = 0 ]; then
  echo "[v5brain] instance $id: QEMU exited normally (window closed, Ctrl-A X or QMP quit)" >&2
elif [ "$rc" -gt 128 ]; then
  echo "[v5brain] instance $id: QEMU killed by signal $((rc - 128))" >&2
else
  echo "[v5brain] instance $id: QEMU failed, exit code $rc (see $CONSOLE_LOG)" >&2
fi
exit "$rc"
