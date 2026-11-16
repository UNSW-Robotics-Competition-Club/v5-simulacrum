#!/bin/bash
# Manage running V5 brain simulator instances (see scripts/v5brain.sh).
#
#   v5fleet.sh list                 one line per instance 0..9: up/down, ports
#   v5fleet.sh status [id]          detail for one instance (default 0)
#   v5fleet.sh stop <id|all>        graceful QMP quit -> clean USB/IP detach
#   v5fleet.sh attach <id>          (re)attach the brain to Windows USB
#   v5fleet.sh detach <id|all>      take the brain off Windows USB, leave it up
#
# Instances self-register: v5brain.sh holds /tmp/v5sim<N>.lock and listens on
# a QMP socket while it runs. "up" here means that QMP socket is listening.
# USB/IP server port is V5SIM_USBIP_BASE (default 3300) + id; field view is
# 8765 + id. This script is normally reached through `v5sim <verb>` on Windows.
set -uo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MAX_ID=9
BASE=${V5SIM_USBIP_BASE:-3300}
RUNS=~/re/qemu-runs
USBIP_EXE="/mnt/c/Program Files/USBip/usbip.exe"

qmp_path() { [ "$1" = 0 ] && echo /tmp/v5qmp.sock || echo "/tmp/v5sim$1/v5qmp.sock"; }
is_up() { ss -xl 2>/dev/null | grep -q " $(qmp_path "$1") "; }
# A USB/IP instance runs a TCP server on BASE+id; a pty instance does not.
usbip_server_up() { ss -ltn 2>/dev/null | grep -q "127.0.0.1:$(( BASE + $1 )) "; }
valid_id() { case $1 in ''|*[!0-9]*) return 1;; esac; [ "$1" -ge 0 ] && [ "$1" -le "$MAX_ID" ]; }

# "<id> <vport>" for each attached instance, from one `usbip.exe port` run.
# A "Port NN:" block carries a "usbip://localhost:<srvport>/1-1" line.
attached() {
  local port="" line srv
  [ -x "$USBIP_EXE" ] || return 0
  while IFS= read -r line; do
    case $line in
      *Port\ [0-9]*) port=$(echo "$line" | grep -oE 'Port [0-9]+' | grep -oE '[0-9]+') ;;
      *usbip://localhost:*/*)
        srv=${line##*usbip://localhost:}; srv=${srv%%/*}
        case $srv in ''|*[!0-9]*) continue ;; esac
        valid_id "$(( srv - BASE ))" && echo "$(( srv - BASE )) $port" ;;
    esac
  done < <("$USBIP_EXE" port 2>/dev/null | tr -d '\r')
}

# Windows vport attached for a given instance; "" if none.
vport_for() { attached | awk -v id="$1" '$1 == id { print $2; exit }'; }

cmd_list() {
  printf '%-4s %-5s %-9s %-11s %s\n' ID STATE "VIEW" "USB/IP" "WINDOWS USB"
  local n up usb
  for n in $(seq 0 "$MAX_ID"); do
    if is_up "$n"; then
      up=up
      if usbip_server_up "$n"; then
        usb=$(vport_for "$n"); usb=${usb:+port $usb}; usb=${usb:-"not attached"}
      else
        usb="pty"
      fi
      printf '%-4s %-5s %-9s %-11s %s\n' "$n" "$up" "$((8765+n))" "$((BASE+n))" "$usb"
    fi
  done
  is_any_up || echo "(no instances running)"
}
is_any_up() { local n; for n in $(seq 0 "$MAX_ID"); do is_up "$n" && return 0; done; return 1; }

cmd_status() {
  local id=${1:-0}
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID)" >&2; return 2; }
  if is_up "$id"; then
    echo "instance $id: UP"
    echo "  field view : http://localhost:$((8765+id))/"
    if usbip_server_up "$id"; then
      local v=$(vport_for "$id") winusb
      if [ -n "$v" ]; then winusb="attached (usbip port $v)"; else winusb="not attached (run: v5sim attach $id)"; fi
      echo "  USB/IP     : localhost:$((BASE+id)) busid 1-1"
      echo "  Windows USB: $winusb"
    else
      local tty=/tmp/ttyV5sys; [ "$id" = 0 ] || tty=/tmp/v5sim$id/ttyV5sys
      echo "  USB (pty)  : $tty (system), ${tty%sys}user (program)"
    fi
    echo "  QMP        : $(qmp_path "$id")"
  else
    echo "instance $id: DOWN"
  fi
}

cmd_stop() {
  local id=$1
  if [ "$id" = all ]; then
    local n any=0
    for n in $(seq 0 "$MAX_ID"); do is_up "$n" && { cmd_stop "$n"; any=1; }; done
    [ "$any" = 1 ] || echo "nothing running"
    return 0
  fi
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID or all)" >&2; return 2; }
  if ! is_up "$id"; then echo "instance $id not running"; return 0; fi
  echo "stopping instance $id ..."
  # Graceful: QMP quit ends QEMU, which lets v5brain.sh run its EXIT trap and
  # detach the USB/IP brain from Windows. Fall back to a direct detach.
  if python3 "$REPO/scripts/v5ctl.py" "$(qmp_path "$id")" quit 2>/dev/null; then
    echo "  QEMU quit sent."
    local i
    for i in $(seq 1 24); do is_up "$id" || break; sleep 0.5; done
    if is_up "$id"; then echo "  (still shutting down)"; else echo "  instance $id down."; fi
  else
    echo "  QMP busy; detaching USB/IP directly." >&2
    cmd_detach "$id"
  fi
}

cmd_attach() {
  local id=$1
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID)" >&2; return 2; }
  is_up "$id" || { echo "instance $id is not running (start it first)" >&2; return 1; }
  [ -x "$USBIP_EXE" ] || { echo "usbip.exe not found ($USBIP_EXE)" >&2; return 1; }
  local existing=$(vport_for "$id")
  [ -n "$existing" ] && { echo "instance $id already attached (usbip port $existing)"; return 0; }
  # One USB/IP brain at a time: a second attach can bugcheck usbip-win2
  # (0.9.7.8), the same guard V5Sim.cmd applies at start.
  local n v
  read -r n v < <(attached | awk -v id="$id" '$1 != id { print; exit }')
  [ -n "$n" ] && { echo "instance $n is attached (usbip port $v); detach it first: v5sim detach $n" >&2; return 1; }
  local out=$("$USBIP_EXE" -t "$((BASE+id))" attach -r localhost -b 1-1 --once 2>&1 | tr -d '\r')
  echo "  $out"
}

cmd_detach() {
  local id=$1
  [ -x "$USBIP_EXE" ] || { echo "usbip.exe not found" >&2; return 1; }
  # Only ever single, targeted detaches. `detach --stop-all` and tearing a
  # vport down while its USB/IP server is gone can bugcheck the usbip-win2
  # driver (IRQL_NOT_LESS_OR_EQUAL), so we never use them.
  if [ "$id" = all ]; then
    local n v any=0
    for n in $(seq 0 "$MAX_ID"); do
      v=$(vport_for "$n"); [ -n "$v" ] || continue
      echo "detach instance $n (usbip port $v)"; "$USBIP_EXE" detach -p "$v" 2>&1 | tr -d '\r'; any=1
    done
    [ "$any" = 1 ] || echo "nothing attached"
    return 0
  fi
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID or all)" >&2; return 2; }
  local v=$(vport_for "$id")
  [ -n "$v" ] || { echo "instance $id not attached"; return 0; }
  "$USBIP_EXE" detach -p "$v" 2>&1 | tr -d '\r'
}

console_path() { [ "$1" = 0 ] && echo "$RUNS/console.txt" || echo "$RUNS/console$1.txt"; }
qspi_path()    { [ "$1" = 0 ] && echo "$RUNS/qspi.img"    || echo "$RUNS/qspi$1.img"; }
qspiprog_path(){ [ "$1" = 0 ] && echo "$RUNS/qspi_programs.img" || echo "$RUNS/qspi_programs$1.img"; }

# Empty the program SPIFFS in the instance's flash: removes all 8 program slots
# but keeps the rest of the flash (VEXos, settings, calibration).
cmd_clear() {
  local id=${1:-0}
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID)" >&2; return 2; }
  is_up "$id" && { echo "instance $id is running; stop it first (v5sim stop $id)" >&2; return 1; }
  local img; img=$(qspi_path "$id")
  [ -f "$img" ] || { echo "instance $id has no flash yet; already clear"; return 0; }
  python3 -I "$REPO/scripts/mkspiffs_vex.py" "$img" >/dev/null || { echo "clear failed" >&2; return 1; }
  rm -f "$(qspiprog_path "$id")"   # transient run copy; rebuilt on next start
  echo "instance $id: all 8 program slots cleared (settings kept)"
}

# Factory reset: delete the instance's flash image so it is rebuilt fresh from
# firmware (id 0) or re-seeded from the id-0 base (id N) on the next start.
cmd_reset() {
  local id=${1:-0}
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID)" >&2; return 2; }
  is_up "$id" && { echo "instance $id is running; stop it first (v5sim stop $id)" >&2; return 1; }
  local img; img=$(qspi_path "$id")
  [ -f "$img" ] || { echo "instance $id is already factory-fresh (no flash image)"; return 0; }
  rm -f "$img" "$(qspiprog_path "$id")"
  echo "instance $id: factory reset (flash wiped; rebuilt fresh on next start)"
}

# Follow an instance's serial console live (Ctrl-C stops following, not the sim).
cmd_log() {
  local id=${1:-0}
  valid_id "$id" || { echo "bad id: $id (0..$MAX_ID)" >&2; return 2; }
  local f; f=$(console_path "$id")
  [ -f "$f" ] || { echo "no log yet for instance $id ($f)" >&2; return 1; }
  if is_up "$id"; then
    echo "== instance $id live log  ($f)  --  Ctrl-C stops following, not the sim =="
    exec tail -n 400 -f "$f"
  else
    echo "== instance $id is DOWN; last log ($f) ==" >&2
    tail -n 400 "$f"
  fi
}

# Wait (briefly) for a just-started instance to record its id under $tok, then
# print it. Used by `v5sim start` to report which instance it opened.
cmd_waitid() {
  local tok=$1 i id
  for i in $(seq 1 80); do
    if [ -s "$RUNS/win-$tok" ]; then
      id=$(cat "$RUNS/win-$tok" 2>/dev/null)
      case $id in ''|*[!0-9]*) : ;; *) echo "$id"; return 0 ;; esac
    fi
    sleep 0.25
  done
  return 1
}

verb=${1:-list}; shift || true
case $verb in
  list|ls)        cmd_list ;;
  status|info)    cmd_status "${1:-0}" ;;
  log|logs|tail)  cmd_log "${1:-0}" ;;
  clear)          cmd_clear "${1:-0}" ;;
  reset)          cmd_reset "${1:-0}" ;;
  stop|kill)      [ $# -ge 1 ] || { echo "usage: v5sim stop <id|all>" >&2; exit 2; }; cmd_stop "$1" ;;
  attach)         [ $# -ge 1 ] || { echo "usage: v5sim attach <id>" >&2; exit 2; }; cmd_attach "$1" ;;
  detach)         [ $# -ge 1 ] || { echo "usage: v5sim detach <id|all>" >&2; exit 2; }; cmd_detach "$1" ;;
  waitid)         [ $# -ge 1 ] || { echo "usage: v5fleet.sh waitid <token>" >&2; exit 2; }; cmd_waitid "$1" ;;
  *) echo "unknown verb: $verb (list|status|log|clear|reset|stop|attach|detach)" >&2; exit 2 ;;
esac
