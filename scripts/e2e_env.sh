# Sourced by the end-to-end checks: which simulator instance they run on
# (V5SIM_ID, default 0; v5brain.sh reads it too) and that instance's paths.
# e2e_all.sh picks a free instance, so the checks never touch a sim that's
# already running.
export V5SIM_ID=${V5SIM_ID:-0}
export V5SIM_NO_BROWSER=1         # --competition would open the field-control page
if [ "$V5SIM_ID" = 0 ]; then
  E2E_T=/tmp
  export ROBOT_LOG=~/re/qemu-runs/robot.txt
else
  E2E_T=/tmp/v5sim$V5SIM_ID
  export ROBOT_LOG=~/re/qemu-runs/robot$V5SIM_ID.txt
fi
export QMP=$E2E_T/v5qmp.sock
export TTY_USER=$E2E_T/ttyV5user TTY_SYS=$E2E_T/ttyV5sys
export VIEW_PORT=$((8765 + V5SIM_ID))

# Events instead of fixed sleeps (scripts/e2e_wait.py). e2e_started waits for
# v5brain.sh's "started slot" line in $R/v5brain.out (or $1), then sleeps $2
# more seconds; e2e_still waits until the robot moved and then it and every
# motor stayed still for $1 s; e2e_log FILE REGEX waits for a line.
e2e_started() {
  python3 -I "$REPO/scripts/e2e_wait.py" started "${1:-$R/v5brain.out}" 60
  [ -n "${2:-}" ] && sleep "$2"
  return 0
}
e2e_still() { python3 -I "$REPO/scripts/e2e_wait.py" still "$VIEW_PORT" "${1:-2}" "${2:-60}"; }
e2e_log() { python3 -I "$REPO/scripts/e2e_wait.py" log "$1" "$2" "${3:-60}"; }
