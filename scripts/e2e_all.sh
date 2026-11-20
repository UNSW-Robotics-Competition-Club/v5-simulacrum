#!/bin/bash
# Run every end-to-end check in turn and print the tail of each. About 11
# minutes. They run on the first free simulator instance from 1 (V5SIM_ID
# to pick one; scripts/e2e_env.sh), so a sim you have open is never
# touched; a check that isn't instance-aware yet only runs on instance 0
# (V5SIM_ID=0).
#   wsl -d Ubuntu-24.04 -- bash <repo>/scripts/e2e_all.sh [names...]
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
. "$V5SIM_ROOT/scripts/platform.sh"
PATH=$(v5_path); export PATH
REPO=$V5SIM_ROOT
if [ -z "${V5SIM_ID:-}" ]; then
  for n in $(seq 1 9) 0; do       # instance 0 is the one people open
    if ( lock_fd 9 ) 9>"/tmp/v5sim$n.lock"; then export V5SIM_ID=$n; break; fi
  done
fi
[ -n "${V5SIM_ID:-}" ] || { echo "no free simulator instance" >&2; exit 1; }
echo "e2e checks on simulator instance $V5SIM_ID"
ALL="smoke_run ui_test button_test arm_test motor55_test motor_fw_test misc_test adi_extra_test vexcode_test imu_test driver_test comp_test radio_test adi_test intake_test opponent_test sensors_test aivision_test aitags_test vision_test usb_test vexcom_test hotcold_test python_run_test live_upload_test"
# Keep a USB byte trace of the whole run (intermittent user-port oddities)
export V5USB_TRACE=${V5USB_TRACE:-/tmp/v5usb_e2e_trace.txt}
rm -f "$V5USB_TRACE"
for t in ${*:-$ALL}; do
  echo "=== $t"
  echo "=== $t" >> "$V5USB_TRACE"
  start=$(date +%s)
  if [ "$V5SIM_ID" != 0 ] && ! grep -q e2e_env.sh "$REPO/scripts/$t.sh"; then
    echo "skipped: not instance-aware, runs on instance 0 only"
    continue
  fi
  bash "$REPO/scripts/$t.sh" 2>&1 | tail -6
  echo "--- $t: $(( $(date +%s) - start )) s"
done
