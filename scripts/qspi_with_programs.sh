#!/bin/bash
# Print the path of a QSPI image to boot: the base image, or with PROGRAMS
# set ("name=prog.bin[@slot][,autorun] ..."), a copy with those programs in
# its SPIFFS (slots are 0-based; the UI shows slot+1).
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
REPO=$V5SIM_ROOT
BASE=${QSPI:-$HOME/re/qemu-runs/qspi.img}
[ -f "$BASE" ] || python3 -I "$REPO/scripts/mkqspi.py" "$REPO/extracted/VEXOS_V5_1_1_5_0" "$BASE" >&2
# A real brain ships with a formatted (empty) SPIFFS; VEXos mounts without
# formatting on most paths (USB uploads among them), so format a blank one
# once. Uploads then persist in $BASE like on a real brain.
if ! python3 -I "$REPO/scripts/mkspiffs_vex.py" --formatted "$BASE"; then
  python3 -I "$REPO/scripts/mkspiffs_vex.py" "$BASE" >&2
fi
if [ -z "${PROGRAMS:-}" ]; then
  echo "$BASE"
  exit 0
fi
OUT=${QSPI_OUT:-$HOME/re/qemu-runs/qspi_programs.img}
cp "$BASE" "$OUT"
# shellcheck disable=SC2086
python3 -I "$REPO/scripts/mkspiffs_vex.py" "$OUT" $PROGRAMS >&2
echo "$OUT"
