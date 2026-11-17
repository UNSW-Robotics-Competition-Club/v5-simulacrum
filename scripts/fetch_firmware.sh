#!/bin/bash
# Download VEXos 1.1.5.0 from VEX's public server and unpack it into
# extracted/ for the simulator. The repo never ships VEX's firmware.
#
#   wsl -d Ubuntu-24.04 -- bash scripts/fetch_firmware.sh
#
# Source: the URL the VEXcode VS Code extension downloads VEXos from. If the
# download fails, it falls back to the copy the extension keeps in
# %APPDATA%\Code\User\globalStorage\vexrobotics.vexcode\vexos\V5\ (on macOS
# ~/Library/Application Support/Code/User/globalStorage/...), or to a
# file you name with V5SIM_VEXOS_FILE=/path/to/VEXOS_V5_1_1_5_0.vexos.
# Whatever the source, the file must match the pinned SHA-256.
# Idempotent: does nothing when extracted/ is already complete.
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
. "$V5SIM_ROOT/scripts/platform.sh"
PATH=$(v5_path); export PATH      # platform.sh: no slow Windows PATH; Homebrew on macOS
OUT=${V5SIM_FIRMWARE_DIR:-$V5SIM_ROOT/extracted}   # the extension keeps it elsewhere

VEXOS_NAME=VEXOS_V5_1_1_5_0
VEXOS_SHA256=56cbf3c3160dc571e5fb228e709357512671dfd2e3921b500e456dac94e054e3
CDN=https://content.vexrobotics.com/vexos/public/V5
DONE=$OUT/.fetched-$VEXOS_NAME

if [ -f "$DONE" ] && [ -f "$OUT/boot/p2_MainLoop_ps_01000000.bin" ] \
   && [ -f "$OUT/assets/files/DEVFRM02.BIN" ]; then
  echo "firmware: $VEXOS_NAME already in extracted/"
  exit 0
fi

sha_ok() { sha256_ok "$1" "$VEXOS_SHA256"; }

# The VEXcode extension's cached copy, if Windows interop is available.
vexcode_cache() {
  local appdata cmd
  if [ "$V5SIM_OS" = macos ]; then
    echo "$HOME/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/vexos/V5/$VEXOS_NAME.vexos"
    return 0
  fi
  # PATH above leaves out the Windows folders, so look where cmd.exe lives
  cmd=$(command -v cmd.exe || echo /mnt/c/Windows/System32/cmd.exe)
  [ -x "$cmd" ] || return 0
  appdata=$(cd / && "$cmd" /c 'echo %APPDATA%' 2>/dev/null | tr -d '\r') || return 0
  [ -n "$appdata" ] || return 0
  echo "$(wslpath -u "$appdata")/Code/User/globalStorage/vexrobotics.vexcode/vexos/V5/$VEXOS_NAME.vexos"
}

dl=$(mktemp -d); trap 'rm -rf "$dl"' EXIT
file=$dl/$VEXOS_NAME.vexos
if [ -n "${V5SIM_VEXOS_FILE:-}" ]; then
  echo "firmware: using $V5SIM_VEXOS_FILE"
  cp "$V5SIM_VEXOS_FILE" "$file"
else
  echo "firmware: downloading $CDN/$VEXOS_NAME.vexos"
  if ! curl -fsSL --retry 3 -o "$file" "$CDN/$VEXOS_NAME.vexos" || ! sha_ok "$file"; then
    cache=$(vexcode_cache)
    if [ -n "$cache" ] && [ -f "$cache" ]; then
      echo "firmware: download failed or didn't match; using VEXcode's copy $cache"
      cp "$cache" "$file"
    fi
  fi
fi
if ! sha_ok "$file"; then
  echo "firmware: $VEXOS_NAME.vexos is missing or doesn't match the expected SHA-256." >&2
  echo "  Download it in VEXcode (VEXos update) or set V5SIM_VEXOS_FILE, then rerun." >&2
  exit 1
fi

latest=$(curl -fsSL --max-time 10 "$CDN/catalog.txt" 2>/dev/null | tr -d '\r\n' || true)
if [ -n "$latest" ] && [ "$latest" != "$VEXOS_NAME" ]; then
  echo "firmware: note: VEX now publishes $latest; the simulator supports $VEXOS_NAME"
fi

echo "firmware: unpacking into extracted/"
rm -rf "$OUT/$VEXOS_NAME" "$OUT/boot" "$OUT/assets"
mkdir -p "$OUT"
python3 -I -c 'import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])' \
  "$file" "$OUT"
python3 -I "$V5SIM_ROOT/scripts/zynq_boot.py" "$OUT/$VEXOS_NAME/BOOT.bin" "$OUT/boot" >/dev/null
python3 -I "$V5SIM_ROOT/scripts/v5as.py" "$OUT/$VEXOS_NAME/assets.bin" \
  "$OUT/boot/p2_MainLoop_ps_01000000.bin" "$OUT/assets" >/dev/null
for f in boot/p2_MainLoop_ps_01000000.bin boot/p3_system_0_ps_03400000.bin assets/files/DEVFRM02.BIN; do
  [ -s "$OUT/$f" ] || { echo "firmware: unpacking didn't produce extracted/$f" >&2; exit 1; }
done
touch "$DONE"
echo "firmware: $VEXOS_NAME ready in extracted/"
