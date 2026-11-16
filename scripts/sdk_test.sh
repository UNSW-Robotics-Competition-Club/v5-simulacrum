#!/usr/bin/env bash
# Build-check the generated C/C++ SDK (sdk/) with arm-none-eabi-gcc (in ~/re/arm-gnu, see below):
# header as C11 and C++17 with -Werror, the stubs and wrappers, then link a sample at the
# user-program base and check that calls reach the right slots.
# Toolchain: Arm GNU 14.2.rel1 x86_64 arm-none-eabi, unpacked into ~/re/arm-gnu.
set -euo pipefail
V5SIM_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); export V5SIM_ROOT
ROOT=$V5SIM_ROOT
SDK="$ROOT/sdk"
BIN="$HOME/re/arm-gnu/bin"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
FLAGS=(-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -ffunction-sections -fdata-sections -O2
       -Wall -Wextra -Werror -I"$SDK/include")

"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -std=c11 -pedantic -c "$SDK/test/sample.c" -o "$OUT/sample.o"
"$BIN/arm-none-eabi-g++" "${FLAGS[@]}" -std=c++17 -pedantic -fno-exceptions -c "$SDK/test/sample.cpp" -o "$OUT/sample_cpp.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -std=c11 -c "$SDK/src/v5_sdk.c" -o "$OUT/v5_sdk.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -c "$SDK/src/v5_jumptable.S" -o "$OUT/v5_jumptable.o"
if "$BIN/arm-none-eabi-gcc" -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon -I"$SDK/include" -fsyntax-only -x c "$SDK/test/sample.c" 2>/dev/null; then
  echo "FAIL: hard-float build was not rejected"; exit 1
fi

"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-z,noexecstack -Wl,-Ttext=0x03800000 \
  -Wl,-e,sample_main "$OUT/sample.o" "$OUT/sample_cpp.o" "$OUT/v5_sdk.o" "$OUT/v5_jumptable.o" -lgcc -o "$OUT/sample.elf"
"$BIN/arm-none-eabi-objdump" -d "$OUT/sample.elf" > "$OUT/sample.dis"

# Each stub must be movw/movt of 0x037fc000+off, then ldr pc,[ip].
check_stub() {
  local name=$1 off=$2 addr
  addr=$(printf '%x' $((0x037fc000 + off)))
  awk -v n="<$name>:" '$2 == n {on=1; next} on && NF == 0 {exit} on {print}' "$OUT/sample.dis" > "$OUT/stub.txt"
  grep -q "movw.*ip, #$((0x${addr} & 0xffff))" "$OUT/stub.txt" &&
    grep -q "movt.*ip, #$((0x${addr} >> 16))" "$OUT/stub.txt" &&
    grep -q "ldr.*pc, \[ip\]" "$OUT/stub.txt" || { echo "FAIL: stub $name"; cat "$OUT/stub.txt"; exit 1; }
  echo "ok  $name -> slot $(printf '0x%03x' "$off")"
}
check_stub vexSystemTimeGet 0x118
check_stub vexDeviceMotorVelocitySet 0x2d0
check_stub vexDeviceMotorPositionSet 0x334
check_stub vexTaskSleep 0x06c
check_stub vexGettime 0x11c
check_stub vexDeviceGpsHeadingGet 0x5cc

"$BIN/arm-none-eabi-size" "$OUT/sample.elf"
echo SDK_TEST_OK
