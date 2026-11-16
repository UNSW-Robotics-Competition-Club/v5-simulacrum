#!/bin/bash
# Build a test user program: programs/test-programs/build.sh <name>  ->  programs/test-programs/<name>/<name>.bin
set -euo pipefail
export PATH=/usr/bin:/bin           # see qemu_build.sh: slow Windows PATH
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
V5SIM_ROOT=$(cd "$HERE/../.." && pwd); export V5SIM_ROOT
ROOT=$V5SIM_ROOT
BIN="$HOME/re/arm-gnu/bin"
NAME=${1:?program name}
DIR="$HERE/$NAME"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
# -fno-tree-loop-distribute-patterns + -fno-builtin-{memset,memcpy}: at -O2 GCC
# rewrites a hand-rolled `while(n--) *p++=c` into a call to memset/memcpy. When
# the program *provides* its own memset (freestanding, no libc), that becomes an
# infinite self-recursion: the first call blows the CPU1 stack, overruns sys0's
# .bss (clobbering the XScuGic instance ptr g_scuGic @0x036c89dc), and the next
# CPU1 interrupt faults in XScuGic_InterruptHandler -> Xil_DataAbortHandler hangs.
# (Root-caused 2026-10-22.)
FLAGS=(-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -marm -O2
       -fno-tree-loop-distribute-patterns -fno-builtin-memset -fno-builtin-memcpy
       -ffunction-sections -fdata-sections -Wall -Wextra -I"$ROOT/sdk/include")

"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -c "$DIR/header.S" -o "$OUT/header.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -std=c11 -c "$DIR/main.c" -o "$OUT/main.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -std=c11 -c "$ROOT/sdk/src/v5_sdk.c" -o "$OUT/v5_sdk.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -c "$ROOT/sdk/src/v5_jumptable.S" -o "$OUT/jt.o"
"$BIN/arm-none-eabi-gcc" "${FLAGS[@]}" -nostartfiles -nostdlib -T "$HERE/link.ld" \
  -Wl,--gc-sections -Wl,-z,noexecstack "$OUT"/*.o -lgcc -o "$DIR/$NAME.elf"
"$BIN/arm-none-eabi-objcopy" -O binary "$DIR/$NAME.elf" "$DIR/$NAME.bin"
"$BIN/arm-none-eabi-size" "$DIR/$NAME.elf"
xxd -l 48 "$DIR/$NAME.bin"
