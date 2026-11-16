#!/bin/bash
# Build a VEXcode-style C++ project with VEXcode's own toolchain and SDK,
# using the flags of VEXcode's vex/mkenv.mk. Runs in Git Bash on Windows.
# Usage: programs/test-programs/vexcode_build.sh <project dir>  ->  <dir>/build/<name>.bin
set -euo pipefail
PROJ=$(cd "${1:?project dir}" && pwd)
NAME=$(basename "$PROJ")
VEX="$APPDATA/Code/User/globalStorage/vexrobotics.vexcode"
TC="$VEX/tools/cpp/toolchain_win32"
SDK=$(ls -d "$VEX"/sdk/cpp/V5/V5_*/vexv5 | sort -V | tail -1)
BUILD="$PROJ/build"
mkdir -p "$BUILD"

CFLAGS_CL=(-target thumbv7-none-eabi -fshort-enums -Wno-unknown-attributes
           -U__INT32_TYPE__ -U__UINT32_TYPE__ -D__INT32_TYPE__=long
           "-D__UINT32_TYPE__=unsigned long")
CFLAGS_V7=(-march=armv7-a -mfpu=neon -mfloat-abi=softfp)
CXX_FLAGS=(-Os -Wall -Werror=return-type -fno-rtti -fno-threadsafe-statics
           -fno-exceptions -std=gnu++11 -ffunction-sections -fdata-sections
           -DVexV5)
INC=(-I"$PROJ/include" -I"$SDK/clang/8.0.0/include" -I"$SDK/gcc/include"
     -I"$SDK/gcc/include/c++/4.9.3"
     -I"$SDK/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb"
     -I"$SDK/include")
[ -d "$PROJ/custom/include" ] && INC+=(-I"$PROJ/custom/include")

# Sources as a VEXcode project makefile lists them (RW-Template style too):
# src/*.cpp, src/*/*.cpp, custom/src/*.cpp
shopt -s nullglob
objs=()
for src in "$PROJ"/src/*.cpp "$PROJ"/src/*/*.cpp "$PROJ"/custom/src/*.cpp; do
  rel=${src#"$PROJ"/}
  obj="$BUILD/${rel//\//__}"          # src/a/b.cpp -> src__a__b.o: no clashes
  obj="${obj%.cpp}.o"
  "$TC/clang/bin/clang.exe" "${CFLAGS_CL[@]}" "${CFLAGS_V7[@]}" "${CXX_FLAGS[@]}" \
    "${INC[@]}" -c "$src" -o "$obj"
  objs+=("$obj")
done

(( ${#objs[@]} )) || { echo "no sources in $PROJ (src/, custom/src/)" >&2; exit 1; }
"$TC/gcc/bin/arm-none-eabi-ld.exe" -nostdlib -T "$SDK/lscript.ld" \
  -R "$SDK/stdlib_0.lib" -Map="$BUILD/$NAME.map" --gc-section -L"$SDK" \
  "${objs[@]}" --start-group -L"$SDK/gcc/libs" -lv5rt -lstdc++ -lc -lm -lgcc \
  --end-group -o "$BUILD/$NAME.elf"
"$TC/gcc/bin/arm-none-eabi-objcopy.exe" -O binary "$BUILD/$NAME.elf" "$BUILD/$NAME.bin"
"$TC/gcc/bin/arm-none-eabi-size.exe" "$BUILD/$NAME.elf"
