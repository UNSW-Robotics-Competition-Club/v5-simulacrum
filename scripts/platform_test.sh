#!/bin/bash
# Checks scripts/platform.sh's helpers on this machine, then again with the
# macOS code paths forced (V5SIM_OS=macos), which work on Linux too.
#   bash scripts/platform_test.sh
set -uo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fail=0
check() { if eval "$2"; then echo "ok   $1"; else echo "FAIL $1"; fail=1; fi; }

run_checks() {
  local os=$1 t; t=$(mktemp -d)
  # shellcheck disable=SC1091
  V5SIM_OS=$os . "$HERE/platform.sh"
  echo "== V5SIM_OS=$V5SIM_OS"
  printf 'hello\n' > "$t/f"
  check "sha256_ok match" 'sha256_ok "$t/f" 5891b5b522d5df086d0ff0b110fbd9d21bb4fc7163af34d08286a2e846f6be03'
  check "sha256_ok mismatch" '! sha256_ok "$t/f" 0000000000000000000000000000000000000000000000000000000000000000'
  check "ncpu" '[ "$(ncpu)" -ge 1 ]'
  printf 'a\nCONFIG_X\nb\n' > "$t/m"
  sed_i '/CONFIG_X/d' "$t/m"
  check "sed_i" '[ "$(cat "$t/m")" = "$(printf "a\nb")" ] && [ ! -e "$t/m.v5bak" ]'
  check "v5_path" '[ -n "$(v5_path)" ]'
  # lock_fd: a second holder is refused while the first keeps its fd open
  exec 8>"$t/lock"
  check "lock_fd takes a free lock" 'lock_fd 8'
  check "lock_fd refuses a held lock" '! ( exec 7>"$t/lock"; lock_fd 7 )'
  exec 8>&-
  check "lock_fd after release" '( exec 7>"$t/lock"; lock_fd 7 )'
  # unix_listening / tcp_listening against a real listener
  python3 -I -c 'import socket, sys, time
u = socket.socket(socket.AF_UNIX); u.bind(sys.argv[1]); u.listen(1)
t = socket.socket(); t.bind(("127.0.0.1", 0)); t.listen(1)
open(sys.argv[2], "w").write(str(t.getsockname()[1]))
time.sleep(5)' "$t/s.sock" "$t/port" &
  local pid=$!
  for _ in $(seq 50); do [ -s "$t/port" ] && break; sleep 0.1; done
  check "unix_listening (up)" 'unix_listening "$t/s.sock"'
  check "unix_listening (none)" '! unix_listening "$t/nothing.sock"'
  if [ "$os" != macos ]; then
    # under pipefail, grep -q quitting at the match SIGPIPEs a long `ss` and
    # the check reads as "not listening" (v5fleet stop then did nothing)
    mkdir -p "$t/bin"
    printf '#!/bin/sh\necho "u_str LISTEN 0 5 %s 1 * 0"\necho "127.0.0.1:4242 x"\nyes "u_str LISTEN 0 5 /tmp/other 2 * 0" | head -n 200000\n' "$t/s.sock" > "$t/bin/ss"
    chmod +x "$t/bin/ss"
    check "unix_listening with a long ss" 'PATH="$t/bin:$PATH" unix_listening "$t/s.sock"'
    check "tcp_listening with a long ss" 'PATH="$t/bin:$PATH" tcp_listening 4242'
  fi
  if command -v lsof >/dev/null || [ "$os" != macos ]; then
    check "tcp_listening (up)" 'tcp_listening "$(cat "$t/port")"'
    check "tcp_listening (none)" '! tcp_listening 1'
  else
    echo "skip tcp_listening (no lsof here)"
  fi
  kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
  rm -rf "$t"
}

native=$( unset V5SIM_OS; . "$HERE/platform.sh"; echo "$V5SIM_OS" )
run_checks "$native"
[ "$native" = macos ] || run_checks macos
[ "$fail" = 0 ] && echo "platform_test: all ok" || { echo "platform_test: FAILED"; exit 1; }
