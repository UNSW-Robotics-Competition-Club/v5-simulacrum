# Sourced by the scripts: what differs between WSL/Linux and macOS.
# Bash 3.2 (macOS's /bin/bash) safe: no mapfile, associative arrays or ${x,,}.
#
#   V5SIM_OS                wsl | linux | macos
#   v5_path                 a short PATH with the build tools on it
#   sha256_ok FILE HASH     the file's SHA-256 is HASH
#   ncpu                    CPU count
#   sed_i EXPR FILE         sed -i, for GNU and BSD sed
#   lock_fd FD              non-blocking exclusive lock on open fd FD (flock -n)
#   unix_listening PATH     a unix socket is listening at PATH
#   tcp_listening PORT      something listens on 127.0.0.1:PORT

if [ -z "${V5SIM_OS:-}" ]; then
  case $(uname -s) in
    Darwin) V5SIM_OS=macos ;;
    *) if grep -qi microsoft /proc/version 2>/dev/null; then V5SIM_OS=wsl; else V5SIM_OS=linux; fi ;;
  esac
fi
export V5SIM_OS

# WSL appends the Windows PATH; a slow /mnt/c entry made every gcc helper
# lookup take ~40 s, so the scripts run with only the system's own folders.
# On macOS the build tools and Python come from Homebrew (/opt/homebrew on
# Apple Silicon, /usr/local on Intel), so those go first.
v5_path() {
  local p=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  if [ "$V5SIM_OS" = macos ]; then
    local brew=""
    for b in /opt/homebrew/bin/brew /usr/local/bin/brew; do [ -x "$b" ] && { brew=$b; break; }; done
    if [ -n "$brew" ]; then
      local prefix; prefix=$("$brew" --prefix)
      p=$prefix/bin:$prefix/sbin:$p
      # setup.sh's Python: its libexec/bin has the unversioned python3
      [ -d "$prefix/opt/python@3.12/libexec/bin" ] && p=$prefix/opt/python@3.12/libexec/bin:$p
    fi
  fi
  # a bundled Python (the VS Code extension's) first: on a Mac without the
  # developer tools /usr/bin/python3 is only a stub that offers to install them
  if [ -n "${V5SIM_PYTHON:-}" ] && [ -x "$V5SIM_PYTHON" ]; then
    p=$(dirname "$V5SIM_PYTHON"):$p
  fi
  echo "$p"
}

sha256_ok() {
  if command -v sha256sum >/dev/null 2>&1; then
    echo "$2  $1" | sha256sum -c --status - 2>/dev/null
  else
    echo "$2  $1" | shasum -a 256 -c --status - 2>/dev/null
  fi
}

ncpu() {
  nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2
}

sed_i() {
  sed -i.v5bak "$1" "$2" && rm -f "$2.v5bak"
}

# flock(1) isn't on macOS. flock(2) is, and the lock belongs to the open
# file, so a short Python taking it on the shell's fd leaves it held for as
# long as the shell keeps the fd open, the same as `flock -n FD`.
lock_fd() {
  if [ "$V5SIM_OS" != macos ] && command -v flock >/dev/null 2>&1; then
    flock -n "$1"
  else
    python3 -I -c 'import fcntl, sys
try:
    fcntl.flock(int(sys.argv[1]), fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    sys.exit(1)' "$1"
  fi
}

# Listening, not answering: QMP serves one client at a time, so a busy brain
# doesn't greet a second one, but its socket still accepts the connection.
unix_listening() {
  if [ "$V5SIM_OS" != macos ] && command -v ss >/dev/null 2>&1; then
    ss -xl 2>/dev/null | grep -qF " $1 "
  else
    [ -S "$1" ] && python3 -I -c 'import socket, sys
s = socket.socket(socket.AF_UNIX)
s.settimeout(0.3)
try:
    s.connect(sys.argv[1])
except OSError:
    sys.exit(1)
finally:
    s.close()' "$1"
  fi
}

tcp_listening() {
  if [ "$V5SIM_OS" != macos ] && command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | grep -q "127.0.0.1:$1 "
  else
    lsof -nP -iTCP@127.0.0.1:"$1" -sTCP:LISTEN >/dev/null 2>&1
  fi
}
