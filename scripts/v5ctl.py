"""Drive a running vex-v5 QEMU over QMP: touch taps, screenshots, waits.

Usage: python3 -I v5ctl.py <qmp.sock> <cmd> [<cmd> ...]
  wait:<secs>          sleep (wall clock)
  tap:<x>,<y>          press and release at screen pixel (480x272)
  hold:<x>,<y>,<secs>  press, hold, release
  down:<x>,<y> / up    press / release separately
  run:<slot>           from Home: Programs > slot 1..8 > Run
  button:<secs>        hold the brain's front button: 0.2 = Back/Home, ~1.5 stops
                       a running program, 5 with no program powers off (QEMU exits)
  shot:<file.ppm>      screendump
  pmem:<addr>,<size>,<file>  save guest physical memory
  hmp:<monitor cmd>    run a human-monitor command, print its output
                       (use '_' for spaces, e.g. hmp:xp_/4wx_0xffff12d6)
  quit                 stop QEMU
Example: v5ctl.py /tmp/v5qmp.sock wait:12 shot:a.ppm tap:400,252 wait:1 shot:b.ppm quit
"""
import json
import os
import socket
import sys
import time

SCREEN_W, SCREEN_H = 480, 272
ABS_MAX = 0x7FFF


class Qmp:
    def __init__(self, path):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.connect(path)
        self.buf = b""
        self._recv()  # greeting
        self.cmd("qmp_capabilities")

    def _recv(self):
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("QMP closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line)

    def cmd(self, name, **args):
        msg = {"execute": name}
        if args:
            msg["arguments"] = args
        self.sock.sendall(json.dumps(msg).encode() + b"\n")
        while True:
            reply = self._recv()
            if "event" in reply:
                continue
            if "error" in reply:
                raise RuntimeError(f"{name}: {reply['error']}")
            return reply.get("return")


def _abs(axis, pixels, size):
    return {"type": "abs", "data": {"axis": axis, "value": (pixels * (ABS_MAX + 1) + size // 2) // size}}


def _btn(down):
    return {"type": "btn", "data": {"button": "left", "down": down}}


def geometry(q):
    """(canvas w, h, screen x, y): a windowed brain shows its screen inside
    a picture of the brain (/machine/display's skin); touches and
    screendumps cover the whole picture."""
    if not hasattr(q, "geom"):
        try:
            q.geom = tuple(q.cmd("qom-get", path="/machine/display", property=name)
                           for name in ("canvas-w", "canvas-h", "screen-x", "screen-y"))
        except RuntimeError:                     # an older machine: bare screen
            q.geom = (SCREEN_W, SCREEN_H, 0, 0)
    return q.geom


def _press_at(q, x, y):
    """Events putting the finger on screen pixel (x, y)."""
    w, h, sx, sy = geometry(q)
    return [_abs("x", x + sx, w), _abs("y", y + sy, h)]


def touch(q, x, y, secs):
    q.cmd("input-send-event", events=_press_at(q, x, y) + [_btn(True)])
    time.sleep(secs)
    q.cmd("input-send-event", events=[_btn(False)])


# Home screen > Programs > slot grid (4x2) > program page "Run".
PROGRAMS_BUTTON = (440, 230)
SLOT_GRID_X0, SLOT_GRID_DX = 60, 120
SLOT_GRID_Y = (95, 215)
RUN_BUTTON = (60, 75)
HEADER_ROWS = 32              # the status bar (clock, battery) changes by itself
POLL_S = 0.05
RETAP_S = 1.0                 # no reaction yet (still booting): tap again
RUN_TIMEOUT_S = 30.0


def _frame(q, path):
    """The brain's screen (cut out of the skin, if any) as raw RGB bytes."""
    q.cmd("screendump", filename=path)
    with open(path, "rb") as f:
        data = f.read()
    header_end = 0
    for _ in range(3):        # P6 / width height / maxval lines
        header_end = data.index(b"\n", header_end) + 1
    pixels = data[header_end:]
    w, _, sx, sy = geometry(q)
    if (w, sx, sy) == (SCREEN_W, 0, 0):
        return pixels
    return b"".join(pixels[((sy + r) * w + sx) * 3:((sy + r) * w + sx + SCREEN_W) * 3]
                    for r in range(SCREEN_H))


def _screen(q, path):
    """The screen below the status bar (which changes by itself)."""
    return _frame(q, path)[HEADER_ROWS * SCREEN_W * 3:]


def _drawn_at(frame, x, y, r=8):
    """Something other than black around (x, y): a button to tap."""
    for yy in range(max(0, y - r), min(SCREEN_H, y + r)):
        row = frame[(yy * SCREEN_W + max(0, x - r)) * 3:(yy * SCREEN_W + min(SCREEN_W, x + r)) * 3]
        if any(row):
            return True
    return False


def _wait_for_button(q, x, y, path, deadline):
    """Until the screen is still and shows something at (x, y). During boot
    VEXos shows a splash, then black, then Home; taps before Home are lost."""
    last = None
    while time.monotonic() < deadline:
        frame = _frame(q, path)
        if frame == last and _drawn_at(frame, x, y):
            return
        last = frame
        time.sleep(POLL_S)
    raise TimeoutError(f"nothing to tap at ({x}, {y})")


def tap_until_changed(q, x, y, path, deadline):
    """Tap, then wait for the screen to change; tap again while it doesn't.
    Returns once the new screen has held still for one poll."""
    _wait_for_button(q, x, y, path, deadline)
    before = _screen(q, path)
    while time.monotonic() < deadline:
        touch(q, x, y, 0.1)
        retap = time.monotonic() + RETAP_S
        while time.monotonic() < min(retap, deadline):
            time.sleep(POLL_S)
            now = _screen(q, path)
            if now != before:
                time.sleep(POLL_S)
                if _screen(q, path) == now:
                    return
                before = now      # still drawing: wait for it to settle
    raise TimeoutError(f"the screen never reacted to a tap at ({x}, {y})")


def run_slot(q, slot):
    """Start the program in slot 1..8 from the Home screen, as a user would:
    each tap waits for the screen it opens, so this takes about as long as
    VEXos takes to draw them."""
    col, row = (slot - 1) % 4, (slot - 1) // 4
    path = f"/tmp/v5ctl-{os.getpid()}.ppm"
    deadline = time.monotonic() + RUN_TIMEOUT_S
    try:
        for x, y in (PROGRAMS_BUTTON,
                     (SLOT_GRID_X0 + SLOT_GRID_DX * col, SLOT_GRID_Y[row]),
                     RUN_BUTTON):
            tap_until_changed(q, x, y, path, deadline)
    finally:
        if os.path.exists(path):
            os.unlink(path)


def run(q, step):
    op, _, arg = step.partition(":")
    if op == "wait":
        time.sleep(float(arg))
    elif op == "tap":
        x, y = (int(v) for v in arg.split(","))
        touch(q, x, y, 0.15)
    elif op == "run":
        run_slot(q, int(arg))
    elif op == "button":
        q.cmd("qom-set", path="/machine/gpio1", property="in1", value=0x00)
        time.sleep(float(arg or 0.2))
        q.cmd("qom-set", path="/machine/gpio1", property="in1", value=0x80)
    elif op == "hold":
        x, y, secs = arg.split(",")
        touch(q, int(x), int(y), float(secs))
    elif op == "down":
        x, y = (int(v) for v in arg.split(","))
        q.cmd("input-send-event", events=_press_at(q, x, y) + [_btn(True)])
    elif op == "up":
        q.cmd("input-send-event", events=[_btn(False)])
    elif op == "hmp":
        out = q.cmd("human-monitor-command", **{"command-line": arg.replace("_", " ")})
        print(f"[{arg}]\n{out}", end="")
    elif op == "pmem":
        addr, size, path = arg.split(",", 2)
        q.cmd("pmemsave", val=int(addr, 0), size=int(size, 0), filename=path)
    elif op == "shot":
        q.cmd("screendump", filename=arg)
    elif op == "quit":
        try:
            q.cmd("quit")
        except ConnectionError:
            pass
    else:
        raise SystemExit(f"unknown command: {step}")


def main():
    q = Qmp(sys.argv[1])
    for step in sys.argv[2:]:
        run(q, step)


if __name__ == "__main__":
    main()
