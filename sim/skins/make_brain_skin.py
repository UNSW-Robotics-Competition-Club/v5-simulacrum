"""Draw the V5 brain's face around its screen: sim/skins/v5-brain.ppm.

  python3 sim/skins/make_brain_skin.py [out.ppm]

Traced from a top-down photo of a V5 brain (1200 px square, screen at
x 205..917, y 395..805), scaled so the screen is exactly 480 x 272: ports
1-10 along the top edge and 11-20 along the bottom in two groups of five,
the 3-wire ports A-H on the left edge, port 21 / USB / battery on the right,
and the front button, a rounded square with an exit icon, to the right of
the screen halfway down.

The PPM header carries the geometry QEMU's display and touch use:
  # v5skin screen=X,Y button=X,Y,HALF
(the screen's top-left, the button's centre and half its side). Pure Python,
no image libraries.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

PHOTO_ORIGIN = (108, 238)            # the body's top-left in the photo
K = 480 / 712                        # photo px -> sim px (screen 712 px wide)
MARGIN = 12
# width a multiple of 4: QEMU's screendump pads rows otherwise
W = -(-(round(992 * K) + 2 * MARGIN) // 4) * 4
H = round(722 * K) + 2 * MARGIN

BG = (24, 25, 28)
BODY = (28, 29, 31)
BODY_EDGE = (52, 54, 58)
PANEL = (70, 73, 78)
PANEL_EDGE = (88, 91, 96)
BEZEL = (58, 61, 65)
SCREEN_RIM = (12, 12, 13)
HOUSING = (16, 16, 17)
SLOT = (34, 35, 37)
LABEL = (205, 208, 212)
LABEL_TEXT = (245, 246, 248)
LABEL_BG = (52, 55, 59)
BUTTON = (40, 42, 45)
BUTTON_EDGE = (96, 99, 104)
ICON = (225, 228, 232)


def p(x: float, y: float) -> tuple[float, float]:
    """A point in the photo, in skin pixels."""
    return (x - PHOTO_ORIGIN[0]) * K + MARGIN, (y - PHOTO_ORIGIN[1]) * K + MARGIN


SCREEN = (round(p(205, 395)[0]), round(p(205, 395)[1]))
BUTTON_C = tuple(round(v) for v in p(1013, 599))
BUTTON_HALF = round(46 * K)


class Canvas:
    def __init__(self, w: int, h: int, fill: tuple[int, int, int]):
        self.w, self.h = w, h
        self.px = [list(fill) for _ in range(w * h)]

    def put(self, x: int, y: int, c, a: float = 1.0) -> None:
        if 0 <= x < self.w and 0 <= y < self.h and a > 0:
            d = self.px[y * self.w + x]
            for i in range(3):
                d[i] = round(d[i] + (c[i] - d[i]) * min(a, 1.0))

    def rrect(self, x0, y0, x1, y1, r, c, edge=None, width=1.5):
        """Filled rounded rectangle, antialiased, with an optional edge."""
        for y in range(int(y0) - 1, int(y1) + 2):
            for x in range(int(x0) - 1, int(x1) + 2):
                # signed distance to the rounded box
                qx = max(abs(x + 0.5 - (x0 + x1) / 2) - ((x1 - x0) / 2 - r), 0)
                qy = max(abs(y + 0.5 - (y0 + y1) / 2) - ((y1 - y0) / 2 - r), 0)
                d = math.hypot(qx, qy) - r
                self.put(x, y, c, 0.5 - d)
                if edge is not None:
                    self.put(x, y, edge, min(0.5 - d, 0.5 + d + width))

    def ring(self, cx, cy, r, width, c, gap=None):
        """A circle outline; gap = (start, end) angles in degrees left open."""
        for y in range(int(cy - r - width), int(cy + r + width) + 2):
            for x in range(int(cx - r - width), int(cx + r + width) + 2):
                dx, dy = x + 0.5 - cx, y + 0.5 - cy
                a = math.degrees(math.atan2(dy, dx)) % 360
                if gap and (gap[0] <= a <= gap[1]):
                    continue
                self.put(x, y, c, width / 2 + 0.5 - abs(math.hypot(dx, dy) - r))

    def line(self, x0, y0, x1, y1, width, c):
        minx, maxx = int(min(x0, x1) - width), int(max(x0, x1) + width) + 1
        miny, maxy = int(min(y0, y1) - width), int(max(y0, y1) + width) + 1
        L = math.hypot(x1 - x0, y1 - y0) or 1
        for y in range(miny, maxy + 1):
            for x in range(minx, maxx + 1):
                px, py = x + 0.5, y + 0.5
                t = max(0, min(1, ((px - x0) * (x1 - x0) + (py - y0) * (y1 - y0)) / L ** 2))
                d = math.hypot(px - (x0 + t * (x1 - x0)), py - (y0 + t * (y1 - y0)))
                self.put(x, y, c, width / 2 + 0.5 - d)

    def write_ppm(self, path: Path, comment: str) -> None:
        with open(path, "wb") as f:
            f.write(f"P6\n# {comment}\n{self.w} {self.h}\n255\n".encode())
            f.write(bytes(v for px in self.px for v in px))


# 5x9 digits for the port labels, drawn on whole pixels so they stay sharp
DIGITS = {
    "0": [".###.", "#...#", "#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."],
    "1": ["..#..", ".##..", "#.#..", "..#..", "..#..", "..#..", "..#..", "..#..", "#####"],
    "2": [".###.", "#...#", "....#", "....#", "...#.", "..#..", ".#...", "#....", "#####"],
    "3": [".###.", "#...#", "....#", "....#", "..##.", "....#", "....#", "#...#", ".###."],
    "4": ["...#.", "..##.", ".#.#.", "#..#.", "#..#.", "#####", "...#.", "...#.", "...#."],
    "5": ["#####", "#....", "#....", "####.", "....#", "....#", "....#", "#...#", ".###."],
    "6": [".###.", "#...#", "#....", "#....", "####.", "#...#", "#...#", "#...#", ".###."],
    "7": ["#####", "....#", "...#.", "...#.", "..#..", "..#..", ".#...", ".#...", ".#..."],
    "8": [".###.", "#...#", "#...#", "#...#", ".###.", "#...#", "#...#", "#...#", ".###."],
    "9": [".###.", "#...#", "#...#", "#...#", ".####", "....#", "....#", "#...#", ".###."],
}
GLYPH_W, GLYPH_H, GAP = 5, 9, 1


def text(cv: Canvas, s: str, cx: float, cy: float, c) -> None:
    """Digits centred on (cx, cy), snapped to the pixel grid."""
    x0 = round(cx - (len(s) * (GLYPH_W + GAP) - GAP) / 2)
    y0 = round(cy - GLYPH_H / 2)
    for i, ch in enumerate(s):
        for r, row in enumerate(DIGITS[ch]):
            for col, bit in enumerate(row):
                if bit == "#":
                    cv.put(x0 + i * (GLYPH_W + GAP) + col, y0 + r, c)


PORT_GROUPS = (228, 580)              # photo x of each group of five


def port_housings(cv: Canvas, house_y0: float, house_y1: float) -> None:
    for x0 in PORT_GROUPS:
        hx0, hy0 = p(x0, house_y0)
        hx1, hy1 = p(x0 + 315, house_y1)
        cv.rrect(hx0, hy0, hx1, hy1, 2, HOUSING)
        for k in range(5):
            sx = x0 + 6 + k * 63
            a, b = p(sx, house_y0 + 14)
            c, d = p(sx + 52, house_y1 - 10)
            cv.rrect(a, b, c, d, 2, SLOT)


def port_labels(cv: Canvas, label_y: float, first: int) -> None:
    for group, x0 in enumerate(PORT_GROUPS):
        for k in range(5):
            lx, ly = p(x0 + 32 + k * 63, label_y)
            lx, ly = round(lx), round(ly)
            cv.rrect(lx - 11.5, ly - 9.5, lx + 11.5, ly + 9.5, 3, LABEL_BG,
                     edge=LABEL, width=1.0)
            text(cv, str(first + group * 5 + k), lx, ly, LABEL_TEXT)


def draw() -> Canvas:
    cv = Canvas(W, H, BG)
    bx0, by0 = p(108, 238)
    bx1, by1 = p(1100, 960)
    cv.rrect(bx0, by0, bx1, by1, 40, BODY, edge=BODY_EDGE, width=1.0)
    port_housings(cv, 255, 322)
    port_housings(cv, 880, 945)
    px0, py0 = p(120, 322)
    px1, py1 = p(1088, 880)
    cv.rrect(px0, py0, px1, py1, 26, PANEL, edge=PANEL_EDGE, width=1.0)
    qx0, qy0 = p(176, 378)
    qx1, qy1 = p(1060, 822)
    cv.rrect(qx0, qy0, qx1, qy1, 14, BEZEL)
    sx, sy = SCREEN
    cv.rrect(sx - 4, sy - 4, sx + 480 + 4, sy + 272 + 4, 4, SCREEN_RIM)
    port_labels(cv, 352, 1)
    port_labels(cv, 849, 11)
    # 3-wire ports A..H on the left edge, port 21 / USB on the right
    for i in range(8):
        tx, ty = p(110, 482 + i * 36)
        cv.line(tx - 2, ty - 5, tx + 4, ty, 1.4, LABEL)
        cv.line(tx + 4, ty, tx - 2, ty + 5, 1.4, LABEL)
    rx, ry = p(1093, 395)
    cv.rrect(rx - 5, ry - 7, rx + 5, ry + 7, 2, BODY, edge=LABEL, width=1.0)
    ux, uy = p(1093, 600)
    cv.line(ux, uy - 7, ux, uy + 7, 1.4, LABEL)
    cv.line(ux - 3, uy - 3, ux + 3, uy - 3, 1.4, LABEL)
    # battery connector sticking out of the right edge
    ax0, ay0 = p(1098, 732)
    ax1, ay1 = p(1116, 790)
    cv.rrect(ax0, ay0, ax1, ay1, 2, HOUSING, edge=BODY_EDGE, width=1.0)
    # the front button: rounded square, exit icon (ring open on the left, arrow in)
    cx, cy = BUTTON_C
    hb = BUTTON_HALF
    cv.rrect(cx - hb, cy - hb, cx + hb, cy + hb, 6, BUTTON, edge=BUTTON_EDGE, width=1.2)
    cv.ring(cx + 3, cy, hb * 0.48, 2.6, ICON, gap=(140, 220))
    cv.line(cx - hb * 0.62, cy, cx + 1, cy, 2.6, ICON)
    cv.line(cx - hb * 0.62, cy, cx - hb * 0.36, cy - hb * 0.24, 2.6, ICON)
    cv.line(cx - hb * 0.62, cy, cx - hb * 0.36, cy + hb * 0.24, 2.6, ICON)
    return cv


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("v5-brain.ppm")
    cx, cy = BUTTON_C
    draw().write_ppm(out, f"v5skin screen={SCREEN[0]},{SCREEN[1]} "
                          f"button={cx},{cy},{BUTTON_HALF}")
    print(f"{out}: {W}x{H}, screen at {SCREEN}, button {BUTTON_C} +-{BUTTON_HALF}")


if __name__ == "__main__":
    main()
