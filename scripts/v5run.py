#!/usr/bin/env python3
"""Run a program on the simulator headless and report how it did, as JSON:
for iterating on autonomous routines without watching each run.

    v5run.py PROGRAM.bin [--robot "ARGS" | --preset FILE.args] [--seconds S]
             [--settle S] [--expect X:Y:H] [--tol IN:DEG] [--min-score N]
             [--json OUT.json]
    v5run.py --batch RUNS.json [--jobs N] [--json OUT.json]

Each run starts its own brain (v5brain.sh --id auto: the first free
instance from 1, so several run side by side and an open sim is never
touched),
starts the program, and watches the field view. It stops S seconds after
the program started (--seconds, default 15: an autonomous period), or
earlier once the robot and every motor have been still for --settle
seconds (the routine is over), then reports
the final pose (sim metres and RW-Template inches/degrees), the score, the
blocks, opponents, the path, and each motor's temperature and faults.

--expect / --min-score turn it into a check: the exit code is 1 if the end
pose is off by more than --tol (default 2 in, 5 deg) or the score is low.

A batch file is a list of runs, each {"name", "program", "robot" or
"preset", "seconds", "settle", "expect", "tol", "min_score"} (only program
is needed); --jobs runs that many at once (default 2; each brain takes
about one CPU core). Robot args default to a six-motor blue drive on
MuJoCo if none are given. Run it in WSL:

    wsl -d Ubuntu-24.04 -- python3 scripts/v5run.py \\
        programs/test-programs/vexcode_intake/build/vexcode_intake.bin \\
        --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --intake 7:4 --physics mujoco" \\
        --seconds 12
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
INCH = 0.0254
DEFAULT_ROBOT = "--drive 1,2,3:4,5,6:blue --physics mujoco"
BOOT_TIMEOUT_S = 60.0            # launch to the program running
POLL_S = 0.1
STILL_M = 0.003                  # "still": moved less than this ...
STILL_DEG = 0.5                  # ... and turned less than this, per --settle
STILL_RPM = 2.0                  # ... and no motor turning faster than this


def rw_pose(x: float, y: float, h: float) -> dict:
    """Sim pose -> RW-Template field coordinates (in, in, deg clockwise)."""
    return {"x": round(-y / INCH, 2), "y": round(x / INCH, 2),
            "heading": round(-math.degrees(h) % 360, 2)}


def parse_expect(text: str) -> tuple[float, float, float]:
    x, y, h = (float(v) for v in text.split(":"))
    return x, y, h


def heading_error(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)


class Brain:
    """One v5brain.sh: its instance, its output, and its field view."""

    def __init__(self, program: str, robot_args: str, log_path: Path):
        cmd = ["bash", str(REPO / "scripts" / "v5brain.sh"), "--id", "auto",
               "--headless", "--run", program, "--robot", robot_args]
        env = dict(os.environ, PATH="/usr/local/bin:/usr/bin:/bin",
                   V5SIM_AUTO_FIRST=os.environ.get("V5SIM_AUTO_FIRST", "1"))
        self.log = log_path.open("w", encoding="utf-8", errors="replace")
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, env=env, text=True,
                                     errors="replace", start_new_session=True)
        self.instance: int | None = None
        self.qmp = ""
        self.started = threading.Event()
        self.lines: list[str] = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in self.proc.stdout:
            self.log.write(line)
            self.lines.append(line.rstrip("\n"))
            m = re.search(r"\[v5brain\] instance (\d+): QMP (\S+?),", line)
            if m:
                self.instance, self.qmp = int(m.group(1)), m.group(2)
            if "[v5brain] started slot" in line:
                self.started.set()
        self.log.flush()

    @property
    def view(self) -> str:
        return f"http://127.0.0.1:{8765 + (self.instance or 0)}"

    def get(self, path: str) -> dict | None:
        try:
            with urllib.request.urlopen(self.view + path, timeout=1.0) as r:
                return json.load(r)
        except (OSError, ValueError, urllib.error.URLError):
            return None

    def quit(self) -> None:
        if self.qmp:
            subprocess.run([sys.executable, "-I", str(REPO / "scripts" / "v5ctl.py"),
                            self.qmp, "quit"], capture_output=True, timeout=20)
        try:
            self.proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, 15)
            self.proc.wait(timeout=10)
        self.log.close()


def watch(brain: Brain, seconds: float, settle: float | None) -> tuple[dict, float, str]:
    """Wait for the program, then until `seconds` after it started or the
    robot settles. Returns the last state, the run time, why it stopped."""
    deadline = time.monotonic() + BOOT_TIMEOUT_S
    while not brain.started.wait(POLL_S):
        if brain.proc.poll() is not None or time.monotonic() > deadline:
            raise RuntimeError("the program never started: " +
                               " | ".join(brain.lines[-5:]))
    state = None
    while state is None:
        state = brain.get("/state.json")
        if time.monotonic() > deadline:
            raise RuntimeError("no field view (give the robot a --drive)")
    t0 = state["t"]
    anchor = (state["t"], state["x"], state["y"], state["heading"])
    moved = False
    while True:
        time.sleep(POLL_S)
        if brain.proc.poll() is not None:
            raise RuntimeError("the simulator exited: " + " | ".join(brain.lines[-5:]))
        s = brain.get("/state.json")
        if s is None:
            continue
        state = s
        run_t = s["t"] - t0
        if run_t >= seconds:
            return state, run_t, "time"
        d = math.hypot(s["x"] - anchor[1], s["y"] - anchor[2])
        dh = abs(math.degrees(s["heading"] - anchor[3]))
        if settle:
            devs = brain.get("/devices.json") or {"devices": []}
            spinning = any(abs(m.get("velocity_rpm") or 0) > STILL_RPM
                           for m in devs["devices"] if m["kind"] == "motor")
        else:
            spinning = False
        if d > STILL_M or dh > STILL_DEG or spinning:
            anchor, moved = (s["t"], s["x"], s["y"], s["heading"]), True
        elif settle and moved and s["t"] - anchor[0] >= settle:
            return state, run_t, "settled"


def summarise(state: dict, devices: dict | None, run_t: float, why: str) -> dict:
    out = {"run_s": round(run_t, 2), "stopped": why,
           "pose": {"x_m": round(state["x"], 4), "y_m": round(state["y"], 4),
                    "heading_rad": round(state["heading"], 4)},
           "rw": rw_pose(state["x"], state["y"], state["heading"]),
           "score": state.get("score"),
           "held": state.get("held", []),
           "blocks": state.get("blocks", []),
           "opponents": [rw_pose(*o) for o in state.get("opponents", [])],
           "path": [[round(-y / INCH, 1), round(x / INCH, 1)] for x, y in state.get("trail", [])]}
    if devices:
        out["motors"] = {str(d["port"]): {k: d.get(k) for k in
                                          ("position_deg", "velocity_rpm", "current_a",
                                           "temperature_c", "faults")}
                         for d in devices["devices"] if d["kind"] == "motor"}
        out["battery"] = next(({k: d[k] for k in ("volts", "amps", "percent")}
                               for d in devices["devices"] if d["kind"] == "battery"), None)
    return out


def check(result: dict, run: dict) -> list[str]:
    problems = []
    if run.get("expect"):
        x, y, h = parse_expect(run["expect"])
        tol_in, tol_deg = (float(v) for v in str(run.get("tol") or "2:5").split(":"))
        rw = result["rw"]
        off = math.hypot(rw["x"] - x, rw["y"] - y)
        dh = heading_error(rw["heading"], h)
        result["pose_error"] = {"in": round(off, 2), "deg": round(dh, 2)}
        if off > tol_in or dh > tol_deg:
            problems.append(f"ended {off:.1f} in / {dh:.1f} deg from {run['expect']}")
    if run.get("min_score") is not None:
        score = result["score"] or {}
        best = max(score.values(), default=0)
        if best < run["min_score"]:
            problems.append(f"score {best} < {run['min_score']}")
    return problems


def robot_args(run: dict, base: Path) -> str:
    """The robot sim runs in sim/, so files named in its args (blocks,
    goals, tags, obstacles, presets) are made absolute, relative to `base`."""
    def resolve(tok: str) -> str:
        at = tok.startswith("@")
        path = base / tok.lstrip("@")
        return ("@" if at else "") + str(path.resolve())             if not tok.startswith("-") and path.is_file() else tok
    if run.get("preset"):
        return resolve("@" + run["preset"])
    return " ".join(shlex.quote(resolve(t)) for t in shlex.split(run.get("robot") or DEFAULT_ROBOT))


def run_one(run: dict, log_dir: Path, base: Path) -> dict:
    name = run.get("name") or Path(run["program"]).stem
    program = str((base / run["program"]).resolve())
    if not Path(program).is_file():
        return {"name": name, "ok": False, "error": f"no program {program}"}
    brain = Brain(program, robot_args(run, base), log_dir / f"v5run_{name}.log")
    t_wall = time.monotonic()
    try:
        state, run_t, why = watch(brain, float(run.get("seconds", 15)), run.get("settle"))
        result = summarise(state, brain.get("/devices.json"), run_t, why)
    except RuntimeError as e:
        return {"name": name, "ok": False, "error": str(e), "instance": brain.instance}
    finally:
        brain.quit()
    problems = check(result, run)
    return {"name": name, "ok": not problems, "problems": problems,
            "instance": brain.instance, "wall_s": round(time.monotonic() - t_wall, 1),
            **result}


def one_line(r: dict) -> str:
    if "error" in r:
        return f"{r['name']:<20} ERROR {r['error']}"
    rw, sc = r["rw"], r.get("score")
    score = f"  score red {sc['red']} blue {sc['blue']}" if sc else ""
    err = r.get("pose_error")
    err_s = f"  off {err['in']} in {err['deg']} deg" if err else ""
    return (f"{r['name']:<20} {'ok  ' if r['ok'] else 'FAIL'} rw=({rw['x']:+.1f}, "
            f"{rw['y']:+.1f}, {rw['heading']:.0f}) after {r['run_s']} s ({r['stopped']})"
            f"{score}{err_s}" + ("  " + "; ".join(r["problems"]) if r["problems"] else ""))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("program", nargs="?", help="the program .bin")
    ap.add_argument("--robot", help=f'robot sim args (default "{DEFAULT_ROBOT}")')
    ap.add_argument("--preset", help="robot args from a file (robots/*.args)")
    ap.add_argument("--seconds", type=float, default=15.0,
                    help="stop this long after the program starts (15)")
    ap.add_argument("--settle", type=float, help="or once the robot is still this long")
    ap.add_argument("--expect", metavar="X:Y:H", help="expected end pose, RW in/deg")
    ap.add_argument("--tol", default="2:5", metavar="IN:DEG")
    ap.add_argument("--min-score", type=int)
    ap.add_argument("--batch", help="a JSON list of runs")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--json", help="write the full results here")
    args = ap.parse_args(argv)
    if bool(args.batch) == bool(args.program):
        ap.error("give a PROGRAM or --batch FILE")
    if args.batch:
        try:
            runs = json.loads(Path(args.batch).read_text())
            if not isinstance(runs, list) or not all(isinstance(r, dict) and "program" in r
                                                     for r in runs):
                raise ValueError("expected a list of {\"program\": ...} runs")
        except (OSError, ValueError) as e:
            ap.error(f"--batch {args.batch}: {e}")
        base = Path(args.batch).resolve().parent
    else:
        base = Path.cwd()
        runs = [{"program": args.program, "robot": args.robot, "preset": args.preset,
                 "seconds": args.seconds, "settle": args.settle, "expect": args.expect,
                 "tol": args.tol, "min_score": args.min_score}]
    names = [r.get("name") or Path(r["program"]).stem for r in runs]
    if len(set(names)) != len(names):
        ap.error("give each run a different \"name\"")
    log_dir = Path.home() / "re" / "qemu-runs"
    log_dir.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = []
        for r in pool.map(lambda run: run_one(run, log_dir, base), runs):
            print(one_line(r), flush=True)
            results.append(r)
    if args.json:
        Path(args.json).write_text(json.dumps(results if args.batch else results[0], indent=1))
    elif not args.batch:
        print(json.dumps(results[0], indent=1))
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
