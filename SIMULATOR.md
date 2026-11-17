# V5 Brain Simulator: user guide

The simulator runs the real VEX V5 brain firmware (VEXos 1.1.5.0) in an emulator, with a simulated robot attached to its smart ports. Programs built for a real V5 run unchanged: VEXcode C++, VEXcode Python and PROS. You see the brain's screen and tap it with the mouse. You watch the robot on a field view in your browser and drive it from the keyboard. You upload with the same tools you use on a real brain.

Use it to:

- test autonomous routines without a robot or field;
- practise odometry, distance resets, MCL and path following (RW-Template runs unmodified);
- run matches: autonomous, a pause, then driver control;
- try sensors (IMU, distance, optical, GPS, AI Vision, 3-wire) and game objects;
- debug vexide programs in GDB with v5gdb (section 5).

Developer notes: [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md).

---

## 1. Quick start (Windows)

```bat
V5Sim                                    :: drivetrain robot, no program
V5Sim competition path\to\program.bin    :: full sensor robot, runs your program
V5Sim pushback-practice                  :: competition robot + 24 blocks on the field
V5Sim ui-test path\to\program.bin        :: on a field controller, disabled: test your UI
V5Sim competition my_auton.bin --match 20              :: field control match
V5Sim pushback-practice - --physics mujoco              :: no program, MuJoCo physics
```

Anything after the program (or `-` for none) is added to the preset's robot options (section 4). Every preset has a VEXnet radio with a wireless controller (`drivetrain` and `competition` on port 11, `rw-template` on 21). A `--radio` you add replaces the preset's (e.g. `--radio 11:partner` for a second controller); `--controller P` alone does not, since a radio replaces a tethered controller. cmd.exe splits arguments at `=`, so quote those: `"--start=-36:-60:0"`.

`V5Sim.cmd` is in the repo root. It does four things:

1. Boots the brain in a window. Click on the screen to touch it.
2. Attaches a simulated robot, the **preset** named first: see section 4.
3. Opens the **field view** at http://localhost:8765/.
4. Exposes the brain's **USB ports** inside WSL as `/tmp/ttyV5sys` and `/tmp/ttyV5user`, for uploading and the terminal (section 5).

If you give a program, it is installed in slot 1 and started once the brain has booted: VEXos boots in about 1.6 s and the program runs about 2.5 s after launch. Close the brain window to stop everything. Set `V5SIM_HEADLESS=1` to run without the window.

**Several at once:** each `V5Sim` takes the first free simulator instance. The first one uses the addresses above. If one is already running (yours, or a test run), the next becomes instance N: field view on port 8765+N, USB ports `/tmp/v5simN/ttyV5sys` and `/tmp/v5simN/ttyV5user`, and its own program storage. The `[v5brain] instance N: ...` line it prints says where everything is. Set `V5SIM_ID=N` to pick one; a busy instance refuses to start rather than take over. From WSL, `scripts/v5brain.sh --id N|auto` does the same (it defaults to instance 0, which the e2e checks use).

### On macOS (and Linux)

`./v5sim` in the repo root is the same launcher for a Mac (Apple Silicon or Intel) or a Linux PC, after `bash scripts/setup.sh` (section 2):

```sh
./v5sim                                    # drivetrain robot, no program
./v5sim competition path/to/program.bin    # full sensor robot, runs your program
./v5sim competition my_auton.bin --match 20 --start=-36:-60:0
./v5sim start competition                  # in the background (no window); ./v5sim list / log / stop
```

Presets, programs, robot options and instances work as above. The brain's USB is always the two serial ports, `/tmp/ttyV5sys` and `/tmp/ttyV5user` (section 5, "On macOS"). There is no `attach`/`detach`: they're for Windows USB. A background instance has no window; its screen is at http://localhost:8765/screen. Stop it with `./v5sim stop <id>`.

### Testing your UI code (autonomous selectors, screens)

```bat
V5Sim ui-test path\to\program.bin
```

This starts one brain running your program, with a wireless controller, plugged into a field controller, and opens the **field control page** (http://localhost:8765/field) in your browser. Like a robot waiting on the field at an event, it starts **disabled**: VEXcode's `pre_auton` (or PROS's `competition_initialize`) runs, so an autonomous selector on the brain screen or controller screen behaves as it will at a match.

1. Pick on the brain screen by clicking it (click = touch), or with the controller buttons from the field view (J K U I = A B X Y).
2. On the field control page, press **START** for a full match: 0:15 autonomous, a 2 s pause, then 1:45 driver control, timed by the sim like a real field (the brain's banner timer counts each period). The **Driver Skills** and **Auton Skills** tabs run 1:00 skills runs; **Manual** switches between disabled, autonomous and driver yourself. The station box shows whether the robot's radio is linked, whether your program is running, and its battery.
3. To rerun from the start, stop the program with the brain's button and run it again from the brain screen. Press **disabled** first so it starts the way it will on a field.

For a starting point, `programs/test-programs/vexcode_ui` is a VEXcode competition program with an autonomous selector on both screens. Build it in Git Bash with `bash programs/test-programs/vexcode_build.sh programs/test-programs/vexcode_ui` and run `V5Sim ui-test programs	est-programsexcode_uiuildexcode_ui.bin`.

The controller's screen is shown in the field view, so controller-screen UI can be checked there too. Any robot works the same way: add `--competition field` to another preset (`V5Sim drivetrain program.bin --competition field`), or use `--competition switch` to test against a competition switch instead of a field. For a timed match with no clicking, use `--match` instead.

### Managing instances (open, list, shut down)

`V5Sim <preset>` runs the sim in the current window (its serial console in that window; close it or Ctrl-C to stop). To run instances in the **background** and manage them by id, use the verbs:

```bat
V5Sim start competition     :: run in the background; prints the instance id
V5Sim log 0                 :: follow instance 0's console live (Ctrl-C quits the view)
V5Sim list                  :: running instances, their field-view and USB/IP ports
V5Sim status 0              :: detail for one instance
V5Sim stop 0                :: graceful shutdown (QEMU quit; USB detaches cleanly)
V5Sim stop all              :: stop every running instance
V5Sim clear 0               :: empty all 8 program slots (keep settings)
V5Sim reset 0               :: factory reset the brain (wipe its flash)
V5Sim attach 0              :: (re)connect instance 0's brain to Windows USB
V5Sim detach 0              :: take it off Windows USB, leave the sim running
```

`clear` empties the program storage (all 8 slots) but keeps the brain's settings and calibration; `reset` wipes the whole flash so the brain comes up factory-fresh on the next `start`. Both need the instance **stopped** first (they edit its flash image on disk) and default to instance 0. `clear`/`reset <id>` act on that instance.

`start` opens **no window** — the sim runs hidden and keeps running until you stop it. It prints the id it claimed (`--id auto`) and how to reach it:

```
Started V5 sim instance 0.
  field view : http://localhost:8765/
  live log   : v5sim log 0
  stop       : v5sim stop 0
```

`v5sim log <id>` tails that instance's serial console live (what used to scroll in the window); Ctrl-C stops the view, not the sim. Every instance always logs to `~/re/qemu-runs/console<id>.txt` in WSL, so `log` works however it was started.

`stop` is always a graceful QEMU quit, which lets the instance detach its USB/IP brain cleanly — it is the way to shut a background instance down. Don't `taskkill /F` the WSL/qemu processes or `wsl --shutdown` while a brain is attached (see the USB/IP note in section 5); if a sim is ever left orphaned, `v5sim list` still shows it and `v5sim stop <id>` cleans it up (QEMU keeps its control socket, so a graceful quit still works). The verbs delegate to `scripts/v5fleet.sh` in WSL.

### The field view (http://localhost:8765/)

- It shows the robot, its trail, any blocks and goals, the score, and the controller's screen.
- It's drawn in audience view, in the field coordinates VEXcode libraries use (RW-Template, mikLib, JAR): red wall on the left, audience at the bottom, +Y up, heading clockwise from +Y. The readout under the field gives x, y in inches and that heading.
- The **Override field** checkbox draws the V5RC Override (2026-27) starting layout under the robot, using the field figure from the game manual (v1.1, Figure FO-2, stored as `sim/v5sim/assets/override-field.png`). It's a picture only: the robot drives through it, and distance sensors still see only the walls.
- **Xbox controller** (or any gamepad the browser lists as standard): plug it in, click the page, then press any button on it so the browser picks it up. The line under the field names it. The sticks are the V5 sticks (left stick = Axis3/Axis4, right stick = Axis2/Axis1), A B X Y are the same letters, LB RB = L1 R1, LT RT = L2 R2, and the d-pad is the arrow buttons. Keyboard and gamepad work together.
- To drive, click the page first so it gets the keys:

  | key | controller |
  |---|---|
  | W / S | left stick up/down (Axis3) |
  | A / D | right stick left/right (Axis1) |
  | arrows | the other two axes |
  | J K U I | A B X Y |
  | Q E Z C | L1 R1 L2 R2 |
  | 1 2 3 4 | a hand swipe up / down / left / right in front of the Optical Sensors (gesture mode) |
  | T (hold) | press the Light Tower's button |

- The buttons under the field switch the competition mode: none, driver, autonomous or disabled.
- **Field control** (http://localhost:8765/field): a page laid out like the field controller app, with timed matches and skills runs (Start / Stop) and a manual mode. `--competition` opens it automatically; set `V5SIM_NO_BROWSER=1` to stop that.

### The brain's button

The V5 brain's front button works as on a real brain: a **tap** goes Back / Home, holding it **about 1 s** stops a running program, and holding it **3 to 5 s** with no program powers the brain off, which **closes the sim** (QEMU, the robot and the USB ports). Press it:
- in the brain window: the window shows a picture of the brain around its screen; click its **button**, right of the screen as on the real brain (it lights green while held), or hold the **B** key;
- in the field view: the **tap** button under the field, or on the dashboard: tap, hold 1.5 s, hold 5 s;
- from a script: `python3 scripts/v5ctl.py /tmp/v5qmp.sock button:1.5`, or post `{"brain_button": 1.5}` to the field view's `/input`.

### The brain screen in a browser (http://localhost:8765/screen)

The same screen as the brain window, as a web page: click or drag on it to touch, and the buttons under it press the brain's button (tap, hold 1.5 s, hold 5 s). It works with `V5SIM_HEADLESS=1` or `V5Sim start` too, where there is no window. The field view links to it. The robot sim takes frames from QEMU only while a page is open (up to 20 a second, `/screen.png`) and posts touches to it (`/touch`, `{"x": 0..479, "y": 0..271, "down": true|false}`, no `down` = a drag).

### In VS Code

The **V5 Brain Simulator** extension ([v5-simulacrum-vscode](https://github.com/UNSW-Robotics-Competition-Club/v5-simulacrum-vscode)) runs the sim from VS Code: `V5 Sim: Start` / `Stop` / `Restart`, the brain screen, device dashboard and field view as panes, and **Configure Ports**, a port map editor for the robot preset (pick the device on each smart port and 3-wire port, drag to move, Apply & restart). It runs `scripts/v5brain.sh` in WSL as `V5Sim` does (or directly on macOS), and stops it gracefully. Set `v5sim.repoPath` to this checkout if it isn't open in the workspace. See the extension's README.

### The device dashboard (http://localhost:8765/dashboard)

Every device on the robot, live: each motor's position, velocity, voltage, current, power, temperature, control mode and target, as the brain sees them, with rolling 15 s charts of velocity against its target and position against its target; and every sensor's reading (IMU heading, rotation, distance, optical, vision, ADI, controller). The field view links to it. The same data is at `/devices.json` for scripts.
- Scripts can post the same input as JSON to `/input`, for example `{"axes": {"Axis3": 80}}`, `{"gesture": "up"}`, `{"tower": true}`, or for a `--radio P:partner` second controller `{"partner": {"axes": {"Axis3": 60}, "buttons": ["A"]}}`. Use `Content-Type: application/json`.

---

## 2. One-time setup

You need all of the following on the PC:

- **WSL** with `Ubuntu-24.04`. Every command below runs there, apart from the Windows launcher and the VEXcode/PROS builds.
- **This repo**, cloned anywhere without spaces in the path (`git clone https://github.com/UNSW-Robotics-Competition-Club/v5-simulacrum.git`). It doesn't contain VEX's firmware: the setup script below downloads VEXos 1.1.5.0 from VEX's public server and unpacks it into `extracted/` (`scripts/fetch_firmware.sh`).
- **QEMU** with the VEX device models, the Arm GNU toolchain (`~/re/arm-gnu`, builds the test programs: `bash programs/test-programs/build.sh hello`) and the MuJoCo venv. One script does all three (it uses sudo for apt; the first QEMU build takes several minutes, later rebuilds about 20 s):

  ```sh
  # in WSL, from the repo
  bash scripts/setup.sh
  ```
- **VS Code with the VEXcode extension**, to build VEXcode C++ (section 3). Its toolchain is found automatically.
- **On a Mac** instead of WSL: macOS 13 or later on Apple Silicon or Intel, Apple's Command Line Tools (`xcode-select --install`) and [Homebrew](https://brew.sh). Clone the repo to a path without spaces and run the same `bash scripts/setup.sh` in Terminal: it installs the build tools and Python 3.12 with Homebrew (no sudo), downloads the firmware, builds QEMU (several minutes the first time), installs the macOS Arm GNU toolchain and makes `~/venvs/sim`. Then run the sim with `./v5sim` (section 1, "On macOS").
- **Optional:**
  - PROS: the VS Code PROS extension, to build PROS projects.
  - PROS CLI inside WSL, for `pros upload` and `pros terminal`:

    ```sh
    python3 -m venv ~/venvs/pros && ~/venvs/pros/bin/pip install pros-cli "setuptools<70"
    ```

---

## 3. Getting a program to run

### VEXcode C++

Build the project with VEXcode's own compiler, from Git Bash:

```sh
bash programs/test-programs/vexcode_build.sh path/to/project     # -> path/to/project/build/<name>.bin
```

This works for projects laid out as `src/`, `include/`, and also `custom/src`, `custom/include` (RW-Template). Then run it with `V5Sim <preset> path\to\project\build\<name>.bin`. You can also upload it over USB (section 5).

Projects whose sources sit deeper than `src/*/` (mikLib, for one) build with their own VEXcode makefile and VEXcode's `make`. `P=` names the output, which a folder name with a space needs:

```sh
VEX="$APPDATA/Code/User/globalStorage/vexrobotics.vexcode"; TC="$VEX/tools/cpp/toolchain_win32"
PATH="$TC/clang/bin:$TC/gcc/bin:$TC/tools/bin:$PATH" make.exe P=myrobot \
  T="$(cygpath -w "$VEX/sdk/cpp/V5/V5_20240802_15_00_00")"    # -> build/myrobot.bin
```

### VEXcode Python

Upload over USB with VEXcode's own uploader (section 5). The brain needs VEX's MicroPython VM once; after that, `.py` programs run like on a real brain.

### PROS 4

Build in Git Bash with PROS's toolchain on `PATH`, then upload with `pros upload` (section 5):

```sh
export PATH="$APPDATA/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin:$PATH"
cd my-pros-project && make.exe
```

The installed toolchain is GCC 13.3. If `make` complains about `gnu++26`, add these lines to the project's `Makefile` before `-include ./common.mk`:

```make
C_STANDARD=gnu2x
CXX_STANDARD=gnu++23
```

PROS builds two files (hot and cold), so use `pros upload` rather than `V5Sim ... program.bin`.

### Starting the program

The program starts automatically when you pass it to `V5Sim`, or with `--after run` / `--run` when uploading. Otherwise start it on the brain screen the usual way: Programs, pick the slot, Run.

---

## 4. The simulated robot

A **preset** is a text file of robot options in `sim/robots/`. Copy one to make your own. The shipped presets:

| preset | robot |
|---|---|
| `drivetrain` | 6-motor blue drive (1-3 left, 4-6 right), IMU 10, radio 11 |
| `competition` | drive 1-6, intake motor 7, optical 8, GPS 9, IMU 10, radio 11, tracking wheel 12, distance 13 (front), AI Vision 14 |
| `pushback-practice` | `competition` plus 24 red/blue blocks scattered on the field |
| `ui-test` | `competition` plugged into a field controller, starting disabled (`--competition field`): for testing pre-auton UI |
| `rw-template` | matches RW-Template's default `robot-config.cpp`: motors 1, 2 and 6 reversed in code, IMU 7, distance sensors 12-15 (front, left, right, back), intake 18, radio 21 |

The right-hand drive motors are mounted mirrored, as on a real robot, so programs set them **reversed**, like most VEXcode configs.

### Robot options

Use these in a preset file, or with `scripts/v5brain.sh --robot "..."` (section 6). `P` is a smart port, 1-21.

**Motors and drive**

| option | what it is |
|---|---|
| `--drive L:R:CART` | drivetrain: ports per side, e.g. `1,2,3:4,5,6:blue`. CART is `red`/`green`/`blue` (11 W cartridges) or `5.5w`. A port written `4r` means that motor is reversed in code, e.g. RW-Template's `1r,2r,3:4r,5r,6:blue` |
| `--motor P[r]:CART` | a motor on a light shaft (an intake, an arm with no load) |
| `--motor-fw P\|all` | that motor (repeatable, or every motor) runs VEX's own motor firmware instead of the sim's model: section "Real motor firmware" |
| `--arm P:RATIO:LEN:KG[:START:LO:HI]` | that `--motor` lifts an arm: gear RATIO (motor turns per arm turn), LEN inches to the load's centre, KG at its end, starting at START degrees (0 = level, default -90) between hard stops LO..HI (default -90..90). Gravity pulls it down, so holding it level costs current, and a motor that stops driving lets it fall to its stop |
| `--intake P[:CAP]` | that `--motor` is a front intake: forward captures blocks, up to CAP (default 6); reverse ejects them |
| `--holonomic x\|mecanum` | the `--drive` is a four-motor X-drive (omni wheels at 45°) or mecanum drive: it strafes and turns like mikLib/VEXcode drive it. List each side front then back, e.g. `--drive 14r,13r:12r,11r:green --holonomic x`. With the builtin physics walls stop it but its wheels don't slip; with MuJoCo its rollers and traction are modelled too |
| `--wheel-diameter IN`, `--track IN` | drive geometry (default 3.25 in wheels, 12 in between the sides; a holonomic frame is square) |
| `--wheel-rpm RPM` | the drive's gearing, as the wheels' free speed (VEXcode/mikLib "drivetrain RPM"), e.g. green geared up to `333.333333`. Default: 3/4 of the cartridge (blue 600 -> 450) |
| `--start X:Y:H` | start position in RW-Template field coordinates: inches from the field centre, heading clockwise from +Y. Write it as `--start=-36:-60:0` when it begins with a minus sign |

The 5.5 W motor (`5.5w`) has a fixed 200 rpm output and about half the torque of an 11 W motor. VEXcode's `getMotorType()` reports it as type 1.

**Sensors**

| option | what it is |
|---|---|
| `--imu P` | Inertial Sensor: heading clockwise, 2 s calibration |
| `--rotation P[r]:track[:D[:X:Y]]` | Rotation Sensor on a tracking wheel rolling forward: diameter D (default 2.75 in), touching the floor X, Y inches from the turning centre (right, forward; default 0, 0). `r` = it reads the other way round. E.g. mikLib's forward tracker 2.803 in left: `8:track:2:-2.803:0` |
| `--rotation P[r]:strafe[:D[:X:Y]]` | the same on a sideways tracking wheel (positive = robot moving right). mikLib's "2.276 in behind" with diameter -2: `7r:strafe:2:0:-2.276` |
| `--rotation P:motorN` | Rotation Sensor on the shaft of motor N |
| `--distance P[:FACE[:X:Y]]` | Distance Sensor. FACE is `front`/`right`/`back`/`left` or degrees clockwise; X, Y is its offset in inches (right, forward), RW-Template's convention. E.g. `--distance 15:back:0:-6`. Readings jitter a few mm like a real sensor; `--no-noise` makes them exact |
| `--optical P` | Optical Sensor at the intake: sees the block it holds or the one right in front (red ~hue 0, blue ~225). In gesture mode it reports the swipes from the field view's keys 1-4 |
| `--gps P` | GPS Sensor: the robot's x, y (m from the field centre) and heading |
| `--aivision P` | AI Vision Sensor at the front: colour objects for VEXcode `colordesc` signatures, colour codes (`codedesc`) for touching blobs in the code's order, `Cube(Red/Blue)` objects with model detection on, and AprilTags from `--tags` when tag detection is on |
| `--vision P` | the older Vision Sensor at the front (316x212 image). It reports blocks matching its signatures; signatures copied from the Vision Utility work, e.g. a red one around u 9000..11000, v -2100..-1000 |
| `--adi P:A=KIND,...` | 3-wire ports A-H: `22` is the brain's own, any other port is a 3-Wire Expander. KIND: `bumper` or `limit` (front, pressed at a wall), `encoder` (tracking wheel), `sonar` (front), `piston` (digital out), `gyro` (robot heading, calibrates for 1 s), `pot:N` or `potv2:N` (a potentiometer on motor N's shaft), `line` (floor, 30 % reflective), `light` (room light, 50 %), `accel` (forward acceleration). `encoder` and `sonar` take two ports, so use A, C, E or G. E.g. `--adi 22:A=bumper,C=piston,E=gyro,F=pot:7`. Legacy PWM motors and addressable LED strips on digital-out ports are shown in the status line |
| `--controller P` | a V5 Controller, tethered on port P. It's driven from the field view; its 3-line screen and rumble show there too |
| `--radio P[:partner]` | a VEXnet robot radio on port P with a wireless controller, driven like `--controller` (use one or the other). `:partner` adds a second controller. With `--match`, the match runs as field control and the brain's header shows the match timer |

**Other devices**

| option | what it is |
|---|---|
| `--magnet P` | V5 Electromagnet: power, timeout, current and temperature, as `vexMagnetPickup`/`Drop` set them |
| `--tower P` | Light Tower (`cte::signaltower`): its five lights and blinking show in the status line. Hold T in the field view to press its button |
| `--pneumatic P` | CTE pneumatics (`cte::pneumatic`): pump, four cylinders, actuation counters and a simple pressure model |
| `--led P` | smart-port LED (`vexLedRgbSet`): its colour shows in the status line |

**Field, matches and display**

| option | what it is |
|---|---|
| `--blocks random:N[:SEED]` or `FILE.json` | game blocks: N scattered, or a list of `[x, y, "red"|"blue"]` in metres from the centre |
| `--goals FILE.json` | scoring zones for the blocks: `[{"name": "...", "rect": [x0, y0, x1, y1], "block_points": 3, "control_points": 10}]`. The score shows on the field view. No official field layout is built in, so write one for your field |
| `--tags FILE.json` | AprilTags on the field for `--aivision`: `[{"id": 3, "x": 1.78, "y": 0, "facing": 180}]` in metres from the centre, `facing` = the way the tag's face points, degrees counter-clockwise from +x. Optional `"z"` (centre height, m, default 0.2), `"size"` (m, default 0.1016) and `"family"` (`circle21h7`, the sensor's default, `tag16h5`, `tag25h9`, `tag36h11`) |
| `--competition field\|switch` | plug into competition control (needs `--controller` or `--radio`), disabled until the field view's buttons pick autonomous or driver. `field` reports a field controller over the radio, `switch` a competition switch. Not with `--match` |
| `--match START[:AUTON:DRIVER]` | a timed match (needs `--controller` or `--radio`): disabled until START seconds, then autonomous (default 15 s), 2 s disabled, driver control (default 105 s), disabled. `--match 30:0:60` is a 60 s driver-skills run |
| `--view PORT` | the field view (8765 in the presets) |
| `--physics builtin\|mujoco` | the physics engine (section "What the physics does") |
| `--obstacles FILE.json` | MuJoCo only: fixed field elements the robot and blocks hit, a list of `{"kind": "box", "x", "y", "sx", "sy", "angle"}` (half sizes, degrees anticlockwise) or `{"kind": "cylinder", "x", "y", "r"}`, metres from the centre like `--blocks`. Drawn grey on the field view |
| `--opponent X:Y:H[:braked]` | MuJoCo only, repeatable: another robot (18 in, 7 kg) placed like `--start`. A `braked` one doesn't move until pushed harder than its friction (a 6.5 kg robot of the same tiles can't, but a run-up can shove it); otherwise it rolls along its heading and grips sideways. Drawn purple on the field view |
| `--battery PCT`, `--no-battery` | the robot battery, starting at PCT % (default 100): its voltage sags under load and as it drains, and no motor gets more than it gives. The brain's header and `vexcom`/PROS status show its charge. `--no-battery` is an ideal 12 V supply |
| `--traction MU` | wheel-tile friction, default 0.8: where the wheels slip at walls and, with MuJoCo, at hard launches and stops |
| `--status S` | print the robot's state every S seconds (to `~/re/qemu-runs/robot.txt` when started by the scripts) |

The status line shows each device and the pose, also in RW-Template coordinates as `rw=(x, y, heading)`.

### What the physics does

- The field is 140.5 in square inside the walls, with walls at ±70.25 in. 
- Driving into a wall stops the robot. With enough power the wheels **slip and keep spinning**, so encoder odometry drifts the way it does on a real field. That's what distance resets and MCL are for.
- Blocks are pushed out of the robot's way, picked up by the intake, ejected and scored. They don't push each other or slow the robot.

**MuJoCo physics** (`--physics mujoco`): the same robot in a [MuJoCo](https://mujoco.org) world with real contact. Blocks push each other and the robot, slide with friction, and pile against walls. The wheels have traction (μ 0.8), so a full-power launch spins them and acceleration tops out near 7.8 m/s², as on tiles. On open ground it agrees with the default physics within a few percent. It needs the `mujoco` Python package; `scripts/v5brain.sh` and `V5Sim` use `~/venvs/sim` automatically when it exists:

```sh
python3 -m venv --system-site-packages ~/venvs/sim && ~/venvs/sim/bin/pip install mujoco
```

Any check or preset can run on it with `V5SIM_ROBOT_EXTRA="--physics mujoco"`. MuJoCo also gives you field elements and opponent robots (`--obstacles`, `--opponent`): `scripts/opponent_test.sh` drives the intake program into a braked opponent and checks that it stops at its face.

**Gearbox friction**: every motor's gearbox has friction like a real one. A free-spinning motor draws about 0.12 A just to turn, a stopped one needs about 1 V before it moves, and a holding motor sits still instead of dithering. The free speeds and stall torques still match VEX's figures. The friction values are estimates until they're measured on a real motor.

**Motors heat up and derate**: past 55 °C a motor's current limit halves every 5 °C, down to nothing at 70 °C, as VEX describes.

**Real motor firmware** (`--motor-fw 7`, or `--motor-fw all`): normally each motor is a Python model of the motor's firmware. With this option the motor on that port runs VEX's own motor firmware (the 11 W or 5.5 W image from VEXos) in a small emulator of its own. Its real velocity and position PID, current limit and faults then drive the physics, so tuning that depends on how the motor's own controller reacts behaves more like a real robot. Each emulated motor is one extra QEMU process using about a third of a CPU core, so a six-motor drive costs about two cores; start-up adds about a second. The 5.5 W firmware caps its drive at 60 % of the battery, as a real 5.5 W motor does. Velocity readings jitter by about 1 %, a little like a real encoder.

**Battery**: the robot runs off a modelled V5 battery (LiFePO4, 1.1 Ah), so a hard launch sags the voltage and a long run drains it. The brain sees it as a real pack: the header icon and the percent in `vexcom`/PROS status follow it.

---

## 5. Uploading and the terminal (USB)

The brain's USB port is emulated (`V5Sim` always enables it).

### On Windows: VEXcode and PROS, like a plugged-in brain

With **usbip-win2** installed once (`winget install vadimgrn.usbip-win2`; signed drivers, it briefly restarts the USB hubs), `V5Sim` plugs the simulated brain into the PC's USB: it shows up as a real V5 brain (USB 2888:0501) with two COM ports, the Communications port and the User port. Windows' own serial driver runs them, so no VEX driver is needed. Then:

- **VEXcode** (VS Code extension or IDE) finds the brain: Download, Run and the terminal work as with a real one.
- **PROS** finds it without a port: `pros upload`, `pros v5 status`, `pros terminal`. With several brains on USB (a sim and a real brain or controller), `pros upload` crashes in its own "Multiple V5 Brains" prompt (`'NoneType' object has no attribute 'split'`; pyserial on Windows has no USB product name). Use **`V5Upload`** from the repo root instead, in the PROS project folder: `V5Upload [path] [pros upload options]`, e.g. `V5Upload --slot 2 --after run`. It lists the brains, naming each sim by instance and field view (each sim reports USB serial number `V5SIM<N>`), asks which one, and runs `pros upload <path> COMn ...`. Or give pros the port yourself: `pros upload . COMn` (the port is positional, after the path). `pros terminal` is worse off: with more than one V5 device it finds no port and prints nothing. Use **`V5Term`** (`V5Term [pros terminal options]`), which asks the same way and runs `pros terminal` on that device's User port; open it before starting the program to see the PROS banner. Both skip duplicate USB/IP attaches of a sim (devices on usbip-win2's host controller without a `V5SIM<N>` serial; Windows drops a serial it has already seen), which otherwise show up as extra "V5 brain" COM pairs until a Windows restart. Only one sim is attached to Windows USB at a time (a second attach can crash the usbip-win2 driver), so later sims use the WSL ttys below; the serial numbers keep each sim on its own COM ports across restarts.
- `vexcom.exe COMn ...` works too when given the Communications port. Its own port scan looks for VEX's driver names, so name the port.

The brain unplugs itself when the sim closes. Under the hood, `v5brain.sh --usbip` serves the brain's USB as a USB/IP device on `localhost:3300+N` (`scripts/v5usb_usbip.py`) and runs `usbip.exe attach --once` / `detach`. With usbip-win2 installed, the WSL ttys below aren't there; without it, `V5Sim` falls back to them.

> The port was moved off the standard USB/IP port 3240 because **usbipd-win** (the separate dorssel/usbipd-win service for sharing real USB devices into WSL) listens there on Windows; if the sim used 3240, the Windows `usbip.exe` client would reach usbipd-win instead of the emulated brain and fail with *"Device not found by bus id."* Set `V5SIM_USBIP_BASE` if 3300+ is also taken.

> **One USB/IP brain at a time, and shut down gracefully.** The usbip-win2 0.9.7.8 kernel driver can bugcheck Windows (BSOD) if a second brain is attached while one is already on Windows USB, or if a brain is torn down ungracefully (killing QEMU/WSL, or `usbip.exe detach --stop-all`). So: `V5Sim` attaches only one brain over USB/IP — if one is already attached, a second instance uses the WSL pty bridge instead and says so; always stop with `V5Sim stop <id>` (graceful QEMU quit) or by closing the window, never by killing the processes. Set `V5SIM_NO_USBIP=1` to force the pty bridge and skip the Windows attach entirely.

### In WSL

With `--usb` (or `V5Sim` without usbip-win2) it appears as two serial ports:

| port | what it's for |
|---|---|
| `/tmp/ttyV5sys` | uploads and brain info (pros upload, vexcom) |
| `/tmp/ttyV5user` | your program's `printf` / `print` output and keyboard input |

**PROS CLI** (always give the port explicitly):

```sh
~/venvs/pros/bin/pros upload --slot 2 --after run . /tmp/ttyV5sys   # in a PROS project
~/venvs/pros/bin/pros upload --target v5 --slot 3 --after run prog.bin /tmp/ttyV5sys
~/venvs/pros/bin/pros terminal /tmp/ttyV5user                       # program output
~/venvs/pros/bin/pros v5 status /tmp/ttyV5sys
```

**VEXcode's uploader** (`vexcom`, the Linux build that comes with the VEXcode extension; `scripts/vexcom_test.sh` copies it to `~/re/tools/vexcom` the first time):

```sh
V=~/re/tools/vexcom
$V /tmp/ttyV5sys --python                                # once: install the Python VM (port first)
$V --slot 2 --write build/prog.bin --run /tmp/ttyV5sys   # a VEXcode C++ program
$V --slot 7 --write main.py --run /tmp/ttyV5sys          # a VEXcode Python program
$V --json /tmp/ttyV5sys                                  # brain info and files
```

To watch program output without PROS, open `/tmp/ttyV5user` in any serial terminal, e.g. `screen /tmp/ttyV5user`.

Uploaded programs stay on the simulated brain between runs, like on a real one. `V5Sim clear` empties all 8 program slots (keeping settings); `V5Sim reset` factory-resets the brain (equivalent to deleting `~/re/qemu-runs/qspi.img`). A program passed to `V5Sim` runs on a fresh copy and doesn't disturb uploaded ones.

The VEXcode IDE's Download button needs the brain on Windows' USB: see "On Windows" above.

### On macOS

`./v5sim` exposes the brain's USB as the same two serial ports, `/tmp/ttyV5sys` and `/tmp/ttyV5user` (`/tmp/v5simN/...` for instance N). macOS can't plug a virtual USB device in, so **VEXcode's Download button doesn't see the sim**. Get programs onto it one of these ways:

- **As you start it:** `./v5sim competition build/prog.bin` installs the program in slot 1 and runs it. This works for VEXcode C++ builds and any single-file `.bin`.
- **From VS Code:** the extension's **V5 Sim: Run Program on Sim** does the same for a running sim, and in a PROS project it runs `pros upload` to the sim's port for you.
- **PROS CLI** (`pip install pros-cli`, or the PROS VS Code extension's CLI), with the port given, as in WSL above: `pros upload --slot 2 --after run . /tmp/ttyV5sys`, `pros terminal /tmp/ttyV5user`.
- **VEXcode's uploader**: the VEXcode VS Code extension downloads a `vexcom` for each platform under `~/Library/Application Support/Code/User/globalStorage/vexrobotics.vexcode/tools/vexcom/<version>/` (build a project once first); use the macOS one. Use it like the WSL one above, port last: `vexcom --slot 2 --write build/prog.bin --run /tmp/ttyV5sys`.

Program output: `pros terminal /tmp/ttyV5user`, or `screen /tmp/ttyV5user` (Ctrl-A K quits).

### Debugging with GDB (vexide/v5gdb)

The sim models the Cortex-A9 debug hardware that [v5gdb](https://github.com/vexide/v5gdb) uses: a debugger built into a vexide (Rust) program that serves GDB over the brain's User port. Attach, software breakpoints (they fire again on each `continue`), `stepi`, Ctrl-C into a running program, registers, memory, backtraces and `detach` all work as on a real brain.

1. Build the program with v5gdb. vexide emits the V5 header, so `arm-none-eabi-objcopy -O binary` turns the ELF into a `.bin`. Keep the ELF: GDB reads its symbols from it.
2. Run it: `V5Sim <preset> program.bin`.
3. Connect v5gdb's host CLI to the User port. It sends GDB's bytes in and splits the brain's replies (COBS frames tagged `u` user output, `d` GDB, `l` log) out to `arm-none-eabi-gdb` on TCP 35537.

On the WSL pty bridge (`/tmp/ttyV5user`), GDB was tested through a small serial↔TCP bridge that does the CLI's job. It isn't in the repo yet. The CLI itself finds the brain by its USB IDs, so it needs the brain on Windows USB (usbip-win2). That route should work like a real brain but hasn't been tested yet.

GDB hardware breakpoints (`hbreak`) may be refused. Plain `break` uses software breakpoints in RAM, and those work. The debug support is QEMU patch `qemu-vex/patches/0006-a9-debug-rom-mmio.patch`. A clone set up before it was added needs a QEMU rebuild: `bash scripts/qemu_install_vex.sh` in WSL.

---

## 6. Running from WSL directly

`V5Sim.cmd` wraps `scripts/v5brain.sh`, which you can use directly in WSL:

```sh
bash scripts/v5brain.sh \
  --run /mnt/c/.../program.bin --usb \
  --robot "@robots/competition.args --match 30 --blocks random:10"
```

| option | |
|---|---|
| `--run FILE` | install FILE (a V5 `.bin`) and start it after boot |
| `--slot N` | the slot for `--run` (1-8, default 1) |
| `--robot "ARGS"` | robot options (section 4); `@robots/NAME.args` pulls in a preset |
| `--usb` | the USB ports (section 5) |
| `--headless` | no window. Take screenshots with `python3 scripts/v5ctl.py /tmp/v5qmp.sock shot:file.ppm`, and convert them with `scripts/ppm2png.py` |
| `--run-after S` | seconds to wait before tapping Programs > slot > Run (default 0.5; env `V5SIM_RUN_AFTER`). The taps wait for the Home screen and for each screen they open, so a short value is safe |

### Automated runs: run, score, compare (`scripts/v5run.py`)

To iterate on an autonomous routine without watching it, `v5run.py` boots a headless brain, runs the program and reports how it ended, as JSON: the pose (also in RW-Template inches and degrees), the score, blocks held and on the field, opponents, the path, and each motor's temperature and faults plus the battery. It stops `--seconds` after the program starts (default 15, an autonomous period), or as soon as the robot and every motor have been still for `--settle` seconds. A full run takes about 10 s.

```sh
python3 scripts/v5run.py programs/test-programs/vexcode_intake/build/vexcode_intake.bin   --robot "--drive 1,2,3:4,5,6:blue --motor 7:green --intake 7:4 --blocks programs/test-programs/vexcode_intake/blocks.json --goals programs/test-programs/vexcode_intake/goals.json --physics mujoco"   --seconds 20 --settle 2 --expect 3:37.6:197 --tol 3:10 --min-score 10 --json result.json
```

`--expect X:Y:H` (RW coordinates; `--tol IN:DEG`, default 2 in and 5 degrees) and `--min-score N` make it a check: the exit code is 1 if the robot ends somewhere else or scores less. With `--batch RUNS.json` it runs a list of these, `--jobs N` at a time on separate simulator instances (each brain takes about one CPU core), and prints one line per run; `scripts/v5run_example.json` runs the intake program on both physics engines side by side. In a batch file, paths in `program` and `robot` are relative to the file. Each run's console is in `~/re/qemu-runs/v5run_NAME.log`.

### Field control: the VEX field app and Tournament Manager

One sim brain can run VEX's V5RC field-control app (`field_1_0_3_2.bin`) and be driven by Tournament Manager over USB, like a real field controller. Robots join it through their controllers, smart-cabled to the field brain's ports: 1-5 red 1, 6-10 blue 1, 11-15 blue 2, 16-20 red 2.

1. **Field brain:** `v5sim start competition`, then install and run the app. Its file must go up with CDC2 FILE_INIT exttype 115, which vexcom/PROS can't set (VEX's web installer does; a small pyserial uploader that does is not in the repo yet). Connect TM to the brain's Communications port.
2. **Robot:** start a second instance with no simulated radio, so the controller below can be its radio, e.g. `v5sim start fieldbot C:\full\path\to\program.bin` (an absolute program path) with `V5SIM_NO_USBIP=1` (only one sim at a time goes on Windows USB).
3. **The robot's controller**, cabled to the field and radio-linked to the robot (from WSL, in `sim/`):
   ```sh
   python -m v5sim.field_link /tmp/v5port15.sock /tmp/v5sim1/v5port11.sock --team 1234A
   ```
   Field brain port 15 (blue 2) to the robot's port 11. The station shows the team, the robot's battery and program, and its square turns green once the robot runs a program. As on a real controller (DEVFRM09), the app's `56` requests (the robot's VEXos version, the clock sync) go to the robot brain over the radio and its answers come back, a refusal included; the controller answers its own `58` queries, i.e. its radio version (1.0.0b48, the radio firmware in the VEXos bundle, DEVFRM08 v0x30). `--trace` prints that traffic. Driver / Auton / Disable from TM or the field screen reach the robot as field control (its program's `Competition` callbacks run).

A controller alone, with no robot behind it (it reports a robot linked and running slot 1): `python -m v5sim.robot --no-builtin-adi --no-battery --field-controller 15[:TEAM] --sock '/tmp/v5port{port}.sock'`, repeatable for more stations. `--no-builtin-adi` lets this second plant leave port 22 to the brain's own robot.

---

## 7. Not supported yet / limits

- **Devices:** every V5 smart-port device has a model. What they sense is simplified:
  - Cameras see only blocks and `--tags`, never walls, goals or other robots. The legacy Vision Sensor's colour maths is a best guess fitted to a Vision Utility signature. It has no colour codes or line modes.
  - The electromagnet's current and temperature, and the pneumatics' pressure, are simple models; the magnet picks nothing up.
  - The field view only sends the radio's competition through `--match` or its buttons. A real field link (field-control app + TM) goes through `v5sim.field_link` (section 6, "Field control"); its controller has no sticks (the robot's driver control gets centred sticks). The radio link's RSSI and signal are a model (`v5sim/radio_signal.py`: about -55 dBm, wandering a few dB, signal 90-100 %) shared by the field's station panel and the robot's radio, on competition channel 81; the link never drops.
  - Addressable LED strips are decoded but only shown in the status line. VEXcode has no class for them; PROS's `pros::adi::Led` drives them.
- **Match timer:** with `--radio` and `--match` the brain's header timer counts each phase. With a tethered `--controller` it stays at 0:00, because the sim's tether sends no seconds counter.
- **Physics:**
  - Physics is a flat, simple drivetrain model. It has no robot-to-robot or block-to-block contact.
  - Goals are floor zones, not obstacles.
  - The GPS heading convention is a best guess (0 = facing +y).
- **Debugging:** v5gdb works with software breakpoints and single-step. GDB hardware breakpoints (`hbreak`) and watchpoints aren't verified. There is no JTAG.
- **macOS:** no Windows-style USB (VEXcode's Download button), see section 5. With `--motor-fw`, the motor firmware's velocity readings may jitter a little more than on Linux, which can make its timers fire later (Linux's timer-slack setting has no macOS equivalent).
- **Speed:** the simulator runs close to real time. A busy PC can slow it slightly. While v5gdb single-steps, the program runs one instruction per step and so runs much slower.

---

## 8. Troubleshooting

| problem | try |
|---|---|
| Nothing happens after starting | VEXos boots in ~2 s; with the brain window, check it reached the Home screen. The end-to-end scripts deliberately start programs ~24 s in (`V5SIM_RUN_AFTER=22`) |
| `pros upload` fails with `'NoneType' object has no attribute 'split'` | several V5 devices on USB (e.g. sim + real brain): use `V5Upload` (section 5) or `pros upload . COMn` |
| `pros terminal` shows nothing | several V5 devices on USB (`pros lsusb` lists no ports): use `V5Term` (section 5) or `pros terminal COMn` with the User port |
| `pros upload` can't find the brain | give the port: `... /tmp/ttyV5sys`, and check that `--usb` / `V5Sim` is running |
| VSCode / Windows PROS see no brain, no COM ports (usbip) | a Windows USB/IP service (usbipd-win) on port 3240 collided with the sim; fixed by moving the sim to 3300+. If it still clashes, set `V5SIM_USBIP_BASE` to a free base. Check `usbip.exe port` shows a 2888:0501 device |
| `vexcom --python` says "Device not specified" | put the port first: `vexcom /tmp/ttyV5sys --python` |
| The robot doesn't move in autonomous | autonomous only runs in a match: use `--match`, or the field view's competition buttons |
| A routine ends in the wrong place | if it starts with `setPose()`, start the robot there with `--start=X:Y:H` |
| The robot drives backwards or spins in place | motor reversal: in the preset, add or remove an `r` on a drive port (`4r`) to match the program's `reversed` settings |
| Odd behaviour after many uploads | `V5Sim clear` (wipe programs) or `V5Sim reset` (factory reset), with the instance stopped |
| Check what the robot sees | `~/re/qemu-runs/robot.txt` has the status line every second |
