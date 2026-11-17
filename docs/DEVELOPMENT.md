# V5 simulator: development notes

User guide: [`SIMULATOR.md`](../SIMULATOR.md).

Code comments cite `notes/*.md`. Those are the maintainers' protocol notes, which aren't in this repo.

## Layout

| path | what |
|---|---|
| `qemu-vex/hw/...` | our QEMU device models (machine `vex-v5`, smart ports, USB device, display, touch, NXP, I2C, AXI blocks), symlinked into `~/re/qemu` (tag v11.1.2) by `scripts/qemu_install_vex.sh`; upstream fixes in `qemu-vex/patches/` |
| `sim/v5sim/` | the robot: `robot.py` (runner/CLI), `smart_device.py` (shared link layer), one module per device (`smart_motor`, `smart_imu`, `smart_sensors`, `smart_optical`, `smart_gps`, `smart_aivision` + `apriltags`, `smart_vision`, `smart_adi` + `adi_world`, `smart_controller`, `smart_radio`, `smart_misc` for the electromagnet, light tower, pneumatics and LED), physics (`motor`, `drivetrain`, `field_objects`, `scoring`, `match`; `physics.py` is the backend contract, `physics_mujoco.py` the MuJoCo backend), `fieldview.py` (+ `dashboard.py`, the `/dashboard` page; `brainctl.py` presses the brain's button over QMP; `brainscreen.py` serves the `/screen` page: frames and touch over its own QMP socket, `v5qmp-screen.sock`) |
| `sim/robots/*.args` | robot presets (robot-sim `@FILE` argument files) |
| `sim/tests/` | unit tests: physics, and every device frame checked byte for byte against reference frames captured from real devices |
| `scripts/v5brain.sh` | main entry: QEMU + robot sim + USB pty bridge (`scripts/v5usb_pty.py`) |
| `V5Sim.cmd` | Windows launcher around `v5brain.sh` |
| `programs/` | test programs and build scripts (`vexcode_build.sh` for VEXcode C++, `build.sh` for plain C) |

## Tests

The MuJoCo tests need the `~/venvs/sim` venv (`python3 -m venv --system-site-packages ~/venvs/sim && ~/venvs/sim/bin/pip install mujoco`); with the system python3 they are skipped.

```sh
cd sim && ~/venvs/sim/bin/python -m pytest -q -p no:cacheprovider tests      # unit tests (~8 s)
bash scripts/e2e_all.sh [names...]                                              # end-to-end checks (~15 min)
```

Each end-to-end check boots VEXos, runs a real program and prints what it saw (`scripts/*_test.sh`):

| check | shows |
|---|---|
| `smoke_run` | `--run` + one motor |
| `ui_test` | `--competition field` (the `ui-test` preset) with the `vexcode_ui` template: starts disabled, a brain-screen tap picks a routine, the field-control page's START runs it |
| `button_test` | the brain's front button: a 1.5 s hold through `/input` stops the program, the B key goes Back; `/devices.json` and `/dashboard` serve |
| `motor55_test` | 5.5 W motor: type 1, 100 rpm, `spinToPosition(720)` |
| `misc_test` | electromagnet, light tower, CTE pneumatics, LED read back through the SDK; tower button and optical gestures from `/input` |
| `adi_extra_test` | 3-wire gyro (vs the robot's heading), pot on a motor shaft, line, light, accelerometer |
| `vexcode_test`, `imu_test` | VEXcode timed drive; IMU turn to 90° |
| `driver_test` | driver control from the field view's input, controller screen + rumble |
| `comp_test` | VEXcode competition template through `--match` |
| `radio_test` | the same over a VEXnet radio as field control (status a9, 91, 99, a1, a9) |
| `adi_test` | 3-wire bumper, pneumatics, legacy encoder, sonar |
| `intake_test` | blocks: intake 3, plow 1, eject into a goal (score) |
| `sensors_test`, `aivision_test` | optical + GPS; AI Vision colour signature aim-and-collect |
| `aitags_test`, `vision_test` | AI Vision AprilTag + colour code; legacy Vision Sensor signature aim |
| `usb_test`, `vexcom_test` | PROS CLI status/upload/terminal, VEXcode `vexcom` + Python |

The VEXcode checks build their programs with `programs/test-programs/vexcode_build.sh`, which uses the toolchain and SDK from your VEXcode VS Code extension (Git Bash on Windows).

## Rebuild QEMU

```sh
wsl -d Ubuntu-24.04 -- bash scripts/qemu_install_vex.sh      # from the repo root
```

It links the sources, applies the patches and builds incrementally (about 20 s). Once a patch is applied in `~/re/qemu`, editing it doesn't reach the tree: undo the old version first (`git apply --reverse` with `git show HEAD~:qemu-vex/patches/NAME.patch`), then rerun the script.

Patch `0006-a9-debug-rom-mmio.patch` is the vexide/v5gdb support:
- A9 `DBGDRAR`/`DBGDSAR` point at a per-core debug bank, mapped in `vex_v5.c` at 0xF8890000 / 0xF8892000.
- The bank routes `DBGBVR`/`DBGBCR`/`DBGWVR`/`DBGWCR`/`DBGDSCRext` (MDE only) to `env->cp15` and `hw_breakpoint_update`.
- ARMv7 address-mismatch breakpoints (BCR.BT=0b0100), which upstream QEMU lacks, run as per-instruction single-step: TB flag `VEX_MISMATCH_STEP` plus `helper_vex_mismatch_check`.
- Single-step is only active in USR/SVC/SYS mode, so v5gdb's own abort handler isn't stepped.

## Debugging tools

- `--trace P` (robot sim): prints the brain's frames on port P to `robot.txt`.
- `V5USB_TRACE=FILE`: the USB pty bridge logs every chunk. View it with `python3 scripts/usbtrace_view.py FILE [SECTION] [--user|--sys]`. `e2e_all.sh` keeps one per run in `/tmp/v5usb_e2e_trace.txt`.
- QEMU: `-- -d exec,cpu,nochain -dfilter ADDR+LEN -D file`, appended after `v5brain.sh`'s options, logs register state at each translation block in a function. That shows callee return values without gdb.
- QMP: `scripts/v5ctl.py /tmp/v5qmp.sock` takes `shot:file.ppm`, `tap:x,y`, `run:N`, `hmp:...`, `quit`.

## Adding a device

1. Work out the device's smart-port protocol and capture reference frames and CRCs.
2. Write a `SmartDevice` subclass, with tests that match those frames byte for byte.
3. Give it a world model, i.e. what it senses, in `robot.py`.
4. Add a robot option, a VEXcode e2e program and a check script.

## Adding a physics backend

Implement `physics.PhysicsWorld` (`step`, `field`, `pose`) and construct it in `Robot.use_<name>()`, like `use_mujoco()`. Step the engine in lock-step with the robot sim's 1 ms tick; the robot sim's clock follows QEMU's real time. Test it against the builtin model on open ground (`tests/test_physics_mujoco.py` shows how, with the wheel inertia folded in), then run the drive checks with `V5SIM_ROBOT_EXTRA="--physics <name>"`.
