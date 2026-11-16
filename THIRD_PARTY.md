# Third-party code and material

| what | where | license |
|---|---|---|
| QEMU v11.1.2 (cloned and built by `scripts/setup.sh`, not vendored) | `~/re/qemu` | GPL-2.0 |
| Device models forked from QEMU (`xilinx_zynq.c` and others) | `qemu-vex/hw/` | GPL-2.0-or-later |
| Patches to QEMU | `qemu-vex/patches/` | GPL-2.0-or-later |
| vexide vex-sdk (types, docs and signatures the C SDK was generated from) | `sdk/` | MIT (`sdk/LICENSE`) |
| Arm GNU Toolchain 14.2.rel1 (downloaded by `setup.sh`, not vendored) | `~/re/arm-gnu` | GPL / various |
| MuJoCo (installed with pip by `setup.sh`, not vendored) | `~/venvs/sim` | Apache-2.0 |

## Not included

- **VEXos firmware.** `scripts/fetch_firmware.sh` downloads VEXos 1.1.5.0 from VEX's public download server into `extracted/`, which git ignores. VEX's firmware is VEX's property; don't commit it.
- **VEXcode SDK and toolchain.** `programs/test-programs/vexcode_build.sh` uses the copy installed by the VEXcode VS Code extension.

## Other material

- `sim/v5sim/assets/override-field.png` is drawn from the field diagrams in VEX's published game manual.
- `sim/skins/v5-brain.ppm` is drawn by `sim/skins/make_brain_skin.py`.
- VEX, V5, VEXos and VEXcode are trademarks of Innovation First, Inc. This project isn't affiliated with or endorsed by VEX Robotics.
