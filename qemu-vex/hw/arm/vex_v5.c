/*
 * VEX V5 Robot Brain (Zynq-7010) emulation.
 *
 * Forked from QEMU's hw/arm/xilinx_zynq.c:
 *   Copyright (c) 2010 Xilinx.
 *   Copyright (c) 2012 Peter A.G. Crosthwaite (peter.crosthwaite@petalogix.com)
 *   Copyright (c) 2012 Petalogix Pty Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Adds the PS I2C0 controller with the
 * brain's NXP co-processor, and logs the PL (FPGA) address space.
 *
 * Copyright (c) 2010 Xilinx.
 * Copyright (c) 2012 Peter A.G. Crosthwaite (peter.croshtwaite@petalogix.com)
 * Copyright (c) 2012 Petalogix Pty Ltd.
 * Written by Haibing Ma
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "net/net.h"
#include "system/system.h"
#include "hw/core/boards.h"
#include "hw/block/flash.h"
#include "hw/core/loader.h"
#include "hw/adc/zynq-xadc.h"
#include "hw/ssi/ssi.h"
#include "hw/usb/chipidea.h"
#include "qemu/error-report.h"
#include "hw/sd/sdhci.h"
#include "hw/char/cadence_uart.h"
#include "hw/net/cadence_gem.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "system/reset.h"
#include "qom/object.h"
#include "exec/tswap.h"
#include "target/arm/cpu-qom.h"
#include "qapi/visitor.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "chardev/char.h"
#include "qemu/timer.h"
#include "target/arm/arm-powerctl.h"

#define TYPE_VEX_V5_MACHINE MACHINE_TYPE_NAME("vex-v5")
OBJECT_DECLARE_SIMPLE_TYPE(VexV5MachineState, VEX_V5_MACHINE)

#define ZYNQ_MAX_CPUS 2

struct VexV5MachineState {
    MachineState parent;
    Clock *ps_clk;
    ARMCPU *cpu[ZYNQ_MAX_CPUS];
    uint8_t boot_mode;

    MemoryRegion *slcr_mr;
    MemoryRegion cpu_rst;       /* SLCR A9_CPU_RST_CTRL overlay */
    QEMUTimer *launch_timer;
    uint32_t launch_ms;         /* direct user-program launch, 0 = off */
    char *skin;                 /* picture of the brain around the screen */
};

/* Brain NXP co-processor address on PS I2C0. */
#define VEX_NXP_I2C_ADDR 0x50

/* PL (FPGA) peripherals, see notes/mmio-map.md and smartport-tx.md. */
#define VEX_NUM_PORTS       22          /* smart port n = UART n-1 */
#define VEX_UART_BASE       0x41000000
#define VEX_UART_STRIDE     0x10000
#define VEX_INTC0_BASE      0x41800000
#define VEX_INTC_PORT_EDGES 0x007FFFFE  /* sources 1..22 edge-triggered */
#define VEX_INTC_TIMER0     26
#define VEX_GPIO1_BASE      0x41210000
#define VEX_GPIO1_IDLE      0x80        /* bit 7 front button, active low */
#define VEX_GPIO1_BUTTON    0x80
#define VEX_TIMER0_BASE     0x42800000
#define VEX_TIMER1_BASE     0x42810000
#define VEX_MUTEX0_BASE     0x43600000
#define VEX_MUTEX1_BASE     0x43610000
#define VEX_PL_NIRQ_PPI     31          /* nIRQ from PL, CPU0 */
#define VEX_TOUCH_ACK_BASE  0x43520000
#define VEX_INTC_TOUCH      23
#define VEX_INTC1_BASE      0x41810000  /* CPU1's INTC (system_0) */
#define VEX_INTC1_EDGES     0xFFFFFFFC
#define VEX_INTC1_PORT0     3           /* INTC1 source of port index 0 */

/* CPU1 start, see notes/cpu1-launch.md. */
#define SLCR_BASE           0xF8000000
#define SLCR_LOCK           0x004
#define SLCR_UNLOCK         0x008
#define SLCR_A9_CPU_RST     0x244
#define A9_RST1             (1 << 1)
#define A9_CLKSTOP1         (1 << 5)
#define SLCR_LOCK_KEY       0x767B
#define SLCR_UNLOCK_KEY     0xDF0D
#define FSBL_CPU1_ENTRY     0x00000024  /* FSBL dispatch: CPU1 jumps to [0x24] */
#define SYS0_BASE           0x03400000
#define SHARED_BASE         0xFFFF0000  /* OCM CPU0 <-> CPU1 block */
#define SHARED_BOOT_SIG     (SHARED_BASE + 0x8)
#define BOOT_SIG_LAUNCHING  0xFACE00FF
#define MAINLOOP_B2C8       0x0180B2C8
#define MAINLOOP_B2CC       0x0180B2CC
#define MAINLOOP_SCREEN_MODE 0x0180B3D0
#define MAINLOOP_B33C       0x0180B33C  /* controller / enable bits */
/*
 * Forces motors enabled (sysEvalFlagCondition). Unlike the UI motor test's
 * 0x01000000, which uiClearController9Alert drops when no controller is
 * seen, it stays set until the program exits.
 */
#define B33C_MOTORS_ENABLED 0x02000000
#define LOGICVC_BASE        0x43C00000
#define LOGICVC_LAYER_CTRL(n) (0x100 + 0x80 * (n) + 0x38)

/*
 * UART1 talks to the brain's NXP co-processor. Unless the user gave a
 * second -serial, answer it with the built-in NXP responder.
 */
static Chardev *vex_nxp_uart_chardev(void)
{
    if (serial_hd(1)) {
        return serial_hd(1);
    }
    return qemu_chardev_new("vex-nxp", "chardev-vex-nxp", NULL, NULL,
                            &error_fatal);
}

static uint32_t vex_cpu_rst_get(VexV5MachineState *m)
{
    uint64_t v = 0;

    memory_region_dispatch_read(m->slcr_mr, SLCR_A9_CPU_RST, &v, MO_32,
                                MEMTXATTRS_UNSPECIFIED);
    return v;
}

static uint64_t vex_cpu_rst_read(void *opaque, hwaddr off, unsigned size)
{
    return vex_cpu_rst_get(opaque);
}

/*
 * The stock SLCR stores A9_CPU_RST_CTRL without acting on it. VEXos starts
 * a user program by setting A9_RST1 and clearing it again, so CPU1 comes
 * out of reset at address 0 (the FSBL's CPU dispatch).
 */
static void vex_cpu_rst_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    VexV5MachineState *m = opaque;
    uint32_t before = vex_cpu_rst_get(m);
    uint32_t after;

    memory_region_dispatch_write(m->slcr_mr, SLCR_A9_CPU_RST, val, MO_32,
                                 MEMTXATTRS_UNSPECIFIED);
    after = vex_cpu_rst_get(m);
    if (m->parent.smp.cpus < 2) {
        return;
    }
    if (!(before & A9_RST1) && (after & A9_RST1)) {
        arm_set_cpu_off(1);     /* "already off" is fine */
    } else if ((before & A9_RST1) && !(after & A9_RST1)) {
        arm_set_cpu_on_and_reset(1);
    }
}

static const MemoryRegionOps vex_cpu_rst_ops = {
    .read = vex_cpu_rst_read,
    .write = vex_cpu_rst_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void vex_stl(hwaddr addr, uint32_t v)
{
    address_space_stl_le(&address_space_memory, addr, v,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint32_t vex_ldl(hwaddr addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * M4a bring-up: start the user program already loaded at 0x03800000 the
 * way MainLoop's sysStopCpu1 + sysReleaseCpu1Reset do, plus the screen and
 * flag writes of userProgramLaunchSlot so CPU0 stops drawing over it.
 */
static void vex_launch_user(void *opaque)
{
    vex_stl(FSBL_CPU1_ENTRY, SYS0_BASE);
    vex_stl(SHARED_BOOT_SIG, BOOT_SIG_LAUNCHING);
    vex_stl(MAINLOOP_B2C8,
            (vex_ldl(MAINLOOP_B2C8) & ~0x08000000u) | 0x01001000u);
    vex_stl(MAINLOOP_B2CC, vex_ldl(MAINLOOP_B2CC) | 0x100);
    /*
     * No controller is emulated yet, and without one MainLoop rewrites every
     * motor command into a stop (notes/motor-protocol.md 4.4).
     */
    vex_stl(MAINLOOP_B33C, vex_ldl(MAINLOOP_B33C) | B33C_MOTORS_ENABLED);
    address_space_stb(&address_space_memory, MAINLOOP_SCREEN_MODE, 4,
                      MEMTXATTRS_UNSPECIFIED, NULL);

    vex_stl(SLCR_BASE + SLCR_UNLOCK, SLCR_UNLOCK_KEY);
    vex_stl(SLCR_BASE + SLCR_A9_CPU_RST,
            vex_ldl(SLCR_BASE + SLCR_A9_CPU_RST) | A9_RST1 | A9_CLKSTOP1);
    vex_stl(SLCR_BASE + SLCR_A9_CPU_RST,
            vex_ldl(SLCR_BASE + SLCR_A9_CPU_RST) & ~(A9_RST1 | A9_CLKSTOP1));
    vex_stl(SLCR_BASE + SLCR_LOCK, SLCR_LOCK_KEY);

    /*
     * MainLoop's own launch path hands the screen to the program; this
     * shortcut only shows the user layer (2) over the home UI (3).
     */
    vex_stl(LOGICVC_BASE + LOGICVC_LAYER_CTRL(3), 0);
    vex_stl(LOGICVC_BASE + LOGICVC_LAYER_CTRL(2), 1);
}

static void vex_launch_reset(void *opaque)
{
    VexV5MachineState *m = opaque;

    if (m->launch_ms) {
        timer_mod(m->launch_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  m->launch_ms);
    }
}

static Chardev *vex_port_chardev(int port)
{
    g_autofree char *id = g_strdup_printf("port%d", port);

    return qemu_chr_find(id);
}

/*
 * Smart ports, AXI INTC0, AXI timers, AXI GPIO and AXI mutexes. INTC0
 * feeds PPI 31 on CPU0, which the a9mpcore wrapper doesn't expose, so go
 * to its GIC directly. Port n is UartLite n-1 on INTC source n, attached
 * to "-chardev ...,id=port<n>" when one exists.
 */
static void vex_v5_init_pl(DeviceState *mpcore)
{
    A9MPPrivState *priv = A9MPCORE_PRIV(mpcore);
    DeviceState *intc, *intc1 = NULL, *dev;
    SysBusDevice *busdev;
    int i;

    intc = qdev_new("xlnx.xps-intc");
    qdev_prop_set_enum(intc, "endianness", ENDIAN_MODE_LITTLE);
    /* Smart ports pulse their line per received frame: latch those edges. */
    qdev_prop_set_uint32(intc, "kind-of-intr", VEX_INTC_PORT_EDGES);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(intc), 0, VEX_INTC0_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(intc), 0,
                       qdev_get_gpio_in(DEVICE(&priv->gic),
                                        priv->num_irq - GIC_INTERNAL +
                                        VEX_PL_NIRQ_PPI));

    if (priv->num_cpu > 1) {
        intc1 = dev = qdev_new("xlnx.xps-intc");
        qdev_prop_set_enum(dev, "endianness", ENDIAN_MODE_LITTLE);
        /* system_0's config: sources 2..31 edge (XIntc cfg id 1) */
        qdev_prop_set_uint32(dev, "kind-of-intr", VEX_INTC1_EDGES);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, VEX_INTC1_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(DEVICE(&priv->gic),
                                            priv->num_irq - GIC_INTERNAL +
                                            GIC_INTERNAL + VEX_PL_NIRQ_PPI));
    }

    for (i = 0; i < VEX_NUM_PORTS; i++) {
        dev = qdev_new("vex-v5-smartport");
        qdev_prop_set_chr(dev, "chardev", vex_port_chardev(i + 1));
        qdev_prop_set_uint32(dev, "index", i + 1);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, VEX_UART_BASE + i * VEX_UART_STRIDE);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(intc, i + 1));
        /* generic serial: system_0 g_serialIntrIdTable 0x03499BB8 maps
           port index i (0..20) to INTC1 source i + 3; index 21 has none */
        if (intc1 && i < VEX_NUM_PORTS - 1) {
            sysbus_connect_irq(busdev, 1,
                               qdev_get_gpio_in(intc1, i + VEX_INTC1_PORT0));
        }
    }

    for (i = 0; i < 2; i++) {
        dev = qdev_new("xlnx.xps-timer");
        qdev_prop_set_enum(dev, "endianness", ENDIAN_MODE_LITTLE);
        qdev_prop_set_uint32(dev, "clock-frequency", 100 * 1000 * 1000);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, i ? VEX_TIMER1_BASE : VEX_TIMER0_BASE);
        if (i == 0) {
            sysbus_connect_irq(busdev, 0,
                               qdev_get_gpio_in(intc, VEX_INTC_TIMER0));
        }
    }

    /*
     * The front button: /machine/gpio1 "in1" bit 7 (QMP qom-set), or hold B
     * in the display window.
     */
    dev = qdev_new("xlnx.axi-gpio");
    object_property_add_child(qdev_get_machine(), "gpio1", OBJECT(dev));
    object_property_set_uint(OBJECT(dev), "in1", VEX_GPIO1_IDLE, &error_fatal);
    qdev_prop_set_string(dev, "button-key", "b");
    qdev_prop_set_uint32(dev, "button-mask", VEX_GPIO1_BUTTON);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, VEX_GPIO1_BASE);

    {
        DeviceState *gpio = dev;
        VexV5MachineState *m = VEX_V5_MACHINE(qdev_get_machine());

        /* Clicks on the skin's front button press it. */
        dev = qdev_new("vex-v5-touch");
        object_property_add_child(OBJECT(m), "touch", OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, VEX_TOUCH_ACK_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(intc, VEX_INTC_TOUCH));
        qdev_connect_gpio_out_named(dev, "button", 0,
                                    qdev_get_gpio_in_named(gpio, "button", 0));
    }

    sysbus_create_simple("xlnx.axi-mutex", VEX_MUTEX0_BASE, NULL);
    sysbus_create_simple("xlnx.axi-mutex", VEX_MUTEX1_BASE, NULL);
}

/* board base frequency: 33.333333 MHz */
#define PS_CLK_FREQUENCY (100 * 1000 * 1000 / 3)

#define NUM_SPI_FLASHES 4
#define NUM_QSPI_FLASHES 2
#define NUM_QSPI_BUSSES 2

#define FLASH_SIZE (64 * 1024 * 1024)
#define FLASH_SECTOR_SIZE (128 * 1024)

#define MPCORE_PERIPHBASE 0xF8F00000
#define ZYNQ_BOARD_MIDR 0x413FC090

#define GIC_EXT_IRQS 64 /* Zynq 7000 SoC */

static const int dma_irqs[8] = {
    46, 47, 48, 49, 72, 73, 74, 75
};

#define BOARD_SETUP_ADDR        0x100

#define SLCR_LOCK_OFFSET        0x004
#define SLCR_UNLOCK_OFFSET      0x008
#define SLCR_ARM_PLL_OFFSET     0x100

#define SLCR_XILINX_UNLOCK_KEY  0xdf0d
#define SLCR_XILINX_LOCK_KEY    0x767b

#define ZYNQ_SDHCI_CAPABILITIES 0x69ec0080  /* Datasheet: UG585 (v1.12.1) */

#define ARMV7_IMM16(x) (extract32((x),  0, 12) | \
                        extract32((x), 12,  4) << 16)

/* Write immediate val to address r0 + addr. r0 should contain base offset
 * of the SLCR block. Clobbers r1.
 */

#define SLCR_WRITE(addr, val) \
    0xe3001000 + ARMV7_IMM16(extract32((val),  0, 16)), /* movw r1 ... */ \
    0xe3401000 + ARMV7_IMM16(extract32((val), 16, 16)), /* movt r1 ... */ \
    0xe5801000 + (addr)

static void zynq_write_board_setup(ARMCPU *cpu,
                                   const struct arm_boot_info *info)
{
    int n;
    uint32_t board_setup_blob[] = {
        0xe3a004f8, /* mov r0, #0xf8000000 */
        SLCR_WRITE(SLCR_UNLOCK_OFFSET, SLCR_XILINX_UNLOCK_KEY),
        SLCR_WRITE(SLCR_ARM_PLL_OFFSET, 0x00014008),
        SLCR_WRITE(SLCR_LOCK_OFFSET, SLCR_XILINX_LOCK_KEY),
        0xe12fff1e, /* bx lr */
    };
    for (n = 0; n < ARRAY_SIZE(board_setup_blob); n++) {
        board_setup_blob[n] = tswap32(board_setup_blob[n]);
    }
    rom_add_blob_fixed("board-setup", board_setup_blob,
                       sizeof(board_setup_blob), BOARD_SETUP_ADDR);
}

static struct arm_boot_info zynq_binfo = {};

static void gem_init(uint32_t base, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_new(TYPE_CADENCE_GEM);
    qemu_configure_nic_device(dev, true, NULL);
    object_property_set_int(OBJECT(dev), "phy-addr", 7, &error_abort);
    s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
}

static inline int zynq_init_spi_flashes(uint32_t base_addr, qemu_irq irq,
                                        bool is_qspi, int unit0)
{
    int unit = unit0;
    DeviceState *dev;
    SysBusDevice *busdev;
    SSIBus *spi;
    DeviceState *flash_dev;
    int i, j;
    int num_busses =  is_qspi ? NUM_QSPI_BUSSES : 1;
    int num_ss = is_qspi ? NUM_QSPI_FLASHES : NUM_SPI_FLASHES;

    dev = qdev_new(is_qspi ? "xlnx.ps7-qspi" : "xlnx.ps7-spi");
    qdev_prop_set_uint8(dev, "num-txrx-bytes", is_qspi ? 4 : 1);
    qdev_prop_set_uint8(dev, "num-ss-bits", num_ss);
    qdev_prop_set_uint8(dev, "num-busses", num_busses);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, base_addr);
    if (is_qspi) {
        sysbus_mmio_map(busdev, 1, 0xFC000000);
    }
    sysbus_connect_irq(busdev, 0, irq);

    for (i = 0; i < num_busses; ++i) {
        char bus_name[16];
        qemu_irq cs_line;

        snprintf(bus_name, 16, "spi%d", i);
        spi = (SSIBus *)qdev_get_child_bus(dev, bus_name);

        for (j = 0; j < num_ss; ++j) {
            DriveInfo *dinfo = drive_get(IF_MTD, 0, unit++);
            /*
             * VEX brain: QSPI 32 MB (SPIFFS at 0x1800000); SPI0 holds the
             * 2 MB event log, which logInitFlashMap expects to be ISSI.
             */
            flash_dev = qdev_new(is_qspi ? "n25q256a" : "is25lp016d");
            if (dinfo) {
                qdev_prop_set_drive_err(flash_dev, "drive",
                                        blk_by_legacy_dinfo(dinfo),
                                        &error_fatal);
            }
            qdev_prop_set_uint8(flash_dev, "cs", j);
            qdev_realize_and_unref(flash_dev, BUS(spi), &error_fatal);

            cs_line = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
            sysbus_connect_irq(busdev, i * num_ss + j + 1, cs_line);
        }
    }

    return unit;
}

static void zynq_set_boot_mode(Object *obj, const char *str,
                                               Error **errp)
{
    VexV5MachineState *m = VEX_V5_MACHINE(obj);
    uint8_t mode = 0;

    if (!g_ascii_strcasecmp(str, "qspi")) {
        mode = 1;
    } else if (!g_ascii_strcasecmp(str, "sd")) {
        mode = 5;
    } else if (!g_ascii_strcasecmp(str, "nor")) {
        mode = 2;
    } else if (!g_ascii_strcasecmp(str, "jtag")) {
        mode = 0;
    } else {
        error_setg(errp, "%s boot mode not supported", str);
        return;
    }
    m->boot_mode = mode;
}

static void ddr_ctrl_init(uint32_t base)
{
    DeviceState *dev;
    SysBusDevice *busdev;

    dev = qdev_new("zynq.ddr-ctlr");
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, base);
}

/*
 * Cortex-A9 memory-mapped (APB) debug register bank. Defined in
 * target/arm/tcg/debug.c (see qemu-vex/patches/0006): routes the ARMv7 debug
 * register offsets vexide/v5gdb uses to the CPU's cp15 debug state and QEMU's
 * breakpoint machinery, so self-hosted hardware breakpoints and single-step
 * fire as guest debug exceptions. The opaque is the owning ARMCPU.
 */
uint64_t arm_cpu_debug_mmio_read(ARMCPU *cpu, hwaddr offset);
void arm_cpu_debug_mmio_write(ARMCPU *cpu, hwaddr offset, uint64_t value);

static uint64_t vex_a9_debug_read(void *opaque, hwaddr offset, unsigned size)
{
    return arm_cpu_debug_mmio_read(ARM_CPU(opaque), offset);
}

static void vex_a9_debug_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    arm_cpu_debug_mmio_write(ARM_CPU(opaque), offset, value);
}

static const MemoryRegionOps vex_a9_debug_ops = {
    .read = vex_a9_debug_read,
    .write = vex_a9_debug_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void zynq_init(MachineState *machine)
{
    VexV5MachineState *zynq_machine = VEX_V5_MACHINE(machine);
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ocm_ram = g_new(MemoryRegion, 1);
    DeviceState *dev, *slcr;
    SysBusDevice *busdev;
    qemu_irq pic[GIC_EXT_IRQS];
    int n;
    unsigned int smp_cpus = machine->smp.cpus;

    /* max 2GB ram */
    if (machine->ram_size > 2 * GiB) {
        error_report("RAM size more than 2 GiB is not supported");
        exit(EXIT_FAILURE);
    }

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        object_property_set_int(cpuobj, "midr", ZYNQ_BOARD_MIDR,
                                &error_fatal);
        object_property_set_int(cpuobj, "reset-cbar", MPCORE_PERIPHBASE,
                                &error_fatal);
        /* CPU1 waits in reset until MainLoop launches a user program. */
        object_property_set_bool(cpuobj, "start-powered-off", n != 0,
                                 &error_fatal);

        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

        zynq_machine->cpu[n] = ARM_CPU(cpuobj);
    }

    /* DDR remapped to address zero.  */
    memory_region_add_subregion(address_space_mem, 0, machine->ram);

    /* 256K of on-chip memory */
    memory_region_init_ram(ocm_ram, NULL, "zynq.ocm_ram", 256 * KiB,
                           &error_fatal);
    memory_region_add_subregion(address_space_mem, 0xFFFC0000, ocm_ram);

    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    /* AMD */
    pflash_cfi02_register(0xe2000000, "zynq.pflash", FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          FLASH_SECTOR_SIZE, 1,
                          1, 0x0066, 0x0022, 0x0000, 0x0000, 0x0555, 0x2aa,
                          0);

    /* Create the main clock source, and feed slcr with it */
    zynq_machine->ps_clk = CLOCK(object_new(TYPE_CLOCK));
    object_property_add_child(OBJECT(zynq_machine), "ps_clk",
                              OBJECT(zynq_machine->ps_clk));
    object_unref(OBJECT(zynq_machine->ps_clk));
    clock_set_hz(zynq_machine->ps_clk, PS_CLK_FREQUENCY);

    /* Create slcr, keep a pointer to connect clocks */
    slcr = qdev_new("xilinx-zynq_slcr");
    qdev_connect_clock_in(slcr, "ps_clk", zynq_machine->ps_clk);
    qdev_prop_set_uint8(slcr, "boot-mode", zynq_machine->boot_mode);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(slcr), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(slcr), 0, 0xF8000000);
    zynq_machine->slcr_mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(slcr), 0);
    memory_region_init_io(&zynq_machine->cpu_rst, OBJECT(machine),
                          &vex_cpu_rst_ops, zynq_machine, "vex.cpu-rst", 4);
    memory_region_add_subregion_overlap(address_space_mem,
                                        SLCR_BASE + SLCR_A9_CPU_RST,
                                        &zynq_machine->cpu_rst, 1);
    zynq_machine->launch_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                              vex_launch_user, zynq_machine);
    qemu_register_reset(vex_launch_reset, zynq_machine);

    dev = qdev_new(TYPE_A9MPCORE_PRIV);
    qdev_prop_set_uint32(dev, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(dev, "num-irq", GIC_EXT_IRQS + GIC_INTERNAL);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, MPCORE_PERIPHBASE);
    zynq_binfo.gic_cpu_if_addr = MPCORE_PERIPHBASE + 0x100;
    sysbus_create_varargs("l2x0", MPCORE_PERIPHBASE + 0x2000, NULL);
    for (n = 0; n < smp_cpus; n++) {
        /* See "hw/intc/arm_gic.h" for the IRQ line association */
        DeviceState *cpudev = DEVICE(zynq_machine->cpu[n]);
        sysbus_connect_irq(busdev, n,
                           qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(busdev, smp_cpus + n,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
    }

    for (n = 0; n < GIC_EXT_IRQS; n++) {
        pic[n] = qdev_get_gpio_in(dev, n);
    }
    vex_v5_init_pl(dev);

    n = zynq_init_spi_flashes(0xE0006000, pic[58 - GIC_INTERNAL], false, 0);
    n = zynq_init_spi_flashes(0xE0007000, pic[81 - GIC_INTERNAL], false, n);
    n = zynq_init_spi_flashes(0xE000D000, pic[51 - GIC_INTERNAL], true, n);

    /*
     * USB0 is the brain's USB port, a device (notes/usb-protocol.md): its
     * two CDC ports go to "-chardev ...,id=usbsys" / "id=usbuser".
     */
    dev = qdev_new("vex-v5-usbdev");
    qdev_prop_set_chr(dev, "sys", qemu_chr_find("usbsys"));
    qdev_prop_set_chr(dev, "user", qemu_chr_find("usbuser"));
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xE0002000);
    sysbus_connect_irq(busdev, 0, pic[53 - GIC_INTERNAL]);
    sysbus_create_simple(TYPE_CHIPIDEA, 0xE0003000, pic[76 - GIC_INTERNAL]);

    dev = qdev_new(TYPE_CADENCE_UART);
    busdev = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardev", serial_hd(0));
    qdev_connect_clock_in(dev, "refclk",
                          qdev_get_clock_out(slcr, "uart0_ref_clk"));
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xE0000000);
    sysbus_connect_irq(busdev, 0, pic[59 - GIC_INTERNAL]);
    dev = qdev_new(TYPE_CADENCE_UART);
    busdev = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardev", vex_nxp_uart_chardev());
    qdev_connect_clock_in(dev, "refclk",
                          qdev_get_clock_out(slcr, "uart1_ref_clk"));
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xE0001000);
    sysbus_connect_irq(busdev, 0, pic[82 - GIC_INTERNAL]);

    sysbus_create_varargs("cadence_ttc", 0xF8001000,
            pic[42-GIC_INTERNAL], pic[43-GIC_INTERNAL], pic[44-GIC_INTERNAL], NULL);
    sysbus_create_varargs("cadence_ttc", 0xF8002000,
            pic[69-GIC_INTERNAL], pic[70-GIC_INTERNAL], pic[71-GIC_INTERNAL], NULL);

    ddr_ctrl_init(0xF8006000);

    /*
     * Cortex-A9 memory-mapped (APB) debug register banks. Real Zynq-7000
     * exposes per-core debug logic at 0xF8890000 (CPU0) and 0xF8892000 (CPU1);
     * the PMU/CTI/PTM CoreSight stubs sit in the 4 KiB slots between them (see
     * below). DBGDRAR/DBGDSAR (patched in target/arm/debug_helper.c for the A9)
     * point self-hosted debuggers such as vexide/v5gdb at these banks. Accesses
     * go to arm_cpu_debug_mmio_read/write (patch 0006), which back
     * DBGBVR/DBGBCR/DBGWVR/DBGWCR/DBGDSCR with the CPU's cp15 debug state and
     * QEMU's breakpoint machinery, so breakpoints and ARMv7 address-mismatch
     * single-step fire as guest debug exceptions.
     */
    {
        static const hwaddr a9_debug_base[ZYNQ_MAX_CPUS] = {
            0xF8890000, 0xF8892000,
        };
        for (n = 0; n < ZYNQ_MAX_CPUS; n++) {
            MemoryRegion *dbg = g_new(MemoryRegion, 1);
            g_autofree char *name = g_strdup_printf("cortex-a9.debug%d", n);
            memory_region_init_io(dbg, NULL, &vex_a9_debug_ops,
                                  zynq_machine->cpu[n], name, 0x1000);
            memory_region_add_subregion(address_space_mem, a9_debug_base[n],
                                        dbg);
        }
    }

    gem_init(0xE000B000, pic[54 - GIC_INTERNAL]);
    gem_init(0xE000C000, pic[77 - GIC_INTERNAL]);

    for (n = 0; n < 2; n++) {
        int hci_irq = n ? 79 : 56;
        hwaddr hci_addr = n ? 0xE0101000 : 0xE0100000;
        DriveInfo *di;
        BlockBackend *blk;
        DeviceState *carddev;

        /* Compatible with:
         * - SD Host Controller Specification Version 2.0 Part A2
         * - SDIO Specification Version 2.0
         * - MMC Specification Version 3.31
         */
        dev = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(dev, "sd-spec-version", 2);
        qdev_prop_set_uint64(dev, "capareg", ZYNQ_SDHCI_CAPABILITIES);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, hci_addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[hci_irq - GIC_INTERNAL]);

        di = drive_get(IF_SD, 0, n);
        blk = di ? blk_by_legacy_dinfo(di) : NULL;
        carddev = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
        qdev_realize_and_unref(carddev, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }

    dev = qdev_new(TYPE_ZYNQ_XADC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xF8007100);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[39-GIC_INTERNAL]);

    dev = qdev_new("pl330");
    object_property_set_link(OBJECT(dev), "memory",
                             OBJECT(address_space_mem),
                             &error_fatal);
    qdev_prop_set_uint8(dev, "num_chnls",  8);
    qdev_prop_set_uint8(dev, "num_periph_req",  4);
    qdev_prop_set_uint8(dev, "num_events",  16);

    qdev_prop_set_uint8(dev, "data_width",  64);
    qdev_prop_set_uint8(dev, "wr_cap",  8);
    qdev_prop_set_uint8(dev, "wr_q_dep",  16);
    qdev_prop_set_uint8(dev, "rd_cap",  8);
    qdev_prop_set_uint8(dev, "rd_q_dep",  16);
    qdev_prop_set_uint16(dev, "data_buffer_dep",  256);

    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xF8003000);
    sysbus_connect_irq(busdev, 0, pic[45-GIC_INTERNAL]); /* abort irq line */
    for (n = 0; n < ARRAY_SIZE(dma_irqs); ++n) { /* event irqs */
        sysbus_connect_irq(busdev, n + 1, pic[dma_irqs[n] - GIC_INTERNAL]);
    }

    dev = qdev_new("xlnx.ps7-dev-cfg");
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_connect_irq(busdev, 0, pic[40 - GIC_INTERNAL]);
    sysbus_mmio_map(busdev, 0, 0xF8007000);

    /*
     * Refer to the ug585-Zynq-7000-TRM manual B.3 (Module Summary) and
     * the zynq-7000.dtsi. Add placeholders for unimplemented devices.
     */
    dev = qdev_new("cadence.i2c");
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0xE0004000);
    sysbus_connect_irq(busdev, 0, pic[57 - GIC_INTERNAL]);
    i2c_slave_create_simple((I2CBus *)qdev_get_child_bus(dev, "i2c"),
                            "vex-v5-nxp", VEX_NXP_I2C_ADDR);

    create_unimplemented_device("zynq.i2c1", 0xE0005000, 4 * KiB);
    create_unimplemented_device("zynq.can0", 0xE0008000, 4 * KiB);
    create_unimplemented_device("zynq.can1", 0xE0009000, 4 * KiB);
    create_unimplemented_device("zynq.gpio", 0xE000A000, 4 * KiB);
    create_unimplemented_device("zynq.smcc", 0xE000E000, 4 * KiB);

    /* Direct Memory Access Controller, PL330, Non-Secure Mode */
    create_unimplemented_device("zynq.dma_ns", 0xF8004000, 4 * KiB);

    /* System Watchdog Timer Registers */
    create_unimplemented_device("zynq.swdt", 0xF8005000, 4 * KiB);

    /* AXI_HP Interface (AFI) */
    create_unimplemented_device("zynq.axi_hp0", 0xF8008000, 0x28);
    create_unimplemented_device("zynq.axi_hp1", 0xF8009000, 0x28);
    create_unimplemented_device("zynq.axi_hp2", 0xF800A000, 0x28);
    create_unimplemented_device("zynq.axi_hp3", 0xF800B000, 0x28);

    create_unimplemented_device("zynq.efuse", 0xF800d000, 0x20);

    /* Embedded Trace Buffer */
    create_unimplemented_device("zynq.etb", 0xF8801000, 4 * KiB);

    /* Cross Trigger Interface, ETB and TPIU */
    create_unimplemented_device("zynq.cti_etb_tpiu", 0xF8802000, 4 * KiB);

    /* Trace Port Interface Unit */
    create_unimplemented_device("zynq.tpiu", 0xF8803000, 4 * KiB);

    /* CoreSight Trace Funnel */
    create_unimplemented_device("zynq.funnel", 0xF8804000, 4 * KiB);

    /* Instrumentation Trace Macrocell */
    create_unimplemented_device("zynq.itm", 0xF8805000, 4 * KiB);

    /* Cross Trigger Interface, FTM */
    create_unimplemented_device("zynq.cti_ftm", 0xF8809000, 4 * KiB);

    /* Fabric Trace Macrocell */
    create_unimplemented_device("zynq.ftm", 0xF880B000, 4 * KiB);

    /* Cortex A9 Performance Monitoring Unit, CPU */
    create_unimplemented_device("cortex-a9.pmu0", 0xF8891000, 4 * KiB);
    create_unimplemented_device("cortex-a9.pmu1", 0xF8893000, 4 * KiB);

    /* Cross Trigger Interface, CPU */
    create_unimplemented_device("zynq.cpu_cti0", 0xF8898000, 4 * KiB);
    create_unimplemented_device("zynq.cpu_cti1", 0xF8899000, 4 * KiB);

    /* CoreSight PTM-A9, CPU */
    create_unimplemented_device("cortex-a9.ptm0", 0xF889c000, 4 * KiB);
    create_unimplemented_device("cortex-a9.ptm1", 0xF889d000, 4 * KiB);

    /* AMBA NIC301 TrustZone */
    create_unimplemented_device("zynq.trustZone", 0xF8900000, 0x20);

    /* AMBA Network Interconnect Advanced Quality of Service (QoS-301) */
    create_unimplemented_device("zynq.qos301_cpu", 0xF8946000, 0x130);
    create_unimplemented_device("zynq.qos301_dmac", 0xF8947000, 0x130);
    create_unimplemented_device("zynq.qos301_iou", 0xF8948000, 0x130);

    /* PL: logiCVC display controller. IRQ is a guess (first PL IRQ). */
    dev = qdev_new("logicvc");
    object_property_add_child(OBJECT(machine), "display", OBJECT(dev));
    if (zynq_machine->skin) {
        qdev_prop_set_string(dev, "skin", zynq_machine->skin);
    }
    object_property_set_link(OBJECT(dev), "button-gpio",
                             object_resolve_path("/machine/gpio1", NULL), &error_fatal);
    qdev_prop_set_uint32(dev, "button-mask", VEX_GPIO1_BUTTON);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, 0x43C00000);
    sysbus_connect_irq(busdev, 0, pic[61 - GIC_INTERNAL]);
    {
        /* the touch panel covers the same picture */
        static const char *const geom[] = { "canvas-w", "canvas-h", "screen-x",
            "screen-y", "button-x", "button-y", "button-half" };
        Object *touch = object_resolve_path("/machine/touch", NULL);

        for (n = 0; n < ARRAY_SIZE(geom); n++) {
            object_property_set_uint(touch, geom[n],
                object_property_get_uint(OBJECT(dev), geom[n], &error_fatal),
                &error_fatal);
        }
    }

    /* Rest of PL (FPGA) AXI GP0/GP1: not modelled yet, log every access. */
    create_unimplemented_device("vex.pl.gp0", 0x40000000, 0x40000000);
    create_unimplemented_device("vex.pl.gp1", 0x80000000, 0x40000000);

    zynq_binfo.ram_size = machine->ram_size;
    zynq_binfo.board_id = 0xd32;
    zynq_binfo.loader_start = 0;
    zynq_binfo.board_setup_addr = BOARD_SETUP_ADDR;
    zynq_binfo.write_board_setup = zynq_write_board_setup;

    arm_load_kernel(zynq_machine->cpu[0], machine, &zynq_binfo);
}

static void zynq_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a9"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);
    ObjectProperty *prop;
    mc->desc = "VEX V5 Robot Brain (Zynq-7010)";
    mc->init = zynq_init;
    mc->max_cpus = ZYNQ_MAX_CPUS;
    mc->ignore_memory_transaction_failures = true;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "zynq.ext_ram";
    mc->default_ram_size = 512 * MiB;
    prop = object_class_property_add_str(oc, "boot-mode", NULL,
                                         zynq_set_boot_mode);
    object_class_property_set_description(oc, "boot-mode",
                                          "Supported boot modes:"
                                          " jtag qspi sd nor");
    object_property_set_default_str(prop, "qspi");
}

static char *vex_v5_get_skin(Object *obj, Error **errp)
{
    return g_strdup(VEX_V5_MACHINE(obj)->skin);
}

static void vex_v5_set_skin(Object *obj, const char *value, Error **errp)
{
    VexV5MachineState *m = VEX_V5_MACHINE(obj);

    g_free(m->skin);
    m->skin = g_strdup(value);
}

static void vex_v5_machine_instance_init(Object *obj)
{
    VexV5MachineState *m = VEX_V5_MACHINE(obj);

    object_property_add_uint32_ptr(obj, "launch-ms", &m->launch_ms,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "launch-ms",
                                    "Start the user program at 0x03800000 "
                                    "on CPU1 this many ms after reset "
                                    "(0 = let VEXos do it)");
    object_property_add_str(obj, "skin", vex_v5_get_skin, vex_v5_set_skin);
    object_property_set_description(obj, "skin",
                                    "A binary PPM of the brain around its screen "
                                    "(sim/skins/make_brain_skin.py): the window "
                                    "shows it, and clicks on its front button "
                                    "press the button");
}

static const TypeInfo vex_v5_machine_type = {
    .name = TYPE_VEX_V5_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = zynq_machine_class_init,
    .instance_size = sizeof(VexV5MachineState),
    .instance_init = vex_v5_machine_instance_init,
    .interfaces = arm_machine_interfaces,
};

static void vex_v5_machine_register_types(void)
{
    type_register_static(&vex_v5_machine_type);
}

type_init(vex_v5_machine_register_types)
