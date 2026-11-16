/*
 * VEX V5 smart motor board: an NXP LPC824 (Cortex-M0+, 30 MHz) running the
 * motor firmware VEXos ships to devices (DEVFRM02 11 W, DEVFRM05 5.5 W,
 * DEVFRM31 MC55). See notes/motor-emulation.md.
 *
 *   qemu-system-arm -M vex-motor -bios DEVFRM02.BIN \
 *       -chardev socket,id=port,... -chardev socket,id=plant,...
 *
 * The image is the application only; VEX's bootloader at 0..0x1FFF is not
 * in VEXos, so the core starts from the app's vector table (VTOR 0x2000).
 * Chardev "port" is the smart-port UART (to the brain), "plant" the analogue
 * link to the robot sim (hw/misc/lpc82x.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/clock.h"
#include "hw/arm/armv7m.h"
#include "hw/core/irq.h"
#include "exec/cpu-interrupt.h"
#include "target/arm/cpu.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/loader.h"
#include "hw/misc/unimp.h"
#include "chardev/char.h"
#include "system/address-spaces.h"

#define MOTOR_SYSCLK_HZ  30000000ULL    /* 12 MHz IRC x5 PLL / 2 */
#define FLASH_SIZE       (32 * KiB)
#define APP_BASE         0x2000
#define SRAM_BASE        0x10000000
#define SRAM_SIZE        (8 * KiB)
#define BOOTROM_BASE     0x1FFF0000
#define BOOTROM_SIZE     (64 * KiB)
#define IAP_ENTRY        0x1FFF1FF0     /* firmware calls 0x1FFF1FF1 */

/* MMIO regions of lpc82x-periph, in its sysbus_init_mmio order. */
static const hwaddr periph_base[] = {
    0x40048000,     /* SYSCON */
    0x40004000,     /* MRT */
    0x4001C000,     /* ADC */
    0x40064000,     /* USART0 */
    0x4000C000,     /* SWM */
    0x40044000,     /* IOCON */
    0x50000000,     /* CRC */
    0x50004000,     /* SCT */
    0x50008000,     /* DMA */
    0xA0000000,     /* GPIO */
    0xA0004000,     /* PININT */
};

/*
 * The ROM's IAP entry: answer every command with CMD_SUCCESS (result[0] = 0).
 * The firmware only uses IAP for firmware updates.
 */
static const uint16_t iap_stub[] = {
    0x2200,         /* movs r2, #0 */
    0x600A,         /* str  r2, [r1] */
    0x4770,         /* bx   lr */
};

/*
 * The firmware never sleeps: its main loop polls flags that only interrupts
 * set (SysTick, DMA, MRT, ADC). lpc82x pulses "idle-poll" when the loop reads
 * its fault pin; parking the core there until the next interrupt, as a WFI
 * would, saves a host core per motor and changes nothing the firmware sees
 * except up to one interrupt of extra latency for a second pending job.
 * Only in thread mode with interrupts unmasked, so it can always wake.
 */
static void vex_motor_idle(void *opaque, int n, int level)
{
    ARMCPU *cpu = opaque;
    CPUARMState *env = &cpu->env;

    if (level && !arm_v7m_is_handler_mode(env) &&
        !env->v7m.primask[env->v7m.secure]) {
        cpu_interrupt(CPU(cpu), CPU_INTERRUPT_HALT);
    }
}

static void vex_motor_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    Clock *sysclk;
    DeviceState *armv7m, *periph;
    SysBusDevice *sbd;
    ssize_t size;

    if (!machine->firmware) {
        error_report("vex-motor: give the motor firmware with -bios "
                     "(e.g. extracted/assets/files/DEVFRM02.BIN)");
        exit(1);
    }

    memory_region_init_rom(flash, NULL, "vex-motor.flash", FLASH_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, flash);
    memory_region_init_ram(sram, NULL, "vex-motor.sram", SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SRAM_BASE, sram);
    memory_region_init_rom(bootrom, NULL, "vex-motor.bootrom", BOOTROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, BOOTROM_BASE, bootrom);

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, MOTOR_SYSCLK_HZ);

    armv7m = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(machine), "armv7m", OBJECT(armv7m));
    qdev_prop_set_uint32(armv7m, "num-irq", 32);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 2);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    qdev_prop_set_uint32(armv7m, "init-nsvtor", APP_BASE);
    qdev_connect_clock_in(armv7m, "cpuclk", sysclk);
    object_property_set_link(OBJECT(armv7m), "memory", OBJECT(sysmem),
                             &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(armv7m), &error_fatal);

    /* Everything not modelled reads as zero and logs under -d unimp. */
    create_unimplemented_device("lpc82x.apb", 0x40000000, 0x80000);
    create_unimplemented_device("lpc82x.ahb", 0x50000000, 0x10000);
    create_unimplemented_device("lpc82x.gpio-ahb", 0xA0000000, 0x10000);

    periph = qdev_new("lpc82x-periph");
    object_property_add_child(OBJECT(machine), "periph", OBJECT(periph));
    qdev_prop_set_chr(periph, "port", qemu_chr_find("port"));
    qdev_prop_set_chr(periph, "plant", qemu_chr_find("plant"));
    sbd = SYS_BUS_DEVICE(periph);
    sysbus_realize_and_unref(sbd, &error_fatal);
    for (size_t i = 0; i < ARRAY_SIZE(periph_base); i++) {
        sysbus_mmio_map(sbd, i, periph_base[i]);
    }
    for (int i = 0; i < 32; i++) {
        sysbus_connect_irq(sbd, i, qdev_get_gpio_in(armv7m, i));
    }
    qdev_connect_gpio_out_named(periph, "idle-poll", 0,
                                qemu_allocate_irq(vex_motor_idle,
                                                  ARMV7M(armv7m)->cpu, 0));

    size = load_image_targphys(machine->firmware, APP_BASE,
                               FLASH_SIZE - APP_BASE, &error_fatal);
    if (size <= 0) {
        error_report("vex-motor: could not load %s", machine->firmware);
        exit(1);
    }
    rom_add_blob_fixed("vex-motor.iap", iap_stub, sizeof(iap_stub), IAP_ENTRY);

    armv7m_load_kernel(ARMV7M(armv7m)->cpu, NULL, 0, FLASH_SIZE);
}

static void vex_motor_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m0"),
        NULL
    };

    mc->desc = "VEX V5 smart motor (NXP LPC824, Cortex-M0+)";
    mc->init = vex_motor_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m0");
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("vex-motor", vex_motor_machine_init)
