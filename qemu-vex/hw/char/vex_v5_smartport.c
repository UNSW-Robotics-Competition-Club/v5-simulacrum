/*
 * VEX V5 brain smart-port UART: a Xilinx UartLite extended by VEX's PL
 * with a packet engine (notes/smartport-tx.md section 1).
 *
 *   +0x00 R  RX FIFO byte
 *   +0x04 W  TX FIFO byte (goes straight to the chardev)
 *   +0x08 R  STAT: b0 RX valid, b2 TX empty, b4 interrupts enabled,
 *            b11 / b10 / b1 a whole 16 / 32 / 64-byte packet is waiting,
 *            b14 TX slot open
 *   +0x0C W  CTRL: b0 reset TX FIFO, b1 reset RX FIFO, b4 enable interrupt
 *   +0x10    smart-mode config (4), stored only
 *   +0x14    b4 enable, b1:0 mode: 1 = smart packet engine (CPU0 init
 *            0x11), 2 = raw generic serial (CPU1 0x12). Going generic, CPU0
 *            zeroes it and masks its INTC0 source (devSetPortGenericType),
 *            then CPU1 writes mode + enable (devGenericSerialSetMode,
 *            devGenericSerialSetFlag10) and takes the port's RX interrupt
 *            on INTC1 (uartRxTxInterruptHandler 0x0343DCCC)
 *   +0x18    TX arm: nonzero = open a TX slot after that many 100 MHz
 *            ticks (default "uartDelay" 0x4C48); 0 = close it
 *
 * The brain only answers: a device sends a CRC-valid 16-byte frame, the
 * brain arms +0x18, the slot opens (STAT b14 + IRQ), and the INTC ISR at
 * MainLoop 0x01035914 writes the 16-byte reply and zeroes +0x18.
 *
 * MainLoop's poll drains every RX byte once a packet-ready bit is set, so
 * only bytes of complete frames are visible. In smart mode (+0x14 != 0) a
 * frame is 16, 32 or 64 bytes (devices negotiate the length; the IMU uses
 * 64) ending in a big-endian CRC16-CCITT-FALSE; frames are delimited by
 * checking it at each length, dropping a byte only once 64 bytes match none.
 * Once a device has negotiated a longer length (its "13 <len>" answered by
 * the brain's "02 13", both snooped here), that length is tried first, so a
 * long frame whose first 16 bytes happen to pass the CRC (~2/65536) isn't
 * split; a 16-byte "0A"/"0F" announce (the device restarted) still is.
 * On hardware the bus slot timing keeps the stream aligned. Here device
 * frames arrive on the host's schedule, and an RX FIFO reset (CTRL b1) in
 * the middle of one used to leave the port misaligned until it timed out.
 * Generic-serial ports (+0x14 mode != 1) see every byte as it comes,
 * and each arrival pulses the second IRQ output (CPU1's INTC1).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "qom/object.h"
#include "hw/core/cpu.h"

#define TYPE_VEX_V5_SMARTPORT "vex-v5-smartport"
OBJECT_DECLARE_SIMPLE_TYPE(VexV5SmartPortState, VEX_V5_SMARTPORT)

#define R_RX        0x00
#define R_TX        0x04
#define R_STAT      0x08
#define R_CTRL      0x0C
#define R_MODE      0x10
#define R_ENGINE    0x14
#define R_ARM       0x18

#define STAT_RX_VALID     (1 << 0)
#define STAT_RX_PKT64     (1 << 1)
#define STAT_TX_EMPTY     (1 << 2)
#define STAT_INTR_EN      (1 << 4)
#define STAT_RX_PKT32     (1 << 10)
#define STAT_RX_PKT16     (1 << 11)
#define STAT_TX_SLOT      (1 << 14)

#define CTRL_RST_TX       (1 << 0)
#define CTRL_RST_RX       (1 << 1)
#define CTRL_INTR_EN      (1 << 4)

#define ENGINE_MODE_MASK  0x3
#define ENGINE_MODE_SMART 0x1

#define PL_CLOCK_HZ       (100 * 1000 * 1000)
#define RX_FIFO_SIZE      1024

struct VexV5SmartPortState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;          /* CPU0: INTC0 source = port */
    qemu_irq irq_cpu1;     /* CPU1: INTC1 source = port + 2 (generic serial) */
    CharFrontend chr;
    QEMUTimer *slot_timer;

    uint8_t rx[RX_FIFO_SIZE];
    uint32_t rx_len;       /* bytes buffered */
    uint32_t rx_visible;   /* leading bytes released to the brain */

    uint32_t ctrl;
    uint32_t mode;
    uint32_t engine;
    uint32_t arm;
    bool tx_slot;

    uint8_t tx_head[2];    /* first bytes the brain wrote in this TX slot */
    uint8_t tx_count;
    uint8_t proposed_len;  /* from the device's "13 <len>" */
    uint8_t link_len;      /* agreed by the brain's "02 13"; 16 at start */
    bool cpu1_owner;       /* CPU1 last wrote +0x14 (see smartport_owner_irq) */
    uint32_t index;        /* smart port number (1-based), for debugging */
};

/*
 * A user program can run its own packet engine on a generic-serial port:
 * the field-control app switches a port to 32-byte packets with
 * vexDeviceGenericSerialEnable(port, ..|5), and devGenericSerialSetMode on
 * CPU1 writes the same 0x11 as CPU0's smart-port init. CPU0 masked the
 * port's INTC0 source when it went generic, so the port's interrupts must
 * follow the CPU that configured it, not the mode value.
 */
static qemu_irq smartport_owner_irq(VexV5SmartPortState *s)
{
    return s->cpu1_owner ? s->irq_cpu1 : s->irq;
}

static void smartport_update_irq(VexV5SmartPortState *s)
{
    qemu_set_irq(smartport_owner_irq(s), s->tx_slot && (s->ctrl & CTRL_INTR_EN));
}

/*
 * A whole frame arrived: pulse the (edge-triggered) INTC input so the ISR
 * stamps the RX time. MainLoop drops 32/64-byte frames that it handles
 * more than ~2.3 ms after that stamp (devIsPortRecentlyUpdated).
 */
static void smartport_rx_event(VexV5SmartPortState *s)
{
    if ((s->ctrl & CTRL_INTR_EN) && !s->tx_slot) {
        qemu_irq_raise(smartport_owner_irq(s));
        qemu_irq_lower(smartport_owner_irq(s));
    }
}

static bool smartport_generic(VexV5SmartPortState *s)
{
    return (s->engine & ENGINE_MODE_MASK) != ENGINE_MODE_SMART;
}

/* Generic serial: CPU1's UartLite ISR drains the RX FIFO on each pulse. */
static void smartport_generic_rx_event(VexV5SmartPortState *s)
{
    if (s->ctrl & CTRL_INTR_EN) {
        qemu_irq_raise(s->irq_cpu1);
        qemu_irq_lower(s->irq_cpu1);
    }
}

static uint32_t smartport_stat(VexV5SmartPortState *s)
{
    uint32_t v = STAT_TX_EMPTY;

    if (s->rx_visible) {
        v |= STAT_RX_VALID;
    }
    if (s->rx_visible >= 16) {
        v |= STAT_RX_PKT16;
    }
    if (s->rx_visible >= 32) {
        v |= STAT_RX_PKT32;
    }
    if (s->rx_visible >= 64) {
        v |= STAT_RX_PKT64;
    }
    if (s->ctrl & CTRL_INTR_EN) {
        v |= STAT_INTR_EN;
    }
    if (s->tx_slot) {
        v |= STAT_TX_SLOT;
    }
    return v;
}

static uint16_t crc16_ccitt_false(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    int i, b;

    for (i = 0; i < len; i++) {
        crc ^= buf[i] << 8;
        for (b = 0; b < 8; b++) {
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

static void smartport_drop_rx(VexV5SmartPortState *s, uint32_t at)
{
    memmove(s->rx + at, s->rx + at + 1, s->rx_len - at - 1);
    s->rx_len--;
}

/*
 * Length of the valid frame at @f (16, 32 or 64 bytes), 0 if none fits in
 * @avail bytes yet, or -1 if no frame can start here.
 */
static bool smartport_crc_ok(const uint8_t *f, uint32_t len)
{
    return crc16_ccitt_false(f, len - 2) == lduw_be_p(f + len - 2);
}

static int smartport_frame_len(VexV5SmartPortState *s, const uint8_t *f,
                               uint32_t avail)
{
    static const uint32_t lens[] = { 16, 32, 64 };
    int i;

    if (s->link_len > 16) {
        if (avail >= 16 && (f[0] == 0x0A || f[0] == 0x0F) &&
            smartport_crc_ok(f, 16)) {
            return 16;                    /* announce: the device restarted */
        }
        if (avail < s->link_len) {
            return 0;
        }
        if (smartport_crc_ok(f, s->link_len)) {
            return s->link_len;
        }
    }
    for (i = 0; i < ARRAY_SIZE(lens); i++) {
        if (avail < lens[i]) {
            return 0;
        }
        if (smartport_crc_ok(f, lens[i])) {
            return lens[i];
        }
    }
    return -1;
}

/* Release buffered bytes to the brain: whole valid frames, or raw bytes. */
static void smartport_frame_rx(VexV5SmartPortState *s)
{
    uint32_t dropped = 0;
    uint32_t before = s->rx_visible;
    int len;

    if (smartport_generic(s)) {
        s->rx_visible = s->rx_len;
        if (s->rx_visible > before) {
            smartport_generic_rx_event(s);
        }
        return;
    }
    while ((len = smartport_frame_len(s, s->rx + s->rx_visible,
                                      s->rx_len - s->rx_visible)) != 0) {
        if (len > 0) {
            const uint8_t *f = s->rx + s->rx_visible;

            if (len == 16 && f[0] == 0x13 &&
                (f[1] == 16 || f[1] == 32 || f[1] == 64)) {
                s->proposed_len = f[1];
            } else if (len == 16 && (f[0] == 0x0A || f[0] == 0x0F)) {
                s->link_len = 16;
                s->proposed_len = 0;
            }
            s->rx_visible += len;
        } else {
            smartport_drop_rx(s, s->rx_visible);
            dropped++;
        }
    }
    if (dropped) {
        qemu_log_mask(LOG_UNIMP, "vex-v5-smartport: resynced, dropped %u "
                      "byte(s)\n", dropped);
    }
    if (s->rx_visible > before) {
        smartport_rx_event(s);
    }
}

static void smartport_reset_rx(VexV5SmartPortState *s)
{
    s->rx_len = 0;
    s->rx_visible = 0;
    qemu_chr_fe_accept_input(&s->chr);
}

static uint8_t smartport_pop_rx(VexV5SmartPortState *s)
{
    uint8_t c;

    if (!s->rx_visible) {
        return 0;
    }
    c = s->rx[0];
    memmove(s->rx, s->rx + 1, --s->rx_len);
    s->rx_visible--;
    qemu_chr_fe_accept_input(&s->chr);
    return c;
}

static void smartport_slot_open(void *opaque)
{
    VexV5SmartPortState *s = opaque;

    s->tx_slot = true;
    s->tx_count = 0;                      /* the brain's next frame starts */
    smartport_update_irq(s);
}

static void smartport_arm(VexV5SmartPortState *s, uint32_t ticks)
{
    s->arm = ticks;
    if (ticks) {
        timer_mod(s->slot_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  muldiv64(ticks, NANOSECONDS_PER_SECOND, PL_CLOCK_HZ));
    } else {
        timer_del(s->slot_timer);
        s->tx_slot = false;
        smartport_update_irq(s);
    }
}

static uint64_t smartport_read(void *opaque, hwaddr offset, unsigned size)
{
    VexV5SmartPortState *s = opaque;

    switch (offset) {
    case R_RX:
        return smartport_pop_rx(s);
    case R_STAT:
        return smartport_stat(s);
    case R_MODE:
        return s->mode;
    case R_ENGINE:
        return s->engine;
    case R_ARM:
        return s->arm;
    default:
        return 0;
    }
}

static void smartport_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    VexV5SmartPortState *s = opaque;
    uint8_t c = value;

    switch (offset) {
    case R_TX:
        if (s->tx_count < 2) {
            s->tx_head[s->tx_count++] = c;
            if (s->tx_count == 2 && s->tx_head[0] == 0x02 &&
                s->tx_head[1] == 0x13 && s->proposed_len) {
                s->link_len = s->proposed_len;   /* brain accepted "13 <len>" */
            }
        }
        qemu_chr_fe_write_all(&s->chr, &c, 1);
        break;
    case R_CTRL:
        if (value & CTRL_RST_RX) {
            smartport_reset_rx(s);
        }
        s->ctrl = value & CTRL_INTR_EN;
        smartport_update_irq(s);
        break;
    case R_MODE:
        s->mode = value;
        break;
    case R_ENGINE:
        if (s->cpu1_owner != (current_cpu && current_cpu->cpu_index == 1)) {
            /* drop a level held on the other CPU's line before switching */
            qemu_irq_lower(smartport_owner_irq(s));
            s->cpu1_owner = !s->cpu1_owner;
        }
        s->engine = value;
        smartport_frame_rx(s);
        smartport_update_irq(s);
        break;
    case R_ARM:
        smartport_arm(s, value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "vex-v5-smartport: bad write 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", offset, value);
        break;
    }
}

static const MemoryRegionOps smartport_ops = {
    .read = smartport_read,
    .write = smartport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static int smartport_can_receive(void *opaque)
{
    VexV5SmartPortState *s = opaque;

    return RX_FIFO_SIZE - s->rx_len;
}

static void smartport_receive(void *opaque, const uint8_t *buf, int size)
{
    VexV5SmartPortState *s = opaque;

    size = MIN(size, RX_FIFO_SIZE - s->rx_len);
    memcpy(s->rx + s->rx_len, buf, size);
    s->rx_len += size;
    smartport_frame_rx(s);
}

static void smartport_reset(DeviceState *dev)
{
    VexV5SmartPortState *s = VEX_V5_SMARTPORT(dev);

    s->ctrl = 0;
    s->mode = 0;
    s->engine = 0;
    s->arm = 0;
    s->tx_slot = false;
    s->tx_count = 2;
    s->proposed_len = 0;
    s->link_len = 16;
    s->cpu1_owner = false;
    timer_del(s->slot_timer);
    smartport_reset_rx(s);
    smartport_update_irq(s);
}

static void smartport_realize(DeviceState *dev, Error **errp)
{
    VexV5SmartPortState *s = VEX_V5_SMARTPORT(dev);

    s->slot_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, smartport_slot_open, s);
    qemu_chr_fe_set_handlers(&s->chr, smartport_can_receive,
                             smartport_receive, NULL, NULL, s, NULL, true);
}

static void smartport_init(Object *obj)
{
    VexV5SmartPortState *s = VEX_V5_SMARTPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &smartport_ops, s,
                          TYPE_VEX_V5_SMARTPORT, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq_cpu1);
}

static const Property smartport_props[] = {
    DEFINE_PROP_CHR("chardev", VexV5SmartPortState, chr),
    DEFINE_PROP_UINT32("index", VexV5SmartPortState, index, 0),
};

static void smartport_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = smartport_realize;
    device_class_set_legacy_reset(dc, smartport_reset);
    device_class_set_props(dc, smartport_props);
    dc->desc = "VEX V5 smart-port UART";
}

static const TypeInfo smartport_info = {
    .name = TYPE_VEX_V5_SMARTPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VexV5SmartPortState),
    .instance_init = smartport_init,
    .class_init = smartport_class_init,
};

static void smartport_register_types(void)
{
    type_register_static(&smartport_info);
}

type_init(smartport_register_types)
