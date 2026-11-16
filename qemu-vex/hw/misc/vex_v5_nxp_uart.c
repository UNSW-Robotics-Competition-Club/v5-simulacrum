/*
 * VEX V5 brain NXP co-processor, UART side (PS UART1, 115200 8N1).
 *
 * A chardev backend the machine attaches to UART1 when no -serial is given
 * for it. Protocol (notes/smartport-tx.md section 4):
 *   brain -> NXP  16 bytes "17 C9 <cmd> <args...>", no CRC
 *                 (22-byte "17 C8 ..." pass-through frames are ignored)
 *   NXP -> brain  16 bytes "17 C9 <cmd> ...", CRC16-CCITT-FALSE over
 *                 bytes 0..13, little-endian at 14..15
 * Answers cmd 0x20 (status/version: leaves fail-safe mode) and 0x23
 * (battery: "battery-volts" / "battery-amps", settable at run time, e.g. by
 * the robot sim's battery model over QMP at /chardevs/vex-nxp).
 *
 * The battery pack itself (device type 14, the brain's PortRecord 23) talks
 * through the NXP too: once the brain has the 0x20 reply, every 20 ms we
 * pass on a pack packet as "17 C8 26 10 <16 bytes> <crc16 LE over 0..19>",
 * a 0x11 status (percent, mV, mA, deg C) and every 5th time a 0x12 info
 * (notes/battery-protocol.md). These drive the brain's battery percent and
 * what the SDK reads; "battery-percent" and "battery-temp" set them, with
 * battery-volts / battery-amps.
 *
 * "17 C9 20 00 00" is the brain's power-off request (VEXos sends
 * it after "SendNXPPowerOff", a ~3 s hold of the front button with no
 * program running): the NXP cuts power, so QEMU shuts down. Everything else
 * needs no reply.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "chardev/char.h"
#include "system/runstate.h"
#include "qapi/visitor.h"
#include "qom/object.h"

#define TYPE_CHARDEV_VEX_NXP "chardev-vex-nxp"
OBJECT_DECLARE_SIMPLE_TYPE(VexNxpChardev, CHARDEV_VEX_NXP)

#define NXP_SYNC0          0x17
#define NXP_SYNC_SHORT     0xC9
#define NXP_SYNC_LONG      0xC8
#define NXP_FRAME_SHORT    16
#define NXP_FRAME_LONG     22

#define NXP_CMD_STATUS     0x20
#define NXP_CMD_BATTERY    0x23

/* App version 3.63.255.255 = 0xFFFF: never older than the DEVFRM01 asset. */
#define NXP_VER_MAJOR      0x03
#define NXP_VER_MINOR      0x3F
#define NXP_VER_BUILD      0xFF
#define NXP_VER_PATCH      0xFF
#define NXP_BATTERY_PRESENT 1

/* Battery scale from nxpParseRxByte: A = raw * 16.12e-3, V = raw * 9.7333e-3 */
#define NXP_AMPS_PER_LSB   0.01612
#define NXP_VOLTS_PER_LSB  0.0097333

#define NXP_CMD_PASSTHRU   0x26
#define PACK_STATUS        0x11
#define PACK_INFO          0x12
#define PACK_VERSION       0x19      /* DEVFRM14: at least the brain's minimum */
#define PACK_ID            0x4001
#define PACK_LEN           16
#define PACK_PERIOD_NS     (20 * SCALE_MS)
#define PACK_INFO_EVERY    5

#define REPLY_DELAY_NS     (2 * SCALE_MS)
#define TXQ_SIZE           256

struct VexNxpChardev {
    Chardev parent;
    QEMUTimer *timer;

    uint8_t rx[NXP_FRAME_LONG];
    int rx_len;
    int rx_need;

    uint8_t txq[TXQ_SIZE];
    int txq_len;

    double battery_volts;
    double battery_amps;
    double battery_percent;
    double battery_temp;

    QEMUTimer *pack_timer;
    int pack_ticks;
};

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

static void nxp_queue_bytes(VexNxpChardev *s, uint8_t *frame, int len)
{
    uint16_t crc = crc16_ccitt_false(frame, len - 2);

    frame[len - 2] = crc;
    frame[len - 1] = crc >> 8;
    if (s->txq_len + len > TXQ_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "vex-nxp: reply queue full\n");
        return;
    }
    memcpy(s->txq + s->txq_len, frame, len);
    s->txq_len += len;
    if (!timer_pending(s->timer)) {
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + REPLY_DELAY_NS);
    }
}

static void nxp_queue_frame(VexNxpChardev *s, uint8_t *frame)
{
    nxp_queue_bytes(s, frame, NXP_FRAME_SHORT);
}

static int clamp_int(double v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : (int)(v + (v < 0 ? -0.5 : 0.5));
}

/* One pack packet, CRC big-endian over 0..13, wrapped for the pass-through. */
static void nxp_send_pack(VexNxpChardev *s)
{
    uint8_t f[NXP_FRAME_LONG] = {
        NXP_SYNC0, NXP_SYNC_LONG, NXP_CMD_PASSTHRU, PACK_LEN,
    };
    uint8_t *pk = f + 4;
    uint16_t crc;

    if (s->pack_ticks++ % PACK_INFO_EVERY == PACK_INFO_EVERY - 1) {
        pk[0] = PACK_INFO;
        pk[1] = PACK_VERSION;
        pk[2] = PACK_ID & 0xff;
        pk[3] = PACK_ID >> 8;
    } else {
        int mv = clamp_int(s->battery_volts * 1000, 0, 0xffff);
        /* the SDK negates it: a discharging pack reports negative mA */
        int ma = clamp_int(-s->battery_amps * 1000, -0x8000, 0x7fff);

        pk[0] = PACK_STATUS;
        pk[2] = clamp_int(s->battery_percent, 0, 100);
        pk[8] = mv;
        pk[9] = mv >> 8;
        pk[10] = ma;
        pk[11] = ma >> 8;
        pk[12] = clamp_int(s->battery_temp, 0, 255);
    }
    crc = crc16_ccitt_false(pk, PACK_LEN - 2);
    pk[14] = crc >> 8;
    pk[15] = crc;
    nxp_queue_bytes(s, f, NXP_FRAME_LONG);
}

static void nxp_pack_tick(void *opaque)
{
    VexNxpChardev *s = opaque;

    nxp_send_pack(s);
    timer_mod(s->pack_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + PACK_PERIOD_NS);
}

static void nxp_reply_status(VexNxpChardev *s)
{
    uint8_t f[NXP_FRAME_SHORT] = {
        NXP_SYNC0, NXP_SYNC_SHORT, NXP_CMD_STATUS,
        NXP_VER_MAJOR, NXP_VER_MINOR, NXP_VER_BUILD, NXP_VER_PATCH,
        0, NXP_BATTERY_PRESENT,
    };

    nxp_queue_frame(s, f);
}

static void nxp_reply_battery(VexNxpChardev *s)
{
    uint16_t amps = s->battery_amps / NXP_AMPS_PER_LSB + 0.5;
    uint16_t volts = s->battery_volts / NXP_VOLTS_PER_LSB + 0.5;
    uint8_t f[NXP_FRAME_SHORT] = {
        NXP_SYNC0, NXP_SYNC_SHORT, NXP_CMD_BATTERY, 0, 0, 0,
        amps, amps >> 8, volts, volts >> 8,
    };

    nxp_queue_frame(s, f);
}

static void nxp_handle_frame(VexNxpChardev *s)
{
    if (s->rx[1] != NXP_SYNC_SHORT) {
        return;
    }
    qemu_log_mask(LOG_UNIMP, "vex-nxp: cmd 0x%02x %02x %02x\n",
                  s->rx[2], s->rx[3], s->rx[4]);
    switch (s->rx[2]) {
    case NXP_CMD_STATUS:
        if (s->rx[3] || s->rx[4]) {
            nxp_reply_status(s);
            if (!timer_pending(s->pack_timer)) {
                /* the brain parses pass-through frames from now on */
                timer_mod(s->pack_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                          + PACK_PERIOD_NS);
            }
        } else {
            /* "17 C9 20 00 00": power off (the brain's last words) */
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    case NXP_CMD_BATTERY:
        nxp_reply_battery(s);
        break;
    default:
        break;
    }
}

static void nxp_rx_byte(VexNxpChardev *s, uint8_t c)
{
    if (s->rx_len == 0) {
        if (c == NXP_SYNC0) {
            s->rx[s->rx_len++] = c;
        }
        return;
    }
    if (s->rx_len == 1) {
        if (c == NXP_SYNC_SHORT) {
            s->rx_need = NXP_FRAME_SHORT;
        } else if (c == NXP_SYNC_LONG) {
            s->rx_need = NXP_FRAME_LONG;
        } else {
            s->rx_len = c == NXP_SYNC0;
            return;
        }
    }
    s->rx[s->rx_len++] = c;
    if (s->rx_len == s->rx_need) {
        nxp_handle_frame(s);
        s->rx_len = 0;
    }
}

static int nxp_chr_write(Chardev *chr, const uint8_t *buf, int len)
{
    VexNxpChardev *s = CHARDEV_VEX_NXP(chr);
    int i;

    for (i = 0; i < len; i++) {
        nxp_rx_byte(s, buf[i]);
    }
    return len;
}

/* Feed queued replies to the UART as fast as its FIFO takes them. */
static void nxp_flush(void *opaque)
{
    VexNxpChardev *s = opaque;
    Chardev *chr = CHARDEV(s);
    int n = MIN(qemu_chr_be_can_write(chr), s->txq_len);

    if (n > 0) {
        qemu_chr_be_write(chr, s->txq, n);
        memmove(s->txq, s->txq + n, s->txq_len - n);
        s->txq_len -= n;
    }
    if (s->txq_len) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
    }
}

static void nxp_get_double(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    visit_type_number(v, name, opaque, errp);
}

static void nxp_set_double(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    double value;

    if (visit_type_number(v, name, &value, errp)) {
        *(double *)opaque = value;
    }
}

static void nxp_chr_init(Object *obj)
{
    VexNxpChardev *s = CHARDEV_VEX_NXP(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nxp_flush, s);
    s->pack_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nxp_pack_tick, s);
    s->battery_volts = 12.0;
    s->battery_amps = 1.6;
    s->battery_percent = 100.0;
    s->battery_temp = 25.0;
    object_property_add(obj, "battery-volts", "number", nxp_get_double,
                        nxp_set_double, NULL, &s->battery_volts);
    object_property_add(obj, "battery-amps", "number", nxp_get_double,
                        nxp_set_double, NULL, &s->battery_amps);
    object_property_add(obj, "battery-percent", "number", nxp_get_double,
                        nxp_set_double, NULL, &s->battery_percent);
    object_property_add(obj, "battery-temp", "number", nxp_get_double,
                        nxp_set_double, NULL, &s->battery_temp);
}

static void nxp_chr_finalize(Object *obj)
{
    VexNxpChardev *s = CHARDEV_VEX_NXP(obj);

    timer_del(s->timer);
    timer_free(s->timer);
    timer_del(s->pack_timer);
    timer_free(s->pack_timer);
}

static void nxp_chr_class_init(ObjectClass *oc, const void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);

    cc->chr_write = nxp_chr_write;
}

static const TypeInfo nxp_chr_info = {
    .name = TYPE_CHARDEV_VEX_NXP,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(VexNxpChardev),
    .instance_init = nxp_chr_init,
    .instance_finalize = nxp_chr_finalize,
    .class_init = nxp_chr_class_init,
};

static void nxp_chr_register_types(void)
{
    type_register_static(&nxp_chr_info);
}

type_init(nxp_chr_register_types)
