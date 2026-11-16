/*
 * VEX V5 brain USB0: the Zynq-7000 USB controller (ChipIdea / ARC core,
 * "XUsbPs") in device mode, with a built-in virtual USB host.
 *
 * QEMU's chipidea model is host-only, and nothing in QEMU can be the host
 * of an emulated *device* controller. So this model is both ends: it
 * implements the device-mode programming model the firmware drives
 * (Zynq TRM ch. 15: dQH list at ENDPOINTLISTADDR, dTD chains, ENDPTPRIME /
 * ENDPTSTAT / ENDPTCOMPLETE, SETUP packets via ENDPTSETUPSTAT), and a
 * small host that resets the bus, enumerates the brain like a PC would,
 * and then bridges the CDC-ACM bulk endpoints to two chardevs:
 *
 *   chardev "sys"   first CDC data interface: VEX system protocol
 *                   (C9 36 B8 47 ... / AA 55 ...), what PROS / VEXcode use
 *   chardev "user"  second CDC data interface: the user program's stdio
 *
 * Endpoints are found by parsing the configuration descriptor the
 * firmware returns, so nothing about VEX's descriptors is hard-coded.
 * See notes/usb-protocol.md for what the firmware expects.
 *
 * Simplifications: a dTD completes in one go (all of an IN dTD is sent;
 * an OUT dTD takes whatever host data is queued, as one short transfer);
 * no isochronous/interrupt endpoint timing, no suspend, high speed only.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_VEX_V5_USBDEV "vex-v5-usbdev"
OBJECT_DECLARE_SIMPLE_TYPE(VexV5UsbDevState, VEX_V5_USBDEV)

/* Registers (offsets from 0xE0002000) */
#define R_ID            0x000
#define R_HWGENERAL     0x004
#define R_HWHOST        0x008
#define R_HWDEVICE      0x00C
#define R_HWTXBUF       0x010
#define R_HWRXBUF       0x014
#define R_CAPLENGTH     0x100
#define R_HCSPARAMS     0x104
#define R_HCCPARAMS     0x108
#define R_DCIVERSION    0x120
#define R_DCCPARAMS     0x124
#define R_USBCMD        0x140
#define R_USBSTS        0x144
#define R_USBINTR       0x148
#define R_FRINDEX       0x14C
#define R_DEVICEADDR    0x154
#define R_EPLISTADDR    0x158
#define R_BURSTSIZE     0x160
#define R_ULPIVIEW      0x170
#define R_PORTSC1       0x184
#define R_OTGSC         0x1A4
#define R_USBMODE       0x1A8
#define R_SETUPSTAT     0x1AC
#define R_PRIME         0x1B0
#define R_FLUSH         0x1B4
#define R_EPSTAT        0x1B8
#define R_COMPLETE      0x1BC
#define R_EPCTRL0       0x1C0
#define NUM_EP          12
#define MMIO_SIZE       0x1000

#define CMD_RS          (1u << 0)
#define CMD_RST         (1u << 1)
#define CMD_SUTW        (1u << 13)

#define STS_UI          (1u << 0)
#define STS_UEI         (1u << 1)
#define STS_PCI         (1u << 2)
#define STS_URI         (1u << 6)
#define STS_SLI         (1u << 8)
#define STS_HCH         (1u << 12)
#define STS_W1C         0x000F01FFu

#define PORTSC_CCS      (1u << 0)
#define PORTSC_PE       (1u << 2)
#define PORTSC_PR       (1u << 8)
#define PORTSC_HSP      (1u << 9)
#define PORTSC_PSPD_HS  (2u << 26)

#define ULPI_RUN        (1u << 30)
#define ULPI_WU         (1u << 31)
#define ULPI_SS         (1u << 27)

#define USBMODE_CM_DEV  2

/* dQH: 64 bytes per endpoint direction, OUT = 2n, IN = 2n + 1 */
#define DQH_SIZE        64
#define DQH_CAPS        0x00
#define DQH_CUR         0x04
#define DQH_NEXT        0x08
#define DQH_TOKEN       0x0C
#define DQH_SETUP       0x28
/* dTD */
#define DTD_NEXT        0x00
#define DTD_TOKEN       0x04
#define DTD_BUF0        0x08
#define DTD_TERMINATE   1u
#define TOK_ACTIVE      0x80u
#define TOK_IOC         (1u << 15)
#define TOK_TOTAL(t)    (((t) >> 16) & 0x7FFF)

#define TICK_NS         (1 * 1000 * 1000)     /* virtual host step, 1 ms */
#define RESET_DELAY     20                    /* ticks from RS=1 to bus reset */
#define CTRL_TIMEOUT    1000                  /* ticks per control transfer */
#define HOST_FIFO       4096
#define CTRL_MAX        512

enum { PORT_SYS, PORT_USER, NUM_PORTS };

typedef enum {
    VH_DETACHED,      /* waiting for USBCMD.RS in device mode */
    VH_RESET,         /* bus reset signalled, waiting a few ticks */
    VH_ENUM,          /* running the enumeration script */
    VH_CONFIGURED,    /* bulk bridging */
} VhState;

typedef enum {
    CT_IDLE,
    CT_SETUP,         /* setup posted, waiting for the device to take it */
    CT_DATA_IN,       /* collecting IN data */
    CT_DATA_OUT,      /* host data queued on EP0 OUT */
    CT_STATUS_IN,     /* waiting for the device's zero-length IN */
    CT_STATUS_OUT,    /* host ZLP queued on EP0 OUT */
    CT_DONE,
} CtPhase;

typedef struct {
    int ep_in, ep_out;           /* endpoint numbers, -1 = not found */
    int comm_if;                 /* CDC comm interface for DTR, -1 = none */
    Fifo8 to_dev;                /* chardev -> bulk OUT */
} BridgePort;

struct VexV5UsbDevState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    CharFrontend chr[NUM_PORTS];

    uint32_t usbcmd, usbsts, usbintr, frindex, deviceaddr, eplistaddr;
    uint32_t burstsize, ulpi, portsc, otgsc, usbmode;
    uint32_t setupstat, epstat, complete;
    uint32_t epctrl[NUM_EP];

    /* virtual host */
    VhState vh;
    int vh_ticks;
    int script_step;
    uint8_t address;

    CtPhase ct;
    int ct_ticks;
    uint8_t ct_setup[8];
    uint8_t ct_in[CTRL_MAX];
    uint32_t ct_in_len;
    uint8_t ct_out[CTRL_MAX];
    uint32_t ct_out_len;
    bool ep0_out_zlp;            /* host ZLP / data waiting on EP0 OUT */
    bool ct_failed;              /* a control transfer timed out */

    uint8_t config_desc[CTRL_MAX];
    uint32_t config_len;
    uint32_t config_total;
    BridgePort port[NUM_PORTS];

    bool in_run;                 /* ep_run is not reentrant (DMA, chardevs) */
    uint8_t xfer[5 * 0x1000];    /* one dTD's data */
};

/* ---------------------------------------------------------------- IRQ */

static void usbdev_update_irq(VexV5UsbDevState *s)
{
    qemu_set_irq(s->irq, !!(s->usbsts & s->usbintr & 0x1FF));
}

static void usbdev_raise(VexV5UsbDevState *s, uint32_t bits)
{
    s->usbsts |= bits;
    usbdev_update_irq(s);
}

/* ---------------------------------------------------------------- memory */

static uint32_t ld32(hwaddr a)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void st32(hwaddr a, uint32_t v)
{
    v = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
}

static hwaddr dqh_addr(VexV5UsbDevState *s, int ep, bool in)
{
    return (s->eplistaddr & ~0x7FFu) + (2 * ep + (in ? 1 : 0)) * DQH_SIZE;
}

/* Bytes a dTD's 5 buffer pages can hold from page 0's offset. */
static uint32_t dtd_span(hwaddr dtd)
{
    return 5 * 0x1000 - (ld32(dtd + DTD_BUF0) & 0xFFF);
}

/*
 * Copy len bytes (at most dtd_span) of a dTD's buffer (5 page pointers:
 * page 0 carries the start offset, pages 1..4 are 4K aligned) to or from
 * data.
 */
static void dtd_buf_rw(hwaddr dtd, uint8_t *data, uint32_t len, bool to_mem)
{
    uint32_t pos = ld32(dtd + DTD_BUF0) & 0xFFF;

    while (len) {
        uint32_t page = pos >> 12;
        uint32_t in_page = pos & 0xFFF;
        uint32_t n = MIN(len, 0x1000 - in_page);
        hwaddr base;

        assert(page <= 4);
        base = (ld32(dtd + DTD_BUF0 + 4 * page) & ~0xFFFu) + in_page;
        if (to_mem) {
            dma_memory_write(&address_space_memory, base, data, n,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_read(&address_space_memory, base, data, n,
                            MEMTXATTRS_UNSPECIFIED);
        }
        data += n;
        pos += n;
        len -= n;
    }
}

/* ---------------------------------------------------------------- endpoints */

static uint32_t ep_bit(int ep, bool in)
{
    return 1u << (ep + (in ? 16 : 0));
}

static void usbdev_host_in(VexV5UsbDevState *s, int ep, const uint8_t *d,
                           uint32_t n);
static bool usbdev_host_out(VexV5UsbDevState *s, int ep, uint8_t *d,
                            uint32_t max, uint32_t *n);
static int bridge_for_ep(VexV5UsbDevState *s, int ep, bool in);

/*
 * Run the endpoint's dTD chain as far as data allows. IN dTDs always
 * complete; OUT dTDs complete only when the host has data (or a ZLP).
 */
static void ep_run(VexV5UsbDevState *s, int ep, bool in)
{
    hwaddr qh = dqh_addr(s, ep, in);
    uint32_t bit = ep_bit(ep, in);
    bool done_any = false, error = false;
    uint8_t *buf = s->xfer;

    if (s->in_run || !s->eplistaddr) {
        return;           /* re-entered via DMA or a chardev; or unconfigured */
    }
    s->in_run = true;
    while (s->epstat & bit) {
        uint32_t dtd = ld32(qh + DQH_NEXT);
        uint32_t tok, total, n = 0;

        if (dtd & DTD_TERMINATE) {
            s->epstat &= ~bit;
            break;
        }
        dtd &= ~0x1Fu;
        tok = ld32(dtd + DTD_TOKEN);
        if (!(tok & TOK_ACTIVE)) {
            /*
             * End of the queued work, as on hardware. The firmware's dTD
             * rings are circular (notes/usb-protocol.md), so walking past
             * an inactive dTD could loop forever.
             */
            s->epstat &= ~bit;
            break;
        }
        total = TOK_TOTAL(tok);
        if (total > dtd_span(dtd)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "vex-usbdev: ep%d%s dTD of %u bytes overruns its pages\n",
                          ep, in ? "in" : "out", total);
            st32(dtd + DTD_TOKEN, (tok & ~TOK_ACTIVE) | 0x40);   /* halted */
            s->epstat &= ~bit;
            error = true;
            break;
        }
        if (in) {
            dtd_buf_rw(dtd, buf, total, false);
            n = total;
            usbdev_host_in(s, ep, buf, n);
        } else {
            if (!usbdev_host_out(s, ep, buf, total, &n)) {
                break;                       /* wait for host data */
            }
            dtd_buf_rw(dtd, buf, n, true);
        }
        tok = (tok & ~(0x7FFFu << 16) & ~0xFFu) | ((TOK_TOTAL(tok) - n) << 16);
        st32(dtd + DTD_TOKEN, tok);
        st32(qh + DQH_CUR, dtd);
        st32(qh + DQH_NEXT, ld32(dtd + DTD_NEXT));
        st32(qh + DQH_TOKEN, tok);
        if (tok & TOK_IOC) {
            s->complete |= bit;
            done_any = true;
        }
    }
    s->in_run = false;
    if (done_any || error) {
        usbdev_raise(s, (done_any ? STS_UI : 0) | (error ? STS_UEI : 0));
    }
    if (!in) {
        int p = bridge_for_ep(s, ep, false);

        if (p >= 0) {
            qemu_chr_fe_accept_input(&s->chr[p]);   /* fifo has room again */
        }
    }
}

static void ep_run_all(VexV5UsbDevState *s)
{
    int ep;

    for (ep = 0; ep < NUM_EP; ep++) {
        if (s->epstat & ep_bit(ep, false)) {
            ep_run(s, ep, false);
        }
        if (s->epstat & ep_bit(ep, true)) {
            ep_run(s, ep, true);
        }
    }
}

/* ---------------------------------------------------------------- host side */

static int bridge_for_ep(VexV5UsbDevState *s, int ep, bool in)
{
    int p;

    for (p = 0; p < NUM_PORTS; p++) {
        if ((in ? s->port[p].ep_in : s->port[p].ep_out) == ep) {
            return p;
        }
    }
    return -1;
}

/*
 * The device took the setup (cleared ENDPTSETUPSTAT): enter the data or
 * status phase. Done inline because the firmware usually clears the bit
 * and primes the next stage in the same ISR, before the next host tick.
 */
static void ct_take_setup(VexV5UsbDevState *s)
{
    uint16_t len = s->ct_setup[6] | s->ct_setup[7] << 8;

    if (s->ct != CT_SETUP || (s->setupstat & 1)) {
        return;
    }
    if (len == 0) {
        s->ct = CT_STATUS_IN;
    } else if (s->ct_setup[0] & 0x80) {
        s->ct = CT_DATA_IN;
    } else {
        s->ct = CT_DATA_OUT;
        s->ep0_out_zlp = true;          /* data stage queued */
    }
}

/* Device sent n bytes on IN endpoint ep. */
static void usbdev_host_in(VexV5UsbDevState *s, int ep, const uint8_t *d,
                           uint32_t n)
{
    int p;

    if (ep == 0) {
        ct_take_setup(s);
        if (s->ct == CT_DATA_IN) {
            uint32_t k = MIN(n, CTRL_MAX - s->ct_in_len);

            memcpy(s->ct_in + s->ct_in_len, d, k);
            s->ct_in_len += k;
        } else if (s->ct == CT_STATUS_IN && n == 0) {
            s->ct = CT_DONE;
        } else if (s->ct == CT_DATA_OUT && n == 0) {
            /*
             * The firmware queues the status ZLP before re-priming EP0 OUT
             * for the data stage (notes/usb-protocol.md 8.2); it never uses
             * the data (line coding), so the transfer is over.
             */
            s->ep0_out_zlp = false;
            s->ct_out_len = 0;
            s->ct = CT_DONE;
        }
        return;
    }
    p = bridge_for_ep(s, ep, true);
    if (p >= 0 && n) {
        /* never block the vCPU: a host that isn't reading loses data */
        qemu_chr_fe_write(&s->chr[p], d, n);
    }
}

/* Device wants up to max bytes on OUT endpoint ep. False = nothing yet. */
static bool usbdev_host_out(VexV5UsbDevState *s, int ep, uint8_t *d,
                            uint32_t max, uint32_t *n)
{
    int p;

    if (ep == 0) {
        ct_take_setup(s);
        if (!s->ep0_out_zlp) {
            return false;
        }
        *n = MIN(max, s->ct_out_len);
        memcpy(d, s->ct_out, *n);
        s->ct_out_len = 0;
        s->ep0_out_zlp = false;
        if (s->ct == CT_STATUS_OUT) {
            s->ct = CT_DONE;
        } else if (s->ct == CT_DATA_OUT) {
            s->ct = CT_STATUS_IN;
        }
        return true;
    }
    p = bridge_for_ep(s, ep, false);
    if (p < 0 || fifo8_is_empty(&s->port[p].to_dev)) {
        return false;
    }
    *n = 0;
    while (*n < max && !fifo8_is_empty(&s->port[p].to_dev)) {
        d[(*n)++] = fifo8_pop(&s->port[p].to_dev);
    }
    return true;
}

/* ---------------------------------------------------------------- enumeration */

static void ct_start(VexV5UsbDevState *s, uint8_t type, uint8_t req,
                     uint16_t value, uint16_t index, uint16_t length,
                     const uint8_t *out)
{
    uint8_t *p = s->ct_setup;
    hwaddr qh = dqh_addr(s, 0, false);

    p[0] = type; p[1] = req;
    p[2] = value; p[3] = value >> 8;
    p[4] = index; p[5] = index >> 8;
    p[6] = length; p[7] = length >> 8;
    s->ct_in_len = 0;
    s->ct_out_len = 0;
    s->ep0_out_zlp = false;
    if (out && length) {
        memcpy(s->ct_out, out, MIN(length, CTRL_MAX));
        s->ct_out_len = MIN(length, CTRL_MAX);
    }
    dma_memory_write(&address_space_memory, qh + DQH_SETUP, p, 8,
                     MEMTXATTRS_UNSPECIFIED);
    s->epstat &= ~(ep_bit(0, false) | ep_bit(0, true));
    s->setupstat |= 1;
    s->ct = CT_SETUP;
    s->ct_ticks = 0;
    usbdev_raise(s, STS_UI);
}

/* Called each tick: advance a control transfer whose setup was taken. */
static void ct_advance(VexV5UsbDevState *s)
{
    uint16_t len = s->ct_setup[6] | s->ct_setup[7] << 8;

    ct_take_setup(s);
    if (s->ct == CT_DATA_IN &&
        (s->ct_in_len >= len || (s->ct_in_len && !(s->epstat & ep_bit(0, true))))) {
        /* all data in (or a short packet ended it): status OUT ZLP */
        s->ct = CT_STATUS_OUT;
        s->ct_out_len = 0;
        s->ep0_out_zlp = true;
    }
    if (s->ct != CT_IDLE && s->ct != CT_DONE && ++s->ct_ticks > CTRL_TIMEOUT) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "vex-usbdev: control %02x %02x timed out in phase %d\n",
                      s->ct_setup[0], s->ct_setup[1], s->ct);
        s->ct = CT_DONE;
        s->ct_failed = true;
    }
}

/* Find CDC data interfaces' bulk endpoints and comm interfaces. */
static void parse_config(VexV5UsbDevState *s)
{
    uint32_t i = 0;
    int cls = -1, ifnum = -1, n_data = 0, last_comm = -1, last_data_if = -1;

    for (int p = 0; p < NUM_PORTS; p++) {
        s->port[p].ep_in = s->port[p].ep_out = s->port[p].comm_if = -1;
    }
    while (i + 2 <= s->config_len && s->config_desc[i] >= 2) {
        const uint8_t *d = &s->config_desc[i];

        if (d[1] == 4 && d[0] >= 9 && i + 9 <= s->config_len) {  /* interface */
            ifnum = d[2];
            cls = d[5];
            if (cls == 0x02) {
                last_comm = ifnum;
            } else if (cls == 0x0A && ifnum != last_data_if) {
                n_data++;            /* alternate settings count once */
                last_data_if = ifnum;
            }
        } else if (d[1] == 5 && d[0] >= 7 && i + 7 <= s->config_len && /* endpoint */
                   cls == 0x0A && n_data >= 1 && n_data <= NUM_PORTS &&
                   (d[3] & 3) == 2) {
            BridgePort *bp = &s->port[n_data - 1];

            if (d[2] & 0x80) {
                bp->ep_in = d[2] & 0xF;
            } else {
                bp->ep_out = d[2] & 0xF;
            }
            bp->comm_if = last_comm;
        }
        i += d[0];
    }
    for (int p = 0; p < NUM_PORTS; p++) {
        qemu_log_mask(LOG_UNIMP, "vex-usbdev: port %d: bulk in %d out %d comm if %d\n",
                      p, s->port[p].ep_in, s->port[p].ep_out, s->port[p].comm_if);
    }
}

/*
 * The enumeration script, one control transfer per step, like a PC:
 * device descriptor, address, device descriptor again, configuration
 * (header then all), set configuration, then CDC line coding + DTR/RTS on
 * each comm interface found.
 */
static bool script_step(VexV5UsbDevState *s)
{
    static const uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };
    int step = s->script_step++;
    int p;

    switch (step) {
    case 0: ct_start(s, 0x80, 6, 0x0100, 0, 64, NULL); return true;
    case 1: ct_start(s, 0x00, 5, s->address, 0, 0, NULL); return true;
    case 2: ct_start(s, 0x80, 6, 0x0100, 0, 18, NULL); return true;
    case 3: ct_start(s, 0x80, 6, 0x0200, 0, 9, NULL); return true;
    case 4: ct_start(s, 0x80, 6, 0x0200, 0, MAX(s->config_total, 9), NULL);
            return true;
    case 5: ct_start(s, 0x00, 9, 1, 0, 0, NULL); return true;
    default:
        /* per port with a comm interface: SET_LINE_CODING, then DTR + RTS */
        step -= 6;
        for (p = 0; p < NUM_PORTS; p++) {
            if (s->port[p].comm_if < 0) {
                continue;
            }
            if (step < 2) {
                break;
            }
            step -= 2;
        }
        if (p >= NUM_PORTS) {
            return false;
        }
        if (step == 0) {
            ct_start(s, 0x21, 0x20, 0, s->port[p].comm_if, 7, line_coding);
        } else {
            ct_start(s, 0x21, 0x22, 0x0003, s->port[p].comm_if, 0, NULL);
        }
        return true;
    }
}

static void script_result(VexV5UsbDevState *s)
{
    int step = s->script_step - 1;

    if (step == 3 && s->ct_in_len >= 4) {
        s->config_total = MIN(s->ct_in[2] | s->ct_in[3] << 8, CTRL_MAX);
    } else if (step == 4) {
        memcpy(s->config_desc, s->ct_in, s->ct_in_len);
        s->config_len = s->ct_in_len;
        parse_config(s);
    }
}

/* The host lets go of the bus: forget the session, wait for a new attach. */
static void vh_detach(VexV5UsbDevState *s)
{
    s->vh = VH_DETACHED;
    s->vh_ticks = 0;
    s->portsc &= ~(PORTSC_CCS | PORTSC_PE | PORTSC_PR | PORTSC_HSP | (3u << 26));
    s->ct = CT_IDLE;
    s->ep0_out_zlp = false;
    s->ct_failed = false;
    for (int p = 0; p < NUM_PORTS; p++) {
        s->port[p].ep_in = s->port[p].ep_out = s->port[p].comm_if = -1;
        fifo8_reset(&s->port[p].to_dev);
    }
}

/* Bus reset: what the controller clears in hardware, then URI to the ISR. */
static void vh_bus_reset(VexV5UsbDevState *s)
{
    s->setupstat = s->epstat = s->complete = 0;
    s->deviceaddr = 0;
    s->config_len = s->config_total = 0;
    s->vh = VH_RESET;
    s->vh_ticks = 0;
    s->portsc |= PORTSC_CCS | PORTSC_PE | PORTSC_PR | PORTSC_HSP |
                 PORTSC_PSPD_HS;
    usbdev_raise(s, STS_URI | STS_PCI);
}

static bool vh_running(VexV5UsbDevState *s)
{
    return (s->usbcmd & CMD_RS) && (s->usbmode & 3) == USBMODE_CM_DEV;
}

static void vh_tick(void *opaque)
{
    VexV5UsbDevState *s = opaque;

    s->frindex = (s->frindex + 8) & 0x3FFF;
    if (!vh_running(s) && s->vh != VH_DETACHED) {
        vh_detach(s);
    }
    switch (s->vh) {
    case VH_DETACHED:
        /* attach once the firmware has set up its dQH list */
        if (vh_running(s) && s->eplistaddr && ++s->vh_ticks >= RESET_DELAY) {
            vh_bus_reset(s);
        }
        break;
    case VH_RESET:
        if (++s->vh_ticks >= 10) {
            s->portsc &= ~PORTSC_PR;
            usbdev_raise(s, STS_PCI);
            s->vh = VH_ENUM;
            s->script_step = 0;
            s->ct = CT_IDLE;
            s->address = 9;
        }
        break;
    case VH_ENUM:
        ct_advance(s);
        if (s->ct_failed) {
            /* like a PC giving up on a device: reset the bus and retry */
            qemu_log_mask(LOG_GUEST_ERROR, "vex-usbdev: re-enumerating\n");
            vh_detach(s);
            break;
        }
        if (s->ct == CT_DONE) {
            script_result(s);
            s->ct = CT_IDLE;
        }
        if (s->ct == CT_IDLE && !script_step(s)) {
            s->vh = VH_CONFIGURED;
            qemu_log_mask(LOG_UNIMP, "vex-usbdev: enumerated\n");
        }
        break;
    case VH_CONFIGURED:
        break;
    }
    if (s->vh != VH_DETACHED) {
        ep_run_all(s);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
}

/* ---------------------------------------------------------------- chardevs */

static int usbdev_can_receive(void *opaque, int p)
{
    VexV5UsbDevState *s = opaque;

    if (s->vh != VH_CONFIGURED || s->port[p].ep_out < 0) {
        return 0;
    }
    return fifo8_num_free(&s->port[p].to_dev);
}

static void usbdev_receive(void *opaque, int p, const uint8_t *buf, int size)
{
    VexV5UsbDevState *s = opaque;

    fifo8_push_all(&s->port[p].to_dev, buf,
                   MIN(size, fifo8_num_free(&s->port[p].to_dev)));
    ep_run(s, s->port[p].ep_out, false);
}

static int can_receive_sys(void *o) { return usbdev_can_receive(o, PORT_SYS); }
static int can_receive_user(void *o) { return usbdev_can_receive(o, PORT_USER); }
static void receive_sys(void *o, const uint8_t *b, int n)
{
    usbdev_receive(o, PORT_SYS, b, n);
}
static void receive_user(void *o, const uint8_t *b, int n)
{
    usbdev_receive(o, PORT_USER, b, n);
}

/* ---------------------------------------------------------------- MMIO */

static void usbdev_soft_reset(VexV5UsbDevState *s)
{
    s->usbcmd = 0x00080000;          /* ITC = 8 microframes (reset value) */
    s->usbsts = STS_HCH;
    s->usbintr = 0;
    s->frindex = 0;
    s->deviceaddr = 0;
    s->eplistaddr = 0;
    s->portsc = 0;
    s->otgsc = 0;
    s->usbmode = 0;
    s->setupstat = s->epstat = s->complete = 0;
    memset(s->epctrl, 0, sizeof(s->epctrl));
    s->epctrl[0] = 0x00800080;       /* EP0 always enabled */
    s->in_run = false;
    vh_detach(s);
    usbdev_update_irq(s);
}

static uint64_t usbdev_read(void *opaque, hwaddr addr, unsigned size)
{
    VexV5UsbDevState *s = opaque;

    switch (addr) {
    case R_ID:          return 0xE441FA05;
    case R_HWGENERAL:   return 0x00000083;
    case R_HWHOST:      return 0x10020001;
    case R_HWDEVICE:    return 0x00000019;    /* device capable, 12 endpoints */
    case R_HWTXBUF:     return 0x80060A10;
    case R_HWRXBUF:     return 0x00000A10;
    case R_CAPLENGTH:   return 0x01000040;
    case R_HCSPARAMS:   return 0x00010011;
    case R_HCCPARAMS:   return 0x00000006;
    case R_DCIVERSION:  return 0x00000001;
    case R_DCCPARAMS:   return 0x0000018C;    /* DC, DEN = 12 */
    case R_USBCMD:      return s->usbcmd;
    case R_USBSTS:      return s->usbsts | ((s->usbcmd & CMD_RS) ? 0 : STS_HCH);
    case R_USBINTR:     return s->usbintr;
    case R_FRINDEX:     return s->frindex;
    case R_DEVICEADDR:  return s->deviceaddr;
    case R_EPLISTADDR:  return s->eplistaddr;
    case R_BURSTSIZE:   return s->burstsize;
    case R_ULPIVIEW:    return s->ulpi;
    case R_PORTSC1:     return s->portsc;
    case R_OTGSC:       return s->otgsc | 0x00000F00;  /* ID, A/B valid, BSE */
    case R_USBMODE:     return s->usbmode;
    case R_SETUPSTAT:   return s->setupstat;
    case R_PRIME:       return 0;                       /* primes take at once */
    case R_FLUSH:       return 0;
    case R_EPSTAT:      return s->epstat;
    case R_COMPLETE:    return s->complete;
    default:
        if (addr >= R_EPCTRL0 && addr < R_EPCTRL0 + 4 * NUM_EP) {
            return s->epctrl[(addr - R_EPCTRL0) / 4];
        }
        qemu_log_mask(LOG_UNIMP, "vex-usbdev: read 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void usbdev_write(void *opaque, hwaddr addr, uint64_t val64,
                         unsigned size)
{
    VexV5UsbDevState *s = opaque;
    uint32_t val = val64;

    switch (addr) {
    case R_USBCMD:
        if (val & CMD_RST) {
            usbdev_soft_reset(s);     /* RST self-clears */
            return;
        }
        if ((s->usbcmd & CMD_RS) && !(val & CMD_RS)) {
            vh_detach(s);             /* soft disconnect, however brief */
        }
        s->usbcmd = val;
        break;
    case R_USBSTS:
        s->usbsts &= ~(val & STS_W1C);
        usbdev_update_irq(s);
        break;
    case R_USBINTR:
        s->usbintr = val;
        usbdev_update_irq(s);
        break;
    case R_FRINDEX:
        s->frindex = val & 0x3FFF;
        break;
    case R_DEVICEADDR:
        s->deviceaddr = val;
        break;
    case R_EPLISTADDR:
        s->eplistaddr = val & ~0x7FFu;
        break;
    case R_BURSTSIZE:
        s->burstsize = val;
        break;
    case R_ULPIVIEW:
        /* PHY register access: complete at once, reads return 0 */
        s->ulpi = (val & ~(ULPI_RUN | ULPI_WU | 0xFF00)) | ULPI_SS;
        break;
    case R_PORTSC1:
        /* PE/CSC-style change bits are W1C; keep connection state */
        s->portsc = (s->portsc & (PORTSC_CCS | PORTSC_PE | PORTSC_PR |
                                  PORTSC_HSP | (3u << 26))) |
                    (val & ~(PORTSC_CCS | PORTSC_PE | PORTSC_PR | PORTSC_HSP |
                             (3u << 26) | 0x2A));
        break;
    case R_OTGSC:
        s->otgsc = val & 0xFF0000FF;
        break;
    case R_USBMODE:
        s->usbmode = val;
        break;
    case R_SETUPSTAT:
        s->setupstat &= ~val;
        break;
    case R_PRIME:
        if (s->eplistaddr) {
            s->epstat |= val & 0x0FFF0FFF;
            ep_run_all(s);
        }
        break;
    case R_FLUSH:
        s->epstat &= ~(val & 0x0FFF0FFF);
        break;
    case R_COMPLETE:
        s->complete &= ~val;
        break;
    default:
        if (addr >= R_EPCTRL0 && addr < R_EPCTRL0 + 4 * NUM_EP) {
            s->epctrl[(addr - R_EPCTRL0) / 4] = val;
            break;
        }
        qemu_log_mask(LOG_UNIMP, "vex-usbdev: write 0x%03" HWADDR_PRIx
                      " = 0x%08x\n", addr, val);
        break;
    }
}

static const MemoryRegionOps usbdev_ops = {
    .read = usbdev_read,
    .write = usbdev_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---------------------------------------------------------------- device */

static void usbdev_reset(DeviceState *dev)
{
    VexV5UsbDevState *s = VEX_V5_USBDEV(dev);

    usbdev_soft_reset(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
}

static void usbdev_realize(DeviceState *dev, Error **errp)
{
    VexV5UsbDevState *s = VEX_V5_USBDEV(dev);

    for (int p = 0; p < NUM_PORTS; p++) {
        fifo8_create(&s->port[p].to_dev, HOST_FIFO);
    }
    qemu_chr_fe_set_handlers(&s->chr[PORT_SYS], can_receive_sys, receive_sys,
                             NULL, NULL, s, NULL, true);
    qemu_chr_fe_set_handlers(&s->chr[PORT_USER], can_receive_user, receive_user,
                             NULL, NULL, s, NULL, true);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vh_tick, s);
}

static void usbdev_init(Object *obj)
{
    VexV5UsbDevState *s = VEX_V5_USBDEV(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &usbdev_ops, s, TYPE_VEX_V5_USBDEV,
                          MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property usbdev_props[] = {
    DEFINE_PROP_CHR("sys", VexV5UsbDevState, chr[PORT_SYS]),
    DEFINE_PROP_CHR("user", VexV5UsbDevState, chr[PORT_USER]),
};

static void usbdev_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = usbdev_realize;
    device_class_set_legacy_reset(dc, usbdev_reset);
    device_class_set_props(dc, usbdev_props);
    dc->desc = "VEX V5 USB0 (Zynq USB, device mode) with a virtual host";
}

static const TypeInfo usbdev_info = {
    .name = TYPE_VEX_V5_USBDEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VexV5UsbDevState),
    .instance_init = usbdev_init,
    .class_init = usbdev_class_init,
};

static void usbdev_register_types(void)
{
    type_register_static(&usbdev_info);
}

type_init(usbdev_register_types)
