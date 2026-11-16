/*
 * LPC82x smart-port link (machine vex-motor, see lpc82x.c): USART0 with a
 * paced transmitter, the DMA controller that feeds it, and the CRC engine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "lpc82x.h"

/* ---- DMA ------------------------------------------------------------- */

static void dma_update_irq(Lpc82xState *s)
{
    qemu_set_irq(s->irq[IRQ_DMA],
                 ((s->dma_inta | s->dma_intb | s->dma_errint) & s->dma_inten)
                 != 0);
}

static void dma_load_descriptor(Lpc82xState *s, unsigned n, hwaddr addr,
                                bool with_cfg)
{
    uint32_t d[4];
    DmaChannel *c = &s->dma[n];

    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       d, sizeof(d));
    if (with_cfg) {
        c->xfercfg = le32_to_cpu(d[0]);
        c->remaining = ((c->xfercfg & XF_COUNT_MASK) >> XF_COUNT_SHIFT) + 1;
    }
    c->srcend = le32_to_cpu(d[1]);
    c->dstend = le32_to_cpu(d[2]);
    c->link = le32_to_cpu(d[3]);
}

static void dma_start(Lpc82xState *s, unsigned n)
{
    DmaChannel *c = &s->dma[n];

    dma_load_descriptor(s, n, (s->srambase & ~0x1FFu) + 16 * n, false);
    DBG("dma_start %u xf=%08x rem=%u", n, c->xfercfg, c->remaining);
    c->active = true;
    if (c->xfercfg & XF_SWTRIG) {
        s->dma_trig |= 1u << n;
    }
}

static bool dma_request(Lpc82xState *s, unsigned n)
{
    if (!(s->dma[n].cfg & CFG_PERIPHREQEN)) {
        return true;
    }
    switch (n) {
    case DMA_REQ_U0_RX:
        return s->rx_full && (s->u_cfg & U_ENABLE);
    case DMA_REQ_U0_TX:
        return (s->u_cfg & U_ENABLE) && !s->tx_hold_full;
    default:
        return false;
    }
}

static uint8_t usart_pop(Lpc82xState *s);
static void usart_tx(Lpc82xState *s, uint8_t b);

/*
 * The USART data registers are reached directly: going through the address
 * space from inside one of this device's MMIO handlers trips QEMU's
 * re-entrancy guard, which drops the access.
 */
static void dma_bus_read(Lpc82xState *s, uint32_t addr, uint8_t *buf,
                         unsigned width)
{
    if (addr == USART0_BASE + U_RXDAT) {
        memset(buf, 0, width);
        buf[0] = usart_pop(s);
        return;
    }
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, width);
}

static void dma_bus_write(Lpc82xState *s, uint32_t addr, const uint8_t *buf,
                          unsigned width)
{
    if (addr == USART0_BASE + U_TXDAT) {
        usart_tx(s, buf[0]);
        return;
    }
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, width);
}

static uint32_t dma_addr(uint32_t end, uint32_t remaining, unsigned inc,
                         unsigned width)
{
    unsigned step = inc ? (1u << (inc - 1)) * width : 0;

    return end - (remaining - 1) * step;
}

static void dma_complete(Lpc82xState *s, unsigned n)
{
    DmaChannel *c = &s->dma[n];
    uint32_t cfg = c->xfercfg;

    DBG("dma%u done", n);
    if (cfg & XF_SETINTA) {
        s->dma_inta |= 1u << n;
    }
    if (cfg & XF_SETINTB) {
        s->dma_intb |= 1u << n;
    }
    if (cfg & XF_CLRTRIG) {
        s->dma_trig &= ~(1u << n);
    }
    c->active = false;
    if ((cfg & XF_RELOAD) && c->link) {
        dma_load_descriptor(s, n, c->link, true);
        c->active = (c->xfercfg & XF_CFGVALID) != 0;
        if (c->xfercfg & XF_SWTRIG) {
            s->dma_trig |= 1u << n;
        }
    }
}

static void dma_service(Lpc82xState *s)
{
    bool progress = true;

    if (s->dma_busy || !(s->dma_ctrl & 1)) {
        return;
    }
    s->dma_busy = true;
    while (progress) {
        progress = false;
        for (unsigned n = 0; n < NUM_DMA; n++) {
            DmaChannel *c = &s->dma[n];
            unsigned width, srcinc, dstinc;
            uint8_t buf[4];

            if (!c->active || !c->remaining ||
                !(s->dma_enable & (1u << n)) ||
                !(s->dma_trig & (1u << n)) || !dma_request(s, n)) {
                continue;
            }
            width = 1u << ((c->xfercfg >> 8) & 3);
            srcinc = (c->xfercfg >> 12) & 3;
            dstinc = (c->xfercfg >> 14) & 3;
            dma_bus_read(s, dma_addr(c->srcend, c->remaining, srcinc, width),
                         buf, width);
            dma_bus_write(s, dma_addr(c->dstend, c->remaining, dstinc, width),
                          buf, width);
            if (--c->remaining == 0) {
                dma_complete(s, n);
            }
            progress = true;
        }
    }
    s->dma_busy = false;
    dma_update_irq(s);
}

static uint64_t dma_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t r = 0;

    if (off >= DMA_CH_BASE && off < DMA_CH_BASE + 0x10 * NUM_DMA) {
        DmaChannel *c = &s->dma[(off - DMA_CH_BASE) / 0x10];
        unsigned n = c - s->dma;

        switch (off & 0xC) {
        case DMA_CH_CFG:
            r = c->cfg;
            break;
        case DMA_CH_CTLSTAT:
            r = ((s->dma_trig >> n) & 1) << 2;
            break;
        case DMA_CH_XFERCFG:
            r = (c->xfercfg & ~XF_COUNT_MASK) |
                (((c->remaining - 1) << XF_COUNT_SHIFT) & XF_COUNT_MASK);
            break;
        }
        return field32(r, off, size);
    }
    switch (off & ~3) {
    case DMA_CTRL:
        r = s->dma_ctrl;
        break;
    case DMA_INTSTAT:
        r = (((s->dma_inta | s->dma_intb) & s->dma_inten) ? 2 : 0) |
            ((s->dma_errint & s->dma_inten) ? 4 : 0);
        break;
    case DMA_SRAMBASE:
        r = s->srambase;
        break;
    case DMA_ENABLESET:
        r = s->dma_enable;
        break;
    case DMA_ACTIVE:
        for (unsigned n = 0; n < NUM_DMA; n++) {
            r |= (s->dma[n].active ? 1u : 0) << n;
        }
        r &= s->dma_enable;
        break;
    case DMA_ERRINT:
        r = s->dma_errint;
        break;
    case DMA_INTENSET:
        r = s->dma_inten;
        break;
    case DMA_INTA:
        r = s->dma_inta;
        break;
    case DMA_INTB:
        r = s->dma_intb;
        break;
    }
    return field32(r, off, size);
}

static void dma_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t w = merge32(0, off, v, size);

    if (off >= DMA_CH_BASE && off < DMA_CH_BASE + 0x10 * NUM_DMA) {
        unsigned n = (off - DMA_CH_BASE) / 0x10;
        DmaChannel *c = &s->dma[n];

        switch (off & 0xC) {
        case DMA_CH_CFG:
            c->cfg = merge32(c->cfg, off, v, size);
            break;
        case DMA_CH_XFERCFG:
            c->xfercfg = merge32(c->xfercfg, off, v, size);
            c->remaining = ((c->xfercfg & XF_COUNT_MASK) >> XF_COUNT_SHIFT) + 1;
            if (c->xfercfg & XF_CFGVALID) {
                dma_start(s, n);
            }
            break;
        }
    } else {
        switch (off & ~3) {
        case DMA_CTRL:
            s->dma_ctrl = w & 1;
            break;
        case DMA_SRAMBASE:
            s->srambase = w;
            break;
        case DMA_ENABLESET:
            s->dma_enable |= w;
            break;
        case DMA_ENABLECLR:
            s->dma_enable &= ~w;
            break;
        case DMA_ERRINT:
            s->dma_errint &= ~w;
            break;
        case DMA_INTENSET:
            s->dma_inten |= w;
            break;
        case DMA_INTENCLR:
            s->dma_inten &= ~w;
            break;
        case DMA_INTA:
            s->dma_inta &= ~w;
            break;
        case DMA_INTB:
            s->dma_intb &= ~w;
            break;
        case DMA_SETVALID:
            /* our transfers finish at once: only a fresh count restarts */
            for (unsigned n = 0; n < NUM_DMA; n++) {
                if ((w & (1u << n)) && !s->dma[n].active &&
                    s->dma[n].remaining) {
                    dma_start(s, n);
                }
            }
            break;
        case DMA_SETTRIG:
            s->dma_trig |= w;
            break;
        case DMA_ABORT:
            for (unsigned n = 0; n < NUM_DMA; n++) {
                if (w & (1u << n)) {
                    s->dma[n].active = false;
                }
            }
            break;
        }
    }
    dma_service(s);
    dma_update_irq(s);
}

/* ---- USART0 ---------------------------------------------------------- */

static uint32_t usart_stat(Lpc82xState *s)
{
    uint32_t st = s->u_stat;

    if (!s->tx_hold_full) {
        st |= ST_TXRDY;
        if (!s->tx_busy) {
            st |= ST_TXIDLE;
        }
    }

    return st | (s->rx_full ? ST_RXRDY : ST_RXIDLE);
}

static void usart_update_irq(Lpc82xState *s)
{
    qemu_set_irq(s->irq[IRQ_USART0], (usart_stat(s) & s->u_inten) != 0);
}

/*
 * A frame takes 10 bit times on the wire. Sending it instantly let the
 * brain's reply arrive before the firmware had finished queueing the frame
 * it answers (portQueuePacket sets its "awaiting reply" flag last), and
 * the reply was dropped.
 */
static int64_t usart_byte_ns(Lpc82xState *s)
{
    uint32_t div = rd32(s->syscon, SC_UARTCLKDIV) & 0xFF;
    uint32_t mult = rd32(s->syscon, SC_UARTFRGMULT) & 0xFF;
    uint64_t pclk, baud;

    if (!div) {
        return 10000;                   /* clock off: 1 Mbaud */
    }
    pclk = lpc_main_hz(s) * 256 / div / (256 + mult);
    baud = pclk / ((s->u_brg & 0xFFFF) + 1) / ((s->u_osr & 0xF) + 1);
    return baud ? 10 * NANOSECONDS_PER_SECOND / baud : 10000;
}

static void usart_tx_start(Lpc82xState *s, uint8_t b)
{
    DBG("tx %02x byte %" PRId64 "ns", b, usart_byte_ns(s));
    s->tx_shift = b;
    s->tx_busy = true;
    timer_mod(s->tx_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + usart_byte_ns(s));
}

static void usart_tx(Lpc82xState *s, uint8_t b)
{
    if (!s->tx_busy) {
        usart_tx_start(s, b);
    } else if (!s->tx_hold_full) {
        s->tx_hold = b;
        s->tx_hold_full = true;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "lpc82x usart: TX overrun\n");
    }
}

void lpc82x_usart_tx_done(void *opaque)
{
    Lpc82xState *s = opaque;

    qemu_chr_fe_write_all(&s->port, &s->tx_shift, 1);
    s->tx_busy = false;
    if (s->tx_hold_full) {
        s->tx_hold_full = false;
        usart_tx_start(s, s->tx_hold);
    }
    usart_update_irq(s);
    dma_service(s);
}

int lpc82x_usart_can_receive(void *opaque)
{
    Lpc82xState *s = opaque;

    return (s->u_cfg & U_ENABLE) && !s->rx_full ? 1 : 0;
}

void lpc82x_usart_receive(void *opaque, const uint8_t *buf, int size)
{
    Lpc82xState *s = opaque;

    if (size < 1) {
        return;
    }
    s->rx_byte = buf[0];
    s->rx_full = true;
    DBG("rx %02x cfg=%x dma0 act=%d rem=%u trig=%d", buf[0], s->u_cfg,
        s->dma[0].active, s->dma[0].remaining, (s->dma_trig & 1));
    s->u_stat |= ST_START;
    usart_update_irq(s);
    dma_service(s);
}

static uint8_t usart_pop(Lpc82xState *s)
{
    uint8_t b = s->rx_byte;

    if (s->rx_full) {
        s->rx_full = false;
        qemu_chr_fe_accept_input(&s->port);
    }
    usart_update_irq(s);
    return b;
}

static uint64_t usart_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;

    switch (off & ~3) {
    case U_CFG:
        return s->u_cfg;
    case U_CTL:
        return s->u_ctl;
    case U_STAT:
        return usart_stat(s);
    case U_INTENSET:
        return s->u_inten;
    case U_RXDAT:
    case U_RXDATSTAT:
        DBG("cpu read rxdat full=%d", s->rx_full);
        return usart_pop(s);
    case U_BRG:
        return s->u_brg;
    case U_INTSTAT:
        return usart_stat(s) & s->u_inten;
    case U_OSR:
        return s->u_osr;
    case U_ADDR:
        return s->u_addr;
    }
    return 0;
}

static void usart_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;
    switch (off & ~3) {
    case U_CFG:
        s->u_cfg = v;
        if (s->u_cfg & U_ENABLE) {
            qemu_chr_fe_accept_input(&s->port);
        }
        break;
    case U_CTL:
        s->u_ctl = v;
        break;
    case U_STAT:
        s->u_stat &= ~(v & ST_W1C);
        break;
    case U_INTENSET:
        s->u_inten |= v;
        break;
    case U_INTENCLR:
        s->u_inten &= ~v;
        break;
    case U_TXDAT:
        usart_tx(s, v);
        break;
    case U_BRG:
        s->u_brg = v;
        break;
    case U_OSR:
        s->u_osr = v;
        break;
    case U_ADDR:
        s->u_addr = v;
        break;
    }
    usart_update_irq(s);
    dma_service(s);
}

/* ---- CRC ------------------------------------------------------------- */

static uint32_t crc_step(uint32_t sum, uint8_t byte, unsigned poly)
{
    static const uint32_t polys[3] = { 0x1021, 0x8005, 0x04C11DB7 };
    unsigned bits = poly == 2 ? 32 : 16;
    uint32_t top = 1u << (bits - 1);

    sum ^= (uint32_t)byte << (bits - 8);
    for (int i = 0; i < 8; i++) {
        sum = (sum & top) ? (sum << 1) ^ polys[poly] : sum << 1;
    }
    return bits == 32 ? sum : sum & 0xFFFF;
}

static uint64_t crc_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;

    switch (off & ~3) {
    case CRC_MODE:
        return s->crc_mode;
    case CRC_SUM:
        return s->crc_sum;
    }
    return 0;
}

static void crc_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;
    unsigned poly = s->crc_mode & 3;

    switch (off & ~3) {
    case CRC_MODE:
        s->crc_mode = v;
        if (s->crc_mode & 0x3C) {
            qemu_log_mask(LOG_UNIMP, "lpc82x crc: reverse/complement mode "
                          "0x%x\n", s->crc_mode);
        }
        break;
    case CRC_SEED:
        s->crc_sum = v;
        break;
    case CRC_SUM:
        for (unsigned i = 0; i < size; i++) {
            s->crc_sum = crc_step(s->crc_sum, v >> (8 * i), poly < 3 ? poly : 0);
        }
        break;
    }
}


LPC_OPS(usart);
LPC_OPS(dma);
LPC_OPS(crc);
