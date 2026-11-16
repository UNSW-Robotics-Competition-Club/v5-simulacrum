/*
 * LPC82x timers and analogue (machine vex-motor, see lpc82x.c): the MRT
 * down-counters, the SCT as a PWM generator and ADC sequence A.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "lpc82x.h"

/* ---- MRT ------------------------------------------------------------- */

static void mrt_update_irq(Lpc82xState *s)
{
    bool level = false;

    for (int i = 0; i < NUM_MRT; i++) {
        level |= (s->mrt[i].stat & MRT_INTFLAG) && (s->mrt[i].ctrl & MRT_INTEN);
    }
    qemu_set_irq(s->irq[IRQ_MRT], level);
}

static void mrt_start(Lpc82xState *s, MrtChannel *c, uint32_t load)
{
    if (!load) {
        c->stat &= ~MRT_RUN;
        timer_del(c->timer);
        return;
    }
    c->load = load;
    c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->stat |= MRT_RUN;
    timer_mod(c->timer, c->start_ns + ticks_to_ns(s, load));
}

static uint32_t mrt_value(Lpc82xState *s, MrtChannel *c)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t done;

    if (!(c->stat & MRT_RUN)) {
        return 0;
    }
    done = muldiv64(now - c->start_ns, lpc_sys_hz(s), NANOSECONDS_PER_SECOND);
    return done >= c->load ? 0 : c->load - done;
}

static uint64_t mrt_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t r = 0;

    if (off < 0x10 * NUM_MRT) {
        MrtChannel *c = &s->mrt[off / 0x10];

        switch (off & 0xC) {
        case MRT_INTVAL:
            r = c->intval;
            break;
        case MRT_TIMER:
            r = mrt_value(s, c);
            break;
        case MRT_CTRL:
            r = c->ctrl;
            break;
        case MRT_STAT:
            r = c->stat;
            break;
        }
    } else if ((off & ~3) == MRT_IRQ_FLAG) {
        for (int i = 0; i < NUM_MRT; i++) {
            r |= (s->mrt[i].stat & MRT_INTFLAG) << i;
        }
    } else if ((off & ~3) == MRT_IDLE_CH) {
        r = NUM_MRT << 4;
        for (int i = NUM_MRT - 1; i >= 0; i--) {
            if (!(s->mrt[i].stat & MRT_RUN)) {
                r = i << 4;
            }
        }
    }
    return field32(r, off, size);
}

static void mrt_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;

    if (off < 0x10 * NUM_MRT) {
        MrtChannel *c = &s->mrt[off / 0x10];
        uint32_t w;

        switch (off & 0xC) {
        case MRT_INTVAL:
            w = merge32(c->intval, off, v, size);
            c->intval = w & ~MRT_LOAD;
            if ((w & MRT_LOAD) || !(c->stat & MRT_RUN) || !c->intval) {
                mrt_start(s, c, c->intval);
            }
            break;
        case MRT_CTRL:
            c->ctrl = merge32(c->ctrl, off, v, size) & 7;
            break;
        case MRT_STAT:
            if (merge32(0, off, v, size) & MRT_INTFLAG) {
                c->stat &= ~MRT_INTFLAG;
            }
            break;
        }
    } else if ((off & ~3) == MRT_IRQ_FLAG) {
        uint32_t w = merge32(0, off, v, size);

        for (int i = 0; i < NUM_MRT; i++) {
            if (w & (1u << i)) {
                s->mrt[i].stat &= ~MRT_INTFLAG;
            }
        }
    }
    mrt_update_irq(s);
}

/* ---- ADC ------------------------------------------------------------- */

static void adc_update_irq(Lpc82xState *s)
{
    qemu_set_irq(s->irq[IRQ_ADC_SEQA],
                 (s->adc_flags & FLAG_SEQA_INT) && (s->adc_inten & 1));
}

static void adc_convert_seqa(Lpc82xState *s)
{
    uint32_t chans = s->seqa_ctrl & 0xFFF;

    for (int ch = 0; ch < NUM_ADC; ch++) {
        if (!(chans & (1u << ch))) {
            continue;
        }
        uint32_t ovr = (s->adc_dat[ch] & DAT_VALID) ? DAT_OVERRUN : 0;

        s->adc_dat[ch] = DAT_VALID | ovr | (ch << 26) |
                         ((s->adc_in[ch] & 0xFFF) << 4);
        s->seqa_gdat = s->adc_dat[ch];
    }
    s->adc_flags |= FLAG_SEQA_INT;
    adc_update_irq(s);
}

void lpc82x_adc_trigger(Lpc82xState *s, int source)
{
    if ((s->seqa_ctrl & SEQ_ENA) &&
        ((s->seqa_ctrl >> 12) & 7) == (uint32_t)source) {
        adc_convert_seqa(s);
    }
}

static int64_t sct_period_ns(Lpc82xState *s);

/* Hardware-trigger and burst sampling run off one periodic timer. */
static void adc_schedule(Lpc82xState *s)
{
    int64_t period;

    if (!(s->seqa_ctrl & SEQ_ENA)) {
        timer_del(s->adc_timer);
        return;
    }
    if (s->seqa_ctrl & SEQ_BURST) {
        period = ADC_MIN_PERIOD_NS;
    } else if (((s->seqa_ctrl >> 12) & 7) == ADC_TRIG_SCT_OUT3) {
        period = MAX(sct_period_ns(s), ADC_MIN_PERIOD_NS);
    } else {
        timer_del(s->adc_timer);
        return;
    }
    if (!timer_pending(s->adc_timer)) {
        timer_mod(s->adc_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period);
    }
}

void lpc82x_adc_tick(void *opaque)
{
    Lpc82xState *s = opaque;

    adc_convert_seqa(s);
    adc_schedule(s);
}

static uint64_t adc_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t r = 0;

    switch (off & ~3) {
    case ADC_CTRL:
        r = s->adc_ctrl;
        break;
    case ADC_SEQA_CTRL:
        r = s->seqa_ctrl;
        break;
    case ADC_SEQB_CTRL:
        r = s->seqb_ctrl;
        break;
    case ADC_SEQA_GDAT:
        r = s->seqa_gdat;
        s->seqa_gdat &= ~DAT_VALID;
        break;
    case ADC_INTEN:
        r = s->adc_inten;
        break;
    case ADC_FLAGS:
        r = s->adc_flags;
        break;
    default:
        if (off >= ADC_DAT0 && off < ADC_DAT0 + 4 * NUM_ADC) {
            unsigned ch = (off - ADC_DAT0) / 4;

            r = s->adc_dat[ch];
            s->adc_dat[ch] &= ~(DAT_VALID | DAT_OVERRUN);
        }
    }
    return field32(r, off, size);
}

static void adc_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;

    switch (off & ~3) {
    case ADC_CTRL:
        /* calibration finishes at once */
        s->adc_ctrl = merge32(s->adc_ctrl, off, v, size) & ~ADC_CALMODE;
        break;
    case ADC_SEQA_CTRL:
        s->seqa_ctrl = merge32(s->seqa_ctrl, off, v, size);
        if ((s->seqa_ctrl & SEQ_START) && (s->seqa_ctrl & SEQ_ENA)) {
            adc_convert_seqa(s);
        }
        s->seqa_ctrl &= ~SEQ_START;
        adc_schedule(s);
        break;
    case ADC_SEQB_CTRL:
        s->seqb_ctrl = merge32(s->seqb_ctrl, off, v, size) & ~SEQ_START;
        break;
    case ADC_INTEN:
        s->adc_inten = merge32(s->adc_inten, off, v, size);
        break;
    case ADC_FLAGS:
        s->adc_flags &= ~merge32(0, off, v, size);
        break;
    }
    adc_update_irq(s);
}

/* ---- SCT ------------------------------------------------------------- */

static uint32_t sct_match(Lpc82xState *s, unsigned n)
{
    uint32_t v = rd32(s->sct, SCT_MATCHREL0 + 4 * n);

    return (rd32(s->sct, SCT_CONFIG) & SCT_UNIFY) ? v : (v & 0xFFFF);
}

static int64_t sct_period_ns(Lpc82xState *s)
{
    uint32_t ctrl = rd32(s->sct, SCT_CTRL);
    uint64_t ticks = (uint64_t)(sct_match(s, 0) + 1) *
                     (((ctrl >> 5) & 0xFF) + 1);

    if (ctrl & SCT_BIDIR_L) {
        ticks *= 2;
    }
    return ticks_to_ns(s, ticks);
}

/* Match register of the first state-0 match event in mask, or -1. */
static int64_t sct_event_match(Lpc82xState *s, uint32_t events)
{
    for (int e = 0; e < SCT_NUM_EV; e++) {
        if ((events & (1u << e)) && (rd32(s->sct, SCT_EV_STATE0 + 8 * e) & 1)) {
            return sct_match(s, rd32(s->sct, SCT_EV_CTRL0 + 8 * e) & 0xF);
        }
    }
    return -1;
}

/*
 * Fraction of each PWM period that output n is high: it is set by one match
 * event and cleared by another, the counter running 0..MATCH0. A halted
 * counter leaves the output at its OUTPUT register level.
 */
float lpc82x_sct_out_duty(Lpc82xState *s, unsigned n)
{
    int64_t period = (int64_t)sct_match(s, 0) + 1;
    int64_t set = sct_event_match(s, rd32(s->sct, SCT_OUT_SET0 + 8 * n));
    int64_t clr = sct_event_match(s, rd32(s->sct, SCT_OUT_CLR0 + 8 * n));
    bool level = (rd32(s->sct, SCT_OUTPUT) >> n) & 1;

    if (rd32(s->sct, SCT_CTRL) & SCT_HALT_L) {
        return level;
    }
    /* the counter runs 0..MATCH0: a match past it never fires */
    set = set >= period ? -1 : set;
    clr = clr >= period ? -1 : clr;
    if (set < 0 || clr < 0) {
        return set >= 0 ? 1.0f : (clr >= 0 ? 0.0f : level);
    }
    if (set == clr) {
        switch ((rd32(s->sct, SCT_RES) >> (2 * n)) & 3) {
        case 1:
            return 1.0f;
        case 2:
            return 0.0f;
        default:
            return level;
        }
    }
    return (float)((clr - set + period) % period) / period;
}

static uint64_t sct_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;

    return bytes_read(s->sct, SCT_SIZE, off, size);
}

static void sct_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;

    bytes_write(s->sct, SCT_SIZE, off, v, size);
    if (s->seqa_ctrl & SEQ_ENA) {
        adc_schedule(s);
    }
}


static void mrt_expire_n(Lpc82xState *s, int n)
{
    MrtChannel *c = &s->mrt[n];

    c->stat |= MRT_INTFLAG;
    if (((c->ctrl >> 1) & 3) == 0 && c->intval) {
        /* repeat mode: count from the expiry, not from the late callback */
        c->start_ns += ticks_to_ns(s, c->load);
        c->load = c->intval;
        timer_mod(c->timer, c->start_ns + ticks_to_ns(s, c->load));
    } else {
        c->stat &= ~MRT_RUN;
    }
    mrt_update_irq(s);
}

static void mrt_expire0(void *opaque) { mrt_expire_n(opaque, 0); }
static void mrt_expire1(void *opaque) { mrt_expire_n(opaque, 1); }
static void mrt_expire2(void *opaque) { mrt_expire_n(opaque, 2); }
static void mrt_expire3(void *opaque) { mrt_expire_n(opaque, 3); }

QEMUTimerCB * const lpc82x_mrt_expire[NUM_MRT] = {
    mrt_expire0, mrt_expire1, mrt_expire2, mrt_expire3,
};


LPC_OPS(mrt);
LPC_OPS(adc);
LPC_OPS(sct);
