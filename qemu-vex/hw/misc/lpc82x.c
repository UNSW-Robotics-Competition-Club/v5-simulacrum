/*
 * NXP LPC82x peripherals for the V5 smart motor boards (machine vex-motor,
 * notes/motor-emulation.md). Only what the motor firmware (DEVFRM02/05/31)
 * touches, following the LPC82x user manual (UM10800):
 *
 *   SYSCON   0x40048000  register file; PLL always locked; clock tree read
 *                        back for the timer rate
 *   MRT      0x40004000  4 down-counters at the system clock, IRQ 10
 *   ADC      0x4001C000  sequence A only; samples come from the plant link,
 *                        IRQ 16 (SEQA)
 *   USART0   0x40064000  the smart-port link: chardev "port", IRQ 3
 *   SWM      0x4000C000  register file (PINASSIGN tells which pin an SCT
 *                        output drives)
 *   IOCON    0x40044000  register file
 *   CRC      0x50000000  engine (CCITT / CRC16 / CRC32)
 *   SCT      0x50004000  register file; PWM duty of each output computed
 *                        from its set/clear events
 *   DMA      0x50008000  18 channels, descriptor table in SRAM, IRQ 20
 *   GPIO     0xA0000000  port 0
 *   PININT   0xA0004000  edge interrupts, IRQ 24..31
 *
 * The plant link (chardev "plant") carries the motor's analogue world. The
 * host sends PLANT_IN messages: 12 ADC samples, encoder position and
 * velocity, GPIO input levels. Each one is answered with a PLANT_OUT
 * message: pin levels, directions, and for each SCT output its pin and duty.
 * The encoder model turns position and velocity into quadrature edges on
 * the PININT0 pin (A) and enc-b-pin (B) at the right virtual times; the
 * firmware times them with an MRT channel.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "lpc82x.h"

bool lpc82x_debug;

/* ---- GPIO / PININT --------------------------------------------------- */

static void pinint_update(Lpc82xState *s)
{
    uint32_t ist = (s->pi_rise & s->pi_ienr) | (s->pi_fall & s->pi_ienf);

    for (int i = 0; i < NUM_PININT; i++) {
        qemu_set_irq(s->irq[IRQ_PININT0 + i], (ist >> i) & 1);
    }
}

void lpc82x_gpio_recompute(Lpc82xState *s)
{
    uint32_t level = (s->gpio_out & s->gpio_dir) | (s->gpio_in & ~s->gpio_dir);
    uint32_t changed = level ^ s->pin_level;

    s->pin_level = level;
    if (!changed) {
        return;
    }
    for (int i = 0; i < NUM_PININT; i++) {
        uint32_t pin = rd32(s->syscon, SC_PINTSEL0 + 4 * i) & 0x3F;
        uint32_t bit = 1u << i;

        if (pin >= 32 || !(changed & (1u << pin)) || (s->pi_isel & bit)) {
            continue;
        }
        if (level & (1u << pin)) {
            s->pi_rise |= bit;
        } else {
            s->pi_fall |= bit;
        }
        if (i < 2 && ((level & (1u << pin)) ? s->pi_ienr : s->pi_ienf) & bit) {
            lpc82x_adc_trigger(s, i == 0 ? ADC_TRIG_PININT0 : ADC_TRIG_PININT1);
        }
    }
    pinint_update(s);
}

void lpc82x_gpio_set_input(Lpc82xState *s, unsigned pin, bool high)
{
    if (pin < 32) {
        s->gpio_in = deposit32(s->gpio_in, pin, 1, high);
        lpc82x_gpio_recompute(s);
    }
}

static uint64_t gpio_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;

    if (off < GP_B0 + NUM_PINS) {
        if (off == s->idle_pin) {
            /*
             * The firmware's main loop reads this pin on every pass and
             * only ever has work after an interrupt; the machine parks the
             * core here (a WFI) instead of spinning a host core.
             */
            qemu_irq_pulse(s->idle_poll);
        }
        return (s->pin_level >> off) & 1;
    }
    if (off >= GP_W0 && off < GP_W0 + 4 * NUM_PINS) {
        return (s->pin_level >> ((off - GP_W0) / 4)) & 1 ? 0xFFFFFFFF : 0;
    }
    switch (off & ~3) {
    case GP_DIR:
        return field32(s->gpio_dir, off, size);
    case GP_MASK:
        return field32(s->gpio_mask, off, size);
    case GP_PIN:
        return field32(s->pin_level, off, size);
    case GP_MPIN:
        return field32(s->pin_level & ~s->gpio_mask, off, size);
    case GP_SET:
        return field32(s->gpio_out, off, size);
    }
    return 0;
}

static void gpio_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t w = merge32(0, off, v, size);

    if (off < GP_B0 + NUM_PINS) {
        s->gpio_out = deposit32(s->gpio_out, off, 1, v & 1);
    } else if (off >= GP_W0 && off < GP_W0 + 4 * NUM_PINS) {
        s->gpio_out = deposit32(s->gpio_out, (off - GP_W0) / 4, 1, v != 0);
    } else {
        switch (off & ~3) {
        case GP_DIR:
            s->gpio_dir = merge32(s->gpio_dir, off, v, size);
            break;
        case GP_MASK:
            s->gpio_mask = merge32(s->gpio_mask, off, v, size);
            break;
        case GP_PIN:
            s->gpio_out = merge32(s->gpio_out, off, v, size);
            break;
        case GP_MPIN:
            s->gpio_out = (s->gpio_out & s->gpio_mask) |
                          (merge32(s->gpio_out, off, v, size) & ~s->gpio_mask);
            break;
        case GP_SET:
            s->gpio_out |= w;
            break;
        case GP_CLR:
            s->gpio_out &= ~w;
            break;
        case GP_NOT:
            s->gpio_out ^= w;
            break;
        case GP_DIRSET:
            s->gpio_dir |= w;
            break;
        case GP_DIRCLR:
            s->gpio_dir &= ~w;
            break;
        case GP_DIRNOT:
            s->gpio_dir ^= w;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "lpc82x gpio: write 0x%" HWADDR_PRIx
                          "\n", off);
        }
    }
    lpc82x_gpio_recompute(s);
}

static uint64_t pinint_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t r = 0;

    switch (off & ~3) {
    case PI_ISEL:
        r = s->pi_isel;
        break;
    case PI_IENR:
        r = s->pi_ienr;
        break;
    case PI_IENF:
        r = s->pi_ienf;
        break;
    case PI_RISE:
        r = s->pi_rise;
        break;
    case PI_FALL:
        r = s->pi_fall;
        break;
    case PI_IST:
        r = (s->pi_rise & s->pi_ienr) | (s->pi_fall & s->pi_ienf);
        break;
    }
    return field32(r, off, size);
}

static void pinint_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;
    uint32_t w = merge32(0, off, v, size) & 0xFF;

    switch (off & ~3) {
    case PI_ISEL:
        s->pi_isel = w;
        break;
    case PI_IENR:
        s->pi_ienr = w;
        break;
    case PI_SIENR:
        s->pi_ienr |= w;
        break;
    case PI_CIENR:
        s->pi_ienr &= ~w;
        break;
    case PI_IENF:
        s->pi_ienf = w;
        break;
    case PI_SIENF:
        s->pi_ienf |= w;
        break;
    case PI_CIENF:
        s->pi_ienf &= ~w;
        break;
    case PI_RISE:
        s->pi_rise &= ~w;
        break;
    case PI_FALL:
        s->pi_fall &= ~w;
        break;
    case PI_IST:
        s->pi_rise &= ~w;
        s->pi_fall &= ~w;
        break;
    }
    pinint_update(s);
}


/* ---- SYSCON, SWM, IOCON register files -------------------------------- */

static uint64_t syscon_read(void *opaque, hwaddr off, unsigned size)
{
    Lpc82xState *s = opaque;

    if ((off & ~3) == SC_SYSPLLSTAT) {
        return 1;                       /* PLL locked */
    }
    return bytes_read(s->syscon, SC_SIZE, off, size);
}

static void syscon_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    Lpc82xState *s = opaque;

    bytes_write(s->syscon, SC_SIZE, off, v, size);
    if ((off & ~3) >= SC_PINTSEL0 && (off & ~3) < SC_PINTSEL0 + 32) {
        lpc82x_gpio_recompute(s);
    }
}

static uint64_t swm_read(void *opaque, hwaddr off, unsigned size)
{
    return bytes_read(LPC82X(opaque)->swm, SWM_SIZE, off, size);
}

static void swm_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    bytes_write(LPC82X(opaque)->swm, SWM_SIZE, off, v, size);
}

static uint64_t iocon_read(void *opaque, hwaddr off, unsigned size)
{
    return bytes_read(LPC82X(opaque)->iocon, IOCON_SIZE, off, size);
}

static void iocon_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    bytes_write(LPC82X(opaque)->iocon, IOCON_SIZE, off, v, size);
}

/* ---- device ---------------------------------------------------------- */

LPC_OPS(syscon);
LPC_OPS(swm);
LPC_OPS(iocon);
LPC_OPS(gpio);
LPC_OPS(pinint);

static void lpc82x_reset(DeviceState *dev)
{
    Lpc82xState *s = LPC82X(dev);

    memset(s->syscon, 0, sizeof(s->syscon));
    stl_le_p(s->syscon + SC_SYSAHBCLKDIV, 1);
    stl_le_p(s->syscon + SC_DEVICE_ID, 0x00008242);   /* LPC824M201 */
    memset(s->swm, 0xFF, sizeof(s->swm));
    memset(s->iocon, 0, sizeof(s->iocon));
    memset(s->sct, 0, sizeof(s->sct));
    stl_le_p(s->sct + SCT_CTRL, 0x00040004);           /* halted */

    for (int i = 0; i < NUM_MRT; i++) {
        s->mrt[i].intval = s->mrt[i].ctrl = s->mrt[i].stat = 0;
        timer_del(s->mrt[i].timer);
    }
    s->adc_ctrl = s->seqa_ctrl = s->seqb_ctrl = s->adc_inten = 0;
    s->adc_flags = s->seqa_gdat = 0;
    memset(s->adc_dat, 0, sizeof(s->adc_dat));
    timer_del(s->adc_timer);
    /*
     * Until the plant link speaks, read like an idle motor on a charged
     * battery (sim/v5sim/firmware_motor.py adc_samples): the firmware
     * samples from boot, before the host connects.
     */
    for (int i = 0; i < NUM_ADC; i++) {
        s->adc_in[i] = 0x7FF;
    }
    s->adc_in[2] = s->adc_in[3] = s->adc_in[4] = 0;   /* current, terminals */
    s->adc_in[5] = 2078;                               /* battery 12.8 V */
    s->adc_in[7] = 2045;                               /* 25 degC */

    s->u_cfg = s->u_ctl = s->u_stat = s->u_inten = 0;
    s->u_brg = s->u_addr = 0;
    s->u_osr = 0xF;                     /* 16x oversampling */
    s->rx_full = false;
    s->tx_hold_full = s->tx_busy = false;
    timer_del(s->tx_timer);

    s->dma_ctrl = s->srambase = s->dma_enable = s->dma_inten = 0;
    s->dma_inta = s->dma_intb = s->dma_errint = s->dma_trig = 0;
    memset(s->dma, 0, sizeof(s->dma));

    s->crc_mode = 0;
    s->crc_sum = 0xFFFF;

    s->gpio_out = s->gpio_dir = s->gpio_mask = 0;
    s->gpio_in = 0xFFFFFFFF;           /* pull-ups */
    s->pin_level = s->gpio_in;
    s->pi_isel = s->pi_ienr = s->pi_ienf = s->pi_rise = s->pi_fall = 0;

    s->enc_n = 0;
    s->enc_pos0 = s->enc_vel = 0;
    s->enc_t0 = 0;
    timer_del(s->enc_timer);
    s->plant_len = 0;
    if (s->plant_reset) {
        static const uint8_t ack = PLANT_RESET;

        s->plant_reset = false;
        qemu_chr_fe_write_all(&s->plant, &ack, 1);
    }
}

static void lpc82x_realize(DeviceState *dev, Error **errp)
{
    Lpc82xState *s = LPC82X(dev);

    lpc82x_debug = getenv("LPC82X_DEBUG") != NULL;
    if (s->enc_b_pin >= NUM_PINS) {
        error_setg(errp, "lpc82x: enc-b-pin %u is not a port 0 pin",
                   s->enc_b_pin);
        return;
    }

    for (int i = 0; i < NUM_MRT; i++) {
        s->mrt[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lpc82x_mrt_expire[i], s);
    }
    s->adc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lpc82x_adc_tick, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lpc82x_usart_tx_done, s);
    s->enc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lpc82x_enc_tick, s);
    qemu_chr_fe_set_handlers(&s->port, lpc82x_usart_can_receive, lpc82x_usart_receive,
                             NULL, NULL, s, NULL, true);
    qemu_chr_fe_set_handlers(&s->plant, lpc82x_plant_can_receive, lpc82x_plant_receive,
                             NULL, NULL, s, NULL, true);
}

static void lpc82x_init(Object *obj)
{
    Lpc82xState *s = LPC82X(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    struct {
        MemoryRegion *mr;
        const MemoryRegionOps *ops;
        const char *name;
        uint64_t size;
    } regions[] = {
        { &s->syscon_mr, &lpc82x_syscon_ops, "lpc82x.syscon", 0x4000 },
        { &s->mrt_mr,    &lpc82x_mrt_ops,    "lpc82x.mrt",    0x4000 },
        { &s->adc_mr,    &lpc82x_adc_ops,    "lpc82x.adc",    0x4000 },
        { &s->usart_mr,  &lpc82x_usart_ops,  "lpc82x.usart0", 0x4000 },
        { &s->swm_mr,    &lpc82x_swm_ops,    "lpc82x.swm",    0x4000 },
        { &s->iocon_mr,  &lpc82x_iocon_ops,  "lpc82x.iocon",  0x4000 },
        { &s->crc_mr,    &lpc82x_crc_ops,    "lpc82x.crc",    0x4000 },
        { &s->sct_mr,    &lpc82x_sct_ops,    "lpc82x.sct",    0x4000 },
        { &s->dma_mr,    &lpc82x_dma_ops,    "lpc82x.dma",    0x4000 },
        { &s->gpio_mr,   &lpc82x_gpio_ops,   "lpc82x.gpio",   0x4000 },
        { &s->pinint_mr, &lpc82x_pinint_ops, "lpc82x.pinint", 0x4000 },
    };

    for (size_t i = 0; i < ARRAY_SIZE(regions); i++) {
        memory_region_init_io(regions[i].mr, obj, regions[i].ops, s,
                              regions[i].name, regions[i].size);
        sysbus_init_mmio(sbd, regions[i].mr);
    }
    for (int i = 0; i < 32; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), &s->idle_poll, "idle-poll", 1);
}

static const Property lpc82x_props[] = {
    DEFINE_PROP_CHR("port", Lpc82xState, port),
    DEFINE_PROP_CHR("plant", Lpc82xState, plant),
    DEFINE_PROP_UINT32("enc-b-pin", Lpc82xState, enc_b_pin, 7),
    DEFINE_PROP_UINT32("idle-pin", Lpc82xState, idle_pin, 0xFF),
};

static void lpc82x_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lpc82x_realize;
    device_class_set_legacy_reset(dc, lpc82x_reset);
    device_class_set_props(dc, lpc82x_props);
    dc->desc = "NXP LPC82x peripherals (V5 smart motor)";
}

static const TypeInfo lpc82x_info = {
    .name = TYPE_LPC82X,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Lpc82xState),
    .instance_init = lpc82x_init,
    .class_init = lpc82x_class_init,
};

static void lpc82x_register_types(void)
{
    type_register_static(&lpc82x_info);
}

type_init(lpc82x_register_types)
