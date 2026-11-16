/*
 * NXP LPC82x peripherals for the V5 smart motor boards: state shared by
 * lpc82x.c (overview), lpc82x_timers.c, lpc82x_link.c and lpc82x_plant.c.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_LPC82X_H
#define HW_MISC_LPC82X_H

#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_LPC82X "lpc82x-periph"
OBJECT_DECLARE_SIMPLE_TYPE(Lpc82xState, LPC82X)

#define IRC_HZ          12000000ULL
#define NUM_MRT         4
#define NUM_DMA         18
#define NUM_ADC         12
#define NUM_PININT      8
#define NUM_SCT_OUT     6

#define IRQ_USART0      3
#define IRQ_MRT         10
#define IRQ_ADC_SEQA    16
#define IRQ_DMA         20
#define IRQ_PININT0     24

/* SYSCON */
#define SC_SYSPLLCTRL   0x08
#define SC_SYSPLLSTAT   0x0C
#define SC_SYSPLLCLKSEL 0x40
#define SC_MAINCLKSEL   0x70
#define SC_SYSAHBCLKDIV 0x78
#define SC_PINTSEL0     0x178
#define SC_UARTCLKDIV   0x094
#define SC_UARTFRGDIV   0x0F0
#define SC_UARTFRGMULT  0x0F4
#define SC_DEVICE_ID    0x3F8
#define SC_SIZE         0x400

/* MRT channel registers (stride 0x10) and globals */
#define MRT_INTVAL      0x0
#define MRT_TIMER       0x4
#define MRT_CTRL        0x8
#define MRT_STAT        0xC
#define MRT_IDLE_CH     0xF4
#define MRT_IRQ_FLAG    0xF8
#define MRT_LOAD        (1u << 31)
#define MRT_INTEN       (1u << 0)
#define MRT_INTFLAG     (1u << 0)
#define MRT_RUN         (1u << 1)

/* ADC */
#define ADC_CTRL        0x00
#define ADC_SEQA_CTRL   0x08
#define ADC_SEQB_CTRL   0x0C
#define ADC_SEQA_GDAT   0x10
#define ADC_SEQB_GDAT   0x14
#define ADC_DAT0        0x20
#define ADC_INTEN       0x64
#define ADC_FLAGS       0x68
#define ADC_CALMODE     (1u << 30)
#define SEQ_START       (1u << 26)
#define SEQ_BURST       (1u << 27)
#define SEQ_ENA         (1u << 31)
#define DAT_VALID       (1u << 31)
#define DAT_OVERRUN     (1u << 30)
#define FLAG_SEQA_INT   (1u << 28)
#define ADC_TRIG_PININT0 1
#define ADC_TRIG_PININT1 2
#define ADC_TRIG_SCT_OUT3 3
/*
 * Hardware-triggered sampling runs at most every 200 us (5 kHz) instead of
 * every PWM period (~24 kHz): the plant changes its inputs once per ms, the
 * firmware averages its last few samples, and each conversion costs QEMU a
 * timer and a vCPU kick (a quarter of a host core at the full rate).
 */
#define ADC_MIN_PERIOD_NS 200000

/* USART */
#define USART0_BASE     0x40064000
#define U_CFG           0x00
#define U_CTL           0x04
#define U_STAT          0x08
#define U_INTENSET      0x0C
#define U_INTENCLR      0x10
#define U_RXDAT         0x14
#define U_RXDATSTAT     0x18
#define U_TXDAT         0x1C
#define U_BRG           0x20
#define U_INTSTAT       0x24
#define U_OSR           0x28
#define U_ADDR          0x2C
#define U_ENABLE        (1u << 0)
#define ST_RXRDY        (1u << 0)
#define ST_RXIDLE       (1u << 1)
#define ST_TXRDY        (1u << 2)
#define ST_TXIDLE       (1u << 3)
#define ST_START        (1u << 12)
#define ST_W1C          (0xF920u)   /* sticky flags cleared by writing 1 */

/* DMA */
#define DMA_CTRL        0x00
#define DMA_INTSTAT     0x04
#define DMA_SRAMBASE    0x08
#define DMA_ENABLESET   0x20
#define DMA_ENABLECLR   0x28
#define DMA_ACTIVE      0x30
#define DMA_BUSY        0x38
#define DMA_ERRINT      0x40
#define DMA_INTENSET    0x48
#define DMA_INTENCLR    0x50
#define DMA_INTA        0x58
#define DMA_INTB        0x60
#define DMA_SETVALID    0x68
#define DMA_SETTRIG     0x70
#define DMA_ABORT       0x78
#define DMA_CH_BASE     0x400
#define DMA_CH_CFG      0x0
#define DMA_CH_CTLSTAT  0x4
#define DMA_CH_XFERCFG  0x8
#define CFG_PERIPHREQEN (1u << 0)
#define XF_CFGVALID     (1u << 0)
#define XF_RELOAD       (1u << 1)
#define XF_SWTRIG       (1u << 2)
#define XF_CLRTRIG      (1u << 3)
#define XF_SETINTA      (1u << 4)
#define XF_SETINTB      (1u << 5)
#define XF_COUNT_SHIFT  16
#define XF_COUNT_MASK   (0x3FFu << XF_COUNT_SHIFT)
#define DMA_REQ_U0_RX   0
#define DMA_REQ_U0_TX   1

/* SCT */
#define SCT_CONFIG      0x000
#define SCT_CTRL        0x004
#define SCT_OUTPUT      0x050
#define SCT_RES         0x058
#define SCT_MATCHREL0   0x200
#define SCT_EV_STATE0   0x300
#define SCT_EV_CTRL0    0x304
#define SCT_OUT_SET0    0x500
#define SCT_OUT_CLR0    0x504
#define SCT_SIZE        0x600
#define SCT_NUM_EV      8
#define SCT_UNIFY       (1u << 0)
#define SCT_HALT_L      (1u << 2)
#define SCT_BIDIR_L     (1u << 4)

/* GPIO port 0 */
#define GP_B0           0x0000
#define GP_W0           0x1000
#define GP_DIR          0x2000
#define GP_MASK         0x2080
#define GP_PIN          0x2100
#define GP_MPIN         0x2180
#define GP_SET          0x2200
#define GP_CLR          0x2280
#define GP_NOT          0x2300
#define GP_DIRSET       0x2380
#define GP_DIRCLR       0x2400
#define GP_DIRNOT       0x2480
#define NUM_PINS        29

/* PININT */
#define PI_ISEL         0x00
#define PI_IENR         0x04
#define PI_SIENR        0x08
#define PI_CIENR        0x0C
#define PI_IENF         0x10
#define PI_SIENF        0x14
#define PI_CIENF        0x18
#define PI_RISE         0x1C
#define PI_FALL         0x20
#define PI_IST          0x24

/* CRC */
#define CRC_MODE        0x0
#define CRC_SEED        0x4
#define CRC_SUM         0x8

#define SWM_SIZE        0x200
#define IOCON_SIZE      0x100

/* Plant link */
#define PLANT_IN        'I'
#define PLANT_OUT       'O'
#define PLANT_RESET     'R'         /* power-cycle the board; echoed once done */
#define PLANT_IN_LEN    (1 + 2 * NUM_ADC + 8 + 8 + 4)
#define PLANT_OUT_LEN   (1 + 4 + 4 + NUM_SCT_OUT + 4 * NUM_SCT_OUT + 8)
#define ENC_BURST_NS    5000        /* gap between catch-up edges */
#define ENC_MIN_NS      1000
#define ENC_MAX_JUMP    2000        /* bigger jumps re-sync without edges */
#define ENC_RESYNC      50.0        /* counts off: jump instead of steering */
#define ENC_LOCK_S      0.020       /* steer position errors out over this */

typedef struct MrtChannel {
    uint32_t intval;
    uint32_t ctrl;
    uint32_t stat;
    uint32_t load;
    int64_t start_ns;
    QEMUTimer *timer;
} MrtChannel;

typedef struct DmaChannel {
    uint32_t cfg;
    uint32_t xfercfg;
    uint32_t srcend;
    uint32_t dstend;
    uint32_t link;
    uint32_t remaining;
    bool active;
} DmaChannel;

struct Lpc82xState {
    SysBusDevice parent_obj;

    MemoryRegion syscon_mr, mrt_mr, adc_mr, usart_mr, swm_mr, iocon_mr;
    MemoryRegion crc_mr, sct_mr, dma_mr, gpio_mr, pinint_mr;
    qemu_irq irq[32];
    CharFrontend port;
    CharFrontend plant;
    uint32_t enc_b_pin;
    uint32_t idle_pin;          /* main-loop poll: see lpc82x_idle */
    qemu_irq idle_poll;

    uint8_t syscon[SC_SIZE];
    uint8_t swm[SWM_SIZE];
    uint8_t iocon[IOCON_SIZE];
    uint8_t sct[SCT_SIZE];

    MrtChannel mrt[NUM_MRT];

    uint32_t adc_ctrl, seqa_ctrl, seqb_ctrl, adc_inten, adc_flags;
    uint32_t adc_dat[NUM_ADC], seqa_gdat;
    uint16_t adc_in[NUM_ADC];
    QEMUTimer *adc_timer;

    uint32_t u_cfg, u_ctl, u_stat, u_inten, u_brg, u_osr, u_addr;
    uint8_t rx_byte;
    bool rx_full;
    uint8_t tx_hold, tx_shift;
    bool tx_hold_full, tx_busy;
    QEMUTimer *tx_timer;

    uint32_t dma_ctrl, srambase, dma_enable, dma_inten;
    uint32_t dma_inta, dma_intb, dma_errint, dma_trig;
    DmaChannel dma[NUM_DMA];
    bool dma_busy;

    uint32_t crc_mode, crc_sum;

    uint32_t gpio_out, gpio_dir, gpio_in, gpio_mask, pin_level;

    uint32_t pi_isel, pi_ienr, pi_ienf, pi_rise, pi_fall;

    int64_t enc_n;
    double enc_pos0, enc_vel;
    int64_t enc_t0;
    QEMUTimer *enc_timer;

    uint8_t plant_buf[PLANT_IN_LEN];
    int plant_len;
    bool plant_reset;           /* the host asked for this reset */
};

extern bool lpc82x_debug;
extern const MemoryRegionOps lpc82x_syscon_ops, lpc82x_mrt_ops, lpc82x_adc_ops,
    lpc82x_usart_ops, lpc82x_swm_ops, lpc82x_iocon_ops, lpc82x_crc_ops,
    lpc82x_sct_ops, lpc82x_dma_ops, lpc82x_gpio_ops, lpc82x_pinint_ops;
extern QEMUTimerCB * const lpc82x_mrt_expire[NUM_MRT];

/* lpc82x.c */
void lpc82x_gpio_recompute(Lpc82xState *s);
void lpc82x_gpio_set_input(Lpc82xState *s, unsigned pin, bool high);
/* lpc82x_timers.c */
void lpc82x_adc_trigger(Lpc82xState *s, int source);
void lpc82x_adc_tick(void *opaque);
float lpc82x_sct_out_duty(Lpc82xState *s, unsigned n);
/* lpc82x_link.c */
void lpc82x_usart_tx_done(void *opaque);
int lpc82x_usart_can_receive(void *opaque);
void lpc82x_usart_receive(void *opaque, const uint8_t *buf, int size);
/* lpc82x_plant.c */
void lpc82x_enc_tick(void *opaque);
int lpc82x_plant_can_receive(void *opaque);
void lpc82x_plant_receive(void *opaque, const uint8_t *buf, int size);

/* ---- helpers ------------------------------------------------------- */

#define DBG(fmt, ...) do {                                              \
        if (lpc82x_debug) {                                      \
            qemu_log("%10" PRId64 "us " fmt "\n",                       \
                     qemu_clock_get_us(QEMU_CLOCK_VIRTUAL), ##__VA_ARGS__); \
        }                                                               \
    } while (0)

static inline uint64_t bytes_read(const uint8_t *a, size_t n, hwaddr off,
                           unsigned size)
{
    return off + size <= n ? ldn_le_p(a + off, size) : 0;
}

static inline void bytes_write(uint8_t *a, size_t n, hwaddr off, uint64_t v,
                        unsigned size)
{
    if (off + size <= n) {
        stn_le_p(a + off, size, v);
    }
}

static inline uint32_t rd32(const uint8_t *a, hwaddr off)
{
    return ldl_le_p(a + off);
}

/* Replace bytes [off, off+size) of a 32-bit register value. */
static inline uint32_t merge32(uint32_t old, hwaddr off, uint64_t v, unsigned size)
{
    unsigned shift = (off & 3) * 8;
    uint32_t mask = (size >= 4 ? 0xFFFFFFFFu : ((1u << (size * 8)) - 1))
                    << shift;

    return (old & ~mask) | (((uint32_t)v << shift) & mask);
}

static inline uint64_t field32(uint32_t reg, hwaddr off, unsigned size)
{
    uint32_t v = reg >> ((off & 3) * 8);

    return size >= 4 ? v : v & ((1u << (size * 8)) - 1);
}

static inline uint64_t lpc_main_hz(Lpc82xState *s)
{
    uint32_t pllctrl = rd32(s->syscon, SC_SYSPLLCTRL);

    switch (rd32(s->syscon, SC_MAINCLKSEL) & 3) {
    case 3:
        return IRC_HZ * ((pllctrl & 0x1F) + 1);
    case 2:
        return 600000;                  /* watchdog oscillator, nominal */
    default:
        return IRC_HZ;
    }
}

static inline uint64_t lpc_sys_hz(Lpc82xState *s)
{
    uint32_t div = rd32(s->syscon, SC_SYSAHBCLKDIV) & 0xFF;

    return lpc_main_hz(s) / (div ? div : 1);
}

static inline int64_t ticks_to_ns(Lpc82xState *s, uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, lpc_sys_hz(s));
}

#define LPC_OPS(name)                                                   \
    const MemoryRegionOps lpc82x_##name##_ops = {                       \
        .read = name##_read,                                            \
        .write = name##_write,                                          \
        .endianness = DEVICE_LITTLE_ENDIAN,                             \
        .valid = { .min_access_size = 1, .max_access_size = 4 },        \
        .impl = { .min_access_size = 1, .max_access_size = 4 },         \
    }

#endif
