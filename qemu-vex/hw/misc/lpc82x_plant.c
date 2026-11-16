/*
 * The motor board around the LPC82x (machine vex-motor, see lpc82x.c): the
 * quadrature encoder on PININT0 and the plant link to the robot sim.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "system/runstate.h"
#include "lpc82x.h"

/* SWM: PINASSIGN byte of each SCT output (UM10800 table 69) */
static const uint8_t swm_sct_out_byte[NUM_SCT_OUT] = {
    0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
};

/* ---- encoder --------------------------------------------------------- */

static unsigned enc_a_pin(Lpc82xState *s)
{
    unsigned pin = rd32(s->syscon, SC_PINTSEL0) & 0x3F;

    return pin < 32 ? pin : 31;         /* keep shifts defined */
}

/*
 * Count n is the number of A edges from zero: A = n & 1. B leads by half a
 * count, so at an A edge to or from count n, B = n & 1 for the larger n.
 * The firmware's PININT0 handler counts rise with B high or fall with B low
 * as +1.
 */
static void enc_set_levels(Lpc82xState *s, int64_t n, int64_t b_count)
{
    s->gpio_in = deposit32(s->gpio_in, s->enc_b_pin, 1, b_count & 1);
    lpc82x_gpio_recompute(s);
    lpc82x_gpio_set_input(s, enc_a_pin(s), n & 1);
}

static double enc_target(Lpc82xState *s, int64_t now)
{
    return s->enc_pos0 + s->enc_vel * (now - s->enc_t0) / 1e9;
}

void lpc82x_enc_tick(void *opaque)
{
    Lpc82xState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t goal = (int64_t)floor(enc_target(s, now));
    int64_t next;

    if (llabs(goal - s->enc_n) > ENC_MAX_JUMP) {
        s->enc_n = goal;
        enc_set_levels(s, goal, goal);
    } else if (goal > s->enc_n) {
        s->enc_n++;
        enc_set_levels(s, s->enc_n, s->enc_n);
    } else if (goal < s->enc_n) {
        enc_set_levels(s, s->enc_n - 1, s->enc_n);
        s->enc_n--;
    }

    if (goal != s->enc_n) {
        next = now + ENC_BURST_NS;
    } else if (s->enc_vel != 0) {
        double edge = s->enc_vel > 0 ? s->enc_n + 1 : s->enc_n;
        double dt = (edge - s->enc_pos0) / s->enc_vel * 1e9;

        next = MAX(s->enc_t0 + (int64_t)ceil(dt), now + ENC_MIN_NS);
    } else {
        timer_del(s->enc_timer);
        return;
    }
    timer_mod(s->enc_timer, next);
}

/*
 * Follow the host's shaft. The host steps on its own clock, so its position
 * at "now" is off by its step jitter; jumping to it each update made bursts
 * and gaps of edges that the firmware read as speed spikes. Instead keep
 * integrating and steer the speed to remove the error over ENC_LOCK_S.
 */
static void enc_follow(Lpc82xState *s, double pos, double vel)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double here = enc_target(s, now);
    double err = pos - here;

    if (fabs(err) > ENC_RESYNC) {
        here = pos;                     /* reset or teleport: jump */
        err = 0;
    }
    s->enc_pos0 = here;
    s->enc_vel = vel + err / ENC_LOCK_S;
    s->enc_t0 = now;
    lpc82x_enc_tick(s);
}


/* ---- plant link ------------------------------------------------------ */

static void plant_reply(Lpc82xState *s)
{
    uint8_t out[PLANT_OUT_LEN];
    uint8_t *p = out;

    *p++ = PLANT_OUT;
    stl_le_p(p, s->pin_level);
    p += 4;
    stl_le_p(p, s->gpio_dir);
    p += 4;
    for (int n = 0; n < NUM_SCT_OUT; n++) {
        *p++ = s->swm[swm_sct_out_byte[n]];
    }
    for (int n = 0; n < NUM_SCT_OUT; n++) {
        float d = lpc82x_sct_out_duty(s, n);
        uint32_t bits;

        memcpy(&bits, &d, 4);
        stl_le_p(p, bits);
        p += 4;
    }
    stq_le_p(p, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    qemu_chr_fe_write_all(&s->plant, out, sizeof(out));
}

static void plant_apply(Lpc82xState *s, const uint8_t *m)
{
    const uint8_t *p = m + 1;
    uint64_t bits;
    double pos, vel;

    for (int i = 0; i < NUM_ADC; i++, p += 2) {
        s->adc_in[i] = lduw_le_p(p);
    }
    bits = ldq_le_p(p);
    memcpy(&pos, &bits, 8);
    p += 8;
    bits = ldq_le_p(p);
    memcpy(&vel, &bits, 8);
    p += 8;
    if (!isfinite(pos) || !isfinite(vel)) {
        qemu_log_mask(LOG_GUEST_ERROR, "lpc82x plant: bad encoder input\n");
        pos = s->enc_pos0;
        vel = 0;
    }
    s->gpio_in = (ldl_le_p(p) & ~(1u << enc_a_pin(s)) & ~(1u << s->enc_b_pin))
                 | (s->gpio_in & ((1u << enc_a_pin(s)) | (1u << s->enc_b_pin)));
    lpc82x_gpio_recompute(s);

    enc_follow(s, pos, vel);
}

int lpc82x_plant_can_receive(void *opaque)
{
    return PLANT_IN_LEN;
}

void lpc82x_plant_receive(void *opaque, const uint8_t *buf, int size)
{
    Lpc82xState *s = opaque;

    if (s->plant_reset) {
        return;                         /* reset pending: lpc82x_reset clears */
    }
    for (int i = 0; i < size; i++) {
        if (s->plant_len == 0 && buf[i] == PLANT_RESET) {
            /*
             * Plugged back in: power-on reset, acked by lpc82x_reset. The
             * reset only runs later, from the main loop, so drop the rest:
             * a message started now would be cut off midway by it.
             */
            s->plant_reset = true;
            qemu_system_reset_request(SHUTDOWN_CAUSE_HOST_QMP_SYSTEM_RESET);
            return;
        }
        if (s->plant_len == 0 && buf[i] != PLANT_IN) {
            continue;                   /* resync on the tag byte */
        }
        s->plant_buf[s->plant_len++] = buf[i];
        if (s->plant_len == PLANT_IN_LEN) {
            plant_apply(s, s->plant_buf);
            plant_reply(s);
            s->plant_len = 0;
        }
    }
}
