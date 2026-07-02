/*
 * CC2538 Sleep Timer (part of SMWDTHROSC module at 0x400D5000)
 *
 * 32-bit up-counter running at 32,768 Hz with compare interrupt.
 * Counter value is computed on-demand from sim_time_ns.
 */
#include "cc2538_sleeptimer.h"
#include <string.h>
#include <stdio.h>

#define SMWDTHROSC_BASE  0x400D5000
#define SMWDTHROSC_SIZE  0x1000

/* Current ns derived from CPU cycles — always accurate, even mid-step.
 * (sim_time_ns is only updated at end of arm_step/arm_step_until batches,
 *  so using it here returns frozen values during CSMA busy-waits.) */
static inline int64_t sleeptimer_now_ns(cc2538_sleeptimer_t *st) {
    return arm_cycles_to_ns(st->cpu->cycles, st->cpu->cpu_freq_hz);
}

/* Compute current counter value from CPU cycles */
static uint32_t sleeptimer_get_count(cc2538_sleeptimer_t *st) {
    int64_t elapsed_ns = sleeptimer_now_ns(st) - st->base_ns;
    if (elapsed_ns < 0) elapsed_ns = 0;
    /* ticks = elapsed_ns * 32768 / 1_000_000_000 */
    uint64_t ticks = (uint64_t)elapsed_ns * SLEEPTIMER_FREQ_HZ / 1000000000ULL;
    return st->base_count + (uint32_t)ticks;
}

/* Schedule the compare match event.
 * Uses direct cycle-based scheduling (arm_schedule_event) instead of ns-based
 * (arm_schedule_event_ns) because the latter uses sim_time_ns which is only
 * updated at the end of arm_step_until() batches.  When the compare fires
 * mid-step and firmware immediately schedules the next compare (e.g. TSCH
 * slot timing), the stale sim_time_ns caused fire_cycle to be up to 1ms late,
 * breaking TSCH deadline checks. */
static void sleeptimer_schedule_compare(cc2538_sleeptimer_t *st) {
    uint32_t current = sleeptimer_get_count(st);
    uint32_t delta = (st->compare - current) & 0xFFFFFFFF;

    /* If delta is 0, fire on next wrap (full 2^32 ticks) — but practically
       just schedule a tiny delta to fire soon */
    if (delta == 0) delta = 1;

    /* Convert delta ticks directly to CPU cycles:
     * delta_cycles = delta * cpu_freq / 32768
     * Use 128-bit arithmetic to avoid overflow.  Round UP: the compare
     * must fire when the counter has reached the compare value.  With
     * floor the event fired up to one tick early, so an ISR reading the
     * counter saw compare-1 (30.5 us early) — poison for TSCH slot
     * timing that immediately schedules the next deadline from "now". */
    /* delta <= 2^32, cpu_freq <= ~2^27 -> product < 2^59, fits int64. */
    int64_t delta_cycles = ((int64_t)delta * st->cpu->cpu_freq_hz
                            + SLEEPTIMER_FREQ_HZ - 1)
                           / SLEEPTIMER_FREQ_HZ;

    arm_schedule_event(st->cpu, &st->compare_event,
                       st->cpu->cycles + delta_cycles);
}

/* Compare match callback — fires interrupt */
static void sleeptimer_compare_fire(void *user_data, arm_event_t *ev) {
    cc2538_sleeptimer_t *st = (cc2538_sleeptimer_t *)user_data;
    (void)ev;

    /* Re-anchor base on the exact TICK BOUNDARY, not on "now".
     *
     * The old code set base_count = floor(current count) and
     * base_ns = now, silently discarding the fractional tick
     * (0..30.5 us) at every compare fire.  TSCH fires several rtimer
     * compares per active slot, phase-locked to the slot grid, so the
     * discarded fraction was systematic — an activity-proportional
     * clock loss (~100 us per slotframe) that differed between
     * coordinator and node and walked their slot grids apart by
     * ~300 us/s.  Sync collapsed within seconds of association and
     * every keepalive missed the peer's RX window (CSMA never noticed:
     * it has no long-horizon absolute timing).
     *
     * Advance base_count by the whole elapsed ticks and base_ns by
     * exactly those ticks' duration, preserving the sub-tick phase.
     * Residual integer-division error is <1 ns per fire (~0.1 ppm),
     * well under the 9 ppm measured on real zoul hardware. */
    int64_t now = sleeptimer_now_ns(st);
    int64_t elapsed = now - st->base_ns;
    if (elapsed > 0) {
        uint64_t ticks = (uint64_t)elapsed * SLEEPTIMER_FREQ_HZ / 1000000000ULL;
        st->base_count += (uint32_t)ticks;
        /* ticks < 2^33 between fires -> ticks * 1e9 < 2^63, fits int64. */
        st->base_ns += (int64_t)(ticks * 1000000000ULL / SLEEPTIMER_FREQ_HZ);
    }

    /* Set interrupt pending — wakes CPU from WFI */
    arm_nvic_set_pending(st->nvic, st->irq);
}

/* IO read callback */
static int sleeptimer_read(void *user_data, uint32_t addr) {
    cc2538_sleeptimer_t *st = (cc2538_sleeptimer_t *)user_data;
    uint32_t offset = addr - SMWDTHROSC_BASE;

    switch (offset) {
        case SMWDTHROSC_WDCTL:
            return 0;  /* Watchdog stub */

        case SMWDTHROSC_ST0:
            /* Reading ST0 latches the full 32-bit counter */
            st->latched_count = sleeptimer_get_count(st);
            return (int)(st->latched_count & 0xFF);

        case SMWDTHROSC_ST1:
            return (int)((st->latched_count >> 8) & 0xFF);

        case SMWDTHROSC_ST2:
            return (int)((st->latched_count >> 16) & 0xFF);

        case SMWDTHROSC_ST3:
            return (int)((st->latched_count >> 24) & 0xFF);

        case SMWDTHROSC_STLOAD:
            return 1;  /* Compare value always loaded */

        case SMWDTHROSC_STCV0:
            return (int)(st->latched_count & 0xFF);
        case SMWDTHROSC_STCV1:
            return (int)((st->latched_count >> 8) & 0xFF);
        case SMWDTHROSC_STCV2:
            return (int)((st->latched_count >> 16) & 0xFF);
        case SMWDTHROSC_STCV3:
            return (int)((st->latched_count >> 24) & 0xFF);

        default:
            return 0;
    }
}

/* IO write callback */
static void sleeptimer_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_sleeptimer_t *st = (cc2538_sleeptimer_t *)user_data;
    uint32_t offset = addr - SMWDTHROSC_BASE;

    switch (offset) {
        case SMWDTHROSC_WDCTL:
            break;  /* Watchdog stub — ignore */

        case SMWDTHROSC_ST0:
            /* Writing ST0 latches compare from all staging bytes and
             * schedules the compare event.  The CC2538 datasheet says:
             * "When writing, the value is the compare value. The value
             *  written to ST0 along with the values in ST1, ST2, ST3
             *  are loaded."
             * Firmware writes ST3→ST2→ST1→ST0 (ST0 last to trigger latch). */
            st->compare_staging[0] = (uint8_t)value;
            st->compare = (uint32_t)st->compare_staging[0]
                        | ((uint32_t)st->compare_staging[1] << 8)
                        | ((uint32_t)st->compare_staging[2] << 16)
                        | ((uint32_t)st->compare_staging[3] << 24);
            sleeptimer_schedule_compare(st);
            break;

        case SMWDTHROSC_ST1:
            st->compare_staging[1] = (uint8_t)value;
            break;

        case SMWDTHROSC_ST2:
            st->compare_staging[2] = (uint8_t)value;
            break;

        case SMWDTHROSC_ST3:
            st->compare_staging[3] = (uint8_t)value;
            break;

        default:
            break;
    }
}

void cc2538_sleeptimer_init(cc2538_sleeptimer_t *st, arm_cpu_t *cpu,
                            arm_nvic_t *nvic, int irq) {
    memset(st, 0, sizeof(*st));
    st->cpu = cpu;
    st->nvic = nvic;
    st->irq = irq;

    st->compare_event.callback = sleeptimer_compare_fire;
    st->compare_event.user_data = st;

    arm_register_io(cpu, SMWDTHROSC_BASE, SMWDTHROSC_SIZE,
                    sleeptimer_read, sleeptimer_write, st);
}
