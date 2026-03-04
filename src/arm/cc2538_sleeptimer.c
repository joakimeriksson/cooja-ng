/*
 * CC2538 Sleep Timer (part of SMWDTHROSC module at 0x400D5000)
 *
 * 32-bit up-counter running at 32,768 Hz with compare interrupt.
 * Counter value is computed on-demand from sim_time_ns.
 */
#include "cc2538_sleeptimer.h"
#include <string.h>

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

/* Schedule the compare match event */
static void sleeptimer_schedule_compare(cc2538_sleeptimer_t *st) {
    uint32_t current = sleeptimer_get_count(st);
    uint32_t delta = (st->compare - current) & 0xFFFFFFFF;

    /* If delta is 0, fire on next wrap (full 2^32 ticks) — but practically
       just schedule a tiny delta to fire soon */
    if (delta == 0) delta = 1;

    /* Convert ticks to ns: delta_ns = delta * 1_000_000_000 / 32768 */
    int64_t delta_ns = (int64_t)delta * 1000000000LL / SLEEPTIMER_FREQ_HZ;

    arm_schedule_event_ns(st->cpu, &st->compare_event,
                          sleeptimer_now_ns(st) + delta_ns);
}

/* Compare match callback — fires interrupt */
static void sleeptimer_compare_fire(void *user_data, arm_event_t *ev) {
    cc2538_sleeptimer_t *st = (cc2538_sleeptimer_t *)user_data;
    (void)ev;

    /* Update base to current time (use cycles-derived ns, not sim_time_ns) */
    st->base_count = sleeptimer_get_count(st);
    st->base_ns = sleeptimer_now_ns(st);

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
