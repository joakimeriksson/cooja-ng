/*
 * MSP430 Timer A/B peripheral — on-demand counter with event-driven interrupts.
 *
 * The counter is not incremented every CPU cycle. Instead, the current counter
 * value is computed from elapsed CPU cycles when it needs to be read. Compare
 * events are scheduled in the CPU event queue so interrupts fire at the right time.
 */
#include "msp430_timer.h"
#include "msp430_clock.h"
#include <string.h>
#include <stdio.h>

/* Timer register offsets from base_addr */
#define OFF_CTL    0x00   /* TxCTL */
#define OFF_CCTL0  0x02   /* TxCCTL0 */
/* TxCCTLn at OFF_CCTL0 + n*2 */
#define OFF_R      0x10   /* TxR (counter) */
#define OFF_CCR0   0x12   /* TxCCR0 */
/* TxCCRn at OFF_CCR0 + n*2 */

/* Overflow TIV values: Timer A = 0x0A, Timer B = 0x0E */
#define TIMER_A_OVERFLOW_TIV  0x0A
#define TIMER_B_OVERFLOW_TIV  0x0E

static void update_cycles_per_tick(msp430_timer_t *timer);
static void sync_counter(msp430_timer_t *timer);
static void schedule_ccr_event(msp430_timer_t *timer, int ccr_idx);
static void schedule_overflow_event(msp430_timer_t *timer);
static void schedule_all_events(msp430_timer_t *timer);
static void cancel_all_events(msp430_timer_t *timer);
static void trigger_ccr_interrupt(msp430_timer_t *timer, int ccr_idx);
static void reevaluate_tiv(msp430_timer_t *timer);
static void timer_interrupt_handler(void *user_data, int vector);

static int overflow_tiv(const msp430_timer_t *timer) {
    /* Timer A has 3 CCRs → overflow TIV = 0x0A (5 sources * 2)
     * Timer B has 7 CCRs → overflow TIV = 0x0E (7 sources * 2) */
    return timer->num_ccr * 2 + (timer->num_ccr <= 3 ? 4 : 0);
}

/* ---------- Counter computation ---------- */

static void sync_counter(msp430_timer_t *timer) {
    if (timer->mode == TIMER_MC_STOP || timer->cycles_per_tick <= 0) {
        return;
    }

    int64_t elapsed = timer->cpu->cycles - timer->counter_start;
    if (elapsed < 0) elapsed = 0;
    int64_t ticks = elapsed / timer->cycles_per_tick;

    switch (timer->mode) {
    case TIMER_MC_CONT:
        timer->counter = (timer->counter + (uint16_t)ticks) & 0xFFFF;
        break;
    case TIMER_MC_UP:
        if (timer->ccr[0] > 0) {
            uint32_t pos = timer->counter + (uint32_t)ticks;
            timer->counter = (uint16_t)(pos % (timer->ccr[0] + 1));
        }
        break;
    case TIMER_MC_UPDOWN:
        if (timer->ccr[0] > 0) {
            uint32_t period = timer->ccr[0] * 2;
            uint32_t pos = timer->counter + (uint32_t)ticks;
            pos = pos % period;
            if (pos > timer->ccr[0]) {
                timer->counter = (uint16_t)(period - pos);
            } else {
                timer->counter = (uint16_t)pos;
            }
        }
        break;
    }

    /* Advance counter_start by consumed ticks only, preserving sub-tick remainder.
     * Using cpu->cycles would discard fractional progress, causing the counter
     * to stall when read more frequently than cycles_per_tick. */
    timer->counter_start += ticks * timer->cycles_per_tick;
}

static void update_cycles_per_tick(msp430_timer_t *timer) {
    uint32_t aclk_freq = msp430_clock_get_aclk_freq(timer->clock);
    uint32_t smclk_freq = msp430_clock_get_smclk_freq(timer->clock);

    switch (timer->clock_source) {
    case TIMER_SRC_ACLK:
        /* cycles_per_tick = (cpu_freq / aclk_freq) * input_divider */
        if (aclk_freq > 0) {
            timer->cycles_per_tick = (int)((smclk_freq / aclk_freq) * timer->input_divider);
        }
        break;
    case TIMER_SRC_SMCLK:
        timer->cycles_per_tick = timer->input_divider;
        break;
    default:
        /* TCLK, INCLK — treat as SMCLK for now */
        timer->cycles_per_tick = timer->input_divider;
        break;
    }
    if (timer->cycles_per_tick < 1) timer->cycles_per_tick = 1;
}

/* ---------- Event callbacks ---------- */

static void ccr_event_callback(void *user_data, msp430_event_t *event) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;

    /* Find which CCR this event belongs to */
    int idx = (int)(event - timer->ccr_event);
    if (idx < 0 || idx >= timer->num_ccr) return;

    sync_counter(timer);

    if (timer->cctl[idx] & TIMER_CAP) {
        /* Capture mode: write pre-computed expected value to CCR
         * (matches Java MSPSim's expCompare approach) */
        if (timer->cctl[idx] & TIMER_CCIFG) {
            /* Previous capture not read yet — set overflow flag */
            timer->cctl[idx] |= TIMER_COV;
        }
        timer->ccr[idx] = timer->exp_compare[idx];
        /* Advance expCompare for next capture */
        timer->exp_compare[idx] = (timer->exp_compare[idx] + timer->exp_cap_interval[idx]) & 0xFFFF;
    }

    /* Set CCIFG */
    timer->cctl[idx] |= TIMER_CCIFG;

    /* Trigger interrupt */
    trigger_ccr_interrupt(timer, idx);

    /* Reschedule for next event */
    schedule_ccr_event(timer, idx);
}

static void overflow_event_callback(void *user_data, msp430_event_t *event) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;
    (void)event;

    sync_counter(timer);

    /* Set overflow flag in CTL */
    timer->ctl |= TIMER_TIFG;

    /* If overflow interrupt enabled, trigger on shared vector */
    if (timer->ctl & TIMER_TIE) {
        /* Overflow is lowest priority, so only set TIV if nothing else pending */
        if (timer->last_tiv == 0) {
            timer->last_tiv = overflow_tiv(timer);
        }
        msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer,
                              timer_interrupt_handler, true);
    }

    /* Reschedule overflow */
    schedule_overflow_event(timer);
}

/* ---------- Event scheduling ---------- */

static void schedule_ccr_event(msp430_timer_t *timer, int ccr_idx) {
    if (timer->mode == TIMER_MC_STOP || timer->cycles_per_tick <= 0) return;

    uint16_t cctl = timer->cctl[ccr_idx];

    /* Capture mode: schedule based on capture input source frequency.
     * Matches Java MSPSim's updateCaptures/expCompare approach:
     * - exp_cap_interval = timer_clock_freq / capture_input_freq (in timer ticks)
     * - exp_compare = previous CCR + exp_cap_interval (pre-computed capture value)
     * - Event scheduled at current + exp_cap_interval * cycles_per_tick CPU cycles
     * - This is called on CCTL writes (including firmware clearing CCIFG),
     *   which recalculates exp_cap_interval with the current clock frequencies. */
    if (cctl & TIMER_CAP) {
        int cm = (cctl & TIMER_CM_MASK) >> TIMER_CM_SHIFT;
        if (cm == 0) return;  /* No capture (disabled) */

        int ccis = (cctl & TIMER_CCIS_MASK) >> TIMER_CCIS_SHIFT;
        int cap_interval;

        if (ccis == 1) {
            /* CCIS=01: CCI_B input, typically ACLK */
            uint32_t aclk_freq = msp430_clock_get_aclk_freq(timer->clock);
            uint32_t smclk_freq = msp430_clock_get_smclk_freq(timer->clock);
            if (aclk_freq == 0) return;
            /* Timer ticks between ACLK edges */
            if (timer->clock_source == TIMER_SRC_SMCLK) {
                cap_interval = (int)(smclk_freq / aclk_freq);
            } else if (timer->clock_source == TIMER_SRC_ACLK) {
                /* Timer clocked by ACLK, capturing ACLK — each tick is a capture */
                cap_interval = 1;
            } else {
                cap_interval = (int)(smclk_freq / aclk_freq);
            }
        } else if (ccis == 0) {
            /* CCIS=00: CCI_A input (external pin) — not connected, use large period */
            cap_interval = 100000;
        } else {
            /* CCIS=10: GND, CCIS=11: VCC — no capture events */
            return;
        }
        if (cap_interval < 1) cap_interval = 1;

        /* Update expected capture interval and compute next expected value */
        timer->exp_cap_interval[ccr_idx] = cap_interval;
        timer->exp_compare[ccr_idx] = (timer->ccr[ccr_idx] + cap_interval) & 0xFFFF;

        /* Schedule event: cap_interval timer ticks from counter_start.
         * Use counter_start (not cpu->cycles) so sub-tick remainder is preserved. */
        int64_t fire_cycle = timer->counter_start + (int64_t)cap_interval * timer->cycles_per_tick;
        msp430_schedule_event(timer->cpu, &timer->ccr_event[ccr_idx], fire_cycle);
        return;
    }

    /* Compare mode: schedule when counter reaches CCR value */
    uint16_t ccr_val = timer->ccr[ccr_idx];
    uint16_t counter = timer->counter;
    int64_t ticks_to_match;

    switch (timer->mode) {
    case TIMER_MC_CONT:
        if (ccr_val > counter) {
            ticks_to_match = ccr_val - counter;
        } else {
            /* Counter must wrap around */
            ticks_to_match = (0x10000 - counter) + ccr_val;
        }
        break;
    case TIMER_MC_UP:
        if (ccr_idx == 0) {
            /* CCR0 in UP mode: fires when counter reaches CCR0 */
            if (ccr_val > counter) {
                ticks_to_match = ccr_val - counter;
            } else {
                ticks_to_match = ccr_val + 1;  /* wrap to 0 then count up */
            }
        } else {
            if (ccr_val > counter && ccr_val <= timer->ccr[0]) {
                ticks_to_match = ccr_val - counter;
            } else {
                /* Wait for wrap then count to ccr_val */
                ticks_to_match = (timer->ccr[0] + 1 - counter) + ccr_val;
            }
        }
        break;
    case TIMER_MC_UPDOWN:
        /* Simplified: schedule like continuous for now */
        if (ccr_val > counter) {
            ticks_to_match = ccr_val - counter;
        } else {
            ticks_to_match = (0x10000 - counter) + ccr_val;
        }
        break;
    default:
        return;
    }

    /* Use counter_start (not cpu->cycles) so sub-tick remainder is preserved. */
    int64_t fire_cycle = timer->counter_start + ticks_to_match * timer->cycles_per_tick;
    msp430_schedule_event(timer->cpu, &timer->ccr_event[ccr_idx], fire_cycle);
}

static void schedule_overflow_event(msp430_timer_t *timer) {
    if (timer->mode == TIMER_MC_STOP || timer->cycles_per_tick <= 0) return;

    int64_t ticks_to_overflow;

    switch (timer->mode) {
    case TIMER_MC_CONT:
        ticks_to_overflow = 0x10000 - timer->counter;
        break;
    case TIMER_MC_UP:
        if (timer->ccr[0] > 0) {
            ticks_to_overflow = timer->ccr[0] + 1 - timer->counter;
        } else {
            ticks_to_overflow = 0x10000;
        }
        break;
    case TIMER_MC_UPDOWN:
        ticks_to_overflow = 0x10000 - timer->counter;
        break;
    default:
        return;
    }

    /* Use counter_start (not cpu->cycles) so sub-tick remainder is preserved. */
    int64_t fire_cycle = timer->counter_start + ticks_to_overflow * timer->cycles_per_tick;
    msp430_schedule_event(timer->cpu, &timer->overflow_event, fire_cycle);
}

static void schedule_all_events(msp430_timer_t *timer) {
    sync_counter(timer);
    for (int i = 0; i < timer->num_ccr; i++) {
        schedule_ccr_event(timer, i);
    }
    schedule_overflow_event(timer);
}

static void cancel_all_events(msp430_timer_t *timer) {
    for (int i = 0; i < timer->num_ccr; i++) {
        msp430_cancel_event(timer->cpu, &timer->ccr_event[i]);
    }
    msp430_cancel_event(timer->cpu, &timer->overflow_event);
}

/* ---------- Interrupt management ---------- */

static void trigger_ccr_interrupt(msp430_timer_t *timer, int ccr_idx) {
    if (!(timer->cctl[ccr_idx] & TIMER_CCIE)) return;
    if (!(timer->cctl[ccr_idx] & TIMER_CCIFG)) return;

    if (ccr_idx == 0) {
        /* CCR0 has its own dedicated vector — auto-clears CCIFG on service */
        msp430_flag_interrupt(timer->cpu, timer->ccr0_vector, timer,
                              timer_interrupt_handler, true);
    } else {
        /* CCR1+ share vector, use TIV for identification */
        int tiv_val = ccr_idx * 2;
        if (timer->last_tiv == 0 || tiv_val < timer->last_tiv) {
            timer->last_tiv = tiv_val;
        }
        msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer,
                              timer_interrupt_handler, true);
    }
}

static void reevaluate_tiv(msp430_timer_t *timer) {
    timer->last_tiv = 0;

    /* Check CCR1+ (lower index = higher priority = lower TIV) */
    for (int i = 1; i < timer->num_ccr; i++) {
        if ((timer->cctl[i] & TIMER_CCIE) && (timer->cctl[i] & TIMER_CCIFG)) {
            timer->last_tiv = i * 2;
            msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer,
                                  timer_interrupt_handler, true);
            return;
        }
    }

    /* Check overflow (lowest priority) */
    if ((timer->ctl & TIMER_TIE) && (timer->ctl & TIMER_TIFG)) {
        timer->last_tiv = overflow_tiv(timer);
        msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer,
                              timer_interrupt_handler, true);
        return;
    }

    /* Nothing pending — unflag interrupt */
    msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer, NULL, false);
}

/* ---------- IO read/write handlers ---------- */

static int timer_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - timer->base_addr;

    /* TxR — counter register */
    if (offset == OFF_R || offset == OFF_R + 1) {
        sync_counter(timer);
        if (offset == OFF_R + 1) return (timer->counter >> 8) & 0xFF;
        return timer->counter;
    }

    /* TxCTL */
    if (offset == OFF_CTL || offset == OFF_CTL + 1) {
        if (offset == OFF_CTL + 1) return (timer->ctl >> 8) & 0xFF;
        return timer->ctl;
    }

    /* TxCCTLn */
    if (offset >= OFF_CCTL0 && offset < OFF_CCTL0 + timer->num_ccr * 2) {
        int idx = (offset - OFF_CCTL0) / 2;
        if ((offset - OFF_CCTL0) & 1) return (timer->cctl[idx] >> 8) & 0xFF;
        return timer->cctl[idx];
    }

    /* TxCCRn */
    if (offset >= OFF_CCR0 && offset < OFF_CCR0 + timer->num_ccr * 2) {
        int idx = (offset - OFF_CCR0) / 2;
        if ((offset - OFF_CCR0) & 1) return (timer->ccr[idx] >> 8) & 0xFF;
        return timer->ccr[idx];
    }

    return 0;
}

static int timer_iv_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;
    (void)addr; (void)word; (void)cycles;

    int val = timer->last_tiv;

    /* Clear the source that generated this TIV value */
    if (val > 0) {
        int ovf_tiv = overflow_tiv(timer);
        if (val == ovf_tiv) {
            /* Overflow: clear TIFG */
            timer->ctl &= ~TIMER_TIFG;
        } else {
            /* CCR source: clear CCIFG for that CCR */
            int ccr_idx = val / 2;
            if (ccr_idx >= 1 && ccr_idx < timer->num_ccr) {
                timer->cctl[ccr_idx] &= ~TIMER_CCIFG;
            }
        }
        /* Re-evaluate for next highest priority pending source */
        reevaluate_tiv(timer);
    }

    return val;
}

static void timer_iv_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    /* TAIV/TBIV is read-only; writes are ignored */
    (void)user_data; (void)addr; (void)value; (void)word; (void)cycles;
}

static void timer_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - timer->base_addr;

    /* TxCTL */
    if (offset == OFF_CTL || offset == OFF_CTL + 1) {
        sync_counter(timer);

        uint16_t old_ctl = timer->ctl;
        if (word || offset == OFF_CTL) {
            if (word) {
                timer->ctl = (uint16_t)value;
            } else {
                timer->ctl = (timer->ctl & 0xFF00) | (value & 0xFF);
            }
        } else {
            timer->ctl = (timer->ctl & 0x00FF) | ((value & 0xFF) << 8);
        }

        /* Parse fields */
        timer->clock_source = (timer->ctl >> TIMER_TASSEL_SHIFT) & 3;
        timer->input_divider = 1 << ((timer->ctl >> TIMER_ID_SHIFT) & 3);
        int new_mode = (timer->ctl >> TIMER_MC_SHIFT) & 3;

        /* Clear counter if TCLR bit set */
        if (timer->ctl & TIMER_TCLR) {
            timer->counter = 0;
            timer->counter_start = timer->cpu->cycles;
            timer->ctl &= ~TIMER_TCLR;
        }

        update_cycles_per_tick(timer);

        int old_mode = (old_ctl >> TIMER_MC_SHIFT) & 3;
        timer->mode = new_mode;

        if (new_mode != TIMER_MC_STOP && old_mode == TIMER_MC_STOP) {
            /* Timer started — schedule events */
            timer->counter_start = timer->cpu->cycles;
            schedule_all_events(timer);
        } else if (new_mode == TIMER_MC_STOP) {
            cancel_all_events(timer);
        } else if (new_mode != old_mode) {
            /* Mode changed while running */
            schedule_all_events(timer);
        }
        return;
    }

    /* TxR — direct counter write */
    if (offset == OFF_R || offset == OFF_R + 1) {
        sync_counter(timer);
        if (word || offset == OFF_R) {
            timer->counter = (uint16_t)value;
        } else {
            timer->counter = (timer->counter & 0x00FF) | ((value & 0xFF) << 8);
        }
        timer->counter_start = timer->cpu->cycles;
        if (timer->mode != TIMER_MC_STOP) {
            schedule_all_events(timer);
        }
        return;
    }

    /* TxCCTLn */
    if (offset >= OFF_CCTL0 && offset < OFF_CCTL0 + timer->num_ccr * 2) {
        int idx = (offset - OFF_CCTL0) / 2;
        if (word || !((offset - OFF_CCTL0) & 1)) {
            if (word) {
                timer->cctl[idx] = (uint16_t)value;
            } else {
                timer->cctl[idx] = (timer->cctl[idx] & 0xFF00) | (value & 0xFF);
            }
        } else {
            timer->cctl[idx] = (timer->cctl[idx] & 0x00FF) | ((value & 0xFF) << 8);
        }

        /* Re-evaluate interrupts */
        if (idx == 0) {
            if ((timer->cctl[0] & TIMER_CCIE) && (timer->cctl[0] & TIMER_CCIFG)) {
                msp430_flag_interrupt(timer->cpu, timer->ccr0_vector, timer,
                                      timer_interrupt_handler, true);
            } else {
                msp430_flag_interrupt(timer->cpu, timer->ccr0_vector, timer, NULL, false);
            }
        } else {
            reevaluate_tiv(timer);
        }

        /* Reschedule this CCR's event (capture mode may have changed) */
        if (timer->mode != TIMER_MC_STOP) {
            schedule_ccr_event(timer, idx);
        }
        return;
    }

    /* TxCCRn */
    if (offset >= OFF_CCR0 && offset < OFF_CCR0 + timer->num_ccr * 2) {
        int idx = (offset - OFF_CCR0) / 2;

        sync_counter(timer);

        if (word || !((offset - OFF_CCR0) & 1)) {
            if (word) {
                timer->ccr[idx] = (uint16_t)value;
            } else {
                timer->ccr[idx] = (timer->ccr[idx] & 0xFF00) | (value & 0xFF);
            }
        } else {
            timer->ccr[idx] = (timer->ccr[idx] & 0x00FF) | ((value & 0xFF) << 8);
        }

        /* Reschedule event for this CCR */
        if (timer->mode != TIMER_MC_STOP) {
            schedule_ccr_event(timer, idx);
            /* If CCR0 changed and we're in UP mode, reschedule overflow too */
            if (idx == 0 && timer->mode == TIMER_MC_UP) {
                schedule_overflow_event(timer);
            }
        }
        return;
    }
}

/* ---------- Interrupt handler (called when CPU services the interrupt) ---------- */

static void timer_interrupt_handler(void *user_data, int vector) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;

    if (vector == timer->ccr0_vector) {
        /* CCR0 interrupt serviced: clear CCIFG */
        timer->cctl[0] &= ~TIMER_CCIFG;
    }
    /* For CCR1+ vector, the firmware reads TAIV/TBIV which clears flags */
}

/* ---------- Clock frequency changed ---------- */

void msp430_timer_clock_changed(msp430_timer_t *timer) {
    if (timer->mode == TIMER_MC_STOP) return;

    sync_counter(timer);
    update_cycles_per_tick(timer);

    /* Reschedule compare events and overflow with updated frequencies.
     * Capture events are NOT rescheduled here — they are only rescheduled
     * when the firmware writes to CCTLn (e.g., clearing CCIFG), which
     * matches Java MSPSim's behavior and ensures correct DCO calibration. */
    for (int i = 0; i < timer->num_ccr; i++) {
        if (!(timer->cctl[i] & TIMER_CAP)) {
            schedule_ccr_event(timer, i);
        }
    }
    schedule_overflow_event(timer);
}

/* ---------- Initialization ---------- */

void msp430_timer_init(msp430_timer_t *timer, msp430_cpu_t *cpu,
                        struct msp430_clock *clock,
                        const char *name, uint32_t base_addr, uint32_t iv_addr,
                        int num_ccr, int ccr0_vector, int ccr1_vector) {
    memset(timer, 0, sizeof(*timer));
    timer->cpu = cpu;
    timer->clock = clock;
    timer->name = name;
    timer->base_addr = base_addr;
    timer->iv_addr = iv_addr;
    timer->num_ccr = num_ccr > TIMER_MAX_CCR ? TIMER_MAX_CCR : num_ccr;
    timer->ccr0_vector = ccr0_vector;
    timer->ccr1_vector = ccr1_vector;
    timer->cycles_per_tick = 1;

    /* Initialize event callbacks */
    for (int i = 0; i < timer->num_ccr; i++) {
        timer->ccr_event[i].callback = ccr_event_callback;
        timer->ccr_event[i].user_data = timer;
    }
    timer->overflow_event.callback = overflow_event_callback;
    timer->overflow_event.user_data = timer;

    /* Register IO for timer registers.
     * Base registers: CTL at base+0, CCTLn at base+2..base+2+2*n,
     * R at base+0x10, CCRn at base+0x12..
     * Total range: base to base + 0x12 + 2*num_ccr */
    uint32_t reg_size = OFF_CCR0 + num_ccr * 2;
    msp430_register_io(cpu, base_addr, reg_size, timer_read, timer_write, timer);

    /* Register TAIV/TBIV separately (different address, different read behavior) */
    msp430_register_io(cpu, iv_addr, 2, timer_iv_read, timer_iv_write, timer);

    /* Register interrupt handlers so we get notified when CPU services our interrupts */
    /* We use flag_interrupt with handler for CCR0 auto-clear */
    /* Actually, we set up the handler at flag time. Instead let's just note
     * that CCR0 CCIFG is auto-cleared by the hardware when serviced. We handle
     * this via the interrupt_handler callback. */
}
