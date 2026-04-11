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

/* Match MSPSim Timer.updateCounter() exactly:
 * Computes counter ABSOLUTELY from (cycles - counterStart) / divider + counterAcc.
 * No incremental accumulation → no drift from repeated syncs. */
static void sync_counter(msp430_timer_t *timer) {
    if (timer->mode == TIMER_MC_STOP || timer->cycles_per_tick <= 0) {
        return;
    }

    int64_t cycctr = timer->cpu->cycles - timer->counter_start;
    if (cycctr < 0) cycctr = 0;
    double tick = (double)cycctr / timer->cycles_per_tick;
    int64_t tick_long = (int64_t)tick;
    timer->counter_passed = (int)(timer->cycles_per_tick * (tick - (double)tick_long));
    int64_t big_counter = tick_long + timer->counter_acc;

    switch (timer->mode) {
    case TIMER_MC_CONT:
        timer->counter = (uint16_t)(big_counter & 0xFFFF);
        break;
    case TIMER_MC_UP:
        if (timer->ccr[0] > 0) {
            timer->counter = (uint16_t)(big_counter % (timer->ccr[0] + 1));
        } else {
            timer->counter = 0;
        }
        break;
    case TIMER_MC_UPDOWN:
        if (timer->ccr[0] > 0) {
            int64_t period = timer->ccr[0] * 2;
            int64_t pos = big_counter % period;
            if (pos > timer->ccr[0])
                timer->counter = (uint16_t)(period - pos);
            else
                timer->counter = (uint16_t)pos;
        } else {
            timer->counter = 0;
        }
        break;
    }
}

/* Match MSPSim Timer.resetCounter(): snapshot counterAcc and counterStart.
 * Called after counter writes, mode changes, or frequency changes. */
static void reset_counter(msp430_timer_t *timer) {
    int64_t cycctr = timer->cpu->cycles - timer->counter_start;
    if (cycctr < 0) cycctr = 0;
    double tick = (double)cycctr / timer->cycles_per_tick;
    timer->counter_passed = (int)(timer->cycles_per_tick * (tick - (double)(int64_t)tick));
    timer->counter_start = timer->cpu->cycles - timer->counter_passed;
    timer->counter_acc = timer->counter;
}

static void update_cycles_per_tick(msp430_timer_t *timer) {
    /* Match MSPSim's updateCyclesMultiplicator():
     *   cyclesMultiplicator = inputDivider;
     *   if (clockSource == SRC_ACLK)
     *       cyclesMultiplicator = (cyclesMultiplicator * cpu.smclkFrq) / cpu.aclkFrq; */
    uint32_t aclk_freq = msp430_clock_get_aclk_freq(timer->clock);
    uint32_t smclk_freq = msp430_clock_get_smclk_freq(timer->clock);
    /* ACLK is always 32768 Hz (crystal), even before clock module init */
    if (aclk_freq == 0) aclk_freq = 32768;
    /* SMCLK defaults to DCO; use cpu_freq_hz if clock module not ready */
    if (smclk_freq == 0 && timer->cpu) smclk_freq = timer->cpu->cpu_freq_hz;

    double new_cpt = timer->input_divider;
    if (timer->clock_source == TIMER_SRC_ACLK && aclk_freq > 0 && smclk_freq > 0) {
        new_cpt = (new_cpt * (double)smclk_freq) / aclk_freq;
    }
    if (new_cpt < 1.0) new_cpt = 1.0;

    /* When cpt changes, sync counter with OLD cpt first (to get correct
     * current counter value), then update cpt, then reset counter base.
     * Without this, counter_acc from cpt=1.0 era corrupts the absolute formula. */
    if (new_cpt != timer->cycles_per_tick && timer->cycles_per_tick > 0 &&
        timer->mode != TIMER_MC_STOP) {
        sync_counter(timer);     /* sync with OLD cpt → correct counter value */
        timer->cycles_per_tick = new_cpt;
        reset_counter(timer);    /* reset counter_acc/start with NEW cpt */
    } else {
        timer->cycles_per_tick = new_cpt;
    }
}

/* ---------- Event callbacks ---------- */

static int ccr_fire_count[2][8]; /* [timer_a=0/timer_b=1][ccr_idx] */

static void ccr_event_callback(void *user_data, msp430_event_t *event) {
    msp430_timer_t *timer = (msp430_timer_t *)user_data;

    /* Find which CCR this event belongs to */
    int idx = (int)(event - timer->ccr_event);
    if (idx < 0 || idx >= timer->num_ccr) return;

    /* Debug counter */
    int timer_idx = (timer->base_addr == 0x160) ? 0 : 1;
    ccr_fire_count[timer_idx][idx]++;

    /* Debug: count Timer A CCR1 fires (clock tick) */

    sync_counter(timer);


    if (timer->cctl[idx] & TIMER_CAP) {
        /* Capture mode: matches MSPSim's CCR.execute() for captures.
         * Write pre-computed expected value to CCR, advance for next capture,
         * and reschedule incrementally (not from scratch). */
        if (timer->cctl[idx] & TIMER_CCIFG) {
            /* Previous capture not read yet — set overflow flag */
            timer->cctl[idx] |= TIMER_COV;
        }
        timer->ccr[idx] = timer->exp_compare[idx];
        /* Advance expCompare for next capture */
        timer->exp_compare[idx] = (timer->exp_compare[idx] + timer->exp_cap_interval[idx]) & 0xFFFF;

        /* Set CCIFG and trigger interrupt */
        timer->cctl[idx] |= TIMER_CCIFG;
        trigger_ccr_interrupt(timer, idx);

        /* MSPSim: expCaptureTime += expCapInterval * cyclesMultiplicator
         * Schedule next capture incrementally from current fire time.
         * Throttle non-interrupt captures (e.g., Timer B CCR6 SFD timestamps)
         * to prevent event queue starvation. ACLK-rate captures at 32K/s
         * create 62M events in 600s. Limit to ~100/s when CCIE is not set. */
        int64_t interval = (int64_t)(timer->exp_cap_interval[idx] * timer->cycles_per_tick);
        if (!(timer->cctl[idx] & TIMER_CCIE)) {
            int64_t min_interval = timer->cpu->cpu_freq_hz / 100;  /* 100/s */
            if (interval < min_interval) interval = min_interval;
        }
        int64_t next_fire = event->fire_cycle + interval;
        msp430_schedule_event(timer->cpu, &timer->ccr_event[idx], next_fire);
        return;
    }

    /* Set CCIFG */
    timer->cctl[idx] |= TIMER_CCIFG;

    /* Trigger interrupt */
    trigger_ccr_interrupt(timer, idx);

    /* Reschedule for next wrap from THIS fire time.
     * Using event->fire_cycle as base (not recalculating from counter)
     * avoids the rounding issue where ccr_val ≈ counter causes a full
     * 65536-tick wrap-around instead of the correct next-wrap schedule.
     * The firmware ISR writes new TACCR value, which re-schedules earlier. */
    {
        int64_t wrap_cycles = (int64_t)(0x10000 * timer->cycles_per_tick);
        int64_t next_fire = event->fire_cycle + wrap_cycles;
        msp430_schedule_event(timer->cpu, &timer->ccr_event[idx], next_fire);
    }
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

    /* Capture mode: matches MSPSim's CCR.updateCaptures() exactly.
     *
     * MSPSim computes:
     *   frqClk = (clockSource==SMCLK) ? smclkFrq/inputDivider
     *          : (clockSource==ACLK)  ? aclkFrq/inputDivider : 1
     *   divisor = (inputSrc==ACLK) ? aclkFrq : 1
     *   expCapInterval = frqClk / divisor   (timer ticks between captures)
     *   if clkSource: expCompare = (tccr + expCapInterval) & 0xffff
     *   else:         expCompare = (counter + expCapInterval) & 0xffff
     *   expCaptureTime = cycles + expCapInterval * cyclesMultiplicator
     */
    if (cctl & TIMER_CAP) {
        int cm = (cctl & TIMER_CM_MASK) >> TIMER_CM_SHIFT;
        if (cm == 0) return;  /* No capture (disabled) */

        int ccis = (cctl & TIMER_CCIS_MASK) >> TIMER_CCIS_SHIFT;

        /* Determine capture input source.  CCIS=1 => ACLK for most configs. */
        bool clk_source = false;  /* true when capture source is a clock (ACLK) */
        int divisor = 1;
        if (ccis == 1) {
            /* CCI_B input, typically ACLK */
            uint32_t aclk_freq = msp430_clock_get_aclk_freq(timer->clock);
            if (aclk_freq == 0) return;
            divisor = (int)aclk_freq;
            clk_source = true;
        } else if (ccis == 0) {
            /* CCI_A input (external pin) — not connected, use large period */
            divisor = 1;
            clk_source = false;
        } else {
            /* CCIS=10: GND, CCIS=11: VCC — no capture events */
            return;
        }

        /* Compute frqClk = timer clock frequency after input divider */
        uint32_t smclk_freq = msp430_clock_get_smclk_freq(timer->clock);
        uint32_t aclk_freq = msp430_clock_get_aclk_freq(timer->clock);
        int frq_clk = 1;
        if (timer->clock_source == TIMER_SRC_SMCLK) {
            frq_clk = (int)(smclk_freq / timer->input_divider);
        } else if (timer->clock_source == TIMER_SRC_ACLK) {
            frq_clk = (int)(aclk_freq / timer->input_divider);
        }

        int cap_interval = frq_clk / divisor;
        if (cap_interval < 1) cap_interval = 1;

        /* Update expected capture interval and compute next expected value.
         * When capture source is a clock (ACLK), use tccr as base (MSPSim).
         * Otherwise use current counter value. */
        timer->exp_cap_interval[ccr_idx] = cap_interval;
        if (clk_source) {
            timer->exp_compare[ccr_idx] = (timer->ccr[ccr_idx] + cap_interval) & 0xFFFF;
        } else {
            timer->exp_compare[ccr_idx] = (timer->counter + cap_interval) & 0xFFFF;
        }

        /* Schedule event: cap_interval timer ticks from now, using double
         * cyclesMultiplicator matching MSPSim's:
         *   expCaptureTime = cycles + (long)(expCapInterval * cyclesMultiplicator) */
        int64_t fire_cycle = timer->cpu->cycles + (int64_t)(cap_interval * timer->cycles_per_tick);
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

    /* Schedule based on counter_start. The absolute counter formula means:
     * counter = (cycles - counter_start) / cpt + counter_acc
     * We want counter to reach ccr_val, which is counter_acc + total_ticks.
     * So: total_ticks = ccr_val - counter_acc (mod 0x10000)
     * fire_cycle = counter_start + total_ticks * cpt
     * But simpler: ticks_to_match from current counter, use cpu->cycles as base. */
    int64_t fire_cycle = timer->cpu->cycles - timer->counter_passed +
        (int64_t)(ticks_to_match * timer->cycles_per_tick + 1);

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

    int64_t fire_cycle = timer->cpu->cycles - timer->counter_passed +
        (int64_t)(ticks_to_overflow * timer->cycles_per_tick + 1);
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
            timer->counter_acc = 0;
            timer->counter_start = timer->cpu->cycles;
            timer->counter_passed = 0;
            timer->ctl &= ~TIMER_TCLR;
        }

        update_cycles_per_tick(timer);

        int old_mode = (old_ctl >> TIMER_MC_SHIFT) & 3;
        timer->mode = new_mode;

        if (new_mode != TIMER_MC_STOP && old_mode == TIMER_MC_STOP) {
            /* Timer started — reset and schedule events */
            reset_counter(timer);
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
        /* Reset the absolute counter base (like MSPSim resetCounter) */
        reset_counter(timer);
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
    reset_counter(timer);
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

/* External capture input: capture timer counter into CCR when pin changes.
 * Matches MSPSim Timer.java capture behavior for SFD timestamping.
 * The firmware configures CCRn in capture mode (CAP=1, CM=both edges).
 * When the external input (SFD pin) changes, the current counter value
 * is latched into CCRn, CCIFG is set, and an interrupt is generated. */
void msp430_timer_capture_input(msp430_timer_t *timer, int ccr_idx, bool value) {
    if (!timer || !timer->cpu) return;
    if (ccr_idx < 0 || ccr_idx >= timer->num_ccr) return;
    if (timer->mode == TIMER_MC_STOP) return;
    uint16_t cctl = timer->cctl[ccr_idx];
    if (!(cctl & TIMER_CAP)) return; /* Not in capture mode */

    /* Check capture mode (CM): 01=rising, 10=falling, 11=both */
    int cm = (cctl >> 14) & 3;
    if (cm == 0) return;
    if (cm == 1 && !value) return;
    if (cm == 2 && value) return;

    /* Sync counter using the proper sync_counter (preserves sub-tick remainder) */
    sync_counter(timer);

    /* Latch counter into CCR, set CCIFG. If CCIFG is already set
     * (previous capture not yet read), set COV and do NOT overwrite CCR.
     * This matches MSP430 hardware: capture overflow preserves the first value. */
    if (cctl & TIMER_CCIFG) {
        timer->cctl[ccr_idx] |= TIMER_COV;
        /* Don't overwrite CCR — preserve first capture value */
    } else {
        timer->ccr[ccr_idx] = timer->counter;
        timer->cctl[ccr_idx] |= TIMER_CCIFG;
    }

    /* Set CCI bit */
    if (value) timer->cctl[ccr_idx] |= (1 << 3);
    else timer->cctl[ccr_idx] &= ~(1 << 3);

    /* Flag interrupt if CCIE enabled */
    if (timer->cctl[ccr_idx] & TIMER_CCIE) {
        if (ccr_idx == 0) {
            msp430_flag_interrupt(timer->cpu, timer->ccr0_vector, timer,
                                  timer_interrupt_handler, true);
        } else {
            int tiv_val = ccr_idx * 2;
            if (timer->last_tiv == 0 || tiv_val < timer->last_tiv)
                timer->last_tiv = tiv_val;
            msp430_flag_interrupt(timer->cpu, timer->ccr1_vector, timer,
                                  timer_interrupt_handler, true);
        }
    }
}

/* Debug: report CCR fire counts */
void msp430_timer_dump_ccr_counts(void) {
    fprintf(stderr, "  Timer A CCR fires:");
    for (int i = 0; i < 8; i++)
        if (ccr_fire_count[0][i]) fprintf(stderr, " CCR%d=%d", i, ccr_fire_count[0][i]);
    fprintf(stderr, "\n  Timer B CCR fires:");
    for (int i = 0; i < 8; i++)
        if (ccr_fire_count[1][i]) fprintf(stderr, " CCR%d=%d", i, ccr_fire_count[1][i]);
    fprintf(stderr, "\n");
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
