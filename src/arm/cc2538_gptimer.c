/*
 * CC2538 General Purpose Timer
 */
#include "cc2538_gptimer.h"
#include "arm_nvic.h"
#include <string.h>

#define GPTIMER_REG_SIZE  0x1000

static uint32_t gptimer_ta_prescaler(const cc2538_gptimer_t *timer) {
    /* Prescaler: in 16-bit individual mode (cfg=0x04), TAPR divides clock */
    return (timer->cfg == 0x04) ? (timer->tapr + 1) : 1;
}

static uint32_t gptimer_ta_load(const cc2538_gptimer_t *timer) {
    /* Load value (16-bit in individual mode, 32-bit otherwise) */
    uint32_t load = timer->tailr;
    if (timer->cfg == 0x04) load &= 0xFFFF;
    return load;
}

static void gptimer_raise_ta_irq(cc2538_gptimer_t *timer) {
    timer->ris |= (1 << 0);  /* TATORIS: timeout raw interrupt */
    if ((timer->imr & (1 << 0)) && timer->cpu->nvic)
        arm_nvic_set_pending((arm_nvic_t *)timer->cpu->nvic, timer->irq_a);
}

/* Timeout event — fires at the scheduled expiry so a WFI'd CPU wakes up.
 * One-shot auto-stops; periodic reloads (advance the start anchor by whole
 * periods so TAR/TAV reads stay phase-correct) and reschedules. */
static void gptimer_ta_fired(void *user_data, arm_event_t *ev) {
    (void)ev;
    cc2538_gptimer_t *timer = (cc2538_gptimer_t *)user_data;
    if (!(timer->ctl & GPTIMER_CTL_TAEN)) return;

    uint32_t mode = timer->tamr & 0x03;
    gptimer_raise_ta_irq(timer);
    if (mode == 0x01) {
        /* One-shot: auto-stop */
        timer->ctl &= ~GPTIMER_CTL_TAEN;
        timer->tav = 0;
    } else if (mode == 0x02) {
        /* Periodic: reload and rearm one period out */
        uint32_t load = gptimer_ta_load(timer);
        int64_t period = (int64_t)(load + 1) * gptimer_ta_prescaler(timer);
        if (period <= 0) period = 1;
        timer->ta_start_cycle += period;
        arm_schedule_event(timer->cpu, &timer->ta_event,
                           timer->ta_start_cycle + period);
    }
}

/* (Re)arm the timeout event from the current anchor. */
static void gptimer_ta_schedule(cc2538_gptimer_t *timer) {
    arm_cancel_event(timer->cpu, &timer->ta_event);
    if (!(timer->ctl & GPTIMER_CTL_TAEN)) return;
    uint32_t mode = timer->tamr & 0x03;
    if (mode != 0x01 && mode != 0x02) return;   /* capture/PWM not modeled */
    uint32_t load = gptimer_ta_load(timer);
    int64_t period = (int64_t)(load + 1) * gptimer_ta_prescaler(timer);
    if (period <= 0) period = 1;
    arm_schedule_event(timer->cpu, &timer->ta_event,
                       timer->ta_start_cycle + period);
}

/* Poll-path expiry check (registers read before the event fires). */
static void gptimer_check_ta_expiry(cc2538_gptimer_t *timer) {
    if (!(timer->ctl & GPTIMER_CTL_TAEN)) return;

    int64_t elapsed_cycles = timer->cpu->cycles - timer->ta_start_cycle;
    if (elapsed_cycles < 0) elapsed_cycles = 0;

    uint32_t prescaler = gptimer_ta_prescaler(timer);
    uint32_t elapsed_ticks = (uint32_t)((uint64_t)elapsed_cycles / prescaler);
    uint32_t load = gptimer_ta_load(timer);
    uint32_t mode = timer->tamr & 0x03;

    if (mode == 0x01 && elapsed_ticks >= load) {
        /* One-shot: timer expired — clear TAEN (auto-stop) */
        arm_cancel_event(timer->cpu, &timer->ta_event);
        timer->ctl &= ~GPTIMER_CTL_TAEN;
        timer->tav = 0;
        timer->ris |= (1 << 0);  /* TATORIS: timeout raw interrupt */
    } else if (mode == 0x02 && elapsed_ticks > load) {
        /* Periodic: at least one wrap since the anchor — latch the raw
         * interrupt for pollers; the scheduled event owns IRQ + reload. */
        timer->ris |= (1 << 0);
    }
}

static int gptimer_read(void *user_data, uint32_t addr) {
    cc2538_gptimer_t *timer = (cc2538_gptimer_t *)user_data;
    uint32_t offset = addr - timer->base_addr;

    switch (offset) {
        case GPTIMER_CFG:      return (int)timer->cfg;
        case GPTIMER_TAMR:     return (int)timer->tamr;
        case GPTIMER_TBMR:     return (int)timer->tbmr;
        case GPTIMER_CTL:
            gptimer_check_ta_expiry(timer);
            return (int)timer->ctl;
        case GPTIMER_IMR:      return (int)timer->imr;
        case GPTIMER_RIS:
            gptimer_check_ta_expiry(timer);
            return (int)timer->ris;
        case GPTIMER_MIS:
            gptimer_check_ta_expiry(timer);
            return (int)(timer->ris & timer->imr);
        case GPTIMER_ICR:      return 0;
        case GPTIMER_TAILR:    return (int)timer->tailr;
        case GPTIMER_TBILR:    return (int)timer->tbilr;
        case GPTIMER_TAMATCHR: return (int)timer->tamatchr;
        case GPTIMER_TBMATCHR: return (int)timer->tbmatchr;
        case GPTIMER_TAPR:     return (int)timer->tapr;
        case GPTIMER_TBPR:     return (int)timer->tbpr;
        case GPTIMER_TAR:
        case GPTIMER_TAV: {
            gptimer_check_ta_expiry(timer);
            if (timer->ctl & GPTIMER_CTL_TAEN) {
                int64_t elapsed = timer->cpu->cycles - timer->ta_start_cycle;
                uint32_t prescaler = (timer->cfg == 0x04) ? (timer->tapr + 1) : 1;
                uint32_t elapsed_ticks = (uint32_t)((uint64_t)elapsed / prescaler);
                uint32_t load = timer->tailr;
                if (timer->cfg == 0x04) load &= 0xFFFF;
                int count_down = !(timer->tamr & (1 << 4)); /* TACDIR: 0=down */
                if ((timer->tamr & 0x03) == 0x02)
                    elapsed_ticks %= (load + 1);   /* periodic: wrap per period */
                if (count_down) {
                    if (elapsed_ticks >= load) return 0;
                    return (int)(load - elapsed_ticks);
                } else {
                    return (int)(elapsed_ticks % (load + 1));
                }
            }
            return (int)timer->tav;
        }
        case GPTIMER_TBR:
        case GPTIMER_TBV: {
            if (timer->ctl & GPTIMER_CTL_TBEN) {
                int64_t elapsed = timer->cpu->cycles - timer->tb_start_cycle;
                return (int)((timer->tbv + (uint32_t)elapsed) & 0xFFFFFFFF);
            }
            return (int)timer->tbv;
        }
        default: return 0;
    }
}

static void gptimer_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_gptimer_t *timer = (cc2538_gptimer_t *)user_data;
    uint32_t offset = addr - timer->base_addr;

    switch (offset) {
        case GPTIMER_CFG:
            timer->cfg = value;
            break;
        case GPTIMER_TAMR:
            timer->tamr = value;
            break;
        case GPTIMER_TBMR:
            timer->tbmr = value;
            break;
        case GPTIMER_CTL: {
            uint32_t old = timer->ctl;
            timer->ctl = value;
            /* Detect timer A enable */
            if ((value & GPTIMER_CTL_TAEN) && !(old & GPTIMER_CTL_TAEN)) {
                timer->ta_start_cycle = timer->cpu->cycles;
                timer->tav = 0;
                gptimer_ta_schedule(timer);
            } else if (!(value & GPTIMER_CTL_TAEN) && (old & GPTIMER_CTL_TAEN)) {
                arm_cancel_event(timer->cpu, &timer->ta_event);
            }
            /* Detect timer B enable */
            if ((value & GPTIMER_CTL_TBEN) && !(old & GPTIMER_CTL_TBEN)) {
                timer->tb_start_cycle = timer->cpu->cycles;
                timer->tbv = 0;
            }
            break;
        }
        case GPTIMER_IMR:
            timer->imr = value;
            break;
        case GPTIMER_ICR:
            timer->ris &= ~value;
            break;
        case GPTIMER_TAILR:
            timer->tailr = value;
            if (timer->ctl & GPTIMER_CTL_TAEN)
                gptimer_ta_schedule(timer);
            break;
        case GPTIMER_TBILR:
            timer->tbilr = value;
            break;
        case GPTIMER_TAMATCHR:
            timer->tamatchr = value;
            break;
        case GPTIMER_TBMATCHR:
            timer->tbmatchr = value;
            break;
        case GPTIMER_TAPR:
            timer->tapr = value & 0xFF;
            if (timer->ctl & GPTIMER_CTL_TAEN)
                gptimer_ta_schedule(timer);
            break;
        case GPTIMER_TBPR:
            timer->tbpr = value & 0xFF;
            break;
        case GPTIMER_TAV:
            timer->tav = value;
            timer->ta_start_cycle = timer->cpu->cycles;
            break;
        case GPTIMER_TBV:
            timer->tbv = value;
            timer->tb_start_cycle = timer->cpu->cycles;
            break;
    }
}

void cc2538_gptimer_init(cc2538_gptimer_t *timer, arm_cpu_t *cpu,
                         uint32_t base_addr, int irq_a, int irq_b, int index) {
    memset(timer, 0, sizeof(*timer));
    timer->cpu = cpu;
    timer->base_addr = base_addr;
    timer->irq_a = irq_a;
    timer->irq_b = irq_b;
    timer->index = index;

    timer->tailr = 0xFFFFFFFF;
    timer->tbilr = 0xFFFF;

    timer->ta_event.callback  = gptimer_ta_fired;
    timer->ta_event.user_data = timer;
    timer->tb_event.callback  = NULL;   /* Timer B stays poll-only for now */

    arm_register_io(cpu, base_addr, GPTIMER_REG_SIZE,
                    gptimer_read, gptimer_write, timer);
}
