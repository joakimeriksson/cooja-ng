/*
 * CC2538 General Purpose Timer
 */
#include "cc2538_gptimer.h"
#include "arm_nvic.h"
#include <string.h>

#define GPTIMER_REG_SIZE  0x1000

static int gptimer_read(void *user_data, uint32_t addr) {
    cc2538_gptimer_t *timer = (cc2538_gptimer_t *)user_data;
    uint32_t offset = addr - timer->base_addr;

    switch (offset) {
        case GPTIMER_CFG:      return (int)timer->cfg;
        case GPTIMER_TAMR:     return (int)timer->tamr;
        case GPTIMER_TBMR:     return (int)timer->tbmr;
        case GPTIMER_CTL:      return (int)timer->ctl;
        case GPTIMER_IMR:      return (int)timer->imr;
        case GPTIMER_RIS:      return (int)timer->ris;
        case GPTIMER_MIS:      return (int)(timer->ris & timer->imr);
        case GPTIMER_ICR:      return 0;
        case GPTIMER_TAILR:    return (int)timer->tailr;
        case GPTIMER_TBILR:    return (int)timer->tbilr;
        case GPTIMER_TAMATCHR: return (int)timer->tamatchr;
        case GPTIMER_TBMATCHR: return (int)timer->tbmatchr;
        case GPTIMER_TAPR:     return (int)timer->tapr;
        case GPTIMER_TBPR:     return (int)timer->tbpr;
        case GPTIMER_TAR:
        case GPTIMER_TAV: {
            /* On-demand counter value */
            if (timer->ctl & GPTIMER_CTL_TAEN) {
                int64_t elapsed = timer->cpu->cycles - timer->ta_start_cycle;
                return (int)((timer->tav + (uint32_t)elapsed) & 0xFFFFFFFF);
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

    timer->ta_event.callback = NULL;
    timer->tb_event.callback = NULL;

    arm_register_io(cpu, base_addr, GPTIMER_REG_SIZE,
                    gptimer_read, gptimer_write, timer);
}
