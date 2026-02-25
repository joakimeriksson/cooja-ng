/*
 * CC2538 General Purpose Timer (GPTimer 0-3, each with Timer A + Timer B)
 */
#ifndef CC2538_GPTIMER_H
#define CC2538_GPTIMER_H

#include "arm_cpu.h"

/* GPTimer base addresses */
#define GPTIMER0_BASE  0x40030000
#define GPTIMER1_BASE  0x40031000
#define GPTIMER2_BASE  0x40032000
#define GPTIMER3_BASE  0x40033000

/* GPTimer register offsets */
#define GPTIMER_CFG      0x000
#define GPTIMER_TAMR     0x004
#define GPTIMER_TBMR     0x008
#define GPTIMER_CTL      0x00C
#define GPTIMER_SYNC     0x010
#define GPTIMER_IMR      0x018
#define GPTIMER_RIS      0x01C
#define GPTIMER_MIS      0x020
#define GPTIMER_ICR      0x024
#define GPTIMER_TAILR    0x028
#define GPTIMER_TBILR    0x02C
#define GPTIMER_TAMATCHR 0x030
#define GPTIMER_TBMATCHR 0x034
#define GPTIMER_TAPR     0x038
#define GPTIMER_TBPR     0x03C
#define GPTIMER_TAV      0x050
#define GPTIMER_TBV      0x054
#define GPTIMER_TAR      0x048
#define GPTIMER_TBR      0x04C

/* CTL bits */
#define GPTIMER_CTL_TAEN  (1 << 0)
#define GPTIMER_CTL_TBEN  (1 << 8)

typedef struct cc2538_gptimer {
    arm_cpu_t  *cpu;
    uint32_t    base_addr;
    int         irq_a;       /* IRQ number for Timer A */
    int         irq_b;       /* IRQ number for Timer B */
    int         index;       /* Timer index (0-3) */

    /* Registers */
    uint32_t    cfg;
    uint32_t    tamr;
    uint32_t    tbmr;
    uint32_t    ctl;
    uint32_t    imr;
    uint32_t    ris;
    uint32_t    icr;
    uint32_t    tailr;
    uint32_t    tbilr;
    uint32_t    tamatchr;
    uint32_t    tbmatchr;
    uint32_t    tapr;
    uint32_t    tbpr;
    uint32_t    tav;
    uint32_t    tbv;

    /* Cycle tracking for on-demand counter */
    int64_t     ta_start_cycle;
    int64_t     tb_start_cycle;

    arm_event_t ta_event;
    arm_event_t tb_event;
} cc2538_gptimer_t;

/* Initialize GPTimer and register IO region */
void cc2538_gptimer_init(cc2538_gptimer_t *timer, arm_cpu_t *cpu,
                         uint32_t base_addr, int irq_a, int irq_b, int index);

#endif /* CC2538_GPTIMER_H */
