/*
 * ARM Cortex-M3 SysTick 24-bit countdown timer
 */
#ifndef ARM_SYSTICK_H
#define ARM_SYSTICK_H

#include "arm_cpu.h"
#include "arm_nvic.h"

/* SysTick register offsets (relative to 0xE000E010) */
#define SYST_CSR    0x00  /* Control and Status Register */
#define SYST_RVR    0x04  /* Reload Value Register */
#define SYST_CVR    0x08  /* Current Value Register */
#define SYST_CALIB  0x0C  /* Calibration Value Register */

/* CSR bits */
#define SYST_CSR_ENABLE     (1 << 0)
#define SYST_CSR_TICKINT    (1 << 1)
#define SYST_CSR_CLKSOURCE  (1 << 2)
#define SYST_CSR_COUNTFLAG  (1 << 16)

typedef struct arm_systick {
    arm_cpu_t  *cpu;
    arm_nvic_t *nvic;

    uint32_t    csr;        /* Control and Status */
    uint32_t    rvr;        /* Reload Value (24-bit) */
    uint32_t    cvr;        /* Current Value (24-bit) */
    int64_t     last_cycle; /* Cycle count when CVR was last updated */

    arm_event_t tick_event; /* Scheduled tick event */
} arm_systick_t;

/* Initialize SysTick (registers IO at 0xE000E010) */
void arm_systick_init(arm_systick_t *st, arm_cpu_t *cpu, arm_nvic_t *nvic);

/* Recalculate and schedule next tick event */
void arm_systick_update(arm_systick_t *st);

/* Debug: SysTick fire counter */
int arm_systick_get_fire_count(void);
void arm_systick_reset_fire_count(void);

#endif /* ARM_SYSTICK_H */
