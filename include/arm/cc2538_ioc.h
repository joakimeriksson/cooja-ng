/*
 * CC2538 I/O Controller (pin muxing)
 */
#ifndef CC2538_IOC_H
#define CC2538_IOC_H

#include "arm_cpu.h"

/* IOC base address */
#define IOC_BASE  0x400D4000

/* IOC has 32 pin selectors (PA0-PA7, PB0-PB7, PC0-PC7, PD0-PD7) */
#define IOC_NUM_PINS  32

typedef struct cc2538_ioc {
    arm_cpu_t *cpu;
    uint32_t   pin_sel[IOC_NUM_PINS];    /* PA0_SEL through PD7_SEL */
    uint32_t   pin_over[IOC_NUM_PINS];   /* PA0_OVER through PD7_OVER */
} cc2538_ioc_t;

/* Initialize IOC and register IO region */
void cc2538_ioc_init(cc2538_ioc_t *ioc, arm_cpu_t *cpu);

#endif /* CC2538_IOC_H */
