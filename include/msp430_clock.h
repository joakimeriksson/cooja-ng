/*
 * Basic Clock Module — models DCO frequency based on DCOCTL/BCSCTL1 registers.
 * DCO frequency formula matches Java MSPSim's BasicClockModule.
 */
#ifndef MSP430_CLOCK_H
#define MSP430_CLOCK_H

#include "msp430_cpu.h"

/* Forward declaration for timer update callback */
struct msp430_timer;

typedef struct msp430_clock {
    msp430_cpu_t *cpu;
    uint32_t      base_addr;    /* DCOCTL address (0x56 for f1611) */
    uint8_t       dcoctl;       /* DCOCTL register */
    uint8_t       bcsctl1;      /* BCSCTL1 register */
    uint8_t       bcsctl2;      /* BCSCTL2 register */
    uint32_t      max_dco_freq; /* MAX_DCO_FRQ (4915200 for f1611) */
    uint32_t      dco_freq;     /* Current calculated DCO frequency */
    uint32_t      smclk_freq;   /* Current SMCLK frequency */
    int           div_aclk;     /* ACLK divider (1, 2, 4, 8) */
    int           div_smclk;    /* SMCLK divider (1, 2, 4, 8) */

    /* Timer update callbacks */
    struct msp430_timer *timers[4];
    int num_timers;
} msp430_clock_t;

void     msp430_clock_init(msp430_clock_t *clk, msp430_cpu_t *cpu,
                            uint32_t base_addr, uint32_t max_dco_freq);
uint32_t msp430_clock_get_aclk_freq(const msp430_clock_t *clk);
uint32_t msp430_clock_get_smclk_freq(const msp430_clock_t *clk);

/* Register a timer to be notified when clock frequencies change */
void     msp430_clock_add_timer(msp430_clock_t *clk, struct msp430_timer *timer);

#endif /* MSP430_CLOCK_H */
