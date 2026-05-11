/*
 * Clock Module — supports both BCS (Basic Clock System, classic MSP430)
 * and CS (Clock System, FR5xxx).
 *
 * BCS DCO frequency formula matches Java MSPSim's BasicClockModule.
 * CS uses a lookup table indexed by DCOFSEL/DCORSEL.
 */
#ifndef MSP430_CLOCK_H
#define MSP430_CLOCK_H

#include "msp430_cpu.h"

/* Forward declaration for timer update callback */
struct msp430_timer;

/* Clock module types */
#define MSP430_CLOCK_BCS  0   /* Basic Clock System (F1xx/F2xx, also UCS on F5xxx) */
#define MSP430_CLOCK_CS   1   /* Clock System (FR5xxx) */

typedef struct msp430_clock {
    msp430_cpu_t *cpu;
    uint32_t      base_addr;
    int           clock_type;    /* MSP430_CLOCK_BCS or MSP430_CLOCK_CS */
    uint32_t      max_dco_freq;
    uint32_t      dco_freq;      /* Current calculated DCO frequency */
    uint32_t      mclk_freq;     /* Current MCLK frequency (drives the CPU) */
    uint32_t      smclk_freq;    /* Current SMCLK frequency */
    uint32_t      aclk_freq;     /* Current ACLK frequency */
    int           div_aclk;      /* ACLK divider (1, 2, 4, 8, ...) */
    int           div_smclk;     /* SMCLK divider (1, 2, 4, 8, ...) */
    int           div_mclk;      /* MCLK divider (CS only; BCS keeps MCLK = SMCLK) */

    /* BCS registers */
    uint8_t       dcoctl;
    uint8_t       bcsctl1;
    uint8_t       bcsctl2;

    /* CS registers (FR5xxx) */
    uint16_t      csctl[7];      /* CSCTL0-CSCTL6 */
    bool          cs_unlocked;   /* Password unlock state */

    /* Timer update callbacks */
    struct msp430_timer *timers[4];
    int num_timers;
} msp430_clock_t;

void     msp430_clock_init(msp430_clock_t *clk, msp430_cpu_t *cpu,
                            uint32_t base_addr, uint32_t max_dco_freq,
                            int clock_type);
uint32_t msp430_clock_get_aclk_freq(const msp430_clock_t *clk);
uint32_t msp430_clock_get_smclk_freq(const msp430_clock_t *clk);

/* Register a timer to be notified when clock frequencies change */
void     msp430_clock_add_timer(msp430_clock_t *clk, struct msp430_timer *timer);

#endif /* MSP430_CLOCK_H */
