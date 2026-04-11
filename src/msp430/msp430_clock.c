/*
 * Basic Clock Module — models DCO frequency based on DCOCTL/BCSCTL1 registers.
 * Formula matches Java MSPSim's BasicClockModule:
 *   calcDCOFrq = ((dcoFreq<<5) + dcoMod + (rsel<<8)) * DCO_FACTOR + MIN_DCO_FRQ
 */
#include "msp430_clock.h"
#include "msp430_timer.h"
#include <string.h>

#define MIN_DCO_FRQ  1000
#define ACLK_FRQ     32768

static void notify_timers(msp430_clock_t *clk) {
    for (int i = 0; i < clk->num_timers; i++) {
        msp430_timer_clock_changed(clk->timers[i]);
    }
}

static void recalculate_dco(msp430_clock_t *clk) {
    int dco_factor = (int)(clk->max_dco_freq - MIN_DCO_FRQ) / 2048;

    int dco_freq = (clk->dcoctl >> 5) & 0x7;   /* DCOCTL bits [7:5] */
    int dco_mod  = clk->dcoctl & 0x1f;          /* DCOCTL bits [4:0] */
    int rsel     = clk->bcsctl1 & 0x7;          /* BCSCTL1 bits [2:0] */

    uint32_t new_dco = (uint32_t)(((dco_freq << 5) + dco_mod + (rsel << 8)) * dco_factor + MIN_DCO_FRQ);
    uint32_t new_smclk = new_dco / (uint32_t)clk->div_smclk;

    clk->dco_freq = new_dco;
    clk->smclk_freq = new_smclk;

    /* Update CPU frequency — recomputes fire_cycle for all ns-based events */
    msp430_cpu_set_frequency(clk->cpu, new_smclk);
}

static int clock_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_clock_t *clk = (msp430_clock_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - clk->base_addr;
    switch (offset) {
    case 0: return clk->dcoctl;
    case 1: return clk->bcsctl1;
    case 2: return clk->bcsctl2;
    default: return 0;
    }
}

static void clock_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_clock_t *clk = (msp430_clock_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - clk->base_addr;
    switch (offset) {
    case 0:
        clk->dcoctl = (uint8_t)value;
        recalculate_dco(clk);
        notify_timers(clk);
        break;
    case 1:
        clk->bcsctl1 = (uint8_t)value;
        clk->div_aclk = 1 << ((value >> 4) & 3);
        recalculate_dco(clk);
        notify_timers(clk);
        break;
    case 2:
        clk->bcsctl2 = (uint8_t)value;
        /* Match MSPSim: divSMclk = 1 << ((data >> 2) & 3) */
        clk->div_smclk = 1 << ((value >> 2) & 3);
        recalculate_dco(clk);
        notify_timers(clk);
        break;
    }
}

void msp430_clock_init(msp430_clock_t *clk, msp430_cpu_t *cpu,
                        uint32_t base_addr, uint32_t max_dco_freq) {
    memset(clk, 0, sizeof(*clk));
    clk->cpu = cpu;
    clk->base_addr = base_addr;
    clk->max_dco_freq = max_dco_freq;
    clk->div_aclk = 1;
    clk->div_smclk = 1;

    /* Default register values after reset (MSP430F1611 datasheet) */
    clk->dcoctl = 0x60;    /* DCO=3, MOD=0 */
    clk->bcsctl1 = 0x84;   /* XT2OFF=1, RSEL=4 */
    clk->bcsctl2 = 0x00;

    /* Calculate initial DCO frequency */
    recalculate_dco(clk);

    /* Register IO for DCOCTL (base), BCSCTL1 (base+1), BCSCTL2 (base+2) */
    msp430_register_io(cpu, base_addr, 3, clock_read, clock_write, clk);
}

uint32_t msp430_clock_get_aclk_freq(const msp430_clock_t *clk) {
    return ACLK_FRQ / (uint32_t)clk->div_aclk;
}

uint32_t msp430_clock_get_smclk_freq(const msp430_clock_t *clk) {
    return clk->smclk_freq;
}

void msp430_clock_add_timer(msp430_clock_t *clk, struct msp430_timer *timer) {
    if (clk->num_timers < 4) {
        clk->timers[clk->num_timers++] = timer;
    }
}
