/*
 * Clock Module — supports both BCS (Basic Clock System) and CS (Clock System).
 *
 * BCS (classic MSP430): models DCO frequency based on DCOCTL/BCSCTL1 registers.
 *   Formula matches Java MSPSim's BasicClockModule:
 *     calcDCOFrq = ((dcoFreq<<5) + dcoMod + (rsel<<8)) * DCO_FACTOR + MIN_DCO_FRQ
 *
 * CS (FR5xxx): DCO frequency from lookup table indexed by DCOFSEL/DCORSEL.
 *   CSCTL0 is password-protected; the high byte must be 0xA5 to enable writes.
 */
#include "msp430_clock.h"
#include "msp430_timer.h"
#include <string.h>

#define MIN_DCO_FRQ  1000
#define ACLK_FRQ     32768

/* ----- Common ----- */

static void notify_timers(msp430_clock_t *clk) {
    for (int i = 0; i < clk->num_timers; i++) {
        msp430_timer_clock_changed(clk->timers[i]);
    }
}

/* ----- BCS (Basic Clock System, F1xx/F2xx; also covers UCS on F5xxx
 *        which exposes the same register layout for our purposes) ----- */

static void bcs_recalculate(msp430_clock_t *clk) {
    int dco_factor = (int)(clk->max_dco_freq - MIN_DCO_FRQ) / 2048;

    int dco_freq = (clk->dcoctl >> 5) & 0x7;   /* DCOCTL bits [7:5] */
    int dco_mod  = clk->dcoctl & 0x1f;          /* DCOCTL bits [4:0] */
    int rsel     = clk->bcsctl1 & 0x7;          /* BCSCTL1 bits [2:0] */

    uint32_t new_dco = (uint32_t)(((dco_freq << 5) + dco_mod + (rsel << 8)) * dco_factor + MIN_DCO_FRQ);

    clk->dco_freq = new_dco;
    clk->smclk_freq = new_dco / (uint32_t)clk->div_smclk;
    clk->aclk_freq = ACLK_FRQ / (uint32_t)clk->div_aclk;
    clk->mclk_freq = clk->smclk_freq;  /* BCS: not separately modeled, MCLK == SMCLK */

    /* Update CPU frequency — recomputes fire_cycle for all ns-based events */
    msp430_cpu_set_frequency(clk->cpu, clk->mclk_freq);
}

static int bcs_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
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

static void bcs_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_clock_t *clk = (msp430_clock_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - clk->base_addr;
    switch (offset) {
    case 0:
        clk->dcoctl = (uint8_t)value;
        bcs_recalculate(clk);
        notify_timers(clk);
        break;
    case 1:
        clk->bcsctl1 = (uint8_t)value;
        clk->div_aclk = 1 << ((value >> 4) & 3);
        bcs_recalculate(clk);
        notify_timers(clk);
        break;
    case 2:
        clk->bcsctl2 = (uint8_t)value;
        /* Match MSPSim: divSMclk = 1 << ((data >> 2) & 3) */
        clk->div_smclk = 1 << ((value >> 2) & 3);
        bcs_recalculate(clk);
        notify_timers(clk);
        break;
    }
}

static void bcs_init(msp430_clock_t *clk) {
    clk->div_aclk = 1;
    clk->div_smclk = 1;
    clk->div_mclk = 1;

    /* Default register values after reset (MSP430F1611 datasheet) */
    clk->dcoctl = 0x60;    /* DCO=3, MOD=0 */
    clk->bcsctl1 = 0x84;   /* XT2OFF=1, RSEL=4 */
    clk->bcsctl2 = 0x00;

    bcs_recalculate(clk);

    /* Register IO for DCOCTL (base), BCSCTL1 (base+1), BCSCTL2 (base+2) */
    msp430_register_io(clk->cpu, clk->base_addr, 3, bcs_read, bcs_write, clk);
}

/* ----- CS (Clock System, FR5xxx) ----- */

/* DCO frequency lookup tables (Hz), indexed by DCOFSEL.
 * Values per SLAS704G §5.10 (FR5969 datasheet, "DCO Frequency"). */
static const uint32_t dco_freq_low[8] = {
    1000000, 2670000, 3500000, 4000000,
    5330000, 7000000, 8000000, 8000000
};

static const uint32_t dco_freq_high[8] = {
    1000000,  5330000,  7000000,  8000000,
    16000000, 21000000, 24000000, 24000000
};

/* Fixed clock frequencies */
#define VLOCLK_FRQ    10000
#define LFXTCLK_FRQ   32768
#define MODCLK_FRQ    5000000
#define LFMODCLK_FRQ  39062

/* CSCTL register offsets (word addresses, so offset/2 indexes csctl[]) */
#define CS_CSCTL0  0   /* Password */
#define CS_CSCTL1  1   /* DCO select */
#define CS_CSCTL2  2   /* Clock source select */
#define CS_CSCTL3  3   /* Clock dividers */
#define CS_CSCTL4  4   /* LFXT/HFXT control + drive */
#define CS_CSCTL5  5   /* Status / fault flags */
#define CS_CSCTL6  6   /* Reserved on FR5969 */

#define CS_KEY      0xA500
#define CS_KEY_MASK 0xFF00

/* CSCTL4 control bits */
#define CS_LFXTOFF  0x0001  /* LFXT disabled when set */
#define CS_HFXTOFF  0x0100  /* HFXT disabled when set */
#define CS_VLOOFF   0x0008  /* VLO disabled when set */

/* Resolve a clock source select value to a frequency, honouring the
 * spec's fail-safe fallbacks (SLAU367P §3.3.3 Table 3-6):
 *   LFXT off -> VLO; HFXT off -> DCO. */
static uint32_t cs_get_source_freq(const msp430_clock_t *clk, int sel) {
    uint16_t csctl4 = clk->csctl[CS_CSCTL4];
    bool lfxt_off = (csctl4 & CS_LFXTOFF) != 0;
    bool hfxt_off = (csctl4 & CS_HFXTOFF) != 0;
    switch (sel) {
    case 0: return lfxt_off ? VLOCLK_FRQ : LFXTCLK_FRQ;
    case 1: return VLOCLK_FRQ;
    case 2: return LFMODCLK_FRQ;
    case 3: return clk->dco_freq;
    case 4: return MODCLK_FRQ;
    case 5: return hfxt_off ? clk->dco_freq : clk->dco_freq;  /* HFXT not modeled; use DCO */
    default: return clk->dco_freq;
    }
}

static void cs_recalculate(msp430_clock_t *clk) {
    uint16_t csctl1 = clk->csctl[CS_CSCTL1];
    uint16_t csctl2 = clk->csctl[CS_CSCTL2];
    uint16_t csctl3 = clk->csctl[CS_CSCTL3];

    /* DCO frequency from lookup table */
    int dcofsel = (csctl1 >> 1) & 0x7;
    int dcorsel = (csctl1 >> 6) & 0x1;
    clk->dco_freq = dcorsel ? dco_freq_high[dcofsel] : dco_freq_low[dcofsel];

    /* MCLK source and divider (CPU clock) */
    int selm = csctl2 & 0x7;
    int divm = csctl3 & 0x7;
    clk->div_mclk = 1 << divm;
    clk->mclk_freq = cs_get_source_freq(clk, selm) / (uint32_t)clk->div_mclk;

    /* SMCLK source and divider */
    int sels = (csctl2 >> 4) & 0x7;
    int divs = (csctl3 >> 4) & 0x7;
    clk->div_smclk = 1 << divs;
    clk->smclk_freq = cs_get_source_freq(clk, sels) / (uint32_t)clk->div_smclk;

    /* ACLK source and divider */
    int sela = (csctl2 >> 8) & 0x7;
    int diva = (csctl3 >> 8) & 0x7;
    clk->div_aclk = 1 << diva;
    clk->aclk_freq = cs_get_source_freq(clk, sela) / (uint32_t)clk->div_aclk;

    /* CPU runs at MCLK */
    msp430_cpu_set_frequency(clk->cpu, clk->mclk_freq);
}

static int cs_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_clock_t *clk = (msp430_clock_t *)user_data;
    (void)word; (void)cycles;

    uint32_t offset = addr - clk->base_addr;
    int reg = offset / 2;
    if (reg >= 0 && reg < 7) {
        if (reg == CS_CSCTL0) {
            /* Return key status: 0x96xx when locked, 0xA5xx when unlocked */
            return clk->cs_unlocked ? (0xA500 | (clk->csctl[0] & 0xFF))
                                    : (0x9600 | (clk->csctl[0] & 0xFF));
        }
        return clk->csctl[reg];
    }
    return 0;
}

static void cs_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_clock_t *clk = (msp430_clock_t *)user_data;
    (void)cycles;

    uint32_t offset = addr - clk->base_addr;
    int reg = offset / 2;

    if (reg == CS_CSCTL0) {
        /* CSCTL0 password handling.  Writing 0xA5 to the high byte (or
         * 0xA5xx as a word) unlocks the CS for register writes; any other
         * write locks it again.  Note: SLAU367P §3.3.1 documents PUC on
         * non-A5 writes, but in practice TI driverlib re-locks via
         * `CSCTL0_H = 0` and real silicon merely re-locks (no PUC), so we
         * model it that way for firmware compatibility. */
        if (word) {
            clk->cs_unlocked = ((value & CS_KEY_MASK) == CS_KEY);
            clk->csctl[0] = (uint16_t)value;
        } else if (offset & 1) {
            clk->cs_unlocked = ((uint8_t)value == (CS_KEY >> 8));
            clk->csctl[0] = (clk->csctl[0] & 0x00FF) | ((value & 0xFF) << 8);
        } else {
            clk->csctl[0] = (clk->csctl[0] & 0xFF00) | (value & 0xFF);
        }
        return;
    }

    /* All other registers require unlock */
    if (!clk->cs_unlocked) return;

    if (reg >= 1 && reg < 7) {
        if (word) {
            clk->csctl[reg] = (uint16_t)value;
        } else if (offset & 1) {
            clk->csctl[reg] = (clk->csctl[reg] & 0x00FF) | ((value & 0xFF) << 8);
        } else {
            clk->csctl[reg] = (clk->csctl[reg] & 0xFF00) | (value & 0xFF);
        }
        if (reg <= CS_CSCTL3) {
            cs_recalculate(clk);
            notify_timers(clk);
        }
    }
}

static void cs_init(msp430_clock_t *clk) {
    /* Reset values per SLAU367P §3.3 (FR58xx/FR59xx Family User's Guide).
     * After POR: DCO = 8 MHz, MCLK = SMCLK = DCO/8 = 1 MHz, ACLK = VLO. */
    clk->csctl[CS_CSCTL0] = 0x9600;            /* Locked (status reads 0x96xx) */
    clk->csctl[CS_CSCTL1] = 0x000C;            /* DCORSEL=0, DCOFSEL=6 -> 8 MHz */
    clk->csctl[CS_CSCTL2] = 0x0033;            /* SELA=0 LFXT, SELS=3 DCO, SELM=3 DCO */
    clk->csctl[CS_CSCTL3] = 0x0033;            /* DIVS=DIVM=3 (/8); DIVA=0 (/1) */
    clk->csctl[CS_CSCTL4] = 0xCDC9;            /* LFXTOFF=1, HFXTOFF=1, drive bits */
    clk->csctl[CS_CSCTL5] = 0x00C5;            /* ENSTFCNT1/2=1, fault flags */
    clk->csctl[CS_CSCTL6] = 0x0000;

    cs_recalculate(clk);

    /* Register IO for CSCTL0-CSCTL6 (16-byte register file per SLAS704G Table 6-26) */
    msp430_register_io(clk->cpu, clk->base_addr, 16, cs_read, cs_write, clk);
}

/* ----- Public API ----- */

void msp430_clock_init(msp430_clock_t *clk, msp430_cpu_t *cpu,
                        uint32_t base_addr, uint32_t max_dco_freq,
                        int clock_type) {
    memset(clk, 0, sizeof(*clk));
    clk->cpu = cpu;
    clk->base_addr = base_addr;
    clk->max_dco_freq = max_dco_freq;
    clk->clock_type = clock_type;

    if (clock_type == MSP430_CLOCK_CS) {
        cs_init(clk);
    } else {
        bcs_init(clk);
    }
}

uint32_t msp430_clock_get_aclk_freq(const msp430_clock_t *clk) {
    return clk->aclk_freq;
}

uint32_t msp430_clock_get_smclk_freq(const msp430_clock_t *clk) {
    return clk->smclk_freq;
}

void msp430_clock_add_timer(msp430_clock_t *clk, struct msp430_timer *timer) {
    if (clk->num_timers < 4) {
        clk->timers[clk->num_timers++] = timer;
    }
}
