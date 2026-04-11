/*
 * MSP430 Timer A/B peripheral — on-demand counter with event-driven interrupts.
 *
 * Design matches Java MSPSim: counter is not incremented every cycle.
 * Instead, the current value is computed from elapsed CPU cycles when needed.
 * CCR compare events are scheduled in the CPU event queue.
 */
#ifndef MSP430_TIMER_H
#define MSP430_TIMER_H

#include "msp430_cpu.h"

/* Forward declaration */
struct msp430_clock;

/* Timer CTL bit fields */
#define TIMER_TASSEL_MASK  0x0300  /* Clock source select */
#define TIMER_TASSEL_SHIFT 8
#define TIMER_ID_MASK      0x00C0  /* Input divider */
#define TIMER_ID_SHIFT     6
#define TIMER_MC_MASK      0x0030  /* Mode control */
#define TIMER_MC_SHIFT     4
#define TIMER_TCLR         0x0004  /* Timer clear */
#define TIMER_TIE          0x0002  /* Timer overflow interrupt enable */
#define TIMER_TIFG         0x0001  /* Timer overflow interrupt flag */

/* Mode control values */
#define TIMER_MC_STOP      0
#define TIMER_MC_UP        1
#define TIMER_MC_CONT      2
#define TIMER_MC_UPDOWN    3

/* Clock sources */
#define TIMER_SRC_TCLK     0
#define TIMER_SRC_ACLK     1
#define TIMER_SRC_SMCLK    2
#define TIMER_SRC_INCLK    3

/* CCTLn bit fields */
/* CCTLn bit fields */
#define TIMER_CM_MASK      0xC000  /* Capture mode */
#define TIMER_CM_SHIFT     14
#define TIMER_CCIS_MASK    0x3000  /* Capture input select */
#define TIMER_CCIS_SHIFT   12
#define TIMER_SCS          0x0800  /* Synchronize capture source */
#define TIMER_CAP          0x0100  /* Capture mode (0=compare, 1=capture) */
#define TIMER_CCIE         0x0010  /* Capture/compare interrupt enable */
#define TIMER_CCIFG        0x0001  /* Capture/compare interrupt flag */
#define TIMER_COV          0x0002  /* Capture overflow */

#define TIMER_MAX_CCR      7

typedef struct msp430_timer {
    msp430_cpu_t         *cpu;
    struct msp430_clock  *clock;
    const char           *name;

    /* Configuration */
    uint32_t  base_addr;       /* 0x160 for Timer A, 0x180 for Timer B */
    uint32_t  iv_addr;         /* 0x12E for TAIV, 0x11E for TBIV */
    int       num_ccr;         /* 3 for Timer A, 7 for Timer B */
    int       ccr0_vector;     /* Interrupt vector for CCR0 (6 for TA, 13 for TB) */
    int       ccr1_vector;     /* Shared vector for CCR1+ and overflow (5 for TA, 12 for TB) */

    /* State */
    uint16_t  ctl;             /* TxCTL: mode, clock source, divider */
    uint16_t  counter;         /* TxR: last synced counter value */
    uint16_t  ccr[TIMER_MAX_CCR];   /* TxCCRn: compare/capture values */
    uint16_t  cctl[TIMER_MAX_CCR];  /* TxCCTLn: compare/capture control */

    /* Timing (matches MSPSim's Timer.java fields) */
    int64_t   counter_start;   /* CPU cycle at last resetCounter (MSPSim: counterStart) */
    int64_t   counter_acc;     /* Counter value at last reset (MSPSim: counterAcc) */
    int       counter_passed;  /* Sub-tick cycle remainder (MSPSim: counterPassed) */
    double    cycles_per_tick; /* CPU cycles per timer tick (MSPSim: cyclesMultiplicator) */
    int       input_divider;   /* 1, 2, 4, or 8 */
    int       clock_source;    /* TIMER_SRC_xxx */
    int       mode;            /* TIMER_MC_xxx */

    /* TIV state */
    int       last_tiv;        /* Value to return on next TAIV/TBIV read */

    /* Capture state (matches Java MSPSim's expCompare/expCapInterval approach) */
    int       exp_cap_interval[TIMER_MAX_CCR];  /* Expected timer ticks between captures */
    uint16_t  exp_compare[TIMER_MAX_CCR];       /* Expected CCR value at next capture */

    /* Events */
    msp430_event_t ccr_event[TIMER_MAX_CCR];
    msp430_event_t overflow_event;
} msp430_timer_t;

void msp430_timer_init(msp430_timer_t *timer, msp430_cpu_t *cpu,
                        struct msp430_clock *clock,
                        const char *name, uint32_t base_addr, uint32_t iv_addr,
                        int num_ccr, int ccr0_vector, int ccr1_vector);

/* Called by clock module when frequencies change */
void msp430_timer_clock_changed(msp430_timer_t *timer);

/* Trigger external capture input on a CCR channel.
 * Used for SFD pin → Timer B CCR1 capture on Z1/Sky. */
void msp430_timer_capture_input(msp430_timer_t *timer, int ccr_idx, bool value);

#endif /* MSP430_TIMER_H */
