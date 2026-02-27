/*
 * CC2538 Sleep Timer (part of SMWDTHROSC module at 0x400D5000)
 *
 * 32-bit up-counter running at 32,768 Hz with compare interrupt.
 * Used by Contiki-NG as the primary clock source for the event scheduler.
 */
#ifndef CC2538_SLEEPTIMER_H
#define CC2538_SLEEPTIMER_H

#include "arm_cpu.h"
#include "arm_nvic.h"

/* SMWDTHROSC register offsets */
#define SMWDTHROSC_WDCTL   0x00  /* Watchdog Control */
#define SMWDTHROSC_ST0     0x40  /* Sleep Timer 0 (bits 7:0) */
#define SMWDTHROSC_ST1     0x44  /* Sleep Timer 1 (bits 15:8) */
#define SMWDTHROSC_ST2     0x48  /* Sleep Timer 2 (bits 23:16) */
#define SMWDTHROSC_ST3     0x4C  /* Sleep Timer 3 (bits 31:24) */
#define SMWDTHROSC_STLOAD  0x50  /* Sleep Timer Load Status */
#define SMWDTHROSC_STCC    0x54  /* Sleep Timer Capture Control */
#define SMWDTHROSC_STCS    0x58  /* Sleep Timer Capture Status */
#define SMWDTHROSC_STCV0   0x5C  /* Sleep Timer Capture Value 0 */
#define SMWDTHROSC_STCV1   0x60  /* Sleep Timer Capture Value 1 */
#define SMWDTHROSC_STCV2   0x64  /* Sleep Timer Capture Value 2 */
#define SMWDTHROSC_STCV3   0x68  /* Sleep Timer Capture Value 3 */

/* Sleep Timer frequency */
#define SLEEPTIMER_FREQ_HZ  32768

typedef struct cc2538_sleeptimer {
    arm_cpu_t  *cpu;
    arm_nvic_t *nvic;
    int         irq;              /* IRQ number (32 for SM Timer) */

    /* Counter state (on-demand, computed from ns) */
    uint32_t    base_count;       /* counter value at base_ns */
    int64_t     base_ns;          /* sim_time_ns when base_count was set */

    /* Compare register (written byte-by-byte via ST0-ST3) */
    uint32_t    compare;          /* latched compare value */
    uint8_t     compare_staging[4]; /* staging area for byte writes */

    /* Read latch (ST0 read latches ST1-ST3) */
    uint32_t    latched_count;

    /* Event for compare match */
    arm_event_t compare_event;
} cc2538_sleeptimer_t;

/* Initialize Sleep Timer and register IO at 0x400D5000 */
void cc2538_sleeptimer_init(cc2538_sleeptimer_t *st, arm_cpu_t *cpu,
                            arm_nvic_t *nvic, int irq);

#endif /* CC2538_SLEEPTIMER_H */
