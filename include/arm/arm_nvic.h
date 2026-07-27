/*
 * ARM Cortex-M3 Nested Vectored Interrupt Controller
 */
#ifndef ARM_NVIC_H
#define ARM_NVIC_H

#include "arm_cpu.h"

/* NVIC supports up to 240 external IRQs (CC2538 uses ~179) */
#define NVIC_MAX_IRQ 240

/* NVIC register offsets within System Control Space (0xE000E000) */
#define NVIC_ISER_BASE  0x100   /* Interrupt Set-Enable Registers */
#define NVIC_ICER_BASE  0x180   /* Interrupt Clear-Enable Registers */
#define NVIC_ISPR_BASE  0x200   /* Interrupt Set-Pending Registers */
#define NVIC_ICPR_BASE  0x280   /* Interrupt Clear-Pending Registers */
#define NVIC_IABR_BASE  0x300   /* Interrupt Active Bit Registers */
#define NVIC_IPR_BASE   0x400   /* Interrupt Priority Registers */
#define NVIC_ITNS_BASE  0x380   /* Interrupt Target Non-secure (ARMv8-M) */

/* System Control Block offsets */
#define SCB_ICSR    0xD04  /* Interrupt Control and State Register */
#define SCB_VTOR    0xD08  /* Vector Table Offset Register */
#define SCB_AIRCR   0xD0C  /* Application Interrupt and Reset Control */
#define SCB_SCR     0xD10  /* System Control Register */
#define SCB_CCR     0xD14  /* Configuration and Control Register */
#define SCB_SHPR1   0xD18  /* System Handler Priority Register 1 */
#define SCB_SHPR2   0xD1C  /* System Handler Priority Register 2 */
#define SCB_SHPR3   0xD20  /* System Handler Priority Register 3 */
#define SCB_SHCSR   0xD24  /* System Handler Control and State Register */
#define SCB_CPUID   0xD00  /* CPUID Base Register */

/* ARMv8-M Security Attribution Unit (SAU) register offsets within the SCS.
 * Secure-only (RAZ/WI from Non-secure). See arm_trustzone.c. */
#define SAU_CTRL    0xDD0  /* SAU Control Register */
#define SAU_TYPE    0xDD4  /* SAU Type Register (SREGION count, read-only) */
#define SAU_RNR     0xDD8  /* SAU Region Number Register */
#define SAU_RBAR    0xDDC  /* SAU Region Base Address Register */
#define SAU_RLAR    0xDE0  /* SAU Region Limit Address Register */

typedef struct arm_nvic {
    arm_cpu_t *cpu;

    /* Enable bits: 1 = IRQ enabled (8 x 32-bit words = 256 IRQs) */
    uint32_t  iser[8];

    /* Pending bits: 1 = IRQ pending */
    uint32_t  ispr[8];

    /* Active bits: 1 = IRQ currently being serviced */
    uint32_t  iabr[8];

    /* ARMv8-M target-security: bit set => IRQ targets Non-secure. Reset 0
     * (all interrupts Secure). Secure-only registers. */
    uint32_t  itns[8];

    /* Priority: 8-bit priority per IRQ (only upper bits used) */
    uint8_t   ipr[NVIC_MAX_IRQ];

    /* System handler priorities (exceptions 4-15) */
    uint8_t   shpr[12];  /* indices 0-11 map to exceptions 4-15 */

    /* SCB registers */
    uint32_t  vtor;       /* Vector Table Offset */
    uint32_t  aircr;      /* AIRCR */
    uint32_t  scr;        /* System Control Register */
    uint32_t  ccr;        /* Configuration and Control Register */
    uint32_t  shcsr;      /* System Handler Control and State */

    /* Currently active exception (0 = Thread mode) */
    int       active_exception;
    /* Pending system exceptions (PendSV/SysTick/...), bit N = exception N.
     * A bitmask, not a single slot: PendSV pended for a context switch and a
     * SysTick firing before it is taken must both stay pending — the old
     * single int let the second SET silently overwrite the first, dropping
     * the context switch. */
    uint32_t  sys_pending;

    /* Flag: set when there may be serviceable pending interrupts.
     * Checked in the main execution loop to avoid scanning ISPR on every insn. */
    bool      has_pending;

    /* Memo of the last full pending scan: the winning (exception, priority)
     * pair, valid until any pending/enable/priority state mutates.  While an
     * exception stays pending but MASKED (BASEPRI critical section, or the
     * running handler can't be preempted — TSCH spends whole slots there),
     * arm_step re-checks every instruction; without this memo each re-check
     * re-scanned all 8 ISPR words + priorities (~14% of simulation time). */
    bool      scan_valid;
    int       scan_exc;    /* -1 = nothing pending+enabled */
    int       scan_prio;
} arm_nvic_t;

/* Initialize NVIC and register IO regions */
void arm_nvic_init(arm_nvic_t *nvic, arm_cpu_t *cpu);

/* Set an IRQ pending (irq_num = 0-239, maps to exception 16+irq_num) */
void arm_nvic_set_pending(arm_nvic_t *nvic, int irq_num);

/* Clear an IRQ pending */
void arm_nvic_clear_pending(arm_nvic_t *nvic, int irq_num);

/* Check if any exception should preempt current execution */
void arm_nvic_check_pending(arm_nvic_t *nvic);

/* Get the vector address for an exception */
uint32_t arm_nvic_get_vector(arm_nvic_t *nvic, int exception_num);

/* Get priority for an exception number */
int arm_nvic_get_priority(arm_nvic_t *nvic, int exception_num);

/* ARMv8-M: does `exception_num` target the Secure state? External IRQs use
 * NVIC_ITNS; SecureFault and (for now) other system exceptions are Secure. */
bool arm_nvic_targets_secure(const arm_nvic_t *nvic, int exception_num);

#endif /* ARM_NVIC_H */
