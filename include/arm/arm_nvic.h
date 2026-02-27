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

typedef struct arm_nvic {
    arm_cpu_t *cpu;

    /* Enable bits: 1 = IRQ enabled (8 x 32-bit words = 256 IRQs) */
    uint32_t  iser[8];

    /* Pending bits: 1 = IRQ pending */
    uint32_t  ispr[8];

    /* Active bits: 1 = IRQ currently being serviced */
    uint32_t  iabr[8];

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
    int       pending_exception;  /* highest priority pending, -1 = none */

    /* Flag: set when there may be serviceable pending interrupts.
     * Checked in the main execution loop to avoid scanning ISPR on every insn. */
    bool      has_pending;
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

#endif /* ARM_NVIC_H */
