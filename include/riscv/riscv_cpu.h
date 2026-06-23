/*
 * riscv_cpu — minimal RV32E (rv32e_zicsr_zifencei) interpreter.
 *
 * Built for the nRF54L15 FLPR coprocessor scenario: the core does NOT own
 * memory. It shares the host (Cortex-M33) memory + IO bus via an arm_cpu_t,
 * so SRAM and every memory-mapped peripheral (GRTC, GPIO, the shared
 * counter at 0x2003F000) are the same objects the M33 sees. That shared
 * view is the whole point — see docs/design/riscv-vpr-plan.md.
 *
 * ISA: base RV32I (16 GP regs / RVE), Zicsr (CSR ops), Zifencei (fence.i),
 * machine-mode traps (mtvec/mepc/mcause). No M, no C, no F/D.
 */
#ifndef RISCV_CPU_H
#define RISCV_CPU_H

#include <stdint.h>

struct arm_cpu; /* the FLPR borrows the M33's memory + IO bus */

typedef struct riscv_cpu {
    uint32_t x[32];        /* RV32E uses x0..x15; x16+ kept for safe decode  */
    uint32_t pc;

    /* Machine-mode CSRs (the subset the nrf-vpr startup/trap path touches). */
    uint32_t mtvec, mepc, mcause, mtval, mstatus, mscratch, mie, mip;

    int64_t  cycles;       /* 1 per retired instruction                      */
    int      halted;       /* reserved; core keeps running tight loops today */
    uint32_t trap_pc;      /* PC of the most recent trap (diagnostics)        */

    struct arm_cpu *bus;   /* shared memory + memory-mapped IO                */
} riscv_cpu_t;

/* initpc = INITPC the M33 wrote to the VPR launch register. */
void riscv_cpu_init(riscv_cpu_t *rv, struct arm_cpu *bus, uint32_t initpc);

/* Execute up to n instructions; returns the number actually retired. */
int  riscv_step(riscv_cpu_t *rv, int n);

/* Run until cycles >= target_cycle (the FLPR's slice budget). */
void riscv_step_until(riscv_cpu_t *rv, int64_t target_cycle);

#endif /* RISCV_CPU_H */
