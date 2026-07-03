/*
 * nrf54l_vpr — bridge between the nRF54L15 SoC model and the RV32E core.
 *
 * Provides the real nrf54l_vpr_launch() (the SoC model carries only a
 * declaration): on the M33's CPURUN 0->1 edge it instantiates an RV32E
 * core over the shared bus and wires it into the ARM execute tick via the
 * SoC-agnostic coproc hook on arm_cpu_t. See docs/design/riscv-vpr-plan.md.
 */
#include "riscv_cpu.h"
#include "nrf54l15_soc.h"   /* nrf54l_vpr_state_t */
#include "arm_cpu.h"
#include "arm_platform.h"   /* arm_platform_t */
#include <stdlib.h>
#include <stdio.h>

/* Co-step: advance the FLPR by the same cycle budget the M33 just spent,
 * so the two cores share one wall-clock timeline. Called once per ARM
 * execute slice from arm_elf_mote_tick(). */
static void flpr_costep(void *coproc, int64_t delta_cycles) {
    riscv_cpu_t *rv = (riscv_cpu_t *)coproc;
    if (!rv || delta_cycles <= 0) return;
    riscv_step_until(rv, rv->cycles + delta_cycles);
}

/* GRTC (or any SoC peripheral in the FLPR's IRQ group) raises an interrupt:
 * latch the pending bit and wake the hart from WFI. The firmware's mie /
 * mstatus.MIE gate whether it is actually taken. */
static void flpr_raise_irq(void *coproc, uint32_t mip_bit) {
    riscv_cpu_t *rv = (riscv_cpu_t *)coproc;
    if (!rv) return;
    rv->mip   |= mip_bit;
    rv->halted = 0;
}

void nrf54l_vpr_launch(nrf54l_vpr_state_t *vpr) {
    if (vpr->launched) return;

    arm_platform_t *plat = (arm_platform_t *)vpr->plat;
    riscv_cpu_t *rv = (riscv_cpu_t *)calloc(1, sizeof(*rv));
    if (!rv) return;

    /* Share the M33's memory + IO bus — same SRAM, same peripherals. */
    riscv_cpu_init(rv, &plat->cpu, vpr->initpc);

    vpr->flpr     = rv;
    vpr->launched = 1;
    plat->cpu.coproc          = rv;
    plat->cpu.coproc_step     = flpr_costep;
    plat->cpu.coproc_raise_irq = flpr_raise_irq;

    fprintf(stderr, "[VPR] FLPR (RV32E) started at INITPC=0x%08x\n",
            vpr->initpc);
}
