/*
 * ARM Cortex-M3 Nested Vectored Interrupt Controller
 */
#include "arm_nvic.h"
#include <string.h>
#include <stdio.h>

/* NVIC IO region covers 0xE000E000 - 0xE000EFFF */
#define NVIC_IO_BASE  0xE000E000
#define NVIC_IO_SIZE  0x1000

static int nvic_read(void *user_data, uint32_t addr) {
    arm_nvic_t *nvic = (arm_nvic_t *)user_data;
    uint32_t offset = addr - NVIC_IO_BASE;

    /* NVIC_ISER */
    if (offset >= NVIC_ISER_BASE && offset < NVIC_ISER_BASE + 32) {
        int idx = (offset - NVIC_ISER_BASE) / 4;
        return (int)nvic->iser[idx];
    }
    /* NVIC_ICER */
    if (offset >= NVIC_ICER_BASE && offset < NVIC_ICER_BASE + 32) {
        int idx = (offset - NVIC_ICER_BASE) / 4;
        return (int)nvic->iser[idx];
    }
    /* NVIC_ISPR */
    if (offset >= NVIC_ISPR_BASE && offset < NVIC_ISPR_BASE + 32) {
        int idx = (offset - NVIC_ISPR_BASE) / 4;
        return (int)nvic->ispr[idx];
    }
    /* NVIC_ICPR */
    if (offset >= NVIC_ICPR_BASE && offset < NVIC_ICPR_BASE + 32) {
        int idx = (offset - NVIC_ICPR_BASE) / 4;
        return (int)nvic->ispr[idx];
    }
    /* NVIC_IABR */
    if (offset >= NVIC_IABR_BASE && offset < NVIC_IABR_BASE + 32) {
        int idx = (offset - NVIC_IABR_BASE) / 4;
        return (int)nvic->iabr[idx];
    }
    /* NVIC_IPR */
    if (offset >= NVIC_IPR_BASE && offset < NVIC_IPR_BASE + 240) {
        /* Read 4 bytes at a time */
        int base_idx = offset - NVIC_IPR_BASE;
        return nvic->ipr[base_idx] |
               (nvic->ipr[base_idx + 1] << 8) |
               (nvic->ipr[base_idx + 2] << 16) |
               (nvic->ipr[base_idx + 3] << 24);
    }

    /* System Control Block */
    switch (offset) {
        case SCB_CPUID: return 0x412FC231; /* Cortex-M3 r2p1 */
        case SCB_ICSR: {
            uint32_t val = 0;
            if (nvic->active_exception > 0)
                val |= nvic->active_exception & 0x1FF;
            if (nvic->pending_exception > 0)
                val |= ((nvic->pending_exception & 0x1FF) << 12) | (1u << 22);
            return (int)val;
        }
        case SCB_VTOR:  return (int)nvic->vtor;
        case SCB_AIRCR: return (int)nvic->aircr;
        case SCB_SCR:   return (int)nvic->scr;
        case SCB_CCR:   return (int)nvic->ccr;
        case SCB_SHPR1:
            return nvic->shpr[0] | (nvic->shpr[1] << 8) |
                   (nvic->shpr[2] << 16) | (nvic->shpr[3] << 24);
        case SCB_SHPR2:
            return nvic->shpr[4] | (nvic->shpr[5] << 8) |
                   (nvic->shpr[6] << 16) | (nvic->shpr[7] << 24);
        case SCB_SHPR3:
            return nvic->shpr[8] | (nvic->shpr[9] << 8) |
                   (nvic->shpr[10] << 16) | (nvic->shpr[11] << 24);
        case SCB_SHCSR: return (int)nvic->shcsr;
    }

    return 0;
}

static void nvic_write(void *user_data, uint32_t addr, uint32_t value) {
    arm_nvic_t *nvic = (arm_nvic_t *)user_data;
    uint32_t offset = addr - NVIC_IO_BASE;

    /* NVIC_ISER (set-enable) */
    if (offset >= NVIC_ISER_BASE && offset < NVIC_ISER_BASE + 32) {
        int idx = (offset - NVIC_ISER_BASE) / 4;
        nvic->iser[idx] |= value;
        arm_nvic_check_pending(nvic);
        return;
    }
    /* NVIC_ICER (clear-enable) */
    if (offset >= NVIC_ICER_BASE && offset < NVIC_ICER_BASE + 32) {
        int idx = (offset - NVIC_ICER_BASE) / 4;
        nvic->iser[idx] &= ~value;
        return;
    }
    /* NVIC_ISPR (set-pending) */
    if (offset >= NVIC_ISPR_BASE && offset < NVIC_ISPR_BASE + 32) {
        int idx = (offset - NVIC_ISPR_BASE) / 4;
        nvic->ispr[idx] |= value;
        arm_nvic_check_pending(nvic);
        return;
    }
    /* NVIC_ICPR (clear-pending) */
    if (offset >= NVIC_ICPR_BASE && offset < NVIC_ICPR_BASE + 32) {
        int idx = (offset - NVIC_ICPR_BASE) / 4;
        nvic->ispr[idx] &= ~value;
        return;
    }
    /* NVIC_IPR */
    if (offset >= NVIC_IPR_BASE && offset < NVIC_IPR_BASE + 240) {
        int base_idx = offset - NVIC_IPR_BASE;
        nvic->ipr[base_idx]     = value & 0xFF;
        nvic->ipr[base_idx + 1] = (value >> 8) & 0xFF;
        nvic->ipr[base_idx + 2] = (value >> 16) & 0xFF;
        nvic->ipr[base_idx + 3] = (value >> 24) & 0xFF;
        return;
    }

    switch (offset) {
        case SCB_ICSR:
            if (value & (1u << 28)) { /* PENDSVSET */
                nvic->pending_exception = EXC_PENDSV;
                nvic->has_pending = true;
                arm_nvic_check_pending(nvic);
            }
            if (value & (1u << 27)) { /* PENDSVCLR */
                if (nvic->pending_exception == EXC_PENDSV)
                    nvic->pending_exception = -1;
            }
            if (value & (1u << 26)) { /* PENDSTSET */
                nvic->pending_exception = EXC_SYSTICK;
                nvic->has_pending = true;
                arm_nvic_check_pending(nvic);
            }
            if (value & (1u << 25)) { /* PENDSTCLR */
                if (nvic->pending_exception == EXC_SYSTICK)
                    nvic->pending_exception = -1;
            }
            break;
        case SCB_VTOR:
            nvic->vtor = value & 0xFFFFFF80;
            nvic->cpu->vtor = nvic->vtor;
            break;
        case SCB_AIRCR:
            if ((value & 0xFFFF0000) == 0x05FA0000) {
                nvic->aircr = value & 0xFFFF;
                if (value & 4) {
                    /* SYSRESETREQ */
                    arm_cpu_reset(nvic->cpu);
                }
            }
            break;
        case SCB_SCR:
            nvic->scr = value;
            break;
        case SCB_CCR:
            nvic->ccr = value;
            break;
        case SCB_SHPR1:
            nvic->shpr[0] = value & 0xFF;
            nvic->shpr[1] = (value >> 8) & 0xFF;
            nvic->shpr[2] = (value >> 16) & 0xFF;
            nvic->shpr[3] = (value >> 24) & 0xFF;
            break;
        case SCB_SHPR2:
            nvic->shpr[4] = value & 0xFF;
            nvic->shpr[5] = (value >> 8) & 0xFF;
            nvic->shpr[6] = (value >> 16) & 0xFF;
            nvic->shpr[7] = (value >> 24) & 0xFF;
            break;
        case SCB_SHPR3:
            nvic->shpr[8] = value & 0xFF;
            nvic->shpr[9] = (value >> 8) & 0xFF;
            nvic->shpr[10] = (value >> 16) & 0xFF;
            nvic->shpr[11] = (value >> 24) & 0xFF;
            break;
        case SCB_SHCSR:
            nvic->shcsr = value;
            break;
    }
}

void arm_nvic_init(arm_nvic_t *nvic, arm_cpu_t *cpu) {
    memset(nvic, 0, sizeof(*nvic));
    nvic->cpu = cpu;
    nvic->active_exception = 0;
    nvic->pending_exception = -1;
    nvic->vtor = ARM_FLASH_BASE; /* CC2538 default VTOR */
    nvic->ccr = 0x200; /* STKALIGN=1 by default */

    /* Set back-pointer so exception_return can access NVIC */
    cpu->nvic = nvic;

    arm_register_io(cpu, NVIC_IO_BASE, NVIC_IO_SIZE, nvic_read, nvic_write, nvic);
}

void arm_nvic_set_pending(arm_nvic_t *nvic, int irq_num) {
    if (irq_num < 0 || irq_num >= NVIC_MAX_IRQ) return;
    int idx = irq_num / 32;
    int bit = irq_num % 32;
    nvic->ispr[idx] |= (1u << bit);
    nvic->has_pending = true;
    arm_nvic_check_pending(nvic);
}

void arm_nvic_clear_pending(arm_nvic_t *nvic, int irq_num) {
    if (irq_num < 0 || irq_num >= NVIC_MAX_IRQ) return;
    int idx = irq_num / 32;
    int bit = irq_num % 32;
    nvic->ispr[idx] &= ~(1u << bit);
}

int arm_nvic_get_priority(arm_nvic_t *nvic, int exception_num) {
    if (exception_num < 4) return -1; /* Fixed priority for reset/NMI/HardFault */
    if (exception_num < 16) {
        /* System handler: shpr[exception_num - 4] */
        return nvic->shpr[exception_num - 4];
    }
    /* External IRQ */
    int irq = exception_num - 16;
    if (irq >= 0 && irq < NVIC_MAX_IRQ)
        return nvic->ipr[irq];
    return 255;
}

uint32_t arm_nvic_get_vector(arm_nvic_t *nvic, int exception_num) {
    return arm_read32(nvic->cpu, nvic->vtor + exception_num * 4);
}

void arm_nvic_check_pending(arm_nvic_t *nvic) {
    arm_cpu_t *cpu = nvic->cpu;

    /* Check PRIMASK */
    if (cpu->primask & 1) return;

    int best_exc = -1;
    int best_prio = 256;

    /* Check system exceptions (PendSV, SysTick) */
    if (nvic->pending_exception > 0) {
        int prio = arm_nvic_get_priority(nvic, nvic->pending_exception);
        if (prio < best_prio) {
            best_prio = prio;
            best_exc = nvic->pending_exception;
        }
    }

    /* Check external IRQs */
    for (int i = 0; i < 8; i++) {
        uint32_t pending_enabled = nvic->ispr[i] & nvic->iser[i];
        if (pending_enabled == 0) continue;
        for (int bit = 0; bit < 32; bit++) {
            if (pending_enabled & (1u << bit)) {
                int irq = i * 32 + bit;
                int exc = irq + 16;
                int prio = arm_nvic_get_priority(nvic, exc);
                if (prio < best_prio) {
                    best_prio = prio;
                    best_exc = exc;
                }
            }
        }
    }

    if (best_exc < 0) {
        nvic->has_pending = false;
        return;
    }

    /* Check if we can preempt the current handler */
    if (nvic->active_exception > 0) {
        int active_prio = arm_nvic_get_priority(nvic, nvic->active_exception);
        if (best_prio >= active_prio) return; /* Cannot preempt, stay pending */
    }
    nvic->has_pending = false;

    /* Clear pending bit */
    if (best_exc >= 16) {
        int irq = best_exc - 16;
        nvic->ispr[irq / 32] &= ~(1u << (irq % 32));
    }
    if (best_exc == nvic->pending_exception)
        nvic->pending_exception = -1;

    /* Enter exception */
    nvic->active_exception = best_exc;
    arm_exception_entry(cpu, best_exc);
}
