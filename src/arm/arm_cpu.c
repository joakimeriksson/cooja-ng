/*
 * ARM Cortex-M3 CPU emulator — instruction execution engine
 */
#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_nvic.h"
#include "gdb_stub.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* --- Memory access helpers --- */

static inline arm_io_region_t *find_io_region(arm_cpu_t *cpu, uint32_t addr) {
    arm_io_region_t *best = NULL;
    for (int i = 0; i < cpu->num_io_regions; i++) {
        arm_io_region_t *r = &cpu->io_regions[i];
        if (addr >= r->base && addr < r->base + r->size) {
            if (!best || r->size < best->size)
                best = r;
        }
    }
    return best;
}

uint32_t arm_read32(arm_cpu_t *cpu, uint32_t addr) {
    addr &= ~3u;
    if (cpu->rom && addr < cpu->rom_size) {
        return cpu->rom[addr] | (cpu->rom[addr+1]<<8) |
               (cpu->rom[addr+2]<<16) | (cpu->rom[addr+3]<<24);
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        uint32_t off = addr - cpu->flash_base;
        return cpu->flash[off] | (cpu->flash[off+1]<<8) |
               (cpu->flash[off+2]<<16) | (cpu->flash[off+3]<<24);
    }
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        return cpu->sram[off] | (cpu->sram[off+1]<<8) |
               (cpu->sram[off+2]<<16) | (cpu->sram[off+3]<<24);
    }
    /* Bit-band alias for peripheral region */
    if (addr >= ARM_BITBAND_BASE && addr < ARM_BITBAND_BASE + 0x02000000) {
        uint32_t bb_off = addr - ARM_BITBAND_BASE;
        uint32_t base_addr = ARM_IO_BASE + (bb_off >> 5) * 4;
        int bit = (bb_off >> 2) & 0x1F;
        uint32_t val = arm_read32(cpu, base_addr);
        return (val >> bit) & 1;
    }
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) return r->read(r->user_data, addr);
    return 0;
}

/* Old unaligned functions removed — replaced by mem_*_unaligned inline fast-paths below */

/* --- Fast-path inline memory access for interpreter hot loop ---
 * These check SRAM first (most common data target), then flash,
 * then fall back to the public functions for IO/ROM/bitband. */

static inline uint32_t mem_read32(arm_cpu_t *cpu, uint32_t addr) {
    addr &= ~3u;
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1)) {
        uint32_t off = addr - cpu->sram_base;
        return cpu->sram[off] | (cpu->sram[off+1]<<8) |
               (cpu->sram[off+2]<<16) | (cpu->sram[off+3]<<24);
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        uint32_t off = addr - cpu->flash_base;
        return cpu->flash[off] | (cpu->flash[off+1]<<8) |
               (cpu->flash[off+2]<<16) | (cpu->flash[off+3]<<24);
    }
    return arm_read32(cpu, addr);
}

static inline uint16_t mem_read16(arm_cpu_t *cpu, uint32_t addr) {
    addr &= ~1u;
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1)) {
        uint32_t off = addr - cpu->sram_base;
        return cpu->sram[off] | (cpu->sram[off+1]<<8);
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        uint32_t off = addr - cpu->flash_base;
        return cpu->flash[off] | (cpu->flash[off+1]<<8);
    }
    return arm_read16(cpu, addr);
}

static inline uint8_t mem_read8(arm_cpu_t *cpu, uint32_t addr) {
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1))
        return cpu->sram[addr - cpu->sram_base];
    if (addr >= cpu->flash_base && addr < cpu->flash_end)
        return cpu->flash[addr - cpu->flash_base];
    return arm_read8(cpu, addr);
}

static inline void mem_write32(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    addr &= ~3u;
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1)) {
        uint32_t off = addr - cpu->sram_base;
        cpu->sram[off]   = val & 0xFF;
        cpu->sram[off+1] = (val >> 8) & 0xFF;
        cpu->sram[off+2] = (val >> 16) & 0xFF;
        cpu->sram[off+3] = (val >> 24) & 0xFF;
        return;
    }
    arm_write32(cpu, addr, val);
}

static inline void mem_write16(arm_cpu_t *cpu, uint32_t addr, uint16_t val) {
    addr &= ~1u;
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1)) {
        uint32_t off = addr - cpu->sram_base;
        cpu->sram[off]   = val & 0xFF;
        cpu->sram[off+1] = (val >> 8) & 0xFF;
        return;
    }
    arm_write16(cpu, addr, val);
}

static inline void mem_write8(arm_cpu_t *cpu, uint32_t addr, uint8_t val) {
    if (__builtin_expect(addr >= cpu->sram_base && addr < cpu->sram_end, 1)) {
        cpu->sram[addr - cpu->sram_base] = val;
        return;
    }
    arm_write8(cpu, addr, val);
}

static inline uint32_t mem_read32_unaligned(arm_cpu_t *cpu, uint32_t addr) {
    if (__builtin_expect((addr & 3) == 0, 1))
        return mem_read32(cpu, addr);
    return mem_read8(cpu, addr) | ((uint32_t)mem_read8(cpu, addr+1) << 8) |
           ((uint32_t)mem_read8(cpu, addr+2) << 16) | ((uint32_t)mem_read8(cpu, addr+3) << 24);
}

static inline void mem_write32_unaligned(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    if (__builtin_expect((addr & 3) == 0, 1)) {
        mem_write32(cpu, addr, val);
        return;
    }
    mem_write8(cpu, addr, val & 0xFF);
    mem_write8(cpu, addr+1, (val >> 8) & 0xFF);
    mem_write8(cpu, addr+2, (val >> 16) & 0xFF);
    mem_write8(cpu, addr+3, (val >> 24) & 0xFF);
}

static inline uint16_t mem_read16_unaligned(arm_cpu_t *cpu, uint32_t addr) {
    if (__builtin_expect((addr & 1) == 0, 1))
        return mem_read16(cpu, addr);
    return mem_read8(cpu, addr) | ((uint16_t)mem_read8(cpu, addr+1) << 8);
}

static inline void mem_write16_unaligned(arm_cpu_t *cpu, uint32_t addr, uint16_t val) {
    if (__builtin_expect((addr & 1) == 0, 1)) {
        mem_write16(cpu, addr, val);
        return;
    }
    mem_write8(cpu, addr, val & 0xFF);
    mem_write8(cpu, addr+1, (val >> 8) & 0xFF);
}

uint16_t arm_read16(arm_cpu_t *cpu, uint32_t addr) {
    addr &= ~1u;
    if (cpu->rom && addr < cpu->rom_size) {
        return cpu->rom[addr] | (cpu->rom[addr+1]<<8);
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        uint32_t off = addr - cpu->flash_base;
        return cpu->flash[off] | (cpu->flash[off+1]<<8);
    }
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        return cpu->sram[off] | (cpu->sram[off+1]<<8);
    }
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) return (uint16_t)r->read(r->user_data, addr);
    return 0;
}

uint8_t arm_read8(arm_cpu_t *cpu, uint32_t addr) {
    if (cpu->rom && addr < cpu->rom_size) return cpu->rom[addr];
    if (addr >= cpu->flash_base && addr < cpu->flash_end)
        return cpu->flash[addr - cpu->flash_base];
    if (addr >= cpu->sram_base && addr < cpu->sram_end)
        return cpu->sram[addr - cpu->sram_base];
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) return (uint8_t)r->read(r->user_data, addr);
    return 0;
}

void arm_write32(arm_cpu_t *cpu, uint32_t addr, uint32_t val) {
    addr &= ~3u;
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        cpu->sram[off]   = val & 0xFF;
        cpu->sram[off+1] = (val >> 8) & 0xFF;
        cpu->sram[off+2] = (val >> 16) & 0xFF;
        cpu->sram[off+3] = (val >> 24) & 0xFF;
        return;
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        /* Flash is treated as read-only.  Real nrf MCUs require an
         * NVMC/RRAMC unlock before flash writes; csim doesn't model
         * the flash controller, and stray writes through wild pointers
         * (r3=0 in an early-boot ISR) would otherwise corrupt the
         * firmware image and cause unrecoverable PC jumps. */
        return;
    }
    if (cpu->rom && addr < cpu->rom_size) return; /* ROM is read-only */
    /* Bit-band alias for peripheral region */
    if (addr >= ARM_BITBAND_BASE && addr < ARM_BITBAND_BASE + 0x02000000) {
        uint32_t bb_off = addr - ARM_BITBAND_BASE;
        uint32_t base_addr = ARM_IO_BASE + (bb_off >> 5) * 4;
        int bit = (bb_off >> 2) & 0x1F;
        uint32_t old = arm_read32(cpu, base_addr);
        if (val & 1)
            arm_write32(cpu, base_addr, old | (1u << bit));
        else
            arm_write32(cpu, base_addr, old & ~(1u << bit));
        return;
    }
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) r->write(r->user_data, addr, val);
}

void arm_write16(arm_cpu_t *cpu, uint32_t addr, uint16_t val) {
    addr &= ~1u;
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        cpu->sram[off]   = val & 0xFF;
        cpu->sram[off+1] = (val >> 8) & 0xFF;
        return;
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) return;
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) r->write(r->user_data, addr, val);
}

void arm_write8(arm_cpu_t *cpu, uint32_t addr, uint8_t val) {
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        cpu->sram[off] = val;
        return;
    }
    if (addr >= cpu->flash_base && addr < cpu->flash_end) return;
    arm_io_region_t *r = find_io_region(cpu, addr);
    if (r) r->write(r->user_data, addr, val);
}

/* --- Inline fetch helpers --- */

static inline uint16_t fetch16(arm_cpu_t *cpu, uint32_t addr) {
    if (addr >= cpu->flash_base && addr < cpu->flash_end) {
        uint32_t off = addr - cpu->flash_base;
        return cpu->flash[off] | (cpu->flash[off+1] << 8);
    }
    if (cpu->rom && addr < cpu->rom_size)
        return cpu->rom[addr] | (cpu->rom[addr+1] << 8);
    if (addr >= cpu->sram_base && addr < cpu->sram_end) {
        uint32_t off = addr - cpu->sram_base;
        return cpu->sram[off] | (cpu->sram[off+1] << 8);
    }
    return arm_read16(cpu, addr);
}

/* --- Lifecycle --- */

void arm_cpu_init(arm_cpu_t *cpu, const arm_config_t *config) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->config = config;
    cpu->gdb_stub = NULL;

    /* Cache memory layout from config — used by every load/store hot path. */
    cpu->flash_base   = config->flash_base;
    cpu->flash_end    = config->flash_base + config->flash_size;
    cpu->sram_base    = config->sram_base;
    cpu->sram_end     = config->sram_base + config->sram_size;
    cpu->rom_size     = config->rom_size;
    cpu->vtor_default = config->vtor_default;

    cpu->flash = (uint8_t *)calloc(config->flash_size, 1);
    cpu->sram  = (uint8_t *)calloc(config->sram_size,  1);
    cpu->rom   = (config->rom_size > 0) ? (uint8_t *)calloc(config->rom_size, 1) : NULL;

    cpu->next_event_cycle = INT64_MAX;
    cpu->cycle_limit = INT64_MAX;
    cpu->last_execute_us = -1;
    cpu->last_micros_cycles = 0;
    cpu->last_micros_delta = 0;
    cpu->micro_clock_ready = false;
    cpu->step_cycle_remainder = 0.0;
    cpu->cpu_freq_hz = config->default_cpu_freq;
    cpu->interrupts_enabled = true;

    /* CC2538-specific ROM utility table — only present when the SoC has
     * an emulated ROM region (cc2538_config). nRF52840 and other ROM-less
     * SoCs skip this entirely; firmware that doesn't try to call into ROM
     * never notices.
     *
     * CC2538 ROM utility function table layout:
     * ROM[0x10] = pointer to rom_util_api struct (0x48)
     * rom_util_api struct at 0x48:
     *   offset 0x18 (addr 0x60): memset function pointer
     *   offset 0x1C (addr 0x64): memcpy function pointer
     *   offset 0x20 (addr 0x68): memcmp function pointer
     * Firmware loads function pointers via: ldr r3, [r4, #0x60] (with r4=0)
     * We place trap addresses at those ROM locations and intercept execution there.
     */
    if (cpu->rom) {
        cpu->rom_util_memset = 0x00000060;
        cpu->rom_util_memcpy = 0x00000064;
        cpu->rom_util_memcmp = 0x00000068;
#define WRITE_ROM32(off, val) do { \
    cpu->rom[(off)]   = (val) & 0xFF; \
    cpu->rom[(off)+1] = ((val) >> 8) & 0xFF; \
    cpu->rom[(off)+2] = ((val) >> 16) & 0xFF; \
    cpu->rom[(off)+3] = ((val) >> 24) & 0xFF; \
} while(0)
        WRITE_ROM32(0x10, 0x00000048);  /* rom_util_api table pointer */
        WRITE_ROM32(0x60, 0x00000061);  /* memset: points to 0x60 | Thumb */
        WRITE_ROM32(0x64, 0x00000065);  /* memcpy: points to 0x64 | Thumb */
        WRITE_ROM32(0x68, 0x00000069);  /* memcmp: points to 0x68 | Thumb */
#undef WRITE_ROM32
    }
}

void arm_cpu_destroy(arm_cpu_t *cpu) {
    free(cpu->rom);
    free(cpu->flash);
    free(cpu->sram);
    cpu->event_queue = NULL;
}

void arm_cpu_reset(arm_cpu_t *cpu) {
    memset(cpu->reg, 0, sizeof(cpu->reg));
    cpu->xpsr = 0;
    cpu->it_state = 0;
    cpu->primask = 0;
    cpu->faultmask = 0;
    cpu->basepri = 0;
    cpu->use_psp = false;
    cpu->interrupts_enabled = true;
    cpu->cpu_off = false;
    cpu->stopping = false;

    cpu->event_queue = NULL;
    cpu->next_event_cycle = INT64_MAX;
    cpu->last_execute_us = -1;
    cpu->last_micros_cycles = 0;
    cpu->last_micros_delta = 0;
    cpu->micro_clock_ready = false;
    cpu->step_cycle_remainder = 0.0;

    /* Vector table discovery, in order of preference:
     *   1. cpu->vtor_default (seeded from SoC config, may be overridden
     *      by SoC init op based on platform config — e.g. nrf52840
     *      Dongle = 0x1000, DK = 0x0).
     *   2. CC2538 CCA at flash_end - 0x2C, where CCA+8 carries the
     *      application entry point. CCA is a TI-specific convention.
     *   3. Plain flash_base as a last resort. */
    if (cpu->vtor_default != 0) {
        cpu->vtor = cpu->vtor_default;
    } else {
        uint32_t cca_addr = cpu->flash_end - 0x2C;
        cpu->vtor = arm_read32(cpu, cca_addr + 8);
        if (cpu->vtor < cpu->flash_base || cpu->vtor >= cpu->flash_end)
            cpu->vtor = cpu->flash_base; /* fallback */
    }
    uint32_t sp = arm_read32(cpu, cpu->vtor);
    uint32_t pc = arm_read32(cpu, cpu->vtor + 4);

    cpu->reg[ARM_SP] = sp;
    cpu->msp = sp;
    cpu->reg[ARM_PC] = pc & ~1u; /* Clear thumb bit */
    cpu->xpsr = (1u << 24); /* Thumb bit in EPSR must be set */
}

void arm_stop(arm_cpu_t *cpu) {
    cpu->stopping = true;
}

/* --- IO registration --- */

void arm_register_io(arm_cpu_t *cpu, uint32_t base, uint32_t size,
                     arm_io_read_fn read, arm_io_write_fn write, void *data) {
    if (cpu->num_io_regions >= ARM_MAX_IO_REGIONS) {
        fprintf(stderr, "ARM: too many IO regions\n");
        return;
    }
    arm_io_region_t *r = &cpu->io_regions[cpu->num_io_regions++];
    r->base = base;
    r->size = size;
    r->read = read;
    r->write = write;
    r->user_data = data;
}

/* --- Event management (generated from shared template) --- */

#include "event_queue.h"
EVENT_QUEUE_IMPL(arm, arm_cpu_t, arm_event_t)

/* --- Exception entry/return --- */

void arm_exception_entry(arm_cpu_t *cpu, int exception_num) {
    /* Push exception frame: R0-R3, R12, LR, ReturnAddr, xPSR */
    uint32_t sp = cpu->reg[ARM_SP];
    uint32_t sp_align = sp & 4u; /* Non-zero if 4-byte but not 8-byte aligned */
    sp &= ~7u; /* 8-byte align */
    sp -= 32;

    /* Encode IT state into xPSR before saving.
     * ITSTATE is in xPSR bits [26:25] (lower 2) and [15:10] (upper 6).
     * Bit 9 indicates stack alignment padding occurred. */
    uint32_t saved_xpsr = cpu->xpsr;
    saved_xpsr &= ~((0x3u << 25) | (0x3Fu << 10) | (1u << 9)); /* Clear IT + align bits */
    saved_xpsr |= ((uint32_t)(cpu->it_state & 0x3) << 25);
    saved_xpsr |= ((uint32_t)((cpu->it_state >> 2) & 0x3F) << 10);
    if (sp_align) saved_xpsr |= (1u << 9); /* Mark alignment padding */

    arm_write32(cpu, sp + 0,  cpu->reg[0]);
    arm_write32(cpu, sp + 4,  cpu->reg[1]);
    arm_write32(cpu, sp + 8,  cpu->reg[2]);
    arm_write32(cpu, sp + 12, cpu->reg[3]);
    arm_write32(cpu, sp + 16, cpu->reg[12]);
    arm_write32(cpu, sp + 20, cpu->reg[ARM_LR]);
    arm_write32(cpu, sp + 24, cpu->reg[ARM_PC]);
    arm_write32(cpu, sp + 28, saved_xpsr);

    cpu->reg[ARM_SP] = sp;
    cpu->it_state = 0; /* Clear IT state for handler */

    /* Set EXC_RETURN in LR.
     * 0xFFFFFFF1 = Return to Handler mode (nested interrupt), MSP
     * 0xFFFFFFF9 = Return to Thread mode, MSP
     * 0xFFFFFFFD = Return to Thread mode, PSP */
    if ((cpu->xpsr & 0x1FF) != 0) {
        cpu->reg[ARM_LR] = 0xFFFFFFF1; /* Nested: return to Handler, MSP */
    } else if (cpu->use_psp) {
        cpu->reg[ARM_LR] = 0xFFFFFFFD; /* Return to Thread using PSP */
    } else {
        cpu->reg[ARM_LR] = 0xFFFFFFF9; /* Return to Thread using MSP */
    }

    /* Load vector */
    uint32_t vector = arm_read32(cpu, cpu->vtor + exception_num * 4);
    cpu->reg[ARM_PC] = vector & ~1u;
    cpu->xpsr = (cpu->xpsr & ~0x1FFu) | (exception_num & 0x1FF);
    cpu->xpsr |= (1u << 24); /* Thumb bit */

    cpu->cpu_off = false; /* Wake from WFI */
    cpu->cycles += 12; /* Exception entry latency */
}

void arm_check_pending_exceptions(arm_cpu_t *cpu) {
    /* Simplified: this is called by NVIC when it determines an exception should fire */
    (void)cpu;
}

static void exception_return(arm_cpu_t *cpu, uint32_t exc_return) {
    /* Pop exception frame */
    uint32_t sp = cpu->reg[ARM_SP];

    cpu->reg[0]      = arm_read32(cpu, sp + 0);
    cpu->reg[1]      = arm_read32(cpu, sp + 4);
    cpu->reg[2]      = arm_read32(cpu, sp + 8);
    cpu->reg[3]      = arm_read32(cpu, sp + 12);
    cpu->reg[12]     = arm_read32(cpu, sp + 16);
    cpu->reg[ARM_LR] = arm_read32(cpu, sp + 20);
    cpu->reg[ARM_PC] = arm_read32(cpu, sp + 24) & ~1u;
    cpu->xpsr        = arm_read32(cpu, sp + 28);

    /* Restore IT state from xPSR bits [26:25] and [15:10] */
    cpu->it_state = (uint8_t)(((cpu->xpsr >> 25) & 0x3) |
                              (((cpu->xpsr >> 10) & 0x3F) << 2));

    /* Account for stack alignment padding (xPSR bit 9) */
    cpu->reg[ARM_SP] = sp + 32 + ((cpu->xpsr & (1u << 9)) ? 4 : 0);

    /* Restore execution mode from EXC_RETURN.
     * 0xFFFFFFF1: Return to Handler mode (nested) — keep IPSR from popped xPSR
     * 0xFFFFFFF9/FD: Return to Thread mode — clear IPSR */
    if ((exc_return & 0xF) == 0x9 || (exc_return & 0xF) == 0xD) {
        cpu->xpsr &= ~0x1FFu; /* Clear IPSR -> Thread mode */
    }
    /* else: 0xFFFFFFF1 — IPSR already restored from popped xPSR (outer handler) */

    /* Restore active exception from the IPSR of the state we're returning to */
    if (cpu->nvic) {
        arm_nvic_t *nvic = (arm_nvic_t *)cpu->nvic;
        nvic->active_exception = cpu->xpsr & 0x1FF;
        arm_nvic_check_pending(nvic);
    }

    cpu->cycles += 12;
}

/* --- APSR flag helpers --- */

static inline void set_nz(arm_cpu_t *cpu, uint32_t result) {
    cpu->xpsr &= ~(APSR_N | APSR_Z);
    if (result == 0) cpu->xpsr |= APSR_Z;
    if (result & 0x80000000) cpu->xpsr |= APSR_N;
}

static inline void set_nzc(arm_cpu_t *cpu, uint32_t result, int carry) {
    cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C);
    if (result == 0) cpu->xpsr |= APSR_Z;
    if (result & 0x80000000) cpu->xpsr |= APSR_N;
    if (carry) cpu->xpsr |= APSR_C;
}

static inline void set_add_flags(arm_cpu_t *cpu, uint32_t a, uint32_t b, uint64_t result) {
    uint32_t r32 = (uint32_t)result;
    cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C | APSR_V);
    if (r32 == 0) cpu->xpsr |= APSR_Z;
    if (r32 & 0x80000000) cpu->xpsr |= APSR_N;
    if (result > 0xFFFFFFFF) cpu->xpsr |= APSR_C;
    if (((a ^ r32) & (b ^ r32)) >> 31) cpu->xpsr |= APSR_V;
}

static inline void set_sub_flags(arm_cpu_t *cpu, uint32_t a, uint32_t b, uint64_t result) {
    uint32_t r32 = (uint32_t)result;
    cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C | APSR_V);
    if (r32 == 0) cpu->xpsr |= APSR_Z;
    if (r32 & 0x80000000) cpu->xpsr |= APSR_N;
    if (a >= b) cpu->xpsr |= APSR_C; /* No borrow = carry set */
    if (((a ^ b) & (a ^ r32)) >> 31) cpu->xpsr |= APSR_V;
}

static inline int condition_passed(arm_cpu_t *cpu, int cond) {
    uint32_t f = cpu->xpsr;
    int result;
    switch (cond >> 1) {
        case 0: result = (f & APSR_Z) != 0; break;       /* EQ/NE */
        case 1: result = (f & APSR_C) != 0; break;       /* CS/CC */
        case 2: result = (f & APSR_N) != 0; break;       /* MI/PL */
        case 3: result = (f & APSR_V) != 0; break;       /* VS/VC */
        case 4: result = ((f & APSR_C) != 0) && ((f & APSR_Z) == 0); break; /* HI/LS */
        case 5: result = ((f >> 31) & 1) == ((f >> 28) & 1); break; /* GE/LT */
        case 6: result = (((f >> 31) & 1) == ((f >> 28) & 1)) && ((f & APSR_Z) == 0); break; /* GT/LE */
        case 7: result = 1; break; /* AL */
        default: result = 1; break;
    }
    if (cond & 1) result = !result;
    return result;
}

/* --- Barrel shifter helpers --- */

static inline uint32_t lsl_c(uint32_t val, int shift, int *carry) {
    if (shift == 0) return val;
    if (shift >= 32) { *carry = (shift == 32) ? (val & 1) : 0; return 0; }
    *carry = (val >> (32 - shift)) & 1;
    return val << shift;
}

static inline uint32_t lsr_c(uint32_t val, int shift, int *carry) {
    if (shift >= 32) { *carry = (shift == 32) ? ((val >> 31) & 1) : 0; return 0; }
    *carry = (val >> (shift - 1)) & 1;
    return val >> shift;
}

static inline uint32_t asr_c(uint32_t val, int shift, int *carry) {
    int32_t sval = (int32_t)val;
    if (shift >= 32) {
        *carry = (val >> 31) & 1;
        return (sval < 0) ? 0xFFFFFFFF : 0;
    }
    *carry = (val >> (shift - 1)) & 1;
    return (uint32_t)(sval >> shift);
}

static inline uint32_t ror_c(uint32_t val, int shift, int *carry) {
    shift &= 31;
    if (shift == 0) { *carry = (val >> 31) & 1; return val; }
    uint32_t result = (val >> shift) | (val << (32 - shift));
    *carry = (result >> 31) & 1;
    return result;
}

/* Thumb-2 modified immediate constant (i:imm3:imm8 with rotation) */
static inline uint32_t thumb_expand_imm_c(uint16_t imm12, int *carry_out, int carry_in) {
    if ((imm12 >> 10) == 0) {
        uint32_t imm8 = imm12 & 0xFF;
        switch ((imm12 >> 8) & 3) {
            case 0: *carry_out = carry_in; return imm8;
            case 1: *carry_out = carry_in; return (imm8 << 16) | imm8;
            case 2: *carry_out = carry_in; return (imm8 << 24) | (imm8 << 8);
            case 3: *carry_out = carry_in; return (imm8 << 24) | (imm8 << 16) | (imm8 << 8) | imm8;
        }
    }
    /* ROR rotation */
    int rot = imm12 >> 7;
    uint32_t val = 0x80 | (imm12 & 0x7F);
    return ror_c(val, rot, carry_out);
}

/* --- SP-balance auditor (ARM_SP_AUDIT) ---
 * On every BL/BLX, push (return_pc, sp_before_call, callee_pc, in_exc).
 * On every step, if PC matches the top entry's return_pc, pop and report
 * SP delta if non-zero — the callee returned with SP unbalanced.
 *
 * Cheap to leave compiled in; the per-call env-var check is cached on
 * first hit so the hot-path overhead is one inlined branch when the
 * diagnostic is off.
 */
static int arm_sp_audit_enabled = -1;
static const char *arm_bl_probe_str;

static void arm_audit_probe_env(void) {
    arm_sp_audit_enabled = getenv("ARM_SP_AUDIT") ? 1 : 0;
    arm_bl_probe_str = getenv("ARM_BL_PROBE");
}

static inline void arm_sp_audit_push(arm_cpu_t *cpu, uint32_t return_pc,
                                       uint32_t callee_pc) {
    if (__builtin_expect(arm_sp_audit_enabled < 0, 0)) arm_audit_probe_env();
    /* ARM_BL_PROBE=<callee_pc>:<sram_addr> — when bl jumps to <callee_pc>,
     * dump byte at <sram_addr>.  Lets us watch a state variable just
     * before a function reads it. */
    if (__builtin_expect(arm_bl_probe_str != NULL, 0)) {
        const char *e = arm_bl_probe_str;
        const char *colon = strchr(e, ':');
        if (colon) {
            uint32_t want_callee = (uint32_t)strtoul(e, NULL, 0) & ~1u;
            uint32_t addr = (uint32_t)strtoul(colon + 1, NULL, 0);
            if ((callee_pc & ~1u) == want_callee &&
                addr >= cpu->sram_base && addr + 1 <= cpu->sram_end) {
                static int n = 0;
                if (n++ < 30) {
                    uint8_t v = cpu->sram[addr - cpu->sram_base];
                    fprintf(stderr,
                            "[BL_PROBE cpu=%p callee=0x%08x mem[0x%08x]=0x%02x "
                            "sp=0x%08x lr=0x%08x cyc=%lld]\n",
                            (void*)cpu, callee_pc & ~1u, addr, v,
                            cpu->reg[ARM_SP], cpu->reg[ARM_LR],
                            (long long)cpu->cycles);
                }
            }
        }
    }
    if (__builtin_expect(!arm_sp_audit_enabled, 1)) return;
    if (cpu->sp_audit_top >= ARM_SP_AUDIT_DEPTH) {
        cpu->sp_audit_overflow++;
        return;
    }
    int i = cpu->sp_audit_top++;
    cpu->sp_audit[i].return_pc    = return_pc & ~1u;
    cpu->sp_audit[i].saved_sp     = cpu->reg[ARM_SP];
    cpu->sp_audit[i].callee_pc    = callee_pc & ~1u;
    cpu->sp_audit[i].in_exception = (cpu->xpsr & 0x1FF) != 0 ? 1 : 0;
}

static inline void arm_sp_audit_check(arm_cpu_t *cpu) {
    if (__builtin_expect(arm_sp_audit_enabled <= 0, 1)) return;
    while (cpu->sp_audit_top > 0) {
        int i = cpu->sp_audit_top - 1;
        uint32_t pc = cpu->reg[ARM_PC] & ~1u;
        if (pc != cpu->sp_audit[i].return_pc) return;
        uint32_t expected_sp = cpu->sp_audit[i].saved_sp;
        uint32_t actual_sp = cpu->reg[ARM_SP];
        if (expected_sp != actual_sp) {
            static int n = 0;
            if (n++ < 30)
                fprintf(stderr,
                        "[SP_AUDIT cpu=%p callee=0x%08x return_pc=0x%08x "
                        "saved_sp=0x%08x sp_now=0x%08x delta=%d cyc=%lld]\n",
                        (void*)cpu, cpu->sp_audit[i].callee_pc,
                        cpu->sp_audit[i].return_pc,
                        expected_sp, actual_sp, (int)(actual_sp - expected_sp),
                        (long long)cpu->cycles);
        }
        cpu->sp_audit_top--;
    }
}

/* --- ROM utility trap handler --- */
static int handle_fw_trap(arm_cpu_t *cpu) {
    uint32_t pc = cpu->reg[ARM_PC];

    if (cpu->fw_udivmoddi4 && pc == cpu->fw_udivmoddi4) {
        /* __udivmoddi4(uint64_t num, uint64_t den, uint64_t *rem_p)
           R0:R1 = numerator, R2:R3 = denominator, [SP] = remainder pointer */
        uint64_t num = ((uint64_t)cpu->reg[1] << 32) | cpu->reg[0];
        uint64_t den = ((uint64_t)cpu->reg[3] << 32) | cpu->reg[2];
        uint64_t quot = 0;
        uint64_t rem = 0;
        if (den != 0) {
            quot = num / den;
            rem = num % den;
        }
        cpu->reg[0] = (uint32_t)(quot & 0xFFFFFFFFu);
        cpu->reg[1] = (uint32_t)(quot >> 32);
        /* Write remainder via pointer (5th arg on stack) */
        uint32_t rem_p = arm_read32(cpu, cpu->reg[ARM_SP]);
        if (rem_p) {
            arm_write32(cpu, rem_p, (uint32_t)(rem & 0xFFFFFFFFu));
            arm_write32(cpu, rem_p + 4, (uint32_t)(rem >> 32));
        }
        cpu->reg[ARM_PC] = cpu->reg[ARM_LR] & ~1u;
        cpu->cycles += 20;
        return 1;
    }

    if (cpu->fw_aeabi_uldivmod && pc == cpu->fw_aeabi_uldivmod) {
        /* __aeabi_uldivmod: R0:R1=num, R2:R3=den -> R0:R1=quot, R2:R3=rem */
        uint64_t num = ((uint64_t)cpu->reg[1] << 32) | cpu->reg[0];
        uint64_t den = ((uint64_t)cpu->reg[3] << 32) | cpu->reg[2];
        uint64_t quot = 0;
        uint64_t rem = 0;
        if (den != 0) {
            quot = num / den;
            rem = num % den;
        }
        cpu->reg[0] = (uint32_t)(quot & 0xFFFFFFFFu);
        cpu->reg[1] = (uint32_t)(quot >> 32);
        cpu->reg[2] = (uint32_t)(rem & 0xFFFFFFFFu);
        cpu->reg[3] = (uint32_t)(rem >> 32);
        cpu->reg[ARM_PC] = cpu->reg[ARM_LR] & ~1u;
        cpu->cycles += 20;
        return 1;
    }

    return 0;
}

static int handle_rom_trap(arm_cpu_t *cpu) {
    uint32_t pc = cpu->reg[ARM_PC];

    if (pc == cpu->rom_util_memcpy || pc == cpu->rom_util_memcpy + 1) {
        /* memcpy(dst, src, len) - R0=dst, R1=src, R2=len */
        uint32_t dst = cpu->reg[0], src = cpu->reg[1], len = cpu->reg[2];
        for (uint32_t i = 0; i < len; i++)
            arm_write8(cpu, dst + i, arm_read8(cpu, src + i));
        /* Return value in R0 = dst (already there) */
        cpu->reg[ARM_PC] = cpu->reg[ARM_LR] & ~1u;
        cpu->cycles += len + 3;
        return 1;
    }
    if (pc == cpu->rom_util_memset || pc == cpu->rom_util_memset + 1) {
        /* memset(dst, val, len) - R0=dst, R1=val, R2=len */
        uint32_t dst = cpu->reg[0], val = cpu->reg[1] & 0xFF, len = cpu->reg[2];
        for (uint32_t i = 0; i < len; i++)
            arm_write8(cpu, dst + i, (uint8_t)val);
        cpu->reg[ARM_PC] = cpu->reg[ARM_LR] & ~1u;
        cpu->cycles += len + 3;
        return 1;
    }
    if (pc == cpu->rom_util_memcmp || pc == cpu->rom_util_memcmp + 1) {
        /* memcmp(s1, s2, len) - R0=s1, R1=s2, R2=len */
        uint32_t s1 = cpu->reg[0], s2 = cpu->reg[1], len = cpu->reg[2];
        int result = 0;
        for (uint32_t i = 0; i < len; i++) {
            int a = arm_read8(cpu, s1 + i), b = arm_read8(cpu, s2 + i);
            if (a != b) { result = a - b; break; }
        }
        cpu->reg[0] = (uint32_t)result;
        cpu->reg[ARM_PC] = cpu->reg[ARM_LR] & ~1u;
        cpu->cycles += len + 3;
        return 1;
    }
    return 0;
}

/* --- Main execution loop --- */

/* ============================================================
 * Single-step instruction trace facility.
 *
 * Disabled by default (zero perf hit — one bool check per step).
 * Enable with ARM_TRACE_PC=1.  Optional filters:
 *   ARM_TRACE_FROM=<cycle>   start cycle (default 0)
 *   ARM_TRACE_TO=<cycle>     end cycle (default INT64_MAX)
 *   ARM_TRACE_MAX=<count>    max lines (default 5000)
 *   ARM_TRACE_NODE=<sram>    only trace cpu whose sram_base hash
 *                            matches the low 32 bits of this value;
 *                            otherwise trace all nodes
 *
 * Each line: PC, SP, LR, r0-r3, xPSR for one executed instruction.
 * Designed to be cross-referenced with `arm-none-eabi-objdump -d
 * <firmware> --start-address=<pc> --stop-address=<pc+8>`.
 * ============================================================ */
static int   arm_trace_state = -1;   /* -1 = not yet probed */
static int64_t arm_trace_from = 0;
static int64_t arm_trace_to   = INT64_MAX;
static int   arm_trace_max    = 5000;
static int   arm_trace_node   = 0;   /* 0 = any */
static int   arm_trace_count  = 0;

static void arm_trace_probe(void) {
    const char *v = getenv("ARM_TRACE_PC");
    arm_trace_state = (v && *v && *v != '0') ? 1 : 0;
    if (!arm_trace_state) return;
    const char *f = getenv("ARM_TRACE_FROM");
    const char *t = getenv("ARM_TRACE_TO");
    const char *m = getenv("ARM_TRACE_MAX");
    const char *n = getenv("ARM_TRACE_NODE");
    if (f) arm_trace_from = strtoll(f, NULL, 0);
    if (t) arm_trace_to   = strtoll(t, NULL, 0);
    if (m) arm_trace_max  = atoi(m);
    if (n) arm_trace_node = (int)strtoul(n, NULL, 0);
    fprintf(stderr, "[arm_trace ON from=%lld to=%lld max=%d node=0x%x]\n",
            (long long)arm_trace_from, (long long)arm_trace_to,
            arm_trace_max, arm_trace_node);
}

static inline void arm_trace_step(arm_cpu_t *cpu) {
    if (__builtin_expect(arm_trace_state == 0, 1)) return;
    if (arm_trace_state < 0) {
        arm_trace_probe();
        if (!arm_trace_state) return;
    }
    if (cpu->cycles < arm_trace_from || cpu->cycles > arm_trace_to) return;
    /* Node filter: low 16 bits of cpu pointer give an unstable but
     * unique-enough per-instance ID.  User picks a value that matches
     * the node they want to trace by trial and error, or 0 for all. */
    if (arm_trace_node) {
        int node_tag = (int)((uintptr_t)cpu & 0xFFFF);
        if (node_tag != (arm_trace_node & 0xFFFF)) return;
    }
    if (arm_trace_count >= arm_trace_max) return;
    arm_trace_count++;
    fprintf(stderr,
            "[t#%d cyc=%lld pc=0x%08x sp=0x%08x lr=0x%08x "
            "r0=%08x r1=%08x r2=%08x r3=%08x xpsr=%08x cpu=%p]\n",
            arm_trace_count, (long long)cpu->cycles,
            cpu->reg[15] & ~1u, cpu->reg[13], cpu->reg[14],
            cpu->reg[0], cpu->reg[1], cpu->reg[2], cpu->reg[3],
            cpu->xpsr, (void*)cpu);
}

int arm_step(arm_cpu_t *cpu, int count) {
    int remaining = count;

    while (remaining > 0 && !cpu->stopping) {
        arm_trace_step(cpu);
        /* GDB stub: check breakpoint at current PC, then poll for halt
         * commands. If halted, stop the inner loop so the multinode
         * driver can pump the stub's command processor. */
        if (cpu->gdb_stub) {
            gdb_stub_t *g = (gdb_stub_t *)cpu->gdb_stub;
            uint32_t pc_check = cpu->reg[ARM_PC] & ~1u;
            if (gdb_stub_check_breakpoint(g, pc_check)) {
                cpu->stopping = true;
                break;
            }
            if (g->halted) {
                cpu->stopping = true;
                break;
            }
        }

        /* Check events */
        if (cpu->cycles >= cpu->next_event_cycle)
            execute_events(cpu);

        /* CPU off (WFI) — advance to next event or wake on interrupt.
         *
         * Cortex-M WFI wakes on any pending interrupt regardless of
         * PRIMASK. If PRIMASK is set, the IRQ stays pending and the
         * CPU just resumes at the instruction after WFI; the IRQ
         * fires later when PRIMASK clears. So `has_pending` always
         * clears `cpu_off` here, even though `arm_nvic_check_pending`
         * may decide not to take the exception yet. */
        if (cpu->cpu_off) {
            if (cpu->nvic) {
                arm_nvic_t *nvic = (arm_nvic_t *)cpu->nvic;
                if (nvic->has_pending) {
                    cpu->cpu_off = false;
                    arm_nvic_check_pending(nvic);
                    continue;
                }
            }
            if (cpu->event_queue) {
                cpu->cycles = cpu->event_queue->fire_cycle;
                continue;
            }
            break;
        }

        uint32_t pc = cpu->reg[ARM_PC];

        arm_sp_audit_check(cpu);

        /* Wild-jump detector: catch the first time PC lands in SRAM
         * (anywhere code shouldn't be executing — XN region on real HW
         * faults).  One-shot per-CPU; dump enough register state to
         * reverse-engineer which prior instruction redirected PC.
         * Enabled via ARM_WILD_TRAP. */
        if (getenv("ARM_WILD_TRAP") && pc >= cpu->sram_base && pc < cpu->sram_end) {
            if (!cpu->wild_trapped) {
                cpu->wild_trapped = 1;
                fprintf(stderr,
                        "[WILD JUMP cpu=%p cyc=%lld pc=0x%08x sp=0x%08x lr=0x%08x "
                        "r0=%08x r1=%08x r2=%08x r3=%08x r4=%08x r5=%08x r6=%08x r7=%08x "
                        "r8=%08x r9=%08x r10=%08x r11=%08x r12=%08x xpsr=%08x]\n",
                        (void*)cpu, (long long)cpu->cycles, pc, cpu->reg[ARM_SP], cpu->reg[ARM_LR],
                        cpu->reg[0], cpu->reg[1], cpu->reg[2], cpu->reg[3],
                        cpu->reg[4], cpu->reg[5], cpu->reg[6], cpu->reg[7],
                        cpu->reg[8], cpu->reg[9], cpu->reg[10], cpu->reg[11], cpu->reg[12],
                        cpu->xpsr);
            }
        }

        /* ROM utility traps */
        if (handle_fw_trap(cpu)) {
            cpu->instructions++;
            remaining--;
            continue;
        }
        if (pc < 0x100 && handle_rom_trap(cpu)) {
            cpu->instructions++;
            remaining--;
            continue;
        }

        /* Fetch first halfword */
        uint16_t hw1 = fetch16(cpu, pc);
        uint16_t top5 = hw1 >> 11;

        /* Periodic PC trace for debugging */
#ifdef DEBUG
        if ((cpu->instructions % 5000000) == 0)
            fprintf(stderr, "[%lldM] PC=0x%08x R0=0x%08x R1=0x%08x R2=0x%08x R3=0x%08x SP=0x%08x\n",
                    cpu->instructions/1000000, pc, cpu->reg[0], cpu->reg[1],
                    cpu->reg[2], cpu->reg[3], cpu->reg[ARM_SP]);
#endif

        /* IT block: check if current instruction should execute */
        int in_it = (cpu->it_state & 0xF) != 0;
        if (in_it) {
            int cond = (cpu->it_state >> 4) & 0xF;
            if (!condition_passed(cpu, cond)) {
                /* Condition failed — skip instruction */
                cpu->reg[ARM_PC] = pc + ((top5 >= 0x1D) ? 4 : 2);
                cpu->cycles += 1;
                cpu->it_state = (cpu->it_state & 0xE0) | ((cpu->it_state << 1) & 0x1F);
                cpu->instructions++;
                remaining--;
                continue;
            }
        }

        /*
         * In IT blocks, Thumb-16 data processing instructions do NOT update
         * condition flags (the implicit S suffix is suppressed). Exceptions
         * are CMP, CMN, and TST which always update flags.
         * Save flags before and restore after, unless it's a compare/test.
         */
        uint32_t saved_flags_it = 0;
        int it_suppress_flags = 0;
        if (in_it && top5 < 0x1D) {
            saved_flags_it = cpu->xpsr & (APSR_N | APSR_Z | APSR_C | APSR_V);
            it_suppress_flags = 1;
            /* Check if instruction is CMP/CMN/TST — these always update flags */
            if ((hw1 >> 13) == 1 && ((hw1 >> 11) & 3) == 1)
                it_suppress_flags = 0; /* CMP Rn, #imm8 */
            else if ((hw1 >> 10) == 0x10) {
                int dp_op = (hw1 >> 6) & 0xF;
                if (dp_op == 0x8 || dp_op == 0xA || dp_op == 0xB)
                    it_suppress_flags = 0; /* TST, CMP, CMN */
            } else if ((hw1 >> 10) == 0x11 && ((hw1 >> 8) & 3) == 1)
                it_suppress_flags = 0; /* CMP Rn, Rm (high) */
        }

        /* --- Computed goto dispatch for Thumb instruction decode --- */
#if defined(__GNUC__) || defined(__clang__)
        {
            static const void *thumb_dispatch[32] = {
                &&t16_lsl_imm,    /*  0 */ &&t16_lsr_imm,    /*  1 */
                &&t16_asr_imm,    /*  2 */ &&t16_addsub,     /*  3 */
                &&t16_mov_imm,    /*  4 */ &&t16_cmp_imm,    /*  5 */
                &&t16_add_imm8,   /*  6 */ &&t16_sub_imm8,   /*  7 */
                &&t16_dp_spec,    /*  8 */ &&t16_ldr_lit,    /*  9 */
                &&t16_ls_reg,     /* 10 */ &&t16_ls_reg,     /* 11 */
                &&t16_str_imm,    /* 12 */ &&t16_ldr_imm,    /* 13 */
                &&t16_strb_imm,   /* 14 */ &&t16_ldrb_imm,   /* 15 */
                &&t16_strh_imm,   /* 16 */ &&t16_ldrh_imm,   /* 17 */
                &&t16_str_sp,     /* 18 */ &&t16_ldr_sp,     /* 19 */
                &&t16_adr,        /* 20 */ &&t16_add_sp_imm, /* 21 */
                &&t16_misc,       /* 22 */ &&t16_misc,       /* 23 */
                &&t16_stm,        /* 24 */ &&t16_ldm,        /* 25 */
                &&t16_bcond,      /* 26 */ &&t16_bcond,      /* 27 */
                &&t16_b_uncond,   /* 28 */ &&t32_decode,     /* 29 */
                &&t32_decode,     /* 30 */ &&t32_decode,     /* 31 */
            };
            if (top5 < 29) {
                cpu->reg[ARM_PC] = pc + 2;
                cpu->cycles += 1;
            }
            goto *thumb_dispatch[top5];
        }
#else
        /* Fallback for non-GCC/Clang compilers */
        if (top5 < 29) {
            cpu->reg[ARM_PC] = pc + 2;
            cpu->cycles += 1;
        }
        switch (top5) {
            case 0: goto t16_lsl_imm; case 1: goto t16_lsr_imm;
            case 2: goto t16_asr_imm; case 3: goto t16_addsub;
            case 4: goto t16_mov_imm; case 5: goto t16_cmp_imm;
            case 6: goto t16_add_imm8; case 7: goto t16_sub_imm8;
            case 8: goto t16_dp_spec; case 9: goto t16_ldr_lit;
            case 10: case 11: goto t16_ls_reg;
            case 12: goto t16_str_imm; case 13: goto t16_ldr_imm;
            case 14: goto t16_strb_imm; case 15: goto t16_ldrb_imm;
            case 16: goto t16_strh_imm; case 17: goto t16_ldrh_imm;
            case 18: goto t16_str_sp; case 19: goto t16_ldr_sp;
            case 20: goto t16_adr; case 21: goto t16_add_sp_imm;
            case 22: case 23: goto t16_misc;
            case 24: goto t16_stm; case 25: goto t16_ldm;
            case 26: case 27: goto t16_bcond;
            case 28: goto t16_b_uncond;
            default: goto t32_decode;
        }
#endif

        /* --- 16-bit Thumb handlers --- */

        t16_lsl_imm: {
            int rd = hw1 & 7;
            int rm = (hw1 >> 3) & 7;
            int imm5 = (hw1 >> 6) & 0x1F;
            if (imm5 == 0) {
                cpu->reg[rd] = cpu->reg[rm];
                set_nz(cpu, cpu->reg[rd]);
            } else {
                int carry;
                cpu->reg[rd] = lsl_c(cpu->reg[rm], imm5, &carry);
                set_nzc(cpu, cpu->reg[rd], carry);
            }
            goto insn_done;
        }

        t16_lsr_imm: {
            int rd = hw1 & 7;
            int rm = (hw1 >> 3) & 7;
            int imm5 = (hw1 >> 6) & 0x1F;
            int carry;
            if (imm5 == 0) imm5 = 32;
            cpu->reg[rd] = lsr_c(cpu->reg[rm], imm5, &carry);
            set_nzc(cpu, cpu->reg[rd], carry);
            goto insn_done;
        }

        t16_asr_imm: {
            int rd = hw1 & 7;
            int rm = (hw1 >> 3) & 7;
            int imm5 = (hw1 >> 6) & 0x1F;
            int carry;
            if (imm5 == 0) imm5 = 32;
            cpu->reg[rd] = asr_c(cpu->reg[rm], imm5, &carry);
            set_nzc(cpu, cpu->reg[rd], carry);
            goto insn_done;
        }

        t16_addsub: {
            int sub_op = (hw1 >> 9) & 3;
            int rd = hw1 & 7;
            int rn = (hw1 >> 3) & 7;
            if (sub_op == 0) {
                /* ADD Rd, Rn, Rm */
                int rm = (hw1 >> 6) & 7;
                uint64_t result = (uint64_t)cpu->reg[rn] + cpu->reg[rm];
                cpu->reg[rd] = (uint32_t)result;
                set_add_flags(cpu, cpu->reg[rn], cpu->reg[rm], result);
            } else if (sub_op == 1) {
                /* SUB Rd, Rn, Rm — capture Rn before Rd write (Rd may alias Rn) */
                int rm = (hw1 >> 6) & 7;
                uint32_t old_rn = cpu->reg[rn], old_rm = cpu->reg[rm];
                uint64_t result = (uint64_t)old_rn - old_rm;
                cpu->reg[rd] = (uint32_t)result;
                set_sub_flags(cpu, old_rn, old_rm, result);
            } else if (sub_op == 2) {
                /* ADD Rd, Rn, #imm3 */
                uint32_t imm3 = (hw1 >> 6) & 7;
                uint32_t old_rn = cpu->reg[rn];
                uint64_t result = (uint64_t)old_rn + imm3;
                cpu->reg[rd] = (uint32_t)result;
                set_add_flags(cpu, old_rn, imm3, result);
            } else {
                /* SUB Rd, Rn, #imm3 — capture Rn before Rd write */
                uint32_t imm3 = (hw1 >> 6) & 7;
                uint32_t old_rn = cpu->reg[rn];
                uint64_t result = (uint64_t)old_rn - imm3;
                cpu->reg[rd] = (uint32_t)result;
                set_sub_flags(cpu, old_rn, imm3, result);
            }
            goto insn_done;
        }


        t16_mov_imm: {
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = hw1 & 0xFF;
            cpu->reg[rd] = imm8;
            set_nz(cpu, imm8);
            goto insn_done;
        }

        t16_cmp_imm: {
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = hw1 & 0xFF;
            uint64_t result = (uint64_t)cpu->reg[rd] - imm8;
            set_sub_flags(cpu, cpu->reg[rd], imm8, result);
            goto insn_done;
        }

        t16_add_imm8: {
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = hw1 & 0xFF;
            uint64_t result = (uint64_t)cpu->reg[rd] + imm8;
            set_add_flags(cpu, cpu->reg[rd], imm8, result);
            cpu->reg[rd] = (uint32_t)result;
            goto insn_done;
        }

        t16_sub_imm8: {
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = hw1 & 0xFF;
            uint64_t result = (uint64_t)cpu->reg[rd] - imm8;
            set_sub_flags(cpu, cpu->reg[rd], imm8, result);
            cpu->reg[rd] = (uint32_t)result;
            goto insn_done;
        }

        t16_dp_spec: {
            if ((hw1 >> 10) == 0x10) {
                /* Data processing (register) */
                int opcode = (hw1 >> 6) & 0xF;
                int rd = hw1 & 7;
                int rm = (hw1 >> 3) & 7;
                switch (opcode) {
                    case 0x0: { /* AND */
                        cpu->reg[rd] &= cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                    case 0x1: { /* EOR */
                        cpu->reg[rd] ^= cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                    case 0x2: { /* LSL */
                        int shift = cpu->reg[rm] & 0xFF;
                        if (shift >= 32) {
                            int carry = (shift == 32) ? (cpu->reg[rd] & 1) : 0;
                            cpu->reg[rd] = 0;
                            set_nzc(cpu, 0, carry);
                        } else if (shift > 0) {
                            int carry;
                            cpu->reg[rd] = lsl_c(cpu->reg[rd], shift, &carry);
                            set_nzc(cpu, cpu->reg[rd], carry);
                        } else {
                            set_nz(cpu, cpu->reg[rd]);
                        }
                        break;
                    }
                    case 0x3: { /* LSR */
                        int shift = cpu->reg[rm] & 0xFF;
                        if (shift >= 32) {
                            int carry = (shift == 32) ? ((cpu->reg[rd] >> 31) & 1) : 0;
                            cpu->reg[rd] = 0;
                            set_nzc(cpu, 0, carry);
                        } else if (shift > 0) {
                            int carry;
                            cpu->reg[rd] = lsr_c(cpu->reg[rd], shift, &carry);
                            set_nzc(cpu, cpu->reg[rd], carry);
                        } else {
                            set_nz(cpu, cpu->reg[rd]);
                        }
                        break;
                    }
                    case 0x4: { /* ASR */
                        int shift = cpu->reg[rm] & 0xFF;
                        if (shift >= 32) {
                            int carry = (cpu->reg[rd] >> 31) & 1;
                            cpu->reg[rd] = carry ? 0xFFFFFFFF : 0;
                            set_nzc(cpu, cpu->reg[rd], carry);
                        } else if (shift > 0) {
                            int carry;
                            cpu->reg[rd] = asr_c(cpu->reg[rd], shift, &carry);
                            set_nzc(cpu, cpu->reg[rd], carry);
                        } else {
                            set_nz(cpu, cpu->reg[rd]);
                        }
                        break;
                    }
                    case 0x5: { /* ADC */
                        int carry_in = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint64_t result = (uint64_t)cpu->reg[rd] + cpu->reg[rm] + carry_in;
                        set_add_flags(cpu, cpu->reg[rd], cpu->reg[rm] + carry_in, result);
                        cpu->reg[rd] = (uint32_t)result;
                        break;
                    }
                    case 0x6: { /* SBC */
                        int carry_in = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint64_t result = (uint64_t)cpu->reg[rd] - cpu->reg[rm] - (1 - carry_in);
                        uint32_t r32 = (uint32_t)result;
                        cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C | APSR_V);
                        if (r32 == 0) cpu->xpsr |= APSR_Z;
                        if (r32 & 0x80000000) cpu->xpsr |= APSR_N;
                        if ((uint64_t)cpu->reg[rd] >= (uint64_t)cpu->reg[rm] + (1 - carry_in))
                            cpu->xpsr |= APSR_C;
                        if (((cpu->reg[rd] ^ cpu->reg[rm]) & (cpu->reg[rd] ^ r32)) >> 31)
                            cpu->xpsr |= APSR_V;
                        cpu->reg[rd] = r32;
                        break;
                    }
                    case 0x7: { /* ROR */
                        int shift = cpu->reg[rm] & 0xFF;
                        if (shift > 0) {
                            int carry;
                            cpu->reg[rd] = ror_c(cpu->reg[rd], shift, &carry);
                            set_nzc(cpu, cpu->reg[rd], carry);
                        } else {
                            set_nz(cpu, cpu->reg[rd]);
                        }
                        break;
                    }
                    case 0x8: { /* TST */
                        uint32_t result = cpu->reg[rd] & cpu->reg[rm];
                        set_nz(cpu, result);
                        break;
                    }
                    case 0x9: { /* RSB (NEG) Rd, Rm, #0 */
                        uint64_t result = (uint64_t)0 - cpu->reg[rm];
                        cpu->reg[rd] = (uint32_t)result;
                        set_sub_flags(cpu, 0, cpu->reg[rm], result);
                        break;
                    }
                    case 0xA: { /* CMP */
                        uint64_t result = (uint64_t)cpu->reg[rd] - cpu->reg[rm];
                        set_sub_flags(cpu, cpu->reg[rd], cpu->reg[rm], result);
                        break;
                    }
                    case 0xB: { /* CMN */
                        uint64_t result = (uint64_t)cpu->reg[rd] + cpu->reg[rm];
                        set_add_flags(cpu, cpu->reg[rd], cpu->reg[rm], result);
                        break;
                    }
                    case 0xC: { /* ORR */
                        cpu->reg[rd] |= cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                    case 0xD: { /* MUL */
                        cpu->reg[rd] *= cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                    case 0xE: { /* BIC */
                        cpu->reg[rd] &= ~cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                    case 0xF: { /* MVN */
                        cpu->reg[rd] = ~cpu->reg[rm];
                        set_nz(cpu, cpu->reg[rd]);
                        break;
                    }
                }
            } else {
                /* Special data / branch-exchange.
                 *
                 * When PC is used as a source register here, the
                 * architecturally-visible R[15] is `ProcessorPC` =
                 * InstructionAddr + 4. Our `cpu->reg[15]` only updates
                 * at the END of an instruction, so reading it raw
                 * returns the current instruction's address. Read PC+4
                 * explicitly for any source = 15. Same rule applies to
                 * BX/BLX with Rm=PC (uncommon but architecturally
                 * defined). */
                int opcode = (hw1 >> 8) & 3;
                int d = ((hw1 >> 4) & 8) | (hw1 & 7); /* High bit from bit 7 */
                int m = (hw1 >> 3) & 0xF;

                uint32_t m_val = (m == ARM_PC) ? (pc + 4) : cpu->reg[m];
                uint32_t d_val = (d == ARM_PC) ? (pc + 4) : cpu->reg[d];

                switch (opcode) {
                    case 0: /* ADD Rd, Rm (high registers) */
                        cpu->reg[d] = d_val + m_val;
                        if (d == ARM_PC) cpu->reg[ARM_PC] &= ~1u;
                        break;
                    case 1: { /* CMP Rn, Rm (high registers) */
                        uint64_t result = (uint64_t)d_val - m_val;
                        set_sub_flags(cpu, d_val, m_val, result);
                        break;
                    }
                    case 2: /* MOV Rd, Rm (high registers) */
                        cpu->reg[d] = m_val;
                        if (d == ARM_PC) cpu->reg[ARM_PC] &= ~1u;
                        break;
                    case 3: /* BX / BLX */
                        if (hw1 & (1 << 7)) {
                            /* BLX Rm */
                            cpu->reg[ARM_LR] = (cpu->reg[ARM_PC]) | 1;
                            arm_sp_audit_push(cpu, cpu->reg[ARM_PC], m_val);
                            cpu->reg[ARM_PC] = m_val & ~1u;
                        } else {
                            /* BX Rm */
                            uint32_t target = m_val;
                            /* Check for exception return */
                            if ((target & 0xF0000000) == 0xF0000000) {
                                exception_return(cpu, target);
                            } else {
                                cpu->reg[ARM_PC] = target & ~1u;
                            }
                        }
                        break;
                }
            }
            goto insn_done;
        }

        t16_ldr_lit: {
            /* LDR Rt, [PC, #imm8*4] (PC-relative) */
            int rt = (hw1 >> 8) & 7;
            uint32_t imm8 = (hw1 & 0xFF) << 2;
            uint32_t addr = ((pc + 4) & ~3u) + imm8;
            cpu->reg[rt] = mem_read32_unaligned(cpu, addr);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ls_reg: {
            /* Load/store register offset */
            int opcode = (hw1 >> 9) & 7;
            int rm = (hw1 >> 6) & 7;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + cpu->reg[rm];
            switch (opcode) {
                case 0: /* STR */
                    mem_write32_unaligned(cpu, addr, cpu->reg[rt]);
                    break;
                case 1: /* STRH */
                    mem_write16_unaligned(cpu, addr, (uint16_t)cpu->reg[rt]);
                    break;
                case 2: /* STRB */
                    mem_write8(cpu, addr, (uint8_t)cpu->reg[rt]);
                    break;
                case 3: /* LDRSB */
                    cpu->reg[rt] = (uint32_t)(int32_t)(int8_t)mem_read8(cpu, addr);
                    break;
                case 4: /* LDR */
                    cpu->reg[rt] = mem_read32_unaligned(cpu, addr);
                    break;
                case 5: /* LDRH */
                    cpu->reg[rt] = mem_read16_unaligned(cpu, addr);
                    break;
                case 6: /* LDRB */
                    cpu->reg[rt] = mem_read8(cpu, addr);
                    break;
                case 7: /* LDRSH */
                    cpu->reg[rt] = (uint32_t)(int32_t)(int16_t)mem_read16_unaligned(cpu, addr);
                    break;
            }
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_str_imm: {
            /* STR Rt, [Rn, #imm5*4] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + (imm5 << 2);
            mem_write32_unaligned(cpu, addr, cpu->reg[rt]);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ldr_imm: {
            /* LDR Rt, [Rn, #imm5*4] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + (imm5 << 2);
            cpu->reg[rt] = mem_read32_unaligned(cpu, addr);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_strb_imm: {
            /* STRB Rt, [Rn, #imm5] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + imm5;
            mem_write8(cpu, addr, (uint8_t)cpu->reg[rt]);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ldrb_imm: {
            /* LDRB Rt, [Rn, #imm5] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + imm5;
            cpu->reg[rt] = mem_read8(cpu, addr);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_strh_imm: {
            /* STRH Rt, [Rn, #imm5*2] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + (imm5 << 1);
            mem_write16_unaligned(cpu, addr, (uint16_t)cpu->reg[rt]);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ldrh_imm: {
            /* LDRH Rt, [Rn, #imm5*2] */
            int imm5 = (hw1 >> 6) & 0x1F;
            int rn = (hw1 >> 3) & 7;
            int rt = hw1 & 7;
            uint32_t addr = cpu->reg[rn] + (imm5 << 1);
            cpu->reg[rt] = mem_read16_unaligned(cpu, addr);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_str_sp: {
            /* STR Rt, [SP, #imm8*4] */
            int rt = (hw1 >> 8) & 7;
            uint32_t imm8 = (hw1 & 0xFF) << 2;
            uint32_t addr = cpu->reg[ARM_SP] + imm8;
            mem_write32_unaligned(cpu, addr, cpu->reg[rt]);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ldr_sp: {
            /* LDR Rt, [SP, #imm8*4] */
            int rt = (hw1 >> 8) & 7;
            uint32_t imm8 = (hw1 & 0xFF) << 2;
            uint32_t addr = cpu->reg[ARM_SP] + imm8;
            cpu->reg[rt] = mem_read32_unaligned(cpu, addr);
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_adr: {
            /* ADD Rd, PC, #imm8*4 */
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = (hw1 & 0xFF) << 2;
            cpu->reg[rd] = ((pc + 4) & ~3u) + imm8;
            goto insn_done;
        }

        t16_add_sp_imm: {
            /* ADD Rd, SP, #imm8*4 */
            int rd = (hw1 >> 8) & 7;
            uint32_t imm8 = (hw1 & 0xFF) << 2;
            cpu->reg[rd] = cpu->reg[ARM_SP] + imm8;
            goto insn_done;
        }

        t16_misc: {
            /* Miscellaneous 16-bit */
            int sub_op = (hw1 >> 8) & 0xF;
            switch (sub_op) {
                case 0x0: {
                    /* ADD SP, #imm7*4 or SUB SP, #imm7*4 */
                    uint32_t imm7 = (hw1 & 0x7F) << 2;
                    if (hw1 & (1 << 7))
                        cpu->reg[ARM_SP] -= imm7;
                    else
                        cpu->reg[ARM_SP] += imm7;
                    break;
                }
                case 0x2: { /* SXTH, SXTB, UXTH, UXTB */
                    int rd = hw1 & 7;
                    int rm = (hw1 >> 3) & 7;
                    int op2 = (hw1 >> 6) & 3;
                    switch (op2) {
                        case 0: /* SXTH */
                            cpu->reg[rd] = (uint32_t)(int32_t)(int16_t)(uint16_t)cpu->reg[rm];
                            break;
                        case 1: /* SXTB */
                            cpu->reg[rd] = (uint32_t)(int32_t)(int8_t)(uint8_t)cpu->reg[rm];
                            break;
                        case 2: /* UXTH */
                            cpu->reg[rd] = cpu->reg[rm] & 0xFFFF;
                            break;
                        case 3: /* UXTB */
                            cpu->reg[rd] = cpu->reg[rm] & 0xFF;
                            break;
                    }
                    break;
                }
                case 0x1: /* CBZ */
                case 0x3: /* CBZ */
                case 0x9: /* CBNZ */
                case 0xB: { /* CBNZ */
                    int rn = hw1 & 7;
                    int i = (hw1 >> 9) & 1;
                    uint32_t imm5 = (hw1 >> 3) & 0x1F;
                    uint32_t offset = (i << 6) | (imm5 << 1);
                    int is_nonzero = (sub_op >> 3) & 1;
                    if (is_nonzero ? (cpu->reg[rn] != 0) : (cpu->reg[rn] == 0))
                        cpu->reg[ARM_PC] = pc + 4 + offset;
                    break;
                }
                case 0x4: /* PUSH */
                case 0x5: { /* PUSH with LR */
                    uint32_t reglist = hw1 & 0xFF;
                    int push_lr = (hw1 >> 8) & 1;
                    int count_regs = 0;
                    for (int i = 0; i < 8; i++) if (reglist & (1 << i)) count_regs++;
                    if (push_lr) count_regs++;
                    uint32_t sp = cpu->reg[ARM_SP] - count_regs * 4;
                    cpu->reg[ARM_SP] = sp;
                    uint32_t addr = sp;
                    for (int i = 0; i < 8; i++) {
                        if (reglist & (1 << i)) {
                            mem_write32(cpu, addr, cpu->reg[i]);
                            addr += 4;
                        }
                    }
                    if (push_lr) mem_write32(cpu, addr, cpu->reg[ARM_LR]);
                    cpu->cycles += count_regs;
                    break;
                }
                case 0x6: { /* CPS (CPSIE/CPSID) */
                    if (hw1 & (1 << 4)) {
                        cpu->primask = 1; /* CPSID i */
                    } else {
                        cpu->primask = 0; /* CPSIE i */
                        /* Re-evaluate any IRQs that pended while PRIMASK was 1.
                         * Without this, an IRQ that pends during a critical
                         * section is deferred until the next set_pending call
                         * — for the nrf54l15 GRTC tick that meant ~390 ms
                         * between handler runs instead of ~7.8 ms. */
                        if (cpu->nvic && ((arm_nvic_t *)cpu->nvic)->has_pending)
                            arm_nvic_check_pending((arm_nvic_t *)cpu->nvic);
                    }
                    break;
                }
                case 0xA: { /* REV, REV16, REVSH */
                    int rd = hw1 & 7;
                    int rm = (hw1 >> 3) & 7;
                    int op2 = (hw1 >> 6) & 3;
                    uint32_t val = cpu->reg[rm];
                    switch (op2) {
                        case 0: /* REV */
                            cpu->reg[rd] = ((val & 0xFF) << 24) | ((val & 0xFF00) << 8) |
                                           ((val >> 8) & 0xFF00) | ((val >> 24) & 0xFF);
                            break;
                        case 1: /* REV16 */
                            cpu->reg[rd] = ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8) |
                                           ((val & 0x00FF0000) << 8) | ((val & 0xFF000000) >> 8);
                            break;
                        case 3: /* REVSH */
                            cpu->reg[rd] = (uint32_t)(int32_t)(int16_t)(
                                ((val & 0xFF) << 8) | ((val >> 8) & 0xFF));
                            break;
                        default: break;
                    }
                    break;
                }
                case 0xC: /* POP */
                case 0xD: { /* POP with PC */
                    uint32_t reglist = hw1 & 0xFF;
                    int pop_pc = (hw1 >> 8) & 1;
                    uint32_t addr = cpu->reg[ARM_SP];
                    for (int i = 0; i < 8; i++) {
                        if (reglist & (1 << i)) {
                            cpu->reg[i] = mem_read32(cpu, addr);
                            addr += 4;
                        }
                    }
                    if (pop_pc) {
                        uint32_t target = mem_read32(cpu, addr);
                        addr += 4;
                        /* Update SP BEFORE exception_return so it reads
                           the exception frame from the correct address */
                        cpu->reg[ARM_SP] = addr;
                        if ((target & 0xF0000000) == 0xF0000000) {
                            exception_return(cpu, target);
                        } else {
                            cpu->reg[ARM_PC] = target & ~1u;
                        }
                    } else {
                        cpu->reg[ARM_SP] = addr;
                    }
                    cpu->cycles += 1;
                    break;
                }
                case 0xE: { /* BKPT */
                    /* Treat as NOP in emulation */
                    break;
                }
                case 0xF: { /* IT / hints */
                    if ((hw1 & 0xF) != 0) {
                        /* IT instruction: firstcond in bits[7:4], mask in bits[3:0] */
                        cpu->it_state = hw1 & 0xFF;
                    } else {
                        /* Hints: NOP, YIELD, WFE, WFI, SEV */
                        if ((hw1 & 0xFF) == 0x30) {
                            /* WFI */
                            cpu->cpu_off = true;
                        }
                        /* Other hints: NOP */
                    }
                    break;
                }
                default:
                    break;
            }
            goto insn_done;
        }

        t16_stm: {
            /* STMIA Rn!, {reglist} */
            int rn = (hw1 >> 8) & 7;
            uint32_t reglist = hw1 & 0xFF;
            uint32_t addr = cpu->reg[rn];
            for (int i = 0; i < 8; i++) {
                if (reglist & (1 << i)) {
                    mem_write32(cpu, addr, cpu->reg[i]);
                    addr += 4;
                }
            }
            cpu->reg[rn] = addr;
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_ldm: {
            /* LDMIA Rn!, {reglist} */
            int rn = (hw1 >> 8) & 7;
            uint32_t reglist = hw1 & 0xFF;
            uint32_t addr = cpu->reg[rn];
            for (int i = 0; i < 8; i++) {
                if (reglist & (1 << i)) {
                    cpu->reg[i] = mem_read32(cpu, addr);
                    addr += 4;
                }
            }
            /* Update Rn if not in reglist */
            if (!(reglist & (1 << rn)))
                cpu->reg[rn] = addr;
            cpu->cycles += 1;
            goto insn_done;
        }

        t16_bcond: {
            /* Conditional branch B<cond> */
            int cond = (hw1 >> 8) & 0xF;
            if (cond == 0xE) {
                /* UDF (undefined) - treat as NOP */
            } else if (cond == 0xF) {
                /* SVC */
                /* SVC #imm8 - typically not used in Contiki-NG, treat as NOP */
            } else {
                int8_t imm8 = (int8_t)(hw1 & 0xFF);
                if (condition_passed(cpu, cond))
                    cpu->reg[ARM_PC] = pc + 4 + (int32_t)imm8 * 2;
            }
            goto insn_done;
        }

        t16_b_uncond: {
            /* Unconditional branch B */
            int32_t imm11 = hw1 & 0x7FF;
            if (imm11 & 0x400) imm11 |= (int32_t)0xFFFFF800; /* sign extend */
            cpu->reg[ARM_PC] = pc + 4 + imm11 * 2;
            goto insn_done;
        }

        t32_decode: {
            /* 32-bit Thumb-2 instruction */
            uint16_t hw2 = fetch16(cpu, pc + 2);
            cpu->reg[ARM_PC] = pc + 4;
            cpu->cycles += 1;

            uint32_t insn32 = ((uint32_t)hw1 << 16) | hw2;
            int op1 = (hw1 >> 11) & 3;
            int op2_5 = (hw1 >> 4) & 0x7F;
            int op_hw2 = (hw2 >> 15) & 1;

            if (op1 == 1 && (op2_5 & 0x64) == 0x00) {
                /* Load/Store multiple, SRS, RFE */
                int W = (hw1 >> 5) & 1;
                int L = (hw1 >> 4) & 1;  /* 1=load, 0=store */
                int rn = hw1 & 0xF;
                uint32_t reglist = hw2;
                uint32_t addr = cpu->reg[rn];

                if (L) {
                    /* LDM.W — defer exception_return until after writeback */
                    uint32_t exc_ret = 0;
                    bool do_exc_ret = false;
                    for (int i = 0; i < 16; i++) {
                        if (reglist & (1 << i)) {
                            uint32_t val = mem_read32(cpu, addr);
                            if (i == ARM_PC) {
                                if ((val & 0xF0000000) == 0xF0000000) {
                                    do_exc_ret = true;
                                    exc_ret = val;
                                } else {
                                    cpu->reg[ARM_PC] = val & ~1u;
                                }
                            } else {
                                cpu->reg[i] = val;
                            }
                            addr += 4;
                        }
                    }
                    if (W && !(reglist & (1 << rn)))
                        cpu->reg[rn] = addr;
                    if (do_exc_ret)
                        exception_return(cpu, exc_ret);
                } else {
                    /* STM.W */
                    /* Check if decrement before (STMDB) */
                    int U = (hw1 >> 7) & 1; /* 1=increment, 0=decrement */
                    if (!U) {
                        int count_regs = 0;
                        for (int i = 0; i < 16; i++) if (reglist & (1 << i)) count_regs++;
                        addr -= count_regs * 4;
                        uint32_t base_addr = addr;
                        for (int i = 0; i < 16; i++) {
                            if (reglist & (1 << i)) {
                                mem_write32(cpu, addr, cpu->reg[i]);
                                addr += 4;
                            }
                        }
                        if (W) cpu->reg[rn] = base_addr;
                    } else {
                        for (int i = 0; i < 16; i++) {
                            if (reglist & (1 << i)) {
                                mem_write32(cpu, addr, cpu->reg[i]);
                                addr += 4;
                            }
                        }
                        if (W) cpu->reg[rn] = addr;
                    }
                }
                cpu->cycles += 1;
            } else if (op1 == 1 && (op2_5 & 0x64) == 0x04) {
                /* Load/store dual, exclusive, table branch */
                int op2_2 = (hw1 >> 4) & 3;
                int rn = hw1 & 0xF;
                int rt = (hw2 >> 12) & 0xF;
                int rt2 = (hw2 >> 8) & 0xF;
                uint32_t imm8 = (hw2 & 0xFF) << 2;
                int U = (hw1 >> 7) & 1;
                int P = (hw1 >> 8) & 1;
                int W = (hw1 >> 5) & 1;
                int L = (hw1 >> 4) & 1;

                if (U == 1 && (hw2 & 0x0FC0) == 0x0FC0) {
                    /* ARMv8-M LDAEX{B,H} / STLEX{B,H} family — load-acquire
                     * / store-release exclusive.  Encoding T1:
                     *
                     *   hw1 1110 1000 110L Rn   (L=1 load, L=0 store)
                     *   hw2 Rt 1111 110S 1111   (LDAEX{B/H},
                     *                            S=00 byte, 01 halfword, 10 word)
                     *   hw2 Rt 1111 110S Rd     (STLEX{B/H} — Rd at bits 3:0)
                     *
                     * The U bit (hw1[7]) distinguishes the acquire/release
                     * variants from plain LDREX/STREX (U=0).  hw2[5:4] selects
                     * byte (00), halfword (01), or word (10) — the discriminator
                     * is the `0x0FC0` mask matching the high half of the hw2
                     * low byte at positions [11:6] = 0xFC, with [5:4] varying.
                     *
                     * Used by Nordic's nrf_802154 atomic ops via
                     * `nrfx_atomic_u32_fetch_*` (word variant).  Byte/halfword
                     * variants haven't shown up in nrf54l15 firmware yet but
                     * are cheap to support.
                     *
                     * No exclusive-monitor model — single-CPU simulation,
                     * the store always succeeds (returns Rd = 0). */
                    int sz = (hw2 >> 4) & 0x3;   /* 00=B, 01=H, 10=W */
                    uint32_t addr = cpu->reg[rn];
                    if (L) {
                        switch (sz) {
                            case 0:  cpu->reg[rt] = mem_read8(cpu, addr);  break;
                            case 1:  cpu->reg[rt] = mem_read16(cpu, addr); break;
                            default: cpu->reg[rt] = mem_read32(cpu, addr); break;
                        }
                    } else {
                        int rd = hw2 & 0xF;
                        switch (sz) {
                            case 0:  mem_write8 (cpu, addr, cpu->reg[rt] & 0xFF);   break;
                            case 1:  mem_write16(cpu, addr, cpu->reg[rt] & 0xFFFF); break;
                            default: mem_write32(cpu, addr, cpu->reg[rt]);          break;
                        }
                        cpu->reg[rd] = 0;
                    }
                    cpu->cycles += 2;
                } else if ((hw2 & 0xFFE0) == 0xF000) {
                    /* TBB / TBH (Table Branch Byte / Halfword) */
                    int H = (hw2 >> 4) & 1;
                    int rm = hw2 & 0xF;
                    uint32_t base = (rn == 0xF) ? (pc + 4) : cpu->reg[rn];
                    if (H) {
                        /* TBH: halfword table */
                        uint16_t idx = mem_read16(cpu, base + cpu->reg[rm] * 2);
                        cpu->reg[ARM_PC] = pc + 4 + idx * 2;
                    } else {
                        /* TBB: byte table */
                        uint8_t idx = mem_read8(cpu, base + cpu->reg[rm]);
                        cpu->reg[ARM_PC] = pc + 4 + idx * 2;
                    }
                } else if (rn == 0xF && L) {
                    /* LDRD (literal): imm8 already holds the byte offset (field << 2). */
                    uint32_t base = (pc + 4) & ~3u;
                    uint32_t off  = imm8;
                    uint32_t addr = U ? base + off : base - off;
                    cpu->reg[rt]  = mem_read32(cpu, addr);
                    cpu->reg[rt2] = mem_read32(cpu, addr + 4);
                } else if (!P && !U && !L && (hw2 & 0x0F00) != 0x0F00) {
                    /* STREX T1: hw1 = 1110_1000_0100_Rn, hw2 = Rt_Rd_imm8.
                     * Distinguished from LDRD/STRD by hw1 bit 8 = 0.
                     * Distinguished from LDREX by L = 0. */
                    int rd = (hw2 >> 8) & 0xF;
                    uint32_t addr = cpu->reg[rn] + imm8;
                    mem_write32(cpu, addr, cpu->reg[rt]);
                    cpu->reg[rd] = 0; /* Always succeed */
                } else if (!P && !U && L && (hw2 & 0x0F00) == 0x0F00) {
                    /* LDREX T1: hw1 = 1110_1000_0101_Rn, hw2 = Rt_1111_imm8.
                     * Distinguished from LDRD/STRD by hw1 bit 8 = 0
                     * (bit 8 set → LDRD/LDREXB/LDREXH branches).
                     * hw2[11:8] == 0xF means no Rt2 register.
                     *
                     * The old decode used op2_2 == 0, but op2_2 = hw1[5:4]
                     * is 0b01 for LDREX (L=1), so the check always failed
                     * and we fell through to LDRD — which then loaded
                     * mem[rn+4] into rt2 = hw2[11:8] = 0xF = PC.  Nordic
                     * nrf_802154 uses LDREX in nrfx_atomic_u32_fetch_*;
                     * that LDRD-misdecode wrote a stale stack value into
                     * PC, sending the firmware into BSS data. */
                    uint32_t addr = cpu->reg[rn] + imm8;
                    cpu->reg[rt] = mem_read32(cpu, addr);
                } else {
                    /* LDRD / STRD (immediate): imm8 already holds the byte offset (field << 2). */
                    uint32_t offset = imm8;
                    uint32_t addr;
                    if (P) {
                        addr = U ? cpu->reg[rn] + offset : cpu->reg[rn] - offset;
                    } else {
                        addr = cpu->reg[rn];
                    }
                    if (L) {
                        cpu->reg[rt]  = mem_read32(cpu, addr);
                        cpu->reg[rt2] = mem_read32(cpu, addr + 4);
                    } else {
                        mem_write32(cpu, addr, cpu->reg[rt]);
                        mem_write32(cpu, addr + 4, cpu->reg[rt2]);
                    }
                    if (W) {
                        if (P)
                            cpu->reg[rn] = addr;
                        else
                            cpu->reg[rn] = U ? addr + offset : addr - offset;
                    }
                }
                cpu->cycles += 1;
            } else if (op1 == 1 && (op2_5 & 0x60) == 0x20) {
                /* Data processing (shifted register) */
                int op_dp = (hw1 >> 5) & 0xF;
                int S = (hw1 >> 4) & 1;
                int rn = hw1 & 0xF;
                int rd = (hw2 >> 8) & 0xF;
                int rm = hw2 & 0xF;
                int shift_type = (hw2 >> 4) & 3;
                int imm3 = (hw2 >> 12) & 7;
                int imm2 = (hw2 >> 6) & 3;
                int shift_n = (imm3 << 2) | imm2;

                uint32_t rm_val = cpu->reg[rm];
                int carry_out = (cpu->xpsr & APSR_C) ? 1 : 0;

                /* Apply shift */
                if (shift_n > 0 || shift_type != 0) {
                    if (shift_n == 0 && (shift_type == 1 || shift_type == 2)) shift_n = 32;
                    switch (shift_type) {
                        case 0: rm_val = lsl_c(rm_val, shift_n, &carry_out); break;
                        case 1: rm_val = lsr_c(rm_val, shift_n, &carry_out); break;
                        case 2: rm_val = asr_c(rm_val, shift_n, &carry_out); break;
                        case 3:
                            if (shift_n == 0) {
                                /* RRX */
                                int old_c = (cpu->xpsr & APSR_C) ? 1 : 0;
                                carry_out = rm_val & 1;
                                rm_val = (rm_val >> 1) | (old_c << 31);
                            } else {
                                rm_val = ror_c(rm_val, shift_n, &carry_out);
                            }
                            break;
                    }
                }

                uint32_t result;
                switch (op_dp) {
                    case 0x0: /* AND / TST */
                        result = cpu->reg[rn] & rm_val;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x1: /* BIC */
                        result = cpu->reg[rn] & ~rm_val;
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x2: /* ORR / MOV */
                        if (rn == 0xF) {
                            result = rm_val; /* MOV */
                        } else {
                            result = cpu->reg[rn] | rm_val; /* ORR */
                        }
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x3: /* ORN / MVN */
                        if (rn == 0xF) {
                            result = ~rm_val; /* MVN */
                        } else {
                            result = cpu->reg[rn] | ~rm_val; /* ORN */
                        }
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x4: /* EOR / TEQ */
                        result = cpu->reg[rn] ^ rm_val;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x8: { /* ADD / CMN */
                        uint64_t r64 = (uint64_t)cpu->reg[rn] + rm_val;
                        result = (uint32_t)r64;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_add_flags(cpu, cpu->reg[rn], rm_val, r64);
                        break;
                    }
                    case 0xA: { /* ADC */
                        int ci = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint64_t r64 = (uint64_t)cpu->reg[rn] + rm_val + ci;
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) set_add_flags(cpu, cpu->reg[rn], rm_val + ci, r64);
                        break;
                    }
                    case 0xB: { /* SBC — capture Rn before Rd write (Rd may alias Rn) */
                        int ci = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint32_t old_rn = cpu->reg[rn];
                        uint64_t r64 = (uint64_t)old_rn - rm_val - (1 - ci);
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) {
                            cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C | APSR_V);
                            if (result == 0) cpu->xpsr |= APSR_Z;
                            if (result & 0x80000000) cpu->xpsr |= APSR_N;
                            if ((uint64_t)old_rn >= (uint64_t)rm_val + (1 - ci))
                                cpu->xpsr |= APSR_C;
                            if (((old_rn ^ rm_val) & (old_rn ^ result)) >> 31)
                                cpu->xpsr |= APSR_V;
                        }
                        break;
                    }
                    case 0xD: { /* SUB / CMP — capture Rn before Rd write (Rd may alias Rn) */
                        uint32_t old_rn = cpu->reg[rn];
                        uint64_t r64 = (uint64_t)old_rn - rm_val;
                        result = (uint32_t)r64;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_sub_flags(cpu, old_rn, rm_val, r64);
                        break;
                    }
                    case 0xE: { /* RSB */
                        uint64_t r64 = (uint64_t)rm_val - cpu->reg[rn];
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) set_sub_flags(cpu, rm_val, cpu->reg[rn], r64);
                        break;
                    }
                    default:
                        result = 0;
                        break;
                }
                if (rd == ARM_PC) cpu->reg[ARM_PC] &= ~1u;
            } else if (op1 == 2 && (op2_5 & 0x20) == 0 && !op_hw2) {
                /* Data processing (modified 12-bit immediate) */
                int op_dp = (hw1 >> 5) & 0xF;
                int S = (hw1 >> 4) & 1;
                int rn = hw1 & 0xF;
                int rd = (hw2 >> 8) & 0xF;

                int i = (hw1 >> 10) & 1;
                int imm3 = (hw2 >> 12) & 7;
                int imm8 = hw2 & 0xFF;
                uint16_t imm12 = (i << 11) | (imm3 << 8) | imm8;

                int carry_out = (cpu->xpsr & APSR_C) ? 1 : 0;
                uint32_t imm_val = thumb_expand_imm_c(imm12, &carry_out, carry_out);

                uint32_t result;
                switch (op_dp) {
                    case 0x0: /* AND / TST */
                        result = cpu->reg[rn] & imm_val;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x1: /* BIC */
                        result = cpu->reg[rn] & ~imm_val;
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x2: /* ORR / MOV */
                        if (rn == 0xF) {
                            result = imm_val;
                        } else {
                            result = cpu->reg[rn] | imm_val;
                        }
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x3: /* ORN / MVN */
                        if (rn == 0xF) {
                            result = ~imm_val;
                        } else {
                            result = cpu->reg[rn] | ~imm_val;
                        }
                        cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x4: /* EOR / TEQ */
                        result = cpu->reg[rn] ^ imm_val;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_nzc(cpu, result, carry_out);
                        break;
                    case 0x8: { /* ADD / CMN */
                        uint64_t r64 = (uint64_t)cpu->reg[rn] + imm_val;
                        result = (uint32_t)r64;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_add_flags(cpu, cpu->reg[rn], imm_val, r64);
                        break;
                    }
                    case 0xA: { /* ADC */
                        int ci = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint64_t r64 = (uint64_t)cpu->reg[rn] + imm_val + ci;
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) set_add_flags(cpu, cpu->reg[rn], imm_val + ci, r64);
                        break;
                    }
                    case 0xB: { /* SBC — capture Rn before Rd write */
                        int ci = (cpu->xpsr & APSR_C) ? 1 : 0;
                        uint32_t old_rn = cpu->reg[rn];
                        uint64_t r64 = (uint64_t)old_rn - imm_val - (1 - ci);
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) {
                            cpu->xpsr &= ~(APSR_N | APSR_Z | APSR_C | APSR_V);
                            if (result == 0) cpu->xpsr |= APSR_Z;
                            if (result & 0x80000000) cpu->xpsr |= APSR_N;
                            if ((uint64_t)old_rn >= (uint64_t)imm_val + (1 - ci))
                                cpu->xpsr |= APSR_C;
                            if (((old_rn ^ imm_val) & (old_rn ^ result)) >> 31)
                                cpu->xpsr |= APSR_V;
                        }
                        break;
                    }
                    case 0xD: { /* SUB / CMP — capture Rn before Rd write */
                        uint32_t old_rn = cpu->reg[rn];
                        uint64_t r64 = (uint64_t)old_rn - imm_val;
                        result = (uint32_t)r64;
                        if (rd != 0xF) cpu->reg[rd] = result;
                        if (S) set_sub_flags(cpu, old_rn, imm_val, r64);
                        break;
                    }
                    case 0xE: { /* RSB */
                        uint64_t r64 = (uint64_t)imm_val - cpu->reg[rn];
                        result = (uint32_t)r64;
                        cpu->reg[rd] = result;
                        if (S) set_sub_flags(cpu, imm_val, cpu->reg[rn], r64);
                        break;
                    }
                    default:
                        result = 0;
                        break;
                }
            } else if (op1 == 2 && (op2_5 & 0x20) == 0x20 && !op_hw2) {
                /* Data processing (plain 16-bit immediate) */
                int op_imm = (hw1 >> 4) & 0x1F;
                int rn = hw1 & 0xF;
                int rd = (hw2 >> 8) & 0xF;

                int i = (hw1 >> 10) & 1;
                int imm3 = (hw2 >> 12) & 7;
                int imm8 = hw2 & 0xFF;
                /* 12-bit immediate for ADDW/SUBW: i:imm3:imm8 */
                uint32_t imm12 = (i << 11) | (imm3 << 8) | imm8;
                /* 16-bit immediate for MOVW/MOVT: imm4:i:imm3:imm8 */
                uint32_t imm16 = ((hw1 & 0xF) << 12) | imm12;

                if (op_imm == 0x04) {
                    /* MOVW Rd, #imm16 */
                    cpu->reg[rd] = imm16;
                } else if (op_imm == 0x0C) {
                    /* MOVT Rd, #imm16 */
                    cpu->reg[rd] = (cpu->reg[rd] & 0xFFFF) | (imm16 << 16);
                } else if ((op_imm & 0x1A) == 0x00) {
                    /* ADDW Rd, Rn, #imm12 / ADR Rd, label (T3 when Rn==PC).
                     * For Rn == PC the architecture mandates Align(PC, 4) as
                     * the base, where PC = InstructionAddr + 4. Without the
                     * alignment a non-word-aligned ADDW (e.g. dio_input's
                     * `addw ip, pc, #8` at an odd-halfword PC) returns a
                     * base off by 2 → the subsequent jump-table LDR reads
                     * a misaligned word → PC jumps to garbage and the CPU
                     * walks SRAM. */
                    if (rn == 0xF)
                        cpu->reg[rd] = ((pc + 4) & ~3u) + imm12;
                    else
                        cpu->reg[rd] = cpu->reg[rn] + imm12;
                } else if ((op_imm & 0x1A) == 0x0A) {
                    /* SUBW Rd, Rn, #imm12 (ADR T2 when Rn==PC). */
                    if (rn == 0xF)
                        cpu->reg[rd] = ((pc + 4) & ~3u) - imm12;
                    else
                        cpu->reg[rd] = cpu->reg[rn] - imm12;
                } else if ((op_imm & 0x1E) == 0x10) {
                    /* SSAT / SSAT16 — saturate signed (simplified). op=0x10,0x12. */
                    cpu->reg[rd] = cpu->reg[rn];
                } else if (op_imm == 0x14) {
                    /* SBFX — signed bit field extract. Encoding T1: op=10100. */
                    int lsb = imm3 * 4 + ((hw2 >> 6) & 3);
                    int widthm1 = hw2 & 0x1F;
                    int32_t val = (int32_t)(cpu->reg[rn] << (31 - lsb - widthm1));
                    cpu->reg[rd] = (uint32_t)(val >> (31 - widthm1));
                } else if (op_imm == 0x16) {
                    /* BFI / BFC — bit field insert (Rn != PC) / clear (Rn == PC).
                     * Encoding T1: op=10110.  Must be matched exactly — the
                     * earlier `(op_imm & 0x1C) == 0x18` form misrouted BFI
                     * (op=0x16, which `& 0x1C` gives 0x14) into the SBFX path,
                     * silently corrupting any 5-bit bitfield write (e.g.
                     * `uint8_t channel : 5` in nrf_802154's PIB struct). */
                    int lsb = imm3 * 4 + ((hw2 >> 6) & 3);
                    int msb = hw2 & 0x1F;
                    int width = msb - lsb + 1;
                    uint32_t mask = ((1u << width) - 1) << lsb;
                    if (rn == 0xF) {
                        cpu->reg[rd] &= ~mask;
                    } else {
                        cpu->reg[rd] = (cpu->reg[rd] & ~mask) |
                                       ((cpu->reg[rn] << lsb) & mask);
                    }
                } else if ((op_imm & 0x1E) == 0x18) {
                    /* USAT / USAT16 — saturate unsigned (simplified). op=0x18,0x1A. */
                    cpu->reg[rd] = cpu->reg[rn];
                } else if (op_imm == 0x1C) {
                    /* UBFX — unsigned bit field extract. */
                    int lsb = imm3 * 4 + ((hw2 >> 6) & 3);
                    int widthm1 = hw2 & 0x1F;
                    cpu->reg[rd] = (cpu->reg[rn] >> lsb) & ((1u << (widthm1 + 1)) - 1);
                } else if ((op_imm & 0x1A) == 0x02) {
                    /* ADR (subtract) */
                    cpu->reg[rd] = ((pc + 4) & ~3u) - imm12;
                }
            } else if (op1 == 2 && op_hw2) {
                /* Branches and misc control */
                int op_br = (hw1 >> 4) & 0x7F;
                int op2_br = (hw2 >> 12) & 7;

                if ((op2_br & 5) == 0 && (op_br & 0x38) != 0x38) {
                    /* B<cond>.W — conditional branch */
                    int cond = (hw1 >> 6) & 0xF;
                    int S = (hw1 >> 10) & 1;
                    int J1 = (hw2 >> 13) & 1;
                    int J2 = (hw2 >> 11) & 1;
                    int imm6 = hw1 & 0x3F;
                    int imm11 = hw2 & 0x7FF;
                    int32_t offset = (S << 20) | (J2 << 19) | (J1 << 18) |
                                     (imm6 << 12) | (imm11 << 1);
                    if (S) offset |= (int32_t)0xFFE00000;
                    if (condition_passed(cpu, cond))
                        cpu->reg[ARM_PC] = pc + 4 + offset;
                } else if ((op2_br & 5) == 5) {
                    /* BL — branch with link */
                    int S = (hw1 >> 10) & 1;
                    int J1 = (hw2 >> 13) & 1;
                    int J2 = (hw2 >> 11) & 1;
                    int imm10 = hw1 & 0x3FF;
                    int imm11 = hw2 & 0x7FF;
                    int I1 = !(J1 ^ S);
                    int I2 = !(J2 ^ S);
                    int32_t offset = (S << 24) | (I1 << 23) | (I2 << 22) |
                                     (imm10 << 12) | (imm11 << 1);
                    if (S) offset |= (int32_t)0xFE000000;
                    cpu->reg[ARM_LR] = (pc + 4) | 1;
                    cpu->reg[ARM_PC] = pc + 4 + offset;
                    arm_sp_audit_push(cpu, pc + 4, cpu->reg[ARM_PC]);
                } else if ((op2_br & 5) == 1) {
                    /* B.W — unconditional branch */
                    int S = (hw1 >> 10) & 1;
                    int J1 = (hw2 >> 13) & 1;
                    int J2 = (hw2 >> 11) & 1;
                    int imm10 = hw1 & 0x3FF;
                    int imm11 = hw2 & 0x7FF;
                    int I1 = !(J1 ^ S);
                    int I2 = !(J2 ^ S);
                    int32_t offset = (S << 24) | (I1 << 23) | (I2 << 22) |
                                     (imm10 << 12) | (imm11 << 1);
                    if (S) offset |= (int32_t)0xFE000000;
                    cpu->reg[ARM_PC] = pc + 4 + offset;
                } else if ((op_br & 0x38) == 0x38 && (op2_br & 5) == 0) {
                    /* MSR, MRS, hints, barriers, misc */
                    int op_misc = (hw2 >> 8) & 0xFF;
                    if (op_br == 0x38 && (op_misc & 0xF0) == 0xF0) {
                        /* DSB, DMB, ISB */
                        /* Barriers - NOP for single-core emulator */
                    } else if ((op_br & 0x7E) == 0x3E) {
                        /* MRS */
                        int rd = (hw2 >> 8) & 0xF;
                        int sysm = hw2 & 0xFF;
                        switch (sysm) {
                            case 0: cpu->reg[rd] = cpu->xpsr; break;
                            case 1: cpu->reg[rd] = cpu->xpsr & 0x1FF; break; /* IPSR */
                            case 2: cpu->reg[rd] = cpu->xpsr & (1u << 24); break; /* EPSR */
                            case 5: case 6: case 7:
                                cpu->reg[rd] = cpu->xpsr; break; /* Combined */
                            case 8: cpu->reg[rd] = cpu->msp; break;
                            case 9: cpu->reg[rd] = cpu->psp; break;
                            case 16: cpu->reg[rd] = cpu->primask; break;
                            case 17: cpu->reg[rd] = cpu->basepri; break;
                            case 19: cpu->reg[rd] = cpu->faultmask; break;
                            case 20: cpu->reg[rd] = cpu->use_psp ? 2 : 0; break; /* CONTROL */
                            default: cpu->reg[rd] = 0; break;
                        }
                    } else if ((op_br & 0x7E) == 0x38 && (op_misc & 0xF0) != 0xF0) {
                        /* MSR */
                        int rn = hw1 & 0xF;
                        int sysm = hw2 & 0xFF;
                        uint32_t val = cpu->reg[rn];
                        switch (sysm) {
                            case 0: /* APSR */
                                cpu->xpsr = (cpu->xpsr & 0x0FFFFFFF) | (val & 0xF0000000);
                                break;
                            case 8: cpu->msp = val; cpu->reg[ARM_SP] = val; break;
                            case 9: cpu->psp = val; break;
                            case 16:
                                cpu->primask = val & 1;
                                /* Like CPSIE i above — when PRIMASK clears,
                                 * re-evaluate pending IRQs that may have been
                                 * deferred during the critical section. */
                                if (cpu->primask == 0 && cpu->nvic &&
                                    ((arm_nvic_t *)cpu->nvic)->has_pending)
                                    arm_nvic_check_pending((arm_nvic_t *)cpu->nvic);
                                break;
                            case 17: cpu->basepri = val & 0xFF; break;
                            case 19: cpu->faultmask = val & 1; break;
                            case 20: /* CONTROL */
                                cpu->use_psp = (val & 2) != 0;
                                break;
                            default: break;
                        }
                    }
                }
            } else if (op1 == 3 && (op2_5 & 0x71) == 0x00) {
                /* Store single: STR.W, STRB.W, STRH.W (12-bit immediate) */
                int size = (hw1 >> 5) & 3; /* 0=STRB, 1=STRH, 2=STR */
                int rn = hw1 & 0xF;
                int rt = (hw2 >> 12) & 0xF;
                if (hw1 & (1 << 7)) {
                    /* 12-bit unsigned offset (T2/T3 encoding: hw1 bit 7 = 1) */
                    uint32_t imm12 = hw2 & 0xFFF;
                    uint32_t addr = cpu->reg[rn] + imm12;
                    switch (size) {
                        case 0: mem_write8(cpu, addr, (uint8_t)cpu->reg[rt]); break;
                        case 1: mem_write16_unaligned(cpu, addr, (uint16_t)cpu->reg[rt]); break;
                        case 2: mem_write32_unaligned(cpu, addr, cpu->reg[rt]); break;
                    }
                } else {
                    /* 8-bit immediate with pre/post index, or register */
                    int P = (hw2 >> 10) & 1;
                    int U = (hw2 >> 9) & 1;
                    int W = (hw2 >> 8) & 1;

                    if (P == 0 && U == 0 && W == 0) {
                        /* Register offset: STR Rt, [Rn, Rm, LSL #imm2] */
                        int rm = hw2 & 0xF;
                        int imm2 = (hw2 >> 4) & 3;
                        uint32_t addr = cpu->reg[rn] + (cpu->reg[rm] << imm2);
                        switch (size) {
                            case 0: mem_write8(cpu, addr, (uint8_t)cpu->reg[rt]); break;
                            case 1: mem_write16_unaligned(cpu, addr, (uint16_t)cpu->reg[rt]); break;
                            case 2: mem_write32_unaligned(cpu, addr, cpu->reg[rt]); break;
                        }
                    } else {
                        uint32_t imm8 = hw2 & 0xFF;
                        uint32_t addr;
                        if (P) {
                            addr = U ? cpu->reg[rn] + imm8 : cpu->reg[rn] - imm8;
                        } else {
                            addr = cpu->reg[rn];
                        }
                        switch (size) {
                            case 0: mem_write8(cpu, addr, (uint8_t)cpu->reg[rt]); break;
                            case 1: mem_write16_unaligned(cpu, addr, (uint16_t)cpu->reg[rt]); break;
                            case 2: mem_write32_unaligned(cpu, addr, cpu->reg[rt]); break;
                        }
                        if (W) {
                            if (P)
                                cpu->reg[rn] = addr;
                            else
                                cpu->reg[rn] = U ? addr + imm8 : addr - imm8;
                        }
                    }
                }
                cpu->cycles += 1;
            } else if (op1 == 3 && (op2_5 & 0x61) == 0x01) {
                /* Load byte/halfword: LDR.W, LDRB.W, LDRH.W, LDRSB.W, LDRSH.W
                   Mask 0x61 matches bits[6:5]=size, bit[0]=load; excludes bit[4]
                   (hw1[8]) which is the sign-extend bit, handled inside. */
                int size = (hw1 >> 5) & 3;
                int sign_extend = (hw1 >> 8) & 1;
                int rn = hw1 & 0xF;
                int rt = (hw2 >> 12) & 0xF;

                uint32_t addr;
                if (rn == 0xF) {
                    /* PC-relative (literal) */
                    uint32_t imm12 = hw2 & 0xFFF;
                    int U = (hw1 >> 7) & 1;
                    uint32_t base = (pc + 4) & ~3u;
                    addr = U ? base + imm12 : base - imm12;
                } else if (hw1 & (1 << 7)) {
                    /* 12-bit unsigned offset (T2/T3 encoding: hw1 bit 7 = 1) */
                    uint32_t imm12 = hw2 & 0xFFF;
                    addr = cpu->reg[rn] + imm12;
                } else if ((hw2 & 0xFC0) == 0) {
                    /* Register offset */
                    int rm = hw2 & 0xF;
                    int imm2 = (hw2 >> 4) & 3;
                    addr = cpu->reg[rn] + (cpu->reg[rm] << imm2);
                } else {
                    /* 8-bit immediate with P/U/W */
                    int P = (hw2 >> 10) & 1;
                    int U = (hw2 >> 9) & 1;
                    int W = (hw2 >> 8) & 1;
                    uint32_t imm8 = hw2 & 0xFF;
                    if (P) {
                        addr = U ? cpu->reg[rn] + imm8 : cpu->reg[rn] - imm8;
                    } else {
                        addr = cpu->reg[rn];
                    }
                    if (W) {
                        if (P)
                            cpu->reg[rn] = addr;
                        else
                            cpu->reg[rn] = U ? addr + imm8 : addr - imm8;
                    }
                }

                uint32_t val;
                switch (size) {
                    case 0: /* Byte */
                        val = mem_read8(cpu, addr);
                        if (sign_extend) val = (uint32_t)(int32_t)(int8_t)val;
                        break;
                    case 1: /* Halfword */
                        val = mem_read16_unaligned(cpu, addr);
                        if (sign_extend) val = (uint32_t)(int32_t)(int16_t)val;
                        break;
                    case 2: /* Word */
                    default:
                        val = mem_read32_unaligned(cpu, addr);
                        break;
                }
                if (rt == ARM_PC) {
                    if ((val & 0xF0000000) == 0xF0000000)
                        exception_return(cpu, val);
                    else
                        cpu->reg[ARM_PC] = val & ~1u;
                } else {
                    cpu->reg[rt] = val;
                }
                cpu->cycles += 1;
            } else if (op1 == 3 && (op2_5 & 0x71) == 0x10) {
                /* Store single: STR.W 12-bit offset alternate encoding */
                int rn = hw1 & 0xF;
                int rt = (hw2 >> 12) & 0xF;
                uint32_t imm12 = hw2 & 0xFFF;
                uint32_t addr = cpu->reg[rn] + imm12;
                mem_write32(cpu, addr, cpu->reg[rt]);
                cpu->cycles += 1;
            } else if (op1 == 3 && (op2_5 & 0x71) == 0x11) {
                /* Load word: LDR.W 12-bit offset alternate encoding */
                int rn = hw1 & 0xF;
                int rt = (hw2 >> 12) & 0xF;
                uint32_t imm12 = hw2 & 0xFFF;
                uint32_t addr;
                if (rn == 0xF) {
                    uint32_t base = (pc + 4) & ~3u;
                    int U = (hw1 >> 7) & 1;
                    addr = U ? base + imm12 : base - imm12;
                } else {
                    addr = cpu->reg[rn] + imm12;
                }
                uint32_t val = mem_read32(cpu, addr);
                if (rt == ARM_PC) {
                    if ((val & 0xF0000000) == 0xF0000000)
                        exception_return(cpu, val);
                    else
                        cpu->reg[ARM_PC] = val & ~1u;
                } else {
                    cpu->reg[rt] = val;
                }
                cpu->cycles += 1;
            } else if (op1 == 3 && (op2_5 & 0x60) == 0x20) {
                /* Data processing (register): shifts, multiply, divide, misc */
                int op_misc = (hw1 >> 4) & 0xF;
                int rn = hw1 & 0xF;
                int rd = (hw2 >> 8) & 0xF;
                int rm = hw2 & 0xF;
                int ra = (hw2 >> 12) & 0xF;
                int op2_misc = (hw2 >> 4) & 0xF;

                if (!(hw1 & 0x100) && op_misc < 8 && (op2_misc & 0x8) == 0) {
                    /* Register-register shifts: LSL, LSR, ASR, ROR (hw1=0xFA0.-0xFA7.)
                       hw2[7:4] must be 0000 to distinguish from extend instructions */
                    int shift_type = (hw1 >> 5) & 3; /* 0=LSL,1=LSR,2=ASR,3=ROR */
                    int S = (hw1 >> 4) & 1;
                    uint32_t val = cpu->reg[rn];
                    uint32_t shift_n = cpu->reg[rm] & 0xFF;
                    uint32_t result;
                    int carry = (cpu->xpsr >> 29) & 1; /* current C */

                    switch (shift_type) {
                        case 0: /* LSL */
                            if (shift_n == 0) { result = val; }
                            else if (shift_n < 32) { carry = (val >> (32 - shift_n)) & 1; result = val << shift_n; }
                            else if (shift_n == 32) { carry = val & 1; result = 0; }
                            else { carry = 0; result = 0; }
                            break;
                        case 1: /* LSR */
                            if (shift_n == 0) { result = val; }
                            else if (shift_n < 32) { carry = (val >> (shift_n - 1)) & 1; result = val >> shift_n; }
                            else if (shift_n == 32) { carry = (val >> 31) & 1; result = 0; }
                            else { carry = 0; result = 0; }
                            break;
                        case 2: /* ASR */
                            if (shift_n == 0) { result = val; }
                            else if (shift_n < 32) { carry = (val >> (shift_n - 1)) & 1; result = (uint32_t)((int32_t)val >> shift_n); }
                            else { carry = (val >> 31) & 1; result = (uint32_t)((int32_t)val >> 31); }
                            break;
                        case 3: /* ROR */
                            if (shift_n == 0) { result = val; }
                            else { shift_n &= 31; if (shift_n == 0) { carry = (val >> 31) & 1; result = val; } else { carry = (val >> (shift_n - 1)) & 1; result = (val >> shift_n) | (val << (32 - shift_n)); } }
                            break;
                        default: result = val; break;
                    }
                    cpu->reg[rd] = result;
                    if (S) {
                        /* Update N, Z, C flags (V unchanged) */
                        cpu->xpsr = (cpu->xpsr & ~(APSR_N | APSR_Z | APSR_C))
                                  | (result & 0x80000000 ? APSR_N : 0)
                                  | (result == 0 ? APSR_Z : 0)
                                  | (carry ? APSR_C : 0);
                    }
                } else if (!(hw1 & 0x100) && op_misc < 6 && (op2_misc & 0x8)) {
                    /* Signed/unsigned extend (and add): SXTH, UXTH, SXTB, UXTB, etc.
                       hw2[7:4] has bit 7 set, rotation in hw2[5:4] */
                    int rot = ((hw2 >> 4) & 3) * 8;
                    uint32_t rm_val = cpu->reg[rm];
                    if (rot) rm_val = (rm_val >> rot) | (rm_val << (32 - rot));
                    uint32_t result;
                    switch (op_misc) {
                        case 0: /* SXTH / SXTAH */
                            result = (uint32_t)(int32_t)(int16_t)(uint16_t)rm_val;
                            if (rn != 0xF) result += cpu->reg[rn];
                            break;
                        case 1: /* UXTH / UXTAH */
                            result = rm_val & 0xFFFF;
                            if (rn != 0xF) result += cpu->reg[rn];
                            break;
                        case 2: /* SXTB16 / SXTAB16 (M4+, skip for M3) */
                        case 3: /* UXTB16 / UXTAB16 (M4+, skip for M3) */
                            result = rm_val; /* Not supported on M3 */
                            break;
                        case 4: /* SXTB / SXTAB */
                            result = (uint32_t)(int32_t)(int8_t)(uint8_t)rm_val;
                            if (rn != 0xF) result += cpu->reg[rn];
                            break;
                        case 5: /* UXTB / UXTAB */
                            result = rm_val & 0xFF;
                            if (rn != 0xF) result += cpu->reg[rn];
                            break;
                        default:
                            result = rm_val;
                            break;
                    }
                    cpu->reg[rd] = result;
                } else if (op_misc == 0) {
                    /* MUL / MLA / MLS (hw1=0xFB0.) */
                    if (ra == 0xF) {
                        /* MUL Rd, Rn, Rm */
                        cpu->reg[rd] = cpu->reg[rn] * cpu->reg[rm];
                    } else if (op2_misc == 0) {
                        /* MLA Rd, Rn, Rm, Ra */
                        cpu->reg[rd] = cpu->reg[rn] * cpu->reg[rm] + cpu->reg[ra];
                    } else if (op2_misc == 1) {
                        /* MLS Rd, Rn, Rm, Ra */
                        cpu->reg[rd] = cpu->reg[ra] - cpu->reg[rn] * cpu->reg[rm];
                    }
                } else if (op_misc == 1 && (hw1 & 0x100)) {
                    /* M4 DSP — halfword multiply (hw1=0xFB1.).
                     * SMUL{B,T}{B,T}: ra=0xF, no accumulator.
                     * SMLA{B,T}{B,T}: ra<0xF, 32-bit accumulator (sets Q on overflow).
                     * op2_misc layout: 00 N M  — N selects top half of Rn, M of Rm. */
                    if ((op2_misc & 0xC) == 0) {
                        int n_half = (op2_misc >> 1) & 1;
                        int m_half = op2_misc & 1;
                        int16_t a = (int16_t)(n_half ? (cpu->reg[rn] >> 16)
                                                     : (cpu->reg[rn] & 0xFFFF));
                        int16_t b = (int16_t)(m_half ? (cpu->reg[rm] >> 16)
                                                     : (cpu->reg[rm] & 0xFFFF));
                        int32_t prod = (int32_t)a * (int32_t)b;
                        if (ra == 0xF) {
                            cpu->reg[rd] = (uint32_t)prod;
                        } else {
                            int64_t sum = (int64_t)(int32_t)cpu->reg[ra] + (int64_t)prod;
                            cpu->reg[rd] = (uint32_t)(int32_t)sum;
                            if (sum != (int64_t)(int32_t)sum)
                                cpu->xpsr |= APSR_Q;
                        }
                    }
                } else if (op_misc == 3 && (hw1 & 0x100)) {
                    /* M4 DSP — SMULWB/WT and SMLAWB/WT (hw1=0xFB3.).
                     * 32x16 → middle 32 bits of 48-bit product (bits [47:16]).
                     * op2_misc layout: 000 M — M selects top half of Rm. */
                    if ((op2_misc & 0xE) == 0) {
                        int m_half = op2_misc & 1;
                        int16_t b = (int16_t)(m_half ? (cpu->reg[rm] >> 16)
                                                     : (cpu->reg[rm] & 0xFFFF));
                        int64_t prod48 = (int64_t)(int32_t)cpu->reg[rn] * (int64_t)b;
                        int32_t result = (int32_t)(prod48 >> 16);
                        if (ra == 0xF) {
                            cpu->reg[rd] = (uint32_t)result;
                        } else {
                            int64_t sum = (int64_t)(int32_t)cpu->reg[ra] + (int64_t)result;
                            cpu->reg[rd] = (uint32_t)(int32_t)sum;
                            if (sum != (int64_t)(int32_t)sum)
                                cpu->xpsr |= APSR_Q;
                        }
                    }
                } else if (op_misc == 8) {
                    /* SMULL RdLo, RdHi, Rn, Rm (hw1=0xFB8.)
                       hw2[15:12]=RdLo(=ra), hw2[11:8]=RdHi(=rd) */
                    int64_t result = (int64_t)(int32_t)cpu->reg[rn] *
                                     (int64_t)(int32_t)cpu->reg[rm];
                    cpu->reg[ra] = (uint32_t)result;          /* RdLo */
                    cpu->reg[rd] = (uint32_t)(result >> 32);  /* RdHi */
                } else if (op_misc == 9) {
                    if (hw1 & 0x100) {
                        /* SDIV Rd, Rn, Rm (hw1=0xFB9.) */
                        int32_t denom = (int32_t)cpu->reg[rm];
                        if (denom == 0)
                            cpu->reg[rd] = 0;
                        else
                            cpu->reg[rd] = (uint32_t)((int32_t)cpu->reg[rn] / denom);
                    } else {
                        /* Misc register ops (hw1=0xFA9.): REV, REV16, RBIT, REVSH */
                        if (op2_misc == 8) {
                            /* REV */
                            uint32_t val = cpu->reg[rm];
                            cpu->reg[rd] = ((val & 0xFF) << 24) | ((val & 0xFF00) << 8) |
                                           ((val >> 8) & 0xFF00) | ((val >> 24) & 0xFF);
                        } else if (op2_misc == 9) {
                            /* REV16 */
                            uint32_t val = cpu->reg[rm];
                            cpu->reg[rd] = ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8) |
                                           ((val & 0x00FF0000) << 8) | ((val & 0xFF000000) >> 8);
                        } else if (op2_misc == 0xA) {
                            /* RBIT */
                            uint32_t val = cpu->reg[rm];
                            uint32_t result = 0;
                            for (int i = 0; i < 32; i++)
                                if (val & (1u << i)) result |= (1u << (31 - i));
                            cpu->reg[rd] = result;
                        } else if (op2_misc == 0xB) {
                            /* REVSH */
                            uint32_t val = cpu->reg[rm];
                            cpu->reg[rd] = (uint32_t)(int32_t)(int16_t)
                                           (uint16_t)(((val & 0xFF) << 8) | ((val >> 8) & 0xFF));
                        }
                    }
                } else if (op_misc == 0xA) {
                    if (hw1 & 0x100) {
                        /* UMULL RdLo, RdHi, Rn, Rm (hw1=0xFBA.)
                           hw2[15:12]=RdLo(=ra), hw2[11:8]=RdHi(=rd) */
                        uint64_t result = (uint64_t)cpu->reg[rn] * (uint64_t)cpu->reg[rm];
                        cpu->reg[ra] = (uint32_t)result;          /* RdLo */
                        cpu->reg[rd] = (uint32_t)(result >> 32);  /* RdHi */
                    }
                    /* hw1=0xFAA. is reserved/unused on Cortex-M3 */
                } else if (op_misc == 0xB) {
                    if (op2_misc == 8) {
                        /* CLZ (Count Leading Zeros) */
                        uint32_t val = cpu->reg[rm];
                        int count = 0;
                        if (val == 0) { count = 32; }
                        else { while (!(val & 0x80000000)) { count++; val <<= 1; } }
                        cpu->reg[rd] = count;
                    } else if (hw1 & 0x100) {
                        /* UDIV Rd, Rn, Rm (hw1=0xFBB.) */
                        if (cpu->reg[rm] == 0)
                            cpu->reg[rd] = 0;
                        else
                            cpu->reg[rd] = cpu->reg[rn] / cpu->reg[rm];
                    }
                } else if (op_misc == 0xC) {
                    /* SMLAL family (hw1=0xFBC.), discriminated by op2_misc.
                       hw2[15:12]=RdLo(=ra), hw2[11:8]=RdHi(=rd). */
                    if (op2_misc == 0) {
                        /* SMLAL RdLo, RdHi, Rn, Rm — 32x32 + 64-bit accumulator */
                        int64_t acc = ((int64_t)(uint32_t)cpu->reg[ra]) |
                                      ((int64_t)(int32_t)cpu->reg[rd] << 32);
                        acc += (int64_t)(int32_t)cpu->reg[rn] * (int64_t)(int32_t)cpu->reg[rm];
                        cpu->reg[ra] = (uint32_t)acc;          /* RdLo */
                        cpu->reg[rd] = (uint32_t)(acc >> 32);  /* RdHi */
                    } else if ((op2_misc & 0xC) == 0x8) {
                        /* M4 DSP — SMLALBB/BT/TB/TT.
                         * op2_misc layout: 10 N M — N selects top half of Rn, M of Rm.
                         * 16x16 sign-extended to 64, added to {RdHi:RdLo}. */
                        int n_half = (op2_misc >> 1) & 1;
                        int m_half = op2_misc & 1;
                        int16_t a = (int16_t)(n_half ? (cpu->reg[rn] >> 16)
                                                     : (cpu->reg[rn] & 0xFFFF));
                        int16_t b = (int16_t)(m_half ? (cpu->reg[rm] >> 16)
                                                     : (cpu->reg[rm] & 0xFFFF));
                        int64_t prod = (int64_t)((int32_t)a * (int32_t)b);
                        int64_t acc = ((int64_t)(uint32_t)cpu->reg[ra]) |
                                      ((int64_t)(int32_t)cpu->reg[rd] << 32);
                        acc += prod;
                        cpu->reg[ra] = (uint32_t)acc;
                        cpu->reg[rd] = (uint32_t)(acc >> 32);
                    }
                } else if (op_misc == 0xE) {
                    if (op2_misc == 0x6) {
                        /* UMAAL RdLo, RdHi, Rn, Rm (hw1=0xFBE., hw2[7:4]=0110)
                           {RdHi:RdLo} = Rn*Rm + RdLo + RdHi  (all unsigned 32-bit addends) */
                        uint64_t result = (uint64_t)cpu->reg[rn] * (uint64_t)cpu->reg[rm]
                                        + (uint64_t)cpu->reg[ra]
                                        + (uint64_t)cpu->reg[rd];
                        cpu->reg[ra] = (uint32_t)result;          /* RdLo */
                        cpu->reg[rd] = (uint32_t)(result >> 32);  /* RdHi */
                    } else {
                        /* UMLAL RdLo, RdHi, Rn, Rm (hw1=0xFBE., hw2[7:4]=0000)
                           {RdHi:RdLo} += Rn*Rm */
                        uint64_t acc = ((uint64_t)cpu->reg[ra]) |
                                       ((uint64_t)cpu->reg[rd] << 32);
                        acc += (uint64_t)cpu->reg[rn] * (uint64_t)cpu->reg[rm];
                        cpu->reg[ra] = (uint32_t)acc;          /* RdLo */
                        cpu->reg[rd] = (uint32_t)(acc >> 32);  /* RdHi */
                    }
                }
            } else if ((hw1 & 0xEC00) == 0xEC00) {
                /* Cortex-M4F single-precision VFP. Real implementations
                 * live in arm_vfp.c — `arm_vfp_step` returns true if it
                 * handled the instruction, false otherwise. On false we
                 * loudly fault so unhandled FP arithmetic doesn't
                 * silently produce wrong results downstream. */
                if (!arm_vfp_step(cpu, hw1, hw2)) {
                    fprintf(stderr, "ARM: unhandled VFP insn at PC=0x%08x: %04x %04x\n",
                            pc, hw1, hw2);
                }
                (void)insn32;
            } else {
                /* Unhandled 32-bit instruction */
                fprintf(stderr, "ARM: unhandled 32-bit insn at PC=0x%08x: %04x %04x\n",
                        pc, hw1, hw2);
                (void)insn32;
            }
            goto insn_done;
        }

        insn_done:
        /* Restore flags for Thumb-16 in IT block (suppress implicit S) */
        if (it_suppress_flags) {
            cpu->xpsr = (cpu->xpsr & ~(APSR_N | APSR_Z | APSR_C | APSR_V))
                      | saved_flags_it;
        }

        /* Advance IT state after each executed instruction */
        if (in_it) {
            cpu->it_state = (cpu->it_state & 0xE0) | ((cpu->it_state << 1) & 0x1F);
        }

        cpu->instructions++;
        remaining--;

        /* Service any IRQ that became pending mid-instruction.  Peripherals
         * (radio, timer, etc.) call arm_nvic_set_pending → check_pending,
         * which would normally take the exception immediately — but if
         * PRIMASK was set at that moment, the IRQ stays pending until
         * PRIMASK is later cleared (CPSIE i / MSR PRIMASK).  Some driver
         * loops (e.g. nrf_802154's wait_for_flag using __WFE — a NOP for
         * us) never touch PRIMASK while waiting, leaving the IRQ pending
         * for the entire spin.  A per-instruction check is what real
         * Cortex-M does between every instruction; gate on has_pending so
         * the common no-IRQ-pending path stays O(1). */
        if (cpu->nvic) {
            arm_nvic_t *nvic = (arm_nvic_t *)cpu->nvic;
            if (nvic->has_pending && (cpu->primask & 1) == 0)
                arm_nvic_check_pending(nvic);
        }

        /* LR-write trap: dump the instruction that set LR to a target
         * value.  Useful for tracing where bogus return addresses come
         * from when chasing wild-PC bugs.  Set ARM_LR_WATCH=<value>
         * (e.g. 0x20003bb9) — checks against `value` and `value^1`
         * (with/without thumb bit).  One-shot per-CPU. */
        if (getenv("ARM_LR_WATCH") && !cpu->lr_trapped) {
            uint32_t want = (uint32_t)strtoul(getenv("ARM_LR_WATCH"), NULL, 0);
            uint32_t lr = cpu->reg[ARM_LR];
            if (lr == want || lr == (want ^ 1u)) {
                cpu->lr_trapped = 1;
                fprintf(stderr,
                        "[LR_WATCH cpu=%p cyc=%lld src_pc=0x%08x lr=0x%08x sp=0x%08x "
                        "r0=%08x r1=%08x r2=%08x r3=%08x r4=%08x r5=%08x r6=%08x r7=%08x "
                        "r8=%08x r9=%08x r10=%08x r11=%08x r12=%08x next_pc=0x%08x]\n",
                        (void*)cpu, (long long)cpu->cycles, pc, lr, cpu->reg[ARM_SP],
                        cpu->reg[0], cpu->reg[1], cpu->reg[2], cpu->reg[3],
                        cpu->reg[4], cpu->reg[5], cpu->reg[6], cpu->reg[7],
                        cpu->reg[8], cpu->reg[9], cpu->reg[10], cpu->reg[11], cpu->reg[12],
                        cpu->reg[ARM_PC]);
            }
        }

        /* PC trace callback */
        if (cpu->pc_callback)
            cpu->pc_callback(cpu->pc_callback_data, cpu->reg[ARM_PC]);
    }

    if (cpu->cpu_freq_hz > 0)
        cpu->sim_time_ns = arm_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);

    return remaining;
}

void arm_step_until(arm_cpu_t *cpu, int64_t target_cycle) {
    if (cpu->cycles >= target_cycle) return;
    cpu->cycle_limit = target_cycle;
    while (cpu->cycles < target_cycle && !cpu->stopping) {
        /* Estimate instructions needed, batch for efficiency */
        int64_t remaining = target_cycle - cpu->cycles;
        int steps;
        if (remaining > 50000) {
            steps = 10000;
        } else if (remaining <= 10) {
            /* Single-step near target to match MSPSim's per-instruction
             * cycle check. Prevents LPM→event→ISR overshoot that would
             * cause TSCH desync when ported to future ARM TSCH firmware. */
            steps = 1;
        } else {
            steps = (int)(remaining / 2);
            if (steps < 1) steps = 1;
            if (steps > 10000) steps = 10000;
        }
        cpu->stopping = false;
        arm_step(cpu, steps);
        cpu->stopping = false;
    }
    /* Drain events that became due at the boundary.  Without this, a
     * pending event at exactly cycle_limit stays queued until the next
     * arm_step call, introducing one tick of latency per boundary event.
     * Matches msp430_step_until's drain loop. */
    while (cpu->cycles >= cpu->next_event_cycle)
        execute_events(cpu);

    cpu->cycle_limit = INT64_MAX;
    if (cpu->cpu_freq_hz > 0)
        cpu->sim_time_ns = arm_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
}

int64_t arm_step_micros(arm_cpu_t *cpu, int64_t jump_us, int64_t execute_us) {
    if (jump_us < 0) jump_us = 0;
    if (execute_us < 0) execute_us = 0;

    /* Direct port of msp430_step_micros — exact MSPSim stepMicros replication:
     * - last_micros_delta accumulates ALL jump values (never reset)
     * - last_micros_cycles is set ONCE (first call) and NEVER changes
     * - maxCycles = last_micros_cycles + ((last_micros_delta + execute_us) * freq) / 1e6
     * This does ONE integer division for the total accumulated time,
     * bounding truncation error to 1 cycle total (not 1 per tick).
     *
     * Returns an µs hint to the next natural wakeup — the caller uses
     * this to schedule its next tick. Returns 0 if the CPU has pending
     * work (interrupts, events already due, or not in WFI). */

    /* Initialize on first call */
    if (!cpu->micro_clock_ready) {
        cpu->last_micros_cycles = cpu->cycles;
        cpu->last_micros_delta = 0;
        cpu->micro_clock_ready = true;
    }

    /* Accumulate jump (matches: last_micros_delta += jump_us) */
    cpu->last_micros_delta += jump_us;

    /* Compute target from ORIGINAL base + TOTAL accumulated delta */
    int64_t target_cycle = cpu->last_micros_cycles +
        ((cpu->last_micros_delta + execute_us) * (int64_t)cpu->cpu_freq_hz) / 1000000LL;

    if (target_cycle > cpu->cycles)
        arm_step_until(cpu, target_cycle);

    /* If the CPU is in WFI with no pending NVIC interrupts and the next
     * scheduled event is in the future, hint how long the caller can sleep. */
    bool nvic_has_pending = cpu->nvic && ((arm_nvic_t *)cpu->nvic)->has_pending;
    if (cpu->cpu_off && !nvic_has_pending &&
        cpu->cpu_freq_hz > 0 &&
        cpu->next_event_cycle > cpu->cycles) {
        return ((cpu->next_event_cycle - cpu->cycles) * 1000000LL) /
               cpu->cpu_freq_hz;
    }
    return 0;
}
