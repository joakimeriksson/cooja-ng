/*
 * MSP430 CPU emulator — instruction execution engine
 *
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * C reimplementation 2026.
 */
#include "msp430_cpu.h"
#include "msp430_decode.h"
#include "msp430_config.h"
#ifdef HAVE_LIGHTNING
#include "msp430_jit.h"
#endif
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* --- Constant generator tables --- */
/* CG1 (R2/SR): As=0 → SR, As=1 → abs addr, As=2 → 4, As=3 → 8 */
/* CG2 (R3):    As=0 → 0, As=1 → 1, As=2 → 2, As=3 → 0xFFFF */
static const uint32_t cg1_values[4] = { 0, 0, 4, 8 };
static const uint32_t cg2_values[4] = { 0, 1, 2, 0xFFFF };

/* --- Inline helpers --- */

static inline uint16_t read_word(const msp430_cpu_t *cpu, uint32_t addr) {
    return (uint16_t)(cpu->memory[addr] | (cpu->memory[addr + 1] << 8));
}

static inline uint32_t read_word20(const msp430_cpu_t *cpu, uint32_t addr) {
    /* Read 20-bit value: low word at addr, high nibble at addr+2 */
    uint32_t lo = cpu->memory[addr] | (cpu->memory[addr + 1] << 8);
    uint32_t hi = cpu->memory[addr + 2] | (cpu->memory[addr + 3] << 8);
    return lo | ((hi & 0xf) << 16);
}

static inline void write_word(msp430_cpu_t *cpu, uint32_t addr, uint16_t val) {
    cpu->memory[addr]     = val & 0xff;
    cpu->memory[addr + 1] = (val >> 8) & 0xff;
}

static inline void write_word20(msp430_cpu_t *cpu, uint32_t addr, uint32_t val) {
    cpu->memory[addr]     = val & 0xff;
    cpu->memory[addr + 1] = (val >> 8) & 0xff;
    cpu->memory[addr + 2] = (val >> 16) & 0xf;
    cpu->memory[addr + 3] = 0;
}

/* Memory read with IO dispatch */
static inline int mem_read(msp430_cpu_t *cpu, uint32_t addr, bool word) {
    if (addr < cpu->max_mem_io && cpu->io_read[addr]) {
        return cpu->io_read[addr](cpu->io_user_data[addr], addr, word, cpu->cycles);
    }
    if (addr >= cpu->max_mem) return 0;
    if (word) {
        return read_word(cpu, addr);
    }
    return cpu->memory[addr];
}

/* Invalidate JIT cache entries covering address range */
static inline void cache_invalidate(msp430_cpu_t *cpu, uint32_t addr) {
#ifdef HAVE_LIGHTNING
    if (!cpu->compiled_cache) return;
    /* Invalidate a small window around the written address.
     * An instruction can be up to 6 bytes, and a block starting before
     * the write could include this address. We clear entries for
     * (addr-4)...(addr+2) to be safe. */
    uint32_t lo = (addr >= 4) ? (addr - 4) >> 1 : 0;
    uint32_t hi = (addr + 2) >> 1;
    if (hi >= cpu->cache_size) hi = cpu->cache_size - 1;
    for (uint32_t i = lo; i <= hi; i++) {
        if (cpu->compiled_cache[i]) {
            msp430_jit_free((compiled_block_t *)cpu->compiled_cache[i]);
            cpu->compiled_cache[i] = NULL;
            cpu->block_exec_count[i] = 0;
        }
    }
#else
    (void)cpu; (void)addr;
#endif
}

/* Memory write with IO dispatch */
static inline void mem_write(msp430_cpu_t *cpu, uint32_t addr, int val, bool word) {
    if (addr < cpu->max_mem_io && cpu->io_write[addr]) {
        cpu->io_write[addr](cpu->io_user_data[addr], addr, val, word, cpu->cycles);
        return;
    }
    if (addr >= cpu->max_mem) return;
    if (word) {
        write_word(cpu, addr, (uint16_t)val);
    } else {
        cpu->memory[addr] = val & 0xff;
    }
    cache_invalidate(cpu, addr);
}

/* Read memory in appropriate mode: 0=byte, 1=word, 2=word20 */
static inline int mem_read_mode(msp430_cpu_t *cpu, uint32_t addr, int mode) {
    if (mode == 2) {
        /* WORD20: read 20-bit value as two words */
        uint32_t lo = mem_read(cpu, addr, true);
        uint32_t hi = mem_read(cpu, addr + 2, true);
        return (int)(lo | ((hi & 0xf) << 16));
    }
    return mem_read(cpu, addr, mode == 1);
}

/* Write memory in appropriate mode */
static inline void mem_write_mode(msp430_cpu_t *cpu, uint32_t addr, int val, int mode) {
    if (mode == 2) {
        /* WORD20 */
        mem_write(cpu, addr, val & 0xffff, true);
        mem_write(cpu, addr + 2, (val >> 16) & 0xf, true);
        return;
    }
    mem_write(cpu, addr, val, mode == 1);
}

static inline int16_t sign_extend_10(int val) {
    if (val & 0x200) val |= 0xfc00;
    return (int16_t)val;
}

static inline int32_t sign_extend_16(int val) {
    if (val & 0x8000) return val | (int32_t)0xffff0000;
    return val;
}

static inline int32_t sign_extend_20(int val) {
    if (val & 0x80000) return val | (int32_t)0xfff00000;
    return val;
}

/* --- Lifecycle --- */

void msp430_cpu_init(msp430_cpu_t *cpu, const msp430_config_t *config) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->config = config;
    cpu->max_mem = config->max_mem;
    cpu->max_mem_io = config->max_mem_io;
    cpu->is_msp430x = config->is_msp430x;
    cpu->max_interrupt = config->max_interrupt;
    cpu->interrupt_max = -1;
    cpu->serviced_interrupt = -1;

    cpu->memory = (uint8_t *)calloc(cpu->max_mem, 1);

    /* IO dispatch tables */
    cpu->io_read     = (io_read_fn *)calloc(cpu->max_mem_io, sizeof(io_read_fn));
    cpu->io_write    = (io_write_fn *)calloc(cpu->max_mem_io, sizeof(io_write_fn));
    cpu->io_user_data = (void **)calloc(cpu->max_mem_io, sizeof(void *));

    /* Interrupt tables */
    int nvectors = config->max_interrupt + 1;
    cpu->interrupt_handler = (interrupt_handler_fn *)calloc(nvectors, sizeof(interrupt_handler_fn));
    cpu->interrupt_source  = (void **)calloc(nvectors, sizeof(void *));

    cpu->next_event_cycle = INT64_MAX;
    cpu->cycle_limit = INT64_MAX;

    /* JIT cache */
    cpu->cache_size = cpu->max_mem >> 1;
#ifdef HAVE_LIGHTNING
    cpu->compiled_cache = (void **)calloc(cpu->cache_size, sizeof(void *));
    cpu->block_exec_count = (int32_t *)calloc(cpu->cache_size, sizeof(int32_t));
    cpu->jit_threshold = 100;
    cpu->jit_inblock_checks = 0;  /* default: off (faster) */
    /* Check environment for overrides */
    {
        const char *env = getenv("MSPSIM_JIT_THRESHOLD");
        if (env) cpu->jit_threshold = atoi(env);
        env = getenv("MSPSIM_JIT_INBLOCK_CHECKS");
        if (env) cpu->jit_inblock_checks = atoi(env);
    }
    {
        static int jit_initialized = 0;
        if (!jit_initialized) {
            msp430_jit_init();
            jit_initialized = 1;
        }
    }
#endif
}

static void cache_clear_all(msp430_cpu_t *cpu);

void msp430_cpu_destroy(msp430_cpu_t *cpu) {
#ifdef HAVE_LIGHTNING
    /* Free compiled blocks */
    if (cpu->compiled_cache) {
        for (uint32_t i = 0; i < cpu->cache_size; i++) {
            if (cpu->compiled_cache[i]) {
                msp430_jit_free((compiled_block_t *)cpu->compiled_cache[i]);
            }
        }
        free(cpu->compiled_cache);
        cpu->compiled_cache = NULL;
    }
    free(cpu->block_exec_count);
    cpu->block_exec_count = NULL;
#endif

    free(cpu->memory);
    free(cpu->io_read);
    free(cpu->io_write);
    free(cpu->io_user_data);
    free(cpu->interrupt_handler);
    free(cpu->interrupt_source);

    /* Free event queue */
    /* Events are owned externally, just clear pointers */
    cpu->event_queue = NULL;
}

static void cache_clear_all(msp430_cpu_t *cpu) {
#ifdef HAVE_LIGHTNING
    if (cpu->compiled_cache) {
        for (uint32_t i = 0; i < cpu->cache_size; i++) {
            if (cpu->compiled_cache[i]) {
                msp430_jit_free((compiled_block_t *)cpu->compiled_cache[i]);
                cpu->compiled_cache[i] = NULL;
            }
            cpu->block_exec_count[i] = 0;
        }
    }
#else
    (void)cpu;
#endif
}

static void reevaluate_interrupts(msp430_cpu_t *cpu) {
    cpu->interrupt_max = -1;
    int n = cpu->max_interrupt + 1;
    for (int i = 0; i < n; i++) {
        if (cpu->interrupt_source[i] != NULL || cpu->interrupt_handler[i] != NULL) {
            cpu->interrupt_max = i;
        }
    }
}

void msp430_cpu_reset(msp430_cpu_t *cpu) {
    /* Clear interrupt state */
    int nvectors = cpu->max_interrupt + 1;
    memset(cpu->interrupt_source, 0, nvectors * sizeof(void *));
    memset(cpu->interrupt_handler, 0, nvectors * sizeof(interrupt_handler_fn));
    cpu->interrupt_max = -1;
    cpu->serviced_interrupt = -1;

    /* Clear registers */
    memset(cpu->reg, 0, sizeof(cpu->reg));
    cpu->interrupts_enabled = false;
    cpu->cpu_off = false;
    cpu->stopping = false;

    /* Clear events */
    cpu->event_queue = NULL;
    cpu->next_event_cycle = INT64_MAX;

    cpu->ext_word = 0;

    /* Clear decode cache */
    cache_clear_all(cpu);

    /* Flag reset interrupt */
    msp430_flag_interrupt(cpu, cpu->max_interrupt, NULL, NULL, true);
}

void msp430_stop(msp430_cpu_t *cpu) {
    cpu->stopping = true;
}

/* --- IO registration --- */

void msp430_register_io(msp430_cpu_t *cpu, uint32_t addr, uint32_t size,
                         io_read_fn read, io_write_fn write, void *data) {
    for (uint32_t i = 0; i < size; i++) {
        if (addr + i < cpu->max_mem_io) {
            cpu->io_read[addr + i] = read;
            cpu->io_write[addr + i] = write;
            cpu->io_user_data[addr + i] = data;
        }
    }
}

/* --- Interrupt management --- */

void msp430_flag_interrupt(msp430_cpu_t *cpu, int vector, void *source,
                            interrupt_handler_fn handler, bool set) {
    if (set) {
        cpu->interrupt_source[vector] = source ? source : (void *)1;
        cpu->interrupt_handler[vector] = handler;
        if (vector > cpu->interrupt_max) {
            cpu->interrupt_max = vector;
        }
        if (vector == cpu->max_interrupt) {
            /* Reset cannot be masked */
            cpu->interrupts_enabled = true;
            cpu->serviced_interrupt = -1;
        }
    } else {
        cpu->interrupt_source[vector] = NULL;
        cpu->interrupt_handler[vector] = NULL;
        reevaluate_interrupts(cpu);
    }
}

/* --- Event management --- */

void msp430_schedule_event(msp430_cpu_t *cpu, msp430_event_t *ev, int64_t cycle) {
    /* Remove if already queued */
    msp430_cancel_event(cpu, ev);

    ev->fire_cycle = cycle;
    ev->fire_ns = 0;  /* cycle-based event */
    ev->next = NULL;

    /* Insert sorted */
    if (cpu->event_queue == NULL || cycle < cpu->event_queue->fire_cycle) {
        ev->next = cpu->event_queue;
        cpu->event_queue = ev;
    } else {
        msp430_event_t *prev = cpu->event_queue;
        while (prev->next && prev->next->fire_cycle <= cycle) {
            prev = prev->next;
        }
        ev->next = prev->next;
        prev->next = ev;
    }
    cpu->next_event_cycle = cpu->event_queue->fire_cycle;
}

void msp430_schedule_event_ns(msp430_cpu_t *cpu, msp430_event_t *ev, int64_t fire_ns) {
    /* Remove if already queued */
    msp430_cancel_event(cpu, ev);

    ev->fire_ns = fire_ns;
    /* Compute shadow fire_cycle from ns */
    int64_t delta_ns = fire_ns - cpu->sim_time_ns;
    if (delta_ns < 0) delta_ns = 0;
    ev->fire_cycle = cpu->cycles + msp430_ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
    ev->next = NULL;

    /* Insert sorted by fire_cycle */
    int64_t cycle = ev->fire_cycle;
    if (cpu->event_queue == NULL || cycle < cpu->event_queue->fire_cycle) {
        ev->next = cpu->event_queue;
        cpu->event_queue = ev;
    } else {
        msp430_event_t *prev = cpu->event_queue;
        while (prev->next && prev->next->fire_cycle <= cycle) {
            prev = prev->next;
        }
        ev->next = prev->next;
        prev->next = ev;
    }
    cpu->next_event_cycle = cpu->event_queue->fire_cycle;
}

void msp430_cancel_event(msp430_cpu_t *cpu, msp430_event_t *ev) {
    if (cpu->event_queue == ev) {
        cpu->event_queue = ev->next;
    } else {
        msp430_event_t *prev = cpu->event_queue;
        while (prev && prev->next != ev) prev = prev->next;
        if (prev) prev->next = ev->next;
    }
    ev->next = NULL;
    ev->fire_ns = 0;
    cpu->next_event_cycle = cpu->event_queue ? cpu->event_queue->fire_cycle : INT64_MAX;
}

void msp430_cpu_set_frequency(msp430_cpu_t *cpu, uint32_t freq_hz) {
    if (freq_hz == 0) return;
    /* Update sim_time_ns to current cycles before changing frequency */
    if (cpu->cpu_freq_hz > 0) {
        cpu->sim_time_ns = msp430_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
    }
    cpu->cpu_freq_hz = freq_hz;

    /* Recompute fire_cycle for all ns-based events */
    msp430_event_t *ev = cpu->event_queue;
    while (ev) {
        if (ev->fire_ns > 0) {
            int64_t delta_ns = ev->fire_ns - cpu->sim_time_ns;
            if (delta_ns < 0) delta_ns = 0;
            ev->fire_cycle = cpu->cycles + msp430_ns_to_cycles(delta_ns, freq_hz);
        }
        ev = ev->next;
    }

    /* Re-sort event queue (simple insertion sort rebuild) */
    msp430_event_t *old_queue = cpu->event_queue;
    cpu->event_queue = NULL;
    while (old_queue) {
        msp430_event_t *e = old_queue;
        old_queue = e->next;
        e->next = NULL;

        int64_t cycle = e->fire_cycle;
        if (cpu->event_queue == NULL || cycle < cpu->event_queue->fire_cycle) {
            e->next = cpu->event_queue;
            cpu->event_queue = e;
        } else {
            msp430_event_t *prev = cpu->event_queue;
            while (prev->next && prev->next->fire_cycle <= cycle) {
                prev = prev->next;
            }
            e->next = prev->next;
            prev->next = e;
        }
    }
    cpu->next_event_cycle = cpu->event_queue ? cpu->event_queue->fire_cycle : INT64_MAX;
}

static void execute_events(msp430_cpu_t *cpu) {
    while (cpu->event_queue && cpu->cycles >= cpu->event_queue->fire_cycle) {
        msp430_event_t *ev = cpu->event_queue;
        cpu->event_queue = ev->next;
        ev->next = NULL;
        /* Update sim_time_ns from current cycles */
        if (cpu->cpu_freq_hz > 0) {
            cpu->sim_time_ns = msp430_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
        }
        ev->fire_ns = 0;
        ev->callback(ev->user_data, ev);
    }
    cpu->next_event_cycle = cpu->event_queue ? cpu->event_queue->fire_cycle : INT64_MAX;
}

/* --- Interrupt service --- */

static int service_interrupt(msp430_cpu_t *cpu, int pc) {
    int sp = cpu->reg[MSP430_SP];
    int sr = cpu->reg[MSP430_SR];
    int max_int = cpu->max_interrupt;
    int vec = cpu->interrupt_max;

    if (vec < max_int) {
        /* Push PC and SR to stack */
        sp -= 2;
        cpu->reg[MSP430_SP] = sp;
        mem_write(cpu, sp, pc & 0xffff, true);

        sp -= 2;
        cpu->reg[MSP430_SP] = sp;
        mem_write(cpu, sp, (sr & 0x0fff) | ((pc & 0xf0000) >> 4), true);
    }

    /* Clear SR (disables interrupts, exits LPM) */
    cpu->reg[MSP430_SR] = 0;
    cpu->interrupts_enabled = false;
    cpu->cpu_off = false;

    /* Read interrupt vector */
    int vector_addr = 0xfffe - (max_int - vec) * 2;
    pc = read_word(cpu, vector_addr);
    cpu->reg[MSP430_PC] = pc;

    cpu->serviced_interrupt = vec;
    interrupt_handler_fn handler = cpu->interrupt_handler[vec];
    void *source = cpu->interrupt_source[vec];

    /* Clear this interrupt source */
    cpu->interrupt_source[vec] = NULL;
    cpu->interrupt_handler[vec] = NULL;
    reevaluate_interrupts(cpu);

    /* Handle reset */
    if (vec == max_int) {
        /* Full reset: clear all interrupt state, keep memory/config */
        int nvectors = max_int + 1;
        memset(cpu->interrupt_source, 0, nvectors * sizeof(void *));
        memset(cpu->interrupt_handler, 0, nvectors * sizeof(interrupt_handler_fn));
        cpu->interrupt_max = -1;
        cpu->serviced_interrupt = -1;
        cpu->event_queue = NULL;
        cpu->next_event_cycle = INT64_MAX;
    }

    /* 6 cycles for interrupt service */
    cpu->cycles += 6;

    /* Notify handler */
    if (handler) {
        handler(source, vec);
    }

    return pc;
}

/* --- Write to SR with side effects --- */

static inline void write_sr(msp430_cpu_t *cpu, uint32_t val) {
    bool old_ie = cpu->interrupts_enabled;
    cpu->reg[MSP430_SR] = val;
    cpu->interrupts_enabled = (val & SR_GIE) != 0;
    cpu->cpu_off = (val & SR_CPUOFF) != 0;

    /* When GIE transitions 0→1 while servicing an interrupt, the current
     * interrupt is considered done and pending interrupts can fire.
     * Matches Java MSPSim's handlePendingInterrupts() call in SR write. */
    if (!old_ie && cpu->interrupts_enabled && cpu->serviced_interrupt >= 0) {
        reevaluate_interrupts(cpu);
        cpu->serviced_interrupt = -1;
    }
}

/* Write SR for flag-only updates (no LPM checks needed) */
static inline void write_sr_flags(msp430_cpu_t *cpu, uint32_t val) {
    cpu->reg[MSP430_SR] = val;
}

/* Handle pending interrupts (after RETI) */
static inline void handle_pending_interrupts(msp430_cpu_t *cpu) {
    reevaluate_interrupts(cpu);
    cpu->serviced_interrupt = -1;
}

/* Forward declaration — the original interpreter, renamed */
static int msp430_step_interpreter(msp430_cpu_t *cpu, int count);

/* ===================================================================
 * DECODED INSTRUCTION EXECUTION ENGINE
 * =================================================================== */

/*
 * Execute a single decoded instruction.
 * Returns 1 on success, 0 if the instruction needs special handling
 * (e.g., RETI, or CPU state changed).
 */
static int execute_decoded(msp430_cpu_t *cpu, const decoded_insn_t *di, uint32_t next_pc) {
    uint32_t *reg = cpu->reg;

    switch (di->itype) {
    case ITYPE_TWO_OP: {
        int op = di->operation;
        bool bw = di->byte_mode;
        uint32_t mask = bw ? 0xff : 0xffff;
        uint32_t msb_bit = bw ? 0x80 : 0x8000;
        int mode_bytes = bw ? 1 : 2;
        bool dst_reg_mode = (di->dst_kind == OPKIND_REG);
        int src = 0, dst = 0;
        int dst_address = -1;

        /* --- Resolve source --- */
        switch (di->src_kind) {
        case OPKIND_REG:
            src = reg[di->src_reg] & mask;
            cpu->cycles += dst_reg_mode ? 1 : 4;
            if (di->dst_reg == MSP430_PC && dst_reg_mode) cpu->cycles += 1;
            break;
        case OPKIND_CG:
            src = di->src_value;
            if (bw && di->src_reg == MSP430_CG2 && src == (int32_t)0xFFFF) src = 0xff;
            cpu->cycles += dst_reg_mode ? 1 : 4;
            break;
        case OPKIND_INDEXED: {
            uint32_t rval = reg[di->src_reg];
            uint32_t addr;
            if (rval <= 0xffff) {
                addr = (rval + di->src_value) & 0xffff;
            } else {
                addr = (rval + di->src_value) & 0xfffff;
            }
            src = mem_read(cpu, addr, !bw);
            if (bw) src &= 0xff;
            cpu->cycles += dst_reg_mode ? 3 : 6;
            break;
        }
        case OPKIND_ABSOLUTE:
            src = mem_read(cpu, (uint32_t)(uint16_t)di->src_value, !bw);
            if (bw) src &= 0xff;
            cpu->cycles += dst_reg_mode ? 3 : 6;
            break;
        case OPKIND_INDIRECT:
            src = mem_read(cpu, reg[di->src_reg], !bw);
            if (bw) src &= 0xff;
            cpu->cycles += dst_reg_mode ? 2 : 5;
            break;
        case OPKIND_AUTOINC:
            src = mem_read(cpu, reg[di->src_reg], !bw);
            if (bw) src &= 0xff;
            reg[di->src_reg] += mode_bytes;
            cpu->cycles += dst_reg_mode ? 2 : 5;
            if (di->dst_reg == MSP430_PC && dst_reg_mode) cpu->cycles += 1;
            break;
        case OPKIND_IMMEDIATE:
            src = di->src_value;
            if (bw) src &= 0xff;
            cpu->cycles += dst_reg_mode ? 2 : 5;
            if (di->dst_reg == MSP430_PC && dst_reg_mode) cpu->cycles += 1;
            break;
        }

        /* --- Resolve destination --- */
        if (dst_reg_mode) {
            if (op != OP_MOV) dst = reg[di->dst_reg] & mask;
        } else {
            /* Indexed or absolute dst */
            if (di->dst_kind == OPKIND_ABSOLUTE) {
                dst_address = (uint32_t)(uint16_t)di->dst_value;
            } else {
                uint32_t rval = reg[di->dst_reg];
                if (rval <= 0xffff) {
                    dst_address = (rval + di->dst_value) & 0xffff;
                } else {
                    dst_address = (rval + di->dst_value) & 0xfffff;
                }
            }
            if (op != OP_MOV) {
                dst = mem_read(cpu, dst_address, !bw);
                if (bw) dst &= 0xff;
            }
        }

        /* --- ALU --- */
        bool write_result = false;
        bool update_status = true;
        uint32_t sr = reg[MSP430_SR];

        switch (op) {
        case OP_MOV:
            dst = src;
            write_result = true;
            update_status = false;
            break;
        case OP_ADD: {
            sr &= ~(SR_V | SR_C);
            uint32_t tmp = (src ^ dst) & msb_bit;
            dst = dst + src;
            if ((uint32_t)dst > mask) sr |= SR_C;
            if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        case OP_ADDC: {
            int carry = (sr & SR_C) ? 1 : 0;
            sr &= ~(SR_V | SR_C);
            uint32_t tmp = (src ^ dst) & msb_bit;
            dst = dst + src + carry;
            if ((uint32_t)dst > mask) sr |= SR_C;
            if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        case OP_SUB: {
            src = (~src) & mask;
            sr &= ~(SR_V | SR_C);
            uint32_t tmp = (src ^ dst) & msb_bit;
            dst = dst + src + 1;
            if ((uint32_t)dst > mask) sr |= SR_C;
            if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        case OP_SUBC: {
            int carry = (sr & SR_C) ? 1 : 0;
            src = (~src) & mask;
            sr &= ~(SR_V | SR_C);
            uint32_t tmp = (src ^ dst) & msb_bit;
            dst = dst + src + carry;
            if ((uint32_t)dst > mask) sr |= SR_C;
            if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        case OP_CMP: {
            sr = (sr & ~(SR_C | SR_V)) | ((uint32_t)dst >= (uint32_t)src ? SR_C : 0);
            int tmp = dst - src;
            if (((src ^ tmp) & msb_bit) == 0 && ((src ^ dst) & msb_bit) != 0) {
                sr |= SR_V;
            }
            write_sr_flags(cpu, sr);
            dst = tmp;
            write_result = false;
            break;
        }
        case OP_DADD:
            dst = dst + src + ((sr & SR_C) ? 1 : 0);
            write_result = true;
            break;
        case OP_BIT:
            dst = src & dst;
            sr = (sr & ~(SR_C | SR_V)) | ((dst != 0) ? SR_C : 0);
            write_sr_flags(cpu, sr);
            write_result = false;
            break;
        case OP_BIC:
            dst = (~src) & dst;
            write_result = true;
            update_status = false;
            break;
        case OP_BIS:
            dst = src | dst;
            write_result = true;
            update_status = false;
            break;
        case OP_XOR: {
            sr &= ~(SR_C | SR_V);
            if ((src & msb_bit) && (dst & msb_bit)) sr |= SR_V;
            dst = src ^ dst;
            if (dst != 0) sr |= SR_C;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        case OP_AND: {
            sr &= ~(SR_C | SR_V);
            dst = src & dst;
            if (dst != 0) sr |= SR_C;
            write_sr_flags(cpu, sr);
            write_result = true;
            break;
        }
        }

        dst &= mask;

        if (write_result) {
            if (dst_reg_mode) {
                if (di->dst_reg == MSP430_SR) {
                    write_sr(cpu, dst);
                } else {
                    reg[di->dst_reg] = dst;
                }
            } else {
                mem_write(cpu, dst_address, dst, !bw);
            }
        }

        if (update_status) {
            sr = reg[MSP430_SR];
            sr = (sr & ~(SR_Z | SR_N)) |
                 ((dst == 0) ? SR_Z : 0) |
                 ((dst & msb_bit) ? SR_N : 0);
            write_sr_flags(cpu, sr);
        }

        if (!(dst_reg_mode && di->dst_reg == MSP430_PC && write_result)) {
            reg[MSP430_PC] = next_pc;
        }
        return 1;
    }

    case ITYPE_JUMP: {
        cpu->cycles += 2;
        uint32_t sr = reg[MSP430_SR];
        bool jump;
        switch (di->jmp_condition) {
        case JMP_JNE: jump = (sr & SR_Z) == 0; break;
        case JMP_JEQ: jump = (sr & SR_Z) != 0; break;
        case JMP_JNC: jump = (sr & SR_C) == 0; break;
        case JMP_JC:  jump = (sr & SR_C) != 0; break;
        case JMP_JN:  jump = (sr & SR_N) != 0; break;
        case JMP_JGE: jump = ((sr & SR_N) != 0) == ((sr & SR_V) != 0); break;
        case JMP_JL:  jump = ((sr & SR_N) != 0) != ((sr & SR_V) != 0); break;
        case JMP_JMP: jump = true; break;
        default: jump = false; break;
        }
        if (jump) {
            reg[MSP430_PC] = (next_pc + di->jmp_offset) &
                             (cpu->is_msp430x ? 0xfffff : 0xffff);
        } else {
            reg[MSP430_PC] = next_pc;
        }
        return 1;
    }

    default:
        /* Single-op, MSP430X — fall back to interpreter */
        return 0;
    }
}

#ifdef HAVE_LIGHTNING
/*
 * Execute a decoded instruction from JIT-compiled code.
 * This is called by JIT code for instructions it can't inline.
 * Must be a non-static function so the JIT can call it.
 */
int execute_decoded_for_jit(msp430_cpu_t *cpu, const decoded_insn_t *di,
                            uint32_t next_pc) {
    int result = execute_decoded(cpu, di, next_pc);
    if (!result) {
        /* Fall back to interpreter for this single instruction */
        cpu->reg[MSP430_PC] = next_pc - di->size;
        cpu->ext_word = di->ext_word;
        msp430_step_interpreter(cpu, 1);
        return 1;
    }
    return result;
}
#endif


/* ===================================================================
 * MAIN EXECUTION ENGINE
 * =================================================================== */

/* #define JIT_VERIFY 1 */  /* Enable to compare JIT vs interpreter */

int msp430_step(msp430_cpu_t *cpu, int count) {
#ifdef HAVE_LIGHTNING
    if (cpu->compiled_cache) {
        while (count > 0) {
            /* Event processing */
            if (cpu->cycles >= cpu->next_event_cycle) {
                execute_events(cpu);
            }

            /* Interrupt service — same logic as interpreter */
            if (cpu->interrupts_enabled && cpu->serviced_interrupt == -1
                    && cpu->interrupt_max >= 0) {
                service_interrupt(cpu, cpu->reg[MSP430_PC]);
            }

            /* LPM: use interpreter for low-power mode handling */
            if (cpu->cpu_off) {
                int batch = count < 1000 ? count : 1000;
                int rem = msp430_step_interpreter(cpu, batch);
                count -= (batch - rem);
                if (cpu->cycles >= cpu->cycle_limit)
                    return count;
                continue;
            }

            if (cpu->stopping) {
                cpu->stopping = false;
                return count;
            }

            /* Try JIT block at current PC */
            uint32_t pc = cpu->reg[MSP430_PC];
            uint32_t ci = pc >> 1;

            if (ci < cpu->cache_size) {
                compiled_block_t *cb =
                    (compiled_block_t *)cpu->compiled_cache[ci];
                if (cb && count >= cb->length &&
                    cpu->cycles + cb->length * 6 <
                        cpu->next_event_cycle) {
#ifdef JIT_VERIFY
                    /* Save state before JIT execution */
                    uint32_t save_reg[16];
                    memcpy(save_reg, cpu->reg, sizeof(save_reg));
                    int64_t save_cycles = cpu->cycles;

                    int executed = cb->fn(cpu);
                    if (executed > 0) {
                        /* Save JIT results */
                        uint32_t jit_reg[16];
                        memcpy(jit_reg, cpu->reg, sizeof(jit_reg));
                        int64_t jit_cycles = cpu->cycles;

                        /* Restore and run interpreter */
                        memcpy(cpu->reg, save_reg, sizeof(save_reg));
                        cpu->cycles = save_cycles;
                        cpu->interrupts_enabled = (save_reg[MSP430_SR] & SR_GIE) != 0;
                        cpu->cpu_off = (save_reg[MSP430_SR] & SR_CPUOFF) != 0;
                        msp430_step_interpreter(cpu, executed);

                        /* Compare */
                        int mismatch = 0;
                        for (int r = 0; r < 16; r++) {
                            if (jit_reg[r] != cpu->reg[r]) {
                                if (!mismatch) fprintf(stderr,
                                    "JIT MISMATCH at PC=0x%04x (block len=%d, executed=%d):\n",
                                    save_reg[0], cb->length, executed);
                                fprintf(stderr, "  R%d: JIT=0x%05x INTERP=0x%05x\n",
                                        r, jit_reg[r], cpu->reg[r]);
                                mismatch = 1;
                            }
                        }
                        if (jit_cycles != cpu->cycles && !mismatch) {
                            fprintf(stderr,
                                "JIT CYCLE MISMATCH at PC=0x%04x: JIT=%lld INTERP=%lld\n",
                                save_reg[0], (long long)jit_cycles,
                                (long long)cpu->cycles);
                        }

                        cpu->instructions += executed;
                        count -= executed;
                        if (cpu->cycles >= cpu->cycle_limit)
                            return count;
                        continue;
                    }
#else
                    int executed = cb->fn(cpu);
                    if (executed > 0) {
                        cpu->instructions += executed;
                        count -= executed;
                        /* Tight JIT chain: keep executing compiled blocks.
                           Check events and interrupts between blocks. */
                        while (count > 0 &&
                               cpu->cycles < cpu->next_event_cycle &&
                               cpu->interrupt_max < 0) {
                            pc = cpu->reg[MSP430_PC];
                            ci = pc >> 1;
                            if (ci >= cpu->cache_size) break;
                            cb = (compiled_block_t *)
                                cpu->compiled_cache[ci];
                            if (!cb || count < cb->length) break;
                            executed = cb->fn(cpu);
                            if (executed <= 0) break;
                            cpu->instructions += executed;
                            count -= executed;
                        }
                        if (cpu->cycles >= cpu->cycle_limit)
                            return count;
                        continue;
                    }
#endif
                }
                /* Hot block counting */
                if (!cb && cpu->block_exec_count[ci] >= 0) {
                    cpu->block_exec_count[ci]++;
                    if (cpu->block_exec_count[ci] >= cpu->jit_threshold) {
                        basic_block_t block;
                        msp430_decode_block(cpu->memory, pc,
                                            cpu->max_mem, &block);
                        compiled_block_t *ncb =
                            msp430_jit_compile(&block, cpu);
                        if (ncb) {
                            cpu->compiled_cache[ci] = ncb;
                            continue; /* re-check with compiled block */
                        } else {
                            cpu->block_exec_count[ci] = -1000000;
                        }
                    }
                }
            }

            /* Interpreter batch, then re-check JIT */
            {
                int batch = count < 32 ? count : 32;
                int rem = msp430_step_interpreter(cpu, batch);
                count -= (batch - rem);
                if (cpu->cycles >= cpu->cycle_limit)
                    return count;
            }
        }
        return count;
    }
#endif
    return msp430_step_interpreter(cpu, count);
}

void msp430_step_until(msp430_cpu_t *cpu, int64_t target_cycle) {
    if (cpu->cycles >= target_cycle) return;
    cpu->cycle_limit = target_cycle;
    while (cpu->cycles < target_cycle) {
        /* Estimate instructions needed.
         * Use large batches for efficiency (JIT path has per-instruction overhead).
         * Overshooting is handled by the while loop condition. */
        int64_t remaining = target_cycle - cpu->cycles;
        int steps;
        if (remaining > 50000) {
            steps = 10000;  /* large batch for long distances */
        } else {
            steps = (int)(remaining / 2);
            if (steps < 1) steps = 1;
            if (steps > 10000) steps = 10000;
        }
        msp430_step(cpu, steps);
    }
    cpu->cycle_limit = INT64_MAX;
    /* Keep sim_time_ns synchronized with cycles */
    if (cpu->cpu_freq_hz > 0) {
        cpu->sim_time_ns = msp430_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
    }
}

/* ===================================================================
 * INTERPRETER EXECUTION ENGINE (original)
 * =================================================================== */

static int msp430_step_interpreter(msp430_cpu_t *cpu, int count) {
    uint32_t *reg = cpu->reg;
    uint8_t *memory = cpu->memory;
    uint32_t max_mem = cpu->max_mem;

/* --- Computed goto dispatch table --- */
#if defined(__GNUC__) || defined(__clang__)
    static const void *dispatch_table[16] = {
        &&op_msp430x, &&op_single, &&op_jump, &&op_jump,
        &&op_mov, &&op_add, &&op_addc, &&op_subc,
        &&op_sub, &&op_cmp, &&op_dadd, &&op_bit,
        &&op_bic, &&op_bis, &&op_xor, &&op_and
    };
    #define DISPATCH(opcode) goto *dispatch_table[opcode]
    #define OP_LABEL(name) name
#else
    #define DISPATCH(opcode) switch(opcode)
    #define OP_LABEL(name) case name
#endif

    while (count > 0) {
        /* Check for events */
        if (cpu->cycles >= cpu->next_event_cycle) {
            execute_events(cpu);
        }

        /* Interrupt processing */
        if (cpu->interrupts_enabled && cpu->serviced_interrupt == -1
                && cpu->interrupt_max >= 0) {
            int pc = reg[MSP430_PC];
            pc = service_interrupt(cpu, pc);
            /* After reset interrupt, the PC is set; continue normally */
        }

        /* LPM check */
        if (cpu->cpu_off) {
            /* In low-power mode, advance to next event (capped by cycle_limit) */
            if (cpu->interrupts_enabled && cpu->interrupt_max > 0) {
                /* Will service interrupt next iteration */
            } else {
                int64_t target = cpu->next_event_cycle;
                if (target > cpu->cycle_limit) target = cpu->cycle_limit;
                cpu->cycles = target;
                if (cpu->cycles >= cpu->cycle_limit) {
                    return count - 1;  /* ran this "instruction" (LPM tick) */
                }
            }
            count--;
            continue;
        }

        if (cpu->stopping) {
            cpu->stopping = false;
            return count;
        }

        cpu->instructions++;

        /* Fetch instruction */
        uint32_t pc = reg[MSP430_PC];
        if (pc + 1 >= max_mem) {
            return count;
        }

        uint16_t instr = memory[pc] | (memory[pc + 1] << 8);

        int op = (instr >> 12) & 0xf;

#if defined(__GNUC__) || defined(__clang__)
        DISPATCH(op);
#else
        switch (op) {
#endif

        /* ===================================================================
         * MSP430X / Single-operand / Extension word (opcode 0)
         * =================================================================== */
        OP_LABEL(op_msp430x): {
            /* Check if this is an extension word (bit 11 set = 0x1800 prefix) */
            if ((instr & 0xf800) == 0x1800) {
                /* Extension word for next instruction */
                int ext = instr;
                cpu->ext_word = ext;
                pc += 2;
                /* Fetch the actual instruction */
                instr = memory[pc] | (memory[pc + 1] << 8);
                cpu->last_instruction = instr;
                op = (instr >> 12) & 0xf;

                /* The extended instruction is a normal two-op or single-op
                   with 20-bit addressing. For now, handle via normal paths
                   with extWord set. pc points at the actual instruction;
                   the two-op/single-op path will advance past it. */
                /* TODO: full extension word handling for repeat, ZC, 20-bit */
                goto extended_twoop;
            }

            /* MSP430X native instructions (0x0000-0x0FFF, 0x1000-0x17FF) */
            int msp430x_op = instr & 0x00f0;

            /* CALLA variants: 0x1340-0x13B0 */
            if ((instr & 0xff00) == 0x1300 && (instr & 0x00f0) >= 0x0040) {
                goto calla_dispatch;
            }

            /* PUSHM/POPM: 0x1400-0x17FF */
            if ((instr & 0xfc00) >= 0x1400 && (instr & 0xfc00) <= 0x1700) {
                goto pushm_popm_dispatch;
            }

            /* RRXX: 0x0040-0x004F with upper bits indicating type */
            if ((instr & 0xf000) == 0x0000 && (instr & 0x0c00) != 0) {
                goto rrxx_dispatch;
            }

            /* MOVA/CMPA/ADDA/SUBA variants */
            int src_reg = (instr >> 8) & 0xf;
            int dst_reg = instr & 0xf;
            pc += 2;

            switch (msp430x_op) {
            case MOVA_IND: {
                uint32_t addr = reg[src_reg];
                reg[dst_reg] = mem_read_mode(cpu, addr, 2) & 0xfffff;
                cpu->cycles += 3;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_IND_AUTOINC: {
                reg[MSP430_PC] = pc;
                uint32_t addr = reg[src_reg];
                uint32_t val = mem_read_mode(cpu, addr, 2) & 0xfffff;
                reg[src_reg] = (addr + 4) & 0xfffff;
                reg[dst_reg] = val;
                cpu->cycles += 3;
                break;
            }
            case MOVA_ABS2REG: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t addr = lo | ((uint32_t)src_reg << 16);
                reg[dst_reg] = mem_read_mode(cpu, addr, 2) & 0xfffff;
                cpu->cycles += 4;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_INDX2REG: {
                uint16_t idx = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                int32_t index = sign_extend_16(idx);
                int32_t base = sign_extend_20(reg[src_reg]);
                uint32_t addr = (base + index) & 0xfffff;
                reg[dst_reg] = mem_read_mode(cpu, addr, 2) & 0xfffff;
                cpu->cycles += 4;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_REG2ABS: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t addr = lo | ((uint32_t)dst_reg << 16);
                mem_write_mode(cpu, addr, reg[src_reg], 2);
                cpu->cycles += 4;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_REG2INDX: {
                uint16_t idx = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                int32_t index = sign_extend_16(idx);
                int32_t base = sign_extend_20(reg[dst_reg]);
                uint32_t addr = (base + index) & 0xfffff;
                mem_write_mode(cpu, addr, reg[src_reg], 2);
                cpu->cycles += 4;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_IMM2REG: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t val = lo | ((uint32_t)src_reg << 16);
                reg[dst_reg] = val & 0xfffff;
                cpu->cycles += 2;
                reg[MSP430_PC] = pc;
                break;
            }
            case CMPA_IMM: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t imm = lo | ((uint32_t)src_reg << 16);
                uint32_t dst = reg[dst_reg];
                uint32_t sr = reg[MSP430_SR] & ~(SR_N | SR_Z | SR_C | SR_V);
                if (dst >= imm) sr |= SR_C;
                if (dst < imm) sr |= SR_N;
                if (dst == imm) sr |= SR_Z;
                int32_t tmp = (int32_t)dst - (int32_t)imm;
                uint32_t b = 0x80000;
                if (((dst ^ (uint32_t)tmp) & b) == 0 && ((dst ^ imm) & b) != 0) {
                    sr |= SR_V;
                }
                write_sr_flags(cpu, sr);
                cpu->cycles += 3;
                reg[MSP430_PC] = pc;
                break;
            }
            case ADDA_IMM: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t imm = lo | ((uint32_t)src_reg << 16);
                uint32_t dst_val = reg[dst_reg];
                uint32_t result = dst_val + imm;
                /* Update SR */
                uint32_t sr = reg[MSP430_SR];
                /* Use simplified flag update for address arithmetic */
                sr &= ~(SR_C | SR_Z | SR_N | SR_V);
                if ((result & 0xfffff) == 0) sr |= SR_Z;
                if (result & 0x80000) sr |= SR_N;
                if (result > 0xfffff) sr |= SR_C;
                if (!((dst_val ^ imm) & 0x80000) && ((imm ^ result) & 0x80000)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                reg[dst_reg] = result & 0xfffff;
                cpu->cycles += 3;
                reg[MSP430_PC] = pc;
                break;
            }
            case SUBA_IMM: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t imm = lo | ((uint32_t)src_reg << 16);
                reg[dst_reg] = (reg[dst_reg] - imm) & 0xfffff;
                cpu->cycles += 3;
                reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_REG:
                reg[dst_reg] = reg[src_reg];
                cpu->cycles += 1;
                reg[MSP430_PC] = pc;
                break;
            case CMPA_REG: {
                uint32_t src = reg[src_reg];
                uint32_t dst = reg[dst_reg];
                uint32_t sr = reg[MSP430_SR] & ~(SR_N | SR_Z | SR_C | SR_V);
                if (dst >= src) sr |= SR_C;
                if (dst < src) sr |= SR_N;
                if (dst == src) sr |= SR_Z;
                int32_t tmp = (int32_t)dst - (int32_t)src;
                uint32_t b = 0x80000;
                if (((dst ^ (uint32_t)tmp) & b) == 0 && ((dst ^ src) & b) != 0) {
                    sr |= SR_V;
                }
                write_sr_flags(cpu, sr);
                cpu->cycles += 1;
                reg[MSP430_PC] = pc;
                break;
            }
            case ADDA_REG: {
                uint32_t src = reg[src_reg];
                uint32_t dst_val = reg[dst_reg];
                uint32_t result = dst_val + src;
                uint32_t sr = reg[MSP430_SR];
                sr &= ~(SR_C | SR_Z | SR_N | SR_V);
                if ((result & 0xfffff) == 0) sr |= SR_Z;
                if (result & 0x80000) sr |= SR_N;
                if (result > 0xfffff) sr |= SR_C;
                if (!((dst_val ^ src) & 0x80000) && ((src ^ result) & 0x80000)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                reg[dst_reg] = result & 0xfffff;
                cpu->cycles += 1;
                reg[MSP430_PC] = pc;
                break;
            }
            case SUBA_REG:
                reg[dst_reg] = (reg[dst_reg] - reg[src_reg]) & 0xfffff;
                cpu->cycles += 1;
                reg[MSP430_PC] = pc;
                break;
            default:
                fprintf(stderr, "Unsupported MSP430X instruction: 0x%04x op=0x%04x at PC=0x%05x\n",
                        instr, msp430x_op, pc - 2);
                reg[MSP430_PC] = pc;
                cpu->cycles += 1;
                break;
            }
            count--;
            continue;

        rrxx_dispatch: {
            /* RRCM/RRAM/RLAM/RRUM */
            int rr_count = ((instr >> 10) & 0x3) + 1;
            int rr_dst = instr & 0xf;
            int rr_type = instr & RRMASK;
            bool rr_word = (instr & 0x0010) == 0; /* bit 4: 0=word, 1=addr */
            uint32_t dst = reg[rr_dst];
            uint32_t sr = reg[MSP430_SR];
            uint32_t carry = (sr & SR_C) ? 1 : 0;
            uint32_t nxt_carry = 0;

            if (rr_word) {
                dst &= 0xffff;
            }

            pc += 2;
            cpu->cycles += 1 + rr_count;

            switch (rr_type) {
            case RRCM_OP: {
                uint32_t dst_low = dst & ((1 << rr_count) - 1);
                nxt_carry = (dst & (1 << (rr_count + 1))) ? SR_C : 0;
                dst >>= rr_count;
                if (rr_word) {
                    dst |= (dst_low << (17 - rr_count)) | (carry << (16 - rr_count));
                } else {
                    dst |= (dst_low << (21 - rr_count)) | (carry << (20 - rr_count));
                }
                break;
            }
            case RRAM_OP:
                if (dst & (rr_word ? 0x8000 : 0x80000)) {
                    dst |= (rr_word ? 0xf8000u : 0xf80000u);
                }
                dst >>= (rr_count - 1);
                nxt_carry = (dst & 1) ? SR_C : 0;
                dst >>= 1;
                break;
            case RLAM_OP:
                dst <<= (rr_count - 1);
                nxt_carry = (dst & (rr_word ? 0x8000 : 0x80000)) ? SR_C : 0;
                dst <<= 1;
                break;
            case RRUM_OP:
                dst >>= (rr_count - 1);
                nxt_carry = (dst & 1) ? SR_C : 0;
                dst >>= 1;
                break;
            }

            write_sr_flags(cpu, (sr & ~(SR_C | SR_V)) | nxt_carry);
            reg[rr_dst] = dst & (rr_word ? 0xffff : 0xfffff);
            reg[MSP430_PC] = pc;
            count--;
            continue;
        }

        calla_dispatch: {
            int dst_register = instr & 0xf;
            int calla_op = instr & CALLA_MASK;
            int sp, dst = -1;
            pc += 2;

            switch (calla_op) {
            case CALLA_REG:
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
                dst = reg[dst_register];
                cpu->cycles += 5;
                break;
            case CALLA_INDEX: {
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
                dst = reg[dst_register];
                uint16_t idx = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                int32_t index = sign_extend_16(idx);
                dst = (dst + index) & 0xfffff;
                dst = mem_read_mode(cpu, dst, 2);
                cpu->cycles += 5;
                break;
            }
            case CALLA_IMM: {
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                dst = ((dst_register << 16) | lo) & 0xfffff;
                cpu->cycles += 5;
                break;
            }
            case CALLA_IND: {
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
                uint32_t addr = reg[dst_register];
                dst = mem_read_mode(cpu, addr, 2);
                cpu->cycles += 5;
                break;
            }
            case CALLA_ABS: {
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t addr = ((dst_register << 16) | lo) & 0xfffff;
                dst = mem_read_mode(cpu, addr, 2);
                cpu->cycles += 7;
                break;
            }
            default:
                fprintf(stderr, "Unsupported CALLA mode: 0x%04x\n", instr);
                reg[MSP430_PC] = pc;
                count--;
                continue;
            }

            if (dst != -1) {
                sp = reg[MSP430_SP];
                mem_write(cpu, sp, (pc >> 16) & 0xf, true);
                sp -= 2;
                mem_write(cpu, sp, pc & 0xffff, true);
                reg[MSP430_SP] = sp;
                reg[MSP430_PC] = dst & 0xfffff;
            } else {
                reg[MSP430_PC] = pc;
            }
            count--;
            continue;
        }

        pushm_popm_dispatch: {
            int n = ((instr >> 4) & 0xf) + 1;
            int reg_no = instr & 0xf;
            int pm_op = instr & 0xff00;
            bool is_addr = (pm_op == PUSHM_A || pm_op == POPM_A);
            int size = is_addr ? 4 : 2;
            int sp = reg[MSP430_SP];

            pc += 2;

            if (pm_op == PUSHM_A || pm_op == PUSHM_W) {
                if (is_addr) cpu->cycles += 2;
                for (int i = 0; i < n; i++) {
                    sp -= size;
                    cpu->cycles += 2;
                    mem_write_mode(cpu, sp, reg[reg_no], is_addr ? 2 : 1);
                    reg_no--;
                    if (reg_no < 0) reg_no = 15;
                }
                reg[MSP430_SP] = sp;
            } else {
                /* POPM */
                if (is_addr) cpu->cycles += 2;
                for (int i = 0; i < n; i++) {
                    cpu->cycles += 2;
                    reg[reg_no] = mem_read_mode(cpu, sp, is_addr ? 2 : 1);
                    reg_no++;
                    sp += size;
                    if (reg_no > 15) reg_no = 0;
                }
                reg[MSP430_SP] = sp;
            }
            reg[MSP430_PC] = pc;
            count--;
            continue;
        }
        } /* end op_msp430x */

        /* ===================================================================
         * Single-operand instructions (opcode 1: 0x1000-0x1FFF)
         * =================================================================== */
        OP_LABEL(op_single): {
            int single_op = instr & 0x1380; /* mask to get operation */
            int dst_register = instr & 0xf;
            int ad = (instr >> 4) & 0x3;
            bool bw = (instr & 0x0040) != 0;
            uint32_t mask = bw ? 0xff : 0xffff;
            uint32_t msb = bw ? 0x80 : 0x8000;
            int mode_bytes = bw ? 1 : 2;

            pc += 2;

            bool dst_reg_mode = false;
            int dst_address = -1;
            int dst = -1;
            int sp;

            /* Pre-decrement SP for PUSH and CALL */
            if (single_op == OP_PUSH || single_op == OP_CALL) {
                sp = reg[MSP430_SP] - 2;
                reg[MSP430_SP] = sp;
            } else {
                sp = reg[MSP430_SP];
            }

            /* Resolve destination */
            if ((dst_register == MSP430_CG1 && ad > AM_INDEX) || dst_register == MSP430_CG2) {
                dst_reg_mode = true;
                cpu->cycles += 1;
            } else {
                switch (ad) {
                case AM_REG:
                    dst_reg_mode = true;
                    cpu->cycles += 1;
                    break;
                case AM_INDEX: {
                    int32_t idx = sign_extend_16(memory[pc] | (memory[pc + 1] << 8));
                    pc += 2;
                    if (dst_register == MSP430_SR) {
                        /* Absolute addressing mode */
                        dst_address = (uint32_t)(uint16_t)idx;
                    } else {
                        uint32_t rval = reg[dst_register];
                        if (rval <= 0xffff) {
                            dst_address = (rval + idx) & 0xffff;
                        } else {
                            dst_address = (rval + idx) & 0xfffff;
                        }
                    }
                    cpu->cycles += 4;
                    break;
                }
                case AM_IND_REG:
                    dst_address = reg[dst_register];
                    cpu->cycles += 3;
                    break;
                case AM_IND_AUTOINC:
                    if (dst_register == MSP430_PC) {
                        /* Immediate mode */
                        dst = memory[pc] | (memory[pc + 1] << 8);
                        pc += 2;
                        dst_address = -1;
                    } else {
                        dst_address = reg[dst_register];
                        reg[dst_register] = dst_address + mode_bytes;
                    }
                    cpu->cycles += 3;
                    break;
                }
            }

            /* Read value */
            if (dst_reg_mode) {
                if (dst_register == MSP430_CG2) {
                    dst = cg2_values[ad];
                } else if (dst_register == MSP430_CG1 && ad > AM_INDEX) {
                    dst = cg1_values[ad];
                } else {
                    dst = reg[dst_register];
                }
                dst &= mask;
            } else if (dst == -1 && dst_address != -1) {
                dst = mem_read(cpu, dst_address, !bw);
                if (bw) dst &= 0xff;
            }

            bool write_back = false;
            bool update_status = true;
            uint32_t sr = reg[MSP430_SR];
            uint32_t nxt_carry;

            switch (single_op) {
            case OP_RRC:
                nxt_carry = (dst & 1) ? SR_C : 0;
                dst = dst >> 1;
                dst |= (sr & SR_C) ? msb : 0;
                write_back = true;
                write_sr_flags(cpu, (sr & ~(SR_C | SR_V)) | nxt_carry);
                break;
            case OP_SWPB: {
                int tmp = dst;
                dst = ((tmp >> 8) & 0xff) | ((tmp << 8) & 0xff00);
                write_back = true;
                update_status = false;
                break;
            }
            case OP_RRA:
                nxt_carry = (dst & 1) ? SR_C : 0;
                dst = (dst & msb) | (dst >> 1);
                write_back = true;
                write_sr_flags(cpu, (sr & ~(SR_C | SR_V)) | nxt_carry);
                break;
            case OP_SXT:
                dst = (dst & 0x80) ? (dst | 0xfff00) : (dst & 0x7f);
                write_back = true;
                sr &= ~(SR_C | SR_V);
                if (dst != 0) sr |= SR_C;
                write_sr_flags(cpu, sr);
                mask = 0xffff; /* SXT result is word-size */
                msb = 0x8000;
                break;
            case OP_PUSH:
                mem_write(cpu, sp, dst, !bw);
                cpu->cycles += (ad == AM_REG || ad == AM_IND_AUTOINC) ? 2 : 1;
                write_back = false;
                update_status = false;
                break;
            case OP_CALL: {
                uint32_t cur_pc = pc;
                reg[MSP430_PC] = pc;
                mem_write(cpu, sp, cur_pc & 0xffff, true);
                reg[MSP430_PC] = dst & 0xffff;
                pc = dst & 0xffff;
                cpu->cycles += (ad == AM_REG) ? 3 : (ad == AM_IND_AUTOINC) ? 2 : 1;
                write_back = false;
                update_status = false;
                break;
            }
            case OP_RETI: {
                cpu->serviced_interrupt = -1;
                sp = reg[MSP430_SP];
                sr = mem_read(cpu, sp, true);
                write_sr(cpu, sr & 0x0fff);
                sp += 2;
                uint32_t ret_pc = mem_read(cpu, sp, true) | ((sr & 0xf000) << 4);
                reg[MSP430_PC] = ret_pc;
                pc = ret_pc;
                sp += 2;
                reg[MSP430_SP] = sp;
                write_back = false;
                update_status = false;
                cpu->cycles += 4;
                handle_pending_interrupts(cpu);
                break;
            }
            default:
                fprintf(stderr, "Unknown single-op: 0x%04x\n", instr);
                break;
            }

            /* Write back result */
            dst &= mask;
            if (write_back) {
                if (dst_reg_mode) {
                    reg[dst_register] = dst;
                } else if (dst_address != -1) {
                    mem_write(cpu, dst_address, dst, !bw);
                }
            }

            /* Update status flags (Z, N) */
            if (update_status) {
                sr = reg[MSP430_SR];
                sr = (sr & ~(SR_Z | SR_N)) |
                     ((dst == 0) ? SR_Z : 0) |
                     ((dst & msb) ? SR_N : 0);
                write_sr_flags(cpu, sr);
            }

            if (single_op != OP_CALL && single_op != OP_RETI) {
                reg[MSP430_PC] = pc;
            }
            count--;
            continue;
        }

        /* ===================================================================
         * Jump instructions (opcodes 2-3: 0x2000-0x3FFF)
         * =================================================================== */
        OP_LABEL(op_jump): {
            pc += 2;
            cpu->cycles += 2;

            int condition = (instr >> 10) & 0x7;
            int offset = instr & 0x3ff;
            /* Sign-extend 10-bit offset and multiply by 2 */
            if (offset & 0x200) offset |= 0xfc00;
            int32_t jump_offset = ((int16_t)offset) * 2;

            uint32_t sr = reg[MSP430_SR];
            bool jump;

            switch (condition) {
            case JMP_JNE: jump = (sr & SR_Z) == 0; break;
            case JMP_JEQ: jump = (sr & SR_Z) != 0; break;
            case JMP_JNC: jump = (sr & SR_C) == 0; break;
            case JMP_JC:  jump = (sr & SR_C) != 0; break;
            case JMP_JN:  jump = (sr & SR_N) != 0; break;
            case JMP_JGE: jump = ((sr & SR_N) != 0) == ((sr & SR_V) != 0); break;
            case JMP_JL:  jump = ((sr & SR_N) != 0) != ((sr & SR_V) != 0); break;
            case JMP_JMP: jump = true; break;
            default: jump = false; break;
            }

            if (jump) {
                pc = (pc + jump_offset) & (cpu->is_msp430x ? 0xfffff : 0xffff);
            }
            reg[MSP430_PC] = pc;
            count--;
            continue;
        }

        /* ===================================================================
         * Two-operand instructions (opcodes 4-F)
         *
         * Format: opcode(4) | src_reg(4) | Ad(1) | BW(1) | As(2) | dst_reg(4)
         * =================================================================== */

        extended_twoop: ; /* label for extension-word dispatch */

        /* Process all two-op instructions through shared operand resolution,
           then dispatch to individual operations */
        #define TWO_OP_BODY(operation) \
        OP_LABEL(op_##operation):

        TWO_OP_BODY(mov)
        TWO_OP_BODY(add)
        TWO_OP_BODY(addc)
        TWO_OP_BODY(subc)
        TWO_OP_BODY(sub)
        TWO_OP_BODY(cmp)
        TWO_OP_BODY(dadd)
        TWO_OP_BODY(bit)
        TWO_OP_BODY(bic)
        TWO_OP_BODY(bis)
        TWO_OP_BODY(xor)
        TWO_OP_BODY(and)
        {
            int src_register = (instr >> 8) & 0xf;
            int dst_register = instr & 0xf;
            int ad = (instr >> 7) & 0x1;
            bool bw = (instr & 0x0040) != 0;
            int as = (instr >> 4) & 0x3;

            uint32_t mask = bw ? 0xff : 0xffff;
            uint32_t msb_bit = bw ? 0x80 : 0x8000;
            int mode_bytes = bw ? 1 : 2;
            bool dst_reg_mode = (ad == 0);

            int src = 0;
            int dst = 0;
            int dst_address = -1;
            bool src_is_cg = false;

            pc += 2;

            /* ---- FAST PATH: reg/CG source, register destination ---- */
            if (__builtin_expect(dst_reg_mode && cpu->ext_word == 0, 1)) {
                if (src_register == MSP430_CG2) {
                    src = cg2_values[as];
                    if (bw && as == 3) src = 0xff;
                    cpu->cycles += 1;
                } else if (src_register == MSP430_CG1 && as >= 2) {
                    src = cg1_values[as];
                    cpu->cycles += 1;
                } else if (as == AM_REG) {
                    src = reg[src_register] & mask;
                    cpu->cycles += 1;
                    if (dst_register == MSP430_PC) cpu->cycles += 1;
                } else {
                    goto twoop_slow;
                }
                if (op != OP_MOV) dst = reg[dst_register] & mask;
                goto twoop_alu;
            }

            twoop_slow: ;
            /* --- Resolve source operand --- */
            if (src_register == MSP430_CG2) {
                /* CG2 (R3): constant generator */
                src = cg2_values[as];
                if (bw && as == 3) src = 0xff; /* #-1 in byte mode */
                src_is_cg = true;
                cpu->cycles += dst_reg_mode ? 1 : 4;
            } else if (src_register == MSP430_CG1 && as >= 2) {
                /* CG1 (R2/SR): As=2→4, As=3→8 */
                src = cg1_values[as];
                src_is_cg = true;
                cpu->cycles += dst_reg_mode ? 1 : 4;
            } else {
                switch (as) {
                case AM_REG:
                    src = reg[src_register] & mask;
                    cpu->cycles += dst_reg_mode ? 1 : 4;
                    if (dst_register == MSP430_PC) cpu->cycles += 1;
                    break;
                case AM_INDEX: {
                    int32_t idx = sign_extend_16(memory[pc] | (memory[pc + 1] << 8));
                    pc += 2;
                    uint32_t addr;
                    if (src_register == MSP430_SR) {
                        /* Absolute addressing mode: address is just the index word */
                        addr = (uint32_t)(uint16_t)idx;
                    } else {
                        uint32_t rval = reg[src_register];
                        if (rval <= 0xffff) {
                            addr = (rval + idx) & 0xffff;
                        } else {
                            addr = (rval + idx) & 0xfffff;
                        }
                    }
                    src = mem_read(cpu, addr, !bw);
                    if (bw) src &= 0xff;
                    cpu->cycles += dst_reg_mode ? 3 : 6;
                    break;
                }
                case AM_IND_REG:
                    src = mem_read(cpu, reg[src_register], !bw);
                    if (bw) src &= 0xff;
                    cpu->cycles += dst_reg_mode ? 2 : 5;
                    break;
                case AM_IND_AUTOINC:
                    if (src_register == MSP430_PC) {
                        /* Immediate mode: word follows instruction */
                        src = memory[pc] | (memory[pc + 1] << 8);
                        if (bw) src &= 0xff;
                        pc += 2;
                    } else {
                        uint32_t addr = reg[src_register];
                        src = mem_read(cpu, addr, !bw);
                        if (bw) src &= 0xff;
                        reg[src_register] = addr + mode_bytes;
                    }
                    cpu->cycles += dst_reg_mode ? 2 : 5;
                    if (dst_register == MSP430_PC) cpu->cycles += 1;
                    break;
                }
            }

            /* --- Resolve destination operand --- */
            if (dst_reg_mode) {
                if (op != OP_MOV) {
                    dst = reg[dst_register] & mask;
                }
            } else {
                /* Indexed destination */
                if (dst_register == MSP430_SR) {
                    /* Absolute addressing mode */
                    dst_address = memory[pc] | (memory[pc + 1] << 8);
                    pc += 2;
                } else {
                    uint32_t rval = reg[dst_register];
                    int32_t idx = sign_extend_16(memory[pc] | (memory[pc + 1] << 8));
                    pc += 2;
                    if (rval <= 0xffff) {
                        dst_address = (rval + idx) & 0xffff;
                    } else {
                        dst_address = (rval + idx) & 0xfffff;
                    }
                }
                if (op != OP_MOV) {
                    dst = mem_read(cpu, dst_address, !bw);
                    if (bw) dst &= 0xff;
                }
            }

            /* --- Execute ALU operation --- */
            twoop_alu: ;
            bool write_result = false;
            bool update_status = true;
            uint32_t sr = reg[MSP430_SR];

            switch (op) {
            case OP_MOV:
                dst = src;
                write_result = true;
                update_status = false;
                break;

            case OP_ADD: {
                sr &= ~(SR_V | SR_C);
                uint32_t tmp = (src ^ dst) & msb_bit;
                dst = dst + src;
                if ((uint32_t)dst > mask) sr |= SR_C;
                if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }

            case OP_ADDC: {
                int carry = (sr & SR_C) ? 1 : 0;
                sr &= ~(SR_V | SR_C);
                uint32_t tmp = (src ^ dst) & msb_bit;
                dst = dst + src + carry;
                if ((uint32_t)dst > mask) sr |= SR_C;
                if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }

            case OP_SUB: {
                src = (~src) & mask;
                sr &= ~(SR_V | SR_C);
                uint32_t tmp = (src ^ dst) & msb_bit;
                dst = dst + src + 1;
                if ((uint32_t)dst > mask) sr |= SR_C;
                if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }

            case OP_SUBC: {
                int carry = (sr & SR_C) ? 1 : 0;
                src = (~src) & mask;
                sr &= ~(SR_V | SR_C);
                uint32_t tmp = (src ^ dst) & msb_bit;
                dst = dst + src + carry;
                if ((uint32_t)dst > mask) sr |= SR_C;
                if (tmp == 0 && ((src ^ dst) & msb_bit)) sr |= SR_V;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }

            case OP_CMP: {
                sr = (sr & ~(SR_C | SR_V)) | ((uint32_t)dst >= (uint32_t)src ? SR_C : 0);
                int tmp = dst - src;
                if (((src ^ tmp) & msb_bit) == 0 && ((src ^ dst) & msb_bit) != 0) {
                    sr |= SR_V;
                }
                write_sr_flags(cpu, sr);
                dst = tmp;
                write_result = false;
                break;
            }

            case OP_DADD: {
                dst = dst + src + ((sr & SR_C) ? 1 : 0);
                write_result = true;
                break;
            }

            case OP_BIT: {
                dst = src & dst;
                sr = (sr & ~(SR_C | SR_V)) | ((dst != 0) ? SR_C : 0);
                write_sr_flags(cpu, sr);
                write_result = false;
                break;
            }

            case OP_BIC:
                dst = (~src) & dst;
                write_result = true;
                update_status = false;
                break;

            case OP_BIS:
                dst = src | dst;
                write_result = true;
                update_status = false;
                break;

            case OP_XOR: {
                sr &= ~(SR_C | SR_V);
                if ((src & msb_bit) && (dst & msb_bit)) sr |= SR_V;
                dst = src ^ dst;
                if (dst != 0) sr |= SR_C;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }

            case OP_AND: {
                sr &= ~(SR_C | SR_V);
                dst = src & dst;
                if (dst != 0) sr |= SR_C;
                write_sr_flags(cpu, sr);
                write_result = true;
                break;
            }
            } /* end switch(op) */

            /* Mask result */
            dst &= mask;

            /* Write back */
            if (write_result) {
                if (dst_reg_mode) {
                    if (dst_register == MSP430_SR) {
                        write_sr(cpu, dst);
                    } else {
                        reg[dst_register] = dst;
                    }
                } else {
                    mem_write(cpu, dst_address, dst, !bw);
                }
            }

            /* Update Z, N flags */
            if (update_status) {
                sr = reg[MSP430_SR];
                sr = (sr & ~(SR_Z | SR_N)) |
                     ((dst == 0) ? SR_Z : 0) |
                     ((dst & msb_bit) ? SR_N : 0);
                write_sr_flags(cpu, sr);
            }

            /* Don't overwrite PC if the instruction already wrote to it */
            if (!(dst_reg_mode && dst_register == MSP430_PC && write_result)) {
                reg[MSP430_PC] = pc;
            }
            cpu->ext_word = 0;
            count--;
            continue;
        }

#if !defined(__GNUC__) && !defined(__clang__)
        } /* end switch */
#endif
    } /* end while */

    return count;
}
