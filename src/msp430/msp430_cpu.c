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

/* Resolve RAM mirror addresses (e.g., F2617: 0x200-0x9FF -> 0x1100-0x18FF) */
static inline uint32_t resolve_mirror(const msp430_cpu_t *cpu, uint32_t addr) {
    const msp430_config_t *cfg = cpu->config;
    if (cfg->ram_mirror_size &&
        addr >= cfg->ram_mirror_start &&
        addr < cfg->ram_mirror_start + cfg->ram_mirror_size) {
        return cfg->ram_start + (addr - cfg->ram_mirror_start);
    }
    return addr;
}

/* Memory read with IO dispatch */
static inline int mem_read(msp430_cpu_t *cpu, uint32_t addr, bool word) {
    if (addr < cpu->max_mem_io && cpu->io_read[addr]) {
        return cpu->io_read[addr](cpu->io_user_data[addr], addr, word, cpu->cycles);
    }
    addr = resolve_mirror(cpu, addr);
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
    /* A compiled block spans up to MAX_BLOCK_SIZE instructions x 6 bytes,
     * so a block STARTING that far before the written address can still
     * cover it.  Scan the whole reach and free only blocks whose actual
     * [start_pc, end_pc) covers the written word — the old fixed
     * (addr-4)..(addr+2) window missed writes into the interior of any
     * block longer than a few instructions, leaving stale compiled code
     * live after self-modifying-code stores. */
    const uint32_t reach = MAX_BLOCK_SIZE * 6;
    uint32_t lo = (addr >= reach) ? (addr - reach) >> 1 : 0;
    uint32_t hi = (addr + 2) >> 1;
    if (hi >= cpu->cache_size) hi = cpu->cache_size - 1;
    for (uint32_t i = lo; i <= hi; i++) {
        compiled_block_t *cb = (compiled_block_t *)cpu->compiled_cache[i];
        if (!cb) continue;
        /* Word write touches addr and addr+1 */
        if (cb->end_pc > addr && cb->start_pc <= addr + 1) {
            msp430_jit_free(cb);
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
    addr = resolve_mirror(cpu, addr);
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
    cpu->last_execute_us = -1;

    /* JIT cache */
    cpu->cache_size = cpu->max_mem >> 1;
#ifdef HAVE_LIGHTNING
    cpu->compiled_cache = (void **)calloc(cpu->cache_size, sizeof(void *));
    cpu->block_exec_count = (int32_t *)calloc(cpu->cache_size, sizeof(int32_t));
    if (config->is_msp430x) {
        /* Disable JIT for MSP430X — the 0x1xxx opcode range overlaps
         * between single-op, CALLA, PUSHM/POPM, and extension words.
         * The JIT decoder can't reliably distinguish these, causing
         * data words to be compiled as RRXX instructions. */
        free(cpu->compiled_cache);
        cpu->compiled_cache = NULL;
        free(cpu->block_exec_count);
        cpu->block_exec_count = NULL;
    }
    cpu->jit_threshold = 100;
    cpu->jit_inblock_checks = 1;  /* default: on (needed for timer events in loops) */
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
    cpu->last_execute_us = -1;

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

/* --- Event management (generated from shared template) --- */

#include "event_queue.h"
EVENT_QUEUE_IMPL(msp430, msp430_cpu_t, msp430_event_t)

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

/* MSP430 DADD: BCD (binary-coded-decimal) addition of dst + src + carry-in,
 * one 4-bit nibble at a time, propagating a decimal carry.  Sets C on decimal
 * overflow out of the top nibble and clears V (spec-undefined; give it a
 * defined value).  Z/N are left to the generic post-op status update, which
 * only touches those two flags.  `*sr` carries the incoming C and receives the
 * updated C/V; the BCD result is returned (caller masks to byte/word width).
 * Shared by both execution paths so the two can't diverge — the previous
 * copies both did a plain binary add with no decimal carry and no C flag. */
static inline uint32_t msp430_dadd_bcd(uint32_t dst, uint32_t src,
                                       uint32_t mask, uint32_t *sr) {
    int carry = (*sr & SR_C) ? 1 : 0;
    /* One nibble per hex digit of the operand width: 0xFF→2 (byte),
     * 0xFFFF→4 (word), 0xFFFFF→5 (MSP430X 20-bit). */
    int nibbles = 0;
    for (uint32_t m = mask; m; m >>= 4) nibbles++;
    uint32_t result = 0;
    for (int i = 0; i < nibbles; i++) {
        int shift = i * 4;
        int digit = ((dst >> shift) & 0xF) + ((src >> shift) & 0xF) + carry;
        if (digit > 9) { digit += 6; carry = 1; } else { carry = 0; }
        result |= (uint32_t)(digit & 0xF) << shift;
    }
    *sr &= ~(SR_C | SR_V);
    if (carry) *sr |= SR_C;
    return result & mask;
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
            /* Match MSPSim: 16-bit complement even for byte mode.
             * This affects carry flag for SUB.B when src > dst. */
            src = (src ^ 0xffff) & 0xffff;
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
            src = (src ^ 0xffff) & 0xffff;
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
            dst = msp430_dadd_bcd(dst, src, mask, &sr);
            write_sr_flags(cpu, sr);
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
                    reg[di->dst_reg] = dst & 0xfffff;
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
            /* Slice budget (MSPSim: no instruction starts once
             * cycles >= maxCycles).  cycle_limit is INT64_MAX outside
             * msp430_step_until, so this only binds inside a timed slice. */
            if (cpu->cycles >= cpu->cycle_limit)
                return count;
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
                        cpu->next_event_cycle &&
                    cpu->cycles + cb->length * 6 < cpu->cycle_limit) {
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
                               cpu->cycles < cpu->cycle_limit &&
                               cpu->interrupt_max < 0) {
                            pc = cpu->reg[MSP430_PC];
                            ci = pc >> 1;
                            if (ci >= cpu->cache_size) break;
                            cb = (compiled_block_t *)
                                cpu->compiled_cache[ci];
                            if (!cb || count < cb->length) break;
                            if (cpu->cycles + cb->length * 6 >= cpu->cycle_limit)
                                break;  /* finish the slice in the interpreter */
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
            steps = 10000;
        } else if (remaining <= 10) {
            /* Single-step near target to match MSPSim's per-instruction
             * cycle check. Prevents LPM→event→ISR overshoot that causes
             * 29+ tick Timer B outliers and TSCH desync. */
            steps = 1;
        } else {
            steps = (int)(remaining / 3);
            if (steps < 1) steps = 1;
            if (steps > 10000) steps = 10000;
        }
        msp430_step(cpu, steps);
    }
    /* Process events that became due at the boundary.  The LPM handler
     * fast-forwards cycles to min(next_event_cycle, cycle_limit).  When
     * next_event_cycle == cycle_limit the loop exits before the top-of-
     * loop event check runs, leaving the timer interrupt unfired.
     * MSPSim fires events after every instruction — we must match that. */
    while (cpu->cycles >= cpu->next_event_cycle)
        execute_events(cpu);

    cpu->cycle_limit = INT64_MAX;
    /* Keep sim_time_ns synchronized with cycles */
    if (cpu->cpu_freq_hz > 0) {
        cpu->sim_time_ns = msp430_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
    }
}

int64_t msp430_step_micros(msp430_cpu_t *cpu, int64_t jump_us, int64_t execute_us) {
    if (jump_us < 0) jump_us = 0;
    if (execute_us < 0) execute_us = 0;

    /* Exact MSPSim stepMicros replication:
     * - lastMicrosDelta accumulates ALL jump values (never reset)
     * - lastMicrosCycles is set ONCE (first call) and NEVER changes
     * - maxCycles = lastMicrosCycles + ((lastMicrosDelta + executeMicros) * dcoFrq) / 1e6
     * This does ONE integer division for the total accumulated time,
     * bounding truncation error to 1 cycle total (not 1 per tick). */

    /* Initialize on first call */
    if (!cpu->micro_clock_ready) {
        cpu->last_micros_cycles = cpu->cycles;
        cpu->last_micros_delta = 0;
        cpu->micro_clock_ready = true;
    }

    /* Accumulate jump (matches: lastMicrosDelta += jumpMicros) */
    cpu->last_micros_delta += jump_us;

    /* Compute target from ORIGINAL base + TOTAL accumulated delta
     * (matches: maxCycles = lastMicrosCycles + ((lastMicrosDelta + executeMicros) * dcoFrq) / 1e6) */
    int64_t target_cycle = cpu->last_micros_cycles +
        ((cpu->last_micros_delta + execute_us) * (int64_t)cpu->cpu_freq_hz) / 1000000LL;

    if (target_cycle > cpu->cycles)
        msp430_step_until(cpu, target_cycle);

    if (cpu->cpu_off &&
        !(cpu->interrupts_enabled &&
          cpu->serviced_interrupt == -1 &&
          cpu->interrupt_max >= 0) &&
        cpu->cpu_freq_hz > 0 &&
        cpu->next_event_cycle > cpu->cycles) {
        return ((cpu->next_event_cycle - cpu->cycles) * 1000000LL) /
               cpu->cpu_freq_hz;
    }
    return 0;
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
        /* Slice budget: MSPSim's emulateOP loop stops before the first
         * instruction that would start at or past maxCycles.  Without this
         * per-instruction check an active CPU (e.g. a TSCH busy-wait) ran
         * up to several ms past the scheduler's time, and frames it then
         * transmitted were anchored at the (earlier) scheduler time. */
        if (cpu->cycles >= cpu->cycle_limit)
            return count;
        /* Match MSPSim execution order: INTERRUPTS → LPM → INSTRUCTION → EVENTS.
         * MSPSim fires events AFTER instruction execution (emulateOP line 2080),
         * while interrupts are checked BEFORE (line 913). This ensures that
         * after RETI, the interrupted context executes at least one instruction
         * before newly-fired timer events trigger the next ISR. */

        /* Interrupt processing (BEFORE instruction, matching MSPSim line 913) */
        if (cpu->interrupts_enabled && cpu->serviced_interrupt == -1
                && cpu->interrupt_max >= 0) {
            int pc = reg[MSP430_PC];
            pc = service_interrupt(cpu, pc);
            /* If interrupt entry pushed us past cycle_limit, stop.
             * This prevents overshoot when LPM wake → event → ISR
             * happens within a single step iteration. */
            if (cpu->cycles >= cpu->cycle_limit) {
                return count - 1;
            }
        }

        /* LPM check */
        if (cpu->cpu_off) {
            /* In low-power mode, advance to next event (capped by cycle_limit) */
            if (cpu->interrupts_enabled && cpu->interrupt_max >= 0) {
                /* Will service interrupt next iteration */
            } else {
                int64_t prev = cpu->cycles;
                int64_t target = cpu->next_event_cycle;
                if (target > cpu->cycle_limit) target = cpu->cycle_limit;
                cpu->cycles = target;
                if (target > prev)
                    cpu->lpm_ns += msp430_cycles_to_ns(target - prev,
                                                       cpu->cpu_freq_hz);
                /* Process events that became due during LPM fast-forward */
                if (cpu->cycles >= cpu->next_event_cycle)
                    execute_events(cpu);
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
        cpu->last_pc = pc;

        /* PC-range trace hook (for debugging specific firmware functions) */
        if (cpu->pc_trace_fn && pc >= cpu->pc_trace_lo && pc < cpu->pc_trace_hi)
            cpu->pc_trace_fn(cpu->pc_trace_data, pc, reg, memory);




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

                /* Dispatch the extended instruction.
                 * Two-op (opcodes 4-15): use extended two-op path.
                 * Single-op (opcode 1): use single-op path (which handles ext_word).
                 * Jump (opcodes 2-3): rare with extension; route to single-op. */
                if (op >= 4) {
                    goto extended_twoop;
                } else {
                    /* Single-op with extension word — route to op_single.
                     * op_single will check ext_word for 20-bit operand handling. */
                    goto op_single;
                }
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

            /* RRXX: RRCM/RRAM/RLAM/RRUM — 0000 nntt 01bb dddd
             * bits 7-6 must be 01, bits 5-4 select variant.
             * Exclude dst=R3 (CG2) and dst=R2 (SR) as these are never
             * valid targets and typically indicate data misinterpreted
             * as instructions. */
            if ((instr & 0xf0e0) == 0x0040 && (instr & 0xf) > 3) {
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
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_IND_AUTOINC: {
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
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
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
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
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
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
                /* Don't clobber PC if destination IS PC (e.g., BRA #addr) */
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
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
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
                break;
            }
            case SUBA_IMM: {
                uint16_t lo = memory[pc] | (memory[pc + 1] << 8);
                pc += 2;
                uint32_t imm = lo | ((uint32_t)src_reg << 16);
                reg[dst_reg] = (reg[dst_reg] - imm) & 0xfffff;
                cpu->cycles += 3;
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
                break;
            }
            case MOVA_REG:
                reg[dst_reg] = reg[src_reg];
                cpu->cycles += 1;
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
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
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
                break;
            }
            case SUBA_REG:
                reg[dst_reg] = (reg[dst_reg] - reg[src_reg]) & 0xfffff;
                cpu->cycles += 1;
                if (dst_reg != MSP430_PC) reg[MSP430_PC] = pc;
                break;
            default:
                fprintf(stderr, "Unsupported MSP430X instruction: 0x%04x op=0x%04x at PC=0x%05x\n",
                        instr, msp430x_op, pc - 2);
                reg[MSP430_PC] = pc;
                cpu->cycles += 1;
                break;
            }
            count--;
            goto post_instruction_events;

        rrxx_dispatch: {
            /* RRCM/RRAM/RLAM/RRUM */
            int rr_count = ((instr >> 10) & 0x3) + 1;
            int rr_dst = instr & 0xf;
            int rr_type = instr & RRMASK;
            bool rr_word = (instr & 0x0010) != 0; /* bit 4: 0=.A(20-bit), 1=.W(16-bit) per MSPSim */
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
                /* n rotates through carry pop original bits 0..n-1; the LAST
                 * bit shifted out — original bit n-1 — is the final carry
                 * (same convention as RRAM/RRUM below).  Was bit n+1. */
                nxt_carry = (dst & (1 << (rr_count - 1))) ? SR_C : 0;
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

            /* Update Z, N, C flags (matches MSPSim RRXX behavior) */
            {
                uint32_t result = dst & (rr_word ? 0xffff : 0xfffff);
                uint32_t msb = rr_word ? 0x8000 : 0x80000;
                sr = (sr & ~(SR_C | SR_V | SR_Z | SR_N)) | nxt_carry |
                     (result == 0 ? SR_Z : 0) |
                     (result & msb ? SR_N : 0);
                write_sr_flags(cpu, sr);
            }
            reg[rr_dst] = dst & (rr_word ? 0xffff : 0xfffff);
            reg[MSP430_PC] = pc;
            count--;
            goto post_instruction_events;
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
                goto post_instruction_events;
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
            goto post_instruction_events;
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
                /* POPM: Rdst is the LOWEST register. Pop n registers
                 * upward from Rdst to Rdst+n-1.
                 * PUSHM #n, Rhigh pushes Rhigh..Rlow (highest first).
                 * POPM #n, Rlow pops Rlow..Rhigh (lowest first). */
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
            goto post_instruction_events;
        }
        } /* end op_msp430x */

        /* ===================================================================
         * Single-operand instructions (opcode 1: 0x1000-0x1FFF)
         * =================================================================== */
        OP_LABEL(op_single): {
            /* MSP430X instructions in the 0x1xxx range overlap with
             * single-op opcodes.  Redirect to op_msp430x for:
             *   0x1800-0x1FFF: extension words (20-bit operand prefix)
             *   0x1300 with bits[7:4]>=4: CALLA variants
             *   0x1400-0x17FF: PUSHM/POPM */
            if (cpu->is_msp430x) {
                if ((instr & 0xf800) == 0x1800)
                    goto op_msp430x;
                if ((instr & 0xff00) == 0x1300 && (instr & 0x00f0) >= 0x0040)
                    goto op_msp430x;
                if ((instr & 0xfc00) >= 0x1400 && (instr & 0xfc00) <= 0x1700)
                    goto op_msp430x;
            }

            int single_op = instr & 0x1380; /* mask to get operation */
            int dst_register = instr & 0xf;
            int ad = (instr >> 4) & 0x3;
            bool bw = (instr & 0x0040) != 0;

            /* Extension word handling for single-op. Single-op
             * extended instructions have only ONE operand (the dst).
             * Per MSP430X reference and MSPSim (MSP430Core.java:1487
             * and around), bits 3:0 of the ext word hold the operand
             * 19:16. csim previously used bits 10:7 (the source bits)
             * which made indexed/absolute access to upper memory
             * compute the wrong address. */
            int sext = cpu->ext_word;
            uint32_t sext_dst_hi = 0;
            bool is_addr_mode = false;
            if (sext) {
                sext_dst_hi = sext & 0xf;  /* bits 3:0: operand 19:16 */
                /* A/L and BW interaction same as two-op */
                bool al = (sext >> 6) & 1;  /* bit 6: A/L per MSPSim */
                if (!al) {
                    is_addr_mode = true;  /* A/L=0: .A mode (20-bit) per MSPSim */
                    bw = false;
                }
            }

            uint32_t mask = is_addr_mode ? 0xfffff : (bw ? 0xff : 0xffff);
            uint32_t msb = is_addr_mode ? 0x80000 : (bw ? 0x80 : 0x8000);
            int mode_bytes = is_addr_mode ? 4 : (bw ? 1 : 2);

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
                    uint16_t idx16 = memory[pc] | (memory[pc + 1] << 8);
                    pc += 2;
                    if (dst_register == MSP430_SR) {
                        /* Absolute addressing mode */
                        dst_address = sext ? ((sext_dst_hi << 16) | idx16) & 0xfffff
                                           : (uint32_t)idx16;
                    } else {
                        uint32_t rval = reg[dst_register];
                        if (sext) {
                            uint32_t idx20 = ((sext_dst_hi << 16) | idx16) & 0xfffff;
                            dst_address = (rval + idx20) & 0xfffff;
                        } else {
                            int32_t idx = sign_extend_16(idx16);
                            if (rval <= 0xffff) {
                                dst_address = (rval + idx) & 0xffff;
                            } else {
                                dst_address = (rval + idx) & 0xfffff;
                            }
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
                        if (sext) dst = ((sext_dst_hi << 16) | dst) & 0xfffff;
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

            /* MSP430X repeat prefix (RPT #n / RPT Rn) on register-mode
             * single-operand instructions: RRAX/RRCX/SWPBX/SXTX execute
             * 1 + n times (n from ext[3:0] or Rn[3:0]); ZC (ext bit 5)
             * clears the carry-in of every RRCX repetition.  Same layout
             * as the two-operand repeat code.  Only the final repetition
             * falls through to the common flag/write-back path below; the
             * earlier ones are applied here.  Previously the count was
             * ignored, so `RPT R14; RRA R15` (mspgcc's `x >> n` idiom)
             * shifted exactly once. */
            if (sext && dst_reg_mode &&
                (single_op == OP_RRC || single_op == OP_RRA ||
                 single_op == OP_SWPB || single_op == OP_SXT)) {
                bool rep_in_reg = (sext >> 7) & 1;
                int n = sext & 0xf;
                int repeats = 1 + (rep_in_reg ? (int)(reg[n] & 0xf) : n);
                bool zero_carry = (sext & 0x20) != 0;
                for (int rep = 1; rep < repeats; rep++) {
                    switch (single_op) {
                    case OP_RRC: {
                        uint32_t c_in = zero_carry ? 0 : (sr & SR_C);
                        uint32_t c_out = (dst & 1) ? SR_C : 0;
                        dst = (dst >> 1) | (c_in ? msb : 0);
                        sr = (sr & ~(SR_C | SR_V)) | c_out;
                        break;
                    }
                    case OP_RRA: {
                        uint32_t c_out = (dst & 1) ? SR_C : 0;
                        dst = (dst & msb) | (dst >> 1);
                        sr = (sr & ~(SR_C | SR_V)) | c_out;
                        break;
                    }
                    case OP_SWPB:
                        dst = ((dst & 0xff) << 8) | ((dst >> 8) & 0xff);
                        break;
                    case OP_SXT:
                        dst = (dst & 0x80) ? (dst | 0xfff00) : (dst & 0x7f);
                        break;
                    default: break;
                    }
                    dst &= mask;
                    cpu->cycles += 1;
                }
                if (zero_carry && single_op == OP_RRC)
                    sr &= ~SR_C;   /* carry-in of the final repetition */
            }
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
                if (is_addr_mode) {
                    /* PUSHX.A: 20-bit push. The pre-decrement above already
                     * moved SP down by 2. Drop another 2 so that the 4-byte
                     * write covers SP..SP+3 — matches MSPSim's PUSH handler
                     * when AccessMode.WORD20 is used. */
                    sp -= 2;
                    reg[MSP430_SP] = sp;
                    mem_write_mode(cpu, sp, dst, 2);
                } else {
                    mem_write(cpu, sp, dst, !bw);
                }
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
            cpu->ext_word = 0;
            count--;
            goto post_instruction_events;
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
            goto post_instruction_events;
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

            /* Extension word: determine actual operand width.
             * A/L=0, BW=0: .W (word 16-bit)
             * A/L=0, BW=1: .A (address 20-bit)
             * A/L=1, BW=0: reserved (treat as .W)
             * A/L=1, BW=1: .B (byte 8-bit) */
            int ext = cpu->ext_word;
            bool is_addr_mode = false;  /* .A (20-bit) mode */
            uint32_t ext_src_hi = 0;    /* source bits 19:16 */
            uint32_t ext_dst_hi = 0;    /* destination bits 19:16 */
            if (ext) {
                bool al = (ext >> 6) & 1;  /* bit 6: A/L per MSPSim (0=.A, 1=.W/.B) */
                ext_src_hi = (ext >> 7) & 0xf;  /* bits 10:7: src 19:16 per MSPSim */
                ext_dst_hi = ext & 0xf;  /* bits 3:0: dst 19:16 */
                if (!al) {
                    /* A/L=0: .A mode (20-bit) regardless of BW — per MSPSim wordx20 */
                    is_addr_mode = true;
                    bw = false;
                }
                /* A/L=1: follow instruction BW bit (0=.W, 1=.B) — no change needed */
            }

            uint32_t mask = is_addr_mode ? 0xfffff : (bw ? 0xff : 0xffff);
            uint32_t msb_bit = is_addr_mode ? 0x80000 : (bw ? 0x80 : 0x8000);
            int mode_bytes = is_addr_mode ? 4 : (bw ? 1 : 2);
            int mem_mode = is_addr_mode ? 2 : (bw ? 0 : 1);  /* for mem_read_mode */
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
                    uint16_t idx16 = memory[pc] | (memory[pc + 1] << 8);
                    pc += 2;
                    uint32_t addr;
                    if (src_register == MSP430_SR) {
                        /* Absolute addressing mode: address is just the index word.
                         * With extension word, upper bits extend to 20-bit. */
                        addr = ext ? ((ext_src_hi << 16) | idx16) & 0xfffff
                                   : (uint32_t)idx16;
                    } else {
                        uint32_t rval = reg[src_register];
                        if (ext) {
                            /* Extension word: index is 20-bit unsigned,
                             * formed from ext upper bits + 16-bit index */
                            uint32_t idx20 = ((ext_src_hi << 16) | idx16) & 0xfffff;
                            addr = (rval + idx20) & 0xfffff;
                        } else {
                            int32_t idx = sign_extend_16(idx16);
                            if (rval <= 0xffff) {
                                addr = (rval + idx) & 0xffff;
                            } else {
                                addr = (rval + idx) & 0xfffff;
                            }
                        }
                    }
                    src = is_addr_mode ? mem_read_mode(cpu, addr, 2)
                                       : mem_read(cpu, addr, !bw);
                    if (bw) src &= 0xff;
                    if (is_addr_mode) src &= 0xfffff;
                    cpu->cycles += dst_reg_mode ? 3 : 6;
                    break;
                }
                case AM_IND_REG:
                    src = is_addr_mode ? mem_read_mode(cpu, reg[src_register], 2)
                                       : mem_read(cpu, reg[src_register], !bw);
                    if (bw) src &= 0xff;
                    if (is_addr_mode) src &= 0xfffff;
                    cpu->cycles += dst_reg_mode ? 2 : 5;
                    break;
                case AM_IND_AUTOINC:
                    if (src_register == MSP430_PC) {
                        /* Immediate mode: word follows instruction.
                         * With extension word, upper bits extend to 20-bit. */
                        src = memory[pc] | (memory[pc + 1] << 8);
                        if (ext) src = ((ext_src_hi << 16) | src) & 0xfffff;
                        if (bw) src &= 0xff;
                        pc += 2;
                    } else {
                        uint32_t addr = reg[src_register];
                        src = is_addr_mode ? mem_read_mode(cpu, addr, 2)
                                           : mem_read(cpu, addr, !bw);
                        if (bw) src &= 0xff;
                        if (is_addr_mode) src &= 0xfffff;
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
                    /* Absolute addressing mode.
                     * With extension word, upper bits extend to 20-bit. */
                    uint16_t addr16 = memory[pc] | (memory[pc + 1] << 8);
                    dst_address = ext ? ((ext_dst_hi << 16) | addr16) & 0xfffff
                                      : addr16;
                    pc += 2;
                } else {
                    uint32_t rval = reg[dst_register];
                    uint16_t idx16 = memory[pc] | (memory[pc + 1] << 8);
                    pc += 2;
                    if (ext) {
                        uint32_t idx20 = ((ext_dst_hi << 16) | idx16) & 0xfffff;
                        dst_address = (rval + idx20) & 0xfffff;
                    } else {
                        int32_t idx = sign_extend_16(idx16);
                        if (rval <= 0xffff) {
                            dst_address = (rval + idx) & 0xffff;
                        } else {
                            dst_address = (rval + idx) & 0xfffff;
                        }
                    }
                }
                if (op != OP_MOV) {
                    dst = is_addr_mode ? mem_read_mode(cpu, dst_address, 2)
                                       : mem_read(cpu, dst_address, !bw);
                    if (bw) dst &= 0xff;
                    if (is_addr_mode) dst &= 0xfffff;
                }
            }

            /* --- Repeat count from extension word --- */
            int repeats = 1;
            bool zero_carry = false;
            if (ext && dst_reg_mode) {
                bool rep_in_reg = (ext >> 7) & 1;  /* bit 7: repeat count in register */
                int ext3_0 = ext & 0xf;
                if (rep_in_reg) {
                    repeats = 1 + (reg[ext3_0] & 0xf);
                } else {
                    repeats = 1 + ext3_0;
                }
                zero_carry = (ext & 0x20) != 0;  /* bit 5: ZC (zero carry) */
            }

            /* --- Execute ALU operation (with repeat loop) --- */
            twoop_alu: ;
            /* Re-initialize repeats/zero_carry here because the fast path
             * jumps to twoop_alu via goto, skipping the initialization
             * at the declaration site (undefined behavior in C). */
            if (cpu->ext_word == 0) {
                repeats = 1;
                zero_carry = false;
            }
            bool write_result = false;
            bool update_status = true;
            uint32_t sr = reg[MSP430_SR];

            for (int rep = 0; rep < repeats; rep++) {
            /* For repeats > 1, re-read sr and optionally clear carry */
            if (rep > 0) {
                sr = reg[MSP430_SR];
                if (zero_carry) sr &= ~SR_C;
                /* Re-read dst from register for repeated operations */
                if (dst_reg_mode && op != OP_MOV)
                    dst = reg[dst_register] & mask;
                /* Re-read src from register too — when an X-mode RPT
                 * repeats e.g. ADDX R14,R14, src and dst share the same
                 * register, and src must reflect the live register
                 * value each iteration. Without this, src stays at the
                 * pre-loop value (e.g. 3) while dst doubles, producing
                 * 3 + 3 + 3 + 3 = 15 instead of 3 << 4 = 48.
                 *
                 * For non-register addressing modes (immediate, indexed,
                 * autoincrement) src was fetched once at instruction
                 * start; per the MSP430X spec, only register operands
                 * are re-read on each repeat. */
                if (!src_is_cg && as == AM_REG)
                    src = reg[src_register] & mask;
            }
            write_result = false;
            update_status = true;

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
                /* Match MSPSim: 16-bit complement even for byte mode */
                src = (src ^ 0xffff) & 0xffff;
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
                src = (src ^ 0xffff) & 0xffff;
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
                dst = msp430_dadd_bcd(dst, src, mask, &sr);
                write_sr_flags(cpu, sr);
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

            /* Write back — also mask to 20 bits for MSP430X register writes
             * (MSPSim writeRegister always does value &= 0xfffff) */
            if (write_result) {
                if (dst_reg_mode) {
                    if (dst_register == MSP430_SR) {
                        write_sr(cpu, dst);
                    } else {
                        reg[dst_register] = dst & 0xfffff;
                    }
                } else {
                    if (is_addr_mode) {
                        mem_write_mode(cpu, dst_address, dst, 2);
                    } else {
                        mem_write(cpu, dst_address, dst, !bw);
                    }
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

            } /* end repeat loop */

            /* Don't overwrite PC if the instruction already wrote to it */
            if (!(dst_reg_mode && dst_register == MSP430_PC && write_result)) {
                reg[MSP430_PC] = pc;
            }
            cpu->ext_word = 0;
            count--;
            goto post_instruction_events;
        }

        /* Process events AFTER instruction execution (matching MSPSim line 2080).
         * This ensures timer events fire after the instruction that crossed
         * the event cycle, not before the next instruction. Critical for
         * TSCH: after RETI, the interrupted context executes one instruction
         * before timer events trigger the next ISR. */
        post_instruction_events:
        if (cpu->cycles >= cpu->next_event_cycle) {
            execute_events(cpu);
        }

#if !defined(__GNUC__) && !defined(__clang__)
        } /* end switch */
#endif
    } /* end while */

    return count;
}
