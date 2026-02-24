/*
 * MSP430 CPU emulator — core state and execution API
 *
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * C reimplementation 2026.
 */
#ifndef MSP430_CPU_H
#define MSP430_CPU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward declare decoded types */
struct decoded_insn;
struct basic_block;

/* --- Register indices --- */
#define MSP430_PC   0
#define MSP430_SP   1
#define MSP430_SR   2
#define MSP430_CG1  2
#define MSP430_CG2  3

/* --- Status register bits --- */
#define SR_C    0x0001   /* Carry */
#define SR_Z    0x0002   /* Zero */
#define SR_N    0x0004   /* Negative */
#define SR_GIE  0x0008   /* General Interrupt Enable */
#define SR_CPUOFF 0x0010
#define SR_OSCOFF 0x0020
#define SR_SCG0   0x0040
#define SR_SCG1   0x0080
#define SR_V    0x0100   /* Overflow */

/* --- Two-operand instruction opcodes (upper 4 bits) --- */
#define OP_MOV   0x4
#define OP_ADD   0x5
#define OP_ADDC  0x6
#define OP_SUBC  0x7
#define OP_SUB   0x8
#define OP_CMP   0x9
#define OP_DADD  0xa
#define OP_BIT   0xb
#define OP_BIC   0xc
#define OP_BIS   0xd
#define OP_XOR   0xe
#define OP_AND   0xf

/* --- Single-operand instruction opcodes --- */
#define OP_RRC   0x1000
#define OP_SWPB  0x1080
#define OP_RRA   0x1100
#define OP_SXT   0x1180
#define OP_PUSH  0x1200
#define OP_CALL  0x1280
#define OP_RETI  0x1300

/* --- Jump condition codes (bits 12:10) --- */
#define JMP_JNE  0
#define JMP_JEQ  1
#define JMP_JNC  2
#define JMP_JC   3
#define JMP_JN   4
#define JMP_JGE  5
#define JMP_JL   6
#define JMP_JMP  7

/* --- Addressing modes (As field) --- */
#define AM_REG          0
#define AM_INDEX        1
#define AM_IND_REG      2
#define AM_IND_AUTOINC  3

/* --- MSP430X opcodes --- */
#define MOVA_IND          0x0000
#define MOVA_IND_AUTOINC  0x0010
#define MOVA_ABS2REG      0x0020
#define MOVA_INDX2REG     0x0030
#define MOVA_REG2ABS      0x0060
#define MOVA_REG2INDX     0x0070
#define MOVA_IMM2REG      0x0080
#define CMPA_IMM          0x0090
#define ADDA_IMM          0x00a0
#define SUBA_IMM          0x00b0
#define MOVA_REG          0x00c0
#define CMPA_REG          0x00d0
#define ADDA_REG          0x00e0
#define SUBA_REG          0x00f0

#define RRMASK            0x0300
#define RRCM_OP           0x0000
#define RRAM_OP           0x0100
#define RLAM_OP           0x0200
#define RRUM_OP           0x0300

#define CALLA_REG         0x1340
#define CALLA_INDEX       0x1350
#define CALLA_IND         0x1360
#define CALLA_IND_AUTOINC 0x1370
#define CALLA_ABS         0x1380
#define CALLA_EDE         0x1390
#define CALLA_IMM         0x13b0
#define CALLA_MASK        0xfff0

#define PUSHM_A           0x1400
#define PUSHM_W           0x1500
#define POPM_A            0x1600
#define POPM_W            0x1700

/* --- Forward declarations --- */
struct msp430_config;
typedef struct msp430_config msp430_config_t;

/* --- IO callback types --- */
typedef int  (*io_read_fn)(void *user_data, uint32_t addr, bool word, int64_t cycles);
typedef void (*io_write_fn)(void *user_data, uint32_t addr, int value, bool word, int64_t cycles);

/* --- Interrupt handler callback --- */
typedef void (*interrupt_handler_fn)(void *user_data, int vector);

/* --- Event callback --- */
typedef struct msp430_event msp430_event_t;
typedef void (*event_fn)(void *user_data, msp430_event_t *event);

struct msp430_event {
    int64_t          fire_cycle;
    int64_t          fire_ns;       /* wall-clock fire time (0 = cycle-based only) */
    event_fn         callback;
    void            *user_data;
    msp430_event_t  *next;
};

/* --- CPU state --- */
typedef struct msp430_cpu {
    /* Hot path — keep together for cache locality */
    uint32_t  reg[16];            /* 20-bit registers */
    int64_t   cycles;             /* total emulated cycles */
    int64_t   instructions;       /* total instructions executed */

    /* Memory — flat array */
    uint8_t  *memory;             /* 1MB for MSP430X, 64KB for MSP430 */
    uint32_t  max_mem;

    /* Interrupt state */
    int       interrupt_max;      /* highest pending, -1 if none */
    int       serviced_interrupt; /* currently servicing, -1 if none */
    bool      interrupts_enabled;
    bool      cpu_off;
    bool      stopping;

    /* Interrupt source callbacks */
    interrupt_handler_fn *interrupt_handler; /* per-vector callback */
    void                **interrupt_source;  /* per-vector user data */

    /* IO dispatch */
    io_read_fn  *io_read;         /* per-address function pointers */
    io_write_fn *io_write;
    void       **io_user_data;
    uint32_t     max_mem_io;

    /* Event queue */
    msp430_event_t *event_queue;
    int64_t         next_event_cycle;
    int64_t         cycle_limit;       /* max cycle for step_until (INT64_MAX = no limit) */

    /* Nanosecond simulation time */
    int64_t         sim_time_ns;       /* current simulation time in nanoseconds */
    uint32_t        cpu_freq_hz;       /* current CPU frequency (for ns<->cycle conversion) */

    /* Config */
    const msp430_config_t *config;
    bool     is_msp430x;
    int      max_interrupt;

    /* Instruction state (for debugging/tracing) */
    uint16_t  last_instruction;
    int       ext_word;

    uint32_t             cache_size;    /* max_mem >> 1 */

#ifdef HAVE_LIGHTNING
    /* JIT compilation */
    void     **compiled_cache;   /* compiled_cache[pc>>1] */
    int32_t   *block_exec_count; /* hot block detection */
    int        jit_threshold;    /* default: 100 */
    int        jit_inblock_checks; /* emit interrupt checks inside JIT blocks */
#endif
} msp430_cpu_t;

/* --- Public API --- */

/* Lifecycle */
void msp430_cpu_init(msp430_cpu_t *cpu, const msp430_config_t *config);
void msp430_cpu_destroy(msp430_cpu_t *cpu);
void msp430_cpu_reset(msp430_cpu_t *cpu);

/* Execution — returns number of instructions remaining (0 = all executed) */
int  msp430_step(msp430_cpu_t *cpu, int count);
void msp430_step_until(msp430_cpu_t *cpu, int64_t target_cycle);
void msp430_stop(msp430_cpu_t *cpu);

/* IO */
void msp430_register_io(msp430_cpu_t *cpu, uint32_t addr, uint32_t size,
                         io_read_fn read, io_write_fn write, void *data);

/* Interrupts */
void msp430_flag_interrupt(msp430_cpu_t *cpu, int vector, void *source,
                            interrupt_handler_fn handler, bool set);

/* Events */
void msp430_schedule_event(msp430_cpu_t *cpu, msp430_event_t *ev, int64_t cycle);
void msp430_schedule_event_ns(msp430_cpu_t *cpu, msp430_event_t *ev, int64_t fire_ns);
void msp430_cancel_event(msp430_cpu_t *cpu, msp430_event_t *ev);

/* CPU frequency management */
void msp430_cpu_set_frequency(msp430_cpu_t *cpu, uint32_t freq_hz);

/* Nanosecond <-> cycle conversion helpers */
static inline int64_t msp430_ns_to_cycles(int64_t ns, uint32_t freq_hz) {
    return ns * (int64_t)freq_hz / 1000000000LL;
}
static inline int64_t msp430_cycles_to_ns(int64_t cycles, uint32_t freq_hz) {
    if (freq_hz == 0) return 0;
    return cycles * 1000000000LL / (int64_t)freq_hz;
}

/* Memory access (for external use / tests) */
static inline uint16_t msp430_read_word(const msp430_cpu_t *cpu, uint32_t addr) {
    return (uint16_t)(cpu->memory[addr] | (cpu->memory[addr + 1] << 8));
}

static inline void msp430_write_word(msp430_cpu_t *cpu, uint32_t addr, uint16_t val) {
    cpu->memory[addr]     = val & 0xff;
    cpu->memory[addr + 1] = (val >> 8) & 0xff;
}

static inline uint8_t msp430_read_byte(const msp430_cpu_t *cpu, uint32_t addr) {
    return cpu->memory[addr];
}

static inline void msp430_write_byte(msp430_cpu_t *cpu, uint32_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

#endif /* MSP430_CPU_H */
