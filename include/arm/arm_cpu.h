/*
 * ARM Cortex-M3 CPU emulator — core state and execution API
 */
#ifndef ARM_CPU_H
#define ARM_CPU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include "cpu_time.h"

/* --- Register indices --- */
#define ARM_R0   0
#define ARM_R1   1
#define ARM_R2   2
#define ARM_R3   3
#define ARM_R4   4
#define ARM_R5   5
#define ARM_R6   6
#define ARM_R7   7
#define ARM_R8   8
#define ARM_R9   9
#define ARM_R10  10
#define ARM_R11  11
#define ARM_R12  12
#define ARM_SP   13
#define ARM_LR   14
#define ARM_PC   15

/* --- APSR flag positions (in xPSR) --- */
#define APSR_N  (1u << 31)  /* Negative */
#define APSR_Z  (1u << 30)  /* Zero */
#define APSR_C  (1u << 29)  /* Carry */
#define APSR_V  (1u << 28)  /* Overflow */

/* --- Exception numbers --- */
#define EXC_RESET       1
#define EXC_NMI         2
#define EXC_HARDFAULT   3
#define EXC_MEMMANAGE   4
#define EXC_BUSFAULT    5
#define EXC_USAGEFAULT  6
#define EXC_SVCALL     11
#define EXC_PENDSV     14
#define EXC_SYSTICK    15
#define EXC_IRQ0       16

/* Max exception number for CC2538 (IRQ 0-178 -> exception 16-194) */
#define ARM_MAX_EXCEPTIONS 195

/* --- Memory region sizes --- */
#define ARM_ROM_SIZE      (128 * 1024)   /* 128KB ROM */
#define ARM_FLASH_SIZE    (512 * 1024)   /* 512KB Flash */
#define ARM_SRAM_SIZE     (32 * 1024)    /* 32KB SRAM */

/* --- Memory map addresses --- */
#define ARM_ROM_BASE      0x00000000
#define ARM_FLASH_BASE    0x00200000
#define ARM_SRAM_BASE     0x20000000
#define ARM_IO_BASE       0x40000000
#define ARM_BITBAND_BASE  0x42000000
#define ARM_SYSTEM_BASE   0xE0000000

/* --- IO region dispatch --- */
#define ARM_MAX_IO_REGIONS 64

typedef int  (*arm_io_read_fn)(void *user_data, uint32_t addr);
typedef void (*arm_io_write_fn)(void *user_data, uint32_t addr, uint32_t value);

typedef struct {
    uint32_t        base;
    uint32_t        size;
    arm_io_read_fn  read;
    arm_io_write_fn write;
    void           *user_data;
} arm_io_region_t;

/* --- Event callback --- */
typedef struct arm_event arm_event_t;
typedef void (*arm_event_fn)(void *user_data, arm_event_t *event);

struct arm_event {
    int64_t       fire_cycle;
    int64_t       fire_ns;       /* wall-clock fire time (0 = cycle-based only) */
    arm_event_fn  callback;
    void         *user_data;
    arm_event_t  *next;
};

/* --- Forward declarations --- */
struct arm_config;
typedef struct arm_config arm_config_t;

/* --- CPU state --- */
typedef struct arm_cpu {
    /* Hot path — keep together for cache locality */
    uint32_t  reg[16];            /* R0-R15 */
    uint32_t  xpsr;               /* Combined xPSR (APSR + IPSR + EPSR) */
    int64_t   cycles;             /* total emulated cycles */
    int64_t   instructions;       /* total instructions executed */

    /* Stack pointers */
    uint32_t  msp;                /* Main Stack Pointer */
    uint32_t  psp;                /* Process Stack Pointer */
    bool      use_psp;            /* true = Thread mode uses PSP */

    /* IT block state (ITSTATE byte: bits[7:4]=condition, bits[3:0]=mask) */
    uint8_t   it_state;

    /* PRIMASK, FAULTMASK, BASEPRI */
    uint32_t  primask;
    uint32_t  faultmask;
    uint32_t  basepri;

    /* Memory — separate regions */
    uint8_t  *rom;                /* ROM_SIZE bytes */
    uint8_t  *flash;              /* FLASH_SIZE bytes */
    uint8_t  *sram;               /* SRAM_SIZE bytes */

    /* Interrupt/exception state */
    bool      interrupts_enabled;
    bool      cpu_off;            /* WFI sleep */
    bool      stopping;

    /* IO dispatch */
    arm_io_region_t io_regions[ARM_MAX_IO_REGIONS];
    int             num_io_regions;

    /* Event queue */
    arm_event_t *event_queue;
    int64_t      next_event_cycle;
    int64_t      cycle_limit;
    int64_t      last_execute_us;    /* last Cooja-style execute() timestamp */
    int64_t      last_micros_cycles; /* MSPSim: cycle base for stepMicros (set once) */
    int64_t      last_micros_delta;  /* MSPSim: accumulated µs across all stepMicros calls */
    bool         micro_clock_ready;  /* MSPSim: first stepMicros call done */
    double       step_cycle_remainder; /* fractional µs for clock deviation */

    /* Nanosecond simulation time */
    int64_t      sim_time_ns;
    uint32_t     cpu_freq_hz;

    /* Config */
    const arm_config_t *config;

    /* Vector table offset register */
    uint32_t  vtor;

    /* ROM utility traps */
    uint32_t  rom_util_memcpy;    /* Address of rom_util_memcpy entry */
    uint32_t  rom_util_memset;    /* Address of rom_util_memset entry */
    uint32_t  rom_util_memcmp;    /* Address of rom_util_memcmp entry */

    /* Firmware helper traps (resolved from ELF symbols) */
    uint32_t  fw_udivmoddi4;      /* __udivmoddi4 */
    uint32_t  fw_aeabi_uldivmod;  /* __aeabi_uldivmod */

    /* Back-pointer to NVIC (set by arm_nvic_init) */
    void     *nvic;

    /* Debug: non-zero enables debug tracing */
    int       debug_flags;

    /* PC trace callback: called after each instruction if non-NULL */
    void    (*pc_callback)(void *user_data, uint32_t pc);
    void     *pc_callback_data;
} arm_cpu_t;

/* --- Public API --- */

/* Lifecycle */
void arm_cpu_init(arm_cpu_t *cpu, const arm_config_t *config);
void arm_cpu_destroy(arm_cpu_t *cpu);
void arm_cpu_reset(arm_cpu_t *cpu);

/* Execution */
int  arm_step(arm_cpu_t *cpu, int count);
void arm_step_until(arm_cpu_t *cpu, int64_t target_cycle);
int64_t arm_step_micros(arm_cpu_t *cpu, int64_t jump_us, int64_t execute_us);
void arm_stop(arm_cpu_t *cpu);

/* IO region registration */
void arm_register_io(arm_cpu_t *cpu, uint32_t base, uint32_t size,
                     arm_io_read_fn read, arm_io_write_fn write, void *data);

/* Memory access (for external use / tests / peripherals) */
uint32_t arm_read32(arm_cpu_t *cpu, uint32_t addr);
uint16_t arm_read16(arm_cpu_t *cpu, uint32_t addr);
uint8_t  arm_read8(arm_cpu_t *cpu, uint32_t addr);
void     arm_write32(arm_cpu_t *cpu, uint32_t addr, uint32_t val);
void     arm_write16(arm_cpu_t *cpu, uint32_t addr, uint16_t val);
void     arm_write8(arm_cpu_t *cpu, uint32_t addr, uint8_t val);

/* Events */
void arm_schedule_event(arm_cpu_t *cpu, arm_event_t *ev, int64_t cycle);
void arm_schedule_event_ns(arm_cpu_t *cpu, arm_event_t *ev, int64_t fire_ns);
void arm_cancel_event(arm_cpu_t *cpu, arm_event_t *ev);

/* CPU frequency management */
void arm_cpu_set_frequency(arm_cpu_t *cpu, uint32_t freq_hz);

/* Nanosecond <-> cycle conversion: provided by cpu_time.h as macros
 * arm_ns_to_cycles -> cpu_ns_to_cycles
 * arm_cycles_to_ns -> cpu_cycles_to_ns */

/* Exception/interrupt triggering (called by NVIC) */
void arm_exception_entry(arm_cpu_t *cpu, int exception_num);
void arm_check_pending_exceptions(arm_cpu_t *cpu);

#endif /* ARM_CPU_H */
