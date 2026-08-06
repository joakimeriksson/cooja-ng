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
#include "cpu_event.h"

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
#define APSR_Q  (1u << 27)  /* Sticky saturation (M4 DSP / saturating insns) */

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

/* --- Event callback ---
 * Unified per-CPU event type lives in include/common/cpu_event.h.
 * Aliases below keep existing ARM-typed call sites compiling. */
typedef cpu_event_t  arm_event_t;
typedef cpu_event_fn arm_event_fn;

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

    /* WFE event latch: set by SEV / by exception entry / by external event,
     * consumed (cleared) by the first WFE encountered. */
    uint8_t   event_latch;

    /* Memory — separate regions */
    uint8_t  *rom;                /* ROM_SIZE bytes */
    uint8_t  *flash;              /* FLASH_SIZE bytes */
    uint8_t  *sram;               /* SRAM_SIZE bytes */

    /* Interrupt/exception state */
    bool      interrupts_enabled;
    bool      cpu_off;            /* WFI sleep */
    bool      stopping;
    int64_t   lpm_ns;             /* cumulative time fast-forwarded in WFI (energy) */
    uint8_t   wild_trapped;       /* ARM_WILD_TRAP one-shot diagnostic */
    uint8_t   lr_trapped;         /* ARM_LR_WATCH one-shot diagnostic */

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
    /* Anchor for cycle->ns conversion across freq changes — see
     * msp430_cpu.h for rationale. */
    int64_t      anchor_cycles;
    int64_t      anchor_sim_time_ns;

    /* Config */
    const arm_config_t *config;

    /* Per-instance memory layout (cached from config for hot-path use).
     * `*_end` are pre-computed (base + size) to save one add per access. */
    uint32_t  flash_base, flash_end;
    uint32_t  sram_base,  sram_end;
    uint32_t  rom_size;     /* 0 if SoC has no ROM region (e.g. nRF52840) */
    /* Effective default vector table address used at reset. Seeded
     * from `config->vtor_default` by arm_cpu_init; can be overridden
     * by the SoC init op from the platform config (e.g. nrf52840
     * Dongle = 0x1000, DK = 0). 0 → use SoC-specific discovery
     * (CC2538 CCA) or fall back to flash_base. */
    uint32_t  vtor_default;

    /* Optional co-processor stepped in lockstep after each execute slice,
     * sharing this CPU's memory + IO bus. The nRF54L15 FLPR (RV32E) uses
     * this: nrf54l_vpr_launch() sets both, and the ARM execute tick calls
     * coproc_step(coproc, delta_cycles) once the M33 slice has advanced.
     * NULL on every SoC that has no co-processor. */
    void  *coproc;
    void (*coproc_step)(void *coproc, int64_t delta_cycles);
    /* Raise an interrupt on the coprocessor (set the given machine-interrupt
     * pending bit + wake it from WFI). The nRF54L15 GRTC calls this when a
     * compare in the FLPR's IRQ group (INTEN0 / GRTC_0) fires. */
    void (*coproc_raise_irq)(void *coproc, uint32_t mip_bit);

    /* Cortex-M4F VFP — single-precision (32 × s0..s31). Stored as raw
     * 32-bit words; arm_vfp.c interprets them per the IEEE 754 binary32
     * format when arithmetic ops touch them. fpscr holds the FP status
     * (NZCV flags from VCMP, IXC/UFC/OFC/DZC/IOC exception bits). */
    uint32_t  vfp_s[32];
    uint32_t  fpscr;

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

    /* WFI fast-forward guard. When WFI is hit with PRIMASK set and an
     * IRQ pending, the firmware is genuinely idle on real silicon — the
     * CPU sleeps until PRIMASK clears. csim can mirror that by advancing
     * cycles to the next scheduled event, except some peripherals have
     * state that *must* tick instruction-by-instruction (radio mid-TX,
     * mid-RX frame parse, channel-busy windows in the medium model).
     * The chip layer registers a guard via arm_set_wfi_skip_guard()
     * returning non-zero whenever such state is in flight; the WFI
     * handler then runs at full speed instead of skipping ahead. */
    int     (*wfi_skip_guard)(void *user);
    void     *wfi_skip_user;

    /* Debug: non-zero enables debug tracing */
    int       debug_flags;

    /* PC trace callback: called after each instruction if non-NULL */
    void    (*pc_callback)(void *user_data, uint32_t pc);
    void     *pc_callback_data;

    /* Optional GDB stub (gdb_stub_t *) attached via arm_attach_gdb.
     * When non-NULL, arm_step checks for breakpoints and halt requests
     * before each instruction. Untyped here so the header doesn't need
     * to pull in gdb_stub.h for non-debug builds. */
    void     *gdb_stub;

    /* ARM_SP_AUDIT shadow call stack: every BL/BLX pushes
     * (return_pc, sp_before_call, callee_pc); every observed PC match
     * pops + checks SP delta.  Catches sub-functions whose push/pop are
     * unbalanced — including stmdb/ldmia with mismatched reglists.
     * Placed last so it doesn't perturb existing struct field offsets. */
#define ARM_SP_AUDIT_DEPTH 64
    struct {
        uint32_t return_pc;
        uint32_t saved_sp;
        uint32_t callee_pc;
        uint8_t  in_exception;
    } sp_audit[ARM_SP_AUDIT_DEPTH];
    int       sp_audit_top;
    int       sp_audit_overflow;

    /* JIT block cache (src/arm/arm_jit.c).  Declared unconditionally — not
     * under #ifdef HAVE_LIGHTNING — so the struct layout is identical whether
     * or not Lightning was detected, and a stale object file can't disagree
     * with a fresh one about field offsets.  All NULL/0 when the JIT is off.
     *
     * Indexed by (pc - flash_base) >> 1, i.e. one slot per possible Thumb
     * instruction address in flash. */
    void    **jit_cache;         /* arm_compiled_block_t* per slot        */
    int32_t  *jit_exec_count;    /* hot-block detection; <0 = don't retry  */
    uint32_t  jit_cache_size;    /* 0 = JIT disabled for this CPU          */
    int       jit_threshold;     /* executions before compiling (env-set)  */
    int       jit_verify;        /* CSIM_ARM_JIT_VERIFY=1 lockstep check   */
    int32_t   jit_iter_budget;   /* loop blocks: iterations still allowed  */
    uint64_t  jit_blocks_run;    /* diagnostics: compiled-block entries    */
    uint64_t  jit_insns_run;     /* diagnostics: instructions via the JIT  */
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
/* Cortex-M4F VFP step — defined in arm_vfp.c. Returns true if hw1/hw2
 * was handled, false otherwise (caller should fault loudly). */
bool arm_vfp_step(arm_cpu_t *cpu, uint16_t hw1, uint16_t hw2);

void arm_register_io(arm_cpu_t *cpu, uint32_t base, uint32_t size,
                     arm_io_read_fn read, arm_io_write_fn write, void *data);

/* Install the WFI fast-forward guard. See wfi_skip_guard in arm_cpu_t. */
static inline void arm_set_wfi_skip_guard(arm_cpu_t *cpu,
                                           int (*guard)(void *user),
                                           void *user) {
    cpu->wfi_skip_guard = guard;
    cpu->wfi_skip_user  = user;
}

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
