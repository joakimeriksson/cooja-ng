/*
 * arm_jit — GNU Lightning code generation for hot ARM basic blocks.
 *
 * Built on `arm_decode.c`: the decoder turns flash bytes into an
 * `arm_basic_block_t` of `arm_decoded_insn_t`, this file turns that block into
 * native host code.  The subset it accepts is exactly the decoder's subset —
 * anything else leaves `all_supported == false` and the block is refused, so
 * an unmodelled instruction can only ever be *interpreted*, never mis-compiled.
 *
 * The contract the generated code must satisfy is `arm_execute_decoded()` in
 * arm_cpu.c: identical registers, identical APSR, identical cycle count.
 * `arm-decode` already differential-tests that reference model against the
 * interpreter; `CSIM_ARM_JIT_VERIFY=1` then differential-tests the *generated
 * code* against the interpreter, in lockstep, per block.
 *
 * That verify mode exists because of the one correctness bug the MSP430 JIT
 * ever shipped: an inlined `ADDC` left the overflow flag stale, and because it
 * only ran once a block had gone hot, no ordinary test could reach it.  A
 * flag-only divergence is invisible until some later conditional branch goes
 * the wrong way — which is why verification here is lockstep state comparison,
 * not "do the tests still pass".
 */
#ifndef ARM_JIT_H
#define ARM_JIT_H

#ifdef HAVE_LIGHTNING

#include "arm_cpu.h"
#include "arm_decode.h"

#include <lightning.h>

/*
 * THE SIDE-EXIT CONTRACT
 *
 * A compiled block normally runs to its end and leaves `cpu->jit_partial` at
 * the -1 the dispatcher set before the call.  A guarded memory access that
 * misses (address outside SRAM, or misaligned) instead takes a **side exit**:
 * it leaves architectural state exactly as if only insns[0..i-1] had executed,
 * sets `cpu->reg[ARM_PC]` to insns[i].pc, writes `i` to `cpu->jit_partial`,
 * and — inside a loop block — hands its iteration-budget decrement back, so
 * `iter_budget` still counts only *completed* iterations.
 *
 * The interpreter then re-executes insns[i] through the full memory path,
 * including IO callbacks, bit-band aliases and unaligned splits.  Nothing the
 * JIT declined to model can be reached from generated code, which is what
 * keeps "a store might fire a peripheral callback" from being a correctness
 * problem: such a store is never the one that ran inline.
 *
 * The generated code's return value is unused; the dispatcher reconstructs the
 * instruction and cycle counts from `jit_partial`, `jit_iter_budget` and
 * `cyc_prefix`, because a loop block's iteration count is only known after it
 * has run.
 */
/*
 * The block cache is direct-mapped with a tag check (`start_pc`), not a slot
 * per flash address.  A slot-per-address table would be 4-12 MB per node
 * depending on flash size, and csim routinely runs 4+ nodes; this is a fixed
 * 768 KB per node regardless of the part.  A collision just costs a recompile,
 * and the tag check keeps it correct.
 */
#define ARM_JIT_CACHE_SLOTS 65536u   /* power of two */

typedef int (*arm_compiled_fn)(arm_cpu_t *cpu);

typedef struct arm_compiled_block {
    arm_compiled_fn fn;
    int             length;     /* ARM instructions covered                */
    uint32_t        start_pc;
    uint32_t        end_pc;     /* first byte past the block               */
    int             is_loop;    /* self-loop: runs natively under a budget */
    int             has_store;  /* block writes SRAM — see the verify note   */
    jit_state_t    *jit_state;  /* owns the code memory; freed on release  */

    /*
     * Cycle cost is a sum, not a count.  Register-only instructions cost 1
     * cycle but every load/store costs 2 (arm_step's central charge plus the
     * handler's own), so a block's cost stopped being its length the moment
     * memory ops were admitted.  `cyc_prefix[i]` is the cost of insns[0..i-1],
     * which is exactly what a side exit at instruction `i` has to charge;
     * `cyc_prefix[length]` is the whole block.  Both are computed once at
     * compile time so the dispatcher does a table lookup, not a loop.
     */
    uint16_t        cyc_prefix[ARM_MAX_BLOCK_SIZE + 1];
    int             cycles_total;
} arm_compiled_block_t;

/* Lightning's init_jit()/finish_jit() are process-global, and a mixed
 * MSP430+ARM simulation (configs/cross-level-demo.json) initialises both
 * JITs — so the once-only guard has to be shared, not per-module. */
void csim_lightning_init(void);

/* Compile `block` to native code, or return NULL if it is not compilable
 * (unsupported instruction, too short to be worth a call, emit failure). */
arm_compiled_block_t *arm_jit_compile(const arm_basic_block_t *block,
                                      arm_cpu_t *cpu);

void arm_jit_free(arm_compiled_block_t *cb);

#endif /* HAVE_LIGHTNING */
#endif /* ARM_JIT_H */
