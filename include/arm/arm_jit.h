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
 * A compiled block runs to completion and returns the number of ARM
 * instructions it executed (always its full length — v1 blocks are
 * straight-line with no early exit).
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
    jit_state_t    *jit_state;  /* owns the code memory; freed on release  */
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
