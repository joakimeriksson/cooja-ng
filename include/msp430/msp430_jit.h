/*
 * MSP430 JIT compiler using GNU Lightning
 *
 * Compiles hot basic blocks into native code for the host architecture.
 */
#ifndef MSP430_JIT_H
#define MSP430_JIT_H

#ifdef HAVE_LIGHTNING

#include "msp430_cpu.h"
#include "msp430_decode.h"
#include <lightning.h>

/*
 * Compiled function signature:
 *   int compiled_fn(msp430_cpu_t *cpu)
 * Returns: number of instructions executed (>0)
 */
typedef int (*compiled_fn)(msp430_cpu_t *cpu);

typedef struct {
    compiled_fn   fn;          /* Native function pointer */
    int           length;      /* Number of MSP430 instructions */
    jit_state_t  *jit_state;   /* For cleanup (owns the code) */
} compiled_block_t;

/* Initialize JIT subsystem (call once at startup) */
void msp430_jit_init(void);

/* Shut down JIT subsystem */
void msp430_jit_finish(void);

/*
 * Compile a basic block into native code.
 * Returns NULL if the block is not worth compiling (all fallback).
 */
compiled_block_t *msp430_jit_compile(basic_block_t *block, msp430_cpu_t *cpu);

/* Free a compiled block */
void msp430_jit_free(compiled_block_t *cb);

#endif /* HAVE_LIGHTNING */
#endif /* MSP430_JIT_H */
