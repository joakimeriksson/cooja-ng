/*
 * ARM Thumb/Thumb-2 instruction decoder types
 */
#ifndef ARM_DECODE_H
#define ARM_DECODE_H

#include <stdint.h>

/* Instruction categories for dispatch */
typedef enum {
    ARM_INSN_16BIT,
    ARM_INSN_32BIT,
} arm_insn_width_t;

/* Check if a 16-bit halfword starts a 32-bit Thumb-2 instruction */
static inline int arm_is_thumb2(uint16_t hw) {
    uint16_t top5 = hw >> 11;
    return (top5 == 0x1D || top5 == 0x1E || top5 == 0x1F);
}

#endif /* ARM_DECODE_H */
