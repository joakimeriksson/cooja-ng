/*
 * GDB stub glue for ARM Cortex-M3.
 *
 * The GDB protocol expects ARM registers in this order (17 × 32-bit
 * little-endian words, total 68 bytes for the `g` packet):
 *
 *   0..12 : r0 .. r12
 *   13    : sp
 *   14    : lr
 *   15    : pc
 *   16    : xpsr (CPSR equivalent for Cortex-M)
 *
 * Floating-point registers (s0..s31, fpscr) are not implemented for
 * the CC2538 since the M3 has no FPU.
 */
#include "arm_gdb.h"
#include "arm_cpu.h"

#include <string.h>

#define ARM_GDB_REG_COUNT  17
#define ARM_GDB_REGS_BYTES (ARM_GDB_REG_COUNT * 4)

static void put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static uint32_t get_le32(const uint8_t *p) {
    return  (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static int arm_gdb_read_regs(void *cpu_v, uint8_t *buf, int buf_size) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    if (buf_size < ARM_GDB_REGS_BYTES) return 0;

    /* r0..r12 */
    for (int i = 0; i < 13; i++)
        put_le32(buf + i * 4, cpu->reg[i]);
    /* sp, lr, pc, xpsr */
    put_le32(buf + 13 * 4, cpu->reg[ARM_SP]);
    put_le32(buf + 14 * 4, cpu->reg[ARM_LR]);
    put_le32(buf + 15 * 4, cpu->reg[ARM_PC]);
    put_le32(buf + 16 * 4, cpu->xpsr);
    return ARM_GDB_REGS_BYTES;
}

static void arm_gdb_write_regs(void *cpu_v, const uint8_t *buf, int len) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    if (len < ARM_GDB_REGS_BYTES) return;
    for (int i = 0; i < 13; i++)
        cpu->reg[i] = get_le32(buf + i * 4);
    cpu->reg[ARM_SP] = get_le32(buf + 13 * 4);
    cpu->reg[ARM_LR] = get_le32(buf + 14 * 4);
    cpu->reg[ARM_PC] = get_le32(buf + 15 * 4);
    cpu->xpsr        = get_le32(buf + 16 * 4);
}

static int arm_gdb_read_mem(void *cpu_v, uint32_t addr, uint8_t *buf, int len) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    /* Use byte-wise reads — slower than block, but simple and works
     * across all memory regions including IO. */
    for (int i = 0; i < len; i++)
        buf[i] = arm_read8(cpu, addr + (uint32_t)i);
    return len;
}

static int arm_gdb_write_mem(void *cpu_v, uint32_t addr, const uint8_t *buf, int len) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    for (int i = 0; i < len; i++)
        arm_write8(cpu, addr + (uint32_t)i, buf[i]);
    return len;
}

static uint32_t arm_gdb_get_pc(void *cpu_v) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    return cpu->reg[ARM_PC];
}

static void arm_gdb_set_pc(void *cpu_v, uint32_t pc) {
    arm_cpu_t *cpu = (arm_cpu_t *)cpu_v;
    /* Strip Thumb bit so the PC is correctly aligned. */
    cpu->reg[ARM_PC] = pc & ~1u;
}

const gdb_arch_ops_t arm_gdb_ops = {
    .read_regs  = arm_gdb_read_regs,
    .write_regs = arm_gdb_write_regs,
    .read_mem   = arm_gdb_read_mem,
    .write_mem  = arm_gdb_write_mem,
    .get_pc     = arm_gdb_get_pc,
    .set_pc     = arm_gdb_set_pc,
};
