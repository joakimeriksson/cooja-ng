/*
 * Debugger checks for the ARM interpreter loop, out of line: the GDB stub's
 * breakpoints and halt, and the shell's breakpoints and watchpoints.  The loop
 * calls arm_debug_stop only while one of them is attached (see arm_cpu.c).
 */
#include "arm_cpu.h"
#include "gdb_stub.h"

#include <stdint.h>

bool arm_debug_stop(arm_cpu_t *cpu) {
    gdb_stub_t *g = (gdb_stub_t *)cpu->gdb_stub;
    if (g) {
        if (gdb_stub_check_breakpoint(g, cpu->reg[ARM_PC] & ~1u)) return true;
        if (g->halted) return true;
    }
    return cpu->dbg_count > 0 && arm_dbg_check(cpu);
}

/* Cold and out of line: only ever called while a shell breakpoint or
 * watchpoint is armed, and kept out of the interpreter's hot text so arming
 * support does not move the loop's code layout. */
__attribute__((cold, noinline))
bool arm_dbg_check(arm_cpu_t *cpu) {
    if (cpu->dbg_halted) return true;
    uint32_t pc = cpu->reg[ARM_PC] & ~1u;
    /* Watchpoints: a watched value that differs from its shadow was written
     * by the instruction that ran since the last check (or by a peripheral
     * between slices) — report the pc that instruction started at. */
    for (int i = 0; i < cpu->dbg_wp_n; i++) {
        uint32_t v = 0;
        for (int b = 0; b < cpu->dbg_wp[i].len; b++)
            v |= (uint32_t)arm_read8(cpu, cpu->dbg_wp[i].addr + (uint32_t)b) << (8 * b);
        if (v == cpu->dbg_wp[i].shadow) continue;
        cpu->dbg_hit_kind = 2;
        cpu->dbg_hit_index = i;
        cpu->dbg_hit_pc = cpu->dbg_prev_pc;
        cpu->dbg_hit_old = cpu->dbg_wp[i].shadow;
        cpu->dbg_hit_value = v;
        cpu->dbg_wp[i].shadow = v;
        cpu->dbg_halted = cpu->dbg_hit_new = true;
        cpu->dbg_prev_pc = pc;
        return true;
    }
    bool skip = pc == cpu->dbg_skip_pc;
    cpu->dbg_skip_pc = UINT32_MAX;
    for (int i = 0; !skip && i < cpu->dbg_bp_n; i++) {
        if (cpu->dbg_bp[i] != pc) continue;
        cpu->dbg_hit_kind = 1;
        cpu->dbg_hit_index = i;
        cpu->dbg_hit_pc = pc;
        cpu->dbg_halted = cpu->dbg_hit_new = true;
        return true;
    }
    cpu->dbg_prev_pc = pc;
    return false;
}

