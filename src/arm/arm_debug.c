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

/* Release a node halted at a breakpoint or watchpoint at simulation time
 * now_ns.  The breakpoint is not hit again on the way out (the skip — only
 * while the pc still sits at the hit; a `reg pc =` moved on); the halt
 * counts as time the core was not running (LPM in the energy view) and the
 * node's time moves to now, so the missed time is neither replayed as one
 * burst nor charged as active — the node resumes with its clock that much
 * behind, while its peripherals' time jumps forward at the release, as on
 * a real debug halt where the counters keep running. */
void arm_dbg_release(arm_cpu_t *cpu, int64_t now_ns) {
    if (cpu->dbg_hit_kind == 1 && (cpu->reg[ARM_PC] & ~1u) == cpu->dbg_hit_pc)
        cpu->dbg_skip_pc = cpu->dbg_hit_pc;
    cpu->dbg_skip_started = false;
    cpu->dbg_halted = false;
    if (now_ns > cpu->sim_time_ns) {
        cpu->lpm_ns += now_ns - cpu->sim_time_ns;
        cpu->sim_time_ns = now_ns;
    }
    /* Anchor as the tick does: an event drain on a sync path before the
     * first tick derives sim_time_ns from the anchor, and a stale one would
     * set it back to the halt instant while peripheral callbacks run. */
    cpu->anchor_sim_time_ns = now_ns;
    cpu->anchor_cycles = cpu->cycles;
    cpu->last_execute_us = now_ns / 1000LL;
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
    /* The release's skip stays armed until the instruction at that pc has
     * retired — the interpreter clears it after executing it — so an
     * exception taken in between (a due event, a pending IRQ, a WFI
     * fast-forward) that returns to the same pc with the instruction still
     * to run does not re-hit, and the next visit after it ran does. */
    bool skip = pc == cpu->dbg_skip_pc;
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

