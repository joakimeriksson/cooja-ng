/*
 * sim_mote_t — the kernel-facing mote object (Phase 2 of the refactor,
 * docs/design/refactor-plan.md §3.16 / §4.3).
 *
 * A mote is an opaque `impl` pointer plus a vtable.  The runner's four
 * node kinds (MSP430 emulated, ARM emulated, native Cooja, JS app)
 * each provide one `sim_mote_ops_t` table; everything that previously
 * switched on `node_type_t` dispatches through the table instead.
 *
 * Phase 2 scope: the adapter implementations stay in the runner
 * (test/test_mixed_multinode.c) because they reference runner-side
 * policy state (ticking guards, GDB stubs, RX queue drains).  They
 * move to src/motes/ in Phase 4 together with boot policy.
 *
 * Ops are added milestone by milestone (M11..M17); members below are
 * grouped by the milestone that introduced them.  All ops are
 * mandatory for every mote kind unless the comment says optional —
 * keeps call sites free of NULL checks for the common paths.
 */
#ifndef SIM_MOTE_H
#define SIM_MOTE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sim_mote sim_mote_t;

typedef struct sim_mote_ops {
    /* Short human-readable kind tag ("MSP430", "ARM", "NATIVE", "JS").
     * Used for logs/UI only — never compare against it to branch
     * behavior; add an op instead. */
    const char *kind;

    /* ---- M11: read-only accessors -------------------------------- */

    /* Mote-local view of simulation time (ns).  For emulated motes this
     * is cpu->sim_time_ns (cycle-derived, pinned to the scheduler time
     * across execute slices); for native/JS it is the node's own
     * sim_time_ns field. */
    int64_t (*sim_time_ns)(const sim_mote_t *m);

    /* CPU cycle counter.  Native/JS motes report pseudo-cycles
     * (1 cycle = 1 µs) so cycle-budget arithmetic keeps working. */
    int64_t (*cycles)(const sim_mote_t *m);

    /* Current CPU frequency in Hz (native/JS: 1 MHz pseudo-frequency,
     * matching the pseudo-cycle unit). */
    uint32_t (*freq_hz)(const sim_mote_t *m);

    /* Retired instruction count, 0 if not tracked (native/JS). */
    int64_t (*instructions)(const sim_mote_t *m);
} sim_mote_ops_t;

struct sim_mote {
    int id;                      /* node id (firmware-visible), not slot index */
    const sim_mote_ops_t *ops;
    void *impl;                  /* runner-owned node struct (mixed_node_t) */
};

#ifdef __cplusplus
}
#endif

#endif /* SIM_MOTE_H */
