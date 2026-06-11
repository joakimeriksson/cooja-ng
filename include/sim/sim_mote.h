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

/* Typed-interface ids for sim_mote_ops_t.get_interface (M16).  An
 * adapter returns the requested chip/CPU pointer or NULL when the mote
 * kind doesn't carry that interface — callers branch on NULL instead of
 * on node type. */
typedef enum sim_mote_iface {
    SIM_MOTE_IFACE_ARM_CPU = 1,  /* arm_cpu_t*  — GDB stub attach    */
    SIM_MOTE_IFACE_CC2420  = 2,  /* cc2420_t*   — CC2420 chip state  */
} sim_mote_iface_t;

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

    /* ---- M12: execution ------------------------------------------ */

    /* Run the mote's execute slice at global time `now_ns` (§3.6).
     * The mote must not advance global time; it returns the absolute
     * time of its next known wakeup (INT64_MAX = none) and the CALLER
     * does the single sim_schedule_mote_wakeup_if_earlier().  For
     * emulated motes this wraps the Cooja MspMote.execute(t, duration)
     * tick (clock deviation, sim_time pinning, step_micros); for
     * native motes the ContikiMote.execute() tick + the
     * doActionsAfterTick wakeup computation; for JS motes the queued
     * callback dispatch up to now_ns. */
    int64_t (*execute)(sim_mote_t *m, int64_t now_ns);

    /* Step the mote to `target` in its own execution unit: CPU cycles
     * for emulated motes, sim-time ns for native/JS motes (the unit
     * convention predates the vtable; callers already know which they
     * are driving). */
    void (*step_until)(sim_mote_t *m, int64_t target);

    /* Advance the mote to global time `sim_ns` from outside an execute
     * slice (threaded stepping path): convert the mote's lag to its
     * execution unit and step.  Includes the native pending-work fast
     * path. */
    void (*advance_to_time)(sim_mote_t *m, int64_t sim_ns);

    /* OPTIONAL (emulated motes only, NULL otherwise): absolute next
     * wakeup suggestion measured from kernel time `base_ns`, derived
     * from the CPU's next pending cpu-event (and, for MSP430, pending
     * unmasked interrupts).  Used after out-of-slice activity (serial
     * injection, queued-frame drain) re-arms the mote's wakeup. */
    int64_t (*sched_hint_ns)(const sim_mote_t *m, int64_t base_ns);

    /* ---- M13: event-time synchronization -------------------------- */

    /* OPTIONAL (emulated motes only, NULL otherwise): Cooja
     * MspMoteTimeEvent.execute(t) equivalent — advance the CPU to event
     * time `sim_ns` with a zero-duration execute(t, 0) (clock deviation
     * applied, sim_time pinned, capped to the kernel's now_ns) before
     * delivering a chip-level byte or timer at that instant.  Returns
     * the step_micros next-event lead in µs (deviation-corrected). */
    int64_t (*sync_to_time)(sim_mote_t *m, int64_t sim_ns);

    /* ---- M14: serial input ---------------------------------------- */

    /* Inject host-side serial bytes into the mote's console UART.
     * Returns the number of bytes consumed (0 = mote not ready, caller
     * retries later — the serial bridge ring keeps unconsumed bytes).
     * Each adapter implements its platform's delivery contract:
     * native = Cooja ContikiRS232 append + immediate wakeup, MSP430 =
     * MSPSim baud-paced injection with CPU mini-steps, ARM = UART RX
     * register feed, JS = swallowed (no console input). */
    int (*serial_input)(sim_mote_t *m, const uint8_t *buf, int len);

    /* ---- M15: lifecycle ------------------------------------------- */

    /* Tear down the mote's platform state (emulator, native image, JS
     * runtime).  The sim_mote_t itself and its slot registration are
     * owned by the runner. */
    void (*destroy)(sim_mote_t *m);

    /* Re-seed the mote's local clock to global time `now_ns` after a
     * reboot/dynamic add, so stepping resumes at the current sim time
     * instead of catching up from t=0.  Emulated motes also re-derive
     * the cycle counter from the new time. */
    void (*reset_time)(sim_mote_t *m, int64_t now_ns);

    /* ---- M16: introspection ---------------------------------------- */

    /* OPTIONAL: UI radio activity poll, returning sim_radio_state_t
     * values (ui/sim_state.h) as int.  Only motes whose radio state is
     * sampled by polling implement this (MSP430/CC2420); platforms that
     * push state through an async callback (CC2538 rfcore) leave it
     * NULL so the poll never double-reports. */
    int (*ui_radio_state)(const sim_mote_t *m);

    /* OPTIONAL: current LED levels into leds[0..2] (platform mapping:
     * Sky P5.4-6, CC2538DK PC0-2).  NULL = mote has no observable LEDs. */
    void (*ui_leds)(const sim_mote_t *m, uint8_t leds[3]);

    /* Typed interface lookup — returns the requested pointer or NULL.
     * The escape hatch for genuinely chip-specific code (GDB attach,
     * CC2420 debug traces) so it can stay type-blind at the call site. */
    void *(*get_interface)(sim_mote_t *m, int iface);
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
