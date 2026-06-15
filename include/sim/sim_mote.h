/*
 * sim_mote_t — the kernel-facing mote object (Phase 2 of the refactor,
 * docs/design/refactor-plan.md §3.16 / §4.3).
 *
 * A mote is an opaque `impl` pointer plus a vtable.  The runner's four
 * node kinds (MSP430 emulated, ARM emulated, native Cooja, JS app)
 * each provide one `sim_mote_ops_t` table; everything that previously
 * switched on `node_type_t` dispatches through the table instead.
 *
 * Phase 4 (§3.17): the native/JS adapter tables and all boot policy
 * live in src/motes/; the MSP430/ARM execute/serial adapters stay in
 * the runner (test/test_mixed_multinode.c) because they reference
 * runner-side policy state (ticking guard, GDB stubs, RX queue
 * drains) — they follow those dependencies in Phase 5/6.
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

/* End-of-run diagnostic sections for sim_mote_ops_t.dump_diagnostics
 * (Phase 10 M58).  Each names one per-node terminal-report block; the order
 * the runner emits them in (interleaved with aggregates) is preserved. */
typedef enum sim_mote_diag {
    SIM_MOTE_DIAG_SFD_TIMERB = 1,
    SIM_MOTE_DIAG_TSCH_STATE,
    SIM_MOTE_DIAG_CC2420_STATS,
    SIM_MOTE_DIAG_NEIGHBOR_TABLE,
    SIM_MOTE_DIAG_SR_TABLE,
} sim_mote_diag_t;

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

    /* ---- M17: frame-level RX --------------------------------------- */

    /* OPTIONAL (frame-consuming motes only — native Cooja + JS; NULL
     * for emulated motes, which take per-byte / staged delivery):
     * queue one received frame arriving at now_ns.  sender_idx feeds
     * the native rx-queue's arrival bookkeeping; JS motes ignore it
     * and self-schedule an immediate wakeup.  Returns 0 when queued,
     * <0 when the mote's RX queue was already full (the frame is
     * handed to the mote either way — matching the historical native
     * overwrite semantics); the CALLER owns the queued/queue-full
     * stats (Phase 4 — adapters must not touch runner stats). */
    int (*receive_frame)(sim_mote_t *m, const uint8_t *frame, int len,
                         int64_t now_ns, int sender_idx);

    /* ---- M25 (Phase 5 de-typing prep): RF-delivery clock motion ----
     *
     * These let the radio bus drive an emulated receiver's local clock
     * during synchronous frame delivery without knowing the node type
     * (the byte-delivery loop becomes type-blind ahead of moving into
     * the bus in M26/M27).  Both are OPTIONAL and present ONLY on the
     * emulated kinds — native/JS leave them NULL, which is exactly the
     * "is this an emulated receiver?" test the old nodes[].type checks
     * performed. */

    /* Sync the mote's CPU to one incoming byte's air time, just before
     * the byte is handed to the chip (Cooja's execute(t, 0) per byte).
     * MSP430 delegates to its clamped, deviation-aware sync_to_time;
     * ARM uses the unclamped pin-step-pin body — the two differ on
     * purpose, do not unify. */
    void (*rx_byte_sync)(sim_mote_t *m, int64_t byte_time_ns);

    /* MSP430 only: a full execute-slice pre-sync run before a
     * synchronous frame delivery (ex the frame_complete pre-sync tick).
     * NULL elsewhere. */
    void (*rx_pre_sync)(sim_mote_t *m, int64_t time_ns);

    /* ---- M58 (Phase 10): end-of-run per-node diagnostics ------------ */

    /* OPTIONAL: emit one end-of-run diagnostic section for this mote.  Only
     * the MSP430 kind implements it (the per-node SFD/Timer-B/TSCH-state/
     * neighbor+SR-table/CC2420-stats chip-memory + firmware-symbol dumps);
     * NULL elsewhere.  The runner drives the section order (interleaved with
     * cross-cutting aggregates that carry no chip coupling) and treats a NULL
     * op as "this kind has no per-node diagnostics" — which doubles as the
     * type-blind "is this an MSP430 mote?" test the old NODE_MSP430 stat
     * loops performed.  `section` is a sim_mote_diag_t. */
    void (*dump_diagnostics)(const sim_mote_t *m, int section);

    /* ---- M54 (Phase 10): per-kind startup-delay shift ---------------
     *
     * OPTIONAL: apply the randomized startup-delay spread to the mote's
     * platform state and report whether the mote's LOCAL sim-time was
     * advanced by `delay_ns`.  ARM shifts cpu->sim_time_ns + cycles and
     * returns true (so the runner's node_start_ns uses the post-shift
     * sim_time directly); native shifts its pending etimer/rtimer
     * expirations and returns false (sim_time unchanged → runner adds
     * delay_ns).  MSP430/JS leave this NULL (no internal shift; runner
     * treats NULL as "not advanced" and adds delay_ns). */
    bool (*apply_startup_delay)(sim_mote_t *m, int64_t delay_ns);
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
