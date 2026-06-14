/*
 * sim_runtime_t — the deterministic simulation kernel container.
 *
 * Phase 1 milestone 1 of the refactor in docs/design/refactor-plan.md.
 * This is the first slice: the struct bundles five things that the runner
 * currently keeps as file-scope globals.  No scheduling semantics or call
 * sites change in this milestone — the runner still drives the loop, and
 * existing globals (current_sim_ns, sim_eq, radio_medium) are aliased to
 * fields here so old code keeps compiling.
 *
 * Subsequent milestones (§3.15 in the plan) move call sites onto the
 * accessors / wrappers defined here.
 */
#ifndef SIM_RUNTIME_H
#define SIM_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "radio_medium.h"
#include "sim_event_queue.h"
#include "sim_mote.h"
#include "sim_observer.h"
#include "sim_service.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Run/pause state.  Wired into the loop later — milestone 1 only defines it. */
typedef enum sim_run_state {
    SIM_RUN_STOPPED = 0,
    SIM_RUN_RUNNING,
    SIM_RUN_PAUSED,
    SIM_RUN_STOP_REQUESTED,
} sim_run_state_t;

struct sim_radio_bus;  /* see sim_radio_bus.h — owned storage lives with the runner until M10 */

typedef struct sim_runtime {
    /* The five Phase 1 milestone 1 fields, in the order the plan lists them. */
    int64_t            now_ns;        /* current simulation time (ns)        */
    int64_t            end_ns;        /* run-until horizon (ns), 0 = unset   */
    sim_run_state_t    run_state;     /* lifecycle state, see enum above     */
    sim_event_queue_t  event_queue;   /* unified event queue                 */
    radio_medium_t     radio_medium;  /* per-node radio routing/policy       */
    struct sim_radio_bus *radio_bus;  /* RF routing state (M9.2)             */

    /* Per-mote slot generation counters — milestone 4.  Bumped on mote
     * remove/reboot via sim_runtime_bump_mote_generation().  The sim_schedule_*
     * wrappers stamp the current generation onto each event; dispatchers call
     * sim_runtime_event_is_current() to drop stale events.  Initial value is
     * 1, so legacy events with target_generation==0 (untracked) never match
     * the slot and are never dropped by the generation check — they are
     * dispatched as before. */
    uint32_t           mote_generation[SIM_EQ_MAX_NODES];

    /* Mote slot table — Phase 2 milestone 11.  Registered from the
     * runner's init_node() (covers reboots and dynamically added motes,
     * same pattern as radio-bus registration).  The kernel does not call
     * mote ops yet — dispatch stays runner-side until Phase 10; this
     * table makes motes *visible* to kernel-side code and gives the
     * runner one canonical handle per slot.  NULL = empty slot. */
    sim_mote_t        *motes[SIM_EQ_MAX_NODES];

    /* Observer subscribers — milestone 6.  Up to SIM_RUNTIME_MAX_OBSERVERS
     * services subscribe via sim_runtime_subscribe().  Fan-out happens in
     * sim_runtime_emit() in insertion order.  Empty array today; milestone
     * 7 wires timeline as the first subscriber. */
#define SIM_RUNTIME_MAX_OBSERVERS 16
    struct {
        sim_observer_callback_t cb;
        void                   *user;
    }                  observers[SIM_RUNTIME_MAX_OBSERVERS];
    int                observer_count;

    /* Service host — Phase 6 milestone 31 (§3.19).  An ordered table of
     * extracted services (timeline, PCAP, progress, …).  The host
     * subscribes ONE fan-out observer (sim_service_dispatch_event) that
     * walks these services' on_event, so a service does not consume an
     * observer slot of its own.  service_observer_handle is the slot of
     * that fan-out observer, or -1 when not yet subscribed.  See
     * sim_service.h / sim_service.c. */
    sim_service_slot_t services[SIM_RUNTIME_MAX_SERVICES];
    int                service_count;
    int                service_observer_handle;
} sim_runtime_t;

/* Zero-initialize the runtime; does NOT init the event queue or radio medium
 * — those have their own init functions called by the existing runner. */
void sim_runtime_init(sim_runtime_t *sim);

/* Release runtime-owned resources.  Safe to call on a zero-initialized
 * runtime. */
void sim_runtime_destroy(sim_runtime_t *sim);

/* Accessor for the current simulation time.  Milestone 2 makes this the
 * canonical reader. */
static inline int64_t sim_runtime_now_ns(const sim_runtime_t *sim) {
    return sim->now_ns;
}

/* ============================================================
 * Scheduling wrappers — Phase 1 milestone 3.
 *
 * Thin wrappers around the existing sim_eq_* API.  Same semantics, just
 * keyed on `sim_runtime_t *` so call sites speak the kernel API instead
 * of touching the event queue directly.  Inline so there's no extra call
 * overhead vs. the previous direct sim_eq_* calls.
 *
 * Coalescing rules (§3.8 of the refactor plan):
 *   - mote_wakeup:           one pending event per mote; reschedule replaces
 *   - mote_wakeup_if_earlier: only schedules if strictly earlier than the
 *                            existing pending wakeup for that mote
 *   - radio_byte:            never coalesced; multiple in flight allowed
 * ============================================================ */

void sim_schedule_mote_wakeup(sim_runtime_t *sim, int mote_index,
                              int64_t time_ns);

void sim_schedule_mote_wakeup_if_earlier(sim_runtime_t *sim, int mote_index,
                                          int64_t time_ns);

void sim_schedule_radio_byte(sim_runtime_t *sim, int receiver_mote,
                             int sender_mote, uint8_t byte, int8_t rssi,
                             int64_t time_ns);

/* Radio-bus RF timer for receiver_mote (M9.5).  Not coalesced at the
 * queue level; the radio bus keeps one-pending-per-node bookkeeping
 * and re-arms lazily at fire time. */
void sim_schedule_radio_timer(sim_runtime_t *sim, int receiver_mote,
                              int64_t time_ns);

/* Drop every pending event targeting `mote_index` (wakeups + RX bytes).
 * Physical purge of the queue, used on mote remove / reboot together with
 * sim_runtime_bump_mote_generation().  Generation-aware dispatch
 * (sim_runtime_event_is_current) handles any late-scheduled stragglers. */
void sim_cancel_mote_events(sim_runtime_t *sim, int mote_index);

/* ============================================================
 * Mote slot generation — milestone 4.
 *
 * Each mote slot has a monotonic generation counter that bumps every time
 * the slot is removed/rebooted.  Events scheduled via the sim_schedule_*
 * wrappers carry the generation observed at schedule time
 * (target_generation in sim_event_t).  Dispatchers must call
 * sim_runtime_event_is_current() before executing a popped event and skip
 * any that no longer match.
 *
 * Generation 0 is reserved for "untracked" events scheduled directly via
 * sim_eq_* without going through the wrappers.  These are never dropped
 * by the check (the runner's pre-milestone-5 call sites still take this
 * path, so this is the compatibility escape).
 * ============================================================ */

/* Current generation for mote_index, or 0 if out of range. */
uint32_t sim_runtime_mote_generation(const sim_runtime_t *sim, int mote_index);

/* Bump the generation for mote_index.  Call this from node remove/reboot
 * paths so any stragglers (or future late-arriving events scheduled
 * pre-bump) get filtered at dispatch. */
void sim_runtime_bump_mote_generation(sim_runtime_t *sim, int mote_index);

/* True if `ev` should still be dispatched against its target mote slot.
 * Returns true for untracked (target_generation == 0) and for events
 * whose stamped generation still matches the slot. */
bool sim_runtime_event_is_current(const sim_runtime_t *sim,
                                   const sim_event_t *ev);

/* ============================================================
 * Mote slot table — Phase 2 milestone 11 (§3.16).
 *
 * Register/lookup of the kernel-facing mote object per slot.  Storage
 * for the sim_mote_t itself is owned by the registrant (the runner);
 * the runtime only keeps the pointer.  Re-registering a slot (reboot)
 * just overwrites the pointer.
 * ============================================================ */

/* Register `mote` at slot `mote_index`.  Pass NULL to clear the slot. */
void sim_runtime_register_mote(sim_runtime_t *sim, int mote_index,
                               sim_mote_t *mote);

/* The mote at slot `mote_index`, or NULL if empty/out of range. */
static inline sim_mote_t *sim_runtime_mote(sim_runtime_t *sim,
                                           int mote_index) {
    if (mote_index < 0 || mote_index >= SIM_EQ_MAX_NODES) return (sim_mote_t *)0;
    return sim->motes[mote_index];
}

/* ============================================================
 * Observer subscriptions — milestone 6.
 *
 * Services subscribe a `(callback, user)` pair.  On every kernel emission
 * the callback fires with a `const sim_observer_event_t *` whose payload
 * is stack-allocated by the emitter; callbacks must not retain it past
 * the call.  Callbacks must not re-enter kernel dispatch APIs (use
 * sim_runtime_request_stop() for graceful exit).
 *
 * Subscription order is fan-out order.  No priority, no filtering — a
 * service that only cares about LED events just ignores other kinds.
 * ============================================================ */

/* Subscribe.  Returns the subscriber index ≥ 0, or -1 if the table is
 * full.  Pass the returned index back to sim_runtime_unsubscribe(). */
int  sim_runtime_subscribe(sim_runtime_t *sim,
                            sim_observer_callback_t cb, void *user);

/* Unsubscribe.  Safe to call from inside a callback; the callback's own
 * slot becomes empty after the current emission loop completes. */
void sim_runtime_unsubscribe(sim_runtime_t *sim, int handle);

/* Emit one observer event.  Fan-out to every subscribed callback in
 * insertion order.  Safe to call with `observer_count == 0` — no-op.
 * The emitter retains ownership of `ev` and any pointed-to data; the
 * call returns before the event goes out of scope. */
void sim_runtime_emit(sim_runtime_t *sim, const sim_observer_event_t *ev);

/* ============================================================
 * Kernel event pump — milestone 10.
 *
 * sim_runtime_run_until() is the deterministic core loop (§3.7 of the
 * refactor plan): it pops events with time_ns <= end_ns in (time, seq)
 * order, advances now_ns to each event's exact time *before* popping
 * (callbacks observe the event time, matching Cooja's
 * getSimulationTime()), drops stale-generation events, and hands each
 * surviving event to `dispatch`.
 *
 * The runner still owns the outer horizon policy (UI pacing, service
 * polls, timed test actions) and calls the pump once per outer slice.
 * Event dispatch bodies stay runner-side until the Phase 2 mote vtable
 * exists; `dispatch` is that seam.
 *
 * On return, now_ns is left at the last dispatched event's time (or
 * unchanged if no event was due) — the caller advances it to its outer
 * horizon as before.  Honors sim_runtime_request_stop(): the pump
 * returns early and run_state stays SIM_RUN_STOP_REQUESTED so the
 * caller can break its outer loop.
 * ============================================================ */

typedef void (*sim_event_dispatch_fn)(void *user, const sim_event_t *ev);

void sim_runtime_run_until(sim_runtime_t *sim, int64_t end_ns,
                           sim_event_dispatch_fn dispatch, void *user);

/* Request a graceful stop.  Safe from observer callbacks (the documented
 * alternative to re-entering kernel dispatch APIs): the pump finishes
 * the current event and returns. */
void sim_runtime_request_stop(sim_runtime_t *sim);

static inline bool sim_runtime_stop_requested(const sim_runtime_t *sim) {
    return sim->run_state == SIM_RUN_STOP_REQUESTED;
}

#ifdef __cplusplus
}
#endif

#endif /* SIM_RUNTIME_H */
