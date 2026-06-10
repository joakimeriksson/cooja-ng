/*
 * Implementation of sim_runtime_t — see include/sim/sim_runtime.h.
 *
 * Phase 1 milestone 1: behavior-preserving introduction of the runtime
 * container.  The event queue and radio medium are zero-initialized here;
 * the runner still calls their own init functions for the parts that need
 * configuration (node count, propagation parameters, etc.).
 */
#include "sim_runtime.h"

#include "sim_event_queue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sim_runtime_init(sim_runtime_t *sim) {
    if (!sim) return;
    memset(sim, 0, sizeof(*sim));
    sim->run_state = SIM_RUN_STOPPED;
    /* sim->event_queue and sim->radio_medium left zeroed; the runner
     * still calls sim_eq_init() and radio_medium_init() on them via the
     * aliases declared in test/test_mixed_multinode.c. */

    /* Milestone 4: per-mote generations start at 1.  Generation 0 is
     * reserved for "untracked" events (sim_eq_* direct callers); valid
     * mote generations start at 1 so the initial state never matches
     * legacy events but always matches wrapper-scheduled events. */
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        sim->mote_generation[i] = 1u;
}

void sim_runtime_destroy(sim_runtime_t *sim) {
    if (!sim) return;
    /* sim_event_queue_t and radio_medium_t do not own dynamically
     * allocated memory in the current implementation, so destroy is a
     * no-op beyond resetting state. */
    sim->run_state = SIM_RUN_STOPPED;
    sim->now_ns = 0;
    sim->end_ns = 0;
}

/* ============================================================
 * Scheduling wrappers — milestones 3 + 4.  Stamp the current mote slot
 * generation on each event so dispatch can drop stragglers after
 * slot remove/reboot.
 * ============================================================ */

void sim_schedule_mote_wakeup(sim_runtime_t *sim, int mote_index,
                              int64_t time_ns) {
    uint32_t gen = sim_runtime_mote_generation(sim, mote_index);
    sim_eq_schedule_gen(&sim->event_queue, mote_index, time_ns, gen);
}

void sim_schedule_mote_wakeup_if_earlier(sim_runtime_t *sim, int mote_index,
                                          int64_t time_ns) {
    uint32_t gen = sim_runtime_mote_generation(sim, mote_index);
    sim_eq_schedule_if_earlier_gen(&sim->event_queue, mote_index, time_ns, gen);
}

void sim_schedule_radio_byte(sim_runtime_t *sim, int receiver_mote,
                             int sender_mote, uint8_t byte, int8_t rssi,
                             int64_t time_ns) {
    uint32_t gen = sim_runtime_mote_generation(sim, receiver_mote);
    sim_eq_schedule_rx_byte_gen(&sim->event_queue, receiver_mote, sender_mote,
                                 byte, rssi, time_ns, gen);
}

void sim_schedule_radio_timer(sim_runtime_t *sim, int receiver_mote,
                              int64_t time_ns) {
    uint32_t gen = sim_runtime_mote_generation(sim, receiver_mote);
    sim_eq_schedule_radio_timer_gen(&sim->event_queue, receiver_mote,
                                    time_ns, gen);
}

void sim_cancel_mote_events(sim_runtime_t *sim, int mote_index) {
    sim_eq_remove_node(&sim->event_queue, mote_index);
}

/* ============================================================
 * Mote slot generation — milestone 4.
 * ============================================================ */

uint32_t sim_runtime_mote_generation(const sim_runtime_t *sim, int mote_index) {
    if (!sim || mote_index < 0 || mote_index >= SIM_EQ_MAX_NODES) return 0u;
    return sim->mote_generation[mote_index];
}

void sim_runtime_bump_mote_generation(sim_runtime_t *sim, int mote_index) {
    if (!sim || mote_index < 0 || mote_index >= SIM_EQ_MAX_NODES) return;
    uint32_t next = sim->mote_generation[mote_index] + 1u;
    /* Never produce generation 0 — that value is reserved for "untracked"
     * (legacy) events.  Wrap from UINT32_MAX directly to 1. */
    if (next == 0u) next = 1u;
    sim->mote_generation[mote_index] = next;
}

bool sim_runtime_event_is_current(const sim_runtime_t *sim,
                                   const sim_event_t *ev) {
    if (!sim || !ev) return true;
    if (ev->target_generation == 0u) return true;     /* untracked → never drop */
    if (ev->node_idx < 0 || ev->node_idx >= SIM_EQ_MAX_NODES) return true;
    return ev->target_generation == sim->mote_generation[ev->node_idx];
}

/* ============================================================
 * Observer subscriptions — milestone 6.  Fan-out is a simple linear
 * walk; subscriber count is bounded by SIM_RUNTIME_MAX_OBSERVERS.
 *
 * Unsubscribe writes NULL to the slot; emit skips NULL slots.  Slots
 * are never compacted, so handles stay stable across the lifetime of
 * the runtime — important if a service stashes its handle for later.
 * ============================================================ */

int sim_runtime_subscribe(sim_runtime_t *sim,
                          sim_observer_callback_t cb, void *user) {
    if (!sim || !cb) return -1;
    /* Reuse first empty slot ≤ observer_count, else append. */
    for (int i = 0; i < sim->observer_count; i++) {
        if (sim->observers[i].cb == NULL) {
            sim->observers[i].cb = cb;
            sim->observers[i].user = user;
            return i;
        }
    }
    if (sim->observer_count >= SIM_RUNTIME_MAX_OBSERVERS) return -1;
    int h = sim->observer_count++;
    sim->observers[h].cb = cb;
    sim->observers[h].user = user;
    return h;
}

void sim_runtime_unsubscribe(sim_runtime_t *sim, int handle) {
    if (!sim || handle < 0 || handle >= sim->observer_count) return;
    sim->observers[handle].cb = NULL;
    sim->observers[handle].user = NULL;
}

void sim_runtime_emit(sim_runtime_t *sim, const sim_observer_event_t *ev) {
    if (!sim || !ev) return;
    /* Snapshot count so a callback subscribing mid-emission doesn't get
     * called for the event that triggered its subscription. */
    int count = sim->observer_count;
    for (int i = 0; i < count; i++) {
        sim_observer_callback_t cb = sim->observers[i].cb;
        if (cb) cb(sim->observers[i].user, ev);
    }
}

/* ============================================================
 * Kernel event pump — milestone 10.
 * ============================================================ */

void sim_runtime_request_stop(sim_runtime_t *sim) {
    if (!sim) return;
    sim->run_state = SIM_RUN_STOP_REQUESTED;
}

/* Same-time event-spin diagnostics (CSIM_TRACE_EVENT_SPIN=1): warn when
 * the pump dispatches a very large number of events at one sim instant —
 * the signature of a scheduling loop that never advances time. */
static int trace_event_spin = -1;

static bool trace_event_spin_enabled(void) {
    if (trace_event_spin < 0) {
        const char *env = getenv("CSIM_TRACE_EVENT_SPIN");
        trace_event_spin = (env && env[0] && strcmp(env, "0") != 0) ? 1 : 0;
    }
    return trace_event_spin != 0;
}

void sim_runtime_run_until(sim_runtime_t *sim, int64_t end_ns,
                           sim_event_dispatch_fn dispatch, void *user) {
    static int64_t spin_time_ns = INT64_MIN;
    static uint64_t spin_count = 0;

    if (!sim || !dispatch) return;
    if (sim->run_state != SIM_RUN_STOP_REQUESTED)
        sim->run_state = SIM_RUN_RUNNING;

    for (;;) {
        if (sim->run_state == SIM_RUN_STOP_REQUESTED)
            return;
        int64_t next_ev_time = sim_eq_peek_time(&sim->event_queue);
        if (next_ev_time > end_ns)
            return;  /* queue drained past the horizon (or empty) */

        /* Match Cooja's simulation.getSimulationTime(): callbacks fired
         * from the current event observe the exact event time, not the
         * coarser outer stepping horizon.  Set before the generation
         * check so even dropped events advance the clock. */
        sim->now_ns = next_ev_time;

        if (trace_event_spin_enabled()) {
            if (next_ev_time == spin_time_ns) {
                spin_count++;
            } else {
                spin_time_ns = next_ev_time;
                spin_count = 1;
            }
            if (spin_count == 1000000ULL) {
                sim_event_t peek = sim_eq_peek(&sim->event_queue);
                fprintf(stderr,
                        "  [SPIN] t=%.6f kind=%d node=%d sender=%d seq=%llu\n",
                        (double)next_ev_time / 1e9,
                        (int)peek.kind, peek.node_idx, peek.sender_idx,
                        (unsigned long long)peek.seq);
            }
        }

        sim_event_t ev = sim_eq_pop(&sim->event_queue);

        /* Generation check (milestones 4/5): drop events whose target
         * slot has been removed/rebooted since the event was queued.
         * Untracked events (target_generation == 0) always pass. */
        if (!sim_runtime_event_is_current(sim, &ev))
            continue;

        dispatch(user, &ev);
    }
}
