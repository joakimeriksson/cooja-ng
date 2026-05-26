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
