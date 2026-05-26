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
 * Scheduling wrappers — milestone 3.  Thin pass-throughs to sim_eq_*.
 * ============================================================ */

void sim_schedule_mote_wakeup(sim_runtime_t *sim, int mote_index,
                              int64_t time_ns) {
    sim_eq_schedule(&sim->event_queue, mote_index, time_ns);
}

void sim_schedule_mote_wakeup_if_earlier(sim_runtime_t *sim, int mote_index,
                                          int64_t time_ns) {
    sim_eq_schedule_if_earlier(&sim->event_queue, mote_index, time_ns);
}

void sim_schedule_radio_byte(sim_runtime_t *sim, int receiver_mote,
                             int sender_mote, uint8_t byte, int8_t rssi,
                             int64_t time_ns) {
    sim_eq_schedule_rx_byte(&sim->event_queue, receiver_mote, sender_mote,
                            byte, rssi, time_ns);
}

void sim_cancel_mote_events(sim_runtime_t *sim, int mote_index) {
    sim_eq_remove_node(&sim->event_queue, mote_index);
}
