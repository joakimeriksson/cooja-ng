/*
 * mock_sim_host_t — record-and-inspect harness for unit-testing chip drivers
 * without a real CPU or GPIO peripheral.
 *
 * Usage pattern:
 *
 *   mock_sim_host_t mock;
 *   mock_sim_host_init(&mock);
 *
 *   cc2420_t radio;
 *   cc2420_init(&radio, &mock.host);   // chip driver only sees a sim_host_t
 *
 *   // drive the chip:
 *   cc2420_set_vreg(&radio, true);
 *
 *   // inspect what the chip asked the host to do:
 *   assert(mock.set_input_pin_calls > 0);
 *   assert(mock.last_set_pin.port == 1);
 *
 *   // advance virtual time and fire any due events:
 *   mock_sim_host_advance_ns(&mock, 1000000);  // 1 ms
 *
 * The mock keeps the most recent call of each kind plus a counter, which
 * is enough for state-machine assertions without growing the test fixture.
 * Tests that need the full call sequence can swap individual fields for
 * arrays.
 */
#ifndef MOCK_SIM_HOST_H
#define MOCK_SIM_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "sim_host.h"

#define MOCK_SIM_HOST_MAX_EVENTS 32

typedef struct {
    int port, pin;
    bool value;
} mock_pin_event_t;

typedef struct {
    int port, pin;
    bool rising;
} mock_irq_edge_event_t;

typedef struct mock_sim_host {
    sim_host_t  host;            /* embedded vtable — pass &mock.host to chip drivers */

    /* Virtual clock */
    int64_t     now_ns;

    /* Pending events, sorted by fire_ns ascending */
    cpu_event_t *queue[MOCK_SIM_HOST_MAX_EVENTS];
    int         queue_len;

    /* Call counters and most-recent-call records */
    int               set_input_pin_calls;
    mock_pin_event_t  last_set_pin;
    int               force_irq_edge_calls;
    mock_irq_edge_event_t last_force_irq;
    int               schedule_calls;
    int               cancel_calls;
} mock_sim_host_t;

/* Initialize a mock host (zeroes counters, fills the vtable). */
void mock_sim_host_init(mock_sim_host_t *mock);

/* Advance virtual time by `delta_ns` and fire any events whose fire_ns
 * has passed. Events fire in time order; their callbacks may schedule
 * new events, which will fire if their time falls within the window. */
void mock_sim_host_advance_ns(mock_sim_host_t *mock, int64_t delta_ns);

/* Set virtual time directly (for fixture setup). Does NOT fire events. */
static inline void mock_sim_host_set_now(mock_sim_host_t *mock, int64_t now_ns) {
    mock->now_ns = now_ns;
}

/* Reset call counters without touching the event queue or virtual time. */
static inline void mock_sim_host_reset_counters(mock_sim_host_t *mock) {
    mock->set_input_pin_calls = 0;
    mock->force_irq_edge_calls = 0;
    mock->schedule_calls = 0;
    mock->cancel_calls = 0;
}

#endif /* MOCK_SIM_HOST_H */
