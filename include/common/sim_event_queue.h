/*
 * Simulation event queue — min-heap priority queue matching Cooja's model.
 *
 * Holds two kinds of events in a single (time_ns, seq) heap, mirroring
 * Cooja's single Simulation.eventQueue:
 *
 *   SIM_EV_NODE_WAKEUP — run a mote tick at time_ns. At most one wakeup
 *                        per node is in flight; rescheduling replaces the
 *                        existing entry (matches Cooja scheduleNextWakeup).
 *
 *   SIM_EV_RX_BYTE     — deliver a single RF byte to a receiver mote at
 *                        time_ns. Multiple in flight per node; never
 *                        deduped.
 *
 * Same-time events fire in insertion order (FIFO via seq), matching
 * Cooja's EventQueue. node_heap_idx[] tracks each node's pending wakeup
 * slot so reschedule is O(log n); RX_BYTE entries are not tracked there.
 */
#ifndef SIM_EVENT_QUEUE_H
#define SIM_EVENT_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define SIM_EQ_MAX_EVENTS 65536
#define SIM_EQ_MAX_NODES 128

typedef enum {
    SIM_EV_NODE_WAKEUP = 0,
    SIM_EV_RX_BYTE     = 1,
} sim_event_kind_t;

typedef struct {
    sim_event_kind_t kind;
    int      node_idx;     /* mote to wake / receiver of byte (-1 = unused) */
    int64_t  time_ns;
    uint64_t seq;          /* insertion order (tiebreaker) */
    /* SIM_EV_RX_BYTE payload; zero for SIM_EV_NODE_WAKEUP */
    int      sender_idx;
    uint8_t  byte;
    int8_t   rssi;
} sim_event_t;

typedef struct {
    sim_event_t heap[SIM_EQ_MAX_EVENTS];
    int node_heap_idx[SIM_EQ_MAX_NODES];   /* pending NODE_WAKEUP slot, or -1 */
    int count;
    uint64_t next_seq;
} sim_event_queue_t;

/* Initialize an empty event queue */
void sim_eq_init(sim_event_queue_t *q);

/* Schedule a node wakeup at time_ns. Replaces any existing wakeup for
 * this node (matches Cooja scheduleNextWakeup semantics). */
void sim_eq_schedule(sim_event_queue_t *q, int node_idx, int64_t time_ns);

/* Schedule a node wakeup, but only if the new time is EARLIER than any
 * existing wakeup for this node. */
void sim_eq_schedule_if_earlier(sim_event_queue_t *q, int node_idx, int64_t time_ns);

/* Schedule an RX byte delivery to (node_idx) at time_ns. Multiple
 * pending entries per node are allowed; never deduped. */
void sim_eq_schedule_rx_byte(sim_event_queue_t *q, int node_idx, int sender_idx,
                             uint8_t byte, int8_t rssi, int64_t time_ns);

/* Pop the earliest event. Returns event with node_idx=-1 if empty. */
sim_event_t sim_eq_pop(sim_event_queue_t *q);

/* Peek at the earliest event time without removing. Returns INT64_MAX if empty. */
int64_t sim_eq_peek_time(const sim_event_queue_t *q);

/* Peek at the earliest event without removing. Returns node_idx=-1 if empty. */
sim_event_t sim_eq_peek(const sim_event_queue_t *q);

/* Check if queue is empty */
bool sim_eq_empty(const sim_event_queue_t *q);

/* Remove ALL events targeting a specific node (wakeup + RX_BYTE). */
void sim_eq_remove_node(sim_event_queue_t *q, int node_idx);

#endif /* SIM_EVENT_QUEUE_H */
