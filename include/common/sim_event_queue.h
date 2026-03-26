/*
 * Simulation event queue — min-heap priority queue matching COOJA's model.
 *
 * Each event is a (node_idx, time_ns, seq) triple. Events are ordered by
 * time first, then insertion order (seq) for FIFO within same timestamp.
 * This matches COOJA's EventQueue behavior.
 */
#ifndef SIM_EVENT_QUEUE_H
#define SIM_EVENT_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define SIM_EQ_MAX_EVENTS 16384

typedef struct {
    int node_idx;       /* which node to tick (-1 = unused) */
    int64_t time_ns;    /* simulation time */
    uint64_t seq;       /* insertion order (tiebreaker) */
} sim_event_t;

typedef struct {
    sim_event_t heap[SIM_EQ_MAX_EVENTS];
    int count;
    uint64_t next_seq;
} sim_event_queue_t;

/* Initialize an empty event queue */
void sim_eq_init(sim_event_queue_t *q);

/* Schedule a node to wake at time_ns. Multiple events for the same node
 * are allowed — the earliest will fire first. */
void sim_eq_schedule(sim_event_queue_t *q, int node_idx, int64_t time_ns);

/* Schedule a node, but only if the new time is EARLIER than any existing
 * event for this node. Matches COOJA's scheduleNextWakeup behavior. */
void sim_eq_schedule_if_earlier(sim_event_queue_t *q, int node_idx, int64_t time_ns);

/* Pop the earliest event. Returns event with node_idx=-1 if empty. */
sim_event_t sim_eq_pop(sim_event_queue_t *q);

/* Peek at the earliest event time without removing. Returns INT64_MAX if empty. */
int64_t sim_eq_peek_time(const sim_event_queue_t *q);

/* Check if queue is empty */
bool sim_eq_empty(const sim_event_queue_t *q);

/* Remove all events for a specific node */
void sim_eq_remove_node(sim_event_queue_t *q, int node_idx);

#endif /* SIM_EVENT_QUEUE_H */
