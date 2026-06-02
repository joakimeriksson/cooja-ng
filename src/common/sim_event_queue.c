/*
 * Simulation event queue — min-heap implementation.
 *
 * Sorted by (time_ns, seq) so events at the same time execute in
 * insertion order (FIFO), matching Cooja's EventQueue behavior. Holds
 * SIM_EV_NODE_WAKEUP and SIM_EV_RX_BYTE entries in the same heap so
 * mote ticks and per-byte RF deliveries interleave by time as Cooja does.
 */
#include "sim_event_queue.h"
#include <string.h>
#include <stdio.h>

/* Compare two events: returns true if a should come before b */
static inline bool ev_less(const sim_event_t *a, const sim_event_t *b) {
    if (a->time_ns != b->time_ns)
        return a->time_ns < b->time_ns;
    return a->seq < b->seq;
}

/* Track this node's pending NODE_WAKEUP slot. RX_BYTE entries are not
 * tracked in node_heap_idx[] since multiple per node can coexist. */
static inline void track_slot(sim_event_queue_t *q, int slot) {
    const sim_event_t *e = &q->heap[slot];
    if (e->kind == SIM_EV_NODE_WAKEUP &&
        e->node_idx >= 0 && e->node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[e->node_idx] = slot;
}

static void heap_swap(sim_event_queue_t *q, int i, int j) {
    sim_event_t tmp = q->heap[i];
    q->heap[i] = q->heap[j];
    q->heap[j] = tmp;
    track_slot(q, i);
    track_slot(q, j);
}

static void heap_sift_up(sim_event_queue_t *q, int i) {
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (ev_less(&q->heap[i], &q->heap[parent])) {
            heap_swap(q, i, parent);
            i = parent;
        } else {
            break;
        }
    }
}

static void heap_sift_down(sim_event_queue_t *q, int i) {
    int n = q->count;
    while (1) {
        int smallest = i;
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        if (left < n && ev_less(&q->heap[left], &q->heap[smallest]))
            smallest = left;
        if (right < n && ev_less(&q->heap[right], &q->heap[smallest]))
            smallest = right;
        if (smallest != i) {
            heap_swap(q, i, smallest);
            i = smallest;
        } else {
            break;
        }
    }
}

static void clear_index(sim_event_queue_t *q, int slot) {
    const sim_event_t *e = &q->heap[slot];
    if (e->kind == SIM_EV_NODE_WAKEUP &&
        e->node_idx >= 0 && e->node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[e->node_idx] = -1;
}

static void remove_heap_index(sim_event_queue_t *q, int i) {
    if (i < 0 || i >= q->count)
        return;
    clear_index(q, i);
    q->heap[i] = q->heap[--q->count];
    if (i < q->count) {
        track_slot(q, i);
        heap_sift_up(q, i);
        heap_sift_down(q, i);
    }
}

void sim_eq_init(sim_event_queue_t *q) {
    memset(q, 0, sizeof(*q));
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        q->node_heap_idx[i] = -1;
}

void sim_eq_schedule_gen(sim_event_queue_t *q, int node_idx, int64_t time_ns,
                          uint32_t target_generation) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid event node index %d\n", node_idx);
        return;
    }
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), dropping wakeup for node %d\n",
                q->count, node_idx);
        return;
    }
    int existing = q->node_heap_idx[node_idx];
    if (existing >= 0) {
        /* Match Cooja scheduleNextWakeup(): rescheduling an already-queued
         * execute event removes the old queue entry and inserts a new one,
         * giving it a fresh same-time insertion order. */
        remove_heap_index(q, existing);
    }
    int i = q->count++;
    q->heap[i].kind = SIM_EV_NODE_WAKEUP;
    q->heap[i].node_idx = node_idx;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    q->heap[i].sender_idx = -1;
    q->heap[i].byte = 0;
    q->heap[i].rssi = 0;
    q->heap[i].target_generation = target_generation;
    q->node_heap_idx[node_idx] = i;
    heap_sift_up(q, i);
}

void sim_eq_schedule(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    sim_eq_schedule_gen(q, node_idx, time_ns, 0u);
}

void sim_eq_schedule_if_earlier_gen(sim_event_queue_t *q, int node_idx,
                                     int64_t time_ns,
                                     uint32_t target_generation) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid event node index %d\n", node_idx);
        return;
    }
    int i = q->node_heap_idx[node_idx];
    if (i >= 0 && q->heap[i].time_ns <= time_ns)
        return;  /* already scheduled earlier — ignore */
    sim_eq_schedule_gen(q, node_idx, time_ns, target_generation);
}

void sim_eq_schedule_if_earlier(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    sim_eq_schedule_if_earlier_gen(q, node_idx, time_ns, 0u);
}

void sim_eq_schedule_rx_byte_gen(sim_event_queue_t *q, int node_idx,
                                  int sender_idx, uint8_t byte, int8_t rssi,
                                  int64_t time_ns,
                                  uint32_t target_generation) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid rx-byte node index %d\n", node_idx);
        return;
    }
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), dropping rx byte for node %d\n",
                q->count, node_idx);
        return;
    }
    int i = q->count++;
    q->heap[i].kind = SIM_EV_RX_BYTE;
    q->heap[i].node_idx = node_idx;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    q->heap[i].sender_idx = sender_idx;
    q->heap[i].byte = byte;
    q->heap[i].rssi = rssi;
    q->heap[i].target_generation = target_generation;
    /* RX_BYTE events do not update node_heap_idx[] — that index tracks
     * only the single pending NODE_WAKEUP per node. */
    heap_sift_up(q, i);
}

void sim_eq_schedule_rx_byte(sim_event_queue_t *q, int node_idx, int sender_idx,
                             uint8_t byte, int8_t rssi, int64_t time_ns) {
    sim_eq_schedule_rx_byte_gen(q, node_idx, sender_idx, byte, rssi, time_ns, 0u);
}

sim_event_t sim_eq_pop(sim_event_queue_t *q) {
    if (q->count == 0) {
        sim_event_t empty = { .kind = SIM_EV_NODE_WAKEUP, .node_idx = -1,
                              .time_ns = INT64_MAX, .seq = 0,
                              .sender_idx = -1, .byte = 0, .rssi = 0 };
        return empty;
    }
    sim_event_t top = q->heap[0];
    clear_index(q, 0);
    q->heap[0] = q->heap[--q->count];
    if (q->count > 0) {
        track_slot(q, 0);
        heap_sift_down(q, 0);
    }
    return top;
}

int64_t sim_eq_peek_time(const sim_event_queue_t *q) {
    if (q->count == 0) return INT64_MAX;
    return q->heap[0].time_ns;
}

sim_event_t sim_eq_peek(const sim_event_queue_t *q) {
    if (q->count == 0) {
        sim_event_t empty = { .kind = SIM_EV_NODE_WAKEUP, .node_idx = -1,
                              .time_ns = INT64_MAX, .seq = UINT64_MAX,
                              .sender_idx = -1, .byte = 0, .rssi = 0 };
        return empty;
    }
    return q->heap[0];
}

bool sim_eq_empty(const sim_event_queue_t *q) {
    return q->count == 0;
}

void sim_eq_remove_node(sim_event_queue_t *q, int node_idx) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES)
        return;
    /* Compact: drop everything targeting this node, then re-heapify and
     * rebuild node_heap_idx[]. O(n) but only invoked on node teardown. */
    int write = 0;
    for (int read = 0; read < q->count; read++) {
        if (q->heap[read].node_idx != node_idx) {
            if (write != read)
                q->heap[write] = q->heap[read];
            write++;
        }
    }
    q->count = write;
    for (int n = 0; n < SIM_EQ_MAX_NODES; n++)
        q->node_heap_idx[n] = -1;
    for (int i = q->count / 2 - 1; i >= 0; i--)
        heap_sift_down(q, i);
    for (int i = 0; i < q->count; i++)
        track_slot(q, i);
}
