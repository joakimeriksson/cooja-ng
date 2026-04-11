/*
 * Simulation event queue — min-heap implementation.
 *
 * Sorted by (time_ns, seq) so events at the same time execute in
 * insertion order (FIFO), matching COOJA's EventQueue behavior.
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

static void heap_swap(sim_event_queue_t *q, int i, int j) {
    sim_event_t tmp = q->heap[i];
    q->heap[i] = q->heap[j];
    q->heap[j] = tmp;
    if (q->heap[i].node_idx >= 0 && q->heap[i].node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[q->heap[i].node_idx] = i;
    if (q->heap[j].node_idx >= 0 && q->heap[j].node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[q->heap[j].node_idx] = j;
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

static void remove_heap_index(sim_event_queue_t *q, int i) {
    if (i < 0 || i >= q->count)
        return;
    int removed_node = q->heap[i].node_idx;
    if (removed_node >= 0 && removed_node < SIM_EQ_MAX_NODES)
        q->node_heap_idx[removed_node] = -1;
    q->heap[i] = q->heap[--q->count];
    if (i < q->count) {
        if (q->heap[i].node_idx >= 0 && q->heap[i].node_idx < SIM_EQ_MAX_NODES)
            q->node_heap_idx[q->heap[i].node_idx] = i;
        heap_sift_up(q, i);
        heap_sift_down(q, i);
    }
}

void sim_eq_init(sim_event_queue_t *q) {
    memset(q, 0, sizeof(*q));
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        q->node_heap_idx[i] = -1;
}

void sim_eq_schedule(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid event node index %d\n", node_idx);
        return;
    }
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), dropping event for node %d\n",
                q->count, node_idx);
        return;
    }
    int existing = q->node_heap_idx[node_idx];
    if (existing >= 0) {
        /* Match COOJA scheduleNextWakeup(): rescheduling an already-queued
         * execute event removes the old queue entry and inserts a new one,
         * giving it a fresh same-time insertion order. */
        remove_heap_index(q, existing);
    }
    int i = q->count++;
    q->heap[i].node_idx = node_idx;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    q->node_heap_idx[node_idx] = i;
    heap_sift_up(q, i);
}

void sim_eq_schedule_if_earlier(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid event node index %d\n", node_idx);
        return;
    }
    int i = q->node_heap_idx[node_idx];
    if (i >= 0 && q->heap[i].time_ns <= time_ns)
        return;  /* already scheduled earlier — ignore */
    sim_eq_schedule(q, node_idx, time_ns);
}

sim_event_t sim_eq_pop(sim_event_queue_t *q) {
    if (q->count == 0) {
        sim_event_t empty = { .node_idx = -1, .time_ns = INT64_MAX, .seq = 0 };
        return empty;
    }
    sim_event_t top = q->heap[0];
    if (top.node_idx >= 0 && top.node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[top.node_idx] = -1;
    q->heap[0] = q->heap[--q->count];
    if (q->count > 0) {
        if (q->heap[0].node_idx >= 0 && q->heap[0].node_idx < SIM_EQ_MAX_NODES)
            q->node_heap_idx[q->heap[0].node_idx] = 0;
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
        sim_event_t empty = { .node_idx = -1, .time_ns = INT64_MAX, .seq = UINT64_MAX };
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
    int i = q->node_heap_idx[node_idx];
    if (i < 0)
        return;
    remove_heap_index(q, i);
}
