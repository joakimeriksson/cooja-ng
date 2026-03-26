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

void sim_eq_init(sim_event_queue_t *q) {
    memset(q, 0, sizeof(*q));
}

void sim_eq_schedule(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), dropping event for node %d\n",
                q->count, node_idx);
        return;
    }
    int i = q->count++;
    q->heap[i].node_idx = node_idx;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    heap_sift_up(q, i);
}

void sim_eq_schedule_if_earlier(sim_event_queue_t *q, int node_idx, int64_t time_ns) {
    /* Check if this node already has an event at an earlier or equal time */
    for (int i = 0; i < q->count; i++) {
        if (q->heap[i].node_idx == node_idx) {
            if (q->heap[i].time_ns <= time_ns)
                return;  /* already scheduled earlier — ignore */
            /* Remove existing event and re-insert at new time */
            q->heap[i] = q->heap[--q->count];
            /* Re-heapify */
            if (i < q->count) {
                heap_sift_up(q, i);
                heap_sift_down(q, i);
            }
            break;
        }
    }
    sim_eq_schedule(q, node_idx, time_ns);
}

sim_event_t sim_eq_pop(sim_event_queue_t *q) {
    if (q->count == 0) {
        sim_event_t empty = { .node_idx = -1, .time_ns = INT64_MAX, .seq = 0 };
        return empty;
    }
    sim_event_t top = q->heap[0];
    q->heap[0] = q->heap[--q->count];
    if (q->count > 0)
        heap_sift_down(q, 0);
    return top;
}

int64_t sim_eq_peek_time(const sim_event_queue_t *q) {
    if (q->count == 0) return INT64_MAX;
    return q->heap[0].time_ns;
}

bool sim_eq_empty(const sim_event_queue_t *q) {
    return q->count == 0;
}

void sim_eq_remove_node(sim_event_queue_t *q, int node_idx) {
    int i = 0;
    while (i < q->count) {
        if (q->heap[i].node_idx == node_idx) {
            q->heap[i] = q->heap[--q->count];
            if (i < q->count) {
                heap_sift_up(q, i);
                heap_sift_down(q, i);
            }
            /* Don't increment i — check the swapped element */
        } else {
            i++;
        }
    }
}
