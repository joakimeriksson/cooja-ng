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

/* Sift with a hole: lift the element out once, slide parents (or the
 * smaller child) into the hole, and drop the element in at the end.  One
 * copy per level instead of the three a swap costs, and one track_slot
 * per moved element.  Order is a pure function of the (time_ns, seq)
 * keys, which are unique, so the heap's pop sequence is unchanged. */
static void heap_sift_up(sim_event_queue_t *q, int i) {
    sim_event_t ev = q->heap[i];
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (!ev_less(&ev, &q->heap[parent]))
            break;
        q->heap[i] = q->heap[parent];
        track_slot(q, i);
        i = parent;
    }
    q->heap[i] = ev;
    track_slot(q, i);
}

static void heap_sift_down(sim_event_queue_t *q, int i) {
    int n = q->count;
    sim_event_t ev = q->heap[i];
    for (;;) {
        int left = 2 * i + 1;
        if (left >= n) break;
        int child = left;
        int right = left + 1;
        if (right < n && ev_less(&q->heap[right], &q->heap[left]))
            child = right;
        if (!ev_less(&q->heap[child], &ev))
            break;
        q->heap[i] = q->heap[child];
        track_slot(q, i);
        i = child;
    }
    q->heap[i] = ev;
    track_slot(q, i);
}

/* The node's live NODE_WAKEUP slot, or -1.  node_heap_idx[] is trusted
 * only when the slot it names is inside the heap and holds this node's
 * wakeup; a stale entry (past count, or a slot since reused by an
 * untracked kind or another node) reads as "no wakeup" and the caller
 * inserts afresh, as remove_heap_index() did before the in-place path.
 * The invariant holds today (the suite's heap_ok checks it), so this is
 * the safety net for a future path that breaks it: a mote with a lost
 * index gets a new wakeup instead of stalling for good.  What the guard
 * cannot do is find a wakeup the index has lost sight of; if one is in
 * the heap, the node ticks twice, which was also the case before. */
static inline int wakeup_slot(const sim_event_queue_t *q, int node_idx) {
    int i = q->node_heap_idx[node_idx];
    if (i >= 0 && i < q->count &&
        q->heap[i].kind == SIM_EV_NODE_WAKEUP &&
        q->heap[i].node_idx == node_idx)
        return i;
    return -1;
}

static void clear_index(sim_event_queue_t *q, int slot) {
    const sim_event_t *e = &q->heap[slot];
    if (e->kind == SIM_EV_NODE_WAKEUP &&
        e->node_idx >= 0 && e->node_idx < SIM_EQ_MAX_NODES)
        q->node_heap_idx[e->node_idx] = -1;
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
    /* Reschedule: the node already has a pending wakeup.  Match Cooja
     * scheduleNextWakeup() — the old entry is replaced by one with a fresh
     * same-time insertion order — by rewriting the entry in place with a
     * new seq and sifting it to where the new key belongs.  Same pop order
     * as remove + insert (the key alone decides), at one sift instead of
     * two, and net-zero on the count, so a full queue still replaces the
     * entry rather than dropping the wakeup (which stalled the mote). */
    int existing = wakeup_slot(q, node_idx);
    if (existing >= 0) {
        sim_event_t *e = &q->heap[existing];
        bool earlier = time_ns < e->time_ns;
        e->time_ns = time_ns;
        e->seq = q->next_seq++;
        e->target_generation = target_generation;
        /* A larger seq at the same time, or a later time, only ever moves
         * the entry down; an earlier time only ever moves it up. */
        if (earlier)
            heap_sift_up(q, existing);
        else
            heap_sift_down(q, existing);
        return;
    }
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), dropping wakeup for node %d\n",
                q->count, node_idx);
        return;
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
    int i = wakeup_slot(q, node_idx);
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

void sim_eq_schedule_radio_timer_gen(sim_event_queue_t *q, int node_idx,
                                     int64_t time_ns,
                                     uint32_t target_generation) {
    if (node_idx < 0 || node_idx >= SIM_EQ_MAX_NODES) {
        fprintf(stderr, "WARNING: invalid radio-timer node index %d\n", node_idx);
        return;
    }
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), "
                "dropping radio timer for node %d\n", q->count, node_idx);
        return;
    }
    int i = q->count++;
    memset(&q->heap[i], 0, sizeof(q->heap[i]));
    q->heap[i].kind = SIM_EV_RADIO_TIMER;
    q->heap[i].node_idx = node_idx;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    q->heap[i].sender_idx = -1;
    q->heap[i].target_generation = target_generation;
    /* Like RX_BYTE, RADIO_TIMER events do not update node_heap_idx[] —
     * that index tracks only the single pending NODE_WAKEUP per node. */
    heap_sift_up(q, i);
}

void sim_eq_schedule_radio_timer(sim_event_queue_t *q, int node_idx,
                                 int64_t time_ns) {
    sim_eq_schedule_radio_timer_gen(q, node_idx, time_ns, 0u);
}

void sim_eq_schedule_test_action(sim_event_queue_t *q, int64_t time_ns) {
    if (q->count >= SIM_EQ_MAX_EVENTS) {
        fprintf(stderr, "WARNING: event queue full (%d events), "
                "dropping test action\n", q->count);
        return;
    }
    int i = q->count++;
    memset(&q->heap[i], 0, sizeof(q->heap[i]));
    q->heap[i].kind = SIM_EV_TEST_ACTION;
    q->heap[i].node_idx = -1;
    q->heap[i].time_ns = time_ns;
    q->heap[i].seq = q->next_seq++;
    /* generation 0 = untracked; not tied to any mote slot */
    heap_sift_up(q, i);
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
