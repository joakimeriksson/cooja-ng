/*
 * sim_event_queue_t unit tests — the kernel's unified (time_ns, seq) heap.
 *
 * The queue's contract is Cooja's EventQueue: pop in (time, seq) order,
 * same-time events FIFO by insertion, at most one NODE_WAKEUP per node
 * (a reschedule replaces the entry and gives it a fresh insertion order),
 * RX_BYTE / RADIO_TIMER / TEST_ACTION never coalesced.  Everything the
 * simulation does is ordered by this queue, so its pop sequence must be a
 * pure function of the keys — the differential test at the end checks the
 * heap against a sorted-array reference over a long random op sequence.
 */
#include "sim_event_queue.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

/* Each operand is evaluated once: several callers pass sim_eq_pop(&q),
 * and a second evaluation on failure would pop the next event and shift
 * every later assertion. */
#define ASSERT_EQ(actual, expected, msg) do {                            \
    long long got_ = (long long)(actual);                                \
    long long want_ = (long long)(expected);                             \
    if (got_ == want_) { passed++; }                                     \
    else { failed++;                                                     \
        printf("  FAIL: %s — got %lld, want %lld (%s:%d)\n",             \
               msg, got_, want_, __FILE__, __LINE__); }                  \
} while (0)

static sim_event_queue_t q;

/* ====================================================================
 * Ordering basics
 * ==================================================================== */

static void test_same_time_fifo(void) {
    sim_eq_init(&q);
    const int order[] = { 3, 1, 4, 0, 2 };
    for (int i = 0; i < 5; i++)
        sim_eq_schedule(&q, order[i], 100);
    for (int i = 0; i < 5; i++) {
        sim_event_t ev = sim_eq_pop(&q);
        ASSERT_EQ(ev.node_idx, order[i], "same-time wakeups pop in insertion order");
        ASSERT_EQ(ev.time_ns, 100, "same-time wakeups keep their time");
    }
    ASSERT(sim_eq_empty(&q), "queue empty after popping all");
}

static void test_time_order_beats_insertion(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 300);
    sim_eq_schedule(&q, 1, 100);
    sim_eq_schedule(&q, 2, 200);
    ASSERT_EQ(sim_eq_peek_time(&q), 100, "peek returns the earliest time");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 1, "earliest time pops first");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 2, "then the middle");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 0, "then the latest");
    ASSERT_EQ(sim_eq_peek_time(&q), INT64_MAX, "peek on empty is INT64_MAX");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, -1, "pop on empty returns node -1");
}

/* ====================================================================
 * One wakeup per node — reschedule replaces
 * ==================================================================== */

static void test_reschedule_later_replaces(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 100);
    sim_eq_schedule(&q, 1, 100);
    sim_eq_schedule(&q, 0, 500);          /* node 0 moves later */
    ASSERT_EQ(q.count, 2, "reschedule is net-zero on the count");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 1, "node 1 pops first at 100");
    sim_event_t ev = sim_eq_pop(&q);
    ASSERT_EQ(ev.node_idx, 0, "node 0's replacement pops second");
    ASSERT_EQ(ev.time_ns, 500, "at the new time");
    ASSERT(sim_eq_empty(&q), "nothing left of the old entry");
}

static void test_reschedule_earlier_replaces(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 500);
    sim_eq_schedule(&q, 1, 300);
    sim_eq_schedule(&q, 2, 400);
    sim_eq_schedule(&q, 0, 100);          /* node 0 moves earliest */
    ASSERT_EQ(q.count, 3, "reschedule is net-zero on the count");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 0, "node 0 now pops first");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 1, "then node 1");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 2, "then node 2");
}

/* Cooja scheduleNextWakeup(): a same-time reschedule gives the entry a
 * fresh insertion order, so it pops AFTER same-time entries queued in
 * between. */
static void test_reschedule_same_time_gets_fresh_seq(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 100);
    sim_eq_schedule(&q, 1, 100);
    sim_eq_schedule(&q, 0, 100);          /* re-queued at the same time */
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 1, "node 1 keeps its earlier order");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 0, "rescheduled node 0 pops after it");
}

static void test_if_earlier(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 300);
    sim_eq_schedule_if_earlier(&q, 0, 500);   /* later: ignored */
    ASSERT_EQ(sim_eq_peek_time(&q), 300, "if_earlier ignores a later time");
    sim_eq_schedule_if_earlier(&q, 0, 300);   /* equal: ignored */
    ASSERT_EQ(q.count, 1, "if_earlier with an equal time keeps the entry");
    sim_eq_schedule_if_earlier(&q, 0, 200);   /* earlier: replaces */
    ASSERT_EQ(sim_eq_peek_time(&q), 200, "if_earlier applies an earlier time");
    ASSERT_EQ(q.count, 1, "still one entry for the node");
    sim_eq_schedule_if_earlier(&q, 1, 900);   /* no entry yet: schedules */
    ASSERT_EQ(q.count, 2, "if_earlier schedules a node with no wakeup");
}

/* ====================================================================
 * Untracked kinds interleave and are never coalesced
 * ==================================================================== */

static void test_rx_bytes_interleave_and_stack(void) {
    sim_eq_init(&q);
    sim_eq_schedule(&q, 0, 200);
    sim_eq_schedule_rx_byte(&q, 0, 1, 0xAA, -50, 100);
    sim_eq_schedule_rx_byte(&q, 0, 1, 0xBB, -50, 100);   /* same time, same node */
    sim_eq_schedule_radio_timer(&q, 0, 150);
    sim_eq_schedule_test_action(&q, 120);
    sim_eq_schedule_rx_byte(&q, 0, 1, 0xCC, -50, 300);
    ASSERT_EQ(q.count, 6, "rx bytes / timers / actions are never deduped");

    sim_event_t ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_RX_BYTE && ev.byte == 0xAA, "first rx byte at 100");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_RX_BYTE && ev.byte == 0xBB, "second rx byte at 100, FIFO");
    ASSERT_EQ(ev.sender_idx, 1, "rx byte carries its sender");
    ASSERT_EQ(ev.rssi, -50, "rx byte carries its rssi");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_TEST_ACTION && ev.node_idx == -1, "test action at 120");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_RADIO_TIMER && ev.node_idx == 0, "radio timer at 150");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_NODE_WAKEUP && ev.node_idx == 0, "wakeup at 200");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_RX_BYTE && ev.byte == 0xCC, "last rx byte at 300");
}

/* A node's wakeup index must not be disturbed by the untracked kinds
 * moving around it in the heap. */
static void test_wakeup_index_survives_untracked_churn(void) {
    sim_eq_init(&q);
    for (int i = 0; i < 40; i++)
        sim_eq_schedule_rx_byte(&q, 3, 1, (uint8_t)i, -60, 1000 - i * 10);
    sim_eq_schedule(&q, 3, 555);
    for (int i = 0; i < 40; i++)
        sim_eq_schedule_radio_timer(&q, 3, 2000 - i * 30);
    sim_eq_schedule(&q, 3, 5);              /* reschedule earlier */
    ASSERT_EQ(sim_eq_peek_time(&q), 5, "rescheduled wakeup is at the top");
    sim_event_t ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_NODE_WAKEUP && ev.node_idx == 3, "and pops first");
    ASSERT_EQ(q.node_heap_idx[3], -1, "index cleared once popped");
    sim_eq_schedule(&q, 3, 3000);           /* now later than everything */
    int wakeups = 0;
    int64_t last = -1;
    while (!sim_eq_empty(&q)) {
        ev = sim_eq_pop(&q);
        ASSERT(ev.time_ns >= last, "pop times never go backwards");
        last = ev.time_ns;
        if (ev.kind == SIM_EV_NODE_WAKEUP) wakeups++;
    }
    ASSERT_EQ(wakeups, 1, "exactly one wakeup for the node");
    ASSERT_EQ(last, 3000, "and it was the last event");
}

static void test_remove_node(void) {
    sim_eq_init(&q);
    for (int n = 0; n < 4; n++) {
        sim_eq_schedule(&q, n, 100 + n);
        sim_eq_schedule_rx_byte(&q, n, 9, 0x11, -40, 50 + n);
        sim_eq_schedule_radio_timer(&q, n, 200 + n);
    }
    sim_eq_remove_node(&q, 2);
    ASSERT_EQ(q.count, 9, "all three of node 2's events removed");
    ASSERT_EQ(q.node_heap_idx[2], -1, "node 2's wakeup index cleared");
    while (!sim_eq_empty(&q)) {
        sim_event_t ev = sim_eq_pop(&q);
        ASSERT(ev.node_idx != 2, "no event for node 2 survives");
        if (ev.kind == SIM_EV_NODE_WAKEUP)
            ASSERT_EQ(q.node_heap_idx[ev.node_idx], -1, "popped wakeup index cleared");
    }
}

/* A stale node_heap_idx[] entry must read as "no wakeup" and be
 * replaced by a fresh insert, never rewritten in place: a rewritten
 * dead or foreign slot is a wakeup the node never gets. */
static bool heap_ok(const sim_event_queue_t *hq);

static void test_stale_index_self_heals(void) {
    /* Past count. */
    sim_eq_init(&q);
    sim_eq_schedule(&q, 1, 100);
    q.node_heap_idx[0] = q.count;             /* points past the heap */
    sim_eq_schedule(&q, 0, 50);
    ASSERT_EQ(q.count, 2, "stale index past count: a fresh wakeup is inserted");
    ASSERT(heap_ok(&q), "index repaired to the new slot");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 0, "and the node gets its wakeup");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 1, "node 1's wakeup untouched");

    /* Pointing at a slot an RX_BYTE has since taken. */
    sim_eq_init(&q);
    sim_eq_schedule_rx_byte(&q, 2, 3, 0xAB, -50, 100);
    q.node_heap_idx[0] = 0;                   /* names the rx byte's slot */
    sim_eq_schedule(&q, 0, 200);
    ASSERT_EQ(q.count, 2, "stale index on an rx byte: a fresh wakeup is inserted");
    ASSERT(heap_ok(&q), "index repaired to the new slot");
    sim_event_t ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_RX_BYTE && ev.byte == 0xAB, "the rx byte survives intact");
    ev = sim_eq_pop(&q);
    ASSERT(ev.kind == SIM_EV_NODE_WAKEUP && ev.node_idx == 0, "then the node's wakeup");

    /* Pointing at another node's wakeup. */
    sim_eq_init(&q);
    sim_eq_schedule(&q, 5, 100);
    q.node_heap_idx[0] = q.node_heap_idx[5];
    sim_eq_schedule(&q, 0, 300);
    ASSERT_EQ(q.count, 2, "stale index on another node: a fresh wakeup is inserted");
    ASSERT(heap_ok(&q), "both indices point at their own wakeup");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 5, "node 5 keeps its wakeup at 100");
    ASSERT_EQ(sim_eq_pop(&q).node_idx, 0, "node 0 gets its wakeup at 300");

    /* if_earlier reads the same guard: a stale slot's time is not
     * evidence of an earlier wakeup. */
    sim_eq_init(&q);
    sim_eq_schedule_rx_byte(&q, 2, 3, 0xAB, -50, 10);
    q.node_heap_idx[0] = 0;                   /* rx byte at 10 < 100 */
    sim_eq_schedule_if_earlier(&q, 0, 100);
    ASSERT_EQ(q.count, 2, "if_earlier ignores the stale slot and schedules");
    ASSERT(heap_ok(&q), "index repaired to the new slot");
}

/* ====================================================================
 * Heap invariants after arbitrary churn
 * ==================================================================== */

static bool ev_before(const sim_event_t *a, const sim_event_t *b) {
    if (a->time_ns != b->time_ns) return a->time_ns < b->time_ns;
    return a->seq < b->seq;
}

static bool heap_ok(const sim_event_queue_t *hq) {
    for (int i = 1; i < hq->count; i++)
        if (ev_before(&hq->heap[i], &hq->heap[(i - 1) / 2]))
            return false;
    /* node_heap_idx[] must point at exactly the node's wakeup, and every
     * wakeup in the heap must be indexed. */
    for (int n = 0; n < SIM_EQ_MAX_NODES; n++) {
        int i = hq->node_heap_idx[n];
        if (i < 0) continue;
        if (i >= hq->count) return false;
        if (hq->heap[i].kind != SIM_EV_NODE_WAKEUP || hq->heap[i].node_idx != n)
            return false;
    }
    for (int i = 0; i < hq->count; i++)
        if (hq->heap[i].kind == SIM_EV_NODE_WAKEUP &&
            hq->node_heap_idx[hq->heap[i].node_idx] != i)
            return false;
    return true;
}

/* ====================================================================
 * Differential test against a sorted-array reference
 * ==================================================================== */

#define REF_MAX 4096
typedef struct {
    sim_event_t ev[REF_MAX];
    int count;
    uint64_t next_seq;
} ref_queue_t;

static void ref_insert(ref_queue_t *r, sim_event_t ev) {
    if (r->count >= REF_MAX) return;
    int i = r->count;
    while (i > 0 && ev_before(&ev, &r->ev[i - 1])) {
        r->ev[i] = r->ev[i - 1];
        i--;
    }
    r->ev[i] = ev;
    r->count++;
}

static int ref_find_wakeup(const ref_queue_t *r, int node) {
    for (int i = 0; i < r->count; i++)
        if (r->ev[i].kind == SIM_EV_NODE_WAKEUP && r->ev[i].node_idx == node)
            return i;
    return -1;
}

static void ref_erase(ref_queue_t *r, int i) {
    memmove(&r->ev[i], &r->ev[i + 1], (size_t)(r->count - i - 1) * sizeof r->ev[0]);
    r->count--;
}

/* The pre-existing semantics: remove the old wakeup, insert a new one
 * with a fresh seq. */
static void ref_schedule(ref_queue_t *r, int node, int64_t t) {
    int i = ref_find_wakeup(r, node);
    if (i >= 0) ref_erase(r, i);
    sim_event_t ev = { .kind = SIM_EV_NODE_WAKEUP, .node_idx = node,
                       .time_ns = t, .seq = r->next_seq++, .sender_idx = -1 };
    ref_insert(r, ev);
}

static void ref_schedule_if_earlier(ref_queue_t *r, int node, int64_t t) {
    int i = ref_find_wakeup(r, node);
    if (i >= 0 && r->ev[i].time_ns <= t) return;
    ref_schedule(r, node, t);
}

static void ref_other(ref_queue_t *r, sim_event_kind_t kind, int node,
                      int64_t t, int sender, uint8_t byte) {
    sim_event_t ev = { .kind = kind, .node_idx = node, .time_ns = t,
                       .seq = r->next_seq++, .sender_idx = sender, .byte = byte };
    ref_insert(r, ev);
}

static sim_event_t ref_pop(ref_queue_t *r) {
    sim_event_t ev = r->ev[0];
    ref_erase(r, 0);
    return ev;
}

static void ref_remove_node(ref_queue_t *r, int node) {
    for (int i = r->count - 1; i >= 0; i--)
        if (r->ev[i].node_idx == node) ref_erase(r, i);
}

/* Exact: seq and generation are compared too, so a schedule that
 * consumes a seq twice, or not at all, is caught on the next pop rather
 * than only when a same-time collision happens to expose it. */
static bool ev_same(const sim_event_t *a, const sim_event_t *b) {
    return a->kind == b->kind && a->node_idx == b->node_idx &&
           a->time_ns == b->time_ns && a->seq == b->seq &&
           a->target_generation == b->target_generation &&
           a->byte == b->byte &&
           (a->kind != SIM_EV_RX_BYTE || a->sender_idx == b->sender_idx);
}

static uint64_t rng_state = 0x9E3779B97F4A7C15ull;
static uint32_t rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (uint32_t)(rng_state >> 16);
}

static void test_differential(void) {
    static ref_queue_t ref;
    memset(&ref, 0, sizeof ref);
    sim_eq_init(&q);
    const int nodes = 12;
    int64_t now = 0;
    int mismatches = 0, invariant_breaks = 0, pops = 0;

    for (int step = 0; step < 60000; step++) {
        uint32_t r = rnd();
        /* Wakeups only replace (12 nodes -> at most 12 entries); the
         * untracked kinds are inserted and popped at the same rate.  If
         * the backlog ever gets deep, pop until it drains. */
        if (ref.count > 2000) r = 15;
        int node = (int)(rnd() % (uint32_t)nodes);
        /* Mostly near-future times, with same-time collisions on purpose. */
        int64_t t = now + (int64_t)(rnd() % 8) * 1000;
        switch (r % 16) {
        case 0: case 1: case 2: case 3:
            sim_eq_schedule(&q, node, t);
            ref_schedule(&ref, node, t);
            break;
        case 4: case 5: case 6:
            sim_eq_schedule_if_earlier(&q, node, t);
            ref_schedule_if_earlier(&ref, node, t);
            break;
        case 7: case 8: {
            uint8_t b = (uint8_t)rnd();
            sim_eq_schedule_rx_byte(&q, node, (node + 1) % nodes, b, -55, t);
            ref_other(&ref, SIM_EV_RX_BYTE, node, t, (node + 1) % nodes, b);
            break;
        }
        case 9:
            sim_eq_schedule_radio_timer(&q, node, t);
            ref_other(&ref, SIM_EV_RADIO_TIMER, node, t, -1, 0);
            break;
        case 10:
            sim_eq_schedule_test_action(&q, t);
            ref_other(&ref, SIM_EV_TEST_ACTION, -1, t, -1, 0);
            break;
        case 11:
            if (rnd() % 64 == 0) {
                sim_eq_remove_node(&q, node);
                ref_remove_node(&ref, node);
            }
            break;
        default:
            if (ref.count > 0) {
                sim_event_t a = sim_eq_pop(&q);
                sim_event_t b = ref_pop(&ref);
                pops++;
                if (!ev_same(&a, &b)) {
                    if (mismatches == 0)
                        printf("  first pop mismatch at step %d: heap kind=%d node=%d t=%lld seq=%llu | ref kind=%d node=%d t=%lld seq=%llu (counts %d/%d)\n",
                               step, (int)a.kind, a.node_idx, (long long)a.time_ns, (unsigned long long)a.seq,
                               (int)b.kind, b.node_idx, (long long)b.time_ns, (unsigned long long)b.seq, q.count, ref.count);
                    mismatches++;
                }
                if (a.time_ns > now) now = a.time_ns;
            }
            break;
        }
        if (q.count != ref.count) {
            if (mismatches == 0)
                printf("  first count mismatch at step %d (op %u): heap %d ref %d\n", step, r % 16, q.count, ref.count);
            mismatches++;
        }
        if ((step & 63) == 0 && !heap_ok(&q)) invariant_breaks++;
    }
    while (ref.count > 0) {
        sim_event_t a = sim_eq_pop(&q);
        sim_event_t b = ref_pop(&ref);
        pops++;
        if (!ev_same(&a, &b))
            mismatches++;
    }
    printf("  differential: %d pops, %d mismatches\n", pops, mismatches);
    ASSERT(pops > 10000, "the random walk actually popped events");
    ASSERT_EQ(mismatches, 0, "heap pops match the sorted-array reference");
    ASSERT_EQ(invariant_breaks, 0, "heap order + wakeup index held throughout");
    ASSERT(sim_eq_empty(&q), "both queues drained together");
}

/* ====================================================================
 * Entry
 * ==================================================================== */

int run_event_queue_tests(int verbose) {
    (void)verbose;
    printf("=== sim_event_queue_t Unit Tests ===\n");

    test_same_time_fifo();
    test_time_order_beats_insertion();
    test_reschedule_later_replaces();
    test_reschedule_earlier_replaces();
    test_reschedule_same_time_gets_fresh_seq();
    test_if_earlier();
    test_rx_bytes_interleave_and_stack();
    test_wakeup_index_survives_untracked_churn();
    test_remove_node();
    test_stale_index_self_heals();
    test_differential();

    printf("  %d passed, %d failed\n", passed, failed);
    return failed;
}
