/*
 * sim_radio_bus unit tests (Phase 5 guardrail, M24).
 *
 * Drive the radio bus in isolation through mock mote_radio_ops_t
 * receivers + scripted chip TX byte streams, with a real
 * sim_runtime_t event queue + radio_medium.  No real CPU, no chips.
 *
 * Purpose (docs/design/refactor-plan.md §3.18): pin the bus's current
 * behaviour BEFORE Phase 5 moves frame-delivery policy into it, so the
 * later character-identical moves have a fast unit-level detector.
 * Covers what the bus owns today (M9.4/M9.5): frame assembler, TX
 * capture + byte clock, delivery-mode routing, re-entrant depth
 * staging, RX-stall timer, spectrum matching, FIFO sizing.  The
 * collision-window / ACK-window / backpressure tests arrive with the
 * milestones that move that policy in (M26/M27).
 *
 * These tests assert CURRENT behaviour — if one exposes a bug, document
 * it; do not fix it in this milestone.
 */
#include "sim_radio_bus.h"
#include "sim_runtime.h"
#include "radio_medium.h"
#include "sim_event_queue.h"

#include <stdio.h>
#include <string.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

#define ASSERT_EQ(actual, expected, msg) do {                            \
    long long _a = (long long)(actual), _e = (long long)(expected);      \
    if (_a == _e) { passed++; }                                          \
    else { failed++;                                                     \
        printf("  FAIL: %s — got %lld, want %lld (%s:%d)\n",            \
               msg, _a, _e, __FILE__, __LINE__); }                       \
} while (0)

/* ============================================================
 * Mock receiver — records every byte the bus delivers to it.
 * ============================================================ */

#define MOCK_CAP 1024
typedef struct {
    int      idx;
    int      recv_count;
    uint8_t  recv_byte[MOCK_CAP];
    int8_t   recv_rssi[MOCK_CAP];
    int64_t  recv_time[MOCK_CAP];   /* sim now_ns at delivery */
    int      rxfifo;                /* reported free FIFO bytes */
    bool     busy;
    int      stall_count;
    /* Optional re-entrancy: when the byte == reenter_on (>=0) is
     * delivered synchronously (SYNC mode), emit one ACK byte back
     * through the bus as `reenter_sender`. */
    int      reenter_on;
    int      reenter_sender;
    sim_radio_bus_t *bus;
    sim_runtime_t   *sim;
    int64_t  now_at_recv;           /* the pump stamps this before calling */
} mock_rx_t;

static void mock_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    mock_rx_t *r = (mock_rx_t *)m;
    if (r->recv_count < MOCK_CAP) {
        r->recv_byte[r->recv_count] = byte;
        r->recv_rssi[r->recv_count] = rssi;
        r->recv_time[r->recv_count] = r->now_at_recv;
        r->recv_count++;
    }
    if (r->reenter_on >= 0 && byte == (uint8_t)r->reenter_on)
        sim_radio_bus_tx_byte(r->bus, r->sim, r->reenter_sender, 0, 0xAC);
}
static int mock_rxfifo_available(void *m) { return ((mock_rx_t *)m)->rxfifo; }
static bool mock_rx_busy(void *m) { return ((mock_rx_t *)m)->busy; }
static void mock_rx_stall(void *m) { ((mock_rx_t *)m)->stall_count++; }

static const mote_radio_ops_t mock_ops = {
    mock_receive_byte, mock_rxfifo_available, mock_rx_busy, NULL, NULL, NULL,
};
static const mote_radio_ops_t mock_ops_stall = {
    mock_receive_byte, mock_rxfifo_available, mock_rx_busy, mock_rx_stall, NULL, NULL,
};

/* ============================================================
 * Fixture + event pump.
 * ============================================================ */

typedef struct {
    sim_runtime_t   sim;
    sim_radio_bus_t bus;
    mock_rx_t       rx[8];
} fixture_t;

static void fx_init(fixture_t *f, int node_count) {
    memset(f, 0, sizeof(*f));
    sim_runtime_init(&f->sim);
    sim_eq_init(&f->sim.event_queue);
    radio_medium_init(&f->sim.radio_medium, node_count);  /* NONE by default */
    f->sim.radio_bus = &f->bus;
    for (int i = 0; i < (int)(sizeof(f->rx) / sizeof(f->rx[0])); i++) {
        f->rx[i].idx = i;
        f->rx[i].rxfifo = 128;
        f->rx[i].reenter_on = -1;
        f->rx[i].bus = &f->bus;
        f->rx[i].sim = &f->sim;
    }
}

/* Drain SIM_EV_RX_BYTE / SIM_EV_RADIO_TIMER up to horizon, dispatching
 * to the bus's registered ops exactly as the runner's RX_BYTE / timer
 * dispatch does (this is the production seam). */
static void fx_pump(fixture_t *f, int64_t horizon) {
    sim_runtime_t *sim = &f->sim;
    while (!sim_eq_empty(&sim->event_queue)) {
        int64_t t = sim_eq_peek_time(&sim->event_queue);
        if (t > horizon) break;
        sim_event_t ev = sim_eq_pop(&sim->event_queue);
        sim->now_ns = ev.time_ns;
        if (ev.kind == SIM_EV_RX_BYTE) {
            const mote_radio_ops_t *ops = f->bus.ops[ev.node_idx];
            if (ops) {
                mock_rx_t *r = (mock_rx_t *)f->bus.mote[ev.node_idx];
                r->now_at_recv = ev.time_ns;
                ops->receive_byte(r, ev.byte, ev.rssi);
            }
        } else if (ev.kind == SIM_EV_RADIO_TIMER) {
            if (sim_radio_bus_rx_stall_expired(&f->bus, sim, ev.node_idx,
                                               ev.time_ns)) {
                const mote_radio_ops_t *ops = f->bus.ops[ev.node_idx];
                if (ops && ops->rx_stall)
                    ops->rx_stall(f->bus.mote[ev.node_idx]);
            }
        }
    }
}

/* Feed a byte sequence to the assembler, return how many feeds reported
 * frame-complete (should be exactly 1 for a single well-formed frame). */
static int feed_seq(tx_frame_asm_t *a, const uint8_t *seq, int n) {
    int completes = 0;
    for (int i = 0; i < n; i++)
        if (sim_radio_bus_asm_feed(a, seq[i])) completes++;
    return completes;
}

/* Mock emulated mote: provides the sim_mote_ops the bus's delivery path
 * drives (rx_byte_sync / step_until / cycles / freq_hz). */
typedef struct {
    sim_mote_t mote;
    int      sync_count;
    int64_t  sync_times[256];
    int      step_count;
    int64_t  cyc;
    uint32_t freq;
} mock_mote_t;

static void mm_rx_byte_sync(sim_mote_t *m, int64_t t) {
    mock_mote_t *mm = (mock_mote_t *)m->impl;
    if (mm->sync_count < 256) mm->sync_times[mm->sync_count] = t;
    mm->sync_count++;
}
static void mm_step_until(sim_mote_t *m, int64_t target) {
    mock_mote_t *mm = (mock_mote_t *)m->impl;
    mm->step_count++;
    mm->cyc = target;
}
static int64_t  mm_cycles(const sim_mote_t *m) { return ((mock_mote_t *)m->impl)->cyc; }
static uint32_t mm_freq(const sim_mote_t *m)   { return ((mock_mote_t *)m->impl)->freq; }

static const sim_mote_ops_t mm_ops = {
    .rx_byte_sync = mm_rx_byte_sync,
    .step_until   = mm_step_until,
    .cycles       = mm_cycles,
    .freq_hz      = mm_freq,
};

/* Register node idx as an emulated receiver: a mock_rx for the radio
 * endpoint + a mock_mote for the sim_mote ops, with the given caps. */
static void register_emulated(fixture_t *f, mock_mote_t *mm, int idx,
                              uint32_t caps) {
    memset(mm, 0, sizeof(*mm));
    mm->freq = 1000000;          /* 1 MHz → 1 ms = 1000 cycles */
    mm->mote.impl = mm;
    mm->mote.ops = &mm_ops;
    sim_runtime_register_mote(&f->sim, idx, &mm->mote);
    sim_radio_bus_register(&f->bus, idx, &mock_ops, &f->rx[idx],
                           SIM_RADIO_DELIVERY_PER_BYTE, caps);
}

/* A short 802.15.4 frame body (preamble..payload). */
static int short_frame(uint8_t *buf, int payload) {
    int n = 0;
    buf[n++] = 0; buf[n++] = 0; buf[n++] = 0; buf[n++] = 0;
    buf[n++] = 0x7A; buf[n++] = (uint8_t)payload;
    for (int i = 0; i < payload; i++) buf[n++] = (uint8_t)(0x40 + i);
    return n;
}


/* ============================================================
 * Frame assembler.
 * ============================================================ */

static void test_asm_802154(void) {
    tx_frame_asm_t a;
    sim_radio_bus_asm_reset(&a);
    /* preamble(4×0x00) + SFD(0x7A) + len(0x0A=10) + 10 payload bytes.
     * 802.15.4 length already counts the 2 FCS the chip appends, so the
     * frame completes on the 10th payload byte. */
    uint8_t frame[] = { 0,0,0,0, 0x7A, 10, 1,2,3,4,5,6,7,8,9,10 };
    int done = feed_seq(&a, frame, (int)sizeof(frame));
    ASSERT_EQ(done, 1, "802.15.4 frame completes exactly once");
    ASSERT_EQ(a.subghz, 0, "802.15.4 frame not flagged sub-GHz");
    ASSERT_EQ(a.expected_len, 10, "802.15.4 expected_len sticky after complete");

    /* Completes on the LAST payload byte, not before. */
    sim_radio_bus_asm_reset(&a);
    for (int i = 0; i < 15; i++)
        ASSERT(!sim_radio_bus_asm_feed(&a, frame[i]), "no early completion");
    ASSERT(sim_radio_bus_asm_feed(&a, frame[15]), "completes on 10th payload byte");
}

static void test_asm_802154g(void) {
    tx_frame_asm_t a;
    sim_radio_bus_asm_reset(&a);
    /* preamble(4×0x55) + sync(0x6E,0x4E,0x90,0x4E) + PHR(1-byte, len=8) +
     * 8 payload + 2 CRC.  sub-GHz length counts payload only; the chip
     * appends 2 CRC, so completion waits for payload+2. */
    uint8_t frame[] = {
        0x55,0x55,0x55,0x55, 0x6E,0x4E,0x90,0x4E, 8,
        1,2,3,4,5,6,7,8, 0xC1,0xC2
    };
    int done = feed_seq(&a, frame, (int)sizeof(frame));
    ASSERT_EQ(done, 1, "802.15.4g frame completes exactly once");
    ASSERT_EQ(a.subghz, 1, "802.15.4g frame flagged sub-GHz");
    ASSERT_EQ(a.subghz_phr_len, 1, "802.15.4g sniffed 1-byte PHR");
    ASSERT_EQ(a.expected_len, 8, "802.15.4g expected_len sticky");

    /* Does not complete before the 2 trailing CRC bytes. */
    sim_radio_bus_asm_reset(&a);
    int n = (int)sizeof(frame);
    for (int i = 0; i < n - 1; i++)
        ASSERT(!sim_radio_bus_asm_feed(&a, frame[i]), "g: no completion before CRC2");
    ASSERT(sim_radio_bus_asm_feed(&a, frame[n - 1]), "g: completes on 2nd CRC byte");
}

static void test_asm_invalid_length_resets(void) {
    tx_frame_asm_t a;
    sim_radio_bus_asm_reset(&a);
    /* len=0 is invalid (<3) → assembler resets, then a valid frame on
     * the same stream still parses. */
    uint8_t bad[] = { 0,0,0,0, 0x7A, 0 };
    feed_seq(&a, bad, (int)sizeof(bad));
    ASSERT_EQ(a.state, TX_ASM_PREAMBLE, "invalid length returns to PREAMBLE");

    uint8_t good[] = { 0,0,0,0, 0x7A, 5, 1,2,3,4,5 };
    int done = feed_seq(&a, good, (int)sizeof(good));
    ASSERT_EQ(done, 1, "valid frame parses after an invalid-length reset");
}

static void test_asm_reset_clears(void) {
    tx_frame_asm_t a;
    /* Garbage in, then reset clears every field. */
    memset(&a, 0xAB, sizeof(a));
    sim_radio_bus_asm_reset(&a);
    ASSERT_EQ(a.state, TX_ASM_PREAMBLE, "reset state");
    ASSERT_EQ(a.zero_count, 0, "reset zero_count");
    ASSERT_EQ(a.sync_match, 0, "reset sync_match");
    ASSERT_EQ(a.subghz, 0, "reset subghz");
    ASSERT_EQ(a.first_byte_ns, 0, "reset first_byte_ns");
}

/* ============================================================
 * Pure helpers.
 * ============================================================ */

static void test_frame_fifo_bytes(void) {
    uint8_t f24[] = { 0,0,0,0, 0x7A, 20, 0,0,0 };
    ASSERT_EQ(frame_fifo_bytes(f24, (int)sizeof(f24), false), 21,
              "2.4GHz fifo = len+1");
    ASSERT_EQ(frame_fifo_bytes(f24, 5, false), 9999,
              "2.4GHz too-short buffer → sentinel");

    uint8_t fg[] = { 0x55,0x55,0x55,0x55, 0x6E,0x4E,0x90,0x4E, 30, 0,0 };
    ASSERT_EQ(frame_fifo_bytes(fg, (int)sizeof(fg), true), 33,
              "sub-GHz fifo = phr+3");
    ASSERT_EQ(frame_fifo_bytes(fg, 8, true), 9999,
              "sub-GHz too-short buffer → sentinel");
}

static void test_byte_period(void) {
    ASSERT_EQ(byte_period_ns(false), 32000, "2.4GHz byte period 32µs");
    ASSERT_EQ(byte_period_ns(true), 160000, "sub-GHz byte period 160µs");
}

static void test_pick_receiver_radio(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 4);

    /* Unregistered sender slot → legacy "target receiver slot 0". */
    ASSERT_EQ(sim_radio_bus_pick_receiver_radio(&rm, 0, 0, 1), 0,
              "unregistered sender → slot 0");

    /* Registered sender 2.4GHz, receiver 2.4GHz on slot 0 → slot 0. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    ASSERT_EQ(sim_radio_bus_pick_receiver_radio(&rm, 0, 0, 1), 0,
              "matching 2.4GHz → receiver slot 0");

    /* Receiver with sub-GHz on slot 0 and 2.4GHz on slot 1 → picks slot 1. */
    radio_medium_register_radio(&rm, 2, 0, RADIO_SPECTRUM_868MHZ_15_4G);
    radio_medium_register_radio(&rm, 2, 1, RADIO_SPECTRUM_2_4GHZ_15_4);
    ASSERT_EQ(sim_radio_bus_pick_receiver_radio(&rm, 0, 0, 2), 1,
              "2.4GHz sender → receiver's 2.4GHz slot 1");

    /* Receiver registered sub-GHz only (slot 0), no 2.4GHz → drop (-1). */
    radio_medium_register_radio(&rm, 3, 0, RADIO_SPECTRUM_868MHZ_15_4G);
    ASSERT_EQ(sim_radio_bus_pick_receiver_radio(&rm, 0, 0, 3), -1,
              "no matching spectrum on registered receiver → drop");
}

/* ============================================================
 * Delivery-mode routing + byte clock + capture.
 * ============================================================ */

/* Build a minimal valid 802.15.4 frame into buf, return length. */
static int build_802154(uint8_t *buf, int payload_len) {
    int n = 0;
    buf[n++] = 0; buf[n++] = 0; buf[n++] = 0; buf[n++] = 0;
    buf[n++] = 0x7A;
    buf[n++] = (uint8_t)payload_len;
    for (int i = 0; i < payload_len; i++) buf[n++] = (uint8_t)(0x10 + i);
    return n;
}

static void test_delivery_sync(void) {
    /* SYNC receiver gets bytes synchronously, during tx_byte. */
    fixture_t f;
    fx_init(&f, 2);
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1], SIM_RADIO_DELIVERY_SYNC, 0);

    uint8_t frame[64];
    int n = build_802154(frame, 6);
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, frame[i]);

    ASSERT_EQ(f.rx[1].recv_count, n, "SYNC receiver got all bytes synchronously");
    ASSERT(sim_eq_empty(&f.sim.event_queue), "SYNC schedules no RX_BYTE events");
}

static void test_delivery_per_byte(void) {
    /* PER_BYTE receiver gets nothing until the queue is pumped; events
     * land at 32µs byte-clock spacing from first_byte_ns. */
    fixture_t f;
    fx_init(&f, 2);
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1], SIM_RADIO_DELIVERY_PER_BYTE, 0);

    f.sim.now_ns = 0;
    uint8_t frame[64];
    int n = build_802154(frame, 6);
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, frame[i]);

    ASSERT_EQ(f.rx[1].recv_count, 0, "PER_BYTE: nothing delivered before pump");
    ASSERT(!sim_eq_empty(&f.sim.event_queue), "PER_BYTE scheduled RX_BYTE events");

    fx_pump(&f, 100 * 1000000LL);
    ASSERT_EQ(f.rx[1].recv_count, n, "PER_BYTE: all bytes after pump");

    bool spacing_ok = true;
    for (int i = 1; i < f.rx[1].recv_count; i++)
        if (f.rx[1].recv_time[i] - f.rx[1].recv_time[i - 1] != IEEE802154_BYTE_NS)
            spacing_ok = false;
    ASSERT(spacing_ok, "PER_BYTE: 32µs byte-clock spacing");
    ASSERT_EQ(f.rx[1].recv_time[0], 0, "PER_BYTE: first byte at first_byte_ns");
}

static void test_delivery_batch(void) {
    /* BATCH receiver: bytes accumulate in rf_pending during dispatch,
     * then frame_complete (M27) snapshots and delivers the whole frame
     * to the chip — rf_pending ends empty. */
    fixture_t f;
    fx_init(&f, 2);
    mock_mote_t mm;
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    /* node 1 = BATCH receiver with a registered sim_mote (the snapshot
     * delivery drives its rx_byte_sync/step ops). */
    memset(&mm, 0, sizeof(mm));
    mm.freq = 1000000; mm.mote.impl = &mm; mm.mote.ops = &mm_ops;
    sim_runtime_register_mote(&f.sim, 1, &mm.mote);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1], SIM_RADIO_DELIVERY_BATCH, 0);
    f.bus.executing_node = -1;

    uint8_t frame[64];
    int n = build_802154(frame, 6);   /* expected_len 6 → total_air_bytes == n */
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, frame[i]);

    ASSERT_EQ(f.bus.rf_pending[1].count, 0,
              "BATCH: staged frame consumed at frame-complete");
    ASSERT_EQ(f.rx[1].recv_count, n, "BATCH: whole frame delivered to the chip");
    ASSERT_EQ(f.bus.stats.rx_direct, 1, "BATCH: counted as a direct delivery");
}

static void test_capture_and_first_byte(void) {
    fixture_t f;
    fx_init(&f, 2);
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1], SIM_RADIO_DELIVERY_PER_BYTE, 0);

    f.sim.now_ns = 5000;  /* first preamble byte arms first_byte_ns = now */
    uint8_t frame[64];
    int n = build_802154(frame, 4);
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, frame[i]);

    ASSERT_EQ(f.bus.tx_cap[0].len, n, "capture length = frame length");
    ASSERT_EQ(f.bus.tx_asm[0].first_byte_ns, 5000, "first_byte_ns = now at preamble");
    bool bytes_ok = (memcmp(f.bus.tx_cap[0].bytes, frame, (size_t)n) == 0);
    ASSERT(bytes_ok, "capture bytes match the on-air frame");
}

/* ============================================================
 * Re-entrant depth staging (auto-ACK during synchronous delivery).
 * ============================================================ */

static int reentry_frames_completed;
/* M27: the frame_complete host hook became the bus-internal
 * sim_radio_bus_frame_complete; the re-entrancy test now counts frames
 * via the frame_observed notification (fired once per completed frame). */
static void reentry_frame_observed(void *user, const sim_radio_frame_info_t *fi) {
    (void)user; (void)fi;
    reentry_frames_completed++;
}

static void test_reentrant_depth(void) {
    /* A SYNC receiver re-enters tx_byte (emits an "ACK" byte) the moment
     * it is fed a trigger byte.  The re-entrant byte must be dispatched
     * to other receivers but never fed to the sender's assembler — i.e.
     * it must not advance / corrupt the in-flight frame parse, and the
     * outer frame still completes exactly once. */
    fixture_t f;
    fx_init(&f, 3);
    reentry_frames_completed = 0;
    sim_radio_bus_host_t host = { 0 };
    host.frame_observed = reentry_frame_observed;
    sim_radio_bus_set_host(&f.bus, &host);

    /* node 0 = the sender (PER_BYTE), node 1 = a SYNC receiver that emits
     * an ACK byte the moment it is fed the SFD, node 2 = a PER_BYTE
     * bystander that should also receive the re-entrant ACK byte. */
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1], SIM_RADIO_DELIVERY_SYNC, 0);
    sim_radio_bus_register(&f.bus, 2, &mock_ops, &f.rx[2], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    f.bus.executing_node = -1;
    f.rx[1].reenter_on = 0x7A;
    f.rx[1].reenter_sender = 1;

    uint8_t frame[64];
    int n = build_802154(frame, 5);
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, frame[i]);

    /* The outer frame completes exactly once: the re-entrant ACK byte
     * (0xAC) is dispatched to receivers but never fed to a sender's
     * assembler in a way that closes a second frame. */
    ASSERT_EQ(reentry_frames_completed, 1,
              "outer frame completes exactly once despite re-entrant ACK byte");

    /* The re-entrant 0xAC was dispatched as sender 1 → scheduled to the
     * PER_BYTE bystander; pump and confirm it arrived. */
    fx_pump(&f, 100 * 1000000LL);
    bool saw_ack = false;
    for (int i = 0; i < f.rx[2].recv_count; i++)
        if (f.rx[2].recv_byte[i] == 0xAC) saw_ack = true;
    ASSERT(saw_ack, "re-entrant ACK byte dispatched to other receivers");
}

/* ============================================================
 * RX-stall timer (M9.5).
 * ============================================================ */

static void test_rx_stall(void) {
    /* fixture_t embeds sim_runtime_t + sim_radio_bus_t (~5 MB each); this is the
     * only test that needs two live at once, and two on the stack overflow an
     * 8 MB stack (segfaults on macOS and smaller ulimits; survived the Linux CI
     * runner by luck). Keep them in static storage — fx_init re-inits each. */
    static fixture_t f;
    fx_init(&f, 2);
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    /* Receiver with an rx_stall op → the bus arms the watchdog. */
    sim_radio_bus_register(&f.bus, 1, &mock_ops_stall, &f.rx[1], SIM_RADIO_DELIVERY_PER_BYTE, 0);

    f.sim.now_ns = 0;
    /* Deliver three bytes; last air time = 2 * 32µs = 64000. */
    for (int b = 0; b < 3; b++)
        sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, (uint8_t)(0xA0 + b));

    ASSERT(f.bus.rx_stall_pending[1], "rx_stall armed after PER_BYTE delivery");
    int64_t last_byte = 2 * IEEE802154_BYTE_NS;
    ASSERT_EQ(f.bus.rx_stall_deadline_ns[1], last_byte + SIM_RADIO_RX_STALL_NS,
              "deadline = last byte air time + 200µs");

    /* A timer that fires BEFORE the deadline is stale → re-arm, no stall. */
    bool expired_early =
        sim_radio_bus_rx_stall_expired(&f.bus, &f.sim, 1, last_byte);
    ASSERT(!expired_early, "early timer is stale (deadline extended)");
    ASSERT(f.bus.rx_stall_pending[1], "stale timer re-armed");

    /* A timer at/after the deadline truly expires. */
    bool expired =
        sim_radio_bus_rx_stall_expired(&f.bus, &f.sim, 1,
                                       last_byte + SIM_RADIO_RX_STALL_NS);
    ASSERT(expired, "timer at deadline truly expires");

    /* A receiver WITHOUT an rx_stall op never arms the watchdog. */
    static fixture_t g;          /* static: see the note on `f` above */
    fx_init(&g, 2);
    sim_radio_bus_register(&g.bus, 0, &mock_ops, &g.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&g.bus, 1, &mock_ops, &g.rx[1], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_tx_byte(&g.bus, &g.sim, 0, 0, 0x11);
    ASSERT(!g.bus.rx_stall_pending[1], "no rx_stall op → watchdog never armed");
}

/* ============================================================
 * reset_node clears per-node state.
 * ============================================================ */

static void test_reset_node(void) {
    fixture_t f;
    fx_init(&f, 2);
    sim_radio_bus_register(&f.bus, 0, &mock_ops, &f.rx[0], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_register(&f.bus, 1, &mock_ops_stall, &f.rx[1], SIM_RADIO_DELIVERY_PER_BYTE, 0);
    sim_radio_bus_tx_byte(&f.bus, &f.sim, 0, 0, 0x55);
    ASSERT(f.bus.rx_stall_pending[1], "armed before reset");
    sim_radio_bus_reset_node(&f.bus, 1);
    ASSERT(!f.bus.rx_stall_pending[1], "reset clears rx_stall_pending");
    ASSERT_EQ(f.bus.rx_stall_deadline_ns[1], 0, "reset clears rx_stall_deadline");
}

/* ============================================================
 * Emulated-RX delivery / queue / drain (M26).
 * ============================================================ */

static void test_deliver_direct(void) {
    /* Synchronous delivery: each byte is pre-synced to its air time and
     * handed to the chip; a wakeup is scheduled after the burst. */
    fixture_t f; fx_init(&f, 2);
    mock_mote_t mm;
    register_emulated(&f, &mm, 1, 0);
    f.bus.executing_node = -1;
    f.sim.now_ns = 0;

    uint8_t frame[64]; int n = short_frame(frame, 5);
    sim_radio_bus_deliver_bytes(&f.bus, &f.sim, 1, frame, n, -40, 0, false);

    ASSERT_EQ(f.rx[1].recv_count, n, "direct: all bytes reached the chip");
    ASSERT_EQ(mm.sync_count, n, "direct: one clock-sync per byte");
    bool spaced = true;
    for (int i = 1; i < mm.sync_count; i++)
        if (mm.sync_times[i] - mm.sync_times[i-1] != IEEE802154_BYTE_NS) spaced = false;
    ASSERT(spaced, "direct: per-byte sync on the 32µs air clock");
    ASSERT(!sim_eq_empty(&f.sim.event_queue), "direct: wakeup scheduled");
}

static void test_deliver_executing_node(void) {
    /* When the receiver is the executing node, skip per-byte sync; with
     * CAP_RX_TICKING_STEP the bus steps the CPU ~1 ms instead. */
    fixture_t f; fx_init(&f, 2);
    mock_mote_t mm;
    register_emulated(&f, &mm, 1, SIM_RADIO_CAP_RX_TICKING_STEP);
    f.bus.executing_node = 1;

    uint8_t frame[64]; int n = short_frame(frame, 4);
    sim_radio_bus_deliver_bytes(&f.bus, &f.sim, 1, frame, n, -40, 0, false);

    ASSERT_EQ(f.rx[1].recv_count, n, "executing: bytes still delivered");
    ASSERT_EQ(mm.sync_count, 0, "executing: no per-byte clock sync");
    ASSERT_EQ(mm.step_count, 1, "executing: CAP_RX_TICKING_STEP stepped once");
    ASSERT_EQ(mm.cyc, 1000, "executing: stepped ~1 ms (1000 cycles @ 1 MHz)");
}

static void test_queue_and_drain(void) {
    /* queue_frame stages frames FIFO; drain delivers one per call. */
    fixture_t f; fx_init(&f, 2);
    mock_mote_t mm;
    register_emulated(&f, &mm, 1, 0);
    f.bus.executing_node = -1;

    uint8_t a[64], b[64];
    int na = short_frame(a, 3), nb = short_frame(b, 4);
    a[6] = 0xA1; b[6] = 0xB1;   /* distinguishable first payload byte */
    sim_radio_bus_queue_frame(&f.bus, 1, a, na, -40, 0, 1000, false);
    sim_radio_bus_queue_frame(&f.bus, 1, b, nb, -40, 0, 1000, false);
    ASSERT_EQ(f.bus.emu_rx_queue[1].count, 2, "queue: two frames staged");
    ASSERT_EQ(f.bus.stats.rx_queued, 2, "queue: rx_queued counted");

    sim_radio_bus_drain_rx(&f.bus, &f.sim, 1);
    ASSERT_EQ(f.bus.emu_rx_queue[1].count, 1, "drain: one frame consumed");
    ASSERT_EQ(f.bus.stats.rx_drained, 1, "drain: rx_drained counted");
    ASSERT_EQ(f.rx[1].recv_byte[6], 0xA1, "drain: FIFO order (frame A first)");

    sim_radio_bus_drain_rx(&f.bus, &f.sim, 1);
    ASSERT_EQ(f.bus.emu_rx_queue[1].count, 0, "drain: queue empty");
    ASSERT_EQ(f.rx[1].recv_byte[na + 6], 0xB1, "drain: frame B second");
}

static void test_drain_collided_skip(void) {
    /* A collided frame is dropped without delivery; the next good frame
     * is delivered. */
    fixture_t f; fx_init(&f, 2);
    mock_mote_t mm;
    register_emulated(&f, &mm, 1, 0);
    f.bus.executing_node = -1;

    uint8_t a[64], b[64];
    int na = short_frame(a, 3), nb = short_frame(b, 3);
    b[6] = 0xB2;
    sim_radio_bus_queue_frame(&f.bus, 1, a, na, -40, 0, 1000, false);
    sim_radio_bus_queue_frame(&f.bus, 1, b, nb, -40, 0, 1000, false);
    /* Mark the head (frame A) collided. */
    f.bus.emu_rx_queue[1].frames[f.bus.emu_rx_queue[1].head].collided = true;

    sim_radio_bus_drain_rx(&f.bus, &f.sim, 1);
    ASSERT_EQ(f.bus.emu_rx_queue[1].count, 0, "drain: collided skipped + good drained");
    ASSERT_EQ(f.rx[1].recv_count, nb, "drain: only the good frame's bytes");
    ASSERT_EQ(f.rx[1].recv_byte[6], 0xB2, "drain: delivered frame B, not collided A");
}

static void test_drain_max_arrival(void) {
    /* A frame queued with a future arrival_ns is delivered at that time
     * (max(arrival, now)) so the ACK turnaround is preserved. */
    fixture_t f; fx_init(&f, 2);
    mock_mote_t mm;
    register_emulated(&f, &mm, 1, 0);
    f.bus.executing_node = -1;
    f.sim.now_ns = 1000;

    uint8_t a[64]; int na = short_frame(a, 2);
    int64_t future = 500000;   /* well past now */
    sim_radio_bus_queue_frame(&f.bus, 1, a, na, -40, future, 1000, false);
    sim_radio_bus_drain_rx(&f.bus, &f.sim, 1);
    ASSERT_EQ(mm.sync_times[0], future, "drain: delivered at max(arrival, now)");
}

static void test_deliver_native_noop(void) {
    /* A receiver whose sim_mote has no rx_byte_sync (native/JS) gets no
     * byte delivery — only the on_rx timeline notification. */
    fixture_t f; fx_init(&f, 2);
    sim_mote_t bare = {0};
    static const sim_mote_ops_t bare_ops = { .kind = "X" };  /* rx_byte_sync NULL */
    bare.ops = &bare_ops;
    sim_runtime_register_mote(&f.sim, 1, &bare);
    sim_radio_bus_register(&f.bus, 1, &mock_ops, &f.rx[1],
                           SIM_RADIO_DELIVERY_SYNC, 0);

    uint8_t frame[64]; int n = short_frame(frame, 4);
    sim_radio_bus_deliver_bytes(&f.bus, &f.sim, 1, frame, n, -40, 0, false);
    ASSERT_EQ(f.rx[1].recv_count, 0, "native: no byte delivery (rx_byte_sync NULL)");
}

/* ============================================================
 * Frame-complete delivery policy (M27): collision windows, the dual
 * 192 µs auto-ACK windows, and RXFIFO backpressure — driven end to end
 * through sim_radio_bus_tx_byte (which invokes the bus-internal
 * sim_radio_bus_frame_complete) and observed via the frame_observed /
 * on_rx_frame / on_ack hooks.
 * ============================================================ */

static struct {
    int     frame_observed;
    int     outcome[8];       /* last on_rx_frame outcome per receiver */
    int     ack_count;
    int     ack_receiver;
    int64_t ack_start;
} frec;

static void frec_observed(void *u, const sim_radio_frame_info_t *fi) {
    (void)u; (void)fi; frec.frame_observed++;
}
static void frec_rx(void *u, const sim_radio_frame_info_t *fi, int receiver,
                    int outcome, const uint8_t *data, int len,
                    int64_t coll_start, int64_t prev_end) {
    (void)u; (void)fi; (void)data; (void)len; (void)coll_start; (void)prev_end;
    if (receiver >= 0 && receiver < 8) frec.outcome[receiver] = outcome;
}
static void frec_ack(void *u, const sim_radio_frame_info_t *fi, int data_recv,
                     int64_t ack_start, int ack_bytes) {
    (void)u; (void)fi; (void)ack_bytes;
    frec.ack_count++;
    frec.ack_receiver = data_recv;
    frec.ack_start = ack_start;
}
static void frec_set_host(fixture_t *f) {
    memset(&frec, 0, sizeof(frec));
    for (int i = 0; i < 8; i++) frec.outcome[i] = -1;
    static sim_radio_bus_host_t host;
    memset(&host, 0, sizeof(host));
    host.frame_observed = frec_observed;
    host.on_rx_frame = frec_rx;
    host.on_ack = frec_ack;
    sim_radio_bus_set_host(&f->bus, &host);
}

/* Register node idx as a BATCH emulated receiver (snapshot-delivered by
 * frame_complete) with a sim_mote backing the CPU ops. */
static void reg_batch(fixture_t *f, mock_mote_t *mm, int idx, uint32_t caps) {
    memset(mm, 0, sizeof(*mm));
    mm->freq = 1000000;
    mm->mote.impl = mm;
    mm->mote.ops = &mm_ops;
    sim_runtime_register_mote(&f->sim, idx, &mm->mote);
    sim_radio_bus_register(&f->bus, idx, &mock_ops, &f->rx[idx],
                           SIM_RADIO_DELIVERY_BATCH, caps);
}

static void tx_frame(fixture_t *f, int sender, const uint8_t *frame, int n) {
    for (int i = 0; i < n; i++)
        sim_radio_bus_tx_byte(&f->bus, &f->sim, sender, 0, frame[i]);
}

static void test_frame_collision_window(void) {
    /* A frame whose air window opens before the receiver's previous RX
     * ended (emu_rx_end_ns) is dropped as a collision — no delivery. */
    fixture_t f; fx_init(&f, 2);
    frec_set_host(&f);
    mock_mote_t mm; reg_batch(&f, &mm, 1, 0);
    f.bus.executing_node = -1;
    f.sim.now_ns = 1000;                  /* frame air-start == now (M27) */
    f.bus.emu_rx_end_ns[1] = 5000;        /* receiver busy until 5 µs     */

    uint8_t frame[64]; int n = build_802154(frame, 5);
    tx_frame(&f, 0, frame, n);

    ASSERT_EQ(frec.frame_observed, 1, "collision: frame observed once");
    ASSERT_EQ(frec.outcome[1], SIM_RADIO_RX_COLLIDED, "collision: receiver outcome COLLIDED");
    ASSERT_EQ(f.bus.stats.rx_collided, 1, "collision: rx_collided counted");
    ASSERT_EQ(f.rx[1].recv_count, 0, "collision: no bytes delivered");
    ASSERT_EQ(f.bus.stats.rx_direct, 0, "collision: not delivered directly");
}

static void test_frame_no_collision_when_clear(void) {
    /* Same frame with a clear channel delivers directly. */
    fixture_t f; fx_init(&f, 2);
    frec_set_host(&f);
    mock_mote_t mm; reg_batch(&f, &mm, 1, 0);
    f.bus.executing_node = -1;
    f.sim.now_ns = 1000;
    f.bus.emu_rx_end_ns[1] = 0;           /* channel clear */

    uint8_t frame[64]; int n = build_802154(frame, 5);
    tx_frame(&f, 0, frame, n);

    ASSERT_EQ(frec.outcome[1], SIM_RADIO_RX_DIRECT, "clear: outcome DIRECT");
    ASSERT_EQ(f.bus.stats.rx_direct, 1, "clear: delivered directly");
    ASSERT_EQ(f.rx[1].recv_count, n, "clear: all bytes delivered");
}

static void test_frame_backpressure_queue(void) {
    /* RXFIFO too small: frame_complete mini-steps the receiver once, and
     * (still full) queues the frame instead of delivering. */
    fixture_t f; fx_init(&f, 2);
    frec_set_host(&f);
    mock_mote_t mm; reg_batch(&f, &mm, 1, 0);
    f.bus.executing_node = -1;
    f.rx[1].rxfifo = 2;                   /* < frame_fifo_bytes (len+1) */

    uint8_t frame[64]; int n = build_802154(frame, 5);
    tx_frame(&f, 0, frame, n);

    ASSERT_EQ(mm.step_count, 1, "backpressure: mini-stepped once to drain RXFIFO");
    ASSERT_EQ(frec.outcome[1], SIM_RADIO_RX_QUEUED, "backpressure: outcome QUEUED");
    ASSERT_EQ(f.bus.stats.rx_queued, 1, "backpressure: rx_queued counted");
    ASSERT_EQ(f.bus.stats.rx_direct, 0, "backpressure: not delivered directly");
    ASSERT_EQ(f.bus.emu_rx_queue[1].count, 1, "backpressure: frame deferred to RX queue");
}

static void test_frame_ack_window_sender(void) {
    /* Scripted auto-ACK: the receiver emits an ACK byte (re-entering the
     * bus as its own sender) the moment it is fed the SFD during
     * synchronous delivery.  The ACK is staged in the data sender's
     * rf_pending and, at the ACK flush, queued for the sender at
     * now + 192 µs (the sender turnaround window — not tx_end + 192 µs). */
    fixture_t f; fx_init(&f, 2);
    frec_set_host(&f);
    mock_mote_t mm0, mm1;
    reg_batch(&f, &mm0, 0, 0);   /* data sender — also receives the ACK */
    reg_batch(&f, &mm1, 1, 0);   /* data receiver — auto-ACKs           */
    f.bus.executing_node = -1;
    f.sim.now_ns = 0;
    f.rx[1].reenter_on = 0x7A;   /* on the SFD, emit one ACK byte... */
    f.rx[1].reenter_sender = 1;  /* ...as a transmission from node 1 */

    uint8_t frame[64]; int n = build_802154(frame, 5);
    tx_frame(&f, 0, frame, n);

    ASSERT_EQ(frec.ack_count, 1, "ack: one ACK flush notification");
    ASSERT_EQ(frec.ack_receiver, 1, "ack: keyed on the data receiver");
    /* Sender's ACK is queued (not delivered) for the sender turnaround. */
    ASSERT_EQ(f.bus.emu_rx_queue[0].count, 1, "ack: sender ACK deferred to its RX queue");
    ASSERT_EQ(f.bus.emu_rx_queue[0].frames[f.bus.emu_rx_queue[0].head].arrival_ns,
              192000, "ack: sender ACK at now + 192 µs turnaround");
}

/* ============================================================
 * Entry point.
 * ============================================================ */

int run_radio_bus_tests(int verbose);
int run_radio_bus_tests(int verbose) {
    (void)verbose;
    passed = failed = 0;
    printf("=== sim_radio_bus unit tests (Phase 5 guardrail) ===\n");

    test_asm_802154();
    test_asm_802154g();
    test_asm_invalid_length_resets();
    test_asm_reset_clears();
    test_frame_fifo_bytes();
    test_byte_period();
    test_pick_receiver_radio();
    test_delivery_sync();
    test_delivery_per_byte();
    test_delivery_batch();
    test_capture_and_first_byte();
    test_reentrant_depth();
    test_rx_stall();
    test_reset_node();
    test_deliver_direct();
    test_deliver_executing_node();
    test_queue_and_drain();
    test_drain_collided_skip();
    test_drain_max_arrival();
    test_deliver_native_noop();
    test_frame_collision_window();
    test_frame_no_collision_when_clear();
    test_frame_backpressure_queue();
    test_frame_ack_window_sender();

    printf("--- Results: %d passed, %d failed ---\n", passed, failed);
    return failed;
}
