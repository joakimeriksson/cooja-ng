/*
 * radio_medium_t unit tests
 *
 * Pure C, no CPU emulator, no chip drivers — exercise
 * src/common/radio_medium.c against the public API in
 * include/common/radio_medium.h. This is the safety net for the
 * upcoming per-node-per-radio refactor: every behavior pinned here
 * MUST still hold afterwards (or be updated in lockstep with the API
 * change).
 *
 * Mirror the test_cc1200.c style: void test functions, ASSERT/ASSERT_EQ
 * macros, single fixture builder, run_radio_medium_tests dispatch at
 * the bottom.
 */
#include "radio_medium.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

#define ASSERT_EQ(actual, expected, msg) do {                            \
    if ((actual) == (expected)) { passed++; }                            \
    else { failed++;                                                     \
        printf("  FAIL: %s — got %d, want %d (%s:%d)\n",                 \
               msg, (int)(actual), (int)(expected), __FILE__, __LINE__); } \
} while (0)

/* ====================================================================
 * Initialization & defaults
 * ==================================================================== */

/* radio_medium_init produces a NONE-type medium that passes all bytes
 * and frames between any two nodes — backward-compatible with the
 * pre-medium "all-to-all delivery" behavior. */
static void test_init_defaults_pass_all(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 4);

    ASSERT_EQ(rm.type, RADIO_MEDIUM_NONE, "default type = NONE");
    ASSERT_EQ(rm.node_count, 4, "node_count stored");

    /* All nodes default to position (0,0) and channel -1. */
    for (int i = 0; i < 4; i++) {
        ASSERT(rm.nodes[i].x == 0.0, "default x = 0");
        ASSERT(rm.nodes[i].y == 0.0, "default y = 0");
        ASSERT_EQ(rm.nodes[i].channel, -1, "default channel = -1");
    }

    /* Even with arbitrary channel mismatch and arbitrary positions, a
     * NONE medium passes EVERYTHING. */
    radio_medium_set_position(&rm, 0,    0.0,    0.0);
    radio_medium_set_position(&rm, 1, 1000.0, 1000.0);  /* far away */
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, 26);
    for (int b = 0; b < 16; b++) {
        ASSERT(radio_medium_filter_byte(&rm, 0, 1, (uint8_t)b),
               "NONE: byte passes regardless");
    }
    /* And frame-level too. */
    for (int i = 0; i < 8; i++) {
        ASSERT(radio_medium_filter_frame(&rm, 0, 1), "NONE: frame passes");
    }

    /* Cross-band channel pair (sub-GHz vs 2.4 GHz) ALSO passes when
     * type is NONE — the cross-band gate only applies to UDGM. */
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, 100);
    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "NONE: cross-band frame still passes");
}

/* radio_medium_configure_udgm flips type to UDGM and stores params. */
static void test_configure_udgm_activates_filter(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    ASSERT_EQ(rm.type, RADIO_MEDIUM_UDGM, "type = UDGM after configure");
    ASSERT(rm.udgm.tx_range == 50.0, "tx_range stored");
    ASSERT(rm.udgm.interference_range == 100.0, "interference_range stored");
    ASSERT(rm.udgm.success_ratio_tx == 1.0, "success_ratio_tx stored");
    ASSERT(rm.udgm.success_ratio_rx == 1.0, "success_ratio_rx stored");

    /* Filtering is now active: with positions still both at (0,0) and
     * channels still -1, bytes pass (distance 0 < 50). */
    ASSERT(radio_medium_filter_byte(&rm, 0, 1, 0xAB),
           "UDGM, dist 0, ch -1/-1: byte passes");
}

/* radio_medium_set_seed reproduces drop pattern across runs. */
static void test_seed_reproducibility(void) {
    radio_medium_t rm1, rm2;
    radio_medium_init(&rm1, 2);
    radio_medium_init(&rm2, 2);
    radio_medium_configure_udgm(&rm1, 50.0, 100.0, 0.5, 1.0);
    radio_medium_configure_udgm(&rm2, 50.0, 100.0, 0.5, 1.0);

    /* Distance 0, channel match — only the per-frame TX dice roll
     * differs run-to-run. Same seed -> same sequence. */
    radio_medium_set_seed(&rm1, 0xDEADBEEF);
    radio_medium_set_seed(&rm2, 0xDEADBEEF);

    int matches = 0;
    for (int i = 0; i < 100; i++) {
        bool a = radio_medium_filter_frame(&rm1, 0, 1);
        bool b = radio_medium_filter_frame(&rm2, 0, 1);
        if (a == b) matches++;
    }
    ASSERT_EQ(matches, 100, "same seed -> identical 100-frame drop pattern");

    /* And a different seed produces SOME differing decisions
     * (probabilistic — but with 100 rolls @ 0.5, agreement-by-chance is
     * vanishingly improbable for two independent xorshift sequences). */
    radio_medium_set_seed(&rm2, 0x12345678);
    int diffs = 0;
    for (int i = 0; i < 100; i++) {
        if (radio_medium_filter_frame(&rm1, 0, 1) !=
            radio_medium_filter_frame(&rm2, 0, 1)) diffs++;
    }
    ASSERT(diffs > 10, "different seeds diverge quickly");
}

/* set_seed(0) should not freeze the PRNG: implementation falls back to
 * the default seed when a zero seed is passed. */
static void test_set_seed_zero_falls_back(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 0.5, 1.0);
    radio_medium_set_seed(&rm, 0);

    /* xorshift on rng_state==0 stays 0 forever and rng_next would
     * always return 0 -> all frames would pass (roll < prob). The
     * fallback prevents that pathology by reseeding to 0x12345678. */
    int passes = 0, drops = 0;
    for (int i = 0; i < 200; i++) {
        if (radio_medium_filter_frame(&rm, 0, 1)) passes++; else drops++;
    }
    ASSERT(passes > 0 && drops > 0,
           "set_seed(0) reseeds to default — sees both pass and drop");
}

/* ====================================================================
 * Channel matching (current "within-band check disabled" behavior)
 * ==================================================================== */

/* Two nodes on the same 2.4 GHz channel, distance 0, ratios 1.0
 * -> all bytes pass. */
static void test_channel_match_passes(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, 11);

    /* Drive a complete IEEE 802.15.4 frame: 4×0x00 preamble, 0x7A SFD,
     * length=3, payload=3 bytes. All bytes must pass. */
    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x03, 0xAA, 0xBB, 0xCC };
    for (size_t i = 0; i < sizeof(frame); i++) {
        ASSERT(radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "channel match: byte passes");
    }
}

/* Two nodes on different 2.4 GHz channels (11 vs 26): the medium MUST
 * drop because chip drivers now push their channel into the medium
 * synchronously on register write. This is the post-refactor behavior;
 * the pre-refactor pin was "passes" with a TSCH-hopping caveat. */
static void test_within_band_channel_mismatch_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, 26);

    /* Both 2.4 GHz, channel mismatch -> dropped at the channel gate. */
    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "within-band channel mismatch (11 vs 26): frame dropped");
}

/* One node on a known channel, the other on -1 (unknown). The cross-band
 * gate explicitly returns false when EITHER channel is < 0, so frames
 * pass. Pin this. */
static void test_channel_unknown_passes(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, 11);
    /* node 1 stays at default channel -1 */

    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "ch=11 vs ch=-1: cross-band gate disabled, frame passes");
    ASSERT(radio_medium_filter_frame(&rm, 1, 0),
           "ch=-1 vs ch=11: same in reverse direction");
}

/* ====================================================================
 * Cross-band isolation (commit a5007c2)
 * ==================================================================== */

/* Sub-GHz (>= 100) vs 2.4 GHz (< 100) -> ALL frames dropped. */
static void test_cross_band_drops_both_directions(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Place at the same point so the only failing condition is the
     * cross-band channel gate. */
    radio_medium_set_channel(&rm, 0, 100);  /* sub-GHz */
    radio_medium_set_channel(&rm, 1, 11);   /* 2.4 GHz */

    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "sub-GHz -> 2.4 GHz: frame dropped");
    ASSERT(!radio_medium_filter_frame(&rm, 1, 0),
           "2.4 GHz -> sub-GHz: frame dropped");

    /* And at the byte level. The frame tracker still runs on the
     * sender side (advancing its profile-specific state machine), but
     * the cross-band gate fires before the rx-decision is consulted.
     * Use a byte that wouldn't accidentally be a sync-word match. */
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0xAA),
           "sub-GHz -> 2.4 GHz: byte dropped");
    ASSERT(!radio_medium_filter_byte(&rm, 1, 0, 0xAA),
           "2.4 GHz -> sub-GHz: byte dropped");
}

/* Two sub-GHz nodes on the same channel-base (100) -> frames pass. */
static void test_subghz_same_channel_passes(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, 100);
    radio_medium_set_channel(&rm, 1, 100);

    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "sub-GHz pair on ch 100: frame passes");
}

/* Two sub-GHz nodes on different sub-GHz channels (100 vs 132) -> frames
 * are now dropped. Same band, mismatched channel — channel gate fires. */
static void test_subghz_different_channels_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, 100);
    radio_medium_set_channel(&rm, 1, 132);

    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "sub-GHz cross-channel (100 vs 132): frame dropped");
}

/* ====================================================================
 * UDGM distance filter
 * ==================================================================== */

/* Inside tx_range -> bytes pass. */
static void test_distance_inside_tx_range_passes(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_position(&rm, 0,  0.0, 0.0);
    radio_medium_set_position(&rm, 1, 49.0, 0.0);  /* dist 49 < 50 */

    /* No channels set (both -1) — cross-band gate disabled. */
    /* Send a complete frame; every byte should pass. */
    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x02, 0x11, 0x22 };
    for (size_t i = 0; i < sizeof(frame); i++) {
        ASSERT(radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "dist < tx_range: byte passes");
    }
}

/* Just past tx_range — outside-frame bytes (preamble) drop, but the
 * within-frame branch in filter_byte uses udgm_reception_prob which is
 * 0 for dist > tx_range. Net: all bytes drop. */
static void test_distance_outside_tx_range_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_position(&rm, 0,  0.0, 0.0);
    radio_medium_set_position(&rm, 1, 51.0, 0.0);  /* dist 51 > 50 */

    /* All 9 bytes of a complete frame should drop. */
    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x02, 0x11, 0x22 };
    for (size_t i = 0; i < sizeof(frame); i++) {
        ASSERT(!radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "dist > tx_range: byte drops");
    }
    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "dist > tx_range: frame drops");
}

/* Beyond interference_range — current behavior: same as beyond tx_range
 * (no bytes pass), since UDGM filter_byte is keyed on tx_range only.
 * Pin that. */
static void test_distance_beyond_interference_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_position(&rm, 0,   0.0, 0.0);
    radio_medium_set_position(&rm, 1, 200.0, 0.0);  /* dist 200 > 100 */

    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x01, 0xFF };
    for (size_t i = 0; i < sizeof(frame); i++) {
        ASSERT(!radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "dist > interference_range: byte drops");
    }
}

/* Boundary: dist == tx_range exactly. udgm_reception_prob returns 0
 * because dist_sq > range_sq is false but the formula evaluates to
 * srx * stx = 1.0 — so within-frame the byte passes. The OUTSIDE-frame
 * path uses dist_sq < range_sq (strict) so preamble drops. Document
 * this corner with a frame-level filter (which uses the prob path). */
static void test_distance_exact_tx_range_boundary(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_position(&rm, 0,  0.0, 0.0);
    radio_medium_set_position(&rm, 1, 50.0, 0.0);  /* dist == tx_range */

    /* filter_frame uses udgm_reception_prob -> prob = 1.0 -> passes. */
    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "dist == tx_range, ratios 1.0: frame_filter passes");
}

/* radio_medium_compute_neighbors populates neighbor lists from
 * positions. 4-node square with side 30, tx_range=50:
 *
 *   (0,0)─30─(30,0)
 *     │       │
 *    30      30
 *     │       │
 *   (0,30)──(30,30)
 *
 * Each node has 2 axis-neighbors (dist 30) plus 1 diagonal
 * (dist sqrt(1800) ≈ 42.4) — all 3 within tx_range=50. */
static void test_compute_neighbors_4node_square(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 4);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_set_position(&rm, 0,  0.0,  0.0);
    radio_medium_set_position(&rm, 1, 30.0,  0.0);
    radio_medium_set_position(&rm, 2,  0.0, 30.0);
    radio_medium_set_position(&rm, 3, 30.0, 30.0);

    radio_medium_compute_neighbors(&rm);

    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(rm.neighbors[i].count, 3, "square: each node has 3 neighbors");
        ASSERT_EQ(rm.interference_neighbors[i].count, 0,
                  "square: no interference-only neighbors at side 30");
    }

    /* Now widen the square so only axis neighbors land in TX range and
     * diagonals fall into interference_range only. Side 60: axis dist
     * 60 > tx_range 50, so axis neighbors drop into interference;
     * diagonal sqrt(7200) ≈ 84.85 < interference 100. With this geometry
     * NO node has any TX-range neighbor; all three other nodes are
     * interference-only. */
    radio_medium_set_position(&rm, 0,  0.0,  0.0);
    radio_medium_set_position(&rm, 1, 60.0,  0.0);
    radio_medium_set_position(&rm, 2,  0.0, 60.0);
    radio_medium_set_position(&rm, 3, 60.0, 60.0);
    radio_medium_compute_neighbors(&rm);

    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(rm.neighbors[i].count, 0,
                  "square side 60: no TX-range neighbors");
        ASSERT_EQ(rm.interference_neighbors[i].count, 3,
                  "square side 60: 3 interference-only neighbors");
    }
}

/* ====================================================================
 * Probabilistic loss + per-frame caching
 * ==================================================================== */

/* success_ratio_tx = 0.5 with a fixed seed yields a deterministic
 * win/loss pattern across repeated frame-level calls. */
static void test_frame_loss_deterministic_with_seed(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 0.5, 1.0);
    radio_medium_set_seed(&rm, 0xCAFEBABE);

    bool pattern1[40], pattern2[40];
    for (int i = 0; i < 40; i++) pattern1[i] = radio_medium_filter_frame(&rm, 0, 1);
    radio_medium_set_seed(&rm, 0xCAFEBABE);
    for (int i = 0; i < 40; i++) pattern2[i] = radio_medium_filter_frame(&rm, 0, 1);

    int matches = 0;
    for (int i = 0; i < 40; i++) if (pattern1[i] == pattern2[i]) matches++;
    ASSERT_EQ(matches, 40, "same seed -> identical 40-frame pattern");

    /* With ratio 0.5 we should see a healthy mix (not all-pass, not
     * all-drop) — sanity check the dice are actually being rolled. */
    int passes = 0;
    for (int i = 0; i < 40; i++) if (pattern1[i]) passes++;
    ASSERT(passes >= 5 && passes <= 35, "0.5 ratio: mix of pass/drop");
}

/* success_ratio_rx = 0.5 with both nodes at distance 0, ratio_tx=1.0:
 * the formula reduces to (1 - 0 * (1 - 0.5)) * 1.0 = 1.0 -> all pass.
 * A more telling test is at non-zero distance, where ratio_rx becomes
 * a true distance-attenuated success knob. */
static void test_rx_ratio_attenuates_with_distance(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    /* tx_range 100 so dist 50 -> ratio = 0.25, prob = 1 - 0.25*0.5 = 0.875. */
    radio_medium_configure_udgm(&rm, 100.0, 200.0, 1.0, 0.5);
    radio_medium_set_position(&rm, 0,  0.0, 0.0);
    radio_medium_set_position(&rm, 1, 50.0, 0.0);
    radio_medium_set_seed(&rm, 0xFEEDFACE);

    int passes = 0;
    for (int i = 0; i < 200; i++) {
        if (radio_medium_filter_frame(&rm, 0, 1)) passes++;
    }
    /* prob ~= 0.875 -> expect ~175 of 200; allow generous slack. */
    ASSERT(passes > 140, "rx ratio attenuates: many but not all passes");
    ASSERT(passes < 200, "rx ratio attenuates: at least some drops");

    /* And bumping distance to 90 (ratio = 0.81) drops prob to ~0.595. */
    radio_medium_set_position(&rm, 1, 90.0, 0.0);
    radio_medium_set_seed(&rm, 0xFEEDFACE);
    int closer_passes = 0;
    for (int i = 0; i < 200; i++) {
        if (radio_medium_filter_frame(&rm, 0, 1)) closer_passes++;
    }
    ASSERT(closer_passes < passes,
           "more distance -> fewer passes (rx ratio attenuation works)");
}

/* Per-frame loss decision must be cached for the duration of one
 * frame's bytes. With success_ratio_rx=0 (and dist 0 so ratio=0,
 * prob = (1 - 0) * 1 = 1.0) the formula doesn't actually drop. We need
 * dist > 0 to make rx_ratio bite. tx_range 100, dist 50 -> ratio=0.25,
 * prob = (1 - 0.25*1) * 1 = 0.75 — still mixed. To force a clean
 * "all bytes share one decision" test, set success_ratio_tx = 0.0:
 * prob is identically 0, so EVERY byte of every frame must drop.
 * Conversely with success_ratio_tx = 1.0 + match=1.0 every byte passes.
 *
 * Better: verify the cache by doing a per-byte check at success_ratio_tx
 * = 0.5 across one frame's worth of bytes — they must ALL agree (no
 * mixed pass/drop within a single frame). */
static void test_per_frame_cache_consistent_within_frame(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 0.5, 1.0);
    radio_medium_set_seed(&rm, 0xABCD1234);

    /* Drive 40 distinct frames; per frame, collect all data bytes'
     * pass/drop decisions and assert they're unanimous. */
    int frames_seen = 0;
    int unanimous = 0;
    for (int f = 0; f < 40; f++) {
        /* Preamble + SFD + length=8 + 8 payload bytes. */
        uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x08,
                            0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17 };
        bool decisions[8];
        int data_idx = 0;
        for (size_t i = 0; i < sizeof(frame); i++) {
            bool ok = radio_medium_filter_byte(&rm, 0, 1, frame[i]);
            /* The 8 payload bytes are at indices 6..13. */
            if (i >= 6) decisions[data_idx++] = ok;
        }
        frames_seen++;
        bool first = decisions[0];
        bool all_same = true;
        for (int j = 1; j < 8; j++) if (decisions[j] != first) { all_same = false; break; }
        if (all_same) unanimous++;
    }
    ASSERT_EQ(frames_seen, 40, "40 frames driven");
    ASSERT_EQ(unanimous, 40, "every frame's data bytes share one drop decision");
}

/* Hard "all-drop" check: with success_ratio_tx = 0.0 every roll fails
 * -> every frame's data bytes drop unanimously. Pre-SFD preamble bytes
 * are NOT yet in a tracked frame, so they take the dist-only path
 * (dist 0 < tx_range -> pass). After SFD (which itself is the SFD-byte
 * trigger), bytes belong to the tracked frame and drop. */
static void test_zero_tx_ratio_drops_all_data_bytes(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 0.0, 1.0);

    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x04, 0xAA, 0xBB, 0xCC, 0xDD };
    /* Preamble bytes (indices 0..3): tracked-frame is not yet armed,
     * so they take the OUTSIDE-frame path -> dist 0 < 50 -> pass. */
    for (int i = 0; i < 4; i++) {
        ASSERT(radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "preamble bytes pass via outside-frame dist check");
    }
    /* SFD byte (index 4): tracker promotes to FRAME_SFD and starts a
     * new frame inside that branch — by the time filter_byte gets to
     * the "inside frame" decision check, the SFD byte itself is
     * already gated by the new frame_id. With tx_ratio 0, drops. */
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, frame[4]),
           "SFD byte: in-frame -> drops at tx_ratio 0");
    /* Length byte and payload: still inside the frame, all drop. */
    for (int i = 5; i < 10; i++) {
        ASSERT(!radio_medium_filter_byte(&rm, 0, 1, frame[i]),
               "in-frame byte drops at tx_ratio 0");
    }
}

/* ====================================================================
 * Frame tracker — IEEE 802.15.4 (CC2420 / cc2538_rfcore)
 * ==================================================================== */

/* The frame tracker advances PREAMBLE -> SFD -> DATA on the canonical
 * IEEE 802.15.4 byte sequence and bumps frame_id once per frame. Drive
 * filter_byte and inspect rm.frame_track[sender] directly. */
static void test_frame_tracker_802154_state_progression(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    /* Use UDGM so the tracker actually runs (NONE short-circuits). */
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    /* Default channels (-1) => tracker profile is IEEE 802.15.4. */

    frame_tracker_t *ft = &rm.frame_track[0][0];
    ASSERT_EQ(ft->state, FRAME_IDLE, "starts IDLE");
    ASSERT_EQ(ft->frame_id, 0u, "no frame_id yet");

    /* First 0x00 -> PREAMBLE. */
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    ASSERT_EQ(ft->state, FRAME_PREAMBLE, "first 0x00 -> PREAMBLE");

    /* Three more 0x00 bytes -> still PREAMBLE, zero_count grows. */
    for (int i = 0; i < 3; i++) radio_medium_filter_byte(&rm, 0, 1, 0x00);
    ASSERT_EQ(ft->state, FRAME_PREAMBLE, "still PREAMBLE after 4×0x00");
    ASSERT_EQ(ft->zero_count, 4, "zero_count = 4");

    /* SFD byte -> FRAME_SFD, frame_id bumps to 1. */
    uint32_t prior_frame_id = rm.next_frame_id;
    radio_medium_filter_byte(&rm, 0, 1, 0x7A);
    ASSERT_EQ(ft->state, FRAME_SFD, "0x7A after preamble -> FRAME_SFD");
    ASSERT_EQ(ft->frame_id, prior_frame_id + 1, "frame_id bumped on SFD");

    /* Length byte -> FRAME_DATA. */
    radio_medium_filter_byte(&rm, 0, 1, 0x03);
    ASSERT_EQ(ft->state, FRAME_DATA, "after length byte -> FRAME_DATA");
    ASSERT_EQ(ft->length, 3, "length recorded");

    /* Drive 3 payload bytes -> back to IDLE. */
    radio_medium_filter_byte(&rm, 0, 1, 0xAA);
    radio_medium_filter_byte(&rm, 0, 1, 0xBB);
    radio_medium_filter_byte(&rm, 0, 1, 0xCC);
    ASSERT_EQ(ft->state, FRAME_IDLE, "after length bytes -> IDLE");
}

/* Bytes with no preamble pattern (e.g. 0xAA stream) do NOT advance the
 * IEEE 802.15.4 state machine past IDLE/PREAMBLE-aborted. */
static void test_frame_tracker_802154_no_preamble(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    frame_tracker_t *ft = &rm.frame_track[0][0];

    for (int i = 0; i < 16; i++) radio_medium_filter_byte(&rm, 0, 1, 0xAA);
    ASSERT_EQ(ft->state, FRAME_IDLE, "0xAA stream never escapes IDLE");
    ASSERT_EQ(ft->frame_id, 0u, "no frame_id assigned");

    /* A single 0x00 then a non-SFD non-zero byte resets back to IDLE. */
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    radio_medium_filter_byte(&rm, 0, 1, 0xFF);
    ASSERT_EQ(ft->state, FRAME_IDLE, "0x00 then non-SFD -> IDLE");
}

/* A new SFD after a complete frame increments frame_id again. */
static void test_frame_tracker_802154_two_frames(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    frame_tracker_t *ft = &rm.frame_track[0][0];

    /* Frame 1: 4×0x00, SFD, length=2, 2 payload. */
    uint8_t f1[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x02, 0x11, 0x22 };
    for (size_t i = 0; i < sizeof(f1); i++) radio_medium_filter_byte(&rm, 0, 1, f1[i]);
    uint32_t fid1 = ft->frame_id;
    ASSERT(fid1 > 0, "frame 1 got an id");
    ASSERT_EQ(ft->state, FRAME_IDLE, "back to IDLE after frame 1");

    /* Frame 2. */
    uint8_t f2[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x01, 0xFF };
    for (size_t i = 0; i < sizeof(f2); i++) radio_medium_filter_byte(&rm, 0, 1, f2[i]);
    uint32_t fid2 = ft->frame_id;
    ASSERT(fid2 > fid1, "frame 2 got a fresh id (fid2 > fid1)");
}

/* ====================================================================
 * Frame tracker — IEEE 802.15.4g (CC1200)
 * ==================================================================== */

/* set_channel(>= SUBGHZ_BASE) flips the tracker's profile to 802.15.4g.
 * Then a 4×0x55 preamble + the 32-bit sync word triggers FRAME_PHR_LO. */
static void test_frame_tracker_802154g_state_progression(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    radio_medium_set_channel(&rm, 0, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
    radio_medium_set_channel(&rm, 1, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);

    frame_tracker_t *ft = &rm.frame_track[0][0];
    ASSERT_EQ(ft->profile, RADIO_FRAME_PROFILE_IEEE802154G,
              "profile flipped to 802.15.4g on sub-GHz set_channel");
    ASSERT_EQ(ft->state, FRAME_IDLE, "starts IDLE");

    /* Drive preamble bytes (the 802.15.4g tracker doesn't care about
     * preamble content — it just slides bytes through sync_match). */
    for (int i = 0; i < 4; i++) radio_medium_filter_byte(&rm, 0, 1, 0x55);
    /* sync_match should now be 0x55555555; not the target. */
    ASSERT_EQ(ft->state, FRAME_PREAMBLE, "still PREAMBLE before sync");
    ASSERT_EQ(ft->sync_match, 0x55555555u, "sliding register holds preamble");

    /* Sync word = RADIO_FRAME_802154G_SYNC_WORD = 0x6E4E904E. */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    uint32_t prior_id = rm.next_frame_id;
    for (int i = 0; i < 4; i++) radio_medium_filter_byte(&rm, 0, 1, sync[i]);
    ASSERT_EQ(ft->state, FRAME_PHR_LO,
              "after sync word -> FRAME_PHR_LO (waiting for PHR_HI)");
    ASSERT_EQ(ft->frame_id, prior_id + 1, "frame_id bumped on sync");

    /* PHR_HI byte (top 3 bits of 11-bit length). Use 0x00 for length<256. */
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    ASSERT_EQ(ft->state, FRAME_SFD, "PHR_HI consumed -> FRAME_SFD slot");
    ASSERT_EQ(ft->phr_hi, 0, "phr_hi recorded");

    /* PHR_LO = 5 -> length = 5, transition to FRAME_DATA. */
    radio_medium_filter_byte(&rm, 0, 1, 0x05);
    ASSERT_EQ(ft->state, FRAME_DATA, "PHR_LO -> FRAME_DATA");
    ASSERT_EQ(ft->length, 5, "length = 5");

    /* 5 payload bytes -> IDLE. */
    for (int i = 0; i < 5; i++) radio_medium_filter_byte(&rm, 0, 1, 0xA0 + i);
    ASSERT_EQ(ft->state, FRAME_IDLE, "after 5 payload bytes -> IDLE");
}

/* Per-frame loss caching also applies to sub-GHz frames: with
 * tx_ratio=0 every payload byte of a sub-GHz frame must drop. */
static void test_frame_tracker_802154g_per_frame_cache(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 0.0, 1.0);
    radio_medium_set_channel(&rm, 0, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
    radio_medium_set_channel(&rm, 1, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);

    /* Pre-sync bytes use the OUTSIDE-frame path (dist 0 < 50 -> pass). */
    for (int i = 0; i < 4; i++) {
        ASSERT(radio_medium_filter_byte(&rm, 0, 1, 0x55),
               "sub-GHz preamble byte passes (outside frame, dist OK)");
    }
    /* Sync word: the FOURTH sync byte completes the match and starts
     * the frame inside the tracker — but for THAT byte's filter call,
     * the inside-frame branch fires and tx_ratio=0 drops it. */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    radio_medium_filter_byte(&rm, 0, 1, sync[0]);  /* not a match yet */
    radio_medium_filter_byte(&rm, 0, 1, sync[1]);
    radio_medium_filter_byte(&rm, 0, 1, sync[2]);
    /* The 4th sync byte triggers start_new_frame; that byte's filter
     * call lands in the in-frame branch and drops at tx_ratio 0. */
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, sync[3]),
           "sync-completing byte: in-frame, drops at tx_ratio 0");

    /* PHR_HI, PHR_LO, payload — all drop. */
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0x00), "PHR_HI drops");
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0x03), "PHR_LO drops");
    for (int i = 0; i < 3; i++) {
        ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0xA0),
               "sub-GHz payload byte drops");
    }
}

/* set_channel that crosses bands resets the tracker. */
static void test_set_channel_band_change_resets_tracker(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Start on 2.4 GHz, drive partial preamble. */
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    ASSERT_EQ(rm.frame_track[0][0].state, FRAME_PREAMBLE, "mid-preamble");

    /* Switch to sub-GHz — tracker resets. */
    radio_medium_set_channel(&rm, 0, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
    ASSERT_EQ(rm.frame_track[0][0].state, FRAME_IDLE,
              "band change resets state to IDLE");
    ASSERT_EQ(rm.frame_track[0][0].zero_count, 0, "zero_count reset");
    ASSERT_EQ(rm.frame_track[0][0].sync_match, 0u, "sync_match reset");
    ASSERT_EQ(rm.frame_track[0][0].profile, RADIO_FRAME_PROFILE_IEEE802154G,
              "profile flipped to 802.15.4g");
}

/* ====================================================================
 * radio_medium_get_rssi
 * ==================================================================== */

static void test_rssi_none_returns_default(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    /* Even with weird positions, NONE returns the documented -50. */
    radio_medium_set_position(&rm, 0,    0.0,    0.0);
    radio_medium_set_position(&rm, 1, 1000.0, 1000.0);
    ASSERT_EQ(radio_medium_get_rssi(&rm, 0, 1), -50,
              "NONE medium: RSSI default = -50 dBm");
}

static void test_rssi_udgm_linear(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 100.0, 200.0, 1.0, 1.0);

    /* Distance 0 -> -10 dBm. */
    ASSERT_EQ(radio_medium_get_rssi(&rm, 0, 1), -10, "dist 0 -> -10 dBm");

    /* Distance == tx_range -> -90 dBm. */
    radio_medium_set_position(&rm, 1, 100.0, 0.0);
    ASSERT_EQ(radio_medium_get_rssi(&rm, 0, 1), -90,
              "dist == tx_range -> -90 dBm");

    /* Halfway -> linearly interpolated -50 dBm. */
    radio_medium_set_position(&rm, 1, 50.0, 0.0);
    ASSERT_EQ(radio_medium_get_rssi(&rm, 0, 1), -50,
              "dist == tx_range/2 -> -50 dBm");

    /* Beyond tx_range -> clipped at -90. */
    radio_medium_set_position(&rm, 1, 250.0, 0.0);
    ASSERT_EQ(radio_medium_get_rssi(&rm, 0, 1), -90,
              "dist > tx_range -> clamped at -90 dBm");
}

/* ====================================================================
 * Edge cases
 * ==================================================================== */

/* node_count == 0: filter calls with bogus indices must not crash.
 * (The implementation is index-tolerant on the read path; the WRITE
 * path in start_new_frame only iterates 0..node_count, which is 0 here
 * so it's a no-op.) */
static void test_zero_nodes_no_crash(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 0);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    /* These should return some value without segfaulting. */
    (void)radio_medium_filter_byte(&rm, 0, 1, 0xAA);
    (void)radio_medium_filter_frame(&rm, 0, 1);
    radio_medium_compute_neighbors(&rm);
    ASSERT(true, "no crash with node_count = 0");
}

/* node_count == 1 with sender == receiver: pin current behavior. */
static void test_self_send_pins_current_behavior(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 1);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);
    /* Sender == receiver, both at (0,0) — distance 0, prob 1.0. The
     * harness gates self-delivery elsewhere; here we just pin that the
     * medium itself returns true (passes byte/frame). */
    ASSERT(radio_medium_filter_byte(&rm, 0, 0, 0xAA),
           "self-send byte: medium passes (harness gates elsewhere)");
    ASSERT(radio_medium_filter_frame(&rm, 0, 0),
           "self-send frame: medium passes (harness gates elsewhere)");
}

/* set_position bounds: out-of-range node index is silently ignored. */
static void test_set_position_bounds(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_set_position(&rm, -1, 99.0, 99.0);                     /* ignored */
    radio_medium_set_position(&rm, RADIO_MEDIUM_MAX_NODES, 99.0, 99.0); /* ignored */
    radio_medium_set_position(&rm, 0,  3.0,  4.0);                      /* stored */
    ASSERT(rm.nodes[0].x == 3.0 && rm.nodes[0].y == 4.0,
           "in-range set_position stored");
    /* Distance from (0,0) to (3,4) = 5 < tx_range — sanity. */
    radio_medium_configure_udgm(&rm, 10.0, 20.0, 1.0, 1.0);
    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "(3,4) -> (0,0) within tx_range 10");
}

/* ====================================================================
 * Multi-radio per node (refactor coverage)
 * ==================================================================== */

/* Two-radio Firefly node: cc2538_rfcore on slot 0 (2.4 GHz) + cc1200 on
 * slot 1 (sub-GHz). Registering both must not cross-contaminate. */
static void test_multiradio_register_two_radios_independent(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 0, 1, RADIO_SPECTRUM_868MHZ_15_4G);

    ASSERT_EQ(rm.nodes[0].radio_count, 2, "two radios registered");
    ASSERT_EQ(rm.nodes[0].radios[0].spectrum, RADIO_SPECTRUM_2_4GHZ_15_4,
              "slot 0 spectrum = 2.4 GHz");
    ASSERT_EQ(rm.nodes[0].radios[1].spectrum, RADIO_SPECTRUM_868MHZ_15_4G,
              "slot 1 spectrum = 868 MHz");

    /* Channels start unknown and are independent. */
    ASSERT_EQ(rm.nodes[0].radios[0].channel, -1, "slot 0 channel default -1");
    ASSERT_EQ(rm.nodes[0].radios[1].channel, -1, "slot 1 channel default -1");

    radio_medium_set_radio_channel(&rm, 0, 0, 26);
    radio_medium_set_radio_channel(&rm, 0, 1, 5);
    ASSERT_EQ(rm.nodes[0].radios[0].channel, 26, "slot 0 channel set");
    ASSERT_EQ(rm.nodes[0].radios[1].channel, 5, "slot 1 channel set");

    /* Per-radio frame trackers carry the right profile. */
    ASSERT_EQ(rm.frame_track[0][0].profile, RADIO_FRAME_PROFILE_IEEE802154,
              "slot 0 tracker = 802.15.4");
    ASSERT_EQ(rm.frame_track[0][1].profile, RADIO_FRAME_PROFILE_IEEE802154G,
              "slot 1 tracker = 802.15.4g");
}

/* Per-radio dispatch: with a Firefly node 0 (cc2538_rfcore + cc1200) and
 * a Firefly node 1 (same), bytes from slot 0 must reach slot 0, slot 1
 * must reach slot 1, and cross-slot delivery must drop. */
static void test_multiradio_per_radio_dispatch(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    for (int n = 0; n < 2; n++) {
        radio_medium_register_radio(&rm, n, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
        radio_medium_register_radio(&rm, n, 1, RADIO_SPECTRUM_868MHZ_15_4G);
        radio_medium_set_radio_channel(&rm, n, 0, 26);
        radio_medium_set_radio_channel(&rm, n, 1, 5);
    }

    /* Same-band, same-slot: passes. */
    ASSERT(radio_medium_filter_byte_radio(&rm, 0, 0, 1, 0, 0xAB),
           "node0/slot0 -> node1/slot0: passes");
    ASSERT(radio_medium_filter_byte_radio(&rm, 0, 1, 1, 1, 0xAB),
           "node0/slot1 -> node1/slot1: passes");

    /* Cross-slot delivery: spectrum mismatch drops. */
    ASSERT(!radio_medium_filter_byte_radio(&rm, 0, 0, 1, 1, 0xAB),
           "node0/slot0(2.4GHz) -> node1/slot1(868MHz): drops (band mismatch)");
    ASSERT(!radio_medium_filter_byte_radio(&rm, 0, 1, 1, 0, 0xAB),
           "node0/slot1(868MHz) -> node1/slot0(2.4GHz): drops (band mismatch)");
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 0, 1, 1),
           "filter_frame: cross-band cross-slot drops");
}

/* Changing one radio's channel must not affect the other on the same
 * node. */
static void test_multiradio_channel_change_isolation(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 0, 1, RADIO_SPECTRUM_868MHZ_15_4G);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 1, RADIO_SPECTRUM_868MHZ_15_4G);

    radio_medium_set_radio_channel(&rm, 0, 0, 11);
    radio_medium_set_radio_channel(&rm, 0, 1, 5);
    radio_medium_set_radio_channel(&rm, 1, 0, 11);
    radio_medium_set_radio_channel(&rm, 1, 1, 5);

    /* Both pairs match — both deliver. */
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "slot 0 pair matches before any change");
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 1, 1, 1),
           "slot 1 pair matches before any change");

    /* Hop only slot 0 on node 0 to channel 26. Slot 1 must continue to
     * deliver — independence proves per-radio channel state. */
    radio_medium_set_radio_channel(&rm, 0, 0, 26);
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "slot 0 pair drops after slot 0 channel change");
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 1, 1, 1),
           "slot 1 pair still matches (independent)");
    /* Slot 1 still hasn't moved. */
    ASSERT_EQ(rm.nodes[0].radios[1].channel, 5,
              "slot 1 channel unaffected by slot 0 change");
}

/* Spectrum mismatch on the same channel index is still a drop (CC1200
 * channel 5 is not the same as cc2538 channel 5). */
static void test_multiradio_spectrum_mismatch_same_channel_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_868MHZ_15_4G);
    radio_medium_set_radio_channel(&rm, 0, 0, 5);
    radio_medium_set_radio_channel(&rm, 1, 0, 5);

    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "channel 5 on 2.4 GHz vs channel 5 on 868 MHz: drops");
    ASSERT(!radio_medium_filter_byte_radio(&rm, 0, 0, 1, 0, 0xAA),
           "byte: channel 5 on 2.4 GHz vs 868 MHz: drops");
}

/* RX-disabled receiver: even when band + channel match, no delivery. */
static void test_rx_disabled_receiver_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_set_radio_channel(&rm, 0, 0, 26);
    radio_medium_set_radio_channel(&rm, 1, 0, 26);

    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "default RX-on: frame passes");

    radio_medium_set_radio_rx_enabled(&rm, 1, 0, false);
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "RX-off receiver: frame dropped");

    radio_medium_set_radio_rx_enabled(&rm, 1, 0, true);
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "RX re-enabled: frame passes again");
}

/* TSCH-style rapid hopping: 50 channel changes simulated as a sequence
 * of "set channel + check if the in-flight byte matches". Each delivered
 * byte is matched against the channel that was active at the moment of
 * the call — proves channel state is sampled synchronously, not via
 * stale state. */
static void test_tsch_rapid_hopping_per_byte_match(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);

    int delivered = 0, dropped = 0;
    /* 50 hops across channels 11..15. Sender stays on channel 11; the
     * receiver hops every iteration. Only when the receiver is on 11 does
     * the byte deliver. */
    radio_medium_set_radio_channel(&rm, 0, 0, 11);
    for (int i = 0; i < 50; i++) {
        int rx_ch = 11 + (i % 5);  /* 11,12,13,14,15,11,12,... */
        radio_medium_set_radio_channel(&rm, 1, 0, rx_ch);
        bool ok = radio_medium_filter_byte_radio(&rm, 0, 0, 1, 0, 0xAA);
        if (ok) delivered++; else dropped++;
    }
    /* 10 of 50 iterations should land on rx_ch == 11. */
    ASSERT_EQ(delivered, 10, "rapid hop: 10/50 bytes match channel 11");
    ASSERT_EQ(dropped, 40, "rapid hop: 40/50 bytes drop on mismatch");
}

/* Cross-radio TSCH: sender's slot 0 hops while slot 1 stays parked.
 * Receiver's slot 1 must continue to deliver even when slot 0 mismatches. */
static void test_tsch_hop_one_radio_other_unaffected(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    for (int n = 0; n < 2; n++) {
        radio_medium_register_radio(&rm, n, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
        radio_medium_register_radio(&rm, n, 1, RADIO_SPECTRUM_868MHZ_15_4G);
        radio_medium_set_radio_channel(&rm, n, 1, 5);
    }
    radio_medium_set_radio_channel(&rm, 0, 0, 11);
    radio_medium_set_radio_channel(&rm, 1, 0, 11);

    int slot0_match = 0, slot1_match = 0;
    for (int i = 0; i < 16; i++) {
        /* Hop sender's 2.4 GHz radio between 11 and 26. */
        int sch = (i & 1) ? 26 : 11;
        radio_medium_set_radio_channel(&rm, 0, 0, sch);
        if (radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0)) slot0_match++;
        if (radio_medium_filter_frame_radio(&rm, 0, 1, 1, 1)) slot1_match++;
    }
    /* Slot 0: half the iterations land on ch 11 (matches receiver). */
    ASSERT_EQ(slot0_match, 8, "slot 0 hopping: half land on ch 11");
    /* Slot 1: parked on ch 5 both ends, never touched -> all 16 deliver. */
    ASSERT_EQ(slot1_match, 16, "slot 1 unaffected by slot 0 hops");
}

/* Native-style mid-tick channel hop. Simulates a Cooja mote whose
 * simRadioChannel pointer is mutated between two filter calls without
 * any "sync event" inserted. Mirrors what test_mixed_multinode does
 * for native motes after the harness moved channel sync from tick
 * boundaries down to the byte-delivery sites: the harness reads
 * simRadioChannel and pushes it via radio_medium_set_radio_channel
 * just before each filter_byte/filter_frame, so a TSCH-style mid-tick
 * hop is honoured at the moment the medium consults the value.
 *
 * This pins the medium-side guarantee: a set_radio_channel call landing
 * between two filter calls flips the delivery decision immediately,
 * with no inflight-frame caching that could mask the new channel. */
static void test_native_mid_tick_channel_change_honored(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Sender stays parked on ch 11. Receiver represents a native mote:
     * the harness will push whatever simRadioChannel says before each
     * filter call. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_set_radio_channel(&rm, 0, 0, 11);

    /* Pretend the firmware hopped the receiver to ch 11 — frame passes. */
    radio_medium_set_radio_channel(&rm, 1, 0, 11);
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "native sync: rx on ch 11 -> frame passes");

    /* Mid-"tick": firmware (TSCH) hops to ch 25 underneath. The harness
     * reads simRadioChannel and calls set_radio_channel BEFORE the next
     * filter call. The medium must honour the new channel right now. */
    radio_medium_set_radio_channel(&rm, 1, 0, 25);
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "native sync: hopped to ch 25 -> next frame drops");

    /* And back: same tick, hop to ch 11 again. */
    radio_medium_set_radio_channel(&rm, 1, 0, 11);
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "native sync: hopped back to ch 11 -> frame passes again");

    /* Per-byte: feed a complete IEEE 802.15.4 frame's bytes while
     * toggling the receiver's channel each call. Channel match is now
     * decided per-frame at the SFD-detection point, mirroring Cooja's
     * RadioConnection / real-radio commitment to the TX-start channel.
     *
     * Outside the frame (preamble bytes 0-3): live channel comparison
     * — even bytes (rx on ch 11) pass, odd (rx on ch 25) drop.
     * At the SFD byte (index 4): the medium snapshots sender's channel,
     * compares it to the receiver's current channel, caches the
     * decision for this (sender, receiver) pair. SFD lands on i=4
     * (even) so receiver is on ch 11 → match, decision frozen as PASS.
     * Bytes 5-10 use the cached decision and pass regardless of the
     * receiver's per-byte hops — this is the "chip is committed once
     * the demod has locked" model. */
    int delivered = 0, dropped = 0;
    uint8_t frame[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x05,
                        0x11,0x22,0x33,0x44,0x55 };
    for (size_t i = 0; i < sizeof(frame); i++) {
        /* Even index byte: receiver hopped to ch 11; odd: ch 25. */
        radio_medium_set_radio_channel(&rm, 1, 0, (i & 1) ? 25 : 11);
        if (radio_medium_filter_byte_radio(&rm, 0, 0, 1, 0, frame[i]))
            delivered++;
        else
            dropped++;
    }
    /* Pre-frame: i=0,2 pass (even, ch 11), i=1,3 drop (odd, ch 25).
     * In-frame from i=4 onward (7 bytes): all pass via cached decision. */
    ASSERT_EQ(delivered, 9, "per-frame channel: pre-frame even passes + 7 in-frame");
    ASSERT_EQ(dropped, 2, "per-frame channel: pre-frame odd preamble drops");
}

/* ====================================================================
 * Backward compat — single-radio API still works
 * ==================================================================== */

/* Existing platforms that only ever call radio_medium_set_channel(rm,
 * n, ch) must continue to work unchanged. The legacy API forwards to
 * slot 0 and auto-registers the appropriate spectrum. */
static void test_legacy_set_channel_auto_registers_24ghz(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_set_channel(&rm, 0, 26);
    radio_medium_set_channel(&rm, 1, 26);

    ASSERT_EQ(rm.nodes[0].radios[0].spectrum, RADIO_SPECTRUM_2_4GHZ_15_4,
              "legacy 2.4 GHz channel auto-registers spectrum");
    ASSERT_EQ(rm.nodes[0].radio_count, 1, "single-radio: count = 1");
    ASSERT_EQ(rm.nodes[0].channel, 26, "legacy alias mirrors slot 0 channel");
    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "legacy single-radio: same channel passes");
}

static void test_legacy_set_channel_auto_registers_subghz(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_set_channel(&rm, 0, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE + 0);
    radio_medium_set_channel(&rm, 1, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE + 0);

    ASSERT_EQ(rm.nodes[0].radios[0].spectrum, RADIO_SPECTRUM_868MHZ_15_4G,
              "legacy sub-GHz channel auto-registers 868 MHz spectrum");
    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "legacy single-radio sub-GHz: same channel passes");
}

/* Legacy single-radio API drops on within-band channel mismatch — same
 * post-refactor enforcement as the per-radio API. */
static void test_legacy_single_radio_channel_mismatch_drops(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, 26);

    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "legacy: within-band channel mismatch drops");
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0xAA),
           "legacy: within-band channel-mismatch byte drops");
}

/* Mixing the two APIs on the same node — a subsequent
 * radio_medium_set_channel call must keep slot 0's channel in sync
 * with the legacy alias (no drift). */
static void test_legacy_alias_stays_in_sync(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 1);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_set_radio_channel(&rm, 0, 0, 11);
    ASSERT_EQ(rm.nodes[0].channel, 11, "alias updated by per-radio set");

    radio_medium_set_radio_channel(&rm, 0, 0, 26);
    ASSERT_EQ(rm.nodes[0].channel, 26, "alias follows slot-0 hops");

    /* Legacy set_channel also updates the alias. */
    radio_medium_set_channel(&rm, 0, 15);
    ASSERT_EQ(rm.nodes[0].channel, 15, "alias updated by legacy set");
    ASSERT_EQ(rm.nodes[0].radios[0].channel, 15,
              "legacy set propagates to slot 0");
}

/* Per-radio frame trackers don't share state. Driving a 2.4 GHz preamble
 * on slot 0 must not advance slot 1's 802.15.4g sync-word matcher. */
static void test_per_radio_frame_trackers_are_independent(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Same node has both radios — sender side. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 0, 1, RADIO_SPECTRUM_868MHZ_15_4G);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 1, 1, RADIO_SPECTRUM_868MHZ_15_4G);

    /* Drive a complete 802.15.4 frame on slot 0. */
    uint8_t f1[] = { 0x00,0x00,0x00,0x00, 0x7A, 0x02, 0x11, 0x22 };
    for (size_t i = 0; i < sizeof(f1); i++)
        radio_medium_filter_byte_radio(&rm, 0, 0, 1, 0, f1[i]);

    /* Slot 0 tracker advanced; slot 1 tracker untouched. */
    ASSERT(rm.frame_track[0][0].frame_id > 0,
           "slot 0 sender tracker fired");
    ASSERT_EQ(rm.frame_track[0][1].frame_id, 0u,
              "slot 1 sender tracker never fired");
    ASSERT_EQ(rm.frame_track[0][1].sync_match, 0u,
              "slot 1 sync-word matcher untouched");
}

/* Re-registering a radio with the SAME spectrum is a no-op: it does NOT
 * reset the tracker mid-frame (so a redundant chip-driver init call
 * doesn't drop in-flight bytes). */
static void test_register_same_spectrum_is_noop(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    /* Drive partial preamble. */
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    radio_medium_filter_byte(&rm, 0, 1, 0x00);
    ASSERT_EQ(rm.frame_track[0][0].state, FRAME_PREAMBLE, "mid-preamble");

    /* Re-register with same spectrum -> tracker should stay. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    ASSERT_EQ(rm.frame_track[0][0].state, FRAME_PREAMBLE,
              "same-spectrum re-register: tracker preserved");

    /* Switch to a different spectrum -> tracker resets. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_868MHZ_15_4G);
    ASSERT_EQ(rm.frame_track[0][0].state, FRAME_IDLE,
              "spectrum-change register: tracker reset");
    ASSERT_EQ(rm.frame_track[0][0].profile, RADIO_FRAME_PROFILE_IEEE802154G,
              "spectrum-change register: profile flipped");
}

/* The unregistered/legacy "channel-only" path still uses the sub-GHz-base
 * heuristic for cross-band detection. Two nodes that haven't called
 * register_radio (only set_channel) still get the legacy cross-band
 * check based on channel value vs RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE. */
static void test_unregistered_radio_falls_back_to_channel_heuristic(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Both radios get registered automatically by the legacy
     * set_channel path (one to 2.4 GHz, the other to sub-GHz), so the
     * spectrum gate should drop the pair. */
    radio_medium_set_channel(&rm, 0, 11);
    radio_medium_set_channel(&rm, 1, RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);

    ASSERT(!radio_medium_filter_frame(&rm, 0, 1),
           "auto-registered cross-band: drops");
    ASSERT(!radio_medium_filter_byte(&rm, 0, 1, 0xAA),
           "auto-registered cross-band byte: drops");
}

/* Non-zero-slot delivery requires both ends to register that slot.
 * Firefly (slot 1 cc1200) MUST NOT leak bytes onto a single-radio
 * cc2538dk (slot 1 unregistered). Slot 0 keeps the legacy "unregistered
 * receiver allows everything" semantic for backward compat. */
static void test_nonzero_slot_drops_unregistered_receiver(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Sender: dual-radio Firefly. Receiver: single-radio cc2538dk. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_register_radio(&rm, 0, 1, RADIO_SPECTRUM_868MHZ_15_4G);
    radio_medium_register_radio(&rm, 1, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_set_radio_channel(&rm, 0, 0, 26);
    radio_medium_set_radio_channel(&rm, 0, 1, 5);
    radio_medium_set_radio_channel(&rm, 1, 0, 26);

    /* Slot 0 -> Slot 0: passes (both are 2.4 GHz on ch 26). */
    ASSERT(radio_medium_filter_frame_radio(&rm, 0, 0, 1, 0),
           "slot 0 (registered) -> slot 0 (registered): passes");

    /* Slot 1 -> Slot 1: receiver has no slot 1 — drop. */
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 1, 1, 1),
           "slot 1 sub-GHz sender -> slot 1 unregistered receiver: drops");
    ASSERT(!radio_medium_filter_byte_radio(&rm, 0, 1, 1, 1, 0xAA),
           "byte: slot 1 -> slot 1 unregistered: drops");

    /* Slot 1 -> Slot 0: spectrum mismatch (sender 868 vs receiver 2.4). */
    ASSERT(!radio_medium_filter_frame_radio(&rm, 0, 1, 1, 0),
           "slot 1 sub-GHz -> slot 0 2.4 GHz: drops on spectrum");
}

/* If receiver_radio is unregistered (NONE) and sender is registered, the
 * spectrum gate falls back to the channel-heuristic and pairs based on
 * channel range. Useful as a safety net during partial migrations. */
static void test_partial_registration_pairs_via_channel(void) {
    radio_medium_t rm;
    radio_medium_init(&rm, 2);
    radio_medium_configure_udgm(&rm, 50.0, 100.0, 1.0, 1.0);

    /* Sender registers explicit spectrum; receiver only sets channel
     * via the legacy path. The legacy path auto-registers the matching
     * spectrum, so this still works. */
    radio_medium_register_radio(&rm, 0, 0, RADIO_SPECTRUM_2_4GHZ_15_4);
    radio_medium_set_radio_channel(&rm, 0, 0, 11);
    radio_medium_set_channel(&rm, 1, 11);  /* legacy path */

    ASSERT(radio_medium_filter_frame(&rm, 0, 1),
           "partial-reg pair on same band/channel: passes");
}

/* ==================================================================== */

int run_radio_medium_tests(int verbose) {
    (void)verbose;
    printf("=== radio_medium_t Unit Tests ===\n");

    /* Init + defaults */
    test_init_defaults_pass_all();
    test_configure_udgm_activates_filter();
    test_seed_reproducibility();
    test_set_seed_zero_falls_back();

    /* Channel matching */
    test_channel_match_passes();
    test_within_band_channel_mismatch_drops();
    test_channel_unknown_passes();

    /* Cross-band isolation */
    test_cross_band_drops_both_directions();
    test_subghz_same_channel_passes();
    test_subghz_different_channels_drops();

    /* UDGM distance */
    test_distance_inside_tx_range_passes();
    test_distance_outside_tx_range_drops();
    test_distance_beyond_interference_drops();
    test_distance_exact_tx_range_boundary();
    test_compute_neighbors_4node_square();

    /* Probabilistic loss + per-frame caching */
    test_frame_loss_deterministic_with_seed();
    test_rx_ratio_attenuates_with_distance();
    test_per_frame_cache_consistent_within_frame();
    test_zero_tx_ratio_drops_all_data_bytes();

    /* Frame tracker — IEEE 802.15.4 */
    test_frame_tracker_802154_state_progression();
    test_frame_tracker_802154_no_preamble();
    test_frame_tracker_802154_two_frames();

    /* Frame tracker — IEEE 802.15.4g */
    test_frame_tracker_802154g_state_progression();
    test_frame_tracker_802154g_per_frame_cache();
    test_set_channel_band_change_resets_tracker();

    /* RSSI */
    test_rssi_none_returns_default();
    test_rssi_udgm_linear();

    /* Edge cases */
    test_zero_nodes_no_crash();
    test_self_send_pins_current_behavior();
    test_set_position_bounds();

    /* Multi-radio per node */
    test_multiradio_register_two_radios_independent();
    test_multiradio_per_radio_dispatch();
    test_multiradio_channel_change_isolation();
    test_multiradio_spectrum_mismatch_same_channel_drops();
    test_rx_disabled_receiver_drops();
    test_per_radio_frame_trackers_are_independent();
    test_register_same_spectrum_is_noop();

    /* TSCH-style hopping */
    test_tsch_rapid_hopping_per_byte_match();
    test_tsch_hop_one_radio_other_unaffected();
    test_native_mid_tick_channel_change_honored();

    /* Backward compat */
    test_legacy_set_channel_auto_registers_24ghz();
    test_legacy_set_channel_auto_registers_subghz();
    test_legacy_single_radio_channel_mismatch_drops();
    test_legacy_alias_stays_in_sync();
    test_unregistered_radio_falls_back_to_channel_heuristic();
    test_nonzero_slot_drops_unregistered_receiver();
    test_partial_registration_pairs_via_channel();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
