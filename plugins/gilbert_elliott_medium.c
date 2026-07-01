/*
 * gilbert_elliott_medium — a Gilbert-Elliott two-state burst-loss radio medium.
 *
 * A radio-medium plugin (Phase 11 ABI v2): registers a `sim_medium_ops_t` policy
 * via register_radio_medium and is selected by config `medium.type: "gilbert-elliott"`.
 * It reuses the host's UDGM pipeline (frame tracking, spectrum/channel matching,
 * the dice roll) and overrides only the per-frame delivery policy.
 *
 * MODEL.  Each directed link (sender -> receiver) is an independent two-state
 * Markov chain: GOOD (deliver) and BAD (drop).  Transition probabilities p
 * (GOOD->BAD) and r (BAD->GOOD) are derived from two user-facing knobs so the
 * model is directly comparable to the host's i.i.d. UDGM Bernoulli sweep:
 *
 *   - avg_drop  : the target average frame-drop probability.  With the classic
 *                 GOOD=lossless / BAD=lossy assignment used here, the stationary
 *                 bad-state probability pi_B = p/(p+r) equals avg_drop, so the
 *                 long-run drop rate is exactly avg_drop.
 *   - burst_len : the mean dwell time in the BAD state, in frames = 1/r.
 *
 *   r = 1 / burst_len ;  p = r * pi_B / (1 - pi_B) ,  pi_B = avg_drop.
 *
 * burst_len = 1 makes the chain memoryless and reproduces the i.i.d. baseline
 * (a sanity check); larger burst_len clusters losses into bursts while holding
 * the average drop rate fixed.  reception_prob() returns 1.0 in GOOD and 0.0 in
 * BAD (the host then applies its dice roll); the randomness lives in the Markov
 * transitions, advanced once per frame per link (reception_prob is called
 * exactly once per frame per receiver).
 *
 * CONFIG (environment variables, following the codebase CSIM_* convention; the
 * effective values are logged at init for run traceability):
 *   CSIM_GE_AVG_DROP   average frame-drop probability      (default 0.20)
 *   CSIM_GE_BURST_LEN  mean BAD-state dwell, in frames      (default 1.0 = i.i.d.)
 *   CSIM_GE_SEED       PRNG seed for the Markov transitions (default 1)
 *   CSIM_GE_WARMUP_FRAMES  deliver the first N frames loss-free, before the GE
 *                      channel engages (default 0).  This isolates a later phase
 *                      of interest (e.g. an EDHOC handshake) from earlier
 *                      loss-sensitive network setup (RPL DODAG formation), which
 *                      would otherwise dominate the outcome.  Counted globally
 *                      over all frames; the Markov chains stay in their initial
 *                      state until warmup completes.
 *
 * Build via the Makefile `plugins` target; select with
 *   "plugins": ["build/plugins/gilbert_elliott_medium.so"],
 *   "medium":  { "type": "gilbert-elliott", "tx_range": ..., "interference_range": ... }
 * and pass the knobs in the environment, e.g.
 *   CSIM_GE_AVG_DROP=0.2 CSIM_GE_BURST_LEN=3 ./build/test_runner ... <config>
 */
#include "csim_plugin.h"
#include "radio_medium.h"
#include "gilbert_elliott_model.h"

#include <stdlib.h>
#include <string.h>

#define GE_MAX_NODES 64

/* Derived Markov parameters (set in csim_plugin_init from the env knobs). */
static double ge_p;          /* GOOD -> BAD */
static double ge_r;          /* BAD  -> GOOD */
static double ge_pi_bad;     /* stationary BAD probability = avg_drop */

/* Per-link state, lazily initialised to the stationary distribution. */
static unsigned char ge_state[GE_MAX_NODES][GE_MAX_NODES];
static unsigned char ge_init[GE_MAX_NODES][GE_MAX_NODES];

/* One reproducible PRNG (xorshift32) for all transitions; the sim scheduler is
 * deterministic, so a fixed seed yields a fixed run. */
static uint32_t ge_rng_state = 1u;

/* Loss-free warmup: deliver the first ge_warmup_frames frames unconditionally,
 * counted globally, so an earlier setup phase (RPL) is not confounded with the
 * measured phase.  ge_frames counts frames the channel has been asked about. */
static unsigned long ge_warmup_frames = 0ul;
static unsigned long ge_frames = 0ul;

static double ge_rng_uniform(void) {
    uint32_t x = ge_rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    ge_rng_state = x;
    return (double)x / (double)0xFFFFFFFFu;
}

static double ge_env_double(const char *key, double dflt) {
    const char *v = getenv(key);
    return (v && *v) ? atof(v) : dflt;
}

/* The per-frame, per-link delivery policy. */
static double ge_reception_prob(const radio_medium_t *rm, int sender,
                                int receiver) {
    (void)rm;
    if (sender < 0 || receiver < 0 ||
        sender >= GE_MAX_NODES || receiver >= GE_MAX_NODES) {
        return 1.0;   /* out of range: don't drop */
    }
    if (ge_frames < ge_warmup_frames) {
        ge_frames++;
        return 1.0;   /* warmup: deliver loss-free, chain stays at initial state */
    }
    ge_frames++;
    if (!ge_init[sender][receiver]) {
        ge_state[sender][receiver] =
            (unsigned char)ge_initial_state(ge_pi_bad, ge_rng_uniform());
        ge_init[sender][receiver] = 1u;
    }
    int cur = ge_state[sender][receiver];
    /* Advance the chain for the next frame. */
    ge_state[sender][receiver] =
        (unsigned char)ge_advance(cur, ge_p, ge_r, ge_rng_uniform());
    return (cur == GE_GOOD) ? 1.0 : 0.0;
}

static int8_t ge_get_rssi(const radio_medium_t *rm, int sender, int receiver) {
    (void)rm; (void)sender; (void)receiver;
    return -60;
}

/* Distance-disc neighbours, exactly like UDGM/lossy, built through accessors. */
static void ge_compute_neighbors(radio_medium_t *rm) {
    udgm_config_t p;
    radio_medium_udgm_params(rm, &p);
    double tx2  = p.tx_range * p.tx_range;
    double int2 = p.interference_range * p.interference_range;
    int n = radio_medium_node_count(rm);
    for (int i = 0; i < n; i++) {
        radio_medium_clear_neighbors(rm, i);
        double xi, yi;
        radio_medium_node_pos(rm, i, &xi, &yi);
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            double xj, yj;
            radio_medium_node_pos(rm, j, &xj, &yj);
            double dx = xi - xj, dy = yi - yj;
            double d2 = dx * dx + dy * dy;
            if (d2 <= tx2)
                radio_medium_add_neighbor(rm, i, j);
            else if (d2 <= int2)
                radio_medium_add_interferer(rm, i, j);
        }
    }
}

static const sim_medium_ops_t ge_ops = {
    .name              = "gilbert-elliott",
    .reception_prob    = ge_reception_prob,
    .get_rssi          = ge_get_rssi,
    .compute_neighbors = ge_compute_neighbors,
};

static const sim_medium_type_t ge_type = {
    .name     = "gilbert-elliott",
    .pipeline = RADIO_MEDIUM_UDGM,   /* the full filter pipeline, our policy */
    .ops      = &ge_ops,
};

int csim_plugin_init(const csim_api_t *api) {
    if (!api || api->version < 2u)
        return -1;
    if (!api->registry || !api->registry->register_radio_medium)
        return -1;

    double avg_drop  = ge_env_double("CSIM_GE_AVG_DROP", 0.20);
    double burst_len = ge_env_double("CSIM_GE_BURST_LEN", 1.0);
    double seed      = ge_env_double("CSIM_GE_SEED", 1.0);

    ge_calibrate(avg_drop, burst_len, &ge_p, &ge_r, &ge_pi_bad);
    ge_rng_state = (uint32_t)seed ? (uint32_t)seed : 1u;
    ge_warmup_frames = (unsigned long)ge_env_double("CSIM_GE_WARMUP_FRAMES", 0.0);
    ge_frames = 0ul;
    memset(ge_init, 0, sizeof(ge_init));

    if (api->log && api->log->printf) {
        api->log->printf("[gilbert-elliott] avg_drop=%.4f burst_len=%.2f "
                         "seed=%u warmup=%lu -> p(G->B)=%.5f r(B->G)=%.5f "
                         "pi_bad=%.4f\n",
                         avg_drop, burst_len, ge_rng_state, ge_warmup_frames,
                         ge_p, ge_r, ge_pi_bad);
    }

    return api->registry->register_radio_medium(api->reg, &ge_type) < 0 ? -1 : 0;
}
