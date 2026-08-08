/*
 * gilbert_elliott_model.h — the Gilbert-Elliott two-state burst-loss model,
 * shared by the radio-medium plugin (gilbert_elliott_medium.c) and its unit
 * test (test/test_gilbert_elliott_medium.c) so both exercise identical logic.
 *
 * Classic GE with GOOD = lossless, BAD = lossy: the stationary BAD probability
 * pi_B = p/(p+r) equals the user knob `avg_drop`, and the mean BAD dwell 1/r
 * equals the user knob `burst_len` (in frames). burst_len = 1 is memoryless
 * (reproduces an i.i.d. Bernoulli channel at the same average drop rate).
 */
#ifndef GILBERT_ELLIOTT_MODEL_H
#define GILBERT_ELLIOTT_MODEL_H

#define GE_GOOD 0
#define GE_BAD  1

/* Derive the Markov transition probabilities from the two user-facing knobs.
 * avg_drop is clamped to [0, 0.999] (keeps p finite); burst_len to >= 1. */
static inline void ge_calibrate(double avg_drop, double burst_len,
                                double *p, double *r, double *pi_bad) {
    if (avg_drop < 0.0) avg_drop = 0.0;
    if (avg_drop > 0.999) avg_drop = 0.999;
    if (burst_len < 1.0) burst_len = 1.0;
    *pi_bad = avg_drop;
    *r = 1.0 / burst_len;                       /* BAD -> GOOD */
    *p = (*r) * avg_drop / (1.0 - avg_drop);    /* GOOD -> BAD */
}

/* Advance the chain one frame. `state` is GE_GOOD/GE_BAD; `u0`,`u1` are two
 * independent uniforms in [0,1). Returns the (possibly) new state. */
static inline int ge_advance(int state, double p, double r, double u) {
    if (state == GE_GOOD)
        return (u < p) ? GE_BAD : GE_GOOD;
    return (u < r) ? GE_GOOD : GE_BAD;
}

/* Draw the initial state from the stationary distribution. */
static inline int ge_initial_state(double pi_bad, double u) {
    return (u < pi_bad) ? GE_BAD : GE_GOOD;
}

#endif /* GILBERT_ELLIOTT_MODEL_H */
