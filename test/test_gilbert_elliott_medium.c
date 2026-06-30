/*
 * test_gilbert_elliott_medium — statistical validation of the Gilbert-Elliott
 * burst-loss model shared with plugins/gilbert_elliott_medium.c.
 *
 * Drives the model over a large frame count and checks the two calibration
 * invariants:
 *   1. the marginal drop rate equals `avg_drop` for EVERY burst length
 *      (avg drop is held constant; only the clustering changes), and
 *   2. the mean BAD-state run length equals `burst_len`.
 *
 * Standalone (own main); build + run:
 *   cc -std=c11 -I plugins test/test_gilbert_elliott_medium.c -lm -o /tmp/ge_test && /tmp/ge_test
 */
#include "gilbert_elliott_model.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Same xorshift32 the plugin uses, fixed seed for reproducibility. */
static uint32_t s = 2463534242u;
static double uni(void) {
    uint32_t x = s; x ^= x << 13; x ^= x >> 17; x ^= x << 5; s = x;
    return (double)x / (double)0xFFFFFFFFu;
}

static int check(double avg_drop, double burst_len,
                 double drop_tol, double burst_tol) {
    double p, r, pi;
    ge_calibrate(avg_drop, burst_len, &p, &r, &pi);

    const long N = 2000000;
    long drops = 0, bursts = 0, bad_total = 0;
    int in_bad = 0;
    int st = ge_initial_state(pi, uni());
    for (long i = 0; i < N; i++) {
        int cur = st;
        st = ge_advance(cur, p, r, uni());
        if (cur == GE_BAD) {
            drops++; bad_total++;
            if (!in_bad) { bursts++; in_bad = 1; }
        } else {
            in_bad = 0;
        }
    }
    double drate = (double)drops / (double)N;
    double mean_burst = bursts ? (double)bad_total / (double)bursts : 0.0;
    int ok = fabs(drate - avg_drop) < drop_tol &&
             fabs(mean_burst - burst_len) < burst_tol;
    printf("  avg_drop=%.2f burst_len=%.1f -> p=%.5f r=%.5f | "
           "drop=%.4f mean_burst=%.3f  %s\n",
           avg_drop, burst_len, p, r, drate, mean_burst, ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}

int main(void) {
    printf("Gilbert-Elliott model statistical validation (N=2e6 frames):\n");
    int fail = 0;
    /* avg drop held at 0.20 across burst lengths 1 (i.i.d.) / 3 / 8 */
    fail += check(0.20, 1.0, 0.005, 0.10);
    fail += check(0.20, 3.0, 0.006, 0.15);
    fail += check(0.20, 8.0, 0.008, 0.40);
    /* a second operating point */
    fail += check(0.10, 5.0, 0.005, 0.30);
    if (fail) { printf("FAIL: %d case(s)\n", fail); return 1; }
    printf("PASS: all cases\n");
    return 0;
}
