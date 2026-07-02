/*
 * Shared nanosecond <-> cycle conversion helpers
 */
#ifndef CPU_TIME_H
#define CPU_TIME_H

#include <stdint.h>

/* Both conversions compute floor((a * b) / c) exactly, but WITHOUT a
 * 128-bit division: split a = q*c + r, then (a*b)/c == q*b + (r*b)/c
 * (exact integer identity — q*b*c is divisible by c; holds for C's
 * truncating division at any sign since r carries a's sign).  The
 * remainder products fit int64: r < 1e9 (resp. r < freq_hz <= 2^32) and
 * the multiplier is <= 2^32 (resp. 1e9), so |r*b| < 4.3e18 < 2^63.
 *
 * The previous (__int128)a * b / c form compiled to a __divti3 libcall
 * on every conversion — these run per scheduled event / per peripheral
 * counter read, and the libcall showed up at ~3% of simulation wall
 * time.  Results are bit-identical, so event timing / timelines don't
 * change. */
static inline int64_t cpu_ns_to_cycles(int64_t ns, uint32_t freq_hz) {
    int64_t q = ns / 1000000000LL;
    int64_t r = ns % 1000000000LL;
    return q * (int64_t)freq_hz + (r * (int64_t)freq_hz) / 1000000000LL;
}

static inline int64_t cpu_cycles_to_ns(int64_t cycles, uint32_t freq_hz) {
    if (freq_hz == 0) return 0;
    int64_t q = cycles / (int64_t)freq_hz;
    int64_t r = cycles % (int64_t)freq_hz;
    return q * 1000000000LL + (r * 1000000000LL) / (int64_t)freq_hz;
}

/* Architecture-specific aliases for backward compatibility */
#define msp430_ns_to_cycles cpu_ns_to_cycles
#define msp430_cycles_to_ns cpu_cycles_to_ns
#define arm_ns_to_cycles    cpu_ns_to_cycles
#define arm_cycles_to_ns    cpu_cycles_to_ns

#endif /* CPU_TIME_H */
