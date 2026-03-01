/*
 * Shared nanosecond <-> cycle conversion helpers
 */
#ifndef CPU_TIME_H
#define CPU_TIME_H

#include <stdint.h>

static inline int64_t cpu_ns_to_cycles(int64_t ns, uint32_t freq_hz) {
    return ns * (int64_t)freq_hz / 1000000000LL;
}

static inline int64_t cpu_cycles_to_ns(int64_t cycles, uint32_t freq_hz) {
    if (freq_hz == 0) return 0;
    return cycles * 1000000000LL / (int64_t)freq_hz;
}

/* Architecture-specific aliases for backward compatibility */
#define msp430_ns_to_cycles cpu_ns_to_cycles
#define msp430_cycles_to_ns cpu_cycles_to_ns
#define arm_ns_to_cycles    cpu_ns_to_cycles
#define arm_cycles_to_ns    cpu_cycles_to_ns

#endif /* CPU_TIME_H */
