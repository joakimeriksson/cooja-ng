/*
 * Per-CPU peripheral event — unified type used by both MSP430 and ARM
 * CPU event queues.
 *
 * The MSP430 and ARM event queues are structurally identical: each entry
 * carries a fire cycle, an optional wall-clock fire ns, a callback, user
 * data, and a singly-linked next pointer. Keeping a single struct avoids
 * twin types and lets off-SoC chip drivers (CC2420, etc.) be written
 * against a CPU-agnostic event abstraction.
 *
 * Architecture-specific names (msp430_event_t, arm_event_t) are kept as
 * typedef aliases for source-compatibility.
 */
#ifndef CPU_EVENT_H
#define CPU_EVENT_H

#include <stdint.h>

typedef struct cpu_event cpu_event_t;
typedef void (*cpu_event_fn)(void *user_data, cpu_event_t *event);

struct cpu_event {
    int64_t       fire_cycle;
    int64_t       fire_ns;       /* wall-clock fire time (0 = cycle-based only) */
    cpu_event_fn  callback;
    void         *user_data;
    cpu_event_t  *next;
};

#endif /* CPU_EVENT_H */
