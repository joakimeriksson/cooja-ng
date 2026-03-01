/*
 * Generic event queue — instantiate with EVENT_QUEUE_IMPL()
 *
 * Both MSP430 and ARM use identical event scheduling, cancellation,
 * execution, and frequency-change logic. This macro generates all
 * six functions with the correct type names.
 *
 * Usage in msp430_cpu.c:
 *   EVENT_QUEUE_IMPL(msp430, msp430_cpu_t, msp430_event_t)
 *
 * Usage in arm_cpu.c:
 *   EVENT_QUEUE_IMPL(arm, arm_cpu_t, arm_event_t)
 */
#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include "cpu_time.h"
#include <limits.h>

#define EVENT_QUEUE_IMPL(PREFIX, cpu_t, event_t)                                \
                                                                                \
void PREFIX##_schedule_event(cpu_t *cpu, event_t *ev, int64_t cycle) {          \
    PREFIX##_cancel_event(cpu, ev);                                             \
                                                                                \
    ev->fire_cycle = cycle;                                                     \
    ev->fire_ns = 0;                                                            \
    ev->next = NULL;                                                            \
                                                                                \
    if (cpu->event_queue == NULL || cycle < cpu->event_queue->fire_cycle) {     \
        ev->next = cpu->event_queue;                                            \
        cpu->event_queue = ev;                                                  \
    } else {                                                                    \
        event_t *prev = cpu->event_queue;                                       \
        while (prev->next && prev->next->fire_cycle <= cycle)                  \
            prev = prev->next;                                                  \
        ev->next = prev->next;                                                  \
        prev->next = ev;                                                        \
    }                                                                           \
    cpu->next_event_cycle = cpu->event_queue->fire_cycle;                       \
}                                                                               \
                                                                                \
void PREFIX##_schedule_event_ns(cpu_t *cpu, event_t *ev, int64_t fire_ns) {    \
    PREFIX##_cancel_event(cpu, ev);                                             \
                                                                                \
    ev->fire_ns = fire_ns;                                                      \
    int64_t delta_ns = fire_ns - cpu->sim_time_ns;                             \
    if (delta_ns < 0) delta_ns = 0;                                            \
    ev->fire_cycle = cpu->cycles +                                              \
        cpu_ns_to_cycles(delta_ns, cpu->cpu_freq_hz);                          \
    ev->next = NULL;                                                            \
                                                                                \
    int64_t cycle = ev->fire_cycle;                                            \
    if (cpu->event_queue == NULL || cycle < cpu->event_queue->fire_cycle) {     \
        ev->next = cpu->event_queue;                                            \
        cpu->event_queue = ev;                                                  \
    } else {                                                                    \
        event_t *prev = cpu->event_queue;                                       \
        while (prev->next && prev->next->fire_cycle <= cycle)                  \
            prev = prev->next;                                                  \
        ev->next = prev->next;                                                  \
        prev->next = ev;                                                        \
    }                                                                           \
    cpu->next_event_cycle = cpu->event_queue->fire_cycle;                       \
}                                                                               \
                                                                                \
void PREFIX##_cancel_event(cpu_t *cpu, event_t *ev) {                          \
    if (cpu->event_queue == ev) {                                               \
        cpu->event_queue = ev->next;                                            \
    } else {                                                                    \
        event_t *prev = cpu->event_queue;                                       \
        while (prev && prev->next != ev) prev = prev->next;                   \
        if (prev) prev->next = ev->next;                                       \
    }                                                                           \
    ev->next = NULL;                                                            \
    ev->fire_ns = 0;                                                            \
    cpu->next_event_cycle = cpu->event_queue ?                                  \
        cpu->event_queue->fire_cycle : INT64_MAX;                              \
}                                                                               \
                                                                                \
static void execute_events(cpu_t *cpu) {                                       \
    while (cpu->event_queue &&                                                  \
           cpu->cycles >= cpu->event_queue->fire_cycle) {                      \
        event_t *ev = cpu->event_queue;                                         \
        cpu->event_queue = ev->next;                                            \
        ev->next = NULL;                                                        \
        if (cpu->cpu_freq_hz > 0)                                              \
            cpu->sim_time_ns =                                                  \
                cpu_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);               \
        ev->fire_ns = 0;                                                        \
        ev->callback(ev->user_data, ev);                                       \
    }                                                                           \
    cpu->next_event_cycle = cpu->event_queue ?                                  \
        cpu->event_queue->fire_cycle : INT64_MAX;                              \
}                                                                               \
                                                                                \
void PREFIX##_cpu_set_frequency(cpu_t *cpu, uint32_t freq_hz) {                \
    if (freq_hz == 0) return;                                                   \
    if (cpu->cpu_freq_hz > 0)                                                  \
        cpu->sim_time_ns =                                                      \
            cpu_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);                   \
    cpu->cpu_freq_hz = freq_hz;                                                \
                                                                                \
    /* Recompute fire_cycle for all ns-based events */                         \
    event_t *ev = cpu->event_queue;                                             \
    while (ev) {                                                                \
        if (ev->fire_ns > 0) {                                                 \
            int64_t delta_ns = ev->fire_ns - cpu->sim_time_ns;                \
            if (delta_ns < 0) delta_ns = 0;                                    \
            ev->fire_cycle = cpu->cycles +                                      \
                cpu_ns_to_cycles(delta_ns, freq_hz);                           \
        }                                                                       \
        ev = ev->next;                                                          \
    }                                                                           \
                                                                                \
    /* Re-sort event queue (insertion sort rebuild) */                          \
    event_t *old_queue = cpu->event_queue;                                      \
    cpu->event_queue = NULL;                                                    \
    while (old_queue) {                                                         \
        event_t *e = old_queue;                                                 \
        old_queue = e->next;                                                    \
        e->next = NULL;                                                         \
        int64_t cycle = e->fire_cycle;                                         \
        if (cpu->event_queue == NULL ||                                         \
            cycle < cpu->event_queue->fire_cycle) {                            \
            e->next = cpu->event_queue;                                         \
            cpu->event_queue = e;                                               \
        } else {                                                                \
            event_t *prev = cpu->event_queue;                                   \
            while (prev->next && prev->next->fire_cycle <= cycle)              \
                prev = prev->next;                                              \
            e->next = prev->next;                                               \
            prev->next = e;                                                     \
        }                                                                       \
    }                                                                           \
    cpu->next_event_cycle = cpu->event_queue ?                                  \
        cpu->event_queue->fire_cycle : INT64_MAX;                              \
}

#endif /* EVENT_QUEUE_H */
