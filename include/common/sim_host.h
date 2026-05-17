/*
 * sim_host_t — CPU-agnostic vtable for off-SoC chip drivers.
 *
 * Off-SoC peripherals (CC2420 today, more later) need to talk to the
 * CPU and GPIO of the node they're attached to. Hard-coding MSP430 or
 * ARM types into the chip driver prevents the same chip from being
 * reused across architectures (e.g., dropping a CC2420 onto a CC2538).
 *
 * sim_host_t is a thin function-pointer table the platform fills in
 * once at init. The chip driver only sees this table.
 *
 * Required operations:
 *   - now_ns:        read current simulation time (for scheduling)
 *   - schedule_ns:   schedule a cpu_event_t at a wall-clock ns time
 *   - cancel:        remove an event from the queue
 *   - set_input_pin: drive a GPIO input pin (e.g. FIFO/FIFOP/SFD/CCA)
 *   - force_irq_edge:auto-enable IE and pulse IFG on a pin (legacy
 *                    Cooja firmware needs this for FIFOP — abstracts
 *                    direct port-state pokes that previously lived in
 *                    cc2420.c).
 */
#ifndef SIM_HOST_H
#define SIM_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu_event.h"

/* Convenience accessors for any chip driver whose state struct has a
 * `host` field of type `const sim_host_t *`.  The argument `c` is the
 * chip struct (cc2420_t, cc1200_t, …); the macros expand to the right
 * vtable calls.  Hoisted here so off-SoC chip drivers don't each
 * re-define the same five wrappers. */
#define HOST_NOW_NS(c)             ((c)->host->now_ns((c)->host->cpu))
#define HOST_SCHEDULE_NS(c, ev, t) ((c)->host->schedule_ns((c)->host->cpu, (ev), (t)))
#define HOST_CANCEL(c, ev)         ((c)->host->cancel((c)->host->cpu, (ev)))
#define HOST_SET_PIN(c, p, n, v)   ((c)->host->set_input_pin((c)->host->gpio, (p), (n), (v)))
#define HOST_FORCE_IRQ(c, p, n, e) ((c)->host->force_irq_edge((c)->host->gpio, (p), (n), (e)))

typedef struct sim_host {
    /* Opaque handles bound at platform init */
    void *cpu;
    void *gpio;
    /* Opaque handle for the simulation harness (medium dispatcher).
     * The chip driver passes this back through radio_set_channel; the
     * harness uses it to look up which (node, radio_idx) the call came
     * from. Optional — leave NULL on platforms that don't model a
     * radio medium. */
    void *radio_user_data;

    /* Time */
    int64_t (*now_ns)        (void *cpu);

    /* Event queue */
    void    (*schedule_ns)   (void *cpu, cpu_event_t *ev, int64_t fire_ns);
    void    (*cancel)        (void *cpu, cpu_event_t *ev);

    /* GPIO bridging — chip drives input pin into MCU */
    void    (*set_input_pin) (void *gpio, int port, int pin, bool value);

    /* Force IE+IFG edge for legacy Cooja firmware (FIFOP rising/falling).
     * Implementations may no-op if the platform's GPIO already does the
     * right thing on edge transitions. */
    void    (*force_irq_edge)(void *gpio, int port, int pin, bool rising);

    /* Radio: push a chip-driver-observed channel change into the
     * simulation harness so the radio medium can route bytes per-radio.
     *   radio_idx: which radio slot on this node the chip occupies
     *              (0 for single-radio platforms; 0/1 on Firefly)
     *   channel:   band-local channel index (-1 = unknown)
     * Optional — chip drivers must NULL-check; harnesses that don't
     * model a medium leave this unwired. */
    void    (*radio_set_channel)(void *radio_user_data, int radio_idx, int channel);
} sim_host_t;

#endif /* SIM_HOST_H */
