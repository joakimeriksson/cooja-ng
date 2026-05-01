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

typedef struct sim_host {
    /* Opaque handles bound at platform init */
    void *cpu;
    void *gpio;

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
} sim_host_t;

#endif /* SIM_HOST_H */
