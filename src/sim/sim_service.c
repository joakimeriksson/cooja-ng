/*
 * Implementation of the kernel service host — see include/sim/sim_service.h.
 *
 * Phase 6 milestone 31 (§3.19).  Behavior-preserving introduction of the
 * service table: the host owns ordered attach/poll/teardown and a single
 * fan-out observer.  M31 adopts the already-extracted serial-bridge and
 * external-command services as its first two clients; M32+ extract the
 * remaining runner-embedded features onto this vtable.
 */
#include "sim_service.h"
#include "sim_runtime.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int sim_service_attach(sim_runtime_t *sim, const sim_service_ops_t *ops,
                       const void *cfg) {
    if (!sim || !ops) return -1;
    if (sim->service_count >= SIM_RUNTIME_MAX_SERVICES) return -1;

    /* Init constructs the live state.  The shorthand for "adopt an
     * already-built object" is a NULL init with cfg as the state. */
    void *state = (void *)cfg;
    if (ops->init) {
        state = NULL;
        if (ops->init(sim, cfg, &state) != 0)
            return -1;  /* init failure: nothing registered (caller's policy) */
    }

    int idx = sim->service_count++;
    sim->services[idx].ops     = ops;
    sim->services[idx].state   = state;
    sim->services[idx].enabled = true;

    /* Subscribe the host's single fan-out observer lazily, the first time a
     * service that wants events is attached — so event-free services (e.g.
     * the serial bridge, which runs its own observer internally) don't
     * consume an observer slot. */
    if (ops->on_event && sim->service_observer_handle < 0)
        sim->service_observer_handle =
            sim_runtime_subscribe(sim, sim_service_dispatch_event, sim);

    return idx;
}

void sim_service_poll_all(sim_runtime_t *sim) {
    if (!sim) return;
    for (int i = 0; i < sim->service_count; i++) {
        sim_service_slot_t *s = &sim->services[i];
        if (s->enabled && s->ops->poll)
            s->ops->poll(sim, s->state);
    }
}

void sim_service_disable(sim_runtime_t *sim, void *state) {
    if (!sim) return;
    for (int i = 0; i < sim->service_count; i++) {
        if (sim->services[i].state == state) {
            sim->services[i].enabled = false;
            return;
        }
    }
}

void sim_service_dispatch_event(void *user, const sim_observer_event_t *ev) {
    sim_runtime_t *sim = (sim_runtime_t *)user;
    if (!sim || !ev) return;
    /* Re-entrancy guard (M36): a service on_event must not emit an observer
     * event (which would re-enter this fan-out) or otherwise re-enter
     * dispatch — it observes only, deferring sim-mutating work to its
     * poll() or the runner loop.  A non-zero depth here means some service
     * violated that contract. */
    static int dispatch_depth = 0;
    assert(dispatch_depth == 0 && "service on_event re-entered dispatch");
    if (dispatch_depth != 0) {
        /* Release builds (-DNDEBUG) compile the assert away — enforce the
         * invariant for real: drop the re-entrant dispatch instead of
         * recursively fanning out. */
        fprintf(stderr, "sim_service: on_event re-entered dispatch — dropped\n");
        return;
    }
    dispatch_depth++;
    /* Snapshot the count so a service attached mid-dispatch isn't called
     * for the event that triggered it (mirrors sim_runtime_emit). */
    int count = sim->service_count;
    for (int i = 0; i < count; i++) {
        sim_service_slot_t *s = &sim->services[i];
        if (s->enabled && s->ops->on_event)
            s->ops->on_event(sim, s->state, ev);
    }
    dispatch_depth--;
}

void sim_service_destroy_all(sim_runtime_t *sim) {
    if (!sim) return;
    /* Strict reverse attach order; destroy runs regardless of `enabled`. */
    for (int i = sim->service_count - 1; i >= 0; i--) {
        sim_service_slot_t *s = &sim->services[i];
        if (s->ops && s->ops->destroy)
            s->ops->destroy(sim, s->state);
        s->ops = NULL;
        s->state = NULL;
        s->enabled = false;
    }
    sim->service_count = 0;
    if (sim->service_observer_handle >= 0) {
        sim_runtime_unsubscribe(sim, sim->service_observer_handle);
        sim->service_observer_handle = -1;
    }
}
