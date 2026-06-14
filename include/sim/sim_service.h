/*
 * sim_service — the kernel service host (Phase 6 M31).
 *
 * Phase 6 of docs/design/refactor-plan.md (§3.19) extracts the runner's
 * embedded optional/observation features (timeline, PCAP, packet analyzer,
 * progress, JSON/JS test engines, GDB, WebSocket UI, serial bridge,
 * external command) into self-contained services behind this vtable.
 *
 * A service is a `sim_service_ops_t` + an opaque `void *state`.  The host
 * keeps an ordered `{ops, state, enabled}` table inside `sim_runtime_t`
 * and provides:
 *   - sim_service_attach()      — register + init a service
 *   - sim_service_poll_all()    — tick every enabled service (loop cadence)
 *   - sim_service_dispatch_event() — the ONE fan-out observer the host
 *                                  subscribes; it walks services' on_event
 *                                  so each service need not consume its own
 *                                  observer slot
 *   - sim_service_destroy_all() — tear down in strict reverse order
 *
 * Error policy (§Decisions Log): init failure is reported to the caller
 * (attach returns < 0) — the caller decides whether to refuse to start or
 * warn-and-continue per the service's importance.  A service that hits a
 * runtime failure calls sim_service_disable() (or the host marks it) and is
 * skipped by subsequent poll/dispatch; the run continues.  destroy runs for
 * every attached service regardless of `enabled`.
 *
 * The same contracts as raw observers apply to on_event (§sim_observer.h):
 * do not re-enter dispatch (use sim_runtime_request_stop()), do not block,
 * do not retain the event pointer.
 */
#ifndef SIM_SERVICE_H
#define SIM_SERVICE_H

#include "sim_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration — sim_runtime.h completes the type (C11 permits the
 * redefinition of this typedef with the same underlying struct). */
typedef struct sim_runtime sim_runtime_t;

/* A service implements some subset of these; NULL members are skipped. */
typedef struct sim_service_ops {
    const char *name;
    /* Construct service state.  `cfg` is service-specific (may be the
     * already-built state object to adopt).  Store the live state through
     * `*state`.  Return 0 on success, < 0 to signal init failure. */
    int  (*init)(sim_runtime_t *sim, const void *cfg, void **state);
    /* Release service state.  Runs in reverse attach order at teardown. */
    void (*destroy)(sim_runtime_t *sim, void *state);
    /* Observe one kernel event (fan-out from the host's single observer). */
    void (*on_event)(sim_runtime_t *sim, void *state,
                     const sim_observer_event_t *ev);
    /* One service tick at the loop's cadence (socket I/O, periodic work). */
    void (*poll)(sim_runtime_t *sim, void *state);
} sim_service_ops_t;

/* One service table slot.  Defined here so sim_runtime_t can embed the
 * array; the host logic lives in sim_service.c. */
typedef struct sim_service_slot {
    const sim_service_ops_t *ops;
    void *state;
    bool  enabled;
} sim_service_slot_t;

#define SIM_RUNTIME_MAX_SERVICES 16

/* Register and initialize a service.  Calls ops->init(sim, cfg, &state) if
 * present (state defaults to (void*)cfg otherwise — the "adopt an existing
 * object" shorthand).  On the first attach of a service with an on_event,
 * the host subscribes its single fan-out observer.  Returns the service
 * index ≥ 0, or < 0 on a full table or an init failure (nothing is
 * registered in that case). */
int  sim_service_attach(sim_runtime_t *sim, const sim_service_ops_t *ops,
                        const void *cfg);

/* Tick every enabled service that has a poll(), in attach order. */
void sim_service_poll_all(sim_runtime_t *sim);

/* Mark the service owning `state` disabled (runtime-failure path); it is
 * skipped by subsequent poll/dispatch but still destroyed at teardown. */
void sim_service_disable(sim_runtime_t *sim, void *state);

/* Destroy every attached service in strict reverse attach order, then
 * clear the table and drop the fan-out observer.  Safe to call twice. */
void sim_service_destroy_all(sim_runtime_t *sim);

/* The host's single fan-out observer callback (subscribed automatically by
 * sim_service_attach).  Walks enabled services calling on_event.  `user`
 * is the sim_runtime_t*.  Exposed for the runtime's subscribe call. */
void sim_service_dispatch_event(void *user, const sim_observer_event_t *ev);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SERVICE_H */
