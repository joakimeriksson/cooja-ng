/*
 * sim_runtime_t — the deterministic simulation kernel container.
 *
 * Phase 1 milestone 1 of the refactor in docs/design/refactor-plan.md.
 * This is the first slice: the struct bundles five things that the runner
 * currently keeps as file-scope globals.  No scheduling semantics or call
 * sites change in this milestone — the runner still drives the loop, and
 * existing globals (current_sim_ns, sim_eq, radio_medium) are aliased to
 * fields here so old code keeps compiling.
 *
 * Subsequent milestones (§3.15 in the plan) move call sites onto the
 * accessors / wrappers defined here.
 */
#ifndef SIM_RUNTIME_H
#define SIM_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "radio_medium.h"
#include "sim_event_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Run/pause state.  Wired into the loop later — milestone 1 only defines it. */
typedef enum sim_run_state {
    SIM_RUN_STOPPED = 0,
    SIM_RUN_RUNNING,
    SIM_RUN_PAUSED,
    SIM_RUN_STOP_REQUESTED,
} sim_run_state_t;

typedef struct sim_runtime {
    /* The five Phase 1 milestone 1 fields, in the order the plan lists them. */
    int64_t            now_ns;        /* current simulation time (ns)        */
    int64_t            end_ns;        /* run-until horizon (ns), 0 = unset   */
    sim_run_state_t    run_state;     /* lifecycle state, see enum above     */
    sim_event_queue_t  event_queue;   /* unified event queue                 */
    radio_medium_t     radio_medium;  /* per-node radio routing/policy       */
} sim_runtime_t;

/* Zero-initialize the runtime; does NOT init the event queue or radio medium
 * — those have their own init functions called by the existing runner. */
void sim_runtime_init(sim_runtime_t *sim);

/* Release runtime-owned resources.  Safe to call on a zero-initialized
 * runtime. */
void sim_runtime_destroy(sim_runtime_t *sim);

/* Accessor for the current simulation time.  Milestone 2 makes this the
 * canonical reader; until then it just returns `sim->now_ns`. */
static inline int64_t sim_runtime_now_ns(const sim_runtime_t *sim) {
    return sim->now_ns;
}

#ifdef __cplusplus
}
#endif

#endif /* SIM_RUNTIME_H */
