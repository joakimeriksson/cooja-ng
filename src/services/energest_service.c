/*
 * energest_service — the energy estimator as a COMPILED-IN plugin.
 *
 * Linked into the binary and registered in the static registry
 * (sim_registry.c), so a config selects it by bare name —
 * "plugins": ["energest"] — with no .so to ship.  This is csim's analogue of
 * Cooja's built-in plugins (packet_sink / lossy_medium remain the out-of-tree
 * .so examples).  All energy logic lives in the shared energest_engine.
 *
 * Its UI sink is a direct kernel call (sim_runtime_ui_publish_panel) rather
 * than the csim_api vtable — a built-in may name kernel symbols.  log = NULL
 * routes the summary to stdout.
 */
#include "energest_engine.h"
#include "sim_runtime.h"    /* sim_runtime_ui_publish_panel */
#include "sim_service.h"

#include <stddef.h>         /* NULL */

static int energest_bi_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim; (void)cfg;
    energest_sink_t sink = {
        .publish = sim_runtime_ui_publish_panel,  /* same signature */
        .log     = NULL,                          /* → stdout */
    };
    energest_engine_t *e = energest_engine_create(sink);
    if (!e) return -1;
    *state = e;
    return 0;
}

static void energest_bi_on_event(sim_runtime_t *sim, void *state,
                                 const sim_observer_event_t *ev) {
    energest_engine_on_event((energest_engine_t *)state, sim, ev);
}

static void energest_bi_destroy(sim_runtime_t *sim, void *state) {
    (void)sim;
    energest_engine_report((energest_engine_t *)state);
    energest_engine_free((energest_engine_t *)state);
}

const sim_service_ops_t energest_service_ops = {
    .name     = "energest",
    .init     = energest_bi_init,
    .destroy  = energest_bi_destroy,
    .on_event = energest_bi_on_event,
};
