/*
 * Implementation of the JS test-engine line-feed service — see
 * include/sim/js_test_service.h.
 *
 * Phase 6 milestone 36 (§3.19).  Verbatim move of the runner's
 * test_engine_observer JS feed onto the service vtable.
 */
#include "js_test_service.h"
#include "sim_runtime.h"

#include <stddef.h>

void js_test_service_start(js_test_service_t *svc, js_test_engine_t *engine) {
    svc->engine = engine;
    svc->active = (engine != NULL);
}

static int js_test_svc_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim;
    *state = (void *)cfg;   /* runner-owned js_test_service_t */
    return 0;
}

static void js_test_svc_on_event(sim_runtime_t *sim, void *state,
                                 const sim_observer_event_t *ev) {
    (void)sim;
    if (ev->kind != SIM_OBS_MOTE_LOG_LINE) return;
    js_test_service_t *svc = (js_test_service_t *)state;
    if (svc->active && svc->engine)
        js_test_feed_line(svc->engine, ev->u.log_line.line,
                          ev->u.log_line.node_id, ev->time_ns / 1000);
}

const sim_service_ops_t js_test_service_ops = {
    .name     = "js-test",
    .init     = js_test_svc_init,
    .on_event = js_test_svc_on_event,
};
