/*
 * js_test_service — the JavaScript (QuickJS) test engine's line consumer
 * as a kernel service.
 *
 * Phase 6 milestone 36 (§3.19).  Moves the JS-engine console-line feed
 * (ex test_engine_observer's JS half) onto the host fan-out as on_event;
 * with the JSON checker already extracted (M35), this lets the runner's
 * dedicated test_engine_observer go away entirely.
 *
 * Re-entrancy contract (§9 Phase 6): on_event only *feeds* the line to the
 * engine — js_test_feed_line matches patterns, advances the script, and
 * queues GENERATE_MSG/resume actions; it never injects serial, steps a
 * CPU, or re-enters dispatch.  The queued actions are drained and executed
 * in the runner's main loop (the deferred-resume point), not in the
 * callback.  The host's fan-out asserts non-re-entrancy (a service that
 * emitted inside on_event would trip it).
 *
 * What stays runner-side: the engine lifecycle (js_test_init, the
 * TIMEOUT/GENERATE_MSG total_ns + event scheduling, js_test_destroy), the
 * action drain+execute block (shared node scripting), the timeout poll,
 * and the "--- JS Test Results ---" print.  The service only owns the feed
 * + a handle to the runner-owned engine.
 */
#ifndef JS_TEST_SERVICE_H
#define JS_TEST_SERVICE_H

#include <stdbool.h>

#include "sim_service.h"
#include "js_test_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct js_test_service {
    js_test_engine_t *engine;   /* runner-owned; NULL = inactive */
    bool active;
} js_test_service_t;

/* Point the service at the (already-initialized) runner-owned engine. */
void js_test_service_start(js_test_service_t *svc, js_test_engine_t *engine);

/* Host vtable: init adopts the runner-owned service; on_event feeds each
 * SIM_OBS_MOTE_LOG_LINE to the engine. */
extern const sim_service_ops_t js_test_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* JS_TEST_SERVICE_H */
