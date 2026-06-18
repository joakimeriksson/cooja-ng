/*
 * energest_engine — the host-agnostic core of the energy-estimation example.
 *
 * It integrates per-mote time-in-radio-state (off/listen/tx/rx/intf) and CPU
 * active/LPM time from the observer stream, builds the live UI panel JSON, and
 * prints the end-of-run summary.  It names no host symbol — all output goes
 * through the caller-supplied `energest_sink_t` (a publish callback + a log
 * callback), so the SAME logic can be driven either way:
 *   - compiled-in (src/services/energest_service.c) — sink → kernel call;
 *     registered as a built-in, selectable by config name "energest".
 *   - out-of-tree .so — a thin csim_plugin wrapper whose sink is the v3
 *     csim_api ui vtable (publish_panel) + host logger.  (Not shipped in-tree:
 *     the built-in already owns the "energest" name, and one feature = one
 *     provider; the engine/sink split is what makes that wrapper trivial.)
 */
#ifndef ENERGEST_ENGINE_H
#define ENERGEST_ENGINE_H

#include "sim_observer.h"   /* sim_observer_event_t */
#include "sim_service.h"    /* sim_runtime_t (opaque forward decl) */

#ifdef __cplusplus
extern "C" {
#endif

/* Where the engine sends output.  `publish` pushes the live UI panel
 * (sim, "energest", json) — may be NULL when no UI sink is available; its
 * signature matches both csim_ui_ops.publish_panel and
 * sim_runtime_ui_publish_panel.  `log` emits one preformatted summary line —
 * NULL means "write to stdout". */
typedef struct energest_sink {
    void (*publish)(sim_runtime_t *sim, const char *id, const char *json);
    void (*log)(const char *line);
} energest_sink_t;

typedef struct energest_engine energest_engine_t;

/* Create/free the engine.  create returns NULL on OOM. */
energest_engine_t *energest_engine_create(energest_sink_t sink);
void               energest_engine_free(energest_engine_t *e);

/* Feed one observer event (SIM_OBS_RADIO_STATE / SIM_OBS_CPU_STATE; others
 * ignored).  `sim` is passed through to sink.publish for the live panel. */
void energest_engine_on_event(energest_engine_t *e, sim_runtime_t *sim,
                              const sim_observer_event_t *ev);

/* Emit the end-of-run per-mote + network summary through sink.log. */
void energest_engine_report(energest_engine_t *e);

#ifdef __cplusplus
}
#endif

#endif /* ENERGEST_ENGINE_H */
