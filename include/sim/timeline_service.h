/*
 * timeline_service — the activity-timeline observer as a kernel service.
 *
 * Phase 6 milestone 32 (§3.19).  Extracts the runner's timeline consumer
 * (ex `timeline_observer_cb`) onto the sim_service host: it subscribes to
 * the kernel observer stream (via the host fan-out) and translates each
 * radio/LED/packet event into the equivalent tl_*_event() call on its
 * owned `timeline_t`.
 *
 * The service owns the timeline data; the runner still writes a few
 * chip/UI-state-polling events directly (mixed_rf_state_handler,
 * update_radio_state) and the UI broadcast reads the timeline for CBOR
 * deltas — all through the runner-owned `timeline_service_t` instance
 * (`.tl`).  Those direct writers/readers move to the WebSocket-UI service
 * in M39; the per-node radio/LED `node_states[]` delta state stays
 * runner-side until then (the timeline consumer never touches it).
 */
#ifndef TIMELINE_SERVICE_H
#define TIMELINE_SERVICE_H

#include "sim_service.h"
#include "timeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Resolve a mote slot index to its Cooja node id; return < 0 for an
 * invalid/out-of-range slot (the observer event is then ignored).  The
 * runner supplies this so the service needs no access to nodes[]. */
typedef int (*timeline_node_id_fn)(int mote_index);

typedef struct timeline_service {
    timeline_t          tl;       /* the timeline data (service-owned)     */
    timeline_node_id_fn node_id;  /* set by the runner before attach       */
} timeline_service_t;

/* Service vtable — attach with `cfg` pointing at a runner-owned
 * timeline_service_t whose `node_id` is already set.  init runs tl_init()
 * on `.tl`; on_event consumes the radio/LED/frame observer kinds. */
extern const sim_service_ops_t timeline_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* TIMELINE_SERVICE_H */
