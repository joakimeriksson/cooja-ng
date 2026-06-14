/*
 * Implementation of the timeline service — see include/sim/timeline_service.h.
 *
 * Phase 6 milestone 32 (§3.19).  Verbatim move of the runner's
 * timeline_observer_cb body onto the service vtable; the only change is
 * that the mote-index → node-id mapping comes through the runner-supplied
 * resolver instead of a direct nodes[] read.
 */
#include "timeline_service.h"
#include "sim_runtime.h"

#include <stddef.h>

static int timeline_service_init(sim_runtime_t *sim, const void *cfg,
                                 void **state) {
    (void)sim;
    timeline_service_t *svc = (timeline_service_t *)cfg;
    if (!svc) return -1;
    tl_init(&svc->tl);
    *state = svc;
    return 0;
}

/* Translate each kernel-emitted observer event into the equivalent
 * tl_*_event() call.  Radio-state-tracking sites (update_radio_state,
 * mixed_rf_state_handler) still write the timeline directly because they
 * emit chip-state transitions (ON/OFF/INTF/TX/RX) that don't map 1:1 to
 * the START/END observer kinds. */
static void timeline_service_on_event(sim_runtime_t *sim, void *state,
                                      const sim_observer_event_t *ev) {
    (void)sim;
    timeline_service_t *svc = (timeline_service_t *)state;
    int node_id = svc->node_id ? svc->node_id(ev->mote_index) : -1;
    if (node_id < 0) return;
    timeline_t *tl = &svc->tl;
    switch (ev->kind) {
    case SIM_OBS_RADIO_TX_START:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_TX);   break;
    case SIM_OBS_RADIO_TX_END:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_ON);   break;
    case SIM_OBS_RADIO_RX_START:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_RX);   break;
    case SIM_OBS_RADIO_RX_END:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_ON);   break;
    case SIM_OBS_RADIO_INTERFERENCE:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_INTF); break;
    case SIM_OBS_LED_CHANGED:
        tl_led_event(tl, node_id, ev->time_ns,
                     ev->u.led.led_index, ev->u.led.on ? 1 : 0);  break;
    case SIM_OBS_PACKET_FRAME:
        tl_frame_event(tl, node_id, ev->time_ns,
                       ev->u.frame.is_tx ? 1 : 0,
                       ev->u.frame.summary);                       break;
    default:
        break;
    }
}

const sim_service_ops_t timeline_service_ops = {
    .name     = "timeline",
    .init     = timeline_service_init,
    .on_event = timeline_service_on_event,
};
