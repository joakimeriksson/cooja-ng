/*
 * external_mote — boot policy + mote/radio adapter ops for external
 * data-driven motes: the node's behaviour comes from a separate process
 * speaking NDJSON over stdio in simulation time.
 *
 * Design: docs/design/external-nodes-plan.md.  Engine: src/native/ext_node.c.
 *
 * Deliberately the js_app_mote.c shape — the two kinds have the same
 * relationship to the kernel (frame-level radio, pseudo-cycle clock, no
 * emulated-CPU entanglements), so they should read the same.
 *
 * Transmits, logs, and receives: a delivered frame reaches the peer as an
 * `rx` input on its next step, carrying the sender's node id and channel and
 * the per-receiver RSSI the medium computed.
 */
#include "mote_impl.h"

#include <stdio.h>

/* ============================================================
 * Boot policy
 * ============================================================ */

int external_mote_boot(mixed_node_t *node, int slot, const char *path,
                       int node_id, const sim_mote_env_t *env) {
    (void)slot;
    ext_node_t *en = &node->plat.ext;

    if (ext_node_init(en, path, node_id) != 0)
        return -1;

    en->log_callback           = env->uart_byte;
    en->log_callback_data      = node;
    /* The runner's frame hook is not JS-specific: it takes (node, frame,
     * len), casts to mixed_node_t and never reads ->type.  Reusing it is
     * what keeps this kind free of runner changes. */
    en->rf_frame_callback      = env->js_rf_frame;
    en->rf_frame_callback_data = node;

    /* Only now is it safe to hand the peer `hello` — its reply may already
     * carry log lines and a first transmission. */
    double x = 0.0, y = 0.0;
    radio_medium_node_pos(&env->sim->radio_medium, slot, &x, &y);
    if (ext_node_start(en, x, y, (uint32_t)node_id) != 0)
        return -1;

    printf("  Node %d [EXT] initialized (%s)\n", node_id, path);
    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

/* BATCH delivery, like JS motes: an external node consumes RF at frame
 * level, never per byte, so the byte entries are stubs. */
static void ext_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    (void)m; (void)byte; (void)rssi;
}
static int  ext_radio_rxfifo_available(void *m) { (void)m; return 0; }
static bool ext_radio_rx_busy(void *m) { (void)m; return false; }
static const mote_radio_ops_t ext_radio_ops = {
    ext_radio_receive_byte, ext_radio_rxfifo_available,
    ext_radio_rx_busy, NULL /* rx_stall */, NULL /* current_channel */,
    NULL /* mark_collisions */
};

void external_mote_register_radio(mixed_node_t *node, int slot,
                                  sim_radio_bus_t *bus) {
    sim_radio_bus_register(bus, slot, &ext_radio_ops, node,
                           SIM_RADIO_DELIVERY_BATCH,
                           SIM_RADIO_CAP_FRAME_CONSUMER);
}

/* ============================================================
 * Mote ops
 * ============================================================ */

/* Same pseudo-cycle convention as native/JS motes: 1 cycle = 1 µs. */
static int64_t ext_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.ext.sim_time_ns;
}
static int64_t ext_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.ext.sim_time_ns / 1000LL;
}
static uint32_t ext_mote_freq_hz(const sim_mote_t *m) {
    (void)m;
    return 1000000; /* 1 MHz pseudo-freq */
}
static int64_t ext_mote_instructions(const sim_mote_t *m) {
    (void)m;
    return 0; /* not tracked */
}

static int64_t ext_mote_execute(sim_mote_t *m, int64_t now_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    ext_node_t *en = &node->plat.ext;
    ext_node_step_until_ns(en, now_ns);
    /* A peer that died mid-run leaves a node with no behaviour at all.
     * Letting the simulation continue would quietly turn that into a
     * passing test, so stop the run — boot-time failures already abort
     * through the boot return. */
    if (ext_node_failed(en))
        sim_runtime_request_stop(node->env->sim);
    return ext_node_next_wakeup_ns(en);  /* INT64_MAX = no known wakeup */
}

static void ext_mote_step_until(sim_mote_t *m, int64_t target) {
    ext_node_step_until_ns(&MOTE_IMPL(m)->plat.ext, target);
}

/* No console input into the peer yet (the `serial` input arrives with M1);
 * swallow so the serial-bridge ring drains, as JS motes do. */
static int ext_mote_serial_input(sim_mote_t *m, const uint8_t *buf, int len) {
    (void)m; (void)buf;
    return len;
}

static void ext_mote_destroy(sim_mote_t *m) {
    ext_node_destroy(&MOTE_IMPL(m)->plat.ext);
}

static void ext_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    MOTE_IMPL(m)->plat.ext.sim_time_ns = now_ns;
}

static void *ext_mote_get_interface(sim_mote_t *m, int iface) {
    (void)m; (void)iface;
    return NULL;
}

/* A frame cleared the medium's filter for this receiver.  Queue it with the
 * context the peer cannot work out for itself -- who sent it, on what
 * channel, and how strong it was *here* -- then make sure the mote runs at
 * the arrival time so the peer sees it promptly.
 *
 * RSSI is per-receiver, so it has to be asked of the medium rather than
 * carried on the frame: two nodes at different distances hear the same
 * transmission at different strengths. */
static int ext_mote_receive_frame(sim_mote_t *m, const uint8_t *frame,
                                  int len, int64_t now_ns, int sender_idx) {
    mixed_node_t *node = MOTE_IMPL(m);
    radio_medium_t *rm = &node->env->sim->radio_medium;

    int from_id = -1;
    sim_mote_t *sender = sim_runtime_mote(node->env->sim, sender_idx);
    if (sender) from_id = MOTE_IMPL(sender)->id;

    int channel = -1;
    if (sender_idx >= 0 && sender_idx < RADIO_MEDIUM_MAX_NODES)
        channel = rm->nodes[sender_idx].radios[0].channel;

    int8_t rssi = radio_medium_get_rssi(rm, sender_idx, node->slot);

    int rc = ext_node_deliver_frame(&node->plat.ext, frame, len, now_ns,
                                    from_id, channel, rssi);
    sim_schedule_mote_wakeup_if_earlier(node->env->sim, node->slot, now_ns);
    return rc;
}

const sim_mote_ops_t external_mote_ops = {
    .kind            = "EXT",
    .sim_time_ns     = ext_mote_sim_time_ns,
    .cycles          = ext_mote_cycles,
    .freq_hz         = ext_mote_freq_hz,
    .instructions    = ext_mote_instructions,
    .execute         = ext_mote_execute,
    .step_until      = ext_mote_step_until,
    .sched_hint_ns   = NULL, /* emulated motes only */
    .sync_to_time    = NULL, /* emulated motes only */
    .serial_input    = ext_mote_serial_input,
    .destroy         = ext_mote_destroy,
    .reset_time      = ext_mote_reset_time,
    .ui_radio_state  = NULL,
    .ui_leds         = NULL,
    .get_interface   = ext_mote_get_interface,
    .receive_frame   = ext_mote_receive_frame,
};
