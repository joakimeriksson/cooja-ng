/*
 * js_app_mote — boot policy + mote/radio adapter ops for the QuickJS
 * application mote kind (Phase 4, M19 — docs/design/refactor-plan.md
 * §3.17).
 *
 * Bodies are character-identical moves of the former runner adapters
 * (test/test_mixed_multinode.c), with the enumerated seam
 * substitutions: MOTE_IDX(m) → MOTE_IMPL(m)->slot, &sim_rt →
 * node->env->sim, runner callbacks → env members.  Stats moved to the
 * receive_frame call sites (return value carries queue-full).
 */
#include "mote_impl.h"

#include <stdio.h>

/* ============================================================
 * Boot policy
 * ============================================================ */

int js_app_mote_boot(mixed_node_t *node, int slot, const char *script_path,
                     int node_id, const sim_mote_env_t *env) {
    (void)slot;
    js_node_t *jn = &node->plat.js;

    if (js_node_init(jn, script_path, node_id) != 0)
        return -1;

    jn->log_callback        = env->uart_byte;
    jn->log_callback_data   = node;
    jn->rf_frame_callback   = env->js_rf_frame;
    jn->rf_frame_callback_data = node;

    /* Now safe to run init() — log/RF callbacks are wired. */
    js_node_start(jn);

    printf("  Node %d [JS] initialized\n", node_id);
    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

/* JS motes: BATCH delivery so the bus stages bytes in rf_pending[] like
 * before M9.4; receive_byte is a stub because JS motes consume RF at
 * frame level (the runner's rf_frame path), never via byte delivery. */
static void js_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    (void)m; (void)byte; (void)rssi;
}
static int js_radio_rxfifo_available(void *m) { (void)m; return 0; }
static bool js_radio_rx_busy(void *m) { (void)m; return false; }
static const mote_radio_ops_t js_radio_ops = {
    js_radio_receive_byte, js_radio_rxfifo_available,
    js_radio_rx_busy, NULL /* rx_stall */, NULL /* current_channel */, NULL /* mark_collisions */
};

void js_app_mote_register_radio(mixed_node_t *node, int slot,
                                sim_radio_bus_t *bus) {
    sim_radio_bus_register(bus, slot, &js_radio_ops, node,
                           SIM_RADIO_DELIVERY_BATCH, /*caps=*/0);
}

/* ============================================================
 * Mote ops (full table — JS has no emulated-CPU entanglements)
 * ============================================================ */

/* JS app motes (same pseudo-cycle convention as native:
 * 1 cycle = 1 µs, 1 MHz pseudo-freq) */
static int64_t js_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.js.sim_time_ns;
}
static int64_t js_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.js.sim_time_ns / 1000LL;
}
static uint32_t js_mote_freq_hz(const sim_mote_t *m) {
    (void)m;
    return 1000000; /* 1 MHz pseudo-freq (1 cycle = 1 us) */
}
static int64_t js_mote_instructions(const sim_mote_t *m) {
    (void)m;
    return 0; /* not tracked */
}

static int64_t js_mote_execute(sim_mote_t *m, int64_t now_ns) {
    js_node_t *jn = &MOTE_IMPL(m)->plat.js;
    /* Run the JS node up to event time; this fires execute() and
     * dispatches any RX frames scheduled at <= now_ns. */
    js_node_step_until_ns(jn, now_ns);
    return js_node_next_wakeup_ns(jn);  /* INT64_MAX = no known wakeup */
}

static void js_mote_step_until(sim_mote_t *m, int64_t target) {
    js_node_step_until_ns(&MOTE_IMPL(m)->plat.js, target);
}

/* JS motes have no console input — swallow so the bridge ring drains. */
static int js_mote_serial_input(sim_mote_t *m, const uint8_t *buf,
                                int len) {
    (void)m; (void)buf;
    return len;
}

static void js_mote_destroy(sim_mote_t *m) {
    js_node_destroy(&MOTE_IMPL(m)->plat.js);
}

static void js_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    MOTE_IMPL(m)->plat.js.sim_time_ns = now_ns;
}

static void *js_mote_get_interface(sim_mote_t *m, int iface) {
    (void)m; (void)iface;
    return NULL;
}

static int js_mote_receive_frame(sim_mote_t *m, const uint8_t *frame,
                                 int len, int64_t now_ns, int sender_idx) {
    (void)sender_idx;
    mixed_node_t *node = MOTE_IMPL(m);
    js_node_deliver_frame(&node->plat.js, frame, len, now_ns);
    sim_schedule_mote_wakeup_if_earlier(node->env->sim, node->slot, now_ns);
    return 0;
}

const sim_mote_ops_t js_app_mote_ops = {
    .kind            = "JS",
    .sim_time_ns     = js_mote_sim_time_ns,
    .cycles          = js_mote_cycles,
    .freq_hz         = js_mote_freq_hz,
    .instructions    = js_mote_instructions,
    .execute         = js_mote_execute,
    .step_until      = js_mote_step_until,
    .sched_hint_ns   = NULL, /* emulated motes only */
    .sync_to_time    = NULL, /* emulated motes only */
    .serial_input    = js_mote_serial_input,
    .destroy         = js_mote_destroy,
    .reset_time      = js_mote_reset_time,
    .ui_radio_state  = NULL,
    .ui_leds         = NULL,
    .get_interface   = js_mote_get_interface,
    .receive_frame   = js_mote_receive_frame,
};
