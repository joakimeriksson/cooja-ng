/*
 * renode_mote — the csim node that is Renode's radio presence.
 *
 * One node of this kind exists per run when Renode co-simulates with csim.
 * It has no CPU and no clock of its own: everything it does is driven by
 * bus accesses arriving from Renode, which the co-simulation service
 * services at the quantum boundary (src/services/renode_cosim_service.c).
 * What this module provides is the node's *place in the simulation*: a
 * position in the medium, a channel, and the frame in/out path.
 *
 * Deliberately the external_mote.c shape — the two kinds have the same
 * relationship to the kernel (frame-level radio, no emulated-CPU
 * entanglement), so they should read the same.  The difference is where
 * the behaviour lives: an external mote drives itself from a peer process
 * in simulation time, this one is a passive device another simulator
 * drives.
 *
 * Design: docs/design/renode-cosim-plan.md.  Device model:
 * src/native/renode_dev.c.
 */
#include "mote_impl.h"

#include <stdio.h>

/* ============================================================
 * Device hooks — the only path from a bus access into the kernel
 * ============================================================ */

static int64_t renode_hook_now_ns(void *user) {
    mixed_node_t *node = (mixed_node_t *)user;
    return sim_runtime_now_ns(node->env->sim);
}

/* A frame the guest handed us goes on the air at the current kernel time.
 * The service pins now_ns to the quantum's horizon before servicing bus
 * accesses, so "now" here is the tick boundary Renode is at — not the
 * time of the last event csim happened to dispatch. */
static void renode_hook_tx(void *user, const uint8_t *frame, int len) {
    mixed_node_t *node = (mixed_node_t *)user;
    if (!node->env->ext_rf_frame_at)
        return;
    node->env->ext_rf_frame_at(node, frame, len,
                               sim_runtime_now_ns(node->env->sim));
}

static void renode_hook_set_channel(void *user, int channel) {
    mixed_node_t *node = (mixed_node_t *)user;
    if (node->env->radio_set_channel)
        node->env->radio_set_channel(node, 0, channel);
}

/* The medium scales range by indicator/max (Phase 12); the device's 5-bit
 * TXPOWER is the CC2420 PA_LEVEL scale, so max is 31. */
static void renode_hook_set_power(void *user, int indicator) {
    mixed_node_t *node = (mixed_node_t *)user;
    if (node->env->radio_set_power)
        node->env->radio_set_power(node, 0, indicator, 31);
}

/* Carrier sense, from the medium's own view of this node's position and
 * channel.  The runner's CCA is generic despite its name -- it walks the
 * TX-range neighbour list and reads the bus's per-sender medium-busy
 * deadline, with a cross-band check. */
static bool renode_hook_channel_busy(void *user) {
    mixed_node_t *node = (mixed_node_t *)user;
    if (!node->env->cc1200_channel_busy)
        return false;
    return node->env->cc1200_channel_busy(node);
}

/* Console bytes from Renode into a csim node.  Returns what the node took,
 * so the device can keep the rest and retry rather than lose it. */
static int renode_hook_uart_inject(void *user, int slot, const uint8_t *buf,
                                   int len) {
    mixed_node_t *node = (mixed_node_t *)user;
    sim_runtime_t *sim = node->env->sim;
    sim_mote_t *target = sim_runtime_mote(sim, slot);
    if (!target || !target->ops || !target->ops->serial_input)
        return 0;
    int n = target->ops->serial_input(target, buf, len);
    if (n > 0) {
        /* Make sure the node runs soon enough to consume them, the same way
         * the runner's one-shot console injection does. */
        sim_schedule_mote_wakeup_if_earlier(sim, slot,
                                            sim_runtime_now_ns(sim) + 1000000LL);
    }
    return n;
}

/* ============================================================
 * Boot policy
 * ============================================================ */

int renode_mote_boot(mixed_node_t *node, int slot, const char *path,
                     int node_id, const sim_mote_env_t *env) {
    (void)slot; (void)path;   /* the firmware path only picks the kind */
    renode_dev_t *dev = &node->plat.renode;

    renode_dev_init(dev);
    dev->user        = node;
    dev->now_ns      = renode_hook_now_ns;
    dev->tx          = renode_hook_tx;
    dev->set_channel = renode_hook_set_channel;
    dev->set_power   = renode_hook_set_power;
    dev->uart_inject = renode_hook_uart_inject;
    dev->channel_busy = renode_hook_channel_busy;
    dev->self_id     = node_id;
    dev->uart_node   = 0;

    (void)env;   /* hooks read node->env, stamped by init_node before boot */
    printf("  Node %d [RENODE] initialized (co-simulation device)\n", node_id);
    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

/* BATCH delivery, like external and JS motes: the device consumes RF at
 * frame level, never per byte, so the byte entries are stubs. */
static void renode_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    (void)m; (void)byte; (void)rssi;
}
static int  renode_radio_rxfifo_available(void *m) { (void)m; return 0; }
static bool renode_radio_rx_busy(void *m) { (void)m; return false; }

static const mote_radio_ops_t renode_radio_ops = {
    renode_radio_receive_byte, renode_radio_rxfifo_available,
    renode_radio_rx_busy, NULL /* rx_stall */, NULL /* current_channel */,
    NULL /* mark_collisions */
};

void renode_mote_register_radio(mixed_node_t *node, int slot,
                                sim_radio_bus_t *bus) {
    sim_radio_bus_register(bus, slot, &renode_radio_ops, node,
                           SIM_RADIO_DELIVERY_BATCH,
                           SIM_RADIO_CAP_FRAME_CONSUMER);
}

/* ============================================================
 * Mote ops
 * ============================================================ */

/* Same pseudo-cycle convention as native/JS/external motes: 1 cycle = 1 µs. */
static int64_t renode_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->last_execute_ns;
}
static int64_t renode_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->last_execute_ns / 1000LL;
}
static uint32_t renode_mote_freq_hz(const sim_mote_t *m) {
    (void)m;
    return 1000000; /* 1 MHz pseudo-freq */
}
static int64_t renode_mote_instructions(const sim_mote_t *m) {
    (void)m;
    return 0; /* no CPU here */
}

/* The device has no clock of its own: Renode decides when it acts, and the
 * service applies those actions between pumps.  Asking for no wakeup keeps
 * the node out of the event queue entirely, so a run with a Renode node in
 * it schedules exactly what it would without one. */
static int64_t renode_mote_execute(sim_mote_t *m, int64_t now_ns) {
    MOTE_IMPL(m)->last_execute_ns = now_ns;
    return INT64_MAX;
}

static void renode_mote_step_until(sim_mote_t *m, int64_t target) {
    MOTE_IMPL(m)->last_execute_ns = target;
}

/* Console input addressed at the device itself has nowhere to go — the
 * guest's console is Renode's, not ours.  Swallow so a serial-bridge ring
 * drains, as JS and external motes do. */
static int renode_mote_serial_input(sim_mote_t *m, const uint8_t *buf, int len) {
    (void)m; (void)buf;
    return len;
}

static void renode_mote_destroy(sim_mote_t *m) {
    (void)m;  /* the device owns no resources */
}

static void renode_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    MOTE_IMPL(m)->last_execute_ns = now_ns;
}

/* The co-simulation service reaches the device through this, so it never
 * needs mote_impl.h (which is private to src/motes and the runner). */
static void *renode_mote_get_interface(sim_mote_t *m, int iface) {
    if (iface == SIM_MOTE_IFACE_COSIM_DEV)
        return &MOTE_IMPL(m)->plat.renode;
    return NULL;
}

/* A frame cleared the medium's filter for this node.  Queue it with the
 * context the guest cannot work out for itself — who sent it, on what
 * channel, how strong it was *here*, and when it started on the air.
 *
 * RSSI is per-receiver, so it is asked of the medium rather than carried on
 * the frame: two nodes at different distances hear the same transmission at
 * different strengths. */
static int renode_mote_receive_frame(sim_mote_t *m, const uint8_t *frame,
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

    /* The frame's start on the air (the bus's accurate first-byte time for
     * an emulated sender that lags the kernel), so the guest can time its
     * reception and any acknowledgement from the frame's true end. */
    int64_t start_ns = node->env->radio_bus ? node->env->radio_bus->frame_start_ns
                                            : now_ns;
    return renode_dev_rx_push(&node->plat.renode, frame, len, start_ns,
                              from_id, channel, rssi);
}

const sim_mote_ops_t renode_mote_ops = {
    .kind            = "RENODE",
    .sim_time_ns     = renode_mote_sim_time_ns,
    .cycles          = renode_mote_cycles,
    .freq_hz         = renode_mote_freq_hz,
    .instructions    = renode_mote_instructions,
    .execute         = renode_mote_execute,
    .step_until      = renode_mote_step_until,
    .sched_hint_ns   = NULL, /* emulated motes only */
    .sync_to_time    = NULL, /* emulated motes only */
    .serial_input    = renode_mote_serial_input,
    .destroy         = renode_mote_destroy,
    .reset_time      = renode_mote_reset_time,
    .ui_radio_state  = NULL,
    .ui_leds         = NULL,
    .get_interface   = renode_mote_get_interface,
    .receive_frame   = renode_mote_receive_frame,
};
