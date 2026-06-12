/*
 * native_cooja_mote — boot policy + mote/radio adapter ops for the
 * native Cooja mote kind (dlopen'd Contiki shared library; Phase 4,
 * M20 — docs/design/refactor-plan.md §3.17).
 *
 * Bodies are character-identical moves of the former runner adapters
 * (test/test_mixed_multinode.c), with the enumerated seam
 * substitutions: slot-indexed helpers take the node pointer,
 * native_had_tx[]/native_exec_had_tx globals become node fields,
 * `verbose` reads *env->verbose, and the TSCH channel push goes
 * through env->native_channel_sync.  The yield callback stays in the
 * runner (cross-node ACK-turnaround policy) and arrives via
 * env->native_yield.
 */
#include "mote_impl.h"

#include <stdio.h>
#include <string.h>

/* ============================================================
 * Boot policy
 * ============================================================ */

int native_cooja_mote_boot(mixed_node_t *node, int slot,
                           const char *firmware_path, int node_id,
                           const sim_mote_env_t *env) {
    (void)slot;
    native_node_t *nat = &node->plat.native;

    if (native_node_init(nat, firmware_path, node_id) != 0)
        return -1;

    /* Set up UART/log callback */
    nat->log_callback = env->uart_byte;
    nat->log_callback_data = node;

    /* Set up RF callbacks: byte-stream for emulated, frame for native */
    nat->rf_tx_callback = env->rf_tx_byte;
    nat->rf_tx_callback_data = node;
    nat->rf_frame_callback = env->rf_frame;
    nat->rf_frame_callback_data = node;
    nat->yield_callback = env->native_yield;
    nat->yield_callback_data = node;

    /* Reset the RX assembler */
    native_rx_assembler_reset(&nat->rx_asm);

    printf("  Node %d [NATIVE] initialized\n", node_id);
    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

/* Native (Cooja-protocol) motes: SYNC delivery — the bus feeds each
 * accepted on-air byte to the mote's RX frame assembler.  rxfifo 0 / not
 * busy preserve the pre-M9.4 behaviour of the frame paths, which skip
 * natives explicitly. */
static void native_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    (void)rssi;
    mixed_node_t *node = (mixed_node_t *)m;
    native_rx_assembler_feed(&node->plat.native, byte);
}
static int native_radio_rxfifo_available(void *m) { (void)m; return 0; }
static bool native_radio_rx_busy(void *m) { (void)m; return false; }
static const mote_radio_ops_t native_radio_ops = {
    native_radio_receive_byte, native_radio_rxfifo_available,
    native_radio_rx_busy, NULL /* rx_stall */
};

void native_cooja_mote_register_radio(mixed_node_t *node, int slot,
                                      sim_radio_bus_t *bus) {
    sim_radio_bus_register(bus, slot, &native_radio_ops, node,
                           SIM_RADIO_DELIVERY_SYNC);
}

/* ============================================================
 * Tick helpers (ex runner tick_one_native & friends)
 * ============================================================ */

/* Tick a single native node at sim_ns. Single tick, no loop.
 * Matches COOJA's ContikiMote.execute().
 * node->native_had_tx tracks whether the last tick had a TX (for
 * distinguishing TX yield from TSCH busywait in the wakeup logic). */
static void tick_one_native(mixed_node_t *mnode, int64_t sim_ns) {
    native_node_t *nat = &mnode->plat.native;
    nat->sim_time_ns = sim_ns;
    *nat->simCurrentTime = (uint64_t)(sim_ns / 1000000LL);
    *nat->simRtimerCurrentTicks = (uint64_t)(sim_ns / 1000LL);

    if (nat->radio_is_transmitting && sim_ns >= nat->radio_tx_end_ns) {
        nat->radio_is_transmitting = false;
        nat->radio_tx_finished = true;
        *nat->simOutSize = 0;
        if (nat->simProcessRunValue) {
            *nat->simProcessRunValue = 1;
        }
    } else {
        nat->radio_tx_finished = false;
    }

    /* Deliver queued RX frame BEFORE tick.
     * Force simReceiving=0 so pending_packet() returns true.
     * Only dequeue when the radio is on — TSCH turns radio off between
     * slots, and doInterfaceActionsBeforeTick drops frames when off.
     * Keeping the frame in the queue lets it be delivered on a later
     * tick when the radio is in an active RX slot. */
    if (*nat->simReceiving)
        *nat->simReceiving = 0;
    bool radio_on = !nat->simRadioHWOn || *nat->simRadioHWOn;
    if (*nat->simInSize == 0 && nat->rx_queue.count > 0 && radio_on)
        native_dequeue_rx_frame(nat);
    int pre_insize = *nat->simInSize;
    nat->cooja_tick();
    mnode->native_had_tx = (*nat->simOutSize > 0);
    native_check_radio_tx(nat);
    native_check_log_output(nat);
    /* Reset signal strength when frame is consumed (signalReceptionEnd) */
    if (pre_insize > 0 && *nat->simInSize == 0) {
        if (nat->simSignalStrength)
            *nat->simSignalStrength = -100;
        if (*mnode->env->verbose)
            fprintf(stderr, "  [CONSUMED] node %d consumed %d-byte frame at %lld ms\n",
                    mnode->id, pre_insize, (long long)(nat->sim_time_ns / 1000000LL));
    }

    /* Sync this node's channel (for TSCH hopping) */
    if (nat->simRadioChannel)
        mnode->env->native_channel_sync(mnode->slot, *nat->simRadioChannel);
}

/* Compute a native node's next wakeup time after a tick.
 * Exactly matches COOJA's ContikiClock.doActionsAfterTick().  The caller
 * schedules the returned time with sim_schedule_mote_wakeup_if_earlier so
 * a requestImmediateWakeup already scheduled by RF delivery detection is
 * never overridden (M12: this is the native execute() op's return value). */
static int64_t native_next_wakeup_after_tick(mixed_node_t *mnode) {
    native_node_t *nat = &mnode->plat.native;
    int64_t now = nat->sim_time_ns;

    /* Determine next wakeup time (like ContikiClock.doActionsAfterTick). */
    int64_t next = now + 1000000000LL;  /* 1s max gap */

    /* rtimer has priority (exact µs timing) */
    if (*nat->simRtimerPending) {
        int64_t rt = (int64_t)(*nat->simRtimerNextExpirationTime) * 1000LL;
        if (rt <= now) rt = now;
        if (rt < next) next = rt;
    }

    /* ContikiRadio.doActionsAfterTick(): new transmissions keep the mote
     * scheduled until the exact transmission-end time, and completion
     * requests an immediate same-time wakeup. */
    if (nat->radio_tx_finished) {
        nat->radio_tx_finished = false;
        return now;
    }
    if (nat->radio_is_transmitting) {
        int64_t tx_next = nat->radio_tx_end_ns;
        if (tx_next < next) next = tx_next;
    } else if (mnode->native_had_tx) {
        int64_t tx_next = now + 1000000LL;
        if (tx_next < next) next = tx_next;
    }
    mnode->native_had_tx = false;

    /* processRunValue → +1ms (like COOJA's ContikiClock.doActionsAfterTick) */
    if (*nat->simProcessRunValue) {
        int64_t prv = now + 1000000LL;
        if (prv < next) next = prv;
    }

    /* etimer */
    if (*nat->simEtimerPending) {
        int64_t et = (int64_t)(*nat->simEtimerNextExpirationTime) * 1000000LL;
        if (et <= now) et = now + 1000000LL;  /* stale → +1ms */
        if (et < next) next = et;
    }

    /* If rx_queue has frames, schedule +1ms to ensure TSCH slot
     * operation gets a chance to find the frame via pending_packet(). */
    if (nat->rx_queue.count > 0 || *nat->simInSize > 0) {
        int64_t rx_next = now + 1000000LL;
        if (rx_next < next) next = rx_next;
    }

    return next;
}

static bool native_has_pending_work(native_node_t *node, int64_t sim_ns) {
    if (*node->simInSize > 0) return true;  /* frame ready for delivery */
    if (node->rx_queue.count > 0) return true;  /* queued frames waiting */
    if (*node->simProcessRunValue) return true;
    if (*node->simEtimerPending) {
        int64_t et_ns = (int64_t)(*node->simEtimerNextExpirationTime) * 1000000LL;
        if (et_ns <= sim_ns) return true;
    }
    if (*node->simRtimerPending) {
        int64_t rt_ns = (int64_t)(*node->simRtimerNextExpirationTime) * 1000LL;
        if (rt_ns <= sim_ns) return true;
    }
    return false;
}

/* ============================================================
 * Mote ops (full table — natives have no emulated-CPU entanglements;
 * the post-execute RF distribution stays in the runner's dispatcher,
 * which reads node->exec_had_tx)
 * ============================================================ */

/* Native Cooja motes (pseudo-cycles: 1 cycle = 1 µs, 1 MHz pseudo-freq) */
static int64_t native_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.native.sim_time_ns;
}
static int64_t native_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.native.sim_time_ns / 1000LL;
}
static uint32_t native_mote_freq_hz(const sim_mote_t *m) {
    (void)m;
    return 1000000; /* 1 MHz pseudo-freq (1 cycle = 1 us) */
}
static int64_t native_mote_instructions(const sim_mote_t *m) {
    (void)m;
    return 0; /* not tracked */
}

static int64_t native_mote_execute(sim_mote_t *m, int64_t now_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    /* Single tick at event time */
    tick_one_native(node, now_ns);
    /* Handoff to the dispatcher's post-tick RF distribution: whether
     * this tick transmitted (native_had_tx is cleared again inside
     * native_next_wakeup_after_tick, so the dispatcher cannot read it
     * after execute returns). */
    node->exec_had_tx = node->native_had_tx;

    /* Next wakeup (like ContikiClock.doActionsAfterTick) */
    return native_next_wakeup_after_tick(node);
}

static void native_mote_step_until(sim_mote_t *m, int64_t target) {
    native_step_until_ns(&MOTE_IMPL(m)->plat.native, target);
}

static void native_mote_advance_to_time(sim_mote_t *m, int64_t sim_ns) {
    native_node_t *nat = &MOTE_IMPL(m)->plat.native;
    if (!native_has_pending_work(nat, sim_ns)) {
        nat->sim_time_ns = sim_ns;
        return;
    }
    native_step_until_ns(nat, sim_ns);
    /* Process additional queued frames within this time step */
    while (*nat->simInSize == 0 && nat->rx_queue.count > 0 &&
           nat->sim_time_ns < sim_ns) {
        native_dequeue_rx_frame(nat);
        native_step_until_ns(nat, sim_ns);
    }
}

/* Native motes: append into the mote's pending serial buffer (the
 * cooja rs232 backend has a fixed 2048-byte receive buffer). */
static int native_mote_serial_input(sim_mote_t *m, const uint8_t *buf,
                                    int len) {
    mixed_node_t *node = MOTE_IMPL(m);
    int idx = node->slot;
    const sim_mote_env_t *env = node->env;
    native_node_t *nat = &node->plat.native;
    if (!nat->simSerialReceivingData ||
        !nat->simSerialReceivingLength ||
        !nat->simSerialReceivingFlag) return 0;

    /* Match COOJA ContikiRS232: append new bytes into the mote's pending
     * serial buffer even when simSerialReceivingFlag is already set, then
     * request an immediate wakeup so the mote drains that buffer. */
    int old_size = *nat->simSerialReceivingLength;
    if (old_size < 0) old_size = 0;
    if (old_size > 2048) old_size = 2048;
    int space = 2048 - old_size;
    if (space <= 0) {
        if (*env->num_threads == 0) {
            int64_t wake_ns = nat->sim_time_ns;
            if (env->node_start_ns[idx] > wake_ns)
                wake_ns = env->node_start_ns[idx];
            sim_schedule_mote_wakeup_if_earlier(env->sim, idx, wake_ns);
        }
        return 0;
    }

    int to_send = len;
    if (to_send > space) to_send = space;
    memcpy(nat->simSerialReceivingData + old_size, buf, (size_t)to_send);
    *nat->simSerialReceivingLength = old_size + to_send;
    *nat->simSerialReceivingFlag = 1;
    if (nat->simProcessRunValue)
        *nat->simProcessRunValue = 1;
    /* Match COOJA's external serial delivery contract: once serial data
     * is injected, the mote must be re-run at the current simulation
     * time rather than waiting for a stale timer-based wakeup. This is
     * the native equivalent of the immediate reschedule used for MSP430
     * serial injection. */
    if (*env->num_threads == 0) {
        int64_t wake_ns = nat->sim_time_ns;
        if (env->node_start_ns[idx] > wake_ns)
            wake_ns = env->node_start_ns[idx];
        sim_schedule_mote_wakeup_if_earlier(env->sim, idx, wake_ns);
    }
    return to_send;
}

static void native_mote_destroy(sim_mote_t *m) {
    native_node_destroy(&MOTE_IMPL(m)->plat.native);
}

static void native_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    MOTE_IMPL(m)->plat.native.sim_time_ns = now_ns;
}

static void *native_mote_get_interface(sim_mote_t *m, int iface) {
    (void)m; (void)iface;
    return NULL;
}

static int native_mote_receive_frame(sim_mote_t *m, const uint8_t *frame,
                                     int len, int64_t now_ns,
                                     int sender_idx) {
    native_node_t *nat = &MOTE_IMPL(m)->plat.native;
    int rc = (nat->rx_queue.count >= NATIVE_RX_QUEUE_SIZE) ? -1 : 0;
    native_deliver_frame(nat, frame, len, now_ns, sender_idx);
    return rc;  /* <0 = queue was full; caller owns the stats (M19) */
}

const sim_mote_ops_t native_cooja_mote_ops = {
    .kind            = "NATIVE",
    .sim_time_ns     = native_mote_sim_time_ns,
    .cycles          = native_mote_cycles,
    .freq_hz         = native_mote_freq_hz,
    .instructions    = native_mote_instructions,
    .execute         = native_mote_execute,
    .step_until      = native_mote_step_until,
    .advance_to_time = native_mote_advance_to_time,
    .sched_hint_ns   = NULL, /* emulated motes only */
    .sync_to_time    = NULL, /* emulated motes only */
    .serial_input    = native_mote_serial_input,
    .destroy         = native_mote_destroy,
    .reset_time      = native_mote_reset_time,
    .ui_radio_state  = NULL,
    .ui_leds         = NULL,
    .get_interface   = native_mote_get_interface,
    .receive_frame   = native_mote_receive_frame,
};
