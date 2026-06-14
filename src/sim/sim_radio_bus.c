/*
 * sim_radio_bus — RF routing helpers.  See include/sim/sim_radio_bus.h.
 */
#include "sim_radio_bus.h"
#include "sim_runtime.h"

#include <string.h>

static void sim_radio_bus_frame_complete(sim_radio_bus_t *bus,
                                         sim_runtime_t *sim, int sender_idx,
                                         int sender_radio, int64_t byte_time_ns,
                                         int64_t sender_byte_ns);

/* On-air formats:
 *   2.4 GHz: preamble(4) 0x00 + SFD 0x7A + length at data[5]; receiving
 *     chip pushes (length + 1) bytes into its RXFIFO.
 *   sub-GHz: preamble(4) 0x55 + sync(4) + PHR at data[8]; receiving
 *     chip pushes (PHR + payload + 2 status) = (length + 3) bytes. */
int frame_fifo_bytes(const uint8_t *data, int len, bool subghz) {
    if (subghz) {
        /* Need preamble(4) + sync(4) + PHR(1) at minimum to read length. */
        if (len < 9) return 9999;
        return (int)data[8] + 3;
    }
    if (len < 6) return 9999;
    return (int)data[5] + 1;
}

/* ============================================================
 * Per-sender frame assembler (moved from the runner, M9.4).
 * ============================================================ */

bool sim_radio_bus_asm_feed(tx_frame_asm_t *a, uint8_t byte) {
    switch (a->state) {
    case TX_ASM_PREAMBLE:
        /* Look for either preamble pattern. */
        a->sync_match = (a->sync_match << 8) | byte;
        if (a->sync_match == RADIO_FRAME_802154G_SYNC_WORD) {
            /* CC1200 sync word — next 1 or 2 bytes are the PHR (we
             * disambiguate inside TX_ASM_SUBGHZ_PHR). */
            a->state = TX_ASM_SUBGHZ_PHR;
            a->subghz = true;
            a->subghz_phr_len = 0;
            a->phr_lo = -1;
            a->payload_count = 0;
            return false;
        }
        if (byte == 0x00) {
            a->zero_count++;
        } else if (a->zero_count >= 4 && byte == 0x7A) {
            a->state = TX_ASM_LENGTH;
            a->subghz = false;
        } else {
            a->zero_count = 0;
        }
        return false;

    case TX_ASM_LENGTH:
        a->expected_len = byte;
        if (byte < 3 || byte > 127) {
            /* Invalid length, reset */
            a->state = TX_ASM_PREAMBLE;
            a->zero_count = 0;
            return false;
        }
        a->state = TX_ASM_PAYLOAD;
        a->payload_count = 0;
        return false;

    case TX_ASM_SUBGHZ_PHR:
        /* CC1200 PHR is 1 or 2 bytes depending on the firmware's
         * configuration of PKT_CFG2 bit 5 (FG_MODE / 802.15.4g). The
         * test runner doesn't have a back-channel to that register, so
         * we sniff: if the first PHR byte makes a sensible 1-byte length
         * (3..200) we lock into 1-byte mode; otherwise we treat it as
         * the upper 3 bits of a 2-byte 802.15.4g PHR. This works
         * because the firmwares we ship use either standard mode (PHR=1)
         * or 802.15.4g mode (PHR=2) — never both — and the 802.15.4g
         * upper byte's low-3-bits-as-length-high yields values like 0,
         * 1, or 2 (well under 200), so a phra of e.g. 0x10 (CRC bit + 0)
         * for a small 802.15.4g frame still parses correctly via the
         * 1-byte path even though it's 2-byte on the wire. The
         * follow-up PHR-byte path catches the few high-length cases. */
        if (a->phr_lo < 0) {
            /* First PHR byte. */
            uint8_t len = byte;
            if (len >= 3 && len <= 200) {
                a->expected_len = len;
                a->subghz_phr_len = 1;
                a->state = TX_ASM_PAYLOAD;
                a->payload_count = 0;
                return false;
            }
            /* Looks like 802.15.4g: upper byte. */
            a->phr_lo = byte & 0x07;
            a->subghz_phr_len = 2;
        } else {
            /* PHRB byte: low 8 bits */
            a->expected_len = (a->phr_lo << 8) | byte;
            if (a->expected_len < 3 || a->expected_len > 200) {
                a->state = TX_ASM_PREAMBLE;
                a->zero_count = 0;
                a->sync_match = 0;
                a->subghz = false;
                return false;
            }
            a->state = TX_ASM_PAYLOAD;
            a->payload_count = 0;
        }
        return false;

    case TX_ASM_PAYLOAD: {
        a->payload_count++;
        /* For 802.15.4 (CC2420 / cc2538_rfcore) the length byte's value
         * already includes the 2 FCS bytes the chip auto-appends, so
         * frame-complete fires on byte expected_len. For CC1200 the
         * length byte counts payload only and the chip appends 2 extra
         * CRC bytes after the payload — we wait for those too so the
         * receiver-side dispatch's total_air_bytes count includes them. */
        int payload_target = a->expected_len + (a->subghz ? 2 : 0);
        if (a->payload_count >= payload_target) {
            /* Frame complete — reset only the bytes-on-wire tracking
             * (state + sliding sync register + zero counter); leave
             * subghz / subghz_phr_len / expected_len intact so the caller
             * can read them to compute total_air_bytes. They get cleared
             * the next time a fresh frame's first preamble byte arrives. */
            a->state = TX_ASM_PREAMBLE;
            a->zero_count = 0;
            a->sync_match = 0;
            a->phr_lo = -1;
            return true;
        }
        return false;
    }
    }
    return false;
}

void sim_radio_bus_asm_reset(tx_frame_asm_t *a) {
    a->state = TX_ASM_PREAMBLE;
    a->zero_count = 0;
    a->sync_match = 0;
    a->expected_len = 0;
    a->payload_count = 0;
    a->phr_lo = -1;
    a->subghz = false;
    a->subghz_phr_len = 0;
    a->first_byte_ns = 0;
}

/* ============================================================
 * Registration (M9.4).
 * ============================================================ */

void sim_radio_bus_register(sim_radio_bus_t *bus, int idx,
                            const mote_radio_ops_t *ops, void *mote,
                            sim_radio_delivery_mode_t mode, uint32_t caps) {
    if (idx < 0 || idx >= SIM_RADIO_BUS_MAX_NODES) return;
    bus->ops[idx] = ops;
    bus->mote[idx] = mote;
    bus->delivery[idx] = mode;
    bus->caps[idx] = caps;
    if (idx >= bus->node_count) bus->node_count = idx + 1;
}

void sim_radio_bus_set_host(sim_radio_bus_t *bus,
                            const sim_radio_bus_host_t *host) {
    bus->host = *host;
}

/* ============================================================
 * TX path (M9.4, §3.9).
 * ============================================================ */

int sim_radio_bus_pick_receiver_radio(const radio_medium_t *medium,
                                      int sender_idx, int sender_radio,
                                      int receiver_idx) {
    radio_spectrum_t s_spec =
        medium->nodes[sender_idx].radios[sender_radio].spectrum;
    if (s_spec == RADIO_SPECTRUM_NONE) {
        /* Sender slot unregistered: legacy "unknown sender allows
         * everything" — target the receiver's slot 0 (the legacy
         * single-radio slot). */
        return 0;
    }
    /* Sender registered: find the receiver slot with matching spectrum. */
    for (int r = 0; r < RADIO_MEDIUM_MAX_RADIOS_PER_NODE; r++) {
        if (medium->nodes[receiver_idx].radios[r].spectrum == s_spec)
            return r;
    }
    /* No matching spectrum on receiver. Slot 0 unregistered keeps the
     * legacy "receiver-unknown allows everything" behaviour for
     * platforms that never call register_radio. */
    if (medium->nodes[receiver_idx].radios[0].spectrum == RADIO_SPECTRUM_NONE)
        return 0;
    return -1;
}

void sim_radio_bus_push_channel(sim_radio_bus_t *bus, sim_runtime_t *sim,
                                int idx, int radio_idx, int channel) {
    (void)bus;
    radio_medium_t *rm = &sim->radio_medium;
    /* Slot 0 = 2.4 GHz primary; slot 1 = sub-GHz (CC1200) by convention. */
    if (radio_idx < 0 || radio_idx >= RADIO_MEDIUM_MAX_RADIOS_PER_NODE) return;
    /* Auto-register the radio's spectrum on first channel push so
     * pick_receiver_radio routes per-band instead of falling back to
     * slot 0 (which would send CC1200 bytes to a parked cc2538_rfcore). */
    if (channel >= 0 &&
        rm->nodes[idx].radios[radio_idx].spectrum == RADIO_SPECTRUM_NONE) {
        radio_spectrum_t want = (radio_idx == 0)
            ? RADIO_SPECTRUM_2_4GHZ_15_4
            : RADIO_SPECTRUM_868MHZ_15_4G;
        radio_medium_register_radio(rm, idx, radio_idx, want);
    }
    radio_medium_set_radio_channel(rm, idx, radio_idx, channel);
    /* Legacy single-channel alias mirror (the CCA channel-busy query and
     * a few older readers still key on nodes[i].channel): sub-GHz radios
     * appear at SUBGHZ_CHANNEL_BASE + channel; a 2.4 GHz radio writes the
     * raw channel only if slot 1 is unregistered (on dual-radio Firefly
     * the cc1200 is the active radio, so cc2538_rfcore must not stomp the
     * alias and flip the band gate). */
    if (radio_idx == 1 && channel >= 0) {
        rm->nodes[idx].channel = RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE + channel;
    } else if (radio_idx == 0) {
        bool slot1_registered =
            rm->nodes[idx].radios[1].spectrum != RADIO_SPECTRUM_NONE;
        if (!slot1_registered)
            rm->nodes[idx].channel = channel;
    }
}

/* M28: pull a native mote's channel into the medium before filtering
 * (chip motes push synchronously through their channel callbacks, so
 * they leave current_channel NULL and this is a no-op for them).
 * Mirrors the runner's old sync_native_node_channel. */
static void bus_sync_channel(sim_radio_bus_t *bus, sim_runtime_t *sim,
                             int idx) {
    if (!bus->ops[idx] || !bus->ops[idx]->current_channel) return;
    int ch = bus->ops[idx]->current_channel(bus->mote[idx]);
    if (ch < -1) return;  /* no channel reported */
    if (sim->radio_medium.nodes[idx].radios[0].channel == ch) return;
    radio_medium_set_channel(&sim->radio_medium, idx, ch);
}

/* Dispatch one on-air byte to every receiver the medium accepts.
 * Identical policy for outer and re-entrant (auto-ACK) bytes; `depth`
 * only flows into the host's debug hooks. */
static void dispatch_tx_byte(sim_radio_bus_t *bus, sim_runtime_t *sim,
                             int sender_idx, int sender_radio, uint8_t byte,
                             int64_t byte_time_ns, int depth) {
    const sim_radio_bus_host_t *h = &bus->host;
    radio_medium_t *medium = &sim->radio_medium;
    bool sender_sync = bus->delivery[sender_idx] == SIM_RADIO_DELIVERY_SYNC;

    for (int i = 0; i < bus->node_count; i++) {
        if (i == sender_idx) continue;
        if (bus->delivery[i] == SIM_RADIO_DELIVERY_NONE) continue;
        if (h->node_active && !h->node_active(h->user, i)) continue;
        /* Native-to-native traffic goes through the frame-level path
         * (mixed_rf_frame_handler), not the byte stream. */
        if (sender_sync && bus->delivery[i] == SIM_RADIO_DELIVERY_SYNC)
            continue;
        /* Native receiver: pull current channel before the medium decides. */
        bus_sync_channel(bus, sim, i);
        int rr = sim_radio_bus_pick_receiver_radio(medium, sender_idx,
                                                   sender_radio, i);
        if (rr < 0) continue;
        if (!radio_medium_filter_byte_radio(medium, sender_idx, sender_radio,
                                            i, rr, byte))
            continue;
        if (bus->delivery[i] == SIM_RADIO_DELIVERY_SYNC) {
            /* Native: feed the mote's frame assembler synchronously.
             * (RSSI unused — natives get it at frame level.) */
            bus->ops[i]->receive_byte(bus->mote[i], byte, 0);
            continue;
        }
        int8_t rssi = radio_medium_get_rssi(medium, sender_idx, i);
        if (h->on_byte_accepted)
            h->on_byte_accepted(h->user, sender_idx, i, byte, depth);
        if (bus->delivery[i] == SIM_RADIO_DELIVERY_PER_BYTE) {
            /* One kernel event per byte at its air time (clamped to now —
             * the kernel never schedules into the past). */
            int64_t bt = byte_time_ns;
            int64_t now = sim_runtime_now_ns(sim);
            if (bt < now) bt = now;
            sim_schedule_radio_byte(sim, i, sender_idx, byte, rssi, bt);
            /* RX-stall watchdog (M9.5): extend this receiver's deadline
             * past the byte's air time; lazily keep one timer pending. */
            if (bus->ops[i]->rx_stall) {
                int64_t dl = bt + SIM_RADIO_RX_STALL_NS;
                if (dl > bus->rx_stall_deadline_ns[i])
                    bus->rx_stall_deadline_ns[i] = dl;
                if (!bus->rx_stall_pending[i]) {
                    bus->rx_stall_pending[i] = true;
                    sim_schedule_radio_timer(sim, i,
                                             bus->rx_stall_deadline_ns[i]);
                }
            }
        } else {
            /* BATCH: stage for whole-frame delivery at frame-complete. */
            rf_buffer_t *buf = &bus->rf_pending[i];
            if (buf->count < RF_BUF_SIZE)
                buf->bytes[buf->count++] = byte;
        }
    }
}

bool sim_radio_bus_rx_stall_expired(sim_radio_bus_t *bus,
                                    struct sim_runtime *sim,
                                    int node_idx, int64_t time_ns) {
    if (node_idx < 0 || node_idx >= SIM_RADIO_BUS_MAX_NODES) return false;
    bus->rx_stall_pending[node_idx] = false;
    if (time_ns < bus->rx_stall_deadline_ns[node_idx]) {
        /* Fresher bytes arrived after this timer was armed — re-arm at
         * the extended deadline; this event is stale. */
        bus->rx_stall_pending[node_idx] = true;
        sim_schedule_radio_timer(sim, node_idx,
                                 bus->rx_stall_deadline_ns[node_idx]);
        return false;
    }
    return true;
}

void sim_radio_bus_reset_node(sim_radio_bus_t *bus, int idx) {
    if (idx < 0 || idx >= SIM_RADIO_BUS_MAX_NODES) return;
    bus->rx_stall_deadline_ns[idx] = 0;
    bus->rx_stall_pending[idx] = false;
}

void sim_radio_bus_tx_byte(sim_radio_bus_t *bus, struct sim_runtime *sim,
                           int sender_idx, int sender_radio, uint8_t byte) {
    if (sender_idx < 0 || sender_idx >= bus->node_count) return;
    tx_frame_asm_t *a = &bus->tx_asm[sender_idx];
    tx_frame_capture_t *cap = &bus->tx_cap[sender_idx];

    /* Record first byte time for accurate TX start computation and track
     * subsequent bytes on the sender's on-air byte clock.
     *
     * Fires for either preamble flavour: 0x00 (IEEE 802.15.4 2.4 GHz)
     * or 0x55 (CC1200 802.15.4g sub-GHz). Without arming for 0x55, the
     * per-byte schedule fell back to first_byte_ns=0 → bytes clamped
     * to sim_now → whole frame dumped into the receiver in one batch.
     * That short-circuited the ~16 ms real air time to ~5 ms and
     * pushed the receiver's ACK ~10 ms ahead of the firmware-tuned
     * CSMA_ACK_WAIT envelope. The 0x55 arm is paired with the
     * receive-side MARC_IDLE-tolerance fix in cc1200_receive_byte
     * (without that, bytes arriving the same sim_ns as the receiver's
     * CSMA prepare()→SIDLE→SRX transition were silently dropped). */
    if (a->state == TX_ASM_PREAMBLE && a->zero_count == 0 &&
        (byte == 0x00 || byte == 0x55)) {
        /* Match Cooja's radio callbacks: outgoing bytes are observed at the
         * current scheduler time, not from a mote-local sim_time that may
         * have advanced within the current execute slice. */
        a->first_byte_ns = sim_runtime_now_ns(sim);
        cap->len = 0;
    }
    /* Per-sender byte period — sub-GHz CC1200 frames take 5x longer per
     * byte than 2.4 GHz IEEE 802.15.4. Use the sender's frame profile
     * detected by the assembler (subghz set on sync-word match). */
    int64_t sender_byte_ns = byte_period_ns(a->subghz);
    int64_t byte_time_ns = a->first_byte_ns +
                           (int64_t)cap->len * sender_byte_ns;
    if (cap->len < (int)sizeof(cap->bytes))
        cap->bytes[cap->len++] = byte;

    int depth = bus->tx_depth;
    const sim_radio_bus_host_t *h = &bus->host;
    if (h->on_tx_byte)
        h->on_tx_byte(h->user, sender_idx, byte, byte_time_ns, depth);

    dispatch_tx_byte(bus, sim, sender_idx, sender_radio, byte,
                     byte_time_ns, depth);

    if (depth > 0) {
        /* Re-entrant byte (a receiver's auto-ACK emitted while the outer
         * frame_complete delivers): dispatched above, but never fed to
         * the assembler — the outer frame_complete flushes staged ACK
         * bytes per receiver. */
        return;
    }

    if (!sim_radio_bus_asm_feed(a, byte))
        return;  /* frame not yet complete */

    /* Frame complete — the bus delivers staged frames, runs collision/
     * FIFO policy, and flushes auto-ACKs (M27).  tx_depth marks the
     * scope so nested chip TX callbacks take the re-entrant path. */
    (void)h;
    bus->tx_depth++;
    sim_radio_bus_frame_complete(bus, sim, sender_idx, sender_radio,
                                 byte_time_ns, sender_byte_ns);
    bus->tx_depth--;
}

/* ============================================================
 * Emulated-receiver RX delivery (M26 — moved from the runner).
 * CPU motion goes through the mote's sim_mote_ops_t; no chip headers.
 * ============================================================ */

void sim_radio_bus_set_executing(sim_radio_bus_t *bus, int idx) {
    bus->executing_node = idx;
}

/* Push a complete frame into an emulated node's RX queue.  Collision
 * detection is NOT done here — the caller owns it (emu_rx_end_ns check +
 * interference loops), since a data frame and its own auto-ACK queued to
 * the same receiver would otherwise look like a self-collision. */
void sim_radio_bus_queue_frame(sim_radio_bus_t *bus, int idx,
                               const uint8_t *data, int len, int8_t rssi,
                               int64_t arrival_ns, int64_t end_ns,
                               bool subghz) {
    emu_rx_queue_t *q = &bus->emu_rx_queue[idx];
    if (q->count >= EMU_RX_QUEUE_SIZE) {
        bus->stats.rx_dropped++;
        return;
    }
    int slot = (q->head + q->count) % EMU_RX_QUEUE_SIZE;
    int copy_len = len < EMU_RX_FRAME_MAX ? len : EMU_RX_FRAME_MAX;
    memcpy(q->frames[slot].data, data, (size_t)copy_len);
    q->frames[slot].len = copy_len;
    q->frames[slot].rssi = rssi;
    q->frames[slot].arrival_ns = arrival_ns;
    q->frames[slot].end_ns = end_ns;
    q->frames[slot].collided = false;
    q->frames[slot].subghz = subghz;
    q->count++;
    bus->stats.rx_queued++;
}

/* Deliver buffered bytes directly to an emulated node's radio.  The RX
 * timeline + radio state stay runner-side via the on_rx hook.  For
 * MSP430 each byte is preceded by execute(t, 0) at its air time (Cooja's
 * Msp802154Radio.receiveCustomData()); ARM mirrors it with its own
 * per-byte sync.  Both go through the mote's rx_byte_sync op — native/JS
 * leave it NULL, for which this is a no-op past the timeline emit. */
void sim_radio_bus_deliver_bytes(sim_radio_bus_t *bus, sim_runtime_t *sim,
                                 int idx, const uint8_t *data, int len,
                                 int8_t rssi, int64_t air_time_ns,
                                 bool subghz) {
    int64_t byte_ns = byte_period_ns(subghz);
    if (len > 0 && bus->host.on_rx)
        bus->host.on_rx(bus->host.user, idx, air_time_ns,
                        air_time_ns + (int64_t)len * byte_ns);

    sim_mote_t *m = sim_runtime_mote(sim, idx);
    if (!m || !m->ops->rx_byte_sync)
        return;  /* native/JS: timeline only, no byte delivery */

    /* If the radio is mid-frame (RX_FRAME), queue instead of delivering;
     * delivering now would corrupt the in-progress frame.  Only CC2420
     * reports busy — cc2538/nrf rx_busy is always false. */
    if (bus->ops[idx]->rx_busy(bus->mote[idx])) {
        sim_radio_bus_queue_frame(bus, idx, data, len, rssi, air_time_ns,
                                  air_time_ns + (int64_t)len * byte_ns, subghz);
        return;
    }

    /* GUARD: if this node is currently executing (executing_node), skip
     * the per-byte sync to avoid a re-entrant step; bytes go into the
     * chip's rx_incoming buffer and replay when it re-enters RX.  MSP430
     * additionally steps ~1 ms (CAP_RX_TICKING_STEP) so the CC2420
     * processes symbol events + auto-ACK in-line. */
    int64_t last_byte_ns = air_time_ns;
    if (idx == bus->executing_node) {
        for (int j = 0; j < len; j++)
            bus->ops[idx]->receive_byte(bus->mote[idx], data[j], rssi);
        if (bus->caps[idx] & SIM_RADIO_CAP_RX_TICKING_STEP) {
            int64_t budget = (int64_t)m->ops->freq_hz(m) / 1000; /* 1ms */
            if (budget < 1000) budget = 1000;
            m->ops->step_until(m, m->ops->cycles(m) + budget);
        }
    } else {
        for (int j = 0; j < len; j++) {
            int64_t byte_time_ns = air_time_ns + (int64_t)j * byte_ns;
            m->ops->rx_byte_sync(m, byte_time_ns);
            bus->ops[idx]->receive_byte(bus->mote[idx], data[j], rssi);
            last_byte_ns = byte_time_ns;
        }
    }

    /* Request immediate wakeup so the receiver gets CPU time for its
     * FIFOP/ISR work. */
    sim_schedule_mote_wakeup_if_earlier(sim, idx, last_byte_ns);
}

/* Drain queued RX frames for an emulated node: at most one frame per
 * call, skipping collided frames, mini-stepping to free the RXFIFO when
 * the platform allows it (CAP_DRAIN_MINI_STEP). */
void sim_radio_bus_drain_rx(sim_radio_bus_t *bus, sim_runtime_t *sim,
                            int idx) {
    emu_rx_queue_t *q = &bus->emu_rx_queue[idx];
    int blocked_attempts = 0;
    while (q->count > 0) {
        emu_rx_frame_t *f = &q->frames[q->head];

        /* Skip collided frames (like native_dequeue_rx_frame does). */
        if (f->collided) {
            q->head = (q->head + 1) % EMU_RX_QUEUE_SIZE;
            q->count--;
            continue;
        }

        int fifo_needed = frame_fifo_bytes(f->data, f->len, f->subghz);
        if (bus->ops[idx]->rxfifo_available(bus->mote[idx]) < fifo_needed) {
            if ((bus->caps[idx] & SIM_RADIO_CAP_DRAIN_MINI_STEP) &&
                blocked_attempts < 4) {
                sim_mote_t *m = sim_runtime_mote(sim, idx);
                int64_t freq = m->ops->freq_hz(m);
                int64_t spare_cycles = freq / 1000;
                if (spare_cycles <= 0) spare_cycles = 1;
                m->ops->step_until(m, m->ops->cycles(m) + spare_cycles);
                blocked_attempts++;
                continue;
            }
            break;  /* RXFIFO still busy, try next time step */
        }
        blocked_attempts = 0;

        /* Reset collision tracking before each drain delivery: a queued
         * frame's delivery may trigger an auto-ACK whose frame assembler
         * sets emu_rx_end_ns on the ACK receiver; without this reset,
         * sequential drains would falsely collide. */
        memset(bus->emu_rx_end_ns, 0, sizeof(bus->emu_rx_end_ns));

        /* max(arrival, now): deferred ACKs queued with a future arrival
         * (192 µs turnaround past the data frame's TX-end) still advance
         * the receiver's CPU by the turnaround before bytes hit the
         * radio. */
        int64_t now = sim_runtime_now_ns(sim);
        int64_t deliver_ns = f->arrival_ns > now ? f->arrival_ns : now;
        sim_radio_bus_deliver_bytes(bus, sim, idx, f->data, f->len, f->rssi,
                                    deliver_ns, f->subghz);
        bus->stats.rx_drained++;

        q->head = (q->head + 1) % EMU_RX_QUEUE_SIZE;
        q->count--;
        break;
    }
}

void sim_radio_bus_wake_mote(sim_radio_bus_t *bus, sim_runtime_t *sim,
                             int idx) {
    /* sim_runtime_now_ns is the authoritative cross-node "now"; a queued
     * RX frame means re-run at the current sim time (Cooja's
     * requestImmediateWakeup), otherwise use the mote's next-CPU-event
     * lead from sched_hint_ns (emulated motes only — this is never
     * called for native/JS). */
    int64_t now = sim_runtime_now_ns(sim);
    if (bus->emu_rx_queue[idx].count > 0) {
        sim_schedule_mote_wakeup_if_earlier(sim, idx, now);
        return;
    }
    sim_mote_t *m = sim_runtime_mote(sim, idx);
    int64_t next = m->ops->sched_hint_ns(m, now);
    sim_schedule_mote_wakeup_if_earlier(sim, idx, next);
}

/* ============================================================
 * Frame-complete delivery policy (M27 — moved from the runner's
 * bus_host_frame_complete).  Owns: air-time math, CCA busy-until,
 * the per-receiver snapshot, collision windows, RXFIFO backpressure,
 * sync-vs-queue delivery, the dual 192 µs auto-ACK windows, and
 * interference marking.  All observability (timeline, PCAP, packet
 * analyzer, verbose prints, traces) is delegated to the host's
 * frame_observed / on_rx_frame / on_ack hooks so the bus stays free of
 * chip and presentation concerns.
 * ============================================================ */
static void sim_radio_bus_frame_complete(sim_radio_bus_t *bus,
                                         sim_runtime_t *sim, int sender_idx,
                                         int sender_radio, int64_t byte_time_ns,
                                         int64_t sender_byte_ns) {
    const sim_radio_bus_host_t *h = &bus->host;
    tx_frame_asm_t *a = &bus->tx_asm[sender_idx];
    int64_t now = sim_runtime_now_ns(sim);

    /* Air-byte budget for the dispatch loop must match what the chip
     * driver emitted: 802.15.4 = 4 preamble + SFD + length byte +
     * payload (length includes the 2 FCS); 802.15.4g = 4 preamble +
     * 4 sync + PHR(1/2) + payload + 2 auto-CRC. */
    int total_air_bytes = a->subghz
        ? (4 + 4 + a->subghz_phr_len + a->expected_len + 2)
        : (4 + 1 + 1 + a->expected_len);
    int64_t frame_air_dur = (int64_t)total_air_bytes * sender_byte_ns;
    int64_t accurate_tx_end = byte_time_ns + sender_byte_ns;
    int64_t accurate_tx_start = accurate_tx_end - frame_air_dur;
    if (accurate_tx_start < 0) accurate_tx_start = 0;

    /* Mark the medium TX-busy on the sender's channel until the frame
     * leaves the air (CCA reads this).  Sub-GHz first_byte_ns isn't
     * armed, so anchor busy-until to now + air-time for CC1200. */
    int64_t busy_until = accurate_tx_end;
    if (a->subghz)
        busy_until = now + frame_air_dur;
    if (busy_until > bus->tx_busy_until_ns[sender_idx])
        bus->tx_busy_until_ns[sender_idx] = busy_until;

    sim_radio_frame_info_t fi = {
        .sender_idx      = sender_idx,
        .sender_radio    = sender_radio,
        .expected_len    = a->expected_len,
        .total_air_bytes = total_air_bytes,
        .subghz          = a->subghz,
        .tx_start_ns     = accurate_tx_start,
        .tx_end_ns       = accurate_tx_end,
        .air_dur_ns      = frame_air_dur,
        .sender_byte_ns  = sender_byte_ns,
        .capture         = bus->tx_cap[sender_idx].bytes,
        .capture_len     = bus->tx_cap[sender_idx].len,
    };
    /* Entry observability (RTRACE, TX timeline, PCAP, analyzer prints,
     * the [RF] receiver-list trace, stat_rf_frames). */
    if (h->frame_observed)
        h->frame_observed(h->user, &fi);

    bus->in_delivery = true;  /* explicit timeline events emitted above */

    /* The sender is skipped by the snapshot loop, but nested auto-ACK
     * delivery buffers into rf_pending[sender]; clear it so ACK bytes
     * can't append onto stale contents. */
    bus->rf_pending[sender_idx].count = 0;

    /* Snapshot each BATCH receiver's staged frame before delivery, so
     * re-entrant auto-ACK bytes (written to other receivers' rf_pending
     * during synchronous delivery) can't corrupt later receivers. */
    static uint8_t frame_snap[SIM_RADIO_BUS_MAX_NODES][EMU_RX_FRAME_MAX];
    static int     frame_snap_len[SIM_RADIO_BUS_MAX_NODES];
    static int8_t  frame_snap_rssi[SIM_RADIO_BUS_MAX_NODES];
    static int     snap_indices[SIM_RADIO_BUS_MAX_NODES];
    int snap_count = 0;

    for (int i = 0; i < bus->node_count; i++) {
        rf_buffer_t *buf = &bus->rf_pending[i];
        /* Only BATCH receivers stage bytes in rf_pending; SYNC/PER_BYTE
         * leave count==0 (M25 de-typing). */
        if (i == sender_idx ||
            bus->delivery[i] != SIM_RADIO_DELIVERY_BATCH ||
            buf->count == 0)
            continue;
        if (buf->count != total_air_bytes) {
            if (h->on_rx_frame)
                h->on_rx_frame(h->user, &fi, i, SIM_RADIO_RX_STALE_BUFFER,
                               NULL, buf->count, 0, 0);
            buf->count = 0;
            continue;
        }
        int copy_len = buf->count < EMU_RX_FRAME_MAX ? buf->count : EMU_RX_FRAME_MAX;
        memcpy(frame_snap[i], buf->bytes, (size_t)copy_len);
        frame_snap_len[i] = copy_len;
        frame_snap_rssi[i] = radio_medium_get_rssi(&sim->radio_medium,
                                                    sender_idx, i);
        snap_indices[snap_count++] = i;
        buf->count = 0;  /* clear before delivery to isolate re-entrancy */
    }

    for (int s = 0; s < snap_count; s++) {
        int i = snap_indices[s];
        /* Collision window: the frame occupies the channel [coll_start,
         * coll_end).  Sub-GHz anchors to now (first_byte_ns unarmed). */
        int64_t coll_start = accurate_tx_start;
        int64_t coll_end = accurate_tx_end;
        if (a->subghz) {
            coll_start = now;
            coll_end = now + frame_air_dur;
        }

        if (coll_start < bus->emu_rx_end_ns[i]) {
            bus->stats.rx_collided++;
            if (h->on_rx_frame)
                h->on_rx_frame(h->user, &fi, i, SIM_RADIO_RX_COLLIDED,
                               frame_snap[i], frame_snap_len[i],
                               coll_start, bus->emu_rx_end_ns[i]);
            continue;
        }

        int fifo_needed = frame_fifo_bytes(frame_snap[i], frame_snap_len[i],
                                           a->subghz);
        sim_mote_t *mi = sim_runtime_mote(sim, i);
        if (bus->ops[i]->rxfifo_available(bus->mote[i]) < fifo_needed) {
            /* RXFIFO full — mini-step the receiver to read the previous
             * frame; keep it short to avoid cascade TX. */
            mi->ops->step_until(mi, mi->ops->cycles(mi) + 5000);
        }
        if (bus->ops[i]->rxfifo_available(bus->mote[i]) >= fifo_needed) {
            /* Sub-GHz delivery anchors to now (first_byte_ns unarmed). */
            int64_t delivery_start = a->subghz ? now : accurate_tx_start;
            /* MSP430-only full-slice pre-sync before delivery. */
            if (mi->ops->rx_pre_sync && i != bus->executing_node)
                mi->ops->rx_pre_sync(mi, delivery_start);
            sim_radio_bus_deliver_bytes(bus, sim, i, frame_snap[i],
                                        frame_snap_len[i], frame_snap_rssi[i],
                                        delivery_start, a->subghz);
            bus->stats.rx_direct++;
            bus->emu_rx_end_ns[i] = coll_end;
            if (h->on_rx_frame)
                h->on_rx_frame(h->user, &fi, i, SIM_RADIO_RX_DIRECT,
                               frame_snap[i], frame_snap_len[i], 0, 0);
        } else {
            sim_radio_bus_queue_frame(bus, i, frame_snap[i], frame_snap_len[i],
                                      frame_snap_rssi[i], accurate_tx_start,
                                      coll_end, a->subghz);
            bus->emu_rx_end_ns[i] = coll_end;
            if (h->on_rx_frame)
                h->on_rx_frame(h->user, &fi, i, SIM_RADIO_RX_QUEUED,
                               frame_snap[i], frame_snap_len[i], 0, 0);
        }

        /* Flush auto-ACK bytes generated by this delivery.  ACK starts
         * after the data frame ends + 192 µs turnaround (12 symbols).
         * The data sender's own ACK uses now + 192 µs instead: TX is
         * instantaneous here, so its firmware ACK-wait timer starts at
         * now, and accurate_tx_end + 192 µs would land past the
         * RTIMER_BUSYWAIT_UNTIL timeout → perpetual CSMA retransmits. */
        int64_t ack_start = accurate_tx_end + 192000LL;
        int64_t sender_ack_start = now + 192000LL;
        int ack_tx_emitted = 0;
        for (int j = 0; j < bus->node_count; j++) {
            /* SYNC ⇔ native: natives never stage ACK bytes (M25). */
            if (bus->rf_pending[j].count > 0 &&
                bus->delivery[j] != SIM_RADIO_DELIVERY_SYNC) {
                if (!ack_tx_emitted) {
                    if (h->on_ack)
                        h->on_ack(h->user, &fi, i, ack_start,
                                  bus->rf_pending[j].count);
                    ack_tx_emitted = 1;
                }
                int ack_payload = (bus->rf_pending[j].count > 5)
                                      ? bus->rf_pending[j].bytes[5] : 0;
                /* The data sender's ACK goes via the rx_queue (its radio
                 * is still TX/DISABLED at synchronous delivery time; the
                 * end-of-tick drain delivers it after the driver flips
                 * back to RX) at sender_ack_start. */
                if (j == sender_idx)
                    sim_radio_bus_queue_frame(bus, j, bus->rf_pending[j].bytes,
                                              bus->rf_pending[j].count, -50,
                                              sender_ack_start, coll_end,
                                              a->subghz);
                else if (bus->ops[j]->rxfifo_available(bus->mote[j]) >= ack_payload + 1)
                    sim_radio_bus_deliver_bytes(bus, sim, j,
                                                bus->rf_pending[j].bytes,
                                                bus->rf_pending[j].count, -50,
                                                ack_start, a->subghz);
                else
                    sim_radio_bus_queue_frame(bus, j, bus->rf_pending[j].bytes,
                                              bus->rf_pending[j].count, -50,
                                              ack_start, coll_end, a->subghz);
                bus->rf_pending[j].count = 0;
            }
        }
    }

    /* Interference: mark queued frames on the sender's interference-range
     * neighbours as collided if they overlap this transmission. */
    if (sim->radio_medium.type != RADIO_MEDIUM_NONE) {
        neighbor_list_t *inl = &sim->radio_medium.interference_neighbors[sender_idx];
        int64_t int_start = now;
        int64_t int_end = now + SIM_RADIO_INTERFERENCE_WINDOW_NS;
        for (int n = 0; n < inl->count; n++) {
            int i = inl->neighbors[n];
            if (bus->delivery[i] == SIM_RADIO_DELIVERY_SYNC) continue;
            emu_rx_queue_t *q = &bus->emu_rx_queue[i];
            for (int f = 0; f < q->count; f++) {
                int fi2 = (q->head + f) % EMU_RX_QUEUE_SIZE;
                emu_rx_frame_t *existing = &q->frames[fi2];
                if (int_start < existing->end_ns && existing->arrival_ns < int_end) {
                    if (!existing->collided) bus->stats.rx_collided++;
                    existing->collided = true;
                }
            }
        }
    }

    bus->in_delivery = false;

    /* Schedule the sender's wakeup so its radio completes the TX→RX
     * turnaround (enhanced-ACK reception depends on it). */
    if (bus->caps[sender_idx] & SIM_RADIO_CAP_WAKE_SENDER_POST_TX)
        sim_radio_bus_wake_mote(bus, sim, sender_idx);
}

/* ============================================================
 * Frame-level TX path for native/JS senders (M28 — moved from the
 * runner's mixed_rf_frame_handler delivery loop).  Delivers a complete
 * frame to every frame-consuming neighbour through its receive_frame
 * mote op (native's op now does the direct-buffer-or-queue fast path),
 * and marks interference-range collisions (native via its
 * mark_collisions op, emulated queues bus-side).
 * ============================================================ */
void sim_radio_bus_tx_frame(sim_radio_bus_t *bus, sim_runtime_t *sim,
                            int sender_idx, const uint8_t *frame, int len) {
    radio_medium_t *medium = &sim->radio_medium;
    int64_t now = sim_runtime_now_ns(sim);

    /* Native sender: pull its channel into the medium before filtering. */
    bus_sync_channel(bus, sim, sender_idx);

    if (medium->type != RADIO_MEDIUM_NONE) {
        neighbor_list_t *nl = &medium->neighbors[sender_idx];
        for (int n = 0; n < nl->count; n++) {
            int i = nl->neighbors[n];
            /* Native receiver: snap channel before the filter consults
             * it (no-op for chip motes, which push their channel). */
            bus_sync_channel(bus, sim, i);
            sim_mote_t *m = sim_runtime_mote(sim, i);
            if (m && m->ops->receive_frame) {
                if (radio_medium_filter_frame(medium, sender_idx, i)) {
                    if (m->ops->receive_frame(m, frame, len, now, sender_idx) < 0)
                        bus->stats.frame_queue_full++;
                    bus->stats.frame_queued++;
                }
            }
            /* Native/JS-to-emulated is handled via the byte-stream TX
             * callback, not this frame path. */
        }

        /* Interference-range neighbours: mark overlapping queued frames
         * as collided. */
        neighbor_list_t *inl = &medium->interference_neighbors[sender_idx];
        int64_t tx_start = now;
        int64_t tx_end = tx_start + (int64_t)(len + 6) * IEEE802154_BYTE_NS;
        for (int n = 0; n < inl->count; n++) {
            int i = inl->neighbors[n];
            if (bus->ops[i] && bus->ops[i]->mark_collisions) {
                bus->stats.frame_collided +=
                    bus->ops[i]->mark_collisions(bus->mote[i], tx_start, tx_end);
            } else {
                emu_rx_queue_t *q = &bus->emu_rx_queue[i];
                for (int f = 0; f < q->count; f++) {
                    int fi = (q->head + f) % EMU_RX_QUEUE_SIZE;
                    emu_rx_frame_t *existing = &q->frames[fi];
                    if (tx_start < existing->end_ns &&
                        existing->arrival_ns < tx_end) {
                        if (!existing->collided) bus->stats.rx_collided++;
                        existing->collided = true;
                    }
                }
            }
        }
    } else {
        /* NONE: deliver to every other frame-consuming node. */
        for (int i = 0; i < bus->node_count; i++) {
            if (i == sender_idx) continue;
            sim_mote_t *m = sim_runtime_mote(sim, i);
            if (m && m->ops->receive_frame) {
                if (m->ops->receive_frame(m, frame, len, now, sender_idx) < 0)
                    bus->stats.frame_queue_full++;
                bus->stats.frame_queued++;
            }
        }
    }
}
