/*
 * CC1200 sub-GHz radio chip driver. See include/arm/cc1200.h for scope.
 *
 * Design notes:
 *  - Every CPU/GPIO interaction goes through the sim_host_t vtable.
 *    The driver never references arm_cpu_t / cc2538_gpio_t — that
 *    keeps the chip portable across SoCs.
 *  - The on-air bit-stream is modelled as bytes:
 *      [4× 0x55 preamble] [SYNC3 SYNC2 SYNC1 SYNC0] [PHRA PHRB] [payload..]
 *    Hidden inside the 802.15.4g frame is a 2-byte PHR encoding the
 *    payload length (`(phra & 0x07) << 8 | phrb`) and a CRC-16 flag
 *    (phra bit 4 = 1 for 2-byte CRC, the Contiki default).
 *  - Per-byte transmit timing uses the documented 50 kbps profile
 *    (160 µs per byte = 160 000 ns). The radio_medium frame tracker
 *    will see exactly what the firmware put on the air.
 *  - GDO0 IOCFG defaults to PKT_SYNC_RXTX (asserts on SFD-detected/
 *    TX-started, deasserts on packet-end). We drive only those edges.
 */
#include "cc1200.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* HOST_* convenience accessors live in include/common/sim_host.h —
 * shared with cc2420 / any future off-SoC chip driver. */

/* ------------------------------------------------------------------ */
/* Air-side bit-stream timing — 50 kbps 2-FSK as configured by Contiki  */
/* ------------------------------------------------------------------ */

/* CC1200_TIMING_PROBE, latched once — the unlatched getenv() sat in the
 * per-SPI-byte exchange path (see arm_cpu.c getenv latching rationale). */
static int cc1200_timing_probe_flag = -1;
static int cc1200_timing_probe(void) {
    if (__builtin_expect(cc1200_timing_probe_flag < 0, 0))
        cc1200_timing_probe_flag = getenv("CC1200_TIMING_PROBE") ? 1 : 0;
    return cc1200_timing_probe_flag;
}

#define CC1200_BYTE_PERIOD_NS    160000   /* 8 bits @ 50 kbps */
#define CC1200_RESET_TIME_NS     200000   /* SRES → IDLE, ~200 µs in real HW */

/* ------------------------------------------------------------------ */
/* MARCSTATE transition delays                                          */
/* ------------------------------------------------------------------ */

/* Real silicon (datasheet SWRS123, §3.2.1 + Table 4-1) needs measurable
 * wall-clock time to move the MARCSTATE machine.  In csim the same
 * delays are critical for correctness, not realism: synchronous
 * transitions cause inbound preamble bytes to be dropped during the
 * receiver's own CSMA prepare/transmit window, which kills RPL
 * convergence.  Cross-checked against
 *   ~/work/contiki-ng/arch/dev/radio/cc1200/cc1200.c
 * which busy-waits with `RTIMER_BUSYWAIT_UNTIL_STATE(s, RTIMER_SECOND/100)`
 * — the 10 ms upper bound is a safety net, not the typical delay.  Real
 * typical delays per the datasheet:
 *
 *   IDLE → CALIBRATE → IDLE  (SCAL)         ~720 µs full PLL cal
 *   IDLE → CAL → SETTLING → RX  (SRX)       ~90 µs (no AUTOCAL: just settling)
 *   IDLE → CAL → SETTLING → TX  (STX)       ~90 µs (no AUTOCAL)
 *   RX/TX → SETTLING → IDLE  (SIDLE)        ~50 µs
 *
 * We pick rounded values that comfortably exceed one byte period
 * (160 µs) so any inbound frame mid-flight has a fighting chance to
 * reach the air decoder before MARCSTATE leaves RX. */
#define CC1200_SIDLE_DELAY_NS    50000    /* 50 µs settling out of RX/TX */
#define CC1200_SRX_DELAY_NS      200000   /* 200 µs cal + settling into RX */
#define CC1200_STX_DELAY_NS      200000   /* 200 µs cal + settling into TX */
#define CC1200_SCAL_DELAY_NS     720000   /* 720 µs full PLL calibration */
#define CC1200_SETTLING_DELAY_NS 100000   /* 100 µs auto TX → SETTLING → RX hop */

/* End-of-frame PKT_SYNC_RXTX falling edge is fired SYNCHRONOUSLY from
 * cc1200_receive_byte (matching CC2420 set_fifop / CC2538 RFCore
 * RFIRQF0_RXPKTDONE / nRF52840 EVENTS_END). The harness's per-byte
 * RX path syncs the receiver CPU to the byte's air-time before each
 * call to cc1200_receive_byte (see emu_deliver_bytes in
 * test_mixed_multinode.c), so pending the IRQ here is serviced on
 * the receiver's very next instruction — no defer needed. */

/* ------------------------------------------------------------------ */
/* Status byte / MARCSTATE helpers                                      */
/* ------------------------------------------------------------------ */

static uint8_t marcstate_to_status_top(uint8_t marc) {
    switch (marc) {
    case CC1200_MARC_SLEEP:
    case CC1200_MARC_IDLE:        return CC1200_STATUS_IDLE;
    case CC1200_MARC_RX:          return CC1200_STATUS_RX;
    case CC1200_MARC_FSTXON:      return CC1200_STATUS_FSTXON;
    case CC1200_MARC_TX:          return CC1200_STATUS_TX;
    case CC1200_MARC_RX_FIFO_ERR: return CC1200_STATUS_RX_FIFO_ERR;
    case CC1200_MARC_TX_FIFO_ERR: return CC1200_STATUS_TX_FIFO_ERR;
    default:                      return CC1200_STATUS_IDLE;
    }
}

static void refresh_status(cc1200_t *c) {
    /* Status byte top nibble = state, low 4 bits = bytes-in-FIFO clipped to 0xF.
     * The "bytes in FIFO" field is the RX FIFO when in RX, TX FIFO otherwise.
     * Real chip is a bit fancier, but Contiki only cares about the top nibble.
     *
     * If a deferred MARCSTATE transition is pending (marcstate_event live),
     * marc_transit_status carries the intermediate STATE_CAL / SETTLING
     * top-nibble that real silicon would expose to firmware.  Contiki's
     * calibrate() explicitly busy-waits for STATE_CALIBRATE, so this
     * hand-off matters. */
    uint8_t state_bits = (c->marc_transit_status != 0xFF)
        ? c->marc_transit_status
        : marcstate_to_status_top(c->marcstate);
    int bytes = (c->marcstate == CC1200_MARC_RX) ? c->rx_count : c->tx_count;
    if (bytes > 0xF) bytes = 0xF;
    c->status = state_bits | (uint8_t)bytes;
}

uint8_t cc1200_status(const cc1200_t *c) { return c->status; }
int     cc1200_marcstate(const cc1200_t *c) { return c->marcstate; }
int     cc1200_rxfifo_count(const cc1200_t *c) { return c->rx_count; }
int     cc1200_txfifo_count(const cc1200_t *c) { return c->tx_count; }

/* ------------------------------------------------------------------ */
/* GDO0 / GDO2 edge driving + IOCFG signal multiplexing                  */
/* ------------------------------------------------------------------ */

/* Real silicon: each GDOx pin is multiplexed — IOCFGx.GPIOx_CFG selects
 * which of ~64 internal signals drives the pin in real time.  The pin
 * level is recomputed whenever either (a) the selected internal signal
 * changes, or (b) firmware writes IOCFGx (pointing the pin at a
 * possibly-different signal whose current value drives the pin
 * instantly — and may produce an edge).  See devices/zoul-firefly/
 * DATASHEET-FINDINGS.md §1 and SWRU346B p.18-19.
 *
 * The csim model:
 *   - Track each modeled internal signal as a bool field on cc1200_t.
 *   - gdo_signal_value(c, iocfg) maps IOCFG → current bool level.
 *   - propagate_signals(c) is called from every site that mutates a
 *     signal or the IOCFG register; it recomputes both pins and drives
 *     the host (set_input_pin + force_irq_edge) on changes.
 *   - drive_gdo() stays as the shared "on edge → bridge to host" path.
 */

static void drive_gdo(cc1200_t *c, const cc1200_pin_t *pin, bool *cur_level, bool level) {
    if (!pin->enabled) return;
    if (*cur_level == level) return;
    *cur_level = level;
    HOST_SET_PIN(c, pin->port, pin->pin, level);
    HOST_FORCE_IRQ(c, pin->port, pin->pin, level);
}

/* Map the 5-bit MARCSTATE encoding to the 2-bit MARC_2PIN_STATE
 * encoding (SWRU346B p.106 + DATASHEET-FINDINGS §2).
 *   00 SETTLING  → all calibration / settling / FIFO-err sub-states
 *   01 TX        → 0x13 TX, 0x14 TX_END
 *   10 IDLE      → 0x00 SLEEP, 0x01 IDLE, 0x02 STARTUP (we map SLEEP→IDLE)
 *   11 RX        → 0x0D RX, 0x0E RX_END, 0x0F RXDCM
 * Bit 1 → MARC_2PIN_STATUS_1 (signal 37); bit 0 → MARC_2PIN_STATUS_0 (signal 38). */
static uint8_t marc_2pin_state(uint8_t marcstate) {
    switch (marcstate) {
    case CC1200_MARC_SLEEP: case CC1200_MARC_IDLE:
        return 2;  /* IDLE: bit1=1, bit0=0 */
    case 0x02:  /* STARTUP */
        return 2;
    case 0x0D: case 0x0E: case 0x0F:
        return 3;  /* RX: bit1=1, bit0=1 */
    case 0x13: case 0x14:
        return 1;  /* TX: bit1=0, bit0=1 */
    default:
        /* All other sub-states (0x03..0x0C, 0x10, 0x11, 0x12, 0x15, 0x16,
         * 0x17) are SETTLING per the table: 00. */
        return 0;
    }
}

/* Map IOCFGx register value to the chip's current internal signal
 * level.  Returns the boolean that should be driven onto the GDO pin
 * when this signal is selected.  Bit 6 (GPIOx_INV) inverts the result.
 * Other bits (drive strength, etc.) are not modeled. */
static bool gdo_signal_value(const cc1200_t *c, uint8_t iocfg) {
    bool v;
    switch (iocfg & CC1200_IOCFG_GPIO0_CFG_MASK) {
    case CC1200_IOCFG_PKT_SYNC_RXTX:        v = c->sig_pkt_sync_rxtx; break;
    case CC1200_IOCFG_RSSI_VALID:           v = c->sig_rssi_valid; break;
    case CC1200_IOCFG_CARRIER_SENSE_VALID:  v = c->sig_cs_valid; break;
    case CC1200_IOCFG_CARRIER_SENSE:        v = c->sig_carrier_sense; break;
    case CC1200_IOCFG_MARC_2PIN_STATUS_1:   v = (marc_2pin_state(c->marcstate) & 2) != 0; break;
    case CC1200_IOCFG_MARC_2PIN_STATUS_0:   v = (marc_2pin_state(c->marcstate) & 1) != 0; break;
    default:                                v = false; break; /* unmodeled — pin idle low */
    }
    if (iocfg & CC1200_IOCFG_GPIO0_INV) v = !v;
    return v;
}

/* Recompute both GDO pin levels from the current IOCFGx selectors and
 * internal signal state, and drive any edges through to the host. */
static void propagate_signals(cc1200_t *c) {
    bool gdo0 = gdo_signal_value(c, c->regs[CC1200_REG_IOCFG0]);
    bool gdo2 = gdo_signal_value(c, c->regs[CC1200_REG_IOCFG2]);
    drive_gdo(c, &c->gdo0, &c->gdo0_level, gdo0);
    drive_gdo(c, &c->gdo2, &c->gdo2_level, gdo2);
}

/* ------------------------------------------------------------------ */
/* FIFO helpers                                                         */
/* ------------------------------------------------------------------ */

static void fifo_reset_tx(cc1200_t *c) {
    c->tx_first = c->tx_last = c->tx_count = 0;
}

static void fifo_reset_rx(cc1200_t *c) {
    c->rx_first = c->rx_last = c->rx_count = 0;
}

static bool fifo_push_rx(cc1200_t *c, uint8_t byte) {
    if (c->rx_count >= CC1200_FIFO_SIZE) return false;
    c->rx_fifo[c->rx_last] = byte;
    c->rx_last = (c->rx_last + 1) % CC1200_FIFO_SIZE;
    c->rx_count++;
    return true;
}

static uint8_t fifo_pop_rx(cc1200_t *c) {
    if (c->rx_count == 0) return 0;
    uint8_t v = c->rx_fifo[c->rx_first];
    c->rx_first = (c->rx_first + 1) % CC1200_FIFO_SIZE;
    c->rx_count--;
    return v;
}

static bool fifo_push_tx(cc1200_t *c, uint8_t byte) {
    if (c->tx_count >= CC1200_FIFO_SIZE) return false;
    c->tx_fifo[c->tx_last] = byte;
    c->tx_last = (c->tx_last + 1) % CC1200_FIFO_SIZE;
    c->tx_count++;
    return true;
}

static uint8_t fifo_pop_tx(cc1200_t *c) {
    if (c->tx_count == 0) return 0;
    uint8_t v = c->tx_fifo[c->tx_first];
    c->tx_first = (c->tx_first + 1) % CC1200_FIFO_SIZE;
    c->tx_count--;
    return v;
}

/* ------------------------------------------------------------------ */
/* End-of-frame deferred event                                          */
/* ------------------------------------------------------------------ */

/* Drop PKT_SYNC_RXTX synchronously to mark the firmware-visible
 * "packet complete" edge. Called inline from cc1200_receive_byte's
 * end-of-frame paths — same pattern as CC2420's set_fifop(true),
 * CC2538 RFCore's rfcore_check_interrupts(RFIRQF0_RXPKTDONE), and
 * nRF52840's radio_event(&evt_end). */
static void frame_done_now(cc1200_t *c) {
    if (cc1200_timing_probe())
        fprintf(stderr, "[t=%lld cc1200@%p RX_DONE GPIO0↓ rx_count=%d]\n",
                (long long)HOST_NOW_NS(c), (void*)c, c->rx_count);
    c->sig_pkt_sync_rxtx = false;
    propagate_signals(c);
}

/* ------------------------------------------------------------------ */
/* Air decoder                                                           */
/* ------------------------------------------------------------------ */

static uint32_t sync_word_value(const cc1200_t *c) {
    return ((uint32_t)c->regs[CC1200_REG_SYNC3] << 24) |
           ((uint32_t)c->regs[CC1200_REG_SYNC2] << 16) |
           ((uint32_t)c->regs[CC1200_REG_SYNC1] << 8)  |
           ((uint32_t)c->regs[CC1200_REG_SYNC0]);
}

static void air_reset(cc1200_t *c) {
    c->air_state = CC1200_AIR_HUNT;
    c->sync_match = 0;
    c->air_phr_count = 0;
    c->air_payload_remaining = 0;
    c->air_payload_total = 0;
    c->air_crc_remaining = 0;
}

void cc1200_receive_byte(cc1200_t *c, uint8_t byte) {
    /* Bytes only count when the chip is in (or transitioning into) RX.
     *
     * The marc_pending check matters when the firmware's CSMA
     * prepare() fires SIDLE → SRX at the exact sim_ns a frame is
     * about to start hitting our air decoder. Without it, we observe
     * marcstate=IDLE for one byte, drop it silently, then the chip
     * lands in RX too late to spot the sync word — every frame is
     * effectively invisible. Real silicon doesn't have this race
     * because IDLE→RX settling holds the front end "open" during
     * the transition; allowing bytes while marc_pending==RX models
     * the same behaviour. */
    if (c->marcstate != CC1200_MARC_RX &&
        c->marc_pending != CC1200_MARC_RX) return;

    switch (c->air_state) {
    case CC1200_AIR_HUNT: {
        /* Slide the byte into the sync-word match register. */
        c->sync_match = (c->sync_match << 8) | byte;
        if (c->sync_match == sync_word_value(c)) {
            c->air_state = CC1200_AIR_PHR;
            c->air_phr_count = 0;
            /* Sync match → assert PKT_SYNC_RXTX (signal 6).
             * propagate_signals() drives any GDO pin whose IOCFGx
             * currently selects signal 6.  Pins pointing at other
             * signals (e.g. MARC_2PIN_STATUS_0 during the firmware's
             * transmit() window) are unaffected — datasheet-correct.
             * The gpio peripheral's sticky RIS + IE 0→1 re-pend
             * (cc2538_gpio.c) handles the case where firmware
             * re-routes IOCFG back to PKT_SYNC_RXTX before draining
             * the in-flight frame. */
            c->sig_pkt_sync_rxtx = true;
            propagate_signals(c);
        }
        break;
    }
    case CC1200_AIR_PHR: {
        /* PHR is 1 or 2 bytes wide depending on PKT_CFG2[5] (FG_MODE).
         * Standard CC120x mode (FG_MODE=0): single length byte, 1..255.
         * 802.15.4g mode (FG_MODE=1): two PHR bytes — phra (top 3 bits =
         * length high, bit 4 = CRC select, bit 3 = whitening) + phrb
         * (length low). The chip exposes PHR bytes to firmware verbatim
         * via the RX FIFO, so push every PHR byte first. */
        if (!fifo_push_rx(c, byte)) {
            c->marcstate = CC1200_MARC_RX_FIFO_ERR;
            air_reset(c);
            /* RX FIFO overflow → MARCSTATE leaves RX (SETTLING/IDLE per
             * 2-pin map) and PKT_SYNC_RXTX drops synchronously, same as
             * a clean end-of-frame (CC2420 set_fifop / CC2538 RXPKTDONE). */
            propagate_signals(c);
            frame_done_now(c);
            return;
        }
        bool fg_mode = (c->regs[CC1200_REG_PKT_CFG2] &
                        CC1200_PKT_CFG2_FG_MODE_802154G) != 0;
        if (!fg_mode) {
            /* 1-byte PHR: 'byte' is the full length. */
            c->air_payload_total = byte;
            c->air_phr_count++;
        } else if (c->air_phr_count == 0) {
            c->air_payload_total = (byte & 0x07) << 8;
            c->air_phr_count++;
            break;  /* expecting phrb next */
        } else {
            c->air_payload_total |= byte;
            c->air_phr_count++;
        }
        /* Validate length and arm payload reception. The PHR length
         * convention differs between the two modes Contiki's cc1200
         * driver supports (arch/dev/radio/cc1200/cc1200.c:
         * copy_header_to_tx_fifo):
         *   - 802.15.4g (FG_MODE=1, 2-byte PHR): PHR encodes
         *       payload_len + crc_len  (firmware does payload_len += 2
         *       before splitting into phra/phrb).
         *   - Standard CC120x (FG_MODE=0, 1-byte PHR): PHR encodes
         *       payload_len only (the chip auto-appends CRC; firmware
         *       does NOT pre-add it to the length byte).
         * The wire is always `[PHR] [payload] [CRC]`; only the PHR
         * value's interpretation differs. Account for that here so
         * `air_payload_remaining` is the FIFO-push count and
         * `air_crc_remaining` is the consume-silently count. */
        int min_total = fg_mode ? 2 : 1;
        if (c->air_payload_total < min_total ||
            c->air_payload_total > CC1200_FIFO_SIZE - (fg_mode ? 4 : 3)) {
            c->marcstate = CC1200_MARC_RX_FIFO_ERR;
            air_reset(c);
            propagate_signals(c);
            frame_done_now(c);
            return;
        }
        if (fg_mode) {
            /* 15.4g: PHR includes CRC bytes. Peel them off the tail. */
            c->air_payload_remaining = c->air_payload_total - 2;
            c->air_crc_remaining = 2;
        } else {
            /* Standard mode: PHR is payload only; CRC follows separately. */
            c->air_payload_remaining = c->air_payload_total;
            c->air_crc_remaining = 2;
        }
        c->air_state = CC1200_AIR_PAYLOAD;
        break;
    }
    case CC1200_AIR_PAYLOAD: {
        if (c->air_payload_remaining > 0) {
            if (!fifo_push_rx(c, byte)) {
                c->marcstate = CC1200_MARC_RX_FIFO_ERR;
                air_reset(c);
                propagate_signals(c);
                frame_done_now(c);
                return;
            }
            c->air_payload_remaining--;
            break;
        }
        /* Past the payload — these bytes are the on-air CRC. We don't
         * verify (the medium owns loss decisions) but we DO consume
         * the bytes so the next packet's preamble starts cleanly. */
        if (c->air_crc_remaining > 0) {
            c->air_crc_remaining--;
            if (c->air_crc_remaining == 0) {
                /* Packet complete. With APPEND_STATUS=1 the real chip
                 * appends two status bytes after the payload: RSSI
                 * then (CRC_OK<<7 | LQI). Contiki's
                 * cc1200_rx_interrupt() reads `payload_len + 2` bytes
                 * from the FIFO and looks at bit 7 of the very last
                 * byte to decide CRC OK. We always present "CRC OK"
                 * since we never lost bytes in the medium handoff. */
                if (c->rx_count + 2 <= CC1200_FIFO_SIZE) {
                    fifo_push_rx(c, (uint8_t)c->rx_rssi);
                    fifo_push_rx(c, 0x80);
                }
                c->stat_rx_packets++;
                air_reset(c);
                /* Drop PKT_SYNC_RXTX synchronously — receiver CPU has
                 * been stepped to this byte's air time by the harness
                 * before cc1200_receive_byte was called, so pending
                 * the IRQ now lands on its very next instruction.
                 * Same pattern as CC2420 set_fifop(true), CC2538
                 * RFCore RFIRQF0_RXPKTDONE, nRF52840 EVENTS_END. */
                frame_done_now(c);
            }
        }
        break;
    }
    }
    refresh_status(c);
}

/* ------------------------------------------------------------------ */
/* TX byte emission                                                    */
/* ------------------------------------------------------------------ */

static void tx_byte_event_cb(void *user_data, cpu_event_t *ev);
static void schedule_marc_transition(cc1200_t *c, uint8_t target_marc,
                                      uint8_t transit_status, int64_t delay_ns);

static void start_tx(cc1200_t *c) {
    if (c->tx_count == 0) {
        /* Nothing to send — TX FIFO underflow */
        c->marcstate = CC1200_MARC_TX_FIFO_ERR;
        refresh_status(c);
        propagate_signals(c);
        return;
    }

    c->tx_active = true;
    c->marcstate = CC1200_MARC_TX;
    refresh_status(c);

    /* TX-started → assert PKT_SYNC_RXTX (signal 6).  propagate_signals
     * drives any GDO pin selecting signal 6 to high; pins selecting
     * MARC_2PIN_STATUS_0 (38) also see an edge here because marcstate
     * just transitioned IDLE→TX (MARC[0] flips 0→1).  Real firmware
     * busy-waits on whichever pin is mapped, so the edge has to fire
     * before we return. */
    c->sig_pkt_sync_rxtx = true;
    propagate_signals(c);

    /* Emit every air byte synchronously — preamble (4 × 0x55) +
     * sync_word (4 bytes) + payload (TX FIFO contents) + 2 CRC bytes.
     * Synchronous emission matches cc2538_rfcore's ISTXON path: it
     * produces a single contiguous burst into the receiver-side
     * rf_pending buffer with no risk of interleaving when round-robin
     * scheduling hands the next time slice to a different node
     * mid-frame.
     *
     * CRC: real CC120x chips auto-append 2 CRC bytes when PKT_CFG1
     * has CRC_CFG enabled (its default at reset). We don't actually
     * compute the CRC — the receiver-side decoder doesn't verify it
     * either; the radio_medium handles loss probabilistically. The
     * 2 placeholder CRC bytes make the on-air byte count match what
     * the receiver framer expects (length-byte's value = payload
     * bytes + CRC), so the air-byte budget agrees with the receiving
     * chip's framer.
     *
     * The TX-end event is still scheduled by air time so MARCSTATE
     * stays in TX for the right wall-clock duration; firmware that
     * polls MARCSTATE between TX and RX gets the same end-of-TX
     * timing it would on real hardware. */
    int saved_tx_count = c->tx_count;   /* before fifo_pop_tx drains it */
    if (c->rf_tx_callback) {
        for (int i = 0; i < 4; i++)
            c->rf_tx_callback(c->rf_tx_user_data, 0x55);
        uint32_t sw = sync_word_value(c);
        c->rf_tx_callback(c->rf_tx_user_data, (uint8_t)(sw >> 24));
        c->rf_tx_callback(c->rf_tx_user_data, (uint8_t)(sw >> 16));
        c->rf_tx_callback(c->rf_tx_user_data, (uint8_t)(sw >> 8));
        c->rf_tx_callback(c->rf_tx_user_data, (uint8_t)(sw));
        while (c->tx_count > 0)
            c->rf_tx_callback(c->rf_tx_user_data, fifo_pop_tx(c));
        /* Auto-CRC placeholder. */
        c->rf_tx_callback(c->rf_tx_user_data, 0x00);
        c->rf_tx_callback(c->rf_tx_user_data, 0x00);
    } else {
        /* No listener attached — drain the FIFO so subsequent TX still
         * starts from a clean state. */
        while (c->tx_count > 0) (void)fifo_pop_tx(c);
    }

    /* Schedule TX-end (GPIO0 / PKT_SYNC_RXTX falling edge + MARCSTATE
     * return to RX) at the actual on-air completion time:
     *   4 preamble + 4 sync + saved_tx_count (= PHR + payload) + 2 CRC.
     *
     * This pairs with the per-byte delivery fix (first_byte_ns armed
     * for the 0x55 preamble in test_mixed_multinode.c). Real hardware:
     *   - sender TX-end fires at +air_time
     *   - receiver finishes RX at +air_time
     *   - CSMA ACK_WAIT (5 ms default) starts at +air_time
     *   - ACK arrives at sender at +air_time + ~few ms, INSIDE window
     * With the old 1 byte-period proxy here, sender's TX-end fired at
     * +160 µs but receiver's RX (now correctly paced) didn't finish
     * until +16 ms, so ACK_WAIT expired ~11 ms before the receiver
     * could even read the frame. */
    int air_bytes = 4 + 4 + saved_tx_count + 2;
    int64_t fire = HOST_NOW_NS(c) +
        (int64_t)air_bytes * CC1200_BYTE_PERIOD_NS;
    HOST_SCHEDULE_NS(c, &c->tx_byte_event, fire);
}

static void tx_byte_event_cb(void *user_data, cpu_event_t *ev) {
    (void)ev;
    cc1200_t *c = (cc1200_t *)user_data;
    if (!c->tx_active) return;

    /* All on-air bytes were emitted synchronously inside start_tx; this
     * event just finalises TX state (RX-on, GDO0 falling edge,
     * MARCSTATE=RX via SETTLING).  The stat counter increments here so
     * each STX → start_tx → tx_byte_event_cb sequence is visible to
     * unit tests. */
    c->tx_active = false;
    c->stat_tx_packets++;
    /* TX done → PKT_SYNC_RXTX falls.  MARCSTATE must walk TX → SETTLING
     * → RX (RFEND_CFG0.TXOFF_MODE=RX), NOT TX → RX directly.  The
     * Contiki cc1200 driver reconfigures GPIO0 to MARC_2PIN_STATUS_0
     * (signal 38, = bit0 of marc_2pin_state) for the duration of
     * transmit() and busy-waits for it to go low to detect TX-end
     * (see arch/dev/radio/cc1200/cc1200.c:1970, :2111).  In our 2-pin
     * encoding bit0 is HIGH for both TX(01) and RX(11) but LOW for
     * SETTLING(00) — so a direct TX → RX hop never produces a
     * falling edge on signal 38, the driver hits its tx_pkt_lifetime
     * (~50 ms for 50 kbps) timeout, and returns RADIO_TX_ERR.
     *
     * Schedule the SETTLING → RX hop ~100 µs later via marcstate_event
     * so the visible state walk is TX → SETTLING (now) → RX (+100 µs).
     * That gives signal 38 a clean 1 → 0 → 1 trajectory that matches
     * what real silicon does post-TX with RFEND_CFG0.TXOFF_MODE=RX. */
    c->sig_pkt_sync_rxtx = false;
    c->marcstate = CC1200_MARC_SETTLING;
    c->sig_rssi_valid = true;
    c->sig_cs_valid = true;
    air_reset(c);
    refresh_status(c);
    propagate_signals(c);
    schedule_marc_transition(c, CC1200_MARC_RX, CC1200_STATUS_SETTLING,
                              CC1200_SETTLING_DELAY_NS);
}

/* ------------------------------------------------------------------ */
/* Strobes                                                              */
/* ------------------------------------------------------------------ */

static void reset_done_event_cb(void *user_data, cpu_event_t *ev) {
    (void)ev;
    cc1200_t *c = (cc1200_t *)user_data;
    c->marcstate = CC1200_MARC_IDLE;
    c->marc_pending = CC1200_MARC_IDLE;
    c->marc_transit_status = 0xFF;
    refresh_status(c);
    /* MARC moved SLEEP → IDLE: both still encode 2pin = IDLE(10), so no
     * MARC_2PIN edge.  But propagate to honour any IOCFG signal that
     * happens to flip on this transition (e.g. an inverted pin). */
    propagate_signals(c);
}

/* ------------------------------------------------------------------ */
/* Deferred MARCSTATE strobe transitions                                 */
/* ------------------------------------------------------------------ */

/* Fired once the strobe-induced settling/calibration period has
 * elapsed.  At this point we promote marc_pending into marcstate, drop
 * the transitional status-byte override, and refresh status so the
 * next SPI poll observes the real state. */
static void marcstate_event_cb(void *user_data, cpu_event_t *ev) {
    (void)ev;
    cc1200_t *c = (cc1200_t *)user_data;
    uint8_t target = c->marc_pending;
    if (cc1200_timing_probe())
        fprintf(stderr, "[t=%lld cc1200@%p MARC_FIRE 0x%02x→0x%02x]\n",
                (long long)HOST_NOW_NS(c), (void*)c, c->marcstate, target);
    c->marcstate = target;
    c->marc_transit_status = 0xFF;

    /* MARCSTATE-derived signals (MARC_2PIN_STATUS_0/_1) update
     * implicitly via propagate_signals().  The RSSI/CS_VALID signals
     * track RX residency: asserted on RX entry, cleared on exit.  We
     * model "settled immediately" rather than the real ~tens-of-µs
     * AGC settle delay — same approximation as the existing reg_read
     * RSSI0 path.  Carrier-sense itself is recomputed live from the
     * medium on each reg_read; we don't cache it here. */
    if (target == CC1200_MARC_RX) {
        c->sig_rssi_valid = true;
        c->sig_cs_valid   = true;
    } else {
        c->sig_rssi_valid = false;
        c->sig_cs_valid   = false;
        c->sig_carrier_sense = false;
    }

    refresh_status(c);
    propagate_signals(c);

    /* If the target state is TX, kick off the actual frame emission now
     * that calibration/settling has completed.  Mirrors what real
     * silicon does at the IDLE→TX transition.  start_tx will assert
     * sig_pkt_sync_rxtx and call propagate_signals itself. */
    if (target == CC1200_MARC_TX && !c->tx_active) {
        start_tx(c);
    }
    /* If the target state is RX after a transition (SRX), reset the air
     * decoder so it starts fresh.  Already done at strobe time, but
     * keep this idempotent in case a back-to-back SRX races with a
     * pending event. */
    if (target == CC1200_MARC_RX && c->air_state != CC1200_AIR_HUNT) {
        /* Air decoder is mid-frame from before the transition.  Don't
         * reset — let the in-flight frame complete naturally. */
    }
}

/* Schedule a deferred MARCSTATE transition.  current_marc stays in
 * effect (so MARCSTATE register reads still return the prior state and
 * cc1200_receive_byte's RX gating still admits bytes while we settle).
 * The status byte is overridden to transit_status (CALIBRATE or
 * SETTLING) so firmware that polls via SNOP sees a realistic
 * intermediate state.  Cancels any prior in-flight transition. */
static void schedule_marc_transition(cc1200_t *c,
                                      uint8_t  target_marc,
                                      uint8_t  transit_status,
                                      int64_t  delay_ns) {
    HOST_CANCEL(c, &c->marcstate_event);
    c->marc_pending = target_marc;
    c->marc_transit_status = transit_status;
    int64_t fire = HOST_NOW_NS(c) + delay_ns;
    HOST_SCHEDULE_NS(c, &c->marcstate_event, fire);
    refresh_status(c);
}

static void chip_reset(cc1200_t *c) {
    /* Wipe registers but keep the EXT_PARTNUMBER / EXT_PARTVERSION
     * read-only fixed values. */
    memset(c->regs, 0, sizeof(c->regs));
    c->regs[CC1200_EXT_PARTNUMBER]  = 0x20;  /* CC1200 */
    c->regs[CC1200_EXT_PARTVERSION] = 0x11;  /* arbitrary non-zero */
    /* Default sync word from real chip silicon (the 50 kbps Contiki
     * config overwrites these at boot). */
    c->regs[CC1200_REG_SYNC3] = 0x93;
    c->regs[CC1200_REG_SYNC2] = 0x0B;
    c->regs[CC1200_REG_SYNC1] = 0x51;
    c->regs[CC1200_REG_SYNC0] = 0xDE;
    c->regs[CC1200_REG_PKT_LEN] = 0xFF;

    fifo_reset_tx(c);
    fifo_reset_rx(c);
    air_reset(c);
    c->tx_active = false;
    c->marcstate = CC1200_MARC_SLEEP;
    c->marc_pending = CC1200_MARC_SLEEP;
    c->marc_transit_status = 0xFF;
    /* Internal signals all cleared on reset.  propagate_signals will
     * recompute the GDO pins from the freshly-zeroed IOCFG registers
     * (PKT_SYNC_RXTX after init, see cc1200_init re-set below) and
     * drive any falling edges. */
    c->sig_pkt_sync_rxtx = false;
    c->sig_rssi_valid    = false;
    c->sig_cs_valid      = false;
    c->sig_carrier_sense = false;
    refresh_status(c);
    propagate_signals(c);

    HOST_CANCEL(c, &c->tx_byte_event);
    HOST_CANCEL(c, &c->reset_done_event);
    HOST_CANCEL(c, &c->marcstate_event);
    /* SRES → IDLE after a small delay */
    int64_t fire = HOST_NOW_NS(c) + CC1200_RESET_TIME_NS;
    HOST_SCHEDULE_NS(c, &c->reset_done_event, fire);
}

static void handle_strobe(cc1200_t *c, uint8_t strobe) {
    c->stat_strobe_count++;
    if (cc1200_timing_probe()) {
        const char *name;
        switch (strobe) {
        case CC1200_STROBE_SRES:    name = "SRES";    break;
        case CC1200_STROBE_SFSTXON: name = "SFSTXON"; break;
        case CC1200_STROBE_SXOFF:   name = "SXOFF";   break;
        case CC1200_STROBE_SCAL:    name = "SCAL";    break;
        case CC1200_STROBE_SRX:     name = "SRX";     break;
        case CC1200_STROBE_STX:     name = "STX";     break;
        case CC1200_STROBE_SIDLE:   name = "SIDLE";   break;
        case CC1200_STROBE_SPWD:    name = "SPWD";    break;
        case CC1200_STROBE_SFRX:    name = "SFRX";    break;
        case CC1200_STROBE_SFTX:    name = "SFTX";    break;
        case CC1200_STROBE_SNOP:    name = "SNOP";    break;
        default:                    name = "?";       break;
        }
        /* Skip SNOP — it's the polling loop probe and would flood. */
        if (strobe != CC1200_STROBE_SNOP)
            fprintf(stderr, "[t=%lld cc1200@%p STROBE %s marc=0x%02x pend=0x%02x]\n",
                    (long long)HOST_NOW_NS(c), (void*)c, name,
                    c->marcstate, c->marc_pending);
    }
    switch (strobe) {
    case CC1200_STROBE_SRES:
        chip_reset(c);
        break;
    case CC1200_STROBE_SNOP:
        /* No-op — status byte updated below */
        break;
    case CC1200_STROBE_SIDLE:
        /* Real silicon takes ~50 µs of SETTLING to leave RX/TX into
         * IDLE.  Critically, MARCSTATE remains at the prior value
         * (RX or TX) during this window, so:
         *   - cc1200_receive_byte still admits inbound bytes that
         *     are arriving at the same simulated instant the firmware
         *     fires SIDLE → STX in its CSMA path.  This is the entire
         *     reason for event-driven strobes (see CLAUDE.md and
         *     docs/porting-a-device.md §8).
         *   - The status byte returned via SNOP shows STATE_SETTLING
         *     so firmware that polls via state() sees realistic
         *     intermediate state.
         * If we were already IDLE, no settling needed — short-circuit. */
        if (c->marcstate == CC1200_MARC_IDLE) {
            HOST_CANCEL(c, &c->marcstate_event);
            c->marc_pending = CC1200_MARC_IDLE;
            c->marc_transit_status = 0xFF;
            break;
        }
        /* Cancel any in-flight TX byte emission — SIDLE aborts TX.
         * (End-of-frame is now synchronous, so there is no queued
         * frame_done event to worry about racing with.) */
        c->tx_active = false;
        HOST_CANCEL(c, &c->tx_byte_event);
        schedule_marc_transition(c, CC1200_MARC_IDLE,
                                  CC1200_STATUS_SETTLING,
                                  CC1200_SIDLE_DELAY_NS);
        break;
    case CC1200_STROBE_SCAL:
        /* SCAL is only valid from IDLE per the datasheet.  Contiki's
         * calibrate() busy-waits for STATE_CALIBRATE then STATE_IDLE,
         * so we have to expose CALIBRATE in the status byte during
         * the transition.  Real PLL calibration takes ~720 µs. */
        if (c->marcstate != CC1200_MARC_IDLE) break;
        schedule_marc_transition(c, CC1200_MARC_IDLE,
                                  CC1200_STATUS_CAL,
                                  CC1200_SCAL_DELAY_NS);
        break;
    case CC1200_STROBE_SRX:
        /* Real silicon: IDLE → CAL → SETTLING → RX (~200 µs without
         * AUTOCAL).  We stay at the prior MARCSTATE while the
         * transition is in flight.  air_reset is deferred until the
         * transition fires — if we reset the decoder now we'd lose
         * any in-flight frame that's still streaming through during
         * the transition window.  Idempotent if we're already in RX. */
        if (c->marcstate == CC1200_MARC_RX) {
            HOST_CANCEL(c, &c->marcstate_event);
            c->marc_pending = CC1200_MARC_RX;
            c->marc_transit_status = 0xFF;
            break;
        }
        schedule_marc_transition(c, CC1200_MARC_RX,
                                  CC1200_STATUS_CAL,
                                  CC1200_SRX_DELAY_NS);
        break;
    case CC1200_STROBE_STX:
        /* IDLE → CAL → SETTLING → TX, then start_tx() runs from the
         * marcstate_event callback.  Synchronous start_tx would emit
         * bytes at the wrong simulated instant (zero settling time),
         * which both desynchronises sub-GHz byte timing on the
         * receiver side and prevents any inbound frame from being
         * received during the receiver's own RX→TX turnaround.
         *
         * From FSTXON the synthesizer is already locked — Contiki uses
         * SFSTXON before STX precisely so the STX → TX transition is
         * ~25 µs instead of ~200 µs. Accept FSTXON as a valid entry
         * state and skip the long CAL window. */
        if (c->marcstate != CC1200_MARC_RX &&
            c->marcstate != CC1200_MARC_IDLE &&
            c->marcstate != CC1200_MARC_FSTXON) break;
        schedule_marc_transition(c, CC1200_MARC_TX,
                                  CC1200_STATUS_CAL,
                                  c->marcstate == CC1200_MARC_FSTXON
                                      ? 25000LL
                                      : CC1200_STX_DELAY_NS);
        break;
    case CC1200_STROBE_SFRX:
        /* FIFO flush — instantaneous, no MARC change. */
        if (c->marcstate == CC1200_MARC_IDLE ||
            c->marcstate == CC1200_MARC_RX_FIFO_ERR) {
            fifo_reset_rx(c);
            if (c->marcstate == CC1200_MARC_RX_FIFO_ERR) {
                c->marcstate = CC1200_MARC_IDLE;
                c->marc_pending = CC1200_MARC_IDLE;
            }
        }
        break;
    case CC1200_STROBE_SFTX:
        /* FIFO flush — instantaneous, no MARC change. */
        if (c->marcstate == CC1200_MARC_IDLE ||
            c->marcstate == CC1200_MARC_TX_FIFO_ERR) {
            fifo_reset_tx(c);
            if (c->marcstate == CC1200_MARC_TX_FIFO_ERR) {
                c->marcstate = CC1200_MARC_IDLE;
                c->marc_pending = CC1200_MARC_IDLE;
            }
        }
        break;
    case CC1200_STROBE_SPWD:
        /* Power-down request — synchronous in our model. */
        HOST_CANCEL(c, &c->marcstate_event);
        c->marcstate = CC1200_MARC_SLEEP;
        c->marc_pending = CC1200_MARC_SLEEP;
        c->marc_transit_status = 0xFF;
        c->sig_rssi_valid = c->sig_cs_valid = c->sig_carrier_sense = false;
        break;
    case CC1200_STROBE_SXOFF:
        /* Crystal off → still report IDLE in our model. */
        HOST_CANCEL(c, &c->marcstate_event);
        c->marcstate = CC1200_MARC_IDLE;
        c->marc_pending = CC1200_MARC_IDLE;
        c->marc_transit_status = 0xFF;
        c->sig_rssi_valid = c->sig_cs_valid = c->sig_carrier_sense = false;
        break;
    case CC1200_STROBE_SFSTXON:
        /* Fast TX-on: synthesizer settles, NOT transmitting yet. The
         * subsequent STX transitions FSTXON → TX in ~25 µs. Contiki's
         * cc1200 driver (PR #3141 restoring the strobe — commit
         * 8630d2be1) busy-waits for STATE_FSTXON via the status byte
         * (state() = strobe(SNOP) & 0x70), so we must transition
         * marcstate to MARC_FSTXON (0x12) — not MARC_TX — so the
         * status top nibble returns STATUS_FSTXON (0x30). The old
         * synchronous jump-to-TX kept the busy-wait stuck for the full
         * 10 ms RTIMER_BUSYWAIT timeout, then dropped the subsequent
         * STX strobe (marc was already TX, fell off the IDLE/RX
         * guard). With this fix the busy-wait exits at ~200 µs and
         * STX from FSTXON completes in ~25 µs. */
        if (c->marcstate != CC1200_MARC_IDLE &&
            c->marcstate != CC1200_MARC_RX) break;
        schedule_marc_transition(c, CC1200_MARC_FSTXON,
                                  CC1200_STATUS_CAL,
                                  CC1200_STX_DELAY_NS);
        break;
    default:
        break;
    }
    refresh_status(c);
    /* Strobe may have changed marcstate (SFRX clearing RX_FIFO_ERR,
     * SPWD/SXOFF/SFSTXON instant transitions) and therefore the
     * MARC_2PIN_STATUS_x signals.  Propagate to GDOx pins. */
    propagate_signals(c);
}

/* ------------------------------------------------------------------ */
/* Register handling                                                    */
/* ------------------------------------------------------------------ */

static uint8_t reg_read(cc1200_t *c, uint16_t addr) {
    if (addr >= CC1200_REG_SPACE_SIZE) return 0;
    /* Live-computed registers */
    switch (addr) {
    case CC1200_EXT_MARCSTATE:    return c->marcstate & 0x1F;
    case CC1200_EXT_NUM_RXBYTES:  return (uint8_t)c->rx_count;
    case CC1200_EXT_NUM_TXBYTES:  return (uint8_t)c->tx_count;
    case CC1200_EXT_PARTNUMBER:   return 0x20;
    case CC1200_EXT_PARTVERSION:  return c->regs[CC1200_EXT_PARTVERSION];
    case CC1200_EXT_RSSI0: {
        /* Real-chip behaviour: CARRIER_SENSE_VALID is only asserted once
         * the front-end has settled in RX (a few hundred µs after SRX).
         * The Contiki driver busy-waits up to 10 ms on this bit before
         * giving up.  In our model we report VALID immediately whenever
         * MARCSTATE == RX, since the rest of the radio model treats RX
         * entry as instantaneous.  Outside RX we leave VALID=0 so any
         * stray CCA poll outside an open RX window stays "not yet". */
        if (c->marcstate != CC1200_MARC_RX)
            return 0;

        uint8_t v = CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID;

        /* Two carrier-sense sources, OR'd together:
         *
         *  (a) Mid-frame: our air decoder is past the sync word and
         *      consuming PHR/payload.  This catches frames that have
         *      already been delivered to us but not yet finished.
         *
         *  (b) Medium says another node is currently transmitting on a
         *      matching channel.  This catches the gap between "sender
         *      kicked off TX" and "first preamble byte arrived at us"
         *      — the window where naive CSMA on the receiver would
         *      otherwise see a clear channel and start its own TX,
         *      hiding the actual collision. */
        bool busy = (c->air_state != CC1200_AIR_HUNT);
        if (!busy && c->channel_busy_query) {
            busy = c->channel_busy_query(c->channel_busy_user_data);
        }
        /* Side-effect: cache the carrier-sense value as the internal
         * signal 17 (CARRIER_SENSE) and propagate to GDOx pins on
         * change.  Firmware that maps a GDO pin to CARRIER_SENSE will
         * see edges as the channel state changes — real silicon does
         * the same since the comparator output is the pin source. */
        if (busy != c->sig_carrier_sense) {
            c->sig_carrier_sense = busy;
            propagate_signals(c);
        }
        if (busy) v |= CC1200_RSSI0_CARRIER_SENSE;
        return v;
    }
    default:                       return c->regs[addr];
    }
}

/* Reverse-map a 24-bit FREQ register value to a Contiki channel index
 * for the EU 868 50 kbps profile (chan_center_freq0 = 863125 kHz,
 * spacing = 200 kHz, FREQ_MULTIPLIER = 4096, FREQ_DIVIDER = 625).
 *
 * From cc1200.c calculate_freq():
 *   freq_value = (chan_center_freq0_kHz + channel * spacing_kHz) * 4096 / 625
 *
 * Reverse:
 *   freq_kHz = freq_value * 625 / 4096
 *   channel  = (freq_kHz - 863125) / 200    (integer division, nearest)
 *
 * Channels outside the [0..33] range used by the Contiki 50 kbps profile
 * (and the broader plausible 863-870 MHz EU sub-GHz band) are clamped to
 * -1 so the medium doesn't gate on garbage during PLL settle / partial
 * register writes.  See devices/zoul-firefly/DATASHEET-FINDINGS.md and
 * arch/dev/radio/cc1200/cc1200-802154g-863-870-fsk-50kbps.c for the
 * profile constants. */
static int cc1200_channel_from_freq_value(uint32_t freq_value) {
    if (freq_value == 0) return -1;
    /* Use 64-bit math to avoid overflow on the multiplication. */
    int64_t freq_kHz = ((int64_t)freq_value * 625) / 4096;
    int64_t delta = freq_kHz - 863125;
    /* Round to nearest channel rather than truncate — register writes
     * may incur a 1-kHz rounding error from the integer-math
     * formula. */
    int64_t channel = (delta + 100) / 200;  /* +100 = half spacing (200/2) */
    if (channel < 0 || channel > 63) return -1;
    return (int)channel;
}

/* Push a channel update from the chip driver into the simulation
 * harness via the sim_host_t vtable. Idempotent — chip drivers may call
 * this on every FREQ register write; the harness can de-duplicate if
 * needed. NULL-safe. */
static void cc1200_push_channel(cc1200_t *c) {
    if (!c->host || !c->host->radio_set_channel) return;
    uint32_t freq_value =
        ((uint32_t)c->regs[CC1200_EXT_FREQ2] << 16) |
        ((uint32_t)c->regs[CC1200_EXT_FREQ1] << 8)  |
        ((uint32_t)c->regs[CC1200_EXT_FREQ0]);
    int channel = cc1200_channel_from_freq_value(freq_value);
    /* CC1200 sits on radio slot 1 by convention (slot 0 = cc2538_rfcore). */
    c->host->radio_set_channel(c->host->radio_user_data, /*radio_idx=*/1, channel);
}

static void reg_write(cc1200_t *c, uint16_t addr, uint8_t value) {
    if (addr >= CC1200_REG_SPACE_SIZE) return;
    /* MARCSTATE / FIFO counters are read-only — silently drop writes. */
    switch (addr) {
    case CC1200_EXT_MARCSTATE:
    case CC1200_EXT_NUM_RXBYTES:
    case CC1200_EXT_NUM_TXBYTES:
    case CC1200_EXT_PARTNUMBER:
    case CC1200_EXT_PARTVERSION:
        return;
    default:
        c->regs[addr] = value;
        break;
    }
    /* Writing IOCFG0 / IOCFG2 instantly re-routes the corresponding GDO
     * pin to the newly-selected internal signal — possibly producing an
     * edge.  This is the L6 architectural fix per
     * devices/zoul-firefly/DATASHEET-FINDINGS.md §1: Contiki's
     * transmit() reconfigures IOCFG0 from PKT_SYNC_RXTX (signal 6) to
     * MARC_2PIN_STATUS_0 (signal 38) so it can poll GDO0 for the
     * IDLE→TX transition, then writes it back to PKT_SYNC_RXTX after
     * TX completes.  Both writes must immediately drive whatever level
     * the now-selected signal currently holds. */
    if (addr == CC1200_REG_IOCFG0 || addr == CC1200_REG_IOCFG2) {
        propagate_signals(c);
    }
    /* Writing any of the FREQ bytes (FREQ0/1/2) triggers a channel
     * recompute. The Contiki driver writes the three bytes back-to-back
     * (single_write per byte); each write fires this path, but the
     * intermediate values are still valid channels — they just don't
     * match what the firmware ultimately programs. The medium gets the
     * final channel after the third write completes. */
    if (addr == CC1200_EXT_FREQ0 || addr == CC1200_EXT_FREQ1 ||
        addr == CC1200_EXT_FREQ2) {
        cc1200_push_channel(c);
    }
}

/* ------------------------------------------------------------------ */
/* SPI byte exchange                                                    */
/* ------------------------------------------------------------------ */

uint8_t cc1200_spi_exchange(cc1200_t *c, uint8_t mosi) {
    if (!c->csn_low) return 0;  /* CSn high — chip not selected */

    c->stat_spi_xfers++;
    refresh_status(c);
    uint8_t miso = c->status;

    switch (c->spi_state) {
    case CC1200_SPI_WAITING: {
        /* First byte of a new transaction. */
        uint8_t addr_field = mosi & 0x3F;
        c->spi_is_read   = (mosi & CC1200_CMD_READ) != 0;
        c->spi_is_burst  = (mosi & CC1200_CMD_BURST) != 0;
        c->spi_first_data = false;

        if (mosi >= 0x30 && mosi <= 0x3D && addr_field != 0x2F) {
            /* Strobe (0x30..0x3D, excluding the 0x2F extended prefix) */
            handle_strobe(c, mosi & 0x7F);
            /* Strobe is a single-byte transaction — back to WAITING. */
            c->spi_state = CC1200_SPI_WAITING;
            return c->status;
        }
        if (addr_field == CC1200_DIRECT_FIFO) {
            /* TX FIFO write or RX FIFO read at 0x3F */
            c->spi_addr = CC1200_DIRECT_FIFO;
            c->spi_is_ext = false;
            c->spi_state = CC1200_SPI_FIFO_RW;
            return c->status;
        }
        if (addr_field == CC1200_CMD_EXT_PREFIX) {
            /* Extended-address prefix — next byte is the real low addr */
            c->spi_is_ext = true;
            c->spi_state = CC1200_SPI_WAIT_EXT;
            return c->status;
        }
        /* Regular register access */
        c->spi_is_ext = false;
        c->spi_addr = addr_field;
        c->spi_state = CC1200_SPI_REG_RW;
        return c->status;
    }

    case CC1200_SPI_WAIT_EXT: {
        /* Second byte of an extended-address command. */
        c->spi_addr = (uint16_t)0x2F00 | mosi;
        c->spi_state = CC1200_SPI_REG_RW;
        return c->status;
    }

    case CC1200_SPI_REG_RW: {
        if (c->spi_is_read) {
            miso = reg_read(c, c->spi_addr);
            if (c->spi_is_burst) {
                c->spi_addr++;
            }
            return miso;
        } else {
            reg_write(c, c->spi_addr, mosi);
            if (c->spi_is_burst) {
                c->spi_addr++;
            }
            return c->status;
        }
    }

    case CC1200_SPI_FIFO_RW: {
        if (c->spi_is_read) {
            return fifo_pop_rx(c);
        } else {
            fifo_push_tx(c, mosi);
            return c->status;
        }
    }
    }

    return c->status;
}

/* ------------------------------------------------------------------ */
/* CSn / RESET                                                          */
/* ------------------------------------------------------------------ */

void cc1200_set_csn(cc1200_t *c, bool low) {
    if (c->csn_low == low) return;
    c->csn_low = low;
    if (!low) {
        /* CSn rising edge — end of transaction */
        c->spi_state = CC1200_SPI_WAITING;
    }
}

void cc1200_set_reset(cc1200_t *c, bool low) {
    if (c->reset_low == low) return;
    c->reset_low = low;
    if (low) {
        /* Held in reset */
        c->marcstate = CC1200_MARC_SLEEP;
        c->marc_pending = CC1200_MARC_SLEEP;
        c->marc_transit_status = 0xFF;
        c->tx_active = false;
        HOST_CANCEL(c, &c->tx_byte_event);
        HOST_CANCEL(c, &c->reset_done_event);
        HOST_CANCEL(c, &c->marcstate_event);
        air_reset(c);
        /* Reset clears all internal signals → propagate drives both
         * GDO pins to whatever the (still-default) IOCFG selectors map
         * to with sig_* all false (typically low). */
        c->sig_pkt_sync_rxtx = false;
        c->sig_rssi_valid    = false;
        c->sig_cs_valid      = false;
        c->sig_carrier_sense = false;
        refresh_status(c);
        propagate_signals(c);
    } else {
        /* Released from reset → IDLE after startup delay */
        chip_reset(c);
    }
}

/* ------------------------------------------------------------------ */
/* Init / pin wiring / RF listener                                       */
/* ------------------------------------------------------------------ */

void cc1200_init(cc1200_t *c, const sim_host_t *host) {
    memset(c, 0, sizeof(*c));
    c->host = host;

    c->tx_byte_event.callback = tx_byte_event_cb;
    c->tx_byte_event.user_data = c;
    c->reset_done_event.callback = reset_done_event_cb;
    c->reset_done_event.user_data = c;
    c->marcstate_event.callback = marcstate_event_cb;
    c->marcstate_event.user_data = c;

    c->csn_low = false;
    c->reset_low = false;
    c->spi_state = CC1200_SPI_WAITING;
    c->rx_rssi = -60;

    /* Default register state matches what the chip looks like out of
     * reset — fixed PARTNUMBER/PARTVERSION + a default sync word. */
    c->regs[CC1200_EXT_PARTNUMBER]  = 0x20;
    c->regs[CC1200_EXT_PARTVERSION] = 0x11;
    c->regs[CC1200_REG_SYNC3] = 0x93;
    c->regs[CC1200_REG_SYNC2] = 0x0B;
    c->regs[CC1200_REG_SYNC1] = 0x51;
    c->regs[CC1200_REG_SYNC0] = 0xDE;
    c->regs[CC1200_REG_PKT_LEN] = 0xFF;
    c->regs[CC1200_REG_IOCFG0] = CC1200_IOCFG_PKT_SYNC_RXTX;

    c->marcstate = CC1200_MARC_IDLE;
    c->marc_pending = CC1200_MARC_IDLE;
    c->marc_transit_status = 0xFF;
    refresh_status(c);
}

void cc1200_set_gdo0_pin(cc1200_t *c, int port, int pin) {
    c->gdo0.port = port;
    c->gdo0.pin = pin;
    c->gdo0.enabled = (port >= 0);
}

void cc1200_set_gdo2_pin(cc1200_t *c, int port, int pin) {
    c->gdo2.port = port;
    c->gdo2.pin = pin;
    c->gdo2.enabled = (port >= 0);
}

void cc1200_set_rf_listener(cc1200_t *c,
                             cc1200_rf_callback_fn cb,
                             void *user_data) {
    c->rf_tx_callback = cb;
    c->rf_tx_user_data = user_data;
}

void cc1200_set_channel_busy_query(cc1200_t *c,
                                    cc1200_channel_busy_fn cb,
                                    void *user_data) {
    c->channel_busy_query = cb;
    c->channel_busy_user_data = user_data;
}
