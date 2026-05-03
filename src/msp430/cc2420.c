/*
 * CC2420 IEEE 802.15.4 radio transceiver emulation
 *
 * Full-fidelity emulation matching Java MSPSim's CC2420.java.
 * Supports SPI register/RAM/FIFO access, TX/RX state machine,
 * address filtering, auto-CRC, auto-ACK, and GPIO pin driving.
 */
#include "cc2420.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Convenience accessors for the CPU-agnostic host vtable. The CC2420
 * driver never touches msp430_cpu_t / msp430_gpio_t directly so the same
 * driver can sit on a different SoC in the future. */
#define HOST_NOW_NS(r)             ((r)->host->now_ns((r)->host->cpu))
#define HOST_SCHEDULE_NS(r, ev, t) ((r)->host->schedule_ns((r)->host->cpu, (ev), (t)))
#define HOST_CANCEL(r, ev)         ((r)->host->cancel((r)->host->cpu, (ev)))
#define HOST_SET_PIN(r, p, n, v)   ((r)->host->set_input_pin((r)->host->gpio, (p), (n), (v)))
#define HOST_FORCE_IRQ(r, p, n, e) ((r)->host->force_irq_edge((r)->host->gpio, (p), (n), (e)))

static int trace_tsch_ack = -1;
static int trace_tsch_ack_lines = 0;
#define TRACE_TSCH_ACK_START_NS 12000000000LL
#define TRACE_TSCH_ACK_END_NS   16000000000LL
#define TRACE_TSCH_ACK_MAX_LINES 2000

static bool trace_tsch_ack_enabled(void) {
    if (trace_tsch_ack < 0) {
        const char *env = getenv("CSIM_TRACE_TSCH_ACK");
        trace_tsch_ack = (env && env[0] && strcmp(env, "0") != 0) ? 1 : 0;
    }
    return trace_tsch_ack != 0;
}

static const char *cc2420_state_str(cc2420_radio_state_t s) {
    switch (s) {
    case CC2420_VREG_OFF: return "VREG_OFF";
    case CC2420_POWER_DOWN: return "POWER_DOWN";
    case CC2420_IDLE: return "IDLE";
    case CC2420_RX_CALIBRATE: return "RX_CAL";
    case CC2420_RX_SFD_SEARCH: return "RX_SFD";
    case CC2420_RX_WAIT: return "RX_WAIT";
    case CC2420_RX_FRAME: return "RX_FRAME";
    case CC2420_RX_OVERFLOW: return "RX_OVERFLOW";
    case CC2420_TX_CALIBRATE: return "TX_CAL";
    case CC2420_TX_PREAMBLE: return "TX_PREAMBLE";
    case CC2420_TX_FRAME: return "TX_FRAME";
    case CC2420_TX_ACK_CALIBRATE: return "TX_ACK_CAL";
    case CC2420_TX_ACK_PREAMBLE: return "TX_ACK_PREAMBLE";
    case CC2420_TX_ACK: return "TX_ACK";
    case CC2420_TX_UNDERFLOW: return "TX_UNDERFLOW";
    default: return "UNKNOWN";
    }
}

/* SHR (Synchronization Header): 4 preamble bytes + SFD */
static const uint8_t SHR[5] = { 0x00, 0x00, 0x00, 0x00, 0x7A };

/* Broadcast address */
static const uint8_t BC_ADDRESS[2] = { 0xFF, 0xFF };

/* ================================================================
 * CRC — CCITT with bit reversal (matches Java CCITT_CRC exactly)
 * ================================================================ */

static uint8_t bitrev(uint8_t data) {
    return (uint8_t)(
        ((data << 7) & 0x80) | ((data << 5) & 0x40) |
        ((data << 3) & 0x20) | ((data << 1) & 0x10) |
        ((data >> 7) & 0x01) | ((data >> 5) & 0x02) |
        ((data >> 3) & 0x04) | ((data >> 1) & 0x08)
    );
}

static uint16_t crc_add(uint16_t crc, uint8_t data) {
    uint16_t newcrc = ((crc >> 8) & 0xff) | ((crc << 8) & 0xffff);
    newcrc ^= data;
    newcrc ^= (newcrc & 0xff) >> 4;
    newcrc ^= (newcrc << 12) & 0xffff;
    newcrc ^= (newcrc & 0xff) << 5;
    return newcrc & 0xffff;
}

static uint16_t crc_add_bitrev(uint16_t crc, uint8_t data) {
    return crc_add(crc, bitrev(data));
}

/* ================================================================
 * RXFIFO helpers (circular buffer in memory[0x80..0xFF])
 * ================================================================ */

static void rxfifo_write(cc2420_t *r, uint8_t byte) {
    r->memory[CC2420_RAM_RXFIFO + r->rx_fifo_write_pos] = byte;
    r->rx_fifo_write_pos = (r->rx_fifo_write_pos + 1) & 0x7F;
    r->rx_fifo_len++;
}

static uint8_t rxfifo_read(cc2420_t *r) {
    uint8_t val = r->memory[CC2420_RAM_RXFIFO + r->rx_fifo_read_pos];
    r->rx_fifo_read_pos = (r->rx_fifo_read_pos + 1) & 0x7F;
    if (r->rx_fifo_len > 0) r->rx_fifo_len--;
    return val;
}

/* Read byte at offset relative to current write position (negative = backwards) */
static uint8_t rxfifo_get(cc2420_t *r, int offset) {
    int pos = (r->rx_fifo_write_pos + offset) & 0x7F;
    return r->memory[CC2420_RAM_RXFIFO + pos];
}

/* Overwrite byte at offset relative to current write position */
static void rxfifo_set(cc2420_t *r, int offset, uint8_t val) {
    int pos = (r->rx_fifo_write_pos + offset) & 0x7F;
    r->memory[CC2420_RAM_RXFIFO + pos] = val;
}

static bool rxfifo_full(cc2420_t *r) {
    return r->rx_fifo_len >= 128;
}

/* Compare tail of RXFIFO against reference data.
 * tail_offset: how many bytes back from write pos to start comparing
 * (e.g., 2 means the last 2 bytes written) */
static bool rxfifo_tail_equals(cc2420_t *r, const uint8_t *ref, int ref_len, int tail_offset) {
    for (int i = 0; i < ref_len; i++) {
        int pos = (r->rx_fifo_write_pos - tail_offset + i) & 0x7F;
        if (r->memory[CC2420_RAM_RXFIFO + pos] != ref[i])
            return false;
    }
    return true;
}

/* Save FIFO mark for frame rejection (rewind) */
static void rxfifo_mark(cc2420_t *r) {
    r->rx_fifo_mark_pos = r->rx_fifo_write_pos;
    r->rx_fifo_mark_len = r->rx_fifo_len;
}

static void rxfifo_restore(cc2420_t *r) {
    r->rx_fifo_write_pos = r->rx_fifo_mark_pos;
    r->rx_fifo_len = r->rx_fifo_mark_len;
}

/* Debug counters (defined in RX stats section below) */
static int fifop_high_count;
static int rxfifo_spi_reads;
static int spi_exchange_count;
static int rx_bytes_buffered;
static int rx_bytes_replayed;
static int rx_bytes_direct;
static int rx_bytes_dropped;
static int auto_ack_count;
static int sfd_high_count;
static int sfd_low_count;
static int sfd_deferred_count;

/* ================================================================
 * Pin control
 * ================================================================ */

static void set_fifop(cc2420_t *r, bool val) {
    if (val && !r->current_fifop)
        fifop_high_count++;
    r->current_fifop = val;
    bool pin_val = val;
    if (r->registers[CC2420_REG_IOCFG0] & CC2420_FIFOP_POLARITY)
        pin_val = !pin_val;
    HOST_SET_PIN(r, r->fifop_port, r->fifop_pin, pin_val);

    /* For Cooja-built firmware: ensure FIFOP interrupt fires reliably.
     * MSPSim's CC2420 calls IOPort.setPinState() which directly updates
     * IFG on rising edge and calls updateIV(). We emulate this by force-
     * enabling IE for the pin and pulsing IFG on rising/falling edges
     * (clearing on falling avoids an infinite ISR loop on Z1's shared
     * port1_isr where the CC2420 path doesn't clear P1IFG). */
    HOST_FORCE_IRQ(r, r->fifop_port, r->fifop_pin, val);
}

static void set_fifo(cc2420_t *r, bool val) {
    r->current_fifo = val;
    bool pin_val = val;
    if (r->registers[CC2420_REG_IOCFG0] & CC2420_FIFO_POLARITY)
        pin_val = !pin_val;
    HOST_SET_PIN(r, r->fifo_port, r->fifo_pin, pin_val);
}

static void set_sfd(cc2420_t *r, bool val) {
    r->current_sfd = val;
    if (val) sfd_high_count++; else sfd_low_count++;
    bool pin_val = val;
    if (r->registers[CC2420_REG_IOCFG0] & CC2420_SFD_POLARITY)
        pin_val = !pin_val;
    HOST_SET_PIN(r, r->sfd_port, r->sfd_pin, pin_val);
    /* Notify Timer B capture (SFD timestamping for TSCH) */
    if (r->sfd_callback)
        r->sfd_callback(r->sfd_callback_data, pin_val);
}

static void set_cca(cc2420_t *r, bool val) {
    r->current_cca = val;
    bool pin_val = val;
    if (r->registers[CC2420_REG_IOCFG0] & CC2420_CCA_POLARITY)
        pin_val = !pin_val;
    HOST_SET_PIN(r, r->cca_port, r->cca_pin, pin_val);
}

static void update_cca(cc2420_t *r) {
    int ccamux = r->registers[CC2420_REG_IOCFG1] & CC2420_CCAMUX_MASK;
    if (ccamux == CC2420_CCAMUX_XOSC16M) {
        set_cca(r, (r->status & CC2420_STATUS_XOSC16M_STABLE) != 0);
    } else {
        /* Normal CCA: based on RSSI comparison */
        bool rssi_valid = (r->status & CC2420_STATUS_RSSI_VALID) != 0;
        if (rssi_valid) {
            int8_t rssi_val = (int8_t)(r->registers[CC2420_REG_RSSI] & 0xFF);
            int8_t rssi_thr = (int8_t)((r->registers[CC2420_REG_RSSI] >> 8) & 0xFF);
            set_cca(r, rssi_val < rssi_thr);
        } else {
            set_cca(r, false);
        }
    }
}

/* ================================================================
 * State transitions
 * ================================================================ */

static void set_state(cc2420_t *r, cc2420_radio_state_t new_state);

static void flush_rx(cc2420_t *r) {
    r->rx_fifo_write_pos = 0;
    r->rx_fifo_read_pos = 0;
    r->rx_fifo_len = 0;
    r->rx_fifo_read_left = 0;
    r->rxlen = 0;
    r->rxread = 0;
    r->overflow = false;
    r->frame_rejected = false;
    r->rx_incoming_count = 0;
    HOST_CANCEL(r,&r->sfd_clear_event);
    set_fifo(r, false);
    set_fifop(r, false);
    set_sfd(r, false);
    /* Match MSPSim: transition back to RX_SFD_SEARCH if in any RX state.
     * Without this, the radio stays stuck in RX_FRAME/RX_OVERFLOW after
     * SFLUSHRX and can never receive new frames. */
    if (r->state == CC2420_RX_CALIBRATE ||
        r->state == CC2420_RX_SFD_SEARCH ||
        r->state == CC2420_RX_FRAME ||
        r->state == CC2420_RX_OVERFLOW ||
        r->state == CC2420_RX_WAIT) {
        HOST_CANCEL(r,&r->symbol_event);
        set_state(r, CC2420_RX_SFD_SEARCH);
    }
}

static void flush_tx(cc2420_t *r) {
    r->tx_cursor = 0;
    r->tx_fifo_flush = false;
}

static void set_rx_overflow(cc2420_t *r) {
    set_fifop(r, true);
    set_fifo(r, false);
    set_sfd(r, false);
    r->overflow = true;
    r->should_ack = false;
    r->state = CC2420_RX_OVERFLOW;
}

static void reject_frame(cc2420_t *r) {
    rxfifo_restore(r);
    set_sfd(r, false);
    set_fifo(r, r->rx_fifo_len > 0);
    r->frame_rejected = true;
}

/* Deferred SFD clear: fires 1 symbol after frame completion so that
 * the firmware's SFD polling loop can observe SFD=1 during the CPU step.
 * In our batch-delivery model, frame bytes are processed atomically before
 * the step, so without this deferral, SFD goes high→low with 0 CPU cycles
 * in between, making ACK reception invisible to CC2420 firmware drivers. */
static void sfd_clear_callback(void *user_data, cpu_event_t *event) {
    cc2420_t *r = (cc2420_t *)user_data;
    (void)event;
    if (r->current_sfd)
        set_sfd(r, false);
}

/* Forward declarations for event callbacks */
static void shr_next(cc2420_t *r);
static void tx_next(cc2420_t *r);
static void ack_next(cc2420_t *r);

static void symbol_event_callback(void *user_data, cpu_event_t *event) {
    cc2420_t *r = (cc2420_t *)user_data;
    (void)event;

    switch (r->state) {
    case CC2420_RX_CALIBRATE:
        set_state(r, CC2420_RX_SFD_SEARCH);
        /* Buffered bytes are replayed inside set_state(RX_SFD_SEARCH) */
        break;
    case CC2420_TX_CALIBRATE:
        set_state(r, CC2420_TX_PREAMBLE);
        break;
    case CC2420_TX_PREAMBLE:
        shr_next(r);
        break;
    case CC2420_TX_FRAME:
        tx_next(r);
        break;
    case CC2420_TX_ACK_CALIBRATE:
        set_state(r, CC2420_TX_ACK_PREAMBLE);
        break;
    case CC2420_TX_ACK_PREAMBLE:
        shr_next(r);
        break;
    case CC2420_TX_ACK:
        ack_next(r);
        break;
    case CC2420_RX_WAIT:
        set_state(r, CC2420_RX_SFD_SEARCH);
        /* Buffered bytes are replayed inside set_state(RX_SFD_SEARCH) */
        break;
    default:
        break;
    }
}

/* CC2420 timing constants in nanoseconds (independent of CPU clock) */
#define CC2420_SYMBOL_PERIOD_NS  16000   /* 62.5 ksym/s = 16 us/symbol */
#define CC2420_BYTE_PERIOD_NS    32000   /* 2 symbols per byte = 32 us */

static void schedule_symbols(cc2420_t *r, int symbols) {
    int64_t delay_ns = (int64_t)symbols * CC2420_SYMBOL_PERIOD_NS;
    int64_t fire_ns = HOST_NOW_NS(r) + delay_ns;
    HOST_SCHEDULE_NS(r, &r->symbol_event, fire_ns);
}

static void set_state(cc2420_t *r, cc2420_radio_state_t new_state) {
    cc2420_radio_state_t old_state = r->state;
    r->state = new_state;

    /* CSIM_TRACE_RADIO hook: log every state transition for chip-level
     * debugging. Cheap when disabled (one TLS bool check).
     *
     * Note on timestamps: HOST_NOW_NS reads cpu->sim_time_ns, which is
     * cycle-derived inside execute_events callbacks (the harness's pin
     * to scheduler-time only persists until the first chip event fires).
     * The printed t= is therefore the chip's view of "now" rather than
     * the harness's wall clock, and may lag behind it by a slice's worth
     * of cycle vs. wall-clock drift.  This is fine for debugging chip
     * sequencing — the deltas between transitions are still correct —
     * but don't compare these timestamps to current_sim_ns directly. */
    extern int csim_radio_trace_enabled(void);
    if (csim_radio_trace_enabled() && old_state != new_state) {
        fprintf(stderr, "[t=%.6fs] cc2420 node=%d state %s -> %s\n",
                (double)HOST_NOW_NS(r) / 1e9, r->node_id,
                cc2420_state_str(old_state), cc2420_state_str(new_state));
    }

    if (trace_tsch_ack_enabled() && trace_tsch_ack_lines < TRACE_TSCH_ACK_MAX_LINES &&
        r->node_id > 0 && r->node_id <= 2 &&
        old_state != new_state &&
        HOST_NOW_NS(r) >= TRACE_TSCH_ACK_START_NS &&
        HOST_NOW_NS(r) <= TRACE_TSCH_ACK_END_NS) {
        fprintf(stderr, "  [TRACE] %7.3f cc2420 node=%d state %s -> %s\n",
                (double)HOST_NOW_NS(r) / 1e9, r->node_id,
                cc2420_state_str(old_state), cc2420_state_str(new_state));
        trace_tsch_ack_lines++;
    }

    switch (new_state) {
    case CC2420_VREG_OFF:
    case CC2420_POWER_DOWN:
        r->status &= ~CC2420_STATUS_XOSC16M_STABLE;
        r->status &= ~CC2420_STATUS_RSSI_VALID;
        r->rx_incoming_count = 0;
        update_cca(r);
        break;

    case CC2420_IDLE:
        r->status &= ~CC2420_STATUS_RSSI_VALID;
        update_cca(r);
        break;

    case CC2420_RX_CALIBRATE:
        /* 12 symbol periods to calibrate */
        schedule_symbols(r, 12);
        break;

    case CC2420_RX_SFD_SEARCH:
        r->zero_symbols = 0;
        r->rxread = 0;
        r->rxlen = 0;
        r->frame_rejected = false;
        r->should_ack = false;
        /* RSSI valid after 8 symbols — set immediately for simplicity */
        r->status |= CC2420_STATUS_RSSI_VALID;
        /* Set noise floor RSSI: -100 dBm = -100 - RSSI_OFFSET(-45) = -55 */
        r->registers[CC2420_REG_RSSI] =
            (r->registers[CC2420_REG_RSSI] & 0xFF00) | (uint8_t)(int8_t)-55;
        update_cca(r);
        /* Replay any bytes buffered while in non-RX states */
        if (r->rx_incoming_count > 0) {
            int count = r->rx_incoming_count;
            r->rx_incoming_count = 0;
            rx_bytes_replayed += count;
            r->stat_rx_replayed += count;
            for (int i = 0; i < count; i++)
                cc2420_receive_byte(r, r->rx_incoming[i]);
        }
        break;

    case CC2420_RX_FRAME:
        r->rxread = 0;
        r->rxlen = 0;
        r->frame_rejected = false;
        r->should_ack = false;
        r->crc_ok = false;
        rxfifo_mark(r);
        break;

    case CC2420_RX_WAIT:
        /* 8 symbol periods before returning to SFD search (matches Java) */
        schedule_symbols(r, 8);
        break;

    case CC2420_TX_CALIBRATE:
        /* Cancel any pending deferred SFD clear from frame RX */
        HOST_CANCEL(r,&r->sfd_clear_event);
        /* 14 symbol periods (12 cal + 2 settling) */
        schedule_symbols(r, 14);
        break;

    case CC2420_TX_PREAMBLE:
        r->shr_pos = 0;
        shr_next(r);
        break;

    case CC2420_TX_FRAME:
        r->tx_fifo_pos = 0;
        r->crc_ok = false;
        /* Match MSPSim's event ordering: once SHR completes, the first
         * frame byte is emitted on the next radio byte event, not at the
         * same instant as SFD. */
        schedule_symbols(r, 2);
        break;

    case CC2420_TX_ACK_CALIBRATE:
        /* Cancel any pending deferred SFD clear — ACK TX will manage SFD */
        HOST_CANCEL(r,&r->sfd_clear_event);
        set_sfd(r, false);  /* clear SFD from received frame before ACK TX */
        r->status |= CC2420_STATUS_TX_ACTIVE;
        /* Match MSPSim: 12 turnaround + 2 extra + 2 extra. */
        schedule_symbols(r, 16);
        break;

    case CC2420_TX_ACK_PREAMBLE:
        r->shr_pos = 0;
        r->ack_pos = 0;
        shr_next(r);
        break;

    case CC2420_TX_ACK:
        /* Same ordering as normal TX: after SFD, emit the first ACK byte
         * on the next byte event so receivers observe a distinct length
         * byte after the SHR. */
        schedule_symbols(r, 2);
        break;

    default:
        break;
    }
}

/* ================================================================
 * TX path
 * ================================================================ */

static void shr_next(cc2420_t *r) {
    if (r->shr_pos >= 5) {
        /* SHR complete: set SFD and transition */
        set_sfd(r, true);
        if (r->state == CC2420_TX_PREAMBLE)
            set_state(r, CC2420_TX_FRAME);
        else if (r->state == CC2420_TX_ACK_PREAMBLE)
            set_state(r, CC2420_TX_ACK);
    } else {
        if (r->rf_tx_callback)
            r->rf_tx_callback(r->rf_tx_data, SHR[r->shr_pos]);
        r->shr_pos++;
        /* 2 symbol periods per byte */
        schedule_symbols(r, 2);
    }
}

static void tx_next(cc2420_t *r) {
    int len = r->memory[CC2420_RAM_TXFIFO] & 0xFF;

    if (r->tx_fifo_pos <= len) {
        /* Auto-CRC: compute and insert just before sending last 2 bytes */
        if (r->auto_crc && r->tx_fifo_pos == len - 1) {
            uint16_t crc = 0;
            for (int i = 1; i < len - 1; i++)
                crc = crc_add_bitrev(crc, r->memory[CC2420_RAM_TXFIFO + i]);
            r->memory[CC2420_RAM_TXFIFO + len - 1] = bitrev((crc >> 8) & 0xFF);
            r->memory[CC2420_RAM_TXFIFO + len]     = bitrev(crc & 0xFF);
        }

        uint8_t byte = r->memory[CC2420_RAM_TXFIFO + (r->tx_fifo_pos & 0x7F)];
        if (r->rf_tx_callback)
            r->rf_tx_callback(r->rf_tx_data, byte);
        r->tx_fifo_pos++;
        schedule_symbols(r, 2);
    } else {
        /* TX complete */
        r->status &= ~CC2420_STATUS_TX_ACTIVE;
        set_sfd(r, false);
        r->tx_fifo_flush = true;
        if (r->overflow)
            set_state(r, CC2420_RX_OVERFLOW);
        else
            set_state(r, CC2420_RX_CALIBRATE);
    }
}

static void ack_next(cc2420_t *r) {
    if (r->ack_pos == 0) {
        /* Build ACK frame: length=5, FCF=0x0002, DSN, CRC */
        r->ack_buf[0] = 5;   /* length */
        r->ack_buf[1] = 0x02; /* FCF low: ACK frame type */
        r->ack_buf[2] = 0x00; /* FCF high */
        if (r->ack_frame_pending)
            r->ack_buf[1] |= 0x10;  /* frame pending bit */
        r->ack_buf[3] = (uint8_t)r->dsn;

        /* Compute CRC over bytes 1-3 */
        uint16_t crc = 0;
        for (int i = 1; i <= 3; i++)
            crc = crc_add_bitrev(crc, r->ack_buf[i]);
        r->ack_buf[4] = bitrev((crc >> 8) & 0xFF);
        r->ack_buf[5] = bitrev(crc & 0xFF);
    }

    if (r->ack_pos < 6) {
        if (r->rf_tx_callback)
            r->rf_tx_callback(r->rf_tx_data, r->ack_buf[r->ack_pos]);
        r->ack_pos++;
        schedule_symbols(r, 2);
    } else {
        /* ACK complete */
        r->status &= ~CC2420_STATUS_TX_ACTIVE;
        set_sfd(r, false);
        r->ack_frame_pending = false;
        set_state(r, CC2420_RX_CALIBRATE);
    }
}

/* ================================================================
 * RX path
 * ================================================================ */

static int rx_frames_started = 0;
static int rx_frames_completed = 0;
static int rx_frames_rejected = 0;
static int rx_frames_overflow = 0;
static int rx_crc_ok = 0;
static int rx_crc_fail = 0;
/* fifop_high_count, rxfifo_spi_reads, spi_exchange_count declared above */

void cc2420_get_rx_stats(int *started, int *completed, int *rejected,
                          int *overflowed, int *crc_good, int *crc_bad,
                          int *dropped) {
    *started = rx_frames_started;
    *completed = rx_frames_completed;
    *rejected = rx_frames_rejected;
    *overflowed = rx_frames_overflow;
    *crc_good = rx_crc_ok;
    *crc_bad = rx_crc_fail;
    *dropped = rx_bytes_dropped;
}

int cc2420_get_auto_ack_count(void) { return auto_ack_count; }

void cc2420_receive_byte(cc2420_t *radio, uint8_t data) {
    cc2420_t *r = radio;

    if (r->state != CC2420_RX_SFD_SEARCH && r->state != CC2420_RX_FRAME) {
        /* Match MSPSim CC2420.receivedByte(): bytes that arrive while the
         * radio is not actively searching for SFD / receiving a frame are
         * ignored, not replayed later from a side buffer. */
        if (trace_tsch_ack_enabled() && trace_tsch_ack_lines < TRACE_TSCH_ACK_MAX_LINES &&
            r->node_id > 0 && r->node_id <= 2 &&
            HOST_NOW_NS(r) >= TRACE_TSCH_ACK_START_NS &&
            HOST_NOW_NS(r) <= TRACE_TSCH_ACK_END_NS) {
            fprintf(stderr, "  [TRACE] %7.3f cc2420 node=%d drop byte=%02x state=%s\n",
                    (double)HOST_NOW_NS(r) / 1e9, r->node_id, data,
                    cc2420_state_str(r->state));
            trace_tsch_ack_lines++;
        }
        rx_bytes_dropped++;
        r->stat_rx_dropped++;
        return;
    }

    rx_bytes_direct++;

    if (r->state == CC2420_RX_SFD_SEARCH) {
        if (data == 0) {
            r->zero_symbols++;
        } else if (r->zero_symbols >= 4 && data == 0x7A) {
            /* Preamble + SFD detected */
            rx_frames_started++;
            r->stat_rx_started++;
            set_sfd(r, true);
            set_state(r, CC2420_RX_FRAME);
        } else {
            r->zero_symbols = 0;
        }
    } else if (r->state == CC2420_RX_FRAME) {
        if (r->overflow) { rx_bytes_dropped++; r->stat_rx_dropped++; return; }
        if (rxfifo_full(r)) {
            rx_frames_overflow++;
            r->stat_rx_overflow++;
            set_rx_overflow(r);
            return;
        }

        if (!r->frame_rejected) {
            rxfifo_write(r, data);

            if (r->rxread == 0) {
                /* Length byte */
                r->rxlen = data & 0xFF;
                r->decode_address = r->adr_decode;
                set_fifo(r, true);
            } else if (r->rxread < r->rxlen - 1) {
                /* Payload bytes — accumulate CRC */
                /* (CRC state stored inline — use running computation) */

                if (r->rxread == 1) {
                    r->fcf0 = data & 0xFF;
                    r->frame_type = r->fcf0 & CC2420_FRAME_TYPE_MASK;
                } else if (r->rxread == 2) {
                    r->fcf1 = data & 0xFF;
                    if (r->frame_type == CC2420_FRAME_TYPE_DATA ||
                        r->frame_type == CC2420_FRAME_TYPE_CMD) {
                        r->ack_request = (r->fcf0 & CC2420_ACK_REQUEST) != 0;
                        r->dest_addr_mode = (r->fcf1 >> 2) & 3;
                        if (r->adr_decode &&
                            r->dest_addr_mode != CC2420_ADDR_MODE_SHORT &&
                            r->dest_addr_mode != CC2420_ADDR_MODE_LONG) {
                            reject_frame(r);
                        }
                    } else if (r->frame_type == CC2420_FRAME_TYPE_BEACON ||
                               r->frame_type == CC2420_FRAME_TYPE_ACK) {
                        r->decode_address = false;
                        r->ack_request = false;
                    } else if (r->adr_decode) {
                        reject_frame(r);
                    }
                } else if (r->rxread == 3) {
                    r->dsn = data & 0xFF;
                } else if (r->decode_address) {
                    /* Address filtering */
                    if (r->dest_addr_mode == CC2420_ADDR_MODE_LONG &&
                        r->rxread == 13) {  /* 8+5 */
                        /* Check IEEE address (last 8 bytes) */
                        bool addr_match = rxfifo_tail_equals(r,
                            &r->memory[CC2420_RAM_IEEEADDR], 8, 8);
                        /* Check PAN ID (2 bytes before the 8-byte addr) */
                        bool pan_match = rxfifo_tail_equals(r,
                            &r->memory[CC2420_RAM_PANID], 2, 10) ||
                            rxfifo_tail_equals(r, BC_ADDRESS, 2, 10);
                        if (!addr_match || !pan_match)
                            reject_frame(r);
                        r->decode_address = false;
                    } else if (r->dest_addr_mode == CC2420_ADDR_MODE_SHORT &&
                               r->rxread == 7) {  /* 2+5 */
                        /* Check short address (last 2 bytes) */
                        bool addr_match = rxfifo_tail_equals(r,
                            &r->memory[CC2420_RAM_SHORTADDR], 2, 2) ||
                            rxfifo_tail_equals(r, BC_ADDRESS, 2, 2);
                        /* Check PAN ID (2 bytes before short addr) */
                        bool pan_match = rxfifo_tail_equals(r,
                            &r->memory[CC2420_RAM_PANID], 2, 4) ||
                            rxfifo_tail_equals(r, BC_ADDRESS, 2, 4);
                        if (!addr_match || !pan_match)
                            reject_frame(r);
                        r->decode_address = false;
                    }
                }
            }

            /* In real hardware, FIFOP fires mid-frame when rx_fifo_len exceeds
             * fifop_thr. But in our time-stepped simulation, frame bytes arrive
             * in batches (per time step), not one-at-a-time.  If FIFOP fires
             * mid-frame, the firmware reads a partial RXFIFO, gets CRC failure,
             * and then detects FIFOP=1 + FIFO=0 as overflow → flushes RX.
             * So we defer FIFOP to frame completion (handled below). */
        }

        if (r->rxread++ == r->rxlen) {
            /* End of frame */
            if (r->frame_rejected) {
                rx_frames_rejected++;
                r->stat_rx_rejected++;
                set_sfd(r, false);
                set_state(r, CC2420_RX_WAIT);
                return;
            }

            /* CRC check: compute over bytes 1..(rxlen-2) in RXFIFO */
            uint16_t rx_crc = 0;
            /* Walk the FIFO backwards to find frame start */
            int frame_start = (r->rx_fifo_write_pos - (r->rxlen + 1)) & 0x7F;
            for (int i = 1; i < r->rxlen - 1; i++) {
                int pos = (frame_start + i) & 0x7F;
                rx_crc = crc_add_bitrev(rx_crc, r->memory[CC2420_RAM_RXFIFO + pos]);
            }

            /* Compare with received CRC (last 2 bytes) */
            uint8_t crc_hi_recv = rxfifo_get(r, -2);
            uint8_t crc_lo_recv = rxfifo_get(r, -1);
            uint16_t recv_crc = (crc_hi_recv << 8) | crc_lo_recv;
            uint16_t calc_crc_lo = bitrev(rx_crc & 0xFF);
            uint16_t calc_crc_hi = bitrev((rx_crc >> 8) & 0xFF);
            uint16_t calc_crc_bitrev = calc_crc_lo | (calc_crc_hi << 8);
            r->crc_ok = (recv_crc == calc_crc_bitrev);
            if (r->crc_ok) { rx_crc_ok++; r->stat_crc_ok++; }
            else { rx_crc_fail++; r->stat_crc_fail++; }
            rx_frames_completed++;
            r->stat_rx_completed++;
            if (r->frame_type == CC2420_FRAME_TYPE_ACK) {
                r->stat_rx_ack_completed++;
            }

            /* Replace last 2 bytes with RSSI + (corrval | crc_ok<<7) */
            uint8_t rssi_val = (uint8_t)(int8_t)r->rx_rssi;
            /* Derive correlation from RSSI: map [-10..-90] -> [110..50]
             * dist_ratio = -(rssi + 10) / 80; corr = 110 - dist_ratio * 60 */
            int corr = 110 + ((int)r->rx_rssi + 10) * 60 / 80;
            if (corr < 50) corr = 50;
            if (corr > 110) corr = 110;
            rxfifo_set(r, -2, rssi_val);
            rxfifo_set(r, -1, (uint8_t)((corr & 0x7F) | (r->crc_ok ? 0x80 : 0)));

            /* Set FIFOP for completed packet */
            if (r->rx_fifo_len <= r->rxlen + 1)
                set_fifop(r, true);

            /* Defer SFD clear: schedule 1 symbol period later so the firmware
             * can observe SFD=1 during the CPU step.  This is critical for
             * ACK detection — the CC2420 driver busy-waits for SFD transitions. */
            {
                sfd_deferred_count++;
                int64_t fire_ns = HOST_NOW_NS(r) + CC2420_SYMBOL_PERIOD_NS;
                HOST_SCHEDULE_NS(r, &r->sfd_clear_event, fire_ns);
            }

            /* Match MSPSim: a good ACK-requesting frame transitions the
             * receiver radio state machine into TX_ACK_CALIBRATE. ACK bytes
             * are emitted by ack_next() during subsequent CPU stepping, not
             * synthesized inline here. */
            if (((r->auto_ack && r->ack_request) || r->should_ack) && r->crc_ok) {
                auto_ack_count++;
                r->stat_auto_ack++;
                set_state(r, CC2420_TX_ACK_CALIBRATE);
            } else {
                set_state(r, CC2420_RX_WAIT);
            }
        }
    }
}

/* ================================================================
 * Register write handling
 * ================================================================ */

static void set_reg(cc2420_t *r, int addr, uint16_t value) {
    /* RSSI_VAL (low byte of RSSI register) is read-only hardware.
     * Preserve it when firmware writes to set CCA_THR (high byte). */
    if (addr == CC2420_REG_RSSI) {
        value = (value & 0xFF00) | (r->registers[CC2420_REG_RSSI] & 0x00FF);
    }
    r->registers[addr] = value;

    switch (addr) {
    case CC2420_REG_MDMCTRL0:
        r->adr_decode = (value & CC2420_ADR_DECODE) != 0;
        r->auto_crc = (value & CC2420_ADR_AUTOCRC) != 0;
        r->auto_ack = (value & CC2420_AUTOACK) != 0;
        break;

    case CC2420_REG_IOCFG0:
        r->fifop_thr = value & CC2420_FIFOP_THR_MASK;
        /* Re-apply pin polarities */
        set_fifop(r, r->current_fifop);
        set_fifo(r, r->current_fifo);
        set_sfd(r, r->current_sfd);
        set_cca(r, r->current_cca);
        break;

    case CC2420_REG_IOCFG1:
        update_cca(r);
        break;

    case CC2420_REG_RSSI:
        update_cca(r);
        break;

    case CC2420_REG_FSCTRL: {
        /* CC2420 FSCTRL[9:0] = FREQ. Channel mapping per datasheet
         * §13: f_RF = 2048 + FREQ (MHz), and IEEE channel k uses
         * 2405 + 5*(k-11) MHz, so:
         *     FREQ = 357 + 5*(k - 11)
         *     k    = (FREQ - 357) / 5 + 11
         * Push the channel change to the radio medium so per-radio
         * routing can gate delivery.  The chip driver itself stays
         * portable — channel goes out via the sim_host_t vtable. */
        int freq = value & 0x3FF;
        int channel = -1;
        if (freq >= 357) channel = (freq - 357) / 5 + 11;
        if (r->host && r->host->radio_set_channel)
            r->host->radio_set_channel(r->host->radio_user_data, 0, channel);
        break;
    }

    default:
        break;
    }
}

/* ================================================================
 * Strobe commands
 * ================================================================ */

static void start_oscillator(cc2420_t *r);
static void stop_oscillator(cc2420_t *r);


static void strobe(cc2420_t *r, int cmd) {
    /* In POWER_DOWN, only SXOSCON is accepted */
    if (r->state == CC2420_POWER_DOWN && cmd != CC2420_REG_SXOSCON)
        return;

    switch (cmd) {
    case CC2420_REG_SNOP:
        break;

    case CC2420_REG_SXOSCON:
        start_oscillator(r);
        break;

    case CC2420_REG_SRXON:
        r->stat_strobe_srxon++;
        if (r->state == CC2420_IDLE)
            set_state(r, CC2420_RX_CALIBRATE);
        break;

    case CC2420_REG_STXON:
        r->stat_strobe_stxon++;
        if (r->state == CC2420_IDLE ||
            r->state == CC2420_RX_CALIBRATE ||
            r->state == CC2420_RX_SFD_SEARCH ||
            r->state == CC2420_RX_FRAME ||
            r->state == CC2420_RX_OVERFLOW ||
            r->state == CC2420_RX_WAIT) {
            r->status |= CC2420_STATUS_TX_ACTIVE;
            r->stat_tx_calibrate++;
            set_state(r, CC2420_TX_CALIBRATE);
        }
        break;

    case CC2420_REG_STXONCCA:
        r->stat_strobe_stxoncca++;
        if (r->state == CC2420_RX_CALIBRATE ||
            r->state == CC2420_RX_SFD_SEARCH ||
            r->state == CC2420_RX_FRAME ||
            r->state == CC2420_RX_OVERFLOW ||
            r->state == CC2420_RX_WAIT) {
            if (r->current_cca) {
                r->status |= CC2420_STATUS_TX_ACTIVE;
                r->stat_tx_calibrate++;
                set_state(r, CC2420_TX_CALIBRATE);
            }
        }
        break;

    case CC2420_REG_SRFOFF:
        r->stat_strobe_srfoff++;
        set_state(r, CC2420_IDLE);
        break;

    case CC2420_REG_SXOSCOFF:
        stop_oscillator(r);
        break;

    case CC2420_REG_SFLUSHRX:
        flush_rx(r);
        break;

    case CC2420_REG_SFLUSHTX:
        flush_tx(r);
        break;

    case CC2420_REG_SACK:
    case CC2420_REG_SACKPEND:
        r->ack_frame_pending = (cmd == CC2420_REG_SACKPEND);
        if (r->state == CC2420_RX_FRAME) {
            r->should_ack = true;
        } else if (r->crc_ok) {
            set_state(r, CC2420_TX_ACK_CALIBRATE);
        }
        break;

    default:
        break;
    }
}

/* ================================================================
 * Oscillator / VREG control
 * ================================================================ */

static void vreg_event_callback(void *user_data, cpu_event_t *event) {
    cc2420_t *r = (cc2420_t *)user_data;
    (void)event;
    /* VREG startup complete — radio is now in POWER_DOWN */
    r->on = true;
    set_state(r, CC2420_POWER_DOWN);
}

static void oscillator_event_callback(void *user_data, cpu_event_t *event) {
    cc2420_t *r = (cc2420_t *)user_data;
    (void)event;
    /* Oscillator stable — transition to IDLE */
    r->status |= CC2420_STATUS_XOSC16M_STABLE;
    set_state(r, CC2420_IDLE);
    update_cca(r);
}

static void start_oscillator(cc2420_t *r) {
    /* If oscillator is already stable, just stay in current state */
    if (r->status & CC2420_STATUS_XOSC16M_STABLE)
        return;
    /* ~1ms startup delay */
    int64_t fire_ns = HOST_NOW_NS(r) + 1000000LL;  /* 1ms = 1,000,000 ns */
    HOST_SCHEDULE_NS(r, &r->oscillator_event, fire_ns);
}

static void stop_oscillator(cc2420_t *r) {
    r->status &= ~CC2420_STATUS_XOSC16M_STABLE;
    set_state(r, CC2420_POWER_DOWN);
    update_cca(r);
}

void cc2420_set_vreg(cc2420_t *radio, bool on) {
    if (on && !radio->on) {
        /* VREG turning on — firmware handles delay with clock_delay(),
         * so we activate immediately to avoid SXOSCON being ignored */
        radio->on = true;
        set_state(radio, CC2420_POWER_DOWN);
    } else if (!on && radio->on) {
        /* VREG off — immediate */
        radio->on = false;
        radio->status = 0;
        HOST_CANCEL(radio, &radio->vreg_event);
        HOST_CANCEL(radio, &radio->oscillator_event);
        HOST_CANCEL(radio, &radio->symbol_event);
        HOST_CANCEL(radio, &radio->sfd_clear_event);
        set_state(radio, CC2420_VREG_OFF);
    }
}

/* ================================================================
 * SPI protocol
 * ================================================================ */

void cc2420_set_chip_select(cc2420_t *radio, bool select) {
    if (radio->chip_select && !select) {
        /* Chip deselect: if mid-register-write, commit partial write */
        if (radio->spi_state == CC2420_SPI_WRITE_REGISTER &&
            radio->spi_data_pos == 1) {
            uint16_t val = (radio->spi_data_value & 0xFF00) |
                           (radio->registers[radio->spi_address] & 0x00FF);
            set_reg(radio, radio->spi_address, val);
        }
        radio->spi_state = CC2420_SPI_WAITING;
    }
    radio->chip_select = select;
}

uint8_t cc2420_spi_exchange(cc2420_t *radio, uint8_t data) {
    cc2420_t *r = radio;
    uint8_t old_status = r->status;

    spi_exchange_count++;
    r->stat_spi_count++;
    if (!r->chip_select) return 0;
    if (r->state == CC2420_VREG_OFF) return 0;

    switch (r->spi_state) {
    case CC2420_SPI_WAITING: {
        /* First byte: command/address */
        if (data & CC2420_FLAG_RAM) {
            r->spi_state = CC2420_SPI_RAM_ACCESS;
            r->spi_address = data & 0x7F;
            r->spi_data_pos = 0;
        } else {
            r->spi_address = data & 0x3F;

            if (r->spi_address == CC2420_REG_RXFIFO) {
                r->spi_state = CC2420_SPI_READ_RXFIFO;
            } else if (r->spi_address == CC2420_REG_TXFIFO) {
                r->spi_state = CC2420_SPI_WRITE_TXFIFO;
            } else if (data & CC2420_FLAG_READ) {
                r->spi_state = CC2420_SPI_READ_REGISTER;
            } else {
                r->spi_state = CC2420_SPI_WRITE_REGISTER;
            }

            /* Strobe commands: address 0x00-0x0E with no RAM flag */
            if (data < 0x0F) {
                strobe(r, data);
                r->spi_state = CC2420_SPI_WAITING;
            }
        }
        r->spi_data_pos = 0;
        /* Return status AFTER strobe processing (matches Java/real HW) */
        return r->status;
    }

    case CC2420_SPI_WRITE_REGISTER:
        if (r->spi_data_pos == 0) {
            uint8_t ret = (r->registers[r->spi_address] >> 8) & 0xFF;
            r->spi_data_value = (uint16_t)data << 8;
            r->spi_data_pos = 1;
            return ret;
        } else {
            uint8_t ret = r->registers[r->spi_address] & 0xFF;
            r->spi_data_value |= data;
            set_reg(r, r->spi_address, r->spi_data_value);
            r->spi_state = CC2420_SPI_WAITING;
            return ret;
        }

    case CC2420_SPI_READ_REGISTER:
        if (r->spi_data_pos == 0) {
            r->spi_data_pos = 1;
            return (r->registers[r->spi_address] >> 8) & 0xFF;
        } else {
            r->spi_state = CC2420_SPI_WAITING;
            return r->registers[r->spi_address] & 0xFF;
        }

    case CC2420_SPI_READ_RXFIFO: {
        uint8_t fifo_data = rxfifo_read(r);
        rxfifo_spi_reads++;

        /* Clear FIFOP when buffer drops below threshold */
        if (r->current_fifop && !r->overflow && r->rx_fifo_len <= r->fifop_thr) {
            set_fifop(r, false);
        }

        /* Track current packet read progress */
        if (r->rx_fifo_read_left == 0) {
            r->rx_fifo_read_left = fifo_data;  /* first byte = length */
        } else if (--r->rx_fifo_read_left == 0) {
            /* Finished reading a packet — check for another */
            if (r->rx_fifo_len > 0) {
                /* Peek at next packet length */
                uint8_t next_len = r->memory[CC2420_RAM_RXFIFO + r->rx_fifo_read_pos];
                if (r->rx_fifo_len > next_len || r->rx_fifo_len > r->fifop_thr) {
                    set_fifop(r, true);
                }
            }
        }

        /* Clear FIFO when buffer empty */
        if (r->rx_fifo_len == 0)
            set_fifo(r, false);

        return fifo_data;
    }

    case CC2420_SPI_WRITE_TXFIFO:
        if (r->tx_fifo_flush) {
            r->tx_cursor = 0;
            r->tx_fifo_flush = false;
        }
        if (r->tx_cursor < 128) {
            r->memory[CC2420_RAM_TXFIFO + r->tx_cursor] = data;
            r->tx_cursor++;
        }
        return old_status;

    case CC2420_SPI_RAM_ACCESS:
        if (r->spi_data_pos == 0) {
            /* Second byte: upper address bits + read flag */
            r->spi_address |= (data << 1) & 0x180;
            r->spi_ram_read = (data & CC2420_FLAG_RAM_READ) != 0;
            r->spi_data_pos = 1;
            return old_status;
        } else {
            if (!r->spi_ram_read) {
                /* RAM write */
                if (r->spi_address < 384)  /* 0x180 */
                    r->memory[r->spi_address] = data;
                r->spi_address = (r->spi_address + 1) % 384;
                return old_status;
            } else {
                /* RAM read */
                uint8_t val = 0;
                if (r->spi_address < 384)
                    val = r->memory[r->spi_address];
                r->spi_address = (r->spi_address + 1) % 384;
                return val;
            }
        }
    }

    return old_status;
}

/* ================================================================
 * Reset and init
 * ================================================================ */

static void cc2420_reset(cc2420_t *r) {
    memset(r->registers, 0, sizeof(r->registers));
    /* CC2420 datasheet register defaults */
    set_reg(r, CC2420_REG_MDMCTRL0, 0x0AE2);
    r->registers[CC2420_REG_RSSI] = 0xE080;
    r->registers[CC2420_REG_TXCTRL] = 0xA0FF;
    /* Use set_reg so the channel-push side effect runs and the medium
     * sees the chip's default channel (26 = FSCTRL 0x4165) immediately. */
    set_reg(r, CC2420_REG_FSCTRL, 0x4165);
    r->registers[CC2420_REG_IOCFG0] = 0x0040;  /* FIFOP threshold */
    r->registers[CC2420_REG_MANFIDL] = 0x233D;  /* manufacturer ID */
    r->registers[CC2420_REG_MANFIDH] = 0x2000;

    r->status = 0;
    r->spi_state = CC2420_SPI_WAITING;
    r->spi_data_pos = 0;
    r->tx_cursor = 0;
    r->tx_fifo_flush = false;
    r->rx_fifo_write_pos = 0;
    r->rx_fifo_read_pos = 0;
    r->rx_fifo_len = 0;
    r->rx_fifo_read_left = 0;
    r->rxlen = 0;
    r->rxread = 0;
    r->zero_symbols = 0;
    r->crc_ok = false;
    r->frame_rejected = false;
    r->overflow = false;
    r->should_ack = false;
    r->fifop_thr = 64;  /* default FIFOP threshold */
    r->rx_rssi = -50;   /* default RSSI (backward compatible) */
}

void cc2420_init(cc2420_t *radio, const sim_host_t *host) {
    memset(radio, 0, sizeof(*radio));
    radio->host = host;
    radio->state = CC2420_VREG_OFF;

    /* Setup events */
    radio->vreg_event.callback = vreg_event_callback;
    radio->vreg_event.user_data = radio;
    radio->oscillator_event.callback = oscillator_event_callback;
    radio->oscillator_event.user_data = radio;
    radio->symbol_event.callback = symbol_event_callback;
    radio->symbol_event.user_data = radio;
    radio->sfd_clear_event.callback = sfd_clear_callback;
    radio->sfd_clear_event.user_data = radio;

    cc2420_reset(radio);
}

void cc2420_set_pins(cc2420_t *radio,
                      int fifop_port, int fifop_pin,
                      int fifo_port, int fifo_pin,
                      int cca_port, int cca_pin,
                      int sfd_port, int sfd_pin) {
    radio->fifop_port = fifop_port;
    radio->fifop_pin  = fifop_pin;
    radio->fifo_port  = fifo_port;
    radio->fifo_pin   = fifo_pin;
    radio->cca_port   = cca_port;
    radio->cca_pin    = cca_pin;
    radio->sfd_port   = sfd_port;
    radio->sfd_pin    = sfd_pin;
}

void cc2420_set_rf_listener(cc2420_t *radio, cc2420_rf_callback_fn cb, void *data) {
    radio->rf_tx_callback = cb;
    radio->rf_tx_data = data;
}
