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
#include <string.h>
#include <stdio.h>

/* Convenience accessors for the host vtable. */
#define HOST_NOW_NS(c)             ((c)->host->now_ns((c)->host->cpu))
#define HOST_SCHEDULE_NS(c, ev, t) ((c)->host->schedule_ns((c)->host->cpu, (ev), (t)))
#define HOST_CANCEL(c, ev)         ((c)->host->cancel((c)->host->cpu, (ev)))
#define HOST_SET_PIN(c, p, n, v)   ((c)->host->set_input_pin((c)->host->gpio, (p), (n), (v)))
#define HOST_FORCE_IRQ(c, p, n, e) ((c)->host->force_irq_edge((c)->host->gpio, (p), (n), (e)))

/* ------------------------------------------------------------------ */
/* Air-side bit-stream timing — 50 kbps 2-FSK as configured by Contiki  */
/* ------------------------------------------------------------------ */

#define CC1200_BYTE_PERIOD_NS    160000   /* 8 bits @ 50 kbps */
#define CC1200_RESET_TIME_NS     200000   /* SRES → IDLE, ~200 µs in real HW */

/* ------------------------------------------------------------------ */
/* Status byte / MARCSTATE helpers                                      */
/* ------------------------------------------------------------------ */

static uint8_t marcstate_to_status_top(uint8_t marc) {
    switch (marc) {
    case CC1200_MARC_SLEEP:
    case CC1200_MARC_IDLE:        return CC1200_STATUS_IDLE;
    case CC1200_MARC_RX:          return CC1200_STATUS_RX;
    case CC1200_MARC_TX:          return CC1200_STATUS_TX;
    case CC1200_MARC_RX_FIFO_ERR: return CC1200_STATUS_RX_FIFO_ERR;
    case CC1200_MARC_TX_FIFO_ERR: return CC1200_STATUS_TX_FIFO_ERR;
    default:                      return CC1200_STATUS_IDLE;
    }
}

static void refresh_status(cc1200_t *c) {
    /* Status byte top nibble = state, low 4 bits = bytes-in-FIFO clipped to 0xF.
     * The "bytes in FIFO" field is the RX FIFO when in RX, TX FIFO otherwise.
     * Real chip is a bit fancier, but Contiki only cares about the top nibble. */
    uint8_t state_bits = marcstate_to_status_top(c->marcstate);
    int bytes = (c->marcstate == CC1200_MARC_RX) ? c->rx_count : c->tx_count;
    if (bytes > 0xF) bytes = 0xF;
    c->status = state_bits | (uint8_t)bytes;
}

uint8_t cc1200_status(const cc1200_t *c) { return c->status; }
int     cc1200_marcstate(const cc1200_t *c) { return c->marcstate; }
int     cc1200_rxfifo_count(const cc1200_t *c) { return c->rx_count; }
int     cc1200_txfifo_count(const cc1200_t *c) { return c->tx_count; }

/* ------------------------------------------------------------------ */
/* GDO0 / GDO2 edge driving                                              */
/* ------------------------------------------------------------------ */

static void drive_gdo(cc1200_t *c, const cc1200_pin_t *pin, bool *cur_level, bool level) {
    if (!pin->enabled) return;
    if (*cur_level == level) return;
    *cur_level = level;
    HOST_SET_PIN(c, pin->port, pin->pin, level);
    HOST_FORCE_IRQ(c, pin->port, pin->pin, level);
}

static void drive_gdo0(cc1200_t *c, bool level) {
    drive_gdo(c, &c->gdo0, &c->gdo0_level, level);
}

static void drive_gdo2(cc1200_t *c, bool level) {
    drive_gdo(c, &c->gdo2, &c->gdo2_level, level);
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
}

void cc1200_receive_byte(cc1200_t *c, uint8_t byte) {
    /* Bytes only count when chip is in RX */
    if (c->marcstate != CC1200_MARC_RX) return;

    switch (c->air_state) {
    case CC1200_AIR_HUNT: {
        /* Slide the byte into the sync-word match register. */
        c->sync_match = (c->sync_match << 8) | byte;
        if (c->sync_match == sync_word_value(c)) {
            c->air_state = CC1200_AIR_PHR;
            c->air_phr_count = 0;
            /* SFD detected → assert PKT_SYNC_RXTX (GDO0 default IOCFG). */
            if (c->regs[CC1200_REG_IOCFG0] == CC1200_IOCFG_PKT_SYNC_RXTX) {
                drive_gdo0(c, true);
            }
            if (c->regs[CC1200_REG_IOCFG2] == CC1200_IOCFG_PKT_SYNC_RXTX) {
                drive_gdo2(c, true);
            }
        }
        break;
    }
    case CC1200_AIR_PHR: {
        /* Push every PHR byte into the RX FIFO so the firmware can read
         * the PHR with burst_read(CC1200_RXFIFO, &phr, 2). */
        if (!fifo_push_rx(c, byte)) {
            c->marcstate = CC1200_MARC_RX_FIFO_ERR;
            air_reset(c);
            drive_gdo0(c, false);
            return;
        }
        if (c->air_phr_count == 0) {
            c->air_payload_total = (byte & 0x07) << 8;
        } else {
            c->air_payload_total |= byte;
            /* PHR complete — payload bytes follow. */
            if (c->air_payload_total == 0 ||
                c->air_payload_total > CC1200_FIFO_SIZE - 2) {
                /* Bogus PHR — bail out. */
                c->marcstate = CC1200_MARC_RX_FIFO_ERR;
                air_reset(c);
                drive_gdo0(c, false);
                return;
            }
            c->air_payload_remaining = c->air_payload_total;
            c->air_state = CC1200_AIR_PAYLOAD;
        }
        c->air_phr_count++;
        break;
    }
    case CC1200_AIR_PAYLOAD: {
        if (!fifo_push_rx(c, byte)) {
            c->marcstate = CC1200_MARC_RX_FIFO_ERR;
            air_reset(c);
            drive_gdo0(c, false);
            return;
        }
        c->air_payload_remaining--;
        if (c->air_payload_remaining == 0) {
            /* Packet complete. The CRC was not actually computed but the
             * Contiki driver checks bit 7 of the LQI/CRC byte appended at
             * the end. APPEND_STATUS path: the last 2 bytes of the
             * payload (already in the RX FIFO) need to look like
             * RSSI, (CRC_OK | LQI). The sender appended a 2-byte CRC
             * already; convert those to (RSSI, 0x80) so the driver sees
             * "CRC OK". */
            if (c->rx_count >= 2) {
                /* Overwrite the last two bytes (the on-air CRC) */
                int last = (c->rx_last - 1 + CC1200_FIFO_SIZE) % CC1200_FIFO_SIZE;
                int prev = (c->rx_last - 2 + CC1200_FIFO_SIZE) % CC1200_FIFO_SIZE;
                c->rx_fifo[prev] = (uint8_t)c->rx_rssi;
                c->rx_fifo[last] = 0x80;  /* CRC OK, LQI=0 */
            }
            c->stat_rx_packets++;
            air_reset(c);
            /* Packet end → deassert GDO0 (PKT_SYNC_RXTX falling edge). */
            drive_gdo0(c, false);
            drive_gdo2(c, false);
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

static void start_tx(cc1200_t *c) {
    if (c->tx_count == 0) {
        /* Nothing to send — TX FIFO underflow */
        c->marcstate = CC1200_MARC_TX_FIFO_ERR;
        refresh_status(c);
        return;
    }

    /* Build the on-air buffer:
     *   preamble (4 × 0x55) + sync_word (4 bytes) + payload (TX FIFO) */
    int idx = 0;
    for (int i = 0; i < 4; i++) c->tx_air_buf[idx++] = 0x55;
    uint32_t sw = sync_word_value(c);
    c->tx_air_buf[idx++] = (uint8_t)(sw >> 24);
    c->tx_air_buf[idx++] = (uint8_t)(sw >> 16);
    c->tx_air_buf[idx++] = (uint8_t)(sw >> 8);
    c->tx_air_buf[idx++] = (uint8_t)(sw);
    /* Drain TX FIFO into air buffer */
    while (c->tx_count > 0 && idx < (int)sizeof(c->tx_air_buf)) {
        c->tx_air_buf[idx++] = fifo_pop_tx(c);
    }

    c->tx_emit_index = 0;
    c->tx_emit_total = idx;
    c->tx_active = true;
    c->marcstate = CC1200_MARC_TX;
    refresh_status(c);

    /* TX-started → assert PKT_SYNC_RXTX */
    if (c->regs[CC1200_REG_IOCFG0] == CC1200_IOCFG_PKT_SYNC_RXTX) {
        drive_gdo0(c, true);
    }
    if (c->regs[CC1200_REG_IOCFG2] == CC1200_IOCFG_PKT_SYNC_RXTX) {
        drive_gdo2(c, true);
    }

    /* Schedule the first byte at +1 byte period */
    int64_t fire = HOST_NOW_NS(c) + CC1200_BYTE_PERIOD_NS;
    HOST_SCHEDULE_NS(c, &c->tx_byte_event, fire);
}

static void tx_byte_event_cb(void *user_data, cpu_event_t *ev) {
    (void)ev;
    cc1200_t *c = (cc1200_t *)user_data;
    if (!c->tx_active) return;

    if (c->tx_emit_index < c->tx_emit_total) {
        uint8_t b = c->tx_air_buf[c->tx_emit_index++];
        if (c->rf_tx_callback) c->rf_tx_callback(c->rf_tx_user_data, b);
        int64_t fire = HOST_NOW_NS(c) + CC1200_BYTE_PERIOD_NS;
        HOST_SCHEDULE_NS(c, &c->tx_byte_event, fire);
        return;
    }

    /* TX complete — return to RX (TXOFF_MODE = RX is Contiki's default) */
    c->tx_active = false;
    c->stat_tx_packets++;
    /* Deassert GDO0 = packet-end */
    drive_gdo0(c, false);
    drive_gdo2(c, false);
    c->marcstate = CC1200_MARC_RX;
    air_reset(c);
    refresh_status(c);
}

/* ------------------------------------------------------------------ */
/* Strobes                                                              */
/* ------------------------------------------------------------------ */

static void reset_done_event_cb(void *user_data, cpu_event_t *ev) {
    (void)ev;
    cc1200_t *c = (cc1200_t *)user_data;
    c->marcstate = CC1200_MARC_IDLE;
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
    c->gdo0_level = false;
    c->gdo2_level = false;
    c->marcstate = CC1200_MARC_SLEEP;
    refresh_status(c);

    HOST_CANCEL(c, &c->tx_byte_event);
    HOST_CANCEL(c, &c->reset_done_event);
    /* SRES → IDLE after a small delay */
    int64_t fire = HOST_NOW_NS(c) + CC1200_RESET_TIME_NS;
    HOST_SCHEDULE_NS(c, &c->reset_done_event, fire);
}

static void handle_strobe(cc1200_t *c, uint8_t strobe) {
    c->stat_strobe_count++;
    switch (strobe) {
    case CC1200_STROBE_SRES:
        chip_reset(c);
        break;
    case CC1200_STROBE_SNOP:
        /* No-op — status byte updated below */
        break;
    case CC1200_STROBE_SIDLE:
        c->tx_active = false;
        HOST_CANCEL(c, &c->tx_byte_event);
        c->marcstate = CC1200_MARC_IDLE;
        air_reset(c);
        drive_gdo0(c, false);
        drive_gdo2(c, false);
        break;
    case CC1200_STROBE_SCAL:
        /* Calibration completes synchronously in IDLE */
        if (c->marcstate == CC1200_MARC_RX || c->marcstate == CC1200_MARC_TX) {
            /* spec: SCAL only valid from IDLE; ignore otherwise */
            break;
        }
        c->marcstate = CC1200_MARC_IDLE;
        break;
    case CC1200_STROBE_SRX:
        c->marcstate = CC1200_MARC_RX;
        air_reset(c);
        break;
    case CC1200_STROBE_STX:
        if (c->marcstate == CC1200_MARC_RX || c->marcstate == CC1200_MARC_IDLE) {
            start_tx(c);
        }
        break;
    case CC1200_STROBE_SFRX:
        if (c->marcstate == CC1200_MARC_IDLE ||
            c->marcstate == CC1200_MARC_RX_FIFO_ERR) {
            fifo_reset_rx(c);
            if (c->marcstate == CC1200_MARC_RX_FIFO_ERR)
                c->marcstate = CC1200_MARC_IDLE;
        }
        break;
    case CC1200_STROBE_SFTX:
        if (c->marcstate == CC1200_MARC_IDLE ||
            c->marcstate == CC1200_MARC_TX_FIFO_ERR) {
            fifo_reset_tx(c);
            if (c->marcstate == CC1200_MARC_TX_FIFO_ERR)
                c->marcstate = CC1200_MARC_IDLE;
        }
        break;
    case CC1200_STROBE_SPWD:
        c->marcstate = CC1200_MARC_SLEEP;
        break;
    case CC1200_STROBE_SXOFF:
        /* Crystal off → still report IDLE in our model */
        c->marcstate = CC1200_MARC_IDLE;
        break;
    case CC1200_STROBE_SFSTXON:
        /* Settled in fast TX-on state — report TX. */
        c->marcstate = CC1200_MARC_TX;
        break;
    default:
        break;
    }
    refresh_status(c);
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
    default:                       return c->regs[addr];
    }
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
        c->tx_active = false;
        HOST_CANCEL(c, &c->tx_byte_event);
        HOST_CANCEL(c, &c->reset_done_event);
        air_reset(c);
        drive_gdo0(c, false);
        drive_gdo2(c, false);
        refresh_status(c);
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
