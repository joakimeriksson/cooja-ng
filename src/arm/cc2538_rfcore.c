/*
 * CC2538 RF Core — 802.15.4 radio
 *
 * M1: Frame parsing, interrupt generation, frame-based TX/RX.
 */
#include "cc2538_rfcore.h"
#include "arm_nvic.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* Register region sizes */
#define RFCORE_FFSM_SIZE  0x600
#define RFCORE_XREG_SIZE  0x200
#define RFCORE_SFR_SIZE   0x100

/* 802.15.4 PHY constants */
#define IEEE802154_SFD  0x7A
#define PREAMBLE_MIN    4

/* Debug counters */
static int rxfifo_overflow_count = 0;
int cc2538_rfcore_get_rxfifo_overflows(void) { return rxfifo_overflow_count; }
void cc2538_rfcore_reset_rxfifo_overflows(void) { rxfifo_overflow_count = 0; }

/* ================================================================
 * CRC — CCITT-16 with bit reversal (matches CC2420 wire format)
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

/* MAC timer (T2) runs at 32 MHz system clock (not 32.768 kHz like the sleep timer).
 * The firmware uses RADIO_TO_RTIMER(delta) = delta * 32768 / 32000000
 * to convert MAC timer ticks to rtimer (sleep timer) ticks. */
#define MT_FREQ_HZ  32000000
/* Convert ns to MAC timer ticks */
static inline int64_t ns_to_mt_ticks(int64_t ns) {
    return ns * MT_FREQ_HZ / 1000000000LL;
}

/* --- Interrupt helper --- */

static void rfcore_check_interrupts(cc2538_rfcore_t *rf) {
    uint32_t masked0 = rf->rfirqf0 & rf->rfirqm0;
    uint32_t masked1 = rf->rfirqf1 & rf->rfirqm1;
    if (masked0 || masked1)
        arm_nvic_set_pending(rf->nvic, RFCORE_RX_TX_IRQ);
    if (rf->rferrf & rf->rferrm)
        arm_nvic_set_pending(rf->nvic, RFCORE_ERR_IRQ);
}

/* --- State management --- */

static void rfcore_set_state(cc2538_rfcore_t *rf, rf_state_t state) {
    int old = rf->state;
    rf->state = state;
    if (rf->state_callback && old != (int)state)
        rf->state_callback(rf->state_user_data, old, (int)state);
    switch (state) {
        case RF_STATE_IDLE:
            rf->fsmstat1 = 0;
            break;
        case RF_STATE_RX_CALIBR:
        case RF_STATE_SFD_WAIT:
        case RF_STATE_RX:
            rf->fsmstat1 = (1 << 0)   /* RX_ACTIVE */
                         | (1 << 4);  /* CCA — channel always clear */
            break;
        case RF_STATE_TX_CALIBR:
        case RF_STATE_TX:
        case RF_STATE_TX_FINAL:
            rf->fsmstat1 = (1 << 1); /* TX_ACTIVE */
            break;
        default:
            rf->fsmstat1 = 0;
            break;
    }
    rf->fsmstat0 = (uint32_t)state;
}

/* --- TX done event callback --- */

static void rfcore_tx_done(void *user_data, arm_event_t *ev) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    (void)ev;
    rf->rfirqf1 |= RFIRQF1_TXDONE;
    rfcore_check_interrupts(rf);
    /* CC2538 hardware: after TX, return to RX if RXENABLE is set */
    if (rf->rxenable) {
        rfcore_set_state(rf, RF_STATE_RX_CALIBR);
        rfcore_set_state(rf, RF_STATE_SFD_WAIT);
        rf->rx_frame_state = RF_RX_PREAMBLE;
        rf->zero_symbols = 0;
        /* Discard bytes that arrived during TX — on real hardware these
         * are collisions (can't TX and RX simultaneously).  The auto-ACK
         * is already injected into RXFIFO by the ISTXON handler. */
        rf->rx_incoming_len = 0;
    } else {
        rfcore_set_state(rf, RF_STATE_IDLE);
    }
}

/* --- Strobes --- */

static void rfcore_strobe(cc2538_rfcore_t *rf, uint32_t strobe) {
    switch (strobe) {
        case CSP_ISRXON:
            rf->rxenable = 1;
            rf->software_off = false;
            rfcore_set_state(rf, RF_STATE_RX_CALIBR);
            /* Instant calibration for now (M3 will add timed transition) */
            rfcore_set_state(rf, RF_STATE_SFD_WAIT);
            /* Reset frame parsing state */
            rf->rx_frame_state = RF_RX_PREAMBLE;
            rf->zero_symbols = 0;
            /* Discard bytes buffered while radio was off/idle — on real
             * hardware the radio wasn't listening. */
            rf->rx_incoming_len = 0;
            break;

        case CSP_ISTXON: {
            rf->software_off = false;
            rfcore_set_state(rf, RF_STATE_TX_CALIBR);
            rfcore_set_state(rf, RF_STATE_TX);
            /* Emit 802.15.4 preamble + SFD, then TXFIFO bytes + auto-CRC */
            if (rf->tx_callback) {
                /* 4-byte preamble */
                for (int i = 0; i < PREAMBLE_MIN; i++)
                    rf->tx_callback(rf->tx_user_data, 0x00);
                /* SFD — capture MAC timer for TX timestamp */
                {
                    int64_t ns = arm_cycles_to_ns(rf->cpu->cycles, rf->cpu->cpu_freq_hz);
                    int64_t mt_ticks = ns_to_mt_ticks(ns);
                    rf->mt_sfd_capture = (uint16_t)(mt_ticks & 0xFFFF);
                    rf->mt_sfd_ovf_capture = (uint32_t)(mt_ticks >> 16) & 0xFFFFFF;
                }
                rf->tx_callback(rf->tx_user_data, IEEE802154_SFD);
                rf->rfirqf0 |= RFIRQF0_SFD;
                rfcore_check_interrupts(rf);
                /* TXFIFO data (length byte + payload) */
                for (int i = 0; i < rf->txfifo_len; i++)
                    rf->tx_callback(rf->tx_user_data, rf->txfifo[i]);
                /* Auto-append 2-byte FCS — compute CCITT CRC with bit
                 * reversal to match CC2420 wire format for mixed sims */
                if (rf->frmctrl0 & (1 << 6)) {
                    uint16_t crc = 0;
                    for (int i = 1; i < rf->txfifo_len; i++)
                        crc = crc_add_bitrev(crc, rf->txfifo[i]);
                    rf->tx_callback(rf->tx_user_data, bitrev((crc >> 8) & 0xFF));
                    rf->tx_callback(rf->tx_user_data, bitrev(crc & 0xFF));
                }
            }
            /* Schedule TX-done event: leave TX_ACTIVE set so firmware can
             * poll FSMSTAT1.TX_ACTIVE. The event fires after the over-the-air
             * transmission time and transitions to IDLE + TXDONE interrupt.
             * OTA time: (preamble+SFD+data+CRC) * 32µs/byte at 250kbps */
            {
                int ota_bytes = PREAMBLE_MIN + 1 + rf->txfifo_len + 2;
                int64_t tx_ns = (int64_t)ota_bytes * 32000; /* 32µs per byte */
                rf->txfifo_len = 0;
                arm_schedule_event_ns(rf->cpu, &rf->tx_event,
                                      rf->cpu->sim_time_ns + tx_ns);
            }
            /* Inject auto-ACK bytes directly into RXFIFO.
             *
             * On real CC2538, the ACK arrives ~200µs after TX completes.
             * In our simulation, all bytes are emitted/delivered in the
             * ISTXON handler.  The receiver's auto-ACK bytes land in
             * rx_incoming (sender is in TX state).  If we wait for tx_done
             * (3+ ms) to replay them, the firmware's CSMA ACK busy-wait
             * (RTIMER-based, but RTIMER is frozen during arm_step_until)
             * may loop forever or fail to find the ACK.
             *
             * Fix: parse rx_incoming to extract the ACK frame and inject
             * it directly into the RXFIFO.  We skip preamble+SFD and just
             * write [length][payload...] + RSSI/CRC_OK metadata.  No
             * interrupts are raised here — RXPKTDONE will be set later
             * when tx_done fires and transitions to RX. */
            if (rf->rx_incoming_len > 0 && rf->rxenable) {
                /* Find SFD in rx_incoming to locate the frame */
                int sfd_pos = -1;
                for (int i = 0; i < rf->rx_incoming_len; i++) {
                    if (rf->rx_incoming[i] == IEEE802154_SFD) {
                        sfd_pos = i;
                        break;
                    }
                }
                if (sfd_pos >= 0 && sfd_pos + 1 < rf->rx_incoming_len) {
                    int frame_start = sfd_pos + 1; /* length byte */
                    int frame_len = rf->rx_incoming[frame_start]; /* payload length */
                    int total = 1 + frame_len; /* length byte + payload */
                    /* Compact RXFIFO first to reclaim read space */
                    if (rf->rxfifo_rd > 0) {
                        int remaining = rf->rxfifo_len - rf->rxfifo_rd;
                        if (remaining > 0)
                            memmove(rf->rxfifo, rf->rxfifo + rf->rxfifo_rd,
                                    (size_t)remaining);
                        rf->rxfifo_len = remaining;
                        rf->rxfifo_rd = 0;
                    }
                    if (frame_start + total <= rf->rx_incoming_len &&
                        rf->rxfifo_len + total <= RF_RXFIFO_SIZE) {
                        /* Copy frame into RXFIFO */
                        memcpy(rf->rxfifo + rf->rxfifo_len,
                               rf->rx_incoming + frame_start, (size_t)total);
                        rf->rxfifo_len += total;
                        /* Replace last 2 bytes with RSSI + CRC_OK|LQI */
                        if (rf->rxfifo_len >= 2) {
                            rf->rxfifo[rf->rxfifo_len - 2] = (uint8_t)(int8_t)(-50);
                            rf->rxfifo[rf->rxfifo_len - 1] = 0x80 | 50;
                        }
                    }
                }
                rf->rx_incoming_len = 0;
            }
            break;
        }

        case CSP_ISTXONCCA:
            /* Simplified: always transmit (CCA is M2 work) */
            rfcore_strobe(rf, CSP_ISTXON);
            break;

        case CSP_ISRFOFF:
            rf->rxenable = 0;
            rf->software_off = true;
            /* Report IDLE to the state callback (for timeline tracking)
             * but keep the RX frame parser active so the simulation
             * medium can still deliver frames.
             * FSMSTAT0 reads will report IDLE when software_off is set. */
            rfcore_set_state(rf, RF_STATE_IDLE);
            break;

        case CSP_ISFLUSHRX:
            rf->rxfifo_len = 0;
            rf->rxfifo_rd = 0;
            rf->rx_overflow = false;
            /* Do NOT clear rx_incoming here: those are bytes from future
             * transmissions buffered because the radio was busy.  They
             * will be delivered when ISRXON is next called. */
            rf->rx_frame_state = RF_RX_PREAMBLE;
            rf->zero_symbols = 0;
            break;

        case CSP_ISFLUSHTX:
            rf->txfifo_len = 0;
            break;
    }
}

/* --- CC2538 Info Page (IEEE address at 0x00280028) --- */

#define CC2538_INFO_PAGE_BASE  0x00280000
#define CC2538_INFO_PAGE_SIZE  0x1000
#define CC2538_IEEE_ADDR_OFF   0x028   /* Primary IEEE address offset */

static int info_page_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - CC2538_INFO_PAGE_BASE;
    /* IEEE 64-bit address at offset 0x28-0x2F */
    if (offset >= CC2538_IEEE_ADDR_OFF && offset < CC2538_IEEE_ADDR_OFF + 8)
        return rf->ext_addr[offset - CC2538_IEEE_ADDR_OFF];
    return 0;
}

static void info_page_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data; (void)addr; (void)value;
    /* Info page is read-only */
}

/* --- FFSM registers (address matching, EXT_ADDR, SHORT_ADDR, PAN_ID) --- */

/* FFSM register offsets */
/* CC2538 FFSM address register offsets (from RFCORE_FFSM_BASE 0x40088000) */
#define FFSM_EXT_ADDR0    0x5A8
#define FFSM_EXT_ADDR1    0x5AC
#define FFSM_EXT_ADDR2    0x5B0
#define FFSM_EXT_ADDR3    0x5B4
#define FFSM_EXT_ADDR4    0x5B8
#define FFSM_EXT_ADDR5    0x5BC
#define FFSM_EXT_ADDR6    0x5C0
#define FFSM_EXT_ADDR7    0x5C4
#define FFSM_PAN_ID0      0x5C8
#define FFSM_PAN_ID1      0x5CC
#define FFSM_SHORT_ADDR0  0x5D0
#define FFSM_SHORT_ADDR1  0x5D4

static int ffsm_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_FFSM_BASE;

    switch (offset) {
        case FFSM_PAN_ID0:     return rf->pan_id & 0xFF;
        case FFSM_PAN_ID1:     return (rf->pan_id >> 8) & 0xFF;
        case FFSM_SHORT_ADDR0: return rf->short_addr & 0xFF;
        case FFSM_SHORT_ADDR1: return (rf->short_addr >> 8) & 0xFF;
        case FFSM_EXT_ADDR0:   return rf->ext_addr[0];
        case FFSM_EXT_ADDR1:   return rf->ext_addr[1];
        case FFSM_EXT_ADDR2:   return rf->ext_addr[2];
        case FFSM_EXT_ADDR3:   return rf->ext_addr[3];
        case FFSM_EXT_ADDR4:   return rf->ext_addr[4];
        case FFSM_EXT_ADDR5:   return rf->ext_addr[5];
        case FFSM_EXT_ADDR6:   return rf->ext_addr[6];
        case FFSM_EXT_ADDR7:   return rf->ext_addr[7];
        default: return 0;
    }
}

static void ffsm_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_FFSM_BASE;

    switch (offset) {
        case FFSM_PAN_ID0:     rf->pan_id = (rf->pan_id & 0xFF00) | (value & 0xFF); break;
        case FFSM_PAN_ID1:     rf->pan_id = (rf->pan_id & 0x00FF) | ((value & 0xFF) << 8); break;
        case FFSM_SHORT_ADDR0: rf->short_addr = (rf->short_addr & 0xFF00) | (value & 0xFF); break;
        case FFSM_SHORT_ADDR1: rf->short_addr = (rf->short_addr & 0x00FF) | ((value & 0xFF) << 8); break;
        case FFSM_EXT_ADDR0:   rf->ext_addr[0] = (uint8_t)value; break;
        case FFSM_EXT_ADDR1:   rf->ext_addr[1] = (uint8_t)value; break;
        case FFSM_EXT_ADDR2:   rf->ext_addr[2] = (uint8_t)value; break;
        case FFSM_EXT_ADDR3:   rf->ext_addr[3] = (uint8_t)value; break;
        case FFSM_EXT_ADDR4:   rf->ext_addr[4] = (uint8_t)value; break;
        case FFSM_EXT_ADDR5:   rf->ext_addr[5] = (uint8_t)value; break;
        case FFSM_EXT_ADDR6:   rf->ext_addr[6] = (uint8_t)value; break;
        case FFSM_EXT_ADDR7:   rf->ext_addr[7] = (uint8_t)value; break;
    }
}

/* --- XREG registers --- */

static int xreg_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_XREG_BASE;

    switch (offset) {
        case RFCORE_XREG_FRMFILT0:  return (int)rf->frmfilt0;
        case RFCORE_XREG_FRMFILT1:  return (int)rf->frmfilt1;
        case RFCORE_XREG_SRCMATCH:  return (int)rf->srcmatch;
        case RFCORE_XREG_FRMCTRL0:  return (int)rf->frmctrl0;
        case RFCORE_XREG_FRMCTRL1:  return (int)rf->frmctrl1;
        case RFCORE_XREG_RXENABLE:  return (int)rf->rxenable;
        case RFCORE_XREG_FREQCTRL:  return (int)rf->freqctrl;
        case RFCORE_XREG_TXPOWER:   return (int)rf->txpower;
        case RFCORE_XREG_FSMSTAT0:
            return rf->software_off ? 0 : (int)rf->fsmstat0;
        case RFCORE_XREG_FSMSTAT1: {
            uint32_t val = rf->software_off ? 0 : rf->fsmstat1;
            /* FIFO (bit 7): RX FIFO has data — but goes LOW on overflow.
             * Real CC2538 uses FIFOP=1 && FIFO=0 as the overflow indicator;
             * firmware detects this and does ISFLUSHRX to recover. */
            if (rf->rxfifo_rd < rf->rxfifo_len && !rf->rx_overflow)
                val |= (1 << 7);
            /* FIFOP (bit 6): live status — complete frame available in FIFO.
             * Unlike the RFIRQF0.FIFOP interrupt flag (which gets cleared by
             * the ISR), this hardware signal stays high as long as there are
             * unread bytes in the RXFIFO.  Stays high during overflow. */
            if (rf->rxfifo_rd < rf->rxfifo_len)
                val |= (1 << 6);
            /* CCA (bit 4): always set — channel clear for simulation */
            val |= (1 << 4);
            return (int)val;
        }
        case RFCORE_XREG_FIFOPCTRL: return (int)rf->fifopctrl;
        case RFCORE_XREG_FSMCTRL:   return (int)rf->fsmctrl;
        case RFCORE_XREG_CCACTRL0:  return (int)rf->ccactrl0;
        case RFCORE_XREG_RSSI:      return (int)rf->rssi;
        case RFCORE_XREG_RFIRQM0:   return (int)rf->rfirqm0;
        case RFCORE_XREG_RFIRQM1:   return (int)rf->rfirqm1;
        case RFCORE_XREG_RFERRM:    return (int)rf->rferrm;
        case RFCORE_XREG_RFRND: {
            rf->rfrnd_state = rf->rfrnd_state * 1103515245 + 12345;
            return (int)((rf->rfrnd_state >> 16) & 3);
        }
        case RFCORE_XREG_RSSISTAT:  return 1; /* RSSI valid */
        case RFCORE_XREG_AGCCTRL1:  return (int)rf->agcctrl1;
        case RFCORE_XREG_TXFILTCFG: return (int)rf->txfiltcfg;
        default: return 0;
    }
}

static void xreg_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_XREG_BASE;

    switch (offset) {
        case RFCORE_XREG_FRMFILT0:  rf->frmfilt0 = value; break;
        case RFCORE_XREG_FRMFILT1:  rf->frmfilt1 = value; break;
        case RFCORE_XREG_SRCMATCH:  rf->srcmatch = value; break;
        case RFCORE_XREG_FRMCTRL0:  rf->frmctrl0 = value; break;
        case RFCORE_XREG_FRMCTRL1:  rf->frmctrl1 = value; break;
        case RFCORE_XREG_RXENABLE:  rf->rxenable = value; break;
        case RFCORE_XREG_FREQCTRL:
            rf->freqctrl = value;
            rf->channel = ((value & 0x7F) - 11) / 5 + 11;
            break;
        case RFCORE_XREG_TXPOWER:   rf->txpower = value; break;
        case RFCORE_XREG_FIFOPCTRL: rf->fifopctrl = value; break;
        case RFCORE_XREG_FSMCTRL:   rf->fsmctrl = value; break;
        case RFCORE_XREG_CCACTRL0:  rf->ccactrl0 = value; break;
        case RFCORE_XREG_RFIRQM0:   rf->rfirqm0 = value; rfcore_check_interrupts(rf); break;
        case RFCORE_XREG_RFIRQM1:   rf->rfirqm1 = value; rfcore_check_interrupts(rf); break;
        case RFCORE_XREG_RFERRM:    rf->rferrm = value; rfcore_check_interrupts(rf); break;
        case RFCORE_XREG_AGCCTRL1:  rf->agcctrl1 = value; break;
        case RFCORE_XREG_TXFILTCFG: rf->txfiltcfg = value; break;
    }
}

/* --- SFR registers --- */

static int sfr_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_SFR_BASE;

    switch (offset) {
        case RFCORE_SFR_MTCSPCFG: return (int)rf->mtcspcfg;
        case RFCORE_SFR_MTCTRL:   return (int)rf->mtctrl;
        case RFCORE_SFR_MTIRQM:   return (int)rf->mtirqm;
        case RFCORE_SFR_MTIRQF:   return (int)rf->mtirqf;
        case RFCORE_SFR_MTMSEL:   return (int)rf->mtmsel;
        case RFCORE_SFR_MTM0:
        case RFCORE_SFR_MTM1: {
            /* MAC timer MTM0/MTM1: multiplexed via MTMSEL[2:0]
             * 0=Timer, 1=Capture(SFD), 2=Period, 3-7=Compare */
            int sel = rf->mtmsel & 0x07;
            uint16_t val;
            if (sel == 0) {
                /* Free-running timer counter */
                int64_t ns = arm_cycles_to_ns(rf->cpu->cycles, rf->cpu->cpu_freq_hz);
                int64_t mt_ticks = ns_to_mt_ticks(ns);
                val = (uint16_t)(mt_ticks & 0xFFFF);
                /* LATCH_MODE: reading MTM0 latches the overflow counter */
                if (offset == RFCORE_SFR_MTM0 && (rf->mtctrl & 0x08)) {
                    rf->mt_ovf_latched = (uint32_t)(mt_ticks >> 16) & 0xFFFFFF;
                    rf->mt_ovf_latch_valid = true;
                }
            } else if (sel == 1) {
                /* SFD capture */
                val = rf->mt_sfd_capture;
            } else if (sel == 2) {
                /* Period */
                val = rf->mt_cmp_period;
            } else {
                val = 0;
            }
            if (offset == RFCORE_SFR_MTM0) return val & 0xFF;
            return (val >> 8) & 0xFF;
        }
        case RFCORE_SFR_MTMOVF0:
        case RFCORE_SFR_MTMOVF1:
        case RFCORE_SFR_MTMOVF2: {
            /* MAC timer overflow: multiplexed via MTMSEL[6:4]
             * 0=Overflow counter, 1=Overflow capture(SFD), 2=Overflow period */
            int sel = (rf->mtmsel >> 4) & 0x07;
            uint32_t val;
            if (sel == 0) {
                /* If latched by MTM0 read in LATCH_MODE, use latched value.
                 * Don't clear the latch here — it persists until the next
                 * MTM0 read triggers a new latch. Firmware reads MTMOVF0,
                 * MTMOVF1, MTMOVF2 in sequence from the same latch. */
                if (rf->mt_ovf_latch_valid) {
                    val = rf->mt_ovf_latched;
                } else {
                    int64_t ns = arm_cycles_to_ns(rf->cpu->cycles, rf->cpu->cpu_freq_hz);
                    int64_t mt_ticks = ns_to_mt_ticks(ns);
                    val = (uint32_t)(mt_ticks >> 16) & 0xFFFFFF;
                }
            } else if (sel == 1) {
                /* SFD overflow capture */
                val = rf->mt_sfd_ovf_capture;
            } else if (sel == 2) {
                /* Overflow period */
                val = rf->mt_cmp_ovf_period;
            } else {
                val = 0;
            }
            if (offset == RFCORE_SFR_MTMOVF0) return val & 0xFF;
            if (offset == RFCORE_SFR_MTMOVF1) return (val >> 8) & 0xFF;
            return (val >> 16) & 0xFF; /* MTMOVF2 */
        }
        case RFCORE_SFR_RFDATA: {
            if (rf->rxfifo_rd < rf->rxfifo_len) {
                uint8_t b = rf->rxfifo[rf->rxfifo_rd++];
                /* Auto-compact: when all bytes consumed, reclaim space.
                 * Real CC2538 RXFIFO is a circular buffer; consumed
                 * bytes are automatically freed.  The driver's read()
                 * does NOT call ISFLUSHRX after a successful read. */
                if (rf->rxfifo_rd >= rf->rxfifo_len) {
                    rf->rxfifo_rd = 0;
                    rf->rxfifo_len = 0;
                }
                return b;
            }
            return 0;
        }
        case RFCORE_SFR_RFERRF:   return (int)rf->rferrf;
        case RFCORE_SFR_RFIRQF0:
            return (int)rf->rfirqf0;
        case RFCORE_SFR_RFIRQF1:  return (int)rf->rfirqf1;
        case RFCORE_SFR_RFST:     return 0;
        default: return 0;
    }
}

static void sfr_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_SFR_BASE;

    switch (offset) {
        case RFCORE_SFR_MTCSPCFG: rf->mtcspcfg = value; break;
        case RFCORE_SFR_MTCTRL:
            rf->mtctrl = value;
            /* STATE (bit 2) mirrors RUN (bit 0) — firmware busy-waits for this */
            if (value & 0x01) /* RUN */
                rf->mtctrl |= 0x04; /* STATE = running */
            else
                rf->mtctrl &= ~0x04; /* STATE = stopped */

            break;
        case RFCORE_SFR_MTIRQM:   rf->mtirqm = value; break;
        case RFCORE_SFR_MTIRQF:   rf->mtirqf &= value; break; /* W0C */
        case RFCORE_SFR_MTMSEL:   rf->mtmsel = value; break;
        case RFCORE_SFR_MTM0:     rf->mtm0 = value; break;
        case RFCORE_SFR_MTM1:     rf->mtm1 = value; break;
        case RFCORE_SFR_MTMOVF0:  rf->mtmovf0 = value; break;
        case RFCORE_SFR_MTMOVF1:  rf->mtmovf1 = value; break;
        case RFCORE_SFR_MTMOVF2:  rf->mtmovf2 = value; break;
        case RFCORE_SFR_RFDATA:
            if (rf->txfifo_len < RF_TXFIFO_SIZE)
                rf->txfifo[rf->txfifo_len++] = (uint8_t)(value & 0xFF);
            break;
        case RFCORE_SFR_RFERRF:
            rf->rferrf &= value;    /* W0C: write-0-clears */
            rfcore_check_interrupts(rf);
            break;
        case RFCORE_SFR_RFIRQF0:
            rf->rfirqf0 &= value;   /* W0C: write-0-clears */
            rfcore_check_interrupts(rf);
            break;
        case RFCORE_SFR_RFIRQF1:
            rf->rfirqf1 &= value;   /* W0C: write-0-clears */
            rfcore_check_interrupts(rf);
            break;
        case RFCORE_SFR_RFST:
            rfcore_strobe(rf, value);
            break;
    }
}

/* --- Init --- */

void cc2538_rfcore_init(cc2538_rfcore_t *rf, arm_cpu_t *cpu, arm_nvic_t *nvic) {
    memset(rf, 0, sizeof(*rf));
    rf->cpu = cpu;
    rf->nvic = nvic;
    rf->state = RF_STATE_IDLE;
    rf->fifopctrl = 64;       /* Default FIFOP threshold */
    rf->frmfilt0 = 0x0D;      /* Frame filtering enabled by default */
    rf->frmctrl0 = 0x40;      /* AUTOACK enabled */
    rf->rssi = -128 & 0xFF;   /* No signal */
    rf->ccactrl0 = 0xF8;      /* Default CCA threshold */
    rf->agcctrl1 = 0x11;
    rf->sfd_delivery_ns = -1; /* Use cpu->cycles for SFD capture by default */
    rf->txfiltcfg = 0x09;
    rf->txpower = 0xFF;        /* Max power */
    rf->rfrnd_state = 0xDEADBEEF;
    rf->rx_rssi = -50;  /* default RSSI (backward compatible) */

    rf->tx_event.callback = rfcore_tx_done;
    rf->tx_event.user_data = rf;

    arm_register_io(cpu, RFCORE_FFSM_BASE, RFCORE_FFSM_SIZE,
                    ffsm_read, ffsm_write, rf);
    arm_register_io(cpu, RFCORE_XREG_BASE, RFCORE_XREG_SIZE,
                    xreg_read, xreg_write, rf);
    arm_register_io(cpu, RFCORE_SFR_BASE, RFCORE_SFR_SIZE,
                    sfr_read, sfr_write, rf);
    arm_register_io(cpu, CC2538_INFO_PAGE_BASE, CC2538_INFO_PAGE_SIZE,
                    info_page_read, info_page_write, rf);
}

/* --- TX callback --- */

void cc2538_rfcore_set_tx_callback(cc2538_rfcore_t *rf,
                                   cc2538_rf_tx_fn cb, void *user_data) {
    rf->tx_callback = cb;
    rf->tx_user_data = user_data;
}

/* --- 802.15.4 address filter for auto-ACK --- */

/* Check if the received frame's destination address matches this node.
 * Returns true for broadcast or matching short/ext address.
 * Frame layout in RXFIFO: [len][FCF0][FCF1][DSN][DestPAN_lo][DestPAN_hi][DestAddr...]
 * The 'base' index points to FCF0 (first byte after the length byte). */
static bool rfcore_dest_addr_matches(cc2538_rfcore_t *rf) {
    int base = rf->rxfifo_len - rf->rxlen; /* index of FCF0 */
    if (base < 0 || rf->rxlen < 3) return true; /* too short, accept */

    uint8_t fcf1 = rf->rxfifo[base + 1];
    int dest_addr_mode = (fcf1 >> 2) & 0x03;

    if (dest_addr_mode == 0) {
        /* No destination address (beacons, ACKs) — accept */
        return true;
    }

    /* Need DSN(1) + DestPAN(2) = 3 bytes after FCF, so at least 5 payload bytes */
    if (rf->rxlen < 5) return true;

    uint16_t dest_pan = rf->rxfifo[base + 3] | ((uint16_t)rf->rxfifo[base + 4] << 8);
    /* PAN ID must match or be broadcast */
    if (dest_pan != 0xFFFF && dest_pan != rf->pan_id) {
        return false;
    }

    if (dest_addr_mode == 2) {
        /* Short address (16-bit) */
        if (rf->rxlen < 7) return true;
        uint16_t dest_addr = rf->rxfifo[base + 5] | ((uint16_t)rf->rxfifo[base + 6] << 8);
        return (dest_addr == 0xFFFF || dest_addr == rf->short_addr);
    } else if (dest_addr_mode == 3) {
        /* Extended address (64-bit) */
        if (rf->rxlen < 13) return true;
        for (int i = 0; i < 8; i++) {
            if (rf->rxfifo[base + 5 + i] != rf->ext_addr[i]) return false;
        }
        return true;
    }

    return true; /* Unknown mode, accept */
}

/* --- RX byte processing with 802.15.4 frame parsing --- */

void cc2538_rfcore_receive_byte(cc2538_rfcore_t *rf, uint8_t byte) {
    /* Only buffer during actual TX — cannot process while transmitting */
    if (rf->state == RF_STATE_TX || rf->state == RF_STATE_TX_CALIBR ||
        rf->state == RF_STATE_TX_FINAL) {
        if (rf->rx_incoming_len < 256)
            rf->rx_incoming[rf->rx_incoming_len++] = byte;
        return;
    }
    /* In all other states (including IDLE via software_off), process
     * frames directly.  This simulates perfect reception even when
     * the firmware has nominally turned the radio off (LPM). */

    switch (rf->rx_frame_state) {
        case RF_RX_PREAMBLE:
            if (byte == 0x00) {
                rf->zero_symbols++;
            } else if (rf->zero_symbols >= PREAMBLE_MIN && byte == IEEE802154_SFD) {
                /* Compact RXFIFO before new frame — reclaim consumed space.
                 * Real CC2538 has a circular FIFO; we simulate this by
                 * shifting remaining unread bytes to the front. */
                if (rf->rxfifo_rd > 0) {
                    int remaining = rf->rxfifo_len - rf->rxfifo_rd;
                    if (remaining > 0)
                        memmove(rf->rxfifo, rf->rxfifo + rf->rxfifo_rd,
                                (size_t)remaining);
                    rf->rxfifo_len = remaining;
                    rf->rxfifo_rd = 0;
                }
                /* SFD detected — capture MAC timer.
                 * Always use cpu->cycles (not sim_ns) so the capture is
                 * consistent with the current timer read in get_sfd_timestamp().
                 * The firmware computes delta = timer_val - sfd; both must
                 * use the same time base for the delta to be correct. */
                {
                    int64_t ns = arm_cycles_to_ns(rf->cpu->cycles, rf->cpu->cpu_freq_hz);
                    int64_t mt_ticks = ns_to_mt_ticks(ns);
                    rf->mt_sfd_capture = (uint16_t)(mt_ticks & 0xFFFF);
                    rf->mt_sfd_ovf_capture = (uint32_t)(mt_ticks >> 16) & 0xFFFFFF;
                }
                rfcore_set_state(rf, RF_STATE_RX);
                rf->rfirqf0 |= RFIRQF0_SFD;
                rfcore_check_interrupts(rf);
                rf->rx_frame_state = RF_RX_LENGTH;
                rf->rx_overflow = false;
            } else {
                /* Not a valid preamble sequence, reset */
                rf->zero_symbols = 0;
            }
            break;

        case RF_RX_LENGTH:
            rf->rxlen = byte;
            rf->rx_byte_count = 0;
            /* Write length byte to RXFIFO */
            if (rf->rxfifo_len < RF_RXFIFO_SIZE)
                rf->rxfifo[rf->rxfifo_len++] = byte;
            else {
                rxfifo_overflow_count++;

                rf->rx_overflow = true;
            }
            if (rf->rxlen == 0) {
                /* Empty frame — back to preamble search */
                rf->rx_frame_state = RF_RX_PREAMBLE;
                rf->zero_symbols = 0;
                rfcore_set_state(rf, RF_STATE_SFD_WAIT);
            } else {
                rf->rx_frame_state = RF_RX_PAYLOAD;
            }
            break;

        case RF_RX_PAYLOAD:
            /* Write byte to RXFIFO */
            if (rf->rxfifo_len < RF_RXFIFO_SIZE)
                rf->rxfifo[rf->rxfifo_len++] = byte;
            else {
                rxfifo_overflow_count++;

                rf->rx_overflow = true;
            }

            /* Extract frame header fields */
            if (rf->rx_byte_count == 0) {
                rf->rx_fcf0 = byte;
                rf->rx_ack_request = (byte >> 5) & 1;
            } else if (rf->rx_byte_count == 2) {
                rf->rx_dsn = byte;
            }

            rf->rx_byte_count++;

            /* Check if frame is complete */
            if (rf->rx_byte_count >= rf->rxlen) {
                /* Frame complete.  On real CC2538 hardware the last two
                 * FCS bytes in RXFIFO are replaced with metadata:
                 *   byte N-1 = RSSI (signed, we use a fixed value)
                 *   byte N   = CRC_OK (bit 7) | CORR/LQI (bits 6:0)
                 * The firmware checks CRC_OK before accepting a frame. */
                if (rf->rxfifo_len >= 2 && !rf->rx_overflow) {
                    rf->rxfifo[rf->rxfifo_len - 2] = (uint8_t)(int8_t)rf->rx_rssi; /* RSSI */
                    /* Derive LQI from RSSI: map [-10..-90] -> [100..20]
                     * dist_ratio = -(rssi + 10) / 80; lqi = 100 - dist_ratio * 80 */
                    int lqi = (int)rf->rx_rssi + 110;
                    if (lqi < 20) lqi = 20;
                    if (lqi > 100) lqi = 100;
                    rf->rxfifo[rf->rxfifo_len - 1] = (uint8_t)(0x80 | (lqi & 0x7F)); /* CRC_OK | LQI */
                }

                rf->rfirqf0 |= RFIRQF0_RXPKTDONE | RFIRQF0_FIFOP;
                rfcore_check_interrupts(rf);

                /* Auto-ACK: if FRMCTRL0.AUTOACK (bit 5) is set and the
                 * received frame has ACK_REQUEST, transmit an ACK frame.
                 * Skip if RXFIFO overflowed — the frame is corrupted, so
                 * we must NOT ACK.  This lets the sender's CSMA retransmit.
                 * ACK frame: preamble(4) + SFD(1) + len(1) + FCF(2) + DSN(1) + FCS(2) */
                if ((rf->frmctrl0 & (1 << 5)) && rf->rx_ack_request &&
                    rf->tx_callback && !rf->rx_overflow &&
                    rfcore_dest_addr_matches(rf)) {
                    for (int i = 0; i < PREAMBLE_MIN; i++)
                        rf->tx_callback(rf->tx_user_data, 0x00);
                    rf->tx_callback(rf->tx_user_data, IEEE802154_SFD);
                    rf->tx_callback(rf->tx_user_data, 5);    /* length: FCF(2) + DSN(1) + FCS(2) */
                    uint8_t ack_fcf0 = 0x02; /* frame type = ACK */
                    uint8_t ack_fcf1 = 0x00;
                    uint8_t ack_dsn  = rf->rx_dsn;
                    rf->tx_callback(rf->tx_user_data, ack_fcf0);
                    rf->tx_callback(rf->tx_user_data, ack_fcf1);
                    rf->tx_callback(rf->tx_user_data, ack_dsn);
                    /* Compute FCS (CCITT CRC with bit reversal) */
                    uint16_t ack_crc = 0;
                    ack_crc = crc_add_bitrev(ack_crc, ack_fcf0);
                    ack_crc = crc_add_bitrev(ack_crc, ack_fcf1);
                    ack_crc = crc_add_bitrev(ack_crc, ack_dsn);
                    rf->tx_callback(rf->tx_user_data, bitrev((ack_crc >> 8) & 0xFF));
                    rf->tx_callback(rf->tx_user_data, bitrev(ack_crc & 0xFF));
                    rf->rfirqf1 |= RFIRQF1_TXACKDONE;
                    rfcore_check_interrupts(rf);
                }

                /* Reset for next frame */
                rf->rx_frame_state = RF_RX_PREAMBLE;
                rf->zero_symbols = 0;
                rfcore_set_state(rf, RF_STATE_SFD_WAIT);
            }
            break;
    }
}
