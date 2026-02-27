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

/* MAC timer runs at 32.768 kHz — period in nanoseconds */
#define MT_PERIOD_NS  30517  /* 1e9 / 32768 */

/* --- Interrupt helper --- */

static void rfcore_check_interrupts(cc2538_rfcore_t *rf) {
    uint32_t masked0 = rf->rfirqf0 & rf->rfirqm0;
    uint32_t masked1 = rf->rfirqf1 & rf->rfirqm1;
    if (masked0 || masked1) {
        /* Debug: trace RXPKTDONE interrupt */
        if (masked0 & RFIRQF0_RXPKTDONE) {
            int iser_idx = RFCORE_RX_TX_IRQ / 32;
            int iser_bit = RFCORE_RX_TX_IRQ % 32;
            int enabled = (rf->nvic->iser[iser_idx] >> iser_bit) & 1;
            fprintf(stderr, "[RF node%d] RXPKTDONE IRQ: irq=%d iser[%d]bit%d=%d "
                    "primask=%d active_exc=%d vtor=0x%08x\n",
                    rf->node_id, RFCORE_RX_TX_IRQ, iser_idx, iser_bit, enabled,
                    rf->cpu->primask, rf->nvic->active_exception,
                    rf->cpu->vtor);
        }
        arm_nvic_set_pending(rf->nvic, RFCORE_RX_TX_IRQ);
    }
    if (rf->rferrf & rf->rferrm)
        arm_nvic_set_pending(rf->nvic, RFCORE_ERR_IRQ);
}

/* --- State management --- */

static void rfcore_set_state(cc2538_rfcore_t *rf, rf_state_t state) {
    rf->state = state;
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
        /* Deliver any bytes that arrived during TX */
        if (rf->rx_incoming_len > 0) {
            int len = rf->rx_incoming_len;
            uint8_t buf[256];
            memcpy(buf, rf->rx_incoming, (size_t)len);
            rf->rx_incoming_len = 0;
            for (int i = 0; i < len; i++)
                cc2538_rfcore_receive_byte(rf, buf[i]);
        }
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
            /* Deliver any buffered incoming bytes */
            if (rf->rx_incoming_len > 0) {
                int len = rf->rx_incoming_len;
                uint8_t buf[256];
                memcpy(buf, rf->rx_incoming, (size_t)len);
                rf->rx_incoming_len = 0;
                for (int i = 0; i < len; i++)
                    cc2538_rfcore_receive_byte(rf, buf[i]);
            }
            break;

        case CSP_ISTXON: {
            rf->software_off = false;
            rfcore_set_state(rf, RF_STATE_TX_CALIBR);
            rfcore_set_state(rf, RF_STATE_TX);
            /* Debug: dump TX frame */
            {
                fprintf(stderr, "[RF node%d] TX frame len=%d: ",
                        rf->node_id, rf->txfifo_len > 0 ? rf->txfifo[0] : 0);
                for (int k = 1; k < rf->txfifo_len && k < 21; k++)
                    fprintf(stderr, "%02x ", rf->txfifo[k]);
                fprintf(stderr, "...\n");
            }
            /* Emit 802.15.4 preamble + SFD, then TXFIFO bytes + auto-CRC */
            if (rf->tx_callback) {
                /* 4-byte preamble */
                for (int i = 0; i < PREAMBLE_MIN; i++)
                    rf->tx_callback(rf->tx_user_data, 0x00);
                /* SFD */
                rf->tx_callback(rf->tx_user_data, IEEE802154_SFD);
                /* TXFIFO data (length byte + payload) */
                for (int i = 0; i < rf->txfifo_len; i++)
                    rf->tx_callback(rf->tx_user_data, rf->txfifo[i]);
                /* Auto-append 2-byte FCS (dummy CRC) — the length field
                 * includes these bytes but firmware doesn't write them */
                if (rf->frmctrl0 & (1 << 6)) {
                    rf->tx_callback(rf->tx_user_data, 0x00);
                    rf->tx_callback(rf->tx_user_data, 0x00);
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
            break;
        }

        case CSP_ISTXONCCA:
            /* Simplified: always transmit (CCA is M2 work) */
            rfcore_strobe(rf, CSP_ISTXON);
            break;

        case CSP_ISRFOFF:
            rf->rxenable = 0;
            rf->software_off = true;
            /* Don't actually go to IDLE — keep the RX frame parser
             * active so the simulation medium can deliver frames.
             * FSMSTAT0 reads will report IDLE when software_off is set. */
            break;

        case CSP_ISFLUSHRX:
            rf->rxfifo_len = 0;
            rf->rxfifo_rd = 0;
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
#define FFSM_PAN_ID0      0x098
#define FFSM_PAN_ID1      0x09C
#define FFSM_SHORT_ADDR0  0x0A0
#define FFSM_SHORT_ADDR1  0x0A4
#define FFSM_EXT_ADDR0    0x0A8
#define FFSM_EXT_ADDR1    0x0AC
#define FFSM_EXT_ADDR2    0x0B0
#define FFSM_EXT_ADDR3    0x0B4
#define FFSM_EXT_ADDR4    0x0B8
#define FFSM_EXT_ADDR5    0x0BC
#define FFSM_EXT_ADDR6    0x0C0
#define FFSM_EXT_ADDR7    0x0C4

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
            /* FIFO (bit 5): RX FIFO has data */
            if (rf->rxfifo_rd < rf->rxfifo_len)
                val |= (1 << 5);
            /* FIFOP (bit 6): live status — complete frame available in FIFO.
             * Unlike the RFIRQF0.FIFOP interrupt flag (which gets cleared by
             * the ISR), this hardware signal stays high as long as there are
             * unread bytes in the RXFIFO. */
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
        case RFCORE_SFR_MTM1:
        case RFCORE_SFR_MTMOVF0:
        case RFCORE_SFR_MTMOVF1:
        case RFCORE_SFR_MTMOVF2: {
            /* MAC timer: 40-bit counter at 32.768 kHz derived from CPU cycles.
             * Use cycles directly (not sim_time_ns) since sim_time_ns may not
             * be synced during arm_step()-based execution. */
            int64_t ns = arm_cycles_to_ns(rf->cpu->cycles, rf->cpu->cpu_freq_hz);
            int64_t mt_ticks = ns / MT_PERIOD_NS;
            uint16_t mt_count = (uint16_t)(mt_ticks & 0xFFFF);
            uint32_t mt_ovf = (uint32_t)(mt_ticks >> 16);
            if (offset == RFCORE_SFR_MTM0)     return mt_count & 0xFF;
            if (offset == RFCORE_SFR_MTM1)     return (mt_count >> 8) & 0xFF;
            if (offset == RFCORE_SFR_MTMOVF0)  return mt_ovf & 0xFF;
            if (offset == RFCORE_SFR_MTMOVF1)  return (mt_ovf >> 8) & 0xFF;
            return (int)((mt_ovf >> 16) & 0xFF); /* MTMOVF2 */
        }
        case RFCORE_SFR_RFDATA: {
            if (rf->rxfifo_rd < rf->rxfifo_len) {
                uint8_t b = rf->rxfifo[rf->rxfifo_rd++];
                /* Auto-compact: when all bytes consumed, reclaim space.
                 * Real CC2538 RXFIFO is a circular buffer; consumed
                 * bytes are automatically freed.  The driver's read()
                 * does NOT call ISFLUSHRX after a successful read. */
                if (rf->rxfifo_rd >= rf->rxfifo_len) {
                    fprintf(stderr, "[RF node%d] RFDATA: frame consumed (%d bytes)\n",
                            rf->node_id, rf->rxfifo_len);
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
        case RFCORE_SFR_MTCTRL:   rf->mtctrl = value; break;
        case RFCORE_SFR_MTIRQM:   rf->mtirqm = value; break;
        case RFCORE_SFR_MTIRQF:   rf->mtirqf &= value; break; /* W0C */
        case RFCORE_SFR_MTMSEL:   rf->mtmsel = value; break;
        case RFCORE_SFR_MTM0:     rf->mtm0 = value; break;
        case RFCORE_SFR_MTM1:     rf->mtm1 = value; break;
        case RFCORE_SFR_MTMOVF0:  rf->mtmovf0 = value; break;
        case RFCORE_SFR_MTMOVF1:  rf->mtmovf1 = value; break;
        case RFCORE_SFR_MTMOVF2:  rf->mtmovf2 = value; break;
        case RFCORE_SFR_RFDATA:
            if (rf->txfifo_len < RF_FIFO_SIZE)
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
    rf->txfiltcfg = 0x09;
    rf->txpower = 0xFF;        /* Max power */
    rf->rfrnd_state = 0xDEADBEEF;

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
                /* SFD detected — transition to RX_FRAME */
                rfcore_set_state(rf, RF_STATE_RX);
                rf->rfirqf0 |= RFIRQF0_SFD;
                rfcore_check_interrupts(rf);
                rf->rx_frame_state = RF_RX_LENGTH;
            } else {
                /* Not a valid preamble sequence, reset */
                rf->zero_symbols = 0;
            }
            break;

        case RF_RX_LENGTH:
            rf->rxlen = byte;
            rf->rx_byte_count = 0;
            /* Write length byte to RXFIFO */
            if (rf->rxfifo_len < RF_FIFO_SIZE)
                rf->rxfifo[rf->rxfifo_len++] = byte;
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
            if (rf->rxfifo_len < RF_FIFO_SIZE)
                rf->rxfifo[rf->rxfifo_len++] = byte;

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
                if (rf->rxfifo_len >= 2) {
                    rf->rxfifo[rf->rxfifo_len - 2] = (uint8_t)(-50 & 0xFF); /* RSSI */
                    rf->rxfifo[rf->rxfifo_len - 1] = 0x80 | 50; /* CRC_OK | LQI */
                }

                rf->rfirqf0 |= RFIRQF0_RXPKTDONE | RFIRQF0_FIFOP;
                /* Debug: dump received frame for RPL debugging */
                {
                    int start = rf->rxfifo_len - rf->rxlen;
                    fprintf(stderr, "[RF node%d] RX frame len=%d: ",
                            rf->node_id, rf->rxlen);
                    for (int k = start; k < rf->rxfifo_len; k++)
                        fprintf(stderr, "%02x ", rf->rxfifo[k]);
                    fprintf(stderr, "\n");
                }
                rfcore_check_interrupts(rf);

                /* Auto-ACK: if FRMCTRL0.AUTOACK (bit 5) is set and the
                 * received frame has ACK_REQUEST, transmit an ACK frame.
                 * ACK frame: preamble(4) + SFD(1) + len(1) + FCF(2) + DSN(1) + FCS(2) */
                if ((rf->frmctrl0 & (1 << 5)) && rf->rx_ack_request && rf->tx_callback) {
                    for (int i = 0; i < PREAMBLE_MIN; i++)
                        rf->tx_callback(rf->tx_user_data, 0x00);
                    rf->tx_callback(rf->tx_user_data, IEEE802154_SFD);
                    rf->tx_callback(rf->tx_user_data, 5);    /* length: FCF(2) + DSN(1) + FCS(2) */
                    rf->tx_callback(rf->tx_user_data, 0x02); /* FCF byte 0: frame type = ACK */
                    rf->tx_callback(rf->tx_user_data, 0x00); /* FCF byte 1 */
                    rf->tx_callback(rf->tx_user_data, rf->rx_dsn); /* echo DSN */
                    rf->tx_callback(rf->tx_user_data, 0x00); /* FCS (dummy) */
                    rf->tx_callback(rf->tx_user_data, 0x00); /* FCS (dummy) */
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
