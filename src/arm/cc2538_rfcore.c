/*
 * CC2538 RF Core — 802.15.4 radio
 */
#include "cc2538_rfcore.h"
#include "arm_nvic.h"
#include <string.h>
#include <stdio.h>

/* Register region sizes */
#define RFCORE_FFSM_SIZE  0x600
#define RFCORE_XREG_SIZE  0x200
#define RFCORE_SFR_SIZE   0x100

static void rfcore_set_state(cc2538_rfcore_t *rf, rf_state_t state) {
    rf->state = state;
    /* Update FSMSTAT1 */
    switch (state) {
        case RF_STATE_IDLE:
            rf->fsmstat1 = 0;
            break;
        case RF_STATE_RX_CALIBR:
        case RF_STATE_SFD_WAIT:
        case RF_STATE_RX:
            rf->fsmstat1 = (1 << 0); /* RX_ACTIVE */
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

static void rfcore_strobe(cc2538_rfcore_t *rf, uint32_t strobe) {
    switch (strobe) {
        case CSP_ISRXON:
            rfcore_set_state(rf, RF_STATE_RX_CALIBR);
            /* In a real implementation, schedule calibration event */
            rfcore_set_state(rf, RF_STATE_SFD_WAIT);
            /* Deliver any buffered incoming bytes */
            if (rf->rx_incoming_len > 0) {
                for (int i = 0; i < rf->rx_incoming_len && rf->rxfifo_len < RF_FIFO_SIZE; i++) {
                    rf->rxfifo[rf->rxfifo_len++] = rf->rx_incoming[i];
                }
                rf->rx_incoming_len = 0;
            }
            break;

        case CSP_ISTXON: {
            rfcore_set_state(rf, RF_STATE_TX_CALIBR);
            rfcore_set_state(rf, RF_STATE_TX);
            /* Transmit all bytes in TXFIFO */
            if (rf->tx_callback) {
                for (int i = 0; i < rf->txfifo_len; i++)
                    rf->tx_callback(rf->tx_user_data, rf->txfifo[i]);
            }
            rf->txfifo_len = 0;
            /* TX done */
            rf->rfirqf1 |= RFIRQF1_TXDONE;
            rfcore_set_state(rf, RF_STATE_IDLE);
            break;
        }

        case CSP_ISTXONCCA:
            /* Only transmit if CCA indicates channel clear */
            /* For now, always transmit (simplified) */
            rfcore_strobe(rf, CSP_ISTXON);
            break;

        case CSP_ISRFOFF:
            rfcore_set_state(rf, RF_STATE_IDLE);
            break;

        case CSP_ISFLUSHRX:
            rf->rxfifo_len = 0;
            rf->rxfifo_rd = 0;
            rf->rx_incoming_len = 0;
            break;

        case CSP_ISFLUSHTX:
            rf->txfifo_len = 0;
            break;
    }
}

/* FFSM registers - mostly address matching tables */
static int ffsm_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    (void)rf;
    uint32_t offset = addr - RFCORE_FFSM_BASE;
    (void)offset;
    return 0;
}

static void ffsm_write(void *user_data, uint32_t addr, uint32_t value) {
    (void)user_data;
    (void)addr;
    (void)value;
}

/* XREG registers - main RF configuration */
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
        case RFCORE_XREG_FSMSTAT0:  return (int)rf->fsmstat0;
        case RFCORE_XREG_FSMSTAT1:  return (int)rf->fsmstat1;
        case RFCORE_XREG_FIFOPCTRL: return (int)rf->fifopctrl;
        case RFCORE_XREG_FSMCTRL:   return (int)rf->fsmctrl;
        case RFCORE_XREG_CCACTRL0:  return (int)rf->ccactrl0;
        case RFCORE_XREG_RSSI:      return (int)rf->rssi;
        case RFCORE_XREG_RFRND: {
            /* Return pseudo-random bit in bit 0 (and bit 1) */
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
            rf->channel = ((value & 0x7F) - 11) / 5 + 11; /* Approximate */
            break;
        case RFCORE_XREG_TXPOWER:   rf->txpower = value; break;
        case RFCORE_XREG_FIFOPCTRL: rf->fifopctrl = value; break;
        case RFCORE_XREG_FSMCTRL:   rf->fsmctrl = value; break;
        case RFCORE_XREG_CCACTRL0:  rf->ccactrl0 = value; break;
        case RFCORE_XREG_AGCCTRL1:  rf->agcctrl1 = value; break;
        case RFCORE_XREG_TXFILTCFG: rf->txfiltcfg = value; break;
    }
}

/* SFR registers - FIFO data, IRQ flags, CSP */
static int sfr_read(void *user_data, uint32_t addr) {
    cc2538_rfcore_t *rf = (cc2538_rfcore_t *)user_data;
    uint32_t offset = addr - RFCORE_SFR_BASE;

    switch (offset) {
        case RFCORE_SFR_MTCSPCFG: return (int)rf->mtcspcfg;
        case RFCORE_SFR_MTCTRL:   return (int)rf->mtctrl;
        case RFCORE_SFR_MTIRQM:   return (int)rf->mtirqm;
        case RFCORE_SFR_MTIRQF:   return (int)rf->mtirqf;
        case RFCORE_SFR_MTMSEL:   return (int)rf->mtmsel;
        case RFCORE_SFR_MTM0:     return (int)rf->mtm0;
        case RFCORE_SFR_MTM1:     return (int)rf->mtm1;
        case RFCORE_SFR_MTMOVF0:  return (int)rf->mtmovf0;
        case RFCORE_SFR_MTMOVF1:  return (int)rf->mtmovf1;
        case RFCORE_SFR_MTMOVF2:  return (int)rf->mtmovf2;
        case RFCORE_SFR_RFDATA: {
            /* Read from RX FIFO */
            if (rf->rxfifo_rd < rf->rxfifo_len) {
                return rf->rxfifo[rf->rxfifo_rd++];
            }
            return 0;
        }
        case RFCORE_SFR_RFERRF:   return (int)rf->rferrf;
        case RFCORE_SFR_RFIRQF0:  return (int)rf->rfirqf0;
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
        case RFCORE_SFR_MTIRQF:   rf->mtirqf &= ~value; break; /* W1C */
        case RFCORE_SFR_MTMSEL:   rf->mtmsel = value; break;
        case RFCORE_SFR_MTM0:     rf->mtm0 = value; break;
        case RFCORE_SFR_MTM1:     rf->mtm1 = value; break;
        case RFCORE_SFR_MTMOVF0:  rf->mtmovf0 = value; break;
        case RFCORE_SFR_MTMOVF1:  rf->mtmovf1 = value; break;
        case RFCORE_SFR_MTMOVF2:  rf->mtmovf2 = value; break;
        case RFCORE_SFR_RFDATA: {
            /* Write to TX FIFO */
            if (rf->txfifo_len < RF_FIFO_SIZE) {
                rf->txfifo[rf->txfifo_len++] = (uint8_t)(value & 0xFF);
            }
            break;
        }
        case RFCORE_SFR_RFERRF:  rf->rferrf &= ~value; break;
        case RFCORE_SFR_RFIRQF0: rf->rfirqf0 &= ~value; break;
        case RFCORE_SFR_RFIRQF1: rf->rfirqf1 &= ~value; break;
        case RFCORE_SFR_RFST:
            rfcore_strobe(rf, value);
            break;
    }
}

void cc2538_rfcore_init(cc2538_rfcore_t *rf, arm_cpu_t *cpu) {
    memset(rf, 0, sizeof(*rf));
    rf->cpu = cpu;
    rf->state = RF_STATE_IDLE;
    rf->fifopctrl = 64; /* Default FIFOP threshold */
    rf->frmfilt0 = 0x0D; /* Frame filtering enabled by default */
    rf->frmctrl0 = 0x40; /* AUTOACK enabled */
    rf->rssi = -128 & 0xFF; /* No signal */
    rf->ccactrl0 = 0xF8; /* Default CCA threshold */
    rf->agcctrl1 = 0x11;
    rf->txfiltcfg = 0x09;
    rf->txpower = 0xFF; /* Max power */
    rf->rfrnd_state = 0xDEADBEEF; /* Seed for pseudo-random generator */

    arm_register_io(cpu, RFCORE_FFSM_BASE, RFCORE_FFSM_SIZE,
                    ffsm_read, ffsm_write, rf);
    arm_register_io(cpu, RFCORE_XREG_BASE, RFCORE_XREG_SIZE,
                    xreg_read, xreg_write, rf);
    arm_register_io(cpu, RFCORE_SFR_BASE, RFCORE_SFR_SIZE,
                    sfr_read, sfr_write, rf);
}

void cc2538_rfcore_set_tx_callback(cc2538_rfcore_t *rf,
                                   cc2538_rf_tx_fn cb, void *user_data) {
    rf->tx_callback = cb;
    rf->tx_user_data = user_data;
}

void cc2538_rfcore_receive_byte(cc2538_rfcore_t *rf, uint8_t byte) {
    if (rf->state == RF_STATE_SFD_WAIT || rf->state == RF_STATE_RX) {
        /* Direct to RX FIFO */
        if (rf->rxfifo_len < RF_FIFO_SIZE) {
            rf->rxfifo[rf->rxfifo_len++] = byte;
        }
    } else {
        /* Buffer for later delivery */
        if (rf->rx_incoming_len < 256) {
            rf->rx_incoming[rf->rx_incoming_len++] = byte;
        }
    }
}
