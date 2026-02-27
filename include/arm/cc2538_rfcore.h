/*
 * CC2538 RF Core — 802.15.4 radio with CSP (Command Strobe Processor)
 */
#ifndef CC2538_RFCORE_H
#define CC2538_RFCORE_H

#include "arm_cpu.h"
#include "arm_nvic.h"

/* RF Core base addresses */
#define RFCORE_FFSM_BASE   0x40088000
#define RFCORE_XREG_BASE   0x40088600
#define RFCORE_SFR_BASE    0x40088800

/* Key XREG offsets */
#define RFCORE_XREG_FRMFILT0   0x000
#define RFCORE_XREG_FRMFILT1   0x004
#define RFCORE_XREG_SRCMATCH   0x008
#define RFCORE_XREG_FRMCTRL0   0x024
#define RFCORE_XREG_FRMCTRL1   0x028
#define RFCORE_XREG_RXENABLE   0x02C
#define RFCORE_XREG_FREQCTRL   0x03C
#define RFCORE_XREG_TXPOWER    0x040
#define RFCORE_XREG_FSMSTAT0   0x048
#define RFCORE_XREG_FSMSTAT1   0x04C
#define RFCORE_XREG_FIFOPCTRL  0x050
#define RFCORE_XREG_FSMCTRL    0x054
#define RFCORE_XREG_CCACTRL0   0x058
#define RFCORE_XREG_RSSI       0x060
#define RFCORE_XREG_RSSISTAT   0x064
#define RFCORE_XREG_RFRND     0x09C
#define RFCORE_XREG_AGCCTRL1   0x0C8
#define RFCORE_XREG_RFC_OBS_CTRL0 0x1AC
#define RFCORE_XREG_RFC_OBS_CTRL1 0x1B0
#define RFCORE_XREG_RFC_OBS_CTRL2 0x1B4
#define RFCORE_XREG_RFIRQM0   0x08C
#define RFCORE_XREG_RFIRQM1   0x090
#define RFCORE_XREG_RFERRM    0x094
#define RFCORE_XREG_TXFILTCFG  0x1E8

/* SFR offsets (relative to SFR_BASE) */
#define RFCORE_SFR_MTCSPCFG    0x000
#define RFCORE_SFR_MTCTRL      0x004
#define RFCORE_SFR_MTIRQM      0x008
#define RFCORE_SFR_MTIRQF      0x00C
#define RFCORE_SFR_MTMSEL      0x010
#define RFCORE_SFR_MTM0        0x014
#define RFCORE_SFR_MTM1        0x018
#define RFCORE_SFR_MTMOVF2     0x01C
#define RFCORE_SFR_MTMOVF1     0x020
#define RFCORE_SFR_MTMOVF0     0x024
#define RFCORE_SFR_RFDATA      0x028
#define RFCORE_SFR_RFERRF      0x02C
#define RFCORE_SFR_RFIRQF1     0x030
#define RFCORE_SFR_RFIRQF0     0x034
#define RFCORE_SFR_RFST        0x038   /* CSP strobe register */

/* CSP strobes */
#define CSP_ISTXON    0xE9
#define CSP_ISRXON    0xE3
#define CSP_ISTXONCCA 0xEA
#define CSP_ISRFOFF   0xEF
#define CSP_ISFLUSHRX 0xED
#define CSP_ISFLUSHTX 0xEE

/* RF state machine states */
typedef enum {
    RF_STATE_IDLE        = 0,
    RF_STATE_RX_CALIBR   = 2,
    RF_STATE_SFD_WAIT    = 3,
    RF_STATE_RX          = 6,
    RF_STATE_RX_OVERFLOW = 17,
    RF_STATE_TX_CALIBR   = 32,
    RF_STATE_TX          = 34,
    RF_STATE_TX_FINAL    = 35,
} rf_state_t;

/* RFIRQF1 bits */
#define RFIRQF1_TXDONE   (1 << 1)
#define RFIRQF1_TXACKDONE (1 << 0)

/* RFIRQF0 bits */
#define RFIRQF0_RXPKTDONE (1 << 6)
#define RFIRQF0_SFD       (1 << 1)
#define RFIRQF0_FIFOP     (1 << 2)

/* IRQ numbers for RF Core */
#define RFCORE_RX_TX_IRQ   141  /* RF Core Rx/Tx */
#define RFCORE_ERR_IRQ     26   /* RF Core Error */

/* TX/RX FIFO sizes */
#define RF_FIFO_SIZE  128

typedef void (*cc2538_rf_tx_fn)(void *user_data, uint8_t byte);

/* RX frame parsing states */
#define RF_RX_PREAMBLE  0
#define RF_RX_LENGTH    1
#define RF_RX_PAYLOAD   2

typedef struct cc2538_rfcore {
    arm_cpu_t  *cpu;
    arm_nvic_t *nvic;

    /* State */
    rf_state_t  state;
    int         channel;

    /* FIFOs */
    uint8_t     txfifo[RF_FIFO_SIZE];
    int         txfifo_len;
    uint8_t     rxfifo[RF_FIFO_SIZE];
    int         rxfifo_len;
    int         rxfifo_rd;

    /* Registers */
    uint32_t    frmfilt0;
    uint32_t    frmfilt1;
    uint32_t    srcmatch;
    uint32_t    frmctrl0;
    uint32_t    frmctrl1;
    uint32_t    freqctrl;
    uint32_t    txpower;
    uint32_t    fsmstat0;
    uint32_t    fsmstat1;
    uint32_t    fifopctrl;
    uint32_t    fsmctrl;
    uint32_t    ccactrl0;
    uint32_t    rssi;
    uint32_t    agcctrl1;
    uint32_t    txfiltcfg;
    uint32_t    rxenable;

    /* Interrupt mask registers */
    uint32_t    rfirqm0;
    uint32_t    rfirqm1;
    uint32_t    rferrm;

    /* SFR registers */
    uint32_t    mtcspcfg;
    uint32_t    mtctrl;
    uint32_t    mtirqm;
    uint32_t    mtirqf;
    uint32_t    mtmsel;
    uint32_t    mtm0;
    uint32_t    mtm1;
    uint32_t    mtmovf0;
    uint32_t    mtmovf1;
    uint32_t    mtmovf2;
    uint32_t    rferrf;
    uint32_t    rfirqf0;
    uint32_t    rfirqf1;

    /* FFSM address registers */
    uint8_t     ext_addr[8];   /* IEEE 64-bit extended address (EXT_ADDR0..7) */
    uint16_t    short_addr;    /* SHORT_ADDR0/1 */
    uint16_t    pan_id;        /* PAN_ID0/1 */

    /* Pseudo-random number generator state (for RFRND register) */
    uint32_t    rfrnd_state;

    /* Incoming RX buffer (bytes arriving before RX is ready) */
    uint8_t     rx_incoming[256];
    int         rx_incoming_len;

    /* RX frame parsing state */
    int         rx_frame_state;     /* RF_RX_PREAMBLE / RF_RX_LENGTH / RF_RX_PAYLOAD */
    int         zero_symbols;       /* Consecutive 0x00 preamble bytes */
    int         rxlen;              /* Expected frame length (from length byte) */
    int         rx_byte_count;      /* Bytes received in current frame payload */
    uint8_t     rx_fcf0;            /* Frame Control Field byte 0 */
    uint8_t     rx_dsn;             /* Data Sequence Number */
    bool        rx_ack_request;     /* ACK requested in received frame */

    /* RF TX callback */
    cc2538_rf_tx_fn  tx_callback;
    void            *tx_user_data;
    int              node_id;

    /* Software-off flag: firmware called ISRFOFF but we keep receiving */
    bool             software_off;

    /* Events */
    arm_event_t      tx_event;
    arm_event_t      rx_cal_event;
} cc2538_rfcore_t;

/* Initialize RF Core and register IO regions */
void cc2538_rfcore_init(cc2538_rfcore_t *rf, arm_cpu_t *cpu, arm_nvic_t *nvic);

/* Set RF TX callback (for multi-node simulation) */
void cc2538_rfcore_set_tx_callback(cc2538_rfcore_t *rf,
                                   cc2538_rf_tx_fn cb, void *user_data);

/* Receive a byte from another radio (multi-node) */
void cc2538_rfcore_receive_byte(cc2538_rfcore_t *rf, uint8_t byte);

#endif /* CC2538_RFCORE_H */
