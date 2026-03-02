/*
 * CC2420 IEEE 802.15.4 radio transceiver emulation
 *
 * Emulates the TI/Chipcon CC2420 radio as used on the Tmote Sky platform.
 * Connected to the MSP430 via SPI (USART0) and several GPIO pins.
 */
#ifndef CC2420_H
#define CC2420_H

#include "msp430_cpu.h"
#include "msp430_gpio.h"

/* --- Radio states --- */
typedef enum {
    CC2420_VREG_OFF       = -1,
    CC2420_POWER_DOWN     = 0,
    CC2420_IDLE           = 1,
    CC2420_RX_CALIBRATE   = 2,
    CC2420_RX_SFD_SEARCH  = 3,
    CC2420_RX_WAIT        = 14,
    CC2420_RX_FRAME       = 16,
    CC2420_RX_OVERFLOW    = 17,
    CC2420_TX_CALIBRATE   = 32,
    CC2420_TX_PREAMBLE    = 34,
    CC2420_TX_FRAME       = 37,
    CC2420_TX_ACK_CALIBRATE = 48,
    CC2420_TX_ACK_PREAMBLE  = 49,
    CC2420_TX_ACK         = 52,
    CC2420_TX_UNDERFLOW   = 56,
} cc2420_radio_state_t;

/* --- SPI protocol states --- */
typedef enum {
    CC2420_SPI_WAITING,
    CC2420_SPI_WRITE_REGISTER,
    CC2420_SPI_READ_REGISTER,
    CC2420_SPI_RAM_ACCESS,
    CC2420_SPI_READ_RXFIFO,
    CC2420_SPI_WRITE_TXFIFO,
} cc2420_spi_state_t;

/* --- Register addresses --- */
/* Strobes 0x00-0x0E */
#define CC2420_REG_SNOP       0x00
#define CC2420_REG_SXOSCON    0x01
#define CC2420_REG_STXCAL     0x02
#define CC2420_REG_SRXON      0x03
#define CC2420_REG_STXON      0x04
#define CC2420_REG_STXONCCA   0x05
#define CC2420_REG_SRFOFF     0x06
#define CC2420_REG_SXOSCOFF   0x07
#define CC2420_REG_SFLUSHRX   0x08
#define CC2420_REG_SFLUSHTX   0x09
#define CC2420_REG_SACK       0x0A
#define CC2420_REG_SACKPEND   0x0B

/* Configuration registers 0x10-0x2F */
#define CC2420_REG_MAIN       0x10
#define CC2420_REG_MDMCTRL0   0x11
#define CC2420_REG_MDMCTRL1   0x12
#define CC2420_REG_RSSI       0x13
#define CC2420_REG_SYNCWORD   0x14
#define CC2420_REG_TXCTRL     0x15
#define CC2420_REG_RXCTRL0    0x16
#define CC2420_REG_RXCTRL1    0x17
#define CC2420_REG_FSCTRL     0x18
#define CC2420_REG_SECCTRL0   0x19
#define CC2420_REG_SECCTRL1   0x1A
#define CC2420_REG_BATTMON    0x1B
#define CC2420_REG_IOCFG0     0x1C
#define CC2420_REG_IOCFG1     0x1D
#define CC2420_REG_MANFIDL    0x1E
#define CC2420_REG_MANFIDH    0x1F
#define CC2420_REG_FSMTC      0x20
#define CC2420_REG_MANAND     0x21
#define CC2420_REG_MANOR      0x22
#define CC2420_REG_AGCCTRL    0x23
#define CC2420_REG_FSMSTATE   0x2C
#define CC2420_REG_TXFIFO     0x3E
#define CC2420_REG_RXFIFO     0x3F

/* RAM addresses */
#define CC2420_RAM_TXFIFO     0x000
#define CC2420_RAM_RXFIFO     0x080
#define CC2420_RAM_KEY0       0x100
#define CC2420_RAM_RXNONCE    0x110
#define CC2420_RAM_SABUF      0x120
#define CC2420_RAM_KEY1       0x130
#define CC2420_RAM_TXNONCE    0x140
#define CC2420_RAM_CBCSTATE   0x150
#define CC2420_RAM_IEEEADDR   0x160
#define CC2420_RAM_PANID      0x168
#define CC2420_RAM_SHORTADDR  0x16A

/* Status byte bits */
#define CC2420_STATUS_XOSC16M_STABLE  (1 << 6)
#define CC2420_STATUS_TX_UNDERFLOW    (1 << 5)
#define CC2420_STATUS_ENC_BUSY        (1 << 4)
#define CC2420_STATUS_TX_ACTIVE       (1 << 3)
#define CC2420_STATUS_LOCK            (1 << 2)
#define CC2420_STATUS_RSSI_VALID      (1 << 1)

/* SPI flag bits (first byte) */
#define CC2420_FLAG_RAM       0x80
#define CC2420_FLAG_READ      0x40
#define CC2420_FLAG_RAM_READ  0x20

/* IOCFG0 bits */
#define CC2420_BCN_ACCEPT       (1 << 11)
#define CC2420_FIFO_POLARITY    (1 << 10)
#define CC2420_FIFOP_POLARITY   (1 << 9)
#define CC2420_SFD_POLARITY     (1 << 8)
#define CC2420_CCA_POLARITY     (1 << 7)
#define CC2420_FIFOP_THR_MASK   0x7F

/* IOCFG1 bits */
#define CC2420_CCAMUX_MASK    0x1F
#define CC2420_CCAMUX_CCA     0
#define CC2420_CCAMUX_XOSC16M 24

/* MDMCTRL0 bits */
#define CC2420_ADR_DECODE     (1 << 11)
#define CC2420_ADR_AUTOCRC    (1 << 5)
#define CC2420_AUTOACK        (1 << 4)

/* Frame type bits */
#define CC2420_FRAME_TYPE_MASK  0x07
#define CC2420_FRAME_TYPE_BEACON  0
#define CC2420_FRAME_TYPE_DATA    1
#define CC2420_FRAME_TYPE_ACK     2
#define CC2420_FRAME_TYPE_CMD     3
#define CC2420_ACK_REQUEST      (1 << 5)

/* Address modes */
#define CC2420_ADDR_MODE_SHORT  2
#define CC2420_ADDR_MODE_LONG   3

/* Timing (in milliseconds) */
#define CC2420_SYMBOL_PERIOD    0.016  /* 62.5 ksym/s */

/* RF listener callback */
typedef void (*cc2420_rf_callback_fn)(void *user_data, uint8_t byte);

/* --- CC2420 state --- */
typedef struct cc2420 {
    msp430_cpu_t  *cpu;
    msp430_gpio_t *gpio;

    /* Radio state */
    cc2420_radio_state_t state;
    bool on;                    /* VREG enabled */
    uint8_t status;             /* SPI status byte */

    /* Registers (16-bit), 64 entries */
    uint16_t registers[64];

    /* RAM (512 bytes) */
    uint8_t memory[512];

    /* SPI state machine */
    cc2420_spi_state_t spi_state;
    bool chip_select;
    int spi_data_pos;
    uint16_t spi_data_value;
    int spi_address;
    bool spi_ram_read;

    /* TX state */
    int tx_cursor;              /* TXFIFO write position */
    int tx_fifo_pos;            /* TX byte read position */
    bool tx_fifo_flush;
    int shr_pos;

    /* RX FIFO (circular buffer within memory[0x80..0xFF]) */
    int rx_fifo_write_pos;      /* write index (0-127) */
    int rx_fifo_read_pos;       /* read index (0-127) */
    int rx_fifo_len;            /* current byte count */
    int rxlen;                  /* expected frame length */
    int rxread;                 /* bytes received in current frame */
    int zero_symbols;           /* preamble detection */
    bool crc_ok;
    bool frame_rejected;
    bool overflow;
    bool should_ack;

    /* FIFOP tracking */
    int rx_fifo_read_left;      /* bytes remaining in current packet */

    /* Saved write position for frame rejection */
    int rx_fifo_mark_pos;
    int rx_fifo_mark_len;

    /* Pin assignments (1-based port, 0-based pin) */
    int fifop_port, fifop_pin;
    int fifo_port, fifo_pin;
    int cca_port, cca_pin;
    int sfd_port, sfd_pin;

    /* Current pin states */
    bool current_fifop;
    bool current_fifo;
    bool current_cca;
    bool current_sfd;

    /* Configuration (derived from registers) */
    int fifop_thr;
    bool adr_decode;
    bool auto_crc;
    bool auto_ack;

    /* Frame decode state */
    int fcf0, fcf1;
    int frame_type;
    int dsn;
    int dest_addr_mode;
    bool ack_request;
    bool decode_address;

    /* Events */
    msp430_event_t vreg_event;
    msp430_event_t oscillator_event;
    msp430_event_t symbol_event;
    msp430_event_t sfd_clear_event;  /* deferred SFD pin clear after frame RX */

    /* ACK state */
    uint8_t ack_buf[6];
    int ack_pos;
    bool ack_frame_pending;

    /* Incoming byte buffer (for bytes arriving during RX_CALIBRATE/RX_WAIT) */
    uint8_t rx_incoming[256];
    int rx_incoming_count;

    /* Debug: node identity for logging */
    int node_id;

    /* RSSI value set by radio medium for current RX frame */
    int8_t rx_rssi;

    /* RF listener */
    cc2420_rf_callback_fn rf_tx_callback;
    void *rf_tx_data;
} cc2420_t;

/* --- Public API --- */

void cc2420_init(cc2420_t *radio, msp430_cpu_t *cpu, msp430_gpio_t *gpio);

void cc2420_set_pins(cc2420_t *radio,
                      int fifop_port, int fifop_pin,
                      int fifo_port, int fifo_pin,
                      int cca_port, int cca_pin,
                      int sfd_port, int sfd_pin);

void cc2420_set_chip_select(cc2420_t *radio, bool select);
void cc2420_set_vreg(cc2420_t *radio, bool on);

/* SPI data exchange — called by USART SPI callback */
uint8_t cc2420_spi_exchange(cc2420_t *radio, uint8_t data);

/* Inject a received byte (over the air) */
void cc2420_receive_byte(cc2420_t *radio, uint8_t data);

/* Register RF TX listener (called for each byte transmitted over the air) */
void cc2420_set_rf_listener(cc2420_t *radio, cc2420_rf_callback_fn cb, void *data);

/* Get global RX statistics (for debugging) */
void cc2420_get_rx_stats(int *started, int *completed, int *rejected,
                          int *overflowed, int *crc_good, int *crc_bad,
                          int *dropped);

#endif /* CC2420_H */
