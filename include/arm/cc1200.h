/*
 * CC1200 sub-GHz IEEE 802.15.4g radio transceiver emulation
 *
 * Modeled after the TI CC1200 (datasheet SWRS123) as used on the
 * Zolertia Firefly board over SSI0. The driver is **CPU-agnostic**:
 * every CPU/GPIO interaction goes through a `sim_host_t` vtable, so
 * the same chip can sit on a CC2538 today or another SoC tomorrow.
 *
 * Scope: enough behaviour to satisfy what Contiki-NG's
 * arch/dev/radio/cc1200/cc1200.c actually exercises at boot, RX and
 * TX — register R/W (single + burst + extended-address 0x2F prefix),
 * strobes that advance MARCSTATE, EXT_PARTNUMBER/EXT_PARTVERSION
 * read-back, GDO0 PKT_SYNC_RXTX edges, RX/TX FIFOs, 802.15.4g
 * variable-length frame format with 32-bit sync word.
 *
 * What's intentionally *not* modeled:
 *   - Frequency synthesis (channel just changes register values).
 *   - Real symbol-rate timing (we use the configured 50 kbps as a
 *     wall-clock byte period).
 *   - eWOR, AES, manual calibration timing details.
 *   - GPIO1/3 — only GDO0 (PB4) and optional GDO2 (PB0) are wired
 *     on Firefly.
 */
#ifndef CC1200_H
#define CC1200_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu_event.h"
#include "sim_host.h"

/* --- Public register address space (matches cc1200-const.h) --- */
#define CC1200_REG_SPACE_SIZE   0x1000   /* covers regular + 0x2F00 ext */

/* Strobes (bit 7=1, bits 6:0 = strobe code) */
#define CC1200_STROBE_SRES      0x30
#define CC1200_STROBE_SFSTXON   0x31
#define CC1200_STROBE_SXOFF     0x32
#define CC1200_STROBE_SCAL      0x33
#define CC1200_STROBE_SRX       0x34
#define CC1200_STROBE_STX       0x35
#define CC1200_STROBE_SIDLE     0x36
#define CC1200_STROBE_SPWD      0x39
#define CC1200_STROBE_SFRX      0x3A
#define CC1200_STROBE_SFTX      0x3B
#define CC1200_STROBE_SNOP      0x3D

/* SPI command bits */
#define CC1200_CMD_READ         0x80
#define CC1200_CMD_BURST        0x40
#define CC1200_CMD_EXT_PREFIX   0x2F   /* "extended-address" prefix in regular space */

/* FIFO addresses */
#define CC1200_DIRECT_FIFO      0x3F   /* same address for TX (write) and RX (read) */

/* MARCSTATE values (low 5 bits) */
#define CC1200_MARC_SLEEP        0x00
#define CC1200_MARC_IDLE         0x01
#define CC1200_MARC_RX           0x0D
#define CC1200_MARC_RX_FIFO_ERR  0x11
#define CC1200_MARC_TX           0x13
#define CC1200_MARC_TX_FIFO_ERR  0x16

/* Status byte states (top nibble bits 6:4) */
#define CC1200_STATUS_IDLE       (0u << 4)
#define CC1200_STATUS_RX         (1u << 4)
#define CC1200_STATUS_TX         (2u << 4)
#define CC1200_STATUS_FSTXON     (3u << 4)
#define CC1200_STATUS_CAL        (4u << 4)
#define CC1200_STATUS_SETTLING   (5u << 4)
#define CC1200_STATUS_RX_FIFO_ERR (6u << 4)
#define CC1200_STATUS_TX_FIFO_ERR (7u << 4)

/* Notable register addresses we react to internally */
#define CC1200_REG_IOCFG3            0x0000
#define CC1200_REG_IOCFG2            0x0001
#define CC1200_REG_IOCFG0            0x0003
#define CC1200_REG_SYNC3             0x0004
#define CC1200_REG_SYNC2             0x0005
#define CC1200_REG_SYNC1             0x0006
#define CC1200_REG_SYNC0             0x0007
#define CC1200_REG_PKT_CFG0          0x0028
#define CC1200_REG_PKT_LEN           0x002E
#define CC1200_EXT_RSSI1             0x2F71
#define CC1200_EXT_MARCSTATE         0x2F73
#define CC1200_EXT_NUM_TXBYTES       0x2FD6
#define CC1200_EXT_NUM_RXBYTES       0x2FD7
#define CC1200_EXT_PARTNUMBER        0x2F8F
#define CC1200_EXT_PARTVERSION       0x2F90

/* GPIOx I/O configuration values (match cc1200-const.h IOCFG_*) */
#define CC1200_IOCFG_PKT_SYNC_RXTX   6

#define CC1200_FIFO_SIZE             128

/* RF TX listener — invoked once per air byte during TX. */
typedef void (*cc1200_rf_callback_fn)(void *user_data, uint8_t byte);

/* SPI protocol decoder state */
typedef enum {
    CC1200_SPI_WAITING,      /* expecting first command byte */
    CC1200_SPI_WAIT_EXT,     /* received 0x2F prefix, expecting addr byte */
    CC1200_SPI_REG_RW,       /* normal register read/write in progress */
    CC1200_SPI_FIFO_RW,      /* TX/RX FIFO direct access */
} cc1200_spi_state_t;

/* Air decoder state — mirrors the CC1200's RX framer */
typedef enum {
    CC1200_AIR_HUNT,         /* searching for sync word */
    CC1200_AIR_PHR,          /* collecting 2-byte PHR */
    CC1200_AIR_PAYLOAD,      /* receiving payload bytes */
} cc1200_air_state_t;

/* Pin descriptor: port (0..3 = A..D), pin (0..7), enabled flag */
typedef struct {
    int port, pin;
    bool enabled;
} cc1200_pin_t;

typedef struct cc1200 {
    /* CPU-agnostic vtable bound at platform init */
    const sim_host_t *host;

    /* Register file (16-bit address space, sparse — most stay zero) */
    uint8_t  regs[CC1200_REG_SPACE_SIZE];

    /* MARCSTATE — single source of truth for the state machine */
    uint8_t  marcstate;

    /* Status byte returned on every SPI exchange (top nibble = state,
     * low nibble = bytes-in-FIFO). */
    uint8_t  status;

    /* CSn level (true = chip selected, low). */
    bool     csn_low;
    /* RESET pin level (false = held in reset, low). */
    bool     reset_low;

    /* SPI protocol state */
    cc1200_spi_state_t spi_state;
    bool     spi_is_read;
    bool     spi_is_burst;
    bool     spi_is_ext;
    uint16_t spi_addr;     /* current register address (with 0x2F00 for ext) */
    bool     spi_first_data;  /* true: next byte is just status (header)  */

    /* TX FIFO */
    uint8_t  tx_fifo[CC1200_FIFO_SIZE];
    int      tx_first;
    int      tx_last;
    int      tx_count;

    /* RX FIFO */
    uint8_t  rx_fifo[CC1200_FIFO_SIZE];
    int      rx_first;
    int      rx_last;
    int      rx_count;

    /* Air-side framing */
    cc1200_air_state_t air_state;
    uint32_t sync_match;      /* shift register for sync-word search */
    int      air_phr_count;
    int      air_payload_remaining;
    int      air_payload_total;

    /* TX byte-emission state */
    int      tx_emit_index;   /* index into tx_fifo+headers being emitted */
    int      tx_emit_total;   /* total air bytes for this packet */
    bool     tx_active;       /* true while TX in progress */
    /* Synthesised TX preamble buffer:
     *   [0..3]   preamble (0x55 x 4)
     *   [4..7]   sync word (SYNC3..SYNC0)
     *   [8..]    payload bytes from TX FIFO
     */
    uint8_t  tx_air_buf[CC1200_FIFO_SIZE + 16];

    /* Pin assignments (driven from platform glue) */
    cc1200_pin_t gdo0;
    cc1200_pin_t gdo2;

    /* Last GDO0 / GDO2 logical levels (true = asserted = "packet sync"). */
    bool     gdo0_level;
    bool     gdo2_level;

    /* Events */
    cpu_event_t tx_byte_event;
    cpu_event_t reset_done_event;

    /* RF TX listener */
    cc1200_rf_callback_fn rf_tx_callback;
    void                 *rf_tx_user_data;

    /* RX RSSI for current frame (set by medium / test runner) */
    int8_t   rx_rssi;

    /* Stats — useful for unit tests + bench logs */
    int      stat_tx_packets;
    int      stat_rx_packets;
    int      stat_strobe_count;
    int      stat_spi_xfers;
} cc1200_t;

/* --- Public API (chip-driver facing) --- */

void    cc1200_init(cc1200_t *chip, const sim_host_t *host);

/* Pin wiring — call once after init. Either pin may be disabled with
 * port < 0. Pins are board-side: the chip drives an "asserted = high"
 * level, the platform's GPIO peripheral handles polarity if any. */
void    cc1200_set_gdo0_pin(cc1200_t *chip, int port, int pin);
void    cc1200_set_gdo2_pin(cc1200_t *chip, int port, int pin);

/* Bus control (driven from platform GPIO output-callback) */
void    cc1200_set_csn(cc1200_t *chip, bool low);
void    cc1200_set_reset(cc1200_t *chip, bool low);

/* SPI byte exchange — called from the SSI exchange callback. */
uint8_t cc1200_spi_exchange(cc1200_t *chip, uint8_t mosi);

/* Inject a received-from-air byte. */
void    cc1200_receive_byte(cc1200_t *chip, uint8_t byte);

/* Install RF TX listener (called for each on-air byte during TX). */
void    cc1200_set_rf_listener(cc1200_t *chip,
                                cc1200_rf_callback_fn cb,
                                void *user_data);

/* Test/inspection helpers (exposed for unit tests) */
uint8_t cc1200_status(const cc1200_t *chip);
int     cc1200_marcstate(const cc1200_t *chip);
int     cc1200_rxfifo_count(const cc1200_t *chip);
int     cc1200_txfifo_count(const cc1200_t *chip);

#endif /* CC1200_H */
