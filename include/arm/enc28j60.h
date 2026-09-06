/*
 * enc28j60 — Microchip ENC28J60 stand-alone Ethernet controller with
 * SPI interface.  On the nRF54L15-DK this is a module on the expansion
 * header (SPIM22, CS P1.12, 4 MHz, SPI mode 0).
 *
 * Off-SoC chip in the csim sense (docs/porting-a-device.md §6.4): it
 * sees only a sim_host_t, a chip-select level and SPI bytes.
 *
 * Modelled (ENC28J60 datasheet DS39662, §4.0 "Serial Peripheral
 * Interface" and the register map in §3.1):
 *   - the seven SPI instructions: RCR / RBM / WCR / WBM / BFS / BFC /
 *     SRC, encoded as a 3-bit opcode plus a 5-bit argument
 *   - four register banks selected by ECON1.BSEL, with EIE, EIR,
 *     ESTAT, ECON2 and ECON1 (0x1B..0x1F) visible in every bank
 *   - the extra dummy byte that MAC and MII register reads return
 *     before the data (datasheet §4.2.1) — ETH registers do not
 *   - the 8 KiB dual-port buffer behind RBM / WBM, with ERDPT / EWRPT
 *     auto-increment gated by ECON2.AUTOINC and the receive-side
 *     wrap from ERXND back to ERXST
 *   - the PHY registers behind MIREGADR / MIWRL / MIWRH / MIRDL /
 *     MIRDH / MICMD / MISTAT
 *   - ESTAT.CLKRDY appearing ~300 µs after reset (the OST, §2.2), and
 *     ECON1.TXRTS self-clearing with EIR.TXIF once a frame has been
 *     handed to the transmit path
 *
 * Not modelled: DMA/checksum engine, hash-table and pattern-match
 * filters (ERXFCON is stored, not applied), flow control, the built-in
 * self-test (EBSTCON), and wake-on-LAN.
 *
 * The frame path (handing transmitted frames to a host sink, injecting
 * received frames with a status vector) is deliberately absent here —
 * that is a separate step, see devices/nrf54l15-dk/STATUS.md.
 */
#ifndef ENC28J60_H
#define ENC28J60_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_host.h"

#define ENC28J60_MEM_SIZE   0x2000u    /* 8 KiB buffer memory */
#define ENC28J60_REVID      0x06       /* silicon revision B7 */

/* SPI opcodes: bits [7:5] = opcode, bits [4:0] = register address, or a
 * fixed 11010 for the buffer commands (datasheet Table 4-1). */
#define ENC_OP_MASK   0xE0
#define ENC_ARG_MASK  0x1F
#define ENC_OP_RCR    0x00   /* Read Control Register  */
#define ENC_OP_RBM    0x20   /* Read Buffer Memory  (arg must be 11010) */
#define ENC_OP_WCR    0x40   /* Write Control Register */
#define ENC_OP_WBM    0x60   /* Write Buffer Memory (arg must be 11010) */
#define ENC_OP_BFS    0x80   /* Bit Field Set   (ETH registers only) */
#define ENC_OP_BFC    0xA0   /* Bit Field Clear (ETH registers only) */
#define ENC_OP_SRC    0xE0   /* System Reset Command (0xFF) */
#define ENC_BUF_ARG   0x1A   /* the 11010 argument of RBM / WBM */

/* Common registers, visible in every bank */
#define ENC_EIE       0x1B
#define ENC_EIR       0x1C
#define ENC_ESTAT     0x1D
#define ENC_ECON2     0x1E
#define ENC_ECON1     0x1F

#define ENC_ESTAT_CLKRDY  0x01
#define ENC_ESTAT_TXABRT  0x02
#define ENC_ECON1_BSEL    0x03
#define ENC_ECON1_RXEN    0x04
#define ENC_ECON1_TXRTS   0x08
#define ENC_ECON2_PKTDEC  0x40
#define ENC_ECON2_AUTOINC 0x80
#define ENC_EIR_TXIF      0x08
#define ENC_EIR_PKTIF     0x40

/* Bank 0 */
#define ENC_ERDPTL    0x00
#define ENC_EWRPTL    0x02
#define ENC_ETXSTL    0x04
#define ENC_ETXNDL    0x06
#define ENC_ERXSTL    0x08
#define ENC_ERXNDL    0x0A
#define ENC_ERXRDPTL  0x0C
#define ENC_ERXWRPTL  0x0E
/* Bank 1 */
#define ENC_ERXFCON   0x18
#define ENC_EPKTCNT   0x19
/* Bank 2 */
#define ENC_MACON1    0x00
#define ENC_MICMD     0x12
#define ENC_MIREGADR  0x14
#define ENC_MIWRL     0x16
#define ENC_MIWRH     0x17
#define ENC_MIRDL     0x18
#define ENC_MIRDH     0x19
/* Bank 3 */
#define ENC_MAADR5    0x00
#define ENC_MAADR2    0x05
#define ENC_MISTAT    0x0A
#define ENC_EREVID    0x12

#define ENC_MICMD_MIIRD   0x01
#define ENC_MISTAT_BUSY   0x01

typedef enum {
    ENC_ST_OPCODE = 0,   /* CS low, waiting for the instruction byte */
    ENC_ST_RCR_DUMMY,    /* MAC/MII read: one dummy byte first */
    ENC_ST_RCR_DATA,     /* streaming the register value */
    ENC_ST_WCR_DATA,     /* collecting the value to write */
    ENC_ST_BF_DATA,      /* collecting the bit mask for BFS / BFC */
    ENC_ST_RBM,          /* streaming buffer memory out */
    ENC_ST_WBM,          /* streaming buffer memory in */
    ENC_ST_IGNORE,       /* unknown instruction: 0xFF until CS rises */
} enc28j60_state_t;

typedef struct enc28j60 {
    const sim_host_t *host;

    bool     cs_low;
    enc28j60_state_t state;
    uint8_t  opcode;
    uint8_t  reg;              /* register selected by the current RCR/WCR/BF */

    /* Control registers, [bank][address].  Only 0x00..0x1A are banked;
     * 0x1B..0x1F (EIE/EIR/ESTAT/ECON2/ECON1) live in `common`. */
    uint8_t  bank_reg[4][0x1B];
    uint8_t  common[5];        /* EIE, EIR, ESTAT, ECON2, ECON1 */

    uint8_t  mem[ENC28J60_MEM_SIZE];
    uint16_t phy[32];

    cpu_event_t ost_event;     /* ESTAT.CLKRDY after the oscillator start-up */
    cpu_event_t tx_event;      /* ECON1.TXRTS self-clear + EIR.TXIF */

    /* Diagnostics */
    uint64_t stat_commands;
    uint64_t stat_resets;
    uint64_t stat_tx_frames;
} enc28j60_t;

void    enc28j60_init(enc28j60_t *c, const sim_host_t *host);
void    enc28j60_destroy(enc28j60_t *c);
void    enc28j60_set_cs(enc28j60_t *c, bool low);
uint8_t enc28j60_spi_exchange(enc28j60_t *c, uint8_t mosi);

/* Test helpers: read a control register as the host would see it, and
 * the current bank.  No side effects. */
uint8_t enc28j60_peek_reg(const enc28j60_t *c, int bank, uint8_t reg);
int     enc28j60_bank(const enc28j60_t *c);

#endif /* ENC28J60_H */
