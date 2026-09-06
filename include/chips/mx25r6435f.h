/*
 * mx25r6435f — Macronix MX25R6435F 64 Mbit (8 MiB) serial NOR flash,
 * the on-board flash of the nRF54L15-DK (SPIM00, CS on P2.05).
 *
 * Off-SoC chip in the csim sense (docs/porting-a-device.md §6.4): it
 * only sees a sim_host_t, a chip-select level and SPI bytes, so the
 * same model can sit on any SPI master.
 *
 * Modelled (MX25R6435F datasheet, "Command Set" table):
 *   0x9F RDID   → C2 28 17 (manufacturer, memory type, density)
 *   0x5A RDSFDP → 3 address bytes + 1 dummy, then the SFDP table
 *                 (JESD216B header + JEDEC basic parameter table),
 *                 0xFF beyond it
 *   0x05 RDSR   → status: WIP bit 0, WEL bit 1 (re-read while CS low)
 *   0x15 RDCR   → configuration register (2 bytes)
 *   0x06 WREN / 0x04 WRDI
 *   0x03 READ / 0x0B FAST_READ (1 dummy) — erased array reads 0xFF
 *   0x02 PP     → page program (AND into the array, 256-byte page wrap)
 *   0x20 SE / 0x52 BE32K / 0xD8 BE / 0x60,0xC7 CE — erase to 0xFF
 *   0xAB RES / 0x90 REMS — electronic id / manufacturer+device id
 *   0xB9 DP / 0xAB RDP  — deep power-down (reads then answer 0xFF)
 * Anything else is accepted and answered with 0xFF until CS rises.
 *
 * Program / erase completion sets WIP and clears it after the typical
 * datasheet time via host->schedule_ns, so a firmware that polls RDSR
 * sees the busy window instead of an instantly-done operation.
 *
 * The 8 MiB array is allocated lazily on the first program, so a node
 * that only reads IDs costs nothing.
 */
#ifndef MX25R6435F_H
#define MX25R6435F_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_host.h"

#define MX25R6435F_SIZE        (8u * 1024u * 1024u)
#define MX25R6435F_PAGE        256u
#define MX25R6435F_SFDP_SIZE   256u

#define MX25R6435F_ID_MANUF    0xC2
#define MX25R6435F_ID_TYPE     0x28
#define MX25R6435F_ID_DENSITY  0x17
#define MX25R6435F_ID_ELECTRONIC 0x16   /* RES / REMS device id */

#define MX25R6435F_SR_WIP      (1u << 0)
#define MX25R6435F_SR_WEL      (1u << 1)

typedef enum {
    MX25R_ST_OPCODE = 0,     /* CS low, waiting for the command byte */
    MX25R_ST_ADDR,           /* collecting 3 address bytes */
    MX25R_ST_DUMMY,          /* dummy byte(s) before data */
    MX25R_ST_DATA_OUT,       /* streaming bytes to the host */
    MX25R_ST_DATA_IN,        /* collecting program data */
    MX25R_ST_IGNORE,         /* unknown / finished: 0xFF until CS rises */
} mx25r6435f_state_t;

typedef struct mx25r6435f {
    const sim_host_t *host;

    bool     cs_low;
    mx25r6435f_state_t state;
    uint8_t  opcode;
    uint32_t addr;           /* assembled address / stream cursor */
    int      addr_bytes;     /* address bytes still to collect */
    int      dummy_bytes;    /* dummy bytes still to swallow */
    uint32_t stream_idx;     /* index into the current output stream */

    uint8_t  status;         /* SR: WIP / WEL */
    uint8_t  config[2];      /* CR (RDCR): reset 0x00 0x00 */
    bool     deep_power_down;

    uint8_t *array;          /* MX25R6435F_SIZE bytes, NULL until first program */
    uint8_t  page_buf[MX25R6435F_PAGE];
    uint32_t page_len;
    uint32_t page_addr;

    cpu_event_t busy_event;  /* WIP clear */

    /* Diagnostics */
    uint64_t stat_commands;
    uint64_t stat_programs;
    uint64_t stat_erases;
} mx25r6435f_t;

void    mx25r6435f_init(mx25r6435f_t *c, const sim_host_t *host);
void    mx25r6435f_destroy(mx25r6435f_t *c);      /* frees the lazy array */
void    mx25r6435f_set_cs(mx25r6435f_t *c, bool low);
uint8_t mx25r6435f_spi_exchange(mx25r6435f_t *c, uint8_t mosi);

/* Byte at `addr` as a READ would return it (0xFF when never programmed). */
uint8_t mx25r6435f_peek(const mx25r6435f_t *c, uint32_t addr);

/* The SFDP image the chip serves (MX25R6435F_SFDP_SIZE bytes). */
const uint8_t *mx25r6435f_sfdp_table(void);

#endif /* MX25R6435F_H */
