/*
 * mx25r6435f — Macronix MX25R6435F serial NOR flash model.  See the
 * header for the command subset and the datasheet sections it follows.
 *
 * Wire protocol (datasheet "SPI Modes", "Command Set"): CS falls, the
 * first byte is the opcode, then per-command address / dummy bytes,
 * then data streams until CS rises.  Every command is a fresh frame:
 * nothing carries across a CS deassert except the status/config
 * registers, the array and a busy timer.
 */
#include "mx25r6435f.h"
#include <stdlib.h>
#include <string.h>

/* SFDP image (JESD216B).  Header + two parameter headers as the
 * MX25R6435F reports them (datasheet "SFDP Table"): revision 1.6,
 * JEDEC basic table (id 0x00, 16 dwords @ 0x30) and the Macronix
 * vendor table (id 0xC2, 4 dwords @ 0x60).  The basic table body carries
 * the density (0x03FFFFFF bits = 64 Mbit), 4 KiB sector erase (0x20),
 * 32/64 KiB block erase (0x52/0xD8), and the fast-read descriptors —
 * the values a driver would act on.  Everything past the tables reads
 * 0xFF, which is what the 256-byte "long SFDP" probe checks at the end
 * of its buffer. */
static const uint8_t sfdp_image[MX25R6435F_SFDP_SIZE] = {
    /* 0x00: SFDP signature "SFDP", minor 6, major 1, NPH = 1 (two headers) */
    0x53, 0x46, 0x44, 0x50, 0x06, 0x01, 0x01, 0xFF,
    /* 0x08: JEDEC basic flash parameter header: id 00, rev 1.6, 16 dwords @ 0x000030 */
    0x00, 0x06, 0x01, 0x10, 0x30, 0x00, 0x00, 0xFF,
    /* 0x10: Macronix parameter header: id C2, rev 1.0, 4 dwords @ 0x000060 */
    0xC2, 0x00, 0x01, 0x04, 0x60, 0x00, 0x00, 0xFF,
    /* 0x18..0x2F: unused */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 0x30: JEDEC basic table
     *  dw1: 4K erase opcode 0x20, 3-byte addr, fast reads supported     */
    0xE5, 0x20, 0xF1, 0xFF,
    /*  dw2: density = 0x03FFFFFF (64 Mbit) */
    0xFF, 0xFF, 0xFF, 0x03,
    /*  dw3: 1-1-4 fast read 0x6B / 1-4-4 fast read 0xEB */
    0x44, 0xEB, 0x08, 0x6B,
    /*  dw4: 1-1-2 fast read 0x3B / 1-2-2 fast read 0xBB */
    0x08, 0x3B, 0x04, 0xBB,
    /*  dw5..dw7: no 2-2-2 / 4-4-4 */
    0xFE, 0xFF, 0xFF, 0xFF,  0xFF, 0xFF, 0x00, 0xFF,  0xFF, 0xFF, 0x44, 0xEB,
    /*  dw8: erase type 1 = 4 KiB (0x20), type 2 = 32 KiB (0x52) */
    0x0C, 0x20, 0x0F, 0x52,
    /*  dw9: erase type 3 = 64 KiB (0xD8), type 4 unused */
    0x10, 0xD8, 0x00, 0xFF,
    /*  dw10: erase / program timings */
    0x23, 0x72, 0xF5, 0x00,
    /*  dw11: page size 256 (2^8), program timings */
    0x82, 0xED, 0x04, 0xCC,
    /*  dw12..dw16: suspend/resume, power-down, quad enable, 4-byte addr */
    0x44, 0x83, 0x68, 0x44,  0x30, 0xB0, 0x30, 0xB0,  0xF7, 0xBD, 0xD5, 0x5C,
    0x4A, 0x9E, 0x29, 0xFF,  0xF0, 0x50, 0xF9, 0x85,
    /* 0x70: Macronix vendor table (4 dwords) */
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
    /* 0x80..0xFF: unimplemented → 0xFF */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

const uint8_t *mx25r6435f_sfdp_table(void) { return sfdp_image; }

/* Opcodes (datasheet "Command Set") */
#define OP_WRSR      0x01
#define OP_PP        0x02
#define OP_READ      0x03
#define OP_WRDI      0x04
#define OP_RDSR      0x05
#define OP_WREN      0x06
#define OP_FAST_READ 0x0B
#define OP_RDCR      0x15
#define OP_SE        0x20
#define OP_BE32K     0x52
#define OP_RDSFDP    0x5A
#define OP_CE_60     0x60
#define OP_REMS      0x90
#define OP_RDID      0x9F
#define OP_RES       0xAB
#define OP_DP        0xB9
#define OP_CE_C7     0xC7
#define OP_BE        0xD8

/* Typical program / erase times (datasheet "AC Characteristics",
 * high-performance mode): tPP 3.5 ms max / ~0.6 ms typ, tSE 40 ms typ,
 * tBE32 0.2 s typ, tBE 0.4 s typ, tCE 20 s typ. */
#define T_PP_NS      600000LL
#define T_SE_NS      40000000LL
#define T_BE32_NS    200000000LL
#define T_BE_NS      400000000LL
#define T_CE_NS      20000000000LL

static void busy_done_cb(void *user, cpu_event_t *ev) {
    (void)ev;
    mx25r6435f_t *c = user;
    c->status &= (uint8_t)~MX25R6435F_SR_WIP;
}

static void start_busy(mx25r6435f_t *c, int64_t ns) {
    c->status |= MX25R6435F_SR_WIP;
    c->status &= (uint8_t)~MX25R6435F_SR_WEL;   /* WEL auto-clears on completion */
    c->busy_event.callback  = busy_done_cb;
    c->busy_event.user_data = c;
    HOST_SCHEDULE_NS(c, &c->busy_event, HOST_NOW_NS(c) + ns);
}

static int ensure_array(mx25r6435f_t *c) {
    if (c->array) return 0;
    c->array = malloc(MX25R6435F_SIZE);
    if (!c->array) return -1;
    memset(c->array, 0xFF, MX25R6435F_SIZE);
    return 0;
}

void mx25r6435f_init(mx25r6435f_t *c, const sim_host_t *host) {
    memset(c, 0, sizeof(*c));
    c->host  = host;
    c->state = MX25R_ST_IGNORE;      /* CS high: nothing listens */
}

void mx25r6435f_destroy(mx25r6435f_t *c) {
    if (c->status & MX25R6435F_SR_WIP) HOST_CANCEL(c, &c->busy_event);
    free(c->array);
    c->array = NULL;
}

uint8_t mx25r6435f_peek(const mx25r6435f_t *c, uint32_t addr) {
    addr &= MX25R6435F_SIZE - 1;
    return c->array ? c->array[addr] : 0xFF;
}

static void erase_range(mx25r6435f_t *c, uint32_t start, uint32_t len, int64_t t_ns) {
    if (!(c->status & MX25R6435F_SR_WEL)) return;     /* needs WREN first */
    c->stat_erases++;
    if (c->array) memset(c->array + (start & (MX25R6435F_SIZE - 1)), 0xFF, len);
    start_busy(c, t_ns);
}

static void finish_program(mx25r6435f_t *c) {
    if (!(c->status & MX25R6435F_SR_WEL) || c->page_len == 0) return;
    if (ensure_array(c) != 0) return;
    /* Page program: bytes wrap inside the 256-byte page (datasheet
     * "Page Program"), and programming can only clear bits. */
    uint32_t page = c->page_addr & ~(MX25R6435F_PAGE - 1);
    uint32_t off  = c->page_addr & (MX25R6435F_PAGE - 1);
    for (uint32_t i = 0; i < c->page_len; i++) {
        uint32_t a = (page + ((off + i) & (MX25R6435F_PAGE - 1))) & (MX25R6435F_SIZE - 1);
        c->array[a] &= c->page_buf[i];
    }
    c->stat_programs++;
    start_busy(c, T_PP_NS);
}

/* CS rising edge ends the frame and executes program/erase commands
 * (they take effect on the deassert, datasheet "Page Program" / "Sector
 * Erase": "...CS# must go high after the last address/data byte"). */
void mx25r6435f_set_cs(mx25r6435f_t *c, bool low) {
    if (low == c->cs_low) return;
    c->cs_low = low;
    if (low) {
        /* Always listen for an opcode: in deep power-down the chip still
         * decodes RES/RDP (0xAB) to wake up, and ignores everything else
         * (datasheet "Deep Power-down": "...the device is not in Standby
         * mode, only the RES instruction is recognized"). */
        c->state = MX25R_ST_OPCODE;
        c->stream_idx = 0;
        return;
    }
    /* Deassert: complete deferred commands. */
    switch (c->opcode) {
        case OP_PP:
            if (c->state == MX25R_ST_DATA_IN) finish_program(c);
            break;
        case OP_SE:
            if (c->state == MX25R_ST_DATA_OUT) erase_range(c, c->addr & ~0xFFFu, 0x1000, T_SE_NS);
            break;
        case OP_BE32K:
            if (c->state == MX25R_ST_DATA_OUT) erase_range(c, c->addr & ~0x7FFFu, 0x8000, T_BE32_NS);
            break;
        case OP_BE:
            if (c->state == MX25R_ST_DATA_OUT) erase_range(c, c->addr & ~0xFFFFu, 0x10000, T_BE_NS);
            break;
        case OP_CE_60: case OP_CE_C7:
            erase_range(c, 0, MX25R6435F_SIZE, T_CE_NS);
            break;
        case OP_WREN:  c->status |= MX25R6435F_SR_WEL;  break;
        case OP_WRDI:  c->status &= (uint8_t)~MX25R6435F_SR_WEL; break;
        case OP_DP:    c->deep_power_down = true;  break;
        case OP_RES:   c->deep_power_down = false; break;   /* RDP shares 0xAB */
        default: break;
    }
    c->opcode = 0;
    c->state  = MX25R_ST_IGNORE;
}

static uint8_t stream_byte(mx25r6435f_t *c) {
    uint8_t out;
    switch (c->opcode) {
        case OP_RDID: {
            static const uint8_t id[3] = { MX25R6435F_ID_MANUF, MX25R6435F_ID_TYPE, MX25R6435F_ID_DENSITY };
            out = id[c->stream_idx % 3];           /* ID repeats while CS stays low */
            break;
        }
        case OP_RDSFDP:
            out = (c->addr < MX25R6435F_SFDP_SIZE) ? sfdp_image[c->addr] : 0xFF;
            c->addr++;
            break;
        case OP_RDSR:  out = c->status; break;
        case OP_RDCR:  out = c->config[c->stream_idx & 1]; break;
        case OP_READ: case OP_FAST_READ:
            out = mx25r6435f_peek(c, c->addr);
            c->addr = (c->addr + 1) & (MX25R6435F_SIZE - 1);
            break;
        case OP_RES:   out = MX25R6435F_ID_ELECTRONIC; break;
        case OP_REMS: {
            /* Alternates manufacturer / device id starting from ADDR[0] */
            uint32_t k = (c->addr & 1u) + c->stream_idx;
            out = (k & 1u) ? MX25R6435F_ID_ELECTRONIC : MX25R6435F_ID_MANUF;
            break;
        }
        default:       out = 0xFF; break;
    }
    c->stream_idx++;
    return out;
}

uint8_t mx25r6435f_spi_exchange(mx25r6435f_t *c, uint8_t mosi) {
    if (!c->cs_low) return 0xFF;
    switch (c->state) {
        case MX25R_ST_OPCODE:
            if (c->deep_power_down && mosi != OP_RES) {
                c->opcode = 0;
                c->state  = MX25R_ST_IGNORE;
                return 0xFF;
            }
            c->opcode = mosi;
            c->stat_commands++;
            c->addr = 0; c->addr_bytes = 0; c->dummy_bytes = 0; c->stream_idx = 0;
            /* While busy only RDSR (and RDCR/RDID) are answered; program /
             * erase / write-enable are ignored (datasheet: "WIP ... only
             * RDSR may be issued"). */
            bool busy = (c->status & MX25R6435F_SR_WIP) != 0;
            switch (mosi) {
                case OP_RDID: case OP_RDSR: case OP_RDCR: case OP_RES:
                    c->state = MX25R_ST_DATA_OUT; break;
                case OP_RDSFDP: case OP_READ:
                    c->addr_bytes = 3; c->dummy_bytes = (mosi == OP_RDSFDP) ? 1 : 0;
                    c->state = busy ? MX25R_ST_IGNORE : MX25R_ST_ADDR; break;
                case OP_FAST_READ:
                    c->addr_bytes = 3; c->dummy_bytes = 1;
                    c->state = busy ? MX25R_ST_IGNORE : MX25R_ST_ADDR; break;
                case OP_REMS:
                    c->addr_bytes = 3; c->state = MX25R_ST_ADDR; break;
                case OP_PP:
                    c->addr_bytes = 3; c->page_len = 0;
                    c->state = busy ? MX25R_ST_IGNORE : MX25R_ST_ADDR; break;
                case OP_SE: case OP_BE32K: case OP_BE:
                    c->addr_bytes = 3;
                    c->state = busy ? MX25R_ST_IGNORE : MX25R_ST_ADDR; break;
                case OP_WREN: case OP_WRDI: case OP_CE_60: case OP_CE_C7:
                case OP_DP:
                    if (busy) c->opcode = 0;            /* dropped */
                    c->state = MX25R_ST_IGNORE; break;  /* acted on at CS rise */
                default:
                    c->opcode = 0;
                    c->state = MX25R_ST_IGNORE; break;
            }
            return 0xFF;

        case MX25R_ST_ADDR:
            c->addr = (c->addr << 8) | mosi;
            if (--c->addr_bytes == 0) {
                c->addr &= 0xFFFFFFu;
                if (c->opcode == OP_PP) { c->page_addr = c->addr; c->state = MX25R_ST_DATA_IN; }
                else if (c->dummy_bytes) c->state = MX25R_ST_DUMMY;
                else c->state = MX25R_ST_DATA_OUT;
            }
            return 0xFF;

        case MX25R_ST_DUMMY:
            if (--c->dummy_bytes == 0) c->state = MX25R_ST_DATA_OUT;
            return 0xFF;

        case MX25R_ST_DATA_OUT:
            return stream_byte(c);

        case MX25R_ST_DATA_IN:
            if (c->page_len < MX25R6435F_PAGE) c->page_buf[c->page_len] = mosi;
            else c->page_buf[c->page_len % MX25R6435F_PAGE] = mosi;   /* page wrap */
            if (c->page_len < MX25R6435F_PAGE) c->page_len++;
            return 0xFF;

        case MX25R_ST_IGNORE:
        default:
            return 0xFF;
    }
}
