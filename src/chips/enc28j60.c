/*
 * enc28j60 — Microchip ENC28J60 Ethernet controller model.  See the
 * header for the modelled subset and the datasheet sections (DS39662)
 * it follows.
 *
 * SPI framing (§4.0): CS falls, the first byte is the instruction —
 * 3-bit opcode plus a 5-bit argument — then per-instruction data until
 * CS rises.  Register reads of MAC and MII registers return one dummy
 * byte before the data (§4.2.1); ETH register reads do not.  Which
 * registers are MAC/MII depends on the selected bank, so the dummy byte
 * has to follow the same bank map the driver uses.
 */
#include "enc28j60.h"
#include <string.h>

/* Oscillator start-up timer: CLKRDY is set 300 µs after reset (§2.2). */
#define ENC_OST_NS      300000LL
/* A 1518-byte frame at 10 Mbit/s is ~1.2 ms; the model uses a fixed,
 * plausible turnaround for TXRTS rather than timing the frame, since
 * nothing on the receiving side exists yet. */
#define ENC_TX_NS       200000LL

/* MAC and MII registers need the dummy byte on reads (§3.1, Table 3-1):
 * bank 2 registers 0x00..0x1A are all MAC/MII, and in bank 3 the
 * MAADR set (0x00..0x05) and MISTAT (0x0A) are.  Everything else,
 * including EREVID in bank 3, is an ETH register. */
static bool is_mac_mii(int bank, uint8_t reg) {
    if (reg >= ENC_EIE) return false;         /* common registers are ETH */
    if (bank == 2) return true;
    if (bank == 3) return reg <= ENC_MAADR2 || reg == ENC_MISTAT;
    return false;
}

static inline int enc_bank_of(const enc28j60_t *c) {
    return c->common[ENC_ECON1 - ENC_EIE] & ENC_ECON1_BSEL;
}

int enc28j60_bank(const enc28j60_t *c) { return enc_bank_of(c); }

static uint8_t reg_read(enc28j60_t *c, uint8_t reg) {
    if (reg >= ENC_EIE) return c->common[reg - ENC_EIE];
    return c->bank_reg[enc_bank_of(c)][reg];
}

uint8_t enc28j60_peek_reg(const enc28j60_t *c, int bank, uint8_t reg) {
    if (reg >= ENC_EIE) return c->common[reg - ENC_EIE];
    return c->bank_reg[bank & 3][reg];
}

/* 16-bit pointer pairs are stored as two 8-bit registers, low then
 * high; the high byte only carries bits [12:8] of a buffer address. */
static uint16_t ptr_get(const enc28j60_t *c, uint8_t reg_low) {
    return (uint16_t)(c->bank_reg[0][reg_low] |
                      ((uint16_t)c->bank_reg[0][reg_low + 1] << 8));
}
static void ptr_set(enc28j60_t *c, uint8_t reg_low, uint16_t value) {
    c->bank_reg[0][reg_low]     = (uint8_t)(value & 0xFF);
    c->bank_reg[0][reg_low + 1] = (uint8_t)((value >> 8) & 0x1F);
}

static void mii_write(enc28j60_t *c) {
    uint8_t addr = c->bank_reg[2][ENC_MIREGADR] & 0x1F;
    c->phy[addr] = (uint16_t)(c->bank_reg[2][ENC_MIWRL] |
                              ((uint16_t)c->bank_reg[2][ENC_MIWRH] << 8));
    /* The write takes 10.24 µs on silicon; the model completes it
     * immediately and leaves MISTAT.BUSY clear, which a polling driver
     * reads as "already done". */
    c->bank_reg[3][ENC_MISTAT] &= (uint8_t)~ENC_MISTAT_BUSY;
}

static void mii_read(enc28j60_t *c) {
    uint8_t addr = c->bank_reg[2][ENC_MIREGADR] & 0x1F;
    uint16_t v = c->phy[addr];
    c->bank_reg[2][ENC_MIRDL] = (uint8_t)(v & 0xFF);
    c->bank_reg[2][ENC_MIRDH] = (uint8_t)(v >> 8);
    c->bank_reg[3][ENC_MISTAT] &= (uint8_t)~ENC_MISTAT_BUSY;
}

static void tx_done_cb(void *user, cpu_event_t *ev) {
    (void)ev;
    enc28j60_t *c = user;
    c->common[ENC_ECON1 - ENC_EIE] &= (uint8_t)~ENC_ECON1_TXRTS;
    c->common[ENC_EIR - ENC_EIE]   |= ENC_EIR_TXIF;
}

/* ECON1.TXRTS set: the frame between ETXST and ETXND is handed to the
 * transmit path.  Nothing consumes it yet, so the model only completes
 * the handshake the driver waits on — TXRTS clears and EIR.TXIF is set
 * (§7.1).  Scheduled rather than done inline so the driver's
 * `while(ECON1 & TXRTS)` poll observes a real busy window. */
static void tx_start(enc28j60_t *c) {
    c->stat_tx_frames++;
    c->common[ENC_ESTAT - ENC_EIE] &= (uint8_t)~ENC_ESTAT_TXABRT;
    c->tx_event.callback  = tx_done_cb;
    c->tx_event.user_data = c;
    HOST_SCHEDULE_NS(c, &c->tx_event, HOST_NOW_NS(c) + ENC_TX_NS);
}

static void reg_write(enc28j60_t *c, uint8_t reg, uint8_t value) {
    if (reg >= ENC_EIE) {
        uint8_t prev = c->common[reg - ENC_EIE];
        c->common[reg - ENC_EIE] = value;
        if (reg == ENC_ECON1 && (value & ENC_ECON1_TXRTS) && !(prev & ENC_ECON1_TXRTS))
            tx_start(c);
        if (reg == ENC_ECON2 && (value & ENC_ECON2_PKTDEC)) {
            /* PKTDEC is a strobe: decrement EPKTCNT, then read back 0. */
            if (c->bank_reg[1][ENC_EPKTCNT]) c->bank_reg[1][ENC_EPKTCNT]--;
            c->common[reg - ENC_EIE] &= (uint8_t)~ENC_ECON2_PKTDEC;
        }
        return;
    }
    int bank = enc_bank_of(c);
    c->bank_reg[bank][reg] = value;
    if (bank == 0 && reg == ENC_ERXSTL + 1) {
        /* Writing ERXST also loads ERXWRPT (§6.1). */
        ptr_set(c, ENC_ERXWRPTL, ptr_get(c, ENC_ERXSTL));
    }
    if (bank == 2) {
        if (reg == ENC_MIWRH) mii_write(c);
        if (reg == ENC_MICMD && (value & ENC_MICMD_MIIRD)) mii_read(c);
    }
}

/* BFS / BFC apply to ETH registers only (§4.2.3): the datasheet warns
 * that using them on MAC/MII registers is undefined, and the Contiki
 * driver already routes those through read-modify-write.  The model
 * follows the hardware and applies them to ETH registers only. */
static void reg_bitop(enc28j60_t *c, uint8_t reg, uint8_t mask, bool set) {
    if (is_mac_mii(enc_bank_of(c), reg)) return;
    uint8_t cur = reg_read(c, reg);
    reg_write(c, reg, set ? (uint8_t)(cur | mask) : (uint8_t)(cur & ~mask));
}

static void ost_done_cb(void *user, cpu_event_t *ev) {
    (void)ev;
    enc28j60_t *c = user;
    c->common[ENC_ESTAT - ENC_EIE] |= ENC_ESTAT_CLKRDY;
}

/* Power-on / SRC reset values (§3.1 register tables + §11.0). */
static void enc_reset_state(enc28j60_t *c, bool schedule_ost) {
    memset(c->bank_reg, 0, sizeof(c->bank_reg));
    memset(c->common, 0, sizeof(c->common));
    c->bank_reg[0][ENC_ERXNDL]     = 0xFF;   /* ERXND resets to 0x1FFF */
    c->bank_reg[0][ENC_ERXNDL + 1] = 0x1F;
    c->bank_reg[0][ENC_ERXRDPTL]   = 0xFD;   /* ERXRDPT resets to 0x1FFD */
    c->bank_reg[0][ENC_ERXRDPTL + 1] = 0x1F;
    c->bank_reg[1][ENC_ERXFCON]    = 0xA1;   /* UCEN | CRCEN | BCEN */
    c->bank_reg[2][ENC_MACON1]     = 0x00;
    c->bank_reg[3][ENC_EREVID]     = ENC28J60_REVID;
    /* PHY identification and status (§3.2, Registers 3-1..3-6). */
    memset(c->phy, 0, sizeof(c->phy));
    c->phy[0x00] = 0x3000;   /* PHCON1  */
    c->phy[0x01] = 0x1804;   /* PHSTAT1: full/half capable, LLSTAT set */
    c->phy[0x02] = 0x0083;   /* PHID1   */
    c->phy[0x03] = 0x1400;   /* PHID2   */
    c->phy[0x10] = 0x0000;   /* PHCON2  */
    c->phy[0x11] = 0x0400;   /* PHSTAT2: LSTAT (link up) */
    c->phy[0x14] = 0x3422;   /* PHLCON  */
    c->state  = c->cs_low ? ENC_ST_OPCODE : ENC_ST_IGNORE;
    c->opcode = 0;
    if (schedule_ost) {
        c->ost_event.callback  = ost_done_cb;
        c->ost_event.user_data = c;
        HOST_SCHEDULE_NS(c, &c->ost_event, HOST_NOW_NS(c) + ENC_OST_NS);
    } else {
        c->common[ENC_ESTAT - ENC_EIE] |= ENC_ESTAT_CLKRDY;
    }
}

void enc28j60_init(enc28j60_t *c, const sim_host_t *host) {
    memset(c, 0, sizeof(*c));
    c->host   = host;
    c->cs_low = false;
    /* The chip has been powered long before the firmware runs, so the
     * oscillator is already stable: CLKRDY is set from the start. */
    enc_reset_state(c, false);
    c->state = ENC_ST_IGNORE;
}

void enc28j60_destroy(enc28j60_t *c) {
    HOST_CANCEL(c, &c->ost_event);
    HOST_CANCEL(c, &c->tx_event);
}

void enc28j60_set_cs(enc28j60_t *c, bool low) {
    if (low == c->cs_low) return;
    c->cs_low = low;
    c->state  = low ? ENC_ST_OPCODE : ENC_ST_IGNORE;
    if (!low) c->opcode = 0;
}

/* RBM: read from ERDPT, auto-incrementing when ECON2.AUTOINC is set and
 * wrapping from ERXND back to ERXST (§7.2.2 — the receive buffer is
 * circular). */
static uint8_t buffer_read(enc28j60_t *c) {
    uint16_t rdpt = ptr_get(c, ENC_ERDPTL) & (ENC28J60_MEM_SIZE - 1);
    uint8_t v = c->mem[rdpt];
    if (c->common[ENC_ECON2 - ENC_EIE] & ENC_ECON2_AUTOINC) {
        uint16_t rxnd = ptr_get(c, ENC_ERXNDL);
        uint16_t rxst = ptr_get(c, ENC_ERXSTL);
        rdpt = (rdpt == rxnd) ? rxst : (uint16_t)((rdpt + 1) & (ENC28J60_MEM_SIZE - 1));
        ptr_set(c, ENC_ERDPTL, rdpt);
    }
    return v;
}

/* WBM: write at EWRPT, auto-incrementing when ECON2.AUTOINC is set.
 * The write pointer is linear and wraps at the end of the 8 KiB
 * buffer; it has no ERXND/ERXST wrap. */
static void buffer_write(enc28j60_t *c, uint8_t value) {
    uint16_t wrpt = ptr_get(c, ENC_EWRPTL) & (ENC28J60_MEM_SIZE - 1);
    c->mem[wrpt] = value;
    if (c->common[ENC_ECON2 - ENC_EIE] & ENC_ECON2_AUTOINC)
        ptr_set(c, ENC_EWRPTL, (uint16_t)((wrpt + 1) & (ENC28J60_MEM_SIZE - 1)));
}

uint8_t enc28j60_spi_exchange(enc28j60_t *c, uint8_t mosi) {
    if (!c->cs_low) return 0xFF;
    switch (c->state) {
        case ENC_ST_OPCODE: {
            uint8_t op  = mosi & ENC_OP_MASK;
            uint8_t arg = mosi & ENC_ARG_MASK;
            c->stat_commands++;
            c->opcode = op;
            c->reg    = arg;
            switch (op) {
                case ENC_OP_RCR:
                    c->state = is_mac_mii(enc_bank_of(c), arg)
                               ? ENC_ST_RCR_DUMMY : ENC_ST_RCR_DATA;
                    break;
                case ENC_OP_WCR: c->state = ENC_ST_WCR_DATA; break;
                case ENC_OP_BFS:
                case ENC_OP_BFC: c->state = ENC_ST_BF_DATA;  break;
                case ENC_OP_RBM:
                    c->state = (arg == ENC_BUF_ARG) ? ENC_ST_RBM : ENC_ST_IGNORE;
                    break;
                case ENC_OP_WBM:
                    c->state = (arg == ENC_BUF_ARG) ? ENC_ST_WBM : ENC_ST_IGNORE;
                    break;
                default:                       /* 0xFF = SRC (§11.0) */
                    if (mosi == 0xFF) {
                        c->stat_resets++;
                        enc_reset_state(c, true);
                        c->state = ENC_ST_IGNORE;
                    } else {
                        c->state = ENC_ST_IGNORE;
                    }
                    break;
            }
            return 0x00;                        /* MISO is idle during the opcode */
        }

        case ENC_ST_RCR_DUMMY:
            c->state = ENC_ST_RCR_DATA;
            return 0x00;                        /* the MAC/MII dummy byte */

        case ENC_ST_RCR_DATA:
            /* Holding CS low past the first data byte re-reads the same
             * register, which is what the driver's repeated
             * enc28j60_arch_spi_read() on one select expects. */
            return reg_read(c, c->reg);

        case ENC_ST_WCR_DATA:
            reg_write(c, c->reg, mosi);
            c->state = ENC_ST_IGNORE;
            return 0x00;

        case ENC_ST_BF_DATA:
            reg_bitop(c, c->reg, mosi, c->opcode == ENC_OP_BFS);
            c->state = ENC_ST_IGNORE;
            return 0x00;

        case ENC_ST_RBM:
            return buffer_read(c);

        case ENC_ST_WBM:
            buffer_write(c, mosi);
            return 0x00;

        case ENC_ST_IGNORE:
        default:
            return 0x00;
    }
}
