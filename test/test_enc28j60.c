/*
 * ENC28J60 — chip-driver unit tests (mock host, no CPU).
 *
 * The SPI sequences mirror what the Contiki-NG driver and the
 * enc28j60-test example actually emit: RCR with and without the MAC/MII
 * dummy byte, WCR, BFS/BFC for bank switching, RBM/WBM with
 * auto-increment, and SRC.
 */
#include "enc28j60.h"
#include "mock_sim_host.h"
#include <stdio.h>
#include <string.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

#define ASSERT_EQ(actual, expected, msg) do {                            \
    if ((actual) == (expected)) { passed++; }                            \
    else { failed++;                                                     \
        printf("  FAIL: %s — got 0x%lx, want 0x%lx (%s:%d)\n",           \
               msg, (long)(actual), (long)(expected), __FILE__, __LINE__); } \
} while (0)

static void cs(enc28j60_t *c, bool low) { enc28j60_set_cs(c, low); }
static uint8_t xfer(enc28j60_t *c, uint8_t b) { return enc28j60_spi_exchange(c, b); }

/* --- the driver's own primitives (enc28j60.c readreg/writereg/…) ----- */
static uint8_t readreg(enc28j60_t *c, uint8_t reg, bool mac_mii) {
    cs(c, true);
    xfer(c, 0x00 | (reg & 0x1F));       /* RCR */
    if (mac_mii) xfer(c, 0);            /* dummy */
    uint8_t v = xfer(c, 0);
    cs(c, false);
    return v;
}
static void writereg(enc28j60_t *c, uint8_t reg, uint8_t data) {
    cs(c, true);
    xfer(c, 0x40 | (reg & 0x1F));       /* WCR */
    xfer(c, data);
    cs(c, false);
}
static void bitfield(enc28j60_t *c, uint8_t op, uint8_t reg, uint8_t mask) {
    cs(c, true);
    xfer(c, op | (reg & 0x1F));         /* BFS 0x80 / BFC 0xA0 */
    xfer(c, mask);
    cs(c, false);
}
/* setbank() as the enc28j60-test example does it: BFC then BFS on ECON1. */
static void setbank(enc28j60_t *c, uint8_t bank) {
    bitfield(c, 0xA0, ENC_ECON1, ENC_ECON1_BSEL);
    bitfield(c, 0x80, ENC_ECON1, bank & ENC_ECON1_BSEL);
}
static void softreset(enc28j60_t *c) {
    cs(c, true); xfer(c, 0xFF); cs(c, false);
}
static void writedata(enc28j60_t *c, const uint8_t *d, int n) {
    cs(c, true);
    xfer(c, 0x7A);                      /* WBM */
    for (int i = 0; i < n; i++) xfer(c, d[i]);
    cs(c, false);
}
static void readdata(enc28j60_t *c, uint8_t *d, int n) {
    cs(c, true);
    xfer(c, 0x3A);                      /* RBM */
    for (int i = 0; i < n; i++) d[i] = xfer(c, 0);
    cs(c, false);
}
static void set_ptr(enc28j60_t *c, uint8_t reg_low, uint16_t v) {
    writereg(c, reg_low, (uint8_t)(v & 0xFF));
    writereg(c, reg_low + 1, (uint8_t)(v >> 8));
}

static void fixture(mock_sim_host_t *mock, enc28j60_t *c) {
    mock_sim_host_init(mock);
    enc28j60_init(c, &mock->host);
}

/* ====================================================================== */

/* The exact probe() sequence from the enc28j60-test example. */
static void test_probe_sequence(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);

    softreset(&c);
    ASSERT_EQ(readreg(&c, ENC_ESTAT, false) & ENC_ESTAT_CLKRDY, 0,
              "CLKRDY clear immediately after reset");
    mock_sim_host_advance_ns(&mock, 2000000);          /* the driver's 2 ms wait */
    uint8_t estat = readreg(&c, ENC_ESTAT, false);
    ASSERT(estat != 0x00 && estat != 0xFF && (estat & ENC_ESTAT_CLKRDY),
           "ESTAT reads CLKRDY set after the OST");

    setbank(&c, 3);
    ASSERT_EQ(enc28j60_bank(&c), 3, "setbank(3) took effect");
    ASSERT_EQ(readreg(&c, ENC_EREVID, false), 0x06, "EREVID = 06 (rev B7)");

    setbank(&c, 0);
    writereg(&c, ENC_EWRPTL, 0x5A);
    ASSERT_EQ(readreg(&c, ENC_EWRPTL, false), 0x5A, "scratch register round-trips");

    /* MAC address round-trip: six MAC registers, out of address order,
     * each read needing the dummy byte. */
    setbank(&c, 3);
    static const uint8_t maadr[6] = { 0x04, 0x05, 0x02, 0x03, 0x00, 0x01 };
    static const uint8_t mac[6]   = { 0x02, 0xde, 0xad, 0xbe, 0xef, 0x01 };
    for (int i = 0; i < 6; i++) writereg(&c, maadr[i], mac[i]);
    int ok = 1;
    for (int i = 0; i < 6; i++)
        if (readreg(&c, maadr[i], true) != mac[i]) ok = 0;
    ASSERT(ok, "MAC r/w round-trips through all six MAADR registers");

    /* Reading a MAC register *without* the dummy byte returns the dummy,
     * which is the failure the example's error message describes. */
    ASSERT_EQ(readreg(&c, maadr[0], false), 0x00,
              "MAC read without the dummy byte yields the dummy");
}

static void test_dummy_byte_map(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);

    /* Bank 2: everything below EIE is MAC/MII. */
    setbank(&c, 2);
    writereg(&c, ENC_MACON1, 0x0D);
    ASSERT_EQ(readreg(&c, ENC_MACON1, true), 0x0D, "MACON1 needs the dummy byte");

    /* Bank 3: MAADR and MISTAT are MAC/MII, EREVID is not. */
    setbank(&c, 3);
    ASSERT_EQ(readreg(&c, ENC_EREVID, false), 0x06, "EREVID is an ETH register");
    ASSERT_EQ(readreg(&c, ENC_MISTAT, true), 0x00, "MISTAT needs the dummy byte");

    /* Bank 0/1: no MAC/MII registers at all. */
    setbank(&c, 0);
    writereg(&c, ENC_ERDPTL, 0x77);
    ASSERT_EQ(readreg(&c, ENC_ERDPTL, false), 0x77, "bank 0 registers are ETH");

    /* The common registers keep their value across banks. */
    setbank(&c, 0);
    writereg(&c, ENC_EIE, 0x42);
    setbank(&c, 2);
    ASSERT_EQ(readreg(&c, ENC_EIE, false), 0x42, "EIE is visible in every bank");
    setbank(&c, 0);
    ASSERT_EQ(readreg(&c, ENC_EIE, false), 0x42, "…and unchanged on the way back");

    /* Banked registers are per-bank. */
    setbank(&c, 0);
    writereg(&c, 0x10, 0x11);
    setbank(&c, 1);
    writereg(&c, 0x10, 0x22);
    ASSERT_EQ(readreg(&c, 0x10, false), 0x22, "bank 1 copy holds its own value");
    setbank(&c, 0);
    ASSERT_EQ(readreg(&c, 0x10, false), 0x11, "bank 0 copy unaffected");
}

static void test_bit_field_ops(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 0);
    writereg(&c, ENC_ECON2, 0x00);
    bitfield(&c, 0x80, ENC_ECON2, ENC_ECON2_AUTOINC);
    ASSERT_EQ(readreg(&c, ENC_ECON2, false), ENC_ECON2_AUTOINC, "BFS sets bits");
    bitfield(&c, 0xA0, ENC_ECON2, ENC_ECON2_AUTOINC);
    ASSERT_EQ(readreg(&c, ENC_ECON2, false), 0x00, "BFC clears bits");

    /* BFS/BFC on a MAC register must not take effect (datasheet §4.2.3). */
    setbank(&c, 2);
    writereg(&c, ENC_MACON1, 0x00);
    bitfield(&c, 0x80, ENC_MACON1, 0x0D);
    ASSERT_EQ(readreg(&c, ENC_MACON1, true), 0x00, "BFS is a no-op on MAC registers");
}

/* current ERDPT as a 16-bit value */
static uint16_t ptr_low_high(enc28j60_t *c) {
    return (uint16_t)(enc28j60_peek_reg(c, 0, ENC_ERDPTL) |
                      ((uint16_t)enc28j60_peek_reg(c, 0, ENC_ERDPTL + 1) << 8));
}

static void test_buffer_memory(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 0);
    bitfield(&c, 0x80, ENC_ECON2, ENC_ECON2_AUTOINC);

    static const uint8_t frame[8] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11 };
    set_ptr(&c, ENC_EWRPTL, 0x1200);
    writedata(&c, frame, 8);
    ASSERT_EQ(readreg(&c, ENC_EWRPTL, false), 0x08, "EWRPT advanced by 8 (low byte)");
    ASSERT_EQ(readreg(&c, ENC_EWRPTL + 1, false), 0x12, "EWRPT high byte unchanged");

    uint8_t back[8];
    set_ptr(&c, ENC_ERDPTL, 0x1200);
    readdata(&c, back, 8);
    ASSERT(memcmp(frame, back, 8) == 0, "buffer round-trips through WBM/RBM");
    ASSERT_EQ(readreg(&c, ENC_ERDPTL, false), 0x08, "ERDPT advanced by 8");

    /* Without AUTOINC the pointer stands still. */
    bitfield(&c, 0xA0, ENC_ECON2, ENC_ECON2_AUTOINC);
    set_ptr(&c, ENC_ERDPTL, 0x1200);
    readdata(&c, back, 4);
    ASSERT(back[0] == 0x00 && back[1] == 0x00 && back[2] == 0x00,
           "no AUTOINC: every RBM byte re-reads the same address");
    ASSERT_EQ(readreg(&c, ENC_ERDPTL, false), 0x00, "ERDPT unchanged without AUTOINC");

    /* Read pointer wraps from ERXND back to ERXST (circular RX buffer). */
    bitfield(&c, 0x80, ENC_ECON2, ENC_ECON2_AUTOINC);
    set_ptr(&c, ENC_ERXSTL, 0x0000);
    set_ptr(&c, ENC_ERXNDL, 0x0FFF);
    set_ptr(&c, ENC_ERDPTL, 0x0FFF);
    readdata(&c, back, 1);
    ASSERT_EQ(ptr_low_high(&c), 0x0000, "ERDPT wrapped from ERXND to ERXST");
}

static void test_reset_defaults(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 0);
    writereg(&c, ENC_ERDPTL, 0x55);
    writereg(&c, ENC_EIE, 0x33);
    setbank(&c, 2);

    softreset(&c);
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(enc28j60_bank(&c), 0, "SRC returns to bank 0");
    ASSERT_EQ(readreg(&c, ENC_ERDPTL, false), 0x00, "ERDPT cleared by SRC");
    ASSERT_EQ(readreg(&c, ENC_EIE, false), 0x00, "EIE cleared by SRC");
    ASSERT_EQ(readreg(&c, ENC_ERXNDL, false), 0xFF, "ERXND resets to 0x1FFF (low)");
    ASSERT_EQ(readreg(&c, ENC_ERXNDL + 1, false), 0x1F, "ERXND resets to 0x1FFF (high)");
    setbank(&c, 1);
    ASSERT_EQ(readreg(&c, ENC_ERXFCON, false), 0xA1, "ERXFCON resets to UCEN|CRCEN|BCEN");
    setbank(&c, 3);
    ASSERT_EQ(readreg(&c, ENC_EREVID, false), 0x06, "EREVID survives reset");
    ASSERT_EQ(c.stat_resets, 1, "one reset counted");
}

static void test_phy_access(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 2);
    /* Read PHID1 through MIREGADR + MICMD.MIIRD. */
    writereg(&c, ENC_MIREGADR, 0x02);
    writereg(&c, ENC_MICMD, ENC_MICMD_MIIRD);
    ASSERT_EQ(readreg(&c, ENC_MIRDL, true), 0x83, "PHID1 low byte = 0x83");
    ASSERT_EQ(readreg(&c, ENC_MIRDH, true), 0x00, "PHID1 high byte = 0x00");
    setbank(&c, 3);
    ASSERT_EQ(readreg(&c, ENC_MISTAT, true) & ENC_MISTAT_BUSY, 0, "MISTAT not busy");

    /* Link status: PHSTAT2 bit 10. */
    setbank(&c, 2);
    writereg(&c, ENC_MIREGADR, 0x11);
    writereg(&c, ENC_MICMD, ENC_MICMD_MIIRD);
    ASSERT_EQ(readreg(&c, ENC_MIRDH, true), 0x04, "PHSTAT2 reports link up");

    /* Write PHCON2 and read it back. */
    writereg(&c, ENC_MIREGADR, 0x10);
    writereg(&c, ENC_MIWRL, 0x00);
    writereg(&c, ENC_MIWRH, 0x01);          /* the write fires on MIWRH */
    writereg(&c, ENC_MICMD, ENC_MICMD_MIIRD);
    ASSERT_EQ(readreg(&c, ENC_MIRDH, true), 0x01, "PHCON2 write took effect");
}

static void test_transmit_handshake(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 0);
    bitfield(&c, 0x80, ENC_ECON2, ENC_ECON2_AUTOINC);
    set_ptr(&c, ENC_ETXSTL, 0x1200);
    set_ptr(&c, ENC_EWRPTL, 0x1200);
    static const uint8_t pkt[15] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                     0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x06 };
    writedata(&c, pkt, sizeof(pkt));
    set_ptr(&c, ENC_ETXNDL, (uint16_t)(0x1200 + sizeof(pkt) - 1));
    bitfield(&c, 0xA0, ENC_EIR, ENC_EIR_TXIF);

    bitfield(&c, 0x80, ENC_ECON1, ENC_ECON1_TXRTS);
    ASSERT(readreg(&c, ENC_ECON1, false) & ENC_ECON1_TXRTS, "TXRTS set while sending");
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(readreg(&c, ENC_ECON1, false) & ENC_ECON1_TXRTS, 0,
              "TXRTS self-clears (the driver's while-loop terminates)");
    ASSERT(readreg(&c, ENC_EIR, false) & ENC_EIR_TXIF, "EIR.TXIF set after transmit");
    ASSERT_EQ(readreg(&c, ENC_ESTAT, false) & ENC_ESTAT_TXABRT, 0, "no TXABRT");
    ASSERT_EQ(c.stat_tx_frames, 1, "one frame handed to the transmit path");
    ASSERT_EQ(c.mem[0x1201], 0xFF, "frame bytes landed in buffer memory");
}

static void test_pktcnt_and_idle(void) {
    mock_sim_host_t mock; enc28j60_t c;
    fixture(&mock, &c);
    setbank(&c, 1);
    ASSERT_EQ(readreg(&c, ENC_EPKTCNT, false), 0,
              "EPKTCNT is 0 with no frame path (enc28j60_read returns 0)");
    /* PKTDEC is a strobe: it decrements EPKTCNT and reads back clear. */
    setbank(&c, 0);
    bitfield(&c, 0x80, ENC_ECON2, ENC_ECON2_PKTDEC);
    ASSERT_EQ(readreg(&c, ENC_ECON2, false) & ENC_ECON2_PKTDEC, 0, "PKTDEC self-clears");

    /* Bytes clocked while deselected are ignored. */
    cs(&c, false);
    ASSERT_EQ(xfer(&c, 0x9F), 0xFF, "deselected chip floats high");
    enc28j60_destroy(&c);
}

/* ====================================================================== */

int run_enc28j60_tests(int verbose) {
    (void)verbose;
    printf("=== ENC28J60 Mock-Host Tests ===\n");

    test_probe_sequence();
    test_dummy_byte_map();
    test_bit_field_ops();
    test_buffer_memory();
    test_reset_defaults();
    test_phy_access();
    test_transmit_handshake();
    test_pktcnt_and_idle();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
