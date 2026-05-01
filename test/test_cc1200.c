/*
 * CC1200 chip-driver unit tests (L−1)
 *
 * Drive the CC1200 chip in isolation through the mock_sim_host_t
 * harness. Pattern lifted from test_mock_host.c — no real CPU, no
 * real GPIO peripheral, no SSI.
 *
 * Per docs/porting-a-device.md §7 these unit tests are mandatory for
 * new chip drivers. Bugs caught here would otherwise surface six
 * layers later as "RPL doesn't converge after 60 s of multinode sim".
 */
#include "cc1200.h"
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
        printf("  FAIL: %s — got %d, want %d (%s:%d)\n",                 \
               msg, (int)(actual), (int)(expected), __FILE__, __LINE__); } \
} while (0)

/* Fixture: fresh mock + chip, drive RESET-low/high to bring it to IDLE. */
static void fixture(mock_sim_host_t *mock, cc1200_t *chip) {
    mock_sim_host_init(mock);
    cc1200_init(chip, &mock->host);
    cc1200_set_gdo0_pin(chip, 1, 4);   /* GDO0 → PB4 (Firefly) */
    cc1200_set_gdo2_pin(chip, 1, 0);   /* GDO2 → PB0 (Firefly, optional) */
}

/* SPI helpers — chip-select around a single byte exchange. */
static uint8_t spi_byte(cc1200_t *chip, uint8_t b) {
    return cc1200_spi_exchange(chip, b);
}

static uint8_t spi_strobe(cc1200_t *chip, uint8_t strobe) {
    cc1200_set_csn(chip, true);
    uint8_t s = spi_byte(chip, strobe);
    cc1200_set_csn(chip, false);
    return s;
}

static uint8_t spi_single_read(cc1200_t *chip, uint16_t addr) {
    cc1200_set_csn(chip, true);
    uint8_t v;
    if (addr & 0x2F00) {
        spi_byte(chip, 0x2F | 0x80);            /* extended-addr read */
        spi_byte(chip, (uint8_t)addr);
        v = spi_byte(chip, 0x00);
    } else {
        spi_byte(chip, (uint8_t)(addr | 0x80)); /* regular read */
        v = spi_byte(chip, 0x00);
    }
    cc1200_set_csn(chip, false);
    return v;
}

static void spi_single_write(cc1200_t *chip, uint16_t addr, uint8_t val) {
    cc1200_set_csn(chip, true);
    if (addr & 0x2F00) {
        spi_byte(chip, 0x2F);                   /* extended-addr write */
        spi_byte(chip, (uint8_t)addr);
        spi_byte(chip, val);
    } else {
        spi_byte(chip, (uint8_t)addr);          /* regular write */
        spi_byte(chip, val);
    }
    cc1200_set_csn(chip, false);
}

static void spi_burst_write(cc1200_t *chip, uint16_t addr,
                             const uint8_t *data, int len) {
    cc1200_set_csn(chip, true);
    if (addr & 0x2F00) {
        spi_byte(chip, 0x2F | 0x40);            /* extended burst write */
        spi_byte(chip, (uint8_t)addr);
    } else {
        spi_byte(chip, (uint8_t)(addr | 0x40)); /* regular burst write */
    }
    for (int i = 0; i < len; i++) spi_byte(chip, data[i]);
    cc1200_set_csn(chip, false);
}

static void spi_burst_read(cc1200_t *chip, uint16_t addr,
                            uint8_t *out, int len) {
    cc1200_set_csn(chip, true);
    if (addr & 0x2F00) {
        spi_byte(chip, 0x2F | 0x80 | 0x40);     /* extended burst read */
        spi_byte(chip, (uint8_t)addr);
    } else {
        spi_byte(chip, (uint8_t)(addr | 0x80 | 0x40));
    }
    for (int i = 0; i < len; i++) out[i] = spi_byte(chip, 0x00);
    cc1200_set_csn(chip, false);
}

/* ====================================================================
 * Reset + part-number readback
 * ==================================================================== */

static void test_reset_and_part_number(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    /* SRES strobe should reset everything and leave the chip in SLEEP
     * for ~200 µs, then transition to IDLE. */
    spi_strobe(&chip, CC1200_STROBE_SRES);
    /* Drain the reset-done event. */
    mock_sim_host_advance_ns(&mock, 1000000);  /* 1 ms */
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "post-reset MARCSTATE = IDLE");

    /* PARTNUMBER (0x2F8F) MUST be 0x20 — the Contiki driver's chip-id
     * probe relies on this. */
    uint8_t pn = spi_single_read(&chip, CC1200_EXT_PARTNUMBER);
    ASSERT_EQ(pn, 0x20, "EXT_PARTNUMBER = 0x20");

    /* PARTVERSION (0x2F90) just needs to be non-zero. */
    uint8_t pv = spi_single_read(&chip, CC1200_EXT_PARTVERSION);
    ASSERT(pv != 0, "EXT_PARTVERSION non-zero");
}

/* ====================================================================
 * Single register R/W round-trip
 * ==================================================================== */

static void test_single_register_rw(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Regular-space register (CC1200_PKT_LEN @ 0x2E) */
    spi_single_write(&chip, 0x2E, 0x7F);
    ASSERT_EQ(spi_single_read(&chip, 0x2E), 0x7F, "single rw 0x2E");

    /* Extended-space register (CC1200_FREQ0 @ 0x2F0E) */
    spi_single_write(&chip, 0x2F0E, 0xCC);
    ASSERT_EQ(spi_single_read(&chip, 0x2F0E), 0xCC, "single rw 0x2F0E");
}

/* ====================================================================
 * Burst R/W (regular + extended)
 * ==================================================================== */

static void test_burst_register_rw(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Burst-write SYNC3..SYNC0 (0x04..0x07) and read back. */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);

    uint8_t back[4] = { 0 };
    spi_burst_read(&chip, 0x04, back, 4);
    ASSERT(memcmp(sync, back, 4) == 0, "burst rw SYNC3..SYNC0");

    /* Burst-write 8 bytes in extended space starting at FREQ2 (0x2F0C) */
    uint8_t ext[8] = { 0x56, 0xCC, 0xCC, 0x11, 0x22, 0x33, 0x44, 0x55 };
    spi_burst_write(&chip, 0x2F0C, ext, 8);
    uint8_t ext_back[8] = { 0 };
    spi_burst_read(&chip, 0x2F0C, ext_back, 8);
    ASSERT(memcmp(ext, ext_back, 8) == 0, "burst rw extended FREQ2+");
}

/* ====================================================================
 * Strobe state-machine progression
 * ==================================================================== */

static void test_strobe_state_machine(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "SRES → IDLE");

    spi_strobe(&chip, CC1200_STROBE_SRX);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX,   "SRX → RX");

    spi_strobe(&chip, CC1200_STROBE_SIDLE);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "SIDLE → IDLE");

    spi_strobe(&chip, CC1200_STROBE_SPWD);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_SLEEP, "SPWD → SLEEP");

    /* MARCSTATE register must reflect the same value we read via the API */
    spi_strobe(&chip, CC1200_STROBE_SIDLE);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "post-SLEEP SIDLE → IDLE");
    uint8_t marc = spi_single_read(&chip, CC1200_EXT_MARCSTATE) & 0x1F;
    ASSERT_EQ(marc, CC1200_MARC_IDLE, "MARCSTATE register = IDLE");
}

/* ====================================================================
 * SFD detection on RX → GDO0 rising edge
 * ==================================================================== */

static void test_sfd_detect_and_gdo0_edge(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Configure the 50 kbps Contiki sync word (0x6E4E904E). */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);

    /* IOCFG0 = PKT_SYNC_RXTX (default after init, but be explicit) */
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_PKT_SYNC_RXTX);

    /* Force 802.15.4g mode (FG_MODE = PKT_CFG2 bit 5) so the air decoder
     * expects a 2-byte PHR — that's what the test below feeds. */
    spi_single_write(&chip, CC1200_REG_PKT_CFG2,
                      CC1200_PKT_CFG2_FG_MODE_802154G);

    /* Enter RX so receive_byte will run the air decoder. */
    spi_strobe(&chip, CC1200_STROBE_SRX);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX, "SRX → RX before injecting bytes");

    int prior = mock.force_irq_edge_calls;

    /* Inject preamble (4 × 0x55) — chip should still be hunting. */
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, 0x55);
    ASSERT_EQ(mock.force_irq_edge_calls, prior, "no GDO0 edge during preamble");

    /* Inject the 4-byte sync word — chip should fire GDO0 rising on the
     * last byte (PKT_SYNC_RXTX rising edge = SFD detected). */
    cc1200_receive_byte(&chip, sync[0]);
    cc1200_receive_byte(&chip, sync[1]);
    cc1200_receive_byte(&chip, sync[2]);
    cc1200_receive_byte(&chip, sync[3]);

    ASSERT(mock.force_irq_edge_calls > prior, "GDO0 edge fired on SFD");
    ASSERT_EQ(mock.last_force_irq.port, 1, "GDO0 port = B");
    ASSERT_EQ(mock.last_force_irq.pin,  4, "GDO0 pin = 4");
    ASSERT(mock.last_force_irq.rising, "GDO0 rising on SFD");

    /* On-air after sync: PHR(2) + payload(5) + auto-CRC(2). The chip
     * pushes PHR + payload to the RX FIFO, consumes the 2 CRC bytes
     * silently (medium owns loss), then appends 2 status bytes
     * (RSSI, CRC_OK | LQI) per APPEND_STATUS=1 semantics. */
    cc1200_receive_byte(&chip, 0x00);  /* phra: upper bits of length, no CRC flag */
    cc1200_receive_byte(&chip, 0x05);  /* phrb: payload length = 5 */
    cc1200_receive_byte(&chip, 0xAA);
    cc1200_receive_byte(&chip, 0xBB);
    cc1200_receive_byte(&chip, 0xCC);
    cc1200_receive_byte(&chip, 0xDD);
    cc1200_receive_byte(&chip, 0xEE);
    cc1200_receive_byte(&chip, 0x12);  /* on-air CRC byte 1 (consumed, not stored) */
    cc1200_receive_byte(&chip, 0x34);  /* on-air CRC byte 2 (consumed, not stored) */

    /* GDO0 should now be falling (packet end). */
    ASSERT(!mock.last_force_irq.rising, "GDO0 falling on packet end");

    /* RX FIFO should now contain PHR(2) + payload(5) + appendix(2) = 9 bytes. */
    ASSERT_EQ(cc1200_rxfifo_count(&chip), 9, "rx fifo = PHR+payload+appendix");

    /* NUM_RXBYTES (0x2FD7) should match. */
    uint8_t num_rx = spi_single_read(&chip, CC1200_EXT_NUM_RXBYTES);
    ASSERT_EQ(num_rx, 9, "NUM_RXBYTES register");

    /* Read PHR via RX FIFO direct address (0x3F). */
    uint8_t phr[2];
    spi_burst_read(&chip, CC1200_DIRECT_FIFO, phr, 2);
    ASSERT_EQ(phr[0] & 0x07, 0, "PHR phra upper bits");
    ASSERT_EQ(phr[1], 0x05, "PHR phrb lower bits");

    /* Read payload (5 bytes — the actual MAC bytes are intact). */
    uint8_t pl[5];
    spi_burst_read(&chip, CC1200_DIRECT_FIFO, pl, 5);
    ASSERT_EQ(pl[0], 0xAA, "payload[0]");
    ASSERT_EQ(pl[1], 0xBB, "payload[1]");
    ASSERT_EQ(pl[2], 0xCC, "payload[2]");
    ASSERT_EQ(pl[3], 0xDD, "payload[3]");
    ASSERT_EQ(pl[4], 0xEE, "payload[4]");

    /* Read appendix — RSSI byte then (CRC_OK | LQI). */
    uint8_t app[2];
    spi_burst_read(&chip, CC1200_DIRECT_FIFO, app, 2);
    /* app[0] = chip's rx_rssi (whatever default we initialised). */
    ASSERT_EQ(app[1] & 0x80, 0x80, "CRC OK bit in appendix");
}

/* ====================================================================
 * TX path: write to TX FIFO + STX → RF listener emits bytes
 * ==================================================================== */

static int rf_emit_count;
static uint8_t rf_emit_buf[256];

static void rf_emit_cb(void *user_data, uint8_t b) {
    (void)user_data;
    if (rf_emit_count < (int)sizeof(rf_emit_buf))
        rf_emit_buf[rf_emit_count++] = b;
}

static void test_tx_path(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    rf_emit_count = 0;
    cc1200_set_rf_listener(&chip, rf_emit_cb, NULL);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Configure the standard sync word so we can verify the air format. */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);

    /* Write a 5-byte payload preceded by 2-byte PHR into the TX FIFO.
     * Real Contiki firmware does exactly this (copy_header_to_tx_fifo
     * → burst_write(TXFIFO, payload, len)). */
    uint8_t pkt[7] = { 0x10, 0x05, /* PHR: CRC16, len=5 */
                       0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
    spi_burst_write(&chip, CC1200_DIRECT_FIFO, pkt, 7);

    ASSERT_EQ(cc1200_txfifo_count(&chip), 7, "TX FIFO loaded");
    ASSERT_EQ(spi_single_read(&chip, CC1200_EXT_NUM_TXBYTES), 7,
              "NUM_TXBYTES register");

    /* Start TX */
    spi_strobe(&chip, CC1200_STROBE_STX);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_TX, "STX → TX state");

    /* Drain ~30 ms of byte-emission events. With 50 kbps × 8 bits, the
     * 4+4+7 = 15-byte burst takes 15 × 160 µs = 2.4 ms. */
    mock_sim_host_advance_ns(&mock, 5 * 1000000LL);

    ASSERT(rf_emit_count >= 15, "RF listener saw all air bytes");

    /* Air format: 4 × 0x55 preamble + 4-byte sync word + payload */
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(rf_emit_buf[i], 0x55, "preamble byte");
    ASSERT_EQ(rf_emit_buf[4], 0x6E, "sync[0]");
    ASSERT_EQ(rf_emit_buf[5], 0x4E, "sync[1]");
    ASSERT_EQ(rf_emit_buf[6], 0x90, "sync[2]");
    ASSERT_EQ(rf_emit_buf[7], 0x4E, "sync[3]");
    /* Payload (PHR + 5 bytes) immediately after sync word */
    for (int i = 0; i < 7; i++)
        ASSERT_EQ(rf_emit_buf[8 + i], pkt[i], "payload byte");

    /* After TX completes, MARCSTATE returns to RX (TXOFF_MODE = RX) */
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX, "TX→RX turnaround");
}

/* ====================================================================
 * SFTX / SFRX flush strobes
 * ==================================================================== */

static void test_flush_strobes(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);

    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Load some bytes into TX FIFO */
    uint8_t junk[5] = { 1,2,3,4,5 };
    spi_burst_write(&chip, CC1200_DIRECT_FIFO, junk, 5);
    ASSERT_EQ(cc1200_txfifo_count(&chip), 5, "TX loaded");

    /* SFTX in IDLE → flushes TX FIFO */
    spi_strobe(&chip, CC1200_STROBE_SFTX);
    ASSERT_EQ(cc1200_txfifo_count(&chip), 0, "SFTX flushed TX FIFO");

    /* SFRX in IDLE → flushes RX FIFO (already empty, no-op) */
    spi_strobe(&chip, CC1200_STROBE_SFRX);
    ASSERT_EQ(cc1200_rxfifo_count(&chip), 0, "SFRX kept RX FIFO empty");
}

/* ==================================================================== */

int run_cc1200_tests(int verbose) {
    (void)verbose;
    printf("=== CC1200 Mock-Host Tests ===\n");

    test_reset_and_part_number();
    test_single_register_rw();
    test_burst_register_rw();
    test_strobe_state_machine();
    test_sfd_detect_and_gdo0_edge();
    test_tx_path();
    test_flush_strobes();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
