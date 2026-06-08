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

    /* SRX is event-driven: real silicon takes ~200 µs of CAL+SETTLING
     * to enter RX from IDLE.  MARCSTATE stays at IDLE during that
     * window — match the behaviour of real firmware which busy-waits
     * on STATE_RX before assuming the chip is ready.  See
     * src/arm/cc1200.c for the timing-source rationale. */
    spi_strobe(&chip, CC1200_STROBE_SRX);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "SRX before settling → still IDLE");
    /* During settling the status byte exposes STATE_CAL so firmware
     * polling via SNOP sees a realistic intermediate state. */
    uint8_t cal_status = cc1200_status(&chip) & 0x70;
    ASSERT_EQ(cal_status, CC1200_STATUS_CAL, "SRX in flight → STATE_CAL via SNOP");
    mock_sim_host_advance_ns(&mock, 500000);  /* 500 µs >> 200 µs SRX delay */
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX,   "SRX → RX after settling");

    /* SIDLE is event-driven too — ~50 µs of SETTLING before MARCSTATE
     * leaves RX.  This is the architectural fix that lets a receiver
     * ingest preamble bytes that arrive at the same instant it
     * starts its own CSMA SIDLE → STX path. */
    spi_strobe(&chip, CC1200_STROBE_SIDLE);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX, "SIDLE before settling → still RX");
    mock_sim_host_advance_ns(&mock, 200000);  /* 200 µs >> 50 µs SIDLE delay */
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "SIDLE → IDLE after settling");

    /* SPWD is synchronous (we don't model the wake-up delay). */
    spi_strobe(&chip, CC1200_STROBE_SPWD);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_SLEEP, "SPWD → SLEEP");

    /* MARCSTATE register must reflect the same value we read via the API
     * after the (event-driven) SIDLE settles. */
    spi_strobe(&chip, CC1200_STROBE_SIDLE);
    /* SPWD left us at SLEEP; SIDLE from SLEEP is a no-op in our model
     * because the strobe handler only schedules a transition for
     * RX/TX → IDLE.  From SLEEP we jump straight to IDLE on the next
     * SRES, which firmware always does after wake-up.  Force the
     * issue with an explicit SRES + drain so we test the
     * register-read path. */
    spi_strobe(&chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "post-SLEEP SRES → IDLE");
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

    /* IOCFG0 = PKT_SYNC_RXTX (default after init, but be explicit). */
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_PKT_SYNC_RXTX);
    /* IOCFG2 explicitly pointed at an unmodeled signal so it does NOT
     * fire on sync match.  After the IOCFG-multiplexing rework, only
     * GDO pins whose IOCFG selects PKT_SYNC_RXTX (signal 6) see the
     * sync-match rising edge — which is the datasheet-correct
     * behaviour.  See devices/zoul-firefly/DATASHEET-FINDINGS.md §1. */
    spi_single_write(&chip, CC1200_REG_IOCFG2, CC1200_IOCFG_HIGHZ);

    /* Force 802.15.4g mode (FG_MODE = PKT_CFG2 bit 5) so the air decoder
     * expects a 2-byte PHR — that's what the test below feeds. */
    spi_single_write(&chip, CC1200_REG_PKT_CFG2,
                      CC1200_PKT_CFG2_FG_MODE_802154G);

    /* Enter RX so receive_byte will run the air decoder.  SRX is
     * event-driven now (~200 µs settling) so we need to drain the
     * marcstate event before MARCSTATE actually flips to RX. */
    spi_strobe(&chip, CC1200_STROBE_SRX);
    mock_sim_host_advance_ns(&mock, 500000);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_RX, "SRX → RX before injecting bytes");

    int prior = mock.force_irq_edge_calls;

    /* Inject preamble (4 × 0x55) — chip should still be hunting. */
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, 0x55);
    ASSERT_EQ(mock.force_irq_edge_calls, prior, "no GDO0 edge during preamble");

    /* Inject the 4-byte sync word — chip should fire GDO0 rising on the
     * last byte (PKT_SYNC_RXTX rising edge = SFD detected).  GDO2
     * stays silent because IOCFG2 selects HIGHZ. */
    cc1200_receive_byte(&chip, sync[0]);
    cc1200_receive_byte(&chip, sync[1]);
    cc1200_receive_byte(&chip, sync[2]);
    cc1200_receive_byte(&chip, sync[3]);

    /* Datasheet-correct: only GDO0 fires on sync (its IOCFG selects
     * PKT_SYNC_RXTX); GDO2 is unaffected (its IOCFG selects an
     * unmodeled signal). */
    ASSERT_EQ(mock.force_irq_edge_calls - prior, 1, "only GDO0 edge fired on SFD");
    ASSERT_EQ(mock.last_force_irq.port, 1, "GDO0 port = B");
    ASSERT_EQ(mock.last_force_irq.pin,  4, "GDO0 pin = 4");
    ASSERT(mock.last_force_irq.rising, "GDO0 rising on SFD");

    /* On-air after sync: PHR(2) + payload(5) + auto-CRC(2). The chip
     * pushes PHR + payload to the RX FIFO, consumes the 2 CRC bytes
     * silently (medium owns loss), then appends 2 status bytes
     * (RSSI, CRC_OK | LQI) per APPEND_STATUS=1 semantics.
     *
     * Contiki's CC1200 driver encodes PHR as `payload_len + crc_len`
     * (see arch/dev/radio/cc1200/cc1200.c:copy_header_to_tx_fifo
     * and the matching `payload_len -= 2` in cc1200_rx_interrupt for
     * the 16-bit-CRC default). So for 5 bytes of MAC payload + a
     * 2-byte CRC, PHR encodes 7. The wire then carries 5 payload
     * bytes followed by 2 CRC bytes (= 7 total after PHR), and the
     * chip's air decoder strips the trailing 2 as CRC before
     * pushing the 5 payload bytes into the RX FIFO. */
    cc1200_receive_byte(&chip, 0x00);  /* phra: upper bits of length, no CRC flag */
    cc1200_receive_byte(&chip, 0x07);  /* phrb: payload_len(5) + crc_len(2) = 7 */
    cc1200_receive_byte(&chip, 0xAA);
    cc1200_receive_byte(&chip, 0xBB);
    cc1200_receive_byte(&chip, 0xCC);
    cc1200_receive_byte(&chip, 0xDD);
    cc1200_receive_byte(&chip, 0xEE);
    cc1200_receive_byte(&chip, 0x12);  /* on-air CRC byte 1 (consumed, not stored) */
    cc1200_receive_byte(&chip, 0x34);  /* on-air CRC byte 2 (consumed, not stored) */

    /* End-of-frame is now event-driven: the chip schedules the GDO0
     * falling edge one byte-period (~160 µs) after the last on-air CRC
     * byte. This is what guarantees the simulator's main loop steps
     * the receiver CPU forward in time so its IRQ runs — see
     * docs/porting-a-device.md §8. Advance virtual time enough to
     * drain the deferred event before asserting the edge. */
    mock_sim_host_advance_ns(&mock, 1000000);  /* 1 ms */
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
    ASSERT_EQ(phr[1], 0x07, "PHR phrb lower bits (5 payload + 2 CRC)");

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

    /* Start TX.  STX is event-driven now (~200 µs CAL+SETTLING before
     * MARCSTATE leaves IDLE for TX, then start_tx emits bytes
     * synchronously from the marcstate-event callback).  The
     * intermediate status-byte top nibble is STATE_CAL so firmware
     * polling via SNOP sees a realistic transition. */
    spi_strobe(&chip, CC1200_STROBE_STX);
    ASSERT_EQ(cc1200_marcstate(&chip), CC1200_MARC_IDLE, "STX before settling → still IDLE");
    uint8_t stx_status = cc1200_status(&chip) & 0x70;
    ASSERT_EQ(stx_status, CC1200_STATUS_CAL, "STX in flight → STATE_CAL via SNOP");

    /* Drain ~5 ms of marcstate + byte-emission events. With 50 kbps ×
     * 8 bits, the 4+4+7+2 = 17-byte burst takes 17 × 160 µs ≈ 2.7 ms,
     * plus 200 µs of CAL+SETTLING and 160 µs of post-burst settling
     * before TX → RX turnaround. */
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

/* ====================================================================
 * IOCFG multiplexing — pin-routing tests
 *
 * Pins each map IOCFGx → one of the chip's internal signals.  The mock
 * harness' force_irq_edge_calls counter records every level change
 * driven onto a GDO pin; rising/falling polarity is in last_force_irq.
 *
 * These tests pin the L6 architectural fix described in
 * devices/zoul-firefly/DATASHEET-FINDINGS.md §1 + SWRU346B p.18-19.
 * ==================================================================== */

/* Helper — bring the chip from SLEEP to IDLE after SRES.  All IOCFG
 * tests share this preamble. */
static void prep_idle(mock_sim_host_t *mock, cc1200_t *chip) {
    spi_strobe(chip, CC1200_STROBE_SRES);
    mock_sim_host_advance_ns(mock, 1000000);
}

/* Drive the chip from IDLE to RX (drains the SRX settling event). */
static void prep_rx(mock_sim_host_t *mock, cc1200_t *chip) {
    spi_strobe(chip, CC1200_STROBE_SRX);
    mock_sim_host_advance_ns(mock, 500000);  /* >> 200 µs SRX delay */
}

/* Drive the chip back to IDLE from RX (drains SIDLE settling). */
static void prep_idle_from_rx(mock_sim_host_t *mock, cc1200_t *chip) {
    spi_strobe(chip, CC1200_STROBE_SIDLE);
    mock_sim_host_advance_ns(mock, 200000);  /* >> 50 µs SIDLE delay */
}

static void test_iocfg_multiplexing_basic(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);
    prep_idle(&mock, &chip);

    /* Route GDO0 to MARC_2PIN_STATUS_0 (signal 38).  Park GDO2 at an
     * unmodeled signal so it doesn't add edges. */
    spi_single_write(&chip, CC1200_REG_IOCFG2, CC1200_IOCFG_HIGHZ);
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_MARC_2PIN_STATUS_0);

    /* Currently IDLE (MARC_2PIN = 10, bit0 = 0) → GDO0 should be low. */
    int prior = mock.force_irq_edge_calls;

    /* SRX → settling → RX.  MARC[0] flips 0→1 when MARCSTATE arrives at
     * RX.  GDO0 sees a rising edge driven by the propagate path. */
    prep_rx(&mock, &chip);
    ASSERT(mock.force_irq_edge_calls > prior, "GDO0 fired on IDLE→RX");
    ASSERT_EQ(mock.last_force_irq.port, 1, "edge port = B");
    ASSERT_EQ(mock.last_force_irq.pin,  4, "edge pin = 4 (GDO0)");
    ASSERT(mock.last_force_irq.rising, "GDO0 rising on RX entry");

    /* SIDLE → settling → IDLE.  MARC[0] flips 1→0 → GDO0 falling edge. */
    prior = mock.force_irq_edge_calls;
    prep_idle_from_rx(&mock, &chip);
    ASSERT(mock.force_irq_edge_calls > prior, "GDO0 fired on RX→IDLE");
    ASSERT_EQ(mock.last_force_irq.pin, 4, "edge pin = 4 (GDO0)");
    ASSERT(!mock.last_force_irq.rising, "GDO0 falling on IDLE entry");
}

static void test_iocfg_switch_during_rx(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);
    prep_idle(&mock, &chip);

    /* Configure sync word + IOCFG0 = PKT_SYNC_RXTX, IOCFG2 = HIGHZ. */
    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_PKT_SYNC_RXTX);
    spi_single_write(&chip, CC1200_REG_IOCFG2, CC1200_IOCFG_HIGHZ);
    spi_single_write(&chip, CC1200_REG_PKT_CFG2,
                      CC1200_PKT_CFG2_FG_MODE_802154G);

    prep_rx(&mock, &chip);

    int prior = mock.force_irq_edge_calls;

    /* Inject preamble + sync → sig_pkt_sync_rxtx becomes true →
     * GDO0 rises (IOCFG0 selects signal 6). */
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, 0x55);
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, sync[i]);
    ASSERT(mock.force_irq_edge_calls > prior, "GDO0 rose on sync match");
    ASSERT(mock.last_force_irq.rising, "GDO0 rising");

    /* Firmware reroutes IOCFG0 to MARC_2PIN_STATUS_0 mid-flight.  The
     * chip is still in RX (MARC[0]=1), and PKT_SYNC_RXTX is also true,
     * so the new GDO0 level matches the old → no edge expected. */
    int after_sync = mock.force_irq_edge_calls;
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_MARC_2PIN_STATUS_0);
    ASSERT_EQ(mock.force_irq_edge_calls, after_sync,
              "no GDO0 edge: sync match level = MARC[0] level");

    /* Firmware reroutes back to PKT_SYNC_RXTX mid-frame.  Both signals
     * are still high → no edge. */
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_PKT_SYNC_RXTX);
    ASSERT_EQ(mock.force_irq_edge_calls, after_sync,
              "no GDO0 edge: switching IOCFG between two high signals");

    /* Inject the rest of the frame (PHR + payload + on-air CRC) and
     * advance time to drain the deferred frame_done event.  GDO0 must
     * fall on packet end. */
    cc1200_receive_byte(&chip, 0x00);
    cc1200_receive_byte(&chip, 0x05);
    for (int i = 0; i < 5; i++) cc1200_receive_byte(&chip, 0xAA + i);
    cc1200_receive_byte(&chip, 0x12);
    cc1200_receive_byte(&chip, 0x34);

    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT(!mock.last_force_irq.rising, "GDO0 fell on frame_done");
}

static void test_iocfg_high_z_no_edges(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);
    prep_idle(&mock, &chip);

    /* Park BOTH GDO pins at HIGHZ (unmodeled signal).  GDO0 should
     * stay silent through a complete RX frame. */
    spi_single_write(&chip, CC1200_REG_IOCFG0, CC1200_IOCFG_HIGHZ);
    spi_single_write(&chip, CC1200_REG_IOCFG2, CC1200_IOCFG_HIGHZ);

    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);
    spi_single_write(&chip, CC1200_REG_PKT_CFG2,
                      CC1200_PKT_CFG2_FG_MODE_802154G);

    prep_rx(&mock, &chip);
    int prior = mock.force_irq_edge_calls;

    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, 0x55);
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, sync[i]);
    cc1200_receive_byte(&chip, 0x00);
    cc1200_receive_byte(&chip, 0x05);
    for (int i = 0; i < 5; i++) cc1200_receive_byte(&chip, 0xAA + i);
    cc1200_receive_byte(&chip, 0x12);
    cc1200_receive_byte(&chip, 0x34);
    mock_sim_host_advance_ns(&mock, 1000000);

    ASSERT_EQ(mock.force_irq_edge_calls - prior, 0,
              "no GDO0 edges across full frame when IOCFG = HIGHZ");
}

static void test_iocfg_inv_bit(void) {
    mock_sim_host_t mock; cc1200_t chip;
    fixture(&mock, &chip);
    prep_idle(&mock, &chip);

    /* IOCFG0 = PKT_SYNC_RXTX with the GPIO0_INV bit set.  Logical
     * "asserted" (sig_pkt_sync_rxtx = true) should drive the pin LOW. */
    spi_single_write(&chip, CC1200_REG_IOCFG2, CC1200_IOCFG_HIGHZ);
    spi_single_write(&chip, CC1200_REG_IOCFG0,
                      CC1200_IOCFG_PKT_SYNC_RXTX | CC1200_IOCFG_GPIO0_INV);

    uint8_t sync[4] = { 0x6E, 0x4E, 0x90, 0x4E };
    spi_burst_write(&chip, 0x04, sync, 4);
    spi_single_write(&chip, CC1200_REG_PKT_CFG2,
                      CC1200_PKT_CFG2_FG_MODE_802154G);

    prep_rx(&mock, &chip);
    int prior = mock.force_irq_edge_calls;

    /* Sync match → sig_pkt_sync_rxtx=true → inverted → GDO0 falls. */
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, 0x55);
    for (int i = 0; i < 4; i++) cc1200_receive_byte(&chip, sync[i]);
    ASSERT(mock.force_irq_edge_calls > prior, "GDO0 edge on sync (inverted)");
    ASSERT(!mock.last_force_irq.rising,
           "GDO0 falling on sync match (INV bit set)");

    /* Frame done → sig_pkt_sync_rxtx=false → inverted → GDO0 rises. */
    cc1200_receive_byte(&chip, 0x00);
    cc1200_receive_byte(&chip, 0x05);
    for (int i = 0; i < 5; i++) cc1200_receive_byte(&chip, 0xAA + i);
    cc1200_receive_byte(&chip, 0x12);
    cc1200_receive_byte(&chip, 0x34);
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT(mock.last_force_irq.rising,
           "GDO0 rising on frame_done (INV bit set)");
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
    test_iocfg_multiplexing_basic();
    test_iocfg_switch_during_rx();
    test_iocfg_high_z_no_edges();
    test_iocfg_inv_bit();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
