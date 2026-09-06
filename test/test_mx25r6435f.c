/*
 * MX25R6435F flash — chip-driver unit tests (mock host, no CPU).
 *
 * Drives the chip the way the Contiki spi-flash example and a generic
 * NOR driver do: CS low, opcode, address/dummy bytes, data, CS high.
 * The exchange sequences mirror examples/platform-specific/nrf/spi-flash
 * exactly for RDID / RDSFDP, and add program/erase/status coverage the
 * example does not exercise.
 */
#include "mx25r6435f.h"
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

static void fixture(mock_sim_host_t *mock, mx25r6435f_t *c) {
    mock_sim_host_init(mock);
    mx25r6435f_init(c, &mock->host);
}

static uint8_t xfer(mx25r6435f_t *c, uint8_t b) { return mx25r6435f_spi_exchange(c, b); }
static void cs(mx25r6435f_t *c, bool low)     { mx25r6435f_set_cs(c, low); }

static void cmd_addr(mx25r6435f_t *c, uint8_t op, uint32_t addr) {
    xfer(c, op);
    xfer(c, (uint8_t)(addr >> 16));
    xfer(c, (uint8_t)(addr >> 8));
    xfer(c, (uint8_t)addr);
}

static uint8_t rdsr(mx25r6435f_t *c) {
    cs(c, true); xfer(c, 0x05); uint8_t s = xfer(c, 0); cs(c, false);
    return s;
}

/* ====================================================================== */

static void test_jedec_id(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    /* spi-flash.c read_jedec_id(): opcode in one transfer, 3 bytes in the next,
     * CS held low across both — the chip must not care about the split. */
    cs(&c, true);
    xfer(&c, 0x9F);
    uint8_t id[3] = { xfer(&c, 0), xfer(&c, 0), xfer(&c, 0) };
    cs(&c, false);
    ASSERT(id[0] == 0xC2 && id[1] == 0x28 && id[2] == 0x17, "JEDEC ID c2 28 17");

    /* ID keeps cycling while CS stays low (datasheet RDID). */
    cs(&c, true); xfer(&c, 0x9F);
    for (int i = 0; i < 3; i++) xfer(&c, 0);
    ASSERT_EQ(xfer(&c, 0), 0xC2, "RDID wraps to manufacturer byte");
    cs(&c, false);
}

static void test_sfdp_header_and_tail(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    /* read_sfdp_signature(): 5A 00 00 00 + dummy, then 4 bytes. */
    cs(&c, true);
    cmd_addr(&c, 0x5A, 0);
    xfer(&c, 0);                                   /* dummy */
    uint8_t sig[4] = { xfer(&c, 0), xfer(&c, 0), xfer(&c, 0), xfer(&c, 0) };
    cs(&c, false);
    ASSERT(sig[0] == 'S' && sig[1] == 'F' && sig[2] == 'D' && sig[3] == 'P', "SFDP signature");

    /* read_sfdp_long(): 256 bytes in one frame (200 kept + 56 discarded). */
    uint8_t buf[256];
    cs(&c, true);
    cmd_addr(&c, 0x5A, 0);
    xfer(&c, 0);
    for (int i = 0; i < 256; i++) buf[i] = xfer(&c, 0);
    cs(&c, false);
    ASSERT(memcmp(buf, "SFDP", 4) == 0, "long SFDP starts with signature");
    ASSERT_EQ(buf[4], 0x06, "SFDP minor revision 6");
    ASSERT_EQ(buf[5], 0x01, "SFDP major revision 1");
    ASSERT_EQ(buf[6], 0x01, "two parameter headers (NPH=1)");
    ASSERT_EQ(buf[0x0C], 0x30, "JEDEC table pointer 0x30");
    ASSERT_EQ(buf[0x10], 0xC2, "Macronix vendor header id");
    ASSERT(buf[0x34] == 0xFF && buf[0x35] == 0xFF && buf[0x36] == 0xFF && buf[0x37] == 0x03,
           "density dword = 0x03FFFFFF (64 Mbit)");
    ASSERT_EQ(buf[0x31], 0x20, "4 KiB erase opcode 0x20");
    ASSERT(buf[198] == 0xFF && buf[199] == 0xFF, "bytes 198/199 read 0xFF (past the tables)");
    ASSERT(memcmp(buf, mx25r6435f_sfdp_table(), 256) == 0, "stream equals the published table");

    /* Address offset: reading from 0x30 lands on the JEDEC table. */
    cs(&c, true);
    cmd_addr(&c, 0x5A, 0x30);
    xfer(&c, 0);
    ASSERT_EQ(xfer(&c, 0), 0xE5, "SFDP read honours the address");
    cs(&c, false);
}

static void test_status_and_write_enable(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    ASSERT_EQ(rdsr(&c), 0x00, "status idle after reset");
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);           /* WREN */
    ASSERT_EQ(rdsr(&c), MX25R6435F_SR_WEL, "WREN sets WEL");
    cs(&c, true); xfer(&c, 0x04); cs(&c, false);           /* WRDI */
    ASSERT_EQ(rdsr(&c), 0x00, "WRDI clears WEL");

    /* RDSR streams continuously while CS is low. */
    cs(&c, true); xfer(&c, 0x05);
    uint8_t a = xfer(&c, 0), b = xfer(&c, 0);
    cs(&c, false);
    ASSERT(a == 0 && b == 0, "RDSR re-read while selected");
}

static void test_program_read_erase(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    /* Erased array reads 0xFF without allocating anything. */
    cs(&c, true); cmd_addr(&c, 0x03, 0x1000);
    ASSERT_EQ(xfer(&c, 0), 0xFF, "erased byte reads 0xFF");
    cs(&c, false);
    ASSERT(c.array == NULL, "read-only use never allocates the array");

    /* Program without WREN is ignored. */
    cs(&c, true); cmd_addr(&c, 0x02, 0x1000); xfer(&c, 0x12); cs(&c, false);
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1000), 0xFF, "PP without WEL ignored");

    /* WREN + PP 4 bytes. */
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);
    cs(&c, true); cmd_addr(&c, 0x02, 0x1000);
    xfer(&c, 0x12); xfer(&c, 0x34); xfer(&c, 0x56); xfer(&c, 0x78);
    cs(&c, false);
    ASSERT_EQ(rdsr(&c), MX25R6435F_SR_WIP, "WIP set right after PP, WEL cleared");
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1000), 0x12, "byte 0 programmed");
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1003), 0x78, "byte 3 programmed");
    mock_sim_host_advance_ns(&mock, 1000000);              /* > tPP */
    ASSERT_EQ(rdsr(&c), 0x00, "WIP clears after tPP");

    /* FAST_READ with one dummy byte. */
    cs(&c, true); cmd_addr(&c, 0x0B, 0x1001); xfer(&c, 0);
    ASSERT_EQ(xfer(&c, 0), 0x34, "FAST_READ returns programmed data");
    ASSERT_EQ(xfer(&c, 0), 0x56, "FAST_READ auto-increments");
    cs(&c, false);

    /* Program can only clear bits (AND semantics). */
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);
    cs(&c, true); cmd_addr(&c, 0x02, 0x1000); xfer(&c, 0xF0); cs(&c, false);
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1000), 0x10, "PP ANDs into existing data");

    /* Sector erase restores 0xFF, needs WREN, takes tSE. */
    cs(&c, true); cmd_addr(&c, 0x20, 0x1000); cs(&c, false);
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1000), 0x10, "SE without WEL ignored");
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);
    cs(&c, true); cmd_addr(&c, 0x20, 0x1234); cs(&c, false);   /* any addr in the 4K sector */
    ASSERT_EQ(mx25r6435f_peek(&c, 0x1000), 0xFF, "SE erased the sector");
    ASSERT_EQ(rdsr(&c), MX25R6435F_SR_WIP, "WIP set during erase");
    mock_sim_host_advance_ns(&mock, 50000000);
    ASSERT_EQ(rdsr(&c), 0x00, "WIP clears after tSE");
    ASSERT_EQ(c.stat_erases, 1, "one erase counted");
}

static void test_page_wrap(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);
    /* Start at the last byte of a page: the 2nd byte wraps to page start. */
    cs(&c, true); cmd_addr(&c, 0x02, 0x20FF); xfer(&c, 0xAA); xfer(&c, 0xBB); cs(&c, false);
    ASSERT_EQ(mx25r6435f_peek(&c, 0x20FF), 0xAA, "last page byte");
    ASSERT_EQ(mx25r6435f_peek(&c, 0x2000), 0xBB, "wrapped to page start");
    ASSERT_EQ(mx25r6435f_peek(&c, 0x2100), 0xFF, "next page untouched");
}

static void test_busy_rejects_and_unknown_ignored(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);
    cs(&c, true); cmd_addr(&c, 0x02, 0x0000); xfer(&c, 0x00); cs(&c, false);   /* busy now */
    cs(&c, true); xfer(&c, 0x06); cs(&c, false);                                /* WREN while busy: dropped */
    ASSERT_EQ(rdsr(&c) & MX25R6435F_SR_WEL, 0, "WREN ignored while WIP");
    cs(&c, true); cmd_addr(&c, 0x03, 0x0000);
    ASSERT_EQ(xfer(&c, 0), 0xFF, "READ while WIP answers 0xFF");
    cs(&c, false);
    cs(&c, true); xfer(&c, 0x9F);
    ASSERT_EQ(xfer(&c, 0), 0xC2, "RDID still answered while WIP");
    cs(&c, false);
    mock_sim_host_advance_ns(&mock, 1000000);

    /* Unknown opcode: 0xFF for the rest of the frame, no state change. */
    cs(&c, true); xfer(&c, 0x77);
    ASSERT_EQ(xfer(&c, 0), 0xFF, "unknown opcode reads 0xFF");
    ASSERT_EQ(xfer(&c, 0), 0xFF, "…and keeps reading 0xFF");
    cs(&c, false);
    ASSERT_EQ(rdsr(&c), 0x00, "status untouched by unknown opcode");

    /* Bytes while deselected are ignored. */
    ASSERT_EQ(xfer(&c, 0x9F), 0xFF, "deselected chip floats");
    cs(&c, true); xfer(&c, 0x9F);
    ASSERT_EQ(xfer(&c, 0), 0xC2, "…and starts a fresh frame on select");
    cs(&c, false);
}

static void test_rems_res_and_power_down(void) {
    mock_sim_host_t mock; mx25r6435f_t c;
    fixture(&mock, &c);
    cs(&c, true); xfer(&c, 0xAB); xfer(&c, 0); xfer(&c, 0); xfer(&c, 0);
    ASSERT_EQ(xfer(&c, 0), 0x16, "RES electronic id 0x16");
    cs(&c, false);
    cs(&c, true); cmd_addr(&c, 0x90, 0x000000);
    ASSERT_EQ(xfer(&c, 0), 0xC2, "REMS manufacturer first at ADD=0");
    ASSERT_EQ(xfer(&c, 0), 0x16, "REMS device id second");
    cs(&c, false);
    cs(&c, true); xfer(&c, 0xB9); cs(&c, false);           /* DP */
    cs(&c, true); xfer(&c, 0x9F);
    ASSERT_EQ(xfer(&c, 0), 0xFF, "no response in deep power-down");
    cs(&c, false);
    cs(&c, true); xfer(&c, 0xAB); cs(&c, false);           /* RDP */
    cs(&c, true); xfer(&c, 0x9F);
    ASSERT_EQ(xfer(&c, 0), 0xC2, "RDP wakes the chip");
    cs(&c, false);
    mx25r6435f_destroy(&c);
}

/* ====================================================================== */

int run_mx25r6435f_tests(int verbose) {
    (void)verbose;
    printf("=== MX25R6435F Mock-Host Tests ===\n");

    test_jedec_id();
    test_sfdp_header_and_tail();
    test_status_and_write_enable();
    test_program_read_erase();
    test_page_wrap();
    test_busy_rejects_and_unknown_ignored();
    test_rems_res_and_power_down();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
