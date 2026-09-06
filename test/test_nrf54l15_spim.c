/*
 * nRF54L15 SPIM model — unit tests (below L0: no CPU, no firmware).
 *
 * Programs the block exactly as nrfx_spim's blocking path does, through
 * the same nrf54l_spim_read/write entry points the MMIO dispatcher calls,
 * against a mock sim_host_t (virtual clock + event queue) and a byte
 * array standing in for Data RAM.  A recording exchange callback plays
 * the chip: it logs every MOSI byte and answers from a script.
 */
#include "nrf54l15_spim.h"
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
        printf("  FAIL: %s — got %ld, want %ld (%s:%d)\n",               \
               msg, (long)(actual), (long)(expected), __FILE__, __LINE__); } \
} while (0)

/* ---- fake Data RAM ---------------------------------------------------- */
#define RAM_BASE 0x20000000u
#define RAM_SIZE 4096u
static uint8_t ram[RAM_SIZE];

static uint8_t ram_read8(void *mem, uint32_t addr) {
    (void)mem;
    if (addr - RAM_BASE < RAM_SIZE) return ram[addr - RAM_BASE];
    return 0xEE;
}
static void ram_write8(void *mem, uint32_t addr, uint8_t v) {
    (void)mem;
    if (addr - RAM_BASE < RAM_SIZE) ram[addr - RAM_BASE] = v;
}

/* ---- recording chip --------------------------------------------------- */
static uint8_t seen[512];
static int     seen_n;
static int     seen_instance;
static uint8_t reply_base;          /* MISO = reply_base + byte index */

static uint8_t chip_exchange(void *user, int instance, uint8_t mosi) {
    (void)user;
    seen_instance = instance;
    if (seen_n < (int)sizeof(seen)) seen[seen_n] = mosi;
    return (uint8_t)(reply_base + seen_n++);
}

static int irq_count;
static void irq_hook(void *user) { (void)user; irq_count++; }

/* ---- fixture: what nrfx_spim_init + one blocking xfer does ------------- */
static void fixture(mock_sim_host_t *mock, nrf54l_spim_t *s, int instance) {
    mock_sim_host_init(mock);
    nrf54l_spim_init(s, &mock->host, instance);
    s->mem_read8     = ram_read8;
    s->mem_write8    = ram_write8;
    s->exchange      = chip_exchange;
    s->irq           = irq_hook;
    memset(ram, 0, sizeof(ram));
    seen_n = 0; irq_count = 0; reply_base = 0x80;
}

/* nrfy_spim_periph_configure: PSEL, ORC, PRESCALER, CONFIG, RXDELAY. */
static void driver_init(nrf54l_spim_t *s, uint32_t prescaler, uint32_t orc) {
    nrf54l_spim_write(s, SPIM_PSEL_SCK,  (2u << 5) | 1u);   /* P2.01 */
    nrf54l_spim_write(s, SPIM_PSEL_MOSI, (2u << 5) | 2u);   /* P2.02 */
    nrf54l_spim_write(s, SPIM_PSEL_MISO, (2u << 5) | 4u);   /* P2.04 */
    nrf54l_spim_write(s, SPIM_PSEL_CSN,  SPIM_PSEL_DISCONNECTED);
    nrf54l_spim_write(s, SPIM_ORC,       orc);
    nrf54l_spim_write(s, SPIM_PRESCALER, prescaler);
    nrf54l_spim_write(s, SPIM_CONFIG,    0);                 /* MSB first, mode 0 */
    nrf54l_spim_write(s, SPIM_IFTIMING_RXDELAY, 2);
}

/* spim_xfer(): buffers, clear END, ENABLE, START, poll END, then
 * spim_abort(): STOP, poll STOPPED, ENABLE=0.  Returns the number of
 * mock clock advances of `step_ns` it took for END to appear (so a test
 * can assert the transfer was neither instant nor late). */
static int driver_xfer(mock_sim_host_t *mock, nrf54l_spim_t *s,
                       uint32_t tx_addr, uint32_t tx_len,
                       uint32_t rx_addr, uint32_t rx_len, int64_t step_ns) {
    nrf54l_spim_write(s, SPIM_DMA_TX_PTR,    tx_addr);
    nrf54l_spim_write(s, SPIM_DMA_TX_MAXCNT, tx_len);
    nrf54l_spim_write(s, SPIM_DMA_RX_PTR,    rx_addr);
    nrf54l_spim_write(s, SPIM_DMA_RX_MAXCNT, rx_len);
    nrf54l_spim_write(s, SPIM_EVENTS_END, 0);
    nrf54l_spim_write(s, SPIM_ENABLE, SPIM_ENABLE_ENABLED);
    nrf54l_spim_write(s, SPIM_TASKS_START, 1);
    int polls = 0;
    while (!nrf54l_spim_read(s, SPIM_EVENTS_END)) {
        mock_sim_host_advance_ns(mock, step_ns);
        if (++polls > 100000) return -1;
    }
    nrf54l_spim_write(s, SPIM_EVENTS_END, 0);
    nrf54l_spim_write(s, SPIM_TASKS_STOP, 1);
    int stop_polls = 0;
    while (!nrf54l_spim_read(s, SPIM_EVENTS_STOPPED)) {
        mock_sim_host_advance_ns(mock, 1000);
        if (++stop_polls > 100) return -2;
    }
    nrf54l_spim_write(s, SPIM_EVENTS_STOPPED, 0);
    nrf54l_spim_write(s, SPIM_ENABLE, 0);
    return polls;
}

/* ====================================================================== */

static void test_reset_state(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_ENABLE), 0, "ENABLE resets to 0");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_PRESCALER), 0x40, "PRESCALER resets to 0x40");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_PSEL_SCK), SPIM_PSEL_DISCONNECTED, "PSEL.SCK disconnected");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_PSEL_CSN), SPIM_PSEL_DISCONNECTED, "PSEL.CSN disconnected");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_END), 0, "EVENTS_END clear");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_TX_AMOUNT), 0, "TX.AMOUNT 0");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_IFTIMING_RXDELAY), 2, "RXDELAY reset 2");
}

static void test_bit_rate_from_prescaler(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    nrf54l_spim_write(&s, SPIM_PRESCALER, 16);
    ASSERT_EQ(nrf54l_spim_bit_rate_hz(&s), 8000000, "SPIM00 /16 = 8 MHz");
    nrf54l_spim_write(&s, SPIM_PRESCALER, 4);
    ASSERT_EQ(nrf54l_spim_bit_rate_hz(&s), 32000000, "SPIM00 /4 = 32 MHz (max)");
    nrf54l_spim_write(&s, SPIM_PRESCALER, 2);
    ASSERT_EQ(nrf54l_spim_bit_rate_hz(&s), 32000000, "SPIM00 clamps below divisor 4");
    ASSERT_EQ(nrf54l_spim_transfer_ns(&s, 4), 1000, "4 bytes at 32 MHz = 1 µs");

    fixture(&mock, &s, 22);
    nrf54l_spim_write(&s, SPIM_PRESCALER, 4);
    ASSERT_EQ(nrf54l_spim_bit_rate_hz(&s), 4000000, "SPIM22 /4 = 4 MHz");
    nrf54l_spim_write(&s, SPIM_PRESCALER, 0);
    ASSERT_EQ(nrf54l_spim_bit_rate_hz(&s), 8000000, "SPIM22 divisor 0 clamps to 2 → 8 MHz");
    ASSERT_EQ(nrf54l_spim_transfer_ns(&s, 256), 256000, "256 bytes at 8 MHz = 256 µs");
}

static void test_start_ignored_while_disabled(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);
    ram[0] = 0x9F;
    nrf54l_spim_write(&s, SPIM_DMA_TX_PTR, RAM_BASE);
    nrf54l_spim_write(&s, SPIM_DMA_TX_MAXCNT, 1);
    nrf54l_spim_write(&s, SPIM_TASKS_START, 1);        /* ENABLE still 0 */
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_STARTED), 0, "no STARTED while disabled");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_END), 0, "no END while disabled");
    ASSERT_EQ(mock.schedule_calls, 0, "nothing scheduled while disabled");
    ASSERT_EQ(seen_n, 0, "no bytes on the wire while disabled");
}

static void test_tx_only_transfer(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);                            /* 8 MHz */
    ram[0] = 0x06; ram[1] = 0x12; ram[2] = 0x34;
    int polls = driver_xfer(&mock, &s, RAM_BASE, 3, 0, 0, 500);
    ASSERT(polls > 0, "END arrived");
    /* 3 bytes * 8 bits / 8 MHz = 3 µs → 6 polls of 500 ns, not 1 */
    ASSERT_EQ(polls, 6, "3 bytes at 8 MHz take 3 µs of sim time");
    ASSERT_EQ(seen_n, 3, "three bytes clocked");
    ASSERT(seen[0] == 0x06 && seen[1] == 0x12 && seen[2] == 0x34, "MOSI stream is the TX buffer");
    ASSERT_EQ(seen_instance, 0, "exchange tagged with the instance id");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_TX_AMOUNT), 3, "TX.AMOUNT = 3");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_RX_AMOUNT), 0, "RX.AMOUNT = 0");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_DMA_TX_END), 1, "DMA.TX.END set");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_DMA_RX_END), 1, "DMA.RX.END set");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_ENABLE), 0, "driver disabled the block after abort");
}

static void test_rx_longer_than_tx_pads_with_orc(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0xA5);
    reply_base = 0xC2;
    ram[0] = 0x9F;                                     /* JEDEC ID opcode */
    driver_xfer(&mock, &s, RAM_BASE, 1, RAM_BASE + 16, 3, 250);
    ASSERT_EQ(seen_n, 3, "max(TX=1, RX=3) = 3 bytes clocked");
    ASSERT(seen[0] == 0x9F && seen[1] == 0xA5 && seen[2] == 0xA5, "ORC pads after TX runs out");
    ASSERT(ram[16] == 0xC2 && ram[17] == 0xC3 && ram[18] == 0xC4, "MISO stream landed in RX buffer");
    ASSERT_EQ(ram[19], 0, "nothing written past RX.MAXCNT");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_TX_AMOUNT), 1, "TX.AMOUNT = 1");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_RX_AMOUNT), 3, "RX.AMOUNT = 3");
}

static void test_tx_longer_than_rx_drops_extra_miso(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);
    for (int i = 0; i < 5; i++) ram[i] = (uint8_t)(0x10 + i);
    driver_xfer(&mock, &s, RAM_BASE, 5, RAM_BASE + 32, 2, 250);
    ASSERT_EQ(seen_n, 5, "5 bytes clocked");
    ASSERT_EQ(ram[32], 0x80, "first MISO byte stored");
    ASSERT_EQ(ram[33], 0x81, "second MISO byte stored");
    ASSERT_EQ(ram[34], 0, "third MISO byte dropped (RX.MAXCNT=2)");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_RX_AMOUNT), 2, "RX.AMOUNT = 2");
}

static void test_no_chip_reads_ff(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 22);
    s.exchange = NULL;                                 /* nothing selected on the bus */
    driver_init(&s, 4, 0);
    ram[0] = 0x9F;
    driver_xfer(&mock, &s, RAM_BASE, 1, RAM_BASE + 8, 3, 250);
    ASSERT(ram[8] == 0xFF && ram[9] == 0xFF && ram[10] == 0xFF, "MISO floats high with no chip");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_RX_AMOUNT), 3, "transfer still completes");
}

static void test_staged_256_byte_transfer(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);
    /* spi-arch.c transfer_staged: 64-byte chunks, TX beyond wlen is 0. */
    ram[0] = 0x5A;                                     /* SFDP opcode + 3 addr + 1 dummy */
    for (int chunk = 0; chunk < 4; chunk++) {
        memset(ram + 64, 0, 64);
        if (chunk == 0) ram[64] = 0x5A;
        driver_xfer(&mock, &s, RAM_BASE + 64, 64, RAM_BASE + 256, 64, 1000);
    }
    ASSERT_EQ(seen_n, 256, "256 bytes clocked over 4 chunks");
    ASSERT_EQ(seen[0], 0x5A, "first byte is the opcode");
    ASSERT_EQ(seen[255], 0x00, "staging pads with zeros");
    ASSERT_EQ(s.stat_transfers, 4, "4 separate START..END transactions");
    ASSERT_EQ(ram[256 + 63], (uint8_t)(0x80 + 255), "last chunk's last MISO byte stored");
}

static void test_stop_raises_stopped(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);
    nrf54l_spim_write(&s, SPIM_ENABLE, SPIM_ENABLE_ENABLED);
    nrf54l_spim_write(&s, SPIM_TASKS_STOP, 1);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_STOPPED), 1, "STOPPED fires with nothing in flight");
    nrf54l_spim_write(&s, SPIM_EVENTS_STOPPED, 0);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_STOPPED), 0, "STOPPED cleared by writing 0");

    /* STOP mid-transfer: model completes the bytes, then STOPPED. */
    ram[0] = 0xAA; ram[1] = 0xBB;
    nrf54l_spim_write(&s, SPIM_DMA_TX_PTR, RAM_BASE);
    nrf54l_spim_write(&s, SPIM_DMA_TX_MAXCNT, 2);
    nrf54l_spim_write(&s, SPIM_DMA_RX_MAXCNT, 0);
    nrf54l_spim_write(&s, SPIM_TASKS_START, 1);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_STARTED), 1, "STARTED at START");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_END), 0, "END not yet (async)");
    nrf54l_spim_write(&s, SPIM_TASKS_STOP, 1);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_STOPPED), 1, "STOPPED after STOP mid-transfer");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_EVENTS_END), 1, "END raised by the forced completion");
    ASSERT_EQ(seen_n, 2, "both bytes clocked");
    ASSERT_EQ(mock.cancel_calls, 1, "pending completion event cancelled");
    mock_sim_host_advance_ns(&mock, 1000000);
    ASSERT_EQ(s.stat_transfers, 1, "no double completion when the event would have fired");
}

static void test_interrupt_masks(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    driver_init(&s, 16, 0);
    nrf54l_spim_write(&s, SPIM_INTENSET, SPIM_INT_END);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_INTENSET), SPIM_INT_END, "INTENSET readback");
    ram[0] = 0x05;
    driver_xfer(&mock, &s, RAM_BASE, 1, 0, 0, 250);
    ASSERT_EQ(irq_count, 1, "END interrupt raised once");
    nrf54l_spim_write(&s, SPIM_INTENCLR, SPIM_INT_END);
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_INTENSET), 0, "INTENCLR clears");
    driver_xfer(&mock, &s, RAM_BASE, 1, 0, 0, 250);
    ASSERT_EQ(irq_count, 1, "no interrupt once masked");
}

/* The bit-rate probe in examples/platform-specific/nrf/spi-flash re-inits
 * the SPIM seven times with a different PRESCALER each; each cycle must
 * produce a correct 1+3 byte JEDEC exchange and honour the new rate. */
static void test_consecutive_reconfiguration(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    /* spi-arch.c resolve_bit_rate() for 1,2,4,8,16,20,32 MHz on SPIM00:
     * divisor rounded up to even, clamped to the 4..126 DIVISOR range —
     * so "1 MHz" is really 126 (1.016 MHz), and 20 MHz becomes 8. */
    static const uint32_t presc[] = { 126, 64, 32, 16, 8, 8, 4 };
    for (unsigned i = 0; i < sizeof(presc) / sizeof(presc[0]); i++) {
        seen_n = 0; reply_base = 0xC2;
        driver_init(&s, presc[i], 0);
        ram[0] = 0x9F;
        memset(ram + 8, 0, 3);
        int64_t t0 = mock.now_ns;
        int polls = driver_xfer(&mock, &s, RAM_BASE, 1, RAM_BASE + 8, 3, 100);
        int64_t took = mock.now_ns - t0;              /* STOPPED is synchronous: no extra polls */
        int64_t want = (int64_t)(3 * 8) * 1000000000LL / (128000000 / presc[i]);
        ASSERT(polls > 0, "END arrived after reconfiguration");
        ASSERT(seen_n == 3 && seen[0] == 0x9F, "opcode + 2 pads clocked");
        ASSERT(ram[8] == 0xC2 && ram[9] == 0xC3 && ram[10] == 0xC4, "JEDEC bytes stored");
        /* within one poll step of the ideal wire time */
        ASSERT(took >= want && took < want + 200, "transfer time tracks the new PRESCALER");
        nrf54l_spim_write(&s, SPIM_INTENCLR, 0xFFFFFFFFu);   /* nrfx_spim_uninit */
    }
}

static void test_unknown_offsets_are_inert(void) {
    mock_sim_host_t mock; nrf54l_spim_t s;
    fixture(&mock, &s, 0);
    nrf54l_spim_write(&s, 0xC80, 0x82);                 /* nRF54L errata 55 poke */
    nrf54l_spim_write(&s, 0xC84, 0x82);                 /* errata 8 poke */
    nrf54l_spim_write(&s, SPIM_DMA_TX_AMOUNT, 99);      /* read-only */
    ASSERT_EQ(nrf54l_spim_read(&s, 0xC80), 0, "errata scratch reads 0");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_DMA_TX_AMOUNT), 0, "AMOUNT is read-only");
    ASSERT_EQ(nrf54l_spim_read(&s, SPIM_TASKS_START), 0, "tasks read as 0");
}

/* ====================================================================== */

int run_nrf54l15_spim_tests(int verbose) {
    (void)verbose;
    printf("=== nRF54L15 SPIM Model Tests ===\n");

    test_reset_state();
    test_bit_rate_from_prescaler();
    test_start_ignored_while_disabled();
    test_tx_only_transfer();
    test_rx_longer_than_tx_pads_with_orc();
    test_tx_longer_than_rx_drops_extra_miso();
    test_no_chip_reads_ff();
    test_staged_256_byte_transfer();
    test_stop_raises_stopped();
    test_interrupt_masks();
    test_consecutive_reconfiguration();
    test_unknown_offsets_are_inert();

    printf("\n--- Results: %d passed, %d failed ---\n\n", passed, failed);
    return failed;
}
