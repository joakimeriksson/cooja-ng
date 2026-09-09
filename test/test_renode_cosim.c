/*
 * Renode co-simulation unit tests (csim as clock slave).
 *
 * Three layers, each testable without a CPU or a real Renode:
 *   1. the 24-byte wire codec           (renode_proto.c)
 *   2. the register window / FIFOs      (renode_dev.c)
 *   3. the protocol + horizon loop      (renode_cosim_service.c) driven by
 *      a scripted mock master over a socketpair
 *
 * Design: docs/design/renode-cosim-plan.md.
 */
#include "renode_proto.h"
#include "renode_dev.h"
#include "renode_cosim_service.h"
#include "sim_runtime.h"
#include "sim_mote.h"

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

static int passed = 0;
static int failed = 0;

#define ASSERT(cond, msg) do {                                          \
    if (cond) { passed++; }                                             \
    else { failed++; printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

#define ASSERT_EQ(actual, expected, msg) do {                            \
    long long _a = (long long)(actual), _e = (long long)(expected);      \
    if (_a == _e) { passed++; }                                          \
    else { failed++;                                                     \
        printf("  FAIL: %s — got %lld, want %lld (%s:%d)\n",             \
               msg, _a, _e, __FILE__, __LINE__); }                       \
} while (0)

/* ============================================================
 * 1. Wire codec
 * ============================================================ */

static void test_proto(int verbose) {
    if (verbose) printf("  -- protocol codec --\n");

    /* The layout is the contract with Renode: 4/8/8/4 little-endian, no
     * padding.  Pin it against a hand-written byte vector rather than
     * against our own encoder, so a change of layout cannot pass by
     * agreeing with itself. */
    renode_msg_t m = { .action = RENODE_TICK_CLOCK, .addr = 0x1122334455667788ULL,
                       .value = 0x00000000000003E8ULL,
                       .peripheral_index = RENODE_NO_PERIPHERAL_INDEX };
    uint8_t buf[RENODE_MSG_SIZE];
    memset(buf, 0xAA, sizeof(buf));
    renode_msg_encode(&m, buf);

    static const uint8_t want[RENODE_MSG_SIZE] = {
        0x01, 0x00, 0x00, 0x00,                            /* action = 1   */
        0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,    /* addr         */
        0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* value = 1000 */
        0xFF, 0xFF, 0xFF, 0xFF,                            /* index = -1   */
    };
    ASSERT(memcmp(buf, want, sizeof(want)) == 0, "tickClock encodes to the exact wire bytes");
    ASSERT_EQ(sizeof(want), 24, "message is 24 bytes");

    renode_msg_t back;
    memset(&back, 0, sizeof(back));
    renode_msg_decode(want, &back);
    ASSERT_EQ(back.action, RENODE_TICK_CLOCK, "decode action");
    ASSERT(back.addr == 0x1122334455667788ULL, "decode addr");
    ASSERT_EQ(back.value, 1000, "decode value");
    ASSERT_EQ(back.peripheral_index, -1, "decode peripheral index");

    /* Roundtrip a few messages, including a negative index and the high bit
     * of value set (a 64-bit sim time). */
    const renode_msg_t cases[] = {
        { RENODE_HANDSHAKE, 0, 0, -1 },
        { RENODE_READ_QWORD, 0x64, 0, 0 },
        { RENODE_WRITE_BYTE, 0x1C, 0xFF, 3 },
        { RENODE_OK, 0, 0x8000000000000001ULL, -1 },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint8_t b[RENODE_MSG_SIZE];
        renode_msg_t out;
        renode_msg_encode(&cases[i], b);
        renode_msg_decode(b, &out);
        ASSERT(out.action == cases[i].action && out.addr == cases[i].addr &&
               out.value == cases[i].value &&
               out.peripheral_index == cases[i].peripheral_index,
               "codec roundtrip");
    }

    /* Widths and direction: what the service switches on. */
    ASSERT_EQ(renode_action_width(RENODE_READ_BYTE), 1, "readByte width");
    ASSERT_EQ(renode_action_width(RENODE_WRITE_WORD), 2, "writeWord width");
    ASSERT_EQ(renode_action_width(RENODE_READ_DWORD), 4, "readDword width");
    ASSERT_EQ(renode_action_width(RENODE_WRITE_QWORD), 8, "writeQword width");
    ASSERT_EQ(renode_action_width(RENODE_TICK_CLOCK), 0, "tickClock has no width");
    ASSERT(renode_action_is_read(RENODE_READ_WORD), "readWord is a read");
    ASSERT(!renode_action_is_read(RENODE_WRITE_WORD), "writeWord is not a read");
    ASSERT(renode_action_is_write(RENODE_WRITE_QWORD), "writeQword is a write");
    ASSERT(!renode_action_is_write(RENODE_INTERRUPT), "interrupt is not a write");

    ASSERT(strcmp(renode_action_name(RENODE_TICK_CLOCK), "tickClock") == 0,
           "action name matches Renode's spelling");
    ASSERT(strcmp(renode_action_name(12345), "unknown") == 0,
           "unmodelled action names as unknown");
}

/* ============================================================
 * 2. Device model
 * ============================================================ */

/* Hook capture */
static uint8_t  cap_tx[RENODE_DEV_MAX_FRAME];
static int      cap_tx_len;
static int      cap_tx_calls;
static int      cap_channel;
static int      cap_power;
static int64_t  cap_now;
static int      cap_uart_slot;
static uint8_t  cap_uart[64];
static int      cap_uart_len;
static int      cap_uart_accept;   /* how many bytes uart_inject takes */

static void hook_tx(void *u, const uint8_t *f, int len) {
    (void)u;
    if (len > (int)sizeof(cap_tx)) len = (int)sizeof(cap_tx);
    memcpy(cap_tx, f, (size_t)len);
    cap_tx_len = len;
    cap_tx_calls++;
}
static void hook_set_channel(void *u, int ch) { (void)u; cap_channel = ch; }
static void hook_set_power(void *u, int p) { (void)u; cap_power = p; }
static int64_t hook_now(void *u) { (void)u; return cap_now; }
static int hook_uart_inject(void *u, int slot, const uint8_t *b, int n) {
    (void)u;
    cap_uart_slot = slot;
    int take = (cap_uart_accept < n) ? cap_uart_accept : n;
    if (take > (int)sizeof(cap_uart)) take = (int)sizeof(cap_uart);
    memcpy(cap_uart, b, (size_t)take);
    cap_uart_len = take;
    return take;
}

static void dev_setup(renode_dev_t *d) {
    renode_dev_init(d);
    d->now_ns      = hook_now;
    d->tx          = hook_tx;
    d->set_channel = hook_set_channel;
    d->set_power   = hook_set_power;
    d->uart_inject = hook_uart_inject;
    d->user        = NULL;
    d->self_id     = 3;
    d->node_count  = 4;
    cap_tx_len = cap_tx_calls = 0;
    cap_channel = cap_power = -999;
    cap_now = 0;
    cap_uart_slot = -1; cap_uart_len = 0; cap_uart_accept = 64;
}

static void test_dev_identity(int verbose) {
    if (verbose) printf("  -- device: identity + defaults --\n");
    renode_dev_t d;
    dev_setup(&d);

    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_ID, 4), 0x4353494DU, "ID reads CSIM");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_VERSION, 4), 1, "version is 1");
    ASSERT_EQ((int32_t)renode_dev_read(&d, RENODE_REG_CHANNEL, 4), -1,
              "channel defaults to any");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_TXPOWER, 4), 31, "txpower defaults to max");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_SELF_ID, 4), 3, "self id");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_NODE_COUNT, 4), 4, "node count");

    /* A narrow read sees the low bytes of the register. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_ID, 1), 0x4D, "byte read of ID");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_ID, 2), 0x494D, "word read of ID");

    /* Sim time is read through the hook, in two halves. */
    cap_now = 0x0000000123456789LL;
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_SIM_TIME_LO, 4), 0x23456789U, "sim time lo");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_SIM_TIME_HI, 4), 0x1U, "sim time hi");

    /* Unmapped offsets read zero rather than faulting. */
    ASSERT_EQ(renode_dev_read(&d, 0xF0, 4), 0, "unmapped offset reads zero");
}

static void test_dev_tx(int verbose) {
    if (verbose) printf("  -- device: transmit path --\n");
    renode_dev_t d;
    dev_setup(&d);

    static const uint8_t frame[] = { 0x41, 0x88, 0x2A, 0xCD, 0xAB, 0xFF, 0xFF };
    renode_dev_write(&d, RENODE_REG_TX_LEN, 4, sizeof(frame));
    for (unsigned i = 0; i < sizeof(frame); i++)
        renode_dev_write(&d, RENODE_REG_TX_DATA, 1, frame[i]);
    ASSERT_EQ(cap_tx_calls, 0, "no frame on the air before TX_CTRL");
    renode_dev_write(&d, RENODE_REG_TX_CTRL, 4, 1);
    ASSERT_EQ(cap_tx_calls, 1, "TX_CTRL transmits once");
    ASSERT_EQ(cap_tx_len, (int)sizeof(frame), "transmitted length");
    ASSERT(memcmp(cap_tx, frame, sizeof(frame)) == 0, "transmitted bytes are exact");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_TX_COUNT, 4), 1, "TX_COUNT");

    /* Wider appends pack little-endian, so a guest can push 4 bytes a time. */
    dev_setup(&d);
    renode_dev_write(&d, RENODE_REG_TX_LEN, 4, 8);
    renode_dev_write(&d, RENODE_REG_TX_DATA, 4, 0x44332211U);
    renode_dev_write(&d, RENODE_REG_TX_DATA, 4, 0x88776655U);
    renode_dev_write(&d, RENODE_REG_TX_CTRL, 4, 1);
    static const uint8_t want[] = { 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88 };
    ASSERT(cap_tx_len == 8 && memcmp(cap_tx, want, 8) == 0,
           "dword appends are little-endian");

    /* Guest driver bugs must not put a malformed frame on the air. */
    dev_setup(&d);
    renode_dev_write(&d, RENODE_REG_TX_CTRL, 4, 1);
    ASSERT_EQ(cap_tx_calls, 0, "TX_CTRL with no length transmits nothing");

    renode_dev_write(&d, RENODE_REG_TX_LEN, 4, 4);
    renode_dev_write(&d, RENODE_REG_TX_DATA, 1, 0xAA);
    renode_dev_write(&d, RENODE_REG_TX_CTRL, 4, 1);
    ASSERT_EQ(cap_tx_calls, 0, "short buffer transmits nothing");

    renode_dev_write(&d, RENODE_REG_TX_LEN, 4, RENODE_DEV_MAX_FRAME + 1);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_TX_LEN, 4), 4,
              "over-long TX_LEN is ignored");

    /* Extra bytes past TX_LEN are dropped, not written out of bounds. */
    dev_setup(&d);
    renode_dev_write(&d, RENODE_REG_TX_LEN, 4, 2);
    for (int i = 0; i < 10; i++)
        renode_dev_write(&d, RENODE_REG_TX_DATA, 1, (uint64_t)(0x10 + i));
    renode_dev_write(&d, RENODE_REG_TX_CTRL, 4, 1);
    ASSERT_EQ(cap_tx_len, 2, "appends stop at TX_LEN");
    ASSERT(cap_tx[0] == 0x10 && cap_tx[1] == 0x11, "first bytes kept");
}

static void test_dev_rx(int verbose) {
    if (verbose) printf("  -- device: receive path --\n");
    renode_dev_t d;
    dev_setup(&d);

    static const uint8_t f1[] = { 0x01, 0x02, 0x03 };
    static const uint8_t f2[] = { 0xAA, 0xBB };
    ASSERT_EQ(renode_dev_rx_push(&d, f1, sizeof(f1), 1000, 1, 26, -65), 0, "push 1");
    ASSERT_EQ(renode_dev_rx_push(&d, f2, sizeof(f2), 2000, 2, 11, -80), 0, "push 2");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 2, "two queued");
    ASSERT(renode_dev_read(&d, RENODE_REG_STATUS, 4) & RENODE_STATUS_RX_AVAIL,
           "STATUS.RX_AVAIL set");

    /* The head's metadata, then its bytes, then pop and repeat: the order a
     * guest ISR reads them in. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_LEN, 4), 3, "head length");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_FROM, 4), 1, "head sender");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_CHANNEL, 4), 26, "head channel");
    ASSERT_EQ((int32_t)renode_dev_read(&d, RENODE_REG_RX_RSSI, 4), -65,
              "head RSSI is sign-extended");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_TIME_LO, 4), 1000, "head on-air start");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DATA, 1), 0x01, "byte 0");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DATA, 1), 0x02, "byte 1");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DATA, 1), 0x03, "byte 2");
    /* Past the end reads zero rather than the next frame's bytes. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DATA, 1), 0x00, "padding past the end");

    renode_dev_write(&d, RENODE_REG_RX_POP, 4, 1);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 1, "one left after pop");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_LEN, 4), 2, "second frame is head");
    /* A wide read streams the frame zero-padded, which is how a guest can
     * pull a frame in dwords without tracking the tail. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DATA, 4), 0x0000BBAAU,
              "dword read is LE and zero-padded");
    renode_dev_write(&d, RENODE_REG_RX_POP, 4, 1);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 0, "queue drained");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_LEN, 4), 0, "empty head length");
    ASSERT_EQ((int32_t)renode_dev_read(&d, RENODE_REG_RX_FROM, 4), -1,
              "empty head sender is -1");
    renode_dev_write(&d, RENODE_REG_RX_POP, 4, 1);  /* pop on empty: no crash */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 0, "pop on empty is a no-op");

    /* Overflow: the queue is full at RENODE_DEV_RX_QUEUE, the next frame is
     * dropped and counted, and the sticky bit survives until RX_FLUSH. */
    dev_setup(&d);
    for (int i = 0; i < RENODE_DEV_RX_QUEUE; i++)
        ASSERT_EQ(renode_dev_rx_push(&d, f1, sizeof(f1), 0, 1, 26, -60), 0, "fill queue");
    ASSERT_EQ(renode_dev_rx_push(&d, f1, sizeof(f1), 0, 1, 26, -60), -1,
              "push past the depth fails");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DROPPED, 4), 1, "drop counted");
    ASSERT(renode_dev_read(&d, RENODE_REG_STATUS, 4) & RENODE_STATUS_RX_OVERFLOW,
           "overflow bit sticky");
    renode_dev_write(&d, RENODE_REG_CTRL, 4, RENODE_CTRL_RX_FLUSH);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 0, "RX_FLUSH empties the queue");
    ASSERT(!(renode_dev_read(&d, RENODE_REG_STATUS, 4) & RENODE_STATUS_RX_OVERFLOW),
           "RX_FLUSH clears the overflow bit");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_DROPPED, 4), 1,
              "the drop counter is not cleared by a flush");

    /* An over-long frame is truncated to the cap, never written past it. */
    dev_setup(&d);
    uint8_t big[RENODE_DEV_MAX_FRAME + 32];
    memset(big, 0x5A, sizeof(big));
    ASSERT_EQ(renode_dev_rx_push(&d, big, (int)sizeof(big), 0, 1, 26, -60), 0,
              "over-long frame accepted");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_LEN, 4), RENODE_DEV_MAX_FRAME,
              "over-long frame truncated to the cap");
}

static void test_dev_irq(int verbose) {
    if (verbose) printf("  -- device: interrupt level --\n");
    renode_dev_t d;
    dev_setup(&d);
    static const uint8_t f[] = { 0x01 };

    ASSERT(!renode_dev_irq_level(&d), "idle: line low");
    renode_dev_rx_push(&d, f, 1, 0, 1, 26, -60);
    ASSERT(!renode_dev_irq_level(&d), "a frame with the source masked keeps it low");
    renode_dev_write(&d, RENODE_REG_CTRL, 4, RENODE_CTRL_IRQ_RX_EN);
    ASSERT(renode_dev_irq_level(&d), "unmasking with a frame queued raises it");
    ASSERT(renode_dev_read(&d, RENODE_REG_IRQ_STATUS, 4) & RENODE_IRQ_SRC_RX,
           "IRQ_STATUS shows the RX source");
    renode_dev_write(&d, RENODE_REG_RX_POP, 4, 1);
    ASSERT(!renode_dev_irq_level(&d), "draining the queue lowers it");

    /* The UART source is independent and separately masked. */
    renode_dev_uart_push(&d, 'x');
    ASSERT(!renode_dev_irq_level(&d), "console byte with UART masked keeps it low");
    renode_dev_write(&d, RENODE_REG_CTRL, 4,
                     RENODE_CTRL_IRQ_RX_EN | RENODE_CTRL_IRQ_UART_EN);
    ASSERT(renode_dev_irq_level(&d), "unmasking UART raises it");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_DATA, 4), 'x', "console byte read");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_DATA, 4), 0xFFFFFFFFU,
              "empty console reads the sentinel");
    ASSERT(!renode_dev_irq_level(&d), "draining the console lowers it");

    /* CTRL's write-1 bits are actions, not state. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_CTRL, 4),
              RENODE_CTRL_IRQ_RX_EN | RENODE_CTRL_IRQ_UART_EN,
              "CTRL reads back only the persistent bits");
}

static void test_dev_uart(int verbose) {
    if (verbose) printf("  -- device: console bridge --\n");
    renode_dev_t d;
    dev_setup(&d);

    /* csim -> Renode, in order. */
    const char *line = "hi\n";
    for (const char *p = line; *p; p++)
        renode_dev_uart_push(&d, (uint8_t)*p);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_COUNT, 4), 3, "three bytes queued");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_DATA, 4), 'h', "byte order 1");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_DATA, 4), 'i', "byte order 2");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_DATA, 4), '\n', "byte order 3");

    /* Ring overflow drops and counts rather than corrupting. */
    for (int i = 0; i < RENODE_DEV_UART_RING + 10; i++)
        renode_dev_uart_push(&d, (uint8_t)i);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_COUNT, 4), RENODE_DEV_UART_RING,
              "ring caps at its depth");
    ASSERT_EQ(d.uart_dropped, 10, "dropped bytes counted");
    ASSERT(renode_dev_read(&d, RENODE_REG_STATUS, 4) & RENODE_STATUS_UART_OVERFLOW,
           "console overflow bit set");

    /* Renode -> csim: buffered on write, delivered at the quantum boundary,
     * and a node that takes only part keeps the rest for next time. */
    dev_setup(&d);
    renode_dev_write(&d, RENODE_REG_UART_NODE, 4, 2);
    renode_dev_write(&d, RENODE_REG_UART_DATA, 4, 'a');
    renode_dev_write(&d, RENODE_REG_UART_DATA, 4, 'b');
    renode_dev_write(&d, RENODE_REG_UART_DATA, 4, 'c');
    ASSERT(renode_dev_read(&d, RENODE_REG_STATUS, 4) & RENODE_STATUS_UART_IN_PENDING,
           "pending input flagged");
    ASSERT_EQ(cap_uart_len, 0, "nothing injected during the bus write");
    cap_uart_accept = 2;
    renode_dev_uart_retry(&d);
    ASSERT_EQ(cap_uart_slot, 2, "injected into the selected node");
    ASSERT_EQ(cap_uart_len, 2, "node took two bytes");
    ASSERT_EQ(d.ui_count, 1, "the third byte is kept");
    cap_uart_accept = 64;
    renode_dev_uart_retry(&d);
    ASSERT_EQ(d.ui_count, 0, "retry drains the rest");
    ASSERT_EQ(cap_uart[0], 'c', "the kept byte is the one that was not taken");
}

static void test_dev_reset(int verbose) {
    if (verbose) printf("  -- device: reset --\n");
    renode_dev_t d;
    dev_setup(&d);
    static const uint8_t f[] = { 0x01, 0x02 };

    renode_dev_write(&d, RENODE_REG_CTRL, 4, RENODE_CTRL_IRQ_RX_EN);
    renode_dev_write(&d, RENODE_REG_CHANNEL, 4, 26);
    renode_dev_rx_push(&d, f, 2, 0, 1, 26, -60);
    renode_dev_uart_push(&d, 'z');

    renode_dev_reset(&d);
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_RX_COUNT, 4), 0, "reset clears RX");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_UART_COUNT, 4), 0, "reset clears console");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_CTRL, 4), 0, "reset clears CTRL");
    ASSERT_EQ((int32_t)renode_dev_read(&d, RENODE_REG_CHANNEL, 4), -1,
              "reset restores the channel default");
    ASSERT(!renode_dev_irq_level(&d), "reset lowers the interrupt line");
    /* Identity survives: a machine reset in Renode does not renumber csim. */
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_SELF_ID, 4), 3, "reset keeps the node id");
    ASSERT(d.tx == hook_tx, "reset keeps the hooks");
}

static void test_dev_channel_power(int verbose) {
    if (verbose) printf("  -- device: channel + power push --\n");
    renode_dev_t d;
    dev_setup(&d);

    renode_dev_write(&d, RENODE_REG_CHANNEL, 4, 26);
    ASSERT_EQ(cap_channel, 26, "channel pushed to the medium");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_CHANNEL, 4), 26, "channel reads back");

    /* An out-of-band channel would silently mute the node in the medium, so
     * it is refused rather than pushed. */
    cap_channel = -999;
    renode_dev_write(&d, RENODE_REG_CHANNEL, 4, 99);
    ASSERT_EQ(cap_channel, -999, "out-of-band channel is not pushed");
    ASSERT_EQ(renode_dev_read(&d, RENODE_REG_CHANNEL, 4), 26, "channel unchanged");

    renode_dev_write(&d, RENODE_REG_TXPOWER, 4, 7);
    ASSERT_EQ(cap_power, 7, "power pushed to the medium");
    renode_dev_write(&d, RENODE_REG_TXPOWER, 4, 0xFF);
    ASSERT_EQ(cap_power, 31, "power is masked to the 5-bit PA scale");
}

/* ============================================================
 * 3. Service: protocol loop + horizon, driven by a mock master
 *
 * The master side of a socketpair plays Renode.  This exercises the real
 * service — the same code the runner calls — so what is under test is the
 * reply order, the tick-to-nanosecond arithmetic and the async plane, not a
 * paraphrase of them.
 * ============================================================ */

/* A stub mote that carries a device, so the service's discovery path (via
 * get_interface, the only way it can reach one) is what runs. */
static renode_dev_t stub_dev;
static sim_runtime_t *stub_sim;

/* The device reads simulation time through this, exactly as the real mote
 * does — so a test can check WHICH time a bus access observes. */
static int64_t stub_dev_now(void *user) {
    (void)user;
    return stub_sim ? sim_runtime_now_ns(stub_sim) : 0;
}

static void *stub_get_interface(sim_mote_t *m, int iface) {
    (void)m;
    return (iface == SIM_MOTE_IFACE_COSIM_DEV) ? &stub_dev : NULL;
}
static int64_t stub_execute(sim_mote_t *m, int64_t now_ns) {
    (void)m; (void)now_ns; return INT64_MAX;
}
static const sim_mote_ops_t stub_ops = {
    .kind = "RENODE-STUB",
    .execute = stub_execute,
    .get_interface = stub_get_interface,
};

typedef struct {
    int master_main, master_async;   /* the mock Renode's ends   */
    sim_runtime_t sim;
    sim_mote_t mote;
    renode_cosim_service_t svc;
} mock_t;

/* In a real run the service host owns the single fan-out observer and calls
 * every service's on_event from it.  The mock attaches the service directly
 * (there is no Renode to connect to), so it subscribes the same callback
 * itself — the code under test is the service's own on_event either way. */
static void mock_observer(void *user, const sim_observer_event_t *ev) {
    renode_cosim_service_t *s = (renode_cosim_service_t *)user;
    if (renode_cosim_service_ops.on_event)
        renode_cosim_service_ops.on_event(stub_sim, s, ev);
}

/* Write one message into the master's main socket, as Renode would. */
static void master_send(mock_t *k, int32_t action, uint64_t addr, uint64_t value) {
    renode_msg_t m = { action, addr, value, RENODE_NO_PERIPHERAL_INDEX };
    uint8_t buf[RENODE_MSG_SIZE];
    renode_msg_encode(&m, buf);
    ssize_t n = write(k->master_main, buf, sizeof(buf));
    (void)n;
}

/* Read one message the service sent.  Returns 1 on success, 0 if nothing is
 * waiting.  The master side is non-blocking so a test that asserts silence
 * ("nothing was written from an observer callback") cannot hang the suite. */
static int master_recv(int fd, renode_msg_t *m) {
    uint8_t buf[RENODE_MSG_SIZE];
    size_t got = 0;
    int spins = 0;
    while (got < sizeof(buf)) {
        ssize_t n = read(fd, buf + got, sizeof(buf) - got);
        if (n > 0) { got += (size_t)n; continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && got > 0 &&
            spins++ < 1000) {
            continue;                    /* mid-message: let the writer finish */
        }
        return 0;
    }
    renode_msg_decode(buf, m);
    return 1;
}

/* Read exactly `want` bytes of a log message's text (blocking-ish, bounded). */
static size_t master_read_text(int fd, char *out, size_t want) {
    size_t got = 0;
    int spins = 0;
    while (got < want && spins < 1000) {
        ssize_t n = read(fd, out + got, want - got);
        if (n > 0) { got += (size_t)n; continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { spins++; continue; }
        break;
    }
    return got;
}

/* Discard whatever the service has replied.  A socketpair holds only a few
 * kilobytes, so a test that ticks thousands of times must drain, or the
 * service blocks writing an acknowledgement nobody is reading. */
static int master_drain(int fd) {
    renode_msg_t r;
    int n = 0;
    while (master_recv(fd, &r)) n++;
    return n;
}

static void mock_setup(mock_t *k) {
    memset(k, 0, sizeof(*k));
    renode_dev_init(&stub_dev);
    stub_dev.self_id = 3;
    stub_dev.now_ns  = stub_dev_now;

    sim_runtime_init(&k->sim);
    stub_sim = &k->sim;
    k->mote.id = 3;
    k->mote.ops = &stub_ops;
    k->mote.impl = NULL;
    sim_runtime_register_mote(&k->sim, 0, &k->mote);

    int mainp[2], asyncp[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, mainp);
    socketpair(AF_UNIX, SOCK_STREAM, 0, asyncp);
    k->master_main  = mainp[0];
    k->master_async = asyncp[0];
    fcntl(k->master_main, F_SETFL, O_NONBLOCK);
    fcntl(k->master_async, F_SETFL, O_NONBLOCK);

    /* Renode opens with the handshake, so it is already waiting when the
     * service attaches. */
    renode_msg_t hs = { RENODE_HANDSHAKE, 0, 0, RENODE_NO_PERIPHERAL_INDEX };
    uint8_t b[RENODE_MSG_SIZE];
    renode_msg_encode(&hs, b);
    ssize_t wn = write(k->master_main, b, sizeof(b));
    (void)wn;

    k->svc.cfg.freq_hz = 1000000;          /* 1 MHz: 1 tick = 1 µs */
    k->svc.cfg.log_level = RENODE_LOG_INFO;
    k->svc.cfg.wait_timeout_ms = 2000;     /* never hang the suite */
    k->svc.cfg.connect_timeout_ms = 2000;
    renode_cosim_attach_fds(&k->svc, &k->sim, mainp[1], asyncp[1]);
    sim_runtime_subscribe(&k->sim, mock_observer, &k->svc);
}

static void mock_teardown(mock_t *k) {
    if (k->master_main >= 0) close(k->master_main);
    if (k->master_async >= 0) close(k->master_async);
    /* Through the real destroy op, so the teardown path (fd close, buffer
     * release, clearing the attached instance) is exercised too.  It prints
     * the run's tally, which is the line CI greps for. */
    if (renode_cosim_service_ops.destroy)
        renode_cosim_service_ops.destroy(&k->sim, &k->svc);
    ASSERT(renode_cosim_current() == NULL,
           "destroy clears the attached instance");
    sim_runtime_destroy(&k->sim);
    stub_sim = NULL;
}

static void test_service_handshake_ticks(int verbose) {
    if (verbose) printf("  -- service: handshake + tick clock --\n");
    mock_t k;
    mock_setup(&k);

    ASSERT(renode_cosim_active(&k.svc), "attached after the handshake");
    renode_msg_t r;
    ASSERT(master_recv(k.master_main, &r), "handshake reply sent");
    ASSERT_EQ(r.action, RENODE_HANDSHAKE, "handshake is echoed");

    /* Three quanta of 1000 ticks at 1 MHz = 1 ms each.  The horizon must be
     * the exact accumulated time, and each quantum must be acknowledged
     * once, on the main socket. */
    for (int i = 1; i <= 3; i++) {
        master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
        int64_t h = renode_cosim_next_horizon(&k.svc, (i - 1) * 1000000LL);
        ASSERT_EQ(h, i * 1000000LL, "horizon advances by exactly one quantum");
        if (i > 1) {
            /* On the ASYNC socket, as Renode's integration library does
             * (renode_bus.cpp `case tickClock` replies with sendSender).
             * Reading it from main here would let a wrong implementation
             * pass a test that agrees with it rather than with Renode. */
            ASSERT(master_recv(k.master_async, &r), "previous quantum acknowledged");
            ASSERT_EQ(r.action, RENODE_TICK_CLOCK, "acknowledgement is a tickClock");
            ASSERT(!master_recv(k.master_main, &r),
                   "nothing but bus replies goes on the main socket");
        }
    }
    ASSERT_EQ(k.svc.stats.ticks, 3, "three ticks counted");
    mock_teardown(&k);
}

static void test_service_tick_arithmetic(int verbose) {
    if (verbose) printf("  -- service: drift-free tick arithmetic --\n");
    mock_t k;
    mock_setup(&k);
    /* 3 Hz does not divide a second, so an implementation that accumulated
     * per-tick nanoseconds would drift.  After three ticks the horizon must
     * be exactly one second, not 999999999 ns. */
    k.svc.cfg.freq_hz = 3;
    int64_t cur = 0;
    for (int i = 0; i < 3; i++) {
        master_send(&k, RENODE_TICK_CLOCK, 0, 1);
        cur = renode_cosim_next_horizon(&k.svc, cur);
        master_drain(k.master_async);
    }
    ASSERT_EQ(cur, 1000000000LL, "3 ticks at 3 Hz is exactly one second");

    /* And it keeps being exact far from the origin. */
    for (int i = 0; i < 3000; i++) {
        master_send(&k, RENODE_TICK_CLOCK, 0, 1);
        cur = renode_cosim_next_horizon(&k.svc, cur);
        master_drain(k.master_async);
    }
    ASSERT_EQ(cur, 1001000000000LL, "no drift after 3003 ticks");
    mock_teardown(&k);
}

static void test_service_bus(int verbose) {
    if (verbose) printf("  -- service: bus reads and writes --\n");
    mock_t k;
    mock_setup(&k);
    renode_msg_t r;
    master_recv(k.master_main, &r);   /* handshake echo */

    /* Renode interleaves bus accesses with ticks; the service answers each
     * one inline and keeps waiting for the tick. */
    master_send(&k, RENODE_READ_DWORD, RENODE_REG_ID, 0);
    master_send(&k, RENODE_WRITE_DWORD, RENODE_REG_CTRL, RENODE_CTRL_IRQ_RX_EN);
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    int64_t h = renode_cosim_next_horizon(&k.svc, 0);
    ASSERT_EQ(h, 1000000LL, "horizon after the interleaved accesses");

    ASSERT(master_recv(k.master_main, &r), "read answered");
    ASSERT_EQ(r.action, RENODE_READ_REQUEST, "read reply uses the readRequest action");
    ASSERT_EQ(r.value, 0x4353494DU, "read returned the device ID");
    ASSERT_EQ(r.addr, RENODE_REG_ID, "read reply echoes the offset");
    ASSERT(master_recv(k.master_main, &r), "write answered");
    ASSERT_EQ(r.action, RENODE_OK, "write reply is ok");
    ASSERT_EQ(k.svc.stats.reads, 1, "one read counted");
    ASSERT_EQ(k.svc.stats.writes, 1, "one write counted");

    /* Sim time read back through the window is the quantum boundary, which
     * is what the guest must see: the service pins now_ns there before
     * servicing accesses, rather than leaving it at the last event. */
    k.sim.now_ns = 250000;            /* pump stopped early inside the quantum */
    master_send(&k, RENODE_READ_DWORD, RENODE_REG_SIM_TIME_LO, 0);
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    renode_cosim_next_horizon(&k.svc, 1000000LL);
    ASSERT(master_recv(k.master_async, &r), "quantum 1 acknowledged on async");
    ASSERT_EQ(r.action, RENODE_TICK_CLOCK, "acknowledgement is a tickClock");
    ASSERT(master_recv(k.master_main, &r), "sim-time read answered");
    ASSERT_EQ(r.value, 1000000, "sim time reads the quantum boundary, not the last event");
    mock_teardown(&k);
}

static void test_service_async_plane(int verbose) {
    if (verbose) printf("  -- service: interrupts + log forwarding --\n");
    mock_t k;
    mock_setup(&k);
    renode_msg_t r;
    master_recv(k.master_main, &r);   /* handshake echo */

    /* Arm the RX interrupt, then deliver a frame the way the medium would. */
    master_send(&k, RENODE_WRITE_DWORD, RENODE_REG_CTRL, RENODE_CTRL_IRQ_RX_EN);
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    renode_cosim_next_horizon(&k.svc, 0);
    master_recv(k.master_main, &r);   /* the write's ok */

    static const uint8_t frame[] = { 0x41, 0x88, 0x2A };
    renode_dev_rx_push(&stub_dev, frame, sizeof(frame), 500000, 1, 26, -70);

    /* A console line arrives through the observer stream, as the runner
     * emits it.  Nothing may be written from inside that callback. */
    sim_observer_event_t ev = { .kind = SIM_OBS_MOTE_LOG_LINE, .time_ns = 500000,
                                .mote_index = 1, .radio_idx = -1 };
    ev.u.log_line.line = "hello from node 1";
    ev.u.log_line.len = 17;
    ev.u.log_line.node_id = 1;
    sim_runtime_emit(&k.sim, &ev);
    renode_msg_t none;
    ASSERT(!master_recv(k.master_async, &none),
           "nothing is written to the socket from an observer callback");

    /* The quantum boundary flushes both, log first, then the interrupt. */
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    renode_cosim_next_horizon(&k.svc, 1000000LL);

    ASSERT(master_recv(k.master_async, &r), "a log message was flushed");
    ASSERT_EQ(r.action, RENODE_LOG_MESSAGE, "log message action");
    ASSERT_EQ(r.value, RENODE_LOG_INFO, "log level in the value field");
    char text[128];
    size_t want = (size_t)r.addr;
    ASSERT(want < sizeof(text), "log length is sane");
    size_t got = master_read_text(k.master_async, text, want);
    text[got] = '\0';
    ASSERT(strcmp(text, "[node 1] hello from node 1\n") == 0,
           "log text carries the node id and the line");

    ASSERT(master_recv(k.master_async, &r), "an interrupt followed");
    ASSERT_EQ(r.action, RENODE_INTERRUPT, "interrupt action");
    ASSERT_EQ(r.addr, RENODE_DEV_IRQ_INDEX, "interrupt index 0");
    ASSERT_EQ(r.value, 1, "line raised");
    /* The quantum's tick acknowledgement closes the burst: everything that
     * happened during the quantum is reported before it is declared over. */
    ASSERT(master_recv(k.master_async, &r), "tick acknowledged last");
    ASSERT_EQ(r.action, RENODE_TICK_CLOCK, "tick ack after the events");

    /* Draining the queue lowers it, and only the change is reported. */
    master_send(&k, RENODE_WRITE_DWORD, RENODE_REG_RX_POP, 1);
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    renode_cosim_next_horizon(&k.svc, 2000000LL);
    /* Order on the async stream: the previous quantum is acknowledged first
     * (that happens before any new request is serviced), and only then the
     * interrupt the guest's own RX_POP caused. */
    ASSERT(master_recv(k.master_async, &r), "previous quantum acknowledged");
    ASSERT_EQ(r.action, RENODE_TICK_CLOCK, "tick ack comes first");
    ASSERT(master_recv(k.master_async, &r), "interrupt change reported");
    ASSERT_EQ(r.action, RENODE_INTERRUPT, "interrupt action again");
    ASSERT_EQ(r.value, 0, "line lowered");
    ASSERT(!master_recv(k.master_async, &none), "an unchanged level is not resent");
    ASSERT_EQ(k.svc.stats.irqs, 2, "exactly two interrupt messages");
    mock_teardown(&k);
}

static void test_service_disconnect(int verbose) {
    if (verbose) printf("  -- service: disconnect and peer loss --\n");
    mock_t k;
    mock_setup(&k);
    renode_msg_t r;
    master_recv(k.master_main, &r);

    master_send(&k, RENODE_DISCONNECT, 0, 0);
    int64_t h = renode_cosim_next_horizon(&k.svc, 0);
    ASSERT_EQ(h, -1, "disconnect ends the run");
    ASSERT(!renode_cosim_active(&k.svc), "service is inactive after disconnect");
    ASSERT(master_recv(k.master_async, &r), "disconnect is acknowledged");
    ASSERT_EQ(r.action, RENODE_OK, "acknowledgement is ok, on the async socket");
    mock_teardown(&k);

    /* A master that dies without saying goodbye must end the run too, not
     * leave csim free-running without its clock. */
    mock_setup(&k);
    master_recv(k.master_main, &r);
    close(k.master_main);
    k.master_main = -1;
    h = renode_cosim_next_horizon(&k.svc, 0);
    ASSERT_EQ(h, -1, "a closed connection ends the run");
    ASSERT(!renode_cosim_active(&k.svc), "service is inactive after peer loss");
    mock_teardown(&k);
}

static void test_service_reset_and_unsupported(int verbose) {
    if (verbose) printf("  -- service: reset + unsupported actions --\n");
    mock_t k;
    mock_setup(&k);
    renode_msg_t r;
    master_recv(k.master_main, &r);

    static const uint8_t f[] = { 0x01 };
    renode_dev_rx_push(&stub_dev, f, 1, 0, 1, 26, -60);
    master_send(&k, RENODE_WRITE_DWORD, RENODE_REG_CTRL, RENODE_CTRL_IRQ_RX_EN);
    master_send(&k, RENODE_RESET_PERIPHERAL, 0, 0);
    /* A co-simulated-CPU action must be ignored, not answered: a reply
     * Renode is not waiting for would desynchronise every message after it. */
    master_send(&k, RENODE_REGISTER_GET, 0, 0);
    master_send(&k, RENODE_TICK_CLOCK, 0, 1000);
    int64_t h = renode_cosim_next_horizon(&k.svc, 0);
    ASSERT_EQ(h, 1000000LL, "the run continues past reset and an unknown action");

    ASSERT(master_recv(k.master_main, &r), "the CTRL write was answered");
    ASSERT_EQ(r.action, RENODE_OK, "write ok");
    ASSERT(!master_recv(k.master_main, &r),
           "reset and the unsupported action produced no reply");
    ASSERT_EQ(renode_dev_read(&stub_dev, RENODE_REG_RX_COUNT, 4), 0,
              "reset emptied the device");
    mock_teardown(&k);
}

static void test_service_config_parse(int verbose) {
    if (verbose) printf("  -- service: connection spec parsing --\n");
    renode_cosim_config_t c;

    ASSERT_EQ(renode_cosim_config_parse(&c, "127.0.0.1:1234:5678"), 0, "valid spec");
    ASSERT(strcmp(c.host, "127.0.0.1") == 0, "host parsed");
    ASSERT_EQ(c.main_port, 1234, "main port parsed");
    ASSERT_EQ(c.async_port, 5678, "async port parsed");
    ASSERT_EQ(c.freq_hz, 1000000, "default tick frequency");

    ASSERT_EQ(renode_cosim_config_parse(&c, "127.0.0.1:1234"), -1, "missing a port");
    ASSERT_EQ(renode_cosim_config_parse(&c, "1234:5678"), -1, "missing the host");
    ASSERT_EQ(renode_cosim_config_parse(&c, "127.0.0.1:abc:5678"), -1, "non-numeric port");
    ASSERT_EQ(renode_cosim_config_parse(&c, "127.0.0.1:1234:99999"), -1, "port out of range");
    ASSERT_EQ(renode_cosim_config_parse(&c, ""), -1, "empty spec");
}

/* ============================================================
 * Entry point
 * ============================================================ */

int run_renode_cosim_tests(int verbose) {
    printf("\n=== Renode co-simulation tests ===\n");
    passed = failed = 0;

    test_proto(verbose);
    test_dev_identity(verbose);
    test_dev_tx(verbose);
    test_dev_rx(verbose);
    test_dev_irq(verbose);
    test_dev_uart(verbose);
    test_dev_reset(verbose);
    test_dev_channel_power(verbose);
    test_service_config_parse(verbose);
    test_service_handshake_ticks(verbose);
    test_service_tick_arithmetic(verbose);
    test_service_bus(verbose);
    test_service_async_plane(verbose);
    test_service_disconnect(verbose);
    test_service_reset_and_unsupported(verbose);

    printf("  %d passed, %d failed\n", passed, failed);
    return failed;
}
