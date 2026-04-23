/*
 * JS node unit tests — exercise the js_node_t API directly.
 *
 * Tests:
 *   1. init + execute: Load broadcast.js, verify init() runs, execute() fires
 *   2. mote.log: Verify log callback receives expected bytes (including \n)
 *   3. mote.send: Verify rf_frame_callback receives 802.15.4 frame bytes
 *   4. mote.scheduleWakeup: Schedule wakeup, verify next_wakeup_ns, verify execute fires
 *   5. receivedPacket: Deliver frame, verify JS handler runs (broadcast.js logs "rx from=...")
 *   6. ping-pong round trip: Two nodes, manual TX relay, verify pong response
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "js_node.h"

/* --- Capture buffers --- */

#define LOG_BUF_SIZE 4096
static char log_buf[LOG_BUF_SIZE];
static int  log_pos;

#define FRAME_BUF_SIZE 256
static uint8_t frame_buf[FRAME_BUF_SIZE];
static int     frame_len;
static int     frame_count;

static void reset_captures(void) {
    log_pos = 0;
    log_buf[0] = '\0';
    frame_len = 0;
    frame_count = 0;
}

static void test_log_callback(void *user_data, uint8_t byte) {
    (void)user_data;
    if (log_pos < LOG_BUF_SIZE - 1)
        log_buf[log_pos++] = (char)byte;
    log_buf[log_pos] = '\0';
}

static void test_rf_callback(void *user_data, const uint8_t *frame, int len) {
    (void)user_data;
    if (len > FRAME_BUF_SIZE) len = FRAME_BUF_SIZE;
    memcpy(frame_buf, frame, (size_t)len);
    frame_len = len;
    frame_count++;
}

/* --- Test helpers --- */

#define PASS(name) do { if (verbose) printf("  PASS: %s\n", name); passed++; } while(0)
#define FAIL(name, ...) do { printf("  FAIL: %s - ", name); printf(__VA_ARGS__); printf("\n"); failed++; } while(0)

static int passed, failed;

/* --- Tests --- */

static void test_init_and_execute(int verbose) {
    const char *name = "init + execute";
    js_node_t node;
    reset_captures();

    if (js_node_init(&node, "firmware/js/broadcast.js", 1) != 0) {
        FAIL(name, "js_node_init failed");
        return;
    }

    node.log_callback = test_log_callback;
    node.log_callback_data = NULL;
    node.rf_frame_callback = test_rf_callback;
    node.rf_frame_callback_data = NULL;

    js_node_start(&node);

    /* init() should have logged "js mote 1 starting\n" */
    if (strstr(log_buf, "js mote 1 starting") == NULL) {
        FAIL(name, "init() log not found: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }

    /* After start, next_wakeup_ns should be 0 (broadcast.js schedules at 0) */
    if (js_node_next_wakeup_ns(&node) != 0) {
        FAIL(name, "expected wakeup at 0, got %lld",
             (long long)js_node_next_wakeup_ns(&node));
        js_node_destroy(&node);
        return;
    }

    /* Step to t=0 to fire execute() */
    reset_captures();
    js_node_step_until_ns(&node, 0);

    /* execute() should log "tx seq=0" and send a frame */
    if (strstr(log_buf, "tx seq=0") == NULL) {
        FAIL(name, "execute() log not found: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }
    if (frame_count < 1) {
        FAIL(name, "execute() did not send a frame");
        js_node_destroy(&node);
        return;
    }

    js_node_destroy(&node);
    PASS(name);
}

static void test_mote_log(int verbose) {
    const char *name = "mote.log callback";
    js_node_t node;
    reset_captures();

    if (js_node_init(&node, "firmware/js/broadcast.js", 42) != 0) {
        FAIL(name, "js_node_init failed");
        return;
    }

    node.log_callback = test_log_callback;
    node.log_callback_data = NULL;
    node.rf_frame_callback = test_rf_callback;
    node.rf_frame_callback_data = NULL;

    js_node_start(&node);

    /* Check that log includes node id and ends with \n */
    if (strstr(log_buf, "42") == NULL) {
        FAIL(name, "node id not in log: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }
    if (log_pos == 0 || log_buf[log_pos - 1] != '\n') {
        FAIL(name, "log does not end with newline");
        js_node_destroy(&node);
        return;
    }

    js_node_destroy(&node);
    PASS(name);
}

static void test_mote_send(int verbose) {
    const char *name = "mote.send frame";
    js_node_t node;
    reset_captures();

    if (js_node_init(&node, "firmware/js/broadcast.js", 1) != 0) {
        FAIL(name, "js_node_init failed");
        return;
    }

    node.log_callback = test_log_callback;
    node.log_callback_data = NULL;
    node.rf_frame_callback = test_rf_callback;
    node.rf_frame_callback_data = NULL;

    js_node_start(&node);
    js_node_step_until_ns(&node, 0);

    /* broadcast.js sends a frame with FCF 0x41,0x88, broadcast dst, src=mote.id */
    if (frame_count < 1) {
        FAIL(name, "no frame sent");
        js_node_destroy(&node);
        return;
    }
    if (frame_len < 9) {
        FAIL(name, "frame too short: %d bytes", frame_len);
        js_node_destroy(&node);
        return;
    }
    /* Check FCF */
    if (frame_buf[0] != 0x41 || frame_buf[1] != 0x88) {
        FAIL(name, "wrong FCF: 0x%02X 0x%02X", frame_buf[0], frame_buf[1]);
        js_node_destroy(&node);
        return;
    }
    /* Check src addr (bytes 7-8 LE) = node id 1 */
    int src = frame_buf[7] | (frame_buf[8] << 8);
    if (src != 1) {
        FAIL(name, "wrong src addr: %d", src);
        js_node_destroy(&node);
        return;
    }

    js_node_destroy(&node);
    PASS(name);
}

static void test_schedule_wakeup(int verbose) {
    const char *name = "scheduleWakeup";
    js_node_t node;
    reset_captures();

    if (js_node_init(&node, "firmware/js/broadcast.js", 1) != 0) {
        FAIL(name, "js_node_init failed");
        return;
    }

    node.log_callback = test_log_callback;
    node.log_callback_data = NULL;
    node.rf_frame_callback = test_rf_callback;
    node.rf_frame_callback_data = NULL;

    js_node_start(&node);

    /* Fire the first execute at t=0 */
    js_node_step_until_ns(&node, 0);

    /* broadcast.js schedules next wakeup at 2s (SEND_INTERVAL_NS = 2e9) */
    int64_t next = js_node_next_wakeup_ns(&node);
    int64_t expected = 2000000000LL; /* 2s */
    if (next != expected) {
        FAIL(name, "expected next wakeup at %lld, got %lld",
             (long long)expected, (long long)next);
        js_node_destroy(&node);
        return;
    }

    /* Step to 2s, verify second execute fires */
    reset_captures();
    frame_count = 0;
    js_node_step_until_ns(&node, expected);

    if (strstr(log_buf, "tx seq=1") == NULL) {
        FAIL(name, "second execute() log not found: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }
    if (frame_count < 1) {
        FAIL(name, "second execute() did not send a frame");
        js_node_destroy(&node);
        return;
    }

    js_node_destroy(&node);
    PASS(name);
}

static void test_received_packet(int verbose) {
    const char *name = "receivedPacket";
    js_node_t node;
    reset_captures();

    if (js_node_init(&node, "firmware/js/broadcast.js", 2) != 0) {
        FAIL(name, "js_node_init failed");
        return;
    }

    node.log_callback = test_log_callback;
    node.log_callback_data = NULL;
    node.rf_frame_callback = test_rf_callback;
    node.rf_frame_callback_data = NULL;

    js_node_start(&node);
    js_node_step_until_ns(&node, 0);
    reset_captures();

    /* Build a fake 802.15.4 frame from node 1:
     * FCF(2) + seq(1) + PAN(2) + dst(2) + src(2) + payload + FCS(2) */
    uint8_t frame[] = {
        0x41, 0x88,        /* FCF: data, intra-PAN, short addr */
        0x00,              /* seq */
        0xCD, 0xAB,        /* dst PAN */
        0xFF, 0xFF,        /* dst addr (broadcast) */
        0x01, 0x00,        /* src addr = 1 (LE) */
        'H', 'i',          /* payload */
        0x00, 0x00         /* FCS placeholder */
    };

    /* Deliver at t=1s */
    int64_t arrival = 1000000000LL;
    js_node_deliver_frame(&node, frame, sizeof(frame), arrival);

    /* Step past arrival time */
    js_node_step_until_ns(&node, arrival);

    /* broadcast.js receivedPacket logs "rx from=1 len=..." */
    if (strstr(log_buf, "rx from=1") == NULL) {
        FAIL(name, "receivedPacket log not found: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }
    if (strstr(log_buf, "text='Hi") == NULL) {
        FAIL(name, "payload text not in log: '%s'", log_buf);
        js_node_destroy(&node);
        return;
    }

    js_node_destroy(&node);
    PASS(name);
}

static void test_ping_pong(int verbose) {
    const char *name = "ping-pong round trip";
    js_node_t pinger, responder;
    reset_captures();

    /* Separate log buffers per node */
    char pinger_log[LOG_BUF_SIZE];
    char responder_log[LOG_BUF_SIZE];

    /* Frame capture per node */
    uint8_t pinger_frame[FRAME_BUF_SIZE];
    int     pinger_frame_len = 0;
    uint8_t responder_frame[FRAME_BUF_SIZE];
    int     responder_frame_len = 0;

    /* Init both nodes */
    if (js_node_init(&pinger, "firmware/js/ping-pong.js", 1) != 0) {
        FAIL(name, "pinger init failed");
        return;
    }
    if (js_node_init(&responder, "firmware/js/ping-pong.js", 2) != 0) {
        FAIL(name, "responder init failed");
        js_node_destroy(&pinger);
        return;
    }

    /* Wire callbacks that write to the shared log_buf/frame_buf,
     * capturing between steps. */
    pinger.log_callback = test_log_callback;
    pinger.log_callback_data = NULL;
    pinger.rf_frame_callback = test_rf_callback;
    pinger.rf_frame_callback_data = NULL;

    responder.log_callback = test_log_callback;
    responder.log_callback_data = NULL;
    responder.rf_frame_callback = test_rf_callback;
    responder.rf_frame_callback_data = NULL;

    /* Start both */
    reset_captures();
    js_node_start(&pinger);
    memcpy(pinger_log, log_buf, (size_t)log_pos + 1);

    reset_captures();
    js_node_start(&responder);
    memcpy(responder_log, log_buf, (size_t)log_pos + 1);

    /* Verify pinger init log */
    if (strstr(pinger_log, "pinger: starting") == NULL) {
        FAIL(name, "pinger init log not found: '%s'", pinger_log);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }
    /* Verify responder init log */
    if (strstr(responder_log, "responder: ready") == NULL) {
        FAIL(name, "responder init log not found: '%s'", responder_log);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }

    /* Pinger's first execute at 0.5s (NS_PER_S / 2 = 500ms) */
    int64_t ping_time = 500000000LL;

    /* Step both to just before ping time */
    js_node_step_until_ns(&pinger, ping_time - 1);
    js_node_step_until_ns(&responder, ping_time - 1);

    /* Step pinger to fire execute — sends PING frame */
    reset_captures();
    js_node_step_until_ns(&pinger, ping_time);

    if (frame_count < 1) {
        FAIL(name, "pinger did not send PING frame");
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }
    if (strstr(log_buf, "ping seq=0 sent") == NULL) {
        FAIL(name, "pinger log missing 'ping seq=0 sent': '%s'", log_buf);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }

    /* Capture the ping frame */
    memcpy(pinger_frame, frame_buf, (size_t)frame_len);
    pinger_frame_len = frame_len;

    /* Deliver ping frame to responder at ping_time */
    js_node_deliver_frame(&responder, pinger_frame, pinger_frame_len, ping_time);

    /* Step responder — should receive PING and reply with PONG */
    reset_captures();
    js_node_step_until_ns(&responder, ping_time);

    if (strstr(log_buf, "ping from 1") == NULL) {
        FAIL(name, "responder did not log ping receipt: '%s'", log_buf);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }
    if (frame_count < 1) {
        FAIL(name, "responder did not send PONG frame");
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }

    /* Capture pong frame */
    memcpy(responder_frame, frame_buf, (size_t)frame_len);
    responder_frame_len = frame_len;

    /* Deliver pong back to pinger at ping_time + 1ms (simulated propagation) */
    int64_t pong_arrival = ping_time + 1000000LL; /* 1ms */
    js_node_deliver_frame(&pinger, responder_frame, responder_frame_len,
                          pong_arrival);

    /* Step pinger to receive pong */
    reset_captures();
    js_node_step_until_ns(&pinger, pong_arrival);

    if (strstr(log_buf, "pong from 2") == NULL) {
        FAIL(name, "pinger did not log pong receipt: '%s'", log_buf);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }
    if (strstr(log_buf, "rtt=") == NULL) {
        FAIL(name, "pinger did not log RTT: '%s'", log_buf);
        js_node_destroy(&pinger);
        js_node_destroy(&responder);
        return;
    }

    js_node_destroy(&pinger);
    js_node_destroy(&responder);
    PASS(name);
}

/* --- Entry point --- */

int run_js_node_tests(int verbose) {
    printf("\n=== JS Node Unit Tests ===\n");
    passed = 0;
    failed = 0;

    test_init_and_execute(verbose);
    test_mote_log(verbose);
    test_mote_send(verbose);
    test_schedule_wakeup(verbose);
    test_received_packet(verbose);
    test_ping_pong(verbose);

    printf("\nJS node tests: %d passed, %d failed\n", passed, failed);
    return failed;
}
