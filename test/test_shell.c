/*
 * Shell unit tests: the pure parser (tokenizer, time literals, node
 * selectors) and the script engine driven against a mock sim_control
 * bundle — no CPU, no firmware.  `test_runner shell [-v]`.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shell_parse.h"
#include "sim_runtime.h"
#include "sim_control.h"
#include "../src/services/shell_internal.h"

static int g_fail = 0, g_pass = 0, g_verbose = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; if (g_verbose) { printf("  ok: "); printf(__VA_ARGS__); printf("\n"); } } \
    else { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* --- tokenizer ------------------------------------------------------------ */

static void test_tokenize(void) {
    char *argv[SHELL_MAX_ARGS]; int pos[SHELL_MAX_ARGS]; char st[512]; char err[128];
    int n = shell_tokenize("send 1 \"help me\" 'a b' x\\ty  # comment", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 5, "argc=%d", n);
    CHECK(n == 5 && strcmp(argv[0], "send") == 0, "argv0");
    CHECK(n == 5 && strcmp(argv[2], "help me") == 0, "double quotes keep spaces: '%s'", n == 5 ? argv[2] : "");
    CHECK(n == 5 && strcmp(argv[3], "a b") == 0, "single quotes");
    CHECK(n == 5 && strcmp(argv[4], "x\ty") == 0, "bare-word escape");
    CHECK(n == 5 && pos[2] == 7, "argpos of word 2 = %d", n == 5 ? pos[2] : -1);
    n = shell_tokenize("   # only a comment", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 0, "comment-only line -> 0 (%d)", n);
    n = shell_tokenize("", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 0, "empty line -> 0");
    n = shell_tokenize("sendln 1 \"help\\n\"", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 3 && strcmp(argv[2], "help\n") == 0, "\\n escape inside quotes");
    n = shell_tokenize("x \"\\x41\\x42\"", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 2 && strcmp(argv[1], "AB") == 0, "\\xHH escape");
    n = shell_tokenize("x 'lit\\n'", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 2 && strcmp(argv[1], "lit\\n") == 0, "single quotes are literal");
    n = shell_tokenize("x \"unterminated", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == -1 && strstr(err, "unterminated"), "unterminated quote -> error (%s)", err);
    n = shell_tokenize("x \\q", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == -1, "unknown escape -> error");
    n = shell_tokenize("a b c d", argv, pos, 3, st, sizeof(st), err, sizeof(err));
    CHECK(n == -1 && strstr(err, "too many"), "argv overflow -> error");
    n = shell_tokenize("echo it's", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == -1, "stray apostrophe is an unterminated quote");
    n = shell_tokenize("expect 1 \"#> \"", argv, pos, SHELL_MAX_ARGS, st, sizeof(st), err, sizeof(err));
    CHECK(n == 3 && strcmp(argv[2], "#> ") == 0, "# inside quotes is not a comment");
}

/* --- time ----------------------------------------------------------------- */

static void test_time(void) {
    int64_t ns; bool rel;
    CHECK(shell_parse_time("5s", &ns, &rel) == 0 && ns == 5000000000LL && !rel, "5s");
    CHECK(shell_parse_time("250ms", &ns, &rel) == 0 && ns == 250000000LL, "250ms");
    CHECK(shell_parse_time("1500us", &ns, &rel) == 0 && ns == 1500000LL, "1500us");
    CHECK(shell_parse_time("12345ns", &ns, &rel) == 0 && ns == 12345, "12345ns");
    CHECK(shell_parse_time("1.5s", &ns, &rel) == 0 && ns == 1500000000LL, "1.5s");
    CHECK(shell_parse_time("2m", &ns, &rel) == 0 && ns == 120000000000LL, "2m");
    CHECK(shell_parse_time("500", &ns, &rel) == 0 && ns == 500000000LL, "bare number = ms");
    CHECK(shell_parse_time("+2s", &ns, &rel) == 0 && ns == 2000000000LL && rel, "+2s relative");
    CHECK(shell_parse_time("abc", &ns, &rel) != 0, "abc rejected");
    CHECK(shell_parse_time("-1s", &ns, &rel) != 0, "negative rejected");
    CHECK(shell_parse_time("5x", &ns, &rel) != 0, "bad unit rejected");
    CHECK(shell_parse_time("", &ns, &rel) != 0, "empty rejected");
    CHECK(shell_parse_duration("+2s", &ns) != 0, "duration rejects relative form");
    long v;
    CHECK(shell_parse_int("12", &v) == 0 && v == 12, "int");
    CHECK(shell_parse_int("0x10", &v) == 0 && v == 16, "hex int");
    CHECK(shell_parse_int("12a", &v) != 0, "trailing junk rejected");
    CHECK(shell_compare(3, ">=", 3) == 1 && shell_compare(2, "==", 3) == 0 && shell_compare(1, "~", 1) == -1, "compare");
}

/* --- selectors ------------------------------------------------------------- */

static void test_selector(void) {
    int ids[] = { 1, 2, 3, 5, 8 };
    int out[16]; bool any; char err[128];
    int n = shell_parse_selector("1", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 1 && out[0] == 1, "single id");
    n = shell_parse_selector("1,3", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 2 && out[0] == 1 && out[1] == 3, "list");
    n = shell_parse_selector("2-5", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 3 && out[0] == 2 && out[1] == 3 && out[2] == 5, "range expands to existing ids (%d)", n);
    n = shell_parse_selector("8,1-2", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 3 && out[0] == 1 && out[1] == 2 && out[2] == 8, "slot order, mixed");
    n = shell_parse_selector("all", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 5 && out[4] == 8, "all");
    n = shell_parse_selector("any", ids, 5, out, 16, true, &any, err, sizeof(err));
    CHECK(n == 0 && any, "any allowed");
    n = shell_parse_selector("any", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == -1, "any refused when not allowed");
    n = shell_parse_selector("4", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == -1 && strstr(err, "no node"), "unknown id -> error (%s)", err);
    n = shell_parse_selector("9-12", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == -1, "empty range -> error");
    n = shell_parse_selector("5-2", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == -1, "reversed range -> error");
    n = shell_parse_selector("1,,2", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == -1, "syntax error -> error");
    n = shell_parse_selector("1,1,1", ids, 5, out, 16, false, &any, err, sizeof(err));
    CHECK(n == 1, "duplicates removed");
}

/* --- script engine on a mock control bundle -------------------------------- */

typedef struct { int id; int64_t start_ns; char fw[64]; } mock_node_t;
static mock_node_t mock_nodes[8];
static int mock_count;
static sim_runtime_t mock_sim;
static int mock_inject_calls, mock_inject_bytes, mock_reboots, mock_removes, mock_moves;
static char mock_last_inject[256];

static int m_node_count(void *u) { (void)u; return mock_count; }
static bool m_describe(void *u, int idx, sim_control_node_info_t *o) {
    (void)u;
    if (idx < 0 || idx >= mock_count) return false;
    o->index = idx; o->id = mock_nodes[idx].id; o->type = "MOCK"; o->firmware = mock_nodes[idx].fw;
    o->start_ns = mock_nodes[idx].start_ns; o->removed = o->start_ns == INT64_MAX;
    o->active = sim_runtime_now_ns(&mock_sim) >= o->start_ns;
    return true;
}
static int m_inject(void *u, int idx, const uint8_t *b, int n) {
    (void)u; (void)idx; mock_inject_calls++; mock_inject_bytes += n;
    int c = n < (int)sizeof(mock_last_inject) - 1 ? n : (int)sizeof(mock_last_inject) - 1;
    memcpy(mock_last_inject, b, (size_t)c); mock_last_inject[c] = 0;
    return n;
}
static int m_inject_half(void *u, int idx, const uint8_t *b, int n) {
    (void)u; (void)idx; (void)b; mock_inject_calls++;
    int c = n > 2 ? 2 : n; mock_inject_bytes += c; return c;
}
static void m_set_position(void *u, int idx, double x, double y) { (void)u; (void)idx; (void)x; (void)y; mock_moves++; }
static int m_reboot(void *u, int idx) { (void)u; (void)idx; mock_reboots++; return 0; }
static void m_start(void *u, int idx) { (void)u; mock_nodes[idx].start_ns = sim_runtime_now_ns(&mock_sim); }
static void m_remove(void *u, int idx) { (void)u; mock_nodes[idx].start_ns = INT64_MAX; mock_removes++; }
static int m_add(void *u, const char *fw, const char *sfw, int id) {
    (void)u; (void)sfw; if (mock_count >= 8) return -1;
    mock_nodes[mock_count].id = id; mock_nodes[mock_count].start_ns = 0;
    snprintf(mock_nodes[mock_count].fw, 64, "%s", fw);
    return mock_count++;
}
static const char *m_fw_for_type(void *u, const char *t, const char **sfw) {
    (void)u; if (sfw) *sfw = NULL; return strcmp(t, "sky") == 0 ? "firmware/sky/x.sky" : NULL;
}

static sim_control_ops_t mock_ops = {
    .node_count = m_node_count, .describe = m_describe, .inject_serial = m_inject,
    .set_position = m_set_position, .reboot = m_reboot, .start = m_start,
    .remove = m_remove, .add = m_add, .firmware_for_type = m_fw_for_type,
};

static sim_control_t mock_ctl;
static shell_service_t sh;

static void mock_reset(void) {
    sim_runtime_init(&mock_sim);
    sim_eq_init(&mock_sim.event_queue);
    mock_count = 3;
    for (int i = 0; i < 3; i++) { mock_nodes[i].id = i + 1; mock_nodes[i].start_ns = 0; snprintf(mock_nodes[i].fw, 64, "fw%d", i + 1); }
    mock_inject_calls = mock_inject_bytes = mock_reboots = mock_removes = mock_moves = 0;
    mock_ops.inject_serial = m_inject;
    sim_control_init(&mock_ctl, &mock_sim, &mock_ops);
    memset(&sh, 0, sizeof(sh));
    sh.sim = &mock_sim; sh.ctl = &mock_ctl; sh.active = true; sh.interactive = false;
    sh.verbose = false; sh.next_at_id = 1; sh.default_expect_timeout_ns = 30000000000LL;
    sh.stop_when_done = false;
    shell_script_init(&sh);
}

static void emit_line(int idx, const char *line) {
    sim_observer_event_t ev = { .kind = SIM_OBS_MOTE_LOG_LINE, .time_ns = sim_runtime_now_ns(&mock_sim),
                                .mote_index = idx, .radio_idx = -1 };
    ev.u.log_line.line = line; ev.u.log_line.len = (int)strlen(line); ev.u.log_line.node_id = mock_nodes[idx].id;
    shell_script_on_log_line(&sh, idx, mock_nodes[idx].id, line, ev.time_ns);
}

static void advance(int64_t ns) { mock_sim.now_ns += ns; }

static const char *write_script(const char *name, const char *text) {
    static char path[256];
    snprintf(path, sizeof(path), "/tmp/csim_shell_test_%s_%d.cnsh", name, (int)getpid());
    FILE *f = fopen(path, "w"); fputs(text, f); fclose(f);
    return path;
}

static bool queue_has_pin_at(int64_t t) {
    /* Peek-only scan of the event queue for a TEST_ACTION at t. */
    sim_event_queue_t copy = mock_sim.event_queue;
    while (!sim_eq_empty(&copy)) {
        sim_event_t ev = sim_eq_pop(&copy);
        if (ev.kind == SIM_EV_TEST_ACTION && ev.time_ns == t) return true;
    }
    return false;
}

static void test_engine(void) {
    /* sendln + expect in one tick; match resumes; verdict pass. */
    mock_reset();
    const char *p = write_script("a",
        "# comment\n"
        "sendln 1 help\n"
        "expect 1 \"Shows this help\" 4s\n"
        "echo done\n"
        "pass\n");
    CHECK(shell_script_source(&sh, p) == 0, "source ok");
    mock_sim.now_ns = 1000;
    shell_script_tick(&sh);
    CHECK(mock_inject_calls == 1 && strcmp(mock_last_inject, "help\n") == 0, "sendln injected 'help\\n' (calls=%d, '%s')", mock_inject_calls, mock_last_inject);
    CHECK(sh.block == SHELL_BLOCK_EXPECT, "blocked on expect");
    CHECK(queue_has_pin_at(1000 + 4000000000LL), "expect timeout pinned");
    advance(500000);
    emit_line(1, "Shows this help");          /* wrong node */
    CHECK(!sh.matched, "other node does not match");
    emit_line(0, "xx Shows this help yy");
    CHECK(sh.matched, "substring match on node 1");
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_NONE && sh.passed && sh.finished && !sh.failed, "resumed, ran echo+pass (passed=%d failed=%d)", sh.passed, sh.failed);
    CHECK(sh.expect_pass == 1, "expect_pass=1");
    CHECK(sh.depth == 0, "script popped");
    unlink(p);

    /* expect timeout fails the script and aborts it. */
    mock_reset();
    p = write_script("b", "expect any \"never\" 1s\necho unreachable\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_EXPECT, "blocked");
    advance(999999999);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_EXPECT, "still blocked before deadline");
    advance(1);
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "timed out"), "timeout -> failed (%s)", sh.fail_reason);
    CHECK(sh.depth == 0 && sh.block == SHELL_BLOCK_NONE, "aborted");
    CHECK(shell_service_report(&sh, mock_sim.now_ns) == 1, "report exit code 1");
    unlink(p);

    /* sleep / wait-until pins + resume; nested source; at ordering. */
    mock_reset();
    const char *inner = write_script("inner", "echo inner\nmove 2 10 20\n");
    char outer_text[512];
    snprintf(outer_text, sizeof(outer_text),
             "at 3s reboot 1\nat 2s remove 3\nsleep 1s\nsource %s\nwait-until 5s\nassert nodes == 2\nassert node 3 removed\n", inner);
    p = write_script("outer", outer_text);
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_SLEEP && sh.block_deadline_ns == 1000000000LL, "sleep 1s");
    CHECK(sh.atq_count == 2 && queue_has_pin_at(3000000000LL) && queue_has_pin_at(2000000000LL), "at entries pinned");
    mock_sim.now_ns = 1000000000LL;
    shell_script_tick(&sh);
    CHECK(mock_moves == 1, "nested source ran (moves=%d)", mock_moves);
    CHECK(sh.block == SHELL_BLOCK_WAIT_UNTIL && sh.block_deadline_ns == 5000000000LL, "wait-until 5s");
    mock_sim.now_ns = 2000000000LL;
    shell_script_tick(&sh);
    CHECK(mock_removes == 1 && mock_reboots == 0 && sh.atq_count == 1, "at 2s ran remove only");
    mock_sim.now_ns = 3500000000LL;
    shell_script_tick(&sh);
    CHECK(mock_reboots == 1 && sh.atq_count == 0, "at 3s ran reboot");
    mock_sim.now_ns = 5000000000LL;
    shell_script_tick(&sh);
    CHECK(!sh.failed && sh.finished && sh.depth == 0, "asserts passed, script finished (%s)", sh.fail_reason);
    CHECK(shell_service_report(&sh, mock_sim.now_ns) == 0, "report exit 0");
    unlink(p); unlink(inner);

    /* Errors in a script fail it; a script left blocked is incomplete. */
    mock_reset();
    p = write_script("c", "send 9 hello\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "no node with id 9"), "unknown node in script -> fail (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    p = write_script("d", "expect 1 \"x\"\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(shell_service_report(&sh, 0) == 1 && strstr(sh.fail_reason, "did not complete"), "incomplete -> fail (%s)", sh.fail_reason);
    unlink(p);

    /* Stdin queue: lines wait behind a block; "!" runs immediate-safe ones. */
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "sleep 2s");
    shell_enqueue_line(&sh, "move 1 1 1");
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_SLEEP && mock_moves == 0 && sh.qcount == 1, "move queued behind sleep");
    shell_enqueue_line(&sh, "!move 2 2 2");
    CHECK(mock_moves == 0, "!move refused while blocked");
    shell_enqueue_line(&sh, "!pause");
    CHECK(sim_control_paused(&mock_ctl), "!pause ran immediately");
    sim_control_resume(&mock_ctl);
    mock_sim.now_ns = 2000000000LL;
    shell_script_tick(&sh);
    CHECK(mock_moves == 1 && sh.qcount == 0, "queued move ran after the sleep");

    /* fail-on, count, on, every. */
    mock_reset();
    p = write_script("e", "count \"tick\"\non 2 \"boom\" echo boom-seen\nfail-on \"FATAL\" any\nevery 1s echo tick\nsleep 10s\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    emit_line(0, "tick 1"); emit_line(2, "tick 2");
    CHECK(sh.watches[0].count == 2, "count watch counts (%d)", sh.watches[0].count);
    emit_line(0, "boom");
    CHECK(sh.trigger_count == 0, "on: wrong node ignored");
    emit_line(1, "boom");
    CHECK(sh.trigger_count == 1, "on: queued command");
    mock_sim.now_ns = 1000000000LL;
    shell_script_tick(&sh);
    CHECK(sh.trigger_count == 0 && sh.atq_count == 1 && sh.atq[0].at_ns == 2000000000LL, "every re-armed at 2s");
    emit_line(2, "some FATAL error");
    CHECK(sh.pending_fail, "fail-on pending from observer");
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "fail-on"), "fail-on applied at tick (%s)", sh.fail_reason);
    unlink(p);

    /* Partial serial consumption is retried by the control layer. */
    mock_reset();
    mock_ops.inject_serial = m_inject_half;
    sim_control_init(&mock_ctl, &mock_sim, &mock_ops);
    int c = sim_control_send(&mock_ctl, 1, (const uint8_t *)"abcdef", 6, SIM_CONTROL_RETRY);
    CHECK(c == 6 && sim_control_pending_len(&mock_ctl, 0) == 4, "partial consume queues the rest (pending=%d)", sim_control_pending_len(&mock_ctl, 0));
    sim_control_send_flush_pending(&mock_ctl);
    sim_control_send_flush_pending(&mock_ctl);
    CHECK(sim_control_pending_len(&mock_ctl, 0) == 0 && mock_inject_bytes == 6, "flushed in two more calls (bytes=%d)", mock_inject_bytes);
    /* add by mote-type name + id collision */
    mock_ops.inject_serial = m_inject;
    int idx = sim_control_add(&mock_ctl, "sky", NULL, 7, 1.0, 2.0, SIM_CONTROL_START);
    CHECK(idx == 3 && strcmp(mock_nodes[3].fw, "firmware/sky/x.sky") == 0 && mock_moves == 1, "add resolves type name, sets position");
    CHECK(sim_control_add(&mock_ctl, "sky", NULL, 7, 0, 0, 0) == -1, "duplicate id refused");
}

int run_shell_tests(int verbose) {
    g_verbose = verbose;
    printf("=== Shell tests ===\n");
    test_tokenize();
    test_time();
    test_selector();
    test_engine();
    printf("  %d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
