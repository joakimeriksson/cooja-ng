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
#include "sim_mote.h"
#include "arm_cpu.h"
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

/* Node 1 is an ARM node with 4 KB of SRAM at 0x20000000; the others have no
 * CPU interface (like MSP430/native nodes). */
static arm_cpu_t mock_cpu;
static uint8_t mock_sram[4096];
static void *m_get_interface(void *u, int idx, int iface) {
    (void)u;
    return (idx == 0 && iface == SIM_MOTE_IFACE_ARM_CPU) ? &mock_cpu : NULL;
}

static sim_control_ops_t mock_ops = {
    .node_count = m_node_count, .describe = m_describe, .inject_serial = m_inject,
    .set_position = m_set_position, .reboot = m_reboot, .start = m_start,
    .remove = m_remove, .add = m_add, .firmware_for_type = m_fw_for_type,
    .get_interface = m_get_interface,
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
    memset(&mock_cpu, 0, sizeof(mock_cpu));
    memset(mock_sram, 0, sizeof(mock_sram));
    mock_cpu.sram = mock_sram;
    mock_cpu.sram_base = 0x20000000u;
    mock_cpu.sram_end = 0x20000000u + sizeof(mock_sram);
    memset(&sh, 0, sizeof(sh));
    sh.sim = &mock_sim; sh.ctl = &mock_ctl; sh.active = true; sh.interactive = false;
    sh.verbose = false; sh.next_at_id = 1; sh.default_expect_timeout_ns = 30000000000LL;
    sh.stop_when_done = false;
    snprintf(sh.prompt_glob, sizeof(sh.prompt_glob), "#*> ");
    shell_script_init(&sh);
}

/* Feed console bytes of node slot idx to the engine (what the service's
 * UART-byte observer does), emitting a log line at each newline. */
static void emit_bytes(int idx, const char *text) {
    char line[256]; int n = 0;
    for (const char *p = text; *p; p++) {
        shell_script_on_uart_byte(&sh, idx, (uint8_t)*p, sim_runtime_now_ns(&mock_sim));
        if (*p == '\n') {
            line[n] = '\0';
            shell_script_on_log_line(&sh, idx, mock_nodes[idx].id, line, sim_runtime_now_ns(&mock_sim));
            n = 0;
        } else if (n < 255) {
            line[n++] = *p;
        }
    }
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

/* --- review fixes ----------------------------------------------------------- */

static void test_unquote_rest(void) {
    char out[128]; char err[64];
    int n = shell_unquote_rest("a  b   ", out, sizeof(out), err, sizeof(err));
    CHECK(n == 4 && strcmp(out, "a  b") == 0, "inner spacing kept, trailing trimmed ('%s')", out);
    n = shell_unquote_rest("\"x  \"", out, sizeof(out), err, sizeof(err));
    CHECK(n == 3 && strcmp(out, "x  ") == 0, "quoted trailing spaces kept ('%s')", out);
    n = shell_unquote_rest("help # comment", out, sizeof(out), err, sizeof(err));
    CHECK(strcmp(out, "help") == 0, "comment stripped ('%s')", out);
    n = shell_unquote_rest("a#b \"#c\"", out, sizeof(out), err, sizeof(err));
    CHECK(strcmp(out, "a#b #c") == 0, "# inside a word or quotes is text ('%s')", out);
    n = shell_unquote_rest("tab\\there", out, sizeof(out), err, sizeof(err));
    CHECK(strcmp(out, "tab\\there") == 0 || strcmp(out, "tab\there") == 0, "escape decoded");
    n = shell_unquote_rest("'lit\\n'", out, sizeof(out), err, sizeof(err));
    CHECK(strcmp(out, "lit\\n") == 0, "single quotes literal ('%s')", out);
    n = shell_unquote_rest("\"open", out, sizeof(out), err, sizeof(err));
    CHECK(n == -1, "unterminated quote -> error");
}

static int m_inject_none(void *u, int idx, const uint8_t *b, int n) {
    (void)u; (void)idx; (void)b; (void)n; mock_inject_calls++; return 0;
}

static void test_review_fixes(void) {
    const char *p;

    /* A typo typed at the prompt beside a running script does not fail it. */
    mock_reset();
    sh.interactive = true; sh.editor = true; sh.stdin_tty = true;          /* "!" runs immediately */
    p = write_script("f1", "sleep 1s\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    shell_enqueue_line(&sh, "!stauts");
    CHECK(!sh.failed, "!typo beside a script does not fail it");
    mock_sim.now_ns = 1000000000LL;
    shell_script_tick(&sh);
    CHECK(sh.passed && !sh.failed, "script still passes");
    unlink(p);

    /* ...but an error on the script's own line still does. */
    mock_reset();
    p = write_script("f2", "stauts\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "f2_"), "script's own error fails it, blaming the file (%s)", sh.fail_reason);
    unlink(p);

    /* Blocking commands are refused from at/every/on. */
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "at +1s sleep 1s");
    shell_enqueue_line(&sh, "on any \"x\" expect 1 \"y\"");
    shell_enqueue_line(&sh, "every 1s run 5ms");
    shell_enqueue_line(&sh, "at +1s step");
    shell_enqueue_line(&sh, "at +1s run");
    shell_script_tick(&sh);
    CHECK(sh.atq_count == 1 && sh.watch_count == 0, "only the non-blocking 'at +1s run' was scheduled (atq=%d watches=%d)", sh.atq_count, sh.watch_count);
    CHECK(!sh.failed, "refusals typed at the prompt do not set a verdict");

    /* at list / at clear, and the atq / atrm aliases. */
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "at 5s echo a");
    shell_enqueue_line(&sh, "at 6s echo b");
    shell_enqueue_line(&sh, "every 1s echo c");
    shell_enqueue_line(&sh, "at list");
    shell_enqueue_line(&sh, "at clear 1");
    shell_enqueue_line(&sh, "atrm 2");
    shell_enqueue_line(&sh, "atq");
    shell_enqueue_line(&sh, "at clear 9");
    shell_script_tick(&sh);
    CHECK(sh.atq_count == 1 && sh.atq[0].id == 3, "at clear and atrm cancel by id (%d left)", sh.atq_count);
    CHECK(!sh.failed, "at clear of a missing id is a prompt error only");
    shell_enqueue_line(&sh, "at clear all");
    shell_script_tick(&sh);
    CHECK(sh.atq_count == 0, "at clear all empties the queue");
    mock_reset();
    p = write_script("f3", "at +100ms sleep 10s\nsleep 1s\necho after\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "block"), "blocking at inside a script fails it at definition (%s)", sh.fail_reason);
    unlink(p);

    /* at/on errors are attributed to where they were scheduled. */
    mock_reset();
    p = write_script("f4", "at +1s send 9 x\nsleep 2s\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    mock_sim.now_ns = 1000000000LL;
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "at #1") && strstr(sh.fail_reason, "f4_"), "failing at names itself and its script line (%s)", sh.fail_reason);
    unlink(p);

    /* Exit verdict: exit while blocked fails; exit inside a script is a finish. */
    mock_reset();
    sh.interactive = true; sh.editor = true; sh.stdin_tty = true;
    shell_enqueue_line(&sh, "expect 1 \"never\" 10s");
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_EXPECT, "blocked on expect");
    shell_enqueue_line(&sh, "!exit");
    CHECK(sim_runtime_stop_requested(&mock_sim), "!exit requested the stop");
    CHECK(shell_service_report(&sh, 0) == 1 && strstr(sh.fail_reason, "did not complete"), "exit while blocked fails the verdict (%s)", sh.fail_reason);
    mock_reset();
    p = write_script("f5", "echo hi\nexit\necho unreachable\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(shell_service_report(&sh, 0) == 0, "exit inside a script file passes (%s)", sh.fail_reason);
    unlink(p);

    /* Deadlock: paused and blocked with nothing able to resume. */
    mock_reset();
    shell_enqueue_line(&sh, "sleep 1s");   /* tty false, interactive false */
    shell_script_tick(&sh);
    sim_control_pause(&mock_ctl);
    shell_service_pump_paused(&sh, 0);
    CHECK(sh.failed && strstr(sh.fail_reason, "deadlock"), "paused + blocked + no input = deadlock failure (%s)", sh.fail_reason);
    mock_reset();
    shell_enqueue_line(&sh, "sleep 1s");
    shell_script_tick(&sh);
    sh.external_resume = true;             /* e.g. the web UI is up */
    sim_control_pause(&mock_ctl);
    shell_service_pump_paused(&sh, 0);
    CHECK(!sh.failed, "not a deadlock when something outside can resume");

    /* --script alone: EOF waits for pending at entries, then ends the run. */
    mock_reset();
    sh.stop_when_done = true;
    p = write_script("f6", "at 2s echo fired\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.finished && !sim_runtime_stop_requested(&mock_sim), "EOF with a pending at keeps running");
    mock_sim.now_ns = 2000000000LL;
    shell_script_tick(&sh);
    CHECK(sh.atq_count == 0 && sim_runtime_stop_requested(&mock_sim), "run ends once the at fired");
    unlink(p);
    mock_reset();
    sh.stop_when_done = true;
    p = write_script("f7", "at 2s echo fired\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sim_runtime_stop_requested(&mock_sim), "pass ends at once despite pending at");
    unlink(p);

    /* External clock source: time commands are refused. */
    static const sim_clock_source_t fake_clock;
    mock_reset();
    mock_sim.clock_source = &fake_clock;
    sh.interactive = true;
    shell_enqueue_line(&sh, "pause");
    shell_enqueue_line(&sh, "sleep 1s");
    shell_enqueue_line(&sh, "at +1s echo x");
    shell_script_tick(&sh);
    CHECK(!sim_control_paused(&mock_ctl) && sh.block == SHELL_BLOCK_NONE && sh.atq_count == 0,
          "pause/sleep/at refused under an external clock");
    mock_sim.clock_source = NULL;

    /* send keeps spacing; truncation is an error, not silent. */
    mock_reset();
    p = write_script("f8", "sendln 1 a  b   # comment\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(strcmp(mock_last_inject, "a  b\n") == 0, "sendln keeps inner spacing ('%s')", mock_last_inject);
    unlink(p);
    mock_reset();
    mock_ops.inject_serial = m_inject_none;
    sim_control_init(&mock_ctl, &mock_sim, &mock_ops);
    char big[700] = "send 1 ";
    memset(big + 7, 'x', 600); big[607] = '\0';
    sh.interactive = true;
    shell_enqueue_line(&sh, big);
    shell_script_tick(&sh);
    CHECK(sim_control_pending_len(&mock_ctl, 0) == SIM_CONTROL_PENDING_MAX, "pending buffer filled to its limit");
    mock_ops.inject_serial = m_inject;

    /* on trigger overflow is counted and reported, then reset. */
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "on any \"hit\" echo got-it");
    shell_script_tick(&sh);
    for (int i = 0; i < SHELL_TRIGGER_MAX + 4; i++) emit_line(0, "hit");
    CHECK(sh.trigger_count == SHELL_TRIGGER_MAX && sh.triggers_dropped == 4, "overflow counted (%d dropped)", sh.triggers_dropped);
    shell_script_tick(&sh);
    CHECK(sh.trigger_count == 0 && sh.triggers_dropped == 0, "reported and reset at the tick");
}

/* --- piped stdin: a burst larger than the queue is paced, not dropped --- */

const char *shell_dequeue_line(shell_service_t *s, char *buf, size_t len);

static void test_stdin_burst(void) {
    int fds[2];
    if (pipe(fds) != 0) { CHECK(0, "pipe()"); return; }
    int saved = dup(STDIN_FILENO);
    dup2(fds[0], STDIN_FILENO);
    close(fds[0]);

    const int N = SHELL_QUEUE_MAX * 4;      /* far more than one queue-full */
    char line[64];
    for (int i = 0; i < N; i++) {
        int n = snprintf(line, sizeof(line), "echo l%d\n", i);
        if (write(fds[1], line, (size_t)n) != n) break;
    }
    close(fds[1]);                          /* EOF, with no explicit `exit` */

    mock_reset();
    sh.interactive = true;
    sh.sync_stdin = true;

    char buf[SHELL_LINE_MAX];
    int got = 0, order_ok = 1, guard = 0;
    const char *last = NULL;
    for (;;) {
        if (sh.qcount == 0) shell_read_stdin_sync(&sh);
        const char *l = shell_dequeue_line(&sh, buf, sizeof(buf));
        if (!l) break;
        if (guard++ > N + 8) break;
        if (strcmp(l, "exit") == 0) { last = "exit"; break; }
        int idx = -1;
        if (sscanf(l, "echo l%d", &idx) != 1 || idx != got) order_ok = 0;
        got++;
    }
    dup2(saved, STDIN_FILENO);
    close(saved);

    CHECK(got == N, "piped burst of %d lines is fully delivered (%d)", N, got);
    CHECK(order_ok, "piped burst keeps its order");
    CHECK(last && strcmp(last, "exit") == 0, "EOF still delivers the implied exit");
    CHECK(!sh.queue_warned, "no line is dropped");
}

/* --- exit codes say what kind of failure it was ------------------------- */

static void test_exit_codes(void) {
    const char *p;
    mock_reset();
    sh.stop_when_done = true;
    p = write_script("x1", "assert nodes == 9\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && sh.fail_code == SHELL_EXIT_ASSERT,
          "a false assert is an assertion failure (%d)", sh.fail_code);
    unlink(p);

    mock_reset();
    sh.stop_when_done = true;
    p = write_script("x2", "stauts\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && sh.fail_code == SHELL_EXIT_INVALID,
          "an unknown command is an invalid request (%d)", sh.fail_code);
    unlink(p);

    mock_reset();
    sh.stop_when_done = true;
    p = write_script("x3", "fail no good\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.failed && sh.fail_code == SHELL_EXIT_ASSERT,
          "`fail` is an assertion failure (%d)", sh.fail_code);
    unlink(p);

    mock_reset();
    p = write_script("x4", "pass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(shell_service_report(&sh, 0) == SHELL_EXIT_PASS, "a pass reports 0");
    unlink(p);
}

/* --- the -t horizon is not cleared by pause/run ------------------------- */

static void test_horizon(void) {
    mock_reset();
    sim_control_set_horizon(&mock_ctl, 2000000000LL);   /* -t 2000 */
    CHECK(sim_control_slice_cap(&mock_ctl, INT64_MAX) == 2000000000LL,
          "horizon caps the slice");
    sim_control_run_for(&mock_ctl, 500000000LL);        /* run 500ms */
    sim_control_pause(&mock_ctl);
    sim_control_resume(&mock_ctl);
    CHECK(sim_control_slice_cap(&mock_ctl, INT64_MAX) == 2000000000LL,
          "pause/run/run-for leave the horizon alone");
    mock_sim.now_ns = 1999999999LL;
    CHECK(!sim_control_after_pump(&mock_ctl), "no pause before the horizon");
    mock_sim.now_ns = 2000000000LL;
    CHECK(sim_control_after_pump(&mock_ctl) && sim_control_horizon_hit(&mock_ctl),
          "the horizon pauses once it is reached");
    sim_control_resume(&mock_ctl);
    mock_sim.now_ns = 3000000000LL;
    CHECK(!sim_control_after_pump(&mock_ctl), "one-shot: it does not fire again");
}

static void test_glob(void) {
    CHECK(shell_glob_match("#*> ", "#f4ce.3601.f1f4.0001> "), "contiki prompt");
    CHECK(!shell_glob_match("#*> ", "#f4ce.3601> Command"), "prompt followed by text");
    CHECK(!shell_glob_match("#*> ", "'> help': Shows this help"), "help line is not a prompt");
    CHECK(shell_glob_match("> ", "> ") && !shell_glob_match("> ", ">  "), "exact");
    CHECK(shell_glob_match("uart:~$ ", "uart:~$ "), "zephyr prompt");
    CHECK(shell_glob_match("*", "") && shell_glob_match("a*b*c", "aXbYYc") && !shell_glob_match("a*b*c", "aXbYY"), "multi-star");
}

static void test_cmd(void) {
    const char *p;

    /* Pass: line sent, output checked, released at the prompt. */
    mock_reset();
    p = write_script("c1", "cmd -f \"not found\" 1 help me\necho after\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(strcmp(mock_last_inject, "help me\n") == 0, "cmd sends text + newline ('%s')", mock_last_inject);
    CHECK(sh.block == SHELL_BLOCK_CMD, "blocked on the prompt");
    emit_bytes(0, "#0001.0001> ");
    shell_script_tick(&sh);
    CHECK(!sh.cmd_prompt_seen && sh.block == SHELL_BLOCK_CMD, "a prompt is not confirmed before the quiet window");
    CHECK(queue_has_pin_at(sim_runtime_now_ns(&mock_sim) + SHELL_PROMPT_QUIET_NS), "quiet window pinned");
    advance(SHELL_PROMPT_QUIET_NS);
    shell_script_tick(&sh);
    CHECK(sh.cmd_prompt_seen && sh.passed, "prompt + quiet window releases the cmd");
    unlink(p);
    mock_reset();
    p = write_script("c1b", "cmd -e \"Shows\" 1 help\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    emit_bytes(0, "#0001.0001> Shows this help\n");      /* prompt followed by text */
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(!sh.cmd_prompt_seen && sh.cmd_expect_seen, "prompt followed by output is not the prompt; line matched");
    emit_bytes(0, "'> reboot': Reboot\n");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(!sh.cmd_prompt_seen, "'> ' inside a help line is not a prompt");
    emit_bytes(1, "#0002.0002> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(!sh.cmd_prompt_seen, "another node's prompt does not count");
    emit_bytes(0, "#0001.0001> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.cmd_prompt_seen && sh.cmd_lines == 2, "prompt seen after 2 lines (%d)", sh.cmd_lines);
    CHECK(sh.passed && !sh.failed && sh.cmd_pass == 1, "script passes");
    unlink(p);

    /* -e not printed / -f printed / no prompt: each fails. */
    mock_reset();
    p = write_script("c2", "cmd -e \"wanted\" 1 x\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    emit_bytes(0, "something else\n#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "was not printed"), "-e missing fails (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    p = write_script("c3", "cmd -f \"not found\" 1 x\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    emit_bytes(0, "Command not found.\n#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "printed \"not found\""), "-f seen fails (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    p = write_script("c4", "cmd -t 500ms 1 x\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    CHECK(queue_has_pin_at(500000000LL), "timeout pinned");
    mock_sim.now_ns = 500000000LL;
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "no prompt"), "no prompt within the timeout fails (%s)", sh.fail_reason);
    unlink(p);

    /* set prompt; empty cmd waits for a prompt; refused from at; errors. */
    mock_reset();
    p = write_script("c5", "set prompt \"uart:~$ \"\ncmd 1\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    CHECK(strcmp(mock_last_inject, "\n") == 0, "empty cmd sends a bare newline");
    emit_bytes(0, "#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(!sh.cmd_prompt_seen, "old glob no longer matches");
    emit_bytes(0, "\nuart:~$ ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.passed, "custom prompt releases the cmd");
    unlink(p);
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "at +1s cmd 1 help");
    shell_enqueue_line(&sh, "cmd 1,2 help");
    shell_enqueue_line(&sh, "cmd -x 1 help");
    shell_enqueue_line(&sh, "cmd -t");
    shell_enqueue_line(&sh, "console 1");      /* not a terminal */
    shell_script_tick(&sh);
    CHECK(sh.atq_count == 0 && sh.block == SHELL_BLOCK_NONE && mock_inject_calls == 0 && !sh.console_mode,
          "at cmd, node lists, bad options and console without a tty are refused");
}

static const char *t_lookup(void *u, const char *name) {
    (void)u;
    if (!strcmp(name, "x")) return "42";
    if (!strcmp(name, "sp")) return "a  b";
    return NULL;
}

static void test_expand(void) {
    char out[256]; char err[96];
    int n = shell_expand_vars("echo $x ${x}y $$x \\$x '$x' \"$x\" $ $1 # $nope", out, sizeof(out), t_lookup, NULL, err, sizeof(err));
    CHECK(n > 0 && !strcmp(out, "echo 42 42y $x \\$x '$x' \"42\" $ $1 # $nope"), "expansion rules ('%s')", out);
    n = shell_expand_vars("send 1 $sp", out, sizeof(out), t_lookup, NULL, err, sizeof(err));
    CHECK(n > 0 && !strcmp(out, "send 1 a  b"), "value substituted as text");
    n = shell_expand_vars("echo $missing", out, sizeof(out), t_lookup, NULL, err, sizeof(err));
    CHECK(n == -1 && strstr(err, "undefined variable 'missing'"), "undefined -> error (%s)", err);
    n = shell_expand_vars("echo ${x", out, sizeof(out), t_lookup, NULL, err, sizeof(err));
    CHECK(n == -1, "unterminated ${ -> error");
    char *argv[8]; char st[64];
    int argc = shell_tokenize("echo \\$x", argv, NULL, 8, st, sizeof(st), err, sizeof(err));
    CHECK(argc == 2 && !strcmp(argv[1], "$x"), "\\$ decodes to a literal $");
}

static void test_node_commands(void) {
    const char *p;

    /* var / capture / expect options / expect-not / cmd -c / assert var */
    mock_reset();
    sh.verbose = false;
    p = write_script("v1",
        "var name hello  there\n"
        "assert var name == \"hello  there\"\n"
        "capture num 1 \"value=([0-9]+)\" 1s\n"
        "assert var num == 17\n"
        "assert var num > 10\n"
        "expect -re -n 2 -c last 2 \"^tick ([0-9])$\" 1s\n"
        "assert var last == 2\n"
        "sendln 1 $name $$literal\n"
        "expect-not any \"ERROR\" 100ms\n"
        "cmd -c ip \"(fe80::[0-9a-f:]+)\" 1 ip-addr\n"
        "assert var ip == fe80::1\n"
        "pass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_EXPECT && !sh.failed, "capture blocks (%s)", sh.fail_reason);
    emit_line(0, "value=abc");
    emit_line(0, "value=17 more");
    shell_script_tick(&sh);
    CHECK(!strcmp(shell_var_get(&sh, "num") ? shell_var_get(&sh, "num") : "", "17"), "capture stored group 1");
    CHECK(sh.block == SHELL_BLOCK_EXPECT && sh.expect_needed == 2, "-n 2 armed");
    emit_line(1, "tick 1");
    emit_line(0, "tick 9");                   /* node 1: not selected */
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_EXPECT, "one of two matches is not enough");
    emit_line(1, "tick 2");
    shell_script_tick(&sh);
    CHECK(strcmp(mock_last_inject, "hello  there $literal\n") == 0, "expanded send ('%s')", mock_last_inject);
    CHECK(sh.block == SHELL_BLOCK_EXPECT_NOT, "expect-not blocks");
    emit_line(2, "all fine");
    advance(100000000LL);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_CMD, "expect-not passed, cmd blocks (%s)", sh.fail_reason);
    emit_bytes(0, "IPv6 addresses:\n  fe80::1\n#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.passed && !sh.failed, "script passes (%s)", sh.fail_reason);
    unlink(p);

    mock_reset();
    p = write_script("v2", "expect-not 1 \"ERROR\" 1s\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    emit_line(0, "fatal ERROR here");
    shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "expect-not"), "expect-not match fails (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    p = write_script("v3", "cmd -c v \"nothing ([0-9]+)\" 1 x\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    emit_bytes(0, "other\n#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "nothing to capture"), "cmd -c without a match fails (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "expect -re 1 \"(unclosed\"");
    shell_enqueue_line(&sh, "echo $nope");
    shell_enqueue_line(&sh, "var 1bad x");
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_NONE && sh.var_count == 0, "bad regex, undefined var, bad name are errors");

    /* sendfile: each line a cmd, in order. */
    mock_reset();
    char data[256];
    snprintf(data, sizeof(data), "/tmp/csim_shell_test_sendfile_%d.txt", (int)getpid());
    FILE *f = fopen(data, "w"); fputs("first line\n  second  line\n", f); fclose(f);
    char text[400];
    snprintf(text, sizeof(text), "sendfile -t 1s 1 %s\necho done\npass\n", data);
    p = write_script("v4", text);
    shell_script_source(&sh, p); shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_CMD && !strcmp(mock_last_inject, "first line\n"), "first line sent ('%s')", mock_last_inject);
    emit_bytes(0, "#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_CMD && !strcmp(mock_last_inject, "  second  line\n"), "second line sent verbatim ('%s')", mock_last_inject);
    emit_bytes(0, "#1> ");
    advance(SHELL_PROMPT_QUIET_NS); shell_script_tick(&sh);
    CHECK(sh.passed && sh.cmd_pass == 2, "sendfile done, script passes (%s)", sh.fail_reason);
    unlink(p); unlink(data);
}

static void test_arm_inspection(void) {
    const char *p;
    mock_reset();
    mock_cpu.reg[ARM_PC] = 0x1234;
    mock_cpu.tz_enabled = true;
    mock_sram[0x10] = 0x78; mock_sram[0x11] = 0x56; mock_sram[0x12] = 0x34; mock_sram[0x13] = 0x12;
    p = write_script("a1",
        "mem -w -c w 1 0x20000010 1\n"
        "assert var w == 0x12345678\n"
        "assert mem 1 0x20000010 == 0x12345678\n"
        "mem -w 1 0x20000020 = 0xcafebabe 7\n"
        "assert mem 1 0x20000024 == 7\n"
        "mem 1 0x20000020 = 0x11\n"
        "assert mem 1 0x20000020 == 0xcafeba11\n"
        "reg -c pcv 1 pc\n"
        "assert var pcv == 0x00001234\n"
        "reg 1 r3 = 0x55\n"
        "reg 1 pc = 0x2001\n"
        "tz 1\n"
        "faults 1\n"
        "expect-fault 1 securefault,hardfault 5ms\n"
        "pass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(!sh.failed, "mem/reg/assert mem on an ARM node (%s)", sh.fail_reason);
    CHECK(mock_cpu.reg[3] == 0x55 && mock_cpu.reg[ARM_PC] == 0x2000, "register writes (pc Thumb bit dropped)");
    CHECK(sh.block == SHELL_BLOCK_FAULT, "expect-fault blocks");
    CHECK(queue_has_pin_at(sim_runtime_now_ns(&mock_sim) + SHELL_MS_TO_NS), "fault poll pinned 1 ms ahead");
    mock_cpu.exc_entry_count[6]++;          /* a UsageFault: not selected */
    advance(SHELL_MS_TO_NS); shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_FAULT, "unselected fault kind ignored");
    mock_cpu.exc_entry_count[7]++;
    mock_cpu.last_fault_exc = 7;
    advance(SHELL_MS_TO_NS); shell_script_tick(&sh);
    CHECK(sh.passed && !sh.failed, "SecureFault releases expect-fault (%s)", sh.fail_reason);
    unlink(p);

    mock_reset();
    p = write_script("a2", "expect-fault 1 any 2ms\npass\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    advance(2 * SHELL_MS_TO_NS); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "took no fault"), "no fault within the timeout fails (%s)", sh.fail_reason);
    unlink(p);

    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "mem 2 0x20000000 4");       /* node 2 has no ARM CPU */
    shell_enqueue_line(&sh, "reg 1 cpsr = 1");
    shell_enqueue_line(&sh, "expect-fault 1 nosuchfault");
    shell_enqueue_line(&sh, "sym 1 main");               /* firmware "fw1" does not exist */
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_NONE && !sh.failed, "non-ARM node, bad register, bad fault kind, missing symbol are errors");
    CHECK(shell_line_blocks("expect-fault 1") && shell_line_blocks("capture v 1 \"x\"") &&
          shell_line_blocks("sendfile 1 f") && shell_line_blocks("expect-not 1 \"x\" 1s") &&
          !shell_line_blocks("mem 1 0x0"), "blocking classification");
}

static void test_workflow(void) {
    const char *p;

    /* repeat / if / else / end, nested, with a loop variable. */
    mock_reset();
    p = write_script("w1",
        "var acc x\n"
        "repeat 3 i\n"
        "  if var i == 2\n"
        "    var acc ${acc}T\n"
        "  else\n"
        "    repeat 2\n"
        "      var acc ${acc}e\n"
        "    end\n"
        "  end\n"
        "end\n"
        "repeat 0\n"
        "  fail \"repeat 0 ran\"\n"
        "end\n"
        "if nodes == 99\n"
        "  fail \"false if ran\"\n"
        "end\n"
        "assert var acc == xeeTee\n"
        "assert var i == 3\n"
        "pass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.passed && !sh.failed, "nested repeat/if/else (%s)", sh.fail_reason);
    unlink(p);

    /* A blocking command inside a loop resumes the loop. */
    mock_reset();
    p = write_script("w2", "repeat 2 k\n  sleep 1s\nend\nassert var k == 2\npass\n");
    shell_script_source(&sh, p);
    shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_SLEEP, "first sleep");
    advance(1000000000LL); shell_script_tick(&sh);
    CHECK(sh.block == SHELL_BLOCK_SLEEP, "second sleep after looping back");
    advance(1000000000LL); shell_script_tick(&sh);
    CHECK(sh.passed && !sh.failed, "loop with blocking body (%s)", sh.fail_reason);
    unlink(p);

    /* Structure errors. */
    mock_reset();
    p = write_script("w3", "repeat 2\necho x\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "missing `end`"), "missing end (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    p = write_script("w4", "else\n");
    shell_script_source(&sh, p); shell_script_tick(&sh);
    CHECK(sh.failed && strstr(sh.fail_reason, "else without if"), "else without if (%s)", sh.fail_reason);
    unlink(p);
    mock_reset();
    sh.interactive = true;
    shell_enqueue_line(&sh, "repeat 2");
    shell_script_tick(&sh);
    CHECK(!sh.failed, "repeat at the prompt is an error, not a verdict");

    /* on --once, on list / on clear, history ring, exit status, transcript. */
    mock_reset();
    sh.interactive = true;
    sh.hist = calloc(SHELL_HISTORY_LINES, sizeof(*sh.hist));
    shell_enqueue_line(&sh, "on --once any \"boom\" echo once");
    shell_enqueue_line(&sh, "count \"boom\" any");
    shell_script_tick(&sh);
    CHECK(sh.watch_count == 2 && sh.watches[0].once, "--once watch added");
    emit_line(0, "boom"); emit_line(0, "boom");
    CHECK(sh.trigger_count == 1, "--once fires once (%d)", sh.trigger_count);
    shell_script_tick(&sh);
    CHECK(sh.watch_count == 1 && sh.watches[0].kind == SHELL_WATCH_COUNT, "fired --once watch removed");
    shell_enqueue_line(&sh, "on list");
    shell_enqueue_line(&sh, "on clear 1");
    shell_script_tick(&sh);
    CHECK(sh.watch_count == 0, "on clear");
    for (int i = 0; i < SHELL_HISTORY_LINES + 5; i++) {
        char l[32]; snprintf(l, sizeof(l), "line %d", i);
        /* the service's observer fills the ring; mimic it */
        int slot = (sh.hist_head + sh.hist_count) % SHELL_HISTORY_LINES;
        if (sh.hist_count < SHELL_HISTORY_LINES) sh.hist_count++; else sh.hist_head = (sh.hist_head + 1) % SHELL_HISTORY_LINES;
        sh.hist[slot].node_id = 1 + (i % 2); snprintf(sh.hist[slot].text, sizeof(sh.hist[slot].text), "%s", l);
    }
    CHECK(sh.hist_count == SHELL_HISTORY_LINES && !strcmp(sh.hist[sh.hist_head].text, "line 5"), "ring keeps the newest %d lines", SHELL_HISTORY_LINES);
    char tpath[128];
    snprintf(tpath, sizeof(tpath), "/tmp/csim_shell_test_transcript_%d.cnsh", (int)getpid());
    unlink(tpath);
    char tcmd[200]; snprintf(tcmd, sizeof(tcmd), "transcript %s", tpath);
    shell_enqueue_line(&sh, tcmd);
    shell_enqueue_line(&sh, "echo one");
    shell_enqueue_line(&sh, "move 1 2 3");
    shell_enqueue_line(&sh, "transcript off");
    shell_enqueue_line(&sh, "echo not recorded");
    shell_enqueue_line(&sh, "exit");
    shell_script_tick(&sh);
    FILE *tf = fopen(tpath, "r");
    char tbuf[512] = ""; size_t tn = tf ? fread(tbuf, 1, sizeof(tbuf) - 1, tf) : 0; tbuf[tn] = 0;
    if (tf) fclose(tf);
    CHECK(strstr(tbuf, "echo one\nmove 1 2 3\n") && !strstr(tbuf, "not recorded") && !strstr(tbuf, "transcript off") && !strstr(tbuf, "\ntranscript /"),
          "transcript records typed lines ('%s')", tbuf);
    unlink(tpath);
    CHECK(shell_service_report(&sh, 0) == 0, "exit ends the run");
    free(sh.hist); sh.hist = NULL;
}

int run_shell_tests(int verbose) {
    g_verbose = verbose;
    printf("=== Shell tests ===\n");
    test_tokenize();
    test_time();
    test_selector();
    test_engine();
    test_unquote_rest();
    test_review_fixes();
    test_stdin_burst();
    test_horizon();
    test_exit_codes();
    test_glob();
    test_cmd();
    test_expand();
    test_node_commands();
    test_arm_inspection();
    test_workflow();
    printf("  %d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
