/*
 * shell_service — the Cooja-NG command shell and script engine as a kernel
 * service (docs/shell.md).
 *
 * One command stream feeds three inputs — interactive stdin (--shell; a
 * linenoise line editor on a TTY, plain lines from a pipe), script files
 * (--script FILE at start, `source FILE` later, nested), and the `at`
 * queue — into one executor whose commands act through sim_control.
 * Blocking commands (`expect`, `sleep`, `wait-until`) hold the stream
 * until a log line matches or a simulation instant is reached; the engine
 * is a state machine resumed from the service host's poll() at every slice
 * boundary, never a thread, so script-driven runs are deterministic.
 *
 * Wiring (test/test_mixed_multinode.c): attached right after the json-test
 * service — BEFORE the motes boot, so boot-time console lines route through
 * the shell's console mask like every other line; polled through
 * sim_service_poll_all; while the simulation is paused the loop calls
 * shell_service_pump_paused() so input is still read.
 *
 * Console lines: when the shell is active the runner's own per-line printf
 * is off and this service prints the identical "  %7.3f [Node id/KIND] text"
 * line for every node in the console mask (default: all on, all off under
 * -q), plus into any `log-file`.
 */
#ifndef SHELL_SERVICE_H
#define SHELL_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sim_service.h"
#include "sim_control.h"
#include "sim_event_queue.h"   /* SIM_EQ_MAX_NODES */
#include "linenoise.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHELL_LINE_MAX      1024
#define SHELL_QUEUE_MAX     32
#define SHELL_SOURCE_DEPTH  8
#define SHELL_ATQ_MAX       64
#define SHELL_LOGFILES_MAX  8
#define SHELL_WATCH_MAX     16
#define SHELL_TRIGGER_MAX   16
#define SHELL_PATTERN_MAX   256
#define SHELL_PATH_MAX      512
#define SHELL_REASON_MAX    512

typedef struct shell_source {
    FILE *f;
    char  path[SHELL_PATH_MAX];
    int   lineno;
} shell_source_t;

typedef enum shell_block {
    SHELL_BLOCK_NONE = 0,
    SHELL_BLOCK_EXPECT,
    SHELL_BLOCK_SLEEP,
    SHELL_BLOCK_WAIT_UNTIL,
    SHELL_BLOCK_RUN,        /* `run <dur>` / `step`: until the auto-pause */
} shell_block_t;

typedef struct shell_at_entry {
    int     id;
    int64_t at_ns;
    int64_t period_ns;     /* > 0: `every` — re-armed after each run */
    char    cmd[SHELL_LINE_MAX];
} shell_at_entry_t;

typedef struct shell_logfile {
    FILE   *f;
    char    path[SHELL_PATH_MAX];
    uint8_t mask[SIM_EQ_MAX_NODES];   /* by slot index */
} shell_logfile_t;

typedef enum shell_watch_kind {
    SHELL_WATCH_COUNT = 1,   /* `count "pat" [sel]`   → assert count   */
    SHELL_WATCH_FAIL,        /* `fail-on "pat" [sel]` → script fails   */
    SHELL_WATCH_RUN,         /* `on sel "pat" cmd`    → run cmd        */
} shell_watch_kind_t;

typedef struct shell_watch {
    shell_watch_kind_t kind;
    char    pattern[SHELL_PATTERN_MAX];
    bool    any;
    int     ids[SIM_EQ_MAX_NODES];
    int     nids;
    int     count;
    char    cmd[SHELL_LINE_MAX];
} shell_watch_t;

typedef struct shell_service {
    sim_runtime_t *sim;
    sim_control_t *ctl;
    bool active;            /* --shell or --script                         */
    bool interactive;       /* --shell: stdin is a command source          */
    bool tty;               /* stdin and stdout are a terminal             */
    bool verbose;           /* runner's -v/-q (console mask default, echo) */
    bool started;           /* first tick done (editor started lazily)     */

    /* linenoise multiplexing state (tty only). */
    struct linenoiseState ls;
    char   linebuf[SHELL_LINE_MAX];
    bool   editing;
    char   prompt[96];
    char   history_path[SHELL_PATH_MAX];
    int64_t prompt_ns;      /* sim time the prompt currently shows          */
    double  prompt_ms;      /* wall time of the last prompt refresh         */

    /* Raw stdin (non-tty). */
    char   inbuf[4096];
    int    inlen;
    bool   stdin_eof;

    /* Lines typed/piped but not yet executed (a script or block holds the
     * stream).  Ring buffer. */
    char   queue[SHELL_QUEUE_MAX][SHELL_LINE_MAX];
    int    qhead, qcount;
    bool   queue_warned;

    /* stdin poll gate (wall clock, see shell_service.c). */
    uint32_t iter;
    double   last_poll_ms;
    int64_t  last_poll_sim_ns;

    /* Console routing. */
    uint8_t console_mask[SIM_EQ_MAX_NODES];   /* by slot index */
    shell_logfile_t logfiles[SHELL_LOGFILES_MAX];
    int    logfile_count;

    /* Scheduled commands (`at`, `every`). */
    shell_at_entry_t atq[SHELL_ATQ_MAX];
    int    atq_count;
    int    next_at_id;

    /* Log-line watches and the commands they trigger (run at the tick). */
    shell_watch_t watches[SHELL_WATCH_MAX];
    int    watch_count;
    char   triggers[SHELL_TRIGGER_MAX][SHELL_LINE_MAX];
    int    trigger_count;

    /* Command stream: source stack (depth 0 = stdin) + blocking state. */
    shell_source_t stack[SHELL_SOURCE_DEPTH];
    int    depth;
    shell_block_t block;
    int64_t block_start_ns;
    int64_t block_deadline_ns;
    char   expect_pattern[SHELL_PATTERN_MAX];
    bool   expect_any;
    int    expect_ids[SIM_EQ_MAX_NODES];
    int    expect_n;
    bool   matched;
    char   matched_line[SHELL_PATTERN_MAX];
    int    matched_node;
    int64_t matched_ns;
    int64_t default_expect_timeout_ns;

    /* Verdict (docs/shell.md "Exit codes"). */
    bool   script_used;     /* a script ran or a verdict command was used  */
    bool   failed;
    char   fail_reason[SHELL_REASON_MAX];
    bool   passed;          /* `pass` seen                                 */
    bool   exited;          /* `exit`/`quit` executed                      */
    bool   finished;        /* root script reached its end                 */
    int    expect_pass, expect_fail;
    bool   pending_fail;    /* set from the observer, applied at the tick   */
    char   pending_fail_reason[SHELL_REASON_MAX];
    bool   stop_when_done;  /* --script without --shell: stop at pass/fail  */

    int64_t run_for_target_ns;   /* `run <dur>` in flight (for the note)   */
} shell_service_t;

/* Arm the service.  `interactive` = --shell (read stdin); `script_path` =
 * --script (may be NULL); `verbose` = the runner's flag (console mask
 * default + command echo).  Returns 0, or -1 if the script cannot be
 * opened (message printed). */
int  shell_service_start(shell_service_t *s, sim_runtime_t *sim,
                         sim_control_t *ctl, bool interactive,
                         const char *script_path, bool verbose);

static inline bool shell_service_active(const shell_service_t *s) {
    return s && s->active;
}
static inline bool shell_service_interactive(const shell_service_t *s) {
    return s && s->active && s->interactive;
}

/* Wait up to timeout_ms for input while the simulation is paused, then run
 * one engine tick (so `run`/`exit` typed at a paused prompt take effect). */
void shell_service_pump_paused(shell_service_t *s, int timeout_ms);

/* Non-blocking stdin check for the loop's pacing waits. */
void shell_service_poll_input(shell_service_t *s);

/* The loop paused the run (run-for horizon / step budget / duration). */
void shell_service_on_autopause(shell_service_t *s);

/* End-of-run: print "--- Script Results ---" when a script/verdict was
 * used and return the exit code (0/1). */
int  shell_service_report(shell_service_t *s, int64_t now_ns);

/* UI restart: drop the at-queue and abort any script; keep masks/files. */
void shell_service_on_restart(shell_service_t *s);

extern const sim_service_ops_t shell_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* SHELL_SERVICE_H */
