/*
 * shell_service — I/O half of the Cooja-NG shell (see
 * include/sim/shell_service.h).  Terminal/pipe input, prompt, output
 * routing (console mask + log files), the service-host glue, the
 * end-of-run report.  Command bodies live in shell_commands.c, the
 * command stream / script engine in shell_script.c.
 */
#include "shell_internal.h"
#include "sim_runtime.h"

#include <errno.h>
#include <poll.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* --- wall clock (input pacing only; never influences the simulation) --- */

static double wall_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* --- prompt + output ---------------------------------------------------- */

static void build_prompt(shell_service_t *s) {
    int64_t now = sim_runtime_now_ns(s->sim);
    const char *state = "";
    if (sim_control_paused(s->ctl)) state = " [paused]";
    else if (s->block == SHELL_BLOCK_EXPECT) state = " [expect]";
    else if (s->block == SHELL_BLOCK_SLEEP) state = " [sleep]";
    else if (s->block == SHELL_BLOCK_WAIT_UNTIL) state = " [wait]";
    else if (s->block == SHELL_BLOCK_RUN) state = " [run]";
    else if (s->depth > 0) state = " [script]";
    snprintf(s->prompt, sizeof(s->prompt), "cooja %.3fs%s> ",
             (double)now / 1e9, state);
    s->prompt_ns = now;
    s->ls.prompt = s->prompt;
    s->ls.plen = strlen(s->prompt);
}

static void edit_begin(shell_service_t *s) {
    if (!s->tty || s->editing) return;
    build_prompt(s);
    if (linenoiseEditStart(&s->ls, -1, -1, s->linebuf, sizeof(s->linebuf),
                           s->prompt) == 0) {
        s->editing = true;
        s->prompt_ms = wall_ms();
    } else {
        /* Not a usable terminal after all: fall back to plain lines. */
        s->tty = false;
    }
}

static void edit_end(shell_service_t *s) {
    if (!s->editing) return;
    linenoiseEditStop(&s->ls);
    s->editing = false;
}

void shell_out(shell_service_t *s, const char *fmt, ...) {
    va_list ap;
    if (s->editing) linenoiseHide(&s->ls);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
    if (s->editing) {
        build_prompt(s);
        linenoiseShow(&s->ls);
    }
}

const char *shell_origin(shell_service_t *s, char *buf, size_t len) {
    if (s->depth > 0) {
        const shell_source_t *src = &s->stack[s->depth - 1];
        const char *base = strrchr(src->path, '/');
        snprintf(buf, len, "%s:%d", base ? base + 1 : src->path, src->lineno);
    } else {
        snprintf(buf, len, "stdin");
    }
    return buf;
}

void shell_error(shell_service_t *s, const char *fmt, ...) {
    char msg[SHELL_REASON_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    char origin[64];
    shell_origin(s, origin, sizeof(origin));
    shell_out(s, "error: %s (%s)\n", msg, origin);
    if (shell_in_script(s)) {
        char reason[SHELL_REASON_MAX];
        snprintf(reason, sizeof(reason), "%s: %.*s", origin,
                 (int)(sizeof(reason) - sizeof(origin) - 3), msg);
        shell_script_fail(s, reason);
    }
}

/* --- console routing ---------------------------------------------------- */

static void console_line(shell_service_t *s, int idx, int node_id,
                         const char *line, int64_t ns) {
    if (idx < 0 || idx >= SIM_EQ_MAX_NODES) return;
    const char *type = "?";
    sim_control_node_info_t info;
    if (sim_control_describe(s->ctl, idx, &info) && info.type) type = info.type;
    if (s->console_mask[idx])
        shell_out(s, "  %7.3f [Node %d/%s] %s\n", (double)ns / 1e9, node_id,
                  type, line);
    for (int i = 0; i < s->logfile_count; i++) {
        shell_logfile_t *lf = &s->logfiles[i];
        if (!lf->f || !lf->mask[idx]) continue;
        fprintf(lf->f, "  %7.3f [Node %d/%s] %s\n", (double)ns / 1e9, node_id,
                type, line);
        fflush(lf->f);
    }
}

/* --- stdin ---------------------------------------------------------------- */

static bool stdin_readable(int timeout_ms) {
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
    int rc = poll(&pfd, 1, timeout_ms);
    return rc > 0 && (pfd.revents & (POLLIN | POLLHUP | POLLERR));
}

static void handle_ctrl_c(shell_service_t *s) {
    if (s->block != SHELL_BLOCK_NONE || s->depth > 0) {
        shell_out(s, "^C — script aborted\n");
        shell_script_abort(s);
        if (!s->failed) {
            s->failed = true;
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "aborted by user (Ctrl-C)");
        }
    } else {
        shell_out(s, "^C (type exit to quit)\n");
    }
}

/* Feed the editor / read the pipe until stdin runs dry. */
static void read_stdin(shell_service_t *s) {
    if (!s->interactive || s->stdin_eof) return;
    if (s->tty) {
        edit_begin(s);
        int guard = 0;
        while (s->editing && stdin_readable(0) && guard++ < 4096) {
            char *r = linenoiseEditFeed(&s->ls);
            if (r == linenoiseEditMore) continue;
            if (r == NULL) {
                int e = errno;
                edit_end(s);
                if (e == EAGAIN) {          /* Ctrl-C */
                    handle_ctrl_c(s);
                    edit_begin(s);
                    continue;
                }
                /* Ctrl-D on an empty line, or read error: end of input. */
                s->stdin_eof = true;
                shell_enqueue_line(s, "exit");
                return;
            }
            edit_end(s);
            if (r[0]) {
                linenoiseHistoryAdd(r);
                if (s->history_path[0]) linenoiseHistorySave(s->history_path);
            }
            shell_enqueue_line(s, r);
            free(r);
            edit_begin(s);
        }
        return;
    }
    /* Pipe / file: plain lines, no prompt. */
    while (stdin_readable(0)) {
        if (s->inlen >= (int)sizeof(s->inbuf) - 1) {
            /* Overlong line: drop it. */
            shell_out(s, "error: input line too long, dropped\n");
            s->inlen = 0;
        }
        ssize_t n = read(STDIN_FILENO, s->inbuf + s->inlen,
                         sizeof(s->inbuf) - 1 - (size_t)s->inlen);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) break;
            n = 0;
        }
        if (n == 0) {
            s->stdin_eof = true;
            if (s->inlen > 0) {           /* unterminated last line */
                s->inbuf[s->inlen] = '\0';
                shell_enqueue_line(s, s->inbuf);
                s->inlen = 0;
            }
            shell_enqueue_line(s, "exit");
            return;
        }
        s->inlen += (int)n;
        s->inbuf[s->inlen] = '\0';
        char *start = s->inbuf;
        char *nl;
        while ((nl = memchr(start, '\n', (size_t)(s->inbuf + s->inlen - start)))) {
            *nl = '\0';
            if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
            shell_enqueue_line(s, start);
            start = nl + 1;
        }
        int rest = (int)(s->inbuf + s->inlen - start);
        memmove(s->inbuf, start, (size_t)rest);
        s->inlen = rest;
    }
}

void shell_enqueue_line(shell_service_t *s, const char *line) {
    /* "!cmd": run now, if the command is immediate-safe. */
    if (line[0] == '!') {
        shell_exec_line(s, line + 1, true);
        return;
    }
    if (s->qcount >= SHELL_QUEUE_MAX) {
        if (!s->queue_warned) {
            shell_out(s, "error: input queue full (%d lines), dropping input\n",
                      SHELL_QUEUE_MAX);
            s->queue_warned = true;
        }
        return;
    }
    int slot = (s->qhead + s->qcount) % SHELL_QUEUE_MAX;
    snprintf(s->queue[slot], SHELL_LINE_MAX, "%s", line);
    s->qcount++;
}

/* Called from the engine (shell_script.c) to fetch a stdin line. */
const char *shell_dequeue_line(shell_service_t *s, char *buf, size_t len);
const char *shell_dequeue_line(shell_service_t *s, char *buf, size_t len) {
    if (s->qcount <= 0) return NULL;
    snprintf(buf, len, "%s", s->queue[s->qhead]);
    s->qhead = (s->qhead + 1) % SHELL_QUEUE_MAX;
    s->qcount--;
    s->queue_warned = false;
    return buf;
}

/* Cheap gate: one poll(2) on stdin at most every 20 ms of wall time, with
 * the clock read only every 1024 loop iterations or 10 ms of sim time. */
static void gated_read_stdin(shell_service_t *s) {
    if (!s->interactive) return;
    int64_t now = sim_runtime_now_ns(s->sim);
    if (((++s->iter) & 1023) != 0 &&
        now - s->last_poll_sim_ns < 10 * SHELL_MS_TO_NS)
        return;
    double t = wall_ms();
    if (t - s->last_poll_ms < 20.0) return;
    s->last_poll_ms = t;
    s->last_poll_sim_ns = now;
    read_stdin(s);
    /* Keep the prompt's clock fresh (once a second, only when it changed). */
    if (s->editing && t - s->prompt_ms >= 1000.0 && s->prompt_ns != now) {
        s->prompt_ms = t;
        linenoiseHide(&s->ls);
        build_prompt(s);
        linenoiseShow(&s->ls);
    }
}

void shell_service_poll_input(shell_service_t *s) {
    if (!shell_service_active(s)) return;
    gated_read_stdin(s);
}

void shell_service_pump_paused(shell_service_t *s, int timeout_ms) {
    if (!shell_service_active(s)) return;
    if (s->interactive && !s->stdin_eof) {
        if (s->tty) edit_begin(s);
        if (stdin_readable(timeout_ms)) read_stdin(s);
    } else {
        usleep((useconds_t)timeout_ms * 1000);
    }
    shell_script_tick(s);
    if (s->editing && s->prompt_ns != sim_runtime_now_ns(s->sim)) {
        linenoiseHide(&s->ls);
        build_prompt(s);
        linenoiseShow(&s->ls);
    }
}

void shell_service_on_autopause(shell_service_t *s) {
    if (!shell_service_active(s)) return;
    char t[32];
    shell_format_time(sim_runtime_now_ns(s->sim), t, sizeof(t));
    if (!s->interactive) return;
    if (s->block == SHELL_BLOCK_RUN)      /* the user's own run/step */
        shell_out(s, "paused at %s\n", t);
    else                                  /* the -t horizon */
        shell_out(s, "-t duration reached: paused at %s (run to continue, exit to quit)\n", t);
}

/* --- service ops --------------------------------------------------------- */

static void shell_poll(sim_runtime_t *sim, void *state) {
    (void)sim;
    shell_service_t *s = (shell_service_t *)state;
    if (!s->active) return;
    if (!s->started) {
        s->started = true;
        if (s->tty) edit_begin(s);
    }
    gated_read_stdin(s);
    shell_script_tick(s);
}

static void shell_on_event(sim_runtime_t *sim, void *state,
                           const sim_observer_event_t *ev) {
    (void)sim;
    shell_service_t *s = (shell_service_t *)state;
    if (!s->active || ev->kind != SIM_OBS_MOTE_LOG_LINE) return;
    console_line(s, ev->mote_index, ev->u.log_line.node_id,
                 ev->u.log_line.line, ev->time_ns);
    shell_script_on_log_line(s, ev->mote_index, ev->u.log_line.node_id,
                             ev->u.log_line.line, ev->time_ns);
}

static void close_logfiles(shell_service_t *s) {
    for (int i = 0; i < s->logfile_count; i++) {
        if (s->logfiles[i].f) { fclose(s->logfiles[i].f); s->logfiles[i].f = NULL; }
    }
    s->logfile_count = 0;
}

static void shell_destroy(sim_runtime_t *sim, void *state) {
    (void)sim;
    shell_service_t *s = (shell_service_t *)state;
    if (!s->active) return;
    edit_end(s);
    if (s->history_path[0]) linenoiseHistorySave(s->history_path);
    close_logfiles(s);
    shell_script_abort(s);
    s->active = false;
}

const sim_service_ops_t shell_service_ops = {
    .name     = "shell",
    .init     = NULL,          /* adopt the runner-constructed struct */
    .destroy  = shell_destroy,
    .on_event = shell_on_event,
    .poll     = shell_poll,
};

/* --- lifecycle ------------------------------------------------------------ */

int shell_service_start(shell_service_t *s, sim_runtime_t *sim,
                        sim_control_t *ctl, bool interactive,
                        const char *script_path, bool verbose) {
    memset(s, 0, sizeof(*s));
    s->sim = sim;
    s->ctl = ctl;
    s->active = true;
    s->interactive = interactive;
    s->verbose = verbose;
    s->tty = interactive && isatty(STDIN_FILENO) && isatty(STDOUT_FILENO);
    s->next_at_id = 1;
    s->default_expect_timeout_ns = 30LL * 1000 * SHELL_MS_TO_NS;
    s->stop_when_done = !interactive;
    memset(s->console_mask, verbose ? 1 : 0, sizeof(s->console_mask));
    shell_script_init(s);

    /* Pipes: make command echo / prompts visible promptly. */
    if (!s->tty) setvbuf(stdout, NULL, _IOLBF, 0);

    if (s->tty) {
        const char *hp = getenv("CSIM_SHELL_HISTORY");
        if (hp) {
            snprintf(s->history_path, sizeof(s->history_path), "%s", hp);
        } else {
            const char *home = getenv("HOME");
            if (home && home[0])
                snprintf(s->history_path, sizeof(s->history_path),
                         "%s/.cooja-ng_history", home);
        }
        if (s->history_path[0]) {
            linenoiseHistorySetMaxLen(500);
            linenoiseHistoryLoad(s->history_path);
        }
        linenoiseSetCompletionCallback(shell_complete);
    }
    if (script_path && script_path[0]) {
        if (shell_script_source(s, script_path) != 0) {
            fprintf(stderr, "--script: cannot open %s\n", script_path);
            return -1;
        }
    }
    if (interactive)
        printf("Cooja-NG shell: type 'help' for commands%s\n",
               s->tty ? "" : " (no terminal: plain line mode)");
    return 0;
}

void shell_service_on_restart(shell_service_t *s) {
    if (!shell_service_active(s)) return;
    shell_script_abort(s);
    shell_script_at_remove(s, -1);
    s->trigger_count = 0;
}

int shell_service_report(shell_service_t *s, int64_t now_ns) {
    if (!shell_service_active(s)) return 0;
    edit_end(s);
    if (!s->script_used) return 0;
    /* A script still in flight at the end of the run is a failure unless it
     * ended the run itself (exit) or was passed explicitly. */
    if (!s->failed && !s->passed && !s->exited && !s->finished) {
        s->failed = true;
        if (s->block == SHELL_BLOCK_EXPECT)
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "script did not complete: still waiting for \"%s\"",
                     s->expect_pattern);
        else if (s->block != SHELL_BLOCK_NONE)
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "script did not complete: blocked at %.3f s",
                     (double)s->block_deadline_ns / 1e9);
        else
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "script did not complete");
    }
    printf("\n--- Script Results ---\n");
    printf("  expects: %d passed, %d failed\n", s->expect_pass, s->expect_fail);
    if (!s->failed) {
        printf("\n  SCRIPT PASSED (%lld ms simulated)\n",
               (long long)(now_ns / SHELL_MS_TO_NS));
        return 0;
    }
    printf("\n  SCRIPT FAILED: %s\n", s->fail_reason);
    return 1;
}
