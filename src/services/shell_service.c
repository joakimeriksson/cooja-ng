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
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
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
    else if (s->block == SHELL_BLOCK_CMD) state = " [cmd]";
    else if (s->block == SHELL_BLOCK_EXPECT_NOT) state = " [expect-not]";
    else if (s->block == SHELL_BLOCK_FAULT) state = " [expect-fault]";
    else if (s->block == SHELL_BLOCK_HALT) state = " [expect-halt]";
    else if (s->depth > 0) state = " [script]";
    snprintf(s->prompt, sizeof(s->prompt), "cooja %.3fs%s> ",
             (double)now / 1e9, state);
    s->prompt_ns = now;
    s->ls.prompt = s->prompt;
    s->ls.plen = strlen(s->prompt);
}

static void edit_begin(shell_service_t *s) {
    if (!s->editor || s->editing) return;
    build_prompt(s);
    if (linenoiseEditStart(&s->ls, -1, -1, s->linebuf, sizeof(s->linebuf),
                           s->prompt) == 0) {
        s->editing = true;
        s->prompt_ms = wall_ms();
    } else {
        /* Not a usable terminal after all: fall back to plain lines. */
        s->editor = false;
    }
}

static void edit_end(shell_service_t *s) {
    if (!s->editing) return;
    linenoiseEditStop(&s->ls);
    s->editing = false;
}

/* Output bursts: while `hidden`, the prompt stays off the screen and every
 * line is printed plainly; one release redraws it.  Console lines printed
 * during a pump and the output of a tick's commands (including the runner's
 * own prints from add/reboot) are one burst each, instead of a
 * hide/print/show round trip per line. */
void shell_hold_output(shell_service_t *s) {
    if (s->hidden) return;
    if (s->editing) linenoiseHide(&s->ls);
    s->hidden = true;
}

void shell_release_output(shell_service_t *s) {
    if (!s->hidden) return;
    s->hidden = false;
    fflush(stdout);
    if (s->editing) {
        build_prompt(s);
        linenoiseShow(&s->ls);
    }
}

void shell_out(shell_service_t *s, const char *fmt, ...) {
    va_list ap;
    bool redraw = s->editing && !s->hidden;
    if (redraw) linenoiseHide(&s->ls);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    if (redraw) {
        fflush(stdout);
        build_prompt(s);
        linenoiseShow(&s->ls);
    }
}

const char *shell_origin(shell_service_t *s, char *buf, size_t len) {
    snprintf(buf, len, "%s", s->origin.where[0] ? s->origin.where : "stdin");
    return buf;
}

static void shell_error_code(shell_service_t *s, int code, const char *msg);

void shell_error(shell_service_t *s, const char *fmt, ...) {
    char msg[SHELL_REASON_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    shell_error_code(s, SHELL_EXIT_INVALID, msg);
}

/* A failed `assert`: reported like an error, but it is an assertion, so it
 * carries the assertion exit code. */
void shell_assert_failed(shell_service_t *s, const char *fmt, ...) {
    char msg[SHELL_REASON_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    shell_error_code(s, SHELL_EXIT_ASSERT, msg);
}

static void shell_error_code(shell_service_t *s, int code, const char *msg) {
    char origin[sizeof(s->origin.where)];
    shell_origin(s, origin, sizeof(origin));
    shell_out(s, "error: %s (%s)\n", msg, origin);
    /* Only a script's own lines fail it — never a typo at the prompt.  A
     * malformed or impossible request is EXIT_INVALID, not an assertion;
     * a false assert arrives via shell_assert_failed with EXIT_ASSERT. */
    if (s->origin.script) {
        char reason[SHELL_REASON_MAX];
        snprintf(reason, sizeof(reason), "%s: %.*s", origin,
                 (int)(sizeof(reason) - sizeof(origin) - 3), msg);
        shell_script_fail_code(s, code, reason);
    }
}

/* --- console routing ---------------------------------------------------- */

static void console_line(shell_service_t *s, int idx, int node_id,
                         const char *line, int64_t ns) {
    if (idx < 0 || idx >= SIM_EQ_MAX_NODES) return;
    if (s->hist) {
        int slot = (s->hist_head + s->hist_count) % SHELL_HISTORY_LINES;
        if (s->hist_count < SHELL_HISTORY_LINES) s->hist_count++;
        else s->hist_head = (s->hist_head + 1) % SHELL_HISTORY_LINES;
        s->hist[slot].node_id = node_id;
        s->hist[slot].ns = ns;
        snprintf(s->hist[slot].text, sizeof(s->hist[slot].text), "%s", line);
    }
    const char *type = "?";
    sim_control_node_info_t info;
    if (sim_control_describe(s->ctl, idx, &info) && info.type) type = info.type;
    /* In console mode the terminal shows only the console node's raw
     * bytes; prefixed lines would duplicate and interleave with them. */
    if (s->console_mask[idx] && !s->console_mode) {
        shell_hold_output(s);      /* released at the next poll */
        shell_out(s, "  %7.3f [Node %d/%s] %s\n", (double)ns / 1e9, node_id,
                  type, line);
    }
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
            s->fail_code = SHELL_EXIT_CANCELLED;
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "aborted by user (Ctrl-C)");
        }
    } else {
        shell_out(s, "^C (type exit to quit)\n");
    }
}

/* Move complete lines out of inbuf into the queue, stopping when the queue
 * is full: what is left stays buffered (and what has not been read stays in
 * the kernel), so a burst larger than the queue is paced, never dropped. */
static void split_buffered_lines(shell_service_t *s) {
    char *start = s->inbuf;
    char *end = s->inbuf + s->inlen;
    char *nl;
    while (s->qcount < SHELL_QUEUE_MAX &&
           (nl = memchr(start, '\n', (size_t)(end - start)))) {
        *nl = '\0';
        if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
        shell_enqueue_line(s, start);
        start = nl + 1;
    }
    int rest = (int)(end - start);
    memmove(s->inbuf, start, (size_t)rest);
    s->inlen = rest;
}

/* At EOF the input ends with an implied `exit`.  Queue it once there is
 * room and nothing buffered is still waiting — dropping it would leave a
 * run with no duration running forever.  Whatever is buffered without a
 * newline is the last line; run it first, whichever path saw the EOF. */
static void deliver_eof_exit(shell_service_t *s) {
    if (!s->stdin_eof || s->eof_exit_queued) return;
    if (s->inlen > 0) {
        if (s->qcount >= SHELL_QUEUE_MAX) return;
        s->inbuf[s->inlen] = '\0';   /* unterminated last line */
        shell_enqueue_line(s, s->inbuf);
        s->inlen = 0;
    }
    if (s->qcount >= SHELL_QUEUE_MAX) return;
    s->eof_exit_queued = true;
    shell_enqueue_line(s, "exit");
}

/* --- console mode ------------------------------------------------------------ */

static struct termios g_console_saved;

int shell_console_enter(shell_service_t *s, int idx, int node_id) {
    if (!s->editor || !s->interactive) return -1;
    shell_release_output(s);
    edit_end(s);                         /* linenoise restores cooked mode */
    struct termios t;
    if (tcgetattr(STDIN_FILENO, &g_console_saved) != 0) return -1;
    t = g_console_saved;
    /* Line mode with local echo, like a serial terminal to a shell that
     * does not echo (Contiki-NG's does not).  ISIG off: Ctrl-C is a byte
     * for the node, not a signal that ends the simulation. */
    t.c_lflag |= (ICANON | ECHO);
    t.c_lflag &= ~ISIG;
    t.c_oflag |= OPOST;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    s->console_mode = true;
    s->console_idx = idx;
    s->console_id = node_id;
    s->inlen = 0;
    printf("[console to node %d: lines you type go to the node; ~. or Ctrl-D returns to the shell]\n",
           node_id);
    if (sim_control_paused(s->ctl))
        printf("[note: the simulation is paused; the node answers after `run`]\n");
    fflush(stdout);
    return 0;
}

static void console_leave(shell_service_t *s) {
    if (!s->console_mode) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &g_console_saved);
    s->console_mode = false;
    s->inlen = 0;
    printf("\n[back in the Cooja-NG shell]\n");
    fflush(stdout);
}

static void console_send_line(shell_service_t *s, const char *line, int len) {
    char buf[SHELL_LINE_MAX + 1];
    if (len > SHELL_LINE_MAX - 1) len = SHELL_LINE_MAX - 1;
    memcpy(buf, line, (size_t)len);
    buf[len++] = '\n';
    if (!sim_control_node_active(s->ctl, s->console_idx)) {
        printf("[node %d is not running]\n", s->console_id);
        return;
    }
    int took = sim_control_send(s->ctl, s->console_id, (const uint8_t *)buf, len,
                                SIM_CONTROL_WAKE | SIM_CONTROL_RETRY);
    if (took < len)
        printf("[node %d: input truncated, %d of %d bytes queued]\n",
               s->console_id, took < 0 ? 0 : took, len);
}

/* Console mode input: whole lines from the cooked terminal. */
static void console_read(shell_service_t *s) {
    while (s->console_mode && stdin_readable(0)) {
        ssize_t n = read(STDIN_FILENO, s->inbuf + s->inlen,
                         sizeof(s->inbuf) - 1 - (size_t)s->inlen);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) break;
            n = 0;
        }
        if (n == 0) {                   /* Ctrl-D */
            console_leave(s);
            break;
        }
        s->inlen += (int)n;
        char *start = s->inbuf;
        char *nl;
        while (s->console_mode &&
               (nl = memchr(start, '\n', (size_t)(s->inbuf + s->inlen - start)))) {
            int len = (int)(nl - start);
            if ((len == 2 && start[0] == '~' && start[1] == '.') ||
                (len >= 1 && start[0] == 0x1d)) {
                console_leave(s);
                break;
            }
            console_send_line(s, start, len);
            start = nl + 1;
        }
        if (!s->console_mode) break;
        int rest = (int)(s->inbuf + s->inlen - start);
        if (rest >= (int)sizeof(s->inbuf) - 1) rest = 0;      /* overlong: drop */
        memmove(s->inbuf, start, (size_t)rest);
        s->inlen = rest;
    }
    if (!s->console_mode) edit_begin(s);
}

static void console_flush(shell_service_t *s) {
    if (!s->console_dirty) return;
    s->console_dirty = false;
    fflush(stdout);
}

/* Feed the editor / read the pipe until stdin runs dry. */
static void read_stdin(shell_service_t *s) {
    if (!s->interactive) return;
    if (s->stdin_eof) {
        split_buffered_lines(s);
        deliver_eof_exit(s);
        return;
    }
    if (s->console_mode) {
        console_read(s);
        return;
    }
    if (s->editor) {
        edit_begin(s);
        int guard = 0;
        while (s->editing && s->qcount < SHELL_QUEUE_MAX &&
               stdin_readable(0) && guard++ < 4096) {
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
                deliver_eof_exit(s);
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
    /* Pipe / file: plain lines, no prompt.  Buffered lines are split first,
     * and reading stops while the queue is full — the rest waits in the pipe
     * until the engine has consumed what it has. */
    for (;;) {
        split_buffered_lines(s);
        if (s->qcount >= SHELL_QUEUE_MAX) return;
        if (!stdin_readable(0)) return;
        if (s->inlen >= (int)sizeof(s->inbuf) - 1) {
            /* A full buffer with no newline left in it: the line is longer
             * than the buffer and cannot be run. */
            shell_out(s, "error: input line too long, dropped\n");
            s->inlen = 0;
        }
        ssize_t n = read(STDIN_FILENO, s->inbuf + s->inlen,
                         sizeof(s->inbuf) - 1 - (size_t)s->inlen);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) return;
            n = 0;
        }
        if (n == 0) {
            s->stdin_eof = true;
            deliver_eof_exit(s);
            return;
        }
        s->inlen += (int)n;
        s->inbuf[s->inlen] = '\0';
    }
}

void shell_enqueue_line(shell_service_t *s, const char *line) {
    /* "!cmd" at a terminal: run now, if the command is immediate-safe.  A
     * pipe is sequential (sync_stdin): its "!" lines queue in order and
     * run like any other line. */
    if (line[0] == '!' && !s->sync_stdin) {
        shell_transcript_record(s, line);
        shell_origin_t o = { .kind = SHELL_ORIGIN_STDIN, .script = false, .where = "stdin" };
        shell_exec_line(s, line + 1, true, &o);
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

/* --- signals ---------------------------------------------------------------- */

static volatile sig_atomic_t g_shell_signal = 0;

static void shell_on_signal(int sig) {
    g_shell_signal = sig;
}

static void install_signals(bool on) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sigemptyset(&sa.sa_mask);
    if (on) {
        sa.sa_handler = shell_on_signal;
        /* No SA_RESTART: a blocking poll on stdin returns EINTR.  A second
         * signal gets the default action, so a stuck run can still be
         * killed. */
        sa.sa_flags = SA_RESETHAND;
    } else {
        sa.sa_handler = SIG_DFL;
    }
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

bool shell_check_signal(shell_service_t *s) {
    int sig = g_shell_signal;
    if (!sig) return false;
    g_shell_signal = 0;
    shell_out(s, "\n%s: ending the run (reports and --save-config still run)\n",
              sig == SIGINT ? "SIGINT" : "SIGTERM");
    if ((s->block != SHELL_BLOCK_NONE || s->depth > 0) && !s->failed) {
        s->failed = true;
        s->fail_code = SHELL_EXIT_CANCELLED;
        snprintf(s->fail_reason, sizeof(s->fail_reason),
                 "cancelled by %s", sig == SIGINT ? "SIGINT" : "SIGTERM");
    }
    s->exited = true;
    sim_control_request_exit(s->ctl);
    return true;
}

/* sync_stdin: block until piped stdin yields at least one line, or EOF. */
void shell_read_stdin_sync(shell_service_t *s) {
    while (s->interactive && s->qcount == 0) {
        /* Lines already buffered (a burst larger than the queue) must be
         * split before blocking, or the poll waits for input that has
         * already arrived. */
        read_stdin(s);
        if (s->qcount > 0 || s->stdin_eof) return;
        struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
        fflush(stdout);
        /* With a wall-clock bound, wake up often enough to honour it: a
         * sequential session otherwise waits here forever for a line that
         * is never coming. */
        int timeout = -1;
        if (s->wall_deadline_ms > 0) {
            double left = s->wall_deadline_ms - wall_ms();
            if (left <= 0) {
                shell_out(s, "--wall-timeout reached while waiting for input; "
                          "ending the run\n");
                s->wall_timeout_hit = true;
                s->exited = true;
                sim_control_request_exit(s->ctl);
                return;
            }
            timeout = left < 200.0 ? (int)left + 1 : 200;
        }
        int rc = poll(&pfd, 1, timeout);
        if (rc < 0) {
            if (errno == EINTR) {
                if (shell_check_signal(s)) return;
                continue;
            }
            s->stdin_eof = true;
            deliver_eof_exit(s);
            return;
        }
        read_stdin(s);
    }
}

/* Cheap gate: one poll(2) on stdin at most every 20 ms of wall time, with
 * the clock read only every 1024 loop iterations or 10 ms of sim time.
 * Terminal only: piped stdin is read synchronously by the engine. */
static void gated_read_stdin(shell_service_t *s) {
    if (!s->interactive || s->sync_stdin) return;
    int64_t now = sim_runtime_now_ns(s->sim);
    if (((++s->iter) & 1023) != 0 &&
        now - s->last_poll_sim_ns < 10 * SHELL_MS_TO_NS)
        return;
    double t = wall_ms();
    if (t - s->last_poll_ms < 20.0) return;
    s->last_poll_ms = t;
    s->last_poll_sim_ns = now;
    shell_release_output(s);
    read_stdin(s);
    /* Keep the prompt's clock fresh (once a second, only when it changed). */
    if (s->editing && !s->hidden && t - s->prompt_ms >= 1000.0 && s->prompt_ns != now) {
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

static const char *block_name(shell_block_t b) {
    switch (b) {
    case SHELL_BLOCK_EXPECT:     return "expect";
    case SHELL_BLOCK_SLEEP:      return "sleep";
    case SHELL_BLOCK_WAIT_UNTIL: return "wait-until";
    case SHELL_BLOCK_RUN:        return "run";
    case SHELL_BLOCK_CMD:        return "cmd";
    case SHELL_BLOCK_EXPECT_NOT: return "expect-not";
    case SHELL_BLOCK_FAULT:      return "expect-fault";
    case SHELL_BLOCK_HALT:       return "expect-halt";
    default:                     return "nothing";
    }
}

void shell_service_pump_paused(shell_service_t *s, int timeout_ms) {
    if (!shell_service_active(s)) return;
    if (shell_check_signal(s)) return;
    shell_release_output(s);
    console_flush(s);
    if (s->interactive && !s->stdin_eof) {
        if (s->editor) edit_begin(s);
        if (stdin_readable(timeout_ms)) read_stdin(s);
    } else if (timeout_ms > 0) {
        usleep((useconds_t)timeout_ms * 1000);
    }
    shell_script_tick(s);

    /* Nothing advances simulated time while paused, so a stream blocked on
     * time or on a console line can only be released by a resume.  If
     * nothing can resume — no terminal input left (a pipe is sequential:
     * its lines queue behind the block), no web UI — that is a deadlock:
     * fail it instead of hanging. */
    if (sim_control_paused(s->ctl) && !sim_runtime_stop_requested(s->sim)) {
        bool can_resume = (s->stdin_tty && !s->stdin_eof) || s->external_resume;
        bool time_block = s->block == SHELL_BLOCK_SLEEP ||
                          s->block == SHELL_BLOCK_WAIT_UNTIL ||
                          s->block == SHELL_BLOCK_EXPECT ||
                          s->block == SHELL_BLOCK_CMD ||
                          s->block == SHELL_BLOCK_EXPECT_NOT ||
                          s->block == SHELL_BLOCK_FAULT ||
                          s->block == SHELL_BLOCK_HALT;
        if (time_block && !can_resume) {
            char reason[SHELL_REASON_MAX];
            char cause[SHELL_LINE_MAX + 96] = "";
            int halted = shell_dbg_halted_node(s);
            if (halted >= 0)
                snprintf(cause, sizeof(cause),
                         " — node %d is halted at a breakpoint; `continue` "
                         "releases it (from a terminal, `!continue`)", halted);
            else if (s->run_pause_cmd[0])
                snprintf(cause, sizeof(cause),
                         " — the simulation is paused because `%.200s` (%s) "
                         "ran to its end; put a `run` before this command",
                         s->run_pause_cmd, s->run_pause_where);
            snprintf(reason, sizeof(reason),
                     "%s blocks the command stream while the simulation is "
                     "paused, and nothing can resume it (deadlock)%s",
                     block_name(s->block), cause);
            shell_script_fail_code(s, SHELL_EXIT_INVALID, reason);
            if (!s->interactive) sim_control_request_exit(s->ctl);
        } else if (time_block && s->stdin_tty && !s->paused_hint) {
            s->paused_hint = true;
            shell_out(s, "note: %s is blocked while the simulation is paused; "
                      "type !run to continue\n", block_name(s->block));
        } else if (!s->interactive && !s->external_resume &&
                   s->block == SHELL_BLOCK_NONE && s->qcount == 0 &&
                   s->depth == 0) {
            /* --script alone, script done, paused: nothing will ever run. */
            shell_out(s, "note: the simulation is paused and no input is left; ending the run\n");
            sim_control_request_exit(s->ctl);
        }
    } else {
        s->paused_hint = false;
        s->run_pause_cmd[0] = '\0';   /* running again: no longer the cause */
    }

    shell_release_output(s);
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
    if (!sim_control_horizon_hit(s->ctl)) {   /* the user's own run/step */
        shell_out(s, "paused at %s\n", t);
        return;
    }
    /* The -t horizon.  A terminal keeps the prompt and can type `run`; a
     * pipe cannot resume, so -t ends the run there, as it does without
     * --shell. */
    if (!s->stdin_tty && !s->external_resume) {
        shell_out(s, "-t duration reached at %s: ending the run\n", t);
        s->exited = true;
        sim_control_request_exit(s->ctl);
        return;
    }
    shell_out(s, "-t duration reached: paused at %s (run to continue, exit to quit)\n", t);
}

/* --- service ops --------------------------------------------------------- */

static void shell_poll(sim_runtime_t *sim, void *state) {
    (void)sim;
    shell_service_t *s = (shell_service_t *)state;
    if (!s->active) return;
    if (shell_check_signal(s)) return;
    shell_release_output(s);          /* console lines from the last pump */
    console_flush(s);
    if (!s->started) {
        s->started = true;
        if (s->editor) edit_begin(s);
    }
    gated_read_stdin(s);
    shell_script_tick(s);
    shell_release_output(s);
}

static void shell_on_event(sim_runtime_t *sim, void *state,
                           const sim_observer_event_t *ev) {
    (void)sim;
    shell_service_t *s = (shell_service_t *)state;
    if (!s->active) return;
    if (ev->kind == SIM_OBS_MOTE_UART_BYTE) {
        if (s->console_mode && ev->mote_index == s->console_idx && ev->u.uart.byte != '\r') {
            putchar(ev->u.uart.byte);
            s->console_dirty = true;
        }
        shell_script_on_uart_byte(s, ev->mote_index, ev->u.uart.byte, ev->time_ns);
        return;
    }
    if (ev->kind != SIM_OBS_MOTE_LOG_LINE) return;
    console_line(s, ev->mote_index, ev->u.log_line.node_id,
                 ev->u.log_line.line, ev->time_ns);
    shell_script_on_log_line(s, ev->mote_index, ev->u.log_line.node_id,
                             ev->u.log_line.line, ev->time_ns);
}

void shell_transcript_record(shell_service_t *s, const char *line) {
    if (!s->transcript) return;
    const char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (!*p || !strncmp(p, "transcript", 10)) return;
    fprintf(s->transcript, "%s\n", line);
    fflush(s->transcript);
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
    shell_release_output(s);
    console_leave(s);
    edit_end(s);
    if (s->history_path[0]) linenoiseHistorySave(s->history_path);
    close_logfiles(s);
    if (s->transcript) { fclose(s->transcript); s->transcript = NULL; }
    free(s->hist);
    s->hist = NULL;
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++) { free(s->sym_cache[i]); s->sym_cache[i] = NULL; }
    shell_script_abort(s);
    install_signals(false);
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
    /* Three separate questions: can a human type at us (stdin), can we
     * draw a prompt (both ends), and must input run strictly in order.
     * `--shell | tee log` has a terminal to type at but no editor, and it
     * must NOT become a sequential session — that would stop the
     * simulation between commands. */
    s->stdin_tty  = interactive && isatty(STDIN_FILENO);
    s->editor     = s->stdin_tty && isatty(STDOUT_FILENO);
    s->sync_stdin = interactive && !s->stdin_tty;
    s->origin.kind = SHELL_ORIGIN_STDIN;
    snprintf(s->origin.where, sizeof(s->origin.where), "stdin");
    s->next_at_id = 1;
    s->next_dbg_id = 1;
    s->default_expect_timeout_ns = 30LL * 1000 * SHELL_MS_TO_NS;
    s->max_line = 128;   /* Contiki-NG SERIAL_LINE_CONF_BUFSIZE default */
    s->hist = calloc(SHELL_HISTORY_LINES, sizeof(*s->hist));
    snprintf(s->prompt_glob, sizeof(s->prompt_glob), "#*> ");  /* Contiki-NG */
    s->stop_when_done = !interactive;
    memset(s->console_mask, verbose ? 1 : 0, sizeof(s->console_mask));
    shell_script_init(s);


    if (s->editor) {
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
    install_signals(true);
    if (interactive)
        printf("Cooja-NG shell: type 'help' for commands%s\n",
               s->stdin_tty ? "" : " (no terminal: lines run in order, like a script)");
    return 0;
}

void shell_service_on_restart(shell_service_t *s) {
    if (!shell_service_active(s)) return;
    /* A restart the stream itself asked for keeps the stream: stdin lines
     * after `restart` run against the new simulation.  Script files are
     * aborted — their state described the old run. */
    shell_script_abort(s);
    shell_script_at_remove(s, -1);
    s->trigger_count = 0;
    s->triggers_dropped = 0;
    s->restart_pending = false;
}

int shell_service_report(shell_service_t *s, int64_t elapsed_ns) {
    if (!shell_service_active(s)) return 0;
    shell_release_output(s);
    edit_end(s);
    if (!s->script_used) return 0;
    /* A blocked command or an unfinished script file at the end of the run
     * is a failure however the run ended — duration reached, `exit` typed
     * at the prompt, or a signal.  (`exit` inside a script file closes the
     * script first, so it counts as finished.)  Without either, an
     * interactive session that ended with `exit` is not a failure. */
    bool in_flight = s->block != SHELL_BLOCK_NONE || s->depth > 0;
    if (!s->failed && !s->passed &&
        (in_flight || (!s->exited && !s->finished))) {
        s->failed = true;
        if (s->block == SHELL_BLOCK_EXPECT)
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "script did not complete: still waiting for \"%s\"",
                     s->expect_pattern);
        else if (s->block == SHELL_BLOCK_CMD)
            snprintf(s->fail_reason, sizeof(s->fail_reason),
                     "script did not complete: cmd %d \"%.60s\" still waiting for the prompt",
                     s->cmd_id, s->cmd_text);
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
    if (s->cmd_pass || s->cmd_fail)
        printf("  cmds:    %d passed, %d failed\n", s->cmd_pass, s->cmd_fail);
    if (!s->failed) {
        printf("\n  SCRIPT PASSED (%lld ms simulated)\n",
               (long long)(elapsed_ns / SHELL_MS_TO_NS));
        return 0;
    }
    printf("\n  SCRIPT FAILED: %s\n", s->fail_reason);
    return s->fail_code ? s->fail_code : SHELL_EXIT_ASSERT;
}
