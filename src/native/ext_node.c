/*
 * ext_node — external data-driven mote engine (see include/native/ext_node.h
 * and docs/design/external-nodes-plan.md §4).
 *
 * Single-threaded and blocking by design: the kernel is single-threaded, and
 * a slow peer slice is indistinguishable from a slow emulated slice.  Every
 * read is bounded by CSIM_EXT_TIMEOUT_MS so a wedged peer fails the run
 * instead of hanging it.
 *
 * The fork/exec + teardown shape is lifted from sim_external_command.c.
 */
#include "ext_node.h"

#include <errno.h>
#include <stdarg.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "cJSON.h"

/* ============================================================
 * Failure policy
 * ============================================================ */

/* A protocol violation or a dead peer is fatal to the run: the node's
 * behaviour is simply gone, and continuing would silently simulate a node
 * that stopped existing.  Mark failed and stop stepping it; the runner sees
 * no more traffic and the message says why. */
static void ext_fail(ext_node_t *node, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static void ext_fail(ext_node_t *node, const char *fmt, ...) {
    if (node->failed) return;
    node->failed = true;
    node->next_wakeup_ns = INT64_MAX;

    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "ext_node[%d] (%s): ", node->node_id, node->command);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

/* ============================================================
 * Line I/O
 * ============================================================ */

static int write_all(ext_node_t *node, const char *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(node->to_child, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) continue;
            ext_fail(node, "write failed: %s", strerror(errno));
            return -1;
        }
        off += (size_t)n;
    }
    return 0;
}

/* Read one '\n'-terminated line into out (NUL-terminated, newline stripped).
 * Returns 0 on success, -1 on timeout / EOF / error (node marked failed). */
static int read_line(ext_node_t *node, char *out, size_t out_size) {
    for (;;) {
        /* Complete line already buffered? */
        char *nl = memchr(node->rbuf, '\n', (size_t)node->rlen);
        if (nl) {
            size_t line_len = (size_t)(nl - node->rbuf);
            if (line_len >= out_size) {
                ext_fail(node, "reply line too long (%zu bytes)", line_len);
                return -1;
            }
            memcpy(out, node->rbuf, line_len);
            out[line_len] = '\0';
            size_t rest = (size_t)node->rlen - line_len - 1;
            memmove(node->rbuf, nl + 1, rest);
            node->rlen = (int)rest;
            return 0;
        }

        if (node->rlen == (int)sizeof(node->rbuf)) {
            ext_fail(node, "reply line exceeds %zu bytes with no newline",
                     sizeof(node->rbuf));
            return -1;
        }

        struct pollfd pfd = { .fd = node->from_child, .events = POLLIN };
        int pr = poll(&pfd, 1, node->timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            ext_fail(node, "poll failed: %s", strerror(errno));
            return -1;
        }
        if (pr == 0) {
            ext_fail(node, "timed out after %d ms waiting for a reply "
                           "(CSIM_EXT_TIMEOUT_MS)", node->timeout_ms);
            return -1;
        }

        ssize_t n = read(node->from_child, node->rbuf + node->rlen,
                         sizeof(node->rbuf) - (size_t)node->rlen);
        if (n < 0) {
            if (errno == EINTR) continue;
            ext_fail(node, "read failed: %s", strerror(errno));
            return -1;
        }
        if (n == 0) {
            ext_fail(node, "peer closed its output (exited?) with no reply");
            return -1;
        }
        node->rlen += (int)n;
    }
}

/* ============================================================
 * Hex
 * ============================================================ */

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Returns byte count, or -1 on a malformed/oversize string. */
static int hex_decode(const char *hex, uint8_t *out, int out_max) {
    size_t len = strlen(hex);
    if (len % 2 != 0) return -1;
    int n = (int)(len / 2);
    if (n > out_max) return -1;
    for (int i = 0; i < n; i++) {
        int hi = hex_nibble(hex[2 * i]), lo = hex_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return n;
}

/* ============================================================
 * Applying the peer's output events
 * ============================================================ */

static void emit_log_line(ext_node_t *node, const char *line) {
    if (!node->log_callback) return;
    for (const char *p = line; *p; p++)
        node->log_callback(node->log_callback_data, (uint8_t)*p);
    node->log_callback(node->log_callback_data, (uint8_t)'\n');
}

static void apply_out_event(ext_node_t *node, cJSON *ev, int64_t step_t) {
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(ev, "type");
    if (!cJSON_IsString(type)) {
        ext_fail(node, "output event with no `type`");
        return;
    }

    /* Events are stamped in sim time; a peer may not schedule into the past.
     * Times at or before `now` fire immediately in this cut — deferring a
     * future-stamped event needs the pending queue that arrives with M1. */
    const cJSON *t = cJSON_GetObjectItemCaseSensitive(ev, "t");
    if (cJSON_IsNumber(t) && (int64_t)t->valuedouble < step_t) {
        ext_fail(node, "output event stamped t=%lld before step t=%lld",
                 (long long)t->valuedouble, (long long)step_t);
        return;
    }

    if (strcmp(type->valuestring, "tx") == 0) {
        const cJSON *frame = cJSON_GetObjectItemCaseSensitive(ev, "frame");
        if (!cJSON_IsString(frame)) {
            ext_fail(node, "tx event with no `frame`");
            return;
        }
        uint8_t buf[EXT_NODE_MAX_FRAME];
        int len = hex_decode(frame->valuestring, buf, (int)sizeof(buf));
        if (len < 0) {
            /* Loud, because the runner's PHY wrap drops an oversize frame
             * silently — a jammer would look like it was working. */
            ext_fail(node, "tx frame is malformed hex or longer than %d bytes",
                     EXT_NODE_MAX_FRAME);
            return;
        }
        if (node->rf_frame_callback)
            node->rf_frame_callback(node->rf_frame_callback_data, buf, len);

    } else if (strcmp(type->valuestring, "log") == 0) {
        const cJSON *line = cJSON_GetObjectItemCaseSensitive(ev, "line");
        if (cJSON_IsString(line))
            emit_log_line(node, line->valuestring);

    } else if (strcmp(type->valuestring, "wake") == 0) {
        if (cJSON_IsNumber(t)) {
            int64_t when = (int64_t)t->valuedouble;
            if (when < node->next_wakeup_ns) node->next_wakeup_ns = when;
        }

    } else {
        /* Unknown types are ignored on purpose: the protocol only ever gains
         * event kinds, and an old csim must not reject a newer peer. */
    }
}

/* Read one `done` reply and apply it. */
static int consume_done(ext_node_t *node, int64_t step_t) {
    char line[EXT_NODE_LINE_MAX];
    if (read_line(node, line, sizeof(line)) != 0) return -1;

    cJSON *msg = cJSON_Parse(line);
    if (!msg) {
        ext_fail(node, "reply is not JSON: %.120s", line);
        return -1;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(msg, "type");
    if (!cJSON_IsString(type) || strcmp(type->valuestring, "done") != 0) {
        ext_fail(node, "expected a `done` reply, got: %.120s", line);
        cJSON_Delete(msg);
        return -1;
    }

    /* `wake` first, so a `wake` output event can only pull it earlier. */
    const cJSON *wake = cJSON_GetObjectItemCaseSensitive(msg, "wake");
    node->next_wakeup_ns = cJSON_IsNumber(wake)
                         ? (int64_t)wake->valuedouble : INT64_MAX;

    const cJSON *out = cJSON_GetObjectItemCaseSensitive(msg, "out");
    if (cJSON_IsArray(out)) {
        cJSON *ev = NULL;
        cJSON_ArrayForEach(ev, out) {
            apply_out_event(node, ev, step_t);
            if (node->failed) break;
        }
    }

    cJSON_Delete(msg);
    return node->failed ? -1 : 0;
}

/* ============================================================
 * Lifecycle
 * ============================================================ */

int ext_node_init(ext_node_t *node, const char *path, int node_id) {
    memset(node, 0, sizeof(*node));
    node->node_id        = node_id;
    node->sim_time_ns    = 0;
    node->next_wakeup_ns = INT64_MAX;
    node->child_pid      = -1;
    node->to_child       = -1;
    node->from_child     = -1;

    const char *tmo = getenv("CSIM_EXT_TIMEOUT_MS");
    node->timeout_ms = tmo ? atoi(tmo) : 5000;
    if (node->timeout_ms <= 0) node->timeout_ms = 5000;

    snprintf(node->command, sizeof(node->command), "%s", path ? path : "");

    int in_pipe[2], out_pipe[2];   /* in: csim->peer, out: peer->csim */
    if (pipe(in_pipe) != 0) {
        fprintf(stderr, "ext_node: pipe: %s\n", strerror(errno));
        return -1;
    }
    if (pipe(out_pipe) != 0) {
        fprintf(stderr, "ext_node: pipe: %s\n", strerror(errno));
        close(in_pipe[0]); close(in_pipe[1]);
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "ext_node: fork: %s\n", strerror(errno));
        close(in_pipe[0]);  close(in_pipe[1]);
        close(out_pipe[0]); close(out_pipe[1]);
        return -1;
    }
    if (pid == 0) {
        /* Child: stdin from csim, stdout to csim, stderr passes through so
         * the peer's own diagnostics reach the terminal unfiltered. */
        dup2(in_pipe[0], STDIN_FILENO);
        dup2(out_pipe[1], STDOUT_FILENO);
        close(in_pipe[0]);  close(in_pipe[1]);
        close(out_pipe[0]); close(out_pipe[1]);
        execl("/bin/sh", "sh", "-c", node->command, (char *)NULL);
        perror("ext_node: exec");
        _exit(127);
    }

    close(in_pipe[0]);
    close(out_pipe[1]);
    node->child_pid  = pid;
    node->to_child   = in_pipe[1];
    node->from_child = out_pipe[0];

    /* A peer that dies mid-run must not take csim down with SIGPIPE; the
     * write path reports EPIPE through ext_fail instead. */
    signal(SIGPIPE, SIG_IGN);
    return 0;
}

int ext_node_start(ext_node_t *node, double x, double y, uint32_t seed) {
    char line[512];
    int n = snprintf(line, sizeof(line),
                     "{\"type\":\"hello\",\"proto\":1,\"id\":%d,"
                     "\"x\":%.6g,\"y\":%.6g,\"seed\":%u,"
                     "\"max_frame\":%d}\n",
                     node->node_id, x, y, seed, EXT_NODE_MAX_FRAME);
    if (n < 0 || n >= (int)sizeof(line)) {
        ext_fail(node, "hello does not fit in %zu bytes", sizeof(line));
        return -1;
    }
    if (write_all(node, line, (size_t)n) != 0) return -1;

    /* The reply to hello is what carries the peer's first wakeup. */
    return consume_done(node, 0);
}

static void do_step(ext_node_t *node, int64_t when) {
    char line[128];
    int n = snprintf(line, sizeof(line),
                     "{\"type\":\"step\",\"t\":%lld,\"in\":[]}\n",
                     (long long)when);
    if (write_all(node, line, (size_t)n) != 0) return;
    consume_done(node, when);
}

void ext_node_step_until_ns(ext_node_t *node, int64_t target_ns) {
    /* Mirror js_node_step_until_ns: dispatch every event due at or before
     * target_ns, advancing sim_time_ns to each event's exact time. */
    while (!node->failed && node->next_wakeup_ns <= target_ns) {
        int64_t when = node->next_wakeup_ns;
        if (when > node->sim_time_ns) node->sim_time_ns = when;
        node->next_wakeup_ns = INT64_MAX;
        do_step(node, when);
    }
}

int64_t ext_node_next_wakeup_ns(const ext_node_t *node) {
    return node->failed ? INT64_MAX : node->next_wakeup_ns;
}

bool ext_node_failed(const ext_node_t *node) {
    return node->failed;
}

void ext_node_destroy(ext_node_t *node) {
    if (node->child_pid <= 0) return;

    if (!node->failed && node->to_child >= 0) {
        char line[128];
        int n = snprintf(line, sizeof(line),
                         "{\"type\":\"stop\",\"t\":%lld,\"reason\":\"end\"}\n",
                         (long long)node->sim_time_ns);
        (void)!write(node->to_child, line, (size_t)n);
    }

    if (node->to_child   >= 0) { close(node->to_child);   node->to_child = -1; }
    if (node->from_child >= 0) { close(node->from_child); node->from_child = -1; }

    /* Closing stdin is the peer's cue to exit; give it a moment before the
     * signals (sim_external_command.c's shape). */
    usleep(20000);
    kill(node->child_pid, SIGTERM);
    usleep(50000);
    kill(node->child_pid, SIGKILL);
    waitpid(node->child_pid, NULL, 0);
    node->child_pid = -1;
}
