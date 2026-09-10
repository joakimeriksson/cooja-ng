/*
 * shell_script — the command stream and script engine of the Cooja-NG shell
 * (see include/sim/shell_service.h).
 *
 * A state machine, not a thread: shell_script_tick() runs from the service
 * host's poll() at every slice boundary (sim now == the loop's sim_ns) and
 *   1. retries pending console bytes,
 *   2. runs `at` entries that are due,
 *   3. runs commands queued by `on` watches,
 *   4. resolves a blocking command (expect matched / timed out, sleep or
 *      wait-until reached),
 *   5. executes lines from the top of the source stack (a file, or the
 *      stdin queue at depth 0) until one blocks or the input runs dry.
 * Non-blocking commands run back to back within one tick, so a `sendln`
 * immediately followed by an `expect` arms the expect before the mote runs.
 * Every deadline is pinned on the event queue (SIM_EV_TEST_ACTION) so the
 * outer loop lands exactly on it; a match pins now+1µs so the script resumes
 * on the very next slice.  Nothing here reads the wall clock.
 */
#include "shell_internal.h"
#include "sim_runtime.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

const char *shell_dequeue_line(shell_service_t *s, char *buf, size_t len);

void shell_pin(shell_service_t *s, int64_t t) {
    if (t <= sim_runtime_now_ns(s->sim) || t == INT64_MAX) return;
    sim_eq_schedule_test_action(&s->sim->event_queue, t);
}

void shell_script_init(shell_service_t *s) {
    s->depth = 0;
    s->block = SHELL_BLOCK_NONE;
    s->atq_count = 0;
    s->watch_count = 0;
    s->trigger_count = 0;
}

/* --- sources ------------------------------------------------------------- */

int shell_script_source(shell_service_t *s, const char *path) {
    if (s->depth >= SHELL_SOURCE_DEPTH) {
        shell_out(s, "error: source nesting too deep (max %d)\n", SHELL_SOURCE_DEPTH);
        return -1;
    }
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    shell_source_t *src = &s->stack[s->depth++];
    src->f = f;
    src->lineno = 0;
    snprintf(src->path, sizeof(src->path), "%s", path);
    s->script_used = true;
    s->finished = false;
    return 0;
}

static void pop_source(shell_service_t *s) {
    if (s->depth <= 0) return;
    shell_source_t *src = &s->stack[--s->depth];
    if (src->f) fclose(src->f);
    src->f = NULL;
}

static void root_finished(shell_service_t *s) {
    s->finished = true;
    if (s->stop_when_done) sim_control_request_exit(s->ctl);
}

void shell_script_abort(shell_service_t *s) {
    while (s->depth > 0) pop_source(s);
    s->block = SHELL_BLOCK_NONE;
}

void shell_script_fail(shell_service_t *s, const char *reason) {
    s->script_used = true;
    if (!s->failed) {
        s->failed = true;
        snprintf(s->fail_reason, sizeof(s->fail_reason), "%s", reason);
    }
    shell_out(s, "SCRIPT FAILED: %s\n", reason);
    shell_script_abort(s);
    s->finished = true;
    if (s->stop_when_done) sim_control_request_exit(s->ctl);
}

void shell_script_pass(shell_service_t *s) {
    s->script_used = true;
    s->passed = true;
    shell_script_abort(s);
    root_finished(s);
}

/* --- blocking ------------------------------------------------------------ */

void shell_script_block_expect(shell_service_t *s, const char *pattern,
                               const int *ids, int nids, bool any,
                               int64_t timeout_ns) {
    int64_t now = sim_runtime_now_ns(s->sim);
    s->block = SHELL_BLOCK_EXPECT;
    s->block_start_ns = now;
    s->block_deadline_ns = now + timeout_ns;
    snprintf(s->expect_pattern, sizeof(s->expect_pattern), "%s", pattern);
    s->expect_any = any;
    s->expect_n = nids < SIM_EQ_MAX_NODES ? nids : SIM_EQ_MAX_NODES;
    memcpy(s->expect_ids, ids, (size_t)s->expect_n * sizeof(int));
    s->matched = false;
    s->script_used = true;
    shell_pin(s, s->block_deadline_ns);
}

void shell_script_block_until(shell_service_t *s, shell_block_t kind,
                              int64_t deadline_ns) {
    s->block = kind;
    s->block_start_ns = sim_runtime_now_ns(s->sim);
    s->block_deadline_ns = deadline_ns;
    shell_pin(s, deadline_ns);
}

/* --- at queue ------------------------------------------------------------ */

int shell_script_at_add(shell_service_t *s, int64_t at_ns, int64_t period_ns,
                        const char *cmd) {
    if (s->atq_count >= SHELL_ATQ_MAX) return -1;
    shell_at_entry_t *e = &s->atq[s->atq_count++];
    e->id = s->next_at_id++;
    e->at_ns = at_ns;
    e->period_ns = period_ns;
    snprintf(e->cmd, sizeof(e->cmd), "%s", cmd);
    shell_pin(s, at_ns);
    return e->id;
}

int shell_script_at_remove(shell_service_t *s, int id) {
    if (id < 0) { int n = s->atq_count; s->atq_count = 0; return n; }
    for (int i = 0; i < s->atq_count; i++) {
        if (s->atq[i].id != id) continue;
        memmove(&s->atq[i], &s->atq[i + 1],
                (size_t)(s->atq_count - i - 1) * sizeof(s->atq[0]));
        s->atq_count--;
        return 1;
    }
    return 0;
}

/* Pop the earliest due entry (lowest time, then lowest id). */
static bool at_pop_due(shell_service_t *s, int64_t now, shell_at_entry_t *out) {
    int best = -1;
    for (int i = 0; i < s->atq_count; i++) {
        if (s->atq[i].at_ns > now) continue;
        if (best < 0 || s->atq[i].at_ns < s->atq[best].at_ns ||
            (s->atq[i].at_ns == s->atq[best].at_ns && s->atq[i].id < s->atq[best].id))
            best = i;
    }
    if (best < 0) return false;
    *out = s->atq[best];
    if (out->period_ns > 0) {
        s->atq[best].at_ns += out->period_ns;
        shell_pin(s, s->atq[best].at_ns);
    } else {
        memmove(&s->atq[best], &s->atq[best + 1],
                (size_t)(s->atq_count - best - 1) * sizeof(s->atq[0]));
        s->atq_count--;
    }
    return true;
}

/* --- watches -------------------------------------------------------------- */

int shell_script_watch_add(shell_service_t *s, shell_watch_kind_t kind,
                           const char *pattern, const int *ids, int nids,
                           bool any, const char *cmd) {
    if (s->watch_count >= SHELL_WATCH_MAX) return -1;
    shell_watch_t *w = &s->watches[s->watch_count++];
    memset(w, 0, sizeof(*w));
    w->kind = kind;
    snprintf(w->pattern, sizeof(w->pattern), "%s", pattern);
    w->any = any;
    w->nids = nids < SIM_EQ_MAX_NODES ? nids : SIM_EQ_MAX_NODES;
    memcpy(w->ids, ids, (size_t)w->nids * sizeof(int));
    if (cmd) snprintf(w->cmd, sizeof(w->cmd), "%s", cmd);
    return s->watch_count - 1;
}

static bool sel_hit(bool any, const int *ids, int nids, int node_id) {
    if (any) return true;
    for (int i = 0; i < nids; i++) if (ids[i] == node_id) return true;
    return false;
}

void shell_script_on_log_line(shell_service_t *s, int idx, int node_id,
                              const char *line, int64_t ns) {
    (void)idx;
    for (int i = 0; i < s->watch_count; i++) {
        shell_watch_t *w = &s->watches[i];
        if (!sel_hit(w->any, w->ids, w->nids, node_id)) continue;
        if (!strstr(line, w->pattern)) continue;
        switch (w->kind) {
        case SHELL_WATCH_COUNT:
            w->count++;
            break;
        case SHELL_WATCH_FAIL:
            if (!s->pending_fail) {
                s->pending_fail = true;
                snprintf(s->pending_fail_reason, sizeof(s->pending_fail_reason),
                         "fail-on \"%s\" matched on node %d at %.3f s: %s",
                         w->pattern, node_id, (double)ns / 1e9, line);
                shell_pin(s, ns + 1000);
            }
            break;
        case SHELL_WATCH_RUN:
            if (s->trigger_count < SHELL_TRIGGER_MAX) {
                snprintf(s->triggers[s->trigger_count++], SHELL_LINE_MAX, "%s",
                         w->cmd);
                shell_pin(s, ns + 1000);
            }
            break;
        }
    }
    if (s->block == SHELL_BLOCK_EXPECT && !s->matched &&
        sel_hit(s->expect_any, s->expect_ids, s->expect_n, node_id) &&
        strstr(line, s->expect_pattern)) {
        s->matched = true;
        s->matched_node = node_id;
        s->matched_ns = ns;
        snprintf(s->matched_line, sizeof(s->matched_line), "%s", line);
        /* Resume on the very next slice (scheduling from an observer is
         * fine; executing commands here is not). */
        shell_pin(s, ns + 1000);
    }
}

/* --- the tick -------------------------------------------------------------- */

static void resolve_block(shell_service_t *s, int64_t now) {
    switch (s->block) {
    case SHELL_BLOCK_NONE:
        return;
    case SHELL_BLOCK_EXPECT:
        if (s->matched) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_pass++;
            if (s->verbose)
                shell_out(s, "  expect \"%s\": matched on node %d at %.3f s\n",
                          s->expect_pattern, s->matched_node,
                          (double)s->matched_ns / 1e9);
        } else if (now >= s->block_deadline_ns) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_fail++;
            char reason[SHELL_REASON_MAX];
            snprintf(reason, sizeof(reason),
                     "expect \"%s\" timed out after %.3f s (at %.3f s)",
                     s->expect_pattern,
                     (double)(s->block_deadline_ns - s->block_start_ns) / 1e9,
                     (double)now / 1e9);
            shell_script_fail(s, reason);
        }
        return;
    case SHELL_BLOCK_SLEEP:
    case SHELL_BLOCK_WAIT_UNTIL:
        if (now >= s->block_deadline_ns) s->block = SHELL_BLOCK_NONE;
        return;
    case SHELL_BLOCK_RUN:
        /* Released by the pause itself — the run-for horizon, a spent step
         * budget, or a `!pause` typed meanwhile. */
        if (sim_control_paused(s->ctl)) s->block = SHELL_BLOCK_NONE;
        return;
    }
}

/* Next line for the stream: the top file, else the stdin queue. */
static const char *next_line(shell_service_t *s, char *buf, size_t len) {
    while (s->depth > 0) {
        shell_source_t *src = &s->stack[s->depth - 1];
        if (fgets(buf, (int)len, src->f)) {
            src->lineno++;
            size_t n = strlen(buf);
            while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';
            return buf;
        }
        pop_source(s);
        if (s->depth == 0 && !s->finished) root_finished(s);
    }
    return shell_dequeue_line(s, buf, len);
}

void shell_script_tick(shell_service_t *s) {
    if (!s->active) return;
    int64_t now = sim_runtime_now_ns(s->sim);
    sim_control_send_flush_pending(s->ctl);

    shell_at_entry_t e;
    int guard = 0;
    while (at_pop_due(s, now, &e) && guard++ < SHELL_ATQ_MAX * 4) {
        if (s->verbose) shell_out(s, "at #%d> %s\n", e.id, e.cmd);
        shell_exec_line(s, e.cmd, false);
    }
    for (int i = 0; i < s->trigger_count; i++) {
        if (s->verbose) shell_out(s, "on> %s\n", s->triggers[i]);
        shell_exec_line(s, s->triggers[i], false);
    }
    s->trigger_count = 0;

    if (s->pending_fail) {
        s->pending_fail = false;
        shell_script_fail(s, s->pending_fail_reason);
    }
    resolve_block(s, now);

    char line[SHELL_LINE_MAX];
    int lines = 0;
    while (s->block == SHELL_BLOCK_NONE &&
           !sim_runtime_stop_requested(s->sim) && lines++ < 10000) {
        bool from_file = s->depth > 0;
        const char *l = next_line(s, line, sizeof(line));
        if (!l) break;
        /* Echo: script lines always (when verbose), stdin lines when they
         * were not visibly typed at a terminal prompt.  Blank and
         * comment-only lines are not echoed. */
        const char *first = l;
        while (*first == ' ' || *first == '\t') first++;
        if (s->verbose && (from_file || !s->tty) && *first && *first != '#')
            shell_out(s, "> %s\n", l);
        shell_exec_line(s, l, false);
    }
}
