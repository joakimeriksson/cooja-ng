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
#include "sim_mote.h"
#include "arm_cpu.h"

#include <ctype.h>
#include <limits.h>
#include <regex.h>
#include <stdlib.h>
#include <string.h>

const char *shell_dequeue_line(shell_service_t *s, char *buf, size_t len);

/* A pin at or before now has nothing to land on (the block resolves at
 * the next tick anyway) and INT64_MAX is `step`'s "no deadline"; neither
 * is a caller bug, so they are skipped here rather than clamped. */
void shell_pin(shell_service_t *s, int64_t t) {
    if (t <= sim_runtime_now_ns(s->sim) || t == INT64_MAX) return;
    sim_schedule_test_action(s->sim, t);
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
    if (s->depth >= SHELL_SOURCE_DEPTH) return -1;   /* `source` reports it */
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    shell_source_t *src = &s->stack[s->depth++];
    src->f = f;
    src->lineno = 0;
    src->send_idx = -1;
    src->send_id = -1;
    src->send_timeout_ns = 0;
    src->nframes = 0;
    snprintf(src->path, sizeof(src->path), "%s", path);
    s->script_used = true;
    s->finished = false;
    return 0;
}

int shell_script_sendfile(shell_service_t *s, const char *path, int idx,
                          int node_id, int64_t timeout_ns) {
    bool finished = s->finished;
    if (shell_script_source(s, path) != 0) return -1;
    shell_source_t *src = &s->stack[s->depth - 1];
    src->send_idx = idx;
    src->send_id = node_id;
    src->send_timeout_ns = timeout_ns;
    s->finished = finished;    /* a sendfile is not a new script */
    return 0;
}

static void pop_source(shell_service_t *s) {
    if (s->depth <= 0) return;
    shell_source_t *src = &s->stack[--s->depth];
    if (src->f) fclose(src->f);
    src->f = NULL;
}

/* The root script is done.  With --script alone the run ends here — unless
 * the script left `at`/`every` entries behind, which keep it running until
 * they have fired (or the duration ends the run).  `pass`/`fail` force the
 * end. */
static void root_finished(shell_service_t *s, bool force) {
    s->finished = true;
    if (!s->stop_when_done) return;
    if (force || s->atq_count == 0) {
        sim_control_request_exit(s->ctl);
    } else if (!s->waiting_note) {
        s->waiting_note = true;
        shell_out(s, "script ended; running on for %d scheduled command(s)\n",
                  s->atq_count);
    }
}

void shell_script_abort(shell_service_t *s) {
    while (s->depth > 0) pop_source(s);
    s->block = SHELL_BLOCK_NONE;
    shell_regex_free(&s->expect_re);
    shell_regex_free(&s->cmd_re);
}

/* --- variables and matching --------------------------------------------- */

bool shell_var_name_ok(const char *name) {
    if (!name || !(isalpha((unsigned char)name[0]) || name[0] == '_')) return false;
    size_t n = 0;
    for (const char *p = name; *p; p++, n++)
        if (!(isalnum((unsigned char)*p) || *p == '_')) return false;
    return n < SHELL_VAR_NAME_MAX;
}

const char *shell_var_get(void *user, const char *name) {
    shell_service_t *s = (shell_service_t *)user;
    for (int i = 0; i < s->var_count; i++)
        if (strcmp(s->vars[i].name, name) == 0) return s->vars[i].value;
    return NULL;
}

int shell_var_set(shell_service_t *s, const char *name, const char *value) {
    if (!shell_var_name_ok(name)) return -1;
    for (int i = 0; i < s->var_count; i++) {
        if (strcmp(s->vars[i].name, name) == 0) {
            snprintf(s->vars[i].value, sizeof(s->vars[i].value), "%s", value);
            return 0;
        }
    }
    if (s->var_count >= SHELL_VARS_MAX) return -1;
    shell_var_t *v = &s->vars[s->var_count++];
    snprintf(v->name, sizeof(v->name), "%s", name);
    snprintf(v->value, sizeof(v->value), "%s", value);
    return 0;
}

void *shell_regex_compile(const char *pattern, char *err, size_t errlen) {
    regex_t *re = malloc(sizeof(*re));
    if (!re) { if (err) snprintf(err, errlen, "out of memory"); return NULL; }
    int rc = regcomp(re, pattern, REG_EXTENDED);
    if (rc != 0) {
        char msg[128];
        regerror(rc, re, msg, sizeof(msg));
        if (err) snprintf(err, errlen, "bad regex \"%s\": %s", pattern, msg);
        free(re);
        return NULL;
    }
    return re;
}

void shell_regex_free(void **re) {
    if (!re || !*re) return;
    regfree((regex_t *)*re);
    free(*re);
    *re = NULL;
}

bool shell_line_match(const char *line, const char *pattern, void *re,
                      char *cap, size_t caplen) {
    if (re) {
        regmatch_t m[2];
        if (regexec((regex_t *)re, line, 2, m, 0) != 0) return false;
        if (cap && caplen) {
            regmatch_t g = (m[1].rm_so >= 0) ? m[1] : m[0];
            int len = (int)(g.rm_eo - g.rm_so);
            if (len >= (int)caplen) len = (int)caplen - 1;
            memcpy(cap, line + g.rm_so, (size_t)len);
            cap[len] = '\0';
        }
        return true;
    }
    if (!strstr(line, pattern)) return false;
    if (cap && caplen) snprintf(cap, caplen, "%s", line);
    return true;
}

void shell_script_fail_code(shell_service_t *s, int code, const char *reason) {
    s->script_used = true;
    if (!s->failed) {
        s->failed = true;
        s->fail_code = code;
        snprintf(s->fail_reason, sizeof(s->fail_reason), "%s", reason);
    }
    shell_out(s, "SCRIPT FAILED: %s\n", reason);
    shell_script_abort(s);
    root_finished(s, true);
}

/* An assertion-shaped failure: a false assert, an expect timeout, fail-on,
 * or the script's own `fail`. */
void shell_script_fail(shell_service_t *s, const char *reason) {
    shell_script_fail_code(s, SHELL_EXIT_ASSERT, reason);
}

void shell_script_pass(shell_service_t *s) {
    s->script_used = true;
    s->passed = true;
    shell_script_abort(s);
    root_finished(s, true);
}

/* --- blocking ------------------------------------------------------------ */

void shell_script_block_expect2(shell_service_t *s, shell_block_t kind,
                                const char *pattern, void *re, int needed,
                                const char *var, const int *ids, int nids,
                                bool any, int64_t timeout_ns) {
    int64_t now = sim_runtime_now_ns(s->sim);
    shell_regex_free(&s->expect_re);
    s->expect_re = re;
    s->block = kind;
    s->block_start_ns = now;
    s->block_deadline_ns = now + timeout_ns;
    snprintf(s->expect_pattern, sizeof(s->expect_pattern), "%s", pattern);
    snprintf(s->expect_var, sizeof(s->expect_var), "%s", var ? var : "");
    s->expect_needed = needed > 0 ? needed : 1;
    s->expect_seen = 0;
    s->expect_any = any;
    s->expect_n = nids < SIM_EQ_MAX_NODES ? nids : SIM_EQ_MAX_NODES;
    memcpy(s->expect_ids, ids, (size_t)s->expect_n * sizeof(int));
    s->matched = false;
    s->matched_capture[0] = '\0';
    s->script_used = true;
    shell_pin(s, s->block_deadline_ns);
}

void shell_script_block_expect(shell_service_t *s, const char *pattern,
                               const int *ids, int nids, bool any,
                               int64_t timeout_ns) {
    shell_script_block_expect2(s, SHELL_BLOCK_EXPECT, pattern, NULL, 1, NULL,
                               ids, nids, any, timeout_ns);
}

void shell_script_cmd_capture(shell_service_t *s, const char *var, void *re) {
    shell_regex_free(&s->cmd_re);
    s->cmd_re = re;
    snprintf(s->cmd_var, sizeof(s->cmd_var), "%s", var);
    s->cmd_captured = false;
    s->cmd_capture[0] = '\0';
    s->script_used = true;
}

void shell_script_block_fault(shell_service_t *s, int idx, int node_id,
                              unsigned mask, const char *what,
                              const uint64_t *base, int64_t timeout_ns) {
    int64_t now = sim_runtime_now_ns(s->sim);
    s->block = SHELL_BLOCK_FAULT;
    s->block_start_ns = now;
    s->block_deadline_ns = now + timeout_ns;
    s->fault_idx = idx;
    s->fault_id = node_id;
    s->fault_mask = mask;
    snprintf(s->fault_what, sizeof(s->fault_what), "%s", what);
    memcpy(s->fault_base, base, sizeof(s->fault_base));
    s->script_used = true;
    shell_pin(s, s->block_deadline_ns);
    shell_pin(s, now + SHELL_MS_TO_NS);   /* polled every simulated ms */
}

void shell_script_block_until(shell_service_t *s, shell_block_t kind,
                              int64_t deadline_ns) {
    s->block = kind;
    s->block_start_ns = sim_runtime_now_ns(s->sim);
    s->block_deadline_ns = deadline_ns;
    shell_pin(s, deadline_ns);
}

void shell_script_block_cmd(shell_service_t *s, int idx, int node_id,
                            const char *text, const char *expect,
                            const char *fail_on, int64_t timeout_ns) {
    int64_t now = sim_runtime_now_ns(s->sim);
    s->block = SHELL_BLOCK_CMD;
    s->block_start_ns = now;
    s->block_deadline_ns = now + timeout_ns;
    s->cmd_idx = idx;
    s->cmd_id = node_id;
    snprintf(s->cmd_text, sizeof(s->cmd_text), "%s", text);
    snprintf(s->cmd_expect, sizeof(s->cmd_expect), "%s", expect ? expect : "");
    snprintf(s->cmd_fail_on, sizeof(s->cmd_fail_on), "%s", fail_on ? fail_on : "");
    s->cmd_expect_seen = s->cmd_fail_seen = s->cmd_prompt_seen = false;
    s->cmd_fail_line[0] = '\0';
    s->cmd_lines = 0;
    /* Bytes already on the node's current line (say, the previous prompt)
     * are not this command's prompt: match only what arrives from now. */
    s->cmd_plen = 0;
    s->cmd_partial[0] = '\0';
    s->cmd_candidate_len = -1;
    shell_regex_free(&s->cmd_re);          /* `cmd -c` re-arms it after this */
    s->cmd_var[0] = '\0';
    if ((expect && expect[0]) || (fail_on && fail_on[0])) s->script_used = true;
    shell_pin(s, s->block_deadline_ns);
}

void shell_script_on_uart_byte(shell_service_t *s, int idx, uint8_t byte,
                               int64_t ns) {
    if (s->block != SHELL_BLOCK_CMD || idx != s->cmd_idx || s->cmd_prompt_seen)
        return;
    (void)ns;
    s->cmd_candidate_len = -1;        /* any new byte cancels a candidate */
    if (byte == '\n') { s->cmd_plen = 0; s->cmd_partial[0] = '\0'; return; }
    if (byte == '\r') return;
    if (s->cmd_plen >= (int)sizeof(s->cmd_partial) - 1) return;  /* long line: no prompt */
    s->cmd_partial[s->cmd_plen++] = (char)byte;
    s->cmd_partial[s->cmd_plen] = '\0';
    if (shell_glob_match(s->prompt_glob, s->cmd_partial)) {
        /* A candidate: it becomes the prompt if the console stays quiet.
         * Console bytes carry the mote's own clock, which can trail the
         * kernel's; use kernel time so the check and the pin are exact. */
        int64_t now = sim_runtime_now_ns(s->sim);
        s->cmd_candidate_len = s->cmd_plen;
        s->cmd_candidate_ns = now;
        shell_pin(s, now + SHELL_PROMPT_QUIET_NS);
    }
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
    e->origin = s->origin;
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
    w->origin = s->origin;
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
            if (w->dead) break;
            if (w->once) w->dead = true;
            if (s->trigger_count < SHELL_TRIGGER_MAX) {
                shell_trigger_t *t = &s->triggers[s->trigger_count++];
                snprintf(t->cmd, sizeof(t->cmd), "%s", w->cmd);
                t->origin.kind = SHELL_ORIGIN_ON;
                t->origin.script = w->origin.script;
                snprintf(t->origin.where, sizeof(t->origin.where), "on \"%.40s\" (%.60s)",
                         w->pattern, w->origin.where);
                shell_pin(s, ns + 1000);
            } else {
                s->triggers_dropped++;
            }
            break;
        }
    }
    if (s->block == SHELL_BLOCK_CMD && idx == s->cmd_idx && !s->cmd_prompt_seen) {
        s->cmd_lines++;
        if (s->cmd_re && !s->cmd_captured &&
            shell_line_match(line, "", s->cmd_re, s->cmd_capture, sizeof(s->cmd_capture)))
            s->cmd_captured = true;
        if (s->cmd_expect[0] && strstr(line, s->cmd_expect))
            s->cmd_expect_seen = true;
        if (s->cmd_fail_on[0] && !s->cmd_fail_seen && strstr(line, s->cmd_fail_on)) {
            s->cmd_fail_seen = true;
            snprintf(s->cmd_fail_line, sizeof(s->cmd_fail_line), "%s", line);
        }
    }
    if ((s->block == SHELL_BLOCK_EXPECT || s->block == SHELL_BLOCK_EXPECT_NOT) &&
        !s->matched &&
        sel_hit(s->expect_any, s->expect_ids, s->expect_n, node_id) &&
        shell_line_match(line, s->expect_pattern, s->expect_re,
                         s->matched_capture, sizeof(s->matched_capture)) &&
        ++s->expect_seen >= s->expect_needed) {
        s->matched = true;
        s->matched_node = node_id;
        s->matched_ns = ns;
        snprintf(s->matched_line, sizeof(s->matched_line), "%s", line);
        /* Resume on the very next slice (scheduling from an observer is
         * fine; executing commands here is not).  Console lines carry the
         * mote's clock, which may trail the kernel's. */
        int64_t now = sim_runtime_now_ns(s->sim);
        shell_pin(s, (ns > now ? ns : now) + 1000);
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
            shell_regex_free(&s->expect_re);
            if (s->expect_var[0] && shell_var_set(s, s->expect_var, s->matched_capture) != 0) {
                shell_script_fail(s, "too many variables");
                return;
            }
            if (s->verbose) {
                if (s->expect_var[0])
                    shell_out(s, "  expect \"%s\": matched on node %d at %.3f s, %s = \"%s\"\n",
                              s->expect_pattern, s->matched_node,
                              (double)s->matched_ns / 1e9, s->expect_var, s->matched_capture);
                else
                    shell_out(s, "  expect \"%s\": matched on node %d at %.3f s\n",
                              s->expect_pattern, s->matched_node,
                              (double)s->matched_ns / 1e9);
            }
        } else if (now >= s->block_deadline_ns) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_fail++;
            shell_regex_free(&s->expect_re);
            char reason[SHELL_REASON_MAX];
            if (s->expect_needed > 1)
                snprintf(reason, sizeof(reason),
                         "expect \"%s\" timed out after %.3f s (at %.3f s), matched %d/%d",
                         s->expect_pattern,
                         (double)(s->block_deadline_ns - s->block_start_ns) / 1e9,
                         (double)now / 1e9, s->expect_seen, s->expect_needed);
            else
                snprintf(reason, sizeof(reason),
                         "expect \"%s\" timed out after %.3f s (at %.3f s)",
                         s->expect_pattern,
                         (double)(s->block_deadline_ns - s->block_start_ns) / 1e9,
                         (double)now / 1e9);
            shell_script_fail(s, reason);
        }
        return;
    case SHELL_BLOCK_EXPECT_NOT:
        if (s->matched) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_fail++;
            shell_regex_free(&s->expect_re);
            char reason[SHELL_REASON_MAX];
            snprintf(reason, sizeof(reason),
                     "expect-not \"%.100s\": node %d printed it at %.3f s: %.200s",
                     s->expect_pattern, s->matched_node, (double)s->matched_ns / 1e9,
                     s->matched_line);
            shell_script_fail(s, reason);
        } else if (now >= s->block_deadline_ns) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_pass++;
            shell_regex_free(&s->expect_re);
            if (s->verbose)
                shell_out(s, "  expect-not \"%s\": not printed in %.3f s\n", s->expect_pattern,
                          (double)(s->block_deadline_ns - s->block_start_ns) / 1e9);
        }
        return;
    case SHELL_BLOCK_FAULT: {
        void *iface = s->ctl->ops.get_interface
            ? s->ctl->ops.get_interface(s->ctl->ops.user, s->fault_idx, SIM_MOTE_IFACE_ARM_CPU)
            : NULL;
        const arm_cpu_t *cpu = (const arm_cpu_t *)iface;
        int hit = 0;
        for (int k = 1; cpu && k < 16; k++) {
            if (!(s->fault_mask & (1u << k))) continue;
            if (cpu->exc_entry_count[k] < s->fault_base[k])      /* node rebooted */
                s->fault_base[k] = 0;
            if (cpu->exc_entry_count[k] > s->fault_base[k]) { hit = k; break; }
        }
        char reason[SHELL_REASON_MAX];
        if (!cpu) {
            s->block = SHELL_BLOCK_NONE;
            snprintf(reason, sizeof(reason), "expect-fault: node %d has no ARM CPU", s->fault_id);
            shell_script_fail(s, reason);
        } else if (hit) {
            static const char *names[16] = { [3] = "HardFault", [4] = "MemManage",
                [5] = "BusFault", [6] = "UsageFault", [7] = "SecureFault" };
            s->block = SHELL_BLOCK_NONE;
            s->expect_pass++;
            if (s->verbose)
                shell_out(s, "  expect-fault: node %d took %s (pc 0x%08x, from %s) by %.3f s\n",
                          s->fault_id, names[hit] ? names[hit] : "?",
                          cpu->last_fault_pc, cpu->last_fault_bg_secure ? "Secure" : "Non-secure",
                          (double)now / 1e9);
        } else if (now >= s->block_deadline_ns) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_fail++;
            snprintf(reason, sizeof(reason), "expect-fault: node %d took no %s within %.3f s",
                     s->fault_id, s->fault_what,
                     (double)(s->block_deadline_ns - s->block_start_ns) / 1e9);
            shell_script_fail(s, reason);
        } else {
            shell_pin(s, now + SHELL_MS_TO_NS);
        }
        return;
    }
    case SHELL_BLOCK_SLEEP:
    case SHELL_BLOCK_WAIT_UNTIL:
        if (now >= s->block_deadline_ns) s->block = SHELL_BLOCK_NONE;
        return;
    case SHELL_BLOCK_RUN:
        /* Released by the pause itself — the run-for horizon, a spent step
         * budget, or a `!pause` typed meanwhile. */
        if (sim_control_paused(s->ctl)) s->block = SHELL_BLOCK_NONE;
        return;
    case SHELL_BLOCK_CMD: {
        char reason[SHELL_REASON_MAX];
        if (!s->cmd_prompt_seen && s->cmd_candidate_len >= 0 &&
            s->cmd_candidate_len == s->cmd_plen &&
            now >= s->cmd_candidate_ns + SHELL_PROMPT_QUIET_NS) {
            s->cmd_prompt_seen = true;
            s->cmd_prompt_ns = s->cmd_candidate_ns;
        }
        if (s->cmd_prompt_seen) {
            s->block = SHELL_BLOCK_NONE;
            bool had_re = s->cmd_re != NULL;
            shell_regex_free(&s->cmd_re);
            if (had_re && !s->cmd_captured) {
                s->cmd_fail++;
                snprintf(reason, sizeof(reason),
                         "cmd %d \"%.60s\": nothing to capture into %s before the prompt",
                         s->cmd_id, s->cmd_text, s->cmd_var);
                shell_script_fail(s, reason);
            } else if (had_re && shell_var_set(s, s->cmd_var, s->cmd_capture) != 0) {
                shell_script_fail(s, "too many variables");
            } else if (s->cmd_expect[0] && !s->cmd_expect_seen) {
                s->cmd_fail++;
                snprintf(reason, sizeof(reason),
                         "cmd %d \"%.60s\": \"%.100s\" was not printed before the prompt",
                         s->cmd_id, s->cmd_text, s->cmd_expect);
                shell_script_fail(s, reason);
            } else if (s->cmd_fail_seen) {
                s->cmd_fail++;
                snprintf(reason, sizeof(reason), "cmd %d \"%.60s\" printed \"%.60s\": %.200s",
                         s->cmd_id, s->cmd_text, s->cmd_fail_on, s->cmd_fail_line);
                shell_script_fail(s, reason);
            } else {
                s->cmd_pass++;
                if (s->verbose)
                    shell_out(s, "  cmd %d \"%s\": prompt at %.3f s (%d line%s)\n",
                              s->cmd_id, s->cmd_text, (double)s->cmd_prompt_ns / 1e9,
                              s->cmd_lines, s->cmd_lines == 1 ? "" : "s");
            }
        } else if (now >= s->block_deadline_ns) {
            s->block = SHELL_BLOCK_NONE;
            shell_regex_free(&s->cmd_re);
            s->cmd_fail++;
            snprintf(reason, sizeof(reason),
                     "cmd %d \"%.60s\": no prompt matching \"%s\" within %.3f s",
                     s->cmd_id, s->cmd_text, s->prompt_glob,
                     (double)(s->block_deadline_ns - s->block_start_ns) / 1e9);
            shell_script_fail(s, reason);
        }
        return;
    }
    }
}

/* Next line for the stream: the top file, else the stdin queue (read
 * synchronously from a pipe when it is empty).  Sets s->origin. */
static const char *next_line(shell_service_t *s, char *buf, size_t len) {
    while (s->depth > 0) {
        shell_source_t *src = &s->stack[s->depth - 1];
        if (fgets(buf, (int)len, src->f)) {
            src->lineno++;
            size_t n = strlen(buf);
            while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';
            const char *base = strrchr(src->path, '/');
            s->origin.kind = SHELL_ORIGIN_FILE;
            s->origin.script = true;
            snprintf(s->origin.where, sizeof(s->origin.where), "%.100s:%d",
                     base ? base + 1 : src->path, src->lineno);
            s->line_is_send = src->send_idx >= 0;
            s->line_send_idx = src->send_idx;
            s->line_send_id = src->send_id;
            s->line_send_timeout_ns = src->send_timeout_ns;
            return buf;
        }
        if (src->nframes > 0) {
            snprintf(s->origin.where, sizeof(s->origin.where), "%.100s:%d",
                     strrchr(src->path, '/') ? strrchr(src->path, '/') + 1 : src->path,
                     src->lineno);
            s->origin.kind = SHELL_ORIGIN_FILE;
            s->origin.script = true;
            src->nframes = 0;
            shell_error(s, "end of file inside a %s block (missing `end`)",
                        src->frames[0].kind == 1 ? "repeat" : "if");
            if (s->depth == 0) continue;     /* the error aborted every source */
        }
        pop_source(s);
        if (s->depth == 0 && !s->finished) root_finished(s, false);
    }
    /* Also after EOF: a burst larger than the queue leaves lines buffered,
     * and the implied `exit` is queued only once they have all run. */
    if (s->qcount == 0 && s->sync_stdin &&
        !sim_runtime_stop_requested(s->sim))
        shell_read_stdin_sync(s);
    s->origin.kind = SHELL_ORIGIN_STDIN;
    /* A piped session is a script: its lines run in order, nobody is there
     * to read an error, and the exit code is the only verdict.  A typo at a
     * terminal prompt still only prints. */
    s->origin.script = s->sync_stdin;
    s->line_is_send = false;
    snprintf(s->origin.where, sizeof(s->origin.where), "stdin");
    if (s->sync_stdin) s->script_used = true;
    return shell_dequeue_line(s, buf, len);
}

void shell_script_tick(shell_service_t *s) {
    if (!s->active) return;
    int64_t now = sim_runtime_now_ns(s->sim);
    sim_control_send_flush_pending(s->ctl);

    shell_at_entry_t e;
    int guard = 0;
    while (at_pop_due(s, now, &e) && guard++ < SHELL_ATQ_MAX * 4) {
        shell_hold_output(s);
        if (s->verbose) shell_out(s, "at #%d> %s\n", e.id, e.cmd);
        shell_origin_t o = { .kind = SHELL_ORIGIN_AT, .script = e.origin.script };
        snprintf(o.where, sizeof(o.where), "at #%d (%.100s)", e.id, e.origin.where);
        shell_exec_line(s, e.cmd, false, &o);
    }
    for (int i = 0; i < s->trigger_count; i++) {
        shell_hold_output(s);
        if (s->verbose) shell_out(s, "on> %s\n", s->triggers[i].cmd);
        shell_exec_line(s, s->triggers[i].cmd, false, &s->triggers[i].origin);
    }
    s->trigger_count = 0;
    for (int i = 0; i < s->watch_count; i++) {       /* drop fired --once watches */
        if (!s->watches[i].dead) continue;
        memmove(&s->watches[i], &s->watches[i + 1],
                (size_t)(s->watch_count - i - 1) * sizeof(s->watches[0]));
        s->watch_count--;
        i--;
    }
    if (s->triggers_dropped) {
        shell_out(s, "warning: %d `on` command(s) dropped: more than %d fired "
                  "between two slices\n", s->triggers_dropped, SHELL_TRIGGER_MAX);
        s->triggers_dropped = 0;
    }

    if (s->pending_fail) {
        s->pending_fail = false;
        shell_script_fail(s, s->pending_fail_reason);
    }
    resolve_block(s, now);

    char line[SHELL_LINE_MAX];
    int lines = 0;
    while (s->block == SHELL_BLOCK_NONE && !s->restart_pending &&
           !sim_runtime_stop_requested(s->sim) && lines++ < 10000) {
        bool from_file = s->depth > 0;
        const char *l = next_line(s, line, sizeof(line));
        if (!l) break;
        if (s->line_is_send) {
            /* A sendfile line: straight to the node, then wait for its prompt. */
            shell_hold_output(s);
            if (s->verbose) shell_out(s, "> [sendfile %d] %s\n", s->line_send_id, l);
            int len = (int)strlen(l);
            char text[SHELL_LINE_MAX + 1];
            memcpy(text, l, (size_t)len);
            text[len] = '\n';
            if (!sim_control_node_active(s->ctl, s->line_send_idx)) {
                shell_error(s, "sendfile: node %d is not running", s->line_send_id);
                continue;
            }
            if (s->max_line > 0 && len >= s->max_line)
                shell_out(s, "warning: a line of %d bytes reaches max-line %d; it may be truncated\n",
                          len, s->max_line);
            int took = sim_control_send(s->ctl, s->line_send_id, (const uint8_t *)text,
                                        len + 1, SIM_CONTROL_WAKE | SIM_CONTROL_RETRY);
            if (took < len + 1) {
                shell_error(s, "sendfile: node %d console input truncated", s->line_send_id);
                continue;
            }
            shell_script_block_cmd(s, s->line_send_idx, s->line_send_id, l, NULL, NULL,
                                   s->line_send_timeout_ns);
            continue;
        }
        /* Echo: script lines always (when verbose), stdin lines when they
         * were not visibly typed at a terminal prompt.  Blank and
         * comment-only lines are not echoed. */
        const char *first = l;
        while (*first == ' ' || *first == '\t') first++;
        if (*first && *first != '#') shell_hold_output(s);
        if (s->verbose && (from_file || !s->editor) && *first && *first != '#')
            shell_out(s, "> %s\n", l);
        /* A pipe's "!" lines are queued in order; run them as plain lines. */
        if (!from_file && *first) shell_transcript_record(s, first);
        if (!from_file && *first == '!') first++;
        shell_origin_t o = s->origin;
        shell_exec_line(s, from_file ? l : first, false, &o);
    }

    /* --script alone: a finished script that was waiting for its scheduled
     * commands ends the run once they have all fired. */
    if (s->finished && s->stop_when_done && s->atq_count == 0 &&
        !sim_runtime_stop_requested(s->sim))
        sim_control_request_exit(s->ctl);
}
