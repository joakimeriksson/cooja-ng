/*
 * shell_commands — the command table and handlers of the Cooja-NG shell
 * (docs/shell.md).  Every mutation goes through sim_control; blocking and
 * scheduling commands hand over to the engine in shell_script.c.
 */
#include "shell_internal.h"
#include "sim_runtime.h"
#include "sim_mote.h"
#include "arm_cpu.h"
#include "arm_trustzone.h"
#include "elf_loader.h"

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

/* --- helpers --------------------------------------------------------------- */

static int64_t now_ns(shell_service_t *s) { return sim_runtime_now_ns(s->sim); }

bool shell_refuse_external_clock(shell_service_t *s, const char *what) {
    if (!s->sim || !s->sim->clock_source) return false;
    shell_error(s, "%s is not available while an external clock source "
                "(e.g. Renode) drives the simulation", what);
    return true;
}

/* A run/step typed as "!cmd" runs beside the stream and must not hold it;
 * from the stream itself it holds it until the auto-pause. */
static void hold_stream_for_run(shell_service_t *s, int64_t deadline_ns,
                                const char *line) {
    /* Remember what left the simulation paused: the auto-pause at the end of
     * a `run <d>` is what makes the *next* blocking command a deadlock, and
     * the message should say so. */
    snprintf(s->run_pause_cmd, sizeof(s->run_pause_cmd), "%s", line ? line : "run");
    snprintf(s->run_pause_where, sizeof(s->run_pause_where), "%s",
             s->origin.where[0] ? s->origin.where : "stdin");
    if (s->exec_immediate) return;
    shell_script_block_until(s, SHELL_BLOCK_RUN, deadline_ns);
}

/* All node ids in slot order (removed slots included). */
static int all_ids(shell_service_t *s, int *ids, int max) {
    int n = sim_control_node_count(s->ctl), k = 0;
    for (int i = 0; i < n && k < max; i++) {
        sim_control_node_info_t info;
        if (sim_control_describe(s->ctl, i, &info)) ids[k++] = info.id;
    }
    return k;
}

int shell_resolve_selector(shell_service_t *s, const char *sel, int *ids,
                           int max, bool allow_any, bool *any) {
    int all[SIM_EQ_MAX_NODES];
    int n = all_ids(s, all, SIM_EQ_MAX_NODES);
    char err[128];
    int rc = shell_parse_selector(sel, all, n, ids, max, allow_any, any,
                                  err, sizeof(err));
    if (rc < 0) shell_error(s, "%s", err);
    return rc;
}

/* Parse an absolute-or-relative time argument into an absolute instant. */
static int parse_instant(shell_service_t *s, const char *arg, int64_t *out) {
    int64_t t; bool rel = false;
    if (shell_parse_time(arg, &t, &rel) != 0) {
        shell_error(s, "bad time '%s' (e.g. 5s, 250ms, +2s)", arg);
        return -1;
    }
    *out = rel ? now_ns(s) + t : t;
    return 0;
}

static int parse_dur(shell_service_t *s, const char *arg, int64_t *out) {
    if (shell_parse_duration(arg, out) != 0) {
        shell_error(s, "bad duration '%s' (e.g. 5s, 250ms)", arg);
        return -1;
    }
    return 0;
}

/* Join argv[from..] with single spaces into buf. */
static void join_args(int argc, char **argv, int from, char *buf, size_t len) {
    size_t p = 0;
    buf[0] = '\0';
    for (int i = from; i < argc; i++) {
        int n = snprintf(buf + p, len - p, "%s%s", i > from ? " " : "", argv[i]);
        if (n < 0 || (size_t)n >= len - p) break;
        p += (size_t)n;
    }
}

static const char *node_state(const sim_control_node_info_t *info, int64_t now) {
    if (info->removed) return "removed";
    if (now < info->start_ns) return "pending";
    return "running";
}

/* --- simulation control ------------------------------------------------- */

static int cmd_run(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argpos;
    if (shell_refuse_external_clock(s, "run")) return -1;
    if (argc >= 2) {
        int64_t d;
        if (parse_dur(s, argv[1], &d) != 0) return -1;
        sim_control_run_for(s->ctl, d);
        s->run_for_target_ns = now_ns(s) + d;
        /* Hold the command stream until the auto-pause, so `run 500ms`
         * followed by `status` in a script/pipe sees the later time. */
        hold_stream_for_run(s, s->run_for_target_ns, line);
    } else {
        sim_control_resume(s->ctl);
    }
    return 0;
}

static int cmd_pause(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (shell_refuse_external_clock(s, "pause")) return -1;
    sim_control_pause(s->ctl);
    return 0;
}

static int cmd_step(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argpos;
    if (shell_refuse_external_clock(s, "step")) return -1;
    if (argc < 2) {
        sim_control_step_events(s->ctl, 1);
        hold_stream_for_run(s, INT64_MAX, line);
        return 0;
    }
    long n;
    if (shell_parse_int(argv[1], &n) == 0) {
        if (n < 1) { shell_error(s, "step count must be >= 1"); return -1; }
        sim_control_step_events(s->ctl, (int)(n > INT_MAX ? INT_MAX : n));
        hold_stream_for_run(s, INT64_MAX, line);
        return 0;
    }
    int64_t d;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    sim_control_run_for(s->ctl, d);
    hold_stream_for_run(s, now_ns(s) + d, line);
    return 0;
}

static int cmd_speed(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 2) {
        double r = sim_control_speed(s->ctl);
        if (r > 0) shell_out(s, "speed: %.3gx real-time\n", r);
        else shell_out(s, "speed: max (unpaced)\n");
        return 0;
    }
    if (shell_refuse_external_clock(s, "speed")) return -1;
    if (strcmp(argv[1], "max") == 0) { sim_control_set_speed(s->ctl, 0.0); return 0; }
    if (strcmp(argv[1], "realtime") == 0) { sim_control_set_speed(s->ctl, 1.0); return 0; }
    double r;
    if (shell_parse_double(argv[1], &r) != 0 || r <= 0.0) {
        shell_error(s, "speed: expected a positive ratio, 'max' or 'realtime'");
        return -1;
    }
    sim_control_set_speed(s->ctl, r);
    return 0;
}

static int cmd_status(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    int64_t now = now_ns(s);
    int n = sim_control_node_count(s->ctl), active = 0;
    for (int i = 0; i < n; i++) if (sim_control_node_active(s->ctl, i)) active++;
    double r = sim_control_speed(s->ctl);
    char spd[32];
    if (r > 0) snprintf(spd, sizeof(spd), "%.3gx", r); else snprintf(spd, sizeof(spd), "max");
    shell_out(s, "  time: %.3f s  state: %s  speed: %s  nodes: %d (%d active)\n",
              (double)now / 1e9,
              sim_control_paused(s->ctl) ? "paused" : "running", spd, n, active);
    if (sim_control_pause_at(s->ctl) != INT64_MAX)
        shell_out(s, "  auto-pause at %.3f s\n", (double)sim_control_pause_at(s->ctl) / 1e9);
    char origin[64];
    if (s->depth > 0)
        shell_out(s, "  script: %s\n", shell_origin(s, origin, sizeof(origin)));
    switch (s->block) {
    case SHELL_BLOCK_EXPECT:
        shell_out(s, "  blocked: expect \"%s\" until %.3f s\n", s->expect_pattern,
                  (double)s->block_deadline_ns / 1e9);
        break;
    case SHELL_BLOCK_SLEEP:
        shell_out(s, "  blocked: sleep until %.3f s\n", (double)s->block_deadline_ns / 1e9);
        break;
    case SHELL_BLOCK_WAIT_UNTIL:
        shell_out(s, "  blocked: wait-until %.3f s\n", (double)s->block_deadline_ns / 1e9);
        break;
    case SHELL_BLOCK_RUN:
        shell_out(s, "  blocked: run/step until the auto-pause\n");
        break;
    case SHELL_BLOCK_CMD:
        shell_out(s, "  blocked: cmd %d \"%s\" waiting for a prompt matching \"%s\" until %.3f s\n",
                  s->cmd_id, s->cmd_text, s->prompt_glob, (double)s->block_deadline_ns / 1e9);
        break;
    default: break;
    }
    if (s->qcount > 0) shell_out(s, "  queued input lines: %d\n", s->qcount);
    if (s->atq_count > 0) shell_out(s, "  scheduled commands: %d (atq)\n", s->atq_count);
    if (s->watch_count > 0) shell_out(s, "  watches: %d\n", s->watch_count);
    if (s->logfile_count > 0) shell_out(s, "  log files: %d\n", s->logfile_count);
    if (s->script_used)
        shell_out(s, "  expects: %d passed, %d failed%s%s\n", s->expect_pass, s->expect_fail,
                  s->failed ? "  FAILED: " : "", s->failed ? s->fail_reason : "");
    return 0;
}

static int cmd_time(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    int64_t now = now_ns(s);
    shell_out(s, "%.6f s (%lld ns)\n", (double)now / 1e9, (long long)now);
    return 0;
}

static int cmd_exit(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    /* `exit` inside a script file ends that script normally: it counts as
     * finished, not as a script cut short by the end of the run. */
    if (s->origin.kind == SHELL_ORIGIN_FILE) {
        shell_script_abort(s);
        s->finished = true;
    }
    s->exited = true;
    sim_control_request_exit(s->ctl);
    return 0;
}

/* --- nodes ----------------------------------------------------------------- */

static int cmd_nodes(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    int n = sim_control_node_count(s->ctl);
    int64_t now = now_ns(s);
    shell_out(s, "  %4s  %-7s %-8s %8s %8s %10s  %s\n", "id", "kind", "state", "x", "y", "time", "firmware");
    for (int i = 0; i < n; i++) {
        sim_control_node_info_t info;
        if (!sim_control_describe(s->ctl, i, &info)) continue;
        shell_out(s, "  %4d  %-7s %-8s %8.1f %8.1f %9.3fs  %s\n", info.id,
                  info.type ? info.type : "?", node_state(&info, now), info.x, info.y,
                  (double)info.sim_time_ns / 1e9, info.firmware ? info.firmware : "");
    }
    return 0;
}

static int cmd_add(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc == 4) { shell_error(s, "usage: add <firmware|type> [id] [x y]"); return -1; }
    int id = -1;
    double x = 0, y = 0;
    if (argc >= 3) {
        long v;
        if (shell_parse_int(argv[2], &v) != 0 || v < 0) { shell_error(s, "bad node id '%s'", argv[2]); return -1; }
        id = (int)v;
    }
    if (argc >= 5) {
        if (shell_parse_double(argv[3], &x) != 0 || shell_parse_double(argv[4], &y) != 0) {
            shell_error(s, "bad position '%s %s'", argv[3], argv[4]); return -1;
        }
    }
    if (id < 0) {
        int ids[SIM_EQ_MAX_NODES];
        int n = all_ids(s, ids, SIM_EQ_MAX_NODES);
        id = 0;
        for (int i = 0; i < n; i++) if (ids[i] > id) id = ids[i];
        id++;
    } else if (sim_control_index_of_id(s->ctl, id) >= 0) {
        shell_error(s, "node id %d already exists (use reboot)", id);
        return -1;
    }
    int idx = sim_control_add(s->ctl, argv[1], NULL, id, x, y,
                              SIM_CONTROL_START | SIM_CONTROL_WAKE);
    if (idx < 0) { shell_error(s, "cannot add node %d from '%s'", id, argv[1]); return -1; }
    /* Inherit the console mask default. */
    s->console_mask[idx] = s->verbose ? 1 : 0;
    shell_out(s, "added node %d at (%.1f, %.1f)\n", id, x, y);
    return 0;
}

static int cmd_remove(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, argv[1], ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    for (int i = 0; i < n; i++) {
        if (sim_control_remove(s->ctl, ids[i]) != 0) { shell_error(s, "cannot remove node %d", ids[i]); return -1; }
        if (s->verbose) shell_out(s, "removed node %d\n", ids[i]);
    }
    return 0;
}

static int cmd_move(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    long id; double x, y;
    if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "bad node id '%s'", argv[1]); return -1; }
    if (shell_parse_double(argv[2], &x) != 0 || shell_parse_double(argv[3], &y) != 0) {
        shell_error(s, "bad position '%s %s'", argv[2], argv[3]); return -1;
    }
    if (sim_control_move(s->ctl, (int)id, x, y) != 0) { shell_error(s, "no node with id %ld", id); return -1; }
    if (s->verbose) shell_out(s, "moved node %ld to (%.1f, %.1f)\n", id, x, y);
    return 0;
}

static int cmd_reboot(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, argv[1], ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    for (int i = 0; i < n; i++) {
        if (sim_control_reboot(s->ctl, ids[i], SIM_CONTROL_WAKE) != 0) {
            shell_error(s, "cannot reboot node %d", ids[i]); return -1;
        }
        if (s->verbose) shell_out(s, "rebooted node %d\n", ids[i]);
    }
    return 0;
}

/* --- console --------------------------------------------------------------- */

static int set_mask(shell_service_t *s, uint8_t *mask, const char *sel, int value) {
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, sel, ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    for (int i = 0; i < n; i++) {
        int idx = sim_control_index_of_id(s->ctl, ids[i]);
        if (idx >= 0 && idx < SIM_EQ_MAX_NODES) mask[idx] = (uint8_t)value;
    }
    return 0;
}

static void show_mask(shell_service_t *s, const uint8_t *mask, const char *what) {
    int n = sim_control_node_count(s->ctl);
    char buf[512]; size_t p = 0;
    buf[0] = '\0';
    int on = 0;
    for (int i = 0; i < n && i < SIM_EQ_MAX_NODES; i++) {
        if (!mask[i]) continue;
        sim_control_node_info_t info;
        if (!sim_control_describe(s->ctl, i, &info)) continue;
        int k = snprintf(buf + p, sizeof(buf) - p, "%s%d", on ? "," : "", info.id);
        if (k < 0 || (size_t)k >= sizeof(buf) - p) break;
        p += (size_t)k;
        on++;
    }
    shell_out(s, "%s: %s\n", what, on ? buf : "(none)");
}

static int cmd_log(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 2) { show_mask(s, s->console_mask, "console output on"); return 0; }
    const char *sel = argc >= 3 ? argv[2] : "all";
    if (strcmp(argv[1], "on") == 0)  return set_mask(s, s->console_mask, sel, 1);
    if (strcmp(argv[1], "off") == 0) return set_mask(s, s->console_mask, sel, 0);
    if (strcmp(argv[1], "only") == 0) {
        if (argc < 3) { shell_error(s, "usage: log only <nodes>"); return -1; }
        memset(s->console_mask, 0, sizeof(s->console_mask));
        return set_mask(s, s->console_mask, argv[2], 1);
    }
    shell_error(s, "usage: log [on|off [nodes] | only <nodes>]");
    return -1;
}

static int cmd_logfile(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 2) {
        if (s->logfile_count == 0) { shell_out(s, "no log files\n"); return 0; }
        for (int i = 0; i < s->logfile_count; i++) {
            char what[SHELL_PATH_MAX + 16];
            snprintf(what, sizeof(what), "%s", s->logfiles[i].path);
            show_mask(s, s->logfiles[i].mask, what);
        }
        return 0;
    }
    if (strcmp(argv[1], "off") == 0) {
        for (int i = 0; i < s->logfile_count; i++) {
            if (argc >= 3 && strcmp(s->logfiles[i].path, argv[2]) != 0) continue;
            if (s->logfiles[i].f) fclose(s->logfiles[i].f);
            memmove(&s->logfiles[i], &s->logfiles[i + 1],
                    (size_t)(s->logfile_count - i - 1) * sizeof(s->logfiles[0]));
            s->logfile_count--;
            i--;
        }
        return 0;
    }
    shell_logfile_t *lf = NULL;
    for (int i = 0; i < s->logfile_count; i++)
        if (strcmp(s->logfiles[i].path, argv[1]) == 0) lf = &s->logfiles[i];
    if (!lf) {
        if (s->logfile_count >= SHELL_LOGFILES_MAX) {
            shell_error(s, "too many log files (max %d)", SHELL_LOGFILES_MAX); return -1;
        }
        FILE *f = fopen(argv[1], "a");
        if (!f) { shell_error(s, "cannot open %s for writing", argv[1]); return -1; }
        lf = &s->logfiles[s->logfile_count++];
        memset(lf, 0, sizeof(*lf));
        lf->f = f;
        snprintf(lf->path, sizeof(lf->path), "%s", argv[1]);
    }
    memset(lf->mask, 0, sizeof(lf->mask));
    if (set_mask(s, lf->mask, argc >= 3 ? argv[2] : "all", 1) != 0) return -1;
    if (s->verbose) shell_out(s, "logging to %s\n", lf->path);
    return 0;
}

/* The text after argument `from`, spacing preserved (shell_unquote_rest). */
static int rest_text(shell_service_t *s, const char *line, const int *argpos,
                     int argc, int from, char *out, size_t outlen) {
    if (argc <= from) { out[0] = '\0'; return 0; }
    char err[128];
    int n = shell_unquote_rest(line + argpos[from], out, outlen, err, sizeof(err));
    if (n < 0) { shell_error(s, "%s", err); return -1; }
    return n;
}

static int do_send(shell_service_t *s, int argc, char **argv, const char *line,
                   const int *argpos, bool newline) {
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, argv[1], ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    char text[SHELL_LINE_MAX];
    int len = rest_text(s, line, argpos, argc, 2, text, sizeof(text) - 1);
    if (len < 0) return -1;
    if (newline) text[len++] = '\n';
    /* A Contiki-NG node drops a line longer than its serial-line buffer;
     * say so rather than let an expect time out mysteriously. */
    if (s->max_line > 0) {
        int run = 0, longest = 0;
        for (int i = 0; i < len; i++) {
            if (text[i] == '\n' || text[i] == '\r') run = 0;
            else if (++run > longest) longest = run;
        }
        if (longest >= s->max_line)
            shell_out(s, "warning: a line of %d bytes reaches max-line %d (the node's "
                      "serial-line buffer); it may be truncated (set max-line 0 to silence)\n",
                      longest, s->max_line);
    }
    for (int i = 0; i < n; i++) {
        int idx = sim_control_index_of_id(s->ctl, ids[i]);
        if (idx < 0 || !sim_control_node_active(s->ctl, idx)) {
            shell_error(s, "node %d is not running", ids[i]); return -1;
        }
        int took = sim_control_send(s->ctl, ids[i], (const uint8_t *)text, len,
                                    SIM_CONTROL_WAKE | SIM_CONTROL_RETRY);
        if (took < len) {
            shell_error(s, "node %d: console input truncated, %d of %d bytes queued "
                        "(the node is not reading its console)", ids[i],
                        took < 0 ? 0 : took, len);
            return -1;
        }
    }
    return 0;
}

static int cmd_send(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    return do_send(s, argc, argv, line, argpos, false);
}

static int cmd_sendln(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    return do_send(s, argc, argv, line, argpos, true);
}

/* cmd [-e "<pat>"] [-f "<pat>"] [-t <timeout>] <node> [text...]
 * Send one line, then hold the stream until the node prints its shell
 * prompt again.  -e: the output must contain the pattern; -f: it must not. */
static int cmd_cmd(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    const char *expect = NULL, *fail_on = NULL, *cap_var = NULL, *cap_re = NULL;
    int64_t timeout = s->default_expect_timeout_ns;
    int i = 1;
    for (; i < argc && argv[i][0] == '-' && argv[i][1]; i++) {
        const char *o = argv[i];
        bool is_e = !strcmp(o, "-e") || !strcmp(o, "--expect");
        bool is_f = !strcmp(o, "-f") || !strcmp(o, "--fail-on");
        bool is_t = !strcmp(o, "-t") || !strcmp(o, "--timeout");
        bool is_c = !strcmp(o, "-c") || !strcmp(o, "--capture");
        if (!is_e && !is_f && !is_t && !is_c) { shell_error(s, "cmd: unknown option '%s'", o); return -1; }
        if (i + (is_c ? 2 : 1) >= argc) { shell_error(s, "cmd: %s needs %s", o, is_c ? "a variable and a regex" : "a value"); return -1; }
        i++;
        if (is_e) expect = argv[i];
        else if (is_f) fail_on = argv[i];
        else if (is_c) { cap_var = argv[i]; cap_re = argv[++i]; }
        else if (parse_dur(s, argv[i], &timeout) != 0) return -1;
    }
    if (cap_var && !shell_var_name_ok(cap_var)) { shell_error(s, "cmd: bad variable name '%s'", cap_var); return -1; }
    if ((expect && !expect[0]) || (fail_on && !fail_on[0])) {
        shell_error(s, "cmd: empty pattern"); return -1;
    }
    if (i >= argc) { shell_error(s, "usage: cmd [-e \"<pat>\"] [-f \"<pat>\"] [-t <timeout>] <node> [text...]"); return -1; }
    long id;
    if (shell_parse_int(argv[i], &id) != 0) { shell_error(s, "cmd: expected one node id, got '%s'", argv[i]); return -1; }
    int idx = sim_control_index_of_id(s->ctl, (int)id);
    if (idx < 0) { shell_error(s, "no node with id %ld", id); return -1; }
    if (!sim_control_node_active(s->ctl, idx)) { shell_error(s, "node %ld is not running", id); return -1; }

    char text[SHELL_LINE_MAX];
    int len = rest_text(s, line, argpos, argc, i + 1, text, sizeof(text) - 1);
    if (len < 0) return -1;
    if (s->max_line > 0 && len >= s->max_line)
        shell_out(s, "warning: a line of %d bytes reaches max-line %d (the node's "
                  "serial-line buffer); it may be truncated (set max-line 0 to silence)\n",
                  len, s->max_line);
    text[len] = '\n';
    int took = sim_control_send(s->ctl, (int)id, (const uint8_t *)text, len + 1,
                                SIM_CONTROL_WAKE | SIM_CONTROL_RETRY);
    text[len] = '\0';
    if (took < len + 1) {
        shell_error(s, "node %ld: console input truncated, %d of %d bytes queued",
                    id, took < 0 ? 0 : took, len + 1);
        return -1;
    }
    void *re = NULL;
    if (cap_re) {
        char err[160];
        if (!(re = shell_regex_compile(cap_re, err, sizeof(err)))) { shell_error(s, "%s", err); return -1; }
    }
    /* Armed in the same tick as the send, before the node runs a single
     * instruction, so neither its output nor its prompt can be missed. */
    shell_script_block_cmd(s, idx, (int)id, text, expect, fail_on, timeout);
    if (re) shell_script_cmd_capture(s, cap_var, re);
    return 0;
}

/* --- variables --------------------------------------------------------------- */

static int cmd_var(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    if (argc == 1) {
        if (s->var_count == 0) { shell_out(s, "no variables\n"); return 0; }
        for (int i = 0; i < s->var_count; i++)
            shell_out(s, "  %s = \"%s\"\n", s->vars[i].name, s->vars[i].value);
        return 0;
    }
    if (!strcmp(argv[1], "-d")) {
        if (argc != 3) { shell_error(s, "usage: var -d <name>"); return -1; }
        for (int i = 0; i < s->var_count; i++) {
            if (strcmp(s->vars[i].name, argv[2]) != 0) continue;
            memmove(&s->vars[i], &s->vars[i + 1], (size_t)(s->var_count - i - 1) * sizeof(s->vars[0]));
            s->var_count--;
            return 0;
        }
        shell_error(s, "undefined variable '%s'", argv[2]);
        return -1;
    }
    if (!shell_var_name_ok(argv[1])) { shell_error(s, "bad variable name '%s'", argv[1]); return -1; }
    if (argc == 2) {
        const char *v = shell_var_get(s, argv[1]);
        if (!v) { shell_error(s, "undefined variable '%s'", argv[1]); return -1; }
        shell_out(s, "%s\n", v);
        return 0;
    }
    char text[SHELL_LINE_MAX];
    if (rest_text(s, line, argpos, argc, 2, text, sizeof(text)) < 0) return -1;
    if (shell_var_set(s, argv[1], text) != 0) {
        shell_error(s, "too many variables (max %d)", SHELL_VARS_MAX); return -1;
    }
    return 0;
}

/* --- ARM node inspection ------------------------------------------------------ */

static arm_cpu_t *node_arm_cpu(shell_service_t *s, const char *what, const char *arg,
                               int *idx_out, sim_control_node_info_t *info) {
    long id;
    if (shell_parse_int(arg, &id) != 0) { shell_error(s, "%s: expected one node id, got '%s'", what, arg); return NULL; }
    int idx = sim_control_index_of_id(s->ctl, (int)id);
    if (idx < 0) { shell_error(s, "no node with id %ld", id); return NULL; }
    arm_cpu_t *cpu = s->ctl->ops.get_interface
        ? (arm_cpu_t *)s->ctl->ops.get_interface(s->ctl->ops.user, idx, SIM_MOTE_IFACE_ARM_CPU)
        : NULL;
    if (!cpu) { shell_error(s, "%s: node %ld has no ARM CPU", what, id); return NULL; }
    if (idx_out) *idx_out = idx;
    if (info) sim_control_describe(s->ctl, idx, info);
    return cpu;
}

/* "0x2000", "main", "main+0x10": a number, or a symbol of the node's
 * firmware (then its Secure-world image) plus an optional offset. */
static int resolve_addr(shell_service_t *s, const sim_control_node_info_t *info,
                        const char *spec, uint32_t *out) {
    long v;
    if (shell_parse_int(spec, &v) == 0) { *out = (uint32_t)v; return 0; }
    char name[128];
    snprintf(name, sizeof(name), "%s", spec);
    long off = 0;
    char *plus = strchr(name, '+');
    if (plus) {
        *plus = '\0';
        if (shell_parse_int(plus + 1, &off) != 0) { shell_error(s, "bad offset in '%s'", spec); return -1; }
    }
    uint32_t a = 0;
    if (info->firmware && info->firmware[0]) a = elf_find_symbol(info->firmware, name);
    if (!a && info->secure_firmware && info->secure_firmware[0])
        a = elf_find_symbol(info->secure_firmware, name);
    if (!a) { shell_error(s, "no symbol '%s' in node %d's firmware", name, info->id); return -1; }
    *out = a + (uint32_t)off;
    return 0;
}

/* Leading "-c <var>" on single-value commands; returns the index after it. */
static int take_capture_opt(shell_service_t *s, int argc, char **argv, const char **var) {
    *var = NULL;
    if (argc >= 3 && !strcmp(argv[1], "-c")) {
        if (!shell_var_name_ok(argv[2])) { shell_error(s, "bad variable name '%s'", argv[2]); return -1; }
        *var = argv[2];
        return 3;
    }
    return 1;
}

static void emit_value(shell_service_t *s, const char *var, const char *label, uint32_t v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "0x%08x", v);
    if (var) shell_var_set(s, var, buf);
    shell_out(s, "%s%s\n", label, buf);
}

static int cmd_sym(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    const char *var;
    int i = take_capture_opt(s, argc, argv, &var);
    if (i < 0) return -1;
    if (argc - i != 2) { shell_error(s, "usage: sym [-c <var>] <node> <symbol>"); return -1; }
    sim_control_node_info_t info;
    long id;
    if (shell_parse_int(argv[i], &id) != 0) { shell_error(s, "sym: expected one node id"); return -1; }
    int idx = sim_control_index_of_id(s->ctl, (int)id);
    if (idx < 0 || !sim_control_describe(s->ctl, idx, &info)) { shell_error(s, "no node with id %ld", id); return -1; }
    uint32_t a;
    if (resolve_addr(s, &info, argv[i + 1], &a) != 0) return -1;
    char label[160];
    snprintf(label, sizeof(label), "%s = ", argv[i + 1]);
    emit_value(s, var, label, a);
    return 0;
}

/* mem [-w] [-c var] <node> <addr|sym> [count]   read
 * mem [-w] <node> <addr|sym> = <value...>       write */
static int cmd_mem(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    bool words = false;
    const char *var = NULL;
    int i = 1;
    for (; i < argc && argv[i][0] == '-' && argv[i][1] && !isdigit((unsigned char)argv[i][1]); i++) {
        if (!strcmp(argv[i], "-w")) words = true;
        else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
            var = argv[++i];
            if (!shell_var_name_ok(var)) { shell_error(s, "bad variable name '%s'", var); return -1; }
        } else { shell_error(s, "mem: unknown option '%s'", argv[i]); return -1; }
    }
    if (argc - i < 2) { shell_error(s, "usage: mem [-w] [-c <var>] <node> <addr|symbol> [count] | mem [-w] <node> <addr> = <values...>"); return -1; }
    sim_control_node_info_t info;
    arm_cpu_t *cpu = node_arm_cpu(s, "mem", argv[i], NULL, &info);
    if (!cpu) return -1;
    uint32_t addr;
    if (resolve_addr(s, &info, argv[i + 1], &addr) != 0) return -1;
    int step = words ? 4 : 1;

    if (argc - i >= 3 && !strcmp(argv[i + 2], "=")) {
        if (argc - i < 4) { shell_error(s, "mem: nothing to write"); return -1; }
        uint32_t last = addr + (uint32_t)((argc - i - 3) * step) - 1;
        if (addr < cpu->flash_end && last >= cpu->flash_base) {
            shell_error(s, "mem: 0x%08x-0x%08x is flash, which is read-only here (as on hardware)", addr, last);
            return -1;
        }
        for (int k = i + 3; k < argc; k++) {
            long v;
            if (shell_parse_int(argv[k], &v) != 0) { shell_error(s, "mem: bad value '%s'", argv[k]); return -1; }
            uint32_t a = addr + (uint32_t)((k - i - 3) * step);
            for (int b = 0; b < step; b++)
                arm_write8(cpu, a + (uint32_t)b, (uint8_t)((unsigned long)v >> (8 * b)));
        }
        if (s->verbose) shell_out(s, "wrote %d %s at 0x%08x\n", argc - i - 3, words ? "word(s)" : "byte(s)", addr);
        return 0;
    }

    long count = words ? 8 : 64;
    if (argc - i >= 3 && shell_parse_int(argv[i + 2], &count) != 0) { shell_error(s, "mem: bad count '%s'", argv[i + 2]); return -1; }
    if (count < 1 || count > 4096) { shell_error(s, "mem: count must be 1..4096"); return -1; }
    if (var && count != 1) { shell_error(s, "mem: -c needs a count of 1"); return -1; }
    if (words) {
        for (long k = 0; k < count; k++) {
            uint32_t a = addr + (uint32_t)(k * 4), v = 0;
            for (int b = 0; b < 4; b++) v |= (uint32_t)arm_read8(cpu, a + (uint32_t)b) << (8 * b);
            if (var) { char buf[16]; snprintf(buf, sizeof(buf), "0x%08x", v); shell_var_set(s, var, buf); }
            if (k % 4 == 0) shell_out(s, "%s0x%08x:", k ? "\n" : "", a);
            shell_out(s, " 0x%08x", v);
        }
        shell_out(s, "\n");
        return 0;
    }
    for (long k = 0; k < count; k += 16) {
        char hex[64] = "", asc[20] = "";
        int n = (int)((count - k) < 16 ? (count - k) : 16);
        for (int b = 0; b < n; b++) {
            uint8_t v = arm_read8(cpu, addr + (uint32_t)(k + b));
            if (var) { char buf[8]; snprintf(buf, sizeof(buf), "0x%02x", v); shell_var_set(s, var, buf); }
            snprintf(hex + strlen(hex), sizeof(hex) - strlen(hex), " %02x", v);
            asc[b] = (v >= 32 && v < 127) ? (char)v : '.';
            asc[b + 1] = '\0';
        }
        shell_out(s, "0x%08x:%-48s |%s|\n", addr + (uint32_t)k, hex, asc);
    }
    return 0;
}

static int cmd_reg(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    const char *var;
    int i = take_capture_opt(s, argc, argv, &var);
    if (i < 0) return -1;
    /* reg <node> <name> = <value>: r0-r12, sp, lr, pc, xpsr. */
    if (!var && argc - i == 4 && !strcmp(argv[i + 2], "=")) {
        arm_cpu_t *cpu = node_arm_cpu(s, "reg", argv[i], NULL, NULL);
        if (!cpu) return -1;
        long v;
        if (shell_parse_int(argv[i + 3], &v) != 0) { shell_error(s, "reg: bad value '%s'", argv[i + 3]); return -1; }
        const char *n = argv[i + 1];
        int r = -1;
        if (n[0] == 'r' && isdigit((unsigned char)n[1])) {
            long k;
            if (shell_parse_int(n + 1, &k) == 0 && k >= 0 && k <= 12) r = (int)k;
        } else if (!strcmp(n, "sp")) r = ARM_SP;
        else if (!strcmp(n, "lr")) r = ARM_LR;
        else if (!strcmp(n, "pc")) r = ARM_PC;
        if (r == ARM_PC) cpu->reg[ARM_PC] = (uint32_t)v & ~1u;   /* Thumb bit is not part of PC */
        else if (r >= 0) cpu->reg[r] = (uint32_t)v;
        else if (!strcmp(n, "xpsr")) cpu->xpsr = (uint32_t)v;
        else { shell_error(s, "reg: can write r0-r12, sp, lr, pc, xpsr (not '%s')", n); return -1; }
        if (s->verbose) shell_out(s, "%s = 0x%08x\n", n, (uint32_t)v);
        return 0;
    }
    if (argc - i < 1 || argc - i > 2) { shell_error(s, "usage: reg [-c <var>] <node> [name] | reg <node> <name> = <value>"); return -1; }
    arm_cpu_t *cpu = node_arm_cpu(s, "reg", argv[i], NULL, NULL);
    if (!cpu) return -1;
    struct { const char *name; uint32_t v; bool tz; } r[] = {
        {"r0", cpu->reg[0], false}, {"r1", cpu->reg[1], false}, {"r2", cpu->reg[2], false}, {"r3", cpu->reg[3], false},
        {"r4", cpu->reg[4], false}, {"r5", cpu->reg[5], false}, {"r6", cpu->reg[6], false}, {"r7", cpu->reg[7], false},
        {"r8", cpu->reg[8], false}, {"r9", cpu->reg[9], false}, {"r10", cpu->reg[10], false}, {"r11", cpu->reg[11], false},
        {"r12", cpu->reg[12], false}, {"sp", cpu->reg[ARM_SP], false}, {"lr", cpu->reg[ARM_LR], false}, {"pc", cpu->reg[ARM_PC], false},
        {"xpsr", cpu->xpsr, false}, {"primask", cpu->primask, false}, {"basepri", cpu->basepri, false}, {"faultmask", cpu->faultmask, false},
        {"msp_s", cpu->msp_s, true}, {"psp_s", cpu->psp_s, true}, {"msp_ns", cpu->msp_ns, true},
        {"psp_ns", cpu->psp_ns, true}, {"control_s", cpu->control_s, true}, {"control_ns", cpu->control_ns, true},
    };
    int nr = (int)(sizeof(r) / sizeof(r[0]));
    if (argc - i == 2) {
        for (int k = 0; k < nr; k++) {
            if (strcmp(r[k].name, argv[i + 1]) != 0) continue;
            if (r[k].tz && !arm_cpu_has_trustzone(cpu)) break;
            char label[32];
            snprintf(label, sizeof(label), "%s = ", r[k].name);
            emit_value(s, var, label, r[k].v);
            return 0;
        }
        shell_error(s, "reg: no register '%s' on this CPU", argv[i + 1]);
        return -1;
    }
    if (var) { shell_error(s, "reg: -c needs a register name"); return -1; }
    int col = 0;
    for (int k = 0; k < nr; k++) {
        if (r[k].tz && !arm_cpu_has_trustzone(cpu)) continue;
        shell_out(s, "  %-10s 0x%08x%s", r[k].name, r[k].v, (++col % 4) ? "" : "\n");
    }
    if (col % 4) shell_out(s, "\n");
    if (arm_cpu_has_trustzone(cpu))
        shell_out(s, "  state: %s\n", arm_cpu_is_secure(cpu) ? "Secure" : "Non-secure");
    return 0;
}

static const char *const sfsr_bits[8] = {
    "INVEP", "INVIS", "INVER", "AUVIOL", "INVTRAN", "LSPERR", "SFARVALID", "LSERR"
};

static void print_sfsr(shell_service_t *s, const arm_cpu_t *cpu) {
    char bits[96] = "";
    for (int b = 0; b < 8; b++)
        if (cpu->sfsr & (1u << b))
            snprintf(bits + strlen(bits), sizeof(bits) - strlen(bits), "%s%s", bits[0] ? " " : "", sfsr_bits[b]);
    shell_out(s, "  SFSR 0x%08x [%s]  SFAR 0x%08x\n", cpu->sfsr, bits[0] ? bits : "-", cpu->sfar);
}

static int cmd_tz(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    sim_control_node_info_t info;
    arm_cpu_t *cpu = node_arm_cpu(s, "tz", argv[1], NULL, &info);
    if (!cpu) return -1;
    if (!arm_cpu_has_trustzone(cpu)) {
        shell_out(s, "node %d: no TrustZone-M (ARMv8-M security extension) on this CPU\n", info.id);
        return 0;
    }
    shell_out(s, "node %d: TrustZone-M, state %s, pc 0x%08x\n", info.id,
              arm_cpu_is_secure(cpu) ? "Secure" : "Non-secure", cpu->reg[ARM_PC]);
    shell_out(s, "  SG entries %llu  BXNS returns %llu  secure exceptions from NS %llu\n",
              (unsigned long long)cpu->tz_sg_count, (unsigned long long)cpu->tz_bxns_count,
              (unsigned long long)cpu->tz_secexc_count);
    print_sfsr(s, cpu);
    shell_out(s, "  SAU_CTRL 0x%08x [%s%s]\n", cpu->sau_ctrl,
              (cpu->sau_ctrl & ARM_SAU_CTRL_ENABLE) ? "ENABLE" : "disabled",
              (cpu->sau_ctrl & ARM_SAU_CTRL_ALLNS) ? " ALLNS" : "");
    for (int r = 0; r < 8; r++) {
        if (!(cpu->sau_rlar[r] & ARM_SAU_RLAR_ENABLE)) continue;
        shell_out(s, "  SAU region %d: 0x%08x-0x%08x %s\n", r, cpu->sau_rbar[r] & ~0x1fu,
                  (cpu->sau_rlar[r] & ~0x1fu) | 0x1fu,
                  (cpu->sau_rlar[r] & ARM_SAU_RLAR_NSC) ? "Non-secure callable" : "Non-secure");
    }
    shell_out(s, "  MSP_S 0x%08x  PSP_S 0x%08x  MSP_NS 0x%08x  PSP_NS 0x%08x\n",
              cpu->msp_s, cpu->psp_s, cpu->msp_ns, cpu->psp_ns);
    return 0;
}

static const char *const fault_names[16] = {
    [3] = "HardFault", [4] = "MemManage", [5] = "BusFault", [6] = "UsageFault", [7] = "SecureFault"
};

static int cmd_faults(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    sim_control_node_info_t info;
    arm_cpu_t *cpu = node_arm_cpu(s, "faults", argv[1], NULL, &info);
    if (!cpu) return -1;
    shell_out(s, "node %d fault entries:", info.id);
    for (int k = 3; k <= 7; k++)
        if (k != 7 || arm_cpu_has_trustzone(cpu))
            shell_out(s, " %s %llu", fault_names[k], (unsigned long long)cpu->exc_entry_count[k]);
    shell_out(s, "\n");
    if (cpu->last_fault_exc)
        shell_out(s, "  last: %s at pc 0x%08x, taken from %s\n", fault_names[cpu->last_fault_exc],
                  cpu->last_fault_pc, cpu->last_fault_bg_secure ? "Secure" : "Non-secure");
    if (arm_cpu_has_trustzone(cpu)) print_sfsr(s, cpu);
    return 0;
}

/* expect-fault <node> [kind[,kind...]|any] [timeout] */
static int cmd_expect_fault(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int idx;
    arm_cpu_t *cpu = node_arm_cpu(s, "expect-fault", argv[1], &idx, NULL);
    if (!cpu) return -1;
    unsigned mask = 0;
    const char *what = "fault (any)";
    int64_t timeout = s->default_expect_timeout_ns;
    int next = 2;
    if (argc > 2) {
        int64_t t;
        if (shell_parse_duration(argv[2], &t) == 0 && argc == 3) {
            timeout = t;
        } else {
            char kinds[128];
            snprintf(kinds, sizeof(kinds), "%s", argv[2]);
            for (char *tok = strtok(kinds, ","); tok; tok = strtok(NULL, ",")) {
                if (!strcmp(tok, "any")) mask |= 0xf8u;
                else if (!strcmp(tok, "hardfault")) mask |= 1u << 3;
                else if (!strcmp(tok, "memmanage") || !strcmp(tok, "memfault")) mask |= 1u << 4;
                else if (!strcmp(tok, "busfault")) mask |= 1u << 5;
                else if (!strcmp(tok, "usagefault")) mask |= 1u << 6;
                else if (!strcmp(tok, "securefault")) mask |= 1u << 7;
                else { shell_error(s, "expect-fault: unknown fault '%s' (hardfault, memmanage, busfault, usagefault, securefault, any)", tok); return -1; }
            }
            what = strcmp(argv[2], "any") ? argv[2] : "fault (any)";
            next = 3;
        }
    }
    if (argc > next && parse_dur(s, argv[next], &timeout) != 0) return -1;
    if (argc > next + 1) { shell_error(s, "usage: expect-fault <node> [kind[,kind]|any] [timeout]"); return -1; }
    if (!mask) mask = 0xf8u;
    long id = 0;
    shell_parse_int(argv[1], &id);
    shell_script_block_fault(s, idx, (int)id, mask, what, cpu->exc_entry_count, timeout);
    return 0;
}

static int cmd_console(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (!s->editor || !s->interactive || s->origin.kind != SHELL_ORIGIN_STDIN) {
        shell_error(s, "console needs an interactive terminal (--shell on a tty)");
        return -1;
    }
    long id;
    if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "console: expected one node id, got '%s'", argv[1]); return -1; }
    int idx = sim_control_index_of_id(s->ctl, (int)id);
    if (idx < 0) { shell_error(s, "no node with id %ld", id); return -1; }
    if (shell_console_enter(s, idx, (int)id) != 0) {
        shell_error(s, "console: cannot set up the terminal");
        return -1;
    }
    return 0;
}

/* --- scheduling ------------------------------------------------------------ */

/* Validate a command that at/every/on will run later.  Blocking commands
 * are refused: they would take over the command stream's own wait. */
static int check_command_text(shell_service_t *s, const char *what, const char *cmd) {
    char *argv[SHELL_MAX_ARGS]; char storage[SHELL_LINE_MAX]; char err[128];
    int argc = shell_tokenize(cmd, argv, NULL, SHELL_MAX_ARGS, storage, sizeof(storage), err, sizeof(err));
    if (argc < 0) { shell_error(s, "%s", err); return -1; }
    if (argc == 0) { shell_error(s, "missing command"); return -1; }
    if (!shell_find_command(argv[0])) { shell_error(s, "unknown command '%s'", argv[0]); return -1; }
    if (shell_line_blocks(cmd)) {
        shell_error(s, "%s cannot run '%s': it would block the command stream "
                    "(put the sequence in a script instead)", what, argv[0]);
        return -1;
    }
    return 0;
}

static int cmd_atq(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos);
static int cmd_atrm(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos);

static int cmd_at(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    int64_t t;
    /* `at list` / `at clear` manage the queue (atq / atrm are aliases). */
    if (strcmp(argv[1], "list") == 0) {
        if (argc != 2) { shell_error(s, "usage: at list"); return -1; }
        return cmd_atq(s, 1, argv + 1, line, argpos);
    }
    if (strcmp(argv[1], "clear") == 0) {
        if (argc != 3) { shell_error(s, "usage: at clear <id>|all"); return -1; }
        return cmd_atrm(s, 2, argv + 1, line, argpos);
    }
    if (argc < 3) { shell_error(s, "usage: at <time> <command...>"); return -1; }
    if (shell_refuse_external_clock(s, "at")) return -1;
    if (parse_instant(s, argv[1], &t) != 0) return -1;
    if (t <= now_ns(s)) { shell_error(s, "at: time %.3f s is not in the future", (double)t / 1e9); return -1; }
    const char *cmd = line + argpos[2];
    if (check_command_text(s, "at", cmd) != 0) return -1;
    int id = shell_script_at_add(s, t, 0, cmd);
    if (id < 0) { shell_error(s, "at: queue full (max %d)", SHELL_ATQ_MAX); return -1; }
    if (s->verbose) shell_out(s, "at #%d scheduled for %.3f s\n", id, (double)t / 1e9);
    return 0;
}

static int cmd_every(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc;
    int64_t d;
    if (shell_refuse_external_clock(s, "every")) return -1;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    if (d <= 0) { shell_error(s, "every: period must be > 0"); return -1; }
    const char *cmd = line + argpos[2];
    if (check_command_text(s, "every", cmd) != 0) return -1;
    int id = shell_script_at_add(s, now_ns(s) + d, d, cmd);
    if (id < 0) { shell_error(s, "every: queue full (max %d)", SHELL_ATQ_MAX); return -1; }
    if (s->verbose) shell_out(s, "every #%d scheduled, period %.3f s\n", id, (double)d / 1e9);
    return 0;
}

static int cmd_atq(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (s->atq_count == 0) { shell_out(s, "no scheduled commands\n"); return 0; }
    for (int i = 0; i < s->atq_count; i++) {
        const shell_at_entry_t *e = &s->atq[i];
        if (e->period_ns > 0)
            shell_out(s, "  #%-3d %10.3f s  every %.3f s  %s\n", e->id, (double)e->at_ns / 1e9,
                      (double)e->period_ns / 1e9, e->cmd);
        else
            shell_out(s, "  #%-3d %10.3f s  %s\n", e->id, (double)e->at_ns / 1e9, e->cmd);
    }
    return 0;
}

static int cmd_atrm(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (strcmp(argv[1], "all") == 0) { shell_script_at_remove(s, -1); return 0; }
    long id;
    if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "bad scheduled command id '%s'", argv[1]); return -1; }
    if (shell_script_at_remove(s, (int)id) == 0) { shell_error(s, "no scheduled command #%ld", id); return -1; }
    return 0;
}

/* --- scripting ------------------------------------------------------------- */

static int cmd_source(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    /* Checked here, not per attempt: the path is tried twice below, and
     * the depth is the same both times. */
    if (s->depth >= SHELL_SOURCE_DEPTH) {
        shell_error(s, "source nesting too deep (max %d)", SHELL_SOURCE_DEPTH);
        return -1;
    }
    /* From a script file, a relative path is looked up next to that file
     * first, then in the working directory. */
    if (s->origin.kind == SHELL_ORIGIN_FILE && s->depth > 0 && argv[1][0] != '/') {
        const char *cur = s->stack[s->depth - 1].path;
        const char *slash = strrchr(cur, '/');
        if (slash) {
            char path[SHELL_PATH_MAX];
            snprintf(path, sizeof(path), "%.*s/%s", (int)(slash - cur), cur, argv[1]);
            if (shell_script_source(s, path) == 0) return 0;
        }
    }
    if (shell_script_source(s, argv[1]) != 0) { shell_error(s, "source: cannot open %s", argv[1]); return -1; }
    return 0;
}

/* Shared by expect, expect-not and capture:
 *   [-re] [-n N] [-c var] <nodes|any> "<pattern>" [timeout]            */
static int expect_common(shell_service_t *s, shell_block_t kind, const char *what,
                         int argc, char **argv, const char *forced_var, bool force_re) {
    bool use_re = force_re;
    long needed = 1;
    const char *var = forced_var;
    int i = 1;
    for (; i < argc && argv[i][0] == '-' && argv[i][1] && !isdigit((unsigned char)argv[i][1]); i++) {
        if (!strcmp(argv[i], "-re")) use_re = true;
        else if (!strcmp(argv[i], "-n") && kind == SHELL_BLOCK_EXPECT && i + 1 < argc) {
            if (shell_parse_int(argv[++i], &needed) != 0 || needed < 1) { shell_error(s, "%s: -n needs a count >= 1", what); return -1; }
        } else if (!strcmp(argv[i], "-c") && kind == SHELL_BLOCK_EXPECT && i + 1 < argc) {
            var = argv[++i];
        } else { shell_error(s, "%s: unknown option '%s'", what, argv[i]); return -1; }
    }
    bool need_time = kind == SHELL_BLOCK_EXPECT_NOT;
    int left = argc - i;
    if (left < (need_time ? 3 : 2) || left > 3) {
        shell_error(s, "usage: %s", shell_find_command(what)->syntax); return -1;
    }
    if (var && !shell_var_name_ok(var)) { shell_error(s, "%s: bad variable name '%s'", what, var); return -1; }
    int ids[SIM_EQ_MAX_NODES]; bool any = false;
    int n = shell_resolve_selector(s, argv[i], ids, SIM_EQ_MAX_NODES, true, &any);
    if (n < 0) return -1;
    const char *pattern = argv[i + 1];
    if (!pattern[0]) { shell_error(s, "%s: empty pattern", what); return -1; }
    int64_t timeout = s->default_expect_timeout_ns;
    if (left == 3 && parse_dur(s, argv[i + 2], &timeout) != 0) return -1;
    void *re = NULL;
    if (use_re) {
        char err[160];
        if (!(re = shell_regex_compile(pattern, err, sizeof(err)))) { shell_error(s, "%s", err); return -1; }
    }
    shell_script_block_expect2(s, kind, pattern, re, (int)needed, var, ids, n, any, timeout);
    return 0;
}

static int cmd_expect(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    return expect_common(s, SHELL_BLOCK_EXPECT, "expect", argc, argv, NULL, false);
}

static int cmd_expect_not(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    return expect_common(s, SHELL_BLOCK_EXPECT_NOT, "expect-not", argc, argv, NULL, false);
}

/* capture <var> <nodes|any> "<regex>" [timeout] = expect -re -c <var> */
static int cmd_capture(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 4) { shell_error(s, "usage: %s", shell_find_command("capture")->syntax); return -1; }
    if (!shell_var_name_ok(argv[1])) { shell_error(s, "capture: bad variable name '%s'", argv[1]); return -1; }
    return expect_common(s, SHELL_BLOCK_EXPECT, "capture", argc - 1, argv + 1, argv[1], true);
}

/* sendfile [-t timeout] <node> <path> */
static int cmd_sendfile(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int64_t timeout = s->default_expect_timeout_ns;
    int i = 1;
    if (argc >= 3 && !strcmp(argv[1], "-t")) {
        if (parse_dur(s, argv[2], &timeout) != 0) return -1;
        i = 3;
    }
    if (argc - i != 2) { shell_error(s, "usage: sendfile [-t <timeout>] <node> <path>"); return -1; }
    long id;
    if (shell_parse_int(argv[i], &id) != 0) { shell_error(s, "sendfile: expected one node id, got '%s'", argv[i]); return -1; }
    int idx = sim_control_index_of_id(s->ctl, (int)id);
    if (idx < 0) { shell_error(s, "no node with id %ld", id); return -1; }
    const char *path = argv[i + 1];
    if (s->origin.kind == SHELL_ORIGIN_FILE && s->depth > 0 && path[0] != '/') {
        const char *cur = s->stack[s->depth - 1].path;
        const char *slash = strrchr(cur, '/');
        if (slash) {
            char rel[SHELL_PATH_MAX];
            snprintf(rel, sizeof(rel), "%.*s/%s", (int)(slash - cur), cur, path);
            if (shell_script_sendfile(s, rel, idx, (int)id, timeout) == 0) return 0;
        }
    }
    if (shell_script_sendfile(s, path, idx, (int)id, timeout) != 0) {
        shell_error(s, "sendfile: cannot open %s", path); return -1;
    }
    return 0;
}

static int cmd_sleep(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int64_t d;
    if (shell_refuse_external_clock(s, "sleep")) return -1;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    shell_script_block_until(s, SHELL_BLOCK_SLEEP, now_ns(s) + d);
    return 0;
}

static int cmd_wait_until(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int64_t t;
    if (shell_refuse_external_clock(s, "wait-until")) return -1;
    if (parse_instant(s, argv[1], &t) != 0) return -1;
    if (t <= now_ns(s)) {
        if (s->verbose) shell_out(s, "wait-until: %.3f s already passed\n", (double)t / 1e9);
        return 0;
    }
    shell_script_block_until(s, SHELL_BLOCK_WAIT_UNTIL, t);
    return 0;
}

static int cmd_assert(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    const char *what = argv[1];
    if (strcmp(what, "mem") == 0 && argc == 6) {
        sim_control_node_info_t info;
        arm_cpu_t *cpu = node_arm_cpu(s, "assert mem", argv[2], NULL, &info);
        if (!cpu) return -1;
        uint32_t addr;
        if (resolve_addr(s, &info, argv[3], &addr) != 0) return -1;
        long want;
        if (shell_parse_int(argv[5], &want) != 0) { shell_error(s, "assert mem: bad value '%s'", argv[5]); return -1; }
        uint32_t v = 0;
        for (int b = 0; b < 4; b++) v |= (uint32_t)arm_read8(cpu, addr + (uint32_t)b) << (8 * b);
        int r;
        if (!strcmp(argv[4], "==")) r = v == (uint32_t)want;
        else if (!strcmp(argv[4], "!=")) r = v != (uint32_t)want;
        else r = shell_compare((long)v, argv[4], (long)(uint32_t)want);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[4]); return -1; }
        if (!r) { shell_error(s, "assertion failed: mem %s 0x%08x (= 0x%08x) %s %s", argv[2], addr, v, argv[4], argv[5]); return -1; }
        return 0;
    }
    if (strcmp(what, "var") == 0 && argc == 5) {
        const char *v = shell_var_get(s, argv[2]);
        if (!v) { shell_error(s, "undefined variable '%s'", argv[2]); return -1; }
        long a, b;
        int r;
        if (shell_parse_int(v, &a) == 0 && shell_parse_int(argv[4], &b) == 0) r = shell_compare(a, argv[3], b);
        else if (!strcmp(argv[3], "==")) r = strcmp(v, argv[4]) == 0;
        else if (!strcmp(argv[3], "!=")) r = strcmp(v, argv[4]) != 0;
        else { shell_error(s, "assert var: '%s' needs numbers", argv[3]); return -1; }
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[3]); return -1; }
        if (!r) { shell_error(s, "assertion failed: %s (\"%s\") %s %s", argv[2], v, argv[3], argv[4]); return -1; }
        return 0;
    }
    if (strcmp(what, "time") == 0 && argc == 4) {
        int64_t t;
        if (parse_dur(s, argv[3], &t) != 0) return -1;
        int r = shell_compare((long)(now_ns(s) / 1000), argv[2], (long)(t / 1000));
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { shell_assert_failed(s, "assertion failed: time (%.6f s) %s %s", (double)now_ns(s) / 1e9, argv[2], argv[3]); return -1; }
        return 0;
    }
    if (strcmp(what, "nodes") == 0 && argc == 4) {
        long v;
        if (shell_parse_int(argv[3], &v) != 0) { shell_error(s, "assert: bad count '%s'", argv[3]); return -1; }
        int n = sim_control_node_count(s->ctl), active = 0;
        for (int i = 0; i < n; i++) if (sim_control_node_active(s->ctl, i)) active++;
        int r = shell_compare(active, argv[2], v);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { shell_assert_failed(s, "assertion failed: nodes (%d active) %s %ld", active, argv[2], v); return -1; }
        return 0;
    }
    if (strcmp(what, "node") == 0 && argc == 4) {
        long id;
        if (shell_parse_int(argv[2], &id) != 0) { shell_error(s, "assert: bad node id '%s'", argv[2]); return -1; }
        int idx = sim_control_index_of_id(s->ctl, (int)id);
        sim_control_node_info_t info;
        bool exists = idx >= 0 && sim_control_describe(s->ctl, idx, &info);
        bool ok;
        if (strcmp(argv[3], "exists") == 0) ok = exists;
        else if (strcmp(argv[3], "active") == 0) ok = exists && info.active;
        else if (strcmp(argv[3], "removed") == 0) ok = !exists || info.removed;
        else if (strcmp(argv[3], "secure") == 0 || strcmp(argv[3], "non-secure") == 0) {
            arm_cpu_t *cpu = node_arm_cpu(s, "assert node", argv[2], NULL, NULL);
            if (!cpu) return -1;
            if (!arm_cpu_has_trustzone(cpu)) { shell_error(s, "assert node: node %ld has no TrustZone-M", id); return -1; }
            ok = arm_cpu_is_secure(cpu) == (argv[3][0] == 's');
        }
        else { shell_error(s, "assert node: expected active|removed|exists|secure|non-secure"); return -1; }
        if (!ok) { shell_assert_failed(s, "assertion failed: node %ld %s", id, argv[3]); return -1; }
        return 0;
    }
    if (strcmp(what, "count") == 0 && argc == 5) {
        long v;
        if (shell_parse_int(argv[4], &v) != 0) { shell_error(s, "assert: bad count '%s'", argv[4]); return -1; }
        const shell_watch_t *w = NULL;
        for (int i = 0; i < s->watch_count; i++)
            if (s->watches[i].kind == SHELL_WATCH_COUNT && strcmp(s->watches[i].pattern, argv[2]) == 0)
                w = &s->watches[i];
        if (!w) { shell_error(s, "assert count: no `count \"%s\"` watch", argv[2]); return -1; }
        int r = shell_compare(w->count, argv[3], v);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[3]); return -1; }
        if (!r) { shell_assert_failed(s, "assertion failed: count \"%s\" (%d) %s %ld", argv[2], w->count, argv[3], v); return -1; }
        return 0;
    }
    shell_error(s, "usage: assert time <op> <t> | nodes <op> N | node <id> active|removed|exists|secure|non-secure | count \"pat\" <op> N");
    return -1;
}

static int cmd_pass(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    shell_script_pass(s);
    if (s->verbose) shell_out(s, "SCRIPT PASSED (%.3f s)\n", (double)now_ns(s) / 1e9);
    return 0;
}

static int cmd_fail(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    char msg[SHELL_REASON_MAX];
    if (argc >= 2) join_args(argc, argv, 1, msg, sizeof(msg));
    else snprintf(msg, sizeof(msg), "fail at %.3f s", (double)now_ns(s) / 1e9);
    shell_script_fail(s, msg);
    return 0;
}

static int cmd_set(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 2) {
        shell_out(s, "expect-timeout: %.3f s\n", (double)s->default_expect_timeout_ns / 1e9);
        shell_out(s, "max-line: %d%s\n", s->max_line, s->max_line ? "" : " (off)");
        shell_out(s, "prompt: \"%s\"\n", s->prompt_glob);
        return 0;
    }
    if (strcmp(argv[1], "prompt") == 0 && argc == 3) {
        if (!argv[2][0] || strlen(argv[2]) >= sizeof(s->prompt_glob)) {
            shell_error(s, "prompt: expected a pattern of 1..%d characters",
                        (int)sizeof(s->prompt_glob) - 1);
            return -1;
        }
        snprintf(s->prompt_glob, sizeof(s->prompt_glob), "%s", argv[2]);
        return 0;
    }
    if (strcmp(argv[1], "expect-timeout") == 0 && argc == 3) {
        int64_t d;
        if (parse_dur(s, argv[2], &d) != 0) return -1;
        s->default_expect_timeout_ns = d;
        return 0;
    }
    if (strcmp(argv[1], "max-line") == 0 && argc == 3) {
        long v;
        if (shell_parse_int(argv[2], &v) != 0 || v < 0 || v > SHELL_LINE_MAX) {
            shell_error(s, "max-line: expected 0..%d", SHELL_LINE_MAX); return -1;
        }
        s->max_line = (int)v;
        return 0;
    }
    shell_error(s, "usage: set [expect-timeout <duration> | max-line <bytes> | prompt \"<glob>\"]");
    return -1;
}

static int add_watch(shell_service_t *s, shell_watch_kind_t kind, const char *sel,
                     const char *pattern, const char *cmd) {
    int ids[SIM_EQ_MAX_NODES]; bool any = false;
    int n = shell_resolve_selector(s, sel, ids, SIM_EQ_MAX_NODES, true, &any);
    if (n < 0) return -1;
    if (!pattern[0]) { shell_error(s, "empty pattern"); return -1; }
    if (shell_script_watch_add(s, kind, pattern, ids, n, any, cmd) < 0) {
        shell_error(s, "too many watches (max %d)", SHELL_WATCH_MAX); return -1;
    }
    return 0;
}

static int cmd_fail_on(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    s->script_used = true;
    return add_watch(s, SHELL_WATCH_FAIL, argc >= 3 ? argv[2] : "any", argv[1], NULL);
}

static int cmd_count(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    return add_watch(s, SHELL_WATCH_COUNT, argc >= 3 ? argv[2] : "any", argv[1], NULL);
}

static int cmd_on(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc;
    const char *cmd = line + argpos[3];
    if (check_command_text(s, "on", cmd) != 0) return -1;
    return add_watch(s, SHELL_WATCH_RUN, argv[1], argv[2], cmd);
}

/* --- misc ------------------------------------------------------------------ */

static int cmd_echo(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argv;
    char text[SHELL_LINE_MAX];
    if (rest_text(s, line, argpos, argc, 1, text, sizeof(text)) < 0) return -1;
    shell_out(s, "%s\n", text);
    return 0;
}

static int cmd_save_config(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (!s->ctl->ops.save_config) { shell_error(s, "save-config is not available in this mode"); return -1; }
    if (s->ctl->ops.save_config(s->ctl->ops.user, argv[1]) != 0) {
        shell_error(s, "save-config: cannot write %s", argv[1]); return -1;
    }
    return 0;
}

static int cmd_help(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    shell_print_help(s, argc >= 2 ? argv[1] : NULL);
    return 0;
}

/* --- table ----------------------------------------------------------------- */

#define IMM SHELL_CMD_IMMEDIATE
#define BLK SHELL_CMD_BLOCKING
static const shell_command_t commands[] = {
    { "run",        "run [duration]",                  "resume; with a duration, pause again after it (run 500ms)", 0, 1, IMM, cmd_run },
    { "pause",      "pause",                           "stop dispatching events (services keep polling)", 0, 0, IMM, cmd_pause },
    { "step",       "step [N|duration]",               "run N events (default 1) or a duration, then pause", 0, 1, IMM | BLK, cmd_step },
    { "speed",      "speed [ratio|max|realtime]",      "wall-clock pacing: sim seconds per wall second; max = unpaced", 0, 1, IMM, cmd_speed },
    { "status",     "status",                          "time, run state, speed, node count, script/queue state", 0, 0, IMM, cmd_status },
    { "time",       "time",                            "print the simulation time", 0, 0, IMM, cmd_time },
    { "exit",       "exit",                            "end the run (normal teardown, reports, --save-config)", 0, 0, IMM, cmd_exit },
    { "quit",       "quit",                            "same as exit", 0, 0, IMM, cmd_exit },
    { "nodes",      "nodes",                           "list nodes: id, kind, state, position, time, firmware", 0, 0, IMM, cmd_nodes },
    { "add",        "add <firmware|type> [id] [x y]",  "add a node (config mote-type name or firmware path); id defaults to max+1", 1, 4, 0, cmd_add },
    { "remove",     "remove <nodes>",                  "stop nodes for good (start gate closed; reboot revives)", 1, 1, 0, cmd_remove },
    { "move",       "move <id> <x> <y>",               "set a node's position (metres) and recompute neighbours", 3, 3, 0, cmd_move },
    { "reboot",     "reboot <nodes>",                  "destroy + re-init nodes from their firmware, clock re-seeded to now", 1, 1, 0, cmd_reboot },
    { "log",        "log [on|off [nodes] | only <nodes>]", "console output to this terminal per node (default all on; off under -q)", 0, 2, IMM, cmd_log },
    { "log-file",   "log-file [<path> [nodes] | off [path]]", "append console lines of nodes to a file (same format); off closes", 0, 2, IMM, cmd_logfile },
    { "send",       "send <nodes> <text...>",          "console input (escapes like \\n honoured; no newline added)", 2, -1, 0, cmd_send },
    { "sendln",     "sendln <nodes> <text...>",        "send + one \"\\n\" — one Contiki-NG shell command", 2, -1, 0, cmd_sendln },
    { "cmd",        "cmd [-e \"<pat>\"] [-f \"<pat>\"] [-c <var> \"<regex>\"] [-t <timeout>] <node> [text...]", "sendln, then wait for the node's shell prompt; -e output must contain, -f must not, -c captures into $var", 1, -1, BLK, cmd_cmd },
    { "console",    "console <node>",                  "talk to a node's console directly (terminal only); ~. or Ctrl-D returns", 1, 1, IMM, cmd_console },
    { "at",         "at <time> <command...> | at list | at clear <id>|all", "run a command at a simulation time (5s, 1500ms, +2s); list or cancel scheduled commands", 1, -1, IMM, cmd_at },
    { "every",      "every <period> <command...>",     "run a command periodically, first after one period", 2, -1, IMM, cmd_every },
    { "atq",        "atq",                             "list scheduled commands (= at list)", 0, 0, IMM, cmd_atq },
    { "atrm",       "atrm <id>|all",                   "cancel scheduled command(s) (= at clear)", 1, 1, IMM, cmd_atrm },
    { "source",     "source <file>",                   "run a script file (nested up to 8 deep; relative to the calling script first)", 1, 1, BLK, cmd_source },
    { "expect",     "expect [-re] [-n N] [-c <var>] <nodes|any> \"<pattern>\" [timeout]", "block until N console lines (default 1) contain the pattern (-re: extended regex; -c: capture group 1 into $var); timeout fails the script", 2, -1, BLK, cmd_expect },
    { "expect-not", "expect-not [-re] <nodes|any> \"<pattern>\" <duration>", "block for a duration; a console line containing the pattern fails the script", 3, -1, BLK, cmd_expect_not },
    { "capture",    "capture <var> <nodes|any> \"<regex>\" [timeout]", "block until a line matches the regex; store group 1 (or the match) in $var", 3, 4, BLK, cmd_capture },
    { "var",        "var [<name> [value...] | -d <name>]", "list variables, show one, set one ($name / ${name} expand in later lines; $$ is a literal $)", 0, -1, IMM, cmd_var },
    { "sendfile",   "sendfile [-t <timeout>] <node> <path>", "send a file line by line, each as a cmd (waits for the prompt)", 2, 4, BLK, cmd_sendfile },
    { "sym",        "sym [-c <var>] <node> <symbol>",  "address of a firmware symbol (Non-secure image, then Secure image)", 2, 4, IMM, cmd_sym },
    { "mem",        "mem [-w] [-c <var>] <node> <addr|sym[+off]> [count] | mem [-w] <node> <addr> = <values...>", "read (hexdump, -w 32-bit words) or write memory, debugger view: no TrustZone checks, IO reads reach peripherals", 2, -1, IMM, cmd_mem },
    { "reg",        "reg [-c <var>] <node> [name] | reg <node> <name> = <value>", "read CPU registers (ARM; banked TrustZone stacks and CONTROL on ARMv8-M), or write r0-r12/sp/lr/pc/xpsr", 1, 4, IMM, cmd_reg },
    { "tz",         "tz <node>",                       "TrustZone-M state: security state, SG/BXNS/secure-exception counters, SFSR/SFAR, SAU regions, banked stacks", 1, 1, IMM, cmd_tz },
    { "faults",     "faults <node>",                   "fault exception counts, the last fault (pc, security state), SFSR/SFAR", 1, 1, IMM, cmd_faults },
    { "expect-fault","expect-fault <node> [kind[,kind]|any] [timeout]", "block until the node takes a fault (hardfault, memmanage, busfault, usagefault, securefault)", 1, 3, BLK, cmd_expect_fault },
    { "sleep",      "sleep <duration>",                "block for a simulated duration", 1, 1, BLK, cmd_sleep },
    { "wait-until", "wait-until <time>",               "block until a simulation time (absolute or +relative)", 1, 1, BLK, cmd_wait_until },
    { "assert",     "assert time|nodes <op> <v> | node <id> active|removed|exists|secure|non-secure | count \"pat\" <op> N | mem <node> <addr|sym> <op> <word> | var <name> <op> <value>", "check a condition; failure fails the script", 3, 5, 0, cmd_assert },
    { "pass",       "pass",                            "end the script with a PASS verdict", 0, 0, 0, cmd_pass },
    { "fail",       "fail [message...]",               "end the script with a FAIL verdict (non-zero exit code)", 0, -1, 0, cmd_fail },
    { "fail-on",    "fail-on \"<pattern>\" [nodes|any]", "fail the script as soon as a console line contains pattern", 1, 2, 0, cmd_fail_on },
    { "count",      "count \"<pattern>\" [nodes|any]", "count console lines containing pattern (see assert count)", 1, 2, 0, cmd_count },
    { "on",         "on <nodes|any> \"<pattern>\" <command...>", "run a command whenever a console line contains pattern", 3, -1, 0, cmd_on },
    { "set",        "set [expect-timeout <dur> | max-line <bytes> | prompt \"<glob>\"]", "settings; defaults: expect-timeout 30s, max-line 128, prompt \"#*> \"", 0, 2, IMM, cmd_set },
    { "echo",       "echo <text...>",                  "print text", 0, -1, IMM, cmd_echo },
    { "save-config","save-config <file.yaml>",         "write the live setup (positions, nodes, seed) as a config", 1, 1, IMM, cmd_save_config },
    { "help",       "help [command]",                  "this list, or one command's syntax", 0, 1, IMM, cmd_help },
};
#undef IMM
#undef BLK
static const int command_count = (int)(sizeof(commands) / sizeof(commands[0]));

const shell_command_t *shell_find_command(const char *name) {
    for (int i = 0; i < command_count; i++)
        if (strcmp(commands[i].name, name) == 0) return &commands[i];
    return NULL;
}

void shell_print_help(shell_service_t *s, const char *name) {
    if (name) {
        const shell_command_t *c = shell_find_command(name);
        if (!c) { shell_error(s, "unknown command '%s'", name); return; }
        shell_out(s, "  %s\n      %s\n", c->syntax, c->help);
        return;
    }
    shell_out(s, "Commands (nodes = id, 1,3, 2-5, all; times = 5s, 250ms, +2s; $name = variable; \"!cmd\" runs at once while a script blocks):\n");
    for (int i = 0; i < command_count; i++)
        shell_out(s, "  %-44s %s\n", commands[i].syntax, commands[i].help);
}

void shell_complete(const char *prefix, linenoiseCompletions *lc) {
    /* Complete the first word only. */
    for (const char *p = prefix; *p; p++) if (isspace((unsigned char)*p)) return;
    size_t n = strlen(prefix);
    for (int i = 0; i < command_count; i++)
        if (strncmp(commands[i].name, prefix, n) == 0)
            linenoiseAddCompletion(lc, commands[i].name);
}

bool shell_line_blocks(const char *line) {
    char *argv[SHELL_MAX_ARGS]; char storage[SHELL_LINE_MAX];
    int argc = shell_tokenize(line, argv, NULL, SHELL_MAX_ARGS, storage,
                              sizeof(storage), NULL, 0);
    if (argc <= 0) return false;
    const shell_command_t *c = shell_find_command(argv[0]);
    if (!c) return false;
    if (c->flags & SHELL_CMD_BLOCKING) return true;
    return strcmp(c->name, "run") == 0 && argc >= 2;   /* run <duration> */
}

static int exec_tokens(shell_service_t *s, const char *line, bool immediate_only) {
    char *argv[SHELL_MAX_ARGS];
    int argpos[SHELL_MAX_ARGS];
    char storage[SHELL_LINE_MAX];
    char err[128];
    char expanded[SHELL_LINE_MAX];
    if (strchr(line, '$')) {
        if (shell_expand_vars(line, expanded, sizeof(expanded), shell_var_get, s,
                              err, sizeof(err)) < 0) {
            shell_error(s, "%s", err);
            return -1;
        }
        line = expanded;
    }
    int argc = shell_tokenize(line, argv, argpos, SHELL_MAX_ARGS, storage,
                              sizeof(storage), err, sizeof(err));
    if (argc < 0) { shell_error(s, "%s", err); return -1; }
    if (argc == 0) return 0;
    const shell_command_t *c = shell_find_command(argv[0]);
    if (!c) { shell_error(s, "unknown command '%s' (try help)", argv[0]); return -1; }
    if (immediate_only && !(c->flags & SHELL_CMD_IMMEDIATE)) {
        shell_error(s, "'%s' cannot run while a script blocks (Ctrl-C aborts it)", c->name);
        return -1;
    }
    /* Scheduled commands were validated when scheduled; this is the
     * backstop for anything that slipped through. */
    if ((s->origin.kind == SHELL_ORIGIN_AT || s->origin.kind == SHELL_ORIGIN_ON) &&
        shell_line_blocks(line)) {
        shell_error(s, "'%s' cannot run from at/every/on: it would block the command stream",
                    c->name);
        return -1;
    }
    int nargs = argc - 1;
    if (nargs < c->min_args || (c->max_args >= 0 && nargs > c->max_args)) {
        shell_error(s, "usage: %s", c->syntax);
        return -1;
    }
    return c->fn(s, argc, argv, line, argpos);
}

int shell_exec_line(shell_service_t *s, const char *line, bool immediate_only,
                    const shell_origin_t *origin) {
    shell_origin_t saved = s->origin;
    bool saved_imm = s->exec_immediate;
    if (origin) s->origin = *origin;
    s->exec_immediate = immediate_only;
    int rc = exec_tokens(s, line, immediate_only);
    s->origin = saved;
    s->exec_immediate = saved_imm;
    return rc;
}
