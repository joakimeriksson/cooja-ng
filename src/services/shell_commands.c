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
#include "msp430_cpu.h"
#include "msp430_config.h"
#include "elf_loader.h"
#include "radio_medium.h"

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

/* Memory of an ARM or MSP430 node, in the debugger's view. */
typedef struct node_mem {
    arm_cpu_t     *arm;
    msp430_cpu_t  *msp;
    int            word;             /* native word: 4 (ARM) or 2 (MSP430) */
    int            id, idx;
    sim_control_node_info_t info;
    /* Byte reads of a peripheral register come from one word read of it,
     * so a dump touches a read-to-clear or FIFO register once per word. */
    bool           io_cached;
    uint32_t       io_cache_addr, io_cache_val;
} node_mem_t;

static int node_mem_open(shell_service_t *s, const char *what, const char *arg, node_mem_t *m) {
    memset(m, 0, sizeof(*m));
    long id;
    if (shell_parse_int(arg, &id) != 0) { shell_error(s, "%s: expected one node id, got '%s'", what, arg); return -1; }
    m->id = (int)id;
    m->idx = sim_control_index_of_id(s->ctl, (int)id);
    if (m->idx < 0 || !sim_control_describe(s->ctl, m->idx, &m->info)) { shell_error(s, "no node with id %ld", id); return -1; }
    if (s->ctl->ops.get_interface) {
        m->arm = s->ctl->ops.get_interface(s->ctl->ops.user, m->idx, SIM_MOTE_IFACE_ARM_CPU);
        if (!m->arm) m->msp = s->ctl->ops.get_interface(s->ctl->ops.user, m->idx, SIM_MOTE_IFACE_MSP430_CPU);
    }
    if (!m->arm && !m->msp) { shell_error(s, "%s: node %ld has no emulated CPU (ARM and MSP430 only)", what, id); return -1; }
    m->word = m->arm ? 4 : 2;
    return 0;
}

/* The debugger's view of an ARM address.  A peripheral reached through its
 * Non-secure alias would be a Non-secure bus transaction, which the
 * security unit refuses and latches (PERIPHACCERR) — guest-visible.  A
 * debugger accesses as Secure, so fold the alias onto the Secure address. */
static uint32_t node_mem_addr(const node_mem_t *m, uint32_t a) {
    if (m->arm && m->arm->io_ns_alias && (a >> 28) == 4) a |= 0x10000000u;
    return a;
}

/* ROM, flash or SRAM: bytes; anything else is a peripheral register. */
static bool arm_is_memory(const arm_cpu_t *cpu, uint32_t a) {
    return (cpu->rom && a < cpu->rom_size) ||
           (a >= cpu->flash_base && a < cpu->flash_end) ||
           (a >= cpu->sram_base && a < cpu->sram_end);
}

static bool node_mem_read8(node_mem_t *m, uint32_t a, uint8_t *v) {
    if (m->arm) {
        a = node_mem_addr(m, a);
        if (arm_is_memory(m->arm, a)) { *v = arm_read8(m->arm, a); return true; }
        uint32_t w = a & ~3u;
        if (!m->io_cached || m->io_cache_addr != w) {
            m->io_cache_val = arm_read32(m->arm, w);
            m->io_cache_addr = w;
            m->io_cached = true;
        }
        *v = (uint8_t)(m->io_cache_val >> (8 * (a & 3)));
        return true;
    }
    if (a >= m->msp->max_mem) return false;
    *v = m->msp->memory[a];
    return true;
}

/* A native word.  A peripheral register is read once, as a word: byte-wise
 * access would replicate the low byte and hit a read-to-clear or FIFO
 * register once per byte. */
static bool node_mem_read_word(node_mem_t *m, uint32_t a, uint32_t *v) {
    *v = 0;
    if (m->arm) {
        a = node_mem_addr(m, a);
        if (!arm_is_memory(m->arm, a)) { *v = arm_read32(m->arm, a); return true; }
    }
    for (int b = 0; b < m->word; b++) {
        uint8_t x;
        if (!node_mem_read8(m, a + (uint32_t)b, &x)) return false;
        *v |= (uint32_t)x << (8 * b);
    }
    return true;
}

static void node_mem_write(const node_mem_t *m, uint32_t a, uint32_t v, int step) {
    if (m->arm) {
        a = node_mem_addr(m, a);
        if (step == 4 && !arm_is_memory(m->arm, a)) { arm_write32(m->arm, a, v); return; }
        for (int b = 0; b < step; b++) arm_write8(m->arm, a + (uint32_t)b, (uint8_t)(v >> (8 * b)));
        return;
    }
    for (int b = 0; b < step; b++) m->msp->memory[a + (uint32_t)b] = (uint8_t)(v >> (8 * b));
}

/* NULL if [first, last] may be written, else why not. */
static const char *node_mem_write_refusal(const node_mem_t *m, uint32_t first, uint32_t last) {
    if (m->arm) {
        if (first < m->arm->flash_end && last >= m->arm->flash_base)
            return "is flash, which is read-only here (as on hardware)";
        return NULL;
    }
    const msp430_config_t *c = m->msp->config;
    if (!c || first < c->ram_start || last >= c->ram_start + c->ram_size)
        return "is outside RAM (MSP430 writes are limited to RAM)";
    return NULL;
}

/* mem [-w] [-c var] <node> <addr|sym> [count]   read
 * mem [-w] <node> <addr|sym> = <value...>       write
 * -w: native words (32-bit ARM, 16-bit MSP430), little-endian. */
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
    node_mem_t m;
    if (node_mem_open(s, "mem", argv[i], &m) != 0) return -1;
    uint32_t addr;
    if (resolve_addr(s, &m.info, argv[i + 1], &addr) != 0) return -1;
    int step = words ? m.word : 1;

    if (argc - i >= 3 && !strcmp(argv[i + 2], "=")) {
        if (argc - i < 4) { shell_error(s, "mem: nothing to write"); return -1; }
        uint64_t end = (uint64_t)addr + (uint64_t)(argc - i - 3) * (uint64_t)step;   /* 64-bit: no wrap */
        if (end > 0x100000000ull) {
            shell_error(s, "mem: 0x%08x + %d bytes runs past the end of the address space", addr, (argc - i - 3) * step);
            return -1;
        }
        uint32_t last = (uint32_t)(end - 1);
        const char *why = node_mem_write_refusal(&m, addr, last);
        if (why) { shell_error(s, "mem: 0x%08x-0x%08x %s", addr, last, why); return -1; }
        for (int k = i + 3; k < argc; k++) {
            long v;
            if (shell_parse_int(argv[k], &v) != 0) { shell_error(s, "mem: bad value '%s'", argv[k]); return -1; }
            node_mem_write(&m, addr + (uint32_t)((k - i - 3) * step), (uint32_t)v, step);
        }
        if (s->verbose) shell_out(s, "wrote %d %s at 0x%08x\n", argc - i - 3, words ? "word(s)" : "byte(s)", addr);
        return 0;
    }

    long count = words ? 8 : 64;
    if (argc - i >= 3 && shell_parse_int(argv[i + 2], &count) != 0) { shell_error(s, "mem: bad count '%s'", argv[i + 2]); return -1; }
    if (count < 1 || count > 4096) { shell_error(s, "mem: count must be 1..4096"); return -1; }
    if (var && count != 1) { shell_error(s, "mem: -c needs a count of 1"); return -1; }
    uint64_t span_end = (uint64_t)addr + (uint64_t)count * (uint64_t)step;
    if (span_end > (m.msp ? (uint64_t)m.msp->max_mem : 0x100000000ull)) {
        shell_error(s, "mem: 0x%08x+%ld is beyond the 0x%llx bytes of address space", addr, count * step,
                    m.msp ? (unsigned long long)m.msp->max_mem : 0x100000000ull);
        return -1;
    }
    char buf[24];
    if (words) {
        for (long k = 0; k < count; k++) {
            uint32_t at = addr + (uint32_t)(k * m.word), v;
            node_mem_read_word(&m, at, &v);
            snprintf(buf, sizeof(buf), m.word == 4 ? "0x%08x" : "0x%04x", v);
            if (var) shell_var_set(s, var, buf);
            if (k % (m.word == 4 ? 4 : 8) == 0) shell_out(s, "%s0x%08x:", k ? "\n" : "", at);
            shell_out(s, " %s", buf);
        }
        shell_out(s, "\n");
        return 0;
    }
    for (long k = 0; k < count; k += 16) {
        char hex[64] = "", asc[20] = "";
        int n = (int)((count - k) < 16 ? (count - k) : 16);
        for (int b = 0; b < n; b++) {
            uint8_t v = 0;
            node_mem_read8(&m, addr + (uint32_t)(k + b), &v);
            if (var) { snprintf(buf, sizeof(buf), "0x%02x", v); shell_var_set(s, var, buf); }
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
    bool writing = !var && argc - i == 4 && !strcmp(argv[i + 2], "=");
    if (!writing && (argc - i < 1 || argc - i > 2)) {
        shell_error(s, "usage: reg [-c <var>] <node> [name] | reg <node> <name> = <value>"); return -1;
    }
    node_mem_t m;
    if (node_mem_open(s, "reg", argv[i], &m) != 0) return -1;

    /* The register table for this CPU: name, pointer, width. */
    struct { const char *name; uint32_t *p; bool tz; } r[40];
    int nr = 0;
#define REG(n, ptr, tzonly) do { r[nr].name = (n); r[nr].p = (ptr); r[nr].tz = (tzonly); nr++; } while (0)
    static const char *const armn[13] = { "r0","r1","r2","r3","r4","r5","r6","r7","r8","r9","r10","r11","r12" };
    static const char *const mspn[16] = { "pc","sp","sr","r3","r4","r5","r6","r7","r8","r9","r10","r11","r12","r13","r14","r15" };
    if (m.arm) {
        arm_cpu_t *c = m.arm;
        for (int k = 0; k < 13; k++) REG(armn[k], &c->reg[k], false);
        REG("sp", &c->reg[ARM_SP], false); REG("lr", &c->reg[ARM_LR], false); REG("pc", &c->reg[ARM_PC], false);
        REG("xpsr", &c->xpsr, false); REG("primask", &c->primask, false);
        REG("basepri", &c->basepri, false); REG("faultmask", &c->faultmask, false);
        REG("msp_s", &c->msp_s, true); REG("psp_s", &c->psp_s, true);
        REG("msp_ns", &c->msp_ns, true); REG("psp_ns", &c->psp_ns, true);
        REG("control_s", &c->control_s, true); REG("control_ns", &c->control_ns, true);
    } else {
        for (int k = 0; k < 16; k++) REG(mspn[k], &m.msp->reg[k], false);
    }
#undef REG
    bool tz = m.arm && arm_cpu_has_trustzone(m.arm);

    if (writing) {
        long v;
        if (shell_parse_int(argv[i + 3], &v) != 0) { shell_error(s, "reg: bad value '%s'", argv[i + 3]); return -1; }
        const char *n = argv[i + 1];
        uint32_t *target = NULL;
        for (int k = 0; k < nr; k++) if (!strcmp(r[k].name, n) && !r[k].tz) target = r[k].p;
        if (m.arm && target && (!strcmp(n, "primask") || !strcmp(n, "basepri") || !strcmp(n, "faultmask")))
            target = NULL;     /* banked by security state: not a plain write */
        if (!target) {
            shell_error(s, "reg: can write %s (not '%s')",
                        m.arm ? "r0-r12, sp, lr, pc, xpsr" : "pc, sp, sr, r3-r15", n);
            return -1;
        }
        uint32_t val = (uint32_t)v;
        if (m.arm && target == &m.arm->reg[ARM_PC]) val &= ~1u;      /* Thumb bit is not PC */
        if (m.msp) val &= m.msp->is_msp430x ? 0xfffffu : 0xffffu;
        *target = val;
        if (s->verbose) shell_out(s, "%s = 0x%08x\n", n, val);
        return 0;
    }
    if (argc - i == 2) {
        for (int k = 0; k < nr; k++) {
            if (strcmp(r[k].name, argv[i + 1]) != 0 || (r[k].tz && !tz)) continue;
            char label[32];
            snprintf(label, sizeof(label), "%s = ", r[k].name);
            emit_value(s, var, label, *r[k].p);
            return 0;
        }
        shell_error(s, "reg: no register '%s' on this CPU", argv[i + 1]);
        return -1;
    }
    if (var) { shell_error(s, "reg: -c needs a register name"); return -1; }
    int col = 0;
    for (int k = 0; k < nr; k++) {
        if (r[k].tz && !tz) continue;
        shell_out(s, "  %-10s 0x%08x%s", r[k].name, *r[k].p, (++col % 4) ? "" : "\n");
    }
    if (col % 4) shell_out(s, "\n");
    if (tz) shell_out(s, "  state: %s\n", arm_cpu_is_secure(m.arm) ? "Secure" : "Non-secure");
    return 0;
}

/* --- breakpoints and watchpoints (ARM) -------------------------------------- */

static arm_cpu_t *dbg_cpu(shell_service_t *s, int node_id) {
    int idx = sim_control_index_of_id(s->ctl, node_id);
    if (idx < 0 || !s->ctl->ops.get_interface) return NULL;
    return s->ctl->ops.get_interface(s->ctl->ops.user, idx, SIM_MOTE_IFACE_ARM_CPU);
}

/* Write a node's breakpoint/watchpoint tables from the shell's list, in list
 * order (so cpu->dbg_hit_index maps back to it).  Watch shadows restart from
 * the current memory. */
static void dbg_arm_node(shell_service_t *s, int node_id) {
    arm_cpu_t *cpu = dbg_cpu(s, node_id);
    if (!cpu) return;
    cpu->dbg_bp_n = cpu->dbg_wp_n = 0;
    for (int i = 0; i < s->dbg_count; i++) {
        const shell_dbg_t *d = &s->dbg[i];
        if (d->node_id != node_id) continue;
        if (d->kind == 1 && cpu->dbg_bp_n < ARM_DBG_MAX_BP) {
            cpu->dbg_bp[cpu->dbg_bp_n++] = d->addr;
        } else if (d->kind == 2 && cpu->dbg_wp_n < ARM_DBG_MAX_WP) {
            int w = cpu->dbg_wp_n++;
            cpu->dbg_wp[w].addr = d->addr;
            cpu->dbg_wp[w].len = d->len;
            uint32_t v = 0;
            for (int b = 0; b < d->len; b++) v |= (uint32_t)arm_read8(cpu, d->addr + (uint32_t)b) << (8 * b);
            cpu->dbg_wp[w].shadow = v;
        }
    }
    cpu->dbg_count = cpu->dbg_bp_n + cpu->dbg_wp_n;
    if (cpu->dbg_count == 0) cpu->dbg_halted = cpu->dbg_hit_new = false;
}

/* The shell entry behind a CPU's hit (kind, index into that kind's table). */
static shell_dbg_t *dbg_entry_for_hit(shell_service_t *s, int node_id, int kind, int index) {
    for (int i = 0; i < s->dbg_count; i++) {
        shell_dbg_t *d = &s->dbg[i];
        if (d->node_id != node_id || d->kind != kind) continue;
        if (index-- == 0) return d;
    }
    return NULL;
}

void shell_debug_tick(shell_service_t *s) {
    if (s->dbg_count == 0) return;
    int done[SHELL_DBG_MAX], ndone = 0;
    for (int i = 0; i < s->dbg_count; i++) {
        int node_id = s->dbg[i].node_id;
        bool seen = false;
        for (int k = 0; k < ndone; k++) seen |= done[k] == node_id;
        if (seen) continue;
        done[ndone++] = node_id;
        arm_cpu_t *cpu = dbg_cpu(s, node_id);
        if (!cpu) continue;
        int want = 0;
        for (int k = 0; k < s->dbg_count; k++) want += s->dbg[k].node_id == node_id;
        if (cpu->dbg_count != want) dbg_arm_node(s, node_id);       /* rebooted */
        if (!cpu->dbg_hit_new) continue;
        cpu->dbg_hit_new = false;
        shell_dbg_t *d = dbg_entry_for_hit(s, node_id, cpu->dbg_hit_kind, cpu->dbg_hit_index);
        if (d) { d->hits++; d->halted = true; }
        shell_hold_output(s);
        if (cpu->dbg_hit_kind == 1)
            shell_out(s, "breakpoint #%d: node %d at pc 0x%08x (%.6f s)\n", d ? d->id : 0, node_id,
                      cpu->dbg_hit_pc, (double)now_ns(s) / 1e9);
        else
            shell_out(s, "watchpoint #%d: node %d 0x%08x changed 0x%x -> 0x%x, written at pc 0x%08x (%.6f s)\n",
                      d ? d->id : 0, node_id, d ? d->addr : 0, cpu->dbg_hit_old, cpu->dbg_hit_value,
                      cpu->dbg_hit_pc, (double)now_ns(s) / 1e9);
        if (!s->sim->clock_source) sim_control_pause(s->ctl);
        if (s->block == SHELL_BLOCK_HALT && s->halt_node_id == node_id) {
            s->block = SHELL_BLOCK_NONE;
            s->expect_pass++;
        }
    }
}

/* break <node> <addr|sym[+off]>, watch <node> <addr|sym[+off]> [bytes] */
static int cmd_break(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos, int kind) {
    (void)line; (void)argpos;
    const char *what = kind == 1 ? "break" : "watch";
    if (argc < 3) { shell_error(s, "usage: %s", shell_find_command(what)->syntax); return -1; }
    sim_control_node_info_t info;
    arm_cpu_t *cpu = node_arm_cpu(s, what, argv[1], NULL, &info);
    if (!cpu) return -1;
    uint32_t addr;
    if (resolve_addr(s, &info, argv[2], &addr) != 0) return -1;
    long len = 4;
    if (kind == 1) {
        addr &= ~1u;                          /* Thumb bit of a function symbol */
        if (argc > 3) { shell_error(s, "usage: break <node> <addr|symbol>"); return -1; }
        if (!arm_is_memory(cpu, addr)) {
            shell_error(s, "break: 0x%08x is not code memory (ROM, flash or SRAM)", addr); return -1;
        }
    } else {
        if (argc == 4 && (shell_parse_int(argv[3], &len) != 0 || len < 1 || len > 4)) {
            shell_error(s, "watch: length is 1..4 bytes"); return -1;
        }
        if (addr < cpu->sram_base || (uint64_t)addr + (uint64_t)len > cpu->sram_end) {
            shell_error(s, "watch: 0x%08x is not SRAM (watching peripherals would read their registers every instruction)", addr);
            return -1;
        }
    }
    int per_kind = 0;
    for (int i = 0; i < s->dbg_count; i++)
        per_kind += s->dbg[i].node_id == info.id && s->dbg[i].kind == kind;
    if (s->dbg_count >= SHELL_DBG_MAX) {
        shell_error(s, "%s: at most %d breakpoints and watchpoints in total", what, SHELL_DBG_MAX); return -1;
    }
    if (per_kind >= (kind == 1 ? ARM_DBG_MAX_BP : ARM_DBG_MAX_WP)) {
        shell_error(s, "%s: at most %d per node", what, kind == 1 ? ARM_DBG_MAX_BP : ARM_DBG_MAX_WP); return -1;
    }
    shell_dbg_t *d = &s->dbg[s->dbg_count++];
    memset(d, 0, sizeof(*d));
    d->id = s->next_dbg_id++;
    d->node_id = info.id;
    d->kind = kind;
    d->addr = addr;
    d->len = (int)len;
    dbg_arm_node(s, info.id);
    if (s->verbose)
        shell_out(s, "%s #%d on node %d at 0x%08x%s\n", kind == 1 ? "breakpoint" : "watchpoint", d->id,
                  info.id, addr, kind == 1 ? " (the node runs interpreted while it is armed)" : "");
    return 0;
}

static int cmd_break_list(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos);
static int cmd_break_clear(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos);

/* `break list` / `break clear` cover watchpoints too: they share one list. */
static int cmd_breakpoint(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    if (!strcmp(argv[1], "list")) {
        if (argc != 2) { shell_error(s, "usage: break list"); return -1; }
        return cmd_break_list(s, 1, argv + 1, line, argpos);
    }
    if (!strcmp(argv[1], "clear")) {
        if (argc != 3) { shell_error(s, "usage: break clear <n>|all"); return -1; }
        return cmd_break_clear(s, 2, argv + 1, line, argpos);
    }
    return cmd_break(s, argc, argv, line, argpos, 1);
}

static int cmd_watch(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    return cmd_break(s, argc, argv, line, argpos, 2);
}

static int cmd_break_list(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (s->dbg_count == 0) { shell_out(s, "no breakpoints or watchpoints\n"); return 0; }
    for (int i = 0; i < s->dbg_count; i++) {
        const shell_dbg_t *d = &s->dbg[i];
        arm_cpu_t *cpu = dbg_cpu(s, d->node_id);
        bool halted_here = d->halted && cpu && cpu->dbg_halted;
        if (d->kind == 1)
            shell_out(s, "  #%d breakpoint node %d 0x%08x  hits %d%s\n", d->id, d->node_id, d->addr, d->hits,
                      halted_here ? "  (halted here)" : "");
        else
            shell_out(s, "  #%d watchpoint node %d 0x%08x/%d  hits %d%s\n", d->id, d->node_id, d->addr, d->len,
                      d->hits, halted_here ? "  (halted here)" : "");
    }
    return 0;
}

static int cmd_break_clear(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    bool all = !strcmp(argv[1], "all");
    long id = 0;
    if (!all && shell_parse_int(argv[1], &id) != 0) { shell_error(s, "break clear: expected a number or all"); return -1; }
    int nodes[SHELL_DBG_MAX], nn = 0, removed = 0;
    for (int i = 0; i < s->dbg_count; i++) {
        if (!all && s->dbg[i].id != id) continue;
        nodes[nn++] = s->dbg[i].node_id;
        memmove(&s->dbg[i], &s->dbg[i + 1], (size_t)(s->dbg_count - i - 1) * sizeof(s->dbg[0]));
        s->dbg_count--;
        i--;
        removed++;
    }
    if (!removed) { shell_error(s, "break clear: no breakpoint or watchpoint #%ld", id); return -1; }
    for (int k = 0; k < nn; k++) dbg_arm_node(s, nodes[k]);
    return 0;
}

/* continue [node]: release halted nodes (a breakpoint is not hit again on
 * the way out) and resume the simulation. */
static int cmd_continue(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int only = -1;
    if (argc == 2) {
        long id;
        if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "continue: expected a node id"); return -1; }
        only = (int)id;
    }
    int released = 0;
    int n = sim_control_node_count(s->ctl);
    for (int i = 0; i < n; i++) {
        sim_control_node_info_t info;
        if (!sim_control_describe(s->ctl, i, &info) || (only >= 0 && info.id != only)) continue;
        arm_cpu_t *cpu = dbg_cpu(s, info.id);
        if (!cpu || !cpu->dbg_halted) continue;
        if (cpu->dbg_hit_kind == 1) cpu->dbg_skip_pc = cpu->reg[ARM_PC] & ~1u;
        cpu->dbg_halted = false;
        for (int k = 0; k < s->dbg_count; k++)
            if (s->dbg[k].node_id == info.id) s->dbg[k].halted = false;
        sim_schedule_mote_wakeup_if_earlier(s->sim, i, now_ns(s));
        released++;
    }
    if (only >= 0 && !released) { shell_error(s, "continue: node %d is not halted", only); return -1; }
    if (!shell_refuse_external_clock(s, "continue")) sim_control_resume(s->ctl);
    return 0;
}

/* expect-halt <node> [timeout] */
static int cmd_expect_halt(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    long id;
    if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "expect-halt: expected a node id"); return -1; }
    arm_cpu_t *cpu = dbg_cpu(s, (int)id);
    if (!cpu) { shell_error(s, "expect-halt: node %ld has no ARM CPU", id); return -1; }
    int64_t timeout = s->default_expect_timeout_ns;
    if (argc == 3 && parse_dur(s, argv[2], &timeout) != 0) return -1;
    s->script_used = true;
    if (cpu->dbg_halted && !cpu->dbg_hit_new) { s->expect_pass++; return 0; }   /* already stopped */
    s->halt_node_id = (int)id;
    shell_script_block_until(s, SHELL_BLOCK_HALT, now_ns(s) + timeout);
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

/* A condition shared by `assert` and `if`: argv[1..] is "time <op> <t>",
 * "mem ...", etc.  Returns -1 after printing a usage/lookup error; else 0
 * with *result set and, when false, cond_desc describing why. */
static char cond_desc[SHELL_REASON_MAX];
static int eval_condition(shell_service_t *s, int argc, char **argv, bool *result) {
    *result = true;
    cond_desc[0] = '\0';
    const char *what = argv[1];
    if (strcmp(what, "mem") == 0 && argc == 6) {
        node_mem_t m;
        if (node_mem_open(s, "assert mem", argv[2], &m) != 0) return -1;
        uint32_t addr;
        if (resolve_addr(s, &m.info, argv[3], &addr) != 0) return -1;
        long want;
        if (shell_parse_int(argv[5], &want) != 0) { shell_error(s, "assert mem: bad value '%s'", argv[5]); return -1; }
        uint32_t v;
        if (!node_mem_read_word(&m, addr, &v)) { shell_error(s, "assert mem: 0x%08x is outside the address space", addr); return -1; }
        int r;
        if (!strcmp(argv[4], "==")) r = v == (uint32_t)want;
        else if (!strcmp(argv[4], "!=")) r = v != (uint32_t)want;
        else r = shell_compare((long)v, argv[4], (long)(uint32_t)want);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[4]); return -1; }
        if (!r) { snprintf(cond_desc, sizeof(cond_desc), "mem %s 0x%08x (= 0x%08x) %s %s", argv[2], addr, v, argv[4], argv[5]); *result = false; return 0; }
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
        if (!r) { snprintf(cond_desc, sizeof(cond_desc), "%s (\"%s\") %s %s", argv[2], v, argv[3], argv[4]); *result = false; return 0; }
        return 0;
    }
    if (strcmp(what, "time") == 0 && argc == 4) {
        int64_t t;
        if (parse_dur(s, argv[3], &t) != 0) return -1;
        int r = shell_compare((long)(now_ns(s) / 1000), argv[2], (long)(t / 1000));
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { snprintf(cond_desc, sizeof(cond_desc), "time (%.6f s) %s %s", (double)now_ns(s) / 1e9, argv[2], argv[3]); *result = false; return 0; }
        return 0;
    }
    if (strcmp(what, "nodes") == 0 && argc == 4) {
        long v;
        if (shell_parse_int(argv[3], &v) != 0) { shell_error(s, "assert: bad count '%s'", argv[3]); return -1; }
        int n = sim_control_node_count(s->ctl), active = 0;
        for (int i = 0; i < n; i++) if (sim_control_node_active(s->ctl, i)) active++;
        int r = shell_compare(active, argv[2], v);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { snprintf(cond_desc, sizeof(cond_desc), "nodes (%d active) %s %ld", active, argv[2], v); *result = false; return 0; }
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
        if (!ok) { snprintf(cond_desc, sizeof(cond_desc), "node %ld %s", id, argv[3]); *result = false; return 0; }
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
        if (!r) { snprintf(cond_desc, sizeof(cond_desc), "count \"%s\" (%d) %s %ld", argv[2], w->count, argv[3], v); *result = false; return 0; }
        return 0;
    }
    shell_error(s, "usage: %s time <op> <t> | nodes <op> N | node <id> active|removed|exists|secure|non-secure | count \"pat\" <op> N | mem <node> <addr> <op> <word> | var <name> <op> <value>", argv[0]);
    return -1;
}

int shell_eval_condition(shell_service_t *s, int argc, char **argv,
                         bool *result, const char **why) {
    int rc = eval_condition(s, argc, argv, result);
    if (why) *why = cond_desc;
    return rc;
}

/* --- script control flow: repeat / if / else / end ---------------------- */

static shell_source_t *script_source(shell_service_t *s, const char *what) {
    if (s->origin.kind != SHELL_ORIGIN_FILE || s->depth == 0 ||
        s->stack[s->depth - 1].send_idx >= 0) {
        shell_error(s, "%s is only available in script files", what);
        return NULL;
    }
    return &s->stack[s->depth - 1];
}

/* Skip lines of `src` to the `else` (when stop_at_else) or `end` matching
 * the block just entered.  Returns 1 at `else`, 0 at `end`, -1 at EOF. */
static int skip_block(shell_service_t *s, shell_source_t *src, bool stop_at_else) {
    char buf[SHELL_LINE_MAX], storage[SHELL_LINE_MAX];
    int depth = 0;
    while (fgets(buf, sizeof(buf), src->f)) {
        src->lineno++;
        char *argv[4];
        int argc = shell_tokenize(buf, argv, NULL, 4, storage, sizeof(storage), NULL, 0);
        if (argc < 0) {                      /* >3 words, or a bad line: first word only */
            char *p = buf;
            while (*p == ' ' || *p == '\t') p++;
            size_t n = strcspn(p, " \t\r\n");
            if (n == 0 || n >= sizeof(storage)) continue;
            memcpy(storage, p, n); storage[n] = '\0';
            argv[0] = storage; argc = 1;
        }
        if (argc == 0) continue;
        if (!strcmp(argv[0], "repeat") || !strcmp(argv[0], "if")) depth++;
        else if (!strcmp(argv[0], "end")) { if (depth-- == 0) return 0; }
        else if (!strcmp(argv[0], "else") && depth == 0 && stop_at_else) return 1;
    }
    shell_error(s, "end of file inside a block (missing `end`)");
    return -1;
}

static int push_frame(shell_service_t *s, shell_source_t *src, int kind) {
    if (src->nframes >= SHELL_FRAMES_MAX) {
        shell_error(s, "blocks nested too deep (max %d)", SHELL_FRAMES_MAX);
        return -1;
    }
    memset(&src->frames[src->nframes], 0, sizeof(src->frames[0]));
    src->frames[src->nframes].kind = kind;
    return src->nframes++;
}

/* repeat <count> [var] ... end */
static int cmd_repeat(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    shell_source_t *src = script_source(s, "repeat");
    if (!src) return -1;
    long n;
    if (shell_parse_int(argv[1], &n) != 0 || n < 0) { shell_error(s, "repeat: expected a count >= 0, got '%s'", argv[1]); return -1; }
    if (argc == 3 && !shell_var_name_ok(argv[2])) { shell_error(s, "repeat: bad variable name '%s'", argv[2]); return -1; }
    if (n == 0) return skip_block(s, src, false) < 0 ? -1 : 0;
    int f = push_frame(s, src, 1);
    if (f < 0) return -1;
    src->frames[f].pos = ftell(src->f);
    src->frames[f].lineno = src->lineno;
    src->frames[f].count = 1;
    src->frames[f].total = n;
    if (argc == 3) {
        snprintf(src->frames[f].var, sizeof(src->frames[f].var), "%s", argv[2]);
        if (shell_var_set(s, argv[2], "1") != 0) { shell_error(s, "too many variables"); return -1; }
    }
    return 0;
}

/* if <condition> ... [else ...] end — conditions as for assert. */
static int cmd_if(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    shell_source_t *src = script_source(s, "if");
    if (!src) return -1;
    bool ok;
    if (eval_condition(s, argc, argv, &ok) != 0) return -1;
    if (ok) return push_frame(s, src, 2) < 0 ? -1 : 0;
    int r = skip_block(s, src, true);
    if (r < 0) return -1;
    if (r == 1) return push_frame(s, src, 2) < 0 ? -1 : 0;   /* in the else branch */
    return 0;
}

static int cmd_else(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    shell_source_t *src = script_source(s, "else");
    if (!src) return -1;
    if (src->nframes == 0 || src->frames[src->nframes - 1].kind != 2) {
        shell_error(s, "else without if"); return -1;
    }
    /* Reached at the end of the taken branch: skip the other one. */
    src->nframes--;
    return skip_block(s, src, false) < 0 ? -1 : 0;
}

static int cmd_end(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    shell_source_t *src = script_source(s, "end");
    if (!src) return -1;
    if (src->nframes == 0) { shell_error(s, "end without repeat or if"); return -1; }
    int f = src->nframes - 1;
    if (src->frames[f].kind == 1 && src->frames[f].count < src->frames[f].total) {
        src->frames[f].count++;
        if (src->frames[f].var[0]) {
            char num[24];
            snprintf(num, sizeof(num), "%ld", src->frames[f].count);
            shell_var_set(s, src->frames[f].var, num);
        }
        fseek(src->f, src->frames[f].pos, SEEK_SET);
        src->lineno = src->frames[f].lineno;
        return 0;
    }
    src->nframes--;
    return 0;
}

/* --- environment: radio medium, capture, clocks, pins, LEDs, run ----------- */

static int one_node(shell_service_t *s, const char *what, const char *arg, int *idx) {
    long id;
    if (shell_parse_int(arg, &id) != 0) { shell_error(s, "%s: expected one node id, got '%s'", what, arg); return -1; }
    *idx = sim_control_index_of_id(s->ctl, (int)id);
    if (*idx < 0) { shell_error(s, "no node with id %ld", id); return -1; }
    return (int)id;
}

static int slot_id(shell_service_t *s, int idx) {
    sim_control_node_info_t info;
    return sim_control_describe(s->ctl, idx, &info) ? info.id : -1;
}

/* radio | radio range <tx> [interference] | radio success <tx> [rx] */
static int cmd_radio(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    radio_medium_t *rm = &s->sim->radio_medium;
    if (argc == 1) {
        shell_out(s, "  medium: %s%s\n", rm->ops && rm->ops->name ? rm->ops->name : "?",
                  rm->type == RADIO_MEDIUM_NONE ? " (every frame reaches every node)" : "");
        if (rm->type != RADIO_MEDIUM_NONE)
            shell_out(s, "  tx range %.1f m  interference range %.1f m  success tx %.2f rx %.2f\n",
                      rm->udgm.tx_range, rm->udgm.interference_range,
                      rm->udgm.success_ratio_tx, rm->udgm.success_ratio_rx);
        if (rm->any_link_blocked) shell_out(s, "  some links are cut (see links)\n");
        return 0;
    }
    if (rm->type == RADIO_MEDIUM_NONE) {
        shell_error(s, "radio: the medium is 'none' (all-to-all); range and success do not apply");
        return -1;
    }
    double a = 0, b = 0;
    if (argc < 3 || argc > 4 || shell_parse_double(argv[2], &a) != 0 ||
        (argc == 4 && shell_parse_double(argv[3], &b) != 0)) {
        shell_error(s, "usage: radio [range <tx> [interference] | success <tx> [rx]]");
        return -1;
    }
    if (!strcmp(argv[1], "range")) {
        if (a < 0 || (argc == 4 && b < 0)) { shell_error(s, "radio range: metres must be >= 0"); return -1; }
        rm->udgm.tx_range = a;
        if (argc == 4) rm->udgm.interference_range = b;
        radio_medium_compute_neighbors(rm);
    } else if (!strcmp(argv[1], "success")) {
        if (a < 0 || a > 1 || (argc == 4 && (b < 0 || b > 1))) { shell_error(s, "radio success: ratios are 0..1"); return -1; }
        rm->udgm.success_ratio_tx = a;
        if (argc == 4) rm->udgm.success_ratio_rx = b;
    } else {
        shell_error(s, "usage: radio [range <tx> [interference] | success <tx> [rx]]");
        return -1;
    }
    return 0;
}

/* link <a> <b> off|on    (both directions)
 * link <a> -> <b> off|on (one direction) */
static int cmd_link(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    bool oneway = argc == 5 && !strcmp(argv[2], "->");
    if (!(argc == 4 || oneway)) { shell_error(s, "usage: link <a> <b> off|on | link <a> -> <b> off|on"); return -1; }
    const char *state = argv[argc - 1];
    bool off = !strcmp(state, "off");
    if (!off && strcmp(state, "on")) { shell_error(s, "link: expected off or on, got '%s'", state); return -1; }
    int ia, ib;
    if (one_node(s, "link", argv[1], &ia) < 0 || one_node(s, "link", argv[oneway ? 3 : 2], &ib) < 0) return -1;
    if (ia == ib) { shell_error(s, "link: a node has no link to itself"); return -1; }
    radio_medium_set_link_blocked(&s->sim->radio_medium, ia, ib, off);
    if (!oneway) radio_medium_set_link_blocked(&s->sim->radio_medium, ib, ia, off);
    return 0;
}

static int cmd_links(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    const radio_medium_t *rm = &s->sim->radio_medium;
    int n = sim_control_node_count(s->ctl), cut = 0;
    for (int a = 0; a < n; a++)
        for (int b = 0; b < n; b++)
            if (radio_medium_link_blocked(rm, a, b)) {
                bool both = radio_medium_link_blocked(rm, b, a);
                if (both && b < a) continue;
                shell_out(s, "  %d %s %d cut\n", slot_id(s, a), both ? "<->" : "->", slot_id(s, b));
                cut++;
            }
    if (!cut) shell_out(s, "no links are cut\n");
    return 0;
}

static int cmd_neighbors(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    const radio_medium_t *rm = &s->sim->radio_medium;
    int n = sim_control_node_count(s->ctl);
    int only = -1;
    if (argc == 2 && one_node(s, "neighbors", argv[1], &only) < 0) return -1;
    if (rm->type == RADIO_MEDIUM_NONE) { shell_out(s, "medium 'none': every node hears every node\n"); return 0; }
    for (int i = 0; i < n && i < RADIO_MEDIUM_MAX_NODES; i++) {
        if (only >= 0 && i != only) continue;
        char buf[512] = "";
        const neighbor_list_t *nl = &rm->neighbors[i];
        for (int k = 0; k < nl->count; k++) {
            int j = nl->neighbors[k];
            snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "%s%d%s", k ? " " : "",
                     slot_id(s, j), radio_medium_link_blocked(rm, i, j) ? "(cut)" : "");
        }
        shell_out(s, "  node %d hears: %s\n", slot_id(s, i), buf[0] ? buf : "(nobody)");
    }
    return 0;
}

static int cmd_pcap(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (!s->ctl->ops.pcap) { shell_error(s, "pcap is not available in this mode"); return -1; }
    const char *path = strcmp(argv[1], "off") ? argv[1] : NULL;
    if (s->ctl->ops.pcap(s->ctl->ops.user, path) != 0) { shell_error(s, "pcap: cannot write %s", argv[1]); return -1; }
    return 0;
}

static int cmd_clock(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int idx;
    int id = one_node(s, "clock", argv[1], &idx);
    if (id < 0) return -1;
    if (argc == 2) {
        sim_control_node_info_t info;
        sim_control_describe(s->ctl, idx, &info);
        shell_out(s, "node %d clock deviation %.10f\n", id, info.clock_deviation);
        return 0;
    }
    double d;
    if (shell_parse_double(argv[2], &d) != 0 || d <= 0.5 || d >= 1.5) {
        shell_error(s, "clock: deviation must be a ratio between 0.5 and 1.5 (1.0 = exact)"); return -1;
    }
    if (!s->ctl->ops.set_clock_deviation) { shell_error(s, "clock is not available in this mode"); return -1; }
    s->ctl->ops.set_clock_deviation(s->ctl->ops.user, idx, d);
    return 0;
}

static int cmd_leds(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, argc == 2 ? argv[1] : "all", ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    for (int k = 0; k < n; k++) {
        uint8_t l[3];
        int idx = sim_control_index_of_id(s->ctl, ids[k]);
        if (!s->ctl->ops.leds || !s->ctl->ops.leds(s->ctl->ops.user, idx, l))
            shell_out(s, "  node %d: no LEDs modelled\n", ids[k]);
        else
            shell_out(s, "  node %d: LED1 %s  LED2 %s  LED3 %s\n", ids[k],
                      l[0] ? "on" : "off", l[1] ? "on" : "off", l[2] ? "on" : "off");
    }
    return 0;
}

/* "1.13", "P1.13", "A.3" (CC2538 ports A-D). */
static int parse_pin(const char *spec, int *port, int *pin) {
    const char *p = spec;
    if (*p == 'P' || *p == 'p') p++;
    if (isalpha((unsigned char)*p) && p[1] == '.') { *port = toupper((unsigned char)*p) - 'A'; p += 2; }
    else {
        char *end;
        long v = strtol(p, &end, 10);
        if (end == p || *end != '.') return -1;
        *port = (int)v;
        p = end + 1;
    }
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p || *end) return -1;
    *pin = (int)v;
    return 0;
}

static int drive_pin(shell_service_t *s, const char *what, int idx, int port, int pin, int level) {
    if (!s->ctl->ops.set_input_pin ||
        s->ctl->ops.set_input_pin(s->ctl->ops.user, idx, port, pin, level) != 0) {
        shell_error(s, "%s: node %d has no modelled GPIO input %d.%d", what, slot_id(s, idx), port, pin);
        return -1;
    }
    sim_schedule_mote_wakeup_if_earlier(s->sim, idx, now_ns(s));
    return 0;
}

/* Schedule the release half of a pulse/click through the at-queue. */
static int schedule_release(shell_service_t *s, int64_t dur, const char *cmd) {
    if (shell_script_at_add(s, now_ns(s) + dur, 0, cmd) < 0) {
        shell_error(s, "at queue full (max %d)", SHELL_ATQ_MAX); return -1;
    }
    return 0;
}

/* gpio <node> <port.pin> high|low|pulse [duration] */
static int cmd_gpio(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int idx;
    int id = one_node(s, "gpio", argv[1], &idx);
    if (id < 0) return -1;
    int port, pin;
    if (parse_pin(argv[2], &port, &pin) != 0) { shell_error(s, "gpio: expected <port>.<pin>, got '%s'", argv[2]); return -1; }
    const char *op = argv[3];
    if (!strcmp(op, "high") || !strcmp(op, "low"))
        return drive_pin(s, "gpio", idx, port, pin, op[0] == 'h');
    if (!strcmp(op, "pulse")) {
        int64_t dur = 100 * SHELL_MS_TO_NS;
        if (argc == 5 && parse_dur(s, argv[4], &dur) != 0) return -1;
        if (drive_pin(s, "gpio", idx, port, pin, 1) != 0) return -1;
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "gpio %d %d.%d low", id, port, pin);
        return schedule_release(s, dur, cmd);
    }
    shell_error(s, "gpio: expected high, low or pulse, got '%s'", op);
    return -1;
}

/* button <node> press|release|click [duration] */
static int cmd_button(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int idx;
    int id = one_node(s, "button", argv[1], &idx);
    if (id < 0) return -1;
    int port, pin;
    bool active_low;
    if (!s->ctl->ops.button_pin || s->ctl->ops.button_pin(s->ctl->ops.user, idx, &port, &pin, &active_low) != 0) {
        shell_error(s, "button: node %d's board describes no user button (use gpio)", id); return -1;
    }
    const char *op = argv[2];
    int pressed = active_low ? 0 : 1;
    if (!strcmp(op, "press"))   return drive_pin(s, "button", idx, port, pin, pressed);
    if (!strcmp(op, "release")) return drive_pin(s, "button", idx, port, pin, !pressed);
    if (!strcmp(op, "click")) {
        int64_t dur = 100 * SHELL_MS_TO_NS;
        if (argc == 4 && parse_dur(s, argv[3], &dur) != 0) return -1;
        if (drive_pin(s, "button", idx, port, pin, pressed) != 0) return -1;
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "button %d release", id);
        return schedule_release(s, dur, cmd);
    }
    shell_error(s, "button: expected press, release or click, got '%s'", op);
    return -1;
}

static int cmd_restart(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (!s->ctl->ops.restart) { shell_error(s, "restart is not available in this mode"); return -1; }
    if (shell_refuse_external_clock(s, "restart")) return -1;
    s->ctl->ops.restart(s->ctl->ops.user);
    s->restart_pending = true;      /* the next line runs after the restart */
    return 0;
}

static int cmd_ui(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    long port;
    if (shell_parse_int(argv[1], &port) != 0 || port < 1 || port > 65535) { shell_error(s, "ui: expected a TCP port"); return -1; }
    if (!s->ctl->ops.start_ui || s->ctl->ops.start_ui(s->ctl->ops.user, (int)port) != 0) {
        shell_error(s, "ui: cannot start the web UI on port %ld (already running, or the port is taken)", port);
        return -1;
    }
    shell_out(s, "web UI on http://localhost:%ld/\n", port);
    return 0;
}

/* --- console history, watches, transcript, stats -------------------------- */

static bool id_in_list(const int *ids, int n, int id) {
    for (int i = 0; i < n; i++) if (ids[i] == id) return true;
    return false;
}

/* tail [-n N] [nodes] */
static int cmd_tail(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    long want = 20;
    int i = 1;
    if (argc >= 3 && !strcmp(argv[1], "-n")) {
        if (shell_parse_int(argv[2], &want) != 0 || want < 1) { shell_error(s, "tail: -n needs a count >= 1"); return -1; }
        i = 3;
    }
    if (argc - i > 1) { shell_error(s, "usage: tail [-n N] [nodes]"); return -1; }
    int ids[SIM_EQ_MAX_NODES], n = -1;
    if (argc - i == 1 && (n = shell_resolve_selector(s, argv[i], ids, SIM_EQ_MAX_NODES, false, NULL)) < 0) return -1;
    if (!s->hist) return 0;
    int found = 0, start = 0;
    for (int k = s->hist_count - 1; k >= 0 && found < want; k--) {
        const shell_hline_t *h = &s->hist[(s->hist_head + k) % SHELL_HISTORY_LINES];
        if (n < 0 || id_in_list(ids, n, h->node_id)) { found++; start = k; }
    }
    for (int k = start; found > 0 && k < s->hist_count; k++) {
        const shell_hline_t *h = &s->hist[(s->hist_head + k) % SHELL_HISTORY_LINES];
        if (n >= 0 && !id_in_list(ids, n, h->node_id)) continue;
        shell_out(s, "  %7.3f [Node %d] %s\n", (double)h->ns / 1e9, h->node_id, h->text);
    }
    return 0;
}

/* grep [-re] [-c] "<pattern>" [nodes] */
static int cmd_grep(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    bool use_re = false, count_only = false;
    int i = 1;
    for (; i < argc && argv[i][0] == '-' && argv[i][1]; i++) {
        if (!strcmp(argv[i], "-re")) use_re = true;
        else if (!strcmp(argv[i], "-c")) count_only = true;
        else { shell_error(s, "grep: unknown option '%s'", argv[i]); return -1; }
    }
    if (argc - i < 1 || argc - i > 2) { shell_error(s, "usage: grep [-re] [-c] \"<pattern>\" [nodes]"); return -1; }
    int ids[SIM_EQ_MAX_NODES], n = -1;
    if (argc - i == 2 && (n = shell_resolve_selector(s, argv[i + 1], ids, SIM_EQ_MAX_NODES, false, NULL)) < 0) return -1;
    void *re = NULL;
    if (use_re) {
        char err[160];
        if (!(re = shell_regex_compile(argv[i], err, sizeof(err)))) { shell_error(s, "%s", err); return -1; }
    }
    int hits = 0;
    for (int k = 0; s->hist && k < s->hist_count; k++) {
        const shell_hline_t *h = &s->hist[(s->hist_head + k) % SHELL_HISTORY_LINES];
        if (n >= 0 && !id_in_list(ids, n, h->node_id)) continue;
        if (!shell_line_match(h->text, argv[i], re, NULL, 0)) continue;
        hits++;
        if (!count_only)
            shell_out(s, "  %7.3f [Node %d] %s\n", (double)h->ns / 1e9, h->node_id, h->text);
    }
    shell_regex_free(&re);
    if (count_only) shell_out(s, "%d\n", hits);
    return 0;
}

static int cmd_on_list(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (s->watch_count == 0) { shell_out(s, "no watches\n"); return 0; }
    for (int i = 0; i < s->watch_count; i++) {
        const shell_watch_t *w = &s->watches[i];
        char nodes[128] = "any";
        if (!w->any) {
            nodes[0] = '\0';
            for (int k = 0; k < w->nids; k++)
                snprintf(nodes + strlen(nodes), sizeof(nodes) - strlen(nodes), "%s%d", k ? "," : "", w->ids[k]);
        }
        const char *kind = w->kind == SHELL_WATCH_COUNT ? "count" : w->kind == SHELL_WATCH_FAIL ? "fail-on" : w->once ? "on --once" : "on";
        shell_out(s, "  #%d %-9s %-8s \"%s\"%s%s  (%d matches)\n", i + 1, kind, nodes, w->pattern,
                  w->kind == SHELL_WATCH_RUN ? " -> " : "", w->kind == SHELL_WATCH_RUN ? w->cmd : "", w->count);
    }
    return 0;
}

static int cmd_on_clear(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (!strcmp(argv[1], "all")) { s->watch_count = 0; return 0; }
    long n;
    if (shell_parse_int(argv[1], &n) != 0 || n < 1 || n > s->watch_count) {
        shell_error(s, "on clear: no watch #%s (see on list)", argv[1]); return -1;
    }
    memmove(&s->watches[n - 1], &s->watches[n], (size_t)(s->watch_count - n) * sizeof(s->watches[0]));
    s->watch_count--;
    return 0;
}

static int cmd_transcript(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc == 1) {
        shell_out(s, s->transcript ? "transcript: %s\n" : "transcript: off\n", s->transcript_path);
        return 0;
    }
    if (s->transcript) { fclose(s->transcript); s->transcript = NULL; }
    if (!strcmp(argv[1], "off")) return 0;
    FILE *f = fopen(argv[1], "a");
    if (!f) { shell_error(s, "transcript: cannot open %s", argv[1]); return -1; }
    fprintf(f, "# transcript started at simulated time %.3f s\n", (double)now_ns(s) / 1e9);
    fflush(f);
    s->transcript = f;
    snprintf(s->transcript_path, sizeof(s->transcript_path), "%s", argv[1]);
    return 0;
}

static int cmd_history(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc != 3 || strcmp(argv[1], "save") != 0) { shell_error(s, "usage: history save <file>"); return -1; }
    if (!s->editor) { shell_error(s, "history: no line-editing history without a terminal"); return -1; }
    if (linenoiseHistorySave(argv[2]) != 0) { shell_error(s, "history: cannot write %s", argv[2]); return -1; }
    return 0;
}

static int cmd_stats(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    if (s->ctl->ops.stats) {
        sim_control_stats_t st = {0};
        s->ctl->ops.stats(s->ctl->ops.user, &st);
        shell_out(s, "  time %.3f s  rf bytes %ld  frames %ld  collided %ld  rx dropped %ld  console bytes %ld\n",
                  (double)now_ns(s) / 1e9, st.rf_bytes, st.frames, st.frames_collided, st.rx_dropped, st.uart_bytes);
    }
    int nn = sim_control_node_count(s->ctl);
    for (int i = 0; i < nn; i++) {
        sim_control_node_info_t info;
        if (!sim_control_describe(s->ctl, i, &info)) continue;
        shell_out(s, "  node %-4d %-7s %14lld cycles %14lld instructions  %.1f MHz%s\n", info.id,
                  info.type ? info.type : "?", (long long)info.cycles, (long long)info.instructions,
                  info.freq_hz / 1e6, info.removed ? "  (removed)" : "");
    }
    return 0;
}

static int cmd_assert(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    bool ok;
    if (eval_condition(s, argc, argv, &ok) != 0) return -1;
    if (!ok) { shell_assert_failed(s, "assertion failed: %s", cond_desc); return -1; }
    return 0;
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
    /* `on list` / `on clear` manage every console-line watch: on, fail-on, count. */
    if (!strcmp(argv[1], "list")) {
        if (argc != 2) { shell_error(s, "usage: on list"); return -1; }
        return cmd_on_list(s, 1, argv + 1, line, argpos);
    }
    if (!strcmp(argv[1], "clear")) {
        if (argc != 3) { shell_error(s, "usage: on clear <n>|all"); return -1; }
        return cmd_on_clear(s, 2, argv + 1, line, argpos);
    }
    bool once = !strcmp(argv[1], "--once");
    int a = once ? 2 : 1;
    if (argc < a + 3) { shell_error(s, "usage: on [--once] <nodes|any> \"<pattern>\" <command...>"); return -1; }
    const char *cmd = line + argpos[a + 2];
    if (check_command_text(s, "on", cmd) != 0) return -1;
    int before = s->watch_count;
    if (add_watch(s, SHELL_WATCH_RUN, argv[a], argv[a + 1], cmd) != 0) return -1;
    if (s->watch_count > before) s->watches[s->watch_count - 1].once = once;
    return 0;
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
    { "radio",      "radio [range <tx> [interference] | success <tx> [rx]]", "show or change the radio medium (UDGM range in metres, success ratios 0..1)", 0, 3, IMM, cmd_radio },
    { "link",       "link <a> <b> off|on | link <a> -> <b> off|on", "cut or restore a radio link (both directions, or one)", 3, 4, IMM, cmd_link },
    { "links",      "links",                           "list cut links", 0, 0, IMM, cmd_links },
    { "neighbors",  "neighbors [node]",                "who each node hears (the medium's neighbour lists)", 0, 1, IMM, cmd_neighbors },
    { "pcap",       "pcap <file>|off",                 "start or stop an 802.15.4 capture", 1, 1, IMM, cmd_pcap },
    { "clock",      "clock <node> [deviation]",        "show or set a node's clock deviation (1.0 exact, e.g. 1.00002 = 20 ppm fast)", 1, 2, IMM, cmd_clock },
    { "leds",       "leds [nodes]",                    "LED states", 0, 1, IMM, cmd_leds },
    { "gpio",       "gpio <node> <port>.<pin> high|low|pulse [duration]", "drive a GPIO input pin (MSP430 P1-P10, CC2538 A-D, nRF54L15 P0-P2 without GPIOTE)", 3, 4, IMM, cmd_gpio },
    { "button",     "button <node> press|release|click [duration]", "the board's user button (click = press, release after 100ms)", 2, 3, IMM, cmd_button },
    { "restart",    "restart",                         "restart the simulation from its configuration (aborts scripts, clears at)", 0, 0, IMM, cmd_restart },
    { "ui",         "ui <port>",                       "start the live web UI now", 1, 1, IMM, cmd_ui },
    { "stats",      "stats",                           "RF bytes, frames, collisions, console bytes; per-node cycles and instructions", 0, 0, IMM, cmd_stats },
    { "tail",       "tail [-n N] [nodes]",             "the last N console lines (default 20) from the remembered 2000", 0, 3, IMM, cmd_tail },
    { "grep",       "grep [-re] [-c] \"<pattern>\" [nodes]", "remembered console lines matching a pattern (-c: just count)", 1, 4, IMM, cmd_grep },
    { "transcript", "transcript [<file>|off]",         "append every command line you type or pipe to a file, to replay as a script", 0, 1, IMM, cmd_transcript },
    { "history",    "history save <file>",             "save the line-editing history (terminal only)", 2, 2, IMM, cmd_history },
    { "repeat",     "repeat <count> [var] ... end",    "scripts: run the lines up to `end` count times; $var counts 1..count", 1, 2, 0, cmd_repeat },
    { "if",         "if <condition> ... [else ...] end", "scripts: conditions as for assert (time, nodes, node, count, mem, var)", 3, 5, 0, cmd_if },
    { "else",       "else",                            "scripts: see if", 0, 0, 0, cmd_else },
    { "end",        "end",                             "scripts: closes repeat / if", 0, 0, 0, cmd_end },
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
    { "break",      "break <node> <addr|sym[+off]> | break list | break clear <n>|all", "stop the node (and pause the simulation) before it executes that address; list or remove breakpoints and watchpoints", 1, 3, IMM, cmd_breakpoint },
    { "watch",      "watch <node> <addr|sym[+off]> [bytes]", "stop when an SRAM value (1-4 bytes, default 4) changes; reports the writing pc", 2, 3, IMM, cmd_watch },
    { "continue",   "continue [node]",                 "release halted nodes and resume", 0, 1, IMM, cmd_continue },
    { "expect-halt","expect-halt <node> [timeout]",    "block until the node hits a breakpoint or watchpoint", 1, 2, BLK, cmd_expect_halt },
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
    { "on",         "on [--once] <nodes|any> \"<pattern>\" <command...> | on list | on clear <n>|all", "run a command whenever (or the first time) a console line contains pattern; list or remove on / fail-on / count watches", 1, -1, IMM, cmd_on },
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
