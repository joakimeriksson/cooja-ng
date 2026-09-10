/*
 * shell_commands — the command table and handlers of the Cooja-NG shell
 * (docs/shell.md).  Every mutation goes through sim_control; blocking and
 * scheduling commands hand over to the engine in shell_script.c.
 */
#include "shell_internal.h"
#include "sim_runtime.h"

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

/* --- helpers --------------------------------------------------------------- */

static int64_t now_ns(shell_service_t *s) { return sim_runtime_now_ns(s->sim); }

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
    (void)line; (void)argpos;
    if (argc >= 2) {
        int64_t d;
        if (parse_dur(s, argv[1], &d) != 0) return -1;
        sim_control_run_for(s->ctl, d);
        s->run_for_target_ns = now_ns(s) + d;
        /* Hold the command stream until the auto-pause, so `run 500ms`
         * followed by `status` in a script/pipe sees the later time. */
        shell_script_block_until(s, SHELL_BLOCK_RUN, s->run_for_target_ns);
    } else {
        sim_control_resume(s->ctl);
    }
    return 0;
}

static int cmd_pause(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)argv; (void)line; (void)argpos;
    sim_control_pause(s->ctl);
    return 0;
}

static int cmd_step(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    if (argc < 2) {
        sim_control_step_events(s->ctl, 1);
        shell_script_block_until(s, SHELL_BLOCK_RUN, INT64_MAX);
        return 0;
    }
    long n;
    if (shell_parse_int(argv[1], &n) == 0) {
        if (n < 1) { shell_error(s, "step count must be >= 1"); return -1; }
        sim_control_step_events(s->ctl, (int)(n > INT_MAX ? INT_MAX : n));
        shell_script_block_until(s, SHELL_BLOCK_RUN, INT64_MAX);
        return 0;
    }
    int64_t d;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    sim_control_run_for(s->ctl, d);
    shell_script_block_until(s, SHELL_BLOCK_RUN, now_ns(s) + d);
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

static int do_send(shell_service_t *s, int argc, char **argv, bool newline) {
    int ids[SIM_EQ_MAX_NODES];
    int n = shell_resolve_selector(s, argv[1], ids, SIM_EQ_MAX_NODES, false, NULL);
    if (n < 0) return -1;
    char text[SHELL_LINE_MAX];
    join_args(argc, argv, 2, text, sizeof(text) - 1);
    if (newline) strncat(text, "\n", sizeof(text) - strlen(text) - 1);
    int len = (int)strlen(text);
    for (int i = 0; i < n; i++) {
        int idx = sim_control_index_of_id(s->ctl, ids[i]);
        if (idx < 0 || !sim_control_node_active(s->ctl, idx)) {
            shell_error(s, "node %d is not running", ids[i]); return -1;
        }
        sim_control_send(s->ctl, ids[i], (const uint8_t *)text, len,
                         SIM_CONTROL_WAKE | SIM_CONTROL_RETRY);
    }
    return 0;
}

static int cmd_send(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    return do_send(s, argc, argv, false);
}

static int cmd_sendln(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    return do_send(s, argc, argv, true);
}

/* --- scheduling ------------------------------------------------------------ */

static int check_command_text(shell_service_t *s, const char *cmd) {
    char *argv[SHELL_MAX_ARGS]; char storage[SHELL_LINE_MAX]; char err[128];
    int argc = shell_tokenize(cmd, argv, NULL, SHELL_MAX_ARGS, storage, sizeof(storage), err, sizeof(err));
    if (argc < 0) { shell_error(s, "%s", err); return -1; }
    if (argc == 0) { shell_error(s, "missing command"); return -1; }
    if (!shell_find_command(argv[0])) { shell_error(s, "unknown command '%s'", argv[0]); return -1; }
    return 0;
}

static int cmd_at(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc;
    int64_t t;
    if (parse_instant(s, argv[1], &t) != 0) return -1;
    if (t <= now_ns(s)) { shell_error(s, "at: time %.3f s is not in the future", (double)t / 1e9); return -1; }
    const char *cmd = line + argpos[2];
    if (check_command_text(s, cmd) != 0) return -1;
    int id = shell_script_at_add(s, t, 0, cmd);
    if (id < 0) { shell_error(s, "at: queue full (max %d)", SHELL_ATQ_MAX); return -1; }
    if (s->verbose) shell_out(s, "at #%d scheduled for %.3f s\n", id, (double)t / 1e9);
    return 0;
}

static int cmd_every(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc;
    int64_t d;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    if (d <= 0) { shell_error(s, "every: period must be > 0"); return -1; }
    const char *cmd = line + argpos[2];
    if (check_command_text(s, cmd) != 0) return -1;
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
    if (shell_parse_int(argv[1], &id) != 0) { shell_error(s, "atrm: bad id '%s'", argv[1]); return -1; }
    if (shell_script_at_remove(s, (int)id) == 0) { shell_error(s, "atrm: no scheduled command #%ld", id); return -1; }
    return 0;
}

/* --- scripting ------------------------------------------------------------- */

static int cmd_source(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    if (shell_script_source(s, argv[1]) != 0) { shell_error(s, "source: cannot open %s", argv[1]); return -1; }
    return 0;
}

static int cmd_expect(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    int ids[SIM_EQ_MAX_NODES]; bool any = false;
    int n = shell_resolve_selector(s, argv[1], ids, SIM_EQ_MAX_NODES, true, &any);
    if (n < 0) return -1;
    if (!argv[2][0]) { shell_error(s, "expect: empty pattern"); return -1; }
    int64_t timeout = s->default_expect_timeout_ns;
    if (argc >= 4 && parse_dur(s, argv[3], &timeout) != 0) return -1;
    shell_script_block_expect(s, argv[2], ids, n, any, timeout);
    return 0;
}

static int cmd_sleep(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int64_t d;
    if (parse_dur(s, argv[1], &d) != 0) return -1;
    shell_script_block_until(s, SHELL_BLOCK_SLEEP, now_ns(s) + d);
    return 0;
}

static int cmd_wait_until(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)argc; (void)line; (void)argpos;
    int64_t t;
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
    if (strcmp(what, "time") == 0 && argc == 4) {
        int64_t t;
        if (parse_dur(s, argv[3], &t) != 0) return -1;
        int r = shell_compare((long)(now_ns(s) / 1000), argv[2], (long)(t / 1000));
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { shell_error(s, "assertion failed: time (%.6f s) %s %s", (double)now_ns(s) / 1e9, argv[2], argv[3]); return -1; }
        return 0;
    }
    if (strcmp(what, "nodes") == 0 && argc == 4) {
        long v;
        if (shell_parse_int(argv[3], &v) != 0) { shell_error(s, "assert: bad count '%s'", argv[3]); return -1; }
        int n = sim_control_node_count(s->ctl), active = 0;
        for (int i = 0; i < n; i++) if (sim_control_node_active(s->ctl, i)) active++;
        int r = shell_compare(active, argv[2], v);
        if (r < 0) { shell_error(s, "assert: bad operator '%s'", argv[2]); return -1; }
        if (!r) { shell_error(s, "assertion failed: nodes (%d active) %s %ld", active, argv[2], v); return -1; }
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
        else { shell_error(s, "assert node: expected active|removed|exists"); return -1; }
        if (!ok) { shell_error(s, "assertion failed: node %ld %s", id, argv[3]); return -1; }
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
        if (!r) { shell_error(s, "assertion failed: count \"%s\" (%d) %s %ld", argv[2], w->count, argv[3], v); return -1; }
        return 0;
    }
    shell_error(s, "usage: assert time <op> <t> | nodes <op> N | node <id> active|removed|exists | count \"pat\" <op> N");
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
        return 0;
    }
    if (strcmp(argv[1], "expect-timeout") == 0 && argc == 3) {
        int64_t d;
        if (parse_dur(s, argv[2], &d) != 0) return -1;
        s->default_expect_timeout_ns = d;
        return 0;
    }
    shell_error(s, "usage: set expect-timeout <duration>");
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
    if (check_command_text(s, cmd) != 0) return -1;
    return add_watch(s, SHELL_WATCH_RUN, argv[1], argv[2], cmd);
}

/* --- misc ------------------------------------------------------------------ */

static int cmd_echo(shell_service_t *s, int argc, char **argv, const char *line, const int *argpos) {
    (void)line; (void)argpos;
    char text[SHELL_LINE_MAX];
    join_args(argc, argv, 1, text, sizeof(text));
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
static const shell_command_t commands[] = {
    { "run",        "run [duration]",                  "resume; with a duration, pause again after it (run 500ms)", 0, 1, IMM, cmd_run },
    { "pause",      "pause",                           "stop dispatching events (services keep polling)", 0, 0, IMM, cmd_pause },
    { "step",       "step [N|duration]",               "run N events (default 1) or a duration, then pause", 0, 1, IMM, cmd_step },
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
    { "at",         "at <time> <command...>",          "run a command at a simulation time (5s, 1500ms, +2s)", 2, -1, IMM, cmd_at },
    { "every",      "every <period> <command...>",     "run a command periodically, first after one period", 2, -1, IMM, cmd_every },
    { "atq",        "atq",                             "list scheduled commands", 0, 0, IMM, cmd_atq },
    { "atrm",       "atrm <id>|all",                   "cancel scheduled command(s)", 1, 1, IMM, cmd_atrm },
    { "source",     "source <file>",                   "run a script file (nested up to 8 deep)", 1, 1, 0, cmd_source },
    { "expect",     "expect <nodes|any> \"<pattern>\" [timeout]", "block until a console line contains pattern; timeout fails the script", 2, 3, 0, cmd_expect },
    { "sleep",      "sleep <duration>",                "block for a simulated duration", 1, 1, 0, cmd_sleep },
    { "wait-until", "wait-until <time>",               "block until a simulation time (absolute or +relative)", 1, 1, 0, cmd_wait_until },
    { "assert",     "assert time|nodes <op> <v> | node <id> active|removed|exists | count \"pat\" <op> N", "check a condition; failure fails the script", 3, 4, 0, cmd_assert },
    { "pass",       "pass",                            "end the script with a PASS verdict", 0, 0, 0, cmd_pass },
    { "fail",       "fail [message...]",               "end the script with a FAIL verdict (non-zero exit code)", 0, -1, 0, cmd_fail },
    { "fail-on",    "fail-on \"<pattern>\" [nodes|any]", "fail the script as soon as a console line contains pattern", 1, 2, 0, cmd_fail_on },
    { "count",      "count \"<pattern>\" [nodes|any]", "count console lines containing pattern (see assert count)", 1, 2, 0, cmd_count },
    { "on",         "on <nodes|any> \"<pattern>\" <command...>", "run a command whenever a console line contains pattern", 3, -1, 0, cmd_on },
    { "set",        "set [expect-timeout <duration>]", "settings; default expect timeout is 30s", 0, 2, IMM, cmd_set },
    { "echo",       "echo <text...>",                  "print text", 0, -1, IMM, cmd_echo },
    { "save-config","save-config <file.yaml>",         "write the live setup (positions, nodes, seed) as a config", 1, 1, IMM, cmd_save_config },
    { "help",       "help [command]",                  "this list, or one command's syntax", 0, 1, IMM, cmd_help },
};
#undef IMM
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
    shell_out(s, "Commands (nodes = id, 1,3, 2-5, all; times = 5s, 250ms, +2s; \"!cmd\" runs at once while a script blocks):\n");
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

int shell_exec_line(shell_service_t *s, const char *line, bool immediate_only) {
    char *argv[SHELL_MAX_ARGS];
    int argpos[SHELL_MAX_ARGS];
    char storage[SHELL_LINE_MAX];
    char err[128];
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
    int nargs = argc - 1;
    if (nargs < c->min_args || (c->max_args >= 0 && nargs > c->max_args)) {
        shell_error(s, "usage: %s", c->syntax);
        return -1;
    }
    return c->fn(s, argc, argv, line, argpos);
}
