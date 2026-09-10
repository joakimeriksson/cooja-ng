/* Private interfaces shared by shell_service.c / shell_commands.c /
 * shell_script.c.  Not installed. */
#ifndef SHELL_INTERNAL_H
#define SHELL_INTERNAL_H

#include "shell_service.h"
#include "shell_parse.h"

#define SHELL_MS_TO_NS 1000000LL

/* --- output (shell_service.c) ------------------------------------------ */

/* Print to the terminal, hiding/redrawing the prompt around the text. */
void shell_out(shell_service_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
/* "error: ..." — and, when the line came from a script file, fails it. */
void shell_error(shell_service_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
/* Where the current line comes from, for messages ("file:12" or "stdin"). */
const char *shell_origin(shell_service_t *s, char *buf, size_t len);

/* --- command execution (shell_commands.c) ------------------------------ */

#define SHELL_CMD_IMMEDIATE 1u   /* allowed via "!" while the stream is blocked */

typedef int (*shell_cmd_fn)(shell_service_t *s, int argc, char **argv,
                            const char *line, const int *argpos);
typedef struct shell_command {
    const char *name;
    const char *syntax;
    const char *help;
    int         min_args, max_args;   /* max_args < 0 = variadic */
    unsigned    flags;
    shell_cmd_fn fn;
} shell_command_t;

const shell_command_t *shell_find_command(const char *name);
void shell_print_help(shell_service_t *s, const char *name);
void shell_complete(const char *prefix, linenoiseCompletions *lc);

/* Execute one line now (tokenize + dispatch).  `immediate_only` restricts
 * to SHELL_CMD_IMMEDIATE commands ("!" prefix while blocked).  Returns 0,
 * or -1 after printing an error. */
int  shell_exec_line(shell_service_t *s, const char *line, bool immediate_only);

/* Resolve a selector against the current node table (prints errors).
 * Returns the count, 0 with *any for "any", -1 on error. */
int  shell_resolve_selector(shell_service_t *s, const char *sel, int *ids,
                            int max, bool allow_any, bool *any);

/* --- script engine (shell_script.c) ------------------------------------ */

void shell_script_init(shell_service_t *s);
/* One engine tick at a slice boundary (now == sim now). */
void shell_script_tick(shell_service_t *s);
/* A console line (observer context: no command execution here). */
void shell_script_on_log_line(shell_service_t *s, int idx, int node_id,
                              const char *line, int64_t ns);
int  shell_script_source(shell_service_t *s, const char *path);
/* Pop every file source and clear the block (Ctrl-C, fail). */
void shell_script_abort(shell_service_t *s);
void shell_script_fail(shell_service_t *s, const char *reason);
void shell_script_pass(shell_service_t *s);
/* Blocking commands arm the block; the tick resolves it. */
void shell_script_block_expect(shell_service_t *s, const char *pattern,
                               const int *ids, int nids, bool any,
                               int64_t timeout_ns);
void shell_script_block_until(shell_service_t *s, shell_block_t kind,
                              int64_t deadline_ns);
int  shell_script_at_add(shell_service_t *s, int64_t at_ns, int64_t period_ns,
                         const char *cmd);
int  shell_script_at_remove(shell_service_t *s, int id);   /* id < 0 = all */
int  shell_script_watch_add(shell_service_t *s, shell_watch_kind_t kind,
                            const char *pattern, const int *ids, int nids,
                            bool any, const char *cmd);
/* Pin a slice boundary at t (SIM_EV_TEST_ACTION, dispatch is a no-op). */
void shell_pin(shell_service_t *s, int64_t t);
static inline bool shell_in_script(const shell_service_t *s) {
    return s->depth > 0;
}
/* Enqueue a stdin line for the stream ("!cmd" runs at once when allowed). */
void shell_enqueue_line(shell_service_t *s, const char *line);

#endif /* SHELL_INTERNAL_H */
