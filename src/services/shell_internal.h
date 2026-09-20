/* Private interfaces shared by shell_service.c / shell_commands.c /
 * shell_script.c.  Not installed. */
#ifndef SHELL_INTERNAL_H
#define SHELL_INTERNAL_H

#include "shell_service.h"
#include "shell_parse.h"

#define SHELL_MS_TO_NS 1000000LL
/* A prompt counts only once the console has been quiet this long after it
 * (about 20 characters at 115200 baud): a prompt followed at once by more
 * text on the same line is output, not the end of the command. */
#define SHELL_PROMPT_QUIET_NS (2 * SHELL_MS_TO_NS)

/* --- output (shell_service.c) ------------------------------------------ */

/* Print to the terminal, hiding/redrawing the prompt around the text. */
void shell_out(shell_service_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
/* An error in the request itself (unknown command, bad selector, a node
 * that does not exist): prints, and fails a script with EXIT_INVALID. */
void shell_error(shell_service_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
/* Where the current line comes from, for messages ("file:12" or "stdin"). */
/* A failed `assert` (EXIT_ASSERT, not EXIT_INVALID). */
void shell_assert_failed(shell_service_t *s, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
const char *shell_origin(shell_service_t *s, char *buf, size_t len);

/* --- command execution (shell_commands.c) ------------------------------ */

#define SHELL_CMD_IMMEDIATE 1u   /* allowed via "!" while the stream is blocked */
#define SHELL_CMD_BLOCKING  2u   /* holds the command stream: not from at/every/on */

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
int  shell_exec_line(shell_service_t *s, const char *line, bool immediate_only,
                     const shell_origin_t *origin);
/* Would this command line block the stream?  (expect, sleep, wait-until,
 * step, source, and run with a duration.) */
bool shell_line_blocks(const char *line);
/* Refuse a command that needs the simulation's own clock while an external
 * clock source (Renode) drives it.  Returns true (and prints) if refused. */
bool shell_refuse_external_clock(shell_service_t *s, const char *what);

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
/* Fail with an explicit exit code (shell_script_fail = ASSERT). */
void shell_script_fail_code(shell_service_t *s, int code, const char *reason);
void shell_script_pass(shell_service_t *s);
/* Blocking commands arm the block; the tick resolves it. */
void shell_script_block_expect(shell_service_t *s, const char *pattern,
                               const int *ids, int nids, bool any,
                               int64_t timeout_ns);
void shell_script_block_until(shell_service_t *s, shell_block_t kind,
                              int64_t deadline_ns);
/* `cmd`: arm the prompt wait for slot idx (the line was already sent). */
void shell_script_block_cmd(shell_service_t *s, int idx, int node_id,
                            const char *text, const char *expect,
                            const char *fail_on, int64_t timeout_ns);
/* A raw console byte (observer context). */
void shell_script_on_uart_byte(shell_service_t *s, int idx, uint8_t byte,
                               int64_t ns);
/* `console`: switch the terminal to talking to one node. */
int  shell_console_enter(shell_service_t *s, int idx, int node_id);

/* Append a typed/piped command line to the transcript, if one is open. */
void shell_transcript_record(shell_service_t *s, const char *line);
/* Condition shared by assert and if (argv[0] is the keyword). */
int  shell_eval_condition(shell_service_t *s, int argc, char **argv,
                          bool *result, const char **why);

/* Variables. */
const char *shell_var_get(void *s, const char *name);   /* shell_var_lookup_fn */
int  shell_var_set(shell_service_t *s, const char *name, const char *value);
bool shell_var_name_ok(const char *name);

/* Compile a POSIX extended regex (heap); NULL with `err` filled on error. */
void *shell_regex_compile(const char *pattern, char *err, size_t errlen);
void  shell_regex_free(void **re);
/* Substring or regex match; with `cap`, the capture (group 1, else the whole
 * match; for a substring, the whole line). */
bool  shell_line_match(const char *line, const char *pattern, void *re,
                       char *cap, size_t caplen);

/* expect / capture / expect-not.  `re` (may be NULL) is adopted. */
void shell_script_block_expect2(shell_service_t *s, shell_block_t kind,
                                const char *pattern, void *re, int needed,
                                const char *var, const int *ids, int nids,
                                bool any, int64_t timeout_ns);
/* `cmd -c var "re"`: set after shell_script_block_cmd.  `re` is adopted. */
void shell_script_cmd_capture(shell_service_t *s, const char *var, void *re);
/* `expect-fault`. */
void shell_script_block_fault(shell_service_t *s, int idx, int node_id,
                              unsigned mask, const char *what,
                              const uint64_t *base, int64_t timeout_ns);
/* Breakpoints/watchpoints: re-arm CPUs that lost them (reboot), report
 * hits (pausing the simulation), release expect-halt.  Called every tick. */
void shell_debug_tick(shell_service_t *s);
int  shell_dbg_halted_node(shell_service_t *s);

/* `sendfile`: push a file whose lines are sent to a node as `cmd`s. */
int  shell_script_sendfile(shell_service_t *s, const char *path, int idx,
                           int node_id, int64_t timeout_ns);
int  shell_script_at_add(shell_service_t *s, int64_t at_ns, int64_t period_ns,
                         const char *cmd);
int  shell_script_at_remove(shell_service_t *s, int id);   /* id < 0 = all */
int  shell_script_watch_add(shell_service_t *s, shell_watch_kind_t kind,
                            const char *pattern, const int *ids, int nids,
                            bool any, const char *cmd);
/* Pin a slice boundary at t (SIM_EV_TEST_ACTION, dispatch is a no-op). */
void shell_pin(shell_service_t *s, int64_t t);
/* Hide the prompt for a burst of output; release redraws it once. */
void shell_hold_output(shell_service_t *s);
void shell_release_output(shell_service_t *s);
/* Wait (blocking) for piped stdin to yield a line or EOF (sync_stdin). */
void shell_read_stdin_sync(shell_service_t *s);
/* Deliver a pending SIGINT/SIGTERM as `exit`.  True if one was pending. */
bool shell_check_signal(shell_service_t *s);
/* Enqueue a stdin line for the stream ("!cmd" runs at once when allowed). */
void shell_enqueue_line(shell_service_t *s, const char *line);

#endif /* SHELL_INTERNAL_H */
