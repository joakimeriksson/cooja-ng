/*
 * sim_external_command — external test-driver process + COOJA.testlog.
 *
 * Phase 1 milestone 8.2 (docs/design/refactor-plan.md §Phase 6,
 * serial-socket caveat): the second half of the legacy serial_socket
 * block.  Owns the forked bash test driver (test-border-router.sh and
 * friends) and the Cooja-compatibility COOJA.testlog file that those
 * drivers `tail -F`/grep for mote-side markers.
 *
 * The testlog tee subscribes to SIM_OBS_MOTE_LOG_LINE on the kernel
 * observer stream — every assembled console line from every mote is
 * written as `<time-ms> <node-id> <line>`, matching Cooja's
 * `log.log(time + " " + id + " " + msg + "\n")` format so existing
 * bash drivers grep it identically.
 *
 * The service never steps a CPU and never touches sim state; the
 * runner polls sim_external_command_exited() to decide when to stop
 * the simulation (the child's exit code is the test verdict).
 */
#ifndef SIM_EXTERNAL_COMMAND_H
#define SIM_EXTERNAL_COMMAND_H

#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "sim_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sim_external_command {
    sim_runtime_t *sim;
    pid_t  child_pid;       /* -1 = no child                   */
    bool   child_exited;
    int    child_status;    /* exit code once child_exited     */
    FILE  *testlog;         /* COOJA.testlog tee, NULL = none  */
    int    observer_handle; /* LOG_LINE subscription, -1 unset */
} sim_external_command_t;

/* Zero/sentinel init; no fork, no file. */
void sim_external_command_init(sim_external_command_t *ec);

/* Open COOJA.testlog next to the command's script (first whitespace-
 * separated token's directory — bash drivers grep `$THIS_DIR/COOJA.testlog`
 * as the run unfolds), subscribe the tee to the observer stream, then
 * fork/exec `command` via /bin/sh.  Returns 0 if the child launched,
 * -1 otherwise (testlog failure alone is non-fatal, matching legacy). */
int sim_external_command_start(sim_external_command_t *ec, sim_runtime_t *sim,
                               const char *command);

/* Non-blocking child reap.  Call once per runner loop iteration. */
void sim_external_command_poll(sim_external_command_t *ec);

static inline bool sim_external_command_running(const sim_external_command_t *ec) {
    return ec->child_pid > 0 && !ec->child_exited;
}
static inline bool sim_external_command_exited(const sim_external_command_t *ec) {
    return ec->child_exited;
}
static inline int sim_external_command_status(const sim_external_command_t *ec) {
    return ec->child_status;
}
static inline bool sim_external_command_launched(const sim_external_command_t *ec) {
    return ec->child_pid > 0;
}

/* SIGTERM→SIGKILL a still-running child, close the testlog,
 * unsubscribe. */
void sim_external_command_stop(sim_external_command_t *ec);

#ifdef __cplusplus
}
#endif

#endif /* SIM_EXTERNAL_COMMAND_H */
