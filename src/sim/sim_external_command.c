/*
 * sim_external_command — external test-driver process + COOJA.testlog.
 * See include/sim/sim_external_command.h.
 */
#include "sim_external_command.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

/* Tee every assembled console line into COOJA.testlog in Cooja's
 * `log.log(time + " " + id + " " + msg + "\n")` format. */
static void testlog_observer(void *user, const sim_observer_event_t *ev) {
    sim_external_command_t *ec = (sim_external_command_t *)user;
    if (ev->kind != SIM_OBS_MOTE_LOG_LINE || !ec->testlog) return;
    fprintf(ec->testlog, "%lld %d %s\n",
            (long long)(ev->time_ns / 1000000LL),
            ev->u.log_line.node_id, ev->u.log_line.line);
    fflush(ec->testlog);
}

void sim_external_command_init(sim_external_command_t *ec) {
    memset(ec, 0, sizeof(*ec));
    ec->child_pid = -1;
    ec->child_status = -1;
    ec->observer_handle = -1;
}

/* Open COOJA.testlog in the script's directory before launching the
 * script — bash drivers (test-native-nat64.sh et al) `tail -F` or grep
 * this file as the run unfolds.  First whitespace-separated token in
 * the command line is the script path. */
static void open_testlog(sim_external_command_t *ec, const char *command) {
    char script_path[1024];
    snprintf(script_path, sizeof(script_path), "%s", command);
    char *space = strchr(script_path, ' ');
    if (space) *space = '\0';
    char *slash = strrchr(script_path, '/');
    if (!slash) return;  /* no directory component — skip the tee */
    *slash = '\0';
    char logpath[1100];
    snprintf(logpath, sizeof(logpath), "%s/COOJA.testlog", script_path);
    ec->testlog = fopen(logpath, "w");
    if (ec->testlog) {
        printf("  Serial socket: writing COOJA.testlog to %s\n", logpath);
    } else {
        fprintf(stderr, "serial_socket: cannot open %s: %s\n",
                logpath, strerror(errno));
    }
}

int sim_external_command_start(sim_external_command_t *ec, sim_runtime_t *sim,
                               const char *command) {
    sim_external_command_init(ec);
    ec->sim = sim;
    if (!command || !command[0]) return -1;

    open_testlog(ec, command);
    if (ec->testlog)
        ec->observer_handle = sim_runtime_subscribe(sim, testlog_observer, ec);

    pid_t pid = fork();
    if (pid < 0) {
        perror("serial_socket: fork");
        /* Undo the pre-fork setup: the runtime must not keep a fan-out
         * observer pointing at a service the caller believes failed, and
         * the testlog FILE* must not leak. */
        if (ec->observer_handle >= 0) {
            sim_runtime_unsubscribe(sim, ec->observer_handle);
            ec->observer_handle = -1;
        }
        if (ec->testlog) { fclose(ec->testlog); ec->testlog = NULL; }
        return -1;
    }
    if (pid == 0) {
        /* Child: exec the command via shell */
        execl("/bin/sh", "sh", "-c", command, (char *)NULL);
        perror("serial_socket: exec");
        _exit(127);
    }

    ec->child_pid = pid;
    printf("  Serial socket: launched command (pid %d): %s\n", pid, command);
    return 0;
}

void sim_external_command_poll(sim_external_command_t *ec) {
    if (ec->child_pid <= 0 || ec->child_exited) return;

    int status;
    pid_t res = waitpid(ec->child_pid, &status, WNOHANG);
    if (res > 0) {
        ec->child_exited = true;
        if (WIFEXITED(status)) {
            ec->child_status = WEXITSTATUS(status);
            printf("  Serial socket: command exited with status %d\n",
                   ec->child_status);
        } else {
            ec->child_status = 1;
            printf("  Serial socket: command killed by signal\n");
        }
    }
}

void sim_external_command_stop(sim_external_command_t *ec) {
    if (ec->child_pid > 0 && !ec->child_exited) {
        kill(ec->child_pid, SIGTERM);
        usleep(100000);
        kill(ec->child_pid, SIGKILL);
        waitpid(ec->child_pid, NULL, 0);
        ec->child_pid = -1;
    }
    if (ec->testlog) { fclose(ec->testlog); ec->testlog = NULL; }
    if (ec->observer_handle >= 0 && ec->sim) {
        sim_runtime_unsubscribe(ec->sim, ec->observer_handle);
        ec->observer_handle = -1;
    }
}
