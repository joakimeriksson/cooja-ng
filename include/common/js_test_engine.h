/*
 * JavaScript test engine for running COOJA test scripts natively.
 *
 * Embeds QuickJS to execute COOJA's JavaScript test scripts verbatim,
 * implementing the COOJA scripting API: TIMEOUT, YIELD, WAIT_UNTIL,
 * GENERATE_MSG, log, sim, write, msg/id/time globals.
 *
 * The JS script runs in a separate pthread. YIELD() blocks on a condvar
 * until the main simulation thread feeds the next console line.
 */
#ifndef JS_TEST_ENGINE_H
#define JS_TEST_ENGINE_H

#include "sim_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#define JS_MAX_PENDING_ACTIONS 64
#define JS_MAX_GEN_MSGS 64

typedef struct {
    int64_t at_us;      /* simulation time in microseconds */
    char label[256];
} js_gen_msg_t;

typedef struct {
    /* Threading */
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t line_ready;   /* main → JS: new line available */
    pthread_cond_t line_done;    /* JS → main: done processing line */

    /* QuickJS context (owned by JS thread) */
    void *rt;   /* JSRuntime* */
    void *ctx;  /* JSContext* */

    /* Shared state: main thread writes, JS thread reads (under mutex) */
    char msg[512];          /* current console line */
    int id;                 /* source node ID */
    int64_t time_us;        /* simulation time in microseconds */
    bool has_line;          /* true when a new line is pending */
    bool shutdown;          /* true when simulation is ending */

    /* Result: JS thread writes, main thread reads (under mutex) */
    int finished;           /* 0=running, 1=pass, -1=fail */
    char fail_reason[512];

    /* Action queue: JS thread writes, main thread drains (under mutex) */
    sim_test_action_t pending_actions[JS_MAX_PENDING_ACTIONS];
    int pending_action_count;

    /* GENERATE_MSG schedule (JS thread writes during init) */
    js_gen_msg_t gen_msgs[JS_MAX_GEN_MSGS];
    int gen_msg_count;
    int gen_msg_next;       /* index of next undelivered msg */

    /* Timeout */
    int64_t timeout_us;     /* 0 = no timeout */

    /* Node list */
    int node_ids[128];
    int node_count;
    bool node_removed[128];  /* true if removeMote was called for this node */

    /* Tracks whether JS thread is waiting in YIELD */
    bool js_waiting;

    /* Set by main thread to tell JS thread to eval __timeout_cb() */
    volatile bool timeout_fired;
} js_test_engine_t;

/*
 * Initialize the JS test engine and start the JS thread.
 * script: the JavaScript source code to execute.
 * node_ids: array of node IDs in the simulation.
 * node_count: number of nodes.
 * Returns 0 on success, -1 on error.
 */
int js_test_init(js_test_engine_t *engine, const char *script,
                 const int *node_ids, int node_count);

/*
 * Feed a console output line to the JS engine.
 * Wakes the JS thread from YIELD(), waits for it to process the line.
 * Returns: 0=still running, 1=test passed, -1=test failed.
 */
int js_test_feed_line(js_test_engine_t *engine, const char *line,
                      int node_id, int64_t sim_time_us);

/*
 * Check for GENERATE_MSG events that should fire at the given time.
 * Delivers them as synthetic console lines with id=-1.
 * Returns: 0=still running, 1=test passed, -1=test failed.
 */
int js_test_check_gen_msgs(js_test_engine_t *engine, int64_t sim_time_us);

/*
 * Check if timeout has been reached.
 * Returns: 0=not reached, -1=timeout (test failed).
 */
int js_test_check_timeout(js_test_engine_t *engine, int64_t sim_time_us);

/*
 * Drain pending actions (write, move, remove, add) from the JS engine.
 * Copies actions to dst, returns the number of actions copied.
 */
int js_test_drain_actions(js_test_engine_t *engine,
                          sim_test_action_t *dst, int max_actions);

/*
 * Get the test result string (pass/fail reason).
 */
const char *js_test_fail_reason(const js_test_engine_t *engine);

/*
 * Destroy the JS engine: cancel thread, free QuickJS context.
 */
void js_test_destroy(js_test_engine_t *engine);

#endif /* JS_TEST_ENGINE_H */
