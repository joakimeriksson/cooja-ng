/*
 * JSON simulation configuration loader
 *
 * Loads simulation scenarios from JSON config files:
 *   {
 *     "title": "RPL-UDP test",
 *     "timeout_ms": 60000,
 *     "seed": 123456,
 *     "nodes": [
 *       { "firmware": "firmware/cooja/udp-server.cooja", "id": 1 },
 *       { "firmware": "firmware/cooja/udp-client.cooja", "id": 2 }
 *     ]
 *   }
 *
 * Node type auto-detected from firmware extension (.sky / .cc2538dk / .cooja).
 */
#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <stdint.h>

#define MAX_SIM_NODES 128
#define MAX_TEST_STEPS 32
#define MAX_FAIL_PATTERNS 8
#define MAX_TEST_ACTIONS 64
#define MAX_TEST_VALIDATORS 8

typedef struct {
    char pattern[256];
    int  node;          /* -1 = any */
    int  count;         /* default 1 */
    int  timeout_ms;    /* 0 = use global */
} sim_test_step_t;

typedef enum {
    TEST_ACTION_MOVE = 1,
    TEST_ACTION_SEND = 2,
    TEST_ACTION_REMOVE = 3,
    TEST_ACTION_ADD = 4,
    TEST_ACTION_SEND_ALL = 5,
} test_action_type_t;

typedef struct {
    char pattern[256];
    int  min_count;    /* minimum matches required at end of test */
    int  node;         /* -1 = any */
} sim_test_validator_t;

typedef struct {
    test_action_type_t type;
    int64_t at_ms;
    int     node;       /* node ID */
    double  x, y;       /* for move */
    char    data[256];  /* for send */
} sim_test_action_t;

typedef struct {
    int step_count;
    sim_test_step_t steps[MAX_TEST_STEPS];

    /* fail_on: patterns that cause immediate test failure */
    int fail_on_count;
    char fail_on[MAX_FAIL_PATTERNS][256];

    /* timeout_is_success: reaching timeout without failure = PASS */
    int timeout_is_success;

    /* timed actions (move, send) */
    int action_count;
    sim_test_action_t actions[MAX_TEST_ACTIONS];

    /* end-of-test validators: pattern counters checked at timeout */
    int validator_count;
    sim_test_validator_t validators[MAX_TEST_VALIDATORS];
} sim_test_config_t;

typedef struct {
    char firmware[256];
    int  id;         /* 0 = auto-assign */
    double x, y;     /* position in meters */
    int  has_position; /* true if x,y specified in JSON */
} sim_node_config_t;

typedef struct {
    char title[128];
    int  timeout_ms;   /* default 20000 */
    int  seed;         /* 0 = not set */
    int  node_count;
    sim_node_config_t nodes[MAX_SIM_NODES];

    /* Radio medium (0=NONE, 1=UDGM) */
    int    medium_type;
    double tx_range;             /* default 50.0 */
    double interference_range;   /* default 100.0 */
    double success_ratio_tx;     /* default 1.0 */
    double success_ratio_rx;     /* default 1.0 */

    /* Startup delay spread: each node gets a random delay in [0, startup_delay_ms) */
    int startup_delay_ms;   /* 0 = no stagger (default) */

    /* Simulation speed multiplier for real-time pacing (UI mode) */
    double speed;           /* 0 = default (10x), e.g. 1.0 = real-time, 100.0 = 100x */

    /* Test scripting */
    int has_test;
    sim_test_config_t test;
} sim_config_t;

/* Load a JSON config file. Returns 0 on success, -1 on error. */
int  sim_config_load(sim_config_t *cfg, const char *json_path);

/* Print config summary to stdout. */
void sim_config_print(const sim_config_t *cfg);

#endif /* SIM_CONFIG_H */
