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

#define MAX_SIM_NODES 128
#define MAX_TEST_STEPS 32

typedef struct {
    char pattern[256];
    int  node;          /* -1 = any */
    int  count;         /* default 1 */
    int  timeout_ms;    /* 0 = use global */
} sim_test_step_t;

typedef struct {
    int step_count;
    sim_test_step_t steps[MAX_TEST_STEPS];
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

    /* Test scripting */
    int has_test;
    sim_test_config_t test;
} sim_config_t;

/* Load a JSON config file. Returns 0 on success, -1 on error. */
int  sim_config_load(sim_config_t *cfg, const char *json_path);

/* Print config summary to stdout. */
void sim_config_print(const sim_config_t *cfg);

#endif /* SIM_CONFIG_H */
