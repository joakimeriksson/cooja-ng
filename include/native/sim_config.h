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

#define MAX_SIM_NODES 16

typedef struct {
    char firmware[256];
    int  id;         /* 0 = auto-assign */
} sim_node_config_t;

typedef struct {
    char title[128];
    int  timeout_ms;   /* default 20000 */
    int  seed;         /* 0 = not set */
    int  node_count;
    sim_node_config_t nodes[MAX_SIM_NODES];
} sim_config_t;

/* Load a JSON config file. Returns 0 on success, -1 on error. */
int  sim_config_load(sim_config_t *cfg, const char *json_path);

/* Print config summary to stdout. */
void sim_config_print(const sim_config_t *cfg);

#endif /* SIM_CONFIG_H */
