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
    int     mote_type;  /* mote type index (for generateMote) */
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
    double clock_deviation; /* 1.0 = normal, <1.0 = slower (Cooja MspClock deviation) */
    char type_name[64]; /* v2: the named mote-type this node uses ("" for v1) */
} sim_node_config_t;

/* v2 named mote type (config v2 §8.2).  In Phase 7 only `firmware` drives
 * the runtime (board resolved from its extension as in v1); name/kind/cpu/
 * soc/board are captured for the Phase 8 static registry. */
typedef struct {
    char name[64];
    char kind[32];      /* "emulated-elf" | "native-cooja" | "js-app" */
    char cpu[32];       /* "msp430" | "armv7m" | ...                  */
    char soc[32];       /* "msp430f1611" | "cc2538" | ...             */
    char board[32];     /* "sky" | "zoul-firefly" | "cc2538dk" | ...  */
    char firmware[256];
} sim_mote_type_t;

typedef struct {
    int  version;      /* config schema version: 1 (legacy) or 2 (M41+) */
    char title[128];
    int  timeout_ms;   /* default 20000 */
    int  seed;         /* 0 = not set */
    int  node_count;
    sim_node_config_t nodes[MAX_SIM_NODES];

    /* Radio medium (0=NONE, 1=UDGM).  medium_name is the config "type" string
     * verbatim (Phase 11) — built-ins resolve via medium_type, a custom name
     * (a registered/plugin medium) resolves through the registry post-load. */
    int    medium_type;
    char   medium_name[32];
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

    /* JavaScript test script (alternative to JSON test steps) */
    int has_js_script;
    char js_script_path[512];       /* path to .js file */
    char *js_script_inline;         /* inline script (malloc'd, or NULL) */

    /* Mote types (for getMoteTypes()[index] mapping).  The legacy
     * index-keyed firmware table is read by TEST_ACTION_ADD at runtime; the
     * v2 parser writes both it and the richer mote_types[] table below. */
    int mote_type_count;
    char mote_type_firmware[8][256]; /* firmware path per type index */
    sim_mote_type_t mote_types[8];   /* v2 named mote types (M42)       */

    /* v2 plugin/service names (captured for Phase 8 register-by-name;
     * inert in Phase 7).  e.g. "builtin:timeline", "builtin:pcap". */
    int  plugin_count;
    char plugins[16][48];

    /* Serial socket server (for border-router tests) */
    int  has_serial_socket;
    int  serial_socket_port;        /* TCP port (default 60001) */
    int  serial_socket_node;        /* node ID to bridge serial to */
    char serial_socket_command[1024]; /* external command to run after socket ready */
} sim_normalized_config_t;

/* The single normalized config struct both the v1 and v2 JSON parsers
 * populate; the runtime consumes only this, never the raw JSON layout
 * (§8.3).  Phase 7 renamed it from sim_config_t. */

/* Load a JSON config file (v1 or v2, dispatched on the top-level "version"
 * key).  Returns 0 on success, -1 on error. */
int  sim_config_load(sim_normalized_config_t *cfg, const char *json_path);

/* Print config summary to stdout. */
void sim_config_print(const sim_normalized_config_t *cfg);

#endif /* SIM_CONFIG_H */
