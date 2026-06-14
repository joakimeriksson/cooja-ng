/*
 * json_test_service — the JSON config-driven test runner as a kernel
 * service (the step/validator/fail_on checker + end-of-run report).
 *
 * Phase 6 milestone 35 (§3.19).  Owns the per-run test state
 * (ex sim_test_state_t) and consumes SIM_OBS_MOTE_LOG_LINE through the
 * host fan-out (ex test_engine_observer's JSON half).  The runner drives
 * three position-pinned calls: the per-step timeout check and the
 * finished-query inside the loop, and the end-of-run report.
 *
 * What stays runner-side: the timed-action executor
 * (config.test.actions[] → MOVE/SEND/SEND_ALL/REMOVE/ADD).  That is
 * config-driven *node scripting* (it mutates nodes[], radio_medium, and
 * does dynamic node add), structurally shared with the JS engine's action
 * path (M36) and not test-engine state — it is not extracted here.
 *
 * The JS test engine (active_js_engine) is the separate M36 milestone; it
 * keeps its half of test_engine_observer until then.
 */
#ifndef JSON_TEST_SERVICE_H
#define JSON_TEST_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "sim_service.h"
#include "sim_config.h"   /* sim_test_config_t, sim_test_step_t, … */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct json_test_service {
    const sim_test_config_t *config;
    int     current_step;
    int     match_count;
    int64_t step_start_ns;
    int     finished;       /* 0=running, 1=pass, -1=fail */
    char    fail_reason[512];
    int     validator_counts[MAX_TEST_VALIDATORS];
    bool    active;         /* false = no JSON test this run               */
} json_test_service_t;

/* Activate (cfg != NULL) or leave inactive (cfg == NULL); zeroes the
 * per-run counters.  Does not print — the runner keeps the "Test: …"
 * banner at its original config-processing site. */
void json_test_service_start(json_test_service_t *svc,
                             const sim_test_config_t *cfg);

/* finished value: 0 running, 1 pass, -1 fail (0 when inactive). */
static inline int json_test_finished(const json_test_service_t *svc) {
    return svc->active ? svc->finished : 0;
}

/* Per-loop per-step timeout check.  `sim_ms` is the run duration used as
 * the default step timeout when a step sets none. */
void json_test_check_step_timeout(json_test_service_t *svc, int64_t now_ns,
                                  int sim_ms);

/* End-of-run: resolve a still-running test, check validators, print the
 * "--- Test Results ---" block, and return the process exit code (0 pass,
 * 1 fail; 0 when inactive).  Marks the service inactive. */
int  json_test_report(json_test_service_t *svc, int64_t now_ns);

/* Host vtable: init adopts the runner-owned service; on_event runs the
 * step/validator/fail_on checker on each SIM_OBS_MOTE_LOG_LINE. */
extern const sim_service_ops_t json_test_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* JSON_TEST_SERVICE_H */
