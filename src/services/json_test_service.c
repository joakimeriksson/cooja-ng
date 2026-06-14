/*
 * Implementation of the JSON test-runner service — see
 * include/sim/json_test_service.h.
 *
 * Phase 6 milestone 35 (§3.19).  Verbatim moves of the runner's
 * sim_test_check_line, the per-step timeout block, and the end-of-run
 * finalize + validator check + "--- Test Results ---" print, retargeted
 * from the file-scope `active_test` onto the service struct.
 */
#include "json_test_service.h"
#include "sim_runtime.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>

#define JT_MS_TO_NS 1000000LL

void json_test_service_start(json_test_service_t *svc,
                             const sim_test_config_t *cfg) {
    memset(svc, 0, sizeof(*svc));
    svc->config = cfg;
    svc->active = (cfg != NULL);
}

/* Step/validator/fail_on checker — verbatim from sim_test_check_line. */
static void json_test_check_line(json_test_service_t *t, int node_id,
                                 const char *line, int64_t sim_ns) {
    if (!t->active || t->finished)
        return;

    /* Check fail_on patterns first */
    for (int i = 0; i < t->config->fail_on_count; i++) {
        if (strstr(line, t->config->fail_on[i])) {
            t->finished = -1;
            snprintf(t->fail_reason, sizeof(t->fail_reason),
                     "fail_on pattern \"%s\" matched at node %d, %lld ms: %s",
                     t->config->fail_on[i], node_id,
                     (long long)(sim_ns / JT_MS_TO_NS), line);
            return;
        }
    }

    /* Update validator counters (always, regardless of steps) */
    for (int i = 0; i < t->config->validator_count; i++) {
        const sim_test_validator_t *v = &t->config->validators[i];
        if (v->node >= 0 && v->node != node_id)
            continue;
        if (strstr(line, v->pattern))
            t->validator_counts[i]++;
    }

    /* If no steps, nothing more to check (fail_on-only / timeout_is_success) */
    if (t->config->step_count == 0)
        return;

    const sim_test_step_t *step = &t->config->steps[t->current_step];

    /* Node filter: -1 = any */
    if (step->node >= 0 && step->node != node_id)
        return;

    /* Substring match */
    if (!strstr(line, step->pattern))
        return;

    t->match_count++;
    if (t->match_count >= step->count) {
        printf("  TEST step %d PASS: \"%s\" matched %d/%d (node %d, %lld ms)\n",
               t->current_step, step->pattern,
               t->match_count, step->count, node_id,
               (long long)(sim_ns / JT_MS_TO_NS));
        t->current_step++;
        t->match_count = 0;
        t->step_start_ns = sim_ns;

        if (t->current_step >= t->config->step_count)
            t->finished = 1;  /* all steps passed */
    }
}

void json_test_check_step_timeout(json_test_service_t *t, int64_t now_ns,
                                  int sim_ms) {
    if (t->active && !t->finished && t->config->step_count > 0) {
        const sim_test_step_t *step = &t->config->steps[t->current_step];
        int step_timeout = step->timeout_ms > 0 ? step->timeout_ms : sim_ms;
        int64_t elapsed_step_ns = now_ns - t->step_start_ns;
        if (elapsed_step_ns >= (int64_t)step_timeout * JT_MS_TO_NS) {
            t->finished = -1;
            snprintf(t->fail_reason, sizeof(t->fail_reason),
                     "step %d timed out after %d ms waiting for \"%s\" "
                     "(matched %d/%d)",
                     t->current_step, step_timeout,
                     step->pattern, t->match_count, step->count);
        }
    }
}

int json_test_report(json_test_service_t *t, int64_t now_ns) {
    if (!t->active) return 0;
    int test_exit_code = 0;

    /* If test is still running when simulation ends */
    if (!t->finished) {
        if (t->config->timeout_is_success) {
            /* timeout_is_success: reaching timeout without failure = PASS */
            t->finished = 1;
        } else if (t->config->step_count == 0) {
            /* No steps and no failure hit — pass (fail_on-only test) */
            t->finished = 1;
        } else {
            t->finished = -1;
            const sim_test_step_t *step = &t->config->steps[t->current_step];
            snprintf(t->fail_reason, sizeof(t->fail_reason),
                     "simulation ended at %lld ms, step %d waiting for \"%s\" "
                     "(matched %d/%d)",
                     (long long)(now_ns / JT_MS_TO_NS), t->current_step,
                     step->pattern, t->match_count, step->count);
        }
    }
    /* Check validators at end of test (when test would otherwise pass) */
    if (t->finished == 1 && t->config->validator_count > 0) {
        for (int i = 0; i < t->config->validator_count; i++) {
            if (t->validator_counts[i] <
                t->config->validators[i].min_count) {
                t->finished = -1;
                snprintf(t->fail_reason, sizeof(t->fail_reason),
                         "validator \"%s\" matched %d/%d times",
                         t->config->validators[i].pattern,
                         t->validator_counts[i],
                         t->config->validators[i].min_count);
                break;
            }
        }
    }

    printf("\n--- Test Results ---\n");
    for (int i = 0; i < t->config->step_count; i++) {
        const sim_test_step_t *s = &t->config->steps[i];
        const char *status;
        if (i < t->current_step)
            status = "PASS";
        else if (i == t->current_step && t->finished == -1)
            status = "FAIL";
        else
            status = "SKIP";
        printf("  Step %d [%s]: wait \"%s\"", i, status, s->pattern);
        if (s->node >= 0)
            printf(" node=%d", s->node);
        if (s->count > 1)
            printf(" count=%d", s->count);
        printf("\n");
    }
    /* Print validator results */
    for (int i = 0; i < t->config->validator_count; i++) {
        const sim_test_validator_t *v = &t->config->validators[i];
        int count = t->validator_counts[i];
        const char *vstat = (count >= v->min_count) ? "PASS" : "FAIL";
        printf("  Validator [%s]: \"%s\" matched %d/%d",
               vstat, v->pattern, count, v->min_count);
        if (v->node >= 0)
            printf(" node=%d", v->node);
        printf("\n");
    }
    if (t->finished == 1) {
        printf("\n  TEST PASSED (%lld ms simulated)\n",
               (long long)(now_ns / JT_MS_TO_NS));
    } else {
        printf("\n  TEST FAILED: %s\n", t->fail_reason);
        test_exit_code = 1;
    }
    t->active = false;
    return test_exit_code;
}

static int json_test_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim;
    *state = (void *)cfg;   /* runner-owned json_test_service_t */
    return 0;
}

static void json_test_on_event(sim_runtime_t *sim, void *state,
                               const sim_observer_event_t *ev) {
    (void)sim;
    if (ev->kind != SIM_OBS_MOTE_LOG_LINE) return;
    json_test_service_t *t = (json_test_service_t *)state;
    if (t->active)
        json_test_check_line(t, ev->u.log_line.node_id,
                             ev->u.log_line.line, ev->time_ns);
}

const sim_service_ops_t json_test_service_ops = {
    .name     = "json-test",
    .init     = json_test_init,
    .on_event = json_test_on_event,
};
