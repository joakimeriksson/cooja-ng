/*
 * Implementation of the progress-report service — see
 * include/sim/progress_service.h.
 *
 * Phase 6 milestone 34 (§3.19).  Verbatim move of the runner's per-tick
 * progress block; the per-node enumeration goes through the runner-supplied
 * describe callback instead of a direct nodes[]/mote-ops read.
 */
#include "progress_service.h"
#include "sim_runtime.h"

#include <stdio.h>
#include <stddef.h>

#define PROGRESS_MS_TO_NS 1000000LL

void progress_service_start(progress_service_t *svc, int64_t start_ns,
                            int64_t total_ns, int64_t end_ns,
                            const int *node_count,
                            progress_describe_fn describe) {
    svc->total_ns    = total_ns;
    svc->end_ns      = end_ns;
    svc->interval_ns = total_ns / 10;
    svc->next_ns     = start_ns + svc->interval_ns;
    svc->node_count  = node_count;
    svc->describe    = describe;
}

void progress_service_tick(progress_service_t *svc, int64_t now_ns,
                           int rf_bytes, int uart_bytes) {
    if (now_ns < svc->next_ns) return;
    int pct = (int)((now_ns - (svc->end_ns - svc->total_ns)) * 100 / svc->total_ns);
    printf("  --- Progress: %d%% (%lld ms) rf_bytes=%d uart_bytes=%d ---\n",
           pct, (long long)(now_ns / PROGRESS_MS_TO_NS), rf_bytes, uart_bytes);
    int n = svc->node_count ? *svc->node_count : 0;
    for (int i = 0; i < n; i++) {
        int node_id = 0; const char *type = ""; int64_t cycles = 0; uint32_t freq = 0;
        if (svc->describe) svc->describe(i, &node_id, &type, &cycles, &freq);
        printf("    Node %d [%s]: %lld cycles, freq=%u Hz\n",
               node_id, type, (long long)cycles, freq);
    }
    svc->next_ns += svc->interval_ns;
}

static int progress_service_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim;
    *state = (void *)cfg;
    return 0;
}

const sim_service_ops_t progress_service_ops = {
    .name = "progress",
    .init = progress_service_init,
};
