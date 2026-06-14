/*
 * progress_service — the periodic "--- Progress: N% ---" report as a
 * kernel service.
 *
 * Phase 6 milestone 34 (§3.19).  Owns the progress cadence state
 * (interval, next deadline, run horizon) that the runner used to keep as
 * loop-local variables, and prints the 10%-interval progress line plus the
 * per-node cycles/freq summary.
 *
 * The per-tick check is an explicit call (progress_service_tick) at the
 * runner's original loop position — not the host poll() — because the line
 * must interleave with the step's UART output exactly as before
 * (byte-identity).  The live RF/UART byte counters are passed in per tick;
 * the per-node id/type/cycles/freq come through a runner-supplied describe
 * callback so the service needs no access to nodes[] or the mote ops.
 *
 * End-of-run statistics (Performance / Phase-Timing / detailed counters)
 * are a separate, chip-coupled report and stay runner-side (M40 / §3.19).
 */
#ifndef PROGRESS_SERVICE_H
#define PROGRESS_SERVICE_H

#include <stdint.h>

#include "sim_service.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fill the display fields for node slot `i`.  `*type` receives a static
 * string (not owned by the service). */
typedef void (*progress_describe_fn)(int i, int *node_id, const char **type,
                                     int64_t *cycles, uint32_t *freq_hz);

typedef struct progress_service {
    int64_t total_ns;      /* total simulated span (for the % computation) */
    int64_t end_ns;        /* run horizon; start = end_ns - total_ns       */
    int64_t interval_ns;   /* print every total_ns/10                       */
    int64_t next_ns;       /* next print deadline                           */
    const int *node_count; /* runner's live node count                      */
    progress_describe_fn describe;
} progress_service_t;

/* Initialize the cadence (print every total_ns/10 starting one interval
 * after start_ns) and wire the node-describe callback. */
void progress_service_start(progress_service_t *svc, int64_t start_ns,
                            int64_t total_ns, int64_t end_ns,
                            const int *node_count, progress_describe_fn describe);

/* If `now_ns` has reached the next deadline, print the progress line and
 * the per-node summary, then advance the deadline. */
void progress_service_tick(progress_service_t *svc, int64_t now_ns,
                           int rf_bytes, int uart_bytes);

/* Host vtable: init adopts the already-started service (no poll/destroy —
 * the tick is explicit to preserve the print position). */
extern const sim_service_ops_t progress_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* PROGRESS_SERVICE_H */
