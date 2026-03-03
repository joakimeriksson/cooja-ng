/*
 * Simulation state serializer for WebSocket UI
 *
 * Converts current simulation state to JSON for browser consumption.
 * Uses cJSON library. Caller must free the returned string.
 */
#ifndef SIM_STATE_H
#define SIM_STATE_H

#include <stdint.h>

/* Per-node info passed to serializer */
typedef struct {
    int id;
    const char *type;       /* "MSP430", "ARM", "NATIVE" */
    double x, y;            /* position in meters */
    int64_t cycles;
    uint32_t freq_hz;
    int64_t sim_time_ns;
    const char **console;   /* array of recent console lines */
    int console_count;
    int64_t last_tx_ns;     /* sim time of last RF TX (0 = never) */
} sim_node_info_t;

/* Radio medium info */
typedef struct {
    const char *type;       /* "NONE" or "UDGM" */
    double tx_range;
    int node_count;
    /* Neighbor adjacency: neighbors[i] is array of neighbor indices for node i */
    const int *const *neighbors;
    const int *neighbor_counts;
} sim_radio_info_t;

/* Global simulation stats */
typedef struct {
    int64_t sim_time_ns;
    int rf_bytes;
    int uart_bytes;
    int rx_frames_queued;
    int rx_frames_collided;
} sim_stats_t;

/*
 * Serialize simulation state to JSON string.
 * Caller must free() the returned string.
 * Returns NULL on failure.
 */
char *sim_state_to_json(const sim_node_info_t *nodes, int node_count,
                        const sim_radio_info_t *radio,
                        const sim_stats_t *stats);

#endif /* SIM_STATE_H */
