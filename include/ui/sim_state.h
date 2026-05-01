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
    double speed_ratio;     /* current speed multiplier (e.g. 10.0 = 10x real-time) */
    int paused;             /* 1 if simulation is paused */
} sim_stats_t;

/* Per-node radio state for timeline (Cooja Classic semantics) */
typedef enum {
    SIM_RADIO_OFF  = 0,   /* Transceiver off (white) */
    SIM_RADIO_ON   = 1,   /* On, listening (gray) */
    SIM_RADIO_TX   = 2,   /* Transmitting (blue) */
    SIM_RADIO_RX   = 3,   /* Receiving data (green) */
    SIM_RADIO_INTF = 4,   /* Interference (red) */
} sim_radio_state_t;

/* Per-node LED state */
#define SIM_MAX_LEDS 3

typedef struct {
    sim_radio_state_t radio_state;
    uint8_t led[SIM_MAX_LEDS];     /* 1=on, 0=off per LED */
    /* TODO(dual-radio): one `radio_state` per node assumes one radio.
     * For dual-radio, replace with `radio_state[NUM_RADIOS]` and update
     * the CBOR delta protocol below — the UI will need to render two
     * timeline rows per node and the "radio changes" map (key 2) will
     * need to encode (node_id, radio_idx) pairs. */
} sim_node_state_t;

/*
 * Serialize full simulation state to JSON string (type: "full").
 * Includes node positions, topology, all console lines, all state.
 * Used on initial client connect and explicit refresh requests.
 * Caller must free() the returned string.
 */
char *sim_state_to_json(const sim_node_info_t *nodes, int node_count,
                        const sim_radio_info_t *radio,
                        const sim_stats_t *stats,
                        const sim_node_state_t *node_states,
                        const char *timeline_json);

/*
 * Serialize delta state to JSON string (type: "delta").
 * Only includes changing data: stats, per-node cycles/freq/radio/leds,
 * new console lines (sparse), and new timeline events.
 * Caller must free() the returned string.
 */
char *sim_state_delta_json(const sim_stats_t *stats,
                           const sim_node_state_t *node_states,
                           int node_count,
                           const int *node_ids,
                           const int64_t *node_cycles,
                           const uint32_t *node_freqs,
                           const int64_t *node_last_tx_ns,
                           const char ***console_lines,
                           const int *console_counts,
                           const char *timeline_json);

/*
 * Serialize delta state to CBOR binary (sent as WebSocket binary frame).
 * Only includes fields that changed since prev_* state.
 * Returns size written to buf, or 0 on error.
 *
 * CBOR map with integer keys:
 *   0: sim_time_ms (unsigned)
 *   1: stats array [rf_bytes, uart_bytes, rx_queued, rx_collided, speed_x10, paused]
 *   2: radio changes map { node_id: state_enum }
 *   3: LED changes map { node_id: [r,g,b] }
 *   4: last_tx_ms map { node_id: ms }
 *   5: console map { node_id: [lines...] }
 *   6: timeline array of arrays [[t_ms*1000, node_id, type, value, summary?], ...]
 */
int sim_state_delta_cbor(uint8_t *buf, int buf_size,
                         const sim_stats_t *stats,
                         const sim_node_state_t *node_states,
                         const sim_node_state_t *prev_node_states,
                         int node_count,
                         const int *node_ids,
                         const int64_t *node_last_tx_ns,
                         const int64_t *prev_last_tx_ns,
                         const char ***console_lines,
                         const int *console_counts,
                         const uint8_t *tl_cbor,
                         int tl_cbor_len);

/*
 * Serialize new timeline events to CBOR.
 * Returns size written, or 0 on error.
 */
struct timeline_s;
int tl_events_to_cbor(struct timeline_s *tl, uint8_t *buf, int buf_size);

#endif /* SIM_STATE_H */
