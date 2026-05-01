/*
 * UDGM (Unit Disk Graph Medium) radio medium model
 *
 * Filters RF byte delivery between nodes based on:
 *   - Distance (tx_range / interference_range)
 *   - Channel matching
 *   - Probabilistic packet loss (per-frame, not per-byte)
 *
 * When type is RADIO_MEDIUM_NONE (default), all bytes pass through
 * unchanged — backward compatible with existing all-to-all behavior.
 *
 * TODO(dual-radio): every API and table here keys on a single node
 * index. For nodes with two radios (e.g. CC2538 RF Core + external
 * CC2420), the keys must become (node_idx, radio_idx) — `nodes[]`,
 * `frame_track[]`, `rx_decisions[][]`, `neighbors[]`,
 * `filter_byte/frame()`, `set_channel()`, `get_rssi()`. Keep the
 * existing single-radio API as `radio_idx == 0`.
 */
#ifndef RADIO_MEDIUM_H
#define RADIO_MEDIUM_H

#include <stdint.h>
#include <stdbool.h>

#define RADIO_MEDIUM_MAX_NODES 128

typedef enum {
    RADIO_MEDIUM_NONE = 0,   /* all-to-all, no filtering */
    RADIO_MEDIUM_UDGM = 1,   /* Unit Disk Graph Medium */
} radio_medium_type_t;

typedef struct {
    double tx_range;            /* meters, default 50.0 */
    double interference_range;  /* meters, default 100.0 */
    double success_ratio_tx;    /* [0,1], default 1.0 */
    double success_ratio_rx;    /* [0,1], default 1.0 */
} udgm_config_t;

typedef struct {
    double x, y;   /* position in meters */
    int channel;   /* 802.15.4 channel (11-26), -1 = unknown */
    /* TODO(dual-radio): per-radio channel — second radio may sit on a
     * different channel, or even a different band entirely. */
} radio_node_state_t;

/* Frame tracker state machine for detecting frame boundaries in byte stream */
typedef enum {
    FRAME_IDLE = 0,
    FRAME_PREAMBLE,
    FRAME_SFD,
    FRAME_DATA,
} frame_track_state_t;

typedef struct {
    frame_track_state_t state;
    int zero_count;     /* consecutive 0x00 preamble bytes */
    int length;         /* frame length from PHY header */
    int byte_count;     /* bytes received in current frame */
    uint32_t frame_id;  /* unique ID for current frame */
} frame_tracker_t;

/* Per-(sender,receiver) frame decision cache */
typedef struct {
    uint32_t frame_id;
    bool decided;       /* true if we've rolled the dice for this frame */
    bool drop;          /* true if this frame should be dropped */
} rx_decision_t;

/* Precomputed neighbor list per node */
typedef struct {
    int neighbors[RADIO_MEDIUM_MAX_NODES];
    int count;
} neighbor_list_t;

typedef struct {
    radio_medium_type_t type;
    udgm_config_t       udgm;
    int                 node_count;
    radio_node_state_t  nodes[RADIO_MEDIUM_MAX_NODES];
    frame_tracker_t     frame_track[RADIO_MEDIUM_MAX_NODES];        /* per sender */
    rx_decision_t       rx_decisions[RADIO_MEDIUM_MAX_NODES][RADIO_MEDIUM_MAX_NODES]; /* [sender][receiver] */
    neighbor_list_t     neighbors[RADIO_MEDIUM_MAX_NODES];          /* precomputed TX-range */
    neighbor_list_t     interference_neighbors[RADIO_MEDIUM_MAX_NODES]; /* interference-only */
    uint32_t            next_frame_id;
    uint32_t            rng_state;
} radio_medium_t;

/* Initialize radio medium (defaults to NONE type) */
void radio_medium_init(radio_medium_t *rm, int node_count);

/* Configure as UDGM with given parameters */
void radio_medium_configure_udgm(radio_medium_t *rm, double tx_range,
    double interference_range, double success_ratio_tx, double success_ratio_rx);

/* Set node position in meters */
void radio_medium_set_position(radio_medium_t *rm, int node, double x, double y);

/* Set node channel (11-26 for 802.15.4, -1 for unknown) */
void radio_medium_set_channel(radio_medium_t *rm, int node, int channel);

/* Set PRNG seed for reproducible simulations */
void radio_medium_set_seed(radio_medium_t *rm, uint32_t seed);

/*
 * Filter a byte from sender to receiver.
 * Returns true if the byte should be delivered, false if dropped.
 *
 * Tracks frame boundaries (preamble/SFD/length) to make per-frame
 * probabilistic decisions rather than per-byte.
 */
bool radio_medium_filter_byte(radio_medium_t *rm, int sender, int receiver, uint8_t byte);

/*
 * Frame-level filter: decides whether a frame from sender should be
 * delivered to receiver. Uses probabilistic loss (one roll per call).
 * Returns true if the frame should be delivered.
 */
bool radio_medium_filter_frame(radio_medium_t *rm, int sender, int receiver);

/*
 * Recompute neighbor lists from current positions and tx_range.
 * Call after setting positions or changing tx_range.
 */
void radio_medium_compute_neighbors(radio_medium_t *rm);

/*
 * Compute RSSI (in dBm) for a frame from sender to receiver.
 * Linear-in-dB model: -10 dBm at distance 0, -90 dBm at tx_range.
 * Returns -50 dBm when radio medium is NONE (backward compatible).
 */
int8_t radio_medium_get_rssi(const radio_medium_t *rm, int sender, int receiver);

#endif /* RADIO_MEDIUM_H */
