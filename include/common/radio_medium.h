/*
 * UDGM (Unit Disk Graph Medium) radio medium model
 *
 * Filters RF byte delivery between nodes based on:
 *   - Distance (tx_range / interference_range)
 *   - Per-radio spectrum + channel matching
 *   - Probabilistic packet loss (per-frame, not per-byte)
 *
 * Per-node multi-radio model
 * --------------------------
 * Each node owns up to RADIO_MEDIUM_MAX_RADIOS_PER_NODE radio_t slots,
 * each carrying:
 *   - spectrum    — band identity (2.4 GHz 802.15.4, 868 MHz 15.4g, ...)
 *   - channel     — band-local channel index (-1 = unknown)
 *   - rx_enabled  — whether the chip is currently in RX
 *
 * For two radios to deliver bytes to each other the medium requires
 * matching spectrum AND matching channel (with the legacy "if either
 * channel is -1, allow" semantic preserved).
 *
 * Backward compatibility: the legacy `radio_medium_set_channel(rm, n, ch)`
 * API forwards to radio slot 0 (auto-registering it on the legacy
 * 2.4 GHz spectrum if not already registered, or 868 MHz if `ch >=
 * RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE`). All single-radio platforms (sky,
 * cc2538dk, openmote, esb, z1, wismote, exp5438, cc430) continue to
 * work without modification through this single-radio API path.
 */
#ifndef RADIO_MEDIUM_H
#define RADIO_MEDIUM_H

#include <stdint.h>
#include <stdbool.h>

#define RADIO_MEDIUM_MAX_NODES 128

/* How many radios may live on one node. Firefly = 2 (cc2538_rfcore +
 * CC1200). Most platforms use 1. */
#define RADIO_MEDIUM_MAX_RADIOS_PER_NODE 2

/* Sub-GHz channel range threshold. Channels >= this value sent through
 * the legacy single-radio API are interpreted as 802.15.4g (CC1200) and
 * land in the sub-GHz spectrum. Channels below stay 2.4 GHz IEEE
 * 802.15.4. The new multi-radio API skips this remapping — callers pass
 * a real band-local channel index. */
#define RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE  100

typedef enum {
    RADIO_MEDIUM_NONE = 0,   /* all-to-all, no filtering */
    RADIO_MEDIUM_UDGM = 1,   /* Unit Disk Graph Medium */
} radio_medium_type_t;

/* Frame format profile — picked per-sender from the sender's currently
 * tuned spectrum. Keeps the on-air SFD / sync-word definition next to
 * the radio that uses it. */
typedef enum {
    /* IEEE 802.15.4 OQPSK (CC2420 / CC2538): 4×0x00 preamble + 0x7A SFD. */
    RADIO_FRAME_PROFILE_IEEE802154 = 0,
    /* IEEE 802.15.4g MR-FSK (CC1200): variable-length preamble + 32-bit
     * sync word + 2-byte PHR + payload. Sync word is configurable on
     * the chip; the medium uses the Contiki 50 kbps default. */
    RADIO_FRAME_PROFILE_IEEE802154G = 1,
} radio_frame_profile_t;

/* Spectrum / band identity for a radio. Each chip type maps to one
 * spectrum at platform init time. The ordering below is stable so test
 * code can reference values by name. */
typedef enum {
    RADIO_SPECTRUM_NONE         = 0,  /* unregistered slot */
    RADIO_SPECTRUM_2_4GHZ_15_4  = 1,  /* CC2420 / cc2538_rfcore — channels 11-26 */
    RADIO_SPECTRUM_868MHZ_15_4G = 2,  /* CC1200 EU sub-GHz */
    RADIO_SPECTRUM_915MHZ_15_4G = 3,  /* CC1200 NA sub-GHz */
} radio_spectrum_t;

/* The CC1200 50 kbps Contiki config sets SYNC3..SYNC0 = 0x6E,0x4E,0x90,0x4E
 * (see arch/dev/radio/cc1200/cc1200-802154g-863-870-fsk-50kbps.c). */
#define RADIO_FRAME_802154G_SYNC_WORD  0x6E4E904Eu

typedef struct {
    double tx_range;            /* meters, default 50.0 */
    double interference_range;  /* meters, default 100.0 */
    double success_ratio_tx;    /* [0,1], default 1.0 */
    double success_ratio_rx;    /* [0,1], default 1.0 */
} udgm_config_t;

/* Per-radio slot.
 *
 * The medium does NOT hold a chip pointer or callback table — that
 * would couple the medium to chip drivers. The harness is responsible
 * for dispatching delivered bytes to the right chip instance based on
 * (node, radio_idx). */
typedef struct {
    radio_spectrum_t spectrum;     /* RADIO_SPECTRUM_NONE = unregistered */
    int              channel;      /* band-local channel index, -1 = unknown */
    bool             rx_enabled;   /* chip currently in RX */
} radio_t;

typedef struct {
    double  x, y;                                      /* shared by all radios */
    radio_t radios[RADIO_MEDIUM_MAX_RADIOS_PER_NODE];
    int     radio_count;                                /* 0 = uninitialized */
    /* Legacy alias: forwards to radios[0].channel. The implementation
     * keeps this in lockstep with radios[0].channel so existing readers
     * (test_mixed_multinode CCA path) keep working until they migrate. */
    int     channel;
} radio_node_state_t;

/* Frame tracker state machine for detecting frame boundaries in byte stream.
 * Profile-aware: the IEEE 802.15.4 path detects SFD = 0x7A after 4×0x00
 * preamble; the 802.15.4g path detects a 32-bit sync word and consumes a
 * 2-byte PHR for length (CC1200). */
typedef enum {
    FRAME_IDLE = 0,
    FRAME_PREAMBLE,
    FRAME_SFD,        /* IEEE 802.15.4: just past SFD, expecting length byte */
    FRAME_PHR_LO,     /* 802.15.4g: just past sync word, expecting PHR low byte */
    FRAME_DATA,
} frame_track_state_t;

typedef struct {
    frame_track_state_t state;
    int zero_count;     /* consecutive 0x00 preamble bytes (802.15.4) */
    uint32_t sync_match;/* sliding 32-bit register for 802.15.4g sync detect */
    int length;         /* frame length from PHY header */
    int byte_count;     /* bytes received in current frame */
    uint32_t frame_id;  /* unique ID for current frame */
    int phr_hi;         /* 802.15.4g: top 3 bits of 11-bit length */
    radio_frame_profile_t profile;  /* updated when set_channel changes band */
    /* Sender's channel at the moment this frame started, snapshotted at
     * SFD detection. The per-byte channel filter compares the receiver's
     * current channel to this snapshot, not to the sender's current
     * channel — matches Cooja's createConnections() semantic and real
     * hardware (chip is committed to TX channel for the frame duration). */
    int frame_start_channel;
} frame_tracker_t;

/* Per-(sender,receiver) frame decision cache */
typedef struct {
    uint32_t frame_id;
    bool decided;       /* true if we've rolled the dice for this frame */
    bool drop;          /* true if this frame should be dropped */
    /* Channel-match decision frozen at the first byte we processed for
     * this (sender, receiver) pair for the current frame_id. Mirrors
     * Cooja's RadioConnection destination set: once the receiver's
     * channel matched the sender's channel at frame start, we keep
     * delivering even if either side retunes mid-frame. */
    bool channel_decided;
    bool channel_match;
} rx_decision_t;

/* Precomputed neighbor list per node */
typedef struct {
    int neighbors[RADIO_MEDIUM_MAX_NODES];
    int count;
} neighbor_list_t;

/* The pluggable medium policy vtable (Phase 11 §3.24) — forward-declared here
 * so radio_medium_t can hold a pointer; fully defined after the struct. */
typedef struct sim_medium_ops sim_medium_ops_t;

typedef struct {
    radio_medium_type_t type;
    udgm_config_t       udgm;
    int                 node_count;
    radio_node_state_t  nodes[RADIO_MEDIUM_MAX_NODES];
    /* Frame trackers and rx-decisions are keyed on (sender_node,
     * sender_radio_idx). Each sender radio carries its own framing state. */
    frame_tracker_t     frame_track[RADIO_MEDIUM_MAX_NODES][RADIO_MEDIUM_MAX_RADIOS_PER_NODE];
    rx_decision_t       rx_decisions[RADIO_MEDIUM_MAX_NODES][RADIO_MEDIUM_MAX_NODES];
    neighbor_list_t     neighbors[RADIO_MEDIUM_MAX_NODES];          /* TX-range */
    neighbor_list_t     interference_neighbors[RADIO_MEDIUM_MAX_NODES];
    uint32_t            next_frame_id;
    uint32_t            rng_state;
    /* Policy vtable (Phase 11).  Set by radio_medium_init (= &csim_none_ops)
     * and radio_medium_configure_udgm (= &csim_udgm_ops); a plugin medium
     * installs its own.  Appended at the end so existing field offsets and
     * the memset-zero init are unchanged. */
    const sim_medium_ops_t *ops;
} radio_medium_t;

/* Pluggable medium policy (Phase 11 §3.24).  A medium overrides only the
 * policy — the generic frame tracking / spectrum+channel matching / RNG dice
 * roll / neighbor-list bookkeeping stay in radio_medium.c.  `reception_prob`
 * returns a clamped [0,1] and MUST NOT call the RNG (the generic code owns the
 * roll, so the draw sequence stays identical across media). */
struct sim_medium_ops {
    const char *name;
    double (*reception_prob)(const radio_medium_t *rm, int sender, int receiver);
    int8_t (*get_rssi)(const radio_medium_t *rm, int sender, int receiver);
    void   (*compute_neighbors)(radio_medium_t *rm);
};

/* Built-in media (radio_medium.c).  csim_udgm_ops points at the existing UDGM
 * function bodies; csim_none_ops is the all-to-all bypass (reception_prob /
 * get_rssi are never dispatched — the NONE early-returns handle them). */
extern const sim_medium_ops_t csim_udgm_ops;
extern const sim_medium_ops_t csim_none_ops;

/* --- Initialization --- */

/* Initialize radio medium (defaults to NONE type) */
void radio_medium_init(radio_medium_t *rm, int node_count);

/* Configure as UDGM with given parameters */
void radio_medium_configure_udgm(radio_medium_t *rm, double tx_range,
    double interference_range, double success_ratio_tx, double success_ratio_rx);

/* Set node position in meters */
void radio_medium_set_position(radio_medium_t *rm, int node, double x, double y);

/* Set PRNG seed for reproducible simulations */
void radio_medium_set_seed(radio_medium_t *rm, uint32_t seed);

/* --- Per-radio API (preferred) --- */

/* Register a radio slot on a node with its band identity. May be called
 * any time before traffic begins on that radio. Idempotent — repeating
 * the same (node, idx, spectrum) is safe; changing the spectrum resets
 * the radio's frame tracker. */
void radio_medium_register_radio(radio_medium_t *rm, int node, int radio_idx,
                                  radio_spectrum_t spectrum);

/* Set the band-local channel index for a specific radio slot. -1 marks
 * "unknown" — pairs with -1 still pass through the channel filter to
 * keep legacy multi-platform setups (where channel was never reported)
 * functional. */
void radio_medium_set_radio_channel(radio_medium_t *rm, int node, int radio_idx,
                                     int channel);

/* Mark a radio as RX-enabled (chip is in RX state) so the medium can
 * gate delivery on it. Defaults to true for compatibility — chip
 * drivers that don't push this state get the legacy "always receiving"
 * behavior. */
void radio_medium_set_radio_rx_enabled(radio_medium_t *rm, int node, int radio_idx,
                                        bool on);

/* --- Legacy single-radio API (forwards to radio_idx == 0) --- */

/* Set node channel via the legacy single-radio API. Auto-registers
 * radio slot 0 with the appropriate spectrum if needed: channels >=
 * RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE land on RADIO_SPECTRUM_868MHZ_15_4G,
 * everything else on RADIO_SPECTRUM_2_4GHZ_15_4. */
void radio_medium_set_channel(radio_medium_t *rm, int node, int channel);

/* --- Filter functions --- */

/*
 * Filter a byte from sender (radio 0) to receiver (radio 0).
 * Returns true if the byte should be delivered, false if dropped.
 *
 * Tracks frame boundaries (preamble/SFD/length) to make per-frame
 * probabilistic decisions rather than per-byte.
 */
bool radio_medium_filter_byte(radio_medium_t *rm, int sender, int receiver, uint8_t byte);

/*
 * Per-radio variant of filter_byte. Pass explicit (sender, sender_radio,
 * receiver, receiver_radio) to deliver across multi-radio nodes. The
 * sender_radio's frame tracker is advanced; the receiver_radio's
 * spectrum + channel + rx_enabled state are checked against the sender.
 */
bool radio_medium_filter_byte_radio(radio_medium_t *rm,
    int sender, int sender_radio, int receiver, int receiver_radio, uint8_t byte);

/*
 * Frame-level filter: decides whether a frame from sender (radio 0)
 * should be delivered to receiver (radio 0). Uses probabilistic loss
 * (one roll per call). Returns true if the frame should be delivered.
 */
bool radio_medium_filter_frame(radio_medium_t *rm, int sender, int receiver);

/*
 * Per-radio variant of filter_frame.
 */
bool radio_medium_filter_frame_radio(radio_medium_t *rm,
    int sender, int sender_radio, int receiver, int receiver_radio);

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
