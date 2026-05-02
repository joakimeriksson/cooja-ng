/*
 * UDGM (Unit Disk Graph Medium) radio medium implementation
 */
#include "radio_medium.h"
#include <string.h>
#include <math.h>

/* IEEE 802.15.4 (2.4 GHz CC2420/CC2538) PHY constants */
#define SFD_BYTE        0x7A
#define MIN_PREAMBLE    4

/* IEEE 802.15.4g (CC1200 50 kbps) sync word — see radio_medium.h. */
#define SUBGHZ_SYNC_WORD  RADIO_FRAME_802154G_SYNC_WORD

/* --- Helpers --- */

static inline bool valid_node(const radio_medium_t *rm, int node) {
    (void)rm;
    return node >= 0 && node < RADIO_MEDIUM_MAX_NODES;
}

static inline bool valid_radio(int idx) {
    return idx >= 0 && idx < RADIO_MEDIUM_MAX_RADIOS_PER_NODE;
}

static inline radio_frame_profile_t profile_for_spectrum(radio_spectrum_t s) {
    switch (s) {
    case RADIO_SPECTRUM_868MHZ_15_4G:
    case RADIO_SPECTRUM_915MHZ_15_4G:
        return RADIO_FRAME_PROFILE_IEEE802154G;
    default:
        return RADIO_FRAME_PROFILE_IEEE802154;
    }
}

static inline radio_spectrum_t legacy_spectrum_for_channel(int ch) {
    return (ch >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE)
        ? RADIO_SPECTRUM_868MHZ_15_4G
        : RADIO_SPECTRUM_2_4GHZ_15_4;
}

/* Mark a radio's frame tracker as needing a profile reset. The actual
 * reset happens lazily when the tracker is next used; here we just stomp
 * the state so any in-flight pattern is dropped. */
static void reset_tracker(frame_tracker_t *ft, radio_frame_profile_t prof) {
    ft->profile = prof;
    ft->state = FRAME_IDLE;
    ft->zero_count = 0;
    ft->sync_match = 0;
}

/* --- xorshift32 PRNG --- */

static double rng_next(radio_medium_t *rm) {
    uint32_t x = rm->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rm->rng_state = x;
    return (double)(x & 0x7FFFFFFF) / (double)0x7FFFFFFF;
}

/* --- UDGM distance-based reception probability --- */

static double udgm_reception_prob(const radio_medium_t *rm, int sender, int receiver) {
    double dx = rm->nodes[sender].x - rm->nodes[receiver].x;
    double dy = rm->nodes[sender].y - rm->nodes[receiver].y;
    double dist_sq = dx * dx + dy * dy;
    double range = rm->udgm.tx_range;
    double range_sq = range * range;

    if (range_sq <= 0.0)
        return 0.0;
    if (dist_sq > range_sq)
        return 0.0;

    /* Cooja UDGM formula:
     * prob = (1 - dist²/range² × (1 - success_ratio_rx)) × success_ratio_tx */
    double ratio = dist_sq / range_sq;
    double prob = (1.0 - ratio * (1.0 - rm->udgm.success_ratio_rx)) * rm->udgm.success_ratio_tx;
    if (prob < 0.0) prob = 0.0;
    if (prob > 1.0) prob = 1.0;
    return prob;
}

/* --- Frame tracking state machine --- */

/* Mark a new frame: bump frame_id and invalidate per-receiver decisions
 * so each receiver gets a fresh dice roll for this frame. */
static void start_new_frame(radio_medium_t *rm, int sender, frame_tracker_t *ft) {
    ft->frame_id = ++rm->next_frame_id;
    for (int r = 0; r < rm->node_count; r++) {
        rm->rx_decisions[sender][r].decided = false;
        rm->rx_decisions[sender][r].frame_id = ft->frame_id;
    }
}

static void track_byte_802154(radio_medium_t *rm, int sender, uint8_t byte,
                               frame_tracker_t *ft) {
    switch (ft->state) {
    case FRAME_IDLE:
        if (byte == 0x00) {
            ft->zero_count = 1;
            ft->state = FRAME_PREAMBLE;
        }
        break;

    case FRAME_PREAMBLE:
        if (byte == 0x00) {
            ft->zero_count++;
        } else if (byte == SFD_BYTE && ft->zero_count >= MIN_PREAMBLE) {
            ft->state = FRAME_SFD;
            start_new_frame(rm, sender, ft);
        } else {
            ft->state = FRAME_IDLE;
            ft->zero_count = 0;
        }
        break;

    case FRAME_SFD:
        ft->length = byte;
        ft->byte_count = 0;
        ft->state = (ft->length > 0) ? FRAME_DATA : FRAME_IDLE;
        break;

    case FRAME_DATA:
        ft->byte_count++;
        if (ft->byte_count >= ft->length) {
            ft->state = FRAME_IDLE;
            ft->zero_count = 0;
        }
        break;

    default:
        ft->state = FRAME_IDLE;
        break;
    }
}

static void track_byte_802154g(radio_medium_t *rm, int sender, uint8_t byte,
                                frame_tracker_t *ft) {
    switch (ft->state) {
    case FRAME_IDLE:
    case FRAME_PREAMBLE:
        /* Slide the byte into the sync-word match register. */
        ft->sync_match = (ft->sync_match << 8) | byte;
        if (ft->sync_match == SUBGHZ_SYNC_WORD) {
            ft->state = FRAME_PHR_LO;
            start_new_frame(rm, sender, ft);
        } else {
            ft->state = FRAME_PREAMBLE;  /* keep sliding */
        }
        break;

    case FRAME_PHR_LO:
        /* PHR_HI byte: bits 2:0 = top 3 bits of 11-bit length */
        ft->phr_hi = byte & 0x07;
        ft->state = FRAME_SFD;  /* reuse SFD slot for "expecting PHR low" */
        break;

    case FRAME_SFD:
        /* PHR_LO byte: lower 8 bits of length */
        ft->length = (ft->phr_hi << 8) | byte;
        ft->byte_count = 0;
        ft->state = (ft->length > 0) ? FRAME_DATA : FRAME_IDLE;
        break;

    case FRAME_DATA:
        ft->byte_count++;
        if (ft->byte_count >= ft->length) {
            ft->state = FRAME_IDLE;
            ft->zero_count = 0;
            ft->sync_match = 0;
        }
        break;
    }
}

static void track_byte(radio_medium_t *rm, int sender, int sender_radio, uint8_t byte) {
    frame_tracker_t *ft = &rm->frame_track[sender][sender_radio];
    if (ft->profile == RADIO_FRAME_PROFILE_IEEE802154G) {
        track_byte_802154g(rm, sender, byte, ft);
    } else {
        track_byte_802154(rm, sender, byte, ft);
    }
}

/* --- Public API: init / configure --- */

void radio_medium_init(radio_medium_t *rm, int node_count) {
    memset(rm, 0, sizeof(*rm));
    rm->type = RADIO_MEDIUM_NONE;
    rm->node_count = node_count;
    rm->rng_state = 0x12345678;  /* default seed */
    rm->next_frame_id = 0;

    for (int i = 0; i < RADIO_MEDIUM_MAX_NODES; i++) {
        rm->nodes[i].channel = -1;
        for (int r = 0; r < RADIO_MEDIUM_MAX_RADIOS_PER_NODE; r++) {
            rm->nodes[i].radios[r].spectrum   = RADIO_SPECTRUM_NONE;
            rm->nodes[i].radios[r].channel    = -1;
            /* Default to RX-enabled so platforms that never push
             * rx_enabled state still deliver bytes. */
            rm->nodes[i].radios[r].rx_enabled = true;
        }
    }
}

void radio_medium_configure_udgm(radio_medium_t *rm, double tx_range,
    double interference_range, double success_ratio_tx, double success_ratio_rx)
{
    rm->type = RADIO_MEDIUM_UDGM;
    rm->udgm.tx_range = tx_range;
    rm->udgm.interference_range = interference_range;
    rm->udgm.success_ratio_tx = success_ratio_tx;
    rm->udgm.success_ratio_rx = success_ratio_rx;
}

void radio_medium_set_position(radio_medium_t *rm, int node, double x, double y) {
    if (valid_node(rm, node)) {
        rm->nodes[node].x = x;
        rm->nodes[node].y = y;
    }
}

void radio_medium_set_seed(radio_medium_t *rm, uint32_t seed) {
    rm->rng_state = seed ? seed : 0x12345678;
}

/* --- Per-radio API --- */

void radio_medium_register_radio(radio_medium_t *rm, int node, int radio_idx,
                                  radio_spectrum_t spectrum) {
    if (!valid_node(rm, node) || !valid_radio(radio_idx)) return;
    radio_t *r = &rm->nodes[node].radios[radio_idx];
    radio_spectrum_t prev = r->spectrum;
    r->spectrum = spectrum;
    if (radio_idx + 1 > rm->nodes[node].radio_count)
        rm->nodes[node].radio_count = radio_idx + 1;
    /* Reset the frame tracker on spectrum change (or first registration)
     * so we don't carry stale framing across band switches. */
    if (prev != spectrum) {
        reset_tracker(&rm->frame_track[node][radio_idx],
                      profile_for_spectrum(spectrum));
    }
}

void radio_medium_set_radio_channel(radio_medium_t *rm, int node, int radio_idx,
                                     int channel) {
    if (!valid_node(rm, node) || !valid_radio(radio_idx)) return;
    rm->nodes[node].radios[radio_idx].channel = channel;
    /* Keep the legacy alias in sync for callers that still read
     * rm->nodes[i].channel directly. The alias tracks radio 0. */
    if (radio_idx == 0) {
        rm->nodes[node].channel = channel;
    }
}

void radio_medium_set_radio_rx_enabled(radio_medium_t *rm, int node, int radio_idx,
                                        bool on) {
    if (!valid_node(rm, node) || !valid_radio(radio_idx)) return;
    rm->nodes[node].radios[radio_idx].rx_enabled = on;
}

/* --- Legacy single-radio API --- */

void radio_medium_set_channel(radio_medium_t *rm, int node, int channel) {
    if (!valid_node(rm, node)) return;
    /* Auto-register slot 0 with the spectrum implied by the channel
     * range (sub-GHz vs 2.4 GHz). The legacy path accepts -1 (unknown)
     * and leaves the spectrum at its current value in that case. */
    radio_t *r0 = &rm->nodes[node].radios[0];
    if (channel >= 0) {
        radio_spectrum_t want = legacy_spectrum_for_channel(channel);
        if (r0->spectrum != want) {
            radio_medium_register_radio(rm, node, 0, want);
        } else if (rm->nodes[node].radio_count == 0) {
            /* Slot 0 already had this spectrum but radio_count was
             * never bumped (init path). Make sure the count reflects
             * the live registration. */
            rm->nodes[node].radio_count = 1;
        }
    }
    radio_medium_set_radio_channel(rm, node, 0, channel);
}

/* --- Compute neighbors --- */

void radio_medium_compute_neighbors(radio_medium_t *rm) {
    double tx_range_sq = rm->udgm.tx_range * rm->udgm.tx_range;
    double int_range_sq = rm->udgm.interference_range * rm->udgm.interference_range;
    for (int i = 0; i < rm->node_count; i++) {
        rm->neighbors[i].count = 0;
        rm->interference_neighbors[i].count = 0;
        for (int j = 0; j < rm->node_count; j++) {
            if (i == j) continue;
            double dx = rm->nodes[i].x - rm->nodes[j].x;
            double dy = rm->nodes[i].y - rm->nodes[j].y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq <= tx_range_sq) {
                rm->neighbors[i].neighbors[rm->neighbors[i].count++] = j;
            } else if (dist_sq <= int_range_sq) {
                /* Within interference range but outside TX range */
                rm->interference_neighbors[i].neighbors[
                    rm->interference_neighbors[i].count++] = j;
            }
        }
    }
}

/* --- Per-radio match: spectrum + channel + rx_enabled --- */

/* Returns true if the (sender, sender_radio) -> (receiver, receiver_radio)
 * pair would deliver based on radio identity alone (band match, channel
 * match, receiver is in RX). The legacy "either channel is -1 -> allow"
 * semantic is preserved so platforms that never push channel state still
 * communicate. */
static bool radio_pair_match(const radio_medium_t *rm,
                              int sender, int sender_radio,
                              int receiver, int receiver_radio) {
    const radio_t *s = &rm->nodes[sender].radios[sender_radio];
    const radio_t *r = &rm->nodes[receiver].radios[receiver_radio];
    /* Spectrum gate.
     *
     * Both registered: must match — otherwise different bands.
     *
     * Sender registered, receiver unregistered: the receiver doesn't
     * have a radio in this slot at all — drop. This is what stops e.g.
     * a Firefly's CC1200 (slot 1, 868 MHz) from leaking onto a
     * cc2538dk's empty slot 1.
     *
     * Sender unregistered, receiver registered: same logic, drop.
     *
     * Both unregistered: legacy fallback — use the sub-GHz-channel-base
     * heuristic via the channel field. This keeps platforms that never
     * call register_radio (or auto-register via the legacy
     * radio_medium_set_channel API) talking to each other. */
    bool s_reg = s->spectrum != RADIO_SPECTRUM_NONE;
    bool r_reg = r->spectrum != RADIO_SPECTRUM_NONE;
    if (s_reg && r_reg) {
        if (s->spectrum != r->spectrum) return false;
    } else if (s_reg != r_reg) {
        /* One side registered, the other not. For slot 0 we keep the
         * legacy "unknown receiver allows everything" semantic so a
         * single-radio platform that never bothered to push channel
         * state still receives. For non-zero slots we treat the
         * unregistered side as "no radio on that slot" — drops. This is
         * what stops a Firefly's CC1200 (slot 1, 868 MHz) from leaking
         * into a cc2538dk's empty slot 1. */
        if (sender_radio != 0 || receiver_radio != 0) return false;
    } else {
        /* Both unregistered: cross-band drop based on legacy alias. */
        int sc = s->channel, rc = r->channel;
        if (sc >= 0 && rc >= 0) {
            bool s_sub = sc >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE;
            bool r_sub = rc >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE;
            if (s_sub != r_sub) return false;
        }
    }
    /* Channel match: -1 on either side is "unknown, allow" — TSCH may
     * not have published its hop yet, and the legacy single-channel API
     * never tracks per-radio state. Otherwise channels must agree. */
    if (s->channel >= 0 && r->channel >= 0 && s->channel != r->channel)
        return false;
    /* RX-enabled gate: receiver must be willing to listen. */
    if (!r->rx_enabled) return false;
    return true;
}

/* --- Frame-level filter --- */

bool radio_medium_filter_frame_radio(radio_medium_t *rm,
    int sender, int sender_radio, int receiver, int receiver_radio) {
    if (rm->type == RADIO_MEDIUM_NONE)
        return true;
    if (!valid_node(rm, sender) || !valid_node(rm, receiver)) return true;
    if (!valid_radio(sender_radio) || !valid_radio(receiver_radio)) return true;

    if (!radio_pair_match(rm, sender, sender_radio, receiver, receiver_radio))
        return false;

    /* Distance-based probabilistic check */
    double prob = udgm_reception_prob(rm, sender, receiver);
    if (prob <= 0.0)
        return false;
    if (prob >= 1.0)
        return true;
    return rng_next(rm) < prob;
}

bool radio_medium_filter_frame(radio_medium_t *rm, int sender, int receiver) {
    return radio_medium_filter_frame_radio(rm, sender, 0, receiver, 0);
}

/* --- RSSI --- */

int8_t radio_medium_get_rssi(const radio_medium_t *rm, int sender, int receiver) {
    if (rm->type == RADIO_MEDIUM_NONE)
        return -50;

    double dx = rm->nodes[sender].x - rm->nodes[receiver].x;
    double dy = rm->nodes[sender].y - rm->nodes[receiver].y;
    double dist = sqrt(dx * dx + dy * dy);
    double range = rm->udgm.tx_range;

    if (range <= 0.0)
        return -90;

    double ratio = dist / range;
    if (ratio > 1.0) ratio = 1.0;

    /* Linear interpolation: -10 dBm at distance 0, -90 dBm at tx_range */
    double rssi = -10.0 - 80.0 * ratio;
    if (rssi < -128.0) rssi = -128.0;
    if (rssi > 0.0) rssi = 0.0;
    return (int8_t)rssi;
}

/* --- Byte-level filter --- */

bool radio_medium_filter_byte_radio(radio_medium_t *rm,
    int sender, int sender_radio, int receiver, int receiver_radio, uint8_t byte) {
    /* NONE type: pass everything through */
    if (rm->type == RADIO_MEDIUM_NONE)
        return true;
    if (!valid_node(rm, sender) || !valid_node(rm, receiver)) return true;
    if (!valid_radio(sender_radio) || !valid_radio(receiver_radio)) return true;

    /* Track frame boundaries for this (sender, sender_radio) — needs to
     * happen even on dropped bytes so the tracker stays in sync with the
     * sender's byte stream. */
    track_byte(rm, sender, sender_radio, byte);

    if (!radio_pair_match(rm, sender, sender_radio, receiver, receiver_radio))
        return false;

    /* UDGM distance-based filtering */
    frame_tracker_t *ft = &rm->frame_track[sender][sender_radio];
    rx_decision_t *dec = &rm->rx_decisions[sender][receiver];

    /* If we're inside a tracked frame, use cached per-frame decision */
    if (ft->frame_id > 0 && dec->frame_id == ft->frame_id) {
        if (!dec->decided) {
            /* First byte of this frame for this receiver — roll the dice */
            double prob = udgm_reception_prob(rm, sender, receiver);
            double roll = rng_next(rm);
            dec->drop = (roll >= prob);
            dec->decided = true;
        }
        return !dec->drop;
    }

    /* Outside a tracked frame (e.g. preamble bytes before SFD):
     * Apply distance check only (no probabilistic loss for preamble) */
    double dx = rm->nodes[sender].x - rm->nodes[receiver].x;
    double dy = rm->nodes[sender].y - rm->nodes[receiver].y;
    double dist_sq = dx * dx + dy * dy;
    double range = rm->udgm.tx_range;
    return dist_sq < (range * range);
}

bool radio_medium_filter_byte(radio_medium_t *rm, int sender, int receiver, uint8_t byte) {
    return radio_medium_filter_byte_radio(rm, sender, 0, receiver, 0, byte);
}
