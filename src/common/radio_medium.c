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

/* Cross-band check: drop bytes between sub-GHz and 2.4 GHz nodes. The
 * within-band channel check stays disabled (TSCH hopping makes stale
 * channel values unreliable between ticks), but the cross-band check
 * is needed for dual-radio Firefly fan-out — it isolates CC1200 chatter
 * from cc2538 RF Core nodes (and vice versa) without a real dual-radio
 * refactor. See devices/zoul-firefly/SPEC.md "Radio medium strategy". */
static inline bool cross_band_drop(int ch_s, int ch_r) {
    if (ch_s < 0 || ch_r < 0) return false;
    bool s_sub = (ch_s >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
    bool r_sub = (ch_r >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
    return s_sub != r_sub;
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

static void track_byte(radio_medium_t *rm, int sender, uint8_t byte) {
    frame_tracker_t *ft = &rm->frame_track[sender];
    if (ft->profile == RADIO_FRAME_PROFILE_IEEE802154G) {
        track_byte_802154g(rm, sender, byte, ft);
    } else {
        track_byte_802154(rm, sender, byte, ft);
    }
}

/* --- Public API --- */

void radio_medium_init(radio_medium_t *rm, int node_count) {
    memset(rm, 0, sizeof(*rm));
    rm->type = RADIO_MEDIUM_NONE;
    rm->node_count = node_count;
    rm->rng_state = 0x12345678;  /* default seed */
    rm->next_frame_id = 0;

    for (int i = 0; i < RADIO_MEDIUM_MAX_NODES; i++) {
        rm->nodes[i].channel = -1;
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
    if (node >= 0 && node < RADIO_MEDIUM_MAX_NODES) {
        rm->nodes[node].x = x;
        rm->nodes[node].y = y;
    }
}

void radio_medium_set_channel(radio_medium_t *rm, int node, int channel) {
    if (node >= 0 && node < RADIO_MEDIUM_MAX_NODES) {
        rm->nodes[node].channel = channel;
        /* Pick the on-air frame profile from the channel range. The
         * sub-GHz CC1200 lives at channels >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE;
         * everything else is 2.4 GHz IEEE 802.15.4. */
        radio_frame_profile_t prof =
            (channel >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE)
            ? RADIO_FRAME_PROFILE_IEEE802154G
            : RADIO_FRAME_PROFILE_IEEE802154;
        if (rm->frame_track[node].profile != prof) {
            /* Profile changed — reset the frame tracker so we're not
             * carrying forward state from the old framing. */
            rm->frame_track[node].profile = prof;
            rm->frame_track[node].state = FRAME_IDLE;
            rm->frame_track[node].zero_count = 0;
            rm->frame_track[node].sync_match = 0;
        }
    }
}

void radio_medium_set_seed(radio_medium_t *rm, uint32_t seed) {
    rm->rng_state = seed ? seed : 0x12345678;
}

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

bool radio_medium_filter_frame(radio_medium_t *rm, int sender, int receiver) {
    if (rm->type == RADIO_MEDIUM_NONE)
        return true;

    /* Cross-band check: same as filter_byte — 2.4 GHz never reaches sub-GHz. */
    int ch_s = rm->nodes[sender].channel;
    int ch_r = rm->nodes[receiver].channel;
    if (cross_band_drop(ch_s, ch_r))
        return false;
    /* Within-band channel check disabled — see filter_byte. */

    /* Distance-based probabilistic check */
    double prob = udgm_reception_prob(rm, sender, receiver);
    if (prob <= 0.0)
        return false;
    if (prob >= 1.0)
        return true;
    return rng_next(rm) < prob;
}

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

bool radio_medium_filter_byte(radio_medium_t *rm, int sender, int receiver, uint8_t byte) {
    /* NONE type: pass everything through */
    if (rm->type == RADIO_MEDIUM_NONE)
        return true;

    /* Track frame boundaries for this sender */
    track_byte(rm, sender, byte);

    /* Cross-band check: 2.4 GHz <-> sub-GHz never overhear each other. */
    int ch_s = rm->nodes[sender].channel;
    int ch_r = rm->nodes[receiver].channel;
    if (cross_band_drop(ch_s, ch_r))
        return false;
    /* Within-band channel check disabled for now — TSCH channel hopping
     * makes stale simRadioChannel values unreliable between ticks.
     * TODO: implement per-event channel sync for TSCH. */

    /* UDGM distance-based filtering */
    frame_tracker_t *ft = &rm->frame_track[sender];
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
