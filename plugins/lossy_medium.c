/*
 * lossy_medium — the Phase 11 example radio-medium plugin (§3.24).
 *
 * A minimal external RADIO MEDIUM: it registers a `sim_medium_ops_t` policy
 * via the plugin ABI's register_radio_medium (v2) and is selected by config
 * `medium.type: "lossy"`.  It reuses the host's UDGM pipeline (frame tracking,
 * spectrum/channel matching, the dice roll) and overrides only the policy:
 *
 *   - reception_prob = a constant 0.5 — every in-range link delivers a frame
 *     with 50% probability, regardless of distance (visibly distinct from
 *     UDGM's distance falloff, which is lossless at success_ratio=1).
 *   - get_rssi       = a constant -60 dBm.
 *   - compute_neighbors = the same distance-disc as UDGM, built through the
 *     host accessors (so the plugin never names radio_medium_t internals).
 *
 * It reaches the host only through the csim_api vtable + the radio_medium
 * accessors.  Build via the Makefile `plugins` target; select with
 *   "plugins": ["build/plugins/lossy_medium.so"], "medium": {"type": "lossy", ...}
 */
#include "csim_plugin.h"
#include "radio_medium.h"

static double lossy_reception_prob(const radio_medium_t *rm, int sender,
                                   int receiver) {
    (void)rm; (void)sender; (void)receiver;
    return 0.5;   /* constant 50% loss — distinct from UDGM's distance curve */
}

static int8_t lossy_get_rssi(const radio_medium_t *rm, int sender,
                             int receiver) {
    (void)rm; (void)sender; (void)receiver;
    return -60;
}

/* Distance-disc neighbors, exactly like UDGM but built through the accessors. */
static void lossy_compute_neighbors(radio_medium_t *rm) {
    udgm_config_t p;
    radio_medium_udgm_params(rm, &p);
    double tx2  = p.tx_range * p.tx_range;
    double int2 = p.interference_range * p.interference_range;
    int n = radio_medium_node_count(rm);
    for (int i = 0; i < n; i++) {
        radio_medium_clear_neighbors(rm, i);
        double xi, yi;
        radio_medium_node_pos(rm, i, &xi, &yi);
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            double xj, yj;
            radio_medium_node_pos(rm, j, &xj, &yj);
            double dx = xi - xj, dy = yi - yj;
            double d2 = dx * dx + dy * dy;
            if (d2 <= tx2)
                radio_medium_add_neighbor(rm, i, j);
            else if (d2 <= int2)
                radio_medium_add_interferer(rm, i, j);
        }
    }
}

static const sim_medium_ops_t lossy_ops = {
    .name              = "lossy",
    .reception_prob    = lossy_reception_prob,
    .get_rssi          = lossy_get_rssi,
    .compute_neighbors = lossy_compute_neighbors,
};

static const sim_medium_type_t lossy_type = {
    .name     = "lossy",
    .pipeline = RADIO_MEDIUM_UDGM,   /* the full filter pipeline, our policy */
    .ops      = &lossy_ops,
};

int csim_plugin_init(const csim_api_t *api) {
    /* register_radio_medium is a v2 capability. */
    if (!api || api->version < 2u)
        return -1;
    if (!api->registry || !api->registry->register_radio_medium)
        return -1;
    return api->registry->register_radio_medium(api->reg, &lossy_type) < 0
               ? -1 : 0;
}
