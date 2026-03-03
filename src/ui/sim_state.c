/*
 * Simulation state serializer — converts state to JSON via cJSON
 */
#include "sim_state.h"
#include "cJSON.h"
#include <stdlib.h>
#include <stdio.h>

#define MS_TO_NS 1000000LL

char *sim_state_to_json(const sim_node_info_t *nodes, int node_count,
                        const sim_radio_info_t *radio,
                        const sim_stats_t *stats) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    /* Simulation time in ms */
    cJSON_AddNumberToObject(root, "sim_time_ms",
                            (double)(stats->sim_time_ns / MS_TO_NS));

    /* Nodes array */
    cJSON *jarr = cJSON_CreateArray();
    for (int i = 0; i < node_count; i++) {
        cJSON *jnode = cJSON_CreateObject();
        cJSON_AddNumberToObject(jnode, "id", nodes[i].id);
        cJSON_AddStringToObject(jnode, "type", nodes[i].type);
        cJSON_AddNumberToObject(jnode, "x", nodes[i].x);
        cJSON_AddNumberToObject(jnode, "y", nodes[i].y);
        cJSON_AddNumberToObject(jnode, "cycles", (double)nodes[i].cycles);
        cJSON_AddNumberToObject(jnode, "freq", nodes[i].freq_hz);

        if (nodes[i].last_tx_ns > 0)
            cJSON_AddNumberToObject(jnode, "last_tx_ms",
                                    (double)(nodes[i].last_tx_ns / MS_TO_NS));

        /* Console lines */
        cJSON *jcons = cJSON_CreateArray();
        for (int c = 0; c < nodes[i].console_count; c++)
            cJSON_AddItemToArray(jcons, cJSON_CreateString(nodes[i].console[c]));
        cJSON_AddItemToObject(jnode, "console", jcons);

        cJSON_AddItemToArray(jarr, jnode);
    }
    cJSON_AddItemToObject(root, "nodes", jarr);

    /* Radio info */
    cJSON *jradio = cJSON_CreateObject();
    cJSON_AddStringToObject(jradio, "type", radio->type);
    cJSON_AddNumberToObject(jradio, "tx_range", radio->tx_range);

    /* Neighbor adjacency lists */
    cJSON *jneighbors = cJSON_CreateArray();
    for (int i = 0; i < radio->node_count; i++) {
        cJSON *jnl = cJSON_CreateArray();
        for (int j = 0; j < radio->neighbor_counts[i]; j++)
            cJSON_AddItemToArray(jnl, cJSON_CreateNumber(radio->neighbors[i][j]));
        cJSON_AddItemToArray(jneighbors, jnl);
    }
    cJSON_AddItemToObject(jradio, "neighbors", jneighbors);
    cJSON_AddItemToObject(root, "radio", jradio);

    /* Stats */
    cJSON *jstats = cJSON_CreateObject();
    cJSON_AddNumberToObject(jstats, "rf_bytes", stats->rf_bytes);
    cJSON_AddNumberToObject(jstats, "uart_bytes", stats->uart_bytes);
    cJSON_AddNumberToObject(jstats, "rx_frames_queued", stats->rx_frames_queued);
    cJSON_AddNumberToObject(jstats, "rx_frames_collided", stats->rx_frames_collided);
    cJSON_AddNumberToObject(jstats, "speed", stats->speed_ratio);
    cJSON_AddBoolToObject(jstats, "paused", stats->paused);
    cJSON_AddItemToObject(root, "stats", jstats);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
