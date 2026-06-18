/*
 * Implementation of the WebSocket UI service — see
 * include/sim/websocket_ui_service.h.
 *
 * Phase 6 milestone 39 (§3.19).  Verbatim moves of the runner's
 * ui_message_handler, ui_add_console_line, the ws_server init, and the
 * full-state-JSON / CBOR-delta broadcast block, retargeted onto the
 * service struct + its stored shared-state pointers and describe/move
 * callbacks.  The wall-clock pacing and loop control stay in the runner.
 */
#include "websocket_ui_service.h"
#include "sim_runtime.h"      /* sim_runtime_ui_panels_json (plugin UI panels) */

#include "cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ui_service_poll(websocket_ui_service_t *svc) {
    if (svc->server) ws_server_poll(svc->server);
}

void ui_service_set_stats(websocket_ui_service_t *svc, int rf_bytes,
                          int uart_bytes, int rx_frames_queued,
                          int rx_frames_collided) {
    svc->stat_rf_bytes = rf_bytes;
    svc->stat_uart_bytes = uart_bytes;
    svc->stat_rx_frames_queued = rx_frames_queued;
    svc->stat_rx_frames_collided = rx_frames_collided;
}

/* Incoming UI command: speed / pause / play / full / restart / move. */
static void ui_message_handler(const char *data, int len, void *userdata) {
    websocket_ui_service_t *svc = (websocket_ui_service_t *)userdata;
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return;
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cmd && cJSON_IsString(cmd)) {
        if (strcmp(cmd->valuestring, "speed") == 0) {
            cJSON *val = cJSON_GetObjectItem(root, "value");
            if (val && cJSON_IsNumber(val)) {
                double v = val->valuedouble;
                if (v >= 0.1 && v <= 1000.0)
                    svc->speed_ratio = v;
            }
        } else if (strcmp(cmd->valuestring, "pause") == 0) {
            svc->paused = 1;
        } else if (strcmp(cmd->valuestring, "play") == 0) {
            svc->paused = 0;
        } else if (strcmp(cmd->valuestring, "full") == 0) {
            svc->full_state_requested = 1;
        } else if (strcmp(cmd->valuestring, "restart") == 0) {
            svc->restart_requested = 1;
            svc->paused = 0;
        } else if (strcmp(cmd->valuestring, "move") == 0) {
            cJSON *jnode = cJSON_GetObjectItem(root, "node");
            cJSON *jx = cJSON_GetObjectItem(root, "x");
            cJSON *jy = cJSON_GetObjectItem(root, "y");
            if (jnode && cJSON_IsNumber(jnode) && jx && cJSON_IsNumber(jx) &&
                jy && cJSON_IsNumber(jy) && svc->on_move) {
                svc->on_move(jnode->valueint, jx->valuedouble, jy->valuedouble);
            }
        }
    }
    cJSON_Delete(root);
}

void ui_service_add_console_line(websocket_ui_service_t *svc, int idx,
                                 int64_t sim_ns, const char *line) {
    /* Prepend simulation timestamp */
    char stamped[UI_CONSOLE_LINELEN];
    double sim_s = (double)sim_ns / 1e9;
    snprintf(stamped, sizeof(stamped), "[%7.3f] %s", sim_s, line);

    /* Add to ring buffer */
    int slot = (svc->console_head[idx] + svc->console_count[idx]) % UI_CONSOLE_LINES;
    if (svc->console_count[idx] < UI_CONSOLE_LINES)
        svc->console_count[idx]++;
    else
        svc->console_head[idx] = (svc->console_head[idx] + 1) % UI_CONSOLE_LINES;
    strncpy(svc->console[idx][slot], stamped, UI_CONSOLE_LINELEN - 1);
    svc->console[idx][slot][UI_CONSOLE_LINELEN - 1] = '\0';

    /* Add to new-lines buffer for next broadcast */
    if (svc->console_new_count[idx] < UI_CONSOLE_LINES) {
        int ni = svc->console_new_count[idx]++;
        strncpy(svc->console_new[idx][ni], stamped, UI_CONSOLE_LINELEN - 1);
        svc->console_new[idx][ni][UI_CONSOLE_LINELEN - 1] = '\0';
    }
}

bool ui_service_start(websocket_ui_service_t *svc, int port,
                      const sim_node_state_t *node_states,
                      sim_node_state_t *prev_node_states,
                      const int64_t *node_last_tx_ns,
                      int64_t *prev_last_tx_ns,
                      radio_medium_t *medium, timeline_t *tl,
                      const int *node_count,
                      ui_describe_fn describe, ui_move_fn on_move) {
    svc->node_states = node_states;
    svc->prev_node_states = prev_node_states;
    svc->node_last_tx_ns = node_last_tx_ns;
    svc->prev_last_tx_ns = prev_last_tx_ns;
    svc->medium = medium;
    svc->tl = tl;
    svc->node_count = node_count;
    svc->describe = describe;
    svc->on_move = on_move;

    svc->server = ws_server_init(port);
    if (!svc->server)
        return false;
    ws_server_set_message_callback(svc->server, ui_message_handler, svc);
    /* Load HTML from ui/index.html */
    FILE *hf = fopen("ui/index.html", "r");
    if (hf) {
        fseek(hf, 0, SEEK_END);
        long hlen = ftell(hf);
        fseek(hf, 0, SEEK_SET);
        char *html = malloc((size_t)hlen + 1);
        if (html) {
            size_t rd = fread(html, 1, (size_t)hlen, hf);
            html[rd] = '\0';
            ws_server_set_html(svc->server, html, (int)rd);
            free(html);
        }
        fclose(hf);
    } else {
        fprintf(stderr, "Warning: ui/index.html not found, serving default page\n");
    }
    svc->full_state_requested = 1;
    return true;
}

void ui_service_broadcast(websocket_ui_service_t *svc, int64_t sim_ns) {
    if (!svc->server) return;
    int n = svc->node_count ? *svc->node_count : 0;

    sim_stats_t st = {
        .sim_time_ns = sim_ns,
        .rf_bytes = svc->stat_rf_bytes,
        .uart_bytes = svc->stat_uart_bytes,
        .rx_frames_queued = svc->stat_rx_frames_queued,
        .rx_frames_collided = svc->stat_rx_frames_collided,
        .speed_ratio = svc->speed_ratio,
        .paused = svc->paused,
    };

    int has_clients = ws_server_client_count(svc->server) > 0;
    char *json = NULL;

    if (svc->full_state_requested && has_clients) {
        /* === Full state: on connect/reload === */
        svc->full_state_requested = 0;

        sim_node_info_t ni[UI_SVC_MAX_NODES];
        const char *con_ptrs[UI_SVC_MAX_NODES][UI_CONSOLE_LINES];
        for (int i = 0; i < n; i++) {
            int id = 0; const char *type = ""; int64_t cycles = 0;
            uint32_t freq = 0; int64_t simt = 0;
            if (svc->describe) svc->describe(i, &id, &type, &cycles, &freq, &simt);
            ni[i].id = id;
            ni[i].type = type;
            ni[i].x = svc->medium->nodes[i].x;
            ni[i].y = svc->medium->nodes[i].y;
            ni[i].cycles = cycles;
            ni[i].freq_hz = freq;
            ni[i].sim_time_ns = simt;
            ni[i].last_tx_ns = svc->node_last_tx_ns[i];
            /* Send ALL console history for full state */
            ni[i].console_count = svc->console_count[i];
            int base = (svc->console_head[i] - svc->console_count[i]
                        + UI_CONSOLE_LINES) % UI_CONSOLE_LINES;
            for (int c = 0; c < svc->console_count[i]; c++)
                con_ptrs[i][c] = svc->console[i][(base + c) % UI_CONSOLE_LINES];
            ni[i].console = con_ptrs[i];
        }

        int neighbor_counts[UI_SVC_MAX_NODES];
        const int *neighbor_ptrs[UI_SVC_MAX_NODES];
        for (int i = 0; i < n; i++) {
            neighbor_ptrs[i] = svc->medium->neighbors[i].neighbors;
            neighbor_counts[i] = svc->medium->neighbors[i].count;
        }
        sim_radio_info_t ri = {
            .type = svc->medium->type == RADIO_MEDIUM_UDGM ? "UDGM" : "NONE",
            .tx_range = svc->medium->udgm.tx_range,
            .node_count = n,
            .neighbors = neighbor_ptrs,
            .neighbor_counts = neighbor_counts,
        };

        /* Skip timeline in full state — the viewer accumulates events from
         * the CBOR deltas after connect (keeps full state ~10 KB). */
        json = sim_state_to_json(ni, n, &ri, &st, svc->node_states, NULL);
        /* Flush old timeline events so the first delta only contains events
         * from NOW, not stale history. */
        tl_flush_new(svc->tl);
        for (int i = 0; i < n; i++)
            svc->console_new_count[i] = 0;

    } else if (has_clients) {
        /* === Delta: CBOR binary, change-only === */
        int ids[UI_SVC_MAX_NODES];
        const char *con_new_ptrs[UI_SVC_MAX_NODES][UI_CONSOLE_LINES];
        const char **con_ptr_arr[UI_SVC_MAX_NODES];
        int con_counts[UI_SVC_MAX_NODES];

        for (int i = 0; i < n; i++) {
            int id = 0; const char *type = ""; int64_t cycles = 0;
            uint32_t freq = 0; int64_t simt = 0;
            if (svc->describe) svc->describe(i, &id, &type, &cycles, &freq, &simt);
            ids[i] = id;
            con_counts[i] = svc->console_new_count[i];
            for (int c = 0; c < svc->console_new_count[i]; c++)
                con_new_ptrs[i][c] = svc->console_new[i][c];
            con_ptr_arr[i] = con_new_ptrs[i];
            svc->console_new_count[i] = 0;
        }

        static uint8_t tl_cbor_buf[262144];
        int tl_cbor_len = tl_events_to_cbor(
            (struct timeline_s *)svc->tl, tl_cbor_buf, (int)sizeof(tl_cbor_buf));
        tl_flush_new(svc->tl);

        static uint8_t cbor_buf[524288];
        int cbor_len = sim_state_delta_cbor(
            cbor_buf, (int)sizeof(cbor_buf),
            &st, svc->node_states, svc->prev_node_states,
            n, ids,
            svc->node_last_tx_ns, svc->prev_last_tx_ns,
            (const char ***)con_ptr_arr, con_counts,
            tl_cbor_buf, tl_cbor_len);

        if (cbor_len > 0)
            ws_server_broadcast_binary(svc->server, cbor_buf, cbor_len);

        /* Save current state as previous for next delta */
        memcpy(svc->prev_node_states, svc->node_states,
               sizeof(sim_node_state_t) * UI_SVC_MAX_NODES);
        memcpy(svc->prev_last_tx_ns, svc->node_last_tx_ns,
               sizeof(int64_t) * UI_SVC_MAX_NODES);
    }

    if (json) {
        ws_server_broadcast(svc->server, json, (int)strlen(json));
        free(json);
    }

    /* Plugin UI panels — a small text frame, independent of the full/CBOR
     * state paths.  Sent every broadcast tick so panels track live; the
     * front-end replaces its panel set from this message. */
    if (has_clients && svc->rt) {
        char *panels = sim_runtime_ui_panels_json(svc->rt);
        if (panels) {
            size_t need = strlen(panels) + 40;
            char *msg = malloc(need);
            if (msg) {
                int len = snprintf(msg, need,
                                   "{\"type\":\"panels\",\"panels\":%s}", panels);
                ws_server_broadcast(svc->server, msg, len);
                free(msg);
            }
            free(panels);
        }
    }
}

void ui_service_reset(websocket_ui_service_t *svc) {
    memset(svc->console, 0, sizeof(svc->console));
    memset(svc->console_head, 0, sizeof(svc->console_head));
    memset(svc->console_count, 0, sizeof(svc->console_count));
    memset(svc->console_new, 0, sizeof(svc->console_new));
    memset(svc->console_new_count, 0, sizeof(svc->console_new_count));
    svc->full_state_requested = 1;
}

void ui_service_destroy(websocket_ui_service_t *svc) {
    if (svc->server) {
        ws_server_destroy(svc->server);
        svc->server = NULL;
    }
}
