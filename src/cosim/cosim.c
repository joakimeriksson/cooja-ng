/*
 * Co-simulation interface — TCP client for external simulation coordinator
 *
 * Protocol: 4-byte big-endian length prefix + JSON payload.
 * Uses cJSON (bundled in Cooja-NG at lib/cJSON.c).
 */
#include "cosim.h"
#include "cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

/* Base64 encoding/decoding tables */
static const char b64_enc[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static const uint8_t b64_dec[256] = {
    ['A']=0,  ['B']=1,  ['C']=2,  ['D']=3,  ['E']=4,  ['F']=5,
    ['G']=6,  ['H']=7,  ['I']=8,  ['J']=9,  ['K']=10, ['L']=11,
    ['M']=12, ['N']=13, ['O']=14, ['P']=15, ['Q']=16, ['R']=17,
    ['S']=18, ['T']=19, ['U']=20, ['V']=21, ['W']=22, ['X']=23,
    ['Y']=24, ['Z']=25,
    ['a']=26, ['b']=27, ['c']=28, ['d']=29, ['e']=30, ['f']=31,
    ['g']=32, ['h']=33, ['i']=34, ['j']=35, ['k']=36, ['l']=37,
    ['m']=38, ['n']=39, ['o']=40, ['p']=41, ['q']=42, ['r']=43,
    ['s']=44, ['t']=45, ['u']=46, ['v']=47, ['w']=48, ['x']=49,
    ['y']=50, ['z']=51,
    ['0']=52, ['1']=53, ['2']=54, ['3']=55, ['4']=56, ['5']=57,
    ['6']=58, ['7']=59, ['8']=60, ['9']=61, ['+']=62, ['/']=63,
};

static int base64_encode(const uint8_t *in, int in_len, char *out, int out_max) {
    int i, j = 0;
    for (i = 0; i + 2 < in_len; i += 3) {
        if (j + 4 >= out_max) return -1;
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i+1] << 8) | in[i+2];
        out[j++] = b64_enc[(v >> 18) & 0x3F];
        out[j++] = b64_enc[(v >> 12) & 0x3F];
        out[j++] = b64_enc[(v >> 6) & 0x3F];
        out[j++] = b64_enc[v & 0x3F];
    }
    if (i < in_len) {
        if (j + 4 >= out_max) return -1;
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < in_len) v |= (uint32_t)in[i+1] << 8;
        out[j++] = b64_enc[(v >> 18) & 0x3F];
        out[j++] = b64_enc[(v >> 12) & 0x3F];
        out[j++] = (i + 1 < in_len) ? b64_enc[(v >> 6) & 0x3F] : '=';
        out[j++] = '=';
    }
    out[j] = '\0';
    return j;
}

static int base64_decode(const char *in, int in_len, uint8_t *out, int out_max) {
    int i, j = 0;
    for (i = 0; i + 3 < in_len; i += 4) {
        if (in[i] == '=' || in[i+1] == '=') break;
        if (j + 3 > out_max) return -1;
        uint32_t v = ((uint32_t)b64_dec[(uint8_t)in[i]] << 18) |
                     ((uint32_t)b64_dec[(uint8_t)in[i+1]] << 12) |
                     ((uint32_t)b64_dec[(uint8_t)in[i+2]] << 6) |
                     (uint32_t)b64_dec[(uint8_t)in[i+3]];
        out[j++] = (uint8_t)(v >> 16);
        if (in[i+2] != '=') out[j++] = (uint8_t)(v >> 8);
        if (in[i+3] != '=') out[j++] = (uint8_t)v;
    }
    return j;
}

/* Connection state */
static int cosim_fd = -1;

/* Read exactly n bytes from socket */
static int read_full(int fd, void *buf, int n) {
    int total = 0;
    while (total < n) {
        ssize_t r = read(fd, (uint8_t *)buf + total, (size_t)(n - total));
        if (r <= 0) {
            if (r == 0) return -1; /* EOF */
            if (errno == EINTR) continue;
            return -1;
        }
        total += (int)r;
    }
    return 0;
}

/* Write exactly n bytes to socket */
static int write_full(int fd, const void *buf, int n) {
    int total = 0;
    while (total < n) {
        ssize_t w = write(fd, (const uint8_t *)buf + total, (size_t)(n - total));
        if (w <= 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        total += (int)w;
    }
    return 0;
}

/* Send a JSON message with 4-byte big-endian length prefix */
static int send_json(cJSON *root) {
    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) return -1;

    int json_len = (int)strlen(json_str);
    uint8_t len_buf[4];
    len_buf[0] = (uint8_t)(json_len >> 24);
    len_buf[1] = (uint8_t)(json_len >> 16);
    len_buf[2] = (uint8_t)(json_len >> 8);
    len_buf[3] = (uint8_t)(json_len);

    int rc = 0;
    if (write_full(cosim_fd, len_buf, 4) < 0) rc = -1;
    if (rc == 0 && write_full(cosim_fd, json_str, json_len) < 0) rc = -1;

    free(json_str);
    return rc;
}

/* Receive a JSON message (4-byte length prefix + JSON payload).
 * Returns cJSON object (caller must cJSON_Delete) or NULL on error. */
static cJSON *recv_json(void) {
    uint8_t len_buf[4];
    if (read_full(cosim_fd, len_buf, 4) < 0)
        return NULL;

    uint32_t json_len = ((uint32_t)len_buf[0] << 24) |
                        ((uint32_t)len_buf[1] << 16) |
                        ((uint32_t)len_buf[2] << 8) |
                        (uint32_t)len_buf[3];

    if (json_len > 1024 * 1024) {
        fprintf(stderr, "cosim: message too large: %u bytes\n", json_len);
        return NULL;
    }

    char *buf = malloc(json_len + 1);
    if (!buf) return NULL;

    if (read_full(cosim_fd, buf, (int)json_len) < 0) {
        free(buf);
        return NULL;
    }
    buf[json_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    return root;
}

/* --- Public API --- */

int cosim_init(const char *addr, int port) {
    cosim_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (cosim_fd < 0) {
        perror("cosim: socket");
        return -1;
    }

    /* Disable Nagle for low-latency messaging */
    int flag = 1;
    setsockopt(cosim_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, addr, &sa.sin_addr) <= 0) {
        fprintf(stderr, "cosim: invalid address: %s\n", addr);
        close(cosim_fd);
        cosim_fd = -1;
        return -1;
    }

    /* Retry connection (coordinator may not be listening yet) */
    for (int attempt = 0; attempt < 30; attempt++) {
        if (connect(cosim_fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            printf("cosim: connected to %s:%d\n", addr, port);
            return 0;
        }
        if (attempt < 29) {
            usleep(500000); /* 500ms between retries */
        }
    }

    fprintf(stderr, "cosim: failed to connect to %s:%d after retries\n",
            addr, port);
    close(cosim_fd);
    cosim_fd = -1;
    return -1;
}

void cosim_close(void) {
    if (cosim_fd >= 0) {
        close(cosim_fd);
        cosim_fd = -1;
    }
}

int cosim_send_init(const cosim_node_info_t *nodes, int node_count) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "init");
    cJSON_AddNumberToObject(root, "node_count", node_count);

    cJSON *ids = cJSON_CreateArray();
    for (int i = 0; i < node_count; i++) {
        cJSON_AddItemToArray(ids, cJSON_CreateString(nodes[i].node_id));
    }
    cJSON_AddItemToObject(root, "node_ids", ids);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

int cosim_send_tx(int node_index, const char *node_id,
                   const uint8_t *payload, int len,
                   int tx_power_dbm, int channel,
                   uint64_t timestamp_ns) {
    char b64[512];
    if (base64_encode(payload, len, b64, sizeof(b64)) < 0) return -1;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "tx");
    cJSON_AddStringToObject(root, "node_id", node_id);
    cJSON_AddNumberToObject(root, "node_index", node_index);
    cJSON_AddStringToObject(root, "payload", b64);
    cJSON_AddNumberToObject(root, "tx_power_dbm", tx_power_dbm);
    cJSON_AddNumberToObject(root, "channel", channel);
    /* cJSON uses double which has 53 bits of mantissa — sufficient for ns timestamps
     * up to ~104 days. For longer simulations, use string encoding. */
    cJSON_AddNumberToObject(root, "timestamp_ns", (double)timestamp_ns);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

int cosim_send_time_ack(int64_t target_time_ns) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "time_ack");
    cJSON_AddNumberToObject(root, "target_time_ns", (double)target_time_ns);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

int cosim_send_node_idle(const char *node_id,
                          const uint64_t *timer_times_ns,
                          int timer_count) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "node_idle");
    cJSON_AddStringToObject(root, "node_id", node_id);

    cJSON *timers = cJSON_CreateArray();
    for (int i = 0; i < timer_count; i++) {
        cJSON_AddItemToArray(timers,
            cJSON_CreateNumber((double)timer_times_ns[i]));
    }
    cJSON_AddItemToObject(root, "scheduled_timer_times_ns", timers);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

int cosim_send_tx_done(int64_t time_ns) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "tx_done");
    cJSON_AddNumberToObject(root, "time_ns", (double)time_ns);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

int cosim_send_console(const char *node_id, const char *text,
                        uint64_t timestamp_ns) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "console");
    cJSON_AddStringToObject(root, "node_id", node_id);
    cJSON_AddStringToObject(root, "text", text);
    cJSON_AddNumberToObject(root, "timestamp_ns", (double)timestamp_ns);

    int rc = send_json(root);
    cJSON_Delete(root);
    return rc;
}

/* Parse a single RX message into the queue. Returns 1 if added, 0 if full. */
static int parse_rx_message(cJSON *root, cosim_rx_inject_t *rx_queue,
                            int *rx_count, int rx_max) {
    if (*rx_count >= rx_max) return 0;

    cosim_rx_inject_t *rx = &rx_queue[*rx_count];
    cJSON *idx = cJSON_GetObjectItemCaseSensitive(root, "node_index");
    cJSON *src = cJSON_GetObjectItemCaseSensitive(root, "source_index");
    cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
    cJSON *rssi = cJSON_GetObjectItemCaseSensitive(root, "rssi_dbm");
    cJSON *ch = cJSON_GetObjectItemCaseSensitive(root, "channel");

    rx->source_index = -1;
    if (idx && cJSON_IsNumber(idx))
        rx->node_index = idx->valueint;
    if (src && cJSON_IsNumber(src))
        rx->source_index = src->valueint;
    if (rssi && cJSON_IsNumber(rssi))
        rx->rssi_dbm = (int8_t)rssi->valueint;
    if (ch && cJSON_IsNumber(ch))
        rx->channel = ch->valueint;
    if (payload && cJSON_IsString(payload)) {
        rx->payload_len = base64_decode(payload->valuestring,
            (int)strlen(payload->valuestring),
            rx->payload, COSIM_MAX_FRAME_LEN);
        if (rx->payload_len > 0) {
            (*rx_count)++;
            return 1;
        }
    }
    return 0;
}

int cosim_wait_command(cosim_time_cmd_t *cmd,
                        cosim_rx_inject_t *rx_queue, int rx_max) {
    int rx_count = 0;
    cmd->type = COSIM_CMD_NONE;
    cmd->target_time_ns = 0;
    cmd->quantum_ns = 0;

    /* Read messages until we get a time/control command.
     * RX packets are accumulated in the queue along the way. */
    while (cmd->type == COSIM_CMD_NONE) {
        cJSON *root = recv_json();
        if (!root) return -1;

        const char *type = cJSON_GetStringValue(
            cJSON_GetObjectItemCaseSensitive(root, "type"));
        if (!type) {
            cJSON_Delete(root);
            return -1;
        }

        if (strcmp(type, "time_advance") == 0) {
            cmd->type = COSIM_CMD_TIME_ADVANCE;
            cJSON *target = cJSON_GetObjectItemCaseSensitive(root, "target_time_ns");
            cJSON *quantum = cJSON_GetObjectItemCaseSensitive(root, "quantum_ns");
            if (target && cJSON_IsNumber(target))
                cmd->target_time_ns = (int64_t)target->valuedouble;
            if (quantum && cJSON_IsNumber(quantum))
                cmd->quantum_ns = (int64_t)quantum->valuedouble;

        } else if (strcmp(type, "step_to") == 0) {
            cmd->type = COSIM_CMD_STEP_TO;
            cJSON *target = cJSON_GetObjectItemCaseSensitive(root, "target_time_ns");
            if (target && cJSON_IsNumber(target))
                cmd->target_time_ns = (int64_t)target->valuedouble;

        } else if (strcmp(type, "rx") == 0) {
            parse_rx_message(root, rx_queue, &rx_count, rx_max);
            /* Keep looping — RX alone doesn't end the wait */

        } else if (strcmp(type, "run_until") == 0) {
            cmd->type = COSIM_CMD_RUN_UNTIL;
            cJSON *target = cJSON_GetObjectItemCaseSensitive(root, "target_time_ns");
            if (target && cJSON_IsNumber(target))
                cmd->target_time_ns = (int64_t)target->valuedouble;

        } else if (strcmp(type, "continue") == 0) {
            cmd->type = COSIM_CMD_CONTINUE;

        } else if (strcmp(type, "sim_control") == 0) {
            cJSON *command = cJSON_GetObjectItemCaseSensitive(root, "command");
            if (command && cJSON_IsString(command) &&
                strcmp(command->valuestring, "stop") == 0) {
                cmd->type = COSIM_CMD_STOP;
            }
        }

        cJSON_Delete(root);
    }

    return rx_count;
}
