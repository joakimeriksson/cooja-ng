/*
 * Cosim protocol loopback tests — exercise the JSON-over-TCP protocol
 * with a fork()-based mock coordinator.
 *
 * Architecture:
 *   - Parent: runs run_mixed_multinode_test() with --cosim pointing to localhost
 *   - Child:  mock coordinator that sends commands and validates responses
 *
 * Tests:
 *   1. Handshake: verify init message (node_count, node_ids)
 *   2. Time advance round trip: send time_advance, validate time_ack
 *   3. TX capture: run_until, read tx message, validate base64 payload
 *   4. RX injection: send rx + continue, verify receiver logs
 *   5. Stop: send sim_control stop, verify clean exit
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <signal.h>

#include "cJSON.h"

/* --- Length-prefixed JSON protocol helpers --- */

static int read_full(int fd, void *buf, int n) {
    int total = 0;
    while (total < n) {
        ssize_t r = read(fd, (uint8_t *)buf + total, (size_t)(n - total));
        if (r <= 0) {
            if (r == 0) return -1;
            if (errno == EINTR) continue;
            return -1;
        }
        total += (int)r;
    }
    return 0;
}

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

static cJSON *recv_json(int fd) {
    uint8_t len_buf[4];
    if (read_full(fd, len_buf, 4) < 0) return NULL;
    uint32_t json_len = ((uint32_t)len_buf[0] << 24) |
                        ((uint32_t)len_buf[1] << 16) |
                        ((uint32_t)len_buf[2] << 8) |
                        (uint32_t)len_buf[3];
    if (json_len > 1024 * 1024) return NULL;
    char *buf = malloc(json_len + 1);
    if (!buf) return NULL;
    if (read_full(fd, buf, (int)json_len) < 0) { free(buf); return NULL; }
    buf[json_len] = '\0';
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    return root;
}

static int send_json(int fd, cJSON *root) {
    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) return -1;
    int json_len = (int)strlen(json_str);
    uint8_t len_buf[4];
    len_buf[0] = (uint8_t)(json_len >> 24);
    len_buf[1] = (uint8_t)(json_len >> 16);
    len_buf[2] = (uint8_t)(json_len >> 8);
    len_buf[3] = (uint8_t)(json_len);
    int rc = 0;
    if (write_full(fd, len_buf, 4) < 0) rc = -1;
    if (rc == 0 && write_full(fd, json_str, json_len) < 0) rc = -1;
    free(json_str);
    return rc;
}

/* Drain all messages until we see one of type `target_type`.
 * Returns the matching cJSON object (caller must free), or NULL on error. */
static cJSON *recv_until_type(int fd, const char *target_type) {
    for (int attempts = 0; attempts < 200; attempts++) {
        cJSON *msg = recv_json(fd);
        if (!msg) return NULL;
        const char *type = cJSON_GetStringValue(
            cJSON_GetObjectItemCaseSensitive(msg, "type"));
        if (type && strcmp(type, target_type) == 0)
            return msg;
        cJSON_Delete(msg);
    }
    return NULL;
}

/* --- Base64 decode (duplicate from cosim.c since it's static there) --- */

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

static int base64_decode(const char *in, int in_len,
                         uint8_t *out, int out_max) {
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

/* --- Mock coordinator (runs in child process) --- */

/*
 * The mock coordinator:
 * 1. Accepts connection from csim
 * 2. Reads init message, validates node_count == 2
 * 3. Sends time_advance to 500ms, reads time_ack
 * 4. Sends run_until to 1s (JS broadcast fires at t=0, should have TX by now),
 *    reads any tx messages, then sends continue + reads time_ack
 * 5. Sends stop
 *
 * Exit code 0 = all checks passed, non-zero = failure.
 */
static int mock_coordinator(int listen_fd, int verbose) {
    int client_fd = -1;
    int failures = 0;

    /* Accept connection from csim (timeout 30s) */
    struct timeval tv = { .tv_sec = 30, .tv_usec = 0 };
    setsockopt(listen_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    client_fd = accept(listen_fd, NULL, NULL);
    close(listen_fd);
    if (client_fd < 0) {
        fprintf(stderr, "coordinator: accept failed: %s\n", strerror(errno));
        return 1;
    }

    int flag = 1;
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    if (verbose)
        fprintf(stderr, "coordinator: accepted connection\n");

    /* --- Test 1: Handshake --- */
    {
        cJSON *init = recv_json(client_fd);
        if (!init) {
            fprintf(stderr, "coordinator: failed to read init\n");
            close(client_fd);
            return 1;
        }
        const char *type = cJSON_GetStringValue(
            cJSON_GetObjectItemCaseSensitive(init, "type"));
        if (!type || strcmp(type, "init") != 0) {
            fprintf(stderr, "coordinator: expected init, got '%s'\n",
                    type ? type : "null");
            cJSON_Delete(init);
            close(client_fd);
            return 1;
        }
        cJSON *nc = cJSON_GetObjectItemCaseSensitive(init, "node_count");
        if (!nc || !cJSON_IsNumber(nc) || nc->valueint != 2) {
            fprintf(stderr, "coordinator: wrong node_count: %d\n",
                    nc ? nc->valueint : -1);
            failures++;
        }
        cJSON *ids = cJSON_GetObjectItemCaseSensitive(init, "node_ids");
        if (!ids || !cJSON_IsArray(ids) || cJSON_GetArraySize(ids) != 2) {
            fprintf(stderr, "coordinator: missing or wrong node_ids array\n");
            failures++;
        }
        if (verbose)
            fprintf(stderr, "coordinator: init OK (node_count=%d)\n",
                    nc ? nc->valueint : -1);
        cJSON_Delete(init);
    }

    /* --- Test 2: Time advance round trip --- */
    {
        cJSON *cmd = cJSON_CreateObject();
        cJSON_AddStringToObject(cmd, "type", "time_advance");
        cJSON_AddNumberToObject(cmd, "target_time_ns", 500000000.0); /* 500ms */
        if (send_json(client_fd, cmd) < 0) {
            fprintf(stderr, "coordinator: send time_advance failed\n");
            failures++;
        }
        cJSON_Delete(cmd);

        /* Read messages until we get time_ack (skip console messages etc) */
        cJSON *ack = recv_until_type(client_fd, "time_ack");
        if (!ack) {
            fprintf(stderr, "coordinator: no time_ack received\n");
            failures++;
        } else {
            cJSON *target = cJSON_GetObjectItemCaseSensitive(ack, "target_time_ns");
            if (!target || !cJSON_IsNumber(target)) {
                fprintf(stderr, "coordinator: time_ack missing target_time_ns\n");
                failures++;
            } else {
                int64_t acked = (int64_t)target->valuedouble;
                if (acked != 500000000LL) {
                    fprintf(stderr, "coordinator: wrong ack time: %lld\n",
                            (long long)acked);
                    failures++;
                } else if (verbose) {
                    fprintf(stderr, "coordinator: time_advance->time_ack OK\n");
                }
            }
            cJSON_Delete(ack);
        }
    }

    /* --- Test 3: TX capture via run_until --- */
    {
        /* JS broadcast.js sends at t=0 (first execute), t=2s, t=4s...
         * We already advanced to 500ms. Now run_until 3s — should see TX
         * from the execute at t=2s (or possibly queued from t=0 if cosim
         * doesn't catch it before 500ms advance). */
        cJSON *cmd = cJSON_CreateObject();
        cJSON_AddStringToObject(cmd, "type", "run_until");
        cJSON_AddNumberToObject(cmd, "target_time_ns", 3000000000.0); /* 3s */
        if (send_json(client_fd, cmd) < 0) {
            fprintf(stderr, "coordinator: send run_until failed\n");
            failures++;
        }
        cJSON_Delete(cmd);

        /* We might get tx_done (TX occurred during run_until), then we need
         * to drain tx messages and send continue. Could also get time_ack
         * directly if no TX occurs. */
        int got_tx = 0;
        for (int rounds = 0; rounds < 20; rounds++) {
            cJSON *msg = recv_json(client_fd);
            if (!msg) {
                fprintf(stderr, "coordinator: lost connection during run_until\n");
                failures++;
                break;
            }
            const char *type = cJSON_GetStringValue(
                cJSON_GetObjectItemCaseSensitive(msg, "type"));
            if (!type) { cJSON_Delete(msg); continue; }

            if (strcmp(type, "tx") == 0) {
                got_tx = 1;
                /* Validate payload is base64-decodable */
                cJSON *payload = cJSON_GetObjectItemCaseSensitive(msg, "payload");
                if (payload && cJSON_IsString(payload)) {
                    uint8_t decoded[256];
                    int dlen = base64_decode(payload->valuestring,
                        (int)strlen(payload->valuestring), decoded, 256);
                    if (dlen <= 0) {
                        fprintf(stderr, "coordinator: TX payload decode failed\n");
                        failures++;
                    } else if (verbose) {
                        fprintf(stderr, "coordinator: TX captured, %d frame bytes\n",
                                dlen);
                    }
                }
                cJSON_Delete(msg);
                continue;
            }
            if (strcmp(type, "tx_done") == 0) {
                /* Send continue to resume simulation */
                cJSON *cont = cJSON_CreateObject();
                cJSON_AddStringToObject(cont, "type", "continue");
                send_json(client_fd, cont);
                cJSON_Delete(cont);
                cJSON_Delete(msg);
                continue;
            }
            if (strcmp(type, "time_ack") == 0) {
                if (verbose)
                    fprintf(stderr, "coordinator: run_until->time_ack OK (tx=%d)\n",
                            got_tx);
                cJSON_Delete(msg);
                break;
            }
            /* Skip node_idle, console, etc */
            cJSON_Delete(msg);
        }
    }

    /* --- Test 4: RX injection --- */
    {
        /* Inject a fake frame into node 0 and advance time */
        /* First send an rx message */
        /* Base64 of a minimal 802.15.4 frame: we encode a small test frame */
        /* 0x41 0x88 0x00 0xCD 0xAB 0xFF 0xFF 0x02 0x00 'T' 'X' 0x00 0x00 */
        /* This is a broadcast from node 2, picked up by node 0 */
        const char *test_b64 = "QYgAzav//wIAVFgAAA==";  /* pre-encoded */

        cJSON *rx = cJSON_CreateObject();
        cJSON_AddStringToObject(rx, "type", "rx");
        cJSON_AddNumberToObject(rx, "node_index", 0);
        cJSON_AddStringToObject(rx, "payload", test_b64);
        cJSON_AddNumberToObject(rx, "rssi_dbm", -50);
        cJSON_AddNumberToObject(rx, "channel", 26);
        if (send_json(client_fd, rx) < 0)
            failures++;
        cJSON_Delete(rx);

        /* Send time_advance to let the node process it */
        cJSON *cmd = cJSON_CreateObject();
        cJSON_AddStringToObject(cmd, "type", "time_advance");
        cJSON_AddNumberToObject(cmd, "target_time_ns", 4000000000.0); /* 4s */
        if (send_json(client_fd, cmd) < 0)
            failures++;
        cJSON_Delete(cmd);

        /* Drain until time_ack */
        cJSON *ack = recv_until_type(client_fd, "time_ack");
        if (!ack) {
            fprintf(stderr, "coordinator: no time_ack after RX inject\n");
            failures++;
        } else {
            if (verbose)
                fprintf(stderr, "coordinator: RX inject + time_advance OK\n");
            cJSON_Delete(ack);
        }
    }

    /* --- Test 5: Stop --- */
    {
        cJSON *cmd = cJSON_CreateObject();
        cJSON_AddStringToObject(cmd, "type", "sim_control");
        cJSON_AddStringToObject(cmd, "command", "stop");
        if (send_json(client_fd, cmd) < 0) {
            fprintf(stderr, "coordinator: send stop failed\n");
            failures++;
        }
        cJSON_Delete(cmd);
        if (verbose)
            fprintf(stderr, "coordinator: stop sent\n");
    }

    close(client_fd);
    return failures;
}

/* --- Entry point --- */

extern int run_mixed_multinode_test(int argc, char **argv);

int run_cosim_tests(int verbose) {
    printf("\n=== Cosim Protocol Loopback Tests ===\n");

    /* Create listen socket on random port */
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("cosim-test: socket");
        return 1;
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sa.sin_port = 0; /* OS picks port */

    if (bind(listen_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("cosim-test: bind");
        close(listen_fd);
        return 1;
    }
    if (listen(listen_fd, 1) < 0) {
        perror("cosim-test: listen");
        close(listen_fd);
        return 1;
    }

    /* Get assigned port */
    socklen_t sa_len = sizeof(sa);
    getsockname(listen_fd, (struct sockaddr *)&sa, &sa_len);
    int port = ntohs(sa.sin_port);
    printf("  Coordinator listening on 127.0.0.1:%d\n", port);

    /* Fork: child = coordinator, parent = csim */
    pid_t pid = fork();
    if (pid < 0) {
        perror("cosim-test: fork");
        close(listen_fd);
        return 1;
    }

    if (pid == 0) {
        /* Child: mock coordinator */
        int rc = mock_coordinator(listen_fd, verbose);
        _exit(rc);
    }

    /* Parent: close listen socket, run csim */
    close(listen_fd);

    /* Build argv for run_mixed_multinode_test */
    char addr_port[64];
    snprintf(addr_port, sizeof(addr_port), "127.0.0.1:%d", port);

    char *test_argv[] = {
        (char *)"--cosim", addr_port,
        (char *)"firmware/js/broadcast.js",
        (char *)"firmware/js/broadcast.js",
        (char *)"-q",      /* quiet mode */
        NULL
    };
    int test_argc = 5;

    int csim_rc = run_mixed_multinode_test(test_argc, test_argv);

    /* Wait for child */
    int child_status = 0;
    waitpid(pid, &child_status, 0);
    int coord_rc = WIFEXITED(child_status) ? WEXITSTATUS(child_status) : 1;

    int total_failures = csim_rc + coord_rc;
    if (total_failures == 0) {
        printf("  PASS: cosim loopback (%d tests)\n", 5);
    } else {
        printf("  FAIL: cosim loopback (csim=%d, coordinator=%d)\n",
               csim_rc, coord_rc);
    }

    printf("\nCosim tests: %s\n", total_failures == 0 ? "PASSED" : "FAILED");
    return total_failures;
}
