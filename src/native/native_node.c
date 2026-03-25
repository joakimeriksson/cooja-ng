/*
 * Native Cooja mote — dlopen/tick/step implementation
 *
 * Each native node is a TARGET=cooja Contiki-NG shared library loaded
 * via dlopen(). The library exports cooja_init()/cooja_tick() and shared
 * variables for time, radio, and serial I/O.
 *
 * To ensure independent instances, the .cooja file is copied to a
 * per-node temp file before loading.
 */
#include "native_node.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <dlfcn.h>

/* Resolve a dlsym symbol, print error and return -1 on failure */
#define RESOLVE_SYM(node, name, type) do { \
    (node)->name = (type)dlsym((node)->dl_handle, #name); \
    if (!(node)->name) { \
        fprintf(stderr, "native_node: dlsym(%s) failed: %s\n", #name, dlerror()); \
        return -1; \
    } \
} while (0)

/* Resolve a dlsym symbol, allow NULL (optional) */
#define RESOLVE_SYM_OPT(node, name, type) do { \
    (node)->name = (type)dlsym((node)->dl_handle, #name); \
} while (0)

static int copy_file(const char *src, const char *dst) {
    FILE *in = fopen(src, "rb");
    if (!in) return -1;
    FILE *out = fopen(dst, "wb");
    if (!out) { fclose(in); return -1; }

    char buf[8192];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) {
            fclose(in); fclose(out); return -1;
        }
    }
    fclose(in);
    fclose(out);
    return 0;
}

int native_node_init(native_node_t *node, const char *firmware_path, int node_id) {
    memset(node, 0, sizeof(*node));
    node->node_id = node_id;

    /* Copy firmware to a per-node temp file for independent dlopen */
#ifdef __APPLE__
    const char *ext = ".dylib";
#else
    const char *ext = ".so";
#endif
    snprintf(node->dl_path, sizeof(node->dl_path),
             "/tmp/csim_node_%d_%d%s", node_id, (int)getpid(), ext);
    node->dl_path_is_temp = true;

    if (copy_file(firmware_path, node->dl_path) != 0) {
        fprintf(stderr, "native_node: failed to copy %s -> %s\n",
                firmware_path, node->dl_path);
        return -1;
    }

    /* dlopen with RTLD_NOW | RTLD_LOCAL for independent symbol spaces */
    node->dl_handle = dlopen(node->dl_path, RTLD_NOW | RTLD_LOCAL);
    if (!node->dl_handle) {
        fprintf(stderr, "native_node: dlopen(%s) failed: %s\n",
                node->dl_path, dlerror());
        unlink(node->dl_path);
        return -1;
    }

    /* Resolve function pointers */
    RESOLVE_SYM(node, cooja_init, int (*)(void));
    RESOLVE_SYM(node, cooja_tick, void (*)(void));

    /* Resolve shared variable pointers */
    RESOLVE_SYM(node, simCurrentTime, uint64_t *);
    RESOLVE_SYM(node, simRtimerCurrentTicks, uint64_t *);
    RESOLVE_SYM(node, simEtimerPending, int *);
    RESOLVE_SYM(node, simEtimerNextExpirationTime, uint64_t *);
    RESOLVE_SYM(node, simRtimerPending, int *);
    RESOLVE_SYM(node, simRtimerNextExpirationTime, uint64_t *);
    RESOLVE_SYM(node, simProcessRunValue, int *);
    RESOLVE_SYM(node, simInDataBuffer, char *);
    RESOLVE_SYM(node, simInSize, int *);
    RESOLVE_SYM(node, simOutDataBuffer, char *);
    RESOLVE_SYM(node, simOutSize, int *);
    RESOLVE_SYM(node, simRadioHWOn, char *);
    RESOLVE_SYM(node, simRadioChannel, int *);
    RESOLVE_SYM(node, simReceiving, char *);
    RESOLVE_SYM(node, simLoggedData, char *);
    RESOLVE_SYM(node, simLoggedLength, int *);
    RESOLVE_SYM(node, simLoggedFlag, char *);
    RESOLVE_SYM(node, simMoteID, int *);
    RESOLVE_SYM(node, simMoteIDChanged, char *);
    RESOLVE_SYM(node, simRandomSeed, int *);
    RESOLVE_SYM_OPT(node, simSerialReceivingData, char *);
    RESOLVE_SYM_OPT(node, simSerialReceivingLength, int *);
    RESOLVE_SYM_OPT(node, simSerialReceivingFlag, char *);

    /* Set initial state before cooja_init */
    *node->simMoteID = node_id;
    *node->simMoteIDChanged = 1;
    *node->simRandomSeed = node_id * 12345 + 67890;
    *node->simCurrentTime = 0;
    *node->simRtimerCurrentTicks = 0;
    *node->simInSize = 0;
    *node->simOutSize = 0;
    *node->simLoggedFlag = 0;
    *node->simLoggedLength = 0;

    printf("  Native node %d: calling cooja_init()...\n", node_id);
    int ret = node->cooja_init();
    if (ret != 0) {
        fprintf(stderr, "native_node: cooja_init() returned %d\n", ret);
    }

    /* Initial ticks to complete boot.
     * Tick 1: contiki_init() runs, reads simMoteID.
     * Tick 2: process simMoteIDChanged (node_id_init re-reads if changed).
     * Note: don't call native_check_log_output here — the caller sets up
     * the log callback after init, then does additional boot ticks. */
    *node->simProcessRunValue = 1;
    node->cooja_tick();

    /* Ensure node_id is applied — set changed flag and tick again */
    *node->simMoteIDChanged = 1;
    *node->simProcessRunValue = 1;
    node->cooja_tick();

    node->sim_time_ns = 0;
    printf("  Native node %d: initialized successfully\n", node_id);
    return 0;
}

void native_node_destroy(native_node_t *node) {
    if (node->dl_handle) {
        dlclose(node->dl_handle);
        node->dl_handle = NULL;
    }
    if (node->dl_path_is_temp && node->dl_path[0]) {
        unlink(node->dl_path);
        node->dl_path[0] = '\0';
    }
}

/* Compute the next time the node needs to wake up (ns).
 * Only checks timer events and processRunValue.
 * RX frame queue is handled externally by native_dequeue_rx_frame(). */
static int64_t compute_next_wakeup(const native_node_t *node) {
    int64_t next_ns = INT64_MAX;

    /* Etimer: expiration time is in ms */
    if (*node->simEtimerPending) {
        int64_t et_ns = (int64_t)(*node->simEtimerNextExpirationTime) * 1000000LL;
        if (et_ns < next_ns) next_ns = et_ns;
    }

    /* Rtimer: expiration time is in us */
    if (*node->simRtimerPending) {
        int64_t rt_ns = (int64_t)(*node->simRtimerNextExpirationTime) * 1000LL;
        if (rt_ns < next_ns) next_ns = rt_ns;
    }

    /* processRunValue forces a tick — schedule +1ms like COOJA does
     * (ContikiClock.doActionsAfterTick: currentSimulationTime + MILLISECOND) */
    if (*node->simProcessRunValue) {
        int64_t prv_ns = node->sim_time_ns + 1000000LL;
        if (prv_ns < next_ns) next_ns = prv_ns;
    }

    return next_ns;
}

int64_t native_next_wakeup_ns(const native_node_t *node) {
    /* Only report timer-based wakeups for adaptive stepping.
     * processRunValue and rx_queue are handled by idle-skip in the outer loop. */
    int64_t next_ns = INT64_MAX;
    if (*node->simEtimerPending) {
        int64_t et_ns = (int64_t)(*node->simEtimerNextExpirationTime) * 1000000LL;
        if (et_ns < next_ns) next_ns = et_ns;
    }
    if (*node->simRtimerPending) {
        int64_t rt_ns = (int64_t)(*node->simRtimerNextExpirationTime) * 1000LL;
        if (rt_ns < next_ns) next_ns = rt_ns;
    }
    return next_ns;
}

/*
 * Dequeue one frame from the RX queue for delivery.
 * Skips collided frames. Returns true if a good frame was delivered.
 */
bool native_dequeue_rx_frame(native_node_t *node) {
    while (node->rx_queue.count > 0) {
        native_pending_frame_t *head =
            &node->rx_queue.frames[node->rx_queue.head];
        node->rx_queue.head =
            (node->rx_queue.head + 1) % NATIVE_RX_QUEUE_SIZE;
        node->rx_queue.count--;

        if (!head->collided) {
            /* Good frame — deliver to simInDataBuffer */
            memcpy(node->simInDataBuffer, head->data, (size_t)head->len);
            *node->simInSize = head->len;
            return true;
        }
        /* Collided frame — skip it and try next */
    }
    return false;
}

void native_step_until_ns(native_node_t *node, int64_t target_ns) {
    int64_t last_tick_ns = -1;  /* detect stale timer loops */
    while (node->sim_time_ns < target_ns) {
        int64_t next_ns = compute_next_wakeup(node);
        if (next_ns > target_ns) {
            node->sim_time_ns = target_ns;
            break;
        }

        /* Advance time to the next event */
        if (next_ns > node->sim_time_ns)
            node->sim_time_ns = next_ns;

        /* Stale timer detection: if we're about to tick at the same time
         * as last iteration AND no rtimer/processRun is pending, the etimer
         * is stale. Skip ahead by 1ms. BUT if rtimer is pending or process
         * has work, allow same-time ticking (needed for TSCH slot processing). */
        if (node->sim_time_ns == last_tick_ns &&
            !*node->simProcessRunValue && !*node->simRtimerPending) {
            node->sim_time_ns += 1000000LL;
            if (node->sim_time_ns > target_ns) {
                node->sim_time_ns = target_ns;
                break;
            }
        }
        last_tick_ns = node->sim_time_ns;

        /* Update simulation time variables */
        *node->simCurrentTime        = (uint64_t)(node->sim_time_ns / 1000000LL);
        *node->simRtimerCurrentTicks = (uint64_t)(node->sim_time_ns / 1000LL);

        /* Process ticks. After TX, call yield callback for ACK delivery.
         * For non-TX ticks with processRunValue=1, continue ticking
         * at same time (needed for CSMA ACK busywait). */
        for (int same_time = 0; same_time < 20; same_time++) {
            node->cooja_tick();
            int had_tx = (*node->simOutSize > 0);
            native_check_radio_tx(node);
            native_check_log_output(node);

            if (had_tx && node->yield_callback) {
                node->yield_callback(node->yield_callback_data);
                continue;
            }

            if (!*node->simProcessRunValue) break;
            if (node->yield_callback)
                node->yield_callback(node->yield_callback_data);
        }
    }
}

void native_check_log_output(native_node_t *node) {
    if (!*node->simLoggedFlag) return;

    if (node->log_callback) {
        int len = *node->simLoggedLength;
        for (int i = 0; i < len; i++) {
            node->log_callback(node->log_callback_data,
                               (uint8_t)node->simLoggedData[i]);
        }
        /* Add newline if not already present */
        if (len > 0 && node->simLoggedData[len - 1] != '\n') {
            node->log_callback(node->log_callback_data, '\n');
        }
    }

    *node->simLoggedFlag = 0;
    *node->simLoggedLength = 0;
}

void native_check_radio_tx(native_node_t *node) {
    if (*node->simOutSize <= 0) return;

    int frame_len = *node->simOutSize;
    uint8_t *frame = (uint8_t *)node->simOutDataBuffer;

    /* Notify frame callback (for native-to-native) */
    if (node->rf_frame_callback) {
        node->rf_frame_callback(node->rf_frame_callback_data, frame, frame_len);
    }

    /* Notify byte-stream callback (for native-to-emulated) */
    if (node->rf_tx_callback) {
        uint8_t byte_buf[300];
        int byte_len = native_frame_to_bytes(frame, frame_len, byte_buf, sizeof(byte_buf));
        for (int i = 0; i < byte_len; i++) {
            node->rf_tx_callback(node->rf_tx_callback_data, byte_buf[i]);
        }
    }

    *node->simOutSize = 0;
}

void native_deliver_frame(native_node_t *node, const uint8_t *frame, int len,
                          int64_t arrival_ns, int sender_idx) {
    if (len > 128) len = 128;

    native_rx_queue_t *q = &node->rx_queue;

    /* If queue is full, drop oldest */
    if (q->count >= NATIVE_RX_QUEUE_SIZE) {
        q->head = (q->head + 1) % NATIVE_RX_QUEUE_SIZE;
        q->count--;
    }

    /* Compute slot for new frame */
    int tail = (q->head + q->count) % NATIVE_RX_QUEUE_SIZE;
    native_pending_frame_t *slot = &q->frames[tail];

    memcpy(slot->data, frame, (size_t)len);
    slot->len = len;
    slot->arrival_ns = arrival_ns;
    /* Frame duration: (len + 6) bytes at 32us/byte = (len+6)*32000 ns
     * The +6 accounts for preamble(4) + SFD(1) + length(1) */
    slot->end_ns = arrival_ns + (int64_t)(len + 6) * 32000LL;
    slot->sender_idx = sender_idx;
    slot->collided = false;

    /* Check temporal overlap with existing queued frames → collision */
    for (int i = 0; i < q->count; i++) {
        int idx = (q->head + i) % NATIVE_RX_QUEUE_SIZE;
        native_pending_frame_t *existing = &q->frames[idx];
        /* Two frames overlap if one starts before the other ends */
        if (slot->arrival_ns < existing->end_ns &&
            existing->arrival_ns < slot->end_ns) {
            existing->collided = true;
            slot->collided = true;
        }
    }

    q->count++;
}
