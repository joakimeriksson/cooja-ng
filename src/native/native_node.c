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

/* Resolve a dlsym symbol; on failure jump to the cleanup ladder in
 * native_node_init so the dl handle and temp file are not leaked. */
#define RESOLVE_SYM(node, name, type) do { \
    (node)->name = (type)dlsym((node)->dl_handle, #name); \
    if (!(node)->name) { \
        fprintf(stderr, "native_node: dlsym(%s) failed: %s\n", #name, dlerror()); \
        goto resolve_fail; \
    } \
} while (0)

/* Resolve a dlsym symbol, allow NULL (optional) */
#define RESOLVE_SYM_OPT(node, name, type) do { \
    (node)->name = (type)dlsym((node)->dl_handle, #name); \
} while (0)

/* Copy src into an already-open destination stream (from mkstemps — the
 * destination is created O_EXCL by the libc, never following a pre-planted
 * symlink at a predictable name; CWE-377/59). Closes `out` either way. */
static int copy_file_to(const char *src, FILE *out) {
    FILE *in = fopen(src, "rb");
    if (!in) { fclose(out); return -1; }

    char buf[8192];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) {
            fclose(in); fclose(out); return -1;
        }
    }
    fclose(in);
    if (fclose(out) != 0) return -1;
    return 0;
}

int native_node_init(native_node_t *node, const char *firmware_path, int node_id) {
    memset(node, 0, sizeof(*node));
    node->node_id = node_id;

    /* Copy firmware to a per-node temp file for independent dlopen.
     * mkstemps gives an unpredictable name created O_EXCL (a fixed
     * /tmp/csim_node_<id>_<pid> name was a symlink/planted-library vector
     * on shared hosts). */
#ifdef __APPLE__
    const char *ext = ".dylib";
#else
    const char *ext = ".so";
#endif
    snprintf(node->dl_path, sizeof(node->dl_path),
             "/tmp/csim_node_%d_XXXXXX%s", node_id, ext);
    int tmp_fd = mkstemps(node->dl_path, (int)strlen(ext));
    if (tmp_fd < 0) {
        fprintf(stderr, "native_node: mkstemps(%s) failed\n", node->dl_path);
        return -1;
    }
    node->dl_path_is_temp = true;

    FILE *tmp_out = fdopen(tmp_fd, "wb");
    if (!tmp_out) { close(tmp_fd); unlink(node->dl_path); return -1; }
    if (copy_file_to(firmware_path, tmp_out) != 0) {
        fprintf(stderr, "native_node: failed to copy %s -> %s\n",
                firmware_path, node->dl_path);
        unlink(node->dl_path);
        return -1;
    }

    /* dlopen with RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND for independent
     * symbol spaces.  DEEPBIND ensures the .cooja library's --wrap=printf
     * symbol does not shadow the host process's printf (which would deadlock
     * any printf call from the JS test-engine pthread). */
#ifdef RTLD_DEEPBIND
    node->dl_handle = dlopen(node->dl_path, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
#else
    node->dl_handle = dlopen(node->dl_path, RTLD_NOW | RTLD_LOCAL);
#endif
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
    RESOLVE_SYM(node, simSignalStrength, int *);
    RESOLVE_SYM(node, simLastPacketTimestamp, uint64_t *);
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
    node->clock_drift_ns = 0;
    *node->simInSize = 0;
    *node->simOutSize = 0;
    *node->simLoggedFlag = 0;
    *node->simLoggedLength = 0;

    printf("  Native node %d: calling cooja_init()...\n", node_id);
    int ret = node->cooja_init();
    if (ret != 0) {
        fprintf(stderr, "native_node: cooja_init() returned %d\n", ret);
    }

    /* Do NOT tick here.  cooja_init() only creates the Contiki coroutines;
     * the first cooja_tick() runs the boot-up yield and the second runs
     * Contiki's main() (platform init, netstack, autostart).  COOJA performs
     * both on the mote's first scheduled wakeups, i.e. at its randomized
     * start time, so everything the boot anchors on RTIMER_NOW() (TSCH's
     * slot schedule in particular) is anchored to the mote's real start.
     * Ticking at load time anchored it at t=0 instead and left the mote
     * a full startup delay behind its own schedule (the "!dl-miss
     * TxBeforeTx <start delay>" at the coordinator's first slot).
     * simProcessRunValue=1 marks the pending boot for the wakeup logic. */
    *node->simProcessRunValue = 1;

    node->sim_time_ns = 0;
    printf("  Native node %d: initialized successfully\n", node_id);
    return 0;

resolve_fail:
    /* A required symbol was missing (RESOLVE_SYM) — release the dl handle
     * and the temp copy instead of leaking one pair per failed attempt. */
    dlclose(node->dl_handle);
    node->dl_handle = NULL;
    unlink(node->dl_path);
    node->dl_path[0] = '\0';
    return -1;
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

    if (node->radio_tx_finished) {
        return node->sim_time_ns;
    }

    /* Etimer: expiration time is in mote-local ms */
    if (*node->simEtimerPending) {
        int64_t et_ns = native_node_etimer_expiry_ns(node);
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

    if (node->radio_is_transmitting && node->radio_tx_end_ns < next_ns) {
        next_ns = node->radio_tx_end_ns;
    }

    return next_ns;
}

int64_t native_next_wakeup_ns(const native_node_t *node) {
    /* Only report timer-based wakeups for adaptive stepping.
     * processRunValue and rx_queue are handled by idle-skip in the outer loop. */
    int64_t next_ns = INT64_MAX;
    if (*node->simEtimerPending) {
        int64_t et_ns = native_node_etimer_expiry_ns(node);
        if (et_ns < next_ns) next_ns = et_ns;
    }
    if (*node->simRtimerPending) {
        int64_t rt_ns = (int64_t)(*node->simRtimerNextExpirationTime) * 1000LL;
        if (rt_ns < next_ns) next_ns = rt_ns;
    }
    return next_ns;
}

void native_node_set_clocks(native_node_t *node, int64_t sim_ns) {
    int64_t mote_ns = sim_ns + node->clock_drift_ns;
    *node->simRtimerCurrentTicks = (uint64_t)(sim_ns / 1000LL);
    if (mote_ns > 0)
        *node->simCurrentTime = (uint64_t)(mote_ns / 1000000LL);
}

void native_node_set_clock_drift(native_node_t *node, int64_t drift_ns) {
    int64_t drift_us = drift_ns / 1000LL;
    drift_us -= drift_us % 1000LL;   /* ContikiClock.setDrift: round to ms */
    node->clock_drift_ns = drift_us * 1000LL;
}

int64_t native_node_etimer_expiry_ns(const native_node_t *node) {
    return (int64_t)(*node->simEtimerNextExpirationTime) * 1000000LL
           - node->clock_drift_ns;
}

/*
 * Complete every in-flight frame whose on-air time has ended at
 * node->sim_time_ns, exactly like COOJA's ContikiRadio.signalReceptionEnd():
 * a frame that was not interfered lands in simInDataBuffer (the newest one
 * wins, as in COOJA), an interfered one is dropped, and simReceiving stays
 * 1 only while a frame is still in the air.  Frames still in flight are
 * kept.  Returns true if a good frame was delivered.
 */
bool native_dequeue_rx_frame(native_node_t *node) {
    bool delivered = false;
    native_rx_queue_t *q = &node->rx_queue;
    int kept = 0;
    /* Frames may end out of arrival order (a short frame that started
     * after a long one ends first), so scan the whole queue, complete
     * every ended frame and compact the rest in place. */
    for (int f = 0; f < q->count; f++) {
        int idx = (q->head + f) % NATIVE_RX_QUEUE_SIZE;
        native_pending_frame_t *fr = &q->frames[idx];
        if (fr->end_ns > node->sim_time_ns) {
            int dst = (q->head + kept) % NATIVE_RX_QUEUE_SIZE;
            if (dst != idx) q->frames[dst] = *fr;
            kept++;
            continue;
        }
        if (!fr->collided) {
            memcpy(node->simInDataBuffer, fr->data, (size_t)fr->len);
            *node->simInSize = fr->len;
            delivered = true;
        }
    }
    q->count = kept;
    if (node->simReceiving)
        *node->simReceiving = q->count > 0 ? 1 : 0;
    return delivered;
}

/* Drop everything in the air and in the buffer (ContikiRadio.doActionsAfterTick
 * when simRadioHWOn goes to 0). */
void native_radio_flush_rx(native_node_t *node) {
    node->rx_queue.head = 0;
    node->rx_queue.count = 0;
    if (node->simReceiving) *node->simReceiving = 0;
    *node->simInSize = 0;
}

/* Earliest end-of-frame time among frames in flight, or INT64_MAX. */
int64_t native_rx_next_end_ns(const native_node_t *node) {
    int64_t next = INT64_MAX;
    for (int f = 0; f < node->rx_queue.count; f++) {
        int idx = (node->rx_queue.head + f) % NATIVE_RX_QUEUE_SIZE;
        if (node->rx_queue.frames[idx].end_ns < next)
            next = node->rx_queue.frames[idx].end_ns;
    }
    return next;
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
        native_node_set_clocks(node, node->sim_time_ns);

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
        if (len < 0) len = 0;
        if (len > COOJA_LOG_MAX) len = COOJA_LOG_MAX;   /* runaway guard */
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
    if (node->radio_is_transmitting || *node->simOutSize <= 0) return;

    /* simOutSize is firmware-controlled; clamp before reading the [128]
     * simOutDataBuffer and before the length propagates to receivers. */
    int frame_len = native_clamp_frame_len(*node->simOutSize);
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
    /* Match COOJA ContikiRadio: once simOutSize becomes non-zero, the
     * packet is delivered to the medium immediately, but the mote stays
     * in a transmitting state until the on-air duration has elapsed.
     * simOutSize is only cleared when transmission finishes. */
    node->radio_is_transmitting = true;
    node->radio_tx_finished = false;
    node->radio_tx_end_ns = node->sim_time_ns + ((int64_t)frame_len * 32000LL);
    if (node->radio_tx_end_ns <= node->sim_time_ns) {
        node->radio_tx_end_ns = node->sim_time_ns + 1000LL;
    }
}

void native_deliver_frame(native_node_t *node, const uint8_t *frame, int len,
                          int64_t arrival_ns, int sender_idx) {
    len = native_clamp_frame_len(len);

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
    /* On-air time as COOJA computes it for ContikiRadio: 8*len bits at
     * 250 kbit/s = 32 µs per payload byte, ending exactly when the
     * sender's simOutSize is cleared (radio_tx_end_ns uses the same rule). */
    slot->end_ns = arrival_ns + (int64_t)len * 32000LL;
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
