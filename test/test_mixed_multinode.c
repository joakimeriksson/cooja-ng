/*
 * Mixed-platform multi-node simulation test
 *
 * Runs MSP430 (Tmote Sky / CC2420), ARM (CC2538DK / RF Core), and native
 * Cooja motes in the same simulation. All radios use wire-compatible
 * 802.15.4 frames (4x0x00 preamble + 0x7A SFD + length + payload),
 * enabling cross-platform Contiki-NG networking (RPL, nullnet, etc.).
 *
 * Node type is auto-detected from firmware file extension:
 *   .sky      -> MSP430 (Tmote Sky)
 *   .cc2538dk -> ARM (CC2538DK)
 *   .cooja    -> Native (Cooja mote via dlopen)
 */
#include "msp430_platform.h"
#include "msp430_elf.h"
#include "cc2420.h"
#include "arm_platform.h"
#include "arm_systick.h"
#include "arm_elf.h"
#include "native_node.h"
#include "sim_config.h"
#include "radio_medium.h"
#include "ws_server.h"
#include "sim_state.h"
#include "sim_threads.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#define MAX_NODES 128
#define TIME_STEP_NS      1000000LL  /* 1ms in nanoseconds */
#define DEFAULT_SIM_MS    20000      /* 20 seconds of simulated time */
#define MS_TO_NS          1000000LL

typedef enum { NODE_MSP430, NODE_ARM, NODE_NATIVE } node_type_t;

typedef struct {
    node_type_t type;
    int id;
    char line_buf[256];
    int line_pos;
    union {
        msp430_platform_t msp;
        arm_platform_t arm;
        native_node_t native;
    } plat;
} mixed_node_t;

/* RF byte buffer for deferred delivery */
#define RF_BUF_SIZE 4096
typedef struct {
    uint8_t bytes[RF_BUF_SIZE];
    int count;
} rf_buffer_t;

/* TX frame assembler: detects complete 802.15.4 frames in byte stream.
 * Per-sender state machine: preamble(4×0x00) + SFD(0x7A) + length → complete. */
#define TX_ASM_PREAMBLE  0
#define TX_ASM_LENGTH    1
#define TX_ASM_PAYLOAD   2

typedef struct {
    int state;
    int zero_count;
    int expected_len;   /* PHY length byte value */
    int payload_count;  /* bytes received after length byte */
} tx_frame_asm_t;

/* Returns true when a complete frame has been assembled */
static bool tx_frame_asm_feed(tx_frame_asm_t *a, uint8_t byte) {
    switch (a->state) {
    case TX_ASM_PREAMBLE:
        if (byte == 0x00) {
            a->zero_count++;
        } else if (a->zero_count >= 4 && byte == 0x7A) {
            a->state = TX_ASM_LENGTH;
        } else {
            a->zero_count = 0;
        }
        return false;

    case TX_ASM_LENGTH:
        a->expected_len = byte;
        if (byte < 3 || byte > 127) {
            /* Invalid length, reset */
            a->state = TX_ASM_PREAMBLE;
            a->zero_count = 0;
            return false;
        }
        a->state = TX_ASM_PAYLOAD;
        a->payload_count = 0;
        return false;

    case TX_ASM_PAYLOAD:
        a->payload_count++;
        /* expected_len = PHY length byte = payload + FCS(2).
         * On the wire after length: (expected_len - 2) payload + 2 auto-CRC
         * = expected_len bytes total. */
        if (a->payload_count >= a->expected_len) {
            /* Frame complete — reset for next frame */
            a->state = TX_ASM_PREAMBLE;
            a->zero_count = 0;
            return true;
        }
        return false;
    }
    return false;
}

static void tx_frame_asm_reset(tx_frame_asm_t *a) {
    a->state = TX_ASM_PREAMBLE;
    a->zero_count = 0;
    a->expected_len = 0;
    a->payload_count = 0;
}

/* Per-receiver RX frame queue for emulated nodes */
#define EMU_RX_QUEUE_SIZE 16
#define EMU_RX_FRAME_MAX  160  /* preamble(4)+SFD(1)+len(1)+payload(≤127)+CRC(2) */

typedef struct {
    uint8_t data[EMU_RX_FRAME_MAX];
    int     len;
    int8_t  rssi;
    int64_t arrival_ns;
    int64_t end_ns;
    bool    collided;
} emu_rx_frame_t;

typedef struct {
    emu_rx_frame_t frames[EMU_RX_QUEUE_SIZE];
    int head, count;
} emu_rx_queue_t;

static mixed_node_t nodes[MAX_NODES];
static int num_nodes = 0;
static int verbose = 1;
static rf_buffer_t rf_pending[MAX_NODES];
static tx_frame_asm_t tx_asm[MAX_NODES];     /* per-sender frame assembler */
static emu_rx_queue_t emu_rx_queue[MAX_NODES]; /* per-receiver frame queue */
static int64_t emu_rx_end_ns[MAX_NODES];      /* end time of last RX for each emulated receiver */
static int rf_byte_count = 0;
static int uart_byte_count = 0;
static radio_medium_t radio_medium;
static int64_t current_sim_ns = 0;

/* Statistics counters */
static int stat_rf_frames = 0;       /* total TX frames (all node types) */
static int stat_emu_rx_direct = 0;   /* frames delivered synchronously (auto-ACK works) */
static int stat_emu_rx_queued = 0;   /* frames queued (RXFIFO busy) */
static int stat_emu_rx_drained = 0;  /* frames delivered from queue via mini-step */
static int stat_emu_rx_dropped = 0;  /* frames dropped (queue full) */
static int stat_emu_rx_collided = 0; /* frames dropped due to RF collision */
static int stat_rx_frames_queued = 0;
static int stat_rx_frames_collided = 0;
static int stat_rx_frames_queue_full = 0;
static bool channels_dirty = false;

/* --- Threading support --- */
static int num_threads = 0;  /* 0 = single-threaded (default) */

/* Per-sender RF outgoing buffer (written only by sender's thread) */
typedef struct {
    uint8_t bytes[RF_BUF_SIZE];
    int count;
} rf_outgoing_t;

/* Per-sender frame outgoing buffer for native nodes */
#define MAX_OUTGOING_FRAMES 16
typedef struct {
    uint8_t data[MAX_OUTGOING_FRAMES][128];
    int lengths[MAX_OUTGOING_FRAMES];
    int count;
} frame_outgoing_t;

/* Per-node deferred UART line buffer */
#define MAX_PENDING_LINES 32
typedef struct {
    char lines[MAX_PENDING_LINES][256];
    int node_idx[MAX_PENDING_LINES];
    int node_id[MAX_PENDING_LINES];
    int count;
    int rf_byte_count;      /* local accumulator */
    int uart_byte_count;    /* local accumulator */
    int rf_frame_count;     /* local accumulator */
    bool channels_dirty;    /* local flag */
    int64_t last_tx_ns;     /* last TX time for UI arrows */
} node_thread_state_t;

static rf_outgoing_t rf_outgoing[MAX_NODES];
static frame_outgoing_t frame_outgoing[MAX_NODES];
static node_thread_state_t thread_state[MAX_NODES];

/* Per-node last TX timestamp for UI communication arrows */
static int64_t node_last_tx_ns[MAX_NODES];

/* Per-node start time: node is inactive (no step, no RF) until sim_ns >= start_ns */
static int64_t node_start_ns[MAX_NODES];  /* 0 = start immediately */

static inline int node_active(int idx) {
    return current_sim_ns >= node_start_ns[idx];
}

/* --- WebSocket UI state --- */
static ws_server_t *ui_server = NULL;
#define UI_CONSOLE_LINES 20
#define UI_CONSOLE_LINELEN 256
static char ui_console[MAX_NODES][UI_CONSOLE_LINES][UI_CONSOLE_LINELEN];
static int ui_console_head[MAX_NODES];
static int ui_console_count[MAX_NODES];
/* New lines since last broadcast */
static char ui_console_new[MAX_NODES][UI_CONSOLE_LINES][UI_CONSOLE_LINELEN];
static int ui_console_new_count[MAX_NODES];
static double ui_speed_ratio = 10.0;  /* adjustable from UI, default 10x */
static int ui_paused = 0;             /* play/pause state */

static void ui_message_handler(const char *data, int len, void *userdata) {
    (void)userdata;
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) return;
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cmd && cJSON_IsString(cmd)) {
        if (strcmp(cmd->valuestring, "speed") == 0) {
            cJSON *val = cJSON_GetObjectItem(root, "value");
            if (val && cJSON_IsNumber(val)) {
                double v = val->valuedouble;
                if (v >= 0.1 && v <= 1000.0)
                    ui_speed_ratio = v;
            }
        } else if (strcmp(cmd->valuestring, "pause") == 0) {
            ui_paused = 1;
        } else if (strcmp(cmd->valuestring, "play") == 0) {
            ui_paused = 0;
        } else if (strcmp(cmd->valuestring, "move") == 0) {
            cJSON *jnode = cJSON_GetObjectItem(root, "node");
            cJSON *jx = cJSON_GetObjectItem(root, "x");
            cJSON *jy = cJSON_GetObjectItem(root, "y");
            if (jnode && cJSON_IsNumber(jnode) && jx && cJSON_IsNumber(jx) && jy && cJSON_IsNumber(jy)) {
                int nid = jnode->valueint;
                for (int i = 0; i < num_nodes; i++) {
                    if (nodes[i].id == nid) {
                        radio_medium_set_position(&radio_medium, i, jx->valuedouble, jy->valuedouble);
                        radio_medium_compute_neighbors(&radio_medium);
                        break;
                    }
                }
            }
        }
    }
    cJSON_Delete(root);
}

static void ui_add_console_line(int node_idx, int64_t sim_ns, const char *line) {
    /* Prepend simulation timestamp */
    char stamped[UI_CONSOLE_LINELEN];
    double sim_s = (double)sim_ns / 1e9;
    snprintf(stamped, sizeof(stamped), "[%7.3f] %s", sim_s, line);

    /* Add to ring buffer */
    int slot = (ui_console_head[node_idx] + ui_console_count[node_idx]) % UI_CONSOLE_LINES;
    if (ui_console_count[node_idx] < UI_CONSOLE_LINES)
        ui_console_count[node_idx]++;
    else
        ui_console_head[node_idx] = (ui_console_head[node_idx] + 1) % UI_CONSOLE_LINES;
    strncpy(ui_console[node_idx][slot], stamped, UI_CONSOLE_LINELEN - 1);
    ui_console[node_idx][slot][UI_CONSOLE_LINELEN - 1] = '\0';

    /* Add to new-lines buffer for next broadcast */
    if (ui_console_new_count[node_idx] < UI_CONSOLE_LINES) {
        int ni = ui_console_new_count[node_idx]++;
        strncpy(ui_console_new[node_idx][ni], stamped, UI_CONSOLE_LINELEN - 1);
        ui_console_new[node_idx][ni][UI_CONSOLE_LINELEN - 1] = '\0';
    }
}

/* --- Test scripting state --- */

typedef struct {
    const sim_test_config_t *config;
    int  current_step;
    int  match_count;
    int64_t step_start_ns;
    int  finished;          /* 0=running, 1=pass, -1=fail */
    char fail_reason[512];
} sim_test_state_t;

static sim_test_state_t *active_test = NULL;

static void sim_test_check_line(int node_id, const char *line, int64_t sim_ns) {
    if (!active_test || active_test->finished)
        return;

    const sim_test_step_t *step =
        &active_test->config->steps[active_test->current_step];

    /* Node filter: -1 = any */
    if (step->node >= 0 && step->node != node_id)
        return;

    /* Substring match */
    if (!strstr(line, step->pattern))
        return;

    active_test->match_count++;
    if (active_test->match_count >= step->count) {
        printf("  TEST step %d PASS: \"%s\" matched %d/%d (node %d, %lld ms)\n",
               active_test->current_step, step->pattern,
               active_test->match_count, step->count, node_id,
               (long long)(sim_ns / MS_TO_NS));
        active_test->current_step++;
        active_test->match_count = 0;
        active_test->step_start_ns = sim_ns;

        if (active_test->current_step >= active_test->config->step_count)
            active_test->finished = 1;  /* all steps passed */
    }
}

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

/* --- Type-dispatched accessors --- */

static int64_t node_sim_time_ns(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        return nodes[idx].plat.msp.cpu.sim_time_ns;
    else if (nodes[idx].type == NODE_ARM)
        return nodes[idx].plat.arm.cpu.sim_time_ns;
    else
        return nodes[idx].plat.native.sim_time_ns;
}

static int64_t node_cycles(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        return nodes[idx].plat.msp.cpu.cycles;
    else if (nodes[idx].type == NODE_ARM)
        return nodes[idx].plat.arm.cpu.cycles;
    else
        return nodes[idx].plat.native.sim_time_ns / 1000LL; /* pseudo-cycles: us */
}

static uint32_t node_freq(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        return nodes[idx].plat.msp.cpu.cpu_freq_hz;
    else if (nodes[idx].type == NODE_ARM)
        return nodes[idx].plat.arm.cpu.cpu_freq_hz;
    else
        return 1000000; /* native: 1 MHz pseudo-freq (1 cycle = 1 us) */
}

static int64_t node_instructions(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        return nodes[idx].plat.msp.cpu.instructions;
    else if (nodes[idx].type == NODE_ARM)
        return nodes[idx].plat.arm.cpu.instructions;
    else
        return 0; /* native: not tracked */
}

static const char *node_type_str(int idx) {
    if (nodes[idx].type == NODE_MSP430) return "MSP430";
    if (nodes[idx].type == NODE_ARM) return "ARM";
    return "NATIVE";
}

/* --- RF TX/RX bridging --- */

/* Forward declarations */
static void mixed_deliver_rf_bytes(int idx);
static void step_node_until(int idx, int64_t target);

/* Check available RXFIFO space for an emulated node */
static int emulated_rxfifo_available(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        return 128 - nodes[idx].plat.msp.cc2420.rx_fifo_len;
    else if (nodes[idx].type == NODE_ARM) {
        cc2538_rfcore_t *rf = &nodes[idx].plat.arm.rfcore;
        return RF_RXFIFO_SIZE - (rf->rxfifo_len - rf->rxfifo_rd);
    }
    return 0;
}

/* Push a complete frame into an emulated node's RX queue.
 * Collision detection is NOT done here — it's handled by:
 *   1. emu_rx_end_ns[] check before delivery (multi-sender same-step collision)
 *   2. Interference-range loops marking queued frames as collided
 * Doing overlap checks here would cause false positives (e.g., a data frame
 * and its auto-ACK from the same sender queued to the same receiver). */
static void emu_rx_queue_push(int idx, const uint8_t *data, int len,
                               int8_t rssi, int64_t arrival_ns, int64_t end_ns) {
    emu_rx_queue_t *q = &emu_rx_queue[idx];
    if (q->count >= EMU_RX_QUEUE_SIZE) {
        stat_emu_rx_dropped++;
        return;
    }
    int slot = (q->head + q->count) % EMU_RX_QUEUE_SIZE;
    int copy_len = len < EMU_RX_FRAME_MAX ? len : EMU_RX_FRAME_MAX;
    memcpy(q->frames[slot].data, data, (size_t)copy_len);
    q->frames[slot].len = copy_len;
    q->frames[slot].rssi = rssi;
    q->frames[slot].arrival_ns = arrival_ns;
    q->frames[slot].end_ns = end_ns;
    q->frames[slot].collided = false;
    q->count++;
    stat_emu_rx_queued++;
}

/* Deliver buffered bytes directly to an emulated node's radio */
static void emu_deliver_bytes(int idx, const uint8_t *data, int len,
                               int8_t rssi) {
    if (nodes[idx].type == NODE_MSP430) {
        nodes[idx].plat.msp.cc2420.rx_rssi = rssi;
        for (int j = 0; j < len; j++)
            cc2420_receive_byte(&nodes[idx].plat.msp.cc2420, data[j]);
    } else if (nodes[idx].type == NODE_ARM) {
        nodes[idx].plat.arm.rfcore.rx_rssi = rssi;
        for (int j = 0; j < len; j++)
            cc2538_rfcore_receive_byte(&nodes[idx].plat.arm.rfcore, data[j]);
    }
}

/*
 * Byte-level TX handler: called by CC2420 and CC2538 RF when transmitting
 * individual bytes. Also called by native_check_radio_tx() when converting
 * a native frame to byte-stream for emulated receivers.
 *
 * Strategy: always buffer bytes for emulated receivers. When the per-sender
 * frame assembler signals frame complete, decide per-receiver:
 *   - RXFIFO has room -> deliver synchronously (auto-ACK works)
 *   - RXFIFO full -> queue for later delivery via mini-step
 *
 * Re-entrancy guard: auto-ACK bytes from synchronous delivery re-enter
 * this handler. Those bytes are buffered and flushed after the outer
 * delivery completes to prevent byte interleaving.
 */
static int rf_tx_depth = 0;

static void mixed_rf_tx_handler(void *user_data, uint8_t byte) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    rf_byte_count++;
    channels_dirty = true;
    node_last_tx_ns[sender_idx] = current_sim_ns;

    if (rf_tx_depth > 0) {
        /* Re-entrant call (auto-ACK from a receiver). Buffer for all
         * eligible neighbors — flushed after outer delivery completes. */
        for (int i = 0; i < num_nodes; i++) {
            if (&nodes[i] == sender) continue;
            if (!node_active(i)) continue;
            if (sender->type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
                continue;
            if (!radio_medium_filter_byte(&radio_medium, sender_idx, i, byte))
                continue;
            if (nodes[i].type == NODE_NATIVE) {
                native_rx_assembler_feed(&nodes[i].plat.native, byte);
            } else {
                rf_buffer_t *buf = &rf_pending[i];
                if (buf->count < RF_BUF_SIZE)
                    buf->bytes[buf->count++] = byte;
            }
        }
        return;
    }

    /* Outer (non-reentrant) call: buffer byte for each emulated receiver,
     * feed native assemblers directly. */
    for (int i = 0; i < num_nodes; i++) {
        if (&nodes[i] == sender) continue;
        if (!node_active(i)) continue;
        if (sender->type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
            continue;
        if (!radio_medium_filter_byte(&radio_medium, sender_idx, i, byte))
            continue;
        if (nodes[i].type == NODE_NATIVE) {
            native_rx_assembler_feed(&nodes[i].plat.native, byte);
        } else {
            rf_buffer_t *buf = &rf_pending[i];
            if (buf->count < RF_BUF_SIZE)
                buf->bytes[buf->count++] = byte;
        }
    }

    /* Feed the per-sender frame assembler */
    if (!tx_frame_asm_feed(&tx_asm[sender_idx], byte))
        return;  /* frame not yet complete */

    /* Frame complete — snapshot each receiver's frame data, then deliver.
     * Snapshot is necessary because synchronous delivery triggers auto-ACK,
     * which re-enters and appends ACK bytes to OTHER receivers' rf_pending.
     * Without snapshot, later receivers would see corrupted (interleaved) data. */
    stat_rf_frames++;
    rf_tx_depth++;

    /* Snapshot: copy frame data and RSSI for each emulated receiver */
    static uint8_t frame_snap[MAX_NODES][EMU_RX_FRAME_MAX];
    static int frame_snap_len[MAX_NODES];
    static int8_t frame_snap_rssi[MAX_NODES];
    int snap_count = 0;
    static int snap_indices[MAX_NODES];

    for (int i = 0; i < num_nodes; i++) {
        rf_buffer_t *buf = &rf_pending[i];
        if (buf->count == 0 || nodes[i].type == NODE_NATIVE)
            continue;
        int copy_len = buf->count < EMU_RX_FRAME_MAX ? buf->count : EMU_RX_FRAME_MAX;
        memcpy(frame_snap[i], buf->bytes, (size_t)copy_len);
        frame_snap_len[i] = copy_len;
        frame_snap_rssi[i] = radio_medium_get_rssi(&radio_medium, sender_idx, i);
        snap_indices[snap_count++] = i;
        buf->count = 0;  /* Clear before delivery to isolate from re-entrancy */
    }

    /* Deliver from snapshots — auto-ACK re-entrancy now writes to
     * empty rf_pending buffers, not our snapshot data.
     * Flush ACK bytes after EACH delivery to prevent concatenation of
     * multiple ACK frames in the same rf_pending buffer. */
    for (int s = 0; s < snap_count; s++) {
        int i = snap_indices[s];
        /* Collision window: one time step. Frames are delivered synchronously
         * (all bytes at once), so the collision granularity is the time step.
         * Using frame_len * 32000 would span multiple steps and cause false
         * positives when legitimate frames arrive in subsequent steps. */
        int64_t tx_start = current_sim_ns;
        int64_t tx_end = current_sim_ns + TIME_STEP_NS;

        /* Collision check: does this frame overlap with a previously
         * delivered frame on this receiver? */
        if (tx_start < emu_rx_end_ns[i]) {
            /* Collision with previously delivered frame — drop this one */
            stat_emu_rx_collided++;
            continue;
        }

        int frame_payload = (frame_snap_len[i] > 5) ? frame_snap[i][5] : 0;
        if (emulated_rxfifo_available(i) >= frame_payload + 1) {
            emu_deliver_bytes(i, frame_snap[i], frame_snap_len[i],
                              frame_snap_rssi[i]);
            stat_emu_rx_direct++;
            emu_rx_end_ns[i] = tx_end;
        } else {
            emu_rx_queue_push(i, frame_snap[i], frame_snap_len[i],
                              frame_snap_rssi[i], tx_start, tx_end);
            emu_rx_end_ns[i] = tx_end;
        }

        /* Flush auto-ACK bytes generated by this delivery.
         * Each ACK is a single frame — flush immediately to prevent
         * multiple ACKs concatenating in the same rf_pending buffer.
         * ACK timing is NOT tracked in emu_rx_end_ns — auto-ACK is
         * a response to the data frame and shouldn't block future frames. */
        for (int j = 0; j < num_nodes; j++) {
            if (rf_pending[j].count > 0 && nodes[j].type != NODE_NATIVE) {
                int ack_payload = (rf_pending[j].count > 5) ? rf_pending[j].bytes[5] : 0;
                if (emulated_rxfifo_available(j) >= ack_payload + 1)
                    emu_deliver_bytes(j, rf_pending[j].bytes, rf_pending[j].count, -50);
                else
                    emu_rx_queue_push(j, rf_pending[j].bytes, rf_pending[j].count,
                                      -50, tx_start, tx_end);
                rf_pending[j].count = 0;
            }
        }
    }

    /* Interference check for emulated nodes: mark existing queued frames
     * as collided if a TX-range or interference-range neighbor transmitted. */
    if (radio_medium.type != RADIO_MEDIUM_NONE) {
        neighbor_list_t *inl = &radio_medium.interference_neighbors[sender_idx];
        int64_t int_start = current_sim_ns;
        int64_t int_end = current_sim_ns + TIME_STEP_NS;
        for (int n = 0; n < inl->count; n++) {
            int i = inl->neighbors[n];
            if (nodes[i].type == NODE_NATIVE) continue;  /* native handled elsewhere */
            emu_rx_queue_t *q = &emu_rx_queue[i];
            for (int f = 0; f < q->count; f++) {
                int fi = (q->head + f) % EMU_RX_QUEUE_SIZE;
                emu_rx_frame_t *existing = &q->frames[fi];
                if (int_start < existing->end_ns && existing->arrival_ns < int_end) {
                    if (!existing->collided) stat_emu_rx_collided++;
                    existing->collided = true;
                }
            }
        }
    }

    rf_tx_depth--;
}

/*
 * Frame-level TX handler: called by native_check_radio_tx() when a native
 * node transmits. Delivers the frame directly to other native nodes and
 * converts to byte-stream for emulated nodes.
 */
static void mixed_rf_frame_handler(void *user_data, const uint8_t *frame, int len) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);

    stat_rf_frames++;
    channels_dirty = true;  /* TX happened, channels may have changed */
    node_last_tx_ns[sender_idx] = current_sim_ns;

    if (radio_medium.type != RADIO_MEDIUM_NONE) {
        /* UDGM: iterate precomputed TX-range neighbors */
        neighbor_list_t *nl = &radio_medium.neighbors[sender_idx];
        for (int n = 0; n < nl->count; n++) {
            int i = nl->neighbors[n];
            if (nodes[i].type == NODE_NATIVE) {
                if (radio_medium_filter_frame(&radio_medium, sender_idx, i)) {
                    native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                    if (q->count >= NATIVE_RX_QUEUE_SIZE)
                        stat_rx_frames_queue_full++;
                    native_deliver_frame(&nodes[i].plat.native, frame, len,
                                         current_sim_ns, sender_idx);
                    stat_rx_frames_queued++;
                }
            }
            /* Native-to-emulated: handled via rf_tx_callback (byte stream) */
        }

        /* Interference-range neighbors: mark overlapping frames as collided */
        neighbor_list_t *inl = &radio_medium.interference_neighbors[sender_idx];
        int64_t tx_start = current_sim_ns;
        int64_t tx_end = tx_start + (int64_t)(len + 6) * 32000LL;
        for (int n = 0; n < inl->count; n++) {
            int i = inl->neighbors[n];
            if (nodes[i].type == NODE_NATIVE) {
                native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                for (int f = 0; f < q->count; f++) {
                    int idx = (q->head + f) % NATIVE_RX_QUEUE_SIZE;
                    native_pending_frame_t *existing = &q->frames[idx];
                    if (tx_start < existing->end_ns &&
                        existing->arrival_ns < tx_end) {
                        if (!existing->collided) stat_rx_frames_collided++;
                        existing->collided = true;
                    }
                }
            } else {
                /* Emulated receiver: mark queued frames as collided */
                emu_rx_queue_t *q = &emu_rx_queue[i];
                for (int f = 0; f < q->count; f++) {
                    int fi = (q->head + f) % EMU_RX_QUEUE_SIZE;
                    emu_rx_frame_t *existing = &q->frames[fi];
                    if (tx_start < existing->end_ns &&
                        existing->arrival_ns < tx_end) {
                        if (!existing->collided) stat_emu_rx_collided++;
                        existing->collided = true;
                    }
                }
            }
        }
    } else {
        /* NONE: deliver to all other nodes */
        for (int i = 0; i < num_nodes; i++) {
            if (&nodes[i] != sender && nodes[i].type == NODE_NATIVE) {
                native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                if (q->count >= NATIVE_RX_QUEUE_SIZE)
                    stat_rx_frames_queue_full++;
                native_deliver_frame(&nodes[i].plat.native, frame, len,
                                     current_sim_ns, sender_idx);
                stat_rx_frames_queued++;
            }
        }
    }
}

/* Deliver buffered RF bytes to a native node's assembler.
 * Only called for native nodes — emulated nodes' bytes stay in rf_pending
 * until the per-sender frame assembler detects a complete frame. */
static void mixed_deliver_rf_bytes(int idx) {
    rf_buffer_t *buf = &rf_pending[idx];
    if (buf->count == 0) return;
    emu_deliver_bytes(idx, buf->bytes, buf->count, 0);
    buf->count = 0;
}

/*
 * Drain queued RX frames for an emulated node. Delivers one frame at a time
 * with CPU mini-steps between to let the ISR read the RXFIFO.
 */
static void emu_rx_queue_drain(int idx) {
    emu_rx_queue_t *q = &emu_rx_queue[idx];
    while (q->count > 0) {
        emu_rx_frame_t *f = &q->frames[q->head];

        /* Skip collided frames (like native_dequeue_rx_frame does) */
        if (f->collided) {
            q->head = (q->head + 1) % EMU_RX_QUEUE_SIZE;
            q->count--;
            continue;
        }

        /* PHY length byte is at data[5] (after 4×preamble + SFD) */
        int frame_payload = (f->len > 5) ? f->data[5] : 0;
        if (emulated_rxfifo_available(idx) < frame_payload + 1)
            break;  /* RXFIFO still busy, try next time step */

        /* Reset collision tracking before each drain delivery.
         * Delivering a queued frame may trigger auto-ACK, which goes through
         * the frame assembler and sets emu_rx_end_ns on the ACK receiver.
         * Without reset, sequential drain deliveries would falsely collide
         * (each auto-ACK would see emu_rx_end_ns set by the previous one). */
        memset(emu_rx_end_ns, 0, sizeof(emu_rx_end_ns));

        /* Deliver frame bytes to radio */
        emu_deliver_bytes(idx, f->data, f->len, f->rssi);
        stat_emu_rx_drained++;

        q->head = (q->head + 1) % EMU_RX_QUEUE_SIZE;
        q->count--;

        /* Mini-step CPU to process RX interrupt + read RXFIFO.
         * 5000 cycles at 32 MHz = ~156us, enough for ISR. */
        if (q->count > 0)
            step_node_until(idx, node_cycles(idx) + 5000);
    }
}

/* --- UART callback --- */

static void mixed_uart_callback(void *user_data, uint8_t byte) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    uart_byte_count++;

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        int nidx = (int)(node - nodes);
        int64_t ns = node_sim_time_ns(nidx);
        if (verbose)
            printf("  %7.3f [Node %d/%s] %s\n", (double)ns / 1e9,
                   node->id, node_type_str(nidx), node->line_buf);
        sim_test_check_line(node->id, node->line_buf, ns);
        if (ui_server)
            ui_add_console_line(nidx, ns, node->line_buf);
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
    }
}

/* --- Threaded callbacks (write only to sender's own buffers) --- */

/*
 * Threaded RF TX handler: buffers bytes in rf_outgoing[sender] instead of
 * delivering directly. No cross-node writes, no radio_medium access.
 */
static void threaded_rf_tx_handler(void *user_data, uint8_t byte) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    node_thread_state_t *ts = &thread_state[sender_idx];
    ts->rf_byte_count++;
    ts->channels_dirty = true;
    ts->last_tx_ns = current_sim_ns;

    rf_outgoing_t *out = &rf_outgoing[sender_idx];
    if (out->count < RF_BUF_SIZE)
        out->bytes[out->count++] = byte;
}

/*
 * Threaded frame handler: buffers frames in frame_outgoing[sender].
 * No cross-node writes.
 */
static void threaded_rf_frame_handler(void *user_data, const uint8_t *frame, int len) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    node_thread_state_t *ts = &thread_state[sender_idx];
    ts->rf_frame_count++;
    ts->channels_dirty = true;
    ts->last_tx_ns = current_sim_ns;

    frame_outgoing_t *out = &frame_outgoing[sender_idx];
    if (out->count < MAX_OUTGOING_FRAMES && len <= 128) {
        memcpy(out->data[out->count], frame, (size_t)len);
        out->lengths[out->count] = len;
        out->count++;
    }
}

/*
 * Threaded UART callback: buffers completed lines for deferred processing.
 * Still accumulates bytes in node->line_buf (per-node, no contention).
 */
static void threaded_uart_callback(void *user_data, uint8_t byte) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    int idx = (int)(node - nodes);
    node_thread_state_t *ts = &thread_state[idx];
    ts->uart_byte_count++;

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        if (ts->count < MAX_PENDING_LINES) {
            strncpy(ts->lines[ts->count], node->line_buf, 255);
            ts->lines[ts->count][255] = '\0';
            ts->node_idx[ts->count] = idx;
            ts->node_id[ts->count] = node->id;
            ts->count++;
        }
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
    }
}

/*
 * Sequential phase: distribute rf_outgoing[sender] to receivers via
 * radio_medium filtering, then deliver to rf_pending[] / native assemblers.
 */
static void distribute_rf_outgoing(void) {
    for (int sender = 0; sender < num_nodes; sender++) {
        /* Distribute byte-stream RF with frame assembly */
        rf_outgoing_t *out = &rf_outgoing[sender];
        if (out->count > 0) {
            /* NOTE: do NOT reset tx_asm[sender] here — frame assembly state
             * must persist across time steps since frames may span multiple
             * steps (at 250kbps, a 100-byte frame takes ~3.2ms > 1ms step). */
            for (int b = 0; b < out->count; b++) {
                uint8_t byte = out->bytes[b];
                for (int i = 0; i < num_nodes; i++) {
                    if (i == sender) continue;
                    if (!node_active(i)) continue;
                    if (nodes[sender].type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
                        continue;
                    if (!radio_medium_filter_byte(&radio_medium, sender, i, byte))
                        continue;
                    if (nodes[i].type == NODE_NATIVE) {
                        native_rx_assembler_feed(&nodes[i].plat.native, byte);
                    } else {
                        rf_buffer_t *buf = &rf_pending[i];
                        if (buf->count < RF_BUF_SIZE)
                            buf->bytes[buf->count++] = byte;
                    }
                }

                /* Check if frame is complete */
                if (tx_frame_asm_feed(&tx_asm[sender], byte)) {
                    stat_rf_frames++;
                    /* Deliver or queue each emulated receiver's buffered frame */
                    for (int i = 0; i < num_nodes; i++) {
                        rf_buffer_t *buf = &rf_pending[i];
                        if (buf->count == 0 || nodes[i].type == NODE_NATIVE)
                            continue;
                        int64_t tx_start = current_sim_ns;
                        int64_t tx_end = current_sim_ns + TIME_STEP_NS;

                        /* Collision check against previously delivered frame */
                        if (tx_start < emu_rx_end_ns[i]) {
                            stat_emu_rx_collided++;
                            buf->count = 0;
                            continue;
                        }

                        int8_t rssi = radio_medium_get_rssi(&radio_medium, sender, i);
                        int frame_payload = (buf->count > 5) ? buf->bytes[5] : 0;
                        if (emulated_rxfifo_available(i) >= frame_payload + 1) {
                            emu_deliver_bytes(i, buf->bytes, buf->count, rssi);
                            stat_emu_rx_direct++;
                            emu_rx_end_ns[i] = tx_end;
                        } else {
                            emu_rx_queue_push(i, buf->bytes, buf->count, rssi,
                                              tx_start, tx_end);
                            emu_rx_end_ns[i] = tx_end;
                        }
                        buf->count = 0;
                    }

                    /* Interference check for emulated nodes */
                    if (radio_medium.type != RADIO_MEDIUM_NONE) {
                        neighbor_list_t *inl = &radio_medium.interference_neighbors[sender];
                        int64_t int_start = current_sim_ns;
                        int64_t int_end = current_sim_ns + TIME_STEP_NS;
                        for (int n = 0; n < inl->count; n++) {
                            int i = inl->neighbors[n];
                            if (nodes[i].type == NODE_NATIVE) continue;
                            emu_rx_queue_t *q = &emu_rx_queue[i];
                            for (int f = 0; f < q->count; f++) {
                                int fi = (q->head + f) % EMU_RX_QUEUE_SIZE;
                                emu_rx_frame_t *existing = &q->frames[fi];
                                if (int_start < existing->end_ns &&
                                    existing->arrival_ns < int_end) {
                                    if (!existing->collided) stat_emu_rx_collided++;
                                    existing->collided = true;
                                }
                            }
                        }
                    }
                }
            }
            out->count = 0;
        }

        /* Distribute complete frames (native TX) */
        frame_outgoing_t *fout = &frame_outgoing[sender];
        for (int f = 0; f < fout->count; f++) {
            uint8_t *frame = fout->data[f];
            int len = fout->lengths[f];

            if (radio_medium.type != RADIO_MEDIUM_NONE) {
                neighbor_list_t *nl = &radio_medium.neighbors[sender];
                for (int n = 0; n < nl->count; n++) {
                    int i = nl->neighbors[n];
                    if (nodes[i].type == NODE_NATIVE) {
                        if (radio_medium_filter_frame(&radio_medium, sender, i)) {
                            native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                            if (q->count >= NATIVE_RX_QUEUE_SIZE)
                                stat_rx_frames_queue_full++;
                            native_deliver_frame(&nodes[i].plat.native, frame, len,
                                                 current_sim_ns, sender);
                            stat_rx_frames_queued++;
                        }
                    }
                }
                /* Interference collision detection */
                neighbor_list_t *inl = &radio_medium.interference_neighbors[sender];
                int64_t tx_start = current_sim_ns;
                int64_t tx_end = tx_start + (int64_t)(len + 6) * 32000LL;
                for (int n = 0; n < inl->count; n++) {
                    int i = inl->neighbors[n];
                    if (nodes[i].type == NODE_NATIVE) {
                        native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                        for (int ff = 0; ff < q->count; ff++) {
                            int idx = (q->head + ff) % NATIVE_RX_QUEUE_SIZE;
                            native_pending_frame_t *existing = &q->frames[idx];
                            if (tx_start < existing->end_ns &&
                                existing->arrival_ns < tx_end) {
                                if (!existing->collided) stat_rx_frames_collided++;
                                existing->collided = true;
                            }
                        }
                    } else {
                        emu_rx_queue_t *q = &emu_rx_queue[i];
                        for (int ff = 0; ff < q->count; ff++) {
                            int fi = (q->head + ff) % EMU_RX_QUEUE_SIZE;
                            emu_rx_frame_t *existing = &q->frames[fi];
                            if (tx_start < existing->end_ns &&
                                existing->arrival_ns < tx_end) {
                                if (!existing->collided) stat_emu_rx_collided++;
                                existing->collided = true;
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < num_nodes; i++) {
                    if (i != sender && nodes[i].type == NODE_NATIVE) {
                        native_rx_queue_t *q = &nodes[i].plat.native.rx_queue;
                        if (q->count >= NATIVE_RX_QUEUE_SIZE)
                            stat_rx_frames_queue_full++;
                        native_deliver_frame(&nodes[i].plat.native, frame, len,
                                             current_sim_ns, sender);
                        stat_rx_frames_queued++;
                    }
                }
            }
        }
        fout->count = 0;
    }
}

/*
 * Sequential phase: flush deferred UART lines — printf, test check, UI console.
 * Also merge per-node counters into globals.
 */
static void flush_pending_output(void) {
    for (int i = 0; i < num_nodes; i++) {
        node_thread_state_t *ts = &thread_state[i];

        /* Merge counters */
        rf_byte_count += ts->rf_byte_count;
        uart_byte_count += ts->uart_byte_count;
        stat_rf_frames += ts->rf_frame_count;
        if (ts->channels_dirty) channels_dirty = true;

        if (ts->last_tx_ns > node_last_tx_ns[i])
            node_last_tx_ns[i] = ts->last_tx_ns;
        ts->rf_byte_count = 0;
        ts->uart_byte_count = 0;
        ts->rf_frame_count = 0;
        ts->channels_dirty = false;
        ts->last_tx_ns = 0;

        /* Flush buffered UART lines */
        for (int l = 0; l < ts->count; l++) {
            int nidx = ts->node_idx[l];
            int nid = ts->node_id[l];
            if (verbose)
                printf("  %7.3f [Node %d/%s] %s\n", (double)current_sim_ns / 1e9,
                       nid, node_type_str(nidx), ts->lines[l]);
            sim_test_check_line(nid, ts->lines[l], current_sim_ns);
            if (ui_server)
                ui_add_console_line(nidx, current_sim_ns, ts->lines[l]);
        }
        ts->count = 0;
    }
}

/* --- Detect node type from firmware extension --- */

static node_type_t detect_node_type(const char *path) {
    const char *dot = strrchr(path, '.');
    if (dot && strcmp(dot, ".cc2538dk") == 0)
        return NODE_ARM;
    if (dot && strcmp(dot, ".cooja") == 0)
        return NODE_NATIVE;
    return NODE_MSP430;  /* default to MSP430 (.sky or other) */
}

/* --- MSP430 node initialization --- */

static int init_msp430_node(int idx, const char *firmware_path, int node_id) {
    mixed_node_t *node = &nodes[idx];
    msp430_platform_t *plat = &node->plat.msp;

    const msp430_platform_config_t *pcfg = msp430_platform_find("sky");
    if (!pcfg) { fprintf(stderr, "Platform 'sky' not found\n"); return -1; }

    msp430_platform_init(plat, pcfg);
    if (msp430_load_elf(&plat->cpu, firmware_path) != 0) {
        fprintf(stderr, "Cannot load firmware: %s\n", firmware_path);
        msp430_platform_destroy(plat);
        return -1;
    }

    /* Patch ds2411_init() to RET immediately */
    uint32_t ds2411_init_addr = msp430_elf_find_symbol(firmware_path, "ds2411_init");
    if (ds2411_init_addr != 0) {
        plat->cpu.memory[ds2411_init_addr]     = 0x30;
        plat->cpu.memory[ds2411_init_addr + 1] = 0x41;
        printf("  Patched ds2411_init at 0x%04x to RET\n", ds2411_init_addr);
    }

    msp430_platform_set_console(plat, mixed_uart_callback, node);
    cc2420_set_rf_listener(&plat->cc2420, mixed_rf_tx_handler, node);
    plat->cc2420.node_id = node_id;
    msp430_cpu_reset(&plat->cpu);

    /* Run past crt0 to main, then patch ds2411_id */
    uint32_t main_addr = msp430_elf_find_symbol(firmware_path, "main");
    if (main_addr != 0) {
        for (int s = 0; s < 200000; s++) {
            msp430_step(&plat->cpu, 1);
            if ((plat->cpu.reg[0] & 0xFFFF) == (main_addr & 0xFFFF))
                break;
        }
        printf("  Node %d [MSP430]: ran to main (0x%04x) in %lld cycles\n",
               node_id, main_addr, (long long)plat->cpu.cycles);
    } else {
        msp430_step(&plat->cpu, 50000);
    }

    /* Patch ds2411_id with unique address */
    uint32_t ds2411_addr = msp430_elf_find_symbol(firmware_path, "ds2411_id");
    if (ds2411_addr != 0) {
        uint8_t *id = plat->cpu.memory + ds2411_addr;
        id[0] = 0x00; id[1] = 0x12; id[2] = 0x74; id[3] = (uint8_t)node_id;
        id[4] = 0x00; id[5] = 0x00; id[6] = 0x00; id[7] = (uint8_t)node_id;
        printf("  Patched ds2411_id at 0x%04x for node %d: ", ds2411_addr, node_id);
        for (int i = 0; i < 8; i++) printf("%02x%s", id[i], i < 7 ? ":" : "\n");
    }

    printf("  Node %d [MSP430] initialized: PC=0x%04x SP=0x%04x\n",
           node_id, plat->cpu.reg[0], plat->cpu.reg[1]);
    return 0;
}

/* --- ARM node initialization --- */

static int init_arm_node(int idx, const char *firmware_path, int node_id) {
    mixed_node_t *node = &nodes[idx];
    arm_platform_t *plat = &node->plat.arm;

    const arm_platform_config_t *pcfg = arm_platform_find("cc2538dk");
    if (!pcfg) { fprintf(stderr, "Platform 'cc2538dk' not found\n"); return -1; }

    arm_platform_init(plat, pcfg);
    if (arm_load_elf(&plat->cpu, firmware_path) != 0) {
        fprintf(stderr, "Cannot load firmware: %s\n", firmware_path);
        arm_platform_destroy(plat);
        return -1;
    }

    arm_platform_set_console(plat, mixed_uart_callback, node);
    cc2538_rfcore_set_tx_callback(&plat->rfcore, mixed_rf_tx_handler, node);
    plat->rfcore.node_id = node_id;

    /* Seed RFRND and sleep timer uniquely per node.
     * Use bit mixing so close IDs produce divergent sequences. */
    {
        uint32_t h = (uint32_t)node_id;
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;   /* xorshift32 */
        h *= 2654435761u;                             /* Knuth multiplicative hash */
        h ^= h >> 16;
        plat->rfcore.rfrnd_state = h ? h : 0xDEADBEEF;

        if (verbose)
            printf("  Node %d: rfrnd_seed=0x%08x\n", node_id, h);
    }

    /* Set unique IEEE 64-bit extended address: 00:12:74:XX:00:00:00:XX
     * Stored in little-endian (reversed) for ieee_addr_cpy_to() reversal */
    uint8_t unique_addr[8] = { (uint8_t)node_id, 0x00, 0x00, 0x00,
                                (uint8_t)node_id, 0x74, 0x12, 0x00 };
    memcpy(plat->rfcore.ext_addr, unique_addr, 8);

    arm_cpu_reset(&plat->cpu);

    uint32_t main_addr = arm_elf_find_symbol(firmware_path, "main") & ~1u;
    if (main_addr) {
        for (int s = 0; s < 500000; s++) {
            arm_step(&plat->cpu, 1);
            if ((plat->cpu.reg[ARM_PC] & ~1u) == main_addr)
                break;
        }
        printf("  Node %d [ARM]: ran to main (0x%08x) in %lld cycles\n",
               node_id, main_addr, (long long)plat->cpu.cycles);

        /* Patch linkaddr_node_addr */
        uint32_t la_addr = arm_elf_find_symbol(firmware_path, "linkaddr_node_addr");
        if (la_addr) {
            la_addr &= ~1u;
            uint8_t la_bytes[8] = { 0x00, 0x12, 0x74, (uint8_t)node_id,
                                     0x00, 0x00, 0x00, (uint8_t)node_id };
            for (int b = 0; b < 8; b++)
                arm_write8(&plat->cpu, la_addr + (uint32_t)b, la_bytes[b]);
            printf("  Node %d [ARM]: patched linkaddr_node_addr at 0x%08x\n",
                   node_id, la_addr);
        }
    } else {
        printf("  Node %d [ARM]: 'main' symbol not found, skipping crt0 run\n", node_id);
    }

    printf("  Node %d [ARM] initialized: PC=0x%08x SP=0x%08x\n",
           node_id, plat->cpu.reg[ARM_PC], plat->cpu.reg[ARM_SP]);
    return 0;
}

/* --- Native node initialization --- */

static int init_native_node(int idx, const char *firmware_path, int node_id) {
    mixed_node_t *node = &nodes[idx];
    native_node_t *nat = &node->plat.native;

    if (native_node_init(nat, firmware_path, node_id) != 0)
        return -1;

    /* Set up UART/log callback */
    nat->log_callback = mixed_uart_callback;
    nat->log_callback_data = node;

    /* Set up RF callbacks: byte-stream for emulated, frame for native */
    nat->rf_tx_callback = mixed_rf_tx_handler;
    nat->rf_tx_callback_data = node;
    nat->rf_frame_callback = mixed_rf_frame_handler;
    nat->rf_frame_callback_data = node;

    /* Reset the RX assembler */
    native_rx_assembler_reset(&nat->rx_asm);

    printf("  Node %d [NATIVE] initialized\n", node_id);
    return 0;
}

/* --- Top-level node init (dispatches by type) --- */

static int init_node(int idx, const char *firmware_path, int node_id) {
    mixed_node_t *node = &nodes[idx];
    memset(node, 0, sizeof(*node));
    node->type = detect_node_type(firmware_path);
    node->id = node_id;
    memset(&rf_pending[idx], 0, sizeof(rf_pending[idx]));
    memset(&emu_rx_queue[idx], 0, sizeof(emu_rx_queue[idx]));
    emu_rx_end_ns[idx] = 0;
    tx_frame_asm_reset(&tx_asm[idx]);

    printf("Initializing node %d (%s) as %s...\n", node_id, firmware_path,
           node->type == NODE_MSP430 ? "MSP430/Sky" :
           node->type == NODE_ARM ? "ARM/CC2538DK" : "Native/Cooja");

    if (node->type == NODE_MSP430)
        return init_msp430_node(idx, firmware_path, node_id);
    else if (node->type == NODE_ARM)
        return init_arm_node(idx, firmware_path, node_id);
    else
        return init_native_node(idx, firmware_path, node_id);
}

/* --- Destroy node --- */

static void destroy_node(int idx) {
    if (nodes[idx].type == NODE_MSP430)
        msp430_platform_destroy(&nodes[idx].plat.msp);
    else if (nodes[idx].type == NODE_ARM)
        arm_platform_destroy(&nodes[idx].plat.arm);
    else
        native_node_destroy(&nodes[idx].plat.native);
}

/* --- Simulation step for one node --- */

static void step_node_until(int idx, int64_t target) {
    if (nodes[idx].type == NODE_MSP430)
        msp430_step_until(&nodes[idx].plat.msp.cpu, target);
    else if (nodes[idx].type == NODE_ARM)
        arm_step_until(&nodes[idx].plat.arm.cpu, target);
    else
        native_step_until_ns(&nodes[idx].plat.native, target);
}

/* Check if a native node has pending work at the given sim time */
static bool native_has_pending_work(native_node_t *node, int64_t sim_ns) {
    if (*node->simInSize > 0) return true;  /* frame ready for delivery */
    if (node->rx_queue.count > 0) return true;  /* queued frames waiting */
    if (*node->simProcessRunValue) return true;
    if (*node->simEtimerPending) {
        int64_t et_ns = (int64_t)(*node->simEtimerNextExpirationTime) * 1000000LL;
        if (et_ns <= sim_ns) return true;
    }
    if (*node->simRtimerPending) {
        int64_t rt_ns = (int64_t)(*node->simRtimerNextExpirationTime) * 1000LL;
        if (rt_ns <= sim_ns) return true;
    }
    return false;
}

/*
 * Thread pool step callback: steps a single node to the target sim time.
 * user_data carries the sim_ns cast to void*.
 * RF delivery and native frame dequeue happen in the sequential phase,
 * so this only does the CPU stepping.
 */
static void threaded_step_node(int idx, void *user_data) {
    int64_t sim_ns = (int64_t)(intptr_t)user_data;

    if (!node_active(idx)) return;

    if (nodes[idx].type == NODE_NATIVE) {
        if (!native_has_pending_work(&nodes[idx].plat.native, sim_ns)) {
            nodes[idx].plat.native.sim_time_ns = sim_ns;
            return;
        }
        step_node_until(idx, sim_ns);
        /* Process additional queued frames within this time step */
        while (*nodes[idx].plat.native.simInSize == 0 &&
               nodes[idx].plat.native.rx_queue.count > 0 &&
               nodes[idx].plat.native.sim_time_ns < sim_ns) {
            native_dequeue_rx_frame(&nodes[idx].plat.native);
            step_node_until(idx, sim_ns);
        }
    } else {
        int64_t delta_ns = sim_ns - node_sim_time_ns(idx);
        if (delta_ns > 0) {
            int64_t target_cycle;
            if (nodes[idx].type == NODE_MSP430)
                target_cycle = node_cycles(idx) + msp430_ns_to_cycles(delta_ns, node_freq(idx));
            else
                target_cycle = node_cycles(idx) + arm_ns_to_cycles(delta_ns, node_freq(idx));
            step_node_until(idx, target_cycle);
        }
    }
}

/* --- Channel synchronization --- */

static void sync_node_channels(void) {
    for (int i = 0; i < num_nodes; i++) {
        int ch = -1;
        if (nodes[i].type == NODE_MSP430) {
            /* CC2420: FSCTRL register (0x18), channel = (FREQ[9:0] - 357) / 5 + 11 */
            uint16_t fsctrl = nodes[i].plat.msp.cc2420.registers[CC2420_REG_FSCTRL];
            int freq = fsctrl & 0x3FF;
            if (freq >= 357)
                ch = (freq - 357) / 5 + 11;
        } else if (nodes[i].type == NODE_ARM) {
            /* CC2538: channel already computed on FREQCTRL write */
            ch = nodes[i].plat.arm.rfcore.channel;
        } else {
            /* Native: simRadioChannel pointer */
            if (nodes[i].plat.native.simRadioChannel)
                ch = *nodes[i].plat.native.simRadioChannel;
        }
        radio_medium_set_channel(&radio_medium, i, ch);
    }
}

/* --- Main entry point --- */

/* Check if path ends with .json */
static int is_json_file(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot && strcmp(dot, ".json") == 0;
}

int run_mixed_multinode_test(int argc, char **argv) {
    static const char *firmware_paths[MAX_NODES] = { NULL };
    static char firmware_bufs[MAX_NODES][256]; /* storage for config-loaded paths */
    int firmware_count = 0;
    int sim_ms = DEFAULT_SIM_MS;
    int node_count = 0;
    int sim_ms_set = 0;  /* track if -t was given (overrides config) */
    int ui_enabled = 0;
    int ui_port = 8080;
    sim_config_t config;
    int config_loaded = 0;

    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--ui") == 0) {
            ui_enabled = 1;
            if (i + 1 < argc && argv[i+1][0] != '-') {
                ui_port = atoi(argv[++i]);
                if (ui_port <= 0) ui_port = 8080;
            }
        }
        else if (strcmp(argv[i], "-v") == 0) verbose = 1;
        else if (strcmp(argv[i], "-q") == 0) verbose = 0;
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            node_count = atoi(argv[++i]);
            if (node_count > MAX_NODES) node_count = MAX_NODES;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            sim_ms = atoi(argv[++i]);
            sim_ms_set = 1;
        } else if (strcmp(argv[i], "--threads") == 0 && i + 1 < argc) {
            num_threads = atoi(argv[++i]);
            if (num_threads < 0) num_threads = 0;
        } else if (argv[i][0] != '-') {
            if (is_json_file(argv[i])) {
                /* Load JSON config */
                if (sim_config_load(&config, argv[i]) != 0)
                    return 1;
                config_loaded = 1;
                sim_config_print(&config);
            } else {
                if (firmware_count < MAX_NODES)
                    firmware_paths[firmware_count++] = argv[i];
            }
        }
    }

    /* If a JSON config was loaded, populate firmware_paths from it */
    if (config_loaded) {
        if (firmware_count == 0) {
            /* No CLI firmware args — use all nodes from config */
            for (int i = 0; i < config.node_count && i < MAX_NODES; i++) {
                snprintf(firmware_bufs[i], sizeof(firmware_bufs[i]),
                         "%s", config.nodes[i].firmware);
                firmware_paths[i] = firmware_bufs[i];
            }
            firmware_count = config.node_count;
        }
        if (!sim_ms_set)
            sim_ms = config.timeout_ms;
        if (node_count == 0)
            node_count = config.node_count;
    }

    if (node_count == 0)
        node_count = firmware_count;

    if (firmware_count < 1) {
        printf("Usage: test_runner mixed-multinode <firmware1> [firmware2...] [-t ms] [-n nodes] [-v] [-q] [--threads N] [--ui [port]]\n");
        printf("       test_runner mixed-multinode <config.json> [-t ms] [-v] [-q] [--threads N] [--ui [port]]\n");
        printf("  Firmware types detected by extension:\n");
        printf("    .sky      -> MSP430 (Tmote Sky)\n");
        printf("    .cc2538dk -> ARM (CC2538DK)\n");
        printf("    .cooja    -> Native (Cooja mote)\n");
        printf("Example:\n");
        printf("  test_runner mixed-multinode firmware/sky/udp-server.sky firmware/cooja/udp-client.cooja -t 60000\n");
        printf("  test_runner mixed-multinode configs/rpl-udp-native.json -v\n");
        return 1;
    }

    int64_t total_ns = (int64_t)sim_ms * MS_TO_NS;

    printf("=== Mixed-Platform Multi-Node Test ===\n");
    for (int i = 0; i < firmware_count; i++) {
        node_type_t t = detect_node_type(firmware_paths[i]);
        printf("Firmware[%d]: %s (%s)\n", i, firmware_paths[i],
               t == NODE_MSP430 ? "MSP430" : t == NODE_ARM ? "ARM" : "NATIVE");
    }
    printf("Nodes: %d, Simulated time: %d ms (%lld ns), Threads: %d%s\n",
           node_count, sim_ms, (long long)total_ns,
           num_threads > 0 ? num_threads : 1,
           num_threads > 0 ? "" : " (sequential)");
    printf("Time step: %lld ns\n\n", (long long)TIME_STEP_NS);

    num_nodes = node_count;
    for (int i = 0; i < node_count; i++) {
        const char *fw = firmware_paths[i < firmware_count ? i : firmware_count - 1];
        int node_id = (config_loaded && i < config.node_count)
                      ? config.nodes[i].id : i + 1;
        if (init_node(i, fw, node_id) != 0) {
            fprintf(stderr, "Failed to initialize node %d\n", node_id);
            return 1;
        }
    }

    /* Initialize radio medium */
    radio_medium_init(&radio_medium, node_count);

    /* Assign default positions in a circle for visualization */
    if (node_count == 1) {
        radio_medium_set_position(&radio_medium, 0, 0.0, 0.0);
    } else {
        for (int i = 0; i < node_count; i++) {
            double angle = 2.0 * 3.14159265 * i / node_count;
            double radius = 20.0;
            radio_medium_set_position(&radio_medium, i,
                radius * cos(angle), radius * sin(angle));
        }
    }

    if (config_loaded && config.medium_type == 1) {
        radio_medium_configure_udgm(&radio_medium,
            config.tx_range, config.interference_range,
            config.success_ratio_tx, config.success_ratio_rx);
        for (int i = 0; i < node_count; i++) {
            if (i < config.node_count && config.nodes[i].has_position)
                radio_medium_set_position(&radio_medium, i,
                    config.nodes[i].x, config.nodes[i].y);
        }
        radio_medium_compute_neighbors(&radio_medium);
        if (config.seed)
            radio_medium_set_seed(&radio_medium, (uint32_t)config.seed);
        printf("Radio medium: UDGM (tx_range=%.1f m, interference=%.1f m, "
               "tx_ratio=%.2f, rx_ratio=%.2f)\n",
               config.tx_range, config.interference_range,
               config.success_ratio_tx, config.success_ratio_rx);
        for (int i = 0; i < node_count; i++) {
            printf("  Node %d: pos=(%.1f, %.1f) neighbors=%d\n", nodes[i].id,
                   radio_medium.nodes[i].x, radio_medium.nodes[i].y,
                   radio_medium.neighbors[i].count);
        }
    }

    /* Initialize test engine if config has test section */
    sim_test_state_t test_state;
    if (config_loaded && config.has_test) {
        memset(&test_state, 0, sizeof(test_state));
        test_state.config = &config.test;
        active_test = &test_state;
        printf("Test: %d steps to verify\n", config.test.step_count);
    } else {
        active_test = NULL;
    }

    /* Initialize WebSocket UI server */
    if (ui_enabled) {
        ui_server = ws_server_init(ui_port);
        if (ui_server) {
            ws_server_set_message_callback(ui_server, ui_message_handler, NULL);
            /* Load HTML from ui/index.html */
            FILE *hf = fopen("ui/index.html", "r");
            if (hf) {
                fseek(hf, 0, SEEK_END);
                long hlen = ftell(hf);
                fseek(hf, 0, SEEK_SET);
                char *html = malloc((size_t)hlen + 1);
                if (html) {
                    fread(html, 1, (size_t)hlen, hf);
                    html[hlen] = '\0';
                    ws_server_set_html(ui_server, html, (int)hlen);
                    free(html);
                }
                fclose(hf);
            } else {
                fprintf(stderr, "Warning: ui/index.html not found, serving default page\n");
            }
        } else {
            fprintf(stderr, "Warning: failed to start UI server on port %d\n", ui_port);
        }
    }

    /* Set initial simulation speed from config */
    if (config_loaded && config.speed > 0)
        ui_speed_ratio = config.speed;

    /* Apply per-node startup delay to desynchronize timers.
     * Each node gets a random start_ns offset. Before its start time,
     * the node is not stepped and does not receive RF. */
    if (config_loaded && config.startup_delay_ms > 0) {
        unsigned int delay_seed = config.seed ? (unsigned int)config.seed : 12345;
        printf("Applying startup delay spread: 0-%d ms\n", config.startup_delay_ms);
        for (int i = 0; i < node_count; i++) {
            delay_seed = delay_seed * 1103515245 + 12345;
            int delay_ms = (int)(delay_seed % (unsigned int)config.startup_delay_ms);
            int64_t delay_ns = (int64_t)delay_ms * MS_TO_NS;

            /* Shift each node's sim_time_ns and cycle counter forward by the
             * delay.  This makes the sleep timer (which derives ticks from
             * sim_time_ns) show different elapsed times per node, genuinely
             * desynchronizing Contiki-NG's etimer tick phase.  Pending boot
             * events fire immediately on the first step (the CPU is in WFI
             * between clock ticks anyway, so the cascade is harmless). */
            if (nodes[i].type == NODE_ARM) {
                arm_cpu_t *cpu = &nodes[i].plat.arm.cpu;
                cpu->sim_time_ns += delay_ns;
                cpu->cycles += delay_ns * cpu->cpu_freq_hz / 1000000000LL;
            } else if (nodes[i].type == NODE_MSP430) {
                msp430_cpu_t *cpu = &nodes[i].plat.msp.cpu;
                cpu->sim_time_ns += delay_ns;
                cpu->cycles += (uint64_t)delay_ns * cpu->cpu_freq_hz / 1000000000ULL;
            }

            node_start_ns[i] = node_sim_time_ns(i);
            printf("  Node %d: start at %lld ms (delay %d ms)\n",
                   nodes[i].id,
                   (long long)(node_start_ns[i] / MS_TO_NS), delay_ms);
        }
    }

    /* Initialize thread pool and swap to threaded callbacks */
    sim_thread_pool_t thread_pool;
    memset(&thread_pool, 0, sizeof(thread_pool));
    if (num_threads > 0) {
        /* Clear thread state buffers */
        memset(rf_outgoing, 0, sizeof(rf_outgoing));
        memset(frame_outgoing, 0, sizeof(frame_outgoing));
        memset(thread_state, 0, sizeof(thread_state));

        if (sim_thread_pool_init(&thread_pool, num_threads, node_count) != 0) {
            fprintf(stderr, "Failed to initialize thread pool\n");
            return 1;
        }

        /* Swap callbacks to threaded versions */
        for (int i = 0; i < node_count; i++) {
            if (nodes[i].type == NODE_MSP430) {
                msp430_platform_set_console(&nodes[i].plat.msp,
                    threaded_uart_callback, &nodes[i]);
                cc2420_set_rf_listener(&nodes[i].plat.msp.cc2420,
                    threaded_rf_tx_handler, &nodes[i]);
            } else if (nodes[i].type == NODE_ARM) {
                arm_platform_set_console(&nodes[i].plat.arm,
                    threaded_uart_callback, &nodes[i]);
                cc2538_rfcore_set_tx_callback(&nodes[i].plat.arm.rfcore,
                    threaded_rf_tx_handler, &nodes[i]);
            } else {
                nodes[i].plat.native.log_callback = threaded_uart_callback;
                nodes[i].plat.native.log_callback_data = &nodes[i];
                nodes[i].plat.native.rf_tx_callback = threaded_rf_tx_handler;
                nodes[i].plat.native.rf_tx_callback_data = &nodes[i];
                nodes[i].plat.native.rf_frame_callback = threaded_rf_frame_handler;
                nodes[i].plat.native.rf_frame_callback_data = &nodes[i];
            }
        }
        printf("Thread pool: %d threads\n", num_threads);
    }

    printf("\n--- Simulation running ---\n\n");

    /* Start sim_time_ns at max of all nodes' current sim_time_ns */
    int64_t sim_ns = 0;
    for (int i = 0; i < node_count; i++) {
        int64_t t = node_sim_time_ns(i);
        if (t > sim_ns) sim_ns = t;
    }
    printf("  Initial sim_time: %lld ns (%lld ms)\n",
           (long long)sim_ns, (long long)(sim_ns / MS_TO_NS));

    int64_t end_ns = sim_ns + total_ns;
    int64_t progress_interval = total_ns / 10;
    int64_t next_progress = sim_ns + progress_interval;
    int64_t ui_interval_ns = 100LL * MS_TO_NS;  /* 100ms sim time between UI updates */
    int64_t next_ui_ns = sim_ns + ui_interval_ns;

    /* Phase timing accumulators (ms) */
    double time_distribute = 0, time_deliver = 0, time_step = 0;
    double time_flush = 0, time_channel_sync = 0, time_test_ui = 0;

    double t_start = get_time_ms();

    while (sim_ns < end_ns || ui_server) {
        /* When paused, poll WebSocket and sleep but skip to UI broadcast */
        if (ui_paused && ui_server) {
            ws_server_poll(ui_server);
            usleep(50000); /* 50ms */
            /* Reset pacing baseline so resuming doesn't cause a burst */
            t_start = get_time_ms() - (double)(sim_ns - (end_ns - total_ns)) / 1e6 / ui_speed_ratio;
            goto ui_broadcast;
        }

        sim_ns += TIME_STEP_NS;
        current_sim_ns = sim_ns;

        /* Lazy channel sync: only when TX activity has occurred */
        double t_phase;
        if (channels_dirty && radio_medium.type != RADIO_MEDIUM_NONE) {
            t_phase = get_time_ms();
            sync_node_channels();
            channels_dirty = false;
            time_channel_sync += get_time_ms() - t_phase;
        }

        if (num_threads > 0) {
            /* === PARALLEL PATH === */

            /* Sequential: distribute RF from previous step */
            t_phase = get_time_ms();
            distribute_rf_outgoing();
            time_distribute += get_time_ms() - t_phase;

            /* Sequential: deliver to radios (native nodes only).
             * Emulated nodes' bytes stay in rf_pending until the per-sender
             * frame assembler detects a complete frame — delivering partial
             * frames would leave the radio mid-RX and corrupt the queue. */
            t_phase = get_time_ms();
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                if (nodes[i].type == NODE_NATIVE) {
                    mixed_deliver_rf_bytes(i);
                    native_dequeue_rx_frame(&nodes[i].plat.native);
                }
            }
            time_deliver += get_time_ms() - t_phase;

            /* Parallel: step all nodes */
            t_phase = get_time_ms();
            sim_thread_pool_run(&thread_pool, threaded_step_node, (void *)(intptr_t)sim_ns);
            time_step += get_time_ms() - t_phase;

            /* Sequential: drain queued RX frames for emulated nodes */
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                if (nodes[i].type != NODE_NATIVE)
                    emu_rx_queue_drain(i);
            }

            /* Sequential: flush deferred output, merge counters */
            t_phase = get_time_ms();
            flush_pending_output();
            time_flush += get_time_ms() - t_phase;
        } else {
            /* === SEQUENTIAL PATH (unchanged) === */

            t_phase = get_time_ms();
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                /* Deliver only to native nodes — emulated nodes' bytes stay
                 * in rf_pending until frame assembler completes the frame. */
                if (nodes[i].type == NODE_NATIVE) {
                    mixed_deliver_rf_bytes(i);
                    native_dequeue_rx_frame(&nodes[i].plat.native);
                }
            }
            time_deliver += get_time_ms() - t_phase;

            t_phase = get_time_ms();
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                if (nodes[i].type == NODE_NATIVE) {
                    /* Skip idle native nodes — just advance their time */
                    if (!native_has_pending_work(&nodes[i].plat.native, sim_ns)) {
                        nodes[i].plat.native.sim_time_ns = sim_ns;
                        continue;
                    }
                    step_node_until(i, sim_ns);
                    /* Process additional queued frames within this time step */
                    while (*nodes[i].plat.native.simInSize == 0 &&
                           nodes[i].plat.native.rx_queue.count > 0 &&
                           nodes[i].plat.native.sim_time_ns < sim_ns) {
                        native_dequeue_rx_frame(&nodes[i].plat.native);
                        step_node_until(i, sim_ns);
                    }
                } else {
                    /* Emulated nodes step by cycle count */
                    int64_t delta_ns = sim_ns - node_sim_time_ns(i);
                    if (delta_ns > 0) {
                        int64_t target_cycle;
                        if (nodes[i].type == NODE_MSP430)
                            target_cycle = node_cycles(i) + msp430_ns_to_cycles(delta_ns, node_freq(i));
                        else
                            target_cycle = node_cycles(i) + arm_ns_to_cycles(delta_ns, node_freq(i));
                        step_node_until(i, target_cycle);
                    }
                }
            }
            /* Drain queued RX frames for emulated nodes */
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                if (nodes[i].type != NODE_NATIVE)
                    emu_rx_queue_drain(i);
            }
            time_step += get_time_ms() - t_phase;
        }

        /* Test engine: check per-step timeout */
        if (active_test && !active_test->finished) {
            const sim_test_step_t *step =
                &active_test->config->steps[active_test->current_step];
            int step_timeout = step->timeout_ms > 0
                ? step->timeout_ms : sim_ms;
            int64_t elapsed_step_ns = sim_ns - active_test->step_start_ns;
            if (elapsed_step_ns >= (int64_t)step_timeout * MS_TO_NS) {
                active_test->finished = -1;
                snprintf(active_test->fail_reason,
                         sizeof(active_test->fail_reason),
                         "step %d timed out after %d ms waiting for \"%s\" "
                         "(matched %d/%d)",
                         active_test->current_step, step_timeout,
                         step->pattern, active_test->match_count, step->count);
            }
        }
        if (active_test && active_test->finished)
            break;

        /* WebSocket UI: poll and broadcast state */
        ui_broadcast:
        if (ui_server) {
            if (!ui_paused) ws_server_poll(ui_server);
            if (sim_ns >= next_ui_ns || ui_paused) {
                if (!ui_paused) next_ui_ns = sim_ns + ui_interval_ns;

                /* Build node info array with only new console lines */
                sim_node_info_t ni[MAX_NODES];
                const char *con_ptrs[MAX_NODES][UI_CONSOLE_LINES];
                for (int i = 0; i < node_count; i++) {
                    ni[i].id = nodes[i].id;
                    ni[i].type = node_type_str(i);
                    ni[i].x = radio_medium.nodes[i].x;
                    ni[i].y = radio_medium.nodes[i].y;
                    ni[i].cycles = node_cycles(i);
                    ni[i].freq_hz = node_freq(i);
                    ni[i].sim_time_ns = node_sim_time_ns(i);
                    ni[i].last_tx_ns = node_last_tx_ns[i];
                    ni[i].console_count = ui_console_new_count[i];
                    for (int c = 0; c < ui_console_new_count[i]; c++)
                        con_ptrs[i][c] = ui_console_new[i][c];
                    ni[i].console = con_ptrs[i];
                    ui_console_new_count[i] = 0;
                }

                /* Build radio info */
                int neighbor_counts[MAX_NODES];
                const int *neighbor_ptrs[MAX_NODES];
                for (int i = 0; i < node_count; i++) {
                    neighbor_ptrs[i] = radio_medium.neighbors[i].neighbors;
                    neighbor_counts[i] = radio_medium.neighbors[i].count;
                }
                sim_radio_info_t ri = {
                    .type = radio_medium.type == RADIO_MEDIUM_UDGM ? "UDGM" : "NONE",
                    .tx_range = radio_medium.udgm.tx_range,
                    .node_count = node_count,
                    .neighbors = neighbor_ptrs,
                    .neighbor_counts = neighbor_counts,
                };

                sim_stats_t st = {
                    .sim_time_ns = sim_ns,
                    .rf_bytes = rf_byte_count,
                    .uart_bytes = uart_byte_count,
                    .rx_frames_queued = (int)radio_medium.next_frame_id + stat_rf_frames,
                    .rx_frames_collided = stat_rx_frames_collided,
                    .speed_ratio = ui_speed_ratio,
                    .paused = ui_paused,
                };

                char *json = sim_state_to_json(ni, node_count, &ri, &st);
                if (json) {
                    ws_server_broadcast(ui_server, json, (int)strlen(json));
                    free(json);
                }

                /* Real-time pacing: throttle to target speed for UI.
                 * Sleep in small increments (50ms max) so ws_server_poll
                 * can process incoming speed changes promptly. */
                double sim_elapsed_ms = (double)(sim_ns - (end_ns - total_ns)) / 1e6;
                for (;;) {
                    double wall_elapsed = get_time_ms() - t_start;
                    double target_wall = sim_elapsed_ms / ui_speed_ratio;
                    double wait_ms = target_wall - wall_elapsed;
                    if (wait_ms <= 0) break;
                    if (wait_ms > 50.0) wait_ms = 50.0;
                    usleep((useconds_t)(wait_ms * 1000.0));
                    ws_server_poll(ui_server);
                }
            }
        }

        if (sim_ns >= next_progress) {
            int pct = (int)((sim_ns - (end_ns - total_ns)) * 100 / total_ns);
            printf("  --- Progress: %d%% (%lld ms) rf_bytes=%d uart_bytes=%d ---\n",
                   pct, (long long)(sim_ns / MS_TO_NS),
                   rf_byte_count, uart_byte_count);
            for (int i = 0; i < node_count; i++) {
                printf("    Node %d [%s]: %lld cycles, freq=%u Hz\n",
                       nodes[i].id, node_type_str(i),
                       (long long)node_cycles(i), node_freq(i));
            }
            next_progress += progress_interval;
        }
    }

    double t_end = get_time_ms();
    double elapsed_ms = t_end - t_start;

    /* If test is still running when simulation ends, mark as failed */
    int test_exit_code = 0;
    if (active_test && !active_test->finished) {
        active_test->finished = -1;
        const sim_test_step_t *step =
            &active_test->config->steps[active_test->current_step];
        snprintf(active_test->fail_reason, sizeof(active_test->fail_reason),
                 "simulation ended at %lld ms, step %d waiting for \"%s\" "
                 "(matched %d/%d)",
                 (long long)(sim_ns / MS_TO_NS), active_test->current_step,
                 step->pattern, active_test->match_count, step->count);
    }
    if (active_test) {
        printf("\n--- Test Results ---\n");
        for (int i = 0; i < active_test->config->step_count; i++) {
            const sim_test_step_t *s = &active_test->config->steps[i];
            const char *status;
            if (i < active_test->current_step)
                status = "PASS";
            else if (i == active_test->current_step && active_test->finished == -1)
                status = "FAIL";
            else
                status = "SKIP";
            printf("  Step %d [%s]: wait \"%s\"", i, status, s->pattern);
            if (s->node >= 0)
                printf(" node=%d", s->node);
            if (s->count > 1)
                printf(" count=%d", s->count);
            printf("\n");
        }
        if (active_test->finished == 1) {
            printf("\n  TEST PASSED (%lld ms simulated)\n",
                   (long long)(sim_ns / MS_TO_NS));
        } else {
            printf("\n  TEST FAILED: %s\n", active_test->fail_reason);
            test_exit_code = 1;
        }
        active_test = NULL;
    }

    printf("\n--- Simulation complete ---\n");
    printf("  Total RF bytes: %d\n", rf_byte_count);
    printf("  Total UART bytes: %d\n", uart_byte_count);
    printf("  Emu RX frames: %d direct, %d queued, %d drained, %d dropped, %d collided\n",
           stat_emu_rx_direct, stat_emu_rx_queued, stat_emu_rx_drained,
           stat_emu_rx_dropped, stat_emu_rx_collided);
    printf("  Native RX frames queued: %d\n", stat_rx_frames_queued);
    printf("  Native RX frames collided: %d\n", stat_rx_frames_collided);
    printf("  Native RX frames queue full: %d\n", stat_rx_frames_queue_full);
    extern int cc2538_rfcore_get_rxfifo_overflows(void);
    printf("  CC2538 RXFIFO overflows: %d\n", cc2538_rfcore_get_rxfifo_overflows());

    int64_t total_node_cycles = 0;
    int64_t total_node_instructions = 0;
    for (int i = 0; i < node_count; i++) {
        printf("  Node %d [%s]: %lld cycles, %lld instructions\n",
               nodes[i].id, node_type_str(i),
               (long long)node_cycles(i), (long long)node_instructions(i));
        total_node_cycles += node_cycles(i);
        total_node_instructions += node_instructions(i);
        destroy_node(i);
    }

    /* Cleanup thread pool */
    if (num_threads > 0)
        sim_thread_pool_destroy(&thread_pool);

    /* Cleanup UI server */
    if (ui_server) {
        ws_server_destroy(ui_server);
        ui_server = NULL;
    }

    printf("\n--- Phase Timing ---\n");
    double time_accounted = time_distribute + time_deliver + time_step + time_flush + time_channel_sync;
    double time_other = elapsed_ms - time_accounted;
    printf("  distribute:    %7.1f ms (%4.1f%%)\n", time_distribute, 100.0 * time_distribute / elapsed_ms);
    printf("  deliver:       %7.1f ms (%4.1f%%)\n", time_deliver, 100.0 * time_deliver / elapsed_ms);
    printf("  step (CPU):    %7.1f ms (%4.1f%%)\n", time_step, 100.0 * time_step / elapsed_ms);
    printf("  flush/output:  %7.1f ms (%4.1f%%)\n", time_flush, 100.0 * time_flush / elapsed_ms);
    printf("  channel sync:  %7.1f ms (%4.1f%%)\n", time_channel_sync, 100.0 * time_channel_sync / elapsed_ms);
    printf("  other/overhead:%7.1f ms (%4.1f%%)\n", time_other, 100.0 * time_other / elapsed_ms);

    printf("\n--- Performance ---\n");
    printf("  Wall-clock time:  %.1f ms (%.2f s)\n", elapsed_ms, elapsed_ms / 1000.0);
    printf("  Simulated time:   %d ms (%.1f s)\n", sim_ms, sim_ms / 1000.0);
    double speedup = sim_ms / elapsed_ms;
    printf("  Speed ratio:      %.1fx real-time (%d nodes, %d thread%s)\n",
           speedup, node_count,
           num_threads > 0 ? num_threads : 1,
           (num_threads > 0 ? num_threads : 1) == 1 ? "" : "s");
    printf("  Total cycles:     %lld across %d nodes\n",
           (long long)total_node_cycles, node_count);
    printf("  Total instrs:     %lld across %d nodes\n",
           (long long)total_node_instructions, node_count);
    double mips = total_node_instructions / (elapsed_ms * 1000.0);
    printf("  Throughput:       %.1f MIPS (total), %.1f MIPS (per-node)\n",
           mips, mips / node_count);

    return test_exit_code;
}
