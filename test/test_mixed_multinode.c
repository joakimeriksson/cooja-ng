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
#include "timeline.h"
#include "packet_analyzer.h"
#include "cJSON.h"
#include "js_test_engine.h"
#include "sim_event_queue.h"
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
    char firmware_path[256];
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
static int ui_full_state_requested = 1; /* send full state on first broadcast */
static int ui_restart_requested = 0;   /* restart simulation from UI */

/* --- Timeline and packet analyzer state --- */
static timeline_t timeline;
static sim_node_state_t node_states[MAX_NODES];
static sim_node_state_t prev_node_states[MAX_NODES]; /* previous delta state for change detection */
static int64_t prev_last_tx_ns[MAX_NODES];           /* previous TX timestamps for change detection */
static int suppress_state_callback = 0; /* suppress during synchronous delivery */

/* RF state change callback — fires during CPU step for real-time timeline tracking.
 * TX and RX events are handled by explicit timeline events in the TX/delivery
 * handlers (with proper frame duration). This callback handles radio ON/OFF
 * transitions (ISRXON, ISRFOFF, tx_done return-to-RX). */
static void mixed_rf_state_handler(void *user_data, int old_state, int new_state) {
    if (!ui_server || suppress_state_callback) return;
    mixed_node_t *node = (mixed_node_t *)user_data;
    int idx = (int)(node - nodes);

    sim_radio_state_t sim_state = SIM_RADIO_OFF;
    if (new_state >= RF_STATE_TX_CALIBR && new_state <= RF_STATE_TX_FINAL)
        sim_state = SIM_RADIO_TX;
    else if (new_state == RF_STATE_RX)
        sim_state = SIM_RADIO_RX;
    else if (new_state == RF_STATE_RX_CALIBR || new_state == RF_STATE_SFD_WAIT)
        sim_state = SIM_RADIO_ON;

    /* Skip TX/RX events — explicit timeline events handle those with
     * proper frame duration.  Only emit ON/OFF transitions here. */
    if (sim_state == SIM_RADIO_TX || sim_state == SIM_RADIO_RX) {
        node_states[idx].radio_state = sim_state;
        return;
    }

    if (sim_state != node_states[idx].radio_state) {
        node_states[idx].radio_state = sim_state;
        tl_event_type_t etype;
        switch (sim_state) {
        case SIM_RADIO_ON:   etype = TL_RADIO_ON;   break;
        default:             etype = TL_RADIO_OFF;   break;
        }
        /* Use CPU cycles for accurate intra-step timing */
        int64_t ts = arm_cycles_to_ns(node->plat.arm.cpu.cycles,
                                       node->plat.arm.cpu.cpu_freq_hz);
        tl_radio_event(&timeline, node->id, ts, etype);
    }
}

/* Track per-node radio state for timeline events */
static void update_radio_state(int idx) {
    sim_radio_state_t new_state = SIM_RADIO_OFF;
    if (nodes[idx].type == NODE_MSP430) {
        cc2420_radio_state_t rs = nodes[idx].plat.msp.cc2420.state;
        if (rs >= CC2420_TX_CALIBRATE && rs <= CC2420_TX_UNDERFLOW)
            new_state = SIM_RADIO_TX;
        else if (rs == CC2420_RX_FRAME)
            new_state = SIM_RADIO_RX;       /* actively receiving data (green) */
        else if (rs == CC2420_RX_CALIBRATE || rs == CC2420_RX_SFD_SEARCH)
            new_state = SIM_RADIO_ON;       /* on, listening (gray) */
        /* else: VREG_OFF, POWER_DOWN, IDLE -> OFF (white) */
    } else if (nodes[idx].type == NODE_ARM) {
        rf_state_t rs = nodes[idx].plat.arm.rfcore.state;
        if (rs >= RF_STATE_TX_CALIBR && rs <= RF_STATE_TX_FINAL)
            new_state = SIM_RADIO_TX;
        else if (rs == RF_STATE_RX)
            new_state = SIM_RADIO_RX;       /* actively receiving data (green) */
        else if (rs == RF_STATE_RX_CALIBR || rs == RF_STATE_SFD_WAIT)
            new_state = SIM_RADIO_ON;       /* on, listening (gray) */
        /* else: IDLE -> OFF (white) */
    }
    if (new_state != node_states[idx].radio_state) {
        node_states[idx].radio_state = new_state;
        if (ui_server) {
            tl_event_type_t etype;
            switch (new_state) {
            case SIM_RADIO_TX:   etype = TL_RADIO_TX;   break;
            case SIM_RADIO_RX:   etype = TL_RADIO_RX;   break;
            case SIM_RADIO_ON:   etype = TL_RADIO_ON;   break;
            case SIM_RADIO_INTF: etype = TL_RADIO_INTF; break;
            default:             etype = TL_RADIO_OFF;   break;
            }
            tl_radio_event(&timeline, nodes[idx].id, current_sim_ns, etype);
        }
    }
}

/* Track LED state changes for Tmote Sky (P5.4=green, P5.5=yellow, P5.6=red)
 * and CC2538DK (PC0=red, PC1=yellow, PC2=green) */
static void update_led_state(int idx) {
    uint8_t leds[SIM_MAX_LEDS] = {0, 0, 0};
    if (nodes[idx].type == NODE_MSP430) {
        uint8_t p5out = nodes[idx].plat.msp.gpio.ports[4].out;
        leds[0] = (p5out >> 4) & 1;  /* green  = P5.4 */
        leds[1] = (p5out >> 5) & 1;  /* yellow = P5.5 */
        leds[2] = (p5out >> 6) & 1;  /* red    = P5.6 */
    } else if (nodes[idx].type == NODE_ARM) {
        uint32_t pc_data = nodes[idx].plat.arm.gpio.ports[2].data;
        leds[0] = (pc_data >> 0) & 1;  /* red */
        leds[1] = (pc_data >> 1) & 1;  /* yellow */
        leds[2] = (pc_data >> 2) & 1;  /* green */
    }
    for (int l = 0; l < SIM_MAX_LEDS; l++) {
        if (leds[l] != node_states[idx].led[l]) {
            node_states[idx].led[l] = leds[l];
            if (ui_server)
                tl_led_event(&timeline, nodes[idx].id, current_sim_ns, l, leds[l]);
        }
    }
}

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
        } else if (strcmp(cmd->valuestring, "full") == 0) {
            ui_full_state_requested = 1;
        } else if (strcmp(cmd->valuestring, "restart") == 0) {
            ui_restart_requested = 1;
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
    int  validator_counts[MAX_TEST_VALIDATORS];
} sim_test_state_t;

static sim_test_state_t *active_test = NULL;
static js_test_engine_t *active_js_engine = NULL;

static void sim_test_check_line(int node_id, const char *line, int64_t sim_ns) {
    if (!active_test || active_test->finished)
        return;

    /* Check fail_on patterns first */
    for (int i = 0; i < active_test->config->fail_on_count; i++) {
        if (strstr(line, active_test->config->fail_on[i])) {
            active_test->finished = -1;
            snprintf(active_test->fail_reason, sizeof(active_test->fail_reason),
                     "fail_on pattern \"%s\" matched at node %d, %lld ms: %s",
                     active_test->config->fail_on[i], node_id,
                     (long long)(sim_ns / MS_TO_NS), line);
            return;
        }
    }

    /* Update validator counters (always, regardless of steps) */
    for (int i = 0; i < active_test->config->validator_count; i++) {
        const sim_test_validator_t *v = &active_test->config->validators[i];
        if (v->node >= 0 && v->node != node_id)
            continue;
        if (strstr(line, v->pattern))
            active_test->validator_counts[i]++;
    }

    /* If no steps, nothing more to check (fail_on-only or timeout_is_success test) */
    if (active_test->config->step_count == 0)
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

/* Feed a console line to whichever test engine is active */
static void test_check_line(int node_id, const char *line, int64_t sim_ns) {
    if (active_js_engine) {
        js_test_feed_line(active_js_engine, line, node_id, sim_ns / 1000);
    }
    if (active_test) {
        sim_test_check_line(node_id, line, sim_ns);
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

/* 802.15.4 byte duration at 250 kbps = 32 µs = 32000 ns */
#define IEEE802154_BYTE_NS 32000LL

/* Deliver buffered bytes directly to an emulated node's radio.
 * Emits explicit RX timeline events with computed frame air time,
 * since all bytes are delivered synchronously at the same sim time. */
static void emu_deliver_bytes(int idx, const uint8_t *data, int len,
                               int8_t rssi, int64_t air_time_ns) {
    /* Emit RX timeline event using the provided air-time timestamp
     * (derived from the sender's TX timing) for consistency. */
    if (ui_server && len > 0) {
        int64_t rx_dur = (int64_t)len * IEEE802154_BYTE_NS;
        tl_radio_event(&timeline, nodes[idx].id, air_time_ns, TL_RADIO_RX);
        tl_radio_event(&timeline, nodes[idx].id, air_time_ns + rx_dur, TL_RADIO_ON);
        node_states[idx].radio_state = SIM_RADIO_ON;
    }
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

    /* Compute cycle-accurate TX timestamps for timeline and RX delivery.
     * Total bytes on air: 4 preamble + 1 SFD + 1 length + expected_len */
    int total_air_bytes = 4 + 1 + 1 + tx_asm[sender_idx].expected_len;
    int64_t frame_air_dur = (int64_t)total_air_bytes * IEEE802154_BYTE_NS;
    int64_t accurate_tx_end;
    if (nodes[sender_idx].type == NODE_ARM)
        accurate_tx_end = arm_cycles_to_ns(nodes[sender_idx].plat.arm.cpu.cycles,
                                            nodes[sender_idx].plat.arm.cpu.cpu_freq_hz);
    else if (nodes[sender_idx].type == NODE_MSP430)
        accurate_tx_end = msp430_cycles_to_ns(nodes[sender_idx].plat.msp.cpu.cycles,
                                               nodes[sender_idx].plat.msp.cpu.cpu_freq_hz);
    else
        accurate_tx_end = current_sim_ns;
    int64_t accurate_tx_start = accurate_tx_end - frame_air_dur;
    if (accurate_tx_start < 0) accurate_tx_start = 0;

    /* Emit explicit TX timeline event */
    if (ui_server) {
        tl_radio_event(&timeline, nodes[sender_idx].id, accurate_tx_start, TL_RADIO_TX);
        tl_radio_event(&timeline, nodes[sender_idx].id, accurate_tx_end, TL_RADIO_ON);
        node_states[sender_idx].radio_state = SIM_RADIO_ON;
    }

    /* Packet analysis: decode and log frame contents */
    if (ui_server || verbose) {
        /* Find any receiver's buffered frame to decode (they're all identical) */
        for (int i = 0; i < num_nodes; i++) {
            if (rf_pending[i].count >= 6 && &nodes[i] != sender) {
                /* Frame data: starts at preamble. Find length byte (after 4×0x00+0x7A SFD) */
                uint8_t *buf = rf_pending[i].bytes;
                int fstart = -1;
                for (int b = 0; b + 5 < rf_pending[i].count; b++) {
                    if (buf[b] == 0x7A && b >= 4) {
                        fstart = b + 1; /* length byte position */
                        break;
                    }
                }
                if (fstart >= 0 && fstart < rf_pending[i].count) {
                    pkt_info_t pinfo;
                    pkt_analyze(buf + fstart, rf_pending[i].count - fstart, &pinfo);
                    if (verbose)
                        fprintf(stderr, "  [PKT] Node %d TX: %s\n",
                                nodes[sender_idx].id, pinfo.summary);
                    if (ui_server)
                        tl_frame_event(&timeline, nodes[sender_idx].id,
                                       accurate_tx_start, 1, pinfo.summary);
                }
                break;
            }
        }
    }

    rf_tx_depth++;
    suppress_state_callback = 1;  /* explicit timeline events emitted above */

    /* Snapshot: copy frame data and RSSI for each emulated receiver */
    static uint8_t frame_snap[MAX_NODES][EMU_RX_FRAME_MAX];
    static int frame_snap_len[MAX_NODES];
    static int8_t frame_snap_rssi[MAX_NODES];
    int snap_count = 0;
    static int snap_indices[MAX_NODES];

    /* DEBUG: trace frame delivery */
    if (verbose) {
        fprintf(stderr, "  [RF] Node %d TX frame (%d bytes) @ %.3f s -> receivers:",
                nodes[sender_idx].id, tx_asm[sender_idx].expected_len,
                (double)current_sim_ns / 1e9);
        for (int i = 0; i < num_nodes; i++) {
            if (rf_pending[i].count > 0 && nodes[i].type != NODE_NATIVE)
                fprintf(stderr, " %d(%d bytes)", nodes[i].id, rf_pending[i].count);
        }
        fprintf(stderr, "\n");
    }

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
        /* Collision window: use actual frame air time for realistic
         * hidden-terminal collision detection. A frame occupies the
         * channel from accurate_tx_start to accurate_tx_end. */
        int64_t coll_start = accurate_tx_start;
        int64_t coll_end = accurate_tx_end;

        /* Collision check: does this frame overlap with a previously
         * delivered frame on this receiver? */
        if (coll_start < emu_rx_end_ns[i]) {
            /* Collision with previously delivered frame — drop this one */
            stat_emu_rx_collided++;
            if (verbose)
                fprintf(stderr, "  [COLLISION] Node %d->%d frame dropped @ %.3f s "
                        "(channel busy until %.3f s)\n",
                        nodes[sender_idx].id, nodes[i].id,
                        (double)coll_start / 1e9,
                        (double)emu_rx_end_ns[i] / 1e9);
            /* Emit interference event on the receiver's timeline */
            if (ui_server) {
                int64_t intf_dur = (int64_t)frame_snap_len[i] * IEEE802154_BYTE_NS;
                tl_radio_event(&timeline, nodes[i].id, accurate_tx_start, TL_RADIO_INTF);
                tl_radio_event(&timeline, nodes[i].id, accurate_tx_start + intf_dur, TL_RADIO_ON);
            }
            continue;
        }

        int frame_payload = (frame_snap_len[i] > 5) ? frame_snap[i][5] : 0;
        if (emulated_rxfifo_available(i) < frame_payload + 1) {
            /* RXFIFO full — mini-step the receiver to let it read the
             * previous frame before delivering this one.  On real hardware
             * frames arrive with multi-ms gaps; here all TX/delivery is
             * synchronous within one sender step. Keep the step short to
             * avoid triggering cascade transmissions from the receiver. */
            step_node_until(i, node_cycles(i) + 5000);
        }
        if (emulated_rxfifo_available(i) >= frame_payload + 1) {
            emu_deliver_bytes(i, frame_snap[i], frame_snap_len[i],
                              frame_snap_rssi[i], accurate_tx_start);
            stat_emu_rx_direct++;
            emu_rx_end_ns[i] = coll_end;

            /* Generate RX frame event for packet log.
             * Parse the frame from after preamble+SFD (byte 5 onward). */
            if (ui_server && frame_snap_len[i] > 5) {
                int fstart = -1;
                for (int b = 0; b + 5 < frame_snap_len[i]; b++) {
                    if (frame_snap[i][b] == 0x7A && b >= 4) {
                        fstart = b + 1;
                        break;
                    }
                }
                if (fstart >= 0 && fstart < frame_snap_len[i]) {
                    pkt_info_t rxinfo;
                    pkt_analyze(frame_snap[i] + fstart,
                                frame_snap_len[i] - fstart, &rxinfo);
                    tl_frame_event(&timeline, nodes[i].id,
                                   accurate_tx_start, 0, rxinfo.summary);
                }
            }
        } else {
            emu_rx_queue_push(i, frame_snap[i], frame_snap_len[i],
                              frame_snap_rssi[i], accurate_tx_start, coll_end);
            emu_rx_end_ns[i] = coll_end;
        }

        /* Flush auto-ACK bytes generated by this delivery.
         * Each ACK is a single frame — flush immediately to prevent
         * multiple ACKs concatenating in the same rf_pending buffer.
         * ACK timing: ACK starts after data frame ends + 192µs turnaround
         * (12 symbol periods per 802.15.4). */
        int64_t ack_start = accurate_tx_end + 192000LL; /* 192µs turnaround */
        int ack_tx_emitted = 0;  /* emit ACK TX event only once */
        for (int j = 0; j < num_nodes; j++) {
            if (rf_pending[j].count > 0 && nodes[j].type != NODE_NATIVE) {
                /* Emit TX timeline event for the auto-ACK sender (node i) — once */
                if (ui_server && !ack_tx_emitted) {
                    int64_t ack_dur = (int64_t)rf_pending[j].count * IEEE802154_BYTE_NS;
                    tl_radio_event(&timeline, nodes[i].id, ack_start, TL_RADIO_TX);
                    tl_radio_event(&timeline, nodes[i].id, ack_start + ack_dur, TL_RADIO_ON);
                    ack_tx_emitted = 1;
                }
                int ack_payload = (rf_pending[j].count > 5) ? rf_pending[j].bytes[5] : 0;
                if (emulated_rxfifo_available(j) >= ack_payload + 1)
                    emu_deliver_bytes(j, rf_pending[j].bytes, rf_pending[j].count, -50, ack_start);
                else
                    emu_rx_queue_push(j, rf_pending[j].bytes, rf_pending[j].count,
                                      -50, ack_start, coll_end);
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
    suppress_state_callback = 0;
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
                /* Only deliver to receivers with radio ON (like COOJA's UDGM). */
                if (*nodes[i].plat.native.simRadioHWOn &&
                    radio_medium_filter_frame(&radio_medium, sender_idx, i)) {
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
    emu_deliver_bytes(idx, buf->bytes, buf->count, 0, current_sim_ns);
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
        emu_deliver_bytes(idx, f->data, f->len, f->rssi, current_sim_ns);
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
        test_check_line(node->id, node->line_buf, ns);
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
                        if (emulated_rxfifo_available(i) < frame_payload + 1)
                            step_node_until(i, node_cycles(i) + 5000);
                        if (emulated_rxfifo_available(i) >= frame_payload + 1) {
                            emu_deliver_bytes(i, buf->bytes, buf->count, rssi, tx_start);
                            stat_emu_rx_direct++;
                            emu_rx_end_ns[i] = tx_end;
                        } else {
                            emu_rx_queue_push(i, buf->bytes, buf->count, rssi,
                                              tx_start, tx_end);
                            emu_rx_end_ns[i] = tx_end;
                        }
                        buf->count = 0;
                    }

                    /* Flush auto-ACK bytes generated by this delivery */
                    {
                        int64_t native_ack_start = current_sim_ns + 192000LL;
                        for (int j = 0; j < num_nodes; j++) {
                            if (rf_pending[j].count > 0 && nodes[j].type != NODE_NATIVE) {
                                int ack_payload = (rf_pending[j].count > 5) ? rf_pending[j].bytes[5] : 0;
                                if (emulated_rxfifo_available(j) >= ack_payload + 1)
                                    emu_deliver_bytes(j, rf_pending[j].bytes, rf_pending[j].count, -50, native_ack_start);
                                else
                                    emu_rx_queue_push(j, rf_pending[j].bytes, rf_pending[j].count,
                                                      -50, current_sim_ns, current_sim_ns + TIME_STEP_NS);
                                rf_pending[j].count = 0;
                            }
                        }
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
            test_check_line(nid, ts->lines[l], current_sim_ns);
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
    plat->rfcore.state_callback = mixed_rf_state_handler;
    plat->rfcore.state_user_data = node;

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

    /* Set unique IEEE 64-bit extended address using Cooja's scheme:
     * linkaddr = {id>>8, id&0xff} repeated 4 times (big-endian pairs)
     * Stored in little-endian (reversed) for ieee_addr_cpy_to() reversal.
     * This gives node N the IPv6 IID 02XX:00XX:00XX:00XX where XX=node_id,
     * matching Contiki-NG's Cooja platform addressing. */
    uint8_t unique_addr[8] = { (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8) };
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

        /* Patch node_id (uint16_t, little-endian) */
        uint32_t nid_addr = arm_elf_find_symbol(firmware_path, "node_id");
        if (nid_addr) {
            nid_addr &= ~1u;
            arm_write8(&plat->cpu, nid_addr, (uint8_t)(node_id & 0xFF));
            arm_write8(&plat->cpu, nid_addr + 1, (uint8_t)(node_id >> 8));
            printf("  Node %d [ARM]: patched node_id at 0x%08x\n", node_id, nid_addr);
        }

        /* Patch linkaddr_node_addr */
        uint32_t la_addr = arm_elf_find_symbol(firmware_path, "linkaddr_node_addr");
        if (la_addr) {
            la_addr &= ~1u;
            /* Cooja-compatible: {id>>8, id&0xff} repeated 4 times */
            uint8_t la_bytes[8] = { (uint8_t)(node_id >> 8), (uint8_t)(node_id & 0xFF),
                                     (uint8_t)(node_id >> 8), (uint8_t)(node_id & 0xFF),
                                     (uint8_t)(node_id >> 8), (uint8_t)(node_id & 0xFF),
                                     (uint8_t)(node_id >> 8), (uint8_t)(node_id & 0xFF) };
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

/* Yield callback: called when a native node is in RTIMER_BUSYWAIT_UNTIL
 * (e.g. waiting for 802.15.4 ACK after TX). This node just sent a frame
 * and is waiting for the ACK. We need to:
 * 1. Deliver the frame to receivers
 * 2. Step the receivers so they generate the ACK
 * 3. Deliver the ACK back to this node */
static void native_yield_callback(void *user_data) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    int64_t now_ns = sender->plat.native.sim_time_ns;

    for (int r = 0; r < num_nodes; r++) {
        if (r == sender_idx || !node_active(r)) continue;
        if (nodes[r].type != NODE_NATIVE) continue;
        if (nodes[r].plat.native.rx_queue.count == 0 &&
            *nodes[r].plat.native.simInSize == 0) continue;

        native_node_t *rcv = &nodes[r].plat.native;

        /* Deliver frame to receiver if not already in simInDataBuffer */
        if (*rcv->simInSize == 0)
            native_dequeue_rx_frame(rcv);

        if (*rcv->simInSize == 0) continue;  /* no frame to process */

        /* Update receiver's time */
        rcv->sim_time_ns = now_ns;
        *rcv->simCurrentTime = (uint64_t)(now_ns / 1000000LL);
        *rcv->simRtimerCurrentTicks = (uint64_t)(now_ns / 1000LL);

        /* Temporarily disable receiver's yield callback to prevent recursion */
        void (*saved_cb)(void*) = rcv->yield_callback;
        rcv->yield_callback = NULL;

        /* Tick receiver to process the frame and generate soft ACK.
         * The first tick wakes the radio process (simInSize > 0 triggers it).
         * Subsequent ticks only if the firmware has more work (processRunValue). */
        int ack_sent = 0;
        for (int tick = 0; tick < 5; tick++) {
            rcv->cooja_tick();
            if (*rcv->simOutSize > 0) {
                native_check_radio_tx(rcv);
                ack_sent = 1;
            }
            native_check_log_output(rcv);
            if (!*rcv->simProcessRunValue) break;
        }

        rcv->yield_callback = saved_cb;

        /* If receiver sent an ACK, deliver it to the sender now */
        if (ack_sent) {
            if (*sender->plat.native.simInSize == 0)
                native_dequeue_rx_frame(&sender->plat.native);
        }
    }

    /* Also check for byte-stream ACK delivery (emulated→native path) */
    mixed_deliver_rf_bytes(sender_idx);
    if (*sender->plat.native.simInSize == 0)
        native_dequeue_rx_frame(&sender->plat.native);
}

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
    nat->yield_callback = native_yield_callback;
    nat->yield_callback_data = node;

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
    snprintf(node->firmware_path, sizeof(node->firmware_path), "%s", firmware_path);
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

/* --- Reboot node (destroy + reinitialize) --- */

static int reboot_node(int idx) {
    int node_id = nodes[idx].id;
    /* Copy firmware path before destroy — init_node's memset would zero it */
    char fw[256];
    snprintf(fw, sizeof(fw), "%s", nodes[idx].firmware_path);

    /* Clear RF state for this node */
    memset(&rf_pending[idx], 0, sizeof(rf_pending[idx]));
    memset(&emu_rx_queue[idx], 0, sizeof(emu_rx_queue[idx]));
    emu_rx_end_ns[idx] = 0;
    tx_frame_asm_reset(&tx_asm[idx]);

    /* Destroy and reinitialize */
    destroy_node(idx);
    return init_node(idx, fw, node_id);
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

/* --- Event-driven scheduling helpers (COOJA model) --- */

/* Tick a single native node at sim_ns. Single tick, no loop.
 * Matches COOJA's ContikiMote.execute(). */
static void tick_one_native(int idx, int64_t sim_ns) {
    native_node_t *nat = &nodes[idx].plat.native;
    nat->sim_time_ns = sim_ns;
    *nat->simCurrentTime = (uint64_t)(sim_ns / 1000000LL);
    *nat->simRtimerCurrentTicks = (uint64_t)(sim_ns / 1000LL);

    /* Deliver pending RX frame before tick (like COOJA's doActionsBeforeTick) */
    if (*nat->simInSize == 0 && nat->rx_queue.count > 0) {
        *nat->simReceiving = 0;
        native_dequeue_rx_frame(nat);
    }

    nat->cooja_tick();
    native_check_radio_tx(nat);
    native_check_log_output(nat);

    /* Sync this node's channel (for TSCH hopping) */
    if (nat->simRadioChannel)
        radio_medium_set_channel(&radio_medium, idx, *nat->simRadioChannel);
}

/* Schedule a native node's next wakeup in the event queue.
 * Exactly matches COOJA's ContikiClock.doActionsAfterTick(). */
static void schedule_native_wakeup(sim_event_queue_t *eq, int idx) {
    native_node_t *nat = &nodes[idx].plat.native;
    int64_t now = nat->sim_time_ns;

    /* Always schedule for rtimer if pending (exact µs timing) */
    if (*nat->simRtimerPending) {
        int64_t rt = (int64_t)(*nat->simRtimerNextExpirationTime) * 1000LL;
        sim_eq_schedule_if_earlier(eq, idx, rt);
    }

    /* processRunValue → +1ms (like COOJA's ContikiClock) */
    if (*nat->simProcessRunValue) {
        sim_eq_schedule(eq, idx, now + 1000000LL);
        return;
    }

    /* etimer */
    if (*nat->simEtimerPending) {
        int64_t et = (int64_t)(*nat->simEtimerNextExpirationTime) * 1000000LL;
        if (et <= now)
            sim_eq_schedule(eq, idx, now + 1000000LL);  /* stale → +1ms */
        else
            sim_eq_schedule(eq, idx, et);
        return;
    }

    /* No timers or process work — schedule a check in 1ms.
     * This ensures nodes don't go silent (TSCH scanning needs periodic ticks). */
    sim_eq_schedule(eq, idx, now + 1000000LL);
}

/* Schedule an emulated node's next wakeup */
static void schedule_emulated_wakeup(sim_event_queue_t *eq, int idx) {
    int64_t next;
    if (nodes[idx].type == NODE_ARM) {
        arm_cpu_t *cpu = &nodes[idx].plat.arm.cpu;
        if (cpu->next_event_cycle <= cpu->cycles)
            next = cpu->sim_time_ns;
        else
            next = cpu->sim_time_ns + arm_cycles_to_ns(
                cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
    } else {
        msp430_cpu_t *cpu = &nodes[idx].plat.msp.cpu;
        if ((int64_t)cpu->next_event_cycle <= (int64_t)cpu->cycles)
            next = cpu->sim_time_ns;
        else
            next = cpu->sim_time_ns + msp430_cycles_to_ns(
                cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
    }
    sim_eq_schedule(eq, idx, next);
}

/* Check if a native node has pending work at the given sim time */
/* Compute the next time a node needs to wake up (ns).
 * Used by event-driven stepping to skip idle periods. */
static int64_t node_next_wakeup_ns(int idx) {
    if (!node_active(idx)) return INT64_MAX;
    if (nodes[idx].type == NODE_NATIVE) {
        native_node_t *nat = &nodes[idx].plat.native;
        /* Check immediate work first */
        if (*nat->simInSize > 0 || nat->rx_queue.count > 0 ||
            *nat->simProcessRunValue)
            return nat->sim_time_ns;
        return native_next_wakeup_ns(nat);
    } else if (nodes[idx].type == NODE_ARM) {
        arm_cpu_t *cpu = &nodes[idx].plat.arm.cpu;
        if (cpu->next_event_cycle <= cpu->cycles)
            return cpu->sim_time_ns;
        int64_t delta = arm_cycles_to_ns(
            cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
        return cpu->sim_time_ns + delta;
    } else {
        msp430_cpu_t *cpu = &nodes[idx].plat.msp.cpu;
        if ((int64_t)cpu->next_event_cycle <= (int64_t)cpu->cycles)
            return cpu->sim_time_ns;
        int64_t delta = msp430_cycles_to_ns(
            cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
        return cpu->sim_time_ns + delta;
    }
}

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
sim_restart:
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
    } else if (!config_loaded || config.medium_type == 0) {
        /* Default: UDGM with tx_range covering all nodes (radius*2 + margin) */
        double default_range = (node_count > 1) ? 50.0 : 10.0;
        radio_medium_configure_udgm(&radio_medium,
            default_range, default_range * 2.0, 1.0, 1.0);
        radio_medium_compute_neighbors(&radio_medium);
    }
    if (radio_medium.type == RADIO_MEDIUM_UDGM) {
        printf("Radio medium: UDGM (tx_range=%.1f m, interference=%.1f m, "
               "tx_ratio=%.2f, rx_ratio=%.2f)\n",
               radio_medium.udgm.tx_range, radio_medium.udgm.interference_range,
               radio_medium.udgm.success_ratio_tx, radio_medium.udgm.success_ratio_rx);
        for (int i = 0; i < node_count; i++) {
            printf("  Node %d: pos=(%.1f, %.1f) neighbors=%d\n", nodes[i].id,
                   radio_medium.nodes[i].x, radio_medium.nodes[i].y,
                   radio_medium.neighbors[i].count);
        }
    }

    /* Initialize test engine if config has test section */
    sim_test_state_t test_state;
    js_test_engine_t js_engine;
    bool use_js_engine = false;

    if (config_loaded && config.has_js_script) {
        /* Load JS script */
        char *js_script = NULL;
        if (config.js_script_inline) {
            js_script = config.js_script_inline;
        } else if (config.js_script_path[0]) {
            FILE *jf = fopen(config.js_script_path, "r");
            if (!jf) {
                fprintf(stderr, "Failed to open JS script: %s\n",
                        config.js_script_path);
                return 1;
            }
            fseek(jf, 0, SEEK_END);
            long jlen = ftell(jf);
            fseek(jf, 0, SEEK_SET);
            js_script = malloc((size_t)jlen + 1);
            fread(js_script, 1, (size_t)jlen, jf);
            js_script[jlen] = '\0';
            fclose(jf);
        }
        if (js_script) {
            /* Collect node IDs */
            int js_node_ids[MAX_NODES];
            for (int i = 0; i < node_count; i++)
                js_node_ids[i] = nodes[i].id;

            if (js_test_init(&js_engine, js_script, js_node_ids, node_count) == 0) {
                use_js_engine = true;
                active_js_engine = &js_engine;
                /* Use timeout from JS TIMEOUT() call if set, else from config.
                 * Add margin so the sim loop runs past the timeout point,
                 * allowing js_test_check_timeout to fire the callback. */
                if (js_engine.timeout_us > 0)
                    total_ns = js_engine.timeout_us * 1000LL + 10 * MS_TO_NS;
                printf("Test: JavaScript engine (timeout=%lld ms)\n",
                       (long long)(js_engine.timeout_us / 1000));
            } else {
                fprintf(stderr, "Failed to initialize JS test engine\n");
                return 1;
            }
            if (js_script != config.js_script_inline)
                free(js_script);
        }
        active_test = NULL;
    } else if (config_loaded && config.has_test) {
        memset(&test_state, 0, sizeof(test_state));
        test_state.config = &config.test;
        active_test = &test_state;
        printf("Test: %d steps", config.test.step_count);
        if (config.test.fail_on_count > 0)
            printf(", %d fail_on patterns", config.test.fail_on_count);
        if (config.test.timeout_is_success)
            printf(", timeout_is_success");
        if (config.test.validator_count > 0)
            printf(", %d validators", config.test.validator_count);
        if (config.test.action_count > 0)
            printf(", %d actions", config.test.action_count);
        printf("\n");
    } else {
        active_test = NULL;
    }

    /* Initialize timeline and node state tracking */
    tl_init(&timeline);
    memset(node_states, 0, sizeof(node_states));
    memset(prev_node_states, 0, sizeof(prev_node_states));
    memset(prev_last_tx_ns, 0, sizeof(prev_last_tx_ns));

    /* Initialize WebSocket UI server (only on first run, not restart) */
    if (ui_enabled && !ui_server) {
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

    /* Track action execution index */
    int action_idx = 0;

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

    /* Initialize event queue for COOJA-model sequential stepping */
    sim_event_queue_t sim_eq;
    sim_eq_init(&sim_eq);
    if (num_threads == 0) {
        for (int i = 0; i < node_count; i++) {
            if (!node_active(i)) continue;
            sim_eq_schedule(&sim_eq, i, node_start_ns[i]);
        }
    }

    /* Phase timing accumulators (ms) */
    double time_distribute = 0, time_deliver = 0, time_step = 0;
    double time_flush = 0, time_channel_sync = 0, time_test_ui = 0;

    double t_start = get_time_ms();

    while (sim_ns < end_ns || ui_server) {
        /* Check for restart request from UI */
        if (ui_restart_requested) break;

        /* When paused, poll WebSocket and sleep but skip to UI broadcast */
        if (ui_paused && ui_server) {
            ws_server_poll(ui_server);
            usleep(50000); /* 50ms */
            /* Reset pacing baseline so resuming doesn't cause a burst */
            t_start = get_time_ms() - (double)(sim_ns - (end_ns - total_ns)) / 1e6 / ui_speed_ratio;
            goto ui_broadcast;
        }

        /* Advance simulation time.
         * For threaded mode: fixed 1ms steps.
         * For sequential mode: jump to next event in queue, capped for UI. */
        if (num_threads > 0) {
            sim_ns += TIME_STEP_NS;
        } else {
            int64_t next_event = sim_eq_peek_time(&sim_eq);
            int64_t max_ns = ui_server ? sim_ns + 100LL * MS_TO_NS : end_ns;
            if (next_event < max_ns) max_ns = next_event;
            if (max_ns <= sim_ns) max_ns = sim_ns + 1000;  /* min 1µs advance */
            sim_ns = max_ns;
        }
        current_sim_ns = sim_ns;

        /* Execute timed actions */
        if (config_loaded && config.has_test) {
            while (action_idx < config.test.action_count &&
                   sim_ns >= config.test.actions[action_idx].at_ms * MS_TO_NS) {
                const sim_test_action_t *act = &config.test.actions[action_idx];
                if (act->type == TEST_ACTION_MOVE) {
                    /* Find node index by ID */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            radio_medium_set_position(&radio_medium, i, act->x, act->y);
                            radio_medium_compute_neighbors(&radio_medium);
                            if (verbose)
                                printf("  ACTION: move node %d to (%.1f, %.1f) at %lld ms\n",
                                       act->node, act->x, act->y,
                                       (long long)(sim_ns / MS_TO_NS));
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_SEND) {
                    /* Find node index by ID and send data via serial input */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            if (nodes[i].type == NODE_ARM) {
                                for (int b = 0; act->data[b]; b++)
                                    cc2538_uart_receive_byte(
                                        &nodes[i].plat.arm.uart0,
                                        (uint8_t)act->data[b]);
                            } else if (nodes[i].type == NODE_NATIVE) {
                                native_node_t *nat = &nodes[i].plat.native;
                                if (nat->simSerialReceivingData) {
                                    int len = (int)strlen(act->data);
                                    memcpy(nat->simSerialReceivingData,
                                           act->data, (size_t)len);
                                    *nat->simSerialReceivingLength = len;
                                    *nat->simSerialReceivingFlag = 1;
                                }
                            }
                            if (verbose)
                                printf("  ACTION: send to node %d \"%s\" at %lld ms\n",
                                       act->node, act->data,
                                       (long long)(sim_ns / MS_TO_NS));
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_SEND_ALL) {
                    /* Send data via serial input to all active nodes */
                    for (int i = 0; i < node_count; i++) {
                        if (node_start_ns[i] > sim_ns)
                            continue;  /* not yet started or removed */
                        if (nodes[i].type == NODE_ARM) {
                            for (int b = 0; act->data[b]; b++)
                                cc2538_uart_receive_byte(
                                    &nodes[i].plat.arm.uart0,
                                    (uint8_t)act->data[b]);
                        } else if (nodes[i].type == NODE_NATIVE) {
                            native_node_t *nat = &nodes[i].plat.native;
                            if (nat->simSerialReceivingData) {
                                int len = (int)strlen(act->data);
                                memcpy(nat->simSerialReceivingData,
                                       act->data, (size_t)len);
                                *nat->simSerialReceivingLength = len;
                                *nat->simSerialReceivingFlag = 1;
                            }
                        }
                    }
                    if (verbose)
                        printf("  ACTION: send_all \"%s\" at %lld ms\n",
                               act->data, (long long)(sim_ns / MS_TO_NS));
                } else if (act->type == TEST_ACTION_REMOVE) {
                    /* Stop node: set start_ns to far future so it's never active */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            node_start_ns[i] = INT64_MAX;
                            if (verbose)
                                printf("  ACTION: remove node %d at %lld ms\n",
                                       act->node, (long long)(sim_ns / MS_TO_NS));
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_ADD) {
                    /* Reboot node: destroy, reinitialize, set time to now */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            if (verbose)
                                printf("  ACTION: add (reboot) node %d at %lld ms\n",
                                       act->node, (long long)(sim_ns / MS_TO_NS));
                            reboot_node(i);
                            /* Set sim_time_ns to current time so stepping
                             * resumes correctly (don't catch up from t=0) */
                            if (nodes[i].type == NODE_MSP430) {
                                msp430_cpu_t *cpu = &nodes[i].plat.msp.cpu;
                                cpu->sim_time_ns = sim_ns;
                                cpu->cycles = (uint64_t)sim_ns * cpu->cpu_freq_hz / 1000000000ULL;
                            } else if (nodes[i].type == NODE_ARM) {
                                arm_cpu_t *cpu = &nodes[i].plat.arm.cpu;
                                cpu->sim_time_ns = sim_ns;
                                cpu->cycles = sim_ns * cpu->cpu_freq_hz / 1000000000LL;
                            } else {
                                nodes[i].plat.native.sim_time_ns = sim_ns;
                            }
                            node_start_ns[i] = sim_ns;
                            break;
                        }
                    }
                }
                action_idx++;
            }
        }

        /* JS engine: check GENERATE_MSG events and drain pending actions */
        if (use_js_engine) {
            js_test_check_gen_msgs(&js_engine, sim_ns / 1000);
            sim_test_action_t js_actions[16];
            int js_act_count = js_test_drain_actions(&js_engine, js_actions, 16);
            for (int ja = 0; ja < js_act_count; ja++) {
                const sim_test_action_t *act = &js_actions[ja];
                if (act->type == TEST_ACTION_SEND) {
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            if (nodes[i].type == NODE_ARM) {
                                for (int b = 0; act->data[b]; b++)
                                    cc2538_uart_receive_byte(
                                        &nodes[i].plat.arm.uart0, (uint8_t)act->data[b]);
                            } else if (nodes[i].type == NODE_NATIVE) {
                                native_node_t *nat = &nodes[i].plat.native;
                                if (nat->simSerialReceivingData) {
                                    int len = (int)strlen(act->data);
                                    memcpy(nat->simSerialReceivingData, act->data, (size_t)len);
                                    *nat->simSerialReceivingLength = len;
                                    *nat->simSerialReceivingFlag = 1;
                                    /* Step the node so it processes the serial input */
                                    *nat->simProcessRunValue = 1;
                                    step_node_until(i, sim_ns);
                                }
                            }
                            if (verbose)
                                printf("  JS ACTION: send node %d \"%s\"\n",
                                       act->node, act->data);
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_SEND_ALL) {
                    for (int i = 0; i < node_count; i++) {
                        if (node_start_ns[i] > sim_ns) continue;
                        if (nodes[i].type == NODE_ARM) {
                            for (int b = 0; act->data[b]; b++)
                                cc2538_uart_receive_byte(
                                    &nodes[i].plat.arm.uart0, (uint8_t)act->data[b]);
                        } else if (nodes[i].type == NODE_NATIVE) {
                            native_node_t *nat = &nodes[i].plat.native;
                            if (nat->simSerialReceivingData) {
                                int len = (int)strlen(act->data);
                                memcpy(nat->simSerialReceivingData, act->data, (size_t)len);
                                *nat->simSerialReceivingLength = len;
                                *nat->simSerialReceivingFlag = 1;
                                *nat->simProcessRunValue = 1;
                                step_node_until(i, sim_ns);
                            }
                        }
                    }
                    if (verbose)
                        printf("  JS ACTION: send_all \"%s\"\n", act->data);
                } else if (act->type == TEST_ACTION_REMOVE) {
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            node_start_ns[i] = INT64_MAX;
                            if (verbose)
                                printf("  JS ACTION: remove node %d\n", act->node);
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_ADD) {
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            if (verbose)
                                printf("  JS ACTION: add (reboot) node %d\n", act->node);
                            reboot_node(i);
                            if (nodes[i].type == NODE_MSP430) {
                                msp430_cpu_t *cpu = &nodes[i].plat.msp.cpu;
                                cpu->sim_time_ns = sim_ns;
                                cpu->cycles = (uint64_t)sim_ns * cpu->cpu_freq_hz / 1000000000ULL;
                            } else if (nodes[i].type == NODE_ARM) {
                                arm_cpu_t *cpu = &nodes[i].plat.arm.cpu;
                                cpu->sim_time_ns = sim_ns;
                                cpu->cycles = sim_ns * cpu->cpu_freq_hz / 1000000000LL;
                            } else {
                                nodes[i].plat.native.sim_time_ns = sim_ns;
                            }
                            node_start_ns[i] = sim_ns;
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_MOVE) {
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            radio_medium_set_position(&radio_medium, i, act->x, act->y);
                            radio_medium_compute_neighbors(&radio_medium);
                            if (verbose)
                                printf("  JS ACTION: move node %d to (%.1f, %.1f)\n",
                                       act->node, act->x, act->y);
                            break;
                        }
                    }
                }
            }

            /* Check JS timeout and test completion */
            if (js_engine.finished != 0)
                break;
            if (js_test_check_timeout(&js_engine, sim_ns / 1000) != 0)
                break;
        }

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
            /* === SEQUENTIAL EVENT-DRIVEN PATH (COOJA model) ===
             * Pop events from the event queue, tick one node per event.
             * After each tick, check for TX and schedule receivers at
             * the same time (requestImmediateWakeup). Schedule the
             * ticked node's next wakeup based on timer state. */

            t_phase = get_time_ms();

            /* Process events up to sim_ns */
            while (!sim_eq_empty(&sim_eq) && sim_eq_peek_time(&sim_eq) <= sim_ns) {
                /* Sync ALL native node channels before each event.
                 * TSCH hops channels during ticks; we need current state
                 * for frame delivery filtering. */
                for (int n = 0; n < node_count; n++) {
                    if (nodes[n].type == NODE_NATIVE && nodes[n].plat.native.simRadioChannel)
                        radio_medium_set_channel(&radio_medium, n,
                            *nodes[n].plat.native.simRadioChannel);
                }
                channels_dirty = false;

                sim_event_t ev = sim_eq_pop(&sim_eq);
                int i = ev.node_idx;
                if (i < 0 || i >= node_count || !node_active(i))
                    continue;

                int64_t ev_time = ev.time_ns;

                /* Snapshot receiver queues to detect TX */
                int rx_before[MAX_NODES];
                for (int r = 0; r < node_count; r++)
                    rx_before[r] = (nodes[r].type == NODE_NATIVE)
                        ? nodes[r].plat.native.rx_queue.count : 0;

                if (nodes[i].type == NODE_NATIVE) {
                    /* Single tick at event time */
                    tick_one_native(i, ev_time);

                    /* Schedule next wakeup (like ContikiClock.doActionsAfterTick) */
                    schedule_native_wakeup(&sim_eq, i);
                } else {
                    /* Emulated: step to event time */
                    int64_t delta_ns = ev_time - node_sim_time_ns(i);
                    if (delta_ns > 0) {
                        int64_t target_cycle;
                        if (nodes[i].type == NODE_MSP430)
                            target_cycle = node_cycles(i) + msp430_ns_to_cycles(delta_ns, node_freq(i));
                        else
                            target_cycle = node_cycles(i) + arm_ns_to_cycles(delta_ns, node_freq(i));
                        step_node_until(i, target_cycle);
                    }
                    schedule_emulated_wakeup(&sim_eq, i);
                }

                /* RF delivery: if this node TX'd, schedule receivers
                 * at the same time (requestImmediateWakeup).
                 * Set simReceiving=1 on receivers so TSCH knows a frame
                 * is in the air (like COOJA's signalReceptionStart).
                 * Note: radio-off filtering is done in mixed_rf_frame_handler. */
                for (int r = 0; r < node_count; r++) {
                    if (r == i || !node_active(r)) continue;
                    if (nodes[r].type == NODE_NATIVE &&
                        nodes[r].plat.native.rx_queue.count > rx_before[r]) {
                        *nodes[r].plat.native.simReceiving = 1;
                        sim_eq_schedule(&sim_eq, r, ev_time);
                    }
                }

                /* Drain emulated RX queues */
                for (int r = 0; r < node_count; r++) {
                    if (!node_active(r) || nodes[r].type == NODE_NATIVE) continue;
                    emu_rx_queue_drain(r);
                }

                /* Deliver pending bytes to native assemblers */
                for (int r = 0; r < node_count; r++) {
                    if (!node_active(r) || nodes[r].type != NODE_NATIVE) continue;
                    mixed_deliver_rf_bytes(r);
                }
            }

            time_step += get_time_ms() - t_phase;
        }

        /* Update per-node radio/LED state for timeline */
        if (ui_server) {
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i) || nodes[i].type == NODE_NATIVE) continue;
                if (nodes[i].type == NODE_MSP430)
                    update_radio_state(i);  /* ARM uses state callback instead */
                update_led_state(i);
            }
        }

        /* Test engine: check per-step timeout */
        if (active_test && !active_test->finished &&
            active_test->config->step_count > 0) {
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

                sim_stats_t st = {
                    .sim_time_ns = sim_ns,
                    .rf_bytes = rf_byte_count,
                    .uart_bytes = uart_byte_count,
                    .rx_frames_queued = (int)radio_medium.next_frame_id + stat_rf_frames,
                    .rx_frames_collided = stat_rx_frames_collided,
                    .speed_ratio = ui_speed_ratio,
                    .paused = ui_paused,
                };

                int has_clients = ws_server_client_count(ui_server) > 0;
                char *json = NULL;

                if (ui_full_state_requested && has_clients) {
                    /* === Full state: on connect/reload === */
                    ui_full_state_requested = 0;

                    /* Build full node info with ALL console history */
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
                        /* Send ALL console history for full state */
                        ni[i].console_count = ui_console_count[i];
                        int base = (ui_console_head[i] - ui_console_count[i]
                                    + UI_CONSOLE_LINES) % UI_CONSOLE_LINES;
                        for (int c = 0; c < ui_console_count[i]; c++)
                            con_ptrs[i][c] = ui_console[i][(base + c) % UI_CONSOLE_LINES];
                        ni[i].console = con_ptrs[i];
                        /* Don't reset new counts — delta still needs them */
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

                    /* Skip timeline in full state — viewer accumulates
                     * events from CBOR deltas after connect.  This keeps the
                     * full state JSON small (~10 KB vs multi-MB). */
                    json = sim_state_to_json(ni, node_count, &ri, &st,
                                             node_states, NULL);
                    /* Flush old timeline events so the first delta only
                     * contains events from NOW, not stale history that
                     * would be off-screen in the viewer. */
                    tl_flush_new(&timeline);
                    /* Clear new console counts */
                    for (int i = 0; i < node_count; i++)
                        ui_console_new_count[i] = 0;

                } else if (has_clients) {
                    /* === Delta: CBOR binary, change-only === */
                    int ids[MAX_NODES];
                    const char *con_new_ptrs[MAX_NODES][UI_CONSOLE_LINES];
                    const char **con_ptr_arr[MAX_NODES];
                    int con_counts[MAX_NODES];

                    for (int i = 0; i < node_count; i++) {
                        ids[i] = nodes[i].id;
                        con_counts[i] = ui_console_new_count[i];
                        for (int c = 0; c < ui_console_new_count[i]; c++)
                            con_new_ptrs[i][c] = ui_console_new[i][c];
                        con_ptr_arr[i] = con_new_ptrs[i];
                        ui_console_new_count[i] = 0;
                    }

                    /* Encode timeline events to CBOR.
                     * Always flush new_count to prevent accumulation if
                     * buffer overflows — losing a batch is better than
                     * permanently losing all future events. */
                    static uint8_t tl_cbor_buf[262144];
                    int tl_cbor_len = tl_events_to_cbor(
                        (struct timeline_s *)&timeline,
                        tl_cbor_buf, (int)sizeof(tl_cbor_buf));
                    tl_flush_new(&timeline);

                    /* Encode full delta to CBOR — must be large enough
                     * for timeline data + stats + console + radio changes */
                    static uint8_t cbor_buf[524288];
                    int cbor_len = sim_state_delta_cbor(
                        cbor_buf, (int)sizeof(cbor_buf),
                        &st, node_states, prev_node_states,
                        node_count, ids,
                        node_last_tx_ns, prev_last_tx_ns,
                        (const char ***)con_ptr_arr, con_counts,
                        tl_cbor_buf, tl_cbor_len);

                    if (cbor_len > 0)
                        ws_server_broadcast_binary(ui_server, cbor_buf, cbor_len);

                    /* Save current state as previous for next delta */
                    memcpy(prev_node_states, node_states, sizeof(node_states));
                    memcpy(prev_last_tx_ns, node_last_tx_ns, sizeof(prev_last_tx_ns));
                }

                /* Broadcast the JSON (full state only) */
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

    /* Handle restart request from UI */
    if (ui_restart_requested && ui_server) {
        printf("\n--- Restarting simulation (requested from UI) ---\n\n");
        ui_restart_requested = 0;

        /* Destroy all nodes */
        for (int i = 0; i < node_count; i++)
            destroy_node(i);

        /* Cleanup thread pool */
        if (num_threads > 0)
            sim_thread_pool_destroy(&thread_pool);

        /* Reset all global state */
        rf_byte_count = 0;
        uart_byte_count = 0;
        current_sim_ns = 0;
        stat_rf_frames = 0;
        stat_emu_rx_direct = 0;
        stat_emu_rx_queued = 0;
        stat_emu_rx_drained = 0;
        stat_emu_rx_dropped = 0;
        stat_emu_rx_collided = 0;
        stat_rx_frames_queued = 0;
        stat_rx_frames_collided = 0;
        stat_rx_frames_queue_full = 0;
        channels_dirty = false;
        memset(rf_pending, 0, sizeof(rf_pending));
        memset(emu_rx_queue, 0, sizeof(emu_rx_queue));
        memset(emu_rx_end_ns, 0, sizeof(emu_rx_end_ns));
        memset(tx_asm, 0, sizeof(tx_asm));
        memset(rf_outgoing, 0, sizeof(rf_outgoing));
        memset(frame_outgoing, 0, sizeof(frame_outgoing));
        memset(thread_state, 0, sizeof(thread_state));
        memset(node_last_tx_ns, 0, sizeof(node_last_tx_ns));
        memset(node_start_ns, 0, sizeof(node_start_ns));
        memset(ui_console, 0, sizeof(ui_console));
        memset(ui_console_head, 0, sizeof(ui_console_head));
        memset(ui_console_count, 0, sizeof(ui_console_count));
        memset(ui_console_new, 0, sizeof(ui_console_new));
        memset(ui_console_new_count, 0, sizeof(ui_console_new_count));
        memset(node_states, 0, sizeof(node_states));
        tl_init(&timeline);
        extern void cc2538_rfcore_reset_rxfifo_overflows(void);
        cc2538_rfcore_reset_rxfifo_overflows();

        /* Request full state send on first UI broadcast */
        ui_full_state_requested = 1;

        goto sim_restart;
    }

    double t_end = get_time_ms();
    double elapsed_ms = t_end - t_start;

    /* If test is still running when simulation ends */
    int test_exit_code = 0;
    if (active_test && !active_test->finished) {
        if (active_test->config->timeout_is_success) {
            /* timeout_is_success: reaching timeout without failure = PASS */
            active_test->finished = 1;
        } else if (active_test->config->step_count == 0) {
            /* No steps and no failure hit — pass (fail_on-only test) */
            active_test->finished = 1;
        } else {
            active_test->finished = -1;
            const sim_test_step_t *step =
                &active_test->config->steps[active_test->current_step];
            snprintf(active_test->fail_reason, sizeof(active_test->fail_reason),
                     "simulation ended at %lld ms, step %d waiting for \"%s\" "
                     "(matched %d/%d)",
                     (long long)(sim_ns / MS_TO_NS), active_test->current_step,
                     step->pattern, active_test->match_count, step->count);
        }
    }
    /* Check validators at end of test (when test would otherwise pass) */
    if (active_test && active_test->finished == 1 &&
        active_test->config->validator_count > 0) {
        for (int i = 0; i < active_test->config->validator_count; i++) {
            if (active_test->validator_counts[i] <
                active_test->config->validators[i].min_count) {
                active_test->finished = -1;
                snprintf(active_test->fail_reason, sizeof(active_test->fail_reason),
                         "validator \"%s\" matched %d/%d times",
                         active_test->config->validators[i].pattern,
                         active_test->validator_counts[i],
                         active_test->config->validators[i].min_count);
                break;
            }
        }
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
        /* Print validator results */
        for (int i = 0; i < active_test->config->validator_count; i++) {
            const sim_test_validator_t *v = &active_test->config->validators[i];
            int count = active_test->validator_counts[i];
            const char *vstat = (count >= v->min_count) ? "PASS" : "FAIL";
            printf("  Validator [%s]: \"%s\" matched %d/%d",
                   vstat, v->pattern, count, v->min_count);
            if (v->node >= 0)
                printf(" node=%d", v->node);
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

    /* JS test engine results */
    if (use_js_engine) {
        printf("\n--- JS Test Results ---\n");
        if (js_engine.finished == 1) {
            printf("  TEST PASSED (%lld ms simulated)\n",
                   (long long)(sim_ns / MS_TO_NS));
        } else {
            printf("  TEST FAILED: %s\n", js_test_fail_reason(&js_engine));
            test_exit_code = 1;
        }
        js_test_destroy(&js_engine);
        active_js_engine = NULL;
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
