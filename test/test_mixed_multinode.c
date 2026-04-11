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
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>

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
    double clock_deviation; /* 1.0 = normal, <1.0 = slower (Cooja MspClock) */
    int64_t last_execute_ns; /* last tick sim_ns (for ns-precision stepping) */
    double ideal_cycles;     /* cumulative ideal cycle target (like MSPSim lastMicrosCycles) */
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
    int64_t first_byte_ns; /* sim time of first preamble byte */
} tx_frame_asm_t;

typedef struct {
    uint8_t bytes[160];
    int len;
} tx_frame_capture_t;

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
    a->first_byte_ns = 0;
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

#define MSP_RX_BYTE_QUEUE_SIZE 32768
typedef struct {
    int node_idx;
    int sender_idx;
    uint8_t byte;
    int8_t rssi;
    int64_t time_ns;
    uint64_t seq;
} msp_rx_byte_event_t;

typedef struct {
    msp_rx_byte_event_t heap[MSP_RX_BYTE_QUEUE_SIZE];
    int count;
} msp_rx_byte_queue_t;

static mixed_node_t nodes[MAX_NODES];
static int ticking_node_idx = -1;  /* node currently inside tick_one_msp430 (re-entrancy guard) */
static int64_t tick_one_msp430(int idx, int64_t sim_ns);  /* forward decl */

/* PC trace callback for debugging */
static uint32_t srh_trace_fn_addr;
static int cc2420_transmit_count;
static int tsch_eb_process_count;
static int tsch_queue_add_count;
void srh_trace_cb(void *data, uint32_t pc, uint32_t *reg, uint8_t *mem) {
    (void)data; (void)mem; (void)reg;
    /* Track firmware-level cc2420_transmit calls */
    if (srh_trace_fn_addr && pc == srh_trace_fn_addr)
        cc2420_transmit_count++;
    /* Track EB process calls */
    if (pc == 0xcb32)
        tsch_eb_process_count++;
    /* Track queue add */
    if (pc == 0xb138)
        tsch_queue_add_count++;
}
static int num_nodes = 0;
static int verbose = 1;
static rf_buffer_t rf_pending[MAX_NODES];
static tx_frame_asm_t tx_asm[MAX_NODES];     /* per-sender frame assembler */
static tx_frame_capture_t tx_cap[MAX_NODES]; /* sender-side full-frame capture */
static emu_rx_queue_t emu_rx_queue[MAX_NODES]; /* per-receiver frame queue */
static msp_rx_byte_queue_t msp_rx_byte_queue;
static int64_t emu_rx_end_ns[MAX_NODES];      /* end time of last RX for each emulated receiver */
static int rf_byte_count = 0;
static sim_event_queue_t sim_eq;              /* event queue (used by emu_deliver_bytes) */
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
static uint64_t stat_msp_byte_push[MAX_NODES];
static uint64_t stat_msp_byte_pop[MAX_NODES];
static uint64_t stat_msp_byte_push_2_to_1 = 0;
static uint64_t stat_msp_byte_pop_2_to_1 = 0;
static int trace_tsch_ack = -1;
static int trace_tsch_ack_lines = 0;
static int trace_event_spin = -1;
#define TRACE_TSCH_ACK_START_NS 12000000000LL
#define TRACE_TSCH_ACK_END_NS   16000000000LL
#define TRACE_TSCH_ACK_MAX_LINES 2000

static bool trace_tsch_ack_enabled(void) {
    if (trace_tsch_ack < 0) {
        const char *env = getenv("CSIM_TRACE_TSCH_ACK");
        trace_tsch_ack = (env && env[0] && strcmp(env, "0") != 0) ? 1 : 0;
    }
    return trace_tsch_ack != 0;
}

static bool trace_event_spin_enabled(void) {
    if (trace_event_spin < 0) {
        const char *env = getenv("CSIM_TRACE_EVENT_SPIN");
        trace_event_spin = (env && env[0] && strcmp(env, "0") != 0) ? 1 : 0;
    }
    return trace_event_spin != 0;
}

static const char *cc2420_state_str(cc2420_radio_state_t s) {
    switch (s) {
    case CC2420_VREG_OFF: return "VREG_OFF";
    case CC2420_POWER_DOWN: return "POWER_DOWN";
    case CC2420_IDLE: return "IDLE";
    case CC2420_RX_CALIBRATE: return "RX_CAL";
    case CC2420_RX_SFD_SEARCH: return "RX_SFD";
    case CC2420_RX_WAIT: return "RX_WAIT";
    case CC2420_RX_FRAME: return "RX_FRAME";
    case CC2420_RX_OVERFLOW: return "RX_OVERFLOW";
    case CC2420_TX_CALIBRATE: return "TX_CAL";
    case CC2420_TX_PREAMBLE: return "TX_PREAMBLE";
    case CC2420_TX_FRAME: return "TX_FRAME";
    case CC2420_TX_ACK_CALIBRATE: return "TX_ACK_CAL";
    case CC2420_TX_ACK_PREAMBLE: return "TX_ACK_PREAMBLE";
    case CC2420_TX_ACK: return "TX_ACK";
    case CC2420_TX_UNDERFLOW: return "TX_UNDERFLOW";
    default: return "UNKNOWN";
    }
}

static void trace_tsch_ack_log(const char *fmt, ...) {
    if (!trace_tsch_ack_enabled() || trace_tsch_ack_lines >= TRACE_TSCH_ACK_MAX_LINES ||
        current_sim_ns < TRACE_TSCH_ACK_START_NS ||
        current_sim_ns > TRACE_TSCH_ACK_END_NS)
        return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [TRACE] %7.3f ", (double)current_sim_ns / 1e9);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    trace_tsch_ack_lines++;
}

static void trace_tsch_ack_log_at(int64_t time_ns, const char *fmt, ...) {
    if (!trace_tsch_ack_enabled() || trace_tsch_ack_lines >= TRACE_TSCH_ACK_MAX_LINES ||
        time_ns < TRACE_TSCH_ACK_START_NS || time_ns > TRACE_TSCH_ACK_END_NS)
        return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [TRACE] %7.3f ", (double)time_ns / 1e9);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    trace_tsch_ack_lines++;
}

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

/* --- Serial socket (TCP bridge for border-router tests) --- */
static int ss_listen_fd = -1;    /* listening socket */
static int ss_client_fd = -1;    /* accepted client connection */
static int ss_node_idx = -1;     /* index of the bridged node */
static pid_t ss_child_pid = -1;  /* PID of external command */
static int ss_child_exited = 0;  /* set when child exits */
static int ss_child_status = -1; /* child exit status */

/* TX ring buffer: UART output from bridged mote → TCP socket */
/* Border-router tests can burst multi-fragment SLIP traffic, especially when
 * native border-router mode forwards large IPv6 packets. Keep enough buffered
 * data to absorb those bursts without silently dropping bytes. */
#define SS_TX_BUF_SIZE 65536
static uint8_t ss_tx_buf[SS_TX_BUF_SIZE];
static int ss_tx_head = 0, ss_tx_count = 0;

/* RX ring buffer: TCP socket → bridged mote serial input */
#define SS_RX_BUF_SIZE 65536
static uint8_t ss_rx_buf[SS_RX_BUF_SIZE];
static int ss_rx_head = 0, ss_rx_count = 0;

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
static void schedule_emulated_wakeup(sim_event_queue_t *eq, int idx);
static int64_t sync_msp430_to_time(int idx, int64_t sim_ns);

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

static bool msp_rx_byte_event_less(const msp_rx_byte_event_t *a,
                                   const msp_rx_byte_event_t *b) {
    if (a->time_ns != b->time_ns)
        return a->time_ns < b->time_ns;
    return a->seq < b->seq;
}

static bool msp_rx_byte_event_before_node(const msp_rx_byte_event_t *byte_ev,
                                          const sim_event_t *node_ev) {
    if (byte_ev->time_ns != node_ev->time_ns)
        return byte_ev->time_ns < node_ev->time_ns;
    return byte_ev->seq < node_ev->seq;
}

static void msp_rx_byte_heap_swap(msp_rx_byte_queue_t *q, int i, int j) {
    msp_rx_byte_event_t tmp = q->heap[i];
    q->heap[i] = q->heap[j];
    q->heap[j] = tmp;
}

static void msp_rx_byte_heap_sift_up(msp_rx_byte_queue_t *q, int idx) {
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        if (!msp_rx_byte_event_less(&q->heap[idx], &q->heap[parent]))
            break;
        msp_rx_byte_heap_swap(q, idx, parent);
        idx = parent;
    }
}

static void msp_rx_byte_heap_sift_down(msp_rx_byte_queue_t *q, int idx) {
    while (1) {
        int smallest = idx;
        int left = 2 * idx + 1;
        int right = 2 * idx + 2;
        if (left < q->count && msp_rx_byte_event_less(&q->heap[left], &q->heap[smallest]))
            smallest = left;
        if (right < q->count && msp_rx_byte_event_less(&q->heap[right], &q->heap[smallest]))
            smallest = right;
        if (smallest == idx)
            break;
        msp_rx_byte_heap_swap(q, idx, smallest);
        idx = smallest;
    }
}

static void msp_rx_byte_queue_push(int idx, int sender_idx, uint8_t byte, int8_t rssi,
                                   int64_t time_ns) {
    msp_rx_byte_queue_t *q = &msp_rx_byte_queue;
    if (q->count >= MSP_RX_BYTE_QUEUE_SIZE) {
        stat_emu_rx_dropped++;
        return;
    }
    stat_msp_byte_push[idx]++;
    if (sender_idx >= 0 && sender_idx < num_nodes &&
        nodes[sender_idx].id == 2 && nodes[idx].id == 1) {
        stat_msp_byte_push_2_to_1++;
    }
    int slot = q->count++;
    q->heap[slot].node_idx = idx;
    q->heap[slot].sender_idx = sender_idx;
    q->heap[slot].byte = byte;
    q->heap[slot].rssi = rssi;
    q->heap[slot].time_ns = time_ns;
    q->heap[slot].seq = sim_eq.next_seq++;
    msp_rx_byte_heap_sift_up(q, slot);
}

static int64_t msp_rx_byte_queue_peek_time(void) {
    return msp_rx_byte_queue.count > 0 ? msp_rx_byte_queue.heap[0].time_ns : INT64_MAX;
}

static msp_rx_byte_event_t msp_rx_byte_queue_peek(void) {
    if (msp_rx_byte_queue.count <= 0) {
        msp_rx_byte_event_t empty = {
            .node_idx = -1, .sender_idx = -1, .byte = 0, .rssi = 0,
            .time_ns = INT64_MAX, .seq = UINT64_MAX
        };
        return empty;
    }
    return msp_rx_byte_queue.heap[0];
}

static bool msp_rx_byte_queue_pop(msp_rx_byte_event_t *out) {
    msp_rx_byte_queue_t *q = &msp_rx_byte_queue;
    if (q->count <= 0)
        return false;
    *out = q->heap[0];
    q->heap[0] = q->heap[--q->count];
    if (q->count > 0)
        msp_rx_byte_heap_sift_down(q, 0);
    return true;
}

static void msp_rx_byte_queue_remove_node(int idx) {
    msp_rx_byte_queue_t *q = &msp_rx_byte_queue;
    int write = 0;
    for (int read = 0; read < q->count; read++) {
        if (q->heap[read].node_idx == idx)
            continue;
        if (write != read)
            q->heap[write] = q->heap[read];
        write++;
    }
    q->count = write;
    for (int i = q->count / 2 - 1; i >= 0; i--)
        msp_rx_byte_heap_sift_down(q, i);
}

static bool drain_msp430_rx_byte_event(int64_t up_to_ns) {
    msp_rx_byte_event_t ev;
    if (msp_rx_byte_queue.count <= 0 || msp_rx_byte_queue.heap[0].time_ns > up_to_ns)
        return false;
    if (!msp_rx_byte_queue_pop(&ev))
        return false;
    if (ev.node_idx < 0 || ev.node_idx >= num_nodes ||
        !node_active(ev.node_idx) || nodes[ev.node_idx].type != NODE_MSP430)
        return true;
    stat_msp_byte_pop[ev.node_idx]++;
    if (ev.sender_idx >= 0 && ev.sender_idx < num_nodes &&
        nodes[ev.sender_idx].id == 2 && nodes[ev.node_idx].id == 1) {
        stat_msp_byte_pop_2_to_1++;
    }
    int64_t returned_us = sync_msp430_to_time(ev.node_idx, ev.time_ns);
    cc2420_radio_state_t old_state = nodes[ev.node_idx].plat.msp.cc2420.state;
    if (trace_tsch_ack_enabled() &&
        nodes[ev.node_idx].id > 0 && nodes[ev.node_idx].id <= 2) {
        trace_tsch_ack_log("byte event node=%d byte=%02x t=%.6f state_before=%s",
                           nodes[ev.node_idx].id,
                           ev.byte, (double)ev.time_ns / 1e9,
                           cc2420_state_str(old_state));
    }
    nodes[ev.node_idx].plat.msp.cc2420.rx_rssi = ev.rssi;
    cc2420_receive_byte(&nodes[ev.node_idx].plat.msp.cc2420, ev.byte);
    cc2420_radio_state_t new_state = nodes[ev.node_idx].plat.msp.cc2420.state;
    if (trace_tsch_ack_enabled() &&
        nodes[ev.node_idx].id > 0 && nodes[ev.node_idx].id <= 2 &&
        new_state != old_state) {
        trace_tsch_ack_log("byte event node=%d state %s -> %s",
                           nodes[ev.node_idx].id,
                           cc2420_state_str(old_state),
                           cc2420_state_str(new_state));
    }
    if (num_threads == 0) {
        /* Match Cooja's MspMoteTimeEvent + requestImmediateWakeup:
         * execute(t, 0) first, then receivedByte(), then a same-time mote
         * wakeup request. Also retain the next wakeup returned by execute. */
        int64_t next_ns = ev.time_ns + returned_us * 1000LL;
        sim_eq_schedule_if_earlier(&sim_eq, ev.node_idx, next_ns);
        sim_eq_schedule_if_earlier(&sim_eq, ev.node_idx, ev.time_ns);
    }
    return true;
}

/* 802.15.4 byte duration at 250 kbps = 32 µs = 32000 ns */
#define IEEE802154_BYTE_NS 32000LL

static int64_t sync_msp430_to_time(int idx, int64_t sim_ns) {
    /* Cap to global sim time — no node should advance past it (Cooja invariant) */
    if (sim_ns > current_sim_ns)
        sim_ns = current_sim_ns;
    msp430_cpu_t *cpu = &nodes[idx].plat.msp.cpu;
    int64_t t_us = sim_ns / 1000LL;
    int64_t jump_us = 0;

    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }

    /* Apply clock deviation (same as tick_one_msp430) */
    double deviation = nodes[idx].clock_deviation;
    if (deviation != 1.0 && jump_us > 0) {
        double exact = (double)jump_us * deviation;
        jump_us = (int64_t)exact;
        cpu->step_cycle_remainder += exact - (double)jump_us;
        if (cpu->step_cycle_remainder > 1.0) {
            jump_us++;
            cpu->step_cycle_remainder -= 1.0;
        }
    }

    int64_t returned_us = msp430_step_micros(cpu, jump_us, 0);
    /* Cooja's MspMoteTimeEvent.execute(t) performs execute(t, 0): the mote is
     * synchronized to the scheduler's event time t for this delivery, rather
     * than leaving peripheral event scheduling anchored to a cycle-derived
     * local time that may have drifted ahead. Keep the CPU's event-time view
     * pinned to the byte-delivery timestamp after a zero-duration sync. */
    cpu->sim_time_ns = sim_ns;
    cpu->last_execute_us = t_us;
    if (deviation != 1.0 && returned_us > 0)
        returned_us = (int64_t)((double)returned_us / deviation);
    return returned_us;
}

/* Deliver buffered bytes directly to an emulated node's radio.
 * Emits explicit RX timeline events with computed frame air time.
 * For MSP430, advance the receiver's internal clock before each byte to
 * better approximate Cooja's byte-delivery events: execute(t, 0) +
 * receivedByte(byte) + requestImmediateWakeup(). */
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
        /* If CC2420 is mid-frame (RX_FRAME), queue instead of delivering.
         * Delivering now would corrupt the in-progress frame. This happens
         * when two senders' frames are delivered in the same event tick. */
        cc2420_radio_state_t rx_state = nodes[idx].plat.msp.cc2420.state;
        if (rx_state == CC2420_RX_FRAME || rx_state == CC2420_RX_OVERFLOW) {
            emu_rx_queue_push(idx, data, len, rssi, air_time_ns,
                              air_time_ns + (int64_t)len * IEEE802154_BYTE_NS);
            return;
        }
        /* Match Cooja's Msp802154Radio.receiveCustomData(): incoming bytes
         * for MSP radios are delivered at the same simulation time, each
         * preceded by execute(t, 0). Keep ACK frames atomic for now, but
         * still synchronize the receiver to the ACK arrival time once so
         * the sender observes the turnaround at the correct simulated time.
         *
         * GUARD: If this node is currently being ticked (ticking_node_idx),
         * skip sync to prevent re-entrant msp430_step_until.  Bytes go into
         * CC2420's rx_incoming buffer and are replayed when the CC2420
         * transitions to RX_SFD_SEARCH naturally. */
        nodes[idx].plat.msp.cc2420.rx_rssi = rssi;
        int64_t last_byte_ns = air_time_ns;
        if (idx == ticking_node_idx) {
            /* Ticking node: deliver bytes then process CC2420 events.
             * Match Cooja's pattern: execute(t, 0) + receivedByte + requestImmediateWakeup.
             * We can't do full sync (cascade risk), but we CAN process pending
             * CC2420 symbol events by stepping a tiny amount. This allows the
             * CC2420 to transition to RX_SFD_SEARCH, replay rx_incoming, process
             * the frame, and generate auto-ACK — all within this delivery call. */
            for (int j = 0; j < len; j++)
                cc2420_receive_byte(&nodes[idx].plat.msp.cc2420, data[j]);
            /* Step enough cycles for CC2420 calibration (12 symbols = 192µs ≈ 750 cycles)
             * + frame processing + ACK generation. Use 1ms = ~4000 cycles as safe budget. */
            {
                int64_t budget = (int64_t)node_freq(idx) / 1000; /* 1ms */
                if (budget < 1000) budget = 1000;
                step_node_until(idx, node_cycles(idx) + budget);
            }
        } else {
            /* Normal receiver: sync clock then deliver bytes.
             * Match Cooja's Msp802154Radio.receiveCustomData():
             * each byte is preceded by execute(t, 0) at the byte's air time. */
            for (int j = 0; j < len; j++) {
                int64_t byte_time_ns = air_time_ns + (int64_t)j * IEEE802154_BYTE_NS;
                sync_msp430_to_time(idx, byte_time_ns);
                cc2420_receive_byte(&nodes[idx].plat.msp.cc2420, data[j]);
                last_byte_ns = byte_time_ns;
            }
        }

        /* Request immediate wakeup after the delivered byte burst so the
         * receiver gets CPU time to process FIFOP/ISR work. */
        if (num_threads == 0) {
            sim_eq_schedule_if_earlier(&sim_eq, idx, last_byte_ns);
        }
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
    /* Record first byte time for accurate TX start computation and track
     * subsequent bytes on the sender's on-air byte clock. */
    if (tx_asm[sender_idx].state == TX_ASM_PREAMBLE && tx_asm[sender_idx].zero_count == 0 && byte == 0x00) {
        /* Match Cooja's radio callbacks: outgoing bytes are observed at the
         * current scheduler time, not from a mote-local sim_time that may
         * have advanced within the current execute slice. */
        tx_asm[sender_idx].first_byte_ns = current_sim_ns;
        tx_cap[sender_idx].len = 0;
    }
    int64_t byte_time_ns = tx_asm[sender_idx].first_byte_ns +
                           (int64_t)tx_cap[sender_idx].len * IEEE802154_BYTE_NS;
    node_last_tx_ns[sender_idx] = byte_time_ns;
    if (tx_cap[sender_idx].len < (int)sizeof(tx_cap[sender_idx].bytes))
        tx_cap[sender_idx].bytes[tx_cap[sender_idx].len++] = byte;

    if (rf_tx_depth > 0) {
        /* Re-entrant call (auto-ACK from a receiver). Deliver directly
         * for MSP430, buffer for others. */
        if (trace_tsch_ack_enabled() && sender->type == NODE_MSP430) {
            trace_tsch_ack_log("reentrant tx byte sender=%d state=%s byte=%02x depth=%d",
                               nodes[sender_idx].id,
                               cc2420_state_str(sender->plat.msp.cc2420.state),
                               byte, rf_tx_depth);
        }
        for (int i = 0; i < num_nodes; i++) {
            if (&nodes[i] == sender) continue;
            if (!node_active(i)) continue;
            if (sender->type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
                continue;
            if (!radio_medium_filter_byte(&radio_medium, sender_idx, i, byte))
                continue;
            if (nodes[i].type == NODE_NATIVE) {
                native_rx_assembler_feed(&nodes[i].plat.native, byte);
            } else if (nodes[i].type == NODE_MSP430) {
                int8_t rssi = radio_medium_get_rssi(&radio_medium, sender_idx, i);
                if (trace_tsch_ack_enabled() &&
                    sender->type == NODE_MSP430 &&
                    nodes[sender_idx].id == 1 && nodes[i].id == 2) {
                    trace_tsch_ack_log("ack-byte queued 1->2 byte=%02x rx_state=%s node2_sim=%.6f",
                                       byte,
                                       cc2420_state_str(nodes[i].plat.msp.cc2420.state),
                                       (double)nodes[i].plat.msp.cpu.sim_time_ns / 1e9);
                }
                /* Match Cooja CustomDataRadio delivery: each transmitted byte
                 * is forwarded at the sender's current simulation time. */
                msp_rx_byte_queue_push(i, sender_idx, byte, rssi, current_sim_ns);
            } else {
                rf_buffer_t *buf = &rf_pending[i];
                if (buf->count < RF_BUF_SIZE)
                    buf->bytes[buf->count++] = byte;
            }
        }
        return;
    }

    /* Outer (non-reentrant) call: deliver bytes to receivers.
     * Match Cooja's CUSTOM_DATA_TRANSMITTED: each byte is delivered
     * immediately to the receiver at the sender's current sim time,
     * preceded by execute(t, 0) to sync the receiver's clock. */
    for (int i = 0; i < num_nodes; i++) {
        if (&nodes[i] == sender) continue;
        if (!node_active(i)) continue;
        if (sender->type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
            continue;
        if (!radio_medium_filter_byte(&radio_medium, sender_idx, i, byte))
            continue;
        if (nodes[i].type == NODE_NATIVE) {
            native_rx_assembler_feed(&nodes[i].plat.native, byte);
        } else if (nodes[i].type == NODE_MSP430) {
            int8_t rssi = radio_medium_get_rssi(&radio_medium, sender_idx, i);
            /* Match Cooja CustomDataRadio delivery: per-byte receive events
             * are scheduled at the current simulation time of the sender's
             * transmit callback, not with an extra receiver-side byte delay. */
            msp_rx_byte_queue_push(i, sender_idx, byte, rssi, current_sim_ns);
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

    /* Compute TX timestamps for timeline and RX delivery.
     * Use the sender's own radio time for the byte stream end time. */
    int total_air_bytes = 4 + 1 + 1 + tx_asm[sender_idx].expected_len;
    int64_t frame_air_dur = (int64_t)total_air_bytes * IEEE802154_BYTE_NS;
    int64_t accurate_tx_end = byte_time_ns + IEEE802154_BYTE_NS;
    int64_t accurate_tx_start = accurate_tx_end - frame_air_dur;
    if (accurate_tx_start < 0) accurate_tx_start = 0;

    if (trace_tsch_ack_enabled() && sender->type == NODE_MSP430 &&
        (tx_asm[sender_idx].expected_len == 23 || tx_asm[sender_idx].expected_len == 19 ||
         tx_asm[sender_idx].expected_len == 17)) {
        trace_tsch_ack_log("frame-complete sender=%d len=%d start=%.6f end=%.6f state=%s",
                           nodes[sender_idx].id, tx_asm[sender_idx].expected_len,
                           (double)accurate_tx_start / 1e9,
                           (double)accurate_tx_end / 1e9,
                           cc2420_state_str(sender->plat.msp.cc2420.state));
    }

    /* Emit explicit TX timeline event */
    if (ui_server) {
        tl_radio_event(&timeline, nodes[sender_idx].id, accurate_tx_start, TL_RADIO_TX);
        tl_radio_event(&timeline, nodes[sender_idx].id, accurate_tx_end, TL_RADIO_ON);
        node_states[sender_idx].radio_state = SIM_RADIO_ON;
    }

    /* Packet analysis: decode and log frame contents */
    if (ui_server || verbose) {
        uint8_t *buf = tx_cap[sender_idx].bytes;
        int buf_len = tx_cap[sender_idx].len;
        int fstart = -1;
        for (int b = 0; b + 5 < buf_len; b++) {
            if (buf[b] == 0x7A && b >= 4) {
                fstart = b + 1;
                break;
            }
        }
        if (fstart >= 0 && fstart < buf_len) {
            pkt_info_t pinfo;
            int frame_len = buf_len - fstart;
            pkt_analyze(buf + fstart, frame_len, &pinfo);
            if (verbose)
                fprintf(stderr, "  [PKT] Node %d TX: %s\n",
                        nodes[sender_idx].id, pinfo.summary);
            if (verbose && pinfo.dst_mode == 3 && frame_len < 120) {
                fprintf(stderr, "  [HEX] ");
                for (int h = 0; h < frame_len; h++)
                    fprintf(stderr, "%02x", buf[fstart + h]);
                fprintf(stderr, "\n");
            }
            if (verbose && nodes[sender_idx].type == NODE_MSP430 && nodes[sender_idx].id == 1) {
                uint32_t uip_buf_sym = msp430_elf_find_symbol(
                    nodes[sender_idx].firmware_path, "uip_aligned_buf");
                if (uip_buf_sym && uip_buf_sym + 40 < nodes[sender_idx].plat.msp.cpu.max_mem) {
                    uint8_t *ip6 = nodes[sender_idx].plat.msp.cpu.memory + uip_buf_sym;
                    uint16_t ulen = (ip6[4] << 8) | ip6[5];
                    uint8_t *cpumem = nodes[sender_idx].plat.msp.cpu.memory;
                    uint8_t pfx_len = cpumem[0x2964];
                    uint8_t inst_used = cpumem[0x2930];
                    fprintf(stderr, "  [UIP] dest=%02x%02x:...:%02x%02x nh=%d plen=%d pfxlen=%d inst=%d SRH@40=",
                        ip6[24],ip6[25],ip6[38],ip6[39], ip6[6], ulen,
                        pfx_len, inst_used);
                    for (int h = 40; h < 64; h++)
                        fprintf(stderr, "%02x", ip6[h]);
                    fprintf(stderr, "\n");
                }
            }
            if (ui_server)
                tl_frame_event(&timeline, nodes[sender_idx].id,
                               accurate_tx_start, 1, pinfo.summary);
        }
    }

    rf_tx_depth++;
    suppress_state_callback = 1;  /* explicit timeline events emitted above */

    /* The current sender is skipped by the receiver snapshot loop below, but
     * nested auto-ACK delivery buffers into rf_pending[sender_idx]. Ensure it
     * starts empty so ACK bytes cannot append onto stale contents. */
    rf_pending[sender_idx].count = 0;

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
            if (i == sender_idx || nodes[i].type == NODE_NATIVE || !node_active(i))
                continue;
            if (radio_medium_filter_byte(&radio_medium, sender_idx, i, 0))
                fprintf(stderr, " %d", nodes[i].id);
        }
        fprintf(stderr, "\n");
    }

    for (int i = 0; i < num_nodes; i++) {
        rf_buffer_t *buf = &rf_pending[i];
        if (i == sender_idx || nodes[i].type == NODE_NATIVE || nodes[i].type == NODE_MSP430 || buf->count == 0)
            continue;
        if (buf->count != total_air_bytes) {
            if (verbose) {
                fprintf(stderr,
                        "  [RF-BUF] dropping stale/misaligned buffer for node %d: "
                        "count=%d expected=%d sender=%d\n",
                        nodes[i].id, buf->count, total_air_bytes, nodes[sender_idx].id);
            }
            buf->count = 0;
            continue;
        }
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
            /* Match Cooja's setReceivedPacket: deliver ALL bytes starting
             * at the current sim time (TX completion time), spaced 32µs apart.
             * Pre-sync the receiver to this time first. */
            int64_t delivery_start = accurate_tx_start;
            if (trace_tsch_ack_enabled() &&
                sender->type == NODE_MSP430 && nodes[sender_idx].id == 2 &&
                nodes[i].type == NODE_MSP430 && nodes[i].id == 1 &&
                tx_asm[sender_idx].expected_len == 23) {
                trace_tsch_ack_log("deliver data 2->1 start=%.6f node1_state=%s",
                                   (double)delivery_start / 1e9,
                                   cc2420_state_str(nodes[i].plat.msp.cc2420.state));
            }
            if (nodes[i].type == NODE_MSP430 && i != ticking_node_idx) {
                tick_one_msp430(i, delivery_start);
            }
            emu_deliver_bytes(i, frame_snap[i], frame_snap_len[i],
                              frame_snap_rssi[i], delivery_start);
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
        int64_t ack_start = accurate_tx_end + 192000LL;
        int ack_tx_emitted = 0;
        for (int j = 0; j < num_nodes; j++) {
            if (rf_pending[j].count > 0 && nodes[j].type != NODE_NATIVE) {
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

    /* Schedule sender for wakeup so its CC2420 completes the TX->RX
     * transition. Use the node's actual next internal event timing
     * rather than a coarse fixed delay; enhanced ACK reception depends
     * on the sender re-entering RX on the normal CC2420 turnaround path. */
    if (nodes[sender_idx].type == NODE_MSP430 && num_threads == 0) {
        schedule_emulated_wakeup(&sim_eq, sender_idx);
    }
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
                /* Direct delivery to simInDataBuffer if possible.
                 * Force simRadioHWOn=1 to prevent firmware from dropping.
                 * Queue to rx_queue as fallback if simInSize > 0. */
                if (radio_medium_filter_frame(&radio_medium, sender_idx, i)) {
                    native_node_t *rcv = &nodes[i].plat.native;
                    if (*rcv->simInSize == 0) {
                        memcpy(rcv->simInDataBuffer, frame, (size_t)len);
                        *rcv->simInSize = len;
                        /* Set packet timestamp (TSCH uses this for sync) */
                        if (rcv->simLastPacketTimestamp)
                            *rcv->simLastPacketTimestamp =
                                (uint64_t)(current_sim_ns / 1000LL);
                    } else {
                        native_deliver_frame(rcv, frame, len,
                                             current_sim_ns, sender_idx);
                    }
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
 * Drain queued RX frames for an emulated node. Deliver at most one frame per
 * call and let the normal event queue provide the immediate wakeup.
 */
static void emu_rx_queue_drain(int idx) {
    emu_rx_queue_t *q = &emu_rx_queue[idx];
    int blocked_attempts = 0;
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
        if (emulated_rxfifo_available(idx) < frame_payload + 1) {
            if (nodes[idx].type == NODE_MSP430 && blocked_attempts < 4) {
                int64_t freq = node_freq(idx);
                int64_t spare_cycles = freq / 1000;
                if (spare_cycles <= 0) spare_cycles = 1;
                step_node_until(idx, node_cycles(idx) + spare_cycles);
                blocked_attempts++;
                continue;
            }
            break;  /* RXFIFO still busy, try next time step */
        }
        blocked_attempts = 0;

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

        break;
    }
}

/* --- UART callback --- */

static void mixed_uart_callback(void *user_data, uint8_t byte) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    uart_byte_count++;
    /* UART byte received */

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

/* Serial socket UART callback: same as mixed_uart_callback but also buffers
 * bytes for TCP transmission to the external process (tunslip6). */
static void serial_socket_uart_callback(void *user_data, uint8_t byte) {
    /* Normal UART processing (console, test matching) */
    mixed_uart_callback(user_data, byte);

    /* Buffer byte for TCP transmission */
    if (ss_client_fd >= 0 && ss_tx_count < SS_TX_BUF_SIZE) {
        int tail = (ss_tx_head + ss_tx_count) % SS_TX_BUF_SIZE;
        ss_tx_buf[tail] = byte;
        ss_tx_count++;
    }
}

/* --- Serial socket TCP helpers --- */

static int ss_create_listener(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("serial_socket: socket");
        return -1;
    }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("serial_socket: bind");
        close(fd);
        return -1;
    }
    if (listen(fd, 1) < 0) {
        perror("serial_socket: listen");
        close(fd);
        return -1;
    }

    /* Non-blocking */
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

    printf("  Serial socket listening on port %d\n", port);
    return fd;
}

static void ss_accept(void) {
    if (ss_listen_fd < 0 || ss_client_fd >= 0) return;

    int fd = accept(ss_listen_fd, NULL, NULL);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("serial_socket: accept");
        return;
    }

    /* Set non-blocking */
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    ss_client_fd = fd;
    printf("  Serial socket: client connected\n");
}

/* Flush buffered TX bytes to TCP socket */
static void ss_flush_tx(void) {
    if (ss_client_fd < 0 || ss_tx_count == 0) return;

    while (ss_tx_count > 0) {
        /* Contiguous chunk from head */
        int chunk = ss_tx_count;
        if (ss_tx_head + chunk > SS_TX_BUF_SIZE)
            chunk = SS_TX_BUF_SIZE - ss_tx_head;

        ssize_t n = write(ss_client_fd, ss_tx_buf + ss_tx_head, (size_t)chunk);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                break;
            /* Connection closed or error */
            printf("  Serial socket: client disconnected (write)\n");
            close(ss_client_fd);
            ss_client_fd = -1;
            ss_tx_count = 0;
            ss_tx_head = 0;
            return;
        }
        ss_tx_head = (ss_tx_head + (int)n) % SS_TX_BUF_SIZE;
        ss_tx_count -= (int)n;
    }
}

/* Read TCP data into RX buffer */
static void ss_read_tcp(void) {
    if (ss_client_fd < 0) return;

    /* Read into ring buffer */
    while (ss_rx_count < SS_RX_BUF_SIZE) {
        int tail = (ss_rx_head + ss_rx_count) % SS_RX_BUF_SIZE;
        int space = SS_RX_BUF_SIZE - ss_rx_count;
        /* Contiguous chunk from tail */
        int chunk = SS_RX_BUF_SIZE - tail;
        if (chunk > space) chunk = space;

        ssize_t n = read(ss_client_fd, ss_rx_buf + tail, (size_t)chunk);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            printf("  Serial socket: client disconnected (read)\n");
            close(ss_client_fd);
            ss_client_fd = -1;
            return;
        }
        ss_rx_count += (int)n;
        break;  /* one read per poll */
    }
}

/* Drain RX buffer into bridged mote's serial input.
 * For native motes: only inject when simSerialReceivingFlag==0 (mote consumed
 * previous batch). For emulated motes: inject byte-by-byte via UART RX. */
static void ss_inject_serial(void) {
    if (ss_node_idx < 0 || ss_rx_count == 0) return;

    mixed_node_t *node = &nodes[ss_node_idx];
    if (node->type == NODE_NATIVE) {
        native_node_t *nat = &node->plat.native;
        if (!nat->simSerialReceivingData ||
            !nat->simSerialReceivingLength ||
            !nat->simSerialReceivingFlag) return;

        /* Match COOJA ContikiRS232: append new bytes into the mote's pending
         * serial buffer even when simSerialReceivingFlag is already set, then
         * request an immediate wakeup so the mote drains that buffer. The
         * cooja rs232 backend has a fixed 2048-byte receive buffer. */
        int old_size = *nat->simSerialReceivingLength;
        if (old_size < 0) old_size = 0;
        if (old_size > 2048) old_size = 2048;
        int space = 2048 - old_size;
        if (space <= 0) {
            if (num_threads == 0) {
                int64_t wake_ns = nat->sim_time_ns;
                if (node_start_ns[ss_node_idx] > wake_ns)
                    wake_ns = node_start_ns[ss_node_idx];
                sim_eq_schedule_if_earlier(&sim_eq, ss_node_idx, wake_ns);
            }
            return;
        }

        int to_send = ss_rx_count;
        if (to_send > space) to_send = space;
        for (int i = 0; i < to_send; i++) {
            nat->simSerialReceivingData[old_size + i] =
                (char)ss_rx_buf[(ss_rx_head + i) % SS_RX_BUF_SIZE];
        }
        *nat->simSerialReceivingLength = old_size + to_send;
        *nat->simSerialReceivingFlag = 1;
        if (nat->simProcessRunValue)
            *nat->simProcessRunValue = 1;
        ss_rx_head = (ss_rx_head + to_send) % SS_RX_BUF_SIZE;
        ss_rx_count -= to_send;
        /* Match COOJA's external serial delivery contract: once serial data
         * is injected, the mote must be re-run at the current simulation
         * time rather than waiting for a stale timer-based wakeup. This is
         * the native equivalent of the immediate reschedule used for MSP430
         * serial injection below. */
        if (num_threads == 0) {
            int64_t wake_ns = nat->sim_time_ns;
            if (node_start_ns[ss_node_idx] > wake_ns)
                wake_ns = node_start_ns[ss_node_idx];
            sim_eq_schedule_if_earlier(&sim_eq, ss_node_idx, wake_ns);
        }
    } else if (node->type == NODE_MSP430) {
        /* Match MSPSim's MspSerial: inject ALL pending bytes at baud-rate
         * intervals (69µs = 115200 baud), stepping CPU between each byte.
         * Cooja queues all bytes and delivers them at baud-rate intervals
         * with requestImmediateWakeup() after each.
         *
         * Set ticking_node_idx to prevent re-entrant cascade: if the CPU
         * TX's during the baud-rate step, the delivery path must not
         * sync this node back (same guard as tick_one_msp430). */
        msp430_platform_t *plat = &node->plat.msp;
        msp430_usart_t *console = (plat->config->console_usart == 0)
            ? &plat->usart0 : &plat->usart1;
        bool has_dma = (console->dma_trigger != NULL);
        if (!has_dma) {
            if (!console->ie_ptr ||
                !(*console->ie_ptr & console->ie_rx_mask))
                return;  /* firmware hasn't enabled UART RX yet */
        }
        int baud_cycles = (int)((int64_t)node_freq(ss_node_idx) * 69 / 1000000);
        if (baud_cycles < 200) baud_cycles = 200;
        ticking_node_idx = ss_node_idx;
        while (ss_rx_count > 0) {
            if (!has_dma) {
                /* Wait for RXIFG clear (firmware consumed previous byte) */
                if (console->ifg_ptr &&
                    (*console->ifg_ptr & console->ifg_rx_mask))
                    break;  /* try again next poll */
            }
            msp430_usart_receive_byte(console, ss_rx_buf[ss_rx_head]);
            ss_rx_head = (ss_rx_head + 1) % SS_RX_BUF_SIZE;
            ss_rx_count--;
            step_node_until(ss_node_idx,
                node_cycles(ss_node_idx) + baud_cycles);
        }
        ticking_node_idx = -1;
        /* Sync last_execute_us and re-schedule in event queue.
         * Without this, the node's sim_eq entry is stale and the
         * trickle timer / other events scheduled during injection
         * won't fire until the old wakeup time.  Matches Cooja's
         * requestImmediateWakeup() after serial byte delivery. */
        {
            msp430_cpu_t *cpu = &node->plat.msp.cpu;
            cpu->last_execute_us = (int64_t)msp430_cycles_to_ns(
                cpu->cycles, cpu->cpu_freq_hz) / 1000LL;
            if (num_threads == 0)
                schedule_emulated_wakeup(&sim_eq, ss_node_idx);
        }
    } else if (node->type == NODE_ARM) {
        while (ss_rx_count > 0) {
            cc2538_uart_receive_byte(&node->plat.arm.uart0, ss_rx_buf[ss_rx_head]);
            ss_rx_head = (ss_rx_head + 1) % SS_RX_BUF_SIZE;
            ss_rx_count--;
        }
    }
}

/* Launch external command as child process */
static pid_t ss_launch_command(const char *command) {
    if (!command || !command[0]) return -1;

    pid_t pid = fork();
    if (pid < 0) {
        perror("serial_socket: fork");
        return -1;
    }
    if (pid == 0) {
        /* Child: exec the command via shell */
        execl("/bin/sh", "sh", "-c", command, (char *)NULL);
        perror("serial_socket: exec");
        _exit(127);
    }

    printf("  Serial socket: launched command (pid %d): %s\n", pid, command);
    return pid;
}

/* Check if child process has exited (non-blocking) */
static void ss_check_child(void) {
    if (ss_child_pid <= 0 || ss_child_exited) return;

    int status;
    pid_t res = waitpid(ss_child_pid, &status, WNOHANG);
    if (res > 0) {
        ss_child_exited = 1;
        if (WIFEXITED(status)) {
            ss_child_status = WEXITSTATUS(status);
            printf("  Serial socket: command exited with status %d\n",
                   ss_child_status);
        } else {
            ss_child_status = 1;
            printf("  Serial socket: command killed by signal\n");
        }
    }
}

static void ss_cleanup(void) {
    if (ss_client_fd >= 0) { close(ss_client_fd); ss_client_fd = -1; }
    if (ss_listen_fd >= 0) { close(ss_listen_fd); ss_listen_fd = -1; }
    if (ss_child_pid > 0 && !ss_child_exited) {
        kill(ss_child_pid, SIGTERM);
        usleep(100000);
        kill(ss_child_pid, SIGKILL);
        waitpid(ss_child_pid, NULL, 0);
        ss_child_pid = -1;
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

    /* Derive platform from firmware extension: .sky -> sky, .z1 -> z1 */
    const char *dot = strrchr(firmware_path, '.');
    const char *plat_name = "sky";
    if (dot) {
        if (strcmp(dot, ".z1") == 0) plat_name = "z1";
        else if (strcmp(dot, ".esb") == 0) plat_name = "esb";
        else if (strcmp(dot, ".wismote") == 0) plat_name = "wismote";
        else if (strcmp(dot, ".exp5438") == 0) plat_name = "exp5438";
    }
    const msp430_platform_config_t *pcfg = msp430_platform_find(plat_name);
    if (!pcfg) { fprintf(stderr, "Platform '%s' not found\n", plat_name); return -1; }

    msp430_platform_init(plat, pcfg);

    /* Disable JIT for multinode — the event-driven scheduler steps in
     * small increments (~1µs) where JIT overhead exceeds any benefit.
     * The interpreter also avoids the inblock-check performance cost. */
#ifdef HAVE_LIGHTNING
    if (plat->cpu.compiled_cache) {
        free(plat->cpu.compiled_cache);
        plat->cpu.compiled_cache = NULL;
    }
#endif

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

    /* Patch xmem_init() and node_id_z1_restore() to RET on MSP430X.
     * Without file-backed flash storage, xmem reads return garbage
     * and node_id_z1_restore sets node_id=0 + empty linkaddr, which
     * causes printf to output 70+ zero bytes overflowing the stack. */
    if (plat->cpu.config->is_msp430x) {
        const char *patch_fns[] = {"xmem_init", "node_id_z1_restore",
                                   NULL};
        for (int p = 0; patch_fns[p]; p++) {
            uint32_t addr = msp430_elf_find_symbol(firmware_path, patch_fns[p]);
            if (addr != 0) {
                plat->cpu.memory[addr]     = 0x10;  /* RETA */
                plat->cpu.memory[addr + 1] = 0x01;
                printf("  Patched %s at 0x%04x to RETA\n", patch_fns[p], addr);
            }
        }
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

    /* Patch ds2411_id with unique address (Sky/classic MSP430).
     * Format: 00:12:74:id:00:id:id:id
     * Firmware does ds2411_id[2] &= 0xfe (0x74 is already even).
     * With IPv6: linkaddr = memcpy(ds2411_id, 8)
     * shortaddr = (linkaddr[0] << 8) | linkaddr[1] = 0x0012
     * IPv6 IID = 0212:74id:00id:idid → fd00::0212:74id:00id:idid
     * This matches Cooja's test CSC ping targets (e.g. fd00::0212:7404:0004:0404). */
    uint32_t ds2411_addr = msp430_elf_find_symbol(firmware_path, "ds2411_id");
    if (ds2411_addr != 0) {
        uint8_t *id = plat->cpu.memory + ds2411_addr;
        id[0] = 0x00; id[1] = 0x12; id[2] = 0x74;
        id[3] = (uint8_t)(node_id & 0xff);
        id[4] = (uint8_t)((node_id >> 8) & 0xff);
        id[5] = (uint8_t)(node_id & 0xff);
        id[6] = (uint8_t)(node_id & 0xff);
        id[7] = (uint8_t)(node_id & 0xff);
        printf("  Patched ds2411_id at 0x%04x for node %d: ", ds2411_addr, node_id);
        for (int i = 0; i < 8; i++) printf("%02x%s", id[i], i < 7 ? ":" : "\n");
    }

    /* Write infomem at 0x1980 — this is how Cooja's MspMoteID sets the
     * node ID and MAC address. The firmware reads infomem during init
     * to set node_id and linkaddr. Format: ABCD:0212:7400:0001:HI:LO */
    {
        uint32_t infomem_addr = 0x1980;
        if (infomem_addr + 10 <= plat->cpu.max_mem) {
            uint8_t *im = plat->cpu.memory + infomem_addr;
            im[0] = 0xAB; im[1] = 0xCD;  /* magic */
            im[2] = 0x02; im[3] = 0x12; im[4] = 0x74;
            im[5] = 0x00; im[6] = 0x00; im[7] = 0x01;
            im[8] = (uint8_t)((node_id >> 8) & 0xFF);
            im[9] = (uint8_t)(node_id & 0xFF);
            printf("  Patched infomem at 0x%04x for node %d: ", infomem_addr, node_id);
            for (int i = 0; i < 10; i++) printf("%02x%s", im[i], i < 9 ? ":" : "\n");
        }
    }

    /* Write node_id variable directly (like Cooja's MspMoteID.setMoteID).
     * The firmware uses node_id for RPL and application logic.
     * Must be written AFTER crt0 clears BSS but BEFORE platform_init. */
    {
        uint32_t nid = msp430_elf_find_symbol(firmware_path, "node_id");
        if (nid != 0 && nid + 2 <= plat->cpu.max_mem) {
            plat->cpu.memory[nid] = (uint8_t)(node_id & 0xFF);
            plat->cpu.memory[nid + 1] = (uint8_t)((node_id >> 8) & 0xFF);
            printf("  Patched node_id at 0x%04x = %d\n", nid, node_id);
        }
    }

    /* Patch linkaddr_node_addr and node_id for Z1/MSP430X.
     * Must happen AFTER crt0 clears BSS (run-to-main above) but
     * BEFORE platform_init reads them. */
    if (plat->cpu.config->is_msp430x) {
        const char *addr_syms[] = {"linkaddr_node_addr", "node_mac", "uip_lladdr", NULL};
        for (int a = 0; addr_syms[a]; a++) {
            uint32_t sym = msp430_elf_find_symbol(firmware_path, addr_syms[a]);
            if (sym != 0) {
                uint8_t *p = plat->cpu.memory + sym;
                /* Use same IEEE format as Cooja MspMote: c1:0c:00:00:00:00:00:id
                 * First byte must be non-zero or Z1 platform overwrites it */
                p[0] = 0xc1; p[1] = 0x0c;
                p[2] = 0x00; p[3] = 0x00;
                p[4] = 0x00; p[5] = 0x00;
                p[6] = 0x00; p[7] = (uint8_t)node_id;
            }
        }
        uint32_t nid_addr = msp430_elf_find_symbol(firmware_path, "node_id");
        if (nid_addr != 0) {
            plat->cpu.memory[nid_addr] = (uint8_t)node_id;
            plat->cpu.memory[nid_addr + 1] = 0;
        }
        printf("  Patched node_id=%d, linkaddr, node_mac for Z1\n", node_id);
    }


    /* Run past main() entry through DCO calibration and platform init.
     * Must stop AFTER TimerA is running with events scheduled AND
     * CC2420 VREG enabled (if platform has CC2420). On Z1, clock_init
     * starts Timer A before cc2420_init enables VREG, so we need both
     * conditions. Use step batches of 10K for speed. */
    {
        int64_t limit = plat->cpu.cycles + 8000000;
        bool has_cc2420 = plat->config->cc2420.has_cc2420;
        while ((int64_t)plat->cpu.cycles < limit) {
            msp430_step_until(&plat->cpu, plat->cpu.cycles + 10000);
            bool timer_ok = plat->timer_a.mode != 0 && plat->cpu.event_queue != NULL;
            bool radio_ok = !has_cc2420 || plat->cc2420.state != CC2420_VREG_OFF;
            if (timer_ok && radio_ok) break;
        }
    }

    /* Note: no extended init needed for MSP430X. The event order fix
     * (events after instructions) and CCR0 reschedule fix allow the
     * clock ISR (CCR1) to fire normally during simulation, advancing
     * etimers and enabling process_run to execute. */

    printf("  Node %d [MSP430] initialized: PC=0x%04x SP=0x%04x cycles=%lld eq=%s next_ev=%lld\n",
           node_id, plat->cpu.reg[0], plat->cpu.reg[1],
           (long long)plat->cpu.cycles,
           plat->cpu.event_queue ? "yes" : "nil",
           (long long)plat->cpu.next_event_cycle);

    /* MSP430X: patch IPv6 link-local address into uip_ds6_if addr_list.
     * The firmware's uip_ds6_addr_add can't find free list entries on MSP430X
     * because after memset to 0, the compiled code's isused check (at entry
     * offset 16) treats 0 as "in use". This matches Cooja's MspMote which
     * configures node addresses externally. */
    if (plat->cpu.config->is_msp430x) {
        uint32_t ds6 = msp430_elf_find_symbol(firmware_path, "uip_ds6_if");
        uint32_t lla = msp430_elf_find_symbol(firmware_path, "uip_lladdr");
        if (ds6 && lla) {
            uint8_t *mem = plat->cpu.memory;
            uint8_t ll[8];
            memcpy(ll, &mem[lla], 8);
            /* fe80::<IID> with U/L bit toggled */
            uint8_t ip[16] = {0xfe,0x80, 0,0, 0,0, 0,0,
                              ll[0]^0x02,ll[1],ll[2],ll[3],ll[4],ll[5],ll[6],ll[7]};
            /* Entry layout: ipaddr(16) + isused(1) + state(1) + ... = 28 bytes.
             * Header before addr_list: 18 bytes. */
            uint32_t e = ds6 + 18;
            memcpy(&mem[e], ip, 16);
            mem[e + 16] = 0xFF;  /* isused = non-zero */
            mem[e + 17] = 0x02;  /* state = PREFERRED */
            printf("  Patched IPv6 fe80::%02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
                   ip[8],ip[9],ip[10],ip[11],ip[12],ip[13],ip[14],ip[15]);

            /* Patch rpl_has_joined to always return true (RETA with R15=1).
             * This bypasses the RPL DODAG check for TSCH EB generation. */
            uint32_t rhj = msp430_elf_find_symbol(firmware_path, "rpl_has_joined");
            if (rhj) {
                mem[rhj + 0] = 0x1F;  /* MOV #1, R15 (0x431F) */
                mem[rhj + 1] = 0x43;
                mem[rhj + 2] = 0x10;  /* RETA (0x0110) */
                mem[rhj + 3] = 0x01;
                printf("  Patched rpl_has_joined → always true\n");
            }
        }
    }

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
    memset(&tx_cap[idx], 0, sizeof(tx_cap[idx]));
    msp_rx_byte_queue_remove_node(idx);
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
    memset(&tx_cap[idx], 0, sizeof(tx_cap[idx]));
    msp_rx_byte_queue_remove_node(idx);
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

static int64_t tick_one_msp430(int idx, int64_t sim_ns) {
    msp430_cpu_t *cpu = &nodes[idx].plat.msp.cpu;
    int64_t t_us = sim_ns / 1000LL;
    int64_t jump_us = 0;

    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }

    /* Apply clock deviation (Cooja MspClock drift simulation) */
    double deviation = nodes[idx].clock_deviation;
    if (deviation != 1.0 && jump_us > 0) {
        double exact = (double)jump_us * deviation;
        jump_us = (int64_t)exact;
        /* Track sub-µs remainder (Cooja jumpError) */
        cpu->step_cycle_remainder += exact - (double)jump_us;
        if (cpu->step_cycle_remainder > 1.0) {
            jump_us++;
            cpu->step_cycle_remainder -= 1.0;
        }
    }

    /* Match Cooja's MspMote.execute(t, duration): peripheral events raised
     * during this execute slice should be scheduled relative to the current
     * scheduler time t, not a cycle-derived local time that may have drifted.
     * Keep the CPU's event-time view pinned to sim_ns across the slice. */
    cpu->sim_time_ns = sim_ns;
    int64_t returned_us = msp430_step_micros(cpu, jump_us, 1);
    cpu->sim_time_ns = sim_ns;

    if (deviation != 1.0 && returned_us > 0)
        returned_us = (int64_t)((double)returned_us / deviation);

    cpu->last_execute_us = t_us;
    return returned_us;
}

/* --- Event-driven scheduling helpers (COOJA model) --- */

/* Tick a single native node at sim_ns. Single tick, no loop.
 * Matches COOJA's ContikiMote.execute(). */
/* Track whether the last tick had a TX (for distinguishing TX yield
 * from TSCH busywait in schedule_native_wakeup). */
static bool native_had_tx[MAX_NODES];

static void tick_one_native(int idx, int64_t sim_ns) {
    native_node_t *nat = &nodes[idx].plat.native;
    nat->sim_time_ns = sim_ns;
    *nat->simCurrentTime = (uint64_t)(sim_ns / 1000000LL);
    *nat->simRtimerCurrentTicks = (uint64_t)(sim_ns / 1000LL);

    if (nat->radio_is_transmitting && sim_ns >= nat->radio_tx_end_ns) {
        nat->radio_is_transmitting = false;
        nat->radio_tx_finished = true;
        *nat->simOutSize = 0;
        if (nat->simProcessRunValue) {
            *nat->simProcessRunValue = 1;
        }
    } else {
        nat->radio_tx_finished = false;
    }

    /* Deliver queued RX frame BEFORE tick.
     * Force simReceiving=0 so pending_packet() returns true.
     * Only dequeue when the radio is on — TSCH turns radio off between
     * slots, and doInterfaceActionsBeforeTick drops frames when off.
     * Keeping the frame in the queue lets it be delivered on a later
     * tick when the radio is in an active RX slot. */
    if (*nat->simReceiving)
        *nat->simReceiving = 0;
    bool radio_on = !nat->simRadioHWOn || *nat->simRadioHWOn;
    if (*nat->simInSize == 0 && nat->rx_queue.count > 0 && radio_on)
        native_dequeue_rx_frame(nat);
    int pre_insize = *nat->simInSize;
    nat->cooja_tick();
    native_had_tx[idx] = (*nat->simOutSize > 0);
    native_check_radio_tx(nat);
    native_check_log_output(nat);
    /* Reset signal strength when frame is consumed (signalReceptionEnd) */
    if (pre_insize > 0 && *nat->simInSize == 0) {
        if (nat->simSignalStrength)
            *nat->simSignalStrength = -100;
        if (verbose)
            fprintf(stderr, "  [CONSUMED] node %d consumed %d-byte frame at %lld ms\n",
                    nodes[idx].id, pre_insize, (long long)(nat->sim_time_ns / 1000000LL));
    }

    /* Sync this node's channel (for TSCH hopping) */
    if (nat->simRadioChannel)
        radio_medium_set_channel(&radio_medium, idx, *nat->simRadioChannel);
}

/* Schedule a native node's next wakeup in the event queue.
 * Exactly matches COOJA's ContikiClock.doActionsAfterTick(). */
static void schedule_native_wakeup(sim_event_queue_t *eq, int idx) {
    native_node_t *nat = &nodes[idx].plat.native;
    int64_t now = nat->sim_time_ns;

    /* Determine next wakeup time (like ContikiClock.doActionsAfterTick).
     * Use schedule_if_earlier to avoid overriding a requestImmediateWakeup
     * that was already scheduled by RF delivery detection. */
    int64_t next = now + 1000000000LL;  /* 1s max gap */

    /* rtimer has priority (exact µs timing) */
    if (*nat->simRtimerPending) {
        int64_t rt = (int64_t)(*nat->simRtimerNextExpirationTime) * 1000LL;
        if (rt <= now) rt = now;
        if (rt < next) next = rt;
    }

    /* ContikiRadio.doActionsAfterTick(): new transmissions keep the mote
     * scheduled until the exact transmission-end time, and completion
     * requests an immediate same-time wakeup. */
    if (nat->radio_tx_finished) {
        next = now;
        nat->radio_tx_finished = false;
        sim_eq_schedule_if_earlier(eq, idx, next);
        return;
    }
    if (nat->radio_is_transmitting) {
        int64_t tx_next = nat->radio_tx_end_ns;
        if (tx_next < next) next = tx_next;
    } else if (native_had_tx[idx]) {
        int64_t tx_next = now + 1000000LL;
        if (tx_next < next) next = tx_next;
    }
    native_had_tx[idx] = false;

    /* processRunValue → +1ms (like COOJA's ContikiClock.doActionsAfterTick) */
    if (*nat->simProcessRunValue) {
        int64_t prv = now + 1000000LL;
        if (prv < next) next = prv;
    }

    /* etimer */
    if (*nat->simEtimerPending) {
        int64_t et = (int64_t)(*nat->simEtimerNextExpirationTime) * 1000000LL;
        if (et <= now) et = now + 1000000LL;  /* stale → +1ms */
        if (et < next) next = et;
    }

    /* If rx_queue has frames, schedule +1ms to ensure TSCH slot
     * operation gets a chance to find the frame via pending_packet(). */
    if (nat->rx_queue.count > 0 || *nat->simInSize > 0) {
        int64_t rx_next = now + 1000000LL;
        if (rx_next < next) next = rx_next;
    }

    sim_eq_schedule_if_earlier(eq, idx, next);
}

/* Schedule an emulated node's next wakeup */
static void schedule_emulated_wakeup(sim_event_queue_t *eq, int idx) {
    int64_t next;
    if (emu_rx_queue[idx].count > 0) {
        /* Match Cooja requestImmediateWakeup(): re-run the mote at the
         * current simulation time, not one microsecond later. */
        next = node_sim_time_ns(idx);
        sim_eq_schedule_if_earlier(eq, idx, next);
        return;
    }
    if (nodes[idx].type == NODE_ARM) {
        arm_cpu_t *cpu = &nodes[idx].plat.arm.cpu;
        if (cpu->next_event_cycle <= cpu->cycles)
            next = cpu->sim_time_ns;
        else
            next = cpu->sim_time_ns + arm_cycles_to_ns(
                cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
    } else {
        msp430_cpu_t *cpu = &nodes[idx].plat.msp.cpu;
        if ((cpu->interrupts_enabled &&
             cpu->serviced_interrupt == -1 &&
             cpu->interrupt_max >= 0) ||
            cpu->next_event_cycle <= cpu->cycles) {
            next = cpu->sim_time_ns;
        } else {
            next = cpu->sim_time_ns + msp430_cycles_to_ns(
                cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
        }
    }
    sim_eq_schedule_if_earlier(eq, idx, next);
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
        if (emu_rx_queue[idx].count > 0)
            return cpu->sim_time_ns;
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
        /* Clock deviation from config (Cooja MspClock deviation) */
        nodes[i].clock_deviation = (config_loaded && i < config.node_count &&
                                    config.nodes[i].clock_deviation > 0.0)
                                   ? config.nodes[i].clock_deviation : 1.0;
        nodes[i].last_execute_ns = 0;
        if (nodes[i].clock_deviation != 1.0)
            printf("  Node %d: clock deviation=%.10f\n", nodes[i].id, nodes[i].clock_deviation);
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
                 * allowing js_test_check_timeout to fire the callback.
                 * If no TIMEOUT but GENERATE_MSG is used, compute duration
                 * from the last scheduled message + margin. */
                if (js_engine.timeout_us > 0) {
                    total_ns = js_engine.timeout_us * 1000LL + 10 * MS_TO_NS;
                } else if (js_engine.gen_msg_count > 0) {
                    /* No TIMEOUT — use last GENERATE_MSG time + 60s margin */
                    int64_t last_gen = 0;
                    for (int g = 0; g < js_engine.gen_msg_count; g++)
                        if (js_engine.gen_msgs[g].at_us > last_gen)
                            last_gen = js_engine.gen_msgs[g].at_us;
                    total_ns = last_gen * 1000LL + 60000LL * MS_TO_NS;
                }
                printf("Test: JavaScript engine (timeout=%lld ms)\n",
                       (long long)(total_ns / MS_TO_NS));
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

    /* Set up serial socket server for border-router tests */
    if (config_loaded && config.has_serial_socket) {
        /* Find the node index for the bridged node */
        for (int i = 0; i < node_count; i++) {
            if (nodes[i].id == config.serial_socket_node) {
                ss_node_idx = i;
                break;
            }
        }
        if (ss_node_idx < 0) {
            fprintf(stderr, "serial_socket: node %d not found\n",
                    config.serial_socket_node);
        } else {
            /* Replace the UART callback for this node with the TCP-bridging one */
            mixed_node_t *sn = &nodes[ss_node_idx];
            if (sn->type == NODE_NATIVE) {
                sn->plat.native.log_callback = serial_socket_uart_callback;
            } else if (sn->type == NODE_MSP430) {
                msp430_platform_set_console(&sn->plat.msp,
                    serial_socket_uart_callback, sn);
            } else if (sn->type == NODE_ARM) {
                arm_platform_set_console(&sn->plat.arm,
                    serial_socket_uart_callback, sn);
            }

            /* Create TCP server */
            ss_listen_fd = ss_create_listener(config.serial_socket_port);
            if (ss_listen_fd < 0) {
                fprintf(stderr, "serial_socket: failed to create listener\n");
            } else if (config.serial_socket_command[0]) {
                /* Launch the external command */
                ss_child_pid = ss_launch_command(config.serial_socket_command);
            }
        }
    }

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

            /* For ARM nodes: shift sim_time_ns and cycles forward so
             * the sleep timer shows different elapsed times per node. */
            if (nodes[i].type == NODE_ARM) {
                arm_cpu_t *cpu = &nodes[i].plat.arm.cpu;
                cpu->sim_time_ns += delay_ns;
                cpu->cycles += delay_ns * cpu->cpu_freq_hz / 1000000000LL;
            } else if (nodes[i].type == NODE_NATIVE) {
                native_node_t *nat = &nodes[i].plat.native;
                /* Native/cooja motes are initialized at t=0 before the
                 * randomized startup spread is applied. Shift any absolute
                 * pending timer deadlines by the same startup delay so their
                 * first wakeup is anchored to the delayed start time, not the
                 * boot-time zero point. */
                if (nat->simEtimerPending && *nat->simEtimerPending &&
                    nat->simEtimerNextExpirationTime) {
                    *nat->simEtimerNextExpirationTime += (uint64_t)delay_ms;
                }
                if (nat->simRtimerPending && *nat->simRtimerPending &&
                    nat->simRtimerNextExpirationTime) {
                    *nat->simRtimerNextExpirationTime += (uint64_t)(delay_ns / 1000LL);
                }
            }
            /* For all node types: set start_ns so the node isn't stepped
             * until its delay has elapsed.  Don't shift MSP430 internals
             * — just start it later. */
            node_start_ns[i] = node_sim_time_ns(i) + delay_ns;
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
    /* Verify trace is set */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type == NODE_MSP430 && nodes[i].plat.msp.cpu.pc_trace_fn)
            fprintf(stderr, "  [DBG] Node %d pc_trace_fn=%p lo=0x%x hi=0x%x\n",
                nodes[i].id, (void*)nodes[i].plat.msp.cpu.pc_trace_fn,
                nodes[i].plat.msp.cpu.pc_trace_lo, nodes[i].plat.msp.cpu.pc_trace_hi);
    }

    /* Debug: PC trace for cc2420_transmit + TSCH EB on all MSP430 nodes */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type == NODE_MSP430) {
            uint32_t tx_addr = msp430_elf_find_symbol(
                nodes[i].firmware_path, "cc2420_transmit");
            if (tx_addr) {
                srh_trace_fn_addr = tx_addr;
                cc2420_transmit_count = 0;
                tsch_eb_process_count = 0;
                tsch_queue_add_count = 0;
                /* Set range to cover all code including memcpy in .rodata area */
                nodes[i].plat.msp.cpu.pc_trace_lo = 0x3d00;
                nodes[i].plat.msp.cpu.pc_trace_hi = 0x10000;
                nodes[i].plat.msp.cpu.pc_trace_fn = srh_trace_cb;
                printf("  PC trace: cc2420_transmit=0x%04x eb_process=0xcb32 queue_add=0xb138 (Node %d)\n",
                       tx_addr, nodes[i].id);
            }
        }
    }

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
    int64_t spin_trace_time_ns = INT64_MIN;
    uint64_t spin_trace_count = 0;
    int64_t ui_interval_ns = 100LL * MS_TO_NS;  /* 100ms sim time between UI updates */
    int64_t next_ui_ns = sim_ns + ui_interval_ns;

    /* Initialize event queue for COOJA-model sequential stepping.
     * File-scope so emu_deliver_bytes() can schedule receivers. */
    sim_eq_init(&sim_eq);
    memset(&msp_rx_byte_queue, 0, sizeof(msp_rx_byte_queue));
    if (num_threads == 0) {
        for (int i = 0; i < node_count; i++) {
            if (node_start_ns[i] >= INT64_MAX) continue;  /* removed node */
            sim_eq_schedule(&sim_eq, i, node_start_ns[i]);
        }
    }



    /* Phase timing accumulators (ms) */
    double time_distribute = 0, time_deliver = 0, time_step = 0;
    double time_flush = 0, time_channel_sync = 0, time_test_ui = 0;

    double t_start = get_time_ms();

    int ss_has_command = (ss_child_pid > 0);
    while (sim_ns < end_ns || ui_server || (ss_listen_fd >= 0 && ss_has_command)) {
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
            int64_t max_ns = ui_server ? sim_ns + 100LL * MS_TO_NS
                : ss_listen_fd >= 0 ? sim_ns + TIME_STEP_NS
                : end_ns;
            if (next_event < max_ns) max_ns = next_event;
            if (max_ns <= sim_ns) max_ns = sim_ns + 1000;  /* min 1µs advance */
            sim_ns = max_ns;
        }
        current_sim_ns = sim_ns;

        /* Cooja model: each node schedules its own next wakeup via sim_eq.
         * No explicit "step all nodes" — the event queue ensures all active
         * nodes get regular ticks through schedule_emulated_wakeup(). */

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
                    if (verbose)
                        printf("  JS ACTION: add node %d (type=%d, types=%d)\n",
                               act->node, act->mote_type, config.mote_type_count);
                    int found = -1;
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) { found = i; break; }
                    }
                    if (found < 0 && node_count < MAX_NODES) {
                        /* Dynamic node creation using mote_types from config */
                        int type_idx = act->mote_type;
                        const char *fw = NULL;
                        if (type_idx >= 0 && type_idx < config.mote_type_count &&
                            config.mote_type_firmware[type_idx][0]) {
                            fw = config.mote_type_firmware[type_idx];
                        }
                        if (fw) {
                            found = node_count;
                            nodes[found].id = act->node;
                            if (init_node(found, fw, act->node) == 0) {
                                node_count++;
                                radio_medium.node_count = node_count;
                                if (act->x != 0.0 || act->y != 0.0)
                                    radio_medium_set_position(&radio_medium, found, act->x, act->y);
                                radio_medium_compute_neighbors(&radio_medium);
                                if (verbose)
                                    printf("  JS ACTION: create new node %d (type %d, fw=%s)\n",
                                           act->node, type_idx, fw);
                            } else {
                                found = -1;
                            }
                        }
                    }
                    if (found >= 0) {
                        int i = found;
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
                        /* Schedule the rebooted node in the event queue
                         * so it gets ticked immediately */
                        sim_eq_schedule_if_earlier(&sim_eq, i, sim_ns);
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

        /* Serial socket: accept connections, read/write TCP, check child */
        if (ss_listen_fd >= 0 || (ss_child_pid > 0 && !ss_child_exited)) {
            if (ss_listen_fd >= 0) {
                ss_accept();
                ss_flush_tx();
                ss_read_tcp();
                ss_inject_serial();
            }
            ss_check_child();
            if (ss_child_exited)
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
            while (1) {
                sim_event_t next_node_ev = sim_eq_peek(&sim_eq);
                msp_rx_byte_event_t next_byte_ev = msp_rx_byte_queue_peek();
                int64_t next_ev_time = next_node_ev.time_ns < next_byte_ev.time_ns
                    ? next_node_ev.time_ns : next_byte_ev.time_ns;
                if (next_ev_time > sim_ns)
                    break;
                /* Match Cooja's simulation.getSimulationTime(): callbacks
                 * fired from the current event observe the exact event time,
                 * not the coarser outer stepping horizon. */
                current_sim_ns = next_ev_time;
                if (trace_event_spin_enabled()) {
                    if (next_ev_time == spin_trace_time_ns) {
                        spin_trace_count++;
                    } else {
                        spin_trace_time_ns = next_ev_time;
                        spin_trace_count = 1;
                    }
                    if (spin_trace_count == 1000000ULL) {
                        fprintf(stderr,
                                "  [SPIN] t=%.6f node_ev={node=%d time=%.6f seq=%llu} "
                                "byte_ev={node=%d sender=%d time=%.6f seq=%llu}\n",
                                (double)next_ev_time / 1e9,
                                next_node_ev.node_idx,
                                (double)next_node_ev.time_ns / 1e9,
                                (unsigned long long)next_node_ev.seq,
                                next_byte_ev.node_idx,
                                next_byte_ev.sender_idx,
                                (double)next_byte_ev.time_ns / 1e9,
                                (unsigned long long)next_byte_ev.seq);
                    }
                }
                /* Sync ALL native node channels before each event.
                 * TSCH hops channels during ticks; we need current state
                 * for frame delivery filtering. */
                for (int n = 0; n < node_count; n++) {
                    if (nodes[n].type == NODE_NATIVE && nodes[n].plat.native.simRadioChannel)
                        radio_medium_set_channel(&radio_medium, n,
                            *nodes[n].plat.native.simRadioChannel);
                }
                channels_dirty = false;

                /* Cooja has a single EventQueue. MSP byte deliveries are
                 * MspMoteTimeEvent instances in that same queue, so same-time
                 * ordering is FIFO by insertion sequence, not a blanket
                 * node-before-byte rule. */
                bool do_byte_event = msp_rx_byte_event_before_node(&next_byte_ev, &next_node_ev);
                if (do_byte_event) {
                    (void)drain_msp430_rx_byte_event(next_byte_ev.time_ns);
                    continue;
                }

                sim_event_t ev = sim_eq_pop(&sim_eq);
                int i = ev.node_idx;
                if (i < 0 || i >= node_count || !node_active(i))
                    continue;

                int64_t ev_time = ev.time_ns;

                /* Snapshot ALL nodes' state to detect frame delivery.
                 * Must be fresh for each event — ACK chains require
                 * detecting frames delivered during the current event. */
                int rx_before[MAX_NODES];
                int insize_before[MAX_NODES];
                for (int r = 0; r < node_count; r++) {
                    if (nodes[r].type == NODE_NATIVE) {
                        rx_before[r] = nodes[r].plat.native.rx_queue.count;
                        insize_before[r] = *nodes[r].plat.native.simInSize;
                    } else {
                        rx_before[r] = 0;
                        insize_before[r] = 0;
                    }
                }

                bool sender_had_tx = false;
                if (nodes[i].type == NODE_NATIVE) {
                    /* Single tick at event time */
                    tick_one_native(i, ev_time);
                    sender_had_tx = native_had_tx[i];

                    /* Schedule next wakeup (like ContikiClock.doActionsAfterTick) */
                    schedule_native_wakeup(&sim_eq, i);
                } else if (nodes[i].type == NODE_MSP430) {
                    /* MSP430 RF delivery is frame-assembled on the sender
                     * side and then delivered from the complete-frame path
                     * below. Consuming rf_pending here can split an in-flight
                     * frame and corrupt the receiver-side buffer state. */
                    ticking_node_idx = i;
                    int64_t returned_us = tick_one_msp430(i, ev_time);
                    ticking_node_idx = -1;
                    if (emu_rx_queue[i].count > 0)
                        emu_rx_queue_drain(i);
                    /* Match Cooja's MspMote.execute(t, 1): this slice
                     * schedules the mote's next normal wakeup itself. */
                    int64_t next_ns = ev_time + (returned_us + 1) * 1000LL;
                    sim_eq_schedule_if_earlier(&sim_eq, i, next_ns);
                } else {
                    /* Emulated ARM: step to event time, then self-schedule. */
                    ticking_node_idx = i;
                    int64_t delta_ns = ev_time - node_sim_time_ns(i);
                    if (delta_ns < 1000) delta_ns = 1000;  /* min 1µs */
                    int64_t target_cycle;
                    target_cycle = node_cycles(i) + arm_ns_to_cycles(delta_ns, node_freq(i));
                    if (target_cycle <= node_cycles(i))
                        target_cycle = node_cycles(i) + 1;
                    step_node_until(i, target_cycle);
                    ticking_node_idx = -1;
                    schedule_emulated_wakeup(&sim_eq, i);
                }

                /* RF delivery: if this node TX'd, schedule receivers
                 * and set signal strength on ALL in-range neighbors
                 * (like COOJA's signalReceptionStart + createConnections).
                 * This prevents simultaneous TX via CCA. */
                for (int r = 0; r < node_count; r++) {
                    if (r == i || !node_active(r)) continue;
                    if (nodes[r].type == NODE_NATIVE) {
                        bool got_frame =
                            nodes[r].plat.native.rx_queue.count > rx_before[r] ||
                            *nodes[r].plat.native.simInSize > insize_before[r];
                        if (got_frame) {
                            *nodes[r].plat.native.simReceiving = 1;
                            sim_eq_schedule_if_earlier(&sim_eq, r, ev_time);
                        }
                        /* Set signal strength on in-range neighbors so CCA
                         * detects the channel as busy. Only for nodes within
                         * interference range (matching COOJA's signalReceptionStart).
                         * Setting on ALL nodes causes CCA poisoning where out-of-range
                         * nodes permanently see channel busy, breaking SMRF/ESMRF. */
                        if (sender_had_tx && got_frame &&
                            nodes[r].plat.native.simSignalStrength)
                            *nodes[r].plat.native.simSignalStrength = -60;
                    }
                }

                /* Drain emulated RX queues. After delivery, step receivers
                 * with enough cycles for the FIFOP ISR + process_run() to
                 * read the RXFIFO. In Cooja, requestImmediateWakeup() gives
                 * the receiver a full tick at the current sim time. We give
                 * 1ms of CPU time (~4000 cycles at 4MHz) which covers ISR +
                 * one process_run() iteration. */
                for (int r = 0; r < node_count; r++) {
                    if (!node_active(r) || nodes[r].type == NODE_NATIVE) continue;
                    emu_rx_queue_drain(r);
                }

                /* Deliver pending bytes to native assemblers */
                for (int r = 0; r < node_count; r++) {
                    if (!node_active(r) || nodes[r].type != NODE_NATIVE) continue;
                    mixed_deliver_rf_bytes(r);
                }

                /* (debug stepping removed — was causing cascade on Node 1
                 * via unguarded step_node_until on Node 2) */
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

        /* Real-time pacing for serial socket mode (no UI server needed).
         * Throttle simulation to match wall-clock time at ui_speed_ratio. */
        if (ss_listen_fd >= 0 && !ui_server) {
            double sim_elapsed_ms = (double)(sim_ns - (end_ns - total_ns)) / 1e6;
            double wall_elapsed = get_time_ms() - t_start;
            double target_wall = sim_elapsed_ms / ui_speed_ratio;
            double wait_ms = target_wall - wall_elapsed;
            if (wait_ms > 0) {
                if (wait_ms > 10.0) wait_ms = 10.0;
                usleep((useconds_t)(wait_ms * 1000.0));
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
        memset(&msp_rx_byte_queue, 0, sizeof(msp_rx_byte_queue));
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
    extern void msp430_timer_dump_ccr_counts(void);
    msp430_timer_dump_ccr_counts();
    extern int msp430_gpio_get_isr_count(void);
    printf("  GPIO ISR count: %d\n", msp430_gpio_get_isr_count());
    printf("  FW cc2420_transmit=%d eb_process=%d queue_add=%d\n",
           cc2420_transmit_count, tsch_eb_process_count, tsch_queue_add_count);
    /* Dump SFD timestamp and Timer B state for TSCH debugging */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430) continue;
        uint8_t *mem = nodes[i].plat.msp.cpu.memory;
        uint16_t sfd_time = mem[0x24f8] | (mem[0x24f9] << 8);
        msp430_timer_t *tb = &nodes[i].plat.msp.timer_b;
        printf("  Node %d: sfd_start_time=%u TB_mode=%d TB_CCR1=%u TB_CCTL1=0x%04x\n",
               nodes[i].id, sfd_time, tb->mode, tb->ccr[1], tb->cctl[1]);
    }
    /* Dump TSCH state for Z1 nodes (firmware-specific addresses) */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430) continue;
        uint32_t tsch_coord = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_is_coordinator");
        uint32_t tsch_init = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_is_initialized");
        uint32_t tsch_start = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_is_started");
        if (tsch_coord && tsch_init && tsch_start) {
            uint8_t *mem = nodes[i].plat.msp.cpu.memory;
            uint32_t asn_addr = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_current_asn");
            uint32_t in_slot = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_in_slot_operation");
            uint32_t asn_lo = asn_addr ? (mem[asn_addr] | (mem[asn_addr+1]<<8) | (mem[asn_addr+2]<<16) | (mem[asn_addr+3]<<24)) : 0;
            uint32_t tsch_assoc = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_is_associated");
            uint32_t tsch_secured = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_is_pan_secured");
            uint32_t eb_period_addr = msp430_elf_find_symbol(nodes[i].firmware_path, "tsch_current_eb_period");
            uint32_t eb_period = eb_period_addr ? (mem[eb_period_addr] | (mem[eb_period_addr+1]<<8) |
                (mem[eb_period_addr+2]<<16) | (mem[eb_period_addr+3]<<24)) : 0;
            uint32_t leaf_only_addr = msp430_elf_find_symbol(nodes[i].firmware_path, "rpl_leaf_only");
            uint32_t used_addr = 0x2568;  /* curr_instance.used */
            uint32_t rank1 = 0x2572, rank2 = 0x25a6;
            uint32_t clock_count_addr = msp430_elf_find_symbol(nodes[i].firmware_path, "count");
            uint32_t clock_seconds_addr = msp430_elf_find_symbol(nodes[i].firmware_path, "seconds");
            uint32_t clock_count_val = clock_count_addr ?
                (mem[clock_count_addr] | (mem[clock_count_addr+1]<<8) |
                 (mem[clock_count_addr+2]<<16) | (mem[clock_count_addr+3]<<24)) : 0;
            uint32_t clock_seconds_val = clock_seconds_addr ?
                (mem[clock_seconds_addr] | (mem[clock_seconds_addr+1]<<8) |
                 (mem[clock_seconds_addr+2]<<16) | (mem[clock_seconds_addr+3]<<24)) : 0;
            printf("  Node %d TSCH: coord=%d init=%d started=%d assoc=%d secured=%d in_slot=%d asn=%u eb_period=%u "
                   "leaf_only=%d rpl_used=%d rank=%d/%d clock=%u/%us\n",
                nodes[i].id, mem[tsch_coord], mem[tsch_init], mem[tsch_start],
                tsch_assoc ? mem[tsch_assoc] : -1,
                tsch_secured ? mem[tsch_secured] : -1,
                in_slot ? mem[in_slot] : -1, asn_lo, eb_period,
                leaf_only_addr ? mem[leaf_only_addr] : -1,
                mem[used_addr],
                mem[rank1] | (mem[rank1+1]<<8),
                mem[rank2] | (mem[rank2+1]<<8),
                clock_count_val, clock_seconds_val);
        }
    }
    printf("  Total RF bytes: %d\n", rf_byte_count);
    printf("  Total UART bytes: %d\n", uart_byte_count);
    printf("  Emu RX frames: %d direct, %d queued, %d drained, %d dropped, %d collided\n",
           stat_emu_rx_direct, stat_emu_rx_queued, stat_emu_rx_drained,
           stat_emu_rx_dropped, stat_emu_rx_collided);
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430) continue;
        printf("  Node %d MSP byte queue: pushed=%llu popped=%llu\n",
               nodes[i].id,
               (unsigned long long)stat_msp_byte_push[i],
               (unsigned long long)stat_msp_byte_pop[i]);
    }
    printf("  MSP byte queue 2->1: pushed=%llu popped=%llu\n",
           (unsigned long long)stat_msp_byte_push_2_to_1,
           (unsigned long long)stat_msp_byte_pop_2_to_1);
    printf("  Native RX frames queued: %d\n", stat_rx_frames_queued);
    printf("  Native RX frames collided: %d\n", stat_rx_frames_collided);
    printf("  Native RX frames queue full: %d\n", stat_rx_frames_queue_full);
    extern int cc2538_rfcore_get_rxfifo_overflows(void);
    printf("  CC2538 RXFIFO overflows: %d\n", cc2538_rfcore_get_rxfifo_overflows());
    { int s,c,rej,ov,cg,cb,dr;
      cc2420_get_rx_stats(&s,&c,&rej,&ov,&cg,&cb,&dr);
      extern int cc2420_get_auto_ack_count(void);
      printf("  CC2420 RX: started=%d completed=%d rejected=%d overflow=%d crc_ok=%d crc_fail=%d dropped=%d auto_ack=%d\n",
             s,c,rej,ov,cg,cb,dr, cc2420_get_auto_ack_count());
    }
    /* Per-node CC2420 stats */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430) continue;
        cc2420_t *r = &nodes[i].plat.msp.cc2420;
        printf("  Node %d CC2420: state=%d on=%d spi=%d rx=%d/%d ack_rx=%d rej=%d ov=%d crc=%d/%d ack=%d "
               "incoming=%d replayed=%d dropped=%d "
               "strobes: rxon=%d txon=%d txoncca=%d rfoff=%d tx_cal=%d\n",
               nodes[i].id, r->state, r->on, r->stat_spi_count,
               r->stat_rx_started, r->stat_rx_completed, r->stat_rx_ack_completed,
               r->stat_rx_rejected, r->stat_rx_overflow,
               r->stat_crc_ok, r->stat_crc_fail, r->stat_auto_ack,
               r->stat_rx_buffered, r->stat_rx_replayed, r->stat_rx_dropped,
               r->stat_strobe_srxon, r->stat_strobe_stxon,
               r->stat_strobe_stxoncca, r->stat_strobe_srfoff,
               r->stat_tx_calibrate);
    }

    /* Dump MSP430 neighbor tables for debugging */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430) continue;
        msp430_cpu_t *cpu = &nodes[i].plat.msp.cpu;
        /* Find _ds6_neighbors_mem and neighbor_addr_mem_memb_mem */
        uint32_t nbr_addr = msp430_elf_find_symbol(nodes[i].firmware_path,
            "neighbor_addr_mem_memb_mem");
        uint32_t nbr_used = msp430_elf_find_symbol(nodes[i].firmware_path,
            "neighbor_addr_mem_memb_used");
        if (nbr_addr && nbr_used && nbr_addr < cpu->max_mem) {
            /* neighbor_addr_mem_memb: 16 entries × 10 bytes (2-byte list ptr + 8-byte lladdr) */
            int num_entries = 16;
            int entry_size = 10;
            printf("  Node %d neighbor table (addr_mem at 0x%04x):\n", nodes[i].id, nbr_addr);
            for (int n = 0; n < num_entries; n++) {
                if (nbr_used + n < cpu->max_mem && cpu->memory[nbr_used + n]) {
                    uint8_t *ll = cpu->memory + nbr_addr + n * entry_size + 2;
                    printf("    [%d] lladdr=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                           n, ll[0],ll[1],ll[2],ll[3],ll[4],ll[5],ll[6],ll[7]);
                }
            }
        }
    }

    /* Dump SR table from border-router (Node 1 = first MSP430) */
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].type != NODE_MSP430 || nodes[i].id != 1) continue;
        msp430_cpu_t *cpu = &nodes[i].plat.msp.cpu;
        uint32_t sr_mem = msp430_elf_find_symbol(nodes[i].firmware_path,
            "nodememb_memb_mem");
        uint32_t sr_used = msp430_elf_find_symbol(nodes[i].firmware_path,
            "nodememb_memb_used");
        if (sr_mem && sr_used && sr_mem < cpu->max_mem) {
            printf("  Node 1 SR table (nodememb at 0x%04x):\n", sr_mem);
            /* uip_sr_node_t: list ptr(2) + ipaddr suffix(2) + parent ptr(2) + ... = 18 bytes */
            for (int n = 0; n < 16; n++) {
                if (sr_used < cpu->max_mem && cpu->memory[sr_used + n]) {
                    uint8_t *entry = cpu->memory + sr_mem + n * 18;
                    /* Dump raw bytes to understand layout */
                    printf("    [%d] raw:", n);
                    for (int b = 0; b < 18; b++)
                        printf(" %02x", entry[b]);
                    printf("\n");
                }
            }
        }
    }

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

    /* Serial socket: use child exit status as test result */
    if (ss_child_exited) {
        printf("\n--- Serial Socket Test Results ---\n");
        if (ss_child_status == 0) {
            printf("  TEST PASSED (external command exited 0)\n");
        } else {
            printf("  TEST FAILED (external command exited %d)\n", ss_child_status);
            test_exit_code = ss_child_status;
        }
    }
    ss_cleanup();

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
