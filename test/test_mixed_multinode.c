/*
 * Mixed-platform multi-node simulation test
 *
 * Runs MSP430 (Tmote Sky / CC2420), ARM (CC2538DK / RF Core), native
 * Cooja motes, and JavaScript application motes in the same simulation.
 * All radios use wire-compatible 802.15.4 frames (4x0x00 preamble + 0x7A
 * SFD + length + payload), enabling cross-platform Contiki-NG networking
 * (RPL, nullnet, etc.).
 *
 * Node type is auto-detected from firmware file extension:
 *   .sky      -> MSP430 (Tmote Sky)
 *   .cc2538dk -> ARM (CC2538DK)
 *   .cooja    -> Native (Cooja mote via dlopen)
 *   .js       -> JavaScript application mote (QuickJS)
 */
#include "msp430_platform.h"
#include "msp430_elf.h"
#include "cc2420.h"
#include "arm_platform.h"
#include "cc2538_soc.h"
#include "nrf52840_soc.h"
#include "nrf54l15_soc.h"
#include "arm_systick.h"
#include "arm_elf.h"
#include "native_node.h"
#include "js_node.h"
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
#include "sim_runtime.h"
#include "sim_board.h"
#include "sim_serial_bridge.h"
#include "sim_external_command.h"
#include "sim_radio_bus.h"
#include "gdb_stub.h"
#include "arm_gdb.h"
#include "pcap_writer.h"
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

/* node_type_t / mixed_node_t / rf_listener_ctx_t / sim_mote_env_t live
 * in src/motes/mote_impl.h (Phase 4, M18) — shared with the per-kind
 * mote modules. */
#include "mote_impl.h"

/* rf_buffer_t / tx_frame_asm_t / tx_frame_capture_t live in sim_radio_bus.h (M9.1) */

/* tx frame assembler (sim_radio_bus_asm_feed/reset) lives in
 * src/sim/sim_radio_bus.c (M9.4) */

/* emu_rx_frame_t / emu_rx_queue_t live in sim_radio_bus.h (M9.1) */

static mixed_node_t nodes[MAX_NODES];
/* Kernel-facing mote objects (Phase 2, M11) — one per slot, bound to
 * nodes[] by register_node_mote().  Adapter tables live further down. */
static sim_mote_t mote_store[MAX_NODES];
/* MOTE_IMPL lives in mote_impl.h (M19); MOTE_IDX is runner-only —
 * module code uses MOTE_IMPL(m)->slot instead. */
#define MOTE_IDX(m)  ((int)(MOTE_IMPL(m) - nodes))
/* The "currently-executing node" guard moved onto the bus
 * (radio_bus.executing_node, M26); set via sim_radio_bus_set_executing
 * around CPU steps, read by the bus's synchronous RX delivery. */

/* PC trace callback for debugging */
static uint32_t srh_trace_fn_addr;
static int cc2420_transmit_count;
static int tsch_eb_process_count;
static int tsch_queue_add_count;
void srh_trace_cb(void *data, uint32_t pc, uint32_t *reg, uint8_t *mem) {
    (void)data; (void)reg; (void)mem;
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
/* M9.2: RF routing state lives in the kernel-owned radio bus.  The
 * aliases keep call sites unchanged until the dispatch functions
 * migrate into the bus (M9.3/9.4). */
static sim_radio_bus_t radio_bus;
#define rf_pending    (radio_bus.rf_pending)
#define tx_asm        (radio_bus.tx_asm)
#define tx_cap        (radio_bus.tx_cap)
#define emu_rx_queue  (radio_bus.emu_rx_queue)
#define emu_rx_end_ns (radio_bus.emu_rx_end_ns)
static int rf_byte_count = 0;
static int uart_byte_count = 0;

/* Simulation kernel container — Phase 1 of the refactor (see
 * docs/design/refactor-plan.md §3.15).  This bundles the unified event queue,
 * the radio medium, and the current simulation time that were previously
 * three separate file-scope globals.
 *
 * Milestone 1: introduce the container, alias the legacy globals.
 * Milestone 2 (this commit): readers go through sim_runtime_now_ns(&sim_rt);
 *   the `current_sim_ns` alias is gone.  Writers use `sim_rt.now_ns = X`
 *   directly until milestone 3 introduces scheduling wrappers.
 * Subsequent milestones migrate sim_eq_* / radio_medium accesses onto
 * sim_schedule_* / radio bus APIs and drop the remaining aliases. */
static sim_runtime_t sim_rt;
#define sim_eq          (sim_rt.event_queue)
#define radio_medium    (sim_rt.radio_medium)

/* ============================================================
 * CSIM_TRACE_RADIO — channel + TX + filter trace for debugging
 * ============================================================
 * Enabled by `CSIM_TRACE_RADIO=1` env var. When on, every channel
 * change, frame TX (start + complete), filter decision (deliver
 * or drop with reason), and per-node CPU step is logged with a
 * sim_ns timestamp. Output is one line per event, parseable.
 *
 * Format:
 *   [t=12.345678s] event_type field=val field=val ...
 *
 * Disabled overhead = one TLS bool check.
 */
static int csim_radio_trace = -1;
int csim_radio_trace_enabled(void) {
    if (csim_radio_trace < 0) {
        const char *e = getenv("CSIM_TRACE_RADIO");
        csim_radio_trace = (e && e[0] && strcmp(e, "0") != 0) ? 1 : 0;
    }
    return csim_radio_trace;
}
#define RTRACE(fmt, ...) do { \
    if (csim_radio_trace_enabled()) \
        fprintf(stderr, "[t=%.6fs] " fmt "\n", \
                (double)sim_runtime_now_ns(&sim_rt) / 1e9, ##__VA_ARGS__); \
} while (0)

void csim_radio_trace_filter(int s, int sr, int rcv, int rr,
                              int s_ch, int r_ch, int delivered) {
    if (delivered)
        RTRACE("filter sender=%d/%d receiver=%d/%d ch=%d/%d -> DELIVER",
               s, sr, rcv, rr, s_ch, r_ch);
    else
        RTRACE("filter sender=%d/%d receiver=%d/%d ch=%d/%d -> DROP "
               "(channel_mismatch)", s, sr, rcv, rr, s_ch, r_ch);
}

/* Optional pcap writer — opened by --pcap PATH, captures every TX frame
 * once at the on-air timestamp. */
static pcap_writer_t pcap_writer = { 0 };

/* Statistics counters */
static int stat_rf_frames = 0;       /* total TX frames (all node types) */
/* The five emu-RX delivery counters moved to radio_bus.stats (M26):
 * rx_direct / rx_queued / rx_drained / rx_dropped / rx_collided. */
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
        sim_runtime_now_ns(&sim_rt) < TRACE_TSCH_ACK_START_NS ||
        sim_runtime_now_ns(&sim_rt) > TRACE_TSCH_ACK_END_NS)
        return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [TRACE] %7.3f ", (double)sim_runtime_now_ns(&sim_rt) / 1e9);
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

/* rf_outgoing_t lives in sim_radio_bus.h (M9.1) */

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

#define rf_outgoing   (radio_bus.rf_outgoing)
static frame_outgoing_t frame_outgoing[MAX_NODES];
static node_thread_state_t thread_state[MAX_NODES];

/* Per-node last TX timestamp for UI communication arrows */
static int64_t node_last_tx_ns[MAX_NODES];

/* Per-node "TX busy until" timestamp moved to radio_bus.tx_busy_until_ns
 * (M27).  Read by the CC1200 channel-busy query so receivers' CCA
 * reflects in-progress transmissions on the medium. */

/* Per-node start time: node is inactive (no step, no RF) until sim_ns >= start_ns */
static int64_t node_start_ns[MAX_NODES];  /* 0 = start immediately */

static inline int node_active(int idx) {
    return sim_runtime_now_ns(&sim_rt) >= node_start_ns[idx];
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

/* --- Serial socket (TCP bridge for border-router tests) ---
 *
 * Milestone 8.1: socket + ring-buffer plumbing lives in the
 * sim_serial_bridge kernel service.  The runner keeps the bridged-node
 * index (the inject callback is mote-type-specific until the Phase 2
 * mote vtable lands) and the external-command management (milestone
 * 8.2's external_command_service). */
static sim_serial_bridge_t serial_bridge;
static int ss_node_idx = -1;     /* index of the bridged node */

/* External test-driver process + COOJA.testlog tee — milestone 8.2
 * service (sim_external_command).  The testlog tee rides the
 * SIM_OBS_MOTE_LOG_LINE observer stream. */
static sim_external_command_t external_cmd;

/* --- Timeline and packet analyzer state --- */
static timeline_t timeline;
static sim_node_state_t node_states[MAX_NODES];
static sim_node_state_t prev_node_states[MAX_NODES]; /* previous delta state for change detection */
static int64_t prev_last_tx_ns[MAX_NODES];           /* previous TX timestamps for change detection */
/* "suppress chip state callbacks during synchronous delivery" moved to
 * radio_bus.in_delivery (M27). */

/* Timeline observer — Phase 1 milestone 7.  Subscribed to the runtime via
 * sim_runtime_subscribe() at startup; translates each kernel-emitted
 * sim_observer_event_t into the equivalent tl_*_event() call.
 *
 * This commit migrates the frame-level events (TX/RX start/end,
 * interference, packet frame, LED change).  The two radio-state-tracking
 * call sites in update_radio_state() still call tl_radio_event() directly
 * because they emit chip-state transitions (ON/OFF/INTF/TX/RX) that
 * don't have a clean 1:1 mapping to the plan's START/END observer kinds. */
static void timeline_observer_cb(void *user, const sim_observer_event_t *ev) {
    timeline_t *tl = (timeline_t *)user;
    if (!tl || ev->mote_index < 0 || ev->mote_index >= MAX_NODES) return;
    int node_id = nodes[ev->mote_index].id;
    switch (ev->kind) {
    case SIM_OBS_RADIO_TX_START:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_TX);   break;
    case SIM_OBS_RADIO_TX_END:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_ON);   break;
    case SIM_OBS_RADIO_RX_START:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_RX);   break;
    case SIM_OBS_RADIO_RX_END:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_ON);   break;
    case SIM_OBS_RADIO_INTERFERENCE:
        tl_radio_event(tl, node_id, ev->time_ns, TL_RADIO_INTF); break;
    case SIM_OBS_LED_CHANGED:
        tl_led_event(tl, node_id, ev->time_ns,
                     ev->u.led.led_index, ev->u.led.on ? 1 : 0);  break;
    case SIM_OBS_PACKET_FRAME:
        tl_frame_event(tl, node_id, ev->time_ns,
                       ev->u.frame.is_tx ? 1 : 0,
                       ev->u.frame.summary);                       break;
    default:
        break;
    }
}

/* Helpers to keep the call-site changes one-liners and visually close to the
 * legacy tl_*_event() forms they replace. */
static inline void emit_radio_obs(int mote_index, int64_t time_ns,
                                   sim_observer_kind_t kind) {
    sim_observer_event_t ev = { .kind = kind, .time_ns = time_ns,
                                .mote_index = mote_index, .radio_idx = 0 };
    sim_runtime_emit(&sim_rt, &ev);
}
static inline void emit_frame_obs(int mote_index, int64_t time_ns,
                                   bool is_tx, const char *summary) {
    sim_observer_event_t ev = { .kind = SIM_OBS_PACKET_FRAME, .time_ns = time_ns,
                                .mote_index = mote_index, .radio_idx = 0 };
    ev.u.frame.is_tx = is_tx;
    ev.u.frame.summary = summary;
    sim_runtime_emit(&sim_rt, &ev);
}
static inline void emit_led_obs(int mote_index, int64_t time_ns,
                                 int led_index, bool on) {
    sim_observer_event_t ev = { .kind = SIM_OBS_LED_CHANGED, .time_ns = time_ns,
                                .mote_index = mote_index, .radio_idx = -1 };
    ev.u.led.led_index = led_index;
    ev.u.led.on = on;
    sim_runtime_emit(&sim_rt, &ev);
}

/* RF state change callback — fires during CPU step for real-time timeline tracking.
 * TX and RX events are handled by explicit timeline events in the TX/delivery
 * handlers (with proper frame duration). This callback handles radio ON/OFF
 * transitions (ISRXON, ISRFOFF, tx_done return-to-RX). */
static void mixed_rf_state_handler(void *user_data, int old_state, int new_state) {
    if (!ui_server || radio_bus.in_delivery) return;
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
    /* M16: poll the mote's ui_radio_state op.  NULL op = this mote kind
     * is not state-polled (CC2538 pushes through an async callback;
     * natives/JS have no chip model to sample). */
    const sim_mote_t *m = &mote_store[idx];
    if (!m->ops->ui_radio_state)
        return;
    sim_radio_state_t new_state =
        (sim_radio_state_t)m->ops->ui_radio_state(m);
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
            tl_radio_event(&timeline, nodes[idx].id, sim_runtime_now_ns(&sim_rt), etype);
        }
    }
}

/* Track LED state changes (M16: ui_leds op; platform mapping lives in
 * the adapters — Tmote Sky P5.4-6, CC2538DK PC0-2).  NULL op = no
 * observable LEDs for this mote kind. */
static void update_led_state(int idx) {
    const sim_mote_t *m = &mote_store[idx];
    if (!m->ops->ui_leds)
        return;
    uint8_t leds[SIM_MAX_LEDS] = {0, 0, 0};
    m->ops->ui_leds(m, leds);
    for (int l = 0; l < SIM_MAX_LEDS; l++) {
        if (leds[l] != node_states[idx].led[l]) {
            node_states[idx].led[l] = leds[l];
            if (ui_server)
                emit_led_obs(idx, sim_runtime_now_ns(&sim_rt), l, leds[l] ? true : false);
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

/* Feed a console line to whichever test engine is active.  Subscribed
 * to the kernel observer stream (milestone 8.3) — both line-assembly
 * sites emit SIM_OBS_MOTE_LOG_LINE and this adapter fans the line into
 * the JS engine and the JSON step/validator checker. */
static void test_engine_observer(void *user, const sim_observer_event_t *ev) {
    (void)user;
    if (ev->kind != SIM_OBS_MOTE_LOG_LINE) return;
    if (active_js_engine) {
        js_test_feed_line(active_js_engine, ev->u.log_line.line,
                          ev->u.log_line.node_id, ev->time_ns / 1000);
    }
    if (active_test) {
        sim_test_check_line(ev->u.log_line.node_id, ev->u.log_line.line,
                            ev->time_ns);
    }
}

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

/* ============================================================
 * Mote vtable adapters — Phase 2 milestone 11 (§3.16).
 *
 * One sim_mote_ops_t per node kind; mote_store[idx] is the kernel-facing
 * object registered into sim_rt from init_node().  The node_* accessor
 * functions below dispatch through the table instead of switching on
 * node_type_t.  Adapter bodies are character-identical moves of the old
 * switch arms; they stay in this file until Phase 4 (boot policy move).
 * ============================================================ */

/* M12 execution-op adapters — bodies live next to the tick/step helpers
 * they wrap (after native_has_pending_work); tables below only need the
 * prototypes. */
static int64_t msp_mote_execute(sim_mote_t *m, int64_t now_ns);
static int64_t arm_mote_execute(sim_mote_t *m, int64_t now_ns);
static void msp_mote_step_until(sim_mote_t *m, int64_t target);
static void arm_mote_step_until(sim_mote_t *m, int64_t target);
static void msp_mote_advance_to_time(sim_mote_t *m, int64_t sim_ns);
static void arm_mote_advance_to_time(sim_mote_t *m, int64_t sim_ns);
static int64_t msp_mote_sched_hint_ns(const sim_mote_t *m, int64_t base_ns);
static int64_t arm_mote_sched_hint_ns(const sim_mote_t *m, int64_t base_ns);
static int64_t msp_mote_sync_to_time(sim_mote_t *m, int64_t sim_ns);
static int64_t arm_mote_sync_to_time(sim_mote_t *m, int64_t sim_ns);
static void msp_mote_rx_byte_sync(sim_mote_t *m, int64_t byte_time_ns);
static void arm_mote_rx_byte_sync(sim_mote_t *m, int64_t byte_time_ns);
static void msp_mote_rx_pre_sync(sim_mote_t *m, int64_t time_ns);
static int msp_mote_serial_input(sim_mote_t *m, const uint8_t *buf, int len);
static int arm_mote_serial_input(sim_mote_t *m, const uint8_t *buf, int len);
static void msp_mote_destroy(sim_mote_t *m);
static void arm_mote_destroy(sim_mote_t *m);
static void msp_mote_reset_time(sim_mote_t *m, int64_t now_ns);
static void arm_mote_reset_time(sim_mote_t *m, int64_t now_ns);
static int msp_mote_ui_radio_state(const sim_mote_t *m);
static void msp_mote_ui_leds(const sim_mote_t *m, uint8_t leds[3]);
static void arm_mote_ui_leds(const sim_mote_t *m, uint8_t leds[3]);
static void *msp_mote_get_interface(sim_mote_t *m, int iface);
static void *arm_mote_get_interface(sim_mote_t *m, int iface);
/* (JS + native adapters live in src/motes/{js_app,native_cooja}_mote.c
 * — M19/M20) */

/* Convenience: the mote's CC2420, or NULL for non-Sky motes (debug
 * traces + per-byte stats branch on this instead of node type). */
static inline cc2420_t *mote_cc2420(int idx) {
    sim_mote_t *m = &mote_store[idx];
    return (cc2420_t *)m->ops->get_interface(m, SIM_MOTE_IFACE_CC2420);
}

/* MSP430 emulated motes */
static int64_t msp_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.msp.cpu.sim_time_ns;
}
static int64_t msp_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.msp.cpu.cycles;
}
static uint32_t msp_mote_freq_hz(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.msp.cpu.cpu_freq_hz;
}
static int64_t msp_mote_instructions(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.msp.cpu.instructions;
}
static const sim_mote_ops_t msp_mote_ops = {
    .kind            = "MSP430",
    .sim_time_ns     = msp_mote_sim_time_ns,
    .cycles          = msp_mote_cycles,
    .freq_hz         = msp_mote_freq_hz,
    .instructions    = msp_mote_instructions,
    .execute         = msp_mote_execute,
    .step_until      = msp_mote_step_until,
    .advance_to_time = msp_mote_advance_to_time,
    .sched_hint_ns   = msp_mote_sched_hint_ns,
    .sync_to_time    = msp_mote_sync_to_time,
    .serial_input    = msp_mote_serial_input,
    .destroy         = msp_mote_destroy,
    .reset_time      = msp_mote_reset_time,
    .ui_radio_state  = msp_mote_ui_radio_state,
    .ui_leds         = msp_mote_ui_leds,
    .get_interface   = msp_mote_get_interface,
    .receive_frame   = NULL, /* per-byte / staged delivery */
    .rx_byte_sync    = msp_mote_rx_byte_sync,
    .rx_pre_sync     = msp_mote_rx_pre_sync,
};

/* ARM emulated motes */
static int64_t arm_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.sim_time_ns;
}
static int64_t arm_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.cycles;
}
static uint32_t arm_mote_freq_hz(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.cpu_freq_hz;
}
static int64_t arm_mote_instructions(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.instructions;
}
static const sim_mote_ops_t arm_mote_ops = {
    .kind            = "ARM",
    .sim_time_ns     = arm_mote_sim_time_ns,
    .cycles          = arm_mote_cycles,
    .freq_hz         = arm_mote_freq_hz,
    .instructions    = arm_mote_instructions,
    .execute         = arm_mote_execute,
    .step_until      = arm_mote_step_until,
    .advance_to_time = arm_mote_advance_to_time,
    .sched_hint_ns   = arm_mote_sched_hint_ns,
    .sync_to_time    = arm_mote_sync_to_time,
    .serial_input    = arm_mote_serial_input,
    .destroy         = arm_mote_destroy,
    .reset_time      = arm_mote_reset_time,
    .ui_radio_state  = NULL, /* CC2538 pushes state via async callback */
    .ui_leds         = arm_mote_ui_leds,
    .get_interface   = arm_mote_get_interface,
    .receive_frame   = NULL, /* per-byte / staged delivery */
    .rx_byte_sync    = arm_mote_rx_byte_sync,
    .rx_pre_sync     = NULL, /* MSP430-only pre-sync tick */
};

/* Native + JS mote ops live in src/motes/{native_cooja,js_app}_mote.c
 * (M19/M20). */

/* Bind mote_store[idx] to nodes[idx] and register it with the kernel.
 * Called from init_node() right after the node's type is set, so the
 * accessors below work during platform init and for rebooted /
 * dynamically added motes.  Ops come from the kind-registry row
 * (M23) — the MSP430/ARM tables are injected at startup. */
static void register_node_mote(int idx, const sim_mote_ops_t *ops) {
    mote_store[idx].id = nodes[idx].id;
    mote_store[idx].ops = ops;
    mote_store[idx].impl = &nodes[idx];
    sim_runtime_register_mote(&sim_rt, idx, &mote_store[idx]);
}

/* --- Type-dispatched accessors (now vtable dispatches) --- */

static int64_t node_sim_time_ns(int idx) {
    const sim_mote_t *m = &mote_store[idx];
    return m->ops->sim_time_ns(m);
}

static int64_t node_cycles(int idx) {
    const sim_mote_t *m = &mote_store[idx];
    return m->ops->cycles(m);
}

static uint32_t node_freq(int idx) {
    const sim_mote_t *m = &mote_store[idx];
    return m->ops->freq_hz(m);
}

static int64_t node_instructions(int idx) {
    const sim_mote_t *m = &mote_store[idx];
    return m->ops->instructions(m);
}

static const char *node_type_str(int idx) {
    return mote_store[idx].ops->kind;
}

/* --- RF TX/RX bridging --- */

/* Forward declarations */
static void mixed_deliver_rf_bytes(int idx);
static void step_node_until(int idx, int64_t target);
static void schedule_emulated_wakeup(sim_runtime_t *sim, int idx);

/* Check available RXFIFO space for an emulated node. Firefly nodes have
 * two radios; we report the more constrained side so back-pressure
 * still pumps frames in/out correctly. */
/* --- Mote radio endpoint ops (M9.3) --------------------------------
 * Per-platform implementations registered on the radio bus at node
 * init.  All RF byte delivery, FIFO back-pressure, and mid-frame
 * checks dispatch through these — the bus no longer switches on node
 * type. */

/* All four kinds' radio endpoint ops live in their src/motes modules
 * (M19–M22); init_node registers them through the kind-registry row
 * (M23). */

static int emulated_rxfifo_available(int idx) {
    if (radio_bus.ops[idx])
        return radio_bus.ops[idx]->rxfifo_available(radio_bus.mote[idx]);
    return 0;
}

/* Deliver one on-air byte to node idx's radio(s) via its registered ops. */
static inline void radio_receive_byte(int idx, uint8_t byte, int8_t rssi) {
    if (radio_bus.ops[idx])
        radio_bus.ops[idx]->receive_byte(radio_bus.mote[idx], byte, rssi);
}

/* True while node idx's radio is mid-frame (delivery now would corrupt it). */
static inline bool radio_rx_busy(int idx) {
    return radio_bus.ops[idx] && radio_bus.ops[idx]->rx_busy(radio_bus.mote[idx]);
}

/* frame_fifo_bytes lives in sim_radio_bus.c (M9.1) */

/* Emulated RX queue push/deliver/drain bodies moved to
 * src/sim/sim_radio_bus.c (M26); these stay as thin forwarders so the
 * call sites are unchanged.  emu_rx_queue[] / emu_rx_end_ns[] / the
 * RX stat counters now live on the bus. */
static void emu_rx_queue_push(int idx, const uint8_t *data, int len,
                               int8_t rssi, int64_t arrival_ns, int64_t end_ns,
                               bool subghz) {
    sim_radio_bus_queue_frame(&radio_bus, idx, data, len, rssi,
                              arrival_ns, end_ns, subghz);
}

/* Process a SIM_EV_RX_BYTE event for an emulated receiver (M13: one
 * type-blind path for MSP430 and ARM).  Mirrors Cooja's
 * MspMoteTimeEvent: execute(t, 0) to advance the receiver's clock, then
 * receivedByte(), then requestImmediateWakeup. */
static void deliver_rx_byte(const sim_event_t *ev) {
    int idx = ev->node_idx;
    if (idx < 0 || idx >= num_nodes || !node_active(idx))
        return;
    sim_mote_t *m = &mote_store[idx];
    if (!m->ops->sync_to_time)
        return;  /* RX_BYTE events only target emulated motes */

    /* CC2420-carrying motes only: byte-flow stats + TSCH-ACK trace
     * (M16: the chip peek goes through get_interface — NULL means this
     * mote has no CC2420 and the debug block is skipped). */
    cc2420_t *cc = mote_cc2420(idx);
    if (cc) {
        stat_msp_byte_pop[idx]++;
        if (ev->sender_idx >= 0 && ev->sender_idx < num_nodes &&
            nodes[ev->sender_idx].id == 2 && nodes[idx].id == 1) {
            stat_msp_byte_pop_2_to_1++;
        }
    }

    int64_t returned_us = m->ops->sync_to_time(m, ev->time_ns);

    cc2420_radio_state_t old_state = CC2420_VREG_OFF;
    if (cc) {
        old_state = cc->state;
        if (trace_tsch_ack_enabled() &&
            nodes[idx].id > 0 && nodes[idx].id <= 2) {
            trace_tsch_ack_log("byte event node=%d byte=%02x t=%.6f state_before=%s",
                               nodes[idx].id,
                               ev->byte, (double)ev->time_ns / 1e9,
                               cc2420_state_str(old_state));
        }
    }

    radio_receive_byte(idx, ev->byte, ev->rssi);

    if (cc) {
        cc2420_radio_state_t new_state = cc->state;
        if (trace_tsch_ack_enabled() &&
            nodes[idx].id > 0 && nodes[idx].id <= 2 &&
            new_state != old_state) {
            trace_tsch_ack_log("byte event node=%d state %s -> %s",
                               nodes[idx].id,
                               cc2420_state_str(old_state),
                               cc2420_state_str(new_state));
        }
    }

    if (num_threads == 0) {
        /* Match Cooja's MspMoteTimeEvent + requestImmediateWakeup:
         * execute(t, 0) first, then receivedByte(), then a same-time mote
         * wakeup request. Also retain the next wakeup returned by execute. */
        int64_t next_ns = ev->time_ns + returned_us * 1000LL;
        sim_schedule_mote_wakeup_if_earlier(&sim_rt, idx, next_ns);
        sim_schedule_mote_wakeup_if_earlier(&sim_rt, idx, ev->time_ns);
    }
}

/* Process a SIM_EV_RADIO_TIMER event (M9.5): the radio bus's per-receiver
 * RX-stall deadline.  The bus decides stale-vs-expired (fresher bytes
 * extend the deadline and re-arm); on true expiry, sync the mote's CPU
 * to the deadline time and fire ops->rx_stall so the chip can abandon a
 * mid-frame parse the way real HW's signal-loss detector would. */
static void deliver_radio_timer(const sim_event_t *ev) {
    int idx = ev->node_idx;
    if (idx < 0 || idx >= num_nodes || !node_active(idx))
        return;
    if (!radio_bus.ops[idx] || !radio_bus.ops[idx]->rx_stall)
        return;
    if (!sim_radio_bus_rx_stall_expired(&radio_bus, &sim_rt, idx, ev->time_ns))
        return;  /* deadline extended; bus re-armed */
    sim_mote_t *m = &mote_store[idx];
    if (m->ops->sync_to_time)
        m->ops->sync_to_time(m, ev->time_ns);
    radio_bus.ops[idx]->rx_stall(radio_bus.mote[idx]);
    if (num_threads == 0)
        sim_schedule_mote_wakeup_if_earlier(&sim_rt, idx, ev->time_ns);
}

/* byte_period_ns lives in sim_radio_bus.h (M9.1) */

/* M13: sync_to_time op adapters (Cooja MspMoteTimeEvent.execute(t, 0)).
 * Emulated motes only; the RX-byte/radio-timer delivery paths dispatch
 * through ops->sync_to_time and treat NULL as "not an emulated mote". */
static int64_t msp_mote_sync_to_time(sim_mote_t *m, int64_t sim_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    /* Cap to global sim time — no node should advance past it (Cooja invariant) */
    if (sim_ns > sim_runtime_now_ns(&sim_rt))
        sim_ns = sim_runtime_now_ns(&sim_rt);
    msp430_cpu_t *cpu = &node->plat.msp.cpu;
    int64_t t_us = sim_ns / 1000LL;
    int64_t jump_us = 0;

    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }

    /* Apply clock deviation (same as tick_one_msp430) */
    double deviation = node->clock_deviation;
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

/* ARM equivalent of msp_mote_sync_to_time — Cooja MspMoteTimeEvent.execute
 * pattern: advance CPU to event time before feeding chip-level byte. */
static int64_t arm_mote_sync_to_time(sim_mote_t *m, int64_t sim_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    if (sim_ns > sim_runtime_now_ns(&sim_rt))
        sim_ns = sim_runtime_now_ns(&sim_rt);
    arm_cpu_t *cpu = &node->plat.arm.cpu;
    int64_t t_us = sim_ns / 1000LL;
    int64_t jump_us = 0;
    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }
    double deviation = node->clock_deviation;
    if (deviation != 1.0 && jump_us > 0) {
        double exact = (double)jump_us * deviation;
        jump_us = (int64_t)exact;
        cpu->step_cycle_remainder += exact - (double)jump_us;
        if (cpu->step_cycle_remainder > 1.0) {
            jump_us++;
            cpu->step_cycle_remainder -= 1.0;
        }
    }
    int64_t returned_us = arm_step_micros(cpu, jump_us, 0);
    cpu->sim_time_ns = sim_ns;
    cpu->last_execute_us = t_us;
    if (deviation != 1.0 && returned_us > 0)
        returned_us = (int64_t)((double)returned_us / deviation);
    return returned_us;
}

/* M25 (Phase 5 de-typing): per-byte RX clock sync used by the
 * synchronous frame-delivery path (emu_deliver_bytes).  Bodies are the
 * verbatim ex-inline per-byte sync from the old nodes[].type branches:
 * MSP430 reuses its clamped, deviation-aware sync_to_time; ARM keeps
 * its distinct unclamped pin-step-pin body (the two differ on
 * purpose — do not unify). */
static void msp_mote_rx_byte_sync(sim_mote_t *m, int64_t byte_time_ns) {
    msp_mote_sync_to_time(m, byte_time_ns);
}
static void arm_mote_rx_byte_sync(sim_mote_t *m, int64_t byte_time_ns) {
    arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    int64_t t_us = byte_time_ns / 1000LL;
    int64_t jump_us = 0;
    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }
    cpu->sim_time_ns = byte_time_ns;
    arm_step_micros(cpu, jump_us, 0);
    cpu->sim_time_ns = byte_time_ns;
    cpu->last_execute_us = t_us;
}

/* M25: full execute-slice pre-sync before a synchronous frame delivery
 * (ex the frame_complete pre-sync tick; MSP430 only). */
static void msp_mote_rx_pre_sync(sim_mote_t *m, int64_t time_ns) {
    msp430_elf_mote_tick(MOTE_IMPL(m), time_ns);
}

/* Deliver buffered bytes to an emulated node's radio — body moved to
 * sim_radio_bus_deliver_bytes (M26); thin forwarder keeps call sites
 * unchanged.  The RX timeline emit moved to the bus's on_rx hook
 * (bus_host_on_rx). */
static void emu_deliver_bytes(int idx, const uint8_t *data, int len,
                               int8_t rssi, int64_t air_time_ns, bool subghz) {
    sim_radio_bus_deliver_bytes(&radio_bus, &sim_rt, idx, data, len, rssi,
                                air_time_ns, subghz);
}

/* Per-chip TX listener contexts live in node->rf_ctx[2] (M18 — the
 * rf_listener_ctx_t definition and rationale are in mote_impl.h).
 * nodes[] is static storage, so the captured addresses stay stable
 * across the chip's lifetime exactly as the old rf_ctx_slot0/1[]
 * arrays did. */

/* Forward decl — full body lives below. */
static void mixed_rf_tx_handler_radio(int sender_idx, int sender_radio, uint8_t byte);

/* Chip-side TX listener trampoline.  user_data is the per-(node, slot)
 * rf_listener_ctx_t the chip was registered with. */
static void mixed_rf_tx_chip_cb(void *user_data, uint8_t byte) {
    rf_listener_ctx_t *ctx = (rf_listener_ctx_t *)user_data;
    mixed_rf_tx_handler_radio(ctx->node_idx, ctx->radio_idx, byte);
}

/*
 * Native (Cooja-mote) channel sync. Emulated nodes push their channel
 * into the medium synchronously through chip-driver callbacks; native
 * motes have no such callback, so we read simRadioChannel and push it
 * into the medium at the moment it matters: just before each byte- or
 * frame-delivery decision involving a native node. This catches TSCH-
 * style mid-tick channel hops at the byte boundary where the medium
 * actually consults the value.
 *
 * Cheap inline guard — only does work when the channel actually changed.
 * Safe to call from the byte-delivery hot path.
 */
static inline void sync_native_node_channel(int idx) {
    if (idx < 0 || idx >= num_nodes) return;
    if (nodes[idx].type != NODE_NATIVE) return;
    int *p = nodes[idx].plat.native.simRadioChannel;
    if (!p) return;
    int ch = *p;
    if (radio_medium.nodes[idx].radios[0].channel == ch) return;
    radio_medium_set_channel(&radio_medium, idx, ch);
}

/*
 * Per-radio channel-change adapter — installed as the sim_host_t
 * radio_set_channel callback for off-SoC chip drivers (CC2420 on
 * MSP430 platforms, CC1200 on Firefly), and as the
 * cc2538_rfcore_set_channel_callback observer for the on-SoC RF Core.
 *
 * The chip driver passes its (radio_idx, channel) to the harness; we
 * convert to a global (node_idx, radio_idx) and push into the medium.
 * Sub-GHz channels are remapped onto the legacy sub-GHz channel base so
 * the CCA-busy heuristic above (which still keys on the legacy alias)
 * keeps working until we migrate it.
 */
static void mixed_node_radio_set_channel(mixed_node_t *node, int radio_idx,
                                          int channel) {
    int idx = (int)(node - nodes);
    if (idx < 0 || idx >= num_nodes) return;
    /* Keep slot 0 reserved for the 2.4 GHz primary radio; CC1200 lives
     * on slot 1 by convention. The chip drivers already call us with
     * the correct slot, so this is just a defensive sanity guard. */
    if (radio_idx < 0 || radio_idx >= RADIO_MEDIUM_MAX_RADIOS_PER_NODE) return;
    int prev = radio_medium.nodes[idx].radios[radio_idx].channel;
    if (prev != channel)
        RTRACE("ch_set node=%d radio=%d ch=%d (was %d)",
               idx, radio_idx, channel, prev);
    /* Auto-register the radio's spectrum on first channel push if not
     * already registered. Without this, pick_receiver_radio sees
     * SPECTRUM_NONE and falls back to "target slot 0" — which routes
     * CC1200 (slot 1) bytes to the receiver's parked cc2538_rfcore
     * (slot 0) instead of to its CC1200. Convention:
     *   slot 0 → 2.4 GHz IEEE 802.15.4 (CC2420 / cc2538_rfcore)
     *   slot 1 → sub-GHz 802.15.4g (CC1200 EU 868 MHz default)
     * This is harness-side metadata; chip drivers stay portable
     * (they just push a (radio_idx, channel) pair). */
    if (channel >= 0 &&
        radio_medium.nodes[idx].radios[radio_idx].spectrum
            == RADIO_SPECTRUM_NONE) {
        radio_spectrum_t want = (radio_idx == 0)
            ? RADIO_SPECTRUM_2_4GHZ_15_4
            : RADIO_SPECTRUM_868MHZ_15_4G;
        radio_medium_register_radio(&radio_medium, idx, radio_idx, want);
    }
    radio_medium_set_radio_channel(&radio_medium, idx, radio_idx, channel);
    /* Legacy alias — the CCA channel-busy query and a couple of older
     * call sites still read radio_medium.nodes[i].channel directly.
     * Mirror writes onto the legacy alias so those readers keep
     * returning the same answer they did before the per-radio refactor:
     *   - sub-GHz radios (slot 1) appear as RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE
     *     + their channel (legacy "channel >= base means sub-GHz")
     *   - 2.4 GHz radios (slot 0) write the raw channel ONLY if slot 1
     *     is not registered. On dual-radio Firefly nodes the cc1200 is
     *     the active radio (NETSTACK_RADIO=cc1200_driver), so the
     *     CCA-busy query keys on its channel. Letting cc2538_rfcore
     *     (parked) stomp the alias would flip the band gate and kill
     *     all sub-GHz CCA. */
    if (radio_idx == 1 && channel >= 0) {
        radio_medium.nodes[idx].channel =
            RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE + channel;
    } else if (radio_idx == 0) {
        bool slot1_registered =
            radio_medium.nodes[idx].radios[1].spectrum != RADIO_SPECTRUM_NONE;
        if (!slot1_registered)
            radio_medium.nodes[idx].channel = channel;
    }
}

/* sim_host_t adapter (off-SoC chip drivers). */
static void mixed_host_radio_set_channel(void *user_data, int radio_idx,
                                          int channel) {
    mixed_node_radio_set_channel((mixed_node_t *)user_data, radio_idx, channel);
}

/* cc2538_rfcore observer adapter (on-SoC, slot 0 = 2.4 GHz). */
static void mixed_rfcore_channel_callback(void *user_data, int channel) {
    mixed_node_radio_set_channel((mixed_node_t *)user_data, /*radio_idx=*/0, channel);
}

/*
 * CC1200 channel-busy query — called from the chip driver's RSSI0 read
 * path.  Returns true if any neighbor on this node's channel is currently
 * mid-frame transmit (per radio_bus.tx_busy_until_ns[]).  This drives the
 * CARRIER_SENSE bit so CSMA backs off when another node is on-air,
 * even before the first preamble byte has reached our chip's air
 * decoder.  Without this, CCA always saw a clear channel and senders
 * blasted on top of each other — root cause of L6 RPL non-convergence.
 */
static bool mixed_cc1200_channel_busy(void *user_data) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    int idx = (int)(node - nodes);
    int my_ch = radio_medium.nodes[idx].channel;
    /* Only consider TX-range neighbors — interference-range nodes are too
     * far for our CCA to detect.  Use the precomputed neighbor list. */
    neighbor_list_t *nbrs = (radio_medium.type == RADIO_MEDIUM_NONE)
        ? NULL : &radio_medium.neighbors[idx];
    int n = nbrs ? nbrs->count : 0;
    for (int k = 0; k < n; k++) {
        int j = nbrs->neighbors[k];
        if (j == idx) continue;
        if (radio_bus.tx_busy_until_ns[j] <= sim_runtime_now_ns(&sim_rt)) continue;
        /* Cross-band check: 2.4 GHz neighbors don't affect sub-GHz CCA. */
        int their_ch = radio_medium.nodes[j].channel;
        if (my_ch >= 0 && their_ch >= 0) {
            bool my_sub    = (my_ch    >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
            bool their_sub = (their_ch >= RADIO_MEDIUM_SUBGHZ_CHANNEL_BASE);
            if (my_sub != their_sub) continue;
        }
        return true;
    }
    /* Fallback (no medium configured / no neighbor list): scan all nodes. */
    if (!nbrs) {
        for (int j = 0; j < num_nodes; j++) {
            if (j == idx) continue;
            if (radio_bus.tx_busy_until_ns[j] > sim_runtime_now_ns(&sim_rt))
                return true;
        }
    }
    return false;
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
 *
 * M9.4: the per-byte path (byte clock, capture, frame assembly,
 * medium-filtered dispatch, re-entrancy depth) lives in
 * sim_radio_bus_tx_byte(); the runner provides the host hooks below —
 * node lifecycle, native channel pulls, debug stats, and the
 * frame-complete delivery machinery (which mini-steps receivers and so
 * must stay outside the bus per refactor-plan §3.9). */

/* Legacy entry: callers that don't carry a sender_radio (native motes,
 * JS motes, frame-to-byte conversion in mixed_js_rf_handler) treat the
 * sender as slot 0. */
static void mixed_rf_tx_handler(void *user_data, uint8_t byte) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    mixed_rf_tx_handler_radio(sender_idx, 0, byte);
}

static void mixed_rf_tx_handler_radio(int sender_idx, int sender_radio, uint8_t byte) {
    if (sender_idx < 0 || sender_idx >= num_nodes) return;
    rf_byte_count++;
    channels_dirty = true;
    if (csim_radio_trace_enabled()) {
        int s_ch = (sender_idx >= 0 && sender_radio >= 0)
            ? radio_medium.nodes[sender_idx].radios[sender_radio].channel : -1;
        RTRACE("tx_byte node=%d radio=%d ch=%d byte=0x%02x state=%d zc=%d",
               sender_idx, sender_radio, s_ch, byte,
               tx_asm[sender_idx].state, tx_asm[sender_idx].zero_count);
    }
    /* Native sender: pull channel into the medium right now. Chip-emulated
     * senders push synchronously through their FSCTRL/FREQ callbacks, so
     * this only matters for native (Cooja) motes. */
    sync_native_node_channel(sender_idx);
    sim_radio_bus_tx_byte(&radio_bus, &sim_rt, sender_idx, sender_radio, byte);
}

/* --- Radio-bus host hooks (M9.4) ----------------------------------- */

static bool bus_host_node_active(void *user, int idx) {
    (void)user;
    return node_active(idx) != 0;
}

static void bus_host_sync_channel(void *user, int idx) {
    (void)user;
    sync_native_node_channel(idx);
}

/* M26: the bus delivered a frame's on-air bytes to emulated receiver
 * idx over [start_ns, end_ns]; emit the RX timeline + radio state (the
 * observability that used to live inline in emu_deliver_bytes). */
static void bus_host_on_rx(void *user, int idx, int64_t start_ns,
                           int64_t end_ns) {
    (void)user;
    if (!ui_server) return;
    emit_radio_obs(idx, start_ns, SIM_OBS_RADIO_RX_START);
    emit_radio_obs(idx, end_ns, SIM_OBS_RADIO_RX_END);
    node_states[idx].radio_state = SIM_RADIO_ON;
}

static void bus_host_on_tx_byte(void *user, int sender, uint8_t byte,
                                int64_t byte_time_ns, int depth) {
    (void)user;
    node_last_tx_ns[sender] = byte_time_ns;
    cc2420_t *scc = mote_cc2420(sender);
    if (depth > 0 && trace_tsch_ack_enabled() && scc) {
        trace_tsch_ack_log("reentrant tx byte sender=%d state=%s byte=%02x depth=%d",
                           nodes[sender].id,
                           cc2420_state_str(scc->state),
                           byte, depth);
    }
}

static void bus_host_on_byte_accepted(void *user, int sender, int receiver,
                                      uint8_t byte, int depth) {
    (void)user;
    /* CC2420-carrying receivers only (M16: chip presence, not type). */
    cc2420_t *rcc = mote_cc2420(receiver);
    if (!rcc) return;
    if (depth > 0 && trace_tsch_ack_enabled() &&
        mote_cc2420(sender) != NULL &&
        nodes[sender].id == 1 && nodes[receiver].id == 2) {
        trace_tsch_ack_log("ack-byte queued 1->2 byte=%02x rx_state=%s node2_sim=%.6f",
                           byte,
                           cc2420_state_str(rcc->state),
                           (double)node_sim_time_ns(receiver) / 1e9);
    }
    stat_msp_byte_push[receiver]++;
    if (nodes[sender].id == 2 && nodes[receiver].id == 1)
        stat_msp_byte_push_2_to_1++;
}

/* Frame-complete observability hooks (M27): the delivery policy moved
 * to sim_radio_bus_frame_complete; these emit the TX/RX timeline, PCAP,
 * packet-analyzer prints, and traces from the bus's notifications. */
static void bus_host_frame_observed(void *user,
                                    const sim_radio_frame_info_t *fi) {
    (void)user;
    int sender_idx = fi->sender_idx;
    int sender_radio = fi->sender_radio;
    int64_t accurate_tx_start = fi->tx_start_ns;
    int64_t accurate_tx_end = fi->tx_end_ns;

    if (csim_radio_trace_enabled()) {
        int s_ch = radio_medium.nodes[sender_idx].radios[sender_radio].channel;
        RTRACE("tx_frame_complete node=%d radio=%d ch=%d "
               "subghz=%d expected_len=%d payload_count=%d",
               sender_idx, sender_radio, s_ch,
               fi->subghz ? 1 : 0, fi->expected_len,
               tx_asm[sender_idx].payload_count);
    }

    stat_rf_frames++;

    cc2420_t *trace_scc = mote_cc2420(sender_idx);
    if (trace_tsch_ack_enabled() && trace_scc &&
        (fi->expected_len == 23 || fi->expected_len == 19 ||
         fi->expected_len == 17)) {
        trace_tsch_ack_log("frame-complete sender=%d len=%d start=%.6f end=%.6f state=%s",
                           nodes[sender_idx].id, fi->expected_len,
                           (double)accurate_tx_start / 1e9,
                           (double)accurate_tx_end / 1e9,
                           cc2420_state_str(trace_scc->state));
    }

    if (ui_server) {
        emit_radio_obs(sender_idx, accurate_tx_start, SIM_OBS_RADIO_TX_START);
        emit_radio_obs(sender_idx, accurate_tx_end,   SIM_OBS_RADIO_TX_END);
        node_states[sender_idx].radio_state = SIM_RADIO_ON;
    }

    if (ui_server || verbose || pcap_writer_is_open(&pcap_writer)) {
        const uint8_t *buf = fi->capture;
        int buf_len = fi->capture_len;
        int fstart = -1;
        for (int b = 0; b + 5 < buf_len; b++) {
            if (buf[b] == 0x7A && b >= 4) {
                fstart = b + 1;
                break;
            }
        }
        if (fstart >= 0 && fstart < buf_len) {
            int mac_off = fstart + 1;
            int mac_len = buf_len - mac_off;
            if (pcap_writer_is_open(&pcap_writer) && mac_len > 0) {
                pcap_writer_packet(&pcap_writer, accurate_tx_start,
                                   buf + mac_off, mac_len);
            }
            pkt_info_t pinfo;
            int frame_len = buf_len - fstart;
            pkt_analyze(buf + fstart, frame_len, &pinfo);
            if (verbose)
                fprintf(stderr, "  [PKT] Node %d TX: %s\n",
                        nodes[sender_idx].id, pinfo.summary);
            if (verbose && frame_len < 120 &&
                (pinfo.dst_mode == 3 || frame_len > 80)) {
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
                emit_frame_obs(sender_idx, accurate_tx_start, true,
                               pinfo.summary);
        }
    }

    /* [RF] receiver-list trace (verbose): which receivers would pass the
     * spectrum/channel gate.  Non-probabilistic check; matches the bus's
     * snapshot eligibility. */
    if (verbose) {
        fprintf(stderr, "  [RF] Node %d TX frame (%d bytes) @ %.3f s -> receivers:",
                nodes[sender_idx].id, fi->expected_len,
                (double)sim_runtime_now_ns(&sim_rt) / 1e9);
        for (int i = 0; i < num_nodes; i++) {
            if (i == sender_idx ||
                radio_bus.delivery[i] == SIM_RADIO_DELIVERY_SYNC ||
                !node_active(i))
                continue;
            int rr = sim_radio_bus_pick_receiver_radio(&radio_medium,
                                                       sender_idx,
                                                       sender_radio, i);
            if (rr < 0) continue;
            if (radio_medium_filter_frame_radio(&radio_medium, sender_idx,
                                                 sender_radio, i, rr))
                fprintf(stderr, " %d", nodes[i].id);
        }
        fprintf(stderr, "\n");
    }
}

/* Per emulated receiver: the bus's delivery outcome for this frame. */
static void bus_host_on_rx_frame(void *user, const sim_radio_frame_info_t *fi,
                                 int receiver, int outcome,
                                 const uint8_t *data, int len,
                                 int64_t coll_start_ns, int64_t prev_rx_end_ns) {
    (void)user;
    int sender_idx = fi->sender_idx;
    int i = receiver;
    switch (outcome) {
    case SIM_RADIO_RX_STALE_BUFFER:
        if (verbose)
            fprintf(stderr,
                    "  [RF-BUF] dropping stale/misaligned buffer for node %d: "
                    "count=%d expected=%d sender=%d\n",
                    nodes[i].id, len, fi->total_air_bytes, nodes[sender_idx].id);
        break;
    case SIM_RADIO_RX_COLLIDED:
        if (verbose)
            fprintf(stderr, "  [COLLISION] Node %d->%d frame dropped @ %.3f s "
                    "(channel busy until %.3f s)\n",
                    nodes[sender_idx].id, nodes[i].id,
                    (double)coll_start_ns / 1e9,
                    (double)prev_rx_end_ns / 1e9);
        if (ui_server) {
            int64_t intf_dur = (int64_t)len * fi->sender_byte_ns;
            emit_radio_obs(i, fi->tx_start_ns, SIM_OBS_RADIO_INTERFERENCE);
            emit_radio_obs(i, fi->tx_start_ns + intf_dur, SIM_OBS_RADIO_RX_END);
        }
        break;
    case SIM_RADIO_RX_DIRECT: {
        cc2420_t *trace_rcc = mote_cc2420(i);
        if (trace_tsch_ack_enabled() &&
            mote_cc2420(sender_idx) != NULL && nodes[sender_idx].id == 2 &&
            trace_rcc != NULL && nodes[i].id == 1 && fi->expected_len == 23) {
            int64_t delivery_start = fi->subghz ? sim_runtime_now_ns(&sim_rt)
                                                : fi->tx_start_ns;
            trace_tsch_ack_log("deliver data 2->1 start=%.6f node1_state=%s",
                               (double)delivery_start / 1e9,
                               cc2420_state_str(trace_rcc->state));
        }
        if (ui_server && len > 5) {
            int fstart = -1;
            for (int b = 0; b + 5 < len; b++) {
                if (data[b] == 0x7A && b >= 4) { fstart = b + 1; break; }
            }
            if (fstart >= 0 && fstart < len) {
                pkt_info_t rxinfo;
                pkt_analyze(data + fstart, len - fstart, &rxinfo);
                emit_frame_obs(i, fi->tx_start_ns, false, rxinfo.summary);
            }
        }
        break;
    }
    case SIM_RADIO_RX_QUEUED:
    default:
        break;
    }
}

/* A data delivery to `data_receiver` triggered an auto-ACK. */
static void bus_host_on_ack(void *user, const sim_radio_frame_info_t *fi,
                            int data_receiver, int64_t ack_start_ns,
                            int ack_byte_count) {
    (void)user;
    if (!ui_server) return;
    int64_t ack_dur = (int64_t)ack_byte_count * fi->sender_byte_ns;
    emit_radio_obs(data_receiver, ack_start_ns, SIM_OBS_RADIO_TX_START);
    emit_radio_obs(data_receiver, ack_start_ns + ack_dur, SIM_OBS_RADIO_TX_END);
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
    node_last_tx_ns[sender_idx] = sim_runtime_now_ns(&sim_rt);
    /* Native sender: snap current channel into the medium before the
     * neighbour-loop filter calls. */
    sync_native_node_channel(sender_idx);

    if (radio_medium.type != RADIO_MEDIUM_NONE) {
        /* UDGM: iterate precomputed TX-range neighbors */
        neighbor_list_t *nl = &radio_medium.neighbors[sender_idx];
        for (int n = 0; n < nl->count; n++) {
            int i = nl->neighbors[n];
            if (nodes[i].type == NODE_NATIVE) {
                /* Native receiver: snap channel before filter consults it. */
                sync_native_node_channel(i);
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
                                (uint64_t)(sim_runtime_now_ns(&sim_rt) / 1000LL);
                    } else {
                        native_deliver_frame(rcv, frame, len,
                                             sim_runtime_now_ns(&sim_rt), sender_idx);
                    }
                    stat_rx_frames_queued++;
                }
            } else if (mote_store[i].ops->receive_frame) {
                /* JS (or any future frame-consuming mote kind) — M17 op */
                if (radio_medium_filter_frame(&radio_medium, sender_idx, i)) {
                    if (mote_store[i].ops->receive_frame(
                            &mote_store[i], frame, len,
                            sim_runtime_now_ns(&sim_rt), sender_idx) < 0)
                        stat_rx_frames_queue_full++;
                    stat_rx_frames_queued++;
                }
            }
            /* Native/JS-to-emulated: handled via rf_tx_callback (byte stream) */
        }

        /* Interference-range neighbors: mark overlapping frames as collided */
        neighbor_list_t *inl = &radio_medium.interference_neighbors[sender_idx];
        int64_t tx_start = sim_runtime_now_ns(&sim_rt);
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
                        if (!existing->collided) radio_bus.stats.rx_collided++;
                        existing->collided = true;
                    }
                }
            }
        }
    } else {
        /* NONE: deliver to all other frame-consuming nodes (M17 op;
         * NULL = emulated, fed via the byte stream instead) */
        for (int i = 0; i < num_nodes; i++) {
            if (&nodes[i] == sender) continue;
            sim_mote_t *m = &mote_store[i];
            if (m->ops->receive_frame) {
                if (m->ops->receive_frame(m, frame, len,
                                          sim_runtime_now_ns(&sim_rt),
                                          sender_idx) < 0)
                    stat_rx_frames_queue_full++;
                stat_rx_frames_queued++;
            }
        }
    }
}

/* Deliver buffered RF bytes to a native node's assembler.
 * Only called for native nodes — emulated nodes' bytes stay in rf_pending
 * until the per-sender frame assembler detects a complete frame.
 * Native nodes use the 2.4 GHz framing in this test runner. */
static void mixed_deliver_rf_bytes(int idx) {
    rf_buffer_t *buf = &rf_pending[idx];
    if (buf->count == 0) return;
    emu_deliver_bytes(idx, buf->bytes, buf->count, 0, sim_runtime_now_ns(&sim_rt),
                      /*subghz=*/false);
    buf->count = 0;
}

/* Drain queued RX frames for an emulated node — body moved to
 * sim_radio_bus_drain_rx (M26); thin forwarder keeps call sites
 * unchanged. */
static void emu_rx_queue_drain(int idx) {
    sim_radio_bus_drain_rx(&radio_bus, &sim_rt, idx);
}

/* --- UART callback --- */

static void mixed_uart_callback(void *user_data, uint8_t byte) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    uart_byte_count++;

    /* Kernel observer stream: every console byte, every node.  The
     * serial bridge service filters on its bridged mote; other
     * subscribers (timeline) ignore the kind.  See refactor plan §3.11
     * for why raw bytes and assembled lines are separate kinds. */
    {
        int nidx = (int)(node - nodes);
        sim_observer_event_t ev = { .kind = SIM_OBS_MOTE_UART_BYTE,
                                    .time_ns = node_sim_time_ns(nidx),
                                    .mote_index = nidx, .radio_idx = -1 };
        ev.u.uart.byte = byte;
        sim_runtime_emit(&sim_rt, &ev);
    }

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        int nidx = (int)(node - nodes);
        int64_t ns = node_sim_time_ns(nidx);
        if (verbose)
            printf("  %7.3f [Node %d/%s] %s\n", (double)ns / 1e9,
                   node->id, node_type_str(nidx), node->line_buf);
        /* test engines receive this line via test_engine_observer */
        if (ui_server)
            ui_add_console_line(nidx, ns, node->line_buf);
        /* Kernel observer stream: assembled console line.  The
         * external-command service tees this into COOJA.testlog. */
        {
            sim_observer_event_t lev = { .kind = SIM_OBS_MOTE_LOG_LINE,
                                         .time_ns = ns,
                                         .mote_index = nidx, .radio_idx = -1 };
            lev.u.log_line.line = node->line_buf;
            lev.u.log_line.len = node->line_pos;
            lev.u.log_line.node_id = node->id;
            sim_runtime_emit(&sim_rt, &lev);
        }
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
    }
}

/* --- Serial bridge inject callback ---
 *
 * Socket + ring plumbing lives in sim_serial_bridge (milestone 8.1);
 * this callback is the mote-type-specific injection half the bridge
 * drains its RX ring through.  It stays in the runner because native
 * injection touches the shared-memory serial buffer, MSP430 injection
 * steps the CPU at baud-rate intervals, and ARM injection feeds the
 * CC2538 UART directly — all of which need node internals until the
 * Phase 2 mote vtable provides a serial-input op. */

/* ============================================================
 * M14: serial_input mote-op adapters + the single injection helper.
 *
 * One delivery contract per platform (bodies verbatim from the former
 * ss_inject_into_node type switch, generalized from ss_node_idx to the
 * mote's own slot).  All five former injection sites — the serial
 * bridge callback and the four TEST_ACTION_SEND/SEND_ALL blocks —
 * funnel through inject_serial().
 * ============================================================ */

/* (Native serial_input — Cooja rs232 append + immediate wakeup — lives
 * in native_cooja_mote.c, M20.) */

/* MSP430: match MSPSim's MspSerial — inject ALL pending bytes at
 * baud-rate intervals (69µs = 115200 baud), stepping CPU between each
 * byte.  Cooja queues all bytes and delivers them at baud-rate
 * intervals with requestImmediateWakeup() after each.
 *
 * Mark this node executing (radio_bus.executing_node) to prevent a
 * re-entrant cascade: if the CPU TX's during the baud-rate step, the
 * delivery path must not sync this node back. */
static int msp_mote_serial_input(sim_mote_t *m, const uint8_t *buf,
                                 int len) {
    mixed_node_t *node = MOTE_IMPL(m);
    int idx = MOTE_IDX(m);
    msp430_platform_t *plat = &node->plat.msp;
    msp430_usart_t *console = (plat->config->console_usart == 0)
        ? &plat->usart0 : &plat->usart1;
    bool has_dma = (console->dma_trigger != NULL);
    if (!has_dma) {
        if (!console->ie_ptr ||
            !(*console->ie_ptr & console->ie_rx_mask))
            return 0;  /* firmware hasn't enabled UART RX yet */
    }
    int baud_cycles = (int)((int64_t)node_freq(idx) * 69 / 1000000);
    if (baud_cycles < 200) baud_cycles = 200;
    int consumed = 0;
    sim_radio_bus_set_executing(&radio_bus, idx);
    while (consumed < len) {
        if (!has_dma) {
            /* Wait for RXIFG clear (firmware consumed previous byte) */
            if (console->ifg_ptr &&
                (*console->ifg_ptr & console->ifg_rx_mask))
                break;  /* try again next poll */
        }
        msp430_usart_receive_byte(console, buf[consumed]);
        consumed++;
        step_node_until(idx, node_cycles(idx) + baud_cycles);
    }
    sim_radio_bus_set_executing(&radio_bus, -1);
    /* Sync last_execute_us and re-schedule in event queue.
     * Without this, the node's sim_eq entry is stale and the
     * trickle timer / other events scheduled during injection
     * won't fire until the old wakeup time.  Matches Cooja's
     * requestImmediateWakeup() after serial byte delivery. */
    if (consumed > 0) {
        msp430_cpu_t *cpu = &node->plat.msp.cpu;
        cpu->last_execute_us = (int64_t)msp430_cycles_to_ns(
            cpu->cycles, cpu->cpu_freq_hz) / 1000LL;
        if (num_threads == 0)
            schedule_emulated_wakeup(&sim_rt, idx);
    }
    return consumed;
}

/* ARM: feed the console UART RX register directly.  (CC2538-family
 * boards only — the cc2538_soc lookup matches the pre-M14 behavior;
 * Nordic consoles are output-only today.) */
static int arm_mote_serial_input(sim_mote_t *m, const uint8_t *buf,
                                 int len) {
    cc2538_soc_t *soc = arm_platform_cc2538(&MOTE_IMPL(m)->plat.arm);
    for (int i = 0; i < len; i++)
        cc2538_uart_receive_byte(&soc->uart0, buf[i]);
    return len;
}

/* (JS serial_input — no console input — lives in js_app_mote.c, M19.) */

/* The single serial-injection entry point: dispatch to the mote's
 * platform delivery contract.  Returns bytes consumed. */
static int inject_serial(int idx, const uint8_t *buf, int len) {
    if (idx < 0 || idx >= num_nodes || len <= 0) return 0;
    sim_mote_t *m = &mote_store[idx];
    return m->ops->serial_input(m, buf, len);
}

/* Serial-bridge callback: inject into the bridged node. */
static int ss_inject_into_node(void *user, const uint8_t *buf, int len) {
    (void)user;
    return inject_serial(ss_node_idx, buf, len);
}

static void ss_cleanup(void) {
    sim_serial_bridge_stop(&serial_bridge);
    sim_external_command_stop(&external_cmd);
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
    ts->last_tx_ns = sim_runtime_now_ns(&sim_rt);

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
    ts->last_tx_ns = sim_runtime_now_ns(&sim_rt);

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
            /* Native sender: pull channel once per sender before this batch. */
            sync_native_node_channel(sender);
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
                    /* Native receiver: pull current channel inline. */
                    sync_native_node_channel(i);
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
                if (sim_radio_bus_asm_feed(&tx_asm[sender], byte)) {
                    stat_rf_frames++;
                    /* Deliver or queue each emulated receiver's buffered frame */
                    for (int i = 0; i < num_nodes; i++) {
                        rf_buffer_t *buf = &rf_pending[i];
                        if (buf->count == 0 || nodes[i].type == NODE_NATIVE)
                            continue;
                        int64_t tx_start = sim_runtime_now_ns(&sim_rt);
                        int64_t tx_end = sim_runtime_now_ns(&sim_rt) + TIME_STEP_NS;

                        /* Collision check against previously delivered frame */
                        if (tx_start < emu_rx_end_ns[i]) {
                            radio_bus.stats.rx_collided++;
                            buf->count = 0;
                            continue;
                        }

                        int8_t rssi = radio_medium_get_rssi(&radio_medium, sender, i);
                        int frame_payload = (buf->count > 5) ? buf->bytes[5] : 0;
                        if (emulated_rxfifo_available(i) < frame_payload + 1)
                            step_node_until(i, node_cycles(i) + 5000);
                        /* Native senders only emit 2.4 GHz frames in this
                         * runner — pass subghz=false. */
                        if (emulated_rxfifo_available(i) >= frame_payload + 1) {
                            emu_deliver_bytes(i, buf->bytes, buf->count, rssi, tx_start,
                                              /*subghz=*/false);
                            radio_bus.stats.rx_direct++;
                            emu_rx_end_ns[i] = tx_end;
                        } else {
                            emu_rx_queue_push(i, buf->bytes, buf->count, rssi,
                                              tx_start, tx_end, /*subghz=*/false);
                            emu_rx_end_ns[i] = tx_end;
                        }
                        buf->count = 0;
                    }

                    /* Flush auto-ACK bytes generated by this delivery */
                    {
                        int64_t native_ack_start = sim_runtime_now_ns(&sim_rt) + 192000LL;
                        for (int j = 0; j < num_nodes; j++) {
                            if (rf_pending[j].count > 0 && nodes[j].type != NODE_NATIVE) {
                                int ack_payload = (rf_pending[j].count > 5) ? rf_pending[j].bytes[5] : 0;
                                if (emulated_rxfifo_available(j) >= ack_payload + 1)
                                    emu_deliver_bytes(j, rf_pending[j].bytes, rf_pending[j].count,
                                                      -50, native_ack_start, /*subghz=*/false);
                                else
                                    emu_rx_queue_push(j, rf_pending[j].bytes, rf_pending[j].count,
                                                      -50, sim_runtime_now_ns(&sim_rt),
                                                      sim_runtime_now_ns(&sim_rt) + TIME_STEP_NS,
                                                      /*subghz=*/false);
                                rf_pending[j].count = 0;
                            }
                        }
                    }

                    /* Interference check for emulated nodes */
                    if (radio_medium.type != RADIO_MEDIUM_NONE) {
                        neighbor_list_t *inl = &radio_medium.interference_neighbors[sender];
                        int64_t int_start = sim_runtime_now_ns(&sim_rt);
                        int64_t int_end = sim_runtime_now_ns(&sim_rt) + TIME_STEP_NS;
                        for (int n = 0; n < inl->count; n++) {
                            int i = inl->neighbors[n];
                            if (nodes[i].type == NODE_NATIVE) continue;
                            emu_rx_queue_t *q = &emu_rx_queue[i];
                            for (int f = 0; f < q->count; f++) {
                                int fi = (q->head + f) % EMU_RX_QUEUE_SIZE;
                                emu_rx_frame_t *existing = &q->frames[fi];
                                if (int_start < existing->end_ns &&
                                    existing->arrival_ns < int_end) {
                                    if (!existing->collided) radio_bus.stats.rx_collided++;
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
        if (fout->count > 0)
            sync_native_node_channel(sender);
        for (int f = 0; f < fout->count; f++) {
            uint8_t *frame = fout->data[f];
            int len = fout->lengths[f];

            if (radio_medium.type != RADIO_MEDIUM_NONE) {
                neighbor_list_t *nl = &radio_medium.neighbors[sender];
                for (int n = 0; n < nl->count; n++) {
                    int i = nl->neighbors[n];
                    sim_mote_t *m = &mote_store[i];
                    if (m->ops->receive_frame) {
                        sync_native_node_channel(i);
                        if (radio_medium_filter_frame(&radio_medium, sender, i)) {
                            if (m->ops->receive_frame(m, frame, len,
                                                      sim_runtime_now_ns(&sim_rt),
                                                      sender) < 0)
                                stat_rx_frames_queue_full++;
                            stat_rx_frames_queued++;
                        }
                    }
                }
                /* Interference collision detection */
                neighbor_list_t *inl = &radio_medium.interference_neighbors[sender];
                int64_t tx_start = sim_runtime_now_ns(&sim_rt);
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
                                if (!existing->collided) radio_bus.stats.rx_collided++;
                                existing->collided = true;
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < num_nodes; i++) {
                    sim_mote_t *m = &mote_store[i];
                    if (i != sender && m->ops->receive_frame) {
                        if (m->ops->receive_frame(m, frame, len,
                                                  sim_runtime_now_ns(&sim_rt),
                                                  sender) < 0)
                            stat_rx_frames_queue_full++;
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
                printf("  %7.3f [Node %d/%s] %s\n", (double)sim_runtime_now_ns(&sim_rt) / 1e9,
                       nid, node_type_str(nidx), ts->lines[l]);
            /* test engines receive this line via test_engine_observer */
            if (ui_server)
                ui_add_console_line(nidx, sim_runtime_now_ns(&sim_rt), ts->lines[l]);
            {
                sim_observer_event_t lev = { .kind = SIM_OBS_MOTE_LOG_LINE,
                                             .time_ns = sim_runtime_now_ns(&sim_rt),
                                             .mote_index = nidx, .radio_idx = -1 };
                lev.u.log_line.line = ts->lines[l];
                lev.u.log_line.len = (int)strlen(ts->lines[l]);
                lev.u.log_line.node_id = nid;
                sim_runtime_emit(&sim_rt, &lev);
            }
        }
        ts->count = 0;
    }
}

/* (Extension → board → mote-kind resolution: sim_board_for_path()
 * row + sim_mote_kind_for() — the Phase 3 extension ladders and the
 * Phase 4 node_type_for_board switch are both gone, M23.) */

/* --- MSP430 node initialization --- */

/* (MSP430 boot policy — ELF load, ds2411/xmem/infomem/node-id/
 * Z1-linkaddr patches, run-to-main, run-to-ready — lives in
 * src/motes/msp430_elf_mote.c, M21.) */

/* --- ARM node initialization --- */

/* (ARM boot policy — ELF load, cc2538/firefly/nrf52840/nrf54l15 chip
 * wiring, FICR/RFRND seeding, node-id/linkaddr patches, run-to-main,
 * run-until-event + step-past-first-event — lives in
 * src/motes/arm_elf_mote.c, M22.) */

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

/* (Native boot policy lives in src/motes/native_cooja_mote.c — M20.
 * native_yield_callback above stays runner-side: it is cross-node
 * ACK-turnaround policy, injected via env->native_yield.) */

/* JS node TX handler: when mote.send(bytes) fires, bridge the frame to
 * both native receivers (frame-level) and emulated receivers (byte-stream
 * via mixed_rf_tx_handler with PHY wrapping). */
static void mixed_js_rf_handler(void *user_data, const uint8_t *frame, int len) {
    mixed_node_t *sender = (mixed_node_t *)user_data;

    /* Frame-level delivery to native receivers */
    mixed_rf_frame_handler(user_data, frame, len);

    /* Byte-stream delivery to emulated receivers: wrap with PHY
     * (4x0x00 preamble + 0x7A SFD + length) and feed mixed_rf_tx_handler. */
    uint8_t bytes[160];
    int n = native_frame_to_bytes(frame, len, bytes, (int)sizeof(bytes));
    for (int i = 0; i < n; i++)
        mixed_rf_tx_handler(sender, bytes[i]);
}

/* (JS boot policy lives in src/motes/js_app_mote.c — M19.) */

/* --- Top-level node init (dispatches by type) --- */

/* TSCH channel sync for native motes (M20): the module's tick pushes
 * the mote's current channel into the runner-owned radio medium.
 * Dissolves into the bus in Phase 5. */
static void mixed_native_channel_sync(int slot, int channel) {
    radio_medium_set_channel(&radio_medium, slot, channel);
}

/* Runner glue bundle for the per-kind mote modules (Phase 4, M18 —
 * see mote_impl.h).  One static instance; init_node() stamps it on
 * every node.  All referenced callbacks are defined above.  The
 * MSP430/ARM boot bodies still call the callbacks directly until
 * M21/M22 move them into src/motes/. */
static const sim_mote_env_t mixed_mote_env = {
    .sim                   = &sim_rt,
    .radio_bus             = &radio_bus,
    .verbose               = &verbose,
    .num_threads           = &num_threads,
    .node_start_ns         = node_start_ns,
    .uart_byte             = mixed_uart_callback,
    .chip_tx_byte          = mixed_rf_tx_chip_cb,
    .radio_set_channel     = mixed_host_radio_set_channel,
    .rfcore_state_change   = mixed_rf_state_handler,
    .rfcore_channel_change = mixed_rfcore_channel_callback,
    .cc1200_channel_busy   = mixed_cc1200_channel_busy,
    .rf_tx_byte            = mixed_rf_tx_handler,
    .rf_frame              = mixed_rf_frame_handler,
    .native_yield          = native_yield_callback,
    .js_rf_frame           = mixed_js_rf_handler,
    .native_channel_sync   = mixed_native_channel_sync,
};

static int init_node(int idx, const char *firmware_path, int node_id) {
    mixed_node_t *node = &nodes[idx];
    memset(node, 0, sizeof(*node));
    /* Phase 3: one registry lookup owns the board decision — node kind,
     * arch platform name, and banner label all come from the row.
     * M23: the kind row owns boot + radio registration + ops. */
    node->board = sim_board_for_path(firmware_path);
    const sim_mote_kind_t *kind = sim_mote_kind_for(node->board->kind);
    node->type = kind->node_type;
    node->id = node_id;
    node->slot = idx;
    node->env = &mixed_mote_env;
    snprintf(node->firmware_path, sizeof(node->firmware_path), "%s", firmware_path);
    /* M11: bind + register the kernel-facing mote object early so the
     * node_* vtable accessors work during platform init below. */
    register_node_mote(idx, kind->ops);
    memset(&rf_pending[idx], 0, sizeof(rf_pending[idx]));
    memset(&emu_rx_queue[idx], 0, sizeof(emu_rx_queue[idx]));
    memset(&tx_cap[idx], 0, sizeof(tx_cap[idx]));
    sim_cancel_mote_events(&sim_rt, idx);
    sim_runtime_bump_mote_generation(&sim_rt, idx);
    sim_radio_bus_reset_node(&radio_bus, idx);
    emu_rx_end_ns[idx] = 0;
    sim_radio_bus_asm_reset(&tx_asm[idx]);

    printf("Initializing node %d (%s) as %s...\n", node_id, firmware_path,
           node->board->label);

    int rc = kind->boot(node, idx, firmware_path, node_id, &mixed_mote_env);
    if (rc != 0)
        return rc;

    /* M9.3/9.4: radio endpoint ops + delivery mode onto the bus.  Done
     * here (not in a one-shot loop in main) so reboots and dynamically
     * added motes stay registered.  Strictly after boot — js_node_start
     * can TX during script init(), which historically happens before
     * radio registration (§3.17). */
    kind->register_radio(node, idx, &radio_bus);
    return 0;
}

/* --- Destroy node (M15: lifecycle ops) --- */

static void msp_mote_destroy(sim_mote_t *m) {
    msp430_platform_destroy(&MOTE_IMPL(m)->plat.msp);
}
static void arm_mote_destroy(sim_mote_t *m) {
    arm_platform_destroy(&MOTE_IMPL(m)->plat.arm);
}

/* Re-seed the mote's local clock after reboot/dynamic add so stepping
 * resumes at the current sim time (don't catch up from t=0).  Bodies
 * verbatim from the former TEST_ACTION_ADD type switches (the JS body,
 * now in js_app_mote.c, fixed a latent union aliasing slip — the old
 * else-branch wrote plat.native.sim_time_ns even for JS motes). */
static void msp_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    msp430_cpu_t *cpu = &MOTE_IMPL(m)->plat.msp.cpu;
    cpu->sim_time_ns = now_ns;
    cpu->cycles = (uint64_t)now_ns * cpu->cpu_freq_hz / 1000000000ULL;
}
static void arm_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    cpu->sim_time_ns = now_ns;
    cpu->cycles = now_ns * cpu->cpu_freq_hz / 1000000000LL;
}

/* --- M16 introspection adapters --- */

static int msp_mote_ui_radio_state(const sim_mote_t *m) {
    cc2420_radio_state_t rs = MOTE_IMPL(m)->plat.msp.cc2420.state;
    if (rs >= CC2420_TX_CALIBRATE && rs <= CC2420_TX_UNDERFLOW)
        return SIM_RADIO_TX;
    if (rs == CC2420_RX_FRAME)
        return SIM_RADIO_RX;        /* actively receiving data (green) */
    if (rs == CC2420_RX_CALIBRATE || rs == CC2420_RX_SFD_SEARCH)
        return SIM_RADIO_ON;        /* on, listening (gray) */
    return SIM_RADIO_OFF;           /* VREG_OFF, POWER_DOWN, IDLE */
}

static void msp_mote_ui_leds(const sim_mote_t *m, uint8_t leds[3]) {
    uint8_t p5out = MOTE_IMPL(m)->plat.msp.gpio.ports[4].out;
    leds[0] = (p5out >> 4) & 1;  /* green  = P5.4 */
    leds[1] = (p5out >> 5) & 1;  /* yellow = P5.5 */
    leds[2] = (p5out >> 6) & 1;  /* red    = P5.6 */
}

static void arm_mote_ui_leds(const sim_mote_t *m, uint8_t leds[3]) {
    cc2538_soc_t *soc = arm_platform_cc2538(&MOTE_IMPL(m)->plat.arm);
    if (!soc)
        return;  /* Nordic boards: no LED model wired up (the pre-M16
                  * type-switch would have NULL-dereferenced here) */
    uint32_t pc_data = soc->gpio.ports[2].data;
    leds[0] = (pc_data >> 0) & 1;  /* red */
    leds[1] = (pc_data >> 1) & 1;  /* yellow */
    leds[2] = (pc_data >> 2) & 1;  /* green */
}

static void *msp_mote_get_interface(sim_mote_t *m, int iface) {
    if (iface == SIM_MOTE_IFACE_CC2420)
        return &MOTE_IMPL(m)->plat.msp.cc2420;
    return NULL;
}
static void *arm_mote_get_interface(sim_mote_t *m, int iface) {
    if (iface == SIM_MOTE_IFACE_ARM_CPU)
        return &MOTE_IMPL(m)->plat.arm.cpu;
    return NULL;
}
/* (M17 frame-level RX adapters live in the native/JS modules —
 * M19/M20.) */

static void destroy_node(int idx) {
    sim_mote_t *m = &mote_store[idx];
    m->ops->destroy(m);
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
    sim_cancel_mote_events(&sim_rt, idx);
    sim_runtime_bump_mote_generation(&sim_rt, idx);
    sim_radio_bus_reset_node(&radio_bus, idx);
    emu_rx_end_ns[idx] = 0;
    sim_radio_bus_asm_reset(&tx_asm[idx]);

    /* Destroy and reinitialize.  init_node bumps generation again — fine;
     * what matters is that any events queued between this point and the
     * old slot's last activity are guaranteed to miss the new generation. */
    destroy_node(idx);
    return init_node(idx, fw, node_id);
}

/* --- Simulation step for one node ---
 *
 * Used by the main time-stepping loop and threaded-mode driver to
 * advance a node's CPU. Do NOT call this from chip-driver code or the
 * per-byte RX delivery path — that's a code smell indicating the chip
 * driver is missing a host->schedule_ns() call. The simulator is
 * event-driven: chip drivers must put state-change side effects on
 * sim_eq so the main loop steps the receiver CPU forward naturally.
 * See docs/porting-a-device.md §8 ("Synchronous side effects in
 * chip-driver byte handlers"). */

static void step_node_until(int idx, int64_t target) {
    sim_mote_t *m = &mote_store[idx];
    m->ops->step_until(m, target);
}

/* (MSP430 execute tick lives in src/motes/msp430_elf_mote.c — M21,
 * exported as msp430_elf_mote_tick.) */

/* (ARM execute tick lives in src/motes/arm_elf_mote.c — M22,
 * exported as arm_elf_mote_tick.) */

/* --- Event-driven scheduling helpers (COOJA model) --- */

/* (tick_one_native / native_next_wakeup_after_tick /
 * native_has_pending_work live in src/motes/native_cooja_mote.c —
 * M20.  native_had_tx[] became node->native_had_tx.) */

/* GDB stub state (file scope so the kernel-pump event dispatcher can
 * poll stubs from dispatch_mote_wakeup — M10).  gdb_node[i] is the TCP
 * port to bind for node i, or 0 = no stub. */
static int gdb_node[MAX_NODES];
static gdb_stub_t gdb_stubs[MAX_NODES];

/* Schedule an emulated node's next wakeup */
/* Body moved to sim_radio_bus_wake_mote (M27); thin forwarder keeps the
 * msp serial-injection caller unchanged. */
static void schedule_emulated_wakeup(sim_runtime_t *sim, int idx) {
    sim_radio_bus_wake_mote(&radio_bus, sim, idx);
}

/* (node_next_wakeup_ns removed in M12 — dead since the M10 kernel pump;
 * the per-arch lead computation lives on in the sched_hint_ns ops.) */

/* ============================================================
 * M12 mote execution-op adapters (MSP430/ARM — native/JS moved to
 * src/motes/ in M19/M20).
 *
 * Bodies are verbatim moves of the former dispatch_mote_wakeup /
 * threaded_step_node / schedule_emulated_wakeup type-switch arms.
 * MOTE_IDX recovers the slot index — the wrapped helpers
 * (tick_one_*, emu_rx_queue_drain, gdb_stubs) are still indexed by
 * slot; they follow their dependencies in Phase 5/6 (§3.17).
 * ============================================================ */

/* --- step_until: mote-unit stepping (cycles emulated, ns native/JS) --- */

static void msp_mote_step_until(sim_mote_t *m, int64_t target) {
    msp430_step_until(&MOTE_IMPL(m)->plat.msp.cpu, target);
}
static void arm_mote_step_until(sim_mote_t *m, int64_t target) {
    arm_step_until(&MOTE_IMPL(m)->plat.arm.cpu, target);
}

/* --- execute: one Cooja-style execute slice at global time now_ns.
 * Returns the absolute next-wakeup time; the caller schedules it. --- */

static int64_t msp_mote_execute(sim_mote_t *m, int64_t now_ns) {
    int idx = MOTE_IDX(m);
    /* MSP430 RF delivery is frame-assembled on the sender side and then
     * delivered from the complete-frame path. Consuming rf_pending here
     * can split an in-flight frame and corrupt the receiver-side buffer
     * state. */
    sim_radio_bus_set_executing(&radio_bus, idx);
    int64_t returned_us = msp430_elf_mote_tick(&nodes[idx], now_ns);
    sim_radio_bus_set_executing(&radio_bus, -1);
    if (emu_rx_queue[idx].count > 0)
        emu_rx_queue_drain(idx);
    /* Match Cooja's MspMote.execute(t, 1): this slice schedules the
     * mote's next normal wakeup itself. */
    return now_ns + (returned_us + 1) * 1000LL;
}

static int64_t arm_mote_execute(sim_mote_t *m, int64_t now_ns) {
    int idx = MOTE_IDX(m);
    /* Emulated ARM: same Cooja-style tick as MSP430 so peripheral events
     * are anchored to the scheduler's event time and the CPU accumulates
     * cycle time with the MSPSim stepMicros accuracy bound. */

    /* GDB stub: poll for incoming commands; if the CPU is halted at a
     * breakpoint, skip the tick and return a +1µs wakeup so we keep
     * checking the stub.  (The caller still runs post-tick RF
     * distribution, matching the old goto arm_tick_done flow.) */
    if (gdb_node[idx] != 0) {
        gdb_stub_poll(&gdb_stubs[idx]);
        if (gdb_stubs[idx].halted) {
            nodes[idx].plat.arm.cpu.stopping = false;
            return now_ns + 1000LL;
        }
    }

    sim_radio_bus_set_executing(&radio_bus, idx);
    int64_t returned_us = arm_elf_mote_tick(&nodes[idx], now_ns);
    sim_radio_bus_set_executing(&radio_bus, -1);

    /* GDB stub: a breakpoint may have fired during the tick.  Clear the
     * cpu->stopping flag set by gdb_stub_check_breakpoint so subsequent
     * ticks (after `continue`) can run again. */
    if (gdb_node[idx] != 0) {
        nodes[idx].plat.arm.cpu.stopping = false;
        if (gdb_stubs[idx].halted)
            return now_ns + 1000LL;  /* poll the stub again soon */
    }

    /* Drain any frames queued for this node (mirrors the MSP430 path).
     * For nRF52840, the pending_rx buffer inside the radio model handles
     * delivery when the radio was not in RX state at delivery time; this
     * drain covers the emu_rx_queue path which fires when direct
     * delivery was deferred to a later tick. */
    if (emu_rx_queue[idx].count > 0)
        emu_rx_queue_drain(idx);

    /* Match MspMote.execute(t, 1): next normal wakeup from the
     * step_micros lead hint. */
    return now_ns + (returned_us + 1) * 1000LL;
}

/* (Native execute moved to native_cooja_mote.c — M20.  Its handoff to
 * the dispatcher's post-tick RF distribution is node->exec_had_tx,
 * ex the native_exec_had_tx global.) */

/* --- advance_to_time: out-of-slice catch-up to global sim time
 * (threaded stepping path) --- */

static void msp_mote_advance_to_time(sim_mote_t *m, int64_t sim_ns) {
    msp430_cpu_t *cpu = &MOTE_IMPL(m)->plat.msp.cpu;
    int64_t delta_ns = sim_ns - cpu->sim_time_ns;
    if (delta_ns > 0)
        msp430_step_until(cpu, (int64_t)cpu->cycles +
                          msp430_ns_to_cycles(delta_ns, cpu->cpu_freq_hz));
}
static void arm_mote_advance_to_time(sim_mote_t *m, int64_t sim_ns) {
    arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    int64_t delta_ns = sim_ns - cpu->sim_time_ns;
    if (delta_ns > 0)
        arm_step_until(cpu, cpu->cycles +
                       arm_ns_to_cycles(delta_ns, cpu->cpu_freq_hz));
}

/* --- sched_hint_ns: next-cpu-event wakeup lead from kernel time base_ns
 * (emulated motes only; see schedule_emulated_wakeup) --- */

static int64_t msp_mote_sched_hint_ns(const sim_mote_t *m, int64_t base_ns) {
    const msp430_cpu_t *cpu = &MOTE_IMPL(m)->plat.msp.cpu;
    if ((cpu->interrupts_enabled &&
         cpu->serviced_interrupt == -1 &&
         cpu->interrupt_max >= 0) ||
        cpu->next_event_cycle <= cpu->cycles) {
        return base_ns;
    }
    return base_ns + msp430_cycles_to_ns(
        cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
}
static int64_t arm_mote_sched_hint_ns(const sim_mote_t *m, int64_t base_ns) {
    const arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    if (cpu->next_event_cycle <= cpu->cycles)
        return base_ns;
    return base_ns + arm_cycles_to_ns(
        cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
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

    sim_mote_t *m = &mote_store[idx];
    m->ops->advance_to_time(m, sim_ns);
}

/* --- Channel synchronization ---
 *
 * Emulated nodes (MSP430 / ARM) push their channel into the medium
 * synchronously through the sim_host_t.radio_set_channel adapter (CC2420
 * FSCTRL writes, CC1200 FREQ writes) and via cc2538_rfcore's
 * channel_callback observer (FREQCTRL writes). Sync_node_channels only
 * needs to handle native (Cooja-mote) nodes, which do not run an emulated
 * chip driver — their `simRadioChannel` pointer is the only source of
 * truth for the current channel selection. */
static void sync_node_channels(void) {
    for (int i = 0; i < num_nodes; i++) {
        if (nodes[i].type != NODE_NATIVE) continue;
        int ch = -1;
        if (nodes[i].plat.native.simRadioChannel)
            ch = *nodes[i].plat.native.simRadioChannel;
        radio_medium_set_channel(&radio_medium, i, ch);
    }
}

/* --- Main entry point --- */

/* Check if path ends with .json */
static int is_json_file(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot && strcmp(dot, ".json") == 0;
}

/* ============================================================
 * Kernel-pump event dispatch (M10).
 *
 * sim_runtime_run_until() owns the pop/peek ordering, the now_ns
 * advance to each event's exact time, and the generation check;
 * everything below is runner-side dispatch policy per event kind.
 * Bodies move behind the mote vtable in Phase 2.
 * ============================================================ */

/* NODE_WAKEUP: tick one mote, then run the post-tick RF distribution
 * (receiver wakeups, queued-frame drains, native assembler delivery) —
 * the Cooja requestImmediateWakeup() + connection-finish equivalents. */
static void dispatch_mote_wakeup(const sim_event_t *ev) {
    int i = ev->node_idx;
    if (i < 0 || i >= num_nodes || !node_active(i))
        return;

    int64_t ev_time = ev->time_ns;

    /* Snapshot ALL nodes' state to detect frame delivery.
     * Must be fresh for each event — ACK chains require
     * detecting frames delivered during the current event. */
    int rx_before[MAX_NODES];
    int insize_before[MAX_NODES];
    for (int r = 0; r < num_nodes; r++) {
        if (nodes[r].type == NODE_NATIVE) {
            rx_before[r] = nodes[r].plat.native.rx_queue.count;
            insize_before[r] = *nodes[r].plat.native.simInSize;
        } else {
            rx_before[r] = 0;
            insize_before[r] = 0;
        }
    }

    /* M12: the per-type tick bodies live in the mote execute() ops.
     * Each op runs one Cooja-style execute slice at ev_time and returns
     * its next-wakeup time; the single schedule_if_earlier below
     * preserves any requestImmediateWakeup already queued during the
     * slice (RF delivery detection, drains). */
    sim_mote_t *m = &mote_store[i];
    nodes[i].exec_had_tx = false;
    int64_t next_ns = m->ops->execute(m, ev_time);
    bool sender_had_tx = nodes[i].exec_had_tx;
    if (next_ns < INT64_MAX)
        sim_schedule_mote_wakeup_if_earlier(&sim_rt, i, next_ns);

    /* RF delivery: if this node TX'd, schedule receivers
     * and set signal strength on ALL in-range neighbors
     * (like COOJA's signalReceptionStart + createConnections).
     * This prevents simultaneous TX via CCA. */
    for (int r = 0; r < num_nodes; r++) {
        if (r == i || !node_active(r)) continue;
        if (nodes[r].type == NODE_NATIVE) {
            bool got_frame =
                nodes[r].plat.native.rx_queue.count > rx_before[r] ||
                *nodes[r].plat.native.simInSize > insize_before[r];
            if (got_frame) {
                *nodes[r].plat.native.simReceiving = 1;
                sim_schedule_mote_wakeup_if_earlier(&sim_rt, r, ev_time);
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
    for (int r = 0; r < num_nodes; r++) {
        if (!node_active(r) || nodes[r].type == NODE_NATIVE) continue;
        emu_rx_queue_drain(r);
    }

    /* Deliver pending bytes to native assemblers */
    for (int r = 0; r < num_nodes; r++) {
        if (!node_active(r) || nodes[r].type != NODE_NATIVE) continue;
        mixed_deliver_rf_bytes(r);
    }

    /* (debug stepping removed — was causing cascade on Node 1
     * via unguarded step_node_until on Node 2) */
}

static void mixed_dispatch_event(void *user, const sim_event_t *ev) {
    (void)user;
    switch (ev->kind) {
    case SIM_EV_TEST_ACTION:
        /* Timed test-action marker (milestone 8.3b): its sole job was
         * to make the outer time-advance land exactly on a GENERATE_MSG
         * instant.  The JS gen-msg check + action drain in the outer
         * loop run with sim_ns == marker time and inject the scripted
         * serial input on the dot. */
        return;
    case SIM_EV_RX_BYTE:
        /* Cooja-style per-byte delivery — same MspMoteTimeEvent pattern
         * for every emulated receiver (M13: type-blind). */
        deliver_rx_byte(ev);
        return;
    case SIM_EV_RADIO_TIMER:
        /* Radio-bus RF timer (M9.5): RX-stall deadline. */
        deliver_radio_timer(ev);
        return;
    case SIM_EV_NODE_WAKEUP:
        dispatch_mote_wakeup(ev);
        return;
    }
}

int run_mixed_multinode_test(int argc, char **argv) {
    /* Phase 1 milestone 1: initialize the sim_runtime container.  Zeros the
     * fields and sets run_state to STOPPED.  The event-queue and
     * radio-medium init still happen at their existing call sites below,
     * just through the runtime fields. */
    sim_runtime_init(&sim_rt);
    /* M23: inject the runner-side MSP430/ARM adapter tables into the
     * kind registry (they move to src/motes when their dependencies do
     * — radio bus Phase 5, GDB service Phase 6). */
    sim_mote_kind_set_ops(SIM_BOARD_KIND_MSP430, &msp_mote_ops);
    sim_mote_kind_set_ops(SIM_BOARD_KIND_ARM, &arm_mote_ops);
    /* M11: pre-bind every mote slot to its node struct with a safe default
     * ops table so the node_* vtable accessors never see a NULL ops, even
     * for slots that haven't been through init_node() yet (the old type
     * switch read zeroed structs in that case). */
    for (int i = 0; i < MAX_NODES; i++) {
        mote_store[i].id = 0;
        mote_store[i].ops = sim_mote_kind_for(SIM_BOARD_KIND_NATIVE)->ops;
        mote_store[i].impl = &nodes[i];
    }
    /* fd/pid fields must be -1, not 0, before any _active()/_launched()
     * check — zeroed static structs would read as live. */
    sim_serial_bridge_init(&serial_bridge);
    sim_external_command_init(&external_cmd);
    sim_rt.radio_bus = &radio_bus;
    /* M9.4: runner host hooks for the bus TX path (per-node ops register
     * in init_node via register_node_radio_ops). */
    {
        static const sim_radio_bus_host_t mixed_bus_host = {
            .user = NULL,
            .node_active = bus_host_node_active,
            .sync_channel = bus_host_sync_channel,
            .on_tx_byte = bus_host_on_tx_byte,
            .on_byte_accepted = bus_host_on_byte_accepted,
            .frame_observed = bus_host_frame_observed,
            .on_rx_frame = bus_host_on_rx_frame,
            .on_ack = bus_host_on_ack,
            .on_rx = bus_host_on_rx,
        };
        sim_radio_bus_set_host(&radio_bus, &mixed_bus_host);
        radio_bus.executing_node = -1;
    }
    /* Milestone 8.3: JS/JSON test engines consume console lines off the
     * observer stream instead of a hardwired call in the UART path. */
    sim_runtime_subscribe(&sim_rt, test_engine_observer, NULL);

    static const char *firmware_paths[MAX_NODES] = { NULL };
    static char firmware_bufs[MAX_NODES][256]; /* storage for config-loaded paths */
    int firmware_count = 0;
    int sim_ms = DEFAULT_SIM_MS;
    int node_count = 0;
    int sim_ms_set = 0;  /* track if -t was given (overrides config) */
    int ui_enabled = 0;
    int ui_port = 8080;
    int gdb_wait = 0;  /* if true, block on first connect before starting sim */
    /* Optional pcap output path (--pcap PATH) */
    const char *pcap_path = NULL;
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
        else if (strcmp(argv[i], "--gdb") == 0 && i + 1 < argc) {
            /* Forms accepted:
             *   --gdb 3333          attach node 0 (1-indexed: node 1) to port 3333
             *   --gdb 1:3333        attach node N=1 to port 3333
             *   --gdb 2:3334        attach node N=2 to port 3334  (can repeat) */
            const char *spec = argv[++i];
            int node = 1;
            int port;
            const char *colon = strchr(spec, ':');
            if (colon) {
                node = atoi(spec);
                port = atoi(colon + 1);
            } else {
                port = atoi(spec);
            }
            if (node >= 1 && node <= MAX_NODES && port > 0)
                gdb_node[node - 1] = port;
            else
                fprintf(stderr, "--gdb: bad spec '%s' (use [node:]port)\n", spec);
        }
        else if (strcmp(argv[i], "--gdb-wait") == 0) {
            gdb_wait = 1;
        }
        else if (strcmp(argv[i], "--pcap") == 0 && i + 1 < argc) {
            pcap_path = argv[++i];
        }
        else if (strncmp(argv[i], "--pcap=", 7) == 0) {
            pcap_path = argv[i] + 7;
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
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            /* Per-node startup delay spread (ms). Each node gets a
             * pseudo-random offset in [0, N) ms. Without this, all
             * nodes execute their boot/timer work on identical
             * simulated-clock instants, causing CSMA collisions that
             * never resolve (csim has zero hardware-level XO drift).
             * 100-1000 ms is a sensible range for RPL/CSMA tests. */
            config.startup_delay_ms = atoi(argv[++i]);
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
        printf("    .js       -> JavaScript application mote\n");
        printf("Example:\n");
        printf("  test_runner mixed-multinode firmware/sky/udp-server.sky firmware/cooja/udp-client.cooja -t 60000\n");
        printf("  test_runner mixed-multinode configs/rpl-udp-native.json -v\n");
        return 1;
    }

    int64_t total_ns = (int64_t)sim_ms * MS_TO_NS;

    printf("=== Mixed-Platform Multi-Node Test ===\n");
    for (int i = 0; i < firmware_count; i++) {
        node_type_t t =
            sim_mote_kind_for(sim_board_for_path(firmware_paths[i])->kind)
                ->node_type;
        printf("Firmware[%d]: %s (%s)\n", i, firmware_paths[i],
               t == NODE_MSP430 ? "MSP430" : t == NODE_ARM ? "ARM" : "NATIVE");
    }
    printf("Nodes: %d, Simulated time: %d ms (%lld ns), Threads: %d%s\n",
           node_count, sim_ms, (long long)total_ns,
           num_threads > 0 ? num_threads : 1,
           num_threads > 0 ? "" : " (sequential)");
    printf("Time step: %lld ns\n\n", (long long)TIME_STEP_NS);

    num_nodes = node_count;
    /* Threaded mode advances motes itself, so the bus's RX delivery
     * must not also schedule wakeups (M26). */
    radio_bus.defer_wakeups = (num_threads != 0);
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

    /* GDB stubs: bind one TCP listener per --gdb-tagged node, attach the
     * arch vtable, and optionally block until a client connects. */
    bool any_gdb = false;
    for (int i = 0; i < node_count; i++) {
        if (gdb_node[i] == 0) continue;
        /* M16: ask the mote for its ARM CPU instead of testing type. */
        arm_cpu_t *gdb_cpu = (arm_cpu_t *)mote_store[i].ops->get_interface(
            &mote_store[i], SIM_MOTE_IFACE_ARM_CPU);
        if (!gdb_cpu) {
            fprintf(stderr, "  Node %d: --gdb only supports ARM nodes for now (skipped)\n",
                    nodes[i].id);
            gdb_node[i] = 0;
            continue;
        }
        if (gdb_stub_init(&gdb_stubs[i], gdb_node[i]) != 0) {
            fprintf(stderr, "  Node %d: failed to bind GDB stub on port %d\n",
                    nodes[i].id, gdb_node[i]);
            gdb_node[i] = 0;
            continue;
        }
        gdb_stub_attach(&gdb_stubs[i], gdb_cpu, &arm_gdb_ops);
        gdb_cpu->gdb_stub = &gdb_stubs[i];
        any_gdb = true;
        printf("  Node %d: GDB stub on port %d (connect with: target remote :%d)\n",
               nodes[i].id, gdb_node[i], gdb_node[i]);
    }
    if (any_gdb && gdb_wait) {
        for (int i = 0; i < node_count; i++) {
            if (gdb_node[i] == 0) continue;
            printf("  Node %d: waiting for GDB on port %d ...\n",
                   nodes[i].id, gdb_node[i]);
            if (gdb_stub_wait_for_client(&gdb_stubs[i]) != 0)
                fprintf(stderr, "  Node %d: GDB wait failed\n", nodes[i].id);
        }
    }

    /* PCAP capture: open the file and arm the TX hook in the frame
     * delivery path.  All frame transmissions will be captured at the
     * sender's on-air timestamp until the writer is closed at end. */
    if (pcap_path) {
        if (pcap_writer_open(&pcap_writer, pcap_path,
                             PCAP_LINKTYPE_IEEE802_15_4_WITHFCS) == 0) {
            printf("  PCAP: writing 802.15.4 capture to %s\n", pcap_path);
        } else {
            fprintf(stderr, "  PCAP: failed to open %s for writing\n", pcap_path);
            pcap_path = NULL;
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

    /* Initial-state setup: register native motes' radio slot 0 with the
     * appropriate spectrum (chip-emulated nodes do this through their
     * own driver init -> sim_host_t.radio_set_channel callback). After
     * this point, native channels are pulled inline at every byte/frame
     * delivery site by sync_native_node_channel(). */
    sync_node_channels();

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
                /* Milestone 8.3b: pin each GENERATE_MSG firing time on
                 * the kernel event queue so the sequential loop's time
                 * advance lands exactly on it — scripted serial input
                 * is injected at the scripted instant, not the next
                 * iteration boundary. */
                for (int g = 0; g < js_engine.gen_msg_count; g++)
                    sim_eq_schedule_test_action(&sim_eq,
                        js_engine.gen_msgs[g].at_us * 1000LL);
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
    /* Milestone 7: timeline becomes the first observer subscriber.  All
     * frame-level / LED / interference events now flow through
     * sim_runtime_emit() instead of direct tl_*_event() calls. */
    sim_runtime_subscribe(&sim_rt, timeline_observer_cb, &timeline);
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
            /* The bridge taps the bridged node's UART bytes off the
             * kernel observer stream (SIM_OBS_MOTE_UART_BYTE emitted in
             * mixed_uart_callback) — no per-node callback swap needed. */
            if (sim_serial_bridge_start(&serial_bridge, &sim_rt,
                                        ss_node_idx,
                                        config.serial_socket_port,
                                        ss_inject_into_node, NULL) < 0) {
                fprintf(stderr, "serial_socket: failed to create listener\n");
            } else if (config.serial_socket_command[0]) {
                /* COOJA.testlog tee + child launch live in the
                 * external-command service (milestone 8.2). */
                sim_external_command_start(&external_cmd, &sim_rt,
                                           config.serial_socket_command);
            }
        }
    }

    /* Apply per-node startup delay to desynchronize timers.
     * Each node gets a random start_ns offset. Before its start time,
     * the node is not stepped and does not receive RF. */
    if (config.startup_delay_ms > 0) {
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
             * — just start it later.
             *
             * For ARM nodes the shift above already moved sim_time_ns
             * forward by delay_ns, so node_sim_time_ns(i) already reflects
             * the post-delay timestamp; adding delay_ns again would
             * double-count it (Node 1 reported "start at 109 ms" for a
             * 54 ms delay because of this). */
            int64_t base_ns = node_sim_time_ns(i);
            node_start_ns[i] = (nodes[i].type == NODE_ARM)
                ? base_ns
                : base_ns + delay_ns;
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
                cc2538_rfcore_set_tx_callback(
                    &arm_platform_cc2538(&nodes[i].plat.arm)->rfcore,
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
                nodes[i].plat.msp.cpu.pc_trace_data = &nodes[i];
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
    int64_t ui_interval_ns = 100LL * MS_TO_NS;  /* 100ms sim time between UI updates */
    int64_t next_ui_ns = sim_ns + ui_interval_ns;

    /* Initialize event queue for Cooja-model sequential stepping.
     * File-scope so emu_deliver_bytes() can schedule receivers. */
    sim_eq_init(&sim_eq);
    if (num_threads == 0) {
        for (int i = 0; i < node_count; i++) {
            if (node_start_ns[i] >= INT64_MAX) continue;  /* removed node */
            sim_schedule_mote_wakeup(&sim_rt, i, node_start_ns[i]);
        }
    }



    /* Phase timing accumulators (ms) */
    double time_distribute = 0, time_deliver = 0, time_step = 0;
    double time_flush = 0, time_channel_sync = 0, time_test_ui = 0;

    double t_start = get_time_ms();

    int ss_has_command = sim_external_command_launched(&external_cmd);
    while (sim_ns < end_ns || ui_server ||
           (sim_serial_bridge_active(&serial_bridge) && ss_has_command)) {
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
                : sim_serial_bridge_active(&serial_bridge) ? sim_ns + TIME_STEP_NS
                : end_ns;
            if (next_event < max_ns) max_ns = next_event;
            if (max_ns <= sim_ns) max_ns = sim_ns + 1000;  /* min 1µs advance */
            sim_ns = max_ns;
        }
        sim_rt.now_ns = sim_ns;

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
                    /* Find node index by ID and send data via serial input
                     * (M14: one platform-dispatched path; natives get the
                     * Cooja append + immediate-wakeup contract). */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            inject_serial(i, (const uint8_t *)act->data,
                                          (int)strlen(act->data));
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
                        inject_serial(i, (const uint8_t *)act->data,
                                      (int)strlen(act->data));
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
                            /* Re-seed the mote clock to current time so
                             * stepping resumes correctly (M15 op). */
                            mote_store[i].ops->reset_time(&mote_store[i],
                                                          sim_ns);
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
                    /* M14: same platform-dispatched injection as the JSON
                     * action path.  Natives now use the Cooja append +
                     * immediate-wakeup contract instead of overwrite +
                     * synchronous step — the wakeup fires in the next pump
                     * at the mote's own sim time. */
                    for (int i = 0; i < node_count; i++) {
                        if (nodes[i].id == act->node) {
                            inject_serial(i, (const uint8_t *)act->data,
                                          (int)strlen(act->data));
                            if (verbose)
                                printf("  JS ACTION: send node %d \"%s\"\n",
                                       act->node, act->data);
                            break;
                        }
                    }
                } else if (act->type == TEST_ACTION_SEND_ALL) {
                    for (int i = 0; i < node_count; i++) {
                        if (node_start_ns[i] > sim_ns) continue;
                        inject_serial(i, (const uint8_t *)act->data,
                                      (int)strlen(act->data));
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
                                num_nodes = node_count;
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
                        mote_store[i].ops->reset_time(&mote_store[i], sim_ns);
                        node_start_ns[i] = sim_ns;
                        /* Schedule the rebooted node in the event queue
                         * so it gets ticked immediately */
                        sim_schedule_mote_wakeup_if_earlier(&sim_rt, i, sim_ns);
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

        /* Serial bridge service tick + external-command child check */
        if (sim_serial_bridge_active(&serial_bridge) ||
            sim_external_command_running(&external_cmd)) {
            sim_serial_bridge_poll(&serial_bridge);
            sim_external_command_poll(&external_cmd);
            if (sim_external_command_exited(&external_cmd))
                break;
        }

        /* Native channels are now sampled inline at every byte/frame
         * delivery site (sync_native_node_channel), so the periodic lazy
         * sync at tick boundaries is no longer needed. The dirty flag is
         * still cleared so the bookkeeping stays consistent for any
         * future readers. */
        double t_phase;
        if (channels_dirty)
            channels_dirty = false;

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

            /* Kernel event pump (milestone 10): pops events with
             * time <= sim_ns in (time, seq) order, advances now_ns to
             * each event's exact time (Cooja getSimulationTime()
             * semantics), drops stale-generation events, and dispatches
             * through mixed_dispatch_event.  Same-time-spin diagnostics
             * (CSIM_TRACE_EVENT_SPIN) live in the kernel now. */
            sim_runtime_run_until(&sim_rt, sim_ns, mixed_dispatch_event,
                                  NULL);
            if (sim_runtime_stop_requested(&sim_rt))
                break;

            time_step += get_time_ms() - t_phase;
        }

        /* Update per-node radio/LED state for timeline (M16: the ops'
         * NULL-ness encodes which mote kinds are polled — CC2538 pushes
         * radio state via async callback, natives/JS have neither). */
        if (ui_server) {
            for (int i = 0; i < node_count; i++) {
                if (!node_active(i)) continue;
                update_radio_state(i);
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
        if (sim_serial_bridge_active(&serial_bridge) && !ui_server) {
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
        sim_rt.now_ns = 0;
        sim_eq_init(&sim_eq);
        stat_rf_frames = 0;
        radio_bus.stats.rx_direct = 0;
        radio_bus.stats.rx_queued = 0;
        radio_bus.stats.rx_drained = 0;
        radio_bus.stats.rx_dropped = 0;
        radio_bus.stats.rx_collided = 0;
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
        memset(radio_bus.tx_busy_until_ns, 0, sizeof(radio_bus.tx_busy_until_ns));
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
    if (pcap_writer_is_open(&pcap_writer)) {
        printf("  PCAP: wrote %lld frames to %s\n",
               (long long)pcap_writer.packet_count, pcap_path);
        pcap_writer_close(&pcap_writer);
    }
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
           radio_bus.stats.rx_direct, radio_bus.stats.rx_queued, radio_bus.stats.rx_drained,
           radio_bus.stats.rx_dropped, radio_bus.stats.rx_collided);
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
        uint32_t cur_pc = 0;
        if (nodes[i].type == NODE_MSP430)
            cur_pc = nodes[i].plat.msp.cpu.reg[MSP430_PC];
        printf("  Node %d [%s]: %lld cycles, %lld instructions, PC=0x%04x\n",
               nodes[i].id, node_type_str(i),
               (long long)node_cycles(i), (long long)node_instructions(i),
               cur_pc);
        total_node_cycles += node_cycles(i);
        total_node_instructions += node_instructions(i);
        destroy_node(i);
    }

    /* Cleanup thread pool */
    if (num_threads > 0)
        sim_thread_pool_destroy(&thread_pool);

    /* Serial socket: use child exit status as test result */
    if (sim_external_command_exited(&external_cmd)) {
        int st = sim_external_command_status(&external_cmd);
        printf("\n--- Serial Socket Test Results ---\n");
        if (st == 0) {
            printf("  TEST PASSED (external command exited 0)\n");
        } else {
            printf("  TEST FAILED (external command exited %d)\n", st);
            test_exit_code = st;
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
