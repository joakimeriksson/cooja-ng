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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

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

static mixed_node_t nodes[MAX_NODES];
static int num_nodes = 0;
static int verbose = 1;
static rf_buffer_t rf_pending[MAX_NODES];
static int rf_byte_count = 0;
static int uart_byte_count = 0;
static radio_medium_t radio_medium;
static int64_t current_sim_ns = 0;

/* Statistics counters */
static int stat_rx_frames_queued = 0;
static int stat_rx_frames_collided = 0;
static int stat_rx_frames_queue_full = 0;
static bool channels_dirty = false;

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

/*
 * Byte-level TX handler: called by CC2420 and CC2538 RF when transmitting
 * individual bytes. Also called by native_check_radio_tx() when converting
 * a native frame to byte-stream for emulated receivers.
 */
static void mixed_rf_tx_handler(void *user_data, uint8_t byte) {
    mixed_node_t *sender = (mixed_node_t *)user_data;
    int sender_idx = (int)(sender - nodes);
    rf_byte_count++;
    channels_dirty = true;
    for (int i = 0; i < num_nodes; i++) {
        if (&nodes[i] != sender) {
            /* Native-to-native: skip byte-stream path, handled by frame handler */
            if (sender->type == NODE_NATIVE && nodes[i].type == NODE_NATIVE)
                continue;
            /* Apply radio medium filter */
            if (!radio_medium_filter_byte(&radio_medium, sender_idx, i, byte))
                continue;
            if (nodes[i].type == NODE_NATIVE) {
                /* Feed byte into native node's reassembler (emulated->native) */
                native_rx_assembler_feed(&nodes[i].plat.native, byte);
            } else {
                /* Buffer byte for emulated node delivery */
                rf_buffer_t *buf = &rf_pending[i];
                if (buf->count < RF_BUF_SIZE)
                    buf->bytes[buf->count++] = byte;
            }
        }
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

    channels_dirty = true;  /* TX happened, channels may have changed */

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
            if (nodes[i].type != NODE_NATIVE) continue;
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

static void mixed_deliver_rf_bytes(int idx) {
    rf_buffer_t *buf = &rf_pending[idx];
    if (buf->count == 0) return;
    for (int j = 0; j < buf->count; j++) {
        if (nodes[idx].type == NODE_MSP430)
            cc2420_receive_byte(&nodes[idx].plat.msp.cc2420, buf->bytes[j]);
        else if (nodes[idx].type == NODE_ARM)
            cc2538_rfcore_receive_byte(&nodes[idx].plat.arm.rfcore, buf->bytes[j]);
        /* NODE_NATIVE bytes go through the assembler in mixed_rf_tx_handler */
    }
    buf->count = 0;
}

/* --- UART callback --- */

static void mixed_uart_callback(void *user_data, uint8_t byte) {
    mixed_node_t *node = (mixed_node_t *)user_data;
    uart_byte_count++;

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        if (verbose)
            printf("  [Node %d/%s] %s\n", node->id, node_type_str((int)(node - nodes)), node->line_buf);
        sim_test_check_line(node->id, node->line_buf, current_sim_ns);
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
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

    /* Seed RFRND uniquely per node */
    plat->rfcore.rfrnd_state = 0xDEADBEEF + (uint32_t)node_id;

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
    sim_config_t config;
    int config_loaded = 0;

    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) verbose = 1;
        else if (strcmp(argv[i], "-q") == 0) verbose = 0;
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            node_count = atoi(argv[++i]);
            if (node_count > MAX_NODES) node_count = MAX_NODES;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            sim_ms = atoi(argv[++i]);
            sim_ms_set = 1;
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
        printf("Usage: test_runner mixed-multinode <firmware1> [firmware2...] [-t ms] [-n nodes] [-v] [-q]\n");
        printf("       test_runner mixed-multinode <config.json> [-t ms] [-v] [-q]\n");
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
    printf("Nodes: %d, Simulated time: %d ms (%lld ns)\n",
           node_count, sim_ms, (long long)total_ns);
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

    double t_start = get_time_ms();

    while (sim_ns < end_ns) {
        sim_ns += TIME_STEP_NS;
        current_sim_ns = sim_ns;

        /* Lazy channel sync: only when TX activity has occurred */
        if (channels_dirty && radio_medium.type != RADIO_MEDIUM_NONE) {
            sync_node_channels();
            channels_dirty = false;
        }

        for (int i = 0; i < node_count; i++) {
            /* Deliver pending RF bytes to emulated nodes */
            mixed_deliver_rf_bytes(i);

            if (nodes[i].type == NODE_NATIVE) {
                /* Dequeue one non-collided frame for delivery */
                native_dequeue_rx_frame(&nodes[i].plat.native);

                /* Skip idle native nodes — just advance their time */
                if (!native_has_pending_work(&nodes[i].plat.native, sim_ns)) {
                    nodes[i].plat.native.sim_time_ns = sim_ns;
                    continue;
                }
                step_node_until(i, sim_ns);
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
    printf("  RX frames queued: %d\n", stat_rx_frames_queued);
    printf("  RX frames collided: %d\n", stat_rx_frames_collided);
    printf("  RX frames queue full: %d\n", stat_rx_frames_queue_full);

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

    printf("\n--- Performance ---\n");
    printf("  Wall-clock time:  %.1f ms (%.2f s)\n", elapsed_ms, elapsed_ms / 1000.0);
    printf("  Simulated time:   %d ms (%.1f s)\n", sim_ms, sim_ms / 1000.0);
    double speedup = sim_ms / elapsed_ms;
    printf("  Speed ratio:      %.1fx real-time (%d nodes)\n", speedup, node_count);
    printf("  Total cycles:     %lld across %d nodes\n",
           (long long)total_node_cycles, node_count);
    printf("  Total instrs:     %lld across %d nodes\n",
           (long long)total_node_instructions, node_count);
    double mips = total_node_instructions / (elapsed_ms * 1000.0);
    printf("  Throughput:       %.1f MIPS (total), %.1f MIPS (per-node)\n",
           mips, mips / node_count);

    return test_exit_code;
}
