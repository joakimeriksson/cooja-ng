/*
 * Multi-node simulation test — connects two Sky nodes via CC2420 radio
 *
 * Uses time-synchronized stepping: all nodes advance to the same simulation
 * time in small increments, with RF bytes buffered and delivered between
 * time steps to ensure proper radio state synchronization.
 */
#include "msp430_platform.h"
#include "msp430_elf.h"
#include "cc2420.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define MAX_NODES 2
#define TIME_STEP_NS      1000000LL  /* 1ms in nanoseconds */
#define DEFAULT_SIM_MS    20000      /* 20 seconds of simulated time */
#define MS_TO_NS          1000000LL

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

typedef struct {
    msp430_platform_t plat;
    int id;
    char line_buf[256];
    int line_pos;
} sim_node_t;

/* RF byte buffer for deferred delivery */
#define RF_BUF_SIZE 4096
typedef struct {
    uint8_t bytes[RF_BUF_SIZE];
    int count;
} rf_buffer_t;

static sim_node_t nodes[MAX_NODES];
static int num_nodes = 0;
static int verbose = 1;
static rf_buffer_t rf_pending[MAX_NODES];  /* per-receiver buffers */

/* RF callback: buffer transmitted bytes for delivery during receiver's time step */
static int rf_byte_count = 0;
static void rf_tx_handler(void *user_data, uint8_t byte) {
    sim_node_t *sender = (sim_node_t *)user_data;
    rf_byte_count++;
    for (int i = 0; i < num_nodes; i++) {
        if (&nodes[i] != sender) {
            rf_buffer_t *buf = &rf_pending[i];
            if (buf->count < RF_BUF_SIZE) {
                buf->bytes[buf->count++] = byte;
            }
        }
    }
}

/* Deliver buffered RF bytes to a node's CC2420 */
static void deliver_rf_bytes(int node_idx) {
    rf_buffer_t *buf = &rf_pending[node_idx];
    if (buf->count == 0) return;
    for (int j = 0; j < buf->count; j++) {
        cc2420_receive_byte(&nodes[node_idx].plat.cc2420, buf->bytes[j]);
    }
    buf->count = 0;
}

/* UART callback: print output with node prefix */
static int uart_byte_count = 0;
static void uart_callback(void *user_data, uint8_t byte) {
    sim_node_t *node = (sim_node_t *)user_data;
    uart_byte_count++;

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        if (verbose) {
            printf("  [Node %d] %s\n", node->id, node->line_buf);
        }
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore CR */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
    }
}

/* Patch ds2411_init() to RET immediately */
static void patch_ds2411_init(sim_node_t *node, const char *firmware_path) {
    uint32_t ds2411_init_addr = msp430_elf_find_symbol(firmware_path, "ds2411_init");
    if (ds2411_init_addr != 0) {
        node->plat.cpu.memory[ds2411_init_addr]     = 0x30;
        node->plat.cpu.memory[ds2411_init_addr + 1] = 0x41;
        printf("  Patched ds2411_init at 0x%04x to RET\n", ds2411_init_addr);
    }
}

/* Set ds2411_id in firmware memory */
static int patch_node_id(sim_node_t *node, const char *firmware_path, int node_id) {
    uint32_t ds2411_addr = msp430_elf_find_symbol(firmware_path, "ds2411_id");
    if (ds2411_addr == 0) {
        fprintf(stderr, "  Warning: ds2411_id symbol not found\n");
        return -1;
    }
    uint8_t *id = node->plat.cpu.memory + ds2411_addr;
    id[0] = 0x00; id[1] = 0x12; id[2] = 0x74; id[3] = (uint8_t)node_id;
    id[4] = 0x00; id[5] = 0x00; id[6] = 0x00; id[7] = (uint8_t)node_id;
    printf("  Patched ds2411_id at 0x%04x for node %d: ",
           ds2411_addr, node_id);
    for (int i = 0; i < 8; i++) printf("%02x%s", id[i], i < 7 ? ":" : "\n");
    return 0;
}

static int init_node(int idx, const char *firmware_path, int node_id) {
    sim_node_t *node = &nodes[idx];
    memset(node, 0, sizeof(*node));
    node->id = node_id;
    memset(&rf_pending[idx], 0, sizeof(rf_pending[idx]));

    const msp430_platform_config_t *pcfg = msp430_platform_find("sky");
    if (!pcfg) { fprintf(stderr, "Platform 'sky' not found\n"); return -1; }

    msp430_platform_init(&node->plat, pcfg);
    if (msp430_load_elf(&node->plat.cpu, firmware_path) != 0) {
        fprintf(stderr, "Cannot load firmware: %s\n", firmware_path);
        msp430_platform_destroy(&node->plat);
        return -1;
    }

    patch_ds2411_init(node, firmware_path);
    msp430_platform_set_console(&node->plat, uart_callback, node);
    cc2420_set_rf_listener(&node->plat.cc2420, rf_tx_handler, node);
    node->plat.cc2420.node_id = node_id;
    msp430_cpu_reset(&node->plat.cpu);

    /* Run past crt0 so .bss clearing is done, then patch ds2411_id */
    uint32_t main_addr = msp430_elf_find_symbol(firmware_path, "main");
    if (main_addr != 0) {
        for (int s = 0; s < 200000; s++) {
            msp430_step(&node->plat.cpu, 1);
            if ((node->plat.cpu.reg[0] & 0xFFFF) == (main_addr & 0xFFFF))
                break;
        }
        printf("  crt0 done: PC=0x%04x (main=0x%04x) after %lld cycles\n",
               node->plat.cpu.reg[0], main_addr, (long long)node->plat.cpu.cycles);
    } else {
        msp430_step(&node->plat.cpu, 50000);
    }
    patch_node_id(node, firmware_path, node_id);
    return 0;
}

int run_multinode_test(int argc, char **argv) {
    const char *firmware_paths[MAX_NODES] = { NULL };
    int firmware_count = 0;
    int sim_ms = DEFAULT_SIM_MS;
    int node_count = 2;

    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) verbose = 1;
        else if (strcmp(argv[i], "-q") == 0) verbose = 0;
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            node_count = atoi(argv[++i]);
            if (node_count > MAX_NODES) node_count = MAX_NODES;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            sim_ms = atoi(argv[++i]);
        } else if (argv[i][0] != '-') {
            if (firmware_count < MAX_NODES)
                firmware_paths[firmware_count++] = argv[i];
        }
    }

    if (firmware_count == 0) {
        firmware_paths[0] = "../firmware/sky/nullnet-broadcast.sky";
        firmware_count = 1;
    }

    int64_t total_ns = (int64_t)sim_ms * MS_TO_NS;

    printf("=== Multi-Node CC2420 Radio Test ===\n");
    for (int i = 0; i < firmware_count; i++)
        printf("Firmware[%d]: %s\n", i, firmware_paths[i]);
    printf("Nodes: %d, Simulated time: %d ms (%lld ns)\n",
           node_count, sim_ms, (long long)total_ns);
    printf("Time step: %lld ns\n\n", (long long)TIME_STEP_NS);

    num_nodes = node_count;
    for (int i = 0; i < node_count; i++) {
        const char *fw = firmware_paths[i < firmware_count ? i : firmware_count - 1];
        printf("Initializing node %d (%s)...\n", i + 1, fw);
        if (init_node(i, fw, i + 1) != 0) {
            fprintf(stderr, "Failed to initialize node %d\n", i + 1);
            return 1;
        }
    }

    printf("\n--- Simulation running ---\n\n");

    /* Start sim_time_ns at max of all nodes' current sim_time_ns */
    int64_t sim_ns = 0;
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].plat.cpu.sim_time_ns > sim_ns)
            sim_ns = nodes[i].plat.cpu.sim_time_ns;
    }
    printf("  Initial sim_time: %lld ns (%lld ms)\n",
           (long long)sim_ns, (long long)(sim_ns / MS_TO_NS));

    int64_t end_ns = sim_ns + total_ns;
    int64_t progress_interval = total_ns / 10;
    int64_t next_progress = sim_ns + progress_interval;

    double t_start = get_time_ms();

    while (sim_ns < end_ns) {
        sim_ns += TIME_STEP_NS;

        for (int i = 0; i < node_count; i++) {
            deliver_rf_bytes(i);
            /* Convert ns target to cycle target for this node */
            msp430_cpu_t *cpu = &nodes[i].plat.cpu;
            int64_t delta_ns = sim_ns - cpu->sim_time_ns;
            if (delta_ns > 0) {
                int64_t target_cycle = cpu->cycles +
                    msp430_ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
                msp430_step_until(cpu, target_cycle);
            }
        }

        if (sim_ns >= next_progress) {
            int pct = (int)((sim_ns - (end_ns - total_ns)) * 100 / total_ns);
            printf("  --- Progress: %d%% (%lld ms) rf_bytes=%d uart_bytes=%d ---\n",
                   pct, (long long)(sim_ns / MS_TO_NS),
                   rf_byte_count, uart_byte_count);
            for (int i = 0; i < node_count; i++) {
                printf("    Node %d: %lld cycles, cc2420_state=%d\n",
                       nodes[i].id, (long long)nodes[i].plat.cpu.cycles,
                       nodes[i].plat.cc2420.state);
            }
            next_progress += progress_interval;
        }
    }

    double t_end = get_time_ms();
    double elapsed_ms = t_end - t_start;

    printf("\n--- Simulation complete ---\n");
    printf("  Total RF bytes: %d\n", rf_byte_count);
    printf("  Total UART bytes: %d\n", uart_byte_count);
    int64_t total_node_cycles = 0;
    int64_t total_instructions = 0;
    for (int i = 0; i < node_count; i++) {
        printf("  Node %d: %lld cycles, %lld instructions, CC2420 state=%d, status=0x%02x\n",
               nodes[i].id, (long long)nodes[i].plat.cpu.cycles,
               (long long)nodes[i].plat.cpu.instructions,
               nodes[i].plat.cc2420.state, nodes[i].plat.cc2420.status);
        printf("    PC=0x%04x SR=0x%04x SP=0x%04x cpu_off=%d gie=%d\n",
               nodes[i].plat.cpu.reg[0], nodes[i].plat.cpu.reg[2],
               nodes[i].plat.cpu.reg[1],
               nodes[i].plat.cpu.cpu_off,
               (nodes[i].plat.cpu.reg[2] & 0x08) != 0);
        total_node_cycles += nodes[i].plat.cpu.cycles;
        total_instructions += nodes[i].plat.cpu.instructions;
        msp430_platform_destroy(&nodes[i].plat);
    }

    printf("\n--- Performance ---\n");
    printf("  Wall-clock time:  %.1f ms (%.2f s)\n", elapsed_ms, elapsed_ms / 1000.0);
    printf("  Simulated time:   %d ms (%.1f s)\n", sim_ms, sim_ms / 1000.0);
    double speedup = sim_ms / elapsed_ms;
    printf("  Speed ratio:      %.1fx real-time (%d nodes)\n", speedup, node_count);
    printf("  Total cycles:     %lld across %d nodes\n",
           (long long)total_node_cycles, node_count);
    printf("  Total instrs:     %lld across %d nodes\n",
           (long long)total_instructions, node_count);
    double mips = total_instructions / (elapsed_ms * 1000.0);
    printf("  Throughput:       %.1f MIPS (total), %.1f MIPS (per-node)\n",
           mips, mips / node_count);

    return 0;
}
