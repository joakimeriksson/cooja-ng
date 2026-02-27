/*
 * ARM multi-node simulation test — CC2538 nodes with RF Core
 */
#include "arm_platform.h"
#include "arm_systick.h"
#include "arm_elf.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define ARM_MAX_NODES 2
#define ARM_TIME_STEP_NS   1000000LL  /* 1ms */
#define ARM_DEFAULT_SIM_MS 20000
#define MS_TO_NS           1000000LL

static double arm_get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

typedef struct {
    arm_platform_t plat;
    int id;
    char line_buf[256];
    int line_pos;
} arm_sim_node_t;

#define ARM_RF_BUF_SIZE 4096
typedef struct {
    uint8_t bytes[ARM_RF_BUF_SIZE];
    int count;
} arm_rf_buffer_t;

static arm_sim_node_t arm_nodes[ARM_MAX_NODES];
static int arm_num_nodes = 0;
static int arm_verbose = 1;
static arm_rf_buffer_t arm_rf_pending[ARM_MAX_NODES];
static int arm_rf_byte_count = 0;
static int arm_uart_byte_count = 0;

static void arm_rf_tx_handler(void *user_data, uint8_t byte) {
    arm_sim_node_t *sender = (arm_sim_node_t *)user_data;
    arm_rf_byte_count++;
    for (int i = 0; i < arm_num_nodes; i++) {
        if (&arm_nodes[i] != sender) {
            arm_rf_buffer_t *buf = &arm_rf_pending[i];
            if (buf->count < ARM_RF_BUF_SIZE)
                buf->bytes[buf->count++] = byte;
        }
    }
}

static void arm_deliver_rf_bytes(int node_idx) {
    arm_rf_buffer_t *buf = &arm_rf_pending[node_idx];
    if (buf->count == 0) return;
    for (int j = 0; j < buf->count; j++)
        cc2538_rfcore_receive_byte(&arm_nodes[node_idx].plat.rfcore, buf->bytes[j]);
    buf->count = 0;
}

static void arm_uart_callback(void *user_data, uint8_t byte) {
    arm_sim_node_t *node = (arm_sim_node_t *)user_data;
    arm_uart_byte_count++;

    if (byte == '\n') {
        node->line_buf[node->line_pos] = '\0';
        if (arm_verbose)
            printf("  [Node %d] %s\n", node->id, node->line_buf);
        node->line_pos = 0;
    } else if (byte == '\r') {
        /* ignore */
    } else if (node->line_pos < (int)sizeof(node->line_buf) - 1) {
        node->line_buf[node->line_pos++] = (char)byte;
    }
}

static int arm_init_node(int idx, const char *firmware_path, int node_id) {
    arm_sim_node_t *node = &arm_nodes[idx];
    memset(node, 0, sizeof(*node));
    node->id = node_id;
    memset(&arm_rf_pending[idx], 0, sizeof(arm_rf_pending[idx]));

    const arm_platform_config_t *pcfg = arm_platform_find("cc2538dk");
    if (!pcfg) { fprintf(stderr, "Platform 'cc2538dk' not found\n"); return -1; }

    arm_platform_init(&node->plat, pcfg);
    if (arm_load_elf(&node->plat.cpu, firmware_path) != 0) {
        fprintf(stderr, "Cannot load firmware: %s\n", firmware_path);
        arm_platform_destroy(&node->plat);
        return -1;
    }

    arm_platform_set_console(&node->plat, arm_uart_callback, node);
    cc2538_rfcore_set_tx_callback(&node->plat.rfcore, arm_rf_tx_handler, node);
    node->plat.rfcore.node_id = node_id;

    /* Seed RFRND uniquely per node */
    node->plat.rfcore.rfrnd_state = 0xDEADBEEF + (uint32_t)node_id;

    /* Set unique IEEE 64-bit extended address: 00:12:74:XX:00:00:00:XX
     * The info page stores bytes in little-endian order; ieee_addr_cpy_to()
     * reverses them: dst[i] = location[len-1-i].  So we store reversed. */
    uint8_t unique_addr[8] = { (uint8_t)node_id, 0x00, 0x00, 0x00,
                                (uint8_t)node_id, 0x74, 0x12, 0x00 };
    memcpy(node->plat.rfcore.ext_addr, unique_addr, 8);

    /* Reset and run past crt0 to main */
    arm_cpu_reset(&node->plat.cpu);

    uint32_t main_addr = arm_elf_find_symbol(firmware_path, "main") & ~1u;
    if (main_addr) {
        for (int s = 0; s < 500000; s++) {
            arm_step(&node->plat.cpu, 1);
            if ((node->plat.cpu.reg[ARM_PC] & ~1u) == main_addr)
                break;
        }
        printf("  Node %d: ran to main (0x%08x) in %lld cycles\n",
               node_id, main_addr, (long long)node->plat.cpu.cycles);

        /* Patch linkaddr_node_addr with the address as it appears after
         * ieee_addr_cpy_to() reversal: 00:12:74:XX:00:00:00:XX */
        uint32_t la_addr = arm_elf_find_symbol(firmware_path, "linkaddr_node_addr");
        if (la_addr) {
            la_addr &= ~1u;
            uint8_t la_bytes[8] = { 0x00, 0x12, 0x74, (uint8_t)node_id,
                                     0x00, 0x00, 0x00, (uint8_t)node_id };
            for (int b = 0; b < 8; b++)
                arm_write8(&node->plat.cpu, la_addr + (uint32_t)b, la_bytes[b]);
            printf("  Node %d: patched linkaddr_node_addr at 0x%08x\n",
                   node_id, la_addr);
        }
    } else {
        printf("  Node %d: 'main' symbol not found, skipping crt0 run\n", node_id);
    }

    printf("  Node %d initialized: PC=0x%08x SP=0x%08x\n",
           node_id, node->plat.cpu.reg[ARM_PC], node->plat.cpu.reg[ARM_SP]);
    return 0;
}


int run_arm_multinode_test(int argc, char **argv) {
    const char *firmware_paths[ARM_MAX_NODES] = { NULL };
    int firmware_count = 0;
    int sim_ms = ARM_DEFAULT_SIM_MS;
    int node_count = 2;

    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) arm_verbose = 1;
        else if (strcmp(argv[i], "-q") == 0) arm_verbose = 0;
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            node_count = atoi(argv[++i]);
            if (node_count > ARM_MAX_NODES) node_count = ARM_MAX_NODES;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            sim_ms = atoi(argv[++i]);
        } else if (argv[i][0] != '-') {
            if (firmware_count < ARM_MAX_NODES)
                firmware_paths[firmware_count++] = argv[i];
        }
    }

    if (firmware_count == 0) {
        printf("Usage: test_runner arm-multinode <firmware1> [firmware2] [-t ms] [-n nodes]\n");
        return 1;
    }

    int64_t total_ns = (int64_t)sim_ms * MS_TO_NS;

    printf("=== ARM Multi-Node RF Core Test ===\n");
    for (int i = 0; i < firmware_count; i++)
        printf("Firmware[%d]: %s\n", i, firmware_paths[i]);
    printf("Nodes: %d, Simulated time: %d ms\n\n", node_count, sim_ms);

    arm_num_nodes = node_count;
    for (int i = 0; i < node_count; i++) {
        const char *fw = firmware_paths[i < firmware_count ? i : firmware_count - 1];
        printf("Initializing node %d (%s)...\n", i + 1, fw);
        if (arm_init_node(i, fw, i + 1) != 0) {
            fprintf(stderr, "Failed to initialize node %d\n", i + 1);
            return 1;
        }
    }

    printf("\n--- Simulation running ---\n\n");

    int64_t sim_ns = 0;
    for (int i = 0; i < node_count; i++) {
        if (arm_nodes[i].plat.cpu.sim_time_ns > sim_ns)
            sim_ns = arm_nodes[i].plat.cpu.sim_time_ns;
    }

    int64_t end_ns = sim_ns + total_ns;
    int64_t progress_interval = total_ns / 10;
    int64_t next_progress = sim_ns + progress_interval;

    double t_start = arm_get_time_ms();

    while (sim_ns < end_ns) {
        sim_ns += ARM_TIME_STEP_NS;
        for (int i = 0; i < node_count; i++) {
            arm_deliver_rf_bytes(i);
            arm_cpu_t *cpu = &arm_nodes[i].plat.cpu;
            int64_t delta_ns = sim_ns - cpu->sim_time_ns;
            if (delta_ns > 0) {
                int64_t target_cycle = cpu->cycles +
                    arm_ns_to_cycles(delta_ns, cpu->cpu_freq_hz);
                arm_step_until(cpu, target_cycle);
            }
        }
        if (sim_ns >= next_progress) {
            int pct = (int)((sim_ns - (end_ns - total_ns)) * 100 / total_ns);
            printf("  --- Progress: %d%% (%lld ms) ---\n",
                   pct, (long long)(sim_ns / MS_TO_NS));
            next_progress += progress_interval;
        }
    }

    double t_end = arm_get_time_ms();
    double elapsed_ms = t_end - t_start;

    printf("\n--- Simulation complete ---\n");
    printf("  Total RF bytes: %d\n", arm_rf_byte_count);
    printf("  Total UART bytes: %d\n", arm_uart_byte_count);
    printf("  SysTick fires: %d\n", arm_systick_get_fire_count());
    for (int i = 0; i < node_count; i++) {
        printf("  Node %d: %lld cycles, %lld instructions\n",
               arm_nodes[i].id,
               (long long)arm_nodes[i].plat.cpu.cycles,
               (long long)arm_nodes[i].plat.cpu.instructions);
        arm_platform_destroy(&arm_nodes[i].plat);
    }
    printf("  Wall-clock: %.1f ms, Speed: %.1fx real-time\n",
           elapsed_ms, sim_ms / elapsed_ms);

    return 0;
}
