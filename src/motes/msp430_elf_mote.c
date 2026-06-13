/*
 * msp430_elf_mote — boot policy, Cooja-style execute tick, and radio
 * endpoint ops for the MSP430 emulated-ELF mote kind (Phase 4, M21 —
 * docs/design/refactor-plan.md §3.17).
 *
 * Bodies are character-identical moves of the former runner code
 * (test/test_mixed_multinode.c init_msp430_node / tick_one_msp430 /
 * msp_radio_ops), with the enumerated seam substitutions: runner
 * callbacks → env members, nodes[idx] → the node pointer.
 *
 * The msp_mote_ops adapter table (execute/serial_input/...) stays in
 * the runner: it is entangled with emu_rx_queue_drain (frame-delivery
 * policy → Phase 5), ticking_node_idx (shared with runner RF
 * handlers → Phase 5), and the baud-paced serial injection's
 * scheduling.  Per §3.17 those follow their dependencies.
 */
#include "mote_impl.h"

#include "msp430_elf.h"

#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 * Boot policy
 *
 * Every firmware patch below names the firmware symbol it depends on
 * (§9 Phase 4 stop condition).
 * ============================================================ */

int msp430_elf_mote_boot(mixed_node_t *node, int slot,
                         const char *firmware_path, int node_id,
                         const sim_mote_env_t *env) {
    int idx = slot;
    msp430_platform_t *plat = &node->plat.msp;

    /* Platform name comes from the board registry row (Phase 3). */
    const char *plat_name = node->board->name;
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

    msp430_platform_set_console(plat, env->uart_byte, node);
    /* Per-radio TX listener: CC2420 lives in slot 0. The chip stays
     * unaware of which slot it occupies; the harness encodes that in
     * the rf_listener_ctx_t it captures on the chip's side. */
    node->rf_ctx[0].node_idx  = idx;
    node->rf_ctx[0].radio_idx = 0;
    cc2420_set_rf_listener(&plat->cc2420, env->chip_tx_byte, &node->rf_ctx[0]);
    plat->cc2420.node_id = node_id;
    /* CC2420's FSCTRL writes push channel via the sim_host_t vtable
     * onto radio slot 0 for this node. Same adapter as the ARM path. */
    plat->host.radio_user_data  = node;
    plat->host.radio_set_channel = env->radio_set_channel;
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

    /* No firmware patches: Cooja MSPSim runs the firmware unmodified
     * and so should we. The firmware exports uip_ds6_addr_size and
     * uip_ds6_netif_addr_list_offset for tools (Cooja's IPAddress.java)
     * to observe — Cooja never WRITES the addr_list. The firmware does
     * its own IPv6 setup via uip_ds6_addr_add(). */

    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

static int msp_radio_rxfifo_available(void *m) {
    mixed_node_t *node = (mixed_node_t *)m;
    return 128 - node->plat.msp.cc2420.rx_fifo_len;
}
static void msp_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    mixed_node_t *node = (mixed_node_t *)m;
    node->plat.msp.cc2420.rx_rssi = rssi;
    cc2420_receive_byte(&node->plat.msp.cc2420, byte);
}
static bool msp_radio_rx_busy(void *m) {
    mixed_node_t *node = (mixed_node_t *)m;
    cc2420_radio_state_t s = node->plat.msp.cc2420.state;
    return s == CC2420_RX_FRAME || s == CC2420_RX_OVERFLOW;
}
static const mote_radio_ops_t msp_radio_ops = {
    msp_radio_receive_byte, msp_radio_rxfifo_available, msp_radio_rx_busy,
    NULL /* rx_stall */
};

void msp430_elf_mote_register_radio(mixed_node_t *node, int slot,
                                    sim_radio_bus_t *bus) {
    /* CC2420 has an rx_incoming buffer + state guard: one kernel
     * RX_BYTE event per on-air byte.  The three MSP430-only RF-delivery
     * quirks (in-line ticking step on synchronous RX, mini-step drain
     * when the RXFIFO is full, sender self-wake after a frame) are
     * declared as caps so the delivery code stays type-blind (M25). */
    sim_radio_bus_register(bus, slot, &msp_radio_ops, node,
                           SIM_RADIO_DELIVERY_PER_BYTE,
                           SIM_RADIO_CAP_RX_TICKING_STEP |
                           SIM_RADIO_CAP_DRAIN_MINI_STEP |
                           SIM_RADIO_CAP_WAKE_SENDER_POST_TX);
}

/* ============================================================
 * Cooja-style execute tick (ex runner tick_one_msp430)
 *
 * Exported — besides the runner's msp_mote_execute adapter, the
 * frame-delivery pre-sync path calls it for receivers that are not
 * the currently-ticking node (that path moves to the bus in Phase 5).
 * ============================================================ */

int64_t msp430_elf_mote_tick(mixed_node_t *node, int64_t sim_ns) {
    msp430_cpu_t *cpu = &node->plat.msp.cpu;
    int64_t t_us = sim_ns / 1000LL;
    int64_t jump_us = 0;

    if (cpu->last_execute_us >= 0) {
        jump_us = t_us - cpu->last_execute_us;
        if (jump_us < 0) jump_us = 0;
    }

    /* Apply clock deviation (Cooja MspClock drift simulation) */
    double deviation = node->clock_deviation;
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

    /* Match Cooja's MspMote.execute(t, duration): peripheral events
     * raised during this execute slice should be scheduled relative to
     * the current scheduler time t, not a cycle-derived local time that
     * may have drifted. Pin sim_time_ns to sim_ns across the slice AND
     * re-anchor the cycle/ns conversion: the anchor expresses
     * "sim_time_ns at this cycle count" so subsequent execute_events
     * re-derives sim_time_ns from sim_ns + cycles-since-anchor at the
     * current freq, never lagging behind the scheduler. */
    cpu->sim_time_ns = sim_ns;
    cpu->anchor_sim_time_ns = sim_ns;
    cpu->anchor_cycles = cpu->cycles;
    int64_t returned_us = msp430_step_micros(cpu, jump_us, 1);
    cpu->sim_time_ns = sim_ns;
    cpu->anchor_sim_time_ns = sim_ns;
    cpu->anchor_cycles = cpu->cycles;

    if (deviation != 1.0 && returned_us > 0)
        returned_us = (int64_t)((double)returned_us / deviation);

    cpu->last_execute_us = t_us;
    return returned_us;
}
