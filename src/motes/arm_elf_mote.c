/*
 * arm_elf_mote — boot policy, Cooja-style execute tick, and radio
 * endpoint ops for the ARM emulated-ELF mote kind (CC2538 /
 * zoul-firefly / nRF52840 / nRF54L15; Phase 4, M22 —
 * docs/design/refactor-plan.md §3.17).
 *
 * Bodies are character-identical moves of the former runner code
 * (test/test_mixed_multinode.c init_arm_node / tick_one_arm /
 * arm_radio_ops / arm54l_radio_ops), with the enumerated seam
 * substitutions: runner callbacks → env members, `verbose` →
 * *env->verbose, nodes[idx] → the node pointer.
 *
 * The arm_mote_ops adapter table (execute/serial_input/...) stays in
 * the runner: it is entangled with emu_rx_queue_drain (frame-delivery
 * policy → Phase 5), ticking_node_idx (→ Phase 5), and the GDB stubs
 * (→ Phase 6 service).  Per §3.17 those follow their dependencies.
 */
#include "mote_impl.h"

#include "arm_elf.h"
#include "cc2538_soc.h"
#include "cc2538_uart.h"    /* arm_mote_serial_input (M38) */
#include "nrf52840_soc.h"
#include "nrf54l15_soc.h"
#include "gdb_stub.h"       /* arm_mote_execute cpu->gdb_stub poll (M38) */
#include "sim_state.h"      /* SIM_RADIO_* for arm_elf_mote_rf_sim_state (M61) */

#include <stdio.h>
#include <string.h>

/* ============================================================
 * Boot policy
 * ============================================================ */

int arm_elf_mote_boot(mixed_node_t *node, int slot,
                      const char *firmware_path, int node_id,
                      const sim_mote_env_t *env) {
    int idx = slot;
    arm_platform_t *plat = &node->plat.arm;

    /* Platform name comes from the board registry row (Phase 3). */
    const char *plat_name = node->board->name;
    const arm_platform_config_t *pcfg = arm_platform_find(plat_name);
    if (!pcfg) { fprintf(stderr, "Platform '%s' not found\n", plat_name); return -1; }

    arm_platform_init(plat, pcfg);
    if (arm_load_elf(&plat->cpu, firmware_path) != 0) {
        fprintf(stderr, "Cannot load firmware: %s\n", firmware_path);
        arm_platform_destroy(plat);
        return -1;
    }

    arm_platform_set_console(plat, env->uart_byte, node);
    plat->host.radio_user_data  = node;
    plat->host.radio_set_channel = env->radio_set_channel;
    plat->host.radio_set_power   = env->radio_set_power;

    cc2538_soc_t   *cc_soc   = arm_platform_cc2538(plat);
    nrf52840_soc_t *nrf_soc  = arm_platform_nrf52840(plat);
    nrf54l15_soc_t *nrfl_soc = arm_platform_nrf54l15(plat);

    if (cc_soc) {
        /* CC2538-class platform (cc2538dk / openmote / zoul-firefly). */
        node->rf_ctx[0].node_idx  = idx;
        node->rf_ctx[0].radio_idx = 0;
        cc2538_rfcore_set_tx_callback(&cc_soc->rfcore, env->chip_tx_byte,
                                       &node->rf_ctx[0]);
        cc_soc->rfcore.node_id = node_id;
        cc_soc->rfcore.state_callback = env->rfcore_state_change;
        cc_soc->rfcore.state_user_data = node;
        cc2538_rfcore_set_channel_callback(&cc_soc->rfcore,
                                            env->rfcore_channel_change, node);

        if (pcfg->has_cc1200) {
            node->rf_ctx[1].node_idx  = idx;
            node->rf_ctx[1].radio_idx = 1;
            cc1200_set_rf_listener(&cc_soc->cc1200, env->chip_tx_byte,
                                    &node->rf_ctx[1]);
            cc1200_set_channel_busy_query(&cc_soc->cc1200,
                                           env->cc1200_channel_busy, node);
        }

        /* Seed RFRND and sleep timer uniquely per node. */
        {
            uint32_t h = (uint32_t)node_id;
            h ^= h << 13; h ^= h >> 17; h ^= h << 5;
            h *= 2654435761u;
            h ^= h >> 16;
            cc_soc->rfcore.rfrnd_state = h ? h : 0xDEADBEEF;
            if (*env->verbose)
                printf("  Node %d: rfrnd_seed=0x%08x\n", node_id, h);
        }

        /* Unique IEEE 64-bit ext_addr using Cooja's repeated scheme. */
        uint8_t unique_addr[8] = { (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                    (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                    (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8),
                                    (uint8_t)(node_id & 0xFF), (uint8_t)(node_id >> 8) };
        memcpy(cc_soc->rfcore.ext_addr, unique_addr, 8);
    } else if (nrf_soc) {
        /* nRF52840 dongle. The SoC has only one radio (on-die), so
         * only slot 0 is in play. TX bytes flow out through the radio
         * listener; RX bytes flow in via nrf_radio_receive_byte (see
         * mixed_deliver_rf_bytes branch). */
        node->rf_ctx[0].node_idx  = idx;
        node->rf_ctx[0].radio_idx = 0;
        nrf_radio_set_tx_listener(nrf_soc, env->chip_tx_byte,
                                   &node->rf_ctx[0]);

        /* FICR.DEVICEADDR per-node — Contiki uses these to derive the
         * IEEE EUI-64 (Nordic OUI f4:ce:36 prepended in platform.c).
         * Seed with a per-node hash so addresses are distinct. */
        uint32_t h = (uint32_t)node_id;
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        h *= 2654435761u;
        h ^= h >> 16;
        nrf_soc->ficr.deviceaddr0 = h;
        nrf_soc->ficr.deviceaddr1 = (uint32_t)node_id;
        nrf_soc->rng.prng_state   = h ? h : 0xDEADBEEF;

        /* nRF doesn't drive a "channel" register the way cc2538 does;
         * the medium currently uses the on-chip 2.4 GHz channel range
         * (11..26) and we let the firmware's RADIO->FREQUENCY land in
         * unmapped IO. Default to channel 26 (matches firmware default
         * "802.15.4 Default channel: 26"). */
        if (*env->verbose)
            printf("  Node %d [nrf52840]: ficr=%08x:%08x prng=%08x\n",
                   node_id, nrf_soc->ficr.deviceaddr1, nrf_soc->ficr.deviceaddr0,
                   nrf_soc->rng.prng_state);
    } else if (nrfl_soc) {
        /* nRF54L15 (DK).  Single on-die 2.4 GHz radio, slot 0.  TX
         * bytes flow out through the radio listener; RX bytes flow in
         * via nrf54l_radio_receive_byte (see mixed_deliver_rf_bytes). */
        node->rf_ctx[0].node_idx  = idx;
        node->rf_ctx[0].radio_idx = 0;
        nrf54l_radio_set_tx_listener(nrfl_soc, env->chip_tx_byte,
                                      &node->rf_ctx[0]);

        /* FICR.INFO.DEVICEID per-node — Contiki's linkaddr-arch.c
         * combines these two 32-bit words with the Nordic OUI
         * (f4:ce:36) into the EUI-64 stored in linkaddr_node_addr.
         * Every node booting from the same FICR snapshot ends up with
         * the same MAC, RPL drops incoming DIOs as self-frames, and
         * the DAG never forms. Seed with the same per-node hash the
         * nRF52840 path uses for FICR.DEVICEADDR. */
        uint32_t h = (uint32_t)node_id;
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        h *= 2654435761u;
        h ^= h >> 16;
        nrfl_soc->ficr.deviceid0 = h;
        nrfl_soc->ficr.deviceid1 = (uint32_t)node_id;

        if (*env->verbose)
            printf("  Node %d [nrf54l15]: TX listener installed, "
                   "ficr deviceid=%08x:%08x\n",
                   node_id, nrfl_soc->ficr.deviceid1, nrfl_soc->ficr.deviceid0);
    }

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

    /* Run past Contiki main() and platform peripheral setup until at least
     * one event is scheduled (SysTick, sleep timer, or GPTimer).  Without
     * this, the multinode event loop wakes the ARM at node_start_ns, runs
     * a handful of instructions, then asks "next_event_cycle?" — which is
     * still INT64_MAX because the firmware hasn't finished configuring its
     * timer peripherals yet — and schedules the next wakeup for infinity.
     * That's the root cause of the arm-multinode silent no-op bug.
     *
     * Mirror msp430_elf_mote_boot's pattern: run in 10k-cycle batches up
     * to an 8M-cycle budget, break as soon as event_queue has something
     * in it. */
    {
        int64_t limit = plat->cpu.cycles + 8000000;
        while ((int64_t)plat->cpu.cycles < limit) {
            arm_step_until(&plat->cpu, plat->cpu.cycles + 10000);
            if (plat->cpu.event_queue != NULL) break;
        }
        /* Run past the first scheduled event so that early-boot stack
         * initialization (stack_check_init) completes before the multinode
         * simulation starts.  Without this, format_str_v reads 0xCDCDCDCD
         * from an uninitialized stack slot and loops for ~3.4B iterations. */
        if (plat->cpu.event_queue != NULL &&
            plat->cpu.next_event_cycle > (int64_t)plat->cpu.cycles) {
            arm_step_until(&plat->cpu, plat->cpu.next_event_cycle + 100000);
        }
    }

    printf("  Node %d [ARM] initialized: PC=0x%08x SP=0x%08x cycles=%lld eq=%s next_ev=%lld\n",
           node_id, plat->cpu.reg[ARM_PC], plat->cpu.reg[ARM_SP],
           (long long)plat->cpu.cycles,
           plat->cpu.event_queue ? "yes" : "nil",
           (long long)plat->cpu.next_event_cycle);
    return 0;
}

/* ============================================================
 * Radio endpoint ops + bus registration
 * ============================================================ */

static int arm_radio_rxfifo_available(void *m) {
    mixed_node_t *node = (mixed_node_t *)m;
    cc2538_soc_t   *cc_soc   = arm_platform_cc2538(&node->plat.arm);
    nrf52840_soc_t *nrf_soc  = arm_platform_nrf52840(&node->plat.arm);
    nrf54l15_soc_t *nrfl_soc = arm_platform_nrf54l15(&node->plat.arm);
    if (cc_soc) {
        cc2538_rfcore_t *rf = &cc_soc->rfcore;
        int avail = RF_RXFIFO_SIZE - (rf->rxfifo_len - rf->rxfifo_rd);
        const arm_platform_config_t *pcfg = node->plat.arm.config;
        if (pcfg && pcfg->has_cc1200) {
            int cc1200_avail = 128 - cc_soc->cc1200.rx_count;
            if (cc1200_avail < avail) avail = cc1200_avail;
        }
        return avail;
    }
    if (nrf_soc || nrfl_soc) {
        /* EasyDMA — no shared fixed-size FIFO; parser drops bytes when
         * not in RX state. */
        return 128;
    }
    return 0;
}
static void arm_radio_receive_byte(void *m, uint8_t byte, int8_t rssi) {
    mixed_node_t *node = (mixed_node_t *)m;
    cc2538_soc_t   *cc_soc   = arm_platform_cc2538(&node->plat.arm);
    nrf52840_soc_t *nrf_soc  = arm_platform_nrf52840(&node->plat.arm);
    nrf54l15_soc_t *nrfl_soc = arm_platform_nrf54l15(&node->plat.arm);
    if (cc_soc) {
        const arm_platform_config_t *pcfg = node->plat.arm.config;
        cc_soc->rfcore.rx_rssi = rssi;
        cc2538_rfcore_receive_byte(&cc_soc->rfcore, byte);
        if (pcfg && pcfg->has_cc1200) {
            cc_soc->cc1200.rx_rssi = rssi;
            cc1200_receive_byte(&cc_soc->cc1200, byte);
        }
    }
    if (nrf_soc)  nrf_radio_receive_byte(nrf_soc, byte);
    if (nrfl_soc) nrf54l_radio_receive_byte(nrfl_soc, byte);
}
static bool arm_radio_rx_busy(void *m) { (void)m; return false; }
static const mote_radio_ops_t arm_radio_ops = {
    arm_radio_receive_byte, arm_radio_rxfifo_available, arm_radio_rx_busy,
    NULL /* rx_stall */, NULL /* current_channel */, NULL /* mark_collisions */
};

/* nrf54l15 variant: same endpoint plus the RX-stall recovery op (M9.5).
 * The bus only arms its per-receiver stall timer when rx_stall is set,
 * so other ARM platforms don't pay for timer events they ignore. */
static void arm54l_radio_rx_stall(void *m) {
    mixed_node_t *node = (mixed_node_t *)m;
    nrf54l15_soc_t *nrfl_soc = arm_platform_nrf54l15(&node->plat.arm);
    if (nrfl_soc) nrf54l_radio_rx_stall(nrfl_soc);
}
static const mote_radio_ops_t arm54l_radio_ops = {
    arm_radio_receive_byte, arm_radio_rxfifo_available, arm_radio_rx_busy,
    arm54l_radio_rx_stall, NULL /* current_channel */, NULL /* mark_collisions */
};

void arm_elf_mote_register_radio(mixed_node_t *node, int slot,
                                 sim_radio_bus_t *bus) {
    /* Chips with an rx_incoming buffer + state guard (cc2538_rfcore,
     * nrf54l15) take one kernel RX_BYTE event per on-air byte;
     * nrf52840's DMA-style RADIO needs whole frames (per-byte
     * regressed 4-node RPL convergence — see 9ebe99a investigation),
     * so it stays BATCH. */
    bool nrf54l = arm_platform_nrf54l15(&node->plat.arm) != NULL;
    sim_radio_delivery_mode_t mode =
        (nrf54l || arm_platform_cc2538(&node->plat.arm) != NULL)
        ? SIM_RADIO_DELIVERY_PER_BYTE : SIM_RADIO_DELIVERY_BATCH;
    sim_radio_bus_register(bus, slot,
                           nrf54l ? &arm54l_radio_ops : &arm_radio_ops,
                           node, mode, /*caps=*/0);
}

/* ============================================================
 * Cooja-style execute tick (ex runner tick_one_arm)
 * ============================================================ */

/* Mirror of the MSP430 execute tick for ARM/CC2538 nodes.  Applies the same
 * Cooja MspClock-style per-node clock deviation, pins sim_time_ns across
 * the slice, uses arm_step_micros for cycle-accurate accumulation, and
 * returns the next-event lead time so the caller can self-schedule. */
int64_t arm_elf_mote_tick(mixed_node_t *node, int64_t sim_ns) {
    arm_cpu_t *cpu = &node->plat.arm.cpu;
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
        cpu->step_cycle_remainder += exact - (double)jump_us;
        if (cpu->step_cycle_remainder > 1.0) {
            jump_us++;
            cpu->step_cycle_remainder -= 1.0;
        }
    }

    /* Match Cooja's execute(t, duration): peripheral events raised
     * during this slice should be scheduled relative to the scheduler's
     * time t, not a cycle-derived local time. Pin sim_time_ns before
     * and after, AND re-anchor cycles → ns conversion so freq changes
     * during the slice don't drift sim_time_ns away from sim_ns. */
    cpu->sim_time_ns = sim_ns;
    cpu->anchor_sim_time_ns = sim_ns;
    cpu->anchor_cycles = cpu->cycles;
    int64_t returned_us = arm_step_micros(cpu, jump_us, 1);
    cpu->sim_time_ns = sim_ns;
    cpu->anchor_sim_time_ns = sim_ns;
    cpu->anchor_cycles = cpu->cycles;

    if (deviation != 1.0 && returned_us > 0)
        returned_us = (int64_t)((double)returned_us / deviation);

    cpu->last_execute_us = t_us;
    return returned_us;
}

/* ============================================================
 * Mote vtable adapters (M38 — moved from the runner; §3.19).
 *
 * Character-identical to the runner's arm_mote_* functions, with the
 * runner-global seams rewired through the mote (nodes[idx] → MOTE_IMPL(m),
 * &radio_bus → node->env->radio_bus, &sim_rt → node->env->sim,
 * emu_rx_queue_drain → sim_radio_bus_drain_rx).  The GDB poll consults the
 * CPU's own cpu->gdb_stub back-pointer (set by the gdb service, M37).
 * ============================================================ */

static int64_t arm_mote_sim_time_ns(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.sim_time_ns;
}
static int64_t arm_mote_cycles(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.cycles;
}
static uint32_t arm_mote_freq_hz(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.cpu_freq_hz;
}

/* Cycle-derived "now" in ns for the UI/timeline rf-state event (Phase 10
 * M53).  Deliberately the raw intra-step value — arm_cycles_to_ns(cycles,
 * freq), NOT the pinned sim_time_ns — matching the runner's historical
 * computation so the timeline timestamps are byte-identical. */
int64_t arm_elf_mote_now_ns(const sim_mote_t *m) {
    const arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    return arm_cycles_to_ns(cpu->cycles, cpu->cpu_freq_hz);
}

/* Classify a cc2538 RF-core state (RF_STATE_*) into a sim_radio_state_t value
 * for the UI/timeline rf-state handler (Phase 10 M61) — keeps the chip enum
 * out of the runner.  Returns the sim_radio_state_t as int. */
int arm_elf_mote_rf_sim_state(int rf_state) {
    if (rf_state >= RF_STATE_TX_CALIBR && rf_state <= RF_STATE_TX_FINAL)
        return SIM_RADIO_TX;
    if (rf_state == RF_STATE_RX)
        return SIM_RADIO_RX;
    if (rf_state == RF_STATE_RX_CALIBR || rf_state == RF_STATE_SFD_WAIT)
        return SIM_RADIO_ON;
    return SIM_RADIO_OFF;
}
static int64_t arm_mote_instructions(const sim_mote_t *m) {
    return MOTE_IMPL(m)->plat.arm.cpu.instructions;
}

static void arm_mote_step_until(sim_mote_t *m, int64_t target) {
    arm_step_until(&MOTE_IMPL(m)->plat.arm.cpu, target);
}

static int64_t arm_mote_sync_to_time(sim_mote_t *m, int64_t sim_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    if (sim_ns > sim_runtime_now_ns(node->env->sim))
        sim_ns = sim_runtime_now_ns(node->env->sim);
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

/* ARM per-byte RX clock sync: distinct unclamped pin-step-pin body (it
 * differs from the MSP430 clamped sync on purpose — do not unify). */
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

/* M54: shift sim_time_ns + cycles forward by the startup delay so the sleep
 * timer shows different elapsed times per node.  Returns true — the runner's
 * node_start_ns then uses the post-shift sim_time directly (no double-add). */
static bool arm_mote_apply_startup_delay(sim_mote_t *m, int64_t delay_ns) {
    arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    cpu->sim_time_ns += delay_ns;
    cpu->cycles += delay_ns * cpu->cpu_freq_hz / 1000000000LL;
    return true;
}

static int64_t arm_mote_execute(sim_mote_t *m, int64_t now_ns) {
    mixed_node_t *node = MOTE_IMPL(m);
    sim_radio_bus_t *bus = node->env->radio_bus;
    int idx = node->slot;
    /* Emulated ARM: same Cooja-style tick as MSP430 so peripheral events
     * are anchored to the scheduler's event time. */

    /* GDB stub (M37): consulted via the CPU's own back-pointer, set by the
     * gdb service at attach.  If the CPU is halted at a breakpoint, skip
     * the tick and return a +1µs wakeup so we keep checking the stub. */
    arm_cpu_t *cpu = &node->plat.arm.cpu;
    gdb_stub_t *gdb = (gdb_stub_t *)cpu->gdb_stub;
    if (gdb) {
        gdb_stub_poll(gdb);
        if (gdb->halted) {
            cpu->stopping = false;
            return now_ns + 1000LL;
        }
    }

    sim_radio_bus_set_executing(bus, idx);
    int64_t returned_us = arm_elf_mote_tick(node, now_ns);
    sim_radio_bus_set_executing(bus, -1);

    /* GDB stub: a breakpoint may have fired during the tick.  Clear the
     * cpu->stopping flag so subsequent ticks (after `continue`) can run. */
    if (gdb) {
        cpu->stopping = false;
        if (gdb->halted)
            return now_ns + 1000LL;  /* poll the stub again soon */
    }

    /* Drain any frames queued for this node (mirrors the MSP430 path). */
    if (bus->emu_rx_queue[idx].count > 0)
        sim_radio_bus_drain_rx(bus, node->env->sim, idx);

    /* Match MspMote.execute(t, 1): next normal wakeup from the
     * step_micros lead hint. */
    return now_ns + (returned_us + 1) * 1000LL;
}

static int64_t arm_mote_sched_hint_ns(const sim_mote_t *m, int64_t base_ns) {
    const arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    if (cpu->next_event_cycle <= cpu->cycles)
        return base_ns;
    return base_ns + arm_cycles_to_ns(
        cpu->next_event_cycle - cpu->cycles, cpu->cpu_freq_hz);
}

/* ARM: feed the console UART RX register directly (CC2538-family boards
 * only; Nordic consoles are output-only today). */
static int arm_mote_serial_input(sim_mote_t *m, const uint8_t *buf, int len) {
    cc2538_soc_t *soc = arm_platform_cc2538(&MOTE_IMPL(m)->plat.arm);
    for (int i = 0; i < len; i++)
        cc2538_uart_receive_byte(&soc->uart0, buf[i]);
    return len;
}

static void arm_mote_destroy(sim_mote_t *m) {
    arm_platform_destroy(&MOTE_IMPL(m)->plat.arm);
}
static void arm_mote_reset_time(sim_mote_t *m, int64_t now_ns) {
    arm_cpu_t *cpu = &MOTE_IMPL(m)->plat.arm.cpu;
    cpu->sim_time_ns = now_ns;
    cpu->cycles = now_ns * cpu->cpu_freq_hz / 1000000000LL;
}

static void arm_mote_ui_leds(const sim_mote_t *m, uint8_t leds[3]) {
    cc2538_soc_t *soc = arm_platform_cc2538(&MOTE_IMPL(m)->plat.arm);
    if (!soc)
        return;  /* Nordic boards: no LED model wired up */
    uint32_t pc_data = soc->gpio.ports[2].data;
    leds[0] = (pc_data >> 0) & 1;  /* red */
    leds[1] = (pc_data >> 1) & 1;  /* yellow */
    leds[2] = (pc_data >> 2) & 1;  /* green */
}

static void *arm_mote_get_interface(sim_mote_t *m, int iface) {
    if (iface == SIM_MOTE_IFACE_ARM_CPU)
        return &MOTE_IMPL(m)->plat.arm.cpu;
    return NULL;
}

const sim_mote_ops_t arm_elf_mote_ops = {
    .kind            = "ARM",
    .sim_time_ns     = arm_mote_sim_time_ns,
    .cycles          = arm_mote_cycles,
    .freq_hz         = arm_mote_freq_hz,
    .instructions    = arm_mote_instructions,
    .execute         = arm_mote_execute,
    .step_until      = arm_mote_step_until,
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
    .apply_startup_delay = arm_mote_apply_startup_delay,
};
