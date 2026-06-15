/*
 * mote_impl.h — shared node-implementation types for the per-kind mote
 * modules (Phase 4 of the refactor, docs/design/refactor-plan.md §3.17).
 *
 * PRIVATE to the src/motes modules and the runner frontend
 * (test/test_mixed_multinode.c).  Not part of the include/sim API: the
 * node struct embeds all four platform types, which the public kernel
 * headers must never depend on.
 *
 * Naming note: `mixed_node_t` / `node_type_t` keep their runner-era
 * names so the Phase 4 extractions stay character-identical moves.
 * Renames are Phase 10 cosmetics.
 */
#ifndef MOTE_IMPL_H
#define MOTE_IMPL_H

#include <stdint.h>
#include <stdbool.h>

#include "msp430_platform.h"
#include "arm_platform.h"
#include "native_node.h"
#include "js_node.h"
#include "sim_board.h"
#include "sim_mote.h"
#include "sim_runtime.h"
#include "sim_radio_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { NODE_MSP430, NODE_ARM, NODE_NATIVE, NODE_JS } node_type_t;

/*
 * Per-chip TX listener context.
 *
 * Each chip's RF TX callback fires with a (node_idx, radio_idx) tag so
 * the harness can dispatch into the medium's per-radio filter API
 * without sniffing the byte stream.  Stable storage is required — the
 * chip's TX listener captures the address.  Slot 0 is the on-board
 * 2.4 GHz radio (CC2420 on MSP430, cc2538_rfcore on ARM); slot 1 is
 * the off-SoC sub-GHz radio (CC1200 on Firefly).  Native motes don't
 * go through this path — they use the legacy rf_tx_byte entry which
 * assumes slot 0.
 */
typedef struct {
    int node_idx;
    int radio_idx;
} rf_listener_ctx_t;

typedef struct sim_mote_env sim_mote_env_t;

typedef struct mixed_node {
    node_type_t type;
    const sim_board_desc_t *board;  /* registry row (Phase 3) — owns
                                     * platform name + banner label */
    int id;
    int slot;                       /* slot index in the runtime mote
                                     * table (Phase 4 — replaces the
                                     * runner's MOTE_IDX pointer math
                                     * in module code) */
    const sim_mote_env_t *env;      /* runner glue bundle, set by
                                     * init_node() before boot */
    char line_buf[256];
    int line_pos;
    char firmware_path[256];
    double clock_deviation; /* 1.0 = normal, <1.0 = slower (Cooja MspClock) */
    int64_t last_execute_ns; /* last tick sim_ns (for ns-precision stepping) */
    double ideal_cycles;     /* cumulative ideal cycle target (like MSPSim lastMicrosCycles) */
    rf_listener_ctx_t rf_ctx[2];    /* per-radio TX listener tags (ex
                                     * runner rf_ctx_slot0/1[] arrays) */
    bool native_had_tx;             /* native: last tick had a TX (ex
                                     * runner native_had_tx[] — TX yield
                                     * vs TSCH busywait discrimination) */
    bool exec_had_tx;               /* native: execute→dispatcher handoff
                                     * for post-tick RF distribution (ex
                                     * runner native_exec_had_tx global;
                                     * dispatcher pre-clears + reads it) */
    union {
        msp430_platform_t msp;
        arm_platform_t arm;
        native_node_t native;
        js_node_t js;
    } plat;
} mixed_node_t;

/*
 * sim_mote_env_t — runner-owned glue injected into the per-kind boot
 * functions.  src/motes code must never link against runner symbols
 * (the runner becomes a thin frontend by Phase 10); everything a boot
 * or adapter body needs from the harness arrives through this bundle.
 *
 * Static lifetime required — chips capture these pointers for the
 * node's lifetime.
 *
 * Every member is documented Phase 5/6 debt: the RF glue dissolves
 * into the radio bus (Phase 5), and the console/observer callbacks become
 * observer events (Phase 6).
 */
struct sim_mote_env {
    sim_runtime_t   *sim;        /* kernel instance (public API) */
    sim_radio_bus_t *radio_bus;
    const int       *verbose;
    const int64_t   *node_start_ns;   /* per-slot start gates */

    /* Console: every UART byte off the node (user_data = node). */
    void (*uart_byte)(void *node, uint8_t byte);

    /* Chip-side RF TX trampoline (user_data = &node->rf_ctx[slot]). */
    void (*chip_tx_byte)(void *rf_ctx, uint8_t byte);

    /* sim_host channel push (user_data = node). */
    void (*radio_set_channel)(void *node, int radio_idx, int channel);

    /* cc2538 RF Core observer + channel callbacks (user_data = node). */
    void (*rfcore_state_change)(void *node, int old_state, int new_state);
    void (*rfcore_channel_change)(void *node, int channel);

    /* CC1200 CCA query (user_data = node). */
    bool (*cc1200_channel_busy)(void *node);

    /* Native-mote RF paths (user_data = node).  rf_tx_byte assumes
     * radio slot 0; native_yield is cross-node ACK-turnaround policy
     * and stays in the runner — most tempting wrong move (§3.17). */
    void (*rf_tx_byte)(void *node, uint8_t byte);
    void (*rf_frame)(void *node, const uint8_t *frame, int len);
    void (*native_yield)(void *node);

    /* JS-mote frame TX (user_data = node). */
    void (*js_rf_frame)(void *node, const uint8_t *frame, int len);

    /* TSCH channel sync for native motes (M20): push a channel change
     * into the radio medium for slot `slot`. */
    void (*native_channel_sync)(int slot, int channel);
};

/* The mote vtable's opaque impl pointer is always a mixed_node_t. */
#define MOTE_IMPL(m) ((mixed_node_t *)(m)->impl)

/* ============================================================
 * Per-kind module APIs (one module per mote kind, M19–M22).
 *
 * <kind>_mote_boot: platform init + firmware load + boot patching +
 * chip/console wiring for one node.  <kind>_mote_register_radio:
 * radio-endpoint ops + delivery mode onto the bus — a SEPARATE
 * post-boot call because js_node_start() can TX during script init(),
 * which historically happens before radio registration (§3.17).
 * ============================================================ */

/* M19: QuickJS application motes. */
extern const sim_mote_ops_t js_app_mote_ops;
int  js_app_mote_boot(mixed_node_t *node, int slot, const char *script_path,
                      int node_id, const sim_mote_env_t *env);
void js_app_mote_register_radio(mixed_node_t *node, int slot,
                                sim_radio_bus_t *bus);

/* M20: native Cooja motes (dlopen'd Contiki shared library). */
extern const sim_mote_ops_t native_cooja_mote_ops;
int  native_cooja_mote_boot(mixed_node_t *node, int slot,
                            const char *firmware_path, int node_id,
                            const sim_mote_env_t *env);
void native_cooja_mote_register_radio(mixed_node_t *node, int slot,
                                      sim_radio_bus_t *bus);

/* M21/M38: MSP430 emulated-ELF motes.  Boot + tick + radio ops + the full
 * mote vtable (msp430_elf_mote_ops; the execute/serial/sync adapters moved
 * here in M38 once their radio-bus + GDB deps cleared).  The tick is
 * exported because the frame-delivery pre-sync path also calls it. */
extern const sim_mote_ops_t msp430_elf_mote_ops;
int  msp430_elf_mote_boot(mixed_node_t *node, int slot,
                          const char *firmware_path, int node_id,
                          const sim_mote_env_t *env);
void msp430_elf_mote_register_radio(mixed_node_t *node, int slot,
                                    sim_radio_bus_t *bus);
int64_t msp430_elf_mote_tick(mixed_node_t *node, int64_t sim_ns);
/* PC-trace debug instrumentation (Phase 10 M55).  install returns the
 * resolved cc2420_transmit address (0 = non-MSP430 or unresolved); the
 * counts getter feeds the end-of-run stats. */
uint32_t msp430_elf_mote_install_pc_trace(mixed_node_t *node);
void msp430_elf_mote_pc_trace_counts(int *cc2420_tx, int *eb_process,
                                     int *queue_add);

/* M22/M38: ARM emulated-ELF motes (CC2538 / firefly / nRF52840 /
 * nRF54L15).  Same shape as M21: boot + tick + radio ops + the full mote
 * vtable (arm_elf_mote_ops). */
extern const sim_mote_ops_t arm_elf_mote_ops;
int  arm_elf_mote_boot(mixed_node_t *node, int slot,
                       const char *firmware_path, int node_id,
                       const sim_mote_env_t *env);
void arm_elf_mote_register_radio(mixed_node_t *node, int slot,
                                 sim_radio_bus_t *bus);
int64_t arm_elf_mote_tick(mixed_node_t *node, int64_t sim_ns);
/* Cycle-derived "now" in ns (raw intra-step value) for the UI/timeline
 * rf-state event — keeps arm_systick.h out of the runner (Phase 10 M53). */
int64_t arm_elf_mote_now_ns(const sim_mote_t *m);

/* ============================================================
 * Mote-kind registry (M23) — one row per mote kind, indexed by the
 * board registry's sim_board_kind_t.  init_node() becomes data-driven:
 * kind lookup → boot → register_radio, no per-kind branches.
 *
 * `ops` is module-owned for every kind (M38 moved the MSP430/ARM
 * adapter tables into their modules, retiring the runner injection).
 * ============================================================ */

typedef struct sim_mote_kind {
    const char *name;            /* "msp430-elf", "arm-elf", ...      */
    node_type_t node_type;       /* feeds nodes[].type until Phase 5
                                  * de-types frame-delivery policy    */
    int  (*boot)(mixed_node_t *node, int slot, const char *path,
                 int node_id, const sim_mote_env_t *env);
    void (*register_radio)(mixed_node_t *node, int slot,
                           sim_radio_bus_t *bus);
    const sim_mote_ops_t *ops;
} sim_mote_kind_t;

const sim_mote_kind_t *sim_mote_kind_for(sim_board_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif /* MOTE_IMPL_H */
