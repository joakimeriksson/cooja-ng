/*
 * mote_kinds — the mote-kind registry (Phase 4, M23 —
 * docs/design/refactor-plan.md §3.17).
 *
 * One row per mote kind, indexed by sim_board_kind_t.  The runner's
 * init_node() drives boot and radio registration through the row
 * instead of switching on node_type_t.  Rows are data: adding a mote
 * kind means adding a module and a row, not editing the runner.
 *
 * The MSP430/ARM `ops` slots start NULL and are injected by the
 * runner (sim_mote_kind_set_ops) because those adapter tables still
 * live runner-side — they follow their dependencies in Phase 5/6.
 * Phase 8 folds this table into the full sim_registry.
 */
#include "mote_impl.h"

static sim_mote_kind_t kinds[] = {
    [SIM_BOARD_KIND_MSP430] = {
        .name           = "msp430-elf",
        .node_type      = NODE_MSP430,
        .boot           = msp430_elf_mote_boot,
        .register_radio = msp430_elf_mote_register_radio,
        .ops            = NULL,   /* runner-injected until Phase 5/6 */
    },
    [SIM_BOARD_KIND_ARM] = {
        .name           = "arm-elf",
        .node_type      = NODE_ARM,
        .boot           = arm_elf_mote_boot,
        .register_radio = arm_elf_mote_register_radio,
        .ops            = NULL,   /* runner-injected until Phase 5/6 */
    },
    [SIM_BOARD_KIND_NATIVE] = {
        .name           = "native-cooja",
        .node_type      = NODE_NATIVE,
        .boot           = native_cooja_mote_boot,
        .register_radio = native_cooja_mote_register_radio,
        .ops            = &native_cooja_mote_ops,
    },
    [SIM_BOARD_KIND_JS] = {
        .name           = "js-app",
        .node_type      = NODE_JS,
        .boot           = js_app_mote_boot,
        .register_radio = js_app_mote_register_radio,
        .ops            = &js_app_mote_ops,
    },
};

const sim_mote_kind_t *sim_mote_kind_for(sim_board_kind_t kind) {
    return &kinds[kind];
}

void sim_mote_kind_set_ops(sim_board_kind_t kind,
                           const sim_mote_ops_t *ops) {
    kinds[kind].ops = ops;
}
