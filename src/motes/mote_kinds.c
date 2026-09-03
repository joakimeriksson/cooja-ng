/*
 * mote_kinds — the mote-kind registry (Phase 4, M23 —
 * docs/design/refactor-plan.md §3.17).
 *
 * One row per mote kind, indexed by sim_board_kind_t.  The runner's
 * init_node() drives boot and radio registration through the row
 * instead of switching on node_type_t.  Rows are data: adding a mote
 * kind means adding a module and a row, not editing the runner.
 *
 * Every kind's `ops` is module-owned (M38 moved the MSP430/ARM adapter
 * tables into their modules, retiring the runner injection).  Phase 8
 * folds this table into the full sim_registry.
 */
#include "mote_impl.h"
#include "sim_registry.h"

static sim_mote_kind_t kinds[] = {
    [SIM_BOARD_KIND_MSP430] = {
        .name           = "msp430-elf",
        .banner_label   = "MSP430",
        .node_type      = NODE_MSP430,
        .boot           = msp430_elf_mote_boot,
        .register_radio = msp430_elf_mote_register_radio,
        .ops            = &msp430_elf_mote_ops,
    },
    [SIM_BOARD_KIND_ARM] = {
        .name           = "arm-elf",
        .banner_label   = "ARM",
        .node_type      = NODE_ARM,
        .boot           = arm_elf_mote_boot,
        .register_radio = arm_elf_mote_register_radio,
        .ops            = &arm_elf_mote_ops,
    },
    [SIM_BOARD_KIND_NATIVE] = {
        .name           = "native-cooja",
        .banner_label   = "NATIVE",
        .node_type      = NODE_NATIVE,
        .boot           = native_cooja_mote_boot,
        .register_radio = native_cooja_mote_register_radio,
        .ops            = &native_cooja_mote_ops,
    },
    [SIM_BOARD_KIND_JS] = {
        .name           = "js-app",
        .banner_label   = "NATIVE",  /* banner lumps JS with native (historical) */
        .node_type      = NODE_JS,
        .boot           = js_app_mote_boot,
        .register_radio = js_app_mote_register_radio,
        .ops            = &js_app_mote_ops,
    },
    [SIM_BOARD_KIND_EXTERNAL] = {
        .name           = "external",
        .banner_label   = "EXT",
        .node_type      = NODE_EXT,
        .boot           = external_mote_boot,
        .register_radio = external_mote_register_radio,
        .ops            = &external_mote_ops,
    },
};

const sim_mote_kind_t *sim_mote_kind_for(sim_board_kind_t kind) {
    return &kinds[kind];
}

/* --- Registry glue (Phase 8 M45).  The registry references this table; the
 * accessor forwards to the enum-indexed body above. --- */

void csim_register_builtin_mote_types(sim_registry_t *r) {
    if (!r) return;
    r->kinds = kinds;
    r->kind_count = (int)(sizeof(kinds) / sizeof(kinds[0]));
}

const sim_mote_kind_t *sim_registry_mote_kind_for(const sim_registry_t *r,
                                                  sim_board_kind_t kind) {
    (void)r;
    return sim_mote_kind_for(kind);
}
