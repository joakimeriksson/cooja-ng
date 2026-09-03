/*
 * sim_board — the board/platform registry (Phase 3 of the refactor,
 * docs/design/refactor-plan.md §9 Phase 3).
 *
 * Owns the firmware-extension → board mapping that previously lived as
 * four separate strcmp ladders in the runner (detect_node_type, the
 * MSP430/ARM platform-name derivations, and the init banner label).
 * The filename extension stays a compatibility input — config/CLI
 * platform overrides can layer on sim_board_find() later.
 *
 * Phase 4 (§3.17, M23): `kind` binds to a `sim_mote_kind_t` registry
 * row in src/motes/mote_kinds.c — boot policy, radio registration,
 * and (for native/JS) the sim_mote_ops_t table hang off the row.  The
 * MSP430/ARM ops tables are runner-injected until their execute/serial
 * adapters follow their dependencies (radio bus Phase 5, GDB service
 * Phase 6).  Phase 8 folds both registries into sim_registry.
 */
#ifndef SIM_BOARD_H
#define SIM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum sim_board_kind {
    SIM_BOARD_KIND_MSP430 = 0,
    SIM_BOARD_KIND_ARM,
    SIM_BOARD_KIND_NATIVE,
    SIM_BOARD_KIND_JS,
    SIM_BOARD_KIND_EXTERNAL,
} sim_board_kind_t;

typedef struct sim_board_desc {
    const char *extension;     /* ".sky" — filename compatibility input  */
    const char *name;          /* registry key, also the arch platform
                                * lookup name (msp430_platform_find /
                                * arm_platform_find); NULL for native/JS */
    sim_board_kind_t kind;     /* which mote adapter family runs it      */
    const char *label;         /* "MSP430/Sky" — init banner             */
} sim_board_desc_t;

/* Board for a firmware path, by extension.  Unknown/missing extensions
 * return the default board (Tmote Sky) — the historical fallback. */
const sim_board_desc_t *sim_board_for_path(const char *path);

/* Board by registry name (e.g. "zoul-firefly"), or NULL.  For explicit
 * platform overrides from config/CLI. */
const sim_board_desc_t *sim_board_find(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* SIM_BOARD_H */
