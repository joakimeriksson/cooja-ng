/*
 * sim_registry — the static built-in registry (Phase 8 M45,
 * docs/design/refactor-plan.md §3.21 / §7.1).
 *
 * One registry is the single lookup surface for the simulator's built-in
 * platforms (boards), mote kinds, and services.  Phase 8 routes the runner's
 * three previously-separate lookup mechanisms through it:
 *
 *   - boards:    sim_board_for_path / sim_board_find  (src/sim/sim_board.c)
 *   - mote kinds: sim_mote_kind_for                   (src/motes/mote_kinds.c)
 *   - services:  the hard-coded &xxx_service_ops at the attach sites
 *
 * Design (locked, §3.21): the registry REFERENCES the existing tables — it
 * does not own them.  csim_register_builtin_{platforms,mote_types} store a
 * pointer to the existing static boards[]/kinds[] table (+ count) for
 * introspection and the Phase-9 plugin path; the lookup accessors forward to
 * the existing function bodies so the board Sky-default fallback and the
 * enum-indexed kind access stay byte-for-byte verbatim.  No data literal is
 * relocated in Phase 8.
 *
 * The service catalog is the one piece the registry genuinely owns: a
 * name → ops table that the attach sites resolve through by name
 * (sim_registry_find_service), so adding/replacing a service becomes a
 * registration, not an edit at the call site.
 *
 * v1 rules (§7.1): built-in registration only, no dlopen, no ABI promises.
 * SoC/media populate functions are intentionally absent until §5 lands a
 * descriptor catalog; runtime-owned ownership (sim_runtime_registry) is
 * Phase 9.  "plugin" stays reserved for the Phase-9 dlopen case.
 */
#ifndef SIM_REGISTRY_H
#define SIM_REGISTRY_H

#include "sim_board.h"     /* sim_board_desc_t, sim_board_kind_t */
#include "sim_service.h"   /* sim_service_ops_t                  */
#include "radio_medium.h"  /* radio_medium_type_t, sim_medium_ops_t (Phase 11) */

#ifdef __cplusplus
extern "C" {
#endif

/* Completed in src/motes/mote_impl.h.  The registry only stores/returns
 * pointers to it, so the forward declaration is enough here (and keeps the
 * private mote header out of the public registry surface). */
typedef struct sim_mote_kind sim_mote_kind_t;

#define SIM_REGISTRY_MAX_SERVICES 16
#define SIM_REGISTRY_MAX_MEDIA     8

/* A registered radio medium (Phase 11): a name → policy ops, plus which
 * generic pipeline it runs (UDGM = the full filter machinery with custom
 * policy; NONE = the all-to-all bypass).  Built-ins "udgm"/"none"; a plugin
 * registers a custom medium that runs the UDGM pipeline with its own ops. */
typedef struct sim_medium_type {
    const char             *name;
    radio_medium_type_t     pipeline;
    const sim_medium_ops_t *ops;
} sim_medium_type_t;

typedef struct sim_registry {
    /* Referenced built-in tables (owned by sim_board.c / mote_kinds.c). */
    const sim_board_desc_t *boards;
    int                     board_count;
    const sim_mote_kind_t  *kinds;
    int                     kind_count;
    /* The service catalog the registry owns (name → ops). */
    const sim_service_ops_t *services[SIM_REGISTRY_MAX_SERVICES];
    int                      service_count;
    /* The radio-medium catalog (name → policy). */
    const sim_medium_type_t *media[SIM_REGISTRY_MAX_MEDIA];
    int                      media_count;
} sim_registry_t;

/* Built-in registration (call once at startup, after zero-init).  Each stores
 * the referenced table pointer + count into the registry. */
void csim_register_builtin_platforms(sim_registry_t *r);
void csim_register_builtin_mote_types(sim_registry_t *r);
void csim_register_builtin_services(sim_registry_t *r);
void csim_register_builtin_media(sim_registry_t *r);   /* "udgm", "none" */

/* Service catalog: append an ops by name; reject duplicates by name and a full
 * table.  Returns the slot index ≥ 0, or < 0 on failure.  find returns the
 * registered ops whose ->name matches, or NULL. */
int  sim_registry_register_service(sim_registry_t *r,
                                   const sim_service_ops_t *ops);
const sim_service_ops_t *sim_registry_find_service(const sim_registry_t *r,
                                                   const char *name);

/* Radio-medium catalog: append a medium by name (rejects duplicate name + a
 * full table); find returns the registered medium whose ->name matches, or
 * NULL.  Mirrors the service catalog. */
int  sim_registry_register_radio_medium(sim_registry_t *r,
                                        const sim_medium_type_t *m);
const sim_medium_type_t *sim_registry_find_radio_medium(const sim_registry_t *r,
                                                        const char *name);

/* Board lookup — forwards to the existing sim_board.c bodies (identical Sky
 * default fallback for unknown/missing extensions). */
const sim_board_desc_t *sim_registry_board_for_path(const sim_registry_t *r,
                                                    const char *path);
const sim_board_desc_t *sim_registry_board_find(const sim_registry_t *r,
                                                const char *name);

/* Mote-kind lookup — forwards to the existing mote_kinds.c body (enum-indexed
 * row, no bounds change). */
const sim_mote_kind_t *sim_registry_mote_kind_for(const sim_registry_t *r,
                                                  sim_board_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif /* SIM_REGISTRY_H */
