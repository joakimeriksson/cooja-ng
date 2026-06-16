/*
 * csim_plugin — the public ABI an external plugin compiles against
 * (Phase 9, docs/design/refactor-plan.md §3.23 / §7.2).
 *
 * A plugin is a shared object (.so) that exports exactly one symbol,
 * `csim_plugin_init`.  The host dlopen()s it, hands it a `csim_api_t` — a
 * bundle of host capabilities (function-pointer vtables) — and the plugin
 * registers a service through it.  The plugin reaches the host ONLY through
 * the passed vtables; it never dlsym()s host symbols, so the host needs no
 * `-rdynamic`/`-ldl`.
 *
 * v1 capability surface is intentionally minimal (the §9 Phase-9 stop
 * condition): registering a built-in-style **service** (a `sim_service_ops_t`
 * — a name + four function pointers, an interface, not an internal struct).
 * The service's `on_event` sees the clean value struct `sim_observer_event_t`
 * and an OPAQUE `sim_runtime_t *` it only passes back into kernel APIs.
 * Registering platforms / SoCs / radio media is DEFERRED — those descriptor
 * catalogs don't exist yet (§5/§6), and exporting them would breach the stop
 * condition.  They are version-gated future additions.
 *
 * ABI stability: growing `csim_api_t` (or any vtable) later is compatible only
 * if version-gated — a plugin must check `api->version` before touching a
 * field newer than the version it was built for, and the host must bump
 * CSIM_PLUGIN_API_VERSION whenever the surface grows.
 */
#ifndef CSIM_PLUGIN_H
#define CSIM_PLUGIN_H

#include <stdint.h>

#include "sim_service.h"    /* sim_service_ops_t (interface); sim_runtime_t (opaque fwd) */
#include "sim_observer.h"   /* sim_observer_event_t (the value the on_event sees)        */

#ifdef __cplusplus
extern "C" {
#endif

/* ABI version.  GROWS additively: new fields are APPENDED to the end of the
 * vtable structs and the version is bumped.  A plugin checks
 * `api->version >= <the version it needs>` (NOT exact equality), so a newer
 * host always satisfies an older plugin; a v1 plugin keeps working on a v2
 * host because it never reads past the field it knows.
 *   v1: register_service.
 *   v2: + register_radio_medium (Phase 11). */
#define CSIM_PLUGIN_API_VERSION 2u

/* Opaque to the plugin — it only passes the pointer back into the registry
 * vtable, never dereferences it. */
typedef struct sim_registry sim_registry_t;
/* A registrable radio medium — defined in radio_medium.h (a plugin medium
 * includes it).  Opaque here (only used as a pointer in the vtable). */
typedef struct sim_medium_type sim_medium_type_t;

/* Registry capability handed to the plugin.  Fields are append-only (see the
 * version contract above).  register_platform / register_mote_type remain
 * absent (their descriptor catalogs don't exist yet — §5/§6). */
typedef struct csim_registry_ops {
    /* Register a service into the host registry's catalog.  Returns the slot
     * index >= 0, or < 0 on a duplicate name or a full table.  The host
     * attaches newly-registered services after the plugin-load loop. */
    int (*register_service)(sim_registry_t *reg, const sim_service_ops_t *ops);
    /* v2 (gate on api->version >= 2): register a radio-medium policy, selectable
     * by config medium.type = its name.  Returns the slot index >= 0, or < 0. */
    int (*register_radio_medium)(sim_registry_t *reg, const sim_medium_type_t *m);
} csim_registry_ops_t;

/* Optional host-routed logging — one printf-like function.  Keeps plugin
 * output ordered with the host's output.  May be NULL in v1. */
typedef struct csim_log_ops {
    void (*printf)(const char *fmt, ...);
} csim_log_ops_t;

typedef struct csim_api {
    uint32_t                   version;   /* = CSIM_PLUGIN_API_VERSION */
    const csim_registry_ops_t *registry;  /* non-NULL in v1 */
    const csim_log_ops_t      *log;       /* may be NULL */
    sim_registry_t            *reg;        /* the registry to register into */
} csim_api_t;

/* The plugin's one exported symbol — dlsym()'d by the host loader.  The plugin
 * should check `api->version` and return < 0 if it is incompatible.  Return 0
 * on success; < 0 makes the host report the error and skip the plugin.
 *
 * Lifetime: the `csim_api_t` *pointer* is valid only for the duration of this
 * call — the host may pass a stack local.  To use the api later (e.g. from a
 * service callback), copy the struct BY VALUE; the vtable pointers it holds
 * (`registry`, `log`) and `reg` are stable for the process lifetime. */
int csim_plugin_init(const csim_api_t *api);

#ifdef __cplusplus
}
#endif

#endif /* CSIM_PLUGIN_H */
