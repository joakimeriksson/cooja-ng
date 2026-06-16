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

#define CSIM_PLUGIN_API_VERSION 1u

/* Opaque to the plugin — it only passes the pointer back into the registry
 * vtable, never dereferences it. */
typedef struct sim_registry sim_registry_t;

/* Registry capability handed to the plugin.  v1 = service registration only.
 * register_radio_medium / register_platform / register_mote_type are
 * deliberately absent in v1 (see the file header). */
typedef struct csim_registry_ops {
    /* Register a service into the host registry's catalog.  Returns the slot
     * index >= 0, or < 0 on a duplicate name or a full table.  The host
     * attaches newly-registered services after the plugin-load loop. */
    int (*register_service)(sim_registry_t *reg, const sim_service_ops_t *ops);
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
 * on success; < 0 makes the host report the error and skip the plugin. */
int csim_plugin_init(const csim_api_t *api);

#ifdef __cplusplus
}
#endif

#endif /* CSIM_PLUGIN_H */
