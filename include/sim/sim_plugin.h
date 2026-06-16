/*
 * sim_plugin — the host-side plugin loader (Phase 9, §3.23).
 *
 * Loads a plugin .so (see include/sim/csim_plugin.h for the ABI it must
 * implement), builds a csim_api_t, and calls the plugin's csim_plugin_init.
 * The loader is host-only; plugins do NOT include this header.
 */
#ifndef SIM_PLUGIN_H
#define SIM_PLUGIN_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sim_registry sim_registry_t;

/* dlopen `path`, resolve `csim_plugin_init`, hand it a csim_api built around
 * `reg`, and call it.  On success (0) any services the plugin registered are
 * in the registry catalog, ready for the caller to attach; the .so stays
 * loaded for the process lifetime (its ops function pointers must remain
 * valid).  On failure (< 0) a human-readable reason is written to
 * `errbuf` (NUL-terminated, truncated to `errlen`) and nothing is registered.
 */
int sim_plugin_load(const char *path, sim_registry_t *reg,
                    char *errbuf, size_t errlen);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PLUGIN_H */
