/*
 * sim_plugin — host-side plugin loader.  See include/sim/sim_plugin.h.
 *
 * Mirrors the dlopen pattern in src/native/native_node.c.  The plugin reaches
 * the host only through the csim_api vtables we pass it, so we need no
 * -rdynamic / -ldl and use RTLD_LOCAL (NOT RTLD_DEEPBIND — the plugin shares
 * nothing with the host by symbol name).
 */
#include "sim_plugin.h"
#include "csim_plugin.h"
#include "sim_registry.h"

#include <dlfcn.h>
#include <stdarg.h>
#include <stdio.h>

/* Host-routed logging: a plain stdout printf so plugin output orders with the
 * kernel's stdout output (determinism). */
static void plugin_log_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

static const csim_log_ops_t g_log_ops = {
    .printf = plugin_log_printf,
};

/* The registry capability is the real catalog-append, called with the
 * registry the host hands the plugin via api->reg. */
static const csim_registry_ops_t g_registry_ops = {
    .register_service = sim_registry_register_service,
};

int sim_plugin_load(const char *path, sim_registry_t *reg,
                    char *errbuf, size_t errlen) {
    if (!path || !reg) {
        if (errbuf && errlen) snprintf(errbuf, errlen, "null path/registry");
        return -1;
    }
    dlerror();  /* clear any stale error */
    void *handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        if (errbuf && errlen) snprintf(errbuf, errlen, "dlopen: %s", dlerror());
        return -1;
    }
    /* POSIX: cast through the object pointer to a function pointer. */
    int (*plugin_init)(const csim_api_t *) =
        (int (*)(const csim_api_t *))(uintptr_t)dlsym(handle, "csim_plugin_init");
    if (!plugin_init) {
        if (errbuf && errlen)
            snprintf(errbuf, errlen, "dlsym(csim_plugin_init): %s", dlerror());
        dlclose(handle);
        return -1;
    }
    csim_api_t api = {
        .version  = CSIM_PLUGIN_API_VERSION,
        .registry = &g_registry_ops,
        .log      = &g_log_ops,
        .reg      = reg,
    };
    int rc = plugin_init(&api);
    if (rc != 0) {
        /* Keep the handle loaded: the plugin may have registered services
         * whose ops point into this .so.  The caller skips the plugin's
         * services (it only attaches those registered without error). */
        if (errbuf && errlen)
            snprintf(errbuf, errlen, "csim_plugin_init returned %d", rc);
        return -1;
    }
    /* Success: leave the .so loaded for the process lifetime. */
    return 0;
}
