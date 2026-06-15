/*
 * sim_registry — generic registry container + the service catalog.
 *
 * See include/sim/sim_registry.h.  Board glue lives in sim_board.c and
 * mote-kind glue in mote_kinds.c (each domain registers itself and implements
 * its own forwarding accessor); this file owns the name → ops service catalog
 * and the built-in service registration.
 */
#include "sim_registry.h"

#include <string.h>

int sim_registry_register_service(sim_registry_t *r,
                                  const sim_service_ops_t *ops) {
    if (!r || !ops || !ops->name)
        return -1;
    /* Reject a duplicate name (the catalog is a set keyed by name) and a full
     * table. */
    if (sim_registry_find_service(r, ops->name))
        return -1;
    if (r->service_count >= SIM_REGISTRY_MAX_SERVICES)
        return -1;
    int idx = r->service_count++;
    r->services[idx] = ops;
    return idx;
}

const sim_service_ops_t *sim_registry_find_service(const sim_registry_t *r,
                                                   const char *name) {
    if (!r || !name)
        return NULL;
    for (int i = 0; i < r->service_count; i++) {
        const sim_service_ops_t *ops = r->services[i];
        if (ops && ops->name && strcmp(ops->name, name) == 0)
            return ops;
    }
    return NULL;
}
