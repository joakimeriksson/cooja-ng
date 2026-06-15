/*
 * sim_registry — generic registry container + the service catalog.
 *
 * See include/sim/sim_registry.h.  Board glue lives in sim_board.c and
 * mote-kind glue in mote_kinds.c (each domain registers itself and implements
 * its own forwarding accessor); this file owns the name → ops service catalog
 * and the built-in service registration.
 */
#include "sim_registry.h"

#include "timeline_service.h"     /* timeline_service_ops   */
#include "pcap_service.h"         /* pcap_service_ops       */
#include "progress_service.h"     /* progress_service_ops   */
#include "json_test_service.h"    /* json_test_service_ops  */
#include "js_test_service.h"      /* js_test_service_ops    */
#include "gdb_service.h"          /* gdb_service_ops        */

#include <string.h>

/* Register the built-in library services that ship as exported ops structs.
 * The runner registers its two file-local serial-socket services
 * (serial-bridge, external-command) on top of these.  Registration order is
 * not significant — the catalog is keyed by name, and attach order (= the
 * service host's fan-out order) is decided at the attach sites, not here. */
void csim_register_builtin_services(sim_registry_t *r) {
    if (!r) return;
    sim_registry_register_service(r, &timeline_service_ops);
    sim_registry_register_service(r, &pcap_service_ops);
    sim_registry_register_service(r, &progress_service_ops);
    sim_registry_register_service(r, &json_test_service_ops);
    sim_registry_register_service(r, &js_test_service_ops);
    sim_registry_register_service(r, &gdb_service_ops);
}

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
