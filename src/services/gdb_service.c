/*
 * Implementation of the GDB stub service — see include/sim/gdb_service.h.
 *
 * Phase 6 milestone 37 (§3.19).  Verbatim move of the runner's GDB
 * bind+attach loop and teardown; the per-tick poll moved into the ARM
 * mote's execute (consulting cpu->gdb_stub), which this service sets.
 */
#include "gdb_service.h"
#include "sim_runtime.h"
#include "sim_mote.h"     /* SIM_MOTE_IFACE_ARM_CPU */
#include "arm_cpu.h"      /* arm_cpu_t (cpu->gdb_stub) */
#include "arm_gdb.h"      /* arm_gdb_ops */

#include <stdio.h>
#include <stddef.h>

void gdb_service_attach_all(gdb_service_t *svc, sim_runtime_t *sim,
                            int node_count, gdb_node_id_fn node_id) {
    bool any_gdb = false;
    for (int i = 0; i < node_count; i++) {
        if (svc->port[i] == 0) continue;
        /* Ask the mote for its ARM CPU instead of testing type (M16). */
        sim_mote_t *m = sim_runtime_mote(sim, i);
        arm_cpu_t *gdb_cpu = m ? (arm_cpu_t *)m->ops->get_interface(
            m, SIM_MOTE_IFACE_ARM_CPU) : NULL;
        if (!gdb_cpu) {
            fprintf(stderr, "  Node %d: --gdb only supports ARM nodes for now (skipped)\n",
                    node_id(i));
            svc->port[i] = 0;
            continue;
        }
        if (gdb_stub_init(&svc->stubs[i], svc->port[i]) != 0) {
            fprintf(stderr, "  Node %d: failed to bind GDB stub on port %d\n",
                    node_id(i), svc->port[i]);
            svc->port[i] = 0;
            continue;
        }
        gdb_stub_attach(&svc->stubs[i], gdb_cpu, &arm_gdb_ops);
        gdb_cpu->gdb_stub = &svc->stubs[i];
        any_gdb = true;
        printf("  Node %d: GDB stub on port %d (connect with: target remote :%d)\n",
               node_id(i), svc->port[i], svc->port[i]);
    }
    if (any_gdb && svc->wait) {
        for (int i = 0; i < node_count; i++) {
            if (svc->port[i] == 0) continue;
            printf("  Node %d: waiting for GDB on port %d ...\n",
                   node_id(i), svc->port[i]);
            if (gdb_stub_wait_for_client(&svc->stubs[i]) != 0)
                fprintf(stderr, "  Node %d: GDB wait failed\n", node_id(i));
        }
    }
}

static int gdb_service_init(sim_runtime_t *sim, const void *cfg, void **state) {
    (void)sim;
    *state = (void *)cfg;   /* runner-owned gdb_service_t */
    return 0;
}

static void gdb_service_destroy(sim_runtime_t *sim, void *state) {
    (void)sim;
    gdb_service_t *svc = (gdb_service_t *)state;
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        if (svc->port[i] != 0)
            gdb_stub_destroy(&svc->stubs[i]);  /* silent: closes sockets */
}

const sim_service_ops_t gdb_service_ops = {
    .name    = "gdb",
    .init    = gdb_service_init,
    .destroy = gdb_service_destroy,
};
