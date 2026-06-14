/*
 * gdb_service — the per-mote GDB remote-serial-protocol stub as a kernel
 * service.
 *
 * Phase 6 milestone 37 (§3.19).  Owns the per-node gdb_stub_t storage, the
 * `--gdb [node:]port` / `--gdb-wait` configuration, and the bind+attach
 * sequence the runner used to keep as file-scope globals.  Crucially it
 * sets each tagged ARM CPU's `gdb_stub` back-pointer, so the per-tick poll
 * in the ARM mote's execute consults `cpu->gdb_stub` rather than the
 * runner's `gdb_node[]`/`gdb_stubs[]` — that decoupling is what lets the
 * MSP430/ARM execute/serial adapters finally leave the runner in M38.
 *
 * ARM-only for now (matching the prior runner behavior): attach asks each
 * mote for its ARM CPU via get_interface(SIM_MOTE_IFACE_ARM_CPU) and uses
 * the arm_gdb arch vtable; a future MSP430 GDB would generalize the arch
 * vtable lookup through the mote interface.
 *
 * No host poll() / keepalive: the stub is polled from inside the ARM
 * execute slice (a halted CPU returns +1µs wakeups, so it stays scheduled
 * and polled), and --gdb-wait blocks during attach before the run loop —
 * so the loop's termination at end_ns is unchanged.
 */
#ifndef GDB_SERVICE_H
#define GDB_SERVICE_H

#include <stdbool.h>

#include "sim_service.h"
#include "sim_event_queue.h"   /* SIM_EQ_MAX_NODES */
#include "gdb_stub.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sim_runtime;

/* Resolve a mote slot index to its Cooja node id (for the status prints). */
typedef int (*gdb_node_id_fn)(int mote_index);

typedef struct gdb_service {
    gdb_stub_t stubs[SIM_EQ_MAX_NODES];
    int        port[SIM_EQ_MAX_NODES];  /* TCP port per slot, 0 = no stub */
    bool       wait;                    /* --gdb-wait: block on first connect */
} gdb_service_t;

/* Bind a listener for each tagged node, attach the ARM gdb vtable to its
 * CPU (set via get_interface), store the stub on cpu->gdb_stub, print the
 * connect hint, and — if `wait` — block until each client connects.  Nodes
 * whose CPU is not ARM, or whose bind fails, are dropped (port → 0). */
void gdb_service_attach_all(gdb_service_t *svc, struct sim_runtime *sim,
                            int node_count, gdb_node_id_fn node_id);

/* True if any node was configured with a stub (drives whether to attach). */
static inline bool gdb_service_configured(const gdb_service_t *svc) {
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        if (svc->port[i] != 0) return true;
    return false;
}

/* Host vtable: init adopts the runner-owned service; destroy closes every
 * stub's sockets (silent). */
extern const sim_service_ops_t gdb_service_ops;

#ifdef __cplusplus
}
#endif

#endif /* GDB_SERVICE_H */
