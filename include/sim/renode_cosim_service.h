/*
 * renode_cosim_service — csim as a clock SLAVE to Renode.
 *
 * Everywhere else in csim, csim owns the simulation clock: the runner picks
 * a horizon and the kernel pump runs to it.  This service inverts that for
 * one case.  Renode's CoSimulationPlugin drives an external simulator with
 * fixed-quantum `tickClock` messages, and this service turns each of those
 * into the runner's next horizon, so Renode's virtual time and csim's
 * advance together.  From Renode's side csim is an ordinary co-simulated
 * peripheral — a `CoSimulated.CoSimulatedPeripheral` line in a .repl, no
 * Renode-side code — whose register window is csim's entire 802.15.4
 * network.
 *
 * Three pieces, deliberately separate:
 *   - include/common/renode_proto.h  the 24-byte wire codec
 *   - include/native/renode_dev.h    the register window and its FIFOs
 *   - this file                      the sockets, the protocol loop, and
 *                                    the horizon the runner asks for
 *
 * The runner seam is two calls: renode_cosim_active() keeps the loop alive
 * while a master is attached, and renode_cosim_next_horizon() supplies the
 * horizon in place of the runner's own computation.  Everything else about
 * the loop is unchanged, and with no master attached nothing in it is
 * touched at all.
 *
 * Ordering rules that matter (docs/design/renode-cosim-plan.md §Limitations):
 *   - Bus accesses take zero simulation time and are serviced at the
 *     boundary of the last completed quantum.  The service pins
 *     sim->now_ns there first, so a frame the guest injects is stamped at
 *     the tick boundary rather than at whatever event the pump last
 *     dispatched.
 *   - Console lines and interrupt-level changes seen from observer
 *     callbacks are buffered, not written to the socket, because observers
 *     must not block.  They are flushed on the async socket at the quantum
 *     boundary, immediately before the tickClock reply.
 */
#ifndef RENODE_COSIM_SERVICE_H
#define RENODE_COSIM_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "sim_runtime.h"
#include "sim_service.h"

#ifdef __cplusplus
extern "C" {
#endif

struct renode_dev;

/* Environment overrides, all optional except CSIM_RENODE itself:
 *   CSIM_RENODE                  "ADDR:MAIN:ASYNC"
 *   CSIM_RENODE_FREQ_HZ          tick frequency (default 1000000)
 *   CSIM_RENODE_CONNECT_TIMEOUT_MS   default 10000
 *   CSIM_RENODE_TIMEOUT_MS       wait for a tick; 0 = forever (default)
 *   CSIM_RENODE_LOG_LEVEL        Renode LogLevel for forwarded lines
 */
typedef struct renode_cosim_config {
    char     host[64];
    int      main_port;
    int      async_port;
    uint64_t freq_hz;            /* ticks per simulated second           */
    int      log_level;          /* Renode LogLevel for forwarded lines  */
    int      connect_timeout_ms;
    int      wait_timeout_ms;    /* 0 = wait forever for the next tick   */
} renode_cosim_config_t;

typedef struct renode_cosim_service {
    renode_cosim_config_t cfg;
    struct sim_runtime   *sim;

    int   main_fd;
    int   async_fd;
    bool  active;
    bool  owned;                 /* allocated here (env path), not adopted */

    struct renode_dev *dev;
    int   dev_slot;

    /* Time base.  The horizon is recomputed from the running tick total
     * rather than accumulated per tick, so a frequency that does not
     * divide a second exactly cannot drift over a long run. */
    uint64_t total_ticks;
    int64_t  base_ns;
    bool     base_set;

    bool  tick_outstanding;      /* a tickClock reply is owed to Renode  */
    bool  irq_level_sent;        /* last level reported on the async socket */

    /* Console lines waiting to become logMessage frames. */
    struct { char *buf; size_t len, cap; uint32_t dropped; } logq;

    struct { uint32_t ticks, reads, writes, irqs, logs; } stats;

    /* Installed on the runtime while attached, so the runner's loop takes
     * its horizon through the kernel's generic hook rather than by naming
     * this service. */
    sim_clock_source_t clock_source;
} renode_cosim_service_t;

/* Registered in sim_registry.c as the built-in service "renode". */
extern const sim_service_ops_t renode_cosim_service_ops;

/* Parse "ADDR:MAIN:ASYNC" (the order Renode's SimulationContext supplies as
 * {2}:{0}:{1}).  Returns 0 on success, -1 on a malformed spec. */
int  renode_cosim_config_parse(renode_cosim_config_t *c, const char *spec);

/* Fill from the environment.  Returns 0 on success, -1 when CSIM_RENODE is
 * unset or malformed. */
int  renode_cosim_config_from_env(renode_cosim_config_t *c);

/* True while a master is attached — the runner's loop stays alive on this. */
bool renode_cosim_active(const renode_cosim_service_t *s);

/* The attached instance, or NULL.  At most one exists per process, because
 * the device node it drives has to be unique.  The runner asks for it rather
 * than holding its own pointer, so both ways of attaching behave the same:
 * the --renode flag, which hands over a struct, and a config's
 * plugins: ["renode"], where the service allocates its own. */
renode_cosim_service_t *renode_cosim_current(void);

/* Supply the next horizon.
 *
 * Flushes anything owed to Renode (buffered log lines, an interrupt-level
 * change, the reply for the quantum that just finished), then blocks on the
 * main socket servicing bus accesses at `cur_ns` until the next tickClock
 * arrives.  Returns the new horizon (>= cur_ns), or -1 when the master
 * disconnects, the connection fails, or the wait times out — in which case
 * the service marks itself inactive and the runner ends the run. */
int64_t renode_cosim_next_horizon(renode_cosim_service_t *s, int64_t cur_ns);

/* Test seam: adopt two already-connected sockets instead of connecting, then
 * run the same discovery + handshake an attach would.  Returns 0 or -1. */
int  renode_cosim_attach_fds(renode_cosim_service_t *s, struct sim_runtime *sim,
                             int main_fd, int async_fd);

#ifdef __cplusplus
}
#endif

#endif /* RENODE_COSIM_SERVICE_H */
