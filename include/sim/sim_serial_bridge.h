/*
 * sim_serial_bridge — TCP ↔ mote-UART byte plumbing as a kernel service.
 *
 * Phase 1 milestone 8 of docs/design/refactor-plan.md ("convert serial
 * socket … into scheduled kernel events", service split per §Phase 6's
 * serial-socket caveat).  This is the `serial_bridge_service` half: pure
 * byte plumbing between a TCP client (tunslip6, border-router test
 * drivers) and one bridged mote's console UART.  The external-command /
 * COOJA.testlog half stays in the runner until milestone 8.2
 * (`external_command_service`).
 *
 * Direction 1 — mote UART → TCP:
 *   The kernel emits SIM_OBS_MOTE_UART_BYTE for every console byte
 *   (mixed_uart_callback).  The bridge subscribes via
 *   sim_runtime_subscribe(), filters on its bridged mote index, and
 *   rings the byte for the next poll's TCP write.  Bytes arriving while
 *   no TCP client is connected are dropped — same contract as the
 *   pre-service serial_socket_uart_callback.
 *
 * Direction 2 — TCP → mote UART:
 *   sim_serial_bridge_poll() reads the socket into an RX ring, then
 *   drains it through the runner-provided inject callback.  Injection
 *   is mote-type-specific (native shared-memory serial buffer, MSP430
 *   USART with baud pacing, CC2538 UART) and needs CPU stepping, so it
 *   stays a callback until the mote vtable (Phase 2) gives it a proper
 *   home.  The callback consumes 0..len bytes and returns the count;
 *   returning < len means "mote not ready, retry next poll".
 *
 * The bridge never steps a CPU and never touches chip structs — the
 * §3.9 critical rule.  All sockets are non-blocking; poll() is safe to
 * call when inactive (no listener) or with no client connected.
 */
#ifndef SIM_SERIAL_BRIDGE_H
#define SIM_SERIAL_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "sim_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inject `buf[0..len)` into the bridged mote's serial input.  Returns the
 * number of bytes consumed (0 = mote busy, retry on a later poll).  The
 * runner supplies this; it may step the bridged mote's CPU for baud
 * pacing (legacy behavior preserved from ss_inject_serial). */
typedef int (*sim_serial_inject_fn)(void *user, const uint8_t *buf, int len);

/* Ring sizes match the pre-service SS_TX/RX_BUF_SIZE: border-router
 * tests burst multi-fragment SLIP traffic and need this much headroom
 * to avoid silently dropping bytes. */
#define SIM_SERIAL_BRIDGE_TX_BUF 65536   /* mote UART → TCP */
#define SIM_SERIAL_BRIDGE_RX_BUF 65536   /* TCP → mote UART */

typedef struct sim_serial_bridge {
    sim_runtime_t *sim;
    int   node_idx;        /* bridged mote index; -1 = unconfigured */
    int   listen_fd;       /* -1 = inactive                          */
    int   client_fd;       /* -1 = no client connected               */
    int   observer_handle; /* sim_runtime_subscribe() slot, -1 unset */

    sim_serial_inject_fn inject;
    void *inject_user;

    uint8_t tx_buf[SIM_SERIAL_BRIDGE_TX_BUF];
    int     tx_head, tx_count;
    uint8_t rx_buf[SIM_SERIAL_BRIDGE_RX_BUF];
    int     rx_head, rx_count;
} sim_serial_bridge_t;

/* Zero/fd-init only; no sockets opened.  Safe to call before start. */
void sim_serial_bridge_init(sim_serial_bridge_t *sb);

/* Open the loopback listener on `tcp_port`, subscribe to the runtime's
 * observer stream for `node_idx`'s UART bytes.  Returns 0 on success,
 * -1 on socket/bind/listen failure (bridge stays inactive). */
int sim_serial_bridge_start(sim_serial_bridge_t *sb, sim_runtime_t *sim,
                            int node_idx, int tcp_port,
                            sim_serial_inject_fn inject, void *inject_user);

/* One service tick: accept a pending client, flush the TX ring to the
 * socket, read the socket into the RX ring, drain RX through the inject
 * callback.  No-op when inactive. */
void sim_serial_bridge_poll(sim_serial_bridge_t *sb);

/* True while the listener is open (the bridge was started and not
 * stopped).  Used by the runner's loop-continuation condition. */
static inline bool sim_serial_bridge_active(const sim_serial_bridge_t *sb) {
    return sb->listen_fd >= 0;
}

/* Close sockets and unsubscribe from the observer stream. */
void sim_serial_bridge_stop(sim_serial_bridge_t *sb);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SERIAL_BRIDGE_H */
