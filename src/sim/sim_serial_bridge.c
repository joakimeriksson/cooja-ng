/*
 * sim_serial_bridge — TCP ↔ mote-UART byte plumbing.
 * See include/sim/sim_serial_bridge.h for the contract and
 * docs/design/refactor-plan.md §Phase 6 (serial-socket caveat) for why
 * this service owns sockets+rings but not injection or child-process
 * management.
 */
#include "sim_serial_bridge.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

/* ============================================================
 * HOST-LINK LATENCY  —  WORKAROUND, REMOVE WHEN SLIP GETS FLOW CONTROL
 * ============================================================
 *
 * We delay mote->socket delivery by a few tens of milliseconds of
 * WALL-CLOCK time.  This is not a performance knob; without it the
 * Contiki native border router (`border-router.native`) deadlocks
 * against us, and `17-tun-rpl-br/09-native-border-router-cooja-frag`
 * fails with 100% packet loss.
 *
 * Why the peer needs us to be slow
 * -------------------------------
 * SLIP has no flow control at all — no window, no ACK, no back-pressure.
 * Contiki substitutes a fixed inter-packet delay,
 * `SLIP_DEV_CONF_SEND_DELAY = CLOCK_SECOND/32` = 31 ms
 * (os/services/rpl-border-router/native/module-macros.h), and drains its
 * queue exactly one SLIP packet per `slip_flushbuf()`.  Its `set_fd()`
 * refuses to arm the write fd until that timer expires.
 *
 * The timer is a plain `struct timer` — a passive stopwatch that posts no
 * event — and `arch/platform/native/platform.c` gives `select()` a flat
 * `SELECT_TIMEOUT` of 1000 ms when Contiki is otherwise idle.  So while
 * the router sits in its 31 ms window, *nothing* is scheduled to wake it
 * at the 31 ms mark: the only early wake-up available is one of our bytes
 * arriving.
 *
 * A 1200-byte ping is fragmented host-side into 13 SLIP packets queued in
 * one unpaced burst (sicslowpan `fragment_copy_payload_and_send`).  If our
 * reply to fragment N lands *inside* the 31 ms window, that single early
 * wake-up is consumed while flushing is still forbidden, and the loop then
 * sleeps the full second — one fragment per second.  The queue then grows
 * faster than it drains (ping6 re-injects every 4 s) until
 * `slip_send overflow` calls `err(EXIT_FAILURE)` and the router *exits*.
 *
 * Measured, same instrumented border router under both simulators:
 *
 *   flush -> reply latency     max      >31 ms     drain/fragment
 *   csim   0.2 .. 26.5 ms      26.5      0 / 13     1001 ms  (dies)
 *   Cooja  0.0 .. 47.3 ms      47.3      5 / 14     32-59 ms (passes)
 *
 * Cooja is not correct here, merely lucky: its wall-clock jitter straddles
 * the 31 ms threshold so it wins the race ~36% of the time (and it stalls
 * for a full second whenever it loses).  csim's link is tight and fast, so
 * it loses *every* time.  Protocol traffic is identical — one 7-8 byte
 * `!R` confirmation per fragment in both.
 *
 * What we model
 * -------------
 * Neither simulator models host-link latency, and a real slip-radio is a
 * USB-CDC dongle: 1 ms USB frames, an FTDI-style 16 ms latency timer, plus
 * OS scheduling.  Tens of milliseconds is the physical case; our ~0 ms is
 * the unphysical one.  That latency is precisely why Contiki's 31 ms
 * pacing constant works on real hardware.
 *
 * WALL-clock, not sim-time, because the peer is a real OS process running
 * against `gettimeofday`/`select`.  (Serial-socket runs are pinned to
 * speed 1.0 by tools/csc2json.py, so the two coincide today; they would
 * not at any other speed.)  The gate is inert unless a serial socket is
 * actually bridged.
 *
 * REMOVING THIS
 * -------------
 * Delete `tx_latency_ms()`, the `tx_release_ms` field, and the two call
 * sites below.  It is safe to remove once EITHER holds:
 *
 *   1. The serial link gains real flow control — an explicit
 *      ready/credit handshake between host and radio, so the peer stops
 *      depending on a fixed delay and on wake-up timing; or
 *   2. Contiki's native platform derives its `select()` timeout from
 *      `etimer_next_expiration_time()` instead of a flat 1000 ms, AND
 *      `slip-dev.c` uses a timer the platform can observe — at which
 *      point the router wakes at 31 ms on its own regardless of peer
 *      speed.  (Worth reporting upstream; it would fix a whole class of
 *      native-platform latency bugs, not just this one.)
 *
 * Until then, removing it re-breaks 09-native-border-router-cooja-frag.
 */
#define SIM_SERIAL_TX_LATENCY_MS_DEFAULT 40   /* > the peer's 31 ms window */

static double mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1.0e6;
}

/* 0 disables the gate entirely (CSIM_SERIAL_TX_LATENCY_MS=0). */
static double tx_latency_ms(void) {
    static double v = -1.0;
    if (v < 0.0) {
        const char *e = getenv("CSIM_SERIAL_TX_LATENCY_MS");
        v = e ? atof(e) : (double)SIM_SERIAL_TX_LATENCY_MS_DEFAULT;
        if (v < 0.0) v = 0.0;
    }
    return v;
}

/* Observer tap: ring every UART byte the bridged mote emits while a TCP
 * client is connected.  Other motes' bytes and other event kinds pass
 * through untouched. */
static void bridge_observer(void *user, const sim_observer_event_t *ev) {
    sim_serial_bridge_t *sb = (sim_serial_bridge_t *)user;
    if (ev->kind != SIM_OBS_MOTE_UART_BYTE) return;
    if (ev->mote_index != sb->node_idx) return;
    if (sb->client_fd < 0) return;  /* drop when no client — legacy contract */
    if (sb->tx_count >= SIM_SERIAL_BRIDGE_TX_BUF) return;  /* ring full */
    /* Host-link latency: stamp the release time on the first byte of a
     * burst, so a burst is delayed as a unit rather than per byte.  A
     * continuous stream keeps a non-empty ring and so is delayed once,
     * then flows at line rate — it cannot starve. */
    if (sb->tx_count == 0)
        sb->tx_release_ms = mono_ms() + tx_latency_ms();
    int tail = (sb->tx_head + sb->tx_count) % SIM_SERIAL_BRIDGE_TX_BUF;
    sb->tx_buf[tail] = ev->u.uart.byte;
    sb->tx_count++;
}

void sim_serial_bridge_init(sim_serial_bridge_t *sb) {
    memset(sb, 0, sizeof(*sb));
    sb->node_idx = -1;
    sb->listen_fd = -1;
    sb->client_fd = -1;
    sb->observer_handle = -1;
}

static int create_listener(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("serial_bridge: socket");
        return -1;
    }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("serial_bridge: bind");
        close(fd);
        return -1;
    }
    if (listen(fd, 1) < 0) {
        perror("serial_bridge: listen");
        close(fd);
        return -1;
    }

    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

    printf("  Serial socket listening on port %d\n", port);
    return fd;
}

int sim_serial_bridge_start(sim_serial_bridge_t *sb, sim_runtime_t *sim,
                            int node_idx, int tcp_port,
                            sim_serial_inject_fn inject, void *inject_user) {
    sim_serial_bridge_init(sb);
    sb->sim = sim;
    sb->node_idx = node_idx;
    sb->inject = inject;
    sb->inject_user = inject_user;

    sb->listen_fd = create_listener(tcp_port);
    if (sb->listen_fd < 0)
        return -1;

    sb->observer_handle = sim_runtime_subscribe(sim, bridge_observer, sb);
    return 0;
}

static void bridge_accept(sim_serial_bridge_t *sb) {
    if (sb->listen_fd < 0 || sb->client_fd >= 0) return;

    int fd = accept(sb->listen_fd, NULL, NULL);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("serial_bridge: accept");
        return;
    }

    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    sb->client_fd = fd;
    printf("  Serial socket: client connected\n");
}

static void bridge_flush_tx(sim_serial_bridge_t *sb) {
    if (sb->client_fd < 0 || sb->tx_count == 0) return;
    /* Host-link latency gate — see "HOST-LINK LATENCY" above.  Remove
     * together with tx_latency_ms()/tx_release_ms once the serial link
     * has real flow control. */
    if (tx_latency_ms() > 0.0 && mono_ms() < sb->tx_release_ms) return;

    while (sb->tx_count > 0) {
        /* Contiguous chunk from head */
        int chunk = sb->tx_count;
        if (sb->tx_head + chunk > SIM_SERIAL_BRIDGE_TX_BUF)
            chunk = SIM_SERIAL_BRIDGE_TX_BUF - sb->tx_head;

        ssize_t n = write(sb->client_fd, sb->tx_buf + sb->tx_head, (size_t)chunk);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                break;
            printf("  Serial socket: client disconnected (write)\n");
            close(sb->client_fd);
            sb->client_fd = -1;
            sb->tx_count = 0;
            sb->tx_head = 0;
            sb->tx_release_ms = 0.0;
            return;
        }
        sb->tx_head = (sb->tx_head + (int)n) % SIM_SERIAL_BRIDGE_TX_BUF;
        sb->tx_count -= (int)n;
    }
}

static void bridge_read_tcp(sim_serial_bridge_t *sb) {
    if (sb->client_fd < 0) return;

    while (sb->rx_count < SIM_SERIAL_BRIDGE_RX_BUF) {
        int tail = (sb->rx_head + sb->rx_count) % SIM_SERIAL_BRIDGE_RX_BUF;
        int space = SIM_SERIAL_BRIDGE_RX_BUF - sb->rx_count;
        /* Contiguous chunk from tail */
        int chunk = SIM_SERIAL_BRIDGE_RX_BUF - tail;
        if (chunk > space) chunk = space;

        ssize_t n = read(sb->client_fd, sb->rx_buf + tail, (size_t)chunk);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            printf("  Serial socket: client disconnected (read)\n");
            close(sb->client_fd);
            sb->client_fd = -1;
            return;
        }
        sb->rx_count += (int)n;
        break;  /* one read per poll */
    }
}

static void bridge_drain_rx(sim_serial_bridge_t *sb) {
    if (!sb->inject || sb->node_idx < 0) return;

    while (sb->rx_count > 0) {
        /* Contiguous chunk from head */
        int chunk = sb->rx_count;
        if (sb->rx_head + chunk > SIM_SERIAL_BRIDGE_RX_BUF)
            chunk = SIM_SERIAL_BRIDGE_RX_BUF - sb->rx_head;

        int consumed = sb->inject(sb->inject_user,
                                  sb->rx_buf + sb->rx_head, chunk);
        if (consumed <= 0) break;          /* mote busy — retry next poll  */
        if (consumed > chunk) consumed = chunk;
        sb->rx_head = (sb->rx_head + consumed) % SIM_SERIAL_BRIDGE_RX_BUF;
        sb->rx_count -= consumed;
        if (consumed < chunk) break;       /* injector stalled mid-chunk   */
    }
}

void sim_serial_bridge_poll(sim_serial_bridge_t *sb) {
    if (sb->listen_fd < 0) return;
    bridge_accept(sb);
    bridge_flush_tx(sb);
    bridge_read_tcp(sb);
    bridge_drain_rx(sb);
}

void sim_serial_bridge_stop(sim_serial_bridge_t *sb) {
    if (sb->client_fd >= 0) { close(sb->client_fd); sb->client_fd = -1; }
    if (sb->listen_fd >= 0) { close(sb->listen_fd); sb->listen_fd = -1; }
    if (sb->observer_handle >= 0 && sb->sim) {
        sim_runtime_unsubscribe(sb->sim, sb->observer_handle);
        sb->observer_handle = -1;
    }
}
