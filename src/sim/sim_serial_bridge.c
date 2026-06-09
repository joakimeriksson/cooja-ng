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
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/* Observer tap: ring every UART byte the bridged mote emits while a TCP
 * client is connected.  Other motes' bytes and other event kinds pass
 * through untouched. */
static void bridge_observer(void *user, const sim_observer_event_t *ev) {
    sim_serial_bridge_t *sb = (sim_serial_bridge_t *)user;
    if (ev->kind != SIM_OBS_MOTE_UART_BYTE) return;
    if (ev->mote_index != sb->node_idx) return;
    if (sb->client_fd < 0) return;  /* drop when no client — legacy contract */
    if (sb->tx_count >= SIM_SERIAL_BRIDGE_TX_BUF) return;  /* ring full */
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
