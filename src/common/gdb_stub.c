/*
 * GDB Remote Serial Protocol stub — implementation.
 *
 * See include/common/gdb_stub.h for the public API and per-arch glue
 * model. This file deals only with the protocol, sockets, and the
 * command dispatch.
 */
#include "gdb_stub.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>

/* ============================================================
 * Hex helpers — GDB RSP encodes most binary as ASCII hex
 * ============================================================ */

static const char hex_digits[] = "0123456789abcdef";

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return -1;
}

static void byte_to_hex(uint8_t b, char *out) {
    out[0] = hex_digits[(b >> 4) & 0xf];
    out[1] = hex_digits[b & 0xf];
}

/* Decode a hex byte pair starting at p. Returns -1 on bad char. */
static int hex_to_byte(const char *p) {
    int hi = hex_value(p[0]);
    int lo = hex_value(p[1]);
    if (hi < 0 || lo < 0) return -1;
    return (hi << 4) | lo;
}

/* Parse a variable-length hex number, advancing *pp. Stops on first
 * non-hex char. Returns the value; *pp points at the stop char. */
static uint32_t parse_hex(const char **pp) {
    uint32_t v = 0;
    const char *p = *pp;
    while (*p) {
        int d = hex_value(*p);
        if (d < 0) break;
        v = (v << 4) | (uint32_t)d;
        p++;
    }
    *pp = p;
    return v;
}

/* ============================================================
 * Packet framing
 *
 * GDB RSP packet format:
 *   $<data>#<2-hex-checksum>
 * Checksum is the sum of <data> bytes mod 256.
 *
 * Receiver sends '+' to ack, '-' to request retransmit.
 * ============================================================ */

/* Send a fully formed reply packet. data is the payload (no $ or #). */
static int send_packet(gdb_stub_t *stub, const char *data) {
    if (stub->client_fd < 0) return -1;

    int len = (int)strlen(data);
    if (len + 5 > (int)sizeof(stub->tx_buf)) {
        fprintf(stderr, "gdb_stub: outgoing packet too large (%d bytes)\n", len);
        return -1;
    }

    /* Compute checksum */
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += (uint8_t)data[i];

    /* Frame: $<data>#XX */
    char *p = stub->tx_buf;
    *p++ = '$';
    memcpy(p, data, (size_t)len);
    p += len;
    *p++ = '#';
    byte_to_hex(sum, p);
    p += 2;

    int total = (int)(p - stub->tx_buf);
    int written = 0;
    while (written < total) {
        ssize_t n = write(stub->client_fd, stub->tx_buf + written, (size_t)(total - written));
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        written += (int)n;
    }

    /* Wait for + ack (single byte). We don't retry on - to keep things
     * simple — modern GDB rarely nacks unless the link is corrupting. */
    char ack;
    ssize_t n = read(stub->client_fd, &ack, 1);
    if (n != 1) return -1;
    return 0;
}

/* Send an OK reply (used after most write commands). */
static int send_ok(gdb_stub_t *stub) {
    return send_packet(stub, "OK");
}

/* Send an empty packet — means "command not supported". GDB will then
 * try alternative phrasings or fall back to defaults. */
static int send_empty(gdb_stub_t *stub) {
    return send_packet(stub, "");
}

/* Send an error reply: "EXX" where XX is a hex error number. */
static int send_error(gdb_stub_t *stub, int err) {
    char buf[8];
    snprintf(buf, sizeof(buf), "E%02x", err & 0xff);
    return send_packet(stub, buf);
}

/* Receive one packet body (without the $ ... # framing) into stub->rx_buf.
 * Returns the body length on success, -1 on disconnect, -2 on bad packet.
 * Also handles incoming '+'/'-' ack bytes (silently consumed) and the
 * Ctrl+C interrupt byte (0x03), which is reported as a special return. */
#define GDB_RX_INTERRUPT  -3

static int recv_packet(gdb_stub_t *stub) {
    if (stub->client_fd < 0) return -1;

    char c;
    /* Skip ack bytes and detect Ctrl+C */
    for (;;) {
        ssize_t n = read(stub->client_fd, &c, 1);
        if (n == 0) return -1;            /* peer closed */
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return -2;
            return -1;
        }
        if (c == '+' || c == '-') continue;
        if (c == 0x03) return GDB_RX_INTERRUPT;
        if (c == '$') break;
        /* Anything else: framing error, drop it */
    }

    /* Read body until '#', then 2 hex chars */
    int len = 0;
    uint8_t sum = 0;
    for (;;) {
        ssize_t n = read(stub->client_fd, &c, 1);
        if (n == 0) return -1;
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (c == '#') break;
        if (len >= (int)sizeof(stub->rx_buf) - 1) {
            return -2;  /* overflow */
        }
        stub->rx_buf[len++] = c;
        sum += (uint8_t)c;
    }
    stub->rx_buf[len] = '\0';
    stub->rx_len = len;

    /* Read 2 checksum chars */
    char ck[2];
    int got = 0;
    while (got < 2) {
        ssize_t n = read(stub->client_fd, ck + got, (size_t)(2 - got));
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return -1;
        }
        got += (int)n;
    }
    int expected = hex_to_byte(ck);
    if (expected < 0 || (uint8_t)expected != sum) {
        /* Bad checksum — nack and try again */
        ssize_t r = write(stub->client_fd, "-", 1);
        (void)r;
        return -2;
    }

    /* Send + ack */
    ssize_t r = write(stub->client_fd, "+", 1);
    (void)r;
    return len;
}

/* ============================================================
 * Command dispatch
 *
 * Each handler reads stub->rx_buf (the body without $...#XX) and
 * sends a reply via send_packet() / send_ok() / send_empty() /
 * send_error().
 * ============================================================ */

/* T05 stop reply: "S05" or "T05<key>:<val>;..." */
static void send_stop_reply(gdb_stub_t *stub, int signal) {
    char buf[16];
    snprintf(buf, sizeof(buf), "S%02x", signal & 0xff);
    send_packet(stub, buf);
}

/* qSupported — feature negotiation. Reply with the small set we handle. */
static void handle_qSupported(gdb_stub_t *stub) {
    /* PacketSize is the maximum incoming packet body we can handle. */
    char reply[128];
    snprintf(reply, sizeof(reply),
             "PacketSize=%x;swbreak+;hwbreak-",
             (unsigned)(GDB_PACKET_BUF_SIZE - 16));
    send_packet(stub, reply);
}

/* g — read all general-purpose registers as a hex blob */
static void handle_read_regs(gdb_stub_t *stub) {
    if (!stub->ops || !stub->ops->read_regs) { send_empty(stub); return; }

    uint8_t regs[256];  /* enough for ARM Cortex-M (17 * 4 = 68 bytes) */
    int n = stub->ops->read_regs(stub->cpu, regs, (int)sizeof(regs));
    if (n <= 0) { send_error(stub, 1); return; }

    /* Hex-encode */
    char hex[256 * 2 + 1];
    for (int i = 0; i < n; i++) {
        byte_to_hex(regs[i], hex + i * 2);
    }
    hex[n * 2] = '\0';
    send_packet(stub, hex);
}

/* G<hex> — write all general-purpose registers */
static void handle_write_regs(gdb_stub_t *stub) {
    if (!stub->ops || !stub->ops->write_regs) { send_empty(stub); return; }

    const char *p = stub->rx_buf + 1;  /* skip 'G' */
    int hex_len = stub->rx_len - 1;
    if (hex_len <= 0 || (hex_len & 1)) { send_error(stub, 1); return; }
    int n = hex_len / 2;
    if (n > 256) { send_error(stub, 2); return; }

    uint8_t regs[256];
    for (int i = 0; i < n; i++) {
        int b = hex_to_byte(p + i * 2);
        if (b < 0) { send_error(stub, 3); return; }
        regs[i] = (uint8_t)b;
    }
    stub->ops->write_regs(stub->cpu, regs, n);
    send_ok(stub);
}

/* m<addr>,<length> — read memory */
static void handle_read_mem(gdb_stub_t *stub) {
    if (!stub->ops || !stub->ops->read_mem) { send_empty(stub); return; }

    const char *p = stub->rx_buf + 1;  /* skip 'm' */
    uint32_t addr = parse_hex(&p);
    if (*p != ',') { send_error(stub, 1); return; }
    p++;
    uint32_t len = parse_hex(&p);
    if (len == 0) { send_packet(stub, ""); return; }
    if (len > 1024) len = 1024;

    uint8_t buf[1024];
    int n = stub->ops->read_mem(stub->cpu, addr, buf, (int)len);
    if (n <= 0) { send_error(stub, 2); return; }

    char hex[1024 * 2 + 1];
    for (int i = 0; i < n; i++) byte_to_hex(buf[i], hex + i * 2);
    hex[n * 2] = '\0';
    send_packet(stub, hex);
}

/* M<addr>,<length>:<hex> — write memory */
static void handle_write_mem(gdb_stub_t *stub) {
    if (!stub->ops || !stub->ops->write_mem) { send_empty(stub); return; }

    const char *p = stub->rx_buf + 1;  /* skip 'M' */
    uint32_t addr = parse_hex(&p);
    if (*p != ',') { send_error(stub, 1); return; }
    p++;
    uint32_t len = parse_hex(&p);
    if (*p != ':') { send_error(stub, 2); return; }
    p++;
    if (len > 1024) { send_error(stub, 3); return; }

    uint8_t buf[1024];
    for (uint32_t i = 0; i < len; i++) {
        int b = hex_to_byte(p + i * 2);
        if (b < 0) { send_error(stub, 4); return; }
        buf[i] = (uint8_t)b;
    }
    int n = stub->ops->write_mem(stub->cpu, addr, buf, (int)len);
    if (n != (int)len) { send_error(stub, 5); return; }
    send_ok(stub);
}

/* c[addr] — continue (optionally from addr) */
static void handle_continue(gdb_stub_t *stub) {
    const char *p = stub->rx_buf + 1;
    if (*p) {
        uint32_t addr = parse_hex(&p);
        if (stub->ops && stub->ops->set_pc)
            stub->ops->set_pc(stub->cpu, addr);
    }
    stub->halted = false;
    /* No reply now — send a stop reply when we hit a breakpoint or
     * receive Ctrl+C from GDB. */
}

/* s[addr] — single-step */
static void handle_step(gdb_stub_t *stub) {
    const char *p = stub->rx_buf + 1;
    if (*p) {
        uint32_t addr = parse_hex(&p);
        if (stub->ops && stub->ops->set_pc)
            stub->ops->set_pc(stub->cpu, addr);
    }
    /* Mark a single-step request: we'll execute exactly one instruction
     * from the sim loop, then re-halt. The poll loop checks this flag. */
    stub->halted = false;
    stub->stop_signal = 5;  /* SIGTRAP after the step completes */
    /* The architecture-side wrapper is responsible for stepping one
     * insn and then calling gdb_stub_notify_halt(stub, 5). */
}

/* Z0,addr,kind — set software breakpoint */
static void handle_set_bp(gdb_stub_t *stub) {
    if (stub->rx_buf[1] != '0') { send_empty(stub); return; }  /* only Z0 */
    const char *p = stub->rx_buf + 3;  /* skip "Z0," */
    uint32_t addr = parse_hex(&p);
    /* kind (length) ignored — we just match on PC */

    /* Already set? idempotent */
    for (int i = 0; i < stub->num_breakpoints; i++) {
        if (stub->breakpoints[i] == addr) { send_ok(stub); return; }
    }
    if (stub->num_breakpoints >= GDB_MAX_BREAKPOINTS) {
        send_error(stub, 1); return;
    }
    stub->breakpoints[stub->num_breakpoints++] = addr;
    send_ok(stub);
}

/* z0,addr,kind — clear software breakpoint */
static void handle_clear_bp(gdb_stub_t *stub) {
    if (stub->rx_buf[1] != '0') { send_empty(stub); return; }
    const char *p = stub->rx_buf + 3;
    uint32_t addr = parse_hex(&p);

    for (int i = 0; i < stub->num_breakpoints; i++) {
        if (stub->breakpoints[i] == addr) {
            stub->breakpoints[i] = stub->breakpoints[--stub->num_breakpoints];
            send_ok(stub);
            return;
        }
    }
    send_ok(stub);  /* clearing a non-existent bp is not an error */
}

/* D — detach */
static void handle_detach(gdb_stub_t *stub) {
    send_ok(stub);
    close(stub->client_fd);
    stub->client_fd = -1;
    stub->connected = false;
    stub->halted = false;
    stub->num_breakpoints = 0;
}

/* k — kill (we treat as detach + leave the simulator running) */
static void handle_kill(gdb_stub_t *stub) {
    close(stub->client_fd);
    stub->client_fd = -1;
    stub->connected = false;
    stub->halted = false;
    stub->num_breakpoints = 0;
}

/* Main command dispatcher: called for each received packet. */
static void dispatch(gdb_stub_t *stub) {
    if (stub->rx_len == 0) { send_empty(stub); return; }
    char cmd = stub->rx_buf[0];

    switch (cmd) {
    case '?':
        send_stop_reply(stub, stub->stop_signal ? stub->stop_signal : 5);
        break;
    case 'g': handle_read_regs(stub); break;
    case 'G': handle_write_regs(stub); break;
    case 'm': handle_read_mem(stub); break;
    case 'M': handle_write_mem(stub); break;
    case 'c': handle_continue(stub); break;
    case 's': handle_step(stub); break;
    case 'Z': handle_set_bp(stub); break;
    case 'z': handle_clear_bp(stub); break;
    case 'D': handle_detach(stub); break;
    case 'k': handle_kill(stub); break;
    case 'q':
        if (strncmp(stub->rx_buf, "qSupported", 10) == 0) {
            handle_qSupported(stub);
        } else if (strcmp(stub->rx_buf, "qAttached") == 0) {
            send_packet(stub, "1");
        } else if (strcmp(stub->rx_buf, "qC") == 0) {
            send_packet(stub, "QC1");  /* current thread = 1 */
        } else if (stub->rx_buf[1] == 'f' || stub->rx_buf[1] == 's') {
            /* qfThreadInfo / qsThreadInfo */
            if (strncmp(stub->rx_buf, "qfThreadInfo", 12) == 0)
                send_packet(stub, "m1");
            else
                send_packet(stub, "l");
        } else {
            send_empty(stub);
        }
        break;
    case 'H':
        /* Hg0 / Hc-1 — set thread for subsequent operations. We always say OK. */
        send_ok(stub);
        break;
    case 'v':
        if (strcmp(stub->rx_buf, "vCont?") == 0) {
            send_packet(stub, "vCont;c;s");
        } else if (strncmp(stub->rx_buf, "vCont;c", 7) == 0) {
            handle_continue(stub);
        } else if (strncmp(stub->rx_buf, "vCont;s", 7) == 0) {
            handle_step(stub);
        } else {
            send_empty(stub);
        }
        break;
    default:
        send_empty(stub);
        break;
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

int gdb_stub_init(gdb_stub_t *stub, int port) {
    memset(stub, 0, sizeof(*stub));
    stub->client_fd = -1;
    stub->port = port;
    stub->stop_signal = 5;

    stub->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (stub->listen_fd < 0) {
        perror("gdb_stub: socket");
        return -1;
    }
    int yes = 1;
    setsockopt(stub->listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);
    if (bind(stub->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("gdb_stub: bind");
        close(stub->listen_fd);
        stub->listen_fd = -1;
        return -1;
    }
    if (listen(stub->listen_fd, 1) < 0) {
        perror("gdb_stub: listen");
        close(stub->listen_fd);
        stub->listen_fd = -1;
        return -1;
    }

    /* Non-blocking accept */
    int flags = fcntl(stub->listen_fd, F_GETFL, 0);
    fcntl(stub->listen_fd, F_SETFL, flags | O_NONBLOCK);

    fprintf(stderr, "gdb_stub: listening on 127.0.0.1:%d\n", port);
    return 0;
}

void gdb_stub_attach(gdb_stub_t *stub, void *cpu, const gdb_arch_ops_t *ops) {
    stub->cpu = cpu;
    stub->ops = ops;
}

static int try_accept(gdb_stub_t *stub) {
    if (stub->listen_fd < 0 || stub->connected) return 0;

    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);
    int fd = accept(stub->listen_fd, (struct sockaddr *)&peer, &len);
    if (fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        perror("gdb_stub: accept");
        return -1;
    }
    /* Disable Nagle so single-byte ack/packets aren't delayed */
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    stub->client_fd = fd;
    stub->connected = true;
    /* On connect, halt the CPU so the developer can set breakpoints
     * before anything runs. */
    stub->halted = true;
    stub->stop_signal = 5;
    fprintf(stderr, "gdb_stub: client connected on port %d\n", stub->port);
    return 1;
}

int gdb_stub_wait_for_client(gdb_stub_t *stub) {
    /* Switch listen_fd to blocking for the wait, then back. */
    int flags = fcntl(stub->listen_fd, F_GETFL, 0);
    fcntl(stub->listen_fd, F_SETFL, flags & ~O_NONBLOCK);
    int r = try_accept(stub);
    fcntl(stub->listen_fd, F_SETFL, flags);
    return r > 0 ? 0 : -1;
}

bool gdb_stub_poll(gdb_stub_t *stub) {
    if (!stub->connected) try_accept(stub);
    if (!stub->connected) return false;

    /* Drain all pending packets. When halted, block on the read; when
     * running, use select() with zero timeout so we don't slow the sim. */
    for (;;) {
        if (!stub->halted) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(stub->client_fd, &rfds);
            struct timeval tv = {0, 0};
            int n = select(stub->client_fd + 1, &rfds, NULL, NULL, &tv);
            if (n <= 0) break;
        }
        int r = recv_packet(stub);
        if (r == -1) {
            /* Disconnect */
            close(stub->client_fd);
            stub->client_fd = -1;
            stub->connected = false;
            stub->halted = false;
            return false;
        }
        if (r == -2) continue;  /* bad packet, retry */
        if (r == GDB_RX_INTERRUPT) {
            /* Ctrl+C from GDB while running */
            stub->halted = true;
            stub->stop_signal = 2;  /* SIGINT */
            send_stop_reply(stub, 2);
            continue;
        }
        dispatch(stub);
        if (!stub->halted) {
            /* Continue / vCont;c — return to sim loop without further blocking */
            break;
        }
    }
    return stub->halted;
}

bool gdb_stub_check_breakpoint(gdb_stub_t *stub, uint32_t pc) {
    if (!stub->connected || stub->num_breakpoints == 0) return false;
    for (int i = 0; i < stub->num_breakpoints; i++) {
        if (stub->breakpoints[i] == pc) {
            stub->halted = true;
            stub->stop_signal = 5;
            send_stop_reply(stub, 5);
            return true;
        }
    }
    return false;
}

void gdb_stub_notify_halt(gdb_stub_t *stub, int signal) {
    if (!stub->connected) return;
    stub->halted = true;
    stub->stop_signal = signal;
    send_stop_reply(stub, signal);
}

void gdb_stub_destroy(gdb_stub_t *stub) {
    if (stub->client_fd >= 0) close(stub->client_fd);
    if (stub->listen_fd >= 0) close(stub->listen_fd);
    stub->client_fd = -1;
    stub->listen_fd = -1;
    stub->connected = false;
}
