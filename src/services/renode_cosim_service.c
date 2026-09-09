/*
 * renode_cosim_service — csim as a clock slave to Renode.
 * See include/sim/renode_cosim_service.h for the contract and the rules.
 */
#include "renode_cosim_service.h"

#include "renode_proto.h"
#include "renode_dev.h"
#include "sim_runtime.h"
#include "sim_mote.h"

#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define LOGQ_CAP_MAX (64 * 1024)

/* The one attached instance (see renode_cosim_current).  A process has at
 * most one, because the device node it drives must be unique. */
static renode_cosim_service_t *g_attached = NULL;

/* CSIM_RENODE_TRACE=1: one line per protocol step, with wall-clock time.
 * The failure mode this exists for is a stall — either side waiting for the
 * other — and the only way to tell which is to see who spoke last. */
static int trace_on = -1;
static bool trace_enabled(void) {
    if (trace_on < 0) {
        const char *e = getenv("CSIM_RENODE_TRACE");
        trace_on = (e && e[0] && strcmp(e, "0") != 0) ? 1 : 0;
    }
    return trace_on != 0;
}

static void trace(const char *fmt, ...) {
    if (!trace_enabled()) return;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    fprintf(stderr, "[renode-trace %ld.%06d] ", (long)tv.tv_sec, (int)tv.tv_usec);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    fflush(stderr);
}

renode_cosim_service_t *renode_cosim_current(void) { return g_attached; }

/* ============================================================
 * Configuration
 * ============================================================ */

static void config_defaults(renode_cosim_config_t *c) {
    memset(c, 0, sizeof(*c));
    snprintf(c->host, sizeof(c->host), "127.0.0.1");
    c->freq_hz            = 1000000;   /* 1 MHz: 1 µs per tick */
    c->log_level          = RENODE_LOG_INFO;
    c->connect_timeout_ms = 10000;
    c->wait_timeout_ms    = 0;         /* Renode may sit paused in its monitor */
}

int renode_cosim_config_parse(renode_cosim_config_t *c, const char *spec) {
    if (!c || !spec || !*spec)
        return -1;
    config_defaults(c);

    /* "ADDR:MAIN:ASYNC" — the order Renode's SimulationContext hands over as
     * {2}:{0}:{1}.  Split on the last two colons so an IPv6 literal or a
     * hostname with colons is not mangled. */
    const char *p2 = strrchr(spec, ':');
    if (!p2 || p2 == spec)
        return -1;
    const char *p1 = p2 - 1;
    while (p1 > spec && *p1 != ':') p1--;
    if (*p1 != ':' || p1 == spec)
        return -1;

    size_t hlen = (size_t)(p1 - spec);
    if (hlen == 0 || hlen >= sizeof(c->host))
        return -1;
    memcpy(c->host, spec, hlen);
    c->host[hlen] = '\0';

    char *end = NULL;
    long main_port = strtol(p1 + 1, &end, 10);
    if (end != p2 || main_port <= 0 || main_port > 65535)
        return -1;
    long async_port = strtol(p2 + 1, &end, 10);
    if (!end || *end != '\0' || async_port <= 0 || async_port > 65535)
        return -1;

    c->main_port  = (int)main_port;
    c->async_port = (int)async_port;
    return 0;
}

static void env_int(const char *name, int *out) {
    const char *v = getenv(name);
    if (v && *v) {
        char *end = NULL;
        long n = strtol(v, &end, 10);
        if (end && *end == '\0')
            *out = (int)n;
    }
}

int renode_cosim_config_from_env(renode_cosim_config_t *c) {
    const char *spec = getenv("CSIM_RENODE");
    if (!spec || !*spec)
        return -1;
    if (renode_cosim_config_parse(c, spec) != 0)
        return -1;

    const char *f = getenv("CSIM_RENODE_FREQ_HZ");
    if (f && *f) {
        char *end = NULL;
        long long n = strtoll(f, &end, 10);
        if (end && *end == '\0' && n > 0)
            c->freq_hz = (uint64_t)n;
    }
    env_int("CSIM_RENODE_CONNECT_TIMEOUT_MS", &c->connect_timeout_ms);
    env_int("CSIM_RENODE_TIMEOUT_MS", &c->wait_timeout_ms);
    env_int("CSIM_RENODE_LOG_LEVEL", &c->log_level);
    return 0;
}

bool renode_cosim_active(const renode_cosim_service_t *s) {
    return s && s->active;
}

/* ============================================================
 * Socket I/O
 * ============================================================ */

static int write_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        ssize_t n = write(fd, p, len);
        if (n > 0) { p += n; len -= (size_t)n; continue; }
        if (n < 0 && (errno == EINTR || errno == EAGAIN))
            continue;
        return -1;
    }
    return 0;
}

/* Read exactly `len` bytes.  Returns 1 on success, 0 on timeout (nothing
 * consumed), -1 on EOF or error.  A partial message is a protocol error:
 * once the first byte of a message has arrived the rest must follow, so the
 * timeout only applies before the first byte. */
static int read_exact(int fd, void *buf, size_t len, int timeout_ms) {
    uint8_t *p = (uint8_t *)buf;
    size_t got = 0;
    while (got < len) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        /* Wait in slices so an unbounded wait still notices a closed fd and
         * can report progress; -1 would block forever with no diagnostics. */
        int slice = (got == 0 && timeout_ms > 0) ? timeout_ms : 1000;
        if (got > 0) slice = 5000;          /* mid-message: bounded */
        int pr = poll(&pfd, 1, slice);
        if (pr < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (pr == 0) {
            if (got > 0)
                return -1;                  /* truncated message */
            if (timeout_ms > 0)
                return 0;                   /* caller's timeout expired */
            continue;                       /* wait forever: next slice */
        }
        ssize_t n = read(fd, p + got, len - got);
        if (n > 0) { got += (size_t)n; continue; }
        if (n == 0) return -1;              /* peer closed */
        if (errno == EINTR || errno == EAGAIN) continue;
        return -1;
    }
    return 1;
}

static int send_msg(int fd, int32_t action, uint64_t addr, uint64_t value) {
    renode_msg_t m = { .action = action, .addr = addr, .value = value,
                       .peripheral_index = RENODE_NO_PERIPHERAL_INDEX };
    uint8_t buf[RENODE_MSG_SIZE];
    renode_msg_encode(&m, buf);
    return write_all(fd, buf, sizeof(buf));
}

static int recv_msg(int fd, renode_msg_t *m, int timeout_ms) {
    uint8_t buf[RENODE_MSG_SIZE];
    int rc = read_exact(fd, buf, sizeof(buf), timeout_ms);
    if (rc != 1)
        return rc;
    renode_msg_decode(buf, m);
    return 1;
}

/* ============================================================
 * Async plane: buffered logs + interrupt level
 * ============================================================ */

static void logq_append(renode_cosim_service_t *s, const char *text, size_t len) {
    if (s->logq.len + len > LOGQ_CAP_MAX) {
        s->logq.dropped++;
        return;
    }
    if (s->logq.len + len > s->logq.cap) {
        size_t want = s->logq.cap ? s->logq.cap * 2 : 4096;
        while (want < s->logq.len + len) want *= 2;
        if (want > LOGQ_CAP_MAX) want = LOGQ_CAP_MAX;
        char *nb = (char *)realloc(s->logq.buf, want);
        if (!nb) { s->logq.dropped++; return; }
        s->logq.buf = nb;
        s->logq.cap = want;
    }
    memcpy(s->logq.buf + s->logq.len, text, len);
    s->logq.len += len;
}

/* One logMessage per flush: the header carries the byte count and the text
 * follows it raw, which is how Renode's async reader expects it. */
static int flush_logs(renode_cosim_service_t *s) {
    if (s->logq.len == 0)
        return 0;
    trace("logMessage header, %zu bytes of text", s->logq.len);
    if (send_msg(s->async_fd, RENODE_LOG_MESSAGE, (uint64_t)s->logq.len,
                 (uint64_t)(int64_t)s->cfg.log_level) != 0)
        return -1;
    if (write_all(s->async_fd, s->logq.buf, s->logq.len) != 0)
        return -1;
    trace("logMessage text written");
    s->logq.len = 0;
    s->stats.logs++;
    return 0;
}

/* The line is level-triggered, so only a change is worth a message. */
static int sync_irq(renode_cosim_service_t *s) {
    bool level = renode_dev_irq_level(s->dev);
    if (level == s->irq_level_sent)
        return 0;
    if (send_msg(s->async_fd, RENODE_INTERRUPT, RENODE_DEV_IRQ_INDEX,
                 level ? 1 : 0) != 0)
        return -1;
    s->irq_level_sent = level;
    s->stats.irqs++;
    return 0;
}

/* ============================================================
 * Time base
 * ============================================================ */

/* Recomputed from the running total rather than accumulated, so a frequency
 * that does not divide a second exactly (3 Hz, say) cannot drift. */
static int64_t horizon_for(const renode_cosim_service_t *s) {
    uint64_t f = s->cfg.freq_hz ? s->cfg.freq_hz : 1;
    uint64_t whole = s->total_ticks / f;
    uint64_t rem   = s->total_ticks % f;
    return s->base_ns + (int64_t)(whole * 1000000000ULL) +
           (int64_t)((rem * 1000000000ULL) / f);
}

/* ============================================================
 * Protocol loop
 * ============================================================ */

static int64_t renode_clock_next_horizon(void *state, int64_t cur_ns);

static void mark_dead(renode_cosim_service_t *s, const char *why) {
    if (s->active)
        printf("  renode: %s\n", why);
    s->active = false;
}

/* One request from Renode.  Returns 1 when it was a tickClock (the caller
 * has its horizon), 0 to keep reading, -1 to end the run. */
static int handle_request(renode_cosim_service_t *s, const renode_msg_t *m) {
    int width = renode_action_width(m->action);

    if (m->action == RENODE_TICK_CLOCK) {
        s->total_ticks += m->value;
        s->stats.ticks++;
        return 1;
    }
    if (renode_action_is_read(m->action)) {
        uint64_t v = renode_dev_read(s->dev, m->addr, width);
        s->stats.reads++;
        /* Every read width replies with the generic readRequest action —
         * that is what Renode's reader matches on. */
        if (send_msg(s->main_fd, RENODE_READ_REQUEST, m->addr, v) != 0)
            return -1;
        /* A read can drain a FIFO and lower the line. */
        return sync_irq(s) == 0 ? 0 : -1;
    }
    if (renode_action_is_write(m->action)) {
        renode_dev_write(s->dev, m->addr, width, m->value);
        s->stats.writes++;
        if (send_msg(s->main_fd, RENODE_OK, m->addr, 0) != 0)
            return -1;
        return sync_irq(s) == 0 ? 0 : -1;
    }

    switch (m->action) {
    case RENODE_HANDSHAKE:
        /* A late or repeated handshake: echo it and carry on. */
        return send_msg(s->main_fd, RENODE_HANDSHAKE, 0, 0) == 0 ? 0 : -1;
    case RENODE_RESET_PERIPHERAL:
        /* Renode expects no reply here.  The machine reset it reflects
         * resets the device, not csim's motes — see the plan's limitations. */
        renode_dev_reset(s->dev);
        s->irq_level_sent = false;
        return 0;
    case RENODE_DISCONNECT:
        send_msg(s->async_fd, RENODE_OK, 0, 0);
        mark_dead(s, "master disconnected");
        return -1;
    default:
        /* Co-simulated-CPU actions (registerGet, step, push/get …) never
         * come from a CoSimulatedPeripheral.  Log and ignore: replying to a
         * request Renode is not waiting for would desynchronise the stream
         * for every message after it. */
        printf("  renode: ignoring unsupported action %s (%d)\n",
               renode_action_name(m->action), (int)m->action);
        return 0;
    }
}

/* sim_clock_source_t trampoline. */
static int64_t renode_clock_next_horizon(void *state, int64_t cur_ns) {
    return renode_cosim_next_horizon((renode_cosim_service_t *)state, cur_ns);
}

int64_t renode_cosim_next_horizon(renode_cosim_service_t *s, int64_t cur_ns) {
    if (!s || !s->active)
        return -1;

    /* Bus accesses take no simulation time and are serviced at the boundary
     * of the quantum that just finished.  The pump leaves now_ns at the last
     * event it dispatched, which is at or before that boundary, so pin it
     * here: a frame the guest injects must be stamped at the boundary. */
    if (s->sim && cur_ns > sim_runtime_now_ns(s->sim))
        s->sim->now_ns = cur_ns;

    if (!s->base_set) {
        s->base_ns = cur_ns;
        s->base_set = true;
    }

    /* Anything owed to Renode for the quantum that just ended, in the order
     * Renode should see it: what happened, then the fact it is over. */
    if (s->tick_outstanding) {
        trace("quantum done at %lld; flushing async", (long long)cur_ns);
        renode_dev_uart_retry(s->dev);
        if (flush_logs(s) != 0 || sync_irq(s) != 0) {
            mark_dead(s, "async channel closed");
            return -1;
        }
        /* The tick confirmation goes on the ASYNC socket, not main — see
         * Renode's own integration library (renode_bus.cpp, `case tickClock`
         * replies with sendSender).  Only bus read/write replies go back on
         * main.  Sending it on main instead deadlocks: csim thinks it has
         * answered and Renode waits out its whole tick timeout. */
        trace("async flushed; sending tickClock reply");
        if (send_msg(s->async_fd, RENODE_TICK_CLOCK, 0, 0) != 0) {
            mark_dead(s, "master closed the connection");
            return -1;
        }
        trace("tickClock reply sent; waiting for next request");
        s->tick_outstanding = false;
    }

    for (;;) {
        renode_msg_t m;
        int rc = recv_msg(s->main_fd, &m, s->cfg.wait_timeout_ms);
        if (rc == 0) {
            mark_dead(s, "timed out waiting for the next tick");
            return -1;
        }
        if (rc < 0) {
            mark_dead(s, "peer closed the connection");
            return -1;
        }
        trace("recv %s addr=0x%llx value=%llu", renode_action_name(m.action),
              (unsigned long long)m.addr, (unsigned long long)m.value);
        int hr = handle_request(s, &m);
        if (hr < 0) {
            if (s->active)
                mark_dead(s, "protocol error");
            return -1;
        }
        if (hr == 1)
            break;
    }

    s->tick_outstanding = true;
    int64_t horizon = horizon_for(s);
    /* A horizon behind the clock would ask the pump to run backwards.  It
     * cannot happen with a monotonic tick count, but clamping keeps the
     * contract with the runner explicit. */
    if (horizon < cur_ns)
        horizon = cur_ns;
    return horizon;
}

/* ============================================================
 * Connection + service ops
 * ============================================================ */

static int connect_with_retry(const char *host, int port, int timeout_ms) {
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &sa.sin_addr) != 1) {
        fprintf(stderr, "renode: '%s' is not an IPv4 address\n", host);
        return -1;
    }

    int waited = 0;
    for (;;) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0)
            return -1;
        if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            return fd;
        }
        close(fd);
        if (waited >= timeout_ms)
            return -1;
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 100 * 1000 * 1000 };
        nanosleep(&ts, NULL);
        waited += 100;
    }
}

/* Exactly one device node must exist: with none there is nothing for Renode
 * to talk to, and with two a bus access would be ambiguous. */
static int find_device(renode_cosim_service_t *s, sim_runtime_t *sim) {
    int found = 0;
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++) {
        sim_mote_t *m = sim_runtime_mote(sim, i);
        if (!m || !m->ops || !m->ops->get_interface)
            continue;
        void *dev = m->ops->get_interface(m, SIM_MOTE_IFACE_COSIM_DEV);
        if (!dev)
            continue;
        found++;
        if (found == 1) {
            s->dev = (renode_dev_t *)dev;
            s->dev_slot = i;
        }
    }
    if (found != 1) {
        fprintf(stderr,
                "renode: need exactly one co-simulation device node "
                "(a \".renode\" firmware entry), found %d\n", found);
        return -1;
    }
    return 0;
}

static int finish_attach(renode_cosim_service_t *s, sim_runtime_t *sim) {
    s->sim = sim;

    /* Renode opens with a handshake on the main socket and waits for the
     * same message back. */
    renode_msg_t m;
    int rc = recv_msg(s->main_fd, &m, s->cfg.connect_timeout_ms > 0
                                          ? s->cfg.connect_timeout_ms : 10000);
    if (rc != 1 || m.action != RENODE_HANDSHAKE) {
        fprintf(stderr, "renode: no handshake from the master\n");
        return -1;
    }
    if (send_msg(s->main_fd, RENODE_HANDSHAKE, 0, 0) != 0) {
        fprintf(stderr, "renode: could not answer the handshake\n");
        return -1;
    }

    s->dev->node_count = 0;
    for (int i = 0; i < SIM_EQ_MAX_NODES; i++)
        if (sim_runtime_mote(sim, i)) s->dev->node_count++;

    s->active = true;
    g_attached = s;
    /* From here the runner takes its horizon from us, through the kernel's
     * generic hook — it never names this service. */
    s->clock_source.next_horizon = renode_clock_next_horizon;
    s->clock_source.state = s;
    sim_runtime_set_clock_source(sim, &s->clock_source);
    printf("  Renode co-simulation: attached (node slot %d, %llu Hz tick clock)\n",
           s->dev_slot, (unsigned long long)s->cfg.freq_hz);
    return 0;
}

int renode_cosim_attach_fds(renode_cosim_service_t *s, sim_runtime_t *sim,
                            int main_fd, int async_fd) {
    if (!s || !sim)
        return -1;
    if (!s->cfg.freq_hz)
        config_defaults(&s->cfg);
    s->sim = sim;
    if (find_device(s, sim) != 0)
        return -1;
    s->main_fd  = main_fd;
    s->async_fd = async_fd;
    return finish_attach(s, sim);
}

static int renode_svc_init(sim_runtime_t *sim, const void *cfg, void **state) {
    renode_cosim_service_t *s;

    if (cfg) {
        /* The runner's --renode path hands over its own struct to adopt. */
        s = (renode_cosim_service_t *)(void *)(uintptr_t)cfg;
    } else {
        /* Selected by name from a config's plugins[]: a built-in service
         * receives no arguments there, so the connection comes from the
         * environment. */
        s = (renode_cosim_service_t *)calloc(1, sizeof(*s));
        if (!s)
            return -1;
        s->owned = true;
        if (renode_cosim_config_from_env(&s->cfg) != 0) {
            fprintf(stderr,
                    "renode: CSIM_RENODE is unset or malformed "
                    "(expected \"ADDR:MAIN:ASYNC\")\n");
            free(s);
            return -1;
        }
        s->main_fd = s->async_fd = -1;
    }
    *state = s;

    /* Check the scenario before touching the network: a config with no
     * device node can never answer a bus access, and failing here keeps a
     * waiting master from blocking on a peer that is about to give up. */
    s->sim = sim;
    if (find_device(s, sim) != 0)
        return -1;

    s->main_fd = connect_with_retry(s->cfg.host, s->cfg.main_port,
                                    s->cfg.connect_timeout_ms);
    if (s->main_fd < 0) {
        fprintf(stderr, "renode: could not connect to %s:%d\n",
                s->cfg.host, s->cfg.main_port);
        return -1;
    }
    s->async_fd = connect_with_retry(s->cfg.host, s->cfg.async_port,
                                     s->cfg.connect_timeout_ms);
    if (s->async_fd < 0) {
        fprintf(stderr, "renode: could not connect to %s:%d\n",
                s->cfg.host, s->cfg.async_port);
        close(s->main_fd);
        s->main_fd = -1;
        return -1;
    }
    return finish_attach(s, sim) == 0 ? 0 : -1;
}

/* Observer callbacks must not block, so nothing here writes to a socket —
 * both paths buffer and the quantum boundary flushes. */
static void renode_svc_on_event(sim_runtime_t *sim, void *state,
                                const sim_observer_event_t *ev) {
    (void)sim;
    renode_cosim_service_t *s = (renode_cosim_service_t *)state;
    if (!s || !s->dev)
        return;

    if (ev->kind == SIM_OBS_MOTE_LOG_LINE) {
        char hdr[32];
        int n = snprintf(hdr, sizeof(hdr), "[node %d] ", ev->u.log_line.node_id);
        if (n > 0) logq_append(s, hdr, (size_t)n);
        if (ev->u.log_line.line && ev->u.log_line.len > 0)
            logq_append(s, ev->u.log_line.line, (size_t)ev->u.log_line.len);
        logq_append(s, "\n", 1);
        return;
    }
    if (ev->kind == SIM_OBS_MOTE_UART_BYTE &&
        ev->mote_index == s->dev->uart_node) {
        renode_dev_uart_push(s->dev, ev->u.uart.byte);
    }
}

static void renode_svc_destroy(sim_runtime_t *sim, void *state) {
    renode_cosim_service_t *s = (renode_cosim_service_t *)state;
    if (!s)
        return;
    if (s->main_fd >= 0)  { close(s->main_fd);  s->main_fd = -1; }
    if (s->async_fd >= 0) { close(s->async_fd); s->async_fd = -1; }
    s->active = false;
    if (g_attached == s)
        g_attached = NULL;
    sim_runtime_set_clock_source(sim, NULL);

    /* Deterministic one-liner: what the master actually did, so a CI run can
     * assert the exchange happened rather than that nothing crashed. */
    printf("  renode: %u ticks, %u reads, %u writes, %u irqs, %u log flushes",
           s->stats.ticks, s->stats.reads, s->stats.writes, s->stats.irqs,
           s->stats.logs);
    if (s->logq.dropped)
        printf(", %u log lines dropped", s->logq.dropped);
    printf("\n");

    free(s->logq.buf);
    s->logq.buf = NULL;
    s->logq.len = s->logq.cap = 0;
    if (s->owned)
        free(s);
}

const sim_service_ops_t renode_cosim_service_ops = {
    .name     = "renode",
    .init     = renode_svc_init,
    .destroy  = renode_svc_destroy,
    .on_event = renode_svc_on_event,
    .poll     = NULL,   /* all I/O happens at the quantum boundary */
};
