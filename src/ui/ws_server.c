/*
 * Minimal non-blocking WebSocket server
 *
 * - POSIX sockets, select() with timeout=0; output is queued per client
 *   and drained from the poll, so the caller never waits on a socket
 * - HTTP: serve embedded HTML on GET /, upgrade to WebSocket on GET /ws
 * - WebSocket: text frames (opcode 0x81), handles close/ping
 * - Up to 8 concurrent WebSocket clients
 * - SHA-1 (public domain) + base64 for handshake
 */
#include "ws_server.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>

/* macOS uses SO_NOSIGPIPE instead of MSG_NOSIGNAL */
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define MAX_CLIENTS  8
/* Per-client input buffer: the whole HTTP request, or one WebSocket frame.
 * Cookies are not port-scoped, so a browser sends the page every cookie
 * any dev server on localhost has set; 4 KB of them closed the page load
 * with no reply.  16 KB is Node's request-header limit, and a request that
 * does not fit is answered 431 rather than dropped. */
#define RECV_BUF     16384
/* A client must finish its HTTP request within this long of connecting.
 * Without it, eight connections that never do (idle, or a request the
 * parser never sees end: an embedded NUL, LF-only line endings) hold every
 * slot and the UI turns everyone else away for the rest of the run.  The
 * reply gets longer: the page is 60-odd KB and a slow link is not a
 * fault, but a client that reads it a byte at a time is holding a slot,
 * and 30 s is a 2 KB/s floor that no browser goes under. */
#define HTTP_REQUEST_MS   5000
#define HTTP_RESPONSE_MS 30000

/* When a client that has bytes waiting is given up on (see client_write):
 * more than this queued -- the full state of a large simulation is a few
 * hundred KB, so a viewer this far behind is not slow, it has stopped
 * reading -- or its socket taking nothing at all for this long, which
 * with small messages comes first.  A live link, however slow, takes
 * *something* in 10 s; the browser, not the operator's JavaScript, reads
 * the socket, so a page busy parsing does not go quiet for that long.
 * The queue also carries GET /'s reply, so it holds the largest page
 * ws_server_set_html accepts plus that reply's header: a page can never
 * be cut off at the queue limit. */
#define OUT_QUEUE_MAX  (WS_SERVER_PAGE_MAX + 256)
#define OUT_STALL_MS   10000

/* After a 431 the client may still be writing the rest of its request.
 * Its input is read and discarded until it hangs up, or for this long
 * after the reply went out: closing with unread bytes in the socket would
 * reset the connection, and a browser then reports the reset rather than
 * the 431.  A browser reads the reply and closes at once; this only bounds
 * a client that never does. */
#define HTTP_DRAIN_MS 2000

typedef enum { CLIENT_HTTP, CLIENT_WS } client_state_t;

/* Why a request was refused; each keeps its own place in the log. */
typedef enum {
    REFUSED_HEADER,     /* oversized, repeated or malformed Host/Origin */
    REFUSED_HOST,       /* non-loopback Host on a loopback-bound server */
    REFUSED_ORIGIN,     /* WebSocket upgrade from another site's page */
    REFUSED_REASONS
} refusal_t;

static const char *const refusal_text[REFUSED_REASONS] = {
    "oversized, repeated or malformed header",
    "non-loopback Host",
    "cross-origin WebSocket from",
};

typedef struct {
    int fd;
    client_state_t state;
    int64_t accepted_ms;    /* CLOCK_MONOTONIC; bounds the HTTP exchange */
    char recv_buf[RECV_BUF];
    int recv_len;
    /* Bytes the socket has not taken yet: out[out_head .. out_head+out_len).
     * ws_server_poll sends them as the socket becomes writable. */
    uint8_t *out;
    size_t out_head, out_len, out_cap;
    int64_t out_progress_ms; /* the socket last took some of the queue */
    int close_when_sent;    /* the reply is queued; close after it */
    int64_t drain_until_ms; /* ... but first read its input until EOF or
                               this time (0: close as soon as it is sent) */
} ws_client_t;

struct ws_server {
    int listen_fd;
    int loopback_only;      /* bound to 127.0.0.0/8: refuse non-loopback Host */
    ws_client_t clients[MAX_CLIENTS];
    int client_count;
    char *html;
    int html_len;
    ws_message_cb_t msg_cb;
    void *msg_userdata;
    /* Refusal log: one line a second for each reason, and a count of the
     * ones held back meanwhile (see log_refusal). */
    struct { time_t logged_at; unsigned held; } refused[REFUSED_REASONS];
};

/* ---- SHA-1 (public domain, from RFC 3174 / Steve Reid) ---- */

typedef struct {
    uint32_t state[5];
    uint32_t count[2];
    uint8_t  buffer[64];
} sha1_ctx;

#define SHA1_ROL(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

static void sha1_transform(uint32_t state[5], const uint8_t buffer[64]) {
    uint32_t a, b, c, d, e, w[80];
    for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)buffer[i*4] << 24) | ((uint32_t)buffer[i*4+1] << 16) |
               ((uint32_t)buffer[i*4+2] << 8) | buffer[i*4+3];
    for (int i = 16; i < 80; i++)
        w[i] = SHA1_ROL(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);
    a = state[0]; b = state[1]; c = state[2]; d = state[3]; e = state[4];
    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d;             k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else              { f = b ^ c ^ d;             k = 0xCA62C1D6; }
        uint32_t t = SHA1_ROL(a, 5) + f + e + k + w[i];
        e = d; d = c; c = SHA1_ROL(b, 30); b = a; a = t;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d; state[4] += e;
}

static void sha1_init(sha1_ctx *ctx) {
    ctx->state[0] = 0x67452301; ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE; ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count[0] = ctx->count[1] = 0;
}

static void sha1_update(sha1_ctx *ctx, const uint8_t *data, uint32_t len) {
    uint32_t i = 0, j = (ctx->count[0] >> 3) & 63;
    ctx->count[0] += len << 3;
    if (ctx->count[0] < (len << 3)) ctx->count[1]++;
    ctx->count[1] += len >> 29;
    if ((j + len) > 63) {
        i = 64 - j;
        memcpy(&ctx->buffer[j], data, i);
        sha1_transform(ctx->state, ctx->buffer);
        for (; i + 63 < len; i += 64)
            sha1_transform(ctx->state, &data[i]);
        j = 0;
    }
    memcpy(&ctx->buffer[j], &data[i], len - i);
}

static void sha1_final(sha1_ctx *ctx, uint8_t digest[20]) {
    uint8_t bits[8];
    for (int i = 0; i < 4; i++) {
        bits[i]     = (uint8_t)(ctx->count[1] >> (24 - i * 8));
        bits[i + 4] = (uint8_t)(ctx->count[0] >> (24 - i * 8));
    }
    uint8_t pad = 0x80;
    sha1_update(ctx, &pad, 1);
    pad = 0;
    while ((ctx->count[0] >> 3) % 64 != 56)
        sha1_update(ctx, &pad, 1);
    sha1_update(ctx, bits, 8);
    for (int i = 0; i < 20; i++)
        digest[i] = (uint8_t)(ctx->state[i >> 2] >> (24 - (i & 3) * 8));
}

/* ---- Base64 encode ---- */

static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t *in, int len, char *out) {
    int o = 0;
    for (int i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < len) v |= (uint32_t)in[i+1] << 8;
        if (i + 2 < len) v |= in[i+2];
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = (i + 1 < len) ? b64_table[(v >> 6) & 0x3F] : '=';
        out[o++] = (i + 2 < len) ? b64_table[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

/* ---- Socket helpers ---- */

static void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static int64_t monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void close_client(ws_server_t *srv, int idx) {
    close(srv->clients[idx].fd);
    free(srv->clients[idx].out);
    srv->clients[idx] = srv->clients[--srv->client_count];
}

/* Everything sent to a client goes through client_write: what the socket
 * takes at once goes straight out and the rest waits in the client's queue,
 * which ws_server_poll drains as the socket becomes writable.  The
 * simulation thread therefore never waits on a socket -- no send loop, no
 * poll(), no wall-clock allowance to tune.  A viewer that keeps up on
 * average never queues anything; a slow link queues a little and drains it;
 * a viewer that has stopped reading falls further and further behind until
 * its queue passes OUT_QUEUE_MAX, or its socket has taken nothing for
 * OUT_STALL_MS, and it is dropped (the latter in ws_server_poll).  Neither
 * is a speed: a slow link is never mistaken for a dead one.  Returns 0 on
 * success, CLIENT_BEHIND when the queue is over the limit, and -1 when
 * the client has gone. */
#define CLIENT_BEHIND (-2)

static int client_write(ws_client_t *c, const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    if (c->out_len == 0) {
        while (len > 0) {
            ssize_t n = send(c->fd, p, len, MSG_NOSIGNAL);
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                return -1;
            }
            p += n;
            len -= (size_t)n;
        }
        if (len == 0) return 0;
    }
    if (c->out_len + len > OUT_QUEUE_MAX) return CLIENT_BEHIND;
    if (c->out_len == 0) c->out_progress_ms = monotonic_ms();
    if (c->out_head) {
        memmove(c->out, c->out + c->out_head, c->out_len);
        c->out_head = 0;
    }
    if (c->out_len + len > c->out_cap) {
        size_t cap = c->out_cap ? c->out_cap : 16384;
        while (cap < c->out_len + len) cap *= 2;
        uint8_t *grown = realloc(c->out, cap);
        if (!grown) return -1;
        c->out = grown;
        c->out_cap = cap;
    }
    memcpy(c->out + c->out_len, p, len);
    c->out_len += len;
    return 0;
}

/* Send what the socket will take of the queue.  -1 if the client has gone. */
static int client_flush(ws_client_t *c) {
    while (c->out_len > 0) {
        ssize_t n = send(c->fd, c->out + c->out_head, c->out_len, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        c->out_head += (size_t)n;
        c->out_len -= (size_t)n;
        c->out_progress_ms = monotonic_ms();
    }
    c->out_head = 0;
    return 0;
}

/* Drop a client that has stopped reading, and say so: a viewer that keeps
 * reconnecting and being dropped is otherwise just a UI that never
 * updates.  A client that has simply gone (closed tab) is not news. */
static void drop_behind(ws_server_t *srv, int idx, const char *how) {
    ws_client_t *c = &srv->clients[idx];
    if (c->state == CLIENT_WS) {
        fflush(stdout);
        fprintf(stderr, "ws_server: dropped a WebSocket client that stopped "
                "reading (%s, %zu bytes unsent)\n", how, c->out_len);
    }
    close_client(srv, idx);
}

/* The client's reply is queued: close it now if it has all gone out, else
 * once ws_server_poll has sent the rest. */
static void finish_client(ws_server_t *srv, int idx) {
    ws_client_t *c = &srv->clients[idx];
    if (c->out_len == 0)
        close_client(srv, idx);
    else
        c->close_when_sent = 1;
}

/* ---- Request vetting ----
 *
 * The UI accepts commands (pause, speed, restart, move), and WebSocket is
 * exempt from the same-origin policy, so any page the operator has open
 * could otherwise connect to it.  Two checks:
 *
 *  - Origin: a browser always sends it on the upgrade.  It must name this
 *    server, i.e. equal the Host the browser addressed (the page we serve
 *    connects to ws://<location.host>/ws, so our own page always passes).
 *    Non-browser clients send no Origin and are not the threat.
 *  - Host: on a loopback-bound server it must be a loopback name.  This is
 *    what stops DNS rebinding, where evil.example resolves to 127.0.0.1 and
 *    Origin and Host then agree with each other.
 */

/* Copy the value of header `name` (case-insensitive, per HTTP) into out.
 * Returns 1 if found, 0 if absent, -1 if it cannot be trusted: longer than
 * out, sent twice, or with whitespace before the colon.  RFC 9112 5.1 says
 * to reject that last spelling; skipping it instead would let "Host :"
 * through as a request with no Host at all. */
static int http_header(const char *req, const char *name, char *out, size_t outsz) {
    size_t nlen = strlen(name);
    int found = 0;
    const char *line = strstr(req, "\r\n");
    while (line) {
        line += 2;
        if (line[0] == '\r') break;                 /* blank line: end of headers */
        if (strncasecmp(line, name, nlen) == 0) {
            const char *colon = line + nlen;
            while (*colon == ' ' || *colon == '\t') colon++;
            if (*colon == ':') {
                if (colon != line + nlen || found) return -1;
                const char *v = colon + 1;
                while (*v == ' ' || *v == '\t') v++;
                const char *end = strstr(v, "\r\n");
                if (!end) return 0;
                while (end > v && (end[-1] == ' ' || end[-1] == '\t')) end--;
                size_t vlen = (size_t)(end - v);
                if (vlen >= outsz) return -1;
                memcpy(out, v, vlen);
                out[vlen] = '\0';
                found = 1;
            }
        }
        line = strstr(line, "\r\n");
    }
    return found;
}

/* A Host value ("name[:port]") that names this machine's IPv4 loopback,
 * the only place a loopback-bound server listens (so not "[::1]"). */
static int host_is_loopback(const char *host) {
    char name[64];
    const char *colon = strchr(host, ':');
    size_t n = colon ? (size_t)(colon - host) : strlen(host);
    if (n == 0 || n >= sizeof(name)) return 0;
    memcpy(name, host, n);
    name[n] = '\0';
    if (strcasecmp(name, "localhost") == 0)
        return 1;
    struct in_addr a;
    return inet_pton(AF_INET, name, &a) == 1 &&
           (ntohl(a.s_addr) >> 24) == 127;
}

/* Origin is "scheme://host[:port]"; it names this server if it is exactly
 * "http://" + the Host the request was sent to.  "null" never matches. */
static int origin_matches_host(const char *origin, const char *host) {
    return strncasecmp(origin, "http://", 7) == 0 &&
           strcasecmp(origin + 7, host) == 0;
}

/* One stderr line per refusal, at most one a second for each reason: a
 * page that loops `new WebSocket()` would otherwise flood it.  The ones
 * held back are counted, per reason, and the count is reported when the
 * second is over (flush_refusals, from the poll) -- not with the next
 * refusal, which on a long run could be hours away.  The value is the
 * client's, so it is quoted, capped and escaped -- raw, it could carry
 * terminal control sequences that retitle or clear the operator's
 * terminal. */
static void log_refusal(ws_server_t *srv, refusal_t why, const char *what) {
    time_t now = time(NULL);
    if (now == srv->refused[why].logged_at) {
        srv->refused[why].held++;
        return;
    }
    char safe[64 * 4 + 1];
    size_t o = 0, i;
    for (i = 0; what[i] && i < 64; i++) {
        unsigned char ch = (unsigned char)what[i];
        if (ch >= 0x20 && ch < 0x7f && ch != '"' && ch != '\\')
            safe[o++] = (char)ch;
        else
            o += (size_t)snprintf(safe + o, sizeof(safe) - o, "\\x%02x", ch);
    }
    safe[o] = '\0';
    fflush(stdout);         /* keep the line off a half-written stdout line */
    fprintf(stderr, "ws_server: refused request: %s \"%s\"%s", refusal_text[why],
            safe, what[i] ? "..." : "");
    if (srv->refused[why].held)
        fprintf(stderr, " (%u more since the last report)", srv->refused[why].held);
    fputc('\n', stderr);
    srv->refused[why].logged_at = now;
    srv->refused[why].held = 0;
}

/* Report the refusals held back once their second is over (always, at
 * shutdown). */
static void flush_refusals(ws_server_t *srv, int all) {
    time_t now = time(NULL);
    for (int why = 0; why < REFUSED_REASONS; why++) {
        if (!srv->refused[why].held) continue;
        if (!all && now == srv->refused[why].logged_at) continue;
        fflush(stdout);
        fprintf(stderr, "ws_server: %u more request(s) refused: %s\n",
                srv->refused[why].held, refusal_text[why]);
        srv->refused[why].held = 0;
    }
}

/* Answer a short status-only reply and close: once it has gone out, or,
 * with drain set, once the client has hung up or HTTP_DRAIN_MS have passed
 * (for a reply to a request the client may still be writing). */
static void reply_and_close(ws_server_t *srv, int idx, const char *status,
                            int drain) {
    ws_client_t *c = &srv->clients[idx];
    char resp[128];
    int n = snprintf(resp, sizeof(resp), "HTTP/1.1 %s\r\nContent-Length: 0\r\n"
                     "Connection: close\r\n\r\n", status);
    if (client_write(c, resp, (size_t)n) != 0) {
        close_client(srv, idx);
    } else if (drain) {
        c->close_when_sent = 1;
        c->drain_until_ms = monotonic_ms() + HTTP_DRAIN_MS;
    } else {
        finish_client(srv, idx);
    }
}

static void refuse(ws_server_t *srv, int idx, refusal_t why, const char *what) {
    log_refusal(srv, why, what);
    reply_and_close(srv, idx, "403 Forbidden", 0);
}

/* ---- HTTP / WebSocket handling ---- */

static const char *WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/* The path of a "GET <path> HTTP/..." request line, copied into out.
 * Returns 0 for any other method, or a path longer than out. */
static int request_path(const char *req, char *out, size_t outsz) {
    if (strncmp(req, "GET ", 4) != 0) return 0;
    const char *p = req + 4;
    const char *sp = strchr(p, ' ');
    const char *eol = strstr(p, "\r\n");
    if (!sp || !eol || sp > eol || (size_t)(sp - p) >= outsz) return 0;
    memcpy(out, p, (size_t)(sp - p));
    out[sp - p] = '\0';
    return 1;
}

static void handle_http_request(ws_server_t *srv, int idx) {
    ws_client_t *c = &srv->clients[idx];
    c->recv_buf[c->recv_len] = '\0';

    /* Check for complete HTTP request (double CRLF) */
    if (!strstr(c->recv_buf, "\r\n\r\n")) {
        if (c->recv_len >= RECV_BUF - 1)
            reply_and_close(srv, idx, "431 Request Header Fields Too Large", 1);
        return;
    }

    char host[256] = "", origin[256] = "";
    int has_host = http_header(c->recv_buf, "Host", host, sizeof(host));
    int has_origin = http_header(c->recv_buf, "Origin", origin, sizeof(origin));
    if (has_host < 0 || has_origin < 0) {
        refuse(srv, idx, REFUSED_HEADER, has_host < 0 ? "Host" : "Origin");
        return;
    }
    if (srv->loopback_only && has_host && !host_is_loopback(host)) {
        refuse(srv, idx, REFUSED_HOST, host);
        return;
    }

    /* The method and path come from the request line and the headers from
     * http_header, which matches names case-insensitively as HTTP requires
     * -- not strstr over the whole request, which took "GET /wsx" for
     * "/ws" and missed a lowercase "upgrade:". */
    char path[64] = "", upgrade[32] = "", key[64] = "";
    int is_get = request_path(c->recv_buf, path, sizeof(path));

    /* WebSocket upgrade: GET /ws */
    if (is_get && strcmp(path, "/ws") == 0 &&
        http_header(c->recv_buf, "Upgrade", upgrade, sizeof(upgrade)) == 1 &&
        strcasecmp(upgrade, "websocket") == 0) {
        if (has_origin && !(has_host && origin_matches_host(origin, host))) {
            refuse(srv, idx, REFUSED_ORIGIN, origin);
            return;
        }
        if (http_header(c->recv_buf, "Sec-WebSocket-Key", key, sizeof(key)) != 1) {
            close_client(srv, idx);
            return;
        }

        /* SHA-1(key + magic) */
        sha1_ctx sha;
        sha1_init(&sha);
        sha1_update(&sha, (const uint8_t *)key, (uint32_t)strlen(key));
        sha1_update(&sha, (const uint8_t *)WS_MAGIC, 36);
        uint8_t digest[20];
        sha1_final(&sha, digest);

        char accept[32];
        base64_encode(digest, 20, accept);

        char resp[256];
        int rlen = snprintf(resp, sizeof(resp),
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
        if (client_write(c, resp, (size_t)rlen) != 0) {
            close_client(srv, idx);
            return;
        }
        c->state = CLIENT_WS;
        c->recv_len = 0;
        return;
    }

    /* Serve HTML on GET / */
    if (is_get && (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0)) {
        const char *body = srv->html ? srv->html : "<html><body>No UI loaded</body></html>";
        int blen = srv->html ? srv->html_len : (int)strlen(body);
        char hdr[256];
        int hlen = snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Cache-Control: no-cache, no-store\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n", blen);
        /* One send() delivers what fits the socket buffer; the rest of the
         * page waits in the queue and the slot closes once it has gone. */
        if (client_write(c, hdr, (size_t)hlen) != 0 ||
            client_write(c, body, (size_t)blen) != 0)
            close_client(srv, idx);
        else
            finish_client(srv, idx);
        return;
    }

    /* 404 for everything else */
    reply_and_close(srv, idx, "404 Not Found", 0);
}

static void handle_ws_frame(ws_server_t *srv, int idx) {
    ws_client_t *c = &srv->clients[idx];
    while (c->recv_len >= 2) {
        uint8_t *buf = (uint8_t *)c->recv_buf;
        uint8_t opcode = buf[0] & 0x0F;

        /* Each message must arrive in one frame.  Control frames may never
         * be fragmented (RFC 6455 5.5) and this server does not reassemble
         * data messages, which it would otherwise dispatch piece by piece,
         * each as if whole.  RSV bits need a negotiated extension and the
         * other opcodes are reserved (5.2): fail the connection on all. */
        if ((buf[0] & 0x70) || !(buf[0] & 0x80) ||
            !(opcode == 0x1 || opcode == 0x2 || opcode == 0x8 ||
              opcode == 0x9 || opcode == 0xA)) {
            close_client(srv, idx);
            return;
        }

        int masked = (buf[1] >> 7) & 1;
        uint64_t payload_len = buf[1] & 0x7F;
        int header_len = 2;

        if (payload_len == 126) {
            if (c->recv_len < 4) return;
            payload_len = ((uint64_t)buf[2] << 8) | buf[3];
            header_len = 4;
        } else if (payload_len == 127) {
            if (c->recv_len < 10) return;
            payload_len = 0;
            for (int i = 0; i < 8; i++)
                payload_len = (payload_len << 8) | buf[2 + i];
            header_len = 10;
        }

        /* RFC 6455 5.5: control frames carry at most 125 bytes.  The pong
         * below echoes a ping in a 7-bit length, so a longer one would get
         * a header that does not match its body. */
        if (opcode >= 0x8 && payload_len > 125) {
            close_client(srv, idx);
            return;
        }

        if (masked) header_len += 4;

        /* Validate the declared length before any arithmetic that could
         * overflow.  payload_len is attacker-controlled (up to 2^64-1 via
         * the 127 form); fold it into a signed `total` and a crafted frame
         * makes `total` negative, the `recv_len < total` guard pass, and the
         * memmove below run wild.  Reject anything that cannot fit the fixed
         * RECV_BUF, and require client frames to be masked per RFC 6455. */
        if (!masked || payload_len > (uint64_t)(RECV_BUF - header_len)) {
            close_client(srv, idx);
            return;
        }
        int total = header_len + (int)payload_len;
        if (c->recv_len < total) return;

        /* Unmask payload if needed */
        if (masked) {
            uint8_t *mask = buf + header_len - 4;
            uint8_t *data = buf + header_len;
            for (int i = 0; i < (int)payload_len; i++)
                data[i] ^= mask[i & 3];
        }

        if (opcode == 0x8) {
            /* Close frame — send close back */
            uint8_t close_frame[2] = { 0x88, 0x00 };
            client_write(c, close_frame, 2);
            close_client(srv, idx);
            return;
        } else if (opcode == 0x9) {
            /* Ping — respond with pong */
            uint8_t pong[2] = { 0x8A, (uint8_t)(payload_len & 0x7F) };
            if (client_write(c, pong, 2) != 0 ||
                client_write(c, buf + header_len, (size_t)payload_len) != 0) {
                close_client(srv, idx);
                return;
            }
        }
        /* Dispatch text/binary data frames to callback */
        if ((opcode == 0x1 || opcode == 0x2) && srv->msg_cb) {
            srv->msg_cb((const char *)(buf + header_len), (int)payload_len, srv->msg_userdata);
        }

        /* Remove consumed frame from buffer */
        memmove(c->recv_buf, c->recv_buf + total, c->recv_len - total);
        c->recv_len -= total;
    }
}

/* ---- Public API ---- */

/* NULL and "localhost" both mean 127.0.0.1. */
static const char *bind_or_default(const char *bind_addr) {
    return !bind_addr || strcmp(bind_addr, "localhost") == 0 ? "127.0.0.1"
                                                              : bind_addr;
}

int ws_server_bind_addr_valid(const char *bind_addr) {
    struct in_addr a;
    return inet_pton(AF_INET, bind_or_default(bind_addr), &a) == 1;
}

void ws_server_url(const char *bind_addr, int port, char *out, size_t outsz) {
    bind_addr = bind_or_default(bind_addr);
    /* localhost reaches 127.0.0.1 and a wildcard bind; any other address
     * (127.0.0.2 included) is only reachable as itself. */
    if (strcmp(bind_addr, "127.0.0.1") == 0 || strcmp(bind_addr, "0.0.0.0") == 0)
        bind_addr = "localhost";
    snprintf(out, outsz, "http://%s:%d/", bind_addr, port);
}

ws_server_t *ws_server_init(const char *bind_addr, int port) {
    struct in_addr bind_in;
    bind_addr = bind_or_default(bind_addr);
    if (inet_pton(AF_INET, bind_addr, &bind_in) != 1) {
        fprintf(stderr, "ws_server: '%s' is not an IPv4 address\n", bind_addr);
        return NULL;
    }

    ws_server_t *srv = calloc(1, sizeof(ws_server_t));
    if (!srv) return NULL;
    srv->loopback_only = (ntohl(bind_in.s_addr) >> 24) == 127;

    /* Ignore SIGPIPE globally — broken WebSocket connections must not kill the process */
    signal(SIGPIPE, SIG_IGN);

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd < 0) { free(srv); return NULL; }

    int opt = 1;
    setsockopt(srv->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    set_nonblocking(srv->listen_fd);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr = bind_in;
    addr.sin_port = htons((uint16_t)port);

    if (bind(srv->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "ws_server: bind %s:%d failed: %s\n", bind_addr, port,
                strerror(errno));
        close(srv->listen_fd);
        free(srv);
        return NULL;
    }

    if (listen(srv->listen_fd, 8) < 0) {
        close(srv->listen_fd);
        free(srv);
        return NULL;
    }

    char url[64];
    ws_server_url(bind_addr, port, url, sizeof(url));
    if (srv->loopback_only)
        printf("WebSocket UI server listening on %s\n", url);
    else
        printf("WebSocket UI server listening on %s:%d (NOT loopback-only: "
               "reachable from the network)\n", bind_addr, port);
    return srv;
}

void ws_server_poll(ws_server_t *srv) {
    if (!srv) return;

    flush_refusals(srv, 0);
    int64_t now_ms = monotonic_ms();
    for (int i = 0; i < srv->client_count; i++) {
        ws_client_t *c = &srv->clients[i];
        if (c->state == CLIENT_HTTP &&
            now_ms - c->accepted_ms >
                (c->close_when_sent ? HTTP_RESPONSE_MS : HTTP_REQUEST_MS)) {
            close_client(srv, i); i--;
        } else if (c->drain_until_ms && c->out_len == 0 &&
                   now_ms > c->drain_until_ms) {
            close_client(srv, i); i--;          /* the reply is out; drained long enough */
        } else if (c->out_len > 0 && now_ms - c->out_progress_ms > OUT_STALL_MS) {
            char how[48];
            snprintf(how, sizeof(how), "took nothing for %d s", OUT_STALL_MS / 1000);
            drop_behind(srv, i, how); i--;
        }
    }

    fd_set read_fds, write_fds;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_SET(srv->listen_fd, &read_fds);
    int max_fd = srv->listen_fd;

    for (int i = 0; i < srv->client_count; i++) {
        ws_client_t *c = &srv->clients[i];
        FD_SET(c->fd, &read_fds);
        if (c->out_len > 0) FD_SET(c->fd, &write_fds);
        if (c->fd > max_fd)
            max_fd = c->fd;
    }

    struct timeval tv = { 0, 0 }; /* non-blocking */
    int ready = select(max_fd + 1, &read_fds, &write_fds, NULL, &tv);
    if (ready <= 0) return;

    /* Accept new connections */
    if (FD_ISSET(srv->listen_fd, &read_fds)) {
        struct sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        int client_fd = accept(srv->listen_fd, (struct sockaddr *)&client_addr, &addrlen);
        if (client_fd >= 0) {
            if (srv->client_count < MAX_CLIENTS) {
                set_nonblocking(client_fd);
                int flag = 1;
                setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
                /* The client's queue is the buffer, so the kernel's is kept
                 * small and fixed (Linux would otherwise grow it to
                 * megabytes): OUT_QUEUE_MAX and OUT_STALL_MS are then the
                 * bounds on a client that stopped reading, not the kernel's
                 * buffer plus them.  64 KB is 25 Mbit/s at a 20 ms RTT,
                 * more than the UI needs. */
                int sndbuf = 64 * 1024;
                setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
                ws_client_t *c = &srv->clients[srv->client_count++];
                memset(c, 0, sizeof(*c));
                c->fd = client_fd;
                c->state = CLIENT_HTTP;
                c->accepted_ms = now_ms;
            } else {
                /* Full: say so rather than hang up without a word. */
                const char *busy = "HTTP/1.1 503 Service Unavailable\r\n"
                                   "Content-Length: 0\r\nConnection: close\r\n\r\n";
                send(client_fd, busy, strlen(busy), MSG_NOSIGNAL);
                close(client_fd);
            }
        }
    }

    /* Send what the writable clients will take */
    for (int i = 0; i < srv->client_count; i++) {
        ws_client_t *c = &srv->clients[i];
        if (c->out_len == 0 || !FD_ISSET(c->fd, &write_fds))
            continue;
        if (client_flush(c) < 0 ||
            (c->out_len == 0 && c->close_when_sent && !c->drain_until_ms)) {
            close_client(srv, i); i--;
        }
    }

    /* Read from existing clients */
    for (int i = 0; i < srv->client_count; i++) {
        if (!FD_ISSET(srv->clients[i].fd, &read_fds))
            continue;

        /* A client whose reply is queued has nothing more to say: its
         * input is read and dropped, which notices it hanging up and,
         * for a request that did not fit (drain_until_ms), takes the rest
         * of it off the socket so closing does not reset the connection
         * under the reply. */
        if (srv->clients[i].close_when_sent) {
            char sink[1024];
            ssize_t n = recv(srv->clients[i].fd, sink, sizeof(sink), 0);
            if (n == 0 || (n < 0 && errno != EINTR && errno != EAGAIN &&
                           errno != EWOULDBLOCK)) {
                close_client(srv, i); i--;
            }
            continue;
        }

        /* The HTTP request is parsed as a string and keeps a byte for its
         * terminator; a WebSocket frame may fill the buffer, which is the
         * size the frame parser accepts up to. */
        ws_client_t *c = &srv->clients[i];
        int space = RECV_BUF - c->recv_len - (c->state == CLIENT_HTTP);
        if (space <= 0) { close_client(srv, i); i--; continue; }

        ssize_t n = recv(c->fd, c->recv_buf + c->recv_len, space, 0);
        if (n <= 0) {
            close_client(srv, i);
            i--;
            continue;
        }
        c->recv_len += (int)n;

        if (c->state == CLIENT_HTTP)
            handle_http_request(srv, i);
        else
            handle_ws_frame(srv, i);
    }
}

static int ws_build_header(uint8_t *header, uint8_t opcode, int len) {
    header[0] = 0x80 | opcode; /* FIN + opcode */
    if (len < 126) {
        header[1] = (uint8_t)len;
        return 2;
    } else if (len < 65536) {
        header[1] = 126;
        header[2] = (uint8_t)(len >> 8);
        header[3] = (uint8_t)(len & 0xFF);
        return 4;
    } else {
        header[1] = 127;
        memset(header + 2, 0, 4);
        header[6] = (uint8_t)((len >> 24) & 0xFF);
        header[7] = (uint8_t)((len >> 16) & 0xFF);
        header[8] = (uint8_t)((len >> 8) & 0xFF);
        header[9] = (uint8_t)(len & 0xFF);
        return 10;
    }
}

static void broadcast_frame(ws_server_t *srv, const uint8_t *header, int hlen,
                            const void *data, int len) {
    for (int i = 0; i < srv->client_count; i++) {
        ws_client_t *c = &srv->clients[i];
        if (c->state != CLIENT_WS)
            continue;
        int r = client_write(c, header, (size_t)hlen);
        if (r == 0)
            r = client_write(c, data, (size_t)len);
        if (r == CLIENT_BEHIND) {
            drop_behind(srv, i, "over the queue limit"); i--;
        } else if (r < 0) {
            close_client(srv, i); i--;
        }
    }
}

void ws_server_broadcast(ws_server_t *srv, const char *data, int len) {
    if (!srv || len <= 0) return;

    uint8_t header[10];
    int hlen = ws_build_header(header, 0x01, len); /* text */
    broadcast_frame(srv, header, hlen, data, len);
}

void ws_server_broadcast_binary(ws_server_t *srv, const uint8_t *data, int len) {
    if (!srv || len <= 0) return;

    uint8_t header[10];
    int hlen = ws_build_header(header, 0x02, len); /* binary */
    broadcast_frame(srv, header, hlen, data, len);
}

void ws_server_set_message_callback(ws_server_t *srv, ws_message_cb_t cb, void *userdata) {
    if (!srv) return;
    srv->msg_cb = cb;
    srv->msg_userdata = userdata;
}

int ws_server_set_html(ws_server_t *srv, const char *html, int len) {
    if (!srv) return -1;
    if (len < 0 || len > WS_SERVER_PAGE_MAX) return -1;
    free(srv->html);
    srv->html = malloc(len + 1);
    if (!srv->html) {
        srv->html_len = 0;
        return -1;
    }
    memcpy(srv->html, html, len);
    srv->html[len] = '\0';
    srv->html_len = len;
    return 0;
}

int ws_server_client_count(ws_server_t *srv) {
    return srv ? srv->client_count : 0;
}

void ws_server_destroy(ws_server_t *srv) {
    if (!srv) return;
    flush_refusals(srv, 1);
    for (int i = 0; i < srv->client_count; i++) {
        close(srv->clients[i].fd);
        free(srv->clients[i].out);
    }
    close(srv->listen_fd);
    free(srv->html);
    free(srv);
}
