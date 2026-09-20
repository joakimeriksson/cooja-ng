/*
 * Minimal non-blocking WebSocket server
 *
 * - POSIX sockets, select() with timeout=0
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

/* macOS uses SO_NOSIGPIPE instead of MSG_NOSIGNAL */
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define MAX_CLIENTS  8
#define RECV_BUF     4096

typedef enum { CLIENT_HTTP, CLIENT_WS } client_state_t;

typedef struct {
    int fd;
    client_state_t state;
    char recv_buf[RECV_BUF];
    int recv_len;
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

static void close_client(ws_server_t *srv, int idx) {
    close(srv->clients[idx].fd);
    srv->clients[idx] = srv->clients[--srv->client_count];
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
 * Returns 1 if found, 0 if absent, -1 if present but longer than out. */
static int http_header(const char *req, const char *name, char *out, size_t outsz) {
    size_t nlen = strlen(name);
    const char *line = strstr(req, "\r\n");
    while (line) {
        line += 2;
        if (line[0] == '\r') break;                 /* blank line: end of headers */
        if (strncasecmp(line, name, nlen) == 0 && line[nlen] == ':') {
            const char *v = line + nlen + 1;
            while (*v == ' ' || *v == '\t') v++;
            const char *end = strstr(v, "\r\n");
            if (!end) return 0;
            while (end > v && (end[-1] == ' ' || end[-1] == '\t')) end--;
            size_t vlen = (size_t)(end - v);
            if (vlen >= outsz) return -1;
            memcpy(out, v, vlen);
            out[vlen] = '\0';
            return 1;
        }
        line = strstr(line, "\r\n");
    }
    return 0;
}

/* A Host value ("name[:port]", "[v6]:port") that names this machine. */
static int host_is_loopback(const char *host) {
    char name[64];
    size_t n;
    if (host[0] == '[') {
        const char *close = strchr(host, ']');
        if (!close) return 0;
        n = (size_t)(close - host - 1);
        host++;
    } else {
        const char *colon = strchr(host, ':');
        n = colon ? (size_t)(colon - host) : strlen(host);
    }
    if (n == 0 || n >= sizeof(name)) return 0;
    memcpy(name, host, n);
    name[n] = '\0';
    if (strcasecmp(name, "localhost") == 0 || strcmp(name, "::1") == 0)
        return 1;
    struct in_addr a;
    return inet_pton(AF_INET, name, &a) == 1 &&
           (ntohl(a.s_addr) >> 24) == 127;
}

/* Origin is "scheme://host[:port]"; it names this server if that host[:port]
 * is exactly the Host the request was sent to. */
static int origin_matches_host(const char *origin, const char *host) {
    const char *p = strstr(origin, "://");
    if (!p) return 0;                               /* includes "null" */
    return strcasecmp(p + 3, host) == 0;
}

static void refuse(ws_server_t *srv, int idx, const char *why, const char *what) {
    ws_client_t *c = &srv->clients[idx];
    fprintf(stderr, "ws_server: refused request: %s %s\n", why, what);
    const char *resp = "HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n"
                       "Connection: close\r\n\r\n";
    send(c->fd, resp, (int)strlen(resp), MSG_NOSIGNAL);
    close_client(srv, idx);
}

/* ---- HTTP / WebSocket handling ---- */

static const char *WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static void handle_http_request(ws_server_t *srv, int idx) {
    ws_client_t *c = &srv->clients[idx];
    c->recv_buf[c->recv_len] = '\0';

    /* Check for complete HTTP request (double CRLF) */
    if (!strstr(c->recv_buf, "\r\n\r\n"))
        return;

    char host[256] = "", origin[256] = "";
    int has_host = http_header(c->recv_buf, "Host", host, sizeof(host));
    int has_origin = http_header(c->recv_buf, "Origin", origin, sizeof(origin));
    if (has_host < 0 || has_origin < 0) {
        refuse(srv, idx, "oversized", has_host < 0 ? "Host" : "Origin");
        return;
    }
    if (srv->loopback_only && has_host && !host_is_loopback(host)) {
        refuse(srv, idx, "non-loopback Host", host);
        return;
    }

    /* WebSocket upgrade: GET /ws */
    if (strstr(c->recv_buf, "GET /ws") && strstr(c->recv_buf, "Upgrade: websocket")) {
        if (has_origin && !(has_host && origin_matches_host(origin, host))) {
            refuse(srv, idx, "cross-origin WebSocket from", origin);
            return;
        }
        /* Extract Sec-WebSocket-Key */
        const char *key_hdr = strstr(c->recv_buf, "Sec-WebSocket-Key: ");
        if (!key_hdr) { close_client(srv, idx); return; }
        key_hdr += 19;
        const char *key_end = strstr(key_hdr, "\r\n");
        if (!key_end) { close_client(srv, idx); return; }
        int key_len = (int)(key_end - key_hdr);

        /* SHA-1(key + magic) */
        sha1_ctx sha;
        sha1_init(&sha);
        sha1_update(&sha, (const uint8_t *)key_hdr, (uint32_t)key_len);
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
        send(c->fd, resp, rlen, 0);
        c->state = CLIENT_WS;
        c->recv_len = 0;
        return;
    }

    /* Serve HTML on GET / */
    if (strstr(c->recv_buf, "GET / ") || strstr(c->recv_buf, "GET /index.html")) {
        const char *body = srv->html ? srv->html : "<html><body>No UI loaded</body></html>";
        int blen = srv->html ? srv->html_len : (int)strlen(body);
        char hdr[256];
        int hlen = snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Cache-Control: no-cache, no-store\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n", blen);
        send(c->fd, hdr, hlen, 0);
        send(c->fd, body, blen, 0);
        close_client(srv, idx);
        return;
    }

    /* 404 for everything else */
    const char *resp = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    send(c->fd, resp, (int)strlen(resp), 0);
    close_client(srv, idx);
}

static void handle_ws_frame(ws_server_t *srv, int idx) {
    ws_client_t *c = &srv->clients[idx];
    while (c->recv_len >= 2) {
        uint8_t *buf = (uint8_t *)c->recv_buf;
        uint8_t opcode = buf[0] & 0x0F;
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
            send(c->fd, close_frame, 2, 0);
            close_client(srv, idx);
            return;
        } else if (opcode == 0x9) {
            /* Ping — respond with pong */
            uint8_t pong[2] = { 0x8A, (uint8_t)(payload_len & 0x7F) };
            send(c->fd, pong, 2, 0);
            if (payload_len > 0)
                send(c->fd, buf + header_len, (int)payload_len, 0);
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

ws_server_t *ws_server_init(const char *bind_addr, int port) {
    struct in_addr bind_in;
    if (!bind_addr || strcmp(bind_addr, "localhost") == 0)
        bind_addr = "127.0.0.1";
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

    if (srv->loopback_only)
        printf("WebSocket UI server listening on http://localhost:%d\n", port);
    else
        printf("WebSocket UI server listening on %s:%d (NOT loopback-only: "
               "reachable from the network)\n", bind_addr, port);
    return srv;
}

void ws_server_poll(ws_server_t *srv) {
    if (!srv) return;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(srv->listen_fd, &read_fds);
    int max_fd = srv->listen_fd;

    for (int i = 0; i < srv->client_count; i++) {
        FD_SET(srv->clients[i].fd, &read_fds);
        if (srv->clients[i].fd > max_fd)
            max_fd = srv->clients[i].fd;
    }

    struct timeval tv = { 0, 0 }; /* non-blocking */
    int ready = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
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
                ws_client_t *c = &srv->clients[srv->client_count++];
                c->fd = client_fd;
                c->state = CLIENT_HTTP;
                c->recv_len = 0;
            } else {
                close(client_fd);
            }
        }
    }

    /* Read from existing clients */
    for (int i = 0; i < srv->client_count; i++) {
        if (!FD_ISSET(srv->clients[i].fd, &read_fds))
            continue;

        ws_client_t *c = &srv->clients[i];
        int space = RECV_BUF - c->recv_len - 1;
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

/* Send all bytes, retrying on partial writes.  Returns 0 on success, -1 on error. */
static int send_all(int fd, const void *buf, int len) {
    const uint8_t *p = (const uint8_t *)buf;
    int remaining = len;
    while (remaining > 0) {
        ssize_t n = send(fd, p, remaining, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* Brief spin for non-blocking socket — data should drain quickly */
                usleep(100);
                continue;
            }
            return -1;
        }
        p += n;
        remaining -= (int)n;
    }
    return 0;
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

void ws_server_broadcast(ws_server_t *srv, const char *data, int len) {
    if (!srv || len <= 0) return;

    uint8_t header[10];
    int hlen = ws_build_header(header, 0x01, len); /* text */

    for (int i = 0; i < srv->client_count; i++) {
        if (srv->clients[i].state != CLIENT_WS)
            continue;
        if (send_all(srv->clients[i].fd, header, hlen) < 0 ||
            send_all(srv->clients[i].fd, data, len) < 0) {
            close_client(srv, i); i--;
        }
    }
}

void ws_server_broadcast_binary(ws_server_t *srv, const uint8_t *data, int len) {
    if (!srv || len <= 0) return;

    uint8_t header[10];
    int hlen = ws_build_header(header, 0x02, len); /* binary */

    for (int i = 0; i < srv->client_count; i++) {
        if (srv->clients[i].state != CLIENT_WS)
            continue;
        if (send_all(srv->clients[i].fd, header, hlen) < 0 ||
            send_all(srv->clients[i].fd, data, len) < 0) {
            close_client(srv, i); i--;
        }
    }
}

void ws_server_set_message_callback(ws_server_t *srv, ws_message_cb_t cb, void *userdata) {
    if (!srv) return;
    srv->msg_cb = cb;
    srv->msg_userdata = userdata;
}

void ws_server_set_html(ws_server_t *srv, const char *html, int len) {
    if (!srv) return;
    free(srv->html);
    srv->html = malloc(len + 1);
    if (srv->html) {
        memcpy(srv->html, html, len);
        srv->html[len] = '\0';
        srv->html_len = len;
    }
}

int ws_server_client_count(ws_server_t *srv) {
    return srv ? srv->client_count : 0;
}

void ws_server_destroy(ws_server_t *srv) {
    if (!srv) return;
    for (int i = 0; i < srv->client_count; i++)
        close(srv->clients[i].fd);
    close(srv->listen_fd);
    free(srv->html);
    free(srv);
}
