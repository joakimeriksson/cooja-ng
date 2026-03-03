/*
 * Minimal non-blocking WebSocket server for simulation UI
 *
 * Single-threaded, POSIX sockets, select()-based polling.
 * Serves embedded HTML on GET / and upgrades GET /ws to WebSocket.
 * Supports up to 8 concurrent WebSocket clients.
 */
#ifndef WS_SERVER_H
#define WS_SERVER_H

typedef struct ws_server ws_server_t;

/* Create and bind a WebSocket server on the given port. Returns NULL on failure. */
ws_server_t *ws_server_init(int port);

/* Non-blocking poll: accept new connections, read incoming data, handle close/ping. */
void ws_server_poll(ws_server_t *srv);

/* Send a text WebSocket frame to all connected clients. */
void ws_server_broadcast(ws_server_t *srv, const char *data, int len);

/* Set the HTML content to serve on GET /. The data is copied internally. */
void ws_server_set_html(ws_server_t *srv, const char *html, int len);

/* Shut down all connections and free resources. */
void ws_server_destroy(ws_server_t *srv);

#endif /* WS_SERVER_H */
