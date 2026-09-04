/*
 * ext_node — external data-driven mote: the node's behaviour lives in a
 * separate process that speaks NDJSON over stdio in simulation time.
 *
 * Protocol: docs/design/external-nodes-plan.md §4.  csim owns the clock and
 * drives the exchange; the peer only ever answers.
 *
 *   csim -> peer   {"type":"hello","proto":1,"id":3,"x":..,"y":..,"seed":..}
 *   csim -> peer   {"type":"step","t":<ns>,"in":[]}
 *   peer -> csim   {"type":"done","t":<ns>,"wake":<ns|null>,"out":[...]}
 *   csim -> peer   {"type":"stop","t":<ns>,"reason":"..."}
 *
 * Output events handled: `tx` (whole 802.15.4 frame, hex), `log` (console
 * line) and `wake`.  Inputs carried in `step.in`: `rx` (a received frame with
 * its sender, channel and per-receiver RSSI).  `serial` input is still to
 * come.
 *
 * This is the js_node.c shape without a script engine: the "interpreter" is
 * one line written and one line read per execute slice.
 */
#ifndef EXT_NODE_H
#define EXT_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frames are handed to the runner's frame hook, which PHY-wraps into a
 * 160-byte buffer (4 preamble + SFD + len + frame + 2 CRC).  Anything
 * larger is dropped *silently* by native_frame_to_bytes(), so the engine
 * rejects it loudly instead. */
#define EXT_NODE_MAX_FRAME 152

#define EXT_NODE_LINE_MAX  8192

/* Frames waiting to be handed to the peer on its next step.  Same depth as
 * the JS mote's queue; a full queue drops the frame and says so, which is
 * what a real radio does when its FIFO overruns. */
#define EXT_NODE_RX_QUEUE  16

typedef struct ext_node_rx {
    int64_t arrival_ns;
    int     from_id;      /* sender's node id, -1 if unknown */
    int     channel;      /* sender's channel, -1 if unknown */
    int8_t  rssi;         /* per-receiver, from the medium */
    int     len;
    uint8_t frame[EXT_NODE_MAX_FRAME];
} ext_node_rx_t;

typedef struct ext_node {
    int      node_id;
    int64_t  sim_time_ns;      /* where the peer stopped: `done.t`, or the
                                * step time when it did not say (js_node rule) */
    int64_t  next_wakeup_ns;   /* INT64_MAX = never wake again              */

    pid_t    child_pid;
    int      to_child;         /* write end of the peer's stdin  */
    int      from_child;       /* read end of the peer's stdout  */
    bool     failed;           /* sticky: peer died / protocol violation    */

    int      timeout_ms;       /* CSIM_EXT_TIMEOUT_MS, default 5000         */

    /* Pending RX, oldest first (ring). */
    ext_node_rx_t rx_queue[EXT_NODE_RX_QUEUE];
    int      rx_head;
    int      rx_count;
    int      rx_dropped;       /* queue-full drops, reported at teardown */

    /* Partial-line accumulator for the reader. */
    char     rbuf[EXT_NODE_LINE_MAX];
    int      rlen;

    char     command[512];     /* for diagnostics */

    /* Console: one UART byte at a time (user_data = the mote). */
    void   (*log_callback)(void *user_data, uint8_t byte);
    void    *log_callback_data;

    /* Whole-frame TX (user_data = the mote). */
    void   (*rf_frame_callback)(void *user_data, const uint8_t *frame, int len);
    void    *rf_frame_callback_data;
} ext_node_t;

/* Spawn the peer.  `path` is the executable (run via /bin/sh so a shebang
 * script or a command with arguments both work).  Returns 0, or -1 with a
 * message on stderr. */
int  ext_node_init(ext_node_t *node, const char *path, int node_id);

/* Send `hello` and consume the peer's first `done` (which carries its first
 * wakeup).  Call after the callbacks are wired. */
int  ext_node_start(ext_node_t *node, double x, double y, uint32_t seed);

/* Run the peer up to target_ns: one step exchange per due wakeup. */
void ext_node_step_until_ns(ext_node_t *node, int64_t target_ns);

/* Queue a received frame for delivery to the peer as an `rx` input.
 * Returns 0, or -1 if the queue was full (frame dropped). */
int  ext_node_deliver_frame(ext_node_t *node, const uint8_t *frame, int len,
                            int64_t arrival_ns, int from_id, int channel,
                            int8_t rssi);

int64_t ext_node_next_wakeup_ns(const ext_node_t *node);

/* True once the peer died or broke the protocol.  The node produces no
 * further behaviour, so the run must not be allowed to pass. */
bool ext_node_failed(const ext_node_t *node);

/* Send `stop`, then SIGTERM/SIGKILL and reap. */
void ext_node_destroy(ext_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* EXT_NODE_H */
