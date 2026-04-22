/*
 * JavaScript application mote — abstract-level node scripted in QuickJS.
 *
 * Equivalent to Cooja's AbstractApplicationMote (Java mote level), but
 * scripted in JavaScript. Each js_node has its own QuickJS context and
 * runs a user-supplied script that defines:
 *
 *   function execute(time_ns)        -- scheduled wakeup callback
 *   function receivedPacket(bytes)   -- incoming 802.15.4 MAC frame
 *
 * The script can call:
 *   mote.id                          -- node id (number)
 *   mote.now()                       -- sim time in ns (number)
 *   mote.log(string)                 -- write line to virtual UART
 *   mote.send(uint8array)            -- transmit raw 802.15.4 MAC frame
 *   mote.scheduleWakeup(ns_from_now) -- schedule next execute() call
 */
#ifndef JS_NODE_H
#define JS_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "quickjs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JS_NODE_RX_QUEUE_SIZE 8
#define JS_NODE_FRAME_MAX     128

typedef struct js_node_pending_frame {
    uint8_t data[JS_NODE_FRAME_MAX];
    int     len;
    int64_t arrival_ns;
} js_node_pending_frame_t;

typedef struct js_node {
    JSRuntime *rt;
    JSContext *ctx;
    int        node_id;
    int64_t    sim_time_ns;
    int64_t    next_wakeup_ns;   /* INT64_MAX if none scheduled */

    /* Resolved JS handler functions (JS_UNDEFINED if absent) */
    JSValue js_execute;
    JSValue js_received_packet;

    /* RX queue */
    js_node_pending_frame_t rx_queue[JS_NODE_RX_QUEUE_SIZE];
    int rx_head, rx_count;

    /* Console line buffer (for parity with other mote types) */
    char line_buf[256];
    int  line_pos;

    /* Host callbacks */
    void (*log_callback)(void *user_data, uint8_t byte);
    void *log_callback_data;

    void (*rf_frame_callback)(void *user_data, const uint8_t *frame, int len);
    void *rf_frame_callback_data;
} js_node_t;

/* Load script, create context, install bindings, resolve handler functions.
 * Does NOT run init() — call js_node_start() after wiring callbacks. */
int  js_node_init(js_node_t *node, const char *script_path, int node_id);

/* Run the user's optional init() and ensure a wakeup is scheduled. Call
 * after js_node_init() and after registering log_callback / rf_frame_callback. */
void js_node_start(js_node_t *node);

/* Free QuickJS context/runtime. */
void js_node_destroy(js_node_t *node);

/* Advance node to target_ns: dispatch RX frames whose arrival_ns <= target,
 * then fire scheduled wakeup if its time has come. */
void js_node_step_until_ns(js_node_t *node, int64_t target_ns);

/* Enqueue a frame for delivery at arrival_ns. */
void js_node_deliver_frame(js_node_t *node, const uint8_t *frame, int len,
                           int64_t arrival_ns);

/* Earliest pending event time (RX or wakeup). INT64_MAX if idle. */
int64_t js_node_next_wakeup_ns(const js_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* JS_NODE_H */
