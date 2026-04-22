/*
 * JavaScript application mote — see js_node.h.
 *
 * Single-threaded synchronous model: each node has its own QuickJS
 * runtime+context, and is ticked from the main sim loop. No pthreads,
 * no condvars. Distinct from src/common/js_test_engine.c which is the
 * Cooja test-script harness.
 */
#include "js_node.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define NODE_OPAQUE_TAG ((void *)(uintptr_t)0x6A734E6F /* 'jsNo' */)

/* Retrieve js_node_t* from a JSContext (set via JS_SetContextOpaque). */
static js_node_t *node_from_ctx(JSContext *ctx) {
    return (js_node_t *)JS_GetContextOpaque(ctx);
}

/* --- mote.log(str): synthesize bytes through log_callback --- */
static JSValue js_mote_log(JSContext *ctx, JSValueConst this_val,
                           int argc, JSValueConst *argv) {
    (void)this_val;
    js_node_t *node = node_from_ctx(ctx);
    if (argc < 1) return JS_UNDEFINED;
    const char *s = JS_ToCString(ctx, argv[0]);
    if (!s) return JS_EXCEPTION;
    if (node->log_callback) {
        for (const char *p = s; *p; p++)
            node->log_callback(node->log_callback_data, (uint8_t)*p);
        node->log_callback(node->log_callback_data, (uint8_t)'\n');
    } else {
        printf("[js node %d] %s\n", node->node_id, s);
    }
    JS_FreeCString(ctx, s);
    return JS_UNDEFINED;
}

/* --- mote.send(uint8array): emit raw 802.15.4 MAC frame --- */
static JSValue js_mote_send(JSContext *ctx, JSValueConst this_val,
                            int argc, JSValueConst *argv) {
    (void)this_val;
    js_node_t *node = node_from_ctx(ctx);
    if (argc < 1)
        return JS_ThrowTypeError(ctx, "mote.send(bytes): missing argument");

    size_t byte_offset = 0, byte_length = 0, bytes_per_element = 0;
    JSValue ab = JS_GetTypedArrayBuffer(ctx, argv[0],
                                        &byte_offset, &byte_length,
                                        &bytes_per_element);
    uint8_t *buf = NULL;
    int     len = 0;
    size_t  ab_size = 0;

    if (!JS_IsException(ab)) {
        uint8_t *raw = JS_GetArrayBuffer(ctx, &ab_size, ab);
        if (raw) {
            buf = raw + byte_offset;
            len = (int)byte_length;
        }
        JS_FreeValue(ctx, ab);
    } else {
        /* Not a typed array — try plain array of numbers */
        JS_FreeValue(ctx, JS_GetException(ctx));
        JSValue lenv = JS_GetPropertyStr(ctx, argv[0], "length");
        int32_t n = 0;
        if (JS_ToInt32(ctx, &n, lenv) == 0 && n >= 0 && n < JS_NODE_FRAME_MAX) {
            static uint8_t scratch[JS_NODE_FRAME_MAX];
            for (int32_t i = 0; i < n; i++) {
                JSValue v = JS_GetPropertyUint32(ctx, argv[0], (uint32_t)i);
                int32_t b = 0;
                JS_ToInt32(ctx, &b, v);
                JS_FreeValue(ctx, v);
                scratch[i] = (uint8_t)(b & 0xFF);
            }
            buf = scratch;
            len = n;
        }
        JS_FreeValue(ctx, lenv);
    }

    if (!buf || len <= 0)
        return JS_ThrowTypeError(ctx, "mote.send: expected Uint8Array or array");
    if (len > JS_NODE_FRAME_MAX)
        return JS_ThrowRangeError(ctx, "mote.send: frame too long (max %d)",
                                  JS_NODE_FRAME_MAX);

    if (node->rf_frame_callback)
        node->rf_frame_callback(node->rf_frame_callback_data, buf, len);
    return JS_UNDEFINED;
}

/* --- mote.scheduleWakeup(ns_from_now) --- */
static JSValue js_mote_schedule_wakeup(JSContext *ctx, JSValueConst this_val,
                                       int argc, JSValueConst *argv) {
    (void)this_val;
    js_node_t *node = node_from_ctx(ctx);
    int64_t delta = 0;
    if (argc >= 1 && JS_ToInt64(ctx, &delta, argv[0]) < 0)
        return JS_EXCEPTION;
    if (delta < 0) delta = 0;
    int64_t when = node->sim_time_ns + delta;
    if (when < node->next_wakeup_ns)
        node->next_wakeup_ns = when;
    return JS_UNDEFINED;
}

/* --- mote.now(): return sim_time_ns as Number --- */
static JSValue js_mote_now(JSContext *ctx, JSValueConst this_val,
                           int argc, JSValueConst *argv) {
    (void)this_val; (void)argc; (void)argv;
    js_node_t *node = node_from_ctx(ctx);
    return JS_NewInt64(ctx, node->sim_time_ns);
}

/* --- Build the 'mote' global --- */
static void install_mote_global(js_node_t *node) {
    JSContext *ctx = node->ctx;
    JSValue global = JS_GetGlobalObject(ctx);

    JSValue mote = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, mote, "id",  JS_NewInt32(ctx, node->node_id));
    JS_SetPropertyStr(ctx, mote, "log",
        JS_NewCFunction(ctx, js_mote_log,             "log",  1));
    JS_SetPropertyStr(ctx, mote, "send",
        JS_NewCFunction(ctx, js_mote_send,            "send", 1));
    JS_SetPropertyStr(ctx, mote, "scheduleWakeup",
        JS_NewCFunction(ctx, js_mote_schedule_wakeup, "scheduleWakeup", 1));
    JS_SetPropertyStr(ctx, mote, "now",
        JS_NewCFunction(ctx, js_mote_now,             "now",  0));

    JS_SetPropertyStr(ctx, global, "mote", mote);
    JS_FreeValue(ctx, global);
}

/* --- Read entire file into malloc'd null-terminated buffer --- */
static char *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n < 0) { fclose(f); return NULL; }
    char *buf = malloc((size_t)n + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)n, f);
    fclose(f);
    buf[got] = '\0';
    if (out_len) *out_len = got;
    return buf;
}

static void log_exception(JSContext *ctx, const char *where) {
    JSValue exc = JS_GetException(ctx);
    const char *s = JS_ToCString(ctx, exc);
    fprintf(stderr, "js_node: %s threw: %s\n", where, s ? s : "(unknown)");
    if (s) JS_FreeCString(ctx, s);
    JSValue stack = JS_GetPropertyStr(ctx, exc, "stack");
    if (!JS_IsUndefined(stack)) {
        const char *st = JS_ToCString(ctx, stack);
        if (st) { fprintf(stderr, "%s\n", st); JS_FreeCString(ctx, st); }
    }
    JS_FreeValue(ctx, stack);
    JS_FreeValue(ctx, exc);
}

/* --- Public API --- */

int js_node_init(js_node_t *node, const char *script_path, int node_id) {
    memset(node, 0, sizeof(*node));
    node->node_id        = node_id;
    node->sim_time_ns    = 0;
    node->next_wakeup_ns = INT64_MAX;
    node->js_execute        = JS_UNDEFINED;
    node->js_received_packet = JS_UNDEFINED;

    node->rt = JS_NewRuntime();
    if (!node->rt) {
        fprintf(stderr, "js_node: JS_NewRuntime failed\n");
        return -1;
    }
    node->ctx = JS_NewContext(node->rt);
    if (!node->ctx) {
        fprintf(stderr, "js_node: JS_NewContext failed\n");
        JS_FreeRuntime(node->rt);
        return -1;
    }
    JS_SetContextOpaque(node->ctx, node);
    (void)NODE_OPAQUE_TAG;

    install_mote_global(node);

    size_t script_len = 0;
    char *script = read_file(script_path, &script_len);
    if (!script) {
        fprintf(stderr, "js_node: cannot read script: %s\n", script_path);
        js_node_destroy(node);
        return -1;
    }

    JSValue r = JS_Eval(node->ctx, script, script_len, script_path,
                        JS_EVAL_TYPE_GLOBAL);
    free(script);
    if (JS_IsException(r)) {
        log_exception(node->ctx, "script load");
        JS_FreeValue(node->ctx, r);
        js_node_destroy(node);
        return -1;
    }
    JS_FreeValue(node->ctx, r);

    /* Resolve handler functions on globalThis */
    JSValue global = JS_GetGlobalObject(node->ctx);
    node->js_execute         = JS_GetPropertyStr(node->ctx, global, "execute");
    node->js_received_packet = JS_GetPropertyStr(node->ctx, global,
                                                 "receivedPacket");
    JS_FreeValue(node->ctx, global);

    /* init() is run later by js_node_start(), after host wires callbacks. */
    return 0;
}

void js_node_start(js_node_t *node) {
    JSValue global = JS_GetGlobalObject(node->ctx);
    JSValue init_fn = JS_GetPropertyStr(node->ctx, global, "init");
    JS_FreeValue(node->ctx, global);

    if (JS_IsFunction(node->ctx, init_fn)) {
        JSValue ir = JS_Call(node->ctx, init_fn, JS_UNDEFINED, 0, NULL);
        if (JS_IsException(ir)) log_exception(node->ctx, "init()");
        JS_FreeValue(node->ctx, ir);
    }
    JS_FreeValue(node->ctx, init_fn);

    /* If user didn't schedule a wakeup, schedule one at current sim time
     * so execute() runs at least once. */
    if (node->next_wakeup_ns == INT64_MAX
        && JS_IsFunction(node->ctx, node->js_execute))
        node->next_wakeup_ns = node->sim_time_ns;
}

void js_node_destroy(js_node_t *node) {
    if (!node) return;
    if (node->ctx) {
        JS_FreeValue(node->ctx, node->js_execute);
        JS_FreeValue(node->ctx, node->js_received_packet);
        JS_FreeContext(node->ctx);
        node->ctx = NULL;
    }
    if (node->rt) {
        JS_FreeRuntime(node->rt);
        node->rt = NULL;
    }
}

void js_node_deliver_frame(js_node_t *node, const uint8_t *frame, int len,
                           int64_t arrival_ns) {
    if (node->rx_count >= JS_NODE_RX_QUEUE_SIZE) return;  /* drop */
    if (len > JS_NODE_FRAME_MAX) len = JS_NODE_FRAME_MAX;
    int slot = (node->rx_head + node->rx_count) % JS_NODE_RX_QUEUE_SIZE;
    js_node_pending_frame_t *f = &node->rx_queue[slot];
    memcpy(f->data, frame, (size_t)len);
    f->len = len;
    f->arrival_ns = arrival_ns;
    node->rx_count++;
}

static void dispatch_rx_frame(js_node_t *node,
                              const js_node_pending_frame_t *f) {
    if (!JS_IsFunction(node->ctx, node->js_received_packet)) return;

    /* Wrap the frame as new Uint8Array(buffer) */
    JSValue arr    = JS_NewArrayBufferCopy(node->ctx, f->data, (size_t)f->len);
    JSValue global = JS_GetGlobalObject(node->ctx);
    JSValue u8     = JS_GetPropertyStr(node->ctx, global, "Uint8Array");
    JSValue ctor_args[1] = { arr };
    JSValue u8arr  = JS_CallConstructor(node->ctx, u8, 1, ctor_args);
    JS_FreeValue(node->ctx, u8);
    JS_FreeValue(node->ctx, global);
    JS_FreeValue(node->ctx, arr);

    JSValue cb_args[1] = { u8arr };
    JSValue r = JS_Call(node->ctx, node->js_received_packet, JS_UNDEFINED,
                        1, cb_args);
    if (JS_IsException(r)) log_exception(node->ctx, "receivedPacket()");
    JS_FreeValue(node->ctx, r);
    JS_FreeValue(node->ctx, u8arr);
}

static void dispatch_execute(js_node_t *node, int64_t time_ns) {
    if (!JS_IsFunction(node->ctx, node->js_execute)) return;
    JSValue args[1] = { JS_NewInt64(node->ctx, time_ns) };
    JSValue r = JS_Call(node->ctx, node->js_execute, JS_UNDEFINED, 1, args);
    JS_FreeValue(node->ctx, args[0]);
    if (JS_IsException(r)) log_exception(node->ctx, "execute()");
    JS_FreeValue(node->ctx, r);
}

void js_node_step_until_ns(js_node_t *node, int64_t target_ns) {
    /* Dispatch all pending events with time <= target_ns, including those
     * scheduled at the current sim_time_ns. */
    while (1) {
        int64_t next_rx_ns = INT64_MAX;
        if (node->rx_count > 0)
            next_rx_ns = node->rx_queue[node->rx_head].arrival_ns;
        int64_t next_ev = node->next_wakeup_ns;
        if (next_rx_ns < next_ev) next_ev = next_rx_ns;
        if (next_ev > target_ns) break;

        if (next_ev > node->sim_time_ns)
            node->sim_time_ns = next_ev;

        if (next_rx_ns <= node->next_wakeup_ns) {
            js_node_pending_frame_t f = node->rx_queue[node->rx_head];
            node->rx_head = (node->rx_head + 1) % JS_NODE_RX_QUEUE_SIZE;
            node->rx_count--;
            dispatch_rx_frame(node, &f);
        } else {
            int64_t when = node->next_wakeup_ns;
            node->next_wakeup_ns = INT64_MAX;
            dispatch_execute(node, when);
        }
    }

    if (target_ns > node->sim_time_ns)
        node->sim_time_ns = target_ns;
}

int64_t js_node_next_wakeup_ns(const js_node_t *node) {
    int64_t next_rx_ns = INT64_MAX;
    if (node->rx_count > 0)
        next_rx_ns = node->rx_queue[node->rx_head].arrival_ns;
    int64_t r = node->next_wakeup_ns;
    if (next_rx_ns < r) r = next_rx_ns;
    return r;
}
