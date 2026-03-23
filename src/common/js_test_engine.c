/*
 * JavaScript test engine — runs COOJA test scripts via QuickJS.
 *
 * COOJA scripts use a blocking coroutine model:
 *   while(true) { YIELD(); if(msg.contains("X")) log.testOK(); }
 *
 * We run the JS in a separate pthread. YIELD() blocks on a condvar
 * until the main sim thread feeds the next console line.
 */
#include "js_test_engine.h"
#include "quickjs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

/* Only one JS engine runs at a time — store script for thread pickup */
static char *s_pending_script = NULL;

/* --- Script preprocessing ---
 * TIMEOUT(ms, code) is a COOJA macro where 'code' is NOT evaluated as
 * a JS argument — it's lazily evaluated at timeout. We transform:
 *   TIMEOUT(ms, code)  →  TIMEOUT(ms); var __timeout_cb = function() { code };
 * This prevents immediate evaluation of log.testFailed() etc.
 */
/* Find matching close-paren for an open-paren at *start, handling nesting.
 * Also optionally finds the first comma at depth 1. */
static const char *find_close_paren(const char *start, const char **comma_out) {
    int depth = 1;
    const char *c = start;
    if (comma_out) *comma_out = NULL;
    while (*c && depth > 0) {
        if (*c == '(') depth++;
        else if (*c == ')') { depth--; if (depth == 0) return c; }
        else if (*c == ',' && depth == 1 && comma_out && !*comma_out) *comma_out = c;
        /* Skip string literals */
        else if (*c == '"' || *c == '\'') {
            char q = *c++;
            while (*c && *c != q) { if (*c == '\\') c++; c++; }
        }
        c++;
    }
    return NULL;
}

static char *preprocess_script(const char *script) {
    /* We need to transform:
     * 1. TIMEOUT(ms, callback) → TIMEOUT(ms); var __timeout_cb = function() { callback };
     * 2. WAIT_UNTIL(expr) → WAIT_UNTIL("expr")  (since COOJA treats it as a macro)
     *
     * Work on a mutable copy, applying transforms iteratively. */
    size_t cap = strlen(script) * 2 + 256;
    char *out = malloc(cap);
    if (!out) return strdup(script);
    out[0] = '\0';
    size_t out_len = 0;

    const char *pos = script;
    while (*pos) {
        /* Look for TIMEOUT( or WAIT_UNTIL( */
        const char *t_timeout = strstr(pos, "TIMEOUT(");
        const char *t_waituntil = strstr(pos, "WAIT_UNTIL(");

        /* Pick the earliest match */
        const char *match = NULL;
        int is_timeout = 0, is_waituntil = 0;
        if (t_timeout && (!t_waituntil || t_timeout < t_waituntil)) {
            match = t_timeout; is_timeout = 1;
        } else if (t_waituntil) {
            match = t_waituntil; is_waituntil = 1;
        }

        if (!match) {
            /* No more macros — copy rest */
            size_t rest = strlen(pos);
            if (out_len + rest >= cap) { cap = out_len + rest + 1; out = realloc(out, cap); }
            memcpy(out + out_len, pos, rest);
            out_len += rest;
            out[out_len] = '\0';
            break;
        }

        /* Copy text before the match */
        size_t prefix = (size_t)(match - pos);
        if (out_len + prefix >= cap) { cap = (out_len + prefix) * 2; out = realloc(out, cap); }
        memcpy(out + out_len, pos, prefix);
        out_len += prefix;

        if (is_timeout) {
            const char *args = match + 8; /* after "TIMEOUT(" */
            const char *comma = NULL;
            const char *close = find_close_paren(args, &comma);
            if (!close) { /* malformed — copy as-is */
                memcpy(out + out_len, match, 8); out_len += 8;
                pos = match + 8; continue;
            }

            if (comma) {
                /* TIMEOUT(ms, callback) → TIMEOUT(ms); var __timeout_cb = ... */
                size_t ms_len = (size_t)(comma - args);
                const char *cb = comma + 1;
                while (*cb == ' ') cb++;
                size_t cb_len = (size_t)(close - cb);
                size_t need = 60 + ms_len + cb_len;
                if (out_len + need >= cap) { cap = (out_len + need) * 2; out = realloc(out, cap); }
                out_len += (size_t)snprintf(out + out_len, cap - out_len,
                    "TIMEOUT(%.*s);\nvar __timeout_cb = function() { %.*s };",
                    (int)ms_len, args, (int)cb_len, cb);
            } else {
                /* TIMEOUT(ms) — copy as-is */
                size_t span = (size_t)(close - match + 1);
                if (out_len + span >= cap) { cap = (out_len + span) * 2; out = realloc(out, cap); }
                memcpy(out + out_len, match, span);
                out_len += span;
            }
            pos = close + 1;

        } else if (is_waituntil) {
            const char *args = match + 11; /* after "WAIT_UNTIL(" */
            const char *close = find_close_paren(args, NULL);
            if (!close) {
                memcpy(out + out_len, match, 11); out_len += 11;
                pos = match + 11; continue;
            }
            /* WAIT_UNTIL(expr) → WAIT_UNTIL("expr") — quote the expression
             * so it's passed as a string and eval'd on each YIELD. */
            size_t expr_len = (size_t)(close - args);
            /* Escape any quotes in the expression */
            size_t need = 20 + expr_len * 2;
            if (out_len + need >= cap) { cap = (out_len + need) * 2; out = realloc(out, cap); }
            out_len += (size_t)snprintf(out + out_len, cap - out_len, "WAIT_UNTIL(\"");
            for (const char *e = args; e < close; e++) {
                if (*e == '"') { out[out_len++] = '\\'; out[out_len++] = '"'; }
                else if (*e == '\\') { out[out_len++] = '\\'; out[out_len++] = '\\'; }
                else out[out_len++] = *e;
            }
            out_len += (size_t)snprintf(out + out_len, cap - out_len, "\")");
            pos = close + 1;
        }
    }

    out[out_len] = '\0';
    return out;
}

/* ---- Engine pointer from JSContext ---- */

static js_test_engine_t *get_engine(JSContext *ctx) {
    return (js_test_engine_t *)JS_GetContextOpaque(ctx);
}

/* ---- Queue an action (called from JS thread, under mutex) ---- */

static void queue_action(js_test_engine_t *e, const sim_test_action_t *act) {
    if (e->pending_action_count < JS_MAX_PENDING_ACTIONS)
        e->pending_actions[e->pending_action_count++] = *act;
}

/* ---- Forward declarations ---- */

static JSValue js_mote_get_id(JSContext *ctx, JSValueConst this_val,
                              int argc, JSValueConst *argv);
static JSValue js_sim_get_time_us(JSContext *ctx, JSValueConst this_val,
                                  int argc, JSValueConst *argv);
static void update_globals(JSContext *ctx, js_test_engine_t *e);
static JSValue make_mote_obj(JSContext *ctx, int node_id);
static JSValue js_mote_get_interfaces(JSContext *ctx, JSValueConst this_val,
                                      int argc, JSValueConst *argv);

/* ---- Internal: block until next line (used by YIELD and WAIT_UNTIL) ---- */

/* Returns 0 on success, -1 on shutdown. Must be called WITHOUT mutex held.
 * After returning 0, the JS globals (msg, id, time, mote) have been updated. */
static int yield_wait(js_test_engine_t *e, JSContext *ctx) {
    pthread_mutex_lock(&e->mutex);
    e->js_waiting = true;
    pthread_cond_signal(&e->line_done);

    while (!e->has_line && !e->shutdown)
        pthread_cond_wait(&e->line_ready, &e->mutex);

    e->has_line = false;
    e->js_waiting = false;
    int shutdown = e->shutdown;
    bool do_timeout = e->timeout_fired;
    if (do_timeout) e->timeout_fired = false;
    pthread_mutex_unlock(&e->mutex);

    if (!shutdown) {
        update_globals(ctx, e);

        /* If timeout fired, eval __timeout_cb() before returning to script */
        if (do_timeout) {
            const char *cb_code = "if(typeof __timeout_cb === 'function') __timeout_cb();";
            JSValue r = JS_Eval(ctx, cb_code, strlen(cb_code),
                               "<timeout_cb>", JS_EVAL_TYPE_GLOBAL);
            if (JS_IsException(r)) {
                /* testOK/testFailed threw — result already set via log.testOK/Failed */
                JS_FreeValue(ctx, r);
                JSValue exc = JS_GetException(ctx);
                JS_FreeValue(ctx, exc);
                /* Signal main thread that we're done */
                pthread_mutex_lock(&e->mutex);
                e->js_waiting = true;
                pthread_cond_signal(&e->line_done);
                pthread_mutex_unlock(&e->mutex);
                return -1; /* will cause JS_ThrowInternalError upstream */
            }
            JS_FreeValue(ctx, r);
            /* If callback didn't set a result, default to fail */
            if (e->finished == 0) {
                pthread_mutex_lock(&e->mutex);
                e->finished = -1;
                snprintf(e->fail_reason, sizeof(e->fail_reason),
                         "TIMEOUT callback did not set result");
                e->js_waiting = true;
                pthread_cond_signal(&e->line_done);
                pthread_mutex_unlock(&e->mutex);
                return -1;
            }
        }
    }

    return shutdown ? -1 : 0;
}

/* ---- COOJA API: TIMEOUT(ms [, callback]) ---- */

static JSValue js_timeout(JSContext *ctx, JSValueConst this_val,
                          int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    int64_t ms;
    if (JS_ToInt64(ctx, &ms, argv[0]) < 0)
        return JS_EXCEPTION;
    e->timeout_us = ms * 1000LL;
    /* Timeout callback handling is done by the main thread in
     * js_test_check_timeout(). Default behavior: timeout = test failed. */
    return JS_UNDEFINED;
}

/* ---- COOJA API: YIELD() ---- */

static JSValue js_yield(JSContext *ctx, JSValueConst this_val,
                        int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    if (yield_wait(e, ctx) < 0)
        return JS_ThrowInternalError(ctx, "shutdown");
    return JS_UNDEFINED;
}

/* ---- COOJA API: WAIT_UNTIL(condition_string_or_function) ---- */

static JSValue js_wait_until(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    while (1) {
        if (yield_wait(e, ctx) < 0)
            return JS_ThrowInternalError(ctx, "shutdown");

        JSValue result;
        if (JS_IsString(argv[0])) {
            const char *cond = JS_ToCString(ctx, argv[0]);
            if (!cond) return JS_EXCEPTION;
            result = JS_Eval(ctx, cond, strlen(cond), "<wait_until>",
                           JS_EVAL_TYPE_GLOBAL);
            JS_FreeCString(ctx, cond);
        } else {
            result = JS_Call(ctx, argv[0], JS_UNDEFINED, 0, NULL);
        }
        if (JS_IsException(result)) return JS_EXCEPTION;
        int bval = JS_ToBool(ctx, result);
        JS_FreeValue(ctx, result);
        if (bval) break;
    }
    return JS_UNDEFINED;
}

/* ---- COOJA API: GENERATE_MSG(ms, label) ---- */

static JSValue js_generate_msg(JSContext *ctx, JSValueConst this_val,
                               int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    int64_t ms;
    if (JS_ToInt64(ctx, &ms, argv[0]) < 0)
        return JS_EXCEPTION;
    const char *label = JS_ToCString(ctx, argv[1]);
    if (!label) return JS_EXCEPTION;

    if (e->gen_msg_count < JS_MAX_GEN_MSGS) {
        js_gen_msg_t *gm = &e->gen_msgs[e->gen_msg_count++];
        gm->at_us = ms * 1000LL;
        snprintf(gm->label, sizeof(gm->label), "%s", label);
    }
    JS_FreeCString(ctx, label);
    return JS_UNDEFINED;
}

/* ---- COOJA API: log.testOK() / log.testFailed() / log.log() ---- */

static JSValue js_log_test_ok(JSContext *ctx, JSValueConst this_val,
                              int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    pthread_mutex_lock(&e->mutex);
    if (e->finished == 0) {
        e->finished = 1;
        snprintf(e->fail_reason, sizeof(e->fail_reason), "log.testOK()");
    }
    e->js_waiting = true;
    pthread_cond_signal(&e->line_done);
    pthread_mutex_unlock(&e->mutex);
    return JS_ThrowInternalError(ctx, "testOK");
}

static JSValue js_log_test_failed(JSContext *ctx, JSValueConst this_val,
                                  int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    pthread_mutex_lock(&e->mutex);
    if (e->finished == 0) {
        e->finished = -1;
        snprintf(e->fail_reason, sizeof(e->fail_reason), "log.testFailed()");
    }
    e->js_waiting = true;
    pthread_cond_signal(&e->line_done);
    pthread_mutex_unlock(&e->mutex);
    return JS_ThrowInternalError(ctx, "testFailed");
}

static JSValue js_log_log(JSContext *ctx, JSValueConst this_val,
                          int argc, JSValueConst *argv) {
    if (argc > 0) {
        const char *str = JS_ToCString(ctx, argv[0]);
        if (str) {
            printf("[JS] %s\n", str);
            JS_FreeCString(ctx, str);
        }
    }
    return JS_UNDEFINED;
}

/* ---- COOJA API: write(mote_or_array, data) ---- */

static JSValue js_write(JSContext *ctx, JSValueConst this_val,
                        int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    if (argc < 2) return JS_UNDEFINED;

    const char *data = JS_ToCString(ctx, argv[1]);
    if (!data) return JS_EXCEPTION;

    sim_test_action_t act = {0};
    act.at_ms = e->time_us / 1000;
    snprintf(act.data, sizeof(act.data), "%s", data);
    JS_FreeCString(ctx, data);

    if (JS_IsArray(ctx, argv[0])) {
        act.type = TEST_ACTION_SEND_ALL;
    } else {
        act.type = TEST_ACTION_SEND;
        JSValue id_val = JS_GetPropertyStr(ctx, argv[0], "id");
        JS_ToInt32(ctx, &act.node, id_val);
        JS_FreeValue(ctx, id_val);
    }

    pthread_mutex_lock(&e->mutex);
    queue_action(e, &act);
    pthread_mutex_unlock(&e->mutex);
    return JS_UNDEFINED;
}

/* ---- COOJA API: sim object ---- */

static JSValue make_mote_obj(JSContext *ctx, int node_id) {
    JSValue mote = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, mote, "id", JS_NewInt32(ctx, node_id));
    JS_SetPropertyStr(ctx, mote, "getID",
        JS_NewCFunction(ctx, js_mote_get_id, "getID", 0));
    JS_SetPropertyStr(ctx, mote, "getInterfaces",
        JS_NewCFunction(ctx, js_mote_get_interfaces, "getInterfaces", 0));
    return mote;
}

static JSValue js_sim_get_motes(JSContext *ctx, JSValueConst this_val,
                                int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    JSValue arr = JS_NewArray(ctx);
    for (int i = 0; i < e->node_count; i++)
        JS_SetPropertyUint32(ctx, arr, (uint32_t)i,
                            make_mote_obj(ctx, e->node_ids[i]));
    return arr;
}

static JSValue js_sim_get_mote_with_id(JSContext *ctx, JSValueConst this_val,
                                       int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    int32_t target_id;
    if (JS_ToInt32(ctx, &target_id, argv[0]) < 0)
        return JS_EXCEPTION;
    /* Return null if node doesn't exist (scripts check if(!sim.getMoteWithID(n))) */
    for (int i = 0; i < e->node_count; i++)
        if (e->node_ids[i] == target_id)
            return make_mote_obj(ctx, target_id);
    return JS_NULL;
}

static JSValue js_mote_get_id(JSContext *ctx, JSValueConst this_val,
                              int argc, JSValueConst *argv) {
    return JS_GetPropertyStr(ctx, this_val, "id");
}

static JSValue js_sim_get_time_ms(JSContext *ctx, JSValueConst this_val,
                                  int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    return JS_NewInt64(ctx, e->time_us / 1000);
}

static JSValue js_sim_get_time_us(JSContext *ctx, JSValueConst this_val,
                                  int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    return JS_NewInt64(ctx, e->time_us);
}

static JSValue js_sim_remove_mote(JSContext *ctx, JSValueConst this_val,
                                  int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    JSValue id_val = JS_GetPropertyStr(ctx, argv[0], "id");
    sim_test_action_t act = {0};
    act.type = TEST_ACTION_REMOVE;
    act.at_ms = e->time_us / 1000;
    JS_ToInt32(ctx, &act.node, id_val);
    JS_FreeValue(ctx, id_val);
    pthread_mutex_lock(&e->mutex);
    queue_action(e, &act);
    pthread_mutex_unlock(&e->mutex);
    return JS_UNDEFINED;
}

static JSValue js_sim_add_mote(JSContext *ctx, JSValueConst this_val,
                               int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    JSValue id_val = JS_GetPropertyStr(ctx, argv[0], "id");
    sim_test_action_t act = {0};
    act.type = TEST_ACTION_ADD;
    act.at_ms = e->time_us / 1000;
    JS_ToInt32(ctx, &act.node, id_val);
    JS_FreeValue(ctx, id_val);
    pthread_mutex_lock(&e->mutex);
    queue_action(e, &act);
    pthread_mutex_unlock(&e->mutex);
    return JS_UNDEFINED;
}

/* ---- mote.getInterfaces() chain ----
 * COOJA: mote.getInterfaces().getMoteID().setMoteID(id)
 * COOJA: mote.getInterfaces().getPosition().setCoordinates(x,y,z)
 * We create proxy objects that queue actions. */

static JSValue js_mote_set_id(JSContext *ctx, JSValueConst this_val,
                              int argc, JSValueConst *argv) {
    /* Store the new ID on the mote object's parent chain */
    int32_t new_id;
    if (JS_ToInt32(ctx, &new_id, argv[0]) < 0)
        return JS_EXCEPTION;
    /* Update the mote proxy's id property */
    JSValue mote = JS_GetPropertyStr(ctx, this_val, "_mote");
    if (!JS_IsUndefined(mote))
        JS_SetPropertyStr(ctx, mote, "id", JS_NewInt32(ctx, new_id));
    JS_FreeValue(ctx, mote);
    return JS_UNDEFINED;
}

static JSValue js_mote_set_coordinates(JSContext *ctx, JSValueConst this_val,
                                       int argc, JSValueConst *argv) {
    js_test_engine_t *e = get_engine(ctx);
    JSValue mote = JS_GetPropertyStr(ctx, this_val, "_mote");
    JSValue id_val = JS_GetPropertyStr(ctx, mote, "id");
    int32_t node_id = 0;
    JS_ToInt32(ctx, &node_id, id_val);
    JS_FreeValue(ctx, id_val);
    JS_FreeValue(ctx, mote);

    double x = 0, y = 0;
    JS_ToFloat64(ctx, &x, argv[0]);
    if (argc > 1) JS_ToFloat64(ctx, &y, argv[1]);

    sim_test_action_t act = {0};
    act.type = TEST_ACTION_MOVE;
    act.node = node_id;
    act.x = x;
    act.y = y;
    act.at_ms = e->time_us / 1000;
    pthread_mutex_lock(&e->mutex);
    queue_action(e, &act);
    pthread_mutex_unlock(&e->mutex);
    return JS_UNDEFINED;
}

static JSValue js_mote_get_mote_id_iface(JSContext *ctx, JSValueConst this_val,
                                          int argc, JSValueConst *argv) {
    /* Return object with setMoteID() */
    JSValue iface = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, iface, "setMoteID",
        JS_NewCFunction(ctx, js_mote_set_id, "setMoteID", 1));
    /* Pass mote reference through */
    JSValue mote = JS_GetPropertyStr(ctx, this_val, "_mote");
    JS_SetPropertyStr(ctx, iface, "_mote", mote);
    return iface;
}

static JSValue js_mote_get_position_iface(JSContext *ctx, JSValueConst this_val,
                                           int argc, JSValueConst *argv) {
    JSValue iface = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, iface, "setCoordinates",
        JS_NewCFunction(ctx, js_mote_set_coordinates, "setCoordinates", 3));
    JSValue mote = JS_GetPropertyStr(ctx, this_val, "_mote");
    JS_SetPropertyStr(ctx, iface, "_mote", mote);
    return iface;
}

static JSValue js_mote_get_interfaces(JSContext *ctx, JSValueConst this_val,
                                      int argc, JSValueConst *argv) {
    JSValue ifaces = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, ifaces, "getMoteID",
        JS_NewCFunction(ctx, js_mote_get_mote_id_iface, "getMoteID", 0));
    JS_SetPropertyStr(ctx, ifaces, "getPosition",
        JS_NewCFunction(ctx, js_mote_get_position_iface, "getPosition", 0));
    /* Store reference to mote for ID/position changes */
    JS_SetPropertyStr(ctx, ifaces, "_mote", JS_DupValue(ctx, this_val));
    return ifaces;
}

/* ---- sim.getMoteTypes() ---- */

static JSValue js_generate_mote(JSContext *ctx, JSValueConst this_val,
                                int argc, JSValueConst *argv) {
    /* Returns a new mote proxy object (ID will be set later via setMoteID) */
    JSValue mote = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, mote, "id", JS_NewInt32(ctx, 0));
    JS_SetPropertyStr(ctx, mote, "getID",
        JS_NewCFunction(ctx, js_mote_get_id, "getID", 0));
    JS_SetPropertyStr(ctx, mote, "getInterfaces",
        JS_NewCFunction(ctx, js_mote_get_interfaces, "getInterfaces", 0));
    return mote;
}

static JSValue js_sim_get_mote_types(JSContext *ctx, JSValueConst this_val,
                                     int argc, JSValueConst *argv) {
    /* Return array of mote type objects, each with generateMote() */
    JSValue arr = JS_NewArray(ctx);
    /* All types are equivalent for our purposes */
    for (int i = 0; i < 3; i++) {
        JSValue mt = JS_NewObject(ctx);
        JS_SetPropertyStr(ctx, mt, "generateMote",
            JS_NewCFunction(ctx, js_generate_mote, "generateMote", 1));
        JS_SetPropertyUint32(ctx, arr, (uint32_t)i, mt);
    }
    return arr;
}

/* ---- String polyfills for Java-style methods ---- */

static const char *js_string_polyfills =
    "if(!String.prototype.contains)"
    "  String.prototype.contains=function(s){return this.indexOf(s)!==-1};\n"
    "if(!String.prototype.equals)"
    "  String.prototype.equals=function(s){return this.valueOf()===s};\n";

/* ---- Setup global objects ---- */

static void setup_globals(JSContext *ctx, js_test_engine_t *e) {
    JSValue global = JS_GetGlobalObject(ctx);

    /* Control flow */
    JS_SetPropertyStr(ctx, global, "TIMEOUT",
        JS_NewCFunction(ctx, js_timeout, "TIMEOUT", 2));
    JS_SetPropertyStr(ctx, global, "YIELD",
        JS_NewCFunction(ctx, js_yield, "YIELD", 0));
    JS_SetPropertyStr(ctx, global, "WAIT_UNTIL",
        JS_NewCFunction(ctx, js_wait_until, "WAIT_UNTIL", 1));
    JS_SetPropertyStr(ctx, global, "GENERATE_MSG",
        JS_NewCFunction(ctx, js_generate_msg, "GENERATE_MSG", 2));
    JS_SetPropertyStr(ctx, global, "write",
        JS_NewCFunction(ctx, js_write, "write", 2));

    /* log object */
    JSValue log_obj = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, log_obj, "testOK",
        JS_NewCFunction(ctx, js_log_test_ok, "testOK", 0));
    JS_SetPropertyStr(ctx, log_obj, "testFailed",
        JS_NewCFunction(ctx, js_log_test_failed, "testFailed", 0));
    JS_SetPropertyStr(ctx, log_obj, "log",
        JS_NewCFunction(ctx, js_log_log, "log", 1));
    JS_SetPropertyStr(ctx, global, "log", log_obj);

    /* sim object */
    JSValue sim_obj = JS_NewObject(ctx);
    JS_SetPropertyStr(ctx, sim_obj, "getMotes",
        JS_NewCFunction(ctx, js_sim_get_motes, "getMotes", 0));
    JS_SetPropertyStr(ctx, sim_obj, "getMoteWithID",
        JS_NewCFunction(ctx, js_sim_get_mote_with_id, "getMoteWithID", 1));
    JS_SetPropertyStr(ctx, sim_obj, "getSimulationTimeMillis",
        JS_NewCFunction(ctx, js_sim_get_time_ms, "getSimulationTimeMillis", 0));
    JS_SetPropertyStr(ctx, sim_obj, "getSimulationTime",
        JS_NewCFunction(ctx, js_sim_get_time_us, "getSimulationTime", 0));
    JS_SetPropertyStr(ctx, sim_obj, "removeMote",
        JS_NewCFunction(ctx, js_sim_remove_mote, "removeMote", 1));
    JS_SetPropertyStr(ctx, sim_obj, "addMote",
        JS_NewCFunction(ctx, js_sim_add_mote, "addMote", 1));
    JS_SetPropertyStr(ctx, sim_obj, "getMoteTypes",
        JS_NewCFunction(ctx, js_sim_get_mote_types, "getMoteTypes", 0));
    JS_SetPropertyStr(ctx, global, "sim", sim_obj);

    /* Initial globals: msg, id, time, mote */
    JS_SetPropertyStr(ctx, global, "msg", JS_NewString(ctx, ""));
    JS_SetPropertyStr(ctx, global, "id", JS_NewInt32(ctx, 0));
    JS_SetPropertyStr(ctx, global, "time", JS_NewInt64(ctx, 0));
    JS_SetPropertyStr(ctx, global, "mote", JS_NewObject(ctx));

    JS_FreeValue(ctx, global);

    /* Install String polyfills */
    JS_Eval(ctx, js_string_polyfills, strlen(js_string_polyfills),
            "<polyfills>", JS_EVAL_TYPE_GLOBAL);
}

/* ---- Update globals before each YIELD resume ---- */

static void update_globals(JSContext *ctx, js_test_engine_t *e) {
    JSValue global = JS_GetGlobalObject(ctx);
    JS_SetPropertyStr(ctx, global, "msg", JS_NewString(ctx, e->msg));
    JS_SetPropertyStr(ctx, global, "id", JS_NewInt32(ctx, e->id));
    JS_SetPropertyStr(ctx, global, "time", JS_NewInt64(ctx, e->time_us));
    JS_SetPropertyStr(ctx, global, "mote", make_mote_obj(ctx, e->id));
    JS_FreeValue(ctx, global);
}

/* ---- JS thread entry point ---- */

static void *js_thread_func(void *arg) {
    js_test_engine_t *e = (js_test_engine_t *)arg;
    JSContext *ctx = (JSContext *)e->ctx;

    /* Pick up script from static storage */
    char *script = s_pending_script;
    s_pending_script = NULL;

    /* Execute the script — calls to YIELD() will block in yield_wait() */
    JSValue result = JS_Eval(ctx, script, strlen(script),
                            "<test_script>", JS_EVAL_TYPE_GLOBAL);
    free(script);

    /* Script finished */
    pthread_mutex_lock(&e->mutex);
    if (e->finished == 0) {
        if (JS_IsException(result)) {
            JSValue exc = JS_GetException(ctx);
            const char *str = JS_ToCString(ctx, exc);
            /* testOK/testFailed throw "testOK"/"testFailed" —
             * if finished is already set, this is expected */
            if (e->finished == 0 && str) {
                if (strcmp(str, "testOK") != 0 &&
                    strcmp(str, "testFailed") != 0) {
                    e->finished = -1;
                    snprintf(e->fail_reason, sizeof(e->fail_reason),
                             "JS exception: %s", str);
                }
            }
            if (str) JS_FreeCString(ctx, str);
            JS_FreeValue(ctx, exc);
        } else {
            /* Script ended without testOK/testFailed — treated as success
             * if it ran to completion (e.g., all steps done). */
        }
    }
    e->js_waiting = true;
    pthread_cond_signal(&e->line_done);
    pthread_mutex_unlock(&e->mutex);

    JS_FreeValue(ctx, result);
    return NULL;
}

/* ---- Public API ---- */

int js_test_init(js_test_engine_t *e, const char *script,
                 const int *node_ids, int node_count) {
    memset(e, 0, sizeof(*e));

    e->node_count = node_count > 128 ? 128 : node_count;
    for (int i = 0; i < e->node_count; i++)
        e->node_ids[i] = node_ids[i];

    pthread_mutex_init(&e->mutex, NULL);
    pthread_cond_init(&e->line_ready, NULL);
    pthread_cond_init(&e->line_done, NULL);

    JSRuntime *rt = JS_NewRuntime();
    if (!rt) return -1;
    JS_SetMemoryLimit(rt, 32 * 1024 * 1024);

    JSContext *ctx = JS_NewContext(rt);
    if (!ctx) { JS_FreeRuntime(rt); return -1; }
    JS_SetContextOpaque(ctx, e);

    e->rt = rt;
    e->ctx = ctx;

    setup_globals(ctx, e);

    /* Preprocess and store script for thread pickup */
    s_pending_script = preprocess_script(script);
    if (!s_pending_script) {
        JS_FreeContext(ctx);
        JS_FreeRuntime(rt);
        return -1;
    }

    if (pthread_create(&e->thread, NULL, js_thread_func, e) != 0) {
        free(s_pending_script);
        s_pending_script = NULL;
        JS_FreeContext(ctx);
        JS_FreeRuntime(rt);
        return -1;
    }

    /* Wait for JS thread to reach first YIELD (or finish) */
    pthread_mutex_lock(&e->mutex);
    while (!e->js_waiting && e->finished == 0)
        pthread_cond_wait(&e->line_done, &e->mutex);
    pthread_mutex_unlock(&e->mutex);

    printf("  JS test engine initialized (%d nodes, timeout=%lld ms)\n",
           e->node_count, (long long)(e->timeout_us / 1000));
    return 0;
}

int js_test_feed_line(js_test_engine_t *e, const char *line,
                      int node_id, int64_t sim_time_us) {
    if (e->finished != 0)
        return e->finished;

    pthread_mutex_lock(&e->mutex);

    /* Set shared state */
    snprintf(e->msg, sizeof(e->msg), "%s", line);
    e->id = node_id;
    e->time_us = sim_time_us;

    /* Update JS globals (must be done from... wait, globals must be updated
     * from the JS thread since QuickJS isn't thread-safe for JS operations.
     * We need to update globals in the YIELD/WAIT_UNTIL functions instead. */

    /* Reset js_waiting before signaling — JS thread will set it back to
     * true when it reaches the next YIELD (or finishes). */
    e->js_waiting = false;

    /* Signal JS thread */
    e->has_line = true;
    pthread_cond_signal(&e->line_ready);

    /* Wait for JS to process and reach next YIELD (or finish) */
    while (!e->js_waiting && e->finished == 0)
        pthread_cond_wait(&e->line_done, &e->mutex);

    int result = e->finished;
    pthread_mutex_unlock(&e->mutex);
    return result;
}

int js_test_check_gen_msgs(js_test_engine_t *e, int64_t sim_time_us) {
    if (e->finished != 0)
        return e->finished;

    while (e->gen_msg_next < e->gen_msg_count) {
        js_gen_msg_t *gm = &e->gen_msgs[e->gen_msg_next];
        if (gm->at_us > sim_time_us)
            break;
        e->gen_msg_next++;
        /* Deliver as a synthetic console line */
        int result = js_test_feed_line(e, gm->label, -1, sim_time_us);
        if (result != 0)
            return result;
    }
    return 0;
}

int js_test_check_timeout(js_test_engine_t *e, int64_t sim_time_us) {
    if (e->finished != 0)
        return e->finished;
    if (e->timeout_us > 0 && sim_time_us >= e->timeout_us) {
        /* Set flag so JS thread will call __timeout_cb() on next wake */
        e->timeout_fired = true;

        /* Feed a dummy line to wake the JS thread — __timeout_cb will be
         * eval'd inside yield_wait before returning to the script.
         * If __timeout_cb calls testOK/testFailed, finished will be set. */
        js_test_feed_line(e, "__timeout__", -1, sim_time_us);

        /* If the callback didn't set a result, default to fail */
        if (e->finished == 0) {
            e->finished = -1;
            snprintf(e->fail_reason, sizeof(e->fail_reason),
                     "TIMEOUT reached (%lld ms)", (long long)(e->timeout_us / 1000));
        }
        return e->finished;
    }
    return 0;
}

int js_test_drain_actions(js_test_engine_t *e,
                          sim_test_action_t *dst, int max_actions) {
    pthread_mutex_lock(&e->mutex);
    int count = e->pending_action_count;
    if (count > max_actions) count = max_actions;
    if (count > 0) {
        memcpy(dst, e->pending_actions, count * sizeof(sim_test_action_t));
        /* Shift remaining */
        int remaining = e->pending_action_count - count;
        if (remaining > 0)
            memmove(e->pending_actions, e->pending_actions + count,
                    remaining * sizeof(sim_test_action_t));
        e->pending_action_count = remaining;
    }
    pthread_mutex_unlock(&e->mutex);
    return count;
}

const char *js_test_fail_reason(const js_test_engine_t *e) {
    return e->fail_reason;
}

void js_test_destroy(js_test_engine_t *e) {
    /* Signal shutdown */
    pthread_mutex_lock(&e->mutex);
    e->shutdown = true;
    e->has_line = true; /* wake up if waiting */
    pthread_cond_signal(&e->line_ready);
    pthread_mutex_unlock(&e->mutex);

    /* Wait for thread to finish */
    pthread_join(e->thread, NULL);

    /* Free QuickJS — run GC first to clean up cyclic refs */
    if (e->ctx) {
        JSRuntime *rt = (JSRuntime *)e->rt;
        JS_RunGC(rt);
        JS_FreeContext((JSContext *)e->ctx);
    }
    if (e->rt) JS_FreeRuntime((JSRuntime *)e->rt);

    pthread_mutex_destroy(&e->mutex);
    pthread_cond_destroy(&e->line_ready);
    pthread_cond_destroy(&e->line_done);
}
