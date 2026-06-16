/*
 * JSON simulation configuration loader.
 *
 * sim_config_load reads the file, parses JSON, and dispatches on the
 * top-level "version" key to a per-version parser (parse_v1 / parse_v2 —
 * §8.3).  Each parser populates the same normalized struct
 * (sim_normalized_config_t); the runtime consumes only that struct.
 */
#include "sim_config.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int parse_v1(cJSON *root, sim_normalized_config_t *cfg);
static int parse_v2(cJSON *root, sim_normalized_config_t *cfg);
static void parse_medium_object(cJSON *medium, sim_normalized_config_t *cfg);
static int  parse_test(cJSON *test, sim_normalized_config_t *cfg);

int sim_config_load(sim_normalized_config_t *cfg, const char *json_path) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = 1;         /* default; overridden by the "version" key */
    cfg->timeout_ms = 20000;  /* default */
    cfg->tx_range = 50.0;
    cfg->interference_range = 100.0;
    cfg->success_ratio_tx = 1.0;
    cfg->success_ratio_rx = 1.0;

    /* Read file */
    FILE *f = fopen(json_path, "rb");
    if (!f) {
        fprintf(stderr, "sim_config: cannot open '%s'\n", json_path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buf = malloc((size_t)len + 1);
    if (!buf) {
        fclose(f);
        fprintf(stderr, "sim_config: out of memory\n");
        return -1;
    }
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fclose(f);
        free(buf);
        fprintf(stderr, "sim_config: read error on '%s'\n", json_path);
        return -1;
    }
    buf[len] = '\0';
    fclose(f);

    /* Parse JSON */
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        fprintf(stderr, "sim_config: JSON parse error near: %.40s\n",
                err ? err : "(unknown)");
        return -1;
    }

    /* Dispatch on the schema version (absent/1 = legacy v1). */
    cJSON *ver = cJSON_GetObjectItem(root, "version");
    if (cJSON_IsNumber(ver))
        cfg->version = ver->valueint;

    int rc = (cfg->version >= 2) ? parse_v2(root, cfg) : parse_v1(root, cfg);
    cJSON_Delete(root);
    if (rc != 0)
        return rc;

    /* Auto-assign IDs for nodes without explicit id */
    for (int i = 0; i < cfg->node_count; i++) {
        if (cfg->nodes[i].id == 0)
            cfg->nodes[i].id = i + 1;
    }
    return 0;
}

/* v1 (legacy) JSON → normalized config.  Verbatim move of the original
 * parse body; populates cfg from `root` and leaves cJSON_Delete + node-ID
 * auto-assignment to the caller. */
static int parse_v1(cJSON *root, sim_normalized_config_t *cfg) {
    /* Extract top-level fields */
    cJSON *title = cJSON_GetObjectItemCaseSensitive(root, "title");
    if (cJSON_IsString(title) && title->valuestring) {
        snprintf(cfg->title, sizeof(cfg->title), "%s", title->valuestring);
    }

    cJSON *timeout = cJSON_GetObjectItemCaseSensitive(root, "timeout_ms");
    if (cJSON_IsNumber(timeout)) {
        cfg->timeout_ms = timeout->valueint;
    }

    cJSON *seed = cJSON_GetObjectItemCaseSensitive(root, "seed");
    if (cJSON_IsNumber(seed)) {
        cfg->seed = seed->valueint;
    }

    cJSON *startup_delay = cJSON_GetObjectItemCaseSensitive(root, "startup_delay_ms");
    if (cJSON_IsNumber(startup_delay)) {
        cfg->startup_delay_ms = startup_delay->valueint;
    }

    cJSON *speed = cJSON_GetObjectItemCaseSensitive(root, "speed");
    if (cJSON_IsNumber(speed)) {
        cfg->speed = speed->valuedouble;
    }

    /* Parse optional radiomedium object */
    parse_medium_object(cJSON_GetObjectItemCaseSensitive(root, "radiomedium"), cfg);

    /* Parse nodes array */
    cJSON *nodes = cJSON_GetObjectItemCaseSensitive(root, "nodes");
    if (!cJSON_IsArray(nodes)) {
        fprintf(stderr, "sim_config: 'nodes' array not found\n");
        return -1;  /* caller (sim_config_load) owns cJSON_Delete(root) */
    }

    int count = 0;
    cJSON *node_item;
    cJSON_ArrayForEach(node_item, nodes) {
        if (count >= MAX_SIM_NODES) {
            fprintf(stderr, "sim_config: too many nodes (max %d)\n", MAX_SIM_NODES);
            break;
        }

        cJSON *fw = cJSON_GetObjectItemCaseSensitive(node_item, "firmware");
        if (!cJSON_IsString(fw) || !fw->valuestring) {
            fprintf(stderr, "sim_config: node %d missing 'firmware' field\n", count);
            return -1;  /* caller owns cJSON_Delete(root) */
        }

        snprintf(cfg->nodes[count].firmware,
                 sizeof(cfg->nodes[count].firmware),
                 "%s", fw->valuestring);

        cJSON *id = cJSON_GetObjectItemCaseSensitive(node_item, "id");
        if (cJSON_IsNumber(id)) {
            cfg->nodes[count].id = id->valueint;
        } else {
            cfg->nodes[count].id = 0;  /* auto-assign */
        }

        /* Optional position */
        cJSON *xval = cJSON_GetObjectItemCaseSensitive(node_item, "x");
        cJSON *yval = cJSON_GetObjectItemCaseSensitive(node_item, "y");
        if (cJSON_IsNumber(xval) && cJSON_IsNumber(yval)) {
            cfg->nodes[count].x = xval->valuedouble;
            cfg->nodes[count].y = yval->valuedouble;
            cfg->nodes[count].has_position = 1;
        }

        /* Optional clock deviation (Cooja MspClock deviation) */
        cJSON *dev = cJSON_GetObjectItemCaseSensitive(node_item, "clock_deviation");
        cfg->nodes[count].clock_deviation = cJSON_IsNumber(dev) ? dev->valuedouble : 1.0;

        count++;
    }
    cfg->node_count = count;

    /* Parse optional test section */
    cJSON *test = cJSON_GetObjectItemCaseSensitive(root, "test");
    if (cJSON_IsObject(test) && parse_test(test, cfg) != 0)
        return -1;

    /* Parse mote_types array (for dynamic node creation) */
    cJSON *mt_arr = cJSON_GetObjectItem(root, "mote_types");
    if (cJSON_IsArray(mt_arr)) {
        int ti = 0;
        cJSON *mt_item;
        cJSON_ArrayForEach(mt_item, mt_arr) {
            if (ti >= 8) break;
            cJSON *fw = cJSON_GetObjectItem(mt_item, "firmware");
            if (cJSON_IsString(fw) && fw->valuestring)
                strncpy(cfg->mote_type_firmware[ti], fw->valuestring, 255);
            ti++;
        }
        cfg->mote_type_count = ti;
    }

    /* Parse serial_socket config */
    cJSON *ss = cJSON_GetObjectItem(root, "serial_socket");
    if (cJSON_IsObject(ss)) {
        cfg->has_serial_socket = 1;
        cfg->serial_socket_port = 60001;  /* default */

        cJSON *ss_port = cJSON_GetObjectItem(ss, "port");
        if (cJSON_IsNumber(ss_port))
            cfg->serial_socket_port = ss_port->valueint;

        cJSON *ss_node = cJSON_GetObjectItem(ss, "node");
        if (cJSON_IsNumber(ss_node))
            cfg->serial_socket_node = ss_node->valueint;

        cJSON *ss_cmd = cJSON_GetObjectItem(ss, "command");
        if (cJSON_IsString(ss_cmd) && ss_cmd->valuestring)
            snprintf(cfg->serial_socket_command, sizeof(cfg->serial_socket_command),
                     "%s", ss_cmd->valuestring);
    }

    return 0;
}

/* Shared medium-object parser (v1 "radiomedium" / v2 "medium").  No-op when
 * `medium` is not an object. */
static void parse_medium_object(cJSON *medium, sim_normalized_config_t *cfg) {
    if (!cJSON_IsObject(medium)) return;
    cJSON *mtype = cJSON_GetObjectItemCaseSensitive(medium, "type");
    if (cJSON_IsString(mtype) && mtype->valuestring) {
        snprintf(cfg->medium_name, sizeof(cfg->medium_name), "%s",
                 mtype->valuestring);
        if (strcmp(mtype->valuestring, "udgm") == 0)
            cfg->medium_type = 1;  /* RADIO_MEDIUM_UDGM */
    }

    cJSON *val;
    val = cJSON_GetObjectItemCaseSensitive(medium, "tx_range");
    if (cJSON_IsNumber(val)) cfg->tx_range = val->valuedouble;

    val = cJSON_GetObjectItemCaseSensitive(medium, "interference_range");
    if (cJSON_IsNumber(val)) cfg->interference_range = val->valuedouble;

    val = cJSON_GetObjectItemCaseSensitive(medium, "success_ratio_tx");
    if (cJSON_IsNumber(val)) cfg->success_ratio_tx = val->valuedouble;

    val = cJSON_GetObjectItemCaseSensitive(medium, "success_ratio_rx");
    if (cJSON_IsNumber(val)) cfg->success_ratio_rx = val->valuedouble;
}

/* Shared test-section parser (v1 + v2 use the same "test" schema).  Sets
 * cfg->has_test and fills cfg->test/js_script.  Returns -1 on a malformed
 * step (missing 'wait'); the caller owns cJSON_Delete(root). */
static int parse_test(cJSON *test, sim_normalized_config_t *cfg) {
    cfg->has_test = 1;

    /* Check for JavaScript test script */
    cJSON *js_path = cJSON_GetObjectItemCaseSensitive(test, "js_script");
    if (cJSON_IsString(js_path) && js_path->valuestring) {
        snprintf(cfg->js_script_path, sizeof(cfg->js_script_path),
                 "%s", js_path->valuestring);
        cfg->has_js_script = 1;
    }
    cJSON *js_inline = cJSON_GetObjectItemCaseSensitive(test, "js_script_inline");
    if (cJSON_IsString(js_inline) && js_inline->valuestring) {
        cfg->js_script_inline = strdup(js_inline->valuestring);
        cfg->has_js_script = 1;
    }

    /* Parse test steps */
    cJSON *steps = cJSON_GetObjectItemCaseSensitive(test, "steps");
    if (cJSON_IsArray(steps)) {
        int si = 0;
        cJSON *step_item;
        cJSON_ArrayForEach(step_item, steps) {
            if (si >= MAX_TEST_STEPS) {
                fprintf(stderr, "sim_config: too many test steps (max %d)\n",
                        MAX_TEST_STEPS);
                break;
            }
            sim_test_step_t *s = &cfg->test.steps[si];
            s->node = -1;
            s->count = 1;
            s->timeout_ms = 0;

            cJSON *wait = cJSON_GetObjectItemCaseSensitive(step_item, "wait");
            if (cJSON_IsString(wait) && wait->valuestring) {
                snprintf(s->pattern, sizeof(s->pattern), "%s",
                         wait->valuestring);
            } else {
                fprintf(stderr, "sim_config: test step %d missing 'wait'\n", si);
                return -1;  /* caller owns cJSON_Delete(root) */
            }

            cJSON *snode = cJSON_GetObjectItemCaseSensitive(step_item, "node");
            if (cJSON_IsNumber(snode))
                s->node = snode->valueint;

            cJSON *scount = cJSON_GetObjectItemCaseSensitive(step_item, "count");
            if (cJSON_IsNumber(scount))
                s->count = scount->valueint;

            cJSON *stimeout = cJSON_GetObjectItemCaseSensitive(step_item, "timeout_ms");
            if (cJSON_IsNumber(stimeout))
                s->timeout_ms = stimeout->valueint;

            si++;
        }
        cfg->test.step_count = si;
    }

    /* Parse fail_on patterns */
    cJSON *fail_on = cJSON_GetObjectItemCaseSensitive(test, "fail_on");
    if (cJSON_IsArray(fail_on)) {
        int fi = 0;
        cJSON *fp;
        cJSON_ArrayForEach(fp, fail_on) {
            if (fi >= MAX_FAIL_PATTERNS) break;
            if (cJSON_IsString(fp) && fp->valuestring) {
                snprintf(cfg->test.fail_on[fi], sizeof(cfg->test.fail_on[fi]),
                         "%s", fp->valuestring);
                fi++;
            }
        }
        cfg->test.fail_on_count = fi;
    }

    /* Parse timeout_is_success */
    cJSON *tis = cJSON_GetObjectItemCaseSensitive(test, "timeout_is_success");
    if (cJSON_IsTrue(tis))
        cfg->test.timeout_is_success = 1;

    /* Parse validators array */
    cJSON *validators = cJSON_GetObjectItemCaseSensitive(test, "validators");
    if (cJSON_IsArray(validators)) {
        int vi = 0;
        cJSON *val_item;
        cJSON_ArrayForEach(val_item, validators) {
            if (vi >= MAX_TEST_VALIDATORS) {
                fprintf(stderr, "sim_config: too many validators (max %d)\n",
                        MAX_TEST_VALIDATORS);
                break;
            }
            sim_test_validator_t *v = &cfg->test.validators[vi];
            v->node = -1;
            v->min_count = 1;

            cJSON *vpat = cJSON_GetObjectItemCaseSensitive(val_item, "pattern");
            if (cJSON_IsString(vpat) && vpat->valuestring) {
                snprintf(v->pattern, sizeof(v->pattern), "%s",
                         vpat->valuestring);
            } else {
                fprintf(stderr, "sim_config: validator %d missing 'pattern'\n", vi);
                continue;
            }

            cJSON *vmin = cJSON_GetObjectItemCaseSensitive(val_item, "min_count");
            if (cJSON_IsNumber(vmin))
                v->min_count = vmin->valueint;

            cJSON *vnode = cJSON_GetObjectItemCaseSensitive(val_item, "node");
            if (cJSON_IsNumber(vnode))
                v->node = vnode->valueint;

            vi++;
        }
        cfg->test.validator_count = vi;
    }

    /* Parse actions array */
    cJSON *actions = cJSON_GetObjectItemCaseSensitive(test, "actions");
    if (cJSON_IsArray(actions)) {
        int ai = 0;
        cJSON *act_item;
        cJSON_ArrayForEach(act_item, actions) {
            if (ai >= MAX_TEST_ACTIONS) {
                fprintf(stderr, "sim_config: too many test actions (max %d)\n",
                        MAX_TEST_ACTIONS);
                break;
            }
            sim_test_action_t *a = &cfg->test.actions[ai];
            memset(a, 0, sizeof(*a));

            cJSON *at_ms = cJSON_GetObjectItemCaseSensitive(act_item, "at_ms");
            if (cJSON_IsNumber(at_ms))
                a->at_ms = (int64_t)at_ms->valuedouble;

            cJSON *atype = cJSON_GetObjectItemCaseSensitive(act_item, "type");
            if (cJSON_IsString(atype) && atype->valuestring) {
                if (strcmp(atype->valuestring, "move") == 0)
                    a->type = TEST_ACTION_MOVE;
                else if (strcmp(atype->valuestring, "send") == 0)
                    a->type = TEST_ACTION_SEND;
                else if (strcmp(atype->valuestring, "remove") == 0)
                    a->type = TEST_ACTION_REMOVE;
                else if (strcmp(atype->valuestring, "add") == 0)
                    a->type = TEST_ACTION_ADD;
                else if (strcmp(atype->valuestring, "send_all") == 0)
                    a->type = TEST_ACTION_SEND_ALL;
            }

            cJSON *anode = cJSON_GetObjectItemCaseSensitive(act_item, "node");
            if (cJSON_IsNumber(anode))
                a->node = anode->valueint;

            cJSON *ax = cJSON_GetObjectItemCaseSensitive(act_item, "x");
            if (cJSON_IsNumber(ax)) a->x = ax->valuedouble;

            cJSON *ay = cJSON_GetObjectItemCaseSensitive(act_item, "y");
            if (cJSON_IsNumber(ay)) a->y = ay->valuedouble;

            cJSON *adata = cJSON_GetObjectItemCaseSensitive(act_item, "data");
            if (cJSON_IsString(adata) && adata->valuestring)
                snprintf(a->data, sizeof(a->data), "%s", adata->valuestring);

            ai++;
        }
        cfg->test.action_count = ai;
    }
    return 0;
}

/* Resolve a v2 node's named mote-type to its firmware path; returns NULL if
 * the name is not in cfg->mote_types[]. */
static const char *resolve_mote_type_firmware(const sim_normalized_config_t *cfg,
                                              const char *name) {
    for (int i = 0; i < cfg->mote_type_count; i++)
        if (strcmp(cfg->mote_types[i].name, name) == 0)
            return cfg->mote_types[i].firmware;
    return NULL;
}

/* Config v2 (§8.2) JSON → normalized config.  Reuses the shared top-level
 * scalars, medium, test, and serial-socket parsing; the differences are the
 * `medium` key (vs v1 `radiomedium`), named `mote_types`, a `plugins` list,
 * and nodes that reference a mote-type by `type` name instead of inlining
 * `firmware`.  Resolution still goes through the firmware extension, so the
 * runtime is unchanged (the registry-by-name lookup is Phase 8). */
static int parse_v2(cJSON *root, sim_normalized_config_t *cfg) {
    /* Top-level scalars (same keys as v1) */
    cJSON *title = cJSON_GetObjectItemCaseSensitive(root, "title");
    if (cJSON_IsString(title) && title->valuestring)
        snprintf(cfg->title, sizeof(cfg->title), "%s", title->valuestring);
    cJSON *timeout = cJSON_GetObjectItemCaseSensitive(root, "timeout_ms");
    if (cJSON_IsNumber(timeout)) cfg->timeout_ms = timeout->valueint;
    cJSON *seed = cJSON_GetObjectItemCaseSensitive(root, "seed");
    if (cJSON_IsNumber(seed)) cfg->seed = seed->valueint;
    cJSON *startup_delay = cJSON_GetObjectItemCaseSensitive(root, "startup_delay_ms");
    if (cJSON_IsNumber(startup_delay)) cfg->startup_delay_ms = startup_delay->valueint;
    cJSON *speed = cJSON_GetObjectItemCaseSensitive(root, "speed");
    if (cJSON_IsNumber(speed)) cfg->speed = speed->valuedouble;

    /* v2 uses the "medium" key (v1 used "radiomedium") */
    parse_medium_object(cJSON_GetObjectItemCaseSensitive(root, "medium"), cfg);

    /* Named mote types: {name, kind, cpu, soc, board, firmware}.  Captured
     * for Phase 8; Phase 7 reads only firmware (+ writes the legacy
     * index-keyed table so TEST_ACTION_ADD stays index-compatible). */
    cJSON *mt_arr = cJSON_GetObjectItemCaseSensitive(root, "mote_types");
    if (cJSON_IsArray(mt_arr)) {
        int ti = 0;
        cJSON *mt_item;
        cJSON_ArrayForEach(mt_item, mt_arr) {
            if (ti >= 8) break;
            sim_mote_type_t *mt = &cfg->mote_types[ti];
            struct { const char *key; char *dst; size_t sz; } fields[] = {
                {"name", mt->name, sizeof(mt->name)},
                {"kind", mt->kind, sizeof(mt->kind)},
                {"cpu",  mt->cpu,  sizeof(mt->cpu)},
                {"soc",  mt->soc,  sizeof(mt->soc)},
                {"board", mt->board, sizeof(mt->board)},
                {"firmware", mt->firmware, sizeof(mt->firmware)},
            };
            for (size_t fi = 0; fi < sizeof(fields)/sizeof(fields[0]); fi++) {
                cJSON *v = cJSON_GetObjectItemCaseSensitive(mt_item, fields[fi].key);
                if (cJSON_IsString(v) && v->valuestring)
                    snprintf(fields[fi].dst, fields[fi].sz, "%s", v->valuestring);
            }
            /* mirror into the legacy index-keyed table (TEST_ACTION_ADD) */
            snprintf(cfg->mote_type_firmware[ti], sizeof(cfg->mote_type_firmware[ti]),
                     "%s", mt->firmware);
            ti++;
        }
        cfg->mote_type_count = ti;
    }

    /* Plugins list (captured in Phase 7; consumed by Phase 9 — .so paths are
     * dlopen'd, "builtin:NAME" entries name a built-in service). */
    cJSON *plugins = cJSON_GetObjectItemCaseSensitive(root, "plugins");
    if (cJSON_IsArray(plugins)) {
        int pi = 0;
        cJSON *p;
        cJSON_ArrayForEach(p, plugins) {
            if (pi >= 16) break;
            if (cJSON_IsString(p) && p->valuestring) {
                int n = snprintf(cfg->plugins[pi], sizeof(cfg->plugins[pi]),
                                 "%s", p->valuestring);
                if (n < 0 || n >= (int)sizeof(cfg->plugins[pi])) {
                    /* Refuse a truncated path rather than dlopen the wrong
                     * file — blank the entry so the consumer skips it. */
                    fprintf(stderr, "config: plugin entry too long (>%d), "
                            "ignored: %s\n",
                            (int)sizeof(cfg->plugins[pi]) - 1, p->valuestring);
                    cfg->plugins[pi][0] = '\0';
                }
                pi++;
            }
        }
        cfg->plugin_count = pi;
    }

    /* Nodes: reference a mote-type by `type` name → resolve to firmware */
    cJSON *nodes = cJSON_GetObjectItemCaseSensitive(root, "nodes");
    if (!cJSON_IsArray(nodes)) {
        fprintf(stderr, "sim_config: 'nodes' array not found\n");
        return -1;
    }
    int count = 0;
    cJSON *node_item;
    cJSON_ArrayForEach(node_item, nodes) {
        if (count >= MAX_SIM_NODES) {
            fprintf(stderr, "sim_config: too many nodes (max %d)\n", MAX_SIM_NODES);
            break;
        }
        sim_node_config_t *n = &cfg->nodes[count];

        cJSON *type = cJSON_GetObjectItemCaseSensitive(node_item, "type");
        if (!cJSON_IsString(type) || !type->valuestring) {
            fprintf(stderr, "sim_config: node %d missing 'type'\n", count);
            return -1;
        }
        const char *fw = resolve_mote_type_firmware(cfg, type->valuestring);
        if (!fw) {
            fprintf(stderr, "sim_config: node %d references unknown mote type '%s'\n",
                    count, type->valuestring);
            return -1;
        }
        snprintf(n->type_name, sizeof(n->type_name), "%s", type->valuestring);
        snprintf(n->firmware, sizeof(n->firmware), "%s", fw);

        cJSON *id = cJSON_GetObjectItemCaseSensitive(node_item, "id");
        n->id = cJSON_IsNumber(id) ? id->valueint : 0;  /* auto-assign */

        cJSON *xval = cJSON_GetObjectItemCaseSensitive(node_item, "x");
        cJSON *yval = cJSON_GetObjectItemCaseSensitive(node_item, "y");
        if (cJSON_IsNumber(xval) && cJSON_IsNumber(yval)) {
            n->x = xval->valuedouble;
            n->y = yval->valuedouble;
            n->has_position = 1;
        }

        cJSON *dev = cJSON_GetObjectItemCaseSensitive(node_item, "clock_deviation");
        n->clock_deviation = cJSON_IsNumber(dev) ? dev->valuedouble : 1.0;

        count++;
    }
    cfg->node_count = count;

    /* Test section (same schema as v1) */
    cJSON *test = cJSON_GetObjectItemCaseSensitive(root, "test");
    if (cJSON_IsObject(test) && parse_test(test, cfg) != 0)
        return -1;

    /* Serial socket (same schema as v1) */
    cJSON *ss = cJSON_GetObjectItemCaseSensitive(root, "serial_socket");
    if (cJSON_IsObject(ss)) {
        cfg->has_serial_socket = 1;
        cfg->serial_socket_port = 60001;
        cJSON *ss_port = cJSON_GetObjectItem(ss, "port");
        if (cJSON_IsNumber(ss_port)) cfg->serial_socket_port = ss_port->valueint;
        cJSON *ss_node = cJSON_GetObjectItem(ss, "node");
        if (cJSON_IsNumber(ss_node)) cfg->serial_socket_node = ss_node->valueint;
        cJSON *ss_cmd = cJSON_GetObjectItem(ss, "command");
        if (cJSON_IsString(ss_cmd) && ss_cmd->valuestring)
            snprintf(cfg->serial_socket_command, sizeof(cfg->serial_socket_command),
                     "%s", ss_cmd->valuestring);
    }

    return 0;
}

void sim_config_print(const sim_normalized_config_t *cfg) {
    if (cfg->title[0])
        printf("Config: %s\n", cfg->title);
    printf("  timeout_ms: %d\n", cfg->timeout_ms);
    if (cfg->seed)
        printf("  seed: %d\n", cfg->seed);
    if (cfg->startup_delay_ms > 0)
        printf("  startup_delay_ms: %d\n", cfg->startup_delay_ms);
    if (cfg->speed > 0)
        printf("  speed: %.1fx\n", cfg->speed);
    if (cfg->medium_type == 1) {
        printf("  radiomedium: UDGM\n");
        printf("    tx_range: %.1f m\n", cfg->tx_range);
        printf("    interference_range: %.1f m\n", cfg->interference_range);
        printf("    success_ratio_tx: %.2f\n", cfg->success_ratio_tx);
        printf("    success_ratio_rx: %.2f\n", cfg->success_ratio_rx);
    }
    printf("  nodes: %d\n", cfg->node_count);
    for (int i = 0; i < cfg->node_count; i++) {
        if (cfg->nodes[i].has_position) {
            printf("    [%d] id=%d pos=(%.1f, %.1f) firmware=%s\n",
                   i, cfg->nodes[i].id, cfg->nodes[i].x, cfg->nodes[i].y,
                   cfg->nodes[i].firmware);
        } else {
            printf("    [%d] id=%d firmware=%s\n",
                   i, cfg->nodes[i].id, cfg->nodes[i].firmware);
        }
    }
    if (cfg->has_test) {
        printf("  test: %d steps\n", cfg->test.step_count);
        for (int i = 0; i < cfg->test.step_count; i++) {
            const sim_test_step_t *s = &cfg->test.steps[i];
            printf("    [%d] wait \"%s\"", i, s->pattern);
            if (s->node >= 0)
                printf(" node=%d", s->node);
            if (s->count > 1)
                printf(" count=%d", s->count);
            if (s->timeout_ms > 0)
                printf(" timeout=%dms", s->timeout_ms);
            printf("\n");
        }
        if (cfg->test.fail_on_count > 0) {
            printf("  fail_on: ");
            for (int i = 0; i < cfg->test.fail_on_count; i++) {
                printf("\"%s\"%s", cfg->test.fail_on[i],
                       i < cfg->test.fail_on_count - 1 ? ", " : "");
            }
            printf("\n");
        }
        if (cfg->test.timeout_is_success)
            printf("  timeout_is_success: true\n");
        if (cfg->test.validator_count > 0) {
            printf("  validators: %d\n", cfg->test.validator_count);
            for (int i = 0; i < cfg->test.validator_count; i++) {
                const sim_test_validator_t *v = &cfg->test.validators[i];
                printf("    [%d] \"%s\" min_count=%d", i, v->pattern, v->min_count);
                if (v->node >= 0)
                    printf(" node=%d", v->node);
                printf("\n");
            }
        }
        if (cfg->test.action_count > 0) {
            printf("  actions: %d\n", cfg->test.action_count);
            for (int i = 0; i < cfg->test.action_count; i++) {
                const sim_test_action_t *a = &cfg->test.actions[i];
                if (a->type == TEST_ACTION_MOVE)
                    printf("    [%d] at %lld ms: move node %d to (%.1f, %.1f)\n",
                           i, (long long)a->at_ms, a->node, a->x, a->y);
                else if (a->type == TEST_ACTION_SEND)
                    printf("    [%d] at %lld ms: send node %d \"%s\"\n",
                           i, (long long)a->at_ms, a->node, a->data);
                else if (a->type == TEST_ACTION_REMOVE)
                    printf("    [%d] at %lld ms: remove node %d\n",
                           i, (long long)a->at_ms, a->node);
                else if (a->type == TEST_ACTION_ADD)
                    printf("    [%d] at %lld ms: add node %d\n",
                           i, (long long)a->at_ms, a->node);
                else if (a->type == TEST_ACTION_SEND_ALL)
                    printf("    [%d] at %lld ms: send_all \"%s\"\n",
                           i, (long long)a->at_ms, a->data);
            }
        }
    }
    if (cfg->has_serial_socket) {
        printf("  serial_socket: port=%d node=%d\n",
               cfg->serial_socket_port, cfg->serial_socket_node);
        if (cfg->serial_socket_command[0])
            printf("    command: %s\n", cfg->serial_socket_command);
    }
}
