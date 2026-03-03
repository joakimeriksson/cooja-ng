/*
 * JSON simulation configuration loader
 */
#include "sim_config.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int sim_config_load(sim_config_t *cfg, const char *json_path) {
    memset(cfg, 0, sizeof(*cfg));
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
    cJSON *medium = cJSON_GetObjectItemCaseSensitive(root, "radiomedium");
    if (cJSON_IsObject(medium)) {
        cJSON *mtype = cJSON_GetObjectItemCaseSensitive(medium, "type");
        if (cJSON_IsString(mtype) && mtype->valuestring &&
            strcmp(mtype->valuestring, "udgm") == 0) {
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

    /* Parse nodes array */
    cJSON *nodes = cJSON_GetObjectItemCaseSensitive(root, "nodes");
    if (!cJSON_IsArray(nodes)) {
        fprintf(stderr, "sim_config: 'nodes' array not found\n");
        cJSON_Delete(root);
        return -1;
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
            cJSON_Delete(root);
            return -1;
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

        count++;
    }
    cfg->node_count = count;

    /* Parse optional test section */
    cJSON *test = cJSON_GetObjectItemCaseSensitive(root, "test");
    if (cJSON_IsObject(test)) {
        cJSON *steps = cJSON_GetObjectItemCaseSensitive(test, "steps");
        if (cJSON_IsArray(steps)) {
            cfg->has_test = 1;
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
                    cJSON_Delete(root);
                    return -1;
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
    }

    cJSON_Delete(root);

    /* Auto-assign IDs for nodes without explicit id */
    for (int i = 0; i < cfg->node_count; i++) {
        if (cfg->nodes[i].id == 0) {
            cfg->nodes[i].id = i + 1;
        }
    }

    return 0;
}

void sim_config_print(const sim_config_t *cfg) {
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
    }
}
