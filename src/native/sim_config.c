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

        count++;
    }
    cfg->node_count = count;

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
    printf("  nodes: %d\n", cfg->node_count);
    for (int i = 0; i < cfg->node_count; i++) {
        printf("    [%d] id=%d firmware=%s\n",
               i, cfg->nodes[i].id, cfg->nodes[i].firmware);
    }
}
