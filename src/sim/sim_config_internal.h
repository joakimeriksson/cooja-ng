/*
 * sim_config internals shared between the loader (sim_config.c) and the
 * YAML front end + schema validator + writer (sim_config_yaml.c).
 */
#ifndef SIM_CONFIG_INTERNAL_H
#define SIM_CONFIG_INTERNAL_H

#include "cJSON.h"
#include <stddef.h>

/* Parse YAML text into a cJSON tree using the strict subset Cooja-NG accepts
 * (see sim_config_yaml.c for the rules).  Returns NULL and prints a
 * "path:line:col: message" diagnostic on any error. */
cJSON *sim_config_yaml_to_cjson(const char *text, size_t len, const char *path);

/* Validate a parsed config tree against the v1/v2 schema: unknown keys,
 * duplicate keys and wrong value types are all errors, printed with their
 * path.  Returns 0 if clean, -1 otherwise.  Runs on both formats. */
int sim_config_validate_tree(const cJSON *root, int version, const char *path);

#endif
