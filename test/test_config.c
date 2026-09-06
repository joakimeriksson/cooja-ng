/*
 * Config tooling modes:
 *
 *   config-convert   <in.{json,yaml}> <out.yaml>
 *   config-roundtrip <config...>
 *
 * The round-trip is the writer's correctness proof: for each file, load ->
 * write YAML -> load again must give the same normalized config, and writing
 * that second load must give byte-identical text (idempotence).  A field the
 * writer forgets shows up here as a mismatch, not as a quietly different
 * simulation later.
 */
#include "sim_config.h"
#include "csim_version.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static char *slurp(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    char *buf = malloc((size_t)n + 1);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); fclose(f); return NULL; }
    buf[n] = '\0'; fclose(f); return buf;
}

static int write_to(const sim_normalized_config_t *cfg, const char *path,
                    const char *header) {
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return -1; }
    int rc = sim_config_write_yaml(cfg, f, header);
    fclose(f);
    return rc;
}

int run_config_convert(int argc, char **argv);
int run_config_convert(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: test_runner config-convert <in.{json,yaml}> <out.yaml>\n");
        return 1;
    }
    sim_normalized_config_t cfg;
    if (sim_config_load(&cfg, argv[0]) != 0) return 1;
    char header[512];
    snprintf(header, sizeof(header), "converted from %s by cooja-ng %s", argv[0], CSIM_VERSION);
    int rc = write_to(&cfg, argv[1], header);
    sim_config_free(&cfg);
    if (rc == 0) printf("Wrote %s\n", argv[1]);
    return rc == 0 ? 0 : 1;
}

/* Fields the v1 -> v2 lift legitimately changes; blanked before comparing. */
static void blank_lifted(sim_normalized_config_t *c) {
    c->version = 0;
    for (int i = 0; i < c->node_count; i++) memset(c->nodes[i].type_name, 0, sizeof(c->nodes[i].type_name));
    memset(c->mote_types, 0, sizeof(c->mote_types));
    memset(c->mote_type_firmware, 0, sizeof(c->mote_type_firmware));
    c->mote_type_count = 0;
    c->js_script_inline = NULL;
}

#define CHECK(cond, what) do { if (!(cond)) { printf("    mismatch: %s\n", what); bad++; } } while (0)

static int compare(const sim_normalized_config_t *a, const sim_normalized_config_t *b) {
    int bad = 0;
    CHECK(strcmp(a->title, b->title) == 0, "title");
    CHECK(strcmp(a->description, b->description) == 0, "description");
    CHECK(a->timeout_ms == b->timeout_ms, "timeout_ms");
    CHECK(a->seed == b->seed, "seed");
    CHECK(a->startup_delay_ms == b->startup_delay_ms, "startup_delay_ms");
    CHECK(a->speed == b->speed, "speed");
    CHECK(a->medium_type == b->medium_type, "medium type");
    CHECK(strcmp(a->medium_name, b->medium_name) == 0, "medium name");
    CHECK(a->tx_range == b->tx_range && a->interference_range == b->interference_range &&
          a->success_ratio_tx == b->success_ratio_tx && a->success_ratio_rx == b->success_ratio_rx,
          "medium parameters");
    CHECK(a->node_count == b->node_count, "node count");
    for (int i = 0; i < a->node_count && i < b->node_count; i++) {
        const sim_node_config_t *x = &a->nodes[i], *y = &b->nodes[i];
        char what[64]; snprintf(what, sizeof(what), "node[%d]", i);
        CHECK(strcmp(x->firmware, y->firmware) == 0 && x->id == y->id &&
              x->has_position == y->has_position && x->x == y->x && x->y == y->y &&
              x->clock_deviation == y->clock_deviation, what);
        CHECK(x->has_peripherals == y->has_peripherals &&
              x->peripheral_count == y->peripheral_count &&
              memcmp(x->peripherals, y->peripherals, sizeof(x->peripherals)) == 0, what);
    }
    CHECK(a->plugin_count == b->plugin_count && memcmp(a->plugins, b->plugins, sizeof(a->plugins)) == 0, "plugins");
    CHECK(a->has_serial_socket == b->has_serial_socket && a->serial_socket_port == b->serial_socket_port &&
          a->serial_socket_node == b->serial_socket_node &&
          strcmp(a->serial_socket_command, b->serial_socket_command) == 0, "serial_socket");
    CHECK(a->has_test == b->has_test, "has_test");
    CHECK(memcmp(&a->test, &b->test, sizeof(a->test)) == 0, "test section");
    CHECK(a->has_js_script == b->has_js_script && strcmp(a->js_script_path, b->js_script_path) == 0, "js_script");
    CHECK((a->js_script_inline == NULL) == (b->js_script_inline == NULL) &&
          (!a->js_script_inline || strcmp(a->js_script_inline, b->js_script_inline) == 0),
          "js_script_inline");
    /* and everything else, bitwise, with the lifted fields blanked */
    sim_normalized_config_t ca = *a, cb = *b;
    blank_lifted(&ca); blank_lifted(&cb);
    CHECK(memcmp(&ca, &cb, sizeof(ca)) == 0, "remaining fields (bitwise)");
    return bad;
}

int run_config_roundtrip(int argc, char **argv);
int run_config_roundtrip(int argc, char **argv) {
    if (argc < 1) {
        fprintf(stderr, "Usage: test_runner config-roundtrip <config...>\n");
        return 1;
    }
    const char *tmpdir = getenv("TMPDIR");
    if (!tmpdir || !*tmpdir) tmpdir = "/tmp";
    char p1[1024], p2[1024];
    snprintf(p1, sizeof(p1), "%s/csim-roundtrip-%d-a.yaml", tmpdir, (int)getpid());
    snprintf(p2, sizeof(p2), "%s/csim-roundtrip-%d-b.yaml", tmpdir, (int)getpid());

    int failed = 0, passed = 0;
    printf("=== Config round-trip: load -> YAML -> load ===\n");
    for (int i = 0; i < argc; i++) {
        const char *path = argv[i];
        int bad = 0;
        sim_normalized_config_t a, b, c;
        memset(&a, 0, sizeof(a)); memset(&b, 0, sizeof(b)); memset(&c, 0, sizeof(c));
        if (sim_config_load(&a, path) != 0) { printf("  FAIL %s: does not load\n", path); failed++; continue; }
        if (write_to(&a, p1, NULL) != 0)     { printf("  FAIL %s: write\n", path); failed++; sim_config_free(&a); continue; }
        if (sim_config_load(&b, p1) != 0)    { printf("  FAIL %s: written YAML does not load back (see %s)\n", path, p1); failed++; sim_config_free(&a); continue; }
        bad += compare(&a, &b);
        if (write_to(&b, p2, NULL) != 0)     { bad++; printf("    mismatch: second write\n"); }
        else {
            char *t1 = slurp(p1), *t2 = slurp(p2);
            if (!t1 || !t2 || strcmp(t1, t2) != 0) { bad++; printf("    mismatch: writer is not idempotent (%s vs %s)\n", p1, p2); }
            free(t1); free(t2);
            if (sim_config_load(&c, p2) != 0) { bad++; printf("    mismatch: second YAML does not load\n"); }
            else { bad += compare(&b, &c); sim_config_free(&c); }
        }
        sim_config_free(&a); sim_config_free(&b);
        if (bad) { printf("  FAIL %s (%d)\n", path, bad); failed++; }
        else     { printf("  PASS %s\n", path); passed++; }
    }
    if (!failed) { unlink(p1); unlink(p2); }
    printf("\n--- Results: %d passed, %d failed ---\n", passed, failed);
    return failed;
}

/* config-reject <config...>: every file must be REFUSED by the loader.  This
 * is the fail-loudly half of the config contract under test — the fixtures
 * in test/configs/invalid/ each contain exactly one thing a config is not
 * allowed to contain (a typo'd key, a YAML anchor, a "Norway" boolean, ...),
 * and a loader that accepts any of them has a silent-green hole. */
int run_config_reject(int argc, char **argv);
int run_config_reject(int argc, char **argv) {
    int failed = 0, passed = 0;
    printf("=== Config rejection: each file must fail to load ===\n");
    for (int i = 0; i < argc; i++) {
        sim_normalized_config_t cfg;
        fprintf(stderr, "--- %s ---\n", argv[i]);
        int rc = sim_config_load(&cfg, argv[i]);
        if (rc == 0) { sim_config_free(&cfg); printf("  FAIL %s: loaded, but must be rejected\n", argv[i]); failed++; }
        else { printf("  PASS %s (rejected)\n", argv[i]); passed++; }
    }
    printf("\n--- Results: %d passed, %d failed ---\n", passed, failed);
    return failed;
}
