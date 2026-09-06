/*
 * YAML front end, schema validator and canonical writer for sim_config.
 *
 * THE SUBSET
 *
 * Cooja-NG reads YAML with libyaml but accepts only what JSON can express,
 * plus comments and block scalars.  The point is predictability: a config
 * must mean exactly one thing, and anything outside the subset is an error
 * with a file:line:column — never a guess.  Concretely:
 *
 *   - one document per file; `---` twice is an error
 *   - no anchors (&x), aliases (*x) or explicit tags (!!str, !foo)
 *   - mapping keys are plain scalars; duplicates are errors
 *   - plain scalars are typed by the YAML 1.2 core rules, strictly:
 *       true / false            -> bool
 *       null / ~ / (empty)      -> null
 *       [-+]?digits             -> number
 *       [-+]?decimal[.e]…       -> number
 *       anything else           -> string
 *     and the YAML 1.1 booleans (yes/no/on/off/y/n, any capitalization of
 *     true/false/null) are REJECTED as ambiguous rather than being read as
 *     strings — the classic "Norway problem" (`country: NO` -> false) can
 *     not happen here in either direction.  .inf/.nan, hex (0x), octal (0o)
 *     and underscore-separated numbers are rejected too.
 *   - quoted scalars ('…' / "…") and block scalars (| / >) are always strings
 *
 * JSON files go through cJSON instead (JSON is valid YAML, but JSON users
 * expect JSON's exact rules, e.g. tabs as whitespace); both parsers produce
 * the same cJSON tree, and sim_config_validate_tree() below runs on that
 * tree for both, so there is one schema and one set of rules.
 */
#include "sim_config.h"
#include "sim_config_internal.h"

#include <yaml.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* YAML -> cJSON                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    yaml_parser_t parser;
    const char   *path;
    int           failed;
} yctx_t;

static void yerr(yctx_t *c, const yaml_mark_t *m, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
static void yerr(yctx_t *c, const yaml_mark_t *m, const char *fmt, ...) {
    if (c->failed) return;
    c->failed = 1;
    fprintf(stderr, "%s:%d:%d: ", c->path,
            m ? (int)m->line + 1 : 0, m ? (int)m->column + 1 : 0);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static int next_event(yctx_t *c, yaml_event_t *ev) {
    if (!yaml_parser_parse(&c->parser, ev)) {
        yerr(c, &c->parser.problem_mark, "YAML syntax error: %s%s%s",
             c->parser.problem ? c->parser.problem : "(unknown)",
             c->parser.context ? " — " : "",
             c->parser.context ? c->parser.context : "");
        return 0;
    }
    return 1;
}

static int is_all(const char *s, int (*pred)(int)) {
    for (; *s; s++) if (!pred((unsigned char)*s)) return 0;
    return 1;
}

/* Strict number test: sign, digits, optional fraction, optional exponent.
 * Deliberately narrower than strtod (no hex, inf, nan, leading '.'-less
 * exponent forms are fine, underscores are not). */
static int looks_numeric(const char *s, double *out) {
    const char *p = s;
    if (*p == '+' || *p == '-') p++;
    int digits = 0, dot = 0;
    while (isdigit((unsigned char)*p)) { p++; digits++; }
    if (*p == '.') { dot = 1; p++; while (isdigit((unsigned char)*p)) { p++; digits++; } }
    if (!digits) return 0;
    if (*p == 'e' || *p == 'E') {
        p++;
        if (*p == '+' || *p == '-') p++;
        int ed = 0;
        while (isdigit((unsigned char)*p)) { p++; ed++; }
        if (!ed) return 0;
    }
    if (*p) return 0;
    (void)dot;
    errno = 0;
    *out = strtod(s, NULL);
    return errno == 0;
}

/* Words that YAML 1.1 read as booleans/null and that a human might still
 * write expecting *something* — refused so the file says what it means. */
static int is_ambiguous_word(const char *s) {
    static const char *words[] = {
        "yes", "no", "on", "off", "y", "n", "true", "false", "null", NULL
    };
    char low[8];
    size_t n = strlen(s);
    if (n == 0 || n >= sizeof(low)) return 0;
    for (size_t i = 0; i <= n; i++) low[i] = (char)tolower((unsigned char)s[i]);
    for (int i = 0; words[i]; i++)
        if (strcmp(low, words[i]) == 0) {
            /* the exact lowercase spellings are the accepted ones */
            if (strcmp(s, "true") == 0 || strcmp(s, "false") == 0 ||
                strcmp(s, "null") == 0) return 0;
            return 1;
        }
    return 0;
}

static cJSON *scalar_to_cjson(yctx_t *c, const yaml_event_t *ev) {
    const char *v = (const char *)ev->data.scalar.value;
    if (ev->data.scalar.style != YAML_PLAIN_SCALAR_STYLE)
        return cJSON_CreateString(v);                 /* quoted / block: string */

    if (*v == '\0' || strcmp(v, "~") == 0 || strcmp(v, "null") == 0)
        return cJSON_CreateNull();
    if (strcmp(v, "true") == 0)  return cJSON_CreateTrue();
    if (strcmp(v, "false") == 0) return cJSON_CreateFalse();
    if (is_ambiguous_word(v)) {
        yerr(c, &ev->start_mark,
             "ambiguous scalar '%s': write true/false/null in lowercase, or "
             "quote it (\"%s\") to mean the string", v, v);
        return NULL;
    }
    double d;
    if (looks_numeric(v, &d)) return cJSON_CreateNumber(d);
    if (*v == '.' && (strcasecmp(v, ".inf") == 0 || strcasecmp(v, "-.inf") == 0 ||
                      strcasecmp(v, "+.inf") == 0 || strcasecmp(v, ".nan") == 0)) {
        yerr(c, &ev->start_mark, "unsupported numeric scalar '%s'", v);
        return NULL;
    }
    if ((v[0] == '0' && (v[1] == 'x' || v[1] == 'o' || v[1] == 'b')) ||
        (isdigit((unsigned char)v[0]) && strchr(v, '_') && is_all(v, isalnum))) {
        yerr(c, &ev->start_mark,
             "unsupported numeric form '%s' (use plain decimal, or quote it)", v);
        return NULL;
    }
    return cJSON_CreateString(v);
}

static cJSON *build_node(yctx_t *c, yaml_event_t *ev);

static int reject_anchor_tag(yctx_t *c, const yaml_event_t *ev,
                             const yaml_char_t *anchor, const yaml_char_t *tag,
                             int implicit) {
    if (anchor) {
        yerr(c, &ev->start_mark, "anchors (&%s) are not supported — repeat the "
             "value, or generate the config", (const char *)anchor);
        return -1;
    }
    if (tag && !implicit) {
        yerr(c, &ev->start_mark, "explicit tags (%s) are not supported",
             (const char *)tag);
        return -1;
    }
    return 0;
}

static cJSON *build_mapping(yctx_t *c) {
    cJSON *obj = cJSON_CreateObject();
    for (;;) {
        yaml_event_t ev;
        if (!next_event(c, &ev)) goto fail;
        if (ev.type == YAML_MAPPING_END_EVENT) { yaml_event_delete(&ev); return obj; }
        if (ev.type != YAML_SCALAR_EVENT ||
            ev.data.scalar.style != YAML_PLAIN_SCALAR_STYLE) {
            yerr(c, &ev.start_mark, "mapping keys must be plain scalars");
            yaml_event_delete(&ev); goto fail;
        }
        char key[256];
        snprintf(key, sizeof(key), "%s", (const char *)ev.data.scalar.value);
        yaml_mark_t kmark = ev.start_mark;
        yaml_event_delete(&ev);
        if (cJSON_GetObjectItemCaseSensitive(obj, key)) {
            yerr(c, &kmark, "duplicate key '%s'", key);
            goto fail;
        }
        if (!next_event(c, &ev)) goto fail;
        cJSON *val = build_node(c, &ev);
        if (!val) goto fail;
        cJSON_AddItemToObject(obj, key, val);
    }
fail:
    cJSON_Delete(obj);
    return NULL;
}

static cJSON *build_sequence(yctx_t *c) {
    cJSON *arr = cJSON_CreateArray();
    for (;;) {
        yaml_event_t ev;
        if (!next_event(c, &ev)) goto fail;
        if (ev.type == YAML_SEQUENCE_END_EVENT) { yaml_event_delete(&ev); return arr; }
        cJSON *val = build_node(c, &ev);
        if (!val) goto fail;
        cJSON_AddItemToArray(arr, val);
    }
fail:
    cJSON_Delete(arr);
    return NULL;
}

/* Consumes (and deletes) `ev`. */
static cJSON *build_node(yctx_t *c, yaml_event_t *ev) {
    cJSON *out = NULL;
    switch (ev->type) {
    case YAML_SCALAR_EVENT:
        if (reject_anchor_tag(c, ev, ev->data.scalar.anchor, ev->data.scalar.tag,
                              ev->data.scalar.plain_implicit ||
                              ev->data.scalar.quoted_implicit) == 0)
            out = scalar_to_cjson(c, ev);
        break;
    case YAML_SEQUENCE_START_EVENT:
        if (reject_anchor_tag(c, ev, ev->data.sequence_start.anchor,
                              ev->data.sequence_start.tag,
                              ev->data.sequence_start.implicit) == 0)
            out = build_sequence(c);
        break;
    case YAML_MAPPING_START_EVENT:
        if (reject_anchor_tag(c, ev, ev->data.mapping_start.anchor,
                              ev->data.mapping_start.tag,
                              ev->data.mapping_start.implicit) == 0)
            out = build_mapping(c);
        break;
    case YAML_ALIAS_EVENT:
        yerr(c, &ev->start_mark, "aliases (*%s) are not supported",
             (const char *)ev->data.alias.anchor);
        break;
    default:
        yerr(c, &ev->start_mark, "unexpected YAML event %d", (int)ev->type);
        break;
    }
    yaml_event_delete(ev);
    return out;
}

cJSON *sim_config_yaml_to_cjson(const char *text, size_t len, const char *path) {
    yctx_t c = { .path = path, .failed = 0 };
    if (!yaml_parser_initialize(&c.parser)) {
        fprintf(stderr, "%s: cannot initialize YAML parser\n", path);
        return NULL;
    }
    yaml_parser_set_input_string(&c.parser, (const unsigned char *)text, len);

    cJSON *root = NULL;
    yaml_event_t ev;
    if (!next_event(&c, &ev)) goto out;
    if (ev.type != YAML_STREAM_START_EVENT) { yerr(&c, &ev.start_mark, "expected stream start"); yaml_event_delete(&ev); goto out; }
    yaml_event_delete(&ev);

    if (!next_event(&c, &ev)) goto out;
    if (ev.type == YAML_STREAM_END_EVENT) { yerr(&c, &ev.start_mark, "empty file: no configuration"); yaml_event_delete(&ev); goto out; }
    if (ev.type != YAML_DOCUMENT_START_EVENT) { yerr(&c, &ev.start_mark, "expected document start"); yaml_event_delete(&ev); goto out; }
    yaml_event_delete(&ev);

    if (!next_event(&c, &ev)) goto out;
    root = build_node(&c, &ev);
    if (!root) goto out;

    if (!next_event(&c, &ev)) goto fail;
    if (ev.type != YAML_DOCUMENT_END_EVENT) { yerr(&c, &ev.start_mark, "expected end of document"); yaml_event_delete(&ev); goto fail; }
    yaml_event_delete(&ev);
    if (!next_event(&c, &ev)) goto fail;
    if (ev.type != YAML_STREAM_END_EVENT) {
        yerr(&c, &ev.start_mark, "only one document per config file (second '---' found)");
        yaml_event_delete(&ev); goto fail;
    }
    yaml_event_delete(&ev);
    if (!cJSON_IsObject(root)) {
        fprintf(stderr, "%s: top level must be a mapping (key: value pairs)\n", path);
        goto fail;
    }
    goto out;
fail:
    cJSON_Delete(root);
    root = NULL;
out:
    yaml_parser_delete(&c.parser);
    return c.failed ? (cJSON_Delete(root), NULL) : root;
}

/* ------------------------------------------------------------------ */
/* Schema validation (both formats)                                    */
/* ------------------------------------------------------------------ */
/*
 * Table-driven: every key the parsers read, with the type they expect.  A
 * key not in the table for its object is an error — that is what turns a
 * typo like `tx_rang:` from a silently-ignored default into a diagnosis.
 * Type codes: s string, n number, b bool, o object (sub), a array of
 * objects (sub), S array of strings, * anything (tooling metadata that the
 * runtime never reads: csc2json's build info, kept so those files load).
 */
typedef struct schema schema_t;
typedef struct { const char *key; char type; const schema_t *sub; } field_t;
struct schema { const char *name; const field_t *fields; };

static const schema_t medium_schema, node_v1_schema, node_v2_schema,
    mote_type_schema, test_schema, step_schema, validator_schema,
    action_schema, serial_schema, build_schema, root_v1_schema, root_v2_schema,
    peripheral_schema;

static const field_t medium_fields[] = {
    {"type", 's', 0}, {"tx_range", 'n', 0}, {"interference_range", 'n', 0},
    {"success_ratio_tx", 'n', 0}, {"success_ratio_rx", 'n', 0}, {0, 0, 0}
};
static const field_t build_fields[] = {   /* csc2json metadata, runtime-inert */
    {"make_args", 'S', 0}, {"source_dir", 's', 0}, {"target", 's', 0}, {0, 0, 0}
};
static const field_t peripheral_fields[] = {   /* off-SoC SPI chip on a node */
    {"chip", 's', 0}, {"spim", 'n', 0}, {"cs", 's', 0}, {0, 0, 0}
};
static const field_t node_v1_fields[] = {
    {"firmware", 's', 0}, {"id", 'n', 0}, {"x", 'n', 0}, {"y", 'n', 0},
    {"clock_deviation", 'n', 0}, {"peripherals", 'a', &peripheral_schema},
    {"build", 'o', &build_schema}, {"_mote_type_desc", 's', 0}, {0, 0, 0}
};
static const field_t node_v2_fields[] = {
    {"type", 's', 0}, {"id", 'n', 0}, {"x", 'n', 0}, {"y", 'n', 0},
    {"clock_deviation", 'n', 0}, {"peripherals", 'a', &peripheral_schema},
    {"build", 'o', &build_schema}, {"_mote_type_desc", 's', 0}, {0, 0, 0}
};
static const field_t mote_type_fields[] = {
    {"name", 's', 0}, {"kind", 's', 0}, {"cpu", 's', 0}, {"soc", 's', 0},
    {"board", 's', 0}, {"firmware", 's', 0}, {"description", 's', 0}, {0, 0, 0}
};
static const field_t step_fields[] = {
    {"wait", 's', 0}, {"node", 'n', 0}, {"count", 'n', 0}, {"timeout_ms", 'n', 0}, {0, 0, 0}
};
static const field_t validator_fields[] = {
    {"pattern", 's', 0}, {"min_count", 'n', 0}, {"node", 'n', 0}, {0, 0, 0}
};
static const field_t action_fields[] = {
    {"at_ms", 'n', 0}, {"type", 's', 0}, {"node", 'n', 0}, {"x", 'n', 0},
    {"y", 'n', 0}, {"data", 's', 0}, {"mote_type", 'n', 0}, {0, 0, 0}
};
static const field_t test_fields[] = {
    {"description", 's', 0}, {"js_script", 's', 0}, {"js_script_inline", 's', 0},
    {"steps", 'a', &step_schema}, {"fail_on", 'S', 0}, {"timeout_is_success", 'b', 0},
    {"validators", 'a', &validator_schema}, {"actions", 'a', &action_schema}, {0, 0, 0}
};
static const field_t serial_fields[] = {
    {"port", 'n', 0}, {"node", 'n', 0}, {"command", 's', 0}, {0, 0, 0}
};
static const field_t root_v1_fields[] = {
    {"version", 'n', 0}, {"title", 's', 0}, {"description", 's', 0}, {"note", 's', 0},
    {"timeout_ms", 'n', 0}, {"seed", 'n', 0}, {"startup_delay_ms", 'n', 0},
    {"speed", 'n', 0}, {"radiomedium", 'o', &medium_schema},
    {"nodes", 'a', &node_v1_schema}, {"test", 'o', &test_schema},
    {"mote_types", 'a', &mote_type_schema}, {"serial_socket", 'o', &serial_schema},
    {0, 0, 0}
};
static const field_t root_v2_fields[] = {
    {"version", 'n', 0}, {"title", 's', 0}, {"description", 's', 0}, {"note", 's', 0},
    {"timeout_ms", 'n', 0}, {"seed", 'n', 0}, {"startup_delay_ms", 'n', 0},
    {"speed", 'n', 0}, {"medium", 'o', &medium_schema},
    {"mote_types", 'a', &mote_type_schema}, {"plugins", 'S', 0},
    {"nodes", 'a', &node_v2_schema}, {"test", 'o', &test_schema},
    {"serial_socket", 'o', &serial_schema}, {0, 0, 0}
};
static const schema_t medium_schema    = {"medium", medium_fields};
static const schema_t build_schema     = {"build", build_fields};
static const schema_t peripheral_schema = {"peripheral", peripheral_fields};
static const schema_t node_v1_schema   = {"node", node_v1_fields};
static const schema_t node_v2_schema   = {"node", node_v2_fields};
static const schema_t mote_type_schema = {"mote_type", mote_type_fields};
static const schema_t step_schema      = {"step", step_fields};
static const schema_t validator_schema = {"validator", validator_fields};
static const schema_t action_schema    = {"action", action_fields};
static const schema_t test_schema      = {"test", test_fields};
static const schema_t serial_schema    = {"serial_socket", serial_fields};
static const schema_t root_v1_schema   = {"config (v1)", root_v1_fields};
static const schema_t root_v2_schema   = {"config (v2)", root_v2_fields};

static const char *type_name(char t) {
    switch (t) {
    case 's': return "a string"; case 'n': return "a number"; case 'b': return "a boolean";
    case 'o': return "a mapping"; case 'a': return "a list of mappings";
    case 'S': return "a list of strings"; default: return "any value";
    }
}
static const char *actual_name(const cJSON *v) {
    if (cJSON_IsString(v)) return "a string";
    if (cJSON_IsNumber(v)) return "a number";
    if (cJSON_IsBool(v))   return "a boolean";
    if (cJSON_IsNull(v))   return "null";
    if (cJSON_IsObject(v)) return "a mapping";
    if (cJSON_IsArray(v))  return "a list";
    return "an unknown value";
}

static int validate_object(const cJSON *obj, const schema_t *sc,
                           const char *path, const char *where, int *errors);

static int validate_value(const cJSON *v, const field_t *f, const char *path,
                          const char *where, int *errors) {
    int ok;
    switch (f->type) {
    case 's': ok = cJSON_IsString(v); break;
    case 'n': ok = cJSON_IsNumber(v); break;
    case 'b': ok = cJSON_IsBool(v);   break;
    case 'o': ok = cJSON_IsObject(v); break;
    case 'a': case 'S': ok = cJSON_IsArray(v); break;
    default:  ok = 1; break;
    }
    if (!ok) {
        fprintf(stderr, "%s: %s must be %s, got %s\n", path, where,
                type_name(f->type), actual_name(v));
        (*errors)++;
        return -1;
    }
    if (f->type == 'o' && f->sub)
        validate_object(v, f->sub, path, where, errors);
    if (f->type == 'a' || f->type == 'S') {
        int i = 0;
        const cJSON *item;
        cJSON_ArrayForEach(item, v) {
            char sub[256];
            snprintf(sub, sizeof(sub), "%s[%d]", where, i++);
            if (f->type == 'S') {
                if (!cJSON_IsString(item)) {
                    fprintf(stderr, "%s: %s must be a string, got %s\n", path, sub,
                            actual_name(item));
                    (*errors)++;
                }
            } else if (!cJSON_IsObject(item)) {
                fprintf(stderr, "%s: %s must be a mapping, got %s\n", path, sub,
                        actual_name(item));
                (*errors)++;
            } else if (f->sub) {
                validate_object(item, f->sub, path, sub, errors);
            }
        }
    }
    return 0;
}

static int validate_object(const cJSON *obj, const schema_t *sc,
                           const char *path, const char *where, int *errors) {
    const cJSON *child;
    cJSON_ArrayForEach(child, obj) {
        const char *key = child->string ? child->string : "";
        /* duplicate key (the JSON parser accepts them; the YAML front end
         * already refused them, so this only fires for JSON) */
        for (const cJSON *later = child->next; later; later = later->next) {
            if (later->string && strcmp(later->string, key) == 0) {
                fprintf(stderr, "%s: duplicate key '%s' in %s\n", path, key, where);
                (*errors)++;
                break;
            }
        }
        const field_t *f = sc->fields;
        for (; f->key; f++) if (strcmp(f->key, key) == 0) break;
        if (!f->key) {
            fprintf(stderr, "%s: unknown key '%s' in %s", path, key, where);
            /* the one cross-version mistake worth a hint */
            if (strcmp(key, "radiomedium") == 0)
                fprintf(stderr, " (v2 calls it 'medium')");
            else if (strcmp(key, "medium") == 0)
                fprintf(stderr, " (v1 calls it 'radiomedium'; set 'version: 2')");
            else if (strcmp(key, "plugins") == 0)
                fprintf(stderr, " ('plugins' needs 'version: 2')");
            else if (strcmp(key, "firmware") == 0 && sc == &node_v2_schema)
                fprintf(stderr, " (v2 nodes reference a mote type by 'type')");
            fputc('\n', stderr);
            (*errors)++;
            continue;
        }
        char sub[256];
        snprintf(sub, sizeof(sub), "%s.%s", where, key);
        validate_value(child, f, path, sub, errors);
    }
    return *errors ? -1 : 0;
}

int sim_config_validate_tree(const cJSON *root, int version, const char *path) {
    int errors = 0;
    if (!cJSON_IsObject(root)) {
        fprintf(stderr, "%s: top level must be a mapping\n", path);
        return -1;
    }
    validate_object(root, version >= 2 ? &root_v2_schema : &root_v1_schema,
                    path, version >= 2 ? "config" : "config", &errors);
    if (errors) {
        fprintf(stderr, "%s: %d config error%s — every key must be known and "
                "correctly typed; nothing is ignored silently\n",
                path, errors, errors == 1 ? "" : "s");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Canonical YAML writer                                               */
/* ------------------------------------------------------------------ */

/* Emit a string scalar: plain when it is unambiguous, double-quoted
 * otherwise.  "Unambiguous" is deliberately narrow so that whatever we write
 * reads back through the strict parser above as the same string. */
static void put_str(FILE *f, const char *s) {
    int plain = (*s != '\0');
    if (plain) {
        double d;
        if (looks_numeric(s, &d) || is_ambiguous_word(s) ||
            strcmp(s, "true") == 0 || strcmp(s, "false") == 0 ||
            strcmp(s, "null") == 0 || strcmp(s, "~") == 0)
            plain = 0;
    }
    if (plain) {
        /* first char must not be an indicator; body must be "word-ish" */
        if (strchr("-?:,[]{}#&*!|>'\"%@` ", *s)) plain = 0;
        for (const char *p = s; plain && *p; p++)
            if (!(isalnum((unsigned char)*p) || strchr("_-./+", *p))) plain = 0;
    }
    if (plain) { fputs(s, f); return; }
    fputc('"', f);
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        switch (*p) {
        case '"':  fputs("\\\"", f); break;
        case '\\': fputs("\\\\", f); break;
        case '\n': fputs("\\n", f);  break;
        case '\t': fputs("\\t", f);  break;
        case '\r': fputs("\\r", f);  break;
        default:
            if (*p < 0x20) fprintf(f, "\\x%02x", *p);
            else fputc(*p, f);
        }
    }
    fputc('"', f);
}

/* Shortest decimal that reads back to the same double; always carries a
 * '.' or 'e' so it is visibly a float. */
static void put_num(FILE *f, double d) {
    char buf[32], first_exact[32] = "";
    for (int prec = 1; prec <= 17; prec++) {
        snprintf(buf, sizeof(buf), "%.*g", prec, d);
        if (strtod(buf, NULL) != d) continue;
        if (!first_exact[0]) snprintf(first_exact, sizeof(first_exact), "%s", buf);
        if (!strpbrk(buf, "eE")) break;    /* prefer 50.0 over 5e+01 */
    }
    if (strtod(buf, NULL) != d) snprintf(buf, sizeof(buf), "%s", first_exact);
    fputs(buf, f);
    if (!strpbrk(buf, ".eE")) fputs(".0", f);
}

static void put_comment_lines(FILE *f, const char *text) {
    const char *p = text;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        fprintf(f, "# %.*s\n", (int)n, p);
        p += n + (nl ? 1 : 0);
    }
}

/* Literal block scalar, exact for any text.  Two YAML details matter:
 *  - if the first content line starts with a space the block needs an
 *    explicit indentation indicator, otherwise YAML would take that deeper
 *    indentation as the block's own;
 *  - chomping: `|-` strips the final line break, `|` keeps exactly one, and
 *    `|+` keeps every trailing line break — so the indicator is chosen from
 *    how many newlines the text actually ends with.  (This is what the
 *    round-trip test caught on a script ending in a blank line.) */
static void put_block(FILE *f, const char *key, const char *text, int indent) {
    const char *first = text;
    while (*first == '\n') first++;
    if (*first == '\0') {                 /* nothing but newlines: a block
                                          * scalar cannot express that exactly */
        fprintf(f, "%*s%s: ", indent, "", key);
        put_str(f, text);
        fputc('\n', f);
        return;
    }
    size_t n = strlen(text), trailing = 0;
    while (trailing < n && text[n - 1 - trailing] == '\n') trailing++;
    fprintf(f, "%*s%s: |%s%s\n", indent, "", key,
            (*first == ' ' || *first == '\t') ? "2" : "",
            trailing == 0 ? "-" : trailing == 1 ? "" : "+");
    const char *p = text;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        if (len == 0) fputc('\n', f);
        else fprintf(f, "%*s%.*s\n", indent + 2, "", (int)len, p);
        p += len + (nl ? 1 : 0);
    }
}

/* Mote-type naming for the writer.  Existing mote_types keep their order
 * (getMoteTypes()[i] in JS scripts depends on it); firmware paths not
 * covered by one get a synthesized type named after the file stem. */
typedef struct { char name[64]; char firmware[256]; const sim_mote_type_t *src; } wtype_t;

static void stem_of(const char *fw, char *out, size_t sz) {
    const char *base = strrchr(fw, '/');
    base = base ? base + 1 : fw;
    snprintf(out, sz, "%s", base);
    char *dot = strrchr(out, '.');
    if (dot && dot != out) *dot = '\0';
}

static int build_types(const sim_normalized_config_t *cfg, wtype_t *types, int max) {
    int n = 0;
    for (int i = 0; i < cfg->mote_type_count && n < max; i++) {
        wtype_t *t = &types[n++];
        const sim_mote_type_t *mt = &cfg->mote_types[i];
        const char *fw = mt->firmware[0] ? mt->firmware : cfg->mote_type_firmware[i];
        snprintf(t->firmware, sizeof(t->firmware), "%s", fw);
        t->src = mt->name[0] ? mt : NULL;
        if (mt->name[0]) snprintf(t->name, sizeof(t->name), "%s", mt->name);
        else stem_of(fw, t->name, sizeof(t->name));
    }
    for (int i = 0; i < cfg->node_count && n < max; i++) {
        const char *fw = cfg->nodes[i].firmware;
        int found = 0;
        for (int k = 0; k < n; k++) if (strcmp(types[k].firmware, fw) == 0) { found = 1; break; }
        if (found) continue;
        wtype_t *t = &types[n];
        snprintf(t->firmware, sizeof(t->firmware), "%s", fw);
        t->src = NULL;
        stem_of(fw, t->name, sizeof(t->name));
        n++;
    }
    /* de-duplicate names (same stem, different extension): append the ext */
    for (int i = 0; i < n; i++)
        for (int k = i + 1; k < n; k++)
            if (strcmp(types[i].name, types[k].name) == 0) {
                const char *ext = strrchr(types[k].firmware, '.');
                if (ext) {
                    char tmp[64];
                    snprintf(tmp, sizeof(tmp), "%s-%s", types[k].name, ext + 1);
                    snprintf(types[k].name, sizeof(types[k].name), "%s", tmp);
                }
            }
    return n;
}

int sim_config_write_yaml(const sim_normalized_config_t *cfg, FILE *f,
                          const char *header) {
    if (header && *header) { put_comment_lines(f, header); fputc('\n', f); }

    fprintf(f, "version: 2\n");
    if (cfg->title[0]) { fputs("title: ", f); put_str(f, cfg->title); fputc('\n', f); }
    if (cfg->description[0]) { fputs("description: ", f); put_str(f, cfg->description); fputc('\n', f); }
    fprintf(f, "timeout_ms: %d\n", cfg->timeout_ms);
    if (cfg->seed) fprintf(f, "seed: %d          # same seed => byte-identical run\n", cfg->seed);
    if (cfg->startup_delay_ms) fprintf(f, "startup_delay_ms: %d\n", cfg->startup_delay_ms);
    if (cfg->speed > 0) { fputs("speed: ", f); put_num(f, cfg->speed); fputc('\n', f); }

    if (cfg->medium_type || cfg->medium_name[0]) {
        fputs("\nmedium:\n", f);
        fputs("  type: ", f);
        put_str(f, cfg->medium_name[0] ? cfg->medium_name : "udgm");
        fputc('\n', f);
        fputs("  tx_range: ", f);           put_num(f, cfg->tx_range);           fputs("           # metres\n", f);
        fputs("  interference_range: ", f); put_num(f, cfg->interference_range); fputc('\n', f);
        fputs("  success_ratio_tx: ", f);   put_num(f, cfg->success_ratio_tx);   fputc('\n', f);
        fputs("  success_ratio_rx: ", f);   put_num(f, cfg->success_ratio_rx);   fputc('\n', f);
    }

    if (cfg->plugin_count) {
        fputs("\nplugins:\n", f);
        for (int i = 0; i < cfg->plugin_count; i++) {
            if (!cfg->plugins[i][0]) continue;
            fputs("  - ", f); put_str(f, cfg->plugins[i]); fputc('\n', f);
        }
    }

    wtype_t types[MAX_SIM_NODES + 8];
    int ntypes = build_types(cfg, types, (int)(sizeof(types) / sizeof(types[0])));
    fputs("\nmote_types:\n", f);
    for (int i = 0; i < ntypes; i++) {
        const wtype_t *t = &types[i];
        fputs("  - { name: ", f); put_str(f, t->name);
        if (t->src) {
            if (t->src->kind[0])  { fputs(", kind: ", f);  put_str(f, t->src->kind); }
            if (t->src->cpu[0])   { fputs(", cpu: ", f);   put_str(f, t->src->cpu); }
            if (t->src->soc[0])   { fputs(", soc: ", f);   put_str(f, t->src->soc); }
            if (t->src->board[0]) { fputs(", board: ", f); put_str(f, t->src->board); }
        }
        fputs(", firmware: ", f); put_str(f, t->firmware);
        fputs(" }\n", f);
    }

    fputs("\nnodes:\n", f);
    for (int i = 0; i < cfg->node_count; i++) {
        const sim_node_config_t *n = &cfg->nodes[i];
        const char *tname = "?";
        for (int k = 0; k < ntypes; k++)
            if (strcmp(types[k].firmware, n->firmware) == 0) { tname = types[k].name; break; }
        fputs("  - { type: ", f); put_str(f, tname);
        fprintf(f, ", id: %d", n->id);
        if (n->has_position) {
            fputs(", x: ", f); put_num(f, n->x);
            fputs(", y: ", f); put_num(f, n->y);
        }
        if (n->clock_deviation != 1.0 && n->clock_deviation != 0.0) {
            fputs(", clock_deviation: ", f); put_num(f, n->clock_deviation);
        }
        if (n->has_peripherals) {
            fputs(", peripherals: [", f);
            for (int k = 0; k < n->peripheral_count; k++) {
                const sim_peripheral_config_t *pc = &n->peripherals[k];
                fprintf(f, "%s{ chip: ", k ? ", " : " "); put_str(f, pc->chip);
                fprintf(f, ", spim: %d, cs: P%d.%02d }", pc->spim, pc->cs_port, pc->cs_pin);
            }
            fputs(n->peripheral_count ? " ]" : "]", f);
        }
        fputs(" }\n", f);
    }

    if (cfg->has_serial_socket) {
        fputs("\nserial_socket:\n", f);
        fprintf(f, "  port: %d\n", cfg->serial_socket_port);
        fprintf(f, "  node: %d\n", cfg->serial_socket_node);
        if (cfg->serial_socket_command[0]) {
            fputs("  command: ", f); put_str(f, cfg->serial_socket_command); fputc('\n', f);
        }
    }

    if (cfg->has_test) {
        const sim_test_config_t *t = &cfg->test;
        fputs("\ntest:\n", f);
        if (t->description[0]) { fputs("  description: ", f); put_str(f, t->description); fputc('\n', f); }
        if (cfg->js_script_path[0]) { fputs("  js_script: ", f); put_str(f, cfg->js_script_path); fputc('\n', f); }
        if (cfg->js_script_inline) put_block(f, "js_script_inline", cfg->js_script_inline, 2);
        if (t->timeout_is_success) fputs("  timeout_is_success: true\n", f);
        if (t->step_count) {
            fputs("  steps:\n", f);
            for (int i = 0; i < t->step_count; i++) {
                const sim_test_step_t *s = &t->steps[i];
                fputs("    - { wait: ", f); put_str(f, s->pattern);
                if (s->node >= 0)  fprintf(f, ", node: %d", s->node);
                if (s->count != 1) fprintf(f, ", count: %d", s->count);
                if (s->timeout_ms) fprintf(f, ", timeout_ms: %d", s->timeout_ms);
                fputs(" }\n", f);
            }
        }
        if (t->fail_on_count) {
            fputs("  fail_on:\n", f);
            for (int i = 0; i < t->fail_on_count; i++) { fputs("    - ", f); put_str(f, t->fail_on[i]); fputc('\n', f); }
        }
        if (t->validator_count) {
            fputs("  validators:\n", f);
            for (int i = 0; i < t->validator_count; i++) {
                const sim_test_validator_t *v = &t->validators[i];
                fputs("    - { pattern: ", f); put_str(f, v->pattern);
                if (v->node >= 0) fprintf(f, ", node: %d", v->node);
                fprintf(f, ", min_count: %d }\n", v->min_count);
            }
        }
        if (t->action_count) {
            fputs("  actions:\n", f);
            for (int i = 0; i < t->action_count; i++) {
                const sim_test_action_t *a = &t->actions[i];
                static const char *names[] = { "?", "move", "send", "remove", "add", "send_all" };
                const char *tn = (a->type >= 1 && a->type <= 5) ? names[a->type] : "?";
                fprintf(f, "    - { at_ms: %lld, type: %s", (long long)a->at_ms, tn);
                if (a->type != TEST_ACTION_SEND_ALL) fprintf(f, ", node: %d", a->node);
                if (a->type == TEST_ACTION_MOVE) { fputs(", x: ", f); put_num(f, a->x); fputs(", y: ", f); put_num(f, a->y); }
                if (a->type == TEST_ACTION_ADD)  { fprintf(f, ", mote_type: %d", a->mote_type); fputs(", x: ", f); put_num(f, a->x); fputs(", y: ", f); put_num(f, a->y); }
                if (a->data[0]) { fputs(", data: ", f); put_str(f, a->data); }
                fputs(" }\n", f);
            }
        }
    }
    return ferror(f) ? -1 : 0;
}
