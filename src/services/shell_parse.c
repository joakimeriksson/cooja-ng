/* shell_parse — see include/sim/shell_parse.h. */
#include "shell_parse.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decode the escape whose backslash precedes *pp.  On return *pp points at
 * the escape's last character (the caller advances past it).  0 on success,
 * -1 with `err` filled. */
static int decode_escape(const char **pp, char *out, char *err, size_t errlen) {
    const char *p = *pp;
    char c;
    switch (*p) {
    case 'n': c = '\n'; break;
    case 'r': c = '\r'; break;
    case 't': c = '\t'; break;
    case 'e': c = 27;   break;
    case '\\': c = '\\'; break;
    case '"': c = '"'; break;
    case '\'': c = '\''; break;
    case ' ': c = ' '; break;
    case '#': c = '#'; break;
    case '$': c = '$'; break;
    case 'x': {
        int h = hexval(p[1]), l = (h >= 0) ? hexval(p[2]) : -1;
        if (h < 0 || l < 0) {
            if (err) snprintf(err, errlen, "bad \\x escape");
            return -1;
        }
        c = (char)(h * 16 + l);
        p += 2;
        break;
    }
    case '\0':
        if (err) snprintf(err, errlen, "trailing backslash");
        return -1;
    default:
        if (err) snprintf(err, errlen, "unknown escape \\%c", *p);
        return -1;
    }
    *pp = p;
    *out = c;
    return 0;
}

int shell_tokenize(const char *line, char **argv, int *argpos, int max_args,
                   char *storage, size_t storage_len, char *err, size_t errlen) {
    int argc = 0;
    size_t sp = 0;
    const char *p = line;
    if (err && errlen) err[0] = '\0';
    for (;;) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (!*p || *p == '#') break;
        if (argc >= max_args) {
            if (err) snprintf(err, errlen, "too many words (max %d)", max_args);
            return -1;
        }
        if (argpos) argpos[argc] = (int)(p - line);
        argv[argc] = storage + sp;
        while (*p && !isspace((unsigned char)*p)) {
            char q = 0;
            if (*p == '"' || *p == '\'') { q = *p++; }
            for (;;) {
                if (q) {
                    if (!*p) {
                        if (err) snprintf(err, errlen, "unterminated %c quote", q);
                        return -1;
                    }
                    if (*p == q) { p++; break; }
                } else {
                    if (!*p || isspace((unsigned char)*p) || *p == '"' || *p == '\'')
                        break;
                }
                char c = *p;
                if (c == '\\' && q != '\'') {
                    p++;
                    if (decode_escape(&p, &c, err, errlen) != 0) return -1;
                }
                if (sp + 2 > storage_len) {
                    if (err) snprintf(err, errlen, "line too long");
                    return -1;
                }
                storage[sp++] = c;
                p++;
            }
        }
        if (sp + 1 > storage_len) {
            if (err) snprintf(err, errlen, "line too long");
            return -1;
        }
        storage[sp++] = '\0';
        argc++;
    }
    return argc;
}

int shell_parse_time(const char *s, int64_t *out_ns, bool *relative) {
    if (!s || !*s) return -1;
    bool rel = false;
    if (*s == '+') { rel = true; s++; }
    if (!*s || *s == '-' || *s == '+') return -1;
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (end == s || errno || v < 0 || isnan(v) || isinf(v)) return -1;
    double mult;
    if (!*end)                       mult = 1e6;   /* bare number = ms */
    else if (strcmp(end, "ns") == 0) mult = 1;
    else if (strcmp(end, "us") == 0) mult = 1e3;
    else if (strcmp(end, "ms") == 0) mult = 1e6;
    else if (strcmp(end, "s") == 0)  mult = 1e9;
    else if (strcmp(end, "m") == 0)  mult = 60e9;
    else if (strcmp(end, "h") == 0)  mult = 3600e9;
    else return -1;
    double ns = v * mult;
    if (ns > 9.2e18) return -1;
    *out_ns = (int64_t)llround(ns);
    if (relative) *relative = rel;
    return 0;
}

int shell_parse_duration(const char *s, int64_t *out_ns) {
    bool rel = false;
    if (shell_parse_time(s, out_ns, &rel) != 0 || rel) return -1;
    return 0;
}

int shell_parse_int(const char *s, long *out) {
    if (!s || !*s) return -1;
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 0);
    if (end == s || *end || errno) return -1;
    *out = v;
    return 0;
}

int shell_parse_double(const char *s, double *out) {
    if (!s || !*s) return -1;
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (end == s || *end || errno || !isfinite(v)) return -1;   /* no nan, no inf */
    *out = v;
    return 0;
}

static bool id_in(const int *ids, int nids, int id) {
    for (int i = 0; i < nids; i++) if (ids[i] == id) return true;
    return false;
}

static int add_id(int *out, int n, int max_out, int id) {
    if (id_in(out, n, id)) return n;
    if (n >= max_out) return -1;
    out[n] = id;
    return n + 1;
}

int shell_parse_selector(const char *s, const int *ids, int nids,
                         int *out, int max_out, bool allow_any, bool *any,
                         char *err, size_t errlen) {
    if (any) *any = false;
    if (err && errlen) err[0] = '\0';
    if (!s || !*s) {
        if (err) snprintf(err, errlen, "empty node selector");
        return -1;
    }
    if (strcmp(s, "all") == 0 || strcmp(s, "*") == 0) {
        int n = nids < max_out ? nids : max_out;
        for (int i = 0; i < n; i++) out[i] = ids[i];
        return n;
    }
    if (strcmp(s, "any") == 0) {
        if (!allow_any) {
            if (err) snprintf(err, errlen, "'any' is not allowed here");
            return -1;
        }
        if (any) *any = true;
        return 0;
    }
    /* Explicit ids and ranges; collect as a set of wanted ids, then emit in
     * slot order so the result is deterministic. */
    bool want[4096] = {0};
    bool want_any_explicit = false;
    const char *p = s;
    while (*p) {
        char *end = NULL;
        long lo = strtol(p, &end, 10);
        if (end == p) {
            if (err) snprintf(err, errlen, "bad node selector '%s'", s);
            return -1;
        }
        long hi = lo;
        p = end;
        if (*p == '-') {
            p++;
            hi = strtol(p, &end, 10);
            if (end == p) {
                if (err) snprintf(err, errlen, "bad range in '%s'", s);
                return -1;
            }
            p = end;
        }
        if (lo < 0 || hi < lo || hi >= 4096) {
            if (err) snprintf(err, errlen, "bad range %ld-%ld", lo, hi);
            return -1;
        }
        if (lo == hi) {
            if (!id_in(ids, nids, (int)lo)) {
                if (err) snprintf(err, errlen, "no node with id %ld", lo);
                return -1;
            }
            want[lo] = true;
        } else {
            bool hit = false;
            for (long v = lo; v <= hi; v++)
                if (id_in(ids, nids, (int)v)) { want[v] = true; hit = true; }
            if (!hit) {
                if (err) snprintf(err, errlen, "no nodes in range %ld-%ld", lo, hi);
                return -1;
            }
        }
        want_any_explicit = true;
        if (*p == ',') { p++; continue; }
        if (*p) {
            if (err) snprintf(err, errlen, "bad node selector '%s'", s);
            return -1;
        }
    }
    if (!want_any_explicit) {
        if (err) snprintf(err, errlen, "empty node selector");
        return -1;
    }
    int n = 0;
    for (int i = 0; i < nids; i++) {
        int id = ids[i];
        if (id < 0 || id >= 4096 || !want[id]) continue;
        n = add_id(out, n, max_out, id);
        if (n < 0) {
            if (err) snprintf(err, errlen, "too many nodes selected");
            return -1;
        }
    }
    return n;
}

int shell_unquote_rest(const char *rest, char *out, size_t outlen,
                       char *err, size_t errlen) {
    size_t n = 0, keep = 0;        /* keep = length up to the last char that
                                    * must survive trailing-space trimming */
    char q = 0;
    bool word_start = true;
    if (err && errlen) err[0] = '\0';
    for (const char *p = rest; *p; p++) {
        char c = *p;
        bool quoted = false;
        if (!q && (c == '"' || c == '\'')) {
            q = c;
            keep = n;              /* an (even empty) quoted part is content */
            word_start = false;
            continue;
        }
        if (q && c == q) {
            q = 0;
            keep = n;
            continue;
        }
        if (!q && c == '#' && word_start)
            break;                 /* comment */
        if (c == '\\' && q != '\'') {
            p++;
            if (decode_escape(&p, &c, err, errlen) != 0) return -1;
            quoted = true;         /* an escaped space is content, not padding */
        } else if (q) {
            quoted = true;
        }
        if (n + 1 >= outlen) {
            if (err) snprintf(err, errlen, "line too long");
            return -1;
        }
        out[n++] = c;
        if (quoted || !isspace((unsigned char)c)) keep = n;
        word_start = !q && !quoted && isspace((unsigned char)c);
    }
    if (q) {
        if (err) snprintf(err, errlen, "unterminated %c quote", q);
        return -1;
    }
    out[keep] = '\0';
    return (int)keep;
}

static bool is_name_start(int c) { return isalpha(c) || c == '_'; }
static bool is_name_char(int c)  { return isalnum(c) || c == '_'; }

int shell_expand_vars(const char *in, char *out, size_t outlen,
                      shell_var_lookup_fn lookup, void *user,
                      char *err, size_t errlen) {
    size_t n = 0;
    char q = 0;
    bool word_start = true;
    if (err && errlen) err[0] = '\0';
#define EMIT(ch) do { if (n + 1 >= outlen) { if (err) snprintf(err, errlen, "line too long after expanding variables"); return -1; } out[n++] = (ch); } while (0)
    for (const char *p = in; *p; p++) {
        char c = *p;
        if (!q && c == '#' && word_start) {          /* comment: copy verbatim */
            for (; *p; p++) EMIT(*p);
            break;
        }
        if (c == '\\' && q != '\'' && p[1]) {      /* escaped char, incl. \$ */
            EMIT(c);
            EMIT(p[1]);
            p++;
            word_start = false;
            continue;
        }
        if (c == '$' && q != '\'') {
            if (p[1] == '$') { EMIT('$'); p++; word_start = false; continue; }
            char name[64];
            size_t k = 0;
            const char *e = p + 1;
            if (*e == '{') {
                e++;
                while (*e && *e != '}' && k < sizeof(name) - 1) name[k++] = *e++;
                if (*e != '}' || k == 0) {
                    if (err) snprintf(err, errlen, "bad ${...} variable reference");
                    return -1;
                }
                e++;
            } else if (is_name_start((unsigned char)*e)) {
                while (is_name_char((unsigned char)*e) && k < sizeof(name) - 1) name[k++] = *e++;
            }
            if (k == 0) { EMIT('$'); word_start = false; continue; }   /* a lone $ */
            name[k] = '\0';
            const char *val = lookup ? lookup(user, name) : NULL;
            if (!val) {
                if (err) snprintf(err, errlen, "undefined variable '%s'", name);
                return -1;
            }
            for (const char *v = val; *v; v++) EMIT(*v);
            p = e - 1;
            word_start = false;
            continue;
        }
        if (!q && (c == '"' || c == '\'')) q = c;
        else if (q && c == q) q = 0;
        EMIT(c);
        word_start = !q && isspace((unsigned char)c);
    }
#undef EMIT
    out[n] = '\0';
    return (int)n;
}

bool shell_glob_match(const char *p, const char *t) {
    const char *star = NULL, *resume = NULL;
    while (*t) {
        if (*p == '*') {
            star = p++;
            resume = t;
        } else if (*p && *p == *t) {
            p++;
            t++;
        } else if (star) {
            p = star + 1;
            t = ++resume;
        } else {
            return false;
        }
    }
    while (*p == '*') p++;
    return *p == '\0';
}

const char *shell_format_time(int64_t ns, char *buf, size_t len) {
    snprintf(buf, len, "%.3fs", (double)ns / 1e9);
    return buf;
}

int shell_compare(long a, const char *op, long b) {
    if (!op) return -1;
    if (strcmp(op, "==") == 0 || strcmp(op, "=") == 0) return a == b;
    if (strcmp(op, "!=") == 0) return a != b;
    if (strcmp(op, "<") == 0)  return a < b;
    if (strcmp(op, "<=") == 0) return a <= b;
    if (strcmp(op, ">") == 0)  return a > b;
    if (strcmp(op, ">=") == 0) return a >= b;
    return -1;
}
