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
    if (end == s || *end || errno) return -1;
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
