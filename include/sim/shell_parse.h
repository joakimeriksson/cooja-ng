/*
 * shell_parse — the pure, sim-free half of the Cooja-NG shell: tokenizer,
 * time literals, node selectors.  No I/O, no kernel types, so it is unit-
 * tested directly (test/test_shell.c) and reused by the script engine.
 */
#ifndef SHELL_PARSE_H
#define SHELL_PARSE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHELL_MAX_ARGS 32

/* Split `line` into words.  Whitespace separates; "..." and '...' quote;
 * inside double quotes and bare words the escapes \n \r \t \\ \" \' \e and
 * \xHH are decoded (single quotes are literal); a '#' at the start of a
 * word begins a comment.  argv[i] points into `storage`; argpos[i] (may be
 * NULL) receives the byte offset of word i in `line`, so a command can take
 * "the rest of the line" verbatim (at, on).  Returns argc (0 for a blank
 * or comment-only line), or -1 with `err` filled (unterminated quote, bad
 * escape, too many words, storage exhausted). */
int shell_tokenize(const char *line, char **argv, int *argpos, int max_args,
                   char *storage, size_t storage_len, char *err, size_t errlen);

/* Time literal: "5s" "250ms" "1500us" "12345ns" "1.5s" "2m" "1h"; a bare
 * number is milliseconds; a leading '+' marks it relative (the caller adds
 * `now`).  Returns 0 and fills out_ns/relative, or -1. */
int shell_parse_time(const char *s, int64_t *out_ns, bool *relative);

/* Duration = time literal without the relative form ("+" rejected). */
int shell_parse_duration(const char *s, int64_t *out_ns);

/* Strict integer ("12", "-3", "0x10"); -1 on trailing junk. */
int shell_parse_int(const char *s, long *out);

/* Strict double; -1 on trailing junk. */
int shell_parse_double(const char *s, double *out);

/* Node selector → Cooja node ids.  Forms: "3", "1,4", "2-5", "1,4-6",
 * "all", and (when allow_any) "any".  `ids`/`nids` is the list of ids the
 * caller accepts; explicit ids must be in it, ranges expand to the ids in
 * it within [lo,hi] (empty range = error).  Output is in the order the ids
 * appear in `ids` (ascending slot order), duplicates removed.  Returns the
 * count, 0 with *any=true for "any", or -1 on error with `err` filled. */
int shell_parse_selector(const char *s, const int *ids, int nids,
                         int *out, int max_out, bool allow_any, bool *any,
                         char *err, size_t errlen);

/* "12.345s" style (three decimals of seconds). */
const char *shell_format_time(int64_t ns, char *buf, size_t len);

/* Compare two long values with one of == != < <= > >=; returns 1/0, or -1
 * for an unknown operator. */
int shell_compare(long a, const char *op, long b);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_PARSE_H */
