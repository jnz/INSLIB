/** @file mini_yaml.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Minimal YAML-subset reader shared by tools/replay.c and tools/insrcv.c
 * for the config.yaml files: two levels ("section:" +
 * 2-space-indented "key: value"), scalars, float lists in either the
 * inline "[a, b, c]" form the converters emit or a block sequence
 * (REQ-VER-028), and '#' comments. Anything fancier (anchors, nested
 * mappings, quoting) stays unsupported: a nested mapping's inner keys
 * arrive under the outer section and are refused as unknown keys
 * (REQ-VER-025), so it cannot pass unnoticed.
 */
#ifndef MINI_YAML_H
#define MINI_YAML_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Parse a float list "[a, b, c, ...]" into out[0..n-1]. A block sequence
 * in the file reaches this in the same form, mini_yaml_parse folds it.
 * Returns 0 only when all n values converted, -1 otherwise, and on -1
 * leaves out[] untouched so the caller keeps whatever default it had
 * (REQ-VER-028). */
static int mini_yaml_list(const char* v, float* out, int n)
{
    float       tmp[16];
    int         i;
    const char* p = strchr(v, '[');
    if (!p || n < 1 || n > (int)(sizeof(tmp) / sizeof(tmp[0]))) return -1;
    p++;
    for (i = 0; i < n; ++i)
    {
        char*        endp = (char*)0;
        const double d    = strtod(p, &endp);
        if (endp == p) return -1; /* fewer values than the caller asked for */
        tmp[i] = (float)d;
        p      = endp;
        while (*p == ' ' || *p == ',') p++;
    }
    memcpy(out, tmp, (size_t)n * sizeof(float));
    return 0;
}

/* Hand a collected block sequence to set() as the inline list the rest of
 * this reader speaks. A key that opened no items at all is a nested
 * mapping, not a list, and is left alone (see the file comment). */
static int mini_yaml_flush_block(void* ctx,
                                 int (*set)(void*, const char*, const char*, const char*),
                                 char* sec, char* key, char* val, size_t len, size_t cap,
                                 int items)
{
    int rc = 0;
    if (items > 0 && len + 2 <= cap)
    {
        val[len]     = ']';
        val[len + 1] = '\0';
        rc           = set(ctx, sec, key, val);
    }
    key[0] = '\0';
    return rc;
}

/* Read `path` line by line and call set(ctx, sec, key, val) for every
 * "key: value" pair (sec == "" at top level, otherwise the most recently
 * seen "section:" header). set() returns 0 to continue or nonzero to
 * abort the parse; mini_yaml_parse then returns -1 without printing
 * anything itself: set() still has sec/key/val at the point of failure
 * and is the right place for a caller-specific diagnostic (or none at
 * all, if unknown keys are meant to be ignored).
 * Returns -1 if `path` cannot be opened, a list is too long for the line
 * buffer, or set() aborted, 0 otherwise. */
static int mini_yaml_parse(const char* path, void* ctx,
                            int (*set)(void* ctx, const char* sec, const char* key, const char* val))
{
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    char section[512] = "";
    /* An indented "key:" with no value opens a possible block sequence.
       Its "- item" lines accumulate in block_val as "[a, b, c" and are
       handed to set() closed on the first line that is not an item, so a
       config written in either YAML form reaches the harness identically
       (REQ-VER-028). */
    char   block_key[512] = "";
    char   block_sec[512] = "";
    char   block_val[512];
    size_t block_len = 0;
    int    block_items = 0;
    int    rc          = 0;

    while (rc == 0 && fgets(line, sizeof(line), f))
    {
        /* strip comments (the config files never quote '#') */
        char* hash = strchr(line, '#');
        if (hash != (char*)0) { *hash = '\0'; }

        const int indented = (line[0] == ' ' || line[0] == '\t');
        char*     p        = line;
        while (*p == ' ' || *p == '\t') p++;
        char* end = p + strlen(p);
        while (end > p && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' ')) *--end = '\0';

        /* Nothing left after the comment strip. Skipping the line rather
           than falling through keeps a blank or commented line between a
           key and its block items from closing the sequence early. */
        if (p[0] == '\0') continue;

        if (block_key[0] != '\0' && p[0] == '-' && (p[1] == ' ' || p[1] == '\0'))
        {
            const char* item = p + 1;
            while (*item == ' ') item++;
            const size_t sep  = (block_items > 0) ? 2u : 0u;
            const size_t need = block_len + sep + strlen(item);
            if (need + 2 >= sizeof(block_val)) { rc = -1; break; }
            if (sep) { memcpy(block_val + block_len, ", ", 2); }
            memcpy(block_val + block_len + sep, item, strlen(item));
            block_len = need;
            block_items++;
            continue;
        }
        if (block_key[0] != '\0')
        {
            rc = mini_yaml_flush_block(ctx, set, block_sec, block_key, block_val, block_len,
                                       sizeof(block_val), block_items);
            if (rc != 0) break;
        }

        char* colon = strchr(p, ':');
        if (!colon) continue;
        *colon    = '\0';
        char* key = p;
        char* val = colon + 1;
        while (*val == ' ') val++;
        /* trim trailing spaces of the key */
        end = key + strlen(key);
        while (end > key && end[-1] == ' ') *--end = '\0';
        if (key[0] == '\0') continue;

        if (!indented)
        {
            if (val[0] == '\0')
            {
                snprintf(section, sizeof(section), "%s", key); /* "gnss:" */
                continue;
            }
            section[0] = '\0'; /* top-level scalar */
        }
        if (val[0] == '\0')
        {
            snprintf(block_key, sizeof(block_key), "%s", key);
            snprintf(block_sec, sizeof(block_sec), "%s", indented ? section : "");
            block_val[0] = '[';
            block_len    = 1;
            block_items  = 0;
            continue;
        }
        rc = set(ctx, indented ? section : "", key, val);
    }
    if (rc == 0 && block_key[0] != '\0')
    {
        rc = mini_yaml_flush_block(ctx, set, block_sec, block_key, block_val, block_len,
                                   sizeof(block_val), block_items);
    }
    fclose(f);
    return (rc != 0) ? -1 : 0;
}

#endif /* MINI_YAML_H */
