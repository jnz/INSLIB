/** @file mini_yaml.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Minimal YAML-subset reader shared by tools/replay.c and tools/insrcv.c
 * for the machine-generated config.yaml files: two levels ("section:" +
 * 2-space-indented "key: value"), scalars, inline float lists
 * "[a, b, c]" and '#' comments. Anything fancier (anchors, block lists,
 * quoting) is deliberately unsupported - the converters only emit this
 * subset (REQ-VER-025's shared schema).
 */
#ifndef MINI_YAML_H
#define MINI_YAML_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Parse an inline float list "[a, b, c, ...]" into out[0..n-1].
 * Returns -1 if no '[' is found, 0 otherwise. */
static int mini_yaml_list(const char* v, float* out, int n)
{
    int         i;
    const char* p = strchr(v, '[');
    if (!p) return -1;
    p++;
    for (i = 0; i < n; ++i)
    {
        out[i] = (float)strtod(p, (char**)&p);
        while (*p == ' ' || *p == ',') p++;
    }
    return 0;
}

/* Read `path` line by line and call set(ctx, sec, key, val) for every
 * "key: value" pair (sec == "" at top level, otherwise the most recently
 * seen "section:" header). set() returns 0 to continue or nonzero to
 * abort the parse; mini_yaml_parse then returns -1 without printing
 * anything itself -- set() still has sec/key/val at the point of failure
 * and is the right place for a caller-specific diagnostic (or none at
 * all, if unknown keys are meant to be ignored).
 * Returns -1 if `path` cannot be opened or set() aborted, 0 otherwise. */
static int mini_yaml_parse(const char* path, void* ctx,
                            int (*set)(void* ctx, const char* sec, const char* key, const char* val))
{
    FILE* f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    char section[512] = "";
    while (fgets(line, sizeof(line), f))
    {
        /* strip comments (the generated file never quotes '#') */
        char* hash = strchr(line, '#');
        if (hash != (char*)0) { *hash = '\0'; }

        const int indented = (line[0] == ' ' || line[0] == '\t');
        char*     p        = line;
        while (*p == ' ' || *p == '\t') p++;
        char* colon = strchr(p, ':');
        if (!colon) continue;
        *colon    = '\0';
        char* key = p;
        char* val = colon + 1;
        while (*val == ' ') val++;
        char* end = val + strlen(val);
        while (end > val && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' ')) *--end = '\0';
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
        if (val[0] == '\0') continue;
        if (set(ctx, indented ? section : "", key, val) != 0)
        {
            fclose(f);
            return -1;
        }
    }
    fclose(f);
    return 0;
}

#endif /* MINI_YAML_H */
