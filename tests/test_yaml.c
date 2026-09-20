/** @file test_yaml.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the YAML subset reader shared by tools/replay.c and
 * tools/insrcv.c (tools/mini_yaml.h).
 *
 * A dataset directory is read by this reader and by python/replay.py
 * (PyYAML) alike, so what the two disagree about is not caught anywhere:
 * both harnesses run, each on its own idea of the configuration. That is
 * the failure REQ-VER-028 is about and what these scenarios pin down.
 *
 * Scenarios:
 *   1. block_sequence: a list written as a YAML block sequence arrives
 *                      as the inline list the converters emit, with the
 *                      section it was written under, and does not
 *                      disturb the keys around it.
 *   2. malformed_list: a list that is too short, or is not numbers at
 *                      all, is refused and leaves the destination at
 *                      whatever it held.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mini_yaml.h"

static int fails = 0;

#define CHECK_TRUE(cond, msg)                    \
    do {                                         \
        if (!(cond))                             \
        {                                        \
            printf("  FAIL  %-52s\n", msg);      \
            fails++;                             \
        }                                        \
        else { printf("  ok    %-52s\n", msg); } \
    } while (0)

/* ---------------------------------------------------------------------------
 * Recording callback: keeps every (section, key, value) mini_yaml_parse
 * hands out, so a scenario can assert on what the reader produced rather
 * than on what some consumer made of it.
 * ------------------------------------------------------------------------- */

#define MAX_PAIRS 32

typedef struct
{
    char sec[MAX_PAIRS][64];
    char key[MAX_PAIRS][64];
    char val[MAX_PAIRS][128];
    int  n;
} rec_t;

static int rec_set(void* ctx, const char* sec, const char* key, const char* val)
{
    rec_t* r = (rec_t*)ctx;
    if (r->n >= MAX_PAIRS) return -1;
    snprintf(r->sec[r->n], sizeof(r->sec[0]), "%s", sec);
    snprintf(r->key[r->n], sizeof(r->key[0]), "%s", key);
    snprintf(r->val[r->n], sizeof(r->val[0]), "%s", val);
    r->n++;
    return 0;
}

/* Value recorded for "sec.key", or NULL if the reader never emitted it. */
static const char* rec_get(const rec_t* r, const char* sec, const char* key)
{
    int i;
    for (i = 0; i < r->n; ++i)
    {
        if (!strcmp(r->sec[i], sec) && !strcmp(r->key[i], key)) return r->val[i];
    }
    return (const char*)0;
}

static const char* TMP_PATH = "test_yaml_tmp.yaml";

/* Write `text` to TMP_PATH and read it back through mini_yaml_parse. */
static int parse_text(const char* text, rec_t* out)
{
    FILE* f = fopen(TMP_PATH, "w");
    int   rc;
    memset(out, 0, sizeof(*out));
    if (!f)
    {
        printf("  FAIL  cannot create %s\n", TMP_PATH);
        fails++;
        return -1;
    }
    fputs(text, f);
    fclose(f);
    rc = mini_yaml_parse(TMP_PATH, out, rec_set);
    remove(TMP_PATH);
    return rc;
}

/* ---------------------------------------------------------------------------
 * 1. block sequences read like the inline form
 * ------------------------------------------------------------------------- */

static void scenario_block_sequence(void)
{
    rec_t       rec;
    const char* v;
    float       lever[3]        = {9.0f, 9.0f, 9.0f};
    float       inline_lever[3] = {9.0f, 9.0f, 9.0f};

    printf("\n-- block_sequence\n");

    CHECK_TRUE(parse_text("gnss:\n"
                          "  leverarm_frd:\n"
                          "  - -0.01\n"
                          "  - 0.04\n"
                          "  - -0.015\n"
                          "  delay_ms: 200\n"
                          "imu:\n"
                          "  gyr_psd: 5.0e-07\n",
                          &rec) == 0,
               "a config with a block sequence parses");

    v = rec_get(&rec, "gnss", "leverarm_frd");
    CHECK_TRUE(v != (const char*)0, "the block sequence reaches the caller at all");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "[-0.01, 0.04, -0.015]"),
               "it arrives as the inline list form");
    CHECK_TRUE(v != (const char*)0 && mini_yaml_list(v, lever, 3) == 0,
               "and converts through mini_yaml_list");
    CHECK_TRUE(fabsf(lever[0] + 0.01f) < 1e-6f && fabsf(lever[1] - 0.04f) < 1e-6f &&
                   fabsf(lever[2] + 0.015f) < 1e-6f,
               "with the values the file states");

    /* The keys around it are unaffected: the one following the sequence in
       the same section, and the section header after it. */
    v = rec_get(&rec, "gnss", "delay_ms");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "200"),
               "the key after the sequence still arrives");
    v = rec_get(&rec, "imu", "gyr_psd");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "5.0e-07"),
               "the next section is still opened correctly");

    /* Same file written inline: both forms have to mean the same thing,
       which is the whole point (REQ-VER-028). */
    CHECK_TRUE(parse_text("gnss:\n"
                          "  leverarm_frd: [-0.01, 0.04, -0.015]\n",
                          &rec) == 0,
               "the inline form parses");
    v = rec_get(&rec, "gnss", "leverarm_frd");
    CHECK_TRUE(v != (const char*)0 && mini_yaml_list(v, inline_lever, 3) == 0,
               "the inline form converts");
    CHECK_TRUE(fabsf(inline_lever[0] - lever[0]) < 1e-9f &&
                   fabsf(inline_lever[1] - lever[1]) < 1e-9f &&
                   fabsf(inline_lever[2] - lever[2]) < 1e-9f,
               "both YAML forms produce identical values");

    /* A sequence that ends the file has no following line to flush it. */
    CHECK_TRUE(parse_text("imu:\n"
                          "  acc_fixed_bias:\n"
                          "  - 1\n"
                          "  - 2\n"
                          "  - 3\n",
                          &rec) == 0,
               "a sequence at end of file parses");
    v = rec_get(&rec, "imu", "acc_fixed_bias");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "[1, 2, 3]"),
               "and is flushed rather than dropped");

    /* Comments and trailing spaces are stripped from items like anywhere
       else in the file. */
    CHECK_TRUE(parse_text("gnss:\n"
                          "  pos_stddev_fallback_m:\n"
                          "  - 0.8   # horizontal\n"
                          "  - 1.5   # vertical\n",
                          &rec) == 0,
               "a commented sequence parses");
    v = rec_get(&rec, "gnss", "pos_stddev_fallback_m");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "[0.8, 1.5]"), "comments are stripped from items");

    /* A blank or comment-only line between the key and its items must not
       close the sequence: the items after it belong to the same key. */
    CHECK_TRUE(parse_text("gnss:\n"
                          "  leverarm_frd:\n"
                          "  # antenna vs. IMU\n"
                          "\n"
                          "  - 0.2\n"
                          "  - 0.6\n"
                          "  - -0.5\n",
                          &rec) == 0,
               "a sequence broken by a blank/comment line parses");
    v = rec_get(&rec, "gnss", "leverarm_frd");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "[0.2, 0.6, -0.5]"), "and keeps every item of it");
}

/* ---------------------------------------------------------------------------
 * 2. a list that cannot be read in full is refused
 * ------------------------------------------------------------------------- */

static void scenario_malformed_list(void)
{
    rec_t       rec;
    const char* v;
    float       out[3];

    printf("\n-- malformed_list\n");

    /* Too short: filling the rest with zeros would hand the harness a
       lever arm nobody wrote down. */
    out[0] = 7.0f;
    out[1] = 8.0f;
    out[2] = 9.0f;
    CHECK_TRUE(mini_yaml_list("[1, 2]", out, 3) != 0, "a list shorter than asked for is refused");
    CHECK_TRUE(fabsf(out[0] - 7.0f) < 1e-9f && fabsf(out[1] - 8.0f) < 1e-9f &&
                   fabsf(out[2] - 9.0f) < 1e-9f,
               "and leaves the destination untouched");

    out[0] = 7.0f;
    CHECK_TRUE(mini_yaml_list("[1, oops, 3]", out, 3) != 0, "a non-numeric entry is refused");
    CHECK_TRUE(fabsf(out[0] - 7.0f) < 1e-9f, "and leaves the destination untouched");

    CHECK_TRUE(mini_yaml_list("0.5", out, 3) != 0, "a scalar where a list belongs is refused");
    CHECK_TRUE(mini_yaml_list("[1, 2, 3]", out, 0) != 0, "a zero-length request is refused");
    CHECK_TRUE(mini_yaml_list("[1, 2, 3]", out, 99) != 0,
               "a request beyond the scratch size is refused");

    CHECK_TRUE(mini_yaml_list("[1, 2, 3]", out, 3) == 0, "a complete list is accepted");
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-9f && fabsf(out[2] - 3.0f) < 1e-9f,
               "with the values it states");

    /* Same through the reader: a short block sequence is refused at the
       point the consumer converts it, not silently zero-filled. */
    out[0] = 7.0f;
    CHECK_TRUE(parse_text("gnss:\n"
                          "  leverarm_frd:\n"
                          "  - 0.1\n"
                          "  - 0.2\n",
                          &rec) == 0,
               "a short block sequence still parses as a value");
    v = rec_get(&rec, "gnss", "leverarm_frd");
    CHECK_TRUE(v != (const char*)0 && mini_yaml_list(v, out, 3) != 0,
               "but a 3-element consumer refuses it");
    CHECK_TRUE(fabsf(out[0] - 7.0f) < 1e-9f, "leaving its destination untouched");

    /* A key with nothing under it at all is a nested mapping or an empty
       value, not a list: it stays out of the callback, and its inner keys
       arrive under the outer section where REQ-VER-025 refuses them. */
    CHECK_TRUE(parse_text("gnss:\n"
                          "  leverarm_frd:\n"
                          "  delay_ms: 200\n",
                          &rec) == 0,
               "an empty key parses");
    CHECK_TRUE(rec_get(&rec, "gnss", "leverarm_frd") == (const char*)0,
               "an empty key produces no value");
    v = rec_get(&rec, "gnss", "delay_ms");
    CHECK_TRUE(v != (const char*)0 && !strcmp(v, "200"), "and does not swallow the next key");
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    scenario_block_sequence();
    scenario_malformed_list();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
