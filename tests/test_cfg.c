/** @file test_cfg.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the firmware configuration store, the temperature dependent
 * sensor calibration and the configuration protocol
 * (embedded/stm32f429/Core/Src/inslib_sensor/{cfg,cal,cfg_ubx}.c).
 *
 * These files are firmware, but the only platform dependent thing in
 * them is raw flash access, which cfg_port.h abstracts. This file
 * supplies a RAM backed flash instead, which is what makes the parts
 * that are otherwise untestable observable: slot rotation, a torn
 * write, a bit flip in a stored image.
 *
 * Scenarios:
 *   1. defaults:     unset keys read as their compiled in default,
 *                    unknown keys are refused, ranges are enforced.
 *   2. persistence:  an image survives a save and a reload, and a save
 *                    with no changes does not consume a slot.
 *   3. rotation:     saves append into free slots and roll over into
 *                    the other sector, erasing it, without ever losing
 *                    the image that was valid before.
 *   4. corruption:   a torn write and a flipped bit both leave the
 *                    previous image in charge.
 *   5. calibration:  interpolation between nodes, the bias polynomial,
 *                    clamping outside the calibrated range, and the
 *                    rejection of an incomplete table.
 *  5b. housing:      the mounting rotation applies with and without a
 *                    node table, does not vary with temperature, and a
 *                    record that is not a rotation is refused.
 *   6. protocol:     VALSET applies and persists, VALGET answers with
 *                    what is set, a wildcard expands, and a malformed
 *                    request is refused.
 *   7. uplink:       the sniffer consumes our frames and passes on
 *                    everything else byte for byte, including bytes
 *                    that only look like the start of one of ours.
 *   8. ubx framing:  the wire layout of every encoder, read back field
 *                    by field against an independently computed
 *                    checksum.
 *   9. ubx parser:   resynchronization after garbage, a broken
 *                    checksum, an implausible length field and a
 *                    payload larger than the parser buffer.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "inslib/cfg.h"
#include "inslib/cfg_port.h"
#include "inslib/cfg_ubx.h"
#include "inslib/cal.h"
#include "inslib/ubx.h"
#include "inslib/imu_saturation.h"
#include "inslib/nav_glue.h"

static int fails = 0;

#define CHECK_TRUE(cond, msg)                    \
    do {                                         \
        if (!(cond))                             \
        {                                        \
            printf("  FAIL  %-56s\n", msg);      \
            fails++;                             \
        }                                        \
        else { printf("  ok    %-56s\n", msg); } \
    } while (0)

/* ---------------------------------------------------------------------------
 * RAM backed flash. Erased state is 0xFF, and programming clears bits
 * only, exactly like the real part: a program over non erased bytes
 * therefore corrupts rather than replaces, which is the failure the
 * store's slot allocation has to avoid.
 * ---------------------------------------------------------------------------
 */

static uint8_t  g_flash[INSLIB_CFG_SECTORS * INSLIB_CFG_SECTOR_SZ];
static uint32_t g_erase_count;
static uint32_t g_program_bytes;
static uint32_t g_program_limit; /* 0 = unlimited, else stop after n bytes */

static void flash_wipe(void)
{
    memset(g_flash, 0xFF, sizeof(g_flash));
    g_erase_count   = 0;
    g_program_bytes = 0;
    g_program_limit = 0;
}

int inslib_cfg_port_read(uint32_t off, void* dst, uint32_t len)
{
    if (off > sizeof(g_flash) || len > sizeof(g_flash) - off) { return -1; }
    memcpy(dst, &g_flash[off], len);
    return 0;
}

int inslib_cfg_port_program(uint32_t off, const void* src, uint32_t len)
{
    const uint8_t* s = (const uint8_t*)src;
    if ((off & 3U) || (len & 3U)) { return -1; }
    if (off > sizeof(g_flash) || len > sizeof(g_flash) - off) { return -1; }
    for (uint32_t i = 0; i < len; i++)
    {
        if (g_program_limit && g_program_bytes >= g_program_limit)
        {
            return -1; /* simulated power loss part way through */
        }
        g_flash[off + i] &= s[i];
        g_program_bytes++;
    }
    return 0;
}

int inslib_cfg_port_erase(uint32_t sector)
{
    if (sector >= INSLIB_CFG_SECTORS) { return -1; }
    memset(&g_flash[sector * INSLIB_CFG_SECTOR_SZ], 0xFF, INSLIB_CFG_SECTOR_SZ);
    g_erase_count++;
    return 0;
}

#define SLOTS_PER_SECTOR (INSLIB_CFG_SECTOR_SZ / INSLIB_CFG_SLOT_SZ)

/* ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------
 */

static void put_u16(uint8_t* p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static void put_u32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_f32(uint8_t* p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32(p, u);
}

/* One calibration node: temperature, the 3x3 in column major order, and
 * the bias measured at that node. */
static void make_point(uint8_t* out, float t_c, const float* M, const float* b)
{
    static const float zero[3] = {0.0f, 0.0f, 0.0f};
    if (b == 0) { b = zero; }
    put_f32(out, t_c);
    for (int i = 0; i < 9; i++) { put_f32(out + 4 + 4 * i, M[i]); }
    for (int i = 0; i < 3; i++) { put_f32(out + 40 + 4 * i, b[i]); }
}

/* c is axis major, INSLIB_CAL_POLY_MAX coefficients per axis. */
static void make_poly(uint8_t* out, float t_ref, uint8_t deg, const float* c)
{
    memset(out, 0, INSLIB_CAL_BIASPOLY_LEN);
    put_f32(out, t_ref);
    out[4] = deg;
    for (int i = 0; i < 3 * (int)INSLIB_CAL_POLY_MAX; i++) { put_f32(out + 8 + 4 * i, c[i]); }
}

static void diag_matrix(float* M, float sx, float sy, float sz)
{
    memset(M, 0, 9 * sizeof(float));
    M[0] = sx;
    M[4] = sy;
    M[8] = sz;
}

/* Install a two node accelerometer calibration and activate it. bx is
 * the x axis bias stored at each node, which the firmware interpolates
 * as long as no polynomial has been fitted over them. */
static void install_two_node_acc(float t0, float s0, float t1, float s1, float bx0, float bx1)
{
    uint8_t pt[INSLIB_CAL_POINT_LEN];
    float   M[9];
    float   b0[3] = {bx0, 0.0f, 0.0f};
    float   b1[3] = {bx1, 0.0f, 0.0f};

    diag_matrix(M, s0, s0, s0);
    make_point(pt, t0, M, b0);
    inslib_cfg_set(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 0), pt, sizeof(pt));

    diag_matrix(M, s1, s1, s1);
    make_point(pt, t1, M, b1);
    inslib_cfg_set(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 1), pt, sizeof(pt));

    uint8_t n = 2;
    inslib_cfg_set(INSLIB_CFG_CAL_NPTS(INSLIB_CFG_GRP_CAL_ACC), &n, 1);
}

/* Drive the calibration far enough for its temperature filter to settle
 * on t_c, then return the corrected accelerometer reading of raw. */
static void settle_and_apply(float t_c, const float* raw, float* out)
{
    imu_sample_t s;
    memset(&s, 0, sizeof(s));

    /* The filter is a first order lag, so a few time constants of
     * simulated time bring it onto the input to well below the rebuild
     * threshold. */
    for (int i = 0; i < 200; i++)
    {
        /* Both operands uint64_t: a plain ULL literal would promote the
           multiplication to unsigned long long on a target where
           uint64_t is unsigned long, which -Wsign-conversion flags. */
        s.t_us = (uint64_t)(i + 1) * UINT64_C(100000); /* 10 Hz */
        s.temp = t_c;
        for (int k = 0; k < 3; k++) { s.accel[k] = raw[k]; }
        inslib_cal_apply_imu(&s);
    }
    for (int k = 0; k < 3; k++) { out[k] = s.accel[k]; }
}

/* ---------------------------------------------------------------------------
 * Scenario 1: defaults and validation
 * ---------------------------------------------------------------------------
 */

static void scenario_defaults(void)
{
    printf("\n-- scenario: defaults and validation --\n");

    flash_wipe();
    inslib_cfg_init();

    uint8_t  v1           = 0;
    uint16_t len          = 0;
    uint8_t  from_default = 0;

    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_MSGOUT_IMU, &v1, &len, &from_default) == INSLIB_CFG_OK,
               "a known key reads back on a virgin store");
    CHECK_TRUE(from_default == 1, "and reports that it came from the default");
    CHECK_TRUE(len == 1, "with the length the key declares");

    uint32_t bogus = 0x2FFF0001U; /* group and item that have no descriptor */
    CHECK_TRUE(inslib_cfg_get(bogus, &v1, &len, 0) == INSLIB_CFG_E_KEY,
               "an unknown key is refused on read");
    CHECK_TRUE(inslib_cfg_set(bogus, &v1, 1) == INSLIB_CFG_E_KEY,
               "an unknown key is refused on write");

    uint8_t two = 2;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_MSGOUT_IMU, &two, 1) == INSLIB_CFG_E_RANGE,
               "a value outside the key's range is refused");
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_MSGOUT_IMU, &two, 2) == INSLIB_CFG_E_LEN,
               "a value of the wrong length is refused");

    uint8_t point[INSLIB_CAL_POINT_LEN];
    memset(point, 0, sizeof(point));
    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 0), point, &len, 0) ==
                   INSLIB_CFG_E_UNSET,
               "a blob key with nothing stored reports unset, not unknown");

    /* Keys whose permitted values are a list and not an interval. A value
     * inside the bounds but off the list has to be refused: the sensor
     * has no such rate or bandwidth, and accepting it would leave the
     * device running at something the host never asked for. */
    uint16_t odr = 600;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_IMU_ODR_HZ, &odr, 2) == INSLIB_CFG_E_RANGE,
               "a rate inside the bounds but off the ladder is refused");
    odr = 400;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_IMU_ODR_HZ, &odr, 2) == INSLIB_CFG_OK,
               "a rate on the ladder is accepted");
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_IMU_ODR_HZ) == 400, "and reads back");

    uint16_t div = 24;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_IMU_LPF_DIV, &div, 2) == INSLIB_CFG_E_RANGE,
               "a low pass divider the part does not have is refused");
    div = 128;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_IMU_LPF_DIV, &div, 2) == INSLIB_CFG_OK,
               "the narrowest divider is accepted");

    uint16_t nav_ms = 5;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_RATE_NAV_MS, &nav_ms, 2) == INSLIB_CFG_OK,
               "the navigation period reaches 5 ms, i.e. 200 Hz");
    nav_ms = 4;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_RATE_NAV_MS, &nav_ms, 2) == INSLIB_CFG_E_RANGE,
               "and no further, a period below one that HAL ticks can count");

    uint8_t zero = 0;
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_MSGOUT_IMU, &zero, 1) == INSLIB_CFG_OK,
               "an in range value is accepted");
    CHECK_TRUE(inslib_cfg_u1(INSLIB_CFG_MSGOUT_IMU) == 0, "and reads back");
    CHECK_TRUE(inslib_cfg_dirty() == 1, "the image is marked unsaved after a change");
}

/* ---------------------------------------------------------------------------
 * Scenario 2: persistence
 * ---------------------------------------------------------------------------
 */

static void scenario_persistence(void)
{
    printf("\n-- scenario: persistence --\n");

    flash_wipe();
    inslib_cfg_init();

    uint16_t period = 250;
    uint8_t  off    = 0;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &period, 2);
    inslib_cfg_set(INSLIB_CFG_MSGOUT_BARO, &off, 1);

    uint8_t pt[INSLIB_CAL_POINT_LEN];
    float   M[9];
    diag_matrix(M, 1.01f, 0.99f, 1.0f);
    make_point(pt, 25.0f, M, 0);
    inslib_cfg_set(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 0), pt, sizeof(pt));

    uint32_t crc_before = inslib_cfg_image_crc();

    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "save succeeds");
    CHECK_TRUE(inslib_cfg_dirty() == 0, "and clears the unsaved flag");
    CHECK_TRUE(inslib_cfg_stored_seq() == 1, "the first stored image has sequence 1");

    uint32_t bytes_after_first = g_program_bytes;
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "saving again is accepted");
    CHECK_TRUE(g_program_bytes == bytes_after_first,
               "but writes nothing when the image has not changed");

    /* Reload from scratch, as a power cycle would. */
    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_image_crc() == crc_before, "the reloaded image is the stored one");
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 250, "a scalar survives the reload");
    CHECK_TRUE(inslib_cfg_u1(INSLIB_CFG_MSGOUT_BARO) == 0, "and so does a changed flag");

    uint8_t  back[INSLIB_CAL_POINT_LEN];
    uint16_t len = 0;
    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 0), back, &len, 0) ==
                   INSLIB_CFG_OK,
               "the calibration node survives the reload");
    CHECK_TRUE(memcmp(back, pt, sizeof(pt)) == 0, "byte for byte");
    CHECK_TRUE(inslib_cfg_u1(INSLIB_CFG_MSGOUT_IMU) == 1,
               "a key that was never set still reads as its default");
}

/* ---------------------------------------------------------------------------
 * Scenario 3: slot rotation
 * ---------------------------------------------------------------------------
 */

static void scenario_rotation(void)
{
    printf("\n-- scenario: slot rotation --\n");

    flash_wipe();
    inslib_cfg_init();

    /* The first save has to erase, because nothing usable is stored and
     * the store starts from a clean sector. */
    uint16_t v = 100;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &v, 2);
    inslib_cfg_save();
    CHECK_TRUE(g_erase_count == 1, "the first save erases one sector");

    /* Filling the rest of that sector must not erase anything. */
    for (uint32_t i = 1; i < SLOTS_PER_SECTOR; i++)
    {
        v = (uint16_t)(100 + i);
        inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &v, 2);
        CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "a further save succeeds");
    }
    CHECK_TRUE(g_erase_count == 1, "saves into free slots of the same sector do not erase");
    CHECK_TRUE(inslib_cfg_stored_seq() == SLOTS_PER_SECTOR,
               "the sequence number counts every stored image");

    /* One more has to roll over into the other sector. */
    v = 999;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &v, 2);
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "the save after a full sector succeeds");
    CHECK_TRUE(g_erase_count == 2, "and erases the other sector to do it");

    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 999,
               "the newest image wins after the rollover");

    /* And back again, which proves both sectors are in the rotation. */
    for (uint32_t i = 1; i < SLOTS_PER_SECTOR; i++)
    {
        v = (uint16_t)(1000 + i);
        inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &v, 2);
        inslib_cfg_save();
    }
    v = 1234;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &v, 2);
    inslib_cfg_save();
    CHECK_TRUE(g_erase_count == 3, "the next rollover erases the first sector again");

    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 1234, "and the newest image is intact");
}

/* ---------------------------------------------------------------------------
 * Scenario 4: corruption and torn writes
 * ---------------------------------------------------------------------------
 */

static void scenario_corruption(void)
{
    printf("\n-- scenario: corruption and torn writes --\n");

    flash_wipe();
    inslib_cfg_init();

    uint16_t good = 111;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &good, 2);
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "a first image is stored");

    /* Cut the power in the middle of the next save. */
    uint16_t doomed = 222;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &doomed, 2);
    g_program_limit = g_program_bytes + 16; /* dies inside the slot header */
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_E_FLASH, "an interrupted save reports failure");
    g_program_limit = 0;

    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 111,
               "the previous image is still the one that loads");

    /* The damaged slot counts as used, so the next save moves past it
     * rather than programming over bytes that are no longer erased. */
    uint16_t next = 333;
    inslib_cfg_set(INSLIB_CFG_RATE_MAG_MS, &next, 2);
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "the save after a torn one succeeds");
    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 333, "and is what loads afterwards");

    /* A single flipped bit in the newest payload must send the loader
     * back to the previous image rather than into a corrupt one. */
    /* Find the slot that is actually in charge. A sequence number alone
     * does not identify it: the torn write above carries the same one,
     * because it was cut off before it could fail. Slot header layout is
     * magic, version, header length, sequence, payload length, payload
     * checksum, header checksum. */
    uint32_t seq_before = inslib_cfg_stored_seq();
    for (uint32_t slot = 0; slot < INSLIB_CFG_SECTORS * SLOTS_PER_SECTOR; slot++)
    {
        uint8_t* p = &g_flash[slot * INSLIB_CFG_SLOT_SZ];
        if (get_u32(p + 8) != seq_before) { continue; }
        if (inslib_cfg_crc32(p + INSLIB_CFG_HDR_SZ, get_u32(p + 12)) != get_u32(p + 16))
        {
            continue; /* the torn slot, already unusable */
        }
        p[INSLIB_CFG_HDR_SZ + 9] ^= 0x08U; /* somewhere inside the records */
        break;
    }
    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 111,
               "a flipped bit rejects that image and the older one takes over");

    /* Nothing valid at all leaves the device on its defaults. */
    flash_wipe();
    memset(g_flash, 0x5A, 64); /* looks written, checksums like noise */
    inslib_cfg_init();
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 100,
               "a store with nothing valid in it falls back to the defaults");
    CHECK_TRUE(inslib_cfg_stored_seq() == 0, "and reports that nothing is stored");
}

/* ---------------------------------------------------------------------------
 * Scenario 5: calibration
 * ---------------------------------------------------------------------------
 */

static void scenario_calibration(void)
{
    printf("\n-- scenario: calibration over temperature --\n");

    flash_wipe();
    inslib_cfg_init();
    inslib_cal_refresh();

    const float raw[3] = {1.0f, 1.0f, 1.0f};
    float       out[3];

    settle_and_apply(25.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-6f,
               "without a calibration the sample passes through unchanged");

    /* Scale 1.0 at 0 C and 2.0 at 40 C, so 20 C has to give 1.5. */
    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    inslib_cal_refresh();

    /* The correction is rebuilt only once the filtered temperature has
     * moved by the threshold in cal.c, so the effective matrix trails
     * the true one by up to that much. The tolerances below allow for
     * it rather than pretending the rebuild is continuous. */
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.5f) < 5e-3f, "the misalignment interpolates between nodes");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) != 0,
               "and the calibration reports itself as valid");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_CLAMPED) == 0,
               "inside the range nothing is clamped");

    settle_and_apply(0.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 5e-3f, "at the lower node it matches that node");
    settle_and_apply(40.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 2.0f) < 5e-3f, "at the upper node likewise");

    /* Outside the table nothing is extrapolated. */
    settle_and_apply(80.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 2.0f) < 1e-6f, "above the range the last node is held");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_CLAMPED) != 0, "and reported as clamped");
    settle_and_apply(-40.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-6f, "below the range the first node is held");

    /* Until a polynomial has been fitted, the bias stored at each node
     * carries the correction and is interpolated like the matrix. This
     * is what makes a node by node upload useful before the table is
     * complete: at 20 C the scale is 1.5 and the bias 0.4, so a raw 1.0
     * becomes 1.5 * (1.0 - 0.4). */
    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.2f, 0.6f);
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 0.9f) < 5e-3f, "node biases are interpolated without a polynomial");
    CHECK_TRUE(fabsf(out[1] - 1.5f) < 5e-3f, "on the axes they were given for only");

    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    inslib_cal_refresh();

    /* Bias polynomial: b_x(T) = 0.1 + 0.01 * (T - 20). */
    uint8_t poly[INSLIB_CAL_BIASPOLY_LEN];
    float   c[3 * INSLIB_CAL_POLY_MAX];
    memset(c, 0, sizeof(c));
    c[0] = 0.1f;  /* x, constant term */
    c[1] = 0.01f; /* x, linear term   */
    make_poly(poly, 20.0f, 1, c);
    inslib_cfg_set(INSLIB_CFG_CAL_BIASPOLY(INSLIB_CFG_GRP_CAL_ACC), poly, sizeof(poly));
    inslib_cal_refresh();

    /* At 30 C the interpolated scale is 1.75 and the x bias is 0.2, so
     * a raw 1.0 becomes 1.75 * (1.0 - 0.2) = 1.4 on x and 1.75 on the
     * axes the polynomial left alone. */
    settle_and_apply(30.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.4f) < 5e-3f, "the bias polynomial is evaluated at temperature");
    CHECK_TRUE(fabsf(out[1] - 1.75f) < 5e-3f, "and only on the axis it was given for");

    /* A record can be replaced but never removed, so a fit made over an
     * older node set is retired with a degree marker rather than with
     * zero coefficients, which would mean a bias of zero. */
    make_poly(poly, 0.0f, 0xFF, c);
    inslib_cfg_set(INSLIB_CFG_CAL_BIASPOLY(INSLIB_CFG_GRP_CAL_ACC), poly, sizeof(poly));
    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.2f, 0.6f);
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 0.9f) < 5e-3f,
               "a retired polynomial hands the bias back to the nodes");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) != 0,
               "and does not invalidate the calibration");

    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    inslib_cal_refresh();

    /* A node count that promises more than is stored must not activate. */
    uint8_t n = 4;
    inslib_cfg_set(INSLIB_CFG_CAL_NPTS(INSLIB_CFG_GRP_CAL_ACC), &n, 1);
    inslib_cal_refresh();
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) == 0,
               "an incomplete node table is rejected");
    settle_and_apply(30.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-6f, "and the samples pass through uncorrected");

    /* Nodes have to rise in temperature, otherwise bracketing them is
     * meaningless. */
    n = 2;
    inslib_cfg_set(INSLIB_CFG_CAL_NPTS(INSLIB_CFG_GRP_CAL_ACC), &n, 1);
    uint8_t pt[INSLIB_CAL_POINT_LEN];
    float   M[9];
    diag_matrix(M, 1.0f, 1.0f, 1.0f);
    make_point(pt, -10.0f, M, 0); /* below node 0, so the order breaks */
    inslib_cfg_set(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 1), pt, sizeof(pt));
    inslib_cal_refresh();
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) == 0,
               "nodes out of temperature order are rejected");

    /* A NaN anywhere in the table disqualifies it. */
    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    inslib_cal_refresh();
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) != 0, "a sound table is valid again");
    diag_matrix(M, 1.0f, 1.0f, 1.0f);
    M[4] = NAN;
    make_point(pt, 40.0f, M, 0);
    inslib_cfg_set(INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 1), pt, sizeof(pt));
    inslib_cal_refresh();
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_ACC_VALID) == 0,
               "a non finite entry disqualifies the table");

    /* The correction can be switched off without removing it, which is
     * what a calibration recording needs. */
    install_two_node_acc(0.0f, 1.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    uint8_t apply = 0;
    inslib_cfg_set(INSLIB_CFG_IMU_APPLY_CAL, &apply, 1);
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-6f, "APPLY_CAL off leaves the sample raw");

    apply = 1;
    inslib_cfg_set(INSLIB_CFG_IMU_APPLY_CAL, &apply, 1);
    inslib_cal_refresh();
    /* The polynomial was retired further up and the nodes carry zero
     * bias, so this is the interpolated matrix on its own. */
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.5f) < 5e-3f, "and switching it back on restores the correction");
    CHECK_TRUE(fabsf(out[1] - 1.5f) < 5e-3f, "on every axis");
}

/* ---------------------------------------------------------------------------
 * Scenario 5b: the housing rotation
 * ---------------------------------------------------------------------------
 */

/* A yaw of deg, column major, which is what CFG-FRAME-HOUSING carries. */
static void make_housing_yaw(uint8_t* out, float deg)
{
    const float a  = deg * 3.14159265358979f / 180.0f;
    const float c  = cosf(a);
    const float sn = sinf(a);
    float       R[9];

    memset(R, 0, sizeof(R));
    R[0] = c;   /* (0,0) */
    R[1] = sn;  /* (1,0) */
    R[3] = -sn; /* (0,1) */
    R[4] = c;   /* (1,1) */
    R[8] = 1.0f;
    for (int i = 0; i < 9; i++) { put_f32(out + 4 * i, R[i]); }
}

static void scenario_housing(void)
{
    printf("\n-- scenario: housing rotation --\n");

    flash_wipe();
    inslib_cfg_init();
    inslib_cal_refresh();

    const float raw[3] = {1.0f, 0.0f, 0.0f};
    float       out[3];
    uint8_t     rec[INSLIB_CAL_HOUSING_LEN];

    settle_and_apply(25.0f, raw, out);
    CHECK_TRUE(fabsf(out[0] - 1.0f) < 1e-6f, "no housing record leaves the sample alone");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_HOUSING) == 0, "and reports none");

    /* A rotation applies on its own, without any node table. A unit
     * measured only in its box is still a unit with something to
     * correct. */
    make_housing_yaw(rec, 90.0f);
    inslib_cfg_set(INSLIB_CFG_FRAME_HOUSING, rec, sizeof(rec));
    inslib_cal_refresh();
    settle_and_apply(25.0f, raw, out);
    CHECK_TRUE(fabsf(out[0]) < 1e-5f && fabsf(out[1] - 1.0f) < 1e-5f,
               "a housing rotation applies with no node table at all");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_HOUSING) != 0, "and reports itself");

    /* On top of the table it is R * M, and the bias stays on the raw
     * side of both: scale 2 at every node, so x=1 becomes y=2. */
    install_two_node_acc(0.0f, 2.0f, 40.0f, 2.0f, 0.0f, 0.0f);
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[1] - 2.0f) < 5e-3f, "and premultiplies the node matrix");

    /* The point of keeping it out of the table: it is the SAME rotation
     * at both ends of the node range. Folded into the nodes it would be
     * interpolated, and an interpolated rotation is not one. */
    float lo[3], hi[3];
    settle_and_apply(0.0f, raw, lo);
    settle_and_apply(40.0f, raw, hi);
    CHECK_TRUE(fabsf(lo[1] - hi[1]) < 5e-3f && fabsf(lo[0] - hi[0]) < 5e-3f,
               "the rotation does not vary with temperature");

    /* A record that is not a rotation is refused rather than applied: it
     * would scale the very magnitude the calibration exists to get
     * right, and it would do it silently. */
    float M[9];
    diag_matrix(M, 1.5f, 1.0f, 1.0f);
    for (int i = 0; i < 9; i++) { put_f32(rec + 4 * i, M[i]); }
    inslib_cfg_set(INSLIB_CFG_FRAME_HOUSING, rec, sizeof(rec));
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_HOUSING_BAD) != 0,
               "a matrix that also scales is reported as no rotation");
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_HOUSING) == 0, "and is not applied");
    CHECK_TRUE(fabsf(out[0] - 2.0f) < 5e-3f, "the node table alone carries the correction");

    /* A reflection is orthonormal and still not a mounting: it would
     * mirror the frame rather than turn it. */
    diag_matrix(M, 1.0f, 1.0f, -1.0f);
    for (int i = 0; i < 9; i++) { put_f32(rec + 4 * i, M[i]); }
    inslib_cfg_set(INSLIB_CFG_FRAME_HOUSING, rec, sizeof(rec));
    inslib_cal_refresh();
    CHECK_TRUE((inslib_cal_flags() & INSLIB_CAL_F_HOUSING_BAD) != 0,
               "a reflection is refused as well");

    /* And it survives a save and a reload like any other record. */
    make_housing_yaw(rec, 90.0f);
    inslib_cfg_set(INSLIB_CFG_FRAME_HOUSING, rec, sizeof(rec));
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "the housing rotation persists");
    inslib_cfg_init();
    inslib_cal_refresh();
    settle_and_apply(20.0f, raw, out);
    CHECK_TRUE(fabsf(out[1] - 2.0f) < 5e-3f, "and comes back after a reload");
}

/* ---------------------------------------------------------------------------
 * Scenario 5c: the GNSS antenna lever arm
 * ---------------------------------------------------------------------------
 */

static void scenario_leverarm(void)
{
    printf("\n-- scenario: gnss antenna lever arm --\n");

    flash_wipe();
    inslib_cfg_init();

    uint8_t  rec[INSLIB_CFG_LEVERARM_LEN];
    uint8_t  back[INSLIB_CFG_LEVERARM_LEN];
    uint16_t len          = 0;
    uint8_t  from_default = 0;

    /* Absent, not zero-valued. The two fuse identically, but only one of
     * them is a decision somebody made, and the device has to be able to
     * say which it is holding. */
    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_FRAME_LEVERARM, back, &len, &from_default) ==
                   INSLIB_CFG_E_UNSET,
               "an unmeasured lever arm reads as unset, not as three zeros");

    /* Wrong length is refused: a record one float short would otherwise
     * be read with a garbage third axis. */
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_FRAME_LEVERARM, rec, 8U) == INSLIB_CFG_E_LEN,
               "a record of the wrong length is refused");

    put_f32(rec + 0, 0.20f);
    put_f32(rec + 4, 0.60f);
    put_f32(rec + 8, -0.50f);
    CHECK_TRUE(inslib_cfg_set(INSLIB_CFG_FRAME_LEVERARM, rec, sizeof(rec)) == INSLIB_CFG_OK,
               "a 12 byte record is accepted");

    len = 0;
    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_FRAME_LEVERARM, back, &len, &from_default) ==
                       INSLIB_CFG_OK &&
                   len == INSLIB_CFG_LEVERARM_LEN,
               "and reads back at its own length");
    CHECK_TRUE(memcmp(back, rec, sizeof(rec)) == 0,
               "byte for byte, so the axis order cannot drift");

    /* Like every other record, it has to survive a power cycle -- a lever
     * arm that is lost on reboot is worse than one that was never set,
     * because the first drive after it looks configured. */
    CHECK_TRUE(inslib_cfg_save() == INSLIB_CFG_OK, "the lever arm persists");
    inslib_cfg_init();
    len = 0;
    CHECK_TRUE(inslib_cfg_get(INSLIB_CFG_FRAME_LEVERARM, back, &len, &from_default) ==
                       INSLIB_CFG_OK &&
                   memcmp(back, rec, sizeof(rec)) == 0,
               "and comes back after a reload");

    /* It sits in the same group as the housing rotation but is a
     * separate item: setting one must not disturb the other. */
    CHECK_TRUE(INSLIB_CFG_KEY_GROUP(INSLIB_CFG_FRAME_LEVERARM) ==
                   INSLIB_CFG_KEY_GROUP(INSLIB_CFG_FRAME_HOUSING),
               "it lives in the frame group with the housing rotation");
    CHECK_TRUE(INSLIB_CFG_FRAME_LEVERARM != INSLIB_CFG_FRAME_HOUSING, "as its own item");
}

/* ---------------------------------------------------------------------------
 * Scenario 5d: measurement range saturation episodes
 * ---------------------------------------------------------------------------
 */

/* n clipped samples in a row on `mask`, starting at seq. */
static uint16_t sat_run(uint8_t mask, uint16_t seq, uint16_t n)
{
    for (uint16_t k = 0; k < n; k++) { inslib_sat_note(mask, (uint16_t)(seq + k)); }
    return (uint16_t)(seq + n);
}

static void scenario_saturation(void)
{
    printf("\n-- scenario: saturation episodes --\n");

    inslib_sat_episode_t ep;

    inslib_sat_reset();
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 0, "nothing clipped, nothing to report");
    CHECK_TRUE(inslib_sat_sticky() == 0 && inslib_sat_samples() == 0, "and no counters move");

    /* A run is reported when it ENDS, not while it is going: until then
     * its last sample is not known yet. */
    (void)sat_run(INSLIB_SAT_GYR_Z, 100, 3);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 0, "an open run is not reported yet");
    CHECK_TRUE(inslib_sat_sticky() == INSLIB_SAT_GYR_Z, "but the sticky mask is live");
    CHECK_TRUE(inslib_sat_samples() == 3, "and so is the sample count");

    inslib_sat_note(0U, 103); /* a clean sample closes it */
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1, "a clean sample closes the run");
    CHECK_TRUE(ep.first_seq == 100 && ep.last_seq == 102 && ep.samples == 3,
               "bracketed by the first and last clipped sample");
    CHECK_TRUE(ep.mask == INSLIB_SAT_GYR_Z && ep.flags == 0, "with the axis and no flags");
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 0, "and it is only reported once");

    /* The mask is ORed over the run: an episode that starts on one axis
     * and spreads to another is one episode naming both. */
    inslib_sat_reset();
    inslib_sat_note(INSLIB_SAT_ACC_X, 10);
    inslib_sat_note((uint8_t)(INSLIB_SAT_ACC_X | INSLIB_SAT_GYR_Y), 11);
    inslib_sat_note(0U, 12);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1 &&
                   ep.mask == (INSLIB_SAT_ACC_X | INSLIB_SAT_GYR_Y),
               "the axis mask accumulates over the run");

    /* A dropped sample leaves a gap in seq, which is why the count is
     * carried rather than derived from the bracket. */
    inslib_sat_reset();
    inslib_sat_note(INSLIB_SAT_ACC_Z, 500);
    inslib_sat_note(INSLIB_SAT_ACC_Z, 502); /* 501 never arrived */
    inslib_sat_note(0U, 503);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1 && ep.first_seq == 500 && ep.last_seq == 502 &&
                   ep.samples == 2,
               "a gap in seq makes the bracket wider than the run inside it");

    /* Clipping that never stops still has to report itself, or the one
     * failure that matters most would be the one that stays silent. */
    inslib_sat_reset();
    (void)sat_run(INSLIB_SAT_GYR_X, 0, (uint16_t)INSLIB_SAT_EP_MAX_SAMPLES);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1 && ep.samples == INSLIB_SAT_EP_MAX_SAMPLES,
               "a run that does not end is forced out at its length limit");
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 0, "and the next one starts from scratch");

    /* Queue overflow. The ring holds three, so the fourth episode is
     * lost -- and the NEXT one that fits has to say so, otherwise the
     * host reads an incomplete record as a complete one. */
    inslib_sat_reset();
    uint16_t seq = 0;
    for (int k = 0; k < 4; k++)
    {
        seq = sat_run(INSLIB_SAT_ACC_Y, seq, 2);
        inslib_sat_note(0U, seq++);
    }
    for (int k = 0; k < 3; k++)
    {
        CHECK_TRUE(inslib_sat_take_episode(&ep) == 1, "the queued episodes come back");
        CHECK_TRUE(ep.flags == 0, "none of them claims anything was lost");
    }
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 0, "the overflowing one is gone");

    seq = sat_run(INSLIB_SAT_ACC_Y, seq, 2);
    inslib_sat_note(0U, seq++);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1 && (ep.flags & INSLIB_SAT_EP_DROPPED) != 0,
               "and the next episode reports that one was lost before it");

    seq = sat_run(INSLIB_SAT_ACC_Y, seq, 2);
    inslib_sat_note(0U, seq++);
    CHECK_TRUE(inslib_sat_take_episode(&ep) == 1 && ep.flags == 0,
               "the flag is not sticky: it belongs to one gap, not to the stream");

    /* The counters are taken at detection, not at queueing, so they stay
     * the authority on whether it happened at all. */
    CHECK_TRUE(inslib_sat_samples() == 12 && inslib_sat_sticky() == INSLIB_SAT_ACC_Y,
               "the sticky counters count every clipped sample, queued or not");
}

/* ---------------------------------------------------------------------------
 * Scenario 6: the protocol
 * ---------------------------------------------------------------------------
 */

/* Bytes the sniffer decided were not ours, in order. */
static uint8_t  g_fwd[4096];
static uint16_t g_fwd_n;

static int fwd_capture(const uint8_t* buf, uint16_t len)
{
    if ((uint32_t)g_fwd_n + len <= sizeof(g_fwd))
    {
        memcpy(&g_fwd[g_fwd_n], buf, len);
        g_fwd_n = (uint16_t)(g_fwd_n + len);
    }
    return 0;
}

/* Wrap a payload into a UBX frame of our class and push it through the
 * sniffer, then service it. Returns the response length. */
static uint16_t exchange(uint8_t msg_id, const uint8_t* payload, uint16_t len, uint8_t* resp,
                         uint16_t resp_sz)
{
    uint8_t  frame[600];
    uint16_t n   = ubx_frame(frame, UBX_CLASS_INSLIB, msg_id, payload, len);
    uint8_t  src = 0xFF;

    g_fwd_n = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, frame, n, fwd_capture);
    return inslib_cfg_service(resp, resp_sz, &src);
}

static void scenario_protocol(void)
{
    printf("\n-- scenario: configuration protocol --\n");

    flash_wipe();
    inslib_cfg_init();
    inslib_cfg_ubx_init();

    uint8_t  resp[600];
    uint16_t n;

    /* VALSET of two scalars, RAM only. */
    uint8_t  set[32];
    uint16_t k = 0;
    set[k++]   = 0x01;
    set[k++]   = INSLIB_CFG_LAYER_RAM;
    set[k++]   = 0;
    set[k++]   = 0;
    put_u32(&set[k], INSLIB_CFG_RATE_MAG_MS);
    k += 4;
    put_u16(&set[k], 200);
    k += 2;
    put_u32(&set[k], INSLIB_CFG_MSGOUT_MAG);
    k += 4;
    set[k++] = 0;

    n = exchange(UBX_MSG_VALSET, set, k, resp, sizeof(resp));
    CHECK_TRUE(n > 0, "a VALSET is answered");
    CHECK_TRUE(resp[3] == UBX_MSG_CFGACK, "with an acknowledgement");
    CHECK_TRUE(resp[7] == INSLIB_CFG_OK, "reporting success");
    CHECK_TRUE(get_u32(&resp[14]) == 2, "and the number of values applied");
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 200, "the first value took effect");
    CHECK_TRUE(inslib_cfg_u1(INSLIB_CFG_MSGOUT_MAG) == 0, "and so did the second");
    CHECK_TRUE(inslib_cfg_stored_seq() == 0, "the RAM layer alone writes nothing to flash");

    /* A blob, persisted this time. */
    uint8_t pt[INSLIB_CAL_POINT_LEN];
    float   M[9];
    diag_matrix(M, 1.0f, 1.0f, 1.0f);
    make_point(pt, 25.0f, M, 0);

    uint8_t big[128];
    k        = 0;
    big[k++] = 0x01;
    big[k++] = INSLIB_CFG_LAYER_RAM | INSLIB_CFG_LAYER_FLASH;
    big[k++] = 0;
    big[k++] = 0;
    put_u32(&big[k], INSLIB_CFG_CAL_POINT(INSLIB_CFG_GRP_CAL_ACC, 0));
    k += 4;
    put_u16(&big[k], INSLIB_CAL_POINT_LEN);
    k += 2;
    memcpy(&big[k], pt, sizeof(pt));
    k = (uint16_t)(k + sizeof(pt));

    n = exchange(UBX_MSG_VALSET, big, k, resp, sizeof(resp));
    CHECK_TRUE(resp[7] == INSLIB_CFG_OK, "a blob value is accepted");
    CHECK_TRUE(inslib_cfg_stored_seq() == 1, "and the flash layer persists the image");

    /* VALGET of one key. */
    uint8_t get[8];
    get[0] = 0x00;
    get[1] = INSLIB_CFG_VALGET_RAM;
    get[2] = 0;
    get[3] = 0;
    put_u32(&get[4], INSLIB_CFG_RATE_MAG_MS);
    n = exchange(UBX_MSG_VALGET, get, 8, resp, sizeof(resp));
    CHECK_TRUE(resp[3] == UBX_MSG_VALGET_R, "a VALGET gets a value response");
    CHECK_TRUE(get_u32(&resp[10]) == INSLIB_CFG_RATE_MAG_MS, "carrying the key asked for");
    CHECK_TRUE(resp[14] == 200 && resp[15] == 0, "and its current value");

    /* The default layer answers with what a reset would give. */
    get[1] = INSLIB_CFG_VALGET_DEFAULT;
    n      = exchange(UBX_MSG_VALGET, get, 8, resp, sizeof(resp));
    CHECK_TRUE(resp[14] == 100 && resp[15] == 0, "the default layer reports the default");

    /* A wildcard expands to the group. */
    get[1] = INSLIB_CFG_VALGET_RAM;
    put_u32(&get[4],
            INSLIB_CFG_KEY(INSLIB_CFG_SZ_U1, INSLIB_CFG_GRP_MSGOUT, INSLIB_CFG_ITEM_WILDCARD));
    n = exchange(UBX_MSG_VALGET, get, 8, resp, sizeof(resp));
    /* Sized from what the descriptor table actually holds rather than a
     * literal number here, which would silently stop testing anything as
     * new message output keys get added. What the check is FOR is that
     * the wildcard expands at all and expands to the whole group. */
    {
        const uint16_t items = inslib_cfg_group_count(INSLIB_CFG_GRP_MSGOUT);

        CHECK_TRUE(items >= 5, "the message output group has its keys");
        CHECK_TRUE(n == 8 + 4 + items * (4 + 1), "a wildcard answers with every item of the group");
    }

    /* A malformed request is refused rather than guessed at. */
    get[1] = 0x03; /* no such layer */
    put_u32(&get[4], INSLIB_CFG_RATE_MAG_MS);
    n = exchange(UBX_MSG_VALGET, get, 8, resp, sizeof(resp));
    CHECK_TRUE(resp[3] == UBX_MSG_CFGACK && resp[7] == INSLIB_CFG_E_LAYER,
               "an unsupported layer is refused");

    k        = 0;
    set[k++] = 0x01;
    set[k++] = INSLIB_CFG_LAYER_RAM;
    set[k++] = 0;
    set[k++] = 0;
    put_u32(&set[k], INSLIB_CFG_RATE_MAG_MS);
    k += 4;
    set[k++] = 0x01; /* one byte short for a two byte key */
    n        = exchange(UBX_MSG_VALSET, set, k, resp, sizeof(resp));
    CHECK_TRUE(resp[3] == UBX_MSG_CFGACK && resp[7] == INSLIB_CFG_E_FRAME,
               "a truncated value is refused");

    /* Reset, both layers. */
    uint8_t rst[4] = {0x01, INSLIB_CFG_LAYER_RAM | INSLIB_CFG_LAYER_FLASH, 0, 0};
    n              = exchange(UBX_MSG_CFGRESET, rst, sizeof(rst), resp, sizeof(resp));
    CHECK_TRUE(resp[7] == INSLIB_CFG_OK, "a reset is accepted");
    CHECK_TRUE(inslib_cfg_u2(INSLIB_CFG_RATE_MAG_MS) == 100, "and puts the defaults back");
    CHECK_TRUE(inslib_cfg_stored_seq() == 0, "and empties the store");
}

/* ---------------------------------------------------------------------------
 * Scenario 7: the uplink sniffer
 * ---------------------------------------------------------------------------
 */

static void scenario_uplink(void)
{
    printf("\n-- scenario: uplink passthrough --\n");

    flash_wipe();
    inslib_cfg_init();
    inslib_cfg_ubx_init();

    /* An RTCM3 frame, which is what actually shares this link. */
    const uint8_t rtcm[] = {0xD3, 0x00, 0x08, 0x3E, 0xD0, 0x00, 0x03,
                            0x8A, 0xA1, 0x00, 0x00, 0x00, 0xB1, 0x9C};
    g_fwd_n              = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, rtcm, (uint16_t)sizeof(rtcm), fwd_capture);
    CHECK_TRUE(g_fwd_n == (uint16_t)sizeof(rtcm) && memcmp(g_fwd, rtcm, sizeof(rtcm)) == 0,
               "an RTCM frame passes through byte for byte");

    /* A UBX frame for the receiver, class 0x06, must not be touched. */
    uint8_t  cfg_f9p[64];
    uint8_t  payload[8] = {0x01, 0x01, 0, 0, 0x01, 0x00, 0x52, 0x40};
    uint16_t n          = ubx_frame(cfg_f9p, 0x06, 0x8A, payload, sizeof(payload));
    g_fwd_n             = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, cfg_f9p, n, fwd_capture);
    CHECK_TRUE(g_fwd_n == n && memcmp(g_fwd, cfg_f9p, n) == 0,
               "a UBX frame for the receiver passes through unchanged");

    /* Bytes that look like the start of one of ours but are not: the
     * checksum decides, and until then nothing may be lost. */
    uint8_t fake[16];
    n = ubx_frame(fake, UBX_CLASS_INSLIB, UBX_MSG_VALGET, payload, 4);
    fake[n - 1] ^= 0xFFU; /* break the checksum */
    g_fwd_n = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, fake, n, fwd_capture);
    CHECK_TRUE(g_fwd_n == n && memcmp(g_fwd, fake, n) == 0,
               "a frame of our class with a bad checksum is passed on, not eaten");

    /* A stray sync byte in front of a real frame must not swallow it. */
    uint8_t stray[80];
    stray[0]   = 0xB5;
    uint16_t m = ubx_frame(&stray[1], 0x06, 0x01, payload, 4);
    g_fwd_n    = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, stray, (uint16_t)(1 + m), fwd_capture);
    CHECK_TRUE(g_fwd_n == 1 + m && memcmp(g_fwd, stray, 1 + m) == 0,
               "a stray sync byte before a frame does not lose either");

    /* One of ours is consumed and nothing of it reaches the receiver. */
    uint8_t mine[64];
    uint8_t get[8];
    get[0] = 0x00;
    get[1] = INSLIB_CFG_VALGET_RAM;
    get[2] = 0;
    get[3] = 0;
    put_u32(&get[4], INSLIB_CFG_RATE_MAG_MS);
    n       = ubx_frame(mine, UBX_CLASS_INSLIB, UBX_MSG_VALGET, get, sizeof(get));
    g_fwd_n = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, mine, n, fwd_capture);
    CHECK_TRUE(g_fwd_n == 0, "one of our frames is consumed");

    uint8_t  resp[600];
    uint8_t  src = 0xFF;
    uint16_t rn  = inslib_cfg_service(resp, sizeof(resp), &src);
    CHECK_TRUE(rn > 0 && src == INSLIB_CFG_SRC_VCP, "and is answered on the port it arrived on");

    /* Split across two calls, as a DMA ring wrap would deliver it. */
    g_fwd_n = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, mine, 3, fwd_capture);
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, &mine[3], (uint16_t)(n - 3), fwd_capture);
    CHECK_TRUE(g_fwd_n == 0, "a frame split across two feeds is still consumed");
    rn = inslib_cfg_service(resp, sizeof(resp), &src);
    CHECK_TRUE(rn > 0, "and still answered");

    /* The two ports do not share parser state. */
    g_fwd_n = 0;
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, mine, 4, fwd_capture);
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_CDC, rtcm, (uint16_t)sizeof(rtcm), fwd_capture);
    CHECK_TRUE(g_fwd_n == (uint16_t)sizeof(rtcm),
               "traffic on one port does not disturb a frame arriving on the other");
    inslib_cfg_uplink_feed(INSLIB_CFG_SRC_VCP, &mine[4], (uint16_t)(n - 4), fwd_capture);
    rn = inslib_cfg_service(resp, sizeof(resp), &src);
    CHECK_TRUE(rn > 0 && src == INSLIB_CFG_SRC_VCP, "and the interleaved frame completes");
}

/* ---------------------------------------------------------------------------
 * Scenario 8: UBX framing and sample encoders (ubx.c)
 *
 * The encoders are what the host actually decodes, so the payload
 * layout is part of the wire contract: a reordered field would still
 * produce a checksum valid frame and only show up as garbage on the
 * host. Each encoder is therefore read back field by field, not just
 * checked for its length.
 * ---------------------------------------------------------------------------
 */

/* Fletcher-8 over class..end of payload, recomputed here rather than
 * reused from ubx.c so a broken checksum in the encoder cannot validate
 * itself. */
static void ubx_test_checksum(const uint8_t* frame, uint16_t len, uint8_t* ckA, uint8_t* ckB)
{
    uint8_t  a = 0, b = 0;
    uint16_t i;
    for (i = 2; i < (uint16_t)(len - 2); ++i)
    {
        a = (uint8_t)(a + frame[i]);
        b = (uint8_t)(b + a);
    }
    *ckA = a;
    *ckB = b;
}

static bool ubx_frame_ok(const uint8_t* frame, uint16_t len, uint8_t cls, uint8_t id,
                         uint16_t payload_len)
{
    uint8_t ckA, ckB;
    if (len != (uint16_t)(payload_len + 8)) { return false; }
    if (frame[0] != 0xB5 || frame[1] != 0x62) { return false; }
    if (frame[2] != cls || frame[3] != id) { return false; }
    if (frame[4] != (uint8_t)(payload_len & 0xFFU) || frame[5] != (uint8_t)(payload_len >> 8))
    {
        return false;
    }
    ubx_test_checksum(frame, len, &ckA, &ckB);
    return frame[len - 2] == ckA && frame[len - 1] == ckB;
}

/* Encoded floats are compared as bit patterns, not as floats: the
 * encoder must reproduce the sample bit for bit, and an exact float
 * comparison would trip -Wfloat-equal for a check that is not about
 * numeric tolerance at all. */
static bool f32_at(const uint8_t* p, float expect)
{
    uint32_t u;
    memcpy(&u, &expect, sizeof(u));
    return get_u32(p) == u;
}

static uint64_t get_u64(const uint8_t* p)
{
    return (uint64_t)get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
}

static void scenario_ubx_framing(void)
{
    printf("\n-- scenario: UBX framing and encoders --\n");

    uint8_t  frame[128];
    uint16_t n;

    /* Empty payload: the shortest legal frame is header + checksum. */
    n = ubx_frame(frame, UBX_CLASS_INSLIB, UBX_MSG_STATUS, (const uint8_t*)0, 0);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_STATUS, 0),
               "a zero length payload yields an 8 byte frame");

    /* A length that does not fit in one byte must land little endian in
     * both length bytes. */
    static uint8_t big[300];
    static uint8_t big_frame[320];
    uint16_t       i;
    for (i = 0; i < (uint16_t)sizeof(big); ++i) { big[i] = (uint8_t)i; }
    n = ubx_frame(big_frame, UBX_CLASS_INSLIB, UBX_MSG_IMU, big, (uint16_t)sizeof(big));
    CHECK_TRUE(ubx_frame_ok(big_frame, n, UBX_CLASS_INSLIB, UBX_MSG_IMU, (uint16_t)sizeof(big)),
               "a payload above 255 bytes gets a little endian length");
    CHECK_TRUE(memcmp(&big_frame[6], big, sizeof(big)) == 0, "and the payload is copied verbatim");

    /* IMU sample: 8 B time, 3+3 floats, temperature, sequence. */
    imu_sample_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.t_us        = 0x0102030405060708ULL;
    imu.accel[0]    = 0.25f;
    imu.accel[1]    = -1.5f;
    imu.accel[2]    = 9.75f;
    imu.gyro[0]     = -0.125f;
    imu.gyro[1]     = 2.5f;
    imu.gyro[2]     = 0.0625f;
    imu.temp        = 31.5f;
    imu.seq         = 0xBEEF;
    imu.sat         = INSLIB_SAT_GYR_Y;
    imu.cal_applied = 1U;
    n               = ubx_encode_imu(frame, &imu);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_IMU, 38),
               "IMU frame is well formed");
    CHECK_TRUE(get_u64(&frame[6]) == imu.t_us, "IMU timestamp survives the encoding");
    CHECK_TRUE(f32_at(&frame[14], imu.accel[0]) && f32_at(&frame[18], imu.accel[1]) &&
                   f32_at(&frame[22], imu.accel[2]),
               "IMU accel triple in order");
    CHECK_TRUE(f32_at(&frame[26], imu.gyro[0]) && f32_at(&frame[30], imu.gyro[1]) &&
                   f32_at(&frame[34], imu.gyro[2]),
               "IMU gyro triple in order");
    /* The status word, which replaced the f32 temperature. Checked field
     * by field rather than against a magic constant: three flag bits sit
     * directly above an eleven bit signed number, and an off-by-one in
     * the temperature would flip a flag instead of failing loudly. */
    {
        const uint32_t st = get_u32(&frame[38]);
        int32_t        t  = (int32_t)(st & UBX_IMU_ST_TEMP_MASK);

        if (t & 0x400) { t -= 0x800; } /* sign extend 11 bits */
        CHECK_TRUE(t == 315, "the die temperature survives as tenths of a degree");
        CHECK_TRUE((st & UBX_IMU_ST_SAT_GYR) != 0, "a clipped gyro axis sets the gyro bit");
        CHECK_TRUE((st & UBX_IMU_ST_SAT_ACC) == 0, "and not the accelerometer's");
        CHECK_TRUE((st & UBX_IMU_ST_CAL_APPLIED) != 0, "the calibration flag travels along");
        CHECK_TRUE((st >> 14) == 0, "and the reserved bits go out as zero");
    }
    CHECK_TRUE(((uint16_t)frame[42] | ((uint16_t)frame[43] << 8)) == imu.seq,
               "IMU sequence number is byte identical as u16");

    /* Negative temperatures are the case the sign extension exists for,
     * and the clamp is what keeps an absurd reading from coming back as
     * a plausible one of the opposite sign. */
    imu.temp        = -12.3f;
    imu.sat         = 0;
    imu.cal_applied = 0U;
    (void)ubx_encode_imu(frame, &imu);
    {
        int32_t t = (int32_t)(get_u32(&frame[38]) & UBX_IMU_ST_TEMP_MASK);

        if (t & 0x400) { t -= 0x800; }
        CHECK_TRUE(t == -123, "a negative temperature survives as two's complement");
        CHECK_TRUE(get_u32(&frame[38]) == (uint32_t)(-123 & UBX_IMU_ST_TEMP_MASK),
                   "with no flag bits set alongside it");
    }
    imu.temp = 500.0f;
    (void)ubx_encode_imu(frame, &imu);
    {
        int32_t t = (int32_t)(get_u32(&frame[38]) & UBX_IMU_ST_TEMP_MASK);

        if (t & 0x400) { t -= 0x800; }
        CHECK_TRUE(t == 1023, "a temperature past the field clamps instead of wrapping");
    }

    /* Baro and mag. */
    baro_sample_t baro;
    memset(&baro, 0, sizeof(baro));
    baro.t_us        = 42;
    baro.pressure_pa = 96500.0f;
    baro.temp_c      = -7.25f;
    n                = ubx_encode_baro(frame, &baro);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_BARO, 16),
               "baro frame is well formed");
    CHECK_TRUE(get_u64(&frame[6]) == baro.t_us && f32_at(&frame[14], baro.pressure_pa) &&
                   f32_at(&frame[18], baro.temp_c),
               "baro fields survive the encoding");

    mag_sample_t mag;
    memset(&mag, 0, sizeof(mag));
    mag.t_us   = 7;
    mag.mag[0] = 20.5f;
    mag.mag[1] = -3.25f;
    mag.mag[2] = 44.0f;
    mag.temp_c = 22.0f;
    n          = ubx_encode_mag(frame, &mag);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_MAG, 24),
               "mag frame is well formed");
    CHECK_TRUE(f32_at(&frame[14], mag.mag[0]) && f32_at(&frame[18], mag.mag[1]) &&
                   f32_at(&frame[22], mag.mag[2]) && f32_at(&frame[26], mag.temp_c),
               "mag fields survive the encoding");

    /* Time sync: the one message with a signed field, so the negative
     * quantization error is the interesting case. */
    inslib_timesync_t ts;
    memset(&ts, 0, sizeof(ts));
    ts.t_us       = 0x00000000DEADBEEFULL;
    ts.count      = 5;
    ts.tow_ms     = 123456;
    ts.tow_sub_ms = 0x80000000u;
    ts.q_err_ps   = -1234;
    ts.week       = 2300;
    ts.flags      = INSLIB_TS_GPS_VALID | INSLIB_TS_QERR_VALID;
    n             = ubx_encode_timesync(frame, &ts);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_TIMESYNC, 28),
               "timesync frame is well formed");
    CHECK_TRUE(get_u64(&frame[6]) == ts.t_us && get_u32(&frame[14]) == ts.count &&
                   get_u32(&frame[18]) == ts.tow_ms && get_u32(&frame[22]) == ts.tow_sub_ms,
               "timesync time fields survive the encoding");
    CHECK_TRUE((int32_t)get_u32(&frame[26]) == ts.q_err_ps,
               "a negative quantization error survives as two's complement");
    CHECK_TRUE(((uint16_t)frame[30] | ((uint16_t)frame[31] << 8)) == ts.week &&
                   ((uint16_t)frame[32] | ((uint16_t)frame[33] << 8)) == ts.flags,
               "timesync week and flags survive the encoding");

    /* IMU health. The one message whose length is not a constant, and
     * the one that had no test: the encoder clamps at its payload buffer
     * and drops the overflow at the FAR END of the array, which is where
     * a newly added field sits. It shipped holding 16 while app.c passed
     * 17, so reg_read_rc never reached the host, and nothing noticed
     * because the frame's own length agreed with its own content. */
    {
        uint32_t hf[UBX_IMUHEALTH_MAX_FIELDS];
        bool     ordered = true;

        for (i = 0; i < (int)UBX_IMUHEALTH_MAX_FIELDS; ++i) { hf[i] = 0xA0000000U + (uint32_t)i; }

        n = ubx_encode_imuhealth(frame, hf, 19);
        CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_IMUHEALTH, 19 * 4),
                   "the health frame is as long as the field count says");
        for (i = 0; i < 19; ++i)
        {
            if (get_u32(&frame[6 + 4 * i]) != hf[i]) { ordered = false; }
        }
        CHECK_TRUE(ordered, "and every field of it arrives, in order");

        /* The buffer has to hold what the header promises, or the promise
         * is what a caller asserts against and the assert is worthless. */
        ordered = true;
        n       = ubx_encode_imuhealth(frame, hf, (uint8_t)UBX_IMUHEALTH_MAX_FIELDS);
        CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_IMUHEALTH,
                                (uint16_t)(UBX_IMUHEALTH_MAX_FIELDS * 4U)),
                   "a full payload of UBX_IMUHEALTH_MAX_FIELDS is not truncated");
        for (i = 0; i < (int)UBX_IMUHEALTH_MAX_FIELDS; ++i)
        {
            if (get_u32(&frame[6 + 4 * i]) != hf[i]) { ordered = false; }
        }
        CHECK_TRUE(ordered, "and arrives whole");
    }

    /* Saturation episode. Fixed length, and the field ORDER is the whole
     * content of the message: three u16 of the same width in a row
     * transpose without changing anything a length check could see. */
    n = ubx_encode_satur(frame, 0x1234, 0x5678, 0x009A, 0x2AU, 0x01U);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_SATUR, UBX_SATUR_LEN),
               "the saturation frame is well formed");
    CHECK_TRUE(((uint16_t)frame[6] | ((uint16_t)frame[7] << 8)) == 0x1234U &&
                   ((uint16_t)frame[8] | ((uint16_t)frame[9] << 8)) == 0x5678U &&
                   ((uint16_t)frame[10] | ((uint16_t)frame[11] << 8)) == 0x009AU,
               "first_seq, last_seq and samples keep their order");
    CHECK_TRUE(frame[12] == 0x2AU && frame[13] == 0x01U, "the axis mask and the flags follow them");

    /* The navigation solution. Its point is that a value and the bit
     * saying whether to believe it travel together, so that is what is
     * checked: a filter with no position must not be able to report one
     * by accident. */
    {
        inslib_nav_status_t ns;
        uint32_t            fl;

        memset(&ns, 0, sizeof(ns));
        ns.nav_ready       = 1U;
        ns.rpy_valid       = 1U;
        ns.roll_deg        = 1.5f;
        ns.pitch_deg       = -2.5f;
        ns.yaw_deg         = 213.0f;
        ns.height_valid    = 1U;
        ns.height_m        = -0.25f;
        ns.vzupt_active    = 1U;
        ns.leverarm_frd[0] = 0.2f;
        ns.update_wcet_us  = 752U;
        /* Deliberately left invalid: velocity, position, lat/lon, the
         * magnetic reference and the lever arm's "came from the store". */

        n = ubx_encode_nav(frame, &ns, 0x07U, 0x00U, 30U, 803U);
        CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_NAV, UBX_NAV_LEN),
                   "the navigation frame is well formed");
        CHECK_TRUE(frame[6] == 0x01 && frame[7] == 0 && frame[8] == 0,
                   "version, mode and attitude source lead it");
        CHECK_TRUE(frame[9] == 0x07U && frame[10] == 0x00U && frame[11] == 30U,
                   "then the calibration, saturation and load bytes");
        CHECK_TRUE(((uint16_t)frame[12] | ((uint16_t)frame[13] << 8)) == 803U,
                   "and the measured IMU rate");

        fl = get_u32(&frame[14]);
        CHECK_TRUE((fl & UBX_NAV_F_READY) && (fl & UBX_NAV_F_RPY),
                   "the flags carry what IS available");
        CHECK_TRUE(!(fl & UBX_NAV_F_VEL) && !(fl & UBX_NAV_F_POS) && !(fl & UBX_NAV_F_LATLON) &&
                       !(fl & UBX_NAV_F_MAG_REF),
                   "and leave clear what is not");
        CHECK_TRUE((fl & UBX_NAV_F_VZUPT) && !(fl & UBX_NAV_F_ZUPT),
                   "a vertical zero update is not a horizontal one");
        CHECK_TRUE(!(fl & UBX_NAV_F_LEVERARM), "a lever arm of all zeroes is not a configured one");
        CHECK_TRUE((fl >> 18) == 0, "the reserved flag bits go out as zero");
        CHECK_TRUE(get_u32(&frame[18]) == 752U, "the fusion worst case survives");
        CHECK_TRUE(f32_at(&frame[22], 1.5f) && f32_at(&frame[30], 213.0f),
                   "roll leads the attitude and yaw closes it");
    }

    /* Status counters, in the declared order. */
    n = ubx_encode_status(frame, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_INSLIB, UBX_MSG_STATUS, 40),
               "status frame is well formed");
    {
        bool ordered = true;
        for (i = 0; i < 10; ++i)
        {
            if (get_u32(&frame[6 + 4 * i]) != (uint32_t)(i + 1)) { ordered = false; }
        }
        CHECK_TRUE(ordered, "status counters keep their declared order");
    }

    /* CFG-VALSET for the receiver: version, layer bitfield, reserved,
     * then the key/value pairs. */
    n = ubx_cfg_valset_u1(frame, 0x10930006u, 1, UBX_VALSET_RAM);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_CFG, UBX_CFG_VALSET, 9),
               "VALSET U1 frame is well formed");
    CHECK_TRUE(frame[6] == 0x01 && frame[7] == UBX_VALSET_RAM && frame[8] == 0 && frame[9] == 0,
               "VALSET header carries version and layers");
    CHECK_TRUE(get_u32(&frame[10]) == 0x10930006u && frame[14] == 1, "VALSET U1 key and value");

    n = ubx_cfg_valset_u2(frame, 0x30210001u, 100, UBX_VALSET_ALL);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_CFG, UBX_CFG_VALSET, 10),
               "VALSET U2 frame is well formed");
    CHECK_TRUE(get_u32(&frame[10]) == 0x30210001u &&
                   ((uint16_t)frame[14] | ((uint16_t)frame[15] << 8)) == 100,
               "VALSET U2 key and little endian value");

    n = ubx_cfg_valset_u4(frame, 0x40520001u, 460800u, UBX_VALSET_ALL);
    CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_CFG, UBX_CFG_VALSET, 12),
               "VALSET U4 frame is well formed");
    CHECK_TRUE(get_u32(&frame[10]) == 0x40520001u && get_u32(&frame[14]) == 460800u,
               "VALSET U4 key and value");

    /* Batched U1: one atomic frame, and the hard cap at 16 keys, which is
     * what keeps the fixed payload buffer from overflowing. */
    {
        uint32_t keys[20];
        uint8_t  vals[20];
        for (i = 0; i < 20; ++i)
        {
            keys[i] = 0x10110000u + i;
            vals[i] = (uint8_t)(i + 1);
        }
        n = ubx_cfg_valset_u1_multi(frame, keys, vals, 3, UBX_VALSET_RAM);
        CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_CFG, UBX_CFG_VALSET, (uint16_t)(4 + 3 * 5)),
                   "batched VALSET holds three key/value pairs in one frame");
        CHECK_TRUE(get_u32(&frame[10]) == keys[0] && frame[14] == vals[0] &&
                       get_u32(&frame[15]) == keys[1] && frame[19] == vals[1] &&
                       get_u32(&frame[20]) == keys[2] && frame[24] == vals[2],
                   "batched VALSET pairs stay in order");

        n = ubx_cfg_valset_u1_multi(frame, keys, vals, 20, UBX_VALSET_RAM);
        CHECK_TRUE(ubx_frame_ok(frame, n, UBX_CLASS_CFG, UBX_CFG_VALSET, (uint16_t)(4 + 16 * 5)),
                   "more than 16 keys are capped instead of overflowing the buffer");
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 9: incremental UBX parser (ubx.c)
 *
 * The parser sits in front of every byte arriving from the receiver, so
 * its resynchronization behaviour decides whether a corrupt byte costs
 * one frame or the whole stream.
 * ---------------------------------------------------------------------------
 */

/* Feed a whole buffer, return the result of the last byte and count how
 * many complete frames and how many errors the run produced. */
static int ubx_feed(ubx_parser_t* p, const uint8_t* buf, uint16_t len, int* frames, int* errors)
{
    int      last = 0;
    uint16_t i;
    for (i = 0; i < len; ++i)
    {
        last = ubx_parse_byte(p, buf[i]);
        if (last == 1 && frames) { (*frames)++; }
        if (last == -1 && errors) { (*errors)++; }
    }
    return last;
}

static void scenario_ubx_parser(void)
{
    printf("\n-- scenario: incremental UBX parser --\n");

    ubx_parser_t   p;
    uint8_t        frame[64];
    uint8_t        payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint16_t n =
        ubx_frame(frame, UBX_CLASS_INSLIB, UBX_MSG_IMU, payload, (uint16_t)sizeof(payload));
    int frames, errors;

    ubx_parser_reset(&p);
    CHECK_TRUE(ubx_parser_idle(&p) == 1, "a reset parser reports idle");

    frames = errors = 0;
    ubx_feed(&p, frame, n, &frames, &errors);
    CHECK_TRUE(frames == 1 && errors == 0, "a clean frame is accepted exactly once");
    CHECK_TRUE(p.cls == UBX_CLASS_INSLIB && p.id == UBX_MSG_IMU && p.len == sizeof(payload),
               "class, id and length are reported");
    CHECK_TRUE(p.truncated == 0 && memcmp(p.buf, payload, sizeof(payload)) == 0,
               "a small payload is buffered untruncated");
    CHECK_TRUE(ubx_parser_idle(&p) == 1, "and the parser is idle again afterwards");

    /* Leading garbage, including a stray sync byte right before the real
     * frame: staying in S_SYNC2 on a second 0xB5 is what saves this. */
    {
        uint8_t  stream[80];
        uint16_t k  = 0;
        stream[k++] = 0x00;
        stream[k++] = 0xFF;
        stream[k++] = 0xB5;
        memcpy(&stream[k], frame, n);
        k = (uint16_t)(k + n);
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, stream, k, &frames, &errors);
        CHECK_TRUE(frames == 1 && errors == 0,
                   "a stray sync byte before a frame does not swallow it");
    }

    /* Wrong second sync byte: back to hunting, no error reported (nothing
     * was ever claimed to be a frame). */
    ubx_parser_reset(&p);
    CHECK_TRUE(ubx_parse_byte(&p, 0xB5) == 0 && ubx_parse_byte(&p, 0x11) == 0,
               "a wrong second sync byte is silently discarded");
    CHECK_TRUE(ubx_parser_idle(&p) == 1, "and the parser returns to hunting for sync");

    /* Broken checksum: -1 once, then the parser is usable again. */
    {
        uint8_t bad[64];
        memcpy(bad, frame, n);
        bad[n - 1] = (uint8_t)(bad[n - 1] ^ 0xFFU);
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, bad, n, &frames, &errors);
        CHECK_TRUE(frames == 0 && errors == 1, "a broken checksum is reported once");
        frames = errors = 0;
        ubx_feed(&p, frame, n, &frames, &errors);
        CHECK_TRUE(frames == 1 && errors == 0, "and the next clean frame is accepted");
    }

    /* Implausible length field: rejected at the length, not after
     * swallowing 64 kB (REQ-independent robustness, see ubx.h). */
    {
        uint8_t hdr[6] = {0xB5, 0x62, UBX_CLASS_INSLIB, UBX_MSG_IMU, 0xFF, 0xFF};
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, hdr, (uint16_t)sizeof(hdr), &frames, &errors);
        CHECK_TRUE(errors == 1 && ubx_parser_idle(&p) == 1,
                   "an implausible length field resyncs immediately");
    }

    /* Zero length payload: legal, goes straight to the checksum. */
    {
        const uint16_t z = ubx_frame(frame, UBX_CLASS_INSLIB, UBX_MSG_STATUS, (const uint8_t*)0, 0);
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, frame, z, &frames, &errors);
        CHECK_TRUE(frames == 1 && p.len == 0, "an empty payload is a valid frame");
    }

    /* A payload larger than the parser's buffer is still validated, but
     * flagged as truncated instead of overrunning the buffer. */
    {
        static uint8_t big[UBX_PARSE_BUF + 40];
        static uint8_t big_frame[UBX_PARSE_BUF + 64];
        uint16_t       i;
        for (i = 0; i < (uint16_t)sizeof(big); ++i) { big[i] = (uint8_t)(i * 7u); }
        const uint16_t bn =
            ubx_frame(big_frame, UBX_CLASS_INSLIB, UBX_MSG_IMU, big, (uint16_t)sizeof(big));
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, big_frame, bn, &frames, &errors);
        CHECK_TRUE(frames == 1 && errors == 0, "an oversized payload still passes the checksum");
        CHECK_TRUE(p.truncated == 1 && p.len == (uint16_t)sizeof(big),
                   "and is flagged truncated with the real length kept");
        CHECK_TRUE(memcmp(p.buf, big, UBX_PARSE_BUF) == 0,
                   "the buffered prefix is intact up to the buffer size");
    }

    /* An out of range state (a struct that was never reset) must resync
     * instead of running off the switch. */
    memset(&p, 0xAA, sizeof(p));
    CHECK_TRUE(ubx_parse_byte(&p, 0x00) == 0 && ubx_parser_idle(&p) == 1,
               "an unknown parser state resyncs on the next byte");

    /* Two frames back to back, no gap. */
    {
        uint8_t        stream[128];
        const uint16_t a = ubx_frame(stream, UBX_CLASS_INSLIB, UBX_MSG_BARO, payload, 4);
        const uint16_t b = ubx_frame(&stream[a], UBX_CLASS_INSLIB, UBX_MSG_MAG, payload, 8);
        ubx_parser_reset(&p);
        frames = errors = 0;
        ubx_feed(&p, stream, (uint16_t)(a + b), &frames, &errors);
        CHECK_TRUE(frames == 2 && errors == 0 && p.id == UBX_MSG_MAG,
                   "two frames back to back are both accepted");
    }
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    scenario_defaults();
    scenario_persistence();
    scenario_rotation();
    scenario_corruption();
    scenario_calibration();
    scenario_housing();
    scenario_leverarm();
    scenario_saturation();
    scenario_protocol();
    scenario_uplink();
    scenario_ubx_framing();
    scenario_ubx_parser();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
