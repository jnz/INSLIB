/* IMU dither for the synthetic test streams.
 *
 * A real IMU never repeats a sample bit for bit, and the stillness detectors
 * read a window of identical samples as a frozen sensor (REQ-NAV-013,
 * REQ-AHRS-017), so a noise-free synthetic stream would veto every
 * auto-ZUPT/ZARU. Including this header wraps the four entry points that take
 * IMU samples (ahrs_update, ahrs_predict_step, nav_suite_update,
 * nav_suite_predict_step) in macros that add a zero-mean dither far below
 * every test tolerance.
 *
 * The dither sign is a hash of the timestamp, not a call counter, so
 * ahrs_update() and ahrs_predict_step() fed the same epoch see the same
 * sample and the update == predict + correct equivalence scenarios stay
 * exact. A scenario that needs the raw stream (a frozen sensor on purpose)
 * calls the real function with its name in parentheses, for example
 * (ahrs_update)(...), which bypasses the function-like macro.
 *
 * Include after ahrs.h and nav_suite.h. */
#ifndef TEST_IMU_DITHER_H
#define TEST_IMU_DITHER_H

#include <stdbool.h>
#include <stdint.h>

#include "ahrs.h"
#include "nav_suite.h"

#define TEST_IMU_DITHER_GYR_RPS  (1e-6f)
#define TEST_IMU_DITHER_ACC_MPS2 (1e-5f)

static inline float test_dither_sign(int64_t t)
{
    const uint64_t h = (uint64_t)t * 0x9E3779B97F4A7C15ull;
    return (h >> 63) ? 1.0f : -1.0f;
}

static inline void test_dither3(const float in[3], float out[3], float amp, float s)
{
    out[0] = in[0] + s * amp;
    out[1] = in[1] - s * amp;
    out[2] = in[2] + s * amp;
}

static inline int test_ahrs_predict_step(ahrs_t* a, ahrs_time_us_t t, const float gyr[3],
                                         const float acc[3], const float mag_b[3], bool zaru,
                                         float* phi)
{
    const float s = test_dither_sign(t);
    float       g[3], f[3];
    if (gyr) { test_dither3(gyr, g, TEST_IMU_DITHER_GYR_RPS, s); }
    if (acc) { test_dither3(acc, f, TEST_IMU_DITHER_ACC_MPS2, s); }
    return ahrs_predict_step(a, t, gyr ? g : gyr, acc ? f : acc, mag_b, zaru, phi);
}

static inline void test_ahrs_update(ahrs_t* a, ahrs_time_us_t t, const float gyr[3],
                                    const float acc[3], const float mag_b[3], bool zaru)
{
    const float s = test_dither_sign(t);
    float       g[3], f[3];
    if (gyr) { test_dither3(gyr, g, TEST_IMU_DITHER_GYR_RPS, s); }
    if (acc) { test_dither3(acc, f, TEST_IMU_DITHER_ACC_MPS2, s); }
    ahrs_update(a, t, gyr ? g : gyr, acc ? f : acc, mag_b, zaru);
}

static inline void test_dither_meas(const ins_measurements_t* m, ins_measurements_t* out)
{
    *out          = *m;
    const float s = test_dither_sign(m->timestamp);
    test_dither3(m->gyr.data, out->gyr.data, TEST_IMU_DITHER_GYR_RPS, s);
    test_dither3(m->acc.data, out->acc.data, TEST_IMU_DITHER_ACC_MPS2, s);
}

static inline int test_nav_suite_predict_step(nav_suite_t* s, const ins_measurements_t* m,
                                              float* phi)
{
    ins_measurements_t md;
    test_dither_meas(m, &md);
    return nav_suite_predict_step(s, &md, phi);
}

static inline void test_nav_suite_update(nav_suite_t* s, const ins_measurements_t* m)
{
    ins_measurements_t md;
    test_dither_meas(m, &md);
    nav_suite_update(s, &md);
}

#define ahrs_predict_step(a, t, g, f, mg, z, p) test_ahrs_predict_step(a, t, g, f, mg, z, p)
#define ahrs_update(a, t, g, f, mg, z)          test_ahrs_update(a, t, g, f, mg, z)
#define nav_suite_predict_step(s, m, p)         test_nav_suite_predict_step(s, m, p)
#define nav_suite_update(s, m)                  test_nav_suite_update(s, m)

#endif /* TEST_IMU_DITHER_H */
