/** @file test_baro.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the baro/accelerometer vertical channel filter
 * (src/baro_alt.c) and its nav_suite integration.
 *
 * Scenarios:
 *   1. ISA:          pressure-to-altitude conversion round-trips the
 *                    inverse ISA formula.
 *   2. Convergence:  vertical sinusoid (tilted body, biased + noisy
 *                    accel, noisy baro); h/v/a_b must converge.
 *   3. Deadreckon:   accelerometer is a control input. With the baro
 *                    switched off, the filter keeps tracking the
 *                    trajectory on accel alone.
 *   4. Outlier:      a baro spike is downweighted (no jump in h); a
 *                    persistent baro offset is followed eventually
 *                    (downweight, not skip: no deadlock).
 *   5. NaN inputs:   non-finite acc/quaternion drop the epoch,
 *                    non-finite/implausible pressure only drops the
 *                    baro sample; init rejects a bad anchor pressure.
 *   6. Time anomaly: backwards step skips the epoch (state untouched),
 *                    a > 0.2 s gap skips the propagation while baro
 *                    fusion continues.
 *   7. nav_suite:    wrapper bootstraps the filter on the first baro
 *                    sample (h = 0 anchor) and tracks a slow climb.
 *   8. h_init:       datum-aligned init: the anchor sample maps to a
 *                    caller-given height instead of 0.
 *   9.-12. offset:   baro-GNSS offset filter: init/convergence, slow
 *                    random walk (weather drift + outage growth),
 *                    outlier downweighting, NaN/invalid inputs.
 *   13. low-rate:    offset filter's low-rate defaults: pairs faster
 *                    than min_update_interval_sec are decimated
 *                    (counted, not fused), and the default stddev
 *                    inflation factor derates the combined variance.
 *   14. chi2_disable: global outlier-rejection override: an outlier
 *                    that would normally be chi2-downweighted is fused
 *                    at face value instead (both baro_alt and the
 *                    offset filter).
 *   15. anchor:      nav_suite anchors the baro filter at the ins
 *                    NED height (ins first, baro later).
 *   16. strategy:    auto-init height strategy end-to-end: baro
 *                    first, origin shift at the first fix, offset
 *                    estimation, continuous heights through an outage.
 *   17. guard rails: NULL/uninitialized instances refuse to operate.
 *   18. corrupted:   corrupted covariance factors fail safe and recover
 *                    on re-init.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "baro_alt.h"
#include "nav_suite.h"
#include "geodetic_toolbox.h"
#include "sensor_defaults.h"
#include "linalg.h"

#define GRAVITY    INS_GRAVITY_NOMINAL
#define US_PER_SEC (1000000LL)

static int fails = 0;

#define CHECK_NEAR(a, b, tol, msg)                                                                \
    do {                                                                                          \
        __typeof__(a) _av = (a);                                                                  \
        __typeof__(b) _bv = (b);                                                                  \
        double        _d  = fabs((double)_av - (double)_bv);                                      \
        if (_d > (tol) || !__builtin_isfinite(_d))                                                \
        {                                                                                         \
            printf("  FAIL  %-40s: %g vs %g  (diff %g, tol %g)\n", msg, (double)_av, (double)_bv, \
                   _d, (double)(tol));                                                            \
            fails++;                                                                              \
        }                                                                                         \
        else                                                                                      \
        {                                                                                         \
            printf("  ok    %-40s: (%g vs %g, diff %g)\n", msg, (double)_av, (double)_bv, _d);    \
        }                                                                                         \
    } while (0)

#define CHECK_TRUE(cond, msg)                    \
    do {                                         \
        if (!(cond))                             \
        {                                        \
            printf("  FAIL  %-40s\n", msg);      \
            fails++;                             \
        }                                        \
        else { printf("  ok    %-40s\n", msg); } \
    } while (0)

/* ---------------------------------------------------------------------------
 * Deterministic gaussian noise (LCG + Box-Muller)
 * ---------------------------------------------------------------------------
 */

static uint32_t g_rng = 4242u;

static float frand01(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return (float)(g_rng >> 8) * (1.0f / 16777216.0f);
}

static float gauss(float stddev)
{
    float u1 = frand01();
    float u2 = frand01();
    if (u1 < 1e-7f) u1 = 1e-7f;
    return stddev * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/* ---------------------------------------------------------------------------
 * Inverse ISA: pressure for a given altitude above the 101325 Pa level
 * ---------------------------------------------------------------------------
 */

static float pressure_from_altitude(float h_m)
{
    return 101325.0f * powf(1.0f - h_m / 44330.0f, 5.255f);
}

/* Body-frame specific force of a body with attitude q (b-to-n) whose
 * only n-frame acceleration is hdd (up): f_n = (0, 0, -(g + hdd)),
 * f_b = R' * f_n. An n-frame down error err_n_z is added before the
 * rotation (models an accelerometer error on the vertical axis). */
static void body_specific_force(const float q[4], float hdd_up, float err_n_down,
                                float noise_stddev, float f_b[3])
{
    float R[9];
    int   i;
    ins_quat_to_rotmat(q, R);
    const float f_n_z = -(GRAVITY + hdd_up) + err_n_down;
    for (i = 0; i < 3; ++i) { f_b[i] = MAT_ELEM(R, 2, i, 3, 3) * f_n_z + gauss(noise_stddev); }
}

/* ---------------------------------------------------------------------------
 * Scenario 1: ISA pressure-to-altitude conversion
 * ---------------------------------------------------------------------------
 */

static void scenario_isa_conversion(void)
{
    printf("\n-- scenario: ISA pressure-to-altitude --\n");

    CHECK_NEAR(baro_alt_pressure_to_altitude(101325.0f), 0.0, 1e-3, "p0 maps to 0 m");
    CHECK_NEAR(baro_alt_pressure_to_altitude(pressure_from_altitude(500.0f)), 500.0, 0.5,
               "round-trip 500 m");
    CHECK_NEAR(baro_alt_pressure_to_altitude(pressure_from_altitude(3000.0f)), 3000.0, 1.0,
               "round-trip 3000 m");
    CHECK_TRUE(baro_alt_pressure_to_altitude(pressure_from_altitude(-100.0f)) < -99.0f,
               "below-sea-level pressure maps below 0");

    /* The filtered ISA altitude accessor (REQ-BARO-019) reports the
       barometric height system: local height plus the datum zero
       point, independent of where the caller put its datum. */
    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    float              h_isa;
    memset(&b, 0, sizeof(b));
    CHECK_TRUE(!baro_alt_get_isa_altitude(&b, &h_isa), "uninitialized: no ISA altitude");

    /* Anchor 25 m above the caller's datum: the local height starts at
       25, but the ISA altitude starts at the anchor's own 300 m. */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 25.0f, 2.0f) == 0,
               "init with h_init");
    CHECK_TRUE(baro_alt_get_isa_altitude(&b, &h_isa), "ISA altitude published");
    CHECK_NEAR(h_isa, base_alt, 0.01, "ISA altitude is datum-independent");

    /* A 3 m climb must show up in the ISA altitude the same way it shows
       up in the local height. */
    int i;
    for (i = 1; i <= 1500; ++i)
    {
        t += US_PER_SEC / 100;
        const float dh = (i < 1000) ? 3.0f * (float)i / 1000.0f : 3.0f;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt + dh), 0.0f, (i % 5) == 0);
    }
    float h_local;
    CHECK_TRUE(baro_alt_get_isa_altitude(&b, &h_isa), "healthy");
    CHECK_TRUE(baro_alt_get_height(&b, &h_local), "healthy");
    CHECK_NEAR(h_isa, base_alt + 3.0, 0.5, "ISA altitude follows the climb");
    CHECK_NEAR(h_isa - h_local, base_alt - 25.0, 0.01, "ISA minus local = datum zero point");
}

/* ---------------------------------------------------------------------------
 * Scenario 2: convergence on a vertical sinusoid
 *
 * Tilted static attitude, 100 Hz accel with a constant n-frame down
 * error of -0.2 m/s^2 and gaussian noise, 20 Hz baro with 0.5 m noise.
 * Truth: h(t) = A*sin(w*t). All three states must converge.
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_convergence(void)
{
    printf("\n-- scenario: convergence on vertical sinusoid --\n");

    const float dt       = 0.01f;
    const float A        = 5.0f;
    const float w        = 2.0f * (float)M_PI * 0.2f;
    const float base_alt = 300.0f; /* ISA altitude at start */
    const float err_n    = -0.2f;  /* accel error, n-frame down */
    const float acc_no   = 0.05f;
    const float baro_no  = 0.5f;

    float q[4];
    ins_quat_from_rpy(DEG2RAD(30.0f), DEG2RAD(-10.0f), DEG2RAD(45.0f), q);

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* The library defaults (BARO_ALT_DEFAULT_H_STDDEV_M/V_STDDEV_MPS/
       AB_STDDEV_MPS2/H_NOISE_M_SQRTHZ) are a tuning parameter: tight for a
       good sensor with a typically-small, already-near-converged bias,
       trading off against slower convergence on a genuinely large one. This
       scenario deliberately injects a large -0.2 m/s^2 bias and checks
       convergence within a fixed time, so it pins the original, wider
       starting conditions explicitly rather than riding along with
       whatever the defaults are currently tuned to. */
    cfg.h_init_stddev_m           = 1.0f;
    cfg.v_init_stddev_mps         = 0.5f;
    cfg.acc_bias_init_stddev_mps2 = 0.1f;
    cfg.h_process_noise_m_sqrthz  = 0.02f;

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    /* Anchor at the start altitude (h_true(0) = 0). */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    int       i;
    const int n_epochs = 6000; /* 60 s */
    for (i = 1; i <= n_epochs; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts     = (float)i * dt;
        const float h_true = A * sinf(w * ts);
        const float hdd    = -A * w * w * sinf(w * ts);

        float f_b[3];
        body_specific_force(q, hdd, err_n, acc_no, f_b);

        const bool  baro_valid = (i % 5) == 0; /* 20 Hz */
        const float p          = pressure_from_altitude(base_alt + h_true + gauss(baro_no));
        baro_alt_update(&b, t, f_b, q, p, 0.0f, baro_valid);
    }

    /* t = 60 s is a full number of periods: h = 0, v = A*w. */
    float h, v, ab;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "get_height");
    CHECK_TRUE(baro_alt_get_velocity(&b, &v), "get_velocity");
    CHECK_TRUE(baro_alt_get_acc_bias(&b, &ab), "get_acc_bias");
    CHECK_NEAR(h, 0.0, 0.5, "height tracks truth");
    CHECK_NEAR(v, A * w, 0.4, "velocity (hidden state) tracks truth");
    CHECK_NEAR(ab, err_n, 0.1, "acc correction converges to error");
    CHECK_TRUE(b.epoch == (uint32_t)n_epochs, "all epochs processed");
    CHECK_TRUE(b.n_invalid_input == 0, "no inputs dropped");
}

/* ---------------------------------------------------------------------------
 * Scenario 3: dead-reckoning on the accelerometer control input
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_deadreckon(void)
{
    printf("\n-- scenario: accel dead-reckoning without baro --\n");

    const float dt       = 0.01f;
    const float A        = 5.0f;
    const float w        = 2.0f * (float)M_PI * 0.2f;
    const float base_alt = 300.0f;
    const float err_n    = -0.2f;

    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* See scenario_baro_convergence: this scenario's injected -0.2 m/s^2
       bias needs the original, wider starting conditions to converge within
       phase 1's 30 s, independent of what the defaults are tuned to. */
    cfg.h_init_stddev_m           = 1.0f;
    cfg.v_init_stddev_mps         = 0.5f;
    cfg.acc_bias_init_stddev_mps2 = 0.1f;
    cfg.h_process_noise_m_sqrthz  = 0.02f;

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    /* Phase 1: 30 s with baro (converge, including the accel error). */
    int i;
    for (i = 1; i <= 3000; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts     = (float)i * dt;
        const float h_true = A * sinf(w * ts);
        const float hdd    = -A * w * w * sinf(w * ts);
        float       f_b[3];
        body_specific_force(q, hdd, err_n, 0.05f, f_b);
        const float p = pressure_from_altitude(base_alt + h_true + gauss(0.5f));
        baro_alt_update(&b, t, f_b, q, p, 0.0f, (i % 5) == 0);
    }

    /* Phase 2: 5 s accel only. The control input must keep the
       estimate on the trajectory (REQ-BARO-002). */
    for (; i <= 3500; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts  = (float)i * dt;
        const float hdd = -A * w * w * sinf(w * ts);
        float       f_b[3];
        body_specific_force(q, hdd, err_n, 0.05f, f_b);
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }

    const float ts_end  = (float)(i - 1) * dt;
    const float h_truth = A * sinf(w * ts_end);
    const float v_truth = A * w * cosf(w * ts_end);
    float       h, v;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "still healthy");
    CHECK_TRUE(baro_alt_get_velocity(&b, &v), "get_velocity");
    /* Tolerance 3.0/1.0 (was 1.0/0.5): the accel-noise process-noise fix
       (REQ-BARO-003, see baro_alt_predict's G/Q -- a unit-weight noise
       entry into v instead of a dt-weighted one) makes the filter
       correctly LESS confident in its own accel-derived dead-reckoning
       relative to fresh baro fixes during phase 1's convergence, at the
       large default acc_noise_mps2_sqrthz (0.5, deliberately loose -- see
       BARO_ALT_DEFAULT_ACC_NOISE_MPS2_SQRTHZ's own comment). That shifts
       how tightly a_b converges by the time phase 2's unaided coasting
       starts, so phase 2 drifts a bit further off the sinusoid than
       before the fix. Was previously passing only because the too-small
       injected noise made the filter overconfident, not because a_b was
       genuinely better converged. */
    CHECK_NEAR(h, h_truth, 3.0, "height after 5 s accel-only");
    CHECK_NEAR(v, v_truth, 1.0, "velocity after 5 s accel-only");
}

/* ---------------------------------------------------------------------------
 * Scenario 3b: zero-velocity update, ungated by the velocity estimate
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_zupt(void)
{
    printf("\n-- scenario: zero-velocity update (REQ-BARO-022) --\n");

    const float base_alt = 300.0f;
    const float err_n    = 0.5f; /* constant accel error -> v runs away */
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.precision_restart_disable = true; /* keep the watchdog out of this */

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    /* Baro-aided, stationary warm-up so the covariance settles. */
    int i;
    for (i = 1; i <= 1000; ++i)
    {
        t += US_PER_SEC / 100;
        float f_b[3];
        body_specific_force(q, 0.0f, 0.0f, 0.05f, f_b);
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }

    /* Barometer outage with a biased accelerometer: nothing observes the
       vertical channel, so v dead-reckons away from zero. */
    for (i = 1; i <= 500; ++i)
    {
        t += US_PER_SEC / 100;
        float f_b[3];
        body_specific_force(q, 0.0f, err_n, 0.0f, f_b);
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    float v_drift;
    CHECK_TRUE(baro_alt_get_velocity(&b, &v_drift), "get_velocity after the outage");
    CHECK_TRUE(fabsf(v_drift) > 0.5f, "accel-only dead reckoning ran the velocity away");

    /* The platform is now declared still. The update must be applied even
       though the filter's OWN velocity estimate is far from zero: there is
       no velocity gate, which is the whole point (a gate would suppress it
       exactly here, where the drift is largest). */
    const uint32_t zupt_before = b.n_zupt;
    baro_alt_zero_velocity_update(&b, 0.0f);
    CHECK_TRUE(b.n_zupt == zupt_before + 1, "ZUPT applied despite a large velocity estimate");
    float v_after;
    CHECK_TRUE(baro_alt_get_velocity(&b, &v_after), "get_velocity after the first ZUPT");
    CHECK_TRUE(fabsf(v_after) < fabsf(v_drift), "first ZUPT pulls the velocity toward zero");

    /* Sustained stillness, still no barometer: the pseudo-measurement
       alone must pin the vertical velocity at zero. */
    for (i = 1; i <= 500; ++i)
    {
        t += US_PER_SEC / 100;
        float f_b[3];
        body_specific_force(q, 0.0f, err_n, 0.0f, f_b);
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
        baro_alt_zero_velocity_update(&b, 0.0f);
    }
    /* A persistent unmodelled acceleration cannot be driven to an exact
       zero velocity -- the filter settles where the ZUPT pull balances the
       drift per epoch -- but it must bound what was an unbounded runaway. */
    float v_end;
    CHECK_TRUE(baro_alt_get_velocity(&b, &v_end), "still healthy after sustained ZUPTs");
    CHECK_TRUE(fabsf(v_end) < 0.3f * fabsf(v_drift), "sustained ZUPT bounds the vertical velocity");

    /* Through the v/a_b covariance coupling the standstill also observes
       the acceleration correction: a_b moves to oppose the drift, i.e.
       against the direction the velocity was running away in. */
    float ab;
    CHECK_TRUE(baro_alt_get_acc_bias(&b, &ab), "get_acc_bias");
    CHECK_TRUE(ab * v_drift < 0.0f, "ZUPT drives a_b against the accelerometer error");

    /* Robustness: an explicit stddev is honoured, and the call is a no-op
       (not a crash) on an uninitialized filter. */
    const uint32_t n_before = b.n_zupt;
    baro_alt_zero_velocity_update(&b, 0.2f);
    CHECK_TRUE(b.n_zupt == n_before + 1, "explicit stddev override still fuses");

    baro_alt_t dead;
    memset(&dead, 0, sizeof(dead));
    baro_alt_zero_velocity_update(&dead, 0.0f); /* must not touch anything */
    CHECK_TRUE(dead.n_zupt == 0, "ZUPT on an uninitialized filter is a no-op");
}

/* ---------------------------------------------------------------------------
 * Scenario 3c: nav_suite drives the vertical ZUPT from the ZARU trigger
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_zaru_drives_baro_zupt(void)
{
    printf("\n-- scenario: ZARU trigger -> baro_alt ZUPT (REQ-SUITE-015) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.time                      = 1000000;
    init.llh[0]                    = 48.783 * M_PI / 180.0;
    init.llh[1]                    = 9.181 * M_PI / 180.0;
    init.llh[2]                    = 300.0;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.5f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.5f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    /* Isolate the explicit flag: without this, ins's own auto-ZUPT
       detector would raise the same trigger on this static platform. */
    opt.auto_zupt_disable = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    baro_alt_time_us_t t = 1000000;
    ins_measurements_t m;
    int                i;

    /* Static, locally aided, with barometer: brings the vertical filter up. */
    for (i = 1; i <= 800; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp            = t;
        m.strapdown_dt_sec     = 0.01f;
        m.acc.is_valid         = true;
        m.gyr.is_valid         = true;
        m.acc.data[2]          = -GRAVITY;
        m.local_pos.is_valid   = true;
        m.local_pos.pos_ned[2] = 0.0f;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
        m.baro.is_valid                                                          = true;
        m.baro.pressure_pa = pressure_from_altitude(300.0f + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    float h;
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "vertical filter running");

    /* No zero-rotation flag -> no vertical ZUPT. */
    const uint32_t idle_before = s.baro_alt.n_zupt;
    for (i = 1; i <= 100; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(s.baro_alt.n_zupt == idle_before, "no ZARU trigger -> no vertical ZUPT");

    /* Zero-rotation flag set -> one vertical ZUPT per epoch. */
    const uint32_t armed_before = s.baro_alt.n_zupt;
    for (i = 1; i <= 100; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp            = t;
        m.strapdown_dt_sec     = 0.01f;
        m.acc.is_valid         = true;
        m.gyr.is_valid         = true;
        m.acc.data[2]          = -GRAVITY;
        m.zero_rotation_update = true;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(s.baro_alt.n_zupt == armed_before + 100,
               "ZARU trigger drives one vertical ZUPT per epoch");
    CHECK_TRUE(nav_suite_get_zaru_active(&s), "zaru trigger reported active");
    CHECK_TRUE(nav_suite_get_vertical_zupt_active(&s), "vertical ZUPT reported active");

    /* The ARS/AHRS velocity-blind auto-ZARU fallback must reach the
       vertical channel too (REQ-SUITE-015): without an absolute position
       aid ins never initializes, so ins_auto_zupt_active() stays false
       and this fallback is the suite's only stillness detector. Arm it on
       a fresh suite and drive a static platform with NO explicit flag. */
    static nav_suite_t s2;
    memset(&s2, 0, sizeof(s2));
    CHECK_TRUE(nav_suite_init(&s2, &init, &opt) == 0, "nav_suite_init (fallback suite)");
    /* Arm the velocity-blind fallback (same knob the Python capi pokes in
       ins_suite_set_auto_zaru). Only the ARS bootstraps here -- the
       magnetometer AHRS needs a mag sample, which this scenario has none. */
    s2.ars_cfg.auto_zaru_disable  = false;
    s2.ahrs_cfg.auto_zaru_disable = false;

    t = 1000000;
    for (i = 1; i <= 800; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(300.0f + gauss(0.2f));
        nav_suite_update(&s2, &m);
    }
    CHECK_TRUE(nav_suite_get_baro_alt(&s2, &h, (float*)0), "vertical filter running (fallback)");
    CHECK_TRUE(!s2.last_zaru_trigger, "no explicit flag, ins not ready -> plain trigger false");
    CHECK_TRUE(nav_suite_get_zaru_active(&s2),
               "the accessor still reports the fallback (REQ-SUITE-010)");
    CHECK_TRUE(nav_suite_get_vertical_zupt_active(&s2),
               "ARS/AHRS auto-ZARU fallback reaches the vertical channel");

    const uint32_t fallback_before = s2.baro_alt.n_zupt;
    for (i = 1; i <= 100; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        nav_suite_update(&s2, &m);
    }
    CHECK_TRUE(s2.baro_alt.n_zupt == fallback_before + 100,
               "fallback drives one vertical ZUPT per epoch");
}

/* ---------------------------------------------------------------------------
 * Scenario 4: outlier downweighting (spike + persistent offset)
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_outlier(void)
{
    printf("\n-- scenario: baro outlier downweighting --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY}; /* static, no noise */

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    /* Converge for 10 s on the clean static baro. */
    int i;
    for (i = 1; i <= 1000; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }

    /* Single +50 m spike: must be downweighted away, h must not jump. */
    t += US_PER_SEC / 100;
    baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt + 50.0f), 0.0f, true);
    float h;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy after spike");
    CHECK_NEAR(h, 0.0, 1.0, "50 m spike does not move h");

    /* Persistent +5 m offset: downweighted, NOT skipped. The filter
       must follow eventually instead of deadlocking (REQ-BARO-005). */
    for (i = 1; i <= 4000; ++i) /* 40 s */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt + 5.0f), 0.0f, (i % 5) == 0);
    }
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy after offset");
    CHECK_NEAR(h, 5.0, 1.5, "persistent offset is followed (no deadlock)");
    CHECK_TRUE(b.n_fuse_fail == 0, "no fusion errors");
}

/* ---------------------------------------------------------------------------
 * Scenario 5: non-finite input handling
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_nan_inputs(void)
{
    printf("\n-- scenario: NaN/Inf inputs --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};
    const float nan      = nanf("");

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;

    /* Init must reject a non-finite or implausible anchor pressure. */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, nan, 0.0f, 0.0f) == -1, "init rejects NaN p");
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, 5.0f, 0.0f, 0.0f) == -1, "init rejects implausible p");
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "init with valid pressure");

    /* Warm up. */
    int i;
    for (i = 1; i <= 200; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }
    const uint32_t epochs_before = b.epoch;
    float          h_before;
    CHECK_TRUE(baro_alt_get_height(&b, &h_before), "healthy before");

    /* NaN accel: epoch dropped, state untouched. */
    t += US_PER_SEC / 100;
    const float acc_nan[3] = {0.0f, nan, -GRAVITY};
    baro_alt_update(&b, t, acc_nan, q, pressure_from_altitude(base_alt), 0.0f, true);
    CHECK_TRUE(b.n_invalid_input == 1, "NaN accel counted");
    CHECK_TRUE(b.epoch == epochs_before, "NaN accel epoch dropped");

    /* Inf quaternion: epoch dropped. */
    t += US_PER_SEC / 100;
    const float q_inf[4] = {1.0f, 0.0f, INFINITY, 0.0f};
    baro_alt_update(&b, t, f_b, q_inf, pressure_from_altitude(base_alt), 0.0f, true);
    CHECK_TRUE(b.n_invalid_input == 2, "Inf quaternion counted");

    /* Every OTHER acc/quaternion component, one at a time (MC/DC: each
       condition in the isfinite() chain must be independently
       sensitized, not just the two above). */
    const uint32_t n_invalid_before_components = b.n_invalid_input;
    {
        uint32_t n_before = b.n_invalid_input;
        t += US_PER_SEC / 100;
        const float acc_bad[3] = {nan, 0.0f, -GRAVITY};
        baro_alt_update(&b, t, acc_bad, q, pressure_from_altitude(base_alt), 0.0f, true);
        CHECK_TRUE(b.n_invalid_input == n_before + 1, "NaN acc[0] counted");
    }
    {
        uint32_t n_before = b.n_invalid_input;
        t += US_PER_SEC / 100;
        const float acc_bad[3] = {0.0f, 0.0f, (float)INFINITY};
        baro_alt_update(&b, t, acc_bad, q, pressure_from_altitude(base_alt), 0.0f, true);
        CHECK_TRUE(b.n_invalid_input == n_before + 1, "Inf acc[2] counted");
    }
    {
        uint32_t n_before = b.n_invalid_input;
        t += US_PER_SEC / 100;
        const float q_bad[4] = {nan, 0.0f, 0.0f, 0.0f};
        baro_alt_update(&b, t, f_b, q_bad, pressure_from_altitude(base_alt), 0.0f, true);
        CHECK_TRUE(b.n_invalid_input == n_before + 1, "NaN q_bn[0] counted");
    }
    {
        uint32_t n_before = b.n_invalid_input;
        t += US_PER_SEC / 100;
        const float q_bad[4] = {1.0f, nan, 0.0f, 0.0f};
        baro_alt_update(&b, t, f_b, q_bad, pressure_from_altitude(base_alt), 0.0f, true);
        CHECK_TRUE(b.n_invalid_input == n_before + 1, "NaN q_bn[1] counted");
    }
    {
        uint32_t n_before = b.n_invalid_input;
        t += US_PER_SEC / 100;
        const float q_bad[4] = {1.0f, 0.0f, 0.0f, (float)INFINITY};
        baro_alt_update(&b, t, f_b, q_bad, pressure_from_altitude(base_alt), 0.0f, true);
        CHECK_TRUE(b.n_invalid_input == n_before + 1, "Inf q_bn[3] counted");
    }

    CHECK_TRUE(b.n_invalid_input == n_invalid_before_components + 5,
               "all 5 extra components counted");

    /* NaN pressure with valid IMU: only the baro sample is dropped,
       the propagation still runs. */
    const uint32_t n_invalid_before_pressure = b.n_invalid_input;
    t += US_PER_SEC / 100;
    baro_alt_update(&b, t, f_b, q, nan, 0.0f, true);
    CHECK_TRUE(b.n_invalid_input == n_invalid_before_pressure + 1, "NaN pressure counted");
    CHECK_TRUE(b.epoch == epochs_before + 1, "epoch still processed");

    /* Filter continues normally with valid data. */
    for (i = 1; i <= 200; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }
    float h;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "still healthy");
    CHECK_NEAR(h, h_before, 0.2, "state undamaged");
}

/* ---------------------------------------------------------------------------
 * Scenario 6: time anomalies
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_time_anomaly(void)
{
    printf("\n-- scenario: time anomalies --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    int i;
    for (i = 1; i <= 100; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }

    /* Backwards step: epoch skipped, state exactly untouched. Deliberate
       bit-exact memcmp: checks the memory wasn't touched at all, not
       float value equality, so the usual "don't memcmp floats" concern
       (NaN, +-0) doesn't apply. */
    float x_before[BARO_ALT_STATES];
    memcpy(x_before, b.x, sizeof(x_before));
    baro_alt_update(&b, t - US_PER_SEC / 2, f_b, q, pressure_from_altitude(base_alt), 0.0f, true);
    // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
    CHECK_TRUE(memcmp(x_before, b.x, sizeof(x_before)) == 0,
               "backwards step leaves state untouched");
    CHECK_TRUE(b.is_initialized, "still initialized");

    /* The clock is re-anchored at the backwards timestamp: continue
       from there. */
    t = t - US_PER_SEC / 2 + US_PER_SEC / 100;
    baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, false);
    CHECK_TRUE(b.is_initialized, "continues after re-anchor");

    /* Forward gap > 0.2 s: the propagation is skipped (an absurd
       specific force over the gap must NOT be integrated), baro fusion
       still runs. */
    const float f_absurd[3] = {0.0f, 0.0f, -(GRAVITY + 10.0f)};
    t += US_PER_SEC; /* 1 s gap */
    baro_alt_update(&b, t, f_absurd, q, pressure_from_altitude(base_alt), 0.0f, true);
    float h;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy after gap");
    CHECK_NEAR(h, 0.0, 0.2, "gap epoch not integrated");
}

/* ---------------------------------------------------------------------------
 * Scenario 7: nav_suite integration
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_suite(void)
{
    printf("\n-- scenario: nav_suite integration --\n");

    const float base_alt = 300.0f;
    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};

    /* Static level body: f_b = (0, 0, -g), mag rotated by yaw. */
    float q_true[4], R[9];
    ins_quat_from_rpy(0.0f, 0.0f, yaw_true, q_true);
    ins_quat_to_rotmat(q_true, R);
    float f_b[3], mag_b[3];
    int   i;
    for (i = 0; i < 3; ++i)
    {
        f_b[i]   = -GRAVITY * MAT_ELEM(R, 2, i, 3, 3);
        mag_b[i] = MAT_ELEM(R, 0, i, 3, 3) * mag_n[0] + MAT_ELEM(R, 1, i, 3, 3) * mag_n[1] +
                   MAT_ELEM(R, 2, i, 3, 3) * mag_n[2];
    }

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = 48.783 * M_PI / 180.0;
    init.llh[1]                    = 9.181 * M_PI / 180.0;
    init.llh[2]                    = 300.0;
    init.rpy_init_rad[2]           = yaw_true;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    memcpy(init.magnetic_n, mag_n, sizeof(mag_n));

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;

    static nav_suite_t s; /* keep the large struct off the stack */
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h, v;

    /* Phase 1: IMU + mag, no baro. The vertical filter must wait. */
    for (i = 0; i < 100; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.mag.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        memcpy(m.mag.data, mag_b, sizeof(mag_b));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(!nav_suite_get_baro_alt(&s, &h, &v), "no baro seen: accessor false");

    /* Phase 2: baro appears. First sample bootstraps (h = 0 there). */
    for (i = 0; i < 200; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(base_alt + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, &v), "baro filter running");
    CHECK_NEAR(h, 0.0, 0.4, "static: height above start = 0");
    CHECK_NEAR(v, 0.0, 0.2, "static: vertical velocity = 0");

    /* Phase 3: slow climb of 2 m over 10 s (constant climb rate, so
       the static accel stays consistent), then 5 s hold. */
    for (i = 0; i < 1500; ++i)
    {
        t += US_PER_SEC / 100;
        const float h_true = (i < 1000) ? 2.0f * (float)i / 1000.0f : 2.0f;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(base_alt + h_true + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, &v), "still running");
    CHECK_NEAR(h, 2.0, 0.5, "climb tracked");
    CHECK_NEAR(v, 0.0, 0.3, "velocity settled after hold");
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "NULL output pointer allowed");
}

/* ---------------------------------------------------------------------------
 * Scenario 8: datum-aligned initialization (h_init)
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_h_init(void)
{
    printf("\n-- scenario: datum-aligned init (h_init) --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    /* The anchor pressure sample sits 25 m above the caller's datum. */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 25.0f, 2.0f) == 0,
               "init with h_init");
    float h;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "get_height");
    CHECK_NEAR(h, 25.0, 1e-4, "state starts at h_init");

    /* Static samples at the anchor altitude keep h at h_init. */
    int i;
    for (i = 1; i <= 500; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy");
    CHECK_NEAR(h, 25.0, 0.2, "measurements map into the datum");

    /* Slow +3 m climb in the pressure (10 s ramp + 5 s hold). */
    for (i = 1; i <= 1500; ++i)
    {
        t += US_PER_SEC / 100;
        const float dh = (i < 1000) ? 3.0f * (float)i / 1000.0f : 3.0f;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt + dh), 0.0f, (i % 5) == 0);
    }
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy 2");
    /* The a_b state follows this synthetic ramp with a lag, so the check is
       on the datum being preserved, not on an exact match. */
    CHECK_NEAR(h, 28.0, 0.7, "datum preserved while following baro");

    /* Non-finite h_init is rejected. */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), nanf(""), 0.0f) == -1,
               "NaN h_init rejected");
}

/* ---------------------------------------------------------------------------
 * Scenario 8b: process noise is rate-invariant (REQ-BARO-003)
 * ---------------------------------------------------------------------------
 */

/* P_ii = d_i + sum_{k>i} U(i,k)^2 * d_k (UDU-factored diagonal covariance,
   same formula as python/csrc/ins_capi.c's diag_covariance_of -- not
   exposed through baro_alt.h, so duplicated here for this test only). */
static float baro_alt_test_state_var(const baro_alt_t* b, int i)
{
    float p = b->d[i];
    int   k;
    for (k = i + 1; k < BARO_ALT_STATES; ++k)
    {
        const float u = MAT_ELEM(b->U, i, k, BARO_ALT_STATES, BARO_ALT_STATES);
        p += u * u * b->d[k];
    }
    return p;
}

/* Free-coasts baro_alt (init only, no barometer fix ever) for T_sec at the
   given IMU rate, returns the velocity state's variance at the end. */
static float baro_alt_free_coast_v_var(float acc_noise_sqrthz, float dt, float T_sec)
{
    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.acc_noise_mps2_sqrthz      = acc_noise_sqrthz;
    cfg.acc_bias_drift_mps2_sqrthz = 1e-9f; /* negligible: isolate sigma_a */
    cfg.acc_bias_init_stddev_mps2  = 1e-9f;
    cfg.v_init_stddev_mps          = 1e-9f;

    baro_alt_t         b;
    baro_alt_time_us_t t = 0;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(0.0f), 0.0f, 1e-9f) == 0,
               "rate-invariance test init");

    const float q[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3] = {0.0f, 0.0f, -GRAVITY};
    const int   n      = (int)(T_sec / dt + 0.5f);
    int         i;
    for (i = 1; i <= n; ++i)
    {
        t += (baro_alt_time_us_t)(dt * (float)US_PER_SEC);
        /* baro_valid=false: pure accelerometer dead reckoning throughout
           (REQ-BARO-002), so v's growth is process noise alone. */
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(0.0f), 1.0f, false);
    }
    return baro_alt_test_state_var(&b, 1);
}

static void scenario_baro_rate_invariant_process_noise(void)
{
    printf("\n-- scenario: process noise is rate-invariant (REQ-BARO-003) --\n");

    const float T         = 15.0f;
    const float acc_noise = 0.2f;
    const float var_100hz = baro_alt_free_coast_v_var(acc_noise, 0.01f, T);
    const float var_50hz  = baro_alt_free_coast_v_var(acc_noise, 0.02f, T);
    const float var_25hz  = baro_alt_free_coast_v_var(acc_noise, 0.04f, T);

    /* Same accumulated velocity variance over the same free-coasting
       interval regardless of the IMU rate -- a discretization artifact
       once made the accumulated variance shrink by ~4x per halved rate
       (Q paired with a dt-weighted noise entry instead of a unit one). */
    CHECK_NEAR(var_50hz, var_100hz, var_100hz * 0.1f, "rate-invariant: 50 Hz matches 100 Hz");
    CHECK_NEAR(var_25hz, var_100hz, var_100hz * 0.15f, "rate-invariant: 25 Hz matches 100 Hz");

    /* Matches the closed-form q*T (v is a plain 1-integral random walk
       driven by sigma_a^2, bias-drift negligible here by construction). */
    const float expect_var = acc_noise * acc_noise * T;
    CHECK_NEAR(var_100hz, expect_var, expect_var * 0.1f, "matches sigma_a^2 * T closed form");
}

/* ---------------------------------------------------------------------------
 * Scenario 8c: direct process noise on h is rate-invariant (REQ-BARO-003)
 * ---------------------------------------------------------------------------
 */

/* Free-coasts baro_alt (init only, no barometer fix ever) for T_sec at the
   given IMU rate, returns the height state's variance at the end, with
   sigma_a/sigma_b driven to ~0 so h's variance growth is sigma_h alone
   (isolates the new direct e1 noise column from the v->h Phi coupling
   REQ-BARO-003 already covers via scenario_baro_rate_invariant_process_noise). */
static float baro_alt_free_coast_h_var(float h_noise_sqrthz, float dt, float T_sec)
{
    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.h_process_noise_m_sqrthz   = h_noise_sqrthz;
    cfg.acc_noise_mps2_sqrthz      = 1e-9f; /* negligible: isolate sigma_h */
    cfg.acc_bias_drift_mps2_sqrthz = 1e-9f;
    cfg.acc_bias_init_stddev_mps2  = 1e-9f;
    cfg.v_init_stddev_mps          = 1e-9f;

    baro_alt_t         b;
    baro_alt_time_us_t t = 0;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(0.0f), 0.0f, 1e-9f) == 0,
               "h process-noise test init");

    const float q[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3] = {0.0f, 0.0f, -GRAVITY};
    const int   n      = (int)(T_sec / dt + 0.5f);
    int         i;
    for (i = 1; i <= n; ++i)
    {
        t += (baro_alt_time_us_t)(dt * (float)US_PER_SEC);
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(0.0f), 1.0f, false);
    }
    return baro_alt_test_state_var(&b, 0);
}

static void scenario_baro_h_process_noise(void)
{
    printf("\n-- scenario: direct process noise on h (REQ-BARO-003) --\n");

    const float T         = 15.0f;
    const float h_noise   = 0.02f;
    const float var_100hz = baro_alt_free_coast_h_var(h_noise, 0.01f, T);
    const float var_50hz  = baro_alt_free_coast_h_var(h_noise, 0.02f, T);

    /* Same accumulated height variance over the same free-coasting
       interval regardless of the IMU rate, same rationale as the
       accelerometer-into-v term above. */
    CHECK_NEAR(var_50hz, var_100hz, var_100hz * 0.1f, "rate-invariant: 50 Hz matches 100 Hz");

    /* Matches the closed-form q*T: h's direct e1 noise column is a plain
       random walk driven by sigma_h^2, with sigma_a/sigma_b negligible by
       construction so the v->h Phi coupling contributes nothing here. */
    const float expect_var = h_noise * h_noise * T;
    CHECK_NEAR(var_100hz, expect_var, expect_var * 0.1f, "matches sigma_h^2 * T closed form");
}

/* ---------------------------------------------------------------------------
 * Scenario 9: baro-GNSS offset filter basics
 * ---------------------------------------------------------------------------
 */

static void scenario_offset_filter(void)
{
    printf("\n-- scenario: baro-gnss offset filter --\n");

    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* Exercise the base Kalman convergence at the pair rate itself.
       The low-rate decimation/derating defaults are covered separately
       (scenario_offset_low_rate_defaults). rw is pinned as well: the
       tolerances below are the steady state of THIS random walk against a
       constant truth, which is a property of the Kalman recursion, not of
       whatever the shipped default happens to be tuned for (that default
       is exercised in scenario_offset_drift_tracking). */
    cfg.min_update_interval_sec = 0.5f;
    cfg.stddev_inflation_factor = 1.0f;
    cfg.rw_stddev_mps           = 0.03f;

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;

    /* Init from the first pair: offset and combined pair accuracy. The
       local stddev is passed as 0, so it resolves to the shared barometer
       default; the GNSS vertical one is 1.0 m. Derived from the constant
       rather than written out, so retuning the default cannot silently
       leave this expectation behind. */
    const double sd_baro = (double)INS_DEFAULT_BARO_STDDEV_M;
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, 331.4f, 1.0f) == 0, "init from pair");
    float off, sd;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, &sd), "get");
    CHECK_NEAR(off, 51.4, 1e-3, "offset = h_gnss - h_baro");
    CHECK_NEAR(sd, sqrt(sd_baro * sd_baro + 1.0 * 1.0), 1e-3, "stddev = combined pair accuracy");

    /* 120 noisy pairs at 1 Hz: converge, uncertainty shrinks. */
    int i;
    for (i = 0; i < 120; ++i)
    {
        t += US_PER_SEC;
        local_gnss_alt_update(&g, t, 280.0f + gauss(0.5f), 0.0f, 331.4f + gauss(1.0f), 1.0f);
    }
    CHECK_TRUE(local_gnss_alt_get(&g, &off, &sd), "get 2");
    CHECK_NEAR(off, 51.4, 0.4, "offset converged");
    CHECK_TRUE(sd < 0.4f, "uncertainty shrunk");
    CHECK_TRUE(g.n_invalid_input == 0, "no pairs dropped");
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "NULL output pointer allowed");
}

/* ---------------------------------------------------------------------------
 * Scenario 8b: baro_alt datum relocation (REQ-BARO-020)
 *
 * Moving the datum must move the local height and leave the barometric
 * ISA altitude (a property of the atmosphere) untouched.
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_datum_shift(void)
{
    printf("\n-- scenario: baro_alt datum invariant --\n");

    /* The datum-relocation primitives (baro_alt_shift_datum,
       local_gnss_alt_shift_datum) were removed: the vertical datum is now
       fixed once and never moves. The late filter conforms at its init
       instead (REQ-SUITE-007). This scenario just pins the property those
       primitives used to provide: the barometric ISA altitude is a
       property of the atmosphere, independent of where the caller put its
       datum (h_init). */
    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t         b0, b25;
    baro_alt_time_us_t t = 1000000;

    /* Same pressure, two different datum choices (h_init 0 vs 25). */
    CHECK_TRUE(baro_alt_init(&b0, &cfg, t, pressure_from_altitude(300.0f), 0.0f, 1.0f) == 0,
               "init0");
    CHECK_TRUE(baro_alt_init(&b25, &cfg, t, pressure_from_altitude(300.0f), 25.0f, 1.0f) == 0,
               "init25");
    float h0, h25, isa0, isa25;
    CHECK_TRUE(baro_alt_get_height(&b0, &h0) && baro_alt_get_height(&b25, &h25), "heights");
    CHECK_TRUE(baro_alt_get_isa_altitude(&b0, &isa0) && baro_alt_get_isa_altitude(&b25, &isa25),
               "isa");
    CHECK_NEAR(h25 - h0, 25.0, 1e-4, "local height reflects the datum choice");
    CHECK_NEAR(isa0, isa25, 1e-3, "ISA altitude is the same regardless of datum");
    CHECK_NEAR(isa0, 300.0, 0.01, "ISA altitude equals the anchor");
}

/* ---------------------------------------------------------------------------
 * Scenario 10: slow random walk, drift tracking + outage growth
 * ---------------------------------------------------------------------------
 */

static void scenario_offset_drift_tracking(void)
{
    printf("\n-- scenario: offset drift tracking --\n");

    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* Exercise drift tracking / outage growth at the pair rate itself.
       The low-rate decimation/derating defaults are covered separately
       (scenario_offset_low_rate_defaults). */
    cfg.min_update_interval_sec = 0.5f;
    cfg.stddev_inflation_factor = 1.0f;

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, 331.4f, 1.0f) == 0, "init");

    /* Weather drift: the baro reading sinks by 5 m over 600 s while
       the true (GNSS) height is constant -> the offset must follow. */
    int i;
    for (i = 1; i <= 600; ++i)
    {
        t += US_PER_SEC;
        const float drift = 5.0f * (float)i / 600.0f;
        local_gnss_alt_update(&g, t, 280.0f - drift + gauss(0.5f), 0.0f, 331.4f + gauss(1.0f),
                              1.0f);
    }
    float off;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy");
    /* The residual is the lag a higher barometer stddev default
       (INS_DEFAULT_BARO_STDDEV_M) adds against a linear drift, since it
       weighs the barometer down relative to the (drift-free) GNSS side of
       the pair. */
    CHECK_NEAR(off, 56.4, 0.65, "weather drift tracked");

    /* Pair outage: the variance keeps growing (stale offsets lose
       confidence), even across the fusion of the next pair. */
    const float var_before = g.var_m2;
    t += 300 * US_PER_SEC;
    local_gnss_alt_update(&g, t, 275.0f + gauss(0.5f), 0.0f, 331.4f + gauss(1.0f), 1.0f);
    CHECK_TRUE(g.var_m2 > var_before, "variance grew during outage");

    /* The other half of REQ-BARO-011, and the harder one: the ISA-model
       error, which grows with the height EXCURSION rather than with time.
       A vehicle climbing 200 m in 5 minutes against an ISA scale error of
       ~12 % (an ordinary figure - the model assumes a 288 K sea level that
       the real atmosphere rarely has) swings the offset by ~23 m inside
       those 5 minutes, an order of magnitude faster than the weather ramp
       above. Run at the SHIPPED defaults, since what is on trial here is
       whether they meet the requirement they are documented under. */
    {
        local_gnss_alt_config_t dcfg;
        memset(&dcfg, 0, sizeof(dcfg)); /* everything at its default */
        local_gnss_alt_t   d;
        baro_alt_time_us_t td = 1000000;
        CHECK_TRUE(local_gnss_alt_init(&d, &dcfg, td, 280.0f, 0.0f, 331.4f, 1.0f) == 0,
                   "isa excursion: init");

        const float climb_m = 200.0f;
        const float scale   = 0.12f;
        int         k;
        for (k = 1; k <= 300; ++k)
        {
            td += US_PER_SEC;
            const float f_climb = climb_m * (float)k / 300.0f;
            /* GNSS sees the whole climb, the barometer only (1 - scale) of
               it, so the true offset grows by scale * climb. */
            local_gnss_alt_update(&d, td, 280.0f + f_climb * (1.0f - scale) + gauss(0.5f), 0.0f,
                                  331.4f + f_climb + gauss(1.0f), 1.0f);
        }
        float doff;
        CHECK_TRUE(local_gnss_alt_get(&d, &doff, (float*)0), "isa excursion: healthy");
        /* 10 m of a 24 m swing: the random walk can only BOUND a drift whose
           driver is altitude and not the clock, so a lag remains by
           construction. What must not happen is the offset sitting out the
           excursion entirely, which is what a walk slow enough to make the
           real drift look like a stream of chi2 outliers does. */
        CHECK_NEAR(doff, 51.4 + climb_m * scale, 10.0, "ISA-excursion drift followed");
        CHECK_TRUE(d.n_downweighted * 3u <= d.epoch,
                   "real drift is not mistaken for a stream of outliers");
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 11: offset outlier downweighting
 * ---------------------------------------------------------------------------
 */

static void scenario_offset_outlier(void)
{
    printf("\n-- scenario: offset outlier downweighting --\n");

    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* Exercise the chi2 downweighting itself, at the pair rate. The
       low-rate decimation default would otherwise just skip the spike
       outright instead of downweighting it (covered separately by
       scenario_offset_low_rate_default). */
    cfg.min_update_interval_sec = 0.5f;
    cfg.stddev_inflation_factor = 1.0f;

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, 331.4f, 1.0f) == 0, "init");

    int i;
    for (i = 0; i < 60; ++i)
    {
        t += US_PER_SEC;
        local_gnss_alt_update(&g, t, 280.0f, 0.0f, 331.4f, 1.0f);
    }
    float off;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "converged");
    CHECK_NEAR(off, 51.4, 0.05, "clean convergence");

    /* Single +30 m GNSS spike: downweighted away. */
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.0f, 361.4f, 1.0f);
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy after spike");
    CHECK_NEAR(off, 51.4, 0.1, "spike does not move the offset");

    /* Persistent +3 m step: followed eventually (downweight, not skip). */
    for (i = 0; i < 600; ++i)
    {
        t += US_PER_SEC;
        local_gnss_alt_update(&g, t, 280.0f, 0.0f, 334.4f, 1.0f);
    }
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy after step");
    CHECK_NEAR(off, 54.4, 0.5, "persistent step followed (no deadlock)");
}

/* ---------------------------------------------------------------------------
 * Scenario 12: offset non-finite/invalid input handling
 * ---------------------------------------------------------------------------
 */

static void scenario_offset_nan_inputs(void)
{
    printf("\n-- scenario: offset NaN/invalid inputs --\n");

    const float             nan = nanf("");
    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* Exercise the backwards-timestamp re-anchor itself, at the pair
       rate. The low-rate decimation default would otherwise absorb the
       backwards pair before it ever reaches that check. */
    cfg.min_update_interval_sec = 0.5f;
    cfg.stddev_inflation_factor = 1.0f;

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;

    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, nan, 0.0f, 331.4f, 1.0f) == -1,
               "init rejects NaN baro altitude");
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, nan, 1.0f) == -1,
               "init rejects NaN gnss height");
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, 331.4f, 0.0f) == -1,
               "init rejects zero gnss accuracy");
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.0f, 331.4f, 1.0f) == 0,
               "init with valid pair");

    float off_before;
    CHECK_TRUE(local_gnss_alt_get(&g, &off_before, (float*)0), "get");

    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.0f, nan, 1.0f);
    CHECK_TRUE(g.n_invalid_input == 1, "NaN gnss height counted");
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, nan, 0.0f, 331.4f, 1.0f);
    CHECK_TRUE(g.n_invalid_input == 2, "NaN baro altitude counted");
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.0f, 331.4f, -1.0f);
    CHECK_TRUE(g.n_invalid_input == 3, "negative gnss accuracy counted");

    float off;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "still healthy");
    CHECK_NEAR(off, off_before, 1e-6, "state untouched by bad pairs");

    /* Backwards timestamp (relative to the last VALID pair; dropped
       pairs do not advance the clock): skipped, clock re-anchored. */
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.0f, 331.4f, 1.0f);
    float off_anchor;
    CHECK_TRUE(local_gnss_alt_get(&g, &off_anchor, (float*)0), "valid pair");
    local_gnss_alt_update(&g, t - US_PER_SEC, 285.0f, 0.0f, 331.4f, 1.0f);
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy after t-jump");
    CHECK_NEAR(off, off_anchor, 1e-6, "backwards pair skipped");

    /* Continues normally with the next valid pair. */
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.0f, 331.4f, 1.0f);
    CHECK_TRUE(g.is_initialized, "continues with valid pairs");
}

/* ---------------------------------------------------------------------------
 * Scenario 13: offset filter low-rate defaults (decimation + stddev
 * derating), REQ-BARO-014, REQ-BARO-015
 * ---------------------------------------------------------------------------
 */

static void scenario_offset_low_rate_default(void)
{
    printf("\n-- scenario: offset filter low-rate defaults --\n");

    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg)); /* all-zero: exercise the documented defaults */

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.5f, 331.4f, 1.0f) == 0, "init");

    float off, sd;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, &sd), "get after init");
    CHECK_NEAR(sd, sqrt(11.25), 1e-3, /* (0.5*3)^2 + (1.0*3)^2 = 11.25 */
               "combined variance uses the default 3x stddev inflation");

    /* Pairs at 1 Hz for 9 s: all decimated (default interval is 10 s),
       the offset must not move even though the true offset changed. */
    int i;
    for (i = 0; i < 9; ++i)
    {
        t += US_PER_SEC;
        local_gnss_alt_update(&g, t, 280.0f, 0.5f, 341.4f, 1.0f); /* true offset now 61.4 */
    }
    CHECK_TRUE(g.n_decimated == 9, "9 pairs decimated");
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "still healthy");
    CHECK_NEAR(off, 51.4, 1e-3, "offset untouched while decimated");

    /* The 10th second crosses the interval: this pair fuses. */
    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.5f, 341.4f, 1.0f);
    CHECK_TRUE(g.n_decimated == 9, "10th pair not decimated");
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy after fusion");
    CHECK_TRUE(off > 51.4f && off < 61.4f, "offset moved towards the new true value");
}

/* ---------------------------------------------------------------------------
 * Scenario 14: global chi2-disable override (REQ-BARO-016): an outlier
 * that would normally be chi2-downweighted is fused at face value in
 * both baro_alt (main h fusion) and the local_gnss offset filter.
 * ---------------------------------------------------------------------------
 */

/* baro_alt: converge on a clean static baro for 10 s (h -> tight
   covariance), then fuse one +50 m spike. Returns h right after the
   spike. With chi2 downweighting, the well-converged filter barely
   moves; disabled, the same tiny gain still applies but without the
   gate's extra R inflation, so the shift is measurably larger. */
static float run_baro_alt_spike(bool chi2_disable)
{
    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.chi2_disable = chi2_disable;

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");
    int i;
    for (i = 1; i <= 1000; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
    }
    t += US_PER_SEC / 100;
    baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt + 50.0f), 0.0f, true);
    float h;
    CHECK_TRUE(baro_alt_get_height(&b, &h), "healthy after spike");

    /* REQ-BARO-017: the downweight counter must fire exactly on the
       spike when chi2 is active, and never when disabled. */
    if (chi2_disable) { CHECK_TRUE(b.n_downweighted == 0, "chi2_disable: n_downweighted stays 0"); }
    else { CHECK_TRUE(b.n_downweighted == 1, "default: n_downweighted counts the spike"); }

    return h;
}

/* Same idea for the offset filter: converge, then fuse one +30 m step. */
static float run_offset_step(bool chi2_disable)
{
    local_gnss_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.min_update_interval_sec = 0.5f; /* isolate from the low-rate default */
    cfg.stddev_inflation_factor = 1.0f;
    cfg.chi2_disable            = chi2_disable;

    local_gnss_alt_t   g;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(local_gnss_alt_init(&g, &cfg, t, 280.0f, 0.5f, 331.4f, 1.0f) == 0, "offset init");
    int i;
    for (i = 0; i < 60; ++i)
    {
        t += US_PER_SEC;
        local_gnss_alt_update(&g, t, 280.0f, 0.5f, 331.4f, 1.0f);
    }
    float off;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "converged");
    CHECK_NEAR(off, 51.4, 0.05, "clean convergence");

    t += US_PER_SEC;
    local_gnss_alt_update(&g, t, 280.0f, 0.5f, 361.4f, 1.0f);
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "healthy after step");

    /* REQ-BARO-017: the downweight counter must fire exactly on the
       step when chi2 is active, and never when disabled. */
    if (chi2_disable) { CHECK_TRUE(g.n_downweighted == 0, "chi2_disable: n_downweighted stays 0"); }
    else { CHECK_TRUE(g.n_downweighted == 1, "default: n_downweighted counts the step"); }

    return off;
}

static void scenario_chi2_disable(void)
{
    printf("\n-- scenario: global chi2-disable override --\n");

    const float h_downweighted = run_baro_alt_spike(false);
    const float h_raw          = run_baro_alt_spike(true);
    printf("  baro_alt h after 50 m spike: downweighted=%g m, chi2_disable=%g m\n", h_downweighted,
           h_raw);
    CHECK_TRUE(fabsf(h_downweighted) < 1.0f, "default: spike downweighted, barely moves h");
    CHECK_TRUE(h_raw > 2.0f * fabsf(h_downweighted) + 0.5f,
               "chi2_disable: spike fused at face value, h shifts more");

    const float off_downweighted = run_offset_step(false);
    const float off_raw          = run_offset_step(true);
    printf("  offset after +30 m step: downweighted=%g m, chi2_disable=%g m\n", off_downweighted,
           off_raw);
    CHECK_TRUE(fabsf(off_downweighted - 51.4f) < 0.1f, "default: step downweighted, barely moves");
    CHECK_TRUE(off_raw > off_downweighted + 0.2f,
               "chi2_disable: step fused at face value, offset shifts more");

    /* baro_alt_zero_velocity_update() resolves its own chi2 threshold
       from the same cfg.chi2_disable flag, a separate call site from
       the two helpers above. */
    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.chi2_disable = true;
    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, 101325.0f, 0.0f, 0.0f) == 0, "init (chi2_disable, zupt)");
    baro_alt_zero_velocity_update(&b, 0.0f); /* must not crash, gate skipped */
    CHECK_TRUE(b.is_initialized, "zupt with chi2_disable stays healthy");
}

/* ---------------------------------------------------------------------------
 * Scenario 15: nav_suite anchors baro_alt at the ins NED height
 *
 * ins (manual init) climbs 5 m on local-position aiding before the
 * first barometer sample arrives. The baro filter must then start at
 * h = 5, not 0, so both share the origin datum. When the position
 * aiding stops, nav_suite_get_height() falls back to the baro filter
 * without a datum jump.
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_anchor_from_ins(void)
{
    printf("\n-- scenario: baro anchor from INSLIB NED height --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = 48.783 * M_PI / 180.0;
    init.llh[1]                    = 9.181 * M_PI / 180.0;
    init.llh[2]                    = 300.0;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    opt.max_deadreckoning_sec              = 1.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    int                i;

    /* 0-2 s static warm-up, 2-6 s smooth 5 m climb, 6-8 s static at
       h = 5. Local-position aiding tracks the truth; the IMU sees the
       matching specific force. No barometer yet. */
    float h_true = 0.0f;
    for (i = 1; i <= 800; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts  = (float)i * 0.01f;
        float       hdd = 0.0f;
        if (ts > 2.0f && ts <= 6.0f)
        {
            const float ph = (float)M_PI * (ts - 2.0f) / 4.0f;
            h_true         = 2.5f * (1.0f - cosf(ph));
            hdd            = 2.5f * ((float)M_PI / 4.0f) * ((float)M_PI / 4.0f) * cosf(ph);
        }
        memset(&m, 0, sizeof(m));
        m.timestamp            = t;
        m.strapdown_dt_sec     = 0.01f;
        m.acc.is_valid         = true;
        m.gyr.is_valid         = true;
        m.acc.data[2]          = -(GRAVITY + hdd);
        m.local_pos.is_valid   = true;
        m.local_pos.pos_ned[2] = -h_true;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
        nav_suite_update(&s, &m);
    }
    float h;
    CHECK_TRUE(nav_suite_get_height(&s, &h), "ins height available");
    CHECK_NEAR(h, 5.0, 0.3, "ins climbed to 5 m");
    CHECK_TRUE(!nav_suite_get_baro_alt(&s, &h, (float*)0), "baro filter still waiting");

    /* First barometer sample: must anchor at the NED height (5 m). */
    for (i = 1; i <= 200; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp            = t;
        m.strapdown_dt_sec     = 0.01f;
        m.acc.is_valid         = true;
        m.gyr.is_valid         = true;
        m.acc.data[2]          = -GRAVITY;
        m.local_pos.is_valid   = true;
        m.local_pos.pos_ned[2] = -5.0f;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
        m.baro.is_valid                                                          = true;
        m.baro.pressure_pa = pressure_from_altitude(320.0f + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "baro running");
    CHECK_NEAR(h, 5.0, 0.4, "baro anchored at NED height, not 0");

    /* Aiding stops: after the 1 s coasting window nav_suite_get_height
       falls back to the baro filter, continuous and same datum. */
    for (i = 1; i <= 300; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(320.0f + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) != NAV_SUITE_MODE_FULL, "position aiding gone");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height still available");
    CHECK_NEAR(h, 5.0, 0.4, "fallback height continuous (same datum)");
}

/* ---------------------------------------------------------------------------
 * Scenario 15b: baro joins while ins is initialized but not yet ready
 *
 * IMU + local-position aiding (e.g. a lighthouse system, manual init),
 * no GNSS. The vehicle climbs 3 m during ins's readiness warm-up
 * (INS_MIN_RUNTIME_UNTIL_READY_MS) and the barometer only joins at
 * t = 1.0 s, i.e. after the climb has started but before ins reports
 * ready. The anchor must still land on ins's current local height:
 * gating it on readiness instead of initialization would anchor at 0
 * and leave a permanent datum offset that nothing later corrects
 * (the origin shift is already disarmed for a manual init).
 * ---------------------------------------------------------------------------
 */

static void scenario_baro_anchor_during_warmup(void)
{
    printf("\n-- scenario: baro anchor during the INSLIB warm-up --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = 48.783 * M_PI / 180.0;
    init.llh[1]                    = 9.181 * M_PI / 180.0;
    init.llh[2]                    = 300.0;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec      = 0.01f;
    opt.max_prediction_time_sec   = 0.5f;
    opt.magnetometer_min_delay_ms = 200;
    opt.max_deadreckoning_sec     = 10.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    /* 3 m cosine climb over the first 1.2 s, then hold. The barometer
       joins at 1.0 s, inside the 1.5 s readiness window. */
    ins_measurements_t m;
    int                i;
    bool               anchored_before_ready = false;
    for (i = 1; i <= 600; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts     = (float)i * 0.01f;
        float       h_true = 3.0f, hdd = 0.0f;
        if (ts <= 1.2f)
        {
            const float ph = (float)M_PI * ts / 1.2f;
            h_true         = 1.5f * (1.0f - cosf(ph));
            hdd            = 1.5f * ((float)M_PI / 1.2f) * ((float)M_PI / 1.2f) * cosf(ph);
        }
        memset(&m, 0, sizeof(m));
        m.timestamp            = t;
        m.strapdown_dt_sec     = 0.01f;
        m.acc.is_valid         = true;
        m.gyr.is_valid         = true;
        m.acc.data[2]          = -(GRAVITY + hdd);
        m.local_pos.is_valid   = true;
        m.local_pos.pos_ned[2] = -h_true;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
        if (ts >= 1.0f)
        {
            m.baro.is_valid    = true;
            m.baro.pressure_pa = pressure_from_altitude(300.0f + h_true);
        }
        nav_suite_update(&s, &m);
        /* Record that the anchor really did happen before readiness.
           Otherwise the scenario would silently stop testing anything. */
        if (s.baro_alt.is_initialized && !ins_is_ready(&s.ins)) { anchored_before_ready = true; }
    }
    CHECK_TRUE(anchored_before_ready, "baro anchored while ins was not ready yet");

    float h_baro, pos_ned[3];
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h_baro, (float*)0), "baro running");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins local position");
    CHECK_NEAR(-pos_ned[2], 3.0, 0.2, "ins tracked the climb");
    CHECK_NEAR(h_baro, 3.0, 0.3, "baro anchored in the ins datum, not at 0");
    CHECK_NEAR(h_baro - (-pos_ned[2]), 0.0, 0.3, "both filters share one datum");
}

/* Shared ins init/options for the source-permutation scenarios below:
 * auto-init, origin ellipsoid height h_ell_org at the Stuttgart test
 * coordinates. */
static void permutation_setup(ins_init_t* init, ins_options_t* opt, ins_time_us_t t,
                              double h_ell_org)
{
    memset(init, 0, sizeof(*init));
    init->time                      = t;
    init->llh[0]                    = 48.783 * M_PI / 180.0;
    init->llh[1]                    = 9.181 * M_PI / 180.0;
    init->llh[2]                    = h_ell_org;
    init->pos_init_stddev_m         = 1.0f;
    init->vel_init_stddev_mps       = 0.1f;
    init->rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init->rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init->acc_bias_init_stddev_mps2 = 0.05f;
    init->gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init->pos_pred_stddev_m_sqrts   = 0.01f;
    init->vel_pred_stddev_mps_sqrts = 0.05f;
    init->rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init->zero_vel_stddev_mps       = 0.01f;
    init->zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init->magnetic_n[0]             = 20.0f;
    init->magnetic_n[2]             = 44.0f;

    memset(opt, 0, sizeof(*opt));
    opt->kalman_update_dt_sec               = 0.01f;
    opt->max_prediction_time_sec            = 0.5f;
    opt->gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt->gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt->gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt->gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt->magnetometer_min_delay_ms          = 200;
    opt->auto_init                          = true;
    opt->max_deadreckoning_sec              = 30.0f;
    /* These scenarios test datum arbitration, not stillness detection.
       The synthetic climbs below are gentle enough (|f| - g stays well
       inside the static gate, gyro is exactly 0) to look stationary to
       the auto-ZUPT detector, which would then pin the vertical velocity
       to zero and make ins lag the climb: an artefact of noise-free
       test data, not of the datum logic under test. */
    opt->auto_zupt_disable = true;
}

/* ---------------------------------------------------------------------------
 * Scenario 15c: permutation GNSS -> baro (auto-init)
 *
 * The fix comes first and ins owns the origin; the barometer only joins
 * after a 6 m climb. It must anchor into the ins datum rather than at
 * its own start, so both heights agree and the offset filter (which
 * switches to the baro as its reference the moment it appears) keeps
 * reporting the origin's ellipsoid height across that switch.
 * ---------------------------------------------------------------------------
 */

static void scenario_datum_gnss_then_baro(void)
{
    printf("\n-- scenario: permutation GNSS -> baro --\n");

    const double lat = 48.783 * M_PI / 180.0;
    const double lon = 9.181 * M_PI / 180.0;
    /* Static at the origin, then a 6 m climb. The origin is where the
       vehicle starts, so the offset must converge to h_ell_org. */
    const float h_ell_org = 380.0f;
    const float isa_start = 300.0f;

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, (double)h_ell_org);

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h_true = 0.0f, h_baro, h_ell, off, pos_ned[3];
    int                i;

    for (i = 1; i <= 6000; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts  = (float)i * 0.01f;
        float       hdd = 0.0f;
        /* 0-20 s static (ins bootstraps, baro still absent), 20-30 s a
           6 m climb, then hold. The baro joins at 35 s, well after the
           climb, so anchoring at 0 would be off by 6 m. */
        if (ts > 20.0f && ts <= 30.0f)
        {
            const float ph = (float)M_PI * (ts - 20.0f) / 10.0f;
            h_true         = 3.0f * (1.0f - cosf(ph));
            hdd            = 3.0f * ((float)M_PI / 10.0f) * ((float)M_PI / 10.0f) * cosf(ph);
        }
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -(GRAVITY + hdd);
        if (ts >= 35.0f)
        {
            m.baro.is_valid    = true;
            m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        }
        if ((i % 100) == 1)
        {
            double e[3];
            ins_latlonh_to_ecef(lat, lon, (double)(h_ell_org + h_true), e);
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(e, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1], &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 2.25f;
        }
        nav_suite_update(&s, &m);
        if (i == 3000) { CHECK_TRUE(!s.baro_alt.is_initialized, "baro still absent at 30 s"); }
    }

    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h_baro, (float*)0), "baro running");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins local position");
    CHECK_NEAR(-pos_ned[2], 6.0, 0.7, "ins tracked the climb");
    CHECK_NEAR(h_baro, 6.0, 0.7, "baro anchored into the ins datum, not at 0");
    CHECK_NEAR(h_baro - (-pos_ned[2]), 0.0, 0.5, "both filters share one datum");
    CHECK_TRUE(local_gnss_alt_get(&s.local_gnss, &off, (float*)0), "offset filter running");
    CHECK_NEAR(off, h_ell_org, 1.5, "offset survives the reference switch to the baro");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height");
    CHECK_NEAR(h_ell, h_ell_org + 6.0, 1.5, "ellipsoid height correct");
}

/* ---------------------------------------------------------------------------
 * Scenario 15e: the origin shift must not drag the offset filter
 *
 * GNSS and baro both run from the start, but the vehicle is never
 * quasi-static, so ins's auto-init gate keeps rejecting. Meanwhile the
 * GNSS/baro pairs already initialize the offset filter against the BARO
 * datum. The vehicle then goes still and ins finally bootstraps, which
 * fires the origin shift.
 *
 * That shift moves ins ONTO the baro datum; the baro (and therefore the
 * offset filter, which is fed from it) does not move. Shifting the offset
 * along with the origin would put a permanent error of dz on every
 * absolute height. This is the only reachable case where the shift is not
 * a no-op, which is why it needs its own scenario.
 * ---------------------------------------------------------------------------
 */

static void scenario_datum_offset_survives_origin_shift(void)
{
    printf("\n-- scenario: origin shift leaves the offset filter alone --\n");

    const double lat       = 48.783 * M_PI / 180.0;
    const double lon       = 9.181 * M_PI / 180.0;
    const float  h_ell_org = 380.0f; /* mission start = the baro datum */
    const float  isa_start = 300.0f;

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, (double)h_ell_org);

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h_true = 0.0f, off_before_shift = 0.0f, off, h_ell;
    int                i;
    bool               offset_ran_before_bootstrap = false;

    for (i = 1; i <= 6000; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts  = (float)i * 0.01f;
        float       hdd = 0.0f;
        if (ts > 5.0f && ts <= 25.0f)
        {
            const float ph = (float)M_PI * (ts - 5.0f) / 20.0f;
            h_true         = 5.0f * (1.0f - cosf(ph));
            hdd            = 5.0f * ((float)M_PI / 20.0f) * ((float)M_PI / 20.0f) * cosf(ph);
        }
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -(GRAVITY + hdd);
        /* Vibrate until 40 s: keeps ins's quasi-static auto-init gate
           from ever passing while the offset filter happily converges. */
        if (ts <= 40.0f) { m.gyr.data[0] = ((i % 2) ? 0.5f : -0.5f); }
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        if ((i % 100) == 1)
        {
            double e[3];
            ins_latlonh_to_ecef(lat, lon, (double)(h_ell_org + h_true), e);
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(e, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1], &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 2.25f;
        }

        const bool was_init = s.ins.is_initialized;
        float      o_pre    = 0.0f;
        const bool had_off  = local_gnss_alt_get(&s.local_gnss, &o_pre, (float*)0);
        nav_suite_update(&s, &m);
        if (!was_init && s.ins.is_initialized && had_off)
        {
            offset_ran_before_bootstrap = true;
            off_before_shift            = o_pre;
        }
    }

    /* Without this the scenario would prove nothing: the shift is only
       interesting while the offset filter is already running. */
    CHECK_TRUE(offset_ran_before_bootstrap, "offset filter ran before the ins bootstrap");
    CHECK_NEAR(off_before_shift, h_ell_org, 1.5, "offset was already correct before the shift");

    CHECK_TRUE(local_gnss_alt_get(&s.local_gnss, &off, (float*)0), "offset filter still running");
    CHECK_NEAR(off, h_ell_org, 1.5, "origin shift did not drag the offset");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height");
    CHECK_NEAR(h_ell, h_ell_org + 10.0, 2.0, "ellipsoid height correct after the shift");
}

/* ---------------------------------------------------------------------------
 * Scenario 15f: a quality-loss re-arm must not move the datum
 *
 * The tunnel case. GNSS and baro run from the start, ins bootstraps and is
 * aligned onto the baro datum. The fixes then degrade for longer than the
 * exit dwell, ins re-arms (REQ-NAV-052) and carries its n-frame origin into
 * the next bootstrap (REQ-NAV-062). Meanwhile the barometer has drifted
 * against the fix height, as it does over a long drive.
 *
 * The wrapper must NOT re-align on that re-bootstrap: ins never left the
 * datum, so the only thing an alignment could do is push the accumulated
 * baro-vs-fix disagreement into the origin -- relocating every local
 * coordinate the consumer holds, which is exactly what the origin carry
 * exists to prevent. A re-arm that does anchor a NEW origin (here: the
 * health-check path, which inherits nothing) must still re-join the datum,
 * so the second half of the scenario covers that direction too.
 * ---------------------------------------------------------------------------
 */

/* One epoch of the scenario above: a platform standing at H_M above the
 * datum, its barometer reading BARO_H_M (the two differ once the barometer
 * has drifted), and a 1 Hz fix at H_M whose velocity 1-sigma is VSTD -- the
 * only quality knob, the position stays inside every gate, so the exit is
 * unambiguously the quality decision and not a rejected fix. WITH_FIX false
 * feeds no fix at all (a plain outage). */
#define QEXIT_STEP(S, T, I, H_M, BARO_H_M, WITH_FIX, VSTD)                                         \
    do {                                                                                           \
        ins_measurements_t mm;                                                                     \
        memset(&mm, 0, sizeof(mm));                                                                \
        (T) += US_PER_SEC / 100;                                                                   \
        mm.timestamp        = (T);                                                                 \
        mm.strapdown_dt_sec = 0.01f;                                                               \
        mm.acc.is_valid     = true;                                                                \
        mm.gyr.is_valid     = true;                                                                \
        mm.acc.data[2]      = -GRAVITY;                                                            \
        mm.baro.is_valid    = true;                                                                \
        mm.baro.pressure_pa = pressure_from_altitude(qexit_isa_start + (BARO_H_M) + gauss(0.2f));  \
        if ((WITH_FIX) && (((I) % 100) == 1))                                                      \
        {                                                                                          \
            double e[3];                                                                           \
            ins_latlonh_to_ecef(qexit_lat, qexit_lon, (double)(qexit_h_ell_org + (H_M)), e);       \
            mm.gnss_pos.is_valid = true;                                                           \
            ins_ecef_to_latlonh(e, &mm.gnss_pos.llh[0], &mm.gnss_pos.llh[1], &mm.gnss_pos.llh[2]); \
            mm.gnss_pos.Qll_ned[0] = 1.0f;                                                         \
            mm.gnss_pos.Qll_ned[4] = 1.0f;                                                         \
            mm.gnss_pos.Qll_ned[8] = 2.25f;                                                        \
            mm.gnss_vel.is_valid   = true;                                                         \
            mm.gnss_vel.Qll_ned[0] = (VSTD) * (VSTD);                                              \
            mm.gnss_vel.Qll_ned[4] = (VSTD) * (VSTD);                                              \
            mm.gnss_vel.Qll_ned[8] = (VSTD) * (VSTD);                                              \
        }                                                                                          \
        nav_suite_update(&(S), &mm);                                                               \
    } while (0)

static const double qexit_lat       = 48.783 * M_PI / 180.0;
static const double qexit_lon       = 9.181 * M_PI / 180.0;
static const float  qexit_h_ell_org = 380.0f; /* mission start = the datum */
static const float  qexit_isa_start = 300.0f;

static void scenario_datum_survives_quality_exit(void)
{
    printf("\n-- scenario: quality-loss re-arm keeps the vertical datum --\n");

    const float baro_drift_m = 10.0f; /* barometer walks off while ins is down */
    const float climb_m      = 30.0f; /* height gained during the health outage */

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, (double)qexit_h_ell_org);
    opt.gnss_init_dwell_disable = true; /* re-bootstrap promptly, as after a tunnel */
    opt.gnss_stop_dwell_sec     = 5.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    int   i = 0;
    float h_baro, pos_ned[3];

    /* Phase A: everything nominal for 20 s. ins bootstraps and is aligned
       onto the barometric datum. */
    for (; i < 2000; ++i) { QEXIT_STEP(s, t, i, 0.0f, 0.0f, true, 0.1f); }
    CHECK_TRUE(s.ins.is_initialized, "ins bootstrapped on the good fixes");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins local position");
    const float h_local_before = -pos_ned[2];
    double      datum_origin[3];
    datum_origin[0] = s.ins.origin_llh[0];
    datum_origin[1] = s.ins.origin_llh[1];
    datum_origin[2] = s.ins.origin_llh[2];

    /* Phase B: the fixes keep arriving but their velocity quality collapses,
       which is what drives ins out of the 3D solution. */
    for (; i < 3500 && s.ins.is_initialized; ++i) { QEXIT_STEP(s, t, i, 0.0f, 0.0f, true, 5.0f); }
    CHECK_TRUE(!s.ins.is_initialized, "the degraded fixes drove ins out of the 3D solution");
    CHECK_TRUE(s.ins.origin_carry.valid, "the quality exit armed an origin carry");

    /* Phase C: ins is down. The barometer drifts off the datum (weather, a
       tunnel's own pressure), the platform does not move. */
    const int drift_end = i + 600;
    for (; i < drift_end; ++i)
    {
        const float drift = baro_drift_m * (float)(drift_end - i) / 600.0f;
        QEXIT_STEP(s, t, i, 0.0f, baro_drift_m - drift, false, 0.1f);
    }
    CHECK_TRUE(!s.ins.is_initialized, "ins is still collecting without fixes");

    /* Phase D: the fixes come back. ins re-bootstraps on the carried origin,
       and the wrapper must leave that origin alone. */
    const int reboot_end = i + 1500;
    for (; i < reboot_end; ++i) { QEXIT_STEP(s, t, i, 0.0f, baro_drift_m, true, 0.1f); }
    CHECK_TRUE(s.ins.is_initialized, "ins re-bootstrapped on the recovered fixes");

    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h_baro, (float*)0), "baro running");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins local position");
    /* Without this the scenario would prove nothing: an alignment is only
       observable while the two sources actually disagree, and this is the
       disagreement the old "re-align on every reset" rule would have pushed
       into the origin. */
    CHECK_TRUE(fabsf(h_baro) > 2.0f, "the barometer drifted off the datum while ins was down");

    /* Metres, not radians: a 1e-3 tolerance on a latitude would be 6 km. */
    CHECK_NEAR(s.ins.origin_llh[0] * INS_WGS84_A, datum_origin[0] * INS_WGS84_A, 1e-3,
               "datum origin latitude kept across the re-arm");
    CHECK_NEAR(s.ins.origin_llh[1] * INS_WGS84_A, datum_origin[1] * INS_WGS84_A, 1e-3,
               "datum origin longitude kept across the re-arm");
    CHECK_NEAR(s.ins.origin_llh[2], datum_origin[2], 1e-3,
               "datum origin height kept across the re-arm");
    /* The platform never moved, so the local height a consumer reads has to
       be the one it read before the outage. */
    CHECK_NEAR(-pos_ned[2], h_local_before, 1.0, "local height is continuous across the re-arm");

    /* Phase E: the other direction. A health-check re-arm inherits no origin
       (REQ-NAV-062), so its bootstrap anchors a fresh one -- here 30 m above
       the datum, because the platform climbed while ins was down. That origin
       is NOT the datum and must be shifted back onto it, or the local height
       would restart at zero 30 m up. */
    s.ins.d[INS_IDX_VEL] = NAN; /* trip the health check */
    {
        /* Same timestamp as the last epoch, so the prediction step is skipped
           and the poke reaches the health check instead of being overwritten. */
        ins_measurements_t mp;
        memset(&mp, 0, sizeof(mp));
        mp.timestamp        = t;
        mp.strapdown_dt_sec = 0.01f;
        mp.acc.is_valid     = true;
        mp.gyr.is_valid     = true;
        mp.acc.data[2]      = -GRAVITY;
        nav_suite_update(&s, &mp);
    }
    CHECK_TRUE(!s.ins.is_initialized, "the health check re-armed ins into collecting");
    CHECK_TRUE(!s.ins.origin_carry.valid, "a health re-arm arms no origin carry");

    const int climb_end = i + 1500;
    for (; i < climb_end; ++i)
    {
        /* No fixes: ins stays collecting long enough for the barometer to
           follow the climb, so the alignment has a settled height to work
           against. */
        const float f_up = (float)(i - climb_end + 1500) / 1000.0f;
        const float h    = climb_m * ((f_up < 1.0f) ? f_up : 1.0f);
        QEXIT_STEP(s, t, i, h, baro_drift_m + h, false, 0.1f);
    }

    /* Sampled at the bootstrap epoch itself: the alignment reconciles the two
       heights at that instant, and the barometer is still catching up with
       the climb afterwards, so a later comparison would measure the
       barometer's settling rather than the alignment. */
    float     h_baro_at_boot = 0.0f, h_local_at_boot = 0.0f;
    bool      rejoined_seen = false;
    const int settle_end    = i + 1500;
    for (; i < settle_end; ++i)
    {
        const bool was_init = s.ins.is_initialized;
        QEXIT_STEP(s, t, i, climb_m, baro_drift_m + climb_m, true, 0.1f);
        if (!was_init && s.ins.is_initialized)
        {
            rejoined_seen = nav_suite_get_baro_alt(&s, &h_baro_at_boot, (float*)0) &&
                            ins_get_position_local(&s.ins, pos_ned);
            h_local_at_boot = -pos_ned[2];
        }
    }
    CHECK_TRUE(rejoined_seen, "ins re-bootstrapped after the health re-arm");
    CHECK_TRUE(h_baro_at_boot > climb_m - 5.0f, "the barometer followed the climb");
    CHECK_NEAR(h_local_at_boot, h_baro_at_boot, 1.0, "a fresh origin re-joins the baro datum");
}
#undef QEXIT_STEP

/* ---------------------------------------------------------------------------
 * Scenario 15d: permutation baro -> lighthouse (auto-init)
 *
 * The barometer runs from the start (datum = mission start, h = 0). The
 * vehicle climbs 2 m on the baro, THEN a lighthouse appears whose frame
 * origin is 5 m BELOW the start point (so it reports the 2 m-high vehicle
 * at 2 + 5 = 7 m in its own frame), and auto-init bootstraps ins from it.
 *
 * The external system owns the n-frame, so ins is never shifted onto the
 * baro datum (that would desync the frame and get every local_pos
 * rejected). Instead the lighthouse is offset onto the existing datum:
 * ins bootstraps at (0,0,h) with h = the current baro height (2 m), the
 * local height does NOT jump, and the lighthouse-frame offset is exposed
 * so a caller can convert between the two frames. No GNSS -> no ellipsoid.
 * ---------------------------------------------------------------------------
 */

static void scenario_datum_baro_then_lighthouse(void)
{
    printf("\n-- scenario: permutation baro -> lighthouse (no jump) --\n");

    const float h_ell_org  = 300.0f; /* ellipsoid height of the start point */
    const float frame_drop = 5.0f;   /* lighthouse origin sits this far below start */

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, (double)h_ell_org);

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h_true = 0.0f, h_at_handover = -1.0f;
    int                i;

    /* 0-2 s static, 2-7 s a smooth 2 m climb on the baro, then hold. The
       lighthouse only appears at 12 s, well after the climb, so ins must
       bootstrap at h = 2, not 0. */
    for (i = 1; i <= 2500; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts  = (float)i * 0.01f;
        float       hdd = 0.0f;
        if (ts > 2.0f && ts <= 7.0f)
        {
            const float ph = (float)M_PI * (ts - 2.0f) / 5.0f;
            h_true         = 1.0f * (1.0f - cosf(ph));
            hdd            = 1.0f * ((float)M_PI / 5.0f) * ((float)M_PI / 5.0f) * cosf(ph);
        }
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -(GRAVITY + hdd);
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(h_ell_org + h_true + gauss(0.05f));
        if (ts >= 12.0f)
        {
            /* Record the baro height just before the lighthouse joins, to
               prove the local height does not jump at handover. */
            if (h_at_handover < 0.0f) { (void)nav_suite_get_height(&s, &h_at_handover); }
            m.local_pos.is_valid   = true;
            m.local_pos.pos_ned[2] = -(h_true + frame_drop);
            m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
        }
        nav_suite_update(&s, &m);
        if (i == 1100) { CHECK_TRUE(!s.ins.is_initialized, "ins waits for the first local fix"); }
    }

    float h_baro, pos_ned[3], h, off[3], h_ell;
    CHECK_TRUE(s.ins.is_initialized, "ins bootstrapped from the local fix");
    CHECK_TRUE(s.local_frame_external, "external n-frame owner latched");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins local position");

    /* The core property: no jump. The local height was ~2 from the baro
       and stays ~2 once ins takes over: ins bootstrapped at (0,0,h),
       h = the baro height, NOT at the lighthouse's own zero. */
    CHECK_NEAR(h_at_handover, 2.0, 0.2, "baro height just before handover");
    CHECK_NEAR(-pos_ned[2], 2.0, 0.2, "ins conforms to the baro datum (0,0,h), not the frame zero");
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL on live local aiding");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height available");
    CHECK_NEAR(h, 2.0, 0.2, "local height continuous across the handover");

    /* The baro was NOT moved: it set the datum and keeps it. */
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h_baro, (float*)0), "baro running");
    CHECK_NEAR(h_baro, 2.0, 0.2, "baro keeps the datum it established");

    /* The exposed offset maps the lighthouse frame onto the datum: the
       frame origin sits 5 m below the datum, so its coords of the datum
       origin are (0,0,-5) [NED down]. external = local + offset. */
    CHECK_TRUE(nav_suite_get_local_pos_offset(&s, off), "local_pos offset available");
    CHECK_NEAR(off[2], -frame_drop, 0.1, "offset[down] = frame origin below the datum");
    CHECK_NEAR(off[0], 0.0, 0.1, "offset[north] zeroed");
    CHECK_NEAR(off[1], 0.0, 0.1, "offset[east] zeroed");

    /* No GNSS ever -> no ellipsoid height and no WGS84 conversion, even
       though ins is initialized and FULL: the n-frame origin rests only on
       the prescribed init position, not a real WGS84 anchor. */
    CHECK_TRUE(!nav_suite_get_height_ellipsoid(&s, &h_ell), "no ellipsoid without GNSS");
    double      lt, ln, hh;
    const float probe_ned[3] = {0.0f, 0.0f, 0.0f};
    CHECK_TRUE(!nav_suite_local_to_wgs84(&s, probe_ned, &lt, &ln, &hh),
               "no WGS84 conversion without a GNSS anchor");
}

/* ---------------------------------------------------------------------------
 * Scenario 16: full height strategy (auto-init, baro first, GNSS later)
 *
 * The barometer starts at the mission start (datum h = 0), the vehicle
 * climbs 10 m, then the first GNSS fix arrives: ins bootstraps and
 * its origin is shifted so the local height matches the baro height
 * (datum = mission start), while the absolute height stays on the fix.
 * Baro/GNSS pairs feed the offset filter; when GNSS drops out, both
 * height accessors keep working from the barometer.
 * ---------------------------------------------------------------------------
 */

static void scenario_height_strategy(void)
{
    printf("\n-- scenario: height strategy (auto-init + offset) --\n");

    const double lat       = 48.783 * M_PI / 180.0;
    const double lon       = 9.181 * M_PI / 180.0;
    const float  isa_start = 300.0f; /* barometric ISA altitude at start */
    const float  h_ell_fix = 390.0f; /* ellipsoid height at fix (10 m up) */
    const float  h_ell_org = 380.0f; /* ellipsoid height of the start point */
    /* The offset is the ellipsoid height of the datum origin (the start
       point), NOT the baro-to-ISA offset: 390 - 10 = 380 m. The datum
       here is the mission start (baro ran first), so it equals the
       ellipsoid height the filter is initialized at below. */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = lat;
    init.llh[1]                    = lon;
    init.llh[2]                    = (double)h_ell_org;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    opt.auto_init                          = true;
    opt.max_deadreckoning_sec              = 2.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h, h_ell;
    int                i;

    /* Phase A+B: no GNSS. 3 s static, then a smooth 10 m cosine climb
       over 10 s (the IMU sees the matching specific force), 4 s hold
       at +10 m. The baro filter runs on the ARS attitude with
       datum = start. */
    for (i = 1; i <= 1700; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts     = (float)i * 0.01f;
        float       h_true = 0.0f, hdd = 0.0f;
        if (ts > 3.0f && ts <= 13.0f)
        {
            const float ph = (float)M_PI * (ts - 3.0f) / 10.0f;
            h_true         = 5.0f * (1.0f - cosf(ph));
            hdd            = 5.0f * ((float)M_PI / 10.0f) * ((float)M_PI / 10.0f) * cosf(ph);
        }
        else if (ts > 13.0f) { h_true = 10.0f; }
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -(GRAVITY + hdd);
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "baro running");
    CHECK_NEAR(h, 10.0, 0.5, "baro tracked the climb (datum = start)");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height from baro");
    CHECK_NEAR(h, 10.0, 0.5, "height accessor uses baro");
    CHECK_TRUE(!nav_suite_get_height_ellipsoid(&s, &h_ell),
               "no absolute height yet (no GNSS seen)");

    /* Phase C: first GNSS fix (1 Hz, at the current position). ins
       auto-initializes; the origin shift must land the local height on
       the baro height while the absolute height stays on the fix. */
    double fix_ecef[3];
    ins_latlonh_to_ecef(lat, lon, (double)h_ell_fix, fix_ecef);
    for (i = 1; i <= 1000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + 10.0f + gauss(0.2f));
        if ((i % 100) == 1)
        {
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(fix_ecef, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1],
                                &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 2.25f; /* 1.5 m vertical */
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height from INSLIB");
    CHECK_NEAR(h, 10.0, 0.7, "origin shifted: local height = baro height");
    CHECK_NEAR(s.ins.latlonh[2], h_ell_fix, 0.7, "absolute height stays on the fix");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height");
    CHECK_NEAR(h_ell, h_ell_fix, 1.0, "ellipsoid height from INSLIB");
    float off;
    CHECK_TRUE(local_gnss_alt_get(&s.local_gnss, &off, (float*)0), "offset filter running");
    CHECK_NEAR(off, h_ell_org, 1.5, "offset = ellipsoid height of the datum origin");

    /* Phase D: GNSS outage. Height and absolute height stay available
       and continuous from the barometer (+ offset). */
    for (i = 1; i <= 800; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + 10.0f + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) != NAV_SUITE_MODE_FULL, "GNSS gone");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height survives the outage");
    CHECK_NEAR(h, 10.0, 0.7, "height continuous across the source switch");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height survives the outage");
    CHECK_NEAR(h_ell, h_ell_fix, 2.0, "ellipsoid height from baro + offset");
}

/* ---------------------------------------------------------------------------
 * Scenario 16a2: a drifting barometer must not drag the ABSOLUTE height
 * with it while GNSS is streaming (REQ-SUITE-008)
 *
 * Under the barometric height source ins's vertical datum is anchored once
 * at bootstrap and never re-tied to GNSS (REQ-NAV-055), so the barometer's
 * absolute error - weather and ISA model - accumulates in ins's absolute
 * height for the rest of the run. Its height RELATIVE to that datum stays
 * excellent, which is why it is still what nav_suite_get_height() reports;
 * what goes stale is only where the datum sits, and the offset filter is
 * the thing that knows. So the absolute accessor takes the local reference
 * plus the offset here rather than ins's raw absolute height.
 * ---------------------------------------------------------------------------
 */

static void scenario_height_ellipsoid_baro_drift(void)
{
    printf("\n-- scenario: absolute height ignores barometric datum drift --\n");

    const double lat       = 48.783 * M_PI / 180.0;
    const double lon       = 9.181 * M_PI / 180.0;
    const float  isa_start = 300.0f;
    const float  h_ell_fix = 380.0f;

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = lat;
    init.llh[1]                    = lon;
    init.llh[2]                    = (double)h_ell_fix;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.auto_init                          = true;
    opt.max_deadreckoning_sec              = 2.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    double fix_ecef[3];
    ins_latlonh_to_ecef(lat, lon, (double)h_ell_fix, fix_ecef);

    /* drift_m: how far the barometer's own altitude has walked away from
       the truth by epoch i. The platform never moves. */
#define DRIFT_EPOCH(I, DRIFT_M)                                                           \
    do {                                                                                  \
        t += US_PER_SEC / 100;                                                            \
        ins_measurements_t m;                                                             \
        memset(&m, 0, sizeof(m));                                                         \
        m.timestamp        = t;                                                           \
        m.strapdown_dt_sec = 0.01f;                                                       \
        m.acc.is_valid     = true;                                                        \
        m.gyr.is_valid     = true;                                                        \
        m.acc.data[2]      = -GRAVITY;                                                    \
        m.baro.is_valid    = true;                                                        \
        m.baro.pressure_pa = pressure_from_altitude(isa_start + (DRIFT_M) + gauss(0.2f)); \
        if (((I) % 100) == 1)                                                             \
        {                                                                                 \
            m.gnss_pos.is_valid = true;                                                   \
            ins_ecef_to_latlonh(fix_ecef, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1],         \
                                &m.gnss_pos.llh[2]);                                      \
            m.gnss_pos.Qll_ned[0] = 1.0f;                                                 \
            m.gnss_pos.Qll_ned[4] = 1.0f;                                                 \
            m.gnss_pos.Qll_ned[8] = 2.25f;                                                \
        }                                                                                 \
        nav_suite_update(&s, &m);                                                         \
    } while (0)

    /* Bootstrap with the barometer streaming, so ins latches the
       barometric height source (REQ-NAV-053) - the precondition for any
       of this to be reachable. */
    int i;
    for (i = 1; i <= 3000; ++i) { DRIFT_EPOCH(i, 0.0f); }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode");
    CHECK_TRUE(s.ins.height_from_baro, "ins runs on the barometric height source");

    float h_ell;
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height available");
    CHECK_NEAR(h_ell, h_ell_fix, 1.5, "absolute height starts on the fix");

    /* 20 m of barometric drift over 600 s, GNSS unchanged throughout.
       Weather alone does not move this fast; an ISA-model error against a
       few hundred metres of terrain does (REQ-BARO-011). */
    const float drift_total = 20.0f;
    for (i = 1; i <= 60000; ++i) { DRIFT_EPOCH(i, drift_total * (float)i / 60000.0f); }

    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "still FULL");

    /* The mechanism: ins DID follow the barometer, by design. If this
       assertion ever stops holding the drift stopped reaching ins and the
       one below proves nothing. */
    const float ins_err = (float)s.ins.latlonh[2] - h_ell_fix;
    CHECK_TRUE(ins_err > 0.5f * drift_total, "ins absolute height carried the barometric drift");

    /* The behaviour under test: the absolute accessor did not follow it far.
       Not "did not follow it at all" - two parts of the drift are given up
       by construction and neither is a defect. The offset filter lags a ramp
       by its own time constant, and the correction deliberately keeps back a
       sigma of the drift estimate so that a mission WITHOUT drift is reported
       from ins untouched instead of from a correction indistinguishable from
       noise. What must hold is that most of the drift is gone. */
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height still available");
    CHECK_TRUE(fabsf(h_ell - h_ell_fix) < 0.5f * ins_err,
               "the datum drift correction removed most of what ins carried");
    CHECK_NEAR(h_ell, h_ell_fix, 10.0, "absolute height stayed near the fix");

    /* The relative accessor is untouched: it reports the local height,
       which is what the barometer is good at. */
    float h;
    CHECK_TRUE(nav_suite_get_height(&s, &h), "local height available");

#undef DRIFT_EPOCH
}

/* ---------------------------------------------------------------------------
 * Scenario 16a3: height survives a noisy GNSS entering, dropping and
 * returning (REQ-SUITE-007)
 *
 * The wrapper bootstraps on the barometer alone (no GNSS at all yet), then a
 * GNSS fix arrives whose horizontal component is good but whose implied
 * ellipsoid height is noisy well beyond what its own reported covariance
 * claims (a receiver sitting in a bad multipath spot that still reports a
 * plausible VDOP). Once barometric height is selected at bootstrap
 * (REQ-NAV-053), GNSS's vertical row is dropped outright (REQ-NAV-055), so
 * the spin cannot reach nav_suite_get_height() at all - this drives that
 * guarantee with an actually noisy signal sustained over many fixes instead
 * of the ins-level scenarios' single one-shot glitch, and adds the full
 * arrive/drop/return cycle at the wrapper level, which none of those cover.
 * ---------------------------------------------------------------------------
 */

static void scenario_height_gnss_noise_outage_cycle(void)
{
    printf("\n-- scenario: height survives noisy GNSS entering, dropping and "
           "returning --\n");

    const double lat         = 48.783 * M_PI / 180.0;
    const double lon         = 9.181 * M_PI / 180.0;
    const float  isa_start   = 300.0f;
    const float  h_true      = 5.0f;   /* platform holds here once it climbs */
    const float  h_ell_org   = 380.0f; /* ellipsoid height of the datum origin (0 m local) */
    const float  spin_stddev = 6.0f;   /* GNSS altitude noise, well beyond its reported accuracy */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = lat;
    init.llh[1]                    = lon;
    init.llh[2]                    = (double)h_ell_org;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    opt.auto_init                          = true;
    opt.max_deadreckoning_sec              = 2.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");
    /* Constant-rate climb then a dead-still hold: opt out of every
       velocity-blind stillness fallback, see scenario_height_strategy. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    float              h, h_prev;
    int                i;
    float              max_step = 0.0f;

    /* Phase A: baro-only bootstrap, no GNSS at all. Static, then a 5 m
       climb, then hold - the platform stays at h_true for the rest of the
       test so any later height motion can only come from a source switch. */
    for (i = 1; i <= 1200; ++i)
    {
        t += US_PER_SEC / 100;
        const float ts    = (float)i * 0.01f;
        float       h_now = 0.0f, hdd = 0.0f;
        if (ts > 2.0f && ts <= 7.0f)
        {
            const float ph = (float)M_PI * (ts - 2.0f) / 5.0f;
            h_now          = 0.5f * h_true * (1.0f - cosf(ph));
            hdd            = 0.5f * h_true * ((float)M_PI / 5.0f) * ((float)M_PI / 5.0f) * cosf(ph);
        }
        else if (ts > 7.0f) { h_now = h_true; }
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -(GRAVITY + hdd);
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_now + gauss(0.2f));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(!s.ins.is_initialized, "no GNSS yet: ins still collecting");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height from baro alone");
    CHECK_NEAR(h, h_true, 0.5, "baro tracked the climb before any GNSS");
    h_prev = h;

    /* Phase B: the first GNSS fix arrives, and every fix after it has a
       badly noisy implied height (spin_stddev, ~6 m). Track the height
       across the bootstrap epoch itself and the running per-epoch step. */
    double fix_ecef[3];
    float  h_before_boot = 0.0f, h_after_boot = 0.0f;
    bool   boot_seen = false;
    for (i = 1; i <= 2000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        if ((i % 100) == 1) /* 1 Hz */
        {
            const double h_ell_noisy =
                (double)h_ell_org + (double)h_true + (double)gauss(spin_stddev);
            ins_latlonh_to_ecef(lat, lon, h_ell_noisy, fix_ecef);
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(fix_ecef, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1],
                                &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 0.25f;
            m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8] = 4.0f; /* 2 m vertical - far tighter than the actual spin */
        }
        const bool was_init = s.ins.is_initialized;
        nav_suite_update(&s, &m);
        if (nav_suite_get_height(&s, &h))
        {
            const float step = fabsf(h - h_prev);
            if (step > max_step) max_step = step;
            if (!boot_seen && !was_init && s.ins.is_initialized)
            {
                h_before_boot = h_prev;
                h_after_boot  = h;
                boot_seen     = true;
            }
            h_prev = h;
        }
    }
    CHECK_TRUE(boot_seen, "ins bootstrapped on the first (noisy) GNSS fix");
    CHECK_NEAR(h_after_boot, h_before_boot, 0.5, "no jump at the moment GNSS first arrives");
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode reached");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height available");
    CHECK_NEAR(h, h_true, 0.7, "height still tracks the true (baro) height despite the GNSS spin");

    /* Phase C: GNSS gone past max_deadreckoning_sec. The barometer keeps
       the height going; nothing about a source that is simply absent
       should move it either. */
    for (i = 1; i <= 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        nav_suite_update(&s, &m);
        if (nav_suite_get_height(&s, &h))
        {
            const float step = fabsf(h - h_prev);
            if (step > max_step) max_step = step;
            h_prev = h;
        }
    }
    CHECK_TRUE(nav_suite_get_mode(&s) != NAV_SUITE_MODE_FULL, "GNSS gone");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height survives the outage");
    CHECK_NEAR(h, h_true, 0.7, "height continuous through the outage");

    /* Phase D: GNSS returns, spinning again, and re-acquires ins. The
       re-acquisition edge (REQ-NAV-066) is the interesting instant: ins's
       own frozen height snaps onto the barometer it was already reading,
       not onto the fix's noisy implied height. */
    uint32_t reacq_before   = ins_get_diag(&s.ins)->n_reacquire;
    bool     reacq_seen     = false;
    float    h_before_reacq = 0.0f, h_after_reacq = 0.0f;
    for (i = 1; i <= 2000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_from_altitude(isa_start + h_true + gauss(0.2f));
        if ((i % 100) == 1) /* 1 Hz */
        {
            const double h_ell_noisy =
                (double)h_ell_org + (double)h_true + (double)gauss(spin_stddev);
            ins_latlonh_to_ecef(lat, lon, h_ell_noisy, fix_ecef);
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(fix_ecef, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1],
                                &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 0.25f;
            m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8] = 4.0f;
        }
        nav_suite_update(&s, &m);
        if (nav_suite_get_height(&s, &h))
        {
            const float step = fabsf(h - h_prev);
            if (step > max_step) max_step = step;
            if (!reacq_seen && ins_get_diag(&s.ins)->n_reacquire > reacq_before)
            {
                h_before_reacq = h_prev;
                h_after_reacq  = h;
                reacq_seen     = true;
            }
            h_prev = h;
        }
    }
    CHECK_TRUE(reacq_seen, "the returning (noisy) fix re-acquired ins");
    CHECK_NEAR(h_after_reacq, h_before_reacq, 0.5, "no jump when GNSS returns");
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode restored");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height available after the return");
    CHECK_NEAR(h, h_true, 0.7, "height still the true height after the return");
    CHECK_TRUE(
        max_step < 1.0f,
        "height never took a step larger than 1 m in one epoch, arrival/outage/return alike");
}

/* ---------------------------------------------------------------------------
 * Scenario 16b: absolute height without a barometer (REQ-SUITE-008)
 *
 * The offset filter is source-agnostic: with no barometer at all, the
 * local height reference is the ins local height, and the offset must
 * still converge to the ellipsoid height of the datum origin. After the
 * GNSS drops (mode leaves FULL) the absolute height is then formed from
 * the local height + offset rather than from the fix.
 * ---------------------------------------------------------------------------
 */

static void scenario_height_ellipsoid_no_baro(void)
{
    printf("\n-- scenario: absolute height without a barometer --\n");

    const double lat       = 48.783 * M_PI / 180.0;
    const double lon       = 9.181 * M_PI / 180.0;
    const float  h_ell_fix = 390.0f; /* static: origin ends up here too */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = lat;
    init.llh[1]                    = lon;
    init.llh[2]                    = (double)h_ell_fix;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);
    init.magnetic_n[0]             = 20.0f;
    init.magnetic_n[2]             = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    opt.auto_init                          = true;
    opt.max_deadreckoning_sec              = 30.0f; /* stay ready through the outage */

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    double fix_ecef[3];
    ins_latlonh_to_ecef(lat, lon, (double)h_ell_fix, fix_ecef);

    ins_measurements_t m;
    float              h, h_ell, off;
    int                i;

    /* 40 s static with 1 Hz GNSS, no barometer ever. */
    for (i = 1; i <= 4000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        if ((i % 100) == 1)
        {
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(fix_ecef, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1],
                                &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 2.25f; /* 1.5 m vertical */
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(!s.baro_alt.is_initialized, "no barometer involved");
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode");
    CHECK_TRUE(local_gnss_alt_get(&s.local_gnss, &off, (float*)0),
               "offset filter runs without a barometer");
    CHECK_NEAR(off, h_ell_fix, 1.5, "offset = ellipsoid height of the datum origin");

    /* GNSS drops. ins stays ready but leaves FULL, so the absolute
       height now comes from the local height reference + offset. */
    for (i = 1; i <= 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_COASTING,
               "coasting: offset branch is live");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "local height available");
    CHECK_NEAR(h, 0.0, 1.0, "local height still zero at the origin");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h_ell), "absolute height without a barometer");
    CHECK_NEAR(h_ell, h_ell_fix, 2.0, "ellipsoid height from local height + offset");
}

/* ---------------------------------------------------------------------------
 * Scenario 16c: WGS84 <-> local NED conversion for GNSS users
 *
 * A GNSS drone flying WGS84 waypoints needs to relate the local n-frame
 * solution to WGS84 in both directions. The converters must round-trip,
 * agree with the ins solution, and be gated on a real GNSS anchor.
 * ---------------------------------------------------------------------------
 */

static void scenario_wgs84_conversion(void)
{
    printf("\n-- scenario: WGS84 <-> local conversion --\n");

    const double lat       = 48.783 * M_PI / 180.0;
    const double lon       = 9.181 * M_PI / 180.0;
    const float  h_ell_org = 380.0f;

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, (double)h_ell_org);

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    /* Before any GNSS the n-frame is not WGS84-anchored -> refuse. */
    double lt, ln, hh;
    float  ned_tmp[3] = {0.0f, 0.0f, 0.0f};
    CHECK_TRUE(!nav_suite_local_to_wgs84(&s, ned_tmp, &lt, &ln, &hh), "no conversion before GNSS");
    CHECK_TRUE(!nav_suite_wgs84_to_local(&s, lat, lon, (double)h_ell_org, ned_tmp),
               "no inverse before GNSS");

    double fix[3];
    ins_latlonh_to_ecef(lat, lon, (double)h_ell_org, fix);
    ins_measurements_t m;
    int                i;
    for (i = 1; i <= 3000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        if ((i % 100) == 1)
        {
            m.gnss_pos.is_valid = true;
            ins_ecef_to_latlonh(fix, &m.gnss_pos.llh[0], &m.gnss_pos.llh[1], &m.gnss_pos.llh[2]);
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 2.25f;
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL mode");

    /* The origin (local 0,0,0) must map back to the fix coordinates. */
    const float origin_ned[3] = {0.0f, 0.0f, 0.0f};
    CHECK_TRUE(nav_suite_local_to_wgs84(&s, origin_ned, &lt, &ln, &hh), "origin -> WGS84");
    CHECK_NEAR(lt, lat, 1e-7, "origin latitude");
    CHECK_NEAR(ln, lon, 1e-7, "origin longitude");
    CHECK_NEAR(hh, (double)h_ell_org, 0.5, "origin ellipsoid height");

    /* A waypoint 100 m north / 50 m east / 10 m up, round-tripped. */
    const float wp_ned[3] = {100.0f, 50.0f, -10.0f};
    CHECK_TRUE(nav_suite_local_to_wgs84(&s, wp_ned, &lt, &ln, &hh), "waypoint -> WGS84");
    CHECK_NEAR(hh, (double)h_ell_org + 10.0, 0.5, "waypoint ellipsoid height (10 m up)");
    float back[3];
    CHECK_TRUE(nav_suite_wgs84_to_local(&s, lt, ln, hh, back), "WGS84 -> local");
    CHECK_NEAR(back[0], wp_ned[0], 0.05, "round-trip north");
    CHECK_NEAR(back[1], wp_ned[1], 0.05, "round-trip east");
    CHECK_NEAR(back[2], wp_ned[2], 0.05, "round-trip down");
}

/* ---------------------------------------------------------------------------
 * Scenario 17: API guard rails. NULL/uninitialized instances refuse to
 * publish or crash, invalid init arguments are rejected.
 * ---------------------------------------------------------------------------
 */

static void scenario_api_guards(void)
{
    printf("\n-- scenario: API guard rails --\n");

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Init rejections. */
    CHECK_TRUE(baro_alt_init((baro_alt_t*)0, &cfg, 0, 101325.0f, 0.0f, 0.0f) == -1,
               "init rejects NULL instance");
    baro_alt_t b;
    CHECK_TRUE(baro_alt_init(&b, (const baro_alt_config_t*)0, 0, 101325.0f, 0.0f, 0.0f) == -1,
               "init rejects NULL config");

    /* Uninitialized instance: no outputs, update is a no-op. */
    memset(&b, 0, sizeof(b));
    float       v;
    const float acc[3]  = {0.0f, 0.0f, -GRAVITY};
    const float q_id[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    CHECK_TRUE(!baro_alt_get_height(&b, &v), "uninitialized: no height");
    CHECK_TRUE(!baro_alt_get_velocity(&b, &v), "uninitialized: no velocity");
    CHECK_TRUE(!baro_alt_get_acc_bias(&b, &v), "uninitialized: no bias");
    CHECK_TRUE(!baro_alt_get_height((const baro_alt_t*)0, &v), "NULL: no height");
    baro_alt_update(&b, 1000, acc, q_id, 101325.0f, 0.0f, true);
    CHECK_TRUE(!b.is_initialized, "update on uninitialized is a no-op");

    /* Offset filter: init rejects a non-positive GNSS accuracy and a
       NULL instance/config; accessors gate on initialization. */
    local_gnss_alt_config_t gcfg;
    memset(&gcfg, 0, sizeof(gcfg));
    local_gnss_alt_t g;
    CHECK_TRUE(local_gnss_alt_init((local_gnss_alt_t*)0, &gcfg, 0, 0.0f, 0.5f, 50.0f, 1.0f) == -1,
               "offset init rejects NULL instance");
    CHECK_TRUE(local_gnss_alt_init(&g, (const local_gnss_alt_config_t*)0, 0, 0.0f, 0.5f, 50.0f,
                                   1.0f) == -1,
               "offset init rejects NULL config");
    CHECK_TRUE(local_gnss_alt_init(&g, &gcfg, 0, 0.0f, 0.5f, 50.0f, 0.0f) == -1,
               "offset init rejects zero GNSS stddev");
    memset(&g, 0, sizeof(g));
    CHECK_TRUE(!local_gnss_alt_get(&g, &v, (float*)0), "uninitialized offset: no output");
    local_gnss_alt_update(&g, 1000, 0.0f, 0.5f, 50.0f, 1.0f);
    CHECK_TRUE(!g.is_initialized, "offset update on uninitialized no-op");

    /* Restart thresholds: a negative value means "leave this state
       unchecked" and must resolve to 0, which is NOT what a plain 0 (or
       a NaN) does -- those pick the built-in default. Getting the two
       spellings the wrong way round would silently arm a watchdog the
       caller asked to switch off. */
    {
        baro_alt_config_t cfg_off;
        memset(&cfg_off, 0, sizeof(cfg_off));
        cfg_off.restart_h_stddev_m   = -1.0f;
        cfg_off.restart_v_stddev_mps = -1.0f;
        baro_alt_t b_off;
        CHECK_TRUE(baro_alt_init(&b_off, &cfg_off, 0, 101325.0f, 0.0f, 0.0f) == 0,
                   "init with per-state watchdog opt-out");
        CHECK_NEAR(b_off.cfg.restart_h_stddev_m, 0.0f, 1e-12,
                   "a negative height threshold resolves to unchecked");
        CHECK_NEAR(b_off.cfg.restart_v_stddev_mps, 0.0f, 1e-12,
                   "a negative velocity threshold resolves to unchecked");

        baro_alt_config_t cfg_def;
        memset(&cfg_def, 0, sizeof(cfg_def));
        cfg_def.restart_h_stddev_m   = 0.0f;
        cfg_def.restart_v_stddev_mps = (float)NAN;
        baro_alt_t b_def;
        CHECK_TRUE(baro_alt_init(&b_def, &cfg_def, 0, 101325.0f, 0.0f, 0.0f) == 0,
                   "init with 0/NaN thresholds");
        CHECK_TRUE(b_def.cfg.restart_h_stddev_m > 0.0f, "0 picks the built-in height default");
        CHECK_TRUE(b_def.cfg.restart_v_stddev_mps > 0.0f,
                   "NaN picks the built-in velocity default");
    }

    /* nav_suite's local-position offset accessor: it reports an offset
       only once one has actually been estimated, so every other case has
       to answer false rather than hand out an uninitialized array. */
    {
        static nav_suite_t s_guard;
        float              off[3] = {1.0f, 2.0f, 3.0f};
        memset(&s_guard, 0, sizeof(s_guard));
        CHECK_TRUE(!nav_suite_get_local_pos_offset((const nav_suite_t*)0, off),
                   "local_pos offset: NULL suite refused");
        CHECK_TRUE(!nav_suite_get_local_pos_offset(&s_guard, (float*)0),
                   "local_pos offset: NULL output refused");
        CHECK_TRUE(!nav_suite_get_local_pos_offset(&s_guard, off),
                   "local_pos offset: none estimated yet");
    }

    /* baro_alt_resolve_config(): every "<= 0 or non-finite -> default"
       field is exercised elsewhere only via a zeroed cfg (always
       defaulted) or a NaN (also always defaulted): an explicit, already
       valid value that must survive untouched never runs, and neither
       does a POSITIVE but non-finite (+Inf) one -- isfinite() is a
       separate check from "> 0.0f", not implied by it. */
    {
        baro_alt_config_t cfg_ok;
        memset(&cfg_ok, 0, sizeof(cfg_ok));
        cfg_ok.h_init_stddev_m            = 1.11f;
        cfg_ok.v_init_stddev_mps          = 2.22f;
        cfg_ok.acc_bias_init_stddev_mps2  = 0.033f;
        cfg_ok.acc_noise_mps2_sqrthz      = 0.044f;
        cfg_ok.acc_bias_drift_mps2_sqrthz = 0.0055f;
        cfg_ok.h_process_noise_m_sqrthz   = 0.0066f;
        cfg_ok.baro_stddev_m              = 0.77f;
        cfg_ok.chi2_threshold             = 8.8f;
        cfg_ok.zupt_stddev_mps            = 0.099f;
        cfg_ok.restart_h_stddev_m         = 12.0f;
        cfg_ok.restart_v_stddev_mps       = 13.0f;
        cfg_ok.restart_warmup_sec         = 14.0f;
        baro_alt_t b_ok;
        CHECK_TRUE(baro_alt_init(&b_ok, &cfg_ok, 0, 101325.0f, 0.0f, 0.0f) == 0,
                   "init with explicit values succeeds");
        CHECK_NEAR(b_ok.cfg.h_init_stddev_m, cfg_ok.h_init_stddev_m, 1e-9,
                   "h_init_stddev_m not defaulted");
        CHECK_NEAR(b_ok.cfg.v_init_stddev_mps, cfg_ok.v_init_stddev_mps, 1e-9,
                   "v_init_stddev_mps not defaulted");
        CHECK_NEAR(b_ok.cfg.acc_bias_init_stddev_mps2, cfg_ok.acc_bias_init_stddev_mps2, 1e-9,
                   "acc_bias_init_stddev_mps2 not defaulted");
        CHECK_NEAR(b_ok.cfg.acc_noise_mps2_sqrthz, cfg_ok.acc_noise_mps2_sqrthz, 1e-9,
                   "acc_noise_mps2_sqrthz not defaulted");
        CHECK_NEAR(b_ok.cfg.acc_bias_drift_mps2_sqrthz, cfg_ok.acc_bias_drift_mps2_sqrthz, 1e-9,
                   "acc_bias_drift_mps2_sqrthz not defaulted");
        CHECK_NEAR(b_ok.cfg.h_process_noise_m_sqrthz, cfg_ok.h_process_noise_m_sqrthz, 1e-9,
                   "h_process_noise_m_sqrthz not defaulted");
        CHECK_NEAR(b_ok.cfg.baro_stddev_m, cfg_ok.baro_stddev_m, 1e-9,
                   "baro_stddev_m not defaulted");
        CHECK_NEAR(b_ok.cfg.chi2_threshold, cfg_ok.chi2_threshold, 1e-9,
                   "chi2_threshold not defaulted");
        CHECK_NEAR(b_ok.cfg.zupt_stddev_mps, cfg_ok.zupt_stddev_mps, 1e-9,
                   "zupt_stddev_mps not defaulted");
        CHECK_NEAR(b_ok.cfg.restart_h_stddev_m, cfg_ok.restart_h_stddev_m, 1e-9,
                   "restart_h_stddev_m not defaulted");
        CHECK_NEAR(b_ok.cfg.restart_v_stddev_mps, cfg_ok.restart_v_stddev_mps, 1e-9,
                   "restart_v_stddev_mps not defaulted");
        CHECK_NEAR(b_ok.cfg.restart_warmup_sec, cfg_ok.restart_warmup_sec, 1e-9,
                   "restart_warmup_sec not defaulted");

        baro_alt_config_t cfg_inf;
        memset(&cfg_inf, 0, sizeof(cfg_inf));
        const float inf                    = (float)INFINITY;
        cfg_inf.h_init_stddev_m            = inf;
        cfg_inf.v_init_stddev_mps          = inf;
        cfg_inf.acc_bias_init_stddev_mps2  = inf;
        cfg_inf.acc_noise_mps2_sqrthz      = inf;
        cfg_inf.acc_bias_drift_mps2_sqrthz = inf;
        cfg_inf.h_process_noise_m_sqrthz   = inf;
        cfg_inf.baro_stddev_m              = inf;
        cfg_inf.chi2_threshold             = inf;
        cfg_inf.zupt_stddev_mps            = inf;
        cfg_inf.restart_h_stddev_m         = inf;
        cfg_inf.restart_v_stddev_mps       = inf;
        cfg_inf.restart_warmup_sec         = inf;
        baro_alt_t b_inf;
        CHECK_TRUE(baro_alt_init(&b_inf, &cfg_inf, 0, 101325.0f, 0.0f, 0.0f) == 0,
                   "init with +Inf values succeeds");
        CHECK_TRUE(isfinite(b_inf.cfg.h_init_stddev_m), "+Inf h_init_stddev_m defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.v_init_stddev_mps), "+Inf v_init_stddev_mps defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.acc_bias_init_stddev_mps2),
                   "+Inf acc_bias_init_stddev_mps2 defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.acc_noise_mps2_sqrthz),
                   "+Inf acc_noise_mps2_sqrthz defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.acc_bias_drift_mps2_sqrthz),
                   "+Inf acc_bias_drift_mps2_sqrthz defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.h_process_noise_m_sqrthz),
                   "+Inf h_process_noise_m_sqrthz defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.baro_stddev_m), "+Inf baro_stddev_m defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.chi2_threshold), "+Inf chi2_threshold defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.zupt_stddev_mps), "+Inf zupt_stddev_mps defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.restart_h_stddev_m), "+Inf restart_h_stddev_m defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.restart_v_stddev_mps), "+Inf restart_v_stddev_mps defaulted");
        CHECK_TRUE(isfinite(b_inf.cfg.restart_warmup_sec), "+Inf restart_warmup_sec defaulted");

        /* baro_alt_init()'s own h_init_stddev_m PARAMETER (distinct from
           cfg.h_init_stddev_m) has the same "> 0 && isfinite" override
           gate, tested elsewhere only with finite values. */
        baro_alt_config_t cfg_plain;
        memset(&cfg_plain, 0, sizeof(cfg_plain));
        baro_alt_t b_param_inf;
        CHECK_TRUE(baro_alt_init(&b_param_inf, &cfg_plain, 0, 101325.0f, 0.0f, inf) == 0,
                   "init with +Inf h_init_stddev_m param succeeds");
        CHECK_TRUE(isfinite(b_param_inf.cfg.h_init_stddev_m),
                   "+Inf h_init_stddev_m param falls back to cfg default");
    }

    /* NULL-instance guards on the remaining accessors/mutators, tested
       elsewhere only with a valid-but-uninitialized instance -- the
       is_initialized half of the guard, never the NULL-pointer half. */
    {
        float vv;
        int   ms;
        baro_alt_zero_velocity_update((baro_alt_t*)0, 0.0f); /* must not crash */
        CHECK_TRUE(!baro_alt_get_isa_altitude((const baro_alt_t*)0, &vv), "isa_altitude(NULL)");
        CHECK_TRUE(!baro_alt_get_velocity((const baro_alt_t*)0, &vv), "velocity(NULL)");
        CHECK_TRUE(!baro_alt_get_acc_bias((const baro_alt_t*)0, &vv), "acc_bias(NULL)");
        CHECK_TRUE(!baro_alt_get_measurement_a_z((const baro_alt_t*)0, &vv),
                   "measurement_a_z(NULL)");
        CHECK_TRUE(!baro_alt_get_measurement_h((const baro_alt_t*)0, &vv), "measurement_h(NULL)");
        ms = baro_alt_deadreckoning_ms((const baro_alt_t*)0);
        CHECK_TRUE(ms == -1, "deadreckoning_ms(NULL)");
        CHECK_TRUE(!local_gnss_alt_get((const local_gnss_alt_t*)0, &vv, &vv), "offset get(NULL)");
    }

    /* baro_alt_update()'s own baro_stddev_m PARAMETER and
       baro_alt_zero_velocity_update()'s stddev_mps PARAMETER share the
       same "> 0 && isfinite" override gate as h_init_stddev_m above,
       also tested elsewhere only with finite values. */
    {
        const float       inf        = (float)INFINITY;
        const float       acc_lvl[3] = {0.0f, 0.0f, -GRAVITY};
        const float       q_lvl[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
        baro_alt_config_t cfg2;
        memset(&cfg2, 0, sizeof(cfg2));
        baro_alt_t         b2;
        baro_alt_time_us_t t2 = 0;
        CHECK_TRUE(baro_alt_init(&b2, &cfg2, t2, 101325.0f, 0.0f, 0.0f) == 0, "init (param inf)");
        t2 += US_PER_SEC / 100;
        baro_alt_update(&b2, t2, acc_lvl, q_lvl, 101325.0f, inf, true); /* +Inf baro_stddev_m */
        CHECK_TRUE(b2.is_initialized, "+Inf baro_stddev_m falls back, does not crash");
        baro_alt_zero_velocity_update(&b2, inf); /* +Inf stddev_mps */
        CHECK_TRUE(b2.is_initialized, "+Inf zupt stddev falls back, does not crash");
    }

    /* local_gnss_alt_resolve_config(): the same explicit-value and
       +Inf passthrough coverage as baro_alt_resolve_config() above. */
    {
        local_gnss_alt_config_t gcfg_ok;
        memset(&gcfg_ok, 0, sizeof(gcfg_ok));
        gcfg_ok.rw_stddev_mps           = 0.011f;
        gcfg_ok.local_stddev_m          = 0.022f;
        gcfg_ok.chi2_threshold          = 4.4f;
        gcfg_ok.min_update_interval_sec = 0.5f;
        gcfg_ok.stddev_inflation_factor = 2.5f;
        local_gnss_alt_t g_ok;
        CHECK_TRUE(local_gnss_alt_init(&g_ok, &gcfg_ok, 0, 0.0f, 0.5f, 50.0f, 1.0f) == 0,
                   "offset init with explicit values succeeds");
        CHECK_NEAR(g_ok.cfg.rw_stddev_mps, gcfg_ok.rw_stddev_mps, 1e-9,
                   "rw_stddev_mps not defaulted");
        CHECK_NEAR(g_ok.cfg.local_stddev_m, gcfg_ok.local_stddev_m, 1e-9,
                   "local_stddev_m not defaulted");
        CHECK_NEAR(g_ok.cfg.chi2_threshold, gcfg_ok.chi2_threshold, 1e-9,
                   "offset chi2_threshold not defaulted");
        CHECK_NEAR(g_ok.cfg.min_update_interval_sec, gcfg_ok.min_update_interval_sec, 1e-9,
                   "min_update_interval_sec not defaulted");
        CHECK_NEAR(g_ok.cfg.stddev_inflation_factor, gcfg_ok.stddev_inflation_factor, 1e-9,
                   "stddev_inflation_factor not defaulted");

        const float             inf = (float)INFINITY;
        local_gnss_alt_config_t gcfg_inf;
        memset(&gcfg_inf, 0, sizeof(gcfg_inf));
        gcfg_inf.rw_stddev_mps           = inf;
        gcfg_inf.local_stddev_m          = inf;
        gcfg_inf.chi2_threshold          = inf;
        gcfg_inf.min_update_interval_sec = inf;
        gcfg_inf.stddev_inflation_factor = inf;
        local_gnss_alt_t g_inf;
        CHECK_TRUE(local_gnss_alt_init(&g_inf, &gcfg_inf, 0, 0.0f, 0.5f, 50.0f, 1.0f) == 0,
                   "offset init with +Inf values succeeds");
        CHECK_TRUE(isfinite(g_inf.cfg.rw_stddev_mps), "+Inf rw_stddev_mps defaulted");
        CHECK_TRUE(isfinite(g_inf.cfg.local_stddev_m), "+Inf local_stddev_m defaulted");
        CHECK_TRUE(isfinite(g_inf.cfg.chi2_threshold), "+Inf offset chi2_threshold defaulted");
        CHECK_TRUE(isfinite(g_inf.cfg.min_update_interval_sec),
                   "+Inf min_update_interval_sec defaulted");
        CHECK_TRUE(isfinite(g_inf.cfg.stddev_inflation_factor),
                   "+Inf stddev_inflation_factor defaulted");

        /* local_gnss_alt_pair_ok()'s own gnss_stddev_m/local_stddev_m
           PARAMETERS (distinct from the cfg fields above): a +Inf pair
           value must be rejected outright (unlike h_init_stddev_m, there
           is no override use for a positive Inf here -- gnss_stddev_m
           has no cfg fallback at all, and local_stddev_m only reuses the
           cfg default on the SAME "> 0 && isfinite" failure). */
        local_gnss_alt_config_t gcfg_plain;
        memset(&gcfg_plain, 0, sizeof(gcfg_plain));
        local_gnss_alt_t g_plain;
        CHECK_TRUE(local_gnss_alt_init(&g_plain, &gcfg_plain, 0, 0.0f, 0.5f, 50.0f, 1.0f) == 0,
                   "offset init (plain)");
        local_gnss_alt_update(&g_plain, 1000, 0.0f, 0.5f, 50.0f, inf); /* +Inf gnss_stddev_m */
        CHECK_TRUE(g_plain.n_invalid_input == 1, "+Inf gnss_stddev_m param rejected");
        local_gnss_alt_update(&g_plain, 1000, 0.0f, inf, 50.0f, 1.0f); /* +Inf local_stddev_m */
        CHECK_TRUE(g_plain.n_invalid_input == 1,
                   "+Inf local_stddev_m param falls back, pair accepted");

        /* local_gnss_alt_get(): a valid, initialized instance with a
           NULL offset_m output (stddev_m still requested) -- tested
           elsewhere only on an uninitialized instance, which returns
           before ever reaching this line. */
        float stddev_only;
        CHECK_TRUE(local_gnss_alt_get(&g_plain, (float*)0, &stddev_only),
                   "offset get with NULL offset_m output still succeeds");
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 18: corrupted covariance factors (REQ-SYS-005). Prediction
 * and fusion on broken U/d must not crash; the filter either flags
 * itself unready or keeps publishing finite values, and a re-init
 * fully recovers it. Same drill for the 1-state offset filter.
 * ---------------------------------------------------------------------------
 */

static void scenario_corrupted_covariance(void)
{
    printf("\n-- scenario: corrupted covariance factors fail safe --\n");

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    const float acc[3]  = {0.0f, 0.0f, -GRAVITY};
    const float q_id[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float p0      = 101325.0f;

    const char* names[4] = {"d negative", "d NaN", "d Inf", "U NaN"};
    int         kind, i;
    for (kind = 0; kind < 4; ++kind)
    {
        printf("      -- corruption: %s\n", names[kind]);
        baro_alt_t         b;
        baro_alt_time_us_t t = 1000000;
        CHECK_TRUE(baro_alt_init(&b, &cfg, t, p0, 0.0f, 0.0f) == 0, "init");
        for (i = 0; i < 100; ++i)
        {
            t += US_PER_SEC / 100;
            baro_alt_update(&b, t, acc, q_id, p0, 0.0f, (i % 10) == 0);
        }
        CHECK_TRUE(b.is_initialized, "healthy before corruption");

        switch (kind)
        {
        case 0: b.d[1] = -1e-4f; break;
        case 1: b.d[0] = nanf(""); break;
        case 2: b.d[2] = INFINITY; break;
        case 3: b.U[2 * BARO_ALT_STATES + 0] = nanf(""); break; /* (0,2) */
        }

        /* Prediction (every epoch) + baro fusion on the broken factors:
           must not crash. */
        for (i = 0; i < 10; ++i)
        {
            t += US_PER_SEC / 100;
            baro_alt_update(&b, t, acc, q_id, p0, 0.0f, true);
        }
        if (kind == 1 || kind == 3)
        {
            /* NaN sigma in kalman_udu_predict() is caught by its own
               singularity guard (sigma > KALMAN_UDU_EPS) before it can
               reach d/U or the health check: the guard cannot tell a
               NaN apart from a legitimately singular sigma, so it
               resets the affected d[i]/U column to 0 and the corruption
               is healed rather than detected. */
            CHECK_TRUE(b.is_initialized, "self-healed by predict singularity guard");
        }
        else if (kind != 0)
        {
            /* Inf propagates into d within one predict. */
            CHECK_TRUE(!b.is_initialized, "health check tripped");
        }
        /* Fail-safe: finite output or no output. */
        float h;
        if (baro_alt_get_height(&b, &h)) { CHECK_TRUE(isfinite(h), "published height finite"); }

        /* Re-init of the same instance restores normal operation. */
        CHECK_TRUE(baro_alt_init(&b, &cfg, t, p0, 0.0f, 0.0f) == 0, "re-init");
        for (i = 0; i < 100; ++i)
        {
            t += US_PER_SEC / 100;
            baro_alt_update(&b, t, acc, q_id, p0, 0.0f, (i % 10) == 0);
        }
        CHECK_TRUE(baro_alt_get_height(&b, &h), "height valid again");
        CHECK_NEAR(h, 0.0, 0.3, "recovered height [m]");
    }

    /* Offset filter: a corrupted (negative / non-finite) variance must
       trip its health check on the next pair, and re-init recovers. */
    local_gnss_alt_config_t gcfg;
    memset(&gcfg, 0, sizeof(gcfg));
    gcfg.min_update_interval_sec = 0.5f; /* exercise the health check at 1 Hz */
    local_gnss_alt_t   g;
    baro_alt_time_us_t tg = 1000000;
    CHECK_TRUE(local_gnss_alt_init(&g, &gcfg, tg, 0.0f, 0.5f, 50.0f, 1.0f) == 0, "offset init");
    g.var_m2 = -1.0f;
    tg += US_PER_SEC;
    local_gnss_alt_update(&g, tg, 0.0f, 0.5f, 50.0f, 1.0f);
    CHECK_TRUE(!g.is_initialized, "offset health check (negative var)");
    CHECK_TRUE(local_gnss_alt_init(&g, &gcfg, tg, 0.0f, 0.5f, 50.0f, 1.0f) == 0, "offset re-init");
    g.offset_m = nanf("");
    tg += US_PER_SEC;
    local_gnss_alt_update(&g, tg, 0.0f, 0.5f, 50.0f, 1.0f);
    CHECK_TRUE(!g.is_initialized, "offset health check (NaN offset)");
    CHECK_TRUE(local_gnss_alt_init(&g, &gcfg, tg, 0.0f, 0.5f, 50.0f, 1.0f) == 0, "offset recovers");
    float off;
    CHECK_TRUE(local_gnss_alt_get(&g, &off, (float*)0), "offset published");
    CHECK_NEAR(off, 50.0, 1e-3, "offset value sane [m]");
}

/* ---------------------------------------------------------------------------
 * REQ-SUITE-006: averaged baro bootstrap. The datum anchor is the mean of the
 * plausible pressure samples over a short window, so a single glitched startup
 * reading cannot skew it: bootstrap needs the window (not one sample), and a
 * grossly implausible first sample is screened out of the mean.
 * ---------------------------------------------------------------------------
 */
static void scenario_baro_avg_bootstrap(void)
{
    printf("\n-- scenario: averaged baro bootstrap screens a glitch (REQ-SUITE-006) --\n");

    const float base_alt = 300.0f;
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY}; /* static, level */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ins_time_us_t t                = 1000000;
    init.time                      = t;
    init.llh[0]                    = 48.783 * M_PI / 180.0;
    init.llh[1]                    = 9.181 * M_PI / 180.0;
    init.llh[2]                    = 300.0;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
    init.zero_vel_stddev_mps       = 0.01f;
    init.zero_rot_stddev_rps       = DEG2RAD(0.001f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* These scenarios drive synthetic, noiseless profiles (constant-rate
       climbs, datum handovers). A constant-rate climb has zero sample
       variance, and its acceleration crosses zero at the midpoint of the
       ramp for close to a second -- long enough for a short
       auto_zupt_dwell_sec to also arm ins's own detector, not just the
       ARS/AHRS fallback, none of which see a GNSS velocity here to gate
       on. Opt out of all three; the detector has its own scenario
       above. */
    nav_suite_set_auto_zupt_zaru_disable(&s, true);

    ins_measurements_t m;
    int                i;
#define BOOT_EPOCH(BARO_VALID, PRESSURE)      \
    do {                                      \
        t += US_PER_SEC / 100;                \
        memset(&m, 0, sizeof(m));             \
        m.timestamp        = t;               \
        m.strapdown_dt_sec = 0.01f;           \
        m.acc.is_valid     = true;            \
        m.gyr.is_valid     = true;            \
        memcpy(m.acc.data, f_b, sizeof(f_b)); \
        m.baro.is_valid    = (BARO_VALID);    \
        m.baro.pressure_pa = (PRESSURE);      \
        nav_suite_update(&s, &m);             \
    } while (0)

    /* Warm the ARS up (attitude source) with a few IMU-only epochs. */
    for (i = 0; i < 20; ++i) BOOT_EPOCH(false, 0.0f);
    CHECK_TRUE(!nav_suite_get_baro_alt(&s, (float*)0, (float*)0), "baro waits (no sample yet)");

    /* A grossly implausible first pressure (5000 Pa, ~16 km) must be
       screened out of the mean, not anchor the datum. */
    BOOT_EPOCH(true, 5000.0f);
    CHECK_TRUE(!s.baro_alt.is_initialized, "glitch sample does not bootstrap");

    /* One good sample: still collecting (window not elapsed). */
    BOOT_EPOCH(true, pressure_from_altitude(base_alt));
    CHECK_TRUE(!s.baro_alt.is_initialized, "one good sample: still averaging");

    /* Fill the window with good samples -> bootstrap from their mean. */
    for (i = 0; i < 40; ++i) BOOT_EPOCH(true, pressure_from_altitude(base_alt + gauss(0.2f)));
    CHECK_TRUE(s.baro_alt.is_initialized, "bootstrapped after the window");

    float h;
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "baro running");
    /* Datum from the good-sample mean (glitch screened): height above
       start ~ 0, not skewed toward the 5000 Pa reading. */
    CHECK_NEAR(h, 0.0, 0.5, "datum anchored from the clean mean, glitch ignored");
#undef BOOT_EPOCH
}

/* ---------------------------------------------------------------------------
 * REQ-BARO-021: vertical-channel precision restart watchdog. With the
 * barometer starved, the accelerometer-only dead reckoning drifts until the
 * height 1-sigma blows past the threshold; past the warm-up the watchdog
 * marks the filter uninitialized. A disabled watchdog leaves it running, and
 * a fresh init on a good baro stream recovers.
 * ---------------------------------------------------------------------------
 */
/* Two error paths of the vertical channel that the ordinary scenarios
 * never walk: the log state that tracks a barometer outage (so a
 * returning sensor is reported as returning, not as one more epoch of
 * silence), and a zero-velocity update attempted on covariance factors
 * that can no longer support one.
 *
 * The gap thresholds are private to baro_alt.c (BARO_ALT_LOG_GAP_*):
 * 5 s before an outage is reported, 10 s between repeats.
 */
static void scenario_baro_gap_and_fuse_failure(void)
{
    printf("\n-- scenario: barometer outage tracking and a refused ZUPT --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY};
    const float p0       = pressure_from_altitude(base_alt);

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    /* The accel-only stretch below is longer than the precision
       watchdog's patience; this scenario is about the gap bookkeeping,
       not about the watchdog (scenario_baro_precision_restart covers
       that one). */
    cfg.precision_restart_disable = true;

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, p0, 0.0f, 0.0f) == 0, "baro_alt_init");

    int i;
    for (i = 0; i < 200; ++i) /* 2 s of healthy barometer */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, p0, 0.0f, true);
    }
    CHECK_TRUE(b.log_state.t_last_gap_warn == 0, "no outage reported while the sensor is healthy");

    for (i = 0; i < 800; ++i) /* 8 s without a barometer sample */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(b.log_state.t_last_gap_warn != 0, "an outage past the warn threshold is reported");
    CHECK_TRUE(baro_alt_deadreckoning_ms(&b) >= 8000, "and the dead-reckoning clock agrees");

    /* The sensor comes back. The gap is measured against the last
       ACCEPTED fusion, so the first epoch still sees the old gap; from
       the next one on the outage is over and the state has to be
       cleared, otherwise the next outage would be reported as a repeat
       of this one rather than as a new event. */
    for (i = 0; i < 100; ++i) /* 1 s */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, p0, 0.0f, true);
    }
    CHECK_TRUE(b.log_state.t_last_gap_warn == 0, "the outage state is cleared on reacquisition");
    CHECK_TRUE(baro_alt_deadreckoning_ms(&b) < 100, "and the dead-reckoning clock is reset");

    /* A second outage is reported again rather than being swallowed by
       the first one's throttle. */
    for (i = 0; i < 800; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(b.log_state.t_last_gap_warn != 0, "a second outage is reported as its own event");

    /* A zero-velocity update on covariance factors that cannot support
       one: the UDU update refuses it instead of writing a state built on
       a negative variance, and the health check pulls the plug on the
       instance right after. */
    {
        baro_alt_t         bz;
        baro_alt_time_us_t tz = 1000000;
        baro_alt_config_t  cfgz;
        memset(&cfgz, 0, sizeof(cfgz));
        CHECK_TRUE(baro_alt_init(&bz, &cfgz, tz, p0, 0.0f, 0.0f) == 0, "init for the refused ZUPT");
        for (i = 0; i < 100; ++i)
        {
            tz += US_PER_SEC / 100;
            baro_alt_update(&bz, tz, f_b, q, p0, 0.0f, true);
        }
        CHECK_TRUE(bz.is_initialized && bz.n_fuse_fail == 0, "healthy before the corruption");

        /* Large and negative: the update's own health check adds
           R + a'*D*a term by term and bails as soon as that goes
           non-positive. */
        bz.d[1] = -1.0e6f;
        baro_alt_zero_velocity_update(&bz, 0.05f);
        CHECK_TRUE(bz.n_fuse_fail == 1, "the refused ZUPT is counted as a fusion failure");
        CHECK_TRUE(bz.n_zupt == 0, "and not counted as an applied ZUPT");
        CHECK_TRUE(!bz.is_initialized, "the health check shuts the instance down");
        float h;
        CHECK_TRUE(!baro_alt_get_height(&bz, &h), "nothing published afterwards");

        /* A re-init brings the same instance back. */
        CHECK_TRUE(baro_alt_init(&bz, &cfgz, tz, p0, 0.0f, 0.0f) == 0, "re-init");
        CHECK_TRUE(bz.n_fuse_fail == 0, "the counter starts over");
        for (i = 0; i < 200; ++i)
        {
            tz += US_PER_SEC / 100;
            baro_alt_update(&bz, tz, f_b, q, p0, 0.0f, true);
        }
        CHECK_TRUE(baro_alt_get_height(&bz, &h), "publishing again");
        /* The anchor pressure is the datum, so a platform that has not
           moved reads 0 m again. */
        CHECK_NEAR(h, 0.0f, 1.0, "recovered height above the datum [m]");
    }
}

static void scenario_baro_precision_restart(void)
{
    printf("\n-- scenario: vertical-channel precision restart watchdog (REQ-BARO-021) --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY}; /* static, level */

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.restart_warmup_sec   = 0.5f;
    cfg.restart_h_stddev_m   = 1.5f;  /* trip quickly on accel-only drift */
    cfg.restart_v_stddev_mps = -1.0f; /* unchecked: height alone drives this trip */

    baro_alt_t         b;
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");

    int i;
    /* Within the warm-up: no restart even as the covariance already grows. */
    for (i = 1; i <= 50; ++i) /* 0.5 s, accel only (no baro) */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(b.is_initialized, "no restart during warm-up");
    CHECK_TRUE(b.n_restart == 0, "restart counter 0 in warm-up");

    /* Past the warm-up the accel-only drift blows the height 1-sigma past
       the threshold and the watchdog trips (empirically ~5.5 s with the
       default acc_noise_mps2_sqrthz, generous margin below). */
    for (i = 1; i <= 1500 && b.is_initialized; ++i) /* up to 15 s */
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(!b.is_initialized, "precision watchdog marked the filter uninitialized");
    CHECK_TRUE(b.n_restart == 1, "restart counted once");
    float h;
    CHECK_TRUE(!baro_alt_get_height(&b, &h), "nothing published after restart");

    /* Standalone recovery: a fresh init on a good baro stream restores it. */
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "re-init");
    for (i = 1; i <= 300; ++i) /* 3 s of baro, past the warm-up */
    {
        t += US_PER_SEC / 100;
        const float p = pressure_from_altitude(base_alt + gauss(0.2f));
        baro_alt_update(&b, t, f_b, q, p, 0.0f, (i % 5) == 0);
    }
    CHECK_TRUE(b.is_initialized, "healthy again on a good baro stream");
    CHECK_TRUE(b.n_restart == 0, "no spurious restart once converged");

    /* Opt-out: identical accel-only divergence, watchdog disabled. */
    baro_alt_config_t cfg_off         = cfg;
    cfg_off.precision_restart_disable = true;
    baro_alt_t         b2;
    baro_alt_time_us_t t2 = 1000000;
    CHECK_TRUE(baro_alt_init(&b2, &cfg_off, t2, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "init (watchdog disabled)");
    for (i = 1; i <= 500; ++i) /* 5 s accel only, past the warm-up */
    {
        t2 += US_PER_SEC / 100;
        baro_alt_update(&b2, t2, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(b2.is_initialized, "disabled watchdog never restarts");
    CHECK_TRUE(b2.n_restart == 0, "disabled watchdog counter stays 0");

    /* The height and velocity thresholds are independent OR arms: with
       the height one opted out (< 0 -> unchecked, NOT the same as the
       watchdog being disabled outright above) a tight enough velocity
       threshold must still trip it on its own. */
    baro_alt_config_t cfg_v    = cfg;
    cfg_v.restart_h_stddev_m   = -1.0f; /* unchecked */
    cfg_v.restart_v_stddev_mps = 0.05f; /* trips on velocity alone */
    baro_alt_t         b3;
    baro_alt_time_us_t t3 = 1000000;
    CHECK_TRUE(baro_alt_init(&b3, &cfg_v, t3, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "init (velocity-only watchdog)");
    for (i = 1; i <= 1500 && b3.is_initialized; ++i) /* up to 15 s */
    {
        t3 += US_PER_SEC / 100;
        baro_alt_update(&b3, t3, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_TRUE(!b3.is_initialized, "velocity-only watchdog tripped");
    CHECK_TRUE(b3.n_restart == 1, "restart counted once (velocity arm)");
}

/* ---------------------------------------------------------------------------
 * REQ-BARO-023: baro_alt_deadreckoning_ms() tracks time since the last
 * ACCEPTED barometer fusion, independently of the full 3D filter. A
 * downweighted (but not failed) fusion still counts as accepted.
 * ---------------------------------------------------------------------------
 */
static void scenario_baro_deadreckoning_ms(void)
{
    printf("\n-- scenario: vertical-channel dead-reckoning duration (REQ-BARO-023) --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float f_b[3]   = {0.0f, 0.0f, -GRAVITY}; /* static, level */

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t b;
    memset(&b, 0, sizeof(b));
    CHECK_TRUE(baro_alt_deadreckoning_ms(&b) == -1, "uninitialized: -1");

    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");
    CHECK_TRUE(baro_alt_deadreckoning_ms(&b) == 0, "the anchor sample counts as a fix");

    /* 2 s of accel-only propagation (no barometer): the dead-reckoning
       time grows with the elapsed time, not reset by a mere predict. */
    int i;
    for (i = 1; i <= 200; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_NEAR(baro_alt_deadreckoning_ms(&b), 2000, 15, "2 s without a barometer fix");

    /* A fresh (even heavily downweighted) barometer sample resets it. */
    const float p_bad = pressure_from_altitude(base_alt + 500.0f); /* gross outlier */
    baro_alt_update(&b, t, f_b, q, p_bad, 0.0f, true);
    CHECK_TRUE(baro_alt_deadreckoning_ms(&b) == 0, "a downweighted fusion still counts as a fix");

    /* Another second without the barometer. */
    for (i = 1; i <= 100; ++i)
    {
        t += US_PER_SEC / 100;
        baro_alt_update(&b, t, f_b, q, 0.0f, 0.0f, false);
    }
    CHECK_NEAR(baro_alt_deadreckoning_ms(&b), 1000, 15, "1 s without a barometer fix, again");
}

/* Initial accel-bias-prior consistency check (REQ-BARO-024): while
 * zero-velocity updates arrive the averaged up-acceleration is the
 * vertical accelerometer bias, so a bias far outside the configured
 * initial 1-sigma prior must be flagged (counter + WARN), a clean
 * accelerometer must not be, and a stale stillness marker (no ZUPT feed)
 * must stop the check from evaluating at all. */
static void scenario_baro_bias_prior_check(void)
{
    printf("\n-- scenario: initial accel-bias-prior check (REQ-BARO-024) --\n");

    const float base_alt  = 300.0f;
    const float q[4]      = {1.0f, 0.0f, 0.0f, 0.0f};
    const float acc_prior = 0.01f; /* [m/s^2], 3 sigma = 0.03 */
    const float acc_err   = 0.10f; /* [m/s^2] vertical accelerometer bias */

    int c;
    for (c = 0; c < 3; ++c)
    {
        const bool biased = (c != 0);
        const bool feed   = (c != 2); /* case 2: biased, but no ZUPT feed */

        baro_alt_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.acc_bias_init_stddev_mps2 = acc_prior;
        cfg.precision_restart_disable = true;

        baro_alt_t         b;
        baro_alt_time_us_t t = 1000000;
        CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
                   "baro_alt_init");

        int i;
        for (i = 1; i <= 1200; ++i) /* 12 s: two evaluation windows */
        {
            t += US_PER_SEC / 100;
            float f_b[3];
            body_specific_force(q, 0.0f, biased ? acc_err : 0.0f, 0.0f, f_b);
            baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
            if (feed) { baro_alt_zero_velocity_update(&b, 0.0f); }
        }

        if (c == 0)
        {
            CHECK_TRUE(b.n_acc_bias_prior_exceeded == 0,
                       "clean accelerometer inside the prior: no false alarm");
        }
        else if (c == 1)
        {
            CHECK_TRUE(b.n_acc_bias_prior_exceeded > 0, "accel bias outside the prior is flagged");
            CHECK_TRUE(b.n_zupt == 1200, "check is diagnostic only, ZUPTs still applied");
        }
        else
        {
            CHECK_TRUE(b.n_acc_bias_prior_exceeded == 0,
                       "same bias without a stillness feed is never evaluated");
        }
    }

    /* A window that HAD a ZUPT feed (t_last_zupt != 0), but not recently
       (the feed just stopped for more than the 1 s gap tolerance): the
       partial window must be discarded, same as the "never fed" case
       above but reached from the other side of the OR. */
    {
        baro_alt_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.acc_bias_init_stddev_mps2 = acc_prior;
        cfg.precision_restart_disable = true;
        baro_alt_t         b;
        baro_alt_time_us_t t = 1000000;
        CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
                   "baro_alt_init (gap)");
        int i;
        for (i = 1; i <= 200; ++i) /* 2 s: a partial window, ZUPT-fed */
        {
            t += US_PER_SEC / 100;
            float f_b[3];
            body_specific_force(q, 0.0f, acc_err, 0.0f, f_b);
            baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
            baro_alt_zero_velocity_update(&b, 0.0f);
        }
        CHECK_TRUE(b.acc_bias_prior_count > 0, "partial window accumulated some samples");
        CHECK_TRUE(b.t_last_zupt != 0, "a ZUPT was fed at least once");

        /* 1.5 s gap, no further ZUPT feed: predict steps alone still run. */
        for (i = 0; i < 150; ++i)
        {
            t += US_PER_SEC / 100;
            float f_b[3];
            body_specific_force(q, 0.0f, acc_err, 0.0f, f_b);
            baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, false);
        }
        CHECK_TRUE(b.acc_bias_prior_count == 0, "stale window is discarded, not evaluated");
        CHECK_TRUE(b.n_acc_bias_prior_exceeded == 0, "a discarded window never raises the flag");
    }

    /* The warning throttle itself repeats once the 60 s interval passes,
       for as long as the bias stays outside the prior. */
    {
        baro_alt_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.acc_bias_init_stddev_mps2 = acc_prior;
        cfg.precision_restart_disable = true;
        baro_alt_t         b;
        baro_alt_time_us_t t = 1000000;
        CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
                   "baro_alt_init (repeat)");
        int i;
        for (i = 1; i <= 1200; ++i) /* 12 s: past the first warning */
        {
            t += US_PER_SEC / 100;
            float f_b[3];
            body_specific_force(q, 0.0f, acc_err, 0.0f, f_b);
            baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
            baro_alt_zero_velocity_update(&b, 0.0f);
        }
        CHECK_TRUE(b.log_state.t_last_acc_bias_prior_warn != 0, "first warning fired");
        const baro_alt_time_us_t first_warn = b.log_state.t_last_acc_bias_prior_warn;

        for (i = 0; i < 7000; ++i) /* +70 s: past the 60 s repeat throttle */
        {
            t += US_PER_SEC / 100;
            float f_b[3];
            body_specific_force(q, 0.0f, acc_err, 0.0f, f_b);
            baro_alt_update(&b, t, f_b, q, pressure_from_altitude(base_alt), 0.0f, (i % 5) == 0);
            baro_alt_zero_velocity_update(&b, 0.0f);
        }
        CHECK_TRUE(b.log_state.t_last_acc_bias_prior_warn > first_warn,
                   "the warning repeats past the 60 s throttle");
    }
}

/* Raw measurement telemetry accessors (REQ-BARO-025): both report false
 * before their first predict/fusion step, and once available they carry
 * the RAW quantities the filter consumes (up-acceleration pre a_b
 * correction, barometric altitude pre Kalman update) rather than the
 * filtered state. */
static void scenario_baro_measurement_accessors(void)
{
    printf("\n-- scenario: raw measurement telemetry accessors (REQ-BARO-025) --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
    const float acc_err  = 0.15f; /* [m/s^2] vertical accelerometer bias, n-frame down */

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t b;
    memset(&b, 0, sizeof(b));
    float dummy;
    CHECK_TRUE(!baro_alt_get_measurement_a_z(&b, &dummy), "uninitialized: no a_z measurement");
    CHECK_TRUE(!baro_alt_get_measurement_h(&b, &dummy), "uninitialized: no h measurement");

    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "baro_alt_init");
    CHECK_TRUE(!baro_alt_get_measurement_a_z(&b, &dummy),
               "freshly initialized: no predict step yet");
    CHECK_TRUE(!baro_alt_get_measurement_h(&b, &dummy), "freshly initialized: no fusion yet");

    t += US_PER_SEC / 100;
    float f_b[3];
    body_specific_force(q, 0.0f, acc_err, 0.0f, f_b); /* static, level, biased */
    const float p = pressure_from_altitude(base_alt + 2.0f);
    baro_alt_update(&b, t, f_b, q, p, 0.0f, true);

    float a_z, h_meas, h_filtered;
    CHECK_TRUE(baro_alt_get_measurement_a_z(&b, &a_z), "a_z measurement available after predict");
    CHECK_NEAR(a_z, -acc_err, 1e-3, "a_z is the raw up-acceleration, pre a_b correction");
    CHECK_TRUE(baro_alt_get_measurement_h(&b, &h_meas), "h measurement available after fusion");
    CHECK_NEAR(h_meas, 2.0, 1e-2, "h measurement is the datum-corrected barometric altitude");
    CHECK_TRUE(baro_alt_get_height(&b, &h_filtered), "get_height");
    CHECK_TRUE(fabsf(h_filtered - h_meas) > 0.01f,
               "raw measurement is not the (still converging) filtered state");
}

/* baro_alt_update() must be exactly baro_alt_predict_step() followed by
   baro_alt_correct_step() (REQ-BARO-026): drive two identical filters
   through the same epoch stream, one via baro_alt_update(), the other
   manually split, and require the end state/covariance to match
   bit-for-bit. Also checks that phi_out carries the expected h/v/a_b
   coupling whenever the covariance is actually propagated. */
/* ---------------------------------------------------------------------------
 * MC/DC gap-filling: independent nav_suite.c guard/config arms not
 * exercised by any scenario above (each already covers the "normal"
 * arm of these conditions elsewhere).
 * ---------------------------------------------------------------------------
 */
static void scenario_nav_suite_mcdc_guards(void)
{
    printf("\n-- scenario: nav_suite guard-rail gap filling --\n");

    ins_init_t    init;
    ins_options_t opt;
    ins_time_us_t t = 1000000;
    permutation_setup(&init, &opt, t, 300.0);

    /* nav_suite_init(): NULL init/opt individually, with a valid
       instance -- elsewhere tested only with all three NULL at once. */
    static nav_suite_t s0;
    memset(&s0, 0, sizeof(s0));
    CHECK_TRUE(nav_suite_init(&s0, (const ins_init_t*)0, &opt) == -1, "init rejects NULL init");
    CHECK_TRUE(nav_suite_init(&s0, &init, (const ins_options_t*)0) == -1, "init rejects NULL opt");

    /* nav_suite_set_init_att_hint()'s ARS-yaw-seed else-if is reached
       only with stddev_yaw_rad <= 0, before the ARS bootstraps -- the
       auto_init arm is what turns it into an actual seed (elsewhere
       tested only with a positive yaw stddev, which never reaches this
       else-if at all). */
    static nav_suite_t sy;
    memset(&sy, 0, sizeof(sy));
    CHECK_TRUE(nav_suite_init(&sy, &init, &opt) == 0, "init (yaw hint, auto_init)");
    nav_suite_set_init_att_hint(&sy, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); /* stddev_yaw <= 0 */
    CHECK_TRUE(!sy.ars_yaw_from_init, "auto_init, no yaw stddev: seed flag left false");

    /* Same else-if, auto_init FALSE: nav_suite_init() itself already
       seeds ars_yaw_from_init = !auto_init = true for manual init, and
       this else-if's condition being false means the call leaves it
       exactly there (untouched), not the auto_init branch's false. */
    ins_options_t opt_no_auto = opt;
    opt_no_auto.auto_init     = false;
    static nav_suite_t sn;
    memset(&sn, 0, sizeof(sn));
    CHECK_TRUE(nav_suite_init(&sn, &init, &opt_no_auto) == 0, "init (yaw hint, manual)");
    CHECK_TRUE(sn.ars_yaw_from_init, "manual init: seed flag starts true");
    nav_suite_set_init_att_hint(&sn, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); /* stddev_yaw <= 0 */
    CHECK_TRUE(sn.ars_yaw_from_init, "manual init, no yaw stddev: seed flag left untouched");

    /* The whole seeding block (both arms above) is guarded on the ARS
       NOT being initialized yet -- reached elsewhere only before the
       first update, when it never is. Call it again after the ARS has
       bootstrapped and confirm the (now stale) hint is ignored. */
    static nav_suite_t sb;
    memset(&sb, 0, sizeof(sb));
    CHECK_TRUE(nav_suite_init(&sb, &init, &opt) == 0, "init (yaw hint, post-bootstrap)");
    {
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t + US_PER_SEC / 100;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        m.acc.data[2]      = -GRAVITY;
        nav_suite_update(&sb, &m);
    }
    CHECK_TRUE(sb.ars.is_initialized, "ARS bootstrapped on the first epoch");
    const float rpy_init_yaw_before = sb.ars_cfg.rpy_init_rad[2];
    nav_suite_set_init_att_hint(&sb, 0.0f, 0.0f, 0.0f, DEG2RAD(42.0f), DEG2RAD(5.0f));
    CHECK_NEAR(sb.ars_cfg.rpy_init_rad[2], rpy_init_yaw_before, 1e-9,
               "post-bootstrap hint does not seed the (already-run) ARS");

    /* nav_suite_correct_step() alone, with no preceding predict_step:
       step_ctx.active is false from the zeroed instance, so this must
       be a no-op, not a crash. */
    static nav_suite_t s2;
    memset(&s2, 0, sizeof(s2));
    CHECK_TRUE(nav_suite_init(&s2, &init, &opt) == 0, "init (correct_step no-op)");
    nav_suite_correct_step(&s2); /* must not crash */

    /* nav_suite_predict_step()'s GNSS validity gate: gyr invalid alone
       (acc valid) must also drop the epoch -- elsewhere tested only
       with acc invalid or both invalid together. */
    static nav_suite_t s3;
    memset(&s3, 0, sizeof(s3));
    CHECK_TRUE(nav_suite_init(&s3, &init, &opt) == 0, "init (gyr-only invalid)");
    {
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t + US_PER_SEC / 100;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = false;
        nav_suite_update(&s3, &m); /* must not crash */
    }

    /* GNSS position variance Qll_ned[8]: positive but +Inf must NOT
       latch the WGS84 anchor (isfinite() is a separate check from
       "> 0.0f", not implied by it). */
    static nav_suite_t s4;
    memset(&s4, 0, sizeof(s4));
    CHECK_TRUE(nav_suite_init(&s4, &init, &opt) == 0, "init (Qll_ned Inf)");
    {
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp           = t + US_PER_SEC / 100;
        m.strapdown_dt_sec    = 0.01f;
        m.acc.is_valid        = true;
        m.gyr.is_valid        = true;
        m.acc.data[2]         = -GRAVITY;
        m.gnss_pos.is_valid   = true;
        m.gnss_pos.llh[0]     = init.llh[0];
        m.gnss_pos.llh[1]     = init.llh[1];
        m.gnss_pos.llh[2]     = init.llh[2];
        m.gnss_pos.Qll_ned[8] = (float)INFINITY;
        nav_suite_update(&s4, &m);
        CHECK_TRUE(!s4.wgs84_anchor_seen, "+Inf position variance does not anchor WGS84");
    }

    /* Remaining accessor NULL guards, tested elsewhere only with a
       valid-but-not-ready instance. */
    {
        float  rr, pp, yy, hh, vv;
        double lt, ln, hd;
        float  ned[3] = {0.0f, 0.0f, 0.0f};
        nav_suite_set_auto_zupt_zaru_disable((nav_suite_t*)0, true); /* must not crash */
        CHECK_TRUE(!nav_suite_get_rpy_ins((const nav_suite_t*)0, &rr, &pp, &yy), "rpy_ins(NULL)");
        CHECK_TRUE(!nav_suite_get_rpy_ars((const nav_suite_t*)0, &rr, &pp, &yy), "rpy_ars(NULL)");
        CHECK_TRUE(!nav_suite_get_rpy_ahrs((const nav_suite_t*)0, &rr, &pp, &yy), "rpy_ahrs(NULL)");
        CHECK_TRUE(!nav_suite_get_vertical_zupt_active((const nav_suite_t*)0),
                   "vertical_zupt_active(NULL) false");
        CHECK_TRUE(!nav_suite_local_to_wgs84((const nav_suite_t*)0, ned, &lt, &ln, &hd),
                   "local_to_wgs84(NULL suite)");
        CHECK_TRUE(!nav_suite_local_to_wgs84(&s2, (const float*)0, &lt, &ln, &hd),
                   "local_to_wgs84(NULL ned)");
        CHECK_TRUE(!nav_suite_wgs84_to_local((const nav_suite_t*)0, 0.0, 0.0, 0.0, ned),
                   "wgs84_to_local(NULL suite)");
        CHECK_TRUE(!nav_suite_wgs84_to_local(&s2, 0.0, 0.0, 0.0, (float*)0),
                   "wgs84_to_local(NULL ned)");

        /* nav_suite_get_rpy()'s ARS fallback (neither ins ready, nor
           either attitude filter initialized yet): right after init,
           before the first update. */
        CHECK_TRUE(!nav_suite_get_rpy(&s2, &rr, &pp, &yy),
                   "rpy: no source available right after init");

        /* nav_suite_get_baro_alt(): NULL h_m and NULL v_mps requested
           independently, on an initialized baro filter. */
        static nav_suite_t s5;
        memset(&s5, 0, sizeof(s5));
        CHECK_TRUE(nav_suite_init(&s5, &init, &opt) == 0, "init (baro_alt NULL params)");
        ins_measurements_t m;
        int                i;
        for (i = 0; i < 200; ++i)
        {
            t += US_PER_SEC / 100;
            memset(&m, 0, sizeof(m));
            m.timestamp        = t;
            m.strapdown_dt_sec = 0.01f;
            m.acc.is_valid     = true;
            m.gyr.is_valid     = true;
            m.acc.data[2]      = -GRAVITY;
            m.baro.is_valid    = true;
            m.baro.pressure_pa = 101325.0f;
            nav_suite_update(&s5, &m);
        }
        CHECK_TRUE(s5.baro_alt.is_initialized, "baro_alt bootstrapped");
        CHECK_TRUE(nav_suite_get_baro_alt(&s5, (float*)0, &vv), "baro_alt: NULL h_m requested");
        CHECK_TRUE(nav_suite_get_baro_alt(&s5, &hh, (float*)0), "baro_alt: NULL v_mps requested");
    }
}

static void scenario_predict_correct_equivalence(void)
{
    printf("\n-- scenario: baro_alt_update() == "
           "baro_alt_predict_step()+baro_alt_correct_step() --\n");

    const float base_alt = 300.0f;
    const float q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};

    baro_alt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    baro_alt_t b1, b2;
    memset(&b1, 0, sizeof(b1));
    memset(&b2, 0, sizeof(b2));
    baro_alt_time_us_t t = 1000000;
    CHECK_TRUE(baro_alt_init(&b1, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "b1 init");
    CHECK_TRUE(baro_alt_init(&b2, &cfg, t, pressure_from_altitude(base_alt), 0.0f, 0.0f) == 0,
               "b2 init");

    bool  phi_checked = false;
    float phi_check[BARO_ALT_STATES * BARO_ALT_STATES];
    int   i;
    for (i = 0; i < 500; ++i)
    {
        t += US_PER_SEC / 100;
        float       f_b[3];
        const float hdd_up = 0.05f * sinf(0.01f * (float)i);
        body_specific_force(q, hdd_up, 0.0f, 0.0f, f_b);
        const float p       = pressure_from_altitude(base_alt + 0.01f * (float)i);
        const bool  baro_ok = (i % 5) == 0;

        baro_alt_update(&b1, t, f_b, q, p, 0.0f, baro_ok);

        float phi[BARO_ALT_STATES * BARO_ALT_STATES];
        memset(phi, 0, sizeof(phi));
        const int status = baro_alt_predict_step(&b2, t, f_b, q, p, 0.0f, baro_ok, phi);
        baro_alt_correct_step(&b2);

        if (!phi_checked && (status & BARO_ALT_EPOCH_COV_PROPAGATED))
        {
            memcpy(phi_check, phi, sizeof(phi));
            phi_checked = true;
        }
    }

    CHECK_TRUE(phi_checked, "phi_out was filled at least once");
    if (phi_checked)
    {
        /* Phi = [1 dt 0.5dt^2; 0 1 dt; 0 0 1], col-major: index = row + col*3. */
        const float dt = 0.01f;
        CHECK_NEAR(phi_check[0 + 1 * BARO_ALT_STATES], dt, 1e-9, "phi h/v coupling");
        CHECK_NEAR(phi_check[1 + 2 * BARO_ALT_STATES], dt, 1e-9, "phi v/a_b coupling");
    }

    float h1, h2, v1, v2;
    CHECK_TRUE(baro_alt_get_height(&b1, &h1), "b1 height readable");
    CHECK_TRUE(baro_alt_get_height(&b2, &h2), "b2 height readable");
    CHECK_TRUE(baro_alt_get_velocity(&b1, &v1), "b1 velocity readable");
    CHECK_TRUE(baro_alt_get_velocity(&b2, &v2), "b2 velocity readable");
    CHECK_NEAR(h1, h2, 0.0, "height matches");
    CHECK_NEAR(v1, v2, 0.0, "velocity matches");
    CHECK_TRUE(memcmp(b1.U, b2.U, sizeof(b1.U)) == 0, "covariance U factor matches");
    CHECK_TRUE(memcmp(b1.d, b2.d, sizeof(b1.d)) == 0, "covariance d factor matches");
    CHECK_TRUE(b1.epoch == b2.epoch, "epoch counter matches");
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    scenario_isa_conversion();
    scenario_baro_convergence();
    scenario_baro_deadreckon();
    scenario_baro_zupt();
    scenario_suite_zaru_drives_baro_zupt();
    scenario_baro_outlier();
    scenario_baro_nan_inputs();
    scenario_baro_time_anomaly();
    scenario_baro_suite();
    scenario_baro_h_init();
    scenario_baro_rate_invariant_process_noise();
    scenario_baro_h_process_noise();
    scenario_baro_datum_shift();
    scenario_offset_filter();
    scenario_offset_drift_tracking();
    scenario_offset_outlier();
    scenario_offset_nan_inputs();
    scenario_offset_low_rate_default();
    scenario_chi2_disable();
    scenario_baro_anchor_from_ins();
    scenario_baro_anchor_during_warmup();
    scenario_datum_gnss_then_baro();
    scenario_datum_baro_then_lighthouse();
    scenario_datum_offset_survives_origin_shift();
    scenario_datum_survives_quality_exit();
    scenario_height_strategy();
    scenario_height_gnss_noise_outage_cycle();
    scenario_height_ellipsoid_baro_drift();
    scenario_height_ellipsoid_no_baro();
    scenario_wgs84_conversion();
    scenario_api_guards();
    scenario_corrupted_covariance();
    scenario_baro_avg_bootstrap();
    scenario_baro_gap_and_fuse_failure();
    scenario_baro_precision_restart();
    scenario_baro_deadreckoning_ms();
    scenario_baro_bias_prior_check();
    scenario_baro_measurement_accessors();
    scenario_nav_suite_mcdc_guards();
    scenario_predict_correct_equivalence();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
