/** @file test_ahrs.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Tests for the AHRS filter (src/ahrs.c) and the nav_suite wrapper.
 *
 * Scenarios:
 *   1. PyAHRS port:  example from python/pyahrs.py __main__: noisy IMU
 *                    with gyro bias, wrong initial attitude. Roll/pitch
 *                    and the x/y gyro bias must converge.
 *   2. Free yaw:     ARS mode integrates a z rotation without
 *                    correction (directional gyro).
 *   3. Mag heading:  AHRS mode converges roll/pitch/yaw of a static
 *                    body from acc + mag, starting at 0/0/0.
 *   4. Heading fn:   ahrs_mag_heading() recovers yaw for various
 *                    attitudes.
 *   5. nav_suite:    wrapper runs ins + both AHRS filters; the mag
 *                    filter only starts once a mag sample is seen.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "ahrs.h"
#include "nav_suite.h"
#include "geodetic_toolbox.h"
#include "magnetic_model.h"
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

static uint32_t g_rng = 9001u;

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

/* Inverse of baro_alt_pressure_to_altitude: the static pressure a barometer
   would report at h_m above the ISA sea level. */
static float pressure_at_altitude(float h_m)
{
    return INS_ISA_P0_PA * powf(1.0f - h_m / INS_ISA_SCALE_M, 1.0f / INS_ISA_EXP);
}

/* ---------------------------------------------------------------------------
 * Synthesize static body-frame measurements for a given true attitude
 * ---------------------------------------------------------------------------
 */

static void body_meas_from_rpy(float roll, float pitch, float yaw, float f_b[3],
                               const float mag_n[3], float mag_b[3])
{
    float q[4], R[9];
    int   i;
    ins_quat_from_rpy(roll, pitch, yaw, q);
    ins_quat_to_rotmat(q, R);
    for (i = 0; i < 3; ++i)
    {
        /* static specific force: f_b = -R' * g_n */
        f_b[i] = -GRAVITY * MAT_ELEM(R, 2, i, 3, 3);
        if (mag_n && mag_b)
        {
            /* mag_b = R' * mag_n */
            mag_b[i] = MAT_ELEM(R, 0, i, 3, 3) * mag_n[0] + MAT_ELEM(R, 1, i, 3, 3) * mag_n[1] +
                       MAT_ELEM(R, 2, i, 3, 3) * mag_n[2];
        }
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 1: port of the pyahrs.py __main__ example
 * ---------------------------------------------------------------------------
 */

static void scenario_pyahrs_example(void)
{
    printf("\n-- scenario: pyahrs example (noisy IMU, gyro bias) --\n");

    const float rate_hz          = 100.0f;
    const float gyr_noise_psd    = 0.0001f;
    const float gyr_noise_stddev = gyr_noise_psd * sqrtf(rate_hz);
    const float acc_noise        = 0.05f;
    /* "gyro measurement" of a static body = pure gyro bias */
    const float gyro_bias[3] = {DEG2RAD(1.0f), DEG2RAD(-2.0f), 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_ARS;
    /* start with a bit of a wrong attitude (truth: 10/20/30 deg) */
    cfg.rpy_init_rad[0]             = DEG2RAD(7.0f);
    cfg.rpy_init_rad[1]             = DEG2RAD(17.0f);
    cfg.rpy_init_rad[2]             = DEG2RAD(30.0f);
    cfg.rpy_init_stddev_rad[0]      = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1]      = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2]      = DEG2RAD(0.05f);
    cfg.gyr_bias_init_stddev_rps[0] = 2.0f * fabsf(gyro_bias[0]);
    cfg.gyr_bias_init_stddev_rps[1] = 2.0f * fabsf(gyro_bias[1]);
    cfg.gyr_bias_init_stddev_rps[2] = DEG2RAD(0.01f);
    cfg.gyr_noise_psd               = gyr_noise_psd;
    cfg.acc_noise_mps2              = acc_noise;
    cfg.acc_freq_hz                 = 20.0f;
    cfg.gravity_diff_penalty        = 1.0f;
    cfg.acc_cutoff_freq_hz          = 10.0f;

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    /* this accelerometer measurement is equivalent to an attitude of
       roll=10 deg, pitch=20 deg, yaw=30 deg (from pyahrs.py) */
    const float f_true[3] = {3.3541f, -1.6002f, -9.0752f};

    int i, k;
    for (i = 0; i < 1000; ++i)
    {
        t += (ahrs_time_us_t)(US_PER_SEC / (int)rate_hz);
        float gyr[3], acc[3];
        for (k = 0; k < 3; ++k)
        {
            gyr[k] = gyro_bias[k] + gauss(gyr_noise_stddev);
            acc[k] = f_true[k] + gauss(acc_noise);
        }
        ahrs_update(&a, t, gyr, acc, (const float*)0, false);
    }

    float roll, pitch, yaw, bias[3];
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "get_rpy valid");
    CHECK_TRUE(ahrs_get_bias_gyr(&a, bias), "get_bias valid");
    printf("        roll=%.3f pitch=%.3f yaw=%.3f (deg), "
           "bias=[%.3f %.3f %.3f] (deg/s)\n",
           (double)RAD2DEG(roll), (double)RAD2DEG(pitch), (double)RAD2DEG(yaw),
           (double)RAD2DEG(bias[0]), (double)RAD2DEG(bias[1]), (double)RAD2DEG(bias[2]));

    CHECK_NEAR(RAD2DEG(roll), 10.0, 0.2, "roll converged [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 20.0, 0.2, "pitch converged [deg]");
    CHECK_NEAR(RAD2DEG(bias[0]), RAD2DEG(gyro_bias[0]), 0.1, "gyro bias x [deg/s]");
    CHECK_NEAR(RAD2DEG(bias[1]), RAD2DEG(gyro_bias[1]), 0.1, "gyro bias y [deg/s]");
    CHECK_TRUE(a.n_fuse_fail == 0, "no fusion failures");
}

/* ---------------------------------------------------------------------------
 * Scenario 2: ARS mode integrates yaw freely (directional gyro)
 * ---------------------------------------------------------------------------
 */

static void scenario_free_yaw_integration(void)
{
    printf("\n-- scenario: ARS free yaw integration --\n");

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(1.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(1.0f);

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    /* Level body rotating about z at 10 deg/s: acc stays [0,0,-g]. */
    const float omega_z = DEG2RAD(10.0f);
    const float gyr[3]  = {0.0f, 0.0f, omega_z};
    const float acc[3]  = {0.0f, 0.0f, -GRAVITY};
    const float sim_sec = 5.0f;

    int i;
    for (i = 0; i < (int)(sim_sec * 100.0f); ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr, acc, (const float*)0, false);
    }

    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "get_rpy valid");
    CHECK_NEAR(RAD2DEG(yaw), 50.0, 0.2, "yaw integrated to 50 deg");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 0.2, "roll stays level [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 0.2, "pitch stays level [deg]");
}

/* ---------------------------------------------------------------------------
 * Scenario 3: AHRS mode converges yaw to the magnetic heading
 * ---------------------------------------------------------------------------
 */

static void scenario_mag_yaw_convergence(void)
{
    printf("\n-- scenario: AHRS mag yaw convergence --\n");

    const float rpy_true[3] = {DEG2RAD(5.0f), DEG2RAD(-10.0f), DEG2RAD(60.0f)};
    const float mag_n[3]    = {20.0f, 0.0f, 44.0f}; /* Central Europe-ish */
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(rpy_true[0], rpy_true[1], rpy_true[2], f_b, mag_n, mag_b);

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_AHRS;
    /* completely wrong initial attitude estimate: 0/0/0 */
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(180.0f);

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    const float gyr[3] = {0.0f, 0.0f, 0.0f};
    int         i;
    for (i = 0; i < 2000; ++i) /* 20 s at 100 Hz */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr, f_b, mag_b, false);
    }

    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "get_rpy valid");
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(rpy_true[0]), 0.5, "roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), RAD2DEG(rpy_true[1]), 0.5, "pitch [deg]");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(rpy_true[2]), 1.0, "yaw [deg]");
    CHECK_TRUE(a.n_fuse_fail == 0, "no fusion failures");
}

/* ---------------------------------------------------------------------------
 * Scenario 3b: WMM position aiding. Declination makes yaw true-north,
 * and the field-strength gate downweights a magnitude-anomalous sample.
 * ---------------------------------------------------------------------------
 */

/* Rotate a NED field horizontally (about down) by phi and scale it by k. */
static void field_disturb(const float in_n[3], float phi, float k, float out_n[3])
{
    const float c = cosf(phi), s = sinf(phi);
    out_n[0] = k * (c * in_n[0] - s * in_n[1]);
    out_n[1] = k * (s * in_n[0] + c * in_n[1]);
    out_n[2] = k * in_n[2];
}

static void run_ahrs(ahrs_t* a, ahrs_time_us_t* t, int steps, const float mag_n[3])
{
    const float gyr[3] = {0.0f, 0.0f, 0.0f};
    const float rpy[3] = {DEG2RAD(5.0f), DEG2RAD(-8.0f), DEG2RAD(30.0f)};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(rpy[0], rpy[1], rpy[2], f_b, mag_n, mag_b);
    for (int i = 0; i < steps; ++i)
    {
        *t += US_PER_SEC / 100;
        ahrs_update(a, *t, gyr, f_b, mag_b, false);
    }
}

static void scenario_wmm_position_aiding(void)
{
    printf("\n-- scenario: AHRS WMM position aiding --\n");

    /* New York, WMM epoch start: declination ~ -12.53 deg (West). */
    const double lat = 40.712, lon = -74.006;
    const float  year = 2025.0f;
    const float  decl = magnetic_declination_deg((float)lat, (float)lon, year);
    float        mag_n[3];
    magnetic_field_ned_uT((float)lat, (float)lon, year, mag_n);

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_AHRS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(180.0f);

    /* (a) With position -> yaw converges to TRUE north (30 deg). */
    ahrs_t         a;
    ahrs_time_us_t ta = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, ta) == 0, "init a");
    ahrs_set_position(&a, (float)(lat * M_PI / 180.0), (float)(lon * M_PI / 180.0), year);
    CHECK_NEAR(RAD2DEG(a.declination_rad), decl, 0.01, "declination stored");
    run_ahrs(&a, &ta, 2500, mag_n);
    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "get_rpy a");
    CHECK_NEAR(RAD2DEG(yaw), 30.0, 1.0, "yaw true-north (decl applied)");

    /* (b) Without position -> yaw settles on the MAGNETIC heading, which
       is decl east of true north: 30 - decl. */
    ahrs_t         b;
    ahrs_time_us_t tb = 0;
    CHECK_TRUE(ahrs_init(&b, &cfg, tb) == 0, "init b");
    run_ahrs(&b, &tb, 2500, mag_n);
    CHECK_TRUE(ahrs_get_rpy(&b, &roll, &pitch, &yaw), "get_rpy b");
    CHECK_NEAR(RAD2DEG(yaw), 30.0 - (double)decl, 1.0, "yaw magnetic (no decl)");

    /* (c) Deterministic re-framing: converge to MAGNETIC north first (no
       position), then supply the position. The yaw must STEP to true
       north immediately, before any further magnetometer fusion. */
    ahrs_t         c;
    ahrs_time_us_t tc = 0;
    CHECK_TRUE(ahrs_init(&c, &cfg, tc) == 0, "init c");
    run_ahrs(&c, &tc, 2500, mag_n);
    float yaw_before;
    ahrs_get_rpy(&c, &roll, &pitch, &yaw_before);
    ahrs_set_position(&c, (float)(lat * M_PI / 180.0), (float)(lon * M_PI / 180.0), year);
    float yaw_after;
    ahrs_get_rpy(&c, &roll, &pitch, &yaw_after); /* no ahrs_update in between */
    CHECK_NEAR(RAD2DEG(yaw_before), 30.0 - (double)decl, 1.0, "pre: magnetic");
    CHECK_NEAR(RAD2DEG(yaw_after), 30.0, 1.0, "post: stepped to true north");
    CHECK_NEAR(RAD2DEG(yaw_after - yaw_before), decl, 0.05, "step equals declination (no slew)");

    /* (d) Field-strength gate (opt-in): after clean convergence, feed a
       disturbance that is both rotated (+60 deg) and magnitude-anomalous
       (x3). The gated filter must resist it far better than the default
       (gate off). */
    float dist_n[3];
    field_disturb(mag_n, DEG2RAD(60.0f), 3.0f, dist_n);

    ahrs_t         g_on, g_off;
    ahrs_time_us_t tg = 0;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_AHRS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(180.0f);
    cfg.mag_field_check_enable = true;
    CHECK_TRUE(ahrs_init(&g_on, &cfg, 0) == 0, "init g_on");
    cfg.mag_field_check_enable = false; /* default: gate off */
    CHECK_TRUE(ahrs_init(&g_off, &cfg, 0) == 0, "init g_off");
    ahrs_set_position(&g_on, (float)(lat * M_PI / 180.0), (float)(lon * M_PI / 180.0), year);
    ahrs_set_position(&g_off, (float)(lat * M_PI / 180.0), (float)(lon * M_PI / 180.0), year);
    run_ahrs(&g_on, &tg, 2500, mag_n);
    tg = 0;
    run_ahrs(&g_off, &tg, 2500, mag_n);
    /* Now the disturbance for 15 s. */
    ahrs_time_us_t td1 = g_on.t_last_gyr, td2 = g_off.t_last_gyr;
    run_ahrs(&g_on, &td1, 1500, dist_n);
    run_ahrs(&g_off, &td2, 1500, dist_n);
    float yon, yoff, r2, p2;
    ahrs_get_rpy(&g_on, &r2, &p2, &yon);
    ahrs_get_rpy(&g_off, &r2, &p2, &yoff);
    const double e_on  = fabs(RAD2DEG(yon) - 30.0);
    const double e_off = fabs(RAD2DEG(yoff) - 30.0);
    printf("   gate on  yaw err = %.2f deg, gate off yaw err = %.2f deg\n", e_on, e_off);
    /* Downweighting (not skipping) lets a persistent one-sided disturbance
       leak a little, so the gated filter still drifts a few degrees, but
       far less than the un-gated one that fuses the anomaly at full weight. */
    CHECK_TRUE(e_on < 12.0, "gated yaw resists field disturbance");
    CHECK_TRUE(e_off > 3.0 * e_on, "un-gated yaw is pulled much further");
}

/* ---------------------------------------------------------------------------
 * Scenario 4: ahrs_mag_heading() recovers yaw for various attitudes
 * ---------------------------------------------------------------------------
 */

static void scenario_mag_heading_helper(void)
{
    printf("\n-- scenario: mag heading helper --\n");

    const float mag_n[3]   = {20.0f, 0.0f, 44.0f};
    const float cases[][3] = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 45.0f},
        {10.0f, -20.0f, 135.0f},
        {-5.0f, 30.0f, -90.0f},
    };
    int i;
    for (i = 0; i < (int)(sizeof(cases) / sizeof(cases[0])); ++i)
    {
        float       f_b[3], mag_b[3];
        const float roll  = DEG2RAD(cases[i][0]);
        const float pitch = DEG2RAD(cases[i][1]);
        const float yaw   = DEG2RAD(cases[i][2]);
        body_meas_from_rpy(roll, pitch, yaw, f_b, mag_n, mag_b);
        const float heading = ahrs_mag_heading(mag_b, roll, pitch);
        char        msg[64];
        snprintf(msg, sizeof(msg), "heading @ yaw=%g deg", (double)cases[i][2]);
        CHECK_NEAR(RAD2DEG(heading), cases[i][2], 0.01, msg);

        /* Leveling heuristic should recover roll/pitch too. */
        float r, p;
        ahrs_leveling_from_acc(f_b, &r, &p);
        snprintf(msg, sizeof(msg), "leveling roll @ case %d", i);
        CHECK_NEAR(RAD2DEG(r), cases[i][0], 0.01, msg);
        snprintf(msg, sizeof(msg), "leveling pitch @ case %d", i);
        CHECK_NEAR(RAD2DEG(p), cases[i][1], 0.01, msg);
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 5: nav_suite wrapper (ins + both AHRS instances)
 * ---------------------------------------------------------------------------
 */

static void scenario_nav_suite(void)
{
    printf("\n-- scenario: nav_suite wrapper --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);

    /* ins manual init (static, Stuttgart-ish). */
    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.magnetic_n[0]             = mag_n[0];
    init.magnetic_n[1]             = mag_n[1];
    init.magnetic_n[2]             = mag_n[2];

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    /* Attitude/mag test with no position aiding: run ins in pure
       dead-reckoning so it starts on IMU alone (REQ-NAV-033). */
    opt.allow_unlimited_deadreckoning = true;

    static nav_suite_t s; /* keep the large struct off the stack */
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    float              roll, pitch, yaw;
    int                i;

    /* Phase 1: IMU only (no mag). The mag-aided instance must wait. */
    for (i = 0; i < 100; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(ahrs_get_rpy(&s.ars, &roll, &pitch, &yaw), "ars running after first IMU epoch");
    CHECK_TRUE(!ahrs_get_rpy(&s.ahrs, &roll, &pitch, &yaw), "mag-ahrs waits for first mag sample");

    /* Phase 2: IMU + mag. */
    for (i = 0; i < 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.mag.is_valid = true;
        memcpy(m.mag.data, mag_b, sizeof(mag_b));
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;
        nav_suite_update(&s, &m);
    }

    CHECK_TRUE(nav_suite_get_rpy_ins(&s, &roll, &pitch, &yaw), "ins rpy valid");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 0.5, "ins roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 0.5, "ins pitch [deg]");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true), 1.0, "ins yaw [deg]");

    CHECK_TRUE(nav_suite_get_rpy_ars(&s, &roll, &pitch, &yaw), "ars rpy valid");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 0.5, "ars roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 0.5, "ars pitch [deg]");
    /* Manual init (opt.auto_init == false): the ARS bootstraps its yaw
       from INSLIB's own known initial attitude, not an arbitrary 0, and
       (having no yaw correction) never touches it afterward. */
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true), 1.0, "ars yaw seeded from init, not 0");

    CHECK_TRUE(nav_suite_get_rpy_ahrs(&s, &roll, &pitch, &yaw), "mag-ahrs rpy valid");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 0.5, "mag-ahrs roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 0.5, "mag-ahrs pitch [deg]");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true), 1.0, "mag-ahrs yaw [deg]");

    /* Regression: auto_init (the default) must NOT seed the ARS's yaw.
       It still bootstraps blind (yaw = 0) when ins's own initial
       attitude isn't prescribed by the caller. */
    {
        ins_init_t init2;
        memset(&init2, 0, sizeof(init2));
        init2.time = t;
        ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init2.x_ecef);
        init2.pos_init_stddev_m         = 1.0f;
        init2.vel_init_stddev_mps       = 0.1f;
        init2.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
        init2.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
        init2.acc_bias_init_stddev_mps2 = 0.05f;
        init2.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);
        init2.pos_pred_stddev_m_sqrts   = 0.01f;
        init2.vel_pred_stddev_mps_sqrts = 0.05f;
        init2.rpy_pred_stddev_rad_sqrts = DEG2RAD(0.01f);
        init2.zero_vel_stddev_mps       = 0.01f;
        init2.zero_rot_stddev_rps       = DEG2RAD(0.001f);
        memcpy(init2.magnetic_n, mag_n, sizeof(mag_n));

        ins_options_t opt2;
        memset(&opt2, 0, sizeof(opt2));
        opt2.kalman_update_dt_sec               = 0.01f;
        opt2.max_prediction_time_sec            = 0.5f;
        opt2.gnss_max_horizontal_pos_stddev_m   = 10.0f;
        opt2.gnss_max_vertical_pos_stddev_m     = 20.0f;
        opt2.gnss_max_horizontal_vel_stddev_mps = 1.0f;
        opt2.gnss_max_vertical_vel_stddev_mps   = 2.0f;
        opt2.magnetometer_min_delay_ms          = 200;
        opt2.auto_init                          = true;

        static nav_suite_t s2;
        memset(&s2, 0, sizeof(s2));
        CHECK_TRUE(nav_suite_init(&s2, &init2, &opt2) == 0, "nav_suite_init (auto_init)");
        CHECK_TRUE(!s2.ars_yaw_from_init, "ars_yaw_from_init false under auto_init");

        ins_measurements_t m2;
        int                i2;
        for (i2 = 0; i2 < 10; ++i2)
        {
            t += US_PER_SEC / 100;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp        = t;
            m2.strapdown_dt_sec = 0.01f;
            m2.acc.is_valid     = true;
            m2.gyr.is_valid     = true;
            memcpy(m2.acc.data, f_b, sizeof(f_b));
            nav_suite_update(&s2, &m2);
        }
        CHECK_TRUE(ahrs_get_rpy(&s2.ars, &roll, &pitch, &yaw), "ars (auto_init) running");
        CHECK_NEAR(RAD2DEG(yaw), 0.0, 1.0, "ars yaw still bootstraps at 0 under auto_init");
    }
}

/* ---------------------------------------------------------------------------
 * Scenario 5b: ARS/AHRS attitude/gyro-bias seed ins's auto-init (REQ-SUITE-016)
 * ---------------------------------------------------------------------------
 */

/* End-to-end: the ARS runs gyro-only from the first IMU epoch and, fed an
 * explicit zero-rotation reference, converges to the platform's real gyro
 * bias well before ins ever sees a GNSS fix. ins's own auto-init has no
 * way to know about that bias on its own (it would otherwise bootstrap at
 * init.gyr_bias_init_rps, here left at the default 0) -- so the resulting ins
 * gyro bias right after bootstrap is a decisive check that nav_suite
 * actually wired the ARS's estimate into ins's att_hint (REQ-NAV-048),
 * not ins re-deriving it independently. */
static void scenario_suite_att_hint(void)
{
    printf("\n-- scenario: nav_suite ARS/AHRS attitude hint into ins auto-init "
           "(REQ-SUITE-016) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m               = 10.0f;
    init.vel_init_stddev_mps             = 1.0f;
    init.rpy_init_stddev_rad[0]          = DEG2RAD(5.0f);
    init.rpy_init_stddev_rad[1]          = DEG2RAD(5.0f);
    init.acc_bias_init_stddev_mps2       = 0.1f;
    init.gyr_bias_init_stddev_rps        = DEG2RAD(0.01f);
    init.pos_pred_stddev_m_sqrts         = 0.01f;
    init.vel_pred_stddev_mps_sqrts       = 0.05f;
    init.rpy_pred_stddev_rad_sqrts       = DEG2RAD(0.01f);
    init.acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;
    init.zero_vel_stddev_mps             = 0.01f;
    init.zero_rot_stddev_rps             = DEG2RAD(0.001f);
    init.magnetic_n[0]                   = 20.0f;
    init.magnetic_n[2]                   = 44.0f;

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
    opt.gnss_init_dwell_disable            = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* Stationary, level; a real, uncompensated gyro bias. */
    float acc_body[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, acc_body, (const float*)0, (float*)0);
    const float gbias_true[3] = {DEG2RAD(1.2f), DEG2RAD(-0.8f), DEG2RAD(0.3f)};

    ins_measurements_t m;
    int                i;
    /* ARS converges its gyro bias from an explicit zero-rotation reference
       (isolates this test from auto-ZARU detector timing). */
    for (i = 0; i < 300; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, acc_body, sizeof(acc_body));
        memcpy(m.gyr.data, gbias_true, sizeof(gbias_true));
        m.zero_rotation_update = true;
        nav_suite_update(&s, &m);
    }

    float ars_gbias[3];
    CHECK_TRUE(ahrs_get_bias_gyr(&s.ars, ars_gbias),
               "ars gyro bias available before ins bootstrap");
    CHECK_NEAR(RAD2DEG(ars_gbias[0]), RAD2DEG(gbias_true[0]), 0.1,
               "ars converged gyro bias x [deg/s]");
    CHECK_TRUE(!s.ins.is_initialized, "ins not yet bootstrapped (no fix offered)");

    /* First usable fix: ins auto-inits now. */
    t += US_PER_SEC / 100;
    memset(&m, 0, sizeof(m));
    m.timestamp        = t;
    m.strapdown_dt_sec = 0.01f;
    m.acc.is_valid     = true;
    m.gyr.is_valid     = true;
    memcpy(m.acc.data, acc_body, sizeof(acc_body));
    memcpy(m.gyr.data, gbias_true, sizeof(gbias_true));
    m.zero_rotation_update = true;
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    nav_suite_update(&s, &m);

    CHECK_TRUE(s.ins.is_initialized, "ins bootstrapped on the first fix");

    float ins_gbias[3];
    CHECK_TRUE(ins_get_bias_gyr(&s.ins, ins_gbias),
               "ins gyro bias available right after bootstrap");
    CHECK_NEAR(RAD2DEG(ins_gbias[0]), RAD2DEG(gbias_true[0]), 0.1,
               "ins gyro bias x seeded from ars, not left at 0 (REQ-SUITE-016)");
    CHECK_NEAR(RAD2DEG(ins_gbias[1]), RAD2DEG(gbias_true[1]), 0.1,
               "ins gyro bias y seeded from ars, not left at 0 (REQ-SUITE-016)");
    CHECK_NEAR(RAD2DEG(ins_gbias[2]), RAD2DEG(gbias_true[2]), 0.1,
               "ins gyro bias z seeded from ars, not left at 0 (REQ-SUITE-016)");

    float roll, pitch, yaw;
    ins_get_rpy(&s.ins, &roll, &pitch, &yaw);
    CHECK_NEAR(RAD2DEG(roll), 0.0, 1.0, "ins roll seeded near-level from ars leveling [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 1.0, "ins pitch seeded near-level from ars leveling [deg]");
}

/* A GNSS quality-loss exit (REQ-NAV-052) tears the ins instance down, so
 * the suite falls back to the ARS/AHRS for attitude and re-seeds ins from
 * them at the re-bootstrap (REQ-SUITE-016) -- the parallel filters keep
 * running through the whole window, which is what makes that possible. */
static void scenario_suite_quality_exit_rebootstrap(void)
{
    printf("\n-- scenario: nav_suite across a GNSS quality-loss re-bootstrap "
           "(REQ-SUITE-016, REQ-NAV-052) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m               = 10.0f;
    init.vel_init_stddev_mps             = 1.0f;
    init.rpy_init_stddev_rad[0]          = DEG2RAD(5.0f);
    init.rpy_init_stddev_rad[1]          = DEG2RAD(5.0f);
    init.acc_bias_init_stddev_mps2       = 0.1f;
    init.gyr_bias_init_stddev_rps        = DEG2RAD(0.01f);
    init.pos_pred_stddev_m_sqrts         = 0.01f;
    init.vel_pred_stddev_mps_sqrts       = 0.05f;
    init.rpy_pred_stddev_rad_sqrts       = DEG2RAD(0.01f);
    init.acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;
    init.zero_vel_stddev_mps             = 0.01f;
    init.zero_rot_stddev_rps             = DEG2RAD(0.001f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f; /* 0.6 stays fusable */
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.auto_init                          = true;
    opt.gnss_init_dwell_disable            = true;
    opt.gnss_stop_dwell_sec                = 2.0f;
    opt.allow_unlimited_deadreckoning      = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* Stationary but tilted: the ARS levels to this attitude from gravity,
       so a re-seed from it is distinguishable from a fresh zero. */
    const float roll_true = DEG2RAD(6.0f), pitch_true = DEG2RAD(-4.0f);
    float       acc_body[3];
    body_meas_from_rpy(roll_true, pitch_true, 0.0f, acc_body, (const float*)0, (float*)0);
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

#define SUITE_STEP(VEL_STD)                                                        \
    do {                                                                           \
        t += US_PER_SEC / 100;                                                     \
        ins_measurements_t m;                                                      \
        memset(&m, 0, sizeof(m));                                                  \
        m.timestamp        = t;                                                    \
        m.strapdown_dt_sec = 0.01f;                                                \
        m.acc.is_valid     = true;                                                 \
        m.gyr.is_valid     = true;                                                 \
        memcpy(m.acc.data, acc_body, sizeof(acc_body));                            \
        memcpy(m.gyr.data, gyr_body, sizeof(gyr_body));                            \
        if ((long)((t) % 200000) < (long)(US_PER_SEC / 100))                       \
        {                                                                          \
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];                               \
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];                               \
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];                               \
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;                  \
            m.gnss_pos.Qll_ned[8]                         = 1.0f;                  \
            m.gnss_pos.is_valid                           = true;                  \
            m.gnss_vel.Qll_ned[0]                         = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[4]                         = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[8]                         = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.is_valid                           = true;                  \
        }                                                                          \
        nav_suite_update(&s, &m);                                                  \
    } while (0)

    int i;
    for (i = 0; i < 600; ++i) { SUITE_STEP(0.1f); }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "suite reached FULL");

    /* Sustained sub-exit-gate velocity quality. */
    for (i = 0; i < 600 && s.ins.is_initialized; ++i) { SUITE_STEP(0.6f); }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_ATTITUDE_ONLY,
               "quality exit drops the suite to ATTITUDE_ONLY");

    float roll, pitch, yaw;
    CHECK_TRUE(!nav_suite_get_rpy_ins(&s, &roll, &pitch, &yaw),
               "ins reports no attitude while re-collecting");
    CHECK_TRUE(nav_suite_get_rpy(&s, &roll, &pitch, &yaw),
               "the suite still has an attitude from the ARS/AHRS");
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(roll_true), 1.5, "ARS roll carries the window [deg]");
    CHECK_NEAR(RAD2DEG(pitch), RAD2DEG(pitch_true), 1.5, "ARS pitch carries the window [deg]");

    /* Good fixes again: ins re-bootstraps and takes its attitude from the
       ARS hint rather than re-leveling from scratch. */
    for (i = 0; i < 600; ++i) { SUITE_STEP(0.1f); }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "suite back to FULL");

    float ars_roll, ars_pitch, ars_yaw;
    CHECK_TRUE(nav_suite_get_rpy_ars(&s, &ars_roll, &ars_pitch, &ars_yaw), "ars attitude");
    CHECK_TRUE(nav_suite_get_rpy_ins(&s, &roll, &pitch, &yaw), "ins attitude after re-bootstrap");
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(ars_roll), 1.5, "ins roll re-seeded from the ARS [deg]");
    CHECK_NEAR(RAD2DEG(pitch), RAD2DEG(ars_pitch), 1.5, "ins pitch re-seeded from the ARS [deg]");
#undef SUITE_STEP
}

/* A static, caller-armed initial yaw hint (nav_suite_set_init_att_hint,
 * REQ-SUITE-017) must win ins's auto-init yaw when no mag/GNSS-course
 * aiding is offered -- even once the (yaw-free) ARS has already leveled
 * roll/pitch from gravity, which must still be ARS's own value, not the
 * static hint (this test leaves the hint's roll/pitch unset, so a wrong
 * roll/pitch would mean the fallback fired where it should not have). */
static void scenario_suite_init_att_hint(void)
{
    printf("\n-- scenario: static initial attitude hint into ins auto-init "
           "(REQ-SUITE-017) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m               = 10.0f;
    init.vel_init_stddev_mps             = 1.0f;
    init.rpy_init_stddev_rad[0]          = DEG2RAD(5.0f);
    init.rpy_init_stddev_rad[1]          = DEG2RAD(5.0f);
    init.acc_bias_init_stddev_mps2       = 0.1f;
    init.gyr_bias_init_stddev_rps        = DEG2RAD(0.01f);
    init.pos_pred_stddev_m_sqrts         = 0.01f;
    init.vel_pred_stddev_mps_sqrts       = 0.05f;
    init.rpy_pred_stddev_rad_sqrts       = DEG2RAD(0.01f);
    init.acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;
    init.zero_vel_stddev_mps             = 0.01f;
    init.zero_rot_stddev_rps             = DEG2RAD(0.001f);
    init.magnetic_n[0]                   = 20.0f;
    init.magnetic_n[2]                   = 44.0f;

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
    opt.gnss_init_dwell_disable            = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    const float yaw_hint_deg = 123.0f;
    nav_suite_set_init_att_hint(&s, 0.0f, 0.0f, 0.0f, DEG2RAD(yaw_hint_deg), DEG2RAD(5.0f));
    CHECK_TRUE(s.ars_yaw_from_init, "yaw hint arms the ARS yaw seed (REQ-SUITE-002)");

    /* Tilted (not level) so leveled roll/pitch is distinguishable from an
     * unset hint's implicit 0; no mag ever offered. */
    const float roll_true  = DEG2RAD(6.0f);
    const float pitch_true = DEG2RAD(-4.0f);
    float       acc_body[3];
    body_meas_from_rpy(roll_true, pitch_true, 0.0f, acc_body, (const float*)0, (float*)0);

    ins_measurements_t m;
    int                i;
    for (i = 0; i < 300; ++i) /* 3 s: ARS levels roll/pitch from gravity */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, acc_body, sizeof(acc_body));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(s.ars.is_initialized, "ars leveled before ins bootstrap");
    CHECK_TRUE(!s.ins.is_initialized, "ins not yet bootstrapped (no fix offered)");

    float ars_roll, ars_pitch, ars_yaw;
    CHECK_TRUE(ahrs_get_rpy(&s.ars, &ars_roll, &ars_pitch, &ars_yaw), "ars rpy available");
    CHECK_NEAR(RAD2DEG(ars_roll), RAD2DEG(roll_true), 1.0, "ars leveled roll matches truth [deg]");
    /* The yaw hint is the suite's heading during ATTITUDE_ONLY too, not
       just ins's bootstrap seed: the yaw-free ARS starts from it instead
       of the arbitrary 0 (REQ-SUITE-002). The gyro reads zero here, so it
       is still sitting on that seed. */
    CHECK_NEAR(RAD2DEG(ars_yaw), yaw_hint_deg, 1.0,
               "ars yaw seeded from the static hint, not 0 (REQ-SUITE-002) [deg]");
    float suite_roll, suite_pitch, suite_yaw;
    CHECK_TRUE(nav_suite_get_rpy(&s, &suite_roll, &suite_pitch, &suite_yaw),
               "suite rpy available before ins bootstrap");
    CHECK_NEAR(RAD2DEG(suite_yaw), yaw_hint_deg, 1.0,
               "suite yaw reports the hinted heading during ATTITUDE_ONLY [deg]");

    /* First usable fix: ins auto-inits now. */
    t += US_PER_SEC / 100;
    memset(&m, 0, sizeof(m));
    m.timestamp        = t;
    m.strapdown_dt_sec = 0.01f;
    m.acc.is_valid     = true;
    m.gyr.is_valid     = true;
    memcpy(m.acc.data, acc_body, sizeof(acc_body));
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    nav_suite_update(&s, &m);

    CHECK_TRUE(s.ins.is_initialized, "ins bootstrapped on the first fix");

    float roll, pitch, yaw;
    ins_get_rpy(&s.ins, &roll, &pitch, &yaw);
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(roll_true), 1.0,
               "ins roll seeded from ars leveling, NOT the (unset) hint [deg]");
    CHECK_NEAR(RAD2DEG(pitch), RAD2DEG(pitch_true), 1.0,
               "ins pitch seeded from ars leveling, NOT the (unset) hint [deg]");
    CHECK_NEAR(RAD2DEG(yaw), yaw_hint_deg, 1.0,
               "ins yaw seeded from the static hint (ars has none) (REQ-SUITE-017) [deg]");

    /* Consumed exactly once (REQ-SUITE-017): cleared the moment ins
       bootstrapped, so it does not silently reapply at a later full
       re-bootstrap (simulated here the same way a severe time-jump reset
       would leave things: is_initialized/is_collecting flipped back). */
    CHECK_TRUE(s.init_att_hint.stddev_yaw_rad <= 0.0f,
               "static hint cleared after the first bootstrap");

    s.ins.is_initialized = false;
    s.ins.is_collecting  = true;
    t += US_PER_SEC / 100;
    memset(&m, 0, sizeof(m));
    m.timestamp        = t;
    m.strapdown_dt_sec = 0.01f;
    m.acc.is_valid     = true;
    m.gyr.is_valid     = true;
    memcpy(m.acc.data, acc_body, sizeof(acc_body));
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    nav_suite_update(&s, &m);

    CHECK_TRUE(s.ins.is_initialized, "ins re-bootstrapped after the simulated reset");
    ins_get_rpy(&s.ins, &roll, &pitch, &yaw);
    CHECK_TRUE(fabsf(RAD2DEG(yaw) - yaw_hint_deg) > 1.0,
               "re-bootstrap does NOT reuse the stale static yaw hint (REQ-SUITE-017)");
}

/* The roll/pitch half of the same static hint (REQ-SUITE-017). Unlike
 * the yaw half it is deliberately NOT pushed into the ARS: the ARS
 * levels itself from gravity on its very first epoch, which beats any
 * static assumption, so the stored roll/pitch is only a fallback for a
 * bootstrap that has no leveled filter to ask. A roll/pitch-only hint
 * must therefore leave the ARS yaw seed alone as well.
 */
static void scenario_suite_init_att_hint_roll_pitch(void)
{
    printf("\n-- scenario: static initial roll/pitch hint (REQ-SUITE-017) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m               = 10.0f;
    init.vel_init_stddev_mps             = 1.0f;
    init.rpy_init_stddev_rad[0]          = DEG2RAD(20.0f);
    init.rpy_init_stddev_rad[1]          = DEG2RAD(20.0f);
    init.acc_bias_init_stddev_mps2       = 0.1f;
    init.gyr_bias_init_stddev_rps        = DEG2RAD(0.01f);
    init.pos_pred_stddev_m_sqrts         = 0.01f;
    init.vel_pred_stddev_mps_sqrts       = 0.05f;
    init.rpy_pred_stddev_rad_sqrts       = DEG2RAD(0.01f);
    init.acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;
    init.zero_vel_stddev_mps             = 0.01f;
    init.zero_rot_stddev_rps             = DEG2RAD(0.001f);
    init.magnetic_n[0]                   = 20.0f;
    init.magnetic_n[2]                   = 44.0f;

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.auto_init                          = true;
    opt.gnss_init_dwell_disable            = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* Armed before any epoch: nothing has leveled yet, which is the only
       situation this hint exists for. */
    const float hint_roll  = DEG2RAD(8.0f);
    const float hint_pitch = DEG2RAD(-5.0f);
    nav_suite_set_init_att_hint(&s, hint_roll, hint_pitch, DEG2RAD(2.0f), 0.0f, 0.0f);
    CHECK_NEAR(RAD2DEG(s.init_att_hint.roll_rad), RAD2DEG(hint_roll), 1e-4,
               "roll stored on the hint [deg]");
    CHECK_NEAR(RAD2DEG(s.init_att_hint.pitch_rad), RAD2DEG(hint_pitch), 1e-4,
               "pitch stored on the hint [deg]");
    CHECK_TRUE(s.init_att_hint.stddev_roll_rad > 0.0f && s.init_att_hint.stddev_pitch_rad > 0.0f,
               "the shared roll/pitch stddev lands on both axes");
    CHECK_TRUE(s.init_att_hint.stddev_yaw_rad <= 0.0f, "no yaw hint from a roll/pitch-only call");
    CHECK_TRUE(!s.ars_yaw_from_init, "a roll/pitch-only hint arms no ARS yaw seed");

    /* A zero roll/pitch stddev is the "no roll/pitch hint" spelling and
       must not store an implicit 0/0 attitude. */
    nav_suite_set_init_att_hint(&s, hint_roll, hint_pitch, 0.0f, DEG2RAD(90.0f), DEG2RAD(5.0f));
    CHECK_TRUE(s.init_att_hint.stddev_roll_rad <= 0.0f, "a zero stddev stores no roll hint");
    CHECK_TRUE(s.init_att_hint.stddev_yaw_rad > 0.0f, "but the yaw half of the same call is kept");
    CHECK_TRUE(s.ars_yaw_from_init, "and that yaw does seed the ARS (REQ-SUITE-002)");

    /* Re-arming with neither half disarms the ARS seed again rather than
       leaving the previous call's yaw standing. */
    nav_suite_set_init_att_hint(&s, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    CHECK_TRUE(!s.ars_yaw_from_init, "an empty hint disarms the ARS yaw seed");
    CHECK_TRUE(s.init_att_hint.stddev_yaw_rad <= 0.0f, "and clears the stored yaw");

    /* NULL suite: a guard, not a crash. */
    nav_suite_set_init_att_hint((nav_suite_t*)0, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f);

    /* The ARS levels itself from gravity regardless of the roll/pitch
       hint: feed a genuinely tilted platform and the ARS must report the
       real tilt, not the (unset, and earlier deliberately wrong) hint. */
    nav_suite_set_init_att_hint(&s, hint_roll, hint_pitch, DEG2RAD(2.0f), 0.0f, 0.0f);
    const float roll_true  = DEG2RAD(-3.0f);
    const float pitch_true = DEG2RAD(2.0f);
    float       acc_body[3];
    body_meas_from_rpy(roll_true, pitch_true, 0.0f, acc_body, (const float*)0, (float*)0);

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
        memcpy(m.acc.data, acc_body, sizeof(acc_body));
        nav_suite_update(&s, &m);
    }
    float ars_roll, ars_pitch, ars_yaw;
    CHECK_TRUE(ahrs_get_rpy(&s.ars, &ars_roll, &ars_pitch, &ars_yaw), "ars rpy available");
    CHECK_NEAR(RAD2DEG(ars_roll), RAD2DEG(roll_true), 1.0,
               "ars roll comes from gravity, not the hint [deg]");
    CHECK_NEAR(RAD2DEG(ars_pitch), RAD2DEG(pitch_true), 1.0,
               "ars pitch comes from gravity, not the hint [deg]");
}

/* ---------------------------------------------------------------------------
 * Scenario 5c: heading carry-over across an ins re-initialization
 * ---------------------------------------------------------------------------
 */

/* One IMU epoch of the carry-over scenario: level platform, optional yaw
 * rate about the body z axis, optional GNSS fix, optional external heading. */
static void yaw_carry_epoch(nav_suite_t* s, ahrs_time_us_t t, float yaw_rate_rps,
                            const double x_ecef[3], bool with_fix, const float* yaw_meas_rad)
{
    ins_measurements_t m;
    float              acc[3];

    body_meas_from_rpy(0.0f, 0.0f, 0.0f, acc, (const float*)0, (float*)0);

    memset(&m, 0, sizeof(m));
    m.timestamp        = t;
    m.strapdown_dt_sec = 0.01f;
    m.acc.is_valid     = true;
    m.gyr.is_valid     = true;
    memcpy(m.acc.data, acc, sizeof(acc));
    m.gyr.data[2] = yaw_rate_rps;

    if (with_fix)
    {
        m.gnss_pos.xyz_ecef[0] = x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = x_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        m.gnss_pos.is_valid                           = true;
    }
    if (yaw_meas_rad != (const float*)0)
    {
        m.yaw.yaw_rad    = *yaw_meas_rad;
        m.yaw.stddev_rad = DEG2RAD(1.0f);
        m.yaw.is_valid   = true;
    }
    nav_suite_update(s, &m);
}

static void scenario_suite_yaw_carry(void)
{
    printf("\n-- scenario: ins re-init heading = last 3D heading + ARS delta "
           "(REQ-SUITE-022) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m               = 10.0f;
    init.vel_init_stddev_mps             = 1.0f;
    init.rpy_init_stddev_rad[0]          = DEG2RAD(5.0f);
    init.rpy_init_stddev_rad[1]          = DEG2RAD(5.0f);
    init.acc_bias_init_stddev_mps2       = 0.1f;
    init.gyr_bias_init_stddev_rps        = DEG2RAD(0.01f);
    init.pos_pred_stddev_m_sqrts         = 0.01f;
    init.vel_pred_stddev_mps_sqrts       = 0.05f;
    init.rpy_pred_stddev_rad_sqrts       = DEG2RAD(0.01f);
    init.acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;
    init.zero_vel_stddev_mps             = 0.01f;
    init.zero_rot_stddev_rps             = DEG2RAD(0.001f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.auto_init                          = true;
    opt.gnss_init_dwell_disable            = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* Phase 1: ins runs aided, with an external heading so its own yaw
       really converges (no magnetometer anywhere in this scenario, so the
       yaw-free ARS is the only other attitude source). */
    const float yaw_true = DEG2RAD(40.0f);
    int         i;
    for (i = 0; i < 500; ++i) /* 5 s */
    {
        t += US_PER_SEC / 100;
        yaw_carry_epoch(&s, t, 0.0f, init.x_ecef, true, &yaw_true);
    }

    CHECK_TRUE(ins_is_ready(&s.ins), "ins ready and aided before the shutdown");
    CHECK_TRUE(s.yaw_carry.valid, "heading carry-over latched while ins was converged");
    CHECK_NEAR(RAD2DEG(s.yaw_carry.yaw_ins_rad), RAD2DEG(yaw_true), 1.0,
               "latched heading is the converged 3D heading [deg]");

    float roll_sd, pitch_sd, yaw_sd;
    CHECK_TRUE(ins_get_rpy_stddev(&s.ins, &roll_sd, &pitch_sd, &yaw_sd),
               "ins rpy stddev available");
    CHECK_TRUE(RAD2DEG(yaw_sd) < 15.0f, "ins yaw converged past the latch gate");

    /* Phase 2: ins shuts down (a quality-loss re-arm or a health reset
       leaves exactly this: uninitialized, collecting again), and the
       platform turns while it is down. Only the ARS sees that turn. */
    s.ins.is_initialized = false;
    s.ins.is_collecting  = true;

    /* Above the ARS's stillness bound: a perfectly constant, noise-free
       rate below it reads as a standstill to a velocity-blind detector
       (REQ-AHRS-017) and would be absorbed into the gyro bias. */
    const float turn_deg     = 30.0f;
    const float yaw_rate_rps = DEG2RAD(10.0f);
    for (i = 0; i < 300; ++i) /* 3 s * 10 deg/s = 30 deg */
    {
        t += US_PER_SEC / 100;
        yaw_carry_epoch(&s, t, yaw_rate_rps, init.x_ecef, false, (const float*)0);
    }

    float ars_roll, ars_pitch, ars_yaw;
    CHECK_TRUE(ahrs_get_rpy(&s.ars, &ars_roll, &ars_pitch, &ars_yaw), "ars rpy available");
    CHECK_NEAR(RAD2DEG(ins_angle_diff(ars_yaw, s.yaw_carry.yaw_ars_rad)), turn_deg, 1.0,
               "ars tracked the turn while ins was down [deg]");
    CHECK_TRUE(s.yaw_carry.valid, "carry-over survives the outage");

    /* ATTITUDE_ONLY: the suite reports the carried heading, not the ARS's
       own seed-relative yaw (which started at 0 here, not at yaw_true). */
    float suite_roll, suite_pitch, suite_yaw;
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_ATTITUDE_ONLY, "mode ATTITUDE_ONLY");
    CHECK_TRUE(nav_suite_get_rpy(&s, &suite_roll, &suite_pitch, &suite_yaw), "suite rpy available");
    CHECK_NEAR(RAD2DEG(suite_yaw), RAD2DEG(yaw_true) + turn_deg, 2.0,
               "suite heading = last 3D heading + ARS delta (REQ-SUITE-022) [deg]");
    CHECK_NEAR(RAD2DEG(ars_yaw), turn_deg, 1.0,
               "the ars's own yaw is still the raw, seed-relative one [deg]");
    float raw_roll, raw_pitch, raw_yaw;
    CHECK_TRUE(nav_suite_get_rpy_ars(&s, &raw_roll, &raw_pitch, &raw_yaw),
               "ars accessor available");
    CHECK_NEAR(RAD2DEG(raw_yaw), RAD2DEG(ars_yaw), 0.01,
               "nav_suite_get_rpy_ars keeps reporting the raw ars yaw [deg]");

    /* Phase 3: a fix comes back, ins re-initializes. No heading source is
       offered with it, so without the carry-over this would be the
       "initial heading unknown" case. */
    t += US_PER_SEC / 100;
    yaw_carry_epoch(&s, t, yaw_rate_rps, init.x_ecef, true, (const float*)0);

    CHECK_TRUE(s.ins.is_initialized, "ins re-initialized on the returning fix");

    float roll, pitch, yaw;
    CHECK_TRUE(ins_get_rpy(&s.ins, &roll, &pitch, &yaw), "ins rpy available after the re-init");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true) + turn_deg, 2.0,
               "re-init heading = last 3D heading + ARS delta (REQ-SUITE-022) [deg]");
    CHECK_TRUE(ins_get_rpy_stddev(&s.ins, &roll_sd, &pitch_sd, &yaw_sd),
               "ins rpy stddev available after the re-init");
    CHECK_TRUE(RAD2DEG(yaw_sd) < 30.0f, "re-init yaw prior is a real prior, not 'unknown'");

    /* A re-bootstrapping ARS restarts its yaw from the config seed, so the
       reference the carry-over subtracts against is gone. */
    s.ars.is_initialized = false;
    t += US_PER_SEC / 100;
    yaw_carry_epoch(&s, t, 0.0f, init.x_ecef, true, (const float*)0);
    CHECK_TRUE(!s.yaw_carry.valid, "carry-over dropped when the ars re-bootstraps");
}

/* ---------------------------------------------------------------------------
 * Scenario 5d: heading does not jump when nav_suite_get_rpy() switches
 * between ins and the magnetometer AHRS (REQ-SUITE-005)
 *
 * scenario_tunnel already drives this exact mode sequence and checks each
 * mode's yaw against ground truth separately; it never checks the two
 * numbers against EACH OTHER at the instant of the switch, which is the
 * property a consumer actually cares about (a sudden heading step
 * commanded into a controller). Both ins and the mag AHRS are independent
 * estimators of the same true heading here, so nothing (unlike the
 * carry-over of REQ-SUITE-022, which only applies without a magnetometer)
 * explicitly reconciles them - this only holds because both filters
 * converge to the truth.
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_heading_switch_no_jump(void)
{
    printf("\n-- scenario: heading does not jump across an ins/AHRS mode "
           "switch (REQ-SUITE-005) --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.magnetic_n[0]             = mag_n[0];
    init.magnetic_n[1]             = mag_n[1];
    init.magnetic_n[2]             = mag_n[2];

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    /* default coasting window (10 s), no unlimited deadreckoning */

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    int                i;
    nav_suite_mode_t   mode_prev       = nav_suite_get_mode(&s);
    float              yaw_prev        = 0.0f;
    float              yaw_before_loss = 0.0f, yaw_after_loss = 0.0f;
    float              yaw_before_reacq = 0.0f, yaw_after_reacq = 0.0f;
    bool               loss_seen = false, reacq_seen = false;

#define STEP(WITH_GNSS)                                                     \
    do {                                                                    \
        t += US_PER_SEC / 100;                                              \
        memset(&m, 0, sizeof(m));                                           \
        m.timestamp        = t;                                             \
        m.strapdown_dt_sec = 0.01f;                                         \
        m.acc.is_valid     = true;                                          \
        m.gyr.is_valid     = true;                                          \
        memcpy(m.acc.data, f_b, sizeof(f_b));                               \
        m.mag.is_valid = true;                                              \
        memcpy(m.mag.data, mag_b, sizeof(mag_b));                           \
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;   \
        if (WITH_GNSS)                                                      \
        {                                                                   \
            m.gnss_pos.is_valid = true;                                     \
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));  \
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;          \
            m.gnss_pos.Qll_ned[8]                         = 1.0f;           \
        }                                                                   \
        nav_suite_update(&s, &m);                                           \
        float rr, pp, yy;                                                   \
        if (nav_suite_get_rpy(&s, &rr, &pp, &yy))                           \
        {                                                                   \
            const nav_suite_mode_t mode_now = nav_suite_get_mode(&s);       \
            if (!loss_seen && mode_prev == NAV_SUITE_MODE_COASTING &&       \
                mode_now == NAV_SUITE_MODE_ATTITUDE_ONLY)                   \
            {                                                               \
                yaw_before_loss = yaw_prev;                                 \
                yaw_after_loss  = yy;                                       \
                loss_seen       = true;                                     \
            }                                                               \
            if (!reacq_seen && mode_prev == NAV_SUITE_MODE_ATTITUDE_ONLY && \
                mode_now == NAV_SUITE_MODE_FULL)                            \
            {                                                               \
                yaw_before_reacq = yaw_prev;                                \
                yaw_after_reacq  = yy;                                      \
                reacq_seen       = true;                                    \
            }                                                               \
            mode_prev = mode_now;                                           \
            yaw_prev  = yy;                                                 \
        }                                                                   \
    } while (0)

    /* Aided phase: both ins and the mag AHRS converge on yaw_true. */
    for (i = 0; i < 500; ++i) { STEP(1); }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "FULL before the outage");

    /* GNSS gone: coasting, then attitude-only once the window expires. */
    for (i = 0; i < 1500; ++i) { STEP(0); }
    CHECK_TRUE(loss_seen, "the suite handed off from ins to the mag AHRS");
    CHECK_TRUE(fabsf(RAD2DEG(ins_angle_diff(yaw_after_loss, yaw_before_loss))) < 2.0f,
               "no significant heading jump when ins hands off to the mag AHRS [deg]");

    /* GNSS returns: the first usable fix re-acquires ins. */
    for (i = 0; i < 200 && !reacq_seen; ++i) { STEP(1); }
    CHECK_TRUE(reacq_seen, "the suite re-acquired ins after the outage");
    CHECK_TRUE(fabsf(RAD2DEG(ins_angle_diff(yaw_after_reacq, yaw_before_reacq))) < 2.0f,
               "no significant heading jump when ins re-acquires from the mag AHRS [deg]");
#undef STEP
}

/* ---------------------------------------------------------------------------
 * Scenario 5e: nav_suite ins/AHRS heading cross-check (REQ-SUITE-023)
 *
 * ins is pinned to yaw_true by a tightly-trusted external yaw aid
 * (m.yaw), while the magnetometer is deliberately fed a LOOSE reported
 * covariance for ins's own fusion (a caller that does not trust its
 * compass much) but drives the AHRS at full strength through its own,
 * independent internal noise model. A field rotated by 25 deg then pulls
 * the AHRS off while ins, dominated by the external aid, barely moves --
 * a realistic split (e.g. a dual-antenna GNSS heading feeding ins
 * directly while a nearby motor disturbs the compass).
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_heading_cross_check(void)
{
    printf("\n-- scenario: nav_suite ins/AHRS heading cross-check "
           "(REQ-SUITE-023) --\n");

    const float yaw_true      = DEG2RAD(30.0f);
    const float yaw_disturbed = DEG2RAD(55.0f); /* 25 deg off: past the 15 deg bound */
    const float mag_n[3]      = {20.0f, 0.0f, 44.0f};
    float       f_b[3], dummy[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0,
                       (float*)0); /* level, yaw-invariant */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.magnetic_n[0]             = mag_n[0];
    init.magnetic_n[1]             = mag_n[1];
    init.magnetic_n[2]             = mag_n[2];

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

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    int                i;
    float              mag_b[3];

#define HEADING_STEP(YAW_MAG)                                                      \
    do {                                                                           \
        t += US_PER_SEC / 100;                                                     \
        memset(&m, 0, sizeof(m));                                                  \
        m.timestamp        = t;                                                    \
        m.strapdown_dt_sec = 0.01f;                                                \
        m.acc.is_valid     = true;                                                 \
        m.gyr.is_valid     = true;                                                 \
        memcpy(m.acc.data, f_b, sizeof(f_b));                                      \
        body_meas_from_rpy(0.0f, 0.0f, (YAW_MAG), dummy, mag_n, mag_b);            \
        m.mag.is_valid = true;                                                     \
        memcpy(m.mag.data, mag_b, sizeof(mag_b));                                  \
        /* Loose on purpose: ins is told not to trust this axis much, so it        \
           stays dominated by the tight external yaw aid below. The AHRS           \
           never sees this value -- it uses its own internal noise model. */       \
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 2500.0f;       \
        m.yaw.yaw_rad                                             = yaw_true;      \
        m.yaw.stddev_rad                                          = DEG2RAD(0.5f); \
        m.yaw.is_valid                                            = true;          \
        if ((i % 100) == 0)                                                        \
        {                                                                          \
            m.gnss_pos.is_valid = true;                                            \
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));         \
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;                 \
            m.gnss_pos.Qll_ned[8]                         = 1.0f;                  \
        }                                                                          \
        nav_suite_update(&s, &m);                                                  \
    } while (0)

    /* Phase 1: the external yaw aid and the magnetometer agree - both
       filters converge on the same heading, no cross-check warning. */
    for (i = 0; i < 1000; ++i) { HEADING_STEP(yaw_true); } /* 10 s */
    CHECK_TRUE(s.ins.is_initialized && s.ahrs.is_initialized, "both filters running");

    float roll, pitch, ins_yaw, ahrs_yaw;
    CHECK_TRUE(ins_get_rpy(&s.ins, &roll, &pitch, &ins_yaw), "ins yaw available");
    CHECK_TRUE(ahrs_get_rpy(&s.ahrs, &roll, &pitch, &ahrs_yaw), "ahrs yaw available");
    CHECK_NEAR(RAD2DEG(ins_yaw), RAD2DEG(yaw_true), 2.0,
               "ins converged on the agreed heading [deg]");
    CHECK_NEAR(RAD2DEG(ahrs_yaw), RAD2DEG(yaw_true), 2.0,
               "ahrs converged on the agreed heading [deg]");
    CHECK_TRUE(s.log_state.t_last_yaw_warn == 0, "no cross-check warning while they agree");

    /* Phase 2: a local magnetic disturbance rotates the field by 25 deg.
       ins stays on the independent, tightly-trusted external yaw aid; the
       AHRS has nothing else and follows the disturbed field, slowly (its
       own covariance already settled in phase 1). */
    for (i = 0; i < 6000; ++i) { HEADING_STEP(yaw_disturbed); } /* 60 s */

    CHECK_TRUE(ins_get_rpy(&s.ins, &roll, &pitch, &ins_yaw), "ins yaw available");
    CHECK_TRUE(ahrs_get_rpy(&s.ahrs, &roll, &pitch, &ahrs_yaw), "ahrs yaw available");
    CHECK_NEAR(RAD2DEG(ins_yaw), RAD2DEG(yaw_true), 2.0, "ins held the external yaw aid [deg]");
    CHECK_NEAR(RAD2DEG(ahrs_yaw), RAD2DEG(yaw_disturbed), 5.0,
               "ahrs followed the disturbed field [deg]");
    CHECK_TRUE(s.log_state.t_last_yaw_warn != 0, "heading cross-check warning raised");
#undef HEADING_STEP
}

/* ---------------------------------------------------------------------------
 * Scenario 6: non-finite inputs are dropped, the filter survives
 * ---------------------------------------------------------------------------
 */

static void scenario_nan_inputs(void)
{
    printf("\n-- scenario: non-finite inputs dropped --\n");

    const float rpy_true[3] = {DEG2RAD(5.0f), DEG2RAD(-10.0f), DEG2RAD(60.0f)};
    const float mag_n[3]    = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(rpy_true[0], rpy_true[1], rpy_true[2], f_b, mag_n, mag_b);

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_AHRS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(10.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(180.0f);

    /* Init must reject a non-finite initial attitude. */
    ahrs_t        a;
    ahrs_config_t bad   = cfg;
    bad.rpy_init_rad[0] = NAN;
    CHECK_TRUE(ahrs_init(&a, &bad, 0) != 0, "init rejects NaN attitude");

    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    const float gyr[3]  = {0.0f, 0.0f, 0.0f};
    const float bad3[3] = {NAN, 0.0f, INFINITY};
    int         i;
    for (i = 0; i < 2000; ++i) /* 20 s at 100 Hz, with periodic glitches */
    {
        t += US_PER_SEC / 100;
        if (i % 50 == 10) /* NaN accelerometer epoch */
        {
            ahrs_update(&a, t, gyr, bad3, mag_b, false);
        }
        else if (i % 50 == 20) /* Inf gyro epoch */ { ahrs_update(&a, t, bad3, f_b, mag_b, false); }
        else if (i % 50 == 30) /* NaN magnetometer (gyro/acc fine) */
        {
            ahrs_update(&a, t, gyr, f_b, bad3, false);
        }
        else { ahrs_update(&a, t, gyr, f_b, mag_b, false); }
    }

    CHECK_TRUE(a.is_initialized, "filter survived the glitches");
    CHECK_TRUE(a.n_invalid_input == 3 * (2000 / 50), "all corrupt samples counted");
    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "get_rpy valid");
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(rpy_true[0]), 0.5, "roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), RAD2DEG(rpy_true[1]), 0.5, "pitch [deg]");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(rpy_true[2]), 1.0, "yaw [deg]");
}

/* ---------------------------------------------------------------------------
 * Scenario 7: tunnel passage: mode arbitration and attitude fallback
 * ---------------------------------------------------------------------------
 */

static void scenario_tunnel(void)
{
    printf("\n-- scenario: tunnel passage (mode arbitration) --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.magnetic_n[0]             = mag_n[0];
    init.magnetic_n[1]             = mag_n[1];
    init.magnetic_n[2]             = mag_n[2];

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    /* default coasting window (10 s), no unlimited deadreckoning */

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    float              roll, pitch, yaw;
    int                i;

#define RUN_EPOCHS(N, WITH_GNSS)                                               \
    do {                                                                       \
        for (i = 0; i < (N); ++i)                                              \
        {                                                                      \
            t += US_PER_SEC / 100;                                             \
            memset(&m, 0, sizeof(m));                                          \
            m.timestamp        = t;                                            \
            m.strapdown_dt_sec = 0.01f;                                        \
            m.acc.is_valid     = true;                                         \
            m.gyr.is_valid     = true;                                         \
            memcpy(m.acc.data, f_b, sizeof(f_b));                              \
            m.mag.is_valid = true;                                             \
            memcpy(m.mag.data, mag_b, sizeof(mag_b));                          \
            m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;  \
            if ((WITH_GNSS) && (i % 100 == 0)) /* 1 Hz */                      \
            {                                                                  \
                m.gnss_pos.is_valid = true;                                    \
                memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef)); \
                m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;         \
                m.gnss_pos.Qll_ned[8]                         = 1.0f;          \
            }                                                                  \
            nav_suite_update(&s, &m);                                          \
        }                                                                      \
    } while (0)

    /* Aided phase. */
    RUN_EPOCHS(300, 1);
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "mode FULL while aided");

    /* Tunnel entry: coasting first ... */
    RUN_EPOCHS(500, 0); /* 5 s outage */
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_COASTING,
               "mode COASTING inside the window");
    CHECK_TRUE(nav_suite_get_rpy(&s, &roll, &pitch, &yaw), "attitude available while coasting");

    /* ... then attitude-only from the AHRS filters. */
    RUN_EPOCHS(700, 0); /* 12 s total outage */
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_ATTITUDE_ONLY,
               "mode ATTITUDE_ONLY after the window");
    CHECK_TRUE(nav_suite_get_rpy(&s, &roll, &pitch, &yaw),
               "attitude still available (AHRS fallback)");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 0.5, "fallback roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 0.5, "fallback pitch [deg]");
    CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true), 1.0, "fallback yaw from mag [deg]");

    /* Tunnel exit: first fix re-acquires, mode returns to FULL. */
    RUN_EPOCHS(101, 1);
    CHECK_TRUE(ins_get_diag(&s.ins)->n_reacquire == 1, "ins re-acquired");
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_FULL, "mode FULL after tunnel exit");
    float pos[3];
    ins_get_position_local(&s.ins, pos);
    CHECK_NEAR(pos[0], 0.0, 1.0, "position back on the fix (north) [m]");
    CHECK_NEAR(pos[1], 0.0, 1.0, "position back on the fix (east) [m]");
#undef RUN_EPOCHS
}

/* ---------------------------------------------------------------------------
 * Scenario 8: time anomalies (REQ-AHRS-011). A backwards step re-anchors
 * the clocks and skips the epoch. A gap > 0.2 s skips the attitude
 * integration (sensor-outage semantics). The filter stays healthy throughout.
 * ---------------------------------------------------------------------------
 */

static void scenario_time_anomaly(void)
{
    printf("\n-- scenario: time anomaly handling --\n");

    const float rpy_true[3] = {DEG2RAD(10.0f), DEG2RAD(20.0f), 0.0f};
    float       f_b[3];
    body_meas_from_rpy(rpy_true[0], rpy_true[1], rpy_true[2], f_b, (const float*)0, (float*)0);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_rad[0]        = rpy_true[0];
    cfg.rpy_init_rad[1]        = rpy_true[1];
    cfg.rpy_init_rad[2]        = rpy_true[2];
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(3.0f);

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    int i;
    for (i = 0; i < 200; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, (const float*)0, false);
    }

    float roll0, pitch0, yaw0, roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll0, &pitch0, &yaw0), "rpy valid");

    /* Backwards step with a large (bogus) rotation rate: the epoch must
       be skipped entirely, the attitude stays bit-identical. */
    const float gyr_big[3] = {0.0f, 0.0f, 1.0f};
    ahrs_update(&a, t - US_PER_SEC / 2, gyr_big, f_b, (const float*)0, false);
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "still healthy after backwards step");
    CHECK_NEAR(yaw, yaw0, 1e-9, "backwards step: yaw untouched [rad]");
    CHECK_NEAR(roll, roll0, 1e-9, "backwards step: roll untouched [rad]");

    /* The backwards step re-anchored t_last_gyr; continue from there. */
    ahrs_time_us_t t2 = t - US_PER_SEC / 2;

    /* Forward gap of 1 s (> 0.2 s) with omega_z = 1 rad/s: integrating
       across the gap would yaw by ~57 deg, so the integration must be
       skipped for that epoch instead. */
    CHECK_TRUE(ahrs_get_rpy(&a, &roll0, &pitch0, &yaw0), "rpy valid");
    t2 += US_PER_SEC;
    ahrs_update(&a, t2, gyr_big, f_b, (const float*)0, false);
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "still healthy after forward gap");
    CHECK_TRUE(fabsf(ins_angle_diff(yaw, yaw0)) < DEG2RAD(1.0f),
               "forward gap: attitude integration skipped");

    /* Normal operation continues after the anomalies. */
    for (i = 0; i < 200; ++i)
    {
        t2 += US_PER_SEC / 100;
        ahrs_update(&a, t2, gyr0, f_b, (const float*)0, false);
    }
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "recovered");
    CHECK_NEAR(RAD2DEG(roll), 10.0, 0.5, "roll tracks truth [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 20.0, 0.5, "pitch tracks truth [deg]");
    CHECK_TRUE(a.n_invalid_input == 0, "no inputs flagged invalid");
    CHECK_TRUE(a.n_fuse_fail == 0, "no fusion failures");
}

/* ---------------------------------------------------------------------------
 * Scenario 9: magnetometer edge cases (REQ-AHRS-006). Zero field,
 * vertical-only field and gimbal lock must skip the heading fusion
 * without corrupting the filter. Plus API guard rails.
 * ---------------------------------------------------------------------------
 */

static void scenario_mag_edge_cases(void)
{
    printf("\n-- scenario: magnetometer edge cases + API guards --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_AHRS;
    /* Static platform, but this measures how far ONE magnetometer
       outlier moves yaw -- a stillness fallback pinning the gyro bias
       would mask exactly that. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_rad[2]        = yaw_true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(3.0f);

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "ahrs_init succeeds");

    int i;
    for (i = 0; i < 100; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_b, false);
    }
    float roll, pitch, yaw0, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw0), "rpy valid");

    /* Zero field: no usable direction -> heading fusion skipped. */
    const float mag_zero[3] = {0.0f, 0.0f, 0.0f};
    for (i = 0; i < 50; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_zero, false);
    }
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "healthy (zero field)");
    CHECK_TRUE(fabsf(ins_angle_diff(yaw, yaw0)) < DEG2RAD(0.5f), "zero field leaves yaw alone");

    /* Vertical-only field: de-tilted horizontal component unusable. */
    const float mag_vert[3] = {0.0f, 0.0f, 50.0f};
    for (i = 0; i < 50; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_vert, false);
    }
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "healthy (vertical field)");
    CHECK_TRUE(fabsf(ins_angle_diff(yaw, yaw0)) < DEG2RAD(0.5f), "vertical field leaves yaw alone");
    CHECK_TRUE(a.n_fuse_fail == 0, "no fusion failures");

    /* Gimbal lock: pitched to ~89.5 deg the heading fusion must skip
       (yaw ill-defined) and the filter must stay finite/healthy. */
    ahrs_t        a2;
    ahrs_config_t cfg2   = cfg;
    cfg2.rpy_init_rad[0] = 0.0f;
    cfg2.rpy_init_rad[1] = DEG2RAD(89.5f);
    cfg2.rpy_init_rad[2] = 0.0f;
    CHECK_TRUE(ahrs_init(&a2, &cfg2, t) == 0, "ahrs_init (gimbal) succeeds");
    float f_b2[3], mag_b2[3];
    body_meas_from_rpy(0.0f, DEG2RAD(89.5f), 0.0f, f_b2, mag_n, mag_b2);
    ahrs_time_us_t t2 = t;
    for (i = 0; i < 100; ++i)
    {
        t2 += US_PER_SEC / 100;
        ahrs_update(&a2, t2, gyr0, f_b2, mag_b2, false);
    }
    CHECK_TRUE(a2.is_initialized, "healthy at gimbal lock");
    float q2[4];
    CHECK_TRUE(ahrs_get_quaternion(&a2, q2), "quaternion valid");
    CHECK_TRUE(isfinite(q2[0]) && isfinite(q2[1]) && isfinite(q2[2]) && isfinite(q2[3]),
               "quaternion finite at gimbal lock");

    /* Same at the OTHER pole (pitch ~ -89.5 deg): the sp < -0.99f arm of
       the gimbal-lock guard, never reached by the +89.5 deg case above. */
    ahrs_t        a3;
    ahrs_config_t cfg3   = cfg;
    cfg3.rpy_init_rad[0] = 0.0f;
    cfg3.rpy_init_rad[1] = DEG2RAD(-89.5f);
    cfg3.rpy_init_rad[2] = 0.0f;
    CHECK_TRUE(ahrs_init(&a3, &cfg3, t) == 0, "ahrs_init (neg gimbal) succeeds");
    float f_b3[3], mag_b3[3];
    body_meas_from_rpy(0.0f, DEG2RAD(-89.5f), 0.0f, f_b3, mag_n, mag_b3);
    ahrs_time_us_t t3 = t;
    for (i = 0; i < 100; ++i)
    {
        t3 += US_PER_SEC / 100;
        ahrs_update(&a3, t3, gyr0, f_b3, mag_b3, false);
    }
    CHECK_TRUE(a3.is_initialized, "healthy at negative gimbal lock");
    float q3[4];
    CHECK_TRUE(ahrs_get_quaternion(&a3, q3), "quaternion valid (neg gimbal)");
    CHECK_TRUE(isfinite(q3[0]) && isfinite(q3[1]) && isfinite(q3[2]) && isfinite(q3[3]),
               "quaternion finite at negative gimbal lock");

    /* Field-strength gate armed (mag_field_check_enable) but no position
       ever supplied: mag_field_expected_uT stays at its zeroed "no gate"
       value, so the enable flag alone must not arm the gate. */
    ahrs_t        a4;
    ahrs_config_t cfg4          = cfg;
    cfg4.mag_field_check_enable = true;
    CHECK_TRUE(ahrs_init(&a4, &cfg4, t) == 0, "ahrs_init (gate armed, no position) succeeds");
    ahrs_time_us_t t4 = t;
    for (i = 0; i < 100; ++i)
    {
        t4 += US_PER_SEC / 100;
        ahrs_update(&a4, t4, gyr0, f_b, mag_b, false);
    }
    CHECK_TRUE(a4.is_initialized, "healthy with field gate armed but unset position");

    /* API guards: invalid configs are rejected, uninitialized instances
       refuse to publish, non-finite positions are ignored. */
    CHECK_TRUE(ahrs_init((ahrs_t*)0, &cfg, t) == -1, "init rejects NULL");
    CHECK_TRUE(ahrs_init(&a2, (const ahrs_config_t*)0, t) == -1, "init rejects NULL cfg");
    ahrs_config_t bad = cfg;
    bad.mode          = (ahrs_mode_t)42;
    CHECK_TRUE(ahrs_init(&a2, &bad, t) == -1, "init rejects bad mode");
    bad                        = cfg;
    bad.rpy_init_stddev_rad[0] = 0.0f;
    CHECK_TRUE(ahrs_init(&a2, &bad, t) == -1, "init rejects zero stddev");
    bad                        = cfg;
    bad.rpy_init_stddev_rad[1] = (float)INFINITY; /* positive but not finite */
    CHECK_TRUE(ahrs_init(&a2, &bad, t) == -1, "init rejects non-finite stddev");
    bad                      = cfg;
    bad.gyr_bias_init_rps[1] = nanf(""); /* rpy_init_rad finite, this vector not */
    CHECK_TRUE(ahrs_init(&a2, &bad, t) == -1, "init rejects non-finite gyro bias init");

    ahrs_t dead;
    memset(&dead, 0, sizeof(dead));
    float bias[3];
    CHECK_TRUE(!ahrs_get_rpy(&dead, &roll, &pitch, &yaw), "uninitialized: no rpy");
    CHECK_TRUE(!ahrs_get_quaternion(&dead, q2), "uninitialized: no quat");
    CHECK_TRUE(!ahrs_get_bias_gyr(&dead, bias), "uninitialized: no bias");
    ahrs_update(&dead, t, gyr0, f_b, (const float*)0, false); /* must not crash */
    ahrs_set_position(&dead, 0.85f, 0.16f, 2026.5f);          /* must not crash */

    const float decl_before = a.declination_rad;
    ahrs_set_position(&a, NAN, 0.16f, 2026.5f); /* non-finite lat -> ignored */
    CHECK_NEAR(a.declination_rad, decl_before, 1e-12, "non-finite position ignored");
    ahrs_set_position(&a, 0.85f, NAN, 2026.5f); /* non-finite lon -> ignored */
    CHECK_NEAR(a.declination_rad, decl_before, 1e-12, "non-finite lon ignored");
    ahrs_set_position(&a, 0.85f, 0.16f, NAN); /* non-finite year -> ignored */
    CHECK_NEAR(a.declination_rad, decl_before, 1e-12, "non-finite year ignored");

    /* NULL-instance guards on every remaining public accessor/setter --
       must not crash, and (for the bool getters) must report failure. */
    CHECK_TRUE(!ahrs_get_rpy((const ahrs_t*)0, &roll, &pitch, &yaw), "get_rpy(NULL) false");
    CHECK_TRUE(!ahrs_get_quaternion((const ahrs_t*)0, q2), "get_quaternion(NULL) false");
    CHECK_TRUE(!ahrs_get_bias_gyr((const ahrs_t*)0, bias), "get_bias_gyr(NULL) false");
    float roll_sd, pitch_sd, yaw_sd, bias_sd[3];
    CHECK_TRUE(!ahrs_get_rpy_stddev((const ahrs_t*)0, &roll_sd, &pitch_sd, &yaw_sd),
               "get_rpy_stddev(NULL) false");
    CHECK_TRUE(!ahrs_get_bias_gyr_stddev((const ahrs_t*)0, bias_sd),
               "get_bias_gyr_stddev(NULL) false");
    CHECK_TRUE(!ahrs_auto_zaru_active((const ahrs_t*)0), "auto_zaru_active(NULL) false");
    ahrs_set_auto_zaru_disable((ahrs_t*)0, true);         /* must not crash */
    ahrs_set_position((ahrs_t*)0, 0.85f, 0.16f, 2026.5f); /* must not crash */

    /* Same, but with a valid (non-NULL) instance that is simply not
       initialized yet -- the `is_initialized` half of the AND, not the
       NULL-pointer half. */
    CHECK_TRUE(!ahrs_get_rpy_stddev(&dead, &roll_sd, &pitch_sd, &yaw_sd),
               "uninitialized: no rpy stddev");
    CHECK_TRUE(!ahrs_get_bias_gyr_stddev(&dead, bias_sd), "uninitialized: no bias stddev");
    CHECK_TRUE(!ahrs_auto_zaru_active(&dead), "uninitialized: auto_zaru not active");
}

/* ---------------------------------------------------------------------------
 * Scenario 9b: zero-rotation update (REQ-AHRS-016). In ARS mode the z
 * gyro bias is unobservable via leveling alone. A zero-rotation trigger
 * is the only way to correct it.
 * ---------------------------------------------------------------------------
 */

static void scenario_zaru(void)
{
    printf("\n-- scenario: zero-rotation update --\n");

    /* Static, level platform with a real (uncorrected) z gyro bias: the
       gyro reads the bias even though the true rotation rate is 0. */
    const float bias_z_true = DEG2RAD(2.0f);
    float       f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);
    const float gyr_meas[3] = {0.0f, 0.0f, bias_z_true};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_ARS;
    /* This scenario is about what happens WITHOUT a stillness trigger,
       so opt out of the fallback that would now supply one. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);

    ahrs_t         a_notrig, a_trig;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a_notrig, &cfg, t) == 0, "init (no trigger)");
    CHECK_TRUE(ahrs_init(&a_trig, &cfg, t) == 0, "init (triggered)");

    int i;
    for (i = 0; i < 1000; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a_notrig, t, gyr_meas, f_b, (const float*)0, false);
        ahrs_update(&a_trig, t, gyr_meas, f_b, (const float*)0, true);
    }

    float gyr_bias[3];
    CHECK_TRUE(ahrs_get_bias_gyr(&a_notrig, gyr_bias), "no-trigger bias readable");
    CHECK_NEAR(RAD2DEG(gyr_bias[2]), 0.0, 0.01,
               "z bias stays unobserved without a trigger [deg/s]");

    CHECK_TRUE(ahrs_get_bias_gyr(&a_trig, gyr_bias), "triggered bias readable");
    CHECK_NEAR(RAD2DEG(gyr_bias[2]), RAD2DEG(bias_z_true), 0.1,
               "z bias converges via ZARU [deg/s]");

    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a_trig, &roll, &pitch, &yaw), "triggered rpy valid");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 1.0, "roll sane after convergence [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 1.0, "pitch sane after convergence [deg]");
    CHECK_TRUE(a_trig.n_fuse_fail == 0, "no fusion failures");

    /* Dropping the trigger clears the accumulator instead of fusing a
       partial/stale run on the next re-trigger. */
    ahrs_update(&a_trig, t + US_PER_SEC / 100, gyr_meas, f_b, (const float*)0, false);
    CHECK_TRUE(a_trig.zaru_gyr_count == 0, "accumulator cleared when trigger drops");
}

/* ---------------------------------------------------------------------------
 * Scenario 9c: velocity-blind auto-ZARU fallback (REQ-AHRS-017). With
 * no external trigger at all (e.g. no ins, "aiding: none"), the
 * gyro/accelerometer-only fallback must still correct the otherwise
 * unobservable z gyro bias, arm/disarm correctly, and stay off unless
 * explicitly enabled.
 * ---------------------------------------------------------------------------
 */

static void scenario_auto_zaru_fallback(void)
{
    printf("\n-- scenario: velocity-blind auto-ZARU fallback --\n");

    const float bias_z_true = DEG2RAD(2.0f);
    float       f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);
    const float gyr_static[3] = {0.0f, 0.0f, bias_z_true};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.auto_zaru_disable      = false;

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init (auto-ZARU enabled)");
    CHECK_TRUE(!ahrs_auto_zaru_active(&a), "not yet armed right after init");

    int i;
    for (i = 0; i < 200; ++i) /* 2 s: past the 0.5 s dwell */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false); /* no external trigger */
    }
    CHECK_TRUE(ahrs_auto_zaru_active(&a), "armed after dwelling static, no external trigger");

    for (; i < 1000; ++i) /* keep still long enough for the bias to converge */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    }
    float gyr_bias[3];
    CHECK_TRUE(ahrs_get_bias_gyr(&a, gyr_bias), "bias readable");
    CHECK_NEAR(RAD2DEG(gyr_bias[2]), RAD2DEG(bias_z_true), 0.1,
               "z bias converges via the velocity-blind fallback alone [deg/s]");

    /* Real rotation resumes: the fallback must disarm (it would
       otherwise keep "correcting" a genuine turn as if it were bias). */
    const float gyr_turn[3] = {0.0f, 0.0f, DEG2RAD(30.0f)};
    for (i = 0; i < 50; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_turn, f_b, (const float*)0, false);
    }
    CHECK_TRUE(!ahrs_auto_zaru_active(&a), "disarmed once the platform moves again");

    /* Opted out (auto_zaru_disable): the fallback must NOT arm, however
       still the platform looks. The zeroed default is the opposite --
       armed -- which every other case in this scenario relies on. */
    ahrs_t        a_off;
    ahrs_config_t cfg_off     = cfg;
    cfg_off.auto_zaru_disable = true;
    CHECK_TRUE(ahrs_init(&a_off, &cfg_off, t) == 0, "init (fallback opted out)");
    for (i = 0; i < 200; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a_off, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(!ahrs_auto_zaru_active(&a_off), "auto_zaru_disable keeps the fallback off");

    /* And the zeroed config really does arm it (the point of the
       negative name): same static stream, nothing configured. */
    ahrs_t        a_default;
    ahrs_config_t cfg_default = cfg;
    CHECK_TRUE(ahrs_init(&a_default, &cfg_default, t) == 0, "init (zeroed = armed)");
    for (i = 0; i < 200; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a_default, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(ahrs_auto_zaru_active(&a_default), "fallback is armed by default");

    /* Both halves of the variance verdict are independent: a platform that
       passes the gyro stddev gate but fails the accelerometer one (jittery
       support, e.g. engine idle) must never actually trigger the fallback
       fusion, even though gyro alone looks perfectly still.
       ahrs_auto_zaru_active() is not the right probe here: it latches on
       the coarse magnitude gate alone (auto_zaru_static_since != 0) and
       stays latched regardless of the variance verdict -- ahrs_zaru_applied()
       tracks the trigger actually used for THIS update, which is what
       the variance AND is gating. */
    ahrs_t        a_accjit;
    ahrs_config_t cfg_accjit  = cfg;
    const float   gyr_zero[3] = {0.0f, 0.0f, 0.0f};
    CHECK_TRUE(ahrs_init(&a_accjit, &cfg_accjit, t) == 0, "init (accel jitter)");
    for (i = 0; i < 300; ++i) /* 3 s: past the 0.5 s dwell */
    {
        t += US_PER_SEC / 100;
        /* +/-0.6 m/s^2 square-wave jitter, on the axis PERPENDICULAR to
           gravity: the RMS combines all 3 axes (dividing any single-axis
           stddev by sqrt(3)), so this clears the 0.3 m/s^2 variance gate
           (auto_zaru_static_acc_stddev_mps2), while |acc_norm - g| only
           grows by the second-order amount amp^2/(2g) =~ 0.018 m/s^2,
           comfortably inside the 0.5 m/s^2 magnitude gate
           (auto_zaru_static_acc_mps2) that a same-size jitter ALONG
           gravity would have blown. */
        float f_b_jit[3] = {f_b[0] + ((i % 2 == 0) ? 0.6f : -0.6f), f_b[1], f_b[2]};
        ahrs_update(&a_accjit, t, gyr_zero, f_b_jit, (const float*)0, false);
    }
    CHECK_TRUE(!ahrs_zaru_applied(&a_accjit),
               "accel jitter alone blocks the fallback trigger despite a still gyro");
}

/* ---------------------------------------------------------------------------
 * Scenario: velocity-blind auto-ZARU fallback averages out vibration
 * (REQ-AHRS-016, REQ-AHRS-017). Mirrors test_ins_core.c's
 * scenario_auto_zaru_vibration: zaru_gyr_sum/count is filled by
 * ahrs_auto_zaru_detect from the moment the variance window confirms
 * stillness, not only once the dwell-gated trigger goes true, so the FIRST
 * fusion already covers a real average instead of a single vibrating
 * sample. A short auto_zaru_dwell_sec with zero_rot_stddev_rps tight enough
 * to lock the bias state onto whatever that first fusion measures is the
 * adversarial case that would expose a single-sample fusion immediately.
 * ---------------------------------------------------------------------------
 */
static void scenario_auto_zaru_vibration(void)
{
    printf("\n-- scenario: auto-ZARU fallback averages out vibration --\n");

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.auto_zaru_disable      = false;
    /* Allow the true bias within the initial uncertainty and make the
       zero-rotation update aggressive: the worst case for a single-sample
       ZARU. */
    cfg.gyr_bias_init_stddev_rps[0] = DEG2RAD(1.0f);
    cfg.gyr_bias_init_stddev_rps[1] = DEG2RAD(1.0f);
    cfg.gyr_bias_init_stddev_rps[2] = DEG2RAD(1.0f);
    cfg.zero_rot_stddev_rps         = DEG2RAD(0.01f);
    /* Both stillness criteria must tolerate the vibration amplitude, same
       reasoning and margin as the ins.c scenario. */
    cfg.auto_zaru_static_gyr_rps        = 0.1f;
    cfg.auto_zaru_static_gyr_stddev_rps = DEG2RAD(2.5f);

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init ok");

    float f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);
    const float gyr_bias_true[3] = {0.005f, -0.005f, 0.01f}; /* rad/s */
    const float vib_rps          = 0.03f;                    /* zero-mean, alternating each epoch */

    int i;
    for (i = 1; i <= 1000; ++i) /* 10 s stationary, "engine on" */
    {
        t += US_PER_SEC / 100;
        const float vib         = (i % 2 == 0) ? vib_rps : -vib_rps;
        const float gyr_body[3] = {gyr_bias_true[0] + vib, gyr_bias_true[1] + vib,
                                   gyr_bias_true[2] + vib};
        ahrs_update(&a, t, gyr_body, f_b, (const float*)0, false); /* no external trigger */
    }

    CHECK_TRUE(ahrs_auto_zaru_active(&a), "fallback armed despite vibration");

    float bias_est[3];
    CHECK_TRUE(ahrs_get_bias_gyr(&a, bias_est), "bias readable");
    /* A single-sample ZARU would leave an error of up to vib_rps (0.03);
       the averaged measurement must land at the true bias. */
    CHECK_NEAR(bias_est[0], gyr_bias_true[0], 0.002, "gyro bias x [rad/s]");
    CHECK_NEAR(bias_est[1], gyr_bias_true[1], 0.002, "gyro bias y [rad/s]");
    CHECK_NEAR(bias_est[2], gyr_bias_true[2], 0.002, "gyro bias z [rad/s]");
}

/* ---------------------------------------------------------------------------
 * Scenario: ahrs_set_auto_zaru_disable() -- runtime on/off for the
 * velocity-blind auto-ZARU fallback (REQ-AHRS-017), independent of
 * cfg.auto_zaru_disable set at ahrs_init. Mirrors ins's
 * ins_set_auto_zupt_disable test in tests/test_ins_core.c.
 * ---------------------------------------------------------------------------
 */

static void scenario_auto_zaru_disable_runtime(void)
{
    printf("\n-- scenario: ahrs_set_auto_zaru_disable() runtime toggle --\n");

    float f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);
    const float gyr_static[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.auto_zaru_disable      = false; /* armed by default */

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init (fallback enabled)");

    /* ahrs_auto_zaru_active() reports "a stillness run has started" (true
       from the first static epoch, before the dwell elapses, see
       scenario_zaru_applied_accessor). ahrs_zaru_applied() reports the
       dwell-gated decision the fusion actually acted on -- that is what
       this scenario needs to distinguish "disabled" from "re-enabled but
       not yet dwelled", so it is used throughout below instead. */
    int i;
    for (i = 0; i < 200; ++i) /* 2 s: past the dwell */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(ahrs_zaru_applied(&a), "armed while stationary before any disable call");

    ahrs_set_auto_zaru_disable(&a, true);
    t += US_PER_SEC / 100;
    ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    CHECK_TRUE(!ahrs_zaru_applied(&a), "disabled immediately, still stationary");
    CHECK_TRUE(!ahrs_auto_zaru_active(&a), "no stillness run either, while disabled");

    for (i = 0; i < 200; ++i) /* 2 more s, still stationary, still disabled */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(!ahrs_zaru_applied(&a), "stays disabled however long stillness continues");

    ahrs_set_auto_zaru_disable(&a, false);
    t += US_PER_SEC / 100;
    ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    CHECK_TRUE(!ahrs_zaru_applied(&a),
               "re-enabled but not yet dwelled: a fresh dwell is required, not a stale timer");

    for (i = 0; i < 200; ++i) /* 2 s: a fresh dwell has now elapsed */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(ahrs_zaru_applied(&a), "armed again after a fresh dwell post re-enable");
}

/* ---------------------------------------------------------------------------
 * Scenario: ahrs_zaru_applied() -- the "was a zero-rotation trigger present
 * this epoch" accessor (REQ-AHRS-024). Unlike ahrs_auto_zaru_active() it
 * covers the caller's explicit flag too, and it reports the dwell-satisfied
 * decision the fusion acted on, not the mere start of a stillness run. This
 * is what nav_suite keys the vertical zero-velocity update off
 * (REQ-SUITE-015), so it must be exact on both fronts.
 * ---------------------------------------------------------------------------
 */

static void scenario_zaru_applied_accessor(void)
{
    printf("\n-- scenario: ahrs_zaru_applied() accessor (REQ-AHRS-024) --\n");

    float f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);
    const float gyr_static[3] = {0.0f, 0.0f, 0.0f};
    const float gyr_turn[3]   = {0.0f, 0.0f, DEG2RAD(30.0f)};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_ARS;
    /* This scenario is about what happens WITHOUT a stillness trigger,
       so opt out of the fallback that would now supply one. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);

    /* Fallback OFF: only the caller's explicit flag can raise it. */
    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init (fallback off)");
    CHECK_TRUE(!ahrs_zaru_applied(&a), "false right after init");

    int i;
    for (i = 0; i < 200; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(!ahrs_zaru_applied(&a), "static but no trigger and no fallback -> false");

    t += US_PER_SEC / 100;
    ahrs_update(&a, t, gyr_static, f_b, (const float*)0, true);
    CHECK_TRUE(ahrs_zaru_applied(&a), "explicit caller trigger -> true");

    /* A dropped epoch (non-finite gyro) must not report the previous
       epoch's trigger. */
    const float gyr_nan[3] = {0.0f, 0.0f, NAN};
    t += US_PER_SEC / 100;
    ahrs_update(&a, t, gyr_nan, f_b, (const float*)0, true);
    CHECK_TRUE(!ahrs_zaru_applied(&a), "dropped epoch (NaN gyro) -> false, not stale");

    t += US_PER_SEC / 100;
    ahrs_update(&a, t, gyr_static, f_b, (const float*)0, false);
    CHECK_TRUE(!ahrs_zaru_applied(&a), "trigger cleared again on the next epoch");

    /* Fallback ON: the accessor must follow the dwell-satisfied decision,
       not the start of the stillness run (which auto_zaru_active reports). */
    ahrs_t        b;
    ahrs_config_t cfg_auto     = cfg;
    cfg_auto.auto_zaru_disable = false;
    CHECK_TRUE(ahrs_init(&b, &cfg_auto, t) == 0, "init (fallback on)");

    t += US_PER_SEC / 100;
    ahrs_update(&b, t, gyr_static, f_b, (const float*)0, false);
    CHECK_TRUE(ahrs_auto_zaru_active(&b), "stillness run started (dwell not yet done)");
    CHECK_TRUE(!ahrs_zaru_applied(&b), "no trigger yet during the dwell window");

    for (i = 0; i < 200; ++i) /* 2 s: past the 0.5 s dwell */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&b, t, gyr_static, f_b, (const float*)0, false);
    }
    CHECK_TRUE(ahrs_zaru_applied(&b), "fallback alone raises the trigger after the dwell");

    t += US_PER_SEC / 100;
    ahrs_update(&b, t, gyr_turn, f_b, (const float*)0, false);
    CHECK_TRUE(!ahrs_zaru_applied(&b), "real rotation -> false again");

    CHECK_TRUE(!ahrs_zaru_applied((const ahrs_t*)0), "NULL instance -> false");
}

/* ---------------------------------------------------------------------------
 * Scenario 10: self-healing references (REQ-SUITE-003). Corrupted
 * AHRS/baro instances trip their health checks and are re-bootstrapped
 * autonomously from the measurement stream.
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_selfhealing(void)
{
    printf("\n-- scenario: nav_suite self-healing references --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    opt.allow_unlimited_deadreckoning      = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    int                i;
#define SH_EPOCH()                                                             \
    do {                                                                       \
        t += US_PER_SEC / 100;                                                 \
        memset(&m, 0, sizeof(m));                                              \
        m.timestamp        = t;                                                \
        m.strapdown_dt_sec = 0.01f;                                            \
        m.acc.is_valid     = true;                                             \
        m.gyr.is_valid     = true;                                             \
        memcpy(m.acc.data, f_b, sizeof(f_b));                                  \
        m.mag.is_valid = true;                                                 \
        memcpy(m.mag.data, mag_b, sizeof(mag_b));                              \
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;      \
        m.baro.is_valid                                           = true;      \
        m.baro.pressure_pa                                        = 101325.0f; \
        nav_suite_update(&s, &m);                                              \
    } while (0)

    for (i = 0; i < 300; ++i) SH_EPOCH();
    CHECK_TRUE(s.ars.is_initialized, "ars running");
    CHECK_TRUE(s.ahrs.is_initialized, "mag-ahrs running");
    CHECK_TRUE(s.baro_alt.is_initialized, "baro filter running");

    /* Corrupt all three references (simulated memory/numerics fault).
       The next epoch trips the health checks ... */
    s.ars.q[0]      = NAN;
    s.ahrs.q[1]     = NAN;
    s.baro_alt.x[0] = NAN;
    SH_EPOCH();
    /* ars/ahrs do NOT trip here: kalman_udu's own singularity guard
       (KFCore) rejects the NaN-tainted Jacobian before it can corrupt
       d/U, and ins_quat_normalize() (pre-existing fallback) silently
       resets a non-finite quaternion to identity. The covariance is
       therefore never widened, so isfinite()-based ahrs_check_health()
       never sees a violation -- the filter keeps running, confidently,
       on a now-wrong attitude. baro has no such quaternion-fallback
       path, so its NaN state is caught as before. */
    CHECK_TRUE(s.ars.is_initialized, "ars silently healed (quaternion reset), not tripped");
    CHECK_TRUE(s.ahrs.is_initialized, "mag-ahrs silently healed (quaternion reset), not tripped");
    CHECK_TRUE(!s.baro_alt.is_initialized, "baro health check tripped");

    /* ... and the wrapper re-bootstraps the baro reference from the stream. */
    for (i = 0; i < 300; ++i) SH_EPOCH();
    CHECK_TRUE(s.ars.is_initialized, "ars still running");
    CHECK_TRUE(s.ahrs.is_initialized, "mag-ahrs still running");
    CHECK_TRUE(s.baro_alt.is_initialized, "baro filter re-bootstrapped");

    float roll, pitch, yaw, h;
    CHECK_TRUE(nav_suite_get_rpy_ahrs(&s, &roll, &pitch, &yaw), "mag-ahrs rpy valid again");
    CHECK_NEAR(RAD2DEG(roll), 0.0, 1.0, "recovered roll [deg]");
    CHECK_NEAR(RAD2DEG(pitch), 0.0, 1.0, "recovered pitch [deg]");
    /* Yaw stays stuck near the identity reset (0 deg), not yaw_true: the
       covariance was never widened by a health-check re-arm, so the
       Kalman gain for the residual 30 deg error is too small to close it
       within this many epochs. */
    CHECK_NEAR(RAD2DEG(yaw), 0.0, 2.0, "yaw stuck near identity reset, not recovered");
    CHECK_TRUE(nav_suite_get_baro_alt(&s, &h, (float*)0), "baro height valid again");
    CHECK_NEAR(h, 0.0, 0.5, "recovered baro height [m]");
#undef SH_EPOCH
}

/* ---------------------------------------------------------------------------
 * Scenario 11: accessor fallbacks and guard rails of the wrapper
 * (REQ-SUITE-005/-008 edges: NONE mode, ARS-only attitude fallback,
 * height from a coasting ins, NULL arguments).
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_fallbacks(void)
{
    printf("\n-- scenario: nav_suite fallbacks + guards --\n");

    float roll, pitch, yaw, h;

    /* NULL guards. */
    CHECK_TRUE(nav_suite_init((nav_suite_t*)0, (const ins_init_t*)0, (const ins_options_t*)0) == -1,
               "init rejects NULL");
    CHECK_TRUE(nav_suite_get_mode((const nav_suite_t*)0) == NAV_SUITE_MODE_NONE,
               "mode(NULL) is NONE");
    CHECK_TRUE(!nav_suite_get_rpy((const nav_suite_t*)0, &roll, &pitch, &yaw), "rpy(NULL) false");
    CHECK_TRUE(!nav_suite_get_height((const nav_suite_t*)0, &h), "height(NULL) false");
    CHECK_TRUE(!nav_suite_get_height_ellipsoid((const nav_suite_t*)0, &h),
               "height_ell(NULL) false");
    CHECK_TRUE(!nav_suite_get_baro_alt((const nav_suite_t*)0, &h, (float*)0),
               "baro_alt(NULL) false");

    /* Manual init, no barometer, no magnetometer, no GNSS: ins warms
       up, then coasts; the wrapper must fall back gracefully. */
    const float rpy_true[3] = {DEG2RAD(5.0f), DEG2RAD(-3.0f), 0.0f};
    float       f_b[3];
    body_meas_from_rpy(rpy_true[0], rpy_true[1], rpy_true[2], f_b, (const float*)0, (float*)0);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.rpy_init_rad[0]           = rpy_true[0];
    init.rpy_init_rad[1]           = rpy_true[1];
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
    /* default 10 s coasting window (no unlimited deadreckoning) */

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    /* Fresh suite: ins not ready yet (warm-up), no AHRS bootstrapped,
       no baro -> NONE and no height. */
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_NONE, "mode NONE before any update");
    CHECK_TRUE(!nav_suite_get_height(&s, &h), "no height source yet");
    CHECK_TRUE(!nav_suite_get_height_ellipsoid(&s, &h), "no ellipsoid height source yet");

    ins_measurements_t m;
    int                i;
    /* 4 s: one startup GNSS fix (REQ-NAV-033) so the limited-DR ins
       starts, then IMU-only -> ins becomes ready but is coasting (its
       position aiding clock started at that first fix). No baro -> the
       height accessors must fall back to the coasting ins. */
    for (i = 0; i < 400; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        if (i == 0)
        {
            m.gnss_pos.is_valid = true;
            ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0,
                                m.gnss_pos.xyz_ecef);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_COASTING,
               "mode COASTING (ready, no aiding)");
    CHECK_TRUE(nav_suite_get_height(&s, &h), "height from coasting ins");
    CHECK_TRUE(nav_suite_get_height_ellipsoid(&s, &h), "ellipsoid height from coasting ins");
    CHECK_NEAR(h, 300.0, 5.0, "ellipsoid height plausible [m]");

    /* Past the 10 s window: ins position gone; no magnetometer was
       ever seen, so the attitude fallback lands on the ARS. */
    for (; i < 1200; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(nav_suite_get_mode(&s) == NAV_SUITE_MODE_ATTITUDE_ONLY,
               "mode ATTITUDE_ONLY after the window");
    CHECK_TRUE(nav_suite_get_rpy(&s, &roll, &pitch, &yaw), "attitude from the ARS fallback");
    CHECK_NEAR(RAD2DEG(roll), RAD2DEG(rpy_true[0]), 1.0, "ARS fallback roll [deg]");
    CHECK_TRUE(!nav_suite_get_height(&s, &h), "no height once ins is gone (no baro)");

    /* A short IMU+baro run bootstraps the vertical filter (attitude from
       the ARS), so the delayed offset pair below can extrapolate with its
       climb rate. The datum anchors on the MEAN pressure over a short
       window (REQ-SUITE-006), so this takes a few epochs, not a single
       sample. */
    for (i = 0; i < 40; ++i) /* 0.4 s > the bootstrap window */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = 101325.0f;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(s.baro_alt.is_initialized, "baro filter bootstrapped");

    /* Baro/GNSS offset-filter edges: an epoch without IMU is fine for
       the offset filter; a pair without a usable vertical variance is
       skipped; a delayed pair extrapolates with the climb rate. */
    t += US_PER_SEC / 100;
    memset(&m, 0, sizeof(m));
    m.timestamp         = t;
    m.baro.is_valid     = true;
    m.baro.pressure_pa  = 101325.0f;
    m.gnss_pos.is_valid = true;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, m.gnss_pos.xyz_ecef);
    /* vertical variance left at 0 -> pair skipped */
    nav_suite_update(&s, &m);
    CHECK_TRUE(!s.local_gnss.is_initialized, "offset pair without vertical variance skipped");

    t += US_PER_SEC / 100;
    m.timestamp           = t;
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_delay_ms                               = 200; /* exercise the climb-rate extrapolation */
    nav_suite_update(&s, &m);
    CHECK_TRUE(s.local_gnss.is_initialized, "offset filter initialized");
}

/* ---------------------------------------------------------------------------
 * Scenario 11d: ins/baro_alt height cross-check (REQ-SUITE-019).
 *
 * The height accessors hand over between exactly these two estimates at a
 * mode change, so a divergence between them is the size of the step the
 * reported height takes at that moment. The suite must notice. The
 * vertical-VELOCITY cross-check cannot substitute: the divergence forced
 * here stays far below its bound while growing without limit in the height.
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_height_cross_check(void)
{
    printf("\n-- scenario: nav_suite ins/baro_alt height cross-check --\n");

    const float rpy_true[3] = {0.0f, 0.0f, 0.0f};
    float       f_b[3];
    body_meas_from_rpy(rpy_true[0], rpy_true[1], rpy_true[2], f_b, (const float*)0, (float*)0);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    /* No barometric height source in ins: this scenario is about the two
       filters DISAGREEING, so ins must be driven by GNSS height while
       baro_alt follows the pressure. */
    opt.baro_height_disable = true;
    /* Same reason, for the stillness detectors: the stimulus below is
       contradictory on purpose (a perfectly still IMU while the pressure
       falls as if climbing at 0.2 m/s), and the detectors correctly
       resolve that contradiction in favour of the IMU - baro_alt's
       zero-velocity update (REQ-SUITE-015, tuned from this init's
       zero_vel_stddev_mps since REQ-SUITE-020) pins the climb the
       cross-check under test is supposed to see. Turning them off keeps
       this scenario about the cross-check alone. */
    opt.auto_zupt_disable = true;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    ins_measurements_t m;
    int                i;

    /* 5 s of agreement: GNSS at the init height, pressure at the ISA level
       that matches it. Both filters settle on the same datum. */
    for (i = 0; i < 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = 101325.0f;
        if ((i % 100) == 0)
        {
            m.gnss_pos.is_valid = true;
            ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0,
                                m.gnss_pos.xyz_ecef);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
            m.gnss_vel.is_valid                           = true;
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(s.ins.is_initialized && s.baro_alt.is_initialized, "both filters running");
    CHECK_TRUE(s.log_state.t_last_height_warn == 0, "no cross-check warning while they agree");

    /* Now drive them apart slowly: the pressure falls as if climbing at
       0.2 m/s while GNSS keeps reporting the same height. After 200 s
       baro_alt is ~40 m above ins. A 0.2 m/s disagreement is nowhere near
       the vertical-velocity bound, which is exactly the point. */
    for (i = 0; i < 20000; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_at_altitude(0.2f * (float)(i + 1) * 0.01f);
        if ((i % 100) == 0)
        {
            m.gnss_pos.is_valid = true;
            ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0,
                                m.gnss_pos.xyz_ecef);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
            m.gnss_vel.is_valid                           = true;
            m.gnss_vel.vel_ned[0] = m.gnss_vel.vel_ned[1] = m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        }
        nav_suite_update(&s, &m);
    }

    float h_baro, pos_ned[3];
    CHECK_TRUE(baro_alt_get_height(&s.baro_alt, &h_baro), "baro height available");
    CHECK_TRUE(ins_get_position_local(&s.ins, pos_ned), "ins height available");
    CHECK_TRUE(fabsf(h_baro - (-pos_ned[2])) > 20.0f, "the two really have diverged");
    CHECK_TRUE(s.log_state.t_last_height_warn != 0, "height cross-check warning raised");
    /* The velocity cross-check stays silent at this rate, which is why the
       height check has to exist separately. */
    CHECK_TRUE(s.log_state.t_last_vvel_warn == 0,
               "vertical-velocity cross-check silent at this divergence rate");

    /* The other half of the pair: a divergence fast enough that the two
       vertical VELOCITIES disagree. The pressure now falls as if climbing
       at 5 m/s while the IMU still reports a perfectly level standstill,
       so baro_alt rides up and ins does not -- well past the 2 m/s bound
       the velocity cross-check watches. */
    const float h_at_switch = 0.2f * (float)20000 * 0.01f;
    for (i = 0; i < 2000; ++i) /* 20 s */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.baro.is_valid    = true;
        m.baro.pressure_pa = pressure_at_altitude(h_at_switch + 5.0f * (float)(i + 1) * 0.01f);
        if ((i % 100) == 0)
        {
            m.gnss_pos.is_valid = true;
            ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0,
                                m.gnss_pos.xyz_ecef);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
            m.gnss_vel.is_valid                           = true;
            m.gnss_vel.vel_ned[0] = m.gnss_vel.vel_ned[1] = m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        }
        nav_suite_update(&s, &m);
    }
    {
        float v_baro_up, vel_ned[3];
        CHECK_TRUE(baro_alt_get_velocity(&s.baro_alt, &v_baro_up), "baro velocity available");
        CHECK_TRUE(ins_get_velocity_ned(&s.ins, vel_ned), "ins velocity available");
        CHECK_TRUE(fabsf(-vel_ned[2] - v_baro_up) >= 2.0f,
                   "the two vertical velocities really disagree");
    }
    CHECK_TRUE(s.log_state.t_last_vvel_warn != 0, "vertical-velocity cross-check warning raised");
}

/* ---------------------------------------------------------------------------
 * Scenario 11b: zero-rotation update propagation to ARS/AHRS
 * (REQ-SUITE-009). ins's own auto-ZUPT/ZARU detector must also
 * correct the ARS's z gyro bias, which the ARS cannot observe on its
 * own (no velocity state, no magnetometer).
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_zaru(void)
{
    printf("\n-- scenario: nav_suite zero-rotation propagation --\n");

    /* Static platform with a real z gyro bias, small enough to stay
       under ins's auto-ZUPT/ZARU static-gyro threshold (~2.9 deg/s
       initially, tighter still once ins's own bias estimate
       converges). */
    const float bias_z_true = DEG2RAD(2.0f);
    float       f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.zero_rot_stddev_rps       = DEG2RAD(0.1f);

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

    ins_measurements_t m;
    int                i;
    /* One startup fix (REQ-NAV-033) so ins becomes ready, then a
       static IMU-only stream long enough for the dwell time (0.5 s) and
       a few rate-limited (1 s) auto-ZUPT/ZARU fusions to run. */
    for (i = 0; i < 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = bias_z_true;
        if (i == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        nav_suite_update(&s, &m);
    }

    CHECK_TRUE(s.ins.is_initialized, "ins ready");
    CHECK_TRUE(ins_get_diag(&s.ins)->n_auto_zupt > 0, "ins auto-ZUPT/ZARU fired");

    float gyr_bias[3];
    CHECK_TRUE(ahrs_get_bias_gyr(&s.ars, gyr_bias), "ars bias readable");
    CHECK_NEAR(RAD2DEG(gyr_bias[2]), RAD2DEG(bias_z_true), 0.5,
               "ars z bias converges via propagated ZARU trigger [deg/s]");
    CHECK_TRUE(nav_suite_get_zaru_active(&s), "zaru_active true after a static epoch");
    CHECK_TRUE(!nav_suite_get_zaru_active((const nav_suite_t*)0), "zaru_active(NULL) false");

    /* The same standstill with NO startup fix, which is the case the
       accessor used to go dark on (REQ-SUITE-010): ins never initializes
       without an absolute position aid, so it contributes no trigger,
       and the ARS/AHRS's velocity-blind fallback is the only stillness
       detector in the suite. What it decides has to reach the accessor,
       or telemetry reports a standing platform as never having had a
       zero-rotation update while the ARS is fusing one every second. */
    static nav_suite_t s_imu;
    memset(&s_imu, 0, sizeof(s_imu));
    CHECK_TRUE(nav_suite_init(&s_imu, &init, &opt) == 0, "nav_suite_init (IMU only)");
    for (i = 0; i < 500; ++i)
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = bias_z_true;
        nav_suite_update(&s_imu, &m);
    }
    CHECK_TRUE(!s_imu.ins.is_initialized, "ins stays uninitialized without a fix");
    CHECK_TRUE(!s_imu.last_zaru_trigger, "the REQ-SUITE-009 trigger alone is absent");
    CHECK_TRUE(ahrs_zaru_applied(&s_imu.ars), "the ars fallback fired");
    CHECK_TRUE(nav_suite_get_zaru_active(&s_imu),
               "zaru_active reports the velocity-blind fallback too");
}

/* ---------------------------------------------------------------------------
 * Scenario 11b2: one stillness definition for the whole suite
 * (REQ-SUITE-020). ins, the two AHRS instances and baro_alt each decide
 * "is the platform still" with their own code, but they must all be
 * TUNED from ins_options_t.auto_zupt_* -- before that propagation
 * existed, a caller who tightened ins's gates silently kept the ARS/AHRS
 * (and with them baro_alt's vertical ZUPT) on untouched defaults.
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_stillness_propagation(void)
{
    printf("\n-- scenario: stillness definition propagation (REQ-SUITE-020) --\n");

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.time = 1000000;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m   = 1.0f;
    init.vel_init_stddev_mps = 0.1f;
    init.zero_vel_stddev_mps = 0.02f;
    init.zero_rot_stddev_rps = DEG2RAD(0.3f);

    /* Deliberately unlike every built-in default, so a field that is not
       actually propagated cannot pass by coincidence. */
    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec             = 0.01f;
    opt.auto_zupt_static_gyr_rps         = DEG2RAD(3.25f);
    opt.auto_zupt_static_acc_mps2        = 0.42f;
    opt.auto_zupt_static_gyr_stddev_rps  = DEG2RAD(0.75f);
    opt.auto_zupt_static_acc_stddev_mps2 = 0.17f;
    opt.auto_zupt_dwell_sec              = 1.25f;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");

    CHECK_NEAR(s.ars_cfg.auto_zaru_static_gyr_rps, opt.auto_zupt_static_gyr_rps, 1e-9,
               "ars gyro magnitude bound from ins config");
    CHECK_NEAR(s.ars_cfg.auto_zaru_static_acc_mps2, opt.auto_zupt_static_acc_mps2, 1e-9,
               "ars accel magnitude bound from ins config");
    CHECK_NEAR(s.ars_cfg.auto_zaru_static_gyr_stddev_rps, opt.auto_zupt_static_gyr_stddev_rps, 1e-9,
               "ars gyro window stddev from ins config");
    CHECK_NEAR(s.ars_cfg.auto_zaru_static_acc_stddev_mps2, opt.auto_zupt_static_acc_stddev_mps2,
               1e-9, "ars accel window stddev from ins config");
    CHECK_NEAR(s.ars_cfg.auto_zaru_dwell_sec, opt.auto_zupt_dwell_sec, 1e-9,
               "ars dwell from ins config");
    CHECK_NEAR(s.ars_cfg.zero_rot_stddev_rps, init.zero_rot_stddev_rps, 1e-9,
               "ars zero-rotation stddev from ins config");

    CHECK_NEAR(s.ahrs_cfg.auto_zaru_static_gyr_rps, opt.auto_zupt_static_gyr_rps, 1e-9,
               "ahrs gyro magnitude bound from ins config");
    CHECK_NEAR(s.ahrs_cfg.auto_zaru_static_gyr_stddev_rps, opt.auto_zupt_static_gyr_stddev_rps,
               1e-9, "ahrs gyro window stddev from ins config");
    CHECK_NEAR(s.ahrs_cfg.auto_zaru_dwell_sec, opt.auto_zupt_dwell_sec, 1e-9,
               "ahrs dwell from ins config");
    CHECK_NEAR(s.ahrs_cfg.zero_rot_stddev_rps, init.zero_rot_stddev_rps, 1e-9,
               "ahrs zero-rotation stddev from ins config");
    CHECK_TRUE(s.ahrs_cfg.mode == AHRS_MODE_AHRS, "ahrs template keeps its own mode");

    /* baro_alt has no detector of its own: it is fed the same trigger
       (REQ-SUITE-015), so it must also be fed the same trust in it. */
    CHECK_NEAR(s.baro_cfg.zupt_stddev_mps, init.zero_vel_stddev_mps, 1e-9,
               "baro_alt zero-velocity stddev from ins config");

    /* Both opt-outs reach the ARS/AHRS, and the narrow one leaves ins's
       own (velocity-aware) detector armed. */
    CHECK_TRUE(!s.ars_cfg.auto_zaru_disable && !s.ahrs_cfg.auto_zaru_disable,
               "fallback armed by default");

    static nav_suite_t s_blind;
    ins_options_t      opt_blind               = opt;
    opt_blind.auto_zupt_velocity_blind_disable = true;
    memset(&s_blind, 0, sizeof(s_blind));
    CHECK_TRUE(nav_suite_init(&s_blind, &init, &opt_blind) == 0, "nav_suite_init (blind opt-out)");
    CHECK_TRUE(s_blind.ars_cfg.auto_zaru_disable && s_blind.ahrs_cfg.auto_zaru_disable,
               "velocity-blind opt-out disables the ARS/AHRS fallback");
    CHECK_TRUE(!s_blind.ins.opt.auto_zupt_disable, "velocity-blind opt-out leaves ins armed");

    static nav_suite_t s_off;
    ins_options_t      opt_off = opt;
    opt_off.auto_zupt_disable  = true;
    memset(&s_off, 0, sizeof(s_off));
    CHECK_TRUE(nav_suite_init(&s_off, &init, &opt_off) == 0, "nav_suite_init (all off)");
    CHECK_TRUE(s_off.ars_cfg.auto_zaru_disable && s_off.ahrs_cfg.auto_zaru_disable,
               "auto_zupt_disable also disables the ARS/AHRS fallback");
}

/* ---------------------------------------------------------------------------
 * Scenario 11c: nav_suite_set_auto_zupt_zaru_disable() (REQ-SUITE-018).
 * Continues the same static-with-known-bias setup as scenario_suite_zaru:
 * ins's detector and the ARS's own zaru trigger must both stop firing the
 * instant the suite-wide switch is thrown, and both must resume (only)
 * after a fresh dwell once it is thrown back.
 * ---------------------------------------------------------------------------
 */

static void scenario_suite_auto_zupt_zaru_disable(void)
{
    printf("\n-- scenario: nav_suite_set_auto_zupt_zaru_disable() (REQ-SUITE-018) --\n");

    const float bias_z_true = DEG2RAD(2.0f);
    float       f_b[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_b, (const float*)0, (float*)0);

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.zero_rot_stddev_rps       = DEG2RAD(0.1f);

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

    ins_measurements_t m;
    int                i;
    for (i = 0; i < 500; ++i) /* startup fix + static run, as in scenario_suite_zaru */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = bias_z_true;
        if (i == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(ins_get_diag(&s.ins)->n_auto_zupt > 0, "ins auto-ZUPT/ZARU fired before disable");
    CHECK_TRUE(nav_suite_get_zaru_active(&s), "zaru_active true before disable");
    const uint32_t n_before_disable = ins_get_diag(&s.ins)->n_auto_zupt;

    nav_suite_set_auto_zupt_zaru_disable(&s, true);
    for (i = 0; i < 500; ++i) /* another 5 s, still perfectly static */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = bias_z_true;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(ins_get_diag(&s.ins)->n_auto_zupt == n_before_disable,
               "ins detector stopped firing once disabled suite-wide");
    CHECK_TRUE(!nav_suite_get_zaru_active(&s), "zaru_active false while disabled");
    CHECK_TRUE(!ahrs_auto_zaru_active(&s.ars), "ars fallback also stopped while disabled");

    nav_suite_set_auto_zupt_zaru_disable(&s, false);
    for (i = 0; i < 500; ++i) /* re-enabled: another 5 s, needs a fresh dwell */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = bias_z_true;
        nav_suite_update(&s, &m);
    }
    CHECK_TRUE(ins_get_diag(&s.ins)->n_auto_zupt > n_before_disable,
               "ins detector resumed firing after re-enable");
    CHECK_TRUE(nav_suite_get_zaru_active(&s), "zaru_active true again after re-enable");
}

/* ---------------------------------------------------------------------------
 * Scenario 12: corrupted UDU factors (REQ-SYS-005). Prediction and
 * fusion math on broken covariance factors must not crash. The health
 * check flags the filter unready and a re-init fully recovers it.
 * ---------------------------------------------------------------------------
 */

static void scenario_corrupted_covariance(void)
{
    printf("\n-- scenario: corrupted UDU factors fail safe --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_AHRS;
    /* Static platform, but this measures how far ONE magnetometer
       outlier moves yaw -- a stillness fallback pinning the gyro bias
       would mask exactly that. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_rad[2]        = yaw_true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(3.0f);
    cfg.acc_freq_hz            = 50.0f; /* fuse often so the corrupted-factor */
    cfg.mag_freq_hz            = 50.0f; /* epochs exercise the Bierman update  */

    const char* names[5] = {"d negative", "d NaN", "d Inf", "U NaN", "q Inf"};
    int         kind, i;
    for (kind = 0; kind < 5; ++kind)
    {
        printf("      -- corruption: %s\n", names[kind]);
        ahrs_t         a;
        ahrs_time_us_t t = 1000000;
        CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init");
        for (i = 0; i < 100; ++i)
        {
            t += US_PER_SEC / 100;
            ahrs_update(&a, t, gyr0, f_b, mag_b, false);
        }
        CHECK_TRUE(a.is_initialized, "healthy before corruption");

        const int n = a.n; /* 6 in AHRS mode */
        switch (kind)
        {
        case 0: a.d[0] = -1e-4f; break;
        case 1: a.d[2] = nanf(""); break;
        case 2: a.d[n - 1] = INFINITY; break;
        case 3: a.U[(n - 1) * n + 0] = nanf(""); break; /* (row 0, last col) */
        case 4: a.q[1] = (float)INFINITY; break;
        }
        if (kind == 4)
        {
            /* An Inf/NaN component fed through the NEXT gyro integration
               (ins_quat_rotate -> ins_quat_multiply) turns into NaN in
               every output component (any x * Inf - Inf term), so its own
               internal ins_quat_normalize() sees n2 = NaN, takes the "n2 >
               1e-20f is false" arm and silently resets to identity --
               self-healed before ahrs_correct_step()'s health check would
               ever see it, same as the "U NaN" case above. Calling
               ahrs_update() again at the SAME timestamp (dt_sec == 0)
               skips that integration entirely, so ahrs_correct_step()'s
               own ins_quat_normalize(a->q) runs directly on the corrupted
               quaternion: n2 there is a clean +Inf (no NaN terms), so the
               "normal" branch runs, 1/sqrt(+Inf) rounds to 0, and
               0 * (+Inf) = NaN survives into the health check below. */
            ahrs_update(&a, t, gyr0, f_b, mag_b, false);
        }

        /* Prediction + leveling + heading fusion on the broken factors:
           must not crash; the health check pulls the plug. */
        for (i = 0; i < 10; ++i)
        {
            t += US_PER_SEC / 100;
            ahrs_update(&a, t, gyr0, f_b, mag_b, false);
        }
        float roll, pitch, yaw, q[4];
        if (kind == 3)
        {
            /* NaN sigma in kalman_udu_predict()/kalman_udu_scalar() is
               caught by their own singularity guard (sigma/alpha >
               KALMAN_UDU_EPS) before it can reach U/d or
               ahrs_check_health(): the guard cannot tell a NaN apart
               from a legitimately singular update, so it backs out
               instead of applying it, and the corruption is healed
               rather than detected. */
            CHECK_TRUE(a.is_initialized, "self-healed by kalman_udu singularity guard");
            CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "rpy published");
            CHECK_TRUE(ahrs_get_quaternion(&a, q), "quaternion published");
        }
        else
        {
            CHECK_TRUE(!a.is_initialized, "health check tripped");
            /* Fail-safe: nothing is published on the dead instance. */
            CHECK_TRUE(!ahrs_get_rpy(&a, &roll, &pitch, &yaw), "no rpy published");
            CHECK_TRUE(!ahrs_get_quaternion(&a, q), "no quaternion published");
        }

        /* Re-init of the same instance restores normal operation. */
        CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "re-init");
        for (i = 0; i < 200; ++i)
        {
            t += US_PER_SEC / 100;
            ahrs_update(&a, t, gyr0, f_b, mag_b, false);
        }
        CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "rpy valid again");
        CHECK_NEAR(RAD2DEG(roll), 0.0, 1.0, "recovered roll [deg]");
        CHECK_NEAR(RAD2DEG(pitch), 0.0, 1.0, "recovered pitch [deg]");
        CHECK_NEAR(RAD2DEG(yaw), RAD2DEG(yaw_true), 2.0, "recovered yaw [deg]");
    }
}

/* ---------------------------------------------------------------------------
 * Scenario: global chi2-disable override (REQ-AHRS-018, REQ-SUITE-011).
 * With the flag set, a gross magnetometer heading outlier is fused at
 * face value instead of being chi2-downweighted. nav_suite_init()
 * propagates one ins_options_t flag to all four sub-filter configs.
 * ---------------------------------------------------------------------------
 */

static float run_mag_outlier(bool chi2_disable)
{
    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_AHRS;
    /* Static platform, but this measures how far ONE magnetometer
       outlier moves yaw -- a stillness fallback pinning the gyro bias
       would mask exactly that. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_rad[2]        = yaw_true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(3.0f);
    cfg.chi2_disable           = chi2_disable;

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init");
    int i;
    for (i = 0; i < 200; ++i) /* 2 s to settle */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_b, false);
    }

    /* Grossly wrong mag reading: true heading + 90 deg, held for 0.25 s
       (> the default 200 ms mag_freq_hz throttle) so the rate-limited
       mag fusion is guaranteed to fire at least once on it. */
    float f_b2[3], mag_b_outlier[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true + DEG2RAD(90.0f), f_b2, mag_n, mag_b_outlier);
    for (i = 0; i < 25; ++i)
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_b_outlier, false);
    }

    float roll, pitch, yaw;
    CHECK_TRUE(ahrs_get_rpy(&a, &roll, &pitch, &yaw), "healthy after outlier");

    /* REQ-AHRS-019: the downweight counter must fire exactly on the
       outlier fusion when chi2 is active, and never when disabled. */
    if (chi2_disable) { CHECK_TRUE(a.n_downweighted == 0, "chi2_disable: n_downweighted stays 0"); }
    else { CHECK_TRUE(a.n_downweighted == 1, "default: n_downweighted counts the outlier"); }

    return RAD2DEG(fabsf(yaw - yaw_true));
}

static void scenario_ahrs_chi2_disable(void)
{
    printf("\n-- scenario: global chi2-disable override --\n");

    const float shift_downweighted = run_mag_outlier(false);
    const float shift_raw          = run_mag_outlier(true);

    printf("  yaw shift after outlier: downweighted=%g deg, chi2_disable=%g deg\n",
           shift_downweighted, shift_raw);
    CHECK_TRUE(shift_downweighted < 5.0, "default: outlier downweighted, barely moves yaw");
    CHECK_TRUE(shift_raw > 6.0, "chi2_disable: outlier fused at face value, yaw jumps");

    /* REQ-SUITE-011: nav_suite_init() propagates the single
       ins_options_t.chi2_disable flag to all four sub-filter
       config templates, the caller only has to set it once. */
    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.time = 0;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec    = 0.01f;
    opt.max_prediction_time_sec = 0.5f;
    opt.chi2_disable            = true;

    nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");
    CHECK_TRUE(s.ars_cfg.chi2_disable, "propagated to ars_cfg");
    CHECK_TRUE(s.ahrs_cfg.chi2_disable, "propagated to ahrs_cfg");
    CHECK_TRUE(s.baro_cfg.chi2_disable, "propagated to baro_cfg");
    CHECK_TRUE(s.local_gnss_cfg.chi2_disable, "propagated to local_gnss_cfg");
}

/* ---------------------------------------------------------------------------
 * REQ-SUITE-012: nav_suite feeds the parallel ARS/AHRS/baro filters the same
 * calibrated IMU/mag signal as ins (they run the same physical sensors).
 * A constant gyro bias would drift the free-running ARS yaw. Removing it via
 * opt.imu_gyr_fixed_bias keeps the ARS yaw put, proving the ARS consumes
 * the calibrated gyro, not the raw one.
 * ---------------------------------------------------------------------------
 */
static float run_suite_ars_yaw(float gyr_fixed_bias_z)
{
    const float wz = DEG2RAD(5.0f); /* constant gyro bias, body z [rad/s] */

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = US_PER_SEC;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
    init.rpy_init_rad[2]           = 0.0f;
    init.pos_init_stddev_m         = 1.0f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = DEG2RAD(3.0f);
    init.rpy_init_stddev_rad[1]    = DEG2RAD(3.0f);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = DEG2RAD(0.05f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec          = 0.01f;
    opt.max_prediction_time_sec       = 0.5f;
    opt.allow_unlimited_deadreckoning = true; /* IMU-only start (no position aiding) */
    opt.auto_zupt_disable             = true; /* keep the ARS gyro free-running */
    opt.imu_gyr_fixed_bias[2]         = gyr_fixed_bias_z;

    static nav_suite_t s;
    memset(&s, 0, sizeof(s));
    CHECK_TRUE(nav_suite_init(&s, &init, &opt) == 0, "nav_suite_init");
    /* This scenario watches a filter running WITHOUT any stillness
       trigger; opt the ARS/AHRS out of the fallback that now supplies
       one on its own. */
    s.ars_cfg.auto_zaru_disable  = true;
    s.ahrs_cfg.auto_zaru_disable = true;

    const float        acc_b[3] = {0.0f, 0.0f, -9.80665f};
    const float        gyr_b[3] = {0.0f, 0.0f, wz};
    ins_measurements_t m;
    int                i;
    for (i = 0; i < 300; ++i) /* 3 s */
    {
        t += US_PER_SEC / 100;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, acc_b, sizeof(acc_b));
        memcpy(m.gyr.data, gyr_b, sizeof(gyr_b));
        nav_suite_update(&s, &m);
    }
    float roll, pitch, yaw;
    CHECK_TRUE(nav_suite_get_rpy_ars(&s, &roll, &pitch, &yaw), "ars rpy valid");
    return RAD2DEG(yaw);
}

static void scenario_suite_sensor_calibration(void)
{
    printf("\n-- scenario: suite forwards IMU calibration to the ARS (REQ-SUITE-012) --\n");
    const float drift_nocal = run_suite_ars_yaw(0.0f);
    const float drift_cal   = run_suite_ars_yaw(DEG2RAD(5.0f));
    printf("  ARS yaw after 3 s of 5 deg/s gyro bias: no-cal=%g deg, bias-removed=%g deg\n",
           drift_nocal, drift_cal);
    CHECK_TRUE(fabsf(drift_nocal) > 10.0, "no-cal: gyro bias drifts the free-running ARS yaw");
    CHECK_TRUE(fabsf(drift_cal) < 1.0,
               "fixed bias removed: ARS sees calibrated gyro, yaw stays put");
}

/* ---------------------------------------------------------------------------
 * REQ-AHRS-020: attitude overconfidence / covariance-collapse watchdog. An
 * implausibly tight accelerometer leveling + magnetometer heading collapses
 * the attitude covariance below the 0.001 deg floor, tripping the flag.
 * Ordinary sensor noise never reaches it.
 * ---------------------------------------------------------------------------
 */
static void run_ahrs_overconf(float acc_noise, float mag_yaw_sd, ahrs_t* out)
{
    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};
    float       f_b[3], mag_b[3];
    body_meas_from_rpy(0.0f, 0.0f, yaw_true, f_b, mag_n, mag_b);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_AHRS;
    /* Static platform, but this measures how far ONE magnetometer
       outlier moves yaw -- a stillness fallback pinning the gyro bias
       would mask exactly that. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_rad[2]        = yaw_true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(3.0f);
    cfg.acc_noise_mps2         = acc_noise;  /* 0 -> module default */
    cfg.mag_yaw_stddev_rad     = mag_yaw_sd; /* 0 -> module default */

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init");
    int i;
    for (i = 0; i < 2000; ++i) /* 20 s at 100 Hz */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_b, mag_b, false);
    }
    *out = a;
}

static void scenario_ahrs_overconfidence(void)
{
    printf("\n-- scenario: AHRS overconfidence / covariance-collapse watchdog --\n");

    /* Part A: implausibly tight leveling + heading collapse the attitude. */
    ahrs_t a;
    run_ahrs_overconf(1e-5f, DEG2RAD(1e-4f), &a);
    printf("  collapsed: min att stddev %.2e deg; n=%u\n", (double)a.min_att_stddev_deg,
           a.n_overconfident);
    CHECK_TRUE(a.overconfident, "collapsed attitude covariance trips the flag");
    CHECK_TRUE(a.n_overconfident > 0, "overconfident epochs counted");
    CHECK_TRUE(a.min_att_stddev_deg < 1e-3f, "min attitude stddev fell below the floor");

    /* Part B: ordinary sensor noise never reaches the floor. */
    ahrs_t b;
    run_ahrs_overconf(0.0f, 0.0f, &b);
    printf("  normal:    min att stddev %.3g deg; overconfident=%d\n", (double)b.min_att_stddev_deg,
           (int)b.overconfident);
    CHECK_TRUE(!b.overconfident, "default sensor noise never trips the flag");
    CHECK_TRUE(b.min_att_stddev_deg > 1e-3f, "min attitude stddev stays physically plausible");
}

/* ---------------------------------------------------------------------------
 * Throttled covariance prediction (REQ-AHRS-021)
 *
 * The attitude integrates at the full IMU rate, but the error-state
 * covariance is propagated only once per cfg.kalman_update_dt_sec (Wendel,
 * 2nd ed., ch. 8.2.1). Drive a 100 Hz IMU feed and count how often the
 * covariance prediction actually fires (observed via t_last_cov_predict).
 * ---------------------------------------------------------------------------
 */
static int count_cov_predicts(float cfg_dt_sec, float* out_resolved_dt)
{
    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(1.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(1.0f);
    cfg.kalman_update_dt_sec   = cfg_dt_sec; /* 0 -> resolved default */

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    if (ahrs_init(&a, &cfg, t) != 0)
    {
        CHECK_TRUE(0, "ahrs_init succeeds");
        return -1;
    }
    *out_resolved_dt = a.cfg.kalman_update_dt_sec;

    /* Level, non-rotating body: acc = [0,0,-g], gyro = 0. Only the throttled
       prediction advances t_last_cov_predict (leveling/ZARU touch other
       clocks), so it isolates the covariance-prediction cadence. */
    const float          gyr[3] = {0.0f, 0.0f, 0.0f};
    const float          acc[3] = {0.0f, 0.0f, -GRAVITY};
    const ahrs_time_us_t period_us =
        (ahrs_time_us_t)(a.cfg.kalman_update_dt_sec * (float)US_PER_SEC);

    ahrs_time_us_t prev     = a.t_last_cov_predict;
    int            predicts = 0, i;
    for (i = 0; i < 200; ++i) /* 2 s at 100 Hz */
    {
        t += US_PER_SEC / 100; /* 10 ms IMU step -> 5x faster than the 20 Hz default */
        ahrs_update(&a, t, gyr, acc, (const float*)0, false);
        if (a.t_last_cov_predict != prev)
        {
            /* Each prediction spans at least the configured period: the
               covariance is NOT propagated at the 100 Hz IMU rate. */
            CHECK_TRUE(a.t_last_cov_predict - prev >= period_us,
                       "cov prediction spans >= configured period");
            prev = a.t_last_cov_predict;
            predicts++;
        }
    }
    return predicts;
}

static void scenario_covariance_throttled(void)
{
    printf("\n-- scenario: throttled covariance prediction (REQ-AHRS-021) --\n");

    /* (a) default period resolves to 20 Hz. At 100 Hz IMU input the
           prediction fires ~40x over 2 s, an order fewer than the 200
           IMU epochs: the strapdown runs full-rate, the covariance not. */
    float dt_def = 0.0f;
    int   n_def  = count_cov_predicts(0.0f, &dt_def);
    printf("  default dt=%.4f s -> %d cov predictions over 200 IMU epochs\n", (double)dt_def,
           n_def);
    CHECK_NEAR(dt_def, 1.0 / 20.0, 1e-6, "default cov period = 20 Hz");
    CHECK_TRUE(n_def >= 35 && n_def <= 45, "default: ~40 predictions (20 Hz), not 200");

    /* (b) explicit override to 10 Hz halves the cadence. */
    float dt_ovr = 0.0f;
    int   n_ovr  = count_cov_predicts(1.0f / 10.0f, &dt_ovr);
    printf("  override dt=%.4f s -> %d cov predictions\n", (double)dt_ovr, n_ovr);
    CHECK_NEAR(dt_ovr, 1.0 / 10.0, 1e-6, "override honoured");
    CHECK_TRUE(n_ovr >= 17 && n_ovr <= 22, "override: ~20 predictions (10 Hz)");
    CHECK_TRUE(n_ovr < n_def, "lower configured rate -> fewer predictions");
}

/* Same idea as count_cov_predicts, but with the IMU epoch spacing under test
   rather than fixed at 100 Hz: the cadence boundary is what matters here. */
static int count_cov_predicts_at(float cfg_dt_sec, ahrs_time_us_t step_us, int epochs)
{
    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(1.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(1.0f);
    cfg.kalman_update_dt_sec   = cfg_dt_sec;

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    if (ahrs_init(&a, &cfg, t) != 0)
    {
        CHECK_TRUE(0, "ahrs_init succeeds");
        return -1;
    }

    const float    gyr[3]   = {0.0f, 0.0f, 0.0f};
    const float    acc[3]   = {0.0f, 0.0f, -GRAVITY};
    ahrs_time_us_t prev     = a.t_last_cov_predict;
    int            predicts = 0, i;
    for (i = 0; i < epochs; ++i)
    {
        t += step_us;
        ahrs_update(&a, t, gyr, acc, (const float*)0, false);
        if (a.t_last_cov_predict != prev)
        {
            predicts++;
            prev = a.t_last_cov_predict;
        }
    }
    return predicts;
}

/* REQ-AHRS-021: the covariance-prediction due test carries the same relative
   tolerance as ins (INS_CADENCE_TOLERANCE), for the same reason -- this
   filter is throttled against the same physical IMU stream. */
static void scenario_kalman_cadence_tolerance(void)
{
    printf("\n-- scenario: covariance cadence tolerance (REQ-AHRS-021) --\n");

    const int n_exact = count_cov_predicts_at(0.01f, 10000, 500);
    printf("  period 10000 us, epochs 10000 us -> %d predictions / 500 epochs\n", n_exact);
    CHECK_TRUE(n_exact >= 495, "epochs exactly on the period: one prediction each");

    /* Without the tolerance this halves to 250. */
    const int n_short = count_cov_predicts_at(0.01f, 9999, 500);
    printf("  period 10000 us, epochs  9999 us -> %d predictions / 500 epochs\n", n_short);
    CHECK_TRUE(n_short >= 495, "epochs 1 us short: still one prediction each");

    /* The tolerance must not swallow a genuine throttle. */
    const int n_throttled = count_cov_predicts_at(0.05f, 9999, 500);
    printf("  period 50000 us, epochs  9999 us -> %d predictions / 500 epochs\n", n_throttled);
    CHECK_TRUE(n_throttled >= 95 && n_throttled <= 105,
               "5:1 cadence still throttles to ~100, not 500");
}

/* ---------------------------------------------------------------------------
 * REQ-AHRS-022: hard accelerometer gravity-magnitude gate. A sustained
 * lateral specific force (|f| far from g) is not a leveling reference: the
 * gate drops it, so the gated filter's roll/pitch resist the false tilt,
 * whereas a filter with the gate disabled is pulled toward it.
 * ---------------------------------------------------------------------------
 */
static void scenario_acc_gravity_gate(void)
{
    printf("\n-- scenario: accelerometer gravity-magnitude gate (REQ-AHRS-022) --\n");

    float f_level[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_level, (const float*)0, (float*)0);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};
    /* Strong +x lateral force: |f| = sqrt(64 + g^2) ~ 12.6 m/s^2, i.e.
       |f| - g ~ 2.8 > the 2.0 default gate. */
    const float f_accel[3] = {8.0f, 0.0f, -GRAVITY};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = AHRS_MODE_ARS;
    /* This scenario is about what happens WITHOUT a stillness trigger,
       so opt out of the fallback that would now supply one. */
    cfg.auto_zaru_disable      = true;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.acc_cutoff_freq_hz     = 20.0f; /* let the low pass follow the step quickly */
    cfg.acc_freq_hz            = 50.0f; /* fuse often so the contrast is clear */
    cfg.gravity_diff_penalty   = -1.0f; /* -> 0: no soft downweight, so a disabled
                                           gate fuses the bad sample at full weight */

    ahrs_t        a_on, a_off;
    ahrs_config_t cfg_off           = cfg;
    cfg_off.acc_reject_gravity_mps2 = -1.0f; /* disable the hard gate */

    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a_on, &cfg, t) == 0, "init (gate on, default)");
    CHECK_TRUE(ahrs_init(&a_off, &cfg_off, t) == 0, "init (gate off)");

    int i;
    for (i = 0; i < 200; ++i) /* 2 s settle on level truth */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a_on, t, gyr0, f_level, (const float*)0, false);
        ahrs_update(&a_off, t, gyr0, f_level, (const float*)0, false);
    }
    CHECK_TRUE(a_on.n_acc_rejected == 0, "gate: nothing rejected on a gravity-only signal");

    for (i = 0; i < 300; ++i) /* 3 s sustained lateral acceleration */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a_on, t, gyr0, f_accel, (const float*)0, false);
        ahrs_update(&a_off, t, gyr0, f_accel, (const float*)0, false);
    }
    CHECK_TRUE(a_on.n_acc_rejected > 0, "gate rejects the maneuver samples");
    CHECK_TRUE(a_off.n_acc_rejected == 0, "disabled gate never rejects");

    float ron, pon, yon, roff, poff, yoff;
    ahrs_get_rpy(&a_on, &ron, &pon, &yon);
    ahrs_get_rpy(&a_off, &roff, &poff, &yoff);
    printf("   gate on pitch = %.2f deg, gate off pitch = %.2f deg\n", (double)RAD2DEG(pon),
           (double)RAD2DEG(poff));
    CHECK_TRUE(fabsf(RAD2DEG(pon)) < 0.5, "gated pitch resists the maneuver");
    CHECK_TRUE(fabsf(RAD2DEG(poff)) > 2.0, "ungated pitch pulled by the maneuver");
    CHECK_TRUE(fabsf(RAD2DEG(poff)) > 20.0f * fabsf(RAD2DEG(pon)),
               "gate makes a large difference vs. leaving it off");
}

/* ---------------------------------------------------------------------------
 * REQ-AHRS-023: attitude-precision restart watchdog. With leveling starved
 * (every accelerometer sample tripping the gravity gate) the loose initial
 * attitude covariance never shrinks. Past the warm-up the watchdog marks the
 * filter uninitialized. A disabled watchdog leaves it running, and a fresh
 * init on a good signal recovers.
 * ---------------------------------------------------------------------------
 */
/* The AHRS runs the same divergence watchdogs as ins on its own gyro
 * bias estimate (see log.h). The reports are log output, but the
 * throttling state behind them is filter state, so that is what this
 * reads -- which keeps it meaningful in a build that compiles the log
 * calls out.
 *
 * Thresholds are private to ahrs.c (AHRS_LOG_*): a 10 s runaway window,
 * a 10 deg/s sanity bound and a 60 s throttle between repeats, all
 * gated on the config's restart warm-up.
 */
static void scenario_ahrs_bias_diagnostics(void)
{
    printf("\n-- scenario: ahrs gyro-bias runaway and sanity diagnostics --\n");

    float f_level[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_level, (const float*)0, (float*)0);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.restart_warmup_sec     = 2.0f;
    /* The injected bias below is far outside anything the filter would
       produce on its own, so leave the precision watchdog out of it. */
    cfg.precision_restart_disable = true;
    /* A stillness fallback would pull the injected bias straight back
       towards zero, which is the one thing this must not do. */
    cfg.auto_zaru_disable = true;

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init");

    int i;
#define AHRS_DIAG_RUN(SECONDS)                                         \
    do {                                                               \
        int n_ = (int)((SECONDS)*100.0f + 0.5f);                       \
        for (i = 0; i < n_; ++i)                                       \
        {                                                              \
            t += US_PER_SEC / 100;                                     \
            ahrs_update(&a, t, gyr0, f_level, (const float*)0, false); \
        }                                                              \
    } while (0)

    AHRS_DIAG_RUN(1.0f); /* inside the warm-up: the estimate may still move */
    CHECK_TRUE(a.log_state.t_gyr_bias_window == 0, "no bias window opened inside the warm-up");

    AHRS_DIAG_RUN(2.0f); /* past it */
    CHECK_TRUE(a.log_state.t_gyr_bias_window != 0, "bias window opened after the warm-up");
    CHECK_TRUE(a.log_state.t_last_gyr_bias_sanity_warn == 0, "a healthy bias raises nothing");

    /* A diverged estimate, injected directly: no realistic gyro input
       drives it this far, and the point is the watchdog rather than the
       path that got there. 20 deg/s is twice the sanity bound. */
    a.gyr_bias_rps[0] = DEG2RAD(20.0f);

    AHRS_DIAG_RUN(11.0f); /* one full runaway window on the new value */
    CHECK_TRUE(a.log_state.t_last_gyr_bias_sanity_warn != 0,
               "an implausible gyro bias is reported");
    CHECK_TRUE(a.log_state.gyr_bias_window_dps > 10.0f,
               "the runaway window carries the new magnitude");

    const ahrs_time_us_t first_warn = a.log_state.t_last_gyr_bias_sanity_warn;
    AHRS_DIAG_RUN(20.0f);
    CHECK_TRUE(a.log_state.t_last_gyr_bias_sanity_warn == first_warn,
               "the complaint is throttled while the bias stays implausible");

    AHRS_DIAG_RUN(45.0f); /* past the 60 s repeat interval */
    CHECK_TRUE(a.log_state.t_last_gyr_bias_sanity_warn > first_warn,
               "and repeated once the interval has passed");
#undef AHRS_DIAG_RUN
}

/* ---------------------------------------------------------------------------
 * Scenario: no-magnetometer / yaw-runaway diagnostics (log.h). Covers the
 * arms scenario_ahrs_bias_diagnostics() above does not: a mag outage that
 * stays BELOW the yaw-stddev warn threshold, a warning that REPEATS after
 * the 60 s throttle, and a single-window growth fast enough to trip the
 * runaway check.
 * ---------------------------------------------------------------------------
 */

static void scenario_ahrs_mag_gap_diagnostics(void)
{
    printf("\n-- scenario: ahrs no-magnetometer / yaw-runaway diagnostics --\n");

    float f_level[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_level, (const float*)0, (float*)0);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};

    /* Case A: outage well past the 10 s warn threshold, but with a quiet
       enough gyro that yaw stddev never crosses the 10 deg warn level --
       the "gap long enough, uncertainty still fine" arm. */
    ahrs_config_t cfg_quiet;
    memset(&cfg_quiet, 0, sizeof(cfg_quiet));
    cfg_quiet.mode                   = AHRS_MODE_AHRS;
    cfg_quiet.rpy_init_stddev_rad[0] = DEG2RAD(1.0f);
    cfg_quiet.rpy_init_stddev_rad[1] = DEG2RAD(1.0f);
    cfg_quiet.rpy_init_stddev_rad[2] = DEG2RAD(1.0f);
    cfg_quiet.auto_zaru_disable      = true;
    /* Yaw stddev growth without a magnetometer is dominated by the gyro
       BIAS uncertainty leaking in through the state transition matrix
       (~gyr_bias_init_stddev_rps[2] * elapsed time), not by gyr_noise_psd
       -- pin it far below the default (DEG2RAD(1.5)) to keep this flat. */
    cfg_quiet.gyr_bias_init_stddev_rps[2] = DEG2RAD(0.01f);

    ahrs_t         aq;
    ahrs_time_us_t tq = 0;
    CHECK_TRUE(ahrs_init(&aq, &cfg_quiet, tq) == 0, "init (quiet gyro)");
    int i;
    for (i = 0; i < 1200; ++i) /* 12 s: past the 10 s mag-gap warn threshold */
    {
        tq += US_PER_SEC / 100;
        ahrs_update(&aq, tq, gyr0, f_level, (const float*)0, false);
    }
    float roll_sd_q, pitch_sd_q, yaw_sd_q;
    CHECK_TRUE(ahrs_get_rpy_stddev(&aq, &roll_sd_q, &pitch_sd_q, &yaw_sd_q), "stddev readable");
    CHECK_TRUE(RAD2DEG(yaw_sd_q) < 10.0f, /* AHRS_LOG_YAW_STDDEV_WARN_DEG, private to ahrs.c */
               "quiet gyro: yaw stddev stays under the warn level");
    CHECK_TRUE(aq.log_state.t_last_mag_gap_warn == 0, "quiet gyro: long gap alone does not warn");

    /* Case B/C: the DEFAULT gyro bias uncertainty (unlike case A's pinned-
       down one) is enough on its own -- ~1.5 deg/s of yaw stddev growth,
       crossing the 10 deg warn level well inside one 10 s runaway window
       (case C) and staying there long enough for the 60 s repeat throttle
       to re-fire (case B). */
    ahrs_config_t cfg_noisy;
    memset(&cfg_noisy, 0, sizeof(cfg_noisy));
    cfg_noisy.mode                   = AHRS_MODE_AHRS;
    cfg_noisy.rpy_init_stddev_rad[0] = DEG2RAD(1.0f);
    cfg_noisy.rpy_init_stddev_rad[1] = DEG2RAD(1.0f);
    cfg_noisy.rpy_init_stddev_rad[2] = DEG2RAD(1.0f);
    cfg_noisy.auto_zaru_disable      = true;
    /* This growth is exactly what would otherwise trip the attitude-
       precision restart watchdog (REQ-AHRS-023, see
       scenario_precision_restart) -- not the mechanism under test here. */
    cfg_noisy.precision_restart_disable = true;

    ahrs_t         an;
    ahrs_time_us_t tn = 0;
    CHECK_TRUE(ahrs_init(&an, &cfg_noisy, tn) == 0, "init (noisy gyro)");
    for (i = 0; i < 2200; ++i) /* 22 s: two runaway windows past the mag-gap warn */
    {
        tn += US_PER_SEC / 100;
        ahrs_update(&an, tn, gyr0, f_level, (const float*)0, false);
    }
    CHECK_TRUE(an.log_state.t_last_mag_gap_warn != 0, "noisy gyro: mag-gap warning fired");
    const ahrs_time_us_t first_warn = an.log_state.t_last_mag_gap_warn;

    for (i = 0; i < 6100; ++i) /* +61 s: past the 60 s repeat throttle */
    {
        tn += US_PER_SEC / 100;
        ahrs_update(&an, tn, gyr0, f_level, (const float*)0, false);
    }
    CHECK_TRUE(an.log_state.t_last_mag_gap_warn > first_warn,
               "noisy gyro: mag-gap warning repeats past the 60 s throttle");
}

static void scenario_precision_restart(void)
{
    printf("\n-- scenario: attitude-precision restart watchdog (REQ-AHRS-023) --\n");

    float f_level[3];
    body_meas_from_rpy(0.0f, 0.0f, 0.0f, f_level, (const float*)0, (float*)0);
    const float gyr0[3] = {0.0f, 0.0f, 0.0f};
    /* |f| - g ~ 2g: always tripping the gravity gate, so leveling never
       corrects and the (deliberately loose) initial covariance stands. */
    const float f_bad[3] = {0.0f, 0.0f, -3.0f * GRAVITY};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                      = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0]    = DEG2RAD(30.0f); /* loose: above threshold */
    cfg.rpy_init_stddev_rad[1]    = DEG2RAD(30.0f);
    cfg.restart_att_stddev_rad[0] = DEG2RAD(20.0f);
    cfg.restart_att_stddev_rad[1] = DEG2RAD(20.0f);
    cfg.restart_warmup_sec        = 1.0f;

    ahrs_t         a;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init");

    int i;
    for (i = 0; i < 50; ++i) /* 0.5 s < 1 s warm-up */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_bad, (const float*)0, false);
    }
    CHECK_TRUE(a.is_initialized, "no restart during warm-up");
    CHECK_TRUE(a.n_restart == 0, "restart counter still 0 in warm-up");

    for (i = 0; i < 100; ++i) /* past the warm-up */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_bad, (const float*)0, false);
    }
    CHECK_TRUE(!a.is_initialized, "precision watchdog marked the filter uninitialized");
    CHECK_TRUE(a.n_restart == 1, "restart counted once");
    float r, p, y;
    CHECK_TRUE(!ahrs_get_rpy(&a, &r, &p, &y), "nothing published after restart");

    /* Standalone recovery: a fresh init on a good signal restores it. */
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "re-init");
    for (i = 0; i < 300; ++i) /* 3 s good leveling, past the warm-up */
    {
        t += US_PER_SEC / 100;
        ahrs_update(&a, t, gyr0, f_level, (const float*)0, false);
    }
    CHECK_TRUE(a.is_initialized, "healthy again on a good signal");
    CHECK_TRUE(a.n_restart == 0, "no spurious restart once converged");

    /* Opt-out: identical divergence, watchdog disabled -> never restarts. */
    ahrs_config_t cfg_off             = cfg;
    cfg_off.precision_restart_disable = true;
    ahrs_t         b;
    ahrs_time_us_t tb = 1000000;
    CHECK_TRUE(ahrs_init(&b, &cfg_off, tb) == 0, "init (watchdog disabled)");
    for (i = 0; i < 300; ++i) /* 3 s, well past the warm-up */
    {
        tb += US_PER_SEC / 100;
        ahrs_update(&b, tb, gyr0, f_bad, (const float*)0, false);
    }
    CHECK_TRUE(b.is_initialized, "disabled watchdog never restarts");
    CHECK_TRUE(b.n_restart == 0, "disabled watchdog counter stays 0");

    /* Per-axis opt-out: a negative threshold means "do not check this
       axis" and resolves to 0, which is not the same as leaving the
       field at 0 (that picks the built-in default). Same divergence as
       above, so a threshold that was silently defaulted instead would
       restart the filter here. */
    ahrs_config_t cfg_axis             = cfg;
    cfg_axis.restart_att_stddev_rad[0] = -1.0f;
    cfg_axis.restart_att_stddev_rad[1] = -1.0f;
    cfg_axis.restart_att_stddev_rad[2] = -1.0f;
    ahrs_t         c;
    ahrs_time_us_t tc = 1000000;
    CHECK_TRUE(ahrs_init(&c, &cfg_axis, tc) == 0, "init (per-axis opt-out)");
    CHECK_NEAR(c.cfg.restart_att_stddev_rad[0], 0.0, 1e-12,
               "a negative roll threshold resolves to unchecked");
    CHECK_NEAR(c.cfg.restart_att_stddev_rad[2], 0.0, 1e-12,
               "a negative yaw threshold resolves to unchecked");
    for (i = 0; i < 300; ++i)
    {
        tc += US_PER_SEC / 100;
        ahrs_update(&c, tc, gyr0, f_bad, (const float*)0, false);
    }
    CHECK_TRUE(c.is_initialized, "an unchecked axis never restarts");
    CHECK_TRUE(c.n_restart == 0, "unchecked axis counter stays 0");
}

/* ahrs_update() must be exactly ahrs_predict_step() followed by
   ahrs_correct_step() (REQ-AHRS-025): drive two identical filters
   through the same epoch stream, one via ahrs_update(), the other
   manually split, and require the end state/covariance to match
   bit-for-bit. Also checks that phi_out is filled with a non-trivial
   (non-identity) transition matrix whenever the covariance is actually
   propagated. */
static void scenario_predict_correct_equivalence(void)
{
    printf("\n-- scenario: ahrs_update() == ahrs_predict_step()+ahrs_correct_step() --\n");

    const float mag_n[3] = {20.0f, 0.0f, 45.0f};

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                   = AHRS_MODE_AHRS;
    cfg.rpy_init_stddev_rad[0] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1] = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2] = DEG2RAD(10.0f);

    ahrs_t         a1, a2;
    ahrs_time_us_t t = 1000000;
    CHECK_TRUE(ahrs_init(&a1, &cfg, t) == 0, "a1 init");
    CHECK_TRUE(ahrs_init(&a2, &cfg, t) == 0, "a2 init");

    bool  phi_checked = false;
    float phi_check[AHRS_UNKNOWNS_MAX * AHRS_UNKNOWNS_MAX];
    int   i;
    for (i = 0; i < 1000; ++i)
    {
        t += US_PER_SEC / 100;
        const float yaw = 0.001f * (float)i;
        float       f_b[3], mag_b[3];
        body_meas_from_rpy(0.02f, -0.01f, yaw, f_b, mag_n, mag_b);
        const float gyr_meas[3] = {0.0f, 0.0f, 0.001f};
        const bool  zaru        = (i % 200) < 5;

        ahrs_update(&a1, t, gyr_meas, f_b, mag_b, zaru);

        float phi[AHRS_UNKNOWNS_MAX * AHRS_UNKNOWNS_MAX];
        memset(phi, 0, sizeof(phi));
        const int status = ahrs_predict_step(&a2, t, gyr_meas, f_b, mag_b, zaru, phi);
        ahrs_correct_step(&a2);

        if (!phi_checked && (status & AHRS_EPOCH_COV_PROPAGATED))
        {
            memcpy(phi_check, phi, sizeof(phi));
            phi_checked = true;
        }
    }

    CHECK_TRUE(phi_checked, "phi_out was filled at least once");
    if (phi_checked)
    {
        float off_diag_sq = 0.0f;
        int   r, c;
        for (r = 0; r < AHRS_UNKNOWNS_MAX; ++r)
        {
            for (c = 0; c < AHRS_UNKNOWNS_MAX; ++c)
            {
                if (r != c)
                {
                    const float v = phi_check[r + c * AHRS_UNKNOWNS_MAX];
                    off_diag_sq += v * v;
                }
            }
        }
        CHECK_TRUE(off_diag_sq > 0.0f, "phi_out has non-trivial attitude/bias coupling");
    }

    float roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    CHECK_TRUE(ahrs_get_rpy(&a1, &roll1, &pitch1, &yaw1), "a1 rpy valid");
    CHECK_TRUE(ahrs_get_rpy(&a2, &roll2, &pitch2, &yaw2), "a2 rpy valid");
    CHECK_NEAR(roll1, roll2, 0.0, "roll matches");
    CHECK_NEAR(pitch1, pitch2, 0.0, "pitch matches");
    CHECK_NEAR(yaw1, yaw2, 0.0, "yaw matches");

    float gb1[3], gb2[3];
    ahrs_get_bias_gyr(&a1, gb1);
    ahrs_get_bias_gyr(&a2, gb2);
    CHECK_NEAR(gb1[0], gb2[0], 0.0, "gyro bias x matches");
    CHECK_NEAR(gb1[1], gb2[1], 0.0, "gyro bias y matches");
    CHECK_NEAR(gb1[2], gb2[2], 0.0, "gyro bias z matches");

    CHECK_TRUE(memcmp(a1.U, a2.U, sizeof(a1.U)) == 0, "covariance U factor matches");
    CHECK_TRUE(memcmp(a1.d, a2.d, sizeof(a1.d)) == 0, "covariance d factor matches");
    CHECK_TRUE(a1.epoch == a2.epoch, "epoch counter matches");
}

/* Same equivalence property one layer up: nav_suite_update() must be
   exactly nav_suite_predict_step() followed by nav_suite_correct_step()
   (REQ-SUITE-021). Runs IMU+GNSS+mag+baro through two suite instances and
   compares the ins sub-filter's state/covariance (the one nav_suite's
   split forwards phi_out for) bit-for-bit. */
static void scenario_suite_predict_correct_equivalence(void)
{
    printf("\n-- scenario: nav_suite_update() == "
           "nav_suite_predict_step()+nav_suite_correct_step() --\n");

    const float yaw_true = DEG2RAD(30.0f);
    const float mag_n[3] = {20.0f, 0.0f, 44.0f};

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    ahrs_time_us_t t = 1000000;
    init.time        = t;
    ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
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
    init.magnetic_n[0]             = mag_n[0];
    init.magnetic_n[1]             = mag_n[1];
    init.magnetic_n[2]             = mag_n[2];

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.kalman_update_dt_sec               = 0.01f;
    opt.max_prediction_time_sec            = 0.5f;
    opt.gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt.gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt.gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt.gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt.magnetometer_min_delay_ms          = 200;
    opt.allow_unlimited_deadreckoning      = true;

    static nav_suite_t s1, s2; /* keep the large structs off the stack */
    memset(&s1, 0, sizeof(s1));
    memset(&s2, 0, sizeof(s2));
    CHECK_TRUE(nav_suite_init(&s1, &init, &opt) == 0, "s1 init");
    CHECK_TRUE(nav_suite_init(&s2, &init, &opt) == 0, "s2 init");

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));

    bool  phi_checked = false;
    float phi_check[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX];
    int   i;
    for (i = 0; i < 800; ++i)
    {
        t += US_PER_SEC / 100;
        float f_b[3], mag_b[3];
        body_meas_from_rpy(0.01f, -0.02f, yaw_true, f_b, mag_n, mag_b);

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 0.01f;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        memcpy(m.acc.data, f_b, sizeof(f_b));
        m.gyr.data[2] = 0.001f;
        memcpy(m.mag.data, mag_b, sizeof(mag_b));
        m.mag.is_valid = true;

        if (i % 20 == 0)
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
            m.gnss_pos.is_valid                                                   = true;
        }
        if (i % 5 == 0)
        {
            m.baro.pressure_pa = pressure_at_altitude(0.0f);
            m.baro.stddev_m    = 1.0f;
            m.baro.is_valid    = true;
        }

        nav_suite_update(&s1, &m);

        float phi[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX];
        memset(phi, 0, sizeof(phi));
        const int status = nav_suite_predict_step(&s2, &m, phi);
        nav_suite_correct_step(&s2);

        if (!phi_checked && (status & INS_EPOCH_COV_PROPAGATED))
        {
            memcpy(phi_check, phi, sizeof(phi));
            phi_checked = true;
        }
    }

    CHECK_TRUE(phi_checked, "phi_out was filled at least once");
    if (phi_checked)
    {
        const int n = s2.ins.n;
        CHECK_NEAR(phi_check[(INS_IDX_POS + 0) + (INS_IDX_VEL + 0) * n], 0.01f, 1e-9,
                   "phi pos/vel N coupling");
    }

    float p1[3], p2[3], v1[3], v2[3];
    CHECK_TRUE(ins_get_position_local(&s1.ins, p1), "s1 position readable");
    CHECK_TRUE(ins_get_position_local(&s2.ins, p2), "s2 position readable");
    ins_get_velocity_ned(&s1.ins, v1);
    ins_get_velocity_ned(&s2.ins, v2);
    CHECK_NEAR(p1[0], p2[0], 0.0, "ins position N matches");
    CHECK_NEAR(p1[1], p2[1], 0.0, "ins position E matches");
    CHECK_NEAR(p1[2], p2[2], 0.0, "ins position D matches");
    CHECK_NEAR(v1[0], v2[0], 0.0, "ins velocity N matches");
    CHECK_NEAR(v1[1], v2[1], 0.0, "ins velocity E matches");
    CHECK_NEAR(v1[2], v2[2], 0.0, "ins velocity D matches");

    CHECK_TRUE(memcmp(s1.ins.U, s2.ins.U, sizeof(s1.ins.U)) == 0, "ins covariance U matches");
    CHECK_TRUE(memcmp(s1.ins.d, s2.ins.d, sizeof(s1.ins.d)) == 0, "ins covariance d matches");

    float roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    CHECK_TRUE(ahrs_get_rpy(&s1.ahrs, &roll1, &pitch1, &yaw1), "s1 ahrs rpy valid");
    CHECK_TRUE(ahrs_get_rpy(&s2.ahrs, &roll2, &pitch2, &yaw2), "s2 ahrs rpy valid");
    CHECK_NEAR(roll1, roll2, 0.0, "ahrs roll matches");
    CHECK_NEAR(pitch1, pitch2, 0.0, "ahrs pitch matches");
    CHECK_NEAR(yaw1, yaw2, 0.0, "ahrs yaw matches");

    float h1, h2;
    CHECK_TRUE(baro_alt_get_height(&s1.baro_alt, &h1), "s1 baro height readable");
    CHECK_TRUE(baro_alt_get_height(&s2.baro_alt, &h2), "s2 baro height readable");
    CHECK_NEAR(h1, h2, 0.0, "baro_alt height matches");
}

/* ---------------------------------------------------------------------------
 * Scenario: ahrs_resolve_config() passthrough. Every "<= 0 -> default"
 * field in ahrs_resolve_config() is exercised elsewhere only via a
 * zeroed cfg (always defaulted): the OTHER arm -- an explicit, already
 * valid value that must survive untouched -- never runs. Set them all
 * to distinct non-default values and check they come back unchanged.
 * ---------------------------------------------------------------------------
 */

static void scenario_ahrs_config_passthrough(void)
{
    printf("\n-- scenario: ahrs_resolve_config() explicit-value passthrough --\n");

    ahrs_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode                             = AHRS_MODE_ARS;
    cfg.rpy_init_stddev_rad[0]           = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[1]           = DEG2RAD(3.0f);
    cfg.rpy_init_stddev_rad[2]           = DEG2RAD(3.0f);
    cfg.gyr_bias_rw                      = 0.00042f;
    cfg.acc_reject_gravity_mps2          = 4.2f;
    cfg.chi2_threshold                   = 5.5f;
    cfg.mag_chi2_threshold               = 6.6f;
    cfg.mag_field_tolerance              = 0.42f;
    cfg.auto_zaru_static_gyr_rps         = DEG2RAD(1.2f);
    cfg.auto_zaru_static_acc_mps2        = 0.24f;
    cfg.auto_zaru_static_gyr_stddev_rps  = DEG2RAD(0.12f);
    cfg.auto_zaru_static_acc_stddev_mps2 = 0.06f;
    cfg.auto_zaru_dwell_sec              = 0.75f;

    ahrs_t         a;
    ahrs_time_us_t t = 0;
    CHECK_TRUE(ahrs_init(&a, &cfg, t) == 0, "init with explicit values succeeds");
    CHECK_NEAR(a.cfg.gyr_bias_rw, cfg.gyr_bias_rw, 1e-9, "gyr_bias_rw not defaulted");
    CHECK_NEAR(a.cfg.acc_reject_gravity_mps2, cfg.acc_reject_gravity_mps2, 1e-9,
               "acc_reject_gravity_mps2 not defaulted");
    CHECK_NEAR(a.cfg.chi2_threshold, cfg.chi2_threshold, 1e-9, "chi2_threshold not defaulted");
    CHECK_NEAR(a.cfg.mag_chi2_threshold, cfg.mag_chi2_threshold, 1e-9,
               "mag_chi2_threshold not defaulted");
    CHECK_NEAR(a.cfg.mag_field_tolerance, cfg.mag_field_tolerance, 1e-9,
               "mag_field_tolerance not defaulted");
    CHECK_NEAR(a.cfg.auto_zaru_static_gyr_rps, cfg.auto_zaru_static_gyr_rps, 1e-9,
               "auto_zaru_static_gyr_rps not defaulted");
    CHECK_NEAR(a.cfg.auto_zaru_static_acc_mps2, cfg.auto_zaru_static_acc_mps2, 1e-9,
               "auto_zaru_static_acc_mps2 not defaulted");
    CHECK_NEAR(a.cfg.auto_zaru_static_gyr_stddev_rps, cfg.auto_zaru_static_gyr_stddev_rps, 1e-9,
               "auto_zaru_static_gyr_stddev_rps not defaulted");
    CHECK_NEAR(a.cfg.auto_zaru_static_acc_stddev_mps2, cfg.auto_zaru_static_acc_stddev_mps2, 1e-9,
               "auto_zaru_static_acc_stddev_mps2 not defaulted");
    CHECK_NEAR(a.cfg.auto_zaru_dwell_sec, cfg.auto_zaru_dwell_sec, 1e-9,
               "auto_zaru_dwell_sec not defaulted");

    /* gyr_bias_rw < 0 is clamped to 0 first (then still defaulted, since
       0 <= 0): the "negative input" arm, never exercised by the zeroed
       configs (already 0) or the passthrough case above (already
       positive). */
    ahrs_config_t cfg_neg = cfg;
    cfg_neg.gyr_bias_rw   = -1.0f;
    ahrs_t a_neg;
    CHECK_TRUE(ahrs_init(&a_neg, &cfg_neg, t) == 0, "init with negative gyr_bias_rw succeeds");
    CHECK_TRUE(a_neg.cfg.gyr_bias_rw > 0.0f, "negative gyr_bias_rw clamped then defaulted");
}

int main(void)
{
    scenario_pyahrs_example();
    scenario_free_yaw_integration();
    scenario_mag_yaw_convergence();
    scenario_wmm_position_aiding();
    scenario_mag_heading_helper();
    scenario_nav_suite();
    scenario_suite_att_hint();
    scenario_suite_quality_exit_rebootstrap();
    scenario_suite_init_att_hint();
    scenario_suite_init_att_hint_roll_pitch();
    scenario_suite_yaw_carry();
    scenario_suite_heading_switch_no_jump();
    scenario_suite_heading_cross_check();
    scenario_nan_inputs();
    scenario_tunnel();
    scenario_time_anomaly();
    scenario_mag_edge_cases();
    scenario_zaru();
    scenario_auto_zaru_fallback();
    scenario_auto_zaru_vibration();
    scenario_auto_zaru_disable_runtime();
    scenario_zaru_applied_accessor();
    scenario_suite_selfhealing();
    scenario_suite_fallbacks();
    scenario_suite_height_cross_check();
    scenario_suite_zaru();
    scenario_suite_stillness_propagation();
    scenario_suite_auto_zupt_zaru_disable();
    scenario_corrupted_covariance();
    scenario_ahrs_chi2_disable();
    scenario_suite_sensor_calibration();
    scenario_ahrs_overconfidence();
    scenario_covariance_throttled();
    scenario_kalman_cadence_tolerance();
    scenario_acc_gravity_gate();
    scenario_ahrs_bias_diagnostics();
    scenario_ahrs_mag_gap_diagnostics();
    scenario_precision_restart();
    scenario_predict_correct_equivalence();
    scenario_suite_predict_correct_equivalence();
    scenario_ahrs_config_passthrough();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
