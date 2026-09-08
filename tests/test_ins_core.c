/** @file test_ins_core.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Integration test for the Strapdown+Predict portion of the C filter.
 *
 * Three scenarios:
 *   1. Stationary:  perfect acc=[0,0,-g_body] (body aligned with n-frame),
 *                   omega=0 -> position and velocity must stay constant,
 *                   attitude must stay constant.
 *   2. Free fall:   acc=[0,0,0] (sensor in free fall), omega=0
 *                   -> velocity in NED's Down direction grows as g*t,
 *                   position down grows as 0.5*g*t^2.
 *   3. Rotation:    stationary accel, constant omega about Z.
 *                   -> attitude yaw grows as omega*t.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "ins.h"
#include "geodetic_toolbox.h"
#include "magnetic_model.h"
#include "sensor_defaults.h"

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

/* Epoch spacing in microseconds from a sample period in seconds.
 *
 * Rounds, and must: the obvious us_from_sec(dt) truncates instead,
 * and on any target where FLT_EVAL_METHOD is 2 (32-bit x86 keeping float
 * arithmetic in the x87 registers) the cast to integer reads the extended-
 * precision intermediate rather than a rounded float. dt = 0.01f is
 * 0.00999999977, so that product is 9999.99977 and the cast yields 9999,
 * not 10000 -- while the same expression on an SSE target yields 10000 and
 * every test here passes.
 *
 * A microsecond per epoch sounds harmless and is not: a scenario configuring
 * kalman_update_dt_sec to its own sample period then sits permanently 1 us
 * below the covariance-prediction threshold (ins.c, "dt_pred >=
 * ins_kalman_dt(f)"), so the prediction fires on every SECOND epoch and each
 * Phi spans twice the expected dt. */
static ins_time_us_t us_from_sec(float sec) { return (ins_time_us_t)((double)sec * 1e6 + 0.5); }

static void fill_default_init(ins_init_t* init, ins_time_us_t t0)
{
    memset(init, 0, sizeof(*init));
    init->time = t0;
    /* Stuttgart-ish */
    const double lat = 48.783 * M_PI / 180.0;
    const double lon = 9.181 * M_PI / 180.0;
    const double h   = 300.0;
    ins_latlonh_to_ecef(lat, lon, h, init->x_ecef);
    init->xdot_ecef[0]    = 0;
    init->xdot_ecef[1]    = 0;
    init->xdot_ecef[2]    = 0;
    init->rpy_init_rad[0] = 0.0f;
    init->rpy_init_rad[1] = 0.0f;
    init->rpy_init_rad[2] = 0.0f;

    init->pos_init_stddev_m         = 10.0f;
    init->vel_init_stddev_mps       = 1.0f;
    init->rpy_init_stddev_rad[0]    = (float)(5.0 * M_PI / 180.0);
    init->rpy_init_stddev_rad[1]    = (float)(5.0 * M_PI / 180.0);
    init->acc_bias_init_stddev_mps2 = 0.1f;
    init->gyr_bias_init_stddev_rps  = (float)(0.01 * M_PI / 180.0);

    init->pos_pred_stddev_m_sqrts         = 0.01f;
    init->vel_pred_stddev_mps_sqrts       = 0.05f;
    init->rpy_pred_stddev_rad_sqrts       = (float)(0.01 * M_PI / 180.0);
    init->acc_bias_pred_stddev_mps2_sqrts = 1e-4f;
    init->gyr_bias_pred_stddev_rps_sqrts  = 1e-6f;

    init->zero_vel_stddev_mps = 0.01f;
    init->zero_rot_stddev_rps = (float)(0.001 * M_PI / 180.0);

    /* leave gravity_n = 0 -> filter will compute from position. */
    /* magnetic_n: dummy */
    init->magnetic_n[0] = 20.0f;
    init->magnetic_n[1] = 0.0f;
    init->magnetic_n[2] = 45.0f;
}

static void fill_default_opt(ins_options_t* opt)
{
    memset(opt, 0, sizeof(*opt));
    opt->kalman_update_dt_sec               = 0.01f; /* 100 Hz predict */
    opt->max_prediction_time_sec            = 0.5f;
    opt->gnss_max_horizontal_pos_stddev_m   = 10.0f;
    opt->gnss_max_vertical_pos_stddev_m     = 20.0f;
    opt->gnss_max_horizontal_vel_stddev_mps = 1.0f;
    opt->gnss_max_vertical_vel_stddev_mps   = 2.0f;
    opt->magnetometer_min_delay_ms          = 100;
    opt->allow_unlimited_deadreckoning      = true;
    /* Bootstrap on the first usable fix: most scenarios test the bootstrap
       mechanics, not the entry dwell (REQ-NAV-045, on by default in
       production). The dwell scenarios re-enable it explicitly. */
    opt->gnss_init_dwell_disable = true;
}

static float test_vec3_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vec3_assign(float dst[3], const float src[3])
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

static void set_imu(ins_measurements_t* m, const float acc[3], const float gyr[3], float dt)
{
    m->strapdown_dt_sec = dt;
    m->acc.data[0]      = acc[0];
    m->acc.data[1]      = acc[1];
    m->acc.data[2]      = acc[2];
    m->acc.Qll_diag[0] = m->acc.Qll_diag[1] = m->acc.Qll_diag[2] = 0.01f;
    m->acc.is_valid                                              = true;
    m->gyr.data[0]                                               = gyr[0];
    m->gyr.data[1]                                               = gyr[1];
    m->gyr.data[2]                                               = gyr[2];
    m->gyr.Qll_diag[0] = m->gyr.Qll_diag[1] = m->gyr.Qll_diag[2] = 1e-5f;
    m->gyr.is_valid                                              = true;
}

static void set_baro(ins_measurements_t* m, float pressure_pa, float stddev_m)
{
    m->baro.pressure_pa = pressure_pa;
    m->baro.stddev_m    = stddev_m;
    m->baro.is_valid    = true;
}

/* Inverse ISA: pressure for a given altitude above the 101325 Pa level (same
   formula as test_baro.c's helper of the same name, duplicated here since
   this file builds without baro_alt.c, see ins.c's INS_BARO_ISA_* comment). */
static float pressure_from_altitude(float h_m)
{
    return INS_ISA_P0_PA * powf(1.0f - h_m / INS_ISA_SCALE_M, 1.0f / INS_ISA_EXP);
}

/* P(i,i) from the UDU factors: P = U*diag(d)*U', so P_ii = sum_k U[i,k]^2 d[k]
   (udu_get_diag itself is internal to ins.c). */
static float test_state_variance(const ins_t* f, int i)
{
    float p = 0.0f;
    int   k;
    for (k = 0; k < f->n; ++k)
    {
        const float u = f->U[i + k * f->n];
        p += u * u * f->d[k];
    }
    return p;
}

/* Cross the manual-init startup gate (REQ-NAV-033) with one epoch. A filter
   with allow_unlimited_deadreckoning starts on the IMU sample alone; otherwise
   pass the origin ECEF so the epoch also carries a usable GNSS fix. The
   finalize epoch neither predicts nor fuses, so state/covariance accessors
   read the pristine prescribed init afterwards. Advances *t by one dt step. */
static void start_manual_filter(ins_t* f, ins_time_us_t* t, float dt,
                                const double* origin_ecef /* NULL: IMU-only start */)
{
    *t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp        = *t;
    const float acc[3] = {0.0f, 0.0f, -9.80665f}; /* value irrelevant to manual init */
    const float gyr[3] = {0.0f, 0.0f, 0.0f};
    set_imu(&m, acc, gyr, dt);
    if (origin_ecef != (const double*)0)
    {
        m.gnss_pos.is_valid    = true;
        m.gnss_pos.xyz_ecef[0] = origin_ecef[0];
        m.gnss_pos.xyz_ecef[1] = origin_ecef[1];
        m.gnss_pos.xyz_ecef[2] = origin_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
    }
    ins_update(f, &m);
}

/* -------------------------------------------------------------------------- */

static void scenario_stationary(void)
{
    printf("\n=== Scenario 1: stationary, perfect sensors ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* "Gravity" as seen by the body (sensor reports -g along z in NED).
       With attitude identity (R_b_to_n = I), the body's +z is Down.
       The accelerometer reports specific force = -gravity in the body
       frame when stationary: f_b = -R_b_to_n^T * g_n.
       With R = I, f_b = [0, 0, -g]. */
    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f; /* 100 Hz */
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    float v[3];
    ins_get_velocity_ned(&f, v);
    float p[3];
    ins_get_position_local(&f, p);
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);

    /* Earth rotation introduces tiny drift on a "stationary" body over
     * 10 seconds at non-equatorial latitudes (this is physically correct,
     * not a bug). Tolerances reflect that. */
    CHECK_NEAR(v[0], 0.0f, 0.05, "stationary vN");
    CHECK_NEAR(v[1], 0.0f, 0.05, "stationary vE");
    CHECK_NEAR(v[2], 0.0f, 0.01, "stationary vD");
    CHECK_NEAR(p[0], 0.0f, 0.5, "stationary pN");
    CHECK_NEAR(p[1], 0.0f, 0.5, "stationary pE");
    CHECK_NEAR(p[2], 0.0f, 0.05, "stationary pD");
    /* Attitude drift due to Earth rotation: ~omega_earth * 10s * cos(lat)
     * ~ 7e-5 rad/s * 10s * 0.66 ~ 5e-4 rad. */
    CHECK_NEAR(roll, 0.0, 1e-3, "stationary roll");
    CHECK_NEAR(pitch, 0.0, 1e-3, "stationary pitch");
    CHECK_NEAR(yaw, 0.0, 1e-3, "stationary yaw");
}

static void scenario_free_fall(void)
{
    printf("\n=== Scenario 2: free fall (acc=0) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float g = g_vec[2];

    const float   acc_body[3] = {0.0f, 0.0f, 0.0f}; /* free fall */
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.005f;
    const int     N           = 400; /* 2 seconds */
    ins_time_us_t t           = 0;
    int           step;
    for (step = 1; step <= N; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    const float T           = dt * (float)N;
    const float expected_vD = g * T;
    const float expected_pD = 0.5f * g * T * T;

    float v[3];
    ins_get_velocity_ned(&f, v);
    float p[3];
    ins_get_position_local(&f, p);

    CHECK_NEAR(v[2], expected_vD, 0.05, "freefall vD");
    CHECK_NEAR(p[2], expected_pD, 0.2, "freefall pD");
    CHECK_NEAR(v[0], 0.0f, 0.02, "freefall vN");
    CHECK_NEAR(v[1], 0.0f, 0.02, "freefall vE");
}

static void scenario_yaw_rotation(void)
{
    printf("\n=== Scenario 3: constant yaw rotation ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float rate_deg    = 10.0f;
    const float rate_rad    = rate_deg * (float)M_PI / 180.0f;
    const float gyr_body[3] = {0.0f, 0.0f, rate_rad};

    const float   dt = 0.005f;
    const int     N  = 200; /* 1 second */
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= N; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    const float expected_yaw = rate_rad * dt * (float)N;
    float       roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);

    CHECK_NEAR(yaw, expected_yaw, 1e-3, "yaw rotation yaw");
    CHECK_NEAR(roll, 0.0f, 1e-3, "yaw rotation roll");
    CHECK_NEAR(pitch, 0.0f, 1e-3, "yaw rotation pitch");
}

static void scenario_covariance_grows(void)
{
    printf("\n=== Scenario 4: covariance propagation (no measurements) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    /* This scenario wants *pure* propagation; the auto-ZUPT detector is on
       by default and would otherwise fuse a synthetic zero-vel/zero-rot
       update every ~1 s once the (stationary) IMU has settled, which masks
       the process-noise-driven growth this test is checking. */
    opt.auto_zupt_disable = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* Initial position variance. */
    const float var0 = f.d[0] * 1.0f; /* d[0] is the variance of POS(0) in
                                         the initial UDU = U*d*U' with U=I */

    /* Drive predict step 1000 times (10s) with quiescent IMU. */
    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;
    int           step;
    for (step = 1; step <= 1000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* Extract P(0,0) via udu_get_variance: here it's f->d[0] only if U
       stays identity; otherwise sum of U[0,k]^2 * d[k]. */
    float p00 = 0;
    for (int k = 0; k < INS_UNKNOWNS; ++k)
    {
        const float u = f.U[0 + k * INS_UNKNOWNS];
        p00 += u * u * f.d[k];
    }

    printf("  P(0,0) initial = %g, after 10s = %g\n", var0, p00);
    if (p00 > var0) { printf("  ok    covariance grows over time\n"); }
    else
    {
        printf("  FAIL  covariance did not grow (P(0,0) = %g, initial = %g)\n", p00, var0);
        fails++;
    }

    /* Sanity: all diagonal entries still finite + positive. */
    for (int i = 0; i < INS_UNKNOWNS; ++i)
    {
        if (!isfinite(f.d[i]) || f.d[i] < 0.0f)
        {
            printf("  FAIL  d[%d] = %g (not finite or negative)\n", i, f.d[i]);
            fails++;
        }
    }
}

static void scenario_zupt(void)
{
    printf("\n=== Scenario 5: zero-velocity update fixes velocity error ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth is stationary, but the filter starts believing v = 0.5 m/s N.
       (init velocity is given in ECEF: rotate the NED error there.) */
    float R_n_to_e[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R_n_to_e);
    }
    const float verr_n = 0.5f;
    init.xdot_ecef[0]  = R_n_to_e[0] * verr_n; /* col 0 = d(ecef)/dN */
    init.xdot_ecef[1]  = R_n_to_e[1] * verr_n;
    init.xdot_ecef[2]  = R_n_to_e[2] * verr_n;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 500; ++step) /* 5 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.zero_velocity_update = true;
        ins_update(&f, &m);
    }

    float v[3];
    ins_get_velocity_ned(&f, v);
    CHECK_NEAR(v[0], 0.0f, 0.02, "zupt vN");
    CHECK_NEAR(v[1], 0.0f, 0.02, "zupt vE");
    CHECK_NEAR(v[2], 0.0f, 0.02, "zupt vD");
}

static void scenario_gnss_position(void)
{
    printf("\n=== Scenario 6: delayed GNSS position fixes position error ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* True position. The filter starts 5 m north of the truth. */
    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float R_n_to_e[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R_n_to_e);
    }
    const float perr_n = 5.0f;
    init.x_ecef[0] += R_n_to_e[0] * perr_n;
    init.x_ecef[1] += R_n_to_e[1] * perr_n;
    init.x_ecef[2] += R_n_to_e[2] * perr_n;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 20 == 0) /* 5 Hz GNSS, 100 ms old */
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0]  = 1.0f; /* diag(1,1,1) */
            m.gnss_pos.Qll_ned[4]  = 1.0f;
            m.gnss_pos.Qll_ned[8]  = 1.0f;
            m.gnss_pos.is_valid    = true;
            m.gnss_delay_ms        = 100;
        }
        ins_update(&f, &m);
    }

    /* Final position must be back at the truth (pos_local counts from the
     *wrong* start point, so it should read about -5 m north). */
    double p[3];
    ins_get_position_ecef(&f, p);
    const double dx   = p[0] - truth_ecef[0];
    const double dy   = p[1] - truth_ecef[1];
    const double dz   = p[2] - truth_ecef[2];
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    CHECK_NEAR(dist, 0.0, 0.5, "gnss pos dist to truth");

    float pl[3];
    ins_get_position_local(&f, pl);
    CHECK_NEAR(pl[0], -5.0f, 0.5, "gnss pos local N");

    /* Covariance of the position must have shrunk below its initial value. */
    float p00 = 0;
    for (int k = 0; k < INS_UNKNOWNS; ++k)
    {
        const float u = f.U[0 + k * INS_UNKNOWNS];
        p00 += u * u * f.d[k];
    }
    printf("  P(0,0) after GNSS fusion = %g\n", p00);
    if (p00 < init.pos_init_stddev_m * init.pos_init_stddev_m)
    {
        printf("  ok    position covariance shrank\n");
    }
    else
    {
        printf("  FAIL  position covariance did not shrink (%g)\n", p00);
        fails++;
    }
}

static void scenario_mag_yaw(void)
{
    printf("\n=== Scenario 7: magnetometer fixes yaw error ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth attitude is identity; the filter starts with 10 deg yaw. */
    init.rpy_init_rad[2] = 10.0f * (float)M_PI / 180.0f;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        /* Body frame == n-frame (truth attitude = identity), so the sensor
           measures the model vector plus a strong vertical disturbance
           (wrong dip angle / hard iron in body z). The fusion must ignore
           it: the tilt columns of H are zeroed, so only the horizontal
           (yaw-observable) part of the vector may correct the state. */
        m.mag.data[0]     = init.magnetic_n[0];
        m.mag.data[1]     = init.magnetic_n[1];
        m.mag.data[2]     = init.magnetic_n[2] + 15.0f;
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
        m.mag.is_valid                                            = true;
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, 0.0f, 0.01, "mag yaw converged");
    CHECK_NEAR(roll, 0.0f, 0.01, "mag roll untouched");
    CHECK_NEAR(pitch, 0.0f, 0.01, "mag pitch untouched");
}

/* Feed `steps` epochs of (stationary IMU + the given body-frame magnetic
   vector) into the filter, advancing t. Truth attitude is identity, so
   the body field equals the n-frame field. */
static void feed_mag_steps(ins_t* f, ins_time_us_t* t, int steps, const float mag_body[3],
                           float mag_var)
{
    float g_vec[3];
    ins_gravity_ned((float)f->latlonh[0], (float)f->latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;
    for (int step = 0; step < steps; ++step)
    {
        *t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = *t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]     = mag_body[0];
        m.mag.data[1]     = mag_body[1];
        m.mag.data[2]     = mag_body[2];
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = mag_var;
        m.mag.is_valid                                            = true;
        ins_update(f, &m);
    }
}

/* ins_set_magnetic_model_from_position: the WMM builds the full NED
   reference vector (so yaw is true-north) and arms the field-strength
   disturbance gate. */
static void scenario_magnetic_model_from_position(void)
{
    printf("\n=== Scenario 7b: WMM magnetic reference from position ===\n");
    const double lat  = 48.783 * M_PI / 180.0; /* Stuttgart-ish */
    const double lon  = 9.181 * M_PI / 180.0;
    const float  year = 2027.5f;

    float wmm_n[3];
    magnetic_field_ned_uT((float)(48.783), (float)(9.181), year, wmm_n);
    const float strength = magnetic_field_strength_uT(48.783f, 9.181f);

    /* --- Part A: reference vector set; yaw converges to true north. --- */
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    init.rpy_init_rad[2] = 10.0f * (float)M_PI / 180.0f; /* 10 deg yaw error */
    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    ins_set_magnetic_model_from_position(&f, lat, lon, year);
    CHECK_NEAR(f.magnetic_n[0], wmm_n[0], 1e-3, "magnetic_n north set");
    CHECK_NEAR(f.magnetic_n[1], wmm_n[1], 1e-3, "magnetic_n east set");
    CHECK_NEAR(f.magnetic_n[2], wmm_n[2], 1e-3, "magnetic_n down set");
    CHECK_NEAR(f.mag_field_expected_uT, strength, 1e-2, "expected field set");

    ins_time_us_t t = 0;
    feed_mag_steps(&f, &t, 1500, wmm_n, 0.25f); /* body==n at truth identity */
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, 0.0f, 0.01, "yaw true-north after WMM reference");

    /* --- Part B: field-strength gate downweights a magnitude anomaly. ---
       The vector-valued mag fusion already runs a chi2 residual gate, so
       the field-strength gate matters most when the magnetometer is
       trusted loosely (large R): then a magnitude anomaly slips past the
       chi2 and would bias yaw. Model that with a loose R and a
       disturbance that is rotated (+30 deg, biases yaw) and magnitude-
       anomalous (x2 -> field gate trips). */
    const float loose_var = 100.0f; /* stddev 10 uT: chi2 stays permissive */
    const float phi       = 30.0f * (float)M_PI / 180.0f;
    const float c = cosf(phi), s = sinf(phi), k = 2.0f;
    const float dist[3] = {k * (c * wmm_n[0] - s * wmm_n[1]), k * (s * wmm_n[0] + c * wmm_n[1]),
                           k * wmm_n[2]};

    ins_t f_on, f_off;
    for (int which = 0; which < 2; ++which)
    {
        ins_t*        pf = (which == 0) ? &f_on : &f_off;
        ins_options_t o;
        fill_default_opt(&o);
        if (which == 1) o.mag_field_check_disable = true;
        memset(pf, 0, sizeof(*pf));
        ins_init_t in2;
        fill_default_init(&in2, 0);
        if (ins_init(pf, &in2, &o) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_set_magnetic_model_from_position(pf, lat, lon, year);
        ins_time_us_t tt = 0;
        feed_mag_steps(pf, &tt, 1000, wmm_n, loose_var); /* settle at yaw 0 */
        feed_mag_steps(pf, &tt, 1500, dist, loose_var);  /* disturbance */
    }
    float r, p, y_on, y_off;
    ins_get_rpy(&f_on, &r, &p, &y_on);
    ins_get_rpy(&f_off, &r, &p, &y_off);
    const double e_on  = fabs((double)y_on) * 180.0 / M_PI;
    const double e_off = fabs((double)y_off) * 180.0 / M_PI;
    printf("   gate on yaw err = %.2f deg, gate off yaw err = %.2f deg\n", e_on, e_off);
    CHECK_TRUE(e_on < e_off, "field gate downweights the anomaly");
    CHECK_TRUE(e_off > 2.0 * e_on + 3.0, "un-gated yaw pulled much further");
}

/* 18-state mode (estimate_mag_bias): a constant body-frame hard-iron
   offset on the magnetometer must be estimated during a yaw rotation.
   The bias is fixed in the body frame while the reference field rotates
   with attitude, which separates it from the yaw error (REQ-NAV-029). */
static void scenario_mag_bias_estimation(void)
{
    printf("\n=== Scenario 7c: magnetometer hard-iron bias estimation ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.estimate_mag_bias = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }
    CHECK_TRUE(f.n == 18, "18-state mode active");

    /* Truth: level attitude spinning about down at 30 deg/s; constant
       body-frame hard iron on top of the rotating reference field. */
    const float wz        = 30.0f * (float)M_PI / 180.0f;
    const float b_true[3] = {3.0f, -2.0f, 1.5f};

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* z-spin: constant */
    const float gyr_body[3] = {0.0f, 0.0f, wz};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 6000; ++step) /* 60 s, 5 full turns */
    {
        t += us_from_sec(dt);
        const float        psi = wz * (float)step * dt; /* truth yaw */
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        /* mag_b = R_true' * m_n + b_true (R_true = yaw rotation psi). */
        const float c = cosf(psi), s = sinf(psi);
        m.mag.data[0]     = c * init.magnetic_n[0] + s * init.magnetic_n[1] + b_true[0];
        m.mag.data[1]     = -s * init.magnetic_n[0] + c * init.magnetic_n[1] + b_true[1];
        m.mag.data[2]     = init.magnetic_n[2] + b_true[2];
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
        m.mag.is_valid                                            = true;
        ins_update(&f, &m);
    }

    float b_est[3];
    CHECK_TRUE(ins_get_bias_mag(&f, b_est), "mag bias accessor valid");
    CHECK_NEAR(b_est[0], b_true[0], 0.5, "hard iron x [uT]");
    CHECK_NEAR(b_est[1], b_true[1], 0.5, "hard iron y [uT]");
    CHECK_NEAR(b_est[2], b_true[2], 0.5, "hard iron z [uT]");

    /* Yaw must track truth despite the (initially unknown) hard iron. */
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    const float psi_end = wz * 6000.0f * dt;
    const float yaw_err = ins_angle_diff(yaw, psi_end);
    CHECK_NEAR(yaw_err, 0.0f, 2.0 * M_PI / 180.0, "yaw error [rad]");

    /* Default 15-state instance: option off, accessor must refuse. */
    ins_t f15;
    memset(&f15, 0, sizeof(f15));
    ins_options_t opt15;
    fill_default_opt(&opt15);
    ins_init_t init15;
    fill_default_init(&init15, 0);
    if (ins_init(&f15, &init15, &opt15) != 0)
    {
        printf("init15 failed\n");
        fails++;
        return;
    }
    CHECK_TRUE(f15.n == 15, "default stays 15-state");
    CHECK_TRUE(!ins_get_bias_mag(&f15, b_est), "accessor false in 15-state mode");
}

static void scenario_zero_rotation_bias(void)
{
    printf("\n=== Scenario 8: zero-rotation update estimates gyro bias ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};

    /* True gyro bias (within the initial bias stddev). The body is truly
       not rotating relative to the n-frame. The true gyro reading would
       also contain the Earth rate, which the fusion accounts for: to
       keep the test simple, inject only the bias. */
    const float bias_true   = 1e-4f;
    const float gyr_body[3] = {bias_true, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 500; ++step) /* 5 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.zero_rotation_update = true;
        ins_update(&f, &m);
    }

    float b[3];
    ins_get_bias_gyr(&f, b);
    /* The injected reading lacks the true Earth-rate term, so the
       estimate absorbs it: tolerance covers |omega_earth| ~ 7.3e-5. */
    CHECK_NEAR(b[0], bias_true, 1e-4, "zero-rot bias x");
    CHECK_NEAR(b[1], 0.0f, 1e-4, "zero-rot bias y");
    CHECK_NEAR(b[2], 0.0f, 1e-4, "zero-rot bias z");
}

#define S9_STEPS      1000 /* 10 s at 100 Hz */
#define S9_DELAY_STEP 30   /* 300 ms GNSS latency in steps */

static void scenario_gnss_moving_delayed(void)
{
    printf("\n=== Scenario 9: moving vehicle, 300 ms delayed GNSS ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    /* This scenario feeds no GNSS velocity, only delayed position, so the
       auto-ZUPT/ZARU velocity gate never applies (REQ-NAV-013: no state-
       estimate fallback) and only the IMU criteria would decide. A
       noiseless synthetic constant-velocity trajectory has zero sample
       variance and a gravity-only specific force, indistinguishable from
       standstill on the IMU alone, so the detector would falsely zero the
       very velocity this scenario tests -- opt out, exactly the case
       config.yaml's imu.auto_zupt_velocity_blind_disable exists for on
       the ARS/AHRS side. */
    opt.auto_zupt_disable = true;

    /* Truth: constant 10 m/s north, attitude identity. */
    const float vn_truth[3] = {10.0f, 0.0f, 0.0f};
    double      truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));

    /* Filter init: correct velocity, but 5 m east position error. */
    float R0[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R0);
    }
    init.xdot_ecef[0]  = R0[0] * vn_truth[0]; /* col 0 = d(ecef)/dN */
    init.xdot_ecef[1]  = R0[1] * vn_truth[0];
    init.xdot_ecef[2]  = R0[2] * vn_truth[0];
    const float perr_e = 5.0f;
    init.x_ecef[0] += R0[3] * perr_e; /* col 1 = d(ecef)/dE */
    init.x_ecef[1] += R0[4] * perr_e;
    init.x_ecef[2] += R0[5] * perr_e;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* Truth position history (for generating the delayed measurements). */
    static double truth_hist[S9_STEPS + 1][3];
    memcpy(truth_hist[0], truth_ecef, sizeof(truth_hist[0]));

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= S9_STEPS; ++step)
    {
        /* ---- Propagate truth and generate perfect IMU readings ---- */
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        float R_n_to_e[9];
        ins_rotmat_n_to_e(lat, lon, R_n_to_e);

        float g_vec[3];
        ins_gravity_ned((float)lat, (float)h, g_vec);
        float w_in[3], w_ie[3], w_en[3];
        ins_calc_omega_n_in(lat, h, vn_truth, w_in, w_ie, w_en);

        /* Constant NED velocity requires R*f_b = -(g + f_coriolis),
           i.e. f_b = -g + (2*w_ie + w_en) x v (attitude identity). */
        const float two_wie_plus_wen[3] = {2.0f * w_ie[0] + w_en[0], 2.0f * w_ie[1] + w_en[1],
                                           2.0f * w_ie[2] + w_en[2]};
        float       cross_v[3];
        ins_cross(two_wie_plus_wen, vn_truth, cross_v);
        const float acc_body[3] = {-g_vec[0] + cross_v[0], -g_vec[1] + cross_v[1],
                                   -g_vec[2] + cross_v[2]};
        /* Body stays aligned with the (rotating) n-frame: the gyro sees
           the full n-frame rotation rate. */
        const float gyr_body[3] = {w_in[0], w_in[1], w_in[2]};

        int i;
        for (i = 0; i < 3; ++i)
        {
            truth_ecef[i] += (double)(R_n_to_e[i + 0] * vn_truth[0] * dt) +
                             (double)(R_n_to_e[i + 3] * vn_truth[1] * dt) +
                             (double)(R_n_to_e[i + 6] * vn_truth[2] * dt);
        }
        memcpy(truth_hist[step], truth_ecef, sizeof(truth_hist[0]));

        /* ---- Feed the filter ---- */
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 20 == 0 && step > S9_DELAY_STEP) /* 5 Hz GNSS */
        {
            /* Measurement is 300 ms old: truth at (t - 300 ms). */
            const double* p_tov    = truth_hist[step - S9_DELAY_STEP];
            m.gnss_pos.xyz_ecef[0] = p_tov[0];
            m.gnss_pos.xyz_ecef[1] = p_tov[1];
            m.gnss_pos.xyz_ecef[2] = p_tov[2];
            m.gnss_pos.Qll_ned[0]  = 1.0f; /* diag(1,1,1) */
            m.gnss_pos.Qll_ned[4]  = 1.0f;
            m.gnss_pos.Qll_ned[8]  = 1.0f;
            m.gnss_pos.is_valid    = true;
            m.gnss_delay_ms        = 300;
        }
        ins_update(&f, &m);
    }

    /* A naive fusion (residual against the *current* state) would trail
       the truth by v * delay = 3 m. The 0.5 m tolerance catches that. */
    double p[3];
    ins_get_position_ecef(&f, p);
    const double dx = p[0] - truth_ecef[0];
    const double dy = p[1] - truth_ecef[1];
    const double dz = p[2] - truth_ecef[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 0.0, 0.5, "moving delayed-gnss dist to truth");

    float v[3];
    ins_get_velocity_ned(&f, v);
    CHECK_NEAR(v[0], vn_truth[0], 0.1, "moving delayed-gnss vN");
    CHECK_NEAR(v[1], 0.0f, 0.1, "moving delayed-gnss vE");
    CHECK_NEAR(v[2], 0.0f, 0.1, "moving delayed-gnss vD");
}

static void scenario_gnss_velocity_leverarm(void)
{
    printf("\n=== Scenario 10: GNSS velocity fusion + lever arm ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth: stationary, attitude identity. GNSS antenna 1 m ahead of the
       body origin (leverarm_b = [1,0,0] -> 1 m north in the n-frame). */
    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float R0[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R0);
    }

    /* Filter init: 3 m north position error, 0.5 m/s east velocity error. */
    const float perr_n = 3.0f;
    init.x_ecef[0] += R0[0] * perr_n;
    init.x_ecef[1] += R0[1] * perr_n;
    init.x_ecef[2] += R0[2] * perr_n;
    const float verr_e = 0.5f;
    init.xdot_ecef[0]  = R0[3] * verr_e;
    init.xdot_ecef[1]  = R0[4] * verr_e;
    init.xdot_ecef[2]  = R0[5] * verr_e;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* Antenna position: truth + R_n_to_e * la_n, la_n = [1,0,0]. */
    double antenna_ecef[3];
    antenna_ecef[0] = truth_ecef[0] + (double)R0[0];
    antenna_ecef[1] = truth_ecef[1] + (double)R0[1];
    antenna_ecef[2] = truth_ecef[2] + (double)R0[2];

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 20 == 0) /* 5 Hz GNSS pos + vel, no delay */
        {
            m.gnss_pos.xyz_ecef[0] = antenna_ecef[0];
            m.gnss_pos.xyz_ecef[1] = antenna_ecef[1];
            m.gnss_pos.xyz_ecef[2] = antenna_ecef[2];
            /* Full covariance: 1 m^2 variances with a 0.5 N-E
               correlation -> exercises the decorrelate() path. */
            m.gnss_pos.Qll_ned[0] = 1.0f;
            m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8] = 1.0f;
            m.gnss_pos.Qll_ned[1] = 0.5f; /* cov(N,E) */
            m.gnss_pos.Qll_ned[3] = 0.5f;
            m.gnss_pos.is_valid   = true;

            m.gnss_vel.vel_ned[0] = 0.0f; /* antenna truly stationary */
            m.gnss_vel.vel_ned[1] = 0.0f;
            m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = 0.01f; /* diag(0.01) */
            m.gnss_vel.Qll_ned[4] = 0.01f;
            m.gnss_vel.Qll_ned[8] = 0.01f;
            m.gnss_vel.is_valid   = true;

            /* Pos/vel cross-covariance (corr. 0.5 per axis:
               0.5 * sqrt(1 * 0.01) = 0.05) -> full 6x6 R. */
            m.gnss_Qll_pos_vel_ned[0] = 0.05f;
            m.gnss_Qll_pos_vel_ned[4] = 0.05f;
            m.gnss_Qll_pos_vel_ned[8] = 0.05f;

            m.gnss_leverarm_b[0] = 1.0f;
            m.gnss_leverarm_b[1] = 0.0f;
            m.gnss_leverarm_b[2] = 0.0f;
        }
        ins_update(&f, &m);
    }

    /* The *body* position must converge to the truth: without lever-arm
       compensation it would settle 1 m north (at the antenna). */
    double p[3];
    ins_get_position_ecef(&f, p);
    const double dx = p[0] - truth_ecef[0];
    const double dy = p[1] - truth_ecef[1];
    const double dz = p[2] - truth_ecef[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 0.0, 0.3, "leverarm body dist to truth");

    float v[3];
    ins_get_velocity_ned(&f, v);
    CHECK_NEAR(v[0], 0.0f, 0.05, "gnss-vel vN");
    CHECK_NEAR(v[1], 0.0f, 0.05, "gnss-vel vE");
    CHECK_NEAR(v[2], 0.0f, 0.05, "gnss-vel vD");
}

/* Lever-arm attitude coupling in the position Jacobian (Wendel 8.62,
   REQ-NAV-024). The GNSS antenna sits 2 m forward and the filter starts
   with an 8 deg yaw error. A separate zero-lever-arm local-position fix
   pins the body origin directly (breaking the static pos/yaw degeneracy),
   so the GNSS antenna fix's east residual (~L*yaw) can only be explained
   by yaw. That correction exists only because H_pos includes -[l^n]_x;
   with the pre-fix H (identity block only) the residual would be absorbed
   by a body-position wiggle and the yaw error would persist. */
static void scenario_gnss_leverarm_yaw_from_position(void)
{
    printf("\n=== Scenario: GNSS lever-arm yaw observability (position) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float R0[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R0);
    }

    /* Truth: yaw = 0, level, stationary at the origin. Filter: +8 deg yaw. */
    const float yaw_err  = 8.0f * (float)M_PI / 180.0f;
    init.rpy_init_rad[2] = yaw_err;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* Antenna 2 m forward; at truth yaw = 0 that is l^n = [2, 0, 0]. */
    const float L = 2.0f;
    double      antenna_ecef[3];
    antenna_ecef[0] = truth_ecef[0] + (double)R0[0] * L;
    antenna_ecef[1] = truth_ecef[1] + (double)R0[1] * L;
    antenna_ecef[2] = truth_ecef[2] + (double)R0[2] * L;

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 3000; ++step) /* 30 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 20 == 0) /* 5 Hz GNSS antenna position (2 m lever arm) */
        {
            m.gnss_pos.xyz_ecef[0] = antenna_ecef[0];
            m.gnss_pos.xyz_ecef[1] = antenna_ecef[1];
            m.gnss_pos.xyz_ecef[2] = antenna_ecef[2];
            m.gnss_pos.Qll_ned[0]  = 0.04f;
            m.gnss_pos.Qll_ned[4]  = 0.04f;
            m.gnss_pos.Qll_ned[8]  = 0.09f;
            m.gnss_pos.is_valid    = true;
            m.gnss_leverarm_b[0]   = L;

            /* Direct body-origin anchor (no lever arm) to pin position and
               leave yaw as the only explanation for the GNSS residual. */
            m.local_pos.pos_ned[0] = 0.0f;
            m.local_pos.pos_ned[1] = 0.0f;
            m.local_pos.pos_ned[2] = 0.0f;
            m.local_pos.Qll_ned[0] = 1e-4f;
            m.local_pos.Qll_ned[4] = 1e-4f;
            m.local_pos.Qll_ned[8] = 1e-4f;
            m.local_pos.is_valid   = true;
        }
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, 0.0f, 1.0f * (float)M_PI / 180.0f, "lever-arm yaw from position [rad]");

    /* Body position must stay at the truth (not run off to the antenna). */
    double p[3];
    ins_get_position_ecef(&f, p);
    const double dx = p[0] - truth_ecef[0];
    const double dy = p[1] - truth_ecef[1];
    const double dz = p[2] - truth_ecef[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 0.0, 0.1, "lever-arm body pos to truth [m]");
}

/* Lever-arm attitude coupling in the velocity Jacobian (Wendel 8.74,
   REQ-NAV-024). The body spins in place about the down axis at 20 deg/s
   with a 2 m forward antenna, so the antenna has a real ~0.7 m/s NED
   velocity that rotates with heading. Body velocity is pinned to zero by a
   ZUPT every epoch, so the GNSS antenna-velocity residual caused by an
   8 deg yaw error can only be explained by yaw, and only if H_vel carries
   the -[R*(omega x l^b)]_x block. */
static void scenario_gnss_leverarm_yaw_from_velocity(void)
{
    printf("\n=== Scenario: GNSS lever-arm yaw observability (velocity) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    const float yaw_err  = 8.0f * (float)M_PI / 180.0f;
    init.rpy_init_rad[2] = yaw_err; /* filter starts 8 deg off the truth */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};

    const float L           = 2.0f;                         /* antenna 2 m fwd */
    const float rate        = 20.0f * (float)M_PI / 180.0f; /* yaw rate [rad/s] */
    const float gyr_body[3] = {0.0f, 0.0f, rate};
    /* Lever-arm velocity in body frame: omega x l^b = [0, rate*L, 0]. */
    const float v_la_b1 = rate * L;

    const float   dt       = 0.01f;
    ins_time_us_t t        = 0;
    float         yaw_true = 0.0f;
    int           step;
    for (step = 1; step <= 2000; ++step) /* 20 s */
    {
        t += us_from_sec(dt);
        yaw_true += rate * dt; /* truth heading integrates the yaw rate */

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.zero_velocity_update = true; /* body origin is stationary */

        if (step % 5 == 0) /* 20 Hz GNSS velocity */
        {
            /* True antenna velocity in NED = R(yaw_true) * [0, rate*L, 0]. */
            const float c = cosf(yaw_true), s = sinf(yaw_true);
            m.gnss_vel.vel_ned[0] = -s * v_la_b1;
            m.gnss_vel.vel_ned[1] = c * v_la_b1;
            m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = 0.01f;
            m.gnss_vel.Qll_ned[4] = 0.01f;
            m.gnss_vel.Qll_ned[8] = 0.01f;
            m.gnss_vel.is_valid   = true;
            m.gnss_leverarm_b[0]  = L;
        }
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    /* Compare heading modulo 2*pi (yaw_true has wrapped several times). */
    float dyaw = yaw - yaw_true;
    while (dyaw > (float)M_PI) dyaw -= 2.0f * (float)M_PI;
    while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
    CHECK_NEAR(dyaw, 0.0f, 2.0f * (float)M_PI / 180.0f, "lever-arm yaw error from velocity [rad]");
}

static void scenario_leveling_at_yaw90(void)
{
    printf("\n=== Scenario 11: ZUPT leveling with roll error at yaw=90 ===\n");
    /* Regression test for the attitude-error frame convention:
       the Kalman filter estimates the misalignment in the n-frame
       (psi-angle model, see ins_compute_Phi). If the correction were
       applied about the *body* axes instead, an n-frame roll error at
       yaw=90deg would be corrected about the wrong axis: the error
       precesses into pitch and grows instead of shrinking. */
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth: stationary, yaw = 90 deg, level. Filter starts with a
       5 deg roll error. */
    const float yaw90    = 0.5f * (float)M_PI;
    init.rpy_init_rad[2] = yaw90;
    init.rpy_init_rad[0] = 5.0f * (float)M_PI / 180.0f;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* True attitude: R_true = Rz(90 deg). Perfect IMU readings:
       f_b = -R_true' * g_n,  gyr = R_true' * omega_n_in (Earth rate),
       so that omega_b_nb = 0 for the true attitude. */
    float q_true[4], R_true[9];
    ins_quat_from_rpy(0.0f, 0.0f, yaw90, q_true);
    ins_quat_to_rotmat(q_true, R_true);

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float v_zero[3] = {0.0f, 0.0f, 0.0f};
    float       w_in[3];
    ins_calc_omega_n_in(f.latlonh[0], f.latlonh[2], v_zero, w_in, (float*)0, (float*)0);

    float acc_body[3], gyr_body[3];
    /* R' * v: manual transpose multiply (column-major R). */
    int i;
    for (i = 0; i < 3; ++i)
    {
        acc_body[i] = -(R_true[i * 3 + 0] * g_vec[0] + R_true[i * 3 + 1] * g_vec[1] +
                        R_true[i * 3 + 2] * g_vec[2]);
        gyr_body[i] = (R_true[i * 3 + 0] * w_in[0] + R_true[i * 3 + 1] * w_in[1] +
                       R_true[i * 3 + 2] * w_in[2]);
    }

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 3000; ++step) /* 30 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.zero_velocity_update = true;
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, 0.0f, 0.005, "yaw90 leveling roll");
    CHECK_NEAR(pitch, 0.0f, 0.005, "yaw90 leveling pitch");
    CHECK_NEAR(yaw, yaw90, 0.02, "yaw90 leveling yaw");
}

static void scenario_local_pos_lighthouse(void)
{
    printf("\n=== Scenario 12: local NED position aiding (lighthouse) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth: stationary, level. The filter's init position is 2 m north
       of the truth, so in the filter's local frame the true body sits at
       (-2, 0, 0). The tracking sensor is mounted 0.1 m ahead of the body
       origin (lever arm), so it truly sits at (-1.9, 0, 0). */
    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float R0[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R0);
    }
    const float perr_n = 2.0f;
    init.x_ecef[0] += R0[0] * perr_n;
    init.x_ecef[1] += R0[1] * perr_n;
    init.x_ecef[2] += R0[2] * perr_n;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 500; ++step) /* 5 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 2 == 0 && step > 2) /* 50 Hz, 20 ms latency */
        {
            m.local_pos.pos_ned[0]    = -perr_n + 0.1f; /* sensor truth pos */
            m.local_pos.pos_ned[1]    = 0.0f;
            m.local_pos.pos_ned[2]    = 0.0f;
            m.local_pos.Qll_ned[0]    = 1.6e-3f; /* (4 cm)^2 diagonal */
            m.local_pos.Qll_ned[4]    = 1.6e-3f;
            m.local_pos.Qll_ned[8]    = 1.6e-3f;
            m.local_pos.is_valid      = true;
            m.local_pos_leverarm_b[0] = 0.1f;
            m.local_pos_delay_ms      = 20;
        }
        ins_update(&f, &m);
    }

    /* Body position must converge to (-2, 0, 0) in the local frame:
       the lever arm must be compensated (otherwise -1.9). */
    float p[3];
    ins_get_position_local(&f, p);
    CHECK_NEAR(p[0], -perr_n, 0.05, "lighthouse pos N");
    CHECK_NEAR(p[1], 0.0f, 0.05, "lighthouse pos E");
    CHECK_NEAR(p[2], 0.0f, 0.05, "lighthouse pos D");

    float v[3];
    ins_get_velocity_ned(&f, v);
    CHECK_NEAR(v[0], 0.0f, 0.05, "lighthouse vN");
    CHECK_NEAR(v[1], 0.0f, 0.05, "lighthouse vE");
    CHECK_NEAR(v[2], 0.0f, 0.05, "lighthouse vD");

    /* The absolute position accessor must reflect the correction, too:
       the ECEF output has to be back at the truth. */
    double pe[3];
    ins_get_position_ecef(&f, pe);
    const double dx = pe[0] - truth_ecef[0];
    const double dy = pe[1] - truth_ecef[1];
    const double dz = pe[2] - truth_ecef[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 0.0, 0.1, "lighthouse ecef dist to truth");
}

/* Lever-arm attitude coupling for lighthouse-style local NED position
   aiding (Wendel 8.62 via the local-pos path, REQ-NAV-024). A VR-style
   tracker sits 0.5 m ahead of the body origin (eccentric mount) and the
   body spins in place about the down axis. The body origin is stationary
   (ZUPT + correct init position), so the tracker traces a circle. A
   constant 8 deg yaw error rotates the whole predicted circle. A fixed
   body-position offset cannot match a *rotating* residual, so yaw becomes
   observable. This works only because the local-pos Jacobian carries the
   -[l^n]_x block; with the pre-fix H the yaw error would persist. */
static void scenario_local_pos_lighthouse_yaw(void)
{
    printf("\n=== Scenario: lighthouse lever-arm yaw observability ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth: body origin at the init position (no position error), level,
       spinning about the down axis. Filter starts with +8 deg yaw error. */
    const float yaw_err  = 8.0f * (float)M_PI / 180.0f;
    init.rpy_init_rad[2] = yaw_err;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};

    const float L           = 0.5f;                         /* eccentric tracker */
    const float rate        = 30.0f * (float)M_PI / 180.0f; /* yaw rate [rad/s] */
    const float gyr_body[3] = {0.0f, 0.0f, rate};

    const float   dt       = 0.01f;
    ins_time_us_t t        = 0;
    float         yaw_true = 0.0f;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s -> ~2.5 rotations */
    {
        t += us_from_sec(dt);
        yaw_true += rate * dt;

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.zero_velocity_update = true; /* body origin is stationary */

        if (step % 2 == 0) /* 50 Hz tracker */
        {
            /* True tracker position (local NED) = R(yaw_true) * [L,0,0]. */
            m.local_pos.pos_ned[0]    = L * cosf(yaw_true);
            m.local_pos.pos_ned[1]    = L * sinf(yaw_true);
            m.local_pos.pos_ned[2]    = 0.0f;
            m.local_pos.Qll_ned[0]    = 1e-4f; /* (1 cm)^2 */
            m.local_pos.Qll_ned[4]    = 1e-4f;
            m.local_pos.Qll_ned[8]    = 1e-4f;
            m.local_pos.is_valid      = true;
            m.local_pos_leverarm_b[0] = L;
        }
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    float dyaw = yaw - yaw_true;
    while (dyaw > (float)M_PI) dyaw -= 2.0f * (float)M_PI;
    while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
    CHECK_NEAR(dyaw, 0.0f, 2.0f * (float)M_PI / 180.0f, "lighthouse lever-arm yaw error [rad]");

    /* Body origin must stay put (not drift to absorb the residual). */
    float p[3];
    ins_get_position_local(&f, p);
    CHECK_NEAR(test_vec3_norm(p), 0.0f, 0.05, "lighthouse body pos [m]");
}

static void scenario_yaw_aiding(void)
{
    printf("\n=== Scenario 13: absolute yaw aiding (pose heading) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Truth: stationary, yaw = 30 deg. Filter starts with 20 deg error. */
    const float yaw_true = 30.0f * (float)M_PI / 180.0f;
    init.rpy_init_rad[2] = yaw_true + 20.0f * (float)M_PI / 180.0f;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* Perfect IMU for the true attitude R_true = Rz(30 deg). */
    float q_true[4], R_true[9];
    ins_quat_from_rpy(0.0f, 0.0f, yaw_true, q_true);
    ins_quat_to_rotmat(q_true, R_true);

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    float acc_body[3];
    int   i;
    for (i = 0; i < 3; ++i)
    {
        acc_body[i] = -(R_true[i * 3 + 0] * g_vec[0] + R_true[i * 3 + 1] * g_vec[1] +
                        R_true[i * 3 + 2] * g_vec[2]);
    }
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 500; ++step) /* 5 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 2 == 0 && step > 2) /* 50 Hz, 20 ms latency */
        {
            m.yaw.yaw_rad    = yaw_true;
            m.yaw.stddev_rad = 1.0f * (float)M_PI / 180.0f;
            m.yaw.is_valid   = true;
            m.yaw_delay_ms   = 20;
        }
        ins_update(&f, &m);
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, yaw_true, 0.01, "yaw aiding yaw");
    CHECK_NEAR(roll, 0.0f, 0.01, "yaw aiding roll");
    CHECK_NEAR(pitch, 0.0f, 0.01, "yaw aiding pitch");
}

/* ins_fuse_yaw()'s own guard rails (MC/DC): a non-positive stddev, a delay
 * past INS_MAX_DELAY_MS, and the negative-pitch arm of the gimbal-lock skip
 * (scenario_yaw_aiding only ever runs level) must each drop the sample
 * without moving the state. */
static void scenario_yaw_aiding_guards(void)
{
    printf("\n=== Scenario: absolute yaw aiding guard rails (MC/DC) ===\n");

    const float yaw_true   = 30.0f * (float)M_PI / 180.0f;
    const float yaw_wrong  = -60.0f * (float)M_PI / 180.0f; /* far from yaw_true */
    const float gyr0[3]    = {0.0f, 0.0f, 0.0f};
    const float acc_lvl[3] = {0.0f, 0.0f, -INS_GRAVITY_NOMINAL};

    /* Case 1: stddev_rad <= 0.0f must be dropped. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.rpy_init_rad[2] = yaw_true;
        ins_options_t opt;
        fill_default_opt(&opt);
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (stddev guard)");
        ins_time_us_t      t = 0;
        ins_measurements_t m;
        int                i;
        for (i = 0; i < 10; ++i)
        {
            t += us_from_sec(0.01f);
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_lvl, gyr0, 0.01f);
            m.yaw.is_valid   = true;
            m.yaw.yaw_rad    = yaw_wrong;
            m.yaw.stddev_rad = 0.0f; /* <= 0: must not fuse */
            ins_update(&f, &m);
        }
        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, yaw_true, 0.02, "stddev <= 0: yaw untouched");
    }

    /* Case 2: yaw_delay_ms > INS_MAX_DELAY_MS must be dropped. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.rpy_init_rad[2] = yaw_true;
        ins_options_t opt;
        fill_default_opt(&opt);
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (delay guard)");
        ins_time_us_t      t = 0;
        ins_measurements_t m;
        int                i;
        for (i = 0; i < 10; ++i)
        {
            t += us_from_sec(0.01f);
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_lvl, gyr0, 0.01f);
            m.yaw.is_valid   = true;
            m.yaw.yaw_rad    = yaw_wrong;
            m.yaw.stddev_rad = DEG2RAD(1.0f);
            m.yaw_delay_ms   = INS_MAX_DELAY_MS + 1;
            ins_update(&f, &m);
        }
        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, yaw_true, 0.02, "delay > INS_MAX_DELAY_MS: yaw untouched");
    }
}

/* Heading input contract (REQ-NAV-060): any single-turn convention is
 * accepted and normalized, anything beyond one turn is a unit/unwrapping
 * error and is dropped rather than wrapped into a wrong heading. */
static void scenario_yaw_input_range(void)
{
    printf("\n=== Scenario: heading input range (REQ-NAV-060) ===\n");

    /* Truth: stationary, yaw = -30 deg. Filter starts with 20 deg error.
       The same heading is published once as 330 deg in the [0, 2pi)
       convention and once as a raw degree value in the radian field. */
    const float yaw_true    = -30.0f * (float)M_PI / 180.0f;
    const float yaw_two_pi  = yaw_true + 2.0f * (float)M_PI; /* 330 deg */
    const float yaw_degrees = -30.0f;                        /* deg in a rad field */
    const float dt          = 0.01f;

    int variant;
    for (variant = 0; variant < 2; ++variant)
    {
        const bool  in_range = (variant == 0);
        const float yaw_meas = in_range ? yaw_two_pi : yaw_degrees;

        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        init.rpy_init_rad[2] = yaw_true + 20.0f * (float)M_PI / 180.0f;

        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float q_true[4], R_true[9];
        ins_quat_from_rpy(0.0f, 0.0f, yaw_true, q_true);
        ins_quat_to_rotmat(q_true, R_true);

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        float acc_body[3];
        int   i;
        for (i = 0; i < 3; ++i)
        {
            acc_body[i] = -(R_true[i * 3 + 0] * g_vec[0] + R_true[i * 3 + 1] * g_vec[1] +
                            R_true[i * 3 + 2] * g_vec[2]);
        }
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        ins_time_us_t t         = 0;
        uint32_t      n_yaw_fed = 0;
        int           step;
        for (step = 1; step <= 500; ++step) /* 5 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);

            if (step % 2 == 0) /* 50 Hz heading */
            {
                m.yaw.yaw_rad    = yaw_meas;
                m.yaw.stddev_rad = 1.0f * (float)M_PI / 180.0f;
                m.yaw.is_valid   = true;
                ++n_yaw_fed;
            }
            ins_update(&f, &m);
        }

        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        const ins_diag_t* diag = ins_get_diag(&f);

        if (in_range)
        {
            CHECK_NEAR(yaw, yaw_true, 0.01, "[0,2pi) heading converges to truth");
            CHECK_TRUE(diag->n_invalid_input == 0, "[0,2pi) heading not counted as invalid");
        }
        else
        {
            /* Dropped, not wrapped: the estimate stays where it was
               instead of being pulled towards fmod(-30 rad, 2pi). */
            CHECK_NEAR(yaw, init.rpy_init_rad[2], 0.01, "degree-valued heading leaves yaw alone");
            CHECK_TRUE(diag->n_invalid_input == n_yaw_fed, "every out-of-range heading counted");
            CHECK_TRUE(diag->n_downweighted == 0, "out-of-range heading never reaches the gate");
        }
    }

    /* Attitude hint: an out-of-range yaw costs the hint its heading only,
       roll/pitch and the gyro bias still apply. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        float g_vec[3];
        ins_gravity_ned((float)lat, (float)h, g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float roll_hint  = 25.0f * (float)M_PI / 180.0f;
        const float pitch_hint = -15.0f * (float)M_PI / 180.0f;

        ins_time_us_t t = 0;
        int           step;
        for (step = 1; step <= 20; ++step)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.xyz_ecef[0]      = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1]      = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2]      = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0]       = 1.0f;
        m.gnss_pos.Qll_ned[4]       = 1.0f;
        m.gnss_pos.Qll_ned[8]       = 1.0f;
        m.gnss_pos.is_valid         = true;
        m.att_hint.is_valid         = true;
        m.att_hint.roll_rad         = roll_hint;
        m.att_hint.pitch_rad        = pitch_hint;
        m.att_hint.stddev_roll_rad  = 0.05f;
        m.att_hint.stddev_pitch_rad = 0.06f;
        m.att_hint.yaw_rad          = 40.0f; /* deg in a rad field */
        m.att_hint.stddev_yaw_rad   = 0.1f;
        ins_update(&f, &m);

        if (!f.is_initialized)
        {
            printf("  FAIL  filter did not bootstrap on the hinted fix\n");
            fails++;
            return;
        }

        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(roll, roll_hint, 1e-4, "hint roll survives an out-of-range hint yaw");
        CHECK_NEAR(pitch, pitch_hint, 1e-4, "hint pitch survives an out-of-range hint yaw");
        CHECK_NEAR(yaw, 0.0f, 1e-4, "out-of-range hint yaw not used for the bootstrap");
        /* Yaw prior fell back to "unknown" (~180 deg) instead of keeping
           the hint's 0.1 rad. */
        CHECK_TRUE(f.d[INS_IDX_RPY + 2] > 1.0f, "hint yaw variance falls back to unknown");
        CHECK_TRUE(ins_get_diag(&f)->n_invalid_input == 1, "out-of-range hint yaw counted once");
    }
}

/* -------------------------------------------------------------------------- */
/* Auto-init scenarios                                                       */
/* -------------------------------------------------------------------------- */

static void scenario_autoinit_gnss(void)
{
    printf("\n=== Scenario 14: auto-init collects IMU, bootstraps from GNSS "
           "===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    if (!f.is_collecting || f.is_initialized)
    {
        printf("  FAIL  filter must start collecting, not initialized\n");
        fails++;
    }
    else { printf("  ok    filter starts collecting, not initialized\n"); }

    /* Truth: stationary, tilted (roll=15 deg, pitch=-8 deg). No heading
       source is offered, so yaw must come out "unknown". */
    const float roll_true  = 15.0f * (float)M_PI / 180.0f;
    const float pitch_true = -8.0f * (float)M_PI / 180.0f;
    float       q_true[4], R_true[9];
    ins_quat_from_rpy(roll_true, pitch_true, 0.0f, q_true);
    ins_quat_to_rotmat(q_true, R_true);

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    float acc_body[3];
    int   i;
    for (i = 0; i < 3; ++i)
    {
        acc_body[i] = -(R_true[i * 3 + 0] * g_vec[0] + R_true[i * 3 + 1] * g_vec[1] +
                        R_true[i * 3 + 2] * g_vec[2]);
    }
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    /* IMU-only epochs: no fix yet, must keep collecting. */
    for (step = 1; step <= 20; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    if (f.is_initialized || !f.is_collecting)
    {
        printf("  FAIL  filter initialized without ever seeing a fix\n");
        fails++;
    }
    else { printf("  ok    still collecting after IMU-only epochs\n"); }
    double dummy_p[3];
    if (ins_get_position_ecef(&f, dummy_p) || ins_is_ready(&f))
    {
        printf("  FAIL  accessors must refuse while collecting\n");
        fails++;
    }
    else { printf("  ok    accessors correctly refuse while collecting\n"); }

    /* First usable fix: GNSS position + velocity, stationary at the truth
       (== the provisional anchor, so the filter's position error is 0). */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    m.gnss_vel.vel_ned[0]  = 0.0f;
    m.gnss_vel.vel_ned[1]  = 0.0f;
    m.gnss_vel.vel_ned[2]  = 0.0f;
    m.gnss_vel.Qll_ned[0]  = 0.01f;
    m.gnss_vel.Qll_ned[4]  = 0.01f;
    m.gnss_vel.Qll_ned[8]  = 0.01f;
    m.gnss_vel.is_valid    = true;
    ins_update(&f, &m);

    if (!f.is_initialized || f.is_collecting)
    {
        printf("  FAIL  filter did not bootstrap on the first fix\n");
        fails++;
        return;
    }
    printf("  ok    filter bootstrapped on the first GNSS fix\n");

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, roll_true, 0.02, "autoinit leveled roll");
    CHECK_NEAR(pitch, pitch_true, 0.02, "autoinit leveled pitch");
    CHECK_NEAR(yaw, 0.0f, 1e-6, "autoinit yaw placeholder (unknown)");

    /* No heading source was offered -> yaw variance must be ~ (180 deg)^2. */
    const float yaw_var = f.d[INS_IDX_RPY + 2];
    CHECK_NEAR(yaw_var, (float)(M_PI * M_PI), 0.5, "autoinit yaw variance is 'unknown'");

    double p[3];
    ins_get_position_ecef(&f, p);
    const double dx = p[0] - init.x_ecef[0];
    const double dy = p[1] - init.x_ecef[1];
    const double dz = p[2] - init.x_ecef[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 0.0, 0.01, "autoinit position from GNSS fix");

    float v[3];
    ins_get_velocity_ned(&f, v);
    CHECK_NEAR(v[0], 0.0f, 1e-4, "autoinit velocity N");
    CHECK_NEAR(v[1], 0.0f, 1e-4, "autoinit velocity E");
    CHECK_NEAR(v[2], 0.0f, 1e-4, "autoinit velocity D");
}

static void scenario_autoinit_local_pos(void)
{
    printf("\n=== Scenario 15: auto-init bootstraps from a local NED fix ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* Truth: stationary, level. */
    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* First fix: local NED position 3 m north of the (provisional) origin
       supplied to ins_init (e.g. a lighthouse/mocap system). */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.local_pos.pos_ned[0] = 3.0f;
    m.local_pos.pos_ned[1] = 0.0f;
    m.local_pos.pos_ned[2] = 0.0f;
    m.local_pos.Qll_ned[0] = 0.01f;
    m.local_pos.Qll_ned[4] = 0.01f;
    m.local_pos.Qll_ned[8] = 0.01f;
    m.local_pos.is_valid   = true;
    ins_update(&f, &m);

    if (!f.is_initialized || f.is_collecting)
    {
        printf("  FAIL  filter did not bootstrap on the local-pos fix\n");
        fails++;
        return;
    }

    /* No GNSS fix was ever offered, so the origin must stay the anchor
       supplied to ins_init; pos_local must read the fix directly. */
    CHECK_NEAR(f.origin_ecef[0], init.x_ecef[0], 1e-3, "autoinit origin ECEF x");
    CHECK_NEAR(f.origin_ecef[1], init.x_ecef[1], 1e-3, "autoinit origin ECEF y");
    CHECK_NEAR(f.origin_ecef[2], init.x_ecef[2], 1e-3, "autoinit origin ECEF z");

    float p[3];
    ins_get_position_local(&f, p);
    CHECK_NEAR(p[0], 3.0f, 1e-3, "autoinit local pos N");
    CHECK_NEAR(p[1], 0.0f, 1e-3, "autoinit local pos E");
    CHECK_NEAR(p[2], 0.0f, 1e-3, "autoinit local pos D");

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, 0.0f, 0.01, "autoinit level roll (local-pos path)");
    CHECK_NEAR(pitch, 0.0f, 0.01, "autoinit level pitch (local-pos path)");
}

static void scenario_autoinit_mag_yaw(void)
{
    printf("\n=== Scenario 16: auto-init resolves yaw from the magnetometer "
           "===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* level */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* True yaw = 25 deg. Roll/pitch are level, so the leveled magnetometer
       reading equals the raw reading, and model_hdg = atan2(0, 20) = 0
       (init.magnetic_n = [20, 0, 45]): pick mag.data so that
       atan2(mag.data[1], mag.data[0]) = -yaw_true. */
    const float yaw_true = 25.0f * (float)M_PI / 180.0f;
    const float mh =
        sqrtf(init.magnetic_n[0] * init.magnetic_n[0] + init.magnetic_n[1] * init.magnetic_n[1]);

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.mag.data[0]          = mh * cosf(-yaw_true);
    m.mag.data[1]          = mh * sinf(-yaw_true);
    m.mag.data[2]          = init.magnetic_n[2];
    m.mag.is_valid         = true;
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter did not bootstrap\n");
        fails++;
        return;
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, yaw_true, 0.02, "autoinit yaw from magnetometer");

    const float yaw_var    = f.d[INS_IDX_RPY + 2];
    const float expect_var = init.rpy_init_stddev_rad[0] * init.rpy_init_stddev_rad[0];
    CHECK_NEAR(yaw_var, expect_var, 1e-4, "autoinit yaw variance from mag (not 'unknown')");
}

/* REQ-NAV-044: the bootstrap fix carries no concurrent mag (GNSS and the
 * magnetometer run on independent clocks), but a mag was fed on an earlier
 * epoch. The heading must still bootstrap from that cached sample instead of
 * dropping to the "unknown yaw" fallback. */
static void scenario_autoinit_mag_yaw_cached(void)
{
    printf("\n=== Scenario: auto-init resolves yaw from a CACHED magnetometer "
           "when the fix epoch has none ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* level */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float yaw_true = 25.0f * (float)M_PI / 180.0f;
    const float mh =
        sqrtf(init.magnetic_n[0] * init.magnetic_n[0] + init.magnetic_n[1] * init.magnetic_n[1]);

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;

    /* Warm-up epochs carry IMU + magnetometer but NO position fix: the mag is
       cached, the filter cannot bootstrap yet (no origin). */
    for (step = 1; step <= 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]  = mh * cosf(-yaw_true);
        m.mag.data[1]  = mh * sinf(-yaw_true);
        m.mag.data[2]  = init.magnetic_n[2];
        m.mag.is_valid = true;
        ins_update(&f, &m);
    }

    if (f.is_initialized)
    {
        printf("  FAIL  bootstrapped before any position fix\n");
        fails++;
        return;
    }

    /* Bootstrap epoch: IMU + a position fix, but the magnetometer is absent
       from this call. The cached warm-up sample must supply the heading. */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter did not bootstrap\n");
        fails++;
        return;
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, yaw_true, 0.02, "autoinit yaw from cached magnetometer");

    const float yaw_var    = f.d[INS_IDX_RPY + 2];
    const float expect_var = init.rpy_init_stddev_rad[0] * init.rpy_init_stddev_rad[0];
    CHECK_NEAR(yaw_var, expect_var, 1e-4, "cached-mag yaw variance is mag-derived, not 'unknown'");
}

/* REQ-NAV-045: entry into 3D (ins ready) is held off until the position fix
 * stream has been continuously usable for gnss_init_dwell_sec AND carried
 * >= 1 fix/s over that window, for the cold-start bootstrap and, after a
 * health-fail re-acquisition, again. A rejected fix / gap resets the run, so
 * a marginal or flapping fix cannot toggle 3D on and off. */
static void scenario_gnss_init_dwell(void)
{
    printf("\n=== Scenario: GNSS-stability dwell before entering 3D (REQ-NAV-045) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* level, static */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f; /* 100 Hz IMU */
    const float dwell_sec   = 2.0f;  /* -> min_count = 2 fixes */

    /* One 100 Hz IMU epoch, plus a usable GNSS fix every `fix_period_s`. */
#define DWELL_STEP(F, T, FIX_PERIOD_S)                      \
    do {                                                    \
        (T) += us_from_sec(dt);                             \
        ins_measurements_t m;                               \
        memset(&m, 0, sizeof(m));                           \
        m.timestamp = (T);                                  \
        set_imu(&m, acc_body, gyr_body, dt);                \
        const long period_us = (long)((FIX_PERIOD_S)*1e6f); \
        if ((long)((T) % period_us) < (long)(dt * 1e6f))    \
        {                                                   \
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];        \
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];        \
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];        \
            m.gnss_pos.Qll_ned[0]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[4]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[8]  = 1.0f;                  \
            m.gnss_pos.is_valid    = true;                  \
        }                                                   \
        ins_update(&(F), &m);                               \
    } while (0)

    /* Part 1: a dense (5 Hz) stream. The dwell must hold 3D off until the
       window has elapsed, then admit it. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init               = true;
        opt.gnss_init_dwell_disable = false; /* the behaviour under test */
        opt.gnss_init_dwell_sec     = dwell_sec;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(1.5e6)) { DWELL_STEP(f, t, 0.2f); } /* 5 Hz */
        CHECK_TRUE(!f.is_initialized, "3D held off before the dwell elapses");

        while (t < (ins_time_us_t)(2.6e6)) { DWELL_STEP(f, t, 0.2f); }
        CHECK_TRUE(f.is_initialized, "3D entered once the dwell is satisfied");
    }

    /* Part 2: a sparse (every 3 s > max-gap) stream, below the 1 Hz floor.
       Every fix restarts the run, so 3D must never engage (anti-flap). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init               = true;
        opt.gnss_init_dwell_disable = false;
        opt.gnss_init_dwell_sec     = dwell_sec;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(12e6)) { DWELL_STEP(f, t, 3.0f); }
        CHECK_TRUE(!f.is_initialized, "flapping/sub-1Hz fixes never enter 3D");
    }

    /* Part 3: after a health-fail re-acquisition the dwell must be re-earned.
       A single fresh fix must not snap 3D back on. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init               = true; /* auto_reacquire_disable left false: on by default */
        opt.gnss_init_dwell_disable = false;
        opt.gnss_init_dwell_sec     = dwell_sec;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(2.6e6)) { DWELL_STEP(f, t, 0.2f); }
        CHECK_TRUE(f.is_initialized, "bootstrapped after the initial dwell");

        f.d[INS_IDX_VEL] = NAN; /* trip the health check */
        {
            ins_measurements_t mq;
            memset(&mq, 0, sizeof(mq));
            mq.timestamp = t; /* same timestamp -> predict skipped, poke survives */
            set_imu(&mq, acc_body, gyr_body, dt);
            ins_update(&f, &mq);
        }
        CHECK_TRUE(!f.is_initialized && f.is_collecting, "health fail re-armed into collecting");

        /* One coherent IMU+fix epoch must NOT re-enter 3D immediately. */
        DWELL_STEP(f, t, 0.01f); /* fix every epoch, but the run just restarted */
        CHECK_TRUE(!f.is_initialized, "re-acquisition re-earns the dwell (no instant flap)");

        while (t < (ins_time_us_t)(5.4e6)) { DWELL_STEP(f, t, 0.2f); }
        CHECK_TRUE(f.is_initialized, "3D re-enters after the dwell is re-earned");
    }
#undef DWELL_STEP
}

/* GNSS quality hysteresis around the 3D solution: a strict entry gate
   (REQ-NAV-051) and a much looser exit gate with its own long dwell
   (REQ-NAV-052), both separate from the per-fix fusion gate (REQ-NAV-007). */
static void scenario_gnss_mode_hysteresis(void)
{
    printf("\n=== Scenario: GNSS quality hysteresis entering/leaving 3D "
           "(REQ-NAV-051, REQ-NAV-052) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* level, static */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f; /* 100 Hz IMU */

    /* One IMU epoch, plus a 5 Hz GNSS pos+vel fix whose velocity 1-sigma is
       VEL_STD. The position stays at 1 m (inside every pos gate), so only
       the velocity quality decides which gate the epoch passes. */
#define HYST_STEP(F, T, VEL_STD)                            \
    do {                                                    \
        (T) += us_from_sec(dt);                             \
        ins_measurements_t m;                               \
        memset(&m, 0, sizeof(m));                           \
        m.timestamp = (T);                                  \
        set_imu(&m, acc_body, gyr_body, dt);                \
        if ((long)((T) % 200000) < (long)(dt * 1e6f))       \
        {                                                   \
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];        \
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];        \
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];        \
            m.gnss_pos.Qll_ned[0]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[4]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[8]  = 1.0f;                  \
            m.gnss_pos.is_valid    = true;                  \
            m.gnss_vel.Qll_ned[0]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[4]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[8]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.is_valid    = true;                  \
        }                                                   \
        ins_update(&(F), &m);                               \
    } while (0)

    /* Part 1: the entry gate is stricter than the fusion gate. A velocity
       1-sigma of 0.5 m/s is fusable (fill_default_opt allows 1.0/2.0 m/s)
       but must not admit the 3D solution (entry gate 0.25 m/s). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt); /* dwell disabled: the gate alone is under test */
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(5e6)) { HYST_STEP(f, t, 0.5f); }
        CHECK_TRUE(!f.is_initialized, "fusable but not entry-quality velocity keeps 3D off");

        while (t < (ins_time_us_t)(6e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(f.is_initialized, "3D entered once the velocity meets the entry gate");
    }

    /* Part 2: leaving on a sustained quality loss, and the hysteresis on the
       way back (a fix that is too poor to enter must not re-enter). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init               = true;
        opt.gnss_init_dwell_disable = false;
        opt.gnss_init_dwell_sec     = 2.0f; /* -> min_count = 2 fixes */
        opt.gnss_stop_dwell_sec     = 5.0f;
        /* The bootstrap warm-up below is only observable with the coasting
           timeout in force: allow_unlimited_deadreckoning waives it. Fixes
           arrive at 5 Hz throughout, so the timeout never bites here. */
        opt.allow_unlimited_deadreckoning = false;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(5e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "3D ready on good fixes");

        /* 0.6 m/s: still fused, but below the exit gate (0.4 m/s). */
        while (t < (ins_time_us_t)(9e6)) { HYST_STEP(f, t, 0.6f); }
        CHECK_TRUE(ins_is_ready(&f), "a bad run shorter than the stop dwell keeps 3D");

        while (t < (ins_time_us_t)(11e6)) { HYST_STEP(f, t, 0.6f); }
        CHECK_TRUE(!ins_is_ready(&f), "3D left after the stop dwell of bad fixes");
        CHECK_TRUE(f.diag.n_gnss_quality_exit == 1, "quality exit counted once");
        CHECK_TRUE(f.diag.n_health_reset == 0, "a quality exit is not a health reset");
        /* The filter stops rather than coasting on: re-armed into collecting,
           publishing nothing at all. */
        CHECK_TRUE(!f.is_initialized && f.is_collecting, "quality exit re-arms into collecting");
        {
            float p[3];
            CHECK_TRUE(!ins_get_position_local(&f, p), "no position reported after the exit");
        }

        /* Recovering, but only to a quality that is fusable, not
           entry-worthy: the solution must stay down. */
        while (t < (ins_time_us_t)(15e6)) { HYST_STEP(f, t, 0.35f); }
        CHECK_TRUE(!ins_is_ready(&f), "no re-entry on merely fusable fixes");

        /* Genuinely good fixes: one is not enough, the entry dwell applies. */
        while (t < (ins_time_us_t)(15.5e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(!f.is_initialized, "re-entry waits for the entry dwell");

        while (!f.is_initialized && t < (ins_time_us_t)(20e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(f.is_initialized, "re-bootstrapped after the entry dwell");
        /* Readiness is no longer a flag flip: the re-bootstrap has to serve
           the same warm-up any cold start does. */
        CHECK_TRUE(!ins_is_ready(&f), "readiness still waits out the bootstrap warm-up");

        while (t < (ins_time_us_t)(22e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "3D re-entered after the warm-up");
        CHECK_TRUE(f.diag.n_gnss_quality_exit == 1, "no further exits counted");
    }

    /* Part 2b: with the autonomous re-arm opted out of, the exit falls back
       to the pre-REQ-NAV-061 hold-down: the filter runs on and only
       ins_is_ready() goes false. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init               = true;
        opt.auto_reacquire_disable  = true;
        opt.gnss_init_dwell_disable = false;
        opt.gnss_init_dwell_sec     = 2.0f;
        opt.gnss_stop_dwell_sec     = 5.0f;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(3e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "3D ready on good fixes");

        while (t < (ins_time_us_t)(9e6)) { HYST_STEP(f, t, 0.6f); }
        CHECK_TRUE(!ins_is_ready(&f), "3D left after the stop dwell of bad fixes");
        CHECK_TRUE(f.is_initialized, "auto_reacquire_disable keeps the filter running");
        CHECK_TRUE(!f.bias_carry.valid, "the hold-down path arms no bias carry");

        /* The hold-down is not permanent: the entry dwell has to be
           re-earned, and once it is, the running filter is declared 3D
           again without any re-bootstrap. */
        while (t < (ins_time_us_t)(9.5e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(!ins_is_ready(&f), "the hold-down waits for the entry dwell");
        while (t < (ins_time_us_t)(13e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "the hold-down lifts once the entry dwell is re-earned");
        CHECK_TRUE(f.diag.n_gnss_quality_exit == 1, "recovering is not counted as another exit");
    }

    /* Part 3: gnss_stop_disable keeps the pre-REQ-NAV-052 behaviour. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init           = true;
        opt.gnss_stop_dwell_sec = 5.0f;
        opt.gnss_stop_disable   = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(2e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "3D ready on good fixes");

        while (t < (ins_time_us_t)(20e6)) { HYST_STEP(f, t, 0.6f); }
        CHECK_TRUE(ins_is_ready(&f), "gnss_stop_disable never leaves 3D on quality");
        CHECK_TRUE(f.diag.n_gnss_quality_exit == 0, "no quality exit counted");
    }

    /* Part 4: a plain outage is not a quality loss. No fix at all must never
       trip the exit gate (that is max_deadreckoning_sec's job). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init           = true;
        opt.gnss_stop_dwell_sec = 5.0f;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        while (t < (ins_time_us_t)(2e6)) { HYST_STEP(f, t, 0.1f); }
        CHECK_TRUE(ins_is_ready(&f), "3D ready on good fixes");

        while (t < (ins_time_us_t)(20e6))
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m); /* IMU only: complete GNSS outage */
        }
        CHECK_TRUE(ins_is_ready(&f), "an outage alone does not trip the quality exit");
        CHECK_TRUE(f.diag.n_gnss_quality_exit == 0, "no quality exit counted");
    }

    /* Part 5: gate ordering is enforced at init - an entry gate looser than
       the fusion gate and an exit gate stricter than the entry gate are both
       clamped instead of inverting the hysteresis. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init                                = true;
        opt.gnss_max_horizontal_pos_stddev_m         = 5.0f;
        opt.gnss_start_max_horizontal_pos_stddev_m   = 8.0f; /* > fusion gate */
        opt.gnss_stop_max_horizontal_pos_stddev_m    = 1.0f; /* < entry gate */
        opt.gnss_max_horizontal_vel_stddev_mps       = 0.5f;
        opt.gnss_start_max_horizontal_vel_stddev_mps = 0.2f;
        opt.gnss_stop_max_horizontal_vel_stddev_mps  = 0.1f; /* < entry gate */
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");
        CHECK_NEAR(f.opt.gnss_start_max_horizontal_pos_stddev_m, 5.0f, 1e-9,
                   "entry gate clamped to the fusion gate");
        CHECK_NEAR(f.opt.gnss_stop_max_horizontal_pos_stddev_m, 5.0f, 1e-9,
                   "exit gate clamped to the entry gate");
        CHECK_NEAR(f.opt.gnss_stop_max_horizontal_vel_stddev_mps, 0.2f, 1e-9,
                   "exit vel gate clamped to the entry vel gate");
    }
#undef HYST_STEP
}

/* The IMU biases survive a quality-loss re-arm, with an inflated and
   cold-start-clamped uncertainty, while everything else is re-derived by the
   next bootstrap (REQ-NAV-061). */
static void scenario_gnss_quality_exit_bias_carry(void)
{
    printf("\n=== Scenario: imu bias carry-over across a quality-loss re-arm "
           "(REQ-NAV-061) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    /* A small x-accel offset the filter can work against, so the carried
       bias is not trivially the cold-start value. */
    const float acc_body[3] = {0.2f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

#define CARRY_STEP(F, T, VEL_STD)                           \
    do {                                                    \
        (T) += us_from_sec(dt);                             \
        ins_measurements_t m;                               \
        memset(&m, 0, sizeof(m));                           \
        m.timestamp = (T);                                  \
        set_imu(&m, acc_body, gyr_body, dt);                \
        if ((long)((T) % 200000) < (long)(dt * 1e6f))       \
        {                                                   \
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];        \
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];        \
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];        \
            m.gnss_pos.Qll_ned[0]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[4]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[8]  = 1.0f;                  \
            m.gnss_pos.is_valid    = true;                  \
            m.gnss_vel.Qll_ned[0]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[4]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[8]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.is_valid    = true;                  \
        }                                                   \
        ins_update(&(F), &m);                               \
    } while (0)

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init               = true;
    opt.gnss_init_dwell_disable = true;
    opt.gnss_stop_dwell_sec     = 5.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    ins_time_us_t t = 0;
    while (t < (ins_time_us_t)(10e6)) { CARRY_STEP(f, t, 0.1f); }
    CHECK_TRUE(ins_is_ready(&f), "3D ready on good fixes");
    const float acc_var_ready = test_state_variance(&f, INS_IDX_ACC + 0);

    /* Sustained sub-exit-gate velocity quality: re-arms the filter. Step to
       the exit epoch itself, so the state can be compared against the carry
       without the intervening epochs moving it. */
    while (f.is_initialized && t < (ins_time_us_t)(20e6)) { CARRY_STEP(f, t, 0.6f); }
    CHECK_TRUE(!f.is_initialized && f.is_collecting, "quality exit re-armed the filter");
    CHECK_TRUE(f.bias_carry.valid, "a bias carry is pending");
    /* The re-arm leaves f.state untouched, so it still holds the exit values
       the carry was taken from. */
    CHECK_NEAR(f.bias_carry.acc_bias[0], f.state.acc_bias[0], 1e-9,
               "carried acc bias is the exit value");
    CHECK_NEAR(f.bias_carry.gyr_bias[2], f.state.gyr_bias[2], 1e-12,
               "carried gyr bias is the exit value");
    CHECK_TRUE(f.bias_carry.acc_bias_stddev_mps2 <= init.acc_bias_init_stddev_mps2,
               "carried acc stddev never exceeds the cold-start prior");
    CHECK_TRUE(f.bias_carry.acc_bias_stddev_mps2 * f.bias_carry.acc_bias_stddev_mps2 >
                   acc_var_ready,
               "carried acc stddev is inflated above the converged value");

    const float carried_acc    = f.bias_carry.acc_bias[0];
    const float carried_gyr    = f.bias_carry.gyr_bias[2];
    const float carried_acc_sd = f.bias_carry.acc_bias_stddev_mps2;

    /* Make the cold-start seed unmistakably distinct: if the re-bootstrap
       used it instead of the carry, the assertions below would show it. */
    f.init.acc_bias_init_mps2[0] = 9.0f;

    while (!f.is_initialized && t < (ins_time_us_t)(30e6)) { CARRY_STEP(f, t, 0.1f); }
    CHECK_TRUE(f.is_initialized, "re-bootstrapped on good fixes");
    CHECK_TRUE(!f.bias_carry.valid, "the carry is cleared once consumed");
    CHECK_NEAR(f.state.acc_bias[0], carried_acc, 1e-9,
               "acc bias carried over, not re-seeded from init");
    CHECK_NEAR(f.state.gyr_bias[2], carried_gyr, 1e-12, "gyr bias carried over");
    CHECK_NEAR(test_state_variance(&f, INS_IDX_ACC + 0), carried_acc_sd * carried_acc_sd, 1e-9,
               "acc bias seeded with the carried (inflated) variance");
#undef CARRY_STEP
}

/* An ECEF position offset from llh0 by a NED delta, for building fixes at a
   known distance from the origin. */
static void test_ecef_offset_ned(const double llh0[3], const float dned[3], double out_ecef[3])
{
    double dllh[3];
    ins_dned_to_dlatlonh(dned, llh0[0], llh0[2], dllh);
    ins_latlonh_to_ecef(llh0[0] + dllh[0], llh0[1] + dllh[1], llh0[2] + dllh[2], out_ecef);
}

/* The n-frame origin survives a quality-loss re-arm, so the local NED frame a
   consumer speaks means the same thing before and after the outage
   (REQ-NAV-062). The bootstrap fix then lands at its true offset in that
   frame instead of resetting it to zero, while the absolute solution is
   anchored to the fix either way. Two ways out of the carry are covered as
   well: a bootstrap too far from the inherited origin, and the other re-arm
   paths, which must not inherit one at all. */
static void scenario_gnss_quality_exit_origin_carry(void)
{
    printf("\n=== Scenario: n-frame origin carry-over across a quality-loss re-arm "
           "(REQ-NAV-062) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    double llh0[3];
    ins_ecef_to_latlonh(init.x_ecef, &llh0[0], &llh0[1], &llh0[2]);
    float g_vec[3];
    ins_gravity_ned((float)llh0[0], (float)llh0[2], g_vec);

    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* Where the platform reappears. dned_near is a few hundred metres from
       the start, reachable in any outage. dned_far is 150 km away, which no
       outage in this scenario can cover - but note it is only the DISTANCE
       TRAVELLED that decides (REQ-NAV-062), so the same 150 km appears again
       in part 2b as a legitimate carry, reached before the outage rather
       than during it. */
    const float dned_near[3] = {500.0f, 200.0f, 0.0f};
    const float dned_far[3]  = {150.0e3f, 0.0f, 0.0f};
    double      fix_near[3], fix_far[3];
    test_ecef_offset_ned(llh0, dned_near, fix_near);
    test_ecef_offset_ned(llh0, dned_far, fix_far);

    /* One IMU epoch plus a 5 Hz fix at FIX_ECEF whose velocity 1-sigma is
       VEL_STD, the only quality knob (the position stays inside every gate). */
#define ORIGIN_STEP(F, T, FIX_ECEF, VEL_STD)                \
    do {                                                    \
        (T) += us_from_sec(dt);                             \
        ins_measurements_t m;                               \
        memset(&m, 0, sizeof(m));                           \
        m.timestamp = (T);                                  \
        set_imu(&m, acc_body, gyr_body, dt);                \
        if ((long)((T) % 200000) < (long)(dt * 1e6f))       \
        {                                                   \
            m.gnss_pos.xyz_ecef[0] = (FIX_ECEF)[0];         \
            m.gnss_pos.xyz_ecef[1] = (FIX_ECEF)[1];         \
            m.gnss_pos.xyz_ecef[2] = (FIX_ECEF)[2];         \
            m.gnss_pos.Qll_ned[0]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[4]  = 1.0f;                  \
            m.gnss_pos.Qll_ned[8]  = 1.0f;                  \
            m.gnss_pos.is_valid    = true;                  \
            m.gnss_vel.Qll_ned[0]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[4]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.Qll_ned[8]  = (VEL_STD) * (VEL_STD); \
            m.gnss_vel.is_valid    = true;                  \
        }                                                   \
        ins_update(&(F), &m);                               \
    } while (0)

    /* Same, plus a barometer sample for the altitude BARO_ALT, so the
       bootstrap latches the barometric height source (REQ-NAV-053). */
#define ORIGIN_STEP_BARO(F, T, FIX_ECEF, VEL_STD, BARO_ALT)   \
    do {                                                      \
        (T) += us_from_sec(dt);                               \
        ins_measurements_t m;                                 \
        memset(&m, 0, sizeof(m));                             \
        m.timestamp = (T);                                    \
        set_imu(&m, acc_body, gyr_body, dt);                  \
        set_baro(&m, pressure_from_altitude(BARO_ALT), 1.0f); \
        if ((long)((T) % 200000) < (long)(dt * 1e6f))         \
        {                                                     \
            m.gnss_pos.xyz_ecef[0] = (FIX_ECEF)[0];           \
            m.gnss_pos.xyz_ecef[1] = (FIX_ECEF)[1];           \
            m.gnss_pos.xyz_ecef[2] = (FIX_ECEF)[2];           \
            m.gnss_pos.Qll_ned[0]  = 1.0f;                    \
            m.gnss_pos.Qll_ned[4]  = 1.0f;                    \
            m.gnss_pos.Qll_ned[8]  = 1.0f;                    \
            m.gnss_pos.is_valid    = true;                    \
            m.gnss_vel.Qll_ned[0]  = (VEL_STD) * (VEL_STD);   \
            m.gnss_vel.Qll_ned[4]  = (VEL_STD) * (VEL_STD);   \
            m.gnss_vel.Qll_ned[8]  = (VEL_STD) * (VEL_STD);   \
            m.gnss_vel.is_valid    = true;                    \
        }                                                     \
        ins_update(&(F), &m);                                 \
    } while (0)

#define ORIGIN_SETUP(F, OPT, T)                                                       \
    do {                                                                              \
        memset(&(F), 0, sizeof(F));                                                   \
        fill_default_opt(&(OPT));                                                     \
        (OPT).auto_init               = true;                                         \
        (OPT).gnss_init_dwell_disable = true;                                         \
        (OPT).gnss_stop_dwell_sec     = 5.0f;                                         \
        CHECK_TRUE(ins_init(&(F), &init, &(OPT)) == 0, "init ok");                    \
        (T) = 0;                                                                      \
        while ((T) < (ins_time_us_t)(10e6)) { ORIGIN_STEP(F, T, init.x_ecef, 0.1f); } \
        CHECK_TRUE(ins_is_ready(&(F)), "3D ready on good fixes");                     \
    } while (0)

    /* Part 1: the carry. The re-bootstrap keeps the origin, so pos_local
       reports where the platform actually is in the frame it started in. */
    {
        ins_t         f;
        ins_options_t opt;
        ins_time_us_t t;
        ORIGIN_SETUP(f, opt, t);

        double origin0[3];
        origin0[0] = f.origin_ecef[0];
        origin0[1] = f.origin_ecef[1];
        origin0[2] = f.origin_ecef[2];

        while (f.is_initialized && t < (ins_time_us_t)(20e6))
        {
            ORIGIN_STEP(f, t, init.x_ecef, 0.6f);
        }
        CHECK_TRUE(!f.is_initialized && f.is_collecting, "quality exit re-armed the filter");
        CHECK_TRUE(f.origin_carry.valid, "an origin carry is pending");
        CHECK_NEAR(f.origin_carry.origin_ecef[0], origin0[0], 1e-9,
                   "the carried origin is the exiting instance's origin");

        /* The platform reappears 500 m north / 200 m east. */
        while (!f.is_initialized && t < (ins_time_us_t)(35e6))
        {
            ORIGIN_STEP(f, t, fix_near, 0.1f);
        }
        CHECK_TRUE(f.is_initialized, "re-bootstrapped on good fixes");
        CHECK_TRUE(!f.origin_carry.valid, "the carry is cleared once consumed");

        CHECK_NEAR(f.origin_ecef[0], origin0[0], 1e-9, "origin x kept across the re-arm");
        CHECK_NEAR(f.origin_ecef[1], origin0[1], 1e-9, "origin y kept across the re-arm");
        CHECK_NEAR(f.origin_ecef[2], origin0[2], 1e-9, "origin z kept across the re-arm");

        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "position available after the re-bootstrap");
        CHECK_NEAR(pos[0], dned_near[0], 0.5, "pos_local north is the true offset, not zero");
        CHECK_NEAR(pos[1], dned_near[1], 0.5, "pos_local east is the true offset, not zero");

        /* The absolute solution is anchored to the bootstrap fix regardless,
           which is what makes the local frame free to be inherited. */
        double p_ecef[3];
        CHECK_TRUE(ins_get_position_ecef(&f, p_ecef), "ecef available");
        CHECK_NEAR(p_ecef[0], fix_near[0], 0.1, "absolute x anchored to the bootstrap fix");
        CHECK_NEAR(p_ecef[1], fix_near[1], 0.1, "absolute y anchored to the bootstrap fix");
        CHECK_NEAR(p_ecef[2], fix_near[2], 0.1, "absolute z anchored to the bootstrap fix");
    }

    /* Part 2a: a bootstrap the platform could not possibly have reached
       during the outage refuses the carry and anchors a fresh origin, so
       pos_local restarts at zero. 150 km in a ~15 s outage is a spliced log
       or a different mission, not a drive. */
    {
        ins_t         f;
        ins_options_t opt;
        ins_time_us_t t;
        ORIGIN_SETUP(f, opt, t);

        while (f.is_initialized && t < (ins_time_us_t)(20e6))
        {
            ORIGIN_STEP(f, t, init.x_ecef, 0.6f);
        }
        CHECK_TRUE(f.origin_carry.valid, "an origin carry is pending");
        /* The snapshot the decision is made against comes from the state the
           exiting instance held (the re-arm leaves f.state alone). */
        CHECK_NEAR(f.origin_carry.pos_local[0], f.state.pos_local[0], 1e-6,
                   "the carry snapshots the exiting instance's position");
        CHECK_NEAR(f.origin_carry.latlonh[0], f.latlonh[0], 1e-12,
                   "and the same position as an absolute anchor");
        CHECK_NEAR(f.origin_carry.latlonh[1], f.latlonh[1], 1e-12,
                   "and the same position as an absolute anchor (lon)");

        while (!f.is_initialized && t < (ins_time_us_t)(35e6)) { ORIGIN_STEP(f, t, fix_far, 0.1f); }
        CHECK_TRUE(f.is_initialized, "re-bootstrapped far away");
        CHECK_NEAR(f.origin_ecef[0], fix_far[0], 1e-9,
                   "an unreachable bootstrap anchors a fresh origin");

        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "position available");
        CHECK_NEAR(pos[0], 0.0f, 1e-3, "pos_local restarts at zero in the fresh frame");
        CHECK_NEAR(pos[1], 0.0f, 1e-3, "pos_local restarts at zero in the fresh frame");
    }

    /* Part 2b: the same 150 km, but travelled BEFORE the outage - a long
       trip with a tunnel near the far end. Distance to the origin is
       therefore no reason to refuse: the platform reappears next to where it
       disappeared, which is all the carry needs. This is the case a bound on
       the distance to the origin used to get wrong, resetting the local
       frame of every mission longer than that bound.
       The far starting point is poked into the state rather than driven to
       (150 km at any real speed does not fit in a unit test); the production
       path that copies state -> carry is asserted in part 2a. */
    {
        ins_t         f;
        ins_options_t opt;
        ins_time_us_t t;
        ORIGIN_SETUP(f, opt, t);

        double origin0[3];
        origin0[0] = f.origin_ecef[0];
        origin0[1] = f.origin_ecef[1];
        origin0[2] = f.origin_ecef[2];

        while (f.is_initialized && t < (ins_time_us_t)(20e6))
        {
            ORIGIN_STEP(f, t, init.x_ecef, 0.6f);
        }
        CHECK_TRUE(f.origin_carry.valid, "an origin carry is pending");

        /* "The filter was 150 km from its origin when the aiding failed."
           The carry is a PAIR - the same position in the local frame and as
           an absolute anchor - so both halves move to the far end, otherwise
           the state describes a platform in two places at once. */
        f.origin_carry.pos_local[0] = dned_far[0];
        f.origin_carry.pos_local[1] = dned_far[1];
        f.origin_carry.pos_local[2] = dned_far[2];
        ins_ecef_to_latlonh(fix_far, &f.origin_carry.latlonh[0], &f.origin_carry.latlonh[1],
                            &f.origin_carry.latlonh[2]);

        while (!f.is_initialized && t < (ins_time_us_t)(35e6)) { ORIGIN_STEP(f, t, fix_far, 0.1f); }
        CHECK_TRUE(f.is_initialized, "re-bootstrapped at the far end of the trip");
        CHECK_NEAR(f.origin_ecef[0], origin0[0], 1e-9,
                   "origin kept 150 km from home: distance to the origin is not a refusal reason");
        CHECK_NEAR(f.origin_ecef[1], origin0[1], 1e-9, "origin kept (y)");
        CHECK_NEAR(f.origin_ecef[2], origin0[2], 1e-9, "origin kept (z)");

        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "position available");
        CHECK_NEAR(pos[0], dned_far[0], 0.5, "pos_local continues at the far end, not at zero");

        /* The point of the fix: the absolute solution is the bootstrap fix,
           however far the origin has been left behind. Deriving it from the
           origin instead used to bend it by the curvature cross term
           d_north * d_east * tan(lat) / R_earth - hundreds of metres after a
           hundred kilometres of driving, injected under a 1 m init prior, so
           the chi2 gate then let it back out only over tens of minutes. */
        double p_ecef[3];
        CHECK_TRUE(ins_get_position_ecef(&f, p_ecef), "ecef available");
        CHECK_NEAR(p_ecef[0], fix_far[0], 0.1, "absolute x is the fix, not the origin-mapped fix");
        CHECK_NEAR(p_ecef[1], fix_far[1], 0.1, "absolute y is the fix, not the origin-mapped fix");
        CHECK_NEAR(p_ecef[2], fix_far[2], 0.1, "absolute z is the fix, not the origin-mapped fix");
    }

    /* Part 3: a health-check re-arm inherits nothing. The state has just been
       declared untrustworthy, so the next bootstrap starts a fresh frame. */
    {
        ins_t         f;
        ins_options_t opt;
        ins_time_us_t t;
        ORIGIN_SETUP(f, opt, t);

        f.d[INS_IDX_VEL] = NAN; /* trip the health check */
        {
            ins_measurements_t mq;
            memset(&mq, 0, sizeof(mq));
            mq.timestamp = t; /* same timestamp: predict skipped, poke survives */
            set_imu(&mq, acc_body, gyr_body, dt);
            ins_update(&f, &mq);
        }
        CHECK_TRUE(!f.is_initialized && f.is_collecting, "health fail re-armed into collecting");
        CHECK_TRUE(!f.origin_carry.valid, "a health re-arm arms no origin carry");

        while (!f.is_initialized && t < (ins_time_us_t)(35e6))
        {
            ORIGIN_STEP(f, t, fix_near, 0.1f);
        }
        CHECK_TRUE(f.is_initialized, "re-bootstrapped on good fixes");
        CHECK_NEAR(f.origin_ecef[0], fix_near[0], 1e-9,
                   "the health re-bootstrap anchors the origin on its own fix");

        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "position available");
        CHECK_NEAR(pos[0], 0.0f, 1e-3, "pos_local restarts at zero after a health re-arm");
    }
    /* Part 4: the barometric height anchor follows the inherited origin.
       baro_h0_m defines where pos_local[2] = 0 sits on the pressure scale,
       and a carried origin bootstraps at a nonzero pos_local[2] whenever
       the platform reappears at a different altitude. Anchored against
       zero regardless, the whole height channel would be offset by that
       difference, which is the one way this feature could move the
       absolute solution it is not supposed to touch. */
    {
        ins_t         f;
        ins_options_t opt;
        ins_time_us_t t;

        memset(&f, 0, sizeof(f));
        fill_default_opt(&opt);
        opt.auto_init               = true;
        opt.gnss_init_dwell_disable = true;
        opt.gnss_stop_dwell_sec     = 5.0f;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        t = 0;
        while (t < (ins_time_us_t)(10e6)) { ORIGIN_STEP_BARO(f, t, init.x_ecef, 0.1f, 0.0f); }
        CHECK_TRUE(ins_is_ready(&f) && f.height_from_baro, "3D ready on barometric height");

        while (f.is_initialized && t < (ins_time_us_t)(20e6))
        {
            ORIGIN_STEP_BARO(f, t, init.x_ecef, 0.6f, 0.0f);
        }
        CHECK_TRUE(f.origin_carry.valid, "an origin carry is pending");

        /* The platform reappears 30 m higher, and the barometer agrees. */
        const float climb_m      = 30.0f;
        const float dned_high[3] = {500.0f, 200.0f, -climb_m};
        double      fix_high[3];
        test_ecef_offset_ned(llh0, dned_high, fix_high);

        while (!f.is_initialized && t < (ins_time_us_t)(35e6))
        {
            ORIGIN_STEP_BARO(f, t, fix_high, 0.1f, climb_m);
        }
        CHECK_TRUE(f.is_initialized && f.height_from_baro, "re-bootstrapped on barometric height");

        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "position available");
        CHECK_NEAR(pos[2], -climb_m, 0.5, "the bootstrap starts at the climbed height");

        /* Barometer and state agree, so ten seconds of samples at that same
           altitude must not drag the vertical anywhere. An anchor left at
           pos_local[2] = 0 would pull it by the full climb instead. */
        while (t < (ins_time_us_t)(45e6)) { ORIGIN_STEP_BARO(f, t, fix_high, 0.1f, climb_m); }
        CHECK_TRUE(ins_get_position_local(&f, pos), "position still available");
        CHECK_NEAR(pos[2], -climb_m, 0.5,
                   "the barometric anchor follows the inherited origin, height stays put");
    }
#undef ORIGIN_SETUP
#undef ORIGIN_STEP_BARO
#undef ORIGIN_STEP
}

/* Shared bootstrap helper for the barometric-height scenarios below: a few
 * IMU(+optional barometer) epochs to fill the leveling window (needs >= 3
 * samples, REQ-NAV-047/ins_autoinit_try), then one final epoch that adds the
 * position fix (GNSS or local, WITH_BARO controls whether that final epoch
 * also carries a barometer sample -- REQ-NAV-053 only needs the barometer to
 * have been seen at some point, not on the bootstrap epoch itself).
 */
static void baro_height_bootstrap(ins_t* f, const ins_init_t* init, ins_time_us_t* t,
                                  const float acc_body[3], const float gyr_body[3], float dt,
                                  bool with_baro, bool with_local_pos)
{
    int i;
    for (i = 0; i < 5; ++i)
    {
        *t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = *t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (with_baro) { set_baro(&m, pressure_from_altitude(0.0f), 1.0f); }
        ins_update(f, &m);
    }

    *t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = *t;
    set_imu(&m, acc_body, gyr_body, dt);
    if (with_baro) { set_baro(&m, pressure_from_altitude(0.0f), 1.0f); }
    if (with_local_pos)
    {
        m.local_pos.pos_ned[0] = 0.0f;
        m.local_pos.pos_ned[1] = 0.0f;
        m.local_pos.pos_ned[2] = 0.0f;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 1e-4f;
        m.local_pos.is_valid                                                     = true;
    }
    else
    {
        m.gnss_pos.xyz_ecef[0] = init->x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init->x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init->x_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        m.gnss_pos.is_valid                           = true;
        m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        m.gnss_vel.is_valid                                                   = true;
    }
    ins_update(f, &m);
}

static void scenario_baro_height_source_selection(void)
{
    printf("\n=== Scenario: barometric height source selection at bootstrap "
           "(REQ-NAV-053) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* Part 1: GNSS bootstrap, barometer seen during collecting -> barometric
       height selected. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);

        CHECK_TRUE(f.is_initialized, "bootstrapped");
        CHECK_TRUE(f.height_from_baro, "barometer seen -> barometric height selected");
    }

    /* Part 2: GNSS bootstrap, no barometer ever seen -> GNSS height kept
       (today's behaviour, unchanged). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, false, false);

        CHECK_TRUE(f.is_initialized, "bootstrapped");
        CHECK_TRUE(!f.height_from_baro, "no barometer seen -> GNSS height kept");
    }

    /* Part 3: local-position bootstrap, barometer seen -> GNSS/local height
       selection is never overridden by a barometer: a local-position system
       (lighthouse/UWB/mocap) supplies its own, typically better, vertical
       reference. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, true);

        CHECK_TRUE(f.is_initialized, "bootstrapped");
        CHECK_TRUE(!f.height_from_baro, "local-position bootstrap keeps its own height");
    }

    /* Part 4: opt.baro_height_disable overrides even a GNSS bootstrap with a
       barometer present -- for a GNSS-labelled source that is not really
       satellite GNSS and already reports better-than-barometric vertical
       accuracy (e.g. crazyflie's Lighthouse position). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init           = true;
        opt.baro_height_disable = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);

        CHECK_TRUE(f.is_initialized, "bootstrapped");
        CHECK_TRUE(!f.height_from_baro, "baro_height_disable keeps GNSS height");
    }
}

static void scenario_baro_height_fusion(void)
{
    printf("\n=== Scenario: barometric height fusion replaces GNSS's vertical "
           "row (REQ-NAV-054, REQ-NAV-055) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    /* Climb 5 m on the barometer alone. */
    int i;
    for (i = 0; i < 50; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(5.0f), 1.0f);
        ins_update(&f, &m);
    }
    float pos[3];
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[2], -5.0, 0.5, "barometric height converged to the 5 m climb");

    /* A GNSS fix with the correct horizontal position but a badly wrong
       (+100 m) implied height must leave the height channel untouched: its
       vertical row was dropped, not fused (REQ-NAV-055). No barometer this
       epoch, to isolate the GNSS-only effect. */
    const float pos_before_d = pos[2];
    double      lat0, lon0, h0;
    ins_ecef_to_latlonh(f.origin_ecef, &lat0, &lon0, &h0);
    double bad_ecef[3];
    ins_latlonh_to_ecef(lat0, lon0, h0 + 100.0, bad_ecef);

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = bad_ecef[0];
    m.gnss_pos.xyz_ecef[1] = bad_ecef[1];
    m.gnss_pos.xyz_ecef[2] = bad_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    ins_update(&f, &m);

    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[0], 0.0, 0.2, "N unaffected (fix agreed horizontally)");
    CHECK_NEAR(pos[1], 0.0, 0.2, "E unaffected (fix agreed horizontally)");
    CHECK_NEAR(pos[2], pos_before_d, 1e-3,
               "height NOT moved by GNSS's (dropped) vertical row despite a +100 m fix");
}

static void scenario_baro_height_survives_reacquire(void)
{
    printf("\n=== Scenario: barometric height survives a GNSS re-acquisition "
           "(REQ-NAV-054) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init                     = true;
    opt.allow_unlimited_deadreckoning = false;
    opt.max_deadreckoning_sec         = 1.0f; /* short window, keep the test quick */
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    /* Climb 5 m on the barometer alone. */
    int i;
    for (i = 0; i < 50; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(5.0f), 1.0f);
        ins_update(&f, &m);
    }
    float pos[3];
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[2], -5.0, 0.5, "height converged to the 5 m climb before the outage");

    /* GNSS outage past the coasting window (2 s > max_deadreckoning_sec).
       The barometer keeps streaming throughout: ins_fuse_baro_height runs
       unconditionally, not gated on the dead-reckoning freeze, so the
       height channel never actually goes stale even though the horizontal
       solution does. */
    for (i = 0; i < 200; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(5.0f), 1.0f);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "horizontal solution declared gone after the outage");
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[2], -5.0, 0.5, "height kept tracking the barometer through the outage");

    /* Tunnel exit: a fresh GNSS fix 20 m north, with a badly wrong (+100 m)
       implied height. Re-acquisition must snap N/E to the fix but leave the
       barometric height exactly where the barometer already had it. */
    double lat0, lon0, h0;
    ins_ecef_to_latlonh(init.x_ecef, &lat0, &lon0, &h0);
    const float dned_exit[3] = {20.0f, 0.0f, 0.0f};
    double      dllh[3], exit_ecef[3];
    ins_dned_to_dlatlonh(dned_exit, lat0, h0, dllh);
    ins_latlonh_to_ecef(lat0 + dllh[0], lon0 + dllh[1], h0 + dllh[2] + 100.0, exit_ecef);

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = exit_ecef[0];
    m.gnss_pos.xyz_ecef[1] = exit_ecef[1];
    m.gnss_pos.xyz_ecef[2] = exit_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    ins_update(&f, &m);

    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");
    CHECK_TRUE(ins_is_ready(&f), "ready again right after the fix");

    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[0], 20.0, 1.0, "horizontal position snapped to the fix (north)");
    CHECK_NEAR(pos[2], -5.0, 0.5,
               "height NOT snapped to the fix's implied +100 m -- still the barometer's climb");
}

/* A LONG frozen phase (REQ-NAV-022) during which the platform actually
 * changes height, which is what scenario_baro_height_survives_reacquire
 * above cannot see (2 s at a constant height).
 *
 * Two things are under test. While the window is expired the filter is
 * inert (REQ-NAV-064): the barometer keeps arriving and the height does NOT
 * follow it, because nothing is fused at all. Then the fix that ends the
 * outage re-anchors the height on that same barometer (REQ-NAV-066) rather
 * than on its own vertical row, which is what makes the vertical solution
 * continuous across a 200 s climb the frozen state knew nothing about. */
static void scenario_baro_height_reanchor_after_long_outage(void)
{
    printf("\n=== Scenario: barometric height re-anchored after a long "
           "outage (REQ-NAV-064, REQ-NAV-066) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init                     = true;
    opt.allow_unlimited_deadreckoning = false;
    opt.max_deadreckoning_sec         = 1.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    /* 200 s with no GNSS at all (the 1 s coasting window expires almost
       immediately), climbing steadily to 40 m at 0.2 m/s. The barometer is
       honest throughout and reports at 10 Hz, a realistic sensor rate. */
    const float climb_mps    = 0.2f;
    const int   outage_steps = 20000;
    int         i;
    /* Sampled once the 1 s window has expired, not at the outage's start:
       the first second is still ordinary coasting and does fuse. */
    const int frozen_at     = 200;
    uint32_t  dw_before     = 0;
    uint32_t  baro_before   = 0;
    float     pos_frozen[3] = {0.0f, 0.0f, 0.0f};
    for (i = 0; i < outage_steps; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 10) == 0)
        {
            set_baro(&m, pressure_from_altitude(climb_mps * (float)(i + 1) * dt), 1.0f);
        }
        ins_update(&f, &m);
        if (i == frozen_at)
        {
            ins_get_position_local(&f, pos_frozen);
            dw_before   = ins_get_diag(&f)->n_downweighted;
            baro_before = ins_get_diag(&f)->n_baro_height_used;
        }
    }
    CHECK_TRUE(!ins_is_ready(&f), "horizontal solution gone (frozen the whole time)");

    const float h_true = climb_mps * (float)outage_steps * dt; /* 40 m */
    float       pos[3];
    ins_get_position_local(&f, pos);
    /* REQ-NAV-064: ~2000 barometer samples arrived while inert and not one
       of them was fused, so the height stands where it did at the freeze --
       nearly 40 m below the platform. */
    CHECK_NEAR(-pos[2], -pos_frozen[2], 1e-4, "height frozen with everything else");
    CHECK_TRUE(fabsf(-pos[2] - h_true) > 30.0f,
               "sanity: the platform really did climb away from the frozen height");
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used == baro_before,
               "no barometric fusion while inert");
    CHECK_TRUE(ins_get_diag(&f)->n_downweighted == dw_before, "no fusion of any kind while inert");

    /* Tunnel exit: a fresh fix 20 m north whose implied height is 100 m too
       high. The height must come from the barometer, and the vertical
       velocity must not spike afterwards. */
    double lat0, lon0, h0;
    ins_ecef_to_latlonh(init.x_ecef, &lat0, &lon0, &h0);
    const float dned_exit[3] = {20.0f, 0.0f, 0.0f};
    double      dllh[3], exit_ecef[3];
    ins_dned_to_dlatlonh(dned_exit, lat0, h0, dllh);
    ins_latlonh_to_ecef(lat0 + dllh[0], lon0 + dllh[1], h0 + dllh[2] + 100.0, exit_ecef);

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = exit_ecef[0];
    m.gnss_pos.xyz_ecef[1] = exit_ecef[1];
    m.gnss_pos.xyz_ecef[2] = exit_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_pos.is_valid                           = true;
    ins_update(&f, &m);
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");

    /* REQ-NAV-066: the height jumps straight to what the barometer says,
       lag-free (the cached sample is 0.1 s old), and neither to the fix's
       +100 m nor to the frozen value. Its variance is that of a single
       barometer sample, not whatever the covariance happened to hold. */
    ins_get_position_local(&f, pos);
    CHECK_NEAR(-pos[2], h_true, 0.5, "height re-anchored on the barometer at the exit");
    CHECK_NEAR(sqrtf(test_state_variance(&f, INS_IDX_POS + 2)), 1.0, 0.05,
               "vertical variance rebuilt from the barometer sample accuracy");

    float max_vd = 0.0f;
    for (i = 0; i < 200; ++i) /* 2 s of level flight at the exit height */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 10) == 0) { set_baro(&m, pressure_from_altitude(h_true), 1.0f); }
        ins_update(&f, &m);
        float vel[3];
        ins_get_velocity_ned(&f, vel);
        if (fabsf(vel[2]) > max_vd) { max_vd = fabsf(vel[2]); }
    }
    ins_get_position_local(&f, pos);
    CHECK_NEAR(-pos[2], h_true, 1.0, "height still the barometer's after the re-acquisition");
    CHECK_TRUE(max_vd < 3.0f, "no spurious vertical velocity step at the re-acquisition");
}

/* A barometer that delivered during the collecting window and then stopped
 * must NOT latch the barometric height source (REQ-NAV-053): the decision
 * is never revisited, and the cached pressure would also anchor the datum
 * at a height the platform has long left. */
static void scenario_baro_height_stale_sample(void)
{
    printf("\n=== Scenario: stale barometer sample does not latch the height "
           "source (REQ-NAV-053) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    /* Hold the bootstrap off past the staleness horizon, the way the real
       GNSS entry dwell (REQ-NAV-045) does. */
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    ins_time_us_t t = 0;
    int           i;
    /* One barometer sample right at the start, then 10 s of IMU only. */
    for (i = 0; i < 1000; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (i == 0) { set_baro(&m, pressure_from_altitude(0.0f), 1.0f); }
        ins_update(&f, &m);
    }
    CHECK_TRUE(!f.is_initialized, "still collecting (no fix offered yet)");
    CHECK_TRUE(f.autoinit_baro.valid, "the sample was cached");

    /* Now the fix arrives. The cached pressure is 10 s old. */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    m.gnss_pos.is_valid                                                   = true;
    ins_update(&f, &m);

    CHECK_TRUE(f.is_initialized, "bootstrapped");
    CHECK_TRUE(!f.height_from_baro, "stale barometer sample -> GNSS height kept");

    /* Control: the identical sequence with the barometer still streaming at
       the bootstrap epoch DOES select it, so the check above is about
       staleness and not about the sequence itself. */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        ins_options_t opt2;
        fill_default_opt(&opt2);
        opt2.auto_init = true;
        CHECK_TRUE(ins_init(&f2, &init, &opt2) == 0, "init ok");

        ins_time_us_t t2 = 0;
        for (i = 0; i < 1000; ++i)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            if (i == 0 || i == 999) { set_baro(&m2, pressure_from_altitude(0.0f), 1.0f); }
            ins_update(&f2, &m2);
        }
        t2 += us_from_sec(dt);
        ins_measurements_t m2;
        memset(&m2, 0, sizeof(m2));
        m2.timestamp = t2;
        set_imu(&m2, acc_body, gyr_body, dt);
        m2.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m2.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m2.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m2.gnss_pos.Qll_ned[0] = m2.gnss_pos.Qll_ned[4] = m2.gnss_pos.Qll_ned[8] = 1.0f;
        m2.gnss_pos.is_valid                                                     = true;
        ins_update(&f2, &m2);
        CHECK_TRUE(f2.is_initialized && f2.height_from_baro,
                   "fresh barometer sample -> barometric height selected");
    }
}

/* Under the barometric height source a fix whose VERTICAL accuracy is far
 * outside the configured limits must still be fused horizontally, and must
 * not drop the 3D solution: its vertical row is discarded before fusion
 * anyway (REQ-NAV-055, REQ-NAV-057). */
static void scenario_baro_height_vertical_gate_ignored(void)
{
    printf("\n=== Scenario: vertical GNSS quality does not gate the horizontal "
           "solution under barometric height (REQ-NAV-057) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    /* Tight-ish vertical limits, generous horizontal ones. */
    opt.gnss_max_vertical_pos_stddev_m        = 5.0f;
    opt.gnss_stop_max_vertical_pos_stddev_m   = 5.0f;
    opt.gnss_max_horizontal_pos_stddev_m      = 20.0f;
    opt.gnss_stop_max_horizontal_pos_stddev_m = 20.0f;
    opt.gnss_stop_dwell_sec                   = 2.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    /* 10 s of fixes with a good horizontal but a hopeless vertical accuracy
       (50 m 1-sigma, ten times the limit). 20 m north, so a fused fix is
       visible in the state. */
    const uint32_t used_before     = ins_get_diag(&f)->n_gnss_used;
    const uint32_t rejected_before = ins_get_diag(&f)->n_gnss_rejected_noise;
    double         dllh[3], fix_ecef[3];
    const float    dned[3] = {20.0f, 0.0f, 0.0f};
    ins_dned_to_dlatlonh(dned, lat, h, dllh);
    ins_latlonh_to_ecef(lat + dllh[0], lon + dllh[1], h + dllh[2], fix_ecef);

    int i;
    for (i = 0; i < 1000; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(0.0f), 1.0f);
        if ((i % 10) == 0)
        {
            m.gnss_pos.xyz_ecef[0] = fix_ecef[0];
            m.gnss_pos.xyz_ecef[1] = fix_ecef[1];
            m.gnss_pos.xyz_ecef[2] = fix_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 1.0f;
            m.gnss_pos.Qll_ned[8]                         = 2500.0f; /* 50 m 1-sigma */
            m.gnss_pos.is_valid                           = true;
        }
        ins_update(&f, &m);
    }

    CHECK_TRUE(ins_get_diag(&f)->n_gnss_used > used_before,
               "fix fused despite a vertical accuracy far past the limit");
    CHECK_TRUE(ins_get_diag(&f)->n_gnss_rejected_noise == rejected_before,
               "fix not counted as rejected-for-noise");
    CHECK_TRUE(ins_is_ready(&f), "3D solution kept (vertical quality is not its business)");
    float pos[3];
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[0], 20.0, 1.0, "horizontal aiding actually took effect");
    CHECK_NEAR(pos[2], 0.0, 0.5, "height stayed barometric, unmoved by the bad vertical row");

    /* Control: without the barometric height source the same stream is
       rejected and the 3D solution is dropped, i.e. the vertical limits
       still do their job where the vertical row IS used. */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        ins_options_t opt2 = opt;
        CHECK_TRUE(ins_init(&f2, &init, &opt2) == 0, "init ok");
        ins_time_us_t t2 = 0;
        baro_height_bootstrap(&f2, &init, &t2, acc_body, gyr_body, dt, false, false);
        CHECK_TRUE(f2.is_initialized && !f2.height_from_baro, "bootstrapped on GNSS height");

        const uint32_t rej_before = ins_get_diag(&f2)->n_gnss_rejected_noise;
        for (i = 0; i < 1000; ++i)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            if ((i % 10) == 0)
            {
                m2.gnss_pos.xyz_ecef[0] = fix_ecef[0];
                m2.gnss_pos.xyz_ecef[1] = fix_ecef[1];
                m2.gnss_pos.xyz_ecef[2] = fix_ecef[2];
                m2.gnss_pos.Qll_ned[0] = m2.gnss_pos.Qll_ned[4] = 1.0f;
                m2.gnss_pos.Qll_ned[8]                          = 2500.0f;
                m2.gnss_pos.is_valid                            = true;
            }
            ins_update(&f2, &m2);
        }
        CHECK_TRUE(ins_get_diag(&f2)->n_gnss_rejected_noise > rej_before,
                   "control: same fix IS rejected when the vertical row is used");
    }
}

/* The aiding-gap report (REQ-NAV-058): the barometer stops while GNSS keeps
 * aiding the horizontal solution, so nothing else in the filter notices. */
static void scenario_baro_height_gap_warning(void)
{
    printf("\n=== Scenario: barometric height aiding gap is detected "
           "(REQ-NAV-058) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;
    int           i;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    for (i = 0; i < 200; ++i) /* 2 s with the barometer streaming */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(0.0f), 1.0f);
        ins_update(&f, &m);
    }
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used > 0u, "barometric height was fused");
    CHECK_TRUE(f.log_state.t_last_baro_height_aid != 0, "last aiding time recorded");
    CHECK_TRUE(f.log_state.t_last_baro_height_warn == 0, "no gap reported while streaming");

    const uint32_t used_at_gap_start = ins_get_diag(&f)->n_baro_height_used;

    /* Barometer dies. GNSS keeps coming, so the horizontal solution and
       every other health signal stay perfectly happy. */
    for (i = 0; i < 3000; ++i) /* 30 s */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 100) == 0)
        {
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
            m.gnss_pos.is_valid                                                   = true;
        }
        ins_update(&f, &m);
    }
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used == used_at_gap_start,
               "no barometric height fused during the gap");
    CHECK_TRUE(f.log_state.t_last_baro_height_warn != 0, "aiding gap reported");
    /* The point of the diagnostic: nothing else would have said anything. */
    CHECK_TRUE(ins_is_ready(&f), "the filter itself stays ready (horizontal aiding is fine)");
    CHECK_TRUE(f.height_from_baro, "source NOT switched back to GNSS (REQ-NAV-053)");

    /* Resuming clears the throttle so a later gap is reported again. */
    for (i = 0; i < 100; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(0.0f), 1.0f);
        ins_update(&f, &m);
    }
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used > used_at_gap_start,
               "barometric height fused again after the gap");
    CHECK_TRUE(f.log_state.t_last_baro_height_warn == 0, "gap state cleared on resume");

    /* The sharpest case: the barometer delivers during the collecting window
       (fresh enough to be selected, so the bootstrap latches it) and then
       never again, so there is never a FIRST fusion. The gap clock therefore
       has to start at the bootstrap, not at the first fusion, or this case
       -- the one where the vertical channel has no absolute reference for
       the whole mission -- would report nothing at all. */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        ins_options_t opt2;
        fill_default_opt(&opt2);
        opt2.auto_init = true;
        CHECK_TRUE(ins_init(&f2, &init, &opt2) == 0, "init ok");

        ins_time_us_t t2 = 0;
        baro_height_bootstrap(&f2, &init, &t2, acc_body, gyr_body, dt, true, false);
        CHECK_TRUE(f2.is_initialized && f2.height_from_baro, "barometric height selected");
        CHECK_TRUE(f2.log_state.t_last_baro_height_aid != 0,
                   "gap clock seeded at the bootstrap, before any fusion");

        for (i = 0; i < 2000; ++i) /* 20 s, GNSS only, barometer gone for good */
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            if ((i % 100) == 0)
            {
                m2.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
                m2.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
                m2.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
                m2.gnss_pos.Qll_ned[0] = m2.gnss_pos.Qll_ned[4] = m2.gnss_pos.Qll_ned[8] = 1.0f;
                m2.gnss_pos.is_valid                                                     = true;
            }
            ins_update(&f2, &m2);
        }
        CHECK_TRUE(ins_get_diag(&f2)->n_baro_height_used == 0u, "no barometric height ever fused");
        CHECK_TRUE(f2.log_state.t_last_baro_height_warn != 0,
                   "gap still reported although no fusion ever happened");
    }
}

/* @satisfies REQ-NAV-026 */
static void scenario_initial_yaw_stddev_override(void)
{
    printf("\n=== Scenario: rpy_init_stddev_rad[2] overrides yaw variance "
           "independently of rpy_init_stddev_rad[0] ===\n");

    /* Part 1: manual init. Roll/pitch stay tight (5 deg, the default),
       yaw is set much looser (20 deg). Must not leak into roll/pitch. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.rpy_init_rad[0]        = DEG2RAD(3.0f);
        init.rpy_init_rad[1]        = DEG2RAD(-2.0f);
        init.rpy_init_rad[2]        = DEG2RAD(40.0f);
        init.rpy_init_stddev_rad[2] = DEG2RAD(20.0f);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        /* Deferred start (REQ-NAV-033): apply the prescribed init with one
           IMU epoch (allow_unlimited_deadreckoning -> IMU alone). The finalize
           epoch does not fuse, so the covariance is the pristine init diag. */
        ins_time_us_t t0 = 0;
        start_manual_filter(&f, &t0, 0.01f, (const double*)0);

        const float roll_var   = f.d[INS_IDX_RPY + 0];
        const float pitch_var  = f.d[INS_IDX_RPY + 1];
        const float yaw_var    = f.d[INS_IDX_RPY + 2];
        const float expect_rp  = (init.rpy_init_stddev_rad[0] * init.rpy_init_stddev_rad[0]);
        const float expect_yaw = (init.rpy_init_stddev_rad[2] * init.rpy_init_stddev_rad[2]);

        CHECK_NEAR(roll_var, expect_rp, 1e-9, "manual init roll variance unaffected");
        CHECK_NEAR(pitch_var, expect_rp, 1e-9, "manual init pitch variance unaffected");
        CHECK_NEAR(yaw_var, expect_yaw, 1e-9, "manual init yaw variance overridden");
    }

    /* Part 2: auto-init's magnetometer-derived yaw. Same setup as
       scenario_autoinit_mag_yaw, but with rpy_init_stddev_rad[2] set:
       the resulting yaw variance must follow it, not rpy_init_stddev_rad[0]. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.rpy_init_stddev_rad[2] = DEG2RAD(20.0f);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        float g_vec[3];
        ins_gravity_ned((float)lat, (float)h, g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 10; ++step)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        const float yaw_true = DEG2RAD(25.0f);
        const float mh       = sqrtf(init.magnetic_n[0] * init.magnetic_n[0] +
                                     init.magnetic_n[1] * init.magnetic_n[1]);

        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]          = mh * cosf(-yaw_true);
        m.mag.data[1]          = mh * sinf(-yaw_true);
        m.mag.data[2]          = init.magnetic_n[2];
        m.mag.is_valid         = true;
        m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0]  = 1.0f;
        m.gnss_pos.Qll_ned[4]  = 1.0f;
        m.gnss_pos.Qll_ned[8]  = 1.0f;
        m.gnss_pos.is_valid    = true;
        ins_update(&f, &m);

        if (!f.is_initialized)
        {
            printf("  FAIL  filter did not bootstrap\n");
            fails++;
            return;
        }

        const float yaw_var    = f.d[INS_IDX_RPY + 2];
        const float expect_yaw = (init.rpy_init_stddev_rad[2] * init.rpy_init_stddev_rad[2]);
        const float wrong_yaw  = (init.rpy_init_stddev_rad[0] * init.rpy_init_stddev_rad[0]);
        CHECK_NEAR(yaw_var, expect_yaw, 1e-4,
                   "autoinit mag yaw variance follows rpy_init_stddev_rad[2]");
        if (fabsf(yaw_var - wrong_yaw) < 1e-4f)
        {
            printf("  FAIL  yaw variance still matches rpy_init_stddev_rad[0] "
                   "(override not applied)\n");
            fails++;
        }
        else
        {
            printf("  ok    yaw variance does not match rpy_init_stddev_rad[0] "
                   "(override applied)\n");
        }
    }
}

static void scenario_autoinit_yaw_priority(void)
{
    printf("\n=== Scenario 17: auto-init prefers an external yaw fix over the "
           "magnetometer ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    const float yaw_ext = 40.0f * (float)M_PI / 180.0f;

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    /* Magnetometer reading corresponds to yaw = 0 deg. If it were used,
       the bootstrap yaw would come out at 0, not yaw_ext. */
    m.mag.data[0]          = init.magnetic_n[0];
    m.mag.data[1]          = init.magnetic_n[1];
    m.mag.data[2]          = init.magnetic_n[2];
    m.mag.is_valid         = true;
    m.yaw.yaw_rad          = yaw_ext;
    m.yaw.stddev_rad       = 2.0f * (float)M_PI / 180.0f;
    m.yaw.is_valid         = true;
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter did not bootstrap\n");
        fails++;
        return;
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, yaw_ext, 1e-3, "autoinit yaw uses external fix, not mag");

    const float yaw_var    = f.d[INS_IDX_RPY + 2];
    const float expect_var = m.yaw.stddev_rad * m.yaw.stddev_rad;
    CHECK_NEAR(yaw_var, expect_var, 1e-6, "autoinit yaw variance from external fix");
}

/* Same as scenario_autoinit_yaw_priority, but the external yaw fix has a
 * non-positive stddev (MC/DC: ins_autoinit_yaw()'s external-yaw arm is
 * "is_valid && stddev_rad > 0.0f", never sensitized false elsewhere) -- it
 * must be treated as absent and fall through to the magnetometer heading. */
static void scenario_autoinit_yaw_zero_stddev_falls_through(void)
{
    printf("\n=== Scenario: auto-init yaw with stddev <= 0 falls through to mag "
           "(MC/DC) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    /* Magnetometer reading corresponds to yaw = 0 deg. */
    m.mag.data[0]  = init.magnetic_n[0];
    m.mag.data[1]  = init.magnetic_n[1];
    m.mag.data[2]  = init.magnetic_n[2];
    m.mag.is_valid = true;
    /* External yaw present but stddev <= 0: must be ignored. */
    m.yaw.yaw_rad          = 40.0f * (float)M_PI / 180.0f;
    m.yaw.stddev_rad       = 0.0f;
    m.yaw.is_valid         = true;
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    CHECK_TRUE(f.is_initialized, "filter bootstrapped");
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, 0.0, 0.05, "stddev <= 0: falls through to the mag heading (~0 deg)");
}

/* Auto-init no longer defers the bootstrap while the leveling window is not
   quasi-static (REQ-NAV-047): a platform that never sits still must still
   be able to auto-init. Instead the reported roll/pitch stddev is widened
   whenever the window is rotating and/or accelerating, and stays at the
   configured baseline while quasi-static. */
/* @satisfies REQ-NAV-047 */
static void scenario_autoinit_moving_rpy_stddev(void)
{
    printf("\n=== Scenario 18: auto-init bootstraps while moving, with "
           "inflated roll/pitch stddev (REQ-NAV-047) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_level[3] = {0.0f, 0.0f, -g_vec[2]};
    /* Well above the default static-accel gate (0.2 m/s^2 vs. gravity). */
    const float acc_moving[3] = {5.0f, 0.0f, -g_vec[2]};
    /* Well above the default static-gyro gate (0.01 rad/s). */
    const float gyr_spin[3]  = {0.0f, 0.0f, 1.0f};
    const float gyr_still[3] = {0.0f, 0.0f, 0.0f};
    const float dt           = 0.01f;

#define FEED_FIX_EPOCHS(FLT, T, ACC, GYR, N)                                                 \
    do {                                                                                     \
        int _step;                                                                           \
        for (_step = 0; _step < (N); ++_step)                                                \
        {                                                                                    \
            (T) += us_from_sec(dt);                                                          \
            ins_measurements_t _m;                                                           \
            memset(&_m, 0, sizeof(_m));                                                      \
            _m.timestamp = (T);                                                              \
            set_imu(&_m, (ACC), (GYR), dt);                                                  \
            _m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];                                        \
            _m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];                                        \
            _m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];                                        \
            _m.gnss_pos.Qll_ned[0] = _m.gnss_pos.Qll_ned[4] = _m.gnss_pos.Qll_ned[8] = 1.0f; \
            _m.gnss_pos.is_valid                                                     = true; \
            ins_update(&(FLT), &_m);                                                         \
        }                                                                                    \
    } while (0)

    float rp_var_static;

    /* Baseline: quasi-static window -> exactly the configured stddev. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        FEED_FIX_EPOCHS(f, t, acc_level, gyr_still, 5);
        CHECK_TRUE(f.is_initialized, "bootstraps while static");
        rp_var_static = f.d[INS_IDX_RPY + 0];
        CHECK_NEAR(sqrtf(rp_var_static), init.rpy_init_stddev_rad[0], 1e-4,
                   "static bootstrap keeps the configured baseline roll stddev [rad]");
        CHECK_NEAR(f.d[INS_IDX_RPY + 1], rp_var_static, 1e-9,
                   "pitch variance matches roll (same baseline)");
        CHECK_TRUE(ins_get_diag(&f)->n_autoinit_moving == 0, "not counted as a moving bootstrap");
    }

    /* Rotating throughout: must still bootstrap immediately (no deferral),
       with an inflated roll/pitch variance. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        FEED_FIX_EPOCHS(f, t, acc_level, gyr_spin, 5);
        CHECK_TRUE(f.is_initialized, "bootstraps immediately while still rotating");
        const float rp_var_spin = f.d[INS_IDX_RPY + 0];
        CHECK_TRUE(rp_var_spin > rp_var_static,
                   "rotating bootstrap reports a larger roll/pitch variance than static");
        CHECK_TRUE(sqrtf(rp_var_spin) >= (15.0f * (float)M_PI / 180.0f) - 1e-6f,
                   "rotating roll/pitch stddev is not too small (>= the moving floor) [rad]");
        CHECK_TRUE(ins_get_diag(&f)->n_autoinit_moving == 1, "counted as a moving bootstrap");
    }

    /* Accelerating throughout: same story via the accel gate. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        FEED_FIX_EPOCHS(f, t, acc_moving, gyr_still, 5);
        CHECK_TRUE(f.is_initialized, "bootstraps immediately while accelerating");
        const float rp_var_acc = f.d[INS_IDX_RPY + 0];
        CHECK_TRUE(rp_var_acc > rp_var_static,
                   "accelerating bootstrap reports a larger roll/pitch variance than static");
        CHECK_TRUE(sqrtf(rp_var_acc) >= (15.0f * (float)M_PI / 180.0f) - 1e-6f,
                   "accelerating roll/pitch stddev is not too small (>= the moving floor) [rad]");
        CHECK_TRUE(ins_get_diag(&f)->n_autoinit_moving == 1, "counted as a moving bootstrap");
    }

#undef FEED_FIX_EPOCHS
}

/* An external attitude/gyro-bias hint (REQ-NAV-048) overrides auto-init's
 * own accelerometer leveling / yaw-unknown / zero-bias assumption when
 * supplied on the bootstrap epoch. The IMU feed levels to a DIFFERENT
 * roll/pitch (same physical setup as scenario_autoinit_gnss) than the
 * hint states, so the assertions below unambiguously show which one won. */
static void scenario_autoinit_att_hint(void)
{
    printf("\n=== Scenario: auto-init attitude/gyro-bias hint (REQ-NAV-048) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;
    /* Cold-start gyro-bias prior kept looser than the 0.001 rad/s the hint
       below carries, so REQ-NAV-067's clamp is a no-op here and this
       scenario keeps testing REQ-NAV-048's plain "hint wins" behaviour.
       The clamp itself has its own scenario. */
    init.gyr_bias_init_stddev_rps = 0.01f;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* IMU truth: levels to roll=15, pitch=-8 deg without a hint. */
    const float roll_level  = 15.0f * (float)M_PI / 180.0f;
    const float pitch_level = -8.0f * (float)M_PI / 180.0f;
    float       q_true[4], R_true[9];
    ins_quat_from_rpy(roll_level, pitch_level, 0.0f, q_true);
    ins_quat_to_rotmat(q_true, R_true);

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    float acc_body[3];
    int   i;
    for (i = 0; i < 3; ++i)
    {
        acc_body[i] = -(R_true[i * 3 + 0] * g_vec[0] + R_true[i * 3 + 1] * g_vec[1] +
                        R_true[i * 3 + 2] * g_vec[2]);
    }
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* Hint disagrees with the leveling result and with the "yaw unknown" /
       zero-bias defaults. */
    const float roll_hint     = 25.0f * (float)M_PI / 180.0f;
    const float pitch_hint    = -15.0f * (float)M_PI / 180.0f;
    const float yaw_hint      = 40.0f * (float)M_PI / 180.0f;
    const float gbias_hint[3] = {0.01f, -0.02f, 0.005f}; /* rad/s */

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 20; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0]      = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1]      = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2]      = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]       = 1.0f;
    m.gnss_pos.Qll_ned[4]       = 1.0f;
    m.gnss_pos.Qll_ned[8]       = 1.0f;
    m.gnss_pos.is_valid         = true;
    m.att_hint.is_valid         = true;
    m.att_hint.roll_rad         = roll_hint;
    m.att_hint.pitch_rad        = pitch_hint;
    m.att_hint.stddev_roll_rad  = 0.05f;
    m.att_hint.stddev_pitch_rad = 0.06f;
    m.att_hint.yaw_rad          = yaw_hint;
    m.att_hint.stddev_yaw_rad   = 0.1f;
    for (i = 0; i < 3; ++i)
    {
        m.att_hint.gyr_bias_rps[i]        = gbias_hint[i];
        m.att_hint.stddev_gyr_bias_rps[i] = 0.001f;
    }
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter did not bootstrap on the hinted fix\n");
        fails++;
        return;
    }

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, roll_hint, 1e-4, "att_hint roll wins over leveling");
    CHECK_NEAR(pitch, pitch_hint, 1e-4, "att_hint pitch wins over leveling");
    CHECK_NEAR(yaw, yaw_hint, 1e-4, "att_hint yaw wins over 'unknown'");

    const float rp_var_expect = 0.06f * 0.06f; /* max(0.05, 0.06)^2 */
    CHECK_NEAR(f.d[INS_IDX_RPY + 0], rp_var_expect, 1e-6,
               "att_hint roll variance = max(stddevs)^2");
    CHECK_NEAR(f.d[INS_IDX_RPY + 1], rp_var_expect, 1e-6,
               "att_hint pitch variance = max(stddevs)^2");
    CHECK_NEAR(f.d[INS_IDX_RPY + 2], 0.1f * 0.1f, 1e-6, "att_hint yaw variance");

    float gbias[3];
    ins_get_bias_gyr(&f, gbias);
    CHECK_NEAR(gbias[0], gbias_hint[0], 1e-6, "att_hint gyro bias x");
    CHECK_NEAR(gbias[1], gbias_hint[1], 1e-6, "att_hint gyro bias y");
    CHECK_NEAR(gbias[2], gbias_hint[2], 1e-6, "att_hint gyro bias z");
    CHECK_NEAR(f.d[INS_IDX_GYR + 0], 0.001f * 0.001f, 1e-9, "att_hint gyro bias x variance");

    /* --- Regression: without a hint, behaviour is unchanged (own leveling) --- */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        ins_init_t init2;
        fill_default_init(&init2, 0);
        ins_options_t opt2;
        fill_default_opt(&opt2);
        opt2.auto_init = true;
        if (ins_init(&f2, &init2, &opt2) != 0)
        {
            printf("init2 failed\n");
            fails++;
            return;
        }

        ins_time_us_t t2 = 0;
        for (step = 1; step <= 20; ++step)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            ins_update(&f2, &m2);
        }
        t2 += us_from_sec(dt);
        ins_measurements_t m2;
        memset(&m2, 0, sizeof(m2));
        m2.timestamp = t2;
        set_imu(&m2, acc_body, gyr_body, dt);
        m2.gnss_pos.xyz_ecef[0] = init2.x_ecef[0];
        m2.gnss_pos.xyz_ecef[1] = init2.x_ecef[1];
        m2.gnss_pos.xyz_ecef[2] = init2.x_ecef[2];
        m2.gnss_pos.Qll_ned[0]  = 1.0f;
        m2.gnss_pos.Qll_ned[4]  = 1.0f;
        m2.gnss_pos.Qll_ned[8]  = 1.0f;
        m2.gnss_pos.is_valid    = true;
        /* m2.att_hint left zeroed (is_valid == false) */
        ins_update(&f2, &m2);

        float roll2, pitch2, yaw2;
        ins_get_rpy(&f2, &roll2, &pitch2, &yaw2);
        CHECK_NEAR(roll2, roll_level, 0.02, "no hint: falls back to leveling roll");
        CHECK_NEAR(pitch2, pitch_level, 0.02, "no hint: falls back to leveling pitch");
        float gbias2[3];
        ins_get_bias_gyr(&f2, gbias2);
        CHECK_NEAR(gbias2[0], 0.0f, 1e-9, "no hint: gyro bias stays at prescribed init value");
    }
}

/* Drive a hinted auto-init bootstrap and report the gyro-bias variance the
 * hint installed, per axis. hint_sd is the hint's 1-sigma on every axis,
 * prior_sd the configured cold-start prior. */
static void att_hint_bootstrap_gyr_var(float prior_sd, float hint_sd, const float gbias_hint[3],
                                       float var_out[3], float bias_out[3])
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init                 = true;
    init.gyr_bias_init_stddev_rps = prior_sd;

    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("  FAIL  init failed\n");
        fails++;
        return;
    }

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step, i;
    for (step = 1; step <= 20; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t mi;
        memset(&mi, 0, sizeof(mi));
        mi.timestamp = t;
        set_imu(&mi, acc_body, gyr_body, dt);
        ins_update(&f, &mi);
    }

    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
    m.gnss_pos.Qll_ned[0]       = 1.0f;
    m.gnss_pos.Qll_ned[4]       = 1.0f;
    m.gnss_pos.Qll_ned[8]       = 1.0f;
    m.gnss_pos.is_valid         = true;
    m.att_hint.is_valid         = true;
    m.att_hint.roll_rad         = 0.0f;
    m.att_hint.pitch_rad        = 0.0f;
    m.att_hint.stddev_roll_rad  = 0.05f;
    m.att_hint.stddev_pitch_rad = 0.05f;
    for (i = 0; i < 3; ++i)
    {
        m.att_hint.gyr_bias_rps[i]        = gbias_hint[i];
        m.att_hint.stddev_gyr_bias_rps[i] = hint_sd;
    }
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter did not bootstrap on the hinted fix\n");
        fails++;
        return;
    }
    for (i = 0; i < 3; ++i) { var_out[i] = f.d[INS_IDX_GYR + i]; }
    ins_get_bias_gyr(&f, bias_out);
}

static void scenario_att_hint_gyr_bias_prior_cap(void)
{
    printf("\n=== Scenario: gyro-bias hint clamped to the cold-start prior "
           "(REQ-NAV-067) ===\n");

    /* rad/s; dflt_sd is the library's default cold-start gyro-bias prior. */
    const float gbias_hint[3] = {0.01f, -0.02f, 0.005f};
    const float dflt_sd       = 1.0f * (float)M_PI / 180.0f;
    float       var[3], bias[3];
    int         i;

    /* Loose hint against a prior BELOW the library default: the cap is the
       default, not the configured prior. nav_suite hands ins its ARS
       1-sigma widened by a fixed factor (REQ-SUITE-016), and on the gyro-z
       axis an ARS without a heading reference observes nothing, so what it
       reports is its own init prior. Observed on datasets/tunnel at
       14.8 deg/s against a configured 0.5 deg/s. */
    {
        const float prior_sd = 0.5f * (float)M_PI / 180.0f;
        const float loose_sd = 14.8f * (float)M_PI / 180.0f;
        att_hint_bootstrap_gyr_var(prior_sd, loose_sd, gbias_hint, var, bias);
        for (i = 0; i < 3; ++i)
        {
            CHECK_NEAR(var[i], dflt_sd * dflt_sd, 1e-9,
                       "loose hint clamped, floored at the library default");
            /* The cap bounds the confidence only, never the value. */
            CHECK_NEAR(bias[i], gbias_hint[i], 1e-6, "loose hint: bias value still adopted");
        }
    }

    /* Same hint against a prior ABOVE the library default: a deliberately
       wide configured prior wins over the floor, so the cap follows the
       config rather than overriding it. */
    {
        const float prior_sd = 3.0f * (float)M_PI / 180.0f;
        const float loose_sd = 14.8f * (float)M_PI / 180.0f;
        att_hint_bootstrap_gyr_var(prior_sd, loose_sd, gbias_hint, var, bias);
        for (i = 0; i < 3; ++i)
        {
            CHECK_NEAR(var[i], prior_sd * prior_sd, 1e-9,
                       "loose hint clamped to a wider configured prior");
        }
    }

    /* Tight hint: below the cap, so it must not be touched. A well-scaled
       hint sees exactly the pre-REQ-NAV-067 behaviour. */
    {
        const float prior_sd = 0.5f * (float)M_PI / 180.0f;
        const float tight_sd = 0.05f * (float)M_PI / 180.0f;
        att_hint_bootstrap_gyr_var(prior_sd, tight_sd, gbias_hint, var, bias);
        for (i = 0; i < 3; ++i)
        {
            CHECK_NEAR(var[i], tight_sd * tight_sd, 1e-12, "tight hint installed unchanged");
            CHECK_NEAR(bias[i], gbias_hint[i], 1e-6, "tight hint: bias value adopted");
        }
    }

    /* Boundary: a hint exactly at the cap is not clamped either. */
    {
        const float prior_sd = 0.5f * (float)M_PI / 180.0f;
        att_hint_bootstrap_gyr_var(prior_sd, dflt_sd, gbias_hint, var, bias);
        for (i = 0; i < 3; ++i)
        {
            CHECK_NEAR(var[i], dflt_sd * dflt_sd, 1e-12, "hint at the cap installed unchanged");
        }
    }
}

/* Manual-init a filter in level, unaccelerated cruise at vel_ned and run
 * it far enough to be live. Returns the epoch reached via t_io. */
static bool speed_test_start(ins_t* f, ins_time_us_t* t_io, const float vel_ned[3],
                             float speed_scale, float speed_rel, float speed_min)
{
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.speed_scale      = speed_scale;
    opt.speed_stddev_rel = speed_rel;
    opt.speed_min_mps    = speed_min;
    /* A noiseless IMU in constant-velocity cruise is exactly what the
       variance-based stillness detector (REQ-NAV-013) reads as a
       standstill, and its ZUPTs would drag the velocity to zero before
       the first speed sample arrives. This scenario is about the speed
       channel, not about that detector. */
    opt.auto_zupt_disable = true;

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float R0[9];
    ins_rotmat_n_to_e(lat, lon, R0);

    int k;
    for (k = 0; k < 3; ++k)
    {
        /* columns of R_n_to_e are d(ecef)/d(N,E,D) */
        init.xdot_ecef[k] = R0[k] * vel_ned[0] + R0[3 + k] * vel_ned[1] + R0[6 + k] * vel_ned[2];
    }

    memset(f, 0, sizeof(*f));
    if (ins_init(f, &init, &opt) != 0) { return false; }

    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]}; /* level, no acceleration */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float dt = 0.01f;
    int         step;
    for (step = 0; step < 200; ++step)
    {
        *t_io += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = *t_io;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(f, &m);
    }
    return f->is_initialized;
}

/* One epoch carrying a speed sample on an already-running filter. */
static void speed_test_feed(ins_t* f, ins_time_us_t* t_io, float speed_mps, float stddev_mps,
                            int delay_ms)
{
    float g_vec[3];
    ins_gravity_ned((float)(48.783 * M_PI / 180.0), 300.0f, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float dt = 0.01f;
    *t_io += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = *t_io;
    set_imu(&m, acc_body, gyr_body, dt);
    m.speed.speed_mps  = speed_mps;
    m.speed.stddev_mps = stddev_mps;
    m.speed.is_valid   = true;
    m.speed_delay_ms   = delay_ms;
    ins_update(f, &m);
}

static float speed_test_norm(const ins_t* f)
{
    float v[3];
    ins_get_velocity_ned(f, v);
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void scenario_speed_aiding(void)
{
    printf("\n=== Scenario: absolute speed aiding (REQ-NAV-068) ===\n");

    /* Deliberately off-axis, so a correction along v_hat is distinguishable
       from one along a single NED axis. ||v|| = 13. */
    const float vel_ned[3] = {12.0f, 5.0f, 0.0f};

    /* --- the filter runs 2 m/s fast: the reading pulls it back --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        if (!speed_test_start(&f, &t, vel_ned, 0.0f, 0.0f, 0.0f))
        {
            printf("  FAIL  filter did not start\n");
            fails++;
            return;
        }
        const float before = speed_test_norm(&f);
        CHECK_NEAR(before, 13.0f, 0.05, "cruise speed before aiding");

        speed_test_feed(&f, &t, before - 2.0f, 0.0f, 0);
        const float after = speed_test_norm(&f);

        CHECK_TRUE(ins_get_diag(&f)->n_speed_seen == 1, "speed sample counted as seen");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 1, "speed sample fused");
        CHECK_TRUE(after < before, "speed pulled towards the slower reading");
        /* Partial, not a snap: R > 0, so it must not reach the reading. */
        CHECK_TRUE(after > before - 2.0f, "correction is partial, not a jump to the reading");
        CHECK_NEAR(ins_get_diag(&f)->last_speed_residual_mps, 2.0f, 0.05,
                   "residual is ||v_n|| - z");
    }

    /* --- and symmetrically when it runs slow --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        if (!speed_test_start(&f, &t, vel_ned, 0.0f, 0.0f, 0.0f)) { return; }
        const float before = speed_test_norm(&f);
        speed_test_feed(&f, &t, before + 2.0f, 0.0f, 0);
        CHECK_TRUE(speed_test_norm(&f) > before, "speed pulled towards the faster reading");
    }

    /* --- standstill gate: v_hat is not conditioned, so nothing is fused --- */
    {
        ins_t         f;
        ins_time_us_t t        = 0;
        const float   crawl[3] = {0.2f, 0.0f, 0.0f};
        if (!speed_test_start(&f, &t, crawl, 0.0f, 0.0f, 1.0f)) { return; }
        speed_test_feed(&f, &t, 5.0f, 0.0f, 0);
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 0, "below the gate: not fused");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_skipped == 1, "below the gate: counted as skipped");
    }

    /* --- unsigned contract: a negative reading is an input error --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        if (!speed_test_start(&f, &t, vel_ned, 0.0f, 0.0f, 0.0f)) { return; }
        const uint32_t bad_before = ins_get_diag(&f)->n_invalid_input;
        speed_test_feed(&f, &t, -5.0f, 0.0f, 0);
        CHECK_TRUE(ins_get_diag(&f)->n_invalid_input == bad_before + 1,
                   "negative speed counted as invalid input");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 0, "negative speed not fused");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_seen == 0, "negative speed not counted as seen");
    }

    /* --- scale calibration: a reading scaled back up leaves no residual --- */
    {
        ins_t         f;
        ins_time_us_t t     = 0;
        const float   scale = 1.05f;
        if (!speed_test_start(&f, &t, vel_ned, scale, 0.0f, 0.0f)) { return; }
        const float before = speed_test_norm(&f);
        /* A speedometer reading 5% high against a filter that is right. */
        speed_test_feed(&f, &t, before / scale, 0.0f, 0);
        CHECK_NEAR(ins_get_diag(&f)->last_speed_residual_mps, 0.0f, 0.02,
                   "speed_scale removes the calibrated offset");
    }

    /* --- relative stddev: the same absolute residual moves the filter less
           at high speed, because R grows with the reading --- */
    {
        ins_t         f_slow, f_fast;
        ins_time_us_t t_slow = 0, t_fast = 0;
        const float   slow[3] = {12.0f, 5.0f, 0.0f};   /* 13 m/s */
        const float   fast[3] = {120.0f, 50.0f, 0.0f}; /* 130 m/s */
        if (!speed_test_start(&f_slow, &t_slow, slow, 0.0f, 0.03f, 0.0f)) { return; }
        if (!speed_test_start(&f_fast, &t_fast, fast, 0.0f, 0.03f, 0.0f)) { return; }

        const float s0 = speed_test_norm(&f_slow);
        const float f0 = speed_test_norm(&f_fast);
        speed_test_feed(&f_slow, &t_slow, s0 - 2.0f, 0.0f, 0);
        speed_test_feed(&f_fast, &t_fast, f0 - 2.0f, 0.0f, 0);

        const float d_slow = s0 - speed_test_norm(&f_slow);
        const float d_fast = f0 - speed_test_norm(&f_fast);
        CHECK_TRUE(d_fast < d_slow,
                   "speed-proportional variance shrinks the correction at high speed");
    }

    /* --- a delay beyond the history horizon is skipped, not guessed --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        if (!speed_test_start(&f, &t, vel_ned, 0.0f, 0.0f, 0.0f)) { return; }
        speed_test_feed(&f, &t, 11.0f, 0.0f, INS_MAX_DELAY_MS + 1);
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 0, "over-aged sample not fused");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_skipped == 1, "over-aged sample counted as skipped");
    }

    /* --- a delayed sample is anchored in the history, not on "now" --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        if (!speed_test_start(&f, &t, vel_ned, 0.0f, 0.0f, 0.0f)) { return; }
        speed_test_feed(&f, &t, speed_test_norm(&f) - 2.0f, 0.0f, 50);
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 1, "delayed sample fused from the history");
    }
}

/* -------------------------------------------------------------------------- */
/* Accessors, lifecycle and error-path scenarios                             */
/* -------------------------------------------------------------------------- */

static void scenario_accessors_and_lifecycle(void)
{
    printf("\n=== Scenario 19: accessors, shutdown, world model, is_ready "
           "timing ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* exercise the timed is_ready gate */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    if (ins_is_ready(&f))
    {
        printf("  FAIL  is_ready true right after init\n");
        fails++;
    }
    else { printf("  ok    is_ready false right after init\n"); }

    /* Gravity from the init position (f.latlonh is only set at start now). */
    double ilat, ilon, ih;
    ins_ecef_to_latlonh(init.x_ecef, &ilat, &ilon, &ih);
    float g_vec[3];
    ins_gravity_ned((float)ilat, (float)ih, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 200; ++step) /* 2 s: past both is_ready gates */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        /* First epoch carries the startup fix so the limited-DR filter
           crosses the gate (REQ-NAV-033); keeps the update count at 200. */
        if (step == 1)
        {
            m.gnss_pos.is_valid    = true;
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        ins_update(&f, &m);
    }

    if (!ins_is_ready(&f))
    {
        printf("  FAIL  is_ready still false after enough epochs/runtime\n");
        fails++;
    }
    else { printf("  ok    is_ready true once epochs+runtime gates are cleared\n"); }

    float v_ecef[3];
    if (!ins_get_velocity_ecef(&f, v_ecef))
    {
        printf("  FAIL  get_velocity_ecef\n");
        fails++;
    }
    else
        CHECK_NEAR(test_vec3_norm(v_ecef), 0.0f, 0.05, "velocity_ecef ~ 0 (stationary)");

    float q[4];
    if (!ins_get_quaternion(&f, q))
    {
        printf("  FAIL  get_quaternion\n");
        fails++;
    }
    else
        CHECK_NEAR(q[0], 1.0f, 0.01, "quaternion w ~ 1 (level, no yaw)");

    float R[9];
    if (!ins_get_rotmat_b_to_n(&f, R))
    {
        printf("  FAIL  get_rotmat_b_to_n\n");
        fails++;
    }
    else
        CHECK_NEAR(R[0], 1.0f, 0.01, "R_b_to_n(0,0) ~ 1 (near-identity)");

    float bacc[3];
    if (!ins_get_bias_acc(&f, bacc))
    {
        printf("  FAIL  get_bias_acc\n");
        fails++;
    }
    else
        printf("  ok    get_bias_acc returned\n");

    float w[3];
    if (!ins_get_omega_b_nb(&f, w))
    {
        printf("  FAIL  get_omega_b_nb\n");
        fails++;
    }
    else
        CHECK_NEAR(test_vec3_norm(w), 0.0f, 0.01, "omega_b_nb ~ 0 (stationary)");

    float acc_n[3];
    if (!ins_get_acc_n(&f, acc_n))
    {
        printf("  FAIL  get_acc_n\n");
        fails++;
    }
    else
        CHECK_NEAR(test_vec3_norm(acc_n), 0.0f, 0.05, "acc_n ~ 0 (stationary, gravity removed)");

    const ins_diag_t* diag = ins_get_diag(&f);
    if (diag == (const ins_diag_t*)0)
    {
        printf("  FAIL  get_diag returned NULL\n");
        fails++;
    }
    else
    {
        CHECK_NEAR(diag->n_updates, 200, 0.5, "diag n_updates");
        if (diag->n_predict > 0)
            printf("  ok    diag n_predict > 0\n");
        else
        {
            printf("  FAIL  diag n_predict == 0\n");
            fails++;
        }
    }

    /* set_world_model overrides gravity_n/magnetic_n; NULL leaves a field
       untouched. */
    const float new_grav[3] = {0.0f, 0.0f, 9.5f};
    const float new_mag[3]  = {10.0f, 1.0f, 40.0f};
    ins_set_world_model(&f, new_grav, new_mag);
    CHECK_NEAR(f.gravity_n[2], 9.5f, 1e-6, "set_world_model gravity_n");
    CHECK_NEAR(f.magnetic_n[0], 10.0f, 1e-6, "set_world_model magnetic_n");
    ins_set_world_model(&f, (const float*)0, new_mag);
    CHECK_NEAR(f.gravity_n[2], 9.5f, 1e-6, "set_world_model NULL leaves gravity_n");

    /* shutdown must make the filter report un-initialized everywhere. */
    ins_shutdown(&f);
    if (f.is_initialized || ins_is_ready(&f) || ins_get_position_local(&f, v_ecef) ||
        ins_get_velocity_ecef(&f, v_ecef))
    {
        printf("  FAIL  filter still reports initialized after shutdown\n");
        fails++;
    }
    else { printf("  ok    shutdown de-initializes the filter\n"); }
}

static void scenario_time_jump_handling(void)
{
    printf("\n=== Scenario 20: time-jump handling (dropped / forced reset) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* forward jumps must reset */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    /* Start the (limited-DR) filter with an aligned IMU+GNSS epoch so the
       mid-stream time-jump paths below run on a running filter (REQ-NAV-033). */
    start_manual_filter(&f, &t, dt, init.x_ecef);

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    int step;
    for (step = 1; step <= 100; ++step) /* 1 s of normal operation */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* A mild backward jump (dt < 0 but not older than INS_MAX_DELAY_MS)
       must not drop or reset the filter: it just skips the IMU for that
       one epoch and counts it. */
    float pos_before[3];
    ins_get_position_local(&f, pos_before);
    ins_measurements_t m_mild;
    memset(&m_mild, 0, sizeof(m_mild));
    m_mild.timestamp = t - 50000; /* -50 ms */
    set_imu(&m_mild, acc_body, gyr_body, dt);
    ins_update(&f, &m_mild);

    const ins_diag_t* diag0 = ins_get_diag(&f);
    float             pos_after[3];
    ins_get_position_local(&f, pos_after);
    if (!f.is_initialized || diag0->n_time_backward < 1 || diag0->n_time_dropped != 0 ||
        diag0->n_time_jump_reset != 0)
    {
        printf("  FAIL  mild backward jump mishandled (init=%d, backward=%u, "
               "dropped=%u, reset=%u)\n",
               f.is_initialized, diag0->n_time_backward, diag0->n_time_dropped,
               diag0->n_time_jump_reset);
        fails++;
    }
    else { printf("  ok    mild backward jump skips IMU without drop/reset\n"); }
    CHECK_NEAR(pos_after[0], pos_before[0], 1e-6, "mild backward jump leaves position untouched");

    /* A bogus strapdown_dt_sec (caller-supplied, independent of the
       timestamp delta) beyond max_prediction_time_sec must make
       ins_strapdown skip the integration. The timestamp itself only
       advances by the normal 10 ms, so this is *not* a time jump. */
    ins_get_position_local(&f, pos_before);
    t += us_from_sec(dt);
    ins_measurements_t m_badstrap;
    memset(&m_badstrap, 0, sizeof(m_badstrap));
    m_badstrap.timestamp = t;
    set_imu(&m_badstrap, acc_body, gyr_body, dt);
    m_badstrap.strapdown_dt_sec = 5.0f; /* > max_prediction_time_sec (0.5 s) */
    ins_update(&f, &m_badstrap);
    ins_get_position_local(&f, pos_after);
    if (!f.is_initialized)
    {
        printf("  FAIL  bogus strapdown_dt_sec incorrectly triggered a reset\n");
        fails++;
    }
    else
    {
        CHECK_NEAR(pos_after[0], pos_before[0], 1e-6,
                   "out-of-range strapdown_dt_sec skips integration");
    }

    /* A measurement 600 ms in the past (> INS_MAX_DELAY_MS = 500) must be
       silently dropped: no crash, filter stays initialized and untouched. */
    ins_measurements_t m_old;
    memset(&m_old, 0, sizeof(m_old));
    m_old.timestamp = t - 600000; /* -600 ms */
    set_imu(&m_old, acc_body, gyr_body, dt);
    ins_update(&f, &m_old);

    const ins_diag_t* diag = ins_get_diag(&f);
    if (!f.is_initialized)
    {
        printf("  FAIL  filter reset on a merely-old (not fatal) measurement\n");
        fails++;
    }
    else if (diag->n_time_backward < 1 || diag->n_time_dropped < 1)
    {
        printf("  FAIL  n_time_backward/n_time_dropped not incremented (%u, %u)\n",
               diag->n_time_backward, diag->n_time_dropped);
        fails++;
    }
    else { printf("  ok    old measurement dropped, diag counters incremented\n"); }

    /* A large forward jump (well beyond max_prediction_time_sec) without
       allow_unlimited_deadreckoning must force a reset. */
    ins_measurements_t m_jump;
    memset(&m_jump, 0, sizeof(m_jump));
    m_jump.timestamp = t + 2000000; /* +2 s, max_prediction_time_sec = 0.5 s */
    set_imu(&m_jump, acc_body, gyr_body, dt);
    ins_update(&f, &m_jump);

    diag = ins_get_diag(&f);
    if (f.is_initialized)
    {
        printf("  FAIL  filter did not reset on a large forward time jump\n");
        fails++;
    }
    else if (diag->n_time_jump_reset < 1)
    {
        printf("  FAIL  n_time_jump_reset not incremented\n");
        fails++;
    }
    else { printf("  ok    large forward time jump forces a reset\n"); }
}

static void scenario_time_jump_unlimited_dr_recovery(void)
{
    printf("\n=== Scenario: forward time jump with unlimited dead reckoning recovers "
           "(REQ-NAV-016) ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = true; /* forward jumps must coast, not reset */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    start_manual_filter(&f, &t, dt, (const double*)0); /* IMU-only start (unlimited DR) */

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    int step;
    for (step = 1; step <= 50; ++step) /* 0.5 s of normal operation before the gap */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    const unsigned n_predict_before_gap = ins_get_diag(&f)->n_predict;

    /* A stalled IMU link: one sample arrives 2 s late, well beyond
       max_prediction_time_sec (0.5 s). With unlimited DR this must not
       reset the filter, and -- the actual regression -- must not
       permanently latch time_jump either: normal small-dt epochs
       resumed right after must be processed again, not skipped forever. */
    t += 2000000; /* +2 s gap */
    ins_measurements_t m_jump;
    memset(&m_jump, 0, sizeof(m_jump));
    m_jump.timestamp = t;
    set_imu(&m_jump, acc_body, gyr_body, dt);
    ins_update(&f, &m_jump);
    CHECK_TRUE(f.is_initialized, "unlimited-DR forward jump does not reset the filter");

    for (step = 1; step <= 20; ++step) /* resume normal-rate IMU right after the gap */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    const unsigned n_predict_after = ins_get_diag(&f)->n_predict;
    CHECK_TRUE(f.is_initialized, "filter still initialized after resuming post-gap");
    /* Before the fix, t_last_kalman_predict was never re-baselined on the
       skipped gap epoch, so dt_ms kept growing with every following epoch,
       time_jump latched permanently true and n_predict never moved again. */
    CHECK_TRUE(n_predict_after > n_predict_before_gap,
               "filter resumes predicting after the gap instead of staying stuck");
}

/* @satisfies REQ-NAV-033 */
static void scenario_startup_alignment_gate(void)
{
    printf("\n=== Scenario: startup stream-coherence gate (REQ-NAV-033) ===\n");

    const float dt          = 0.01f;
    const float acc_body[3] = {0.0f, 0.0f, -9.80665f}; /* value irrelevant to manual init */
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* (1) Limited dead reckoning: a prescribed init->time that predates the
       first IMU/GNSS epoch must not reset (the old code mistook the leading
       offset for a forward time jump). The filter consumes until an aligned
       IMU+GNSS pair, then starts, stamping its clock at that epoch. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0); /* init->time = 0 */
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.allow_unlimited_deadreckoning = false;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");
        CHECK_TRUE(!f.is_initialized && f.is_collecting, "consuming, not started after init");

        /* First IMU sample 0.8 s after init->time (> max_prediction_time). */
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = 800000;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
        CHECK_TRUE(!f.is_initialized, "IMU alone does not start a limited-DR filter");
        CHECK_TRUE(ins_get_diag(&f)->n_time_jump_reset == 0, "no spurious time-jump reset");

        /* Aligned IMU + GNSS -> start, clock stamped here (not at init->time). */
        const ins_time_us_t t_start = 803000;
        memset(&m, 0, sizeof(m));
        m.timestamp = t_start;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.is_valid = true;
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        ins_update(&f, &m);
        CHECK_TRUE(f.is_initialized, "starts on the first aligned IMU+GNSS epoch");
        CHECK_TRUE(f.t_init == t_start, "clock stamped at the aligned epoch, not init->time");
        CHECK_TRUE(ins_get_diag(&f)->n_time_jump_reset == 0, "started without any reset");
    }

    /* (2) A lone fix long before the IMU stream is temporally incoherent and
       must not bootstrap the filter; a fresh aligned pair does. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.allow_unlimited_deadreckoning = false;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        /* Lone GNSS fix (no IMU) at t = 1 s. */
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp         = 1000000;
        m.gnss_pos.is_valid = true;
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        ins_update(&f, &m);
        CHECK_TRUE(!f.is_initialized, "lone fix does not start the filter");

        /* IMU arrives 5 s later, far beyond max_prediction from the fix. */
        memset(&m, 0, sizeof(m));
        m.timestamp = 6000000;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
        CHECK_TRUE(!f.is_initialized, "IMU 5 s after a stale fix stays incoherent");

        /* Fresh aligned IMU + GNSS -> start. */
        memset(&m, 0, sizeof(m));
        m.timestamp = 6003000;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.is_valid = true;
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        ins_update(&f, &m);
        CHECK_TRUE(f.is_initialized, "starts once IMU and a fresh fix align");
    }

    /* (3) Unlimited dead reckoning: a prescribed init may start on the first
       IMU sample alone (pure DR), stamped at that sample, with no reset. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.allow_unlimited_deadreckoning = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");
        CHECK_TRUE(!f.is_initialized, "consuming before the first IMU sample");

        const ins_time_us_t t_start = 900000; /* 0.9 s > max_prediction_time */
        ins_measurements_t  m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t_start;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
        CHECK_TRUE(f.is_initialized, "unlimited-DR filter starts on the first IMU alone");
        CHECK_TRUE(f.t_init == t_start, "clock at the first IMU, not the stale init->time");
        CHECK_TRUE(ins_get_diag(&f)->n_time_jump_reset == 0, "no reset from the leading offset");
    }
}

/* @satisfies REQ-NAV-034 */
static void scenario_automotive_gnss_yaw(void)
{
    printf("\n=== Scenario: automotive yaw from GNSS course (REQ-NAV-034) ===\n");

    const float dt          = 0.05f; /* 20 Hz IMU */
    const float course_true = DEG2RAD(40.0f);
    const float speed       = 10.0f; /* m/s, well above the min */
    const float acc_body[3] = {0.0f, 0.0f, -9.80665f};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* Run `epochs` steps at 20 Hz, feeding a GNSS velocity (course/speed) at
       1 Hz. `automotive` and the speed let the caller exercise each branch.
       Level, stationary-looking IMU; yaw starts wrong (0) with a loose prior
       so a working course fusion can pull it to `course`.

       auto_zupt is disabled to isolate the course-yaw path. The same isolation
       applies to the manoeuvre-dependent velocity noise (REQ-NAV-073): this
       scenario feeds a stationary, level IMU next to a 20 m/s GNSS velocity on
       purpose, so the acceleration the term reads is an artefact of that
       contradiction rather than a manoeuvre, and pricing it here would only
       move the yaw the scenario measures. Hence the two acc scales are set to
       -1 to switch the manoeuvre pricing off. */
#define RUN_AUTOMOTIVE(F, AUTOMOTIVE, COURSE, SPD, MINSPD, YAWSTD, EPOCHS)                     \
    do {                                                                                       \
        ins_init_t init;                                                                       \
        fill_default_init(&init, 0);                                                           \
        init.rpy_init_stddev_rad[2] = DEG2RAD(20.0f);                                          \
        ins_options_t opt;                                                                     \
        fill_default_opt(&opt);                                                                \
        opt.auto_zupt_disable            = true;                                               \
        opt.gnss_vel_noise_acc_scale_hor = -1.0f;                                              \
        opt.gnss_vel_noise_acc_scale_ver = -1.0f;                                              \
        opt.automotive_mode              = (AUTOMOTIVE);                                       \
        opt.automotive_min_speed_mps     = (MINSPD);                                           \
        opt.automotive_min_yaw_stddev    = (YAWSTD);                                           \
        CHECK_TRUE(ins_init(&(F), &init, &opt) == 0, "init");                                  \
        ins_time_us_t t = 0;                                                                   \
        int           k;                                                                       \
        for (k = 0; k < (EPOCHS); ++k)                                                         \
        {                                                                                      \
            t += us_from_sec(dt);                                                              \
            ins_measurements_t m;                                                              \
            memset(&m, 0, sizeof(m));                                                          \
            m.timestamp = t;                                                                   \
            set_imu(&m, acc_body, gyr_body, dt);                                               \
            if (k % 20 == 0) /* 1 Hz GNSS velocity */                                          \
            {                                                                                  \
                m.gnss_vel.is_valid   = true;                                                  \
                m.gnss_vel.vel_ned[0] = (SPD)*cosf(COURSE);                                    \
                m.gnss_vel.vel_ned[1] = (SPD)*sinf(COURSE);                                    \
                m.gnss_vel.vel_ned[2] = 0.0f;                                                  \
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.09f; \
            }                                                                                  \
            ins_update(&(F), &m);                                                              \
        }                                                                                      \
    } while (0)

    float roll, pitch, yaw;

    /* (A) automotive ON: yaw converges to the course over ground, yaw only. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        RUN_AUTOMOTIVE(f, true, course_true, speed, 0.0f, 0.0f, 600); /* 30 s */
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, course_true, DEG2RAD(3.0f), "yaw converges to GNSS course");
        CHECK_NEAR(roll, 0.0, DEG2RAD(1.0f), "roll unaffected by course-yaw fusion");
        CHECK_NEAR(pitch, 0.0, DEG2RAD(1.0f), "pitch unaffected by course-yaw fusion");
    }

    /* (B) automotive OFF (default): GNSS velocity contributes no yaw update,
       so the (unobservable) yaw stays at its wrong initial value. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        RUN_AUTOMOTIVE(f, false, course_true, speed, 0.0f, 0.0f, 600);
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, 0.0, DEG2RAD(2.0f), "yaw unchanged without automotive mode");
    }

    /* (C) automotive ON but below the (default) minimum speed: the course
       is noise-dominated, so no yaw update is fused (yaw stays wrong). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        RUN_AUTOMOTIVE(f, true, course_true, 1.0f /* < default min */, 0.0f, 0.0f, 600);
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, 0.0, DEG2RAD(2.0f), "no yaw update below the minimum ground speed");
    }

    /* (E) automotive_min_speed_mps configurable: a caller-set threshold
       overrides the built-in default. A speed that clears the 2 m/s
       default but not a stricter 15 m/s config must not fuse, and one
       clearing the stricter config must. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        RUN_AUTOMOTIVE(f, true, course_true, 10.0f /* > default min, < configured min */, 15.0f,
                       0.0f, 600);
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, 0.0, DEG2RAD(2.0f), "no yaw update below the configured minimum speed");
    }
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        RUN_AUTOMOTIVE(f, true, course_true, 20.0f /* > configured min */, 15.0f, 0.0f, 600);
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, course_true, DEG2RAD(3.0f),
                   "yaw converges above the configured minimum speed");
    }

    /* (F) automotive_min_yaw_stddev configurable: a coarser floor (e.g.
       modeling an aircraft's wind-drift/crab-angle uncertainty rather than
       a car's tight course-over-ground trust) weakens the per-epoch gain,
       so over the SAME number of epochs the yaw estimate is pulled toward
       the GNSS course markedly less than with the tight built-in floor:
       partial convergence only, not the ~30 s full-convergence budget used
       in (A). */
    {
        ins_t f_default, f_loose;
        memset(&f_default, 0, sizeof(f_default));
        memset(&f_loose, 0, sizeof(f_loose));
        RUN_AUTOMOTIVE(f_default, true, course_true, speed, 0.0f, 0.0f, 100); /* 5 s */
        RUN_AUTOMOTIVE(f_loose, true, course_true, speed, 0.0f, DEG2RAD(30.0f), 100);
        float roll_d, pitch_d, yaw_default, roll_l, pitch_l, yaw_loose;
        ins_get_rpy(&f_default, &roll_d, &pitch_d, &yaw_default);
        ins_get_rpy(&f_loose, &roll_l, &pitch_l, &yaw_loose);
        const float err_default = fabsf(yaw_default - course_true);
        const float err_loose   = fabsf(yaw_loose - course_true);
        CHECK_TRUE(err_loose > err_default,
                   "a coarser configured yaw floor converges slower than the default");
    }
#undef RUN_AUTOMOTIVE

    /* (D) A transient reversing burst (course flips ~180 deg for a few
       fixes) must not corrupt the converged yaw: the chi2 downweight bounds
       each outlier's gain, so yaw stays near the forward heading instead of
       flipping. (Sustained reversing is a documented out-of-scope limit,
       REQ-NAV-034: it cannot be told from forward motion by GNSS alone.) */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.rpy_init_stddev_rad[2] = DEG2RAD(20.0f);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_zupt_disable = true;
        opt.automotive_mode   = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init D");

        ins_time_us_t t = 0;
        int           k;
        for (k = 0; k < 800; ++k) /* 40 s forward -> yaw well converged */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            if (k % 20 == 0)
            {
                m.gnss_vel.is_valid   = true;
                m.gnss_vel.vel_ned[0] = speed * cosf(course_true);
                m.gnss_vel.vel_ned[1] = speed * sinf(course_true);
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.09f;
            }
            ins_update(&f, &m);
        }
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, course_true, DEG2RAD(3.0f), "converged forward before the reverse burst");

        /* Reverse burst: course flips 180 deg for ~5 s (5 fixes at 1 Hz). */
        const float course_rev = course_true + (float)M_PI;
        for (k = 0; k < 100; ++k)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            if (k % 20 == 0)
            {
                m.gnss_vel.is_valid   = true;
                m.gnss_vel.vel_ned[0] = speed * cosf(course_rev);
                m.gnss_vel.vel_ned[1] = speed * sinf(course_rev);
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.09f;
            }
            ins_update(&f, &m);
        }
        CHECK_TRUE(f.is_initialized, "filter alive through the reverse burst");
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        const float d     = yaw - course_true;
        const float dflip = fabsf(atan2f(sinf(d), cosf(d))); /* wrapped |error| */
        CHECK_NEAR(RAD2DEG(dflip), 0.0, 45.0, "transient reverse did not flip yaw [deg]");
    }
}

static void scenario_gravity_override(void)
{
    printf("\n=== Scenario 21: a caller-supplied gravity vector overrides the "
           "model ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    /* Non-zero -> ins_init must use this instead of the WGS84 model. */
    init.gravity_n[0] = 0.0f;
    init.gravity_n[1] = 0.0f;
    init.gravity_n[2] = 9.5f; /* deliberately "wrong" so it's distinguishable */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* The world model (incl. the gravity override) is applied at start,
       not at ins_init (REQ-NAV-033); one IMU epoch crosses the gate. */
    ins_time_us_t t0 = 0;
    start_manual_filter(&f, &t0, 0.01f, (const double*)0);

    CHECK_NEAR(f.gravity_n[0], 0.0f, 1e-9, "gravity override N");
    CHECK_NEAR(f.gravity_n[1], 0.0f, 1e-9, "gravity override E");
    CHECK_NEAR(f.gravity_n[2], 9.5f, 1e-9, "gravity override D");
}

static void scenario_gnss_local_pos_gating(void)
{
    printf("\n=== Scenario 22: GNSS/local-pos aiding gates reject bad fixes "
           "===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    /* Start the filter (REQ-NAV-033) before exercising the fusion gates:
       the finalize epoch would otherwise swallow the first (bad) fix. */
    start_manual_filter(&f, &t, dt, (const double*)0);

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    /* GNSS position with zero variance must be rejected (avoid a zero-R
       update), not silently accepted with an ill-defined weight. */
    t += us_from_sec(dt);
    ins_measurements_t m1;
    memset(&m1, 0, sizeof(m1));
    m1.timestamp = t;
    set_imu(&m1, acc_body, gyr_body, dt);
    m1.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m1.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m1.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m1.gnss_pos.Qll_ned[0]  = 0.0f; /* <= 0 -> rejected */
    m1.gnss_pos.Qll_ned[4]  = 1.0f;
    m1.gnss_pos.Qll_ned[8]  = 1.0f;
    m1.gnss_pos.is_valid    = true;
    ins_update(&f, &m1);
    uint32_t rejected_before = ins_get_diag(&f)->n_gnss_rejected_noise;
    if (rejected_before < 1)
    {
        printf("  FAIL  zero-variance GNSS pos not rejected\n");
        fails++;
    }
    else
        printf("  ok    zero-variance GNSS pos rejected\n");

    /* GNSS position with a stddev above the shutdown threshold must be
       rejected the same way. */
    t += us_from_sec(dt);
    ins_measurements_t m2;
    memset(&m2, 0, sizeof(m2));
    m2.timestamp = t;
    set_imu(&m2, acc_body, gyr_body, dt);
    m2.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m2.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m2.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    const float too_noisy   = opt.gnss_max_horizontal_pos_stddev_m + 5.0f;
    m2.gnss_pos.Qll_ned[0]  = too_noisy * too_noisy;
    m2.gnss_pos.Qll_ned[4]  = too_noisy * too_noisy;
    m2.gnss_pos.Qll_ned[8]  = 1.0f;
    m2.gnss_pos.is_valid    = true;
    ins_update(&f, &m2);
    if (ins_get_diag(&f)->n_gnss_rejected_noise <= rejected_before)
    {
        printf("  FAIL  over-threshold GNSS pos stddev not rejected\n");
        fails++;
    }
    else { printf("  ok    over-threshold GNSS pos stddev rejected\n"); }
    rejected_before = ins_get_diag(&f)->n_gnss_rejected_noise;

    /* GNSS velocity with zero variance must be rejected the same way
       (pos left invalid so only the velocity gate is exercised). */
    t += us_from_sec(dt);
    ins_measurements_t m3;
    memset(&m3, 0, sizeof(m3));
    m3.timestamp = t;
    set_imu(&m3, acc_body, gyr_body, dt);
    m3.gnss_vel.vel_ned[0] = 0.0f;
    m3.gnss_vel.Qll_ned[0] = 0.0f; /* <= 0 -> rejected */
    m3.gnss_vel.Qll_ned[4] = 1.0f;
    m3.gnss_vel.Qll_ned[8] = 1.0f;
    m3.gnss_vel.is_valid   = true;
    ins_update(&f, &m3);
    if (ins_get_diag(&f)->n_gnss_rejected_noise <= rejected_before)
    {
        printf("  FAIL  zero-variance GNSS vel not rejected\n");
        fails++;
    }
    else { printf("  ok    zero-variance GNSS vel rejected\n"); }

    /* A GNSS fix older than INS_MAX_DELAY_MS (500 ms) must be dropped
       before even trying to anchor it in the history. */
    t += us_from_sec(dt);
    ins_measurements_t m4;
    memset(&m4, 0, sizeof(m4));
    m4.timestamp = t;
    set_imu(&m4, acc_body, gyr_body, dt);
    m4.gnss_pos.xyz_ecef[0]         = init.x_ecef[0];
    m4.gnss_pos.xyz_ecef[1]         = init.x_ecef[1];
    m4.gnss_pos.xyz_ecef[2]         = init.x_ecef[2];
    m4.gnss_pos.Qll_ned[0]          = 1.0f;
    m4.gnss_pos.Qll_ned[4]          = 1.0f;
    m4.gnss_pos.Qll_ned[8]          = 1.0f;
    m4.gnss_pos.is_valid            = true;
    m4.gnss_delay_ms                = 600; /* > INS_MAX_DELAY_MS */
    const uint32_t no_anchor_before = ins_get_diag(&f)->n_gnss_no_anchor;
    ins_update(&f, &m4);
    if (ins_get_diag(&f)->n_gnss_no_anchor <= no_anchor_before)
    {
        printf("  FAIL  over-max-delay GNSS fix not flagged as un-anchorable\n");
        fails++;
    }
    else { printf("  ok    over-max-delay GNSS fix rejected before history lookup\n"); }

    /* Local-pos fixes follow the same delay cap, but fail silently (no
       diag counter). Just confirm it doesn't perturb the state. */
    float pos_before[3];
    ins_get_position_local(&f, pos_before);
    t += us_from_sec(dt);
    ins_measurements_t m5;
    memset(&m5, 0, sizeof(m5));
    m5.timestamp = t;
    set_imu(&m5, acc_body, gyr_body, dt);
    m5.local_pos.pos_ned[0] = 50.0f; /* way off, would move pos_local if fused */
    m5.local_pos.Qll_ned[0] = 0.01f;
    m5.local_pos.Qll_ned[4] = 0.01f;
    m5.local_pos.Qll_ned[8] = 0.01f;
    m5.local_pos.is_valid   = true;
    m5.local_pos_delay_ms   = 600; /* > INS_MAX_DELAY_MS */
    ins_update(&f, &m5);
    float pos_after[3];
    ins_get_position_local(&f, pos_after);
    CHECK_NEAR(pos_after[0], pos_before[0], 0.01, "over-max-delay local-pos fix ignored");
}

static void scenario_mag_gating(void)
{
    printf("\n=== Scenario 23: magnetometer aiding gate rejects bad fixes ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;

    /* A magnetometer sample that arrives without a variance is fused with
       the library's own magnetometer noise (sensor_defaults.h) instead of
       being dropped: a caller that never characterized its sensor still
       gets a heading anchor, and the harnesses do not each need a copy of
       the number. What must NOT happen is the zero-R reading of a zero
       variance, i.e. the sample being taken as certainty. */
    init.rpy_init_rad[2] = 10.0f * (float)M_PI / 180.0f; /* re-init with yaw error */
    rc                   = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    /* 150 ms > opt.magnetometer_min_delay_ms (100 ms), so exactly one of
       these epochs is fused and the check below sees a single update. */
    int step;
    for (step = 1; step <= 15; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]     = init.magnetic_n[0];
        m.mag.data[1]     = init.magnetic_n[1];
        m.mag.data[2]     = init.magnetic_n[2];
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.0f; /* -> default */
        m.mag.is_valid                                            = true;
        ins_update(&f, &m);
    }
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_TRUE(fabsf(yaw) < fabsf(init.rpy_init_rad[2]),
               "mag fix without variance is fused (yaw pulled toward truth)");
    CHECK_TRUE(fabsf(yaw) > 0.01f, "mag fix without variance is not read as zero-R certainty");

    /* A magnetic model with no horizontal component (straight down, as at
       a magnetic pole) leaves yaw unobservable. The fusion must skip
       rather than divide by ~0. */
    ins_init_t init2    = init;
    init2.magnetic_n[0] = 0.0f;
    init2.magnetic_n[1] = 0.0f;
    init2.magnetic_n[2] = 50.0f;
    rc                  = ins_init(&f, &init2, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    t = 0;
    for (step = 1; step <= 15; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]     = 0.0f;
        m.mag.data[1]     = 0.0f;
        m.mag.data[2]     = 50.0f;
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
        m.mag.is_valid                                            = true;
        ins_update(&f, &m);
    }
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(yaw, init2.rpy_init_rad[2], 0.01,
               "no-horizontal-field mag fix ignored (yaw unchanged)");

    /* Fusion rate limit. A caller who leaves magnetometer_min_delay_ms at
       0 must not get the sensor's full output rate: the magnetometer is a
       long-term heading anchor, so the unconfigured filter throttles to
       the 1 Hz default. A negative value is the explicit way back to
       "fuse every sample". Both are checked through their effect on yaw,
       not only through the resolved option, with a mag noise (20 uT
       1-sigma) at which a handful of fusions cannot hide the difference. */
    ins_init_t init_rl      = init;
    init_rl.rpy_init_rad[2] = 10.0f * (float)M_PI / 180.0f; /* yaw error to pull out */
    ins_options_t opt_rl    = opt;

    opt_rl.magnetometer_min_delay_ms = 0;
    rc                               = ins_init(&f, &init_rl, &opt_rl);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }
    CHECK_TRUE(f.opt.magnetometer_min_delay_ms == 1000,
               "unset mag rate limit resolves to the 1 Hz default");
    t = 0;
    feed_mag_steps(&f, &t, 300, init.magnetic_n, 400.0f); /* 3 s at 100 Hz */
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    const float yaw_throttled = yaw;

    opt_rl.magnetometer_min_delay_ms = -1;
    rc                               = ins_init(&f, &init_rl, &opt_rl);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }
    CHECK_TRUE(f.opt.magnetometer_min_delay_ms == 0,
               "negative mag rate limit disables the throttle");
    t = 0;
    feed_mag_steps(&f, &t, 300, init.magnetic_n, 400.0f);
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_TRUE(fabsf(yaw) < fabsf(yaw_throttled),
               "unthrottled mag pulls yaw in faster than the 1 Hz default");
}

/* Buffer wraparound is now exercised via the absence of a fix (not via the
   old accel gate, which no longer defers anything, REQ-NAV-047): with no
   usable position fix, ins_autoinit_try's own top gate keeps returning
   false regardless of motion, so the FIFO leveling buffer fills past its
   cap and wraps. Once a fix does arrive, the bootstrap proceeds. */
static void scenario_autoinit_buffer_wraparound(void)
{
    printf("\n=== Scenario 24: auto-init leveling buffer wraparound ===\n");
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init = true;

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float acc_level[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3]  = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;

    /* Overfill the leveling buffer (INS_AUTOINIT_SAMPLES_MAX = 64) so the
       FIFO drop path runs: IMU-only, no fix at all, so ins_autoinit_try
       returns immediately without touching the moving/static distinction. */
    for (step = 1; step <= 90; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_level, gyr_body, dt);
        ins_update(&f, &m);
    }
    if (f.is_initialized)
    {
        printf("  FAIL  filter bootstrapped without ever seeing a fix\n");
        fails++;
        return;
    }
    if (f.autoinit_count != INS_AUTOINIT_SAMPLES_MAX)
    {
        printf("  FAIL  autoinit buffer did not fill/wrap to its cap (%d != %d)\n",
               f.autoinit_count, INS_AUTOINIT_SAMPLES_MAX);
        fails++;
    }
    else { printf("  ok    autoinit buffer filled and wrapped at its cap\n"); }

    /* First fix (quasi-static IMU throughout): must trigger the bootstrap
       with the tight, static-case roll/pitch stddev. */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_level, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    if (!f.is_initialized)
    {
        printf("  FAIL  filter never bootstrapped once a fix arrived\n");
        fails++;
        return;
    }
    printf("  ok    bootstrap completed once a fix arrived\n");
    CHECK_NEAR(sqrtf(f.d[INS_IDX_RPY + 0]), init.rpy_init_stddev_rad[0], 1e-4,
               "quasi-static bootstrap keeps the configured baseline roll stddev [rad]");
}

static void scenario_default_mems_noise(void)
{
    printf("\n=== Scenario 25: default MEMS noise (bias RW, ARW/VRW) when "
           "unset ===\n");

    /* Bias random walk: 0 -> conservative consumer-MEMS default. The
       literals below must mirror INS_DEFAULT_ACC_BIAS_RW_MPS2_SQRTS /
       INS_DEFAULT_GYR_BIAS_RW_RPS_SQRTS in ins.c. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        init.acc_bias_pred_stddev_mps2_sqrts = 0.0f;
        init.gyr_bias_pred_stddev_rps_sqrts  = 0.0f;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        CHECK_NEAR(f.init.acc_bias_pred_stddev_mps2_sqrts, 3e-5f, 1e-9,
                   "0 acc bias RW resolves to the MEMS default");
        CHECK_NEAR(f.init.gyr_bias_pred_stddev_rps_sqrts, 2e-6f, 1e-9,
                   "0 gyr bias RW resolves to the MEMS default");
    }

    /* An explicit, non-zero value must be left untouched. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        init.acc_bias_pred_stddev_mps2_sqrts = 7e-4f;
        init.gyr_bias_pred_stddev_rps_sqrts  = 5e-6f;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        CHECK_NEAR(f.init.acc_bias_pred_stddev_mps2_sqrts, 7e-4f, 1e-9,
                   "explicit acc bias RW is left untouched");
        CHECK_NEAR(f.init.gyr_bias_pred_stddev_rps_sqrts, 5e-6f, 1e-9,
                   "explicit gyr bias RW is left untouched");
    }

    /* ARW/VRW: leaving Qll_diag at 0 for acc/gyr must produce the exact same
       covariance growth as explicitly supplying the MEMS default PSD: the
       fallback is actually applied, not silently treated as zero noise.
       gyr_arw reuses the shared INS_DEFAULT_GYR_ARW_RPS_SQRTHZ
       (sensor_defaults.h); the "350e-6" VRW scale factor is ins.c-local
       (INS_DEFAULT_ACC_VRW_MPS2_SQRTHZ, not exposed in a header) and
       stays a literal here, but built from the real INS_GRAVITY_NOMINAL
       so it can't go stale independently of that constant again. */
    {
        ins_t f_zero, f_explicit;
        memset(&f_zero, 0, sizeof(f_zero));
        memset(&f_explicit, 0, sizeof(f_explicit));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        if (ins_init(&f_zero, &init, &opt) != 0 || ins_init(&f_explicit, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f_zero.latlonh[0], (float)f_zero.latlonh[2], g_vec);
        const float acc_body[3]     = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3]     = {0.0f, 0.0f, 0.0f};
        const float acc_vrw         = 350e-6f * INS_GRAVITY_NOMINAL;
        const float gyr_arw         = INS_DEFAULT_GYR_ARW_RPS_SQRTHZ;
        const float acc_psd_default = acc_vrw * acc_vrw;
        const float gyr_psd_default = gyr_arw * gyr_arw;

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 200; ++step)
        {
            t += us_from_sec(dt);

            ins_measurements_t m0;
            memset(&m0, 0, sizeof(m0));
            m0.timestamp        = t;
            m0.strapdown_dt_sec = dt;
            vec3_assign(m0.acc.data, acc_body);
            m0.acc.is_valid = true; /* Qll_diag left 0 */
            vec3_assign(m0.gyr.data, gyr_body);
            m0.gyr.is_valid = true; /* Qll_diag left 0 */
            ins_update(&f_zero, &m0);

            ins_measurements_t me;
            memset(&me, 0, sizeof(me));
            me.timestamp        = t;
            me.strapdown_dt_sec = dt;
            vec3_assign(me.acc.data, acc_body);
            me.acc.Qll_diag[0] = me.acc.Qll_diag[1] = me.acc.Qll_diag[2] = acc_psd_default;
            me.acc.is_valid                                              = true;
            vec3_assign(me.gyr.data, gyr_body);
            me.gyr.Qll_diag[0] = me.gyr.Qll_diag[1] = me.gyr.Qll_diag[2] = gyr_psd_default;
            me.gyr.is_valid                                              = true;
            ins_update(&f_explicit, &me);
        }

        /* Compare P(VEL,VEL) and P(yaw,yaw) between the two runs (via the
           UDU reconstruction, since U need not stay the identity). */
        float p_vel_zero = 0.0f, p_vel_explicit = 0.0f;
        float p_yaw_zero = 0.0f, p_yaw_explicit = 0.0f;
        for (int k = 0; k < INS_UNKNOWNS; ++k)
        {
            const float uz = f_zero.U[INS_IDX_VEL + k * INS_UNKNOWNS];
            const float ue = f_explicit.U[INS_IDX_VEL + k * INS_UNKNOWNS];
            p_vel_zero += uz * uz * f_zero.d[k];
            p_vel_explicit += ue * ue * f_explicit.d[k];
            const float uyz = f_zero.U[(INS_IDX_RPY + 2) + k * INS_UNKNOWNS];
            const float uye = f_explicit.U[(INS_IDX_RPY + 2) + k * INS_UNKNOWNS];
            p_yaw_zero += uyz * uyz * f_zero.d[k];
            p_yaw_explicit += uye * uye * f_explicit.d[k];
        }
        CHECK_NEAR(p_vel_zero, p_vel_explicit, 1e-9,
                   "zero VRW matches explicit MEMS-default velocity covariance");
        CHECK_NEAR(p_yaw_zero, p_yaw_explicit, 1e-9,
                   "zero ARW matches explicit MEMS-default yaw covariance");
    }
}

/* @satisfies REQ-NAV-043 (verification scenario) */
static void scenario_beginner_option_defaults(void)
{
    printf("\n=== Scenario: beginner-friendly option defaults (hard gates) ===\n");

    /* A zeroed options struct (only auto_init on) must resolve the hard
       gates to working defaults, not filter-killing zeros. Literals mirror
       INS_DEFAULT_* in ins.c. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init = true;

        int rc = ins_init(&f, &init, &opt);
        CHECK_TRUE(rc == 0, "init succeeds with a zeroed options struct");

        CHECK_NEAR(f.opt.max_prediction_time_sec, 0.5f, 1e-9,
                   "0 max_prediction_time_sec resolves to 0.5 s");
        /* The fusion gates default to the accuracy caps (REQ-NAV-007,
           REQ-NAV-071): the unconfigured filter downweights a bad fix
           instead of rejecting it. */
        CHECK_NEAR(f.opt.gnss_max_horizontal_pos_stddev_m, 120.0f, 1e-9,
                   "0 GNSS horiz. pos gate resolves to 120 m");
        CHECK_NEAR(f.opt.gnss_max_vertical_pos_stddev_m, 120.0f, 1e-9,
                   "0 GNSS vert. pos gate resolves to 120 m");
        CHECK_NEAR(f.opt.gnss_max_horizontal_vel_stddev_mps, 60.0f, 1e-9,
                   "0 GNSS horiz. vel gate resolves to 60 m/s");
        CHECK_NEAR(f.opt.gnss_max_vertical_vel_stddev_mps, 60.0f, 1e-9,
                   "0 GNSS vert. vel gate resolves to 60 m/s");

        /* Conditioning knobs (REQ-NAV-038, REQ-NAV-071, REQ-NAV-072): 0
           resolves to the default, and a negative value is the opt-out. */
        CHECK_NEAR(f.opt.gnss_pos_stddev_floor_hor_m, 0.5f, 1e-9,
                   "0 GNSS horiz. pos floor resolves to 0.5 m");
        CHECK_NEAR(f.opt.gnss_pos_stddev_floor_ver_m, 1.0f, 1e-9,
                   "0 GNSS vert. pos floor resolves to 1.0 m");
        CHECK_NEAR(f.opt.gnss_vel_stddev_floor_hor_mps, 0.3f, 1e-9,
                   "0 GNSS horiz. vel floor resolves to 0.3 m/s");
        CHECK_NEAR(f.opt.gnss_vel_stddev_floor_ver_mps, 0.5f, 1e-9,
                   "0 GNSS vert. vel floor resolves to 0.5 m/s");
        CHECK_NEAR(f.opt.gnss_pos_stddev_cap_hor_m, 120.0f, 1e-9,
                   "0 GNSS horiz. pos cap resolves to 120 m");
        CHECK_NEAR(f.opt.gnss_pos_stddev_cap_ver_m, 120.0f, 1e-9,
                   "0 GNSS vert. pos cap resolves to 120 m");
        CHECK_NEAR(f.opt.gnss_vel_stddev_cap_hor_mps, 60.0f, 1e-9,
                   "0 GNSS horiz. vel cap resolves to 60 m/s");
        CHECK_NEAR(f.opt.gnss_vel_stddev_cap_ver_mps, 60.0f, 1e-9,
                   "0 GNSS vert. vel cap resolves to 60 m/s");
        CHECK_NEAR(f.opt.gnss_acc_envelope_tau_sec, 5.0f, 1e-9,
                   "0 GNSS accuracy envelope tau resolves to 5 s");
        /* Fusion rate limit (REQ-NAV-074) and position decimation
           (REQ-NAV-063): the rate limit is on at 10 Hz, the decimation off. */
        CHECK_TRUE(f.opt.gnss_min_delay_ms == 100,
                   "0 GNSS min fusion delay resolves to 100 ms (10 Hz)");
        CHECK_TRUE(f.opt.gnss_pos_decimation == 1, "0 GNSS position decimation resolves to off");
        /* The manoeuvre term is on by default, with a margin on the
           vertical axis. */
        CHECK_NEAR(f.opt.gnss_vel_noise_acc_scale_hor, 0.20f, 1e-9,
                   "0 GNSS manoeuvre vel-noise scale (hor) resolves to 0.20");
        CHECK_NEAR(f.opt.gnss_vel_noise_acc_scale_ver, 0.30f, 1e-9,
                   "0 GNSS manoeuvre vel-noise scale (ver) resolves to 0.30");
    }

    /* A negative conditioning knob is the explicit opt-out and resolves to
       0 ("not applied"), not to the default (REQ-NAV-043). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init                     = true;
        opt.gnss_pos_stddev_floor_hor_m   = -1.0f;
        opt.gnss_vel_stddev_floor_ver_mps = -1.0f;
        opt.gnss_pos_stddev_cap_hor_m     = -1.0f;
        opt.gnss_acc_envelope_tau_sec     = -1.0f;
        opt.gnss_min_delay_ms             = -1;

        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init succeeds with opted-out knobs");
        CHECK_NEAR(f.opt.gnss_pos_stddev_floor_hor_m, 0.0f, 1e-9,
                   "negative horiz. pos floor resolves to off");
        CHECK_NEAR(f.opt.gnss_vel_stddev_floor_ver_mps, 0.0f, 1e-9,
                   "negative vert. vel floor resolves to off");
        CHECK_NEAR(f.opt.gnss_pos_stddev_cap_hor_m, 0.0f, 1e-9,
                   "negative horiz. pos cap resolves to off");
        CHECK_NEAR(f.opt.gnss_acc_envelope_tau_sec, 0.0f, 1e-9,
                   "negative envelope tau resolves to off");
        CHECK_TRUE(f.opt.gnss_min_delay_ms == 0, "negative GNSS min fusion delay resolves to off");
        /* The untouched members still take their defaults. */
        CHECK_NEAR(f.opt.gnss_pos_stddev_floor_ver_m, 1.0f, 1e-9,
                   "an opt-out on one axis group leaves the other at its default");
    }

    /* A cap below the floor of the same axis group is raised to the floor
       (REQ-NAV-071), so the floor cannot be silently undone. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init                   = true;
        opt.gnss_pos_stddev_floor_hor_m = 2.0f;
        opt.gnss_pos_stddev_cap_hor_m   = 1.0f;

        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init succeeds with an inverted floor/cap");
        CHECK_NEAR(f.opt.gnss_pos_stddev_cap_hor_m, 2.0f, 1e-9, "cap raised to the floor");
    }

    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "re-init for the mode-gate checks");

        /* Mode-transition gates (REQ-NAV-051, REQ-NAV-052): a zeroed options
           struct must resolve to the strict entry / loose exit pair, not to
           "enter on anything" / "never leave". */
        CHECK_NEAR(f.opt.gnss_start_max_horizontal_pos_stddev_m, 2.0f, 1e-9,
                   "0 GNSS entry horiz. pos gate resolves to 2 m");
        CHECK_NEAR(f.opt.gnss_start_max_vertical_pos_stddev_m, 3.0f, 1e-9,
                   "0 GNSS entry vert. pos gate resolves to 3 m");
        CHECK_NEAR(f.opt.gnss_start_max_horizontal_vel_stddev_mps, 0.25f, 1e-9,
                   "0 GNSS entry horiz. vel gate resolves to 0.25 m/s");
        CHECK_NEAR(f.opt.gnss_start_max_vertical_vel_stddev_mps, 0.30f, 1e-9,
                   "0 GNSS entry vert. vel gate resolves to 0.30 m/s");
        CHECK_NEAR(f.opt.gnss_stop_max_horizontal_pos_stddev_m, 5.0f, 1e-9,
                   "0 GNSS exit horiz. pos gate resolves to 5 m");
        CHECK_NEAR(f.opt.gnss_stop_max_vertical_pos_stddev_m, 7.0f, 1e-9,
                   "0 GNSS exit vert. pos gate resolves to 7 m");
        CHECK_NEAR(f.opt.gnss_stop_max_horizontal_vel_stddev_mps, 0.4f, 1e-9,
                   "0 GNSS exit horiz. vel gate resolves to 0.4 m/s");
        CHECK_NEAR(f.opt.gnss_stop_max_vertical_vel_stddev_mps, 0.5f, 1e-9,
                   "0 GNSS exit vert. vel gate resolves to 0.5 m/s");
        CHECK_NEAR(f.opt.gnss_stop_dwell_sec, 10.0f, 1e-9, "0 GNSS stop dwell resolves to 10 s");
    }

    {
        /* Same two gates, explicit positive values this time (MC/DC: the
           "already valid, keep it" arm, never sensitized by the zeroed
           config above). */
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init                              = true;
        opt.gnss_start_max_vertical_vel_stddev_mps = 0.42f;
        opt.gnss_stop_max_vertical_vel_stddev_mps  = 0.84f;
        opt.mag_field_tolerance                    = 0.15f;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "re-init for the explicit-value gates");
        CHECK_NEAR(f.opt.gnss_start_max_vertical_vel_stddev_mps, 0.42f, 1e-9,
                   "explicit GNSS entry vert. vel gate not defaulted");
        CHECK_NEAR(f.opt.gnss_stop_max_vertical_vel_stddev_mps, 0.84f, 1e-9,
                   "explicit GNSS exit vert. vel gate not defaulted");
        CHECK_NEAR(f.opt.mag_field_tolerance, 0.15f, 1e-9,
                   "explicit mag_field_tolerance not defaulted");
    }

    /* Explicit, non-zero values must be left exactly as supplied. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init                          = true;
        opt.max_prediction_time_sec            = 0.25f;
        opt.gnss_max_horizontal_pos_stddev_m   = 3.0f;
        opt.gnss_max_vertical_pos_stddev_m     = 4.0f;
        opt.gnss_max_horizontal_vel_stddev_mps = 0.5f;
        opt.gnss_max_vertical_vel_stddev_mps   = 0.7f;

        int rc = ins_init(&f, &init, &opt);
        CHECK_TRUE(rc == 0, "init succeeds with explicit gates");

        CHECK_NEAR(f.opt.max_prediction_time_sec, 0.25f, 1e-9,
                   "explicit max_prediction_time_sec is left untouched");
        CHECK_NEAR(f.opt.gnss_max_horizontal_pos_stddev_m, 3.0f, 1e-9,
                   "explicit GNSS horiz. pos gate is left untouched");
        CHECK_NEAR(f.opt.gnss_max_vertical_pos_stddev_m, 4.0f, 1e-9,
                   "explicit GNSS vert. pos gate is left untouched");
        CHECK_NEAR(f.opt.gnss_max_horizontal_vel_stddev_mps, 0.5f, 1e-9,
                   "explicit GNSS horiz. vel gate is left untouched");
        CHECK_NEAR(f.opt.gnss_max_vertical_vel_stddev_mps, 0.7f, 1e-9,
                   "explicit GNSS vert. vel gate is left untouched");
    }
}

/* A zeroed ins_init_t (only x_ecef set, since ins_init rejects a
 * near-origin position) must resolve every *_init_stddev_*, *_pred_stddev_*
 * and zero_*_stddev_* field to a working, correctable default -- not literal
 * 0 (REQ-NAV-049).
 *
 * The expected values below are written out as literals on purpose: they are
 * the ones REQ-NAV-049 states, not a second reference to ins.c's own
 * INS_DEFAULT_* macros. Comparing the macros against themselves would pass no
 * matter what they hold, which is how seven of these values drifted away from
 * the requirement unnoticed. Retuning a default is therefore meant to fail
 * here until the requirement is updated to match. */
static void scenario_beginner_init_defaults(void)
{
    printf("\n=== Scenario: beginner-friendly initial-state/process-noise defaults ===\n");

    /* (A) Zeroed init struct: fields resolve to the documented defaults. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        memset(&init, 0, sizeof(init));
        ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init               = true;
        opt.gnss_init_dwell_disable = true; /* not what this scenario tests */

        int rc = ins_init(&f, &init, &opt);
        CHECK_TRUE(rc == 0, "init succeeds with a zeroed init struct");

        /* The values REQ-NAV-049 states, in the units it states them in.
           Relative tolerance only guards the float conversion. */
        const float deg = (float)(M_PI / 180.0);
        CHECK_NEAR(f.init.pos_init_stddev_m, 10.0f, 1e-6, "0 pos_init_stddev_m -> default [m]");
        CHECK_NEAR(f.init.vel_init_stddev_mps, 1.0f, 1e-6,
                   "0 vel_init_stddev_mps -> default [m/s]");
        for (int i = 0; i < 3; i++)
        {
            CHECK_NEAR(f.init.rpy_init_stddev_rad[i], 5.0f * deg, 1e-6,
                       "0 rpy_init_stddev_rad[i] -> default [rad]");
        }
        CHECK_NEAR(f.init.acc_bias_init_stddev_mps2, 0.03f, 1e-6,
                   "0 acc_bias_init_stddev_mps2 -> default [m/s^2]");
        CHECK_NEAR(f.init.gyr_bias_init_stddev_rps, 1.0f * deg, 1e-6,
                   "0 gyr_bias_init_stddev_rps -> default [rad/s]");
        CHECK_NEAR(f.init.pos_pred_stddev_m_sqrts, 0.01f, 1e-6,
                   "0 pos_pred_stddev_m_sqrts -> default");
        CHECK_NEAR(f.init.vel_pred_stddev_mps_sqrts, 0.0005f, 1e-9,
                   "0 vel_pred_stddev_mps_sqrts -> default");
        CHECK_NEAR(f.init.rpy_pred_stddev_rad_sqrts, 0.001f, 1e-9,
                   "0 rpy_pred_stddev_rad_sqrts -> default");
        CHECK_NEAR(f.init.zero_vel_stddev_mps, 0.05f, 1e-6,
                   "0 zero_vel_stddev_mps -> default [m/s]");
        CHECK_NEAR(f.init.zero_rot_stddev_rps, 0.5f * deg, 1e-6,
                   "0 zero_rot_stddev_rps -> default [rad/s]");

        /* Decisive end-to-end check, mirroring the real incident: a
           genuinely stationary platform with a real, uncompensated gyro
           bias must have that bias converge via auto-ZUPT/ZARU, not stay
           pinned at 0 forever (which is what happened with a caller that
           forgot to set these fields, REQ-NAV-048's rationale). */
        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float   acc_body[3]   = {0.0f, 0.0f, -g_vec[2]};
        const float   gbias_true[3] = {0.02f, -0.01f, 0.005f}; /* rad/s */
        const float   dt            = 0.01f;
        ins_time_us_t t             = 0;
        int           step;
        for (step = 1; step <= 20; ++step) /* auto-init leveling window */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gbias_true, dt);
            ins_update(&f, &m);
        }
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gbias_true, dt);
        m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0]  = 1.0f;
        m.gnss_pos.Qll_ned[4]  = 1.0f;
        m.gnss_pos.Qll_ned[8]  = 1.0f;
        m.gnss_pos.is_valid    = true;
        ins_update(&f, &m);
        CHECK_TRUE(f.is_initialized, "bootstraps on the first fix despite the zeroed init struct");

        for (step = 1; step <= 2000; ++step) /* 20 s stationary */
        {
            t += us_from_sec(dt);
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gbias_true, dt);
            ins_update(&f, &m);
        }
        float gbias[3];
        ins_get_bias_gyr(&f, gbias);
        CHECK_NEAR(gbias[0], gbias_true[0], 0.003,
                   "auto-ZUPT converges gyro bias x despite the zeroed init struct (REQ-NAV-049)");
        CHECK_NEAR(gbias[1], gbias_true[1], 0.003, "auto-ZUPT converges gyro bias y");
        CHECK_NEAR(gbias[2], gbias_true[2], 0.003, "auto-ZUPT converges gyro bias z");
    }

    /* (B) Explicit, non-zero values must be left exactly as supplied. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        memset(&init, 0, sizeof(init));
        ins_latlonh_to_ecef(48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 300.0, init.x_ecef);
        init.pos_init_stddev_m         = 5.0f;
        init.vel_init_stddev_mps       = 2.0f;
        init.rpy_init_stddev_rad[0]    = (float)(10.0 * M_PI / 180.0);
        init.rpy_init_stddev_rad[1]    = (float)(10.0 * M_PI / 180.0);
        init.acc_bias_init_stddev_mps2 = 0.2f;
        init.gyr_bias_init_stddev_rps  = (float)(2.0 * M_PI / 180.0);
        init.pos_pred_stddev_m_sqrts   = 0.02f;
        init.vel_pred_stddev_mps_sqrts = 0.1f;
        init.rpy_pred_stddev_rad_sqrts = (float)(0.05 * M_PI / 180.0);
        init.zero_vel_stddev_mps       = 0.2f;
        init.zero_rot_stddev_rps       = (float)(0.5 * M_PI / 180.0);
        ins_options_t opt;
        memset(&opt, 0, sizeof(opt));
        opt.auto_init = true;

        int rc = ins_init(&f, &init, &opt);
        CHECK_TRUE(rc == 0, "init succeeds with explicit initial-state stddevs");

        CHECK_NEAR(f.init.pos_init_stddev_m, 5.0f, 1e-9, "explicit pos_init_stddev_m kept");
        CHECK_NEAR(f.init.vel_init_stddev_mps, 2.0f, 1e-9, "explicit vel_init_stddev_mps kept");
        CHECK_NEAR(f.init.rpy_init_stddev_rad[0], (float)(10.0 * M_PI / 180.0), 1e-9,
                   "explicit rpy_init_stddev_rad[0] kept");
        CHECK_NEAR(f.init.acc_bias_init_stddev_mps2, 0.2f, 1e-9,
                   "explicit acc_bias_init_stddev_mps2 kept");
        CHECK_NEAR(f.init.gyr_bias_init_stddev_rps, (float)(2.0 * M_PI / 180.0), 1e-9,
                   "explicit gyr_bias_init_stddev_rps kept");
        CHECK_NEAR(f.init.pos_pred_stddev_m_sqrts, 0.02f, 1e-9,
                   "explicit pos_pred_stddev_m_sqrts kept");
        CHECK_NEAR(f.init.vel_pred_stddev_mps_sqrts, 0.1f, 1e-9,
                   "explicit vel_pred_stddev_mps_sqrts kept");
        CHECK_NEAR(f.init.rpy_pred_stddev_rad_sqrts, (float)(0.05 * M_PI / 180.0), 1e-9,
                   "explicit rpy_pred_stddev_rad_sqrts kept");
        CHECK_NEAR(f.init.zero_vel_stddev_mps, 0.2f, 1e-9, "explicit zero_vel_stddev_mps kept");
        CHECK_NEAR(f.init.zero_rot_stddev_rps, (float)(0.5 * M_PI / 180.0), 1e-9,
                   "explicit zero_rot_stddev_rps kept");
    }
}

static void scenario_auto_zupt(void)
{
    printf("\n=== Scenario 26: automatic ZUPT/ZARU detector ===\n");

    float R_n_to_e[9];
    {
        ins_init_t tmp;
        fill_default_init(&tmp, 0);
        double lat, lon, h;
        ins_ecef_to_latlonh(tmp.x_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R_n_to_e);
    }

    /* (A) Default-enabled: purely stationary, no manual ZUPT/ZARU flags at
       all. The detector alone must fire repeatedly but rate-limited
       (clearly less often than every epoch). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        const uint32_t n = ins_get_diag(&f)->n_auto_zupt;
        /* Default dwell 0.2 s (measured from the variance window's first
           verdict, itself needing the default 0.2 s window) + min interval
           0.2 s over ~9.5 s of eligible time -> expect on the order of
           (9.5 - arm time) / 0.2 =~ 45 triggers, far below the 1000 epochs
           run, and definitely more than zero. */
        if (n > 0 && n < 60)
        {
            printf("  ok    auto-ZUPT fired %u times over 10 s (rate-limited)\n", n);
        }
        else
        {
            printf("  FAIL  auto-ZUPT fire count implausible: %u\n", n);
            fails++;
        }
    }

    /* (B) Explicitly disabled: same stationary run, detector must never
       fire. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_zupt_disable = true;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt == 0)
        {
            printf("  ok    auto_zupt_disable suppresses the detector\n");
        }
        else
        {
            printf("  FAIL  detector fired despite auto_zupt_disable (%u)\n",
                   ins_get_diag(&f)->n_auto_zupt);
            fails++;
        }
    }

    /* (C) A large, persistent GNSS-reported velocity (accurate: well under
       auto_zupt_max_vel_stddev_mps) must NOT arm the detector, for the
       whole run, even though the IMU alone looks perfectly still
       (gravity-only specific force, quiet gyro, zero state-velocity
       error). The velocity gate is exclusively GNSS-driven (REQ-NAV-013);
       this is the positive case, (G) below is its no-GNSS counterpart. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            /* GNSS reports a steady 2 m/s, well above auto_zupt_max_vel_mps
               (default 1.5 m/s), accurately (0.1 m/s stddev, well under the
               default 0.5 m/s accuracy requirement). */
            m.gnss_vel.is_valid   = true;
            m.gnss_vel.vel_ned[0] = 2.0f;
            m.gnss_vel.vel_ned[1] = 0.0f;
            m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = 0.01f; /* (0.1 m/s)^2 */
            m.gnss_vel.Qll_ned[4] = 0.01f;
            m.gnss_vel.Qll_ned[8] = 0.01f;
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt == 0)
        {
            printf("  ok    a persistent accurate GNSS velocity above "
                   "auto_zupt_max_vel_mps never arms the detector\n");
        }
        else
        {
            printf("  FAIL  detector armed despite GNSS velocity above "
                   "auto_zupt_max_vel_mps (%u)\n",
                   ins_get_diag(&f)->n_auto_zupt);
            fails++;
        }
    }

    /* (D) A small residual state-velocity error while genuinely stationary
       gets corrected automatically, with no manual
       zero_velocity_update/zero_rotation_update and no GNSS anywhere: with
       no GNSS velocity observation at all the velocity gate does not
       apply (REQ-NAV-013), so this is really exercising the magnitude/
       variance IMU gates alone, not a "below max_vel" case. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        const float verr_n = 0.1f; /* m/s, irrelevant without GNSS, see above */
        init.xdot_ecef[0]  = R_n_to_e[0] * verr_n;
        init.xdot_ecef[1]  = R_n_to_e[1] * verr_n;
        init.xdot_ecef[2]  = R_n_to_e[2] * verr_n;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m); /* no zero_velocity_update/zero_rotation_update set */
        }

        if (ins_get_diag(&f)->n_auto_zupt == 0)
        {
            printf("  FAIL  detector never armed for a small, genuinely static "
                   "velocity error\n");
            fails++;
        }
        else
        {
            printf("  ok    detector armed %u times for the small residual "
                   "error\n",
                   ins_get_diag(&f)->n_auto_zupt);
        }

        float v[3];
        ins_get_velocity_ned(&f, v);
        CHECK_NEAR(v[0], 0.0f, 0.05,
                   "small residual velocity error self-corrects via auto-ZUPT alone");
    }

    /* (E) A dropped IMU sample (acc or gyr invalid) must not confuse the
       detector: it just resets the stillness timer instead of crashing
       or arming on stale data. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 30; ++step) /* 300 ms of dropped IMU */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            /* acc/gyr left is_valid = false (dropped sample). */
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt == 0)
        {
            printf("  ok    dropped IMU samples never arm the detector\n");
        }
        else
        {
            printf("  FAIL  detector armed on dropped IMU samples (%u)\n",
                   ins_get_diag(&f)->n_auto_zupt);
            fails++;
        }
    }

    /* (F) Same corrupted own-velocity estimate as (C), well above the
       max-vel gate -- but this time GNSS reports the platform at rest
       every epoch. Unlike (C), the detector must arm: it should trust
       the external GNSS observation over its own (wrong) state velocity.
       Regression for a real divergence on tools/testbalkon2.ubx: a
       stationary unit with no GNSS lever arm configured (so GNSS could
       not observe attitude directly) had an uncompensated gyro bias tilt
       the attitude, leak gravity into the velocity states, and cross the
       own-velocity gate before the detector ever fired -- permanently
       locking out the correction that would have fixed it. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        const float verr_n = 2.0f; /* m/s, well above the 0.3 m/s gate, same as (C) */
        init.xdot_ecef[0]  = R_n_to_e[0] * verr_n;
        init.xdot_ecef[1]  = R_n_to_e[1] * verr_n;
        init.xdot_ecef[2]  = R_n_to_e[2] * verr_n;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            /* GNSS says the platform is at rest, independent of the
               (wrong) state velocity above. */
            m.gnss_vel.is_valid   = true;
            m.gnss_vel.vel_ned[0] = 0.0f;
            m.gnss_vel.vel_ned[1] = 0.0f;
            m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = 0.01f; /* (0.1 m/s)^2 */
            m.gnss_vel.Qll_ned[4] = 0.01f;
            m.gnss_vel.Qll_ned[8] = 0.01f;
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt > 0)
        {
            printf("  ok    external GNSS velocity arms the detector despite a "
                   "corrupted own-velocity estimate\n");
        }
        else
        {
            printf("  FAIL  detector never armed although GNSS reported "
                   "near-zero velocity\n");
            fails++;
        }
    }

    /* (G) No-GNSS counterpart of (C): same large, persistent state-velocity
       error, but this run never carries a GNSS velocity observation at
       all. REQ-NAV-013 forbids a state-estimate fallback (it would be
       circular, see (F)'s testbalkon2.ubx rationale and
       ins_auto_zupt_velocity_gate_ok's comment in ins.c), so the velocity
       gate must not apply here -- the detector must arm on the IMU
       criteria alone, unlike the pre-fallback-removal behaviour this
       exact setup used to test for (C) before this requirement changed. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        const float verr_n = 2.0f; /* m/s, well above auto_zupt_max_vel_mps */
        init.xdot_ecef[0]  = R_n_to_e[0] * verr_n;
        init.xdot_ecef[1]  = R_n_to_e[1] * verr_n;
        init.xdot_ecef[2]  = R_n_to_e[2] * verr_n;

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s, no GNSS at all */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt > 0)
        {
            printf("  ok    with no GNSS at all the detector arms on the IMU "
                   "criteria alone, ignoring a large state-velocity error\n");
        }
        else
        {
            printf("  FAIL  detector never armed although no GNSS velocity was "
                   "ever offered to gate on\n");
            fails++;
        }
    }

    /* (H) Accuracy requirement on the GNSS velocity gate: a fix reporting
       the same 2 m/s as (C), but with a 1-sigma well above
       auto_zupt_max_vel_stddev_mps (default 0.5 m/s), must not be trusted
       for the gate -- unlike (C), the detector must arm here. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1000; ++step) /* 10 s */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            /* Same reported speed as (C), but 1 m/s stddev (well above the
               0.5 m/s default accuracy requirement) instead of 0.1 m/s. */
            m.gnss_vel.is_valid   = true;
            m.gnss_vel.vel_ned[0] = 2.0f;
            m.gnss_vel.vel_ned[1] = 0.0f;
            m.gnss_vel.vel_ned[2] = 0.0f;
            m.gnss_vel.Qll_ned[0] = 1.0f; /* (1 m/s)^2 */
            m.gnss_vel.Qll_ned[4] = 1.0f;
            m.gnss_vel.Qll_ned[8] = 1.0f;
            ins_update(&f, &m);
        }

        if (ins_get_diag(&f)->n_auto_zupt > 0)
        {
            printf("  ok    a GNSS velocity too inaccurate to trust does not gate "
                   "the detector\n");
        }
        else
        {
            printf("  FAIL  an inaccurate GNSS velocity blocked the detector "
                   "(auto_zupt_max_vel_stddev_mps not enforced?)\n");
            fails++;
        }
    }

    /* (I) ins_set_auto_zupt_disable() suppresses the detector at runtime
       and, on re-enable, requires a fresh dwell period rather than firing
       immediately off a stale timer (REQ-NAV-013). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;

        for (step = 1; step <= 300; ++step) /* 3 s, stationary, detector enabled */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        const uint32_t n_before_disable = ins_get_diag(&f)->n_auto_zupt;

        ins_set_auto_zupt_disable(&f, true);
        for (step = 1; step <= 300; ++step) /* 3 s, still stationary, disabled */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        const uint32_t n_while_disabled = ins_get_diag(&f)->n_auto_zupt;

        ins_set_auto_zupt_disable(&f, false);
        for (step = 1; step <= 30; ++step) /* 0.3 s, below the default 0.2 s dwell
                                               (anchored to the variance window's
                                               first verdict, itself needing the
                                               default 0.2 s window to complete) */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        const uint32_t n_just_after_reenable = ins_get_diag(&f)->n_auto_zupt;

        for (step = 1; step <= 200; ++step) /* 2 more s: past a fresh dwell */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        const uint32_t n_after_reenable = ins_get_diag(&f)->n_auto_zupt;

        if (n_before_disable == 0)
        {
            printf("  FAIL  detector never armed before being disabled\n");
            fails++;
        }
        else if (n_while_disabled != n_before_disable)
        {
            printf("  FAIL  detector fired while disabled (%u -> %u)\n", n_before_disable,
                   n_while_disabled);
            fails++;
        }
        else if (n_just_after_reenable != n_before_disable)
        {
            printf("  FAIL  detector fired off a stale timer immediately on "
                   "re-enable (%u -> %u), before a fresh dwell could elapse\n",
                   n_before_disable, n_just_after_reenable);
            fails++;
        }
        else if (n_after_reenable <= n_before_disable)
        {
            printf("  FAIL  detector never resumed firing after re-enable\n");
            fails++;
        }
        else
        {
            printf("  ok    ins_set_auto_zupt_disable suppresses the detector and "
                   "requires a fresh dwell on re-enable\n");
        }
    }
}

/* The auto-ZARU must fuse the gyro *averaged* over the stillness run.
 * A stationary but vibrating platform (idling engine) produces zero-mean
 * gyro noise well inside the static gate. Sampling a single epoch would
 * inject the momentary vibration into the gyro bias states: with a
 * tight zero_rot_stddev_rps the bias (and with it the heading) gets corrupted
 * at every stop. With averaging, the vibration cancels and the bias must
 * converge to the true value. */
static void scenario_auto_zaru_vibration(void)
{
    printf("\n=== Scenario 27: auto-ZARU averages out vibration ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    /* Allow the true bias within the initial uncertainty and make the
       zero-rotation update aggressive: the worst case for a
       single-sample ZARU. */
    init.gyr_bias_init_stddev_rps = (float)(1.0 * M_PI / 180.0);
    init.zero_rot_stddev_rps      = (float)(0.01 * M_PI / 180.0);
    ins_options_t opt;
    fill_default_opt(&opt);
    /* Both stillness criteria must tolerate the vibration amplitude,
       otherwise the detector never arms and there is no stillness run to
       average over -- which is what this scenario is about, not how the
       detector classifies a vibrating platform. The default variance
       threshold sits well below this amplitude on purpose (REQ-NAV-013),
       so a caller who wants auto-ZARU on a platform that vibrates at
       standstill has to raise it exactly like this.
       Deliberately raised only just past the vibration amplitude used
       here rather than to some round large number: a loose variance
       threshold has been observed to arm mid-flight on real flight-test
       data, so this line doubles as the documented example of how far
       such a platform should go, and no further. */
    opt.auto_zupt_static_gyr_rps        = 0.1f;
    opt.auto_zupt_static_gyr_stddev_rps = (float)(2.5 * M_PI / 180.0);

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3]      = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_bias_true[3] = {0.005f, -0.005f, 0.01f}; /* rad/s */
    const float vib_rps          = 0.03f;                    /* zero-mean, alternating each epoch */

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step) /* 10 s stationary, "engine on" */
    {
        t += us_from_sec(dt);
        const float        vib         = (step % 2 == 0) ? vib_rps : -vib_rps;
        const float        gyr_body[3] = {gyr_bias_true[0] + vib, gyr_bias_true[1] + vib,
                                          gyr_bias_true[2] + vib};
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    if (ins_get_diag(&f)->n_auto_zupt == 0)
    {
        printf("  FAIL  detector never armed (vibration broke the gate?)\n");
        fails++;
        return;
    }
    printf("  ok    detector armed %u times despite vibration\n", ins_get_diag(&f)->n_auto_zupt);

    float bias_est[3];
    ins_get_bias_gyr(&f, bias_est);
    /* A single-sample ZARU would leave an error of up to vib_rps (0.03);
       the averaged measurement must land at the true bias. */
    CHECK_NEAR(bias_est[0], gyr_bias_true[0], 0.002, "gyro bias x [rad/s]");
    CHECK_NEAR(bias_est[1], gyr_bias_true[1], 0.002, "gyro bias y [rad/s]");
    CHECK_NEAR(bias_est[2], gyr_bias_true[2], 0.002, "gyro bias z [rad/s]");
}

/* Variance-based stillness criterion (REQ-NAV-013), tested in isolation:
 * the magnitude bounds are widened out of the way here so that only the
 * variance decides, which is the point of the three cases below.
 *  (A) a large CONSTANT sensor bias must not block it -- a bias shifts
 *      the mean, not the spread, and rejecting stillness there is
 *      self-locking (this detector feeds the very update that would
 *      estimate the bias),
 *  (B) motion with a zero-mean, high-spread signature must block it --
 *      that signature is invisible to a bound on the average, which is
 *      exactly what the magnitude gates are,
 *  (C) the criterion must be configurable out of the way for a platform
 *      whose standstill is inherently noisy. */
static void scenario_static_variance_gate(void)
{
    printf("\n=== Scenario: variance-based stillness gate (REQ-NAV-013) ===\n");

    /* Bias far beyond the magnitude gates, which are widened below so
       they cannot be what decides any of these cases. */
    const float gyr_bias = 0.10f; /* [rad/s], 5.7 deg/s */
    const float acc_bias = 0.22f; /* [m/s^2] */
    /* Motion amplitude, injected as an alternating (zero-mean, high
       spread) signal: invisible to a magnitude gate on the average, but
       exactly what the variance sees. */
    const float shake_gyr = 0.08f;
    const float shake_acc = 0.8f;

    int c;
    for (c = 0; c < 3; ++c)
    {
        const bool moving      = (c == 1);
        const bool relax_gates = (c == 2); /* moving AND criterion configured away */

        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t opt;
        fill_default_opt(&opt);
        /* Take the velocity gate out of the picture: a bias this size
           dead-reckons the velocity estimate past its threshold within a
           second, and that gate would then decide the outcome instead of
           the criterion under test here. (It is its own self-lock, and
           the reason REQ-NAV-013 wants an EXTERNAL velocity there.) */
        opt.auto_zupt_max_vel_mps = 1000.0f;
        /* Magnitude bounds out of the way: they are a separate criterion
           (and the one that catches a constant rotation rate, which the
           variance cannot), not what is under test here. */
        opt.auto_zupt_static_gyr_rps  = 1.0f;
        opt.auto_zupt_static_acc_mps2 = 5.0f;
        if (relax_gates)
        {
            opt.auto_zupt_static_gyr_stddev_rps  = 100.0f;
            opt.auto_zupt_static_acc_stddev_mps2 = 100.0f;
        }

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 600; ++step) /* 6 s */
        {
            /* Alternating sign keeps the MEAN at the bias while the
               per-sample spread is large -- a magnitude gate on the
               average would see nothing. */
            const float s           = ((step % 2) == 0) ? 1.0f : -1.0f;
            const float acc_body[3] = {0.0f, (moving || relax_gates) ? s * shake_acc : 0.0f,
                                       -g_vec[2] + acc_bias};
            const float gyr_body[3] = {gyr_bias, (moving || relax_gates) ? s * shake_gyr : 0.0f,
                                       0.0f};

            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        const uint32_t n = ins_get_diag(&f)->n_auto_zupt;
        if (c == 0)
        {
            if (n > 0)
            {
                printf("  ok    large constant bias does not block the detector (%u)\n", n);
            }
            else
            {
                printf("  FAIL  constant bias blocked the detector (self-lock)\n");
                fails++;
            }
        }
        else if (moving)
        {
            if (n == 0) { printf("  ok    motion with zero-mean spread blocks the detector\n"); }
            else
            {
                printf("  FAIL  detector fired %u times while moving\n", n);
                fails++;
            }
        }
        else
        {
            if (n > 0)
            {
                printf("  ok    configured-away variance criterion lets the same motion "
                       "through (%u)\n",
                       n);
            }
            else
            {
                printf("  FAIL  variance criterion still active after being configured away\n");
                fails++;
            }
        }
    }
}

/* Initial bias-prior consistency check (REQ-NAV-050): standing still,
 * the averaged raw IMU is the sensor bias itself, so a bias far outside
 * the configured initial 1-sigma prior must be flagged (counter + WARN)
 * -- and a clean IMU must stay silent. Both priors are configured well
 * inside the stillness gate here, otherwise the gate would filter out
 * the very samples the check needs to see. */
static void scenario_bias_prior_check(void)
{
    printf("\n=== Scenario: initial bias-prior consistency check ===\n");

    /* Priors a "good IMU" config would state; the injected biases below
       are several sigma outside them but still well within the
       stillness gate (0.04 rad/s, 0.2 m/s^2). */
    const float acc_prior = 0.01f;  /* [m/s^2],  3 sigma = 0.03 */
    const float gyr_prior = 0.002f; /* [rad/s],  3 sigma = 0.006 */
    const float acc_bias  = 0.10f;  /* [m/s^2] */
    const float gyr_bias  = 0.02f;  /* [rad/s] */

    int c;
    for (c = 0; c < 2; ++c)
    {
        const bool biased = (c == 1);

        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        init.acc_bias_init_stddev_mps2 = acc_prior;
        init.gyr_bias_init_stddev_rps  = gyr_prior;
        ins_options_t opt;
        fill_default_opt(&opt);

        int rc = ins_init(&f, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }

        /* f.latlonh is only populated lazily by ins_finalize_init() on the
           first ins_update() call, so right after ins_init() it still reads
           the memset zero. Derive lat/height from the known init ECEF
           instead of the not-yet-populated field. */
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        float g_vec[3];
        ins_gravity_ned((float)lat, (float)h, g_vec);
        /* Bias along z: the accelerometer side only observes the
           component along gravity (see REQ-NAV-050). */
        const float acc_body[3] = {0.0f, 0.0f, -g_vec[2] + (biased ? acc_bias : 0.0f)};
        const float gyr_body[3] = {biased ? gyr_bias : 0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 1200; ++step) /* 12 s: two evaluation windows */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }

        const ins_diag_t* d = ins_get_diag(&f);
        if (biased)
        {
            if (d->n_acc_bias_prior_exceeded > 0 && d->n_gyr_bias_prior_exceeded > 0)
            {
                printf("  ok    bias outside the prior flagged (acc %u, gyro %u windows)\n",
                       d->n_acc_bias_prior_exceeded, d->n_gyr_bias_prior_exceeded);
            }
            else
            {
                printf("  FAIL  bias outside the prior not flagged (acc %u, gyro %u)\n",
                       d->n_acc_bias_prior_exceeded, d->n_gyr_bias_prior_exceeded);
                fails++;
            }
            /* Diagnostic only: the detector itself must be unaffected. */
            if (d->n_auto_zupt == 0)
            {
                printf("  FAIL  auto-ZUPT stopped firing under the check\n");
                fails++;
            }
        }
        else
        {
            if (d->n_acc_bias_prior_exceeded == 0 && d->n_gyr_bias_prior_exceeded == 0)
            {
                printf("  ok    clean IMU inside the prior, no false alarm\n");
            }
            else
            {
                printf("  FAIL  false alarm on a clean IMU (acc %u, gyro %u)\n",
                       d->n_acc_bias_prior_exceeded, d->n_gyr_bias_prior_exceeded);
                fails++;
            }
        }
    }
}

/* Non-finite measurement inputs (NaN/Inf from a flaky sensor bus) must be
 * dropped at the ins_update boundary, NOT reach the fusion math, where
 * they would slip through the chi2/variance gates (NaN comparisons are
 * always false), poison the state and force a permanent health-check
 * shutdown. After each corrupt sample the filter must remain ready and
 * keep working on good data. */
static void scenario_nan_inf_inputs(void)
{
    printf("\n=== Scenario 28: non-finite inputs dropped, filter survives ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float        dt = 0.01f;
    ins_time_us_t      t  = 0;
    ins_measurements_t m;
    int                i;

#define GOOD_EPOCHS(N)                           \
    do {                                         \
        for (i = 0; i < (N); ++i)                \
        {                                        \
            t += us_from_sec(dt);                \
            memset(&m, 0, sizeof(m));            \
            m.timestamp = t;                     \
            set_imu(&m, acc_body, gyr_body, dt); \
            ins_update(&f, &m);                  \
        }                                        \
    } while (0)

    GOOD_EPOCHS(100);

    /* 1: NaN accelerometer sample */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.acc.data[1] = NAN;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 2: Inf gyroscope sample */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gyr.data[0] = INFINITY;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 3: NaN GNSS position (ECEF) */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid    = true;
    m.gnss_pos.xyz_ecef[0] = NAN;
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 3b/3c: same, but the OTHER two ECEF components (MC/DC: vec3d_finite()
       short-circuits on the first non-finite component, so component [0]
       alone never sensitizes [1]/[2] being checked). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
    m.gnss_pos.xyz_ecef[1] = NAN;
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
    m.gnss_pos.xyz_ecef[2] = INFINITY;
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 4: finite GNSS position but NaN covariance */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
    m.gnss_pos.Qll_ned[0] = NAN;
    m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 5: NaN magnetometer */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.mag.is_valid    = true;
    m.mag.data[0]     = NAN;
    m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 6: NaN yaw aiding */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.yaw.is_valid   = true;
    m.yaw.yaw_rad    = NAN;
    m.yaw.stddev_rad = 0.01f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 7: NaN local position */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.local_pos.is_valid   = true;
    m.local_pos.pos_ned[2] = NAN;
    m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.01f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 8: NaN strapdown dt */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.strapdown_dt_sec = NAN;
    ins_update(&f, &m);
    GOOD_EPOCHS(100);

    /* 9: GNSS velocity with a NaN component (velocity block dropped). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_vel.is_valid   = true;
    m.gnss_vel.vel_ned[1] = NAN;
    m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 10: NaN GNSS pos/vel cross-covariance (zeroed, counted). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_Qll_pos_vel_ned[0] = INFINITY;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 11: NaN GNSS lever arm (zeroed, counted). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_leverarm_b[2] = NAN;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 12: NaN local-position lever arm (zeroed, counted). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.local_pos_leverarm_b[0] = NAN;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 13: NaN attitude-hint roll (whole hint dropped, counted). The hint
       feeds the nominal quaternion directly, so this one would poison the
       state rather than cost a fusion. */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.att_hint.is_valid         = true;
    m.att_hint.roll_rad         = NAN;
    m.att_hint.pitch_rad        = 0.0f;
    m.att_hint.stddev_roll_rad  = 0.01f;
    m.att_hint.stddev_pitch_rad = 0.01f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 14: NaN attitude-hint gyro bias (whole hint dropped, counted). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.att_hint.is_valid               = true;
    m.att_hint.gyr_bias_rps[2]        = NAN;
    m.att_hint.stddev_gyr_bias_rps[2] = 0.001f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

    /* 15: NaN attitude-hint yaw (only the yaw part of the hint dropped,
       counted; roll/pitch survive). */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.att_hint.is_valid         = true;
    m.att_hint.roll_rad         = 0.0f;
    m.att_hint.pitch_rad        = 0.0f;
    m.att_hint.stddev_roll_rad  = 0.01f;
    m.att_hint.stddev_pitch_rad = 0.01f;
    m.att_hint.yaw_rad          = NAN;
    m.att_hint.stddev_yaw_rad   = 0.01f;
    ins_update(&f, &m);
    GOOD_EPOCHS(20);

#undef GOOD_EPOCHS

    if (ins_is_ready(&f)) { printf("  ok    filter still ready after the corrupt samples\n"); }
    else
    {
        printf("  FAIL  filter died on non-finite input\n");
        fails++;
    }

    const uint32_t n_bad = ins_get_diag(&f)->n_invalid_input;
    if (n_bad == 17) { printf("  ok    all 17 corrupt blocks counted (n_invalid_input)\n"); }
    else
    {
        printf("  FAIL  n_invalid_input = %u (expected 17)\n", n_bad);
        fails++;
    }

    /* Attitude must be unaffected (still level). */
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, 0.0, 1e-3, "roll still level [rad]");
    CHECK_NEAR(pitch, 0.0, 1e-3, "pitch still level [rad]");

    /* A good GNSS fix afterwards must still be fused. */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    ins_update(&f, &m);
    if (ins_get_diag(&f)->n_gnss_used == 1)
    {
        printf("  ok    good GNSS fix after the glitches is fused\n");
    }
    else
    {
        printf("  FAIL  good GNSS fix not fused (n_gnss_used=%u)\n", ins_get_diag(&f)->n_gnss_used);
        fails++;
    }
}

/* Tunnel passage at the ins level: coast on IMU after losing GNSS,
 * degrade is_ready after the coasting window, then cleanly re-acquire
 * from the first fix after the outage (position snaps to the fix,
 * attitude/biases are kept, filter is immediately ready again).
 * Also covers allow_unlimited_deadreckoning (window disabled). */
/* Re-acquisition far from the n-frame origin (REQ-NAV-023). The re-anchor
   used to build pos_local as a geodetic difference against the ORIGIN and let
   the caller map that back to latitude/longitude against the origin again -
   but the two directions were evaluated at different latitudes, so the round
   trip was not the identity. The gap is the curvature cross term it drops,
   d_north * d_east * tan(lat) / R_earth, which is metres at the origin and
   kilometres a hundred kilometres away. The filter then came out of the
   tunnel that far off with a metre-scale prior, and the chi2 outlier gate,
   working exactly as designed, let it back in only over tens of minutes.
   Working in steps from the position the filter already holds keeps every
   mapping short, and the absolute anchor is the fix itself. */
static void scenario_deadreckoning_reacquire_far_from_origin(void)
{
    printf("\n=== Scenario: re-acquisition stays exact far from the origin (REQ-NAV-023) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float        acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float        dt          = 0.01f;
    ins_time_us_t      t           = 0;
    ins_measurements_t m;
    int                i;

    /* 100 km north AND east of the origin: the dropped term is a product, so
       it needs both. A leg like this is an ordinary afternoon in a car. */
    const float dned_far[3] = {100.0e3f, 100.0e3f, 0.0f};
    double      llh0[3], fix_far[3];
    ins_ecef_to_latlonh(init.x_ecef, &llh0[0], &llh0[1], &llh0[2]);
    test_ecef_offset_ned(llh0, dned_far, fix_far);

#define FAR_EPOCHS(N, WITH_GNSS, ECEF)                                 \
    do {                                                               \
        for (i = 0; i < (N); ++i)                                      \
        {                                                              \
            t += us_from_sec(dt);                                      \
            memset(&m, 0, sizeof(m));                                  \
            m.timestamp = t;                                           \
            set_imu(&m, acc_body, gyr_body, dt);                       \
            if ((WITH_GNSS) && (i % 100 == 0)) /* 1 Hz */              \
            {                                                          \
                m.gnss_pos.is_valid = true;                            \
                memcpy(m.gnss_pos.xyz_ecef, ECEF, sizeof(double) * 3); \
                m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f; \
                m.gnss_pos.Qll_ned[8]                         = 1.0f;  \
            }                                                          \
            ins_update(&f, &m);                                        \
        }                                                              \
    } while (0)

    /* Aided phase at the origin, which is where the filter bootstraps. */
    FAR_EPOCHS(500, 1, init.x_ecef);
    CHECK_TRUE(ins_is_ready(&f), "ready while aided at the origin");

    /* Drive the 100 km in one step instead of in real time: pos_local and
       latlonh are the same position in two frames, so both move together
       (the production path that keeps them in step is the strapdown
       integration itself, covered by the other coasting scenarios). */
    f.state.pos_local[0] = dned_far[0];
    f.state.pos_local[1] = dned_far[1];
    f.state.pos_local[2] = dned_far[2];
    ins_ecef_to_latlonh(fix_far, &f.latlonh[0], &f.latlonh[1], &f.latlonh[2]);

    /* Let the coasting window expire, then hand the filter its first fix. */
    FAR_EPOCHS(1300, 0, init.x_ecef); /* 13 s */
    CHECK_TRUE(!ins_is_ready(&f), "not ready after the window expired");
    FAR_EPOCHS(101, 1, fix_far);
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");

    /* The whole point: the filter is AT the fix, not at the fix bent by the
       mapping. 0.1 m against an error that used to run into the kilometres. */
    double p_ecef[3];
    CHECK_TRUE(ins_get_position_ecef(&f, p_ecef), "ecef available");
    CHECK_NEAR(p_ecef[0], fix_far[0], 0.1, "re-anchored exactly on the fix (x)");
    CHECK_NEAR(p_ecef[1], fix_far[1], 0.1, "re-anchored exactly on the fix (y)");
    CHECK_NEAR(p_ecef[2], fix_far[2], 0.1, "re-anchored exactly on the fix (z)");

    /* And the local frame is still the one the caller has been speaking. */
    float pos[3];
    CHECK_TRUE(ins_get_position_local(&f, pos), "local position available");
    CHECK_NEAR(pos[0], dned_far[0], 1.0, "pos_local stays in the inherited frame (north)");
    CHECK_NEAR(pos[1], dned_far[1], 1.0, "pos_local stays in the inherited frame (east)");

    /* A residual this small is what lets the chi2 gate pass the next fixes
       instead of spending the rest of the trip downweighting them. */
    FAR_EPOCHS(300, 1, fix_far);
    CHECK_TRUE(ins_get_diag(&f)->last_gnss_pos_residual_m < 1.0f,
               "the fixes that follow are ordinary residuals, not outliers");
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "no further re-acquisitions");

#undef FAR_EPOCHS
}

static void scenario_deadreckoning_reacquire(void)
{
    printf("\n=== Scenario 29: coasting window + re-acquisition ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float        acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float        dt          = 0.01f;
    ins_time_us_t      t           = 0;
    ins_measurements_t m;
    int                i;

    /* GNSS fix 50 m north of the origin (for the re-acquisition). */
    double llh0[3], exit_ecef[3], dllh[3];
    ins_ecef_to_latlonh(init.x_ecef, &llh0[0], &llh0[1], &llh0[2]);
    const float dned_exit[3] = {50.0f, 0.0f, 0.0f};
    ins_dned_to_dlatlonh(dned_exit, llh0[0], llh0[2], dllh);
    ins_latlonh_to_ecef(llh0[0] + dllh[0], llh0[1] + dllh[1], llh0[2] + dllh[2], exit_ecef);

#define RUN_EPOCHS(N, WITH_GNSS, ECEF)                                 \
    do {                                                               \
        for (i = 0; i < (N); ++i)                                      \
        {                                                              \
            t += us_from_sec(dt);                                      \
            memset(&m, 0, sizeof(m));                                  \
            m.timestamp = t;                                           \
            set_imu(&m, acc_body, gyr_body, dt);                       \
            if ((WITH_GNSS) && (i % 100 == 0)) /* 1 Hz */              \
            {                                                          \
                m.gnss_pos.is_valid = true;                            \
                memcpy(m.gnss_pos.xyz_ecef, ECEF, sizeof(double) * 3); \
                m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f; \
                m.gnss_pos.Qll_ned[8]                         = 1.0f;  \
            }                                                          \
            ins_update(&f, &m);                                        \
        }                                                              \
    } while (0)

    /* Aided phase (3 s, past warm-up). */
    RUN_EPOCHS(300, 1, init.x_ecef);
    CHECK_TRUE(ins_is_ready(&f), "ready while aided");
    CHECK_TRUE(ins_deadreckoning_ms(&f) <= 1100, "dr age tracks aiding");

    /* Tunnel entry: coasting within the window keeps the solution. */
    RUN_EPOCHS(500, 0, init.x_ecef); /* 5 s outage */
    CHECK_TRUE(ins_is_ready(&f), "still ready while coasting (5 s)");
    CHECK_TRUE(ins_deadreckoning_ms(&f) >= 4900, "dr age grows");

    /* Beyond the window the position solution is declared gone. */
    RUN_EPOCHS(700, 0, init.x_ecef); /* 12 s total outage */
    CHECK_TRUE(!ins_is_ready(&f), "not ready after window expired");
    CHECK_TRUE(f.is_initialized, "filter alive (attitude keeps running)");

    /* Tunnel exit: first fix re-anchors, filter immediately ready. */
    RUN_EPOCHS(101, 1, exit_ecef);
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");
    CHECK_TRUE(ins_is_ready(&f), "ready again right after the fix");

    float pos[3];
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[0], 50.0, 1.0, "position snapped to fix (north) [m]");
    CHECK_NEAR(pos[1], 0.0, 1.0, "position snapped to fix (east) [m]");

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, 0.0, 0.02, "attitude survived the outage (roll)");
    CHECK_NEAR(pitch, 0.0, 0.02, "attitude survived the outage (pitch)");

    /* Continued fusion works on the fresh anchor. */
    RUN_EPOCHS(200, 1, exit_ecef);
    CHECK_TRUE(ins_is_ready(&f), "stays ready with aiding");
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "no further re-acquisitions");

    /* Variant: unlimited dead reckoning disables the window. */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        opt.allow_unlimited_deadreckoning = true;
        rc                                = ins_init(&f2, &init, &opt);
        if (rc != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_time_us_t t2 = 0;
        for (i = 0; i < 1500; ++i) /* 15 s, never aided */
        {
            t2 += us_from_sec(dt);
            memset(&m, 0, sizeof(m));
            m.timestamp = t2;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f2, &m);
        }
        CHECK_TRUE(ins_is_ready(&f2), "unlimited deadreckoning: ready without aiding");
    }
#undef RUN_EPOCHS
}

/* REQ-NAV-022 (freeze): once the coasting window expires, ins_update() must
 * stop consuming IMU samples entirely -- attitude/bias/position/velocity
 * hold at their last value until re-acquisition, instead of continuing to
 * dead-reckon through an outage of unknown length. Drives a constant yaw
 * rate through the whole outage: while still inside the window the yaw
 * keeps turning (IMU still consumed); past the window it must stop dead in
 * its tracks even though the same non-zero gyro keeps arriving. */
static void scenario_deadreckoning_freeze(void)
{
    printf("\n=== Scenario: coasting window freeze, state and covariance both "
           "(REQ-NAV-022, REQ-NAV-064) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float        acc_body[3]  = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_still[3] = {0.0f, 0.0f, 0.0f};
    const float        gyr_turn[3]  = {0.0f, 0.0f, (float)(10.0 * M_PI / 180.0)}; /* 10 deg/s yaw */
    const float        dt           = 0.01f;
    ins_time_us_t      t            = 0;
    ins_measurements_t m;
    int                i;

    /* Aided phase (3 s, past warm-up). */
    for (i = 0; i < 300; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_still, dt);
        if (i % 100 == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        ins_update(&f, &m);
    }

    /* Tunnel entry: 5 s of constant yaw rate, still inside the window. */
    for (i = 0; i < 500; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_turn, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(ins_is_ready(&f), "still ready 5 s into the outage");
    float roll, pitch, yaw_5s;
    ins_get_rpy(&f, &roll, &pitch, &yaw_5s);
    CHECK_NEAR(yaw_5s, 5.0 * 10.0 * M_PI / 180.0, 0.05,
               "yaw kept turning inside the window (IMU still consumed)");

    /* Push well past the window (12 s total outage), same turn rate. */
    for (i = 0; i < 700; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_turn, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "not ready past the window");
    float yaw_12s;
    ins_get_rpy(&f, &roll, &pitch, &yaw_12s);
    const float var_pn_12s  = test_state_variance(&f, INS_IDX_POS + 0);
    const float var_yaw_12s = test_state_variance(&f, INS_IDX_RPY + 2);

    /* Keep feeding the same non-zero gyro for another 3 s: a filter that
       still ran the strapdown here would keep turning; REQ-NAV-022
       requires the nominal state to be frozen instead. */
    for (i = 0; i < 300; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_turn, dt);
        ins_update(&f, &m);
    }
    float yaw_15s;
    ins_get_rpy(&f, &roll, &pitch, &yaw_15s);
    CHECK_NEAR(yaw_15s, yaw_12s, 1e-5,
               "yaw frozen after the window expired despite ongoing rotation");
    CHECK_TRUE(fabsf(yaw_15s - yaw_5s) > 0.01f,
               "sanity: freeze point differs from the still-coasting yaw");

    /* REQ-NAV-064: the covariance is frozen with the state. Nothing is
       fused while the window is expired, so there is nothing for a
       propagation to balance out, and the confidence lost over the outage
       is priced into the re-acquisition instead (REQ-NAV-065). */
    CHECK_NEAR(test_state_variance(&f, INS_IDX_POS + 0), var_pn_12s, 1e-9,
               "position variance frozen with the state");
    CHECK_NEAR(test_state_variance(&f, INS_IDX_RPY + 2), var_yaw_12s, 1e-9,
               "yaw variance frozen with the state");
}

/* The boundary REQ-NAV-064 draws, from the other side: losing aiding is not
 * what stops the filter, the window EXPIRING is. Inside the window an outage
 * is ordinary coasting -- the time update runs and every non-position channel
 * keeps fusing, which is what makes the barometric height follow a climb
 * through a short GNSS gap. One run crosses the boundary so the two halves
 * cannot drift apart: a gate widened to "no aiding" instead of "window
 * expired" would go inert on the first dropped fix and every other scenario
 * would still pass. */
static void scenario_coasting_window_still_fuses(void)
{
    printf("\n=== Scenario: inside the coasting window the filter still runs "
           "(REQ-NAV-022, REQ-NAV-064) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init                     = true;
    opt.allow_unlimited_deadreckoning = false;
    opt.max_deadreckoning_sec         = 10.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;
    int           i;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    const uint32_t predicts_start = ins_get_diag(&f)->n_predict;
    const uint32_t baro_start     = ins_get_diag(&f)->n_baro_height_used;
    const float    var_pn_start   = test_state_variance(&f, INS_IDX_POS + 0);

    /* 8 s without any position aiding, still inside the 10 s window, while
       the platform climbs at 0.5 m/s. The barometer reports at 10 Hz. */
    const float climb_mps = 0.5f;
    for (i = 0; i < 800; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 10) == 0)
        {
            set_baro(&m, pressure_from_altitude(climb_mps * (float)(i + 1) * dt), 1.0f);
        }
        ins_update(&f, &m);
    }

    CHECK_TRUE(ins_is_ready(&f), "still ready inside the window");
    CHECK_TRUE(ins_get_diag(&f)->n_predict > predicts_start + 700u,
               "time update kept running inside the window");
    CHECK_TRUE(test_state_variance(&f, INS_IDX_POS + 0) > var_pn_start,
               "covariance kept growing inside the window");
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used > baro_start + 70u,
               "barometric height kept fusing inside the window");
    float pos[3];
    ins_get_position_local(&f, pos);
    /* Deliberately not a match against the 4 m of true climb: over 8 s a
       vertical channel fed only by a 1 m barometer lags a ramp by a good
       part of it. What is under test is that the height MOVED with the
       platform, which a channel shut off at the first dropped fix could
       not do. */
    CHECK_TRUE(-pos[2] > 1.5f, "height followed the climb through the gap");
    CHECK_TRUE(-pos[2] < climb_mps * 800.0f * dt + 0.5f, "height did not overshoot the climb");

    /* Cross the boundary (expiry lands at 10 s, mid-run) without measuring,
       then measure a stretch that is entirely past it. */
    for (i = 800; i < 1200; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 10) == 0)
        {
            set_baro(&m, pressure_from_altitude(climb_mps * (float)(i + 1) * dt), 1.0f);
        }
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "not ready past the window");

    const uint32_t predicts_frozen = ins_get_diag(&f)->n_predict;
    const uint32_t baro_frozen     = ins_get_diag(&f)->n_baro_height_used;
    for (i = 1200; i < 1700; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if ((i % 10) == 0)
        {
            set_baro(&m, pressure_from_altitude(climb_mps * (float)(i + 1) * dt), 1.0f);
        }
        ins_update(&f, &m);
    }
    CHECK_TRUE(ins_get_diag(&f)->n_predict == predicts_frozen,
               "time update stopped once the window expired");
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used == baro_frozen,
               "barometric fusion stopped once the window expired");
}

/* REQ-NAV-064 head-on: while the window is expired, every aiding channel is
 * shut, not just the strapdown. The epoch fed here carries a magnetometer, a
 * barometer disagreeing with the state by 50 m, and both a zero-velocity and
 * a zero-rotation update -- each of which would visibly move the state or
 * shrink the covariance on a live epoch. Nothing may move. */
static void scenario_frozen_no_fusion(void)
{
    printf("\n=== Scenario: no fusion of any kind while inert (REQ-NAV-064) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init                     = true;
    opt.allow_unlimited_deadreckoning = false;
    opt.max_deadreckoning_sec         = 1.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    double lat, lon, h;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
    float g_vec[3];
    ins_gravity_ned((float)lat, (float)h, g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;
    int           i, k;

    baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, true, false);
    CHECK_TRUE(f.is_initialized && f.height_from_baro, "bootstrapped with barometric height");

    /* Past the 1 s window on IMU alone. */
    for (i = 0; i < 300; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "inert past the window");

    ins_state_t state_before = f.state;
    float       var_before[INS_UNKNOWNS_MAX];
    for (k = 0; k < f.n; ++k) { var_before[k] = test_state_variance(&f, k); }

    /* One epoch carrying everything the filter would normally act on. */
    for (i = 0; i < 100; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        set_baro(&m, pressure_from_altitude(50.0f), 1.0f);
        m.mag.data[0]     = init.magnetic_n[0];
        m.mag.data[1]     = init.magnetic_n[1] + 10.0f;
        m.mag.data[2]     = init.magnetic_n[2];
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
        m.mag.is_valid                                            = true;
        m.zero_velocity_update                                    = true;
        m.zero_rotation_update                                    = true;
        ins_update(&f, &m);
    }

    CHECK_TRUE(memcmp(&state_before, &f.state, sizeof(ins_state_t)) == 0,
               "nominal state untouched by a full measurement epoch while inert");
    bool cov_frozen = true;
    for (k = 0; k < f.n; ++k)
    {
        if (fabsf(test_state_variance(&f, k) - var_before[k]) > 1e-12f) { cov_frozen = false; }
    }
    CHECK_TRUE(cov_frozen, "covariance untouched by a full measurement epoch while inert");
    CHECK_TRUE(ins_get_diag(&f)->n_baro_height_used == 0u, "barometer never fused while inert");
}

/* REQ-NAV-065: what the states carried across the freeze cost. Nothing is
 * propagated while inert (REQ-NAV-064), so the attitude and bias variances
 * would otherwise re-acquire with the confidence they earned before the
 * outage. They are inflated once, by the frozen duration, and clamped at
 * the configured initial priors -- and an external hint (REQ-NAV-048) still
 * overrides the result. */
static void scenario_reacquire_variance_inflation(void)
{
    printf("\n=== Scenario: re-acquisition inflates the carried states "
           "(REQ-NAV-065) ===\n");

    const float hint_rp_stddev = (float)(0.2 * M_PI / 180.0);
    double      lat, lon, h;
    float       g_vec[3];
    int         i, part;

    /* part 0: no hint, ordinary noise -> the carried variances grow and stay
       under their priors. part 1: the same outage with a roll/pitch hint on
       the re-acquiring fix, which must override the inflation. part 2: a gyro
       noisy enough that 99 s of attitude growth overshoots the roll/pitch
       prior -> the clamp. part 3: the 18-state configuration, whose
       magnetometer bias states have to be priced the same way -- they are
       carried across the freeze exactly like the IMU biases. */
    for (part = 0; part < 4; ++part)
    {
        ins_init_t init;
        fill_default_init(&init, 0);

        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init                     = true;
        opt.allow_unlimited_deadreckoning = false;
        opt.max_deadreckoning_sec         = 1.0f;
        if (part == 3) { opt.estimate_mag_bias = true; }

        ins_t f;
        memset(&f, 0, sizeof(f));
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
        const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
        const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
        const float   dt          = 0.01f;
        ins_time_us_t t           = 0;

        baro_height_bootstrap(&f, &init, &t, acc_body, gyr_body, dt, false, false);
        CHECK_TRUE(f.is_initialized, "bootstrapped");
        if (part == 3) { CHECK_TRUE(f.n == 18, "18-state mode active"); }

        /* Aided phase: the carried variances have to come down off their
           priors first, otherwise the clamp would bind immediately and
           "grew" could not be told apart from "was already at the cap". */
        for (i = 0; i < 3000; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            if ((i % 20) == 0)
            {
                m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
                m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
                m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
                m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
                m.gnss_pos.is_valid                                                   = true;
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
                m.gnss_vel.is_valid                                                   = true;
            }
            if (part == 3)
            {
                /* The mag bias states only come down off their prior once
                   they are actually observed, and only then is a later
                   growth distinguishable from "was at the cap all along". */
                m.mag.data[0]     = init.magnetic_n[0];
                m.mag.data[1]     = init.magnetic_n[1];
                m.mag.data[2]     = init.magnetic_n[2];
                m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
                m.mag.is_valid                                            = true;
            }
            ins_update(&f, &m);
        }
        CHECK_TRUE(ins_is_ready(&f), "ready after the aided phase");

        /* 100 s of outage: 1 s coasted, 99 s inert. */
        for (i = 0; i < 10000; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        CHECK_TRUE(!ins_is_ready(&f), "inert after 100 s without aiding");

        const float var_rp_frozen  = test_state_variance(&f, INS_IDX_RPY + 0);
        const float var_acc_frozen = test_state_variance(&f, INS_IDX_ACC + 0);
        const float var_gyr_frozen = test_state_variance(&f, INS_IDX_GYR + 0);
        const float var_mag_frozen = (part == 3) ? test_state_variance(&f, INS_IDX_MAG + 0) : 0.0f;

        /* The fix that ends it. */
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
        m.gnss_pos.is_valid                                                   = true;
        if (part == 1)
        {
            m.att_hint.is_valid         = true;
            m.att_hint.roll_rad         = 0.0f;
            m.att_hint.pitch_rad        = 0.0f;
            m.att_hint.stddev_roll_rad  = hint_rp_stddev;
            m.att_hint.stddev_pitch_rad = hint_rp_stddev;
        }
        if (part == 2)
        {
            /* The attitude inflation runs at the gyro noise this epoch
               reports: 1e-3 rad^2/s over 99 s overshoots the 5 deg prior
               many times over. */
            m.gyr.Qll_diag[0] = m.gyr.Qll_diag[1] = m.gyr.Qll_diag[2] = 1e-3f;
        }
        ins_update(&f, &m);
        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");

        const float var_rp  = test_state_variance(&f, INS_IDX_RPY + 0);
        const float var_acc = test_state_variance(&f, INS_IDX_ACC + 0);
        const float var_gyr = test_state_variance(&f, INS_IDX_GYR + 0);

        CHECK_TRUE(var_acc > var_acc_frozen, "accel bias variance grew over the frozen interval");
        CHECK_TRUE(var_gyr > var_gyr_frozen, "gyro bias variance grew over the frozen interval");
        CHECK_TRUE(var_acc < 0.1f * 0.1f, "accel bias variance still under its own prior");
        CHECK_TRUE(var_gyr < (float)(0.01 * M_PI / 180.0) * (float)(0.01 * M_PI / 180.0),
                   "gyro bias variance still under its own prior");

        if (part == 3)
        {
            const float var_mag = test_state_variance(&f, INS_IDX_MAG + 0);
            const float cap     = f.init.mag_bias_init_stddev_ut * f.init.mag_bias_init_stddev_ut;
            CHECK_TRUE(var_mag > var_mag_frozen, "mag bias variance grew over the frozen interval");
            CHECK_TRUE(var_mag <= cap * 1.001f, "mag bias variance capped at its own prior");
        }

        if (part == 1)
        {
            /* REQ-NAV-048 is applied after the inflation and overwrites it. */
            CHECK_NEAR(var_rp, hint_rp_stddev * hint_rp_stddev, 1e-12,
                       "hinted roll/pitch variance wins over the inflation");
        }
        else if (part == 2)
        {
            const double rp_prior = (5.0 * M_PI / 180.0) * (5.0 * M_PI / 180.0);
            CHECK_NEAR(var_rp, rp_prior, 1e-9, "roll/pitch variance clamped at the init prior");
        }
        else
        {
            CHECK_TRUE(var_rp > var_rp_frozen, "roll/pitch variance grew with no hint");
            CHECK_TRUE(var_rp < (float)(5.0 * M_PI / 180.0) * (float)(5.0 * M_PI / 180.0),
                       "roll/pitch variance still under its own prior");
        }
    }
}

/* An external attitude/gyro-bias hint (REQ-NAV-048) overrides the
 * otherwise-kept dead-reckoned attitude/bias at re-acquisition. Same
 * coasting-window setup as scenario_deadreckoning_reacquire; the hint is
 * only attached to the re-acquiring fix. */
static void scenario_reacquire_att_hint(void)
{
    printf("\n=== Scenario: re-acquisition attitude/gyro-bias hint (REQ-NAV-048) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float        acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float        dt          = 0.01f;
    ins_time_us_t      t           = 0;
    ins_measurements_t m;
    int                i;

    for (i = 0; i < 300; ++i) /* aided phase, 3 s */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (i % 100 == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        ins_update(&f, &m);
    }
    for (i = 0; i < 1200; ++i) /* coast past the 10 s window */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "window expired before re-acquire");

    const float roll_hint     = 12.0f * (float)M_PI / 180.0f;
    const float pitch_hint    = -7.0f * (float)M_PI / 180.0f;
    const float gbias_hint[3] = {0.02f, -0.01f, 0.003f}; /* rad/s */

    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.att_hint.is_valid                           = true;
    m.att_hint.roll_rad                           = roll_hint;
    m.att_hint.pitch_rad                          = pitch_hint;
    m.att_hint.stddev_roll_rad                    = 0.05f;
    m.att_hint.stddev_pitch_rad                   = 0.06f;
    for (i = 0; i < 3; ++i)
    {
        m.att_hint.gyr_bias_rps[i]        = gbias_hint[i];
        m.att_hint.stddev_gyr_bias_rps[i] = 0.001f;
    }
    ins_update(&f, &m);

    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");
    CHECK_TRUE(ins_is_ready(&f), "ready again right after re-acquisition");

    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, roll_hint, 1e-4, "att_hint roll wins over kept dead-reckoned attitude");
    CHECK_NEAR(pitch, pitch_hint, 1e-4, "att_hint pitch wins over kept dead-reckoned attitude");

    float gbias[3];
    ins_get_bias_gyr(&f, gbias);
    CHECK_NEAR(gbias[0], gbias_hint[0], 1e-6, "att_hint gyro bias x wins at re-acquisition");
    CHECK_NEAR(gbias[1], gbias_hint[1], 1e-6, "att_hint gyro bias y wins at re-acquisition");
    CHECK_NEAR(gbias[2], gbias_hint[2], 1e-6, "att_hint gyro bias z wins at re-acquisition");

    /* --- Regression: without a hint, attitude/bias are kept as before --- */
    {
        ins_t f2;
        memset(&f2, 0, sizeof(f2));
        int rc2 = ins_init(&f2, &init, &opt);
        if (rc2 != 0)
        {
            printf("init2 failed\n");
            fails++;
            return;
        }

        ins_time_us_t t2 = 0;
        for (i = 0; i < 300; ++i)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            if (i % 100 == 0)
            {
                m2.gnss_pos.is_valid = true;
                memcpy(m2.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
                m2.gnss_pos.Qll_ned[0] = m2.gnss_pos.Qll_ned[4] = 0.25f;
                m2.gnss_pos.Qll_ned[8]                          = 1.0f;
            }
            ins_update(&f2, &m2);
        }
        for (i = 0; i < 1200; ++i)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m2;
            memset(&m2, 0, sizeof(m2));
            m2.timestamp = t2;
            set_imu(&m2, acc_body, gyr_body, dt);
            ins_update(&f2, &m2);
        }
        t2 += us_from_sec(dt);
        ins_measurements_t m2;
        memset(&m2, 0, sizeof(m2));
        m2.timestamp = t2;
        set_imu(&m2, acc_body, gyr_body, dt);
        m2.gnss_pos.is_valid = true;
        memcpy(m2.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
        m2.gnss_pos.Qll_ned[0] = m2.gnss_pos.Qll_ned[4] = 0.25f;
        m2.gnss_pos.Qll_ned[8]                          = 1.0f;
        /* m2.att_hint left zeroed */
        ins_update(&f2, &m2);

        float roll2, pitch2, yaw2;
        ins_get_rpy(&f2, &roll2, &pitch2, &yaw2);
        CHECK_NEAR(roll2, 0.0, 0.02, "no hint: attitude kept through re-acquisition (roll)");
        CHECK_NEAR(pitch2, 0.0, 0.02, "no hint: attitude kept through re-acquisition (pitch)");
    }
}

/* Drive one filter instance up to the brink of a re-acquisition: aided for
 * 3 s (so the yaw variance converges to something tight), then coasting
 * 12 s past the 10 s window, so the nominal state is frozen and the next
 * usable fix will re-anchor. Shared by the three re-acquisition variants in
 * scenario_reacquire_yaw_unknown below, which differ only in the att_hint
 * attached to that fix. Returns the timestamp to continue from. */
static ins_time_us_t reacquire_yaw_setup(ins_t* f, const ins_init_t* init, const ins_options_t* opt,
                                         float dt)
{
    memset(f, 0, sizeof(*f));
    if (ins_init(f, init, opt) != 0)
    {
        printf("init failed\n");
        fails++;
        return 0;
    }

    float g_vec[3];
    ins_gravity_ned((float)f->latlonh[0], (float)f->latlonh[2], g_vec);
    const float        acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    ins_time_us_t      t           = 0;
    ins_measurements_t m;
    int                i;

    for (i = 0; i < 300; ++i) /* aided, 3 s */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (i % 10 == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init->x_ecef, sizeof(double) * 3);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
            m.gnss_vel.is_valid                           = true;
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        }
        ins_update(f, &m);
    }
    for (i = 0; i < 1200; ++i) /* coast 12 s, past the 10 s window */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(f, &m);
    }
    return t;
}

/* Attach a re-acquiring GNSS fix (plus whatever hint the caller filled in)
 * and step the filter once. */
static void reacquire_yaw_fix(ins_t* f, ins_measurements_t* m, const ins_init_t* init, float dt)
{
    float g_vec[3];
    ins_gravity_ned((float)f->latlonh[0], (float)f->latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    set_imu(m, acc_body, gyr_body, dt);
    m->gnss_pos.is_valid = true;
    memcpy(m->gnss_pos.xyz_ecef, init->x_ecef, sizeof(double) * 3);
    m->gnss_pos.Qll_ned[0] = m->gnss_pos.Qll_ned[4] = 0.25f;
    m->gnss_pos.Qll_ned[8]                          = 1.0f;
    ins_update(f, m);
}

/* REQ-NAV-059: coming out of a frozen coasting window, the yaw estimate has
 * missed whatever the platform turned during the outage while its variance
 * only ever grew at the gyro-noise rate. Unless a hint supplies an absolute
 * heading, re-acquisition must widen that variance to "unknown" - otherwise
 * the following aiding pushes the heading correction into the gyro z bias
 * instead of into yaw. */
static void scenario_reacquire_yaw_unknown(void)
{
    printf("\n=== Scenario: re-acquisition yaw prior reset (REQ-NAV-059) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */

    const float dt          = 0.01f;
    const float unknown_var = (float)(M_PI * M_PI);

    /* --- (a) hint with roll/pitch + gyro bias but NO yaw: the nav_suite
       ARS case, which is what the field data ran into. --- */
    {
        ins_t         f;
        ins_time_us_t t = reacquire_yaw_setup(&f, &init, &opt, dt);
        if (t == 0) return;
        CHECK_TRUE(!ins_is_ready(&f), "window expired before re-acquire");

        const float var_yaw_frozen = test_state_variance(&f, INS_IDX_RPY + 2);
        CHECK_TRUE(var_yaw_frozen < 0.25f * unknown_var,
                   "sanity: the frozen yaw variance is far below the unknown prior");
        float roll_before, pitch_before, yaw_before;
        ins_get_rpy(&f, &roll_before, &pitch_before, &yaw_before);

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp                 = t + us_from_sec(dt);
        m.att_hint.is_valid         = true;
        m.att_hint.roll_rad         = 0.0f;
        m.att_hint.pitch_rad        = 0.0f;
        m.att_hint.stddev_roll_rad  = 0.05f;
        m.att_hint.stddev_pitch_rad = 0.05f;
        /* stddev_yaw_rad deliberately left 0: an ARS has no absolute yaw. */
        reacquire_yaw_fix(&f, &m, &init, dt);

        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran");
        CHECK_NEAR(test_state_variance(&f, INS_IDX_RPY + 2), unknown_var, 1e-3,
                   "no yaw hint: yaw variance reset to the unknown-heading prior");

        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, yaw_before, 1e-5, "yaw STATE untouched: only the confidence was wrong");

        /* Roll/pitch must not be dragged along: they come from the hint and
           keep its (tight) variance. */
        CHECK_TRUE(test_state_variance(&f, INS_IDX_RPY + 0) < 0.01f,
                   "roll variance stays at the hinted value");
        CHECK_TRUE(test_state_variance(&f, INS_IDX_RPY + 1) < 0.01f,
                   "pitch variance stays at the hinted value");
    }

    /* --- (b) no hint at all: same reset, the yaw is no better known. --- */
    {
        ins_t         f;
        ins_time_us_t t = reacquire_yaw_setup(&f, &init, &opt, dt);
        if (t == 0) return;

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t + us_from_sec(dt);
        reacquire_yaw_fix(&f, &m, &init, dt);

        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran (no hint)");
        CHECK_NEAR(test_state_variance(&f, INS_IDX_RPY + 2), unknown_var, 1e-3,
                   "no hint at all: yaw variance reset to the unknown-heading prior");
    }

    /* --- (c) hint WITH an absolute yaw (magnetometer AHRS, dual-antenna
       heading): real information, so it keeps its own variance and the
       hinted yaw wins. The reset must not widen over it. --- */
    {
        ins_t         f;
        ins_time_us_t t = reacquire_yaw_setup(&f, &init, &opt, dt);
        if (t == 0) return;

        const float yaw_hint    = 1.2f; /* rad */
        const float yaw_hint_sd = 0.1f;

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp                 = t + us_from_sec(dt);
        m.att_hint.is_valid         = true;
        m.att_hint.roll_rad         = 0.0f;
        m.att_hint.pitch_rad        = 0.0f;
        m.att_hint.stddev_roll_rad  = 0.05f;
        m.att_hint.stddev_pitch_rad = 0.05f;
        m.att_hint.yaw_rad          = yaw_hint;
        m.att_hint.stddev_yaw_rad   = yaw_hint_sd;
        reacquire_yaw_fix(&f, &m, &init, dt);

        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran (yaw hint)");
        float roll, pitch, yaw;
        ins_get_rpy(&f, &roll, &pitch, &yaw);
        CHECK_NEAR(yaw, yaw_hint, 1e-3, "hinted yaw wins over the kept one");
        CHECK_NEAR(test_state_variance(&f, INS_IDX_RPY + 2), yaw_hint_sd * yaw_hint_sd, 1e-4,
                   "hinted yaw keeps its own variance, not the unknown prior");
    }
}

/* REQ-NAV-023: a GNSS velocity-only fix must not un-expire the coasting
 * window on its own. A receiver coming out of a tunnel/urban canyon often
 * reports usable velocity a beat before usable position, if that alone
 * cleared the window, the first usable POSITION fix afterwards would run
 * through the ordinary residual fuse instead of ins_reacquire(). */
static void scenario_deadreckoning_velocity_only_no_reacquire(void)
{
    printf("\n=== Scenario: GNSS velocity-only fix must not arm re-acquisition "
           "(REQ-NAV-023) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* 10 s default window */

    int rc = ins_init(&f, &init, &opt);
    if (rc != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float        acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float        dt          = 0.01f;
    ins_time_us_t      t           = 0;
    ins_measurements_t m;
    int                i;

    /* GNSS fix 50 m north of the origin (for the re-acquisition). */
    double llh0[3], exit_ecef[3], dllh[3];
    ins_ecef_to_latlonh(init.x_ecef, &llh0[0], &llh0[1], &llh0[2]);
    const float dned_exit[3] = {50.0f, 0.0f, 0.0f};
    ins_dned_to_dlatlonh(dned_exit, llh0[0], llh0[2], dllh);
    ins_latlonh_to_ecef(llh0[0] + dllh[0], llh0[1] + dllh[1], llh0[2] + dllh[2], exit_ecef);

    /* Aided phase (3 s, past warm-up). */
    for (i = 0; i < 300; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (i % 100 == 0)
        {
            m.gnss_pos.is_valid = true;
            memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(double) * 3);
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
            m.gnss_pos.Qll_ned[8]                         = 1.0f;
        }
        ins_update(&f, &m);
    }

    /* Tunnel: 12 s of complete outage, well past the 10 s window. */
    for (i = 0; i < 1200; ++i)
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "window expired before the velocity-only phase");

    /* Tunnel exit, phase 1: GNSS velocity recovers first (good quality,
       well inside the fusion gate), position does not (not offered at
       all yet) -- exactly the receiver behaviour seen in the field. This
       must NOT clear ins_deadreckoning_ms()/re-arm ins_is_ready(). */
    for (i = 0; i < 100; ++i) /* 1 s */
    {
        t += us_from_sec(dt);
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_vel.is_valid   = true;
        m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        ins_update(&f, &m);
    }
    CHECK_TRUE(!ins_is_ready(&f), "velocity-only fixes do not re-arm readiness");
    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 0, "velocity-only fixes do not re-acquire");

    /* Phase 2: the first usable POSITION fix arrives, 50 m away from the
       frozen estimate. It must re-anchor in one step (REQ-NAV-023), not
       be fused as an ordinary (and here, hopeless) residual. */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, exit_ecef, sizeof(double) * 3);
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
    m.gnss_pos.Qll_ned[8]                         = 1.0f;
    m.gnss_vel.is_valid                           = true;
    m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
    ins_update(&f, &m);

    CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "re-acquisition ran on the first usable fix");
    CHECK_TRUE(ins_is_ready(&f), "ready again right after re-acquisition");

    float pos[3];
    ins_get_position_local(&f, pos);
    CHECK_NEAR(pos[0], 50.0, 1.0, "position snapped to the fix in one step (north) [m]");
    CHECK_NEAR(pos[1], 0.0, 1.0, "position snapped to the fix in one step (east) [m]");
}

/* One aided epoch for scenario_gnss_pos_decimation, returning true when
   the GNSS POSITION block was fused. last_gnss_pos_residual_m is written
   only on an epoch that fuses position, so a change in it identifies such
   an epoch - provided no two consecutive residuals coincide, which is
   what the walking north_offset_m of the caller guarantees. */
static bool decim_epoch(ins_t* f, ins_time_us_t t, const float acc_body[3],
                        const double origin_ecef[3], float north_offset_m, bool with_pos,
                        bool with_vel)
{
    const float        gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float        dt          = 0.01f;
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);

    if (with_pos)
    {
        double      llh[3], dllh[3];
        const float dned[3] = {north_offset_m, 0.0f, 0.0f};
        ins_ecef_to_latlonh(origin_ecef, &llh[0], &llh[1], &llh[2]);
        ins_dned_to_dlatlonh(dned, llh[0], llh[2], dllh);
        ins_latlonh_to_ecef(llh[0] + dllh[0], llh[1] + dllh[1], llh[2] + dllh[2],
                            m.gnss_pos.xyz_ecef);
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        m.gnss_pos.is_valid                           = true;
    }
    if (with_vel)
    {
        m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
        m.gnss_vel.is_valid                                                   = true;
    }

    const float before = f->diag.last_gnss_pos_residual_m;
    ins_update(f, &m);
    return with_pos && (fabsf(f->diag.last_gnss_pos_residual_m - before) > 0.0f);
}

/* Count the position epochs in a run of fixes offering both blocks.
   Returns the count and, through first_out, the index of the first one. */
static int decim_run(ins_t* f, ins_time_us_t* t, const float acc_body[3],
                     const double origin_ecef[3], int fixes, bool with_vel, int* first_out)
{
    const float dt    = 0.01f;
    int         count = 0;
    int         k, j;

    if (first_out) { *first_out = -1; }
    for (k = 0; k < fixes; ++k)
    {
        /* 10 Hz fixes on a 100 Hz IMU. The walking offset keeps every
           position residual distinct from the one before it. */
        for (j = 0; j < 9; ++j)
        {
            *t += us_from_sec(dt);
            (void)decim_epoch(f, *t, acc_body, origin_ecef, 0.0f, false, false);
        }
        *t += us_from_sec(dt);
        if (decim_epoch(f, *t, acc_body, origin_ecef, 2.0f + 0.1f * (float)k, true, with_vel))
        {
            if (first_out && *first_out < 0) { *first_out = k; }
            count++;
        }
    }
    return count;
}

/* GNSS position decimation (REQ-NAV-063): with both blocks usable in the
   same epoch, only one of them is fused - the position on every Nth such
   epoch, the velocity on the other N-1. Plus the three cases the
   decimation must keep its hands off: an epoch offering position alone,
   an expired coasting window, and a restart after a gap. */
static void scenario_gnss_pos_decimation(void)
{
    printf("\n=== Scenario: GNSS position decimation (REQ-NAV-063) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    double llh0[3];
    ins_ecef_to_latlonh(init.x_ecef, &llh0[0], &llh0[1], &llh0[2]);
    float g_vec[3];
    ins_gravity_ned((float)llh0[0], (float)llh0[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float dt          = 0.01f;

    /* (a) N = 5: one position epoch in every five that offer both. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.gnss_pos_decimation = 5;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (decimation 5)");

        ins_time_us_t t     = 0;
        int           first = -1;
        const int     n     = decim_run(&f, &t, acc_body, init.x_ecef, 20, true, &first);
        CHECK_TRUE(n == 4, "4 of 20 combined fixes fused the position");
        CHECK_TRUE(first == 0, "the cycle starts on a position");
    }

    /* (b) The opt-out restores the combined fuse: every fix carries a
           position again. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.gnss_pos_decimation = 1;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (decimation off)");

        ins_time_us_t t = 0;
        const int     n = decim_run(&f, &t, acc_body, init.x_ecef, 20, true, (int*)0);
        CHECK_TRUE(n == 20, "decimation off: every combined fix fuses the position");
    }

    /* (c) A fix offering the position alone is never decimated: there is
           no second view of the same solution to double-count. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.gnss_pos_decimation = 5;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (position-only stream)");

        ins_time_us_t t = 0;
        const int     n = decim_run(&f, &t, acc_body, init.x_ecef, 20, false, (int*)0);
        CHECK_TRUE(n == 20, "position-only fixes are never withheld");
    }

    /* (d) An expired coasting window suspends the decimation: the first
           fix out of the tunnel re-anchors (REQ-NAV-023) whatever the
           cycle phase, and the velocity-only epochs before it must not
           have refreshed the aiding timestamp on their own. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.gnss_pos_decimation           = 5;
        opt.allow_unlimited_deadreckoning = false; /* 10 s default window */
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (expired window)");

        ins_time_us_t t = 0;
        /* Leave the cycle mid-run, so a decimation still in force would
           withhold the position at the tunnel exit below. Exactly one of
           these four fixes carrying a position is what says the cycle is
           running and did not land back on its position slot. */
        const int n_before = decim_run(&f, &t, acc_body, init.x_ecef, 4, true, (int*)0);
        CHECK_TRUE(n_before == 1, "cycle left mid-run before the outage");

        int i;
        for (i = 0; i < 1200; ++i) /* 12 s outage, past the window */
        {
            t += us_from_sec(dt);
            (void)decim_epoch(&f, t, acc_body, init.x_ecef, 0.0f, false, false);
        }
        CHECK_TRUE(!ins_is_ready(&f), "window expired before the tunnel exit");

        t += us_from_sec(dt);
        (void)decim_epoch(&f, t, acc_body, init.x_ecef, 50.0f, true, true);
        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1,
                   "expired window re-acquires despite the cycle phase");

        float pos[3];
        ins_get_position_local(&f, pos);
        CHECK_NEAR(pos[0], 50.0, 1.0, "re-anchored on the withheld-phase fix (north) [m]");
    }

    /* (e) A gap in the position stream restarts the cycle at a position,
           and the withheld epochs before it still counted as position
           aiding, so a 10 s coasting window never came close to expiring
           on a 10 Hz stream decimated by 20. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.gnss_pos_decimation = 20;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init (stream gap)");

        ins_time_us_t t = 0;
        /* 15 combined fixes: past the first position of the cycle, well
           short of the next one at 20. */
        int       first = -1;
        const int n     = decim_run(&f, &t, acc_body, init.x_ecef, 15, true, &first);
        CHECK_TRUE(n == 1 && first == 0, "decimation 20: only the first of 15 fixes");
        /* 14 consecutive withheld positions have not opened a
           position-aiding gap: ins_deadreckoning_ms measures the time
           since the last position AIDING, which a withheld position still
           is (REQ-NAV-023). Without that a 10 Hz stream decimated by 20
           would coast the filter out of its window on a perfect fix
           stream. */
        CHECK_TRUE(ins_deadreckoning_ms(&f) <= 150,
                   "withheld positions still count as position aiding");

        int i;
        for (i = 0; i < 300; ++i) /* 3 s without a fix, past the 2 s bound */
        {
            t += us_from_sec(dt);
            (void)decim_epoch(&f, t, acc_body, init.x_ecef, 0.0f, false, false);
        }
        t += us_from_sec(dt);
        CHECK_TRUE(decim_epoch(&f, t, acc_body, init.x_ecef, 3.0f, true, true),
                   "the first fix after a gap carries the position");
    }
}

/* ins_init input validation: NULL arguments and a degenerate (near
   the Earth's centre) initial ECEF must be rejected with -1. */
static void scenario_init_validation(void)
{
    printf("\n=== Scenario: ins_init argument validation ===\n");
    ins_t      f;
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    CHECK_TRUE(ins_init((ins_t*)0, &init, &opt) == -1, "init rejects NULL filter");
    CHECK_TRUE(ins_init(&f, (const ins_init_t*)0, &opt) == -1, "init rejects NULL init");
    CHECK_TRUE(ins_init(&f, &init, (const ins_options_t*)0) == -1, "init rejects NULL options");

    ins_init_t bad = init;
    bad.x_ecef[0] = bad.x_ecef[1] = bad.x_ecef[2] = 0.0; /* norm 0 < 1000 m */
    CHECK_TRUE(ins_init(&f, &bad, &opt) == -1, "init rejects near-origin ECEF");
}

/* Every public accessor's own "!f" arm, tested elsewhere only with a
   valid-but-uninitialized filter (the "!f->is_initialized" half of the
   guard). */
static void scenario_accessor_null_guards(void)
{
    printf("\n=== Scenario: accessor NULL-filter guards ===\n");

    double pe[3];
    float  pl[3], v[3], q[4], rpy0, rpy1, rpy2, R[9], b[3], w[3], a[3];
    CHECK_TRUE(!ins_get_position_ecef((const ins_t*)0, pe), "position_ecef(NULL)");
    CHECK_TRUE(!ins_get_position_local((const ins_t*)0, pl), "position_local(NULL)");
    CHECK_TRUE(!ins_get_velocity_ned((const ins_t*)0, v), "velocity_ned(NULL)");
    CHECK_TRUE(!ins_get_velocity_ecef((const ins_t*)0, v), "velocity_ecef(NULL)");
    CHECK_TRUE(!ins_get_quaternion((const ins_t*)0, q), "quaternion(NULL)");
    CHECK_TRUE(!ins_get_rpy((const ins_t*)0, &rpy0, &rpy1, &rpy2), "rpy(NULL)");
    CHECK_TRUE(!ins_get_rpy_stddev((const ins_t*)0, &rpy0, &rpy1, &rpy2), "rpy_stddev(NULL)");
    CHECK_TRUE(!ins_get_rotmat_b_to_n((const ins_t*)0, R), "rotmat_b_to_n(NULL)");
    CHECK_TRUE(!ins_get_bias_acc((const ins_t*)0, b), "bias_acc(NULL)");
    CHECK_TRUE(!ins_get_bias_gyr((const ins_t*)0, b), "bias_gyr(NULL)");
    CHECK_TRUE(!ins_get_bias_mag((const ins_t*)0, b), "bias_mag(NULL)");
    CHECK_TRUE(!ins_get_omega_b_nb((const ins_t*)0, w), "omega_b_nb(NULL)");
    CHECK_TRUE(!ins_get_acc_n((const ins_t*)0, a), "acc_n(NULL)");

    /* Same accessors, valid-but-uninitialized filter this time -- the
       "!f->is_initialized" half of the guard, which the position/velocity/
       quaternion accessors above already exercise via other scenarios but
       these do not. */
    ins_t dead;
    memset(&dead, 0, sizeof(dead));
    CHECK_TRUE(!ins_get_rpy_stddev(&dead, &rpy0, &rpy1, &rpy2), "rpy_stddev(uninit)");
    CHECK_TRUE(!ins_get_rotmat_b_to_n(&dead, R), "rotmat_b_to_n(uninit)");
    CHECK_TRUE(!ins_get_bias_acc(&dead, b), "bias_acc(uninit)");
    CHECK_TRUE(!ins_get_bias_gyr(&dead, b), "bias_gyr(uninit)");
    CHECK_TRUE(!ins_get_bias_mag(&dead, b), "bias_mag(uninit)");
    CHECK_TRUE(!ins_get_omega_b_nb(&dead, w), "omega_b_nb(uninit)");
    CHECK_TRUE(!ins_get_acc_n(&dead, a), "acc_n(uninit)");

    /* Remaining NULL-filter guards, plus the "valid but uninitialized"
       half where the guard checks it. */
    ins_shutdown((ins_t*)0);                                            /* must not crash */
    ins_set_magnetic_model_from_position((ins_t*)0, 0.0, 0.0, 2025.0f); /* must not crash */
    ins_shift_origin_down((ins_t*)0, 1.0f);                             /* must not crash */
    ins_shift_origin_down(&dead, 1.0f);                                 /* must not crash */
    CHECK_TRUE(!ins_is_ready((const ins_t*)0), "is_ready(NULL)");
    CHECK_TRUE(ins_deadreckoning_ms((const ins_t*)0) == -1, "deadreckoning_ms(NULL)");
    CHECK_TRUE(ins_deadreckoning_ms(&dead) == -1, "deadreckoning_ms(uninit)");
    CHECK_TRUE(ins_get_diag((const ins_t*)0) == (const ins_diag_t*)0, "get_diag(NULL)");
    CHECK_TRUE(!ins_auto_zupt_active((const ins_t*)0), "auto_zupt_active(NULL)");
    ins_set_auto_zupt_disable((ins_t*)0, true); /* must not crash */

    /* ins_set_magnetic_model_from_position()'s isfinite chain: each
       condition independently, one bad field at a time. */
    ins_t good;
    memset(&good, 0, sizeof(good));
    ins_init_t init_g;
    fill_default_init(&init_g, 0);
    ins_options_t opt_g;
    fill_default_opt(&opt_g);
    CHECK_TRUE(ins_init(&good, &init_g, &opt_g) == 0, "init (mag model guard)");
    const float mag_before[3] = {good.magnetic_n[0], good.magnetic_n[1], good.magnetic_n[2]};
    ins_set_magnetic_model_from_position(&good, 0.85, (double)NAN, 2025.0f); /* NaN lon */
    CHECK_NEAR(good.magnetic_n[0], mag_before[0], 1e-9, "NaN lon dropped");
    ins_set_magnetic_model_from_position(&good, 0.85, 0.16, (float)NAN); /* NaN year */
    CHECK_NEAR(good.magnetic_n[0], mag_before[0], 1e-9, "NaN year dropped");
    ins_set_magnetic_model_from_position(&good, 0.85, 0.16, 1980.0f); /* year < 1990 */
    CHECK_NEAR(good.magnetic_n[0], mag_before[0], 1e-9, "year < 1990 dropped");

    /* ins_apply_calibration() / ins_gnss_condition_pos_cov() /
       ins_gnss_condition_vel_cov(): each own NULL-argument guard,
       every argument individually (elsewhere always called with all
       three/two valid). */
    ins_measurements_t mm;
    memset(&mm, 0, sizeof(mm));
    ins_apply_calibration((const ins_options_t*)0, &mm);   /* must not crash */
    ins_apply_calibration(&opt_g, (ins_measurements_t*)0); /* must not crash */

    float Qin[9] = {1.0f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f};
    float Qout[9];
    ins_gnss_condition_pos_cov((const ins_options_t*)0, Qin, Qout); /* must not crash */
    ins_gnss_condition_pos_cov(&opt_g, (const float*)0, Qout);      /* must not crash */
    ins_gnss_condition_pos_cov(&opt_g, Qin, (float*)0);             /* must not crash */
    ins_gnss_condition_vel_cov((const ins_options_t*)0, Qin, Qout); /* must not crash */
    ins_gnss_condition_vel_cov(&opt_g, (const float*)0, Qout);      /* must not crash */
    ins_gnss_condition_vel_cov(&opt_g, Qin, (float*)0);             /* must not crash */

    /* ins_get_bias_mag() on a plain 15-state filter (no estimate_mag_bias):
       elsewhere tested only with the 18-state config, so f->n <= INS_IDX_MAG
       (the "no mag-bias state at all" arm) never runs. Needs an actually
       initialized filter (ins_init() alone never sets is_initialized --
       both init modes defer that to the first ins_update()), or the
       preceding "!is_initialized" arm short-circuits before reaching it. */
    {
        ins_measurements_t mg;
        memset(&mg, 0, sizeof(mg));
        mg.timestamp           = 1000000;
        const float acc_lvl[3] = {0.0f, 0.0f, -9.80665f};
        const float gyr0[3]    = {0.0f, 0.0f, 0.0f};
        set_imu(&mg, acc_lvl, gyr0, 0.01f);
        ins_update(&good, &mg);
    }
    CHECK_TRUE(good.is_initialized, "15-state filter initialized");
    float bias_mag[3];
    CHECK_TRUE(!ins_get_bias_mag(&good, bias_mag), "bias_mag: no mag-bias state in 15-state mode");
}

/* Re-acquisition variants after an expired coasting window:
   (a) a GNSS fix carrying velocity re-anchors position AND velocity
       (exercises the velocity path of ins_reacquire), and
   (b) a local-position fix re-anchors from the same expired state. */
static void scenario_reacquire_variants(void)
{
    printf("\n=== Scenario: re-acquire with velocity / local position ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* enable the window */

    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* (a) GNSS re-acquire with velocity. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_time_us_t t = 0;
        int           i;
        /* Start aligned (REQ-NAV-033), then coast unaided so the window expires. */
        start_manual_filter(&f, &t, dt, init.x_ecef);
        for (i = 0; i < 1600; ++i) /* 16 s, no aiding -> window expires */
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        CHECK_TRUE(!ins_is_ready(&f), "window expired before re-acquire");

        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.is_valid = true;
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;
        m.gnss_pos.Qll_ned[8]                         = 1.0f;
        m.gnss_vel.is_valid                           = true;
        m.gnss_vel.vel_ned[0]                         = 1.5f; /* re-anchored velocity */
        m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.04f;
        ins_update(&f, &m);

        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "gnss+vel re-acquired");
        CHECK_TRUE(ins_is_ready(&f), "ready after gnss+vel re-acquire");
        float v[3];
        ins_get_velocity_ned(&f, v);
        CHECK_NEAR(v[0], 1.5f, 0.3, "velocity re-anchored from the fix [m/s]");
    }

    /* (b) local-position re-acquire. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_time_us_t t = 0;
        int           i;
        /* Start aligned (REQ-NAV-033), then coast unaided so the window expires. */
        start_manual_filter(&f, &t, dt, init.x_ecef);
        for (i = 0; i < 1600; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.local_pos.is_valid   = true;
        m.local_pos.pos_ned[0] = 12.0f;
        m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.04f;
        ins_update(&f, &m);

        CHECK_TRUE(ins_get_diag(&f)->n_reacquire == 1, "local-pos re-acquired");
        float p[3];
        ins_get_position_local(&f, p);
        CHECK_NEAR(p[0], 12.0f, 0.5, "local-pos re-anchored position [m]");
    }
}

/* Robustness edges of the delayed-measurement and gating paths. A delayed
   measurement whose time of validity predates the history cannot be
   anchored and is skipped. An over-noisy GNSS velocity is gated out.
   Yaw aiding near gimbal lock is skipped. */
static void scenario_delayed_no_anchor_and_gates(void)
{
    printf("\n=== Scenario: delayed no-anchor / gates / gimbal skip ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    ins_t f;
    memset(&f, 0, sizeof(f));
    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    ins_time_us_t t = 0;
    int           i;
    for (i = 0; i < 6; ++i) /* only ~60 ms of history so far */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* Delay of 450 ms points the time of validity well before any history
       entry -> the residual cannot be anchored -> skipped. */
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.is_valid = true;
    memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    m.gnss_delay_ms                                                       = 450;
    m.yaw.is_valid                                                        = true;
    m.yaw.yaw_rad                                                         = 0.1f;
    m.yaw.stddev_rad                                                      = 0.05f;
    m.yaw_delay_ms                                                        = 450;
    m.local_pos.is_valid                                                  = true;
    m.local_pos.Qll_ned[0] = m.local_pos.Qll_ned[4] = m.local_pos.Qll_ned[8] = 0.04f;
    m.local_pos_delay_ms                                                     = 450;
    ins_update(&f, &m);
    CHECK_TRUE(ins_get_diag(&f)->n_gnss_no_anchor >= 1,
               "delayed GNSS with no history anchor is skipped");
    CHECK_TRUE(ins_get_diag(&f)->n_gnss_used == 0,
               "no GNSS fusion happened on the un-anchored fix");

    /* Over-noisy GNSS velocity (stddev 2 m/s > 1 m/s gate) is rejected. */
    t += us_from_sec(dt);
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_vel.is_valid   = true;
    m.gnss_vel.vel_ned[0] = 0.0f;
    m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 4.0f;
    ins_update(&f, &m);
    CHECK_TRUE(ins_get_diag(&f)->n_gnss_rejected_noise >= 1,
               "over-noisy GNSS velocity is gated out");

    /* Yaw aiding near gimbal lock (pitch = 90 deg) is skipped. */
    {
        ins_init_t ginit      = init;
        ginit.rpy_init_rad[1] = 0.5f * (float)M_PI;
        ins_t fg;
        memset(&fg, 0, sizeof(fg));
        if (ins_init(&fg, &ginit, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_time_us_t tg = 0;
        for (i = 0; i < 5; ++i)
        {
            tg += us_from_sec(dt);
            ins_measurements_t mg;
            memset(&mg, 0, sizeof(mg));
            mg.timestamp = tg;
            set_imu(&mg, acc_body, gyr_body, dt);
            mg.yaw.is_valid   = true;
            mg.yaw.yaw_rad    = 0.2f;
            mg.yaw.stddev_rad = 0.05f;
            ins_update(&fg, &mg);
        }
        CHECK_TRUE(fg.is_initialized, "filter survives yaw aiding at gimbal lock");
    }
}

/* ZUPT / ZARU rate limiting and the manual zero-rotation path: with the
   auto detector disabled a manual zero-rotation update takes the
   "no stillness run" branch (raw gyro sample), and a second trigger within
   one Kalman period is rate-limited away for both updates. */
static void scenario_zupt_zaru_rate_limit(void)
{
    printf("\n=== Scenario: ZUPT/ZARU rate limit + manual ZARU ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_zupt_disable = true; /* no auto stillness accumulation */

    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    ins_t f;
    memset(&f, 0, sizeof(f));
    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("init failed\n");
        fails++;
        return;
    }

    ins_time_us_t t = 0;
    int           i;
    for (i = 0; i < 50; ++i) /* warm-up */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* First manual ZUPT + ZARU: both fuse (ZARU via the count==0 path). */
    t += us_from_sec(dt);
    ins_measurements_t m1;
    memset(&m1, 0, sizeof(m1));
    m1.timestamp = t;
    set_imu(&m1, acc_body, gyr_body, dt);
    m1.zero_velocity_update = true;
    m1.zero_rotation_update = true;
    ins_update(&f, &m1);

    /* Second trigger only 5 ms later -> both are rate-limited (skipped). */
    t += us_from_sec(0.005f);
    ins_measurements_t m2;
    memset(&m2, 0, sizeof(m2));
    m2.timestamp = t;
    set_imu(&m2, acc_body, gyr_body, 0.005f);
    m2.zero_velocity_update = true;
    m2.zero_rotation_update = true;
    ins_update(&f, &m2);

    CHECK_TRUE(ins_is_ready(&f), "filter healthy after rapid ZUPT/ZARU");
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    CHECK_NEAR(roll, 0.0, 1e-2, "ZUPT/ZARU kept roll level [rad]");
    CHECK_NEAR(pitch, 0.0, 1e-2, "ZUPT/ZARU kept pitch level [rad]");
}

/* Safety net: if the covariance or nominal state ever goes non-finite or
   negative, ins_check_health must deinitialize the filter instead of
   emitting garbage. Driven white-box (the test already reaches into the
   struct) by poking an internal, then running a quiescent update
   (same timestamp -> no predict, no measurements) so the health check
   evaluates the corrupted value. */
static void scenario_health_check_deinit(void)
{
    printf("\n=== Scenario: health check deinitializes on bad state ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    const char* names[4] = {"d[] NaN -> deinit", "d[] negative -> deinit", "state Inf -> deinit",
                            "quaternion NaN -> deinit"};
    int         kind;
    for (kind = 0; kind < 4; ++kind)
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            continue;
        }
        ins_time_us_t t = 0;
        int           i;
        for (i = 0; i < 30; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        CHECK_TRUE(f.is_initialized, "filter initialized before corruption");

        switch (kind)
        {
        case 0: f.d[4] = NAN; break;
        case 1: f.d[4] = -1.0f; break;
        case 2: f.state.vel_ned[1] = INFINITY; break;
        case 3: f.state.qbn[0] = NAN; break;
        }
        ins_measurements_t mq;
        memset(&mq, 0, sizeof(mq));
        mq.timestamp = t; /* same timestamp -> predict skipped, poke survives */
        ins_update(&f, &mq);

        CHECK_TRUE(!f.is_initialized, names[kind]);
    }

    /* A tiny-but-positive variance is floored to INS_MIN_VARIANCE and the
       filter keeps running (guards against denormals / singular covariance). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) == 0)
        {
            ins_time_us_t t = 0;
            int           i;
            for (i = 0; i < 30; ++i)
            {
                t += us_from_sec(dt);
                ins_measurements_t m;
                memset(&m, 0, sizeof(m));
                m.timestamp = t;
                set_imu(&m, acc_body, gyr_body, dt);
                ins_update(&f, &m);
            }
            f.d[4] = 1e-15f; /* below the 1e-12 floor, still positive */
            ins_measurements_t mq;
            memset(&mq, 0, sizeof(mq));
            mq.timestamp = t;
            ins_update(&f, &mq);
            CHECK_TRUE(f.is_initialized, "tiny variance floored, filter alive");
            CHECK_TRUE(f.d[4] >= 1e-13f, "variance raised to the floor");
        }
        else
        {
            printf("init failed\n");
            fails++;
        }
    }
}

/* Feed an auto-init (or unlimited-DR manual) filter to (re)bootstrap: static
   IMU-only epochs then a single IMU+GNSS fix at the anchor. Advances *t. */
static void feed_autoinit_bootstrap(ins_t* f, const double x_ecef[3], ins_time_us_t* t,
                                    const float acc_body[3], const float gyr_body[3], float dt)
{
    int step;
    for (step = 0; step < 20; ++step)
    {
        *t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = *t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(f, &m);
    }
    *t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = *t;
    set_imu(&m, acc_body, gyr_body, dt);
    m.gnss_pos.xyz_ecef[0] = x_ecef[0];
    m.gnss_pos.xyz_ecef[1] = x_ecef[1];
    m.gnss_pos.xyz_ecef[2] = x_ecef[2];
    m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
    m.gnss_pos.is_valid                                                   = true;
    m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
    m.gnss_vel.is_valid                                                   = true;
    ins_update(f, &m);
}

/* Autonomous re-acquisition after a health-check shutdown (REQ-NAV-042): by
   default (opt.auto_reacquire_disable left false) + auto_init, a
   covariance/state corruption that trips ins_check_health re-arms the
   filter into collecting and it re-bootstraps on the next coherent
   IMU+fix window, instead of staying dead
   (scenario_health_check_deinit tests the auto_init-off/no-recovery
   case). Same white-box poke as that scenario: corrupt an internal, then
   a same-timestamp update (no predict) so the health check evaluates the
   corrupted value. */
/* @satisfies REQ-NAV-042 */
static void scenario_auto_reacquire_after_health_fail(void)
{
    printf("\n=== Scenario: auto re-acquire after health-check shutdown (REQ-NAV-042) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* (1) Recovery ON (the default): auto_init, auto_reacquire_disable left
       false. A covariance corruption trips the health check -> re-arm
       (collecting) -> re-bootstrap. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped from the first fix");

        f.d[INS_IDX_VEL] = NAN; /* corrupt a covariance factor */
        ins_measurements_t mq;
        memset(&mq, 0, sizeof(mq));
        mq.timestamp = t; /* same timestamp -> predict skipped, poke survives */
        set_imu(&mq, acc_body, gyr_body, dt);
        ins_update(&f, &mq);

        CHECK_TRUE(!f.is_initialized, "health check tripped");
        CHECK_TRUE(f.is_collecting, "re-armed into collecting, not left dead");
        CHECK_TRUE(ins_get_diag(&f)->n_health_reset == 1, "n_health_reset counted the re-arm");
        /* A corrupted state is exactly what must NOT be carried (REQ-NAV-061). */
        CHECK_TRUE(!f.bias_carry.valid, "a health re-arm arms no bias carry");

        double p_dummy[3];
        CHECK_TRUE(!ins_get_position_ecef(&f, p_dummy) && !ins_is_ready(&f),
                   "accessors refuse while re-collecting");

        /* Autonomous recovery on the next coherent IMU+fix window. */
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "re-bootstrapped autonomously (recovered)");
        CHECK_TRUE(ins_get_diag(&f)->n_health_reset == 1, "counter preserved across recovery");

        float pos_ned[3];
        CHECK_TRUE(ins_get_position_local(&f, pos_ned), "position available after recovery");
        CHECK_TRUE(fabsf(pos_ned[0]) < 5.0f && fabsf(pos_ned[1]) < 5.0f && fabsf(pos_ned[2]) < 5.0f,
                   "recovered position is sane (near the anchor)");
    }

    /* (2) Recovery OFF (opt-out): the same corruption stays dead forever. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init              = true;
        opt.auto_reacquire_disable = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped");

        f.d[INS_IDX_VEL] = NAN;
        ins_measurements_t mq;
        memset(&mq, 0, sizeof(mq));
        mq.timestamp = t;
        set_imu(&mq, acc_body, gyr_body, dt);
        ins_update(&f, &mq);
        CHECK_TRUE(!f.is_initialized && !f.is_collecting, "stays dead with the option opted out");
        CHECK_TRUE(ins_get_diag(&f)->n_health_reset == 0, "no re-arm counted");

        /* A following coherent IMU+fix window must NOT revive it. */
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(!f.is_initialized, "never recovers on its own with the option opted out");
    }

    /* (3) Requires auto_init: default recovery is on, but auto_init off
       (manual init) must stay dead. A re-armed manual init would re-apply
       stale prescribed values, so this case deliberately keeps the caller
       in control. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = false;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "manual init started");

        f.d[INS_IDX_VEL] = NAN;
        ins_measurements_t mq;
        memset(&mq, 0, sizeof(mq));
        mq.timestamp = t;
        set_imu(&mq, acc_body, gyr_body, dt);
        ins_update(&f, &mq);
        CHECK_TRUE(!f.is_initialized && !f.is_collecting,
                   "auto_init off -> stays dead even with recovery on by default");
        CHECK_TRUE(ins_get_diag(&f)->n_health_reset == 0, "no re-arm without auto_init");
    }
}

/* A forced time-jump reset (REQ-NAV-016, dt_ms > max_prediction_time_sec*1000,
   allow_unlimited_deadreckoning off) is a second "is_initialized = false"
   mid-run shutdown, distinct from the health-check one but sharing the same
   auto_reacquire_disable + auto_init re-arm, on by default (REQ-NAV-042).
   Without it, is_collecting is never set back to true either, so
   ins_update's top-level "not initialized" branch has nothing to do on
   every following epoch and the filter is stuck forever, unlike the
   (already covered) health-check case. */
/* @satisfies REQ-NAV-016 REQ-NAV-042 */
static void scenario_time_jump_reset_reacquires(void)
{
    printf("\n=== Scenario: forced time-jump reset re-acquires by default "
           "(REQ-NAV-016, REQ-NAV-042) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* (1) Recovery ON (the default): the reset re-arms into collecting and
       re-bootstraps autonomously, exactly like a health-check shutdown
       would. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init                     = true;
        opt.allow_unlimited_deadreckoning = false; /* forward jumps must reset */
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped from the first fix");

        ins_measurements_t m_jump;
        memset(&m_jump, 0, sizeof(m_jump));
        m_jump.timestamp = t + 2000000; /* +2 s, max_prediction_time_sec = 0.5 s */
        set_imu(&m_jump, acc_body, gyr_body, dt);
        ins_update(&f, &m_jump);
        t = m_jump.timestamp;

        CHECK_TRUE(!f.is_initialized, "time-jump reset tripped");
        CHECK_TRUE(f.is_collecting, "re-armed into collecting, not left dead");
        CHECK_TRUE(ins_get_diag(&f)->n_time_jump_reset == 1, "n_time_jump_reset counted the reset");
        CHECK_TRUE(ins_get_diag(&f)->n_health_reset == 0,
                   "n_health_reset stays untouched by a time-jump reset");

        /* Autonomous recovery on the next coherent IMU+fix window. */
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "re-bootstrapped autonomously (recovered)");
    }

    /* (2) Recovery OFF (opt-out): stays dead forever. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init                     = true;
        opt.auto_reacquire_disable        = true;
        opt.allow_unlimited_deadreckoning = false;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped");

        ins_measurements_t m_jump;
        memset(&m_jump, 0, sizeof(m_jump));
        m_jump.timestamp = t + 2000000;
        set_imu(&m_jump, acc_body, gyr_body, dt);
        ins_update(&f, &m_jump);
        t = m_jump.timestamp;
        CHECK_TRUE(!f.is_initialized && !f.is_collecting, "stays dead with the option opted out");

        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(!f.is_initialized, "never recovers on its own with the option opted out");
    }
}

/* A restarted timestamp source (the sensor board reboots, its counter
   starts over) puts every following epoch far in the past. Dropping them
   one by one never ends: the time-jump reference stays in the abandoned
   timebase, so the filter would be fed valid data forever without moving
   (REQ-NAV-070). */
static void scenario_time_source_restart_recovers(void)
{
    printf("\n=== Scenario: restarted time source recovers (REQ-NAV-070) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* One stale epoch, fed at a timestamp far enough in the past to be
       dropped rather than merely skipped. */
    const ins_time_us_t stale_back = 2000000000; /* 2000 s */

    /* (1) Reordering bursts below the threshold must never accumulate into
       a reset, however many of them arrive. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped from the first fix");

        int burst;
        for (burst = 0; burst < 3; ++burst)
        {
            int k;
            for (k = 0; k < INS_TIME_RESTART_EPOCHS - 1; ++k)
            {
                ins_measurements_t m;
                memset(&m, 0, sizeof(m));
                m.timestamp = t - stale_back + (ins_time_us_t)k * 10000;
                set_imu(&m, acc_body, gyr_body, dt);
                ins_update(&f, &m);
            }
            /* One good epoch between the bursts breaks the run. */
            t += us_from_sec(dt);
            ins_measurements_t m_ok;
            memset(&m_ok, 0, sizeof(m_ok));
            m_ok.timestamp = t;
            set_imu(&m_ok, acc_body, gyr_body, dt);
            ins_update(&f, &m_ok);
        }

        CHECK_TRUE(f.is_initialized, "reordering bursts leave the filter running");
        CHECK_TRUE(ins_get_diag(&f)->n_time_restart_reset == 0,
                   "n_time_restart_reset stays clear for bursts below the threshold");
        CHECK_TRUE(ins_get_diag(&f)->n_time_dropped ==
                       (uint32_t)(3 * (INS_TIME_RESTART_EPOCHS - 1)),
                   "every stale epoch was still counted as dropped");
    }

    /* (2) A sustained backwards discontinuity resets and re-acquires in the
       new timebase. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init = true;
        /* Unlimited dead reckoning must NOT suppress this reset the way it
           suppresses the forward-jump one: the old timebase never returns. */
        opt.allow_unlimited_deadreckoning = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped");

        float pos_frozen[3];
        ins_get_position_local(&f, pos_frozen);

        /* The source restarts: same rate, new timebase, all in the past. */
        ins_time_us_t t2 = t - stale_back;
        int           k;
        bool          reset_seen  = false;
        int           epochs_used = 0;
        for (k = 0; k < INS_TIME_RESTART_EPOCHS; ++k)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t2;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
            if (!f.is_initialized && !reset_seen)
            {
                reset_seen  = true;
                epochs_used = k + 1;
            }
        }

        CHECK_TRUE(reset_seen, "sustained backwards discontinuity forced a reset");
        CHECK_TRUE(epochs_used == INS_TIME_RESTART_EPOCHS,
                   "reset fired exactly at INS_TIME_RESTART_EPOCHS, not earlier");
        CHECK_TRUE(f.is_collecting, "re-armed into collecting, not left dead");
        CHECK_TRUE(ins_get_diag(&f)->n_time_restart_reset == 1,
                   "n_time_restart_reset counted the reset");
        CHECK_TRUE(ins_get_diag(&f)->n_time_jump_reset == 0,
                   "the forward-jump counter stays untouched");

        /* Autonomous recovery on the next coherent window in the new
           timebase -- the whole point: without the reset the filter would
           still be dropping these epochs. */
        feed_autoinit_bootstrap(&f, init.x_ecef, &t2, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "re-bootstrapped in the new timebase");
        CHECK_TRUE(ins_is_ready(&f), "delivers a solution again");
    }

    /* (3) Opted out of re-acquisition: the reset still happens (the filter
       must not keep integrating an abandoned timebase), but recovery is the
       caller's business. */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_init              = true;
        opt.auto_reacquire_disable = true;
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        ins_time_us_t t = 0;
        feed_autoinit_bootstrap(&f, init.x_ecef, &t, acc_body, gyr_body, dt);
        CHECK_TRUE(f.is_initialized, "bootstrapped");

        ins_time_us_t t2 = t - stale_back;
        int           k;
        for (k = 0; k < INS_TIME_RESTART_EPOCHS; ++k)
        {
            t2 += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t2;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        CHECK_TRUE(!f.is_initialized && !f.is_collecting,
                   "reset happens but stays dead with the option opted out");
    }
}

/* ins_fuse must reject an invalid measurement-noise model (non-positive
   variance, or a correlated covariance that is not positive definite)
   without corrupting U,d: it counts n_fuse_fail and discards the update. */
static void scenario_fuse_rejects_bad_noise(void)
{
    printf("\n=== Scenario: fusion rejects invalid noise models ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* (a) ZUPT with a zero measurement stddev -> R diagonal is 0. Forced
       AFTER ins_init() (which now resolves a 0 zero_vel_stddev_mps to a
       beginner-friendly default, REQ-NAV-049): this specifically exercises
       ins_fuse()'s own defensive R <= 0 rejection as a second line of
       defense, not just "does ins_init prevent it". */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        f.init.zero_vel_stddev_mps = 0.0f; /* invalid: yields a zero variance */
        ins_time_us_t t            = 0;
        int           i;
        for (i = 0; i < 40; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            m.zero_velocity_update = true;
            ins_update(&f, &m);
        }
        CHECK_TRUE(ins_get_diag(&f)->n_fuse_fail >= 1, "zero-variance ZUPT counted as fuse fail");
        CHECK_TRUE(f.is_initialized, "filter survives the bad ZUPT noise model");
    }

    /* (b) GNSS position with an indefinite correlated covariance
       (diag 1, cov(N,E) = 5 -> eigenvalues 6 and -4). Passes the stddev
       gate, then fails the decorrelation (Cholesky). */
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        if (ins_init(&f, &init, &opt) != 0)
        {
            printf("init failed\n");
            fails++;
            return;
        }
        ins_time_us_t t = 0;
        int           i;
        for (i = 0; i < 40; ++i)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f, &m);
        }
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.is_valid = true;
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
        m.gnss_pos.Qll_ned[1] = m.gnss_pos.Qll_ned[3] = 5.0f; /* indefinite */
        ins_update(&f, &m);
        CHECK_TRUE(ins_get_diag(&f)->n_fuse_fail >= 1,
                   "indefinite GNSS covariance counted as fuse fail");
        CHECK_TRUE(f.is_initialized, "filter survives the indefinite covariance");
    }
}

/* --------------------------------------------------------------------------
 * Vertical origin relocation: a pure datum shift. The absolute solution
 * (ECEF, lat/lon/height) must be invariant, all local heights shift by
 * the given amount, and the filter keeps running normally afterwards.
 * -------------------------------------------------------------------------- */

static void scenario_origin_shift(void)
{
    printf("\n=== Scenario: vertical origin shift (datum relocation) ===\n");

    ins_time_us_t t = 1000000;
    ins_init_t    init;
    ins_options_t opt;
    fill_default_init(&init, t);
    fill_default_opt(&opt);

    static ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

    /* Run a static second so the history buffer has entries. */
    const float        acc[3] = {0.0f, 0.0f, -9.80665f};
    const float        gyr[3] = {0.0f, 0.0f, 0.0f};
    ins_measurements_t m;
    int                i;
    for (i = 0; i < 100; ++i)
    {
        t += 10000;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc, gyr, 0.01f);
        ins_update(&f, &m);
    }

    double ecef_before[3], llh_before = f.latlonh[2];
    float  pos_before[3];
    CHECK_TRUE(ins_get_position_ecef(&f, ecef_before), "ecef before");
    CHECK_TRUE(ins_get_position_local(&f, pos_before), "pos before");

    const float dz = 12.5f;
    ins_shift_origin_down(&f, dz);

    /* Local height above the origin grows by dz... */
    float pos_after[3];
    CHECK_TRUE(ins_get_position_local(&f, pos_after), "pos after");
    CHECK_NEAR(pos_after[2], pos_before[2] - dz, 1e-4, "pos_local down shifted by -dz");
    CHECK_NEAR(-pos_after[2] - (-pos_before[2]), dz, 1e-4, "local height grows by dz");

    /* ...while the absolute solution is untouched. */
    double ecef_after[3];
    CHECK_TRUE(ins_get_position_ecef(&f, ecef_after), "ecef after");
    CHECK_NEAR(ecef_after[0], ecef_before[0], 1e-3, "ECEF x invariant");
    CHECK_NEAR(ecef_after[1], ecef_before[1], 1e-3, "ECEF y invariant");
    CHECK_NEAR(ecef_after[2], ecef_before[2], 1e-3, "ECEF z invariant");
    CHECK_NEAR(f.latlonh[2], llh_before, 1e-3, "ellipsoid height invariant");

    /* NaN shift is a no-op; the filter keeps running normally. */
    ins_shift_origin_down(&f, nanf(""));
    CHECK_TRUE(ins_get_position_local(&f, pos_after), "pos after NaN");
    CHECK_NEAR(pos_after[2], pos_before[2] - dz, 1e-4, "NaN shift ignored");

    for (i = 0; i < 100; ++i)
    {
        t += 10000;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc, gyr, 0.01f);
        ins_update(&f, &m);
    }
    CHECK_TRUE(f.is_initialized, "filter healthy after shift");
    CHECK_TRUE(ins_get_position_local(&f, pos_after), "pos accessor ok");
    CHECK_NEAR(pos_after[2], pos_before[2] - dz, 0.05, "datum stable while running");
}

/* -------------------------------------------------------------------------- */

/* REQ-NAV-031: degenerate (near-zero) bias process noise under tight
 * aiding (cm-RTK + auto-ZUPT/ZARU on a parked vehicle) must not stop
 * the filter. Mirrors the i2Nav/KF-GINS parked-phase configuration
 * that motivated the bias-RW floor in the dataset presets. */
static void scenario_degenerate_process_noise(void)
{
    printf("\n=== Scenario: degenerate process noise stays alive ===\n");

    const double lat = 30.5 * M_PI / 180.0;
    const double lon = 114.3 * M_PI / 180.0;
    const double hgt = 40.0;

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.time = 0;
    ins_latlonh_to_ecef(lat, lon, hgt, init.x_ecef);
    init.pos_init_stddev_m         = 0.1f;
    init.vel_init_stddev_mps       = 0.1f;
    init.rpy_init_stddev_rad[0]    = (float)(1.0 * M_PI / 180.0);
    init.rpy_init_stddev_rad[1]    = (float)(1.0 * M_PI / 180.0);
    init.acc_bias_init_stddev_mps2 = 0.05f;
    init.gyr_bias_init_stddev_rps  = (float)(0.01 * M_PI / 180.0);
    init.pos_pred_stddev_m_sqrts   = 0.01f;
    init.vel_pred_stddev_mps_sqrts = 0.05f;
    init.rpy_pred_stddev_rad_sqrts = (float)(0.01 * M_PI / 180.0);
    /* GM-derived densities of the KF-GINS preset before the floor:
       Q ~ 1e-20 per step in single precision for the gyro bias. */
    init.acc_bias_pred_stddev_mps2_sqrts = 1.8e-6f;
    init.gyr_bias_pred_stddev_rps_sqrts  = 1.5e-9f;
    init.zero_vel_stddev_mps             = 0.05f;
    init.zero_rot_stddev_rps             = (float)(0.1 * M_PI / 180.0);
    init.magnetic_n[0]                   = 20.0f;
    init.magnetic_n[2]                   = 45.0f;

    ins_options_t opt;
    fill_default_opt(&opt);
    opt.allow_unlimited_deadreckoning = false; /* aided throughout */

    static ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

    float g_n[3];
    ins_gravity_ned((float)lat, (float)hgt, g_n);
    const float acc[3] = {0.0f, 0.0f, -g_n[2]};
    const float gyr[3] = {0.0f, 0.0f, 0.0f};

    const int     rate = 200;
    const int     secs = 120;
    ins_time_us_t t    = 0;
    int           i;
    for (i = 1; i <= secs * rate; ++i)
    {
        t += 1000000LL / rate;
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = 1.0f / (float)rate;
        vec3_assign(m.acc.data, acc);
        m.acc.Qll_diag[0] = m.acc.Qll_diag[1] = m.acc.Qll_diag[2] = 6.9e-7f;
        m.acc.is_valid                                            = true;
        vec3_assign(m.gyr.data, gyr);
        m.gyr.Qll_diag[0] = m.gyr.Qll_diag[1] = m.gyr.Qll_diag[2] = 2.1e-10f;
        m.gyr.is_valid                                            = true;
        if (i % rate == 0 || i == 1) /* 1 Hz cm-level RTK; i==1 crosses the
                                        startup gate (REQ-NAV-033) at t~0 */
        {
            m.gnss_pos.is_valid = true;
            ins_latlonh_to_ecef(lat, lon, hgt, m.gnss_pos.xyz_ecef);
            m.gnss_pos.Qll_ned[0] = 1e-4f;
            m.gnss_pos.Qll_ned[4] = 1e-4f;
            m.gnss_pos.Qll_ned[8] = 9e-4f;
        }
        ins_update(&f, &m);
        if (!f.is_initialized) { break; /* the CHECKs below report the failure */ }
    }

    const ins_diag_t* diag = ins_get_diag(&f);
    CHECK_TRUE(f.is_initialized, "filter alive after 120 s");
    CHECK_TRUE(ins_is_ready(&f), "filter ready");
    CHECK_TRUE(diag->n_predict >= (uint32_t)(secs * 100 - 100), "prediction kept running");
    CHECK_TRUE(diag->n_fuse_fail == 0, "no fusion failures");
    CHECK_TRUE(diag->n_auto_zupt > 0, "auto-ZUPT active (tight aiding)");
    bool cov_ok = true;
    for (i = 0; i < f.n; ++i)
    {
        if (!isfinite(f.d[i]) || f.d[i] <= 0.0f) cov_ok = false;
    }
    CHECK_TRUE(cov_ok, "covariance factors finite and positive");
    float pos[3];
    CHECK_TRUE(ins_get_position_local(&f, pos), "position accessor");
    CHECK_TRUE(test_vec3_norm(pos) < 0.1f, "position pinned to the fix");
}

/* -------------------------------------------------------------------------- */

/* REQ-NAV-032: ins_set_world_model swaps the gravity/magnetic
 * reference vectors at runtime; NULL keeps the old value; the mag
 * fusion follows the new reference (yaw converges to the rotated
 * field). Also covers the non-finite guard of
 * ins_set_magnetic_model_from_position. */
static void scenario_set_world_model(void)
{
    printf("\n=== Scenario: runtime world model update ===\n");

    ins_time_us_t t = 1000000;
    ins_init_t    init;
    ins_options_t opt;
    fill_default_init(&init, t);
    fill_default_opt(&opt);
    opt.magnetometer_min_delay_ms = 100;

    static ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

    double lat, lon, hgt;
    ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &hgt);
    float g_n[3];
    ins_gravity_ned((float)lat, (float)hgt, g_n);
    const float acc[3] = {0.0f, 0.0f, -g_n[2]};
    const float gyr[3] = {0.0f, 0.0f, 0.0f};
    /* body at yaw 0: mag_b == magnetic_n */
    const float mag_b[3] = {20.0f, 0.0f, 45.0f};

    ins_measurements_t m;
    int                i;
#define WM_EPOCH()                                                        \
    do {                                                                  \
        t += 10000;                                                       \
        memset(&m, 0, sizeof(m));                                         \
        m.timestamp = t;                                                  \
        set_imu(&m, acc, gyr, 0.01f);                                     \
        m.mag.is_valid = true;                                            \
        vec3_assign(m.mag.data, mag_b);                                   \
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f; \
        ins_update(&f, &m);                                               \
    } while (0)

    for (i = 0; i < 300; ++i) WM_EPOCH();
    float roll, pitch, yaw;
    CHECK_TRUE(ins_get_rpy(&f, &roll, &pitch, &yaw), "rpy valid");
    CHECK_NEAR(RAD2DEG(yaw), 0.0, 1.0, "yaw settled on old reference");

    /* NULL arguments keep the current model. */
    const float mag_before[3] = {f.magnetic_n[0], f.magnetic_n[1], f.magnetic_n[2]};
    ins_set_world_model(&f, (const float*)0, (const float*)0);
    ins_set_world_model((ins_t*)0, (const float*)0, (const float*)0);
    CHECK_NEAR(f.magnetic_n[0], mag_before[0], 1e-9, "NULL keeps mag ref");

    /* Gravity replacement round-trip. */
    const float g_custom[3] = {0.0f, 0.0f, 9.7f};
    ins_set_world_model(&f, g_custom, (const float*)0);
    CHECK_NEAR(f.gravity_n[2], 9.7, 1e-6, "gravity replaced");
    ins_set_world_model(&f, g_n, (const float*)0);

    /* Rotate the magnetic reference by +30 deg about down: with the
       body-frame measurement unchanged, the estimated yaw must follow
       the new reference to +30 deg. */
    const float a30        = (float)(30.0 * M_PI / 180.0);
    const float mag_rot[3] = {cosf(a30) * 20.0f, sinf(a30) * 20.0f, 45.0f};
    ins_set_world_model(&f, (const float*)0, mag_rot);

    for (i = 0; i < 9000; ++i) WM_EPOCH();
    CHECK_TRUE(ins_get_rpy(&f, &roll, &pitch, &yaw), "rpy valid");
    CHECK_NEAR(RAD2DEG(yaw), 30.0, 3.0, "yaw follows the new reference");
    CHECK_TRUE(f.is_initialized, "filter healthy");

    /* Non-finite position must not disturb the reference (WMM path). */
    const float mag_now = f.magnetic_n[0];
    ins_set_magnetic_model_from_position(&f, (double)NAN, lon, 2026.5f);
    CHECK_NEAR(f.magnetic_n[0], mag_now, 1e-9, "NaN position ignored");
#undef WM_EPOCH
}

/* -------------------------------------------------------------------------- */

/* Fail-safe check: whatever the internal state, the filter must either
 * refuse to publish (accessor false) or publish finite values. */
static void check_failsafe_outputs(const ins_t* f)
{
    double pe[3];
    float  v[3], q[4];
    if (ins_get_position_ecef(f, pe))
    {
        CHECK_TRUE(__builtin_isfinite(pe[0]) && __builtin_isfinite(pe[1]) &&
                       __builtin_isfinite(pe[2]),
                   "published ECEF position finite");
    }
    if (ins_get_velocity_ned(f, v))
    {
        CHECK_TRUE(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]), "published velocity finite");
    }
    if (ins_get_quaternion(f, q))
    {
        CHECK_TRUE(isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3]),
                   "published quaternion finite");
    }
}

/* REQ-SYS-005: corrupted UDU covariance factors (memory fault,
 * numerical breakdown) must not crash the per-epoch routines. One
 * epoch of prediction + full fusion math runs on the broken factors.
 * The filter then either flags itself unready (health check) or keeps
 * publishing finite values, and a re-init of the same instance
 * restores normal operation. */
static void scenario_corrupted_covariance(void)
{
    printf("\n=== Scenario: corrupted UDU factors fail safe ===\n");

    const char*         names[4] = {"d negative", "d NaN", "d Inf", "U NaN"};
    const ins_time_us_t t0       = 1000000;
    static ins_t        f; /* keep the large struct off the stack */
    int                 kind, i;

    for (kind = 0; kind < 4; ++kind)
    {
        printf("      -- corruption: %s\n", names[kind]);
        ins_init_t    init;
        ins_options_t opt;
        fill_default_init(&init, t0);
        fill_default_opt(&opt);
        opt.magnetometer_min_delay_ms = -1; /* no rate limit: fuse the mag every epoch */

        memset(&f, 0, sizeof(f));
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

        double lat, lon, hgt;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &hgt);
        float g_n[3];
        ins_gravity_ned((float)lat, (float)hgt, g_n);
        const float acc[3]   = {0.0f, 0.0f, -g_n[2]};
        const float gyr[3]   = {0.0f, 0.0f, 0.0f};
        const float mag_b[3] = {20.0f, 0.0f, 45.0f};

        ins_time_us_t      t = t0;
        ins_measurements_t m;

        /* One epoch exercising every fusion path: GNSS pos+vel (6-dim),
           magnetometer (3-dim), ZUPT (3-dim), ZARU (3-dim), plus the
           Thornton predict. */
#define CC_EPOCH()                                                                     \
    do {                                                                               \
        t += 10000;                                                                    \
        memset(&m, 0, sizeof(m));                                                      \
        m.timestamp = t;                                                               \
        set_imu(&m, acc, gyr, 0.01f);                                                  \
        m.mag.is_valid = true;                                                         \
        vec3_assign(m.mag.data, mag_b);                                                \
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 1.0f;              \
        m.gnss_pos.is_valid                                       = true;              \
        memcpy(m.gnss_pos.xyz_ecef, init.x_ecef, sizeof(init.x_ecef));                 \
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = 0.25f;                         \
        m.gnss_pos.Qll_ned[8]                         = 1.0f;                          \
        m.gnss_vel.is_valid                           = true;                          \
        m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f; \
        m.zero_velocity_update                                                = true;  \
        m.zero_rotation_update                                                = true;  \
        ins_update(&f, &m);                                                            \
    } while (0)

        for (i = 0; i < 50; ++i) CC_EPOCH();
        CHECK_TRUE(f.is_initialized, "healthy before corruption");

        switch (kind)
        {
        case 0: /* indefinite but finite covariance */
            f.d[INS_IDX_VEL + 0] = -1e-3f;
            f.d[INS_IDX_RPY + 1] = -1e-6f;
            break;
        case 1: f.d[0] = nanf(""); break;
        case 2: f.d[INS_IDX_ACC + 1] = INFINITY; break;
        case 3: /* U is column-major n x n: (row 0, col GYR) */
            f.U[INS_IDX_GYR * f.n + 0] = nanf("");
            break;
        }

        /* Full prediction + fusion math on the broken factors: must
           not crash (the point of this scenario). */
        for (i = 0; i < 10; ++i) CC_EPOCH();

        if (kind == 2)
        {
            /* Inf propagates into d or the state within the epoch ->
               health check pulls the plug. */
            CHECK_TRUE(!f.is_initialized, "health check tripped");
        }
        else if (kind == 1 || kind == 3)
        {
            /* NaN sigma in kalman_udu_predict() is caught by its own
               singularity guard (sigma > KALMAN_UDU_EPS) before it can
               reach d/U or ins_check_health(): the guard cannot tell a
               NaN apart from a legitimately singular sigma, so it
               resets the affected d[i]/U column to 0 and the corruption
               is healed rather than detected. */
            CHECK_TRUE(f.is_initialized, "self-healed by predict singularity guard");
        }
        check_failsafe_outputs(&f);

        /* Re-init of the very same instance restores normal operation. */
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "re-init");
        for (i = 0; i < 50; ++i) CC_EPOCH();
        CHECK_TRUE(f.is_initialized, "healthy after re-init");
        float pos[3];
        CHECK_TRUE(ins_get_position_local(&f, pos), "accessor ok");
        CHECK_TRUE(test_vec3_norm(pos) < 1.0f, "position back on the fix");
#undef CC_EPOCH
    }
}

/* ---------------------------------------------------------------------------
 * Scenario: global chi2-disable override (REQ-NAV-035). With the flag
 * set, a gross GNSS position outlier is fused at face value instead of
 * being chi2-downweighted, so the filter jumps onto it fully.
 * ---------------------------------------------------------------------------
 */

static float run_gnss_outlier(bool chi2_disable)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.chi2_disable = chi2_disable;

    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init");

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 200; ++step) /* 2 s to settle */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }

    /* One grossly implausible GNSS fix: 500 m north, reported as if
       accurate to 1 m. Downweighted (default) it should barely move the
       filter; at face value (chi2_disable) it should jump close to it. */
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    t += us_from_sec(dt);
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    float R_n_to_e[9];
    ins_rotmat_n_to_e(f.latlonh[0], f.latlonh[1], R_n_to_e);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0] + R_n_to_e[0] * 500.0f;
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1] + R_n_to_e[1] * 500.0f;
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2] + R_n_to_e[2] * 500.0f;
    m.gnss_pos.Qll_ned[0]  = 1.0f;
    m.gnss_pos.Qll_ned[4]  = 1.0f;
    m.gnss_pos.Qll_ned[8]  = 1.0f;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    /* REQ-NAV-036: the downweight counter must fire exactly on the
       outlier fusion when chi2 is active, and never when disabled. */
    if (chi2_disable)
    {
        CHECK_TRUE(f.diag.n_downweighted == 0, "chi2_disable: n_downweighted stays 0");
    }
    else { CHECK_TRUE(f.diag.n_downweighted == 1, "default: n_downweighted counts the outlier"); }

    float pl[3];
    ins_get_position_local(&f, pl);
    return pl[0]; /* north component: 0 before the fix, ~500 if fully accepted */
}

/* Settle a GNSS-tightened filter, then inject one MODERATE north outlier and
   report how many downweights it triggered (0 or 1). The position covariance
   is first collapsed with repeated good fixes so the outlier's Mahalanobis
   distance is predictable; chi2_reject_alpha selects the gate (0 -> default).
   Used by scenario_chi2_reject_alpha (REQ-NAV-046). */
static uint32_t run_gnss_moderate_outlier(float chi2_reject_alpha, float shift_m,
                                          float rep_stddev_m)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.chi2_reject_alpha = chi2_reject_alpha;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "moderate-outlier init");

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float rep_var     = rep_stddev_m * rep_stddev_m;

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    float         R_n_to_e[9];
    for (step = 1; step <= 300; ++step) /* 3 s: settle + 5 Hz good fixes */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % 20 == 0) /* good fix at the true origin -> tightens P */
        {
            ins_rotmat_n_to_e(f.latlonh[0], f.latlonh[1], R_n_to_e);
            m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
            m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
            m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
            m.gnss_pos.Qll_ned[0]  = rep_var;
            m.gnss_pos.Qll_ned[4]  = rep_var;
            m.gnss_pos.Qll_ned[8]  = rep_var;
            m.gnss_pos.is_valid    = true;
        }
        ins_update(&f, &m);
    }

    const uint32_t before = f.diag.n_downweighted;
    /* Let the outlier land in the next fix slot rather than 10 ms behind the
       last good one, so the fusion rate limit (REQ-NAV-074) does not skip
       the very sample this scenario is about. */
    int filler;
    for (filler = 0; filler < 19; ++filler)
    {
        t += us_from_sec(dt);
        ins_measurements_t mi;
        memset(&mi, 0, sizeof(mi));
        mi.timestamp = t;
        set_imu(&mi, acc_body, gyr_body, dt);
        ins_update(&f, &mi);
    }
    t += us_from_sec(dt);
    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t;
    set_imu(&m, acc_body, gyr_body, dt);
    ins_rotmat_n_to_e(f.latlonh[0], f.latlonh[1], R_n_to_e);
    m.gnss_pos.xyz_ecef[0] = init.x_ecef[0] + R_n_to_e[0] * shift_m;
    m.gnss_pos.xyz_ecef[1] = init.x_ecef[1] + R_n_to_e[1] * shift_m;
    m.gnss_pos.xyz_ecef[2] = init.x_ecef[2] + R_n_to_e[2] * shift_m;
    m.gnss_pos.Qll_ned[0]  = rep_var;
    m.gnss_pos.Qll_ned[4]  = rep_var;
    m.gnss_pos.Qll_ned[8]  = rep_var;
    m.gnss_pos.is_valid    = true;
    ins_update(&f, &m);

    return f.diag.n_downweighted - before;
}

/* Drive a freshly-initialised, IMU-only filter for N steps with a constant
   acc/gyr sample, returning the final rpy and NED velocity. auto_zupt is
   disabled by the caller so the result is a pure strapdown of the
   (boundary-calibrated) IMU signal. */
static void run_imu_only(const ins_options_t* opt, const float acc[3], const float gyr[3], int N,
                         float dt, float rpy_out[3], float vel_out[3])
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o = *opt;
    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return;
    }
    ins_time_us_t t = 0;
    int           step;
    for (step = 1; step <= N; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc, gyr, dt);
        ins_update(&f, &m);
    }
    ins_get_rpy(&f, &rpy_out[0], &rpy_out[1], &rpy_out[2]);
    ins_get_velocity_ned(&f, vel_out);
}

/* REQ-NAV-037: IMU misalignment matrix + fixed calibration bias, applied at
   the measurement boundary before any downstream use. */
static void scenario_imu_calibration(void)
{
    printf("\n=== Scenario: IMU calibration (misalignment + fixed bias) ===\n");

    const float dt      = 0.005f;
    const int   N       = 200;      /* 1 s */
    const float g       = 9.80665f; /* nominal; only the horizontal channels matter below */
    const float wz      = 10.0f * (float)M_PI / 180.0f;
    const float lvl3[3] = {0.0f, 0.0f, -g};

    ins_options_t base;
    fill_default_opt(&base);
    base.auto_zupt_disable = true; /* pure strapdown, no ZUPT/ZARU clamping */

    float rpy[3], vel[3];

    /* 1) Gyro fixed bias cancels a constant yaw rate. Without calibration the
          filter integrates ~10 deg of yaw over 1 s; with the fixed bias
          removing it, yaw stays ~0. */
    const float gyr_z[3] = {0.0f, 0.0f, wz};
    run_imu_only(&base, lvl3, gyr_z, N, dt, rpy, vel);
    CHECK_TRUE(rpy[2] > 0.15f, "gyro no-cal: yaw drifts (~10 deg)");

    ins_options_t o         = base;
    o.imu_gyr_fixed_bias[2] = wz;
    run_imu_only(&o, lvl3, gyr_z, N, dt, rpy, vel);
    CHECK_NEAR(rpy[2], 0.0, 0.01, "gyro fixed bias: yaw stays ~0");

    /* 2) Gyro misalignment permutes body-x rate into body-z. A pure body-x
          rate normally integrates to roll; the permutation M (out_z = in_x)
          turns it into a yaw rotation instead. */
    const float gyr_x[3] = {wz, 0.0f, 0.0f};
    run_imu_only(&base, lvl3, gyr_x, N, dt, rpy, vel);
    CHECK_TRUE(fabsf(rpy[0]) > 0.15f, "gyro no-cal: body-x rate -> roll");
    CHECK_TRUE(fabsf(rpy[2]) < 0.02f, "gyro no-cal: no yaw from body-x rate");

    ins_options_t om = base;
    /* Column-major 3x3, element (row=2,col=0)=1 -> out[2]=in[0], rest 0. */
    om.imu_gyr_misalignment[2] = 1.0f;
    run_imu_only(&om, lvl3, gyr_x, N, dt, rpy, vel);
    CHECK_TRUE(rpy[2] > 0.15f, "gyro misalignment: body-x rate -> yaw");
    CHECK_TRUE(fabsf(rpy[0]) < 0.02f, "gyro misalignment: no roll");

    /* 3) Accel fixed bias cancels a horizontal specific-force bias. With
          identity attitude, a body-x bias maps to a north acceleration and
          the north velocity drifts; removing it keeps velocity ~0. */
    const float bx           = 0.5f;
    const float acc_bias3[3] = {bx, 0.0f, -g};
    const float gyr_zero[3]  = {0.0f, 0.0f, 0.0f};
    run_imu_only(&base, acc_bias3, gyr_zero, N, dt, rpy, vel);
    CHECK_TRUE(vel[0] > 0.3f, "accel no-cal: north velocity drifts");

    ins_options_t oa         = base;
    oa.imu_acc_fixed_bias[0] = bx;
    run_imu_only(&oa, acc_bias3, gyr_zero, N, dt, rpy, vel);
    CHECK_NEAR(vel[0], 0.0, 0.05, "accel fixed bias: north velocity stays ~0");
}

/* Compute P(0,0) (north position variance) from the UDU factors. */
static float pos_var_ii(const ins_t* f, int idx)
{
    float p = 0.0f;
    int   k;
    for (k = 0; k < INS_UNKNOWNS; ++k)
    {
        const float u = f->U[idx + k * INS_UNKNOWNS];
        p += u * u * f->d[k];
    }
    return p;
}

static float pos_var_00(const ins_t* f) { return pos_var_ii(f, INS_IDX_POS + 0); }

/* Run the standard "filter starts 5 m north of truth, GNSS at truth" setup
   under the given options and return the post-run north position variance
   and whether GNSS was ever fused. */
static float run_gnss_cov(const ins_options_t* opt, uint32_t* used_out, uint32_t* rejected_out)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o = *opt;

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float R_n_to_e[9];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(truth_ecef, &lat, &lon, &h);
        ins_rotmat_n_to_e(lat, lon, R_n_to_e);
    }
    init.x_ecef[0] += R_n_to_e[0] * 5.0f;
    init.x_ecef[1] += R_n_to_e[1] * 5.0f;
    init.x_ecef[2] += R_n_to_e[2] * 5.0f;

    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return 0.0f;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % 20 == 0)
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0]  = 1.0f; /* reported stddev 1 m per axis */
            m.gnss_pos.Qll_ned[4]  = 1.0f;
            m.gnss_pos.Qll_ned[8]  = 1.0f;
            m.gnss_pos.is_valid    = true;
        }
        ins_update(&f, &m);
    }
    const ins_diag_t* d = ins_get_diag(&f);
    if (used_out) { *used_out = d->n_gnss_used; }
    if (rejected_out) { *rejected_out = d->n_gnss_rejected_noise; }
    return pos_var_00(&f);
}

/* Run `epochs` IMU-only updates spaced `step_us` apart under the given
   covariance-prediction period, and report how often the prediction fired. */
static uint32_t count_kalman_predicts(float kalman_dt_sec, ins_time_us_t step_us, int epochs)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.kalman_update_dt_sec = kalman_dt_sec;

    if (ins_init(&f, &init, &opt) != 0)
    {
        CHECK_TRUE(0, "ins_init succeeds");
        return 0;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt_sec      = (float)((double)step_us * 1e-6);

    const uint32_t start = ins_get_diag(&f)->n_predict;
    ins_time_us_t  t     = 0;
    int            i;
    for (i = 0; i < epochs; ++i)
    {
        t += step_us;
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt_sec);
        ins_update(&f, &m);
    }
    return ins_get_diag(&f)->n_predict - start;
}

/* REQ-NAV-004: the due test carries a relative tolerance, so an epoch stream
   landing marginally below the configured period still propagates every
   epoch instead of silently dropping to half the requested rate. */
static void scenario_kalman_cadence_tolerance(void)
{
    printf("\n=== Scenario: Kalman prediction cadence tolerance (REQ-NAV-004) ===\n");

    /* Exactly on the period: the reference case, one prediction per epoch. */
    const uint32_t n_exact = count_kalman_predicts(0.01f, 10000, 500);
    printf("  period 10000 us, epochs 10000 us -> %u predictions / 500 epochs\n",
           (unsigned)n_exact);
    CHECK_TRUE(n_exact >= 495u, "epochs exactly on the period: one prediction each");

    /* One microsecond short -- timestamp quantization, or a part running a
       per-mille fast. Without the tolerance this is 250, i.e. half rate. */
    const uint32_t n_short = count_kalman_predicts(0.01f, 9999, 500);
    printf("  period 10000 us, epochs  9999 us -> %u predictions / 500 epochs\n",
           (unsigned)n_short);
    CHECK_TRUE(n_short >= 495u, "epochs 1 us short: still one prediction each");

    /* 0.5% short: a nominal-100 Hz part actually running at 100.5 Hz. */
    const uint32_t n_half_pct = count_kalman_predicts(0.01f, 9950, 500);
    printf("  period 10000 us, epochs  9950 us -> %u predictions / 500 epochs\n",
           (unsigned)n_half_pct);
    CHECK_TRUE(n_half_pct >= 495u, "epochs 0.5% short: still one prediction each");

    /* The tolerance must NOT swallow a genuine throttle: at a 5:1 cadence
       ratio the next candidate epoch is 20% away, 20x the tolerance, so the
       prediction still fires on every 5th epoch and not more often. */
    const uint32_t n_throttled = count_kalman_predicts(0.05f, 9999, 500);
    printf("  period 50000 us, epochs  9999 us -> %u predictions / 500 epochs\n",
           (unsigned)n_throttled);
    CHECK_TRUE(n_throttled >= 95u && n_throttled <= 105u,
               "5:1 cadence still throttles to ~100, not 500");

    /* Well below the period stays throttled proportionally. */
    const uint32_t n_quarter = count_kalman_predicts(0.04f, 10000, 500);
    printf("  period 40000 us, epochs 10000 us -> %u predictions / 500 epochs\n",
           (unsigned)n_quarter);
    CHECK_TRUE(n_quarter >= 120u && n_quarter <= 130u, "4:1 cadence throttles to ~125");
}

/* REQ-NAV-038: GNSS covariance scale + per-axis stddev floor, applied at the
   measurement boundary before fusion. */
static void scenario_gnss_cov_conditioning(void)
{
    printf("\n=== Scenario: GNSS covariance conditioning (scale + floor) ===\n");

    /* 1) Scale inflates the fused covariance: a larger scale makes the filter
          trust GNSS less, so the posterior north variance is larger. */
    ins_options_t base;
    fill_default_opt(&base);
    base.gnss_max_horizontal_pos_stddev_m = 100.0f; /* keep the gate out of the way */
    base.gnss_max_vertical_pos_stddev_m   = 100.0f;

    uint32_t      used1 = 0, used30 = 0;
    const float   p_scale1 = run_gnss_cov(&base, &used1, (uint32_t*)0);
    ins_options_t s30      = base;
    s30.gnss_pos_cov_scale = 30.0f;
    const float p_scale30  = run_gnss_cov(&s30, &used30, (uint32_t*)0);
    printf("  P00: scale=1 -> %g, scale=30 -> %g\n", p_scale1, p_scale30);
    CHECK_TRUE(used1 > 0 && used30 > 0, "scale: GNSS fused in both runs");
    CHECK_TRUE(p_scale30 > p_scale1 * 4.0f, "scale=30 inflates posterior covariance");

    /* 2) Floor inflates the fused covariance the same way the scale does.
          Reported hor stddev 1 m, floor 3 m -> the fusion weights 3 m. */
    ins_options_t base_floor;
    fill_default_opt(&base_floor);
    base_floor.gnss_max_horizontal_pos_stddev_m = 100.0f;
    base_floor.gnss_max_vertical_pos_stddev_m   = 100.0f;

    uint32_t    used_nofloor = 0;
    const float p_nofloor    = run_gnss_cov(&base_floor, &used_nofloor, (uint32_t*)0);

    ins_options_t fl               = base_floor;
    fl.gnss_pos_stddev_floor_hor_m = 3.0f;
    uint32_t    used_floor         = 0;
    const float p_floor            = run_gnss_cov(&fl, &used_floor, (uint32_t*)0);
    printf("  P00: floor off -> %g, floor 3 m -> %g\n", p_nofloor, p_floor);
    CHECK_TRUE(used_nofloor > 0 && used_floor > 0, "floor: GNSS fused in both runs");
    CHECK_TRUE(p_floor > p_nofloor * 2.0f, "floor 3 m inflates posterior covariance");
}

/* REQ-NAV-038: the conditioning weights the FUSION only. The fix-quality
   gates keep grading what the receiver reported, so a scale or floor big
   enough to blow past every gate must still not reject a single fix -- while
   the posterior covariance does move, proving the knob reached the fusion.

   This is the test that guards the split itself: nothing in the type system
   stops a future consumer from reading the reported covariance where it
   needed the conditioned one (or the reverse), and both keep compiling and
   returning plausible numbers. Only the pair of assertions below tells the
   two apart. */
static void scenario_gnss_cov_conditioning_gates_unscaled(void)
{
    printf("\n=== Scenario: GNSS cov conditioning does not move the gates ===\n");

    /* Reported stddev 1 m per axis. Gates sit at 2 m, well above it, and
       the auto-ZUPT accuracy gate at 0.05 m/s. A scale of 30 (-> 30 m) and
       a floor of 25 m each land far outside every one of them. */
    ins_options_t gate;
    fill_default_opt(&gate);
    gate.gnss_max_horizontal_pos_stddev_m       = 2.0f;
    gate.gnss_max_vertical_pos_stddev_m         = 2.0f;
    gate.gnss_start_max_horizontal_pos_stddev_m = 2.0f;
    gate.gnss_start_max_vertical_pos_stddev_m   = 2.0f;
    gate.gnss_stop_max_horizontal_pos_stddev_m  = 2.0f;
    gate.gnss_stop_max_vertical_pos_stddev_m    = 2.0f;

    uint32_t    used_ref = 0, rej_ref = 0;
    const float p_ref = run_gnss_cov(&gate, &used_ref, &rej_ref);
    CHECK_TRUE(used_ref > 0, "reference: 1 m fix passes the 2 m gates");
    CHECK_TRUE(rej_ref == 0, "reference: nothing rejected");

    ins_options_t sc      = gate;
    sc.gnss_pos_cov_scale = 30.0f;
    uint32_t    used_sc = 0, rej_sc = 0;
    const float p_sc = run_gnss_cov(&sc, &used_sc, &rej_sc);
    printf("  scale 30: used %u (ref %u), rejected %u, P00 %g (ref %g)\n", (unsigned)used_sc,
           (unsigned)used_ref, (unsigned)rej_sc, p_sc, p_ref);
    CHECK_TRUE(used_sc == used_ref, "scale 30: every fix still fused");
    CHECK_TRUE(rej_sc == 0, "scale 30: no fix rejected on noise");
    CHECK_TRUE(p_sc > p_ref * 4.0f, "scale 30: but the fusion did see it");

    ins_options_t fl               = gate;
    fl.gnss_pos_stddev_floor_hor_m = 25.0f;
    fl.gnss_pos_stddev_floor_ver_m = 25.0f;
    uint32_t    used_fl = 0, rej_fl = 0;
    const float p_fl = run_gnss_cov(&fl, &used_fl, &rej_fl);
    printf("  floor 25 m: used %u (ref %u), rejected %u, P00 %g (ref %g)\n", (unsigned)used_fl,
           (unsigned)used_ref, (unsigned)rej_fl, p_fl, p_ref);
    CHECK_TRUE(used_fl == used_ref, "floor 25 m: every fix still fused");
    CHECK_TRUE(rej_fl == 0, "floor 25 m: no fix rejected on noise");
    CHECK_TRUE(p_fl > p_ref * 4.0f, "floor 25 m: but the fusion did see it");
}

/* Runs a stationary filter for 20 s with a 100 Hz IMU and a GNSS position
   fix every `fix_every` IMU epochs, and reports the resulting diagnostics.
   Used for the fusion rate limit (REQ-NAV-074). */
static void run_gnss_rate(int min_delay_ms, int fix_every, ins_diag_t* out)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o;
    fill_default_opt(&o);
    o.gnss_min_delay_ms = min_delay_ms;
    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return;
    }

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 2000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % fix_every == 0)
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 4.0f;
            m.gnss_pos.is_valid                                                   = true;
        }
        ins_update(&f, &m);
    }
    *out = *ins_get_diag(&f);
}

/* REQ-NAV-074: a fix stream running above the configured rate is paced down
   to it, block-for-block, without ever counting as a position-aiding gap. */
static void scenario_gnss_rate_limit(void)
{
    printf("\n=== Scenario: GNSS fusion rate limit (REQ-NAV-074) ===\n");

    /* 50 Hz fix stream over 20 s: 1000 fixes offered, ~200 spendable at the
       default 10 Hz. */
    ins_diag_t d50;
    memset(&d50, 0, sizeof(d50));
    run_gnss_rate(0, 2, &d50);
    printf("  50 Hz stream, default limit: seen %u, used %u, rate-limited %u\n",
           (unsigned)d50.n_gnss_seen, (unsigned)d50.n_gnss_used, (unsigned)d50.n_gnss_rate_limited);
    CHECK_TRUE(d50.n_gnss_seen > 900u, "50 Hz: the whole stream is seen");
    CHECK_TRUE(d50.n_gnss_used > 150u && d50.n_gnss_used < 250u, "50 Hz: fused at ~10 Hz, not 50");
    CHECK_TRUE(d50.n_gnss_rate_limited > 700u, "50 Hz: the skipped fixes are counted");
    CHECK_TRUE(d50.n_gnss_rejected_noise == 0,
               "a paced fix is not a rejected one: the noise counter stays clean");

    /* The same stream with the limit off is fused in full. */
    ins_diag_t doff;
    memset(&doff, 0, sizeof(doff));
    run_gnss_rate(-1, 2, &doff);
    printf("  50 Hz stream, limit off:     seen %u, used %u, rate-limited %u\n",
           (unsigned)doff.n_gnss_seen, (unsigned)doff.n_gnss_used,
           (unsigned)doff.n_gnss_rate_limited);
    CHECK_TRUE(doff.n_gnss_used > d50.n_gnss_used * 3u, "limit off: every fix is spent");
    CHECK_TRUE(doff.n_gnss_rate_limited == 0u, "limit off: nothing is paced");

    /* A stream already below the limit is untouched: 5 Hz passes through. */
    ins_diag_t d5;
    memset(&d5, 0, sizeof(d5));
    run_gnss_rate(0, 20, &d5);
    printf("  5 Hz stream,  default limit: seen %u, used %u, rate-limited %u\n",
           (unsigned)d5.n_gnss_seen, (unsigned)d5.n_gnss_used, (unsigned)d5.n_gnss_rate_limited);
    CHECK_TRUE(d5.n_gnss_rate_limited == 0u, "5 Hz: below the limit, nothing is paced");
    CHECK_TRUE(d5.n_gnss_used == d5.n_gnss_seen, "5 Hz: every offered fix is spent");

    /* An explicit 200 ms (5 Hz) limit paces a 10 Hz stream down by half. */
    ins_diag_t d200;
    memset(&d200, 0, sizeof(d200));
    run_gnss_rate(200, 10, &d200);
    printf("  10 Hz stream, 200 ms limit:  seen %u, used %u, rate-limited %u\n",
           (unsigned)d200.n_gnss_seen, (unsigned)d200.n_gnss_used,
           (unsigned)d200.n_gnss_rate_limited);
    CHECK_TRUE(d200.n_gnss_used > 80u && d200.n_gnss_used < 120u,
               "explicit 200 ms limit: ~5 Hz out of a 10 Hz stream");
}

/* REQ-NAV-071: the conditioning clamps the per-axis stddev from above as
   well as from below. Exercised through the public conditioning entry points
   so the exact conditioned numbers can be asserted rather than inferred from
   a posterior covariance. */
static void scenario_gnss_cov_cap(void)
{
    printf("\n=== Scenario: GNSS covariance cap (REQ-NAV-071) ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.auto_init = true;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init with the default floors and caps");

    /* A degraded fix: 500 m horizontal, 400 m vertical, 80 m/s velocity. The
       cap brings each down to what the fusion is allowed to see. */
    {
        float Qin[9], Qout[9];
        memset(Qin, 0, sizeof(Qin));
        Qin[0] = Qin[4] = 500.0f * 500.0f;
        Qin[8]          = 400.0f * 400.0f;
        ins_gnss_condition_pos_cov(&f.opt, Qin, Qout);
        printf("  pos 500/400 m -> %.1f / %.1f m\n", sqrtf(Qout[0]), sqrtf(Qout[8]));
        CHECK_NEAR(sqrtf(Qout[0]), 120.0f, 1e-3, "horiz. pos stddev capped at 120 m");
        CHECK_NEAR(sqrtf(Qout[4]), 120.0f, 1e-3, "east pos stddev capped the same way");
        CHECK_NEAR(sqrtf(Qout[8]), 120.0f, 1e-3, "vert. pos stddev capped at 120 m");
    }
    {
        float Qin[9], Qout[9];
        memset(Qin, 0, sizeof(Qin));
        Qin[0] = Qin[4] = Qin[8] = 80.0f * 80.0f;
        ins_gnss_condition_vel_cov(&f.opt, Qin, Qout);
        printf("  vel 80 m/s -> %.1f m/s\n", sqrtf(Qout[0]));
        CHECK_NEAR(sqrtf(Qout[0]), 60.0f, 1e-3, "horiz. vel stddev capped at 60 m/s");
        CHECK_NEAR(sqrtf(Qout[8]), 60.0f, 1e-3, "vert. vel stddev capped at 60 m/s");
    }

    /* An over-optimistic fix is lifted to the floor by the same call, and one
       inside the band passes through untouched. */
    {
        float Qin[9], Qout[9];
        memset(Qin, 0, sizeof(Qin));
        Qin[0] = Qin[4] = Qin[8] = 0.01f * 0.01f;
        ins_gnss_condition_pos_cov(&f.opt, Qin, Qout);
        CHECK_NEAR(sqrtf(Qout[0]), 0.5f, 1e-4, "1 cm horiz. pos lifted to the 0.5 m floor");
        CHECK_NEAR(sqrtf(Qout[8]), 1.0f, 1e-4, "1 cm vert. pos lifted to the 1.0 m floor");

        Qin[0] = Qin[4] = 4.0f * 4.0f;
        Qin[8]          = 6.0f * 6.0f;
        ins_gnss_condition_pos_cov(&f.opt, Qin, Qout);
        CHECK_NEAR(sqrtf(Qout[0]), 4.0f, 1e-4, "a value inside the band is left alone (hor)");
        CHECK_NEAR(sqrtf(Qout[8]), 6.0f, 1e-4, "a value inside the band is left alone (ver)");
    }

    /* An absent axis stays absent: a zero diagonal entry states that the fix
       says nothing about that axis, and the floor must not turn that into a
       confident metre-level statement. */
    {
        float Qin[9], Qout[9];
        memset(Qin, 0, sizeof(Qin));
        Qin[0] = Qin[4] = 2.0f * 2.0f;
        Qin[8]          = 0.0f;
        ins_gnss_condition_pos_cov(&f.opt, Qin, Qout);
        CHECK_NEAR(Qout[8], 0.0f, 1e-9, "a zero (absent) vertical variance is not floored");
    }

    /* A correlated fix with a covariance far above the cap: capping the
       diagonal must shrink the off-diagonal terms along with it (not leave
       them at their original scale), or the conditioned matrix stops being
       positive definite and the fusion has to discard it. */
    {
        float Qin[9], Qout[9];
        memset(Qin, 0, sizeof(Qin));
        const float rho = 0.6f; /* correlation coefficient shared by all three axis pairs */
        Qin[0] = Qin[4] = Qin[8] = 6000.0f * 6000.0f; /* huge, well above the cap */
        Qin[3] = Qin[1] = rho * Qin[0];               /* NE */
        Qin[6] = Qin[2] = rho * Qin[0];               /* ND */
        Qin[7] = Qin[5] = rho * Qin[0];               /* ED */
        ins_gnss_condition_pos_cov(&f.opt, Qin, Qout);
        CHECK_NEAR(sqrtf(Qout[0]), 120.0f, 1e-3,
                   "correlated horiz. pos stddev still capped at 120 m");
        CHECK_NEAR(sqrtf(Qout[8]), 120.0f, 1e-3,
                   "correlated vert. pos stddev still capped at 120 m");
        const float rho_out = Qout[3] / sqrtf(Qout[0] * Qout[4]);
        CHECK_NEAR(rho_out, rho, 1e-3, "NE correlation coefficient preserved by the cap");
        /* Sylvester's criterion: all leading principal minors positive. */
        const float minor1 = Qout[0];
        const float minor2 = Qout[0] * Qout[4] - Qout[3] * Qout[1];
        const float minor3 = Qout[0] * (Qout[4] * Qout[8] - Qout[7] * Qout[5]) -
                             Qout[3] * (Qout[1] * Qout[8] - Qout[7] * Qout[2]) +
                             Qout[6] * (Qout[1] * Qout[5] - Qout[4] * Qout[2]);
        CHECK_TRUE(minor1 > 0.0f && minor2 > 0.0f && minor3 > 0.0f,
                   "capped covariance of a correlated fix stays positive definite");
    }

    /* The cap is a fusion weight, not a gate: with the fusion gate left at
       its default the degraded fix is still fused, never rejected. */
    {
        ins_options_t capped;
        fill_default_opt(&capped);
        capped.gnss_max_horizontal_pos_stddev_m = 0.0f; /* -> the default, i.e. the cap */
        capped.gnss_max_vertical_pos_stddev_m   = 0.0f;
        uint32_t used = 0, rejected = 0;
        (void)run_gnss_cov(&capped, &used, &rejected);
        CHECK_TRUE(used > 0, "default gate: fixes are fused");
        CHECK_TRUE(rejected == 0, "default gate: nothing rejected on reported accuracy");
    }
}

/* Runs a stationary filter fed GNSS position AND velocity at 1 Hz with the
   given per-epoch reported horizontal stddev, and reports the conditioned
   POSITION covariance the fusion saw in the final epoch together with the
   envelope state. Used for REQ-NAV-072. */
static void run_gnss_envelope(const ins_options_t* opt, const float* sigma_seq, int n_seq,
                              float* p_fuse_nn_out, float* env_out)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o = *opt;

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));

    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.1f; /* 10 Hz IMU, GNSS every 10th epoch -> 1 Hz */
    ins_time_us_t t  = 0;
    int           step;
    int           seq = 0;
    for (step = 1; step <= n_seq * 10; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % 10 == 0)
        {
            const float sd         = sigma_seq[seq++];
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = sd * sd;
            m.gnss_pos.is_valid                                                   = true;
        }
        ins_update(&f, &m);
    }
    /* step_ctx holds the last epoch's conditioned covariance. */
    if (p_fuse_nn_out) { *p_fuse_nn_out = f.step_ctx.gnss_pos_Qll_fuse[0]; }
    if (env_out) { *env_out = f.gnss_env_pos_hor_m; }
}

/* REQ-NAV-072: the reported accuracy is tracked asymmetrically -- a rise is
   adopted at once, a fall only decays out with the configured time constant. */
static void scenario_gnss_acc_envelope(void)
{
    printf("\n=== Scenario: GNSS accuracy envelope (REQ-NAV-072) ===\n");

    /* Nominal 2 m, one epoch at 20 m, then nominal again. tau 5 s at a 1 Hz
       fix rate -> the envelope loses a fifth of itself per epoch. The floor
       and cap are kept clear of both values so only the envelope acts. */
    float seq[6] = {2.0f, 2.0f, 20.0f, 2.0f, 2.0f, 2.0f};

    ins_options_t on;
    fill_default_opt(&on);
    on.gnss_acc_envelope_tau_sec   = 5.0f;
    on.gnss_pos_stddev_floor_hor_m = -1.0f;
    on.gnss_pos_stddev_floor_ver_m = -1.0f;

    /* Two epochs of quiet before the spike: the envelope sits on the report
       and the fusion sees exactly what was reported. */
    float p2 = 0.0f, e2 = 0.0f;
    run_gnss_envelope(&on, seq, 2, &p2, &e2);
    printf("  quiet:        envelope %.3f m, fused %.3f m\n", e2, sqrtf(p2));
    CHECK_NEAR(e2, 2.0f, 1e-3, "quiet stream: envelope sits on the reported 2 m");
    CHECK_NEAR(sqrtf(p2), 2.0f, 1e-3, "quiet stream: fusion weights the reported 2 m");

    /* The spike epoch itself: adopted in full, immediately. */
    float p3 = 0.0f, e3 = 0.0f;
    run_gnss_envelope(&on, seq, 3, &p3, &e3);
    printf("  spike:        envelope %.3f m, fused %.3f m\n", e3, sqrtf(p3));
    CHECK_NEAR(e3, 20.0f, 1e-3, "a rise is adopted at once");
    CHECK_NEAR(sqrtf(p3), 20.0f, 1e-2, "the spike epoch is fused at its own 20 m");

    /* One second later the receiver claims 2 m again. The envelope has only
       decayed by dt/tau = 1/5, so the fusion still weights 16 m. */
    float p4 = 0.0f, e4 = 0.0f;
    run_gnss_envelope(&on, seq, 4, &p4, &e4);
    printf("  spike + 1 s:  envelope %.3f m, fused %.3f m (reported 2 m)\n", e4, sqrtf(p4));
    CHECK_NEAR(e4, 16.0f, 1e-2, "a fall decays with the time constant, it is not adopted");
    CHECK_NEAR(sqrtf(p4), 16.0f, 1e-1, "the recovered fix is still fused at the envelope");

    /* And it keeps decaying towards the report rather than snapping to it. */
    float p6 = 0.0f, e6 = 0.0f;
    run_gnss_envelope(&on, seq, 6, &p6, &e6);
    printf("  spike + 3 s:  envelope %.3f m\n", e6);
    CHECK_TRUE(e6 < e4 && e6 > 2.0f, "the envelope keeps decaying towards the report");

    /* Opted out, the same stream is fused at face value throughout. */
    ins_options_t off             = on;
    off.gnss_acc_envelope_tau_sec = -1.0f;
    float p_off = 0.0f, e_off = 0.0f;
    run_gnss_envelope(&off, seq, 4, &p_off, &e_off);
    printf("  envelope off: fused %.3f m\n", sqrtf(p_off));
    CHECK_NEAR(sqrtf(p_off), 2.0f, 1e-3, "tau < 0: the recovered fix is fused at face value");
}

/* REQ-NAV-073: the GNSS velocity noise grows with the platform's measured
   acceleration, independently of anything the receiver reports. */
static void scenario_gnss_vel_noise_acc(void)
{
    printf("\n=== Scenario: manoeuvre-dependent GNSS velocity noise (REQ-NAV-073) ===\n");

    /* Level platform accelerating north at 4 m/s^2, GNSS velocity reported at
       1 m/s per axis (clear of the floor). A horizontal scale of 0.5 buys
       0.5 * 4 = 2 m/s of extra noise on N and E, and nothing on D. */
    const float acc_north = 4.0f;
    float       q_nn[2], q_dd[2];
    int         run;
    for (run = 0; run < 2; ++run)
    {
        ins_t f;
        memset(&f, 0, sizeof(f));
        ins_init_t init;
        fill_default_init(&init, 0);
        ins_options_t o;
        fill_default_opt(&o);
        o.auto_zupt_disable             = true;
        o.gnss_vel_noise_acc_scale_hor  = (run == 1) ? 0.5f : -1.0f;
        o.gnss_vel_noise_acc_scale_ver  = -1.0f;
        o.gnss_vel_stddev_floor_hor_mps = -1.0f;
        o.gnss_vel_stddev_floor_ver_mps = -1.0f;
        CHECK_TRUE(ins_init(&f, &init, &o) == 0, "init for the manoeuvre-noise run");

        float g_vec[3];
        ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
        const float acc_body[3] = {acc_north, 0.0f, -g_vec[2]};
        const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

        const float   dt = 0.01f;
        ins_time_us_t t  = 0;
        int           step;
        for (step = 1; step <= 200; ++step)
        {
            t += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = t;
            set_imu(&m, acc_body, gyr_body, dt);
            if (step % 20 == 0)
            {
                m.gnss_vel.vel_ned[0] = acc_north * (float)step * dt;
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 1.0f;
                m.gnss_vel.is_valid                                                   = true;
            }
            ins_update(&f, &m);
        }
        q_nn[run] = f.step_ctx.gnss_vel_Qll_fuse[0];
        q_dd[run] = f.step_ctx.gnss_vel_Qll_fuse[8];
    }

    printf("  term off:  N %.3f (m/s)^2, D %.3f;  scale 0.5: N %.3f, D %.3f\n", q_nn[0], q_dd[0],
           q_nn[1], q_dd[1]);
    CHECK_NEAR(q_nn[0], 1.0f, 1e-3,
               "term off: the reported 1 (m/s)^2 reaches the fusion unchanged");
    /* 1 + (0.5 * 4)^2 = 5, allowing for the strapdown's own acceleration estimate. */
    CHECK_TRUE(q_nn[1] > 4.5f && q_nn[1] < 5.5f, "scale 0.5 at 4 m/s^2 adds (2 m/s)^2 on N");
    CHECK_NEAR(q_dd[1], q_dd[0], 1e-3, "the vertical axis is untouched with its scale off");
}

/* Drives the IMU with a north acceleration profile and fuses a single GNSS
   velocity at the end, returning the N variance the fusion actually weighted
   it with. `profile` selects the acceleration fed on each of the last
   `tail_steps` epochs: 0 = the manoeuvre stops (level), 1 = it alternates
   sign every epoch (zero-mean vibration). Used for REQ-NAV-075. */
static float run_gnss_manoeuvre_window(float window_sec, int profile, int tail_steps)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o;
    fill_default_opt(&o);
    o.auto_zupt_disable             = true;
    o.gnss_vel_noise_acc_scale_hor  = 0.5f;
    o.gnss_vel_noise_acc_scale_ver  = -1.0f;
    o.gnss_vel_noise_acc_window_sec = window_sec;
    o.gnss_vel_stddev_floor_hor_mps = -1.0f;
    o.gnss_vel_stddev_floor_ver_mps = -1.0f;
    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return 0.0f;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_north   = 4.0f;
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt         = 0.01f;
    const int     lead_steps = 200; /* 2 s of steady manoeuvre */
    ins_time_us_t t          = 0;
    int           step;
    for (step = 1; step <= lead_steps + tail_steps; ++step)
    {
        t += us_from_sec(dt);
        float a_n = acc_north;
        if (step > lead_steps)
        {
            /* profile 0: the manoeuvre has ended. profile 1: it never was a
               manoeuvre, only a zero-mean oscillation. */
            if (profile == 0) { a_n = 0.0f; }
            else { a_n = ((step & 1) != 0) ? acc_north : -acc_north; }
        }
        const float        acc_body[3] = {a_n, 0.0f, -g_vec[2]};
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step == lead_steps + tail_steps)
        {
            m.gnss_vel.vel_ned[0] = acc_north * (float)lead_steps * dt;
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
            m.gnss_vel.is_valid                                                   = true;
        }
        ins_update(&f, &m);
    }
    return f.step_ctx.gnss_vel_Qll_fuse[0];
}

/* REQ-NAV-075: the manoeuvre term reads the acceleration averaged over the
   measurement window, not the instantaneous sample. The two properties that
   buys are opposite in sign and are checked separately. */
static void scenario_gnss_manoeuvre_window(void)
{
    printf("\n=== Scenario: manoeuvre-noise averaging window (REQ-NAV-075) ===\n");

    /* (A) A manoeuvre that has just ended still carries its error into the
       next fix. 0.1 s after a 4 m/s^2 run the 0.2 s window still holds most
       of it, while the instantaneous sample has already forgotten it. */
    const float q_win = run_gnss_manoeuvre_window(0.2f, 0, 10);
    const float q_ins = run_gnss_manoeuvre_window(-1.0f, 0, 10);
    printf("  manoeuvre ended 0.1 s ago: windowed %.3f (m/s)^2, instantaneous %.3f\n", q_win,
           q_ins);
    CHECK_NEAR(q_ins, 0.01f, 1e-3,
               "instantaneous: a finished manoeuvre is unpriced on the fix that follows it");
    CHECK_TRUE(q_win > 0.5f, "windowed: the fix is still priced for the manoeuvre it covers");

    /* (B) A zero-mean oscillation is not a manoeuvre. Averaging the vector
       cancels it; the instantaneous sample prices whichever half of the
       cycle the fix happened to land in. */
    const float v_win = run_gnss_manoeuvre_window(0.2f, 1, 100);
    const float v_ins = run_gnss_manoeuvre_window(-1.0f, 1, 100);
    printf("  zero-mean 4 m/s^2 oscillation: windowed %.3f (m/s)^2, instantaneous %.3f\n", v_win,
           v_ins);
    CHECK_TRUE(v_ins > 3.0f, "instantaneous: vibration is priced as if it were a manoeuvre");
    CHECK_TRUE(v_win < 0.2f, "windowed: a zero-mean oscillation cancels and costs nothing");
}

/* Spins a level platform about its own body z-axis at `yaw_rate` (rad/s) with
   the GNSS antenna at `lever_b`, and fuses a single GNSS velocity at the end.
   The accelerometer sees nothing but gravity throughout, so whatever the
   manoeuvre term charges comes from the lever arm alone. `oscillate` flips
   the sign of the rotation on every IMU epoch, turning the steady spin into a
   zero-mean shake. Returns the N variance the fusion weighted the fix with.
   Used for REQ-NAV-076. */
static float run_gnss_leverarm_rate(const float lever_b[3], float yaw_rate, int oscillate)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o;
    fill_default_opt(&o);
    o.auto_zupt_disable             = true;
    o.gnss_vel_noise_acc_scale_hor  = 0.5f;
    o.gnss_vel_noise_acc_scale_ver  = -1.0f;
    o.gnss_vel_noise_acc_window_sec = 0.2f;
    o.gnss_vel_stddev_floor_hor_mps = -1.0f;
    o.gnss_vel_stddev_floor_ver_mps = -1.0f;
    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return 0.0f;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};

    const float   dt    = 0.01f;
    const int     steps = 300; /* 3 s, far past the 0.2 s window */
    ins_time_us_t t     = 0;
    int           step;
    for (step = 1; step <= steps; ++step)
    {
        t += us_from_sec(dt);
        float w = yaw_rate;
        if (oscillate && (step & 1) == 0) { w = -yaw_rate; }
        const float        gyr_body[3] = {0.0f, 0.0f, w};
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step == steps)
        {
            m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = 0.01f;
            m.gnss_vel.is_valid                                                   = true;
            m.gnss_leverarm_b[0]                                                  = lever_b[0];
            m.gnss_leverarm_b[1]                                                  = lever_b[1];
            m.gnss_leverarm_b[2]                                                  = lever_b[2];
        }
        ins_update(&f, &m);
    }
    return f.step_ctx.gnss_vel_Qll_fuse[0];
}

/* REQ-NAV-076: the acceleration the manoeuvre term prices is the antenna's,
   so a platform that only rotates still pays for the arc its antenna swings
   through. The accelerometer at the centre of that rotation reads nothing. */
static void scenario_gnss_vel_noise_leverarm(void)
{
    printf("\n=== Scenario: antenna acceleration from the lever arm (REQ-NAV-076) ===\n");

    /* 3 rad/s about body z with a 1 m arm along body x: the antenna sees
       omega^2 * l = 9 m/s^2 of centripetal acceleration, which a scale of
       0.5 turns into 4.5 m/s of extra noise on top of the reported 0.01. */
    const float arm_x[3]  = {1.0f, 0.0f, 0.0f};
    const float arm_z[3]  = {0.0f, 0.0f, 1.0f};
    const float no_arm[3] = {0.0f, 0.0f, 0.0f};

    const float q_arm  = run_gnss_leverarm_rate(arm_x, 3.0f, 0);
    const float q_none = run_gnss_leverarm_rate(no_arm, 3.0f, 0);
    printf("  spinning at 3 rad/s: 1 m arm %.3f (m/s)^2, no arm %.3f\n", q_arm, q_none);
    CHECK_NEAR(q_none, 0.01f, 1e-3, "no lever arm: rotation alone costs nothing");
    CHECK_TRUE(q_arm > 19.0f && q_arm < 21.5f, "1 m arm at 3 rad/s adds (4.5 m/s)^2 on N");

    /* An arm along the rotation axis sweeps no arc, so it is not a lever arm
       for this rotation and must be priced as none. */
    const float q_axis = run_gnss_leverarm_rate(arm_z, 3.0f, 0);
    printf("  1 m arm along the yaw axis: %.3f (m/s)^2\n", q_axis);
    CHECK_NEAR(q_axis, 0.01f, 1e-3, "an arm parallel to omega adds no centripetal term");

    /* The rotation half is averaged as omega*omega', not as omega, so a
       zero-mean shake keeps its full price. This is deliberately the opposite
       of what the same window does to a zero-mean linear vibration
       (REQ-NAV-075): the antenna really does swing out on every half cycle. */
    const float q_osc = run_gnss_leverarm_rate(arm_x, 3.0f, 1);
    printf("  same rate, sign flipped every epoch: %.3f (m/s)^2\n", q_osc);
    CHECK_TRUE(q_osc > 19.0f && q_osc < 21.5f,
               "a zero-mean yaw oscillation is priced like the steady spin");
}

/* Stationary run with GNSS position fixes carrying a reported NED covariance
   with unit horizontal/vertical variance and a chosen N-D / E-D correlation
   (rho). Reports the posterior north (P_NN), east (P_EE) and down (P_DD)
   variances and whether GNSS was fused. Used to exercise the dedicated
   height downweight (REQ-NAV-041). */
static void run_gnss_height(const ins_options_t* opt, float rho, float* pnn_out, float* pee_out,
                            float* pdd_out, uint32_t* used_out)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t o = *opt;

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));

    if (ins_init(&f, &init, &o) != 0)
    {
        printf("  init failed\n");
        fails++;
        return;
    }

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % 20 == 0)
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            /* Reported NED position covariance, column-major, unit variances,
               N-D and E-D correlation rho (rho=0 -> diagonal). */
            m.gnss_pos.Qll_ned[0] = 1.0f; /* NN */
            m.gnss_pos.Qll_ned[4] = 1.0f; /* EE */
            m.gnss_pos.Qll_ned[8] = 1.0f; /* DD */
            m.gnss_pos.Qll_ned[2] = rho;  /* DN (2,0) */
            m.gnss_pos.Qll_ned[6] = rho;  /* ND (0,2) */
            m.gnss_pos.Qll_ned[5] = rho;  /* DE (2,1) */
            m.gnss_pos.Qll_ned[7] = rho;  /* ED (1,2) */
            m.gnss_pos.is_valid   = true;
        }
        ins_update(&f, &m);
    }
    if (pnn_out) { *pnn_out = pos_var_ii(&f, INS_IDX_POS + 0); }
    if (pee_out) { *pee_out = pos_var_ii(&f, INS_IDX_POS + 1); }
    if (pdd_out) { *pdd_out = pos_var_ii(&f, INS_IDX_POS + 2); }
    if (used_out) { *used_out = ins_get_diag(&f)->n_gnss_used; }
}

/* @satisfies-test REQ-NAV-041 (trace lives in the requirements DB) */
static void scenario_gnss_pos_height_scale(void)
{
    printf("\n=== Scenario: GNSS dedicated height (vertical) downweight ===\n");

    ins_options_t base;
    fill_default_opt(&base);
    base.gnss_max_horizontal_pos_stddev_m = 100.0f; /* keep the gates out of the way */
    base.gnss_max_vertical_pos_stddev_m   = 100.0f;

    /* 1) A pure height scale on a diagonal reported covariance inflates the
          DOWN posterior but leaves the horizontal (N,E) posterior bit-for-bit
          unchanged: the defining difference from the isotropic scale. */
    float    pnn0 = 0, pee0 = 0, pdd0 = 0;
    uint32_t used0 = 0;
    run_gnss_height(&base, 0.0f, &pnn0, &pee0, &pdd0, &used0);

    ins_options_t h             = base;
    h.gnss_pos_cov_scale_height = 5.0f;
    float    pnnH = 0, peeH = 0, pddH = 0;
    uint32_t usedH = 0;
    run_gnss_height(&h, 0.0f, &pnnH, &peeH, &pddH, &usedH);

    printf("  P_DD: height=1 -> %g, height=5 -> %g ; P_NN: %g -> %g\n", pdd0, pddH, pnn0, pnnH);
    CHECK_TRUE(used0 > 0 && usedH > 0, "height: GNSS fused in both runs");
    CHECK_TRUE(pddH > pdd0 * 4.0f, "height=5 inflates the DOWN posterior");
    CHECK_NEAR(pnnH, pnn0, (double)pnn0 * 1e-3, "height scale leaves NORTH posterior ~unchanged");
    CHECK_NEAR(peeH, pee0, (double)pee0 * 1e-3, "height scale leaves EAST posterior ~unchanged");

    /* 2) Contrast: the isotropic pos scale of the same magnitude DOES inflate
          the horizontal posterior, proving the height knob is axis-selective. */
    ins_options_t iso      = base;
    iso.gnss_pos_cov_scale = 5.0f;
    float pnnI             = 0;
    run_gnss_height(&iso, 0.0f, &pnnI, (float*)0, (float*)0, (uint32_t*)0);
    CHECK_TRUE(pnnI > pnn0 * 2.0f, "isotropic scale (unlike height) inflates the NORTH posterior");

    /* 3) With a correlated (N-D, E-D) reported covariance, rho=0.5 keeps the
          base matrix PSD (0.5^2+0.5^2 < 1). The height downweight preserves
          validity: the fix is still fused and the posterior stays finite,
          exercising the off-diagonal congruence terms (N-D/E-D *= s). */
    ins_options_t hc             = base;
    hc.gnss_pos_cov_scale_height = 5.0f;
    float    pnnC = 0, pddC = 0;
    uint32_t usedC = 0;
    run_gnss_height(&hc, 0.5f, &pnnC, (float*)0, &pddC, &usedC);
    CHECK_TRUE(usedC > 0, "height scale on a correlated cov still fuses GNSS (stays PSD)");
    CHECK_TRUE(isfinite(pnnC) && isfinite(pddC) && pddC > 0.0f,
               "correlated + height-scaled posterior is finite and positive");
}

/* Run 10 s of stationary IMU + a fixed body-frame magnetometer vector
   (truth attitude identity, filter starts 10 deg off in yaw) under the
   given mag fixed-bias calibration; return the converged yaw. */
static float run_mag_yaw(const float mag_fixed_bias[3], const float mag_raw[3])
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.mag_fixed_bias[0] = mag_fixed_bias[0];
    opt.mag_fixed_bias[1] = mag_fixed_bias[1];
    opt.mag_fixed_bias[2] = mag_fixed_bias[2];
    init.rpy_init_rad[2]  = 10.0f * (float)M_PI / 180.0f;
    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("  init failed\n");
        fails++;
        return 0.0f;
    }
    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float   acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float   gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float   dt          = 0.01f;
    ins_time_us_t t           = 0;
    int           step;
    for (step = 1; step <= 1000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.mag.data[0]     = mag_raw[0];
        m.mag.data[1]     = mag_raw[1];
        m.mag.data[2]     = mag_raw[2];
        m.mag.Qll_diag[0] = m.mag.Qll_diag[1] = m.mag.Qll_diag[2] = 0.25f;
        m.mag.is_valid                                            = true;
        ins_update(&f, &m);
    }
    float roll, pitch, yaw;
    ins_get_rpy(&f, &roll, &pitch, &yaw);
    return yaw;
}

/* REQ-NAV-039: magnetometer calibration (fixed hard-iron bias) applied to
   the raw mag before fusion. A horizontal hard-iron bias corrupts the
   heading; the fixed-bias config removes it so yaw converges to truth. */
static void scenario_mag_calibration(void)
{
    printf("\n=== Scenario: magnetometer calibration (fixed hard-iron bias) ===\n");
    ins_init_t init;
    fill_default_init(&init, 0);
    /* Raw body field = true n-frame field (identity attitude) + a hard-iron
       bias in body east (10 uT), which rotates the horizontal component and
       so corrupts the derived heading. */
    const float b_hi[3]    = {0.0f, 10.0f, 0.0f};
    const float mag_raw[3] = {init.magnetic_n[0], init.magnetic_n[1] + b_hi[1], init.magnetic_n[2]};
    const float none[3]    = {0.0f, 0.0f, 0.0f};

    const float y_nocal = run_mag_yaw(none, mag_raw);
    const float y_cal   = run_mag_yaw(b_hi, mag_raw);
    printf("  yaw no-cal = %.4f rad, yaw hard-iron-removed = %.4f rad\n", y_nocal, y_cal);
    CHECK_TRUE(fabsf(y_nocal) > 0.15f, "mag no-cal: hard-iron bias corrupts yaw");
    CHECK_NEAR(y_cal, 0.0f, 0.02, "mag fixed bias: yaw converges to truth");
}

/* Run a stationary filter for 10 s feeding GNSS at truth with the given
   per-axis pos/vel stddev; auto-ZUPT off so the covariance is driven only by
   the GNSS we feed. Returns the diag snapshot via out. */
static void run_overconf(float gnss_pos_sd, float gnss_vel_sd, bool feed_vel, ins_diag_t* out)
{
    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_zupt_disable = true; /* isolate the covariance to the fed GNSS */
    /* The watchdog is the subject here, so the GNSS covariance floors
       (REQ-NAV-038) are opted out: with them in place no reported accuracy
       can drive the covariance down far enough to collapse it, which is
       precisely their job and would leave this scenario testing them
       instead. */
    opt.gnss_pos_stddev_floor_hor_m   = -1.0f;
    opt.gnss_pos_stddev_floor_ver_m   = -1.0f;
    opt.gnss_vel_stddev_floor_hor_mps = -1.0f;
    opt.gnss_vel_stddev_floor_ver_mps = -1.0f;
    if (ins_init(&f, &init, &opt) != 0)
    {
        printf("  init failed\n");
        fails++;
        return;
    }
    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));
    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float pv          = gnss_pos_sd * gnss_pos_sd;
    const float vv          = gnss_vel_sd * gnss_vel_sd;

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    for (step = 1; step <= 1000; ++step)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        if (step % 5 == 0)
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = pv;
            m.gnss_pos.is_valid                                                   = true;
            if (feed_vel)
            {
                m.gnss_vel.vel_ned[0] = m.gnss_vel.vel_ned[1] = m.gnss_vel.vel_ned[2] = 0.0f;
                m.gnss_vel.Qll_ned[0] = m.gnss_vel.Qll_ned[4] = m.gnss_vel.Qll_ned[8] = vv;
                m.gnss_vel.is_valid                                                   = true;
            }
        }
        ins_update(&f, &m);
    }
    *out = *ins_get_diag(&f);
}

/* REQ-NAV-040: the filter flags when its reported covariance becomes
   implausibly small (covariance collapse), diagnostic only. */
static void scenario_overconfidence_watchdog(void)
{
    printf("\n=== Scenario: overconfidence / covariance-collapse watchdog ===\n");

    /* Part A: implausibly precise GNSS (1e-6 m / 1e-6 m/s) collapses the
       position and velocity covariance below the floor -> flag trips. */
    ins_diag_t da;
    run_overconf(1e-6f, 1e-6f, true, &da);
    printf("  precise: min stddev pos %.2e m, vel %.2e m/s, att %.2e deg; n=%u\n",
           da.min_pos_stddev_m, da.min_vel_stddev_mps, da.min_att_stddev_deg, da.n_overconfident);
    CHECK_TRUE(da.overconfident, "collapsed covariance trips the overconfidence flag");
    CHECK_TRUE(da.n_overconfident > 0, "overconfident epochs counted");
    CHECK_TRUE(da.min_pos_stddev_m < 1e-4f, "min pos stddev fell below the floor");
    CHECK_TRUE(da.min_vel_stddev_mps < 1e-4f, "min vel stddev fell below the floor");

    /* Part B: ordinary GNSS (1 m) never reaches the floor -> flag stays off. */
    ins_diag_t db;
    run_overconf(1.0f, 0.0f, false, &db);
    printf("  normal:  min stddev pos %.3g m; overconfident=%d\n", db.min_pos_stddev_m,
           (int)db.overconfident);
    CHECK_TRUE(!db.overconfident, "ordinary 1 m GNSS never trips the flag");
    CHECK_TRUE(db.n_overconfident == 0, "no overconfident epochs on ordinary GNSS");
    CHECK_TRUE(db.min_pos_stddev_m > 1e-4f, "min pos stddev stays physically plausible");
}

static void scenario_chi2_disable(void)
{
    printf("\n=== Scenario: global chi2-disable override ===\n");

    const float pn_downweighted = run_gnss_outlier(false);
    const float pn_raw          = run_gnss_outlier(true);

    printf("  north shift: downweighted=%g m, chi2_disable=%g m\n", pn_downweighted, pn_raw);
    CHECK_TRUE(fabsf(pn_downweighted) < 5.0f, "default: outlier downweighted, barely moves");
    CHECK_TRUE(pn_raw > 100.0f, "chi2_disable: outlier fused at face value, jumps close to it");
}

static void scenario_chi2_reject_alpha(void)
{
    printf("\n=== Scenario: global chi2 reject-alpha (REQ-NAV-046) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);

    /* Resolution: alpha 0 resolves to ins.c's single unified default (10.0,
       INS_DEFAULT_CHI2_GATE), identically on all four channels; a configured
       alpha overrides all four with the single global chi2inv(1-alpha, 1).
       No differing per-channel defaults remain. */
    ins_t         f;
    ins_options_t opt;

    memset(&f, 0, sizeof(f));
    fill_default_opt(&opt);
    opt.chi2_reject_alpha = 0.0f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init alpha 0");
    CHECK_NEAR(f.chi2_thr_gnss, 10.0f, 1e-4, "alpha 0: GNSS gate defaults to 10.0");
    CHECK_NEAR(f.chi2_thr_local, 10.0f, 1e-4, "alpha 0: local gate defaults to 10.0");
    CHECK_NEAR(f.chi2_thr_mag, 10.0f, 1e-4, "alpha 0: mag gate defaults to 10.0");
    CHECK_NEAR(f.chi2_thr_yaw, 10.0f, 1e-4, "alpha 0: yaw gate defaults to 10.0");

    memset(&f, 0, sizeof(f));
    fill_default_opt(&opt);
    opt.chi2_reject_alpha = 0.05f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init alpha 0.05");
    CHECK_NEAR(f.chi2_thr_gnss, 3.8415f, 1e-3, "alpha 0.05 -> 95% threshold 3.84 (GNSS)");
    CHECK_NEAR(f.chi2_thr_mag, 3.8415f, 1e-3, "alpha 0.05 -> 95% threshold 3.84 (mag)");
    CHECK_NEAR(f.chi2_thr_yaw, 3.8415f, 1e-3, "alpha 0.05 -> 95% threshold 3.84 (yaw)");

    memset(&f, 0, sizeof(f));
    fill_default_opt(&opt);
    opt.chi2_reject_alpha = 0.01f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init alpha 0.01");
    CHECK_NEAR(f.chi2_thr_gnss, 6.6349f, 1e-3, "alpha 0.01 -> 99% threshold 6.63 (GNSS)");

    /* Interpolation: alpha 0.03 (p=0.97) lands between the 95% and 97.5% rows. */
    memset(&f, 0, sizeof(f));
    fill_default_opt(&opt);
    opt.chi2_reject_alpha = 0.03f;
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init alpha 0.03");
    CHECK_TRUE(f.chi2_thr_gnss > 3.8415f && f.chi2_thr_gnss < 5.0239f,
               "alpha 0.03 interpolates between 95% and 97.5%");

    /* Behaviour: one moderate outlier the default gate (10.0) accepts but a
       stricter alpha downweights: same fix, only the gate changes. */
    const uint32_t dw_default = run_gnss_moderate_outlier(0.0f, 5.0f, 2.0f);
    const uint32_t dw_strict  = run_gnss_moderate_outlier(0.20f, 5.0f, 2.0f);
    printf("  moderate outlier: default downweights=%u, strict(alpha 0.2)=%u\n", dw_default,
           dw_strict);
    CHECK_TRUE(dw_default == 0, "default gate accepts the moderate outlier");
    CHECK_TRUE(dw_strict == 1, "strict alpha downweights the same outlier");
}

/* ins_update() must be exactly ins_predict_step() followed by
   ins_correct_step() (REQ-NAV-069): drive two identical filters through
   the same epoch stream, one via ins_update(), the other manually split,
   and require the end state/covariance to match bit-for-bit. Also checks
   that ins_predict_step()'s phi_out is filled with the expected pos/vel
   coupling whenever the covariance is actually propagated. */
/* Guard rails on the split predict/correct entry points and on
 * configuration values the filter cannot honour. Both are pure
 * error paths, so nothing else in this file ever walks them.
 */
/* Divergence diagnostics (see log.h): the filter watches its own bias
 * estimates and its yaw uncertainty and reports when either leaves the
 * range a MEMS IMU can plausibly produce. The reports themselves are
 * log output, but the throttling state behind them is filter state, so
 * that is what this scenario reads -- which also keeps it meaningful in
 * a build that compiles the log calls out.
 *
 * The thresholds are private to ins.c (INS_LOG_*): 10 s warm-up before
 * any of this runs, a 10 s runaway window, sanity bounds at 10 deg/s and
 * 2 m/s^2, and a 60 s throttle between repeats of the same complaint.
 */
static void scenario_bias_diagnostics(void)
{
    printf("\n=== Scenario: bias-runaway and bias-sanity diagnostics ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_zupt_disable = true; /* a ZUPT would fight the injected bias */

    ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    float g_vec[3];
    ins_gravity_ned((float)f.latlonh[0], (float)f.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    ins_time_us_t t = 0;
    start_manual_filter(&f, &t, dt, (const double*)0);

    int i;
#define DIAG_RUN(SECONDS)                        \
    do {                                         \
        int n_ = (int)((SECONDS) / dt + 0.5f);   \
        for (i = 0; i < n_; ++i)                 \
        {                                        \
            t += us_from_sec(dt);                \
            ins_measurements_t m;                \
            memset(&m, 0, sizeof(m));            \
            m.timestamp = t;                     \
            set_imu(&m, acc_body, gyr_body, dt); \
            ins_update(&f, &m);                  \
        }                                        \
    } while (0)

    /* Below the warm-up nothing is judged yet: the bias estimate is
       legitimately still moving away from a loose prior. */
    DIAG_RUN(5.0f);
    CHECK_TRUE(f.log_state.t_gyr_bias_window == 0, "no bias window opened inside the warm-up");

    DIAG_RUN(8.0f); /* past the 10 s warm-up: the first window opens */
    CHECK_TRUE(f.log_state.t_gyr_bias_window != 0, "bias window opened after the warm-up");
    CHECK_TRUE(f.log_state.t_last_gyr_bias_sanity_warn == 0, "a healthy bias raises nothing");

    /* A diverging filter, injected directly: no realistic sensor input
       drives an estimate this far, and the point here is the watchdog,
       not the path that got there. Both bounds are cleared several times
       over (20 deg/s and 5 m/s^2). */
    f.state.gyr_bias[0] = DEG2RAD(20.0f);
    f.state.acc_bias[0] = 5.0f;

    DIAG_RUN(11.0f); /* one full runaway window with the new values */
    CHECK_TRUE(f.log_state.t_last_gyr_bias_sanity_warn != 0,
               "an implausible gyro bias is reported");
    CHECK_TRUE(f.log_state.t_last_acc_bias_sanity_warn != 0,
               "an implausible accel bias is reported");
    CHECK_TRUE(f.log_state.gyr_bias_window_dps > 10.0f,
               "the gyro window carries the new magnitude");
    CHECK_TRUE(f.log_state.acc_bias_window_mps2 > 2.0f,
               "the accel window carries the new magnitude");

    /* Throttled, not repeated per epoch: still implausible, but the
       complaint stays put until the repeat interval has passed. */
    const ins_time_us_t first_gyr_warn = f.log_state.t_last_gyr_bias_sanity_warn;
    const ins_time_us_t first_acc_warn = f.log_state.t_last_acc_bias_sanity_warn;
    DIAG_RUN(20.0f);
    CHECK_TRUE(f.log_state.t_last_gyr_bias_sanity_warn == first_gyr_warn,
               "the gyro complaint is throttled");
    CHECK_TRUE(f.log_state.t_last_acc_bias_sanity_warn == first_acc_warn,
               "the accel complaint is throttled");

    DIAG_RUN(105.0f); /* now past the bias-sanity repeat interval */
    CHECK_TRUE(f.log_state.t_last_gyr_bias_sanity_warn > first_gyr_warn,
               "and repeated once the interval has passed");
    CHECK_TRUE(f.log_state.t_last_acc_bias_sanity_warn > first_acc_warn,
               "same for the accel complaint");
#undef DIAG_RUN

    /* Yaw-stddev runaway: an unaided yaw whose prior grows fast enough to
       add more than 5 deg of stddev inside one 10 s window. A process
       noise of 2 deg/sqrt(s) does that on its own, with no aiding at all. */
    {
        ins_init_t init_yaw;
        fill_default_init(&init_yaw, 0);
        init_yaw.rpy_init_stddev_rad[2]    = DEG2RAD(15.0f); /* already past the warn level */
        init_yaw.rpy_pred_stddev_rad_sqrts = DEG2RAD(2.0f);

        ins_t f_yaw;
        memset(&f_yaw, 0, sizeof(f_yaw));
        ins_options_t opt_yaw;
        fill_default_opt(&opt_yaw);
        opt_yaw.auto_zupt_disable = true;
        CHECK_TRUE(ins_init(&f_yaw, &init_yaw, &opt_yaw) == 0, "yaw runaway init ok");

        ins_time_us_t ty = 0;
        start_manual_filter(&f_yaw, &ty, dt, (const double*)0);
        for (i = 0; i < 3000; ++i) /* 30 s: three runaway windows */
        {
            ty += us_from_sec(dt);
            ins_measurements_t m;
            memset(&m, 0, sizeof(m));
            m.timestamp = ty;
            set_imu(&m, acc_body, gyr_body, dt);
            ins_update(&f_yaw, &m);
        }
        CHECK_TRUE(f_yaw.log_state.t_yaw_stddev_window != 0, "yaw stddev window tracked");
        CHECK_TRUE(f_yaw.log_state.yaw_stddev_window_deg > 15.0f,
                   "the unaided yaw stddev grew well past its prior");
        CHECK_TRUE(f_yaw.log_state.t_last_yaw_stddev_warn != 0, "the unaided yaw is reported");
    }
}

/* Persistent magnetic disturbance diagnostics (see log.h): a single
 * glitchy magnetometer epoch is ordinary and already handled by the
 * downweight, but a long run of them means the yaw estimate is riding
 * on a corrupted reference, and that is reported once and then
 * repeated at a throttle rather than per epoch.
 *
 * Thresholds are private to ins.c (INS_LOG_MAG_DISTURB_*): 10
 * consecutive off-model samples before the first report, 30 s between
 * repeats.
 */
static void scenario_mag_disturbance_diagnostics(void)
{
    printf("\n=== Scenario: persistent magnetic disturbance diagnostics ===\n");

    ins_t f;
    memset(&f, 0, sizeof(f));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    /* The field-strength check needs a reference magnitude to compare
       against, which only the world magnetic model provides. */
    ins_set_magnetic_model_from_position(&f, 48.783 * M_PI / 180.0, 9.181 * M_PI / 180.0, 2027.5f);
    CHECK_TRUE(f.mag_field_expected_uT > 0.0f, "expected field strength armed");

    /* A field of the right direction but twice the expected magnitude:
       the direction keeps the yaw sane while the magnitude is what the
       field-strength check reads. */
    const float clean[3] = {f.magnetic_n[0], f.magnetic_n[1], f.magnetic_n[2]};
    const float dirty[3] = {2.0f * clean[0], 2.0f * clean[1], 2.0f * clean[2]};

    ins_time_us_t t = 0;
    feed_mag_steps(&f, &t, 500, clean, 0.25f); /* 5 s clean: nothing to report */
    CHECK_TRUE(f.log_state.mag_disturbed_count == 0, "a clean field disturbs nothing");
    CHECK_TRUE(f.log_state.t_last_mag_disturb_warn == 0, "and reports nothing");

    /* The magnetometer is rate limited (magnetometer_min_delay_ms), so
       the run counts fused epochs, not IMU epochs: 10 Hz here. */
    feed_mag_steps(&f, &t, 300, dirty, 0.25f); /* 3 s -> ~30 samples > 10 */
    CHECK_TRUE(f.log_state.mag_disturbed_count > 10, "a persistent disturbance is tracked");
    const ins_time_us_t first_warn = f.log_state.t_last_mag_disturb_warn;
    CHECK_TRUE(first_warn != 0, "and reported once the run is long enough");

    /* Still disturbed, but inside the repeat interval: no new report. */
    feed_mag_steps(&f, &t, 1000, dirty, 0.25f); /* 10 s, well inside it */
    CHECK_TRUE(f.log_state.t_last_mag_disturb_warn == first_warn,
               "the report is throttled while the disturbance continues");

    /* Past the repeat interval it is raised again: a disturbance that
       never ends must not fall silent. */
    feed_mag_steps(&f, &t, 24500, dirty, 0.25f); /* 245 s more -> past it */
    CHECK_TRUE(f.log_state.t_last_mag_disturb_warn > first_warn,
               "and repeated once the interval has passed");

    /* A clean field ends the run, so the next disturbance is a new event
       rather than a continuation of this one. */
    feed_mag_steps(&f, &t, 200, clean, 0.25f);
    CHECK_TRUE(f.log_state.mag_disturbed_count == 0, "a clean field clears the run");
}

static void scenario_api_and_config_guards(void)
{
    printf("\n=== Scenario: predict/correct guards and unusable config values ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = us_from_sec(0.01f);
    {
        const float acc[3] = {0.0f, 0.0f, -9.80665f};
        const float gyr[3] = {0.0f, 0.0f, 0.0f};
        set_imu(&m, acc, gyr, 0.01f);
    }

    /* A NULL filter is not something the caller can be told about other
       than through the return value. */
    CHECK_TRUE((ins_predict_step((ins_t*)0, &m, (float*)0) & INS_EPOCH_DROPPED) != 0,
               "predict_step on a NULL filter drops the epoch");

    /* A NULL measurement set must also disarm the step context, so a
       correct_step() that follows anyway fuses nothing from the previous
       epoch instead of re-running it. */
    CHECK_TRUE(
        (ins_predict_step(&f, (const ins_measurements_t*)0, (float*)0) & INS_EPOCH_DROPPED) != 0,
        "predict_step without measurements drops the epoch");
    CHECK_TRUE(!f.step_ctx.active, "and disarms the step context");
    const uint32_t updates_before = ins_get_diag(&f)->n_updates;
    ins_correct_step(&f); /* must be a no-op, not a crash or a second fusion */
    CHECK_TRUE(ins_get_diag(&f)->n_updates == updates_before,
               "correct_step after a dropped epoch changes nothing");
    ins_correct_step((ins_t*)0); /* NULL filter: same contract */

    /* gnss_pos_decimation: 0 means "default", 1 is the explicit opt-out,
       and anything below 1 is a configuration error that must disable the
       decimation rather than turn into a modulo by a negative number. */
    {
        ins_t         f_neg;
        ins_options_t opt_neg;
        fill_default_opt(&opt_neg);
        opt_neg.gnss_pos_decimation = -3;
        memset(&f_neg, 0, sizeof(f_neg));
        CHECK_TRUE(ins_init(&f_neg, &init, &opt_neg) == 0, "init with a negative decimation");
        CHECK_TRUE(f_neg.opt.gnss_pos_decimation == 1,
                   "a negative decimation factor disables decimation");
    }
    {
        ins_t         f_zero;
        ins_options_t opt_zero;
        fill_default_opt(&opt_zero);
        opt_zero.gnss_pos_decimation = 0;
        memset(&f_zero, 0, sizeof(f_zero));
        CHECK_TRUE(ins_init(&f_zero, &init, &opt_zero) == 0, "init with decimation 0");
        CHECK_TRUE(f_zero.opt.gnss_pos_decimation >= 1, "0 resolves to the built-in default");
    }
}

/* A delayed measurement is anchored on the filter state at its time of
 * validity, which only exists while that time is still inside the
 * history ring. A young filter has not accumulated one yet, so the
 * sample has to be skipped rather than anchored on "now" -- silently
 * fusing it against the current state would inject the delay as a
 * position/heading error.
 */
static void scenario_delayed_without_history(void)
{
    printf("\n=== Scenario: delayed samples with no history anchor yet ===\n");

    /* 13 m/s cruise, so the speed channel and the course-over-ground yaw
       are both above their minimum-speed gates. */
    const float vel_ned[3] = {12.0f, 5.0f, 0.0f};
    /* Inside INS_MAX_DELAY_MS, so the delay itself is acceptable and only
       the missing history entry can reject the sample. */
    const int deep_delay_ms = 400;

    ins_init_t init;
    fill_default_init(&init, 0);
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        float R0[9];
        ins_rotmat_n_to_e(lat, lon, R0);
        int k;
        for (k = 0; k < 3; ++k)
        {
            init.xdot_ecef[k] =
                R0[k] * vel_ned[0] + R0[3 + k] * vel_ned[1] + R0[6 + k] * vel_ned[2];
        }
    }

    float g_vec[3];
    ins_gravity_ned((float)(48.783 * M_PI / 180.0), 300.0f, g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    /* One IMU epoch, optionally carrying a delayed speed sample. */
#define DELAY_STEP(F, T, SPEED_MPS, DELAY_MS)  \
    do {                                       \
        (T) += us_from_sec(dt);                \
        ins_measurements_t mm;                 \
        memset(&mm, 0, sizeof(mm));            \
        mm.timestamp = (T);                    \
        set_imu(&mm, acc_body, gyr_body, dt);  \
        if ((SPEED_MPS) > 0.0f)                \
        {                                      \
            mm.speed.speed_mps  = (SPEED_MPS); \
            mm.speed.stddev_mps = 0.5f;        \
            mm.speed.is_valid   = true;        \
            mm.speed_delay_ms   = (DELAY_MS);  \
        }                                      \
        ins_update(&(F), &mm);                 \
    } while (0)

    /* --- speed: skipped while the history is still shallow, fused once
           it reaches back far enough --- */
    {
        ins_t         f;
        ins_time_us_t t = 0;
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_zupt_disable = true; /* a constant-velocity IMU reads as standstill */
        memset(&f, 0, sizeof(f));
        CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

        start_manual_filter(&f, &t, dt, (const double*)0);
        int i;
        for (i = 0; i < 5; ++i) { DELAY_STEP(f, t, 0.0f, 0); } /* 50 ms of history */
        CHECK_TRUE(f.is_initialized, "filter running");

        DELAY_STEP(f, t, 11.0f, deep_delay_ms);
        CHECK_TRUE(ins_get_diag(&f)->n_speed_seen == 1, "the delayed speed sample was seen");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 0, "but not fused without a history anchor");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_skipped == 1, "and counted as skipped");

        /* Same delay once the ring covers it: now it anchors and fuses. */
        for (i = 0; i < 100; ++i) { DELAY_STEP(f, t, 0.0f, 0); } /* 1 s of history */
        DELAY_STEP(f, t, 11.0f, deep_delay_ms);
        CHECK_TRUE(ins_get_diag(&f)->n_speed_used == 1,
                   "the same delay fuses once the history covers it");
        CHECK_TRUE(ins_get_diag(&f)->n_speed_skipped == 1, "and is not skipped a second time");
    }
#undef DELAY_STEP

    /* --- automotive course-over-ground yaw: same anchor, same rule ---
           A yaw correction has no diagnostic counter of its own, so the
           yaw estimate itself is the observable: with no anchor it must
           not move at all. */
#define COURSE_STEP(F, T, WITH_FIX, DELAY_MS)                                                      \
    do {                                                                                           \
        (T) += us_from_sec(dt);                                                                    \
        ins_measurements_t mm;                                                                     \
        memset(&mm, 0, sizeof(mm));                                                                \
        mm.timestamp = (T);                                                                        \
        set_imu(&mm, acc_body, gyr_body, dt);                                                      \
        if (WITH_FIX)                                                                              \
        {                                                                                          \
            mm.gnss_vel.is_valid   = true;                                                         \
            mm.gnss_vel.vel_ned[0] = vel_ned[0];                                                   \
            mm.gnss_vel.vel_ned[1] = vel_ned[1];                                                   \
            mm.gnss_vel.vel_ned[2] = vel_ned[2];                                                   \
            mm.gnss_vel.Qll_ned[0] = mm.gnss_vel.Qll_ned[4] = mm.gnss_vel.Qll_ned[8] = 0.09f;      \
            mm.gnss_delay_ms                                                         = (DELAY_MS); \
        }                                                                                          \
        ins_update(&(F), &mm);                                                                     \
    } while (0)

    {
        ins_t         f;
        ins_time_us_t t = 0;
        ins_options_t opt;
        fill_default_opt(&opt);
        opt.auto_zupt_disable           = true;
        opt.automotive_mode             = true;
        ins_init_t init_yaw             = init;
        init_yaw.rpy_init_stddev_rad[2] = DEG2RAD(20.0f); /* loose: a fusion would show */
        memset(&f, 0, sizeof(f));
        CHECK_TRUE(ins_init(&f, &init_yaw, &opt) == 0, "init ok");

        start_manual_filter(&f, &t, dt, (const double*)0);
        int i;
        for (i = 0; i < 5; ++i) { COURSE_STEP(f, t, false, 0); }

        float roll, pitch, yaw_before, yaw_after;
        ins_get_rpy(&f, &roll, &pitch, &yaw_before);
        COURSE_STEP(f, t, true, deep_delay_ms);
        ins_get_rpy(&f, &roll, &pitch, &yaw_after);
        CHECK_NEAR(RAD2DEG(yaw_after), RAD2DEG(yaw_before), 1e-3,
                   "no course-yaw fusion without a history anchor [deg]");

        /* Let the history fill, then repeat: the same delayed fix now
           anchors on the stored attitude and does move the yaw. */
        for (i = 0; i < 100; ++i) { COURSE_STEP(f, t, false, 0); }
        ins_get_rpy(&f, &roll, &pitch, &yaw_before);
        COURSE_STEP(f, t, true, deep_delay_ms);
        ins_get_rpy(&f, &roll, &pitch, &yaw_after);
        CHECK_TRUE(fabsf(ins_angle_diff(yaw_after, yaw_before)) > DEG2RAD(0.01f),
                   "the anchored delayed fix does correct the yaw");
    }
#undef COURSE_STEP
}

/* The bootstrap needs a fix that is concurrent with the IMU stream: a
 * fix arriving long after the newest buffered IMU sample would derive
 * the origin and the leveling from data that does not describe the same
 * instant (REQ-NAV-033).
 */
static void scenario_autoinit_stream_coherence(void)
{
    printf("\n=== Scenario: bootstrap refuses a fix incoherent with the IMU stream "
           "(REQ-NAV-033) ===\n");

    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);
    opt.auto_init               = true;
    opt.max_prediction_time_sec = 0.5f;

    ins_t f;
    memset(&f, 0, sizeof(f));
    CHECK_TRUE(ins_init(&f, &init, &opt) == 0, "init ok");

    float g_vec[3];
    {
        double lat, lon, h;
        ins_ecef_to_latlonh(init.x_ecef, &lat, &lon, &h);
        ins_gravity_ned((float)lat, (float)h, g_vec);
    }
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.0f, 0.0f};
    const float dt          = 0.01f;

    ins_time_us_t t = 0;
    int           i;
    for (i = 0; i < 200; ++i) /* 2 s of IMU only: the leveling window fills */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    CHECK_TRUE(!f.is_initialized, "not bootstrapped without a fix");

    /* A fix on its own (no IMU in the same epoch, so nothing refreshes the
       buffer), stamped well beyond max_prediction_time_sec after the last
       IMU sample. */
    {
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp            = t + us_from_sec(2.0f);
        m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
        m.gnss_pos.is_valid                                                   = true;
        ins_update(&f, &m);
        CHECK_TRUE(!f.is_initialized, "a fix long after the last IMU sample does not bootstrap");
    }

    /* The same fix, concurrent with the IMU stream, does. The IMU stream
       resumes after the 2 s hole first (the hole itself is a time jump,
       which re-anchors the clock and refills the leveling window). */
    t += us_from_sec(2.0f);
    for (i = 0; i < 300; ++i)
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        ins_update(&f, &m);
    }
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);
        m.gnss_pos.xyz_ecef[0] = init.x_ecef[0];
        m.gnss_pos.xyz_ecef[1] = init.x_ecef[1];
        m.gnss_pos.xyz_ecef[2] = init.x_ecef[2];
        m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
        m.gnss_pos.is_valid                                                   = true;
        ins_update(&f, &m);
        CHECK_TRUE(f.is_initialized, "a fix concurrent with the IMU stream bootstraps");
    }
}

static void scenario_predict_correct_equivalence(void)
{
    printf("\n=== Scenario: ins_update() == ins_predict_step()+ins_correct_step() ===\n");

    ins_t f1, f2;
    memset(&f1, 0, sizeof(f1));
    memset(&f2, 0, sizeof(f2));
    ins_init_t init;
    fill_default_init(&init, 0);
    ins_options_t opt;
    fill_default_opt(&opt);

    CHECK_TRUE(ins_init(&f1, &init, &opt) == 0, "f1 init");
    CHECK_TRUE(ins_init(&f2, &init, &opt) == 0, "f2 init");

    float g_vec[3];
    ins_gravity_ned((float)f1.latlonh[0], (float)f1.latlonh[2], g_vec);
    const float acc_body[3] = {0.0f, 0.0f, -g_vec[2]};
    const float gyr_body[3] = {0.0f, 0.05f, 0.0f}; /* small rotation exercises strapdown too */

    double truth_ecef[3];
    memcpy(truth_ecef, init.x_ecef, sizeof(truth_ecef));

    const float   dt = 0.01f;
    ins_time_us_t t  = 0;
    int           step;
    float         phi_check[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX];
    bool          phi_checked = false;
    for (step = 1; step <= 1500; ++step) /* 15 s, crosses several kalman/gnss cadences */
    {
        t += us_from_sec(dt);
        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp = t;
        set_imu(&m, acc_body, gyr_body, dt);

        if (step % 20 == 0) /* 5 Hz GNSS */
        {
            m.gnss_pos.xyz_ecef[0] = truth_ecef[0];
            m.gnss_pos.xyz_ecef[1] = truth_ecef[1];
            m.gnss_pos.xyz_ecef[2] = truth_ecef[2];
            m.gnss_pos.Qll_ned[0] = m.gnss_pos.Qll_ned[4] = m.gnss_pos.Qll_ned[8] = 1.0f;
            m.gnss_pos.is_valid                                                   = true;
        }

        ins_update(&f1, &m);

        float phi[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX];
        memset(phi, 0, sizeof(phi));
        const int status = ins_predict_step(&f2, &m, phi);
        ins_correct_step(&f2);

        if (!phi_checked && (status & INS_EPOCH_COV_PROPAGATED))
        {
            memcpy(phi_check, phi, sizeof(phi));
            phi_checked = true;
        }
    }

    CHECK_TRUE(phi_checked, "phi_out was filled at least once");
    if (phi_checked)
    {
        const int n = f2.n;
        /* pos += vel * dt : Phi[POS+i, VEL+i] == dt (col-major: row + col*n) */
        CHECK_NEAR(phi_check[(INS_IDX_POS + 0) + (INS_IDX_VEL + 0) * n], dt, 1e-9,
                   "phi pos/vel N coupling");
        CHECK_NEAR(phi_check[(INS_IDX_POS + 1) + (INS_IDX_VEL + 1) * n], dt, 1e-9,
                   "phi pos/vel E coupling");
        CHECK_NEAR(phi_check[(INS_IDX_POS + 2) + (INS_IDX_VEL + 2) * n], dt, 1e-9,
                   "phi pos/vel D coupling");
    }

    float v1[3], v2[3], p1[3], p2[3];
    ins_get_velocity_ned(&f1, v1);
    ins_get_velocity_ned(&f2, v2);
    ins_get_position_local(&f1, p1);
    ins_get_position_local(&f2, p2);
    CHECK_NEAR(v1[0], v2[0], 0.0, "velocity N matches");
    CHECK_NEAR(v1[1], v2[1], 0.0, "velocity E matches");
    CHECK_NEAR(v1[2], v2[2], 0.0, "velocity D matches");
    CHECK_NEAR(p1[0], p2[0], 0.0, "position N matches");
    CHECK_NEAR(p1[1], p2[1], 0.0, "position E matches");
    CHECK_NEAR(p1[2], p2[2], 0.0, "position D matches");

    float roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    ins_get_rpy(&f1, &roll1, &pitch1, &yaw1);
    ins_get_rpy(&f2, &roll2, &pitch2, &yaw2);
    CHECK_NEAR(roll1, roll2, 0.0, "roll matches");
    CHECK_NEAR(pitch1, pitch2, 0.0, "pitch matches");
    CHECK_NEAR(yaw1, yaw2, 0.0, "yaw matches");

    CHECK_TRUE(memcmp(f1.U, f2.U, sizeof(f1.U)) == 0, "covariance U factor matches");
    CHECK_TRUE(memcmp(f1.d, f2.d, sizeof(f1.d)) == 0, "covariance d factor matches");
    CHECK_TRUE(f1.diag.n_updates == f2.diag.n_updates, "n_updates matches");
    CHECK_TRUE(f1.diag.n_predict == f2.diag.n_predict, "n_predict matches");
}

int main(void)
{
    scenario_stationary();
    scenario_free_fall();
    scenario_yaw_rotation();
    scenario_covariance_grows();
    scenario_kalman_cadence_tolerance();
    scenario_zupt();
    scenario_gnss_position();
    scenario_mag_yaw();
    scenario_magnetic_model_from_position();
    scenario_mag_bias_estimation();
    scenario_zero_rotation_bias();
    scenario_gnss_moving_delayed();
    scenario_gnss_velocity_leverarm();
    scenario_gnss_leverarm_yaw_from_position();
    scenario_gnss_leverarm_yaw_from_velocity();
    scenario_leveling_at_yaw90();
    scenario_local_pos_lighthouse();
    scenario_local_pos_lighthouse_yaw();
    scenario_yaw_aiding();
    scenario_yaw_aiding_guards();
    scenario_yaw_input_range();
    scenario_autoinit_gnss();
    scenario_autoinit_local_pos();
    scenario_autoinit_mag_yaw();
    scenario_autoinit_mag_yaw_cached();
    scenario_gnss_init_dwell();
    scenario_gnss_mode_hysteresis();
    scenario_gnss_quality_exit_bias_carry();
    scenario_gnss_quality_exit_origin_carry();
    scenario_baro_height_source_selection();
    scenario_baro_height_fusion();
    scenario_baro_height_survives_reacquire();
    scenario_baro_height_reanchor_after_long_outage();
    scenario_baro_height_stale_sample();
    scenario_baro_height_vertical_gate_ignored();
    scenario_baro_height_gap_warning();
    scenario_initial_yaw_stddev_override();
    scenario_autoinit_yaw_priority();
    scenario_autoinit_yaw_zero_stddev_falls_through();
    scenario_autoinit_moving_rpy_stddev();
    scenario_autoinit_att_hint();
    scenario_att_hint_gyr_bias_prior_cap();
    scenario_speed_aiding();
    scenario_accessors_and_lifecycle();
    scenario_time_jump_handling();
    scenario_time_jump_unlimited_dr_recovery();
    scenario_startup_alignment_gate();
    scenario_automotive_gnss_yaw();
    scenario_gravity_override();
    scenario_gnss_local_pos_gating();
    scenario_mag_gating();
    scenario_autoinit_buffer_wraparound();
    scenario_default_mems_noise();
    scenario_beginner_option_defaults();
    scenario_beginner_init_defaults();
    scenario_auto_zupt();
    scenario_auto_zaru_vibration();
    scenario_static_variance_gate();
    scenario_bias_prior_check();
    scenario_nan_inf_inputs();
    scenario_deadreckoning_reacquire();
    scenario_deadreckoning_reacquire_far_from_origin();
    scenario_deadreckoning_freeze();
    scenario_coasting_window_still_fuses();
    scenario_frozen_no_fusion();
    scenario_reacquire_variance_inflation();
    scenario_deadreckoning_velocity_only_no_reacquire();
    scenario_gnss_pos_decimation();
    scenario_reacquire_att_hint();
    scenario_reacquire_yaw_unknown();
    scenario_init_validation();
    scenario_accessor_null_guards();
    scenario_reacquire_variants();
    scenario_delayed_no_anchor_and_gates();
    scenario_zupt_zaru_rate_limit();
    scenario_health_check_deinit();
    scenario_auto_reacquire_after_health_fail();
    scenario_time_jump_reset_reacquires();
    scenario_time_source_restart_recovers();
    scenario_fuse_rejects_bad_noise();
    scenario_origin_shift();
    scenario_degenerate_process_noise();
    scenario_set_world_model();
    scenario_corrupted_covariance();
    scenario_imu_calibration();
    scenario_gnss_cov_conditioning();
    scenario_gnss_cov_conditioning_gates_unscaled();
    scenario_gnss_rate_limit();
    scenario_gnss_cov_cap();
    scenario_gnss_acc_envelope();
    scenario_gnss_vel_noise_acc();
    scenario_gnss_manoeuvre_window();
    scenario_gnss_vel_noise_leverarm();
    scenario_gnss_pos_height_scale();
    scenario_mag_calibration();
    scenario_overconfidence_watchdog();
    scenario_chi2_disable();
    scenario_chi2_reject_alpha();
    scenario_mag_disturbance_diagnostics();
    scenario_api_and_config_guards();
    scenario_bias_diagnostics();
    scenario_delayed_without_history();
    scenario_autoinit_stream_coherence();
    scenario_predict_correct_equivalence();
    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
