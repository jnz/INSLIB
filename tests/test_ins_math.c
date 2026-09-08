/** @file test_ins_math.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Minimal sanity tests for geodetic_toolbox.c */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "geodetic_toolbox.h"
#include "magnetic_model.h"
#include "wmm_test_vectors.h"
#include "linalg.h"

static int failures = 0;

#define CHECK_NEAR(a, b, tol, name)                                                      \
    do {                                                                                 \
        double _d = fabs((double)(a) - (double)(b));                                     \
        if (_d > (tol))                                                                  \
        {                                                                                \
            printf("FAIL %s: %g vs %g (diff %g)\n", name, (double)(a), (double)(b), _d); \
            failures++;                                                                  \
        }                                                                                \
        else { printf("ok   %s (diff %g)\n", name, _d); }                                \
    } while (0)

/* Boolean assertions go through their own macro rather than through
   CHECK_NEAR: a bool returning call cast to double trips
   -Wbad-function-cast, which this build treats as noise worth avoiding. */
#define CHECK_TRUE(cond, name)              \
    do {                                    \
        if (!(cond))                        \
        {                                   \
            printf("FAIL %s\n", name);      \
            failures++;                     \
        }                                   \
        else { printf("ok   %s\n", name); } \
    } while (0)

static void test_quat_identity(void)
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float R[9];
    ins_quat_to_rotmat(q, R);
    /* Identity: diagonal = 1, off-diag = 0 */
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
        {
            float expected = (r == c) ? 1.0f : 0.0f;
            CHECK_NEAR(MAT_ELEM(R, r, c, 3, 3), expected, 1e-7, "quat_identity_R");
        }
}

static void test_rpy_roundtrip(void)
{
    const float roll = 0.3f, pitch = -0.2f, yaw = 1.1f;
    float       q[4];
    float       R[9];
    float       r2, p2, y2;
    ins_quat_from_rpy(roll, pitch, yaw, q);
    ins_quat_to_rotmat(q, R);
    ins_rotmat_to_rpy(R, &r2, &p2, &y2);
    CHECK_NEAR(r2, roll, 1e-5, "rpy_roll");
    CHECK_NEAR(p2, pitch, 1e-5, "rpy_pitch");
    CHECK_NEAR(y2, yaw, 1e-5, "rpy_yaw");
}

static void test_ecef_roundtrip(void)
{
    /* Stuttgart-ish */
    const double lat = 48.783 * M_PI / 180.0;
    const double lon = 9.181 * M_PI / 180.0;
    const double h   = 300.0;
    double       xyz[3];
    ins_latlonh_to_ecef(lat, lon, h, xyz);
    printf("Stuttgart ECEF: %.2f %.2f %.2f\n", xyz[0], xyz[1], xyz[2]);
    double lat2, lon2, h2;
    ins_ecef_to_latlonh(xyz, &lat2, &lon2, &h2);
    CHECK_NEAR(lat2, lat, 1e-10, "ecef_lat");
    CHECK_NEAR(lon2, lon, 1e-10, "ecef_lon");
    CHECK_NEAR(h2, h, 1e-5, "ecef_h");
}

static void test_rotation_rate_small(void)
{
    /* With omega = 0, q_new must equal q. */
    float q[4]     = {0.707107f, 0.0f, 0.707107f, 0.0f};
    float omega[3] = {0, 0, 0};
    float qn[4];
    ins_quat_rotate(q, omega, 0.01f, qn);
    CHECK_NEAR(qn[0], q[0], 1e-6, "rotate_zero_w");
    CHECK_NEAR(qn[1], q[1], 1e-6, "rotate_zero_x");
    CHECK_NEAR(qn[2], q[2], 1e-6, "rotate_zero_y");
    CHECK_NEAR(qn[3], q[3], 1e-6, "rotate_zero_z");
}

static void test_rotation_rate_known(void)
{
    /* Starting at identity, rotating around Z by 90deg should yield
       q = [cos(45), 0, 0, sin(45)] = [0.707, 0, 0, 0.707] */
    float q[4]     = {1, 0, 0, 0};
    float omega[3] = {0, 0, (float)(M_PI / 2.0)}; /* 90 deg/s */
    float qn[4];
    ins_quat_rotate(q, omega, 1.0f, qn);
    CHECK_NEAR(qn[0], (float)cos(M_PI / 4), 1e-5, "rotz90_w");
    CHECK_NEAR(qn[1], 0.0f, 1e-5, "rotz90_x");
    CHECK_NEAR(qn[2], 0.0f, 1e-5, "rotz90_y");
    CHECK_NEAR(qn[3], (float)sin(M_PI / 4), 1e-5, "rotz90_z");
}

static void test_transport_rate_pole(void)
{
    /* The vertical (azimuth) transport rate carries tan(lat), which
       diverges at the poles. With any east velocity, an unclamped
       tan(pi/2) ~ 1.6e16 yields ~2.5e11 rad/s here. That would instantly
       wreck the strapdown attitude integration. The pole floor must keep
       every component finite and bounded. */
    const float vel[3] = {50.0f, 100.0f, 0.0f}; /* 100 m/s eastward */
    float       omega[3], wie[3], wen[3];
    ins_calc_omega_n_in(M_PI / 2.0, 0.0, vel, omega, wie, wen);
    for (int i = 0; i < 3; ++i)
    {
        int finite = isfinite(omega[i]) && isfinite(wie[i]) && isfinite(wen[i]);
        if (!finite)
        {
            printf("FAIL pole_finite[%d]\n", i);
            failures++;
        }
        else { printf("ok   pole_finite[%d]\n", i); }
    }
    /* Bounded, not just finite: |cos| floored at 1e-4 caps tan at ~1e4, so
       the azimuth rate stays sub-rad/s instead of ~1e11. */
    if (fabs((double)omega[2]) > 1.0)
    {
        printf("FAIL pole_bounded_az: %g\n", (double)omega[2]);
        failures++;
    }
    else { printf("ok   pole_bounded_az (%g)\n", (double)omega[2]); }

    /* The clamp must not perturb ordinary latitudes: at 45 deg tan(lat)=1,
       so the azimuth transport rate is exactly -v_E / (Re + h). */
    ins_calc_omega_n_in(M_PI / 4.0, 0.0, vel, omega, wie, wen);
    const double Re = INS_WGS84_A / sqrt(1.0 - INS_WGS84_E2 * 0.5);
    CHECK_NEAR(wen[2], (float)(-100.0 / Re), 1e-9, "midlat_az_unchanged");
}

static void test_cross_matrix(void)
{
    const float v[3] = {1, 2, 3};
    const float w[3] = {4, 5, 6};
    float       M[9];
    ins_cross_matrix(v, M);
    /* M * w should equal v x w */
    float Mw[3] = {0, 0, 0};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) Mw[r] += MAT_ELEM(M, r, c, 3, 3) * w[c];
    float vxw[3];
    ins_cross(v, w, vxw);
    CHECK_NEAR(Mw[0], vxw[0], 1e-6, "cross_mat_x");
    CHECK_NEAR(Mw[1], vxw[1], 1e-6, "cross_mat_y");
    CHECK_NEAR(Mw[2], vxw[2], 1e-6, "cross_mat_z");
}

static void test_matrix_to_quat_roundtrip(void)
{
    /* R -> quat -> R must reproduce the matrix (quat sign is irrelevant). */
    const float roll = 0.4f, pitch = -0.7f, yaw = 2.3f;
    float       q[4], R[9], q2[4], R2[9];
    ins_quat_from_rpy(roll, pitch, yaw, q);
    ins_quat_to_rotmat(q, R);
    ins_matrix_to_quat(R, q2);
    ins_quat_to_rotmat(q2, R2);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            CHECK_NEAR(MAT_ELEM(R2, r, c, 3, 3), MAT_ELEM(R, r, c, 3, 3), 1e-5, "matrix_to_quat_R");

    /* Exercise every pivot branch of the trace-based selection:
       identity  -> tr > 0
       180 x     -> r00 largest
       180 y     -> r11 largest
       180 z     -> r22 largest (else)                                  */
    const float pivots[4][9] = {
        {1, 0, 0, 0, 1, 0, 0, 0, 1},   /* identity, tr = 3   */
        {1, 0, 0, 0, -1, 0, 0, 0, -1}, /* Rx(180), r00 pivot */
        {-1, 0, 0, 0, 1, 0, 0, 0, -1}, /* Ry(180), r11 pivot */
        {-1, 0, 0, 0, -1, 0, 0, 0, 1}  /* Rz(180), r22 pivot */
    };
    for (int k = 0; k < 4; ++k)
    {
        float qk[4], Rk2[9];
        ins_matrix_to_quat(pivots[k], qk);
        ins_quat_to_rotmat(qk, Rk2);
        for (int i = 0; i < 9; ++i) CHECK_NEAR(Rk2[i], pivots[k][i], 1e-5, "matrix_to_quat_pivot");
    }
}

static void test_quat_normalize_degenerate(void)
{
    /* A (near-)zero quaternion must fall back to identity, not NaN. */
    float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ins_quat_normalize(q);
    CHECK_NEAR(q[0], 1.0f, 1e-7, "normalize_zero_w");
    CHECK_NEAR(q[1], 0.0f, 1e-7, "normalize_zero_x");
    CHECK_NEAR(q[2], 0.0f, 1e-7, "normalize_zero_y");
    CHECK_NEAR(q[3], 0.0f, 1e-7, "normalize_zero_z");
}

static void test_rpy_gimbal_lock(void)
{
    /* pitch = +90 deg -> the gimbal-lock branch of rotmat_to_rpy: yaw is
       pinned to 0 and roll absorbs the coupled rotation. */
    float q[4], R[9], roll, pitch, yaw;
    ins_quat_from_rpy(0.0f, (float)(M_PI / 2.0), 0.0f, q);
    ins_quat_to_rotmat(q, R);
    ins_rotmat_to_rpy(R, &roll, &pitch, &yaw);
    CHECK_NEAR(pitch, (float)(M_PI / 2.0), 1e-4, "gimbal_pitch");
    CHECK_NEAR(yaw, 0.0f, 1e-6, "gimbal_yaw");
    CHECK_NEAR(roll, 0.0f, 1e-4, "gimbal_roll");

    /* pitch = -90 deg -> the other gimbal-lock side (sp < -0.9999). */
    ins_quat_from_rpy(0.0f, -(float)(M_PI / 2.0), 0.0f, q);
    ins_quat_to_rotmat(q, R);
    ins_rotmat_to_rpy(R, &roll, &pitch, &yaw);
    CHECK_NEAR(pitch, -(float)(M_PI / 2.0), 1e-4, "gimbal_neg_pitch");
    CHECK_NEAR(yaw, 0.0f, 1e-6, "gimbal_neg_yaw");
}

static void test_quat_multiply_and_invert(void)
{
    const float roll = 0.3f, pitch = -0.2f, yaw = 1.1f;
    float       q[4], qi[4], prod[4];
    ins_quat_from_rpy(roll, pitch, yaw, q);
    ins_quat_invert(q, qi);

    /* q * q^-1 == identity [1,0,0,0]. */
    ins_quat_multiply(q, qi, prod);
    CHECK_NEAR(prod[0], 1.0f, 1e-6, "q_times_inv_w");
    CHECK_NEAR(prod[1], 0.0f, 1e-6, "q_times_inv_x");
    CHECK_NEAR(prod[2], 0.0f, 1e-6, "q_times_inv_y");
    CHECK_NEAR(prod[3], 0.0f, 1e-6, "q_times_inv_z");

    /* Multiplying by identity returns q unchanged. */
    const float id[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    ins_quat_multiply(q, id, prod);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(prod[i], q[i], 1e-6, "q_times_id");
}

static void test_quat_to_axis_angle(void)
{
    /* 90 deg about +Z -> axis [0,0,1], angle pi/2. */
    float q[4];
    ins_quat_from_rpy(0.0f, 0.0f, (float)(M_PI / 2.0), q);
    float axis[3], angle;
    ins_quat_to_axis_angle(q, axis, &angle);
    CHECK_NEAR(angle, (float)(M_PI / 2.0), 1e-5, "axisangle_angle");
    CHECK_NEAR(axis[0], 0.0f, 1e-5, "axisangle_axis_x");
    CHECK_NEAR(axis[1], 0.0f, 1e-5, "axisangle_axis_y");
    CHECK_NEAR(axis[2], 1.0f, 1e-5, "axisangle_axis_z");

    /* Identity -> angle 0, default axis. */
    const float qid[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    ins_quat_to_axis_angle(qid, axis, &angle);
    CHECK_NEAR(angle, 0.0f, 1e-6, "axisangle_identity");
}

static void test_angle_diff(void)
{
    CHECK_NEAR(ins_angle_diff(0.1f, -0.1f), 0.2f, 1e-6, "angdiff_simple");
    /* Wrap: 3.0 - (-3.0) = 6.0 -> 6.0 - 2*pi ~ -0.2832. */
    CHECK_NEAR(ins_angle_diff(3.0f, -3.0f), 6.0f - 2.0f * (float)M_PI, 1e-5, "angdiff_wrap");
    /* Negative wrap: -3.0 - 3.0 = -6.0 -> -6.0 + 2*pi (hits the d<0 branch). */
    CHECK_NEAR(ins_angle_diff(-3.0f, 3.0f), -6.0f + 2.0f * (float)M_PI, 1e-5, "angdiff_wrap_neg");
    /* Symmetry / antisymmetry around pi. */
    CHECK_NEAR(ins_angle_diff((float)M_PI + 0.1f, 0.0f), -(float)M_PI + 0.1f, 1e-5, "angdiff_pi");
}

/* ins_wrap_pi_bounded: the single-step wrap used on residuals that are
   known to be within one turn of the interval. Both branches plus the
   pass-through must be exercised -- an unwrapped +pi+eps residual would
   otherwise reach a fusion as a nearly full turn. */
static void test_wrap_pi_bounded(void)
{
    const float pi = (float)M_PI;
    CHECK_NEAR(ins_wrap_pi_bounded(0.5f), 0.5f, 1e-6, "wrap_inside_unchanged");
    CHECK_NEAR(ins_wrap_pi_bounded(pi + 0.25f), -pi + 0.25f, 1e-5, "wrap_above_pi");
    CHECK_NEAR(ins_wrap_pi_bounded(-pi - 0.25f), pi - 0.25f, 1e-5, "wrap_below_minus_pi");
    /* The interval edges themselves are left alone (the comparisons are
       strict), so +/-pi stays +/-pi rather than flipping sign. */
    CHECK_NEAR(ins_wrap_pi_bounded(pi), pi, 1e-6, "wrap_at_plus_pi");
    CHECK_NEAR(ins_wrap_pi_bounded(-pi), -pi, 1e-6, "wrap_at_minus_pi");
    /* Same result as the unbounded form for inputs both can handle. */
    CHECK_NEAR(ins_wrap_pi_bounded(pi + 1.0f), ins_angle_diff(pi + 1.0f, 0.0f), 1e-5,
               "wrap_matches_angle_diff");
}

/* ISA barometric conversion and the plausibility gate in front of it:
   a pressure that is out of range or not a number must be refused
   before it can turn into an altitude. */
static void test_isa_pressure(void)
{
    /* Sea level pressure is the datum, so it converts to 0 m. */
    CHECK_NEAR(ins_isa_altitude_from_pressure(INS_ISA_P0_PA), 0.0f, 1e-3, "isa_sea_level_zero");
    /* Round trip through the inverse formula. */
    const float h_ref = 1500.0f;
    const float p_ref = INS_ISA_P0_PA * powf(1.0f - h_ref / INS_ISA_SCALE_M, 1.0f / INS_ISA_EXP);
    CHECK_NEAR(ins_isa_altitude_from_pressure(p_ref), h_ref, 1e-2, "isa_roundtrip_1500m");
    /* Lower pressure means higher altitude. */
    CHECK_TRUE(ins_isa_altitude_from_pressure(p_ref) > 0.0f, "isa_monotonic");

    CHECK_TRUE(ins_isa_pressure_plausible(INS_ISA_P0_PA), "isa_plausible_sea_level");
    CHECK_TRUE(ins_isa_pressure_plausible(INS_ISA_PRESSURE_MIN_PA), "isa_plausible_min");
    CHECK_TRUE(ins_isa_pressure_plausible(INS_ISA_PRESSURE_MAX_PA), "isa_plausible_max");
    CHECK_TRUE(!ins_isa_pressure_plausible(INS_ISA_PRESSURE_MIN_PA - 1.0f), "isa_reject_below_min");
    CHECK_TRUE(!ins_isa_pressure_plausible(INS_ISA_PRESSURE_MAX_PA + 1.0f), "isa_reject_above_max");
    CHECK_TRUE(!ins_isa_pressure_plausible(0.0f), "isa_reject_zero");
    CHECK_TRUE(!ins_isa_pressure_plausible((float)NAN), "isa_reject_nan");
    CHECK_TRUE(!ins_isa_pressure_plausible((float)INFINITY), "isa_reject_inf");

    /* The finite-vector guard shares the same "drop it at the boundary"
       job on the vector inputs. */
    const float ok[3]  = {1.0f, 2.0f, 3.0f};
    float       bad[3] = {1.0f, (float)NAN, 3.0f};
    CHECK_TRUE(ins_vec3_finite(ok), "vec3_finite_ok");
    CHECK_TRUE(!ins_vec3_finite(bad), "vec3_finite_nan");
    bad[1] = (float)INFINITY;
    CHECK_TRUE(!ins_vec3_finite(bad), "vec3_finite_inf");
}

/* The WMM look-up clamps latitude and wraps longitude before indexing
   its table. Without that, a caller handing over a position at the pole
   or a longitude expressed in [0, 360) would index past the grid. Every
   query must stay finite and land on the same cell as its in-range
   equivalent. */
static void test_wmm_grid_edges(void)
{
    /* Longitude wrapping: the same meridian expressed three ways. */
    const float d_ref  = magnetic_declination_deg(48.137f, -170.0f, 2027.5f);
    const float d_plus = magnetic_declination_deg(48.137f, 190.0f, 2027.5f);  /* +360 */
    const float d_more = magnetic_declination_deg(48.137f, -530.0f, 2027.5f); /* -360 */
    CHECK_NEAR(d_plus, d_ref, 1e-3, "wmm_lon_wrap_plus360");
    CHECK_NEAR(d_more, d_ref, 1e-3, "wmm_lon_wrap_minus360");

    /* Exactly +180 deg is the wrapped-out edge and must not index past
       the last column. */
    const float d_180  = magnetic_declination_deg(10.0f, 180.0f, 2027.5f);
    const float d_m180 = magnetic_declination_deg(10.0f, -180.0f, 2027.5f);
    CHECK_TRUE(isfinite(d_180), "wmm_lon_plus180_finite");
    CHECK_TRUE(isfinite(d_m180), "wmm_lon_minus180_finite");

    /* One ULP below -180 deg: the fmodf() wrap folds this back to
       exactly +180.0f (the clamp's condition IS reachable, not dead
       code), so the post-clamp re-check must still catch it. */
    const float lon_wrap_edge = nextafterf(-180.0f, -INFINITY);
    CHECK_TRUE(isfinite(magnetic_declination_deg(10.0f, lon_wrap_edge, 2027.5f)),
               "wmm_lon_wrap_edge_finite");

    /* Latitude clamping at both poles, for all three tables. */
    const float lat_hi[3] = {90.0f, 95.0f, 89.9999f};
    const float lat_lo[3] = {-90.0f, -95.0f, -89.9999f};
    for (int i = 0; i < 3; ++i)
    {
        char name[48];
        snprintf(name, sizeof(name), "wmm_pole_hi_decl[%d]", i);
        CHECK_TRUE(isfinite(magnetic_declination_deg(lat_hi[i], 11.0f, 2027.5f)), name);
        snprintf(name, sizeof(name), "wmm_pole_lo_decl[%d]", i);
        CHECK_TRUE(isfinite(magnetic_declination_deg(lat_lo[i], 11.0f, 2027.5f)), name);
        snprintf(name, sizeof(name), "wmm_pole_incl[%d]", i);
        CHECK_TRUE(isfinite(magnetic_inclination_deg(lat_hi[i], 11.0f)), name);
        snprintf(name, sizeof(name), "wmm_pole_field[%d]", i);
        CHECK_TRUE(magnetic_field_strength_uT(lat_lo[i], 11.0f) > 0.0f, name);
    }

    /* An out-of-range latitude lands on the same cell as the clamp
       target, so the two queries must agree. */
    CHECK_NEAR(magnetic_inclination_deg(120.0f, 11.0f), magnetic_inclination_deg(89.999f, 11.0f),
               1e-2, "wmm_lat_clamp_high_matches");
    CHECK_NEAR(magnetic_inclination_deg(-120.0f, 11.0f), magnetic_inclination_deg(-90.0f, 11.0f),
               1e-2, "wmm_lat_clamp_low_matches");

    /* Declination is interpolated linearly between the two tabulated
       epochs and extrapolated with the same slope outside them. Three
       equally spaced years, deliberately far outside the tabulated span
       in both directions, must therefore give equally spaced values --
       a property that holds for any epoch pair, so it does not have to
       name one. */
    const float d_a = magnetic_declination_deg(48.137f, 11.575f, 2000.0f);
    const float d_b = magnetic_declination_deg(48.137f, 11.575f, 2050.0f);
    const float d_c = magnetic_declination_deg(48.137f, 11.575f, 2100.0f);
    CHECK_TRUE(isfinite(d_a) && isfinite(d_c), "wmm_epoch_extrapolation_finite");
    CHECK_NEAR(d_c - d_b, d_b - d_a, 1e-2, "wmm_epoch_linear_in_time");

    /* The NED assembly must survive the same edge inputs. */
    float b[3];
    magnetic_field_ned_uT(90.0f, 400.0f, 2027.5f, b);
    CHECK_TRUE(isfinite(b[0]) && isfinite(b[1]) && isfinite(b[2]), "wmm_ned_pole_finite");
}

static void test_gravity(void)
{
    float g[3];
    /* At equator, sea level */
    ins_gravity_ned(0.0f, 0.0f, g);
    printf("gravity at equator: %.4f\n", g[2]);
    CHECK_NEAR(g[0], 0.0f, 1e-6, "grav_eq_x");
    CHECK_NEAR(g[1], 0.0f, 1e-6, "grav_eq_y");
    CHECK_NEAR(g[2], 9.7803f, 5e-3, "grav_eq_z");
    /* At pole, sea level (should be ~9.832) */
    ins_gravity_ned((float)M_PI / 2, 0.0f, g);
    printf("gravity at pole: %.4f\n", g[2]);
    CHECK_NEAR(g[2], 9.8322f, 5e-3, "grav_pole_z");
    /* 45 deg lat, 1000 m height: exercises the free-air term, independently
       computed from the rigorous (non-polynomial) WGS84 Somigliana formula
       plus the DMA TR8350.2 eq. 4-3 height reduction (not derived from this
       file's polynomial approximation), so this catches a broken height or
       latitude term that the sea-level-only cases above cannot. */
    ins_gravity_ned((float)(45.0 * M_PI / 180.0), 1000.0f, g);
    printf("gravity at 45 deg, 1000 m: %.6f\n", g[2]);
    CHECK_NEAR(g[2], 9.803113f, 1e-3, "grav_45deg_1000m_z");
}

/* World Magnetic Model: the table interpolation must reproduce the exact
   pygeomag reference (wmm_test_vectors.h) to within the grid tolerance,
   and the assembled NED reference field must be self-consistent
   (horizontal angle = declination, magnitude = total field). */
static void test_wmm_model(void)
{
    for (int i = 0; i < WMM_TEST_VECTOR_COUNT; ++i)
    {
        const wmm_test_vector_t t   = WMM_TEST_VECTORS[i];
        const float             got = magnetic_declination_deg(t.lat, t.lon, t.year);
        /* Near-pole vectors (|lat| ~ 90) are geometrically degenerate.
           Tolerance widens to 0.6 deg there. */
        const double tol = (fabs((double)t.lat) > 88.0) ? 0.6 : 0.15;
        char         name[48];
        snprintf(name, sizeof(name), "wmm_decl[%d]", i);
        CHECK_NEAR(got, t.expected_decl, tol, name);
    }

    /* NED reference field at a mid-latitude site (Munich). */
    float b[3];
    magnetic_field_ned_uT(48.137f, 11.575f, 2027.5f, b);
    const float decl     = magnetic_declination_deg(48.137f, 11.575f, 2027.5f);
    const float strength = magnetic_field_strength_uT(48.137f, 11.575f);
    const float incl     = magnetic_inclination_deg(48.137f, 11.575f);
    const float horiz    = sqrtf(b[0] * b[0] + b[1] * b[1]);
    const float total    = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    CHECK_NEAR(atan2f(b[1], b[0]) * 180.0f / (float)M_PI, decl, 1e-3, "wmm_ned_declination");
    CHECK_NEAR(total, strength, 1e-2, "wmm_ned_total");
    CHECK_NEAR(atan2f(b[2], horiz) * 180.0f / (float)M_PI, incl, 1e-3, "wmm_ned_inclination");
    CHECK_TRUE(b[0] > 0.0f, "wmm_ned_north_positive"); /* N hemi */
    CHECK_TRUE(b[2] > 0.0f, "wmm_ned_down_positive");  /* dip down */
}

int main(void)
{
    test_quat_identity();
    test_rpy_roundtrip();
    test_ecef_roundtrip();
    test_rotation_rate_small();
    test_rotation_rate_known();
    test_transport_rate_pole();
    test_cross_matrix();
    test_matrix_to_quat_roundtrip();
    test_quat_normalize_degenerate();
    test_rpy_gimbal_lock();
    test_quat_multiply_and_invert();
    test_quat_to_axis_angle();
    test_angle_diff();
    test_wrap_pi_bounded();
    test_isa_pressure();
    test_gravity();
    test_wmm_model();
    test_wmm_grid_edges();
    printf("\n%d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
