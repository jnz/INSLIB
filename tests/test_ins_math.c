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

/* REQ-SYS-021: the local tangent-plane mapping evaluates its curvature radii
 * in single precision. Checked against the same mapping done entirely in
 * double here in the test, over the displacements and latitudes the
 * requirement puts a number on, plus the two properties that have to survive
 * regardless: the pair stays mutually inverse, and the pole stays finite. */
static void test_dned_dlatlonh_precision(void)
{
    static const double lats_deg[] = {0.0, 15.0, 45.0, -45.0, 60.0, 85.0, -85.0};
    static const double dists_m[]  = {1.0, 100.0, 10000.0, 100000.0};
    static const double heights[]  = {0.0, 500.0, 12000.0};

    double worst_rel = 0.0, worst_roundtrip_rel = 0.0;
    int    il, id, ih;

    for (il = 0; il < (int)(sizeof(lats_deg) / sizeof(lats_deg[0])); ++il)
    {
        for (id = 0; id < (int)(sizeof(dists_m) / sizeof(dists_m[0])); ++id)
        {
            for (ih = 0; ih < (int)(sizeof(heights) / sizeof(heights[0])); ++ih)
            {
                const double lat = lats_deg[il] * M_PI / 180.0;
                const double h   = heights[ih];
                const double d   = dists_m[id];

                /* Reference: the identical formula, all double. */
                const double sl         = sin(lat);
                const double cl         = cos(lat);
                const double denom      = 1.0 - INS_WGS84_E2 * sl * sl;
                const double sqrt_denom = sqrt(denom);
                const double Rn         = INS_WGS84_A * (1.0 - INS_WGS84_E2) / (denom * sqrt_denom);
                const double Re         = INS_WGS84_A / sqrt_denom;

                /* A displacement of d spread over all three axes. */
                const float  dned[3]  = {(float)(d * 0.6), (float)(d * 0.8), (float)(-d * 0.3)};
                const double ref_dlat = (double)dned[0] / (Rn + h);
                const double ref_dlon = (double)dned[1] / ((Re + h) * cl);

                double dllh[3];
                ins_dned_to_dlatlonh(dned, lat, h, dllh);

                const double rel_lat = fabs(dllh[0] - ref_dlat) / fabs(ref_dlat);
                const double rel_lon = fabs(dllh[1] - ref_dlon) / fabs(ref_dlon);
                if (rel_lat > worst_rel) { worst_rel = rel_lat; }
                if (rel_lon > worst_rel) { worst_rel = rel_lon; }

                /* And back: what went in has to come out. */
                float back[3];
                ins_dlatlonh_to_dned(dllh, lat, h, back);
                int k;
                for (k = 0; k < 3; ++k)
                {
                    const double rel = fabs((double)back[k] - (double)dned[k]) / d;
                    if (rel > worst_roundtrip_rel) { worst_roundtrip_rel = rel; }
                }
            }
        }
    }

    printf("dned<->dlatlonh: worst %.3g relative vs double, round trip %.3g relative\n", worst_rel,
           worst_roundtrip_rel);
    /* The east term divides by cos(lat), whose error is the rounded latitude
       amplified by tan(lat), so the bound has to cover 85 degrees. The round
       trip is tighter: both directions take the same cosine. */
    CHECK_TRUE(worst_rel < 1e-5, "within 1e-5 of the double mapping");
    CHECK_TRUE(worst_roundtrip_rel < 1e-6, "mutually inverse to single precision");

    /* The pole: cos(lat) is floored, so the longitude term stays finite and
       keeps its sign instead of dividing by a rounded-to-zero cosine. */
    {
        static const double polar[] = {89.0, 89.999, 90.0, -90.0};
        int                 k;
        int                 bad = 0;
        for (k = 0; k < (int)(sizeof(polar) / sizeof(polar[0])); ++k)
        {
            const double lat     = polar[k] * M_PI / 180.0;
            const float  dned[3] = {10.0f, 10.0f, 1.0f};
            double       dllh[3];
            float        back[3];

            ins_dned_to_dlatlonh(dned, lat, 0.0, dllh);
            if (!isfinite(dllh[0]) || !isfinite(dllh[1]) || !isfinite(dllh[2])) { bad++; }
            if (dllh[1] <= 0.0) { bad++; } /* east of here is still east */

            ins_dlatlonh_to_dned(dllh, lat, 0.0, back);
            if (fabs((double)back[1] - 10.0) > 1e-3) { bad++; }
        }
        CHECK_TRUE(bad == 0, "poles stay finite, signed and invertible");
    }
}

/* REQ-SYS-020: INS_WGS84_B and INS_WGS84_EP2 are stored as literals so the
 * closed-form conversion needs no square root to set up, which makes them a
 * second spelling of INS_WGS84_A and INS_WGS84_E2 that can drift away from
 * them. This is the consistency check geodetic_toolbox.c cannot hold as a
 * _Static_assert, because a floating point comparison is not an integer
 * constant expression and clang refuses one there. The tolerances are a few
 * ulp wide: what this catches is an ellipsoid changed in one place only, and
 * that moves both constants by kilometres. */
static void test_wgs84_constants_consistent(void)
{
    CHECK_NEAR(INS_WGS84_B, INS_WGS84_A * sqrt(1.0 - INS_WGS84_E2), 1e-6, "wgs84_b_matches_a_e2");
    CHECK_NEAR(INS_WGS84_EP2, INS_WGS84_E2 / (1.0 - INS_WGS84_E2), 1e-15, "wgs84_ep2_matches_e2");
}

/* REQ-SYS-020: the closed-form ECEF -> geodetic conversion against the exact
 * forward transform. ins_latlonh_to_ecef() is itself closed form and exact, so
 * the round-trip error IS the conversion error, with no iterative reference to
 * argue about. Checked as an aggregate worst case over the grid rather than
 * per point, which would bury the log. */
static void test_ecef_to_latlonh_closed_form(void)
{
    /* The envelope the requirement puts a number on. */
    static const double heights[] = {-1000.0, 0.0, 300.0, 12000.0, 30000.0};
    static const double lons[]    = {0.0, 1.7, -2.9};

    double worst_lat_mm = 0.0, worst_lon_mm = 0.0, worst_h_mm = 0.0;
    int    finite_fail = 0, range_fail = 0;
    int    ilat, ih, ilon;

    for (ilat = -90; ilat <= 90; ++ilat)
    {
        for (ih = 0; ih < (int)(sizeof(heights) / sizeof(heights[0])); ++ih)
        {
            for (ilon = 0; ilon < (int)(sizeof(lons) / sizeof(lons[0])); ++ilon)
            {
                const double lat = (double)ilat * M_PI / 180.0;
                const double lon = lons[ilon];
                double       xyz[3], lat2, lon2, h2;

                ins_latlonh_to_ecef(lat, lon, heights[ih], xyz);
                ins_ecef_to_latlonh(xyz, &lat2, &lon2, &h2);

                if (!isfinite(lat2) || !isfinite(lon2) || !isfinite(h2)) { finite_fail++; }
                if (fabs(lat2) > M_PI / 2.0 + 1e-12 || fabs(lon2) > M_PI + 1e-12) { range_fail++; }

                /* Metres at the surface, so latitude and longitude errors are
                   comparable to the height error. Longitude is skipped at the
                   poles, where it carries no information. */
                const double dlat_mm = fabs(lat2 - lat) * INS_WGS84_A * 1000.0;
                const double dh_mm   = fabs(h2 - heights[ih]) * 1000.0;
                if (dlat_mm > worst_lat_mm) { worst_lat_mm = dlat_mm; }
                if (dh_mm > worst_h_mm) { worst_h_mm = dh_mm; }
                if (ilat > -89 && ilat < 89)
                {
                    const double dlon_mm = fabs(lon2 - lon) * INS_WGS84_A * cos(lat) * 1000.0;
                    if (dlon_mm > worst_lon_mm) { worst_lon_mm = dlon_mm; }
                }
            }
        }
    }

    printf("closed-form ECEF->LLH worst case: lat %.6f mm, lon %.6f mm, h %.6f mm\n", worst_lat_mm,
           worst_lon_mm, worst_h_mm);
    CHECK_TRUE(finite_fail == 0, "ecef_llh_finite");
    CHECK_TRUE(range_fail == 0, "ecef_llh_in_range");
    CHECK_TRUE(worst_lat_mm < 1.0, "ecef_llh_lat_within_1mm");
    CHECK_TRUE(worst_lon_mm < 1.0, "ecef_llh_lon_within_1mm");
    CHECK_TRUE(worst_h_mm < 1.0, "ecef_llh_h_within_1mm");

    /* The pole is the case the height formula has to switch branches for:
       p/cos(lat) is 0/0 there. */
    {
        double xyz[3] = {0.0, 0.0, INS_WGS84_B + 100.0};
        double lat, lon, h;
        ins_ecef_to_latlonh(xyz, &lat, &lon, &h);
        CHECK_NEAR(lat, M_PI / 2.0, 1e-12, "ecef_llh_north_pole_lat");
        CHECK_NEAR(h, 100.0, 1e-6, "ecef_llh_north_pole_h");
        xyz[2] = -(INS_WGS84_B + 100.0);
        ins_ecef_to_latlonh(xyz, &lat, &lon, &h);
        CHECK_NEAR(lat, -M_PI / 2.0, 1e-12, "ecef_llh_south_pole_lat");
        CHECK_NEAR(h, 100.0, 1e-6, "ecef_llh_south_pole_h");
    }

    /* Degenerate inputs from inside the ellipsoid, up to the geocentre
       itself: defined and finite, latitude still a latitude. */
    {
        static const double inner[][3] = {
            {0.0, 0.0, 0.0}, {1000.0, 0.0, 0.0}, {0.0, 0.0, 1000.0}, {-3.0e6, 2.0e6, 1.0e6}};
        int k;
        int bad = 0;
        for (k = 0; k < (int)(sizeof(inner) / sizeof(inner[0])); ++k)
        {
            double lat, lon, h;
            ins_ecef_to_latlonh(inner[k], &lat, &lon, &h);
            if (!isfinite(lat) || !isfinite(lon) || !isfinite(h)) { bad++; }
            if (fabs(lat) > M_PI / 2.0 + 1e-12) { bad++; }
        }
        CHECK_TRUE(bad == 0, "ecef_llh_inside_ellipsoid_defined");
    }
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

static void test_omega_n_in_precision(void)
{
    /* The n-frame rate runs in single precision (REQ-SYS-022). Both terms
       it carries are small -- the Earth rate is 7.3e-5 rad/s and the
       transport rate is a velocity over an Earth radius -- so a float has
       room to spare. Held against the same formula evaluated entirely in
       double, over the latitudes the NED mechanization is meant for. */
    static const double lats_deg[] = {0.0, 15.0, 45.0, -45.0, 60.0, -60.0, 80.0, -80.0};
    static const double heights[]  = {0.0, 500.0, 12000.0, 30000.0};
    static const float  vels[][3]  = {{0.0f, 0.0f, 0.0f},
                                      {30.0f, 30.0f, 1.0f},
                                      {200.0f, 200.0f, 10.0f},
                                      {0.0f, 300.0f, 0.0f},
                                      {600.0f, 600.0f, 20.0f}};
    double              worst_abs  = 0.0;
    int                 il, ih, iv;

    for (il = 0; il < (int)(sizeof(lats_deg) / sizeof(lats_deg[0])); ++il)
    {
        for (ih = 0; ih < (int)(sizeof(heights) / sizeof(heights[0])); ++ih)
        {
            for (iv = 0; iv < (int)(sizeof(vels) / sizeof(vels[0])); ++iv)
            {
                const double lat = lats_deg[il] * M_PI / 180.0;
                const double h   = heights[ih];

                /* Reference: the identical formula, all double. */
                const double sl     = sin(lat);
                const double cl     = cos(lat);
                const double tl     = sl / cl;
                const double denom  = 1.0 - INS_WGS84_E2 * sl * sl;
                const double sd     = sqrt(denom);
                const double Rn     = INS_WGS84_A * (1.0 - INS_WGS84_E2) / (denom * sd);
                const double Re     = INS_WGS84_A / sd;
                const double ref[3] = {INS_WGS84_OMEGA * cl + (double)vels[iv][1] / (Re + h),
                                       -(double)vels[iv][0] / (Rn + h),
                                       -INS_WGS84_OMEGA * sl - (double)vels[iv][1] * tl / (Re + h)};

                float omega[3];
                int   k;
                ins_calc_omega_n_in(lat, h, vels[iv], omega, NULL, NULL);
                for (k = 0; k < 3; ++k)
                {
                    const double e = fabs((double)omega[k] - ref[k]);
                    if (e > worst_abs) { worst_abs = e; }
                }
            }
        }
    }

    printf("omega_n_in: worst %.3g rad/s vs double (%.3g of the Earth rate)\n", worst_abs,
           worst_abs / INS_WGS84_OMEGA);
    /* 1e-9 rad/s is 2e-4 deg/h of attitude drift, orders below the bias
       stability of any gyro these filters run on. */
    CHECK_TRUE(worst_abs < 1e-9, "n-frame rate within 1e-9 rad/s of the double formula");

    /* Every finite input must give a finite rate, including the poles and
       latitudes handed in out of range. */
    {
        static const double edge_deg[] = {89.0, 89.999, 90.0, -90.0, 90.5, -120.0};
        const float         v[3]       = {100.0f, 100.0f, 0.0f};
        int                 k, bad = 0;
        for (k = 0; k < (int)(sizeof(edge_deg) / sizeof(edge_deg[0])); ++k)
        {
            float w[3];
            int   i;
            ins_calc_omega_n_in(edge_deg[k] * M_PI / 180.0, 0.0, v, w, NULL, NULL);
            for (i = 0; i < 3; ++i)
            {
                if (!isfinite(w[i]) || fabs((double)w[i]) > 1.0) { bad++; }
            }
        }
        CHECK_TRUE(bad == 0, "edge latitudes stay finite and bounded");
    }
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

/* Declination is a wrapped angle in (-180, +180], so two neighbouring grid
   nodes can straddle that cut while being a few degrees apart. Interpolating
   the raw tabulated values then runs the long way round. Both the spatial and
   the temporal interpolation have cells where this happens, along the agonic
   line trailing each dip pole. Near 89 S / 148 E the four surrounding nodes
   are -176.5, +178.5, -178.0 and +176.0 deg, which used to average to about
   +36 deg instead of +180. */
static void test_wmm_declination_wraparound(void)
{
    /* Reference values from the exact spherical-harmonics model (pygeomag).
       The horizontal field here is a healthy 16 uT, so the declination is
       well defined and the grid resolves it. */
    CHECK_NEAR(magnetic_declination_deg(-89.0f, 148.0f, 2027.5f), 179.77, 0.5,
               "wmm_wrap_south_pole_side");
    CHECK_NEAR(magnetic_declination_deg(-88.0f, 148.0f, 2027.5f), 179.39, 0.5,
               "wmm_wrap_south_88deg");
    CHECK_NEAR(magnetic_declination_deg(-85.0f, 148.0f, 2027.5f), 177.91, 0.5,
               "wmm_wrap_south_85deg");

    /* Crossing the cut must not produce a jump between two neighbouring
       queries either, so walking the meridian stays continuous. */
    float prev = magnetic_declination_deg(-89.0f, 140.0f, 2027.5f);
    for (int i = 1; i <= 20; ++i)
    {
        const float lon  = 140.0f + (float)i;
        const float now  = magnetic_declination_deg(-89.0f, lon, 2027.5f);
        const float step = fabsf((float)ins_wrap_pi_bounded((now - prev) * (float)M_PI / 180.0f));
        char        name[56];
        snprintf(name, sizeof(name), "wmm_wrap_continuous_lon[%d]", i);
        CHECK_TRUE(step < 0.35f, name); /* < 20 deg per 1 deg of longitude */
        prev = now;
    }

    /* Temporal wrap: the nodes at lat 90, lon 160 and 165 sit at +174 deg in
       the first epoch and -176 deg in the second. Interpolating those raw
       values would sweep ~350 deg over five years and put mid-epoch near
       zero, so the three sampled years must stay near +/-180 instead. */
    const float t0 = magnetic_declination_deg(89.9f, 162.0f, 2025.0f);
    const float t1 = magnetic_declination_deg(89.9f, 162.0f, 2027.5f);
    const float t2 = magnetic_declination_deg(89.9f, 162.0f, 2030.0f);
    CHECK_TRUE(fabsf(t0) > 150.0f, "wmm_wrap_time_start");
    CHECK_TRUE(fabsf(t1) > 150.0f, "wmm_wrap_time_mid");
    CHECK_TRUE(fabsf(t2) > 150.0f, "wmm_wrap_time_end");
}

/* At a magnetic dip pole the horizontal field vanishes, so the declination is
   ill-conditioned no matter how fine the grid is. The model reports the
   distance to the nearest pole so a caller can drop magnetic heading aiding
   there instead of trusting a meaningless declination.

   The pole coordinates below are the mid-epoch positions of the tabulated
   epoch, and the poles drift (the northern one by ~33 km per year), so
   regenerating wmm_lut.h for a new epoch will break these. That is
   deliberate. Re-derive them from the new table and re-run
   magneticmodel/wmm_error_analysis.py at --step 0.5 to confirm the exclusion
   radius still holds, rather than widening the tolerances. */
static void test_wmm_dip_pole_zone(void)
{
    /* Far from both poles the reference is valid and the distance is large. */
    CHECK_TRUE(magnetic_heading_reference_valid(48.137f, 11.575f), "wmm_dip_munich_valid");
    CHECK_TRUE(magnetic_dip_pole_distance_deg(48.137f, 11.575f) > 20.0f, "wmm_dip_munich_far");

    /* The north dip pole sits near 85 N / 133 E at mid-epoch. A query right
       there must report ~0 distance and an invalid reference. */
    const float d_pole = magnetic_dip_pole_distance_deg(85.24f, 132.69f);
    CHECK_TRUE(d_pole < 1.0f, "wmm_dip_north_distance_zero");
    CHECK_TRUE(!magnetic_heading_reference_valid(85.24f, 132.69f), "wmm_dip_north_invalid");
    /* Same for the southern one, which sits at a far lower latitude (~64 S)
       and is therefore NOT excluded by any plain latitude limit. */
    CHECK_TRUE(!magnetic_heading_reference_valid(-63.75f, 134.69f), "wmm_dip_south_invalid");
    CHECK_TRUE(magnetic_dip_pole_distance_deg(-63.75f, 134.69f) < 1.0f,
               "wmm_dip_south_distance_zero");

    /* The zone has to actually cover where the grid fails: at 86 N / 136 E the
       interpolated declination is off by ~105 deg against the exact model. */
    CHECK_TRUE(!magnetic_heading_reference_valid(86.0f, 136.0f),
               "wmm_dip_degenerate_point_excluded");

    /* Distance is a great-circle angle, so it stays bounded and does not care
       how the caller expresses longitude. */
    const float d_ref  = magnetic_dip_pole_distance_deg(10.0f, 20.0f);
    const float d_wrap = magnetic_dip_pole_distance_deg(10.0f, 380.0f);
    CHECK_NEAR(d_wrap, d_ref, 1e-2, "wmm_dip_distance_lon_wrap");
    CHECK_TRUE(d_ref >= 0.0f && d_ref <= 180.0f, "wmm_dip_distance_bounded");

    /* Out-of-range latitude clamps like every other query in the module. */
    CHECK_NEAR(magnetic_dip_pole_distance_deg(120.0f, 11.0f),
               magnetic_dip_pole_distance_deg(90.0f, 11.0f), 1e-2, "wmm_dip_distance_lat_clamp");
}

int main(void)
{
    test_quat_identity();
    test_rpy_roundtrip();
    test_ecef_roundtrip();
    test_wgs84_constants_consistent();
    test_dned_dlatlonh_precision();
    test_ecef_to_latlonh_closed_form();
    test_rotation_rate_small();
    test_rotation_rate_known();
    test_transport_rate_pole();
    test_omega_n_in_precision();
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
    test_wmm_declination_wraparound();
    test_wmm_dip_pole_zone();
    printf("\n%d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
