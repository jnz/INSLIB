/** @file geodetic_toolbox.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Implementation of geodetic / navigation math functions */

#include <math.h>
#include <string.h>

#include "geodetic_toolbox.h"
#include "linalg.h" /* for MAT_ELEM macro */

/* ============================================================================
 * Quaternion operations
 * ============================================================================
 */

void ins_quat_normalize(float q[4])
{
    const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 > 1e-20f)
    {
        const float inv = 1.0f / SQRTF(n2);
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
    else
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}

void ins_quat_multiply(const float q1[4], const float q2[4], float q_out[4])
{
    /* Hamilton product q_out = q1 * q2, q[0] = w (scalar). */
    const float aw = q1[0], ax = q1[1], ay = q1[2], az = q1[3];
    const float bw = q2[0], bx = q2[1], by = q2[2], bz = q2[3];

    q_out[0] = aw * bw - ax * bx - ay * by - az * bz;
    q_out[1] = aw * bx + ax * bw + ay * bz - az * by;
    q_out[2] = aw * by - ax * bz + ay * bw + az * bx;
    q_out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

void ins_quat_invert(const float q[4], float q_out[4])
{
    /* Inverse of a unit quaternion == conjugate [w, -x, -y, -z]. */
    q_out[0] = q[0];
    q_out[1] = -q[1];
    q_out[2] = -q[2];
    q_out[3] = -q[3];
}

void ins_quat_to_rotmat(const float q[4], float R[9])
{
    /* Hamilton quaternion, q[0] = w (scalar). Same convention as the
       MATLAB strapdown.m reference. R is column-major (MAT_ELEM). */
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    /* Row 0 */
    MAT_ELEM(R, 0, 0, 3, 3) = qw * qw + qx * qx - qy * qy - qz * qz;
    MAT_ELEM(R, 0, 1, 3, 3) = 2.0f * (qx * qy - qw * qz);
    MAT_ELEM(R, 0, 2, 3, 3) = 2.0f * (qx * qz + qw * qy);
    /* Row 1 */
    MAT_ELEM(R, 1, 0, 3, 3) = 2.0f * (qx * qy + qw * qz);
    MAT_ELEM(R, 1, 1, 3, 3) = qw * qw - qx * qx + qy * qy - qz * qz;
    MAT_ELEM(R, 1, 2, 3, 3) = 2.0f * (qy * qz - qw * qx);
    /* Row 2 */
    MAT_ELEM(R, 2, 0, 3, 3) = 2.0f * (qx * qz - qw * qy);
    MAT_ELEM(R, 2, 1, 3, 3) = 2.0f * (qy * qz + qw * qx);
    MAT_ELEM(R, 2, 2, 3, 3) = qw * qw - qx * qx - qy * qy + qz * qz;
}

void ins_rotmat_to_rpy(const float R[9], float* roll_rad, float* pitch_rad, float* yaw_rad)
{
    /* ZYX Tait-Bryan extraction for R_b_to_n.
       pitch = asin(-R(2,0))
       roll  = atan2(R(2,1), R(2,2))
       yaw   = atan2(R(1,0), R(0,0))
       (at gimbal lock, yaw is set to 0) */
    const float r20 = MAT_ELEM(R, 2, 0, 3, 3);
    const float r21 = MAT_ELEM(R, 2, 1, 3, 3);
    const float r22 = MAT_ELEM(R, 2, 2, 3, 3);
    const float r10 = MAT_ELEM(R, 1, 0, 3, 3);
    const float r00 = MAT_ELEM(R, 0, 0, 3, 3);

    /* Clamp to [-1, 1] (branchless) so asinf stays well-defined even if
       round-off pushes the matrix element slightly outside the range. */
    const float sp = fmaxf(-1.0f, fminf(1.0f, -r20));

    *pitch_rad = asinf(sp);

    if (sp < 0.9999f && sp > -0.9999f)
    {
        *roll_rad = atan2f(r21, r22);
        *yaw_rad  = atan2f(r10, r00);
    }
    else
    {
        /* Near gimbal lock. */
        *roll_rad = atan2f(-MAT_ELEM(R, 1, 2, 3, 3), MAT_ELEM(R, 1, 1, 3, 3));
        *yaw_rad  = 0.0f;
    }
}

void ins_quat_from_rpy(float roll_rad, float pitch_rad, float yaw_rad, float q[4])
{
    /* ZYX Tait-Bryan: q = qz(yaw) * qy(pitch) * qx(roll) */
    const float cr = cosf(roll_rad * 0.5f);
    const float sr = sinf(roll_rad * 0.5f);
    const float cp = cosf(pitch_rad * 0.5f);
    const float sp = sinf(pitch_rad * 0.5f);
    const float cy = cosf(yaw_rad * 0.5f);
    const float sy = sinf(yaw_rad * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr; /* w */
    q[1] = cy * cp * sr - sy * sp * cr; /* x */
    q[2] = cy * sp * cr + sy * cp * sr; /* y */
    q[3] = sy * cp * cr - cy * sp * sr; /* z */
    ins_quat_normalize(q);
}

void ins_quat_rotate(const float q[4], const float omega[3], float dt_sec, float q_new[4])
{
    /* Implementation of attitude_rotationrate_update.m using
       Taylor series expansion for robustness near omega = 0.
       Reference: Wendel, "Integrierte Navigationssysteme", 2nd ed. p. 47. */

    const float dax  = omega[0] * dt_sec;
    const float day  = omega[1] * dt_sec;
    const float daz  = omega[2] * dt_sec;
    const float x2   = dax * dax + day * day + daz * daz;
    const float x    = SQRTF(x2);
    const float half = 0.5f * x;

    /* Rotation quaternion qr = [cos(x/2), (omega*dt) * sin(x/2)/x].
       cos(x/2) is regular everywhere; only sin(x/2)/x is 0/0 at x=0.
       For x below the threshold the 2-term limit sin(x/2)/x -> 1/2 - x^2/48
       is exact to float precision (next term x^4/3840 < 1e-17 there). */
    const float s = (x > 1.0e-4f) ? sinf(half) / x : 0.5f - x2 / 48.0f;

    float qr[4];
    qr[0] = cosf(half);
    qr[1] = dax * s;
    qr[2] = day * s;
    qr[3] = daz * s;

    /* q_new = q * qr (Hamilton product, q[0] is scalar). */
    ins_quat_multiply(q, qr, q_new);
    ins_quat_normalize(q_new);
}

void ins_quat_small_angle_correction(const float q_in[4], const float drpy[3], float q_out[4])
{
    /* Apply a small angle correction to the attitude quaternion.
     *
     * Error-state convention (psi-angle model, misalignment in the
     * n-frame, consistent with ins_compute_Phi where the gyro bias
     * couples into the attitude error via -R*dt):
     *
     *   R_nominal   = (I + [drpy]_x) * R_true
     *   R_corrected = (I - [drpy]_x) * R_nominal
     *
     * i.e. a LEFT (n-frame side) quaternion multiplication:
     *   q_corrected = q_small_angle(-drpy) * q_in
     *
     * (A right multiplication would apply the n-frame error about the
     * body axes, which is only equivalent near identity attitude.) */
    const float hx    = -0.5f * drpy[0];
    const float hy    = -0.5f * drpy[1];
    const float hz    = -0.5f * drpy[2];
    const float hmag2 = hx * hx + hy * hy + hz * hz;

    float qw, qx, qy, qz;
    if (hmag2 < 1e-16f)
    {
        qw = 1.0f;
        qx = hx;
        qy = hy;
        qz = hz;
    }
    else
    {
        const float h = SQRTF(hmag2);
        const float c = cosf(h);
        const float s = sinf(h) / h;
        qw            = c;
        qx            = hx * s;
        qy            = hy * s;
        qz            = hz * s;
    }

    /* q_out = q_small_angle * q_in (left / n-frame side multiplication). */
    const float q_sa[4] = {qw, qx, qy, qz};
    ins_quat_multiply(q_sa, q_in, q_out);
    ins_quat_normalize(q_out);
}

void ins_matrix_to_quat(const float R[9], float q[4])
{
    /* Numerically stable trace-based conversion (largest pivot), the
       inverse of ins_quat_to_rotmat. R is column-major (MAT_ELEM). */
    const float r00 = MAT_ELEM(R, 0, 0, 3, 3);
    const float r11 = MAT_ELEM(R, 1, 1, 3, 3);
    const float r22 = MAT_ELEM(R, 2, 2, 3, 3);
    const float r01 = MAT_ELEM(R, 0, 1, 3, 3);
    const float r02 = MAT_ELEM(R, 0, 2, 3, 3);
    const float r10 = MAT_ELEM(R, 1, 0, 3, 3);
    const float r12 = MAT_ELEM(R, 1, 2, 3, 3);
    const float r20 = MAT_ELEM(R, 2, 0, 3, 3);
    const float r21 = MAT_ELEM(R, 2, 1, 3, 3);
    const float tr  = r00 + r11 + r22;

    if (tr > 0.0f)
    {
        const float S = 2.0f * SQRTF(tr + 1.0f);
        q[0]          = 0.25f * S;
        q[1]          = (r21 - r12) / S;
        q[2]          = (r02 - r20) / S;
        q[3]          = (r10 - r01) / S;
    }
    else if (r00 > r11 && r00 > r22)
    {
        const float S = 2.0f * SQRTF(1.0f + r00 - r11 - r22);
        q[0]          = (r21 - r12) / S;
        q[1]          = 0.25f * S;
        q[2]          = (r01 + r10) / S;
        q[3]          = (r02 + r20) / S;
    }
    else if (r11 > r22)
    {
        const float S = 2.0f * SQRTF(1.0f + r11 - r00 - r22);
        q[0]          = (r02 - r20) / S;
        q[1]          = (r01 + r10) / S;
        q[2]          = 0.25f * S;
        q[3]          = (r12 + r21) / S;
    }
    else
    {
        const float S = 2.0f * SQRTF(1.0f + r22 - r00 - r11);
        q[0]          = (r10 - r01) / S;
        q[1]          = (r02 + r20) / S;
        q[2]          = (r12 + r21) / S;
        q[3]          = 0.25f * S;
    }

    ins_quat_normalize(q);
}

void ins_quat_to_axis_angle(const float q[4], float axis[3], float* angle_rad)
{
    float qn[4] = {q[0], q[1], q[2], q[3]};
    ins_quat_normalize(qn);

    /* Clamp to [-1, 1] so acosf stays well-defined. */
    const float w = fmaxf(-1.0f, fminf(1.0f, qn[0]));

    const float s = SQRTF((1.0f - w * w) > 0.0f ? (1.0f - w * w) : 0.0f);
    if (s < 1e-8f)
    {
        /* w ~ +/-1: rotation of 0 (or 2*pi), axis undefined -> default. */
        axis[0]    = 1.0f;
        axis[1]    = 0.0f;
        axis[2]    = 0.0f;
        *angle_rad = 0.0f;
        return;
    }

    *angle_rad = 2.0f * acosf(w);
    axis[0]    = qn[1] / s;
    axis[1]    = qn[2] / s;
    axis[2]    = qn[3] / s;
}

/* ============================================================================
 * Geodesy (WGS84)
 * ============================================================================
 */

/* INS_WGS84_B and INS_WGS84_EP2 are stored as literals so the conversion
   below needs no square root to set up, which makes them a second spelling of
   INS_WGS84_A and INS_WGS84_E2. A _Static_assert would be the natural place
   to hold the two spellings against each other, but the comparison is a
   floating point one and C11 wants an integer constant expression there,
   which clang rejects outright and gcc only tolerates outside -pedantic. The
   check therefore lives in the test suite, see
   test_ins_math.c:test_wgs84_constants_consistent. */

void ins_ecef_to_latlonh(const double xyz[3], double* lat_rad, double* lon_rad, double* height_m)
{
    const double a  = INS_WGS84_A;
    const double e2 = INS_WGS84_E2;
    const double x  = xyz[0];
    const double y  = xyz[1];
    const double z  = xyz[2];

    const double p = sqrt(x * x + y * y);
    *lon_rad       = atan2(y, x);

    /* Bowring's closed form via the parametric (reduced) latitude theta.
       One pass, no iteration, so the instruction sequence is the same for
       every input, which is what a caller with a worst case execution time
       to hold needs. The error against the converged solution grows with
       height: below a micrometre up to aircraft altitudes, about a
       millimetre at 400 km and a few decimetres at geostationary altitude.
       Anything reaching past low orbit wants an iterative solution. */
    const double theta = atan2(z * a, p * INS_WGS84_B);
    const double st    = sin(theta);
    const double ct    = cos(theta);
    /* The denominator turns negative only for points within about 43 km of
       the geocentre, where no position the filters work with can lie. It is
       floored so such an input still comes back as a latitude inside
       [-pi/2, pi/2] instead of one reflected past the pole. */
    double den = p - e2 * a * ct * ct * ct;
    if (den < 0.0) { den = 0.0; }
    const double lat = atan2(z + INS_WGS84_EP2 * INS_WGS84_B * st * st * st, den);

    /* N and the two height forms share sin(lat). cos(lat) comes from the
       identity rather than from a second call to the trigonometric
       library: it is only ever used where it is the larger of the two and
       therefore well conditioned, and a double cosine is expensive on a
       target without a double-precision FPU. */
    const double sl = sin(lat);
    const double cl = sqrt(1.0 - sl * sl); /* |lat| <= pi/2, so cos >= 0 */
    const double N  = a / sqrt(1.0 - e2 * sl * sl);

    /* p/cos(lat) collapses at the poles and z/sin(lat) collapses at the
       equator, so the height is taken from whichever of the two divisors
       is the larger. Both arms cost one division, the branch does not
       make the run time depend on the input. */
    *lat_rad  = lat;
    *height_m = (cl > fabs(sl)) ? (p / cl - N) : (z / sl - N * (1.0 - e2));
}

void ins_latlonh_to_ecef(double lat_rad, double lon_rad, double height_m, double xyz[3])
{
    const double a  = INS_WGS84_A;
    const double e2 = INS_WGS84_E2;
    const double sl = sin(lat_rad);
    const double cl = cos(lat_rad);
    const double so = sin(lon_rad);
    const double co = cos(lon_rad);
    const double N  = a / sqrt(1.0 - e2 * sl * sl);

    xyz[0] = (N + height_m) * cl * co;
    xyz[1] = (N + height_m) * cl * so;
    xyz[2] = (N * (1.0 - e2) + height_m) * sl;
}

void ins_rotmat_n_to_e(double lat_rad, double lon_rad, float R[9])
{
    /* R_n_to_e: transforms NED vector into ECEF.
       R_n_to_e = [ -sL*cO   -sO   -cL*cO
                    -sL*sO    cO   -cL*sO
                     cL       0    -sL   ]
       (lat = L, lon = O). Column-major storage. */
    const float latf = (float)lat_rad;
    const float lonf = (float)lon_rad;
    const float sL = sinf(latf), cL = cosf(latf);
    const float sO = sinf(lonf), cO = cosf(lonf);

    /* Column 0 (North) */
    MAT_ELEM(R, 0, 0, 3, 3) = -sL * cO;
    MAT_ELEM(R, 1, 0, 3, 3) = -sL * sO;
    MAT_ELEM(R, 2, 0, 3, 3) = cL;
    /* Column 1 (East) */
    MAT_ELEM(R, 0, 1, 3, 3) = -sO;
    MAT_ELEM(R, 1, 1, 3, 3) = cO;
    MAT_ELEM(R, 2, 1, 3, 3) = 0.0f;
    /* Column 2 (Down) */
    MAT_ELEM(R, 0, 2, 3, 3) = -cL * cO;
    MAT_ELEM(R, 1, 2, 3, 3) = -cL * sO;
    MAT_ELEM(R, 2, 2, 3, 3) = -sL;
}

/* @satisfies REQ-SYS-022 */
void ins_calc_omega_n_in(double lat_rad, double height_m, const float vel_ned[3],
                         float omega_n_in[3], float omega_n_ie_out[3], float omega_n_en_out[3])
{
    /* Single precision throughout, which the magnitudes allow: the Earth
       rate is 7.3e-5 rad/s and the transport rate is a velocity divided by
       an Earth radius. */
    const float lat = (float)lat_rad;
    const float h   = (float)height_m;
    const float sl  = sinf(lat);
    const float cl  = cosf(lat);
    const float a   = (float)INS_WGS84_A;
    const float e2  = (float)INS_WGS84_E2;

    /* tan(lat) = sin/cos drives the vertical (azimuth) transport rate and
       diverges at the poles (cos -> 0): a north-slaved NED frame is
       singular there. At exactly +/-90 deg, tan(pi/2) is ~1.6e16
       (not even Inf, since pi/2 isn't exactly representable), which with any
       non-zero east velocity yields an absurd rate that wrecks the strapdown
       attitude integration. Floor |cos(lat)| so the term stays finite, the
       clamp only bites within ~0.006 deg of the pole (where NED nav is not
       usable anyway) and leaves all lower latitudes untouched. copysign
       preserves the (physically non-negative) cos sign so out-of-range
       latitudes stay well-behaved too. */
    const float cl_safe = copysignf(fmaxf(fabsf(cl), INS_POLE_COS_FLOOR), cl);
    const float tl      = sl / cl_safe;

    const float denom      = 1.0f - e2 * sl * sl;
    const float sqrt_denom = sqrtf(denom);
    const float Rn         = a * (1.0f - e2) / (denom * sqrt_denom); /* meridian */
    const float Re         = a / sqrt_denom;                         /* prime vertical */

    /* Earth rotation rate in n-frame */
    const float wie_n[3] = {(float)INS_WGS84_OMEGA * cl, 0.0f, -(float)INS_WGS84_OMEGA * sl};

    const float wen_n[3] = {vel_ned[1] / (Re + h), -vel_ned[0] / (Rn + h),
                            -vel_ned[1] * tl / (Re + h)};

    omega_n_in[0] = wie_n[0] + wen_n[0];
    omega_n_in[1] = wie_n[1] + wen_n[1];
    omega_n_in[2] = wie_n[2] + wen_n[2];

    if (omega_n_ie_out != NULL)
    {
        omega_n_ie_out[0] = wie_n[0];
        omega_n_ie_out[1] = wie_n[1];
        omega_n_ie_out[2] = wie_n[2];
    }
    if (omega_n_en_out != NULL)
    {
        omega_n_en_out[0] = wen_n[0];
        omega_n_en_out[1] = wen_n[1];
        omega_n_en_out[2] = wen_n[2];
    }
}

void ins_gravity_ned(float lat_rad, float height_m, float gravity_n[3])
{
    /* WGS84 normal gravity (Somigliana) with free-air correction.
       Result in NED, so z is positive (down). */

    const float sin_lat = sinf(lat_rad);
    const float s       = sin_lat * sin_lat;
    const float g0      = INS_GAMMA_E * (1.0f + s * (INS_G0_C1 + s * (INS_G0_C2 + s * INS_G0_C3)));

    /* Free air: 1 + m + f*cos(2L),  cos(2L) = 1 - 2s */
    const float k = 1.0f + INS_WGS84_M + INS_WGS84_F * (1.0f - 2.0f * s);
    const float g = g0 * (1.0f - (2.0f / INS_WGS84_A_F) * k * height_m +
                          (3.0f / (INS_WGS84_A_F * INS_WGS84_A_F)) * height_m * height_m);

    gravity_n[0] = 0.0f;
    gravity_n[1] = 0.0f;
    gravity_n[2] = g;
}

/* Curvature radii of the meridian and the prime vertical at one latitude,
   plus the cosine the east/longitude term divides by, all in single
   precision.

   The geodetic side of both mappings below is double because the caller
   forms it by subtracting two absolute coordinates, and at a latitude of
   about 0.85 rad a single-precision step is 0.38 m while the difference
   itself is metre-scale. That cancellation is over by the time the radii
   are needed: what happens here is a multiplication by about 6.4e6 m,
   where single precision carries 8e-8 relative, under a micrometre on a
   metre-scale residual. A double sine, cosine and square root cost around
   ten times their single-precision counterparts on a part without a
   double-precision FPU, and this sits on the per-epoch path.

   cos(lat) is floored: toward the pole the east term divides by it, and a
   single-precision cosine of a latitude that close to pi/2 can round to
   zero or past it into the wrong sign. The floor keeps the mapping finite
   and signed, the same bound the transport rate uses for tan(lat). */
static void ins_curvature_radii(double lat_rad, double height_m, float* Rn_h, float* Re_h_cl)
{
    const float latf = (float)lat_rad;
    const float sl   = sinf(latf);
    float       cl   = cosf(latf);
    const float e2   = (float)INS_WGS84_E2;
    const float hf   = (float)height_m;

    if (cl < (float)INS_POLE_COS_FLOOR) { cl = (float)INS_POLE_COS_FLOOR; }

    const float denom      = 1.0f - e2 * sl * sl;
    const float sqrt_denom = SQRTF(denom);
    const float Rn         = (float)(INS_WGS84_A * (1.0 - INS_WGS84_E2)) / (denom * sqrt_denom);
    const float Re         = INS_WGS84_A_F / sqrt_denom;

    *Rn_h    = Rn + hf;
    *Re_h_cl = (Re + hf) * cl;
}

void ins_dned_to_dlatlonh(const float dxyz_n[3], double lat_rad, double height_m,
                          double dlatlonh[3])
{
    float Rn_h, Re_h_cl;
    ins_curvature_radii(lat_rad, height_m, &Rn_h, &Re_h_cl);

    /* dnorth -> dlat; deast -> dlon, ddown -> -dheight */
    dlatlonh[0] = (double)(dxyz_n[0] / Rn_h);
    dlatlonh[1] = (double)(dxyz_n[1] / Re_h_cl);
    dlatlonh[2] = -(double)dxyz_n[2];
}

void ins_dlatlonh_to_dned(const double dlatlonh[3], double lat_rad, double height_m,
                          float dxyz_n[3])
{
    /* Exact inverse of ins_dned_to_dlatlonh: same radii, same cosine floor,
       from the same helper, so the two stay each other's inverse wherever
       either of them is defined. */
    float Rn_h, Re_h_cl;
    ins_curvature_radii(lat_rad, height_m, &Rn_h, &Re_h_cl);

    /* dlat -> dnorth; dlon -> deast; dheight -> -ddown */
    dxyz_n[0] = (float)dlatlonh[0] * Rn_h;
    dxyz_n[1] = (float)dlatlonh[1] * Re_h_cl;
    dxyz_n[2] = (float)(-dlatlonh[2]);
}

/* ============================================================================
 * Small helpers
 * ============================================================================
 */

void ins_cross_matrix(const float v[3], float M[9])
{
    /* M = [  0   -vz   vy
             vz    0   -vx
            -vy   vx    0  ]   (column-major) */
    MAT_ELEM(M, 0, 0, 3, 3) = 0.0f;
    MAT_ELEM(M, 1, 0, 3, 3) = v[2];
    MAT_ELEM(M, 2, 0, 3, 3) = -v[1];

    MAT_ELEM(M, 0, 1, 3, 3) = -v[2];
    MAT_ELEM(M, 1, 1, 3, 3) = 0.0f;
    MAT_ELEM(M, 2, 1, 3, 3) = v[0];

    MAT_ELEM(M, 0, 2, 3, 3) = v[1];
    MAT_ELEM(M, 1, 2, 3, 3) = -v[0];
    MAT_ELEM(M, 2, 2, 3, 3) = 0.0f;
}

void ins_cross(const float a[3], const float b[3], float out[3])
{
    const float x = a[1] * b[2] - a[2] * b[1];
    const float y = a[2] * b[0] - a[0] * b[2];
    const float z = a[0] * b[1] - a[1] * b[0];
    out[0]        = x;
    out[1]        = y;
    out[2]        = z;
}

float ins_angle_diff(float a, float b)
{
    /* Signed smallest difference, wrapped to [-pi, pi]. Matches the Python
       floored-modulo form (d + pi) mod 2*pi - pi. */
    const float two_pi = 2.0f * (float)M_PI;
    float       d      = fmodf(a - b + (float)M_PI, two_pi);
    if (d < 0.0f) d += two_pi;
    return d - (float)M_PI;
}

float ins_wrap_pi_bounded(float a)
{
    /* Single-step wrap, so the input must be in (-3pi, 3pi); use
       ins_angle_diff() for unbounded inputs. */
    if (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

bool ins_vec3_finite(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

float ins_isa_altitude_from_pressure(float pressure_pa)
{
    return INS_ISA_SCALE_M * (1.0f - powf(pressure_pa / INS_ISA_P0_PA, INS_ISA_EXP));
}

bool ins_isa_pressure_plausible(float pressure_pa)
{
    return isfinite(pressure_pa) && pressure_pa >= INS_ISA_PRESSURE_MIN_PA &&
           pressure_pa <= INS_ISA_PRESSURE_MAX_PA;
}
