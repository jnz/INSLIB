/** @file geodetic_toolbox.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Geodetic / navigation math toolbox (C port of
 *        python/geodetic_toolbox.py).
 *
 * Contains quaternion operations, geodesy (WGS84) utilities and
 * small helper functions. All matrices are in column-major order
 * (consistent with linalg.h).
 *
 * Conventions:
 *  - Hamilton quaternion, q[0] is the scalar part (w),
 *    q[1..3] is the vector part (x,y,z).
 *  - q represents R_b_to_n (body to NED navigation frame).
 *  - All angles in radians unless otherwise noted.
 *  - lat/lon in radians, height in meters above ellipsoid.
 *
 * Name mapping to the Python reference (python/geodetic_toolbox.py).
 *
 *   Python                        C
 *   quat_from_rpy                 ins_quat_from_rpy
 *   quat_to_matrix                ins_quat_to_rotmat
 *   matrix_to_quat                ins_matrix_to_quat
 *   quat_to_rpy                   (compose quat_to_rotmat + rotmat_to_rpy)
 *   extract_rpy_from_R_b_to_n     ins_rotmat_to_rpy
 *   quat_norm                     ins_quat_normalize
 *   quat_multiply                 ins_quat_multiply
 *   quat_integrate_rotationrate   ins_quat_rotate
 *   quat_invert                   ins_quat_invert
 *   quat_to_axis_angle            ins_quat_to_axis_angle
 *   angle_diff                    ins_angle_diff
 * C-only additions (no Python counterpart): the WGS84 geodesy block,
 * ins_quat_small_angle_correction, ins_cross[_matrix]
 *
 */

/** @addtogroup geodetic
 *  @{ */

#ifndef GEODETIC_TOOLBOX_H
#define GEODETIC_TOOLBOX_H

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdbool.h>

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** WGS84 semi-major axis [m] */
#define INS_WGS84_A (6378137.0)
/** WGS84 eccentricity squared (e^2 = f*(2-f)) */
#define INS_WGS84_E2 (0.00669437999014)
/** WGS84 semi-minor axis [m], b = a*sqrt(1 - e^2) from #INS_WGS84_A and
 *  #INS_WGS84_E2. Stored rather than derived so the closed-form
 *  ECEF to geodetic conversion costs no square root to set up. */
#define INS_WGS84_B (6356752.314245184)
/** WGS84 second eccentricity squared, e'^2 = e^2 / (1 - e^2). Same
 *  reason as #INS_WGS84_B. */
#define INS_WGS84_EP2 (0.0067394967422751)
/** Earth rotation rate [rad/s] */
#define INS_WGS84_OMEGA (7.2921151467E-5)
/** Nominal gravity [m/s^2] (used as a fallback) */
#define INS_GRAVITY_NOMINAL (9.80665f)
/** Floor on |cos(lat)| used to bound tan(lat) in the azimuth transport
 *  rate near the poles (~0.006 deg from +/-90 deg). Keeps the NED
 *  mechanization's polar singularity finite instead of divergent. */
#define INS_POLE_COS_FLOOR (1.0e-4f)

/** Somigliana normal gravity at the equator [m/s^2]. */
#define INS_GAMMA_E (9.7803253359f)
/** Somigliana normal gravity formula, sin^2(lat) coefficient. */
#define INS_G0_C1 (5.27904265e-3f)
/** Somigliana normal gravity formula, sin^4(lat) coefficient. */
#define INS_G0_C2 (2.32718e-5f)
/** Somigliana normal gravity formula, sin^6(lat) coefficient. */
#define INS_G0_C3 (1.26209e-7f)
/** WGS84 flattening f = 1/298.257223563. */
#define INS_WGS84_F (3.35281066475e-3f)
/** WGS84 gravity formula constant m = w^2 a^2 b / GM. */
#define INS_WGS84_M (3.44978650684e-3f)
/** float copy of #INS_WGS84_A, for the height correction in the
 *  free-air gravity formula. */
#define INS_WGS84_A_F ((float)INS_WGS84_A)

/** International Standard Atmosphere, h = SCALE * (1 - (p/p0)^EXP).
 *
 *  Physical constants of the atmosphere model, not tuning defaults: they belong
 *  to the conversion itself and are the same for every consumer. ins and
 *  baro_alt MUST agree on them bit for bit - ins fuses barometric height into
 *  its own vertical position state (REQ-NAV-054) while baro_alt runs the same
 *  pressure stream through its independent vertical filter, and the suite
 *  reconciles the two on one vertical datum by a single origin shift at
 *  bootstrap (REQ-SUITE-007). Two differing ISA curves would turn that one-time
 *  alignment into a pair that drifts apart as a function of pressure. */
#define INS_ISA_P0_PA (101325.0f)
/** @copydoc INS_ISA_P0_PA */
#define INS_ISA_SCALE_M (44330.0f)
/** @copydoc INS_ISA_P0_PA */
#define INS_ISA_EXP (1.0f / 5.255f)

/** Plausibility window for a raw static-pressure sample [Pa]: roughly
 *  16 km altitude down to well below sea level. Also keeps powf() in
 *  ins_isa_altitude_from_pressure() on a sane operand. */
#define INS_ISA_PRESSURE_MIN_PA (10000.0f)
/** @copydoc INS_ISA_PRESSURE_MIN_PA */
#define INS_ISA_PRESSURE_MAX_PA (120000.0f)

// clang-format off
/** @brief Convert degrees to radians. */
#define DEG2RAD(x) _Generic((x), \
    float:       (x) * ((float)M_PI / 180.0f), \
    default:     (double)(x) * (M_PI / 180.0) \
)
/** @brief Convert radians to degrees. */
#define RAD2DEG(x) _Generic((x), \
    float:       (x) * (180.0f / (float)M_PI), \
    default:     (double)(x) * (180.0 / M_PI) \
)
// clang-format on

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /* ------------------------------------------------------------------------
     * Quaternion operations (Hamilton convention: q = [w, x, y, z])
     * ------------------------------------------------------------------------
     */

    /** @brief Normalize a quaternion in place.
     *  @param[in,out] q Quaternion (4x1) to normalize. */
    void ins_quat_normalize(float q[4]);

    /** @brief Convert a quaternion to a 3x3 rotation matrix R_b_to_n.
     *
     * The matrix is stored column-major (as expected by linalg.h).
     *
     * @param[in] q Hamilton quaternion (q[0] = w, scalar).
     * @param[out] R_b_to_n Output 3x3 rotation matrix in column-major. */
    void ins_quat_to_rotmat(const float q[4], float R_b_to_n[9]);

    /** @brief Extract roll/pitch/yaw (Tait-Bryan ZYX) from a rotation matrix.
     *
     *  Assumes matrix is R_b_to_n (body to NED).
     *
     * @param[in] R_b_to_n 3x3 rotation matrix (column-major).
     * @param[out] roll_rad Output roll [rad].
     * @param[out] pitch_rad Output pitch [rad].
     * @param[out] yaw_rad Output yaw [rad]. */
    void ins_rotmat_to_rpy(const float R_b_to_n[9], float* roll_rad, float* pitch_rad,
                           float* yaw_rad);

    /** @brief Build a quaternion from roll/pitch/yaw (Tait-Bryan ZYX).
     *
     *  Resulting quaternion represents R_b_to_n.
     *
     * @param[in] roll_rad Roll [rad].
     * @param[in] pitch_rad Pitch [rad].
     * @param[in] yaw_rad Yaw [rad].
     * @param[out] q Output Hamilton quaternion (q[0] = w). */
    void ins_quat_from_rpy(float roll_rad, float pitch_rad, float yaw_rad, float q[4]);

    /** @brief Rotate quaternion q by a constant rotation rate omega over dt.
     *
     * Implements the Wendel attitude update with Taylor series expansion
     * for numerical robustness near omega == 0.
     * Source: Wendel, "Integrierte Navigationssysteme", 2nd ed., p. 47.
     *
     * @param[in] q Input quaternion at epoch k (Hamilton).
     * @param[in] omega Rotation rate (rad/s), body to navigation, in body
     * frame.
     * @param[in] dt_sec Time interval [s].
     * @param[out] q_new Output quaternion at epoch k+1. */
    void ins_quat_rotate(const float q[4], const float omega[3], float dt_sec, float q_new[4]);

    /** @brief Apply a small-angle correction to a quaternion.
     *
     * For error-state Kalman filtering (psi-angle model). drpy is the
     * estimated attitude misalignment in the *n-frame* with the convention
     * R_nominal = (I + [drpy]_x) * R_true; the correction is applied as a
     * left (n-frame side) rotation: q_out = q(-drpy) * q_in.
     * Must stay consistent with ins_compute_Phi.
     *
     * @param[in] q_in Uncorrected quaternion.
     * @param[in] drpy_rad Small angle correction 3x1 [rad], n-frame.
     * @param[out] q_out Corrected quaternion. */
    void ins_quat_small_angle_correction(const float q_in[4], const float drpy_rad[3],
                                         float q_out[4]);

    /** @brief Hamilton product of two quaternions: q_out = q1 * q2.
     *
     * q1 and q2 use the q[0] = w (scalar) convention. Aliasing q_out with
     * q1 or q2 is not supported (write to a separate buffer).
     *
     * @param[in] q1 Left quaternion (4x1).
     * @param[in] q2 Right quaternion (4x1).
     * @param[out] q_out Result q1 * q2 (4x1). */
    void ins_quat_multiply(const float q1[4], const float q2[4], float q_out[4]);

    /** @brief Inverse of a unit rotation quaternion (the conjugate).
     *
     * For a unit-length quaternion the inverse equals the conjugate
     * [w, -x, -y, -z]; no normalization is performed.
     *
     * @param[in] q Input unit quaternion (4x1).
     * @param[out] q_out Inverse quaternion (4x1). May alias q. */
    void ins_quat_invert(const float q[4], float q_out[4]);

    /** @brief Convert a 3x3 rotation matrix R_b_to_n to a quaternion.
     *
     * Numerically stable trace-based algorithm (largest pivot). The result
     * is a unit Hamilton quaternion (q[0] = w). Inverse of
     * ins_quat_to_rotmat.
     *
     * @param[in] R_b_to_n 3x3 rotation matrix (column-major).
     * @param[out] q Output unit quaternion (4x1, q[0] = w). */
    void ins_matrix_to_quat(const float R_b_to_n[9], float q[4]);

    /** @brief Extract rotation axis and angle from a unit quaternion.
     *
     * For a near-zero rotation (|w| -> 1) the axis is undefined; a default
     * axis [1,0,0] and angle 0 are returned.
     *
     * @param[in] q Input unit quaternion (4x1, q[0] = w).
     * @param[out] axis Output unit rotation axis (3x1).
     * @param[out] angle_rad Output rotation angle [rad], range [0, 2*pi). */
    void ins_quat_to_axis_angle(const float q[4], float axis[3], float* angle_rad);

    /* ------------------------------------------------------------------------
     * Geodesy (WGS84)
     * ------------------------------------------------------------------------
     */

    /** @brief Convert ECEF coordinates to geodetic lat/lon/height (WGS84).
     *
     * Uses an iterative algorithm (Bowring). Valid globally.
     *
     * @param[in] xyz ECEF position [m] as double (3x1).
     * @param[out] lat_rad Output latitude [rad].
     * @param[out] lon_rad Output longitude [rad].
     * @param[out] height_m Output height above ellipsoid [m]. */
    void ins_ecef_to_latlonh(const double xyz[3], double* lat_rad, double* lon_rad,
                             double* height_m);

    /** @brief Convert geodetic lat/lon/height to ECEF coordinates (WGS84).
     *
     * @param[in] lat_rad Latitude [rad].
     * @param[in] lon_rad Longitude [rad].
     * @param[in] height_m Height above ellipsoid [m].
     * @param[out] xyz Output ECEF position [m] (3x1 double). */
    void ins_latlonh_to_ecef(double lat_rad, double lon_rad, double height_m, double xyz[3]);

    /** @brief Get rotation matrix from n-frame (NED) to ECEF (e-frame).
     *
     * Given the current lat/lon, R_n_to_e transforms a vector from NED
     * into ECEF. The transpose (R_n_to_e') transforms ECEF into NED.
     *
     * @param[in] lat_rad Latitude [rad].
     * @param[in] lon_rad Longitude [rad].
     * @param[out] R_n_to_e Output 3x3 rotation matrix (column-major). */
    void ins_rotmat_n_to_e(double lat_rad, double lon_rad, float R_n_to_e[9]);

    /** @brief Compute rotation rate of n-frame relative to inertial frame in
     * n-frame.
     *
     * omega_n_in = omega_n_ie + omega_n_en
     *   omega_n_ie: Earth rotation rate expressed in n-frame
     *   omega_n_en: Transport rate (depends on velocity and curvature radii)
     *
     * @param[in] lat_rad Latitude [rad].
     * @param[in] height_m Height above ellipsoid [m].
     * @param[in] vel_ned Velocity in NED frame [m/s] (3x1).
     * @param[out] omega_n_in Output total rotation rate (3x1) [rad/s].
     * @param[out] omega_n_ie_out Optional output (may be NULL), Earth rate in
     * n-frame.
     * @param[out] omega_n_en_out Optional output (may be NULL), transport rate. */
    void ins_calc_omega_n_in(double lat_rad, double height_m, const float vel_ned[3],
                             float omega_n_in[3], float omega_n_ie_out[3], float omega_n_en_out[3]);

    /** @brief Compute gravity vector in n-frame (includes centrifugal term).
     *
     * Simple WGS84 normal gravity model with latitude dependence.
     * Result is in the n-frame (NED) - so the z-component is positive
     * (pointing down).
     *
     * @param[in] lat_rad Latitude [rad].
     * @param[in] height_m Height above ellipsoid [m].
     * @param[out] gravity_n Output gravity vector in NED (3x1) [m/s^2]. */
    void ins_gravity_ned(float lat_rad, float height_m, float gravity_n[3]);

    /** @brief Convert small position delta in n-frame to delta in
     * lat/lon/height.
     *
     * Given a small local displacement dxyz_n (meters in NED) at current
     * lat/lon/height, compute the corresponding changes in lat/lon/height.
     *
     * The geodetic side is double because the caller forms it against an
     * absolute coordinate, where single precision would quantize a latitude
     * to steps of 0.38 m. The curvature radii themselves are evaluated in
     * single precision, which costs a few units in the last place of the
     * result, and more on the longitude component toward the poles, where
     * the division by cos(lat) amplifies the rounded latitude by tan(lat).
     * |cos(lat)| is bounded by #INS_POLE_COS_FLOOR so the longitude stays
     * finite and signed at the pole itself.
     *
     * @param[in] dxyz_n Delta position in NED [m] (3x1 float).
     * @param[in] lat_rad Current latitude [rad].
     * @param[in] height_m Current height above ellipsoid [m].
     * @param[out] dlatlonh Output delta (dlat_rad, dlon_rad, dheight_m). */
    void ins_dned_to_dlatlonh(const float dxyz_n[3], double lat_rad, double height_m,
                              double dlatlonh[3]);

    /** @brief Convert a small lat/lon/height delta to a position delta in NED.
     *
     * Inverse of ins_dned_to_dlatlonh, from the same curvature radii and the
     * same cosine bound, so the two invert each other to single-precision
     * tolerance wherever either is defined.
     * Intended for meter-scale differences (e.g. measurement residuals).
     *
     * @param[in] dlatlonh Delta (dlat_rad, dlon_rad, dheight_m).
     * @param[in] lat_rad Current latitude [rad].
     * @param[in] height_m Current height above ellipsoid [m].
     * @param[out] dxyz_n Output delta position in NED [m] (3x1 float). */
    void ins_dlatlonh_to_dned(const double dlatlonh[3], double lat_rad, double height_m,
                              float dxyz_n[3]);

    /* ------------------------------------------------------------------------
     * Small helpers
     * ------------------------------------------------------------------------
     */

    /** @brief Fill a 3x3 skew-symmetric cross-product matrix from a vector.
     *
     * Such that [v]_x * w = v x w.
     *
     * @param[in] v Input 3x1 vector.
     * @param[out] M Output 3x3 matrix (column-major). */
    void ins_cross_matrix(const float v[3], float M[9]);

    /** @brief Vector cross product: out = a x b.
     *  @param[in] a First vector (3x1).
     *  @param[in] b Second vector (3x1).
     *  @param[out] out Result (3x1). */
    void ins_cross(const float a[3], const float b[3], float out[3]);

    /** @brief Signed smallest difference (a - b) wrapped to [-pi, pi].
     *  @param[in] a First angle [rad].
     *  @param[in] b Second angle [rad].
     *  @return (a - b) wrapped into [-pi, pi]. */
    float ins_angle_diff(float a, float b);

    /** @brief Wrap an angle to [-pi, pi], single-step (input must be in
     *  (-3pi, 3pi) -- use ins_angle_diff() for unbounded inputs). Cheaper
     *  than ins_angle_diff() for the common hot-path case where the input
     *  is already known to be close to the wrapped range (e.g. a small
     *  correction added to an already-wrapped angle).
     *  @param[in] a Angle [rad], must be in (-3pi, 3pi).
     *  @return a wrapped into [-pi, pi]. */
    float ins_wrap_pi_bounded(float a);

    /** @brief True iff all three components are finite (not NaN/Inf).
     *  @param[in] v Input 3x1 vector.
     *  @return true if every component is finite, false otherwise. */
    bool ins_vec3_finite(const float v[3]);

    /** @brief International Standard Atmosphere (ISA) altitude from static
     *  pressure (REQ-BARO-004/REQ-NAV-054). Shared by ins and baro_alt so
     *  both filters agree on the same vertical datum from the same raw
     *  pressure sample.
     *  @param[in] pressure_pa Static pressure [Pa].
     *  @return Altitude above the ISA sea-level pressure reference [m]. */
    float ins_isa_altitude_from_pressure(float pressure_pa);

    /** @brief Plausibility bounds on a raw static-pressure sample (finite
     *  and within #INS_ISA_PRESSURE_MIN_PA .. #INS_ISA_PRESSURE_MAX_PA),
     *  shared by ins and baro_alt (REQ-BARO-004/REQ-NAV-054).
     *  @param[in] pressure_pa Static pressure [Pa].
     *  @return true if pressure_pa is finite and within bounds. */
    bool ins_isa_pressure_plausible(float pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* GEODETIC_TOOLBOX_H */
/** @} */
