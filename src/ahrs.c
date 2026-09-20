/** @file ahrs.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Attitude (and heading) reference system Kalman Filter.
 *
 * Implementation: error-state formulation correcting a quaternion (a->q) and a
 * gyroscope bias vector (a->gyr_bias_rps). The error state is an attitude
 * misalignment in the n-frame (psi-angle model) plus the gyro bias error:
 *
 *   R_nominal = (I + [datt]_x) * R_true
 *
 * so a correction is applied as a left (n-frame side) quaternion multiplication
 * q <- q(-datt) * q (ins_quat_small_angle_correction) and biases are corrected
 * as bias <- bias - dbias. Residuals are z = measured - predicted, which under
 * this convention gives:
 *
 *   accelerometer: z = f_b + R' * g_n           =  R' * [g_n]_x * datt
 *   magnetic yaw:  z = wrap(yaw_mag - yaw_nom)  = -dyaw
 *
 * The covariance is kept as a UDU factorisation (P = U * diag(d) * U');
 * prediction and fusion use the Thornton/Bierman routines from KFCore
 * (kalman_udu.h), the same backend as ins. The robust update scales the
 * measurement covariance instead of dropping an outlier row (Chang 2014), so a
 * persistent-offset reference cannot deadlock.
 */

#include <math.h>
#include <string.h>
#include <stddef.h>

#include "ahrs.h"
#include "geodetic_toolbox.h"
#include "magnetic_model.h"
#include "sensor_defaults.h"
#include "linalg.h"
#include "kalman_udu.h"
#include "log.h"

/* ============================================================================
 * Local defines
 * ============================================================================
 */

#define AHRS_US_PER_SEC (1000000LL)

/* Diagnostics only (see log.h): AHRS_MODE_AHRS yaw drifts unbounded without
   magnetometer aiding. Warn once the yaw stddev has grown past this floor AND
   the last successful mag fusion is older than the gap below. */
#define AHRS_LOG_YAW_STDDEV_WARN_DEG (10.0f)
#define AHRS_LOG_MAG_GAP_WARN_SEC    (10.0f)
#define AHRS_LOG_MAG_GAP_REPEAT_SEC  (60.0f)

/* Runaway detectors (see log.h): same tumbling-window pattern as ins.c's,
   applied to ahrs's own yaw stddev (AHRS_MODE_AHRS only) and gyro bias. */
#define AHRS_LOG_RUNAWAY_WINDOW_SEC   (10.0f)
#define AHRS_LOG_YAW_RUNAWAY_DEG      (10.0f)
#define AHRS_LOG_GYR_BIAS_RUNAWAY_DPS (1.0f)

/* Bias sanity bound (see log.h): same rationale as ins.c's. */
#define AHRS_LOG_GYR_BIAS_SANITY_DPS    (10.0f)
#define AHRS_LOG_BIAS_SANITY_REPEAT_SEC (60.0f)

/* Gravity [m/s^2] (this code is not safe for other planets). */
#define AHRS_GRAVITY INS_GRAVITY_NOMINAL

/* Covariance-prediction cadence [s], used when cfg.kalman_update_dt_sec is 0.
 * The attitude integration runs at the full IMU rate, the error-state
 * covariance need not (Wendel, 2nd ed., ch. 8.2.1: "typically 10 Hz"). */
#define AHRS_DEFAULT_COVAR_UPDATE_SEC (1.0f / 20.0f)

/* Don't integrate the attitude over longer time periods. */
#define AHRS_MAX_DT_SEC (0.2f)

/* Config defaults. Gyro terms are shared with ins.c (sensor_defaults.h,
 * "low-cost consumer MEMS IMU" defaults): ins and the AHRS filters run against
 * the same physical gyro in nav_suite, so their "unknown sensor" fallback must
 * agree. */
#define AHRS_DEFAULT_GYR_NOISE_PSD INS_DEFAULT_GYR_ARW_RPS_SQRTHZ
#define AHRS_DEFAULT_GYR_BIAS_RW   INS_DEFAULT_GYR_BIAS_RW_RPS_SQRTS
/* Deliberately NOT the ins VRW density: ahrs.c fuses the accelerometer as a
 * discrete leveling correction (not a continuous strapdown input), and this
 * stddev doubles as the maneuver/vibration-rejection margin. Lowering it to
 * sensor-only white noise would make leveling overconfident during real
 * acceleration phases. */
#define AHRS_DEFAULT_ACC_NOISE_MPS2  (0.25f)
#define AHRS_DEFAULT_ACC_FREQ_HZ     (5.0f)
#define AHRS_DEFAULT_GRAVITY_PENALTY (8.0f)
/* Hard gravity-magnitude reject gate: a low-passed specific force whose
 * magnitude is more than this far from g is not a leveling reference. */
#define AHRS_DEFAULT_ACC_REJECT_GRAVITY (2.0f)
#define AHRS_DEFAULT_CHI2_ACC           INS_DEFAULT_CHI2_95_1DOF /* chi2inv(0.95, 1), per scalar row */
#define AHRS_DEFAULT_ACC_CUTOFF_HZ      (10.0f)
#define AHRS_DEFAULT_BIAS_STDDEV_XY     DEG2RAD(1.0f)
#define AHRS_DEFAULT_BIAS_STDDEV_Z      DEG2RAD(1.5f)
#define AHRS_DEFAULT_MAG_YAW_STDDEV     DEG2RAD(10.0f)
#define AHRS_DEFAULT_MAG_FREQ_HZ        (5.0f)
#define AHRS_DEFAULT_CHI2_MAG           INS_DEFAULT_CHI2_95_1DOF
/* Reject/downweight the magnetometer once the measured field magnitude
 * deviates by more than this fraction from the WMM total-field model.
 * Shared with ins.c's own mag field gate (sensor_defaults.h). */
#define AHRS_DEFAULT_MAG_FIELD_TOL INS_DEFAULT_MAG_FIELD_TOLERANCE
/* Zero-rotation update std.-dev. */
#define AHRS_DEFAULT_ZERO_ROT_STDDEV DEG2RAD(0.1f)

/* Velocity-blind auto-ZARU fallback defaults: match ins's own auto-ZUPT/ZARU
 * thresholds and dwell time (sensor_defaults.h), since both run against the
 * same physical IMU. The configurable gyro/accel fields are loose magnitude
 * sanity bounds, the primary criterion is the window variance. */
#define AHRS_DEFAULT_AUTO_ZARU_STATIC_GYR INS_DEFAULT_STATIC_GYR_BOUND_RPS
#define AHRS_DEFAULT_AUTO_ZARU_STATIC_ACC INS_DEFAULT_STATIC_ACC_BOUND_MPS2
#define AHRS_DEFAULT_AUTO_ZARU_DWELL_SEC  INS_DEFAULT_STATIC_DWELL_SEC

/* Attitude-precision restart watchdog (REQ-AHRS-023) defaults: an attitude
 * 1-sigma this large means the filter has effectively lost the axis. Yaw is
 * looser (heading is the weakest reference), roll/pitch are normally pinned by
 * leveling. Unit: rad */
#define AHRS_DEFAULT_RESTART_STDDEV_XY  DEG2RAD(10.0f)
#define AHRS_DEFAULT_RESTART_STDDEV_Z   DEG2RAD(90.0f)
#define AHRS_DEFAULT_RESTART_WARMUP_SEC (10.0f)

/* Yaw 1-sigma for "heading unknown" (REQ-AHRS-014), and the fraction of the
   yaw restart threshold it is capped at while that watchdog is armed. */
#define AHRS_YAW_UNKNOWN_STDDEV         (3.14159265f)
#define AHRS_YAW_UNKNOWN_RESTART_MARGIN (0.95f)

/* ============================================================================
 * Small helpers
 * ============================================================================
 */

static inline float time_diff_sec(ahrs_time_us_t later, ahrs_time_us_t earlier)
{
    return ((float)(later - earlier) * (1.0f / AHRS_US_PER_SEC));
}

static inline float qsquare(float x) { return x * x; }

/* ============================================================================
 * Config resolution (0 -> default)
 * ============================================================================
 */

static void ahrs_resolve_config(const ahrs_config_t* in, ahrs_config_t* out)
{
    *out = *in;
    int i;
    for (i = 0; i < 3; ++i)
    {
        if (out->gyr_bias_init_stddev_rps[i] <= 0.0f)
        {
            out->gyr_bias_init_stddev_rps[i] =
                (i == 2) ? AHRS_DEFAULT_BIAS_STDDEV_Z : AHRS_DEFAULT_BIAS_STDDEV_XY;
        }
    }
    if (out->kalman_update_dt_sec <= 0.0f)
        out->kalman_update_dt_sec = AHRS_DEFAULT_COVAR_UPDATE_SEC;
    if (out->gyr_noise_psd <= 0.0f) out->gyr_noise_psd = AHRS_DEFAULT_GYR_NOISE_PSD;
    /* Negative is clamped to 0 first, so the value is guaranteed >= 0 here.
       "<= 0.0f" is equivalent to "== 0.0f" but keeps the file -Wfloat-equal
       clean, per coding_style.md. */
    if (out->gyr_bias_rw < 0.0f) out->gyr_bias_rw = 0.0f;
    if (out->gyr_bias_rw <= 0.0f) out->gyr_bias_rw = AHRS_DEFAULT_GYR_BIAS_RW;
    if (out->acc_noise_mps2 <= 0.0f) out->acc_noise_mps2 = AHRS_DEFAULT_ACC_NOISE_MPS2;
    if (out->acc_freq_hz <= 0.0f) out->acc_freq_hz = AHRS_DEFAULT_ACC_FREQ_HZ;
    if (out->gravity_diff_penalty < 0.0f)
        out->gravity_diff_penalty = 0.0f;
    else if (out->gravity_diff_penalty <= 0.0f)
        out->gravity_diff_penalty = AHRS_DEFAULT_GRAVITY_PENALTY;
    /* Hard gravity gate: < 0 disables it, 0 -> default (same negative-then-
       zero idiom as gravity_diff_penalty, -Wfloat-equal clean). */
    if (out->acc_reject_gravity_mps2 < 0.0f)
        out->acc_reject_gravity_mps2 = 0.0f;
    else if (out->acc_reject_gravity_mps2 <= 0.0f)
        out->acc_reject_gravity_mps2 = AHRS_DEFAULT_ACC_REJECT_GRAVITY;
    if (out->chi2_threshold <= 0.0f) out->chi2_threshold = AHRS_DEFAULT_CHI2_ACC;
    if (out->acc_cutoff_freq_hz <= 0.0f) out->acc_cutoff_freq_hz = AHRS_DEFAULT_ACC_CUTOFF_HZ;
    if (out->mag_yaw_stddev_rad <= 0.0f) out->mag_yaw_stddev_rad = AHRS_DEFAULT_MAG_YAW_STDDEV;
    if (out->mag_freq_hz <= 0.0f) out->mag_freq_hz = AHRS_DEFAULT_MAG_FREQ_HZ;
    if (out->mag_chi2_threshold <= 0.0f) out->mag_chi2_threshold = AHRS_DEFAULT_CHI2_MAG;
    if (out->mag_field_tolerance <= 0.0f) out->mag_field_tolerance = AHRS_DEFAULT_MAG_FIELD_TOL;
    if (out->zero_rot_stddev_rps <= 0.0f) out->zero_rot_stddev_rps = AHRS_DEFAULT_ZERO_ROT_STDDEV;
    if (out->auto_zaru_static_gyr_rps <= 0.0f)
        out->auto_zaru_static_gyr_rps = AHRS_DEFAULT_AUTO_ZARU_STATIC_GYR;
    if (out->auto_zaru_static_acc_mps2 <= 0.0f)
        out->auto_zaru_static_acc_mps2 = AHRS_DEFAULT_AUTO_ZARU_STATIC_ACC;
    if (out->auto_zaru_static_gyr_stddev_rps <= 0.0f)
        out->auto_zaru_static_gyr_stddev_rps = INS_DEFAULT_STATIC_GYR_STDDEV_RPS;
    if (out->auto_zaru_static_acc_stddev_mps2 <= 0.0f)
        out->auto_zaru_static_acc_stddev_mps2 = INS_DEFAULT_STATIC_ACC_STDDEV_MPS2;
    if (out->auto_zaru_dwell_sec <= 0.0f)
        out->auto_zaru_dwell_sec = AHRS_DEFAULT_AUTO_ZARU_DWELL_SEC;
    /* Precision-restart thresholds: < 0 leaves that axis unchecked,
       0 -> default (roll/pitch vs. yaw). */
    for (i = 0; i < 3; ++i)
    {
        if (out->restart_att_stddev_rad[i] < 0.0f)
            out->restart_att_stddev_rad[i] = 0.0f;
        else if (out->restart_att_stddev_rad[i] <= 0.0f)
            out->restart_att_stddev_rad[i] =
                (i == 2) ? AHRS_DEFAULT_RESTART_STDDEV_Z : AHRS_DEFAULT_RESTART_STDDEV_XY;
    }
    if (out->restart_warmup_sec <= 0.0f) out->restart_warmup_sec = AHRS_DEFAULT_RESTART_WARMUP_SEC;
}

/* Dump every effective (post 0 -> default resolution) ahrs_config_t parameter
 * the filter will actually run with. Unlike ins.c's equivalent, every field is
 * already resolved by ahrs_resolve_config, so this reads a->cfg directly. */
static void ahrs_log_effective_config(const ahrs_t* a)
{
#if LOG_LEVEL >= LOG_LEVEL_INFO
    /* Every statement below exists only to feed LOG_INFO; below the
       compile-time ceiling LOG_INFO expands to nothing (log.h) and the locals
       here would be flagged -Wunused-variable, so the whole body is compiled
       out with it. */
    const ahrs_config_t* cfg = &a->cfg;

    LOG_INFO("ahrs: mode %s (%d states), chi2 downweighting %s",
             (cfg->mode == AHRS_MODE_AHRS) ? "AHRS" : "ARS", a->n,
             cfg->chi2_disable ? "disabled" : "enabled");
    LOG_INFO("ahrs: gyro bias init stddev=[%.2f %.2f %.2f] deg/s, noise psd %.4f deg/s/sqrt(Hz), "
             "bias random walk %.2e (rad/s^2)/sqrt(Hz)",
             (double)(RAD2DEG(cfg->gyr_bias_init_stddev_rps[0])),
             (double)(RAD2DEG(cfg->gyr_bias_init_stddev_rps[1])),
             (double)(RAD2DEG(cfg->gyr_bias_init_stddev_rps[2])),
             (double)(RAD2DEG(cfg->gyr_noise_psd)), (double)cfg->gyr_bias_rw);
    LOG_INFO("ahrs: accelerometer: noise %.3f m/s^2, max rate %.1f Hz, cutoff %.1f Hz, "
             "gravity penalty %.1f stddev/(m/s^2)%s, chi2 threshold %.2f",
             (double)cfg->acc_noise_mps2, (double)cfg->acc_freq_hz, (double)cfg->acc_cutoff_freq_hz,
             (double)cfg->gravity_diff_penalty,
             (cfg->acc_reject_gravity_mps2 > 0.0f) ? "" : " (hard gravity gate disabled)",
             (double)cfg->chi2_threshold);
    if (cfg->acc_reject_gravity_mps2 > 0.0f)
    {
        LOG_INFO("ahrs: accelerometer hard-reject gate: |acc|-g > %.2f m/s^2",
                 (double)cfg->acc_reject_gravity_mps2);
    }
    LOG_INFO("ahrs: cadence: covariance update %.3f s (%.1f Hz)", (double)cfg->kalman_update_dt_sec,
             (double)(1.0f / cfg->kalman_update_dt_sec));
    if (cfg->mode == AHRS_MODE_AHRS)
    {
        LOG_INFO("ahrs: magnetometer: yaw stddev %.1f deg, max rate %.1f Hz, chi2 threshold %.2f, "
                 "field-strength gate %s (tol %.0f%%)",
                 (double)(RAD2DEG(cfg->mag_yaw_stddev_rad)), (double)cfg->mag_freq_hz,
                 (double)cfg->mag_chi2_threshold,
                 cfg->mag_field_check_enable ? "enabled" : "disabled",
                 (double)(cfg->mag_field_tolerance * 100.0f));
    }
    LOG_INFO("ahrs: zero-rotation update stddev %.4f deg/s",
             (double)(RAD2DEG(cfg->zero_rot_stddev_rps)));
    if (!cfg->auto_zaru_disable)
    {
        LOG_INFO("ahrs: auto-zaru fallback: enabled, window stddev gyr %.3f deg/s / acc %.2f "
                 "m/s^2, magnitude bounds gyr %.3f deg/s / acc %.2f m/s^2, dwell %.2f s",
                 (double)(RAD2DEG(cfg->auto_zaru_static_gyr_stddev_rps)),
                 (double)cfg->auto_zaru_static_acc_stddev_mps2,
                 (double)(RAD2DEG(cfg->auto_zaru_static_gyr_rps)),
                 (double)cfg->auto_zaru_static_acc_mps2, (double)cfg->auto_zaru_dwell_sec);
    }
    else { LOG_INFO("ahrs: auto-zaru fallback: disabled"); }
    if (cfg->precision_restart_disable) { LOG_INFO("ahrs: precision-restart watchdog: disabled"); }
    else
    {
        LOG_INFO("ahrs: precision-restart watchdog: enabled, threshold roll/pitch/yaw=[%.0f %.0f "
                 "%.0f] deg (< 0 = axis not checked), warm-up %.1f s",
                 (double)(RAD2DEG(cfg->restart_att_stddev_rad[0])),
                 (double)(RAD2DEG(cfg->restart_att_stddev_rad[1])),
                 (double)(RAD2DEG(cfg->restart_att_stddev_rad[2])),
                 (double)cfg->restart_warmup_sec);
    }
#else
    (void)a;
#endif
}

/* ============================================================================
 * Public API: init
 * ============================================================================
 */

/* @satisfies REQ-AHRS-001 REQ-AHRS-008 */
int ahrs_init(ahrs_t* a, const ahrs_config_t* cfg, ahrs_time_us_t t)
{
    if (a == NULL || cfg == NULL) { return -1; }
    if (cfg->mode != AHRS_MODE_ARS && cfg->mode != AHRS_MODE_AHRS) { return -1; }
    /* Attitude stddevs are mandatory (only positive values accepted;
       NaN fails the > 0 test as well). The yaw entry is only needed in
       AHRS mode. */
    const int n_rpy = (cfg->mode == AHRS_MODE_AHRS) ? 3 : 2;
    int       i;
    for (i = 0; i < n_rpy; ++i)
    {
        if (!(cfg->rpy_init_stddev_rad[i] > 0.0f) || !isfinite(cfg->rpy_init_stddev_rad[i]))
        {
            return -1;
        }
    }
    if (!ins_vec3_finite(cfg->rpy_init_rad) || !ins_vec3_finite(cfg->gyr_bias_init_rps))
    {
        return -1;
    }

    memset(a, 0, sizeof(*a));
    ahrs_resolve_config(cfg, &a->cfg);
    a->n = (a->cfg.mode == AHRS_MODE_AHRS) ? 6 : 5;
    /* No position known yet, so nothing says the magnetometer is unusable. */
    a->mag_heading_usable = true;

    ins_quat_from_rpy(cfg->rpy_init_rad[0], cfg->rpy_init_rad[1], cfg->rpy_init_rad[2], a->q);
    a->gyr_bias_rps[0] = cfg->gyr_bias_init_rps[0];
    a->gyr_bias_rps[1] = cfg->gyr_bias_init_rps[1];
    a->gyr_bias_rps[2] = cfg->gyr_bias_init_rps[2];

    /* Diagonal initial covariance (attitude block, then gyro bias):
       as UDU factors this is simply U = I, d = variances. */
    const int n   = a->n;
    const int off = n - 3; /* gyro bias offset in the error state */
    mateye(a->U, n);
    for (i = 0; i < off; ++i) { a->d[i] = qsquare(a->cfg.rpy_init_stddev_rad[i]); }
    for (i = 0; i < 3; ++i) { a->d[off + i] = qsquare(a->cfg.gyr_bias_init_stddev_rps[i]); }

    /* Initialize the accelerometer low pass with the expected static
       measurement: f_b = -R' * g_n. */
    float R[9];
    ins_quat_to_rotmat(a->q, R);
    for (i = 0; i < 3; ++i) { a->acc_lowpass_mps2[i] = -AHRS_GRAVITY * MAT_ELEM(R, 2, i, 3, 3); }

    a->t_init                 = t;
    a->t_last_gyr             = t;
    a->t_last_cov_predict     = t;
    a->t_last_acc_fusion      = t;
    a->t_last_mag_fusion      = t;
    a->t_last_zero_rot_fusion = t;
    a->is_initialized         = true;
    /* Overconfidence watchdog running-min starts "unseen" (REQ-AHRS-020);
       memset zeroed it, which would pin the minimum at 0 forever. */
    a->min_att_stddev_deg = INFINITY;
    if (a->cfg.mode == AHRS_MODE_AHRS)
    {
        LOG_INFO("ahrs: AHRS filter started, rpy=[%.1f %.1f %.1f] deg, rpy stddev=[%.2f %.2f "
                 "%.2f] deg",
                 (double)(RAD2DEG(cfg->rpy_init_rad[0])), (double)(RAD2DEG(cfg->rpy_init_rad[1])),
                 (double)(RAD2DEG(cfg->rpy_init_rad[2])),
                 (double)(RAD2DEG(a->cfg.rpy_init_stddev_rad[0])),
                 (double)(RAD2DEG(a->cfg.rpy_init_stddev_rad[1])),
                 (double)(RAD2DEG(a->cfg.rpy_init_stddev_rad[2])));
    }
    else
    {
        LOG_INFO("ahrs: ARS filter started, rp=[%.1f %.1f] deg, rp stddev=[%.2f %.2f] deg",
                 (double)(RAD2DEG(cfg->rpy_init_rad[0])), (double)(RAD2DEG(cfg->rpy_init_rad[1])),
                 (double)(RAD2DEG(a->cfg.rpy_init_stddev_rad[0])),
                 (double)(RAD2DEG(a->cfg.rpy_init_stddev_rad[1])));
    }
    ahrs_log_effective_config(a);
    return 0;
}

/* ============================================================================
 * Covariance prediction
 * Source: Wendel, equation 10.9
 *
 * Phi = I + F*dt with F = [ 0  -R[0:col,:] ]   (col = 2 in ARS mode,
 *                         [ 0       0      ]    col = 3 in AHRS mode)
 * and process noise in noise-input form:
 *   G = [ -R[0:col,:]  0 ],  Q = [gyr_noise_psd^2 * dt (x3),
 *       [      0       I ]        gyr_bias_rw^2   * dt (x3)]
 * ============================================================================
 */

/* @satisfies REQ-AHRS-002 REQ-AHRS-021 REQ-AHRS-025 */
static void ahrs_predict_covariance(ahrs_t* a, float dt_sec, float* phi_out)
{
    const int n   = a->n;
    const int col = n - 3;
    int       i, j;

    float R[9];
    ins_quat_to_rotmat(a->q, R);

    float Phi[AHRS_UNKNOWNS_MAX * AHRS_UNKNOWNS_MAX];
    float G[AHRS_UNKNOWNS_MAX * 6];
    float Q[6];
    mateye(Phi, n);
    memset(G, 0, sizeof(G[0]) * (size_t)(n * 6));

    for (i = 0; i < col; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            MAT_ELEM(Phi, i, col + j, n, n) = -MAT_ELEM(R, i, j, 3, 3) * dt_sec;
            MAT_ELEM(G, i, j, n, 6)         = -MAT_ELEM(R, i, j, 3, 3);
        }
    }
    for (i = 0; i < 3; ++i)
    {
        MAT_ELEM(G, col + i, 3 + i, n, 6) = 1.0f;
        Q[i]                              = qsquare(a->cfg.gyr_noise_psd) * dt_sec;
        Q[3 + i]                          = qsquare(a->cfg.gyr_bias_rw) * dt_sec;
    }

    if (phi_out != NULL) { memcpy(phi_out, Phi, sizeof(Phi[0]) * (size_t)(n * n)); }

    /* No state prediction (error state is implicitly zero). */
    kalman_udu_predict(NULL, a->U, a->d, Phi, G, Q, n, 6);
}

/* ============================================================================
 * Measurement fusion
 * ============================================================================
 */

/* Fuse m_count measurements (z = measured - predicted) with diagonal covariance
 * R (m x m) and transposed measurement matrix Ht (n x m, column i holds row i
 * of H). The robust Bierman update tests each scalar measurement row against
 * chi2_threshold (Mahalanobis distance squared) and downweights it by scaling
 * its variance instead of dropping it, so a persistent-offset reference cannot
 * deadlock. On success the correction is applied to the nominal state:
 *   gyr_bias -= dx[bias block], q <- q(-dx[att block]) * q.
 * Returns 0 on success. */
/* Diagnostic-only re-check of kalman_udu's own chi2 gate, used to count
 * downweighted fusions (REQ-AHRS-019) without affecting the fusion. See
 * ins_fuse_is_outlier (ins.c) for the same pattern and caveats. */
static bool ahrs_fuse_is_outlier(const ahrs_t* a, const float* z, const float* R, const float* Ht,
                                 int m_count, float chi2_threshold)
{
    if (!(chi2_threshold > 0.0f)) { return false; }
    int i, j;
    for (i = 0; i < m_count; ++i)
    {
        float tmp[AHRS_UNKNOWNS_MAX];
        matmul("N", "N", 1, a->n, a->n, 1.0f, Ht + (ptrdiff_t)i * a->n, a->U, 0.0f, tmp);
        float HPHT = 0.0f;
        for (j = 0; j < a->n; ++j) { HPHT += tmp[j] * tmp[j] * a->d[j]; }
        const float Rv = MAT_ELEM(R, i, i, m_count, m_count);
        const float s  = HPHT + Rv;
        const float dz = z[i];
        if (dz * dz > chi2_threshold * s) { return true; }
    }
    return false;
}

/* @satisfies REQ-AHRS-005 REQ-AHRS-007 REQ-AHRS-018 REQ-AHRS-019 */
static int ahrs_fuse(ahrs_t* a, const float* z, const float* R, const float* Ht, int m_count,
                     float chi2_threshold)
{
    /* Global override (REQ-AHRS-018): 0.0f makes kalman_udu skip the chi2 test
       entirely, same as the ZARU call already passes literally. The measurement
       is then fused at its nominal variance. */
    if (a->cfg.chi2_disable) { chi2_threshold = 0.0f; }

    const int n = a->n;
    int       i;

    if (ahrs_fuse_is_outlier(a, z, R, Ht, m_count, chi2_threshold)) { a->n_downweighted++; }

    float dx[AHRS_UNKNOWNS_MAX];
    memset(dx, 0, sizeof(dx));
    if (kalman_udu(dx, a->U, a->d, z, R, Ht, n, m_count, chi2_threshold,
                   1 /* downweight outliers */) != 0)
    {
        a->n_fuse_fail++;
        return -1;
    }

    /* Correct the nominal state. */
    const int off = n - 3;
    for (i = 0; i < 3; ++i) { a->gyr_bias_rps[i] -= dx[off + i]; }
    const float datt[3] = {dx[0], dx[1], (off == 3) ? dx[2] : 0.0f};
    float       q_new[4];
    ins_quat_small_angle_correction(a->q, datt, q_new);
    memcpy(a->q, q_new, sizeof(q_new));
    return 0;
}

/* Accelerometer leveling: z = f_b_lowpass + R' * g_n, H = R' * [g_n]_x
 * (yaw column of [g_n]_x is zero, so only roll/pitch are observed). */
/* @satisfies REQ-AHRS-003 REQ-AHRS-004 REQ-AHRS-022 */
static void ahrs_fuse_acc(ahrs_t* a)
{
    const int n = a->n;
    int       i;

    float R_b_to_n[9];
    ins_quat_to_rotmat(a->q, R_b_to_n);

    const float acc_len = SQRTF(qsquare(a->acc_lowpass_mps2[0]) + qsquare(a->acc_lowpass_mps2[1]) +
                                qsquare(a->acc_lowpass_mps2[2]));
    const float s       = fabsf(acc_len - AHRS_GRAVITY);

    /* Hard gravity-magnitude gate (REQ-AHRS-022): a specific force this far
       from gravity is a maneuver/shock, not a leveling reference, so the update
       is dropped outright. The fusion clock was already advanced by the caller,
       so the throttle pace is kept, and the deviation is transient enough that
       a hard drop cannot deadlock (cf. REQ-AHRS-007). */
    if (a->cfg.acc_reject_gravity_mps2 > 0.0f && s > a->cfg.acc_reject_gravity_mps2)
    {
        a->n_acc_rejected++;
        return;
    }

    /* Limit the accelerometer influence during (sub-threshold) acceleration
       phases: inflate the noise proportional to | |f| - g |, capped at g.
       Source: Jay Farrell, Aided Navigation, ch. 10.5.4, p. 366. */
    float stddev = a->cfg.acc_noise_mps2 + s * a->cfg.gravity_diff_penalty;
    if (stddev > AHRS_GRAVITY) { stddev = AHRS_GRAVITY; }

    float z[3];
    float R[3 * 3];
    float Ht[AHRS_UNKNOWNS_MAX * 3];
    memset(R, 0, sizeof(R));
    memset(Ht, 0, sizeof(Ht[0]) * (size_t)(n * 3));
    for (i = 0; i < 3; ++i)
    {
        /* Residual: measured f_b vs. predicted -R' * g_n. */
        z[i] = a->acc_lowpass_mps2[i] + AHRS_GRAVITY * MAT_ELEM(R_b_to_n, 2, i, 3, 3);
        MAT_ELEM(R, i, i, 3, 3) = qsquare(stddev);
        /* H[:,0] =  g * (row 1 of R_b_to_n)'
           H[:,1] = -g * (row 0 of R_b_to_n)'   (Ht holds H rows as cols) */
        MAT_ELEM(Ht, 0, i, n, 3) = AHRS_GRAVITY * MAT_ELEM(R_b_to_n, 1, i, 3, 3);
        MAT_ELEM(Ht, 1, i, n, 3) = -AHRS_GRAVITY * MAT_ELEM(R_b_to_n, 0, i, 3, 3);
    }

    (void)ahrs_fuse(a, z, R, Ht, 3, a->cfg.chi2_threshold);
}

/* De-tilt the body-frame magnetic field with the given roll/pitch:
 * (hx, hy) = horizontal components of Ry(pitch) * Rx(roll) * mag_b.
 * With the n-frame field pointing to magnetic north (east component zero
 * by definition of the reference): h = [mH*cos(yaw), -mH*sin(yaw), mD],
 * so yaw = atan2(-hy, hx). */
static void ahrs_mag_detilt(const float mag_b[3], float roll_rad, float pitch_rad, float* hx,
                            float* hy)
{
    const float cr = cosf(roll_rad), sr = sinf(roll_rad);
    const float ct = cosf(pitch_rad), st = sinf(pitch_rad);
    const float ty = cr * mag_b[1] - sr * mag_b[2];
    const float tz = sr * mag_b[1] + cr * mag_b[2];
    *hx            = ct * mag_b[0] + st * tz;
    *hy            = ty;
}

/* Magnetometer heading (AHRS mode): scalar tilt-compensated heading
 * measurement. Residual z = wrap(yaw_mag - yaw_nominal) = -dyaw, so
 * H = [0 0 -1 0 0 0]. Only yaw is observed, roll/pitch stay untouched by
 * magnetic disturbances. Once a position is known the WMM declination turns the
 * magnetic heading into a true-north heading, and the measured field magnitude
 * is checked against the WMM total field: a gross deviation downweights the
 * sample instead of dropping it. */
/* @satisfies REQ-AHRS-006 REQ-AHRS-014 REQ-AHRS-015 */
static void ahrs_fuse_mag(ahrs_t* a, const float mag_b[3])
{
    const int n = a->n;

    /* Dip pole exclusion zone: the declination that would turn a magnetic
       heading into a true one is meaningless here (REQ-SYS-018). */
    if (!a->mag_heading_usable) { return; }

    /* cppcheck-suppress nullPointer
     * False positive: cppcheck's value-flow conflates the two branches in
     * ahrs_update() where mag_b may become NULL with the caller's
     * mag_b != NULL guard before this call; mag_b is always valid here. */
    /* cppcheck-suppress ctunullpointer */
    const float mag_norm2 = qsquare(mag_b[0]) + qsquare(mag_b[1]) + qsquare(mag_b[2]);
    if (mag_norm2 < 1e-12f)
    {
        LOG_INFO("ahrs: magnetometer fusion skipped, field too weak (|mag|^2 < 1e-12)");
        return; /* no usable field */
    }

    float R_b_to_n[9];
    ins_quat_to_rotmat(a->q, R_b_to_n);

    /* Yaw is ill-defined near pitch = +/-90 deg (gimbal lock) -> skip. */
    const float sp = -MAT_ELEM(R_b_to_n, 2, 0, 3, 3);
    if (sp > 0.99f || sp < -0.99f)
    {
        LOG_INFO("ahrs: magnetometer fusion skipped, pitch %.1f deg too close to +/-90 deg",
                 (double)RAD2DEG(asinf(sp)));
        return;
    }

    float roll, pitch, yaw_nom;
    ins_rotmat_to_rpy(R_b_to_n, &roll, &pitch, &yaw_nom);

    /* De-tilted horizontal field must be usable (relative to the total
       field: near a magnetic pole the horizontal component vanishes). */
    float hx, hy;
    ahrs_mag_detilt(mag_b, roll, pitch, &hx, &hy);
    if (hx * hx + hy * hy < 1e-4f * mag_norm2)
    {
        LOG_INFO("ahrs: magnetometer fusion skipped, horizontal field too weak");
        return;
    }

    /* Magnetic heading, corrected to true north by the WMM declination
       (zero until ahrs_set_position() is called). */
    const float yaw_mag = ins_wrap_pi_bounded(atan2f(-hy, hx) + a->declination_rad);

    float z = ins_wrap_pi_bounded(yaw_mag - yaw_nom);
    float R = qsquare(a->cfg.mag_yaw_stddev_rad);

    /* Field-strength disturbance gate (opt-in): downweight (inflate R) once the
       measured magnitude leaves the tolerance band around the WMM total field.
       Armed only when explicitly enabled AND a position is known. At the band
       edge the factor is 1 and grows quadratically. Requires the magnetometer
       in uT. */
    if (a->cfg.mag_field_check_enable && a->mag_field_expected_uT > 0.0f)
    {
        const float meas = SQRTF(mag_norm2);
        const float dev  = fabsf(meas - a->mag_field_expected_uT) / a->mag_field_expected_uT;
        if (dev > a->cfg.mag_field_tolerance)
        {
            const float r = dev / a->cfg.mag_field_tolerance;
            R *= r * r;
        }
    }

    float Ht[AHRS_UNKNOWNS_MAX];
    memset(Ht, 0, sizeof(Ht[0]) * (size_t)n);
    Ht[2] = -1.0f;

    if (ahrs_fuse(a, &z, &R, Ht, 1, a->cfg.mag_chi2_threshold) == 0 && !a->declination_applied)
    {
        /* Referenced to magnetic north until a declination is applied, see
           ahrs_set_position (REQ-AHRS-014). */
        a->yaw_on_magnetic_north = true;
    }
}

/* Zero-rotation update: with omega_b_nb == 0 the gyro should read just the gyro
 * bias. Earth rotation rate is NOT corrected here (~15 deg/h, far below what
 * this class of sensor/filter resolves), unlike ins_fuse_zero_rotation(). A
 * direct measurement of the gyro bias states, H = [0 0 (0) I_3]. The fused
 * measurement is ahrs_auto_zaru_detect's average over the current confirmed-
 * stillness run where available (zaru_gyr_sum/count, accumulated there, not
 * here -- so it already covers a real average by the time this can first
 * fire), otherwise the instantaneous sample: a manual trigger without an
 * auto-detected stillness run (detector disabled, or the caller knows better
 * than the static gate) has nothing to average, same fallback as
 * ins_fuse_zero_rotation(). chi2 is disabled (threshold 0): the trigger
 * itself is the gate, and downweighting the measurement meant to pull a
 * large initial bias error to truth would defeat the point. Rate-limited to
 * one fusion per covariance-prediction period. */
/* @satisfies REQ-AHRS-016 */
static void ahrs_fuse_zaru(ahrs_t* a, ahrs_time_us_t t, const float gyr_rps[3], bool trigger)
{
    if (!trigger) return;

    /* Same tolerated due test as the covariance throttle below, see
       INS_CADENCE_TOLERANCE. */
    if (!INS_CADENCE_DUE(time_diff_sec(t, a->t_last_zero_rot_fusion), a->cfg.kalman_update_dt_sec))
    {
        return;
    }

    float gyr_meas[3];
    if (a->zaru_gyr_count > 0)
    {
        const float inv_n = 1.0f / (float)a->zaru_gyr_count;
        gyr_meas[0]       = a->zaru_gyr_sum[0] * inv_n;
        gyr_meas[1]       = a->zaru_gyr_sum[1] * inv_n;
        gyr_meas[2]       = a->zaru_gyr_sum[2] * inv_n;
    }
    else
    {
        gyr_meas[0] = gyr_rps[0];
        gyr_meas[1] = gyr_rps[1];
        gyr_meas[2] = gyr_rps[2];
    }

    const int n   = a->n;
    const int off = n - 3; /* gyro bias offset in the error state */

    float z[3];
    float R[3 * 3];
    float Ht[AHRS_UNKNOWNS_MAX * 3];
    memset(R, 0, sizeof(R));
    memset(Ht, 0, sizeof(Ht[0]) * (size_t)(n * 3));
    int i;
    for (i = 0; i < 3; ++i)
    {
        z[i]                           = a->gyr_bias_rps[i] - gyr_meas[i];
        MAT_ELEM(R, i, i, 3, 3)        = qsquare(a->cfg.zero_rot_stddev_rps);
        MAT_ELEM(Ht, off + i, i, n, 3) = 1.0f;
    }

    if (ahrs_fuse(a, z, R, Ht, 3, 0.0f) == 0)
    {
        a->t_last_zero_rot_fusion = t;
        a->zaru_gyr_count         = 0;
    }
}

/* Primary stillness criterion of the auto-ZARU fallback: the per-axis sample
 * variance of the RAW IMU over a short tumbling window. Own copy of ins.c's
 * ins_static_variance_update (the modules are deliberately independent); see
 * there and sensor_defaults.h for why variance rather than magnitude. Doubly
 * important here: ahrs has no accelerometer bias state, so a magnitude-only
 * accel gate can never be escaped once a biased sensor trips it.
 *
 * @return true if the most recently completed window looked stationary. */
static bool ahrs_static_variance_update(ahrs_t* a, const float gyr_rps[3], const float acc_mps2[3],
                                        ahrs_time_us_t t)
{
    const float x[6] = {gyr_rps[0], gyr_rps[1], gyr_rps[2], acc_mps2[0], acc_mps2[1], acc_mps2[2]};
    int         i;

    if (a->static_var_count == 0)
    {
        for (i = 0; i < 6; ++i)
        {
            a->static_var_mean[i] = x[i];
            a->static_var_m2[i]   = 0.0f;
        }
        a->static_var_window_since = t;
        a->static_var_count        = 1;
        return a->static_var_ok;
    }

    a->static_var_count++;
    for (i = 0; i < 6; ++i)
    {
        const float delta = x[i] - a->static_var_mean[i];
        a->static_var_mean[i] += delta / (float)a->static_var_count;
        a->static_var_m2[i] += delta * (x[i] - a->static_var_mean[i]);
    }

    if (a->static_var_count < INS_DEFAULT_STATIC_VAR_MIN_SAMPLES) { return a->static_var_ok; }
    if (time_diff_sec(t, a->static_var_window_since) < INS_DEFAULT_STATIC_VAR_WINDOW_SEC)
    {
        return a->static_var_ok;
    }

    const float inv_dof = 1.0f / (3.0f * (float)(a->static_var_count - 1u));
    const float gyr_rms =
        SQRTF((a->static_var_m2[0] + a->static_var_m2[1] + a->static_var_m2[2]) * inv_dof);
    const float acc_rms =
        SQRTF((a->static_var_m2[3] + a->static_var_m2[4] + a->static_var_m2[5]) * inv_dof);
    a->static_var_ok = (gyr_rms <= a->cfg.auto_zaru_static_gyr_stddev_rps) &&
                       (acc_rms <= a->cfg.auto_zaru_static_acc_stddev_mps2);
    a->static_var_count = 0; /* start the next window */
    return a->static_var_ok;
}

/* Velocity-blind auto-ZARU fallback: the same criteria as ins's own
 * auto-ZUPT/ZARU detector MINUS the velocity check this filter cannot do, so it
 * also arms during genuine constant-velocity cruise. Returns true for the whole
 * remainder of the stillness run once dwelled, so ahrs_fuse_zaru's accumulator
 * can average over it. */
/* @satisfies REQ-AHRS-017 */
static bool ahrs_auto_zaru_detect(ahrs_t* a, const float gyr_rps[3], const float acc_mps2[3],
                                  ahrs_time_us_t t)
{
    if (a->cfg.auto_zaru_disable)
    {
        /* Self-heals every call while disabled: a caller flipping
           cfg.auto_zaru_disable at runtime must not have a dwell timer or a
           latched variance verdict survive into re-enablement. */
        a->auto_zaru_static_since = 0;
        a->auto_zaru_var_since    = 0;
        a->zaru_gyr_count         = 0;
        a->static_var_count       = 0;
        a->static_var_ok          = false;
        return false;
    }

    const float gyr_c[3] = {gyr_rps[0] - a->gyr_bias_rps[0], gyr_rps[1] - a->gyr_bias_rps[1],
                            gyr_rps[2] - a->gyr_bias_rps[2]};
    const float gyr_norm = SQRTF(qsquare(gyr_c[0]) + qsquare(gyr_c[1]) + qsquare(gyr_c[2]));
    const float acc_norm =
        SQRTF(qsquare(acc_mps2[0]) + qsquare(acc_mps2[1]) + qsquare(acc_mps2[2]));
    /* The window is fed unconditionally and its verdict applied after the
       dwell timer has started, not folded into is_static -- same latency
       reasoning as ins.c's ins_auto_zupt_detect, see there. */
    const bool var_static = ahrs_static_variance_update(a, gyr_rps, acc_mps2, t);
    const bool is_static  = (gyr_norm <= a->cfg.auto_zaru_static_gyr_rps) &&
                           (fabsf(acc_norm - AHRS_GRAVITY) <= a->cfg.auto_zaru_static_acc_mps2);

    if (!is_static)
    {
        a->auto_zaru_static_since = 0;
        a->auto_zaru_var_since    = 0;
        a->zaru_gyr_count         = 0;
        return false;
    }
    if (a->auto_zaru_static_since == 0) { a->auto_zaru_static_since = t; }
    if (!var_static)
    {
        a->auto_zaru_var_since = 0;
        a->zaru_gyr_count      = 0;
        return false;
    }
    if (a->auto_zaru_var_since == 0) { a->auto_zaru_var_since = t; }

    /* Accumulate the raw gyro over the confirmed-stillness run, same
       reasoning as ins.c's auto_zupt_gyr_sum: fed here, unconditionally,
       from the moment var_static first confirms it -- NOT only once
       ahrs_fuse_zaru's own trigger goes true, which is dwell-gated and
       would otherwise start the average empty right when the first fuse
       is allowed to happen. Consumed and cleared by ahrs_fuse_zaru. */
    if (a->zaru_gyr_count == 0) { memset(a->zaru_gyr_sum, 0, sizeof(a->zaru_gyr_sum)); }
    a->zaru_gyr_sum[0] += gyr_rps[0];
    a->zaru_gyr_sum[1] += gyr_rps[1];
    a->zaru_gyr_sum[2] += gyr_rps[2];
    a->zaru_gyr_count++;

    /* Anchored to auto_zaru_var_since, not auto_zaru_static_since: see the
       identical reasoning in ins.c's ins_auto_zupt_detect. Because the
       accumulation above already started at that same timestamp, by the
       time this first returns true zaru_gyr_sum/count already covers at
       least auto_zaru_dwell_sec of real samples. */
    const ahrs_time_us_t dwell_us =
        (ahrs_time_us_t)(a->cfg.auto_zaru_dwell_sec * (float)AHRS_US_PER_SEC);
    return (t - a->auto_zaru_var_since) >= dwell_us;
}

/* ============================================================================
 * Health check
 * ============================================================================
 */

static void ahrs_check_health(ahrs_t* a)
{
    const int n = a->n;
    int       i;
    for (i = 0; i < 4; ++i)
    {
        if (!isfinite(a->q[i]))
        {
            LOG_ERROR("ahrs: health check failed (non-finite quaternion), filter shut down");
            a->is_initialized = false;
            return;
        }
    }
    for (i = 0; i < n; ++i)
    {
        if (!isfinite(a->d[i]) || a->d[i] < 0.0f)
        {
            LOG_ERROR("ahrs: health check failed (non-finite/negative covariance), "
                      "filter shut down");
            a->is_initialized = false;
            return;
        }
    }
}

/* Overconfidence / covariance-collapse watchdog floor (REQ-AHRS-020): a
 * reported attitude 1-sigma below this is physically implausible for the
 * sensors this filter targets. Matches ins's INS_OVERCONF_ATT_STDDEV_DEG, kept
 * local so ahrs stays module-independent. Diagnostic only. */
#define AHRS_OVERCONF_ATT_STDDEV_DEG 1e-3f /* 0.001 deg */

/* Per-axis attitude error variance P_ii from the UDU factors
 * (P = U diag(d) U', U unit upper triangular):
 *   P_ii = d_i + sum_{k>i} U(i,k)^2 d_k. */
static float ahrs_att_var(const ahrs_t* a, int i)
{
    float p = a->d[i];
    int   k;
    for (k = i + 1; k < a->n; ++k)
    {
        p += MAT_ELEM(a->U, i, k, a->n, a->n) * MAT_ELEM(a->U, i, k, a->n, a->n) * a->d[k];
    }
    return p;
}

/* @satisfies REQ-AHRS-020 */
static void ahrs_check_overconfidence(ahrs_t* a)
{
    /* Tracked attitude error states: 0..(n-3)-1, roll/pitch (both modes)
       plus yaw in AHRS mode. The remaining 3 are the gyro bias. Smallest
       per-axis 1-sigma is the first channel to look "too good". */
    const int natt   = a->n - 3;
    float     minvar = INFINITY;
    int       i;
    for (i = 0; i < natt; ++i)
    {
        const float p = ahrs_att_var(a, i);
        if (p < minvar) { minvar = p; }
    }
    const float att = RAD2DEG(SQRTF(minvar));
    if (att < a->min_att_stddev_deg) { a->min_att_stddev_deg = att; }
    if (att < AHRS_OVERCONF_ATT_STDDEV_DEG)
    {
        if (!a->overconfident)
        {
            LOG_WARN("ahrs: covariance overconfidence detected (att %.2g deg 1-sigma) -- "
                     "possible covariance collapse",
                     (double)att);
        }
        a->overconfident = true;
        a->n_overconfident++;
    }
}

/* Attitude-precision restart watchdog: once past the warm-up, if any tracked
 * attitude axis' reported 1-sigma exceeds its configured threshold the estimate
 * is no longer trustworthy, so the filter is marked uninitialized (fail-safe,
 * same mechanism as the health check). In nav_suite that triggers an autonomous
 * re-bootstrap, a standalone caller must call ahrs_init() again. */
/* @satisfies REQ-AHRS-023 */
static void ahrs_check_precision(ahrs_t* a)
{
    if (a->cfg.precision_restart_disable) { return; }
    if (time_diff_sec(a->t_last_gyr, a->t_init) < a->cfg.restart_warmup_sec) { return; }

    const int natt = a->n - 3; /* roll/pitch (both modes) + yaw (AHRS) */
    int       i;
    for (i = 0; i < natt; ++i)
    {
        const float thr = a->cfg.restart_att_stddev_rad[i];
        if (thr > 0.0f && ahrs_att_var(a, i) > thr * thr)
        {
            a->is_initialized = false;
            a->n_restart++;
            LOG_WARN("ahrs: attitude precision watchdog tripped on axis %d "
                     "(stddev exceeds %.2g deg threshold), filter restart #%u",
                     i, (double)RAD2DEG(thr), (unsigned int)a->n_restart);
            return;
        }
    }
}

/* ============================================================================
 * Public API: update
 * ============================================================================
 */

/* @satisfies REQ-AHRS-010 REQ-AHRS-011 REQ-AHRS-025 */
int ahrs_predict_step(ahrs_t* a, ahrs_time_us_t t, const float gyr_rps[3], const float acc_mps2[3],
                      const float mag_b[3], bool zero_rotation_update, float* phi_out)
{
    if (!a->is_initialized) { return AHRS_EPOCH_DROPPED; }

    /* Cleared up front so a dropped epoch (non-finite input, backwards
       time) reports "no trigger" instead of the previous epoch's value. */
    a->last_zaru_trigger = false;

    /* Non-finite inputs (NaN/Inf) must not reach the math: every comparison
       with NaN is false, so the chi2/downweighting logic is blind to them and
       one corrupt sample would poison the state. Drop the epoch instead; the
       next epoch's dt spans the gap, which the MAX_DT gate handles. */
    if (!ins_vec3_finite(gyr_rps) || !ins_vec3_finite(acc_mps2))
    {
        a->n_invalid_input++;
        LOG_WARN("ahrs: non-finite gyro/accel input dropped (%u total since init)",
                 (unsigned int)a->n_invalid_input);
        a->step_ctx.active = false;
        return AHRS_EPOCH_DROPPED;
    }
    bool mag_valid = (mag_b != NULL);
    if (mag_valid && !ins_vec3_finite(mag_b))
    {
        a->n_invalid_input++;
        mag_valid = false; /* gyro/acc are fine: only drop the mag */
    }

    a->epoch++;

    const float dt_sec = time_diff_sec(t, a->t_last_gyr);
    a->t_last_gyr      = t;
    if (dt_sec < 0.0f)
    {
        /* Time jumped backwards: re-anchor all clocks, skip the epoch. */
        a->t_last_cov_predict     = t;
        a->t_last_acc_fusion      = t;
        a->t_last_mag_fusion      = t;
        a->t_last_zero_rot_fusion = t;
        a->zaru_gyr_count         = 0;
        a->auto_zaru_static_since = 0;
        a->step_ctx.active        = false;
        return AHRS_EPOCH_DROPPED;
    }

    /* PREDICTION STEP: the error dynamics are slow, so the covariance need not
     * propagate at the IMU rate (throttled to cfg.kalman_update_dt_sec). */
    int   status     = 0;
    float dt_cov_sec = time_diff_sec(t, a->t_last_cov_predict);
    /* Tolerated due test rather than a bare ">=", see INS_CADENCE_TOLERANCE:
       an epoch stream marginally below cfg.kalman_update_dt_sec would
       otherwise propagate on every second epoch only. */
    if (INS_CADENCE_DUE(dt_cov_sec, a->cfg.kalman_update_dt_sec))
    {
        a->t_last_cov_predict = t;
        if (dt_cov_sec > AHRS_MAX_DT_SEC) { dt_cov_sec = AHRS_MAX_DT_SEC; }
        ahrs_predict_covariance(a, dt_cov_sec, phi_out);
        status |= AHRS_EPOCH_COV_PROPAGATED;
    }

    /* Attitude integration with the bias-corrected angular rate, plus
     * accelerometer low pass (first order RC filter). */
    if (dt_sec > 0.0f && dt_sec < AHRS_MAX_DT_SEC)
    {
        const float omega[3] = {gyr_rps[0] - a->gyr_bias_rps[0], gyr_rps[1] - a->gyr_bias_rps[1],
                                gyr_rps[2] - a->gyr_bias_rps[2]};
        float       q_new[4];
        ins_quat_rotate(a->q, omega, dt_sec, q_new);
        memcpy(a->q, q_new, sizeof(q_new));

        const float tau   = 1.0f / (2.0f * (float)M_PI * a->cfg.acc_cutoff_freq_hz);
        const float alpha = dt_sec / (tau + dt_sec);
        int         i;
        for (i = 0; i < 3; ++i)
        {
            a->acc_lowpass_mps2[i] += alpha * (acc_mps2[i] - a->acc_lowpass_mps2[i]);
        }
    }

    /* Hand off to ahrs_correct_step(): the sanitized mag sample (nulled out if
       invalid) plus this epoch's gyro/accel/trigger, since ahrs_correct_step()
       cannot re-derive the timing decisions above afterwards. */
    a->step_ctx.t = t;
    memcpy(a->step_ctx.gyr_rps, gyr_rps, sizeof(a->step_ctx.gyr_rps));
    memcpy(a->step_ctx.acc_mps2, acc_mps2, sizeof(a->step_ctx.acc_mps2));
    a->step_ctx.mag_valid = mag_valid;
    if (mag_valid) { memcpy(a->step_ctx.mag_b, mag_b, sizeof(a->step_ctx.mag_b)); }
    a->step_ctx.zero_rotation_update = zero_rotation_update;
    a->step_ctx.active               = true;
    return status;
}

/* @satisfies REQ-AHRS-025 */
void ahrs_correct_step(ahrs_t* a)
{
    if (!a->step_ctx.active) return;
    a->step_ctx.active = false;

    const ahrs_time_us_t t                    = a->step_ctx.t;
    const float*         gyr_rps              = a->step_ctx.gyr_rps;
    const float*         acc_mps2             = a->step_ctx.acc_mps2;
    const float*         mag_b                = a->step_ctx.mag_valid ? a->step_ctx.mag_b : NULL;
    const bool           zero_rotation_update = a->step_ctx.zero_rotation_update;

    /* FUSION STEP FOR ACCELEROMETER (throttled to acc_freq_hz)
     * --------------------------------------------------------- */
    if (time_diff_sec(t, a->t_last_acc_fusion) >= 1.0f / a->cfg.acc_freq_hz)
    {
        a->t_last_acc_fusion = t;
        ahrs_fuse_acc(a);
    }

    /* FUSION STEP FOR MAGNETOMETER (AHRS mode, throttled to mag_freq_hz)
     * ------------------------------------------------------------------ */
    /* cppcheck-suppress knownConditionTrueFalse
     * False positive, see the matching suppression in ahrs_fuse_mag(). */
    if (a->cfg.mode == AHRS_MODE_AHRS && mag_b != NULL &&
        time_diff_sec(t, a->t_last_mag_fusion) >= 1.0f / a->cfg.mag_freq_hz)
    {
        a->t_last_mag_fusion = t;
        ahrs_fuse_mag(a, mag_b);
    }

    /* ZERO-ROTATION UPDATE (opt-in): the caller's trigger, OR'd with the
       velocity-blind auto-ZARU fallback if enabled (REQ-AHRS-017). */
    const bool auto_zaru = ahrs_auto_zaru_detect(a, gyr_rps, acc_mps2, t);
    a->last_zaru_trigger = zero_rotation_update || auto_zaru;
    ahrs_fuse_zaru(a, t, gyr_rps, a->last_zaru_trigger);

    ins_quat_normalize(a->q);
    ahrs_check_health(a);
    if (a->is_initialized) { ahrs_check_overconfidence(a); }
    if (a->is_initialized) { ahrs_check_precision(a); }

    /* No-magnetometer diagnostics (see log.h): AHRS_MODE_AHRS's yaw is only
       observable through the magnetometer, so a prolonged mag outage lets it
       drift unbounded. Warn once both the outage and the resulting yaw
       uncertainty are large enough to matter. */
    if (a->is_initialized && a->cfg.mode == AHRS_MODE_AHRS)
    {
        const float mag_gap_sec    = time_diff_sec(t, a->t_last_mag_fusion);
        const float yaw_stddev_deg = RAD2DEG(SQRTF(ahrs_att_var(a, 2)));
        if (mag_gap_sec >= AHRS_LOG_MAG_GAP_WARN_SEC &&
            yaw_stddev_deg >= AHRS_LOG_YAW_STDDEV_WARN_DEG)
        {
            const bool  first_warn = (a->log_state.t_last_mag_gap_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f : time_diff_sec(t, a->log_state.t_last_mag_gap_warn);
            if (first_warn || since_warn_sec >= AHRS_LOG_MAG_GAP_REPEAT_SEC)
            {
                LOG_WARN("ahrs: no magnetometer fusion for %.1f s, yaw stddev grown to "
                         "%.1f deg",
                         (double)mag_gap_sec, (double)yaw_stddev_deg);
                a->log_state.t_last_mag_gap_warn = t;
            }
        }
        else { a->log_state.t_last_mag_gap_warn = 0; }

        /* Yaw-stddev runaway (see log.h): fast growth within one window,
           independent of the absolute-level check above -- catches
           divergence even while nominally aided. */
        if (a->log_state.t_yaw_stddev_window == 0)
        {
            a->log_state.t_yaw_stddev_window   = t;
            a->log_state.yaw_stddev_window_deg = yaw_stddev_deg;
        }
        else
        {
            const float window_sec = time_diff_sec(t, a->log_state.t_yaw_stddev_window);
            if (window_sec >= AHRS_LOG_RUNAWAY_WINDOW_SEC)
            {
                const float growth_deg = yaw_stddev_deg - a->log_state.yaw_stddev_window_deg;
                if (growth_deg >= AHRS_LOG_YAW_RUNAWAY_DEG)
                {
                    LOG_WARN("ahrs: yaw stddev runaway: grew %.1f deg in %.1f s (%.2f deg/s) "
                             "-- check aiding availability and Q/R tuning",
                             (double)growth_deg, (double)window_sec,
                             (double)(growth_deg / window_sec));
                }
                a->log_state.t_yaw_stddev_window   = t;
                a->log_state.yaw_stddev_window_deg = yaw_stddev_deg;
            }
        }
    }

    /* Gated on the same post-init warm-up as ahrs_check_precision
       (REQ-AHRS-023): the bias estimate legitimately moves fast while
       converging from a loose initial covariance, which would otherwise trip
       the runaway/sanity checks on nearly every run's first window. */
    if (a->is_initialized && time_diff_sec(t, a->t_init) >= a->cfg.restart_warmup_sec)
    {
        /* Gyro-bias runaway + sanity bound. */
        /* Norm into a local first, conversion second. RAD2DEG is a
           _Generic and repeats its argument textually, which cppcheck
           fails to parse when that argument dereferences a pointer. */
        const float gyr_bias_norm_rps =
            SQRTF(qsquare(a->gyr_bias_rps[0]) + qsquare(a->gyr_bias_rps[1]) +
                  qsquare(a->gyr_bias_rps[2]));
        const float gyr_bias_dps = RAD2DEG(gyr_bias_norm_rps);
        if (a->log_state.t_gyr_bias_window == 0)
        {
            a->log_state.t_gyr_bias_window   = t;
            a->log_state.gyr_bias_window_dps = gyr_bias_dps;
        }
        else
        {
            const float window_sec = time_diff_sec(t, a->log_state.t_gyr_bias_window);
            if (window_sec >= AHRS_LOG_RUNAWAY_WINDOW_SEC)
            {
                const float growth = fabsf(gyr_bias_dps - a->log_state.gyr_bias_window_dps);
                if (growth >= AHRS_LOG_GYR_BIAS_RUNAWAY_DPS)
                {
                    LOG_WARN("ahrs: gyro bias runaway: |bias| changed %.2f deg/s in %.1f s "
                             "(now %.2f deg/s) - possible filter divergence",
                             (double)growth, (double)window_sec, (double)gyr_bias_dps);
                }
                a->log_state.t_gyr_bias_window   = t;
                a->log_state.gyr_bias_window_dps = gyr_bias_dps;
            }
        }
        if (gyr_bias_dps >= AHRS_LOG_GYR_BIAS_SANITY_DPS)
        {
            const bool  first_warn = (a->log_state.t_last_gyr_bias_sanity_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f : time_diff_sec(t, a->log_state.t_last_gyr_bias_sanity_warn);
            if (first_warn || since_warn_sec >= AHRS_LOG_BIAS_SANITY_REPEAT_SEC)
            {
                LOG_WARN("ahrs: gyro bias %.2f deg/s exceeds the sanity bound (%.1f deg/s) -- "
                         "implausible for a MEMS IMU, filter likely diverging",
                         (double)gyr_bias_dps, (double)AHRS_LOG_GYR_BIAS_SANITY_DPS);
                a->log_state.t_last_gyr_bias_sanity_warn = t;
            }
        }
    }
}

/* @satisfies REQ-AHRS-013 REQ-AHRS-025 */
void ahrs_update(ahrs_t* a, ahrs_time_us_t t, const float gyr_rps[3], const float acc_mps2[3],
                 const float mag_b[3], bool zero_rotation_update)
{
    ahrs_predict_step(a, t, gyr_rps, acc_mps2, mag_b, zero_rotation_update, NULL);
    ahrs_correct_step(a);
}

/* ============================================================================
 * Public API: accessors
 * ============================================================================
 */

bool ahrs_get_rpy(const ahrs_t* a, float* roll, float* pitch, float* yaw)
{
    if (a == NULL || !a->is_initialized) { return false; }
    float R[9];
    ins_quat_to_rotmat(a->q, R);
    ins_rotmat_to_rpy(R, roll, pitch, yaw);
    return true;
}

bool ahrs_get_quaternion(const ahrs_t* a, float q[4])
{
    if (a == NULL || !a->is_initialized) { return false; }
    memcpy(q, a->q, sizeof(a->q));
    return true;
}

bool ahrs_get_bias_gyr(const ahrs_t* a, float gyr_bias[3])
{
    if (a == NULL || !a->is_initialized) { return false; }
    memcpy(gyr_bias, a->gyr_bias_rps, sizeof(a->gyr_bias_rps));
    return true;
}

bool ahrs_get_rpy_stddev(const ahrs_t* a, float* roll_stddev_rad, float* pitch_stddev_rad,
                         float* yaw_stddev_rad)
{
    if (a == NULL || !a->is_initialized) { return false; }
    *roll_stddev_rad  = SQRTF(ahrs_att_var(a, 0));
    *pitch_stddev_rad = SQRTF(ahrs_att_var(a, 1));
    /* ARS mode has no filtered yaw state (yaw free-integrates): 0 signals
       "no yaw uncertainty available", matching ins_meas_att_hint_t's
       stddev_yaw_rad convention. */
    *yaw_stddev_rad = (a->cfg.mode == AHRS_MODE_AHRS) ? SQRTF(ahrs_att_var(a, 2)) : 0.0f;
    return true;
}

bool ahrs_get_bias_gyr_stddev(const ahrs_t* a, float gyr_bias_stddev_rps[3])
{
    if (a == NULL || !a->is_initialized) { return false; }
    const int off = a->n - 3; /* gyro bias is always the last 3 error states */
    int       i;
    for (i = 0; i < 3; ++i) { gyr_bias_stddev_rps[i] = SQRTF(ahrs_att_var(a, off + i)); }
    return true;
}

bool ahrs_auto_zaru_active(const ahrs_t* a)
{
    return a != NULL && a->is_initialized && !a->cfg.auto_zaru_disable &&
           a->auto_zaru_static_since != 0;
}

/* @satisfies REQ-AHRS-024 */
bool ahrs_zaru_applied(const ahrs_t* a)
{
    return a != NULL && a->is_initialized && a->last_zaru_trigger;
}

/* @satisfies REQ-AHRS-017 */
void ahrs_set_auto_zaru_disable(ahrs_t* a, bool disable)
{
    if (a == NULL) return;
    a->cfg.auto_zaru_disable = disable;
    if (disable)
    {
        /* ahrs_auto_zaru_detect self-heals on every call while disabled; this
           only makes the disabled state take effect immediately rather than
           after the next ahrs_update. */
        a->auto_zaru_static_since = 0;
        a->static_var_count       = 0;
        a->static_var_ok          = false;
    }
}

/* ============================================================================
 * Initialisation heuristics
 * ============================================================================
 */

/* @satisfies REQ-AHRS-012 */
void ahrs_leveling_from_acc(const float acc_mps2[3], float* roll_rad, float* pitch_rad)
{
    /* At rest the specific force is the reaction to gravity:
       gravity (body frame) = -f_b. */
    const float gx = -acc_mps2[0];
    const float gy = -acc_mps2[1];
    const float gz = -acc_mps2[2];
    *roll_rad      = atan2f(gy, gz);
    *pitch_rad     = atan2f(-gx, SQRTF(gy * gy + gz * gz));
}

float ahrs_mag_heading(const float mag_b[3], float roll_rad, float pitch_rad)
{
    float hx, hy;
    ahrs_mag_detilt(mag_b, roll_rad, pitch_rad, &hx, &hy);
    return atan2f(-hy, hx);
}

/* Widen the yaw variance to "heading unknown", never narrowing it
 * (REQ-AHRS-014). Capped just below the yaw restart threshold while the
 * attitude-precision watchdog is armed (REQ-AHRS-023): tripping it would
 * re-initialize the filter, which drops the zone state together with the
 * position and fuses the magnetometer against magnetic north again. Like
 * ins_reacquire_reset_yaw this rebuilds P as a diagonal and gives up the
 * cross-covariances. */
static void ahrs_widen_yaw_unknown(ahrs_t* a)
{
    if (a->cfg.mode != AHRS_MODE_AHRS) { return; } /* ARS: no yaw state */

    float       sd  = AHRS_YAW_UNKNOWN_STDDEV;
    const float thr = a->cfg.restart_att_stddev_rad[2];
    if (!a->cfg.precision_restart_disable && thr > 0.0f &&
        sd > AHRS_YAW_UNKNOWN_RESTART_MARGIN * thr)
    {
        sd = AHRS_YAW_UNKNOWN_RESTART_MARGIN * thr;
    }

    float var[AHRS_UNKNOWNS_MAX] = {0.0f};
    int   i;
    for (i = 0; i < a->n; ++i) { var[i] = ahrs_att_var(a, i); }
    if (var[2] >= sd * sd) { return; }
    var[2] = sd * sd;
    mateye(a->U, a->n);
    for (i = 0; i < a->n; ++i) { a->d[i] = var[i]; }
    LOG_INFO("ahrs: yaw only ever referenced to magnetic north, yaw stddev widened to %.0f deg",
             (double)RAD2DEG(sd));
}

/* @satisfies REQ-AHRS-014 */
void ahrs_set_position(ahrs_t* a, float lat_rad, float lon_rad, float year)
{
    if (a == NULL || !a->is_initialized) { return; }
    if (!isfinite(lat_rad) || !isfinite(lon_rad) || !isfinite(year))
    {
        return; /* drop non-finite position at the API boundary */
    }
    const float lat_deg = RAD2DEG(lat_rad);
    const float lon_deg = RAD2DEG(lon_rad);

    /* Inside a dip pole exclusion zone the declination is meaningless, so the
       magnetometer yaw is dropped until the position leaves it again
       (REQ-SYS-018). Roll and pitch are unaffected, this only costs the
       magnetic heading reference. */
    const bool was_usable = a->mag_heading_usable;
    a->mag_heading_usable = magnetic_heading_reference_valid(lat_deg, lon_deg);
    if (!a->mag_heading_usable)
    {
        if (was_usable)
        {
            LOG_INFO("ahrs: magnetic dip pole zone entered, yaw coasts on the gyro");
        }
        /* No previous declination to keep: a yaw fused before any position is
           a magnetic heading, and no re-framing will make it a true one. Its
           covariance must not go on claiming otherwise. */
        if (a->yaw_on_magnetic_north)
        {
            ahrs_widen_yaw_unknown(a);
            a->yaw_on_magnetic_north = false;
        }
        return; /* keep the last reference rather than adopting a bogus one */
    }

    const float new_decl = DEG2RAD(magnetic_declination_deg(lat_deg, lon_deg, year));

    /* Deterministic yaw re-framing: rotate the nominal attitude by the CHANGE
       in declination about the n-frame down axis, so the estimated yaw steps to
       true north instead of slewing there via the mag fusion. true = magnetic +
       declination, so d_decl is added to the nominal yaw by left-multiplying q
       by q_z(d_decl). Covariance / gyro bias are unchanged (a known rotation
       carries no new uncertainty).

       This is only valid while the yaw is magnetically anchored. After a pass
       through an exclusion zone it is anchored to the gyro instead, and the
       declination on the far side of a dip pole differs by ~80 deg, so
       re-framing would rotate a sound estimate by that amount. Adopt the new
       declination alone there and let the fusion pull the yaw in normally. */
    if (was_usable)
    {
        const float d_decl = new_decl - a->declination_rad;
        const float h      = 0.5f * d_decl;
        const float qz[4]  = {cosf(h), 0.0f, 0.0f, sinf(h)};
        float       q_new[4];
        ins_quat_multiply(qz, a->q, q_new);
        ins_quat_normalize(q_new);
        memcpy(a->q, q_new, sizeof(a->q));
    }
    else { LOG_INFO("ahrs: magnetic dip pole zone left, magnetometer yaw re-enabled"); }

    a->declination_rad       = new_decl;
    a->mag_field_expected_uT = magnetic_field_strength_uT(lat_deg, lon_deg);
    a->declination_applied   = true;
    a->yaw_on_magnetic_north = false;
}
