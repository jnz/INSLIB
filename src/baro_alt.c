/** @file baro_alt.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Baro/accelerometer vertical channel Kalman filter.
 *
 * See baro_alt.h for the state-vector layout and the model. Unlike INSLIB/ahrs
 * this is a plain (full-state) linear Kalman filter: the model is linear, so no
 * error-state formulation is needed. The covariance is still kept as a UDU
 * factorisation and propagated/updated with the Thornton/Bierman routines from
 * KFCore (kalman_udu.h), the same backend as the other filters.
 *
 * Prediction runs at the IMU rate (a 3-state UDU predict is cheap and the state
 * propagation has to run per sample anyway). The barometer fusion is
 * sample-driven.
 */

#include <math.h>
#include <string.h>

#include "baro_alt.h"
#include "geodetic_toolbox.h"
#include "sensor_defaults.h"
#include "linalg.h"
#include "kalman_udu.h"
#include "log.h"

/* ============================================================================
 * Local defines
 * ============================================================================
 */

#define BARO_ALT_US_PER_SEC (1000000LL)

/* Diagnostics only (see log.h): how long a gap in the barometer stream must run
   before LOG_WARN flags it, and how often that warning repeats. */
#define BARO_ALT_LOG_GAP_WARN_SEC   (5.0f)
#define BARO_ALT_LOG_GAP_REPEAT_SEC (10.0f)

/* Initial bias-prior consistency check (see log.h, REQ-BARO-024). While the
   caller feeds zero-velocity updates the platform stands still, so the
   up-acceleration input a_up (specific force rotated to NED, gravity removed)
   is the vertical accelerometer bias itself, up to the sensor noise. Averaged
   over a long enough window it can be held against the configured 1-sigma prior
   cfg.acc_bias_init_stddev_mps2; a gross mismatch means that prior was written
   for a different sensor.
     - SIGMA_FACTOR: sigma the averaged evidence may exceed before the prior
       counts as mis-specified.
     - MIN_DWELL_SEC / MIN_SAMPLES: window long and dense enough for the white
       accelerometer noise to average well below the threshold.
     - MAX_GAP_SEC: a pause this long in the zero-velocity feed means the
       platform may have moved, so the running window is discarded. */
#define BARO_ALT_BIAS_PRIOR_SIGMA_FACTOR   (3.0f)
#define BARO_ALT_BIAS_PRIOR_MIN_DWELL_SEC  (5.0f)
#define BARO_ALT_BIAS_PRIOR_MIN_SAMPLES    (50u)
#define BARO_ALT_BIAS_PRIOR_MAX_GAP_SEC    (1.0f)
#define BARO_ALT_LOG_BIAS_PRIOR_REPEAT_SEC (60.0f)

/* Gravity [m/s^2]. */
#define BARO_ALT_GRAVITY INS_GRAVITY_NOMINAL

/* Don't propagate the state over longer time periods (sensor outage). */
#define BARO_ALT_MAX_DT_SEC (0.5f)

/* Config defaults. sigma_a/sigma_b are continuous-time spectral noise densities
 * [X/sqrt(Hz)], scaled by dt_sec in baro_alt_predict.
 *
 * sigma_a is deliberately not shared with ins.c's accelerometer VRW density
 * (sensor_defaults.h): a_up_mps2 here is a full 3D-to-vertical projection (body
 * specific force rotated to NED via the supplied attitude, gravity removed,
 * REQ-BARO-002), so roll/pitch error leaks in as additional apparent noise and
 * a much larger effective density is needed.
 *
 * sigma_h (direct process noise on h) is meant to stay small: a safety margin
 * against the discretization/model-mismatch gap described at
 * baro_alt_predict, not a primary noise source.
 *
 * The a_b prior and sigma_b are sized for an UNCALIBRATED accelerometer, which
 * is what the filter has to survive: a sensitivity error of a percent already
 * lands well over 0.1 m/s^2 in a_up, and a prior far below that leaves the
 * residual in v, where it stands out as a vertical velocity that never decays.
 * sigma_b is what lets a_b keep tracking a residual that moves with orientation
 * or temperature instead of freezing at its first estimate. Both are therefore
 * loose relative to the pure sensor-datasheet numbers. */
#define BARO_ALT_DEFAULT_H_STDDEV_M             (0.8f)
#define BARO_ALT_DEFAULT_V_STDDEV_MPS           (0.20f)
#define BARO_ALT_DEFAULT_AB_STDDEV_MPS2         (0.1f)
#define BARO_ALT_DEFAULT_ACC_NOISE_MPS2_SQRTHZ  (0.025f)
#define BARO_ALT_DEFAULT_BIAS_DRIFT_MPS2_SQRTHZ (0.005f)
#define BARO_ALT_DEFAULT_H_NOISE_M_SQRTHZ       (0.005f)
#define BARO_ALT_DEFAULT_BARO_STDDEV_M          INS_DEFAULT_BARO_STDDEV_M
#define BARO_ALT_DEFAULT_CHI2                   INS_DEFAULT_CHI2_95_1DOF

/* Zero-velocity pseudo-measurement 1-sigma [m/s]. Should be tight
 * enough to pin v and pull on a_b, loose enough to absorb e.g. the residual
 * vibration of a still-but-running platform. */
#define BARO_ALT_DEFAULT_ZUPT_STDDEV_MPS INS_DEFAULT_ZERO_VEL_STDDEV_MPS

/* Precision restart watchdog (REQ-BARO-021) defaults: a height/velocity 1-sigma
 * this large means the vertical channel has lost the datum, e.g. after a long
 * baro outage left it dead-reckoning on the accelerometer. Generous vs. normal
 * baro-aided operation, so it only trips on real divergence. */
#define BARO_ALT_DEFAULT_RESTART_H_STDDEV_M   (20.0f)
#define BARO_ALT_DEFAULT_RESTART_V_STDDEV_MPS (10.0f)
#define BARO_ALT_DEFAULT_RESTART_WARMUP_SEC   (8.0f)

/* Offset filter default: slow against pair noise, fast enough for
 * weather-induced baro drift (metres/h) plus the ISA-model error that
 * grows with the height excursion from the anchor (REQ-BARO-011).
 * The ISA term sets it, not the weather: a front moving a few hPa/h is
 * ~0.15 m/sqrt(s), while the ISA scale error is a fraction of the height
 * EXCURSION (a tenth of it is ordinary), so a vehicle climbing a couple
 * of hundred metres in a few minutes swings the offset by tens of metres
 * in that time. A time random walk can only bound that, never model it -
 * the driver is altitude, not the clock - so the bound has to be wide
 * enough that the chi2 gate below sees real drift as drift rather than
 * as a stream of outliers. */
#define LOCAL_GNSS_ALT_DEFAULT_RW_MPS (0.3f)

/* Offset filter defaults: fuse at most once per interval and derate the
   reported pair stddevs by 3x before combining them into the measurement
   variance, so a single pair cannot move the offset far. */
#define LOCAL_GNSS_ALT_DEFAULT_UPDATE_INTERVAL_SEC (10.0f)
#define LOCAL_GNSS_ALT_DEFAULT_STDDEV_INFLATION    (3.0f)

/* How often the chi2 downweight of a pair may be reported. A local height
   source that has genuinely walked away from the ellipsoid (a barometer in a
   vehicle whose cabin pressure moves with speed and ventilation) trips the
   gate on pair after pair, all describing the SAME condition. n_downweighted
   carries the exact count either way, so the log only has to say that it is
   happening and keep saying so while it lasts. */
#define LOCAL_GNSS_ALT_LOG_DOWNWEIGHT_REPEAT_SEC (120.0f)

/* ============================================================================
 * Small helpers
 * ============================================================================
 */

static inline float time_diff_sec(baro_alt_time_us_t later, baro_alt_time_us_t earlier)
{
    return ((float)(later - earlier) * (1.0f / BARO_ALT_US_PER_SEC));
}

static inline int time_diff_ms(baro_alt_time_us_t later, baro_alt_time_us_t earlier)
{
    return (int)((later - earlier) / 1000);
}

static inline float bsquare(float x) { return x * x; }

bool baro_alt_pressure_plausible(float pressure_pa)
{
    return ins_isa_pressure_plausible(pressure_pa);
}

/* ============================================================================
 * ISA pressure-to-altitude conversion
 * ============================================================================
 */

/* @satisfies REQ-BARO-004 */
float baro_alt_pressure_to_altitude(float pressure_pa)
{
    /* Shared with ins.c, which fuses barometric height into its own
       vertical state and must land on the very same curve
       (geodetic_toolbox.h's ins_isa_altitude_from_pressure(), REQ-NAV-054). */
    return ins_isa_altitude_from_pressure(pressure_pa);
}

/* ============================================================================
 * Config resolution (0 -> default)
 * ============================================================================
 */

/* Replace every non-positive or non-finite field by its default. */
/* @satisfies REQ-BARO-006 */
static void baro_alt_resolve_config(const baro_alt_config_t* in, baro_alt_config_t* out)
{
    *out = *in;
    if (!(out->h_init_stddev_m > 0.0f) || !isfinite(out->h_init_stddev_m))
        out->h_init_stddev_m = BARO_ALT_DEFAULT_H_STDDEV_M;
    if (!(out->v_init_stddev_mps > 0.0f) || !isfinite(out->v_init_stddev_mps))
        out->v_init_stddev_mps = BARO_ALT_DEFAULT_V_STDDEV_MPS;
    if (!(out->acc_bias_init_stddev_mps2 > 0.0f) || !isfinite(out->acc_bias_init_stddev_mps2))
        out->acc_bias_init_stddev_mps2 = BARO_ALT_DEFAULT_AB_STDDEV_MPS2;
    if (!(out->acc_noise_mps2_sqrthz > 0.0f) || !isfinite(out->acc_noise_mps2_sqrthz))
        out->acc_noise_mps2_sqrthz = BARO_ALT_DEFAULT_ACC_NOISE_MPS2_SQRTHZ;
    if (!(out->acc_bias_drift_mps2_sqrthz > 0.0f) || !isfinite(out->acc_bias_drift_mps2_sqrthz))
        out->acc_bias_drift_mps2_sqrthz = BARO_ALT_DEFAULT_BIAS_DRIFT_MPS2_SQRTHZ;
    if (!(out->h_process_noise_m_sqrthz > 0.0f) || !isfinite(out->h_process_noise_m_sqrthz))
        out->h_process_noise_m_sqrthz = BARO_ALT_DEFAULT_H_NOISE_M_SQRTHZ;
    if (!(out->baro_stddev_m > 0.0f) || !isfinite(out->baro_stddev_m))
        out->baro_stddev_m = BARO_ALT_DEFAULT_BARO_STDDEV_M;
    if (!(out->chi2_threshold > 0.0f) || !isfinite(out->chi2_threshold))
        out->chi2_threshold = BARO_ALT_DEFAULT_CHI2;
    if (!(out->zupt_stddev_mps > 0.0f) || !isfinite(out->zupt_stddev_mps))
        out->zupt_stddev_mps = BARO_ALT_DEFAULT_ZUPT_STDDEV_MPS;
    /* Restart thresholds: < 0 leaves that state unchecked, 0/invalid ->
       default. Warm-up: 0/invalid -> default. */
    if (out->restart_h_stddev_m < 0.0f)
        out->restart_h_stddev_m = 0.0f;
    else if (!(out->restart_h_stddev_m > 0.0f) || !isfinite(out->restart_h_stddev_m))
        out->restart_h_stddev_m = BARO_ALT_DEFAULT_RESTART_H_STDDEV_M;
    if (out->restart_v_stddev_mps < 0.0f)
        out->restart_v_stddev_mps = 0.0f;
    else if (!(out->restart_v_stddev_mps > 0.0f) || !isfinite(out->restart_v_stddev_mps))
        out->restart_v_stddev_mps = BARO_ALT_DEFAULT_RESTART_V_STDDEV_MPS;
    if (!(out->restart_warmup_sec > 0.0f) || !isfinite(out->restart_warmup_sec))
        out->restart_warmup_sec = BARO_ALT_DEFAULT_RESTART_WARMUP_SEC;
}

/* Dump every effective (post 0 -> default resolution) baro_alt_config_t
 * parameter the filter will actually run with. Every field is already resolved
 * by baro_alt_resolve_config, so this reads b->cfg directly. */
static void baro_alt_log_effective_config(const baro_alt_t* b)
{
#if LOG_LEVEL >= LOG_LEVEL_INFO
    /* Every statement below exists only to feed LOG_INFO; below the
       compile-time ceiling LOG_INFO expands to nothing (log.h) and these locals
       would be flagged -Wunused-variable, so the body is compiled out too. */
    const baro_alt_config_t* cfg = &b->cfg;

    LOG_INFO("baro_alt: init stddevs: h %.2f m, v %.2f m/s, acc bias %.3f m/s^2",
             (double)cfg->h_init_stddev_m, (double)cfg->v_init_stddev_mps,
             (double)cfg->acc_bias_init_stddev_mps2);
    LOG_INFO("baro_alt: noise densities: h %.4f m/sqrt(Hz), acc %.4f m/s^2/sqrt(Hz), "
             "acc bias drift %.2e m/s^2/sqrt(Hz)",
             (double)cfg->h_process_noise_m_sqrthz, (double)cfg->acc_noise_mps2_sqrthz,
             (double)cfg->acc_bias_drift_mps2_sqrthz);
    LOG_INFO("baro_alt: barometer stddev %.2f m, chi2 threshold %.2f, chi2 downweighting %s",
             (double)cfg->baro_stddev_m, (double)cfg->chi2_threshold,
             cfg->chi2_disable ? "disabled" : "enabled");
    LOG_INFO("baro_alt: zero-velocity update stddev %.3f m/s", (double)cfg->zupt_stddev_mps);
    if (cfg->precision_restart_disable)
    {
        LOG_INFO("baro_alt: precision-restart watchdog: disabled");
    }
    else
    {
        LOG_INFO("baro_alt: precision-restart watchdog: enabled, threshold h %.1f m "
                 "(< 0 = not checked) / v %.1f m/s (< 0 = not checked), warm-up %.1f s",
                 (double)cfg->restart_h_stddev_m, (double)cfg->restart_v_stddev_mps,
                 (double)cfg->restart_warmup_sec);
    }
#else
    (void)b;
#endif
}

/* ============================================================================
 * Public API: init
 * ============================================================================
 */

/* @satisfies REQ-BARO-001 REQ-BARO-004 REQ-BARO-006 REQ-BARO-009 */
int baro_alt_init(baro_alt_t* b, const baro_alt_config_t* cfg, baro_alt_time_us_t t,
                  float pressure_pa, float h_init_m, float h_init_stddev_m)
{
    if (b == NULL || cfg == NULL) { return -1; }
    if (!baro_alt_pressure_plausible(pressure_pa) || !isfinite(h_init_m)) { return -1; }

    memset(b, 0, sizeof(*b));
    baro_alt_resolve_config(cfg, &b->cfg);
    if (h_init_stddev_m > 0.0f && isfinite(h_init_stddev_m))
    {
        b->cfg.h_init_stddev_m = h_init_stddev_m;
    }

    /* The anchor sample corresponds to h_init above the caller's datum, so h
       starts there and the datum zero point in ISA altitude is anchor - h_init.
       Diagonal initial covariance as UDU factors: U = I, d = variances. */
    b->x[0] = h_init_m;
    mateye(b->U, BARO_ALT_STATES);
    b->d[0] = bsquare(b->cfg.h_init_stddev_m);
    b->d[1] = bsquare(b->cfg.v_init_stddev_mps);
    b->d[2] = bsquare(b->cfg.acc_bias_init_stddev_mps2);

    b->h0_baro_m       = baro_alt_pressure_to_altitude(pressure_pa) - h_init_m;
    b->t_init          = t;
    b->t_last          = t;
    b->t_last_baro_fix = t; /* the anchor sample counts as a fix */
    b->is_initialized  = true;
    LOG_INFO("baro_alt: filter started, h_init=%.1f m (stddev %.2f m), v stddev=%.2f m/s, "
             "anchor pressure=%.0f Pa",
             (double)h_init_m, (double)b->cfg.h_init_stddev_m, (double)b->cfg.v_init_stddev_mps,
             (double)pressure_pa);
    baro_alt_log_effective_config(b);
    return 0;
}

/* ============================================================================
 * Prediction (accelerometer as control input)
 *
 *   x_k = Phi * x_{k-1} + B * a
 *   Phi = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1],  B = [0.5*dt^2; dt; 0]
 *
 * Process noise in noise-input form for kalman_udu_predict:
 *   G = [e1, e2, e3], Q = diag(sigma_h^2 * dt, sigma_a^2 * dt, sigma_b^2 * dt)
 * sigma_h/sigma_a/sigma_b are continuous-time spectral densities [X/sqrt(Hz)].
 * The "* dt" discretization makes the injected noise rate-invariant.
 * ============================================================================
 */

/* @satisfies REQ-BARO-002 REQ-BARO-003 */
/* Hold the averaged up-acceleration of the current stillness window against the
   configured initial accel-bias prior (see BARO_ALT_BIAS_PRIOR_* above).
   Diagnostic only: counts and warns, never touches the filter state. Called
   once per prediction step; the window only runs while zero-velocity updates
   keep arriving. */
/* @satisfies REQ-BARO-024 */
static void baro_alt_check_bias_prior(baro_alt_t* b, baro_alt_time_us_t t, float a_up_mps2)
{
    if (b->t_last_zupt == 0 || time_diff_sec(t, b->t_last_zupt) > BARO_ALT_BIAS_PRIOR_MAX_GAP_SEC)
    {
        b->acc_bias_prior_count = 0; /* not (known to be) standing still */
        return;
    }
    if (b->acc_bias_prior_count == 0)
    {
        b->acc_bias_prior_sum   = 0.0f;
        b->acc_bias_prior_since = t;
    }
    b->acc_bias_prior_sum += a_up_mps2;
    b->acc_bias_prior_count++;

    if (b->acc_bias_prior_count < BARO_ALT_BIAS_PRIOR_MIN_SAMPLES) { return; }
    if (time_diff_sec(t, b->acc_bias_prior_since) < BARO_ALT_BIAS_PRIOR_MIN_DWELL_SEC) { return; }

    const float mean        = b->acc_bias_prior_sum / (float)b->acc_bias_prior_count;
    b->acc_bias_prior_count = 0; /* window consumed: start a fresh one either way */

    const float thr = BARO_ALT_BIAS_PRIOR_SIGMA_FACTOR * b->cfg.acc_bias_init_stddev_mps2;
    if (fabsf(mean) <= thr) { return; }

    b->n_acc_bias_prior_exceeded++;
    const bool  first_warn = (b->log_state.t_last_acc_bias_prior_warn == 0);
    const float since_warn_sec =
        first_warn ? 0.0f : time_diff_sec(t, b->log_state.t_last_acc_bias_prior_warn);
    if (first_warn || since_warn_sec >= BARO_ALT_LOG_BIAS_PRIOR_REPEAT_SEC)
    {
        LOG_WARN("baro_alt: standing still, but the averaged up-acceleration is %.3f m/s^2 -- "
                 "more than %.0f sigma of the configured initial accel bias (%.3f m/s^2), the "
                 "prior is too tight for this sensor",
                 (double)mean, (double)BARO_ALT_BIAS_PRIOR_SIGMA_FACTOR,
                 (double)b->cfg.acc_bias_init_stddev_mps2);
        b->log_state.t_last_acc_bias_prior_warn = t;
    }
}

static void baro_alt_predict(baro_alt_t* b, float a_up_mps2, float dt_sec, float* phi_out)
{
    const int   n    = BARO_ALT_STATES;
    const float dt   = dt_sec;
    const float dt2h = 0.5f * dt * dt;

    /* State propagation. Phi couples a_b into h and v, so the
       effective acceleration is the measured one plus the correction. */
    const float a_eff = a_up_mps2 + b->x[2];
    b->x[0] += dt * b->x[1] + dt2h * a_eff;
    b->x[1] += dt * a_eff;

    float Phi[BARO_ALT_STATES * BARO_ALT_STATES];
    float G[BARO_ALT_STATES * 3];
    float Q[3];
    mateye(Phi, n);
    MAT_ELEM(Phi, 0, 1, n, n) = dt;
    MAT_ELEM(Phi, 0, 2, n, n) = dt2h;
    MAT_ELEM(Phi, 1, 2, n, n) = dt;
    memset(G, 0, sizeof(G));
    /* Accelerometer noise enters v's own derivative directly (unit weight). h
       gets a small DIRECT term of its own (sigma_h): the exact
       continuous-discrete noise model for this double integrator would also
       place Q_hh ~ sigma_a^2*dt^3/3 and Q_hv ~ sigma_a^2*dt^2/2 terms here,
       which a G with unit weight solely on v leaves at zero for this step.
       sigma_h is a deliberately small, independently tunable stand-in for that
       omitted term - a margin against discretization/model mismatch, not a
       primary noise source. */
    MAT_ELEM(G, 0, 0, n, 3) = 1.0f; /* small direct process noise on h */
    MAT_ELEM(G, 1, 1, n, 3) = 1.0f; /* accelerometer noise into v */
    MAT_ELEM(G, 2, 2, n, 3) = 1.0f; /* bias drift on a_b */

    Q[0] = bsquare(b->cfg.h_process_noise_m_sqrthz) * dt;
    Q[1] = bsquare(b->cfg.acc_noise_mps2_sqrthz) * dt;
    Q[2] = bsquare(b->cfg.acc_bias_drift_mps2_sqrthz) * dt;

    if (phi_out != NULL) { memcpy(phi_out, Phi, sizeof(Phi[0]) * (size_t)(n * n)); }

    /* State already propagated above (control input is not part of
       Phi*x), so only the covariance is predicted here. */
    kalman_udu_predict(NULL, b->U, b->d, Phi, G, Q, n, 3);
}

/* ============================================================================
 * Barometer fusion
 * ============================================================================
 */

/* Diagnostic-only re-check of kalman_udu's own chi2 gate, used to count
 * downweighted fusions (REQ-BARO-017) without affecting the fusion. Exact here:
 * a single scalar measurement, so no multi-row caveat. */
static bool baro_alt_is_outlier(const baro_alt_t* b, float z, float R,
                                const float Ht[BARO_ALT_STATES], float chi2_threshold)
{
    if (!(chi2_threshold > 0.0f)) { return false; }
    float dz = z;
    matmul("N", "N", 1, 1, BARO_ALT_STATES, -1.0f, Ht, b->x, 1.0f, &dz); /* dz = z - H*x */
    float tmp[BARO_ALT_STATES];
    matmul("N", "N", 1, BARO_ALT_STATES, BARO_ALT_STATES, 1.0f, Ht, b->U, 0.0f, tmp);
    float HPHT = 0.0f;
    int   j;
    for (j = 0; j < BARO_ALT_STATES; ++j) { HPHT += tmp[j] * tmp[j] * b->d[j]; }
    const float s = HPHT + R;
    return dz * dz > chi2_threshold * s;
}

/* Scalar altitude measurement z = h_baro - h0 observing only h
 * (H = [1 0 0]). Outliers are chi2-downweighted, not dropped: baro is
 * a persistent absolute reference (same policy as GNSS/yaw/mag). */
/* @satisfies REQ-BARO-005 REQ-BARO-016 REQ-BARO-017 */
static void baro_alt_fuse_baro(baro_alt_t* b, float h_baro_m, float stddev_m)
{
    const float z                   = h_baro_m;
    const float R                   = bsquare(stddev_m);
    const float Ht[BARO_ALT_STATES] = {1.0f, 0.0f, 0.0f};
    /* Global override (REQ-BARO-016): 0.0f makes kalman_udu skip the
       chi2 test entirely, fusing at nominal variance and never
       downweighting. */
    const float chi2_threshold = b->cfg.chi2_disable ? 0.0f : b->cfg.chi2_threshold;

    if (baro_alt_is_outlier(b, z, R, Ht, chi2_threshold)) { b->n_downweighted++; }

    if (kalman_udu(b->x, b->U, b->d, &z, &R, Ht, BARO_ALT_STATES, 1, chi2_threshold,
                   1 /* downweight outliers */) != 0)
    {
        b->n_fuse_fail++;
    }
}

/* ============================================================================
 * Health check
 * ============================================================================
 */

static void baro_alt_check_health(baro_alt_t* b)
{
    int i;
    for (i = 0; i < BARO_ALT_STATES; ++i)
    {
        if (!isfinite(b->x[i]) || !isfinite(b->d[i]) || b->d[i] < 0.0f)
        {
            LOG_ERROR("baro_alt: health check failed (non-finite/negative state or "
                      "covariance), filter shut down");
            b->is_initialized = false;
            return;
        }
    }
}

/* Per-state variance P_ii from the UDU factors (P = U diag(d) U',
 * U unit upper triangular): P_ii = d_i + sum_{k>i} U(i,k)^2 d_k. */
static float baro_alt_state_var(const baro_alt_t* b, int i)
{
    const int n = BARO_ALT_STATES;
    float     p = b->d[i];
    int       k;
    for (k = i + 1; k < n; ++k) { p += bsquare(MAT_ELEM(b->U, i, k, n, n)) * b->d[k]; }
    return p;
}

/* Precision restart watchdog: once past the warm-up, if the reported height or
 * vertical-velocity 1-sigma exceeds its threshold the estimate is no longer
 * trustworthy, so the filter is marked uninitialized (fail-safe, same mechanism
 * as the health check). nav_suite then re-bootstraps it from the live stream, a
 * standalone caller must call baro_alt_init() again. */
/* @satisfies REQ-BARO-021 */
static void baro_alt_check_precision(baro_alt_t* b)
{
    if (b->cfg.precision_restart_disable) { return; }
    if (time_diff_sec(b->t_last, b->t_init) < b->cfg.restart_warmup_sec) { return; }

    const float h_thr = b->cfg.restart_h_stddev_m;
    const float v_thr = b->cfg.restart_v_stddev_mps;
    if ((h_thr > 0.0f && baro_alt_state_var(b, 0) > bsquare(h_thr)) ||
        (v_thr > 0.0f && baro_alt_state_var(b, 1) > bsquare(v_thr)))
    {
        b->is_initialized = false;
        b->n_restart++;
        LOG_WARN("baro_alt: precision watchdog tripped (height/velocity stddev exceeded "
                 "threshold), filter restart #%u",
                 (unsigned int)b->n_restart);
    }
}

/* ============================================================================
 * Public API: update
 * ============================================================================
 */

/* @satisfies REQ-BARO-007 REQ-BARO-026 */
int baro_alt_predict_step(baro_alt_t* b, baro_alt_time_us_t t, const float acc_mps2[3],
                          const float q_bn[4], float pressure_pa, float baro_stddev_m,
                          bool baro_valid, float* phi_out)
{
    if (!b->is_initialized) { return BARO_ALT_EPOCH_DROPPED; }

    /* Non-finite inputs (NaN/Inf) must not reach the math (a NaN passes every
       chi2/variance gate because NaN comparisons are false). Drop the epoch;
       the next epoch's dt spans the gap, which the MAX_DT gate handles. */
    if (!isfinite(acc_mps2[0]) || !isfinite(acc_mps2[1]) || !isfinite(acc_mps2[2]) ||
        !isfinite(q_bn[0]) || !isfinite(q_bn[1]) || !isfinite(q_bn[2]) || !isfinite(q_bn[3]))
    {
        b->n_invalid_input++;
        LOG_WARN("baro_alt: non-finite accel/attitude input dropped (%u total since init)",
                 (unsigned int)b->n_invalid_input);
        b->step_ctx.active = false;
        return BARO_ALT_EPOCH_DROPPED;
    }
    if (baro_valid && !baro_alt_pressure_plausible(pressure_pa))
    {
        b->n_invalid_input++;
        LOG_WARN("baro_alt: implausible/non-finite pressure sample %.0f Pa dropped "
                 "(%u total since init)",
                 (double)pressure_pa, (unsigned int)b->n_invalid_input);
        baro_valid = false; /* acc/attitude are fine: only drop the baro */
    }

    b->epoch++;

    const float dt_sec = time_diff_sec(t, b->t_last);
    b->t_last          = t;
    if (dt_sec < 0.0f)
    {
        b->step_ctx.active = false;
        return BARO_ALT_EPOCH_DROPPED; /* time jumped backwards: re-anchor the clock, skip */
    }

    /* Barometer-gap diagnostics (see log.h): t_last_baro_fix is set at
       init, so this is always meaningful once the filter is running. */
    {
        const float gap_sec = time_diff_sec(t, b->t_last_baro_fix);
        if (gap_sec >= BARO_ALT_LOG_GAP_WARN_SEC)
        {
            const bool  first_warn = (b->log_state.t_last_gap_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f : time_diff_sec(t, b->log_state.t_last_gap_warn);
            if (first_warn || since_warn_sec >= BARO_ALT_LOG_GAP_REPEAT_SEC)
            {
                LOG_WARN("baro_alt: no accepted barometer fix for %.1f s, riding on "
                         "accelerometer dead reckoning",
                         (double)gap_sec);
                b->log_state.t_last_gap_warn = t;
            }
        }
        else if (b->log_state.t_last_gap_warn != 0)
        {
            LOG_INFO("baro_alt: barometer fix reacquired after a %.1f s gap", (double)gap_sec);
            b->log_state.t_last_gap_warn = 0;
        }
    }

    /* PREDICTION STEP (accelerometer as control input)
     * Measured up-acceleration: rotate the body-frame specific force to NED and
     * remove gravity. NED z is down, h is up: a_up = -(f_n_z + g). */
    int status = 0;
    if (dt_sec > 0.0f && dt_sec < BARO_ALT_MAX_DT_SEC)
    {
        float R_b_to_n[9];
        ins_quat_to_rotmat(q_bn, R_b_to_n);
        const float f_n_z = MAT_ELEM(R_b_to_n, 2, 0, 3, 3) * acc_mps2[0] +
                            MAT_ELEM(R_b_to_n, 2, 1, 3, 3) * acc_mps2[1] +
                            MAT_ELEM(R_b_to_n, 2, 2, 3, 3) * acc_mps2[2];
        const float a_up  = -(f_n_z + BARO_ALT_GRAVITY);
        b->last_a_up_mps2 = a_up;
        b->have_last_a_up = true;
        baro_alt_predict(b, a_up, dt_sec, phi_out);
        baro_alt_check_bias_prior(b, t, a_up);
        status |= BARO_ALT_EPOCH_COV_PROPAGATED;
    }

    /* Hand off to baro_alt_correct_step(): the sanitized pressure sample, since
       it cannot re-derive the plausibility decision afterwards. */
    b->step_ctx.pressure_pa   = pressure_pa;
    b->step_ctx.baro_stddev_m = baro_stddev_m;
    b->step_ctx.baro_valid    = baro_valid;
    b->step_ctx.active        = true;
    return status;
}

/* @satisfies REQ-BARO-008 REQ-BARO-026 */
void baro_alt_correct_step(baro_alt_t* b)
{
    if (!b->step_ctx.active) return;
    b->step_ctx.active = false;

    const baro_alt_time_us_t t             = b->t_last;
    const float              pressure_pa   = b->step_ctx.pressure_pa;
    const float              baro_stddev_m = b->step_ctx.baro_stddev_m;
    const bool               baro_valid    = b->step_ctx.baro_valid;

    /* FUSION STEP FOR THE BAROMETER (sample-driven)
     * --------------------------------------------- */
    if (baro_valid)
    {
        float stddev = baro_stddev_m;
        if (!(stddev > 0.0f) || !isfinite(stddev)) { stddev = b->cfg.baro_stddev_m; }
        const float h_baro         = baro_alt_pressure_to_altitude(pressure_pa) - b->h0_baro_m;
        b->last_h_meas_m           = h_baro;
        b->have_last_h_meas        = true;
        const uint32_t fail_before = b->n_fuse_fail;
        baro_alt_fuse_baro(b, h_baro, stddev);
        /* Downweighted counts as accepted (REQ-BARO-017's policy is to
           fuse at an inflated variance, not to drop), only an actual
           kalman_udu failure withholds the fresh fix. */
        if (b->n_fuse_fail == fail_before) { b->t_last_baro_fix = t; }
    }

    baro_alt_check_health(b);
    if (b->is_initialized) { baro_alt_check_precision(b); }
}

/* @satisfies REQ-BARO-007 REQ-BARO-008 */
void baro_alt_update(baro_alt_t* b, baro_alt_time_us_t t, const float acc_mps2[3],
                     const float q_bn[4], float pressure_pa, float baro_stddev_m, bool baro_valid)
{
    baro_alt_predict_step(b, t, acc_mps2, q_bn, pressure_pa, baro_stddev_m, baro_valid, NULL);
    baro_alt_correct_step(b);
}

/* Scalar zero-velocity measurement z = 0 observing only v (H = [0 1 0]).
 *
 * The trigger is the caller's stillness detection (under nav_suite: the
 * zero-rotation trigger, REQ-SUITE-015), never b->x[1], the filter's own
 * velocity estimate.
 *
 * Besides bounding the v drift, the update pulls on a_b through the v/a_b
 * covariance coupling built up by the prediction, so a standstill observes the
 * acceleration correction even while the barometer is out. Outliers are
 * chi2-downweighted, not dropped. */
/* @satisfies REQ-BARO-022 */
void baro_alt_zero_velocity_update(baro_alt_t* b, float stddev_mps)
{
    if (b == NULL || !b->is_initialized) { return; }

    /* Stillness marker for the bias-prior check (REQ-BARO-024): the
       prediction step alone cannot tell whether the platform is moving. */
    b->t_last_zupt = b->t_last;

    float stddev = stddev_mps;
    if (!(stddev > 0.0f) || !isfinite(stddev)) { stddev = b->cfg.zupt_stddev_mps; }

    const float z                   = 0.0f;
    const float R                   = bsquare(stddev);
    const float Ht[BARO_ALT_STATES] = {0.0f, 1.0f, 0.0f};
    /* Global override (REQ-BARO-016): 0.0f makes kalman_udu skip the
       chi2 test entirely, fusing at nominal variance. */
    const float chi2_threshold = b->cfg.chi2_disable ? 0.0f : b->cfg.chi2_threshold;

    if (baro_alt_is_outlier(b, z, R, Ht, chi2_threshold))
    {
        b->n_downweighted++;
        LOG_WARN("baro_alt: zero-velocity chi2 outlier gate tripped, downweighting "
                 "(v=%.3f m/s, chi2 threshold %.2f, #%u since init)",
                 (double)b->x[1], (double)chi2_threshold, (unsigned int)b->n_downweighted);
    }

    if (kalman_udu(b->x, b->U, b->d, &z, &R, Ht, BARO_ALT_STATES, 1, chi2_threshold,
                   1 /* downweight outliers */) != 0)
    {
        b->n_fuse_fail++;
    }
    else { b->n_zupt++; }

    baro_alt_check_health(b);
}

/* ============================================================================
 * Public API: accessors
 * ============================================================================
 */

bool baro_alt_get_height(const baro_alt_t* b, float* h_m)
{
    if (b == NULL || !b->is_initialized) { return false; }
    *h_m = b->x[0];
    return true;
}

/* @satisfies REQ-BARO-019 */
bool baro_alt_get_isa_altitude(const baro_alt_t* b, float* h_isa_m)
{
    if (b == NULL || !b->is_initialized) { return false; }
    *h_isa_m = b->h0_baro_m + b->x[0];
    return true;
}

bool baro_alt_get_velocity(const baro_alt_t* b, float* v_mps)
{
    if (b == NULL || !b->is_initialized) { return false; }
    *v_mps = b->x[1];
    return true;
}

bool baro_alt_get_acc_bias(const baro_alt_t* b, float* acc_bias_mps2)
{
    if (b == NULL || !b->is_initialized) { return false; }
    *acc_bias_mps2 = b->x[2];
    return true;
}

/* @satisfies REQ-BARO-025 */
bool baro_alt_get_measurement_a_z(const baro_alt_t* b, float* a_z_mps2)
{
    if (b == NULL || !b->is_initialized || !b->have_last_a_up) { return false; }
    *a_z_mps2 = b->last_a_up_mps2;
    return true;
}

/* @satisfies REQ-BARO-025 */
bool baro_alt_get_measurement_h(const baro_alt_t* b, float* h_meas_m)
{
    if (b == NULL || !b->is_initialized || !b->have_last_h_meas) { return false; }
    *h_meas_m = b->last_h_meas_m;
    return true;
}

/* @satisfies REQ-BARO-023 */
int baro_alt_deadreckoning_ms(const baro_alt_t* b)
{
    if (b == NULL || !b->is_initialized) { return -1; }
    const int ms = time_diff_ms(b->t_last, b->t_last_baro_fix);
    return (ms < 0) ? 0 : ms;
}

/* ============================================================================
 * local_gnss_alt: 1-state local-height-to-ellipsoid offset filter
 *
 * o = h_gnss_ell - h_local, modelled as a slow random walk. h_local is a height
 * above the caller's vertical datum, so o is the ellipsoid height of the datum
 * origin and h_ell = h_local + o. Source-agnostic: the random walk absorbs
 * whatever drift the local height source has against the ellipsoid (see
 * baro_alt.h). With a single state the covariance is a plain variance, so the
 * Kalman equations are written out directly, a deliberate exception from the
 * UDU backend used by the larger filters.
 * ============================================================================
 */

static void local_gnss_alt_resolve_config(const local_gnss_alt_config_t* in,
                                          local_gnss_alt_config_t*       out)
{
    *out = *in;
    if (!(out->rw_stddev_mps > 0.0f) || !isfinite(out->rw_stddev_mps))
        out->rw_stddev_mps = LOCAL_GNSS_ALT_DEFAULT_RW_MPS;
    if (!(out->local_stddev_m > 0.0f) || !isfinite(out->local_stddev_m))
        out->local_stddev_m = BARO_ALT_DEFAULT_BARO_STDDEV_M;
    if (!(out->chi2_threshold > 0.0f) || !isfinite(out->chi2_threshold))
        out->chi2_threshold = BARO_ALT_DEFAULT_CHI2;
    if (!(out->min_update_interval_sec > 0.0f) || !isfinite(out->min_update_interval_sec))
        out->min_update_interval_sec = LOCAL_GNSS_ALT_DEFAULT_UPDATE_INTERVAL_SEC;
    if (!(out->stddev_inflation_factor > 0.0f) || !isfinite(out->stddev_inflation_factor))
        out->stddev_inflation_factor = LOCAL_GNSS_ALT_DEFAULT_STDDEV_INFLATION;
}

/* Shared input screening for init and update: both altitudes finite, GNSS
 * vertical accuracy positive and finite. Returns the resolved measurement
 * variance R = (local*inflation)^2 + (gnss*inflation)^2 via *R_out. */
static bool local_gnss_alt_pair_ok(const local_gnss_alt_config_t* cfg, float h_local_m,
                                   float local_stddev_m, float h_gnss_ell_m, float gnss_stddev_m,
                                   float* R_out)
{
    if (!isfinite(h_local_m) || !isfinite(h_gnss_ell_m)) { return false; }
    if (!(gnss_stddev_m > 0.0f) || !isfinite(gnss_stddev_m)) { return false; }
    if (!(local_stddev_m > 0.0f) || !isfinite(local_stddev_m))
    {
        local_stddev_m = cfg->local_stddev_m;
    }
    *R_out = bsquare(local_stddev_m * cfg->stddev_inflation_factor) +
             bsquare(gnss_stddev_m * cfg->stddev_inflation_factor);
    return true;
}

/* @satisfies REQ-BARO-010 REQ-BARO-013 */
int local_gnss_alt_init(local_gnss_alt_t* g, const local_gnss_alt_config_t* cfg,
                        baro_alt_time_us_t t, float h_local_m, float local_stddev_m,
                        float h_gnss_ell_m, float gnss_stddev_m)
{
    if (g == NULL || cfg == NULL) { return -1; }
    local_gnss_alt_config_t resolved;
    local_gnss_alt_resolve_config(cfg, &resolved);

    float R;
    if (!local_gnss_alt_pair_ok(&resolved, h_local_m, local_stddev_m, h_gnss_ell_m, gnss_stddev_m,
                                &R))
    {
        return -1;
    }

    memset(g, 0, sizeof(*g));
    g->cfg            = resolved;
    g->offset_m       = h_gnss_ell_m - h_local_m;
    g->var_m2         = R;
    g->t_last         = t;
    g->is_initialized = true;
    LOG_INFO("baro_alt: local/GNSS offset filter started, offset=%.2f m (stddev %.2f m)",
             (double)g->offset_m, (double)SQRTF(g->var_m2));
    return 0;
}

/* @satisfies REQ-BARO-010 REQ-BARO-011 REQ-BARO-012 REQ-BARO-013 REQ-BARO-014 REQ-BARO-015
 * REQ-BARO-016 REQ-BARO-017 */
void local_gnss_alt_update(local_gnss_alt_t* g, baro_alt_time_us_t t, float h_local_m,
                           float local_stddev_m, float h_gnss_ell_m, float gnss_stddev_m)
{
    if (!g->is_initialized) { return; }

    float R;
    if (!local_gnss_alt_pair_ok(&g->cfg, h_local_m, local_stddev_m, h_gnss_ell_m, gnss_stddev_m,
                                &R))
    {
        g->n_invalid_input++;
        return;
    }

    g->epoch++;

    const float dt_sec = time_diff_sec(t, g->t_last);
    if (dt_sec < 0.0f)
    {
        g->t_last = t; /* time jumped backwards: re-anchor the clock, skip */
        return;
    }

    /* Decimate to cfg.min_update_interval_sec: the offset is nudged by
       at most one fusion per interval instead of by every incoming
       pair, which is what made the filter jumpy at the caller's raw
       pair rate. t_last (and thus dt_sec) only advances on an actual
       fusion, so the skipped time is not lost: it is folded into the
       next accepted pair's random-walk propagation below. */
    if (dt_sec < g->cfg.min_update_interval_sec)
    {
        g->n_decimated++;
        return;
    }
    g->t_last = t;

    /* Random-walk propagation: the variance grows with the time since
       the last pair, so the offset loses confidence during outages. */
    g->var_m2 += bsquare(g->cfg.rw_stddev_mps) * dt_sec;

    /* Scalar measurement update, z = o + v. Outliers are downweighted
       by inflating R so the normalized innovation meets the gate
       (Chang 2014, same policy as the robust UDU update). Global
       override (REQ-BARO-016): chi2_disable skips the test, fusing at
       nominal R regardless of the innovation. */
    const float dz = (h_gnss_ell_m - h_local_m) - g->offset_m;
    const float S  = g->var_m2 + R;
    if (!g->cfg.chi2_disable && dz * dz > g->cfg.chi2_threshold * S)
    {
        g->n_downweighted++;
        /* Throttled: see LOCAL_GNSS_ALT_LOG_DOWNWEIGHT_REPEAT_SEC. */
        const bool  first_warn = (g->t_last_downweight_warn == 0);
        const float since_warn_sec =
            first_warn ? 0.0f : time_diff_sec(t, g->t_last_downweight_warn);
        if (first_warn || since_warn_sec >= LOCAL_GNSS_ALT_LOG_DOWNWEIGHT_REPEAT_SEC)
        {
            LOG_WARN("baro_alt: local-height/GNSS-ellipsoid offset innovation %.2f m exceeds "
                     "the chi2 gate, downweighting (current offset %.2f m, #%u since init)",
                     (double)dz, (double)g->offset_m, (unsigned int)g->n_downweighted);
            g->t_last_downweight_warn = t;
        }
        R = dz * dz / g->cfg.chi2_threshold - g->var_m2;
    }
    const float K = g->var_m2 / (g->var_m2 + R);
    g->offset_m += K * dz;
    g->var_m2 *= (1.0f - K);

    if (!isfinite(g->offset_m) || !isfinite(g->var_m2) || g->var_m2 <= 0.0f)
    {
        LOG_ERROR("baro_alt: local/GNSS offset filter health check failed "
                  "(non-finite/non-positive state), filter shut down");
        g->is_initialized = false; /* health check */
    }
}

bool local_gnss_alt_get(const local_gnss_alt_t* g, float* offset_m, float* stddev_m)
{
    if (g == NULL || !g->is_initialized) { return false; }
    if (offset_m != NULL) { *offset_m = g->offset_m; }
    if (stddev_m != NULL) { *stddev_m = SQRTF(g->var_m2); }
    return true;
}
