/** @file baro_alt.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Baro/accelerometer vertical channel Kalman filter.
 *
 * A 3-state linear Kalman filter fusing barometric pressure and accelerometer
 * measurements. Runs standalone next to the full ins filter and the AHRS
 * filters (see nav_suite.h for a wrapper that runs all of them). State vector:
 *
 *   [0] h    height above start [m], positive up
 *   [1] v    vertical velocity [m/s], positive up
 *   [2] a_b  slow-varying additive correction to the measured vertical
 *            acceleration [m/s^2] (absorbs accelerometer bias, attitude error
 *            and gravity model error projected onto the vertical axis)
 *
 * Key idea: the accelerometer is the control input of the prediction, NOT a
 * measurement. Each IMU epoch the body-frame specific force is rotated to NED
 * with the supplied attitude quaternion, gravity is removed
 * (a = -(f_n_z + g), positive up) and the state is propagated with
 * constant-acceleration kinematics:
 *
 *   x_k = Phi * x_{k-1} + B * a
 *   Phi = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1],  B = [0.5*dt^2; dt; 0]
 *   Q   = B * sigma_a^2 * B' + diag(0, 0, sigma_b^2)
 *
 * The barometer observes only h: pressure is converted to altitude with the
 * international standard atmosphere formula
 * h_baro = 44330 * (1 - (p/p0)^(1/5.255)), anchored at the init pressure
 * sample, which corresponds to the height h_init given to baro_alt_init
 * (0 = "height above start"). That is how the filter is aligned to an external
 * vertical datum, e.g. the NED origin of ins (see nav_suite). Implausible
 * barometer innovations are chi2-DOWNWEIGHTED, not dropped: baro is a
 * persistent absolute reference and must not deadlock on a persistent offset.
 *
 * This file also contains local_gnss_alt_t, a separate 1-state Kalman filter
 * estimating the slowly varying offset between a LOCAL height (above the
 * caller's vertical datum) and the GNSS ellipsoid height, i.e. the ellipsoid
 * height of the datum origin. It turns any local height source into an absolute
 * height reference during GNSS outages: h_ellipsoid ~ h_local + offset. Three
 * height systems are in play: the local datum (zero at the origin), the
 * barometric ISA altitude and the ellipsoid.
 *
 * Frames/conventions as everywhere in this library: body FRD, nav NED, Hamilton
 * quaternion (q[0] = w), timestamps int64 microseconds. _Except_: h and v are
 * positive UP, i.e. the negated NED down axis.
 *
 * All memory is part of the baro_alt_t struct, no heap is used.
 *
 */

/** @addtogroup baro_alt
 *  @{ */

#ifndef BARO_ALT_H
#define BARO_ALT_H

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** State vector size (h, v, a_b). */
#define BARO_ALT_STATES 3

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/** Timestamp in microseconds (monotonic), same convention as ins. */
typedef int64_t baro_alt_time_us_t;

/** @brief Filter configuration.
 *
 *  Every field left at 0 is replaced by its default, an all-zero
 *  config is valid. */
typedef struct
{
    /* Initial state std.-devs. (v and a_b start at zero; h starts at
       the h_init passed to baro_alt_init). */
    float h_init_stddev_m;           /**< [m] (0 -> default), can be
                                       overridden per init call */
    float v_init_stddev_mps;         /**< [m/s] (0 -> default) */
    float acc_bias_init_stddev_mps2; /**< [m/s^2] (0 -> BARO_ALT_DEFAULT_AB_STDDEV_MPS2) */

    /* Noise model: continuous-time spectral noise densities, scaled by
       dt_sec each prediction step (Q = density^2 * dt). */
    float acc_noise_mps2_sqrthz;      /**< accelerometer noise density
                                          sigma_a [m/s^2/sqrt(Hz)]
                                          (0 -> default, see
                                          sensor_defaults.h) */
    float acc_bias_drift_mps2_sqrthz; /**< bias drift density sigma_b
                                          [m/s^2/sqrt(Hz)] (0 -> default) */
    float h_process_noise_m_sqrthz;   /**< small additional process noise
                                          density sigma_h injected directly on
                                          h [m/sqrt(Hz)] (0 -> default).
                                          Without it h only inherits
                                          uncertainty indirectly through Phi's
                                          v->h coupling; this term covers
                                          unmodelled height dynamics */
    /* Barometer */
    float baro_stddev_m;  /**< default altitude stddev of a
                               barometer sample [m] (0 -> default),
                               can be overridden per sample in
                               baro_alt_update() */
    float chi2_threshold; /**< chi2 outlier gate for the scalar
                               altitude update, downweights
                               instead of dropping
                               (0 -> 3.8415 = chi2inv(0.95,1)) */

    /* Zero-velocity update (see baro_alt_zero_velocity_update). */
    float zupt_stddev_mps; /**< 1-sigma of the zero-velocity
                                pseudo-measurement [m/s] (0 -> default),
                                can be overridden per call */

    /* Precision restart watchdog (REQ-BARO-021): a fail-safe that marks the
       filter uninitialized once its own reported height or vertical-velocity
       1-sigma stays implausibly large past a warm-up, e.g. after a long
       barometer outage left it dead-reckoning on the accelerometer.
       is_initialized then flips false (same mechanism as the health check):
       nav_suite re-bootstraps from the live stream, a standalone caller must
       re-init. ON by default. */
    bool precision_restart_disable; /**< true -> never auto-restart on
                                         degraded precision */
    float restart_h_stddev_m;       /**< height 1-sigma restart threshold
                                         [m] (0 -> default, < 0 -> not checked) */
    float restart_v_stddev_mps;     /**< vertical-velocity 1-sigma restart
                                         threshold [m/s]
                                         (0 -> default, < 0 -> not checked) */
    float restart_warmup_sec;       /**< grace period after init before
                                         the check arms [s] (0 -> default) */

    /* Global outlier-rejection override (REQ-SYS-015, diagnostics/analysis
     * only): the barometric altitude update is then fused at its nominal
     * variance regardless of the innovation size. Set through nav_suite_init()'s
     * propagation of ins_options_t.chi2_disable (REQ-SUITE-011). */
    bool chi2_disable; /**< true -> never chi2-downweight a fusion */
} baro_alt_config_t;

/** @brief Filter instance. Initialise with baro_alt_init(). */
typedef struct
{
    baro_alt_config_t cfg; /**< resolved config (defaults filled in) */

    /* State [h m, v m/s, a_b m/s^2] and covariance as UDU
       factorisation: P = U * diag(d) * U' (Bierman/Thornton routines
       from kalman_udu.h, same backend as INSLIB/ahrs). */
    float x[BARO_ALT_STATES];                   /**< state vector [h, v, a_b] */
    float U[BARO_ALT_STATES * BARO_ALT_STATES]; /**< unit upper triangular factor */
    float d[BARO_ALT_STATES];                   /**< diagonal factor */

    /** Datum zero point in ISA altitude [m]: ISA altitude of the anchor
        pressure sample minus h_init, so h = isa(p) - h0_baro_m. The
        current barometric ISA altitude estimate is h0_baro_m + x[0]. */
    float h0_baro_m;

    /* Timing / status */
    baro_alt_time_us_t t_init;          /**< init timestamp (restart warm-up base) */
    baro_alt_time_us_t t_last;          /**< last state propagation epoch */
    baro_alt_time_us_t t_last_baro_fix; /**< timestamp of the last accepted
                                             barometer fusion (see
                                             baro_alt_deadreckoning_ms) */
    uint32_t epoch;                     /**< baro_alt_update() calls since init */
    bool     is_initialized;            /**< false after init failure or a
                                             non-finite state (health check) */

    /* Diagnostics (monotonic since baro_alt_init) */
    uint32_t n_fuse_fail;     /**< failed fusion attempts */
    uint32_t n_restart;       /**< times the precision watchdog marked the
                                   filter uninitialized (REQ-BARO-021) */
    uint32_t n_invalid_input; /**< epochs/samples dropped at the
                                   baro_alt_update() boundary (non-finite
                                   acc/quaternion or non-finite/implausible
                                   pressure) */
    uint32_t n_zupt;          /**< zero-velocity updates applied
                                   (baro_alt_zero_velocity_update) */

    /** Initial bias-prior consistency check (REQ-BARO-024, diagnostic only):
     *  while the caller feeds zero-velocity updates the platform stands still,
     *  so the averaged up-acceleration input is the vertical accelerometer bias
     *  itself and can be held against cfg.acc_bias_init_stddev_mps2. A hit
     *  means that prior is far too tight for the actual sensor. */
    uint32_t n_acc_bias_prior_exceeded;

    /* Averaging window for the check above. The stillness signal is the
       zero-velocity feed itself (baro_alt has no detector of its own,
       see REQ-SUITE-015): a gap in that feed ends the window. */
    baro_alt_time_us_t t_last_zupt;          /**< time of the last zero-velocity
                                                  update (0 = none yet) */
    baro_alt_time_us_t acc_bias_prior_since; /**< window start (valid while
                                                  acc_bias_prior_count > 0) */
    float    acc_bias_prior_sum;             /**< sum of a_up over the window [m/s^2] */
    uint32_t acc_bias_prior_count;           /**< samples in the sum */

    /* Outlier downweighting (REQ-SYS-006, REQ-BARO-017). */
    uint32_t n_downweighted; /**< baro_alt_fuse_baro() calls whose chi2
                                  test tripped and was downweighted
                                  rather than dropped, diagnostic only,
                                  0 whenever chi2_disable is set (the
                                  test is skipped entirely then) */

    /* Telemetry-only snapshot of the two raw quantities the filter consumes
       internally (REQ-BARO-025): the effective up-acceleration control input
       a_up (gravity removed, BEFORE the a_b correction is added back) and the
       datum-corrected barometric altitude measurement z = h_baro - h0 last
       handed to the fusion. Neither is otherwise observable from outside. */
    float last_a_up_mps2;  /**< last predict-step up-acceleration input [m/s^2] */
    bool  have_last_a_up;  /**< true once a predict step has run */
    float last_h_meas_m;   /**< last offset-corrected barometric altitude
                                measurement fed to the fusion [m] */
    bool have_last_h_meas; /**< true once a barometer fusion has run */

    /* Handoff from baro_alt_predict_step() to baro_alt_correct_step() for one
       epoch (REQ-BARO-026), same rationale as ins_t.step_ctx: the pressure
       sample is plausibility-checked exactly once by baro_alt_predict_step(),
       and baro_alt_correct_step() cannot re-derive that afterwards. */
    struct
    {
        float pressure_pa;
        float baro_stddev_m;
        bool  baro_valid;
        bool  active; /**< baro_alt_correct_step() has work to do */
    } step_ctx;       /**< per-epoch scratch handed from baro_alt_predict_step()
                           to baro_alt_correct_step() */

    /** Logging-only bookkeeping (see log.h): pure rate-detection so the optional
     *  LOG_* calls in baro_alt.c stay informative instead of flooding the sink.
     *  Never read or acted on by the filter itself. */
    struct
    {
        baro_alt_time_us_t t_last_gap_warn;            /**< throttle for the "barometer
                                                            gap" warning (0 = none yet) */
        baro_alt_time_us_t t_last_acc_bias_prior_warn; /**< throttle for the
                                                            bias-prior warning
                                                            (0 = none yet) */
    } log_state;
} baro_alt_t;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Initialise (or reset) the filter.
     *
     *  The pressure sample is the altitude anchor: its ISA altitude corresponds
     *  to the height h_init_m above the caller's vertical datum (pass 0 for
     *  "height above start"). The sample is consumed here and must not be fed
     *  to baro_alt_update() again.
     *
     *
     *  @param[in,out] b The filter instance (may be uninitialised memory).
     *  @param[in] cfg Configuration (0 fields -> defaults).
     *  @param[in] t Initial timestamp [us].
     *  @param[in] pressure_pa Anchor static pressure [Pa].
     *  @param[in] h_init_m Height of the anchor sample above the vertical
     *                      datum [m], positive up (0 = datum at start).
     *  @param[in] h_init_stddev_m 1-sigma uncertainty of h_init_m [m],
     *                             0 -> cfg.h_init_stddev_m (default).
     *  @return 0 on success, -1 on invalid arguments, a non-finite h_init
     *          or a non-finite/implausible anchor pressure. */
    int baro_alt_init(baro_alt_t* b, const baro_alt_config_t* cfg, baro_alt_time_us_t t,
                      float pressure_pa, float h_init_m, float h_init_stddev_m);

    /** @brief Feed one epoch and advance the filter.
     *
     *  Call at the IMU rate. Performs (in order): state + covariance
     *  propagation driven by the vertical acceleration derived from acc_mps2
     *  and q_bn (skipped for larger gaps), then, if baro_valid, the barometric
     *  altitude fusion.
     *
     *  Timestamps must be monotonic, a backwards step re-anchors the internal
     *  clock and skips the epoch. Non-finite accelerometer or quaternion input
     *  drops the epoch, a non-finite or implausible pressure sample is ignored
     *  (both counted in n_invalid_input).
     *
     *  @param[in,out] b The filter instance.
     *  @param[in] t Timestamp [us].
     *  @param[in] acc_mps2 Accelerometer (specific force), body frame FRD
     *                      [m/s^2].
     *  @param[in] q_bn Body-to-NED attitude quaternion (Hamilton,
     *                  q[0] = w), e.g. from INSLIB or an AHRS instance.
     *  @param[in] pressure_pa Static pressure [Pa], ignored unless
     *                         baro_valid.
     *  @param[in] baro_stddev_m 1-sigma altitude uncertainty of this
     *                           pressure sample [m], 0 -> cfg default.
     *  @param[in] baro_valid Is a barometer sample present this epoch? */
    void baro_alt_update(baro_alt_t* b, baro_alt_time_us_t t, const float acc_mps2[3],
                         const float q_bn[4], float pressure_pa, float baro_stddev_m,
                         bool baro_valid);

    /** @brief baro_alt_predict_step() propagated the covariance this call
     *  (and, if requested, filled phi_out). */
#define BARO_ALT_EPOCH_COV_PROPAGATED (1 << 0)
    /** @brief The epoch was fully dropped (not initialized / non-finite
     *  accel or attitude / backward time jump). The following
     *  baro_alt_correct_step() call is a no-op. */
#define BARO_ALT_EPOCH_DROPPED (1 << 1)

    /** @brief Time-propagation half of baro_alt_update(): the
     *  accelerometer-driven state propagation plus the covariance prediction.
     *
     *  Must be followed by exactly one baro_alt_correct_step() call before the
     *  next baro_alt_predict_step(): baro_alt_t.step_ctx has room for exactly
     *  one pending epoch's sanitized pressure sample. Calling it twice silently
     *  overwrites step_ctx, so the skipped epoch's barometer sample is never
     *  fused and its health/precision checks never run. Batching predicts buys
     *  nothing: the covariance prediction runs on every call anyway.
     *
     *  Split out from baro_alt_update() so a caller (e.g. an offline RTS
     *  smoother) can sample the covariance between prediction and correction.
     *
     *  @param[in,out] b The filter instance.
     *  @param[in] t Timestamp [us].
     *  @param[in] acc_mps2 Accelerometer (specific force), body frame FRD
     *                      [m/s^2].
     *  @param[in] q_bn Body-to-NED attitude quaternion (Hamilton, q[0] = w).
     *  @param[in] pressure_pa Static pressure [Pa], ignored unless
     *                         baro_valid.
     *  @param[in] baro_stddev_m 1-sigma altitude uncertainty of this
     *                           pressure sample [m], 0 -> cfg default.
     *  @param[in] baro_valid Is a barometer sample present this epoch?
     *  @param[out] phi_out Optional (nullable) buffer for the discrete-time
     *      state transition matrix used this call, BARO_ALT_STATES x
     *      BARO_ALT_STATES column-major. Only filled when the return value
     *      has BARO_ALT_EPOCH_COV_PROPAGATED set.
     *  @return Bitwise OR of BARO_ALT_EPOCH_* flags. */
    int baro_alt_predict_step(baro_alt_t* b, baro_alt_time_us_t t, const float acc_mps2[3],
                              const float q_bn[4], float pressure_pa, float baro_stddev_m,
                              bool baro_valid, float* phi_out);

    /** @brief Fusion half of baro_alt_update(): the barometric altitude
     *  fusion (if a valid sample was seen), then health/precision checks.
     *  A no-op if the matching baro_alt_predict_step() dropped the epoch
     *  (see BARO_ALT_EPOCH_DROPPED) or was never called.
     *
     *  @param[in,out] b The filter instance. */
    void baro_alt_correct_step(baro_alt_t* b);

    /** @brief Fuse a zero-velocity pseudo-measurement (ZUPT) into the vertical
     *  channel: z = 0 observing only v (H = [0 1 0]).
     *
     *  Call when the platform is known to be standing still. Under nav_suite
     *  this is driven by the same zero-rotation trigger that feeds the ARS/AHRS
     *  (REQ-SUITE-015): a stationary platform has no vertical velocity either.
     *
     *  NOT gated on the filter's own velocity estimate: the caller's stillness
     *  detection is the authority. Implausible innovations are
     *  chi2-DOWNWEIGHTED, not dropped (same policy as the barometer fusion).
     *  No-op if the filter is not initialized.
     *
     *  @param[in,out] b The filter instance.
     *  @param[in] stddev_mps 1-sigma of the pseudo-measurement [m/s],
     *                        0 -> cfg.zupt_stddev_mps. */
    void baro_alt_zero_velocity_update(baro_alt_t* b, float stddev_mps);

    /** @brief Height above start, positive up.
     *  @param[in] b The filter instance.
     *  @param[out] h_m Height above start [m], positive up.
     *  @return false if the filter is not initialized/healthy. */
    bool baro_alt_get_height(const baro_alt_t* b, float* h_m);

    /** @brief Vertical velocity, positive up.
     *  @param[in] b The filter instance.
     *  @param[out] v_mps Vertical velocity [m/s], positive up.
     *  @return false if the filter is not initialized/healthy. */
    bool baro_alt_get_velocity(const baro_alt_t* b, float* v_mps);

    /** @brief Vertical acceleration correction a_b (added to the measured
     *  up-acceleration in the prediction).
     *  @param[in] b The filter instance.
     *  @param[out] acc_bias_mps2 Acceleration correction [m/s^2].
     *  @return false if the filter is not initialized/healthy. */
    bool baro_alt_get_acc_bias(const baro_alt_t* b, float* acc_bias_mps2);

    /** @brief Effective up-acceleration input of the last predict step (specific
     *  force rotated to NED, gravity removed, BEFORE the a_b correction is added
     *  back in). Telemetry/diagnostics only: while standing still this dithers
     *  around the current accelerometer bias, so plotting it together with
     *  baro_alt_get_acc_bias() shows the bias estimate converge.
     *  @param[in] b The filter instance.
     *  @param[out] a_z_mps2 Up-acceleration input [m/s^2].
     *  @return false if the filter is not initialized/healthy or no
     *          predict step has run yet. */
    bool baro_alt_get_measurement_a_z(const baro_alt_t* b, float* a_z_mps2);

    /** @brief Datum-corrected barometric altitude measurement (z = h_baro - h0)
     *  last handed to the fusion, i.e. the raw barometer reading in the same
     *  height system as baro_alt_get_height(), before the Kalman update.
     *  Telemetry/diagnostics only.
     *  @param[in] b The filter instance.
     *  @param[out] h_meas_m Barometric altitude measurement [m].
     *  @return false if the filter is not initialized/healthy or no
     *          barometer fusion has run yet. */
    bool baro_alt_get_measurement_h(const baro_alt_t* b, float* h_meas_m);

    /** @brief Time since the last accepted barometer fusion.
     *
     *  While a fresh barometer sample keeps arriving this stays near 0, once
     *  the barometer stalls (or was never available) it grows and the height
     *  estimate is riding purely on the accelerometer.
     *
     *  @param[in] b The filter instance.
     *  @return Milliseconds since the last accepted barometer fusion, or
     *          -1 if the filter is not initialized. Never negative
     *          otherwise (a backwards time step is clamped to 0). */
    int baro_alt_deadreckoning_ms(const baro_alt_t* b);

    /** @brief Filtered barometric ISA altitude (h0_baro_m + h), i.e. the height
     *  in the barometric height system rather than above the caller's vertical
     *  datum.
     *
     *  This is NOT an absolute height: it carries the weather-dependent pressure
     *  offset and the ISA model error, and refers to the ISA sea level. For an
     *  absolute height use the local height plus the estimated offset (see
     *  local_gnss_alt_t, nav_suite_get_height_ellipsoid).
     *
     *  @param[in] b The filter instance.
     *  @param[out] h_isa_m Barometric ISA altitude [m], positive up.
     *  @return false if the filter is not initialized/healthy. */
    bool baro_alt_get_isa_altitude(const baro_alt_t* b, float* h_isa_m);

    /** @brief Barometric altitude from static pressure via the
     *  international standard atmosphere formula (p0 = 101325 Pa).
     *  @param[in] pressure_pa Static pressure [Pa].
     *  @return Altitude above the p0 level [m]. */
    float baro_alt_pressure_to_altitude(float pressure_pa);

    /** @brief Is this a plausible static pressure sample?
     *  (finite and within ~16 km altitude .. below sea level)
     *  @param[in] pressure_pa Static pressure [Pa].
     *  @return true if plausible. */
    bool baro_alt_pressure_plausible(float pressure_pa);

    /* ==========================================================================
     * local_gnss_alt: 1-state local-height-to-ellipsoid offset filter
     * ==========================================================================
     */

    /** @brief Offset filter configuration (0 fields -> defaults). */
    typedef struct
    {
        float rw_stddev_mps;           /**< random walk of the offset
                                            [m/sqrt(s)] (0 -> default): slow
                                            enough to average measurement noise,
                                            fast enough for the ISA-model error,
                                            which grows with the height
                                            EXCURSION and outruns weather drift
                                            by an order of magnitude. A
                                            non-drifting source (e.g.
                                            lighthouse) may use less */
        float local_stddev_m;          /**< default local height stddev [m]
                                            (0 -> default) */
        float chi2_threshold;          /**< chi2 outlier gate on the innovation,
                                            downweights instead of dropping
                                            (0 -> 3.8415 = chi2inv(0.95,1)) */
        float min_update_interval_sec; /**< minimum time between two fusions [s]
                                   (0 -> default). Pairs arriving faster are
                                   counted (n_decimated) and ignored, so the
                                   offset is nudged at most once per
                                   interval */
        float stddev_inflation_factor; /**< factor applied to both the local
                                   height and the GNSS 1-sigma before they are
                                   combined into the measurement variance
                                   (0 -> default), derating the fusion so a
                                   single pair cannot move the offset far */

        /* Global outlier-rejection override (REQ-SYS-015, diagnostics/analysis
         * only): the offset measurement is then fused at its nominal variance
         * regardless of the innovation size. Set through nav_suite_init()'s
         * propagation of ins_options_t.chi2_disable (REQ-SUITE-011). */
        bool chi2_disable; /**< true -> never chi2-downweight a fusion */
    } local_gnss_alt_config_t;

    /** @brief Offset filter instance. Initialise with local_gnss_alt_init().
     *
     *  Estimates o = h_gnss_ellipsoid - h_local as a scalar random-walk Kalman
     *  filter, where h_local is a height above the caller's vertical datum. The
     *  offset is the ellipsoid height of the datum origin, so an absolute
     *  height follows as h_ell = h_local + o, for any source of h_local.
     *
     *  Feed it with the local height source that survives a GNSS outage, not
     *  with a GNSS-derived local height: the offset must absorb that source's
     *  drift against the ellipsoid, otherwise the drift reappears uncorrected
     *  in h_ell exactly when GNSS is gone.
     *
     *  n = 1, so the covariance is a plain variance and the update is written
     *  out directly (no UDU factorisation needed). */
    typedef struct
    {
        local_gnss_alt_config_t cfg; /**< resolved config (defaults filled) */

        float offset_m; /**< o = h_gnss_ell - h_local [m], i.e. the
                             ellipsoid height of the datum origin */
        float var_m2;   /**< variance of the offset [m^2] */

        baro_alt_time_us_t t_last;         /**< last propagation epoch */
        uint32_t           epoch;          /**< local_gnss_alt_update() calls since init */
        bool               is_initialized; /**< false before init / after a
                                                 health-check failure */

        uint32_t n_invalid_input; /**< pairs dropped at the update
                                       boundary (non-finite inputs
                                       or bad GNSS accuracy) */
        uint32_t n_decimated;     /**< otherwise-valid pairs skipped by
                                       the min_update_interval_sec gate */
        uint32_t n_downweighted;  /**< fusions whose chi2 test tripped and were
                                       downweighted rather than dropped
                                       (REQ-SYS-006, REQ-BARO-017); 0 whenever
                                       chi2_disable is set */

        baro_alt_time_us_t t_last_downweight_warn; /**< throttle for the chi2
                                       downweight warning (0 = none yet) */
    } local_gnss_alt_t;

    /** @brief Initialise (or reset) the offset filter from the first pair.
     *
     *  @param[in,out] g The filter instance (may be uninitialised memory).
     *  @param[in] cfg Configuration (0 fields -> defaults).
     *  @param[in] t Timestamp of the pair [us].
     *  @param[in] h_local_m Local height above the vertical datum [m],
     *                       positive up.
     *  @param[in] local_stddev_m 1-sigma of the above [m], 0 -> cfg default.
     *  @param[in] h_gnss_ell_m GNSS ellipsoid height [m].
     *  @param[in] gnss_stddev_m 1-sigma vertical GNSS accuracy [m], > 0.
     *  @return 0 on success, -1 on invalid arguments. */
    int local_gnss_alt_init(local_gnss_alt_t* g, const local_gnss_alt_config_t* cfg,
                            baro_alt_time_us_t t, float h_local_m, float local_stddev_m,
                            float h_gnss_ell_m, float gnss_stddev_m);

    /** @brief Feed one baro/GNSS pair.
     *
     *  Pairs arriving faster than cfg.min_update_interval_sec since the last
     *  fusion are decimated (counted in n_decimated, state untouched).
     *  Otherwise propagates the random walk over the time since the last fusion
     *  and fuses the offset measurement z = h_gnss_ell - h_local with variance
     *  (local_stddev * stddev_inflation_factor)^2 +
     *  (gnss_stddev * stddev_inflation_factor)^2. Implausible innovations are
     *  chi2-downweighted, not dropped. Pairs with non-finite altitudes or
     *  non-positive/non-finite GNSS accuracy are dropped (counted in
     *  n_invalid_input), a backwards timestamp re-anchors the clock and skips
     *  the pair.
     *
     *  Both h_local_m and h_gnss_ell_m must refer to the same instant:
     *  extrapolate the local height to the GNSS time of validity if the fix is
     *  delayed.
     *
     *  @param[in,out] g The filter instance.
     *  @param[in] t Timestamp of the pair [us].
     *  @param[in] h_local_m Local height above the vertical datum [m],
     *                       positive up.
     *  @param[in] local_stddev_m 1-sigma of the above [m]; 0 -> cfg default.
     *  @param[in] h_gnss_ell_m GNSS ellipsoid height [m].
     *  @param[in] gnss_stddev_m 1-sigma vertical GNSS accuracy [m], > 0. */
    void local_gnss_alt_update(local_gnss_alt_t* g, baro_alt_time_us_t t, float h_local_m,
                               float local_stddev_m, float h_gnss_ell_m, float gnss_stddev_m);

    /** @brief Current offset o = h_gnss_ell - h_local (the ellipsoid height
     *  of the datum origin) and its 1-sigma uncertainty. Either output
     *  pointer may be NULL.
     *  @param[in] g The filter instance.
     *  @param[out] offset_m Offset o = h_gnss_ell - h_local [m].
     *  @param[out] stddev_m 1-sigma uncertainty of offset_m [m].
     *  @return false if the filter is not initialized/healthy. */
    bool local_gnss_alt_get(const local_gnss_alt_t* g, float* offset_m, float* stddev_m);

#ifdef __cplusplus
}
#endif

#endif /* BARO_ALT_H */
/** @} */
