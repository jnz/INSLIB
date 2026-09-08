/** @file ahrs.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Attitude (and heading) reference system Kalman Filter.
 *
 * An error-state Kalman filter that corrects an attitude quaternion and a
 * gyroscope bias vector with accelerometer (leveling) and optional magnetometer
 * (heading) measurements. Runs standalone next to the full ins filter (see
 * nav_suite.h for a wrapper that runs both).
 *
 * Two modes:
 *  - AHRS_MODE_ARS ("directional gyro"): roll/pitch filter. Yaw is integrated
 *    from the gyroscope but never corrected, i.e. it drifts with the
 *    (estimated) z gyro bias error.
 *    Error state (5): [droll dpitch dbg_x dbg_y dbg_z]
 *  - AHRS_MODE_AHRS: roll/pitch/yaw filter. Yaw is stabilized with
 *    tilt-compensated magnetometer heading measurements. The heading reference
 *    is magnetic north until a position is supplied via ahrs_set_position(),
 *    after which the World Magnetic Model declination is added and the
 *    estimated yaw is relative to TRUE north.
 *    Error state (6): [droll dpitch dyaw dbg_x dbg_y dbg_z]
 *
 * Angular rates and accelerometer measurements are expected in the body frame
 * with X forward (roll axis), Y right (pitch axis), Z down (yaw axis), right
 * hand convention. Attitude output is body-to-NED (Tait-Bryan ZYX).
 *
 * Reasonable defaults for the sensor noise model are provided (any config field
 * left at 0 is replaced by its default), but for optimal performance supply the
 * sensor specific noise and bias random walk parameters.
 *
 * Zero-rotation update (ZARU, opt-in, ahrs_update's zero_rotation_update
 * parameter): a caller-supplied "we know omega_b_nb == 0 this epoch" flag fuses
 * a direct gyro-bias measurement. Unlike ins, this filter has no
 * position/velocity state, so it cannot detect stillness on its own; the
 * trigger must come from outside (nav_suite passes through ins's own
 * velocity-aware auto-ZUPT/ZARU detector).
 *
 * Notes:
 *  - The Earth rotation rate (15 deg/h) is not corrected.
 *  - Make sure the initial attitude is roughly correct, otherwise the filter
 *    takes a while to converge. Use ahrs_leveling_from_acc() /
 *    ahrs_mag_heading() on a static sample.
 *  - The accelerometer influence is tuned with acc_freq_hz: too low and the
 *    gyro bias tracking suffers, too high and acceleration phases corrupt the
 *    attitude.
 *
 * All memory is part of the ahrs_t struct - no heap is used.
 *
 */

/** @addtogroup ahrs
 *  @{ */

#ifndef AHRS_H
#define AHRS_H

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** Max. error-state vector size (AHRS mode: 6 unknowns). */
#define AHRS_UNKNOWNS_MAX 6

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/** Timestamp in microseconds (monotonic), same convention as ins. */
typedef int64_t ahrs_time_us_t;

/** @brief Filter mode (fixed per instance, set in ahrs_config_t). */
typedef enum
{
    AHRS_MODE_ARS  = 0, /**< roll/pitch only, yaw integrated freely */
    AHRS_MODE_AHRS = 1  /**< roll/pitch/yaw, magnetometer-aided yaw */
} ahrs_mode_t;

/** @brief Filter configuration and initial state.
 *
 * Every noise/tuning field left at 0 is replaced by defaults.  Only the
 * initial attitude and its standard deviation are mandatory. */
typedef struct
{
    ahrs_mode_t mode; /**< ARS or AHRS (see ahrs_mode_t) */

    /* Initial state */
    float rpy_init_rad[3];             /**< initial roll/pitch/yaw [rad] */
    float rpy_init_stddev_rad[3];      /**< stddev of the above [rad], > 0.
                                            yaw entry unused in ARS mode. */
    float gyr_bias_init_rps[3];        /**< initial gyro bias [rad/s] */
    float gyr_bias_init_stddev_rps[3]; /**< stddev of the above [rad/s]
                                            (0 -> defaults) */

    /* Sensor noise model */
    float gyr_noise_psd;  /**< gyro spectral noise density [(rad/s)/sqrt(Hz)]
                               (0 -> default, shared with ins, see
                               sensor_defaults.h) */
    float gyr_bias_rw;    /**< gyro bias random walk [(rad/s^2)/sqrt(Hz)]
                               (0 -> default, shared with ins, see
                               sensor_defaults.h) */
    float acc_noise_mps2; /**< accelerometer measurement noise stddev [m/s^2],
                               should cover the worst-case vibration and
                               centrifugal forces of the target environment.
                               (0 -> default) */

    /* Tuning */
    float kalman_update_dt_sec;    /**< covariance-prediction period [s];
                                        the attitude integration still runs
                                        at the full IMU rate. Also rate-limits
                                        the zero-rotation update. (0 -> default) */
    float acc_freq_hz;             /**< max. rate of accelerometer
                                        corrections [Hz] (0 -> default) */
    float gravity_diff_penalty;    /**< downweight accelerometer measurements
                                        whose norm differs from g, [stddev per
                                        m/s^2 of difference]. Higher = more
                                        resistant to acceleration phases.
                                        (0 -> default, < 0 -> 0) */
    float chi2_threshold;          /**< chi2 outlier gate for the
                                        accelerometer update, applied
                                        per scalar measurement row
                                        (0 -> 3.8415 = chi2inv(0.95,1)) */
    float acc_cutoff_freq_hz;      /**< accelerometer low-pass cut-off
                                        [Hz], tolerance against vibration
                                        at the cost of delay (0 -> default) */
    float acc_reject_gravity_mps2; /**< hard-reject the accelerometer leveling
                                        update when the low-passed | |f| - g |
                                        exceeds this [m/s^2]: a specific force
                                        that far from gravity is a
                                        maneuver/shock, not a leveling
                                        reference, and is transient enough that
                                        a hard drop cannot deadlock (cf.
                                        REQ-AHRS-007). (0 -> default,
                                        < 0 -> disabled) */

    /* Magnetometer (AHRS mode only). Expected in uT to match the World Magnetic
       Model, but the heading fusion uses only the field DIRECTION, so any
       consistent unit works. The unit only matters for the optional
       field-strength gate below, which requires uT. */
    float mag_yaw_stddev_rad; /**< stddev of the derived heading
                                   measurement [rad]
                                   (0 -> default) */
    float mag_freq_hz;        /**< max. rate of magnetometer
                                   corrections [Hz] (0 -> default) */
    float mag_chi2_threshold; /**< chi2 outlier gate for the scalar
                                   heading update
                                   (0 -> 3.8415 = chi2inv(0.95,1)) */

    /* Magnetometer field-strength disturbance gate (AHRS mode). OPT-IN: off
       unless mag_field_check_enable is set. When enabled AND a position is
       known (ahrs_set_position), the measured field magnitude is compared
       against the WMM total-field expectation and a sample deviating by more
       than mag_field_tolerance has its heading noise inflated. REQUIRES the
       magnetometer in uT. */
    bool mag_field_check_enable; /**< true -> arm the field-strength
                                      gate (requires mag in uT) */
    float mag_field_tolerance;   /**< max |B|-deviation fraction before
                                      downweighting (0 -> default) */

    /* Zero-rotation update (ZARU, REQ-AHRS-016): opt-in, driven by the caller's
       explicit "omega_b_nb == 0 this epoch" flag, e.g. nav_suite's own flag OR
       ins's velocity-aware auto-ZUPT/ZARU detector (this filter has no
       position/velocity state, so unlike ins it cannot detect stillness on its
       own). Directly measures the gyro bias states from the gyro averaged over
       the current trigger run. Earth rotation rate is NOT corrected. */
    float zero_rot_stddev_rps; /**< 1-sigma zero-rotation-update noise
                                    [rad/s] (0 -> default) */

    /* Auto-ZARU stillness fallback (REQ-AHRS-017): ON by default, hence the
       negative name - a zeroed config arms it. Detects stillness from the
       gyro/accelerometer alone (primarily the short-window sample stddev), so
       it is available from the first samples. What it cannot do is see
       constant-velocity travel: a body cruising in a straight line has an IMU
       signature identical to a standstill. */
    bool auto_zaru_disable;          /**< true -> never arm the fallback
                                          (armed by default) */
    float auto_zaru_static_gyr_rps;  /**< max. |bias-corrected gyro| to
                                          call the platform static
                                          [rad/s] (0 -> default) */
    float auto_zaru_static_acc_mps2; /**< max. |acc| - g to call the
                                          platform static [m/s^2]
                                          (0 -> default) */
    float auto_zaru_dwell_sec;       /**< time static before the
                                          fallback arms [s] (0 -> default),
                                          measured from the variance window's
                                          first stillness verdict, not from
                                          when the magnitude gate alone goes
                                          static */
    /* Variance criterion of the fallback (REQ-AHRS-017): per-axis RMS sample
       stddev of the raw IMU over a short window. This is the PRIMARY stillness
       statement, the two magnitude gates above are only loose bounds next to it
       (see sensor_defaults.h). Raising these far above any plausible sensor
       noise leaves the magnitude bounds alone in charge. */
    float auto_zaru_static_gyr_stddev_rps;  /**< max RMS gyro stddev [rad/s]
                                                 (0 -> default) */
    float auto_zaru_static_acc_stddev_mps2; /**< max RMS accelerometer stddev
                                                 [m/s^2] (0 -> default) */

    /* Attitude-precision restart watchdog (REQ-AHRS-023): a fail-safe that
       marks the filter uninitialized once its own reported attitude 1-sigma
       stays implausibly large past a warm-up, e.g. after a long outage of the
       correcting sensor. is_initialized then flips false (same mechanism as the
       health check): nav_suite re-bootstraps on the next epoch, a standalone
       caller must call ahrs_init() again. ON by default, set
       precision_restart_disable to opt out. */
    bool precision_restart_disable;  /**< true -> never auto-restart on
                                          degraded precision */
    float restart_att_stddev_rad[3]; /**< per-axis attitude 1-sigma
                                          restart threshold [rad]
                                          (roll/pitch/yaw; yaw entry
                                          unused in ARS mode). 0 ->
                                          default (45/45/90 deg); < 0 ->
                                          that axis is not checked */
    float restart_warmup_sec;        /**< grace period after init before
                                          the precision check arms [s]
                                          (0 -> default) */

    /* Global outlier-rejection override (REQ-SYS-015, diagnostics/analysis
     * only): when set, both the accelerometer and magnetometer updates are
     * fused at their nominal variance regardless of the innovation size. Set
     * through nav_suite_init()'s propagation of ins_options_t.chi2_disable
     * (REQ-SUITE-011), or directly by a standalone caller. Off by default. */
    bool chi2_disable; /**< true -> never chi2-downweight a fusion */
} ahrs_config_t;

/** @brief Filter instance. Zero the struct, then call ahrs_init(). */
typedef struct
{
    ahrs_config_t cfg; /**< resolved config (defaults filled in) */
    int           n;   /**< active error-state size: 5 or 6 */

    /* Nominal state */
    float q[4];            /**< body-to-NED quaternion (Hamilton, q[0]=w) */
    float gyr_bias_rps[3]; /**< gyro bias estimate [rad/s] */

    /* World Magnetic Model aiding (set via ahrs_set_position, 0 until a
       position is known, then yaw is referenced to true north). */
    float declination_rad;       /**< true = magnetic + declination */
    float mag_field_expected_uT; /**< WMM total field; 0 -> no gate */

    /* Error-state covariance as UDU factorisation: P = U * diag(d) * U'
       (U is n x n unit upper triangular, column-major). Fusion uses the
       Bierman/Thornton "square root" routines from kalman_udu.h */
    float U[AHRS_UNKNOWNS_MAX * AHRS_UNKNOWNS_MAX]; /**< unit upper triangular factor */
    float d[AHRS_UNKNOWNS_MAX];                     /**< diagonal factor */

    /* Accelerometer low-pass state [m/s^2]. */
    float acc_lowpass_mps2[3]; /**< low-passed specific force, body frame */

    /* Zero-rotation-update accumulator: raw gyro summed over the current
       auto-ZARU fallback's confirmed-stillness run (filled by
       ahrs_auto_zaru_detect, not by the fusion itself, so it already holds a
       real average by the time the fallback's dwell first allows a fuse),
       consumed and cleared by the fusion. Stays at 0 -- so the fusion falls
       back to the instantaneous sample -- for a manual trigger with no such
       run behind it. */
    float    zaru_gyr_sum[3]; /**< raw gyro sum over the current stillness run */
    uint32_t zaru_gyr_count;  /**< samples in the sum */

    /* Velocity-blind auto-ZARU fallback state (see
       ahrs_config_t.auto_zaru_disable). 0 if not currently static, else
       the time the current stillness run started. */
    ahrs_time_us_t auto_zaru_static_since; /**< 0, or stillness-run start time */
    ahrs_time_us_t auto_zaru_var_since;    /**< 0 if the variance window has not
                                                yet confirmed this stillness run,
                                                else the time it first did: the
                                                dwell timer is measured from here,
                                                not from auto_zaru_static_since, so
                                                zaru_gyr_sum/count above always
                                                hold a real average by the time the
                                                first trigger can fire */

    /* Variance-based stillness criterion of that fallback (REQ-AHRS-017, see
       ahrs_static_variance_update). Welford accumulator over a tumbling window:
       index 0..2 gyro xyz, 3..5 accelerometer xyz, raw samples. */
    ahrs_time_us_t static_var_window_since; /**< start of the current window */
    float          static_var_mean[6];      /**< Welford running mean */
    float          static_var_m2[6];        /**< Welford sum of squared deviations */
    uint32_t       static_var_count;        /**< samples in the current window
                                                 (0 = window not started) */
    bool static_var_ok;                     /**< verdict latched from the last COMPLETED
                                                 window; false until the first one
                                                 completes */

    /* Whether the last ahrs_update() saw a zero-rotation trigger at all
       (caller's flag OR'd with the auto-ZARU fallback), regardless of whether
       the rate-limited fusion fired. Read back with ahrs_zaru_applied(). */
    bool last_zaru_trigger; /**< zero-rotation trigger of the last epoch */

    /* Handoff from ahrs_predict_step() to ahrs_correct_step() for one epoch
       (REQ-AHRS-025), same rationale as ins_t.step_ctx: the mag sample is
       sanitized exactly once by ahrs_predict_step(), and ahrs_correct_step()
       cannot re-derive the epoch's timing decisions afterwards. */
    struct
    {
        ahrs_time_us_t t;
        float          gyr_rps[3];
        float          acc_mps2[3];
        float          mag_b[3];
        bool           mag_valid;
        bool           zero_rotation_update;
        bool           active; /**< ahrs_correct_step() has work to do */
    } step_ctx;                /**< per-epoch scratch handed from ahrs_predict_step()
                                    to ahrs_correct_step() */

    /* Timing / status */
    ahrs_time_us_t t_init;                 /**< init timestamp (precision-restart warm-up base) */
    ahrs_time_us_t t_last_gyr;             /**< last gyro integration */
    ahrs_time_us_t t_last_cov_predict;     /**< last covariance prediction */
    ahrs_time_us_t t_last_acc_fusion;      /**< last accelerometer correction */
    ahrs_time_us_t t_last_mag_fusion;      /**< last magnetometer correction */
    ahrs_time_us_t t_last_zero_rot_fusion; /**< last zero-rotation update */
    uint32_t       epoch;                  /**< ahrs_update() calls since init */
    bool           is_initialized;         /**< false after init failure or a
                                                non-finite state (health check) */

    /* Diagnostics (monotonic since ahrs_init) */
    uint32_t n_fuse_fail;     /**< failed fusion attempts */
    uint32_t n_invalid_input; /**< epochs/samples dropped at the
                                   ahrs_update() boundary because
                                   of non-finite values (NaN/Inf) */
    uint32_t n_acc_rejected;  /**< accelerometer leveling updates
                                   hard-dropped by the gravity-magnitude
                                   gate (REQ-AHRS-022) */
    uint32_t n_restart;       /**< times the attitude-precision watchdog
                                   marked the filter uninitialized
                                   (REQ-AHRS-023) */

    /* Outlier downweighting (REQ-SYS-006, REQ-AHRS-019). */
    uint32_t n_downweighted; /**< ahrs_fuse() calls whose chi2 test tripped and
                                  was downweighted rather than dropped;
                                  diagnostic only, 0 whenever chi2_disable is
                                  set */

    /* Overconfidence / covariance-collapse watchdog (REQ-AHRS-020): the
       attitude analogue of ins's REQ-NAV-040. Flags when the reported attitude
       1-sigma becomes physically implausible. Purely diagnostic. Covers
       roll/pitch in both modes, plus yaw in AHRS mode. min_att_stddev_deg is
       the smallest per-axis value seen since ahrs_init. */
    bool     overconfident;      /**< latched: some epoch's attitude stddev tripped the floor */
    uint32_t n_overconfident;    /**< epochs where an attitude stddev tripped the floor */
    float    min_att_stddev_deg; /**< smallest per-axis attitude stddev seen [deg] */

    /** Logging-only bookkeeping (see log.h): pure rate-detection so the optional
     *  LOG_* calls in ahrs.c stay informative instead of flooding the sink.
     *  Never read or acted on by the filter itself. */
    struct
    {
        ahrs_time_us_t t_last_mag_gap_warn;         /**< throttle for the "no magnetometer,
                                                         yaw stddev growing" warning
                                                         (0 = none yet) */
        ahrs_time_us_t t_yaw_stddev_window;         /**< runaway window start (0 = not
                                                         yet sampled), AHRS_MODE_AHRS only */
        ahrs_time_us_t t_gyr_bias_window;           /**< runaway window start (0 = not
                                                         yet sampled) */
        ahrs_time_us_t t_last_gyr_bias_sanity_warn; /**< throttle for the gyro-bias
                                                          sanity-bound warning
                                                          (0 = none yet) */
        float yaw_stddev_window_deg;                /**< yaw stddev at window start [deg] */
        float gyr_bias_window_dps;                  /**< |gyro bias| at window start [deg/s] */
    } log_state;
} ahrs_t;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Initialise (or reset) the filter.
     *
     *  @param[in,out] a The filter instance (may be uninitialised memory).
     *  @param[in] cfg Configuration and initial state (0 fields -> defaults).
     *  @param[in] t Initial timestamp [us].
     *  @return 0 on success, -1 on invalid configuration. */
    int ahrs_init(ahrs_t* a, const ahrs_config_t* cfg, ahrs_time_us_t t);

    /** @brief Feed one IMU epoch and advance the filter.
     *
     *  Call at the IMU rate. Performs (in order): covariance prediction,
     *  attitude integration with the bias-corrected
     *  angular rate, accelerometer leveling fusion (throttled to
     *  acc_freq_hz), magnetometer heading fusion in AHRS mode (throttled
     *  to mag_freq_hz), and, if triggered, a zero-rotation update
     *  (throttled like the covariance prediction).
     *
     *  Timestamps must be monotonic; a backwards step is ignored (the epoch only
     *  re-anchors the internal clocks and clears the zero-rotation accumulator),
     *  a gap larger than 0.2 s skips the attitude integration. Non-finite
     *  gyro/acc samples drop the epoch, a non-finite magnetometer sample is
     *  ignored (both counted in n_invalid_input).
     *
     *  @param[in,out] a The filter instance.
     *  @param[in] t Timestamp [us].
     *  @param[in] gyr_rps Gyroscope measurement, body frame [rad/s].
     *  @param[in] acc_mps2 Accelerometer (specific force), body frame [m/s^2].
     *  @param[in] mag_b Magnetometer, body frame. Expected in uT (WMM units);
     *                   only the direction is used for heading, so any
     *                   consistent unit works unless the field-strength gate
     *                   is enabled (that needs uT). Pass NULL if unavailable.
     *                   Ignored in ARS mode.
     *  @param[in] zero_rotation_update True if the caller knows
     *                   omega_b_nb == 0 this epoch (an external stillness
     *                   detector, e.g. nav_suite's explicit flag or ins's
     *                   auto-ZUPT/ZARU detector; see ahrs_config_t's ZARU
     *                   comment for why this filter needs that externally). */
    void ahrs_update(ahrs_t* a, ahrs_time_us_t t, const float gyr_rps[3], const float acc_mps2[3],
                     const float mag_b[3], bool zero_rotation_update);

    /** @brief ahrs_predict_step() propagated the covariance this call (and,
     *  if requested, filled phi_out). */
#define AHRS_EPOCH_COV_PROPAGATED (1 << 0)
    /** @brief The epoch was fully dropped (not initialized / non-finite
     *  gyro or accel / backward time jump). The following
     *  ahrs_correct_step() call is a no-op. */
#define AHRS_EPOCH_DROPPED (1 << 1)

    /** @brief Time-propagation half of ahrs_update(): quaternion
     *  integration plus the (throttled) Kalman covariance prediction.
     *
     *  Must be followed by exactly one ahrs_correct_step() call before the next
     *  ahrs_predict_step(): ahrs_t.step_ctx has room for exactly one pending
     *  epoch's sanitized mag sample/gyro/accel/zero-rotation trigger. Calling
     *  ahrs_predict_step() twice silently overwrites step_ctx, so the skipped
     *  epoch's mag sample and zero-rotation trigger are never fused and its
     *  health/precision checks never run. There is no reason to batch predicts
     *  either: the covariance prediction is already throttled to
     *  cfg.kalman_update_dt_sec regardless of call rate.
     *
     *  Split out from ahrs_update() so a caller (e.g. an offline RTS smoother)
     *  can sample the covariance between prediction and correction.
     *
     *  @param[in,out] a The filter instance.
     *  @param[in] t Timestamp [us].
     *  @param[in] gyr_rps Gyroscope measurement, body frame [rad/s].
     *  @param[in] acc_mps2 Accelerometer (specific force), body frame [m/s^2].
     *  @param[in] mag_b Magnetometer, body frame, forwarded to the matching
     *      ahrs_correct_step() call, same contract as ahrs_update()'s.
     *      Pass NULL if unavailable.
     *  @param[in] zero_rotation_update Forwarded to the matching
     *      ahrs_correct_step() call, same contract as ahrs_update()'s.
     *  @param[out] phi_out Optional (nullable) buffer for the discrete-time
     *      state transition matrix used this call, n x n column-major
     *      (n = a->n, bounded by AHRS_UNKNOWNS_MAX). Only filled when the
     *      return value has AHRS_EPOCH_COV_PROPAGATED set.
     *  @return Bitwise OR of AHRS_EPOCH_* flags. */
    int ahrs_predict_step(ahrs_t* a, ahrs_time_us_t t, const float gyr_rps[3],
                          const float acc_mps2[3], const float mag_b[3], bool zero_rotation_update,
                          float* phi_out);

    /** @brief Fusion half of ahrs_update(): accelerometer leveling,
     *  magnetometer heading (AHRS mode) and zero-rotation update, using the
     *  measurement set by the matching ahrs_predict_step() call, then
     *  health/precision checks. A no-op if the matching ahrs_predict_step()
     *  dropped the epoch (see AHRS_EPOCH_DROPPED) or was never called.
     *
     *  @param[in,out] a The filter instance. */
    void ahrs_correct_step(ahrs_t* a);

    /** @brief Get current attitude as roll/pitch/yaw.
     *  @param[in] a The filter instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad].
     *  @return false if the filter is not initialized/healthy. */
    bool ahrs_get_rpy(const ahrs_t* a, float* roll_rad, float* pitch_rad, float* yaw_rad);

    /** @brief Get current attitude quaternion.
     *  @param[in] a The filter instance.
     *  @param[out] q Body-to-NED quaternion (Hamilton, q[0] = w).
     *  @return false if the filter is not initialized/healthy. */
    bool ahrs_get_quaternion(const ahrs_t* a, float q[4]);

    /** @brief Get current gyroscope bias estimate.
     *  @param[in] a The filter instance.
     *  @param[out] gyr_bias_rps Gyroscope bias [rad/s].
     *  @return false if the filter is not initialized/healthy. */
    bool ahrs_get_bias_gyr(const ahrs_t* a, float gyr_bias_rps[3]);

    /** @brief Get the 1-sigma uncertainty of the current roll/pitch/yaw.
     *  @param[in] a The filter instance.
     *  @param[out] roll_stddev_rad Roll 1-sigma [rad].
     *  @param[out] pitch_stddev_rad Pitch 1-sigma [rad].
     *  @param[out] yaw_stddev_rad Yaw 1-sigma [rad]; 0 in ARS mode (no
     *              filtered yaw state -- yaw free-integrates).
     *  @return false if the filter is not initialized/healthy. */
    bool ahrs_get_rpy_stddev(const ahrs_t* a, float* roll_stddev_rad, float* pitch_stddev_rad,
                             float* yaw_stddev_rad);

    /** @brief Get the 1-sigma uncertainty of the current gyroscope bias.
     *  @param[in] a The filter instance.
     *  @param[out] gyr_bias_stddev_rps Gyroscope bias 1-sigma [rad/s].
     *  @return false if the filter is not initialized/healthy. */
    bool ahrs_get_bias_gyr_stddev(const ahrs_t* a, float gyr_bias_stddev_rps[3]);

    /** @brief Report whether the velocity-blind auto-ZARU fallback
     *  (ahrs_config_t.auto_zaru_disable) currently considers the platform
     *  stationary (dwelled long enough to be feeding the zero-rotation
     *  update, see REQ-AHRS-017). Always false if the fallback is
     *  disabled or the filter is not initialized. Diagnostic/telemetry
     *  use, does not reflect an externally-supplied trigger.
     *  @param[in] a The filter instance.
     *  @return true if the fallback detector is currently armed. */
    bool ahrs_auto_zaru_active(const ahrs_t* a);

    /** @brief Report whether the last ahrs_update() saw a zero-rotation
     *  trigger: the caller's zero_rotation_update flag OR'd with the auto-ZARU
     *  fallback's decision (REQ-AHRS-024). Unlike ahrs_auto_zaru_active() this
     *  covers BOTH sources and reflects the decision the fusion acted on, which
     *  makes it the stillness signal other filters key off (nav_suite drives the
     *  vertical zero-velocity update from it, REQ-SUITE-015).
     *  @param[in] a The filter instance.
     *  @return true if a zero-rotation trigger was present on the last epoch. */
    bool ahrs_zaru_applied(const ahrs_t* a);

    /** @brief Enable/disable the velocity-blind auto-ZARU at runtime
     *  (REQ-AHRS-017), independent of the cfg.auto_zaru_disable set at
     *  ahrs_init. The ARS/AHRS-side counterpart of
     *  ins_set_auto_zupt_disable(), see there for the rationale.
     *
     *  Disabling clears the in-progress dwell timer and the latched
     *  variance-window verdict, so a stale evaluation cannot fire the instant
     *  the fallback is re-enabled. Does not affect an externally-triggered
     *  update.
     *  @param[in,out] a The filter instance.
     *  @param[in] disable true -> the fallback can never arm until called
     *      again with false. */
    void ahrs_set_auto_zaru_disable(ahrs_t* a, bool disable);

    /** @brief Supply the current position so the filter can apply the World
     *  Magnetic Model (declination + expected field strength).
     *
     *  Optional and may be called at any time, e.g. later in the run, once
     *  a first GNSS fix (or any coarse position) becomes available. Until it
     *  is called the AHRS references magnetic north.
     *
     *  Switching on (or changing) the declination re-frames the estimate
     *  DETERMINISTICALLY: the nominal attitude is instantly yaw-rotated by the
     *  declination change, so the heading STEPS to true north rather than
     *  slewing there over the following magnetometer updates. Covariance and
     *  gyro-bias states are untouched (a known rotation carries no new
     *  uncertainty).
     *
     *  If mag_field_check_enable is set in the config, this also arms the
     *  field-strength gate with the WMM expected field (requires mag in uT).
     *  A non-finite argument is ignored.
     *
     *  @param[in,out] a The filter instance.
     *  @param[in] lat_rad Latitude [rad].
     *  @param[in] lon_rad Longitude [rad].
     *  @param[in] year Decimal year (e.g. 2027.5) for the WMM epoch. */
    void ahrs_set_position(ahrs_t* a, float lat_rad, float lon_rad, float year);

    /* ------------------------------------------------------------------------
     * Initialisation heuristics (static helpers, no filter instance needed)
     * ------------------------------------------------------------------------
     */

    /** @brief Estimate roll/pitch from a single accelerometer sample
     *  (leveling). Works best if the sensor is at rest (specific force =
     *  reaction to gravity only).
     *
     *  @param[in] acc_mps2 Specific force, body frame [m/s^2].
     *  @param[out] roll_rad Estimated roll [rad].
     *  @param[out] pitch_rad Estimated pitch [rad]. */
    void ahrs_leveling_from_acc(const float acc_mps2[3], float* roll_rad, float* pitch_rad);

    /** @brief Tilt-compensated magnetic heading.
     *
     *  Yaw of the body relative to magnetic north (declination is NOT
     *  applied), derived by de-tilting the body-frame field with the given
     *  roll/pitch.
     *
     *  @param[in] mag_b Magnetometer, body frame (any unit).
     *  @param[in] roll_rad Current roll [rad].
     *  @param[in] pitch_rad Current pitch [rad].
     *  @return Heading [rad] in (-pi, pi]. */
    float ahrs_mag_heading(const float mag_b[3], float roll_rad, float pitch_rad);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */
/** @} */
