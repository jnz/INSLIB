/** @file ins.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief 15/18-state error-state Kalman filter for 3D navigation.
 *
 * State vector:
 *   [0..2]   Position error in n-frame (NED) [m]
 *   [3..5]   Velocity error in n-frame (NED) [m/s]
 *   [6..8]   Attitude error (small-angle rpy) [rad]
 *   [9..11]  Accelerometer bias [m/s^2]
 *   [12..14] Gyroscope bias [rad/s]
 *   Optional:
 *   [15..17] Magnetometer bias [uT or mT]
 *
 * Covariance is stored as a UDU factorisation (U is 15x15 unit upper
 * triangular, d is a 15x1 diagonal vector). Fusion uses the
 * Bierman/Thornton "square root" routines from kalman_udu.h.
 *
 * Nominal state (all single precision, the filter works in a local n-frame):
 *   - pos_local in n-frame (NED), relative to the origin set at init
 *   - vel in n-frame (NED)
 *   - qbn Hamilton quaternion (body-to-NED)
 *   - acc/gyr bias in body frame
 *
 * Double precision is only used for the absolute-position anchor: the n-frame
 * origin (origin_llh) and the current lat/lon/height (latlonh, book-kept
 * incrementally from the float n-frame deltas), both geodetic. The hot path
 * (strapdown, predict, fusion corrections) is float-only, and ECEF is a form
 * of the interface rather than of the filter: it is converted where a caller
 * hands one in or asks for one, never per epoch (REQ-NAV-080).
 *
 * The filter keeps a ring-buffer history of recent states to support delayed
 * GNSS measurements (time-of-validity up to INS_MAX_DELAY_MS in the past).
 *
 * All memory is allocated as part of the ins_t struct - no heap is used.
 *
 *
 */

/** @addtogroup ins
 *  @{ */

#ifndef INS_H
#define INS_H

/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** Base state vector size (15 unknowns, no baro-offset state). */
#define INS_UNKNOWNS 15

/** State vector size with the optional magnetometer hard-iron bias
 *  states (see ins_options_t.estimate_mag_bias). */
#define INS_UNKNOWNS_MAG 18

/** Compile-time maximum state size: all covariance/history arrays are
 *  dimensioned to this. The active size is chosen per instance at init
 *  (15, or 18 with estimate_mag_bias). */
#ifndef INS_UNKNOWNS_MAX
#define INS_UNKNOWNS_MAX INS_UNKNOWNS_MAG
#endif

/** Compile-time maximum column count r of the process noise input matrix
 *  G (n x r) handed to kalman_udu_predict(): 12 IMU noise columns plus one
 *  extra column per error state, minus the 6 accelerometer/gyroscope bias
 *  states that are already covered by the IMU columns. */
#define INS_NOISE_COLS_MAX (12 + INS_UNKNOWNS_MAX - 6)

/** Maximum tolerated measurement delay (history buffer size). */
#define INS_MAX_DELAY_MS 500

/** Consecutive epochs dropped as older than INS_MAX_DELAY_MS that are read as
 *  a restarted time source rather than reordering (REQ-NAV-070). Isolated
 *  reordering arrives in short bursts, a restart never ends. Sized so a
 *  plausible burst cannot reach it while recovery still costs well under a
 *  second at typical IMU rates. */
#define INS_TIME_RESTART_EPOCHS 50

/** History ring-buffer size. Sized to cover INS_MAX_DELAY_MS at the
 *  INS_HISTORY_DT_MS grid spacing plus phase-jitter margin. Both constants move
 *  together. */
#ifndef INS_HISTORY_ITEMS_MAX
#define INS_HISTORY_ITEMS_MAX 30
#endif

/** Minimum time between history entries [ms]. The delayed-measurement
 *  anchoring relies on the error-state being ~constant over the delay
 *  (Phi ~ I), so a coarse grid is sufficient: the nearest anchor is at most
 *  INS_HISTORY_DT_MS/2 off the true time of validity. */
#ifndef INS_HISTORY_DT_MS
#define INS_HISTORY_DT_MS 20
#endif

/** Auto-init IMU sample buffer size (for initial alignment). */
#ifndef INS_AUTOINIT_SAMPLES_MAX
#define INS_AUTOINIT_SAMPLES_MAX 64
#endif

/** Min. Kalman epochs until ins_is_ready() returns true. */
#define INS_MIN_KALMAN_EPOCHS_UNTIL_READY 4
/** Min. runtime until ins_is_ready() returns true [ms]. */
#define INS_MIN_RUNTIME_UNTIL_READY_MS 1500

/* @satisfies REQ-NAV-001 */
#define INS_IDX_POS 0  /**< position sub-state offset */
#define INS_IDX_VEL 3  /**< velocity sub-state offset */
#define INS_IDX_RPY 6  /**< attitude (small-angle error) sub-state offset */
#define INS_IDX_ACC 9  /**< accelerometer bias sub-state offset */
#define INS_IDX_GYR 12 /**< gyroscope bias sub-state offset */
#define INS_IDX_MAG 15 /**< magnetometer hard-iron bias offset (18-state mode) */

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/** Timestamp in microseconds (monotonic). */
typedef int64_t ins_time_us_t;

/** @brief 3x1 Measurement (accel, gyro, magnetometer, GNSS velocity). */
typedef struct
{
    float data[3];     /**< actual measurement [see unit in context]. */
    float Qll_diag[3]; /**< measurement noise. For IMUs PSD, per axis (diagonal)
                            [unit^2/Hz]. For acc/gyr, 0 (per axis) uses a
                            conservative consumer-MEMS ARW/VRW default
                            instead of injecting zero process noise.
                            For GNSS velocity standard deviation. */
    bool is_valid;     /**< is this measurement usable? */
} ins_meas3_t;

/** @brief GNSS position measurement (geodetic, with n-frame covariance).
 *
 *  The fix is stated the way receivers report it and the way the fusion works
 *  in: the residual is the geodetic difference to the filter's own anchor, so
 *  nothing is converted on the epoch path (REQ-NAV-079). A source that is
 *  natively ECEF, an RTK solution or UBX-NAV-HPPOSECEF, converts once with
 *  ins_ecef_to_latlonh() before offering the fix, which keeps that cost out of
 *  the filter's worst-case epoch instead of making it depend on the caller. */
typedef struct
{
    double llh[3];        /**< latitude [rad], longitude [rad], height above
                               the WGS84 ellipsoid [m] */
    float Qll_ned[3 * 3]; /**< full covariance in NED frame [m^2]
                               (3x3, column-major, symmetric). For a
                               diagonal-only receiver just set the
                               diagonal entries (rest zero). */
    bool is_valid;        /**< is this measurement usable? */
} ins_meas_gnss_pos_t;

/** @brief Local NED position measurement (e.g. indoor tracking systems:
 *  lighthouse/SteamVR, UWB, motion capture, total station).
 *
 *  The position is expressed in the filter's local n-frame, i.e. the
 *  same frame as ins_state_t.pos_local (NED, origin = position given
 *  to ins_init). Aligning the external system's frame with this frame
 *  (origin offset + rotation into NED) is the caller's responsibility. */
typedef struct
{
    float pos_ned[3];     /**< measured position, local n-frame [m] */
    float Qll_ned[3 * 3]; /**< full covariance (3x3, column-major,
                               symmetric) [m^2]. For a diagonal-only
                               sensor just set the diagonal entries. */
    bool is_valid;        /**< is this measurement usable? */
} ins_meas_local_pos_t;

/** @brief Absolute yaw (heading) measurement.
 *
 *  E.g. from a lighthouse/mocap pose, a dual-antenna GNSS heading or a gyro
 *  compass. ZYX (Tait-Bryan) yaw of the body relative to NED, same convention
 *  as ins_get_rpy().
 *
 *  Any single-turn convention is accepted and normalized internally. A heading
 *  beyond one full turn is treated as a unit or unwrapping error: dropped and
 *  counted in ins_diag_t.n_invalid_input rather than wrapped into a
 *  plausible-looking but wrong heading. */
typedef struct
{
    float yaw_rad;    /**< measured yaw [rad], |yaw| <= 2*pi */
    float stddev_rad; /**< 1-sigma measurement uncertainty [rad] */
    bool  is_valid;   /**< is this measurement usable? */
} ins_meas_yaw_t;

/** @brief Barometric static pressure measurement. */
typedef struct
{
    float pressure_pa; /**< static pressure [Pa] */
    float stddev_m;    /**< 1-sigma altitude uncertainty of the
                            derived barometric altitude [m]
                            (0 -> consumer default) */
    bool is_valid;     /**< is this measurement usable? */
} ins_meas_baro_t;

/** @brief GNSS velocity measurement (NED, with full covariance). */
typedef struct
{
    float vel_ned[3];     /**< velocity in NED [m/s] */
    float Qll_ned[3 * 3]; /**< full covariance in NED frame [(m/s)^2]
                               (3x3, column-major, symmetric). For a
                               diagonal-only receiver just set the
                               diagonal entries (rest zero). */
    bool is_valid;        /**< is this measurement usable? */
} ins_meas_gnss_vel_t;

/** @brief Absolute speed measurement (REQ-NAV-068).
 *
 *  The MAGNITUDE of the platform velocity, with no direction attached: an
 *  OBD-II vehicle speed, a wheel-odometry pulse rate, a Doppler log. Fused as
 *  h(x) = ||v_n||, so it constrains only the component of the velocity error
 *  along the current direction of travel.
 *
 *  Unsigned by contract. A negative value is treated as a unit or sign error,
 *  dropped and counted in ins_diag_t.n_invalid_input.
 *
 *  stddev_mps is the PER-SAMPLE uncertainty only. The systematic scale error is
 *  a property of the installation and is configured once via
 *  ins_options_t.speed_scale / speed_stddev_rel. */
typedef struct
{
    float speed_mps;  /**< measured ground speed [m/s], >= 0 */
    float stddev_mps; /**< per-sample 1-sigma [m/s] (0 -> default) */
    bool  is_valid;   /**< is this measurement usable? */
} ins_meas_speed_t;

/** @brief Optional external attitude/gyro-bias initial attitude for ins's own
 * auto-init bootstrap and re-acquisition (REQ-NAV-048), e.g. from a
 * continuously-running AHRS. */
typedef struct
{
    bool  is_valid;               /**< false -> ignored entirely */
    float roll_rad;               /**< [rad] */
    float pitch_rad;              /**< [rad] */
    float stddev_roll_rad;        /**< 1-sigma [rad], roll/pitch are only used
                                        together, both must be > 0 */
    float stddev_pitch_rad;       /**< 1-sigma [rad] */
    float yaw_rad;                /**< [rad], only used if stddev_yaw_rad > 0.
                                        Same heading contract as
                                        ins_meas_yaw_t: any single-turn
                                        convention, |yaw| <= 2*pi, else the
                                        yaw part of the hint is dropped */
    float stddev_yaw_rad;         /**< 1-sigma [rad], 0 -> no yaw hint (e.g. the
                                        source is a gyro-only filter with no
                                        absolute heading) */
    float gyr_bias_rps[3];        /**< [rad/s] */
    float stddev_gyr_bias_rps[3]; /**< 1-sigma [rad/s] per axis, an axis
                                        with stddev <= 0 is left untouched */
} ins_meas_att_hint_t;

/** @brief Bundle of measurements for a single epoch.
 *
 *  All fields are optional, only those marked is_valid are consumed.
 *  Set the full struct to zero and then fill what you have. */
typedef struct
{
    ins_time_us_t timestamp; /**< local timestamp [us] */

    /* IMU (strapdown integration) */
    float strapdown_dt_sec; /**< dt since last IMU measurement.
                                 If > 0 and both acc+gyr are valid,
                                 the strapdown is run. */
    ins_meas3_t acc;        /**< accelerometer [m/s^2] */
    ins_meas3_t gyr;        /**< gyroscope [rad/s] */
    ins_meas3_t mag;        /**< magnetometer [uT or mT, consistent
                                   with magnetic model] */

    /* GNSS */
    ins_meas_gnss_pos_t gnss_pos; /**< GNSS position measurement */
    ins_meas_gnss_vel_t gnss_vel; /**< GNSS velocity measurement */
    float               gnss_Qll_pos_vel_ned[3 * 3];
    /**< Optional position/velocity cross-covariance:
         element (i,j) = cov(pos_ned_i, vel_ned_j)
         [m^2/s] (3x3, column-major). Together with
         the two Qll_ned blocks this forms the full
         6x6 covariance of [pos, vel]. Leave zero if
         unknown/uncorrelated. Only used when both
         gnss_pos and gnss_vel are valid. */
    float gnss_leverarm_b[3]; /**< GNSS antenna lever arm,
                                   body frame [m] */

    /* GNSS time-of-validity, informational only. The filter does not use
     * it for latency compensation: set gnss_delay_ms, which states how old
     * the measurement is and is history-anchored. */
    int gps_week;      /**< 0 if unknown */
    int gps_itow_ms;   /**< 0 if unknown */
    int gnss_delay_ms; /**< >0: this many ms old. 0 = now. */

    /* Local position aiding (e.g. lighthouse/UWB/mocap) */
    ins_meas_local_pos_t local_pos;               /**< local NED position */
    float                local_pos_leverarm_b[3]; /**< sensor lever arm,
                                                       body frame [m] */
    int local_pos_delay_ms;                       /**< >0: measurement is this
                                                       many ms old (residual is
                                                       anchored in the history,
                                                       like GNSS). 0 = now. */

    /* Absolute yaw aiding (e.g. lighthouse pose, dual-antenna GNSS) */
    ins_meas_yaw_t yaw;          /**< yaw (heading) measurement */
    int            yaw_delay_ms; /**< >0: measurement is this many
                                      ms old (history-anchored).
                                      0 = now. */

    /** External attitude/gyro-bias seed for auto-init and re-acquisition
     *  (REQ-NAV-048), e.g. nav_suite's ARS/AHRS (REQ-SUITE-016). */
    ins_meas_att_hint_t att_hint;

    /* Barometer (static pressure), consumed by ins itself and, through
       nav_suite, by the parallel baro_alt vertical channel filter. ins uses it
       twice: its presence during auto-init decides whether the vertical
       position channel is driven barometrically or by GNSS (REQ-NAV-053), and
       if it is, every sample is fused as a scalar height measurement
       (REQ-NAV-054). Set is_valid only on an epoch carrying a NEW sample: a
       latched flag re-fuses the same reading and makes the filter
       overconfident. */
    ins_meas_baro_t baro; /**< static pressure sample */

    /* Absolute speed aiding (REQ-NAV-068): OBD-II vehicle speed, wheel
       odometry, Doppler log. Same "new sample only" contract as the barometer
       above - a latched is_valid re-fuses one reading every epoch. */
    ins_meas_speed_t speed;          /**< scalar ground-speed sample */
    int              speed_delay_ms; /**< >0: measurement is this many ms
                                          old (residual anchored in the
                                          history, like GNSS). 0 = now. */

    /* Virtual measurements */
    bool zero_velocity_update; /**< we know v == 0 */
    bool zero_rotation_update; /**< we know omega == 0 */

} ins_measurements_t;

/** @brief Initial values supplied to ins_init(). */
typedef struct
{
    ins_time_us_t time; /**< initial timestamp [us] */
    /* Start position and velocity, in the frames the filter works in: it
       anchors geodetically and mechanizes in NED (REQ-NAV-081). A caller
       holding ECEF converts once at startup with ins_ecef_to_latlonh().

       An all-zero block is a legal start, namely where the equator meets
       the prime meridian at ellipsoid height 0, and is NOT rejected as
       "forgotten". Under ins_options_t.auto_init, which is how most callers
       run, the position is a provisional anchor that the first usable fix
       replaces anyway. */
    double llh[3];               /**< lat [rad], lon [rad], height over the WGS84
                                      ellipsoid [m] */
    float vel_ned[3];            /**< velocity in the local NED frame [m/s] */
    float rpy_init_rad[3];       /**< initial roll/pitch/yaw [rad],
                                      same convention as ahrs_config_t.rpy_init_rad */
    float acc_bias_init_mps2[3]; /**< initial accelerometer bias [m/s^2] */
    float gyr_bias_init_rps[3];  /**< initial gyroscope bias [rad/s],
                                    same convention as ahrs_config_t.gyr_bias_init_rps */

    /* Initial std.-devs. A field left at (or explicitly set to) <= 0 is NOT
       "perfectly known": ins_init() resolves it to a generous default
       (REQ-NAV-049). A literal 0 variance would otherwise permanently lock that
       state's Kalman gain at 0. */
    float pos_init_stddev_m;         /**< [m]. 0 -> default. */
    float vel_init_stddev_mps;       /**< [m/s]. 0 -> default. */
    float rpy_init_stddev_rad[3];    /**< roll/pitch/yaw [rad], independently
                                          settable (same convention as
                                          ahrs_config_t.rpy_init_stddev_rad).
                                          Each axis left at <= 0 resolves to its
                                          own default, no cross-axis
                                          inheritance: roll/pitch (leveling) and
                                          yaw (external fix / magnetometer /
                                          unknown) come from very different
                                          accuracy sources. */
    float acc_bias_init_stddev_mps2; /**< [m/s^2]. 0 -> default. */
    float gyr_bias_init_stddev_rps;  /**< [rad/s]. 0 -> default. */

    /* Process noise std.-devs.: spectral densities, NOT plain rates - each is
       squared into a PSD and scaled by dt during prediction (ins_predict,
       "Q = [PSD] * dt"). A state with unit [X] therefore takes a density in
       [X]/sqrt(s), not [X]/s. Same "0 -> default" treatment (REQ-NAV-049) as
       the initial std.-devs above. Each of these is an EXTRA process-noise term
       on top of the IMU-driven prediction noise already propagated through the
       strapdown kinematics from ins_meas3_t.Qll_diag, not a replacement. */
    float pos_pred_stddev_m_sqrts;         /**< [m/sqrt(s)]. 0 -> default */
    float vel_pred_stddev_mps_sqrts;       /**< [m/s/sqrt(s)]. 0 -> default */
    float rpy_pred_stddev_rad_sqrts;       /**< [rad/sqrt(s)]. 0 -> default. Added
                                           on top of the gyro-ARW-driven attitude
                                           process noise (see above), not instead
                                           of it. */
    float acc_bias_pred_stddev_mps2_sqrts; /**< [m/s^2/sqrt(s)] (random walk).
                                     0 -> conservative consumer-MEMS default. */
    float gyr_bias_pred_stddev_rps_sqrts;  /**< [rad/s/sqrt(s)] (random walk).
                                      0 -> conservative consumer-MEMS default. */

    /* Virtual measurement std.-devs. 0 is not "perfectly confident" here
       either: it is an invalid measurement variance ins_fuse() rejects
       outright, so a caller leaving these at 0 would get a ZUPT/ZARU that never
       fuses. ins_init resolves a <= 0 value to a default (REQ-NAV-049). */
    float zero_vel_stddev_mps; /**< [m/s]. 0 -> default. */
    float zero_rot_stddev_rps; /**< [rad/s]. 0 -> default. */

    /* Reference vectors */
    float gravity_n[3];  /**< gravity in NED [m/s^2] (or
                              zeros -> compute from model) */
    float magnetic_n[3]; /**< magnetic model vector in NED */

    /* Magnetometer hard-iron bias states (18-state mode, see
     * ins_options_t.estimate_mag_bias, ignored otherwise). Appended at
     * the end to keep the struct layout offset-stable. */
    float mag_bias_init_stddev_ut;       /**< [uT] (0 -> default) */
    float mag_bias_pred_stddev_ut_sqrts; /**< [uT/sqrt(s)] (random walk).
                                     0 -> default. */
} ins_init_t;

/** @brief Filter options.
 *
 *  New fields are APPENDED at the end of the struct, never inserted, so the
 *  layout (and the ctypes mirror) stays offset-stable. */
typedef struct
{
    float kalman_update_dt_sec;    /**< covariance-prediction period [s],
                                        the strapdown still runs at the full
                                        IMU rate (0 -> 1/20 = 20 Hz) */
    float max_prediction_time_sec; /**< max. allowed dt [s]. 0 -> default
                                        (REQ-NAV-043); a literal 0 would make
                                        every forward epoch a time-jump
                                        reset. */

    /* GNSS fusion thresholds: measurements with larger stddev are ignored. Each
     * 0 -> a beginner-friendly default (REQ-NAV-043), NOT "reject everything".
     * Entering or leaving the 3D solution is decided by the separate
     * gnss_start_max_* / gnss_stop_max_* sets (REQ-NAV-051, REQ-NAV-052). */
    float gnss_max_horizontal_pos_stddev_m;   /**< [m]   (0 -> default) */
    float gnss_max_vertical_pos_stddev_m;     /**< [m]   (0 -> default) */
    float gnss_max_horizontal_vel_stddev_mps; /**< [m/s] (0 -> default) */
    float gnss_max_vertical_vel_stddev_mps;   /**< [m/s] (0 -> default) */

    /* Magnetometer rate limiting. The magnetometer is treated as a long-term
     * heading anchor rather than a per-epoch yaw sensor, so the default
     * throttles it well below a typical sensor output rate: see
     * INS_DEFAULT_MAG_MIN_DELAY_MS (ins.c). */
    int magnetometer_min_delay_ms; /**< min. time between magnetometer
                                        fusions [ms]. 0 -> default,
                                        negative -> no rate limit
                                        (fuse every sample). */

    bool allow_unlimited_deadreckoning; /**< survive long IMU-only periods:
                                             never degrade is_ready and never
                                             trigger re-acquisition */
    float max_deadreckoning_sec;        /**< max. IMU-only coasting time without
                                             absolute position aiding before
                                             ins_is_ready() degrades to false, the
                                             next usable fix then re-acquires
                                             instead of fusing (see
                                             ins_deadreckoning_ms). 0 -> default,
                                             ignored when
                                             allow_unlimited_deadreckoning is
                                             set. */

    /* Auto-initialization: if set, ins_init leaves the filter in a collecting
     * state and bootstraps the nominal state from the first usable position fix
     * (GNSS or local NED). Roll/pitch come from accelerometer leveling, yaw
     * from an external yaw measurement, else the magnetometer, else left
     * unknown. Position/velocity come from the fix. The platform does not need
     * to be still: the reported roll/pitch uncertainty is widened whenever the
     * window is not quasi-static (REQ-NAV-047). */
    bool  auto_init;                       /**< enable auto-initialization */
    float auto_init_window_sec;            /**< IMU leveling window [s] (0 -> default) */
    float auto_init_static_gyr_rps;        /**< max |gyro| over the window still
                                              classed as quasi-static [rad/s]
                                              (0 -> default). Above this the
                                              bootstrap still proceeds, but with
                                              the inflated roll/pitch stddev
                                              (REQ-NAV-047). */
    float auto_init_static_acc_mps2;       /**< max ||f|-g| over the window still
                                              classed as quasi-static [m/s^2] (0
                                              -> default), see
                                              auto_init_static_gyr_rps. */
    float auto_init_moving_rpy_stddev_rad; /**< floor on the roll/pitch initial
                                              stddev [rad] used when the leveling
                                              window is not quasi-static
                                              (REQ-NAV-047), 0 -> default. Only
                                              ever raises rpy_init_stddev_rad. */
    float gnss_init_dwell_sec;             /**< before entering 3D the position
                                              aiding must have passed the
                                              gnss_start_max_* entry gate
                                              continuously for this long AND
                                              delivered >= 1 fix/s over the window
                                              (REQ-NAV-045, REQ-NAV-051). 0 ->
                                              default. Gates the cold start and
                                              every re-arm, so a marginal fix
                                              cannot flap 3D on/off. */
    bool gnss_init_dwell_disable;          /**< true -> no dwell: bootstrap on the
                                              first usable fix (pre-REQ-NAV-045
                                              behaviour, e.g. controlled indoor
                                              starts that want instant 3D). */
    bool auto_reacquire_disable;           /**< true -> opt out of the autonomous
                                               re-arm: the filter stays dead until
                                               an external ins_init(). Otherwise a
                                               health-check shutdown or a forced
                                               time-jump reset re-arms it into the
                                               collecting state and re-bootstraps
                                               from the next coherent IMU+fix
                                               window. Requires auto_init, a no-op
                                               otherwise. See REQ-NAV-042,
                                               REQ-NAV-016. */

    /* Automatic ZUPT/ZARU: on by default. While the IMU (and the filter's own
     * velocity estimate) look stationary for auto_zupt_dwell_sec, the filter
     * injects its own zero_velocity_update/zero_rotation_update, letting any
     * ground-based system keep re-estimating its IMU biases without an external
     * trigger. Rate-limited by auto_zupt_min_interval_sec. Since a body
     * cruising at constant velocity also shows a gravity-only specific force
     * and near-zero rotation rate, auto_zupt_max_vel_mps additionally requires
     * an externally observed (GNSS) velocity to already be small - see
     * ins_auto_zupt_velocity_gate_ok in ins.c for why the filter's own velocity
     * state must NOT be used there.
     *
     * This parameter set is the single place stillness is defined for a caller
     * running the whole suite: nav_suite_init() propagates it into the ARS/AHRS
     * auto-ZARU config (REQ-SUITE-020), which drives baro_alt's vertical
     * zero-velocity update (REQ-SUITE-015). Only the tuning is shared, each
     * filter keeps its own implementation. */
    bool  auto_zupt_disable;          /**< true -> turn the detector off */
    float auto_zupt_static_gyr_rps;   /**< max |gyro| for the static gate [rad/s]
                                         (0 -> default) */
    float auto_zupt_static_acc_mps2;  /**< max ||f|-g| for the static gate
                                         [m/s^2] (0 -> default) */
    float auto_zupt_max_vel_mps;      /**< max. |estimated velocity| to arm [m/s] (0
                                         -> default) */
    float auto_zupt_dwell_sec;        /**< required stillness before triggering [s] (0
                                         -> default), measured from the variance
                                         window's first stillness verdict (not
                                         from when the magnitude/velocity gates
                                         alone go static), so the gyro/accel
                                         average behind the first trigger always
                                         covers at least this much real data */
    float auto_zupt_min_interval_sec; /**< min. time between auto-triggers [s]
                                         (0 -> default) */

    /* Magnetometer field-strength disturbance gate. Once a position is supplied
     * (ins_set_magnetic_model_from_position) the measured field magnitude is
     * checked against the WMM total field, and a sample deviating by more than
     * mag_field_tolerance is downweighted. Requires the magnetometer in the
     * same physical unit as the model (uT). */
    float mag_field_tolerance;    /**< max |B|-deviation fraction before
                                       downweighting (0 -> 0.30) */
    bool mag_field_check_disable; /**< true -> never gate on field
                                       strength (uncalibrated mag) */

    /* Magnetometer hard-iron bias estimation (REQ-NAV-029): extends the error
     * state to 18 (3 body-frame bias states on the magnetometer, random-walk
     * model, unit uT). The bias is only separable from the yaw error under
     * attitude changes, so expect convergence during/after rotations. Requires
     * INS_UNKNOWNS_MAX >= 18, else ins_init fails. */
    bool estimate_mag_bias; /**< true -> 18-state mode */

    /* Automotive mode (REQ-NAV-034): opt-in yaw aiding from the GNSS velocity
     * vector. When the horizontal ground speed is high enough for the course
     * over ground (atan2(vE, vN)) to be well defined, the filter fuses it as a
     * low-weight yaw measurement, assuming the vehicle travels in the direction
     * it points (non-holonomic). Yaw only, never pitch/roll. */
    bool automotive_mode; /**< true -> derive yaw from GNSS course */

    /* Minimum horizontal ground speed for automotive_mode's course-over-ground
     * yaw fusion (REQ-NAV-034), below which the course is noise-dominated
     * (parking, reversing). 0 -> the built-in default, tuned for a road
     * vehicle. This does not change what the assumption itself requires:
     * course over ground must equal heading, which no minimum speed can fix for
     * an aircraft, whose ground velocity differs from its airspeed vector by
     * the wind drift. */
    float automotive_min_speed_mps; /**< 0 -> 2 m/s default */

    /* Floor on automotive_mode's fused yaw stddev [rad] (REQ-NAV-034). The
     * per-epoch measurement noise is sigma_v/speed, never below this floor.
     * 0 -> the built-in default. Raise it to cap how much the
     * course-over-ground assumption is trusted regardless of speed: an aircraft
     * in coordinated flight has its heading aligned with the *airspeed* vector,
     * not the *ground* velocity vector GNSS measures, so a crosswind adds a
     * crab-angle error a car does not have. */
    float automotive_min_yaw_stddev; /**< 0 -> 5 deg default */

    /* Non-holonomic lateral velocity constraint (REQ-NAV-077): a synthetic
     * measurement stating that a wheeled vehicle does not travel sideways,
     * fused as the body-frame lateral velocity against a truth of zero. Its
     * value is the attitude block of the measurement matrix, whose gain is
     * the ground speed: during a GNSS outage the lateral channel is
     * otherwise unobserved and a roll error leaks gravity sideways.
     *
     * Adding it needs a mounting calibration first.
     *
     * Requires automotive_mode, and shares its automotive_min_speed_mps. */
    bool  automotive_lateral_constraint;   /**< true -> fuse the constraint */
    float automotive_lateral_stddev_mps;   /**< measurement noise [m/s]. 0 ->
                                                default. Set it ABOVE the
                                                residual's scatter, not at it:
                                                the residual is a slowly
                                                varying offset, so repeated
                                                fusions do not carry
                                                independent information. */
    float automotive_lateral_max_yaw_rate; /**< skip above this |yaw rate|
                                                [rad/s], where side slip
                                                breaks the assumption.
                                                0 -> default. */
    float automotive_lateral_after_sec;    /**< hold the constraint off until
                                                this long without a GNSS
                                                fusion. 0 -> default,
                                                negative -> no delay. Next to
                                                a live GNSS velocity the
                                                constraint adds nothing, so
                                                the delay costs no accuracy
                                                and keeps a mounting error
                                                out of the state while aiding
                                                is up. */

    /* Global outlier-rejection override (REQ-SYS-015, diagnostics/analysis
     * only): every chi2-downweighted absolute reference is then fused at its
     * nominal variance regardless of the innovation size. Propagated by
     * nav_suite_init() to the AHRS/baro config templates (REQ-SUITE-011). */
    bool chi2_disable; /**< true -> never chi2-downweight a fusion */

    /* IMU calibration (REQ-NAV-037): applied to the raw accelerometer /
     * gyroscope at the measurement boundary, before the estimated-bias
     * subtraction and before the process-noise gates, so every downstream
     * consumer sees the calibrated signal. Per-sensor model:
     *     corrected = M * (raw - fixed_bias)
     * M is a column-major 3x3 (misalignment + scale factor /
     * non-orthogonality), an all-zero M is treated as identity. The fixed bias
     * is a *permanent* calibration offset removed here and never estimated,
     * distinct from ins_init_t.acc_bias_init_mps2 / gyr_bias_init_rps, which
     * merely seed the estimated bias state. */
    float imu_acc_misalignment[9]; /**< accel 3x3, col-major (all-0 -> I) */
    float imu_gyr_misalignment[9]; /**< gyro  3x3, col-major (all-0 -> I) */
    float imu_acc_fixed_bias[3];   /**< permanent accel bias removed [m/s^2] */
    float imu_gyr_fixed_bias[3];   /**< permanent gyro  bias removed [rad/s] */

    /* GNSS covariance conditioning (REQ-NAV-038): vendor NED covariances are
     * often over-optimistic. Applied at the measurement boundary, before
     * fusion, independently to the position and velocity blocks:
     *     stddev_axis = max(scale * stddev_reported, floor_axis)
     * The scale multiplies the reported *stddev* (variance and off-diagonal
     * terms by scale^2, preserving the correlation structure), the floor then
     * lifts each diagonal entry. Horizontal (N,E) and vertical (D) floors are
     * separate. scale 0 -> 1.0, floor 0 -> no floor for that axis group.
     *
     * These weight the FUSION and nothing else. Every fix-quality gate keeps
     * grading the stddev the receiver reported, so telling the filter to trust
     * a noisy receiver less can never turn into refusing to use it at all. */
    float gnss_pos_cov_scale;            /**< multiplies GNSS pos stddev (0 -> 1) */
    float gnss_vel_cov_scale;            /**< multiplies GNSS vel stddev (0 -> 1) */
    float gnss_pos_stddev_floor_hor_m;   /**< min horiz. pos stddev [m]   (0 -> none) */
    float gnss_pos_stddev_floor_ver_m;   /**< min vert.  pos stddev [m]   (0 -> none) */
    float gnss_vel_stddev_floor_hor_mps; /**< min horiz. vel stddev [m/s] (0 -> none) */
    float gnss_vel_stddev_floor_ver_mps; /**< min vert.  vel stddev [m/s] (0 -> none) */

    /* Magnetometer calibration (REQ-NAV-039): same model as the IMU
     * calibration above (corrected = M * (raw - fixed_bias)), applied to the
     * raw magnetometer sample at the measurement boundary. M is a column-major
     * 3x3 capturing soft-iron / scale / axis misalignment (all-zero ->
     * identity), the fixed bias is the known hard-iron offset removed
     * permanently [uT]. Independent of the optional 18-state estimated
     * hard-iron bias: the fixed term removes the calibrated part, the estimated
     * state tracks the residual. In nav_suite it is forwarded to the
     * magnetometer AHRS (REQ-SUITE-012). */
    float mag_misalignment[9]; /**< soft-iron/scale/misalignment 3x3, col-major (all-0 -> I) */
    float mag_fixed_bias[3];   /**< permanent hard-iron bias removed [uT] */

    /* Dedicated GNSS position vertical (height) downweight (REQ-NAV-041): an
     * extra scale on the Down/height axis of the GNSS *position* covariance
     * only, for receivers whose height solution is far worse than their
     * horizontal one. Applied as the correlation-preserving congruence
     * diag(1,1,s) Q diag(1,1,s) on top of gnss_pos_cov_scale. */
    float gnss_pos_cov_scale_height; /**< multiplies GNSS pos *height* stddev (0 -> 1) */

    /* Global chi2 downweight significance level (REQ-NAV-046): one rejection
     * alpha shared by every absolute-reference gate. Each reference is fused
     * scalar-row-wise (1 DOF), so the innovation gate ins_fuse() applies is
     * chi2_threshold = chi2inv(1 - alpha, 1), resolved once in ins_init. alpha
     * is the tail probability of wrongly downweighting a good sample. 0 -> the
     * built-in default. Orthogonal to chi2_disable (REQ-NAV-035), which takes
     * precedence. */
    float chi2_reject_alpha; /**< global chi2 gate significance (0 -> default) */

    /* Variance criterion of the auto-ZUPT/ZARU detector (REQ-NAV-013): the
     * per-axis RMS sample stddev of the raw IMU over a short window must stay
     * below these for the platform to count as still. This is the PRIMARY
     * stillness statement (auto_zupt_static_gyr_rps / auto_zupt_static_acc_mps2
     * above are only loose magnitude bounds, see sensor_defaults.h). Raising
     * them far above any plausible sensor noise effectively disables the
     * criterion, which is what a platform with an inherently noisy standstill
     * needs. */
    float auto_zupt_static_gyr_stddev_rps;  /**< max RMS gyro stddev [rad/s]
                                                 (0 -> default) */
    float auto_zupt_static_acc_stddev_mps2; /**< max RMS accelerometer stddev
                                                 [m/s^2] (0 -> default) */

    /* GNSS quality hysteresis around the 3D solution (REQ-NAV-051,
     * REQ-NAV-052). Three independent threshold sets act on the same reported
     * GNSS 1-sigmas:
     *
     *   1) gnss_max_* above         - may this fix be FUSED at all
     *                                 (per-measurement, the loosest set)
     *   2) gnss_start_max_* here    - is the fix stream good enough to ENTER
     *                                 the 3D solution (strictest, with
     *                                 gnss_init_dwell_sec)
     *   3) gnss_stop_max_* here     - is it still good enough to STAY in it
     *                                 (with gnss_stop_dwell_sec)
     *
     * Sets 2 and 3 form the hysteresis, so a fix stream hovering around one
     * threshold cannot flap the solution mode. Each 0 -> a built-in default
     * (REQ-NAV-043). ins_init clamps the start set to at most the fusion set
     * and the stop set to at least the start set. */
    float gnss_start_max_horizontal_pos_stddev_m;   /**< [m]   (0 -> default) */
    float gnss_start_max_vertical_pos_stddev_m;     /**< [m]   (0 -> default) */
    float gnss_start_max_horizontal_vel_stddev_mps; /**< [m/s] (0 -> default) */
    float gnss_start_max_vertical_vel_stddev_mps;   /**< [m/s] (0 -> default) */
    float gnss_stop_max_horizontal_pos_stddev_m;    /**< [m]   (0 -> default) */
    float gnss_stop_max_vertical_pos_stddev_m;      /**< [m]   (0 -> default) */
    float gnss_stop_max_horizontal_vel_stddev_mps;  /**< [m/s] (0 -> default) */
    float gnss_stop_max_vertical_vel_stddev_mps;    /**< [m/s] (0 -> default) */
    float gnss_stop_dwell_sec;                      /**< how long the GNSS stream must fail the
                                                         gnss_stop_max_* set before the 3D solution
                                                         is left, counted only over epochs that
                                                         actually offered a fix, so a plain outage
                                                         is still governed by
                                                         max_deadreckoning_sec. 0 -> default. */
    bool gnss_stop_disable;                         /**< true -> never leave the 3D solution on GNSS
                                                         quality alone (the pre-REQ-NAV-052
                                                         behaviour). The entry gate stays in
                                                         force. */
    bool baro_height_disable;                       /**< true -> never select the barometric height
                                                         source at bootstrap (REQ-NAV-053), even if a
                                                         barometer was seen: keep GNSS position's
                                                         vertical row instead. For a GNSS-labelled
                                                         source whose vertical accuracy is actually
                                                         better than a barometer's, e.g. a precise
                                                         indoor tracking system fed through the
                                                         gnss_pos/gnss_vel interface. */

    /* Accuracy requirement for the auto-ZUPT/ZARU velocity gate (REQ-NAV-013):
     * the external (GNSS) velocity observation used by
     * ins_auto_zupt_velocity_gate_ok must itself be precise - a low speed
     * reported with a large uncertainty is not evidence of standstill.
     * */
    float auto_zupt_max_vel_stddev_mps; /**< max GNSS velocity 1-sigma
                                             (per axis) to trust it for the
                                             gate [m/s] (0 -> default) */

    /* Opt out of the ARS/AHRS's OWN stillness detector while leaving ins's
     * intact (REQ-SUITE-020). ins ignores this field: it is a suite-level
     * switch that nav_suite_init() routes into ahrs_config_t.auto_zaru_disable
     * (REQ-AHRS-017). That detector is velocity-blind, so it also arms during a
     * genuine constant-velocity cruise. The ARS/AHRS then still receive the
     * zero-rotation trigger ins derives (REQ-SUITE-009). auto_zupt_disable
     * above is the bigger hammer. */
    bool auto_zupt_velocity_blind_disable; /**< true -> the ARS/AHRS never
                                                arm their own stillness
                                                detector (ins is unaffected
                                                and keeps deciding for them) */

    /* GNSS position decimation (REQ-NAV-063). A receiver reports position and
     * velocity out of one internally coupled solution, but the cross-covariance
     * between the two is not part of any common output message, which makes the
     * combined fuse treat two views of the same information as independent
     * evidence. Fusing only one block per epoch makes no assumption about a
     * correlation that cannot be observed, and thins the measurement whose
     * error is the more strongly correlated IN TIME (position: multipath,
     * residual ionosphere, receiver smoothing) while leaving the comparatively
     * white Doppler velocity at full rate. */
    int gnss_pos_decimation; /**< fuse the GNSS position on every Nth epoch
                                  that offers a usable position AND a usable
                                  velocity, the velocity alone on the other
                                  N-1. <= 1 -> no decimation (fuse both)
                                  0 -> default */

    /* Absolute-speed aiding calibration (REQ-NAV-068). Applies to
     * ins_measurements_t.speed, which carries only the per-sample noise: the
     * two terms here describe the INSTALLATION, which the sensor cannot report.
     *
     * The error of this sensor class splits into a per-sample part that does
     * not grow with speed (an OBD-II PID 0x0D value is quantized to 1 km/h) and
     * a multiplicative part that does (the vehicle's speed signal carries a
     * scale error of a few percent from rolling radius, tyre wear and the
     * scaling the ECU applies, and its sign is not predictable: the
     * type-approval margin that keeps an indicated speed from ever falling
     * below the true one constrains the DASHBOARD, not this reading). Fusing
     * the raw value against the per-sample noise alone would present that
     * systematic bias as independent evidence and pull the velocity states
     * permanently off, so speed_scale removes the calibrated part and
     * speed_stddev_rel prices what is left:
     *
     *     z = speed_scale * speed_mps
     *     R = stddev_mps^2 + (speed_stddev_rel * z)^2
     *
     * No scale-factor STATE is estimated. */
    float speed_scale;      /**< multiplies the reported speed (0 -> 1.0) */
    float speed_stddev_rel; /**< speed-proportional 1-sigma, as a fraction
                                 of the speed (0 -> default) */
    float speed_min_mps;    /**< below this FILTERED speed the measurement is
                                 skipped, since v_hat is undefined at v = 0
                                 and meaningless just above it (0 -> default) */

    /* GNSS accuracy caps (REQ-NAV-071): the upper clamp of the conditioning
     * pipeline described at gnss_pos_cov_scale above, applied last of the
     * receiver-accuracy steps:
     *     stddev_axis = min(max(scale * stddev_reported, floor), cap)
     * Like the floor these clamp the diagonal only.
     *
     * The cap is what makes the loosened fusion gate safe: with gnss_max_*
     * defaulting to these same values (REQ-NAV-043), a degraded fix is
     * downweighted rather than discarded, and the cap keeps the unbounded
     * reported accuracy out of the 32-bit hot path. ins_init raises a cap below
     * the floor of the same axis group to that floor.
     *
     * 0 -> built-in default, negative -> no cap. */
    float gnss_pos_stddev_cap_hor_m;   /**< max horiz. pos stddev [m]   (0 -> default, <0 -> off) */
    float gnss_pos_stddev_cap_ver_m;   /**< max vert.  pos stddev [m]   (0 -> default, <0 -> off) */
    float gnss_vel_stddev_cap_hor_mps; /**< max horiz. vel stddev [m/s] (0 -> default, <0 -> off) */
    float gnss_vel_stddev_cap_ver_mps; /**< max vert.  vel stddev [m/s] (0 -> default, <0 -> off) */

    /* Asymmetric tracking of the reported GNSS accuracy (REQ-NAV-072): the
     * FIRST step of the conditioning pipeline. A rise in the reported 1-sigma
     * takes effect immediately and in full, a fall only with a first-order
     * decay of this time constant:
     *
     *     alpha = clamp(dt / tau, 0, 1)
     *     e     = max(stddev_reported, e * (1 - alpha))
     *
     * kept per block (pos, vel) and axis group (horizontal N/E, vertical D),
     * and applied to the fusion covariance as the correlation-preserving
     * congruence diag(r_hor, r_hor, r_ver) with r = e / stddev_reported >= 1.
     *
     * A receiver's accuracy report is trustworthy when it worsens and
     * optimistic when it recovers: after an obstruction the reported figure
     * returns to nominal well before the fix itself has settled. Weights the
     * fusion only, so one bad epoch cannot hold the filter out of the 3D
     * solution for a whole time constant.
     *
     * 0 -> built-in default, negative -> no envelope. */
    float gnss_acc_envelope_tau_sec; /**< decay time constant [s] (0 -> default, <0 -> off) */

    /* Manoeuvre-dependent GNSS velocity noise (REQ-NAV-073): extra independent
     * variance added to the diagonal of the conditioned GNSS VELOCITY
     * covariance, after the floor and the cap, from the current n-frame
     * acceleration of the ANTENNA with gravity removed:
     *
     *     sigma_extra_hor = scale_hor * |a_NE|   (norm of the North/East pair)
     *     sigma_extra_ver = scale_ver * |a_D|
     *
     * a is the body's acceleration plus the centripetal acceleration the
     * antenna lever arm adds under rotation (REQ-NAV-076), since what the
     * receiver averaged is the motion of the phase centre.
     *
     * A Doppler velocity is the average over the receiver's measurement
     * interval while the filter fuses it against an instantaneous state. The
     * two differ by roughly a*T/2 under acceleration - a systematic error the
     * reported accuracy cannot contain, because it belongs to the platform
     * rather than to the signal.
     *
     * Stated in the stddev domain, [m/s] of extra velocity noise per [m/s^2]
     * of acceleration. Raise both for a receiver whose velocity is
     * time-differenced carrier phase over a long output interval. 0 -> default,
     * negative -> off. */
    float gnss_vel_noise_acc_scale_hor; /**< [m/s per m/s^2] on N,E (0 -> default, <0 -> off) */
    float gnss_vel_noise_acc_scale_ver; /**< [m/s per m/s^2] on D   (0 -> default, <0 -> off) */

    /* Averaging window for the acceleration the term above reads
     * (REQ-NAV-075): the same T the scales are half of. The error priced is
     * what averaging over T does to a velocity, so the acceleration over T is
     * what sets it. Without the window a manoeuvre that just ended goes
     * unpriced on the very fix that still carries its error.
     *
     * Implemented as a first-order average of the n-frame acceleration VECTOR,
     * so a zero-mean vibration cancels instead of accumulating. The rotation
     * half of the antenna acceleration is averaged over the same window as the
     * outer product omega*omega' rather than omega (REQ-NAV-076): being
     * quadratic it does NOT cancel for a platform that turns back and forth.
     *
     * 0 -> built-in default, negative -> no window. */
    float gnss_vel_noise_acc_window_sec; /**< [s] (0 -> default, <0 -> off) */

    /* GNSS fusion rate limit (REQ-NAV-074): the minimum time between two fixes
     * this filter spends. A fix arriving sooner is skipped whole - both blocks,
     * unlike gnss_pos_decimation above, which withholds one of them.
     *
     * A GNSS error decorrelates far more slowly than a modern receiver emits
     * fixes: multipath, residual ionosphere and the receiver's own smoothing
     * persist over many seconds. Fusing at 20 Hz therefore carries the same
     * information twice rather than twice the information, and the filter
     * answers by driving its covariance below what the measurements support.
     *
     * Suspended while the coasting window is expired. A skipped fix still
     * counts as position aiding (REQ-NAV-023), so pacing the fusion never
     * shortens the coasting window.
     *
     * 0 -> built-in default, negative -> no limit. */
    int gnss_min_delay_ms; /**< min. time between two fused GNSS epochs [ms]
                                (0 -> default, < 0 -> no limit) */
} ins_options_t;

/** @brief Nominal state vector (float-only, local n-frame). */
typedef struct
{
    float pos_local[3]; /**< position in n-frame (NED), relative to
                             the origin set at ins_init [m] */
    float vel_ned[3];   /**< velocity in NED [m/s] */
    float qbn[4];       /**< body-to-NED quaternion (Hamilton, q[0]=w) */
    float acc_bias[3];  /**< accelerometer bias [m/s^2] */
    float gyr_bias[3];  /**< gyroscope bias [rad/s] */
    float mag_bias[3];  /**< magnetometer hard-iron bias, body frame
                             [same unit as mag/model, uT]. Only
                             estimated in 18-state mode
                             (estimate_mag_bias), zero otherwise. */
} ins_state_t;

/** @brief History item (for delayed-measurement fusion). */
typedef struct
{
    ins_time_us_t time;       /**< epoch timestamp [us] */
    ins_state_t   state;      /**< nominal state at this epoch */
    double        latlonh[3]; /**< absolute position anchor at this epoch
                                   (lat,lon [rad], h [m]), needed to form
                                   delayed GNSS position residuals. */
    /* Covariance as UDU factors (saved for history). Sized to the
       compile-time maximum; the active leading dimension is the
       instance's runtime state size (ins_t.n). */
    float U[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX]; /**< unit upper triangular factor */
    float d[INS_UNKNOWNS_MAX];                    /**< diagonal factor */
    float omega_b_nb[3];                          /**< body rotation rate at this epoch [rad/s] */
    float R_b_to_n[9]; /**< body-to-NED rotation at this epoch (column-major) */
} ins_history_item_t;

/* @satisfies REQ-NAV-019 */
/** @brief Debug/health bookkeeping.
 *
 * A passive record of the silent reject/skip paths inside the filter. All
 * counters are monotonic since ins_init(), the snapshot fields hold the most
 * recent (or running-max) value. Read via ins_get_diag(). */
typedef struct
{
    uint32_t n_updates; /**< ins_update() calls on a live filter */
    uint32_t n_predict; /**< Kalman prediction steps run */

    /* Timing anomalies (see ins_update time-jump handling). */
    uint32_t n_time_backward;      /**< epochs with dt < 0 vs. last predict */
    uint32_t n_time_dropped;       /**< dropped: older than INS_MAX_DELAY_MS */
    uint32_t n_time_jump_reset;    /**< forward jump beyond max_prediction_time
                                        that forced a filter reset */
    uint32_t n_time_restart_reset; /**< backwards discontinuity held for
                                        INS_TIME_RESTART_EPOCHS that forced a
                                        filter reset (REQ-NAV-070) */
    int32_t dt_ms_min;             /**< most-negative dt seen [ms] (0 if none) */
    int32_t dt_ms_max;             /**< largest forward dt seen [ms] */

    /* GNSS aiding. */
    uint32_t n_gnss_seen;             /**< epochs offering a valid GNSS pos/vel */
    uint32_t n_gnss_used;             /**< epochs where GNSS was actually fused */
    uint32_t n_gnss_rejected_noise;   /**< rejected by the stddev shutdown gate */
    uint32_t n_gnss_quality_exit;     /**< times the 3D solution was left because
                                           the GNSS quality stayed below the
                                           gnss_stop_max_* set for longer than
                                           gnss_stop_dwell_sec (REQ-NAV-052) */
    uint32_t n_gnss_no_anchor;        /**< delayed GNSS with no history match */
    uint32_t n_gnss_large_residual;   /**< fusions whose position residual
                                         exceeded 3 sigma of the *measurement*
                                         noise (a coarse proxy, not the chi2
                                         test) */
    ins_time_us_t t_last_gnss_fusion; /**< timestamp of last GNSS fusion
                                       (0 = never), compare against the current
                                       epoch to detect long GNSS outages */

    /* Residual snapshots [SI units]. */
    float last_gnss_pos_residual_m;   /**< |pos residual| at last GNSS fusion */
    float max_gnss_pos_residual_m;    /**< running max of the above */
    float last_gnss_vel_residual_mps; /**< |vel residual| at last GNSS fusion */

    /* Barometric height (REQ-NAV-053/054), only ever nonzero when
       height_from_baro is true. */
    uint32_t n_baro_height_used; /**< epochs where barometric height was
                                      actually fused */

    /* Absolute speed aiding (REQ-NAV-068). */
    uint32_t n_speed_seen;         /**< epochs offering a usable speed sample */
    uint32_t n_speed_used;         /**< epochs where the speed was actually fused */
    uint32_t n_speed_skipped;      /**< usable samples skipped because the
                                        FILTERED speed was below
                                        opt.speed_min_mps, or the delayed state
                                        was no longer in the history */
    float last_speed_residual_mps; /**< ||v_n|| - z at the last fusion [m/s] */

    /* Fusion failures. */
    uint32_t n_fuse_fail;     /**< a UDU update returned an error
                                   (non-PD R / bad variance) */
    uint32_t n_invalid_input; /**< measurement blocks dropped at the
                                   ins_update() boundary because they
                                   contained non-finite values (NaN/Inf,
                                   e.g. from a flaky sensor bus) */

    /* Outlier downweighting (REQ-SYS-006, REQ-NAV-036). */
    uint32_t n_downweighted; /**< ins_fuse() calls where at least one row's chi2
                                  test tripped and was downweighted rather than
                                  dropped. Diagnostic only, approximate for a
                                  multi-row batch (see ins_fuse_is_outlier in
                                  ins.c), 0 whenever chi2_disable is set */

    /* Automatic ZUPT/ZARU detector (see opt.auto_zupt_*). */
    uint32_t n_auto_zupt; /**< epochs where the detector fired */

    /* Initial bias-prior consistency check (REQ-NAV-050, diagnostic only):
       while the platform stands still, the averaged raw IMU is essentially the
       sensor bias itself and can be held against the configured 1-sigma prior.
       A hit means that prior is far too tight for the actual hardware. */
    uint32_t n_acc_bias_prior_exceeded; /**< windows whose mean ||f|| - g exceeded
                                             the accelerometer bias prior */
    uint32_t n_gyr_bias_prior_exceeded; /**< windows whose mean |omega| exceeded
                                             the gyroscope bias prior */

    /* Dead-reckoning / outage recovery (see opt.max_deadreckoning_sec). */
    uint32_t n_reacquire; /**< re-acquisitions: first usable fix after
                               an expired coasting window re-anchored
                               position/velocity instead of fusing */

    /* Auto-init bootstrap under motion (REQ-NAV-047). */
    uint32_t n_autoinit_moving; /**< auto-init bootstraps whose leveling window
                                     was not quasi-static, so the reported
                                     roll/pitch stddev was widened */

    /* Autonomous re-acquisition after a health-check shutdown (REQ-NAV-042). */
    uint32_t n_health_reset; /**< health-check shutdowns that auto-re-armed into
                                  the collecting state. Monotonic since
                                  ins_init and preserved across the re-arm. */

    /* Overconfidence / covariance-collapse watchdog (REQ-NAV-040): flags when
       the filter's own reported 1-sigma accuracy becomes implausibly good, a
       symptom of covariance collapse. Purely diagnostic. The floors are
       INS_OVERCONF_* (see ins.c), min_* are the smallest per-axis 1-sigma seen
       since ins_init. */
    bool     overconfident;      /**< latched: some epoch's stddev fell below a floor */
    uint32_t n_overconfident;    /**< epochs where a pos/vel/att stddev tripped a floor */
    float    min_pos_stddev_m;   /**< smallest per-axis position stddev seen [m] */
    float    min_vel_stddev_mps; /**< smallest per-axis velocity stddev seen [m/s] */
    float    min_att_stddev_deg; /**< smallest per-axis attitude stddev seen [deg] */

    /** Usable fixes skipped by the fusion rate limit (REQ-NAV-074). A steady
        count is normal for a receiver running above opt.gnss_min_delay_ms, not
        a fault. */
    uint32_t n_gnss_rate_limited;
} ins_diag_t;

/** @brief Main filter instance.
 *
 * Everything is statically sized. No heap allocations. Zero the struct before
 * calling ins_init().
 *
 * The fields are grouped by topic (configuration, covariance, earth model,
 * per-epoch context, auto-init, ...), not by alignment. That leaves 47 bytes of
 * padding in a 51.6 kB struct; packing it out would interleave the groups for a
 * negligible gain, so the padding check is switched off for this struct. */
/* NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding) */
typedef struct
{
    /* Configuration */
    ins_init_t    init; /**< init-time values (see ins_init) */
    ins_options_t opt;  /**< resolved options (defaults filled in) */
    /* Resolved per-channel chi2 downweight gates [1 DOF] (REQ-NAV-046).
     * A configured alpha overrides all with a global chi2inv(1-alpha, 1). */
    float chi2_thr_gnss;  /**< GNSS position/velocity gate */
    float chi2_thr_mag;   /**< magnetometer gate */
    float chi2_thr_local; /**< local-position gate */
    float chi2_thr_yaw;   /**< yaw-residual gate (incl. automotive course) */

    /* Nominal state */
    ins_state_t state; /**< current nominal state */

    /* Active error-state size: INS_UNKNOWNS (15) or INS_UNKNOWNS_MAG
       (18, with estimate_mag_bias). Fixed per instance at ins_init,
       all covariance arrays below use it as their leading dimension. */
    int n; /**< active error-state size (15 or 18) */

    /* Covariance UDU factors: P = U * diag(d) * U' */
    float U[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX]; /**< unit upper triangular factor */
    float d[INS_UNKNOWNS_MAX];                    /**< diagonal factor */

    /* Process noise (diag only, per-second). Scaled by dt in Predict. */
    float Qxx_noise_diag[INS_UNKNOWNS_MAX]; /**< [unit^2/s], per error state */

    /* Absolute-position anchor (double). origin_llh is where
       pos_local == 0, latlonh is the current absolute position,
       book-kept incrementally from the n-frame deltas. Both geodetic, so
       neither costs a conversion to keep up to date (REQ-NAV-080). */
    double origin_llh[3]; /**< n-frame origin, lat,lon [rad], h [m] */
    double latlonh[3];    /**< lat,lon [rad], h [m] */

    /* Cached Earth-curvature terms for the incremental latlonh book-keeping.
       Together with gravity_n these change by only ~d/R_earth per meter of
       travel, so they are refreshed every INS_EARTH_REFRESH_DIST_M. The
       NED->ECEF rotation is deliberately NOT among them: nothing on the epoch
       path needs it, so it is built on request in ins_get_velocity_ecef()
       instead of being refreshed inside the worst-case epoch (REQ-NAV-080). */
    double meta_dlat_per_dN;      /**< 1/(Rm+h) */
    double meta_dlon_per_dE;      /**< 1/((Rn+h)*cos(lat)) */
    float  meta_travel_m;         /**< L1 travel since refresh [m] */
    float  R_b_to_n[9];           /**< body -> NED rotation */
    float  gravity_n[3];          /**< gravity in NED [m/s^2] */
    float  magnetic_n[3];         /**< magnetic model in NED [uT] */
    float  mag_field_expected_uT; /**< WMM total field for the
                                       disturbance gate, 0 -> off */
    bool mag_heading_usable;      /**< false inside a dip pole exclusion zone,
                                       where magnetometer fusion is dropped.
                                       True until a position says otherwise */
    float last_omega_b_nb[3];     /**< last computed omega_b_nb */
    float last_acc_n[3];          /**< last body accel in n-frame */
    float acc_n_avg[3];           /**< last_acc_n averaged over
                                       opt.gnss_vel_noise_acc_window_sec
                                       (REQ-NAV-075) */
    float omega_outer_avg[6];     /**< omega_b_nb * omega_b_nb' averaged over
                                       the same window (REQ-NAV-076), packed
                                       xx, yy, zz, xy, xz, yz. The centripetal
                                       operator of the GNSS antenna lever arm,
                                       which is applied at the fix epoch */
    bool acc_n_avg_valid;         /**< true once acc_n_avg and omega_outer_avg
                                       hold a value */
    float last_acc_meas[3];       /**< last bias-corrected f_b_ib */
    bool  last_acc_valid;         /**< true once last_acc_n/last_acc_meas hold a value */

    /* Handoff from ins_predict_step() to ins_correct_step() for one epoch
       (REQ-NAV-069). ins_predict_step() sanitizes the caller's measurement
       exactly once (that has side effects: diagnostic counters, a throttled
       warn log) and stores it here together with the time-jump/coasting
       decision, which ins_correct_step() cannot re-derive afterwards. */
    struct
    {
        ins_measurements_t m; /**< sanitized measurement for this epoch */

        /** GNSS covariance as the FUSION weights it (REQ-NAV-038, REQ-NAV-041,
            REQ-NAV-071, REQ-NAV-072, REQ-NAV-073): what the receiver reported,
            carried through the full conditioning pipeline. Kept beside the
            measurement rather than written back into it, because the
            fix-quality gates must keep grading the reported value. Only
            meaningful while active is true. */
        float gnss_pos_Qll_fuse[3 * 3];
        float gnss_vel_Qll_fuse[3 * 3];
        float gnss_pos_vel_Qll_fuse[3 * 3];

        bool active;     /**< ins_correct_step() has work to do (history/health) */
        bool run_fusion; /**< also run the ins_fuse_* calls (== no time jump) */
        bool dr_frozen;  /**< restrict fusion to the position re-anchor channels */
    } step_ctx;          /**< per-epoch scratch handed from ins_predict_step() to
                              ins_correct_step() */

    /* Timing / status */
    ins_time_us_t t_last_kalman_predict;  /**< last Kalman prediction step */
    uint32_t      time_dropped_run;       /**< consecutive epochs dropped as
                                                older than INS_MAX_DELAY_MS,
                                                cleared by any epoch that is
                                                not (REQ-NAV-070) */
    ins_time_us_t t_last_pos_aiding;      /**< last absolute position aiding
                                                 (GNSS or local pos fusion /
                                                 re-acquisition) */
    ins_time_us_t t_last_mag_fusion;      /**< last magnetometer fusion */
    ins_time_us_t t_last_zero_rot_fusion; /**< last zero-rotation fusion */
    ins_time_us_t t_last_zero_vel_fusion; /**< last zero-velocity fusion */
    ins_time_us_t t_last_nhc_fusion;      /**< last lateral-constraint fusion
                                               (REQ-NAV-077) */
    ins_time_us_t t_init;                 /**< ins_init() timestamp */
    unsigned      kalman_epochs;          /**< ins_update() calls since init */
    bool          is_initialized;         /**< false before init / after a health-check failure */

    /* Startup / auto-initialization: while is_collecting is true the filter has
       not yet started, it only consumes measurements until the streams are
       coherent (REQ-NAV-033). t_pending_imu / t_pending_fix track the last-seen
       IMU sample and usable position fix so the start gate can check their
       temporal alignment. */
    bool          is_collecting;    /**< true while waiting for the start gate */
    bool          have_pending_imu; /**< an IMU sample was seen this epoch */
    bool          have_pending_fix; /**< a usable position fix was seen this epoch */
    ins_time_us_t t_pending_imu;    /**< timestamp of the last-seen IMU sample */
    ins_time_us_t t_pending_fix;    /**< timestamp of the last-seen usable fix */
    int           autoinit_count;   /**< samples collected in autoinit_buf so far */
    struct
    {
        ins_time_us_t t;                      /**< sample timestamp */
        float         acc[3];                 /**< bias-corrected specific force [m/s^2] */
        float         gyr[3];                 /**< bias-corrected angular rate [rad/s] */
    } autoinit_buf[INS_AUTOINIT_SAMPLES_MAX]; /**< static-window buffer for
                                                      the auto-init leveling
                                                      bootstrap */

    /* Most recent magnetometer sample seen while still uninitialized, cached so
       the auto-init heading bootstrap can use it even when the bootstrap fix
       carries no concurrent mag: GNSS and the magnetometer run on independent
       clocks (REQ-NAV-044). */
    struct
    {
        ins_time_us_t t;       /**< sample timestamp */
        float         data[3]; /**< magnetometer measurement (as fed) */
        bool          valid;   /**< a sample has been cached */
    } autoinit_mag;            /**< last magnetometer sample cached for the auto-init
                                    heading bootstrap (REQ-NAV-044) */

    /* Most recent barometer sample seen while still uninitialized, and whether
       any was seen at all: decides the height source at bootstrap
       (REQ-NAV-053). Same caching rationale as autoinit_mag above; t is checked
       against INS_BARO_ANCHOR_MAX_AGE_SEC, so a barometer that stopped cannot
       latch the height source. */
    struct
    {
        ins_time_us_t t;           /**< sample timestamp */
        float         pressure_pa; /**< static pressure measurement (as fed) */
        bool          valid;       /**< a sample has been cached (>=1 seen so far) */
    } autoinit_baro;               /**< last barometer sample cached for the
                                         height-source bootstrap decision
                                         (REQ-NAV-053) */

    /* Most recent plausible barometer sample of a RUNNING filter, kept for the
       vertical re-anchor at re-acquisition (REQ-NAV-066). Cached on every epoch
       including the inert ones (REQ-NAV-064). Age is checked against
       INS_BARO_ANCHOR_MAX_AGE_SEC when it is used. */
    struct
    {
        ins_time_us_t t;           /**< sample timestamp */
        float         pressure_pa; /**< static pressure measurement (as fed) */
        float         stddev_m;    /**< reported accuracy [m], 0 -> use the default */
        bool          valid;       /**< a sample has been cached */
    } last_baro;                   /**< last barometer sample cached for the vertical
                                        re-anchor at re-acquisition (REQ-NAV-066) */

    /* GNSS-stability dwell before entering 3D (REQ-NAV-045). Tracks the current
       run of continuously-usable position fixes: the bootstrap /
       re-acquisition is held off until the run is long and dense enough
       (>= 1 fix/s). A rejected fix or a gap resets it. */
    ins_time_us_t gnss_dwell_since; /**< t of the first fix in the run (0 = none) */
    ins_time_us_t gnss_dwell_last;  /**< t of the most recent usable fix in the run */
    int           gnss_dwell_count; /**< usable fixes accumulated in the run */

    /** Phase of the GNSS position decimation cycle (REQ-NAV-063): counts up to
        opt.gnss_pos_decimation - 1 and wraps, 0 selects the position. Its own
        counter rather than a diag.n_gnss_* tally: those record what happened,
        this one decides what happens next. */
    uint32_t gnss_pos_decim_count;

    /* Reported-accuracy envelope (REQ-NAV-072). One 1-sigma per block and
       axis group, held at the peak and decayed towards the current report
       with opt.gnss_acc_envelope_tau_sec. The two timestamps are separate
       because position and velocity do not have to arrive in the same
       epochs, and each envelope must decay over ITS OWN elapsed time.
       0 (both value and timestamp) is the un-primed state: the first
       epoch that offers the block adopts the reported value outright. */
    float         gnss_env_pos_hor_m;   /**< [m]   position, N/E group */
    float         gnss_env_pos_ver_m;   /**< [m]   position, D */
    float         gnss_env_vel_hor_mps; /**< [m/s] velocity, N/E group */
    float         gnss_env_vel_ver_mps; /**< [m/s] velocity, D */
    ins_time_us_t t_gnss_env_pos;       /**< t of the last position update (0 = none) */
    ins_time_us_t t_gnss_env_vel;       /**< t of the last velocity update (0 = none) */

    /** Epoch of the last GNSS fix this filter actually SPENT (fused or
        re-anchored on), which is what opt.gnss_min_delay_ms paces
        (REQ-NAV-074). Its own field rather than diag.t_last_gnss_fusion: that
        one records what happened, this one decides what happens next. */
    ins_time_us_t t_last_gnss_fused;

    /* GNSS quality state of the running 3D solution (REQ-NAV-052). A sustained
       quality loss re-arms the filter into the collecting state, so the restart
       runs through the same entry dwell that gates the cold start
       (REQ-NAV-051). gnss_quality_ok only carries the fallback hold-down used
       when the re-arm is opted out of. */
    bool gnss_quality_ok;         /**< false while the 3D solution is held
                                       down after a GNSS quality loss */
    ins_time_us_t gnss_bad_since; /**< t of the first fix in the current run of
                                       fixes failing the stop gate (0 = none) */
    ins_time_us_t gnss_bad_last;  /**< t of the most recent fix in that run, so
                                       the dwell can be accumulated over the
                                       epochs that actually offered aiding
                                       instead of read off the wall clock
                                       (REQ-NAV-052) */
    float gnss_bad_accum_sec;     /**< how much of the stop dwell that run has
                                       filled [s]. A gap in the fix stream
                                       contributes at most
                                       INS_GNSS_STOP_MAX_STEP_SEC, so a plain
                                       outage cannot fill it while a run of
                                       bad fixes is not broken by the gaps
                                       between them */

    /* IMU biases carried across a quality-loss re-arm (REQ-NAV-061). Only that
       re-arm populates this; ins_rearm_collecting clears it, so a health
       shutdown or a time-jump reset can never inherit one. */
    struct
    {
        bool  valid;
        float acc_bias[3];          /**< [m/s^2] as of the exit epoch */
        float gyr_bias[3];          /**< [rad/s] as of the exit epoch */
        float acc_bias_stddev_mps2; /**< already inflated and clamped */
        float gyr_bias_stddev_rps;  /**< already inflated and clamped */
    } bias_carry;                   /**< IMU bias carried across a quality-loss re-arm
                                         (REQ-NAV-061) */

    /* n-frame origin carried across a quality-loss re-arm (REQ-NAV-062). Same
       lifetime and exclusivity as bias_carry above. Keeping the origin is what
       makes pos_local and the local-frame accessors continuous across the
       outage; the absolute solution is anchored to the bootstrap fix. */
    struct
    {
        bool   valid;
        double origin_llh[3]; /**< [rad, rad, m] the origin of the exiting
                                   instance */
        float pos_local[3];   /**< [m] where the exiting instance last thought
                                   it was, in that origin's frame */
        double latlonh[3];    /**< [rad, rad, m] the same position as an
                                   absolute anchor. Paired with pos_local
                                   above it turns the next bootstrap into a
                                   SHORT-baseline geodetic step (fix minus
                                   this position), which stays exact no
                                   matter how far the origin has been left
                                   behind (REQ-NAV-062) */
        ins_time_us_t t;      /**< time of its last position aiding, i.e.
                                   since when the platform has been
                                   unobserved, so the next bootstrap can
                                   price how far it could have got (see
                                   ins_autoinit_origin_carry_usable) */
    } origin_carry;           /**< n-frame origin carried across a quality-loss re-arm
                                   (REQ-NAV-062) */

    /* Height-source selection (REQ-NAV-053), latched once at bootstrap
       from autoinit_baro above and fixed for the lifetime of the filter
       instance (never re-evaluated, see ins_autoinit_try). */
    bool height_from_baro; /**< true: vertical position is fused from
                                 barometric height (REQ-NAV-054),
                                 ins_fuse_gnss drops the vertical row of
                                 the GNSS position measurement
                                 (REQ-NAV-055). false: GNSS position (all
                                 3 axes) as before. */
    float baro_h0_m;       /**< ISA-altitude anchor for the barometric
                                height measurement (REQ-NAV-054), same role as
                                baro_alt's own h0 (REQ-BARO-004). Shifted
                                alongside pos_local[2] whenever the origin
                                relocates (ins_shift_origin_down). Valid only
                                while height_from_baro is true. */

    /* Automatic ZUPT/ZARU detector state (see opt.auto_zupt_*). */
    ins_time_us_t auto_zupt_static_since; /**< 0 if not currently
                                              static, else the time the
                                              current stillness run
                                              started */
    ins_time_us_t auto_zupt_var_since;    /**< 0 if the variance window has
                                              not yet confirmed this stillness
                                              run, else the time it first did
                                              (REQ-NAV-013): the dwell timer
                                              is measured from here, not from
                                              auto_zupt_static_since, so the
                                              gyro/accel accumulators below
                                              always hold a real average by
                                              the time the first trigger can
                                              fire */
    ins_time_us_t t_last_auto_zupt;       /**< time of the last
                                              auto-triggered update
                                              (0 = never) */
    float auto_zupt_gyr_sum[3];           /**< raw gyro sum over the current
                                            stillness run, fused as an average
                                            by the zero-rotation update.
                                            Cleared after each fusion. */
    uint32_t auto_zupt_gyr_count;         /**< samples in the sum */
    float    auto_zupt_ext_vel_mps;       /**< last external (GNSS) velocity
                                            magnitude [m/s] seen for the
                                            staticness gate (REQ-NAV-013),
                                            undefined while
                                            auto_zupt_ext_vel_time == 0 */
    ins_time_us_t auto_zupt_ext_vel_time; /**< timestamp of
                                            auto_zupt_ext_vel_mps
                                            (0 = none seen yet) */

    /* Variance-based stillness criterion of the auto-ZUPT/ZARU detector
       (REQ-NAV-013, see ins_static_variance_update). Welford accumulator over a
       tumbling window: index 0..2 gyro xyz, 3..5 accelerometer xyz, raw. */
    ins_time_us_t static_var_window_since; /**< start of the current window */
    float         static_var_mean[6];      /**< Welford running mean */
    float         static_var_m2[6];        /**< Welford sum of squared deviations */
    uint32_t      static_var_count;        /**< samples in the current window
                                                (0 = window not started) */
    bool static_var_ok;                    /**< verdict latched from the last COMPLETED
                                                window, false until the first one
                                                completes */

    /* Initial bias-prior consistency check (REQ-NAV-050). Own accumulator
       instead of auto_zupt_gyr_sum above: that one is cleared by every
       zero-rotation fusion, far too short a window for this check. */
    ins_time_us_t bias_prior_since; /**< start of the current averaging
                                         window (valid while
                                         bias_prior_count > 0) */
    float    bias_prior_acc_sum[3]; /**< raw accelerometer sum over the window */
    float    bias_prior_gyr_sum[3]; /**< raw gyroscope sum over the window */
    uint32_t bias_prior_count;      /**< samples in the two sums above */

    /* Debug/health bookkeeping (passive, see ins_get_diag). */
    ins_diag_t diag; /**< passive counters/snapshots, see ins_diag_t */

    /** Logging-only bookkeeping (see log.h): pure edge-/rate-detection so the
     *  optional LOG_* calls in ins.c stay informative instead of flooding the
     *  sink. Never read or acted on by the filter itself. */
    struct
    {
        /* Fields grouped by size (8-byte timestamps, then 4-byte, then
           1-byte flags) to avoid needless struct padding - this struct
           carries no filter semantics, so layout is free to optimize. */
        ins_time_us_t t_last_gnss_outage_warn; /**< throttle for the GNSS-outage
                                                     warning (0 = none yet) */
        ins_time_us_t t_last_invalid_warn;     /**< throttle for the invalid-input
                                                     warning (0 = none yet) */
        ins_time_us_t t_last_yaw_aid;          /**< last successful mag, absolute-yaw
                                                     or automotive-course fusion (set
                                                     at finalize, so never "unseen") */
        ins_time_us_t t_last_yaw_stddev_warn;  /**< throttle for the unaided-yaw
                                                     warning (0 = none yet) */
        ins_time_us_t t_last_stuck_warn;       /**< throttle for the stuck-sensor
                                                     warning (0 = none yet) */
        ins_time_us_t t_last_mag_disturb_warn; /**< throttle for the persistent
                                                     magnetic-disturbance warning
                                                     (0 = none yet) */
        /* Runaway detectors: tumbling-window rate checks. Each tracks the
           metric's value at the start of the current window and warns if the
           window-over-window growth exceeds a threshold, then resets the
           window. */
        ins_time_us_t t_yaw_stddev_window;         /**< window start (0 = not yet sampled) */
        ins_time_us_t t_gyr_bias_window;           /**< window start (0 = not yet sampled) */
        ins_time_us_t t_acc_bias_window;           /**< window start (0 = not yet sampled) */
        ins_time_us_t t_last_gyr_bias_sanity_warn; /**< throttle for the gyro-bias
                                                         sanity-bound warning (0 = none yet) */
        ins_time_us_t t_last_acc_bias_sanity_warn; /**< throttle for the accel-bias
                                                         sanity-bound warning (0 = none yet) */
        ins_time_us_t t_last_gyr_bias_prior_warn;  /**< throttle for the gyro bias-prior
                                                         warning (0 = none yet) */
        ins_time_us_t t_last_acc_bias_prior_warn;  /**< throttle for the accel bias-prior
                                                         warning (0 = none yet) */
        ins_time_us_t t_last_entry_gate_warn;      /**< throttle for the "fix is fusable but
                                                         not good enough to enter 3D"
                                                         warning (0 = none yet) */
        ins_time_us_t t_last_baro_height_aid;      /**< t of the last barometric height
                                                         fusion (REQ-NAV-058), 0 = none yet
                                                         (no gap can be reported before the
                                                         first one) */
        ins_time_us_t t_last_baro_height_warn;     /**< throttle for the barometric-height
                                                         aiding-gap warning (0 = none yet) */
        ins_time_us_t t_last_large_pos_res_log;    /**< throttle for the "large GNSS position
                                                         correction pending" message
                                                         (0 = none yet) */
        float last_gyr_raw[3];                     /**< gyro sample of the previous epoch,
                                                        for the stuck-sensor check */
        float    yaw_stddev_window_deg;            /**< yaw stddev at window start [deg] */
        float    gyr_bias_window_dps;              /**< |gyro bias| at window start [deg/s] */
        float    acc_bias_window_mps2;             /**< |accel bias| at window start [m/s^2] */
        uint32_t stuck_gyr_count;                  /**< consecutive epochs with a
                                                        bit-identical gyro sample */
        uint32_t mag_disturbed_count;              /**< consecutive epochs the field-strength
                                                        gate has been outside tolerance */
        bool automotive_logged;                    /**< true once the one-shot
                                                        "automotive course yaw is
                                                        aiding" info has been
                                                        printed, see
                                                        ins_fuse_gnss_course_yaw */
        bool nhc_logged;                           /**< same, for the lateral
                                                        velocity constraint
                                                        (REQ-NAV-077) */
        bool gnss_delay_logged;                    /**< true once the one-shot
                                                        "first GNSS delay seen" info
                                                        has been printed */
        bool last_gyr_raw_valid;                   /**< true once last_gyr_raw holds a value */
    } log_state;

    /* History ring buffer (for delayed measurement fusion) */
    ins_history_item_t history[INS_HISTORY_ITEMS_MAX]; /**< ring buffer, newest
                                                                 at history_index-1 */
    int history_index;                                 /**< next slot to write in history[] */
} ins_t;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Initialise the filter.
     *
     *  The struct must be zeroed before the first call.
     *
     *  @param[in,out] f The filter instance.
     *  @param[in] init Initial state and std.-devs.
     *  @param[in] opt  Filter options.
     *  @return 0 on success, -1 on failure (e.g. a latitude or longitude
     *      outside its range, see REQ-NAV-081). */
    int ins_init(ins_t* f, const ins_init_t* init, const ins_options_t* opt);

    /** @brief Shut down the filter (marks it as uninitialised).
     *  Can be re-initialised with ins_init().
     *  @param[in,out] f The filter instance. */
    void ins_shutdown(ins_t* f);

    /** @brief Feed measurements and advance the filter.
     *
     *  Measurement fields containing non-finite values (NaN/Inf) are
     *  dropped at the boundary (counted in ins_diag_t.n_invalid_input)
     *  instead of reaching the fusion math. A single NaN would otherwise
     *  slip through the chi2/variance gates (every comparison with NaN is
     *  false), poison the state and force the health check to shut the
     *  filter down.
     *
     *  This performs (in order, depending on what is valid):
     *   - Strapdown integration (if acc+gyr valid and strapdown_dt_sec > 0)
     *   - Kalman prediction step (if dt since last >= kalman_update_dt_sec)
     *   - Fusion of magnetometer / zero-velocity / zero-rotation
     *   - Fusion of absolute yaw (lighthouse pose, dual-antenna GNSS),
     *     optionally delayed via the history
     *   - Fusion of local NED position (lighthouse/UWB/mocap), optionally
     *     delayed via the history
     *   - Delayed fusion of GNSS position / velocity (using the history)
     *   - Save state to history ring buffer
     *   - Health check
     *
     *  @param[in,out] f The filter instance.
     *  @param[in] m Measurements for this epoch. */
    void ins_update(ins_t* f, const ins_measurements_t* m);

    /** @brief ins_predict_step() propagated the covariance this call (and,
     *  if requested, filled phi_out). Cleared on a dropped/skipped epoch. */
#define INS_EPOCH_COV_PROPAGATED (1 << 0)
    /** @brief ins_predict_step() is coasting through an expired
     *  dead-reckoning window (REQ-NAV-022): ins_correct_step() will only
     *  fuse the position re-anchor channels. */
#define INS_EPOCH_DR_FROZEN (1 << 1)
    /** @brief The epoch was fully dropped (uninitialized / too far in the
     *  past / a forward time-jump reset / a backward jump with no IMU
     *  sample). The following ins_correct_step() call is a no-op. */
#define INS_EPOCH_DROPPED (1 << 2)

    /** @brief Time-propagation half of ins_update(): strapdown mechanization
     *  plus the (throttled) Kalman covariance prediction.
     *
     *  Must be followed by exactly one ins_correct_step() call before the next
     *  ins_predict_step(): ins_t.step_ctx has room for exactly one pending
     *  measurement, and ins_correct_step() is also where the history ring
     *  buffer gets its entry and the health check runs. Calling
     *  ins_predict_step() twice silently overwrites step_ctx, so the skipped
     *  epoch's aiding is never fused and it never enters the history buffer.
     *  There is no reason to batch predicts either: the covariance prediction
     *  is already throttled to opt.kalman_update_dt_sec regardless of call
     *  rate, and ins_correct_step() is a cheap no-op on an epoch with no valid
     *  aiding measurement.
     *
     *
     *  @param[in,out] f The filter instance.
     *  @param[in] m Measurements for this epoch.
     *  @param[out] phi_out Optional (can be NULL) buffer for the discrete-time
     *      state transition matrix used this call, n x n column-major
     *      (n = ins_get_num_states(f), bounded by INS_UNKNOWNS_MAX). Only
     *      filled when the return value has INS_EPOCH_COV_PROPAGATED set,
     *      left untouched otherwise.
     *  @return Bitwise OR of INS_EPOCH_* flags. */
    int ins_predict_step(ins_t* f, const ins_measurements_t* m, float* phi_out);

    /** @brief Fusion half of ins_update(): dispatches the measurement set
     *  by the matching ins_predict_step() call onto ins_fuse_mag/_yaw/
     *  _zero_velocity/_zero_rotation/_local_pos/_gnss/_baro_height/_speed/
     *  _gnss_course_yaw as applicable, then saves history and runs the
     *  health check. A no-op if the matching ins_predict_step() dropped the
     *  epoch (see INS_EPOCH_DROPPED) or was never called.
     *
     *  @param[in,out] f The filter instance. */
    void ins_correct_step(ins_t* f);

    /** @brief Apply the configured sensor calibration to a measurement block.
     *
     *  Corrects the raw accelerometer, gyroscope and magnetometer samples in
     *  place using the misalignment/scale matrices and fixed biases in @p opt
     *  (imu_acc_/imu_gyr_/mag_ *): corrected = M * (raw - fixed_bias) per
     *  sensor. Only samples flagged valid are touched, an all-zero matrix is
     *  treated as identity. Does NOT touch the GNSS covariance conditioning
     *  (that is a ins-internal fusion-tuning step, see ins_options_t).
     *
     *  ins_update() applies this internally, so a standalone ins caller
     *  never needs it. It is public so nav_suite (and any wrapper running the
     *  same physical sensors through several filters) can feed the parallel
     *  AHRS/baro filters the identical calibrated signal (REQ-SUITE-012).
     *  NOT idempotent: call exactly once per raw sample.
     *
     *  @param[in] opt Options carrying the calibration (typically f->opt).
     *  @param[in,out] m Measurement block, acc/gyr/mag data corrected in place. */
    void ins_apply_calibration(const ins_options_t* opt, ins_measurements_t* m);

    /** @brief Condition a reported GNSS covariance into the one the fusion
     *  weights the fix with (REQ-NAV-038, REQ-NAV-041).
     *
     *  Applies opt's scale, the position-only height scale and the
     *  horizontal/vertical stddev floors to a symmetric 3x3 NED covariance:
     *  stddev_axis = max(scale * stddev_reported, floor_axis). An options
     *  struct left at 0 is a no-op, so @p Qll_out is then a plain copy.
     *
     *  This is a FUSION weighting step. It must not be fed to a fix-quality
     *  gate: those grade what the receiver reported, and since conditioning
     *  can only inflate a covariance, using it there turns a downweight into
     *  a rejection (see REQ-NAV-038).
     *
     *  ins_update() applies this internally, so a standalone ins caller never
     *  needs it. It is public so nav_suite can weight its baro/GNSS vertical
     *  offset filter with the same number ins fuses with.
     *
     *  @param[in] opt Options carrying the conditioning (typically f->opt).
     *  @param[in] Qll_in Reported covariance, column-major 3x3, NED.
     *  @param[out] Qll_out Conditioned covariance. May alias @p Qll_in. */
    void ins_gnss_condition_pos_cov(const ins_options_t* opt, const float Qll_in[3 * 3],
                                    float Qll_out[3 * 3]);

    /** @brief Velocity-block twin of ins_gnss_condition_pos_cov().
     *
     *  Same contract, with opt's velocity scale and velocity floors. The
     *  height scale is position-only and is never applied here.
     *  @param[in] opt Options carrying the conditioning (typically f->opt).
     *  @param[in] Qll_in Reported covariance, column-major 3x3, NED.
     *  @param[out] Qll_out Conditioned covariance. May alias @p Qll_in. */
    void ins_gnss_condition_vel_cov(const ins_options_t* opt, const float Qll_in[3 * 3],
                                    float Qll_out[3 * 3]);

    /** @brief Is the filter ready to publish a solution?
     *
     *  False during warm-up, after coasting without absolute position
     *  aiding for longer than opt.max_deadreckoning_sec (unless
     *  allow_unlimited_deadreckoning is set), and once the GNSS quality
     *  stayed below the gnss_stop_max_* set for longer than
     *  opt.gnss_stop_dwell_sec (REQ-NAV-052) -- which re-arms the filter,
     *  so the return runs through the entry gate/dwell (REQ-NAV-051) plus
     *  the usual bootstrap warm-up, never a single recovered fix.
     *
     *  @param[in] f The filter instance.
     *  @return true if a solution is available. */
    bool ins_is_ready(const ins_t* f);

    /** @brief Milliseconds of IMU-only coasting since the last absolute
     *  position aiding (GNSS / local pos), judged at the filter's most
     *  recent epoch. Use together with opt.max_deadreckoning_sec to
     *  distinguish "aided" from "coasting" while ins_is_ready() is
     *  still true.
     *
     *  @param[in] f The filter instance.
     *  @return Coasting time [ms], or -1 if the filter is not initialized. */
    int ins_deadreckoning_ms(const ins_t* f);

    /** @brief Access the passive debug/health bookkeeping.
     *
     *  Valid for the lifetime of the filter instance, the counters reset
     *  on ins_init().
     *
     *  @param[in] f The filter instance.
     *  @return Pointer to the internal, read-only diagnostics record (see
     *  ins_diag_t), or NULL if @p f is NULL. */
    const ins_diag_t* ins_get_diag(const ins_t* f);

    /** @brief Report whether the automatic ZUPT/ZARU detector
     *  (opt.auto_zupt_*, see ins_auto_zupt_detect) currently considers
     *  the filter stationary (dwelled long enough to be feeding the
     *  zero-velocity/zero-rotation updates). Always false if the
     *  detector is disabled (opt.auto_zupt_disable) or the filter is not
     *  initialized. Diagnostic/telemetry use, nav_suite also reads this
     *  to propagate a stillness trigger to the ARS/AHRS (REQ-SUITE-009),
     *  since those filters have no velocity state to detect it with.
     *
     *  @param[in] f The filter instance.
     *  @return true if the detector is currently armed. */
    bool ins_auto_zupt_active(const ins_t* f);

    /** @brief Enable/disable the automatic ZUPT/ZARU detector at runtime
     *  (REQ-NAV-013), independent of the opt.auto_zupt_disable set at
     *  ins_init.
     *
     *  A safety-relevant switch: a platform that is legitimately still by
     *  every gate but must never receive a zero-velocity/zero-rotation update
     *  (e.g. a multicopter briefly holding position mid-flight) can disable the
     *  detector for exactly that window. Disabling clears the in-progress dwell
     *  timer and the latched variance-window verdict, so re-enabling always
     *  requires a fresh stillness run. Does not affect an externally-triggered
     *  update (ins_measurements_t.zero_velocity_update / zero_rotation_update).
     *  @param[in,out] f The filter instance.
     *  @param[in] disable true -> the detector can never arm until called
     *      again with false. */
    void ins_set_auto_zupt_disable(ins_t* f, bool disable);

    /* ------------------------------------------------------------------------
     * Accessors
     * ------------------------------------------------------------------------
     */

    /** @brief Get current position in ECEF.
     *  @param[in] f The filter instance.
     *  @param[out] pos_ecef Position in ECEF [m].
     *  @return true if the filter is initialized. */
    bool ins_get_position_ecef(const ins_t* f, double pos_ecef[3]);

    /** @brief Get current position as geodetic coordinates.
     *
     *  The absolute anchor the filter carries, handed out as it is held. Use
     *  this rather than converting ins_get_position_ecef() back: that call
     *  builds the ECEF vector FROM this one, so the round trip costs two
     *  conversions for a value that needs none, and on a target without a
     *  double-precision FPU those two are expensive (REQ-NAV-078).
     *
     *  @param[in] f The filter instance.
     *  @param[out] llh Latitude [rad], longitude [rad], height above the
     *                  WGS84 ellipsoid [m].
     *  @return true if the filter is initialized. */
    bool ins_get_latlonh(const ins_t* f, double llh[3]);

    /** @brief Get current position in the local NED frame (relative to the
     *  origin set at ins_init()).
     *  @param[in] f The filter instance.
     *  @param[out] pos_ned Position in NED [m].
     *  @return true if the filter is initialized. */
    bool ins_get_position_local(const ins_t* f, float pos_ned[3]);

    /** @brief Get current velocity in the NED frame.
     *  @param[in] f The filter instance.
     *  @param[out] vel_ned Velocity in NED [m/s].
     *  @return true if the filter is initialized. */
    bool ins_get_velocity_ned(const ins_t* f, float vel_ned[3]);

    /** @brief Get current velocity in the ECEF frame.
     *  @param[in] f The filter instance.
     *  @param[out] vel_ecef Velocity in ECEF [m/s].
     *  @return true if the filter is initialized. */
    bool ins_get_velocity_ecef(const ins_t* f, float vel_ecef[3]);

    /** @brief Get current attitude quaternion.
     *  @param[in] f The filter instance.
     *  @param[out] q Body-to-NED quaternion (Hamilton, q[0] = w).
     *  @return true if the filter is initialized. */
    bool ins_get_quaternion(const ins_t* f, float q[4]);

    /** @brief Get current attitude as roll/pitch/yaw.
     *  @param[in] f The filter instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad].
     *  @return true if the filter is initialized. */
    bool ins_get_rpy(const ins_t* f, float* roll_rad, float* pitch_rad, float* yaw_rad);

    /** @brief Get the current attitude 1-sigma uncertainty.
     *
     *  The square root of the attitude error state's covariance diagonal
     *  (INS_IDX_RPY, the same numbers the filter itself reasons about,
     *  e.g. when deciding a heading prior is unknown). Small-angle
     *  errors: only meaningful while they are small.
     *
     *  @param[in] f The filter instance.
     *  @param[out] roll_stddev_rad Roll 1-sigma [rad].
     *  @param[out] pitch_stddev_rad Pitch 1-sigma [rad].
     *  @param[out] yaw_stddev_rad Yaw 1-sigma [rad].
     *  @return true if the filter is initialized. */
    bool ins_get_rpy_stddev(const ins_t* f, float* roll_stddev_rad, float* pitch_stddev_rad,
                            float* yaw_stddev_rad);

    /** @brief Get current body-to-NED rotation matrix.
     *  @param[in] f The filter instance.
     *  @param[out] R_b_to_n 3x3 rotation matrix (column-major).
     *  @return true if the filter is initialized. */
    bool ins_get_rotmat_b_to_n(const ins_t* f, float R_b_to_n[9]);

    /** @brief Get current accelerometer bias estimate.
     *  @param[in] f The filter instance.
     *  @param[out] acc_bias_mps2 Accelerometer bias, body frame [m/s^2].
     *  @return true if the filter is initialized. */
    bool ins_get_bias_acc(const ins_t* f, float acc_bias_mps2[3]);

    /** @brief Get current gyroscope bias estimate.
     *  @param[in] f The filter instance.
     *  @param[out] gyr_bias_rps Gyroscope bias, body frame [rad/s].
     *  @return true if the filter is initialized. */
    bool ins_get_bias_gyr(const ins_t* f, float gyr_bias_rps[3]);

    /** @brief Get current magnetometer hard-iron bias estimate.
     *  @param[in] f The filter instance.
     *  @param[out] mag_bias_uT Magnetometer hard-iron bias, body frame [uT].
     *  @return false unless the filter runs in 18-state mode
     *  (estimate_mag_bias) and is initialized. */
    bool ins_get_bias_mag(const ins_t* f, float mag_bias_uT[3]);

    /** @brief Get current body rotation rate.
     *  @param[in] f The filter instance.
     *  @param[out] omega_rps Rotation rate omega_b_nb, body frame [rad/s].
     *  @return true if the filter is initialized. */
    bool ins_get_omega_b_nb(const ins_t* f, float omega_rps[3]);

    /** @brief Get current body acceleration in the n-frame (gravity removed).
     *  @param[in] f The filter instance.
     *  @param[out] acc_n_mps2 Acceleration in NED, gravity removed [m/s^2].
     *  @return true if the filter is initialized. */
    bool ins_get_acc_n(const ins_t* f, float acc_n_mps2[3]);

    /** Update the gravity/magnetic model (e.g. from WMM).
     *  @param[in,out] f The filter instance.
     *  @param[in] gravity_n Gravity vector in NED [m/s^2].
     *  @param[in] magnetic_n Magnetic model vector in NED. */
    void ins_set_world_model(ins_t* f, const float gravity_n[3], const float magnetic_n[3]);

    /** @brief Set the magnetic reference field from a position, via the
     *  World Magnetic Model (declination + inclination + total field).
     *
     *  Builds the full NED reference vector (magnetic_n) from the WMM and
     *  arms the magnetometer field-strength disturbance gate with the
     *  expected total field. Because the reference carries the declination,
     *  the estimated yaw is relative to true north.
     *
     *  Optional and may be called at any time, e.g. later in the run once
     *  a first GNSS fix (or any coarse position) becomes available. Until
     *  then the filter uses the magnetic_n supplied at init (if any). A
     *  non-finite argument is ignored.
     *
     *  @param[in,out] f The filter instance.
     *  @param[in] lat_rad Latitude [rad].
     *  @param[in] lon_rad Longitude [rad].
     *  @param[in] year Decimal year (e.g. 2027.5) for the WMM epoch. */
    void ins_set_magnetic_model_from_position(ins_t* f, double lat_rad, double lon_rad, float year);

    /** @brief Move the n-frame origin down by dz_m [m] (vertical datum
     *  relocation).
     *
     *  A pure datum shift: the absolute solution (lat/lon/height, ECEF)
     *  is unchanged, only pos_local (nominal state and history) shifts so
     *  that all local heights (-pos_local[2]) increase by dz_m. The
     *  covariance is untouched (the shift is deterministic).
     *
     *  Intended to align the local vertical datum with an external height
     *  reference directly after (auto-)initialization, see nav_suite,
     *  which aligns the origin with the barometric filter's datum. For
     *  local NED position aiding (ins_meas_local_pos_t), the external
     *  system's frame must be aligned to the shifted frame.
     *
     *  No-op if the filter is not initialized or dz_m is non-finite.
     *
     *  @param[in,out] f The filter instance.
     *  @param[in] dz_m Downward shift of the origin [m]. */
    void ins_shift_origin_down(ins_t* f, float dz_m);

#ifdef __cplusplus
}
#endif

#endif /* INS_H */
/** @} */
