/** @file ins_capi.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Thin, stable C ABI over ins / nav_suite for language bindings (the
 * Python ctypes wrapper in the INSLIB package). It hides the large
 * internal structs behind opaque handles and a small, shim-owned config
 * struct, so the binding does not have to mirror -- and stay in sync
 * with -- the filter's internal layout.
 *
 * Two parallel handle families, sharing the same config + input setters:
 *   ins_core_*  wraps the bare ins ESKF (lean; position/velocity/attitude).
 *   ins_suite_*   wraps nav_suite (ins + two AHRS filters, mode arbitration
 *          and attitude fallback -- the "best available solution").
 *
 * Per-epoch pattern: <prefix>_set_imu() BEGINS an epoch (it clears the
 * pending measurement); optional aiding setters add to it; _update()
 * consumes it. Frames/units are ins's (FRD body, NED nav, Hamilton
 * quaternion q=[w,x,y,z], int64 microsecond time, radians).
 *
 * NOTE: this shim allocates handles with malloc -- a host-side
 * convenience for bindings; the INSLIB/nav_suite cores stay heap-free.
 * This file lives under python/ (not src/) on purpose: it is outside the
 * project's requirements/aerospace process, which covers only src/.
 */
#ifndef INS_CAPI_H
#define INS_CAPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Compile-time bound for the covariance getters below; mirrors
 *  INS_UNKNOWNS_MAG in ins.h (checked with a static assert in
 *  ins_capi.c) so this header doesn't need to include ins.h. */
#define INS_CAPI_MAX_STATE 18

    /** Flat, shim-owned configuration. Stable contract for the binding;
     *  translated into ins_init_t / ins_options_t by <prefix>_init(). */
    typedef struct
    {
        int64_t time_us;               /**< init timestamp [us] */
        double  lat_rad, lon_rad, h_m; /**< initial position (auto_init overrides) */

        float pos_init_stddev_m;
        float vel_init_stddev_mps;
        float rpy_init_stddev_rad[3]; /**< roll/pitch/yaw, independently
                                           settable (same convention as
                                           ins_init_t.rpy_init_stddev_rad /
                                           ahrs_config_t.rpy_init_stddev_rad) */
        float acc_bias_init_stddev_mps2;
        float gyr_bias_init_stddev_rps;

        float pos_pred_stddev_m_sqrts;
        float vel_pred_stddev_mps_sqrts;
        float rpy_pred_stddev_rad_sqrts;
        float acc_bias_pred_stddev_mps2_sqrts;
        float gyr_bias_pred_stddev_rps_sqrts;

        float zero_vel_stddev_mps;
        float zero_rot_stddev_rps;

        float gyr_bias_init_rps[3]; /**< initial gyro bias [rad/s] */
        float magnetic_n[3];        /**< magnetic model in NED (mag fusion ref) */

        float   kalman_update_dt_sec;
        float   max_prediction_time_sec;
        float   gnss_max_horizontal_pos_stddev_m;
        float   gnss_max_vertical_pos_stddev_m;
        float   gnss_max_horizontal_vel_stddev_mps;
        float   gnss_max_vertical_vel_stddev_mps;
        int32_t magnetometer_min_delay_ms;
        int32_t auto_init;                     /**< 0/1 */
        int32_t allow_unlimited_deadreckoning; /**< 0/1: never expire the
                                                  coasting window */
        float rpy_init_rad[3];                 /**< initial roll/pitch/yaw b->n [rad]
                                                    (used with auto_init=0) */

        float max_deadreckoning_sec;           /**< max IMU-only coasting [s]
                                                    before is_ready degrades
                                                    (0 -> default 10 s; ignored
                                                    with allow_unlimited_...) */
        int32_t auto_zupt_disable;             /**< 0/1: disable the automatic
                                                    ZUPT/ZARU stillness detector */
        float mag_field_tolerance;             /**< WMM field-strength gate: max
                                                    |B| deviation fraction before
                                                    downweighting (0 -> 0.30) */
        int32_t mag_field_check_disable;       /**< 0/1: never gate on field
                                                    strength (uncalibrated mag) */
        int32_t estimate_mag_bias;             /**< 0/1: 18-state mode with
                                                    magnetometer hard-iron bias
                                                    states */
        float   mag_bias_init_stddev_ut;       /**< [uT] (0 -> default) */
        float   mag_bias_pred_stddev_ut_sqrts; /**< [uT/sqrt(s)] (0 -> default) */
        int32_t automotive_mode;               /**< 0/1: derive yaw from the GNSS
                                                    velocity vector (course over
                                                    ground) above a minimum ground
                                                    speed */
        float automotive_min_speed_mps;        /**< automotive_mode's minimum
                                                    ground speed [m/s]
                                                    (0 -> 2 m/s default) */
        float automotive_min_yaw_stddev;       /**< floor on automotive_mode's
                                                    fused yaw stddev [rad]
                                                    (0 -> 5 deg default) */
        int32_t chi2_disable;                  /**< 0/1: disable chi2 outlier downweighting
                                                    library-wide (ins, both AHRS instances,
                                                    baro_alt, the baro-GNSS offset filter) --
                                                    diagnostics/analysis only (REQ-SYS-015) */
        /* IMU calibration (REQ-NAV-037): col-major 3x3 misalignment (all-0
           -> identity) + permanent fixed bias, applied to raw acc/gyr. */
        float imu_acc_misalignment[9];
        float imu_gyr_misalignment[9];
        float imu_acc_fixed_bias[3]; /**< [m/s^2] */
        float imu_gyr_fixed_bias[3]; /**< [rad/s] */
        /* GNSS covariance conditioning (REQ-NAV-038, REQ-NAV-041): scale +
           dedicated height downweight + per-axis stddev floor (all 0 -> no-op). */
        float gnss_pos_cov_scale;            /**< multiplies GNSS pos stddev (0 -> 1) */
        float gnss_pos_cov_scale_height;     /**< multiplies GNSS pos *height* stddev (0 -> 1) */
        float gnss_vel_cov_scale;            /**< multiplies GNSS vel stddev (0 -> 1) */
        float gnss_pos_stddev_floor_hor_m;   /**< [m]   (0 -> none) */
        float gnss_pos_stddev_floor_ver_m;   /**< [m]   (0 -> none) */
        float gnss_vel_stddev_floor_hor_mps; /**< [m/s] (0 -> none) */
        float gnss_vel_stddev_floor_ver_mps; /**< [m/s] (0 -> none) */
        /* Magnetometer calibration (REQ-NAV-039): col-major 3x3 soft-iron/
           scale/misalignment (all-0 -> identity) + fixed hard-iron bias. */
        float mag_misalignment[9];
        float mag_fixed_bias[3]; /**< [uT] */
        /* Initial NED velocity value [m/s] for a non-stationary manual start
           (auto_init == false); converted to ECEF internally. Zero -> start
           at rest (the previous behaviour). */
        float init_vel_ned[3];
        /* Global chi2 downweight significance (REQ-NAV-046): 0 -> per-channel
           historical gates, positive -> shared chi2inv(1-alpha, 1). Appended
           for ctypes offset stability. */
        float chi2_reject_alpha;
        /* IMU leveling window for ins's auto-init bootstrap [s] (0 -> ins's
           built-in default; REQ-VER-017's tools/replay.c equivalent).
           Raise for a low IMU rate so the window still contains enough
           samples for ins_autoinit_try()'s median -- otherwise the bootstrap
           silently never fires and auto_init waits forever for a fix that
           never triggers it. Appended for ctypes offset stability. */
        float auto_init_window_sec;
        /* Auto-ZUPT/ZARU magnitude bounds (ins.h's auto_zupt_static_gyr_rps
           / auto_zupt_max_vel_mps, REQ-NAV-013/REQ-NAV-014): how still the
           IMU and a recent GNSS velocity observation must look before the
           filter injects its own zero-velocity/zero-rotation update. Both
           0 -> ins's built-in defaults. The rest of the stillness set is
           further down (appended later). Appended for ctypes offset
           stability. */
        float auto_zupt_static_gyr_rps; /**< max |gyro| for the static gate
                                            [rad/s] (0 -> default) */
        float auto_zupt_max_vel_mps;    /**< max |GNSS velocity| to arm
                                            [m/s] (0 -> default) */
        /* GNSS quality hysteresis around the 3D solution (REQ-NAV-051,
           REQ-NAV-052): the strict set that admits the solution and the
           loose set that gives it up, both separate from the per-fix
           fusion gates above. Each 0 -> ins's built-in default. Appended
           for ctypes offset stability. */
        float   gnss_start_max_horizontal_pos_stddev_m;   /**< [m] */
        float   gnss_start_max_vertical_pos_stddev_m;     /**< [m] */
        float   gnss_start_max_horizontal_vel_stddev_mps; /**< [m/s] */
        float   gnss_start_max_vertical_vel_stddev_mps;   /**< [m/s] */
        float   gnss_stop_max_horizontal_pos_stddev_m;    /**< [m] */
        float   gnss_stop_max_vertical_pos_stddev_m;      /**< [m] */
        float   gnss_stop_max_horizontal_vel_stddev_mps;  /**< [m/s] */
        float   gnss_stop_max_vertical_vel_stddev_mps;    /**< [m/s] */
        float   gnss_init_dwell_sec;                      /**< entry dwell [s] (0 -> default) */
        int32_t gnss_init_dwell_disable;                  /**< 0/1: enter on the first good fix */
        float   gnss_stop_dwell_sec;                      /**< exit dwell [s] (0 -> default) */
        int32_t gnss_stop_disable;   /**< 0/1: never leave 3D on GNSS quality */
        int32_t baro_height_disable; /**< 0/1: never select the barometric
                                         height source at bootstrap
                                         (REQ-NAV-053) even with a barometer
                                         present. Appended for ctypes offset
                                         stability. */
        /* Rest of the stillness definition (ins.h's auto_zupt_*,
           REQ-NAV-013). Together with the two magnitude bounds above this
           is the ONE set that defines standstill for the whole suite:
           ins_suite_create() hands it to nav_suite_init(), which
           propagates it to both AHRS instances and, through them, to
           baro_alt's vertical zero-velocity update (REQ-SUITE-020). Each
           0 -> the library's built-in default. Appended for ctypes offset
           stability. */
        float auto_zupt_static_acc_mps2;          /**< max ||f|-g| bound [m/s^2] */
        float auto_zupt_max_vel_stddev_mps;       /**< max GNSS velocity 1-sigma
                                                      (per axis) to trust it for
                                                      the velocity gate [m/s] */
        float auto_zupt_static_gyr_stddev_rps;    /**< max RMS gyro stddev over
                                                      the window [rad/s] */
        float auto_zupt_static_acc_stddev_mps2;   /**< max RMS accel stddev over
                                                      the window [m/s^2] */
        float auto_zupt_dwell_sec;                /**< stillness required before
                                                      a trigger [s] */
        float auto_zupt_min_interval_sec;         /**< min time between
                                                      auto-triggers [s] */
        int32_t auto_zupt_velocity_blind_disable; /**< 0/1: opt only the
                                                      ARS/AHRS's own
                                                      velocity-blind detector
                                                      out (REQ-AHRS-017), ins
                                                      keeps deciding */
        int32_t gnss_pos_decimation;              /**< fuse the GNSS position on every Nth
                                                      epoch that offers a usable position
                                                      AND a usable velocity, the velocity
                                                      alone on the other N-1 (REQ-NAV-063).
                                                      <= 1 -> off (fuse both), 0 -> ins's
                                                      own default. Appended for ctypes
                                                      offset stability. */
        /* Absolute speed aiding calibration (REQ-NAV-068). The per-sample
           noise travels with each sample (ins_core_set_speed); these three
           describe the INSTALLATION and are constant across samples.
           Appended for ctypes offset stability. */
        float speed_scale;      /**< multiplies the reported speed (0 -> 1) */
        float speed_stddev_rel; /**< speed-proportional 1-sigma, as a
                                     fraction of the speed (0 -> default) */
        float speed_min_mps;    /**< below this FILTERED speed the sample is
                                     skipped (0 -> default) */
        /* GNSS accuracy caps (REQ-NAV-071), the upper clamp of the
           conditioning pipeline, and the asymmetric accuracy envelope
           (REQ-NAV-072) that feeds it. For all five: 0 -> ins's own default,
           negative -> that step is not applied. Appended for ctypes offset
           stability. */
        float gnss_pos_stddev_cap_hor_m;   /**< [m]   (0 -> default, <0 -> off) */
        float gnss_pos_stddev_cap_ver_m;   /**< [m]   (0 -> default, <0 -> off) */
        float gnss_vel_stddev_cap_hor_mps; /**< [m/s] (0 -> default, <0 -> off) */
        float gnss_vel_stddev_cap_ver_mps; /**< [m/s] (0 -> default, <0 -> off) */
        float gnss_acc_envelope_tau_sec;   /**< [s]   (0 -> default, <0 -> off) */
        /* Manoeuvre-dependent GNSS velocity noise (REQ-NAV-073), in [m/s] of
           extra velocity 1-sigma per [m/s^2] of n-frame ANTENNA acceleration
           (the body's plus the centripetal term of the lever arm,
           REQ-NAV-076). No default: 0 -> off. */
        float gnss_vel_noise_acc_scale_hor; /**< on N,E */
        float gnss_vel_noise_acc_scale_ver; /**< on D */
        /* Averaging window for the acceleration that term reads
           (REQ-NAV-075) [s]. 0 -> ins's default, <0 -> instantaneous. */
        float gnss_vel_noise_acc_window_sec;
        /* GNSS fusion rate limit (REQ-NAV-074): min time between two fused
           GNSS epochs [ms]. 0 -> ins's default (10 Hz), <0 -> no limit. */
        int32_t gnss_min_delay_ms;
        /* Non-holonomic lateral velocity constraint (REQ-NAV-077).
           Appended at the end for ctypes offset stability. */
        int32_t automotive_lateral_constraint;   /**< 0/1 */
        float   automotive_lateral_stddev_mps;   /**< [m/s], 0 -> default */
        float   automotive_lateral_max_yaw_rate; /**< [rad/s], 0 -> default */
        float   automotive_lateral_after_sec;    /**< [s], 0 -> default,
                                                      negative -> no delay */
    } ins_cfg_t;

    /* Solution mode returned by ins_suite_get_mode (mirrors nav_suite_mode_t). */
    enum
    {
        INS_MODE_NONE = 0,
        INS_MODE_ATTITUDE_ONLY,
        INS_MODE_COASTING,
        INS_MODE_FULL
    };

    /* ======================= bare ins (ins_core_*) =============================
     */

    void* ins_core_create(void);
    void  ins_core_destroy(void* h);
    int   ins_core_init(void* h, const ins_cfg_t* cfg);

    void ins_core_set_imu(void* h, int64_t t_us, float dt_sec, const float acc[3],
                          const float gyr[3], const float acc_var[3], const float gyr_var[3]);
    /* The fix as latitude [rad], longitude [rad], height above the WGS84
     * ellipsoid [m], the form the fusion works in (REQ-NAV-079). An
     * ECEF-native source converts with ins_ecef_to_latlonh() first. */
    void ins_core_set_gnss_pos_llh(void* h, const double llh[3], const float var_ned[3]);
    void ins_core_set_gnss_vel_ned(void* h, const float vel_ned[3], const float var_ned[3]);
    /* Full-covariance variants (column-major 3x3 NED blocks) and the
     * optional pos/vel cross block (element (i,j) = cov(pos_i, vel_j),
     * only consumed when both pos and vel are valid) -- together the full
     * 6x6 covariance of [pos; vel]. */
    void ins_core_set_gnss_pos_llh_cov(void* h, const double llh[3], const float Qll_ned[9]);
    void ins_core_set_gnss_vel_ned_cov(void* h, const float vel_ned[3], const float Qll_ned[9]);
    void ins_core_set_gnss_pos_vel_cov(void* h, const float Q_pos_vel_ned[9]);
    void ins_core_set_gnss_leverarm_b(void* h, const float lever_b[3]);
    void ins_core_set_mag(void* h, const float mag[3], const float var[3]);
    void ins_core_set_yaw(void* h, float yaw_rad, float stddev_rad);
    void ins_core_set_local_pos(void* h, const float pos_ned[3], const float var_ned[3],
                                const float lever_b[3]);
    void ins_core_set_zupt(void* h, int on);
    void ins_core_set_zaru(void* h, int on); /**< zero-rotation update */
    /** Barometer (static pressure [Pa], stddev of derived altitude [m],
     *  0 -> consumer default). The bare ins has no barometric state and
     *  ignores it -- only meaningful through the ins_suite_* family; mirrored here
     *  so both families accept the same input stream. */
    void ins_core_set_baro(void* h, float pressure_pa, float stddev_m);
    /** Absolute speed aiding (REQ-NAV-068): scalar ground speed [m/s],
     *  per-sample 1-sigma [m/s] (0 -> default) and how old the sample is
     *  [ms] (history-anchored, like GNSS). The systematic scale error is
     *  NOT stated here, it belongs in the config's speed_scale /
     *  speed_stddev_rel. */
    void ins_core_set_speed(void* h, float speed_mps, float stddev_mps, int delay_ms);
    /** Measurement ages [ms] for delayed (history-anchored) fusion. Apply
     *  to the pending epoch; call AFTER <prefix>_set_imu(). */
    void ins_core_set_gnss_delay_ms(void* h, int ms);
    void ins_core_set_yaw_delay_ms(void* h, int ms);
    void ins_core_set_local_pos_delay_ms(void* h, int ms);
    void ins_core_update(void* h);

    /** Time-propagation half of ins_core_update() (see ins_predict_step()):
     *  strapdown + throttled covariance prediction on the pending epoch's
     *  IMU sample. Must be followed by exactly one ins_core_correct() call
     *  before the next ins_core_predict()/ins_core_update(). Lets a caller
     *  (e.g. an offline RTS smoother) sample ins_core_get_covariance()
     *  between prediction and correction to get P(k|k-1) as well as
     *  P(k|k).
     *  @param[out] phi_out State transition matrix used this call, same
     *      fixed-size/layout convention as ins_core_get_covariance's
     *      P_out (zeroed if the covariance was not propagated this call,
     *      e.g. throttled or the epoch was dropped).
     *  @return Bitwise OR of ins.h's INS_EPOCH_* flags. */
    int ins_core_predict(void* h, float phi_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE]);
    /** Fusion half of ins_core_update() (see ins_correct_step()): a no-op
     *  if the matching ins_core_predict() dropped the epoch or was never
     *  called. */
    void ins_core_correct(void* h);

    /** Set the magnetic reference from a position via the World Magnetic
     *  Model (declination/inclination/field strength): yaw becomes relative
     *  to TRUE north and the field-strength disturbance gate is armed.
     *  May be called at any time (e.g. on the first GNSS fix). */
    void ins_core_set_magnetic_model_position(void* h, double lat_rad, double lon_rad, float year);

    int ins_core_is_ready(void* h);
    int ins_core_deadreckoning_ms(void* h);
    int ins_core_get_position_ecef(void* h, double out_ecef[3]);
    /* The geodetic anchor as the filter holds it: lat [rad], lon [rad],
     * height over the ellipsoid [m]. Cheaper and more direct than taking
     * the ECEF above and converting it back, which is where it comes from
     * (REQ-NAV-078). */
    int ins_core_get_latlonh(void* h, double out_llh[3]);
    int ins_core_get_position_local(void* h, float out_ned[3]);
    int ins_core_get_velocity_ned(void* h, float out_ned[3]);
    int ins_core_get_quaternion(void* h, float out_wxyz[4]);
    int ins_core_get_rotmat_b_to_n(void* h, float out9[9]); /**< column-major */
    int ins_core_get_rpy(void* h, float out_rpy[3]);
    int ins_core_get_omega_b_nb(void* h, float out[3]);
    int ins_core_get_acc_n(void* h, float out_ned[3]);
    int ins_core_get_origin_ecef(void* h, double out_ecef[3]); /**< local NED frame origin */
    int ins_core_get_bias_acc(void* h, float out[3]);          /**< [m/s^2] */
    int ins_core_get_bias_gyr(void* h, float out[3]);          /**< [rad/s] */
    int ins_core_get_bias_mag(void* h, float out[3]);          /**< [uT]; only estimated
                                                               with estimate_mag_bias */
    /** Full error-state covariance P = U*diag(d)*U', read directly from the
     *  filter's UDU factors (no extra computation is added to the filter
     *  itself). Column-major, flattened into a fixed
     *  INS_CAPI_MAX_STATE x INS_CAPI_MAX_STATE buffer; only the leading
     *  n x n block (the returned n) is meaningful, the rest is zeroed.
     *  State order: pos_ned(3) vel_ned(3) rpy(3, small-angle NED attitude
     *  error) acc_bias(3) gyr_bias(3) [mag_bias(3) in 18-state mode].
     *  Returns n (15 or 18), or 0 if the filter is not yet initialized. */
    int ins_core_get_covariance(void* h, float P_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE]);
    /** Diagnostic counters: n_predict, n_gnss_seen, n_gnss_used,
     *  n_gnss_rejected_noise, n_gnss_no_anchor, n_fuse_fail, n_auto_zupt,
     *  n_invalid_input, n_downweighted (REQ-NAV-036), n_baro_height_used
     *  (REQ-NAV-054, 0 whenever the GNSS height source is in use). */
    void ins_core_get_diag(void* h, uint32_t out[10]);
    /** Absolute-speed aiding counters, see ins_suite_get_speed_diag. */
    void ins_core_get_speed_diag(void* h, uint32_t out_counts[3], float* out_last_residual_mps);
    /** Timestamp-health counters, see ins_suite_get_time_diag. */
    void ins_core_get_time_diag(void* h, uint32_t out_counts[3]);
    /** Overconfidence / covariance-collapse watchdog (REQ-NAV-040): returns
     *  n_overconfident (epochs whose reported 1-sigma tripped a floor; 0 =
     *  never) and fills out_min_stddev with the smallest position [m],
     *  velocity [m/s] and attitude [deg] per-axis 1-sigma seen (INFINITY
     *  until the first post-init epoch). */
    uint32_t ins_core_get_overconfidence(void* h, float out_min_stddev[3]);
    /** 1 if the automatic ZUPT/ZARU detector currently considers the
     *  filter stationary (ins_auto_zupt_active), else 0. */
    int ins_core_auto_zupt_active(void* h);

    /* ======================= nav_suite (ins_suite_*) ==============================
     */

    void* ins_suite_create(void);
    void  ins_suite_destroy(void* h);
    int   ins_suite_init(void* h, const ins_cfg_t* cfg);

    void ins_suite_set_imu(void* h, int64_t t_us, float dt_sec, const float acc[3],
                           const float gyr[3], const float acc_var[3], const float gyr_var[3]);
    void ins_suite_set_gnss_pos_llh(void* h, const double llh[3], const float var_ned[3]);
    void ins_suite_set_gnss_vel_ned(void* h, const float vel_ned[3], const float var_ned[3]);
    void ins_suite_set_gnss_pos_llh_cov(void* h, const double llh[3], const float Qll_ned[9]);
    void ins_suite_set_gnss_vel_ned_cov(void* h, const float vel_ned[3], const float Qll_ned[9]);
    void ins_suite_set_gnss_pos_vel_cov(void* h, const float Q_pos_vel_ned[9]);
    void ins_suite_set_gnss_leverarm_b(void* h, const float lever_b[3]);
    void ins_suite_set_mag(void* h, const float mag[3], const float var[3]);
    void ins_suite_set_yaw(void* h, float yaw_rad, float stddev_rad);
    void ins_suite_set_local_pos(void* h, const float pos_ned[3], const float var_ned[3],
                                 const float lever_b[3]);
    void ins_suite_set_zupt(void* h, int on);
    void ins_suite_set_zaru(void* h, int on);
    /** Arm/disarm the ARS/AHRS's velocity-blind auto-ZARU fallback
     *  (ahrs_config_t.auto_zaru_disable, REQ-AHRS-017) per instance.
     *  Rarely needed: the fallback is ARMED by default and configured
     *  from the same ins_cfg_t.auto_zupt_* set as ins itself
     *  (REQ-SUITE-020), including the two opt-outs
     *  (auto_zupt_disable, auto_zupt_velocity_blind_disable). Use this
     *  only to drive the two instances differently, and be aware that
     *  it overrides whatever the config said. Must be called after
     *  ins_suite_init() and before the first ins_suite_update() (config
     *  template, like the other ars_cfg/ahrs_cfg tuning). */
    void ins_suite_set_auto_zaru(void* h, int ars_on, int ahrs_on);
    void ins_suite_set_baro(void* h, float pressure_pa, float stddev_m);
    /** Absolute speed aiding (REQ-NAV-068), see ins_core_set_speed. */
    void ins_suite_set_speed(void* h, float speed_mps, float stddev_mps, int delay_ms);
    void ins_suite_set_baro_acc_bias_drift(void* h, float density_mps2_sqrthz);
    /** Set the baro/accel vertical filter's own accel-noise density sigma_a
     *  [m/s^2/sqrt(Hz)] (<=0 -> baro_alt's own default, no-op). Must be
     *  called before the first baro sample latches baro_cfg into the
     *  filter (same contract as ins_suite_set_baro_acc_bias_drift). */
    void ins_suite_set_baro_acc_noise(void* h, float density_mps2_sqrthz);
    /** Set baro_alt's own INITIAL accel-bias uncertainty [m/s^2] (<=0 ->
     *  baro_alt default). An INITIAL CONDITION, not a process-noise rate --
     *  see ins_suite_set_baro_acc_noise's neighbor doc comment in
     *  ins_capi.c for why it matters. Same timing contract as
     *  ins_suite_set_baro_acc_bias_drift. */
    void ins_suite_set_baro_acc_bias_init_stddev(void* h, float stddev_mps2);
    /** Set baro_alt's own INITIAL height/velocity uncertainty ([m]/[m/s],
     *  <=0 -> baro_alt defaults 1.0/0.5). Same timing contract as
     *  ins_suite_set_baro_acc_bias_drift. */
    void ins_suite_set_baro_h_init_stddev(void* h, float stddev_m);
    void ins_suite_set_baro_v_init_stddev(void* h, float stddev_mps);
    /** Set baro_alt's own small direct height process-noise density sigma_h
     *  [m/sqrt(Hz)] (<=0 -> baro_alt's own default, no-op) -- a safety
     *  margin against discretization/model mismatch, not a primary noise
     *  source. Same timing contract as ins_suite_set_baro_acc_bias_drift. */
    void ins_suite_set_baro_h_process_noise(void* h, float density_m_sqrthz);
    /** Set the local-height/GNSS-ellipsoid offset filter's random walk
     *  [m/sqrt(s)] (<=0 -> local_gnss_alt default 0.03, no-op). Raise this
     *  for missions with large altitude excursions (e.g. a soaring glider
     *  gaining kilometres of altitude): the offset absorbs the ISA-model
     *  error, which grows with the excursion (REQ-BARO-011) and can
     *  outrun the default random walk, tuned for a multi-metre swing.
     *  Must be called before the offset filter's first local-height/GNSS
     *  pair -- in practice, right after construction, before the first
     *  ins_suite_update(). */
    void ins_suite_set_local_gnss_rw_stddev(void* h, float rw_stddev_mps);
    /** Set the offset filter's chi2 outlier gate on the innovation (<=0 ->
     *  local_gnss_alt default, chi2inv(0.95,1), no-op). Same timing
     *  contract as ins_suite_set_local_gnss_rw_stddev. */
    void ins_suite_set_local_gnss_chi2_threshold(void* h, float threshold);
    /** Set the offset filter's minimum time between two fusions [s] (<=0
     *  -> local_gnss_alt default 10, no-op). Same timing contract as
     *  ins_suite_set_local_gnss_rw_stddev. */
    void ins_suite_set_local_gnss_min_update_interval(void* h, float interval_sec);
    /** Set the offset filter's stddev inflation factor applied to both
     *  sides of the pair before combining into the measurement variance
     *  (<=0 -> local_gnss_alt default 3, no-op). Same timing contract as
     *  ins_suite_set_local_gnss_rw_stddev. */
    void ins_suite_set_local_gnss_stddev_inflation(void* h, float factor);
    /** Set the ARS's/AHRS's SHARED gyro noise density sigma_g
     *  [rad/s/sqrt(Hz)] (<=0 -> ahrs's own default, no-op) -- both
     *  sub-filters consume the same physical gyro, so one call sets both
     *  ars_cfg.gyr_noise_psd and ahrs_cfg.gyr_noise_psd. Must be called
     *  after ins_suite_init() and before the first ins_suite_update()
     *  (config template, like ins_suite_set_auto_zaru). */
    void ins_suite_set_ahrs_gyr_noise(void* h, float density_rps_sqrthz);
    /** Set the ARS's/AHRS's SHARED accelerometer noise stddev [m/s^2]
     *  (<=0 -> ahrs's own default, AHRS_DEFAULT_ACC_NOISE_MPS2) -- both
     *  sub-filters consume the same physical accelerometer for leveling,
     *  so one call sets both ars_cfg.acc_noise_mps2 and
     *  ahrs_cfg.acc_noise_mps2. Same timing contract as
     *  ins_suite_set_ahrs_gyr_noise. */
    void ins_suite_set_ahrs_acc_noise(void* h, float stddev_mps2);
    /** Set the ARS's/AHRS's SHARED gyro bias random walk density
     *  [rad/s^2/sqrt(Hz)] (<=0 -> ahrs's own default, no-op); same timing
     *  contract as ins_suite_set_ahrs_gyr_noise. */
    void ins_suite_set_ahrs_gyr_bias_rw(void* h, float density_rps2_sqrthz);
    /** Set the ARS's/AHRS's SHARED initial gyro-bias uncertainty [rad/s],
     *  all 3 axes (<=0 -> ahrs default, 1/1/5 deg/s xy/z). An INITIAL
     *  CONDITION, not a process-noise rate -- but it dominates yaw's
     *  apparent growth for a long time via the attitude/bias Phi coupling
     *  if left at the (generous) default. Same timing contract as
     *  ins_suite_set_ahrs_gyr_noise. */
    void ins_suite_set_ahrs_gyr_bias_init_stddev(void* h, float stddev_rps);
    /** Known initial roll/pitch and/or yaw for ins's auto-init bootstrap
     *  (REQ-NAV-048's att_hint, the static "I just know it" case -- e.g. a
     *  known heading with no mag/GNSS-course yaw aiding to derive it
     *  from). Call once, right after construction, before the first
     *  update(). Roll/pitch are only used together (stddev_roll_pitch_rad
     *  <= 0 -> no roll/pitch hint); yaw is independent (stddev_yaw_rad
     *  <= 0 -> no yaw hint). Stops applying once ins first initializes --
     *  does not bias any later re-acquisition. */
    void ins_suite_set_init_att_hint(void* h, float roll_rad, float pitch_rad,
                                     float stddev_roll_pitch_rad, float yaw_rad,
                                     float stddev_yaw_rad);
    void ins_suite_set_gnss_delay_ms(void* h, int ms);
    void ins_suite_set_yaw_delay_ms(void* h, int ms);
    void ins_suite_set_local_pos_delay_ms(void* h, int ms);
    void ins_suite_update(void* h);

    /** Time-propagation half of ins_suite_update() (see
     *  nav_suite_predict_step()): only the ins sub-filter's predict is
     *  split out here (see nav_suite_predict_step()'s doc comment for why
     *  ARS/AHRS/baro_alt stay bundled in ins_suite_correct()). Must be
     *  followed by exactly one ins_suite_correct() call before the next
     *  ins_suite_predict()/ins_suite_update().
     *  @param[out] phi_out Same convention as ins_core_predict()'s,
     *      forwarded from the suite's ins sub-filter.
     *  @return Bitwise OR of ins.h's INS_EPOCH_* flags. */
    int ins_suite_predict(void* h, float phi_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE]);
    /** Fusion half of ins_suite_update() (see nav_suite_correct_step()): a
     *  no-op if the matching ins_suite_predict() was never called. */
    void ins_suite_correct(void* h);

    /** WMM from position -- applied to the suite's ins AND its
     *  magnetometer AHRS, so both attitude sources agree on true north. */
    void ins_suite_set_magnetic_model_position(void* h, double lat_rad, double lon_rad, float year);

    int ins_suite_get_mode(void* h);                  /**< INS_MODE_* */
    int ins_suite_get_rpy(void* h, float out_rpy[3]); /**< best available attitude */
    int ins_suite_get_rpy_ins(void* h, float out_rpy[3]);
    int ins_suite_get_rpy_ars(void* h, float out_rpy[3]);
    int ins_suite_get_rpy_ahrs(void* h, float out_rpy[3]);
    /** ARS's/AHRS's own gyro bias estimate [rad/s] and its 1-sigma
     *  uncertainty [rad/s] (from their own, independent error-state
     *  covariance -- not ins's). */
    int ins_suite_get_bias_gyr_ars(void* h, float out[3]);
    int ins_suite_get_bias_gyr_ahrs(void* h, float out[3]);
    int ins_suite_get_gyr_bias_stddev_ars(void* h, float out[3]);
    int ins_suite_get_gyr_bias_stddev_ahrs(void* h, float out[3]);
    /** 1-sigma roll/pitch/yaw uncertainty (ahrs_get_rpy_stddev, ahrs.h) --
     *  ARS's/AHRS's OWN attitude covariance, not ins's. */
    int ins_suite_get_rpy_stddev_ars(void* h, float out[3]);
    int ins_suite_get_rpy_stddev_ahrs(void* h, float out[3]);

    /* ins-level state of the suite's ins instance. */
    int ins_suite_is_ready(void* h);
    int ins_suite_deadreckoning_ms(void* h);
    int ins_suite_get_position_ecef(void* h, double out_ecef[3]);
    /* The geodetic anchor as the filter holds it: lat [rad], lon [rad],
     * height over the ellipsoid [m]. Cheaper and more direct than taking
     * the ECEF above and converting it back, which is where it comes from
     * (REQ-NAV-078). */
    int ins_suite_get_latlonh(void* h, double out_llh[3]);
    int ins_suite_get_position_local(void* h, float out_ned[3]);
    int ins_suite_get_velocity_ned(void* h, float out_ned[3]);
    int ins_suite_get_quaternion(void* h, float out_wxyz[4]);
    int ins_suite_get_rotmat_b_to_n(void* h, float out9[9]);
    int ins_suite_get_omega_b_nb(void* h, float out[3]);
    int ins_suite_get_acc_n(void* h, float out_ned[3]);
    int ins_suite_get_origin_ecef(void* h, double out_ecef[3]);
    int ins_suite_get_bias_acc(void* h, float out[3]);
    int ins_suite_get_bias_gyr(void* h, float out[3]);
    int ins_suite_get_bias_mag(void* h, float out[3]);
    /** Same as ins_core_get_covariance, for the suite's ins instance. */
    int  ins_suite_get_covariance(void* h, float P_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE]);
    void ins_suite_get_diag(void* h, uint32_t out[10]);
    /** Absolute-speed aiding counters (REQ-NAV-068), kept out of the fixed
     *  get_diag() array so that array's ABI stays put: out_counts receives
     *  {seen, used, skipped} and out_last_residual_mps the most recent
     *  ||v_n|| - z. */
    void ins_suite_get_speed_diag(void* h, uint32_t out_counts[3], float* out_last_residual_mps);
    /** Timestamp-health counters (REQ-NAV-016, REQ-NAV-070), likewise kept
     *  out of the fixed get_diag() array: out_counts receives
     *  {n_time_backward, n_time_dropped, n_time_restart_reset}. A rising
     *  n_time_dropped with a frozen solution is a restarted time source. */
    void ins_suite_get_time_diag(void* h, uint32_t out_counts[3]);
    /** Same as ins_core_get_overconfidence, for the suite's ins instance
     *  (REQ-NAV-040). */
    uint32_t ins_suite_get_overconfidence(void* h, float out_min_stddev[3]);
    /** Attitude overconfidence watchdog for the suite's ARS / AHRS attitude
     *  filters (REQ-AHRS-020): returns n_overconfident, fills the smallest
     *  attitude 1-sigma [deg] seen (attitude-only; no pos/vel). */
    uint32_t ins_suite_get_ars_overconfidence(void* h, float* out_min_att_deg);
    uint32_t ins_suite_get_ahrs_overconfidence(void* h, float* out_min_att_deg);

    /** Downweight counters (REQ-AHRS-019, REQ-BARO-017) for the suite's
     *  other sub-filters -- no bare-core equivalent, ars/ahrs/baro only
     *  exist in the suite. ins's own is in ins_suite_get_diag() above. */
    uint32_t ins_suite_get_ars_downweight_count(void* h);
    uint32_t ins_suite_get_ahrs_downweight_count(void* h);
    uint32_t ins_suite_get_baro_downweight_count(void* h);
    uint32_t ins_suite_get_local_gnss_downweight_count(void* h);

    /** 1 if ins's own auto-ZUPT/ZARU detector currently considers the
     *  suite stationary, else 0 (ins_auto_zupt_active). */
    int ins_suite_auto_zupt_active(void* h);
    /** 1 if a zero-rotation update was applied to the ARS/AHRS on the
     *  last ins_suite_update() call (explicit flag OR'd with the above),
     *  else 0 (nav_suite_get_zaru_active, REQ-SUITE-010). */
    int ins_suite_zaru_active(void* h);
    /** 1 if the ARS's/AHRS's own velocity-blind auto-ZARU fallback has
     *  STARTED a stillness run (its loose gyro/accel magnitude gate is
     *  satisfied), else 0 (ahrs_auto_zaru_active). NOT the same as "a
     *  ZARU was actually applied": this is true for the whole run
     *  (including e.g. genuine constant-velocity cruise, which the loose
     *  gate cannot tell from a stop), well before the stricter windowed-
     *  variance criterion and dwell time below have been satisfied -- use
     *  ins_suite_ars_zaru_applied/ins_suite_ahrs_zaru_applied instead for
     *  "did a ZARU actually fire this epoch". */
    int ins_suite_ars_auto_zaru_active(void* h);
    int ins_suite_ahrs_auto_zaru_active(void* h);
    /** 1 if the ARS's/AHRS's own last ahrs_update() call actually saw a
     *  zero-rotation trigger fire, else 0 (ahrs_zaru_applied, REQ-AHRS-
     *  024): the loose gate above AND the windowed-variance criterion AND
     *  the dwell time, OR'd with whatever nav_suite passed down from ins
     *  (ins_suite_zaru_active) -- the decision that actually mattered,
     *  unlike ins_suite_ars_auto_zaru_active/ins_suite_ahrs_auto_zaru_
     *  active above. */
    int ins_suite_ars_zaru_applied(void* h);
    int ins_suite_ahrs_zaru_applied(void* h);
    /** 1 if a zero-velocity update actually reached the vertical channel
     *  (baro_alt) on the last ins_suite_update() call, else 0
     *  (nav_suite_get_vertical_zupt_active, REQ-SUITE-015). Wider than
     *  ins_suite_zaru_active: the ARS/AHRS auto-ZARU fallback counts too,
     *  which is the only stillness source while ins is attitude-only. */
    int ins_suite_vertical_zupt_active(void* h);

    /* Vertical channel (see nav_suite.h; all heights positive up, sharing
     * the NED-origin datum). */
    int ins_suite_get_baro_alt(void* h, float* h_m, float* v_mps); /**< baro filter h/vz */
    /** baro_alt's own z-accel bias estimate [m/s^2] and its 1-sigma
     *  [h m, v m/s, acc_bias m/s^2] uncertainty (own error-state
     *  covariance, independent of ins's). */
    int ins_suite_get_baro_acc_bias(void* h, float* acc_bias_mps2);
    int ins_suite_get_baro_stddev(void* h, float out[3]);
    /** baro-to-ellipsoid offset filter (nav_suite.h): estimated offset
     *  [m] such that ellipsoid height ~= barometric ISA altitude +
     *  offset, and its 1-sigma uncertainty [m]. False until a baro/GNSS
     *  pair has been seen (see nav_suite_update). */
    int ins_suite_get_local_gnss_offset(void* h, float* offset_m, float* stddev_m);
    int ins_suite_get_height(void* h, float* h_m);               /**< best height above datum */
    int ins_suite_get_height_ellipsoid(void* h, float* h_ell_m); /**< best absolute height */

#ifdef __cplusplus
}
#endif

#endif /* INS_CAPI_H */
