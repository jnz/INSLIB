/** @file nav_suite.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Wrapper running the 3D INS filter, an ARS and AHRS filters and a
 * baro/accel vertical filter in parallel.
 *
 * One ins_update() equivalent drives four filters from the same measurement
 * stream:
 *
 *   - ins: the full 15/18-state navigation filter (position, velocity,
 *     attitude, biases), see ins.h.
 *   - ars:   AHRS_MODE_ARS roll/pitch filter. Yaw is integrated from the
 *            gyroscope but never corrected (directional gyro).
 *   - ahrs:  AHRS_MODE_AHRS roll/pitch/yaw filter. Yaw is stabilized with
 *            magnetometer measurements. Only runs once a magnetometer sample
 *            has been seen.
 *   - baro_alt: baro/accelerometer vertical channel filter (height, vertical
 *            velocity, vertical acceleration correction), see baro_alt.h.
 *            Only runs once a barometer sample has been seen; the body-to-NED
 *            rotation comes from the best available attitude source.
 *   - local_gnss: 1-state offset filter estimating the ellipsoid height of the
 *            NED origin, fed from epochs carrying both a GNSS position and a
 *            local height; enables an absolute height during GNSS outages
 *            (nav_suite_get_height_ellipsoid).
 *
 * Height systems:
 *
 *   1. LOCAL height: height above the NED origin, positive up, always zero at
 *      the start point (nav_suite_get_height).
 *   2. BAROMETRIC (ISA) altitude: baro_alt_get_isa_altitude(). Carries the
 *      weather offset and the ISA model error, not absolute.
 *   3. ELLIPSOID height: absolute, WGS84 (nav_suite_get_height_ellipsoid).
 *      Available directly from ins under GNSS, during an outage as local
 *      height + the estimated offset. Returns false while neither is possible.
 *
 * Vertical datum: all local heights (INS -pos_local[2], baro_alt h) share the
 * NED origin as their zero. Whoever initializes first defines it: if INS is
 * initialized when the barometer appears, baro_alt anchors at the current NED
 * height; if the barometer was first, the INS origin is shifted at bootstrap so
 * the local height matches the baro height (the absolute solution is untouched)
 * and the offset filter is shifted with it. nav_suite_get_height() arbitrates
 * the best height source.
 *
 * The offset filter is fed with the local height source that survives a GNSS
 * outage (baro_alt if running, else the ins local height), not with the
 * momentarily best one: the offset has to absorb that source's drift against
 * the ellipsoid while GNSS is up, because that source is what remains when GNSS
 * drops. The same reference evaluates nav_suite_get_height_ellipsoid(). GNSS +
 * baro + an external local height at the same time is not supported: the
 * barometer wins as the reference.
 *
 * The AHRS instances are useful as independent attitude references
 * (cross-check/fallback for the INS attitude): they only consume acc/gyr/mag
 * and keep working when position aiding is unavailable. The baro_alt filter
 * plays the same role for the vertical channel.
 *
 * The AHRS and baro_alt instances initialize themselves from the measurement
 * stream: roll/pitch are leveled from the first valid accelerometer sample, the
 * AHRS instance additionally waits for the first magnetometer sample, baro_alt
 * for the first pressure sample. To tune their noise models, edit s->ars_cfg /
 * s->ahrs_cfg / s->baro_cfg between nav_suite_init() and the first
 * nav_suite_update(). Where ins was given a prescribed attitude, nav_suite_init()
 * seeds the ARS's initial yaw from that known heading instead of an arbitrary 0.
 *
 * Some settings are not per-filter, because they are one decision about the
 * whole system rather than four independent tunings: outlier rejection
 * (ins_options_t.chi2_disable, REQ-SUITE-011) and the definition of standstill
 * (ins_options_t.auto_zupt_*, REQ-SUITE-020). nav_suite_init() copies both from
 * ins's config into every sub-filter template, so the caller states them once.
 * Each filter still implements them itself; the propagation only keeps their
 * thresholds from drifting apart, and a caller who wants them apart can
 * overwrite the templates afterwards.
 *
 * All memory is part of the nav_suite_t struct - no heap is used.
 *
 */

/** @addtogroup nav_suite
 *  @{ */

#ifndef NAV_SUITE_H
#define NAV_SUITE_H

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "ins.h"
#include "ahrs.h"
#include "baro_alt.h"

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/** @brief Solution mode of the suite (graceful degradation, e.g. during
 *  a tunnel passage, see nav_suite_get_mode()). */
typedef enum
{
    NAV_SUITE_MODE_NONE = 0,      /**< nothing usable yet */
    NAV_SUITE_MODE_ATTITUDE_ONLY, /**< INS position/velocity unusable
                                       (coasting window exceeded or not
                                       initialized), attitude available
                                       from the ARS or AHRS filters */
    NAV_SUITE_MODE_COASTING,      /**< INS ready, but dead-reckoning on
                                       IMU only (no recent position
                                       aiding): position drifts */
    NAV_SUITE_MODE_FULL           /**< INS ready with recent absolute
                                       position aiding */
} nav_suite_mode_t;

/** @brief Suite instance: ins + the two AHRS filters + baro_alt.
 *
 *  Zero the struct before calling nav_suite_init(). The members are
 *  public: the individual filters can be inspected with their own
 *  accessors (ins_get_*, ahrs_get_*, baro_alt_get_*). */
typedef struct
{
    ins_t            ins;        /**< full navigation filter */
    ahrs_t           ars;        /**< roll/pitch filter (yaw free) */
    ahrs_t           ahrs;       /**< roll/pitch/yaw filter (magnetometer) */
    baro_alt_t       baro_alt;   /**< baro/accel vertical channel filter */
    local_gnss_alt_t local_gnss; /**< local-height-to-ellipsoid offset
                                    filter, fed from epochs carrying both
                                    a GNSS position and a local height */

    /* Config templates consumed at auto-initialization (the AHRS
       rpy_init fields are overwritten by the leveling/heading
       bootstrap). */
    ahrs_config_t           ars_cfg;        /**< template for the ars filter */
    ahrs_config_t           ahrs_cfg;       /**< template for the ahrs filter */
    baro_alt_config_t       baro_cfg;       /**< template for the baro_alt filter */
    local_gnss_alt_config_t local_gnss_cfg; /**< template for the local_gnss filter */

    /** Caller-supplied static "I just know it" initial attitude hint (see
     *  nav_suite_set_init_att_hint): the fallback nav_suite_build_att_hint uses
     *  for INS's auto-init bootstrap while neither the ARS nor the AHRS has
     *  initialized yet. Set once, right after nav_suite_init(). */
    ins_meas_att_hint_t init_att_hint;

    /* Vertical datum arbitration (see nav_suite_update): true once the ins
       origin height and the baro_alt datum agree (or no alignment is needed).
       Held per n-frame ORIGIN, not per INS instance: the alignment re-arms when
       a re-bootstrap anchors a new origin, and stays put when the origin is
       carried across a re-arm (REQ-NAV-062). */
    bool   vertical_datum_aligned; /**< true once INSLIB/baro_alt share one datum */
    double datum_origin_llh[3];    /**< the ins origin that agreement belongs to,
                                        lat,lon [rad], h [m]; only meaningful
                                        while vertical_datum_aligned is true */

    /* Where the offset filter stood when the datum above was fixed, i.e. the
       ellipsoid height ins's vertical datum was anchored to. Under the
       barometric height source that anchor is never re-tied to GNSS
       (REQ-NAV-055), so the offset filter's movement AWAY from this value is
       exactly how far the datum has since drifted - which is what
       nav_suite_get_height_ellipsoid() takes back off ins's absolute height
       (REQ-SUITE-008). Captured lazily, on the first epoch after the datum
       latch that carries an offset estimate; cleared whenever the datum
       re-arms on a new origin. */
    float datum_offset_m;                   /**< the offset the datum was anchored at [m] */
    float datum_offset_var_min;             /**< smallest offset variance seen so far [m^2],
                                                  the convergence detector; also the
                                                  anchor's own variance once frozen;
                                                  0 -> none yet */
    baro_alt_time_us_t datum_offset_t_last; /**< local_gnss.t_last the detector
                                      last looked at, so it judges FUSIONS and
                                      not the epochs decimated between them
                                      (where the variance cannot move) */
    bool datum_offset_valid;                /**< true once datum_offset_m has settled */

    /* Latched once any local NED position sample has been seen: an
       external mocap system (e.g. lighthouse) then owns the n-frame, so the
       vertical datum alignment never shifts the INS origin (REQ-SUITE-007).
       ins_shift_origin_down() would desynchronize that external frame. */
    bool local_frame_external; /**< true -> an external mocap system owns the n-frame */

    /* Constant offset that maps the external local-position frame onto the
       vertical datum (REQ-SUITE-013): INS is fed local_pos with this
       subtracted, so it always solves at (0,0,h). external_coord = ins_local +
       offset. Tracked until ins locks the datum at bootstrap, then frozen. */
    float local_pos_offset_ned[3]; /**< external-frame coords of the datum origin [m] */
    bool  local_pos_offset_valid;  /**< true once a local_pos sample has been seen */
    bool  local_pos_datum_locked;  /**< true once ins froze the offset at bootstrap */

    /* Latched once a usable GNSS fix has been presented: only then is the ins
       n-frame origin anchored to WGS84, so an absolute/ellipsoid output is real
       rather than the prescribed init origin (REQ-SUITE-008). */
    bool wgs84_anchor_seen; /**< true -> ins solution is GNSS-anchored to WGS84 */

    /* True if a known initial heading was supplied - either as a prescribed
       initial attitude (opt.auto_init == false) or as a static yaw hint
       (nav_suite_set_init_att_hint): the ARS then bootstraps its yaw from that
       known value instead of the arbitrary 0 (REQ-SUITE-002). */
    bool ars_yaw_from_init; /**< true -> ARS yaw seeded from a known heading, not 0 */

    /** Heading carry-over across an ins re-initialization (REQ-SUITE-022): the
     *  last 3D heading INS held while converged, paired with the ARS yaw of
     *  that same epoch. Refreshed every epoch INS is ready, so after a shutdown
     *  it still holds the last good epoch. INS's re-init yaw is that heading
     *  plus the yaw CHANGE the ARS free-integrated in between: the ARS drifts
     *  in absolute terms but tracks the change well. Dropped when the ARS
     *  re-bootstraps. The same reconstruction is what nav_suite_get_rpy()
     *  reports while it falls back to the ARS, resolved once per epoch into
     *  out_yaw_rad. */
    struct
    {
        bool          valid;       /**< a usable pair has been latched */
        float         yaw_ins_rad; /**< ins yaw at the latch epoch [rad] */
        float         yaw_ars_rad; /**< ARS yaw at the same epoch [rad] */
        float         stddev_rad;  /**< ins's own yaw 1-sigma there [rad] */
        ins_time_us_t t;           /**< latch timestamp */
        bool          out_valid;   /**< out_yaw_rad holds this epoch's value */
        float         out_yaw_rad; /**< latched heading + the ARS's yaw
                                        change since, as of the last
                                        update [rad] */
    } yaw_carry;

    /* Zero-rotation trigger given to the ARS/AHRS on the last
       nav_suite_update() call (REQ-SUITE-009/-010): the caller's
       explicit m->zero_rotation_update flag OR'd with ins's own
       auto-ZUPT/ZARU detector. Read via nav_suite_get_zaru_active() for
       diagnostics/telemetry. */
    bool last_zaru_trigger;  /**< zero-rotation update applied to ARS/AHRS last epoch */
    bool last_vertical_zupt; /**< zero-velocity update applied to baro_alt last epoch */

    /* Averaged baro bootstrap (REQ-SUITE-006): the datum anchor is the MEAN of
       the plausible pressure samples seen in a short window at bootstrap, so
       one glitched startup reading cannot skew the vertical datum. The
       accumulator resets whenever baro_alt is not initialized and idle. */
    double             baro_boot_p_sum; /**< sum of collected pressures [Pa] */
    uint32_t           baro_boot_count; /**< samples in the sum (0 -> idle) */
    baro_alt_time_us_t baro_boot_t0;    /**< timestamp of the first collected sample */

    /* Handoff from nav_suite_predict_step() to nav_suite_correct_step() for one
       epoch (REQ-SUITE-021): only the caller's original, unmodified measurement
       block needs to survive the call boundary, everything ins-specific is
       already carried inside ins_t.step_ctx. */
    struct
    {
        ins_measurements_t m;
        bool               active; /**< nav_suite_correct_step() has work to do */
    } step_ctx;                    /**< per-epoch scratch handed from
                                        nav_suite_predict_step() to nav_suite_correct_step() */

    /** Logging-only bookkeeping (see log.h): pure edge-/rate-detection so the
     *  optional LOG_* calls in nav_suite.c stay informative instead of flooding
     *  the sink. Never read or acted on by the wrapper itself. */
    struct
    {
        nav_suite_mode_t last_mode;         /**< nav_suite_get_mode() as of the
                                                  previous nav_suite_update() call */
        ins_time_us_t t_last_gyr_bias_warn; /**< throttle for the INS/ARS gyro
                                                  bias cross-check warning
                                                  (0 = none yet) */
        ins_time_us_t t_last_vvel_warn;     /**< throttle for the INS/baro_alt
                                                  vertical-velocity cross-check
                                                  warning (0 = none yet) */
        ins_time_us_t t_last_height_warn;   /**< throttle for the INS/baro_alt
                                                  height cross-check warning
                                                  (REQ-SUITE-019, 0 = none yet) */
        ins_time_us_t t_last_yaw_warn;      /**< throttle for the INS/AHRS
                                                  heading cross-check warning
                                                  (REQ-SUITE-023, 0 = none yet) */
        bool last_ins_initialized;          /**< ins.is_initialized as of the
                                                  previous nav_suite_update()
                                                  call, to spot a shutdown */
    } log_state;
} nav_suite_t;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Initialise the suite.
     *
     *  Initialises INS with the given init/opt and prepares the AHRS config
     *  templates with defaults. The AHRS filters themselves start on the first
     *  suitable measurement epoch, except the ARS's initial yaw, which is
     *  seeded here from init->rpy_init_rad[2] when opt->auto_init is false.
     *
     *  @param[in,out] s The suite instance (must be zeroed).
     *  @param[in] init ins initial state and std.-devs.
     *  @param[in] opt  ins filter options.
     *  @return 0 on success, -1 on failure (including NULL @p init / @p opt). */
    int nav_suite_init(nav_suite_t* s, const ins_init_t* init, const ins_options_t* opt);

    /** @brief Arm a static "I just know it" initial attitude hint for ins's
     *  auto-init bootstrap (e.g. a known launch heading with no
     *  magnetometer/GNSS-course yaw aiding to derive it from).
     *
     *  Call once, right after nav_suite_init(), before the first
     *  nav_suite_update().
     *
     *  Only used while neither the ARS nor the AHRS has initialized yet (they
     *  take priority once available) and only until INS itself initializes, so
     *  a stale "initial" hint cannot bias a later re-acquisition
     *  (REQ-NAV-048). A yaw hint additionally seeds the yaw-free ARS's own
     *  bootstrap (REQ-SUITE-002); roll/pitch are not seeded there, the ARS
     *  levels them from gravity anyway.
     *
     *
     *  @param[in,out] s The suite instance.
     *  @param[in] roll_rad, pitch_rad Known initial roll/pitch [rad], only
     *      used together.
     *  @param[in] stddev_roll_pitch_rad Shared 1-sigma [rad]; <= 0 -> no
     *      roll/pitch hint.
     *  @param[in] yaw_rad Known initial yaw [rad], independent of roll/pitch.
     *  @param[in] stddev_yaw_rad 1-sigma [rad]; <= 0 -> no yaw hint. */
    void nav_suite_set_init_att_hint(nav_suite_t* s, float roll_rad, float pitch_rad,
                                     float stddev_roll_pitch_rad, float yaw_rad,
                                     float stddev_yaw_rad);

    /** @brief Feed one measurement epoch to all three filters.
     *
     *  The full bundle goes to INS (see ins_update), the AHRS filters
     *  consume timestamp + acc + gyr (+ mag for the AHRS instance).
     *
     *  @param[in,out] s The suite instance.
     *  @param[in] m Measurements for this epoch. */
    void nav_suite_update(nav_suite_t* s, const ins_measurements_t* m);

    /** @brief Time-propagation part of nav_suite_update(): the local-datum
     *  adjustment, the ARS/AHRS attitude hint, and ins_predict_step() on the
     *  INS sub-filter.
     *
     *  Must be followed by exactly one nav_suite_correct_step() call before the
     *  next nav_suite_predict_step(): nav_suite_t.step_ctx has room for exactly
     *  one pending epoch. Calling it twice silently overwrites step_ctx, so the
     *  skipped epoch never reaches ins_correct_step(), the vertical datum
     *  alignment, the local-GNSS offset filter or the ARS/AHRS/baro_alt
     *  bootstrap-or-update calls. Batching predicts buys nothing: INS's
     *  internal covariance throttle already gives the "propagate fast, correct
     *  slow" effect.
     *
     *  The ARS/AHRS/baro_alt sub-filters are NOT split at the suite level:
     *  their bootstrap-or-update branching and the cross-filter feeds between
     *  them sit strictly between "INS done" and "everything else". A caller
     *  that wants to sample their covariance between prediction and correction
     *  can call ahrs_predict_step() / baro_alt_predict_step() directly.
     *
     *  @param[in,out] s The suite instance.
     *  @param[in] m Measurements for this epoch.
     *  @param[out] phi_out Optional (nullable) buffer for ins's
     *      discrete-time state transition matrix, forwarded verbatim to
     *      ins_predict_step() (see there for sizing/validity).
     *  @return Bitwise OR of INS_EPOCH_* flags (ins.h), forwarded verbatim
     *      from the underlying ins_predict_step() call. */
    int nav_suite_predict_step(nav_suite_t* s, const ins_measurements_t* m, float* phi_out);

    /** @brief Fusion half of nav_suite_update(): ins_correct_step() on the
     *  ins sub-filter, then everything else nav_suite_update() does
     *  (vertical datum alignment, the local-GNSS offset filter, mode
     *  logging, and the ARS/AHRS/baro_alt bootstrap-or-update calls with
     *  their cross-check diagnostics), using the measurement handed off by
     *  the matching nav_suite_predict_step() call.
     *
     *  @param[in,out] s The suite instance. */
    void nav_suite_correct_step(nav_suite_t* s);

    /** @brief Current solution mode (see nav_suite_mode_t).
     *
     *  FULL: INS is ready and had absolute position aiding recently.
     *
     *  COASTING: INS is still ready but dead-reckoning on IMU only:
     *  usable, position uncertainty growing.
     *
     *  ATTITUDE_ONLY: INS's position solution is gone
     *  (coasting window exceeded, e.g. long tunnel) or INS is
     *  not initialized, but at least one AHRS instance
     *  provides an attitude: use nav_suite_get_rpy(). INS recovers to
     *  FULL automatically with the first usable fix (re-acquisition).
     *
     *  @param[in] s The suite instance.
     *  @return The current mode (nav_suite_mode_t). */
    nav_suite_mode_t nav_suite_get_mode(const nav_suite_t* s);

    /** @brief Report whether the ARS/AHRS were given a zero-rotation
     *  update on the last nav_suite_update() call (REQ-SUITE-009): the
     *  caller's explicit m->zero_rotation_update flag OR'd with INS's
     *  own auto-ZUPT/ZARU detector (ins_auto_zupt_active). Diagnostic/
     *  telemetry use.
     *  @param[in] s The suite instance.
     *  @return true if a zero-rotation update was applied last epoch. */
    bool nav_suite_get_zaru_active(const nav_suite_t* s);

    /** @brief Report whether the vertical channel (baro_alt) was given a
     *  zero-velocity update on the last nav_suite_update() call
     *  (REQ-SUITE-015). False whenever the vertical filter was not running
     *  that epoch, so this reports what actually reached baro_alt, not just
     *  the trigger.
     *  @param[in] s The suite instance.
     *  @return true if a zero-velocity update reached baro_alt last epoch. */
    bool nav_suite_get_vertical_zupt_active(const nav_suite_t* s);

    /** @brief Enable/disable every automatic ZUPT/ZARU source in the suite
     *  at runtime: INS's own detector (ins_set_auto_zupt_disable) and both
     *  ARS/AHRS velocity-blind fallbacks (ahrs_set_auto_zaru_disable).
     *  baro_alt has no detector of its own - its vertical zero-velocity
     *  update (nav_suite_get_vertical_zupt_active) is entirely driven by
     *  these three, so disabling them here also stops it.
     *
     *  Safety-relevant: a platform that is legitimately still by every gate but
     *  must never receive a stillness update (e.g. a multicopter holding
     *  position mid-flight) can disable this for exactly that window. Also
     *  updates ars_cfg/ahrs_cfg so the setting survives a later ARS/AHRS
     *  re-bootstrap. Does not affect an externally-triggered update
     *  (ins_measurements_t.zero_velocity_update / zero_rotation_update).
     *  @param[in,out] s The suite instance.
     *  @param[in] disable true -> no automatic source can arm until called
     *      again with false. */
    void nav_suite_set_auto_zupt_zaru_disable(nav_suite_t* s, bool disable);

    /** @brief Roll/pitch/yaw from the best available source: INS if
     *  ready, else the magnetometer AHRS, else the ARS.
     *
     *  On the ARS fallback the reported yaw is the last 3D heading
     *  carried forward by the ARS's own yaw change where one is available
     *  (REQ-SUITE-022, the same heading INS is re-initialized from), and
     *  the ARS's raw, only-relative yaw where it is not, e.g. before INS
     *  has ever run. nav_suite_get_rpy_ars() always reports the raw one.
     *  @param[in] s The suite instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad].
     *  @return false if no source is available. */
    bool nav_suite_get_rpy(const nav_suite_t* s, float* roll_rad, float* pitch_rad, float* yaw_rad);

    /** @brief Roll/pitch/yaw of the INS filter.
     *  @param[in] s The suite instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad].
     *  @return false while the filter is not initialized/healthy. */
    bool nav_suite_get_rpy_ins(const nav_suite_t* s, float* roll_rad, float* pitch_rad,
                               float* yaw_rad);

    /** @brief Roll/pitch/yaw of the roll/pitch (free yaw / gyro compassing) filter.
     *  @param[in] s The suite instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad] (relative, drifting heading).
     *  @return false while the filter is not initialized/healthy. */
    bool nav_suite_get_rpy_ars(const nav_suite_t* s, float* roll_rad, float* pitch_rad,
                               float* yaw_rad);

    /** @brief Roll/pitch/yaw of the magnetometer-aided filter.
     *  @param[in] s The suite instance.
     *  @param[out] roll_rad Roll [rad].
     *  @param[out] pitch_rad Pitch [rad].
     *  @param[out] yaw_rad Yaw [rad].
     *  @return false while the filter is not initialized/healthy (or no
     *  magnetometer sample has been seen yet). */
    bool nav_suite_get_rpy_ahrs(const nav_suite_t* s, float* roll_rad, float* pitch_rad,
                                float* yaw_rad);

    /** @brief Height and vertical velocity (both positive up) from the
     *  baro/accelerometer vertical filter, expressed in the common
     *  vertical datum (the ins NED origin, the mission start point
     *  while ins has never been initialized). The filter starts with
     *  the first valid barometer sample. Either output pointer may be
     *  NULL.
     *  @param[in] s The suite instance.
     *  @param[out] h_m Height [m], positive up.
     *  @param[out] v_mps Vertical velocity [m/s], positive up.
     *  @return false while no barometer sample has been seen or the
     *  filter is not healthy. */
    bool nav_suite_get_baro_alt(const nav_suite_t* s, float* h_m, float* v_mps);

    /** @brief Height above the NED origin from the best available source:
     *  ins under fresh position aiding (FULL), else the baro filter
     *  (which keeps measuring the vertical channel during outages), else
     *  a coasting INS. All sources share the same vertical datum, so
     *  the output is continuous across source changes.
     *  @param[in] s The suite instance.
     *  @param[out] h_m Height above the NED origin [m], positive up.
     *  @return false if no source is available. */
    bool nav_suite_get_height(const nav_suite_t* s, float* h_m);

    /** @brief Absolute (ellipsoid) height: ins when ready under fresh
     *  aiding, else the local height reference plus the estimated
     *  offset (requires that local-height/GNSS pairs have been seen at
     *  some point), else a coasting ins.
     *
     *  Unlike nav_suite_get_height(), this is not always available: it
     *  needs GNSS now, or GNSS earlier alongside a local height source
     *  to have estimated the offset. Check the return value.
     *
     *  @param[in] s The suite instance.
     *  @param[out] h_ell_m Absolute (ellipsoid) height [m].
     *  @return false if no source is available. */
    bool nav_suite_get_height_ellipsoid(const nav_suite_t* s, float* h_ell_m);

    /** @brief Offset that maps an external local-position frame (e.g. a
     *  lighthouse system) onto the suite's local datum.
     *
     *  The wrapper solves in a frame anchored at the datum origin (local
     *  height 0 at the start point); a local-position system reports in
     *  its own frame, whose origin is arbitrary. This offset is that
     *  system's coordinates of the datum origin, so a caller converts
     *  between the two frames with
     *
     *      external_coord = local_solution + offset
     *      local_solution = external_coord - offset
     *
     *  Constant once latched; frozen at the ins bootstrap. Only the
     *  vertical component depends on the barometer (it aligns the datum to
     *  the baro height that was already established); the horizontal
     *  component is the external system's report at first sample.
     *
     *  @param[in] s The suite instance.
     *  @param[out] offset_ned External-frame coords of the datum origin,
     *                         NED [m].
     *  @return false until a local NED position sample has been seen. */
    bool nav_suite_get_local_pos_offset(const nav_suite_t* s, float offset_ned[3]);

    /** @brief Convert a local NED position (relative to the datum origin)
     *  to WGS84 geodetic coordinates.
     *
     *  The counterpart of nav_suite_get_local_pos_offset() for the GNSS side:
     *  it relates the local n-frame solution to WGS84, so a caller can turn the
     *  current local position into lat/lon/height (see
     *  nav_suite_wgs84_to_local for the inverse). Available only once real GNSS
     *  has anchored the n-frame origin to WGS84 - without that the origin is
     *  only the prescribed init position and the result would be fictitious.
     *
     *  @param[in] s The suite instance.
     *  @param[in] ned Local NED position relative to the datum origin [m].
     *  @param[out] lat_rad Geodetic latitude [rad].
     *  @param[out] lon_rad Geodetic longitude [rad].
     *  @param[out] h_m Ellipsoid height [m].
     *  @return false if the n-frame is not GNSS-anchored yet. */
    bool nav_suite_local_to_wgs84(const nav_suite_t* s, const float ned[3], double* lat_rad,
                                  double* lon_rad, double* h_m);

    /** @brief Convert WGS84 geodetic coordinates to a local NED position
     *  (relative to the datum origin). Inverse of nav_suite_local_to_wgs84();
     *  e.g. to turn a WGS84 waypoint into the local frame a controller
     *  flies in. Same GNSS-anchoring precondition.
     *
     *  Note that the n-frame origin is not fixed for the whole session: a
     *  re-bootstrap (health failure, time jump, or a GNSS quality-loss re-arm,
     *  REQ-NAV-052) re-anchors it on the new bootstrap fix. Both conversions
     *  read the current origin, so they stay correct across it -- but a local
     *  NED coordinate a caller cached earlier does not. Keep waypoints in WGS84
     *  and convert on use.
     *
     *  @param[in] s The suite instance.
     *  @param[in] lat_rad Geodetic latitude [rad].
     *  @param[in] lon_rad Geodetic longitude [rad].
     *  @param[in] h_m Ellipsoid height [m].
     *  @param[out] ned Local NED position relative to the datum origin [m].
     *  @return false if the n-frame is not GNSS-anchored yet. */
    bool nav_suite_wgs84_to_local(const nav_suite_t* s, double lat_rad, double lon_rad, double h_m,
                                  float ned[3]);

#ifdef __cplusplus
}
#endif

#endif /* NAV_SUITE_H */
/** @} */
