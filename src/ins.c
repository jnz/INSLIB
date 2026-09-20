/** @file ins.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief 15/18-state error-state Kalman filter for 3D navigation.
 *
 * See ins.h for the state-vector layout and overall design.
 */

#include <math.h>
#include <string.h>
#include <assert.h>
#include <stddef.h>
#include <float.h>
#include <stdio.h>

#include "ins.h"
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

/* Compile-time maximum error-state size.
   The ACTIVE size is the runtime f->n: INS_UNKNOWNS (15) or
   INS_UNKNOWNS_MAG (18 with estimate_mag_bias).
   Loops are bounded by f->n <= INS_UNKNOWNS_MAX, so
   execution time stays bounded (REQ-SYS-004). */
#define INS_US_PER_SEC (1000000LL)
#define INS_US_PER_MS  (1000LL)

/* KFCore sizes the scratch matrices of its Kalman backend from the
   compile-time limits KALMAN_MAX_STATE_SIZE / KALMAN_MAX_NOISE_SIZE
   (KFCore/c/kalman_udu.c). They default to 32, which costs several kB of
   stack per predict step, so the build lowers them to what INSLIB
   actually uses. Too small a value would overflow the scratchpads inside
   kalman_udu_predict(), so turn it into a build error here rather than a
   silent out-of-bounds write. Only checked when the build defines them,
   otherwise KFCore's own (larger) defaults apply. */
#ifdef KALMAN_MAX_STATE_SIZE
_Static_assert(KALMAN_MAX_STATE_SIZE >= INS_UNKNOWNS_MAX,
               "KALMAN_MAX_STATE_SIZE too small for INS_UNKNOWNS_MAX");
#endif
#ifdef KALMAN_MAX_NOISE_SIZE
_Static_assert(KALMAN_MAX_NOISE_SIZE >= INS_NOISE_COLS_MAX,
               "KALMAN_MAX_NOISE_SIZE too small for INS_NOISE_COLS_MAX");
#endif

/* Guard against tiny/negative variances after UDU updates. */
#define INS_MIN_VARIANCE (1e-12f)

/* Diagnostics: GNSS outage duration before the first LOG_WARN, and the
 * repeat interval while the outage persists. */
#define INS_LOG_GNSS_OUTAGE_WARN_SEC   (10.0f)
#define INS_LOG_GNSS_OUTAGE_REPEAT_SEC (60.0f)
#define INS_LOG_INVALID_REPEAT_SEC     (30.0f)

/* Diagnostics: a GNSS position correction this large in one epoch is
   worth a WARN, and how often to repeat it while it lasts. A stretch of
   multipath, or a re-acquisition converging back onto the fixes, produces one
   oversized residual per epoch for as long as it takes, all reporting the
   same condition. diag.max_gnss_pos_residual_m keeps the extreme value
   regardless of what the log prints. */
#define INS_LOG_LARGE_POS_JUMP_M     (20.0f)
#define INS_LOG_LARGE_POS_REPEAT_SEC (30.0f)

/* Diagnostics: yaw is only observable through mag/absolute-yaw/
   automotive-course aiding. Warn if yaw is drifting too much. */
#define INS_LOG_YAW_STDDEV_WARN_DEG    (10.0f)
#define INS_LOG_YAW_AID_GAP_WARN_SEC   (2.0f)
#define INS_LOG_YAW_AID_GAP_REPEAT_SEC (10.0f)

/* Diagnostics: a raw gyro sample bit-identical to the previous epoch's,
   repeated this many times in a row, indicates a frozen/stuck sensor bus
   replaying the last register value. */
#define INS_LOG_STUCK_GYR_EPOCHS (50)
#define INS_LOG_STUCK_REPEAT_SEC (240.0f)

/* Diagnostics: the magnetometer field-strength check
   (mag_field_expected_uT) tripping ONCE is a transient disturbance (motor
   current, ferrous structure), tripping this many epochs in a row is a
   persistent one, e.g. a bad mounting position. */
#define INS_LOG_MAG_DISTURB_EPOCHS     (20)
#define INS_LOG_MAG_DISTURB_REPEAT_SEC (240.0f)

/* Runaway detectors: tumbling-window rate checks shared by the
   yaw-stddev/gyro-bias/accel-bias watchdogs. A metric growing this much
   within one window counts as runaway. */
#define INS_LOG_RUNAWAY_WINDOW_SEC    (10.0f)
#define INS_LOG_YAW_RUNAWAY_DEG       (5.0f) /* yaw stddev growth per window */
#define INS_LOG_GYR_BIAS_RUNAWAY_DPS  (0.5f) /* |gyro bias| growth per window */
#define INS_LOG_ACC_BIAS_RUNAWAY_MPS2 (0.2f) /* |accel bias| growth per window */

/* Bias sanity bounds: absolute |bias| beyond these points at filter
   divergence or a bad sensor rather than a legitimate turn-on bias.
   Generous, to avoid false positives on a poor but healthy sensor. */
#define INS_LOG_GYR_BIAS_SANITY_DPS    (10.0f)
#define INS_LOG_ACC_BIAS_SANITY_MPS2   (2.0f)
#define INS_LOG_BIAS_SANITY_REPEAT_SEC (120.0f)

/* Post-(re)init warm-up before the bias-runaway/sanity checks above are
   evaluated: the bias estimate legitimately moves fast while converging from
   its loose initial covariance and would otherwise trip them on the first
   window of nearly every run. Same rationale as
   ahrs_config_t.restart_warmup_sec (REQ-AHRS-023), but fixed. */
#define INS_LOG_BIAS_RUNAWAY_WARMUP_SEC (10.0f)

/* Initial bias-prior consistency check (REQ-NAV-050). While the auto-ZUPT
   detector reports stillness, the averaged RAW gyro is the gyro bias (plus
   earth rate) and the averaged raw ||f|| - g is the accelerometer bias
   projected onto gravity, so both can be held against the configured 1-sigma
   init priors. Raw, not bias-corrected: the test targets the prior the filter
   STARTED from.
     - SIGMA_FACTOR: sigma the averaged measurement has to exceed before the
       initial value counts as potentially mis-specified.
     - MIN_DWELL_SEC / MIN_SAMPLES: window long and dense enough to average the
       white sensor noise well below the threshold. */
#define INS_BIAS_PRIOR_SIGMA_FACTOR   (4.0f)
#define INS_BIAS_PRIOR_MIN_DWELL_SEC  (10.0f)
#define INS_BIAS_PRIOR_MIN_SAMPLES    (200u)
#define INS_LOG_BIAS_PRIOR_REPEAT_SEC (120.0f)

/* Throttle for the "GNSS is fusable but not good enough to enter the 3D
   solution" warning (REQ-NAV-051). Without it a receiver whose reported
   accuracy sits permanently between the fusion gate and the stricter entry
   gate would look like a filter that silently never starts. */
#define INS_LOG_ENTRY_GATE_REPEAT_SEC (30.0f)

/* Chi-square outlier threshold for the robust UDU update (per scalar
   measurement, Mahalanobis distance squared, 1 DOF). One value shared by
   every sensor: this default or chi2inv(1-alpha, 1) when configured
   (REQ-NAV-046). 0 disables the test (chi2_disable, REQ-NAV-035). */
#define INS_DEFAULT_CHI2_GATE (10.0f)

/* Automotive mode (opt.automotive_mode): yaw from GNSS course over ground,
   needs a minimum speed for a reasonable yaw measurement.
   Override: opt.automotive_min_speed_mps (0 -> default).
   INS_AUTOMOTIVE_MIN_YAW_STDDEV is the heading floor: fused yaw stddev =
   max(sigma_v/speed, floor). Override: opt.automotive_min_yaw_stddev, raise it
   where the model is looser than a car's (e.g. wind drift on an aircraft). */
#define INS_AUTOMOTIVE_MIN_SPEED_MPS  (2.0f)
#define INS_AUTOMOTIVE_MIN_YAW_STDDEV DEG2RAD(5.0f)

/* Non-holonomic lateral constraint (REQ-NAV-077).
   The interval is not a performance knob: the residual's correlation time is
   on the order of a minute, so fusing faster adds confidence without adding
   information. The yaw-rate ceiling is where the scatter starts growing. */
#define INS_NHC_STDDEV_MPS       (0.1f)
#define INS_NHC_MAX_YAW_RATE     DEG2RAD(3.0f)
#define INS_NHC_AFTER_SEC        (5.0f)
#define INS_NHC_MIN_INTERVAL_SEC (1.0f)

/* Magnetometer field-strength disturbance gate: relative |B| deviation
   from the WMM total field beyond which the sample is downweighted
   (used when opt.mag_field_tolerance == 0). */
#define INS_DEFAULT_MAG_FIELD_TOL INS_DEFAULT_MAG_FIELD_TOLERANCE

/* Magnetometer hard-iron bias states (18-state mode): initial stddev and
   random walk defaults (unit = the mag/model unit, uT).

   The initial sigma is a prior on how much hard iron there is to find. Ten uT
   is deliberately loose: ten to twenty is ordinary once a unit is installed in
   a vehicle, and a tighter prior would be a claim about an installation the
   library knows nothing about.

   The random walk is better read as how long the estimate keeps LISTENING
   after it has settled than as a physical drift rate. Hard iron does not
   wander continuously; it steps when ferrous mass moves, and drifts with
   temperature otherwise. What the number decides is whether such a step is
   still followed. Against a per-axis sigma of a few uT fused at ~1 Hz, a
   random walk of q settles the state near (q^2*dt*R)^(1/4), and the drift it
   admits over a time T is q*sqrt(T). At 0.129 that is 1 uT per minute, which
   keeps the state correctable; the 0.01 this used to be settles near 0.14 uT
   and then needs a quarter of an hour to admit one uT of genuine change, so a
   payload that moves or a motor that switches on is a change the filter no
   longer follows. */
#define INS_DEFAULT_MAG_BIAS_STDDEV_UT (10.0f)
#define INS_DEFAULT_MAG_BIAS_RW_UT     (0.129f) /* 1 uT per minute */

/* Magnetometer fusion rate limit used when opt.magnetometer_min_delay_ms is
   left at 0. The magnetometer is a long-term heading anchor, not a per-epoch
   yaw sensor: a few uT of noise against a horizontal field of ~20 uT is
   already several degrees of yaw, far more than a MEMS gyro drifts in one
   second. Fusing at full rate therefore buys almost no heading accuracy while
   handing the filter every motor-current disturbance at that rate, and those
   are time-correlated whereas the update treats each sample as independent. A
   negative value disables the rate limit. */
#define INS_DEFAULT_MAG_MIN_DELAY_MS (1000)

/* Reject history matches further away than this from the requested
   time-of-validity. */
#define INS_HISTORY_MATCH_TOL_US (50 * INS_US_PER_MS)

/* Max. number of measurements per single ins_fuse() (e.g. GNSS pos + vel). */
#define INS_FUSE_MAX_MEAS (6)

/* Refresh distance for the slowly-varying Earth-dependent quantities
   (gravity, curvature terms): they change by ~d/R_earth,
   i.e. ~1.6e-5 relative per 100 m of travel. */
#define INS_EARTH_REFRESH_DIST_M (100.0f)

/* The initial local n-frame is not supposed to jump around. When the carry of
   REQ-NAV-062 is refused, the bootstrap fix only defines a fresh origin if it
   cannot be the same platform, i.e. reaching it from the last known position
   would have required more than INS_ORIGIN_CARRY_MAX_SPEED_MPS over the whole
   outage. INS_ORIGIN_CARRY_MIN_TRAVEL_M is the floor of that budget. */
#define INS_ORIGIN_CARRY_MAX_SPEED_MPS (300.0)
#define INS_ORIGIN_CARRY_MIN_TRAVEL_M  (2000.0)

/* Auto-init (initial alignment) defaults (0 -> default). */
#define INS_AUTOINIT_DEFAULT_WINDOW_SEC (0.1f) /* leveling window */
#define INS_AUTOINIT_DEFAULT_STATIC_GYR INS_DEFAULT_STATIC_GYR_RPS
#define INS_AUTOINIT_DEFAULT_STATIC_ACC INS_DEFAULT_STATIC_ACC_MPS2
/* Yaw stddev when no heading source is available (~180 deg -> "unknown").
   Shared by the bootstrap (no heading source at all) and re-acquisition
   after a frozen coasting window (REQ-NAV-059), which is the same state:
   "nothing in the filter knows where the platform points". */
#define INS_YAW_UNKNOWN_STDDEV (3.14159265f)
/* Acceptance band for absolute heading inputs (REQ-NAV-060): the yaw
   measurement and the attitude hint's yaw. Any single-turn convention is
   accepted ([-pi, pi], [0, 2pi), ...) and normalized to [-pi, pi] at the API
   boundary. Beyond one full turn the value is a unit or unwrapping error
   rather than a convention difference, and wrapping it would yield a
   plausible-looking but wrong heading -> dropped and counted instead. */
#define INS_YAW_INPUT_MAX_RAD (2.0f * (float)M_PI + 1e-3f)
/* Floor on the roll/pitch initial stddev (REQ-NAV-047) when the leveling
 * window is not quasi-static: the accelerometer-leveling assumption
 * (specific force ~ gravity) is violated by motion, so the bootstrap must
 * not report a falsely tight roll/pitch covariance in that case. */
#define INS_AUTOINIT_DEFAULT_MOVING_RPY_STDDEV DEG2RAD(15.0f)

/* Default IMU noise, used only when the caller leaves the corresponding
 * field at 0/unset. Conservative placeholders for a low-cost consumer MEMS
 * IMU, enough for a stable filter.
 * Replace with the sensor's actual noise figures when available. */
#define INS_DEFAULT_ACC_VRW_MPS2_SQRTHZ (350e-6f * INS_GRAVITY_NOMINAL)
#define INS_DEFAULT_GYR_ARW_PSD         (INS_DEFAULT_GYR_ARW_RPS_SQRTHZ * INS_DEFAULT_GYR_ARW_RPS_SQRTHZ)
#define INS_DEFAULT_ACC_VRW_PSD         (INS_DEFAULT_ACC_VRW_MPS2_SQRTHZ * INS_DEFAULT_ACC_VRW_MPS2_SQRTHZ)

/* Accel bias random-walk default [m/s^2/sqrt(s)] */
#define INS_DEFAULT_ACC_BIAS_RW_MPS2_SQRTS (3e-5f)

/* Initial-state and process-noise defaults (REQ-NAV-049): a caller who leaves
 * one of these fields at 0 still gets a stable, correctable filter, since a
 * covariance seeded at exactly 0 would claim unwarranted certainty. */
#define INS_DEFAULT_POS_INIT_STDDEV_M         (10.0f)
#define INS_DEFAULT_VEL_INIT_STDDEV_MPS       (1.0f)
#define INS_DEFAULT_RPY_INIT_STDDEV_RAD       DEG2RAD(5.0f)
#define INS_DEFAULT_ACC_BIAS_INIT_STDDEV_MPS2 (0.03f)
#define INS_DEFAULT_GYR_BIAS_INIT_STDDEV_RPS  DEG2RAD(1.0f)
#define INS_DEFAULT_POS_PRED_STDDEV_MPS       (0.01f)
#define INS_DEFAULT_VEL_PRED_STDDEV_MPS2      (0.0005f)
/* Attitude margin [rad/sqrt(s)], sized for a MEMS IMU on a non-rigid mount,
 * where the model error is gyro scale-factor and cross-axis error under large
 * oscillating rates, well above the sensor's own random walk. At the sensor
 * figure the filter is falsely confident in attitude and the tilt error lands
 * in the accelerometer bias instead. */
#define INS_DEFAULT_RPY_PRED_STDDEV_RPS (0.001f)
/* Zero-rotation measurement-noise defaults (REQ-NAV-049). This is
 * the standard deviation of the virtual rotation rate measurement (1-sigma). */
#define INS_DEFAULT_ZERO_ROT_STDDEV_RPS DEG2RAD(0.5f)

/* Covariance-prediction cadence [s], used when opt.kalman_update_dt_sec is 0.
 * The strapdown runs at the full IMU rate, covariance propagation runs at this rate.
 * (Wendel, 2nd ed., ch. 8.2.1: "typically 10 Hz"). 20 Hz keeps a safety
 * margin over that figure, overridable via the option field. */
#define INS_DEFAULT_KALMAN_UPDATE_DT_SEC (1.0f / 20.0f)

/* Automatic ZUPT/ZARU detector defaults (used when the corresponding option
 * field is 0). The configurable STATIC_GYR/STATIC_ACC fields are loose
 * magnitude sanity bounds, not the primary stillness criterion - that is the
 * window's sample variance (INS_DEFAULT_STATIC_*_STDDEV_*). MAX_VEL
 * additionally requires a small GNSS velocity where one is available. */
#define INS_AUTOZUPT_DEFAULT_STATIC_GYR         INS_DEFAULT_STATIC_GYR_BOUND_RPS
#define INS_AUTOZUPT_DEFAULT_STATIC_ACC         INS_DEFAULT_STATIC_ACC_BOUND_MPS2
#define INS_AUTOZUPT_DEFAULT_MAX_VEL_MPS        (1.5f)
#define INS_AUTOZUPT_DEFAULT_MAX_VEL_STDDEV_MPS (0.5f)
#define INS_AUTOZUPT_DEFAULT_DWELL_SEC          INS_DEFAULT_STATIC_DWELL_SEC
#define INS_AUTOZUPT_DEFAULT_MIN_INTERVAL_SEC   (0.2f)
/* How long a cached external (GNSS) velocity observation stays trusted before
 * the velocity gate is treated as inapplicable (a couple of GNSS epochs). */
#define INS_AUTOZUPT_EXT_VEL_MAX_AGE_SEC (2.0f)

/* Default max. IMU-only coasting time without absolute position aiding
 * (used when opt.max_deadreckoning_sec == 0): roughly how long a
 * consumer-MEMS strapdown position stays usable (tunnel passage). */
#define INS_DEFAULT_MAX_DEADRECKONING_SEC (10.0f)

/* Max. time in seconds to perform a filter prediction step and also
 * the max. time to perform an IMU dead-reckoning step. */
#define INS_DEFAULT_MAX_PREDICTION_TIME_SEC (0.5f)

/* Absolute-speed aiding (REQ-NAV-068). The per-sample default is the
 * uniform-quantization 1-sigma of a 1 km/h resolution reading (1/sqrt(12)
 * km/h), as delivered by an OBD-II PID 0x0D speed. The relative default covers
 * the residual scale error of the vehicle speed signal left after
 * speed_scale. The minimum speed is where v_hat stops being a meaningful
 * direction: walking pace. */
#define INS_DEFAULT_SPEED_STDDEV_MPS (0.080f)
#define INS_DEFAULT_SPEED_STDDEV_REL (0.03f)
#define INS_DEFAULT_SPEED_MIN_MPS    (1.0f)

/* Thresholds when to accept GNSS measurements (REQ-NAV-007). Set to the
   accuracy caps below on purpose: a fix that honestly reports a large
   uncertainty carries little information but not wrong information, and the
   cap bounds what reaches the update anyway. Whether the fix STREAM is healthy
   enough for a 3D solution is a separate question with much tighter thresholds
   (REQ-NAV-051/052 below). A caller wanting a hard accuracy gate sets
   gnss_max_* below the caps. */
#define INS_DEFAULT_GNSS_MAX_HPOS_STDDEV_M   INS_DEFAULT_GNSS_POS_STDDEV_CAP_HOR_M
#define INS_DEFAULT_GNSS_MAX_VPOS_STDDEV_M   INS_DEFAULT_GNSS_POS_STDDEV_CAP_VER_M
#define INS_DEFAULT_GNSS_MAX_HVEL_STDDEV_MPS INS_DEFAULT_GNSS_VEL_STDDEV_CAP_HOR_MPS
#define INS_DEFAULT_GNSS_MAX_VVEL_STDDEV_MPS INS_DEFAULT_GNSS_VEL_STDDEV_CAP_VER_MPS

/* GNSS covariance floors (REQ-NAV-038, REQ-NAV-043): the smallest per-axis
   1-sigma the fusion will believe, whatever the receiver reports. Sized at
   what a standalone multi-GNSS solution can physically deliver rather than at
   what the receiver claims: below these the fix drives the solution instead of
   constraining it. The vertical position floor is the loosest of the four
   because the satellite geometry above a ground platform is one-sided. A
   carrier-phase (RTK) installation has to lower these explicitly. */
#define INS_DEFAULT_GNSS_POS_STDDEV_FLOOR_HOR_M   (0.5f)
#define INS_DEFAULT_GNSS_POS_STDDEV_FLOOR_VER_M   (1.0f)
#define INS_DEFAULT_GNSS_VEL_STDDEV_FLOOR_HOR_MPS (0.3f)
#define INS_DEFAULT_GNSS_VEL_STDDEV_FLOOR_VER_MPS (0.5f)

/* GNSS covariance caps (REQ-NAV-071): the largest per-axis 1-sigma that
   reaches the fusion. Above these the fix carries no usable information
   anyway, while the variance itself keeps growing without bound and starts
   costing precision in the 32-bit hot path. */
#define INS_DEFAULT_GNSS_POS_STDDEV_CAP_HOR_M   (120.0f)
#define INS_DEFAULT_GNSS_POS_STDDEV_CAP_VER_M   (120.0f)
#define INS_DEFAULT_GNSS_VEL_STDDEV_CAP_HOR_MPS (60.0f)
#define INS_DEFAULT_GNSS_VEL_STDDEV_CAP_VER_MPS (60.0f)

/* Manoeuvre-dependent GNSS velocity noise (REQ-NAV-073), in [m/s] of extra
   velocity 1-sigma per [m/s^2] of n-frame acceleration.

   A timing quantity rather than a tuning one: a GNSS velocity is formed over a
   measurement interval T while the state it is fused against is instantaneous,
   so under an acceleration a the two differ by about a*T/2. The horizontal
   default is that half-interval for T = 400 ms, four times the rate this
   filter paces GNSS fusion to (REQ-NAV-074). The margin is deliberate: T is a
   property of the RECEIVER's velocity algorithm, not of its output rate, and
   the filter cannot observe it. These are fixed constants, NOT derived from
   the observed fix rate. The vertical default keeps a further margin for the
   one-sided satellite geometry.

   Raise both for a receiver whose velocity is time-differenced carrier phase
   over a long output interval, lower them for instantaneous Doppler. The term
   reads the INSTANTANEOUS n-frame acceleration, so airframe vibration leaks in
   at scale times the vibration amplitude. */
#define INS_DEFAULT_GNSS_VEL_NOISE_ACC_SCALE_HOR (0.20f)
#define INS_DEFAULT_GNSS_VEL_NOISE_ACC_SCALE_VER (0.30f)

/* Averaging window for the acceleration that term reads (REQ-NAV-075) [s].
   Without it a manoeuvre that has just ended is unpriced on the very fix that
   still carries its error, while a fix arriving mid-vibration-cycle is priced
   for a swing that averages to nothing. */
#define INS_DEFAULT_GNSS_VEL_NOISE_ACC_WINDOW_SEC (0.2f)

/* Decay time constant of the reported-accuracy envelope (REQ-NAV-072).
   Long enough to cover the settling of a re-acquired fix (multipath fading
   out, a filter inside the receiver re-converging), short enough that a
   single degraded epoch is forgotten well within a typical outage. */
#define INS_DEFAULT_GNSS_ACC_ENVELOPE_TAU_SEC (5.0f)

/* GNSS-stability dwell before entering the full 3D filter mode (REQ-NAV-045).
   Default window and the minimum sustained fix rate the window must carry.
   A gap longer than INS_GNSS_INIT_MAX_GAP_SEC between usable fixes breaks the
   run (an outage, not a stable stream). */
#define INS_DEFAULT_GNSS_INIT_DWELL_SEC (5.0f)
#define INS_GNSS_INIT_MIN_RATE_HZ       (1.0f)
#define INS_GNSS_INIT_MAX_GAP_SEC       (2.0f)

/* GNSS position decimation (REQ-NAV-063): on every Nth epoch that offers a
   usable position AND a usable velocity, the position is fused alone, on the
   other N-1 the velocity is. Thinning the position targets the block whose
   error is correlated over minutes (multipath, residual ionosphere, receiver
   smoothing) while the Doppler velocity is comparatively white.

   OFF by default: the rate limit below addresses the same over-fusing without
   withholding a block, and doing both thins the position twice. */
#define INS_DEFAULT_GNSS_POS_DECIMATION (1)

/* GNSS fusion rate limit used when opt.gnss_min_delay_ms is left at 0
   (REQ-NAV-074).

   A GNSS error does not decorrelate anywhere near as fast as a modern receiver
   can output fixes: multipath, residual ionosphere and the receiver's own
   smoothing persist over many seconds. Fusing at 20 Hz therefore presents the
   same information twice rather than twice the information, and the filter
   answers by driving its covariance below what the measurements support. */
#define INS_DEFAULT_GNSS_MIN_DELAY_MS (100)

/* Mode-transition quality gates around the 3D solution (REQ-NAV-051,
   REQ-NAV-052), deliberately separate from the fusion gates above: entering
   demands a clearly better fix than merely fusing one, leaving demands a
   clearly worse one. Together with gnss_init_dwell_sec / gnss_stop_dwell_sec
   this is the hysteresis of the transition. The velocity numbers carry the
   decision: above ~0.4 m/s 1-sigma a GNSS velocity no longer constrains the
   accelerometer bias / attitude observability the 3D solution relies on. */
#define INS_DEFAULT_GNSS_START_MAX_HPOS_STDDEV_M   (2.0f)
#define INS_DEFAULT_GNSS_START_MAX_VPOS_STDDEV_M   (3.0f)
#define INS_DEFAULT_GNSS_START_MAX_HVEL_STDDEV_MPS (0.25f)
#define INS_DEFAULT_GNSS_START_MAX_VVEL_STDDEV_MPS (0.30f)
#define INS_DEFAULT_GNSS_STOP_MAX_HPOS_STDDEV_M    (5.0f)
#define INS_DEFAULT_GNSS_STOP_MAX_VPOS_STDDEV_M    (7.0f)
#define INS_DEFAULT_GNSS_STOP_MAX_HVEL_STDDEV_MPS  (0.4f)
#define INS_DEFAULT_GNSS_STOP_MAX_VVEL_STDDEV_MPS  (0.5f)
#define INS_DEFAULT_GNSS_STOP_DWELL_SEC            (10.0f)
/* Most one epoch may add to the stop dwell [s]. Wide enough that a slow but
   continuous fix stream still fills the dwell at close to real time, small
   enough that an outage cannot (REQ-NAV-052). */
#define INS_GNSS_STOP_MAX_STEP_SEC (2.0f)

/* How much the carried IMU-bias 1-sigma is widened at the re-bootstrap
   after a quality-loss re-arm (REQ-NAV-061). The consumption site clamps
   the result to the cold-start prior, so this factor only decides how much
   BETTER than a cold start the carry is still allowed to claim. */
#define INS_BIAS_CARRY_STDDEV_INFLATION (3.0f)

/* Maximum age of a cached barometer sample used to anchor a height. A
   barometer that delivered once and then stopped must not latch a filter into
   a height source it has no sensor for (REQ-NAV-053), and the cached pressure
   must still describe the epoch it anchors: the datum anchor baro_h0_m at
   bootstrap, the vertical position itself at a re-acquisition (REQ-NAV-066). */
#define INS_BARO_ANCHOR_MAX_AGE_SEC (2.0f)

/* Barometric height aiding gap (REQ-NAV-058) before warning that the height
   channel selected at bootstrap has stopped being fed. Not a fallback trigger
   (REQ-NAV-053 fixes the source for the filter's lifetime), a diagnostic: the
   vertical position variance grows without an absolute reference while the
   horizontal solution can stay healthy. */
#define INS_LOG_BARO_HEIGHT_GAP_WARN_SEC   (10.0f)
#define INS_LOG_BARO_HEIGHT_GAP_REPEAT_SEC (30.0f)

/* ============================================================================
 * Timestamp helpers
 * ============================================================================
 */

static inline float time_diff_sec(ins_time_us_t later, ins_time_us_t earlier)
{
    return ((float)(later - earlier) * (1.0f / INS_US_PER_SEC));
}

static inline int time_diff_ms(ins_time_us_t later, ins_time_us_t earlier)
{
    return (int)((later - earlier) / INS_US_PER_MS);
}

static inline int iabs_int(int x) { return x < 0 ? -x : x; }

static int index_mod(int index, int array_length)
{
    int norm = index % array_length;
    if (norm < 0) norm += array_length;
    return norm;
}

/* ============================================================================
 * Tiny vector/matrix helpers (local, column-major)
 * ============================================================================
 */

static inline float vec3_norm(const float v[3])
{
    return SQRTF(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* Range comparison instead of isfinite(): MinGW's isfinite macro narrows
   its double argument to float internally, which trips -Wfloat-conversion
   (see also tools/insrcv.c's j_num). NaN fails both comparisons and an
   infinity fails one, so this rejects exactly the non-finite values. */
static inline bool isfinite_d(double x) { return x >= -DBL_MAX && x <= DBL_MAX; }

static inline bool vec3d_finite(const double v[3])
{
    return isfinite_d(v[0]) && isfinite_d(v[1]) && isfinite_d(v[2]);
}

static inline bool mat33_finite(const float m[9])
{
    int i;
    for (i = 0; i < 9; ++i)
    {
        if (!isfinite(m[i])) return false;
    }
    return true;
}

static inline void vec3_copy(const float src[3], float dst[3])
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

static inline void vec3_zero(float v[3]) { v[0] = v[1] = v[2] = 0.0f; }

/* y = R * x  (R is 3x3 column-major). Delegates to KFCore's matmul (linalg.h),
 * which is backed by sgemm_. */
static inline void mat3_mul_vec3(const float R[9], const float x[3], float y[3])
{
    matmul("N", "N", 3, 1, 3, 1.0f, R, x, 0.0f, y);
}

/* y = R' * x  (R is 3x3 column-major). */
static inline void mat3t_mul_vec3(const float R[9], const float x[3], float y[3])
{
    matmul("T", "N", 3, 1, 3, 1.0f, R, x, 0.0f, y);
}

static inline float qsquare(float x) { return x * x; }

/* True if M (column-major 3x3) carries any nonzero entry. Used to decide
 * "calibrate or pass through" (ins_imu_calibrate) and to report whether a
 * caller-supplied misalignment matrix is in effect (ins_log_effective_config).
 * Magnitude sum rather than element-wise == to stay -Wfloat-equal clean. */
static inline bool mat3_is_set(const float M[9])
{
    const float msum = fabsf(M[0]) + fabsf(M[1]) + fabsf(M[2]) + fabsf(M[3]) + fabsf(M[4]) +
                       fabsf(M[5]) + fabsf(M[6]) + fabsf(M[7]) + fabsf(M[8]);
    return msum > 0.0f;
}

/* IMU calibration (REQ-NAV-037): out = M * (in - bias), M column-major.
 * An all-zero M means "unset" and is treated as identity, so a zeroed
 * options struct is a no-op. */
/* @satisfies REQ-NAV-037 */
static inline void ins_imu_calibrate(const float M[9], const float bias[3], const float in[3],
                                     float out[3])
{
    const float c[3] = {in[0] - bias[0], in[1] - bias[1], in[2] - bias[2]};
    if (mat3_is_set(M)) { mat3_mul_vec3(M, c, out); }
    else
    {
        out[0] = c[0];
        out[1] = c[1];
        out[2] = c[2];
    }
}

/* Scale a symmetric 3x3 NED covariance in place by the congruence
 * Q' = S Q S with S = diag(s_hor, s_hor, s_ver): one stddev factor for the
 * horizontal (N,E) pair, one for the vertical (D) axis. Correlation
 * coefficients are preserved and the matrix stays PSD, which a per-axis clamp
 * of the diagonal would not guarantee. A non-positive factor means 1.0. */
static inline void ins_cov_scale_congruence(float Q[9], float s_hor, float s_ver)
{
    const float h = (s_hor > 0.0f) ? s_hor : 1.0f;
    const float v = (s_ver > 0.0f) ? s_ver : 1.0f;
    Q[0] *= h * h; /* NN (col-major 3x3) */
    Q[4] *= h * h; /* EE */
    Q[8] *= v * v; /* DD */
    Q[3] *= h * h; /* NE = (0,1) */
    Q[1] *= h * h; /* EN = (1,0), symmetric */
    Q[6] *= h * v; /* ND = (0,2) */
    Q[2] *= h * v; /* DN = (2,0), symmetric */
    Q[7] *= h * v; /* ED = (1,2) */
    Q[5] *= h * v; /* DE = (2,1), symmetric */
}

/* Cap a symmetric 3x3 NED covariance in place by the congruence Q' = S Q S
 * with S = diag(s_n, s_e, s_d), one independent shrink factor per axis
 * (s_i = 1 for an axis that is not above its cap). Unlike an isolated
 * diagonal clamp, this scales every correlation term by the product of its
 * two axes' factors, so the matrix stays PSD no matter how large the
 * off-diagonal terms were relative to the capped diagonal. */
static inline void ins_cov_cap_axis_congruence(float Q[9], float s_n, float s_e, float s_d)
{
    Q[0] *= s_n * s_n; /* NN (col-major 3x3) */
    Q[4] *= s_e * s_e; /* EE */
    Q[8] *= s_d * s_d; /* DD */
    Q[3] *= s_n * s_e; /* NE = (0,1) */
    Q[1] *= s_n * s_e; /* EN = (1,0), symmetric */
    Q[6] *= s_n * s_d; /* ND = (0,2) */
    Q[2] *= s_n * s_d; /* DN = (2,0), symmetric */
    Q[7] *= s_e * s_d; /* ED = (1,2) */
    Q[5] *= s_e * s_d; /* DE = (2,1), symmetric */
}

/* GNSS covariance conditioning (REQ-NAV-038, REQ-NAV-041, REQ-NAV-071):
 * scale, then floor, then cap a symmetric 3x3 NED covariance in place.
 * First Q *= scale^2 (correlations preserved). Then, if scale_height > 0, a
 * downweight of the vertical axis only, as the correlation-preserving
 * congruence diag(1,1,sh) Q diag(1,1,sh). Then the horizontal (N,E) and
 * vertical (D) diagonal variances are lifted to at least floor^2 and clamped
 * to at most cap^2. A non-positive scale/scale_height -> 1.0, a non-positive
 * floor/cap leaves that axis group untouched. scale_height applies to the
 * position block only (pass <= 0 for velocity).
 *
 * The floor raises Q[0]/Q[4]/Q[8] independently, which is safe: adding
 * independent variance to one axis is a sum of PSD matrices and stays PSD.
 * The cap instead shrinks each axis with the congruence above, scaling the
 * correlation terms down with it. An independent diagonal clamp would leave
 * them at whatever the uncapped diagonal supported, which can be far larger
 * than the new, smaller diagonal, most visibly right after a GNSS outage
 * when the reacquired fix reports a huge, strongly correlated covariance:
 * clamping the diagonal alone would then yield a non-positive-definite R
 * that decorrelate() rejects. ins_init resolves a cap below the floor up
 * front. */
/* @satisfies REQ-NAV-038 REQ-NAV-041 REQ-NAV-071 */
static inline void ins_condition_gnss_cov(float Q[9], float scale, float scale_height,
                                          float floor_hor, float floor_ver, float cap_hor,
                                          float cap_ver)
{
    const float s  = (scale > 0.0f) ? scale : 1.0f;
    const float s2 = s * s; /* 1.0 when unset -> harmless no-op multiply */
    int         k;
    for (k = 0; k < 9; ++k) { Q[k] *= s2; }
    ins_cov_scale_congruence(Q, 1.0f, scale_height);
    /* Only an axis that carries a covariance at all is floored. A
       non-positive diagonal entry states that the axis has no covariance, not
       that it has an extremely good one, and every consumer downstream reads
       it that way (ins_cov_is_valid, nav_suite's vertical offset filter). */
    if (floor_hor > 0.0f)
    {
        const float fv = floor_hor * floor_hor;
        if (Q[0] > 0.0f && Q[0] < fv) { Q[0] = fv; } /* NN */
        if (Q[4] > 0.0f && Q[4] < fv) { Q[4] = fv; } /* EE */
    }
    if (floor_ver > 0.0f)
    {
        const float fv = floor_ver * floor_ver;
        if (Q[8] > 0.0f && Q[8] < fv) { Q[8] = fv; } /* DD */
    }
    float s_n = 1.0f, s_e = 1.0f, s_d = 1.0f;
    bool  capped = false;
    if (cap_hor > 0.0f)
    {
        const float cv = cap_hor * cap_hor;
        if (Q[0] > cv)
        {
            s_n    = SQRTF(cv / Q[0]);
            capped = true;
        } /* NN */
        if (Q[4] > cv)
        {
            s_e    = SQRTF(cv / Q[4]);
            capped = true;
        } /* EE */
    }
    if (cap_ver > 0.0f)
    {
        const float cv = cap_ver * cap_ver;
        if (Q[8] > cv)
        {
            s_d    = SQRTF(cv / Q[8]);
            capped = true;
        } /* DD */
    }
    if (capped) { ins_cov_cap_axis_congruence(Q, s_n, s_e, s_d); }
}

/* Effective covariance-prediction period [s]: the configured value, or the
 * default (see INS_DEFAULT_KALMAN_UPDATE_DT_SEC) when left at 0. Also
 * paces the zero-velocity/zero-rotation fusions (one per prediction period). */
static inline float ins_kalman_dt(const ins_t* f)
{
    return (f->opt.kalman_update_dt_sec > 0.0f) ? f->opt.kalman_update_dt_sec
                                                : INS_DEFAULT_KALMAN_UPDATE_DT_SEC;
}

/* ============================================================================
 * Meta-data: recompute cached helpers (R_b_to_n, latlonh, gravity)
 * ============================================================================
 */

/* Recompute the slowly-varying Earth-dependent quantities: gravity and the
 * cached curvature terms for the incremental latlonh book-keeping. Called at
 * init, afterwards throttled to once every INS_EARTH_REFRESH_DIST_M meters of
 * travel (see ins_update_meta). */
static void ins_refresh_earth_params(ins_t* f)
{
    /* Curvature terms: evaluate the dNED->dlatlonh mapping for unit
       north/east steps (the mapping is component-wise). */
    const float unit_ne[3] = {1.0f, 1.0f, 0.0f};
    double      dllh[3];
    ins_dned_to_dlatlonh(unit_ne, f->latlonh[0], f->latlonh[2], dllh);
    f->meta_dlat_per_dN = dllh[0];
    f->meta_dlon_per_dE = dllh[1];
    f->meta_travel_m    = 0.0f;

    /* Refresh gravity at the new position if the user did not supply one
     * explicitly (i.e. if the init gravity was zero). Otherwise keep
     * the user-supplied value (e.g. from a WMM-style model). */
    if (vec3_norm(f->init.gravity_n) < 1e-4f)
    {
        ins_gravity_ned((float)f->latlonh[0], (float)f->latlonh[2], f->gravity_n);
    }
}

/* The filter works with float positions in the local n-frame, the absolute
 * position (latlonh, double) is book-kept incrementally from the n-frame
 * deltas. Every position change arrives here as dxyz_n_delta (the delta that
 * was *applied* to pos_local), pass NULL if only the orientation changed. The
 * trigonometry only runs every INS_EARTH_REFRESH_DIST_M meters of travel. */
/* @satisfies REQ-NAV-020 */
static void ins_update_meta(ins_t* f, bool orientation_changed,
                            const float* dxyz_n_delta) /* delta in n-frame, m */
{
    if (orientation_changed) { ins_quat_to_rotmat(f->state.qbn, f->R_b_to_n); }

    if (dxyz_n_delta != NULL)
    {
        f->latlonh[0] += (double)dxyz_n_delta[0] * f->meta_dlat_per_dN;
        f->latlonh[1] += (double)dxyz_n_delta[1] * f->meta_dlon_per_dE;
        f->latlonh[2] -= (double)dxyz_n_delta[2];

        f->meta_travel_m +=
            fabsf(dxyz_n_delta[0]) + fabsf(dxyz_n_delta[1]) + fabsf(dxyz_n_delta[2]);
        if (f->meta_travel_m > INS_EARTH_REFRESH_DIST_M) { ins_refresh_earth_params(f); }
    }
}

/* ============================================================================
 * UDU factorisation utilities
 * ============================================================================
 */

/* Initialise U,d so that P = U*diag(d)*U' is diagonal with the given
 * variances on the diagonal. U becomes the identity, d = variances. */
static void udu_set_diag(float* U, float* d, const float* diag_variances, int n)
{
    int j;
    mateye(U, n);
    for (j = 0; j < n; ++j) { d[j] = diag_variances[j]; }
}

/* Extract the diagonal of P = U*diag(d)*U' (U unit upper triangular):
 * P_ii = sum_k>=i U(i,k)^2 * d(k), with U(i,i) = 1. diag_out is an
 * INS_UNKNOWNS_MAX-sized buffer and is written in full: entries beyond the
 * active state size are zeroed rather than left as stack garbage. */
static void udu_get_diag(const float* U, const float* d, float* diag_out, int n)
{
    int i, k;
    for (i = (n > 0) ? n : 0; i < INS_UNKNOWNS_MAX; ++i) { diag_out[i] = 0.0f; }
    for (i = 0; i < n; ++i)
    {
        float s = d[i];
        for (k = i + 1; k < n; ++k)
        {
            s += MAT_ELEM(U, i, k, n, n) * MAT_ELEM(U, i, k, n, n) * d[k];
        }
        diag_out[i] = s;
    }
}

/* Same equation as udu_get_diag(), for single index: O(n-idx) instead of
 * O(n^2) when a caller only needs one element */
static float udu_get_diag_one(const float* U, const float* d, int n, int idx)
{
    float s = d[idx];
    for (int k = idx + 1; k < n; ++k)
    {
        s += MAT_ELEM(U, idx, k, n, n) * MAT_ELEM(U, idx, k, n, n) * d[k];
    }
    return s;
}

/* ============================================================================
 * Dead-reckoning window ("tunnel" coasting)
 * ============================================================================
 */

/* Timestamp of the filter's most recent epoch (newest history entry, one is
 * saved every update). */
static ins_time_us_t ins_last_time(const ins_t* f)
{
    return f->history[index_mod(f->history_index - 1, INS_HISTORY_ITEMS_MAX)].time;
}

static float ins_max_dr_sec(const ins_t* f)
{
    return (f->opt.max_deadreckoning_sec > 0.0f) ? f->opt.max_deadreckoning_sec
                                                 : INS_DEFAULT_MAX_DEADRECKONING_SEC;
}

/* Has the filter been coasting (no absolute position aiding) beyond the
 * configured window at time t? */
static bool ins_dr_expired(const ins_t* f, ins_time_us_t t)
{
    if (f->opt.allow_unlimited_deadreckoning) return false;
    return time_diff_sec(t, f->t_last_pos_aiding) > ins_max_dr_sec(f);
}

/* How long the filter has been inert at time t (REQ-NAV-064): the
 * position-aiding outage minus the part of it that was coasted normally.
 * The coasted part was propagated at the time, counting it again in the
 * re-acquisition inflation (REQ-NAV-065) would double it. */
static float ins_dr_frozen_sec(const ins_t* f, ins_time_us_t t)
{
    const float frozen = time_diff_sec(t, f->t_last_pos_aiding) - ins_max_dr_sec(f);
    return (frozen > 0.0f) ? frozen : 0.0f;
}

/* ============================================================================
 * Apply error-state correction dx to the nominal state.
 * Sign convention: state <- state - correction, since the error-state is
 * (nominal - truth).
 * ============================================================================
 */

static void ins_apply_correction(ins_t* f, const float dx[INS_UNKNOWNS_MAX])
{
    /* Position correction: float-only in the local n-frame. The applied
       delta is also book-kept into latlonh (see ins_update_meta at the
       end of this function). */
    const float dpos_n_applied[3] = {-dx[INS_IDX_POS + 0], -dx[INS_IDX_POS + 1],
                                     -dx[INS_IDX_POS + 2]};
    f->state.pos_local[0] += dpos_n_applied[0];
    f->state.pos_local[1] += dpos_n_applied[1];
    f->state.pos_local[2] += dpos_n_applied[2];

    /* Velocity correction is in n-frame. */
    f->state.vel_ned[0] -= dx[INS_IDX_VEL + 0];
    f->state.vel_ned[1] -= dx[INS_IDX_VEL + 1];
    f->state.vel_ned[2] -= dx[INS_IDX_VEL + 2];

    /* Attitude correction: small-angle rpy. */
    const float drpy[3] = {dx[INS_IDX_RPY + 0], dx[INS_IDX_RPY + 1], dx[INS_IDX_RPY + 2]};
    float       q_new[4];
    ins_quat_small_angle_correction(f->state.qbn, drpy, q_new);
    memcpy(f->state.qbn, q_new, sizeof(q_new));

    /* Bias corrections. */
    f->state.acc_bias[0] -= dx[INS_IDX_ACC + 0];
    f->state.acc_bias[1] -= dx[INS_IDX_ACC + 1];
    f->state.acc_bias[2] -= dx[INS_IDX_ACC + 2];
    f->state.gyr_bias[0] -= dx[INS_IDX_GYR + 0];
    f->state.gyr_bias[1] -= dx[INS_IDX_GYR + 1];
    f->state.gyr_bias[2] -= dx[INS_IDX_GYR + 2];

    /* Magnetometer hard-iron bias (18-state mode only). */
    if (f->n > INS_IDX_MAG)
    {
        f->state.mag_bias[0] -= dx[INS_IDX_MAG + 0];
        f->state.mag_bias[1] -= dx[INS_IDX_MAG + 1];
        f->state.mag_bias[2] -= dx[INS_IDX_MAG + 2];
    }

    /* Refresh cached helpers (R_b_to_n, latlonh, gravity). */
    ins_update_meta(f, true, dpos_n_applied);
}

/* ============================================================================
 * Strapdown integration: IMU measurements from epoch k to k+1 (attitude,
 * velocity, trapezoidal position). On return f->state is at epoch k+1,
 * f->last_omega_b_nb and f->last_acc_n are updated, and dpos_n_out holds the
 * n-frame position delta (used by ins_update_meta for the latlonh book-keeping).
 * ============================================================================
 */

/* @satisfies REQ-NAV-003 */
static void ins_strapdown(ins_t* f, const ins_measurements_t* m, float dt_sec, float dpos_n_out[3])
{
    if (dt_sec <= 0.0f || dt_sec > f->opt.max_prediction_time_sec)
    {
        if (dpos_n_out != NULL) vec3_zero(dpos_n_out);
        return;
    }

    /* Bias-corrected rotation rate (body frame, omega_b_ib). */
    const float omega_b_ib[3] = {m->gyr.data[0] - f->state.gyr_bias[0],
                                 m->gyr.data[1] - f->state.gyr_bias[1],
                                 m->gyr.data[2] - f->state.gyr_bias[2]};

    /* Compute omega_n_in = omega_n_ie + omega_n_en
       and then omega_b_nb = omega_b_ib - R_b_to_n' * omega_n_in. */
    float omega_n_in[3], omega_n_ie[3], omega_n_en[3];
    ins_calc_omega_n_in(f->latlonh[0], f->latlonh[2], f->state.vel_ned, omega_n_in, omega_n_ie,
                        omega_n_en);

    float R_t_omega_n_in[3];
    mat3t_mul_vec3(f->R_b_to_n, omega_n_in, R_t_omega_n_in);

    const float omega_b_nb[3] = {omega_b_ib[0] - R_t_omega_n_in[0],
                                 omega_b_ib[1] - R_t_omega_n_in[1],
                                 omega_b_ib[2] - R_t_omega_n_in[2]};

    /* Save for later use (Kalman predict, accessors). */
    vec3_copy(omega_b_nb, f->last_omega_b_nb);

    /* Attitude update. */
    float q_new[4];
    ins_quat_rotate(f->state.qbn, omega_b_nb, dt_sec, q_new);
    memcpy(f->state.qbn, q_new, sizeof(q_new));

    /* Rotation matrix at epoch k (before update) is stored in f->R_b_to_n.
       For the specific force integration we use the *old* R_b_to_n plus a
       rotation correction term (Titterton 2nd ed. p. 326, Wendel 2nd ed.
       ch. 3, p. 58). */

    /* Bias-corrected specific force (body frame). */
    const float f_b[3] = {m->acc.data[0] - f->state.acc_bias[0],
                          m->acc.data[1] - f->state.acc_bias[1],
                          m->acc.data[2] - f->state.acc_bias[2]};

    /* Coriolis (+centrifugal is already in gravity_n). */
    const float two_wie_plus_wen[3] = {2.0f * omega_n_ie[0] + omega_n_en[0],
                                       2.0f * omega_n_ie[1] + omega_n_en[1],
                                       2.0f * omega_n_ie[2] + omega_n_en[2]};
    float       f_coriolis[3];
    ins_cross(two_wie_plus_wen, f->state.vel_ned, f_coriolis);
    f_coriolis[0] = -f_coriolis[0];
    f_coriolis[1] = -f_coriolis[1];
    f_coriolis[2] = -f_coriolis[2];

    /* n-frame corrections = gravity + coriolis */
    const float f_corr[3] = {f->gravity_n[0] + f_coriolis[0], f->gravity_n[1] + f_coriolis[1],
                             f->gravity_n[2] + f_coriolis[2]};

    /* Body acceleration in n-frame (saved for accessor). */
    float R_fb[3];
    mat3_mul_vec3(f->R_b_to_n, f_b, R_fb);
    f->last_acc_n[0] = R_fb[0] + f_corr[0];
    f->last_acc_n[1] = R_fb[1] + f_corr[1];
    f->last_acc_n[2] = R_fb[2] + f_corr[2];

    /* Windowed mean of the same vector (REQ-NAV-075), the quantity the
       manoeuvre-dependent GNSS velocity noise is a function of. A first-order
       average rather than a boxcar, so it costs three floats instead of a ring
       buffer at IMU rate. The VECTOR is averaged, not its magnitude: the mean
       acceleration is what sets the a*T/2 error of an interval-averaged
       velocity, and a zero-mean vibration cancels instead of accumulating.

       Alongside it, the outer product omega*omega' over the same window
       (REQ-NAV-076): the lever-arm-free half of the centripetal acceleration
       the GNSS antenna sees, omega x (omega x l) = (omega*omega' - |omega|^2
       I) l. The lever arm arrives with the fix, so only the operator is
       carried, six floats for a symmetric 3x3. Averaging the outer product
       rather than omega keeps the centripetal effect of an oscillation about
       zero.
       @satisfies REQ-NAV-075 REQ-NAV-076 */
    {
        const float w_outer[6] = {omega_b_nb[0] * omega_b_nb[0], omega_b_nb[1] * omega_b_nb[1],
                                  omega_b_nb[2] * omega_b_nb[2], omega_b_nb[0] * omega_b_nb[1],
                                  omega_b_nb[0] * omega_b_nb[2], omega_b_nb[1] * omega_b_nb[2]};
        int         k;
        if (!(f->opt.gnss_vel_noise_acc_window_sec > 0.0f) || !f->acc_n_avg_valid)
        {
            vec3_copy(f->last_acc_n, f->acc_n_avg);
            for (k = 0; k < 6; ++k) { f->omega_outer_avg[k] = w_outer[k]; }
            f->acc_n_avg_valid = true;
        }
        else
        {
            float alpha = dt_sec / f->opt.gnss_vel_noise_acc_window_sec;
            if (alpha > 1.0f) { alpha = 1.0f; }
            if (alpha > 0.0f)
            {
                f->acc_n_avg[0] += alpha * (f->last_acc_n[0] - f->acc_n_avg[0]);
                f->acc_n_avg[1] += alpha * (f->last_acc_n[1] - f->acc_n_avg[1]);
                f->acc_n_avg[2] += alpha * (f->last_acc_n[2] - f->acc_n_avg[2]);
                for (k = 0; k < 6; ++k)
                {
                    f->omega_outer_avg[k] += alpha * (w_outer[k] - f->omega_outer_avg[k]);
                }
            }
        }
    }

    /* Rotation correction: 0.5 * omega x f_b * dt^2 (body frame),
       then rotated into n-frame. */
    float rot_corr_b[3];
    float scaled_omega[3] = {0.5f * omega_b_nb[0], 0.5f * omega_b_nb[1], 0.5f * omega_b_nb[2]};
    ins_cross(scaled_omega, f_b, rot_corr_b);
    /* integrated specific force in body frame (f_b*dt + 0.5*omega x f_b * dt^2)
     */
    const float int_spec_force_b[3] = {f_b[0] * dt_sec + rot_corr_b[0] * dt_sec * dt_sec,
                                       f_b[1] * dt_sec + rot_corr_b[1] * dt_sec * dt_sec,
                                       f_b[2] * dt_sec + rot_corr_b[2] * dt_sec * dt_sec};

    float dv_from_acc[3];
    mat3_mul_vec3(f->R_b_to_n, int_spec_force_b, dv_from_acc);

    const float du[3] = {dv_from_acc[0] + f_corr[0] * dt_sec, dv_from_acc[1] + f_corr[1] * dt_sec,
                         dv_from_acc[2] + f_corr[2] * dt_sec};

    /* Trapezoidal position update: dpos = 0.5 * dt * (v_k + v_{k+1}) */
    const float vk[3] = {f->state.vel_ned[0], f->state.vel_ned[1], f->state.vel_ned[2]};
    f->state.vel_ned[0] += du[0];
    f->state.vel_ned[1] += du[1];
    f->state.vel_ned[2] += du[2];

    const float dpos_n[3] = {0.5f * dt_sec * (vk[0] + f->state.vel_ned[0]),
                             0.5f * dt_sec * (vk[1] + f->state.vel_ned[1]),
                             0.5f * dt_sec * (vk[2] + f->state.vel_ned[2])};
    f->state.pos_local[0] += dpos_n[0];
    f->state.pos_local[1] += dpos_n[1];
    f->state.pos_local[2] += dpos_n[2];

    if (dpos_n_out != NULL) { vec3_copy(dpos_n, dpos_n_out); }

    /* Cache the last bias-corrected specific force for the predict step
       (needed for Phi's velocity-coupling-to-attitude block). */
    vec3_copy(f_b, f->last_acc_meas);
    f->last_acc_valid = true;
}

/* ============================================================================
 * Kalman prediction step
 *
 * Uses the UDU Thornton routine: kalman_udu_predict().
 * ============================================================================
 */

/* Per-axis "0 -> default" fallback for a measurement noise PSD triad
 * (ARW/VRW): a caller that leaves Qll_diag unset would otherwise inject
 * zero process noise for that axis, making the filter silently overconfident
 * instead of visibly wrong. Falls back to a conservative consumer-MEMS
 * noise floor per axis. */
/* @satisfies REQ-NAV-017 */
static void ins_resolve_noise_psd(const float in[3], float default_psd, float out[3])
{
    int i;
    for (i = 0; i < 3; ++i) { out[i] = (in[i] > 0.0f) ? in[i] : default_psd; }
}

/** @brief Discrete-time state transition Jacobian (Phi) for one dt_sec step of
 * the ESKF. Source: Wendel 2nd ed. equation (10.1), p. 280.
 *
 * @param[in] dt_sec Timestep [s].
 * @param[in] f_b_ib Specific force in body frame (3x1) [m/s^2], bias-corrected.
 * @param[in] R_b_to_n Rotation matrix (9, column-major).
 * @param[in] n Active error-state size (15 or 18).
 * @param[out] H Output n x n Phi matrix (column-major). */
static void ins_compute_Phi(float dt_sec, const float f_b_ib[3], const float R[9], int n,
                            float Phi[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX])
{
    int i, j;

    /* Start with identity. In 18-state mode this also covers the
       magnetometer-bias block: a pure random walk (Phi = I, noise via
       Qxx_noise_diag), inert in the strapdown. */
    mateye(Phi, n);

    /* Convert specific force into n-frame: f_n = R * f_b_ib (NED components).
     */
    float f_n[3];
    matmul("N", "N", 3, 1, 3, 1.0f, R, f_b_ib, 0.0f, f_n);
    const float fnN = f_n[0];
    const float fnE = f_n[1];
    const float fnD = f_n[2];

    /* pos += vel * dt : top-right 3x3 block at (POS, VEL) */
    MAT_ELEM(Phi, INS_IDX_POS + 0, INS_IDX_VEL + 0, n, n) += dt_sec;
    MAT_ELEM(Phi, INS_IDX_POS + 1, INS_IDX_VEL + 1, n, n) += dt_sec;
    MAT_ELEM(Phi, INS_IDX_POS + 2, INS_IDX_VEL + 2, n, n) += dt_sec;

    /* vel coupling to rpy: [f_n]_x * dt (specific force cross product) */
    MAT_ELEM(Phi, INS_IDX_VEL + 0, INS_IDX_RPY + 1, n, n) += +fnD * dt_sec;
    MAT_ELEM(Phi, INS_IDX_VEL + 0, INS_IDX_RPY + 2, n, n) += -fnE * dt_sec;
    MAT_ELEM(Phi, INS_IDX_VEL + 1, INS_IDX_RPY + 0, n, n) += -fnD * dt_sec;
    MAT_ELEM(Phi, INS_IDX_VEL + 1, INS_IDX_RPY + 2, n, n) += +fnN * dt_sec;
    MAT_ELEM(Phi, INS_IDX_VEL + 2, INS_IDX_RPY + 0, n, n) += +fnE * dt_sec;
    MAT_ELEM(Phi, INS_IDX_VEL + 2, INS_IDX_RPY + 1, n, n) += -fnN * dt_sec;

    /* accel bias coupling to velocity: -R * dt */
    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            MAT_ELEM(Phi, INS_IDX_VEL + i, INS_IDX_ACC + j, n, n) +=
                -MAT_ELEM(R, i, j, 3, 3) * dt_sec;
        }
    }

    /* gyro bias coupling to attitude: -R * dt */
    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            MAT_ELEM(Phi, INS_IDX_RPY + i, INS_IDX_GYR + j, n, n) +=
                -MAT_ELEM(R, i, j, 3, 3) * dt_sec;
        }
    }
}

/* @satisfies REQ-NAV-002 REQ-NAV-004 REQ-NAV-069 */
static void ins_predict(ins_t* f, float dt_sec, const float Qll_acc_diag[3],
                        const float Qll_gyr_diag[3], float* phi_out)
{
    /* Specific force to pass into Phi. If we have no accelerometer sample
       yet, fall back to zero. */
    const float f_b_ib[3] = {f->last_acc_valid ? f->last_acc_meas[0] : 0.0f,
                             f->last_acc_valid ? f->last_acc_meas[1] : 0.0f,
                             f->last_acc_valid ? f->last_acc_meas[2] : 0.0f};

    const int n = f->n;
    float     Phi[INS_UNKNOWNS_MAX * INS_UNKNOWNS_MAX];
    ins_compute_Phi(dt_sec, f_b_ib, f->R_b_to_n, n, Phi);
    if (phi_out != NULL) { memcpy(phi_out, Phi, sizeof(Phi[0]) * (size_t)(n * n)); }

    /* Process noise in noise-input form, as the Thornton step wants it:
     *   P^- = Phi * P^+ * Phi' + G * diag(Q) * G'
     * G maps the 12 IMU noise inputs [acc, gyr, acc-bias-RW, gyr-bias-RW] plus
     * one per-state extra-noise input for every state whose Qxx_noise_diag can
     * be nonzero (POS/VEL/RPY, plus MAG in 18-state mode) onto the error
     * states. ACC/GYR bias states have no Qxx_noise_diag setter (their process
     * noise comes from the acc_bias_psd/gyr_bias_psd IMU-noise columns
     * instead, see ins_init()), so their extra-noise column is always exactly
     * zero and is skipped here rather than carried through the UDU predict:
     *
     *        acc  gyr  aRW gRW | extra (POS/VEL/RPY[/MAG])
     *   G = [  0    0    0   0 |  I  ]   POS
     *       [  R    0    0   0 |  I  ]   VEL  <- accel white noise
     *       [  0   -R    0   0 |  I  ]   RPY  <- gyro white noise
     *       [  0    0    I   0 |  0  ]   ACC  <- accel bias random walk
     *       [  0    0    0   I |  0  ]   GYR  <- gyro bias random walk
     *
     * with Q = [PSD] * dt per column. */
    const int nr = 12 + n - 6; /* 12 IMU + per-state extra, minus the always-zero ACC/GYR pair */
    float     G[INS_UNKNOWNS_MAX * INS_NOISE_COLS_MAX];
    float     Q[INS_NOISE_COLS_MAX];
    memset(G, 0, sizeof(G[0]) * (size_t)(n * nr));
    int i, j;
    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            MAT_ELEM(G, INS_IDX_VEL + i, 0 + j, n, nr) = MAT_ELEM(f->R_b_to_n, i, j, 3, 3);
            MAT_ELEM(G, INS_IDX_RPY + i, 3 + j, n, nr) = -MAT_ELEM(f->R_b_to_n, i, j, 3, 3);
        }
        MAT_ELEM(G, INS_IDX_ACC + i, 6 + i, n, nr) = 1.0f;
        MAT_ELEM(G, INS_IDX_GYR + i, 9 + i, n, nr) = 1.0f;
    }

    const float acc_bias_psd = qsquare(f->init.acc_bias_pred_stddev_mps2_sqrts);
    const float gyr_bias_psd = qsquare(f->init.gyr_bias_pred_stddev_rps_sqrts);
    for (i = 0; i < 3; ++i)
    {
        Q[0 + i] = Qll_acc_diag[i] * dt_sec;
        Q[3 + i] = Qll_gyr_diag[i] * dt_sec;
        Q[6 + i] = acc_bias_psd * dt_sec;
        Q[9 + i] = gyr_bias_psd * dt_sec;
    }
    int col = 12;
    for (i = 0; i < n; ++i)
    {
        if (i >= INS_IDX_ACC && i < INS_IDX_GYR + 3) { continue; } /* ACC+GYR: no extra column */
        MAT_ELEM(G, i, col, n, nr) = 1.0f;
        Q[col]                     = f->Qxx_noise_diag[i] * dt_sec;
        ++col;
    }

    /* State vector is handled by the error-state framework: Phi*x is not
       applied because x is implicitly zero (the nominal state has been
       predicted by the strapdown). Pass NULL to skip the state update. */
    kalman_udu_predict(/*x=*/NULL, f->U, f->d, Phi, G, Q, n, nr);
}

/* ============================================================================
 * Measurement fusion (Bierman UDU update, kalman_udu.h)
 *
 * All fusions share the same error-state pattern:
 *   - Build the residual  z = h(nominal) - z_measured  ( = H * dx + noise,
 *     since the error-state is dx = nominal - truth and starts at zero).
 *   - Run the robust Bierman update kalman_udu() on (U, d) with dx = 0.
 *   - Subtract the estimated dx from the nominal state and refresh the cached
 *     meta data (ins_apply_correction).
 * ============================================================================
 */

/* Inverse chi-square CDF for 1 degree of freedom, chi2inv(p, 1), by linear
 * interpolation of a small quantile table (REQ-NAV-046). p is the acceptance
 * probability 1 - alpha, clamped to the table's endpoints. Only used at init.
 * Clamped at 5.42 sigma (float precision). */
static float ins_chi2inv_1dof(float p)
{
    static const float P[] = {0.500000000f, 0.600000000f, 0.700000000f, 0.800000000f, 0.850000000f,
                              0.900000000f, 0.950000000f, 0.975000000f, 0.990000000f, 0.995000000f,
                              0.997000000f, 0.998000000f, 0.999000000f, 0.999500000f, 0.999800000f,
                              0.999900000f, 0.999950000f, 0.999980000f, 0.999990000f, 0.999995000f,
                              0.99999809f,  0.99999905f,  0.99999952f,  0.99999976f,  0.99999988f,
                              0.99999994f};
    static const float X[] = {0.4549f,  0.7083f,  1.0742f,  1.6424f,  2.0723f,  2.7055f,  3.8415f,
                              5.0239f,  6.6349f,  7.8794f,  8.8075f,  9.5495f,  10.8276f, 12.1157f,
                              13.8311f, 15.1367f, 16.4476f, 18.1893f, 19.5114f, 20.8364f, 22.6849f,
                              24.0187f, 25.3546f, 26.6889f, 28.0310f, 29.3960f};
    const int          n   = (int)(sizeof(P) / sizeof(P[0]));
    int                i;

    if (!(p > P[0])) { return X[0]; } /* also for NaN */
    if (p >= P[n - 1]) { return X[n - 1]; }

    for (i = 1; i < n; ++i)
    {
        if (p <= P[i])
        {
            const float t = (p - P[i - 1]) / (P[i] - P[i - 1]);
            return X[i - 1] + t * (X[i] - X[i - 1]);
        }
    }
    return X[n - 1]; /* unreachable: p < P[n-1] guaranteed above */
}

/* Diagnostic-only re-check of kalman_udu's own chi2 gate (Chang 2014), used to
 * count downweighted fusions (REQ-NAV-036) without affecting the fusion. Uses
 * f->U/f->d BEFORE the sequential per-row updates, so it matches kalman_udu's
 * internal test exactly only for the first row of a batch. */
static bool ins_fuse_is_outlier(const ins_t* f, const float* z, const float* R, const float* Ht,
                                int m_count, float chi2_threshold)
{
    if (!(chi2_threshold > 0.0f)) { return false; }
    int i, j;
    for (i = 0; i < m_count; ++i)
    {
        float tmp[INS_UNKNOWNS_MAX];
        matmul("N", "N", 1, f->n, f->n, 1.0f, Ht + (ptrdiff_t)i * f->n, f->U, 0.0f, tmp);
        float HPHT = 0.0f;
        for (j = 0; j < f->n; ++j) { HPHT += tmp[j] * tmp[j] * f->d[j]; }
        const float Rv = MAT_ELEM(R, i, i, m_count, m_count);
        const float s  = HPHT + Rv;
        const float dz = z[i];
        if (dz * dz > chi2_threshold * s) { return true; }
    }
    return false;
}

/* Generic fusion of m_count measurements with full covariance R (m x m,
 * column-major, symmetric). Ht is the transposed measurement matrix (n x m,
 * column i holds row i of H). If R has off-diagonal entries, z/Ht/R are
 * whitened in-place with decorrelate() first, so the scalar Bierman updates
 * inside kalman_udu() see independent unit-variance measurements. Returns 0 on
 * success. */
/* @satisfies REQ-NAV-002 REQ-NAV-035 REQ-NAV-036 */
static int ins_fuse(ins_t* f, float* z, float* R, float* Ht, int m_count, float chi2_threshold,
                    int downweight_outlier)
{
    assert(m_count >= 1 && m_count <= INS_FUSE_MAX_MEAS);

    /* Global override (REQ-NAV-035): 0.0f makes kalman_udu skip the chi2
       test entirely. */
    if (f->opt.chi2_disable) { chi2_threshold = 0.0f; }

    float dx[INS_UNKNOWNS_MAX];
    int   i, j;

    bool correlated = false;
    for (j = 0; j < m_count; ++j)
    {
        if (MAT_ELEM(R, j, j, m_count, m_count) <= 0.0f)
        {
            f->diag.n_fuse_fail++;
            LOG_ERROR("ins: Measurement matrix R has elements <= 0.0: %f",
                      (double)MAT_ELEM(R, j, j, m_count, m_count));
            return -1; /* zero/negative variance would corrupt U,d */
        }
        for (i = 0; i < m_count; ++i)
        {
            /* Off-diagonal correlation term? Callers who don't model
               correlation leave this at exact 0.0f (memset/zero-init), so
               exact equality is the correct test, not an epsilon. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
            if (i != j && MAT_ELEM(R, i, j, m_count, m_count) != 0.0f)
#pragma GCC diagnostic pop
            {
                correlated = true;
            }
        }
    }

    float Reye[INS_FUSE_MAX_MEAS * INS_FUSE_MAX_MEAS];

    const float* R_use = R;
    if (correlated)
    {
        if (decorrelate(z, Ht, R, f->n, m_count) != 0)
        {
            f->diag.n_fuse_fail++;
            LOG_ERROR("ins: R not positive definite -> discard measurement");
            return -1;
        }
        mateye(Reye, m_count);
        R_use = Reye;
    }

    memset(dx, 0, sizeof(dx));
    if (downweight_outlier && ins_fuse_is_outlier(f, z, R_use, Ht, m_count, chi2_threshold))
    {
        f->diag.n_downweighted++;
    }
    const int rc =
        kalman_udu(dx, f->U, f->d, z, R_use, Ht, f->n, m_count, chi2_threshold, downweight_outlier);

    /* Apply whatever correction was accumulated (rows skipped by the
       chi2 outlier test simply contribute nothing). This also refreshes
       the cached meta data (latlonh, R_b_to_n, gravity). */
    ins_apply_correction(f, dx);
    if (rc != 0) f->diag.n_fuse_fail++;
    return rc;
}

/* Find the history item closest to t_target. Returns NULL if the history
 * holds no usable entry near that time. */
/* @satisfies REQ-NAV-008 */
static const ins_history_item_t* ins_find_history(const ins_t* f, ins_time_us_t t_target)
{
    const ins_history_item_t* best    = NULL;
    ins_time_us_t             best_dt = 0;

    int i;
    for (i = 0; i < INS_HISTORY_ITEMS_MAX; ++i)
    {
        const ins_history_item_t* h = &f->history[i];
        if (h->d[0] <= 0.0f) { continue; /* empty (never written) ring buffer slot */ }
        ins_time_us_t dt = h->time - t_target;
        if (dt < 0) dt = -dt;
        if (best == NULL || dt < best_dt)
        {
            best    = h;
            best_dt = dt;
        }
    }

    if (best != NULL && best_dt > INS_HISTORY_MATCH_TOL_US) { return NULL; }
    return best;
}

/* Fuse a scalar yaw-error residual (n-frame, [rad]) for the absolute-yaw
 * measurement interface: a heading residual equals the yaw component of the
 * n-frame attitude error state (H = e_z on the RPY block). */
static int ins_fuse_yaw_residual(ins_t* f, float dyaw, float R_yaw, float chi2_threshold)
{
    float Ht[INS_UNKNOWNS_MAX];
    memset(Ht, 0, sizeof(Ht));
    Ht[INS_IDX_RPY + 2] = 1.0f;
    return ins_fuse(f, &dyaw, &R_yaw, Ht, 1, chi2_threshold,
                    1 /* downweight outliers instead of skip */);
}

/* Magnetometer: fused as a 3D vector measurement of the body-frame field,
 * model m_b = R' * magnetic_n.
 *
 * Measurement Jacobian w.r.t. the n-frame attitude error psi:
 *   H_rpy = R' * [magnetic_n]_x     (3x3 on the RPY block)
 * The roll and pitch columns are deliberately zeroed: the magnetic dip angle
 * and hard-iron/deviation disturbances must not tilt the attitude estimate
 * (leveling comes from the accelerometer). Only the yaw column
 *   H(:,yaw) = R' * (magnetic_n x e_D)
 * is kept, so the measurement can only rotate the estimate about the vertical.
 *
 * Field-strength disturbance gate: once a position has been supplied
 * (mag_field_expected_uT > 0) the measured field magnitude is compared against
 * the WMM total field, and a sample outside the tolerance band has its
 * per-axis noise inflated (downweighted, not dropped). */
/* @satisfies REQ-NAV-009 REQ-NAV-028 REQ-NAV-029 */
static void ins_fuse_mag(ins_t* f, const ins_measurements_t* m)
{
    if (!m->mag.is_valid) return;
    /* Dip pole exclusion zone: magnetic_n carries a meaningless declination
       here, so the yaw it would imply is not usable (REQ-SYS-018). */
    if (!f->mag_heading_usable) return;
    if (time_diff_ms(m->timestamp, f->t_last_mag_fusion) < f->opt.magnetometer_min_delay_ms)
    {
        return;
    }
    /* Per-axis variance with the same "0 -> the sensor's default" policy the
       barometer sample gets: a caller who has not characterized its
       magnetometer must not be able to produce a zero-R update. */
    float R_diag[3];
    {
        int k;
        for (k = 0; k < 3; ++k)
        {
            R_diag[k] = (m->mag.Qll_diag[k] > 0.0f)
                            ? m->mag.Qll_diag[k]
                            : (INS_DEFAULT_MAG_STDDEV_UT * INS_DEFAULT_MAG_STDDEV_UT);
        }
    }

    /* Field-strength gate: inflate the measurement noise when the
       measured magnitude leaves the tolerance band around the WMM total
       field. Factor is 1 at the band edge and grows quadratically. */
    float R_infl = 1.0f;
    if (f->mag_field_expected_uT > 0.0f && !f->opt.mag_field_check_disable)
    {
        const float meas = SQRTF(m->mag.data[0] * m->mag.data[0] + m->mag.data[1] * m->mag.data[1] +
                                 m->mag.data[2] * m->mag.data[2]);
        const float tol  = (f->opt.mag_field_tolerance > 0.0f) ? f->opt.mag_field_tolerance
                                                               : INS_DEFAULT_MAG_FIELD_TOL;
        const float dev  = fabsf(meas - f->mag_field_expected_uT) / f->mag_field_expected_uT;
        if (dev > tol)
        {
            const float r = dev / tol;
            R_infl        = r * r;

            /* Persistent (not transient) magnetic disturbance diagnostics: a
               single glitchy epoch is already handled by the downweight above,
               a long run means the yaw rides on a corrupted reference. */
            f->log_state.mag_disturbed_count++;
            if (f->log_state.mag_disturbed_count == INS_LOG_MAG_DISTURB_EPOCHS)
            {
                LOG_WARN("ins: magnetometer field strength %.1f%% off the WMM model for "
                         "%u consecutive samples - persistent magnetic disturbance "
                         "(motor/ferrous structure?), yaw may be unreliable",
                         (double)(dev * 100.0f), (unsigned int)f->log_state.mag_disturbed_count);
                f->log_state.t_last_mag_disturb_warn = m->timestamp;
            }
            else if (f->log_state.mag_disturbed_count > INS_LOG_MAG_DISTURB_EPOCHS)
            {
                const float since_warn_sec =
                    time_diff_sec(m->timestamp, f->log_state.t_last_mag_disturb_warn);
                if (since_warn_sec >= INS_LOG_MAG_DISTURB_REPEAT_SEC)
                {
                    LOG_WARN("ins: magnetometer field strength %.1f%% off the WMM model, "
                             "still disturbed (%u consecutive samples)",
                             (double)(dev * 100.0f),
                             (unsigned int)f->log_state.mag_disturbed_count);
                    f->log_state.t_last_mag_disturb_warn = m->timestamp;
                }
            }
        }
        else { f->log_state.mag_disturbed_count = 0; }
    }

    /* Yaw is only observable from the horizontal field component. */
    const float mh_model =
        f->magnetic_n[0] * f->magnetic_n[0] + f->magnetic_n[1] * f->magnetic_n[1];
    if (mh_model < 1e-6f)
    {
        LOG_WARN("ins: magnetometer horizontal field zero");
        return;
    }

    const int n = f->n;

    /* Predicted body-frame field: R' * m_n plus the hard-iron bias
       estimate (18-state mode, zero otherwise). */
    float m_b_pred[3];
    mat3t_mul_vec3(f->R_b_to_n, f->magnetic_n, m_b_pred);
    m_b_pred[0] += f->state.mag_bias[0];
    m_b_pred[1] += f->state.mag_bias[1];
    m_b_pred[2] += f->state.mag_bias[2];

    /* Yaw column of the Jacobian: R' * (magnetic_n x e_D). */
    const float m_x_ez[3] = {f->magnetic_n[1], -f->magnetic_n[0], 0.0f};
    float       h_yaw[3];
    mat3t_mul_vec3(f->R_b_to_n, m_x_ez, h_yaw);

    float z[3];
    float R[3 * 3];
    float Ht[INS_UNKNOWNS_MAX * 3];
    memset(R, 0, sizeof(R));
    memset(Ht, 0, sizeof(Ht));
    int i;
    for (i = 0; i < 3; ++i)
    {
        z[i]                                   = m_b_pred[i] - m->mag.data[i];
        MAT_ELEM(R, i, i, 3, 3)                = R_diag[i] * R_infl;
        MAT_ELEM(Ht, INS_IDX_RPY + 2, i, n, 3) = h_yaw[i];
        if (n > INS_IDX_MAG)
        {
            /* dz/dbias = +I: the bias error appears one-to-one in the predicted
               body-frame field. Fixed in the body frame while the field term
               rotates with attitude, which makes it observable under
               rotation. */
            MAT_ELEM(Ht, INS_IDX_MAG + i, i, n, 3) = 1.0f;
        }
    }

    if (ins_fuse(f, z, R, Ht, 3, f->chi2_thr_mag,
                   1 /* downweight outliers: same deadlock argument as in
                        ins_fuse_yaw_residual */) == 0)
    {
        f->t_last_mag_fusion        = m->timestamp;
        f->log_state.t_last_yaw_aid = m->timestamp;
    }
}

/* Absolute yaw aiding (e.g. lighthouse/mocap pose orientation, dual-antenna
 * GNSS heading, gyro compass). Fused as a scalar yaw-error measurement,
 * optionally anchored in the history for delayed measurements. */
/* @satisfies REQ-NAV-010 */
static void ins_fuse_yaw(ins_t* f, const ins_measurements_t* m)
{
    if (!m->yaw.is_valid) return;
    if (m->yaw.stddev_rad <= 0.0f) return;

    const int delay_ms = (m->yaw_delay_ms > 0) ? m->yaw_delay_ms : 0;
    if (delay_ms > INS_MAX_DELAY_MS) return;

    /* Attitude at time-of-validity. */
    const float* R_tov = f->R_b_to_n;
    if (delay_ms > 0)
    {
        const ins_history_item_t* h = ins_find_history(f, m->timestamp - delay_ms * INS_US_PER_MS);
        if (h == NULL)
        {
            LOG_INFO("ins: cannot anchor yaw measurement in history -> skip");
            return;
        }
        R_tov = h->R_b_to_n;
    }

    /* Yaw is ill-defined near pitch = +/-90 deg -> skip. */
    const float sp = -MAT_ELEM(R_tov, 2, 0, 3, 3);
    if (sp > 0.99f || sp < -0.99f)
    {
        LOG_INFO("ins: pitch close to +/- 90 deg -> skip yaw measurement ");
        return;
    }

    float roll, pitch, yaw_nom;
    ins_rotmat_to_rpy(R_tov, &roll, &pitch, &yaw_nom);

    const float dyaw = ins_angle_diff(yaw_nom, m->yaw.yaw_rad);
    if (ins_fuse_yaw_residual(f, dyaw, qsquare(m->yaw.stddev_rad), f->chi2_thr_yaw) == 0)
    {
        f->log_state.t_last_yaw_aid = m->timestamp;
    }
}

/* Zero-velocity update: direct measurement of the velocity states.
 * Rate-limited to one fusion per nominal Kalman period. `trigger` is
 * m->zero_velocity_update OR'd with the automatic ZUPT detector. */
/* @satisfies REQ-NAV-012 */
static void ins_fuse_zero_velocity(ins_t* f, const ins_measurements_t* m, bool trigger)
{
    if (!trigger) return;
    const int min_dt_ms = (int)(ins_kalman_dt(f) * 1000.0f);
    if (time_diff_ms(m->timestamp, f->t_last_zero_vel_fusion) < min_dt_ms) { return; }

    float z[3];
    float R[3 * 3];
    float Ht[INS_UNKNOWNS_MAX * 3];
    memset(R, 0, sizeof(R));
    memset(Ht, 0, sizeof(Ht));
    int i;
    for (i = 0; i < 3; ++i)
    {
        z[i]                                      = f->state.vel_ned[i]; /* truth is 0 */
        MAT_ELEM(R, i, i, 3, 3)                   = qsquare(f->init.zero_vel_stddev_mps);
        MAT_ELEM(Ht, INS_IDX_VEL + i, i, f->n, 3) = 1.0f;
    }

    if (ins_fuse(f, z, R, Ht, 3, 0.0f, 0) == 0) { f->t_last_zero_vel_fusion = m->timestamp; }
}

/* Zero-rotation update: with omega_b_nb == 0 the gyro should read the gyro bias
 * plus the n-frame rotation (Earth rate + transport rate) seen in the body
 * frame -> direct measurement of the gyro bias states. Rate-limited to one
 * fusion per nominal Kalman period. `trigger` is m->zero_rotation_update OR'd
 * with the automatic ZARU detector. The fused measurement is the auto-ZUPT
 * detector's average over the current stillness run where available, so the
 * vibration of a still-but-idling platform stays out of the bias states. */
/* @satisfies REQ-NAV-012 REQ-NAV-014 */
static void ins_fuse_zero_rotation(ins_t* f, const ins_measurements_t* m, bool trigger)
{
    if (!trigger || !m->gyr.is_valid) return;
    const int min_dt_ms = (int)(ins_kalman_dt(f) * 1000.0f);
    if (time_diff_ms(m->timestamp, f->t_last_zero_rot_fusion) < min_dt_ms) { return; }

    float gyr_meas[3];
    if (f->auto_zupt_gyr_count > 0)
    {
        const float inv_n = 1.0f / (float)f->auto_zupt_gyr_count;
        gyr_meas[0]       = f->auto_zupt_gyr_sum[0] * inv_n;
        gyr_meas[1]       = f->auto_zupt_gyr_sum[1] * inv_n;
        gyr_meas[2]       = f->auto_zupt_gyr_sum[2] * inv_n;
    }
    else
    {
        /* Manual trigger without a detector stillness run (detector
           disabled, or the caller knows better than the static gate). */
        vec3_copy(m->gyr.data, gyr_meas);
    }

    float omega_n_in[3];
    ins_calc_omega_n_in(f->latlonh[0], f->latlonh[2], f->state.vel_ned, omega_n_in, NULL, NULL);
    float omega_b_in[3];
    mat3t_mul_vec3(f->R_b_to_n, omega_n_in, omega_b_in);

    float z[3];
    float R[3 * 3];
    float Ht[INS_UNKNOWNS_MAX * 3];
    memset(R, 0, sizeof(R));
    memset(Ht, 0, sizeof(Ht));
    int i;
    for (i = 0; i < 3; ++i)
    {
        z[i]                    = f->state.gyr_bias[i] + omega_b_in[i] - gyr_meas[i];
        MAT_ELEM(R, i, i, 3, 3) = qsquare(f->init.zero_rot_stddev_rps);
        MAT_ELEM(Ht, INS_IDX_GYR + i, i, f->n, 3) = 1.0f;
    }

    if (ins_fuse(f, z, R, Ht, 3, 0.0f, 0) == 0)
    {
        f->t_last_zero_rot_fusion = m->timestamp;
        /* Start a fresh averaging window */
        f->auto_zupt_gyr_count = 0;
    }
}

static void ins_save_state(ins_t* f, ins_time_us_t tnow, bool enforce);

/* Use roll/pitch(/yaw) from an external attitude hint (REQ-NAV-048), e.g.
 * nav_suite's ARS/AHRS: used by ins_reacquire(), where the dead-reckoned
 * attitude carried through a long outage can be worse than a
 * continuously-running attitude-only filter's. Discards existing
 * cross-correlations! No-op without a usable roll/pitch. Yaw is NOT resolved
 * here: an absent yaw hint means nobody knows the heading (REQ-NAV-059). */
static void ins_apply_att_hint(ins_t* f, const ins_meas_att_hint_t* hint)
{
    if (!hint->is_valid || !(hint->stddev_roll_rad > 0.0f) || !(hint->stddev_pitch_rad > 0.0f))
    {
        return;
    }

    float roll, pitch, yaw;
    ins_rotmat_to_rpy(f->R_b_to_n, &roll, &pitch, &yaw); /* keep existing yaw unless hinted */
    if (hint->stddev_yaw_rad > 0.0f) { yaw = hint->yaw_rad; }
    ins_quat_from_rpy(hint->roll_rad, hint->pitch_rad, yaw, f->state.qbn);
    ins_quat_to_rotmat(f->state.qbn, f->R_b_to_n);

    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    const float rp_var    = qsquare(fmaxf(hint->stddev_roll_rad, hint->stddev_pitch_rad));
    diag[INS_IDX_RPY + 0] = rp_var;
    diag[INS_IDX_RPY + 1] = rp_var;
    if (hint->stddev_yaw_rad > 0.0f) { diag[INS_IDX_RPY + 2] = qsquare(hint->stddev_yaw_rad); }
    udu_set_diag(f->U, f->d, diag, f->n);
}

/* Re-seed the gyro-bias states from an external hint (REQ-NAV-048), one axis at
 * a time (an axis with stddev <= 0 is left untouched). cap_to_prior is set by
 * the bootstrap caller only (REQ-NAV-067), where the hint lands on a filter
 * that has learned nothing yet. */
/* @satisfies REQ-NAV-067 */
static void ins_apply_gyr_bias_hint(ins_t* f, const ins_meas_att_hint_t* hint, bool cap_to_prior)
{
    if (!hint->is_valid) { return; }
    int  i;
    bool any = false;
    for (i = 0; i < 3; ++i)
    {
        if (hint->stddev_gyr_bias_rps[i] > 0.0f) { any = true; }
    }
    if (!any) { return; }

    const float cap_sd =
        fmaxf(f->init.gyr_bias_init_stddev_rps, INS_DEFAULT_GYR_BIAS_INIT_STDDEV_RPS);
    const float cap = qsquare(cap_sd);

    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    for (i = 0; i < 3; ++i)
    {
        if (hint->stddev_gyr_bias_rps[i] > 0.0f)
        {
            const float var       = qsquare(hint->stddev_gyr_bias_rps[i]);
            f->state.gyr_bias[i]  = hint->gyr_bias_rps[i];
            diag[INS_IDX_GYR + i] = (cap_to_prior && var > cap) ? cap : var;
        }
    }
    udu_set_diag(f->U, f->d, diag, f->n);
}

/* Re-acquisition yaw prior (REQ-NAV-059): unless an external hint carried an
 * absolute heading, the yaw coming out of an expired coasting window is not
 * "the pre-outage yaw plus a little gyro drift" - the nominal state was FROZEN
 * for the whole outage (REQ-NAV-022), so a car leaving a curved tunnel comes
 * out with an arbitrary heading error.
 *
 * That inconsistency does not stay in the yaw state: next to a gyro-bias
 * variance re-acquisition has just inflated (REQ-NAV-048), a yaw variance still
 * claiming sub-degree accuracy is the tighter of the two, so the aiding that
 * follows routes the heading correction into the gyro z bias instead. The
 * honest prior is therefore "heading unknown". Only ever widens: a hinted yaw
 * keeps its own variance. Runs after ins_apply_att_hint(). */
static void ins_reacquire_reset_yaw(ins_t* f, const ins_meas_att_hint_t* hint)
{
    if (hint->is_valid && hint->stddev_yaw_rad > 0.0f) { return; }

    const float yaw_var = qsquare(INS_YAW_UNKNOWN_STDDEV);
    float       diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    if (diag[INS_IDX_RPY + 2] < yaw_var)
    {
        diag[INS_IDX_RPY + 2] = yaw_var;
        udu_set_diag(f->U, f->d, diag, f->n);
        LOG_INFO("ins: re-acquisition without a heading source, yaw prior reset to unknown "
                 "(%.0f deg)",
                 (double)RAD2DEG(INS_YAW_UNKNOWN_STDDEV));
    }
}

/* Grow one variance by psd * dt, capped at the state's initial prior and
 * never lowering what is already there. */
static void ins_inflate_one(float* var, float psd, float dt_sec, float cap)
{
    const float grown = *var + psd * dt_sec;
    const float value = (grown > cap) ? cap : grown;
    if (value > *var) { *var = value; }
}

/* Price the inert interval into the states that survive it (REQ-NAV-065).
 * While the filter was frozen (REQ-NAV-064) nothing was propagated, so the
 * attitude and bias states still carry the confidence they had when aiding
 * stopped. This applies the random-walk growth the time update would have
 * accumulated, capped at the configured initial priors, so an arbitrarily long
 * outage leaves the filter at worst as uncertain as a cold start.
 *
 * Position and velocity are absent on purpose: the caller overwrites both from
 * the measurement. Runs BEFORE the hint (REQ-NAV-048) and the yaw prior
 * (REQ-NAV-059), so an externally supplied variance still wins. */
/* @satisfies REQ-NAV-065 */
static void ins_inflate_frozen_states(const ins_t* f, const ins_measurements_t* m,
                                      float dt_frozen_sec, float diag[])
{
    int i;
    if (!(dt_frozen_sec > 0.0f)) return;

    /* Attitude grows at the gyro noise rate, the same input the prediction
       feeds through G (the epoch's own ARW if the caller reports one). */
    float Qg[3];
    ins_resolve_noise_psd(m->gyr.Qll_diag, INS_DEFAULT_GYR_ARW_PSD, Qg);

    const float acc_bias_psd = qsquare(f->init.acc_bias_pred_stddev_mps2_sqrts);
    const float gyr_bias_psd = qsquare(f->init.gyr_bias_pred_stddev_rps_sqrts);
    const float acc_bias_cap = qsquare(f->init.acc_bias_init_stddev_mps2);
    const float gyr_bias_cap = qsquare(f->init.gyr_bias_init_stddev_rps);

    for (i = 0; i < 3; ++i)
    {
        ins_inflate_one(&diag[INS_IDX_RPY + i], Qg[i] + f->Qxx_noise_diag[INS_IDX_RPY + i],
                        dt_frozen_sec, qsquare(f->init.rpy_init_stddev_rad[i]));
        ins_inflate_one(&diag[INS_IDX_ACC + i], acc_bias_psd + f->Qxx_noise_diag[INS_IDX_ACC + i],
                        dt_frozen_sec, acc_bias_cap);
        ins_inflate_one(&diag[INS_IDX_GYR + i], gyr_bias_psd + f->Qxx_noise_diag[INS_IDX_GYR + i],
                        dt_frozen_sec, gyr_bias_cap);
    }

    if (f->n > INS_IDX_MAG)
    {
        const float mag_bias_cap = qsquare(f->init.mag_bias_init_stddev_ut);
        for (i = 0; i < 3; ++i)
        {
            ins_inflate_one(&diag[INS_IDX_MAG + i], f->Qxx_noise_diag[INS_IDX_MAG + i],
                            dt_frozen_sec, mag_bias_cap);
        }
    }
}

/* Vertical component of a re-anchor (REQ-NAV-066). Under the barometric height
 * source the position fix does not describe the height the filter uses: its
 * vertical row is never fused (REQ-NAV-055), so re-anchoring on it would splice
 * two independently drifting height estimates together. The height comes from
 * the last plausible barometer sample instead, through the same
 * datum-referenced conversion the fusion uses. Falls through to the caller's
 * fix-derived values under a GNSS height source or with no fresh sample cached.
 * pos_new[2]/pos_var[2] are updated in place. */
/* @satisfies REQ-NAV-066 */
static void ins_reacquire_vertical(const ins_t* f, ins_time_us_t t, float pos_new[3],
                                   float pos_var[3])
{
    if (!f->height_from_baro || !f->last_baro.valid) return;

    const float age_sec = time_diff_sec(t, f->last_baro.t);
    if (age_sec < 0.0f || age_sec > INS_BARO_ANCHOR_MAX_AGE_SEC)
    {
        LOG_WARN("ins: re-acquisition falls back to the fix for the height, the cached "
                 "barometer sample is %.1f s old",
                 (double)age_sec);
        return;
    }

    const float h_baro = ins_isa_altitude_from_pressure(f->last_baro.pressure_pa) - f->baro_h0_m;
    const float stddev =
        (f->last_baro.stddev_m > 0.0f) ? f->last_baro.stddev_m : INS_DEFAULT_BARO_STDDEV_M;

    pos_new[2] = -h_baro; /* down-positive state, up-positive height */
    pos_var[2] = qsquare(stddev);
}

/* Re-acquisition after an expired dead-reckoning window ("tunnel exit"): the
 * coasted position/velocity are too far gone to fuse a fix as a residual (and
 * the history anchors are equally stale), so re-anchor the nominal
 * position/velocity directly from the measurement.
 *
 * IMU biases are normally KEPT (best-preserved states during an outage, and
 * re-leveling would need a static phase a tunnel exit does not offer), UNLESS
 * the caller supplies an external attitude/gyro-bias hint (REQ-NAV-048). Their
 * VARIANCES are not: nothing was propagated while the filter was inert
 * (REQ-NAV-064), so the frozen interval is priced in here (REQ-NAV-065).
 *
 * Yaw is the exception: without a hinted heading it is reset to an unknown
 * prior (REQ-NAV-059, ins_reacquire_reset_yaw).
 *
 * @satisfies REQ-NAV-023 REQ-NAV-048 REQ-NAV-059 */
static void ins_reacquire(ins_t* f, const ins_measurements_t* m, const float pos_local_new[3],
                          const double* latlonh_new, /* NULL: derive from the origin */
                          const float*  vel_ned_new, /* NULL: keep value */
                          const float pos_var[3], const float vel_var[3])
{
    int i;

    vec3_copy(pos_local_new, f->state.pos_local);
    if (vel_ned_new != NULL) { vec3_copy(vel_ned_new, f->state.vel_ned); }

    /* Re-derive the absolute anchor from the (unchanged) n-frame origin:
       latlonh = origin + pos_local.
       The vertical row is exact (a sign flip), the horizontal one is a
       tangent-plane mapping evaluated at the origin and therefore only good
       for a short baseline: over d_north x d_east it drops the curvature
       cross term, which is ~ d_north * d_east * tan(lat) / R_earth (hundreds
       of metres once the origin is a hundred kilometres behind). A caller who
       already knows the exact latitude/longitude hands them in and skips it. */
    double        dllh[3];
    const double* origin_llh = f->origin_llh;
    ins_dned_to_dlatlonh(f->state.pos_local, origin_llh[0], origin_llh[2], dllh);
    f->latlonh[0] = (latlonh_new != NULL) ? latlonh_new[0] : (origin_llh[0] + dllh[0]);
    f->latlonh[1] = (latlonh_new != NULL) ? latlonh_new[1] : (origin_llh[1] + dllh[1]);
    f->latlonh[2] = origin_llh[2] + dllh[2];
    ins_refresh_earth_params(f);

    /* Diagonal covariance reset: fresh pos/vel, attitude/bias kept but
       inflated for the interval nothing was propagated over. */
    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    for (i = 0; i < 3; ++i)
    {
        diag[INS_IDX_POS + i] = pos_var[i];
        diag[INS_IDX_VEL + i] = vel_var[i];
    }
    ins_inflate_frozen_states(f, m, ins_dr_frozen_sec(f, m->timestamp), diag);
    udu_set_diag(f->U, f->d, diag, f->n);

    ins_apply_att_hint(f, &m->att_hint); /* REQ-NAV-048 */
    /* No cap: re-acquisition weighs the hint against a converged state,
       not against the cold-start prior (REQ-NAV-067). */
    ins_apply_gyr_bias_hint(f, &m->att_hint, false);
    ins_reacquire_reset_yaw(f, &m->att_hint); /* REQ-NAV-059 */

    /* The history is anchored to the pre-outage trajectory: useless
       (and dangerous) for delayed-measurement fusion now. */
    memset(f->history, 0, sizeof(f->history));
    f->history_index = 0;
    ins_save_state(f, m->timestamp, true);

    f->t_last_pos_aiding = m->timestamp;
    f->diag.n_reacquire++;
    LOG_INFO("ins: position re-acquired after an expired dead-reckoning window (#%u)",
             (unsigned int)f->diag.n_reacquire);
}

/* Local NED position fusion (e.g. lighthouse/UWB/mocap): a direct measurement
 * of pos_local (+ sensor lever arm), no coordinate conversion needed since the
 * measurement lives in the same local n-frame as the position state. Delayed
 * measurements are anchored in the history, like GNSS. */
/* @satisfies REQ-NAV-011 REQ-NAV-024 */
static void ins_fuse_local_pos(ins_t* f, const ins_measurements_t* m)
{
    if (!m->local_pos.is_valid) return;

    /* First fix after an expired coasting window: re-anchor instead of
       fusing a hopeless residual. Body position = sensor - lever arm. */
    if (ins_dr_expired(f, m->timestamp))
    {
        float la_n[3], pos_new[3], pos_var[3], vel_var[3];
        int   i;
        mat3_mul_vec3(f->R_b_to_n, m->local_pos_leverarm_b, la_n);
        for (i = 0; i < 3; ++i)
        {
            pos_new[i]    = m->local_pos.pos_ned[i] - la_n[i];
            const float v = MAT_ELEM(m->local_pos.Qll_ned, i, i, 3, 3);
            pos_var[i]    = (v > 0.0f) ? v : qsquare(f->init.pos_init_stddev_m);
            vel_var[i]    = qsquare(f->init.vel_init_stddev_mps);
        }
        /* REQ-NAV-066: under the barometric height source the height comes
           from the barometer, not from this measurement. */
        ins_reacquire_vertical(f, m->timestamp, pos_new, pos_var);
        ins_reacquire(f, m, pos_new, NULL /* the local frame IS the anchor */, NULL, pos_var,
                      vel_var);
        return;
    }

    const int delay_ms = (m->local_pos_delay_ms > 0) ? m->local_pos_delay_ms : 0;
    if (delay_ms > INS_MAX_DELAY_MS) return;

    /* State at time-of-validity. */
    const float* pos_tov = f->state.pos_local;
    const float* R_tov   = f->R_b_to_n;
    if (delay_ms > 0)
    {
        const ins_history_item_t* h = ins_find_history(f, m->timestamp - delay_ms * INS_US_PER_MS);
        if (h == NULL)
        {
            LOG_WARN("ins: State not found in history.");
            return; /* cannot anchor the residual in time -> skip */
        }
        pos_tov = h->state.pos_local;
        R_tov   = h->R_b_to_n;
    }

    /* Predicted sensor position: body position + lever arm. */
    float la_n[3];
    mat3_mul_vec3(R_tov, m->local_pos_leverarm_b, la_n);

    /* Attitude coupling of the lever arm (Wendel, 2nd ed., eq. 8.62):
       H_pos = [ I3 | 0 | -[l^n]_x | 0 | 0 ], same as the GNSS position
       block. la_n = R_tov * l^b is l^n. */
    float Sla[9];
    ins_cross_matrix(la_n, Sla);

    float z[3];
    float R[3 * 3];
    float Ht[INS_UNKNOWNS_MAX * 3];
    memcpy(R, m->local_pos.Qll_ned, sizeof(R));
    memset(Ht, 0, sizeof(Ht));
    int i, j;
    for (i = 0; i < 3; ++i)
    {
        z[i] = pos_tov[i] + la_n[i] - m->local_pos.pos_ned[i];

        MAT_ELEM(Ht, INS_IDX_POS + i, i, f->n, 3) = 1.0f;
        for (j = 0; j < 3; ++j)
        {
            /* Ht stored transposed: Ht[state, meas] = H[meas, state]. */
            MAT_ELEM(Ht, INS_IDX_RPY + j, i, f->n, 3) = -MAT_ELEM(Sla, i, j, 3, 3);
        }
    }

    if (ins_fuse(f, z, R, Ht, 3, f->chi2_thr_local,
                   0 /* skip outliers: indoor systems glitch (occlusion,
                        reflections) and typically run at high rate, so
                        dropping a sample is cheap */) == 0)
    {
        f->t_last_pos_aiding = m->timestamp;
    }
}

/* Per-axis 1-sigma check on the diagonal of a 3x3 NED covariance against a
 * horizontal (N,E) and a vertical (D) limit. Shared by all three threshold
 * sets (fusion / entry / exit) so they cannot drift apart. A non-positive
 * diagonal entry is a broken covariance and always fails. */
static bool ins_cov_stddev_within(const float Qll_ned[9], float max_hor, float max_ver)
{
    const float qN = MAT_ELEM(Qll_ned, 0, 0, 3, 3);
    const float qE = MAT_ELEM(Qll_ned, 1, 1, 3, 3);
    const float qD = MAT_ELEM(Qll_ned, 2, 2, 3, 3);
    if (qN <= 0.0f || qE <= 0.0f || qD <= 0.0f) return false;
    return qN <= max_hor * max_hor && qE <= max_hor * max_hor && qD <= max_ver * max_ver;
}

static bool ins_cov_is_valid(const float Qll_ned[9])
{
    return MAT_ELEM(Qll_ned, 0, 0, 3, 3) > 0.0f && MAT_ELEM(Qll_ned, 1, 1, 3, 3) > 0.0f &&
           MAT_ELEM(Qll_ned, 2, 2, 3, 3) > 0.0f;
}

/* Effective vertical limit for a GNSS POSITION quality gate (REQ-NAV-057).
 * Under the barometric height source the vertical row of the fix is dropped
 * before fusion (REQ-NAV-055), so grading the fix on an accuracy it no longer
 * contributes would throw away usable horizontal aiding. Velocity gates are
 * untouched. */
static float ins_gnss_pos_vertical_limit(const ins_t* f, float configured)
{
    return f->height_from_baro ? FLT_MAX : configured;
}

/* GNSS fusion thresholds: reject measurements that are too noisy to fuse.
 * The loosest of the three sets - whether the filter may ENTER or must
 * LEAVE the 3D solution is decided separately (REQ-NAV-051/052 below). */
/* @satisfies REQ-NAV-007 REQ-NAV-057 */
static bool ins_gnss_pos_usable(const ins_t* f, const ins_meas_gnss_pos_t* p)
{
    if (!p->is_valid) return false;
    if (!ins_cov_is_valid(p->Qll_ned))
    {
        LOG_WARN("ins: invalid GNSS position covariance matrix");
        return false;
    }
    return ins_cov_stddev_within(
        p->Qll_ned, f->opt.gnss_max_horizontal_pos_stddev_m,
        ins_gnss_pos_vertical_limit(f, f->opt.gnss_max_vertical_pos_stddev_m));
}

static bool ins_gnss_vel_usable(const ins_t* f, const ins_meas_gnss_vel_t* v)
{
    if (!v->is_valid) return false;
    if (!ins_cov_is_valid(v->Qll_ned))
    {
        LOG_WARN("ins: invalid GNSS velocity covariance matrix");
        return false;
    }
    return ins_cov_stddev_within(v->Qll_ned, f->opt.gnss_max_horizontal_vel_stddev_mps,
                                 f->opt.gnss_max_vertical_vel_stddev_mps);
}

/* Marks the axis that tripped a gate in the log lines below, so a
 * "measured/limit" pair that rounds to the same printed digits right at the
 * threshold still says which one is the problem. Log-only. */
#if LOG_LEVEL >= LOG_LEVEL_WARN
static const char* ins_gate_mark(float value, float limit)
{
    return (value > limit) ? " OVER" : "";
}

/* The vertical position gate is disabled (FLT_MAX, see
 * ins_gnss_pos_vertical_limit) whenever baro drives the height channel, and
 * "%.2f" would print its full ~3e38 fixed-point expansion. */
static void ins_fmt_vpos_limit(char* buf, size_t n, float max_vpos)
{
    if (max_vpos >= FLT_MAX) { snprintf(buf, n, "n/a"); }
    else { snprintf(buf, n, "%.2f", (double)max_vpos); }
}
#endif

/* One log line reporting where the GNSS fix stands relative to a threshold. */
static void ins_log_gnss_gate(const char* headline, const ins_measurements_t* m, float max_hpos,
                              float max_vpos, float max_hvel, float max_vvel)
{
#if LOG_LEVEL >= LOG_LEVEL_WARN
    float hpos = 0.0f, vpos = 0.0f, hvel = 0.0f, vvel = 0.0f;

    if (m->gnss_pos.is_valid)
    {
        const float n = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 0, 0, 3, 3));
        const float e = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 1, 1, 3, 3));
        hpos          = (e > n) ? e : n;
        vpos          = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 2, 2, 3, 3));
    }
    if (m->gnss_vel.is_valid)
    {
        const float n = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 0, 0, 3, 3));
        const float e = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 1, 1, 3, 3));
        hvel          = (e > n) ? e : n;
        vvel          = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 2, 2, 3, 3));
    }

    char vpos_limit[16];
    ins_fmt_vpos_limit(vpos_limit, sizeof(vpos_limit), max_vpos);

    if (m->gnss_pos.is_valid && m->gnss_vel.is_valid)
    {
        LOG_WARN("%s: pos hor %.2f/%.2f m%s, ver %.2f/%s m%s, "
                 "vel hor %.3f/%.3f m/s%s, ver %.3f/%.3f m/s%s",
                 headline, (double)hpos, (double)max_hpos, ins_gate_mark(hpos, max_hpos),
                 (double)vpos, vpos_limit, ins_gate_mark(vpos, max_vpos), (double)hvel,
                 (double)max_hvel, ins_gate_mark(hvel, max_hvel), (double)vvel, (double)max_vvel,
                 ins_gate_mark(vvel, max_vvel));
    }
    else if (m->gnss_pos.is_valid)
    {
        LOG_WARN("%s: pos hor %.2f/%.2f m%s, ver %.2f/%s m%s, vel not offered", headline,
                 (double)hpos, (double)max_hpos, ins_gate_mark(hpos, max_hpos), (double)vpos,
                 vpos_limit, ins_gate_mark(vpos, max_vpos));
    }
    else
    {
        LOG_WARN("%s: pos not offered, vel hor %.3f/%.3f m/s%s, ver %.3f/%.3f m/s%s", headline,
                 (double)hvel, (double)max_hvel, ins_gate_mark(hvel, max_hvel), (double)vvel,
                 (double)max_vvel, ins_gate_mark(vvel, max_vvel));
    }
#else
    (void)headline;
    (void)m;
    (void)max_hpos;
    (void)max_vpos;
    (void)max_hvel;
    (void)max_vvel;
#endif
}

/* Entry-quality gate (REQ-NAV-051): may this epoch's aiding count towards
 * entering the full 3D solution? A position fix is mandatory, the velocity only
 * has to pass when the receiver offers one - a position-only receiver must
 * still be able to start. */
static bool ins_gnss_entry_quality_ok(const ins_t* f, const ins_measurements_t* m)
{
    if (m->local_pos.is_valid) return true;
    if (!m->gnss_pos.is_valid || !ins_cov_is_valid(m->gnss_pos.Qll_ned)) return false;
    if (!ins_cov_stddev_within(
            m->gnss_pos.Qll_ned, f->opt.gnss_start_max_horizontal_pos_stddev_m,
            ins_gnss_pos_vertical_limit(f, f->opt.gnss_start_max_vertical_pos_stddev_m)))
    {
        return false;
    }
    if (m->gnss_vel.is_valid)
    {
        if (!ins_cov_is_valid(m->gnss_vel.Qll_ned)) return false;
        if (!ins_cov_stddev_within(m->gnss_vel.Qll_ned,
                                   f->opt.gnss_start_max_horizontal_vel_stddev_mps,
                                   f->opt.gnss_start_max_vertical_vel_stddev_mps))
        {
            return false;
        }
    }
    return true;
}

/* Exit-quality gate (REQ-NAV-052): is this epoch's aiding still good enough to
 * STAY in the 3D solution? Every channel the epoch offers has to pass its stop
 * threshold, since losing either position or velocity accuracy is the
 * degradation this gate exists for. */
static bool ins_gnss_stay_quality_ok(const ins_t* f, const ins_measurements_t* m)
{
    if (m->local_pos.is_valid) return true;
    if (m->gnss_pos.is_valid)
    {
        if (!ins_cov_is_valid(m->gnss_pos.Qll_ned)) return false;
        if (!ins_cov_stddev_within(
                m->gnss_pos.Qll_ned, f->opt.gnss_stop_max_horizontal_pos_stddev_m,
                ins_gnss_pos_vertical_limit(f, f->opt.gnss_stop_max_vertical_pos_stddev_m)))
        {
            return false;
        }
    }
    if (m->gnss_vel.is_valid)
    {
        if (!ins_cov_is_valid(m->gnss_vel.Qll_ned)) return false;
        if (!ins_cov_stddev_within(m->gnss_vel.Qll_ned,
                                   f->opt.gnss_stop_max_horizontal_vel_stddev_mps,
                                   f->opt.gnss_stop_max_vertical_vel_stddev_mps))
        {
            return false;
        }
    }
    return true;
}

/* Track the current run of continuously entry-quality aiding (REQ-NAV-045,
 * REQ-NAV-051). A fix that fails the entry gate, a gap longer than
 * INS_GNSS_INIT_MAX_GAP_SEC, or a backwards timestamp breaks the run; epochs
 * offering no aiding leave it untouched. Runs both while collecting and while
 * the filter is live. */
static void ins_track_gnss_entry_dwell(ins_t* f, const ins_measurements_t* m)
{
    if (!m->gnss_pos.is_valid && !m->gnss_vel.is_valid && !m->local_pos.is_valid) return;

    if (!ins_gnss_entry_quality_ok(f, m))
    {
        /* A fix good enough to fuse but not to enter 3D is worth a note: it
           explains a filter that keeps consuming fixes without ever
           starting. */
#if LOG_LEVEL >= LOG_LEVEL_WARN
        if (ins_gnss_pos_usable(f, &m->gnss_pos))
        {
            const bool  first_warn = (f->log_state.t_last_entry_gate_warn == 0);
            const float since_sec =
                first_warn ? 0.0f
                           : time_diff_sec(m->timestamp, f->log_state.t_last_entry_gate_warn);
            if (first_warn || since_sec >= INS_LOG_ENTRY_GATE_REPEAT_SEC)
            {
                ins_log_gnss_gate(
                    "ins: GNSS fusable but below the 3D entry gate, 3D held off "
                    "(measured/limit)",
                    m, f->opt.gnss_start_max_horizontal_pos_stddev_m,
                    ins_gnss_pos_vertical_limit(f, f->opt.gnss_start_max_vertical_pos_stddev_m),
                    f->opt.gnss_start_max_horizontal_vel_stddev_mps,
                    f->opt.gnss_start_max_vertical_vel_stddev_mps);
                f->log_state.t_last_entry_gate_warn = m->timestamp;
            }
        }
#endif
        f->gnss_dwell_since = 0;
        f->gnss_dwell_count = 0;
        return;
    }

    const ins_time_us_t max_gap =
        (ins_time_us_t)(INS_GNSS_INIT_MAX_GAP_SEC * (float)INS_US_PER_SEC);
    if (f->gnss_dwell_since == 0 || m->timestamp < f->gnss_dwell_last ||
        (m->timestamp - f->gnss_dwell_last) > max_gap)
    {
        f->gnss_dwell_since = m->timestamp;
        f->gnss_dwell_count = 0;
    }
    f->gnss_dwell_last = m->timestamp;
    f->gnss_dwell_count++;
}

/* Has the entry run become both long enough and dense enough (>= 1 fix/s)
 * to admit the 3D solution at time t? (REQ-NAV-045, REQ-NAV-051) */
static bool ins_entry_dwell_satisfied(const ins_t* f, ins_time_us_t t)
{
    if (f->opt.gnss_init_dwell_disable) return true;
    if (f->gnss_dwell_since == 0) return false;
    const float elapsed_s = (float)(t - f->gnss_dwell_since) * 1e-6f;
    const int   min_count = (int)(f->opt.gnss_init_dwell_sec * INS_GNSS_INIT_MIN_RATE_HZ + 0.5f);
    return elapsed_s >= f->opt.gnss_init_dwell_sec && f->gnss_dwell_count >= min_count;
}

static void ins_fail_gnss_quality(ins_t* f, ins_time_us_t t);

/* Mode arbitration of the running 3D solution on GNSS quality (REQ-NAV-052).
 * Once every fix over gnss_stop_dwell_sec has failed the stop gate, the filter
 * re-arms into the collecting state: it stops integrating and fusing and
 * reports nothing. Only epochs that actually offered aiding are used, so a
 * plain GNSS outage stays the business of max_deadreckoning_sec. */
/* @satisfies REQ-NAV-051 REQ-NAV-052 */
static void ins_track_gnss_mode_gates(ins_t* f, const ins_measurements_t* m)
{
    const bool offered = m->gnss_pos.is_valid || m->gnss_vel.is_valid || m->local_pos.is_valid;

    if (!f->gnss_quality_ok)
    {
        /* Held down after a quality loss: re-enter on the entry dwell. Only
           tracked here, not while already in the 3D solution: the dwell state
           is never read outside this branch, and evaluating it unconditionally
           logged "3D held off" against the stricter entry gate while the
           looser stay gate was keeping the 3D solution up. */
        ins_track_gnss_entry_dwell(f, m);
        if (ins_entry_dwell_satisfied(f, m->timestamp))
        {
            f->gnss_quality_ok    = true;
            f->gnss_bad_since     = 0;
            f->gnss_bad_last      = 0;
            f->gnss_bad_accum_sec = 0.0f;
            LOG_INFO("ins: GNSS quality recovered, 3D solution re-entered");
        }
        return;
    }

    if (f->opt.gnss_stop_disable || !offered) return;

    if (ins_gnss_stay_quality_ok(f, m))
    {
        f->gnss_bad_since     = 0;
        f->gnss_bad_last      = 0;
        f->gnss_bad_accum_sec = 0.0f;
        return;
    }

    if (f->gnss_bad_since == 0 || m->timestamp < f->gnss_bad_since)
    {
        f->gnss_bad_since     = m->timestamp;
        f->gnss_bad_last      = m->timestamp;
        f->gnss_bad_accum_sec = 0.0f;
        return;
    }

    /* Accumulate over the epochs that actually offered aiding, not off the
       wall clock: a wall-clock difference lets an OUTAGE fill the dwell, so
       one bad fix before a tunnel and the first bad fix out of it would give
       up a solution that coasted the whole way through. A single step
       contributes at most INS_GNSS_STOP_MAX_STEP_SEC, which is what keeps a
       gap from counting while still not breaking a run of bad fixes at the
       gaps between them (REQ-NAV-052). */
    {
        const float step = time_diff_sec(m->timestamp, f->gnss_bad_last);
        f->gnss_bad_accum_sec +=
            (step < INS_GNSS_STOP_MAX_STEP_SEC) ? step : INS_GNSS_STOP_MAX_STEP_SEC;
        f->gnss_bad_last = m->timestamp;
    }

    const float bad_sec   = f->gnss_bad_accum_sec;
    const float dwell_sec = (f->opt.gnss_stop_dwell_sec > 0.0f) ? f->opt.gnss_stop_dwell_sec
                                                                : INS_DEFAULT_GNSS_STOP_DWELL_SEC;
    if (bad_sec >= dwell_sec)
    {
        f->diag.n_gnss_quality_exit++;
        LOG_WARN("ins: GNSS quality below the 3D exit gate for %.1f s, leaving the 3D solution",
                 (double)bad_sec);
        /* Same measured/limit breakdown as the entry gate: which axis gave up
           is the first question once a vehicle drops out of 3D. */
        ins_log_gnss_gate(
            "ins: 3D exit gate (measured/limit)", m, f->opt.gnss_stop_max_horizontal_pos_stddev_m,
            ins_gnss_pos_vertical_limit(f, f->opt.gnss_stop_max_vertical_pos_stddev_m),
            f->opt.gnss_stop_max_horizontal_vel_stddev_mps,
            f->opt.gnss_stop_max_vertical_vel_stddev_mps);
        ins_fail_gnss_quality(f, m->timestamp);
    }
}

/* GNSS position + velocity fusion (with lever arm and measurement delay).
 *
 * The residual is anchored at the state the filter had at the measurement's
 * time-of-validity (from the history ring buffer), the covariance update is
 * applied to the *current* U,d. A good approximation for delayed measurements:
 * the error-state is nearly constant over the delay (Phi ~ I for a few 100 ms). */
/* @satisfies REQ-NAV-005 REQ-NAV-006 REQ-NAV-008 REQ-NAV-023 REQ-NAV-024 REQ-NAV-055
 * @satisfies REQ-NAV-079 */
static void ins_fuse_gnss(ins_t* f, const ins_measurements_t* m)
{
    const bool offered = m->gnss_pos.is_valid || m->gnss_vel.is_valid;
    if (offered) f->diag.n_gnss_seen++;

    /* GNSS-outage diagnostics, gated on t_last_gnss_fusion != 0: a filter
       intentionally run without GNSS must not get a repeating "outage" warning
       for a channel it never asked for. Doubles as the internal
       "entering/leaving coasting" signal for a standalone caller. */
    if (f->diag.t_last_gnss_fusion != 0)
    {
        const float gap_sec = time_diff_sec(m->timestamp, f->diag.t_last_gnss_fusion);
        if (gap_sec >= INS_LOG_GNSS_OUTAGE_WARN_SEC)
        {
            const bool  first_warn = (f->log_state.t_last_gnss_outage_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f
                           : time_diff_sec(m->timestamp, f->log_state.t_last_gnss_outage_warn);
            if (first_warn || since_warn_sec >= INS_LOG_GNSS_OUTAGE_REPEAT_SEC)
            {
                LOG_WARN("ins: no GNSS fix for %.1f s (%s)", (double)gap_sec,
                         ins_dr_expired(f, m->timestamp) ? "coasting window expired, frozen"
                                                         : "coasting on dead reckoning");
                f->log_state.t_last_gnss_outage_warn = m->timestamp;
            }
        }
        else if (f->log_state.t_last_gnss_outage_warn != 0)
        {
            LOG_INFO("ins: GNSS fix reacquired after a %.1f s outage", (double)gap_sec);
            f->log_state.t_last_gnss_outage_warn = 0;
        }
    }

    bool use_pos = ins_gnss_pos_usable(f, &m->gnss_pos);
    bool use_vel = ins_gnss_vel_usable(f, &m->gnss_vel);

    /* Position aiding (REQ-NAV-023) is decided by the fusion check alone,
       before the decimation below may withhold this epoch's position: a fix
       that carried a usable position keeps the coasting window open whether or
       not the filter chose to spend it. */
    const bool pos_aiding = use_pos;

    /* GNSS fusion rate limit (REQ-NAV-074). A fix arriving less than
       gnss_min_delay_ms after the last one this filter actually spent is
       skipped whole: both blocks, not just the position the decimation thins.
       Suppressed while the coasting window is expired, where the next usable
       fix is a re-anchor. Like the decimation it leaves the coasting window
       alone: the fix carried a usable position, the filter merely chose not to
       spend it (REQ-NAV-023).
       @satisfies REQ-NAV-074 */
    if (f->opt.gnss_min_delay_ms > 0 && (use_pos || use_vel) && f->t_last_gnss_fused != 0 &&
        !ins_dr_expired(f, m->timestamp) &&
        time_diff_ms(m->timestamp, f->t_last_gnss_fused) < f->opt.gnss_min_delay_ms)
    {
        if (pos_aiding) { f->t_last_pos_aiding = m->timestamp; }
        f->diag.n_gnss_rate_limited++;
        return;
    }

    /* GNSS position decimation. Only bites when the same epoch offers both pos
       and vel, where the unknown position/velocity cross-covariance would make
       the combined fuse count one solution twice.
       @satisfies REQ-NAV-063 */
    if (use_pos && use_vel && f->opt.gnss_pos_decimation > 1 && !ins_dr_expired(f, m->timestamp))
    {
        /* An interrupted position-aiding stream restarts the cycle at a position */
        if (time_diff_sec(m->timestamp, f->t_last_pos_aiding) > INS_GNSS_INIT_MAX_GAP_SEC)
        {
            f->gnss_pos_decim_count = 0;
        }

        if (f->gnss_pos_decim_count == 0) { use_vel = false; }
        else { use_pos = false; }

        f->gnss_pos_decim_count++;
        if (f->gnss_pos_decim_count >= (uint32_t)f->opt.gnss_pos_decimation)
        {
            f->gnss_pos_decim_count = 0;
        }
    }

    if (!use_pos && !use_vel)
    {
        /* Offered but blocked by the stddev shutdown gate. */
        if (offered) f->diag.n_gnss_rejected_noise++;
        return;
    }

    /* First usable fix after an expired coasting window (tunnel exit):
       re-anchor position/velocity instead of fusing. The measurement delay is
       ignored here, being bounded by INS_MAX_DELAY_MS and far below the
       re-acquisition position uncertainty. */
    if (use_pos && ins_dr_expired(f, m->timestamp))
    {
        /* Where the fix sits in the n-frame: as a step from the position the
           filter is holding, NOT as a difference against the origin. Both
           reach the same place while the origin is near, but only this one
           keeps the step short once it is not, and the tangent-plane mapping
           is only exact for a short step (see ins_reacquire). The absolute
           anchor is the fix itself and never runs through the mapping. */
        double       dllh[3];
        const double meas_llh[3] = {m->gnss_pos.llh[0], m->gnss_pos.llh[1], m->gnss_pos.llh[2]};
        dllh[0]                  = meas_llh[0] - f->latlonh[0];
        dllh[1]                  = meas_llh[1] - f->latlonh[1];
        dllh[2]                  = meas_llh[2] - f->latlonh[2];

        float pos_new[3], la_n[3], pos_var[3], vel_var[3];
        ins_dlatlonh_to_dned(dllh, f->latlonh[0], f->latlonh[2], pos_new);
        mat3_mul_vec3(f->R_b_to_n, m->gnss_leverarm_b, la_n);

        /* The fix locates the ANTENNA, the state the body: step the absolute
           anchor back along the lever arm as well, so it keeps describing the
           same point as pos_new below. Metre-scale, so exact. */
        double      anchor_llh[3], dllh_la[3];
        const float la_n_neg[3] = {-la_n[0], -la_n[1], -la_n[2]};
        ins_dned_to_dlatlonh(la_n_neg, meas_llh[0], meas_llh[2], dllh_la);
        anchor_llh[0] = meas_llh[0] + dllh_la[0];
        anchor_llh[1] = meas_llh[1] + dllh_la[1];
        anchor_llh[2] = meas_llh[2] + dllh_la[2];

        int i;
        for (i = 0; i < 3; ++i)
        {
            pos_new[i] += f->state.pos_local[i];
            pos_new[i] -= la_n[i];
            /* Re-anchoring seeds the state covariance from the fix, so it is
               a fusion weight and takes the conditioned covariance, not the
               reported one. usable gates guarantee positive variances. */
            pos_var[i] = MAT_ELEM(f->step_ctx.gnss_pos_Qll_fuse, i, i, 3, 3);
            vel_var[i] = use_vel ? MAT_ELEM(f->step_ctx.gnss_vel_Qll_fuse, i, i, 3, 3)
                                 : qsquare(f->init.vel_init_stddev_mps);
        }
        /* REQ-NAV-054/055/066: height is barometric, never re-anchored
           from the GNSS position whose vertical row this filter does not
           even fuse. */
        ins_reacquire_vertical(f, m->timestamp, pos_new, pos_var);
        /* Lever-arm velocity (omega x lever arm) is ignored here: cm/s
           against a fresh, metre-scale velocity uncertainty. */
        ins_reacquire(f, m, pos_new, anchor_llh, use_vel ? m->gnss_vel.vel_ned : NULL, pos_var,
                      vel_var);
        f->diag.n_gnss_used++;
        f->diag.t_last_gnss_fusion = m->timestamp;
        f->t_last_gnss_fused       = m->timestamp;
        return;
    }

    int delay_ms = 0;
    if (m->gnss_delay_ms > 0) { delay_ms = m->gnss_delay_ms; }
    if (!f->log_state.gnss_delay_logged)
    {
        f->log_state.gnss_delay_logged = true;
        LOG_INFO("ins: first GNSS fix carries a %d ms measurement delay "
                 "(anchored via the history buffer)",
                 delay_ms);
    }
    if (delay_ms > INS_MAX_DELAY_MS)
    {
        f->diag.n_gnss_no_anchor++; /* too old to place in the history */
        return;
    }

    /* State at time-of-validity (fall back to the current state for
       delay 0). */
    const double* latlonh_tov = f->latlonh;
    const float*  vel_tov     = f->state.vel_ned;
    const float*  R_tov       = f->R_b_to_n;
    const float*  omg_tov     = f->last_omega_b_nb;
    if (delay_ms > 0)
    {
        const ins_history_item_t* h = ins_find_history(f, m->timestamp - delay_ms * INS_US_PER_MS);
        if (h == NULL)
        {
            LOG_WARN("ins: State not found in history.");
            f->diag.n_gnss_no_anchor++;
            return; /* cannot anchor the residual in time -> skip */
        }
        latlonh_tov = h->latlonh;
        vel_tov     = h->state.vel_ned;
        R_tov       = h->R_b_to_n;
        omg_tov     = h->omega_b_nb;
    }

    /* Assemble the combined measurement: z (mm x 1), full R (mm x mm) and Ht
       (n x mm). R is built from the 3x3 NED blocks, plus the optional
       cross-covariance block when both position and velocity are fused.

       pos_rows (REQ-NAV-055): with the barometric height source active
       (REQ-NAV-053) the vertical row of the GNSS position is dropped -
       ins_fuse_baro_height supplies it instead - so the position block is 2
       rows (N, E). GNSS velocity stays a full 3-row block. */
    const int pos_rows = (use_pos && f->height_from_baro) ? 2 : 3;
    const int mm       = (use_pos ? pos_rows : 0) + (use_vel ? 3 : 0);
    float     z[INS_FUSE_MAX_MEAS];
    float     R[INS_FUSE_MAX_MEAS * INS_FUSE_MAX_MEAS];
    float     Ht[INS_UNKNOWNS_MAX * INS_FUSE_MAX_MEAS];
    memset(R, 0, sizeof(R[0]) * (size_t)(mm * mm));
    memset(Ht, 0, sizeof(Ht));
    int row = 0;
    int i, j;

    if (use_pos)
    {
        /* The fix in the geodetic form the residual below is built from,
           which is the form it arrived in (REQ-NAV-079): nothing is
           converted here, so the epoch path carries no coordinate
           transformation at all. The doubles below are the geodetic
           difference itself, not a conversion. */
        const double meas_llh[3] = {m->gnss_pos.llh[0], m->gnss_pos.llh[1], m->gnss_pos.llh[2]};

        /* Residual (predicted antenna - measured) in the n-frame: from the
           lat/lon/h difference (meter scale, so the small-angle mapping is
           exact for practical purposes) plus the antenna lever arm at the
           time-of-validity attitude. */
        const double dllh[3] = {latlonh_tov[0] - meas_llh[0], latlonh_tov[1] - meas_llh[1],
                                latlonh_tov[2] - meas_llh[2]};
        float        dz_n[3];
        ins_dlatlonh_to_dned(dllh, latlonh_tov[0], latlonh_tov[2], dz_n);

        float la_n[3];
        mat3_mul_vec3(R_tov, m->gnss_leverarm_b, la_n);

        /* Attitude coupling of the lever arm (Wendel, 2nd ed., eq. 8.62): the
           antenna moves with the body's attitude error, so
           H_pos = [ I3 | 0 | -[l^n]_x | 0 | 0 ]. Without that block the filter
           would blame a lever-arm-induced residual on the body position alone.
           la_n = R_tov * l^b is l^n. */
        float Sla[9];
        ins_cross_matrix(la_n, Sla);

        for (i = 0; i < pos_rows; ++i)
        {
            z[row + i]                                       = dz_n[i] + la_n[i];
            MAT_ELEM(Ht, INS_IDX_POS + i, row + i, f->n, mm) = 1.0f;
            /* Attitude coupling uses the full 3x3 Sla regardless of pos_rows:
               dropping the vertical row does not stop the remaining (N, E) rows
               observing attitude through the lever arm. */
            for (j = 0; j < 3; ++j)
            {
                /* Ht is stored transposed: Ht[state, meas] = H[meas, state]. */
                MAT_ELEM(Ht, INS_IDX_RPY + j, row + i, f->n, mm) = -MAT_ELEM(Sla, i, j, 3, 3);
            }
            /* R, unlike Sla above, is only pos_rows x pos_rows here (its
               vertical row/column would not correspond to anything fused
               when pos_rows == 2). */
            for (j = 0; j < pos_rows; ++j)
            {
                MAT_ELEM(R, row + i, row + j, mm, mm) =
                    MAT_ELEM(f->step_ctx.gnss_pos_Qll_fuse, i, j, 3, 3);
            }
        }
        row += pos_rows;
    }

    if (use_vel)
    {
        /* Predicted antenna velocity: v_n + R * (omega_b_nb x leverarm). */
        float wxla_b[3], v_la_n[3];
        ins_cross(omg_tov, m->gnss_leverarm_b, wxla_b);
        mat3_mul_vec3(R_tov, wxla_b, v_la_n);

        /* Attitude coupling of the lever-arm velocity (Wendel, 2nd ed.,
           eq. 8.74): H_vel = [ 0 | I3 | -[R*Omega_b_ib*l^b]_x | 0 | 0 ], where
           v_la_n = R*(omega x l^b). omega_b_nb is used instead of omega_b_ib:
           the difference is ~0.1 mm/s for a 1 m arm. */
        float Svla[9];
        ins_cross_matrix(v_la_n, Svla);

        for (i = 0; i < 3; ++i)
        {
            z[row + i] = vel_tov[i] + v_la_n[i] - m->gnss_vel.vel_ned[i];
            MAT_ELEM(Ht, INS_IDX_VEL + i, row + i, f->n, mm) = 1.0f;
            for (j = 0; j < 3; ++j)
            {
                /* Ht stored transposed: Ht[state, meas] = H[meas, state]. */
                MAT_ELEM(Ht, INS_IDX_RPY + j, row + i, f->n, mm) = -MAT_ELEM(Svla, i, j, 3, 3);
                MAT_ELEM(R, row + i, row + j, mm, mm) =
                    MAT_ELEM(f->step_ctx.gnss_vel_Qll_fuse, i, j, 3, 3);
            }
        }
        /* Position/velocity cross-covariance (only present when the position
           block is also fused). The velocity side spans all 3 rows, the
           position side is bounded by pos_rows (REQ-NAV-055). */
        if (use_pos)
        {
            for (i = 0; i < pos_rows; ++i)
            {
                for (j = 0; j < 3; ++j)
                {
                    const float c = MAT_ELEM(f->step_ctx.gnss_pos_vel_Qll_fuse, i, j, 3, 3);
                    MAT_ELEM(R, i, row + j, mm, mm) = c;
                    MAT_ELEM(R, row + j, i, mm, mm) = c;
                }
            }
        }
    }

    /* Residual bookkeeping: must run before ins_fuse(),
     * which may decorrelate z in place. */
    if (use_pos)
    {
        float rp_sq = 0.0f;
        for (i = 0; i < pos_rows; ++i) { rp_sq += z[i] * z[i]; }
        const float rp = SQRTF(rp_sq);

        f->diag.last_gnss_pos_residual_m = rp;
        if (rp > f->diag.max_gnss_pos_residual_m) { f->diag.max_gnss_pos_residual_m = rp; }
        if (rp > INS_LOG_LARGE_POS_JUMP_M)
        {
            const bool  first_log = (f->log_state.t_last_large_pos_res_log == 0);
            const float since_sec =
                first_log ? 0.0f
                          : time_diff_sec(m->timestamp, f->log_state.t_last_large_pos_res_log);
            if (first_log || since_sec >= INS_LOG_LARGE_POS_REPEAT_SEC)
            {
                if (pos_rows > 2)
                {
                    LOG_INFO("large GNSS position correction %.1f m pending "
                             "(N=%.1f E=%.1f D=%.1f)",
                             (double)rp, (double)z[0], (double)z[1], (double)z[2]);
                }
                else
                {
                    LOG_INFO("large GNSS pos. correction %.1f m pending "
                             "(N=%.1f E=%.1f, height is barometric)",
                             (double)rp, (double)z[0], (double)z[1]);
                }
                f->log_state.t_last_large_pos_res_log = m->timestamp;
            }
        }
        /* Coarse "outlier" proxy: per-axis residual vs. the measurement noise
           only (ignores the state covariance, so not the chi2 test). Graded
           against the conditioned covariance so it agrees with the real gate. */
        bool large = false;
        for (i = 0; i < pos_rows; ++i)
        {
            const float var = MAT_ELEM(f->step_ctx.gnss_pos_Qll_fuse, i, i, 3, 3);
            if (z[i] * z[i] > 9.0f * var) large = true; /* > 3 sigma */
        }
        if (large) f->diag.n_gnss_large_residual++;
    }
    if (use_vel)
    {
        const int vr = use_pos ? pos_rows : 0;
        f->diag.last_gnss_vel_residual_mps =
            SQRTF(z[vr] * z[vr] + z[vr + 1] * z[vr + 1] + z[vr + 2] * z[vr + 2]);
    }

    if (ins_fuse(f, z, R, Ht, mm, f->chi2_thr_gnss, 1 /* downweight outliers */) == 0)
    {
        f->diag.n_gnss_used++;
        f->diag.t_last_gnss_fusion = m->timestamp;
        f->t_last_gnss_fused       = m->timestamp;
        /* REQ-NAV-023: only a fix that actually carried a usable POSITION
           may un-expire the coasting window (REQ-NAV-063). */
        if (pos_aiding) { f->t_last_pos_aiding = m->timestamp; }
    }
}

/* Barometric height (REQ-NAV-054): fuses the datum-referenced barometric
 * altitude directly as a scalar measurement of the vertical position state, on
 * the anchor f->baro_h0_m established at bootstrap (REQ-NAV-053) and kept
 * consistent across an origin relocation (ins_shift_origin_down). Like every
 * other fusion it only runs while the filter is live: an expired coasting
 * window makes the instance inert (REQ-NAV-064) and the height is picked back
 * up from the barometer at the re-anchor (REQ-NAV-066).
 *
 * h = -pos_local[2] (down-positive state, up-positive height), so the residual
 * and its Jacobian carry a sign flip: */
/* @satisfies REQ-NAV-054 REQ-NAV-058 */
static void ins_fuse_baro_height(ins_t* f, const ins_measurements_t* m)
{
    if (!f->height_from_baro) return;

    /* Aiding-gap diagnostic (REQ-NAV-058). The height source is
       latched for this filter instance (REQ-NAV-053), so a barometer that
       stops is not a source change. */
    const bool usable = m->baro.is_valid && ins_isa_pressure_plausible(m->baro.pressure_pa);
    if (f->log_state.t_last_baro_height_aid != 0)
    {
        const float gap_sec = time_diff_sec(m->timestamp, f->log_state.t_last_baro_height_aid);
        if (gap_sec >= INS_LOG_BARO_HEIGHT_GAP_WARN_SEC)
        {
            const bool  first_warn = (f->log_state.t_last_baro_height_warn == 0);
            const float since_sec =
                first_warn ? 0.0f
                           : time_diff_sec(m->timestamp, f->log_state.t_last_baro_height_warn);
            if (first_warn || since_sec >= INS_LOG_BARO_HEIGHT_GAP_REPEAT_SEC)
            {
                float diag[INS_UNKNOWNS_MAX];
                udu_get_diag(f->U, f->d, diag, f->n);
                LOG_WARN("ins: no barometric height aiding for %.1f s - the vertical channel "
                         "has no absolute reference (height 1-sigma %.1f m and growing)",
                         (double)gap_sec, (double)SQRTF(diag[INS_IDX_POS + 2]));
                f->log_state.t_last_baro_height_warn = m->timestamp;
            }
        }
        else if (usable && f->log_state.t_last_baro_height_warn != 0)
        {
            /* No duration here: by the time gap_sec is back under the
               threshold the first post-gap sample has already been fused, so
               it would only ever print ~0. */
            LOG_INFO("ins: barometric height aiding resumed");
            f->log_state.t_last_baro_height_warn = 0;
        }
    }
    if (!usable) return;

    const float h_isa  = ins_isa_altitude_from_pressure(m->baro.pressure_pa);
    float       dz     = -f->state.pos_local[2] - (h_isa - f->baro_h0_m);
    const float stddev = (m->baro.stddev_m > 0.0f) ? m->baro.stddev_m : INS_DEFAULT_BARO_STDDEV_M;
    float       R      = qsquare(stddev);

    float Ht[INS_UNKNOWNS_MAX];
    memset(Ht, 0, sizeof(Ht));
    Ht[INS_IDX_POS + 2] = -1.0f;

    if (ins_fuse(f, &dz, &R, Ht, 1, f->chi2_thr_gnss, 1 /* downweight outliers (REQ-SYS-006) */) ==
        0)
    {
        f->diag.n_baro_height_used++;
        f->log_state.t_last_baro_height_aid = m->timestamp;
    }
}

/* Absolute speed (REQ-NAV-068): a scalar measurement of the velocity
 * MAGNITUDE, e.g. an OBD-II vehicle speed or a wheel-odometry rate.
 *
 *     h(x) = ||v_n||        H = [ 0 | v_hat' | 0 | 0 | 0 ]
 *
 * v_hat' picks out exactly the component of the velocity error ALONG the
 * current direction of travel; cross-track error is invisible to it, the
 * honest limit of a measurement carrying no direction.
 *
 * The gate is on the FILTER's speed, not the reported one: the Jacobian is
 * built from the state. A standing platform is the auto-ZUPT's job
 * (REQ-NAV-013). */
/* @satisfies REQ-NAV-068 */
static void ins_fuse_speed(ins_t* f, const ins_measurements_t* m)
{
    if (!m->speed.is_valid) { return; }

    /* Unsigned by contract: a negative reading is a unit or sign error,
       not a reversing platform (REQ-NAV-018 boundary policy). */
    if (!isfinite(m->speed.speed_mps) || m->speed.speed_mps < 0.0f)
    {
        f->diag.n_invalid_input++;
        return;
    }
    f->diag.n_speed_seen++;

    const float scale = (f->opt.speed_scale > 0.0f) ? f->opt.speed_scale : 1.0f;
    const float z     = m->speed.speed_mps * scale;

    /* Velocity at the time of validity. A Bluetooth OBD-II round-trip is
       tens of ms, so the delayed case is the normal one here. */
    const int delay_ms = (m->speed_delay_ms > 0) ? m->speed_delay_ms : 0;
    if (delay_ms > INS_MAX_DELAY_MS)
    {
        f->diag.n_speed_skipped++;
        return;
    }
    const float* vel_tov = f->state.vel_ned;
    if (delay_ms > 0)
    {
        const ins_history_item_t* h =
            ins_find_history(f, m->timestamp - (ins_time_us_t)delay_ms * INS_US_PER_MS);
        if (h == NULL)
        {
            f->diag.n_speed_skipped++;
            return;
        }
        vel_tov = h->state.vel_ned;
    }

    const float sp =
        SQRTF(vel_tov[0] * vel_tov[0] + vel_tov[1] * vel_tov[1] + vel_tov[2] * vel_tov[2]);
    const float min_speed =
        (f->opt.speed_min_mps > 0.0f) ? f->opt.speed_min_mps : INS_DEFAULT_SPEED_MIN_MPS;
    if (sp < min_speed)
    {
        f->diag.n_speed_skipped++;
        return;
    }

    /* Two-part noise: the sample's own uncertainty plus the installation's
       residual scale error, which does grow with speed (see
       ins_options_t.speed_stddev_rel). */
    const float sd_abs =
        (m->speed.stddev_mps > 0.0f) ? m->speed.stddev_mps : INS_DEFAULT_SPEED_STDDEV_MPS;
    const float sd_rel =
        (f->opt.speed_stddev_rel > 0.0f) ? f->opt.speed_stddev_rel : INS_DEFAULT_SPEED_STDDEV_REL;
    float R = qsquare(sd_abs) + qsquare(sd_rel * z);

    float dz = sp - z;

    float Ht[INS_UNKNOWNS_MAX];
    memset(Ht, 0, sizeof(Ht));
    const float inv_sp = 1.0f / sp;
    int         i;
    for (i = 0; i < 3; ++i) { Ht[INS_IDX_VEL + i] = vel_tov[i] * inv_sp; }

    f->diag.last_speed_residual_mps = dz;

    if (ins_fuse(f, &dz, &R, Ht, 1, f->chi2_thr_gnss, 1 /* downweight outliers (REQ-SYS-006) */) ==
        0)
    {
        f->diag.n_speed_used++;
    }
}

/* Automotive mode: derive a yaw (heading) measurement from the GNSS velocity
 * vector. The course over ground c = atan2(vE, vN) equals the vehicle heading
 * under the non-holonomic assumption that it travels in the direction it
 * points. Only armed above INS_AUTOMOTIVE_MIN_SPEED_MPS, below which the
 * course is noisy and the assumption can break. The heading uncertainty is
 * sigma_v / speed, floored at INS_AUTOMOTIVE_MIN_YAW_STDDEV. Fused on yaw only,
 * chi2-downweighted so a transient reversing outlier gets a near-zero
 * per-epoch gain; SUSTAINED reversing can still drag yaw (REQ-NAV-034 assumes
 * predominantly forward travel). Shares the GNSS delay. */
/* @satisfies REQ-NAV-034 */
static void ins_fuse_gnss_course_yaw(ins_t* f, const ins_measurements_t* m)
{
    if (!f->opt.automotive_mode) return;
    /* No usable GNSS velocity this epoch. Nothing to fuse, nothing to say. */
    if (!ins_gnss_vel_usable(f, &m->gnss_vel)) { return; }

    const float min_speed = (f->opt.automotive_min_speed_mps > 0.0f)
                                ? f->opt.automotive_min_speed_mps
                                : INS_AUTOMOTIVE_MIN_SPEED_MPS;
    const float vN        = m->gnss_vel.vel_ned[0];
    const float vE        = m->gnss_vel.vel_ned[1];
    const float speed     = SQRTF(vN * vN + vE * vE);
    if (speed < min_speed) { return; }

    /* Representative horizontal velocity stddev (usable-gate guarantees > 0).
       This becomes the yaw measurement's noise, so it is a fusion weight and
       reads the conditioned covariance. */
    const float qN             = MAT_ELEM(f->step_ctx.gnss_vel_Qll_fuse, 0, 0, 3, 3);
    const float qE             = MAT_ELEM(f->step_ctx.gnss_vel_Qll_fuse, 1, 1, 3, 3);
    const float sigma_v        = SQRTF(0.5f * (qN + qE));
    const float min_yaw_stddev = (f->opt.automotive_min_yaw_stddev > 0.0f)
                                     ? f->opt.automotive_min_yaw_stddev
                                     : INS_AUTOMOTIVE_MIN_YAW_STDDEV;
    float       stddev         = sigma_v / speed;
    if (stddev < min_yaw_stddev) { stddev = min_yaw_stddev; }

    /* Attitude at time-of-validity */
    const int delay_ms = (m->gnss_delay_ms > 0) ? m->gnss_delay_ms : 0;
    if (delay_ms > INS_MAX_DELAY_MS) { return; }
    const float* R_tov = f->R_b_to_n;
    if (delay_ms > 0)
    {
        const ins_history_item_t* h = ins_find_history(f, m->timestamp - delay_ms * INS_US_PER_MS);
        if (h == NULL)
        {
            LOG_WARN("ins: State not found in history.");
            return;
        }
        R_tov = h->R_b_to_n;
    }

    /* Yaw is ill-defined near pitch = +/-90 deg (gimbal lock) -> skip. */
    const float sp = -MAT_ELEM(R_tov, 2, 0, 3, 3);
    if (sp > 0.99f || sp < -0.99f) { return; }

    float roll, pitch, yaw_nom;
    ins_rotmat_to_rpy(R_tov, &roll, &pitch, &yaw_nom);

    const float course = atan2f(vE, vN);
    const float dyaw   = ins_wrap_pi_bounded(yaw_nom - course);
    if (ins_fuse_yaw_residual(f, dyaw, qsquare(stddev), f->chi2_thr_yaw) == 0)
    {
        f->log_state.t_last_yaw_aid = m->timestamp;
        if (!f->log_state.automotive_logged)
        {
            f->log_state.automotive_logged = true;
            LOG_INFO("ins: automotive mode: yaw is being aided from the GNSS course "
                     "(first fusion at %.1f m/s, pauses below %.1f m/s from here on "
                     "without further notice)",
                     (double)speed, (double)min_speed);
        }
    }
}

/* Non-holonomic lateral velocity constraint: a wheeled vehicle does not
 * travel sideways, so the body-frame lateral velocity is a measurement of
 * zero. Unlike the course-over-ground aiding above this needs no GNSS, which
 * is the whole point -- it is the only thing holding the lateral channel
 * during an outage.
 *
 * Residual and rows, with the psi-angle convention of this filter
 * (R_nominal = (I + [psi x]) R_true, see ins_quat_small_angle_correction):
 *
 *   v_b     = M v_n,  M = R_b_to_n^T
 *   dv_b    = M dv + M [v_n x] psi
 *
 * so row y of M sits on the velocity states and row y of M [v_n x] on the
 * attitude states. The second is what earns the constraint its keep: its
 * gain is the ground speed, so at road speed a fraction of a degree of roll
 * error is a measurable lateral velocity. Using the identity
 * M [v_n x] == [v_b x] M, row y of it is (v_b_z, 0, -v_b_x) M, which needs
 * no 3x3 product.
 *
 * Only the lateral row exists, see REQ-NAV-077 for why the vertical one is
 * not a free addition. */
/* @satisfies REQ-NAV-077 */
static void ins_fuse_lateral_constraint(ins_t* f, const ins_measurements_t* m)
{
    if (!f->opt.automotive_lateral_constraint || !f->opt.automotive_mode) return;

    /* Held off until the filter has actually been coasting: next to a live
       GNSS velocity the constraint adds no information, only the risk that a
       mounting misalignment reaches the state. */
    const float cfg_after = f->opt.automotive_lateral_after_sec;
    /* positive -> as given, negative -> no delay, zero -> default. Spelled
       without an equality test on a float. */
    const float after_sec =
        (cfg_after > 0.0f) ? cfg_after : ((cfg_after < 0.0f) ? cfg_after : INS_NHC_AFTER_SEC);
    if (after_sec > 0.0f)
    {
        if (f->diag.t_last_gnss_fusion == 0) return;
        if (time_diff_sec(m->timestamp, f->diag.t_last_gnss_fusion) < after_sec) return;
    }

    const float min_speed = (f->opt.automotive_min_speed_mps > 0.0f)
                                ? f->opt.automotive_min_speed_mps
                                : INS_AUTOMOTIVE_MIN_SPEED_MPS;
    const float vN        = f->state.vel_ned[0];
    const float vE        = f->state.vel_ned[1];
    const float speed     = SQRTF(vN * vN + vE * vE);
    if (speed < min_speed) return;

    /* Side slip grows with the turn, and with it a systematic residual the
       constraint would read as an attitude error. */
    const float max_yaw_rate = (f->opt.automotive_lateral_max_yaw_rate > 0.0f)
                                   ? f->opt.automotive_lateral_max_yaw_rate
                                   : INS_NHC_MAX_YAW_RATE;
    if (fabsf(f->last_omega_b_nb[2]) > max_yaw_rate) return;

    /* Rate limited because the residual is a slowly varying offset rather
       than white noise (REQ-NAV-077), so a faster fusion rate would only
       make the filter more certain of it, not better informed. */
    const int min_dt_ms = (int)(INS_NHC_MIN_INTERVAL_SEC * 1000.0f);
    if (f->t_last_nhc_fusion != 0 && time_diff_ms(m->timestamp, f->t_last_nhc_fusion) < min_dt_ms)
    {
        return;
    }

    float v_b[3];
    mat3t_mul_vec3(f->R_b_to_n, f->state.vel_ned, v_b);

    const float stddev = (f->opt.automotive_lateral_stddev_mps > 0.0f)
                             ? f->opt.automotive_lateral_stddev_mps
                             : INS_NHC_STDDEV_MPS;

    float z[1];
    float R[1];
    float Ht[INS_UNKNOWNS_MAX];
    int   j;
    memset(Ht, 0, sizeof(float) * (size_t)f->n);
    z[0] = v_b[1]; /* nominal minus the truth of zero */
    R[0] = qsquare(stddev);
    for (j = 0; j < 3; ++j)
    {
        /* velocity: row y of M, i.e. column y of R_b_to_n */
        Ht[INS_IDX_VEL + j] = MAT_ELEM(f->R_b_to_n, j, 1, 3, 3);
        /* attitude: row y of [v_b x] M */
        Ht[INS_IDX_RPY + j] =
            v_b[2] * MAT_ELEM(f->R_b_to_n, j, 0, 3, 3) - v_b[0] * MAT_ELEM(f->R_b_to_n, j, 2, 3, 3);
    }

    if (ins_fuse(f, z, R, Ht, 1, f->chi2_thr_gnss, 1) == 0)
    {
        f->t_last_nhc_fusion = m->timestamp;
        if (!f->log_state.nhc_logged)
        {
            f->log_state.nhc_logged = true;
            LOG_INFO("ins: lateral velocity constraint active (stddev %.2f m/s, %s, above "
                     "%.1f m/s and below %.1f deg/s yaw rate)",
                     (double)stddev,
                     (after_sec > 0.0f) ? "only while coasting" : "whenever the gates pass",
                     (double)min_speed, (double)RAD2DEG(max_yaw_rate));
        }
    }
}

/* ============================================================================
 * History: save current state to ring buffer
 * ============================================================================
 */

/* @satisfies REQ-NAV-008 */
static void ins_save_state(ins_t* f, ins_time_us_t tnow, bool enforce)
{
    const int prev  = index_mod(f->history_index - 1, INS_HISTORY_ITEMS_MAX);
    const int dt_ms = time_diff_ms(tnow, f->history[prev].time);
    if (dt_ms < INS_HISTORY_DT_MS && !enforce) { return; }

    ins_history_item_t* h = &f->history[f->history_index];
    h->time               = tnow;
    h->state              = f->state;
    h->latlonh[0]         = f->latlonh[0];
    h->latlonh[1]         = f->latlonh[1];
    h->latlonh[2]         = f->latlonh[2];
    memcpy(h->U, f->U, sizeof(h->U));
    memcpy(h->d, f->d, sizeof(h->d));
    vec3_copy(f->last_omega_b_nb, h->omega_b_nb);
    memcpy(h->R_b_to_n, f->R_b_to_n, sizeof(h->R_b_to_n));

    f->history_index = (f->history_index + 1) % INS_HISTORY_ITEMS_MAX;
}

/* ============================================================================
 * Health check
 * ============================================================================
 */

/* Shared re-arm for !opt.auto_reacquire_disable (REQ-NAV-042): back to the
 * collecting state so the next epoch's is_collecting branch re-runs auto-init
 * on a fresh, coherent IMU+fix window. Lighter than ins_init: it preserves the
 * monotonic diagnostics and never touches f->init/f->opt. */
static void ins_rearm_collecting(ins_t* f)
{
    f->is_collecting       = true;
    f->autoinit_count      = 0;
    f->autoinit_mag.valid  = false; /* don't reuse a pre-failure heading */
    f->autoinit_baro.valid = false; /* don't reuse a pre-failure pressure sample */
    f->bias_carry.valid    = false; /* only the quality exit sets one (REQ-NAV-061) */
    f->origin_carry.valid  = false; /* ditto for the n-frame origin (REQ-NAV-062) */
    f->height_from_baro    = false; /* re-decided at the next bootstrap (REQ-NAV-053) */
    f->baro_h0_m           = 0.0f;
    /* No gap can be reported against the previous instance's aiding history
       (REQ-NAV-058): the next bootstrap may not select the barometric
       source again. */
    f->log_state.t_last_baro_height_aid  = 0;
    f->log_state.t_last_baro_height_warn = 0;
    f->gnss_dwell_since                  = 0; /* re-earn the stability dwell (REQ-NAV-045) */
    f->gnss_dwell_count                  = 0;
    f->gnss_bad_since                    = 0;
    f->gnss_bad_last                     = 0;
    f->gnss_bad_accum_sec                = 0.0f;
    f->gnss_quality_ok                   = true; /* the entry gate governs the restart */
    f->have_pending_imu                  = false;
    f->have_pending_fix                  = false;
    f->last_acc_valid                    = false;
    f->acc_n_avg_valid                   = false;
}

/* Health-check shutdown. Unless opt.auto_reacquire_disable is set (with
 * auto_init), this re-arms the filter via ins_rearm_collecting() instead of
 * leaving it dead. An explicit ins_shutdown() does NOT route through here. */
/* @satisfies REQ-NAV-042 */
static void ins_fail_health(ins_t* f)
{
    f->is_initialized = false;
    if (!f->opt.auto_reacquire_disable && f->opt.auto_init)
    {
        f->diag.n_health_reset++;
        LOG_ERROR("ins: health check failed (non-finite/negative covariance or state), "
                  "auto-re-arming (reset #%u since init)",
                  (unsigned int)f->diag.n_health_reset);
        ins_rearm_collecting(f);
    }
    else
    {
        LOG_ERROR("ins: health check failed (non-finite/negative covariance or state), "
                  "filter shut down (auto-reacquire disabled or auto_init off)");
    }
}

/* GNSS quality-loss shutdown (REQ-NAV-052). Unlike ins_fail_health this is not
 * a corruption: the state was fine, the aiding stopped being good enough for a
 * 3D solution. IMU biases (REQ-NAV-061) and the n-frame origin (REQ-NAV-062)
 * are kept, position/velocity/attitude are re-derived by the next bootstrap.
 * @satisfies REQ-NAV-052 REQ-NAV-061 REQ-NAV-062 */
static void ins_fail_gnss_quality(ins_t* f, ins_time_us_t t)
{
    if (f->opt.auto_reacquire_disable || !f->opt.auto_init)
    {
        f->gnss_quality_ok    = false;
        f->gnss_bad_since     = 0;
        f->gnss_bad_last      = 0;
        f->gnss_bad_accum_sec = 0.0f;
        f->gnss_dwell_since   = 0; /* the entry dwell has to be re-earned */
        f->gnss_dwell_count   = 0;
        return;
    }

    /* Worst axis, so the seed is never optimistic about any one of them. */
    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    const float acc_sd =
        SQRTF(fmaxf(fmaxf(diag[INS_IDX_ACC + 0], diag[INS_IDX_ACC + 1]), diag[INS_IDX_ACC + 2]));
    const float gyr_sd =
        SQRTF(fmaxf(fmaxf(diag[INS_IDX_GYR + 0], diag[INS_IDX_GYR + 1]), diag[INS_IDX_GYR + 2]));
    float        acc_b[3], gyr_b[3], pos_at_exit[3];
    const double llh_at_exit[3] = {f->latlonh[0], f->latlonh[1], f->latlonh[2]};
    vec3_copy(f->state.acc_bias, acc_b);
    vec3_copy(f->state.gyr_bias, gyr_b);
    vec3_copy(f->state.pos_local, pos_at_exit);

    /* REQ-NAV-062: the origin of a running instance is sound by construction,
       but insisting on that here is cheaper than trusting it at the far end.
       Geodetic form, so the test is the range of a latitude and a longitude
       rather than the norm of a vector (REQ-NAV-080). */
    const double ox = f->origin_llh[0], oy = f->origin_llh[1], oz = f->origin_llh[2];
    const bool   origin_ok =
        vec3d_finite(f->origin_llh) && (fabs(ox) <= (0.5 * M_PI)) && (fabs(oy) <= (2.0 * M_PI));

    f->is_initialized = false;
    ins_rearm_collecting(f);

    if (origin_ok)
    {
        f->origin_carry.origin_llh[0] = ox;
        f->origin_carry.origin_llh[1] = oy;
        f->origin_carry.origin_llh[2] = oz;
        /* Where this instance last thought it was (REQ-NAV-062), in both
           frames: the pair is what lets the bootstrap work in short
           baselines instead of against the possibly far-away origin. */
        vec3_copy(pos_at_exit, f->origin_carry.pos_local);
        f->origin_carry.latlonh[0] = llh_at_exit[0];
        f->origin_carry.latlonh[1] = llh_at_exit[1];
        f->origin_carry.latlonh[2] = llh_at_exit[2];
        f->origin_carry.t          = (f->t_last_pos_aiding != 0) ? f->t_last_pos_aiding : t;
        f->origin_carry.valid      = true;
    }

    vec3_copy(acc_b, f->bias_carry.acc_bias);
    vec3_copy(gyr_b, f->bias_carry.gyr_bias);
    f->bias_carry.acc_bias_stddev_mps2 =
        fminf(INS_BIAS_CARRY_STDDEV_INFLATION * acc_sd, f->init.acc_bias_init_stddev_mps2);
    f->bias_carry.gyr_bias_stddev_rps =
        fminf(INS_BIAS_CARRY_STDDEV_INFLATION * gyr_sd, f->init.gyr_bias_init_stddev_rps);
    f->bias_carry.valid = true;

    LOG_WARN("ins: re-arming after the GNSS quality loss, position/velocity/attitude are "
             "re-derived at the next bootstrap, imu biases carried over with stddev "
             "acc %.4f m/s^2, gyr %.4f deg/s, n-frame origin %s",
             (double)f->bias_carry.acc_bias_stddev_mps2,
             (double)RAD2DEG(f->bias_carry.gyr_bias_stddev_rps),
             origin_ok ? "carried over" : "implausible, will be re-established");
}

/* @satisfies REQ-SYS-005 REQ-NAV-031 REQ-NAV-042 */
static void ins_check_health(ins_t* f)
{
    if (!f->is_initialized) return;

    int i;
    for (i = 0; i < f->n; ++i)
    {
        if (!isfinite(f->d[i]))
        {
            ins_fail_health(f);
            return;
        }
        if (f->d[i] < 0.0f)
        {
            ins_fail_health(f);
            return;
        }
        if (f->d[i] < INS_MIN_VARIANCE) { f->d[i] = INS_MIN_VARIANCE; }
    }
    /* Check state health. */
    for (i = 0; i < 3; ++i)
    {
        if (!isfinite(f->state.vel_ned[i]) || !isfinite(f->state.pos_local[i]))
        {
            ins_fail_health(f);
            return;
        }
    }
    for (i = 0; i < 4; ++i)
    {
        if (!isfinite(f->state.qbn[i]))
        {
            ins_fail_health(f);
            return;
        }
    }
}

/* Overconfidence / covariance-collapse watchdog floors (REQ-NAV-040): a
 * reported 1-sigma accuracy below any of these is implausible and is flagged as
 * a likely covariance collapse. Diagnostic only. */
#define INS_OVERCONF_POS_STDDEV_M   1e-4f /* 0.1 mm */
#define INS_OVERCONF_VEL_STDDEV_MPS 1e-4f /* 0.1 mm/s */
#define INS_OVERCONF_ATT_STDDEV_DEG 1e-3f /* 0.001 deg */

/* @satisfies REQ-NAV-040 */
static void ins_check_overconfidence(ins_t* f)
{
    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);

    /* Smallest per-axis 1-sigma in each group: the first channel to look
       "too good" is what trips. */
    float pos =
        SQRTF(fminf(fminf(diag[INS_IDX_POS + 0], diag[INS_IDX_POS + 1]), diag[INS_IDX_POS + 2]));
    float vel =
        SQRTF(fminf(fminf(diag[INS_IDX_VEL + 0], diag[INS_IDX_VEL + 1]), diag[INS_IDX_VEL + 2]));
    float att = RAD2DEG(
        SQRTF(fminf(fminf(diag[INS_IDX_RPY + 0], diag[INS_IDX_RPY + 1]), diag[INS_IDX_RPY + 2])));

    if (pos < f->diag.min_pos_stddev_m) { f->diag.min_pos_stddev_m = pos; }
    if (vel < f->diag.min_vel_stddev_mps) { f->diag.min_vel_stddev_mps = vel; }
    if (att < f->diag.min_att_stddev_deg) { f->diag.min_att_stddev_deg = att; }

    if (pos < INS_OVERCONF_POS_STDDEV_M || vel < INS_OVERCONF_VEL_STDDEV_MPS ||
        att < INS_OVERCONF_ATT_STDDEV_DEG)
    {
        if (!f->diag.overconfident)
        {
            LOG_WARN("ins: covariance overconfidence detected (pos %.2g m, vel %.2g m/s, "
                     "att %.2g deg 1-sigma) - possible covariance collapse",
                     (double)pos, (double)vel, (double)att);
        }
        f->diag.overconfident = true;
        f->diag.n_overconfident++;
    }
}

/* ============================================================================
 * Public API: init, shutdown, world model
 * ============================================================================
 */

/* Finalize the nominal state, covariance and world model from a resolved
 * attitude (rpy), n-frame origin, local position offset and NED velocity.
 * Shared by the manual ins_init and the auto-init bootstrap. rpy_var lets the
 * caller widen the roll/pitch/yaw variance per axis when the bootstrap is less
 * certain (REQ-NAV-047). f->init/f->opt must already be populated. */
/* @satisfies REQ-NAV-061 REQ-NAV-080 */
static void ins_finalize_init(ins_t* f, const float rpy[3], const double origin_llh[3],
                              const float pos_local[3], const double* latlonh_exact,
                              const float vel_ned[3], const float rpy_var[3], ins_time_us_t t)
{
    f->t_init                   = t;
    f->t_last_kalman_predict    = t;
    f->t_last_pos_aiding        = t;
    f->t_last_mag_fusion        = t;
    f->t_last_zero_rot_fusion   = t;
    f->t_last_zero_vel_fusion   = t;
    f->log_state.t_last_yaw_aid = t;
    f->kalman_epochs            = 0;
    f->last_acc_valid           = false;
    f->acc_n_avg_valid          = false;

    ins_quat_from_rpy(rpy[0], rpy[1], rpy[2], f->state.qbn);

    f->origin_llh[0] = origin_llh[0];
    f->origin_llh[1] = origin_llh[1];
    f->origin_llh[2] = origin_llh[2];
    vec3_copy(pos_local, f->state.pos_local);

    /* The absolute anchor starts at the origin, no conversion in between
       (REQ-NAV-080). */
    f->latlonh[0] = f->origin_llh[0];
    f->latlonh[1] = f->origin_llh[1];
    f->latlonh[2] = f->origin_llh[2];
    ins_refresh_earth_params(f); /* gravity, curvature cache */

    /* Keep latlonh consistent with a nonzero initial pos_local: the origin is
       the anchor, the absolute position is the origin offset by pos_local in
       the n-frame (same mapping ins_reacquire uses, REQ-NAV-062). A caller
       that already knows the exact latitude/longitude of the bootstrap point
       hands them in instead, since that mapping is only exact for a short
       offset (see ins_reacquire). */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (pos_local[0] != 0.0f || pos_local[1] != 0.0f || pos_local[2] != 0.0f)
#pragma GCC diagnostic pop
    {
        double dllh[3];
        ins_dned_to_dlatlonh(f->state.pos_local, f->latlonh[0], f->latlonh[2], dllh);
        f->latlonh[0] += dllh[0];
        f->latlonh[1] += dllh[1];
        f->latlonh[2] += dllh[2];
        /* Re-evaluate the Earth params at the actual position rather than
           at the origin now that the two can be kilometres apart. */
        ins_refresh_earth_params(f);
    }
    if (latlonh_exact != NULL)
    {
        f->latlonh[0] = latlonh_exact[0];
        f->latlonh[1] = latlonh_exact[1];
        ins_refresh_earth_params(f);
    }

    vec3_copy(vel_ned, f->state.vel_ned);

    /* IMU biases: from a quality-loss carry when one is pending, else the
       configured cold-start values (REQ-NAV-061). The carry's stddevs are
       already inflated and clamped at the exit site. */
    const bool  carry = f->bias_carry.valid;
    const float acc_var =
        qsquare(carry ? f->bias_carry.acc_bias_stddev_mps2 : f->init.acc_bias_init_stddev_mps2);
    const float gyr_var =
        qsquare(carry ? f->bias_carry.gyr_bias_stddev_rps : f->init.gyr_bias_init_stddev_rps);
    vec3_copy(carry ? f->bias_carry.acc_bias : f->init.acc_bias_init_mps2, f->state.acc_bias);
    vec3_copy(carry ? f->bias_carry.gyr_bias : f->init.gyr_bias_init_rps, f->state.gyr_bias);

    /* Initial covariance (diagonal). */
    float diag_init[INS_UNKNOWNS_MAX];
    int   i;
    for (i = 0; i < 3; ++i)
    {
        diag_init[INS_IDX_POS + i] = qsquare(f->init.pos_init_stddev_m);
        diag_init[INS_IDX_VEL + i] = qsquare(f->init.vel_init_stddev_mps);
        diag_init[INS_IDX_ACC + i] = acc_var;
        diag_init[INS_IDX_GYR + i] = gyr_var;
        diag_init[INS_IDX_RPY + i] = rpy_var[i];
    }
    if (f->n > INS_IDX_MAG)
    {
        for (i = 0; i < 3; ++i)
        {
            diag_init[INS_IDX_MAG + i] = qsquare(f->init.mag_bias_init_stddev_ut);
        }
        vec3_zero(f->state.mag_bias); /* hard iron starts unknown */
    }
    udu_set_diag(f->U, f->d, diag_init, f->n);

    /* Process noise (per second). The acc/gyr bias random walks are injected
       via the G/Q noise-input form in the predict step; the diagonal here holds
       the pos/vel/rpy extra noise plus the 18-state mag-bias random walk. */
    memset(f->Qxx_noise_diag, 0, sizeof(f->Qxx_noise_diag));
    for (i = 0; i < 3; ++i)
    {
        f->Qxx_noise_diag[INS_IDX_POS + i] = qsquare(f->init.pos_pred_stddev_m_sqrts);
        f->Qxx_noise_diag[INS_IDX_VEL + i] = qsquare(f->init.vel_pred_stddev_mps_sqrts);
        f->Qxx_noise_diag[INS_IDX_RPY + i] = qsquare(f->init.rpy_pred_stddev_rad_sqrts);
        if (f->n > INS_IDX_MAG)
        {
            f->Qxx_noise_diag[INS_IDX_MAG + i] = qsquare(f->init.mag_bias_pred_stddev_ut_sqrts);
        }
    }

    /* World model. A supplied gravity vector overrides the model value that
       ins_refresh_earth_params computed above. */
    if (vec3_norm(f->init.gravity_n) > 1e-4f) { vec3_copy(f->init.gravity_n, f->gravity_n); }
    vec3_copy(f->init.magnetic_n, f->magnetic_n);

    /* R_b_to_n from the initial quaternion. */
    ins_quat_to_rotmat(f->state.qbn, f->R_b_to_n);

    /* Initial history entry. */
    f->history_index = 0;
    ins_save_state(f, t, true);

    f->is_collecting      = false;
    f->is_initialized     = true;
    f->gnss_quality_ok    = true; /* entry gate passed (REQ-NAV-051) */
    f->gnss_bad_since     = 0;
    f->gnss_bad_last      = 0;
    f->gnss_bad_accum_sec = 0.0f;
    f->bias_carry.valid   = false; /* consumed (REQ-NAV-061) */
    f->origin_carry.valid = false; /* consumed (REQ-NAV-062) */
    if (carry)
    {
        LOG_INFO("ins: imu biases carried over from the previous instance, acc=[%.4f %.4f %.4f] "
                 "m/s^2 (stddev %.4f), gyr=[%.3f %.3f %.3f] deg/s (stddev %.4f)",
                 (double)f->state.acc_bias[0], (double)f->state.acc_bias[1],
                 (double)f->state.acc_bias[2], (double)SQRTF(acc_var),
                 (double)RAD2DEG(f->state.gyr_bias[0]), (double)RAD2DEG(f->state.gyr_bias[1]),
                 (double)RAD2DEG(f->state.gyr_bias[2]), (double)RAD2DEG(SQRTF(gyr_var)));
    }
    LOG_INFO("ins: filter started, rpy=[%.1f %.1f %.1f] deg, rpy stddev=[%.2f %.2f %.2f] deg, "
             "pos stddev=%.1f m, vel stddev=%.2f m/s",
             (double)RAD2DEG(rpy[0]), (double)RAD2DEG(rpy[1]), (double)RAD2DEG(rpy[2]),
             (double)(RAD2DEG(SQRTF(rpy_var[0]))), (double)(RAD2DEG(SQRTF(rpy_var[1]))),
             (double)(RAD2DEG(SQRTF(rpy_var[2]))), (double)f->init.pos_init_stddev_m,
             (double)f->init.vel_init_stddev_mps);
}

/* Dump every effective (0 -> default) ins_init_t/ins_options_t parameter the
 * filter will actually run with. f->init is fully resolved by ins_init, most of
 * f->opt is NOT (several fields resolve lazily where read), so those are
 * mirrored here with the same "> 0 ? value : DEFAULT" idiom. */
static void ins_log_effective_config(const ins_t* f)
{
#if LOG_LEVEL >= LOG_LEVEL_INFO
    /* Every statement below exists only to feed LOG_INFO. Below the
       compile-time ceiling LOG_INFO expands to nothing (log.h) and these locals
       would be flagged -Wunused-variable, so the body is compiled out too. */
    const ins_init_t*    init = &f->init;
    const ins_options_t* opt  = &f->opt;
    LOG_INFO("ins: state size %d%s, auto-init %s, auto-reacquire %s", f->n,
             opt->estimate_mag_bias ? " (mag bias states enabled)" : "",
             opt->auto_init ? "enabled" : "disabled",
             opt->auto_reacquire_disable ? "disabled" : "enabled");
    LOG_INFO("ins: init stddevs: pos %.2f m, vel %.2f m/s, rpy=[%.2f %.2f %.2f] deg, "
             "acc bias %.3f m/s^2, gyr bias %.3f deg/s",
             (double)init->pos_init_stddev_m, (double)init->vel_init_stddev_mps,
             (double)RAD2DEG(init->rpy_init_stddev_rad[0]),
             (double)RAD2DEG(init->rpy_init_stddev_rad[1]),
             (double)RAD2DEG(init->rpy_init_stddev_rad[2]), (double)init->acc_bias_init_stddev_mps2,
             (double)RAD2DEG(init->gyr_bias_init_stddev_rps));
    if (opt->estimate_mag_bias)
    {
        LOG_INFO("ins: mag bias init stddev %.2f uT, random walk %.4f uT/sqrt(s)",
                 (double)init->mag_bias_init_stddev_ut,
                 (double)init->mag_bias_pred_stddev_ut_sqrts);
    }
    LOG_INFO("ins: process noise (extra margin on top of IMU-driven prediction noise): "
             "pos %.4f m/sqrt(s), vel %.4f m/s/sqrt(s), rpy %.4f deg/sqrt(s), "
             "acc bias rw %.5f m/s^2/sqrt(s), gyr bias rw %.5f deg/s/sqrt(s)",
             (double)init->pos_pred_stddev_m_sqrts, (double)init->vel_pred_stddev_mps_sqrts,
             (double)RAD2DEG(init->rpy_pred_stddev_rad_sqrts),
             (double)init->acc_bias_pred_stddev_mps2_sqrts,
             (double)RAD2DEG(init->gyr_bias_pred_stddev_rps_sqrts));
    LOG_INFO("ins: virtual measurement stddevs: zero-vel %.3f m/s, zero-rot %.4f deg/s",
             (double)init->zero_vel_stddev_mps, (double)RAD2DEG(init->zero_rot_stddev_rps));

    {
        const float kalman_dt = (opt->kalman_update_dt_sec > 0.0f)
                                    ? opt->kalman_update_dt_sec
                                    : INS_DEFAULT_KALMAN_UPDATE_DT_SEC;
        LOG_INFO("ins: cadence: covariance update %.3f s (%.1f Hz), max prediction gap %.2f s",
                 (double)kalman_dt, (double)(1.0f / kalman_dt),
                 (double)opt->max_prediction_time_sec);
    }
    if (opt->allow_unlimited_deadreckoning)
    {
        LOG_INFO("ins: dead-reckoning coasting: unlimited (is_ready never degrades on its own)");
    }
    else
    {
        const float dr_sec = (opt->max_deadreckoning_sec > 0.0f)
                                 ? opt->max_deadreckoning_sec
                                 : INS_DEFAULT_MAX_DEADRECKONING_SEC;
        LOG_INFO("ins: dead-reckoning coasting: max %.1f s before is_ready degrades",
                 (double)dr_sec);
    }
    LOG_INFO("ins: gnss fusion gates: pos hor/ver %.1f/%.1f m, vel hor/ver %.2f/%.2f m/s",
             (double)opt->gnss_max_horizontal_pos_stddev_m,
             (double)opt->gnss_max_vertical_pos_stddev_m,
             (double)opt->gnss_max_horizontal_vel_stddev_mps,
             (double)opt->gnss_max_vertical_vel_stddev_mps);
    LOG_INFO("ins: gnss 3D entry gates: pos hor/ver %.1f/%.1f m, vel hor/ver %.2f/%.2f m/s, "
             "dwell %.1f s%s",
             (double)opt->gnss_start_max_horizontal_pos_stddev_m,
             (double)opt->gnss_start_max_vertical_pos_stddev_m,
             (double)opt->gnss_start_max_horizontal_vel_stddev_mps,
             (double)opt->gnss_start_max_vertical_vel_stddev_mps, (double)opt->gnss_init_dwell_sec,
             opt->gnss_init_dwell_disable ? " (dwell disabled: enter on the first good fix)" : "");
    LOG_INFO("ins: gnss 3D exit gates: pos hor/ver %.1f/%.1f m, vel hor/ver %.2f/%.2f m/s, "
             "dwell %.1f s%s",
             (double)opt->gnss_stop_max_horizontal_pos_stddev_m,
             (double)opt->gnss_stop_max_vertical_pos_stddev_m,
             (double)opt->gnss_stop_max_horizontal_vel_stddev_mps,
             (double)opt->gnss_stop_max_vertical_vel_stddev_mps, (double)opt->gnss_stop_dwell_sec,
             opt->gnss_stop_disable ? " (disabled: never leave 3D on GNSS quality)" : "");
    {
        /* Defaults are resolved further down, resolve once here so the log
           states what the filter will actually do. */
        const int raw   = opt->gnss_pos_decimation;
        const int decim = (raw == 0) ? INS_DEFAULT_GNSS_POS_DECIMATION : ((raw < 1) ? 1 : raw);
        if (decim > 1)
        {
            LOG_INFO("ins: gnss position decimation: 1 of %d epochs offering both blocks "
                     "fuses the position, the rest the velocity alone",
                     decim);
        }
        else
        {
            LOG_INFO("ins: gnss position decimation: off (position and velocity are fused "
                     "together, assuming they are uncorrelated)");
        }
    }
    {
        /* REQ-NAV-068. Logged unconditionally: the input is optional per epoch,
           so nothing at init time tells whether a speed source is wired up, and
           a caller debugging why its samples do nothing needs the gate. */
        const float sc = (opt->speed_scale > 0.0f) ? opt->speed_scale : 1.0f;
        const float rel =
            (opt->speed_stddev_rel > 0.0f) ? opt->speed_stddev_rel : INS_DEFAULT_SPEED_STDDEV_REL;
        const float vmin =
            (opt->speed_min_mps > 0.0f) ? opt->speed_min_mps : INS_DEFAULT_SPEED_MIN_MPS;
        LOG_INFO("ins: absolute speed aiding: scale %.4f, relative stddev %.1f%%, "
                 "min filtered speed %.2f m/s",
                 (double)sc, (double)(rel * 100.0f), (double)vmin);
    }
    {
        const float pos_scale = (opt->gnss_pos_cov_scale > 0.0f) ? opt->gnss_pos_cov_scale : 1.0f;
        const float vel_scale = (opt->gnss_vel_cov_scale > 0.0f) ? opt->gnss_vel_cov_scale : 1.0f;
        const float height_scale =
            (opt->gnss_pos_cov_scale_height > 0.0f) ? opt->gnss_pos_cov_scale_height : 1.0f;
        LOG_INFO("ins: gnss cov conditioning: pos scale %.2f (height x%.2f), vel scale %.2f, "
                 "pos floor hor/ver %.2f/%.2f m, vel floor hor/ver %.2f/%.2f m/s",
                 (double)pos_scale, (double)height_scale, (double)vel_scale,
                 (double)opt->gnss_pos_stddev_floor_hor_m, (double)opt->gnss_pos_stddev_floor_ver_m,
                 (double)opt->gnss_vel_stddev_floor_hor_mps,
                 (double)opt->gnss_vel_stddev_floor_ver_mps);
    }
    {
        const float mag_tol = (opt->mag_field_tolerance > 0.0f) ? opt->mag_field_tolerance
                                                                : INS_DEFAULT_MAG_FIELD_TOL;
        LOG_INFO("ins: magnetometer: min fusion delay %d ms, fallback noise %.1f uT/axis, "
                 "field-strength gate %s (tol %.0f%%)",
                 opt->magnetometer_min_delay_ms, (double)INS_DEFAULT_MAG_STDDEV_UT,
                 opt->mag_field_check_disable ? "disabled" : "enabled", (double)(mag_tol * 100.0f));
    }
    LOG_INFO("ins: imu calibration: acc misalignment %s, fixed bias=[%.4f %.4f %.4f] m/s^2, "
             "gyr misalignment %s, fixed bias=[%.3f %.3f %.3f] deg/s",
             mat3_is_set(opt->imu_acc_misalignment) ? "custom" : "identity",
             (double)opt->imu_acc_fixed_bias[0], (double)opt->imu_acc_fixed_bias[1],
             (double)opt->imu_acc_fixed_bias[2],
             mat3_is_set(opt->imu_gyr_misalignment) ? "custom" : "identity",
             (double)RAD2DEG(opt->imu_gyr_fixed_bias[0]),
             (double)RAD2DEG(opt->imu_gyr_fixed_bias[1]),
             (double)RAD2DEG(opt->imu_gyr_fixed_bias[2]));
    LOG_INFO("ins: magnetometer calibration: misalignment %s, fixed bias=[%.2f %.2f %.2f] uT",
             mat3_is_set(opt->mag_misalignment) ? "custom" : "identity",
             (double)opt->mag_fixed_bias[0], (double)opt->mag_fixed_bias[1],
             (double)opt->mag_fixed_bias[2]);

    if (opt->auto_init)
    {
        const float win  = (opt->auto_init_window_sec > 0.0f) ? opt->auto_init_window_sec
                                                              : INS_AUTOINIT_DEFAULT_WINDOW_SEC;
        const float sgyr = (opt->auto_init_static_gyr_rps > 0.0f) ? opt->auto_init_static_gyr_rps
                                                                  : INS_AUTOINIT_DEFAULT_STATIC_GYR;
        const float sacc = (opt->auto_init_static_acc_mps2 > 0.0f)
                               ? opt->auto_init_static_acc_mps2
                               : INS_AUTOINIT_DEFAULT_STATIC_ACC;
        const float moving_floor = (opt->auto_init_moving_rpy_stddev_rad > 0.0f)
                                       ? opt->auto_init_moving_rpy_stddev_rad
                                       : INS_AUTOINIT_DEFAULT_MOVING_RPY_STDDEV;
        LOG_INFO(
            "ins: auto-init: leveling window %.2f s, static gates gyr %.3f deg/s / acc %.2f m/s^2, "
            "moving rpy floor %.1f deg",
            (double)win, (double)RAD2DEG(sgyr), (double)sacc, (double)RAD2DEG(moving_floor));
    }
    if (!opt->auto_zupt_disable)
    {
        const float sgyr    = (opt->auto_zupt_static_gyr_rps > 0.0f) ? opt->auto_zupt_static_gyr_rps
                                                                     : INS_AUTOZUPT_DEFAULT_STATIC_GYR;
        const float sacc    = (opt->auto_zupt_static_acc_mps2 > 0.0f)
                                  ? opt->auto_zupt_static_acc_mps2
                                  : INS_AUTOZUPT_DEFAULT_STATIC_ACC;
        const float mvel    = (opt->auto_zupt_max_vel_mps > 0.0f) ? opt->auto_zupt_max_vel_mps
                                                                  : INS_AUTOZUPT_DEFAULT_MAX_VEL_MPS;
        const float dwell   = (opt->auto_zupt_dwell_sec > 0.0f) ? opt->auto_zupt_dwell_sec
                                                                : INS_AUTOZUPT_DEFAULT_DWELL_SEC;
        const float min_int = (opt->auto_zupt_min_interval_sec > 0.0f)
                                  ? opt->auto_zupt_min_interval_sec
                                  : INS_AUTOZUPT_DEFAULT_MIN_INTERVAL_SEC;
        const float svgyr   = (opt->auto_zupt_static_gyr_stddev_rps > 0.0f)
                                  ? opt->auto_zupt_static_gyr_stddev_rps
                                  : INS_DEFAULT_STATIC_GYR_STDDEV_RPS;
        const float svacc   = (opt->auto_zupt_static_acc_stddev_mps2 > 0.0f)
                                  ? opt->auto_zupt_static_acc_stddev_mps2
                                  : INS_DEFAULT_STATIC_ACC_STDDEV_MPS2;
        LOG_INFO("ins: auto-zupt/zaru: window stddev gyr %.3f deg/s / acc %.2f m/s^2, "
                 "magnitude bounds gyr %.3f deg/s / acc %.2f m/s^2 / vel %.2f m/s, "
                 "dwell %.1f s, min interval %.1f s",
                 (double)RAD2DEG(svgyr), (double)svacc, (double)RAD2DEG(sgyr), (double)sacc,
                 (double)mvel, (double)dwell, (double)min_int);
    }
    else { LOG_INFO("ins: auto-zupt/zaru: disabled"); }
    if (opt->automotive_mode)
    {
        const float min_speed      = (opt->automotive_min_speed_mps > 0.0f)
                                         ? opt->automotive_min_speed_mps
                                         : INS_AUTOMOTIVE_MIN_SPEED_MPS;
        const float min_yaw_stddev = (opt->automotive_min_yaw_stddev > 0.0f)
                                         ? opt->automotive_min_yaw_stddev
                                         : INS_AUTOMOTIVE_MIN_YAW_STDDEV;
        LOG_INFO("ins: automotive mode: enabled, min speed %.1f m/s, min yaw stddev %.1f deg",
                 (double)min_speed, (double)RAD2DEG(min_yaw_stddev));
    }
    if (opt->automotive_lateral_constraint && !opt->automotive_mode)
    {
        LOG_WARN("ins: lateral velocity constraint asked for without automotive mode, "
                 "ignored (REQ-NAV-077)");
    }
#else
    (void)f;
#endif
}

/* @satisfies REQ-NAV-081 */
int ins_init(ins_t* f, const ins_init_t* init, const ins_options_t* opt)
{
    if (f == NULL || init == NULL || opt == NULL) { return -1; }

    /* Validate the initial position: a latitude and a longitude by their
       range (REQ-NAV-081). All zero passes, it is a point on the equator. */
    if (!vec3d_finite(init->llh) || fabs(init->llh[0]) > (0.5 * M_PI) ||
        fabs(init->llh[1]) > (2.0 * M_PI))
    {
        return -1;
    }

    /* Optional magnetometer-bias states need the max-sized arrays. With
       INS_UNKNOWNS_MAX overridden to 15 the option must fail loudly. */
    if (opt->estimate_mag_bias && INS_UNKNOWNS_MAX < INS_UNKNOWNS_MAG) { return -1; }

    memset(f, 0, sizeof(*f));
    f->init           = *init;
    f->opt            = *opt;
    f->n              = opt->estimate_mag_bias ? INS_UNKNOWNS_MAG : INS_UNKNOWNS;
    f->history_index  = 0;
    f->last_acc_valid = false;
    /* No position known yet, so nothing says the magnetometer is unusable.
       memset zeroed this, which would suppress fusion for a caller that only
       supplies magnetic_n through ins_set_world_model(). */
    f->mag_heading_usable = true;

    /* Overconfidence watchdog running-minima start "unseen" (REQ-NAV-040),
       memset zeroed them, which would pin the minimum at 0 forever. */
    f->diag.min_pos_stddev_m   = INFINITY;
    f->diag.min_vel_stddev_mps = INFINITY;
    f->diag.min_att_stddev_deg = INFINITY;
    /* Bias random-walk defaults (0 -> defaults). */
    if (f->init.acc_bias_pred_stddev_mps2_sqrts <= 0.0f)
    {
        f->init.acc_bias_pred_stddev_mps2_sqrts = INS_DEFAULT_ACC_BIAS_RW_MPS2_SQRTS;
    }
    if (f->init.gyr_bias_pred_stddev_rps_sqrts <= 0.0f)
    {
        f->init.gyr_bias_pred_stddev_rps_sqrts = INS_DEFAULT_GYR_BIAS_RW_RPS_SQRTS;
    }
    /* Magnetometer fusion rate limit: 0 -> default, negative -> no rate limit.
       Resolved once here so the gate in ins_fuse_mag stays a plain comparison
       and the startup log reports the effective value. */
    if (f->opt.magnetometer_min_delay_ms == 0)
    {
        f->opt.magnetometer_min_delay_ms = INS_DEFAULT_MAG_MIN_DELAY_MS;
    }
    else if (f->opt.magnetometer_min_delay_ms < 0) { f->opt.magnetometer_min_delay_ms = 0; }
    /* Magnetometer hard-iron bias defaults (18-state mode). */
    if (f->init.mag_bias_init_stddev_ut <= 0.0f)
    {
        f->init.mag_bias_init_stddev_ut = INS_DEFAULT_MAG_BIAS_STDDEV_UT;
    }
    if (f->init.mag_bias_pred_stddev_ut_sqrts <= 0.0f)
    {
        f->init.mag_bias_pred_stddev_ut_sqrts = INS_DEFAULT_MAG_BIAS_RW_UT;
    }
    /* Initial-state and process-noise defaults: a caller who leaves these at 0
       gets a usable, correctable filter instead of states permanently locked at
       their seed value. @satisfies REQ-NAV-049 */
    if (f->init.pos_init_stddev_m <= 0.0f)
    {
        f->init.pos_init_stddev_m = INS_DEFAULT_POS_INIT_STDDEV_M;
    }
    if (f->init.vel_init_stddev_mps <= 0.0f)
    {
        f->init.vel_init_stddev_mps = INS_DEFAULT_VEL_INIT_STDDEV_MPS;
    }
    {
        /* Roll/pitch/yaw are independently settable (unlike the pos/vel/bias
           scalars above): each axis left at 0 resolves to the same default on
           its own. @satisfies REQ-NAV-026 */
        int i;
        for (i = 0; i < 3; ++i)
        {
            if (f->init.rpy_init_stddev_rad[i] <= 0.0f)
            {
                f->init.rpy_init_stddev_rad[i] = INS_DEFAULT_RPY_INIT_STDDEV_RAD;
            }
        }
    }
    if (f->init.acc_bias_init_stddev_mps2 <= 0.0f)
    {
        f->init.acc_bias_init_stddev_mps2 = INS_DEFAULT_ACC_BIAS_INIT_STDDEV_MPS2;
    }
    if (f->init.gyr_bias_init_stddev_rps <= 0.0f)
    {
        f->init.gyr_bias_init_stddev_rps = INS_DEFAULT_GYR_BIAS_INIT_STDDEV_RPS;
    }
    if (f->init.pos_pred_stddev_m_sqrts <= 0.0f)
    {
        f->init.pos_pred_stddev_m_sqrts = INS_DEFAULT_POS_PRED_STDDEV_MPS;
    }
    if (f->init.vel_pred_stddev_mps_sqrts <= 0.0f)
    {
        f->init.vel_pred_stddev_mps_sqrts = INS_DEFAULT_VEL_PRED_STDDEV_MPS2;
    }
    if (f->init.rpy_pred_stddev_rad_sqrts <= 0.0f)
    {
        f->init.rpy_pred_stddev_rad_sqrts = INS_DEFAULT_RPY_PRED_STDDEV_RPS;
    }
    if (f->init.zero_vel_stddev_mps <= 0.0f)
    {
        f->init.zero_vel_stddev_mps = INS_DEFAULT_ZERO_VEL_STDDEV_MPS;
    }
    if (f->init.zero_rot_stddev_rps <= 0.0f)
    {
        f->init.zero_rot_stddev_rps = INS_DEFAULT_ZERO_ROT_STDDEV_RPS;
    }

    /* Beginner-friendly defaults (REQ-NAV-043): a zeroed ins_options_t must
       yield a working filter. Unlike the lazily resolved fields these are
       consulted in many places, so they are resolved once into f->opt here.
       @satisfies REQ-NAV-043 */
    if (f->opt.max_prediction_time_sec <= 0.0f)
    {
        f->opt.max_prediction_time_sec = INS_DEFAULT_MAX_PREDICTION_TIME_SEC;
    }
    if (f->opt.gnss_max_horizontal_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_max_horizontal_pos_stddev_m = INS_DEFAULT_GNSS_MAX_HPOS_STDDEV_M;
    }
    if (f->opt.gnss_max_vertical_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_max_vertical_pos_stddev_m = INS_DEFAULT_GNSS_MAX_VPOS_STDDEV_M;
    }
    if (f->opt.gnss_max_horizontal_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_max_horizontal_vel_stddev_mps = INS_DEFAULT_GNSS_MAX_HVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_max_vertical_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_max_vertical_vel_stddev_mps = INS_DEFAULT_GNSS_MAX_VVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_init_dwell_sec <= 0.0f)
    {
        f->opt.gnss_init_dwell_sec = INS_DEFAULT_GNSS_INIT_DWELL_SEC;
    }
    /* GNSS fusion rate limit (REQ-NAV-074), same convention as the
       magnetometer's: 0 -> the default, negative -> no limit.
       @satisfies REQ-NAV-043 */
    if (f->opt.gnss_min_delay_ms == 0) { f->opt.gnss_min_delay_ms = INS_DEFAULT_GNSS_MIN_DELAY_MS; }
    else if (f->opt.gnss_min_delay_ms < 0) { f->opt.gnss_min_delay_ms = 0; }

    /* GNSS covariance conditioning knobs (REQ-NAV-038, REQ-NAV-071,
       REQ-NAV-072). Same "0 -> default" rule as the gates above, plus an
       explicit opt-out: "no floor" and "no cap" are meaningful configurations
       that 0 cannot express once it means "default". A negative value therefore
       resolves to 0, which every conditioning step reads as "not applied".
       @satisfies REQ-NAV-043 */
    {
        struct
        {
            float* v;
            float  def;
        } knob[12] = {
            {&f->opt.gnss_pos_stddev_floor_hor_m, INS_DEFAULT_GNSS_POS_STDDEV_FLOOR_HOR_M},
            {&f->opt.gnss_pos_stddev_floor_ver_m, INS_DEFAULT_GNSS_POS_STDDEV_FLOOR_VER_M},
            {&f->opt.gnss_vel_stddev_floor_hor_mps, INS_DEFAULT_GNSS_VEL_STDDEV_FLOOR_HOR_MPS},
            {&f->opt.gnss_vel_stddev_floor_ver_mps, INS_DEFAULT_GNSS_VEL_STDDEV_FLOOR_VER_MPS},
            {&f->opt.gnss_pos_stddev_cap_hor_m, INS_DEFAULT_GNSS_POS_STDDEV_CAP_HOR_M},
            {&f->opt.gnss_pos_stddev_cap_ver_m, INS_DEFAULT_GNSS_POS_STDDEV_CAP_VER_M},
            {&f->opt.gnss_vel_stddev_cap_hor_mps, INS_DEFAULT_GNSS_VEL_STDDEV_CAP_HOR_MPS},
            {&f->opt.gnss_vel_stddev_cap_ver_mps, INS_DEFAULT_GNSS_VEL_STDDEV_CAP_VER_MPS},
            {&f->opt.gnss_acc_envelope_tau_sec, INS_DEFAULT_GNSS_ACC_ENVELOPE_TAU_SEC},
            {&f->opt.gnss_vel_noise_acc_scale_hor, INS_DEFAULT_GNSS_VEL_NOISE_ACC_SCALE_HOR},
            {&f->opt.gnss_vel_noise_acc_scale_ver, INS_DEFAULT_GNSS_VEL_NOISE_ACC_SCALE_VER},
            {&f->opt.gnss_vel_noise_acc_window_sec, INS_DEFAULT_GNSS_VEL_NOISE_ACC_WINDOW_SEC}};
        int ki;
        for (ki = 0; ki < 12; ++ki)
        {
            if (*knob[ki].v < 0.0f) { *knob[ki].v = 0.0f; }
            else if (!(*knob[ki].v > 0.0f)) { *knob[ki].v = knob[ki].def; }
        }
    }

    /* A cap below the floor of the same axis group would leave the two
       clamping one variance against each other, with the cap winning because
       it runs last. @satisfies REQ-NAV-071 */
    {
        struct
        {
            float*      cap;
            float*      flr;
            const char* name;
        } pair[4] = {{&f->opt.gnss_pos_stddev_cap_hor_m, &f->opt.gnss_pos_stddev_floor_hor_m,
                      "horizontal pos"},
                     {&f->opt.gnss_pos_stddev_cap_ver_m, &f->opt.gnss_pos_stddev_floor_ver_m,
                      "vertical pos"},
                     {&f->opt.gnss_vel_stddev_cap_hor_mps, &f->opt.gnss_vel_stddev_floor_hor_mps,
                      "horizontal vel"},
                     {&f->opt.gnss_vel_stddev_cap_ver_mps, &f->opt.gnss_vel_stddev_floor_ver_mps,
                      "vertical vel"}};
        int pi;
        for (pi = 0; pi < 4; ++pi)
        {
            if (*pair[pi].cap > 0.0f && *pair[pi].cap < *pair[pi].flr)
            {
                LOG_WARN("ins: GNSS %s accuracy cap %.3f below the floor %.3f, raised to the floor",
                         pair[pi].name, (double)*pair[pi].cap, (double)*pair[pi].flr);
                *pair[pi].cap = *pair[pi].flr;
            }
        }
    }

    /* Mode-transition quality checks (REQ-NAV-051, REQ-NAV-052). The entry
       threshold must not be looser than the fusion threshold, and the exit gate
       must not be stricter than the entry gate.
       @satisfies REQ-NAV-051 REQ-NAV-052 */
    if (f->opt.gnss_start_max_horizontal_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_start_max_horizontal_pos_stddev_m = INS_DEFAULT_GNSS_START_MAX_HPOS_STDDEV_M;
    }
    if (f->opt.gnss_start_max_vertical_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_start_max_vertical_pos_stddev_m = INS_DEFAULT_GNSS_START_MAX_VPOS_STDDEV_M;
    }
    if (f->opt.gnss_start_max_horizontal_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_start_max_horizontal_vel_stddev_mps =
            INS_DEFAULT_GNSS_START_MAX_HVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_start_max_vertical_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_start_max_vertical_vel_stddev_mps = INS_DEFAULT_GNSS_START_MAX_VVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_stop_max_horizontal_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_stop_max_horizontal_pos_stddev_m = INS_DEFAULT_GNSS_STOP_MAX_HPOS_STDDEV_M;
    }
    if (f->opt.gnss_stop_max_vertical_pos_stddev_m <= 0.0f)
    {
        f->opt.gnss_stop_max_vertical_pos_stddev_m = INS_DEFAULT_GNSS_STOP_MAX_VPOS_STDDEV_M;
    }
    if (f->opt.gnss_stop_max_horizontal_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_stop_max_horizontal_vel_stddev_mps = INS_DEFAULT_GNSS_STOP_MAX_HVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_stop_max_vertical_vel_stddev_mps <= 0.0f)
    {
        f->opt.gnss_stop_max_vertical_vel_stddev_mps = INS_DEFAULT_GNSS_STOP_MAX_VVEL_STDDEV_MPS;
    }
    if (f->opt.gnss_stop_dwell_sec <= 0.0f)
    {
        f->opt.gnss_stop_dwell_sec = INS_DEFAULT_GNSS_STOP_DWELL_SEC;
    }
    /* GNSS position decimation (REQ-NAV-063). 0 means default.
       The value 1 is the explicit opt-out and is left alone. A negative factor
       is a configuration issue, print a warning then (REQ-NAV-043). */
    if (f->opt.gnss_pos_decimation == 0)
    {
        f->opt.gnss_pos_decimation = INS_DEFAULT_GNSS_POS_DECIMATION;
    }
    else if (f->opt.gnss_pos_decimation < 1)
    {
        LOG_WARN("ins: gnss_pos_decimation %d is not a valid factor, "
                 "position decimation disabled",
                 f->opt.gnss_pos_decimation);
        f->opt.gnss_pos_decimation = 1;
    }
    {
        struct
        {
            float*      start;
            float*      use;
            float*      stop;
            const char* name;
        } gate[4] = {{&f->opt.gnss_start_max_horizontal_pos_stddev_m,
                      &f->opt.gnss_max_horizontal_pos_stddev_m,
                      &f->opt.gnss_stop_max_horizontal_pos_stddev_m, "horizontal pos"},
                     {&f->opt.gnss_start_max_vertical_pos_stddev_m,
                      &f->opt.gnss_max_vertical_pos_stddev_m,
                      &f->opt.gnss_stop_max_vertical_pos_stddev_m, "vertical pos"},
                     {&f->opt.gnss_start_max_horizontal_vel_stddev_mps,
                      &f->opt.gnss_max_horizontal_vel_stddev_mps,
                      &f->opt.gnss_stop_max_horizontal_vel_stddev_mps, "horizontal vel"},
                     {&f->opt.gnss_start_max_vertical_vel_stddev_mps,
                      &f->opt.gnss_max_vertical_vel_stddev_mps,
                      &f->opt.gnss_stop_max_vertical_vel_stddev_mps, "vertical vel"}};
        int gi;
        for (gi = 0; gi < 4; ++gi)
        {
            if (*gate[gi].start > *gate[gi].use)
            {
                LOG_WARN("ins: GNSS %s entry gate %.2f looser than the fusion gate %.2f, clamped",
                         gate[gi].name, (double)*gate[gi].start, (double)*gate[gi].use);
                *gate[gi].start = *gate[gi].use;
            }
            if (*gate[gi].stop < *gate[gi].start)
            {
                LOG_WARN("ins: GNSS %s exit gate %.2f stricter than the entry gate %.2f, clamped",
                         gate[gi].name, (double)*gate[gi].stop, (double)*gate[gi].start);
                *gate[gi].stop = *gate[gi].start;
            }
        }
    }

    /* Global chi2 downweight gate (REQ-NAV-046): a configured alpha maps to the
       single scalar threshold chi2inv(1-alpha, 1), alpha 0 falls back to
       INS_DEFAULT_CHI2_GATE. */
    /* @satisfies REQ-NAV-046 */
    {
        const float thr   = (f->opt.chi2_reject_alpha > 0.0f)
                                ? ins_chi2inv_1dof(1.0f - f->opt.chi2_reject_alpha)
                                : INS_DEFAULT_CHI2_GATE;
        f->chi2_thr_gnss  = thr;
        f->chi2_thr_mag   = thr;
        f->chi2_thr_local = thr;
        f->chi2_thr_yaw   = thr;
        LOG_INFO("ins: chi2 outlier gate (1 DOF, all channels): %.2f%s", (double)thr,
                 opt->chi2_disable ? " (chi2_disable: never downweighted)" : "");
    }

    ins_log_effective_config(f);

    /* Both init modes defer the actual start to ins_update, once the streams
       are coherent (REQ-NAV-033). init->llh is kept as a provisional anchor,
       refined by the first GNSS fix under auto_init. */
    f->t_init             = init->time; /* provisional, restamped at start */
    f->is_collecting      = true;
    f->is_initialized     = false;
    f->autoinit_count     = 0;
    f->gnss_quality_ok    = true; /* until the running solution loses it (REQ-NAV-052) */
    f->gnss_bad_since     = 0;
    f->gnss_bad_last      = 0;
    f->gnss_bad_accum_sec = 0.0f;
    return 0;
}

/* Manual (prescribed-value) init: apply the caller's attitude, origin and
 * velocity, stamped at t_start (the first good epoch, not the possibly stale
 * init->time). pos_local starts at zero. Shares ins_finalize_init with the
 * auto-init bootstrap. */
/* @satisfies REQ-NAV-033 */
static void ins_finalize_manual(ins_t* f, ins_time_us_t t_start)
{
    const float rpy[3] = {f->init.rpy_init_rad[0], f->init.rpy_init_rad[1],
                          f->init.rpy_init_rad[2]};

    /* Position and velocity are already in the frames the filter works in
       (REQ-NAV-081), so the start is a copy in both cases. */
    const double* llh = f->init.llh;
    float         vel_ned[3];
    vec3_copy(f->init.vel_ned, vel_ned);

    const float pos_local0[3] = {0.0f, 0.0f, 0.0f};
    const float rpy_var[3]    = {qsquare(f->init.rpy_init_stddev_rad[0]),
                                 qsquare(f->init.rpy_init_stddev_rad[1]),
                                 qsquare(f->init.rpy_init_stddev_rad[2])};
    /* The caller's own llh, handed straight through: the origin is held in
       exactly the form it arrives in, so the manual init converts nothing
       (REQ-NAV-080). */
    ins_finalize_init(f, rpy, llh, pos_local0, NULL, vel_ned, rpy_var, t_start);
}

/* Startup stream-coherence check (REQ-NAV-033): true once the measurement
 * streams are usable together. With a limited dead-reckoning budget both a
 * valid IMU sample and a usable position fix (GNSS or local NED) are needed
 * within max_prediction_time_sec of each other; with unlimited dead reckoning a
 * prescribed-value init may start on IMU alone. Auto-init does not use this
 * gate, it runs its own start condition in ins_autoinit_try. */
static bool ins_start_gate_ready(const ins_t* f)
{
    if (!f->have_pending_imu) return false;
    if (f->opt.allow_unlimited_deadreckoning) return true;
    if (!f->have_pending_fix) return false;
    ins_time_us_t gap = f->t_pending_imu - f->t_pending_fix;
    if (gap < 0) gap = -gap;
    return gap <= (ins_time_us_t)(f->opt.max_prediction_time_sec * (float)INS_US_PER_SEC);
}

void ins_shutdown(ins_t* f)
{
    if (f != NULL) { f->is_initialized = false; }
}

/* @satisfies REQ-NAV-032 */
void ins_set_world_model(ins_t* f, const float gravity_n[3], const float magnetic_n[3])
{
    if (f == NULL) return;
    if (gravity_n != NULL) vec3_copy(gravity_n, f->gravity_n);
    if (magnetic_n != NULL) vec3_copy(magnetic_n, f->magnetic_n);
}

/* @satisfies REQ-NAV-027 */
void ins_set_magnetic_model_from_position(ins_t* f, double lat_rad, double lon_rad, float year)
{
    if (f == NULL) { return; }
    const float lat_deg = (float)RAD2DEG(lat_rad);
    const float lon_deg = (float)RAD2DEG(lon_rad);
    /* First WMM is from 1990, anything earlier is probably invalid */
    if (!isfinite(lat_deg) || !isfinite(lon_deg) || !isfinite(year) || year < 1990.0f)
    {
        LOG_WARN("ins: invalid magnetic model init: year %.1f lat %f lon %f", (double)year,
                 (double)lat_deg, (double)lon_deg);
        return; /* drop at the API boundary */
    }

    /* Inside a dip pole exclusion zone the declination that orients the
       reference field is meaningless, so magnetometer fusion is dropped until
       the position leaves the zone (REQ-SYS-018). Keep the last reference
       rather than overwriting it with a bogus one. */
    const bool was_usable = f->mag_heading_usable;
    f->mag_heading_usable = magnetic_heading_reference_valid(lat_deg, lon_deg);
    if (!f->mag_heading_usable)
    {
        if (was_usable)
        {
            LOG_INFO("ins: magnetic dip pole zone entered, magnetometer fusion suspended");
        }
        return;
    }
    if (!was_usable) { LOG_INFO("ins: magnetic dip pole zone left, magnetometer fusion resumed"); }

    float b_ned[3];
    magnetic_field_ned_uT(lat_deg, lon_deg, year, b_ned);
    vec3_copy(b_ned, f->magnetic_n);
    /* Keep the init copy consistent so a later auto-init yaw bootstrap
       (which reads init.magnetic_n) uses the same reference field. */
    vec3_copy(b_ned, f->init.magnetic_n);
    f->mag_field_expected_uT = magnetic_field_strength_uT(lat_deg, lon_deg);
}

/* Pure vertical datum shift: the absolute position stays constant because the
 * origin height drops by exactly the amount pos_local's down component
 * shrinks. Down is the ellipsoid normal, so only origin_llh[2] moves and the
 * origin's latitude and longitude come out bit-for-bit unchanged
 * (REQ-NAV-080). latlonh, the Earth-param cache and the covariance stay
 * untouched. History pos_local entries shift too. */
/* @satisfies REQ-NAV-025 REQ-NAV-054 REQ-NAV-080 */
void ins_shift_origin_down(ins_t* f, float dz_m)
{
    if (f == NULL || !f->is_initialized || !isfinite(dz_m)) { return; }
    int i;
    /* Down is along the ellipsoid normal, which is what the height is
       measured along: the shift is the height alone, and latitude and
       longitude come out bit-for-bit unchanged (REQ-NAV-080). */
    f->origin_llh[2] -= (double)dz_m;
    f->state.pos_local[2] -= dz_m;
    for (i = 0; i < INS_HISTORY_ITEMS_MAX; ++i) { f->history[i].state.pos_local[2] -= dz_m; }

    /* REQ-NAV-054: baro_h0_m is this filter's own anchor for the barometric
       height measurement, relative to the same origin as pos_local. Shifting
       one without the other would leave ins_fuse_baro_height forming a residual
       against the wrong reference. */
    if (f->height_from_baro) { f->baro_h0_m -= dz_m; }
}

/* @satisfies REQ-NAV-021 REQ-NAV-022 REQ-NAV-052 */
bool ins_is_ready(const ins_t* f)
{
    if (f == NULL || !f->is_initialized) return false;
    if (!f->gnss_quality_ok) return false;
    if (f->opt.allow_unlimited_deadreckoning) return true;
    const ins_time_us_t t_now = ins_last_time(f);
    if (ins_dr_expired(f, t_now))
    {
        return false; /* coasting time exceeded (max_deadreckoning_sec) */
    }
    const int ms = time_diff_ms(t_now, f->t_init);
    return (f->kalman_epochs > INS_MIN_KALMAN_EPOCHS_UNTIL_READY) &&
           (ms >= INS_MIN_RUNTIME_UNTIL_READY_MS);
}

/* @satisfies REQ-NAV-022 */
int ins_deadreckoning_ms(const ins_t* f)
{
    if (f == NULL || !f->is_initialized) return -1;
    const int ms = time_diff_ms(ins_last_time(f), f->t_last_pos_aiding);
    return (ms < 0) ? 0 : ms;
}

const ins_diag_t* ins_get_diag(const ins_t* f)
{
    if (f == NULL) return NULL;
    return &f->diag;
}

/* @satisfies REQ-NAV-013 */
bool ins_auto_zupt_active(const ins_t* f)
{
    return f != NULL && f->is_initialized && !f->opt.auto_zupt_disable &&
           f->auto_zupt_static_since != 0 && f->static_var_ok;
}

/* @satisfies REQ-NAV-013 */
void ins_set_auto_zupt_disable(ins_t* f, bool disable)
{
    if (f == NULL) return;
    f->opt.auto_zupt_disable = disable;
    if (disable)
    {
        /* ins_auto_zupt_detect self-heals on every call while disabled, this
           only makes the disabled state take effect immediately rather than
           after the next ins_update. */
        f->auto_zupt_static_since = 0;
        f->auto_zupt_var_since    = 0;
        f->auto_zupt_gyr_count    = 0;
        f->bias_prior_count       = 0;
        f->static_var_count       = 0;
        f->static_var_ok          = false;
    }
}

/* ============================================================================
 * Auto-initialization (collect IMU, bootstrap on the first position fix)
 * ============================================================================
 */

/* In-place median of a[0..n-1] (n <= INS_AUTOINIT_SAMPLES_MAX). */
static float ins_medianf(float* a, int n)
{
    int i, j;
    for (i = 1; i < n; ++i) /* insertion sort, n is assumed to be small */
    {
        const float key = a[i];
        for (j = i - 1; j >= 0 && a[j] > key; --j) a[j + 1] = a[j];
        a[j + 1] = key;
    }
    return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

/* Buffer one bias-corrected IMU sample for the leveling window. Oldest
 * samples fall out once the fixed buffer is full. */
static void ins_autoinit_push(ins_t* f, const ins_measurements_t* m)
{
    /* Cache the latest mag independently of the IMU: it may arrive on its
       own update call, and the heading bootstrap needs the most recent one
       even when the bootstrap fix epoch carries none (REQ-NAV-044). */
    if (m->mag.is_valid)
    {
        f->autoinit_mag.t       = m->timestamp;
        f->autoinit_mag.data[0] = m->mag.data[0];
        f->autoinit_mag.data[1] = m->mag.data[1];
        f->autoinit_mag.data[2] = m->mag.data[2];
        f->autoinit_mag.valid   = true;
    }

    /* Track the run of continuously entry-quality position fixes for the
       entry dwell (REQ-NAV-045, REQ-NAV-051). */
    ins_track_gnss_entry_dwell(f, m);

    if (!(m->acc.is_valid && m->gyr.is_valid)) return;

    if (f->autoinit_count >= INS_AUTOINIT_SAMPLES_MAX)
    {
        memmove(&f->autoinit_buf[0], &f->autoinit_buf[1],
                sizeof(f->autoinit_buf[0]) * (INS_AUTOINIT_SAMPLES_MAX - 1));
        f->autoinit_count = INS_AUTOINIT_SAMPLES_MAX - 1;
    }
    const int k          = f->autoinit_count++;
    f->autoinit_buf[k].t = m->timestamp;
    int i;
    for (i = 0; i < 3; ++i)
    {
        f->autoinit_buf[k].acc[i] = m->acc.data[i] - f->init.acc_bias_init_mps2[i];
        f->autoinit_buf[k].gyr[i] = m->gyr.data[i] - f->init.gyr_bias_init_rps[i];
    }
}

/* Resolve the bootstrap yaw. Priority: external heading measurement, then an
 * external attitude hint's own yaw (REQ-NAV-048), then a tilt-compensated
 * magnetometer heading (this epoch's or the most recent cached sample), else
 * "unknown". Writes the variance to *yaw_var and returns the yaw [rad]. */
/* @satisfies REQ-NAV-044 REQ-NAV-048 */
static float ins_autoinit_yaw(const ins_t* f, const ins_measurements_t* m, float roll, float pitch,
                              float* yaw_var)
{
    /* 1) External heading (dual-antenna GNSS, lighthouse pose, ...). */
    if (m->yaw.is_valid && m->yaw.stddev_rad > 0.0f)
    {
        LOG_INFO("ins: using supplied initial yaw: %.0f deg", (double)RAD2DEG(m->yaw.yaw_rad));
        *yaw_var = qsquare(m->yaw.stddev_rad);
        return m->yaw.yaw_rad;
    }

    /* 2) External attitude hint's own yaw (REQ-NAV-048), e.g. nav_suite's
       magnetometer AHRS: a converged, continuously-running heading beats the
       single-sample compass fix below. */
    if (m->att_hint.is_valid && m->att_hint.stddev_yaw_rad > 0.0f)
    {
        LOG_INFO("ins: using yaw hint: %.0f deg", (double)RAD2DEG(m->att_hint.yaw_rad));
        *yaw_var = qsquare(m->att_hint.stddev_yaw_rad);
        return m->att_hint.yaw_rad;
    }

    /* 3) Tilt-compensated magnetic heading: rotate the body field into a level
       (yaw-free) frame, then compare its heading against the model. Use this
       epoch's mag if present, else the most recent cached sample: GNSS and the
       magnetometer run on independent clocks (REQ-NAV-044). */
    const float mh = (f->init.magnetic_n[0] * f->init.magnetic_n[0] +
                      f->init.magnetic_n[1] * f->init.magnetic_n[1]);

    const float* mag_data = NULL;
    if (m->mag.is_valid) { mag_data = m->mag.data; }
    else if (f->autoinit_mag.valid)
    {
        ins_time_us_t age = m->timestamp - f->autoinit_mag.t;
        if (age < 0) age = -age;
        if (age <= (ins_time_us_t)(f->opt.max_prediction_time_sec * (float)INS_US_PER_SEC))
        {
            mag_data = f->autoinit_mag.data;
        }
    }
    /* Starting inside a dip pole exclusion zone: the model heading below would
       seed yaw from a meaningless declination, so fall through to the unknown
       heading case instead (REQ-SYS-018). The filter still starts, roll and
       pitch are unaffected, and yaw carries an honest large covariance that
       GNSS course pulls in. */
    if (mag_data != NULL && mh > 1e-6f && !f->mag_heading_usable)
    {
        LOG_INFO("ins: magnetometer yaw bootstrap skipped, inside a magnetic dip pole zone");
    }
    else if (mag_data != NULL && mh > 1e-6f)
    {
        float q0[4], R0[9], m_l[3];
        ins_quat_from_rpy(roll, pitch, 0.0f, q0);
        ins_quat_to_rotmat(q0, R0);
        mat3_mul_vec3(R0, mag_data, m_l);

        const float meas_hdg  = atan2f(m_l[1], m_l[0]);
        const float model_hdg = atan2f(f->init.magnetic_n[1], f->init.magnetic_n[0]);
        *yaw_var              = qsquare(f->init.rpy_init_stddev_rad[2]);

        LOG_INFO("ins: using magnetometer yaw: %.0f deg", (double)RAD2DEG(model_hdg - meas_hdg));
        return model_hdg - meas_hdg;
    }

    /* 4) Unknown: large covariance, let the running fusion pull it in. */
    *yaw_var = qsquare(INS_YAW_UNKNOWN_STDDEV);
    LOG_WARN("ins: initial heading unknown");
    return 0.0f;
}

/* May the pending origin carry (REQ-NAV-062) be used for a bootstrap on
 * fix_llh at time t, and if so, where does that fix sit in the inherited
 * n-frame? On true, pos_local_out holds the fix expressed in the inherited
 * frame. On false it is untouched and the caller anchors a fresh origin.
 *
 * The mapping is the one ins_reacquire uses after a coasting window
 * (REQ-NAV-023): a geodetic difference, not the flat-earth R_n_to_e * pos
 * product whose chord-vs-arc error reaches hundreds of metres in height over
 * the distance an outage can cover. It is taken against the position the
 * exiting instance last held rather than against the origin, so the step
 * stays as short as the outage - a tangent-plane mapping is only exact for a
 * short step, and the origin may be hundreds of kilometres behind by now.
 * The absolute anchor is the fix itself and never runs through the mapping.
 *
 * The refusal test is "can this be the same platform continuing": how far the
 * fix sits from the position the exiting instance last held, against what the
 * outage could have covered. Distance to the origin does not enter it.
 *
 * @satisfies REQ-NAV-062 */
static bool ins_autoinit_origin_carry_usable(const ins_t* f, const double fix_llh[3],
                                             ins_time_us_t t, float pos_local_out[3],
                                             double fix_llh_out[3])
{
    /* Step from where the exiting instance last was to the fix. */
    const double dllh[3] = {fix_llh[0] - f->origin_carry.latlonh[0],
                            fix_llh[1] - f->origin_carry.latlonh[1],
                            fix_llh[2] - f->origin_carry.latlonh[2]};
    float        step[3], pos[3];
    ins_dlatlonh_to_dned(dllh, f->origin_carry.latlonh[0], f->origin_carry.latlonh[2], step);
    int k;
    for (k = 0; k < 3; ++k) { pos[k] = f->origin_carry.pos_local[k] + step[k]; }

    if (!isfinite(pos[0]) || !isfinite(pos[1]) || !isfinite(pos[2])) { return false; }

#if LOG_LEVEL >= LOG_LEVEL_INFO
    /* Diagnostic only: the distance to the carried origin does not enter the
       refusal test below, it only makes the "origin kept" line readable.
       Explicitly guarded so it is not left unused in a LOG_LEVEL_NONE build. */
    const double dn = (double)pos[0], de = (double)pos[1], dd = (double)pos[2];
    const double dist = sqrt(dn * dn + de * de + dd * dd);
#endif

    /* Travelled since the exiting instance's last known position. */
    const double tn        = (double)step[0];
    const double te        = (double)step[1];
    const double td        = (double)step[2];
    const double travelled = sqrt(tn * tn + te * te + td * td);
    /* Negative (clock stepped back) is not a licence to travel: clamped. */
    const double unobserved_sec = (double)time_diff_sec(t, f->origin_carry.t);
    const double budget =
        INS_ORIGIN_CARRY_MIN_TRAVEL_M +
        INS_ORIGIN_CARRY_MAX_SPEED_MPS * ((unobserved_sec > 0.0) ? unobserved_sec : 0.0);

    if (travelled > budget)
    {
        LOG_WARN("ins: bootstrap fix is %.1f km from where the filter last was, more than the "
                 "%.1f km reachable in the %.1f s without aiding - not the same platform "
                 "continuing, anchoring a fresh origin (local NED coordinates issued before the "
                 "outage no longer refer to the same frame)",
                 travelled / 1000.0, budget / 1000.0, unobserved_sec);
        return false;
    }

#if LOG_LEVEL >= LOG_LEVEL_INFO
    /* Same guard as the dist computation above. */
    LOG_INFO("ins: bootstrap keeps the carried n-frame origin (%.1f km from it, %.1f km travelled "
             "during the %.1f s without aiding), local NED stays continuous",
             dist / 1000.0, travelled / 1000.0, unobserved_sec);
#endif
    vec3_copy(pos, pos_local_out);
    fix_llh_out[0] = fix_llh[0];
    fix_llh_out[1] = fix_llh[1];
    fix_llh_out[2] = fix_llh[2];
    return true;
}

/* Attempt the bootstrap. Needs a usable position fix this epoch plus a
 * quasi-static IMU window for accelerometer leveling. Returns true once the
 * filter has been initialized. */
/* @satisfies REQ-NAV-015 REQ-NAV-045 REQ-NAV-047 REQ-NAV-048 REQ-NAV-051 REQ-NAV-053
   REQ-NAV-062 */
static bool ins_autoinit_try(ins_t* f, const ins_measurements_t* m)
{
    /* Entry-quality gate (REQ-NAV-051): the bootstrap fix itself must be
       good enough to enter the 3D solution, which is stricter than merely
       being good enough to fuse. */
    const bool have_gnss  = m->gnss_pos.is_valid && ins_gnss_entry_quality_ok(f, m);
    const bool have_local = m->local_pos.is_valid;
    if (!have_gnss && !have_local) return false;
    if (f->autoinit_count < 1) return false;

    /* GNSS-stability dwell (REQ-NAV-045): do not enter 3D until the fix stream
       has passed the entry gate continuously for gnss_init_dwell_sec AND
       carried at least 1 fix/s over that window. The run is tracked in
       ins_autoinit_push and reset by a rejected fix / gap / re-acquisition. */
    if (!ins_entry_dwell_satisfied(f, m->timestamp)) return false;

    /* Stream coherence (REQ-NAV-033): the fix must be roughly concurrent with
       the newest buffered IMU sample, otherwise the attitude/origin would be
       bootstrapped from temporally incoherent data. */
    {
        ins_time_us_t gap = m->timestamp - f->autoinit_buf[f->autoinit_count - 1].t;
        if (gap < 0) gap = -gap;
        if (gap > (ins_time_us_t)(f->opt.max_prediction_time_sec * (float)INS_US_PER_SEC))
            return false;
    }

    /* Collect the samples inside the leveling window (newest .. -window). */
    const float         win   = (f->opt.auto_init_window_sec > 0.0f) ? f->opt.auto_init_window_sec
                                                                     : INS_AUTOINIT_DEFAULT_WINDOW_SEC;
    const ins_time_us_t t_new = f->autoinit_buf[f->autoinit_count - 1].t;
    const ins_time_us_t t_min = t_new - (ins_time_us_t)(win * (float)INS_US_PER_SEC);

    float ax[INS_AUTOINIT_SAMPLES_MAX];
    float ay[INS_AUTOINIT_SAMPLES_MAX];
    float az[INS_AUTOINIT_SAMPLES_MAX];
    float gmax = 0.0f;
    int   cnt  = 0, i;
    for (i = 0; i < f->autoinit_count; ++i)
    {
        if (f->autoinit_buf[i].t < t_min) continue;
        ax[cnt]        = f->autoinit_buf[i].acc[0];
        ay[cnt]        = f->autoinit_buf[i].acc[1];
        az[cnt]        = f->autoinit_buf[i].acc[2];
        const float gn = vec3_norm(f->autoinit_buf[i].gyr);
        if (gn > gmax) gmax = gn;
        ++cnt;
    }
    if (cnt < 3) return false; /* not enough IMU in the window yet */

    /* Quasi-static classification: leveling assumes the specific force is ~
       gravity and the platform is nearly not rotating. Movement no longer
       defers the bootstrap (a boat or a taxiing aircraft must still be able to
       auto-init), instead the roll/pitch uncertainty below is widened when this
       does not hold (REQ-NAV-047). */
    const float max_gyr = (f->opt.auto_init_static_gyr_rps > 0.0f)
                              ? f->opt.auto_init_static_gyr_rps
                              : INS_AUTOINIT_DEFAULT_STATIC_GYR;
    const float max_acc = (f->opt.auto_init_static_acc_mps2 > 0.0f)
                              ? f->opt.auto_init_static_acc_mps2
                              : INS_AUTOINIT_DEFAULT_STATIC_ACC;

    const float fx = ins_medianf(ax, cnt);
    const float fy = ins_medianf(ay, cnt);
    const float fz = ins_medianf(az, cnt);
    const bool  is_quasi_static =
        (gmax <= max_gyr) &&
        (fabsf(SQRTF(fx * fx + fy * fy + fz * fz) - INS_GRAVITY_NOMINAL) <= max_acc);

    /* Accelerometer leveling. Static: R_b_to_n * f_b = -g_n, so in the body
       frame f_b = [ g sin(pitch), -g sin(roll) cos(pitch),
                     -g cos(roll) cos(pitch) ]. Under motion this is only
       approximate (the median partially rejects it), which is why the
       resulting roll/pitch stddev is widened below. */
    float roll  = atan2f(-fy, -fz);
    float pitch = atan2f(fx, SQRTF(fy * fy + fz * fz));

    /* @satisfies REQ-NAV-047 */
    const float moving_floor = (f->opt.auto_init_moving_rpy_stddev_rad > 0.0f)
                                   ? f->opt.auto_init_moving_rpy_stddev_rad
                                   : INS_AUTOINIT_DEFAULT_MOVING_RPY_STDDEV;
    float       roll_var, pitch_var;
    /* REQ-NAV-048: an external attitude hint (e.g. nav_suite's ARS/AHRS) has
       integrated far more history than this single leveling window and, unlike
       the median above, isn't defeated by real motion during it. */
    if (m->att_hint.is_valid && m->att_hint.stddev_roll_rad > 0.0f &&
        m->att_hint.stddev_pitch_rad > 0.0f)
    {
        roll      = m->att_hint.roll_rad;
        pitch     = m->att_hint.pitch_rad;
        roll_var  = qsquare(fmaxf(m->att_hint.stddev_roll_rad, m->att_hint.stddev_pitch_rad));
        pitch_var = roll_var;
        LOG_INFO("ins: initial alignment using attitude hint: roll %.0f deg pitch %.0f deg",
                 (double)RAD2DEG(roll), (double)RAD2DEG(pitch));
    }
    else
    {
        const float roll_stddev  = is_quasi_static
                                       ? f->init.rpy_init_stddev_rad[0]
                                       : fmaxf(f->init.rpy_init_stddev_rad[0], moving_floor);
        const float pitch_stddev = is_quasi_static
                                       ? f->init.rpy_init_stddev_rad[1]
                                       : fmaxf(f->init.rpy_init_stddev_rad[1], moving_floor);
        roll_var                 = qsquare(roll_stddev);
        pitch_var                = qsquare(pitch_stddev);
        if (!is_quasi_static)
        {
            f->diag.n_autoinit_moving++;
            LOG_WARN("ins: auto-init leveling window was not quasi-static, roll/pitch "
                     "stddev widened to >= %.2g deg",
                     (double)RAD2DEG(moving_floor));
        }
    }

    float       yaw_var    = 0.0f;
    const float yaw        = ins_autoinit_yaw(f, m, roll, pitch, &yaw_var);
    const float rpy[3]     = {roll, pitch, yaw};
    const float rpy_var[3] = {roll_var, pitch_var, yaw_var};

    /* Origin + velocity from the fix. */
    double origin_llh[3];
    double fix_llh[3];
    /* Non-NULL only for a carried origin: a fresh origin IS the fix, so the
       offset mapping below is a no-op and needs no help. */
    const double* anchor_llh   = NULL;
    float         pos_local[3] = {0.0f, 0.0f, 0.0f};
    float         vel_ned[3]   = {0.0f, 0.0f, 0.0f};
    if (have_gnss)
    {
        /* REQ-NAV-062: a quality-loss re-arm hands its n-frame origin down to
           this bootstrap, so the local frame survives the outage. The fix then
           sets pos_local instead of the origin, via the same geodetic mapping
           the re-acquisition of REQ-NAV-023 uses. */
        const double* fix_llh_in = m->gnss_pos.llh;
        const bool    carry =
            f->origin_carry.valid &&
            ins_autoinit_origin_carry_usable(f, fix_llh_in, m->timestamp, pos_local, fix_llh);
        if (carry)
        {
            origin_llh[0] = f->origin_carry.origin_llh[0];
            origin_llh[1] = f->origin_carry.origin_llh[1];
            origin_llh[2] = f->origin_carry.origin_llh[2];
            anchor_llh    = fix_llh;
        }
        else
        {
            /* The fix becomes the origin as it stands: both are geodetic, so
               there is nothing to convert (REQ-NAV-080). */
            origin_llh[0] = fix_llh_in[0];
            origin_llh[1] = fix_llh_in[1];
            origin_llh[2] = fix_llh_in[2];
            LOG_INFO("ins: Reset of local NED frame");
        }
        if (ins_gnss_vel_usable(f, &m->gnss_vel)) { vec3_copy(m->gnss_vel.vel_ned, vel_ned); }
    }
    else /* have_local: keep the caller's n-frame origin (the init block) so
            the local measurements stay consistent, the fix sets pos_local. */
    {
        origin_llh[0] = f->init.llh[0];
        origin_llh[1] = f->init.llh[1];
        origin_llh[2] = f->init.llh[2];
        vec3_copy(m->local_pos.pos_ned, pos_local);
    }

    ins_finalize_init(f, rpy, origin_llh, pos_local, anchor_llh, vel_ned, rpy_var, m->timestamp);
    /* Bootstrap: cap the hinted 1-sigma at the cold-start prior
       (REQ-NAV-048, REQ-NAV-067). */
    ins_apply_gyr_bias_hint(f, &m->att_hint, true);

    /* Height-source selection (REQ-NAV-053): latched once, here, for the
       lifetime of this filter instance. Only considered for a GNSS bootstrap: a
       local-position system already supplies a vertical reference typically far
       better than a barometer, and REQ-NAV-053 exists specifically for GNSS's
       poor vertical accuracy. A barometer seen during collecting
       (f->autoinit_baro) selects barometric height (REQ-NAV-054) with
       h_init = 0, matched to baro_alt's own height by nav_suite's
       vertical-datum alignment right after this call (REQ-SUITE-007).
       Everything else selects GNSS/local_pos height. */

    const bool baro_fresh =
        f->autoinit_baro.valid &&
        time_diff_sec(m->timestamp, f->autoinit_baro.t) <= INS_BARO_ANCHOR_MAX_AGE_SEC;
    f->height_from_baro = have_gnss && baro_fresh && !f->opt.baro_height_disable;
    if (f->height_from_baro)
    {
        /* The anchor is defined by the residual ins_fuse_baro_height forms,
           -pos_local[2] - (h_isa - baro_h0_m): it has to make the bootstrap
           sample agree with the vertical position the bootstrap starts from.
           That is pos_local[2] = 0 for a fresh origin, but not for one
           inherited across a re-arm (REQ-NAV-062). */
        f->baro_h0_m =
            ins_isa_altitude_from_pressure(f->autoinit_baro.pressure_pa) + f->state.pos_local[2];
        /* Start the aiding-gap clock here rather than at the first successful
           fusion (REQ-NAV-058). A barometer that delivered during the
           collecting window, got the source selected and then never delivered
           again never produces a first fusion, so a gap measured from there
           could never begin. */
        f->log_state.t_last_baro_height_aid = m->timestamp;
        LOG_INFO("ins: height source: barometric (barometer streaming during auto-init)");
    }
    else if (have_gnss && f->autoinit_baro.valid && !f->opt.baro_height_disable)
    {
        LOG_WARN("ins: height source: GNSS -- a barometer was seen during auto-init but its "
                 "last sample is %.1f s old, too stale to anchor the height channel",
                 (double)time_diff_sec(m->timestamp, f->autoinit_baro.t));
    }
    else { LOG_INFO("ins: height source: GNSS/local position"); }
    return true;
}

/* ============================================================================
 * Automatic ZUPT/ZARU detector (see opt.auto_zupt_* in ins.h)
 * ============================================================================
 */

/* Staticness gate's max_vel check: exclusively a recent external (GNSS)
 * velocity observation (the filter's own state estimate would be circular),
 * used whenever a usable sample was seen within
 * INS_AUTOZUPT_EXT_VEL_MAX_AGE_SEC.
 *
 * Without a recent GNSS velocity (pure-inertial coasting, e.g. a tunnel) the
 * check is not applied and only the magnitude/variance IMU check decides. That
 * accepts a known blind spot: a body cruising in a straight line at constant
 * velocity also shows gravity-only specific force and near-zero rotation. It is
 * narrow in practice, since the variance criterion already rejects the
 * road/engine vibration a moving vehicle puts on the IMU.
 *
 * A usable fix also has to be accurate enough
 * (opt.auto_zupt_max_vel_stddev_mps, 1-sigma per axis). */
static bool ins_auto_zupt_velocity_gate_ok(ins_t* f, const ins_measurements_t* m, float max_vel,
                                           float max_vel_stddev)
{
    if (ins_gnss_vel_usable(f, &m->gnss_vel) &&
        ins_cov_stddev_within(m->gnss_vel.Qll_ned, max_vel_stddev, max_vel_stddev))
    {
        f->auto_zupt_ext_vel_mps  = vec3_norm(m->gnss_vel.vel_ned);
        f->auto_zupt_ext_vel_time = m->timestamp;
    }
    if (f->auto_zupt_ext_vel_time != 0 &&
        time_diff_sec(m->timestamp, f->auto_zupt_ext_vel_time) <= INS_AUTOZUPT_EXT_VEL_MAX_AGE_SEC)
    {
        return f->auto_zupt_ext_vel_mps <= max_vel;
    }
    return true;
}

/* Decide whether this epoch should arm a synthetic zero_velocity_update /
 * zero_rotation_update. IMU-only static gate, plus a requirement that the
 * platform's velocity is already small when a recent GNSS observation of it
 * exists (see ins_auto_zupt_velocity_gate_ok). Rate-limited by
 * opt.auto_zupt_min_interval_sec once armed. */
/* @satisfies REQ-NAV-013 REQ-NAV-014 */
/* Primary stillness criterion of the auto-ZUPT/ZARU detector (REQ-NAV-013):
 * the per-axis sample variance of the RAW IMU over a short tumbling window.
 * Bias-invariant by construction (see sensor_defaults.h). Raw, not
 * bias-corrected: the variance does not care about the mean, so correcting it
 * would only couple the verdict to a wobbling bias estimate.
 *
 * Accumulated with Welford's online algorithm rather than a running
 * sum-of-squares: at float32 precision the accelerometer samples sit around 9.8
 * (squares around 96), so E[x^2] - E[x]^2 loses the entire variance to
 * cancellation before the window is full. Welford is the same O(1) per sample
 * and stays bounded (REQ-SYS-004).
 *
 * The window tumbles: it accumulates unconditionally, and on completion latches
 * a verdict and starts over. Before the first completed window the verdict is
 * "not still".
 *
 * @return true if the most recently completed window looked stationary. */
static bool ins_static_variance_update(ins_t* f, const ins_measurements_t* m)
{
    const float x[6] = {m->gyr.data[0], m->gyr.data[1], m->gyr.data[2],
                        m->acc.data[0], m->acc.data[1], m->acc.data[2]};
    int         i;

    if (f->static_var_count == 0)
    {
        for (i = 0; i < 6; ++i)
        {
            f->static_var_mean[i] = x[i];
            f->static_var_m2[i]   = 0.0f;
        }
        f->static_var_window_since = m->timestamp;
        f->static_var_count        = 1;
        return f->static_var_ok;
    }

    f->static_var_count++;
    for (i = 0; i < 6; ++i)
    {
        const float delta = x[i] - f->static_var_mean[i];
        f->static_var_mean[i] += delta / (float)f->static_var_count;
        f->static_var_m2[i] += delta * (x[i] - f->static_var_mean[i]);
    }

    if (f->static_var_count < INS_DEFAULT_STATIC_VAR_MIN_SAMPLES) { return f->static_var_ok; }
    if (time_diff_sec(m->timestamp, f->static_var_window_since) < INS_DEFAULT_STATIC_VAR_WINDOW_SEC)
    {
        return f->static_var_ok;
    }

    const float max_gyr_rms = (f->opt.auto_zupt_static_gyr_stddev_rps > 0.0f)
                                  ? f->opt.auto_zupt_static_gyr_stddev_rps
                                  : INS_DEFAULT_STATIC_GYR_STDDEV_RPS;
    const float max_acc_rms = (f->opt.auto_zupt_static_acc_stddev_mps2 > 0.0f)
                                  ? f->opt.auto_zupt_static_acc_stddev_mps2
                                  : INS_DEFAULT_STATIC_ACC_STDDEV_MPS2;

    /* Window complete: latch the verdict from the per-axis RMS stddev
       (the three axes pooled, so a single moving axis still shows up). */
    const float inv_dof = 1.0f / (3.0f * (float)(f->static_var_count - 1u));
    const float gyr_rms =
        SQRTF((f->static_var_m2[0] + f->static_var_m2[1] + f->static_var_m2[2]) * inv_dof);
    const float acc_rms =
        SQRTF((f->static_var_m2[3] + f->static_var_m2[4] + f->static_var_m2[5]) * inv_dof);
    f->static_var_ok    = (gyr_rms <= max_gyr_rms) && (acc_rms <= max_acc_rms);
    f->static_var_count = 0; /* start the next window */
    return f->static_var_ok;
}

/* Hold the averaged raw IMU of the current stillness window against the
   configured initial bias 1-sigma priors (see INS_BIAS_PRIOR_* above).
   Diagnostic only: counts and warns, never touches the filter state.
   Called once per epoch while the auto-ZUPT detector sees stillness,
   evaluates (and restarts) the window once it is long/dense enough. */
/* @satisfies REQ-NAV-050 */
static void ins_check_bias_prior(ins_t* f, const ins_measurements_t* m)
{
    if (f->bias_prior_count < INS_BIAS_PRIOR_MIN_SAMPLES) { return; }
    if (time_diff_sec(m->timestamp, f->bias_prior_since) < INS_BIAS_PRIOR_MIN_DWELL_SEC) { return; }

    const float inv         = 1.0f / (float)f->bias_prior_count;
    const float acc_mean[3] = {f->bias_prior_acc_sum[0] * inv, f->bias_prior_acc_sum[1] * inv,
                               f->bias_prior_acc_sum[2] * inv};
    const float gyr_mean[3] = {f->bias_prior_gyr_sum[0] * inv, f->bias_prior_gyr_sum[1] * inv,
                               f->bias_prior_gyr_sum[2] * inv};
    f->bias_prior_count     = 0; /* window consumed: start a fresh one either way */

    const float acc_dev = fabsf(vec3_norm(acc_mean) - vec3_norm(f->gravity_n));
    const float acc_thr = INS_BIAS_PRIOR_SIGMA_FACTOR * f->init.acc_bias_init_stddev_mps2;
    /* Earth rate floor: a stationary gyro reads it, so a tighter prior can
       never be met and would warn forever (matters for navigation-grade
       sensors, not for the MEMS default). */
    const float gyr_dev = vec3_norm(gyr_mean);
    const float gyr_thr = INS_BIAS_PRIOR_SIGMA_FACTOR *
                          fmaxf(f->init.gyr_bias_init_stddev_rps, (float)INS_WGS84_OMEGA);

    if (acc_dev > acc_thr)
    {
        f->diag.n_acc_bias_prior_exceeded++;
        const bool  first_warn = (f->log_state.t_last_acc_bias_prior_warn == 0);
        const float since_warn_sec =
            first_warn ? 0.0f
                       : time_diff_sec(m->timestamp, f->log_state.t_last_acc_bias_prior_warn);
        if (first_warn || since_warn_sec >= INS_LOG_BIAS_PRIOR_REPEAT_SEC)
        {
            LOG_WARN("ins: standing still, but avg. accelerometer is %.3f m/s^2 off "
                     "gravity: more than %.0f sigma of the config init. accel bias "
                     "(%.3f m/s^2), prior is too tight for this sensor",
                     (double)acc_dev, (double)INS_BIAS_PRIOR_SIGMA_FACTOR,
                     (double)f->init.acc_bias_init_stddev_mps2);
            f->log_state.t_last_acc_bias_prior_warn = m->timestamp;
        }
    }
    if (gyr_dev > gyr_thr)
    {
        f->diag.n_gyr_bias_prior_exceeded++;
        const bool  first_warn = (f->log_state.t_last_gyr_bias_prior_warn == 0);
        const float since_warn_sec =
            first_warn ? 0.0f
                       : time_diff_sec(m->timestamp, f->log_state.t_last_gyr_bias_prior_warn);
        if (first_warn || since_warn_sec >= INS_LOG_BIAS_PRIOR_REPEAT_SEC)
        {
            LOG_WARN("ins: standing still, but avg. gyro reads %.3f deg/s: more than "
                     "%.0f sigma of the config init. gyro bias (%.3f deg/s), prior is "
                     "too tight for this sensor",
                     (double)RAD2DEG(gyr_dev), (double)INS_BIAS_PRIOR_SIGMA_FACTOR,
                     (double)RAD2DEG(f->init.gyr_bias_init_stddev_rps));
            f->log_state.t_last_gyr_bias_prior_warn = m->timestamp;
        }
    }
}

static bool ins_auto_zupt_detect(ins_t* f, const ins_measurements_t* m)
{
    if (f->opt.auto_zupt_disable)
    {
        /* Self-heals every call while disabled (mirrors ahrs_auto_zaru_detect):
           a caller flipping opt.auto_zupt_disable at runtime must not have a
           dwell timer or a latched variance verdict from before the disable
           window survive into re-enablement. */
        f->auto_zupt_static_since = 0;
        f->auto_zupt_var_since    = 0;
        f->auto_zupt_gyr_count    = 0;
        f->bias_prior_count       = 0;
        f->static_var_count       = 0;
        f->static_var_ok          = false;
        return false;
    }
    if (!m->acc.is_valid || !m->gyr.is_valid)
    {
        f->auto_zupt_static_since = 0;
        f->auto_zupt_gyr_count    = 0;
        f->bias_prior_count       = 0;
        /* Deliberately NOT touching the variance window here: an epoch without
           an IMU sample (a barometer-only epoch, a dropped frame) is an
           absence of evidence, not evidence of motion. Clearing the latched
           verdict would let every interleaved sensor epoch veto the detector
           until a fresh window completes. */
        return false;
    }

    const float max_gyr = (f->opt.auto_zupt_static_gyr_rps > 0.0f)
                              ? f->opt.auto_zupt_static_gyr_rps
                              : INS_AUTOZUPT_DEFAULT_STATIC_GYR;
    const float max_acc = (f->opt.auto_zupt_static_acc_mps2 > 0.0f)
                              ? f->opt.auto_zupt_static_acc_mps2
                              : INS_AUTOZUPT_DEFAULT_STATIC_ACC;
    const float max_vel = (f->opt.auto_zupt_max_vel_mps > 0.0f) ? f->opt.auto_zupt_max_vel_mps
                                                                : INS_AUTOZUPT_DEFAULT_MAX_VEL_MPS;
    const float max_vel_stddev   = (f->opt.auto_zupt_max_vel_stddev_mps > 0.0f)
                                       ? f->opt.auto_zupt_max_vel_stddev_mps
                                       : INS_AUTOZUPT_DEFAULT_MAX_VEL_STDDEV_MPS;
    const float dwell_sec        = (f->opt.auto_zupt_dwell_sec > 0.0f) ? f->opt.auto_zupt_dwell_sec
                                                                       : INS_AUTOZUPT_DEFAULT_DWELL_SEC;
    const float min_interval_sec = (f->opt.auto_zupt_min_interval_sec > 0.0f)
                                       ? f->opt.auto_zupt_min_interval_sec
                                       : INS_AUTOZUPT_DEFAULT_MIN_INTERVAL_SEC;

    const float acc_c[3] = {m->acc.data[0] - f->state.acc_bias[0],
                            m->acc.data[1] - f->state.acc_bias[1],
                            m->acc.data[2] - f->state.acc_bias[2]};
    const float gyr_c[3] = {m->gyr.data[0] - f->state.gyr_bias[0],
                            m->gyr.data[1] - f->state.gyr_bias[1],
                            m->gyr.data[2] - f->state.gyr_bias[2]};
    const float g_nom    = vec3_norm(f->gravity_n);

    /* The variance window is fed unconditionally (a moving platform is exactly
       what has to show up in it) and its verdict is applied below, AFTER the
       dwell timer has been started. Deliberately not folded into is_static: the
       window needs half a second for its first verdict, and a biased IMU leaks
       gravity into the velocity states fast enough that the velocity gate can
       slam shut before the detector ever arms. */
    const bool var_static = ins_static_variance_update(f, m);
    const bool is_static  = (vec3_norm(gyr_c) <= max_gyr) &&
                           (fabsf(vec3_norm(acc_c) - g_nom) <= max_acc) &&
                           ins_auto_zupt_velocity_gate_ok(f, m, max_vel, max_vel_stddev);

    if (!is_static)
    {
        f->auto_zupt_static_since = 0;
        f->auto_zupt_var_since    = 0;
        f->auto_zupt_gyr_count    = 0;
        f->bias_prior_count       = 0;
        return false;
    }
    if (f->auto_zupt_static_since == 0) { f->auto_zupt_static_since = m->timestamp; }

    /* The magnitude/velocity checks above are only bounds: the variance window
       is the actual stillness statement (bias-invariant). Everything below -
       the ZARU gyro average, the bias-prior evidence, the trigger itself -
       requires the platform to really be still. */
    if (!var_static)
    {
        f->auto_zupt_var_since = 0;
        f->auto_zupt_gyr_count = 0;
        f->bias_prior_count    = 0;
        return false;
    }
    if (f->auto_zupt_var_since == 0) { f->auto_zupt_var_since = m->timestamp; }

    /* Initial bias-prior consistency check (REQ-NAV-050): accumulate the
       RAW IMU over the stillness run and evaluate it once the window is
       long enough. Runs before the dwell/rate-limit gates below so it
       keeps averaging even on the epochs where no update is triggered. */
    if (f->bias_prior_count == 0)
    {
        vec3_zero(f->bias_prior_acc_sum);
        vec3_zero(f->bias_prior_gyr_sum);
        f->bias_prior_since = m->timestamp;
    }
    f->bias_prior_acc_sum[0] += m->acc.data[0];
    f->bias_prior_acc_sum[1] += m->acc.data[1];
    f->bias_prior_acc_sum[2] += m->acc.data[2];
    f->bias_prior_gyr_sum[0] += m->gyr.data[0];
    f->bias_prior_gyr_sum[1] += m->gyr.data[1];
    f->bias_prior_gyr_sum[2] += m->gyr.data[2];
    f->bias_prior_count++;
    ins_check_bias_prior(f, m);

    /* Accumulate the raw gyro over the stillness run: the zero-rotation update
       fuses this average instead of the momentary sample, so vibration (e.g.
       from an idling engine) cancels out instead of being injected into the
       gyro bias states. Consumed and cleared by ins_fuse_zero_rotation. */
    if (f->auto_zupt_gyr_count == 0) { vec3_zero(f->auto_zupt_gyr_sum); }
    f->auto_zupt_gyr_sum[0] += m->gyr.data[0];
    f->auto_zupt_gyr_sum[1] += m->gyr.data[1];
    f->auto_zupt_gyr_sum[2] += m->gyr.data[2];
    f->auto_zupt_gyr_count++;

    /* Anchored to auto_zupt_var_since, not auto_zupt_static_since: the
       magnitude/velocity gates can go static within a single sample, but the
       gyr_sum/bias_prior accumulators above only start filling once the
       variance window confirms it. Anchoring dwell to the earlier timestamp
       let the first trigger fire the moment var_static flipped true, with
       whatever the accumulator happened to hold at that instant - as little
       as one sample, which a tight zero_rot_stddev_rps then fuses as if it
       were the true bias, vibration and all, collapsing the covariance
       around it before the (correctly zero-mean) samples after it can pull
       it back out. Anchoring here instead guarantees at least dwell_sec of
       real accumulation before the first trigger can ever fire. */
    const ins_time_us_t dwell_us = (ins_time_us_t)(dwell_sec * (float)INS_US_PER_SEC);
    if (m->timestamp - f->auto_zupt_var_since < dwell_us)
    {
        return false; /* not still long enough yet */
    }

    const ins_time_us_t min_interval_us = (ins_time_us_t)(min_interval_sec * (float)INS_US_PER_SEC);
    if (f->t_last_auto_zupt != 0 && m->timestamp - f->t_last_auto_zupt < min_interval_us)
    {
        return false; /* rate-limited */
    }

    f->t_last_auto_zupt = m->timestamp;
    f->diag.n_auto_zupt++;
    return true;
}

/* ============================================================================
 * Input sanitization
 * ============================================================================
 */

/* Drop measurement blocks containing non-finite values (NaN/Inf) before they
 * reach any math. This cannot be left to the fusion gates: every comparison
 * with NaN is false, so the chi2 outlier tests and the "reject if variance <= 0"
 * checks wave NaN through, and one corrupt sample would poison the state.
 *
 * Policy per field:
 *  - sensor payload (data/position/velocity/yaw) non-finite
 *      -> whole block invalidated
 *  - acc/gyr noise PSD non-finite -> 0 (handled by ins_resolve_noise_psd)
 *  - optional extras (lever arms, cross-covariance) non-finite -> zeroed
 *  - strapdown_dt_sec non-finite -> 0 (no strapdown this epoch)
 *  - absolute headings outside one full turn -> dropped, inside ->
 *      normalized to [-pi, pi] (REQ-NAV-060) */
/* Apply the configured per-sensor calibration (misalignment + fixed bias)
 * to the raw acc/gyr/mag in place. Shared by ins_sanitize_measurements
 * and by nav_suite (which feeds the same calibrated signal to the parallel
 * AHRS/baro filters). See ins.h for the contract. */
/* @satisfies REQ-NAV-039 */
void ins_apply_calibration(const ins_options_t* opt, ins_measurements_t* m)
{
    if (opt == NULL || m == NULL) { return; }
    if (m->acc.is_valid)
    {
        ins_imu_calibrate(opt->imu_acc_misalignment, opt->imu_acc_fixed_bias, m->acc.data,
                          m->acc.data);
    }
    if (m->gyr.is_valid)
    {
        ins_imu_calibrate(opt->imu_gyr_misalignment, opt->imu_gyr_fixed_bias, m->gyr.data,
                          m->gyr.data);
    }
    if (m->mag.is_valid)
    {
        ins_imu_calibrate(opt->mag_misalignment, opt->mag_fixed_bias, m->mag.data, m->mag.data);
    }
}

/* Public face of ins_condition_gnss_cov() for callers that have to weight a
 * GNSS fix with the same measurement noise this filter fuses it with (today:
 * nav_suite's baro/GNSS vertical offset filter). See ins.h for why this must
 * never reach a fix-quality gate. */
/* @satisfies REQ-NAV-038 REQ-NAV-041 */
void ins_gnss_condition_pos_cov(const ins_options_t* opt, const float Qll_in[9], float Qll_out[9])
{
    if (opt == NULL || Qll_in == NULL || Qll_out == NULL) { return; }
    if ((const float*)Qll_out != Qll_in) { memcpy(Qll_out, Qll_in, sizeof(float) * 9u); }
    ins_condition_gnss_cov(Qll_out, opt->gnss_pos_cov_scale, opt->gnss_pos_cov_scale_height,
                           opt->gnss_pos_stddev_floor_hor_m, opt->gnss_pos_stddev_floor_ver_m,
                           opt->gnss_pos_stddev_cap_hor_m, opt->gnss_pos_stddev_cap_ver_m);
}

/* @satisfies REQ-NAV-038 */
void ins_gnss_condition_vel_cov(const ins_options_t* opt, const float Qll_in[9], float Qll_out[9])
{
    if (opt == NULL || Qll_in == NULL || Qll_out == NULL) { return; }
    if ((const float*)Qll_out != Qll_in) { memcpy(Qll_out, Qll_in, sizeof(float) * 9u); }
    /* No height scale on the velocity block (position-only knob). */
    ins_condition_gnss_cov(Qll_out, opt->gnss_vel_cov_scale, 0.0f,
                           opt->gnss_vel_stddev_floor_hor_mps, opt->gnss_vel_stddev_floor_ver_mps,
                           opt->gnss_vel_stddev_cap_hor_mps, opt->gnss_vel_stddev_cap_ver_mps);
}

/* Left/right scaling of the NON-symmetric pos/vel cross-covariance block:
 * Q' = diag(row_hor, row_hor, row_ver) Q diag(col_hor, col_hor, col_ver).
 * The row index of a column-major 3x3 is k % 3, the column index k / 3. */
static inline void ins_cov_cross_scale(float Q[9], float row_hor, float row_ver, float col_hor,
                                       float col_ver)
{
    int k;
    for (k = 0; k < 9; ++k)
    {
        const float r = ((k % 3) == 2) ? row_ver : row_hor;
        const float c = ((k / 3) == 2) ? col_ver : col_hor;
        Q[k] *= r * c;
    }
}

/* Decay factor (1 - alpha) of the reported-accuracy envelope for one block
 * (REQ-NAV-072), from the time since that block was last seen. 0 means the
 * envelope is fully forgotten this epoch, 1 means pure peak-hold.
 *
 * A non-advancing or backwards dt yields 1: an out-of-order or duplicated epoch
 * must not age the envelope. An un-primed envelope (t_prev == 0) and a
 * disabled/absent time constant both yield 0. */
static float ins_gnss_env_decay(const ins_t* f, ins_time_us_t t_prev, ins_time_us_t t_now)
{
    const float tau = f->opt.gnss_acc_envelope_tau_sec;
    if (tau <= 0.0f || t_prev == 0) { return 0.0f; }
    {
        const int   dt_ms  = time_diff_ms(t_now, t_prev);
        const float dt_sec = (float)dt_ms * 1e-3f;
        if (dt_sec <= 0.0f) { return 1.0f; }
        if (dt_sec >= tau) { return 0.0f; }
        return 1.0f - (dt_sec / tau);
    }
}

/* Advance one axis group of the reported-accuracy envelope (REQ-NAV-072) and
 * return the congruence factor r = envelope / reported >= 1 that carries the
 * fusion covariance from the reported 1-sigma to the tracked one.
 * The peak-hold is what makes it asymmetric: the decay is applied first, so
 * a report at or above the decayed envelope resets it and yields r = 1. */
static float ins_gnss_acc_envelope(float* env, float reported, float decay)
{
    float e = *env * decay;
    if (!(reported > 0.0f)) { return 1.0f; } /* broken covariance: leave the envelope alone */
    if (e < reported) { e = reported; }
    *env = e;
    return e / reported;
}

/* Acceleration of the GNSS ANTENNA in the n-frame (REQ-NAV-076), the quantity
 * the manoeuvre term below is a function of: the body's windowed mean
 * acceleration plus the centripetal acceleration the lever arm adds under
 * rotation,
 *
 *     a_ant = a_body + R * (omega x (omega x l))
 *           = a_body + R * (omega*omega' - |omega|^2 I) l
 *
 * The operator in brackets is what the strapdown carries windowed
 * (f->omega_outer_avg, packed xx, yy, zz, xy, xz, yz), the lever arm arriving
 * only here, at the fix. The tangential part alpha x l is left out: it would
 * need the gyro differentiated and vanishes in a steady turn. With no lever
 * arm the whole addition is exactly zero. */
static void ins_gnss_antenna_acc_n(const ins_t* f, const float lever_b[3], float a_ant_n[3])
{
    const float* P           = f->omega_outer_avg;
    const float  tr          = P[0] + P[1] + P[2]; /* |omega|^2, windowed */
    const float  a_cent_b[3] = {(P[0] - tr) * lever_b[0] + P[3] * lever_b[1] + P[4] * lever_b[2],
                                P[3] * lever_b[0] + (P[1] - tr) * lever_b[1] + P[5] * lever_b[2],
                                P[4] * lever_b[0] + P[5] * lever_b[1] + (P[2] - tr) * lever_b[2]};
    float        a_cent_n[3];
    mat3_mul_vec3(f->R_b_to_n, a_cent_b, a_cent_n);
    a_ant_n[0] = f->acc_n_avg[0] + a_cent_n[0];
    a_ant_n[1] = f->acc_n_avg[1] + a_cent_n[1];
    a_ant_n[2] = f->acc_n_avg[2] + a_cent_n[2];
}

/* Manoeuvre-dependent GNSS velocity noise (REQ-NAV-073): extra independent
 * variance on the diagonal of an already conditioned velocity covariance, from
 * the current n-frame acceleration of the antenna with gravity removed.
 *
 * A Doppler velocity is an average over the receiver's measurement interval,
 * the state it is fused against is instantaneous. Under acceleration the two
 * differ systematically, by an amount the reported accuracy cannot describe
 * because it belongs to the platform rather than to the signal. Left unpriced,
 * that difference enters as an innovation and is corrected for in the velocity
 * and, over a sustained manoeuvre, in the accelerometer bias.
 *
 * The acceleration that sets it is the ANTENNA's (REQ-NAV-076), not the IMU's:
 * what the receiver averaged is the motion of the phase centre.
 *
 * Applied AFTER the cap (REQ-NAV-071): the cap bounds what the receiver claims
 * about itself, this term describes what the platform is doing. Off-diagonal
 * terms are untouched, the added noise being independent per axis. Reads the
 * windowed means of REQ-NAV-075, not the instantaneous samples. */
/* @satisfies REQ-NAV-073 REQ-NAV-076 */
static void ins_gnss_add_manoeuvre_vel_noise(const ins_t* f, const ins_measurements_t* m,
                                             float Q[9])
{
    const float s_hor = f->opt.gnss_vel_noise_acc_scale_hor;
    const float s_ver = f->opt.gnss_vel_noise_acc_scale_ver;
    float       a_ant_n[3];
    if (!f->last_acc_valid || !f->acc_n_avg_valid) { return; }
    ins_gnss_antenna_acc_n(f, m->gnss_leverarm_b, a_ant_n);
    if (s_hor > 0.0f)
    {
        const float aN = a_ant_n[0];
        const float aE = a_ant_n[1];
        const float sd = s_hor * SQRTF(aN * aN + aE * aE);
        Q[0] += sd * sd; /* NN (col-major 3x3) */
        Q[4] += sd * sd; /* EE */
    }
    if (s_ver > 0.0f)
    {
        const float sd = s_ver * fabsf(a_ant_n[2]);
        Q[8] += sd * sd; /* DD */
    }
}

/* Build this epoch's GNSS fusion covariances next to the sanitized measurement.
 * Deliberately NOT written back into m->gnss_*.Qll_ned: the fusion weights
 * these, while the fix-quality gates keep grading what the receiver reported.
 * Conditioning only ever inflates a covariance, so sharing one number would
 * turn a downweight into a rejection. A block this epoch does not offer is
 * zeroed rather than left stale. */
/* @satisfies REQ-NAV-038 REQ-NAV-041 REQ-NAV-072 REQ-NAV-073 */
static void ins_build_gnss_fuse_cov(ins_t* f, const ins_measurements_t* m)
{
    /* Envelope congruence factors, 1.0 for a block this epoch does not
       offer so the cross-covariance below can use them unconditionally. */
    float rp_hor = 1.0f, rp_ver = 1.0f, rv_hor = 1.0f, rv_ver = 1.0f;

    if (m->gnss_pos.is_valid)
    {
        if (ins_cov_is_valid(m->gnss_pos.Qll_ned))
        {
            const float decay = ins_gnss_env_decay(f, f->t_gnss_env_pos, m->timestamp);
            const float sN    = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 0, 0, 3, 3));
            const float sE    = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 1, 1, 3, 3));
            const float sD    = SQRTF(MAT_ELEM(m->gnss_pos.Qll_ned, 2, 2, 3, 3));
            /* One envelope per axis group, fed by the worse of the two
               horizontal axes: N and E come out of one solution and one
               satellite geometry. */
            rp_hor = ins_gnss_acc_envelope(&f->gnss_env_pos_hor_m, (sN > sE) ? sN : sE, decay);
            rp_ver = ins_gnss_acc_envelope(&f->gnss_env_pos_ver_m, sD, decay);
            f->t_gnss_env_pos = m->timestamp;
        }
        memcpy(f->step_ctx.gnss_pos_Qll_fuse, m->gnss_pos.Qll_ned,
               sizeof(f->step_ctx.gnss_pos_Qll_fuse));
        ins_cov_scale_congruence(f->step_ctx.gnss_pos_Qll_fuse, rp_hor, rp_ver);
        ins_gnss_condition_pos_cov(&f->opt, f->step_ctx.gnss_pos_Qll_fuse,
                                   f->step_ctx.gnss_pos_Qll_fuse);
    }
    else { memset(f->step_ctx.gnss_pos_Qll_fuse, 0, sizeof(f->step_ctx.gnss_pos_Qll_fuse)); }

    if (m->gnss_vel.is_valid)
    {
        if (ins_cov_is_valid(m->gnss_vel.Qll_ned))
        {
            const float decay = ins_gnss_env_decay(f, f->t_gnss_env_vel, m->timestamp);
            const float sN    = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 0, 0, 3, 3));
            const float sE    = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 1, 1, 3, 3));
            const float sD    = SQRTF(MAT_ELEM(m->gnss_vel.Qll_ned, 2, 2, 3, 3));
            rv_hor = ins_gnss_acc_envelope(&f->gnss_env_vel_hor_mps, (sN > sE) ? sN : sE, decay);
            rv_ver = ins_gnss_acc_envelope(&f->gnss_env_vel_ver_mps, sD, decay);
            f->t_gnss_env_vel = m->timestamp;
        }
        memcpy(f->step_ctx.gnss_vel_Qll_fuse, m->gnss_vel.Qll_ned,
               sizeof(f->step_ctx.gnss_vel_Qll_fuse));
        ins_cov_scale_congruence(f->step_ctx.gnss_vel_Qll_fuse, rv_hor, rv_ver);
        ins_gnss_condition_vel_cov(&f->opt, f->step_ctx.gnss_vel_Qll_fuse,
                                   f->step_ctx.gnss_vel_Qll_fuse);
        ins_gnss_add_manoeuvre_vel_noise(f, m, f->step_ctx.gnss_vel_Qll_fuse);
    }
    else { memset(f->step_ctx.gnss_vel_Qll_fuse, 0, sizeof(f->step_ctx.gnss_vel_Qll_fuse)); }

    /* The pos/vel cross-covariance carries one scale factor from each block to
       stay consistent with them, plus the height scale on its position-Down
       row (REQ-NAV-041) and each block's envelope factors (REQ-NAV-072) on its
       own side. Floors, caps and the manoeuvre term do not apply: this block
       has no diagonal of its own. */
    if (m->gnss_pos.is_valid && m->gnss_vel.is_valid)
    {
        const float sp = (f->opt.gnss_pos_cov_scale > 0.0f) ? f->opt.gnss_pos_cov_scale : 1.0f;
        const float sv = (f->opt.gnss_vel_cov_scale > 0.0f) ? f->opt.gnss_vel_cov_scale : 1.0f;
        const float sh =
            (f->opt.gnss_pos_cov_scale_height > 0.0f) ? f->opt.gnss_pos_cov_scale_height : 1.0f;
        memcpy(f->step_ctx.gnss_pos_vel_Qll_fuse, m->gnss_Qll_pos_vel_ned,
               sizeof(f->step_ctx.gnss_pos_vel_Qll_fuse));
        /* Position on the rows (height scale included), velocity on the
           columns. Equivalent to the plain sp*sv product plus sh on row 2. */
        ins_cov_cross_scale(f->step_ctx.gnss_pos_vel_Qll_fuse, sp * rp_hor, sp * sh * rp_ver,
                            sv * rv_hor, sv * rv_ver);
    }
    else
    {
        memset(f->step_ctx.gnss_pos_vel_Qll_fuse, 0, sizeof(f->step_ctx.gnss_pos_vel_Qll_fuse));
    }
}

/* @satisfies REQ-NAV-018 REQ-NAV-060 */
static void ins_sanitize_measurements(ins_t* f, const ins_measurements_t* in,
                                      ins_measurements_t* out)
{
    *out = *in;
    int            i;
    const uint32_t n_invalid_before = f->diag.n_invalid_input;

    if (!isfinite(out->strapdown_dt_sec))
    {
        out->strapdown_dt_sec = 0.0f;
        f->diag.n_invalid_input++;
    }
    if (out->acc.is_valid && !ins_vec3_finite(out->acc.data))
    {
        out->acc.is_valid = false;
        f->diag.n_invalid_input++;
    }
    if (out->gyr.is_valid && !ins_vec3_finite(out->gyr.data))
    {
        out->gyr.is_valid = false;
        f->diag.n_invalid_input++;
    }
    for (i = 0; i < 3; ++i)
    {
        /* 0 -> default, non-finite must not survive into the process noise. */
        if (!isfinite(out->acc.Qll_diag[i])) out->acc.Qll_diag[i] = 0.0f;
        if (!isfinite(out->gyr.Qll_diag[i])) out->gyr.Qll_diag[i] = 0.0f;
    }
    if (out->mag.is_valid &&
        (!ins_vec3_finite(out->mag.data) || !ins_vec3_finite(out->mag.Qll_diag)))
    {
        out->mag.is_valid = false; /* mag variance has no default */
        f->diag.n_invalid_input++;
    }
    /* Sensor calibration (REQ-NAV-037 / REQ-NAV-039): correct the raw
       acc/gyr/mag in place before any downstream use, once all three have
       passed the finiteness gates above. */
    ins_apply_calibration(&f->opt, out);
    if (out->gnss_pos.is_valid &&
        (!vec3d_finite(out->gnss_pos.llh) || !mat33_finite(out->gnss_pos.Qll_ned)))
    {
        out->gnss_pos.is_valid = false;
        f->diag.n_invalid_input++;
    }
    if (out->gnss_vel.is_valid &&
        (!ins_vec3_finite(out->gnss_vel.vel_ned) || !mat33_finite(out->gnss_vel.Qll_ned)))
    {
        out->gnss_vel.is_valid = false;
        f->diag.n_invalid_input++;
    }
    if (!mat33_finite(out->gnss_Qll_pos_vel_ned))
    {
        memset(out->gnss_Qll_pos_vel_ned, 0, sizeof(out->gnss_Qll_pos_vel_ned));
        f->diag.n_invalid_input++;
    }
    /* The reported GNSS covariances stay exactly as the receiver sent them: the
       fix-quality gates downstream have to grade those (REQ-NAV-038). The
       conditioned copies the fusion weights with are built separately
       (ins_build_gnss_fuse_cov). */
    if (!ins_vec3_finite(out->gnss_leverarm_b))
    {
        vec3_zero(out->gnss_leverarm_b);
        f->diag.n_invalid_input++;
    }
    if (out->local_pos.is_valid &&
        (!ins_vec3_finite(out->local_pos.pos_ned) || !mat33_finite(out->local_pos.Qll_ned)))
    {
        out->local_pos.is_valid = false;
        f->diag.n_invalid_input++;
    }
    if (!ins_vec3_finite(out->local_pos_leverarm_b))
    {
        vec3_zero(out->local_pos_leverarm_b);
        f->diag.n_invalid_input++;
    }
    if (out->yaw.is_valid)
    {
        if (!isfinite(out->yaw.yaw_rad) || !isfinite(out->yaw.stddev_rad) ||
            fabsf(out->yaw.yaw_rad) > INS_YAW_INPUT_MAX_RAD)
        {
            out->yaw.is_valid = false;
            f->diag.n_invalid_input++;
        }
        else { out->yaw.yaw_rad = ins_angle_diff(out->yaw.yaw_rad, 0.0f); /* REQ-NAV-060 */ }
    }
    /* The attitude hint feeds the nominal quaternion and the bias states
       directly (no residual, no chi2 gate), so a non-finite field would poison
       the state instead of costing one fusion. Roll/pitch/bias and the heading
       fail independently: a hint whose only defect is an out-of-band yaw still
       carries usable leveling, and stddev 0 means "no yaw hint". */
    if (out->att_hint.is_valid)
    {
        if (!isfinite(out->att_hint.roll_rad) || !isfinite(out->att_hint.pitch_rad) ||
            !ins_vec3_finite(out->att_hint.gyr_bias_rps))
        {
            out->att_hint.is_valid = false;
            f->diag.n_invalid_input++;
        }
        else if (!isfinite(out->att_hint.yaw_rad) ||
                 fabsf(out->att_hint.yaw_rad) > INS_YAW_INPUT_MAX_RAD)
        {
            out->att_hint.yaw_rad        = 0.0f;
            out->att_hint.stddev_yaw_rad = 0.0f;
            f->diag.n_invalid_input++;
        }
        else
        {
            out->att_hint.yaw_rad = ins_angle_diff(out->att_hint.yaw_rad, 0.0f); /* REQ-NAV-060 */
        }
    }

    /* Throttled summary instead of one LOG_WARN per rejected field: a
       flaky sensor bus can otherwise flood the sink at the update rate. */
    if (f->diag.n_invalid_input != n_invalid_before)
    {
        const bool  first_warn = (f->log_state.t_last_invalid_warn == 0);
        const float since_warn_sec =
            first_warn ? 0.0f : time_diff_sec(in->timestamp, f->log_state.t_last_invalid_warn);
        if (first_warn || since_warn_sec >= INS_LOG_INVALID_REPEAT_SEC)
        {
            LOG_WARN("ins: non-finite/invalid input dropped at the update boundary "
                     "(%u field(s) this epoch, %u total since init)",
                     (unsigned int)(f->diag.n_invalid_input - n_invalid_before),
                     (unsigned int)f->diag.n_invalid_input);
            f->log_state.t_last_invalid_warn = in->timestamp;
        }
    }
}

/* ============================================================================
 * Public API: update (Strapdown + Predict + Fusion)
 * ============================================================================
 */

/* @satisfies REQ-NAV-064 REQ-NAV-069 */
int ins_predict_step(ins_t* f, const ins_measurements_t* m_in, float* phi_out)
{
    if (f == NULL) return INS_EPOCH_DROPPED;
    if (m_in == NULL)
    {
        f->step_ctx.active = false;
        return INS_EPOCH_DROPPED;
    }

    ins_measurements_t m_sane;
    ins_sanitize_measurements(f, m_in, &m_sane);
    const ins_measurements_t* m = &m_sane;
    if (!f->is_initialized)
    {
        /* Not started yet: consume until the streams are coherent
           (REQ-NAV-033). is_collecting distinguishes "pending start" from
           a shut-down / reset filter (which stays dead). */
        if (f->is_collecting)
        {
            f->diag.n_updates++;

            /* Track the streams for the start checks (works across separate
               IMU-only / GNSS-only update calls). */
            if (m->acc.is_valid && m->gyr.is_valid)
            {
                f->t_pending_imu    = m->timestamp;
                f->have_pending_imu = true;
            }
            if (ins_gnss_pos_usable(f, &m->gnss_pos) || m->local_pos.is_valid)
            {
                f->t_pending_fix    = m->timestamp;
                f->have_pending_fix = true;
            }

            /* Cache the latest barometer sample seen while still collecting:
               decides the height source at bootstrap (REQ-NAV-053,
               ins_autoinit_try). The plausibility check (same bounds as
               baro_alt.c's) keeps garbage from being latched as the anchor. */
            if (m->baro.is_valid && ins_isa_pressure_plausible(m->baro.pressure_pa))
            {
                f->autoinit_baro.t           = m->timestamp;
                f->autoinit_baro.pressure_pa = m->baro.pressure_pa;
                f->autoinit_baro.valid       = true;
            }

            if (f->opt.auto_init)
            {
                /* Buffer IMU for leveling, bootstrap on the first fix that
                   is temporally coherent with the IMU window. */
                ins_autoinit_push(f, m);
                (void)ins_autoinit_try(f, m);
            }
            else if (ins_start_gate_ready(f))
            {
                /* Manual init: apply the prescribed values at this (first
                   coherent) epoch. */
                ins_finalize_manual(f, m->timestamp);
            }
        }
        f->step_ctx.active = false;
        return INS_EPOCH_DROPPED;
    }

    f->diag.n_updates++;

    /* Time-jump detection. @satisfies REQ-NAV-016 */
    const int dt_ms = time_diff_ms(m->timestamp, f->t_last_kalman_predict);
    if (dt_ms < f->diag.dt_ms_min) f->diag.dt_ms_min = dt_ms;
    if (dt_ms > f->diag.dt_ms_max) f->diag.dt_ms_max = dt_ms;
    if (dt_ms < 0) f->diag.n_time_backward++;

    const bool time_jump = (dt_ms < 0) || (dt_ms > (int)(f->opt.max_prediction_time_sec * 1000.0f));

    if (time_jump)
    {
        if (dt_ms < -INS_MAX_DELAY_MS)
        {
            /* Too far in the past: drop. One such epoch is reordering, an
               unbroken run of them is a restarted time source (REQ-NAV-070).
               Dropping alone would be a trap there: t_last_kalman_predict
               stays in the abandoned timebase and the filter stays inert
               forever while still being fed valid data.
               allow_unlimited_deadreckoning does not suppress this the way it
               suppresses the forward-jump reset.
               @satisfies REQ-NAV-070 */
            f->diag.n_time_dropped++;
            f->time_dropped_run++;
            if (f->time_dropped_run >= INS_TIME_RESTART_EPOCHS)
            {
                f->diag.n_time_restart_reset++;
                LOG_WARN("ins: timestamp source restarted (%d ms in the past for %u "
                         "consecutive epochs), filter reset",
                         dt_ms, (unsigned)f->time_dropped_run);
                f->is_initialized   = false;
                f->time_dropped_run = 0;
                /* Same re-arm as the forward-jump reset above: without it
                   is_collecting stays false and auto-init never re-runs. */
                if (!f->opt.auto_reacquire_disable && f->opt.auto_init) { ins_rearm_collecting(f); }
            }
            f->step_ctx.active = false;
            return INS_EPOCH_DROPPED;
        }
        /* Within the tolerated depth: whatever run was building, it ends
           here, so a reordering burst can never accumulate into a restart
           reset across the quiet stretches between bursts (REQ-NAV-070). */
        f->time_dropped_run = 0;
        if (dt_ms > 0 && !f->opt.allow_unlimited_deadreckoning)
        {
            f->diag.n_time_jump_reset++;
            LOG_WARN("ins: forward time jump of %d ms exceeds max_prediction_time_sec, "
                     "filter reset",
                     dt_ms);
            f->is_initialized = false;
            /* Without this, is_collecting also stays false (it was cleared at
               start-up and this path never sets it), so the filter would be
               permanently dead until an external ins_init(), unlike every
               other shutdown path (REQ-NAV-042). */
            if (!f->opt.auto_reacquire_disable && f->opt.auto_init) { ins_rearm_collecting(f); }
            f->step_ctx.active = false;
            return INS_EPOCH_DROPPED;
        }
        if (m->acc.is_valid || m->gyr.is_valid)
        {
            if (dt_ms > 0)
            {
                /* Unlimited dead reckoning coasting through a forward gap
                   (e.g. a stalled IMU link): re-baseline instead of just
                   skipping this epoch. t_last_kalman_predict is otherwise only
                   advanced by a successful predict step, which this branch
                   bypasses, so time_jump would latch true forever. */
                f->t_last_kalman_predict = m->timestamp;
            }
            /* Skip IMU on backwards-time jumps. */
            f->step_ctx.active = false;
            return INS_EPOCH_DROPPED;
        }
    }
    else { f->time_dropped_run = 0; /* normal epoch: same run break (REQ-NAV-070) */ }

    /* Coasting window expired (REQ-NAV-022): the filter goes inert rather than
       integrating through an outage of unknown length. No strapdown, no time
       update, no fusion (REQ-NAV-064) - state and covariance hold their last
       value until ins_reacquire() re-anchors on the next usable fix
       (REQ-NAV-023) and prices the interval in (REQ-NAV-065). */
    const bool dr_frozen = ins_dr_expired(f, m->timestamp);

    /* Cache the barometer for the vertical re-anchor (REQ-NAV-066). Runs while
       inert as well: the fix that ends the outage rarely carries a barometer
       sample on its own clock. Caching is not processing - nothing here
       reaches the state. */
    if (m->baro.is_valid && ins_isa_pressure_plausible(m->baro.pressure_pa))
    {
        f->last_baro.t           = m->timestamp;
        f->last_baro.pressure_pa = m->baro.pressure_pa;
        f->last_baro.stddev_m    = m->baro.stddev_m;
        f->last_baro.valid       = true;
    }

    /* -------- Strapdown -------- */
    if (!time_jump && !dr_frozen && m->strapdown_dt_sec > 0.0f && m->gyr.is_valid &&
        m->acc.is_valid)
    {
        float dpos_n[3];
        ins_strapdown(f, m, m->strapdown_dt_sec, dpos_n);
        /* Refresh metadata: orientation + position have changed
           (incremental latlonh update from the n-frame delta). */
        ins_update_meta(f, true, dpos_n);
    }

    /* Stash the latest bias-corrected acc measurement so the predict step has
       an up-to-date specific-force sample (the previous one is kept if this
       epoch has none). Kept fresh while inert too, so the first prediction
       after a re-acquisition does not build its velocity-to-attitude coupling
       from a specific force measured before the outage. */
    if (m->acc.is_valid)
    {
        f->last_acc_meas[0] = m->acc.data[0] - f->state.acc_bias[0];
        f->last_acc_meas[1] = m->acc.data[1] - f->state.acc_bias[1];
        f->last_acc_meas[2] = m->acc.data[2] - f->state.acc_bias[2];
        f->last_acc_valid   = true;
    }

    /* -------- Kalman prediction -------- */
    /* ins_predict propagates the covariance ONLY.
       The nominal state is the strapdown's business
       (x = NULL, error state implicitly zero) */
    int status = 0;
    {
        const float dt_pred = time_diff_sec(m->timestamp, f->t_last_kalman_predict);
        /* Tolerance on the due test, not a bare ">=": see
           INS_CADENCE_TOLERANCE for why an epoch stream a hair below the
           configured period would otherwise halve the prediction rate.
           dt_pred (measured) is what ins_predict integrates. */
        if (!time_jump && !dr_frozen && INS_CADENCE_DUE(dt_pred, ins_kalman_dt(f)) &&
            dt_pred <= f->opt.max_prediction_time_sec)
        {
            /* Use the measurement covariances if present (falling back to a
               conservative consumer-MEMS default per axis if unset), else
               assume no IMU sample at all this epoch (zero noise contribution). */
            float Qa[3] = {0.0f, 0.0f, 0.0f};
            float Qg[3] = {0.0f, 0.0f, 0.0f};
            if (m->acc.is_valid)
            {
                ins_resolve_noise_psd(m->acc.Qll_diag, INS_DEFAULT_ACC_VRW_PSD, Qa);
            }
            if (m->gyr.is_valid)
            {
                ins_resolve_noise_psd(m->gyr.Qll_diag, INS_DEFAULT_GYR_ARW_PSD, Qg);
            }
            ins_predict(f, dt_pred, Qa, Qg, phi_out);
            f->t_last_kalman_predict = m->timestamp;
            f->kalman_epochs++;
            f->diag.n_predict++;
            status |= INS_EPOCH_COV_PROPAGATED;
        }
        /* Inert: re-baseline unconditionally, so the outage does not surface
           as one giant prediction gap once aiding returns (mirrors the
           forward-time-jump handling above). Without it dt_pred would latch
           permanently above max_prediction_time_sec. */
        if (dr_frozen) { f->t_last_kalman_predict = m->timestamp; }
    }

    /* Hand off to ins_correct_step(): the sanitized measurement plus the
       fusion gating this same epoch already decided (time_jump is always
       false past this point except for the fall-through case below, where
       neither acc nor gyr was valid so none of the early returns above
       applied - fusion still needs to be skipped for it). */
    f->step_ctx.m = m_sane;
    ins_build_gnss_fuse_cov(f, &f->step_ctx.m);
    f->step_ctx.active     = true;
    f->step_ctx.run_fusion = !time_jump;
    f->step_ctx.dr_frozen  = dr_frozen;
    if (dr_frozen) status |= INS_EPOCH_DR_FROZEN;
    return status;
}

/* @satisfies REQ-NAV-064 REQ-NAV-069 */
void ins_correct_step(ins_t* f)
{
    if (f == NULL || !f->step_ctx.active) return;
    f->step_ctx.active          = false;
    const ins_measurements_t* m = &f->step_ctx.m;

    /* -------- Measurement fusion -------- */
    if (f->step_ctx.run_fusion)
    {
        /* The stillness detector keeps running while inert: it carries its own
           windowed state, and letting it go stale would have the first epochs
           after a re-acquisition judged on pre-outage motion. */
        const bool auto_zupt = ins_auto_zupt_detect(f, m);

        if (!f->step_ctx.dr_frozen)
        {
            ins_fuse_mag(f, m);
            ins_fuse_yaw(f, m);
            ins_fuse_zero_velocity(f, m, m->zero_velocity_update || auto_zupt);
            ins_fuse_zero_rotation(f, m, m->zero_rotation_update || auto_zupt);
            /* Before the position channels below: the constraint is about the
               state the coasting produced, and it decides whether to act on
               how long that coasting has lasted (REQ-NAV-077). Running it
               after ins_fuse_gnss would let the fix that ends an outage reset
               that clock in the same epoch. */
            ins_fuse_lateral_constraint(f, m);
        }

        /* The two position channels run either way: while inert (REQ-NAV-064)
           their only job is to spot the fix that ends the outage and re-anchor
           on it (REQ-NAV-023), the one thing allowed to write state. */
        ins_fuse_local_pos(f, m);
        ins_fuse_gnss(f, m);

        /* dr_frozen, not a fresh expiry test: on the epoch that re-anchors the
           height has just been set from this same barometer sample
           (REQ-NAV-066), and fusing it again would count one sample twice. */
        if (!f->step_ctx.dr_frozen)
        {
            ins_fuse_baro_height(f, m); /* REQ-NAV-054, no-op unless height_from_baro */
            ins_fuse_speed(f, m);       /* REQ-NAV-068 */
            ins_fuse_gnss_course_yaw(f, m);
        }

        /* Mode arbitration on the aiding quality (REQ-NAV-051/052): after the
           fusion, so an epoch is judged on the same measurement the filter has
           just seen, and only on live epochs. */
        ins_track_gnss_mode_gates(f, m);
    }

    /* -------- History save + health check -------- */
    ins_save_state(f, m->timestamp, false);
    ins_check_health(f);
    if (f->is_initialized) { ins_check_overconfidence(f); }

    if (f->is_initialized)
    {
        const float yaw_aid_gap_sec = time_diff_sec(m->timestamp, f->log_state.t_last_yaw_aid);
        /* Only the yaw element of the diagonal is used below (both here and
         * by the runaway tracker further down, so this can't be skipped when
         * yaw_aid_gap_sec is small) -- extract that one element instead of
         * the full O(n^2) diagonal. */
        const float yaw_var_rad2 = udu_get_diag_one(f->U, f->d, f->n, INS_IDX_RPY + 2);
        /* Split off the variance above instead of nesting the call inside RAD2DEG:
         * cppcheck 2.7, the version the CI image pins, cannot build an AST for a
         * _Generic whose controlling expression is a call taking struct members. */
        const float yaw_stddev_deg = RAD2DEG(SQRTF(yaw_var_rad2));
        if (yaw_aid_gap_sec >= INS_LOG_YAW_AID_GAP_WARN_SEC &&
            yaw_stddev_deg >= INS_LOG_YAW_STDDEV_WARN_DEG)
        {
            const bool  first_warn = (f->log_state.t_last_yaw_stddev_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f
                           : time_diff_sec(m->timestamp, f->log_state.t_last_yaw_stddev_warn);
            if (first_warn || since_warn_sec >= INS_LOG_YAW_AID_GAP_REPEAT_SEC)
            {
                LOG_WARN("ins: yaw not aided (no mag/absolute-yaw/automotive-course fusion) "
                         "for %.1f s, yaw stddev grown to %.1f deg",
                         (double)yaw_aid_gap_sec, (double)yaw_stddev_deg);
                f->log_state.t_last_yaw_stddev_warn = m->timestamp;
            }
        }
        else { f->log_state.t_last_yaw_stddev_warn = 0; }

        /* Yaw-stddev runaway: fast growth within one window, catches a filter
           diverging even while still nominally aided. */
        if (f->log_state.t_yaw_stddev_window == 0)
        {
            f->log_state.t_yaw_stddev_window   = m->timestamp;
            f->log_state.yaw_stddev_window_deg = yaw_stddev_deg;
        }
        else
        {
            const float window_sec = time_diff_sec(m->timestamp, f->log_state.t_yaw_stddev_window);
            if (window_sec >= INS_LOG_RUNAWAY_WINDOW_SEC)
            {
                const float growth_deg = yaw_stddev_deg - f->log_state.yaw_stddev_window_deg;
                if (growth_deg >= INS_LOG_YAW_RUNAWAY_DEG)
                {
                    LOG_WARN("ins: yaw stddev runaway: grew %.1f deg in %.1f s (%.2f deg/s) "
                             "- check aiding availability and Q/R tuning",
                             (double)growth_deg, (double)window_sec,
                             (double)(growth_deg / window_sec));
                }
                f->log_state.t_yaw_stddev_window   = m->timestamp;
                f->log_state.yaw_stddev_window_deg = yaw_stddev_deg;
            }
        }

        /* Bias runaway + sanity bounds. Gyro and accel bias share the same
           tumbling-window rate check as yaw stddev above, plus an absolute
           physical sanity bound (throttled, not windowed). Gated on
           INS_LOG_BIAS_RUNAWAY_WARMUP_SEC since f->t_init. */
        if (time_diff_sec(m->timestamp, f->t_init) >= INS_LOG_BIAS_RUNAWAY_WARMUP_SEC)
        {
            /* Norm into a local first, conversion second. RAD2DEG is a
               _Generic and repeats its argument textually, which cppcheck
               fails to parse when that argument dereferences a pointer. */
            const float gyr_bias_norm_rps = vec3_norm(f->state.gyr_bias);
            const float gyr_bias_dps      = RAD2DEG(gyr_bias_norm_rps);
            if (f->log_state.t_gyr_bias_window == 0)
            {
                f->log_state.t_gyr_bias_window   = m->timestamp;
                f->log_state.gyr_bias_window_dps = gyr_bias_dps;
            }
            else
            {
                const float window_sec =
                    time_diff_sec(m->timestamp, f->log_state.t_gyr_bias_window);
                if (window_sec >= INS_LOG_RUNAWAY_WINDOW_SEC)
                {
                    const float growth = fabsf(gyr_bias_dps - f->log_state.gyr_bias_window_dps);
                    if (growth >= INS_LOG_GYR_BIAS_RUNAWAY_DPS)
                    {
                        LOG_WARN("ins: gyro bias runaway: |bias| changed %.2f deg/s in %.1f s "
                                 "(now %.2f deg/s) - possible filter divergence",
                                 (double)growth, (double)window_sec, (double)gyr_bias_dps);
                    }
                    f->log_state.t_gyr_bias_window   = m->timestamp;
                    f->log_state.gyr_bias_window_dps = gyr_bias_dps;
                }
            }
            if (gyr_bias_dps >= INS_LOG_GYR_BIAS_SANITY_DPS)
            {
                const bool  first_warn = (f->log_state.t_last_gyr_bias_sanity_warn == 0);
                const float since_warn_sec =
                    first_warn
                        ? 0.0f
                        : time_diff_sec(m->timestamp, f->log_state.t_last_gyr_bias_sanity_warn);
                if (first_warn || since_warn_sec >= INS_LOG_BIAS_SANITY_REPEAT_SEC)
                {
                    LOG_WARN("ins: gyro bias %.2f deg/s exceeds the sanity bound (%.1f deg/s) - "
                             "implausible for a MEMS IMU, filter likely diverging",
                             (double)gyr_bias_dps, (double)INS_LOG_GYR_BIAS_SANITY_DPS);
                    f->log_state.t_last_gyr_bias_sanity_warn = m->timestamp;
                }
            }

            const float acc_bias_mps2 = vec3_norm(f->state.acc_bias);
            if (f->log_state.t_acc_bias_window == 0)
            {
                f->log_state.t_acc_bias_window    = m->timestamp;
                f->log_state.acc_bias_window_mps2 = acc_bias_mps2;
            }
            else
            {
                const float window_sec =
                    time_diff_sec(m->timestamp, f->log_state.t_acc_bias_window);
                if (window_sec >= INS_LOG_RUNAWAY_WINDOW_SEC)
                {
                    const float growth = fabsf(acc_bias_mps2 - f->log_state.acc_bias_window_mps2);
                    if (growth >= INS_LOG_ACC_BIAS_RUNAWAY_MPS2)
                    {
                        LOG_WARN("ins: accel bias runaway: |bias| changed %.2f m/s^2 in %.1f s "
                                 "(now %.2f m/s^2) - possible filter divergence",
                                 (double)growth, (double)window_sec, (double)acc_bias_mps2);
                    }
                    f->log_state.t_acc_bias_window    = m->timestamp;
                    f->log_state.acc_bias_window_mps2 = acc_bias_mps2;
                }
            }
            if (acc_bias_mps2 >= INS_LOG_ACC_BIAS_SANITY_MPS2)
            {
                const bool  first_warn = (f->log_state.t_last_acc_bias_sanity_warn == 0);
                const float since_warn_sec =
                    first_warn
                        ? 0.0f
                        : time_diff_sec(m->timestamp, f->log_state.t_last_acc_bias_sanity_warn);
                if (first_warn || since_warn_sec >= INS_LOG_BIAS_SANITY_REPEAT_SEC)
                {
                    LOG_WARN("ins: accel bias %.2f m/s^2 exceeds the sanity bound (%.1f m/s^2) "
                             "- implausible for a MEMS IMU, filter likely diverging",
                             (double)acc_bias_mps2, (double)INS_LOG_ACC_BIAS_SANITY_MPS2);
                    f->log_state.t_last_acc_bias_sanity_warn = m->timestamp;
                }
            }
        }
    }

    /* Stuck-sensor diagnostics: a gyro sample bit-identical to the previous
       epoch's, several times in a row, flags a frozen bus (stale cached
       register value) that passes every finite/range check silently. */
    if (m->gyr.is_valid)
    {
        /* Exact equality is the correct test here, not an epsilon: this asks
           "did the driver hand back the exact same value again", not "are
           these two readings close". */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if (f->log_state.last_gyr_raw_valid && f->log_state.last_gyr_raw[0] == m->gyr.data[0] &&
            f->log_state.last_gyr_raw[1] == m->gyr.data[1] &&
            f->log_state.last_gyr_raw[2] == m->gyr.data[2])
        {
            f->log_state.stuck_gyr_count++;
        }
        else { f->log_state.stuck_gyr_count = 0; }
#pragma GCC diagnostic pop
        vec3_copy(m->gyr.data, f->log_state.last_gyr_raw);
        f->log_state.last_gyr_raw_valid = true;

        if (f->log_state.stuck_gyr_count >= INS_LOG_STUCK_GYR_EPOCHS)
        {
            const bool  first_warn = (f->log_state.t_last_stuck_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f : time_diff_sec(m->timestamp, f->log_state.t_last_stuck_warn);
            if (first_warn || since_warn_sec >= INS_LOG_STUCK_REPEAT_SEC)
            {
                LOG_WARN("ins: gyro sample identical for %u consecutive epochs - likely a "
                         "frozen/stuck sensor bus, not real data (or a synthetic/noise-free "
                         "test input)",
                         (unsigned int)f->log_state.stuck_gyr_count);
                f->log_state.t_last_stuck_warn = m->timestamp;
            }
        }
    }
}

/* @satisfies REQ-NAV-069 */
void ins_update(ins_t* f, const ins_measurements_t* m_in)
{
    ins_predict_step(f, m_in, NULL);
    ins_correct_step(f);
}

/* ============================================================================
 * Public API: accessors
 * ============================================================================
 */

bool ins_get_position_ecef(const ins_t* f, double p[3])
{
    if (!f || !f->is_initialized) return false;
    ins_latlonh_to_ecef(f->latlonh[0], f->latlonh[1], f->latlonh[2], p);
    return true;
}

/* @satisfies REQ-NAV-078 */
bool ins_get_latlonh(const ins_t* f, double llh[3])
{
    if (!f || !f->is_initialized) return false;
    llh[0] = f->latlonh[0];
    llh[1] = f->latlonh[1];
    llh[2] = f->latlonh[2];
    return true;
}

bool ins_get_position_local(const ins_t* f, float p[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->state.pos_local, p);
    return true;
}

bool ins_get_velocity_ned(const ins_t* f, float v[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->state.vel_ned, v);
    return true;
}

/* @satisfies REQ-NAV-080 */
bool ins_get_velocity_ecef(const ins_t* f, float v[3])
{
    if (!f || !f->is_initialized) return false;
    /* Built here rather than cached, for the same reason
       ins_get_position_ecef() converts here: nothing on the epoch path uses
       the n-frame to ECEF rotation, so refreshing it with the other Earth
       parameters would put four trigonometric calls into the worst-case
       epoch to serve a caller that may never ask (REQ-NAV-080). */
    float R_n_to_e[9];
    ins_rotmat_n_to_e(f->latlonh[0], f->latlonh[1], R_n_to_e);
    mat3_mul_vec3(R_n_to_e, f->state.vel_ned, v);
    return true;
}

bool ins_get_quaternion(const ins_t* f, float q[4])
{
    if (!f || !f->is_initialized) return false;
    q[0] = f->state.qbn[0];
    q[1] = f->state.qbn[1];
    q[2] = f->state.qbn[2];
    q[3] = f->state.qbn[3];
    return true;
}

bool ins_get_rpy(const ins_t* f, float* roll, float* pitch, float* yaw)
{
    if (!f || !f->is_initialized) return false;
    ins_rotmat_to_rpy(f->R_b_to_n, roll, pitch, yaw);
    return true;
}

bool ins_get_rpy_stddev(const ins_t* f, float* roll_stddev, float* pitch_stddev, float* yaw_stddev)
{
    if (!f || !f->is_initialized) return false;
    float diag[INS_UNKNOWNS_MAX];
    udu_get_diag(f->U, f->d, diag, f->n);
    *roll_stddev  = SQRTF(diag[INS_IDX_RPY + 0]);
    *pitch_stddev = SQRTF(diag[INS_IDX_RPY + 1]);
    *yaw_stddev   = SQRTF(diag[INS_IDX_RPY + 2]);
    return true;
}

bool ins_get_rotmat_b_to_n(const ins_t* f, float R[9])
{
    if (!f || !f->is_initialized) return false;
    memcpy(R, f->R_b_to_n, sizeof(float) * 9);
    return true;
}

bool ins_get_bias_acc(const ins_t* f, float b[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->state.acc_bias, b);
    return true;
}

bool ins_get_bias_gyr(const ins_t* f, float b[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->state.gyr_bias, b);
    return true;
}

/* @satisfies REQ-NAV-029 */
bool ins_get_bias_mag(const ins_t* f, float b[3])
{
    if (!f || !f->is_initialized || f->n <= INS_IDX_MAG) return false;
    vec3_copy(f->state.mag_bias, b);
    return true;
}

bool ins_get_omega_b_nb(const ins_t* f, float w[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->last_omega_b_nb, w);
    return true;
}

bool ins_get_acc_n(const ins_t* f, float a[3])
{
    if (!f || !f->is_initialized) return false;
    vec3_copy(f->last_acc_n, a);
    return true;
}
