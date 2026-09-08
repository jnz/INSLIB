/** @file sensor_defaults.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Shared fallback defaults for quantities that ins.c, ahrs.c and
 *        baro_alt.c genuinely share: the gyro noise model of a low-cost
 *        consumer MEMS IMU, plus a handful of cross-filter statistics
 *        (stillness gate, chi2 significance, mag field tolerance) that
 *        describe the same physical question in more than one filter.
 *
 * Used as the fallback when the caller leaves the corresponding config
 * field at 0/unset: enough to get a stable filter without any sensor
 * characterization. Replace with the sensor's actual datasheet noise
 * figures whenever available. A better (lower-noise) part left at these
 * defaults will just be fused a bit too cautiously.
 *
 * Deliberately project-specific (ins/ahrs/baro_alt share one physical IMU in
 * nav_suite): unlike geodetic_toolbox.h, this header is NOT meant to be
 * portable to other projects, so it must not be pulled into
 * geodetic_toolbox.h.
 *
 * Only truly shared quantities live here - each consuming file keeps a
 * locally-named macro that aliases the constant below, and defaults that are
 * NOT shared stay local to their own .c file. In particular there is no shared
 * accelerometer *noise* default: every filter that uses one treats it as a
 * different physical quantity. ins.c has its own VRW density ("pure" sensor
 * spec). ahrs.c fuses it as a discrete leveling correction whose noise term
 * doubles as a maneuver-rejection margin. baro_alt.c's "accelerometer" input is
 * actually a full attitude-projected vertical acceleration, so its noise
 * absorbs roll/pitch projection error on top of the raw sensor; its implied
 * density is several times ins.c's, empirically not interchangeable. ins.c's
 * own chi2 gate is likewise one shared value since REQ-NAV-046, but NOT this
 * file's INS_DEFAULT_CHI2_95_1DOF: it keeps its own, looser
 * INS_DEFAULT_CHI2_GATE, because the stricter 3.8415 was seen to destabilize
 * yaw under a sustained gross GNSS-velocity mismatch.
 *
 */

/** @addtogroup sensor_defaults
 *  @{ */

#ifndef INS_SENSOR_DEFAULTS_H
#define INS_SENSOR_DEFAULTS_H

/******************************************************************************
 * DEFINES
 ******************************************************************************/

/** Gyro angle random walk (spectral noise density) [rad/s/sqrt(Hz)],
 *  ~0.02 deg/s/sqrt(Hz). Needs <math.h> (M_PI) included before this
 *  header, same convention as geodetic_toolbox.h's DEG2RAD/RAD2DEG. */
#define INS_DEFAULT_GYR_ARW_RPS_SQRTHZ (0.02f * (float)(M_PI / 180.0))

/** Gyro bias random walk [rad/s/sqrt(s)]. */
#define INS_DEFAULT_GYR_BIAS_RW_RPS_SQRTS (2e-6f)

/** Stillness gate for ins.c's auto-init leveling window: the magnitude of the
 *  gyro must stay below this for the window to count as quasi-static. The
 *  auto-ZUPT/ZARU magnitude bound below currently aliases this value, but the
 *  two are kept as separate names on purpose: during leveling an uncorrected
 *  bias IS a leveling error and has to widen the reported attitude uncertainty,
 *  whereas for the stillness detectors it is only an obstacle. [rad/s] */
#define INS_DEFAULT_STATIC_GYR_RPS (0.04f)
/** Auto-init leveling accelerometer threshold (||f|-g|), companion to
 *  INS_DEFAULT_STATIC_GYR_RPS. [m/s^2] */
#define INS_DEFAULT_STATIC_ACC_MPS2 (0.2f)

/** Stillness DETECTOR thresholds, shared by ins.c's auto-ZUPT/ZARU detector and
 *  ahrs.c's velocity-blind auto-ZARU fallback: "how still is still" is the same
 *  question against the same physical IMU in both places.
 *
 *  The primary criterion is the sample VARIANCE over a short tumbling window,
 *  not the absolute magnitude: a constant sensor bias shifts the mean but not
 *  the spread, so a magnitude-only gate reads a biased but perfectly stationary
 *  IMU as "moving". That is self-locking - the detector is what feeds the
 *  ZUPT/ZARU that would estimate the bias in the first place (ahrs.c has no
 *  accelerometer bias state at all, so there the block is permanent).
 *
 *  The magnitude bounds are KEPT alongside it, because variance alone cannot
 *  see a constant rotation rate or a constant centripetal acceleration (a
 *  steady turntable: near-zero spread, plainly not stationary). Neither
 *  criterion subsumes the other. */
/** Per-axis RMS gyro stddev over the window. This is the criterion that has to
 *  do the actual rejecting, since the magnitude bound below is deliberately
 *  relaxed for bias tolerance: a slow steady turn barely moves the magnitude
 *  but does carry more spread than this.
 *
 *  Tight on purpose, from flight-test experience: at a looser threshold the
 *  detector was observed to arm MID-FLIGHT, injecting a zero-velocity/
 *  zero-rotation update into a moving vehicle. That failure is far more
 *  damaging than the one it trades against (a platform vibrating at standstill
 *  not being recognised as still), because the latter only withholds an update
 *  while the former corrupts the state. A platform in that second category is
 *  expected to raise this threshold explicitly via
 *  auto_zupt_static_gyr_stddev_rps / auto_zaru_static_gyr_stddev_rps.
 *
 *  Needs <math.h> (M_PI) included before this header. [rad/s] */
#define INS_DEFAULT_STATIC_GYR_STDDEV_RPS (0.1f * (float)(M_PI / 180.0))
/** Per-axis RMS accelerometer stddev over the window [m/s^2], same
 *  vibration reasoning as INS_DEFAULT_STATIC_GYR_STDDEV_RPS. */
#define INS_DEFAULT_STATIC_ACC_STDDEV_MPS2 (0.15f)
/** Magnitude sanity bounds, sized for BIAS TOLERANCE rather than for motion
 *  rejection: a turn-on bias below these must not be able to keep the detector
 *  from ever arming, which is the self-lock described above. Rejecting motion
 *  is the variance criterion's job, and it is tightened accordingly - the two
 *  are tuned as a PAIR. Relaxing one of these without also tightening the
 *  variance threshold was measured to start admitting slow steady turns, at a
 *  direct cost in ARS pitch bias. Both bounds are applied to the
 *  bias-CORRECTED sample. [rad/s] / [m/s^2] */
#define INS_DEFAULT_STATIC_GYR_BOUND_RPS (5.0f * (float)(M_PI / 180.0))
/** @copydoc INS_DEFAULT_STATIC_GYR_BOUND_RPS */
#define INS_DEFAULT_STATIC_ACC_BOUND_MPS2 (0.5f)
/** Length of one variance window [s]. A window is evaluated once it
 *  covers BOTH this duration and INS_DEFAULT_STATIC_VAR_MIN_SAMPLES, so
 *  it stretches by itself on a low-rate IMU instead of judging stillness
 *  from a handful of samples. */
#define INS_DEFAULT_STATIC_VAR_WINDOW_SEC (0.2f)
/** Minimum samples in one variance window. */
#define INS_DEFAULT_STATIC_VAR_MIN_SAMPLES (8u)
/** Required dwell time before a stillness detector arms: ins.c's
 *  auto-ZUPT/ZARU and ahrs.c's auto-ZARU fallback (NOT ins.c's unrelated
 *  auto-init leveling *window*, which is a one-shot bootstrap sample and
 *  stays local). [s] */
#define INS_DEFAULT_STATIC_DWELL_SEC (0.2f)

/** Zero-velocity pseudo-measurement 1-sigma, shared by ins.c's own
 *  zero-velocity update and baro_alt.c's vertical-channel ZUPT: both
 *  express the same "platform is standing still" statement, just
 *  projected onto different axes. [m/s] */
#define INS_DEFAULT_ZERO_VEL_STDDEV_MPS (0.05f)

/** Relative tolerance on a throttle's "is this epoch due yet" test, shared by
 *  ins.c's Kalman prediction and ahrs.c's covariance prediction and
 *  zero-rotation rate limit (REQ-NAV-004, REQ-AHRS-021): all three throttle
 *  against the same physical IMU stream. An epoch whose elapsed time reaches
 *  the configured period to within this fraction counts as due.
 *
 *  Needed because a bare ">= period" degrades discontinuously in the one
 *  configuration a caller is most likely to pick: kalman_update_dt_sec set to
 *  the IMU's own sample period. A stream landing marginally BELOW it - a
 *  microsecond of timestamp quantization, or a part running a per-mille above
 *  its nominal rate - then misses every due test, and the throttle silently
 *  runs at half the requested rate.
 *
 *  Safe in the other direction because the propagation is driven by the
 *  MEASURED elapsed time, not by the configured period: the throttle is only a
 *  computational budget. Running it a fraction early costs CPU, skipping it
 *  halves a rate nobody asked to halve. */
#define INS_CADENCE_TOLERANCE (0.01f)

/** "Has @p elapsed_sec reached the throttle period @p period_sec?", with
 *  INS_CADENCE_TOLERANCE applied. Both arguments are evaluated once. */
#define INS_CADENCE_DUE(elapsed_sec, period_sec) \
    ((elapsed_sec) >= (period_sec) * (1.0f - INS_CADENCE_TOLERANCE))

/** chi2inv(0.95, 1): the generic 1-DOF chi-square outlier gate significance
 *  used as the un-configured default by ahrs.c (accelerometer, magnetometer)
 *  and baro_alt.c (barometric altitude). NOT used by ins.c, which keeps its
 *  own looser INS_DEFAULT_CHI2_GATE (ins.c, 10.0) - see the file-level
 *  comment above for why. */
#define INS_DEFAULT_CHI2_95_1DOF (3.8415f)

/** Magnetometer field-strength disturbance gate: relative |B| deviation
 *  from the WMM total field beyond which a sample is downweighted. Shared
 *  by ins.c and ahrs.c, both gating the same physical magnetometer/model
 *  comparison. */
#define INS_DEFAULT_MAG_FIELD_TOLERANCE (0.30f)

/** Fallback 1-sigma for a barometer sample that arrives with stddev_m <= 0 [m].
 *
 *  "How accurate is one barometric height reading" is a property of the
 *  barometer, not of whichever filter consumes it. All three uses ask exactly
 *  that: ins.c's fallback for its own height fusion, baro_alt.c's for its own,
 *  and the offset filter's for the local height it pairs against GNSS.
 *
 *  The ISA conversion constants this default used to sit next to now live in
 *  geodetic_toolbox.h, next to ins_isa_altitude_from_pressure(): they are
 *  physical constants of the atmosphere model, not tuning defaults. */
#define INS_DEFAULT_BARO_STDDEV_M (2.0f)

/** Fallback per-axis 1-sigma for a magnetometer sample that arrives with
 *  Qll_diag <= 0 [uT].
 *
 *  Here for the same reason as INS_DEFAULT_BARO_STDDEV_M above: "how much do I
 *  trust one magnetometer sample" is a property of the sensor and its magnetic
 *  environment. Keeping it here also stops every caller (tools/insrcv.c,
 *  tools/replay.c, python/replay.py) from carrying its own copy.
 *
 *  Deliberately of the same order as the horizontal field itself (~20 uT at
 *  mid-latitudes). ins.c fuses the magnetometer for yaw only, so a per-axis
 *  noise translates into a heading noise of roughly stddev / |B_horizontal|.
 *  That is the intent: the magnetometer keeps the heading from walking away
 *  over minutes, while over seconds a MEMS gyro carries it far better than a
 *  field bent by motor currents can. This only sizes the NOISE - a standing
 *  hard-iron offset is a bias, and no choice of variance removes it.
 *
 *  NOT used by ahrs.c, which states its magnetometer noise directly in the
 *  angle domain (ahrs_config_t.mag_yaw_stddev_rad) because it fuses a scalar
 *  tilt-compensated heading instead of the field vector. */
#define INS_DEFAULT_MAG_STDDEV_UT (20.0f)

#endif /* INS_SENSOR_DEFAULTS_H */
/** @} */
