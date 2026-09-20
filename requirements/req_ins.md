# ins navigation filter requirements (REQ-NAV)

## REQ-NAV-001 — Error-state vector

- **Status:** implemented
- **Parent:** REQ-SYS-001
- **Verification:** Inspection: ins.h state layout (INS_IDX_*)

The filter shall estimate a 15-element error state: position error
(NED), velocity error (NED), attitude error (small-angle roll/pitch/
yaw), accelerometer bias and gyroscope bias.

## REQ-NAV-002 — UDU covariance factorisation

- **Status:** implemented
- **Parent:** REQ-SYS-001
- **Verification:** Inspection: covariance stored as U/d in ins_t, fusion via kalman_udu (Bierman), prediction via kalman_udu_predict (Thornton)

The error-state covariance shall be maintained as a UDU ("square
root") factorisation and updated with the Bierman/Thornton routines
for numerical robustness in single precision.

## REQ-NAV-003 — Strapdown integration

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_stationary; Test: tests/test_ins_core.c:scenario_free_fall; Test: tests/test_ins_core.c:scenario_yaw_rotation

The filter shall integrate bias-corrected IMU measurements into the
nominal state (quaternion attitude update, velocity update with
gravity, Coriolis and rotation correction, trapezoidal position
update), compensating Earth rotation and transport rate.

## REQ-NAV-004 — Throttled covariance prediction

- **Status:** verified
- **Parent:** REQ-SYS-004
- **Verification:** Test: tests/test_ins_core.c:scenario_covariance_grows; Test: tests/test_ins_core.c:scenario_kalman_cadence_tolerance

The Kalman prediction shall run at the configured kalman_update_dt
rate (not the IMU rate), and the state uncertainty shall grow without
aiding. The strapdown integration still runs at the full IMU rate; only
the covariance propagation is throttled -- the error dynamics are slow
enough that a lower cadence (Wendel, 2nd ed., ch. 8.2.1: "typically
10 Hz") tracks them adequately. kalman_update_dt_sec defaults to
1/20 s (20 Hz, a safety margin over Wendel's figure) when left at 0.

The due test shall carry a relative tolerance (INS_CADENCE_TOLERANCE): an
epoch whose elapsed time reaches the configured period to within that
fraction counts as due. Without it the throttle degrades discontinuously
in the one configuration a caller is most likely to choose -- setting
kalman_update_dt_sec to the IMU's own sample period. An epoch stream
landing a hair BELOW the threshold (microsecond timestamp quantization,
or a sensor whose true output rate is marginally above its nominal one)
then misses every due test and the prediction runs on every second epoch,
at half the requested rate and with no diagnostic saying so. The
propagation itself is exact at any cadence (it is driven by the measured
elapsed time, not by the configured period), so the throttle is a
computational budget and erring towards running it is harmless, whereas
erring towards skipping silently halves the rate.

The tolerance shall stay far below the distance to the next candidate
epoch, so it can never pull a prediction a whole epoch early: that
distance is one IMU sample period expressed as a fraction of the
configured period, i.e. at least 20% for any cadence ratio of 5:1 or
tighter, against a tolerance two orders of magnitude smaller.

## REQ-NAV-005 — GNSS position fusion

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_position

The filter shall fuse GNSS position measurements given as geodetic
coordinates (REQ-NAV-079) with a full 3x3 NED covariance, compensating
the antenna lever arm with the attitude at the measurement's time of
validity. The residual shall be formed as the geodetic difference
between the fix and the filter's own absolute anchor (REQ-NAV-080),
mapped to metres with the curvature radii.

## REQ-NAV-006 — GNSS velocity fusion

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_velocity_leverarm

The filter shall fuse GNSS velocity measurements (NED, full 3x3
covariance) including the lever-arm-induced velocity (omega x lever
arm). When position and velocity are fused together, an optional
position/velocity cross-covariance shall complete the 6x6 measurement
covariance.

## REQ-NAV-007 — GNSS quality gates

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_local_pos_gating

GNSS measurements whose reported standard deviation exceeds the
configured horizontal/vertical limits (opt.gnss_max_*) shall be
rejected before fusion and counted in the diagnostics.

This is the *fusion* gate and answers one question only: may this
individual measurement be fused. Whether the filter may enter the 3D
solution (REQ-NAV-051) or has to leave it (REQ-NAV-052) is decided by two
separate threshold sets on the same reported standard deviations.

The gate stays available at any configured limit, but its DEFAULT limits
(REQ-NAV-043) are the accuracy caps of REQ-NAV-071, so an unconfigured
filter rejects no fix for its reported accuracy alone: a fix that honestly
reports a large uncertainty is fused with that large uncertainty instead
of being discarded. Rejection on a broken covariance (a non-positive
diagonal entry) is unaffected and always applies. A caller that does want
a hard accuracy gate sets opt.gnss_max_* below the caps explicitly.

## REQ-NAV-008 — Delayed measurement fusion

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_moving_delayed

The filter shall keep a history of recent states and fuse
measurements that are up to INS_MAX_DELAY_MS old by anchoring the
residual at the state valid at the measurement's time of validity.

## REQ-NAV-009 — Magnetometer fusion, yaw-only

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_mag_yaw; Test: tests/test_ins_core.c:scenario_mag_gating

The filter shall fuse magnetometer measurements against a magnetic
reference vector such that only the heading is corrected: magnetic
disturbances and model errors shall not tilt the roll/pitch estimate.
Fusion shall be rate-limited (magnetometer_min_delay_ms) and skipped
when the horizontal reference field is unusable. The rate limit shall
default to a value well below a typical magnetometer output rate, so
that an unconfigured filter treats the magnetometer as a long-term
heading anchor rather than a per-epoch yaw sensor; a negative value
shall disable the rate limit. A sample supplied without a per-axis
variance shall be fused with the library's own magnetometer noise
default, never as a zero-noise measurement.

## REQ-NAV-010 — Absolute yaw aiding

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_yaw_aiding

The filter shall fuse absolute yaw measurements (e.g. dual-antenna
GNSS, mocap pose) as scalar heading residuals, optionally delayed via
the history, and shall skip fusion near gimbal lock
(|pitch| -> 90 deg).

## REQ-NAV-011 — Local position aiding

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_local_pos_lighthouse

The filter shall fuse local NED position measurements (lighthouse,
UWB, mocap) with full covariance, sensor lever arm and optional
delay. Outliers shall be skipped (not downweighted): such systems
glitch and run at high rate, so dropping a sample is cheap.

## REQ-NAV-012 — Zero-velocity / zero-rotation updates

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_zupt; Test: tests/test_ins_core.c:scenario_zero_rotation_bias

On external trigger the filter shall fuse a zero-velocity update
(direct velocity measurement) and a zero-rotation update (direct
gyro-bias measurement, compensating the Earth/transport rate seen in
the body frame), each rate-limited to one fusion per Kalman period.

## REQ-NAV-013 — Automatic ZUPT/ZARU detection

- **Status:** verified
- **Parent:** REQ-NAV-012
- **Verification:** Test: tests/test_ins_core.c:scenario_auto_zupt; Test: tests/test_ins_core.c:scenario_static_variance_gate

The filter shall detect stillness autonomously and inject
zero-velocity/zero-rotation updates without an external trigger,
rate-limited by a configurable minimum interval. The detector shall
be enabled by default and fully disableable. A public accessor
(ins_auto_zupt_active) shall report whether the detector currently
considers the filter stationary, for diagnostics/telemetry and so
nav_suite can propagate the (velocity-aware) trigger to the ARS/AHRS
(REQ-SUITE-009), which cannot detect stillness on their own. That
accessor shall apply every criterion the detector itself applies: it
feeds filters that have no way to second-guess it, so reporting
stillness the detector would refuse to act on is not admissible.

The primary stillness criterion shall be the per-axis sample VARIANCE
of the raw (not bias-corrected) gyro and accelerometer over a short
tumbling window, evaluated once the window covers both a minimum
duration and a minimum number of samples, with the verdict latched
until the next window completes. Before the first completed window the
verdict shall be "not still", so no update can be triggered off a
single sample. An epoch carrying no IMU sample shall leave the window
and the latched verdict untouched: absence of evidence is not evidence
of motion, and clearing the verdict there would let every interleaved
sensor epoch of a mixed-rate stream veto the detector.

Rationale for variance rather than magnitude: a constant sensor bias
shifts the sample mean but not its spread. A magnitude-only gate
therefore reads a biased but perfectly stationary IMU as moving, and
that failure is self-locking -- this detector is what feeds the
ZUPT/ZARU that would estimate the bias in the first place, so a sensor
whose turn-on bias exceeds the gate can never escape it.

The magnitude gates (|gyro|, ||f| - g|) shall be retained alongside the
variance criterion, because variance alone cannot distinguish a
constant rotation rate or a constant centripetal acceleration (steady
turntable or circular drive: near-zero spread, plainly not stationary)
from a standstill. The two criteria cover different failure modes and
neither subsumes the other, so both must hold. In particular the
magnitude gates shall NOT be widened to accommodate a large turn-on
bias: they are applied to the bias-CORRECTED sample (so the conflict is
transient), and widening them measurably admits slow steady turns that
no variance threshold can reject. A sensor whose uncorrected bias
genuinely exceeds them must widen the configurable gates explicitly.

Both variance thresholds shall be configurable
(auto_zupt_static_gyr_stddev_rps / auto_zupt_static_acc_stddev_mps2,
0 -> built-in default). Raising them out of range leaves the magnitude
bounds alone in charge, which is what a platform whose standstill is
inherently noisy requires -- a station-keeping multicopter holds a
near-zero mean velocity while its rotors put more spread on the IMU
than handheld motion does.

The velocity gate shall use exclusively a recent external (GNSS)
velocity observation, never the filter's own state estimate (gating on
the own-state velocity would be circular: an uncompensated bias tilts
the attitude, gravity leaks into the horizontal velocity states, the
state estimate crosses the gate before the detector ever fires, and
once that has happened the detector can never become armed again to
correct it). The GNSS observation shall additionally meet its own
configurable accuracy requirement (auto_zupt_max_vel_stddev_mps,
1-sigma per axis, 0 -> built-in default): a fix reporting a low speed
with a large uncertainty is not evidence of standstill, so it shall
not be used for the gate even if otherwise usable for fusion (that is
a separate, looser question answered by opt.gnss_max_*_vel_stddev_mps).
Without a GNSS velocity observation meeting both the recency
(INS_AUTOZUPT_EXT_VEL_MAX_AGE_SEC) and the accuracy requirement, the
velocity gate shall not be applied at all -- it shall pass
unconditionally and only the magnitude/variance IMU criteria decide.
This is a deliberate, accepted blind spot for pure-inertial coasting
(no GNSS at all): a body cruising in a straight line at constant,
non-zero velocity also shows a gravity-only specific force and
near-zero rotation rate, indistinguishable from standstill by the IMU
criteria alone. A state-estimate fallback would close that blind spot
but reopen the circular one above, which is the worse failure.

The detector shall additionally be disableable at runtime
(ins_set_auto_zupt_disable), independent of opt.auto_zupt_disable set
at ins_init: e.g. a multicopter briefly holding position mid-flight
must be able to guarantee no zero-velocity/zero-rotation update can
fire for the duration, then resume automatic detection once grounded.
Disabling shall clear the in-progress dwell timer and the latched
variance-window verdict, so no evaluation from before the disable can
fire the instant the detector is re-enabled -- re-enabling shall
always require a fresh stillness run observed from scratch. An
externally-triggered update (ins_measurements_t.zero_velocity_update /
zero_rotation_update) is unaffected either way, remaining entirely
under the caller's control.

## REQ-NAV-014 — ZARU vibration immunity

- **Status:** verified
- **Parent:** REQ-NAV-013
- **Verification:** Test: tests/test_ins_core.c:scenario_auto_zaru_vibration; Test: tools/replay.c:main

The zero-rotation update shall fuse the gyro measurement averaged
over the current stillness run instead of a single sample, so that
zero-mean vibration (e.g. idling engine) does not leak into the gyro bias
states.

## REQ-NAV-015 — Auto-initialization

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_autoinit_gnss; Test: tests/test_ins_core.c:scenario_autoinit_local_pos; Test: tests/test_ins_core.c:scenario_autoinit_mag_yaw; Test: tests/test_ins_core.c:scenario_autoinit_yaw_priority; Test: tests/test_ins_core.c:scenario_autoinit_buffer_wraparound

When enabled, the filter shall bootstrap itself from the measurement
stream: roll/pitch from accelerometer leveling over a window (median),
yaw from an external yaw measurement, else the magnetometer, else left
unknown with a correspondingly large covariance, position/velocity from
the first usable fix. The leveling window does not need to be
quasi-static -- see REQ-NAV-047 for how the roll/pitch uncertainty
reflects that. This is the baseline, standalone behavior: it is what a
caller gets from ins.c alone, unless it explicitly supplies an
external attitude/gyro-bias hint (REQ-NAV-048), which then takes
precedence over the leveling result.

## REQ-NAV-016 — Time-jump handling

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_time_jump_handling; Test: tests/test_ins_core.c:scenario_time_jump_unlimited_dr_recovery; Test: tests/test_ins_core.c:scenario_time_jump_reset_reacquires

The filter shall tolerate timestamp anomalies of a *running* filter:
epochs older than the last prediction shall be handled up to the
history depth and dropped beyond it (a *sustained* run of such drops is
a restarted time source, handled by REQ-NAV-070); forward jumps beyond the maximum
prediction time shall force a defined reset -- unless
allow_unlimited_deadreckoning is enabled, in which case the filter
shall instead coast: the jump epoch is skipped (no strapdown, no
predict, no fusion) but the filter re-baselines its internal
time-jump reference to that epoch's timestamp, so that normal-rate
epochs immediately following the gap are recognized and processed
again rather than the filter remaining permanently stuck re-detecting
the same stale jump; all cases shall be counted in the diagnostics.
Unless opt.auto_reacquire_disable is set, and provided auto_init, a forced
time-jump reset shall additionally re-arm into the collecting state
via the same autonomous-reacquisition mechanism as a health-check
shutdown (REQ-NAV-042), instead of leaving is_collecting false
alongside is_initialized false -- a state from which the filter can
otherwise never recover on its own, since ins_update's top-level
"not initialized" branch only re-runs auto-init when is_collecting is
true. Startup stream coherence (before the filter has started) is a
separate concern, handled by REQ-NAV-033.

## REQ-NAV-017 — Conservative noise defaults

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_default_mems_noise

IMU noise densities and bias random walks left unset (0) by the
caller shall fall back to conservative consumer-MEMS defaults instead
of injecting zero process noise.

## REQ-NAV-018 — Non-finite input sanitization

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_ins_core.c:scenario_nan_inf_inputs

ins_update shall validate every consumed measurement field for
finiteness before any math: non-finite sensor payloads invalidate the
affected measurement block, non-finite noise densities fall back to
defaults, non-finite optional fields (lever arms, cross-covariance)
are zeroed, and each dropped block increments
ins_diag_t.n_invalid_input. This covers the external attitude hint
(REQ-NAV-048) as well, whose angles and gyro biases are consumed
without a residual or a chi2 gate and would otherwise propagate a
non-finite value straight into the nominal state.

## REQ-NAV-019 — Diagnostics

- **Status:** implemented
- **Parent:** REQ-SYS-001
- **Verification:** Inspection: ins_diag_t, counters exercised throughout tests/test_ins_core.c

The filter shall keep passive diagnostic counters for silent
reject/skip paths (timing anomalies, GNSS gating and residuals,
fusion failures, auto-ZUPT triggers, invalid inputs) readable via
ins_get_diag(), so field issues can be diagnosed without
recompiling. The filter shall never act on these counters.

## REQ-NAV-020 — Float hot path

- **Status:** implemented
- **Parent:** REQ-SYS-004
- **Verification:** Inspection: doubles only in the geodetic anchor code paths (init, the GNSS residual, accessors)

The per-epoch hot path (strapdown, prediction, fusion) shall use
single precision; double precision shall be limited to the
absolute-position anchor (the geodetic origin, the lat/lon/height
book-keeping and the geodetic differences formed against them).

## REQ-NAV-021 — Unlimited dead reckoning option

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_deadreckoning_reacquire

With allow_unlimited_deadreckoning enabled, the filter shall survive
arbitrarily long IMU-only periods without declaring itself unready;
the coasting window (REQ-NAV-022) and re-acquisition (REQ-NAV-023)
are disabled in this mode.

## REQ-NAV-022 — Dead-reckoning coasting window

- **Status:** verified
- **Parent:** REQ-SYS-010
- **Verification:** Test: tests/test_ins_core.c:scenario_deadreckoning_reacquire; Test: tests/test_ins_core.c:scenario_deadreckoning_freeze; Test: tests/test_ins_core.c:scenario_coasting_window_still_fuses

The filter shall track the time since the last absolute position
aiding (GNSS or local position fusion) and expose it via
ins_deadreckoning_ms(). Once this time exceeds the configurable
coasting window (max_deadreckoning_sec, default 10 s),
ins_is_ready() shall return false -- the coasted position is no
longer trustworthy -- and the filter shall stop strapdown integration
while the window stays expired, so attitude, IMU bias, position and
velocity all hold their last value instead of dead-reckoning through
an outage of unknown length. The covariance is frozen with them, as is
every other part of the filter: an expired window makes the whole
instance inert until it re-acquires (REQ-NAV-064). Not applicable with
allow_unlimited_deadreckoning.

## REQ-NAV-023 — Re-acquisition after an expired coasting window

- **Status:** verified
- **Parent:** REQ-SYS-010
- **Verification:** Test: tests/test_ins_core.c:scenario_deadreckoning_reacquire; Test: tests/test_ins_core.c:scenario_deadreckoning_reacquire_far_from_origin; Test: tests/test_ins_core.c:scenario_deadreckoning_velocity_only_no_reacquire

The first usable position fix (GNSS or local position) after an
expired coasting window shall re-anchor the filter instead of being
fused as a residual: position (and velocity, if measured) are set
from the measurement with covariance rebuilt from the measurement
noise -- except the vertical position under the barometric height
source, which is re-anchored on the barometer instead (REQ-NAV-066) --
while the frozen attitude and IMU bias estimates -- unchanged since the
window expired (REQ-NAV-022) -- are kept, with their variances inflated
for the frozen duration (REQ-NAV-065), UNLESS an external
attitude/gyro-bias hint is supplied (REQ-NAV-048), in which case that
takes precedence. The yaw VARIANCE is the one exception to "kept": with
no hinted heading it is reset to an unknown-heading prior
(REQ-NAV-059). The stale history buffer shall be invalidated. The
filter shall be ready again immediately after re-acquisition (no
warm-up); re-acquisitions are counted in the diagnostics
(n_reacquire).

A GNSS velocity-only fix (usable velocity, not yet usable position --
a common receiver behaviour right after a tunnel or urban canyon,
where velocity/Doppler re-converges before position) shall NOT count
as position aiding: it must not un-expire the coasting window on its
own. Otherwise the first usable position fix arriving afterwards would
find the window already un-expired and be fused as an ordinary (and,
after an outage long enough to trip this requirement, hopeless)
residual instead of re-anchoring.

"Velocity-only" here means the position did not pass the fusion gates
(REQ-NAV-005) or was not offered at all. An epoch whose position was
usable but withheld by the position decimation (REQ-NAV-063) is NOT a
velocity-only fix in this sense and does count as position aiding: the
fix carried the evidence, the filter chose not to spend it. The
decimation is suspended for as long as the window is expired, so the
first fix out of the tunnel re-anchors here regardless of where its
cycle stood.

The re-anchored ABSOLUTE position shall be the fix itself (less the antenna
lever arm), to full double precision and independent of where the n-frame
origin sits. The local position shall be reached as a step from the position
the filter is holding, not as a difference against the origin. Both mappings
between a geodetic difference and an n-frame offset are tangent-plane
approximations valid only for a short baseline: evaluated over d_north by
d_east they drop a curvature cross term of about
d_north * d_east * tan(lat) / R_earth, and the forward and inverse
directions do not even cancel unless they are evaluated at the same
latitude. Sending the full origin-to-fix baseline through them therefore
re-anchors the filter beside the fix rather than on it -- hundreds of metres
beside it after a hundred kilometres of travel, injected under the
metre-scale prior this requirement seeds the covariance with. The chi2
outlier gate (REQ-NAV-035) then does what it is meant to do with a
several-hundred-sigma residual and downweights every following fix, so the
error bleeds off over tens of minutes instead of being corrected at the
tunnel exit.

Not to be confused with the quality-loss exit (REQ-NAV-052), which answers
a different failure with a stronger remedy: this requirement covers "no
aiding at all for a while" and re-anchors the running instance in place,
keeping is_initialized true throughout, whereas REQ-NAV-052 covers "aiding
that keeps arriving but is too poor" and tears the instance down.

## REQ-NAV-024 — Lever-arm attitude coupling in the measurement Jacobian

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_leverarm_yaw_from_position; Test: tests/test_ins_core.c:scenario_gnss_leverarm_yaw_from_velocity; Test: tests/test_ins_core.c:scenario_local_pos_lighthouse_yaw

For any position or velocity measurement fused with a non-zero sensor
lever arm, the measurement Jacobian shall include the attitude coupling
of the lever arm, not just add it to the residual. A lever-arm-induced
residual shall therefore be observable in the attitude states so the
filter can correct heading/tilt from it.

- Position measurements (GNSS, local NED) shall use
  H_pos = [ I3 | 0 | -[l^n]_x | 0 | 0 ] with l^n = R_b_to_n * l^b the
  lever arm rotated into the n-frame at the measurement's time of
  validity (Wendel, 2nd ed., eq. 8.62).
- Velocity measurements (GNSS) shall use
  H_vel = [ 0 | I3 | -[R_b_to_n * (omega x l^b)]_x | 0 | 0 ], the
  attitude coupling of the lever-arm velocity (Wendel, 2nd ed.,
  eq. 8.74).

## REQ-NAV-025 — Vertical origin relocation

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_ins_core.c:scenario_origin_shift

The filter shall provide an operation that moves the n-frame origin
vertically (along the local down axis) by a given distance without
changing the absolute solution: the latitude/longitude/height and
ECEF outputs are invariant, only pos_local (nominal state and history
entries) shifts by the same amount. The covariance is untouched (the
shift is deterministic). Intended use: aligning the local vertical
datum with an external height reference (e.g. the barometric filter's
datum) directly after (auto-)initialization; callers of local NED
position aiding must align their frame after the shift.

## REQ-NAV-026 — Independent initial yaw uncertainty

- **Status:** verified
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_initial_yaw_stddev_override; Test: tests/test_ins_core.c:scenario_autoinit_mag_yaw

rpy_init_stddev_rad[2] (yaw) shall set the initial yaw variance
independently of rpy_init_stddev_rad[0]/[1] (roll/pitch): all three
axes are independently settable, and each one left at 0/unset
resolves to the same default on its own, with no cross-axis
inheritance.
Rationale: roll/pitch (accelerometer leveling) and yaw (external
measurement, magnetometer, or unknown) are observed through
physically different, generally much less accurate sources, so
coupling all three to one stddev either overstates yaw confidence or
needlessly inflates roll/pitch. Applies to both the manual init path
and auto-init's magnetometer-derived yaw (REQ-NAV-015); auto-init's
external-yaw-measurement and yaw-unknown cases already use their own
independent variances and are unaffected.

## REQ-NAV-027 — Magnetic reference field from position

- **Status:** verified
- **Parent:** REQ-SYS-013
- **Verification:** Test: tests/test_ins_core.c:scenario_magnetic_model_from_position

ins shall provide an interface to set the NED magnetic reference
field from a position (latitude, longitude, decimal year) via the
World Magnetic Model (declination, inclination, total field). Because
the reference carries the declination, the magnetometer-aided yaw is
then relative to true north. The interface may be called at any time
(e.g. once a first position becomes available) and shall ignore a
non-finite argument.

When the supplied position lies inside a magnetic dip pole exclusion
zone (REQ-SYS-018), ins shall suspend magnetometer fusion and retain
its previous reference field rather than store one built from a
meaningless declination. The auto-init yaw bootstrap shall likewise
skip its magnetometer stage inside such a zone and fall through to the
unknown-heading case (REQ-NAV-048), so a filter started there still
runs with valid roll and pitch and an honest yaw covariance instead of
a heading seeded from an ill-conditioned declination. Fusion and the
reference field resume on the first position outside the zone.

## REQ-NAV-028 — Magnetometer field-strength disturbance rejection

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_magnetic_model_from_position

When a position has been supplied (REQ-NAV-027), ins shall compare
the measured magnetic field magnitude against the World Magnetic Model
total field and downweight (inflate the per-axis measurement noise of)
a magnetometer sample whose magnitude deviates by more than a
configurable tolerance (default 30%). This complements the vector
residual chi2 gate for the case of a loosely-trusted magnetometer,
where a magnitude anomaly would otherwise bias yaw. The gate shall be
defeatable for an uncalibrated magnetometer.

## REQ-NAV-029 — Magnetometer hard-iron bias estimation

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_mag_bias_estimation

ins shall optionally (off by default, enabled per instance at init)
extend its error state by a 3-state body-frame magnetometer hard-iron
bias, modelled as an additive offset on the magnetometer measurement
with a random-walk process model, and estimate it online through the
magnetometer fusion. With the option off the filter shall run the
unchanged 15-state formulation. The bias states shall use the same
unit as the magnetometer/model (uT). Soft-iron (matrix) effects are
out of scope (offline calibration). Rationale: the hard-iron bias is
only separable from the yaw error under attitude changes (the bias is
fixed in the body frame while the reference field is fixed in the
navigation frame), so it is estimated inside the filter where the
yaw/bias cross-covariance is available, rather than in an external
estimator. The maximum state size is a compile-time constant
(INS_UNKNOWNS_MAX, default 18) so memory-constrained integrations
can reclaim the extra covariance/history storage by overriding it to
15, which disables the option.

## REQ-NAV-030 — GNSS latency from GPS time of validity

- **Status:** deleted
- **Parent:** REQ-NAV-008
- **Verification:** Inspection: obsolete, GNSS latency is an integration parameter (gnss_delay_ms, REQ-NAV-008).

Not pursued. The GNSS measurement latency is stated explicitly by the
integration via gnss_delay_ms and can be estimated offline with the
post-processing tools. The filter does not derive it from the GPS time
of validity: that would compensate the processing and transport delay
(receiver output, UART), but not the group delay of the receiver's
internal filtering and smoothing, which still places the effective
measurement epoch further in the past than its time tag.

## REQ-NAV-031 — Numerical robustness with degenerate process noise

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ins_core.c:scenario_degenerate_process_noise

The filter shall remain numerically live when configured with
degenerate (near-zero, but positive) bias process noise densities
under tight aiding that collapses the covariance (cm-level position
fixes plus automatic ZUPT/ZARU on a parked vehicle): the covariance
factors shall stay finite and positive (variance floor
INS_MIN_VARIANCE), prediction and fusion shall keep running and
the health check shall not shut the filter down. Rationale: observed
on the i2Nav/KF-GINS parked phase (2026-07-05) with a GM-derived
gyro-bias random walk of ~1.5e-9 rad/s/sqrt(s) (Q ~ 1e-20 in single
precision) the replay stopped updating; the synthetic reproduction
attempts (100 Hz and 200 Hz, cm-RTK, auto-ZUPT, 300 s) stay healthy,
so this pins the property as a regression gate while the field
observation remains under investigation on the dataset side.

## REQ-NAV-032 — Runtime world model update

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_set_world_model

ins_set_world_model shall replace the gravity and/or magnetic
reference vectors of a running filter; a NULL argument shall leave
the corresponding vector unchanged. The magnetometer fusion shall
follow the updated reference (the estimated yaw converges to the new
horizontal field direction). Unlike the WMM path (REQ-NAV-027) no
field-strength gate is armed -- the caller supplies raw reference
vectors in units of their choice.

## REQ-NAV-033 — Startup stream-coherence gate

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_startup_alignment_gate

The filter shall not begin propagating or fusing until the measurement
streams are coherent enough to work with. With a limited dead-reckoning
budget (allow_unlimited_deadreckoning == false) it shall consume
measurements -- without predicting or fusing, keeping is_initialized
false -- until it holds both a valid IMU sample and a usable absolute
position fix (GNSS or local NED) whose timestamps lie within
max_prediction_time_sec of each other; only then does it start,
applying the prescribed init values (or, with auto_init, the
bootstrapped state) stamped at that aligned epoch rather than at the
supplied init->time. With allow_unlimited_deadreckoning enabled a
manual (prescribed-value) init may start on the first valid IMU sample
alone (pure dead reckoning; the caller then owns the unaided drift).
Rationale: a prescribed init->time can predate the first IMU/GNSS epoch
(e.g. a reference-derived start where the first IMU sample trails the
reference epoch by more than max_prediction_time_sec), which
the mid-stream time-jump guard (REQ-NAV-016) would otherwise mistake
for a forward jump and reset; and a lone fix arriving seconds before
the IMU stream begins must not bootstrap the filter from temporally
incoherent data.

## REQ-NAV-034 — Automotive yaw from GNSS course

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_automotive_gnss_yaw

When automotive mode is enabled (opt.automotive_mode, opt-in, off by
default) the filter shall derive a yaw (heading) measurement from the
GNSS velocity vector: the course over ground atan2(vE, vN) is fused as
the vehicle heading under the non-holonomic assumption that the vehicle
travels in the direction it points. It shall only do so above a minimum
horizontal ground speed (below which the course is noise-dominated and
the assumption breaks), configurable via opt.automotive_min_speed_mps
(0 -> a 2 m/s built-in default). The fused measurement shall affect yaw only,
never pitch or roll. Its stddev shall be the velocity-noise-over-speed
heading uncertainty, floored at a conservative minimum -- configurable
via opt.automotive_min_yaw_stddev (0 -> a 5 deg built-in default) -- so a
fast, low-noise fix never claims more precision than the model assumption
warrants. The measurement shall respect the GNSS delay (residual
anchored at the time-of-validity). When automotive mode is off the GNSS
velocity shall not contribute any yaw update.

A transient course outlier -- most notably a vehicle that briefly
reverses, giving a ~180 deg course-vs-heading residual -- shall not
corrupt the yaw estimate: the residual is chi2-downweighted like any
persistent absolute yaw reference (not skipped), so its per-epoch gain
is bounded and the heading stays near its established value rather than
flipping. LIMITATION (out of scope): forward and reverse motion are
indistinguishable from the GNSS velocity vector alone (course theta vs
theta+180 look identical without a second heading source such as wheel
odometry or gear state). The downweight bounds the per-epoch pull, not
its accumulation, so *sustained* reversing can still drag yaw over many
epochs. Automotive mode therefore assumes predominantly forward travel;
handling sustained reverse robustly needs an external forward/reverse
reference and is not provided here.

LIMITATION (out of scope): the minimum-speed gate only screens out
noise-dominated readings; it does not validate the non-holonomic
assumption itself, which requires ground velocity direction to equal
heading. That holds for a wheeled vehicle (tire grip suppresses lateral
motion) but not in general for an aircraft: in coordinated flight the
airframe's heading aligns with the *airspeed* vector, not the *ground*
velocity vector GNSS measures, so any crosswind introduces a wind-drift
(crab) angle between course-over-ground and true heading regardless of
speed. Automotive mode is therefore not a substitute for a real heading
reference (magnetometer, GNSS compass) on airborne platforms.
opt.automotive_min_yaw_stddev can bound the resulting error (set it to a
conservative estimate of the expected crab angle, e.g. 15 deg) so the
fusion still acts as a weak long-term anchor against free-yaw drift
without claiming automotive-grade precision -- this manages the
consequence, it does not remove the underlying assumption violation.

## REQ-NAV-035 — chi2 override (ins)

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tests/test_ins_core.c:scenario_chi2_disable

When opt.chi2_disable is set, every chi2-downweighted absolute
reference fused through ins_fuse() (GNSS position/velocity,
magnetometer, local-position, yaw residual incl. automotive_mode)
shall be fused at its nominal variance regardless of the innovation
size -- the chi2 test is skipped entirely, nothing is ever
downweighted. Zero-velocity/zero-rotation updates are unaffected (they
already fuse unconditionally, chi2_threshold == 0.0f).

## REQ-NAV-036 — Downweight diagnostic counter (ins)

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_chi2_disable

ins_diag_t shall expose a monotonic counter, n_downweighted,
incremented once per ins_fuse() call in which at least one
measurement row's chi2 test tripped and was downweighted. The counter
is diagnostic only (does not affect the fusion) and is an
approximation for a multi-row batch (GNSS position+velocity,
magnetometer): the check re-uses the covariance from before the call's
own sequential per-row updates, so it is exact for the first row of a
batch but may over/under-count later rows relative to kalman_udu's own
internal, evolving test. It stays 0 whenever REQ-NAV-035's
chi2_disable override is set (the chi2 test is skipped entirely, so
nothing can trip it).

## REQ-NAV-037 — IMU calibration (misalignment + fixed bias)

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_imu_calibration

The filter shall, at the measurement boundary and before any downstream
use (estimated-bias subtraction, process-noise gates, auto-init
leveling, ZUPT/ZARU stillness gates, strapdown, fusion), apply a
per-sensor calibration to the raw accelerometer and gyroscope samples:
corrected = M * (raw - fixed_bias), where M is the column-major 3x3
misalignment/scale matrix (opt.imu_acc_misalignment /
opt.imu_gyr_misalignment) and fixed_bias the permanent calibration
offset (opt.imu_acc_fixed_bias / opt.imu_gyr_fixed_bias). An all-zero M
is treated as identity so a zeroed options struct is a no-op. The fixed
bias is removed permanently and is never estimated -- it is distinct
from ins_init_t.init_acc_bias / init_gyr_bias, which only seed the
estimated bias state. Accelerometer and gyroscope carry independent
matrices and biases.

## REQ-NAV-038 — GNSS covariance conditioning (scale + floor)

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_cov_conditioning; Test: tests/test_ins_core.c:scenario_gnss_cov_conditioning_gates_unscaled

The filter shall, at the measurement boundary and before fusion,
condition the reported GNSS NED covariances
to counter over-optimistic vendor values, independently for the
position and velocity blocks:
stddev_axis = min(max(scale * stddev_reported, floor_axis), cap_axis).
The scale (opt.gnss_pos_cov_scale / opt.gnss_vel_cov_scale)
multiplies the reported stddev -- i.e. the full covariance, including
off-diagonal terms and the pos/vel cross-covariance, by scale^2, so the
correlation structure is preserved. The floor then lifts each diagonal
variance so the per-axis stddev is never below the floor, with separate
horizontal (N,E) and vertical (D) floors
(opt.gnss_pos_stddev_floor_hor_m / _ver_m,
opt.gnss_vel_stddev_floor_hor_mps / _ver_mps). The cap of REQ-NAV-071 is
applied last. A scale of 0 means 1.0 (no change).

The floors, unlike the scale, are resolved to a non-zero built-in default
when left at 0 (REQ-NAV-043): a receiver reporting a per-axis accuracy
far below what a standalone GNSS solution can physically deliver is the
normal case, not the exception, and a filter that believes it will chase
the fix instead of the dynamics. A caller who genuinely wants no floor on
an axis group states that explicitly with a negative value, which
ins_init resolves to "disabled" -- the same explicit-opt-out convention
opt.gnss_pos_decimation uses (REQ-NAV-063).

The full conditioning pipeline, in order, is: the accuracy envelope of
REQ-NAV-072, the scale above, the dedicated height scale of REQ-NAV-041,
the floor, the cap of REQ-NAV-071, and finally the manoeuvre-dependent
velocity term of REQ-NAV-073.

The conditioned covariance is the measurement noise the FUSION weights
the fix with, and nothing else. The fix-quality gates shall keep grading
the covariance the receiver actually reported, unconditioned: the
per-epoch fusion gate (REQ-NAV-007), the 3D entry gate (REQ-NAV-051),
the 3D exit gate (REQ-NAV-052) and the auto-ZUPT external-velocity
accuracy gate (REQ-NAV-013). The same holds for the operator-facing gate
log line, which reports the measured value against its limit and must
therefore quote the number the limit is expressed in.

Rationale: every one of those gates rejects a fix whose reported stddev
is too LARGE, so sharing one number couples two unrelated decisions. In
the intended use (scale >= 1 or a floor, countering vendor optimism) the
conditioning inflates, and a scale grown past the ratio between the
vendor's optimism and the tightest gate silently turns "weight this fix
less" into "stop using this receiver" -- with the gate that bites first
being the tightest one rather than the relevant one. A scale below 1 has
the mirrored effect and lets a fix through a gate it should have failed.
Either way the two questions are different ones -- how much should this
fix move the state, versus is this receiver still healthy -- and only the
first is a tuning decision. The second is a statement about the receiver,
and the only evidence for it is the accuracy the receiver itself reports.

Unlike the scale step, the floor is NOT correlation-preserving: it raises
Q's diagonal entries independently, without touching the N-E/N-D/E-D
off-diagonal terms, whenever the reported stddev is below the configured
floor. This is equivalent to adding independent extra variance on the
floored axis (the reported correlation coefficient for that axis shrinks
accordingly), not to rescaling the whole 3x3 block to keep the vendor's
correlation shape -- that would need eigenvalue-based conditioning
instead of the three independent per-element clamps used here. Still
PSD-safe (raising one diagonal entry alone cannot make the matrix
indefinite). This is intentional: "never trust a reported variance
tighter than the floor" is meant literally, independent of what
correlation the vendor reports alongside it.

## REQ-NAV-039 — Magnetometer calibration (misalignment + fixed bias)

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_mag_calibration

The filter shall, at the measurement boundary and before any downstream
use, apply a fixed calibration to the raw magnetometer sample using the
same model as the IMU calibration (REQ-NAV-037): corrected = M * (raw -
fixed_bias), where M is the column-major 3x3 soft-iron/scale/misalignment
matrix (opt.mag_misalignment, all-zero -> identity) and fixed_bias the
permanent hard-iron offset removed (opt.mag_fixed_bias, same unit as the
mag/model). This fixed calibration is independent of the optional
18-state estimated hard-iron bias (opt.estimate_mag_bias): the fixed term
removes the calibrated part, the estimated state tracks the residual. A
zeroed options struct is a no-op. The application is shared with the IMU
calibration through ins_apply_calibration().

## REQ-NAV-040 — Overconfidence / covariance-collapse watchdog

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ins_core.c:scenario_overconfidence_watchdog

The filter shall, each post-init epoch, monitor its own reported 1-sigma
accuracy (from the state covariance diagonal) for the position, velocity
and attitude channels, and flag when any per-axis value falls below a
physically implausible floor (INS_OVERCONF_POS_STDDEV_M = 0.1 mm,
INS_OVERCONF_VEL_STDDEV_MPS = 0.1 mm/s, INS_OVERCONF_ATT_STDDEV_DEG =
0.001 deg) -- a symptom of covariance collapse, where the filter becomes so
certain of its state that it effectively stops trusting new measurements and
can silently diverge. ins_diag_t shall expose a latched flag
(overconfident), a monotonic epoch counter (n_overconfident) and the
smallest per-axis 1-sigma seen since ins_init for each channel
(min_pos_stddev_m, min_vel_stddev_mps, min_att_stddev_deg; INFINITY until
the first post-init epoch). This is purely diagnostic: the filter shall
never act on it (distinct from the far lower INS_MIN_VARIANCE numerical
floor, which is a control safeguard).

## REQ-NAV-041 — Dedicated GNSS position vertical (height) downweight

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_pos_height_scale

The filter shall, at the measurement boundary as part of the GNSS
covariance conditioning (REQ-NAV-038) and before fusion,
support an additional dedicated scale on the vertical (Down /
height) axis of the GNSS *position* covariance only, so a receiver whose
height solution is far worse than its horizontal solution (a common GNSS
failure mode) can be downweighted without inflating the horizontal fix.
The scale (opt.gnss_pos_cov_scale_height) multiplies the reported height
1-sigma; a value of 0 means 1.0 (no change). To preserve the covariance's
correlation structure and keep it positive-semidefinite, it shall be
applied as the congruence Q' = S Q S with S = diag(1, 1, s_height), i.e.
the Down variance is multiplied by s_height^2 and every Down-coupled
off-diagonal term (the N-D and E-D covariances, and the position rows of
the position/velocity cross-covariance) by s_height, while the purely
horizontal terms (N, E) are untouched -- so the N-D and E-D correlation
coefficients are exactly preserved. It composes with the isotropic
position scale of REQ-NAV-038, which is applied first; the height scale
applies only to the position block, never to the velocity block. The
vertical floor (opt.gnss_pos_stddev_floor_ver_m) is applied after both
scales, unchanged. Being part of the same conditioning step, it shares
REQ-NAV-038's scope: it weights the fusion only, and the fix-quality
gates keep grading the reported vertical accuracy.

## REQ-NAV-042 — Autonomous re-acquisition after a health-check shutdown

- **Status:** implemented
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ins_core.c:scenario_auto_reacquire_after_health_fail; Test: tests/test_ins_core.c:scenario_time_jump_reset_reacquires

Unless opt.auto_reacquire_disable is set, AND provided auto-init is enabled
(opt.auto_init), a health-check shutdown (a non-finite or negative
covariance factor, or a non-finite nominal position/velocity/attitude
state) shall not leave the filter permanently dead: the filter shall
return to the collecting state (is_initialized false, is_collecting true)
and re-bootstrap autonomously from the next temporally-coherent IMU+fix
window via the normal auto-init path (REQ-NAV-033), without an external
ins_init call. Each such event shall increment ins_diag_t.n_health_reset,
and the monotonic diagnostic counters shall be preserved across the re-arm
(only ins_init resets them). This is the default (opt.auto_reacquire_disable
== false, i.e. a zero-initialised options struct): a caller running only
nav_suite_update/ins_update in its loop must not silently lose the
navigation solution on a transient corruption unless it explicitly opts
out. With the option set, or under a manual/prescribed-value init
(auto_init off), the filter shall stay shut down as before opting in was
possible, and an explicit ins_shutdown() shall always stay dead regardless
of the option. Rationale: the parallel ARS/AHRS/baro_alt filters already
recover autonomously on !is_initialized (nav_suite re-bootstrap); ins was
the asymmetric exception, hence the default flip. Restricting the recovery
to auto_init is deliberate: a re-armed manual init would re-apply the
(possibly stale) prescribed origin/attitude stamped at the recovery epoch,
which is wrong for a moving platform, so that case keeps the caller in
control regardless of the option.

The same option gates the identical re-arm for the two other mid-run
"is_initialized = false" events, a forced time-jump reset (REQ-NAV-016) and
a GNSS quality-loss exit (REQ-NAV-052): those paths increment
ins_diag_t.n_time_jump_reset and n_gnss_quality_exit instead of
n_health_reset, but otherwise re-arm via the same shared mechanism, for
the same reason -- without it, is_collecting stays false alongside
is_initialized false and the filter can never re-run auto-init on its
own. Of the three only the quality exit carries the IMU biases across
(REQ-NAV-061); this one and the time-jump reset shall not, which is why
the shared re-arm clears any pending carry.

## REQ-NAV-043 — Beginner-friendly hard-gate option defaults

- **Status:** implemented
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_beginner_option_defaults

A zero-initialised ins_options_t (with only the desired feature flags set)
shall yield a working filter. For the option fields that act as *hard
gates* -- max_prediction_time_sec, the four GNSS fusion thresholds
(gnss_max_horizontal_pos_stddev_m, gnss_max_vertical_pos_stddev_m,
gnss_max_horizontal_vel_stddev_mps, gnss_max_vertical_vel_stddev_mps) and
the two mode-transition threshold sets with their dwells
(gnss_start_max_*, gnss_stop_max_*, gnss_init_dwell_sec,
gnss_stop_dwell_sec; REQ-NAV-051, REQ-NAV-052), plus the fusion rate limit
(gnss_min_delay_ms; REQ-NAV-074) -- a
value of 0 is not a benign "no opinion" but a filter-killing setting:
max_prediction_time_sec 0 turns every forward epoch into a time-jump reset,
a 0 GNSS fusion threshold rejects every fix, a 0 entry threshold never
admits the 3D solution and a 0 exit threshold leaves it on the first fix.
ins_init shall therefore resolve each of these fields, when left at 0, to a
beginner-friendly default (0.5 s; fusion 120 m / 120 m horizontal/vertical
position and 60 m/s / 60 m/s horizontal/vertical velocity; entry 2 m /
3 m and 0.25 m/s / 0.30 m/s; exit 5 m / 7 m and 0.4 m/s / 0.5 m/s) by
writing the resolved value back into the
stored options, while leaving any explicitly supplied non-zero value untouched.
This complements the conservative noise defaults of REQ-NAV-017 (which cover
the process-noise knobs) for the gating knobs.

The fusion thresholds default to the accuracy caps of REQ-NAV-071 rather
than to a tight quality gate, so the unconfigured filter downweights a bad
fix instead of discarding it (REQ-NAV-007). The mode-transition sets keep
their own, much tighter defaults: whether a fix is worth fusing and
whether the fix stream is good enough to run a 3D solution on remain
separate questions, and only the second one is answered restrictively by
default.

The same resolution applies to the conditioning knobs that are not hard
gates but whose "no opinion" value is likewise not the intended one: the
four GNSS covariance floors (opt.gnss_pos_stddev_floor_hor_m / _ver_m,
opt.gnss_vel_stddev_floor_hor_mps / _ver_mps; 0.5 m / 1.0 m and
0.3 m/s / 0.5 m/s) and the four caps of REQ-NAV-071 (120 m / 120 m and
60 m/s / 60 m/s), the accuracy-envelope time constant
(gnss_acc_envelope_tau_sec; 5 s, REQ-NAV-072) and the two
manoeuvre-dependent velocity-noise scales
(gnss_vel_noise_acc_scale_hor / _ver; 0.20 and 0.30, REQ-NAV-073) with
their averaging window (gnss_vel_noise_acc_window_sec; 200 ms,
REQ-NAV-075). For these twelve fields a NEGATIVE value is the explicit
opt-out and shall be resolved to "disabled" rather than to the default, so
the behaviour predating each of them stays reachable from configuration.

## REQ-NAV-049 — Beginner-friendly initial-state and process-noise defaults

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_beginner_init_defaults

ins_init shall resolve, when left at (or explicitly set to) <= 0, each of
init.pos_init_stddev_m, vel_init_stddev_mps, rpy_init_stddev_rad[0..2],
acc_bias_init_stddev_mps2, gyr_bias_init_stddev_rps, pos_pred_stddev_m_sqrts,
vel_pred_stddev_mps_sqrts, rpy_pred_stddev_rad_sqrts, zero_vel_stddev_mps and
zero_rot_stddev_rps to a beginner-friendly, generously conservative default
(10 m; 1 m/s; 5 deg per axis; 0.03 m/s^2; 1 deg/s; 0.01 m/sqrt(s);
0.0005 (m/s)/sqrt(s); 0.001 rad/sqrt(s); 0.05 m/s; 0.5 deg/s respectively --
the three *_pred_stddev_* entries are spectral densities, not plain rates, see
ins.h), writing the resolved value back into the stored init struct while
leaving any explicitly supplied positive value untouched. ins_fuse()'s own
rejection of an invalid (R <= 0) measurement variance stays in place regardless
as a second line of defense (e.g. against zero_vel_stddev_mps/zero_rot_stddev_rps
being overwritten back to 0 after init).

Rationale: unlike rpy_init_stddev_rad[2] (yaw, independently settable from
roll/pitch, see REQ-NAV-026) and the mag-bias/bias-random-walk fields (already
defaulted, REQ-NAV-017), these fields had no such treatment. For the
*_init_stddev_*/ *_pred_stddev_* state-covariance fields, a caller who left one
at 0 did not get "very confident", they got a state whose Kalman gain is locked
at exactly 0 forever (in exact arithmetic, P=0 in P H^T (H P H^T + R)^-1 is
always 0, independent of R), with a 0 process-noise stddev compounding it by
also preventing that variance from ever growing during prediction -- no
ZUPT/ZARU trigger, no absolute-position fusion, nothing can ever correct such a
state again. For zero_vel_stddev_mps/zero_rot_stddev_rps -- the ZUPT/ZARU
measurement noise, not a state covariance -- 0 is worse than inert: it is an
invalid measurement variance ins_fuse() rejects outright (n_fuse_fail), so the
auto-detector (REQ-NAV-013) can correctly decide the platform is stationary and
still never actually fuse a correction. Complements REQ-NAV-043 (ins_options_t's
hard gates) and REQ-NAV-017 (IMU noise densities and bias random walks) for the
remaining ins_init_t fields those two do not cover.

rpy_pred_stddev_rad_sqrts is sized for a MEMS IMU on a non-rigid mount, where
the attitude model error (gyro scale-factor and cross-axis error under large
oscillating rates) is orders of magnitude above the sensor's random walk. At
the sensor figure the filter is falsely confident in attitude, so
absolute-position innovations get no gain onto roll/pitch/yaw and the tilt
error is absorbed by the accelerometer bias instead. Raising it further buys
little and costs both accuracy and a believable reported attitude uncertainty.
A rigid mount needs less and sets it explicitly, as every dataset under
datasets/ does.

## REQ-NAV-044 — Auto-init heading bootstrap tolerates non-concurrent magnetometer

- **Status:** implemented
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_autoinit_mag_yaw_cached

The magnetometer-derived heading of the auto-init bootstrap (REQ-NAV-015,
priority 2) shall not require a magnetometer sample in the same
ins_update call as the bootstrapping position fix. The GNSS/position
aiding and the magnetometer generally run on independent sample clocks
(and may be delivered in separate update calls), so a fix rarely
coincides with a mag sample; requiring coincidence would drop the filter
to the "unknown yaw" fallback (yaw 0, ~180 deg stddev) on most real
streams, from which the chi2-downweighted running mag fusion recovers only
slowly. The filter shall therefore cache the most recent valid
magnetometer sample seen while uninitialized and, when the bootstrap epoch
carries no concurrent mag, use that cached sample for the tilt-compensated
heading provided it is no older than opt.max_prediction_time_sec (the
static gate that admits the bootstrap already guarantees the platform is
near-stationary, so a heading that recent still holds). A staler cache, or
none at all, falls back to "unknown" as before. The cache is cleared on
ins_init and on an autonomous re-acquisition (REQ-NAV-042) so a
pre-failure heading is never reused. Rationale: co-timestamped
GNSS+mag+IMU is an artifact of some replay converters, not how real
sensors arrive; without this the auto-init heading silently depends on
sample-timing coincidence.

## REQ-NAV-045 — GNSS-stability dwell before entering 3D

- **Status:** implemented
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_init_dwell

Entry into the 3D (position/velocity) solution — the auto-init cold-start
bootstrap AND every re-arm that routes back through it (REQ-NAV-042,
REQ-NAV-016, REQ-NAV-052) — shall be
held off until the position aiding has been **continuously of entry
quality for at least opt.gnss_init_dwell_sec** (0 -> default 5 s) **and has
delivered at least 1 such fix per second over that window**. "Entry
quality" is the dedicated covariance gate of REQ-NAV-051, not the (looser)
fusion gate, and no separate stability/jump test. A fix that fails that
gate, a gap longer than the max-gap tolerance between good fixes, or a
backwards timestamp resets the dwell run; the run is also reset by every
re-arm (REQ-NAV-042, REQ-NAV-016, REQ-NAV-052) so a prior stream cannot
be credited. This is a hysteresis (dwell/debounce) on
the ins-ready transition: a lone marginal or intermittently-good fix can no
longer flap 3D on and off, which matters most in flight after a GNSS
dropout, where a single recovered fix should not immediately re-enter a
solution that then drops again. With opt.gnss_init_dwell_disable set (or the
dwell resolved to a value the stream trivially meets), entry is immediate as
before — appropriate for controlled starts (e.g. indoor/local-position rigs)
that want instant 3D. The dwell governs only the autonomous auto-init/
re-acquire path; a manual prescribed-value init is the caller's explicit
assertion and is not gated. As a deliberate side effect the dwell also gives
the heading bootstrap (REQ-NAV-044) time to cache a magnetometer sample
before it commits, closing the last window where a fix arriving before any
mag would bootstrap yaw as "unknown".

## REQ-NAV-046 — Configurable global chi2 downweight significance level

- **Status:** implemented
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_chi2_reject_alpha

The chi2 innovation gate that ins_fuse() uses to decide when an absolute
reference is an outlier and has its measurement noise inflated (downweighted,
not dropped) shall be tunable through a single global significance level
opt.chi2_reject_alpha, shared by every absolute-reference channel (GNSS
position/velocity, magnetometer, local-position and the yaw residual incl.
automotive_mode course-as-yaw) -- there shall be no differing per-channel
default gates. Because each reference is fused scalar-row-wise (1 degree of
freedom), a configured alpha shall resolve, in ins_init, to the single
scalar gate threshold = chi2inv(1 - alpha, 1) applied to all channels;
chi2inv(·, 1) is evaluated by linear interpolation of a fixed quantile table
(bounded, WCET-safe), alpha clamped to the table's supported range. A value of
0 (unset) shall resolve to ins.c's own single default INS_DEFAULT_CHI2_GATE,
applied identically to all four channels -- NOT
sensor_defaults.h's stricter INS_DEFAULT_CHI2_95_1DOF (3.8415) that ahrs.c
and baro_alt.c default to. The resolved thresholds are stored
on the filter (f->chi2_thr_gnss / _mag / _local / _yaw) for the fusion
sites to read -- always numerically identical to one another. alpha is
the significance level, not the threshold itself: a LARGER alpha (e.g.
0.2 -> chi2inv(0.8,1) = 1.64) yields a SMALLER threshold, tripping on
smaller residuals -- stricter, more downweighting. A smaller alpha (e.g.
0.01 -> chi2inv(0.99,1) = 6.63) yields a larger threshold -- more
tolerant, less downweighting. This is orthogonal to chi2_disable: that override
forces the gate off for every channel and takes precedence, whereas this
level only retunes where the (active) gate sits. Zero-velocity/zero-rotation
updates remain ungated (chi2_threshold == 0.0f, as before).

## REQ-NAV-047 — Auto-init roll/pitch bootstrap under motion

- **Status:** verified
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_autoinit_moving_rpy_stddev

Auto-init's accelerometer leveling (REQ-NAV-015) shall not require the
platform to be quasi-static: a leveling window whose peak gyro rate
exceeds opt.auto_init_static_gyr_rps, or whose median specific-force
magnitude deviates from gravity by more than opt.auto_init_static_acc_mps2,
shall still be allowed to bootstrap the filter (rationale: a platform that
never sits still -- e.g. a boat or a taxiing aircraft -- must still be able
to auto-init; deferring indefinitely is not an acceptable substitute for a
platform-specific static window). Instead, whenever that quasi-static
classification does not hold, the reported initial roll and pitch stddev
(each resolved independently from init.rpy_init_stddev_rad[0]/[1]) shall
each be at least max(init.rpy_init_stddev_rad[0 or 1], opt.auto_init_moving_rpy_stddev_rad)
(0 -> default 15 deg), i.e. it shall never be smaller than that floor and
shall never fall below the caller-configured static-case baseline either.
While quasi-static, the roll/pitch stddev is exactly
init.rpy_init_stddev_rad[0]/[1] respectively, unaffected by this requirement.
Each bootstrap under motion shall increment
ins_diag_t.n_autoinit_moving. Rationale: the leveling equation assumes the
specific force is gravity alone; under motion that assumption is violated,
so reporting the same tight stddev as the static case would leave the
filter overconfident in an attitude the running fusion then has no reason
to correct quickly.

## REQ-NAV-048 — External attitude/gyro-bias hint for auto-init and re-acquisition

- **Status:** verified
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_autoinit_att_hint; Test: tests/test_ins_core.c:scenario_reacquire_att_hint

ins_measurements_t shall carry an optional, source-agnostic
roll/pitch(/yaw)/gyro-bias hint (att_hint) that auto-init
(REQ-NAV-015) and re-acquisition (REQ-NAV-023) shall use in place of
their own accelerometer leveling / kept dead-reckoned attitude,
whenever the hint is valid:

- Auto-init: when att_hint carries a usable roll/pitch (both stddevs
  > 0), it replaces the window's own accelerometer-leveling result;
  the combined roll/pitch variance is
  max(stddev_roll_rad, stddev_pitch_rad)^2. Its yaw (stddev_yaw_rad >
  0) is a new priority tier in the yaw resolution chain, ranked
  between the external yaw measurement (REQ-NAV-044) and the
  magnetometer.
- Re-acquisition: when att_hint carries a usable roll/pitch, it
  overrides the otherwise-kept dead-reckoned attitude (REQ-NAV-023);
  gyro bias is overridden per axis wherever
  stddev_gyr_bias_rps[axis] > 0, independent of the roll/pitch/yaw
  path.
- Absent or invalid (is_valid == false, or a stddev <= 0 for the
  field in question), each path shall behave exactly as it did before
  this hint existed -- the hint is strictly additive.

Rationale: a continuously-running attitude-only filter fed the same
IMU can have integrated far more history than a single leveling
window, and survive an outage ins itself cannot dead-reckon through
cleanly (an uncompensated gyro bias that is unobservable from GNSS
position/velocity alone -- e.g. no antenna lever arm configured --
otherwise free-integrates into the attitude with nothing to correct
it once auto-ZUPT's own staticness gate is fooled by the resulting
gravity leak into the velocity states; found on a stationary bench
dataset where roll/pitch/yaw diverged to 50+ degrees over well under a
minute this way). ins.c has no dependency on any particular source
filter; nav_suite wires its ARS/AHRS into att_hint (REQ-SUITE-016).

## REQ-NAV-050 — Initial bias-prior consistency check

- **Status:** verified
- **Parent:** REQ-SYS-016
- **Verification:** Test: tests/test_ins_core.c:scenario_bias_prior_check

While the automatic ZUPT/ZARU detector (REQ-NAV-013) reports stillness,
ins shall average the RAW accelerometer and gyroscope samples and, once
that averaging window covers both a minimum duration and a minimum
number of samples, hold the averages against the initial bias 1-sigma
priors the filter was configured with (init.acc_bias_init_stddev_mps2,
init.gyr_bias_init_stddev_rps):

- accelerometer: | ||mean f|| - g | shall be compared against
  3 * acc_bias_init_stddev_mps2,
- gyroscope: ||mean omega|| shall be compared against 3 * max(
  gyr_bias_init_stddev_rps, earth rate), the floor being needed because
  a stationary gyroscope reads earth rate and a prior below that could
  never be met,

and each threshold that is exceeded shall increment its own monotonic
diagnostic counter (n_acc_bias_prior_exceeded / n_gyr_bias_prior_exceeded)
and emit a throttled LOG_WARN. The averaging window shall be restarted
after every evaluation and discarded whenever stillness ends.

The check shall be purely diagnostic: it shall never alter the filter
state, the covariance or any fusion decision.

Rationale: at a standstill the averaged raw IMU is essentially the
sensor bias itself, so it is direct evidence about a prior that is
otherwise never validated. A prior far too tight for the actual
hardware does not fail outright, it just costs a needlessly slow and
hard-to-diagnose init transient, typically after a configuration was
copied from a different IMU class. The raw (not bias-corrected) sample
is used deliberately: the converged bias estimate would hide exactly
the mismatch being looked for. Averaging over a long window is what
keeps white sensor noise from dominating the comparison. The
accelerometer side only observes the bias component along gravity (the
norm is attitude-invariant, which is what stops a leveling error from
masquerading as a bias, but it also makes a purely horizontal bias
invisible), so a hit is evidence while a miss is not an all-clear. The
stillness gate of REQ-NAV-013 bounds the samples that can reach the
check at all, so a prior looser than that gate divided by the factor
above can never be contradicted by standstill data -- the harmless
direction, since a prior that generous is not the mistuning in
question.

## REQ-NAV-051 — GNSS entry-quality gate for the 3D solution

- **Status:** verified
- **Parent:** REQ-NAV-045
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_mode_hysteresis

Entry into the 3D (position/velocity) solution shall be gated on its own
GNSS quality thresholds, independent of the fusion gate of REQ-NAV-007:

- opt.gnss_start_max_horizontal_pos_stddev_m,
- opt.gnss_start_max_vertical_pos_stddev_m,
- opt.gnss_start_max_horizontal_vel_stddev_mps,
- opt.gnss_start_max_vertical_vel_stddev_mps,

each 0 -> a built-in default (REQ-NAV-043). An epoch counts as entry
quality when it carries a GNSS position fix whose per-axis 1-sigma is
within the two position thresholds AND, if the epoch also carries a GNSS
velocity, whose per-axis velocity 1-sigma is within the two velocity
thresholds. A position-only receiver shall therefore still be able to
start; local-position aiding (no comparable covariance) shall count as
entry quality unchanged. This gate governs the auto-init bootstrap and
therefore, since all of them route back through it, every re-arm as well
(REQ-NAV-042, REQ-NAV-016, REQ-NAV-052), and feeds the dwell run of
REQ-NAV-045; a
manual prescribed-value init stays the caller's explicit assertion and is
not gated.

ins_init shall clamp each entry threshold to at most the corresponding
fusion threshold (REQ-NAV-007) and emit a warning when it does: entering
the 3D solution on a fix the filter would then refuse to fuse is never the
intent.

Rationale: the accuracy that justifies *starting* a 3D solution is not the
accuracy that justifies fusing a single measurement into a solution that
already exists and can outvote it. The velocity threshold carries most of
the decision: the vertical channel, the accelerometer bias states and the
yaw observability of a moving platform all lean on the GNSS velocity, so a
default of 0.25 m/s (1-sigma) is what a healthy multi-GNSS receiver
delivers, while a bootstrap on a several-tenths-of-a-m/s velocity solution
starts the filter with an attitude and bias state that has to be unlearned
later.

## REQ-NAV-052 — GNSS quality-loss exit from the 3D solution

- **Status:** verified
- **Parent:** REQ-NAV-021
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_mode_hysteresis

A running 3D solution shall be left (ins_is_ready() -> false) once the GNSS
aiding has failed a third, loosest threshold set

- opt.gnss_stop_max_horizontal_pos_stddev_m,
- opt.gnss_stop_max_vertical_pos_stddev_m,
- opt.gnss_stop_max_horizontal_vel_stddev_mps,
- opt.gnss_stop_max_vertical_vel_stddev_mps

continuously for longer than opt.gnss_stop_dwell_sec (each 0 -> a built-in
default, REQ-NAV-043). An epoch fails the set when any channel it actually
offers (position, velocity) exceeds its threshold; an epoch carrying
local-position aiding never fails it. Only epochs that offer aiding shall
be judged, so a plain outage remains governed by REQ-NAV-022
(max_deadreckoning_sec) alone, and a run of bad fixes shall not be broken
by the gaps between them.

On the exit the filter shall stop, not merely stop reporting readiness: it
shall route through the shared re-arm of REQ-NAV-042 (is_initialized false,
is_collecting true), so strapdown, prediction and every fusion cease and
each position/velocity/attitude accessor reports "unavailable" instead of a
value that keeps moving off a fix stream already judged too poor. The
height-source latch (REQ-NAV-053) and the delayed measurement history shall
be re-established by the next bootstrap; the IMU biases (REQ-NAV-061) and
the n-frame origin (REQ-NAV-062) are carried across, so position, velocity
and attitude are the estimates the bootstrap re-derives. The exit shall be
counted in the diagnostics (n_gnss_quality_exit, not n_health_reset) and
logged.

The re-arm shall be gated exactly like REQ-NAV-042
(!opt.auto_reacquire_disable && opt.auto_init). With that gate closed there
is no autonomous way back, so the exit shall instead fall back to the
previous hold-down: the filter keeps running and keeps fusing, and only
ins_is_ready() reports false until the entry gate of REQ-NAV-051 is met
again. The same reasoning as REQ-NAV-042 applies, more strongly: a
quality exit happens mid-mission, so a re-armed manual init would re-apply
a prescribed origin and attitude stamped at mission start.

Return into the 3D solution shall require the entry gate and dwell of
REQ-NAV-051/REQ-NAV-045, never a single recovered fix. It is now the plain
auto-init bootstrap that enforces this, so readiness shall additionally wait
out the standard post-bootstrap warm-up (a minimum runtime and a minimum
number of Kalman epochs) rather than returning on the dwell alone.
ins_init shall clamp each exit threshold to at least the corresponding entry
threshold and warn when it does, so the two can never invert.
opt.gnss_stop_disable shall switch the exit off entirely (the pre-REQ-NAV-052
behaviour), leaving the entry gate in force.

A filter whose coasting window has already expired (REQ-NAV-022) may still
quality-exit and re-arm: the two conditions describe different failures and
the re-arm is the stronger recovery of the two.

Rationale: together with REQ-NAV-051 this is a hysteresis on the solution
mode -- a strict, short-dwell entry against a loose, long-dwell exit.
Without the loose exit threshold the filter would drop the solution on the
first degraded epoch of an urban canyon; without the long dwell it would
flap between modes for as long as the reception stays marginal. The
asymmetry encodes what the two decisions actually cost: a premature entry
starts a solution from a poor state, while a premature exit throws away a
converged one. Tearing the instance down rather than holding it is the
honest reading of the exit condition: aiding that stayed below the loosest
gate for the whole dwell has left nothing converged worth keeping, and a
position that keeps moving off it is worse than no position at all -- a
consumer cannot tell the two apart, whereas an absent value is
unambiguous. Sustained bad aiding is a different failure from no aiding at
all, which is why this does not simply reuse the dead-reckoning timeout:
the filter is being fed, it is being fed measurements too poor to keep a 3D
solution honest.

## REQ-NAV-053 — Height-source selection at bootstrap (barometric vs GNSS)

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_source_selection; Test: tests/test_ins_core.c:scenario_baro_height_stale_sample

At the same bootstrap event already gated by auto-init (REQ-NAV-015) and
the GNSS entry-quality dwell (REQ-NAV-045/051) -- cold start and the
post-health-fail/time-jump/quality-exit re-arm alike (REQ-NAV-042,
REQ-NAV-016, REQ-NAV-052) -- the
filter shall decide once, for the remaining lifetime of that filter
instance, whether its vertical position channel is driven by barometric
height (REQ-NAV-054) or by GNSS position (REQ-NAV-005): for a GNSS
bootstrap, a barometer that is streaming at bootstrap selects barometric
height, its absence selects GNSS. A local-position bootstrap
(lighthouse/UWB/mocap, REQ-NAV-011) shall never select barometric height
regardless of barometer presence: such a system typically supplies a
vertical reference more accurate than a barometer, and this requirement
exists specifically for GNSS's comparatively poor vertical accuracy
(REQ-NAV-051's rationale), not as a general "prefer barometer when
available" policy.

Unlike the GNSS entry-quality gate, detection is not a stddev-graded
quality dwell -- a barometer sample carries no reported covariance, so
there is nothing to grade -- but it is not bare presence either: the
most recent plausible sample cached during the collecting window shall
also be no older than a fixed maximum age at the bootstrap epoch, and a
sample that fails that age check shall select GNSS height and be
reported. Two independent reasons, both of which bite in the same
situation (a barometer that delivered early in the collecting window and
then stopped, while the GNSS entry dwell held the bootstrap off for
another half minute, REQ-NAV-045): the decision is never re-evaluated,
so a single stale sample would otherwise latch the filter instance into
a height source it has no live sensor for; and that same pressure
becomes the datum anchor of REQ-NAV-054, so a stale one offsets the
whole height channel by the height change since it was taken.

Since a re-arm ends the filter instance, the decision is genuinely taken
again afterwards, against a barometer cache the re-arm has just cleared.
A barometer that has since stopped delivering therefore flips the instance
from barometric to GNSS height, which is correct but easy to miss -- hence
the requirement that the selection is reported at every bootstrap, not
only the first.

The caller shall additionally be able to suppress the barometric height
source outright (option `baro_height_disable`), keeping GNSS position's
vertical row whatever the barometer does. This is for a GNSS-labelled
input that is not satellite GNSS and already reports a
better-than-barometric vertical accuracy -- a precise indoor tracking
system fed through the gnss_pos/gnss_vel interface for convenience --
i.e. the same situation the local-position exception above covers, for
callers who do not use the local-position interface.

The decision shall not be re-evaluated after bootstrap: a barometer that
later stops delivering samples is an ordinary aiding gap on the height
measurement (REQ-NAV-054), reported as such (REQ-NAV-058) but not a
trigger to fall back to GNSS height.

Rationale: a decision re-evaluated per epoch would reintroduce exactly
the discrete source switch (and the unbounded step it can produce
between two independently-drifting height estimates, REQ-SYS-011) that
fusing barometric height directly into ins is meant to avoid
(REQ-NAV-054). Fixing the choice once, at the same moment the rest of
the state bootstraps, keeps the vertical channel's behaviour as
simple to reason about as the horizontal one.

## REQ-NAV-054 — Barometric height fusion (ins-internal)

- **Status:** verified
- **Parent:** REQ-NAV-053
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_fusion; Test: tests/test_ins_core.c:scenario_baro_height_survives_reacquire

When REQ-NAV-053 selects barometric height, the filter shall fuse the
datum-referenced barometric altitude directly as a scalar measurement of
the vertical position state (down axis), using the same ISA conversion
and start anchor h0 as baro_alt (REQ-BARO-004), established once at
bootstrap and not re-derived afterwards. The fusion shall use the same
robust downweighting policy as every other persistent absolute reference
(REQ-SYS-006): outliers downweighted, not dropped, consistent with
baro_alt's own barometer fusion (REQ-BARO-005). This measurement
replaces, not supplements, the vertical row of the GNSS position
measurement (REQ-NAV-005, REQ-NAV-055) for the lifetime of the filter
instance -- the two are never simultaneous alternatives fused into the
same state on the same epoch.

Like every other fusion, it runs on live epochs only: while the coasting
window is expired the filter is inert and fuses nothing (REQ-NAV-064).
Vertical continuity across such an outage is provided at the far end
instead, by re-anchoring the height on the barometer at re-acquisition
(REQ-NAV-066); within nav_suite the vertical solution during the outage
is baro_alt's (REQ-SUITE-006), which is what the wrapper reports for as
long as ins is not ready.

Rationale: see the design-decision discussion in the architecture
documentation (\S "Design decisions", barometric height fused directly
in ins). In short: a discrete switch between independently-drifting
height estimates is an unbounded discontinuity by construction, and
splicing a separately-estimated vertical state onto ins's own
horizontal state discards the cross-axis covariance a single error-state
filter maintains. Fusing barometric height as just another scalar
measurement of the existing position state avoids both: no switch
(the source is fixed at bootstrap, REQ-NAV-053), no splice (it stays
inside the one state/covariance ins already keeps).

## REQ-NAV-055 — GNSS position fusion restricted to horizontal under barometric height

- **Status:** verified
- **Parent:** REQ-NAV-053
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_fusion

When REQ-NAV-053 selects barometric height, GNSS position fusion
(REQ-NAV-005) shall be restricted to its horizontal (N, E) rows: the
vertical row shall be dropped from that measurement, not downweighted,
since barometric height (REQ-NAV-054) already supplies the vertical
row for that epoch and the two are not simultaneous alternatives. GNSS
velocity fusion (REQ-NAV-006), including its vertical component, is
unaffected by the height-source selection and shall continue to be
fused in full (all three axes) regardless of which height source is
active -- discarding an independent, generally reliable vertical
velocity observation would be wasteful and REQ-NAV-054 only concerns
position, not velocity.

Without a barometer (REQ-NAV-053 selects GNSS), GNSS position fusion
shall remain the full 3x3 measurement of REQ-NAV-005, unchanged from
today's behaviour.

## REQ-NAV-056 — Covariance propagation continues while dead reckoning is frozen

- **Status:** deleted
- **Parent:** REQ-SYS-010
- **Verification:** Inspection: obsolete -- the frozen filter no longer fuses anything, so there is no measurement left for an un-propagated covariance to be shrunk by (REQ-NAV-064).

Superseded by REQ-NAV-064. This requirement kept the Kalman time update
running while the nominal state was frozen, because the fusion block
kept running too. Once a filter with an expired coasting window is
inert -- no strapdown, no time update, no fusion -- the reason to
propagate a covariance nobody may use disappears with it: the
confidence lost over the outage is instead priced into the states that
are actually carried across it, once, at re-acquisition (REQ-NAV-065).
Kept as a deleted ID for stability; never reuse.

The failure that motivated it is retained here, because it is what
REQ-NAV-064 exists to make impossible rather than to compensate for: a
state that is not being propagated but is still fused shrinks its own
variance against zero process noise, epoch after epoch, until the
Kalman gain reaches zero and the channel stops responding to its own
measurement. With the barometric height source (REQ-NAV-054) the
vertical position variance collapsed to centimetres over a few minutes
of outage and the height then lagged the barometer it is fused from by
tens of metres. Every measurement then also trips the chi2 gate
(REQ-NAV-036) against its own implausibly small innovation covariance,
and the overconfident variance is carried into the re-acquisition
(REQ-NAV-023), where it makes the fresh velocity absorb the accumulated
position error as a large spurious velocity step. The overconfidence
watchdog (REQ-NAV-040) does not cover this: its floors are set to catch
numerical covariance collapse (0.1 mm), orders of magnitude below the
physically-plausible-but-badly-wrong variance this produces.

## REQ-NAV-057 — Position-aiding quality gates ignore the unused vertical row

- **Status:** verified
- **Parent:** REQ-NAV-055
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_vertical_gate_ignored

While the barometric height source is active (REQ-NAV-053 has latched
it), the vertical component of a GNSS position measurement's reported
covariance shall not be graded by the position quality gates: neither
the fusion gate (REQ-NAV-007) nor the 3D exit gate (REQ-NAV-052) shall
reject a fix, and neither shall drop the 3D solution, on the strength
of a vertical position accuracy whose measurement row is dropped before
fusion anyway (REQ-NAV-055). The horizontal limits apply unchanged, and
the velocity gates -- horizontal and vertical alike -- apply unchanged,
since GNSS velocity continues to be fused on all three axes
(REQ-NAV-055).

The entry gate (REQ-NAV-051) shall keep applying its configured
vertical position limit: it is evaluated during a cold bootstrap, when
the height source has not been latched yet, and the bootstrap fix
defines the n-frame origin, so its vertical accuracy is the accuracy of
the absolute vertical datum no matter which height source runs on top
of it afterwards (REQ-NAV-053, REQ-SYS-012).

Rationale: a good HDOP with a poor VDOP is an everyday satellite
geometry. Vetoing the whole fix over it costs the horizontal solution
its aiding, and at the exit gate costs the entire 3D solution, for a
channel the filter is not using -- while REQ-NAV-053's stated purpose
is that only the horizontal solution degrades once the height is
barometric. Since REQ-NAV-052's exit now tears the filter instance
down rather than holding it, the cost of a wrong drop is a full
re-bootstrap, which makes this exemption load-bearing rather than a
refinement.

## REQ-NAV-058 — Barometric height aiding-gap diagnostic

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_gap_warning

While the barometric height source is active (REQ-NAV-053) the filter
shall count the epochs in which barometric height was actually fused
(diagnostics: n_baro_height_used) and shall report, throttled, when no
such fusion has happened for longer than a fixed gap -- a barometer
that stopped delivering, or one whose samples are all implausible.

This is a diagnostic only and explicitly not a fallback trigger:
REQ-NAV-053 fixes the height source for the filter instance's lifetime,
and re-evaluating it would reintroduce the discrete source switch that
fusing barometric height directly exists to avoid.

Rationale: this is the one failure the latched selection cannot absorb
on its own. The vertical position keeps its (honestly growing) variance
but has no absolute reference left, while GNSS position aiding can
continue undisturbed -- so ins_is_ready() stays true, the solution mode
stays FULL, the health check (REQ-NAV-031) passes and the
overconfidence watchdog (REQ-NAV-040) has nothing to say. Without this
report nothing in the system states that the height channel has lost
its only absolute reference.

## REQ-NAV-059 — Yaw prior reset at re-acquisition without a heading source

- **Status:** verified
- **Parent:** REQ-SYS-010
- **Verification:** Test: tests/test_ins_core.c:scenario_reacquire_yaw_unknown

At re-acquisition after an expired coasting window (REQ-NAV-023), the
filter shall reset the yaw variance to an unknown-heading prior
(~180 degrees 1-sigma, the same one the bootstrap uses with no heading
source, REQ-NAV-015) unless the epoch's attitude hint carries an
absolute yaw (REQ-NAV-048, stddev_yaw_rad > 0), in which case the
hinted yaw and its variance stand. The reset shall never LOWER the yaw
variance, and shall leave the yaw state itself untouched: what is
wrong is the confidence, not the value, and there is nothing better to
replace the value with.

Rationale: while the coasting window is expired the nominal state is
frozen (REQ-NAV-022), so the platform's own rotation over the outage
is missing from the yaw estimate entirely -- a vehicle through a
curved tunnel re-acquires with an arbitrary heading error. The yaw
variance meanwhile is inflated for the outage (REQ-NAV-065), but only
at the gyro-noise rate, which models the drift of a running strapdown
and not the unmodelled motion of a frozen one, it therefore still
claims a fraction of a degree.

That inconsistency does not stay in the yaw state. Re-acquisition
seeds the gyro bias from the external hint with a deliberately
inflated variance (REQ-NAV-048, and nav_suite inflates it again per
REQ-SUITE-016), so a yaw variance still claiming sub-degree accuracy
is the tighter of the two and the aiding that follows routes the whole
accumulated heading correction into the gyro z bias instead of into
yaw. The bias then has to unwind over minutes and trips the
bias-runaway watchdog on the way. Observed on a car dataset with a
94 s GNSS outage: the gyro z bias spiked to 2.1 deg/s and needed over
two minutes to return, while the parallel ARS -- which never sees
GNSS -- held 0.13 deg/s throughout.

## REQ-NAV-060 — Heading input range validation and normalization

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_ins_core.c:scenario_yaw_input_range

ins_update shall accept absolute heading inputs (the yaw measurement
and the attitude hint's yaw, REQ-NAV-010 and REQ-NAV-048) in any
single-turn convention and normalize them to [-pi, pi] before use. A
heading whose magnitude exceeds one full turn shall be rejected, not
wrapped: the yaw measurement block is invalidated, the attitude hint
keeps its roll/pitch/gyro-bias content but loses its yaw
(stddev_yaw_rad set to 0, the documented "no yaw hint" encoding), and
each rejection increments ins_diag_t.n_invalid_input.

Rationale: callers publish headings as [-pi, pi] or as [0, 2pi), and
both must work. Beyond one turn the value is not a convention
difference but a unit or unwrapping error -- degrees written into a
radian field, or an accumulated course that was never wrapped -- and
wrapping it silently would yield a plausible-looking but wrong
heading. Rejecting is the safer failure: a dropped heading leaves the
estimate where it was, whereas a wrong one is consumed unfiltered by
the bootstrap (REQ-NAV-044), which builds the nominal quaternion from
it with no residual and no chi2 gate, and in the fusion path it is
merely downweighted (REQ-SYS-006) rather than skipped, so yaw aiding
would go silently ineffective while the aiding watchdog still counts
it as fed.

The internal residual wrap is likewise not allowed to depend on the
sanitizer's invariant: the yaw residual shall be formed with a wrap
that is correct for arbitrary finite inputs (ins_angle_diff).

## REQ-NAV-061 — IMU bias carry-over across a quality-loss re-arm

- **Status:** verified
- **Parent:** REQ-NAV-052
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_quality_exit_bias_carry

The re-arm of REQ-NAV-052, and only that re-arm, shall carry the
accelerometer and gyroscope bias estimates plus their current 1-sigma into
the next bootstrap, which shall seed the nominal biases from the carried
values instead of the configured init.acc_bias_init_mps2 /
init.gyr_bias_init_rps. The seeded 1-sigma shall be the 1-sigma at the exit
epoch widened by a fixed inflation factor and then clamped to at most the
corresponding cold-start prior (init.acc_bias_init_stddev_mps2,
init.gyr_bias_init_stddev_rps): a carry shall never be seeded more
confidently than a cold start.

The other two mid-run "is_initialized = false" events shall not carry a
bias: a health-check shutdown (REQ-NAV-042) has just declared the state
untrustworthy, and a forced time-jump reset (REQ-NAV-016) says nothing
about the sensors. The shared re-arm shall therefore clear any pending
carry, so neither event can inherit one, and the carry shall be cleared
again once consumed.

An external gyro-bias hint (REQ-NAV-048, e.g. nav_suite's ARS/AHRS via
REQ-SUITE-016) applied right after the bootstrap keeps precedence and
overrides the carried gyro bias per axis. The accelerometer bias, which no
parallel filter estimates, is where the carry-over does the real work.

Rationale: the accelerometer bias is the slowest-converging state in the
filter -- it needs minutes of aided motion -- and it is a property of the
sensor, not of the fix stream that just degraded. Discarding it on every
quality exit would make each restart re-learn a quantity that never became
invalid, and the vertical channel and the attitude would carry that
re-learning as an error the whole time. The clamp is what makes this safe
without a decay timer: however stale the carry becomes during a long
attitude-only window, the seeded uncertainty is bounded by the cold-start
prior, so the worst case is exactly a cold start and never worse. This is
also why no ageing is specified -- the covariance does not grow during the
window (no prediction runs), and a decay constant would be a knob with no
data behind it.

## REQ-NAV-062 — n-frame origin carry-over across a quality-loss re-arm

- **Status:** verified
- **Parent:** REQ-NAV-052
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_quality_exit_origin_carry

The re-arm of REQ-NAV-052, and only that re-arm, shall carry the n-frame
origin into the next bootstrap: that bootstrap shall keep the inherited
origin instead of adopting its own fix as the origin, and derive the
initial pos_local from the fix, using the same geodetic mapping as the
re-acquisition of REQ-NAV-023 (a latitude/longitude/height difference, not a
flat-earth ECEF delta, which at the distances reachable during an outage
would put a kilometre-scale offset hundreds of metres off in height).

The re-arm shall therefore carry the exiting instance's position as a PAIR --
its pos_local and the same position as an absolute latitude/longitude/height
anchor -- and the bootstrap shall map the fix as a step from that carried
position. Taking the difference against the origin instead would send a
baseline of unbounded length through a mapping that is only valid for a short
one, with the error and the consequences REQ-NAV-023 describes.

The absolute solution shall be identical either way -- latitude, longitude,
height and ECEF are anchored to the bootstrap fix in both cases, and the
GNSS residual is formed against the current absolute position, not against
the origin, so no fusion, covariance or readiness behaviour depends on this
choice. This identity holds at any distance from the origin: it is the
bootstrap fix that anchors the absolute solution, never the fix mapped
into the inherited frame and back out of it. What the carry preserves is
the meaning of the local NED frame across the outage: pos_local, the
local-frame accessors and the WGS84 <-> local conversions built on the
origin (REQ-SUITE-014) stay continuous.

Anything anchored to "pos_local = 0" at bootstrap shall be anchored to the
bootstrap's actual pos_local instead, so that the identity above holds. The
barometric height datum of REQ-NAV-054 is the one such anchor: it defines
where pos_local[2] = 0 sits on the pressure scale, and a carried origin
bootstraps at a nonzero pos_local[2] whenever the platform reappears at a
different altitude. Left anchored at zero it would offset the entire height
channel by that altitude difference.

The carry shall be refused, and the bootstrap fix shall define a fresh
origin as before, when

- no origin has been established yet (any bootstrap after ins_init),
- the inherited origin is not a finite, plausible geodetic position, or
- the bootstrap fix cannot be the same platform continuing: it lies further
  from the position the exiting instance last held than
  INS_ORIGIN_CARRY_MAX_SPEED_MPS could have covered in the time since that
  instance's last position aiding, plus a floor of
  INS_ORIGIN_CARRY_MIN_TRAVEL_M.

The reachability test shall be measured from the last known POSITION and
priced by the time since the last POSITION AIDING, not by the time since the
exit: the platform has been unobserved since aiding stopped, and on the case
this exists for -- a tunnel -- that is the whole passage, while the exit only
fires once fixes come back and would price a 100 s blackout as the few
seconds the re-arm took. The floor covers the bootstrap fix's own error plus
however far the frozen dead-reckoned position had drifted from the truth
before the filter gave up.

The other two mid-run "is_initialized = false" events shall not carry an
origin: a health-check shutdown (REQ-NAV-042) has just declared the state
untrustworthy, and a forced time-jump reset (REQ-NAV-016) cannot tell a
long outage apart from a different mission or log file, where the old
origin would be meaningless. The shared re-arm shall therefore clear any
pending carry, so neither event can inherit one, and the carry shall be
cleared again once consumed -- the same lifetime as the IMU bias carry of
REQ-NAV-061.

Rationale: the local n-frame is not an internal detail, it is the frame a
controller flies waypoints in and the frame every local-position consumer
speaks. Redefining its origin mid-mission silently relocates every
previously issued local coordinate on the Earth while the absolute outputs
stay correct, so nothing downstream can detect it. The tunnel case makes
the inconsistency plain: whether the same outage ends in a re-acquisition
(REQ-NAV-023, origin kept) or in a quality-loss re-arm depends only on how
poor the fixes during the outage were, and that should not decide which
local frame the consumer gets. The wrapper's vertical datum arbitration
(REQ-SUITE-007) is the other half of the same statement and is keyed to
this one: it re-aligns a newly anchored origin onto the surviving
barometric datum (REQ-SUITE-006, REQ-NAV-025), but leaves a carried origin
alone, which is already the datum. Without that pairing the horizontal
origin would survive a quality exit while its height was pulled onto
whatever the barometer happened to read at the re-bootstrap.

The refusal test deliberately does NOT bound the distance to the origin.
That distance is only "how far has this mission travelled since it
started": it grows without limit on any long trip, and a trip that reaches
the same distance WITHOUT a re-arm keeps its origin regardless, because the
origin is never re-based horizontally. Bounding it here would therefore
make the survival of the local frame depend on how far from home the outage
happened -- a 105 km drive with a tunnel near the end would lose the frame
that the identical drive without the tunnel keeps -- which is the very
inconsistency this requirement exists to remove. It costs real accuracy
too: a fresh origin discards a converged frame in exchange for nothing the
consumer asked for.

What the test is actually for is telling a platform that kept going apart
from a different mission or a spliced log, and reachability answers that
directly and stays correct at any mission length. The speed bound is set
beyond any vehicle this library targets (car, drone, light aircraft), so it
never refuses a real continuation, while a jump to another region during a
short blackout cannot pass it.

The degradations that do grow with distance to the origin -- pos_local's
float32 resolution, and the tilt between the origin's local level and the
current one (as d/R_earth), since the n-frame axes track the current
position rather than the origin and are refreshed every
INS_EARTH_REFRESH_DIST_M of travel -- are real but are not introduced here:
a long enough mission reaches them with or without a re-arm. They are a
property of a far-travelled local frame, not of inheriting one.

## REQ-NAV-063 — GNSS position decimation against an unknown position/velocity correlation

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_pos_decimation

When an epoch offers a usable GNSS position AND a usable GNSS velocity,
the filter shall fuse only one of the two, selected by a configurable
decimation factor N (opt.gnss_pos_decimation, 0 -> built-in default):
every Nth such epoch fuses the position alone, the remaining N-1 fuse the
velocity alone. N <= 1 disables the decimation and restores the combined
position+velocity fuse. An epoch that offers only one of the two is never
affected: the decimation removes information, it never adds any.

The built-in default shall be N = 1, i.e. OFF. The fusion rate limit of
REQ-NAV-074 addresses the same over-fusing of temporally correlated
measurements, and it does so without ever withholding a block: running
both by default would thin the position twice, once against a correlation
in time and once against a correlation between blocks. Decimation stays
available for a receiver whose position error stays correlated well
beyond the interval the rate limit already spaces fixes out to.

The decimation shall NOT apply while the coasting window has expired
(REQ-NAV-023): there the position is the whole point of the measurement,
and withholding it would leave the frozen position un-anchored while the
velocity-only fuse refreshed the aiding timestamp, so the re-acquisition
would never run at all.

The cycle shall restart at a position whenever the position aiding stream
was interrupted for longer than the fix-stream continuity bound
(INS_GNSS_INIT_MAX_GAP_SEC). After a gap the dead-reckoned position is
the stale quantity, and serving out the remainder of a velocity-only run
before the first position would prolong exactly the error the fix came
back to correct.

An epoch whose position is withheld by the decimation shall still count
as position aiding for the coasting window (REQ-NAV-022, REQ-NAV-023):
the fix carried a usable position, the filter declined to fuse it. Only a
position that fails the fusion gates (REQ-NAV-005) or is absent is a
position-aiding gap. Without this the decimation would trip the coasting
window at 1 Hz for any N above the window length in fixes.

Rationale: a GNSS receiver's reported position and velocity come out of
one internally coupled solution, but the cross-covariance between them is
not part of any common output message (ins_measurements_t carries
gnss_Qll_pos_vel_ned for receivers that do report it, and it is zero
otherwise). Fusing both blocks every epoch as if they were independent
double-counts the same information and drives the covariance below what
the measurements support, which then shrinks the Kalman gain of every
other aiding source and biases the accelerometer/attitude states that
absorb the difference.

Decimation is deliberately the blunt instrument: it makes no assumption
about the correlation it cannot observe, unlike an assumed
cross-correlation coefficient, which is unverifiable against a receiver
that does not report one and drives R towards singularity when guessed
too high. It also attacks the second, larger correlation in the same
data: a receiver's position error is dominated by multipath, residual
ionospheric delay and its own internal smoothing, all correlated over
minutes, while the Doppler-derived velocity is comparatively white. The
velocity is therefore the measurement worth having at full rate, and the
position the one worth thinning.

The proper cure for the time-correlated part is a Gauss-Markov position
bias state per axis, which would let the position be fused at full rate.
That is a larger change to the state vector and is not what this
requirement specifies.

## REQ-NAV-064 — A filter with an expired coasting window is inert

- **Status:** verified
- **Parent:** REQ-SYS-010
- **Verification:** Test: tests/test_ins_core.c:scenario_frozen_no_fusion; Test: tests/test_ins_core.c:scenario_deadreckoning_freeze; Test: tests/test_ins_core.c:scenario_coasting_window_still_fuses; Test: tools/replay.c:main

While the coasting window is expired (REQ-NAV-022) the filter shall run
neither the strapdown, nor the Kalman time update, nor ANY measurement
fusion. Neither the nominal state nor the covariance shall change on
such an epoch. Measurements shall be consumed only for the purposes
that survive the freeze:

- deciding whether this epoch re-acquires (REQ-NAV-023), which is the
  one operation that may write state again,
- caching the latest barometric sample the re-acquisition needs
  (REQ-NAV-066),
- book-keeping that carries no filter state: diagnostics, the aiding
  gates and mode arbitration (REQ-NAV-051, REQ-NAV-052), stillness
  detection, and the epoch timestamps the filter re-baselines on so a
  long outage does not surface as one giant prediction gap on the far
  side.

Not applicable with allow_unlimited_deadreckoning, where the window
never expires.

End-to-end coverage on data with an independent truth comes from the
replay of datasets/simulated/B_drone/config_coasting.yaml, whose
coasting window is deliberately shorter than that flight's GNSS outage
(REQ-VER-016).

Rationale: ins_is_ready() is already false for the whole of this period
(REQ-NAV-022), so there is no consumer entitled to a solution the
filter would be computing under the hood -- within nav_suite the
vertical channel is served by baro_alt (REQ-SUITE-006) and attitude by
the ARS/AHRS, both of which run on the same sensors and are not frozen.
Keeping part of the filter running is worse than either extreme: a
covariance that is propagated but never corrected, or one that is
corrected but never propagated, both drift away from anything the
states mean. The second of those was a real defect, documented in the
deleted REQ-NAV-056: with the barometric height source still fusing
against an unpropagated covariance, the vertical variance collapsed to
centimetres and the height fell tens of metres behind the barometer it
was fused from. Stopping everything removes the coupling that made that
possible, and the confidence genuinely lost over the outage is priced
in once, at re-acquisition (REQ-NAV-065), where the elapsed time is
known exactly.

## REQ-NAV-065 — Re-acquisition inflates the states carried across the freeze

- **Status:** verified
- **Parent:** REQ-NAV-023
- **Verification:** Test: tests/test_ins_core.c:scenario_reacquire_variance_inflation

At re-acquisition (REQ-NAV-023) the filter shall increase the variance
of every state it carries across the freeze -- attitude, accelerometer
bias, gyroscope bias, and the magnetometer bias states of the 18-state
mode -- by that state's process-noise PSD multiplied by the frozen
duration, using the same PSDs the Kalman time update would have used.
The result shall be clamped to the state's configured initial 1-sigma
prior: a filter re-acquiring after an arbitrarily long outage is at
worst as uncertain as one starting cold, never worse.

The frozen duration is the position-aiding outage minus the coasting
window: the coasted part of the outage was propagated normally at the
time, and would otherwise be counted twice.

The inflation shall never LOWER a variance, and shall be applied before
the external attitude/gyro-bias hint (REQ-NAV-048) and before the yaw
prior reset (REQ-NAV-059), so that a hinted state keeps the hint's own
variance and the yaw prior still governs the heading.

Rationale: this is what pays for REQ-NAV-064. The states kept across an
outage did not become more accurate by standing still, and re-acquiring
with pre-outage confidence would make the fresh, wide position and
velocity variances of the re-anchor the loosest states in the filter,
so the aiding that follows routes its corrections into the attitude and
bias states instead -- the mechanism REQ-NAV-059 documents for yaw, and
which applies to the biases just as well. Doing it once with the known
elapsed time is both cheaper and more honest than the epoch-by-epoch
propagation it replaces: over a freeze the propagation had no live
specific-force sample to build a meaningful state-transition matrix
from anyway, so it was already reduced to the same random-walk terms
this computes directly.

## REQ-NAV-066 — Vertical re-anchor from the cached barometric sample

- **Status:** verified
- **Parent:** REQ-NAV-023
- **Verification:** Test: tests/test_ins_core.c:scenario_baro_height_reanchor_after_long_outage; Test: tests/test_ins_core.c:scenario_baro_height_survives_reacquire

While the barometric height source is active (REQ-NAV-053), the
re-acquisition of REQ-NAV-023 shall set the vertical position from the
most recent plausible barometric sample instead of from the position
fix, using the same datum-referenced ISA conversion the fusion uses
(REQ-NAV-054, the h0 anchor of REQ-NAV-053), and shall set the vertical
position variance from that sample's reported accuracy. The filter
shall therefore cache the latest plausible barometric sample on every
epoch, including the epochs on which it is otherwise inert
(REQ-NAV-064).

If no cached sample is available, or the cached one is older than the
same freshness bound that governs anchoring the height channel at
bootstrap, the vertical position and its variance shall be re-anchored
from the position fix like the horizontal ones. A barometer that has
stopped delivering must not anchor a height on a pressure reading of
unknown age.

This applies to both re-acquisition paths, GNSS and local position
(REQ-NAV-011): the height source is a property of the filter instance,
not of the aiding type that ends the outage.

Rationale: the platform's height at a tunnel exit is a quantity the
barometer knows and the frozen state does not, and taking it from the
barometer rather than the fix is what makes the vertical solution
continuous across the re-acquisition -- the GNSS vertical row is not
fused under this height source at all (REQ-NAV-055), so re-anchoring on
it would inject exactly the discontinuity between two independently
drifting height estimates that REQ-NAV-053 exists to avoid. The raw
sample is used rather than a filtered height because it is the same
conversion the vertical state was tracking before the freeze, so the
re-anchor carries no filter lag of its own; its single-sample noise is
honestly reflected in the variance it is given.

## REQ-NAV-067 — A gyro-bias hint never loosens the cold-start prior

- **Status:** verified
- **Parent:** REQ-NAV-048
- **Verification:** Test: tests/test_ins_core.c:scenario_att_hint_gyr_bias_prior_cap

When an external gyro-bias hint (REQ-NAV-048) re-seeds a gyro-bias state
AT THE AUTO-INIT BOOTSTRAP, the variance it installs shall be the hinted
variance clamped, per axis, to at most

    max(init.gyr_bias_init_stddev_rps, the library's default cold-start
        gyro-bias prior)^2

The hinted bias VALUE is adopted either way, only the confidence is
bounded. A hint at or below that bound shall be installed unchanged, so
the clamp is invisible to every well-scaled hint.

The floor at the library default is what keeps the clamp from doing harm
of its own: a configuration whose gyro-bias prior is tighter than a
plausible MEMS cold start would otherwise have the hint clamped to that
same over-tight value, over-constraining a bias state the hint was
supposed to help.

Re-acquisition (REQ-NAV-023) shall NOT clamp. There the hint competes
with a state the filter has genuinely converged rather than with a
starting assumption, and installing the hinted value at a confidence the
hint never claimed would discard that convergence.

Rationale: a hint is information, so consuming it must never leave the
filter more uncertain than ignoring it would have. The supplier does not
know the consumer's prior and can legitimately widen its own covariance
before handing it over -- nav_suite inflates the ARS/AHRS 1-sigma by a
fixed factor precisely so ins does not over-trust a parallel filter
(REQ-SUITE-016) -- and an ARS's gyro-z bias is the worst case for that:
without an absolute heading it is structurally unobservable, so the
1-sigma the ARS reports for it is its own init prior carried forward, not
an estimate, and the inflation is applied to a non-observation. Without
the clamp that product arrives as the bootstrap prior: observed at
14.8 deg/s against a configured 0.5 deg/s, i.e. 30x looser, which is not
a plausible prior for any MEMS gyro (the filter's own runaway watchdog
calls a bias implausible at 10 deg/s).

That matters because of what shares the bootstrap with it. With no
heading source the yaw prior is "unknown" (REQ-NAV-059), and yaw and
gyro-z bias are not separable over the first seconds: how the initial
heading correction splits between them is decided by their relative
priors alone. A bias prior that loose makes the bias absorb it, and the
gyro-z estimate leaves the alignment tens of deg/s wrong. On full-weight
innovations that is recoverable and unwinds within seconds. Under the
chi2 downweight gate (REQ-NAV-035) it is not: the Chang downweight bounds
the applied correction at roughly P*H'*threshold/innovation, which SHRINKS
as the innovation grows, so once the state error has outgrown its own
1-sigma the very corrections that would unwind the bias are the ones the
gate suppresses, and a larger error buys a smaller correction. The gate
cannot tell "this measurement is an outlier" from "my state is wrong",
and here it is the second.

This is the mirror image of the clamp REQ-NAV-061 puts on a CARRIED bias
and closes the same gap from the other side: a carry may not be seeded
more confidently than a cold start, a bootstrap hint may not be seeded
less confidently than one.

## REQ-NAV-068 — Absolute speed aiding

- **Status:** verified
- **Parent:** REQ-NAV-015
- **Verification:** Test: tests/test_ins_core.c:scenario_speed_aiding

ins_measurements_t shall accept an optional scalar ground-speed
measurement (speed), i.e. the MAGNITUDE of the platform velocity with no
direction attached, and fuse it as a single-row measurement of the
velocity states:

    h(x) = ||v_n||        H = [ 0 | v_hat^T | 0 | 0 | 0 ],  v_hat = v_n / ||v_n||

The residual shall be formed from the nonlinear h() evaluated on the
NOMINAL state, h(v_nominal) - z, with H the Jacobian with respect to the
error state -- the same error-state pattern every other nonlinear
reference in this filter uses (REQ-NAV-002), so no linearization of the
measurement itself is needed anywhere.

The measurement shall be treated as unsigned: a negative speed is a unit
or sign error, not a reversing platform, and shall be rejected and
counted in ins_diag_t.n_invalid_input like any other implausible input.

**Delay.** speed_delay_ms shall anchor the residual in the state history
exactly as gnss_delay_ms does (REQ-NAV-008): the predicted speed is taken
from the historical velocity at the time of validity, not from the
current one. A link with tens of ms of latency (a Bluetooth OBD-II
dongle round-trip) is the normal case for this input, not the exception.

**Standstill gate.** Below a configurable minimum FILTERED speed
(opt.speed_min_mps, 0 -> default) the measurement shall be skipped: v_hat
is undefined at v_n = 0 and numerically meaningless just above it, so
there is no direction along which the correction could be applied. The
gate is on the filter's own speed rather than on the reported one on
purpose -- the quantity that has to be well-conditioned is the Jacobian,
which is built from the state. A platform that is genuinely stopped is
covered by the automatic ZUPT (REQ-NAV-013) instead; a filter sitting
near zero velocity while the platform actually moves is a divergence that
absolute-speed aiding cannot resolve in any case, because a magnitude
carries no direction to correct towards.

**Scale calibration.** opt.speed_scale (0 -> 1.0) shall multiply the
reported speed before use, and opt.speed_stddev_rel (0 -> default) shall
contribute a speed-proportional variance term, so the fused noise is

    R = stddev_mps^2 + (speed_stddev_rel * z)^2

Rationale for the two-part noise model: for this class of sensor the
per-sample noise and the systematic scale error are different orders of
magnitude and scale differently. An OBD-II PID 0x0D reading is quantized
to 1 km/h, i.e. a 0.080 m/s uniform-quantization 1-sigma that does not
grow with speed; the scale error of the vehicle's own speed signal
(rolling radius, tyre wear, the scaling the ECU applies) is a
multiplicative error of a few percent, reaching 0.83 m/s at 100 km/h at
the 3% default. Its sign is not predictable: the type-approval margin
that keeps an indicated speed from ever falling below the true one
constrains the dashboard, while an OBD-II PID 0x0D reading is the ECU's
own value. Fusing the raw value against the per-sample noise alone
would present a
systematic bias as if it were an independent measurement and pull the
velocity states permanently off. The scale factor removes the calibrated
part, the relative variance carries what is left.

No scale-factor STATE is estimated: the constant plus the relative
variance keeps this a pure measurement-boundary feature with no change to
the runtime-selected state size (REQ-NAV-001).

Rationale: an absolute speed reference bounds exactly the failure a
GNSS-denied stretch produces first. Dead-reckoned velocity error grows
without bound as attitude and accelerometer-bias errors leak through the
strapdown, and it is that velocity error, integrated, which becomes the
position error. One scalar per second is enough to stop the growth,
without the two-state cloning a delta-position (wheel-tick) formulation
would need. It observes only the component of the velocity error along
the current direction of travel; cross-track error is untouched, which is
the honest limit of a measurement that carries no direction.

## REQ-NAV-069 — Public predict/correct API

- **Status:** verified
- **Parent:** REQ-SYS-017
- **Verification:** Test: tests/test_ins_core.c:scenario_predict_correct_equivalence

ins_predict_step() and ins_correct_step() shall together be exactly
equivalent to ins_update(): ins_update() is implemented as
ins_predict_step() followed by ins_correct_step(), with no bookkeeping
(sanitizing, time-jump handling, dead-reckoning freeze, history save,
health checks) left behind in ins_update() itself that a caller driving
the two halves separately would miss. ins_predict_step() shall accept
an optional phi_out buffer for the n x n (n = ins_get_num_states())
state transition matrix used that call, filled only when the covariance
was actually propagated (see INS_EPOCH_COV_PROPAGATED).

The sanitized measurement and the epoch's time-jump/coasting decision
shall be handed off internally from ins_predict_step() to
ins_correct_step() (not re-derived, since sanitizing has diagnostic side
effects and the timing decision is not recoverable once
t_last_kalman_predict has moved on) -- ins_correct_step() shall be a
no-op if the matching ins_predict_step() dropped the epoch or was never
called.

## REQ-NAV-070 — Recovery from a restarted time source

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_time_source_restart_recovers

A running filter shall recover autonomously when its timestamp source
restarts, i.e. when the epoch timestamps jump backwards by more than the
tolerated measurement delay and then continue at the normal rate in the
new timebase. Dropping such epochs individually (REQ-NAV-016) is correct
for isolated reordering but not for a restart: the internal time-jump
reference stays in the abandoned timebase, every following epoch measures
the same large negative delta, and the filter would remain permanently
inert while still being fed valid data. The filter shall therefore count
consecutive drop-as-too-old epochs and, once the run reaches
INS_TIME_RESTART_EPOCHS, treat the discontinuity as a source restart:
force the same defined reset as a forward time jump and, unless
opt.auto_reacquire_disable is set and provided auto_init, re-arm into the
collecting state (REQ-NAV-042) so the next coherent window in the new
timebase re-initializes the filter. The run counter shall be cleared by
any epoch that is not dropped as too old, so that isolated reordering
never accumulates into a reset. Unlike a forward jump, the reset shall
not be suppressed by allow_unlimited_deadreckoning: coasting cannot
resolve a backwards discontinuity, since the abandoned timebase is never
reached again. The reset shall be counted in the diagnostics
(n_time_restart_reset).

## REQ-NAV-071 — GNSS accuracy caps

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_cov_cap

The covariance conditioning of REQ-NAV-038 shall, as its last
receiver-accuracy step and after the floor, clamp each conditioned
per-axis standard deviation to at most a configured cap, with separate
horizontal (N,E) and vertical (D) caps for the position and the velocity
block (opt.gnss_pos_stddev_cap_hor_m / _ver_m,
opt.gnss_vel_stddev_cap_hor_mps / _ver_mps). Unlike the floor, the cap
cannot act on the diagonal variances alone and leave the off-diagonal
terms untouched: shrinking a diagonal entry while its correlation terms
stay at their original, uncapped scale can push the conditioned matrix
out of positive-semidefiniteness, which is a real occurrence for a
strongly correlated fix (for example right after a GNSS outage, when the
reacquired fix reports a huge and highly correlated covariance). The cap
is therefore applied as the same kind of correlation-preserving
congruence Q' = S Q S used by the scale of REQ-NAV-038 and the height
downweight of REQ-NAV-041, with S = diag(s_N, s_E, s_D) holding one
shrink factor per axis (1.0 for an axis already at or below its cap), so
the reported correlation coefficients are preserved and the matrix stays
positive-semidefinite. A cap of 0 shall resolve to the built-in default,
a negative cap shall disable the cap for that axis group, and ins_init
shall raise any cap that ends up below the floor of the same axis group
to that floor, so a mis-ordered configuration cannot invert the two
clamps.

Rationale: the reported accuracy of a degraded fix is unbounded above,
and a covariance of thousands of metres squared entering the fusion is
numerically harmful in the 32-bit hot path long before it is
informationally harmful. Capping it keeps the fix contributing what
little information it carries, at a weight that cannot destabilise the
solution, and is what makes the loosened fusion gate of REQ-NAV-007 safe:
without the cap, defaulting that gate to 120 m would simply push the same
unbounded values into the update instead of rejecting them.

The cap bounds the receiver-reported term only. The manoeuvre-dependent
velocity term of REQ-NAV-073 is added after it and may exceed it, since
it describes the platform's dynamics rather than the receiver's health.

## REQ-NAV-072 — Asymmetric tracking of the reported GNSS accuracy

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_acc_envelope

The filter shall track the reported GNSS accuracy asymmetrically over
time and weight the fusion with the tracked envelope rather than with the
instantaneous report: a rise in the reported standard deviation shall
take effect immediately and in full, while a fall shall only take effect
with a first-order decay of time constant opt.gnss_acc_envelope_tau_sec.

Per block (position, velocity) and axis group (horizontal N/E, vertical
D) the filter shall keep an envelope e, updated once per epoch that
offers that block:

    alpha = clamp(dt / tau, 0, 1)
    e     = max(stddev_reported, e * (1 - alpha))

with dt the elapsed time since the previous epoch that offered the same
block, and stddev_reported the larger of the two horizontal per-axis
standard deviations for the horizontal group. The envelope shall enter
the conditioning of REQ-NAV-038 as its first step, applied as the
correlation-preserving congruence Q' = S Q S with
S = diag(r_hor, r_hor, r_ver) and r = e / stddev_reported >= 1, so the
correlation structure and positive-semidefiniteness of the reported
covariance are preserved. A tau of 0 shall resolve to the built-in
default, a negative tau shall disable the envelope (r = 1).

Rationale: a receiver's accuracy report is trustworthy when it worsens
and optimistic when it recovers. The moment a receiver re-acquires
satellites after an obstruction its reported accuracy drops back to its
nominal value, while the fix itself is still settling -- multipath, a
partially converged ambiguity or a stale ionospheric correction do not
disappear the instant the accuracy figure does. Believing the recovery
immediately is what produces the position step the filter then has to
absorb. Believing the degradation immediately costs nothing.

Like every other part of the conditioning (REQ-NAV-038), the envelope
weights the FUSION only: the fix-quality gates keep grading the accuracy
the receiver reported for this epoch, so a single bad epoch cannot hold
the filter out of the 3D solution for a whole time constant. An outage
long enough for alpha to reach 1 leaves the envelope at the next reported
value, so no explicit reset across a GNSS gap is needed.

## REQ-NAV-073 — Manoeuvre-dependent GNSS velocity noise

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_vel_noise_acc

The filter shall add an independent, manoeuvre-dependent variance to the
diagonal of the conditioned GNSS VELOCITY covariance, proportional to the
squared magnitude of the current acceleration of the GNSS ANTENNA in the
navigation frame with gravity removed:

    sigma_extra_hor = opt.gnss_vel_noise_acc_scale_hor * |a_NE|
    sigma_extra_ver = opt.gnss_vel_noise_acc_scale_ver * |a_D|

where a is the navigation-frame acceleration of the antenna
(REQ-NAV-076, the platform's acceleration plus the centripetal term of
the antenna lever arm) averaged over the measurement window
(REQ-NAV-075), |a_NE| the norm of its horizontal (North, East)
components and |a_D| the magnitude of its Down component. sigma_extra_hor^2 shall be
added to the N and E diagonal entries and sigma_extra_ver^2 to the D
entry, after the floor and the cap and without touching the off-diagonal
terms. A scale of 0 shall resolve to the built-in default and a negative
scale shall disable the term for that axis group, the same convention the
floors and caps of REQ-NAV-038/071 use.

The default shall be the half measurement interval, T/2 with T = 400 ms,
i.e. 0.20 for the horizontal axes, with a further margin on the vertical
axis (0.30) for the one-sided satellite geometry that makes the vertical
velocity the worse-conditioned of the two. T is taken at four times the
interval REQ-NAV-074 paces fusion to, as deliberate margin: T belongs to
the receiver's velocity algorithm, not to its output rate, and the filter
cannot observe it. The scales shall be fixed constants and shall NOT be
derived from the observed fix rate at runtime -- the half-interval is how
the numbers were chosen, not a quantity the filter recomputes, so the
measurement noise stays independent of a fix stream that may itself be
irregular.

The defaults are sized for a platform that reaches large accelerations
from a standstill: at a 45 degree lean they add roughly 2 m/s, more than
six times the horizontal velocity floor, while at rest they add nothing
and leave the standstill regime (REQ-NAV-013 and its gates) untouched.
They shall not be sized much more generously than this, for two reasons
that both bite at large scales: the term reads the instantaneous
acceleration, so airframe vibration enters at scale times its amplitude,
and horizontal acceleration is what makes heading observable in a
GNSS-aided inertial filter, so inflating the velocity noise in proportion
to it trades directly against the observability that same acceleration
provides.

A caller whose receiver derives velocity from time-differenced carrier
phase over a long output interval shall raise the scales, T then being
the full output interval rather than a short Doppler window. A source
that reports an instantaneous velocity with no measurement interval at
all (a pose tracker, a simulator) shall disable the term.

The scales are stated in the standard-deviation domain, i.e. in
(m/s) of extra velocity noise per (m/s^2) of acceleration, so their
magnitude is directly readable as "how much velocity noise does one g of
manoeuvre buy".

Rationale: a Doppler-derived GNSS velocity is the average over the
receiver's measurement interval, while the filter fuses it against an
instantaneous state. Under constant velocity the two agree; under
acceleration they differ by roughly a * T/2, a systematic error the
reported accuracy does not include because it is a property of the
platform, not of the signal. Inflating the velocity noise with the
measured acceleration keeps the filter from reading that timing mismatch
as an innovation and correcting the state and the accelerometer bias for
it.

The term requires a valid navigation-frame acceleration. Before the
strapdown has produced one it shall be zero, so a filter that has not yet
initialised is unaffected.

## REQ-NAV-074 — GNSS fusion rate limit

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_rate_limit

The filter shall not fuse two GNSS epochs closer together in time than
opt.gnss_min_delay_ms. A usable fix arriving sooner than that after the
last fix the filter actually spent shall be skipped whole -- both the
position and the velocity block, unlike the decimation of REQ-NAV-063,
which withholds one of the two -- and counted in
ins_diag_t.n_gnss_rate_limited. A value of 0 shall resolve to the
built-in default (100 ms, i.e. 10 Hz), a negative value shall disable
the limit, the same convention opt.magnetometer_min_delay_ms uses.

The limit shall be suspended while the coasting window has expired
(REQ-NAV-023): the next usable fix is then a re-acquisition anchor rather
than an update, and the timing of a tunnel exit is not the filter's to
pace.

A fix skipped by the limit shall still count as position aiding for the
coasting window, on the same grounds as REQ-NAV-063: the fix carried a
usable position, the filter declined to spend it. Only a position that
fails the fusion gates or is absent is a position-aiding gap.

Rationale: a GNSS error does not decorrelate anywhere near as fast as a
modern receiver can emit fixes. Multipath, residual ionospheric delay and
the receiver's own internal smoothing persist over many seconds, so
consecutive fixes are largely repeated evidence rather than new evidence.
Fusing at 20 Hz therefore does not present twice the information of
10 Hz, it presents the same information twice, and a Kalman filter has no
way to notice: it shrinks its covariance as if each fix were independent,
until the reported uncertainty no longer bounds the true error and the
gain is too small for the fix stream to correct what it caused. This is
the temporal counterpart of the block correlation REQ-NAV-063 addresses.

Ten hertz is well past the rate at which a healthy fix stream bounds an
inertial solution's drift -- the inertial solution is what carries the
state between fixes, and it does so for far longer than 100 ms -- so the
default costs no accuracy that a real installation depends on, while
bounding the cost of a receiver configured for a high output rate.

## REQ-NAV-075 — Averaging window for the manoeuvre-dependent velocity noise

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_manoeuvre_window

The acceleration that the manoeuvre-dependent GNSS velocity noise of
REQ-NAV-073 is a function of shall be the navigation-frame acceleration
averaged over opt.gnss_vel_noise_acc_window_sec, not the instantaneous
value of the epoch in which a fix happens to be fused. The average shall
be taken over the acceleration VECTOR and the per-axis-group magnitudes
formed from the averaged vector, never the other way round. A window of 0
shall resolve to the built-in default (200 ms, the same T the scales of
REQ-NAV-073 are half of) and a negative window shall select the
instantaneous sample.

The rotation half of the antenna acceleration (REQ-NAV-076) shall be
averaged over the same window and with the same first-order rule, as the
outer product omega*omega' rather than as omega.

The average shall be maintained as a first-order (exponential) average
updated once per strapdown step, so its cost is three floats rather than a
ring buffer at IMU rate, and it shall be invalidated wherever the
strapdown's own acceleration is (filter reset, re-arm), so a restarted
filter does not price a manoeuvre from before the discontinuity.

Rationale: the error being priced is what averaging over an interval T
does to a velocity, so the acceleration over that same interval is the
quantity that sets it. Two failures follow from reading the instantaneous
value instead, and they point in opposite directions.

A manoeuvre that has just ended goes unpriced on the very fix that still
carries its error: the fix was measured while the platform was
accelerating, and it is anchored at a time of validity up to the
measurement delay in the past, but the acceleration read at the fusion
epoch has already returned to zero. The tail of the average is what
carries the finished manoeuvre across that gap.

Conversely, a zero-mean oscillation is priced as though it were a
manoeuvre. Airframe vibration on a multirotor swings the instantaneous
acceleration by several m/s^2 about a mean of zero, and a fix landing in
one half of the cycle is then charged for an acceleration the platform
never had. Averaging the vector cancels it exactly, which averaging the
magnitude would not: the mean of a magnitude is positive whatever the mean
of the signal. The measured effect is large - at a scale of 0.5 and a
4 m/s^2 oscillation, the instantaneous form inflates the velocity variance
by 4 (m/s)^2 while the windowed form adds 0.01.

The vector mean is also the physically exact quantity rather than a
convenient approximation: the difference between an interval-averaged
velocity and the instantaneous velocity at the end of the interval is
governed by the MEAN acceleration over it.

## REQ-NAV-076 — Antenna acceleration for the manoeuvre-dependent velocity noise

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_vel_noise_leverarm

The acceleration that the manoeuvre-dependent GNSS velocity noise of
REQ-NAV-073 is a function of shall be the acceleration of the GNSS
ANTENNA, that is the platform's navigation-frame acceleration plus the
centripetal acceleration the antenna lever arm adds under rotation:

    a_ant = a_body + R_b_to_n * (omega_b_nb x (omega_b_nb x l_b))
          = a_body + R_b_to_n * (omega*omega' - |omega|^2 I) l_b

with l_b the GNSS antenna lever arm the caller supplies with the fix and
omega the bias-corrected rotation rate the strapdown computed. The
resulting vector shall enter REQ-NAV-073 in place of a_body, so it is
split into the horizontal and vertical axis groups the same way and needs
no scale of its own.

The operator (omega*omega' - |omega|^2 I) shall be carried as the windowed
average of the outer product omega*omega' (REQ-NAV-075 window, six floats
for a symmetric 3x3), and the lever arm applied to the averaged operator
at the fix epoch, since the lever arm is not known while the window is
being filled. The averaged quantity shall be the outer product and NOT
omega itself. The rotation into the navigation frame shall use the current
attitude.

The tangential part, alpha x l, shall NOT be included.

With a zero lever arm the addition shall be identically zero, so an
installation whose antenna sits at the IMU is unaffected. Disabling
REQ-NAV-073 for an axis group shall also disable this contribution to it,
the term having no separate switch.

Rationale: what a Doppler receiver averaged over its measurement interval
is the motion of the antenna phase centre, not of the IMU. The two differ
whenever the platform rotates with a lever arm, and they differ most
sharply in the case the body acceleration cannot see at all: a platform
turning about its own IMU registers no acceleration there while the
antenna swings through an arc. The filter compensates the lever-arm
velocity itself (omega x l), so what is left to price is the same a*T/2
mismatch REQ-NAV-073 already prices, evaluated at the point the
measurement actually describes.

The centripetal term is the dominant one by a wide margin. For a lever arm
of 0.3 m at a yaw rate of 90 deg/s it reaches 0.74 m/s^2, which the default
horizontal scale turns into 0.07 m/s of extra noise, while the other error
sources of a rotating lever arm stay near a hundredth of that: 8 mm/s for
a 1 degree attitude error, 7 mm/s for 10 ms of residual latency, 4 mm/s
for 0.5 deg/s of gyro bias. Pricing those separately would add parameters
for effects the velocity floor already covers.

Averaging the outer product rather than omega is what makes the term
survive a platform that turns back and forth. omega*omega' is quadratic,
so its mean stays positive for an oscillation whose own mean is zero,
which is the honest answer here and the opposite of the answer
REQ-NAV-075 wants for a vibrating accelerometer: a zero-mean linear
vibration moves the antenna nowhere on average, a zero-mean yaw
oscillation swings it through an arc on every half cycle.

The tangential term is dropped for two reasons that agree. It would need
the gyro differentiated, which converts gyro noise into noise on the
velocity covariance at IMU rate, and it is zero exactly where the
centripetal term is largest, in the steady turn a platform spends most of
its rotating time in.

This term trades against attitude observability the same way the
acceleration term of REQ-NAV-073 trades against heading observability, and
for the same structural reason: the lever-arm velocity is the mechanism
through which a rotating antenna makes the attitude observable, so
inflating the velocity noise in proportion to the rotation weakens the
aiding that rotation provides. At the default scales the trade is mild (a
0.3 m arm at 90 deg/s costs 0.07 m/s), and it is not sized more generously
than that for this reason.

## REQ-NAV-077 — Non-holonomic lateral velocity constraint

- **Status:** verified
- **Parent:** REQ-NAV-034
- **Verification:** Test: tests/test_ins_core.c:scenario_nhc_lateral

Under `opt.automotive_lateral_constraint` the filter shall fuse a synthetic
measurement stating that the LATERAL component of the body-frame velocity is
zero, as the residual `(R_b_to_n^T * v_ned)_y` against a truth of zero, with
the measurement noise `opt.automotive_lateral_stddev_mps`. The option shall
default to off and shall require `opt.automotive_mode`.

The measurement matrix carries an attitude block, and that block is the point
of the constraint rather than a side effect: with the psi-angle convention of
this filter the residual perturbs as

    dv_b = R^T dv + R^T [v_n x] psi

so the row seen by the error state is `(R^T)_y` on the velocity states and
`(R^T [v_n x])_y` on the attitude states, whose gain is the ground speed. At
20 m/s one degree of roll error shows up as 0.35 m/s of lateral body
velocity. During a GNSS outage the lateral channel is otherwise unobserved
and a roll error leaks gravity sideways.

Only the lateral row is fused.

Fusions shall be rate limited to INS_NHC_MIN_INTERVAL_SEC. The residual is
dominated by a slowly varying offset with a measured correlation time of
88 s (0.91 at 1 s, 0.79 at 8 s), so a coast of a hundred seconds contains on
the order of one independent sample and a faster rate would add confidence
without adding information.

The constraint shall be gated on ground speed (`opt.automotive_min_speed_mps`,
shared with REQ-NAV-034) and on yaw rate
(`opt.automotive_lateral_max_yaw_rate`). Unlike the course-over-ground yaw
aiding of REQ-NAV-034 the constraint stays valid in reverse, where the course
flips by 180 degrees and the lateral velocity does not.

`opt.automotive_lateral_after_sec` shall hold the constraint off until the
filter has been without GNSS fusion for that long (negative -> no delay).
Next to a receiver delivering velocity at 5 Hz and 0.05 m/s the constraint
carries no information, so restricting it to the coast gives nothing up and
keeps a mounting error from reaching the state during normal operation. The
delay shall be short: waiting lets the lateral velocity error grow
unconstrained (0.055 m/s^2 measured), and the constraint then arrives as a
large correction at the moment the attitude covariance is at its widest,
which is the worst time to decide how to distribute it.

## REQ-NAV-078 — Direct geodetic position accessor

- **Status:** verified
- **Parent:** REQ-NAV-005
- **Verification:** Test: tests/test_ins_core.c:scenario_get_latlonh_accessor

The filter shall publish the absolute geodetic position it maintains
(latitude, longitude, height above the WGS84 ellipsoid).
The value shall be the same anchor the filter itself carries, so that
converting it forward with ins_latlonh_to_ecef() reproduces
ins_get_position_ecef() exactly, and it shall report the same readiness
condition as the other position accessors.

Rationale: the absolute anchor is held as latitude, longitude and height
(REQ-NAV-080), and ins_get_position_ecef() converts THAT into ECEF on
every call.

## REQ-NAV-079 — GNSS position in geodetic form

- **Status:** verified
- **Parent:** REQ-NAV-005
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_pos_llh_input

A GNSS position measurement shall be handed over as geodetic coordinates
(latitude, longitude, height above the WGS84 ellipsoid) and shall be
consumed in that form, without a conversion on the epoch path. A
non-finite component shall drop the fix and count as invalid input
(REQ-SYS-007).

Rationale: the fusion works on the geodetic difference between the fix
and the filter's own anchor (REQ-NAV-005), so a fix that arrives as
latitude, longitude and height is already in the form the residual
needs.

A source that is natively ECEF, an RTK solution or UBX-NAV-HPPOSECEF,
converts once with ins_ecef_to_latlonh() before offering the fix.

## REQ-NAV-080 — Geodetic origin, ECEF only on request

- **Status:** verified
- **Parent:** REQ-NAV-005
- **Verification:** Test: tests/test_ins_core.c:scenario_geodetic_origin; Inspection: src/ins.c contains no ins_ecef_to_latlonh() at all, and its only ins_latlonh_to_ecef() is the one ins_get_position_ecef() runs on request, and no ECEF quantity is held in ins_t

The filter shall hold the origin of its local n-frame, and the absolute
position it book-keeps against that origin, as geodetic coordinates.
ECEF shall exist only at the API boundary, where a caller hands one in
or asks for one, and shall be derived there on request rather than kept
as the internal representation or cached in the filter state. No ECEF
conversion shall remain on the per-epoch path, and no ECEF quantity
shall be refreshed by it.

A pure vertical shift of the origin (REQ-NAV-025) shall change its
height alone and leave latitude and longitude bit-for-bit unchanged.

Rationale: the filter mechanizes in a local NED frame anchored on an
absolute geodetic position, and the residuals are geodetic differences
(REQ-NAV-005).

## REQ-NAV-081 — Initial position and velocity in the caller's own frame

- **Status:** verified
- **Parent:** REQ-NAV-080
- **Verification:** Test: tests/test_ins_core.c:scenario_init_llh_vel_ned

The initial position shall be supplied as geodetic coordinates
(ins_init_t.llh) and the initial velocity in the local NED frame
(ins_init_t.vel_ned), the frames the filter itself works in. A latitude
or longitude outside its range, or a non-finite one, shall make
ins_init() fail. An all-zero block shall NOT fail: it denotes the point
where the equator meets the prime meridian at ellipsoid height 0, which
is a position like any other.

Rationale: the filter anchors geodetically (REQ-NAV-080) and mechanizes
in NED, and that is also what callers hold.

