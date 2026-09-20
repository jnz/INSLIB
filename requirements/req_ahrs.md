# AHRS attitude filter requirements (REQ-AHRS)

## REQ-AHRS-001 — Filter modes

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_pyahrs_example; Test: tests/test_ahrs.c:scenario_mag_yaw_convergence

The AHRS shall support two modes fixed per instance: ARS
("directional gyro", 5 error states: roll/pitch error + 3 gyro
biases) where yaw is integrated but never corrected, and AHRS (6
error states: roll/pitch/yaw error + 3 gyro biases) where yaw is
corrected with magnetometer measurements.

## REQ-AHRS-002 — Error-state formulation

- **Status:** implemented
- **Parent:** REQ-SYS-002
- **Verification:** Inspection: ahrs.c corrects quaternion + gyro bias via n-frame small-angle error state, UDU covariance (kalman_udu)

The filter shall estimate an attitude quaternion and a gyroscope bias
via an error-state Kalman filter (n-frame psi-angle model) with the
covariance kept as a UDU factorisation -- the same backend as ins.

## REQ-AHRS-003 — Accelerometer leveling

- **Status:** verified
- **Parent:** REQ-AHRS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_pyahrs_example

Roll and pitch shall be corrected from low-pass-filtered
accelerometer measurements against the gravity direction, with the
measurement noise inflated proportionally to | |f| - g |
(gravity_diff_penalty) to reduce the influence of acceleration
phases.

## REQ-AHRS-004 — Gyro bias estimation

- **Status:** verified
- **Parent:** REQ-AHRS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_pyahrs_example

The filter shall estimate the gyroscope bias online from the leveling
(and, in AHRS mode, heading) corrections; on the pyahrs reference
scenario the x/y bias estimates shall converge to within
0.1 deg/s of truth.

## REQ-AHRS-005 — Free yaw in ARS mode

- **Status:** verified
- **Parent:** REQ-AHRS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_free_yaw_integration

In ARS mode, yaw shall be the pure integral of the bias-corrected
z-rotation rate: no measurement shall correct it.

## REQ-AHRS-006 — Magnetometer heading fusion

- **Status:** verified
- **Parent:** REQ-AHRS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_mag_yaw_convergence; Test: tests/test_ahrs.c:scenario_mag_edge_cases

In AHRS mode the filter shall fuse a tilt-compensated magnetic
heading (reference: magnetic north, no declination applied) as a
scalar yaw measurement. Only yaw shall be affected -- magnetic
disturbances shall not tilt roll/pitch. Fusion shall be skipped near
gimbal lock, for near-zero fields and when the de-tilted horizontal
field component is unusable.

## REQ-AHRS-007 — Outlier downweighting

- **Status:** implemented
- **Parent:** REQ-SYS-006
- **Verification:** Inspection: ahrs_fuse passes chi2 thresholds to the robust kalman_udu update (downweight, not skip)

Accelerometer and heading measurements shall be screened per scalar
measurement row with a chi-square test and downweighted (not
dropped) when implausible, so a persistent reference offset cannot
deadlock the filter.

## REQ-AHRS-008 — Configuration defaults

- **Status:** implemented
- **Parent:** REQ-SYS-002
- **Verification:** Inspection: ahrs_resolve_config, defaults match pyahrs.py constructor defaults

Every noise/tuning configuration field left at 0 shall be replaced by
a documented default (taken from the pyahrs.py reference); only the
initial attitude and its standard deviation are mandatory. Invalid
configurations (non-positive or non-finite mandatory fields) shall be
rejected at init.

## REQ-AHRS-009 — Python reference equivalence

- **Status:** deleted
- **Parent:** REQ-SYS-009
- **Verification:** Inspection: obsolete -- python/pyahrs.py was removed from the repository (2026-07), superseded by REQ-AHRS-013

(Deleted, kept for ID stability.) Required the C implementation to
meet the convergence tolerances of the Python original pyahrs.py on
its reference scenario. The Python implementation is no longer part
of the repository; the numeric convergence criteria live on in
REQ-AHRS-013, verified by the same test scenario.

## REQ-AHRS-013 — Reference-scenario convergence

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_pyahrs_example

On the reference scenario (100 Hz IMU with gaussian noise, constant
gyro bias of 1/-2/0 deg/s, initial attitude error of 3 deg in
roll/pitch, static body) the filter shall converge within 10 s to:
roll/pitch within 0.2 deg of truth and x/y gyro bias within
0.1 deg/s. (Historical note: scenario and tolerances originate from
the pyahrs.py reference implementation this filter was ported from,
see REQ-AHRS-009.)

## REQ-AHRS-010 — Non-finite input handling

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_ahrs.c:scenario_nan_inputs

ahrs_update shall drop epochs with non-finite gyro/accelerometer
samples and ignore non-finite magnetometer samples (counting both in
n_invalid_input), continuing normally with the next valid data.

## REQ-AHRS-011 — Time anomaly handling

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_time_anomaly

A backwards timestamp step shall re-anchor the internal clocks and
skip the epoch; a forward gap larger than 0.2 s shall skip the
attitude integration for that epoch (sensor-outage semantics).

## REQ-AHRS-012 — Initialization heuristics

- **Status:** verified
- **Parent:** REQ-AHRS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_mag_heading_helper

Helper functions shall estimate roll/pitch from a static
accelerometer sample (leveling) and yaw from a magnetometer sample
given roll/pitch (tilt-compensated heading) to bootstrap the filter.

## REQ-AHRS-014 — Position aiding and true-north heading

- **Status:** verified
- **Parent:** REQ-SYS-013
- **Verification:** Test: tests/test_ahrs.c:scenario_wmm_position_aiding

The AHRS shall accept a position (latitude, longitude, decimal year)
at any time via a dedicated interface, so that a coarse fix arriving
after start-up (e.g. the first GNSS position) can be supplied. Once a
position is set, the World Magnetic Model declination shall reference
the estimated yaw to true north; until then the AHRS references
magnetic north. Switching on (or changing) the declination shall
re-frame the nominal attitude deterministically -- the estimated yaw
steps by the declination change instead of slewing there through the
magnetometer fusion -- while leaving the covariance and gyro-bias
states unchanged. A non-finite argument shall be ignored.

When the supplied position lies inside a magnetic dip pole exclusion
zone (REQ-SYS-018), the AHRS shall instead suspend magnetometer yaw
fusion and retain its previous declination, leaving roll and pitch
aiding untouched. On the position that leaves the zone the AHRS shall
adopt the new declination WITHOUT re-framing the attitude, because the
re-framing above is only sound while the yaw is magnetically anchored,
and after a pass through a zone it is anchored to the gyro. The
declination on the far side of a dip pole differs by tens of degrees,
so re-framing there would rotate a sound estimate by that amount
instead of correcting it. The residual gyro drift is then removed by
the resuming magnetometer fusion through the normal covariance
weighting.

If the first position the AHRS ever receives already lies inside an
exclusion zone while its yaw has been fused against magnetic north,
there is no previous declination to retain: the yaw is a magnetic
heading that no re-framing will ever turn into a true one, yet its
covariance still claims the accuracy of that fusion. The AHRS shall
then widen the yaw variance to "heading unknown" (never narrowing it),
capped just below the yaw threshold of the attitude-precision restart
watchdog while that is armed (REQ-AHRS-023), since a restart would
discard the zone state together with the position and fuse the
magnetometer against magnetic north again, so that the yaw it reports, and any consumer of its standard deviation
such as the nav_suite attitude hint (REQ-SUITE-016), does not present a
magnetic heading as a converged true one. A yaw that was never fused
against the magnetometer keeps its variance.

## REQ-AHRS-015 — Magnetometer field-strength disturbance rejection

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ahrs.c:scenario_wmm_position_aiding

The AHRS shall provide an opt-in field-strength gate (off by default):
when enabled and a position is known, it shall compare the measured
magnetic field magnitude against the World Magnetic Model total-field
expectation and downweight (inflate the measurement noise of, not
drop) a heading sample whose magnitude deviates by more than a
configurable tolerance (default 30%), catching local magnetic
disturbances that the heading-only residual test cannot see. Because
the scalar heading fusion is otherwise unit-agnostic (direction only),
enabling the gate shall require the magnetometer in the model's unit
(uT); it is therefore off by default so that supplying a position for
declination alone never disturbs a direction-only magnetometer.
Rationale: the magnetometer is a persistent absolute reference, so it
is downweighted rather than skipped (REQ-SYS-006).

## REQ-AHRS-016 — Zero-rotation update

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_zaru; Test: tests/test_ahrs.c:scenario_auto_zaru_vibration

ahrs_update shall accept a caller-supplied zero_rotation_update flag;
while set (OR'd with the velocity-blind fallback of REQ-AHRS-017), the
filter shall fuse, at most once per covariance-prediction period, a
direct measurement of the gyro bias states (Earth rotation rate is NOT
compensated, unlike ins's equivalent -- this filter has no position
awareness and the rate is far below what this class of sensor/filter
resolves). While the fallback's own stillness criteria confirm the
run, that measurement is the raw gyro averaged over it, so vibration
cancels out instead of being injected into the bias states; a caller-
only trigger with no such run backing it (fallback disabled, or the
caller knows better than the static gate) has nothing to average, so it
fuses the instantaneous sample instead -- the same fallback
ins_fuse_zero_rotation() uses for its own external-trigger case. The
accumulator shall reset whenever those stillness criteria stop holding
or a backwards time step re-anchors the filter's clocks.
This filter has no position/velocity state of its own, so (unlike
ins) it cannot detect stillness by itself -- the trigger must come
from outside; see REQ-SUITE-009 for how nav_suite supplies one.

## REQ-AHRS-017 — Velocity-blind auto-ZARU fallback

- **Status:** verified
- **Parent:** REQ-AHRS-016
- **Verification:** Test: tests/test_ahrs.c:scenario_auto_zaru_fallback

The filter shall provide a fallback stillness detector
(opt-out via ahrs_config_t.auto_zaru_disable, ARMED by default -- the
negative name is what lets a zeroed config arm it, matching
ins_options_t.auto_zupt_disable) that arms the
zero-rotation update from the gyro/accelerometer alone -- the same
criteria and dwell time as ins's own auto-ZUPT/ZARU detector
(REQ-NAV-013): the raw-IMU window variance as the primary stillness
statement, the static-gyro and specific-force-vs-gravity magnitude
gates as loose sanity bounds beside it, and configurable variance
thresholds (auto_zaru_static_gyr_stddev_rps /
auto_zaru_static_acc_stddev_mps2, 0 -> built-in default) -- MINUS the
velocity check ins uses to reject constant-velocity cruise (this filter
has no velocity state). It is armed by default because it is the only
stillness source available before ins has initialized, and it is what
feeds baro_alt's vertical ZUPT (REQ-SUITE-015); the accepted cost is
that constant-velocity travel is indistinguishable from a standstill on
an IMU alone, at any threshold, so a platform whose profile is dominated
by such cruise should opt out. The variance criterion matters more here than
in ins: this filter has no accelerometer bias state at all, so a
magnitude-only accelerometer gate tripped by a biased sensor could
never be escaped. It shall stay armed for the whole stillness run once dwelled
(not fire once and drop), and a public accessor
(ahrs_auto_zaru_active) shall report whether it is currently armed.
Rationale: when no external trigger is available at all (e.g. ins
never initializes -- "aiding: none" -- so REQ-SUITE-009 has nothing to
propagate), this is the only way to correct the otherwise permanently
unobservable z-axis gyro bias; the false-positive risk during real
constant-velocity cruise is the accepted cost of having no
alternative, hence opt-in rather than a new default.

Standalone, all of the above is configured through ahrs_config_t. Inside
nav_suite these fields are not tuned here at all: nav_suite_init() fills
them from ins_options_t.auto_zupt_* (REQ-SUITE-020), so the suite has
one stillness definition and four implementations of it rather than four
independently tuned detectors. The opt-out then also has two levels:
ins_options_t.auto_zupt_velocity_blind_disable turns off this detector
alone (ins keeps deciding for everyone through REQ-SUITE-009), while
auto_zupt_disable turns off every stillness source in the suite.

The fallback shall additionally be disableable at runtime
(ahrs_set_auto_zaru_disable), independent of cfg.auto_zaru_disable set
at ahrs_init, mirroring ins_set_auto_zupt_disable (REQ-NAV-013) so the
two can be toggled together (see nav_suite_set_auto_zupt_zaru_disable,
REQ-SUITE-018). Disabling shall clear the in-progress dwell timer and
the latched variance-window verdict, so no evaluation from before the
disable can fire the instant the fallback is re-enabled.

## REQ-AHRS-018 — chi2 override (AHRS)

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tests/test_ahrs.c:scenario_ahrs_chi2_disable

When ahrs_config_t.chi2_disable is set, both the accelerometer
leveling and the magnetometer heading fusion (ahrs_fuse()) shall be
fused at their nominal variance regardless of the innovation size --
the chi2 test is skipped entirely, nothing is ever downweighted. The
ZARU update is unaffected (it already fuses unconditionally,
chi2_threshold == 0.0f).

## REQ-AHRS-019 — Downweight diagnostic counter (AHRS)

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ahrs.c:scenario_ahrs_chi2_disable

ahrs_t shall expose a monotonic counter, n_downweighted, incremented
once per ahrs_fuse() call in which the chi2 test tripped and was
downweighted. Diagnostic only (does not affect the fusion); stays 0
whenever REQ-AHRS-018's chi2_disable override is set.

## REQ-AHRS-020 — Overconfidence / covariance-collapse watchdog (AHRS)

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ahrs.c:scenario_ahrs_overconfidence

The AHRS filter shall, each post-init epoch, monitor its own reported
attitude 1-sigma (from the covariance diagonal of the tracked attitude
error states -- roll/pitch in both modes, plus yaw in AHRS mode; the ARS
integrates yaw freely, so no yaw state exists to collapse) and flag when
any per-axis value falls below a physically implausible floor
(AHRS_OVERCONF_ATT_STDDEV_DEG = 0.001 deg) -- the attitude analogue of the
ins covariance-collapse watchdog (REQ-NAV-040). ahrs_t shall expose a
latched flag (overconfident), a monotonic epoch counter (n_overconfident)
and the smallest per-axis attitude 1-sigma seen since ahrs_init
(min_att_stddev_deg; INFINITY until the first post-init epoch). Purely
diagnostic: the filter shall never act on it.

## REQ-AHRS-021 — Throttled covariance prediction (AHRS)

- **Status:** verified
- **Parent:** REQ-SYS-004
- **Verification:** Test: tests/test_ahrs.c:scenario_covariance_throttled; Test: tests/test_ahrs.c:scenario_kalman_cadence_tolerance

The attitude integration shall run at the full IMU rate, but the
error-state covariance prediction shall be throttled to a configurable
period (ahrs_config_t.kalman_update_dt_sec), not run every IMU epoch --
the error dynamics are slow enough that a lower cadence tracks them
adequately (Wendel, 2nd ed., ch. 8.2.1: "typically 10 Hz"), the same
rationale as the ins Kalman prediction (REQ-NAV-004). The period defaults
to 1/20 s (20 Hz, a safety margin over Wendel's figure) when left at 0,
and the zero-rotation update is rate-limited to the same period.

Both due tests -- the covariance prediction and the zero-rotation rate
limit -- shall carry the same relative tolerance as REQ-NAV-004
(INS_CADENCE_TOLERANCE), for the same reason and with the same bound:
this filter is throttled against the same physical IMU stream, so an
epoch cadence that lands a hair below the configured period halves its
prediction rate exactly as it would in ins.

## REQ-AHRS-022 — Accelerometer gravity-magnitude gate

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_ahrs.c:scenario_acc_gravity_gate

The accelerometer leveling update shall be hard-rejected (dropped, not
just downweighted) for the epoch when the low-pass-filtered specific
force magnitude deviates from local gravity by more than a configurable
threshold (ahrs_config_t.acc_reject_gravity_mps2; default 2.0 m/s^2, < 0
disables the gate) -- such a specific force is a maneuver/shock/vibration
transient, not a gravity leveling reference. The number of rejected
updates shall be exposed as a monotonic diagnostic counter
(n_acc_rejected). This complements, and runs ahead of, the proportional
noise inflation of REQ-AHRS-003; the fusion throttle clock is advanced
regardless of the outcome so a sustained-vibration environment adds no
extra computational load. A hard drop is admissible here (whereas the
persistent magnetometer/accelerometer reference offsets of REQ-AHRS-007
are downweighted) because a gravity-magnitude deviation is inherently
transient and cannot deadlock the filter.

## REQ-AHRS-023 — Attitude-precision restart watchdog

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_ahrs.c:scenario_precision_restart

The filter shall, each post-init epoch and after a configurable warm-up
(ahrs_config_t.restart_warmup_sec, default 10 s), compare its own
reported per-axis attitude 1-sigma (from the covariance diagonal of the
tracked attitude error states -- roll/pitch in both modes, plus yaw in
AHRS mode) against a per-axis threshold
(ahrs_config_t.restart_att_stddev_rad; default 10/10/90 deg, an axis set
< 0 is not checked). If any checked axis exceeds its threshold the
estimate is no longer trustworthy and the filter shall mark itself
uninitialized (is_initialized = false, the same fail-safe mechanism as
the health check) and increment a monotonic diagnostic counter
(n_restart); nav_suite then re-bootstraps the instance from the live
measurement stream, while a standalone caller must re-init. The warm-up
suppresses the check while the filter is still converging from its
(possibly deliberately loose) initial covariance. The watchdog is on by
default and can be disabled via ahrs_config_t.precision_restart_disable.

## REQ-AHRS-024 — Zero-rotation trigger accessor

- **Status:** verified
- **Parent:** REQ-AHRS-016
- **Verification:** Test: tests/test_ahrs.c:scenario_zaru_applied_accessor

The filter shall expose, via ahrs_zaru_applied(), whether a
zero-rotation trigger was present on the most recent ahrs_update()
call: the caller's explicit zero_rotation_update flag OR'd with the
velocity-blind fallback's dwell-satisfied decision (REQ-AHRS-017). It
shall report the value the fusion actually acted on, so it is false on
an epoch that was dropped (non-finite input, backwards time) rather
than repeating the previous epoch's value, and it is deliberately
narrower than ahrs_auto_zaru_active(), which is already true from the
start of a stillness run, before the dwell has elapsed.

Rationale: this is the suite's only stillness signal when ins never
initializes, so other filters must be able to key off it (nav_suite
drives the vertical channel's zero-velocity update from it,
REQ-SUITE-015). ahrs_auto_zaru_active() cannot serve that purpose: it
covers neither the external trigger nor the dwell gate.

## REQ-AHRS-025 — Public predict/correct API

- **Status:** verified
- **Parent:** REQ-SYS-017
- **Verification:** Test: tests/test_ahrs.c:scenario_predict_correct_equivalence

ahrs_predict_step() and ahrs_correct_step() shall together be exactly
equivalent to ahrs_update(): ahrs_update() is implemented as
ahrs_predict_step() followed by ahrs_correct_step(), with no
bookkeeping (non-finite input handling, backward time-jump re-anchor,
health/precision checks) left behind in ahrs_update() itself that a
caller driving the two halves separately would miss.
ahrs_predict_step() shall accept an optional phi_out buffer for the
n x n (n = a->n) state transition matrix used that call, filled only
when the covariance was actually propagated (see
AHRS_EPOCH_COV_PROPAGATED).

The sanitized magnetometer sample (nulled out if non-finite) and this
epoch's gyro/accel/zero-rotation-trigger shall be handed off internally
from ahrs_predict_step() to ahrs_correct_step() (not re-derived, since
the timing decision is not recoverable once t_last_gyr/t_last_cov_predict
have moved on) -- ahrs_correct_step() shall be a no-op if the matching
ahrs_predict_step() dropped the epoch or was never called.
