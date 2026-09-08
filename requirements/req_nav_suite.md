# nav_suite wrapper requirements (REQ-SUITE)

## REQ-SUITE-001 — Parallel operation

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_nav_suite

nav_suite_update shall drive all three filters (ins, ARS,
magnetometer-AHRS) from a single ins_measurements_t bundle per
epoch: the full bundle goes to ins, timestamp + acc + gyr (+ mag)
to the AHRS instances.

## REQ-SUITE-002 — AHRS auto-bootstrap

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_nav_suite; Test: tests/test_ahrs.c:scenario_suite_init_att_hint

The wrapper shall initialize the AHRS instances from the measurement
stream: roll/pitch from accelerometer leveling on the first valid IMU
epoch; the magnetometer instance shall wait for the first valid
magnetometer sample and bootstrap its heading from it. The ARS's
initial yaw shall be 0, UNLESS a known initial heading was supplied --
either as a prescribed initial attitude (`opt.auto_init == false`) or
as a static yaw hint armed before the ARS bootstraps
(`nav_suite_set_init_att_hint`, REQ-SUITE-017) -- in which case the ARS
shall bootstrap from that known yaw instead of an arbitrary 0 --
so the suite's best-available attitude (`nav_suite_get_rpy`) already
reflects the true heading during the ATTITUDE_ONLY window before ins
becomes ready, not a placeholder. The hint's roll/pitch shall NOT seed
the ARS (its accelerometer leveling is available from the first epoch
and beats a static assumption). The magnetometer instance is
unaffected (independent heading reference, still waits for a real
sample).

## REQ-SUITE-003 — Self-healing attitude references

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_selfhealing

If an AHRS instance becomes uninitialized (init rejected or health
check tripped), the wrapper shall re-bootstrap it on the next
suitable epoch, so the attitude references recover autonomously from
transient corruption.

## REQ-SUITE-005 — Solution-mode arbitration

- **Status:** verified
- **Parent:** REQ-SYS-010
- **Verification:** Test: tests/test_ahrs.c:scenario_tunnel; Test: tests/test_ahrs.c:scenario_suite_fallbacks; Test: tests/test_ahrs.c:scenario_suite_heading_switch_no_jump

The wrapper shall expose the current solution mode
(nav_suite_get_mode): FULL (ins ready with recent position aiding),
COASTING (ins ready but dead-reckoning on IMU only),
ATTITUDE_ONLY (ins position solution unavailable, AHRS attitude
available) or NONE. nav_suite_get_rpy() shall return the attitude
from the best available source (ins, else magnetometer AHRS, else
ARS), so a consumer keeps receiving an attitude throughout an outage.

## REQ-SUITE-006 — Barometric vertical filter integration

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_baro_suite; Test: tests/test_baro.c:scenario_baro_avg_bootstrap

The wrapper shall run the baro/accelerometer vertical channel filter
(baro_alt) next to the other filters. It shall bootstrap it on an IMU
epoch with an available attitude, anchoring the datum on the MEAN of the
plausible pressure samples collected over a short window
(NAV_SUITE_BARO_BOOT_SEC) rather than a single sample, so one glitched
startup reading cannot skew the whole vertical datum, grossly
implausible/non-finite pressure samples are excluded from the mean. The
anchor height follows REQ-SUITE-007. It shall then feed the filter
timestamp + accelerometer + attitude every IMU epoch and the pressure
sample when present. The body-to-NED rotation shall come from the
best available attitude source (ins if ready, else the magnetometer
AHRS, else the ARS). The filter configuration shall be adjustable
between nav_suite_init and the first barometer sample via a public
config template; a filter that trips its health check re-bootstraps
on the next valid pressure sample.

## REQ-SUITE-007 — Vertical datum arbitration

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_baro_anchor_from_ins; Test: tests/test_baro.c:scenario_baro_anchor_during_warmup; Test: tests/test_baro.c:scenario_height_strategy; Test: tests/test_baro.c:scenario_height_gnss_noise_outage_cycle; Test: tests/test_baro.c:scenario_datum_baro_then_lighthouse; Test: tests/test_baro.c:scenario_datum_gnss_then_baro; Test: tests/test_baro.c:scenario_datum_offset_survives_origin_shift; Test: tests/test_baro.c:scenario_datum_survives_quality_exit

The wrapper shall keep all local heights (baro_alt, the ins local
height) in one common vertical datum. The datum is fixed by the first
source and shall never move; a later source shall conform to it at its
own initialization, so the local height is continuous (no jump) across
every source arriving or leaving. The 3D filter's first local solution
is therefore (0, 0, h): horizontal 0 (there is no horizontal reference
before the first fix), vertical h equal to the current baro_alt height
(0 if no baro is running yet).

Concretely, when baro_alt is already running as ins bootstraps:

- If no external system owns the n-frame (GNSS or IMU-only), ins shall
  yield: the wrapper shifts the ins origin vertically (REQ-NAV-025) so
  the local height matches the baro_alt height. The datum is the mission
  start; the absolute solution stays on the fix. The offset filter shall
  NOT be shifted -- it is fed from the barometer (REQ-SUITE-008), which
  did not move, and ins is merely relocated onto that same datum.
- If any local NED position sample has been seen, an external system
  (e.g. a lighthouse) owns the n-frame and ins shall NOT be shifted --
  ins_shift_origin_down()'s contract requires the external frame to move
  along, which the wrapper cannot do, and a constant offset on every
  local_pos innovation would be skipped as an outlier (REQ-SYS-006),
  silently killing the aiding while the mode still reported FULL.
  Instead the wrapper shall subtract a constant offset from every
  local_pos before ins sees it (REQ-SUITE-013), so ins bootstraps at the
  datum height directly; nothing is relocated afterwards.

When ins is initialized before the first pressure sample, baro_alt
shall instead anchor at the current NED height (h_init) -- gated on
initialization, not ins_is_ready(), so a local height that already
moved from the origin during the readiness warm-up is not mistaken for
zero.

The reconciliation shall be held per n-frame ORIGIN, not per ins instance:
it shall re-arm when a bootstrap anchors a NEW origin, so a filter reset
re-joins the datum at the next bootstrap, and shall NOT re-arm when that
bootstrap keeps the origin carried across a quality-loss re-arm
(REQ-NAV-062) -- ins never left the datum there, so the only thing an
alignment could still do is move it.

The distinction is not cosmetic. At the moment of an alignment the
barometric height is taken to BE the datum-relative height, which holds
when the barometer has just anchored (its own datum, no drift yet) and at
any bootstrap that has no better claim to where the datum sits. It does
not hold mid-mission: the barometer has drifted against the fix by then
(weather, ISA model error, a tunnel's own pressure), and shifting a
carried origin onto that reading would push the accumulated disagreement
into the origin -- relocating every local coordinate the consumer holds,
which is precisely what carrying the origin exists to prevent. A carried
origin already is the datum and needs no reading to find it.

## REQ-SUITE-008 — Offset filter integration and height accessors

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_height_strategy; Test: tests/test_baro.c:scenario_height_ellipsoid_baro_drift; Test: tests/test_baro.c:scenario_height_ellipsoid_no_baro; Test: tests/test_baro.c:scenario_datum_baro_then_lighthouse

The wrapper shall define a local height reference: the height above
the NED origin from the source that survives a GNSS outage -- baro_alt
if it is running, else the ins local height (which is then driven by
local position aiding, or by GNSS itself when there is no other
source). The reference shall be used both to feed the offset filter
and to evaluate the absolute height, so that offset and height always
refer to the same source.

On every epoch carrying both a GNSS position and a local height
reference, the wrapper shall feed the offset filter (REQ-BARO-010):
GNSS ellipsoid height from the fix, vertical standard deviation from
its NED covariance (pairs without a positive vertical variance are
skipped), and the local height reference -- extrapolated to the GNSS
time of validity with the estimated climb rate when the measurement is
delayed (gnss_delay_ms).

The wrapper shall provide nav_suite_get_height() (height above the NED
origin from the best source: ins under fresh position aiding, else
baro_alt, else coasting ins) so a consumer keeps receiving a continuous
local height through GNSS outages.

The wrapper shall provide nav_suite_get_height_ellipsoid() (absolute
WGS84 height) returning ins under fresh aiding, else the local height
reference plus the estimated offset, else a coasting ins.

Under the barometric height source, ins's height shall additionally be
corrected for the drift of its own vertical DATUM. That datum is
anchored once at bootstrap and never re-tied to GNSS (REQ-NAV-055), so
the barometer's whole absolute error -- weather plus ISA model --
accumulates in it for the rest of the run and reaches an absolute height
undiminished, while ins's height RELATIVE to it stays excellent (which
is why nav_suite_get_height() keeps reporting ins unmodified). The
offset filter observes exactly that drift, so the correction shall be
its movement since the datum was fixed:

- The anchor -- where the offset filter stood when the datum was fixed --
  shall TRACK the estimate while the filter is still converging and
  freeze once it has settled. A snapshot taken mid-convergence books the
  remaining convergence as drift that never happened, and being a
  constant, that error would sit as a bias on every absolute height for
  the rest of the run. A mission too short for the filter to settle
  therefore never freezes an anchor and reports ins untouched, which is
  correct: nothing has been observed that would justify a correction.
- Only the part of the drift that exceeds the drift estimate's own
  1-sigma shall be applied, and it shall be applied less that margin, so
  the correction is continuous in the drift. ins's absolute height is the
  better of the two whenever the datum has not moved -- most missions,
  most of the time -- so the burden of proof sits on the correction. A
  drift inside the noise of its own estimate is not evidence, and
  replacing ins's height with the filter's estimate outright would trade
  a decimetre-grade height for that filter's metre-grade noise on every
  epoch.
- The correction shall be re-established from scratch whenever the datum
  re-arms on a new origin, which by REQ-SUITE-007 a carried origin
  (REQ-NAV-062) does not do.

Under a GNSS-derived height source no correction applies: ins is tied to
GNSS itself there.

It shall report an absolute height ONLY when a real GNSS fix has
anchored the n-frame origin to WGS84 -- established by any usable GNSS fix, at
bootstrap or by fusion. Without that anchor (e.g. indoor local-position
aiding, where the absolute solution rests only on the prescribed init
origin) it shall report false rather than pass off the init origin as
an ellipsoid height. Both accessors shall report false when no source
is available.

## REQ-SUITE-004 — Tunable AHRS configuration

- **Status:** implemented
- **Parent:** REQ-SYS-002
- **Verification:** Inspection: ars_cfg/ahrs_cfg templates are public struct members consumed at bootstrap, used by tools/replay.c

The noise/tuning configuration of both AHRS instances shall be
adjustable between nav_suite_init and the first update via public
config templates; the auto-bootstrap shall only overwrite the initial
attitude fields.

## REQ-SUITE-009 — Zero-rotation update propagation to ARS/AHRS

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_zaru

The wrapper shall drive the ARS and magnetometer-AHRS instances'
zero-rotation update (REQ-AHRS-016) from the same measurement bundle
given to ins: the caller's explicit m->zero_rotation_update flag,
OR'd with ins_auto_zupt_active() (ins currently inside an
auto-ZUPT/ZARU stillness run). Since that detector is internal to
ins and needs ins's velocity estimate -- something the
position-blind ARS/AHRS do not have -- the wrapper shall read the run
state via that public accessor rather than reimplementing stillness
detection, so the trigger stays true for the whole run (letting the
ARS/AHRS average the gyro over it, not just a single sample) and not
just the epoch ins itself happens to fuse.

## REQ-SUITE-010 — Zero-rotation trigger diagnostic accessor

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_zaru

The wrapper shall expose, via nav_suite_get_zaru_active(), whether a
zero-rotation update was applied to the ARS/AHRS on the most recent
nav_suite_update() call, so a caller can publish it for
diagnostics/telemetry (e.g. to visually correlate ARS gyro-bias
corrections with detected stops).

Both sources count: the trigger computed for REQ-SUITE-009, OR'd with
the ARS/AHRS's own velocity-blind auto-ZARU fallback as read back
through ahrs_zaru_applied() -- the same expression REQ-SUITE-015 drives
the vertical zero-velocity update from. Reporting only the REQ-SUITE-009
trigger would read "no zero-rotation update" throughout a standstill
without absolute position aiding: ins never initializes there and so
contributes no trigger, while the fallback fuses one into the ARS/AHRS
for the whole stop -- and a diagnostic that goes dark exactly where the
fallback is the only stillness detector left is misleading in the one
case it exists for.

## REQ-SUITE-011 — chi2 override propagation

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tests/test_ahrs.c:scenario_ahrs_chi2_disable

nav_suite_init() shall propagate opt->chi2_disable to the chi2_disable
field of all four sub-filter config templates (ars_cfg, ahrs_cfg,
baro_cfg, local_gnss_cfg), so a caller sets the single flag on
ins_options_t once instead of on each template individually. ins
itself already receives opt directly, so the same one flag reaches
every filter run through the suite.

## REQ-SUITE-012 — Sensor calibration reaches the parallel filters

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_sensor_calibration

The ARS, magnetometer-AHRS and baro/accel vertical filter run the same
physical accelerometer/gyroscope/magnetometer as ins, so nav_suite
shall feed them the same calibrated signal: nav_suite_update() shall
apply the configured sensor calibration (opt.imu_*/mag_* misalignment +
fixed bias, REQ-NAV-037/REQ-NAV-039, via ins_apply_calibration()) to
the accelerometer, gyroscope and magnetometer samples before driving the
parallel filters (bootstrap leveling/heading and per-epoch update). ins
itself already calibrates its own copy internally, so the identical
calibration is applied consistently to every filter run through the
suite. A zeroed calibration is a no-op.

## REQ-SUITE-013 — Local-position frame reconciliation and offset

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_datum_baro_then_lighthouse; Test: tests/test_baro.c:scenario_wgs84_conversion

An external local-positioning system (e.g. a lighthouse) reports NED
positions relative to its own, arbitrary frame origin. The wrapper
shall reconcile that frame with the vertical datum at the source: it
shall track a constant offset -- the external frame's coordinates of
the datum origin -- and subtract it from every local_pos sample before
ins consumes it, so ins always solves at (0, 0, h) relative to the
datum (horizontal 0, vertical the datum height) and never adopts the
external frame's zero. The offset shall be tracked (recomputed) until
ins locks the datum at its bootstrap, then frozen, and shall persist
across ins resets so a re-bootstrap re-joins the same datum. Its
vertical component aligns to the current baro_alt height (0 if no baro),
which is what keeps the local height continuous when the local system
comes online after the barometer.

The wrapper shall expose the offset via nav_suite_get_local_pos_offset()
so a caller can convert between the external frame and the local
solution (external_coord = local_solution + offset); it shall report
false until a local_pos sample has been seen.

## REQ-SUITE-014 — WGS84 / local NED conversion

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_wgs84_conversion

The wrapper shall provide nav_suite_local_to_wgs84() and its inverse
nav_suite_wgs84_to_local() to convert between a local NED position
(relative to the datum origin) and WGS84 geodetic coordinates
(latitude, longitude, ellipsoid height), so a GNSS user can relate the
local n-frame solution to WGS84 in both directions -- e.g. a mission
planner turning WGS84 waypoints into the local frame a controller flies
in, or the reverse. The conversions are the GNSS-side counterpart to
the local-position frame offset (REQ-SUITE-013). They shall be
available only once a real GNSS fix has anchored the n-frame origin to
WGS84 (as in REQ-SUITE-008) and report false otherwise, since without
that anchor the origin is only the prescribed init position and the
result would be a fictitious absolute coordinate.

## REQ-SUITE-015 — Zero-rotation update drives the vertical zero-velocity update

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_baro.c:scenario_suite_zaru_drives_baro_zupt

A platform that is rotationally still is standing still, and a platform
that is standing still has no vertical velocity either. The wrapper
shall therefore drive the vertical channel's zero-velocity update
(REQ-BARO-022) from the zero-rotation trigger it computes for the
ARS/AHRS (REQ-SUITE-009) -- the caller's explicit
m->zero_rotation_update flag OR'd with ins_auto_zupt_active() --
widened by the ARS's and AHRS's own velocity-blind auto-ZARU fallback
(REQ-AHRS-017), read back per epoch via ahrs_zaru_applied()
(REQ-AHRS-024). On every epoch that trigger is true and the vertical
filter is running, the wrapper shall call
baro_alt_zero_velocity_update().

The fallback must be included because it is the suite's ONLY stillness
detector in the case the vertical channel needs it most: without an
absolute position aid ins never initializes, so ins_auto_zupt_active()
stays false forever, and a caller that has no external stillness signal
(the live-sensor case: attitude-only until the first GNSS fix) would
otherwise leave the accelerometer-only vertical drift completely
unbounded. A caller that must not have it -- a platform dominated by
constant-velocity cruise, which no velocity-blind detector can tell from
a standstill -- opts out with
ins_options_t.auto_zupt_velocity_blind_disable (REQ-SUITE-020), which
leaves ins's own velocity-aware trigger in place.

How hard the vertical filter then trusts that trigger is not configured
here either: baro_cfg.zupt_stddev_mps comes from
ins_init_t.zero_vel_stddev_mps (REQ-SUITE-020), the same number that
tunes ins's own zero-velocity update. One trigger, one stated confidence
in it.

Consistent with REQ-BARO-022 the wrapper shall not add a velocity check
of its own: the trigger already encodes the stillness decision, and
re-gating it on an estimate that may itself have drifted would defeat
the purpose. This gives the vertical channel a stillness observation it
otherwise lacks -- during a barometer outage it is the only thing
bounding the accelerometer-only vertical drift.

The wrapper shall expose what actually reached the vertical filter via
nav_suite_get_vertical_zupt_active(), false whenever the vertical
filter did not run that epoch, so telemetry can show whether the
zero-velocity updates are arriving rather than only that a trigger
existed: the two differ whenever the vertical filter is not running,
which nav_suite_get_zaru_active() (REQ-SUITE-010) cannot say. That
accessor reports the same OR of both stillness sources, but about the
ARS/AHRS rather than about the vertical channel.

## REQ-SUITE-016 — ARS/AHRS attitude and gyro-bias seed ins's auto-init/re-acquisition

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_att_hint; Test: tests/test_ahrs.c:scenario_suite_quality_exit_rebootstrap

Before calling ins_update() each epoch, the wrapper shall build ins's
external attitude/gyro-bias hint (REQ-NAV-048, ins_measurements_t's
att_hint) from the ARS/AHRS's own current roll/pitch/gyro-bias
estimate (and yaw, magnetometer AHRS only): the magnetometer AHRS is
preferred over the ARS when it is initialized (it has a real,
absolute-heading-referenced yaw the ARS's free-integrating yaw does
not), falling back to the ARS, leaving the hint invalid if neither is
initialized yet. Every hint stddev supplied to ins shall be the
source's own reported 1-sigma inflated by a fixed factor of 10 (the
two filters process the same IMU stream but are otherwise independent
estimators, so passing the source's stddev at face value would
understate ins's actual uncertainty about a value it did not itself
derive). The hint reflects the ARS/AHRS state as of the end of the
PREVIOUS epoch (ins runs before the ARS/AHRS update within
nav_suite_update()); this one-sample staleness is not compensated for,
being negligible against the 10x stddev inflation.

## REQ-SUITE-017 — Static initial attitude hint for ins's auto-init bootstrap

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_init_att_hint

nav_suite_set_init_att_hint() shall let a caller arm a static, one-time
"I just know it" initial roll/pitch and/or yaw (e.g. a known launch
heading with no magnetometer/GNSS-course yaw aiding to derive it from),
distinct from the per-epoch ARS/AHRS-derived hint of REQ-SUITE-016.
Roll/pitch shall only be used together (both stddevs must be > 0); yaw
is independent (its own stddev > 0). The static hint shall be used only
where REQ-SUITE-016's own hint has nothing to offer: its yaw fills in
whenever the resolved hint carries no absolute yaw (the common case: a
yaw-free ARS, or neither ARS nor AHRS initialized yet), and its
roll/pitch fills in only while NEITHER ARS nor AHRS has initialized at
all -- either one's own leveling always wins once available. A yaw hint
armed before the ARS bootstraps shall additionally seed the ARS's own
initial yaw (REQ-SUITE-002), so the suite reports the known heading
from the first IMU epoch onwards. The static
hint shall stop applying once ins itself has initialized, so it cannot
bias a later re-acquisition (REQ-NAV-023 already governs that path); it
shall also be consumed EXACTLY once -- nav_suite_update() clears it the
moment ins first initializes, so a later full re-bootstrap (a severe
time jump or a health-check failure resetting is_initialized,
REQ-NAV-016/REQ-NAV-042, or a GNSS quality-loss re-arm, REQ-NAV-052)
starts without it rather than silently reapplying a
heading that may no longer hold, time having passed since the original
assertion.

## REQ-SUITE-019 — ins/baro_alt height cross-check

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_height_cross_check

While both ins and baro_alt are running, the suite shall compare their
local heights (ins -pos_local[2] against baro_alt's h, the same vertical
datum by REQ-SUITE-007) and report, throttled, a persistent
disagreement beyond a fixed bound.

This complements the vertical-velocity cross-check of the same pair
rather than duplicating it: the two are sensitive to different failures.
A disagreement in the derivative that stays well inside the velocity
bound still integrates into an unbounded height gap, so the velocity
check alone cannot see a slow divergence.

Rationale: the local height accessor (REQ-SUITE-008) takes ins's height
only in FULL mode and falls back to baro_alt otherwise, so a divergence
between the two is not an abstract inconsistency -- it is the size of
the step the reported height will take the next time the solution mode
changes. The bound is deliberately loose: the two are independent
estimators with separate accelerometer-bias states and different aiding,
and are only expected to track the same physical quantity, so this
catches a real fault in one of them, not the routine difference between
two estimates.

## REQ-SUITE-018 — Runtime-wide auto-ZUPT/ZARU disable

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_auto_zupt_zaru_disable

nav_suite_set_auto_zupt_zaru_disable() shall let a caller enable/disable,
at runtime, every automatic stillness source that can inject a
zero-velocity/zero-rotation update anywhere in the suite: ins's own
detector (ins_set_auto_zupt_disable, REQ-NAV-013) and both the ARS's
and the magnetometer-AHRS's velocity-blind auto-ZARU fallback
(ahrs_set_auto_zaru_disable, REQ-AHRS-017). baro_alt has no detector of
its own -- its vertical zero-velocity update is driven entirely by
these three (REQ-SUITE-015) -- so disabling all three transitively
stops it too, without a fourth call. This is safety-relevant, not
config sugar: a platform that is legitimately still by every gate but
must never receive a stillness update (e.g. a multicopter briefly
holding position mid-flight) shall be able to disable this for exactly
that window and re-enable it once grounded.

The call shall also update the ars_cfg/ahrs_cfg templates (not only the
live ars/ahrs instances), so the setting survives a later ARS/AHRS
re-bootstrap (nav_suite_ahrs_bootstrap consumes those templates fresh,
e.g. after a health-check-triggered re-init) instead of silently
lapsing back to enabled. An externally-triggered update
(ins_measurements_t.zero_velocity_update / zero_rotation_update) is
unaffected either way, remaining entirely under the caller's control.

## REQ-SUITE-020 — One stillness definition for the whole suite

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_stillness_propagation

nav_suite_init() shall propagate the stillness parameters on
ins_options_t / ins_init_t into every sub-filter config template, so a
caller states "what counts as standing still" once instead of per
filter:

- `auto_zupt_static_gyr_rps`, `auto_zupt_static_acc_mps2`,
  `auto_zupt_static_gyr_stddev_rps`, `auto_zupt_static_acc_stddev_mps2`
  and `auto_zupt_dwell_sec` into the matching
  `ahrs_config_t.auto_zaru_*` fields of ars_cfg and ahrs_cfg
  (REQ-AHRS-017),
- `ins_init_t.zero_rot_stddev_rps` into both templates'
  `zero_rot_stddev_rps`,
- `ins_init_t.zero_vel_stddev_mps` into `baro_cfg.zupt_stddev_mps`
  (REQ-SUITE-015),
- `auto_zupt_disable` and `auto_zupt_velocity_blind_disable` into both
  templates' `auto_zaru_disable`, the first turning off every stillness
  source in the suite, the second only the ARS/AHRS's own
  velocity-blind detector while ins keeps deciding for all of them.

`auto_zupt_max_vel_mps` / `auto_zupt_max_vel_stddev_mps` shall NOT be
propagated (the ARS/AHRS have no velocity state, which is precisely
what makes their detector velocity-blind), nor shall
`auto_zupt_min_interval_sec` (ahrs_fuse_zaru throttles its own fusion).

The 0 sentinels shall be forwarded unresolved: both sides fall back to
the same sensor_defaults.h constants, so a zeroed field keeps meaning
"module default" on either side.

Rationale: the four filters answer "is the platform still" with four
separate implementations on purpose (ins velocity-aware, the AHRS pair
velocity-blind, baro_alt not at all), but tuning them separately was a
defect source rather than a feature -- a caller who tightened ins's
gates kept an untouched ARS/AHRS fallback, and with it a baro_alt
vertical ZUPT (REQ-SUITE-015) keyed off thresholds nobody had
configured. Same argument as the chi2 override of REQ-SUITE-011. A
caller who genuinely wants them apart can still overwrite the templates
between nav_suite_init() and the first nav_suite_update(), per the
nav_suite.h contract.

## REQ-SUITE-021 — Public predict/correct API

- **Status:** verified
- **Parent:** REQ-SYS-017
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_predict_correct_equivalence

nav_suite_predict_step() and nav_suite_correct_step() shall together be
exactly equivalent to nav_suite_update(): nav_suite_update() is
implemented as nav_suite_predict_step() followed by
nav_suite_correct_step(). The split is drawn at ins's own
predict_step()/correct_step() boundary only: nav_suite_predict_step()
runs the local-datum adjustment, the ARS/AHRS attitude hint and
ins_predict_step(); nav_suite_correct_step() runs ins_correct_step()
followed by everything else nav_suite_update() does today (vertical
datum alignment, the local-GNSS offset filter, mode logging, and the
ARS/AHRS/baro_alt bootstrap-or-update calls with their cross-check
diagnostics), in the same relative order as nav_suite_update() today.
The ARS/AHRS/baro_alt sub-filters are not themselves split at the
suite level (their bootstrap-or-update branching and the cross-filter
feeds between them sit strictly between "ins done" and "everything
else", not before/after a suite-wide predict/correct boundary) -- a
caller that also wants their covariance sampled between prediction and
correction can call ahrs_predict_step() / baro_alt_predict_step()
directly on the s->ars / s->ahrs / s->baro_alt sub-filter fields.

nav_suite_predict_step() shall accept an optional phi_out buffer,
forwarded verbatim to the underlying ins_predict_step() call.

The caller's original, unmodified measurement block shall be handed off
internally from nav_suite_predict_step() to nav_suite_correct_step()
(everything ins-specific is already carried across by ins_t.step_ctx) --
nav_suite_correct_step() shall be a no-op if the matching
nav_suite_predict_step() was never called.

## REQ-SUITE-022 — Heading carry-over across an ins re-initialization

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_yaw_carry

On every epoch where ins is ready and its own yaw 1-sigma is at most 15
degrees, the wrapper shall latch that heading together with the ARS yaw
of the same epoch. While ins is NOT initialized and REQ-SUITE-016's
resolved hint carries no absolute yaw (the case without a magnetometer
AHRS), the wrapper shall supply as the hint yaw the latched heading
advanced by the yaw the ARS has integrated since the latch, at a 1-sigma
of sqrt(latched^2 + (10 * ARS z gyro-bias 1-sigma * elapsed time)^2) --
the latched uncertainty plus the drift the ARS has accumulated on top of
it, inflated by the same factor as every other ARS-derived hint value
(REQ-SUITE-016). The carried heading shall be withheld once that 1-sigma
exceeds 30 degrees, and dropped entirely when the ARS re-bootstraps
(its yaw restarts from the config seed, so the difference against the
latched value stops being a tracked rotation).

The same reconstruction shall be what nav_suite_get_rpy() reports for
yaw whenever it falls back to the ARS (REQ-SUITE-005), so the suite's
heading does not jump to the ARS's seed-relative yaw the moment ins
drops out, and agrees with the heading ins is re-initialized from.
Roll/pitch stay the ARS's own. The per-filter accessor
nav_suite_get_rpy_ars() shall keep reporting the raw ARS yaw, being what
a cross-check of the two estimators reads.

Rationale: a re-initializing ins re-derives its attitude from scratch,
and without a magnetometer or an external heading it has no heading
source at all -- it starts from the "unknown" prior (REQ-NAV-059) and
waits for aiding to pull the heading in. The ARS kept running across
the whole outage. Its yaw is not a heading (it free-integrates and
drifts, which is why REQ-SUITE-016 never passes it on as one), but the
CHANGE it reports over an outage is good, and a change is all this
needs: the last 3D heading plus that change is a real prior. The two
gates keep it one: nothing is carried unless ins had genuinely resolved
its heading, and the carry retires itself once the accumulated drift has
eaten the information, falling back to the honest "unknown".

## REQ-SUITE-023 — ins/AHRS heading cross-check

- **Status:** verified
- **Parent:** REQ-SYS-002
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_heading_cross_check

While both ins and the magnetometer AHRS are initialized, the suite
shall compare their headings (ins yaw against the AHRS's own yaw,
wrapped) and report, throttled, a persistent disagreement beyond a
fixed bound.

Rationale: nav_suite_get_rpy() falls back from ins to the magnetometer
AHRS the moment ins leaves FULL/COASTING (REQ-SUITE-005). The two are
independent heading estimators -- ins additionally corrected by
position aiding, the AHRS referenced to the raw magnetometer -- that
are only expected to agree, not track identically. A persistent gap
(residual hard/soft-iron calibration error, declination, a
converged-but-wrong ins yaw prior) means the reported heading will
STEP by exactly that amount at the next mode switch, the same
situation REQ-SUITE-019 already flags for height, applied to yaw
instead. Sized generously (comparable to REQ-SUITE-019's 10 m): a
magnetic heading reference several degrees off an ins solution is
unremarkable in daily operation and not itself a defect.

Diagnostic only, like the three cross-checks it sits alongside
(ins/ARS gyro bias, ins/baro_alt vertical velocity, ins/baro_alt
height): it does not correct nav_suite_get_rpy() or force a resync,
since neither estimator is authoritative over the other.
