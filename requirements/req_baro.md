# Baro/accelerometer vertical channel filter requirements (REQ-BARO)

## REQ-BARO-001 — Vertical channel state vector

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_baro_convergence

The vertical channel filter shall estimate three states from
barometer and accelerometer input: height above start h [m, positive
up], vertical velocity v [m/s, positive up] and a slow-varying
additive correction a_b [m/s^2] to the measured n-frame vertical
acceleration (absorbing accelerometer bias, attitude error and
gravity model error projected onto the vertical axis). Only h is
directly measured (barometer); v and a_b are hidden states.

## REQ-BARO-002 — Accelerometer as control input

- **Status:** verified
- **Parent:** REQ-BARO-001
- **Verification:** Test: tests/test_baro.c:scenario_baro_deadreckon

The prediction step shall use constant-acceleration kinematics
x_k = Phi * x_{k-1} + B * a with

    Phi = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1],  B = [0.5*dt^2; dt; 0]

where the control input a is the measured n-frame up-acceleration:
the body-frame specific force rotated to NED with the supplied
attitude quaternion, gravity removed (a = -(f_n_z + g), positive up).
The accelerometer is a control input, not a measurement; between
barometer samples the filter shall dead-reckon on it.

## REQ-BARO-003 — Process noise model

- **Status:** verified
- **Parent:** REQ-BARO-001
- **Verification:** Test: tests/test_baro.c:scenario_baro_h_init; Test: tests/test_baro.c:scenario_baro_rate_invariant_process_noise; Test: tests/test_baro.c:scenario_baro_h_process_noise; Inspection: baro_alt_predict builds G = [e1, e2, e3] (e1=[1,0,0]', e2=[0,1,0]', e3=[0,0,1]') and Q = diag(sigma_h^2 * dt, sigma_a^2 * dt, sigma_b^2 * dt) for kalman_udu_predict, equal to e1*(sigma_h^2*dt)*e1' + e2*(sigma_a^2*dt)*e2' + e3*(sigma_b^2*dt)*e3'

sigma_h (small direct process noise on h), sigma_a (accelerometer
noise) and sigma_b (bias drift on a_b) are continuous-time spectral
noise densities [X/sqrt(Hz)], scaled by the actual prediction interval
dt so the injected process noise is rate-invariant (same accumulated
variance per second regardless of the IMU rate), unlike a fixed
per-sample sigma: Q = diag(sigma_h^2 * dt, sigma_a^2 * dt, sigma_b^2 *
dt).

sigma_a shall enter v's OWN derivative directly, with UNIT weight
(e2 = [0,1,0]'); the "B*a" control-input matrix from REQ-BARO-002
describes the DETERMINISTIC state update, not the noise-input
weighting here. Pairing a dt-weighted noise entry with a Q that
already carries its own dt factor double-counts the scaling and makes
the accumulated variance shrink with the IMU rate instead of staying
constant -- exactly backwards from what this requirement calls for,
and exactly the bug scenario_baro_rate_invariant_process_noise now
locks in against (run the identical free-coasting interval at two
different IMU rates and require the same accumulated 1-sigma, within
discretization tolerance).

sigma_h shall likewise enter h's OWN derivative directly, with UNIT
weight (e1 = [1,0,0]'), independent of v's injected noise. This is a
deliberately small, separately configurable stand-in for the exact
continuous-discrete noise terms (Q_hh ~ sigma_a^2*dt^3/3, Q_hv ~
sigma_a^2*dt^2/2) that a G with unit weight solely on v leaves at
exactly zero for a given step -- without it h inherits v's injected
uncertainty only one step later, through Phi's v->h coupling
(Phi[0,1] = dt) accumulating over subsequent prediction steps, the
same convention ins.c uses for its own accelerometer-noise-into-
velocity term (position is never a direct noise column there either).
sigma_h covers unmodelled height dynamics that channel does not reach
in a single step; it is not intended to reproduce the omitted exact
terms precisely.

## REQ-BARO-004 — ISA pressure-to-altitude conversion and start anchor

- **Status:** verified
- **Parent:** REQ-BARO-001
- **Verification:** Test: tests/test_baro.c:scenario_isa_conversion

Static pressure [Pa] shall be converted to barometric altitude with
the international standard atmosphere formula
h_baro = 44330 * (1 - (p/p0)^(1/5.255)), p0 = 101325 Pa. The altitude
of the anchor pressure sample passed to init shall define h = h_init
(see REQ-BARO-009; default 0 = "height above start"); barometer
measurements are fused as z = h_baro - h_0 with h_0 the anchor
altitude minus h_init.

## REQ-BARO-005 — Barometer measurement update with outlier downweighting

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_baro.c:scenario_baro_outlier

Barometric altitude shall be fused as a scalar measurement observing
only h (H = [1 0 0]) through the UDU backend. Implausible
measurements (chi-square test on the innovation) shall be
downweighted, not dropped: the barometer is a persistent absolute
reference, so the filter shall follow a persistent offset instead of
deadlocking on it.

## REQ-BARO-006 — Configuration defaults

- **Status:** implemented
- **Parent:** REQ-SYS-011
- **Verification:** Inspection: baro_alt_resolve_config replaces every field <= 0 or non-finite by the documented default; Inspection: baro_alt_init rejects a non-finite/implausible anchor pressure

Every noise/tuning configuration field left at 0 shall be replaced by
a documented default (sigma_a = 0.025 m/s^2/sqrt(Hz), sigma_b =
0.005 m/s^2/sqrt(Hz), sigma_h = 0.005 m/sqrt(Hz), barometer stddev
2.0 m, chi2 gate 3.8415, and the initial state stddevs, whose
accel-bias prior is 0.1 m/s^2). sigma_a/
sigma_b/sigma_h are continuous-time spectral noise densities (see
REQ-BARO-003), not per-sample values, so the injected process noise
stays rate-invariant. sigma_a is intentionally NOT the same
value as ins's accelerometer default (sensor_defaults.h): the
"accelerometer" input here is a full attitude-projected vertical
acceleration (REQ-BARO-002), so its effective noise also absorbs
roll/pitch projection error, not just raw sensor noise -- the two are
several times apart and not interchangeable (verified: sharing
ins's raw value regressed scenario_baro_h_init). The accel-bias prior
and sigma_b are sized for an uncalibrated accelerometer: a sensitivity
error of a percent puts well over 0.1 m/s^2 into a_up, and a prior
below that leaves the residual in the velocity state instead of the
bias state, where it appears as a standing vertical velocity. Init
shall be rejected for a non-finite or implausible anchor pressure.

## REQ-BARO-007 — Non-finite input handling

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_baro.c:scenario_baro_nan_inputs

baro_alt_update shall drop epochs with non-finite accelerometer or
quaternion input and ignore non-finite or implausible pressure
samples (counting both in n_invalid_input), continuing normally with
the next valid data.

## REQ-BARO-008 — Time anomaly handling

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_baro.c:scenario_baro_time_anomaly

A backwards timestamp step shall re-anchor the internal clock and
skip the epoch; a forward gap larger than 0.2 s shall skip the state
propagation for that epoch (sensor-outage semantics) while barometer
fusion continues.

## REQ-BARO-009 — Datum-aligned initialization

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_baro_h_init

baro_alt_init shall accept an initial height h_init (with standard
deviation) stating the height of the anchor pressure sample above the
caller's vertical datum. The filter state shall start at h = h_init
and all barometer measurements shall be mapped into that datum, so
the filter can be anchored to the NED origin of the navigation filter
instead of its own start point.

## REQ-BARO-010 — Local height / GNSS ellipsoid offset filter

- **Status:** verified
- **Parent:** REQ-SYS-012
- **Verification:** Test: tests/test_baro.c:scenario_offset_filter

A separate 1-state Kalman filter shall estimate the offset
o = h_gnss_ellipsoid - h_local from pairs of simultaneous local height
and GNSS ellipsoid height, where h_local is any height above the
caller's vertical datum. The offset is the ellipsoid height of the
datum origin; an absolute height follows as h_ell = h_local + o.

The filter shall be agnostic to the source of h_local. For the
barometric source (the usual case) the offset additionally contains
the weather-dependent pressure offset and the ISA model error, and for
every source it contains the geoid undulation -- which is why it is
modelled as a random walk rather than a constant (REQ-BARO-011).

The measurement variance shall be the sum of the pair's variances
(local height and GNSS vertical accuracy); the filter shall be
initialized from the first pair with that combined variance.

## REQ-BARO-011 — Slow offset dynamics

- **Status:** verified
- **Parent:** REQ-BARO-010
- **Verification:** Test: tests/test_baro.c:scenario_offset_drift_tracking

The offset shall be modelled as a slow random walk (default
0.3 m/sqrt(s), configurable): slow enough that per-pair measurement
noise is averaged out, fast enough to track weather-induced
barometric drift (order of metres per hour) and the ISA-model error
that grows with the height excursion from the anchor (observed as a
multi-metre swing over minutes, not hours, on routes with large
altitude changes). The offset variance shall keep growing while no
pairs are available, so stale offsets lose confidence during outages.

It is the ISA term that sizes the default, not the weather: weather
alone would be satisfied by an order of magnitude less. The ISA error
is a fraction of the height EXCURSION -- around a tenth of it is
ordinary, since the model assumes a sea-level temperature the real
atmosphere rarely has -- so a vehicle climbing a couple of hundred
metres in a few minutes moves the offset by tens of metres in that
time. A random walk in TIME can only bound a drift whose driver is
altitude; it cannot model it. The bound shall be wide enough that the
chi-square screening of REQ-BARO-012 still reads such a drift as drift.
A walk too slow to keep up turns real, persistent drift into a stream
of apparent outliers, and the downweighting then holds the offset back
precisely when it has the most catching up to do.

The default is tuned for the barometric source. For a local height
source that does not drift against the ellipsoid (e.g. local position
aiding from a lighthouse system) the true offset is constant and the
default random walk is faster than necessary: harmless (it only keeps
the offset variance larger than achievable), but callers may lower
rw_stddev_mps for such a source.

## REQ-BARO-012 — Offset outlier downweighting

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_baro.c:scenario_offset_outlier

Offset measurements shall be screened with a chi-square test on the
innovation and downweighted (not dropped) when implausible: both
inputs are persistent absolute references, so a persistent step must
be followed eventually instead of deadlocking, while a single spike
must not disturb the offset.

## REQ-BARO-013 — Offset non-finite input handling

- **Status:** verified
- **Parent:** REQ-SYS-007
- **Verification:** Test: tests/test_baro.c:scenario_offset_nan_inputs

local_gnss_alt_update shall drop pairs with non-finite altitudes or
non-positive/non-finite GNSS accuracy (counted in n_invalid_input)
and continue with the next valid pair; init shall reject such inputs.

## REQ-BARO-014 — Offset filter low-rate fusion

- **Status:** verified
- **Parent:** REQ-BARO-010
- **Verification:** Test: tests/test_baro.c:scenario_offset_low_rate_default

The offset filter shall fuse at most once per a configurable minimum
interval (default 10 s, i.e. ~0.1 Hz), even if pairs arrive faster.
Pairs arriving before the interval has elapsed since the last fusion
shall be counted (n_decimated) and otherwise ignored, without
advancing the propagation clock; the skipped time is folded into the
random-walk propagation of the next accepted pair. This bounds how
often the offset can be nudged, addressing excessive jumpiness
observed when fusing at the caller's raw pair rate.

## REQ-BARO-015 — Offset filter measurement stddev derating

- **Status:** verified
- **Parent:** REQ-BARO-010
- **Verification:** Test: tests/test_baro.c:scenario_offset_low_rate_default

Before combining the pair's local height and GNSS 1-sigma stddevs into
the measurement variance, both shall be multiplied by a configurable
inflation factor (default 3). This derates each fusion (smaller
Kalman gain), further limiting how far a single pair can move the
offset -- combined with REQ-BARO-014 to address a jumpy offset filter.

## REQ-BARO-016 — chi2 override (baro_alt and offset filter)

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tests/test_baro.c:scenario_chi2_disable

When chi2_disable is set on baro_alt_config_t, the barometric altitude
update (baro_alt_fuse_baro) shall be fused at its nominal variance
regardless of the innovation size, skipping the chi2 test entirely.
When set on local_gnss_alt_config_t, the offset measurement update
(local_gnss_alt_update) shall likewise skip its innovation-vs-chi2
comparison and fuse at the nominal combined variance R. In both cases
nothing is ever downweighted while the flag is set.

## REQ-BARO-017 — Downweight diagnostic counters

- **Status:** verified
- **Parent:** REQ-SYS-006
- **Verification:** Test: tests/test_baro.c:scenario_chi2_disable

baro_alt_t and local_gnss_alt_t shall each expose a monotonic counter,
n_downweighted, incremented once per fusion in which the chi2 test
tripped and was downweighted (exact counts: both are single scalar
measurements, not a multi-row batch). Diagnostic only (does not affect
the fusion); stays 0 whenever REQ-BARO-016's chi2_disable override is
set.

## REQ-BARO-018 — Offset datum relocation

- **Status:** deleted
- **Parent:** REQ-BARO-010
- **Verification:** Inspection: obsolete -- local_gnss_alt_shift_datum was removed, the vertical datum no longer moves (REQ-SUITE-007) so the offset filter never needs relocating.

Superseded. The vertical datum is now fixed once by the first source
and never relocated: a later filter conforms to it at its own
initialization instead (REQ-SUITE-007). With the datum immovable, the
offset filter's datum never moves, so the relocation primitive
(local_gnss_alt_shift_datum) was removed. Kept as a deleted ID for
stability; never reuse.

## REQ-BARO-020 — baro_alt datum relocation

- **Status:** deleted
- **Parent:** REQ-BARO-009
- **Verification:** Inspection: obsolete -- baro_alt_shift_datum was removed, the barometer no longer yields its datum to a late filter (REQ-SUITE-007).

Superseded, same reason as REQ-BARO-018. The barometer no longer
yields its datum to a late-arriving filter (baro_alt_shift_datum
removed); the late filter conforms instead (REQ-SUITE-007). Kept as a
deleted ID for stability; never reuse.

## REQ-BARO-019 — Barometric altitude accessor

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_isa_conversion

baro_alt shall publish the filtered barometric ISA altitude (the datum
zero point plus the estimated local height) through a dedicated
accessor, so a caller can read the barometric height system directly
instead of reconstructing it from struct internals. The accessor shall
report false while the filter is not initialized/healthy.

## REQ-BARO-021 — Vertical-channel precision restart watchdog

- **Status:** verified
- **Parent:** REQ-SYS-005
- **Verification:** Test: tests/test_baro.c:scenario_baro_precision_restart

baro_alt shall, each post-init epoch and after a configurable warm-up
(baro_alt_config_t.restart_warmup_sec, default 8 s), compare its own
reported height and vertical-velocity 1-sigma (from the covariance
diagonal) against per-state thresholds (restart_h_stddev_m, default
20 m; restart_v_stddev_mps, default 10 m/s; a threshold set < 0 is not
checked). If either checked state exceeds its threshold the estimate is
no longer trustworthy -- e.g. a prolonged barometer outage left the
filter dead-reckoning on the accelerometer until it drifted -- and the
filter shall mark itself uninitialized (is_initialized = false, the same
fail-safe mechanism as the health check) and increment a monotonic
diagnostic counter (n_restart); nav_suite then re-bootstraps the vertical
filter from the live stream, while a standalone caller must re-init. The
warm-up suppresses the check while the filter is still converging from
its initial covariance. The watchdog is on by default and can be disabled
via baro_alt_config_t.precision_restart_disable. (This subsumes an
explicit no-barometer timeout: an outage grows the covariance, which the
watchdog then catches.)

## REQ-BARO-022 — Vertical zero-velocity update, ungated by the velocity estimate

- **Status:** verified
- **Parent:** REQ-BARO-001
- **Verification:** Test: tests/test_baro.c:scenario_baro_zupt

baro_alt shall provide baro_alt_zero_velocity_update(), fusing a scalar
zero-velocity pseudo-measurement z = 0 that observes only the vertical
velocity state (H = [0 1 0]) at a configurable 1-sigma
(baro_alt_config_t.zupt_stddev_mps, default 0.05 m/s, overridable per
call). It is applied when the caller knows the platform is standing
still; under nav_suite that trigger is the zero-rotation update
(REQ-SUITE-015).

The update shall NOT be gated on the filter's own velocity estimate.
That estimate is the very quantity being corrected, so a "check the
velocity first" gate would suppress the update exactly when the
accelerometer-only dead reckoning has drifted furthest and needs it
most. The caller's stillness detection is the authority.

Besides bounding the velocity drift, the update shall pull on the
acceleration correction a_b through the v/a_b covariance coupling built
up by the prediction, so a standstill observes a_b even while the
barometer is out. Implausible innovations shall be chi2-DOWNWEIGHTED,
not dropped (the same policy as the barometer fusion, REQ-BARO-017), and
the call shall be a no-op on an uninitialized filter. Applied updates
shall be counted in a monotonic diagnostic counter (n_zupt).

## REQ-BARO-023 — Vertical-channel dead-reckoning duration accessor

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_baro_deadreckoning_ms

baro_alt shall track the time since the last accepted barometer fusion
(a downweighted fusion counts as accepted, per REQ-BARO-017; only an
outright kalman_udu failure withholds it) and expose it through
baro_alt_deadreckoning_ms(), mirroring ins_deadreckoning_ms()
(REQ-NAV-022) for the vertical channel. This lets a caller distinguish
"height still barometer-aided" from "riding purely on the
accelerometer" independently of the full 3D filter's own
dead-reckoning state, e.g. for a diagnostic display or telemetry field.
The accessor shall report -1 while the filter is not initialized and
otherwise never a negative value (a backwards time step is clamped to
0).

## REQ-BARO-024 — Initial accel-bias-prior consistency check

- **Status:** verified
- **Parent:** REQ-SYS-016
- **Verification:** Test: tests/test_baro.c:scenario_baro_bias_prior_check

While zero-velocity updates (REQ-BARO-022) keep arriving, the platform
is standing still and the up-acceleration input a_up is the vertical
accelerometer bias itself. baro_alt shall therefore average a_up over
that stillness run and, once the window covers both a minimum duration
and a minimum number of samples, compare |mean a_up| against
3 * cfg.acc_bias_init_stddev_mps2. Exceeding that threshold shall
increment a monotonic diagnostic counter (n_acc_bias_prior_exceeded) and
emit a throttled LOG_WARN. The window shall be restarted after every
evaluation and discarded whenever the zero-velocity feed pauses longer
than a short gap tolerance, since the platform may have moved in the
meantime.

The check shall be purely diagnostic: it shall never alter the filter
state, the covariance or any fusion decision.

Rationale: the vertical-channel counterpart of REQ-NAV-050, see there.
Because baro_alt has no stillness detector of its own (REQ-SUITE-015),
the caller's zero-velocity feed is the only stillness signal available,
and because a_up is already the gravity-removed vertical specific force,
it is directly the quantity the a_b state models -- no separate
projection is needed.

## REQ-BARO-025 — Raw measurement telemetry accessors

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_baro_measurement_accessors

baro_alt shall expose the two raw quantities its own fusion consumes,
independent of the filtered state, through dedicated accessors:
baro_alt_get_measurement_a_z() returns the up-acceleration control
input a_up of the last predict step (gravity removed, BEFORE the a_b
correction from REQ-BARO-002 is added back in), and
baro_alt_get_measurement_h() returns the datum-corrected barometric
altitude measurement z = h_baro - h0 (REQ-BARO-004) last handed to the
barometer fusion. Both shall report false while the filter is not
initialized/healthy or before the first predict/fusion step
respectively. Diagnostic only: plotting either against the
corresponding filtered state (baro_alt_get_acc_bias, baro_alt_get_height)
is what makes convergence behaviour of the vertical channel visible
externally, e.g. on a live telemetry plot.

## REQ-BARO-026 — Public predict/correct API

- **Status:** verified
- **Parent:** REQ-SYS-017
- **Verification:** Test: tests/test_baro.c:scenario_predict_correct_equivalence

baro_alt_predict_step() and baro_alt_correct_step() shall together be
exactly equivalent to baro_alt_update(): baro_alt_update() is
implemented as baro_alt_predict_step() followed by
baro_alt_correct_step(), with no bookkeeping (non-finite/implausibility
checks, backward time-jump re-anchor, gap diagnostics, health/precision
checks) left behind in baro_alt_update() itself that a caller driving
the two halves separately would miss. baro_alt_predict_step() shall
accept an optional phi_out buffer for the BARO_ALT_STATES x
BARO_ALT_STATES state transition matrix used that call, filled only
when the covariance was actually propagated (see
BARO_ALT_EPOCH_COV_PROPAGATED).

The sanitized pressure sample shall be handed off internally from
baro_alt_predict_step() to baro_alt_correct_step() (not re-derived,
since the plausibility decision is not recoverable once t_last has
moved on) -- baro_alt_correct_step() shall be a no-op if the matching
baro_alt_predict_step() dropped the epoch or was never called.
