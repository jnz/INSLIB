# System requirements (REQ-SYS)

## REQ-SYS-001 — 3D navigation solution

- **Status:** verified
- **Verification:** Test: tests/test_ins_core.c:scenario_gnss_position; Test: tools/replay.c:main

The system shall estimate position, velocity and attitude of a rigid
body in 3D from inertial measurements (accelerometer, gyroscope),
aided by any combination of GNSS position/velocity, magnetometer,
local position references, absolute yaw references and zero-velocity/
zero-rotation information.

## REQ-SYS-002 — Independent attitude references

- **Status:** verified
- **Verification:** Test: tests/test_ahrs.c:scenario_nav_suite

In addition to the main navigation filter, the system shall run two
independent IMU(+magnetometer)-only attitude filters in parallel from
the same measurement stream: a roll/pitch filter with freely
integrated yaw, and a roll/pitch/yaw filter with magnetometer heading
aiding. These serve as cross-check and fallback for the main filter's
attitude.

## REQ-SYS-003 — Static memory allocation

- **Status:** implemented
- **Verification:** Inspection: all state lives in ins_t / ahrs_t / nav_suite_t, no malloc/free in src/

All filter state shall be statically sized and owned by the caller-
provided instance structs. The implementation shall not allocate heap
memory.

## REQ-SYS-004 — Bounded execution time

- **Status:** implemented
- **Verification:** Inspection: per-epoch paths use fixed-size loops, no sorting or input-dependent iteration counts

The per-epoch processing path shall have a bounded worst-case
execution time: no unbounded loops, recursion or algorithms whose
iteration count depends on measurement values (e.g. sorting).
Rationale: this is why the ZARU noise suppression uses a running mean
(O(1)) instead of a median (buffer + selection).

## REQ-SYS-005 — Fail-safe outputs

- **Status:** verified
- **Verification:** Test: tests/test_ins_core.c:scenario_corrupted_covariance; Test: tests/test_ahrs.c:scenario_corrupted_covariance; Test: tests/test_baro.c:scenario_corrupted_covariance; Inspection: ins_check_health / ahrs_check_health run every epoch, accessors gate on is_initialized

The system shall never publish a non-finite navigation solution. If
the internal state becomes non-finite despite input protection, the
affected filter shall flag itself as not ready instead of returning
corrupt values. This includes corruption of the covariance factors
(U, d) themselves -- negative, non-finite or inconsistent entries,
e.g. from a memory fault or a numerical breakdown: running the
per-epoch prediction/fusion routines on such factors shall not crash,
and within the same epoch the filter shall either flag itself unready
(health check) or keep publishing finite values; re-initialisation of
the same instance shall fully restore normal operation.

## REQ-SYS-006 — Robustness against measurement outliers

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_mag_gating; Test: tests/test_ins_core.c:scenario_gnss_local_pos_gating

Aiding measurements shall be screened statistically (chi-square test
on the innovation). Implausible measurements shall not corrupt the
state estimate; persistent absolute references (GNSS, yaw,
magnetometer) shall be downweighted rather than rejected so the filter
cannot deadlock on a persistent offset.

## REQ-SYS-007 — Robustness against non-finite inputs

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_nan_inf_inputs; Test: tests/test_ahrs.c:scenario_nan_inputs

Measurement inputs containing non-finite values (NaN/Inf) shall be
dropped at the API boundary and counted in the diagnostics. A single
corrupt sample shall not degrade or disable the filters; processing
shall continue with the next valid data.

## REQ-SYS-008 — Portability

- **Status:** implemented
- **Verification:** Demonstration: Makefile builds with gcc -std=c11 on Linux and Windows (mingw32-make)

The implementation shall be portable C11 without OS-specific
dependencies; the test suite shall build and run on POSIX and Windows.

## REQ-SYS-010 — Graceful degradation during position-aiding outages

- **Status:** verified
- **Parent:** REQ-SYS-001
- **Verification:** Test: tests/test_ins_core.c:scenario_deadreckoning_reacquire; Test: tests/test_ahrs.c:scenario_tunnel

The system shall survive a temporary loss of all absolute position
aiding (e.g. a tunnel passage): it shall coast on inertial dead
reckoning for as long as the IMU-only position remains usable
(configurable window, ~10 s default), then degrade to attitude-only
output from the AHRS filters while flagging the position solution as
unavailable, and shall recover to full navigation automatically with
the first usable fix after the outage.

## REQ-SYS-011 — Barometric vertical channel reference

- **Status:** verified
- **Verification:** Test: tests/test_baro.c:scenario_baro_convergence; Test: tests/test_baro.c:scenario_baro_suite

When barometric pressure measurements are available, the system shall
run an independent vertical channel filter in parallel to the main
navigation filter and the attitude references, estimating height
above start, vertical velocity and a vertical acceleration correction
from barometer and accelerometer only. It serves as cross-check and
fallback for the main filter's vertical solution (e.g. during GNSS
outages).

## REQ-SYS-012 — Consistent vertical datum and absolute height

- **Status:** verified
- **Parent:** REQ-SYS-011
- **Verification:** Test: tests/test_baro.c:scenario_height_strategy

All parallel filters shall express their local height in a common
vertical datum: the height above the NED origin of the main
navigation filter. A local height shall be available whenever any
local height source (barometer, GNSS, or local position aiding such
as a lighthouse system) is running, and shall be zero at the datum
origin regardless of which source started first.

When GNSS is available alongside a local height source, the system
shall additionally estimate the slowly varying offset between the
local vertical datum and the GNSS ellipsoid -- i.e. the ellipsoid
height of the datum origin -- so that an absolute (ellipsoid) height
remains available from the local height source during GNSS outages.
The offset shall be estimated against the local height source that
survives a GNSS outage, not against a GNSS-derived local height, so
that drift of that source against the ellipsoid (e.g. weather-induced
barometric drift) is absorbed by the offset. The system shall report
whether an absolute height is currently available.

## REQ-SYS-009 — Regression testing

- **Status:** verified
- **Verification:** Test: tests/test_ins_core.c:main; Test: tests/test_ahrs.c:main; Test: tools/replay.c:main

Every requirement-relevant behaviour shall be covered by automated
regression tests: synthetic unit/integration tests (`make test`) and
replay of real sensor data against a survey-grade reference
(`make datasets`), both failing via exit code on violation.

## REQ-SYS-013 — World Magnetic Model

- **Status:** verified
- **Verification:** Test: tests/test_ins_math.c:test_wmm_model

The system shall provide a World Magnetic Model look-up giving, for a
geographic position and decimal year, the magnetic declination,
inclination and total field strength, and shall assemble these into a
full NED reference field vector. Declination is interpolated
bilinearly in space and linearly in time between two model epochs;
the assembled reference field shall be self-consistent (its horizontal
direction equals the declination, its magnitude equals the total
field). The interpolated declination shall match the underlying WMM
spherical-harmonics evaluation to within the grid resolution
(~0.6 deg away from the geomagnetic poles). Rationale: this supplies
the magnetic reference and true-north correction for the AHRS
(REQ-AHRS-014) and ins (REQ-NAV-027) heading aiding without an
online spherical-harmonics evaluation.

## REQ-SYS-014 — Geodetic and quaternion math toolbox

- **Status:** verified
- **Verification:** Test: tests/test_ins_math.c:test_ecef_roundtrip; Test: tests/test_ins_math.c:test_rpy_roundtrip; Test: tests/test_ins_math.c:test_matrix_to_quat_roundtrip; Test: tests/test_ins_math.c:test_rotation_rate_known; Test: tests/test_ins_math.c:test_transport_rate_pole; Test: tests/test_ins_math.c:test_gravity; Test: tests/test_ins_math.c:test_quat_normalize_degenerate; Test: tests/test_ins_math.c:test_rpy_gimbal_lock

The system shall provide a reusable geodetic/quaternion toolbox
(WGS84 ECEF <-> lat/lon/height conversions, NED frame rotations,
normal gravity model, Earth/transport rate, Hamilton quaternion
algebra with Tait-Bryan ZYX Euler mapping) whose round-trip
conversions are consistent within single-precision tolerances and
whose degenerate inputs (zero quaternion, gimbal lock, poles) yield
defined, finite results. In particular the NED transport rate carries
tan(lat), which is singular at the geographic poles; the toolbox shall
bound |cos(lat)| (INS_POLE_COS_FLOOR) so the azimuth transport rate
stays finite and bounded there instead of diverging. Rationale: all
filters build on this layer; its correctness is verified explicitly
instead of only implicitly through the filter scenarios.

## REQ-SYS-015 — Global outlier-rejection override

- **Status:** verified
- **Verification:** Inspection: ins_options_t.chi2_disable is the single flag propagated by nav_suite_init() to every sub-filter config (REQ-SUITE-011) -- see REQ-NAV-035, REQ-AHRS-018, REQ-BARO-016 for the per-filter effect and REQ-VER-011 for the config.yaml surface.

The system shall provide one configuration switch that, when set,
disables chi2-based outlier downweighting across every fusion filter
(ins, both AHRS instances, baro_alt and the baro-GNSS offset filter):
every measurement that would normally be chi2-tested is instead fused
at its nominal variance regardless of the innovation size, nothing is
ever downweighted. This is a diagnostics/analysis tool (e.g. to check
whether a filter's behaviour is chi2-driven on a real dataset), not a
tuning parameter for normal operation -- off by default. A single flag
on ins_options_t reaches every sub-filter automatically when run
through nav_suite; standalone INSLIB/ahrs/baro_alt users set their own
config's chi2_disable field directly.

## REQ-SYS-016 — Optional debug logging facility

- **Status:** implemented
- **Verification:** Test: tests/test_log.c:scenario_runtime_filter; Test: tests/test_log.c:scenario_clamp; Test: tests/test_log.c:scenario_formatting; Test: tests/test_log.c:scenario_sink_redirect; Inspection: src/log.h's LOG_LEVEL threshold guards each LOG_* macro with a preprocessor #if, so a level above the compiled-in ceiling expands to ((void)0) and never emits a call to log_write -- not automatically testable from a single build, reviewed by inspection.

The system shall provide an optional, four-level (INFO/WARN/ERROR/
FATAL) debug logging facility usable from any source file without
threading extra state through existing function signatures. A
compile-time threshold (LOG_LEVEL, project-wide via the build or
overridable per source file) shall gate which severities are compiled
into a call at all, down to fully disabled (LOG_LEVEL_NONE, no
log_write reference emitted, zero code size/call cost). A runtime
threshold shall additionally allow narrowing (or widening back up to
the compiled ceiling) which of the compiled-in severities actually
reach a sink without a rebuild. Output shall default to stdout and
shall be redirectable to a caller-supplied sink callback receiving the
severity, call-site file/line and printf-style format/arguments, so
targets without stdout (or wanting a UART/RTT/ring-buffer destination)
can retarget it. Rationale: this is diagnostic infrastructure only --
it carries no filter state, is not on the WCET-bounded hot path of any
filter here (REQ-SYS-004), and code is not yet instrumented with it
(follow-up work); this requirement covers the facility itself.

## REQ-SYS-017 — Public predict/correct split for offline post-processing

- **Status:** verified
- **Verification:** Test: tests/test_ahrs.c:scenario_suite_predict_correct_equivalence

Each filter that exposes an update() entry point combining time
propagation and measurement fusion in one call (ins, ahrs, nav_suite)
shall also expose the two halves as separate public functions,
predict_step() and correct_step(), such that update() is exactly
predict_step() followed by correct_step() with no other behavioral
difference. A caller that needs the covariance both before and after
fusion (e.g. an offline RTS smoother building P(k|k-1) and P(k|k) per
epoch from a forward pass) can then call the two halves separately and
sample the existing covariance accessors in between, instead of only
ever seeing the combined post-fusion result.

predict_step() shall additionally accept an optional (nullable) output
buffer for the discrete-time state transition matrix Phi used that
call, filled only when the covariance was actually propagated that
epoch (the covariance predict is throttled independently of the
predict_step() call rate). No new library-side storage of Phi or a
dense covariance history is introduced: the existing UDU-factor state
already carries enough information for a caller to reconstruct P at
any epoch it chooses to sample, so this requirement only concerns the
callable seam, not new persisted data (keeps REQ-SYS-003's static/no-heap
constraint unaffected: phi_out is caller-provided, like every other
output buffer in these APIs).

Passing NULL for phi_out and calling only update() (never predict_step/
correct_step directly) shall reproduce every filter's existing
behavior unchanged -- this is a pure API addition, not a semantic
change to any existing entry point.
