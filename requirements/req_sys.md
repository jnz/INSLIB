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

## REQ-SYS-018 — Magnetic dip pole exclusion zone

- **Status:** verified
- **Parent:** REQ-SYS-013
- **Verification:** Test: tests/test_ins_math.c:test_wmm_dip_pole_zone

The magnetic model shall report the great-circle distance from a given
position to the nearest magnetic dip pole, and shall expose a predicate
that is false within a fixed radius of any dip pole. Neither the number
of dip poles nor their positions shall be fixed in the source code:
they are measured properties of the field at that epoch, not
invariants, and are tabulated by the generator. The present field has
two, both drifting, the northern one by roughly 33 km per year.

The positions shall be tabulated once at mid-epoch and shall not be
interpolated in time, so the query takes no year. Over a five-year
epoch the poles move about 1.5 deg, putting a fixed mid-epoch position
at most ~0.8 deg from the truth at either end, which is far smaller
than the margin the exclusion radius carries and smaller than the
uncertainty in choosing that radius at all. The generator shall measure
this drift when it regenerates the tables and shall fail rather than
emit positions whose drift exceeds the budgeted margin.

The distance shall be a great-circle distance and not a separation in
latitude and longitude, because a degree of longitude near a dip pole
covers a small fraction of the ground a degree of latitude does (at the
northern pole, 5 deg of longitude is about 46 km against 555 km for
5 deg of latitude).

Rationale: at a dip pole the horizontal field vanishes, so the
declination is ill-conditioned. The grid cannot resolve it (measured
errors reach ~179 deg within a degree of the pole, and the worst case
outside a 7 deg radius is still ~10 deg, at a cost of 0.75 percent of
the Earth's surface) and a magnetometer carries almost no
heading information there either, because the horizontal component it
measures falls below a few percent of the total field. Callers that
aid heading magnetically (REQ-AHRS-014, REQ-NAV-027) need to tell that
region apart from a merely inaccurate one, which a declination value
alone cannot express. Inclination and total field strength stay valid
inside the zone and are not affected by this predicate.

## REQ-SYS-019 — Bounded stack usage

- **Status:** verified
- **Parent:** REQ-SYS-003
- **Verification:** Analysis: make stack (REQ-VER-033) computes the worst case of every public API function and gates it against the budgets in scripts/stack_usage.cfg

The stack usage of every public API function shall have a static upper
bound: no recursion, no variable length arrays or alloca, and every call
through a function pointer shall have a known set of targets. The worst
case of each entry point on the reference target (Cortex-M4F) shall be
known and shall not grow past its budget unnoticed.

Rationale: all filter state lives in caller-provided structs (REQ-SYS-003),
but the Kalman updates keep their scratch matrices on the stack, so the
stack is where the memory the library needs beyond its structs goes. An
integrator sizing the stack of the task that runs the filter needs that
figure, and a stack overflow on a bare-metal target corrupts memory silently
instead of failing.

## REQ-SYS-020 — Closed-form ECEF to geodetic conversion

- **Status:** verified
- **Parent:** REQ-SYS-014
- **Verification:** Test: tests/test_ins_math.c:test_ecef_to_latlonh_closed_form; Test: tests/test_ins_math.c:test_ecef_roundtrip; Test: tests/test_ins_math.c:test_wgs84_constants_consistent

The ECEF -> geodetic conversion of the toolbox (REQ-SYS-014) shall run
as a fixed, non-iterative instruction sequence whose execution time
does not depend on the coordinates handed to it. For ellipsoidal
heights from -1 km to +30 km it shall reproduce the converged geodetic
latitude and height to within 1 mm, and it shall return finite values
for every finite input, including the geographic poles, points on the
polar axis and points inside the ellipsoid.

Rationale: the conversion is an API-boundary operation (REQ-NAV-080):
ins itself no longer runs it per epoch, but a caller whose source is
natively ECEF does run it once per fix before offering it (REQ-NAV-079),
inside its own worst case. A loop whose iteration count depends on the
coordinates is exactly what REQ-SYS-004 rules out for such a caller, and
on a target without a double-precision FPU it was the single most
expensive operation of a GNSS epoch while it still sat there: every
iteration of the previous fixed-count Bowring loop costs a double sin,
cos, sqrt and atan2, about 72 us on the Cortex-M4F reference target at
180 MHz, and the loop ran six of them whether or not they changed the
result. The closed form (Bowring's parametric-latitude formula) trades
an accuracy that degrades with height for that determinism: below a
micrometre up to aircraft altitudes, about 1 mm at 400 km and a few
decimetres at geostationary altitude. That envelope is the one the
filters navigate in, nothing in the library targets orbit.

## REQ-SYS-021 — Local tangent-plane mapping in single precision

- **Status:** verified
- **Parent:** REQ-SYS-014
- **Verification:** Test: tests/test_ins_math.c:test_dned_dlatlonh_precision

The mapping between a local NED displacement and the corresponding
latitude, longitude and height difference shall take and return the
geodetic side in double precision and evaluate the curvature radii in
single precision. Over displacements up to 100 km and latitudes up to
85 degrees it shall stay within 1e-5 relative of the same mapping
evaluated entirely in double precision, and the two directions shall
remain inverses of each other within 1e-6 relative. Toward the poles
|cos(lat)| shall be bounded by INS_POLE_COS_FLOOR, so the longitude
component stays finite and keeps its sign instead of dividing by a
cosine that single precision can round to zero or past it.

The bound on the north component is a few units in the last place of
single precision. The east component is looser because it divides by
cos(lat), whose error comes from rounding the latitude itself and is
therefore amplified by tan(lat): about 6e-8 relative at 45 degrees,
7e-7 at 85 and 3e-5 at 89. Measured worst case over the envelope above:
8e-7 relative, and 8e-8 on the round trip. The mutual-inverse bound is
the tighter of the two because both directions take their radii from
the same evaluation, so whatever the cosine costs cancels between
them.

Rationale: the geodetic side has to be double because the caller forms
it by subtracting two absolute coordinates, where a latitude of about
0.85 rad leaves a single-precision step of 0.38 m and the difference is
metre-scale. That cancellation happens before this mapping is reached.
What is left here is a multiplication by a curvature radius of about
6.4e6 m, where single precision carries 8e-8 relative, i.e. under a
micrometre on a metre-scale residual and under a millimetre over 10 km.
The model error of a tangent-plane mapping at such a distance is orders
of magnitude larger (REQ-NAV-023).

## REQ-SYS-022 — n-frame rate in single precision

- **Status:** verified
- **Parent:** REQ-SYS-004
- **Verification:** Test: tests/test_ins_math.c:test_omega_n_in_precision; Test: tests/test_ins_math.c:test_transport_rate_pole

The n-frame angular rate (Earth rotation plus transport rate) shall take
the latitude and height in double precision, because that is how the
caller holds them, and evaluate entirely in single precision from there.
Over heights to 30 km, speeds to 600 m/s and latitudes to 80 degrees it
shall stay within 1e-9 rad/s of the same formula evaluated entirely in
double. Every finite latitude, including the poles and values handed in
out of range, shall yield a finite and bounded rate (the
INS_POLE_COS_FLOOR of REQ-SYS-021 bounds the azimuth term).

Accuracy is NOT required near the poles. The bound above holds to 80
degrees, is about 3e-8 rad/s at 89 degrees and degrades quickly beyond
it, because the rounding falls on cos(lat) as an absolute error while
cos(lat) itself is going to zero, and tan(lat) amplifies it. This is
accepted: the quantity it corrupts is the azimuth transport rate, which
a north-slaved NED frame makes singular at the pole whatever the
arithmetic. Working there is a question of choosing a different frame,
not of widening a float.

Rationale: both terms are small and well conditioned away from the
poles. The Earth rate is 7.29e-5 rad/s and the transport rate is a
velocity divided by an Earth radius, so single precision carries them to
~1e-10 rad/s, which is 2e-5 deg/h of attitude drift, orders below the
bias stability of any gyroscope these filters run on. The double sine,
cosine and square root this replaced sat in the strapdown step
(REQ-NAV-020), i.e. once per IMU sample, and on a target without a
double-precision FPU they were the most expensive operation there.
