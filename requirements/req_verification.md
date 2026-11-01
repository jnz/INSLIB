# Verification environment requirements (REQ-VER)

## REQ-VER-001 — Automated unit/integration tests

- **Status:** verified
- **Parent:** REQ-SYS-009
- **Verification:** Demonstration: make test builds and runs test_core, test_math, test_ahrs, test_baro with non-zero exit on failure

All synthetic tests shall build and run via `make test` on a plain
C toolchain (no test framework dependency) and report failures via
exit code and human-readable output.

## REQ-VER-002 — Real-world replay tests

- **Status:** deleted
- **Parent:** REQ-SYS-009
- **Verification:** Inspection: obsolete

(Deleted, kept for ID stability.)

## REQ-VER-003 — Dataset-neutral replay format

- **Status:** implemented
- **Parent:** REQ-VER-002
- **Verification:** Inspection: datasets/replay_format.py defines the shared config.yaml + imu/ref/gnss/mag/baro/speed/heading/ranges/timesync.csv contract (schema documented in doc/INSLIB_manual.tex section "config.yaml", dataset specifics confined to the convert_*.py converters)

A single replay harness (one binary, `tools/replay.c`; mirrored by
`tools/replay.py`) shall consume a dataset-neutral input format: a
generated per-dataset `config.yaml` (aiding/init mode, IMU noise
model, lever arms, warmup, regression limits, GNSS covariance
fallbacks) plus CSVs — imu.csv (FRD body-frame IMU, optionally
carrying the IMU die temperature [degC] as a trailing eighth column,
which both harnesses ignore: they read the first seven fields, so a
dataset written without it stays valid), ref.csv
(position, attitude, NED velocity), optionally gnss.csv (real GNSS
measurements with the full NED position and velocity covariance;
unknown entries zero; optionally followed by the fix quality in RTKLIB's
numbering, the receiver's GPS week and time of week, the satellites
used and the receiver's jamming, spoofing and authentication states,
REQ-VER-045, which the filter does not read, so a gnss.csv without them
stays valid), timesync.csv (the recording's t_us against GPS time, read by
tools/replay.py only, REQ-VER-044), mag.csv (calibrated body-frame field [uT]),
baro.csv (static pressure [Pa]) and speed.csv (scalar ground speed
[m/s], REQ-NAV-068 -- its per-sample uncertainty and delay come from
config.yaml as constants, not from further columns) and heading.csv
(dual-antenna baseline azimuth [deg] with its per-row 1-sigma [deg] and
carrier-phase solution, REQ-VER-035) and ranges.csv (ranges to anchors
at known positions with a per-row 1-sigma, REQ-VER-038). Dataset
specifics (axis conventions,
units, lever arms, sensor calibration, noise, gates) shall be confined
to the per-dataset converters, so new datasets only add a converter
and a Makefile sub-target.

Config `aiding:` shall support three modes: `gnss` (real per-epoch
covariance from gnss.csv), `ref` (fix synthesized from ref.csv, NOT an
independent error profile) and `none` (no absolute position aiding at
all -- ins stays uninitialized for the whole replay; the ARS and
baro_alt filters, which don't need one, keep running regardless). An
unrecognized `aiding:` value shall be rejected (non-zero exit) rather
than silently treated as one of the known modes.

## REQ-VER-004 — Coverage reporting

- **Status:** implemented
- **Parent:** REQ-SYS-009
- **Verification:** Demonstration: make coverage renders an lcov HTML report incl. branch coverage

Statement and branch coverage of the filter sources shall be
measurable via `make coverage` (gcov/lcov).

## REQ-VER-005 — Requirements traceability check

- **Status:** implemented
- **Parent:** REQ-SYS-009
- **Verification:** Demonstration: make reqs runs requirements/check_reqs.py

The requirements database shall be machine-checked: unique IDs,
mandatory fields, valid status values, existing parent references and
existing test functions behind every `Test:` verification entry.
`make reqs` shall fail on violations and report requirements with
open verification.

## REQ-VER-006 — Replay with real GNSS measurements

- **Status:** deleted
- **Parent:** REQ-VER-002
- **Verification:** Inspection: obsolete -- depended on the UrbanNav-HK-Medium-Urban-1 dataset and the optional RTKLIB submodule (rnx2rtkp post-processing), both dropped (REQ-VER-002) for maintenance overhead. datasets/fog and datasets/kfgins already replay real GNSS receiver measurements (gnss.csv), but neither derives that aiding from independent raw-observation post-processing the way this requirement specified.

(Deleted, kept for ID stability.) Previously required the GNSS aiding of
the real-world replay (REQ-VER-002) to come from actual GNSS receiver
measurements, independent of the ground-truth reference: the
UrbanNav-HK-Medium-Urban-1 trial's own u-blox F9P RINEX observations,
processed with RTKLIB (rnx2rtkp, differential vs. the HKSC reference
station) into per-epoch positions with the receiver's covariance -- a
real deep-urban-canyon error profile (multipath, NLOS, mostly
float/DGPS solutions). Velocity aiding, if used, was derived from the
GNSS positions only (differencing), never from the reference. The
harness applied the surveyed GNSS antenna lever arm, scored ins
attitude and position errors against the SPAN-CPT reference and failed
via exit code on regression-gate violations.

Rationale (historical): an independent, real GNSS error process (with
its own multipath/NLOS outliers and honest per-epoch covariance)
exercises the fusion and outlier handling in a way a reference-derived
pseudo-GNSS fix cannot. Deriving the aiding from the trial's raw RINEX
via RTKLIB kept it fully independent of the SPAN-CPT reference used for
scoring.

## REQ-VER-007 — Sanitizer test run

- **Status:** verified
- **Parent:** REQ-SYS-009
- **Verification:** Demonstration: make test-asan builds and runs all four test binaries under AddressSanitizer + UndefinedBehaviorSanitizer with non-recoverable findings

All unit/integration test binaries shall additionally build and run
under AddressSanitizer and UndefinedBehaviorSanitizer
(`make test-asan`, POSIX/gcc only), turning memory errors and
undefined behaviour in the exercised paths into hard test failures.
Rationale: the fail-safe scenarios (REQ-SYS-005) deliberately run the
filter math on corrupted covariance factors; "does not crash" is only
a strong claim if out-of-bounds accesses and UB are detected rather
than silently tolerated.

## REQ-VER-008 — Configurable assumed GNSS delay

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall accept a config
`gnss: delay_ms` (default 0) applied to ins's existing gnss_delay_ms
history-anchoring (ins.c) uniformly on every fix, regardless of aiding
source (gnss or ref). Rationale: the harness currently assumes every fix
arrives with zero latency; this models both a real-time receiver's fixed
processing latency (backdating the fusion correctly, the delay-compensation
infrastructure already exists in ins.c) and a post-processing clock
offset between two independently-recorded logs (e.g. fusing a
separately-captured GNSS log against an IMU log with its own time base;
the value can also be found empirically from the data rather than guessed).

## REQ-VER-009 — GNSS delay estimation from baro_alt cross-correlation

- **Status:** deleted
- **Parent:** REQ-VER-008
- **Verification:** Inspection: obsolete -- tools/replay.py analysis tooling (--estimate-gnss-delay) is not tracked as a requirement (the DB covers the src/ C library and the C regression harness only).

(Deleted, kept for ID stability.) Previously required tools/replay.py to
cross-correlate the baro/accel vertical
filter's down-velocity (assumed near-zero latency) against the held
GNSS fix's down-velocity, over the --plot-hz recorder's uniformly
sampled time series, and report the lag that maximizes the normalized
correlation as an estimated GNSS delay in milliseconds, together with
the correlation value so a low-quality (ambiguous/flat) result -- e.g.
from a trial with little vertical motion -- is distinguishable from a
confident one. This shall run automatically whenever it is meaningful
(aiding: gnss with usable velocity on at least one fix, and a
barometer available) without requiring an explicit flag, since a
ref-synthesized fix would just trivially self-correlate at ~0 ms;
--estimate-gnss-delay shall force it on for other cases. Rationale:
gives a way to find a value for `gnss: delay_ms` (REQ-VER-008) from
the data itself instead of guessing it.

CAVEAT (documented, not resolved): "assumed near-zero latency" is an
approximation -- baro_alt is a Kalman-filtered estimate, not a raw
sensor, and its own group delay is not necessarily zero. The reported
number is therefore the delay of GNSS RELATIVE TO baro_alt, not a
validated measurement of GNSS latency in isolation.

## REQ-VER-010 — Configurable initial-state uncertainty

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall accept a config
`init_stddev:` section overriding ins_init_t's initial-state uncertainty
fields, applying regardless
of `init:` mode (ins's shared finalize step, ins_finalize_init()
in ins.c, sets the initial covariance from these fields the same way
for both init: ref and init: auto -- only the initial state itself,
not its uncertainty, comes from the auto-init window in that mode).
Key names mirror ins_init_t's own fields exactly (same
convention as python/examples/runner.yaml's `filter:` section, which
accepts any ins.Config field directly), not an abbreviated scheme of
their own: pos_init_stddev_m, vel_init_stddev_mps,
rpy_init_stddev_rad_deg, yaw_init_stddev_rad_deg (0 -> falls back to
rpy_init_stddev_rad_deg, like ins's own rpy_init_stddev_rad[0..1]/[2]),
acc_bias_init_stddev_mps2, gyr_bias_init_stddev_rps_deg -- each 0/omitted
-> the harness's built-in default; the same YAML vocabulary is shared
verbatim by tools/replay.py, which reads the same config.yaml files.
Rationale: the built-in defaults
assume a well-characterized initial state (e.g. a surveyed stationary
start); a dataset whose "known" state is itself only a GNSS-derived
estimate with no independent truth system is otherwise silently
over-trusted -- yaw especially, since a GNSS course-over-ground or
compass heading is typically far less certain than roll/pitch from
accelerometer leveling.

## REQ-VER-011 — Configurable global chi2 override

- **Status:** verified
- **Parent:** REQ-SYS-015
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall accept a top-level
`chi2_disable` (0/1, default 0) config key and forward it to
ins_options_t.chi2_disable
before nav_suite_init(), so a real dataset can be replayed with all
chi2-based outlier downweighting disabled (REQ-SYS-015) for
diagnostics/analysis without touching the source. Off by default --
normal chi2-downweighted operation unless explicitly requested.

## REQ-VER-012 — Downweight statistics in replay tooling

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall print the per-filter
chi2-downweight counters (REQ-NAV-036, REQ-AHRS-019, REQ-BARO-017: ins,
ars, ahrs, baro_alt, local_gnss offset) in its end-of-run summary, so an
outlier-heavy replay is visible without instrumenting the source.

## REQ-VER-013 — Outlier-rejection time series in --plot

- **Status:** deleted
- **Parent:** REQ-VER-012
- **Verification:** Inspection: obsolete -- tools/replay.py `--plot` visualization is not tracked as a requirement (python-tool feature).

(Deleted, kept for ID stability.) Previously required tools/replay.py's
`--plot` output to include a page plotting the
cumulative chi2-downweight counters (REQ-VER-012: INSLIB/full3d, ars,
ahrs, baro_alt, local_gnss offset) over time, one line per sub-filter,
each shown only if that sub-filter was ever active in the replay. A
step increase pinpoints WHEN an outlier was downweighted, not just how
many occurred in total (the end-of-run printout), so outlier-heavy
stretches can be correlated by eye with other pages (e.g. a GNSS
multipath patch, a ZUPT-shaded stop).

## REQ-VER-014 — Sensor sampling-rate time series in --plot

- **Status:** deleted
- **Parent:** REQ-VER-002
- **Verification:** Inspection: obsolete -- tools/replay.py `--plot` visualization is not tracked as a requirement (python-tool feature).

(Deleted, kept for ID stability.) Previously required tools/replay.py's
`--plot` output to include a page plotting each
input stream's (IMU, and GNSS/mag/baro when present) sampling rate
[Hz] over time, bucketed into fixed-width (`--plot-rate-bucket-sec`,
default 5 s) windows -- a time-resolved complement to the scalar
avg-Hz/max-gap numbers already in the data-quality summary (see
`_stream_gap_stats`/`_imu_prepass`), so a rate drop or dropout can be
located and correlated against the other pages instead of only
appearing as one aggregate number. The IMU stream, which can be too
large to hold in memory as a raw timestamp list, shall be bucketed in
the single existing streaming pass (`_imu_prepass`) rather than a
separate one.

## REQ-VER-015 — Magnetometer hard-iron bias (18-state) support in tools/replay.py

- **Status:** deleted
- **Parent:** REQ-NAV-029
- **Verification:** Inspection: obsolete -- tools/replay.py config forwarding + `--plot` visualization is not tracked as a requirement (the underlying C-library 18-state mag hard-iron bias support is REQ-NAV-029).

(Deleted, kept for ID stability.) Previously required tools/replay.py to
accept a `mag: estimate_bias` (0/1, default 0)
config key and forward it to `Config.estimate_mag_bias`, so a real
dataset can be replayed in ins's 18-state magnetometer hard-iron
bias mode (REQ-NAV-029) without touching the source. When active, the
`--plot` bias page shall additionally plot the estimated bias with its
1-sigma band (same convention as the existing acc/gyro bias rows),
sourced from `Navigator.bias_mag()`/`Navigator.stddev()['mag_bias']`
(already exposed generically by nav_suite's C API, REQ-SUITE-*)
without any C-side change.

## REQ-VER-016 — Simulated Groves-profile regression datasets

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main; Test: datasets/check_simulated.py:main

The suite shall carry committed, deterministic synthetic datasets under
`datasets/simulated/` (generated from Paul Groves' book profiles by
`Export_Demo_4.m`: a ground-vehicle "car" and a 200 m/s "aircraft"),
each a full replay bundle (config.yaml + imu/gnss/ref.csv) plus the
book's own loosely-coupled Kalman-filter solution `ref_groves_kf_sol.csv`
as a second reference. Unlike the real-world trial (REQ-VER-002/006,
fetched) these are committed and need no download. `make simulated` shall
gate BOTH harnesses on both datasets and fail via exit code on any
regression: the C harness (`tools/replay.c:main`) scores ins and
the ARS/AHRS sub-filters against the true reference; the Python harness
(`datasets/check_simulated.py:main`) re-scores ins through the ctypes
binding AND additionally requires ins's position RMS to stay within a
configured factor (`score: lim_groves_pos_rms_factor`) of the Groves
textbook filter's own position RMS vs. truth. That harness drives the
analysis tool `tools/replay.py` (via its `--summary-json` output) and
owns the pass/fail gates itself, so `replay.py` stays gate-free. This
exercises the whole navigation stack on a known-truth signal end to end
and pins agreement with an independent textbook filter.

`make simulated` shall additionally gate the C harness alone on
`datasets/simulated/A_ideal`, a noise-free flight whose config opts out of
the auto-ZUPT/ZARU (its slow phases have zero sample variance and would
otherwise read as a standstill, pinning the velocity and absorbing the turn
rate into the gyro bias) and carries tight `score:` limits on ins attitude
and position, the height channels and the ARS, so such a fault fails the
run instead of passing unnoticed, and on both configs of
`datasets/simulated/B_drone`, a UAV flight with a GNSS outage. Its own
config.yaml keeps the whole outage inside the coasting window on purpose,
starts from the reference and carries `score:` limits on ins attitude,
position, the height channels and the ARS/AHRS sub-filters.
`datasets/simulated/B_drone/config_coasting.yaml` is a second config over
the same flight that sets the coasting window shorter than the outage, so
only this one drives the filter
across the boundary: inert while the window is expired (REQ-NAV-064),
re-anchored on the first fix afterwards with the carried states inflated
(REQ-NAV-065) and the height taken from the barometer (REQ-NAV-066). The
Python harness is not run on it: its benchmark is the Groves filter,
which the two profile datasets are built for and this flight is not.

The C harness shall likewise gate `datasets/simulated/E_gnss_delay`, a
drone climb/circle/descent whose 2 Hz GNSS fixes arrive 500 ms after their
time of validity, so the history-anchored delayed fusion is scored against
truth. Its config opts out of the auto-ZUPT/ZARU for the same reason as
A_ideal (the vibration-free IMU of the initial steady climb reads as a
standstill before the first fix arrives) and carries `score:` limits on ins
attitude, position, the ellipsoid height and the ARS.

## REQ-VER-017 — Configurable auto-init leveling window

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall accept a top-level
`auto_init_window_sec` config key (default 0 -> ins's own built-in
default) and forward it to ins_options_t.auto_init_window_sec
(REQ-NAV-015) before nav_suite_init(). Rationale: the built-in window
is sized for a reasonably fast IMU; at a low sample rate it can span
fewer than the handful of samples the leveling median needs, so the
auto-init bootstrap silently never fires and `init: auto` waits
forever for a fix that never triggers it -- a per-dataset override
lets a slow-IMU trial widen the window to actually collect enough
samples.

## REQ-VER-018 — Configurable ARS/AHRS initial gyro-bias uncertainty

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The C replay harness (tools/replay.c) shall accept an `ahrs:
gyr_bias_init_stddev_rps_deg` config key (default 0 -> unchanged: the
`gyro_bias_window_sec` seed's own stddev if that window found a parked
phase, else ahrs.c's built-in default) and, when set, apply it to all
3 axes of both `ars_cfg.gyr_bias_init_stddev_rps` and
`ahrs_cfg.gyr_bias_init_stddev_rps` (nav_suite.h), independent of
whether `gyro_bias_window_sec` found a parked phase to seed the mean
from. tools/replay.py shall accept the same `ahrs:
gyr_bias_init_stddev_rps_deg` key (schema shared with the C harness)
and apply it via `Navigator.set_ahrs_gyr_bias_init_stddev()`
(python/csrc's `ins_suite_set_ahrs_gyr_bias_init_stddev()`, which
already existed but was unwired from any config key before this
requirement). Rationale: a trial with no parked phase at all (already
moving at t=0, e.g. an aircraft in cruise) leaves the ARS's/AHRS's
initial gyro bias at [0,0,0] with ahrs.c's generous 1/1/1.5 deg/s
xy/z default uncertainty; for a bias-free (or otherwise
well-characterized) IMU model that default is needlessly loose and
lets a transient measurement mismatch -- e.g. a coordinated turn's
centripetal acceleration briefly fooling the accelerometer leveling,
observed on the `A_ideal` scenario -- get absorbed into the bias
state and then integrate unbounded into the (unobservable, in ARS
mode) yaw. Pinning the initial uncertainty tight and correct instead
keeps that kind of transient a bounded, decaying attitude error
instead of a permanent bias/yaw drift.

## REQ-VER-024 — One stillness config block in the replay harnesses

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main; Test: tests/test_ahrs.c:scenario_suite_stillness_propagation

Both replay harnesses (tools/replay.c and tools/replay.py, schema
shared) shall accept the whole stillness parameter set under `imu:` as
one block and forward it to ins_options_t / ins_init_t before
nav_suite_init(), from where REQ-SUITE-020 distributes it to the
ARS/AHRS and baro_alt:

`zero_vel_stddev_mps`, `zero_rot_stddev_deg`,
`auto_zupt_static_gyr_deg`, `auto_zupt_static_acc_mps2`,
`auto_zupt_static_gyr_stddev_deg`, `auto_zupt_static_acc_stddev_mps2`,
`auto_zupt_max_vel_mps`, `auto_zupt_max_vel_stddev_mps`,
`auto_zupt_dwell_sec`, `auto_zupt_min_interval_sec`,
`auto_zupt_disable`, `auto_zupt_velocity_blind_disable`. Each 0 -> the
library's own built-in default.

Neither harness shall configure the ARS/AHRS stillness gates on its own
(no per-template `auto_zaru_*` assignment, no `set_auto_zaru()` runtime
override at startup): doing so re-creates the divergence REQ-SUITE-020
exists to prevent, and would silently re-arm a dataset that opted out
via `auto_zupt_velocity_blind_disable`.

Rationale: before this, the C harness forwarded only part of the set and
the Python harness only two of the fields, with the rest silently
dropped on the way through the ctypes config struct -- a dataset's
config.yaml and the thresholds the filters actually ran with could
differ, invisibly, in both directions.

## REQ-VER-025 — Unknown config keys are an error

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

Both replay harnesses shall reject a config.yaml containing a key they
do not know, naming the offending key, instead of ignoring it. A
silently ignored key is indistinguishable from a key that had no
effect: a mistyped or renamed tuning parameter otherwise produces a
full run, a score and a plot computed with a configuration nobody asked
for. This applies to top-level keys and to keys inside a known section
alike.

"Known" is the SCHEMA, not this harness's own interpretation: one
dataset directory is read by more than the two replay harnesses, and
the sections/keys belonging to those other consumers are a valid part
of the file. Each harness shall therefore also accept -- and ignore --
the parts of the schema it does not itself consume, and the two
harnesses shall carry the identical list of them. Rejecting them would
force a dataset to choose which of its tools it validates against;
deleting them from the file to satisfy a replay harness would silently
disarm the tool that does read them (e.g. dropping `score:
lim_groves_pos_rms_factor` would turn off check_simulated.py's
comparison against the Groves textbook filter without any harness
saying so). This carve-out covers only whole sections owned elsewhere
and individually named keys -- a typo inside a section either harness
does own stays an error, which is the case the requirement exists for.

A section or key qualifies only once some tool actually reads it.
Config that nothing consumes shall not be carried in the schema to keep
a parser quiet: it is documentation, and belongs in a YAML comment,
where it stays next to the data it describes without any parser having
to know it exists.

## REQ-VER-026 — Tunnel dataset and datasets without an attitude reference

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The suite shall carry a committed real-sensor recording of a road tunnel
passage (`datasets/tunnel/`, a u-blox NAV-PVT/NAV-COV car log trimmed to
roughly three minutes either side of a 94 s total GNSS blackout) and gate
it in `make datasets` and `make test`. It is the only recording here whose
outage is long enough to drive the filter through the whole sequence the
coasting window governs -- expiry into the inert state (REQ-NAV-064), the
returning fixes judged against the stay gate (REQ-NAV-052), and the
re-bootstrap that keeps the n-frame origin (REQ-NAV-062) -- on the fix
quality a receiver actually emits coming out of a tunnel, which is what
decides which of those branches is taken and is the one thing a synthetic
dataset cannot supply.

The replay harnesses shall accept a `score: attitude` flag (default 1)
that, when cleared, suppresses BOTH the reported ins attitude errors and
their pass/fail gates, for datasets whose reference carries no attitude.
Wide limits shall not be used for this purpose: a check reported as passed
against a placeholder reference is indistinguishable from one that means
something, whatever the limit is, whereas a suppressed check states that
the dataset cannot answer the question. The position score of such a
dataset remains gated, with whatever independence its reference has stated
in the dataset's own config.yaml -- for the tunnel it is the GNSS solution
that also supplies the aiding, so the number is self-consistency and is
dominated by the re-acquisition transient rather than by steady-state
tracking.

The blackout itself contributes no scored epochs, since a GNSS-derived
reference cannot cover an interval without GNSS. What the dataset gates is
the state the filter is in once fixes return.

## REQ-VER-027 — Live tool timestamp unwrap: wrap vs. source restart

- **Status:** verified
- **Parent:** REQ-VER-001
- **Verification:** Inspection: tools/insrcv.c:unwrap32 discriminates the two cases by the position of the previous raw value inside the counter range (wrap only when it sat within UNWRAP_WRAP_MARGIN_US of the end of range and the new value is within that margin of zero), stitches a detected restart onto the high-water mark of the emitted timeline and counts it in unwrap32_t.n_restarts, which insrcv publishes as INSLIB/status/src_restarts

The live UBX tool (tools/insrcv.c) reconstructs a monotonic 64-bit
microsecond timeline from the 32-bit counter its sensor source sends.
It shall distinguish a counter overflow from a restart of the source:
an overflow adds one counter range to the high word, whereas a restart
(a backwards step too large to be inter-stream lag, but not originating
from the end of the counter range) shall be stitched onto the previously
emitted timeline instead of being passed on as a backwards jump. Passing
it on would leave the filter dropping every following epoch as too old
(REQ-NAV-070 is the filter-side safety net, at the cost of losing the
converged state). Small backwards steps shall remain untouched: the
streams share one unwrap state deliberately, and the barometer trailing
the IMU by well under a millisecond is normal, not an anomaly.

## REQ-VER-028 — Config list values in both YAML forms

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tests/test_yaml.c:scenario_block_sequence; Test: tests/test_yaml.c:scenario_malformed_list

The C config reader shared by the replay and live harnesses shall read a
list value written as a YAML block sequence

```yaml
gnss:
  leverarm_frd:
  - -0.01
  - 0.04
  - -0.015
```

identically to the inline `[-0.01, 0.04, -0.015]` the converters emit, and
shall reject a list it cannot read in full, naming the offending key,
instead of leaving the destination at whatever it held.

This is REQ-VER-025's rule applied to the value rather than the key. A
dataset directory is read by both `tools/replay.c` (this reader) and
`tools/replay.py` (PyYAML), and PyYAML accepts both forms. A form only
one of them understands therefore does not produce an error anywhere: it
produces two harnesses that disagree about the same file, each convinced
it applied the operator's configuration. `datasets/tunnel/config.yaml` was
in exactly that state, its lever arm reaching one harness and not the
other.

Reading the list in full is part of the same rule. A list shorter than the
destination, or one whose text does not convert, is a mistake in the
config and shall be reported as one. Filling the missing entries with
zeros hands the harness a lever arm, a misalignment matrix or a fixed bias
nobody wrote down, which is the failure REQ-VER-025 exists to prevent.

The reader stays a two-level YAML subset otherwise. A nested mapping is
still unsupported, and needs no separate rule here: its inner keys reach
the harness under the outer section and are refused by REQ-VER-025.

## REQ-VER-029 — Coasting re-acquisition error as a scored metric

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The replay harness shall, for every gap in the position-aiding stream longer
than `score: coast_gap_min_sec`, report the filter's position error at the
first IMU epoch at or after the end of that gap, scored against the
reference epoch nearest to that IMU epoch, together with the gap's duration
and the straight-line distance across it, followed by the mean and maximum
of that error over all gaps that have one, and shall gate that error on
`score: lim_coast_exit_err_m` when one is configured. A reference epoch
further than half a second, or half the gap, from that IMU epoch shall not
be used, and the gap then counts as having no reference epoch.

The sample is tied to the IMU epoch and not to the arrival of a reference
row. With `score: ref_delay_ms` (REQ-VER-030) the reference rows no longer
sit on the fix timestamps: at a 200 ms delay and a 5 Hz receiver the row
describing the end of the gap lands a fraction of a millisecond before or
after it, depending on the receiver's timestamp jitter. Waiting for a row
at or after the gap end then skips that row whenever an IMU sample falls in
between, and scores the next one, 200 ms later, after the returning fix has
already been fused.

A dataset whose reference comes from the same receiver as the aiding cannot
be scored inside such a gap at all: no fixes means no reference. What the
whole-run position RMS does score there is the state the filter is in once
aiding returns, and it scores it perversely. A filter that coasts through
and lands a few hundred metres out contributes those epochs to the RMS; one
that gives the solution up produces no output at the same epochs, re-derives
its position from the returning fix and contributes nothing. On
`datasets/tunnel_coast` the second behaviour scores 0.86 m against the
first's 42.7 m, so a limit on the whole-run RMS rewards discarding the
coasting result. This metric asks the question that matters instead, in the
one place the data can answer it: how far from the truth is the filter when
it comes back out.

Having no solution at that epoch shall count as a failure of the gate, not
as a skipped epoch. Without that rule the metric inherits the same defect it
exists to remove, since a filter that gave up has no error to report.

The gap is a property of the aiding FILE, not of what the filter chose to
fuse: a harness that measured the gap the filter experienced would report a
shorter outage whenever the gate rejected fixes, which is the one case where
the number has to stay comparable across configurations.

The error shall be sampled BEFORE that epoch's measurements are fused. The
returning fix snaps the position onto the truth in the same epoch it arrives,
so a sample taken after the update reports how good the fix was, not how far
the coasting drifted. Against a receiver whose first fixes out of a tunnel
are unusable the difference stays hidden -- they are gated out and nothing
snaps -- and it appears in full against one that comes back clean: on the
Galileo-HAS drive five of eight simulated 100 s outages read 0.0 m that way,
where the coasting had in fact drifted 200 to 550 m.

The first reference epoch after the gap is used as it comes. The fixes a
receiver emits in the first seconds out of a tunnel carry accuracies that
are unusable for aiding (6275 m and 229 m horizontal on the two Wattkopf
recordings), but their POSITION is close enough to score a coasting error of
hundreds of metres against, and `ref.csv` carries no accuracy column that
would let the harness pick a better epoch. Waiting a fixed dwell instead
would score a partly re-converged filter and hide exactly what the metric is
for.

## REQ-VER-030 — Reference epochs carry their own time of validity

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The replay harness shall accept `score: ref_delay_ms`, stating that a
reference row timestamped T is in fact valid at T minus that delay, and shall
score every comparison at the row's true time of validity.

A dataset whose `ref.csv` is derived from the same receiver output that feeds
`gnss.csv` inherits that output's latency. `gnss: delay_ms` tells the FILTER
how old a fix is, so the filter correctly reports where the vehicle is now;
the reference row still describes where it was `delay_ms` ago. Comparing the
two at a shared timestamp therefore penalises the filter by speed times
delay, and penalises it MORE the better it compensates. On
`datasets/tunnel` (delay_ms 200, median speed 19.8 m/s) that product is
3.96 m against a scored position error of mean +3.850 m with a standard
deviation of 1.319 m: the whole reported bias is the artefact, and the
regression limit that gates it is gating a time shift.

Where the reference comes out of the same output as the aiding, the key
shall mirror `gnss: delay_ms`. The score cannot separate the two readings of
the same symptom -- a reference that really is `delay_ms` old, and a filter
that anchors a timely fix `delay_ms` too early -- because both put the filter
`delay_ms` times speed ahead of the reference row. Tying the two keys
together keeps that ambiguity in one place: if the declared latency turns out
to be wrong, both numbers move together, and no run scores well by pairing a
wrong latency with a compensating reference shift.

The key shall stay at 0 wherever the reference is independent of the aiding
even when a delay is configured, which is the case for every other dataset
carrying one: `datasets/crazyflie/*` scores against Lighthouse,
`datasets/simulated/E_gnss_delay` against synthetic truth. It shall also stay
at 0 where a dataset already models the latency in its DATA rather than in
its config: `datasets/crazyflie/motors_off_time_delay_200_ms` shifts the
`gnss.csv` timestamps by +200 ms and leaves `ref.csv` on the true time,
which is the construction this key exists to make unnecessary.

A nonzero `ref_delay_ms` together with `aiding: ref` shall be refused. There
the reference IS the aiding, so a latency belongs in `gnss: delay_ms`, and
shifting the reference would move the measurements with it.

## REQ-VER-031 — Tunnel dataset that coasts through

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The suite shall carry a committed real-sensor recording of a road tunnel
passage the filter survives (`datasets/tunnel_nhc/`, a u-blox X20P
NAV-PVT/NAV-COV car log with Galileo HAS active, across a 97.8 s total GNSS
blackout, 1877 m chord at 15 to 21 m/s, with barometer and magnetometer), and
gate it on the coasting re-acquisition error of REQ-VER-029 in `make
datasets` and `make test`.

This is the branch `datasets/tunnel` does not cover. That recording drives
the coasting window to expiry and the 3D solution to a quality-loss re-arm on
purpose; this one keeps the solution up for the whole outage and lands on the
returning fixes. Both pass the SAME structure, the Wattkopf tunnel near
Ettlingen, but in opposite directions: this one enters at the portal the
other leaves by (66 m and 76 m between the corresponding fixes, chords
1877 m and 1862 m) and descends 17 m where the other climbs 22 m. They are
therefore two passages of one geometry rather than independent evidence
about tunnels. What differs besides the branch and the direction is the
hardware: 10 Hz fixes at 0.45 m reported accuracy over a 500 Hz IMU there,
5 Hz at 0.07 m over an 800 Hz IMU here, block-averaged to 200 Hz for the
repository and trimmed to the stretch from the first fix to 30 s after the
tunnel exit.

It is also the only committed dataset with the non-holonomic lateral
constraint (REQ-NAV-077) enabled, so it is where a regression in that
constraint shows up on real data rather than in a synthetic scenario.

The whole-run position RMS is deliberately NOT gated here. On a dataset with
a real outage it scores the wrong thing: a filter that abandons the coasting
produces no output at the expensive epochs and so lands a better RMS than one
that coasts through. REQ-VER-029's number replaces it.

That gate shall be a regression tripwire, not a plausibility bound: the limit
sits 1 m above the observed exit error, so a change that costs the coast a
metre fails `make test`. This is affordable because the number is
reproducible far below that. The exit errors of this dataset and
`datasets/tunnel_odometry`, and the position RMS of `datasets/tunnel`, agree
to the millimetre across gcc 11 and gcc 15, clang 14, -O0 to -O3, and with
FMA contraction on and off. `datasets/tunnel`, which gives the coasting up by
design, gates its whole-run position and ellipsoid height RMS 0.25 m above the
observed values instead. A limit shall be retightened after an intentional improvement, since
a stale margin hides the next regression of the same size.

The IMU is block-averaged from 800 Hz to 200 Hz: the mean over N samples is
the block's summed delta-theta and delta-v divided by its own dt, so the
integral the strapdown forms is unchanged. On this recording that moves the
tunnel exit error by 0.02 m against the full-rate stream. Plain decimation is
not equivalent and shall not be used: it aliases the vibration.

The recording shall not be trimmed at the front. Its only standstill is at the
very start, and the auto-ZUPTs there are load-bearing: cutting the recording to
300 s or 200 s before the tunnel removes all 194 of them and takes the exit
error from 36.1 m to 42.8 m and 43.4 m. Those three figures were measured
while the metric of REQ-VER-029 still sampled after the returning fix had
been fused. Sampled ahead of it, and at the antenna the reference describes
(score: leverarm_frd, REQ-VER-037), the untrimmed recording exits at 91.5 m,
and at 285.9 m with the lateral constraint disabled.

## REQ-VER-032 — Odometry tunnel dataset

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The suite shall carry a committed real-sensor recording that aids the filter
with a measured wheel speed (REQ-NAV-068) across a total GNSS outage
(`datasets/tunnel_odometry/`, a car log with u-blox X20P NAV-PVT/NAV-COV,
Galileo HAS active, OBD2 PID 0x0D speed frames, barometer and magnetometer,
across a 100.6 s blackout and a 1875 m chord), and gate it on the coasting
re-acquisition error of REQ-VER-029 in `make datasets` and `make test`.

It is the only real-data check of the speed aiding, which is otherwise
verified in a synthetic scenario only. The gate shall therefore be tight
enough to fail when the odometry stops contributing, not only when the
coasting breaks: the exit error, at the antenna the reference describes
(REQ-VER-037), is 22.2 m with the speed aiding and 29.5 m with it disabled.
Like REQ-VER-031 the limit sits 1 m above the observed value (23.2 m).

The speed scale shall be calibrated in the dataset's config, and that
calibration shall not use the scored outage. OBD2 speed on this car reads
about 1.8 percent low against the GNSS horizontal speed, estimated on the
stretch before the tunnel alone. Left at 1.0 the same aiding lands the exit at
54.9 m, worse than no odometry at all, which is a failure the tight gate also
catches.

The tunnel is the Wattkopf tunnel again, driven in the direction of
`datasets/tunnel` (the corresponding fixes lie 27 m and 10 m apart, chords
1875 m and 1862 m), so together with REQ-VER-031 these are three passages of
one structure rather than independent evidence about tunnels. What this
recording adds is the speed channel.

It is cut out of a 45 min drive, from inside the standstill before the tunnel
to 30 s after the tunnel exit, so it keeps the auto-ZUPT phase, and its IMU is
block-averaged from 800 Hz to 200 Hz as in REQ-VER-031.

## REQ-VER-033 — Static worst-case stack usage analysis

- **Status:** verified
- **Parent:** REQ-SYS-019
- **Verification:** Demonstration: make stack (part of make check-all) compiles src/ and KFCore with the host gcc, make stack in embedded/stm32f429 compiles the whole firmware with the target toolchain, both with -fcallgraph-info=su, and both exit non-zero on recursion, VLA/alloca, an unresolved function pointer call, a library call without a stack figure or a budget overrun, the firmware run also when main() plus nested interrupts exceed _Min_Stack_Size

`scripts/stack_usage.py` shall compile the given sources and compute the
worst-case stack of every entry point as its own frame plus the largest worst
case among its callees, from the frame sizes and call edges GCC reports with
`-fcallgraph-info=su`. The call graph is taken after inlining and cloning,
and static functions are qualified with their file, so equal names in
different files do not merge.

It runs in two places:

- `make stack` in the top level Makefile compiles `src/` and KFCore with the
  compiler at hand (`$(CC)`, typically x86_64) and gates the public API
  functions against the x86_64 budgets in `scripts/stack_usage.cfg`. It is
  part of `make check-all` and thereby of the public CI.
- `make stack` in `embedded/stm32f429` compiles every source of the firmware
  with its target toolchain and build flags, reading
  `embedded/stm32f429/stack_usage.cfg` after the library config. It gates the
  library entry points against Cortex-M4F budgets and checks that main() plus
  the worst interrupt handler and exception frame per nesting level fit into
  `_Min_Stack_Size` of the linker script, so the reserved stack is verified
  instead of assumed.

The run shall fail on everything that turns the figure into a guess:

- a cycle in the call graph (recursion),
- a variable length array or alloca, or a frame GCC reports as dynamic
  without a bound,
- a call through a function pointer whose targets are not listed in
  `scripts/stack_usage.cfg`,
- a call into a function that is not compiled by the analysis and has no
  stack figure in `scripts/stack_usage.cfg`,
- a worst case above its budget, or a budget line that matches no function,
- for the firmware, a program stack above the reserved one.

The target's library figures (newlib, libgcc) shall be measured, not
estimated: taken from the disassembly of the toolchain's libraries along
their deepest call chain and rounded up. The host's glibc figures are
generous constants, which is enough for a regression gate on the library's
own frames. A fixed margin covers the compiler generated helper calls some
GCC releases do not record in the call graph. The budgets shall leave enough
room that a different GCC release does not fail the gate, but not so much
that a new large frame on the per-epoch path goes unnoticed.

Not covered: link time optimization, and a user log sink beyond the figure
assumed for it. Function pointer targets are listed by hand in the configs,
as globs where the runtime choice is not visible to the compiler, which
yields an upper bound.

## REQ-VER-034 — Free-inertial round-trip dataset as a scored metric

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main

The suite shall carry a committed, hand-carried, pure-inertial recording
(`datasets/inertial_roundtrip/`, no GNSS, no magnetometer, no barometer,
160.7 s, block-averaged from 797 Hz to 100 Hz IMU data to save repository
storage space) whose unit starts and ends on the same physical mark/socket,
and shall gate the filter's position AND yaw error at the end of the run
against that fact in `make datasets` and `make test`.

A free-inertial run has no ground truth to score against while it moves --
that is the point of running it free-inertial. This dataset carries none
in between, only two declared truth points in `ref.csv`: the start pose,
which doubles as the auto-init anchor (`free_inertial_start`, position
only), and the end pose, identical to the start because the operator put
the unit back where it began. `score: warmup_sec` is set to fall shortly
before the end epoch's timestamp, so the start row is excluded from
scoring (it is the anchor, not a truth to compare against) and the end row
is the only epoch ever scored. The resulting `ins pos rms [m]` and
`ins |yaw bias| [deg]` are therefore not accuracy claims against some
absolute reference frame -- the dataset does not know true north or a
surveyed coordinate any better than the filter does -- they are the
round-trip drift itself: whatever the filter reports at the end that is
not zero is a real error, by construction, without needing a reference
trajectory for the 160 s in between.

Yaw is scored, not just position, because the mount is not rotationally
free: the socket fixes heading as well as position, so a true return
means the same yaw too, not only the same place. Roll and pitch are left
ungated (`lim_att_bias_deg`/`lim_att_std_deg: 999`, `score: attitude`
does not offer a per-axis on/off switch): gravity continuously re-observes
them for the whole run regardless of aiding, so they are not exercising
anything a pure free-inertial regression needs a gate for, and gating them
tightly would only add noise from the initial leveling accuracy to a
dataset about drift.

The limits are set on a single deterministic run (`build/replay.exe`) at
roughly 5% above the observed round-trip error: `lim_pos_rms_m: 0.076`
(observed 0.072 m) and `lim_yaw_bias_deg: 0.216` (observed 0.205 deg), so a
change that costs the free-inertial solution half a centimetre or a
hundredth of a degree fails this. Retighten after an intentional
improvement, the same rule as every other `lim_*` in this database.

The committed rate (100 Hz) was picked by re-scoring the same capture
block-averaged to 50, 100 and 200 Hz and keeping the slowest one that still
passed both gates, not the coarsest rate that merely still ran: the scored
yaw error is not monotonic in the rate (0.444 deg at 50 Hz, 0.205 deg at
100 Hz, 0.306 deg at 200 Hz against the un-decimated 797 Hz capture's
0.200 deg), because block-averaging changes the noise the auto-ZUPT window
statistics see and therefore which stretches get zero-velocity/zero-rotation
updated, which shifts the estimated gyro bias the free-inertial yaw drift is
most sensitive to. A future re-tuning of this dataset shall re-check
neighbouring rates rather than assume the error shrinks monotonically as
the rate goes up.

## REQ-VER-035 — Dual-antenna GNSS heading in the replay tooling

- **Status:** verified
- **Parent:** REQ-VER-003
- **Verification:** Test: python/tests/test_heading_stream.py:test_heading_config_section_and_csv_are_accepted_by_the_replay; Test: python/tests/test_heading_stream.py:test_heading_measurement_gates_and_noise; Test: python/tests/test_heading_stream.py:test_heading_measurement_undoes_a_tilted_cross_baseline; Demonstration: tools/replay.c, tools/replay.py and tools/inspostgui.py --batch report identical heading counts and yaw error on profile_3_aircraft with heading.csv synthesized from ref.csv for a baseline along x and one across the vehicle

**Input.** heading.csv holds one row per usable epoch (datasets/replay_format.py,
HEADING_HEADER): t_us, the baseline azimuth, its 1-sigma, the carrier-phase
solution and the producer's own record. A producer shall give a row the t_us
of the GNSS fix of the same epoch, so that the heading and gnss.csv agree on
the time of one epoch regardless of which IMU sample either message arrived
after. `heading: enable: 1` in a config.yaml turns the stream on.

**Replay.** tools/replay.c, tools/replay.py and tools/inspostgui.py shall
accept a `heading:` config section (enable, baseline_frd, require_fixed,
stddev_scale, stddev_min_deg, delay_ms) and an `inputs: heading` filename
override, and turn the newest heading.csv row of each IMU interval into an
absolute yaw measurement (REQ-NAV-010) with that delay:

- rows whose carr_soln is not 2 are skipped while require_fixed is set,
- the row's 1-sigma is multiplied by stddev_scale (0 -> 1) and floored at
  stddev_min_deg (0 -> no floor), a non-positive result is skipped,
- the azimuth becomes a yaw through REQ-NAV-087 with baseline_frd and the
  suite's current roll/pitch (level while the suite holds no attitude
  yet), a refusal of that geometry is skipped.

Each harness shall report how many rows were offered as yaw and how many
were skipped for each of the three reasons. Rationale: the receiver's
per-epoch accuracy is kept as a column because it moves by an order of
magnitude with the satellite geometry, which a config constant would throw
away, and the mounting stays in config.yaml because a remounted antenna
pair must not need a reconversion.

## REQ-VER-036 — IMU mounting angles in config.yaml

- **Status:** verified
- **Parent:** REQ-VER-003
- **Verification:** Test: tests/test_yaml.c:scenario_imu_mount; Test: python/tests/test_mount_rotation.py:test_mount_composes_all_three_sensors; Test: python/tests/test_mount_rotation.py:test_zero_mount_is_a_noop; Test: python/tests/test_mount_rotation.py:test_mount_survives_a_config_roundtrip; Demonstration: with the imu.csv of datasets/kfgins rotated into a board mounted at roll 2, pitch -3, yaw -9 deg and imu: mount_rpy_deg: [2, -3, -9] in its config.yaml, tools/replay.c and tools/replay.py reproduce the unrotated dataset's attitude and position scores exactly, while the same rotated data without the key misses the attitude gates by about the mounting angles

tools/replay.c, tools/insrcv.c, tools/replay.py and tools/inspostgui.py
shall accept `imu: mount_rpy_deg: [roll, pitch, yaw]` [deg], the attitude
of the sensor board's axes in the vehicle body frame (ZYX, FRD), and
compose it onto the accelerometer, gyroscope and magnetometer calibration
matrices (REQ-NAV-037, all-zero read as identity) before the filter is
configured:

    M := R(roll, pitch, yaw) * M

with R the rotation that takes a board-frame vector into the vehicle
frame (built like R_b_to_n from a roll/pitch/yaw, the vehicle in place of
NED), so the per-axis calibration the matrix already holds is applied
first. The fixed biases stay in the raw sensor frame. A non-finite angle
shall be refused. Missing or all-zero the key changes nothing. The two C
harnesses share one implementation (tools/imu_mount.h), the library itself
is not involved: it only ever sees the composed matrices. After it every
other body-frame quantity of
the config (lever arms, heading baseline_frd) and every attitude the
harness reports refers to the vehicle axes. The composition shall never
be written back into the configuration's own matrices, so a configuration
that is loaded, edited and saved again (the inspostgui.py round trip) does
not apply the mounting twice.

Rationale: a mounting error measured in the field is an angle (e.g. the
yaw offset between the IMU and a dual-antenna heading), and the
misalignment matrices it would otherwise have to be folded into also carry
the board's own per-axis calibration.

## REQ-VER-037 — Estimate compared at the reference point everywhere

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: python/tests/test_score_leverarm.py:test_height_projection_matches_rotated_lever_arm; Test: python/tests/test_score_leverarm.py:test_zero_lever_arm_is_a_noop; Demonstration: with ref.csv of datasets/kfgins moved to a point 2.0 m behind, 0.1 m right and 1.3 m above the IMU and score: leverarm_frd set to that point, tools/replay.c and tools/replay.py report the unmoved dataset's position and ellipsoid height errors, and the state history, North-East and altitude pages of tools/replay.py --plot overlay estimate and reference without the 1.3 m height offset

`score: leverarm_frd` names the point on the vehicle the reference refers
to (a GNSS antenna when ref.csv is the receiver's own solution). Every
place the replay harnesses put the estimate next to the reference shall
compare that point, the IMU position plus R_b_to_n * leverarm_frd with the
suite's current attitude, not the IMU position:

- the position error (as before),
- the ellipsoid height error of nav_suite_get_height_ellipsoid() in
  tools/replay.c and tools/replay.py, using the down component of the
  rotated lever arm with the suite's best available roll and pitch
  (nav_suite_get_rpy, level while the suite holds no attitude), because
  the accessor also answers while ins is coasting or not ready,
- the estimate drawn on the --plot state history (position), North-East
  and altitude pages of tools/replay.py and tools/inspostgui.py (the
  ins_plots pages, the GUI's own replay views follow REQ-VER-039).

A zero lever arm shall change nothing.

Rationale: a reference taken at an antenna 1.3 m above the IMU put a
constant 1.3 m height offset on every page and on the ellipsoid height
score that was not an error of the filter, while the position error next
to it already projected the lever arm and showed none. Two numbers for the
same run that disagree by the lever arm make the plot unusable for
judging the vertical channel.

## REQ-VER-038 — Ranges to anchors in the replay tooling

- **Status:** verified
- **Parent:** REQ-VER-003
- **Verification:** Test: python/tests/test_range_stream.py:test_range_row_and_loader_round_trip; Test: python/tests/test_range_stream.py:test_loader_drops_unusable_rows; Test: python/tests/test_range_stream.py:test_binding_takes_ranges_up_to_the_epoch_limit; Test: python/tests/test_range_stream.py:test_ranges_carry_the_replay_after_the_gnss_stops; Test: python/tests/test_replay_core.py:test_gui_worker_and_replay_py_agree

**Format.** ranges.csv shall hold one calibrated range per row: t_us (the
time the range refers to, the IMU's timebase), anchor id, anchor position
in ECEF [m], range [m] and its 1-sigma [m], trailing columns being the
producer's own record (datasets/replay_format.py range_row()). A row with a
non-positive 1-sigma, a negative range or an id beyond 16 bit shall be
dropped on loading.

**Replay.** tools/replay.c, tools/replay.py and tools/inspostgui.py shall
accept a `ranges:` config section (enable, leverarm_frd, stddev_scale,
stddev_min_m, height_with_baro, aiding_max_hpos_stddev_m) and an
`inputs: ranges` filename override, and hand every ranges.csv row up to each IMU epoch to
the filter as a range measurement (REQ-NAV-082), with the row's age at that
epoch as its delay, the 1-sigma multiplied by stddev_scale (0 -> 1) and
floored at stddev_min_m (0 -> no floor), and the lever arm as the ranging
antenna's. Rows beyond INS_RANGE_MAX in one epoch shall move on to the next
epoch rather than be dropped. All three shall report the rows offered,
fused, rejected, skipped and the epochs counted as position aiding
(REQ-NAV-085), and agree on these counts for the same dataset.

**Binding.** The Python binding shall expose the range measurement
(Navigator.range(), refusing an entry beyond INS_RANGE_MAX in one epoch),
the ranging lever arm and the range diagnostics, and the two ins_options_t
range keys in its Config.

Rationale: the rows keep their own time and age, since a range refers to
the midpoint of its burst, not to the epoch that happens to fuse it.


## REQ-VER-039 — Lever arm compensated reference and fixes in inspostgui

- **Status:** verified
- **Parent:** REQ-VER-037
- **Verification:** Test: python/tests/test_score_leverarm.py:test_leverarm_relation_classifies_gnss_vs_score; Test: python/tests/test_score_leverarm.py:test_gui_records_rotated_lever_arms_for_ref_and_fix; Demonstration: with gnss: leverarm_frd and score: leverarm_frd set to the same antenna point, the Replay tab of tools/inspostgui.py draws the reference line and the GNSS fixes on the estimate's trail while "Lever arm compensated" is checked and offset by the lever arm while it is not, in the 3D view, the altitude profile and the Plots tab's North-East map and altitude profile

tools/inspostgui.py shall offer a "Lever arm compensated" checkbox
(remembered across sessions) for its own replay views: the 3D view and the
altitude profile of the Replay tab, and the North-East map and altitude
profile of the Plots tab. Checked, the reference shall be drawn at the IMU
point, its position minus R_b_to_n * score.leverarm_frd, and every GNSS fix
minus R_b_to_n * gnss.leverarm_frd, both rotated with the suite's best
available attitude (as in REQ-VER-037) of the epoch the reference sample
or the fix arrived and held with it until the next one, so a reference
slower than the plot rate is not bent into an arc per sample while the
vehicle turns. Unchecked, both
shall be drawn where they were taken. Switching shall redraw what is
already on screen. The position error and the score are not affected.

Next to the checkbox the GUI shall classify the lever arms of the config on
screen, before any run: both zero, equal within 1 cm per axis, only the
GNSS arm set (a warning: a reference that is the receiver's own solution
then carries the whole lever arm as error), only the scoring arm set, or
both set but different (a warning: correct for a reference taken at
another point, a typo otherwise). The insdoctor findings of tools/replay.py
and tools/inspostgui.py shall report the last case as info, next to the
existing warning for the GNSS arm set without a scoring arm.

Rationale: the estimate is the IMU's position while the fixes are taken at
the antenna and the reference at its own point, so with a lever arm of a
metre the three lines in the 3D view never meet even when the solution is
right. A mismatch between the two lever arms is the usual cause of a
constant position error floor.

## REQ-VER-040 — Replay without a reference file

- **Status:** verified
- **Verification:** Test: python/tests/test_replay_core.py:test_replay_without_reference_csv

tools/replay.py and tools/inspostgui.py shall run a dataset that has no
ref.csv. Nothing is scored then (zero scored epochs, the reference traces
stay empty) and the GUI shall not report the missing file as a config
error. aiding: ref and init: ref take their state from the reference and
shall be refused with a message naming the missing file.

Rationale: a recording from the field has no ground truth, and the filter
output is still worth looking at.

## REQ-VER-041 — Estimate drawn in one local frame across an ins restart

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: python/tests/test_replay_core.py:test_estimate_stays_in_the_replay_frame_after_an_origin_reset; Demonstration: a field recording with a 0.7 s gap in its data, after which ins restarts with a new local origin 33 m from the first, drawn by tools/inspostgui.py with the reference on top: the estimate stays on the reference after the restart

tools/replay.py and tools/inspostgui.py shall draw the estimate's local
position in the local frame the replay latched once (its origin_ecef),
the frame the reference and the GNSS fixes are converted into: when ins
reports another origin than that one, its position_local() shall be moved
by the offset of its origin in the latched frame, formed with the
mapping the reference gets (ref_to_local_ned). This covers the state
history, the North-East track and the GUI's live position.

Rationale: ins sets a new origin when it restarts, after a time jump for
example (REQ-NAV-016), and position_local() starts again near zero. The
replay keeps its frame on purpose, a frame moving mid-plot would be
worse, so the estimate drawn as it came out of the filter sat shifted
by the distance between the two origins for the rest of the run, while
the error figures, formed in ECEF, showed it on the reference.

## REQ-VER-042 — Tutorial code builds and runs

- **Status:** verified
- **Parent:** REQ-VER-001
- **Verification:** Test: python/tests/test_tutorials.py:test_c_tutorial_first_program; Test: python/tests/test_tutorials.py:test_c_tutorial_lighthouse_snippet; Test: python/tests/test_tutorials.py:test_attitude_tutorial_program; Test: python/tests/test_tutorials.py:test_python_tutorial_first_script; Test: python/tests/test_tutorials.py:test_python_tutorial_tilt_script; Test: python/tests/test_tutorials.py:test_python_tutorial_baro_script; Test: python/tests/test_tutorials.py:test_python_tutorial_compass_script; Test: python/tests/test_tutorials.py:test_python_snippets_use_existing_names; Test: python/tests/test_tutorials.py:test_c_names_in_tutorials_exist

The programs and scripts printed in tutorial/*.md shall build with the
command line the tutorial gives (without warnings), run to completion and
print what the tutorial shows as their output. The Python snippets shall
use only names the binding offers, and the functions and options the
text names shall exist in src/. `make test` runs this check.

Rationale: a tutorial is the first code a new user runs. A change to the
start-up gates, a renamed function or a new source file the build line
does not list breaks it without any other test noticing.

## REQ-VER-043 — Offline RTS smoother in the replay tooling

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: python/tests/test_ins_rts.py:test_state_minus_inverts_the_correction; Test: python/tests/test_ins_rts.py:test_backward_pass_matches_the_textbook_rts; Test: python/tests/test_ins_rts.py:test_a_bias_only_correction_is_a_knot; Test: python/tests/test_ins_rts.py:test_a_growing_variance_cuts_the_chain; Test: python/tests/test_ins_rts.py:test_smoother_beats_the_filter_through_an_outage; Test: python/tests/test_ins_rts.py:test_replay_py_smooth_keeps_the_filter_numbers; Test: python/tests/test_ins_rts.py:test_kml_track_is_the_smoothed_solution; Test: python/tests/test_ins_rts.py:test_gui_worker_smooths_and_the_window_shows_it; Demonstration: datasets/kfgins with tools/replay.py --smooth: position RMS 26 mm -> 17 mm, maximum 271 mm -> 69 mm, velocity RMS 9.8 -> 4.9 mm/s, the filter's own numbers unchanged, and with a 60 s GNSS outage (--gnss-outage 1200:60) the filter's 11.0 m maximum position error inside the outage drops to 0.07 m

tools/replay.py --smooth and the RTS smoother of tools/inspostgui.py
shall run a Rauch-Tung-Striebel smoother (tools/ins_rts.py) over a
forward pass of the unchanged library: every epoch split into
predict() and correct(), recording Phi, P(k|k-1), P(k|k) and the nominal
state, then the backward pass in the filter's error convention (dx =
nominal - truth, attitude through the full rotation), with the smoothed
solution at the full IMU rate. The smoother's forward pass shall be a
second replay with allow_unlimited_deadreckoning, so an outage the
real-time filter gives up on is bridged. The chain shall be cut where
the filter is no plain Kalman filter: ins not ready, a new local origin,
a changed state count, or a correction that grows a variance by more
than 5 %. The filter's own run and every number it reports shall stay
the same with the smoother on. Its accuracy shall be reported next to
the filter's at the same scored epochs (position, velocity, attitude),
in the summary JSON, in pages of the --plot PDF, as a KML path, and in
the GUI as a 3D trail, a Map tab track, plot series and in the Last run
panel.

Rationale: the library is a real-time filter, post-processing gains from
using the measurements after an epoch as well. Running it on the
existing C API keeps one filter implementation. The second forward
pass is needed because a filter that stopped dead reckoning leaves no
trajectory to smooth across the outage, which is where smoothing gains
most.

## REQ-VER-044 — Solution files

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: python/tests/test_ins_solution.py:test_gps_clock_fits_offset_and_rate; Test: python/tests/test_ins_solution.py:test_gps_clock_utc_grid_suspect_and_single_pair; Test: python/tests/test_ins_solution.py:test_gps_clock_refit_bad_rows_and_restart_segments; Test: python/tests/test_ins_solution.py:test_guessed_clock_is_the_system_time; Test: python/tests/test_ins_solution.py:test_gpst_strings; Test: python/tests/test_ins_solution.py:test_pos_layout; Test: python/tests/test_ins_solution.py:test_csv_is_a_ref_csv; Test: python/tests/test_ins_solution.py:test_replay_py_dump_solution; Test: python/tests/test_ins_solution.py:test_gui_exports_the_solution

tools/replay.py --dump-solution and inspostgui.py's Export solution
shall write the filter's solution, and with the RTS smoother on the
smoothed one next to it as <name>_rts, in the format the file extension
picks (tools/ins_solution.py): .pos the RTKLIB solution format (GPST,
geodetic position, Q, the ENU position 1-sigma and signed covariance
roots, age, the demo5 velocity columns), anything else a CSV in the
ref.csv layout plus the position, velocity and attitude 1-sigma, which
both harnesses accept as a ref.csv. In the .pos, Q shall be the quality
of the last GNSS fix while the position is aided (1 where unknown) and
5 while dead reckoning, ns its satellite count while aided (0 where
unknown and while dead reckoning), age the seconds since the last absolute
position aiding. The time shall be GPS time from the dataset's
timesync.csv: offset and rate of the recording clock fitted to its
pulse pairs (the offset alone from one pair), TP_SUSPECT pairs left
out, a pair a whole second off the line dropped by a refit, a pair on
the receiver's UTC grid named in the file header but not corrected, a
row whose flags do not parse skipped, and of several clock segments
(recorder restarts, the mcu_restarts column) only the one with the most
pairs fitted, named in the header.
Without timesync.csv the first epoch shall be put at the system clock
of the export and the header shall say the time is guessed, one guess
for the filter's and the smoother's file.

Rationale: a solution has to leave the tool to be compared with other
estimators (RTKPLOT reads .pos) or reused as a reference, and a
comparison needs a common time base. The recording's microsecond timer
is only that once mapped onto GPS time, which the recorder's
pulse-per-second pairs do, rate included (a crystal is tens of ppm
off).

## REQ-VER-045 — GNSS fix quality, latency and receiver integrity in the evaluation

- **Status:** verified
- **Parent:** REQ-VER-003
- **Verification:** Test: python/tests/test_gnss_quality_latency.py:test_quality_and_latency_stats; Test: python/tests/test_gnss_quality_latency.py:test_integrity_stats_and_short_rows; Test: python/tests/test_gnss_quality_latency.py:test_replay_reports_quality_and_latency; Test: python/tests/test_gnss_quality_latency.py:test_delay_below_latency_and_implausible_latency_are_findings; Test: python/tests/test_gnss_quality_latency.py:test_pdf_page_and_gui_plots

gnss.csv shall optionally carry the fix quality (RTKLIB solution status
numbering: 1 RTK fixed, 2 RTK float, 3 SBAS, 4 DGNSS, 5 single point,
6 PPP, 0 unknown), the receiver's GPS week and time of week of the
fix, the satellites used, the receiver's jamming and spoofing
indications (1 none, 2 warning/indicated, 3 critical/confirmed) and
whether the signals and the navigation message were authenticated
(1 no, 2 yes, 3 failed) as trailing columns, 0 unknown, the first 20
columns unchanged and a row that stops early read as unknown for the
rest. The replay evaluation shall report the share of each quality,
the satellite count (median, minimum), the share of each known
jamming, spoofing and authentication state and, with a
timesync.csv, the transport latency of the fixes (the row's t_us minus
the recorder's t_us of the fix's own GPS time; median, 5 and 95
percentile) in the data quality summary, as Dr. INS findings, on a page
of the --plot PDF and on the GUI's Plots tab. A configured gnss:
delay_ms more than 20 ms below the measured median latency shall be a
warning, a median outside -50 ms .. 2 s a warning that the two clocks
do not belong together. Any jamming or spoofing indication and any
failed authentication shall be a warning.

Rationale: the fix type decides how far a fix can be trusted and is
what a later filter-side use of it needs recorded. Signal and
navigation message authentication and the receiver's interference
monitor decide whether a fix can be trusted at all: the columns are
there before every producer fills them, so a later recording carries
them without another format change. The configured GNSS
delay was an assumption: the measured transport latency is the lower
bound it cannot go below (the receiver's own filter lag adds to it),
and an implausible latency exposes a timesync.csv that does not belong
to the recording.

## REQ-VER-046 — Configurable ARS/AHRS leveling and magnetometer tuning

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main; Test: python/tests/test_ahrs_tuning.py:test_zero_leaves_defaults; Test: python/tests/test_ahrs_tuning.py:test_acc_freq_reaches_both_filters

The replay harnesses (tools/replay.c and tools/replay.py, schema shared)
shall accept the `ahrs:` keys `acc_freq_hz`, `acc_cutoff_freq_hz`,
`gravity_diff_penalty`, `acc_reject_gravity_mps2`, `chi2_threshold`,
`kalman_update_dt_sec`, `mag_yaw_stddev_deg`, `mag_freq_hz` and
`mag_chi2_threshold`, the fields of the same names in `ahrs_config_t`
(ahrs.h, `mag_yaw_stddev_deg` in degrees), and apply each to both
`ars_cfg` and `ahrs_cfg`. A key left at 0 shall leave the field at ahrs's
own default, `gravity_diff_penalty` and `acc_reject_gravity_mps2` also
accept a negative value (penalty off, rejection disabled). The Python
binding shall expose them as `Navigator.set_ahrs_tuning()`
(python/csrc's `ins_suite_set_ahrs_tuning()`).

Rationale: the leveling path (accelerometer rate and low-pass, the
gravity-norm gating and the chi2 gate) decides how well the ARS holds
roll and pitch through accelerations. It could only be tuned by editing
the library defaults, which is not workable when several recordings with
a true attitude reference have to be compared run by run.

## REQ-VER-047 — Reference rows scored at the estimate's epoch

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main; Test: python/tests/test_replay_core.py:test_reference_attitude_is_interpolated_to_the_estimate_epoch

The replay harnesses (tools/replay.c, tools/replay.py) shall carry the
position estimate back to the timestamp of the reference row it is
scored against with the filter's own velocity (not every ref.csv
carries one) before forming the position error, shall score the
attitude against the reference attitude interpolated to the estimate's
epoch between that row and the next one (not across rows further apart
than 1 s), and the C harness shall
hand each GNSS or reference-derived fix to the filter with its age
relative to the IMU epoch it is attached to (gnss_delay_us,
REQ-NAV-092), tools/insrcv.c likewise relative to the newest IMU sample
before the fix arrived.

Rationale: rows are attached to the first IMU epoch at or after their
timestamp, so both a fix and a reference row are up to one IMU period
older than that epoch. Before REQ-NAV-092 the filter fused the fix as
current and lagged the truth by that age; scoring the lagging estimate
against an equally old reference row hid the lag whenever the two
shared timestamps (the Crazyflie captures). Compensating only the
filter would then read as a regression of speed times age. The same age
turns into an attitude error of rotation rate times age: on a 200 Hz
car trial whose IMU samples trail the reference rows by 4.4 ms it made
up most of the scored yaw scatter, so a gate on it measured the
timestamps rather than the filter.

## REQ-VER-048 — ARS/AHRS tuning in the live receiver

- **Status:** verified
- **Parent:** REQ-VER-046
- **Verification:** Inspection: tools/insrcv.c:config_set reads every `ahrs:` key of the shared schema, tools/insrcv.c:suite_apply_ahrs_cfg writes them into both `ars_cfg` and `ahrs_cfg` between nav_suite_init and the first update with the same fallbacks as tools/replay.c:main

The live receiver (tools/insrcv.c) shall accept the whole `ahrs:`
config section of the shared schema (`gyr_noise_psd`, `gyr_bias_rw`,
`acc_noise_mps2`, `rpy_pred_stddev_rad_sqrts`,
`gyr_bias_init_stddev_rps_deg` and the REQ-VER-046 tuning keys) and
apply it to both `ars_cfg` and `ahrs_cfg` the way tools/replay.c does:
`gyr_noise_psd` and `gyr_bias_rw` left at 0 are derived from `imu:`
(`sqrt(gyr_psd)`, `gyr_bias_rw`), `gyr_bias_init_stddev_rps_deg` at 0
leaves the initial bias uncertainty unchanged, every other key at 0
leaves the field at ahrs's own default, and `gravity_diff_penalty` and
`acc_reject_gravity_mps2` pass a negative value through.

Rationale: config.yaml is shared between the live receiver and the
replay harnesses so that a tuning found offline is the tuning that runs
live. The receiver named the `ahrs:` keys as unused and ran its ARS/AHRS
on ahrs's built-in defaults, so the attitude filters behaved differently
live than in the replay that had tuned them.

## REQ-VER-049 — Configurable ins covariance-prediction period

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: python/tests/test_kalman_cadence.py:test_key_reaches_ins_options; Test: python/tests/test_kalman_cadence.py:test_imu_rate_prediction_grows_covariance_sooner; Test: tools/replay.c:main; Inspection: tools/insrcv.c:config_set reads the top-level key and tools/insrcv.c:suite_bootstrap hands it to ins_options_t.kalman_update_dt_sec

The replay harnesses (tools/replay.py and tools/replay.c, schema shared),
the live receiver (tools/insrcv.c) and the config editor of
tools/inspostgui.py shall accept the top-level config key
`kalman_update_dt_sec` [s] and pass it to
`ins_options_t.kalman_update_dt_sec`. A value of 0 shall leave ins's own
covariance-prediction default in force.

Rationale: ins propagates the covariance at a fixed period below the IMU
rate to save CPU time on a microcontroller, while the strapdown itself
runs at the full IMU rate. On the KF-GINS datasets (200 Hz IMU, car with
1 Hz RTK) predicting at the IMU rate lowered the yaw error on the
ICM20602 from 0.233 to 0.179 deg rms and on the ADIS16465 from 0.119 to
0.106 deg rms. The option already existed in the library and the Python
binding, but none of the front ends could set it, so a comparison against
an estimator that predicts at the IMU rate had to run with a handicap
nobody had chosen.

## REQ-VER-050 — GNSS fault and single point replays of the KF-GINS trial

- **Status:** verified
- **Parent:** REQ-VER-002
- **Verification:** Test: tools/replay.c:main; Test: python/tests/test_gnss_faults.py:test_committed_files_reproduce; Test: python/tests/test_gnss_faults.py:test_event_counts_and_spacing; Test: python/tests/test_gnss_faults.py:test_step_is_its_own_correct_answer; Test: python/tests/test_gnss_faults.py:test_spp_accuracy_is_reported; Test: python/tests/test_gnss_faults.py:test_faults_add_to_the_spp_error

The suite shall carry, next to `datasets/kfgins/gnss_vel.csv`, GNSS files
of the same Leador A15 trial with synthetic faults and with L1 single
point (SPP) accuracy, a `config_fault_<type>.yaml`, `config_spp.yaml` or
`config_spp_<type>.yaml` replay of each with ins's chi2 outlier gate on,
and shall gate each against `ref.csv` in `make datasets` and `make test`.

The files shall be written by `tools/gnss_faults.py` from `gnss_vel.csv`
and `ref.csv` alone, with fixed seeds, so that the committed files are
exactly what the script writes (`--check`). Its fault model:

- `spike`: 3 single fixes, 3 s apart, offset U(5, 50) m, the reported
  accuracy kept
- `wrong_fix`: U(5, 30) s of fixes with one offset U(0.5, 3) m, the
  reported accuracy kept (a wrong RTK ambiguity fix)
- `multipath`: U(5, 30) s of fixes with one offset U(3, 15) m, reported at
  1 m horizontal, 2 m vertical
- `degraded`: 30 s of white noise of 1 m horizontal, 2 m vertical,
  reported exactly so
- `vel_spike`: 3 single velocities, 3 s apart, U(1, 5) m/s off
- `step`: 150 s of fixes with one offset U(0.5, 2) m, the reported
  accuracy kept (a change of the RTK base or the datum, a receiver reset)
- `big_step`: as `step`, U(5, 10) m
- `gross`: 3 times U(0, 30) s, the fix at or after each 500 m off (2
  fixes when two times fall on the same one)

Horizontal offsets point in a uniformly random direction, the vertical
one is the horizontal magnitude times U(-1, 1). The events start 120 s
after the platform first moves and follow every 90 s (every 300 s for the
steps) until 90 s before the end of the truth. The SPP error is a
first-order Gauss-Markov process per axis, 1.5 m horizontal and 3 m
vertical (1 sigma), correlation time 30 s, reported with that accuracy.
`spp_<type>` adds the faults of `spike`, `multipath`, `vel_spike`, `step`
or `gross` on top, keeping the SPP accuracy as reported.

Rationale: the chi2 gate (REQ-NAV-090) was only exercised on synthetic
scenarios in `tests/` and on real data without faults. A change to the
gate, to the GNSS noise floors or to the fusion order that lets a blunder
through, or that keeps the filter from following a fix that stays off,
shows here on real IMU data as a position RMS far above its gate. With the
gate off (`chi2_disable: 1`) the 500 m blunders alone cost 76 m position
RMS over the run with RTK (gate 0.015 m) and 18.6 m with SPP (gate
3.111 m). The SPP files keep users without RTK in the regression: with
metre accuracy reported the gate has to stay out of the way of the normal
error and still remove the blunders. The gates are observed + max(5 %,
0.003), rounded up, as for the other `datasets/kfgins` replays.

## REQ-VER-051 — Bit-exact regression check against a reference version

- **Status:** verified
- **Parent:** REQ-VER-003
- **Verification:** Inspection: tests/r1_baseline/state_dump_shim.c:__wrap_nav_suite_update writes every float of the dump in C99 hex; Demonstration: tests/r1_baseline/r1.sh HEAD before a commit that must not change results, all replays reported "same"

`tests/r1_baseline/r1.sh [REF]` shall build `tools/replay.c` from the
git reference REF (default HEAD, sources taken with `git archive`
including the KFCore commit REF pins) and from the working tree, with
the same compiler and the Makefile's flags, and link
`tests/r1_baseline/state_dump_shim.c` into both with
`-Wl,--wrap=nav_suite_update`. The shim shall write, when
`INSLIB_STATE_DUMP` names a file, one line per epoch with:

- the ins nominal state: absolute and local position, velocity, attitude
  quaternion, accelerometer, gyroscope and magnetometer bias
- the diagonal of the ins covariance
- the suite mode and readiness
- the suite attitude and height

Every floating-point value shall be written in C99 hexadecimal notation
(`%a`). The script shall replay every dataset `make test` replays (the
lists are read from the Makefile) through both binaries and report each
run whose dump or exit code differs. It shall exit non-zero if any run
differs.

Rationale: a change that must not alter any result (a refactoring, a
memory layout change, new opt-in behaviour) has to prove that every
existing configuration computes exactly what it computed before. The
replay reports print rounded error statistics, which can hide a changed
result. The hexadecimal dump makes a change of a single bit visible.
Building both sides on one machine keeps the comparison valid on any
toolchain without stored hashes, and the link-time wrap reaches older
versions without changing their source.
