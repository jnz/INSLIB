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
- **Verification:** Inspection: datasets/replay_format.py defines the shared config.yaml + imu/ref/gnss/mag/baro/speed.csv contract (schema documented in doc/INSLIB_manual.tex section "config.yaml", dataset specifics confined to the convert_*.py converters)

A single replay harness (one binary, `tools/replay.c`; mirrored by
`python/replay.py`) shall consume a dataset-neutral input format: a
generated per-dataset `config.yaml` (aiding/init mode, IMU noise
model, lever arms, warmup, regression limits, GNSS covariance
fallbacks) plus CSVs — imu.csv (FRD body-frame IMU, optionally
carrying the IMU die temperature [degC] as a trailing eighth column,
which both harnesses ignore: they read the first seven fields, so a
dataset written without it stays valid), ref.csv
(position, attitude, NED velocity), optionally gnss.csv (real GNSS
measurements with the full NED position and velocity covariance;
unknown entries zero), mag.csv (calibrated body-frame field [uT]),
baro.csv (static pressure [Pa]) and speed.csv (scalar ground speed
[m/s], REQ-NAV-068 -- its per-sample uncertainty and delay come from
config.yaml as constants, not from further columns). Dataset specifics (axis conventions,
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
- **Verification:** Inspection: obsolete -- python/replay.py analysis tooling (--estimate-gnss-delay) is not tracked as a requirement (the DB covers the src/ C library and the C regression harness only).

(Deleted, kept for ID stability.) Previously required python/replay.py to
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
verbatim by python/replay.py, which reads the same config.yaml files.
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
- **Verification:** Inspection: obsolete -- python/replay.py `--plot` visualization is not tracked as a requirement (python-tool feature).

(Deleted, kept for ID stability.) Previously required python/replay.py's
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
- **Verification:** Inspection: obsolete -- python/replay.py `--plot` visualization is not tracked as a requirement (python-tool feature).

(Deleted, kept for ID stability.) Previously required python/replay.py's
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

## REQ-VER-015 — Magnetometer hard-iron bias (18-state) support in python/replay.py

- **Status:** deleted
- **Parent:** REQ-NAV-029
- **Verification:** Inspection: obsolete -- python/replay.py config forwarding + `--plot` visualization is not tracked as a requirement (the underlying C-library 18-state mag hard-iron bias support is REQ-NAV-029).

(Deleted, kept for ID stability.) Previously required python/replay.py to
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
analysis tool `python/replay.py` (via its `--summary-json` output) and
owns the pass/fail gates itself, so `replay.py` stays gate-free. This
exercises the whole navigation stack on a known-truth signal end to end
and pins agreement with an independent textbook filter.

`make simulated` shall additionally gate the C harness alone on
`datasets/simulated/B_drone/config_coasting.yaml`, a second config over
the committed B_drone flight that sets the coasting window shorter than
that flight's GNSS outage. The dataset's own config.yaml keeps the whole
outage inside the window on purpose, so only this one drives the filter
across the boundary: inert while the window is expired (REQ-NAV-064),
re-anchored on the first fix afterwards with the carried states inflated
(REQ-NAV-065) and the height taken from the barometer (REQ-NAV-066). The
Python harness is not run on it: its benchmark is the Groves filter,
which the two profile datasets are built for and this flight is not.

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
from. python/replay.py shall accept the same `ahrs:
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

Both replay harnesses (tools/replay.c and python/replay.py, schema
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
`python/replay.py` (PyYAML), and PyYAML accepts both forms. A form only
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
and the straight-line distance across it, and shall gate that error on
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
been fused. Sampled ahead of it, the untrimmed recording exits at 91.7 m, and
at 285.5 m with the lateral constraint disabled.

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
coasting breaks: the exit error is 22.7 m with the speed aiding and 29.2 m
with it disabled. Like REQ-VER-031 the limit sits 1 m above the observed
value (23.7 m).

The speed scale shall be calibrated in the dataset's config, and that
calibration shall not use the scored outage. OBD2 speed on this car reads
about 1.8 percent low against the GNSS horizontal speed, estimated on the
stretch before the tunnel alone. Left at 1.0 the same aiding lands the exit at
55.4 m, worse than no odometry at all, which is a failure the tight gate also
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
