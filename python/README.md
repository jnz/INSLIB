# INSLIB — Python binding + telemetry

A ctypes binding for the INSLIB navigation library plus telemetry
forwarding to PlotJuggler (JSON) and MAVLink. Pure Python + a small C
shim; the only optional dependency is `pymavlink` (for MAVLink output).

> The Python side is intentionally **outside** the project's
> requirements/aerospace process — that rigor applies only to `src/`.
> The C shim lives in `python/csrc/` (never in `src/`), so `make reqs`
> never sees it.

## Layout

```
python/
  INSLIB/                  the importable package (import INSLIB)
    __init__.py           public API re-exports
    _core.py              ctypes plumbing: Config, State, Ins (bare ESKF)
    suite.py              Navigator, Solution  (nav_suite: best solution)
    telemetry.py          Telemetry -> PlotJuggler + MAVLink
    runner.py             YAML-configured CSV runner (inslib-run / -m INSLIB)
    libINSLIB.so           built here by `make pylib` (git-ignored)
  csrc/ins_capi.c/.h    thin, stable C ABI shim (ins_core_* bare, ins_suite_* suite)
  examples/runner.yaml    fully commented runner reference config (CSV)
  replay.py               dataset replay driving the Navigator + telemetry
                          (consumes the datasets dataset directories)
  allan_variance.py       estimate IMU bias random walk (imu.*_bias_rw)
                          from a long static recording (see below)
  inspostgui.py           Qt post-processing GUI on top of replay.py's
                          machinery (see below)
  f9p_config.py           configure a u-blox receiver persistently
                          (RAM+BBR+Flash), independent of the MCU firmware
  spartn_key.py           load PointPerfect dynamic SPARTN keys into the
                          receiver and poll back what it holds. The keys
                          are volatile, so this belongs in a startup path,
                          not in a one-off setup
  pyproject.toml          packaging metadata (editable install works today)
```

The C regression harness `tools/replay.c` (the `make datasets`
gate) and this Python replay consume the SAME dataset directories — a
generated `config.yaml` (aiding/init mode, noise model, lever arms,
limits) plus the dataset-neutral CSVs (`imu/ref/gnss/mag/baro.csv`,
contract in `datasets/replay_format.py`) — and drive the *same*
filter. Pass either the directory or a config YAML path:

```sh
python3 python/replay.py datasets/fog
python3 python/replay.py datasets/fog/config.yaml
```

At the end of a run `replay.py` also **suggests the `imu: gyr_psd`/`acc_psd`
white-noise model** matching the sensor noise it measured under the trial's
real conditions (vibration included, vehicle dynamics excluded).

## allan_variance — bias random walk (imu.\*\_bias\_rw)

The one IMU-model term `replay.py` can *not* read off a moving trial is the
**bias random walk** (`imu.gyr_bias_rw` / `imu.acc_bias_rw`) — a slow drift
that only shows up over tens of seconds of *stationary* data.
`allan_variance.py` estimates it from a **long static recording** via the
Allan deviation: white noise falls on a −1/2 slope, the bias random walk
rises on a +1/2 slope, and its coefficient (the Rate Random Walk, the
+1/2 **asymptote** at τ = 3 s per IEEE Std 952) is exactly the `bias_rw`
the filter wants (rad/s/√s resp. m/s²/√s). It prints which config value to
set:

```sh
python3 python/allan_variance.py <imu.csv | dataset-dir>
python3 python/allan_variance.py static_bench.csv --config config.yaml --plot allan.pdf
python3 python/allan_variance.py static_bench.csv --skip-start 3600 --plot allan.pdf
```

Feed it a long static log (the sensor sitting still on a bench — hours, not
minutes: the +1/2 branch only emerges well past the bias-instability floor,
which for a MEMS part sits at hundreds of seconds). It reads the same
`imu.csv` replay format, reports the RRW per axis, suggests the scalar
`gyr_bias_rw` / `acc_bias_rw` for `config.yaml`, and as a bonus cross-checks
the white-noise `gyr_psd`/`acc_psd` (which should agree with `replay.py`'s
estimate). Needs `numpy` (and `matplotlib` for `--plot`).

Two things it deliberately does *not* do naively:

- **It fits the asymptotes, it does not read the curve at τ = 3 s.** The
  measured Allan deviation is the sum of all noise terms, and at τ = 3 s a
  MEMS IMU is still deep in its white-noise branch — reading σ(3 s) there
  returns the white noise and over-states the RRW by 10–100×. Each
  asymptote is fitted where its ideal slope actually shows (selected by
  local log-log slope, and only where enough averaging clusters remain),
  then extrapolated to its read-out point. Where no clean +1/2 branch
  exists, a rigorous upper bound is reported and flagged `<=`.
- **It checks the recording really was static.** Per-window peak deviation
  from the recording's own resting attitude (not from zero — a static gyro
  still reads its own bias) flags anyone bumping or picking up the sensor.
  With `--plot` to a `.pdf` you also get a bias-trace page, which is what
  reveals a **thermal warm-up transient**: a slow deterministic drift the
  Allan variance cannot distinguish from a random walk. Exclude it with
  `--skip-start`.

It also tells you **whether the recording was long enough**, in hours rather
than as a vague hint: for an axis whose +1/2 branch does not fit under the
cluster limit, it reports where the branch starts and what recording length
that implies, e.g. *"the branch appears near tau=2134 s, so fitting it wants
roughly 18 h of static data (this record: 8.6 h)"*. Progress goes to stderr
(silence it with `--quiet`), the report to stdout, so `> report.txt` keeps
only the report.

## inspostgui — post-processing GUI

`inspostgui.py` is an interactive front end over the same machinery:
pick a dataset (any `config.yaml` under `datasets/` is auto-discovered),
view/edit/create its config in a form (unknown keys like the `score:
lim_*` regression gates are preserved on save), replay it in-process
with a live 3D trajectory view (speed-colored trail, ground-truth
overlay, attitude model, fly mode) and evaluate: error plots with the
filter's own 1-sigma band, the replay.py accuracy/data-quality summary,
multi-page PDF export (ins_plots) and KML export (ins_kml).

```sh
pip install -r python/requirements-inspostgui.txt
make pylib
python3 python/inspostgui.py                                  # or:
python3 python/inspostgui.py datasets/simulated/profile_1_car
python3 python/inspostgui.py --batch datasets/simulated/profile_1_car  # headless smoke test
```

## Build & install

```sh
make pylib            # builds python/INSLIB/libINSLIB.so
pip install -e python/   # optional: editable install so `import INSLIB` works anywhere
```

`replay.py` additionally needs PyYAML and (for `--mavlink`) pymavlink.
One-time venv setup for that:

```sh
sh python/setup_venv.sh              # creates python/.venv, installs ins + replay deps
. python/.venv/bin/activate
```

On Windows (native `cmd.exe`, needs `mingw32-make` on PATH):

```bat
python\setup_venv.bat
python\.venv\Scripts\activate.bat
```

Or without a venv: `pip install -r python/requirements-replay.txt`
(equivalent extra: `pip install -e "python/[replay]"`).

Full `pip install INSLIB` (compiling the C sources into per-platform
wheels) needs a build backend (scikit-build-core / meson-python) and is
deferred. The ctypes + flat-ABI design means one wheel per *platform*,
not per Python version.

## Swiss army knife: run your own CSV data (inslib-run)

One YAML file describes your inputs (IMU required; GNSS/baro/mag
optional), the filter configuration and the outputs — then:

```sh
python3 -m INSLIB myrun.yaml --realtime      # or: inslib-run myrun.yaml
```

```yaml
imu:
  file: imu.csv                        # columns by 0-based index OR header name
  time: {col: t_s, unit: s}            # s | ms | us
  gyr:  {cols: [gx, gy, gz], unit: deg/s}
  acc:  {cols: [ax, ay, az], unit: m/s2}   # or unit: g
gnss:
  file: gnss.csv
  format: llh_deg                      # llh_deg | llh_rad | ecef
  pos:  {cols: [lat, lon, height]}
  stddev_ned: {cols: [sn, se, sd]}     # or cov_ned/vel_cov_ned/cov_pos_vel
                                       # (full 6x6 [pos; vel] covariance)
baro:
  file: baro.csv
  pressure: {col: 1, unit: hPa}
wmm: {year: 2026.5}                    # true north from the first fix
output:
  plotjuggler: true                    # + mavlink: true, csv: solution.csv
```

Unit conversions (deg/s, g, hPa, gauss, …), mixed sample rates and
`#`-comment/malformed lines are handled; the streams are merged by
timestamp (IMU begins each epoch, the latest GNSS/baro/mag sample ≤ t is
attached). The solution CSV carries one row per epoch: mode, lat/lon/h,
NED pos/vel, roll/pitch/yaw, arbitrated heights. See
[examples/runner.yaml](examples/runner.yaml) for every key, defaults and
the per-epoch full-covariance columns.

## High-level: best available solution (Navigator)

`Navigator` wraps `nav_suite` (ins + two AHRS filters with mode
arbitration). Push whatever sensors you have; read a unified `Solution`.
It degrades gracefully: **FULL** → **COASTING** → **ATTITUDE_ONLY**
(AHRS attitude fallback) → **NONE**.

```python
from INSLIB import Navigator, Config, Telemetry

nav  = Navigator(Config(auto_init=True))
tele = Telemetry(plotjuggler=True, mavlink=False)

# once (e.g. on the first coarse position): reference yaw to TRUE north
# via the World Magnetic Model and arm the mag disturbance gate
nav.set_magnetic_model(lat_rad, lon_rad, 2026.5)

# per epoch — add only the sensors you actually have:
nav.imu(t_us, dt, acc, gyr)              # begins an epoch (acc/gyr var optional)
nav.gnss_pos(ecef_xyz, var_ned)          # ECEF fix + diagonal NED covariance
nav.gnss_pos(ecef_xyz, var_ned, delay_ms=80)  # …or a late fix (history-anchored)
nav.gnss_vel(vel_ned, var_ned)
nav.gnss_leverarm((0, 0, -0.14))
nav.mag(mag_uT, mag_var)                 # magnetometer (or set Config.magnetic_n)
nav.baro(pressure_pa)                    # static pressure -> vertical channel
nav.yaw(yaw_rad, stddev_rad)             # absolute heading (dual-antenna, pose)
nav.local_pos(pos_ned, var_ned)          # lighthouse / UWB / mocap
nav.zupt(True); nav.zaru(True)           # optional: known standstill / clamp

nav.update()                             # run the filter for this epoch, then read:
sol = nav.solution()                     # Solution: mode, ready, roll/pitch/yaw, pos…
sol.height_m                             # best height above datum (ins|baro)
sol.height_ell_m                         # best absolute height (incl. baro+offset)
tele.publish(nav.state())                # stream to PlotJuggler / MAVLink
```

The vertical channel degrades as gracefully as the rest: under fresh
GNSS aiding heights come from INSLIB; in an outage the baro filter keeps
measuring (same datum, continuous), and `height_ell_m` stays absolute
via the estimated baro-to-ellipsoid offset. Estimated sensor biases are
available as `nav.bias_acc()` / `bias_gyr()` / `bias_mag()` (the latter
with `Config(estimate_mag_bias=True)`, the 18-state mode).

## Low-level: the bare ESKF (Ins)

```python
from INSLIB import Ins, Config
nav = Ins(Config(auto_init=True))      # no AHRS fallback, leaner
nav.imu(t_us, dt, acc, gyr); nav.gnss_pos(ecef, var_ned); nav.update()
print(nav.rpy(), nav.is_ready())
```

Conventions match ins: body frame FRD, nav frame NED, Hamilton
quaternion `q=[w,x,y,z]`, time in int64 microseconds, angles in radians.

## Live replay + telemetry

```sh
python3 python/replay.py datasets/fog --realtime
python3 python/replay.py datasets/fog --realtime --speed 10 --mavlink
```

The dataset (`fog`, MEMS ADAHRS vs. an independent FOG strapdown
attitude reference, real GNSS aiding) is committed under
`datasets/fog/`, see `datasets/fog/config.yaml`. Needs PyYAML
(`pip install pyyaml`).

`--realtime` paces the replay to wall-clock time so you can watch the
solution converge live; `--speed N` runs N× faster (`fog` is ~16 min,
KF-GINS ~57 min). Without `--realtime` it runs as fast as possible.

> **Step-by-step live demo (PlotJuggler setup + suggested layout):**
> see [DEMO.md](DEMO.md).

PlotJuggler: add a UDP/JSON source on port 9870. Series:
* `INSLIB/…`  — the arbitrated "best available" estimate: pos_ned, vel_ned,
  **att_deg** (roll/pitch/yaw), rate_dps, global, acc_n, mode.
* `INSLIB/ars/…`, `INSLIB/ahrs/…`, `INSLIB/full3d/…`, `INSLIB/baroalt/…` — the
  individual sub-filter outputs (not just the arbitrated one above), so
  e.g. `INSLIB/ars/att_deg` vs. `INSLIB/ahrs/att_deg` vs.
  `INSLIB/full3d/att_deg`, or `INSLIB/baroalt/pos_ned/d` vs.
  `INSLIB/full3d/pos_ned/d`, can be compared directly.
* `ref/…`    — the ground truth in the **same frames** (pos_ned, att_deg,
  vel_ned, global), so `ref/*` overlays directly on the matching `INSLIB/*`
  (drag both onto one plot to see the error yourself).
* `meas/…`   — the raw input measurements this epoch: `imu` (acc/gyro),
  `gnss` (last fix, same local frame as `INSLIB/pos_ned`), `mag`, `baro`
  (pressure + derived ISA altitude).

MAVLink: any GCS on `udp:14550` gets **ATTITUDE** (Euler) and
ATTITUDE_QUATERNION, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, HIGHRES_IMU,
EKF_STATUS_REPORT (`pip install pymavlink`). `--flight-log PATH` mirrors
the PlotJuggler stream into NDJSON.

## Tests

```sh
make pytest          # binding smoke tests (uses pytest if installed, else standalone)
```
