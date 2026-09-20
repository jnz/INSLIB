"""Swiss-army-knife runner: ins on your own CSV data, YAML-configured.

Feeds standard IMU / GNSS / baro / mag CSV files through the
:class:`~INSLIB.suite.Navigator` (nav_suite: ins + AHRS fallback + baro
vertical channel) and emits the best available solution as:

* PlotJuggler JSON stream (UDP)
* MAVLink (ATTITUDE, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, ...)
* a solution CSV (one line per epoch, decimated to ``output.csv_hz``)

Everything is driven by one YAML file: input files with per-column
mappings (0-based index or header name) and units, the filter
configuration, and the outputs. See ``python/examples/runner.yaml`` for
a fully commented reference; a minimal config is just::

    imu:  {file: imu.csv}                 # t_us, gyr xyz [rad/s], acc xyz [m/s2]
    gnss: {file: gnss.csv}                # t_us, lat, lon, h [deg,m], stddev NED
    baro: {file: baro.csv}                # t_us, pressure [Pa]

Usage::

    python3 -m INSLIB config.yaml [--realtime] [--speed N] [--mavlink]
    inslib-run config.yaml                 # with pip install -e python/

This is deliberately NOT the regression harness (see datasets/ and
python/replay.py for dataset replays scored against ground truth). It
is the "just run my data" front end.

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import csv as _csv
import math
import os
import sys
import time

from ._core import Config, ecef_to_llh
from .suite import Navigator
from .telemetry import Telemetry

US_PER_SEC = 1_000_000

_WGS84_A = 6378137.0
_WGS84_E2 = 0.00669437999014


def _llh_to_ecef(lat, lon, h):
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - _WGS84_E2) + h) * math.sin(lat))


def _ecef_to_llh(x, y, z):
    """For a recording that stores its fixes in ECEF: the filter takes
    lat/lon/h, so the conversion happens here rather than inside it."""
    from ._core import ecef_to_llh
    return tuple(ecef_to_llh(x, y, z))


# --------------------------------------------------------------------------
# Unit conversions: every mapped field can declare its source unit; values
# are converted to ins's conventions (SI, radians, Pa, int64 us).
# --------------------------------------------------------------------------

_G0 = 9.80665

_TIME_TO_US = {"s": 1e6, "ms": 1e3, "us": 1.0}
_SCALE = {
    None: 1.0, "": 1.0,
    # angles / angular rate
    "rad": 1.0, "deg": math.pi / 180.0,
    "rad/s": 1.0, "deg/s": math.pi / 180.0,
    # acceleration
    "m/s2": 1.0, "g": _G0,
    # pressure
    "pa": 1.0, "hpa": 100.0, "mbar": 100.0, "kpa": 1000.0,
    # magnetic field
    "ut": 1.0, "gauss": 100.0, "mgauss": 0.1, "nt": 1e-3,
    # plain lengths / velocities
    "m": 1.0, "m/s": 1.0,
}


def _scale(unit):
    key = (unit or "").lower()
    if key not in _SCALE:
        sys.exit(f"unknown unit '{unit}' (known: {sorted(k for k in _SCALE if k)})")
    return _SCALE[key]


# --------------------------------------------------------------------------
# CSV stream with YAML column mapping (index or header name)
# --------------------------------------------------------------------------

class CsvStream:
    """One sensor CSV: resolves column mappings, converts units, yields
    (t_us, record_dict) in file order. Lines starting with '#' and blank
    lines are skipped; ',' / ';' / whitespace delimited."""

    def __init__(self, path, spec, fields):
        """``fields``: {name: (key, n_cols, default_unit)} where ``key``
        selects the YAML mapping entry ('col' scalar or 'cols' list)."""
        self.path = path
        self.spec = spec or {}
        self.fields = fields
        if not os.path.exists(path):
            sys.exit(f"input file not found: {path}")
        self._header = None          # column name -> index (if a header row)
        self._delim = None

    # -- parsing helpers ----------------------------------------------------
    def _split(self, line):
        if self._delim is None:
            self._delim = "," if "," in line else (";" if ";" in line else None)
        if self._delim:
            return [c.strip() for c in line.split(self._delim)]
        return line.split()

    def _resolve(self, col):
        """0-based index (int) or header name (str) -> index."""
        if isinstance(col, int):
            return col
        if self._header and str(col) in self._header:
            return self._header[str(col)]
        sys.exit(f"{self.path}: column '{col}' not found "
                 f"(header: {sorted(self._header) if self._header else 'none'})")

    def rows(self):
        """Yield (t_us, dict) per data row."""
        tspec = self.spec.get("time", {})
        t_col = tspec.get("col", 0)
        t_scale = _TIME_TO_US.get(str(tspec.get("unit", "us")).lower())
        if t_scale is None:
            sys.exit(f"{self.path}: time unit must be one of s/ms/us")

        plan = None   # [(name, [indices], scale)] resolved on first data row
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = self._split(line)
                if self._header is None:
                    # Header row iff the first field is not numeric.
                    try:
                        float(parts[0])
                        self._header = {}
                    except ValueError:
                        self._header = {n: i for i, n in enumerate(parts)}
                        continue
                if plan is None:
                    plan = []
                    for name, (key, n, unit_default) in self.fields.items():
                        m = self.spec.get(key)
                        if m is None:
                            if name.endswith("?"):     # optional field
                                continue
                            # default: consecutive columns after time
                            sys.exit(f"{self.path}: missing mapping '{key}'")
                        cols = m.get("cols", m.get("col"))
                        cols = cols if isinstance(cols, list) else [cols]
                        if len(cols) != n:
                            sys.exit(f"{self.path}: '{key}' needs {n} column(s)")
                        idx = [self._resolve(c) for c in cols]
                        sc = _scale(m.get("unit", unit_default))
                        plan.append((name.rstrip("?"), idx, sc))
                    t_idx = self._resolve(t_col)
                try:
                    t_us = int(float(parts[t_idx]) * t_scale)
                    rec = {}
                    for name, idx, sc in plan:
                        vals = [float(parts[i]) * sc for i in idx]
                        rec[name] = vals[0] if len(vals) == 1 else vals
                except (ValueError, IndexError):
                    continue      # tolerate malformed lines
                yield t_us, rec


def _default_map(spec, defaults):
    """Fill missing column mappings with the documented default layout
    (consecutive columns). ``defaults``: [(key, cols, unit)]."""
    out = dict(spec or {})
    for key, cols, unit in defaults:
        if key not in out:
            out[key] = ({"cols": cols, "unit": unit} if isinstance(cols, list)
                        else {"col": cols, "unit": unit})
    return out


# --------------------------------------------------------------------------
# YAML -> streams + filter config
# --------------------------------------------------------------------------

def load_yaml(path):
    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML is required: pip install pyyaml")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if "imu" not in cfg:
        sys.exit(f"{path}: an 'imu' input is required")
    return cfg


def _rel(base_dir, p):
    return p if os.path.isabs(p) else os.path.join(base_dir, p)


def open_imu(cfg, base):
    spec = _default_map(cfg["imu"], [
        ("time", 0, None),
        ("gyr", [1, 2, 3], "rad/s"),
        ("acc", [4, 5, 6], "m/s2"),
    ])
    return CsvStream(_rel(base, cfg["imu"]["file"]), spec, {
        "gyr": ("gyr", 3, "rad/s"),
        "acc": ("acc", 3, "m/s2"),
    })


def open_gnss(cfg, base):
    if "gnss" not in cfg:
        return None
    g = cfg["gnss"]
    spec = _default_map(g, [
        ("time", 0, None),
        ("pos", [1, 2, 3], None),
    ])
    fields = {
        "pos": ("pos", 3, None),
        "stddev_ned?": ("stddev_ned", 3, "m"),
        "cov_ned?": ("cov_ned", 9, None),
        "vel_ned?": ("vel_ned", 3, "m/s"),
        "vel_stddev_ned?": ("vel_stddev_ned", 3, "m/s"),
        "vel_cov_ned?": ("vel_cov_ned", 9, None),
        "cov_pos_vel?": ("cov_pos_vel", 9, None),
    }
    stream = CsvStream(_rel(base, g["file"]), spec, fields)
    stream.pos_format = str(g.get("format", "llh_deg"))
    if stream.pos_format not in ("llh_deg", "llh_rad", "ecef"):
        sys.exit("gnss.format must be llh_deg | llh_rad | ecef")
    stream.fix_stddev = g.get("stddev", {})     # fallback scalar stddevs
    stream.delay_ms = int(g.get("delay_ms", 0))
    return stream


def open_baro(cfg, base):
    if "baro" not in cfg:
        return None
    b = cfg["baro"]
    spec = _default_map(b, [("time", 0, None), ("pressure", 1, "pa")])
    stream = CsvStream(_rel(base, b["file"]), spec,
                       {"pressure": ("pressure", 1, "pa")})
    stream.stddev_m = float(b.get("stddev_m", 0.0))
    return stream


def open_mag(cfg, base):
    if "mag" not in cfg:
        return None
    m = cfg["mag"]
    spec = _default_map(m, [("time", 0, None), ("mag", [1, 2, 3], "ut")])
    stream = CsvStream(_rel(base, m["file"]), spec, {"mag": ("mag", 3, "ut")})
    stream.var = tuple(m.get("var", (1.0, 1.0, 1.0)))
    return stream


def build_config(cfg, t0_us, first_fix_llh):
    """Config from the 'filter' section; unknown keys fail loudly."""
    fcfg = dict(cfg.get("filter", {}))
    lat, lon, h = first_fix_llh if first_fix_llh else (0.0, 0.0, 0.0)
    kw = {
        "time_us": t0_us,
        "lat_rad": math.radians(fcfg.pop("lat_deg", math.degrees(lat))),
        "lon_rad": math.radians(fcfg.pop("lon_deg", math.degrees(lon))),
        "h_m": fcfg.pop("h_m", h),
        "auto_init": fcfg.pop("auto_init", True),
    }
    for key in ("rpy_pred_stddev_rad_sqrts", "gyr_bias_init_stddev_rps",
                "zero_rot_stddev_rps", "automotive_min_yaw_stddev"):
        if key + "_deg" in fcfg:
            kw[key] = math.radians(fcfg.pop(key + "_deg"))
    if "rpy_init_rad_deg" in fcfg:
        kw["rpy_init_rad"] = tuple(math.radians(v)
                                   for v in fcfg.pop("rpy_init_rad_deg"))
    if "rpy_init_stddev_rad_deg" in fcfg:
        kw["rpy_init_stddev_rad"] = tuple(math.radians(v)
                                          for v in fcfg.pop("rpy_init_stddev_rad_deg"))
    valid = set(Config.__dataclass_fields__)
    for k, v in fcfg.items():
        if k not in valid:
            sys.exit(f"filter: unknown key '{k}' "
                     f"(valid: {sorted(valid)})")
        kw[k] = tuple(v) if isinstance(v, list) else v
    return Config(**kw)


# --------------------------------------------------------------------------
# Solution CSV writer
# --------------------------------------------------------------------------

_CSV_FIELDS = ("t_us", "mode", "ready", "lat_deg", "lon_deg", "h_ell_m",
               "pos_n_m", "pos_e_m", "pos_d_m", "vel_n_mps", "vel_e_mps",
               "vel_d_mps", "roll_deg", "pitch_deg", "yaw_deg",
               "height_m", "height_ell_m", "baro_height_m", "baro_vz_mps")


class SolutionWriter:
    def __init__(self, path):
        self._f = open(path, "w", newline="", encoding="utf-8")
        self._w = _csv.writer(self._f)
        self._w.writerow(_CSV_FIELDS)
        self.n = 0

    @staticmethod
    def _num(v, digits=6):
        return "" if (v is None or not math.isfinite(v)) else round(v, digits)

    def write(self, t_us, sol):
        p = sol.pos_local or (math.nan,) * 3
        v = sol.vel_ned or (math.nan,) * 3
        self._w.writerow([
            t_us, sol.mode, int(sol.ready),
            self._num(math.degrees(sol.lat_rad), 9),
            self._num(math.degrees(sol.lon_rad), 9),
            self._num(sol.alt_m, 3),
            self._num(p[0], 3), self._num(p[1], 3), self._num(p[2], 3),
            self._num(v[0], 3), self._num(v[1], 3), self._num(v[2], 3),
            self._num(math.degrees(sol.roll), 3),
            self._num(math.degrees(sol.pitch), 3),
            self._num(math.degrees(sol.yaw), 3),
            self._num(sol.height_m, 3), self._num(sol.height_ell_m, 3),
            self._num(sol.baro_height_m, 3), self._num(sol.baro_vz_mps, 3),
        ])
        self.n += 1

    def close(self):
        self._f.close()


# --------------------------------------------------------------------------
# main loop
# --------------------------------------------------------------------------

def _peek_first_fix(gnss):
    """First GNSS position as (lat, lon, h) [rad, m], or None."""
    if gnss is None:
        return None
    rows = gnss.rows()
    for _t, rec in rows:
        p = rec["pos"]
        if gnss.pos_format == "llh_deg":
            return (math.radians(p[0]), math.radians(p[1]), p[2])
        if gnss.pos_format == "llh_rad":
            return (p[0], p[1], p[2])
        return ecef_to_llh(*p)
    return None


def _fix_to_llh(rec, fmt):
    """The fix as lat/lon/h in radians, the form the filter takes. A
    recording that stores ECEF is converted here, in the caller, which is
    where that cost belongs."""
    p = rec["pos"]
    if fmt == "llh_deg":
        return (math.radians(p[0]), math.radians(p[1]), p[2])
    if fmt == "llh_rad":
        return tuple(p)
    return _ecef_to_llh(p[0], p[1], p[2])


def _fix_pos_cov(rec, gnss):
    """Position covariance for this fix: per-epoch full 3x3, per-epoch
    stddevs, or the YAML fallback stddevs (in that priority)."""
    if "cov_ned" in rec:
        c = rec["cov_ned"]
        return [c[0:3], c[3:6], c[6:9]]
    if "stddev_ned" in rec:
        return tuple(s * s for s in rec["stddev_ned"])
    fs = gnss.fix_stddev
    hor = float(fs.get("hor_m", 1.0))
    ver = float(fs.get("ver_m", 2.0))
    return (hor * hor, hor * hor, ver * ver)


def _fix_vel_cov(rec, gnss):
    if "vel_cov_ned" in rec:
        c = rec["vel_cov_ned"]
        return [c[0:3], c[3:6], c[6:9]]
    if "vel_stddev_ned" in rec:
        return tuple(s * s for s in rec["vel_stddev_ned"])
    v = float(gnss.fix_stddev.get("vel_mps", 0.2))
    return (v * v, v * v, v * v)


def run(cfg_path, args):
    cfg = load_yaml(cfg_path)
    base = os.path.dirname(os.path.abspath(cfg_path))

    imu = open_imu(cfg, base)
    gnss = open_gnss(cfg, base)
    baro = open_baro(cfg, base)
    mag = open_mag(cfg, base)

    imu_var = cfg.get("imu") or {}
    acc_var = tuple(imu_var.get("acc_var", (0.01,) * 3))
    gyr_var = tuple(imu_var.get("gyr_var", (1e-4,) * 3))
    gnss_sec = cfg.get("gnss") if isinstance(cfg.get("gnss"), dict) else {}
    leverarm = tuple(gnss_sec.get("leverarm_frd", (0.0,) * 3))

    first_llh = _peek_first_fix(gnss)
    imu_iter = imu.rows()
    try:
        t0_us, first_imu = next(imu_iter)
    except StopIteration:
        sys.exit("imu file has no data rows")

    nav = Navigator(build_config(cfg, t0_us, first_llh))

    # World Magnetic Model: explicit position, or auto from the first fix.
    wmm = cfg.get("wmm")
    if wmm:
        year = float(wmm.get("year", 2026.0))
        if "lat_deg" in wmm:
            nav.set_magnetic_model(math.radians(wmm["lat_deg"]),
                                   math.radians(wmm["lon_deg"]), year)
        elif first_llh:
            nav.set_magnetic_model(first_llh[0], first_llh[1], year)

    out = cfg.get("output", {})
    tele = Telemetry(
        plotjuggler=bool(out.get("plotjuggler", True))
        and not args.no_plotjuggler,
        mavlink=bool(out.get("mavlink", False)) or args.mavlink,
        pj_port=int(out.get("pj_port", args.pj_port)),
        mav_port=int(out.get("mav_port", args.mav_port)),
        pj_log=args.flight_log or out.get("flight_log"))
    writer = None
    csv_path = args.csv or out.get("csv")
    if csv_path:
        writer = SolutionWriter(_rel(base, csv_path))
    csv_hz = float(out.get("csv_hz", 0.0))          # 0 -> every epoch
    tele_hz = float(out.get("telemetry_hz", args.telemetry_hz))

    gnss_iter = gnss.rows() if gnss else iter(())
    baro_iter = baro.rows() if baro else iter(())
    mag_iter = mag.rows() if mag else iter(())
    next_gnss = next(gnss_iter, None)
    next_baro = next(baro_iter, None)
    next_mag = next(mag_iter, None)

    n_imu = n_fix = n_baro = n_mag = 0
    t_prev = None
    last_pub = last_csv = None
    pub_period = US_PER_SEC / max(tele_hz, 1e-3)
    csv_period = US_PER_SEC / csv_hz if csv_hz > 0 else 0
    wall0 = time.perf_counter()

    def epoch(t, rec):
        nonlocal next_gnss, next_baro, next_mag, t_prev
        nonlocal n_imu, n_fix, n_baro, n_mag, last_pub, last_csv
        dt = (t - t_prev) / US_PER_SEC if t_prev is not None else 0.0
        t_prev = t
        n_imu += 1
        nav.imu(t, dt, rec["acc"], rec["gyr"], acc_var, gyr_var)

        while next_baro is not None and next_baro[0] <= t:
            nav.baro(next_baro[1]["pressure"], baro.stddev_m)
            n_baro += 1
            next_baro = next(baro_iter, None)

        while next_mag is not None and next_mag[0] <= t:
            nav.mag(next_mag[1]["mag"], mag.var)
            n_mag += 1
            next_mag = next(mag_iter, None)

        fix = None
        while next_gnss is not None and next_gnss[0] <= t:
            fix = next_gnss[1]
            next_gnss = next(gnss_iter, None)
        if fix is not None:
            nav.gnss_pos_llh(_fix_to_llh(fix, gnss.pos_format),
                             _fix_pos_cov(fix, gnss),
                             delay_ms=gnss.delay_ms)
            if "vel_ned" in fix:
                nav.gnss_vel(fix["vel_ned"], _fix_vel_cov(fix, gnss))
                if "cov_pos_vel" in fix:
                    c = fix["cov_pos_vel"]
                    nav.gnss_pos_vel_cov([c[0:3], c[3:6], c[6:9]])
            if any(leverarm):
                nav.gnss_leverarm(leverarm)
            n_fix += 1

        nav.update()

        if last_pub is None or (t - last_pub) >= pub_period:
            last_pub = t
            tele.publish(nav.state(), nav.stddev())
        if writer and (not csv_period or last_csv is None
                       or (t - last_csv) >= csv_period):
            last_csv = t
            writer.write(t, nav.solution())

        if args.realtime:
            target = wall0 + (t - t0_us) / US_PER_SEC / max(args.speed, 1e-6)
            sleep = target - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

    epoch(t0_us, first_imu)
    for t, rec in imu_iter:
        epoch(t, rec)

    sol = nav.solution()
    diag = nav.diag()
    print(f"processed {n_imu} IMU epochs, {n_fix} GNSS fixes, "
          f"{n_baro} baro, {n_mag} mag samples")
    print(f"ins: {diag['n_predict']} predicts, {diag['n_gnss_used']} GNSS "
          f"fusions ({diag['n_gnss_seen']} seen), {diag['n_fuse_fail']} fuse "
          f"fails, {diag['n_auto_zupt']} auto-zupt, "
          f"{diag['n_invalid_input']} invalid inputs")
    print(f"final mode: {sol.mode}", end="")
    if math.isfinite(sol.lat_rad):
        print(f" | lat {math.degrees(sol.lat_rad):.7f} "
              f"lon {math.degrees(sol.lon_rad):.7f} h {sol.alt_m:.2f} m",
              end="")
    if math.isfinite(sol.height_m):
        print(f" | height {sol.height_m:.2f} m", end="")
    print()
    if writer:
        writer.close()
        print(f"solution csv: {csv_path} ({writer.n} rows)")
    nav.close()
    tele.close()


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="inslib-run", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="YAML run configuration")
    ap.add_argument("--realtime", action="store_true",
                    help="pace to wall-clock time")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="realtime speed multiplier")
    ap.add_argument("--mavlink", action="store_true",
                    help="force MAVLink output on")
    ap.add_argument("--no-plotjuggler", action="store_true")
    ap.add_argument("--pj-port", type=int, default=9870)
    ap.add_argument("--mav-port", type=int, default=14550)
    ap.add_argument("--csv", default=None,
                    help="write the solution CSV to this path")
    ap.add_argument("--flight-log", default=None,
                    help="mirror the PlotJuggler stream to NDJSON")
    ap.add_argument("--telemetry-hz", type=float, default=50.0)
    args = ap.parse_args(argv)
    run(args.config, args)


if __name__ == "__main__":
    main()
