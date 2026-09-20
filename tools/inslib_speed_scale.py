#!/usr/bin/env python3
"""Estimate speed.scale (config.yaml) from GNSS ground speed.

    python3 inslib_speed_scale.py path/to/csv
    python3 inslib_speed_scale.py path/to/csv --plot
    python3 inslib_speed_scale.py --speed speed.csv --gnss gnss.csv --plot scale.png

ins's absolute-speed aiding (REQ-NAV-068) fuses z = speed_scale * speed_mps
against ||v_ned||. config.yaml's own comment on `speed: scale` says what this
script does: "Systematic and vehicle-specific, so calibrate it against
GNSS". Rolling radius, tyre wear and whatever scaling the source applies are
one constant factor that a drive with real speed variation and an
independent GNSS velocity can pin down directly, no separate fixture needed.

Both series carry a `delay_ms`, the age of a sample when its row was
written (src/ins.h's own convention), so each side is shifted back to its
own true event time before the two are compared:

  * odometry: the per-row `delay_ms` column in speed.csv.
  * GNSS: config.yaml's `gnss: delay_ms` (each NAV-PVT epoch is stamped with
    the IMU t_us most recently seen over USB, folding in the passthrough
    latency that value compensates -- see inslib_convert_ubx_to_csv.py).
    Only vel_ok=1 epochs are used; the Doppler velocity does not depend on
    the position fix quality that flag is otherwise about.

A time-sync error costs nothing while speed is constant and grows with the
rate of change, so samples are also dropped near a stop (--min-speed-mps,
below which GNSS Doppler noise and any driveline backlash dominate) and
during fast speed changes (--max-accel-mps2, where a mistimed delay would
bias the fit rather than average out). Reverse motion (speed.csv's own
`reverse=1`) is dropped by default: whether the vehicle scales the same way
in reverse is not something this script can tell.

The fit is weighted least squares through the origin (no offset term: the
model IS a pure scale), weight = 1/(odometry variance + GNSS variance), plus
a robust median-ratio cross-check that a handful of outliers cannot move.
Large disagreement between the two is itself the finding -- it means the
weighted fit is leaning on a few samples, not that the number is wrong.

(c) Jan Zwiener (jan@zwiener.org)
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import statistics
import sys

DELAY_MS_TO_US = 1000.0

# Trust bar a fit has to clear before it is used unattended, shared with
# inslib_convert_ubx_to_csv.py's automatic calibration -- a number that
# does not clear it is printed here as advisory (below) and left at the
# library default (0.0 -> 1.0) there, rather than baked into config.yaml.
MIN_SAMPLES = 20
DISAGREEMENT_SIGMAS = 3.0
SANE_SCALE_RANGE = (0.5, 2.0)


def load_yaml_defaults(path):
    """(gnss_delay_ms, existing_speed_scale) from config.yaml, or (None,
    None) if the file or PyYAML is not available -- CLI defaults and
    --gnss-delay-ms still work without it."""
    if not path or not os.path.exists(path):
        return None, None
    try:
        import yaml
    except ImportError:
        print("note: PyYAML not installed, cannot read %s (pip install pyyaml); "
              "using --gnss-delay-ms / its default instead" % path, file=sys.stderr)
        return None, None
    try:
        with open(path, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh) or {}
    except OSError as e:
        print("note: cannot read %s: %s" % (path, e), file=sys.stderr)
        return None, None
    gnss_delay = cfg.get("gnss", {}).get("delay_ms") if isinstance(cfg.get("gnss"), dict) else None
    scale = cfg.get("speed", {}).get("scale") if isinstance(cfg.get("speed"), dict) else None
    return gnss_delay, scale


def load_gnss(path):
    """[(t_us, vn, ve, var_vn, var_ve)] for vel_ok=1 epochs, sorted by
    t_us. Columns per the gnss.csv header: t_us, lat, lon, h,
    cov_pos_ned(6), vn, ve, vd, cov_vel_ned(6: nn,ne,nd,ee,ed,dd), vel_ok."""
    out = []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 20:
                continue
            try:
                if int(float(row[19])) != 1:
                    continue
                t_us = int(row[0])
                vn, ve = float(row[10]), float(row[11])
                var_vn, var_ve = float(row[13]), float(row[16])
            except ValueError:
                continue
            out.append((t_us, vn, ve, var_vn, var_ve))
    out.sort(key=lambda r: r[0])
    return out


def load_speed(path):
    """[(t_us, speed_mps, stddev_mps, delay_ms, reverse)] sorted by t_us.
    speed.csv's own header: t_us, speed_mps, stddev_mps, delay_ms, reverse
    (blank when the producer does not report a direction)."""
    out = []
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 4:
                continue
            try:
                t_us = int(row[0])
                speed_mps = float(row[1])
                stddev_mps = float(row[2])
                delay_ms = float(row[3])
            except ValueError:
                continue
            reverse = row[4].strip() if len(row) > 4 else ""
            out.append((t_us, speed_mps, stddev_mps, delay_ms, reverse))
    out.sort(key=lambda r: r[0])
    return out


def interp_gnss(gnss_t, gnss_speed, gnss_var, max_gap_us, t):
    """(speed, variance, d(speed)/dt) of the GNSS series at true time `t`,
    linearly interpolated between the bracketing epochs. None if `t` falls
    outside the series, or the bracket is wider than max_gap_us -- a gap in
    GNSS coverage must not silently turn into a straight-line guess."""
    i = bisect.bisect_left(gnss_t, t)
    if i <= 0 or i >= len(gnss_t):
        return None
    t0, t1 = gnss_t[i - 1], gnss_t[i]
    dt = t1 - t0
    if dt <= 0 or dt > max_gap_us:
        return None
    f = (t - t0) / dt
    speed = gnss_speed[i - 1] + f * (gnss_speed[i] - gnss_speed[i - 1])
    var = gnss_var[i - 1] + f * (gnss_var[i] - gnss_var[i - 1])
    accel = (gnss_speed[i] - gnss_speed[i - 1]) / (dt * 1e-6)
    return speed, var, accel


def collect_samples(gnss_rows, speed_rows, gnss_delay_ms, min_speed_mps,
                    max_gap_s, max_accel_mps2, include_reverse):
    """[(t_true_us, odo_speed, gnss_speed, weight)], one per usable
    odometry row. weight = 1/(odometry variance + GNSS variance): the two
    error sources add, so the sample that is more trustworthy overall
    counts for more without either side needing to dominate by
    construction."""
    gnss_t = [r[0] - gnss_delay_ms * DELAY_MS_TO_US for r in gnss_rows]
    gnss_speed = [math.hypot(vn, ve) for _t, vn, ve, _vv, _ve in gnss_rows]
    gnss_var = []
    for (_t, vn, ve, var_vn, var_ve), sp in zip(gnss_rows, gnss_speed):
        # Diagonal-only propagation of var(sqrt(vn^2+ve^2)); the nd/ed cross
        # terms are not carried in gnss.csv, only nn/ee are used.
        if sp > 1e-3:
            gnss_var.append((vn * vn * var_vn + ve * ve * var_ve) / (sp * sp))
        else:
            gnss_var.append(0.5 * (var_vn + var_ve))

    max_gap_us = max_gap_s * 1e6
    samples = []
    for t_us, speed_mps, stddev_mps, delay_ms, reverse in speed_rows:
        if reverse == "1" and not include_reverse:
            continue
        t_true = t_us - delay_ms * DELAY_MS_TO_US
        got = interp_gnss(gnss_t, gnss_speed, gnss_var, max_gap_us, t_true)
        if got is None:
            continue
        g_speed, g_var, accel = got
        if speed_mps < min_speed_mps or g_speed < min_speed_mps:
            continue
        if abs(accel) > max_accel_mps2:
            continue
        w = 1.0 / (stddev_mps * stddev_mps + max(g_var, 1e-6))
        samples.append((t_true, speed_mps, g_speed, w))
    return samples


def fit_scale(samples):
    """Weighted least squares scale for gnss_speed = scale * odo_speed
    (through the origin, the model has no offset term), plus a robust
    median-ratio cross-check.

    Returns a dict of everything the report needs. sigma_scale is the
    formal weighted-LS standard error, inflated by the reduced chi-square
    when the residuals are worse than the weights predict -- a delay or
    alignment error that stayed unmodeled shows up as extra scatter, not as
    a smaller uncertainty than the fit deserves."""
    n = len(samples)
    sw_oo = sum(w * o * o for _t, o, _g, w in samples)
    sw_og = sum(w * o * g for _t, o, g, w in samples)
    sw = sum(w for *_r, w in samples)
    scale = sw_og / sw_oo

    def wrms(s):
        return math.sqrt(sum(w * (g - s * o) ** 2
                             for _t, o, g, w in samples) / sw)

    resid2 = sum(w * (g - scale * o) ** 2 for _t, o, g, w in samples)
    chi2_red = resid2 / max(n - 1, 1)
    sigma_scale = math.sqrt(max(chi2_red, 1.0) / sw_oo)

    ratios = sorted(g / o for _t, o, g, _w in samples if o > 1e-6)
    median_ratio = statistics.median(ratios)
    mad = statistics.median(abs(r - median_ratio) for r in ratios) * 1.4826

    return dict(n=n, scale=scale, sigma_scale=sigma_scale,
               median_ratio=median_ratio, mad=mad,
               rms_raw=wrms(1.0), rms_fit=wrms(scale),
               t_min=min(t for t, *_ in samples), t_max=max(t for t, *_ in samples),
               odo_min=min(o for _t, o, _g, _w in samples),
               odo_max=max(o for _t, o, _g, _w in samples))


def trust_reason(samples, fit):
    """None if `fit` clears the bar this tool holds its own number to;
    otherwise a one-line reason it is advisory only. `samples` is needed
    only for its length (the MIN_SAMPLES check applies before a fit
    exists at all)."""
    if len(samples) < MIN_SAMPLES:
        return "only %d usable sample(s) (need >= %d)" % (len(samples), MIN_SAMPLES)
    if abs(fit["scale"] - fit["median_ratio"]) > DISAGREEMENT_SIGMAS * math.hypot(
            fit["sigma_scale"], fit["mad"]):
        return ("weighted fit and robust ratio disagree by more than their"
                " combined uncertainty")
    lo, hi = SANE_SCALE_RANGE
    if not (lo <= fit["scale"] <= hi):
        return "%.2f is outside the sane range [%.1f, %.1f] for an odometer scale" % (
            fit["scale"], lo, hi)
    return None


def fmt_unc(x):
    """2 significant figures, so a tight fit (GNSS Doppler noise is far
    below the odometry's own stddev) does not just print as 0.0000."""
    return "%.2g" % x if x else "0"


def make_plot(samples, fit, out_path):
    """Build the diagnostic figure, save it if `out_path` is given, and try
    to pop up an interactive window either way -- Agg (headless, no window)
    is deliberately not forced here, so whatever backend matplotlib picks
    on this machine gets a chance to show one."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("note: matplotlib not installed, skipping --plot "
              "(pip install matplotlib)", file=sys.stderr)
        return
    t0 = samples[0][0]
    t = [(s[0] - t0) * 1e-6 for s in samples]
    odo = [s[1] for s in samples]
    gnss = [s[2] for s in samples]
    scale = fit["scale"]

    fig, (ax_ts, ax_scatter) = plt.subplots(1, 2, figsize=(12, 5))
    ax_ts.plot(t, gnss, ".", ms=2, label="GNSS |v_ned|", alpha=0.6)
    ax_ts.plot(t, [o * scale for o in odo], ".", ms=2,
              label="odometry x %.4f" % scale, alpha=0.6)
    ax_ts.set_xlabel("time [s]")
    ax_ts.set_ylabel("speed [m/s]")
    ax_ts.legend()
    ax_ts.set_title("time series (samples used only)")

    lim = max(max(odo), max(gnss)) * 1.05
    ax_scatter.plot(odo, gnss, ".", ms=2, alpha=0.3)
    ax_scatter.plot([0, lim], [0, lim * scale], "r-",
                    label="fit: scale = %.4f" % scale)
    ax_scatter.plot([0, lim], [0, lim], "k--", lw=0.8, label="scale = 1")
    ax_scatter.set_xlabel("odometry speed [m/s]")
    ax_scatter.set_ylabel("GNSS speed [m/s]")
    ax_scatter.set_xlim(0, lim)
    ax_scatter.set_ylim(0, lim)
    ax_scatter.set_aspect("equal")
    ax_scatter.legend()
    ax_scatter.set_title("odometry vs. GNSS")

    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
        print("plot written to %s" % out_path)
    try:
        plt.show()
    except Exception as e:
        if not out_path:
            print("note: could not open a plot window (%s); pass --plot "
                 "FILE.png to save it instead" % e, file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description="Estimate config.yaml's speed: scale from GNSS ground speed.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("csv_dir", nargs="?",
                    help="capture directory holding speed.csv/gnss.csv/config.yaml")
    ap.add_argument("--speed", metavar="FILE", help="override speed.csv path")
    ap.add_argument("--gnss", metavar="FILE", help="override gnss.csv path")
    ap.add_argument("--config", metavar="FILE",
                    help="override config.yaml path (read for gnss: delay_ms)")
    ap.add_argument("--gnss-delay-ms", type=float,
                    help="GNSS epoch latency [ms] (default: config.yaml's "
                         "gnss: delay_ms, else 200)")
    ap.add_argument("--min-speed-mps", type=float, default=3.0,
                    help="drop samples below this speed (default 3.0): scale "
                         "is unobservable near a stop and GNSS Doppler noise "
                         "dominates there")
    ap.add_argument("--max-gap-s", type=float, default=0.5,
                    help="max age of the GNSS pair a sample is interpolated "
                         "between (default 0.5 s)")
    ap.add_argument("--max-accel-mps2", type=float, default=1.5,
                    help="drop samples where GNSS speed changes faster than "
                         "this (default 1.5 m/s^2), keeps the fit clear of "
                         "whatever error is left in the two delay_ms values")
    ap.add_argument("--include-reverse", action="store_true",
                    help="keep samples flagged reverse=1 (default: dropped, "
                         "since a scale valid in reverse cannot be assumed)")
    ap.add_argument("--plot", nargs="?", const="", metavar="PNG",
                    help="open a diagnostic scatter/time-series plot in a "
                         "window (needs matplotlib); give a path to also "
                         "save it as PNG")
    args = ap.parse_args()

    if args.csv_dir:
        speed_path = args.speed or os.path.join(args.csv_dir, "speed.csv")
        gnss_path = args.gnss or os.path.join(args.csv_dir, "gnss.csv")
        config_path = args.config or os.path.join(args.csv_dir, "config.yaml")
    else:
        speed_path, gnss_path, config_path = args.speed, args.gnss, args.config
    if not speed_path or not gnss_path:
        ap.error("need a capture directory, or --speed and --gnss")
    for label, path in (("speed.csv", speed_path), ("gnss.csv", gnss_path)):
        if not os.path.exists(path):
            sys.exit("%s not found: %s" % (label, path))

    cfg_gnss_delay, cfg_scale = load_yaml_defaults(config_path)
    if args.gnss_delay_ms is not None:
        gnss_delay_ms = args.gnss_delay_ms
    elif cfg_gnss_delay is not None:
        gnss_delay_ms = float(cfg_gnss_delay)
    else:
        gnss_delay_ms = 200.0

    gnss_rows = load_gnss(gnss_path)
    speed_rows = load_speed(speed_path)
    if len(gnss_rows) < 2:
        sys.exit("not enough GNSS velocity fixes (vel_ok=1) in %s" % gnss_path)
    if not speed_rows:
        sys.exit("no odometry samples in %s" % speed_path)

    samples = collect_samples(gnss_rows, speed_rows, gnss_delay_ms,
                              args.min_speed_mps, args.max_gap_s,
                              args.max_accel_mps2, args.include_reverse)
    if len(samples) < MIN_SAMPLES:
        sys.exit("only %d usable sample(s) after filtering (need >= %d); "
                 "drive faster/further, or loosen --min-speed-mps / "
                 "--max-accel-mps2 / --max-gap-s" % (len(samples), MIN_SAMPLES))

    fit = fit_scale(samples)
    span_s = (fit["t_max"] - fit["t_min"]) * 1e-6

    print("%d samples used of %d odometry rows (%.0f%%), spanning %.0f s, "
          "odometry speed %.1f - %.1f m/s"
          % (fit["n"], len(speed_rows), 100.0 * fit["n"] / len(speed_rows),
             span_s, fit["odo_min"], fit["odo_max"]))
    print("gnss.delay_ms used: %.0f ms (%s)"
          % (gnss_delay_ms,
             "from config.yaml" if args.gnss_delay_ms is None and cfg_gnss_delay is not None
             else "--gnss-delay-ms" if args.gnss_delay_ms is not None else "default"))
    print()
    print("weighted LS through origin : scale = %.4f +/- %s"
         % (fit["scale"], fmt_unc(fit["sigma_scale"])))
    print("robust median ratio        : scale = %.4f +/- %s (MAD-based)"
         % (fit["median_ratio"], fmt_unc(fit["mad"])))
    print("speed RMS vs GNSS: %.3f m/s raw -> %.3f m/s scaled"
         % (fit["rms_raw"], fit["rms_fit"]))

    reason = trust_reason(samples, fit)
    if reason:
        print("\n! %s -- inslib_convert_ubx_to_csv.py would leave this at the "
              "library default rather than apply it automatically. Check "
              "--plot before trusting it by hand." % reason)
    if cfg_scale not in (None, 0):
        print("\nconfig.yaml currently has speed: scale: %s" % cfg_scale)

    print("\nconfig.yaml:\n  speed:\n    scale: %.4f" % fit["scale"])

    if args.plot is not None:
        make_plot(samples, fit, args.plot or None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
