#!/usr/bin/env python3
"""Simulated-dataset regression gate (REQ-VER-016), Python/ctypes path.

Counterpart to tools/replay.c for the committed synthetic Groves-profile
datasets under datasets/simulated/. It drives one dataset through ins via
python/replay.py (the analysis tool) and then applies the pass/fail gates:

  * ins's own accuracy vs the true reference -- scored-epoch count and the
    roll/pitch/yaw error mean/std + position RMS gates from the dataset's
    config.yaml `score:` block (lim_att_bias_deg, lim_att_std_deg,
    lim_yaw_bias_deg, lim_yaw_std_deg, lim_pos_rms_m, min_epochs);
  * agreement with an INDEPENDENT textbook filter -- ins's position RMS
    must stay within `score: lim_groves_pos_rms_factor` of the Groves book's
    own loosely-coupled Kalman-filter solution (ref_groves_kf_sol.csv) scored
    over the same window.

Exits non-zero on any failed gate (this is what `make simulated` fails on).
The ARS/AHRS sub-filter gates stay in tools/replay.c, which scores those.

The split is deliberate: python/replay.py is a pure analysis/visualization
tool with NO thresholds or verdict of its own -- it just emits its accuracy
summary as JSON (--summary-json); every gate, the Groves comparison and the
exit code live here.

Usage:
    make pylib
    python3 datasets/check_simulated.py datasets/simulated/profile_1_car

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import bisect
import json
import math
import os
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, "python"))
from replay import load_config, load_ref   # noqa: E402  (analysis-tool lib fns)

_REPLAY_PY = os.path.join(_REPO, "python", "replay.py")


def groves_pos_rms_vs_truth(sol_path, ref, t_warmup_end_us):
    """RMS 3D position error of the Groves KF solution vs the true reference
    over the scored window (REQ-VER-016). The Groves solution CSV is the
    book's native 10-column layout (t[s], lat/lon[deg], h[m], vN,vE,vD,
    roll,pitch,yaw[deg]); time is auto-detected as seconds vs microseconds.
    Returns None if the file is absent/empty. Pure-Python (no numpy)."""
    try:
        f = open(sol_path, encoding="utf-8")
    except OSError:
        return None
    R = 6378137.0
    rt = [r["t_us"] for r in ref]
    rlat = [r["lat_rad"] for r in ref]
    rlon = [r["lon_rad"] for r in ref]
    rh = [r["h_m"] for r in ref]

    def interp(t):
        j = bisect.bisect_left(rt, t)
        if j <= 0:
            return rlat[0], rlon[0], rh[0]
        if j >= len(rt):
            return rlat[-1], rlon[-1], rh[-1]
        span = rt[j] - rt[j - 1]
        w = (t - rt[j - 1]) / span if span else 0.0
        return (rlat[j - 1] + w * (rlat[j] - rlat[j - 1]),
                rlon[j - 1] + w * (rlon[j] - rlon[j - 1]),
                rh[j - 1] + w * (rh[j] - rh[j - 1]))

    rows = []
    with f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split(",")
            if len(p) < 4:
                continue
            rows.append((float(p[0]), math.radians(float(p[1])),
                         math.radians(float(p[2])), float(p[3])))
    if not rows:
        return None
    # Groves native time is seconds; a ref-style export would be microseconds.
    tscale = 1e6 if max(r[0] for r in rows) < 1e5 else 1.0
    ssum, n = 0.0, 0
    for t, slat, slon, sh in rows:
        t_us = t * tscale
        if t_us < t_warmup_end_us:
            continue
        tlat, tlon, th = interp(t_us)
        dn = (slat - tlat) * R
        de = (slon - tlon) * R * math.cos(tlat)
        dd = sh - th
        ssum += dn * dn + de * de + dd * dd
        n += 1
    return math.sqrt(ssum / n) if n else None


def run_replay_summary(dataset):
    """Run python/replay.py on the dataset (no telemetry) and return its
    accuracy-summary JSON as a dict. replay.py's console output is inherited
    so the replay progress/summary still shows. Raises on a non-zero exit."""
    fd, tmp = tempfile.mkstemp(suffix=".json", prefix="ins_summary_")
    os.close(fd)
    try:
        subprocess.run([sys.executable, _REPLAY_PY, dataset,
                        "--summary-json", tmp, "--no-plotjuggler"],
                       check=True)
        with open(tmp, encoding="utf-8") as f:
            return json.load(f)
    finally:
        try:
            os.remove(tmp)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset",
                    help="dataset directory (with config.yaml) or a config "
                         "YAML path (datasets/simulated/...)")
    args = ap.parse_args()

    spec, data_dir = load_config(args.dataset)
    summary = run_replay_summary(args.dataset)

    sc = spec["score"]

    def _lim(key):
        return float(sc.get(key, 0.0) or 0.0)

    att = summary["att_err_deg"]
    pos_rms = summary["pos_rms_m"]
    scored = summary["scored_epochs"]

    checks = []  # (label, value, limit) -- pass iff value <= limit
    min_ep = int(sc.get("min_epochs", 0) or 0)
    checks.append(("scored epochs >= min_epochs",
                   0.0 if scored >= min_ep else 1.0, 0.5))
    if _lim("lim_att_bias_deg") > 0:
        checks.append(("ins |roll bias| [deg]", abs(att["roll"]["mean"]), _lim("lim_att_bias_deg")))
        checks.append(("ins |pitch bias| [deg]", abs(att["pitch"]["mean"]), _lim("lim_att_bias_deg")))
    if _lim("lim_att_std_deg") > 0:
        checks.append(("ins roll stddev [deg]", att["roll"]["std"], _lim("lim_att_std_deg")))
        checks.append(("ins pitch stddev [deg]", att["pitch"]["std"], _lim("lim_att_std_deg")))
    if _lim("lim_yaw_bias_deg") > 0:
        checks.append(("ins |yaw bias| [deg]", abs(att["yaw"]["mean"]), _lim("lim_yaw_bias_deg")))
    if _lim("lim_yaw_std_deg") > 0:
        checks.append(("ins yaw stddev [deg]", att["yaw"]["std"], _lim("lim_yaw_std_deg")))
    if _lim("lim_pos_rms_m") > 0:
        checks.append(("ins pos rms [m]", pos_rms, _lim("lim_pos_rms_m")))
    gfac = _lim("lim_groves_pos_rms_factor")
    if gfac > 0:
        ref = load_ref(os.path.join(data_dir, "ref.csv"))
        g_rms = groves_pos_rms_vs_truth(
            os.path.join(data_dir, "ref_groves_kf_sol.csv"), ref,
            summary["t_warmup_end_us"])
        if g_rms is not None:
            print(f"\nins vs Groves textbook filter: ins pos rms "
                  f"{pos_rms:.3f} m, Groves {g_rms:.3f} m")
            checks.append((f"ins pos rms <= Groves x{gfac:g} [m]",
                           pos_rms, g_rms * gfac))
        else:
            print("  (no ref_groves_kf_sol.csv -- skipping Groves comparison)")

    print("\nchecks:")
    fails = 0
    for label, val, limit in checks:
        ok = val <= limit
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'}  {label:36s}: {val:8.3f} (limit {limit:g})")
    if fails:
        sys.exit(f"\n==== {fails} check(s) FAILED ====")
    print(f"\n==== all {len(checks)} checks passed ====")


if __name__ == "__main__":
    main()
