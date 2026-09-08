#!/usr/bin/env python3
"""Allan-variance IMU noise analysis - estimates the bias RANDOM WALK
(``imu.gyr_bias_rw`` / ``imu.acc_bias_rw`` in a replay ``config.yaml``)
from a long STATIC recording, and tells you which config value to set.

Why a separate tool: replay.py's data-quality summary estimates the white
noise (``gyr_psd``/``acc_psd``) from the moving trial by consecutive-sample
differencing, but the bias random walk (Rate Random Walk, RRW) is a slow
drift that only shows up over tens of seconds of *stationary* data, it
cannot be read off a maneuvering trajectory. The Allan deviation separates
the two: white noise falls on a -1/2 slope, the bias random walk rises on a
+1/2 slope. The RRW coefficient K is, by IEEE convention (Std 952), the
value of that +1/2 line at tau = 3 s, and for a random-walk process K is
numerically the driving strength sigma_rw in rad/s/sqrt(s). The
unit the INSLIB filter's bias_rw parameter uses. So:

    imu.gyr_bias_rw  =  K_gyro   (Allan +1/2 ASYMPTOTE at tau=3 s)
    imu.acc_bias_rw  =  K_accel

Note "asymptote", not "curve": the measured Allan deviation is the sum of
all noise terms, and at tau = 3 s a decent MEMS IMU is still deep in its
white-noise branch, its random walk only takes over after hundreds of
seconds. Reading sigma(3 s) straight off the curve therefore reports the
WHITE NOISE and over-states K by one to two orders of magnitude. Each
asymptote is instead fitted on the stretch of curve where it actually
dominates (selected by local log-log slope) and then extrapolated to its
convention read-out point. Where the record is too short for a +1/2 branch
to emerge at all, a rigorous upper bound is reported instead and flagged
as such.

Feed it a long static log (ideally a few hours): the sensor sitting still on a
bench, the longer the better for a stable RRW.

Usage:
    python3 python/allan_variance.py <imu.csv | dataset-dir> [options]

    --config PATH   config.yaml to compare against (shows current -> suggested)
    --plot PATH     write the Allan-deviation plot (PNG/PDF, needs matplotlib)
    --skip-start S  discard the first S seconds, --skip-end S the last S

A MEMS gyro's thermal warm-up is a slow deterministic drift that the Allan
variance cannot distinguish from a random walk, so it inflates the RRW --
on a real 8.5 h bench recording, dropping the first hours moved the worst
gyro axis by 40%. The bias-trace page of --plot shows whether a given
recording has such a transient (and how long it runs), --skip-start then
excludes it.

Needs numpy. The IMU CSV is the replay format (datasets/replay_format.py):
``t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]``.

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import math
import os
import sys
import textwrap

try:
    import numpy as np
except ImportError:
    sys.exit("allan_variance.py needs numpy (pip install numpy)")

# The RRW coefficient K is defined as the +1/2 asymptote's value at
# tau = 3 s (IEEE Std 952) - the averaging time at which a random walk's
# Allan deviation, sigma(tau) = sigma_rw*sqrt(tau/3), equals its driving
# strength sigma_rw exactly. That is what makes K directly usable as the
# filter's bias_rw parameter.
RRW_TAU_S = 3.0
# ARW (white noise) coefficient N is defined at tau = 1 s.
ARW_TAU_S = 1.0
_G = 9.80665

# --- Asymptote fitting -------------------------------------------------
# Each noise term dominates its own stretch of the Allan curve, so each
# asymptote is fitted only where its ideal slope actually shows, then
# extrapolated to the convention read-out point (see the module docstring).

# The overlapping-AVAR estimate scatters badly once few independent
# clusters are left in the record, so fitting stops at duration/this. Ten
# clusters is roughly a 25% 1-sigma uncertainty on the point itself, which
# is about as loose as a fit input should get.
MIN_CLUSTERS_FOR_FIT = 10
# A curve point counts toward an asymptote when its local log-log slope is
# within this of the ideal (-1/2 for white noise, +1/2 for the random walk).
ASYMPTOTE_SLOPE_TOL = 0.25
# Below this many points a "fit" is just noise -> report the bound instead.
MIN_ASYMPTOTE_POINTS = 3
# "How long should I have recorded?": the +1/2 branch has to start (tau_rise)
# and then still leave a stretch of curve below the cluster cap for the fit
# to sit on. Wanting the cap about this far above tau_rise leaves the few
# log-spaced grid points a fit needs - a rule of thumb, not a theorem.
FIT_SPAN_FACTOR = 3.0

# Rough stillness check: bucket the recording into non-overlapping windows
# and flag any window whose PEAK (not mean) gyro/accel exceeds these - a
# brief disturbance (someone bumping/lifting the IMU) shows up as a peak
# even if it's short compared to the window and barely moves the window's
# mean. Deliberately loose (this is a sanity check, not a rigorous
# detector): real MEMS noise peaks well below these within a 5 s window.
# The window stays SHORT on purpose: a longer one dilutes a brief bump.
MOTION_WINDOW_SEC = 5.0
MOTION_GYR_THRESH_DPS = 1.0
MOTION_ACC_THRESH_MPS2 = 0.3

# The bias trace wants the opposite of the stillness check: as LONG a
# window as the plot can afford, because white noise only averages down as
# 1/sqrt(window) and what is left is the bias wander we actually want to
# look at. The window is therefore derived from the record length (aiming
# for this many plotted points) rather than shared with MOTION_WINDOW_SEC.
BIAS_TRACE_TARGET_POINTS = 400
BIAS_TRACE_MIN_WINDOW_SEC = 30.0
# ... but never so long that a short record collapses to a handful of points.
BIAS_TRACE_MIN_POINTS = 20


_QUIET = False


def _progress(msg, step=None, total=None):
    """One-line progress on STDERR, so a piped/redirected stdout keeps only
    the report. Overwrites itself while a TTY is attached, scrolls otherwise
    (a log file then keeps every step instead of a line of control codes)."""
    if _QUIET:
        return
    prefix = f"[{step + 1}/{total}] " if step is not None and total else ""
    line = f"  {prefix}{msg}"
    if sys.stderr.isatty():
        sys.stderr.write("\r\033[K" + line)
    else:
        sys.stderr.write(line + "\n")
    sys.stderr.flush()


def _progress_done(msg):
    """Finish the progress line (leaves the final message standing)."""
    if _QUIET:
        return
    if sys.stderr.isatty():
        sys.stderr.write("\r\033[K")
    sys.stderr.write(f"  {msg}\n")
    sys.stderr.flush()


def read_imu(path):
    """Load a imu.csv into (t_us, gyr Nx3, acc Nx3) arrays.
    Accepts either the file or a dataset directory containing imu.csv."""
    if os.path.isdir(path):
        path = os.path.join(path, "imu.csv")
    try:
        size_mb = os.path.getsize(path) / (1 << 20)
        _progress(f"reading {path} ({size_mb:.0f} MB) ...")
    except OSError:
        _progress(f"reading {path} ...")
    data = np.loadtxt(path, delimiter=",", comments="#")
    if data.ndim != 2 or data.shape[1] < 7:
        sys.exit(f"{path}: expected at least 7 columns (t_us, gyr xyz, acc xyz)")
    return data[:, 0], data[:, 1:4], data[:, 4:7]


def allan_deviation(x, tau0, taus):
    """Overlapping Allan deviation of a 1-D rate/accel series `x` sampled at
    period `tau0` [s], evaluated at each averaging time in `taus` [s].

    Returns (tau_actual, adev) for the taus that fit the record length.
    Uses the standard integrated-signal estimator
        sigma^2(tau) = 1 / (2 tau^2 (M)) * sum_k (theta[k+2m] - 2 theta[k+m] + theta[k])^2
    with theta the cumulative integral of x (angle for a gyro, velocity for
    an accelerometer) and m = tau/tau0."""
    theta = np.concatenate(([0.0], np.cumsum(x) * tau0))  # N+1 points
    n_theta = len(theta)
    out_tau, out_adev = [], []
    for tau in taus:
        m = int(round(tau / tau0))
        if m < 1 or 2 * m >= n_theta:
            continue
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        tau_m = m * tau0
        avar = float(np.sum(d * d)) / (2.0 * tau_m * tau_m * len(d))
        out_tau.append(tau_m)
        out_adev.append(math.sqrt(avar))
    return np.array(out_tau), np.array(out_adev)


def _local_slope(taus, adev):
    """Local power-law exponent d log(sigma) / d log(tau) of the curve -
    -1/2 where white noise dominates, +1/2 where the random walk does,
    ~0 across the bias-instability floor between them."""
    return np.gradient(np.log(adev), np.log(taus))


def _fit_asymptote(taus, adev, mask, slope, tau_ref):
    """Least-squares fit of a power law with FIXED exponent,
        sigma(tau) = C * (tau/tau_ref)**slope
    to the masked points. With the exponent fixed, the log-log fit collapses
    to a geometric mean, so there is nothing to iterate:
        log C = mean( log(sigma_i) - slope*log(tau_i/tau_ref) )
    Returns C, the asymptote's value at tau_ref, or None if the mask holds
    too few points for the fit to mean anything."""
    t = taus[mask]
    a = adev[mask]
    if len(t) < MIN_ASYMPTOTE_POINTS:
        return None
    return float(np.exp(np.mean(np.log(a) - slope * np.log(t / tau_ref))))


def _asymptote_bound(taus, adev, mask, slope, tau_ref):
    """Tightest power law of the given fixed exponent that still passes
    UNDER every masked point. The measured Allan deviation is the sum of all
    noise contributions, so no single asymptote can poke above it - this is
    therefore a genuine upper bound on the coefficient, and it is what to
    report when the branch never emerges cleanly (a record too short to
    resolve the random walk). Returns None if the mask is empty."""
    t = taus[mask]
    a = adev[mask]
    if len(t) == 0:
        return None
    return float(np.min(a / (t / tau_ref) ** slope))


def _analyze_axis(taus, adev, duration):
    """Asymptote read-out for one axis' Allan curve, as a dict:

        arw        white-noise coefficient at tau=1 s
        rrw        random-walk coefficient at tau=3 s
        resolved   True if rrw is a real fit, False if it is the upper bound
        mask       curve points the +1/2 fit used (for the plot)
        tau_rise   smallest tau where the curve reaches the +1/2 branch, or
                   None if it never does. Searched over the WHOLE curve,
                   including the statistically thin tail past the cluster
                   cap: an indication of where the branch is, which is what
                   tells the caller how long a record would have to be to
                   fit it (see FIT_SPAN_FACTOR).

    arw/rrw are None for a degenerate channel (a constant signal has a zero
    Allan deviation, so there is no log-log curve to fit)."""
    empty = np.zeros(len(taus), dtype=bool)
    none = {"arw": None, "rrw": None, "resolved": False, "mask": empty,
            "tau_rise": None}
    ok = np.isfinite(adev) & (adev > 0.0)
    if int(np.count_nonzero(ok)) < MIN_ASYMPTOTE_POINTS:
        return none

    slope = np.full(len(taus), np.nan)          # nan compares False everywhere
    slope[ok] = _local_slope(taus[ok], adev[ok])

    reliable = ok & (taus <= duration / MIN_CLUSTERS_FOR_FIT)
    if not np.any(reliable):
        reliable = ok
    # The curve's minimum (the bias-instability floor) separates the falling
    # white-noise branch from the rising random-walk branch.
    tau_min = taus[int(np.argmin(np.where(reliable, adev, np.inf)))]

    left = ok & (taus <= tau_min)
    on_rrw_branch = ok & (taus >= tau_min) & (slope >= 0.5 - ASYMPTOTE_SLOPE_TOL)
    arw_mask = left & (np.abs(slope + 0.5) <= ASYMPTOTE_SLOPE_TOL)
    rrw_mask = (reliable & (taus >= tau_min)
                & (np.abs(slope - 0.5) <= ASYMPTOTE_SLOPE_TOL))

    n_arw = _fit_asymptote(taus, adev, arw_mask, -0.5, ARW_TAU_S)
    if n_arw is None:
        n_arw = _asymptote_bound(taus, adev, left, -0.5, ARW_TAU_S)

    k_rrw = _fit_asymptote(taus, adev, rrw_mask, +0.5, RRW_TAU_S)
    k_resolved = k_rrw is not None
    if not k_resolved:
        k_rrw = _asymptote_bound(taus, adev, reliable, +0.5, RRW_TAU_S)

    rise = taus[on_rrw_branch]
    return {"arw": n_arw, "rrw": k_rrw, "resolved": k_resolved,
            "mask": rrw_mask,
            "tau_rise": float(rise[0]) if len(rise) else None}


def _tau_grid(tau0, duration, n=60):
    """Log-spaced averaging times from a few samples up to ~1/4 of the
    record (beyond that too few clusters remain), always including the RRW
    and ARW read-out points so they land on the grid."""
    m_max = max(4, int((duration / tau0) / 4))
    ms = np.unique(np.round(np.geomspace(1, m_max, n)).astype(int))
    taus = list(ms * tau0)
    for t in (ARW_TAU_S, RRW_TAU_S):
        if tau0 <= t <= duration / 4.0:
            taus.append(t)
    return sorted(set(taus))


def _window_view(gyr, acc, fs, window_sec):
    """(gyr_w, acc_w, t_center_s) reshaped into non-overlapping windows of
    window_sec (assumes an ~constant sample rate). None if the recording is
    too short for even 2 windows."""
    win_n = max(1, int(round(window_sec * fs)))
    n_win = len(gyr) // win_n
    if n_win < 2:
        return None
    return (gyr[:n_win * win_n].reshape(n_win, win_n, 3),
            acc[:n_win * win_n].reshape(n_win, win_n, 3),
            (np.arange(n_win) + 0.5) * win_n / fs)


def _windowed_diagnostics(gyr, acc, fs, window_sec=MOTION_WINDOW_SEC):
    """Stillness check: per short window, the PEAK deviation of gyr/acc from
    the recording's own resting attitude. None if too short for 2 windows.

    The peaks are measured against the per-axis MEDIAN over the whole
    recording, not against zero/g: a static IMU still reads its own gyro
    bias (a couple of deg/s is ordinary for an uncalibrated MEMS part), so
    comparing raw |gyr| against a motion threshold would flag every single
    window. Subtracting the median leaves only what actually changed. For
    the accelerometer this also catches a pure TILT, which a check on
    |accel| alone would miss because reorienting the sensor does not change
    the magnitude of gravity."""
    view = _window_view(gyr, acc, fs, window_sec)
    if view is None:
        return None
    gyr_w, acc_w, t_center = view
    n = gyr_w.shape[0] * gyr_w.shape[1]
    gyr_rest = np.median(gyr[:n], axis=0)
    acc_rest = np.median(acc[:n], axis=0)
    return {
        "t_center_s": t_center,
        "window_sec": window_sec,
        "gyr_peak_rps": np.linalg.norm(gyr_w - gyr_rest, axis=2).max(axis=1),
        "acc_peak_mps2": np.linalg.norm(acc_w - acc_rest, axis=2).max(axis=1),
    }


def _bias_trace_window_sec(duration):
    """Averaging window for the bias trace: long enough that white noise is
    averaged away (it only falls as 1/sqrt(window)), short enough to still
    plot a useful number of points. Derived from the record length so an
    8 h bench run and a 20 min one both come out readable."""
    win = max(BIAS_TRACE_MIN_WINDOW_SEC, duration / BIAS_TRACE_TARGET_POINTS)
    return min(win, max(1.0, duration / BIAS_TRACE_MIN_POINTS))


def _bias_trace(gyr, acc, fs, duration):
    """Per-axis MEAN gyr/acc over long windows -- with the white noise
    averaged down, what is left IS the bias wander (random walk plus any
    thermal transient), plotted directly as a time series. Deliberately a
    much longer window than the stillness check above, which needs short
    windows so a brief bump is not diluted. None if too short."""
    win_sec = _bias_trace_window_sec(duration)
    view = _window_view(gyr, acc, fs, win_sec)
    if view is None:
        return None
    gyr_w, acc_w, t_center = view
    return {
        "t_center_s": t_center,
        "window_sec": win_sec,
        "gyr_mean": gyr_w.mean(axis=1),
        "acc_mean": acc_w.mean(axis=1),
    }


def _motion_segments(diag, gyr_thresh_dps=MOTION_GYR_THRESH_DPS,
                     acc_thresh_mps2=MOTION_ACC_THRESH_MPS2):
    """Contiguous flagged windows merged into segments: [{t_start_s,
    t_end_s, gyr_peak_dps, acc_peak_mps2}, ...] - a rough "was the IMU
    disturbed here" list, not a rigorous motion detector."""
    flagged = ((diag["gyr_peak_rps"] > math.radians(gyr_thresh_dps)) |
              (diag["acc_peak_mps2"] > acc_thresh_mps2))
    bounds = []
    start = None
    for i, f in enumerate(flagged):
        if f and start is None:
            start = i
        elif not f and start is not None:
            bounds.append((start, i - 1))
            start = None
    if start is not None:
        bounds.append((start, len(flagged) - 1))
    return [{
        "t_start_s": float(diag["t_center_s"][a]),
        "t_end_s": float(diag["t_center_s"][b]),
        "gyr_peak_dps": float(math.degrees(diag["gyr_peak_rps"][a:b + 1].max())),
        "acc_peak_mps2": float(diag["acc_peak_mps2"][a:b + 1].max()),
    } for a, b in bounds]


def analyze(imu_path, skip_start_sec=0.0, skip_end_sec=0.0):
    """Full analysis of one static IMU recording. Returns a dict with the
    per-axis RRW (bias_rw) and ARW estimates, suggested scalar config
    values, the Allan curves (for plotting) and any data-quality warnings.

    skip_start_sec/skip_end_sec trim the record before anything is computed.
    The start trim matters more than it sounds: a MEMS gyro's thermal
    warm-up drift is a slow deterministic ramp, which the Allan variance
    cannot tell from a random walk and which therefore inflates the RRW.
    The bias-trace page of -plot is what shows whether a given recording
    has one (and how long it lasts)."""
    tau = RRW_TAU_S
    t_us, gyr, acc = read_imu(imu_path)
    if skip_start_sec > 0.0 or skip_end_sec > 0.0:
        t_rel = (t_us - t_us[0]) / 1e6
        keep = ((t_rel >= skip_start_sec)
                & (t_rel <= t_rel[-1] - max(0.0, skip_end_sec)))
        if int(np.count_nonzero(keep)) < 100:
            sys.exit(f"{imu_path}: trimming left {int(np.count_nonzero(keep))} samples")
        t_us, gyr, acc = t_us[keep], gyr[keep], acc[keep]
    n = len(t_us)
    if n < 100:
        sys.exit(f"{imu_path}: only {n} samples - need a long static record")
    dt = np.diff(t_us) / 1e6
    tau0 = float(np.median(dt))
    duration = float((t_us[-1] - t_us[0]) / 1e6)
    fs = 1.0 / tau0 if tau0 > 0 else 0.0

    taus = _tau_grid(tau0, duration)
    trimmed = (" (trimmed)" if skip_start_sec > 0.0 or skip_end_sec > 0.0 else "")
    _progress_done(f"{n} samples, {_fmt_hms(duration)} @ {fs:.2f} Hz{trimmed}"
                   f" -> {len(taus)} averaging times, fitting up to "
                   f"tau={duration / MIN_CLUSTERS_FOR_FIT:.0f} s")
    progress_base = 0

    warnings = []
    # The +1/2 branch only emerges well past the bias-instability floor, so
    # a usable RRW wants hours, not minutes (see the per-axis "not resolved"
    # warning below for whether THIS record actually got there).
    if duration < 100.0:
        warnings.append(f"record is only {duration:.0f} s - RRW at tau={tau:g} s "
                        "wants several minutes of static data for a stable estimate")
    # Static sanity: a moving platform makes the RRW slope meaningless
    # (real dynamics masquerade as drift). Gyro should sit near zero rate.
    gyr_span = float(np.max(np.std(gyr, axis=0)))
    if gyr_span > 0.02:  # ~1.1 deg/s
        warnings.append(f"gyro varies by {math.degrees(gyr_span):.2f} deg/s - "
                        "recording may not be static; RRW will be over-estimated")
    acc_norm = float(np.mean(np.linalg.norm(acc, axis=1)))
    if abs(acc_norm - _G) > 1.0:
        warnings.append(f"mean |accel| = {acc_norm:.2f} m/s^2 (expected ~{_G:.1f}) - "
                        "check the recording is static and in m/s^2")
    # The Allan estimator integrates the samples assuming one uniform tau0,
    # so a dropout is silently treated as if no time had passed there.
    gap_thresh = 10.0 * tau0
    n_gaps = int(np.count_nonzero(dt > gap_thresh))
    if n_gaps:
        warnings.append(f"{n_gaps} sampling gap(s) longer than {gap_thresh*1e3:.0f} ms "
                        f"(worst {float(np.max(dt))*1e3:.0f} ms) - the Allan estimator "
                        "assumes a uniform sample period and ignores them")

    # Rough stillness check: did the platform get disturbed (picked up,
    # bumped) somewhere in the middle of an otherwise-static recording?
    # The two checks above look at global statistics, which a brief
    # disturbance in an 8 h recording can hide completely - this instead
    # buckets the recording into short windows and flags any window whose
    # PEAK gyro/accel stands out (see _windowed_diagnostics/_motion_segments).
    _progress("stillness check (windowed peaks)")
    diag = _windowed_diagnostics(gyr, acc, fs)
    bias_trace = _bias_trace(gyr, acc, fs, duration)
    motion_segments = _motion_segments(diag) if diag is not None else []
    for seg in motion_segments[:5]:
        warnings.append(
            f"possible motion at t={_fmt_hms(seg['t_start_s'])}-{_fmt_hms(seg['t_end_s'])} "
            f"(gyro peak {seg['gyr_peak_dps']:.2f} deg/s, accel dev {seg['acc_peak_mps2']:.2f} m/s^2)"
            " - check the IMU wasn't disturbed there, see the bias-trace plot")
    if len(motion_segments) > 5:
        warnings.append(f"... {len(motion_segments) - 5} more possible-motion segment(s), see the plot")

    result = {
        "n": n, "duration": duration, "fs": fs, "tau0": tau0,
        "tau_rrw": tau, "warnings": warnings, "curves": {},
        "rrw": {}, "arw": {}, "diag": diag, "bias_trace": bias_trace,
        "motion_segments": motion_segments,
        "rrw_resolved": {}, "rrw_fit_mask": {},
        "tau_fit_max": duration / MIN_CLUSTERS_FOR_FIT,
    }
    for name, series in (("gyr", gyr), ("acc", acc)):
        rrw_axis, arw_axis, resolved_axis, mask_axis, curves = [], [], [], [], []
        rise_axis = []
        for ax in range(3):
            _progress(f"Allan deviation: {name} {'xyz'[ax]}",
                      progress_base + len(rise_axis), 6)
            ta, ad = allan_deviation(series[:, ax], tau0, taus)
            curves.append((ta, ad))
            a = _analyze_axis(ta, ad, duration)
            arw_axis.append(a["arw"])             # N, white noise at tau=1 s
            rrw_axis.append(a["rrw"])             # K = bias_rw at tau=3 s
            resolved_axis.append(a["resolved"])
            mask_axis.append(a["mask"])
            rise_axis.append(a["tau_rise"])
        progress_base += 3
        result["curves"][name] = curves
        result["rrw"][name] = rrw_axis
        result["arw"][name] = arw_axis
        result["rrw_resolved"][name] = resolved_axis
        result["rrw_fit_mask"][name] = mask_axis

        dead = [a for a, k in zip("xyz", rrw_axis) if k is None]
        if dead:
            warnings.append(f"{name} {'/'.join(dead)}: zero Allan deviation "
                            "(constant channel?) - axis excluded from the estimates")
        unresolved = [(a, r) for a, ok, k, r
                      in zip("xyz", resolved_axis, rrw_axis, rise_axis)
                      if not ok and k is not None]
        if unresolved:
            axes_txt = "/".join(a for a, _ in unresolved)
            msg = (f"{name} {axes_txt}: no clean +1/2 branch within the trustworthy "
                   f"averaging times (tau <= {result['tau_fit_max']:.0f} s) - "
                   "the value below is an UPPER BOUND, not a measurement")
            # Turn "record longer" into an actual number: the branch has to
            # start and then still fit under the cluster cap.
            rises = [r for _, r in unresolved if r is not None]
            if rises:
                want = max(rises) * FIT_SPAN_FACTOR * MIN_CLUSTERS_FOR_FIT
                msg += (f". The branch appears near tau={max(rises):.0f} s, so fitting it "
                        f"wants roughly {want/3600.0:.0f} h of static data "
                        f"(this record: {duration/3600.0:.1f} h)")
            else:
                msg += (f". No +1/2 branch anywhere in this record ({duration/3600.0:.1f} h), "
                        "so the random walk is still buried under the bias-instability "
                        "floor - record substantially longer")
            warnings.append(msg)

    # Config scalars: one per sensor. bias_rw uses the WORST axis (a noise
    # model that is too small makes the filter over-trust its bias states),
    # the bonus psd uses the mean-variance across axes.
    for name in ("gyr", "acc"):
        usable = [(k, i) for i, k in enumerate(result["rrw"][name]) if k is not None]
        i_worst = max(usable)[1] if usable else 0
        result[f"{name}_bias_rw"] = result["rrw"][name][i_worst] or 0.0
        result[f"{name}_bias_rw_axis"] = i_worst
        result[f"{name}_bias_rw_resolved"] = result["rrw_resolved"][name][i_worst]
        arw_ok = [v for v in result["arw"][name] if v is not None]
        result[f"{name}_psd"] = float(np.mean(np.square(arw_ok))) if arw_ok else 0.0
    _progress_done(f"done, {len(warnings)} warning(s)\n")
    return result


def _load_config_imu(path):
    """Read the `imu:` section of a config.yaml (best effort).
    Returns a dict or {} - used only to print current-vs-suggested."""
    try:
        import yaml
    except ImportError:
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        return doc.get("imu", {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _fmt_now(cur, sugg):
    if cur and cur > 0:
        return f"  (now {cur:.3e}, x{sugg / cur:.1f})"
    return "  (not set)"


def format_report(result, config_imu=None):
    """The printed/GUI-shown summary for an analyze() result."""
    config_imu = config_imu or {}
    r = result
    out = []
    out.append(f"Allan-variance IMU noise analysis "
               f"(n={r['n']}, {r['duration']:.0f} s @ {r['fs']:.1f} Hz)")
    for w in r["warnings"]:
        out.append(f"  WARNING: {w}")

    def _axes(vals, unit, resolved=None):
        """One line of per-axis values. A value that came from the upper
        bound rather than a real fit is prefixed '<=' so it can never be
        mistaken for a measurement."""
        cells = []
        for i, v in enumerate(vals):
            if v is None:
                cells.append(f"{'xyz'[i]}: n/a")
                continue
            lead = "" if resolved is None or resolved[i] else "<="
            cells.append(f"{'xyz'[i]}: {lead}{v:.3e}")
        return "    " + "   ".join(cells) + f"  {unit}"

    out.append(f"  gyro bias random walk (RRW), +1/2 asymptote at tau={r['tau_rrw']:g} s:")
    out.append(_axes(r["rrw"]["gyr"], "rad/s/sqrt(s)", r["rrw_resolved"]["gyr"]))
    out.append(f"  accel bias random walk (RRW), +1/2 asymptote at tau={r['tau_rrw']:g} s:")
    out.append(_axes(r["rrw"]["acc"], "m/s^2/sqrt(s)", r["rrw_resolved"]["acc"]))

    out.append("  suggested bias random walk for config.yaml `imu:` "
               "(worst axis):")
    for key, unit in (("gyr_bias_rw", "rad/s/sqrt(s)"), ("acc_bias_rw", "m/s^2/sqrt(s)")):
        val = r[key]
        note = "" if r[f"{key[:3]}_bias_rw_resolved"] else "  UPPER BOUND (not resolved)"
        out.append(f"    {key}: {val:.3e}   # {unit}"
                   f"{_fmt_now(config_imu.get(key), val)}{note}")

    # N (the ARW/VRW coefficient) is an AMPLITUDE spectral density; the
    # config's *_psd is the POWER one, i.e. N^2. Both are printed because
    # datasheets quote the former and the filter wants the latter, and
    # reporting only one invites squaring it twice or not at all.
    out.append("  gyro white noise (ARW), -1/2 asymptote at tau=1 s:")
    out.append(_axes(r["arw"]["gyr"], "rad/s/sqrt(Hz)  (= rad/sqrt(s))"))
    out.append("  accel white noise (VRW), -1/2 asymptote at tau=1 s:")
    out.append(_axes(r["arw"]["acc"], "m/s^2/sqrt(Hz)"))

    out.append("  config psd = that coefficient SQUARED (PSD - a power "
               "spectral density):")
    out.append(f"    gyr_psd: {r['gyr_psd']:.3e}   # (rad/s)^2/Hz"
               f"{_fmt_now(config_imu.get('gyr_psd'), r['gyr_psd'])}")
    out.append(f"    acc_psd: {r['acc_psd']:.3e}   # (m/s^2)^2/Hz"
               f"{_fmt_now(config_imu.get('acc_psd'), r['acc_psd'])}")
    return "\n".join(out)


def _fmt_hms(seconds):
    """Format a duration in seconds as H:MM:SS.s. Rounds to hundredths
    BEFORE splitting into h/m/s, so e.g. 599.999 carries to 0:10:00.00
    instead of the display rounding its seconds field up to a stray
    "0:09:60.00"."""
    total = round(seconds, 2)
    h = int(total // 3600)
    m = int((total % 3600) // 60)
    s = total - h * 3600 - m * 60
    return f"{h:d}:{m:02d}:{s:05.2f}"


def _ref_line_anchors(result, name):
    """(x_left, x_right, k_rrw, n_arw) for one sensor's ('gyr'/'acc')
    reference lines: k_rrw/n_arw are the worst-axis RRW (+1/2 slope,
    tau=3 s) / ARW-VRW (-1/2 slope, tau=1 s) read-outs used to draw the
    ideal slope lines through the actual read-out points. x_left rounds
    DOWN to the decade at/below tau0 - the RRW line, extrapolated back
    to short tau, sits several decades below the data floor there, so the
    y-axis needs that headroom too (see plot_curves' ylim), while a left
    limit above tau0 (e.g. a flat 1e-2 s for a ~500 Hz IMU whose tau0 is
    already ~2e-3 s) would instead clip off real short-tau samples."""
    tau0 = result["tau0"]
    x_left = 10.0 ** math.floor(math.log10(tau0))
    all_ta = np.concatenate([ta for ta, _ in result["curves"][name]])
    x_right = float(np.max(all_ta))
    k_rrw = result[f"{name}_bias_rw"]
    arw_ok = [v for v in result["arw"][name] if v is not None]
    n_arw = max(arw_ok) if arw_ok else 0.0
    return x_left, x_right, k_rrw, n_arw


def _wrap_report_text(text, width=100):
    """Word-wrap each line of a report to `width` chars, keeping its
    leading whitespace as indent (continuation lines get 2 extra spaces)
    - format_report()'s WARNING lines especially can run well past a
    monospace page's usable width otherwise."""
    out = []
    for line in text.split("\n"):
        stripped = line.lstrip()
        indent = line[:len(line) - len(stripped)]
        if not stripped:
            out.append(line)
            continue
        wrapped = textwrap.wrap(stripped, width=max(20, width - len(indent))) or [""]
        out.append(indent + wrapped[0])
        out.extend(indent + "  " + cont for cont in wrapped[1:])
    return "\n".join(out)


def _report_page(result, config_imu):
    """Text-only page for the --plot PDF's 2nd page: dataset stats plus the
    same report printed to the console (format_report)."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(8.5, 11))
    fig.text(0.5, 0.97, "Allan-variance IMU noise analysis - report",
             ha="center", fontsize=13, weight="bold")
    stats = (
        f"samples        : {result['n']}\n"
        f"duration       : {result['duration']:.1f} s  ({_fmt_hms(result['duration'])})\n"
        f"sample rate    : {result['fs']:.2f} Hz\n"
        f"sample period  : {result['tau0'] * 1000:.3f} ms\n"
    )
    body = _wrap_report_text(stats + "\n" + format_report(result, config_imu))
    fig.text(0.06, 0.90, body, va="top", ha="left", fontsize=9, family="monospace")
    return fig


def _text_size_frac(fig, text, fontsize):
    """Rendered (width, height) of `text` as a fraction of the figure. Used
    to lay the formulas page out from measurements rather than guessed
    coordinates: mathtext extent follows from neither the source length nor
    the font size (a tall \\sum or \\frac overruns a plain line pitch)."""
    t = fig.text(0.0, 0.5, text, fontsize=fontsize)
    try:
        bbox = t.get_window_extent(renderer=fig.canvas.get_renderer())
    finally:
        t.remove()
    bbox = bbox.transformed(fig.transFigure.inverted())
    return float(bbox.width), float(bbox.height)


def _text_width_frac(fig, text, fontsize):
    """Rendered width of `text` as a fraction of the figure width."""
    return _text_size_frac(fig, text, fontsize)[0]


def _split_tokens(text):
    """Split on spaces, but keep ``$...$`` math spans atomic: mathtext has
    to be handed to matplotlib whole, so a line break inside one would
    render as literal dollar signs."""
    tokens, cur, in_math = [], "", False
    for ch in text:
        if ch == "$":
            in_math = not in_math
            cur += ch
        elif ch == " " and not in_math:
            if cur:
                tokens.append(cur)
                cur = ""
        else:
            cur += ch
    if cur:
        tokens.append(cur)
    return tokens


def _wrap_to_width(fig, text, fontsize, max_w):
    """Greedy word wrap driven by MEASURED width rather than a character
    count -- with mathtext mixed into the prose, source length says very
    little about how wide a line ends up."""
    lines, cur = [], ""
    for tok in _split_tokens(text):
        trial = tok if not cur else cur + " " + tok
        if cur and _text_width_frac(fig, trial, fontsize) > max_w:
            lines.append(cur)
            cur = tok
        else:
            cur = trial
    if cur:
        lines.append(cur)
    return lines


def _formulas_page():
    """Reference page: the Allan-deviation estimator and how the ARW/RRW
    read-outs map to the Kalman filter's bias_rw/psd config parameters.

    Laid out by measurement, not by hand-placed coordinates: prose is word-
    wrapped to the column, the body font shrinks until the widest unbreakable
    formula fits, and the line pitch is divided out of the remaining height.
    Editing the text below therefore cannot silently push a line off the
    right edge or past the bottom, and prose never has to be hand-broken."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(8.5, 11))
    fig.text(0.5, 0.97, "Allan Variance - Formulas", ha="center",
             fontsize=13, weight="bold")

    # "p" prose (word-wrapped), "m" math (one line, mathtext cannot break),
    # "gap" extra space between sections.
    blocks = [
        ("p", r"Overlapping Allan variance of a rate/accel series $x$ sampled at"
              r" $\tau_0$ ($\theta$ = its running integral, $m=\tau/\tau_0$):"),
        ("m", r"$\sigma^2(\tau) = \frac{1}{2\tau^2 M}\sum_{k=1}^{M}"
              r"\left(\theta_{k+2m} - 2\theta_{k+m} + \theta_k\right)^2\ ,\quad"
              r"\theta_k=\sum_{i<k} x_i\,\tau_0$"),
        ("gap", ""),
        ("p", r"The measured curve is the SUM of all noise terms, so each coefficient"
              r" comes from the ASYMPTOTE fitted where that term dominates, not from"
              r" the raw curve at the read-out $\tau$: at $\tau=3\,$s a MEMS IMU is"
              r" still white-noise dominated, which would inflate $K$ by 10-100x."),
        ("gap", ""),
        ("p", r"White noise (ARW for gyro, VRW for accel): slope $-1/2$,"
              r" read out at $\tau=1\,$s:"),
        ("m", r"$\sigma(\tau) = \frac{N}{\sqrt{\tau}} \ \Rightarrow\ "
              r"N = \sigma_{-1/2}(1\,\mathrm{s})"
              r"\qquad\mathrm{gyr\_psd}=N_{gyr}^2\ \ [(\mathrm{rad/s})^2/\mathrm{Hz}]"
              r"\ ,\ \ \mathrm{acc\_psd}=N_{acc}^2\ \ [(\mathrm{m/s^2})^2/\mathrm{Hz}]$"),
        ("gap", ""),
        ("p", r"Bias random walk (RRW): slope $+1/2$, read out at $\tau=3\,$s"
              r" (IEEE Std 952):"),
        ("m", r"$\sigma(\tau) = K\sqrt{\tau/3} \ \Rightarrow\ "
              r"K = \sigma_{+1/2}(3\,\mathrm{s})$"),
        ("gap", ""),
        ("p", r"$K$ is exactly the process-noise density the filter's bias_rw"
              r" parameter expects (units $[\mathrm{unit}/\sqrt{s}]$). Per Kalman"
              r" predict step, and over an unaided coast of $T$ seconds:"),
        ("m", r"$\mathrm{Var}(\Delta t) = K^2\,\Delta t \qquad \sigma(T) = K\sqrt{T}$"),
        ("gap", ""),
        ("p", r"$\Rightarrow$ config: $\mathrm{gyr\_bias\_rw}=K_{gyr}$"
              r" $[\mathrm{rad/s}/\sqrt{s}]$,"
              r" $\mathrm{acc\_bias\_rw}=K_{acc}$ $[\mathrm{m/s^2}/\sqrt{s}]$"),
    ]

    x_left, x_right = 0.06, 0.97
    y_top, y_bottom = 0.93, 0.03
    col_w = x_right - x_left
    avail_h = y_top - y_bottom
    fig_h_pt = fig.get_size_inches()[1] * 72.0
    leading = 1.5        # line pitch as a multiple of the font size
    gap_mult = 1.0       # a section gap, as a multiple of that pitch

    # Fit iteratively: the wrap depends on the font size, and the font size
    # depends on how many lines the wrap produced. Two or three passes is
    # plenty, the loop just guarantees termination.
    fontsize = 12.0
    steps = []
    for _ in range(6):
        # An unbreakable math line sets the smallest font the page can use.
        widest_math = max((_text_width_frac(fig, txt, fontsize)
                           for kind, txt in blocks if kind == "m"), default=0.0)
        if widest_math > col_w:
            fontsize *= col_w / widest_math

        pitch = leading * fontsize / fig_h_pt
        steps = []                      # (text or None, advance)
        for kind, txt in blocks:
            if kind == "gap":
                steps.append((None, pitch * gap_mult))
            elif kind == "m":
                # A tall formula (\sum, \frac) needs more than the nominal
                # pitch or the next line lands on top of its descender.
                _, h = _text_size_frac(fig, txt, fontsize)
                steps.append((txt, max(pitch, h * 1.15)))
            else:
                for ln in _wrap_to_width(fig, txt, fontsize, col_w):
                    steps.append((ln, pitch))

        need = sum(adv for _, adv in steps)
        if need <= avail_h:
            break
        fontsize *= avail_h / need

    # Fixed pitch from the top down, NOT stretched to fill the page: a
    # reference page should read as a text block with white space under it,
    # not as lines floating a third of a page apart.
    y = y_top
    for line, advance in steps:
        if line is not None:
            fig.text(x_left, y, line, va="top", ha="left", fontsize=fontsize)
        y -= advance
    return fig


def _bias_motion_page(result):
    """4-panel page. Top row is the long-window MEAN gyro/accel trace over
    the whole recording - with the white noise averaged down, what's left IS
    essentially the bias wander, visualized directly as a time series (as
    opposed to the Allan-deviation summary statistic on page 1), which is
    also what makes a thermal warm-up transient visible. Bottom row is the
    SHORT-window peak deviation from rest (the buckets analyze() used for
    the stillness check); any window that tripped the motion threshold is
    shaded red on all four panels, so a plot inspection also answers "was
    the IMU disturbed mid-recording". The two rows deliberately use
    different window lengths (see _bias_trace / _windowed_diagnostics).
    None if the recording was too short to window at all."""
    import matplotlib.pyplot as plt
    diag = result.get("diag")
    trace = result.get("bias_trace")
    if diag is None or trace is None:
        return None
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))
    t_h = diag["t_center_s"] / 3600.0
    t_h_trace = trace["t_center_s"] / 3600.0
    trace_win = trace["window_sec"]

    # Relative to each axis' own median (robust to a brief motion outlier):
    # this is what makes the WANDER comparable across axes - accel z
    # otherwise sits at +g, dwarfing x/y on the same scale.
    def _robust_ylim(data, pad=1.3):
        lo, hi = np.percentile(data, [0.5, 99.5])
        if lo == hi:
            lo, hi = lo - 1.0, hi + 1.0
        span = (hi - lo) * pad
        mid = (hi + lo) / 2.0
        return mid - span / 2.0, mid + span / 2.0

    def _win_txt(sec):
        return f"{sec/60.0:.0f} min" if sec >= 90.0 else f"{sec:.0f} s"

    gyr_trace = np.degrees(trace["gyr_mean"]) * 3600.0                 # deg/h
    gyr_trace -= np.median(gyr_trace, axis=0)
    for i, lbl in enumerate("xyz"):
        axes[0, 0].plot(t_h_trace, gyr_trace[:, i], lw=0.8, label=lbl)
    axes[0, 0].set_ylim(*_robust_ylim(gyr_trace))
    axes[0, 0].set_title(f"gyro bias trace ({_win_txt(trace_win)} mean, rel. to median)")
    axes[0, 0].set_ylabel("bias [deg/h]")

    acc_trace = trace["acc_mean"] * 1000.0                             # mm/s^2
    acc_trace -= np.median(acc_trace, axis=0)
    for i, lbl in enumerate("xyz"):
        axes[0, 1].plot(t_h_trace, acc_trace[:, i], lw=0.8, label=lbl)
    axes[0, 1].set_ylim(*_robust_ylim(acc_trace))
    axes[0, 1].set_title(f"accel bias trace ({_win_txt(trace_win)} mean, rel. to median)")
    axes[0, 1].set_ylabel("bias [mm/s^2]")

    axes[1, 0].semilogy(t_h, np.degrees(diag["gyr_peak_rps"]), color="tab:gray", lw=0.6)
    axes[1, 0].axhline(MOTION_GYR_THRESH_DPS, color="tab:red", ls="--", lw=1,
                       label=f"motion threshold {MOTION_GYR_THRESH_DPS:g} deg/s")
    axes[1, 0].set_title("gyro peak deviation from rest (stillness check)")
    axes[1, 0].set_ylabel("peak |gyro - median| [deg/s]")

    axes[1, 1].semilogy(t_h, diag["acc_peak_mps2"], color="tab:gray", lw=0.6)
    axes[1, 1].axhline(MOTION_ACC_THRESH_MPS2, color="tab:red", ls="--", lw=1,
                       label=f"motion threshold {MOTION_ACC_THRESH_MPS2:g} m/s^2")
    axes[1, 1].set_title("accel peak deviation from rest (stillness check)")
    axes[1, 1].set_ylabel("peak |accel - median| [m/s^2]")

    segments = result.get("motion_segments") or []
    for seg in segments:
        t0, t1 = seg["t_start_s"] / 3600.0, seg["t_end_s"] / 3600.0
        for axp in axes.flat:
            axp.axvspan(t0, t1, color="tab:red", alpha=0.15)

    for axp in axes.flat:
        axp.set_xlabel("time [h]")
        axp.grid(alpha=0.3)
        axp.legend(fontsize=7)

    status = ("no motion detected - looks static throughout" if not segments else
             f"{len(segments)} possible-motion segment(s) flagged (shaded red)")
    fig.suptitle(f"Bias random-walk trace & stillness check - {status}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def plot_curves(result, out_path, config_imu=None):
    """Write the per-axis Allan-deviation log-log plot (needs matplotlib).
    A .pdf gets 3 extra pages: the console report, a formula reference, and
    a bias-trace/stillness-check time series; any other extension (e.g.
    .png) only supports the single plot page."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("--plot needs matplotlib (pip install matplotlib)")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    tau_rrw = result["tau_rrw"]
    tau_fit_max = result["tau_fit_max"]
    for axi, (name, unit) in enumerate((("gyr", "rad/s"), ("acc", "m/s^2"))):
        axp = axes[axi]
        for ax, (ta, ad) in enumerate(result["curves"][name]):
            axp.loglog(ta, ad, lw=1.0, label="xyz"[ax])

        x_left, x_right, k_rrw, n_arw = _ref_line_anchors(result, name)
        i_worst = result[f"{name}_bias_rw_axis"]
        resolved = result[f"{name}_bias_rw_resolved"]

        # Both reference lines are the FITTED asymptotes of the worst axis,
        # extrapolated across the whole tau range and evaluated at their
        # convention read-out points (the dots).
        rrw_x = np.array([x_left, x_right])
        rrw_y = k_rrw * (rrw_x / tau_rrw) ** 0.5
        kind = "fit" if resolved else "UPPER BOUND"
        axp.loglog(rrw_x, rrw_y, "--", color="tab:red", lw=1.2,
                  label=f"RRW +1/2 asymptote, {kind} ({'xyz'[i_worst]})")
        axp.plot(tau_rrw, k_rrw, "o", color="tab:red", ms=5)
        axp.axvline(tau_rrw, color="tab:red", ls=":", lw=0.7, alpha=0.5)

        # Mark the curve points the +1/2 fit was actually taken from, so the
        # drawn line can be checked against the data it came from.
        if resolved:
            ta_w, ad_w = result["curves"][name][i_worst]
            mask = result["rrw_fit_mask"][name][i_worst]
            axp.plot(ta_w[mask], ad_w[mask], "o", color="tab:red", ms=4,
                     mfc="none", label="points used for the +1/2 fit")

        arw_x = np.array([x_left, x_right])
        arw_y = n_arw * (arw_x / ARW_TAU_S) ** -0.5
        axp.loglog(arw_x, arw_y, "--", color="tab:blue", lw=1.2,
                  label="white noise -1/2 asymptote, fit")
        axp.plot(ARW_TAU_S, n_arw, "o", color="tab:blue", ms=5)
        axp.axvline(ARW_TAU_S, color="tab:blue", ls=":", lw=0.7, alpha=0.5)

        # Past this tau too few clusters remain for the estimate to be
        # trustworthy - shown, but never fitted.
        if tau_fit_max < x_right:
            axp.axvspan(tau_fit_max, x_right, color="gray", alpha=0.12,
                        label=f"< {MIN_CLUSTERS_FOR_FIT} clusters (not fitted)")

        y_all = np.concatenate(
            [ad for _, ad in result["curves"][name]] + [rrw_y, arw_y])
        axp.set_xlim(x_left, x_right)
        axp.set_ylim(float(np.min(y_all)) / 1.5, float(np.max(y_all)) * 1.5)

        axp.set_title(f"{name} Allan deviation")
        axp.set_xlabel("tau [s]")
        axp.set_ylabel(f"sigma [{unit}]")
        axp.grid(True, which="both", alpha=0.3)
        axp.legend(fontsize=6.5, loc="upper left")
    fig.tight_layout()

    if os.path.splitext(out_path)[1].lower() == ".pdf":
        from matplotlib.backends.backend_pdf import PdfPages
        pages = [("Allan-deviation curves", lambda: fig),
                 ("report", lambda: _report_page(result, config_imu)),
                 ("formulas", _formulas_page),
                 ("bias trace / stillness", lambda: _bias_motion_page(result))]
        with PdfPages(out_path) as pdf:
            for i, (label, build) in enumerate(pages):
                _progress(f"rendering page: {label}", i, len(pages))
                page = build()
                if page is not None:
                    pdf.savefig(page, dpi=120)
        _progress_done(f"{out_path} written ({len(pages)} pages)")
    else:
        fig.savefig(out_path, dpi=120)
        _progress_done(f"{out_path} written")


def main():
    ap = argparse.ArgumentParser(
        description="Estimate IMU bias random walk (config imu.*_bias_rw) "
                    "from a long static recording via Allan variance.")
    ap.add_argument("imu", help="imu.csv or a dataset directory")
    ap.add_argument("--config", help="config.yaml to compare current values against")
    ap.add_argument("--plot", help="write the Allan-deviation plot to this PNG/PDF"
                    " (.pdf also gets the report, formulas and bias-trace pages)")
    ap.add_argument("--skip-start", type=float, default=0.0, metavar="SEC",
                    help="discard the first SEC seconds - gyro thermal warm-up"
                         " drift is a deterministic ramp that inflates the RRW,"
                         " and the start is also where handling shows up")
    ap.add_argument("--skip-end", type=float, default=0.0, metavar="SEC",
                    help="discard the last SEC seconds (handling when stopping)")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the progress output on stderr")
    args = ap.parse_args()

    global _QUIET
    _QUIET = args.quiet
    result = analyze(args.imu, skip_start_sec=args.skip_start,
                     skip_end_sec=args.skip_end)
    config_imu = _load_config_imu(args.config) if args.config else {}
    print(format_report(result, config_imu))
    if args.plot:
        plot_curves(result, args.plot, config_imu)


if __name__ == "__main__":
    main()
