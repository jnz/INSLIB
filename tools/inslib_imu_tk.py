"""Multi-position IMU calibration without a fixture (imu_tk, ICRA 2014).

A numpy port of the IMU-TK method (BSD, Alberto Pretto et al.,
https://bitbucket.org/alberto_pretto/imu_tk), described in

    D. Tedaldi, A. Pretto, E. Menegatti, "A Robust and Easy to Implement
    Method for IMU Calibration without External Equipments", Proc. IEEE
    International Conference on Robotics and Automation (ICRA), 2014,
    pp. 3042-3049.

The reference recording the port is checked against is kept in
tools/testdata/ (see its README and python/tests/test_imu_tk.py).

Why this and not the classic 6-position tumble test: the accelerometer
cost here is

    residual = |g| - || T*K*(a_raw - b) ||

which uses only the MAGNITUDE of gravity. The orientation of a static
pose never enters it, so the poses do not have to be axis-aligned, level,
or known at all. You hold the unit, put it down somewhere, leave it for a
few seconds, pick it up, turn it, put it down again. The static periods
are found automatically from a rolling variance.

The tumble test, by contrast, estimates the bias as (up + down)/2 per
axis pair, which is exact only when the two halves of a pair are exactly
opposite each other. Measured on this repo's own solver, an independent
1 deg placement error on each position puts ~0.11 m/s^2 into the
accelerometer bias, which is larger than the bias of a decent MEMS part.

The gyroscope gets scale and full misalignment out of the same recording
and still needs no turntable: between two static poses the accelerometer
knows both gravity directions, and the gyro integrated over the rotation
between them has to carry the first onto the second. The residual is that
mismatch.

Sensor model, identical to imu_tk's CalibratedTriad and to INSLIB's
REQ-NAV-037 keys, so the result is written out without conversion:

    corrected = M * (raw - bias),   M = T * K

           [ 1  -mis_yz   mis_zy ]                [ s_x          ]
    T_acc =[ 0     1     -mis_zx ]   K = diag ... [     s_y      ]
           [ 0     0        1    ]                [         s_z  ]

The accelerometer triad DEFINES the body frame (its T is upper
triangular, 3 free terms); the gyro is aligned to that frame and gets all
6. Hamilton quaternions, q[0] = w, as everywhere else in this repo.

Pure numpy: the Levenberg-Marquardt below replaces Ceres, which for 9 and
12 parameters is not a sacrifice worth a dependency.

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import math

import numpy as np

# imu_tk's own defaults (MultiPosCalibration_'s constructor).
DEFAULT_WIN_SIZE = 101          # rolling-variance window, samples
DEFAULT_MIN_INTERVALS = 12      # fewer static poses than this is not a dataset
DEFAULT_INTERVAL_SAMPLES = 100  # samples taken from each static interval
DEFAULT_TH_MULTIPLIERS = range(2, 11)


# ===========================================================================
# Sensor model
# ===========================================================================

def acc_matrix(params):
    """M = T*K for the accelerometer triad (9-parameter vector).

    params = [mis_yz, mis_zy, mis_zx, s_x, s_y, s_z, b_x, b_y, b_z]"""
    mis_yz, mis_zy, mis_zx = params[0], params[1], params[2]
    t = np.array([[1.0, -mis_yz, mis_zy],
                  [0.0, 1.0, -mis_zx],
                  [0.0, 0.0, 1.0]])
    return t @ np.diag(params[3:6])


def gyr_matrix(params):
    """M = T*K for the gyro triad (12-parameter vector).

    params = [mis_yz, mis_zy, mis_zx, mis_xz, mis_xy, mis_yx,
              s_x, s_y, s_z, b_x, b_y, b_z]"""
    mis_yz, mis_zy, mis_zx, mis_xz, mis_xy, mis_yx = params[0:6]
    t = np.array([[1.0, -mis_yz, mis_zy],
                  [mis_xz, 1.0, -mis_zx],
                  [-mis_xy, mis_yx, 1.0]])
    return t @ np.diag(params[6:9])


def apply_calib(raw, matrix, bias):
    """corrected = M * (raw - bias), for an (N,3) block of samples."""
    return (np.asarray(raw) - np.asarray(bias)) @ np.asarray(matrix).T


# ===========================================================================
# Static-interval detection
# ===========================================================================

def rolling_variance_norm(x, win_size):
    """||var|| over a centred window of win_size samples, per sample.

    Same quantity imu_tk's staticIntervalsDetector thresholds: the
    unbiased (n-1) per-axis variance of the window, as a 3-vector norm.
    Positions outside the half-window are NaN and never count as static."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if win_size < 11:
        win_size = 11
    if win_size % 2 == 0:
        win_size += 1
    h = win_size // 2
    out = np.full(n, np.nan)
    if win_size >= n:
        return out
    # Cumulative sums give every window in one pass; the alternative is
    # an O(n*win) loop, and win is 101 by default.
    c1 = np.concatenate([np.zeros((1, 3)), np.cumsum(x, axis=0)])
    c2 = np.concatenate([np.zeros((1, 3)), np.cumsum(x * x, axis=0)])
    i = np.arange(h, n - h)
    lo, hi = i - h, i + h + 1          # window is [i-h, i+h] inclusive
    m = win_size
    s1 = c1[hi] - c1[lo]
    s2 = c2[hi] - c2[lo]
    var = (s2 - s1 * s1 / m) / (m - 1)
    np.maximum(var, 0.0, out=var)      # rounding can dip below zero
    out[i] = np.sqrt((var * var).sum(axis=1))
    return out


def static_intervals(acc, threshold, win_size=DEFAULT_WIN_SIZE):
    """Index ranges (start, end) inclusive where the unit was at rest."""
    norm = rolling_variance_norm(acc, win_size)
    still = np.isfinite(norm) & (norm < threshold)
    if not still.any():
        return []
    # Rising/falling edges of the boolean run, which is what imu_tk's
    # look_for_start state machine amounts to.
    d = np.diff(still.astype(np.int8))
    starts = list(np.flatnonzero(d == 1) + 1)
    ends = list(np.flatnonzero(d == -1))
    if still[0]:
        starts.insert(0, 0)
    if still[-1]:
        ends.append(len(still) - 1)
    return list(zip(starts, ends))


def interval_samples(x, intervals, n_samples=DEFAULT_INTERVAL_SAMPLES,
                     means=False):
    """Samples taken out of each long-enough static interval.

    Returns (values, kept_intervals). means=False takes the first
    n_samples of each interval as individual observations (what the
    accelerometer fit uses), means=True the mean over the whole interval
    (what the gyro fit uses as its gravity reference)."""
    x = np.asarray(x, dtype=float)
    vals, kept = [], []
    for (s, e) in intervals:
        if e - s + 1 < n_samples:
            continue
        kept.append((s, e))
        if means:
            vals.append(x[s:e + 1].mean(axis=0))
        else:
            vals.append(x[s:s + n_samples])
    if not kept:
        return np.zeros((0, 3)), []
    return (np.array(vals) if means else np.concatenate(vals)), kept


# ===========================================================================
# Levenberg-Marquardt (replaces Ceres)
# ===========================================================================

def _numeric_jacobian(fun, x, f0, rel_step=1e-7):
    """Forward differences with a relative step, which is all these two
    well-scaled problems need."""
    j = np.empty((len(f0), len(x)))
    for i in range(len(x)):
        h = rel_step * max(1.0, abs(x[i]))
        xp = x.copy()
        xp[i] += h
        j[:, i] = (fun(xp) - f0) / h
    return j


def lm_solve_free(fun, x0, free, **kw):
    """lm_solve over a SUBSET of the parameters, the rest pinned at x0.

    How "do not estimate the misalignment" is implemented: the off-
    diagonal terms simply stay at their initial zero instead of being
    given to the optimizer. Worth having, because those terms are the
    weakly observable ones -- with few or poorly spread poses they will
    happily absorb scale and bias error, and a diagonal model is then the
    more honest fit."""
    x0 = np.asarray(x0, dtype=float)
    free = np.asarray(free, dtype=int)

    def wrapped(xr):
        x = x0.copy()
        x[free] = xr
        return fun(x)

    xr, cost, it = lm_solve(wrapped, x0[free], **kw)
    x = x0.copy()
    x[free] = xr
    return x, cost, it


def lm_solve(fun, x0, max_iter=200, ftol=1e-12, xtol=1e-12, verbose=False):
    """Minimize 0.5*sum(fun(x)**2). Returns (x, cost, n_iter).

    `cost` is 0.5*sum(r^2), the same number Ceres reports as final_cost,
    so the threshold sweep below compares like with like."""
    x = np.array(x0, dtype=float)
    f = np.asarray(fun(x), dtype=float)
    cost = 0.5 * float(f @ f)
    lam = 1e-3
    n = len(x)
    for it in range(max_iter):
        jac = _numeric_jacobian(fun, x, f)
        jtj = jac.T @ jac
        jtf = jac.T @ f
        diag = np.diag(jtj).copy()
        diag[diag <= 0] = 1.0
        accepted = False
        for _ in range(30):
            try:
                dx = np.linalg.solve(jtj + lam * np.diag(diag), -jtf)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            xn = x + dx
            fn = np.asarray(fun(xn), dtype=float)
            cn = 0.5 * float(fn @ fn)
            if np.isfinite(cn) and cn < cost:
                improved = cost - cn
                step = float(np.linalg.norm(dx))
                x, f, cost = xn, fn, cn
                lam = max(lam * 0.3, 1e-12)
                accepted = True
                if improved < ftol * max(cost, 1.0) or step < xtol * (
                        1.0 + float(np.linalg.norm(x))):
                    return x, cost, it + 1
                break
            lam *= 10.0
            if lam > 1e14:
                return x, cost, it + 1
        if verbose:
            print("  lm it %3d cost %.6e lambda %.1e" % (it, cost, lam))
        if not accepted:
            break
    return x, cost, max_iter


# ===========================================================================
# Gyro integration (RK4, as a chain of linear maps)
# ===========================================================================

def _omega_skew(w):
    """0.5*Omega(w) for qdot = 0.5*Omega(w)*q, batched over (...,3)."""
    wx, wy, wz = w[..., 0], w[..., 1], w[..., 2]
    z = np.zeros_like(wx)
    s = np.stack([
        np.stack([z, -wx, -wy, -wz], axis=-1),
        np.stack([wx, z, wz, -wy], axis=-1),
        np.stack([wy, -wz, z, wx], axis=-1),
        np.stack([wz, wy, -wx, z], axis=-1),
    ], axis=-2)
    return 0.5 * s


def _rk4_step_matrices(w0, w1, dt):
    """The 4x4 map A with q_{n+1} = A*q_n for one RK4 step.

    imu_tk integrates with RK4 and renormalizes after every step. Because
    qdot = 0.5*Omega*q is LINEAR in q, each of its four coefficients is a
    matrix acting on q and the whole step collapses to one matrix. The
    per-step renormalization is a division by a scalar, and scalars
    commute with the matrices, so normalizing once at the end gives the
    identical rotation: the chain can be reduced pairwise instead of
    stepped through one sample at a time."""
    eye = np.eye(4)
    s0 = _omega_skew(w0)
    sm = _omega_skew(0.5 * (w0 + w1))
    s1 = _omega_skew(w1)
    dt = dt[..., None, None]
    k1 = s0
    k2 = sm @ (eye + 0.5 * dt * k1)
    k3 = sm @ (eye + 0.5 * dt * k2)
    k4 = s1 @ (eye + dt * k3)
    return eye + dt * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0)


def _chain_reduce(mats):
    """Product mats[:, -1] @ ... @ mats[:, 0] for a batch of chains.

    Pairwise tree reduction: log2(L) batched matmuls instead of L
    sequential ones, which is what makes the gyro fit tolerable in
    Python (the Jacobian re-integrates everything 13 times per step)."""
    p = mats
    while p.shape[1] > 1:
        if p.shape[1] % 2:
            pad = np.broadcast_to(np.eye(p.shape[-1]),
                                  (p.shape[0], 1) + p.shape[-2:])
            p = np.concatenate([p, pad], axis=1)
        p = p[:, 1::2] @ p[:, 0::2]     # later step applies on the left
    return p[:, 0]


def _quat_to_rotmat(q):
    """Hamilton quaternion (w,x,y,z) -> rotation matrix, batched."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
                  2 * (x * z + w * y)], axis=-1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
                  2 * (y * z - w * x)], axis=-1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x),
                  1 - 2 * (x * x + y * y)], axis=-1),
    ], axis=-2)


class _RotationBatch:
    """The rotation intervals between consecutive static poses, padded to
    a rectangle so one evaluation integrates all of them at once."""

    def __init__(self, gyr_raw, t, spans):
        self.gyr_raw = np.asarray(gyr_raw, dtype=float)
        lengths = [b - a for (a, b) in spans]
        self.n = len(spans)
        self.width = max(lengths) if lengths else 0
        # idx[k, j] is the sample index of step j of interval k; padding
        # repeats the last index and gets dt = 0, i.e. an identity step.
        self.idx = np.zeros((self.n, self.width), dtype=np.intp)
        self.dt = np.zeros((self.n, self.width))
        for k, (a, b) in enumerate(spans):
            m = b - a
            self.idx[k, :m] = np.arange(a, b)
            self.idx[k, m:] = b
            self.dt[k, :m] = np.diff(t[a:b + 1])

    def rotations(self, matrix, bias):
        """Integrated rotation of each interval, for this calibration."""
        if self.n == 0 or self.width == 0:
            return np.zeros((0, 3, 3))
        w = apply_calib(self.gyr_raw, matrix, bias)
        a = _rk4_step_matrices(w[self.idx], w[self.idx + 1], self.dt)
        q = _chain_reduce(a)[:, :, 0]      # A_total applied to (1,0,0,0)
        return _quat_to_rotmat(q)


# ===========================================================================
# Before/after statistics
# ===========================================================================

def acc_error_stats(samples, matrix, bias, g_mag):
    """How far |a| is from |g| over the static poses, for one calibration.

    The one number that says whether a calibration is worth anything: at
    rest the accelerometer must read exactly gravity, in every pose, and
    any misalignment/scale/bias error shows up here. It is also the
    quantity being minimized, so quoting it before and after is quoting
    the fit's own objective."""
    cal = apply_calib(samples, matrix, bias)
    err = np.linalg.norm(cal, axis=1) - g_mag
    return {"rms": float(np.sqrt(np.mean(err ** 2))),
            "mean": float(np.mean(err)),
            "max": float(np.max(np.abs(err)))}


def gyr_error_stats(batch, v0, v1, matrix, bias):
    """Rms angle [rad] by which the integrated gyro misses the gravity
    direction the accelerometer measured at the next pose.

    Reported for the uncalibrated sensor too, where the rotation can
    integrate to something enormous: a raw scale that is out by orders of
    magnitude overflows the quaternion chain. That is a legitimate answer
    to "how bad was it before" and comes back as NaN rather than as a
    numpy warning on the operator's console."""
    if batch.n == 0:
        return {"rms": float("nan")}
    with np.errstate(over="ignore", invalid="ignore"):
        rot = batch.rotations(matrix, bias)
        if not np.isfinite(rot).all():
            return {"rms": float("nan")}
        pred = np.einsum("kji,kj->ki", rot, v0)
        dot = np.clip((pred * v1).sum(axis=1), -1.0, 1.0)
        ang = np.arccos(dot)
    return {"rms": float(np.sqrt(np.mean(ang ** 2)))}


# ===========================================================================
# The calibration itself
# ===========================================================================

class CalibResult:
    """One solved calibration, in the units the config.yaml wants."""

    def __init__(self):
        self.acc_matrix = np.eye(3)
        self.acc_bias = np.zeros(3)
        self.gyr_matrix = np.eye(3)
        self.gyr_bias = np.zeros(3)
        self.acc_cost = float("nan")
        self.gyr_cost = float("nan")
        self.th_multiplier = 0
        self.intervals = []       # (start, end) of the accepted static poses
        self.n_positions = 0
        self.residual_rms = float("nan")   # |g| error after calibration [m/s^2]
        self.gyr_residual_rms = float("nan")  # gravity-direction mismatch [-]
        self.gyr_solved = False
        # Fraction of the declared rest period that was really at rest,
        # and the sample mask for it (the noise model uses the same one).
        self.init_static_frac = 1.0
        self.init_static_mask = None
        # Before/after, keyed "raw" / "bias" / "full" (see calibrate()).
        self.acc_stats = {}
        self.gyr_stats = {}
        self.misalignment_estimated = True
        # Per static pose, for plotting: |a| before and after, and the
        # mean rate before and after (which at rest must be zero).
        self.pose_acc_raw = None      # (n,)  [m/s^2]
        self.pose_acc_cal = None      # (n,)
        self.pose_gyr_raw = None      # (n,3) [rad/s]
        self.pose_gyr_cal = None      # (n,3)

    def acc_misalignment_colmajor(self):
        return [float(self.acc_matrix[i][j]) for j in range(3) for i in range(3)]

    def gyr_misalignment_colmajor(self):
        return [float(self.gyr_matrix[i][j]) for j in range(3) for i in range(3)]

    def acc_scale_percent(self):
        """Per-axis scale correction as a percentage, for the report."""
        return [(float(np.linalg.norm(self.acc_matrix[:, j])) - 1.0) * 100.0
                for j in range(3)]

    def gyr_scale_percent(self):
        return [(float(np.linalg.norm(self.gyr_matrix[:, j])) - 1.0) * 100.0
                for j in range(3)]


def calibrate_accel(acc, g_mag, threshold, win_size=DEFAULT_WIN_SIZE,
                    n_samples=DEFAULT_INTERVAL_SAMPLES,
                    min_intervals=DEFAULT_MIN_INTERVALS, init=None,
                    estimate_misalignment=True):
    """Fit the 9 accelerometer parameters at one static threshold.

    Returns (params, cost, intervals) or None when the threshold does not
    yield enough static poses."""
    ivals = static_intervals(acc, threshold, win_size)
    samples, kept = interval_samples(acc, ivals, n_samples, means=False)
    if len(kept) < min_intervals:
        return None

    def residual(p):
        cal = apply_calib(samples, acc_matrix(p), p[6:9])
        return g_mag - np.linalg.norm(cal, axis=1)

    x0 = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]) \
        if init is None else np.array(init, dtype=float)
    # Off-diagonal terms first in the vector, so "scale and bias only" is
    # simply the tail of it.
    free = range(9) if estimate_misalignment else range(3, 9)
    p, cost, _ = lm_solve_free(residual, x0, list(free))
    return p, cost, kept, samples


def calibrate(acc, gyr, t, g_mag=9.80665, init_static_sec=None,
              win_size=DEFAULT_WIN_SIZE,
              n_samples=DEFAULT_INTERVAL_SAMPLES,
              min_intervals=DEFAULT_MIN_INTERVALS,
              th_multipliers=DEFAULT_TH_MULTIPLIERS,
              acc_init=None, gyr_init_scale=1.0,
              optimize_gyr_bias=False, with_gyro=True,
              estimate_misalignment=True, log=None):
    """Full multi-position calibration of one recording.

    acc/gyr are (N,3) raw samples in whatever units the sensor delivers
    (SI here), t is (N,) seconds. `init_static_sec` is the initial resting
    period used to size the static threshold and to seed the gyro bias;
    None means "the first static interval the detector finds".

    The static threshold is not a tuning knob: imu_tk sweeps a multiplier
    of the initial period's variance and keeps whichever fit came out
    with the lowest residual, which is what this does too."""
    acc = np.asarray(acc, dtype=float)
    gyr = np.asarray(gyr, dtype=float)
    t = np.asarray(t, dtype=float)
    if len(acc) != len(gyr) or len(acc) != len(t):
        raise ValueError("acc, gyr and t must have the same length")

    def say(msg):
        if log is not None:
            log(msg)

    # The initial rest period sets the scale of "still". imu_tk takes the
    # variance of the whole declared block, which silently produces a
    # far too large threshold when the block is not actually quiet
    # throughout -- and the gyro bias, a plain mean over the same block,
    # is then simply wrong. Taking a low percentile of the ROLLING
    # variance instead costs nothing on a clean rest period (the rolling
    # estimate is flat there) and stays right when the operator moved a
    # second too early.
    if init_static_sec is not None and init_static_sec > 0:
        end = int(np.searchsorted(t, t[0] + init_static_sec))
        end = max(min(end, len(acc) - 1), 2)
    else:
        end = min(len(acc), max(win_size, 200)) - 1
    init_block = acc[:end + 1]
    rv = rolling_variance_norm(init_block, win_size)
    quiet = rv[np.isfinite(rv)]
    if quiet.size >= 10:
        norm_th = float(np.percentile(quiet, 10))
    else:
        var = init_block.var(axis=0, ddof=1)
        norm_th = float(np.sqrt((var * var).sum()))
    if not (norm_th > 0.0):
        raise ValueError("the initial period has zero variance: is the "
                         "stream really live?")
    # How much of the declared rest period was genuinely at rest. A
    # contaminated one is reported rather than quietly absorbed.
    init_static = (np.isfinite(rv) & (rv < 4.0 * norm_th)) if quiet.size else \
        np.ones(len(init_block), dtype=bool)
    init_static_frac = (float(init_static.sum()) / float(max(quiet.size, 1))
                        if quiet.size else 1.0)
    say("initial rest period: %d samples, variance norm %.3e, %.0f%% static"
        % (len(init_block), norm_th, 100.0 * init_static_frac))

    best = None
    for mult in th_multipliers:
        got = calibrate_accel(acc, g_mag, mult * norm_th, win_size,
                              n_samples, min_intervals, acc_init,
                              estimate_misalignment)
        if got is None:
            say("  threshold x%-2d : too few static positions" % mult)
            continue
        p, cost, kept, samples = got
        say("  threshold x%-2d : %2d positions, residual %.6e"
            % (mult, len(kept), cost))
        if best is None or cost < best[1]:
            best = (p, cost, kept, mult, samples)
    if best is None:
        raise ValueError("no threshold produced at least %d static "
                         "positions -- too few poses, or the unit never "
                         "really came to rest" % min_intervals)

    p_acc, acc_cost, kept, mult, acc_samples = best
    res = CalibResult()
    res.misalignment_estimated = estimate_misalignment
    res.acc_matrix = acc_matrix(p_acc)
    res.acc_bias = p_acc[6:9].copy()
    res.acc_cost = acc_cost
    res.th_multiplier = mult
    res.intervals = kept
    res.init_static_frac = init_static_frac
    res.init_static_mask = init_static
    res.n_positions = len(kept)
    say("accelerometer: %d positions at threshold x%d, residual %.6e"
        % (len(kept), mult, acc_cost))

    # What the calibration actually bought, on the very samples it was
    # fitted to. "bias" is the middle ground a plain zero-offset
    # measurement would give, so the three rows separate what the bias
    # fixed from what the scale and misalignment added.
    eye = np.eye(3)
    res.acc_stats = {
        "raw": acc_error_stats(acc_samples, eye, np.zeros(3), g_mag),
        "bias": acc_error_stats(acc_samples, eye, res.acc_bias, g_mag),
        "full": acc_error_stats(acc_samples, res.acc_matrix, res.acc_bias,
                                g_mag),
    }

    # Per-pose values for the before/after plot. Mean first, then norm:
    # one number per pose, with the per-sample noise averaged out.
    pose_acc = np.array([acc[s:e + 1].mean(axis=0) for (s, e) in kept])
    pose_gyr = np.array([gyr[s:e + 1].mean(axis=0) for (s, e) in kept])
    res.pose_acc_raw = np.linalg.norm(pose_acc, axis=1)
    res.pose_acc_cal = np.linalg.norm(
        apply_calib(pose_acc, res.acc_matrix, res.acc_bias), axis=1)
    res.pose_gyr_raw = pose_gyr
    res.pose_gyr_cal = pose_gyr.copy()   # replaced once the gyro is solved

    acc_cal = apply_calib(acc, res.acc_matrix, res.acc_bias)
    means, kept_means = interval_samples(acc_cal, kept, n_samples, means=True)
    res.residual_rms = float(np.sqrt(np.mean(
        (np.linalg.norm(means, axis=1) - g_mag) ** 2)))

    if not with_gyro or len(kept_means) < 2:
        return res

    # Gyro bias from the initial rest period, as imu_tk seeds it, but with
    # two changes, because this period is recorded on a desk next to the
    # operator rather than on a lab bench:
    #
    #   - only the samples that were actually static count. A second of
    #     movement at the end drags the estimate by more than the bias
    #     being measured, and nothing downstream could tell.
    #   - the MEDIAN, not the mean, over what is left. Note what this is
    #     and is not for: a gyro measures a RATE, so a disturbance that
    #     starts and ends at rest (typing on the same desk, a fan, a
    #     passing lorry) integrates to zero and a mean shrugs it off by
    #     itself. What a mean cannot survive is a knock that leaves the
    #     unit slightly TURNED, because that is a real net rotation
    #     spread over the window. Measured here on a 20 s window at
    #     100 Hz, worst-axis bias error against a 0.9 deg/s truth: with
    #     5 such knocks a plain mean is 0.016 deg/s and the masked mean
    #     0.0009; with 40 the masked mean degrades to 0.020 while the
    #     median holds 0.0023. On a clean window the median costs
    #     0.0010 -> 0.0013 deg/s, which against a MEMS bias is nothing.
    gyr_init = gyr[:len(init_block)]
    clean = gyr_init[init_static] if init_static.any() else gyr_init
    gyr_bias0 = np.median(clean, axis=0)
    init_block_mean = gyr_bias0
    spans = [(kept_means[i][1], kept_means[i + 1][0])
             for i in range(len(kept_means) - 1)]
    batch = _RotationBatch(gyr, t, spans)
    v = means / np.linalg.norm(means, axis=1, keepdims=True)
    v0, v1 = v[:-1], v[1:]

    def gyr_residual(p):
        bias = gyr_bias0 + p[9:12] if optimize_gyr_bias else gyr_bias0
        rot = batch.rotations(gyr_matrix(p), bias)
        # The gyro has to carry pose i's gravity direction onto pose i+1.
        pred = np.einsum("kji,kj->ki", rot, v0)      # rot.T @ v0
        return (pred - v1).ravel()

    x0 = np.array([0.0] * 6 + [gyr_init_scale] * 3 + [0.0] * 3)
    free = list(range(9)) if estimate_misalignment else list(range(6, 9))
    if optimize_gyr_bias:
        free += [9, 10, 11]
    p_gyr, gyr_cost, _ = lm_solve_free(gyr_residual, x0, free)
    res.gyr_matrix = gyr_matrix(p_gyr)
    res.gyr_bias = (gyr_bias0 + p_gyr[9:12] if optimize_gyr_bias
                    else np.asarray(init_block_mean, dtype=float))
    res.gyr_cost = gyr_cost
    res.gyr_solved = True
    r = gyr_residual(p_gyr).reshape(-1, 3)
    res.gyr_residual_rms = float(np.sqrt(np.mean((r ** 2).sum(axis=1))))
    res.pose_gyr_cal = apply_calib(pose_gyr, res.gyr_matrix, res.gyr_bias)
    res.gyr_stats = {
        "raw": gyr_error_stats(batch, v0, v1, np.eye(3), np.zeros(3)),
        "bias": gyr_error_stats(batch, v0, v1, np.eye(3), res.gyr_bias),
        "full": gyr_error_stats(batch, v0, v1, res.gyr_matrix, res.gyr_bias),
    }
    say("gyroscope: %d rotations, residual %.6e (rms direction error "
        "%.4f = %.2f deg)"
        % (len(spans), gyr_cost, res.gyr_residual_rms,
           math.degrees(min(res.gyr_residual_rms, 2.0))))
    return res
