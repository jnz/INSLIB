#!/usr/bin/env python3
"""Regression tests for tools/inslib_imu_tk.py (multi-position IMU calib).

Two independent checks, because they fail for different reasons:

  * Synthetic data with a KNOWN calibration. Catches an arithmetic or
    bookkeeping error in the solver, since the answer is available.
  * The Xsens recording shipped with imu_tk (tools/testdata/). Catches
    what synthetic data cannot: a misreading of the sensor model would
    make the generator and the solver agree with each other and both be
    wrong. This is real hardware, from the authors of the method.

Runs under pytest or standalone:

    python3 python/tests/test_imu_tk.py
"""

import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inslib_imu_tk as tk   # noqa: E402

TESTDATA = os.path.join(REPO, "tools", "testdata")

# imu_tk's own example application settings for this recording: raw ADC
# counts around 32768, and the local gravity where it was taken.
XSENS_G = 9.81744
XSENS_INIT_SEC = 50.0
XSENS_ACC_INIT = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
                           32768.0, 32768.0, 32768.0])


# --------------------------------------------------------------------------
# Synthetic: a known calibration has to come back out
# --------------------------------------------------------------------------

M_ACC = np.array([[1.0, -0.008, 0.005],
                  [0.0, 1.0, -0.011],
                  [0.0, 0.0, 1.0]]) @ np.diag([1.012, 0.994, 1.007])
B_ACC = np.array([0.075, -0.052, 0.110])
M_GYR = np.array([[1.0, -0.006, 0.009],
                  [0.004, 1.0, -0.007],
                  [-0.010, 0.003, 1.0]]) @ np.diag([1.021, 0.988, 1.005])
B_GYR = np.radians([0.9, -0.4, 0.25])
G = 9.80665
RATE = 100.0


def _rot(axis, ang):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return (np.eye(3) * math.cos(ang) + math.sin(ang) * k
            + (1 - math.cos(ang)) * np.outer(axis, axis))


def _make_session(rng, n_pos=22, still_sec=4.0, turn_sec=1.6, init_sec=20.0):
    """Static poses at arbitrary attitudes, smooth rotations between them.

    The rotations start and end at rest (a bell-shaped rate), which is
    what a hand actually does and what the static detector needs."""
    acc, gyr, ts = [], [], []
    rot = np.eye(3)
    t = [0.0]

    def hold(sec, rot):
        for _ in range(int(sec * RATE)):
            acc.append(-(rot @ np.array([0.0, 0.0, -G])))
            gyr.append(np.zeros(3))
            ts.append(t[0])
            t[0] += 1.0 / RATE

    def turn(sec, rot):
        n = int(sec * RATE)
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        total = rng.uniform(0.5, 2.6)
        for i in range(n):
            w = total * math.pi / sec * math.sin(math.pi * i / n) / 2.0
            acc.append(-(rot @ np.array([0.0, 0.0, -G])))
            gyr.append(axis * w)
            ts.append(t[0])
            t[0] += 1.0 / RATE
            rot = _rot(axis, -w / RATE) @ rot
        return rot

    hold(init_sec, rot)
    for _ in range(n_pos - 1):
        rot = turn(turn_sec, rot)
        hold(still_sec, rot)
    return np.array(acc), np.array(gyr), np.array(ts)


def test_synthetic_recovers_known_calibration():
    rng = np.random.default_rng(4)
    acc_t, gyr_t, t = _make_session(rng)
    acc = acc_t @ np.linalg.inv(M_ACC).T + B_ACC + rng.normal(0, 0.010, acc_t.shape)
    gyr = gyr_t @ np.linalg.inv(M_GYR).T + B_GYR \
        + rng.normal(0, math.radians(0.03), gyr_t.shape)

    res = tk.calibrate(acc, gyr, t, g_mag=G, init_static_sec=19.0)

    assert res.n_positions == 22, res.n_positions
    assert np.abs(res.acc_matrix - M_ACC).max() < 1e-3
    assert np.abs(res.acc_bias - B_ACC).max() < 5e-3
    assert np.abs(res.gyr_matrix - M_GYR).max() < 1e-3
    assert np.abs(np.degrees(res.gyr_bias - B_GYR)).max() < 0.01
    assert res.residual_rms < 0.005
    # The fit must improve on doing nothing, in the obvious direction.
    assert res.acc_stats["full"]["rms"] < 0.2 * res.acc_stats["raw"]["rms"]


def test_no_misalignment_is_diagonal_and_worse_here():
    """With misalignment off the model cannot represent this truth, so it
    must be strictly worse -- and the matrices must come out diagonal."""
    rng = np.random.default_rng(4)
    acc_t, gyr_t, t = _make_session(rng)
    acc = acc_t @ np.linalg.inv(M_ACC).T + B_ACC + rng.normal(0, 0.010, acc_t.shape)
    gyr = gyr_t @ np.linalg.inv(M_GYR).T + B_GYR \
        + rng.normal(0, math.radians(0.03), gyr_t.shape)

    full = tk.calibrate(acc, gyr, t, g_mag=G, init_static_sec=19.0)
    diag = tk.calibrate(acc, gyr, t, g_mag=G, init_static_sec=19.0,
                        estimate_misalignment=False)

    off = ~np.eye(3, dtype=bool)
    assert np.abs(diag.acc_matrix[off]).max() == 0.0
    assert np.abs(diag.gyr_matrix[off]).max() == 0.0
    assert diag.acc_stats["full"]["rms"] > full.acc_stats["full"]["rms"]
    assert diag.gyr_residual_rms > full.gyr_residual_rms


# --------------------------------------------------------------------------
# Real hardware: imu_tk's own Xsens recording
# --------------------------------------------------------------------------

def _load_xsens():
    acc = np.loadtxt(os.path.join(TESTDATA, "imu_tk_xsens_acc.mat.gz"))
    gyr = np.loadtxt(os.path.join(TESTDATA, "imu_tk_xsens_gyro.mat.gz"))
    return acc[:, 0], acc[:, 1:4], gyr[:, 1:4]


def test_xsens_reference_recording():
    t, acc, gyr = _load_xsens()
    assert acc.shape == (51175, 3)

    res = tk.calibrate(acc, gyr, t, g_mag=XSENS_G,
                       init_static_sec=XSENS_INIT_SEC,
                       acc_init=XSENS_ACC_INIT, gyr_init_scale=1.0 / 6258.0)

    # Bounds, not exact values: the numbers below are what this port
    # produces today, with room for a different-but-equivalent optimum.
    assert res.n_positions >= 35, res.n_positions
    assert res.residual_rms < 0.005, res.residual_rms          # ~0.0011
    assert math.degrees(res.gyr_residual_rms) < 1.0            # ~0.53 deg

    # Inverse scale factors: counts per m/s^2 and counts per rad/s. A
    # regression that flipped a convention or lost a factor shows up here
    # long before the residual does.
    inv_acc = [1.0 / np.linalg.norm(res.acc_matrix[:, j]) for j in range(3)]
    inv_gyr = [1.0 / np.linalg.norm(res.gyr_matrix[:, j]) for j in range(3)]
    for v in inv_acc:
        assert 405.0 < v < 425.0, inv_acc
    for v in inv_gyr:
        assert 4600.0 < v < 4950.0, inv_gyr
    # Near the nominal 32768 zero of a 16-bit sensor, but not exactly.
    assert np.abs(res.acc_bias - 32768.0).max() < 1500.0


def test_xsens_gyro_scale_beats_the_physical_bound():
    """A gyro cannot integrate to LESS rotation than gravity turned.

    The model-free check on the gyro scale: for each rotation, the angle
    of the integrated rotation must be at least the angle between the two
    gravity directions the (calibrated) accelerometer measured. Equality
    is reached by a pure tilt, so the minimum over many rotations must sit
    just above 1. imu_tk's example seed of 1/6258 gives 0.76 here, which
    is impossible -- that is how the fitted scale was confirmed."""
    t, acc, gyr = _load_xsens()
    res = tk.calibrate(acc, gyr, t, g_mag=XSENS_G,
                       init_static_sec=XSENS_INIT_SEC,
                       acc_init=XSENS_ACC_INIT, with_gyro=False)

    acc_cal = tk.apply_calib(acc, res.acc_matrix, res.acc_bias)
    means, kept = tk.interval_samples(acc_cal, res.intervals, means=True)
    v = means / np.linalg.norm(means, axis=1, keepdims=True)
    spans = [(kept[i][1], kept[i + 1][0]) for i in range(len(kept) - 1)]
    batch = tk._RotationBatch(gyr, t, spans)
    bias0 = np.median(gyr[:5002], axis=0)

    turned = np.degrees(np.arccos(np.clip((v[:-1] * v[1:]).sum(axis=1), -1, 1)))
    big = turned > 5.0          # tiny rotations make the ratio meaningless

    def min_ratio(counts_per_rad):
        rot = batch.rotations(np.eye(3) / counts_per_rad, bias0)
        tr = np.clip((np.trace(rot, axis1=1, axis2=2) - 1.0) / 2.0, -1, 1)
        return (np.degrees(np.arccos(tr))[big] / turned[big]).min()

    assert min_ratio(6258.0) < 0.85          # the seed value is impossible
    assert min_ratio(4770.0) > 0.99          # the fitted one sits on the bound


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print("ok    %s" % name)
        except AssertionError as e:
            failures += 1
            print("FAIL  %s: %s" % (name, e))
    print("==== %d failures ====" % failures)
    sys.exit(1 if failures else 0)
