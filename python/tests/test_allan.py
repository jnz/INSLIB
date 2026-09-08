"""Self-test for allan_variance.py: synthesize a long static IMU recording
with a KNOWN white noise (ARW) and bias random walk (RRW), then check the
Allan-variance tool recovers the injected bias_rw (the headline output) and,
more loosely, the white-noise psd. Runs under pytest or standalone

(c) Jan Zwiener (jan@zwiener.org)
(``python3 python/tests/test_allan.py``)."""

import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import allan_variance as av  # noqa: E402

_G = 9.80665
_FS = 100.0
_DUR_S = 3600.0           # 1 h -- long enough for the +1/2 branch to develop
_N_ARW = 4.0e-4           # white-noise (ARW) coeff [rad/s/sqrt(s) resp. .../sqrt(s)]
# The bias random walk is deliberately kept two decades below the white
# noise, like a real MEMS IMU: the two branches then cross at
# tau* = sqrt(3)*N/K ~ 170 s, FAR from the tau=3 s read-out point. Reading
# the curve at tau=3 s (rather than fitting the +1/2 asymptote and
# extrapolating) returns the white noise there and over-states K by ~30x,
# so this separation is what makes the test able to catch that mistake --
# with N == K the crossing sits at tau=1.7 s and the two agree by luck.
_K_RRW = 4.0e-6           # bias random-walk (RRW) coeff [rad/s/sqrt(s)]


def _synthesize(path):
    """Write a static imu.csv with white noise + a random-walk bias on every
    axis (accel z carries -g), using the injected ARW/RRW coefficients."""
    rng = np.random.default_rng(7)
    tau0 = 1.0 / _FS
    n = int(_DUR_S * _FS)
    t_us = (np.arange(n) / _FS * 1e6).astype(np.int64)
    sigma_w = _N_ARW / math.sqrt(tau0)        # per-sample white stddev
    sigma_step = _K_RRW * math.sqrt(tau0)     # per-step random-walk increment

    def channel():
        white = rng.normal(0.0, sigma_w, n)
        bias = np.cumsum(rng.normal(0.0, sigma_step, n))
        return white + bias

    gyr = np.column_stack([channel() for _ in range(3)])
    acc = np.column_stack([channel() for _ in range(3)])
    acc[:, 2] -= _G
    with open(path, "w", encoding="utf-8") as f:
        f.write("# t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]\n")
        for i in range(n):
            f.write(f"{t_us[i]},{gyr[i,0]:.6e},{gyr[i,1]:.6e},{gyr[i,2]:.6e},"
                    f"{acc[i,0]:.6e},{acc[i,1]:.6e},{acc[i,2]:.6e}\n")


def test_allan_recovers_bias_rw():
    av._QUIET = True          # keep the progress output out of the test log
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "imu.csv")
        _synthesize(path)
        r = av.analyze(path)

    # Headline: the suggested bias_rw (worst axis) recovers the injected RRW.
    # The bound is deliberately tight enough that reading the curve at
    # tau=3 s instead of fitting the +1/2 asymptote (~30x high here) fails.
    assert 0.5 * _K_RRW < r["gyr_bias_rw"] < 2.0 * _K_RRW, \
        f"gyr_bias_rw {r['gyr_bias_rw']:.2e} vs injected {_K_RRW:.2e}"
    assert 0.5 * _K_RRW < r["acc_bias_rw"] < 2.0 * _K_RRW, \
        f"acc_bias_rw {r['acc_bias_rw']:.2e} vs injected {_K_RRW:.2e}"

    # Bonus: the ARW-derived psd recovers the injected white noise (looser --
    # at tau=1 s the RRW slope leaks in a bit).
    psd_true = _N_ARW ** 2
    assert 0.4 * psd_true < r["gyr_psd"] < 3.0 * psd_true, \
        f"gyr_psd {r['gyr_psd']:.2e} vs injected {psd_true:.2e}"

    # A clean synthetic static record must not trip the motion warnings.
    assert not any("not be static" in w for w in r["warnings"]), r["warnings"]


if __name__ == "__main__":
    test_allan_recovers_bias_rw()
    print("ok  allan_variance recovers injected bias_rw / psd")
