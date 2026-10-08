"""Self-test for replay.py's per-sample IMU noise check (noise_model_metrics):
it must recognise an IMU that low-pass filters its output and then stay
silent, and it may only suggest a psd for a gross error in the dangerous
direction. Runs under pytest or standalone
(``python3 python/tests/test_noise_model.py``).

(c) Jan Zwiener (jan@zwiener.org)
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "tools"))

import replay  # noqa: E402

_HZ = 100.0
_N = 20000


def _stats(x):
    """d2/d3 Stat triples of an (n, 3) series, the way _imu_prepass builds them."""
    d2, d3 = [replay.Stat() for _ in range(3)], [replay.Stat() for _ in range(3)]
    for k in range(3):
        for v in np.diff(x[:, k], 2):
            d2[k].add(float(v))
        for v in np.diff(x[:, k], 3):
            d3[k].add(float(v))
    return d2, d3


def _metrics(gyr, acc, gyr_psd, acc_psd):
    g2, g3 = _stats(gyr)
    a2, a3 = _stats(acc)
    return replay.noise_model_metrics(g2, a2, g3, a3, _HZ,
                                      {"gyr_psd": gyr_psd, "acc_psd": acc_psd})


def _white(sigma, seed):
    return np.random.default_rng(seed).normal(0.0, sigma, (_N, 3))


def _lowpass(x, alpha=0.3):
    """First-order low-pass twice, far below Nyquist like an IMU-internal LPF."""
    y = np.copy(x)
    for _ in range(2):
        for i in range(1, len(y)):
            y[i] = y[i - 1] + alpha * (y[i] - y[i - 1])
    return y


def test_white_noise_matches_and_is_not_bandlimited():
    sigma = 1e-3
    psd = sigma ** 2 / _HZ
    m = _metrics(_white(sigma, 1), _white(sigma, 2), psd, psd)
    for k in ("gyr", "acc"):
        assert not m[f"{k}_bandlimited"], m
        assert abs(m[f"{k}_d3d2"] - 20.0 / 6.0) < 0.2, m
        assert 0.9 < m[f"{k}_ratio"] < 1.1, m
        assert m[f"{k}_psd_sugg"] is None, m


def test_lowpass_is_detected_and_gets_no_suggestion():
    sigma = 1e-3
    # Configured model 1000x too small: without the low-pass test the
    # suggestion gate would fire on the remaining noise.
    psd = (sigma / 1000.0) ** 2 / _HZ
    m = _metrics(_lowpass(_white(sigma, 3)), _lowpass(_white(sigma, 4)), psd, psd)
    for k in ("gyr", "acc"):
        assert m[f"{k}_bandlimited"], m
        assert m[f"{k}_psd_sugg"] is None, m


def test_suggestion_only_for_gross_optimism():
    sigma = 1e-3
    fine = sigma ** 2 / _HZ
    # 50x noisier than configured (stddev): within the tolerance, quiet.
    m = _metrics(_white(sigma, 5), _white(sigma, 6), fine / 50.0 ** 2, fine * 1e4)
    assert m["gyr_psd_sugg"] is None, m
    # Configured 100x above the measurement: conservative, never talked down.
    assert m["acc_psd_sugg"] is None, m
    # 300x noisier than configured: a suggestion near the true psd.
    m = _metrics(_white(sigma, 7), _white(sigma, 8), fine / 300.0 ** 2, fine)
    assert m["gyr_psd_sugg"] is not None and 0.8 * fine < m["gyr_psd_sugg"] < 1.2 * fine, m
    assert m["acc_psd_sugg"] is None, m


_TESTS = [
    test_white_noise_matches_and_is_not_bandlimited,
    test_lowpass_is_detected_and_gets_no_suggestion,
    test_suggestion_only_for_gross_optimism,
]


if __name__ == "__main__":
    fails = 0
    for t in _TESTS:
        try:
            t()
            print(f"ok    {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{fails} failures")
    sys.exit(1 if fails else 0)
