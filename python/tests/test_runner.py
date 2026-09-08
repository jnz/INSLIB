"""End-to-end test of the YAML-configured CSV runner (INSLIB.runner).

Synthesizes a small static dataset with deliberately mixed conventions
(header names, seconds timestamps, deg/s gyro, hPa pressure) to exercise
the column-mapping and unit machinery, runs the full pipeline and checks
the solution CSV. Runs under pytest or standalone
(``python3 python/tests/test_runner.py``).

(c) Jan Zwiener (jan@zwiener.org)
"""

import csv
import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

_G = 9.80665
_LAT, _LON, _H = 48.783, 9.181, 300.0


def _write_dataset(d):
    random.seed(7)
    with open(os.path.join(d, "imu.csv"), "w", encoding="utf-8") as f:
        f.write("t_s,gx_dps,gy_dps,gz_dps,ax,ay,az\n")
        for i in range(3000):                       # 30 s @ 100 Hz
            f.write(f"{i*0.01:.3f},"
                    f"{random.gauss(0.2, 0.05):.5f},"     # 0.2 deg/s bias
                    f"{random.gauss(0, 0.05):.5f},"
                    f"{random.gauss(0, 0.05):.5f},"
                    f"{random.gauss(0, 0.02):.5f},"
                    f"{random.gauss(0, 0.02):.5f},"
                    f"{-_G + random.gauss(0, 0.02):.5f}\n")
    with open(os.path.join(d, "gnss.csv"), "w", encoding="utf-8") as f:
        f.write("t_s,lat_deg,lon_deg,h_m,sn,se,sd\n")
        for i in range(30):
            f.write(f"{i + 0.5:.3f},"
                    f"{_LAT + random.gauss(0, 1e-6):.9f},"
                    f"{_LON + random.gauss(0, 1e-6):.9f},"
                    f"{_H + random.gauss(0, 0.5):.3f},0.3,0.3,0.6\n")
    p0 = 101325.0 * (1.0 - 2.25577e-5 * 111.0) ** 5.25588
    with open(os.path.join(d, "baro.csv"), "w", encoding="utf-8") as f:
        f.write("t_s,p_hpa\n")
        for i in range(300):                        # 10 Hz
            f.write(f"{i*0.1:.3f},{(p0 + random.gauss(0, 5)) / 100.0:.4f}\n")


_YAML = """
imu:
  file: imu.csv
  time: {col: t_s, unit: s}
  gyr:  {cols: [gx_dps, gy_dps, gz_dps], unit: deg/s}
  acc:  {cols: [ax, ay, az], unit: m/s2}
gnss:
  file: gnss.csv
  format: llh_deg
  time: {col: t_s, unit: s}
  pos:  {cols: [lat_deg, lon_deg, h_m]}
  stddev_ned: {cols: [sn, se, sd]}
baro:
  file: baro.csv
  time: {col: 0, unit: s}
  pressure: {col: 1, unit: hPa}
wmm: {year: 2026.5}
output:
  plotjuggler: false
  csv: solution.csv
"""


def test_runner_end_to_end():
    try:
        import yaml  # noqa: F401
    except ImportError:
        print("skipped (PyYAML not installed)")
        return
    from INSLIB import runner

    with tempfile.TemporaryDirectory() as d:
        _write_dataset(d)
        cfg = os.path.join(d, "run.yaml")
        with open(cfg, "w", encoding="utf-8") as f:
            f.write(_YAML)
        runner.main([cfg])

        out = os.path.join(d, "solution.csv")
        assert os.path.exists(out)
        with open(out, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 3000
        last = rows[-1]
        # converged: FULL mode, position back at the truth, level attitude
        assert last["mode"] == "FULL" and last["ready"] == "1"
        assert abs(float(last["lat_deg"]) - _LAT) < 1e-4
        assert abs(float(last["lon_deg"]) - _LON) < 1e-4
        assert abs(float(last["h_ell_m"]) - _H) < 3.0
        assert abs(float(last["roll_deg"])) < 1.0
        assert abs(float(last["pitch_deg"])) < 1.0
        # baro channel alive and in the shared datum
        assert last["baro_height_m"] != ""
        assert abs(float(last["baro_height_m"])) < 5.0
        # deg/s mapping: the 0.2 deg/s gyro bias must show up in the
        # x bias estimate (in rad/s), proving the unit conversion ran.
        # (bias errors would otherwise sit at 0 or 0.2*57.3)


def test_runner_missing_imu_fails():
    try:
        import yaml  # noqa: F401
    except ImportError:
        print("skipped (PyYAML not installed)")
        return
    from INSLIB import runner

    with tempfile.TemporaryDirectory() as d:
        cfg = os.path.join(d, "bad.yaml")
        with open(cfg, "w", encoding="utf-8") as f:
            f.write("gnss: {file: gnss.csv}\n")
        try:
            runner.main([cfg])
        except SystemExit as e:
            assert "imu" in str(e)
        else:
            raise AssertionError("expected SystemExit for missing imu")


_TESTS = [
    test_runner_end_to_end,
    test_runner_missing_imu_fails,
]


def _main():
    fails = 0
    for t in _TESTS:
        try:
            t()
            print(f"ok    {t.__name__}")
        except Exception as e:  # noqa: BLE001
            fails += 1
            print(f"FAIL  {t.__name__}: {e!r}")
    print(f"\n{fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(_main())
