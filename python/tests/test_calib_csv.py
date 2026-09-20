#!/usr/bin/env python3
"""End-to-end test of the offline calibration path (--csv) of
tools/inslib_imu_calib.py.

A whole calibration session is synthesised the way a person would record
it: an initial rest period, then static poses at arbitrary attitudes with
hand-like rotations in between. A known scale factor, misalignment and
bias is put into the gyroscope and the accelerometer, and a known hard
iron, soft iron and mounting rotation into the magnetometer. The raw
values are written as replay-format imu.csv/mag.csv, and everything
downstream is the real tool: CSV loader, static pose detection, IMU
solve, magnetometer solve and the config.yaml writer. The errors have to
come back out.

test_imu_tk.py and test_mag_calib.py hold the two solvers against known
truths on their own. What only this file covers is the plumbing between
them: units and column order of the CSV, the time based pairing of the
slower magnetometer with the IMU poses, and the column-major layout the
command line tool writes.

Runs under pytest or standalone:

    python3 python/tests/test_calib_csv.py
"""

import math
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOLS = os.path.join(REPO, "tools")
sys.path.insert(0, TOOLS)
import inslib_frame_align as fa   # noqa: E402
import inslib_imu_calib as calib   # noqa: E402

G = 9.80665
RATE = 100.0            # IMU [Hz]
MAG_DECIM = 2           # magnetometer at half the IMU rate
T0_US = 7_250_000       # the board clock does not start at zero
N_POSES = 30
INIT_SEC = 20.0

FIELD_UT = 48.5
DIP_DEG = 64.0
DECL_DEG = 3.5

# Truth, in the model the tool writes: corrected = M * (raw - bias).
# The accelerometer triad defines the body frame, so its misalignment is
# upper triangular (inslib_imu_tk.acc_matrix). The gyroscope gets all six
# terms.
M_ACC = np.array([[1.0, -0.008, 0.005],
                  [0.0, 1.0, -0.011],
                  [0.0, 0.0, 1.0]]) @ np.diag([1.012, 0.994, 1.007])
B_ACC = np.array([0.075, -0.052, 0.110])
M_GYR = np.array([[1.0, -0.006, 0.009],
                  [0.004, 1.0, -0.007],
                  [-0.010, 0.003, 1.0]]) @ np.diag([1.021, 0.988, 1.005])
B_GYR = np.radians([0.9, -0.4, 0.25])
# Symmetric soft iron (the only part an ellipsoid fit can see) turned by
# the mounting rotation of the magnetometer onto the IMU.
A_SOFT = np.array([[1.06, 0.03, -0.02],
                   [0.03, 0.95, 0.015],
                   [-0.02, 0.015, 1.01]])
R_MAG = fa.rodrigues(np.radians([1.5, -2.5, 3.0]))
M_MAG = R_MAG @ A_SOFT
B_MAG = np.array([7.5, -4.2, 11.0])

ACC_NOISE = 0.010                   # [m/s^2]
GYR_NOISE = math.radians(0.03)      # [rad/s]
MAG_NOISE = 0.05                    # [uT]


def _field_ned():
    dip, dec = math.radians(DIP_DEG), math.radians(DECL_DEG)
    return FIELD_UT * np.array([math.cos(dip) * math.cos(dec),
                                math.cos(dip) * math.sin(dec),
                                math.sin(dip)])


def _make_session(seed=11):
    """(t_us, gyr_raw, acc_raw, temp, mag_t_us, mag_raw) of one session.

    c_bn is the body-from-NED rotation. At rest the accelerometer reads
    c_bn*(0, 0, -g), the INSLIB convention, and the magnetometer c_bn*F.
    A turn runs about one fixed axis with a bell-shaped rate, which is
    what a hand does and what the static detector needs at both ends."""
    rng = np.random.default_rng(seed)
    f_ned = _field_ned()
    g_ned = np.array([0.0, 0.0, -G])
    gyr_t, acc_t, mag_t = [], [], []
    c_bn = np.eye(3)

    def sample(c_bn, w_body):
        gyr_t.append(w_body)
        acc_t.append(c_bn @ g_ned)
        mag_t.append(c_bn @ f_ned)

    def hold(sec, c_bn):
        for _ in range(int(sec * RATE)):
            sample(c_bn, np.zeros(3))

    def turn(sec, c_bn):
        n = int(sec * RATE)
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        total = rng.uniform(0.6, 2.8)          # [rad]
        for i in range(n):
            w = total * math.pi / sec * math.sin(math.pi * i / n) / 2.0
            sample(c_bn, axis * w)
            c_bn = fa.rodrigues(-axis * w / RATE) @ c_bn
        return c_bn

    hold(INIT_SEC, c_bn)
    for _ in range(N_POSES - 1):
        c_bn = turn(1.6, c_bn)
        hold(4.0, c_bn)

    n = len(acc_t)
    t_us = T0_US + np.arange(n, dtype=np.int64) * int(1e6 / RATE)
    gyr = (np.array(gyr_t) @ np.linalg.inv(M_GYR).T + B_GYR
           + rng.normal(0.0, GYR_NOISE, (n, 3)))
    acc = (np.array(acc_t) @ np.linalg.inv(M_ACC).T + B_ACC
           + rng.normal(0.0, ACC_NOISE, (n, 3)))
    temp = np.linspace(24.0, 25.5, n)
    mag = (np.array(mag_t) @ np.linalg.inv(M_MAG).T + B_MAG
           + rng.normal(0.0, MAG_NOISE, (n, 3)))
    return t_us, gyr, acc, temp, t_us[::MAG_DECIM], mag[::MAG_DECIM]


def _write_session(outdir):
    t_us, gyr, acc, temp, mt_us, mag = _make_session()
    with open(os.path.join(outdir, "imu.csv"), "w", encoding="utf-8",
              newline="\n") as f:
        f.write("# t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2],"
                " imu_temp_c [degC]\n")
        for i in range(len(t_us)):
            f.write("%d,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.2f\n" % (
                t_us[i], *gyr[i], *acc[i], temp[i]))
    with open(os.path.join(outdir, "mag.csv"), "w", encoding="utf-8",
              newline="\n") as f:
        f.write("# t_us, mag_frd_xyz [uT]\n")
        for i in range(len(mt_us)):
            f.write("%d,%.4f,%.4f,%.4f\n" % (mt_us[i], *mag[i]))


_SOLVED = {}


def _solved():
    """Session written to CSV, loaded back and solved, once per run."""
    if not _SOLVED:
        with tempfile.TemporaryDirectory() as d:
            _write_session(d)
            imu_path, mag_path = calib.resolve_csv_paths(d)
            rec = calib.load_csv_recording(imu_path, mag_path)
        cal, magcal = calib.solve_session(rec, gravity=G,
                                          init_sec=INIT_SEC - 1.0,
                                          field_ut=FIELD_UT)
        _SOLVED.update(rec=rec, cal=cal, magcal=magcal)
    return _SOLVED["rec"], _SOLVED["cal"], _SOLVED["magcal"]


def _rotation_angle_deg(r):
    c = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(c))


# --------------------------------------------------------------------------
# Loader
# --------------------------------------------------------------------------

def test_directory_picks_up_both_files():
    rec, _cal, _mag = _solved()
    n_imu = int(INIT_SEC * RATE) + (N_POSES - 1) * int(5.6 * RATE)
    assert len(rec) == n_imu, len(rec)
    assert len(rec.mag) == (n_imu + 1) // MAG_DECIM, len(rec.mag)
    assert abs(rec.rate_hz() - RATE) < 1e-6
    assert abs(rec.temp_min - 24.0) < 0.01 and abs(rec.temp_max - 25.5) < 0.01
    # Both streams on the one clock, relative to the first IMU sample.
    t, _acc, _gyr = rec.arrays()
    tm, _m = rec.mag_arrays()
    assert t[0] == 0.0 and tm[0] == 0.0


def test_loader_skips_what_is_not_a_sample():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "imu.csv")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("# header\n"
                    "\n"
                    "1000,0,0,0,0,0,-9.8\n"
                    "2000,0,0,0,0,0\n"              # short line
                    "3000,0,nan,0,0,0,-9.8\n"       # not finite
                    "4000,0,0,0,x,0,-9.8\n"         # not a number
                    "5000,0.1,0.2,0.3,1,2,-9.8\n")
        rec = calib.load_csv_recording(path)
    assert rec.t_us == [1000, 5000]
    assert rec.gyr[1] == (0.1, 0.2, 0.3) and rec.acc[1] == (1.0, 2.0, -9.8)
    assert not rec.has_mag()
    assert calib.temp_range_text(rec.temp_min, rec.temp_max) == "not recorded"


def test_loader_rejects_time_running_backwards():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "imu.csv")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("1000,0,0,0,0,0,-9.8\n2000,0,0,0,0,0,-9.8\n"
                    "2000,0,0,0,0,0,-9.8\n")
        try:
            calib.load_csv_recording(path)
        except ValueError as e:
            assert "t_us=2000" in str(e)
        else:
            raise AssertionError("duplicate timestamp accepted")


# --------------------------------------------------------------------------
# The errors put into the data come back out
# --------------------------------------------------------------------------

def test_every_static_pose_is_found():
    rec, cal, _mag = _solved()
    assert cal.n_positions == N_POSES, cal.n_positions
    # The live counter (console and GUI) sees the same session.
    assert rec.pose_count(INIT_SEC - 1.0, 4.0) == N_POSES


def test_accelerometer_scale_misalignment_bias():
    _rec, cal, _mag = _solved()
    assert np.abs(np.asarray(cal.acc_matrix) - M_ACC).max() < 1e-3
    assert np.abs(np.asarray(cal.acc_bias) - B_ACC).max() < 5e-3
    assert cal.residual_rms < 0.005, cal.residual_rms
    assert cal.acc_stats["full"]["rms"] < 0.2 * cal.acc_stats["raw"]["rms"]


def test_gyroscope_scale_misalignment_bias():
    _rec, cal, _mag = _solved()
    assert np.abs(np.asarray(cal.gyr_matrix) - M_GYR).max() < 1e-3
    assert np.abs(np.degrees(np.asarray(cal.gyr_bias) - B_GYR)).max() < 0.01
    # The noise model comes from the rest period, sigma^2 * dt.
    assert abs(cal.gyr_psd / (GYR_NOISE ** 2 / RATE) - 1.0) < 0.2
    assert abs(cal.acc_psd / (ACC_NOISE ** 2 / RATE) - 1.0) < 0.2
    assert not cal.warnings, cal.warnings


def test_magnetometer_hard_soft_iron_and_alignment():
    _rec, _cal, magcal = _solved()
    assert magcal is not None
    assert np.abs(np.asarray(magcal.bias) - B_MAG).max() < 0.02
    assert np.abs(magcal.as_matrix() - M_MAG).max() < 1e-3
    # The mounting rotation against the IMU, which the ellipsoid alone
    # cannot see: the leftover rotation has to be a fraction of a degree
    # of the 4.2 deg that were put in.
    assert magcal.align is not None and magcal.align.n_poses == N_POSES
    assert _rotation_angle_deg(magcal.align.as_matrix().T @ R_MAG) < 0.1
    assert magcal.residual_rms_ut < 0.1, magcal.residual_rms_ut
    assert not magcal.warnings, magcal.warnings


# --------------------------------------------------------------------------
# Command line: CSV directory in, config.yaml out
# --------------------------------------------------------------------------

def _read_config(path):
    """{section: {key: value}} of the flat subset write_config emits, so
    the check does not need PyYAML."""
    out, sec = {}, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].rstrip()
            if not line:
                continue
            key, _sep, val = line.strip().partition(":")
            if not line.startswith(" "):
                sec = out.setdefault(key, {})
                continue
            val = val.strip()
            if val.startswith("["):
                val = [float(v) for v in val.strip("[]").split(",")]
            sec[key] = val
    return out


def test_command_line_writes_the_calibration():
    with tempfile.TemporaryDirectory() as d:
        data = os.path.join(d, "session")
        os.mkdir(data)
        _write_session(data)
        cfg_path = os.path.join(d, "config.yaml")
        proc = subprocess.run(
            [sys.executable, os.path.join(TOOLS, "inslib_imu_calib.py"),
             "--csv", data, "--init-sec", str(INIT_SEC - 1.0),
             "--gravity", str(G), "--mag-field-ut", str(FIELD_UT),
             "-y", "-o", cfg_path],
            capture_output=True, text=True, timeout=600)
        assert proc.returncode == 0, proc.stdout + proc.stderr
        assert "%d static poses" % N_POSES in proc.stdout, proc.stdout
        cfg = _read_config(cfg_path)

    imu, mag = cfg["imu"], cfg["mag"]
    cm = calib._colmajor_to_mat3
    assert np.abs(np.asarray(cm(imu["acc_misalignment"])) - M_ACC).max() < 1e-3
    assert np.abs(np.asarray(cm(imu["gyr_misalignment"])) - M_GYR).max() < 1e-3
    assert np.abs(np.asarray(imu["acc_fixed_bias"]) - B_ACC).max() < 5e-3
    assert np.abs(np.asarray(imu["gyr_fixed_bias"]) - B_GYR).max() \
        < math.radians(0.01)
    assert np.abs(np.asarray(cm(mag["misalignment"])) - M_MAG).max() < 3e-3
    assert np.abs(np.asarray(mag["fixed_bias"]) - B_MAG).max() < 0.1


def test_command_line_mag_csv_needs_csv():
    proc = subprocess.run(
        [sys.executable, os.path.join(TOOLS, "inslib_imu_calib.py"),
         "--port", "NONE", "--mag-csv", "mag.csv"],
        capture_output=True, text=True, timeout=60)
    assert proc.returncode != 0
    assert "--mag-csv needs --csv" in proc.stderr, proc.stderr


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
