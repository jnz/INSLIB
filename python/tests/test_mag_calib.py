#!/usr/bin/env python3
"""Regression tests for tools/inslib_mag_calib.py and inslib_frame_align.py.

Synthetic data with a known truth throughout, because that is what these
two solvers can be held against: a hard iron, a soft iron, a magnetometer
turned against the IMU and an IMU turned inside its housing are all put
into the generated data and have to come back out of the fit.

The one thing synthetic data cannot check is whether the model matches the
hardware, so the conventions are pinned separately against the rest of
INSLIB: the specific force of a level unit is (0, 0, -g) as in ins.c's
leveling, and the roll/pitch/yaw extraction is the one in
ins_rotmat_to_rpy.

Runs under pytest or standalone:

    python3 python/tests/test_mag_calib.py
"""

import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inslib_frame_align as fa    # noqa: E402
import inslib_mag_calib as mc      # noqa: E402
import inslib_imu_calib as calib     # noqa: E402  (the config writer)

G = 9.80665
FIELD_UT = 48.5
DIP_DEG = 64.0
DECL_DEG = 3.5

# Truth: a symmetric soft iron (the fit can only ever recover the
# symmetric part, the rest is the alignment), a hard iron of a few uT and
# a magnetometer sitting a couple of degrees off the IMU.
A_SOFT = np.array([[1.06, 0.03, -0.02],
                   [0.03, 0.95, 0.015],
                   [-0.02, 0.015, 1.01]])
B_HARD = np.array([7.5, -4.2, 11.0])
R_MAG = fa.rodrigues(np.radians([1.5, -2.5, 3.0]))


def _field_ned():
    dip, dec = math.radians(DIP_DEG), math.radians(DECL_DEG)
    return FIELD_UT * np.array([math.cos(dip) * math.cos(dec),
                                math.cos(dip) * math.sin(dec),
                                math.sin(dip)])


def _attitudes(rng, n=40):
    """Body-from-nav rotations spread over the sphere."""
    out = []
    for _ in range(n):
        v = rng.normal(size=3)
        ang = rng.uniform(0.0, math.pi)
        out.append(fa.rodrigues(v / np.linalg.norm(v) * ang))
    return out


def _session(rng, n_poses=40, noise_ut=0.05):
    """(mag_raw cloud, pose pairs) for a whole synthetic session."""
    f_n = _field_ned()
    g_n = np.array([0.0, 0.0, G])
    a_inv = np.linalg.inv(A_SOFT)
    cloud, poses = [], []
    for c_bn in _attitudes(rng, n_poses):
        m_body = c_bn @ f_n                     # true field, IMU axes
        acc = -c_bn @ g_n                       # specific force at rest
        raw = a_inv @ (R_MAG.T @ m_body) + B_HARD
        block = raw + rng.normal(0.0, noise_ut, (25, 3))
        cloud.append(block)
        poses.append((acc + rng.normal(0.0, 0.01, 3), block.mean(axis=0)))
    return np.concatenate(cloud), poses


# --------------------------------------------------------------------------
# Magnetometer
# --------------------------------------------------------------------------

def test_ellipsoid_recovers_hard_and_soft_iron():
    rng = np.random.default_rng(7)
    cloud, _poses = _session(rng)
    fit = mc.ellipsoid_fit(cloud)

    assert np.abs(fit.bias - B_HARD).max() < 0.15, fit.bias
    # The fit maps onto the unit sphere, so scaling by the true field must
    # reproduce the symmetric truth.
    assert np.abs(fit.matrix * FIELD_UT - A_SOFT).max() < 5e-3
    # The radius is the geometric mean semi-axis of the fitted ellipsoid,
    # so the soft iron's own volume change is in it. The FIELD_UT scaling
    # above is what puts the calibrated output back on the true field.
    want = FIELD_UT / np.linalg.det(A_SOFT) ** (1.0 / 3.0)
    assert abs(fit.radius - want) < 0.05, (fit.radius, want)
    # 1.0 is a cloud spread evenly over the whole sphere, which 40 hand
    # placed poses are not. What matters is that it is far from the flat
    # case below.
    assert fit.spread > 0.4, fit.spread


def test_full_solve_recovers_alignment_and_dip():
    rng = np.random.default_rng(11)
    cloud, poses = _session(rng)
    cal = mc.solve(cloud, poses, field_ut=FIELD_UT, field_source="test")

    assert np.abs(np.asarray(cal.bias) - B_HARD).max() < 0.15
    # M = R * A, the composition the config carries.
    assert np.abs(cal.as_matrix() - R_MAG @ A_SOFT).max() < 0.01
    assert cal.align is not None
    assert fa.rotation_angle_deg(cal.align.as_matrix() @ R_MAG.T) < 0.5
    assert abs(cal.align.dip_deg - DIP_DEG) < 0.5, cal.align.dip_deg
    # The misalignment is what makes the dip wander, so removing it has to
    # collapse the scatter.
    assert cal.align.scatter_after_deg < 0.2 * cal.align.scatter_before_deg
    assert cal.residual_rms_ut < 0.1, cal.residual_rms_ut
    assert not cal.warnings, cal.warnings


def test_calibration_puts_the_field_where_it_belongs():
    """End to end: applying the result to the raw samples has to give back
    the true body-frame field, not merely the right magnitude."""
    rng = np.random.default_rng(3)
    cloud, poses = _session(rng)
    cal = mc.solve(cloud, poses, field_ut=FIELD_UT)
    f_n = _field_ned()
    for acc, raw in poses[:10]:
        m = mc.apply_calib(np.asarray(raw)[None, :], cal.matrix, cal.bias)[0]
        # Only the field/gravity geometry is checkable per pose (the
        # heading is unknown), which is exactly the constraint that was
        # fitted, so check it against the truth rather than the fit.
        got = fa.angle_between_deg(m, acc)
        want = fa.angle_between_deg(
            np.array([f_n[0], f_n[1], f_n[2]]), np.array([0.0, 0.0, -G]))
        assert abs(got - want) < 0.5, (got, want)


def test_unaligned_when_there_are_no_poses():
    rng = np.random.default_rng(5)
    cloud, _poses = _session(rng)
    cal = mc.solve(cloud, poses=None, field_ut=FIELD_UT)
    assert cal.align is None
    assert cal.warnings
    # Still a valid soft/hard iron fit, just in the magnetometer's frame.
    assert np.abs(np.asarray(cal.bias) - B_HARD).max() < 0.15


def test_flat_coverage_is_reported_not_hidden():
    """Samples that never leave one plane cannot pin the hard iron across
    it. The fit may still converge, so the coverage number is the only
    thing that can say so."""
    rng = np.random.default_rng(9)
    f_n = _field_ned()
    a_inv = np.linalg.inv(A_SOFT)
    rows = []
    for yaw in np.linspace(0.0, 2 * math.pi, 400):
        c_bn = fa.rodrigues([0.0, 0.0, yaw])
        raw = a_inv @ (R_MAG.T @ (c_bn @ f_n)) + B_HARD
        rows.append(raw + rng.normal(0.0, 0.05, 3))
    fit = mc.ellipsoid_fit(np.array(rows))
    assert fit.spread < 0.05, fit.spread


# --------------------------------------------------------------------------
# Housing alignment
# --------------------------------------------------------------------------

R_HOUSING = fa.rodrigues(np.radians([2.0, -1.2, 4.0]))   # IMU inside the box


def _placement_reading(key, rot=R_HOUSING, noise=None, rng=None):
    """What the accelerometer reads with the unit on that face."""
    ideal = fa.PLACEMENT_IDEAL[key] * G
    v = rot.T @ ideal
    if noise and rng is not None:
        v = v + rng.normal(0.0, noise, 3)
    return v


def test_two_faces_recover_the_full_rotation():
    obs = [("bottom", _placement_reading("bottom")),
           ("left", _placement_reading("left"))]
    al = fa.housing_rotation(obs)
    assert not al.tilt_only
    assert fa.rotation_angle_deg(al.as_matrix() @ R_HOUSING.T) < 1e-6
    assert max(al.residual_deg) < 1e-6
    assert not al.warnings


def test_one_face_is_tilt_only_and_leaves_yaw_alone():
    al = fa.housing_rotation([("bottom", _placement_reading("bottom"))])
    assert al.tilt_only
    # Roll and pitch of the correction match the truth, yaw is untouched.
    truth = fa.dcm_to_rpy_deg(R_HOUSING)
    assert abs(al.rpy_deg[0] - truth[0]) < 0.05
    assert abs(al.rpy_deg[1] - truth[1]) < 0.05
    assert abs(al.rpy_deg[2]) < 0.15
    # What it promises is what it delivers: the corrected reading is level.
    got = al.as_matrix() @ _placement_reading("bottom")
    assert abs(got[0]) < 1e-4 and abs(got[1]) < 1e-4
    assert got[2] < 0.0


def test_opposite_faces_alone_cannot_give_yaw():
    """Left and right rest on PARALLEL faces, so they pin the axis they
    turn about and say nothing about the rotation around it."""
    obs = [("left", _placement_reading("left")),
           ("right", _placement_reading("right"))]
    al = fa.housing_rotation(obs)
    assert al.tilt_only
    assert any("parallel" in w for w in al.warnings)
    assert max(al.residual_deg) < 1e-6


def test_placement_error_shows_up_as_a_residual():
    """A face that is not square to the others cannot be told apart from a
    surface that was not level, but it must not pass unnoticed."""
    rng = np.random.default_rng(2)
    bad = fa.rodrigues(np.radians([0.0, 0.0, 0.0])) @ R_HOUSING
    obs = [("bottom", _placement_reading("bottom", bad)),
           ("left", fa.rodrigues(np.radians([5.0, 0.0, 0.0]))
            @ _placement_reading("left", bad)),
           ("nose", _placement_reading("nose", bad, 0.02, rng))]
    al = fa.housing_rotation(obs)
    assert max(al.residual_deg) > 1.0
    assert any("left over" in w for w in al.warnings)


def test_rpy_matches_the_library_convention():
    """Same extraction as ins_rotmat_to_rpy: a pure roll of +10 deg has to
    read as roll +10, and the matrix must map (0,0,-1) the way the leveling
    formula in ins.c does."""
    r = fa.rodrigues(np.radians([10.0, 0.0, 0.0]))
    roll, pitch, yaw = fa.dcm_to_rpy_deg(r)
    assert abs(roll - 10.0) < 1e-9 and abs(pitch) < 1e-9 and abs(yaw) < 1e-9
    # f_b = [g sin(pitch), -g sin(roll) cos(pitch), -g cos(roll) cos(pitch)]
    f = r.T @ np.array([0.0, 0.0, -G])
    assert abs(f[1] - (-G * math.sin(math.radians(10.0)))) < 1e-6
    assert abs(f[2] - (-G * math.cos(math.radians(10.0)))) < 1e-6


def _cfg_mat(flat):
    """A config.yaml 3x3 (flat, column major) as a numpy matrix."""
    return np.asarray([float(x) for x in flat]).reshape(3, 3).T


def test_a_housing_pass_alone_takes_the_magnetometer_with_it():
    """The whole point of the housing rotation is ONE body frame.

    A housing alignment captured on its own rotates the accelerometer and
    the gyroscope of a config that already carries a calibration. A
    magnetometer calibration in the same file has to move by the SAME
    rotation: left behind it stays in the sensor frame, the file is still
    well formed, every residual still looks right, and the heading fed to
    the filter is referenced to axes nothing else uses."""
    r = fa.rodrigues([math.radians(2.0), math.radians(-1.0),
                      math.radians(20.0)])
    acc_m = np.diag([1.01, 0.99, 1.005])
    mag_m = np.array([[1.06, 0.03, -0.02],
                      [0.03, 0.95, 0.015],
                      [-0.02, 0.015, 1.02]])
    existing = {
        "imu": {"acc_misalignment": calib.mat3_to_colmajor(acc_m.tolist()),
                "acc_fixed_bias": [0.0, 0.0, 0.0],
                "gyr_misalignment": calib.mat3_to_colmajor(np.eye(3).tolist()),
                "gyr_fixed_bias": [0.0, 0.0, 0.0]},
        "mag": {"enable": 1,
                "misalignment": calib.mat3_to_colmajor(mag_m.tolist()),
                "fixed_bias": [1.0, 2.0, 3.0]},
    }

    imu_new = calib.rotate_config_imu(existing, r)
    mag_new = calib.rotate_config_mag(existing, r)
    assert mag_new, "the magnetometer section was left behind"

    # Premultiplied, the same way MagCalibration.rotated does it. The
    # order matters: M @ r would rotate the raw samples instead of the
    # corrected ones and is wrong by the soft iron.
    assert np.allclose(_cfg_mat(imu_new["acc_misalignment"][0]), r @ acc_m)
    assert np.allclose(_cfg_mat(mag_new["misalignment"][0]), r @ mag_m)

    # And it is the same rotation on both, which is the property that
    # makes the two sensors describe one frame.
    r_imu = _cfg_mat(imu_new["acc_misalignment"][0]) @ np.linalg.inv(acc_m)
    r_mag = _cfg_mat(mag_new["misalignment"][0]) @ np.linalg.inv(mag_m)
    assert np.allclose(r_imu, r_mag)


def test_a_housing_pass_leaves_a_config_without_a_magnetometer_alone():
    """Nothing to rotate is not an error: most units have no
    magnetometer calibration in the file at all."""
    r = fa.rodrigues([0.0, 0.0, math.radians(5.0)])
    assert calib.rotate_config_mag({"imu": {}}, r) == {}
    assert calib.rotate_config_mag({}, r) == {}
    assert calib.rotate_config_mag(None, r) == {}


def _main():
    fails = []
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok   %s" % name)
            except Exception as e:                  # noqa: BLE001
                fails.append(name)
                print("FAIL %s: %s" % (name, e))
    if fails:
        print("failed: %s" % ", ".join(fails))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
