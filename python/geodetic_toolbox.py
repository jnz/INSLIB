"""
  Geodetic Toolbox
  ----------------

  A collection of math helper functions.

  (c) Jan Zwiener (jan@zwiener.org)
"""

import numpy as np

def quat_from_rpy(r, p, y):
    """
    Convert Euler angle (roll, pitch, and yaw) to a quaternion.

    Valid for arbitrary angles: the result is always the rotation
    Rz(yaw) @ Ry(pitch) @ Rx(roll). Angles outside the canonical Euler
    ranges (e.g. |pitch| > pi/2) describe the same rotation as their
    canonical equivalent, so quat_to_rpy of the result may return a
    different (but equivalent) triplet.

    :param r: roll [rad]
    :param p: pitch [rad]
    :param y: yaw [rad]
    :return: Unit quaternion, describing the rotation. np.array with real part
             at q[0] (qw, qx, qy, qz).
    """
    sr2 = np.sin(r * 0.5)
    cr2 = np.cos(r * 0.5)
    sp2 = np.sin(p * 0.5)
    cp2 = np.cos(p * 0.5)
    sy2 = np.sin(y * 0.5)
    cy2 = np.cos(y * 0.5)
    qreal = cy2 * cp2 * cr2 + sy2 * sp2 * sr2
    q1 = cy2 * cp2 * sr2 - sy2 * sp2 * cr2
    q2 = cy2 * sp2 * cr2 + sy2 * cp2 * sr2
    q3 = sy2 * cp2 * cr2 - cy2 * sp2 * sr2
    q = np.array([qreal, q1, q2, q3])

    return q

def quat_to_matrix(q):
    """
    This function creates a 3x3 rotation matrix from an input quaternion.

    :param q: Input quaternion (qw, qx, qy, qz) that describes the rotation
    :return: Rotation matrix 3x3 (np.array)
    """
    assert len(q) == 4, "Invalid arguments"

    a, b, c, d = q
    a2 = a * a
    b2 = b * b
    c2 = c * c
    d2 = d * d

    R = np.array([
        [a2 + b2 - c2 - d2, 2 * (b * c - a * d), 2 * (b * d + a * c)],
        [2 * (b * c + a * d), a2 - b2 + c2 - d2, 2 * (c * d - a * b)],
        [2 * (b * d - a * c), 2 * (c * d + a * b), a2 - b2 - c2 + d2]
    ])

    return R

def matrix_to_quat(R):
    """
    Converts a 3x3 rotation matrix to a quaternion (w, x, y, z).
    Uses a numerically stable algorithm avoiding division by zero.
    """
    assert R.shape == (3, 3), "Invalid arguments"

    tr = np.trace(R)

    if tr > 0:
        S = 2.0 * np.sqrt(tr + 1.0)
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_rpy(q):
    """
    Extract Euler angles (roll, pitch and yaw) from a quaternion.
    :param q: Input quaternion
    :return: 3x1 vector with angles in radians (roll, pitch, yaw)
    """
    return extract_rpy_from_R_b_to_n(quat_to_matrix(q))

def extract_rpy_from_R_b_to_n(R_b_to_n):
    """
    This function extracts the three angles roll, pitch, and yaw from
    an R_b_to_n matrix (rotation from body to navigation-frame).

    :param R_b_to_n: 3x3 matrix describing a body to n-frame transformation
    :return: 3x1 vector with angles in radians (roll, pitch, yaw)
    """
    assert R_b_to_n.shape == (3, 3), "Invalid arguments"

    sin_pitch = -R_b_to_n[2, 0]
    sin_pitch_clipped = np.clip(sin_pitch, -1.0, 1.0)
    roll = np.arctan2(R_b_to_n[2, 1], R_b_to_n[2, 2])
    pitch = np.arcsin(sin_pitch_clipped)
    yaw = np.arctan2(R_b_to_n[1, 0], R_b_to_n[0, 0])

    return np.array([roll, pitch, yaw])

def quat_norm(q):
    """
    Normalize quaternion (make sure the length is 1.0).
    :param q Input quaternion
    :return Normalized quaternion with length == 1.0
    """
    if len(q) != 4:
        raise ValueError("Invalid arguments")

    abssquared = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2
    if abssquared < 10.0 * np.finfo(float).eps:
        raise ValueError("Quaternion length close to zero")

    qnorm = q / np.sqrt(abssquared)
    return qnorm

def quat_multiply(q1, q2):
    """
    Multiply two quaternions
    :param q1 Quaternion 1
    :param q2 Quaternion 2
    :return Result of q1 * q2
    """
    assert len(q1) == len(q2) == 4, "Invalid arguments"
    a, b, c, d = q1
    q_matrix = np.array([
        [ a, -b, -c, -d],
        [ b,  a, -d,  c],
        [ c,  d,  a, -b],
        [ d, -c,  b,  a]
    ])
    return q_matrix @ q2

def quat_integrate_rotationrate(q, omega, dt_sec):
    """
    Integrate the rotation rate omega over dt_sec to get the new quaternion.
    :param q: 4x1 quaternion (from "body" to "n-frame"/ref. nav. frame). Hamilton.
    :param omega: Rotation rate (rad/s) of body wrt. ref. nav-frame (in body
                  frame coord. system)
    :param dt_sec: Simulation timestep in seconds (>= 0)
    :return: qnext: 4x1 quaternion after dt_sec seconds
    """
    assert q.shape == (4,), "Invalid arguments"
    assert omega.shape == (3,), "Invalid arguments"

    delta = omega*dt_sec
    delta_abs = np.linalg.norm(delta)
    if delta_abs > 1e-8:
        img_part = delta / delta_abs * np.sin(delta_abs*0.5)
        qr = np.block([ np.cos(delta_abs*0.5), img_part ])
        qnext = quat_multiply(q, qr)
        qnext = quat_norm(qnext)
    else:
        qnext = q.copy()

    return qnext

def quat_invert(q):
    """
    Return the inverse of an rotation quaternion.

    :param q: 4x1 orientation (unit-length) quaternion
    :return: Unit quaternion, describing the inverse rotation.
             np.array with real part at q[0] (qw, qx, qy, qz).
    """
    qinv = np.array([ q[0], -q[1], -q[2], -q[3] ])
    return qinv

def angle_diff(a, b):
    """Returns the signed difference between angles a and b in radians, wrapped to [-pi, pi]."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi

def quat_to_axis_angle(q):
    """
    Extract rotation axis and angle from a unit quaternion.

    :param q: 4x1 orientation (unit-length) quaternion as [qw, qx, qy, qz]
    :return: Tuple (axis, angle)
             - axis: 3x1 unit vector describing the rotation axis
             - angle: scalar rotation angle in radians
    """
    if q.shape != (4,):
        raise ValueError("Quaternion must be a 4-element array [qw, qx, qy, qz]")

    q = quat_norm(q)
    w, x, y, z = q
    w = np.clip(w, -1.0, 1.0)

    s = np.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8:
        # w close to +/-1: rotation of 0 or 2*pi, i.e. no rotation.
        # Axis is undefined, return a default.
        return np.array([1.0, 0.0, 0.0]), 0.0

    angle = 2 * np.arccos(w)
    axis = np.array([x, y, z]) / s
    return axis, angle

def mag_heading(mag_body, roll, pitch):
    """
    Tilt-compensated magnetic-compass heading from a magnetometer sample.

    Levels the raw field into the horizontal plane using roll/pitch only
    (yaw is what we solve for), then heading = atan2(-hE, hN). The result
    is relative to MAGNETIC north; add the local declination for true
    north. Returns NaN if the horizontal field vanishes (singular).

    :param mag_body: magnetometer sample in body FRD (x fwd, y right, z down)
    :param roll: roll [rad]
    :param pitch: pitch [rad]
    :return: magnetic heading [rad], NaN if undefined
    """
    mx, my, mz = mag_body
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    h_n = mx * cp + my * sp * sr + mz * sp * cr
    h_e = my * cr - mz * sr
    if h_n == 0.0 and h_e == 0.0:
        return np.nan
    return np.arctan2(-h_e, h_n)
