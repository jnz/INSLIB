#!/usr/bin/env python3
"""Mounting rotations from direction observations (library, not a command).

Two mounting questions live here, because they are the same problem: a
sensor triad sits in a housing at an orientation nobody measured, and the
only instruments available are the sensors themselves.

  1. **Housing alignment.** Put the unit down on a level surface. The
     accelerometer then measures a direction that is known in the HOUSING
     frame (straight up through whichever face is resting on the table),
     so the rotation between what it reads and what it should read is the
     mounting error. One placement gives the two tilt degrees of freedom
     (roll and pitch), and nothing about the rotation around gravity: a
     level box turned on the table still reads level. A second placement
     on a face that is not parallel to the first one adds that third
     degree of freedom, which is what `housing_rotation` solves for with
     Wahba's problem (Kabsch/SVD). Note that two OPPOSITE faces (left and
     right) are parallel and therefore do not: they pin the axis they
     rotate about and leave the rotation about it free. Level plus one
     side is the shortest recipe that gives all three.

  2. **Magnetometer alignment.** No known direction is available here (the
     field points wherever it points, and the heading of the unit is
     exactly what is not known), so `align_by_dip` uses the one thing that
     does not change while the unit is turned: the angle between the local
     magnetic field and gravity. Both sensors see the same rigid rotation
     between poses, so a residual rotation between the two triads shows up
     as that angle wandering with attitude. Driving the wander to zero
     recovers all three degrees of freedom as soon as the poses point in
     genuinely different directions.

Conventions follow the rest of INSLIB: body frame FRD, a rotation matrix
maps a vector out of the frame named first into the frame named second
(`v_housing = R * v_imu`), and Tait-Bryan ZYX roll/pitch/yaw is extracted
the way `ins_rotmat_to_rpy` does it, so the numbers printed here read like
the numbers the filter prints.

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

DEG = 180.0 / math.pi

# Where "up" points, in the housing frame, for each face the unit can be
# rested on. The accelerometer measures specific force, which at rest is
# the reaction to gravity and therefore points UP (level and upright, FRD
# z down, it reads (0, 0, -g), see ins.c's leveling).
#
# Squareness of the housing is assumed, not measured: resting the unit on
# its left face is taken to mean that the housing y axis points straight
# up. A face that is not perpendicular to the others enters the result as
# a mounting error, which is why the per-placement residuals below are
# worth reading before trusting a multi-face solve.
PLACEMENTS = (
    ("bottom", "bottom face down (normal, upright)", (0.0, 0.0, -1.0)),
    ("left", "left face down", (0.0, 1.0, 0.0)),
    ("right", "right face down", (0.0, -1.0, 0.0)),
    ("nose", "front face down (nose down)", (-1.0, 0.0, 0.0)),
    ("tail", "rear face down (nose up)", (1.0, 0.0, 0.0)),
    ("top", "top face down (upside down)", (0.0, 0.0, 1.0)),
)

PLACEMENT_LABEL = {k: label for k, label, _ in PLACEMENTS}
PLACEMENT_IDEAL = {k: np.array(v, dtype=float) for k, _, v in PLACEMENTS}


# ===========================================================================
# Small rotation toolbox
# ===========================================================================

def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rodrigues(rotvec):
    """exp(skew(rotvec)): the rotation of |rotvec| rad about its axis."""
    r = np.asarray(rotvec, dtype=float)
    ang = float(np.linalg.norm(r))
    if ang < 1e-12:
        return np.eye(3) + skew(r)      # first order is exact enough here
    k = skew(r / ang)
    return np.eye(3) + math.sin(ang) * k + (1.0 - math.cos(ang)) * (k @ k)


def unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length direction")
    return v / n


def rotation_angle_deg(R):
    """Total rotation angle of R, the one number that says how far off it
    is regardless of how the three axes share the blame."""
    c = (float(np.trace(R)) - 1.0) * 0.5
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def dcm_to_rpy_deg(R):
    """ZYX Tait-Bryan roll/pitch/yaw [deg], as ins_rotmat_to_rpy extracts
    them: pitch = asin(-R(2,0)), roll = atan2(R(2,1), R(2,2)),
    yaw = atan2(R(1,0), R(0,0)).

    Applied to a mounting rotation `R_a_to_b` these read as the attitude
    of frame a inside frame b, the same way the filter's roll/pitch/yaw
    read as the attitude of the body in the navigation frame."""
    R = np.asarray(R, dtype=float)
    sp = max(-1.0, min(1.0, -float(R[2, 0])))
    pitch = math.asin(sp)
    if abs(sp) < 0.9999:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = math.atan2(-float(R[1, 2]), float(R[1, 1]))
        yaw = 0.0
    return [roll * DEG, pitch * DEG, yaw * DEG]


def angle_between_deg(a, b):
    c = float(np.dot(unit(a), unit(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def minimal_rotation(v_from, v_to):
    """The rotation that takes v_from onto v_to and does nothing else.

    Used when only one direction pair is available: of all the rotations
    that map the one onto the other, this is the one that leaves the
    remaining degree of freedom alone instead of inventing a value for
    it."""
    a, b = unit(v_from), unit(v_to)
    axis = np.cross(a, b)
    s = float(np.linalg.norm(axis))
    c = float(np.dot(a, b))
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        # Antiparallel: every axis perpendicular to a turns one into the
        # other, so pick one rather than dividing by zero.
        seed = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            seed = np.array([0.0, 1.0, 0.0])
        axis = unit(np.cross(a, seed))
        return rodrigues(axis * math.pi)
    return rodrigues(axis / s * math.atan2(s, c))


def kabsch(v_meas, v_ideal, weights=None):
    """Wahba's problem: the rotation R minimising sum |R*v_meas - v_ideal|^2.

    Both sets are normalised first, so this is direction matching and the
    lengths (a slightly-off gravity magnitude) do not weight the fit."""
    a = np.array([unit(v) for v in v_meas])
    b = np.array([unit(v) for v in v_ideal])
    w = np.ones(len(a)) if weights is None else np.asarray(weights, float)
    h = (b * w[:, None]).T @ a
    u, _s, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1.0, 1.0, d]) @ vt


# ===========================================================================
# Housing alignment
# ===========================================================================

@dataclass
class HousingAlignment:
    """The IMU's mounting attitude inside the housing.

    `matrix` maps IMU axes onto housing axes, so it folds into the
    calibration as M_new = matrix * M: the same correction the config's
    3x3 already applies for scale and axis non-orthogonality, extended by
    where the part ended up on the board and where the board ended up in
    the box."""
    matrix: list                    # row-major nested 3x3, v_housing = R*v_imu
    rpy_deg: list                   # IMU attitude in the housing, ZYX
    angle_deg: float                # total rotation
    placements: list = field(default_factory=list)   # keys, in capture order
    residual_deg: list = field(default_factory=list)  # per placement, after fit
    tilt_only: bool = True          # False once yaw is observable too
    free_axis: list = field(default_factory=list)     # the unconstrained axis
    warnings: list = field(default_factory=list)

    def as_matrix(self):
        return np.asarray(self.matrix, dtype=float)

    def summary(self):
        lines = [
            "housing alignment from %d placement(s): %s"
            % (len(self.placements),
               ", ".join(PLACEMENT_LABEL.get(p, p) for p in self.placements)),
            "IMU sits at roll %+.2f, pitch %+.2f, yaw %+.2f deg in the housing"
            % tuple(self.rpy_deg),
            "total correction %.2f deg" % self.angle_deg,
        ]
        if self.residual_deg:
            lines.append("residual per placement [%s] deg"
                         % ", ".join("%.2f" % r for r in self.residual_deg))
        if self.tilt_only:
            lines.append("roll and pitch only: the rotation about the "
                         "measured axis stays as it was")
        return lines + ["! " + w for w in self.warnings]


def placements_parallel(key_a, key_b):
    """Whether two faces rest on the same axis.

    Bottom and top are, left and right are: they measure the same
    direction and the second one says nothing the first one did not."""
    d = float(np.dot(PLACEMENT_IDEAL[key_a], PLACEMENT_IDEAL[key_b]))
    return abs(d) > 0.99


def nearest_placement(acc):
    """(key, angle in deg) of the face that best explains this direction.

    A placement captured as the wrong face is the one mistake the solve
    cannot see: it fits whatever it is given and only the residual grows,
    which reads the same as a surface that was not level. Comparing the
    measurement against every face says which one it actually was."""
    v = unit(acc)
    best = min(PLACEMENT_IDEAL,
               key=lambda k: angle_between_deg(v, PLACEMENT_IDEAL[k]))
    return best, angle_between_deg(v, PLACEMENT_IDEAL[best])


def housing_rotation(observations):
    """Solve the mounting rotation from placements on a level surface.

    `observations` is a list of (placement_key, acc_vector), the mean
    specific force the (calibrated) accelerometer measured while the unit
    rested on that face. Returns a HousingAlignment.

    One placement, or several that all rest on parallel faces, constrains
    two degrees of freedom and the result is the minimal tilt: roll and
    pitch are corrected and the rotation about the measured direction is
    left alone. Two placements on non-parallel faces constrain all three
    and the solve becomes a plain Wahba problem over the direction pairs."""
    obs = [(k, np.asarray(v, dtype=float)) for k, v in observations]
    if not obs:
        raise ValueError("no placements captured")
    for k, _v in obs:
        if k not in PLACEMENT_IDEAL:
            raise ValueError("unknown placement '%s'" % k)
    meas = [unit(v) for _k, v in obs]
    ideal = [PLACEMENT_IDEAL[k] for k, _v in obs]
    keys = [k for k, _v in obs]
    warnings = []

    # Rank of the ideal directions: parallel faces (bottom and top, left
    # and right) all describe the same axis and cannot say anything about
    # the rotation about it, however many of them are captured.
    scatter = sum(np.outer(v, v) for v in ideal)
    evals = np.linalg.eigvalsh(scatter)
    tilt_only = evals[-2] < 1e-6 * max(evals[-1], 1e-12)

    if tilt_only:
        # Average the placements after folding the opposite faces onto the
        # first one, so a bottom/top pair or a left/right pair averages
        # instead of fighting.
        ref = ideal[0]
        acc = np.zeros(3)
        for v, i in zip(meas, ideal):
            acc += v * (1.0 if float(np.dot(i, ref)) >= 0.0 else -1.0)
        R = minimal_rotation(acc, ref)
        free = list(ref)
        if len(obs) > 1:
            warnings.append(
                "all placements rest on parallel faces, so this is still a "
                "roll/pitch correction. Add a placement on a face that is "
                "not parallel to them (level plus one side) to pin the "
                "rotation about that axis")
    else:
        R = kabsch(meas, ideal)
        free = []

    residual = [angle_between_deg(R @ v, i) for v, i in zip(meas, ideal)]
    if residual and max(residual) > 1.0:
        warnings.append(
            "%.1f deg left over on one placement after the fit: either the "
            "surface was not level for every placement, or the housing "
            "faces are not square to one another (nothing here can tell "
            "those apart)" % max(residual))
    ang = rotation_angle_deg(R)
    if ang > 15.0:
        warnings.append(
            "%.0f deg is a lot for a mounting error. Check that the "
            "placement really matches the face it was captured as, "
            "an axis swap belongs in the driver, not in this matrix" % ang)
    return HousingAlignment(matrix=[[float(x) for x in row] for row in R],
                            rpy_deg=dcm_to_rpy_deg(R),
                            angle_deg=ang, placements=keys,
                            residual_deg=residual, tilt_only=tilt_only,
                            free_axis=[float(x) for x in free],
                            warnings=warnings)


# ===========================================================================
# Magnetometer to IMU alignment
# ===========================================================================

@dataclass
class DipAlignment:
    """The magnetometer's mounting attitude relative to the IMU."""
    matrix: list                # row-major 3x3, v_imu = R * v_mag
    rpy_deg: list
    angle_deg: float
    n_poses: int
    dip_deg: float              # measured inclination of the local field
    scatter_before_deg: float   # how much the dip wandered over the poses
    scatter_after_deg: float    # and how much is left after the fit
    observability: float        # smallest singular value of the fit [-]
    warnings: list = field(default_factory=list)

    def as_matrix(self):
        return np.asarray(self.matrix, dtype=float)

    def summary(self):
        return [
            "mag/IMU misalignment roll %+.2f, pitch %+.2f, yaw %+.2f deg "
            "(%.2f deg total)" % (self.rpy_deg[0], self.rpy_deg[1],
                                  self.rpy_deg[2], self.angle_deg),
            "measured dip %.2f deg, scatter over %d poses %.2f -> %.2f deg"
            % (self.dip_deg, self.n_poses, self.scatter_before_deg,
               self.scatter_after_deg),
        ] + ["! " + w for w in self.warnings]


def align_by_dip(acc_vecs, mag_vecs, max_iter=60):
    """Rotation of the magnetometer triad onto the IMU triad.

    One (accelerometer, magnetometer) pair per static pose, in the same
    order. Both are treated as directions.

    The angle between the local field and gravity is a property of where
    on earth the unit is, not of how it is being held, so it must come out
    the same in every pose. It does not when the two triads disagree about
    where the axes are, and the way it varies with attitude identifies the
    rotation: perturbing R about an axis theta changes pose i's cosine by
    theta . ((R*m_i) x a_i), and those gradients point in different
    directions for poses at different attitudes. The dip itself is solved
    in closed form rather than carried as a fourth unknown, which leaves
    N-1 independent constraints from N poses: four is the arithmetic floor
    for the three unknowns and four still misses in practice, so five is
    the lowest count this returns an answer from. More is better.

    What it cannot do is separate a misalignment from a field that really
    is different from pose to pose, so calibrate away from steel furniture
    and do not move the unit across the room between poses."""
    a = np.array([unit(v) for v in acc_vecs])
    m = np.array([unit(v) for v in mag_vecs])
    if len(a) != len(m):
        raise ValueError("need one accelerometer vector per magnetometer one")
    if len(a) < 5:
        raise ValueError("need at least 5 poses to align the magnetometer, "
                         "got %d" % len(a))

    def scatter(rot):
        d = np.clip((m @ rot.T * a).sum(axis=1), -1.0, 1.0)
        return np.degrees(np.arccos(d))

    before = scatter(np.eye(3))
    R = np.eye(3)
    smin = 0.0
    for _ in range(max_iter):
        v = m @ R.T
        d = (v * a).sum(axis=1)
        # The dip angle itself is a nuisance parameter: subtracting the
        # mean from both the residual and the Jacobian solves for it in
        # closed form instead of carrying it as a fourth unknown.
        j = np.cross(v, a)
        r = d - d.mean()
        jc = j - j.mean(axis=0)
        smin = float(np.linalg.svd(jc, compute_uv=False)[-1])
        step, _res, _rank, _sv = np.linalg.lstsq(jc, -r, rcond=None)
        R = rodrigues(step) @ R
        if float(np.linalg.norm(step)) < 1e-12:
            break

    after = scatter(R)
    ang = rotation_angle_deg(R)
    warnings = []
    # Normalised by the number of poses, so it says "how much do the poses
    # disagree about where the axes are" rather than "how many are there".
    obs = smin / math.sqrt(len(a))
    if obs < 0.15:
        warnings.append(
            "the poses are too much alike to pin all three axes (spread "
            "%.2f): the alignment may be absorbing pose noise. Turn the "
            "unit through more varied attitudes" % obs)
    if ang > 20.0:
        warnings.append(
            "%.0f deg is more than a mounting tolerance. Either the "
            "magnetometer axes are remapped relative to the IMU (that "
            "belongs in the driver) or the field was disturbed during the "
            "session" % ang)
    if float(np.std(after)) > 3.0:
        warnings.append(
            "the dip still wanders by %.1f deg over the poses after the "
            "fit: something magnetic moved with the unit, or the hard iron "
            "fit is not converged" % float(np.std(after)))
    # The accelerometer points UP at rest, so the angle it makes with the
    # field is 90 deg plus the inclination, which is what a WMM lookup can
    # be held against.
    return DipAlignment(matrix=[[float(x) for x in row] for row in R],
                        rpy_deg=dcm_to_rpy_deg(R), angle_deg=ang,
                        n_poses=len(a), dip_deg=float(np.mean(after)) - 90.0,
                        scatter_before_deg=float(np.std(before)),
                        scatter_after_deg=float(np.std(after)),
                        observability=obs, warnings=warnings)
