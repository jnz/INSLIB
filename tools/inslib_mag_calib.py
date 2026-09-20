#!/usr/bin/env python3
"""Magnetometer calibration (library, not a command).

Solves the same model the rest of INSLIB applies to a magnetometer
(REQ-NAV-039, `mag: misalignment` / `mag: fixed_bias` in config.yaml):

    corrected = M * (raw - fixed_bias)      [uT]

`fixed_bias` is the hard iron: whatever is permanently magnetised on the
board and in the housing adds a constant field in the body frame and moves
the whole measurement cloud off the origin. `M` carries the soft iron
(nearby ferrous material bends the field, so the cloud becomes an
ellipsoid rather than a sphere), the sensor's own scale and axis
non-orthogonality, and, if it was solved for, the rotation onto the IMU
triad.

**Why the ellipsoid fit leaves room for that rotation.** Turned through
every attitude, a perfect magnetometer traces a sphere of radius |F|, the
local field strength. Hard and soft iron turn that sphere into an offset
ellipsoid, and fitting the ellipsoid recovers the centre and a matrix that
maps it back onto a sphere. But a sphere is unchanged by rotation, so the
fit cannot possibly see one: it determines `M` only up to a rotation,
which is why the fit here is pinned to the symmetric square root, the one
that rotates nothing. That leftover rotation is exactly the mounting
misalignment between the magnetometer and the IMU, and
`inslib_frame_align.align_by_dip` measures it from the static poses of the
same session (see there for how). Composing the two,

    M = R_mag_to_imu * A_symmetric

is a calibration whose output is not just the right SIZE but points along
the IMU axes the filter's attitude refers to. Without it, a magnetometer
that is 3 deg off its board contributes a 3 deg heading error to an
otherwise perfect fusion.

**Scale.** The fit forces |corrected| onto one number, which has to be
chosen: pass the WMM field strength for where the calibration is being
done, or leave it at 0 to keep whatever the sensor is reading on average
(the geometric mean of the fitted ellipsoid's semi-axes). The direction,
and therefore the heading, is the same either way, but INSLIB gates the
magnetometer on the field strength agreeing with its own WMM lookup, so
the real value is the better answer whenever it is known.

Used by tools/inslib_imu_calib.py and tools/inslib_calib_gui.py, which
record the session. Nothing here does I/O.

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import inslib_frame_align as fa   # noqa: E402

# Below this the ellipsoid is not worth fitting: 9 parameters over a cloud
# that has to cover directions, not just be numerous.
MIN_SAMPLES = 200
# For the alignment. The dip is a nuisance parameter solved in closed
# form, so N poses leave N-1 independent constraints for the 3 unknowns of
# the rotation: 4 is the arithmetic floor and 4 is still not enough in
# practice. Over synthetic sessions the median error falls from unusable
# at 4 poses to a fraction of a degree at 5 and settles by 8, so 5 is the
# lowest count worth returning an answer from. More is better.
MIN_POSES = 5


# ===========================================================================
# Ellipsoid fit
# ===========================================================================

def _sym_sqrt(m):
    """Symmetric positive definite square root, via the eigendecomposition.

    The square root is only unique up to a rotation (any Q*sqrt with Q
    orthogonal squares back to m), and this is the symmetric one, which is
    the whole point: it adds no rotation of its own, so the rotation that
    is solved for later is the real mounting misalignment and not an
    artefact of the fit."""
    w, v = np.linalg.eigh(m)
    if np.any(w <= 0.0):
        raise ValueError("the fitted quadric is not an ellipsoid")
    return v @ np.diag(np.sqrt(w)) @ v.T


def _fit_quadric(x):
    """Least-squares general quadric through the (already scaled) cloud.

    Returns (Q, u, d) of x'Qx + 2u'x + d = 0, normalised so that Q is
    positive definite when the cloud really is an ellipsoid."""
    d = np.column_stack([
        x[:, 0] ** 2, x[:, 1] ** 2, x[:, 2] ** 2,
        2.0 * x[:, 0] * x[:, 1], 2.0 * x[:, 0] * x[:, 2],
        2.0 * x[:, 1] * x[:, 2],
        2.0 * x[:, 0], 2.0 * x[:, 1], 2.0 * x[:, 2],
        np.ones(len(x))])
    # The smallest right singular vector is the least-squares solution
    # under |v| = 1, which is the constraint that keeps the fit off the
    # trivial all-zero answer without preferring any particular scaling.
    _u, _s, vt = np.linalg.svd(d, full_matrices=False)
    v = vt[-1]
    q = np.array([[v[0], v[3], v[4]],
                  [v[3], v[1], v[5]],
                  [v[4], v[5], v[2]]])
    if np.trace(q) < 0.0:
        q, v = -q, -v
    return q, v[6:9], float(v[9])


@dataclass
class EllipsoidFit:
    bias: np.ndarray        # hard iron, in the units of the input [uT]
    matrix: np.ndarray      # symmetric, maps the cloud onto the UNIT sphere
    radius: float           # geometric mean semi-axis, the measured |F|
    semi_axes: np.ndarray   # sorted, in input units
    residual_unit: float    # |corrected| scatter, relative to the radius
    n_used: int
    n_dropped: int
    spread: float           # direction coverage, 1.0 = a full sphere


def ellipsoid_fit(samples, iterations=3, reject_sigma=4.0):
    """Hard iron and soft iron from a cloud of raw magnetometer samples.

    The whole recording is used, not only the static parts: unlike the
    accelerometer, a magnetometer does not care whether the unit is
    moving, and the turns between poses are where most of the direction
    coverage comes from.

    Outliers are dropped and the fit repeated, because a magnetometer
    picks up whatever passed close to it (a phone, a screwdriver, the
    laptop the recording is going into) and a handful of such samples pull
    an unweighted least-squares fit noticeably."""
    m = np.asarray(samples, dtype=float)
    if m.ndim != 2 or m.shape[1] != 3:
        raise ValueError("magnetometer samples must be (N, 3)")
    m = m[np.isfinite(m).all(axis=1)]
    if len(m) < MIN_SAMPLES:
        raise ValueError("only %d magnetometer samples, need at least %d"
                         % (len(m), MIN_SAMPLES))

    scale = float(np.mean(np.linalg.norm(m, axis=1)))
    if not (scale > 0.0):
        raise ValueError("the magnetometer reads zero, is it connected?")
    keep = np.ones(len(m), dtype=bool)
    bias = np.zeros(3)
    a_unit = np.eye(3)
    for it in range(max(1, iterations)):
        x = m[keep] / scale
        q, u, d = _fit_quadric(x)
        try:
            centre = np.linalg.solve(q, -u)
        except np.linalg.LinAlgError:
            raise ValueError("the magnetometer samples do not describe an "
                             "ellipsoid: turn the unit through more "
                             "directions") from None
        k = float(centre @ q @ centre) - d
        if k <= 0.0:
            raise ValueError("the magnetometer samples do not describe an "
                             "ellipsoid: turn the unit through more "
                             "directions")
        a_unit = _sym_sqrt(q / k) / scale
        bias = centre * scale
        if it + 1 >= iterations:
            break
        r = np.linalg.norm((m - bias) @ a_unit.T, axis=1) - 1.0
        # Median absolute deviation, so the threshold is not set by the
        # very samples it is meant to remove.
        mad = float(np.median(np.abs(r - np.median(r))))
        lim = max(reject_sigma * 1.4826 * mad, 1e-3)
        new_keep = np.abs(r) <= lim
        if new_keep.sum() < MIN_SAMPLES or (new_keep == keep).all():
            break
        keep = new_keep

    unit_r = np.linalg.norm((m[keep] - bias) @ a_unit.T, axis=1)
    radius = float(np.linalg.det(a_unit) ** (-1.0 / 3.0))
    axes = np.sort(1.0 / np.sqrt(np.linalg.eigvalsh(a_unit.T @ a_unit)))
    dirs = (m[keep] - bias)
    dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
    # Smallest eigenvalue of the direction scatter, scaled so that a cloud
    # spread evenly over the sphere gives 1 and one confined to a plane
    # gives 0. A plane leaves the hard iron along its normal unobservable,
    # which no residual will show.
    spread = float(3.0 * np.linalg.eigvalsh(dirs.T @ dirs / len(dirs))[0])
    return EllipsoidFit(bias=bias, matrix=a_unit, radius=radius,
                        semi_axes=axes,
                        residual_unit=float(np.sqrt(np.mean((unit_r - 1.0) ** 2))),
                        n_used=int(keep.sum()),
                        n_dropped=int(len(m) - keep.sum()),
                        spread=spread)


# ===========================================================================
# The result
# ===========================================================================

@dataclass
class MagCalibration:
    """One session's magnetometer calibration, in config units."""
    matrix: list                    # row-major nested 3x3, uT -> uT
    bias: list                      # hard iron [uT]
    field_ut: float                 # what |corrected| is scaled to
    field_source: str               # where that number came from
    n_samples: int
    n_dropped: int
    spread: float
    residual_rms_ut: float          # |m| scatter after calibration
    raw_rms_ut: float               # and before, for the before/after read
    soft_iron_percent: list         # semi-axis spread of the ellipsoid
    hard_iron_ut: float             # length of the bias, one number
    align: object = None            # inslib_frame_align.DipAlignment or None
    warnings: list = field(default_factory=list)

    def as_matrix(self):
        return np.asarray(self.matrix, dtype=float)

    def rotated(self, R):
        """The same calibration expressed in a rotated body frame.

        Used when a housing alignment is folded in afterwards: the
        magnetometer has to follow the IMU into the new frame, or the
        heading it feeds the filter is referenced to axes nothing else
        uses."""
        if R is None:
            return self
        out = MagCalibration(**{k: getattr(self, k) for k in
                                self.__dataclass_fields__})
        out.matrix = [[float(x) for x in row]
                      for row in np.asarray(R, dtype=float) @ self.as_matrix()]
        return out

    def summary(self):
        lines = [
            "magnetometer: %d samples, %.1f uT field (%s)"
            % (self.n_samples, self.field_ut, self.field_source),
            "hard iron   [%s] uT"
            % ", ".join("%+.2f" % b for b in self.bias),
            "soft iron   [%s] %% (semi-axis spread)"
            % ", ".join("%+.1f" % s for s in self.soft_iron_percent),
            "|m| scatter %.3f -> %.3f uT, direction coverage %.2f"
            % (self.raw_rms_ut, self.residual_rms_ut, self.spread),
        ]
        if self.align is not None:
            lines += self.align.summary()
        else:
            lines.append("not aligned to the IMU: soft iron and hard iron "
                         "only")
        return lines + ["! " + w for w in self.warnings]


def solve(mag_samples, poses=None, field_ut=0.0, field_source="",
          align=True, log=None):
    """Full magnetometer calibration.

    `mag_samples` is the whole session (N, 3) in uT. `poses` is one
    (accelerometer, magnetometer) mean pair per static pose, the
    accelerometer already IMU-calibrated, used only for the alignment.
    `field_ut` is the local field strength to scale the result to, 0 to
    keep what the sensor measures on average.

    Raises ValueError when the session cannot support the fit."""
    def say(msg):
        if log is not None:
            log(msg)

    fit = ellipsoid_fit(mag_samples)
    say("ellipsoid fit over %d samples (%d dropped as outliers)"
        % (fit.n_used, fit.n_dropped))
    known = bool(field_ut) and float(field_ut) > 0.0
    field = float(field_ut) if known else fit.radius
    source = (field_source or "given") if known else "measured, no reference"
    m_sym = fit.matrix * field
    bias = fit.bias

    warnings = []
    dip = None
    if align and poses:
        acc = [p[0] for p in poses]
        cal = [m_sym @ (np.asarray(p[1], dtype=float) - bias) for p in poses]
        if len(acc) >= MIN_POSES:
            dip = fa.align_by_dip(acc, cal)
            say("aligned to the IMU over %d poses: %.2f deg"
                % (dip.n_poses, dip.angle_deg))
        else:
            warnings.append(
                "only %d static poses with magnetometer samples, need %d to "
                "align the magnetometer to the IMU: hard and soft iron only"
                % (len(acc), MIN_POSES))
    elif align:
        warnings.append("no static poses supplied, so the magnetometer is "
                        "calibrated but not aligned to the IMU")

    matrix = m_sym if dip is None else dip.as_matrix() @ m_sym

    raw = np.asarray(mag_samples, dtype=float)
    raw = raw[np.isfinite(raw).all(axis=1)]
    raw_rms = float(np.std(np.linalg.norm(raw, axis=1)))
    axes_pct = [(float(a) / fit.radius - 1.0) * 100.0 for a in fit.semi_axes]

    if known and abs(fit.radius - field) > 0.25 * field:
        warnings.append(
            "the measured field (%.1f uT) is %.0f %% off the reference "
            "(%.1f uT): either the sensor scale is wrong or the field where "
            "this was recorded is not the undisturbed one"
            % (fit.radius, 100.0 * abs(fit.radius - field) / field, field))
    if fit.spread < 0.25:
        warnings.append(
            "the samples cover %.2f of the sphere: with the directions this "
            "flat the hard iron across the thin axis is guesswork. Turn the "
            "unit through all three axes" % fit.spread)
    if max(abs(p) for p in axes_pct) > 25.0:
        warnings.append(
            "the ellipsoid is %.0f %% out of round, which is a lot of soft "
            "iron. Check for steel close to the sensor (it does not have to "
            "be magnetic to bend the field)" % max(abs(p) for p in axes_pct))
    hard = float(np.linalg.norm(bias))
    if hard > 0.7 * field:
        warnings.append(
            "the hard iron (%.1f uT) is comparable to the field itself "
            "(%.1f uT): something on the board is strongly magnetised, and "
            "the calibration only holds as long as it stays that way"
            % (hard, field))
    return MagCalibration(
        matrix=[[float(x) for x in row] for row in matrix],
        bias=[float(b) for b in bias],
        field_ut=field, field_source=source,
        n_samples=int(fit.n_used), n_dropped=int(fit.n_dropped),
        spread=fit.spread, residual_rms_ut=fit.residual_unit * field,
        raw_rms_ut=raw_rms, soft_iron_percent=axes_pct,
        hard_iron_ut=hard, align=dip, warnings=warnings)


def apply_calib(raw, matrix, bias):
    """corrected = M * (raw - bias), for a whole (N, 3) block."""
    return (np.asarray(raw, dtype=float)
            - np.asarray(bias, dtype=float)) @ np.asarray(matrix,
                                                          dtype=float).T


# ===========================================================================
# Local field reference
# ===========================================================================

def wmm_reference(lat_deg, lon_deg, year):
    """(field strength [uT], declination [deg], inclination [deg]) from
    INSLIB's own World Magnetic Model, or None when the shared library is
    not built.

    INSLIB's own model on purpose: the number written into config.yaml is
    held against the same lookup the filter arms its magnetometer fusion
    with, so a disagreement here is a real disagreement and not two
    different models being compared."""
    try:
        import ctypes
        sys.path.insert(0, os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "..", "python"))
        from INSLIB._core import _lib as lib   # noqa: F401
        lib.magnetic_field_ned_uT.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float * 3]
        lib.magnetic_field_ned_uT.restype = None
        out = (ctypes.c_float * 3)()
        lib.magnetic_field_ned_uT(float(lat_deg), float(lon_deg),
                                  float(year), out)
    except Exception:
        return None
    n, e, d = float(out[0]), float(out[1]), float(out[2])
    f = math.sqrt(n * n + e * e + d * d)
    if not (f > 0.0):
        return None
    return (f, math.degrees(math.atan2(e, n)),
            math.degrees(math.atan2(d, math.hypot(n, e))))
