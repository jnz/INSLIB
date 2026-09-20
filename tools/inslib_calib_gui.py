#!/usr/bin/env python3
"""inslib_calib_gui -- guided IMU calibration for the STM32 sensor board.

Graphical front end for tools/inslib_imu_calib.py: same recording,
same solve, same config.yaml writer. Everything that computes a number
lives in that module (and in inslib_imu_tk.py) and is imported here, so
the console tool and this window cannot drift apart.

The procedure needs no fixture and no known orientations (see
inslib_imu_tk.py for why): leave the unit alone for the initial rest
period, then repeatedly pick it up, turn it to any new attitude, and set
it down for a few seconds. What this window adds is knowing where you
are in that: a live count of the static poses the detector has actually
accepted, a sphere showing which gravity directions are already covered
and which part of the sky is still empty, a stillness meter that says
whether the unit is calm enough to count right now, and live traces.

Two more calibrations ride along on the same session, both about where a
sensor sits rather than what it reads:

  * **Magnetometer** (inslib_mag_calib.py). If the board streams one
    (0x40/0x06), the turns between the poses trace out the field cloud
    that gives hard and soft iron, and the poses themselves give the
    rotation of the magnetometer against the IMU: the angle between the
    field and gravity cannot change with attitude, and how much it does
    is the misalignment. Written as the mag: keys.

The result can go into a config.yaml, or straight into the board:
**Upload to board** writes the session as ONE temperature node into the
board's own calibration table, merged into whatever is already stored.
The board interpolates between its nodes, so calibrating at several
temperatures across sessions is how the table grows to cover a range.
See the Upload section in tools/README.md, and inslib_protocol.md for
what actually goes over the wire.

  * **Housing** (inslib_frame_align.py). Rest the unit on a level surface
    and any tilt it reports is the IMU sitting crooked in the box. That
    correction is folded into the same 3x3 matrices the calibration
    already writes. One face gives roll and pitch; a second face that is
    not parallel to the first one (level, then on its side) also gives the
    yaw, which gravity alone can never see.

    python3 tools/inslib_calib_gui.py
    python3 tools/inslib_calib_gui.py --port COM4 -o data/config.yaml
    python3 tools/inslib_calib_gui.py --udp 29801   # alongside inslib_hub.py
    python3 tools/inslib_calib_gui.py --demo        # no board, synthetic IMU

--demo replaces the serial port with a generated UBX stream that acts out
a whole session (rest, then poses with rotations between them, with a
magnetometer whose hard iron, soft iron and mounting rotation are known).
It exists to exercise the window without hardware.

Dependencies: PyQt6, pyqtgraph, numpy, pyserial; PyYAML when merging into
an existing config. Styling follows python/inspostgui.py.

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import argparse
import datetime
import math
import os
import struct
import sys
import threading
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import inslib_imu_calib as calib     # noqa: E402  (recording, solve, writer)
import inslib_imu_tk as imu_tk         # noqa: E402
import inslib_frame_align as fa        # noqa: E402  (mounting rotations)
import inslib_mag_calib as mag_calib   # noqa: E402  (hard/soft iron)
from inslib_ubx import (CLASS_NAV, CLS_INSLIB, ID_IMU, ID_MAG,   # noqa: E402
                        ID_NAV_PVT, IMU_FMT, MAG_FMT, build_imu_status,
                        ubx_frame)
import inslib_ubx as ubx   # noqa: E402  (the configuration interface)

from PyQt6 import QtCore, QtGui, QtWidgets   # noqa: E402
import pyqtgraph as pg                       # noqa: E402

DEG = 180.0 / math.pi
TRACE_SEC = 6.0          # how much history the live traces keep
UI_HZ = 25               # live-view repaint rate
POSE_SCAN_SEC = 1.0      # how often the static-pose detector re-runs
CAPTURE_SEC = 2.5        # averaging window of one housing placement
PLACEMENT_TOL_DEG = 25.0  # how far a capture may sit from the face it was captured as
MAG_HIST = 3000          # mag samples kept for the live coverage sphere
READOUT_TAU_SEC = 0.4    # low pass on the |a| readouts (display only)
READOUT_TEMP_TAU_SEC = 5.0  # ... and on the die temperature, which moves slowly


# ============================================================================
# Look and feel (same palette as python/inspostgui.py)
# ============================================================================

class ReadoutFilter:
    """Time-constant low pass for one live readout. Display only.

    The magnitude readouts exist to answer one question each -- how far
    does |a| sit from ~9.81, and does the calibration hold it there -- and
    a trailing digit that flickers faster than it can be read hides
    exactly that. Smoothing is over wall-clock time rather than per
    sample, so it does not change with the IMU rate or with how the worker
    batches its snapshots.

    Nothing but a QLabel ever sees the output: the traces, the recording
    and the solver all keep the unsmoothed sample."""

    def __init__(self, tau_sec):
        self.tau = tau_sec
        self._v = None
        self._t = None

    def reset(self):
        self._v = self._t = None

    def __call__(self, x):
        if x is None or not math.isfinite(x):
            self.reset()
            return x
        now = time.monotonic()
        if self._v is None:
            self._v, self._t = float(x), now
            return self._v
        dt = now - self._t
        self._t = now
        if dt > 0.0:
            self._v += (1.0 - math.exp(-dt / self.tau)) * (float(x) - self._v)
        return self._v


STYLESHEET = (
    "QMainWindow, QWidget { background: #0f1216; color: #d8dee6; } "
    "QFrame#panel { background: #171b21; border: 1px solid #252c35; "
    "border-radius: 6px; } "
    "QFrame#card { background: #141920; border: 1px solid #2b3642; "
    "border-radius: 8px; } "
    "QGroupBox { border: 1px solid #252c35; border-radius: 6px; "
    "margin-top: 8px; padding-top: 12px; } "
    "QGroupBox::title { color: #8fa0b4; left: 8px; } "
    "QPushButton { background: #1f2832; border: 1px solid #2b3642; "
    "border-radius: 4px; padding: 6px 12px; color: #d8dee6; } "
    "QPushButton:hover { background: #283441; } "
    "QPushButton:disabled { color: #5a6470; } "
    "QPushButton#primary { background: #1d5c96; border-color: #2f7fc4; } "
    "QPushButton#primary:hover { background: #246cae; } "
    "QPushButton#primary:disabled { background: #1a2330; border-color: "
    "#2b3642; } "
    "QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit { "
    "background: #12151a; border: 1px solid #2b3642; padding: 3px; "
    "color: #d8dee6; } "
    "QProgressBar { background: #12151a; border: 1px solid #2b3642; "
    "border-radius: 4px; text-align: center; color: #d8dee6; } "
    "QProgressBar::chunk { background: #2f7fc4; border-radius: 3px; } "
    "QCheckBox::indicator { width: 13px; height: 13px; border: 1px solid "
    "#3a4654; border-radius: 3px; background: #12151a; } "
    "QCheckBox::indicator:checked { background: #54aaff; } "
    # Without these the tab labels keep the platform default colour,
    # which is black text on the dark pane.
    "QTabWidget::pane { background: #0f1216; border: 1px solid #252c35; "
    "border-radius: 6px; top: -1px; } "
    "QTabBar::tab { background: #171b21; color: #8fa0b4; "
    "border: 1px solid #252c35; border-bottom: none; "
    "border-top-left-radius: 4px; border-top-right-radius: 4px; "
    "padding: 5px 12px; margin-right: 2px; } "
    "QTabBar::tab:selected { background: #1f2832; color: #d8dee6; } "
    "QTabBar::tab:hover { background: #232c37; color: #d8dee6; }"
)

def mag_heading_deg(mag, acc):
    """Tilt compensated magnetic heading [deg, 0..360), or None.

    The same two steps ahrs.c takes (ahrs_leveling_from_acc, then
    ahrs_mag_detilt), so what is on screen is the heading the filter
    would derive from the same pair rather than a second opinion. It is
    MAGNETIC: no declination is applied, that is the filter's job.

    `acc` is the specific force in FRD, which points up at rest, so
    gravity is its negation."""
    if mag is None or acc is None:
        return None
    m = np.asarray(mag, dtype=float).ravel()
    a = np.asarray(acc, dtype=float).ravel()
    if m.shape != (3,) or a.shape != (3,):
        return None
    if not (np.isfinite(m).all() and np.isfinite(a).all()):
        return None
    if float(np.linalg.norm(a)) < 1e-6:
        return None
    gx, gy, gz = -a
    roll = math.atan2(gy, gz)
    pitch = math.atan2(-gx, math.hypot(gy, gz))
    cr, sr = math.cos(roll), math.sin(roll)
    ct, st = math.cos(pitch), math.sin(pitch)
    ty = cr * m[1] - sr * m[2]
    tz = sr * m[1] + cr * m[2]
    hx = ct * m[0] + st * tz
    hy = ty
    # A field that de-tilts to nothing horizontal has no heading in it,
    # which is what a sensor held along the field lines looks like.
    if math.hypot(hx, hy) < 1e-9:
        return None
    return math.degrees(math.atan2(-hy, hx)) % 360.0


MONO = "font-family: Consolas, 'DejaVu Sans Mono', monospace; font-size: 12px;"
DIM = "color: #8fa0b4;"

C_OK = "#5fd08a"
C_WARN = "#e0b44a"
C_BAD = "#e06060"
C_ACCENT = "#54aaff"
C_IDLE = "#5a6470"

PEN_X = pg.mkPen("#e06060", width=1)
PEN_Y = pg.mkPen("#5fd08a", width=1)
PEN_Z = pg.mkPen("#54aaff", width=1)


# ============================================================================
# Coverage sphere
# ============================================================================

class PoseSphere(QtWidgets.QWidget):
    """Which directions the session has already visited.

    The poses do not have to point anywhere in particular, but they do
    have to point in DIFFERENT directions: it is the spread of gravity
    over the unit sphere that makes the misalignment and scale terms
    observable. A count alone hides the failure where twenty poses all
    sit near the same attitude, so the directions are drawn instead. The
    magnetometer tab reuses it for the field directions, where a cloud
    confined to one plane is the same failure with the same cause."""

    AZ = math.radians(38.0)
    EL = math.radians(24.0)

    def __init__(self, parent=None, caption="%d poses covered",
                 colour=C_OK):
        super().__init__(parent)
        self.dirs = np.zeros((0, 3))    # collected, unit gravity in body frame
        self.live = None                # the direction right now
        self.caption = caption
        self.colour = colour
        self.setMinimumSize(250, 230)

    def set_poses(self, dirs):
        self.dirs = np.asarray(dirs, dtype=float).reshape(-1, 3)
        self.update()

    def set_live(self, v):
        n = float(np.linalg.norm(v))
        self.live = np.asarray(v, dtype=float) / n if n > 1e-6 else None
        self.update()

    def _project(self, p, scale, cx, cy):
        ca, sa = math.cos(self.AZ), math.sin(self.AZ)
        ce, se = math.cos(self.EL), math.sin(self.EL)
        fwd = ca * p[..., 0] + sa * p[..., 1]
        sx = -sa * p[..., 0] + ca * p[..., 1]
        sy = p[..., 2] * ce - fwd * se
        depth = fwd * ce + p[..., 2] * se
        return cx + sx * scale, cy - sy * scale, depth

    def paintEvent(self, _ev):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        cx, cy = self.width() / 2.0, self.height() / 2.0 + 6
        r = min(self.width(), self.height()) * 0.38

        # Wireframe globe: the outline plus a few parallels and meridians,
        # enough to read a direction off without drawing a solid ball.
        qp.setBrush(QtGui.QBrush(QtGui.QColor("#141920")))
        qp.setPen(QtGui.QPen(QtGui.QColor("#2b3642"), 1))
        qp.drawEllipse(QtCore.QPointF(cx, cy), r, r)
        u = np.linspace(0, 2 * math.pi, 73)
        for lat in (-60, -30, 0, 30, 60):
            a = math.radians(lat)
            p = np.stack([math.cos(a) * np.cos(u), math.cos(a) * np.sin(u),
                          np.full_like(u, math.sin(a))], axis=-1)
            self._stroke(qp, p, cx, cy, r, lat == 0)
        for lon in range(0, 180, 30):
            a = math.radians(lon)
            p = np.stack([np.cos(u) * math.cos(a), np.cos(u) * math.sin(a),
                          np.sin(u)], axis=-1)
            self._stroke(qp, p, cx, cy, r, False)

        # Collected poses: filled in front, hollow behind, so the back of
        # the sphere is still readable.
        if len(self.dirs):
            xs, ys, dp = self._project(self.dirs, r, cx, cy)
            for x, y, d in zip(xs, ys, dp):
                front = d >= 0
                qp.setPen(QtGui.QPen(QtGui.QColor(self.colour), 1))
                qp.setBrush(QtGui.QBrush(QtGui.QColor(self.colour))
                            if front else QtCore.Qt.BrushStyle.NoBrush)
                qp.drawEllipse(QtCore.QPointF(x, y), 4.0 if front else 3.0,
                               4.0 if front else 3.0)
        if self.live is not None:
            x, y, d = self._project(self.live[None, :], r, cx, cy)
            qp.setPen(QtGui.QPen(QtGui.QColor(C_ACCENT), 2))
            qp.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            qp.drawEllipse(QtCore.QPointF(x[0], y[0]), 7, 7)
            qp.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(x[0], y[0]))

        qp.setPen(QtGui.QPen(QtGui.QColor("#8fa0b4")))
        qp.setFont(QtGui.QFont("Consolas", 9))
        qp.drawText(QtCore.QRectF(0, self.height() - 18, self.width(), 16),
                    QtCore.Qt.AlignmentFlag.AlignCenter,
                    self.caption % len(self.dirs))
        qp.end()

    def _stroke(self, qp, pts, cx, cy, r, bold):
        xs, ys, dp = self._project(pts, r, cx, cy)
        col = QtGui.QColor("#37424f" if bold else "#232c37")
        qp.setPen(QtGui.QPen(col, 1))
        qp.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        for i in range(len(xs) - 1):
            if dp[i] < 0 and dp[i + 1] < 0:
                continue        # hidden hemisphere
            qp.drawLine(QtCore.QPointF(xs[i], ys[i]),
                        QtCore.QPointF(xs[i + 1], ys[i + 1]))


class StillnessMeter(QtWidgets.QWidget):
    """Two bars: current gyro / accel noise against the accept limits.

    A pose only counts once the unit is genuinely at rest, so this says
    up front whether the thing you just put down is going to be accepted
    rather than leaving it to be discovered at the end."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(52)
        self.setMaximumHeight(52)
        self.gyr = 0.0          # [deg/s] stddev, worst axis
        self.acc = 0.0          # [m/s^2] stddev, worst axis
        self.gyr_lim = calib.STILL_MAX_GYR_STD_DPS
        self.acc_lim = calib.STILL_MAX_ACC_STD_MPS2
        self.valid = False

    def set_values(self, gyr_dps, acc_mps2, valid=True):
        self.gyr, self.acc, self.valid = gyr_dps, acc_mps2, valid
        self.update()

    def still(self):
        return self.valid and self.gyr <= self.gyr_lim and self.acc <= self.acc_lim

    def paintEvent(self, _ev):
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        qp.setFont(QtGui.QFont("Consolas", 9))
        rows = (("gyro  %5.2f deg/s" % self.gyr, self.gyr, self.gyr_lim),
                ("accel %5.2f m/s2" % self.acc, self.acc, self.acc_lim))
        for i, (text, val, lim) in enumerate(rows):
            y = 4 + i * 24
            qp.setPen(QtGui.QPen(QtGui.QColor("#8fa0b4")))
            qp.drawText(QtCore.QRectF(0, y, 140, 18),
                        QtCore.Qt.AlignmentFlag.AlignVCenter, text)
            x0, w = 148, max(40, self.width() - 158)
            qp.setBrush(QtGui.QBrush(QtGui.QColor("#12151a")))
            qp.setPen(QtGui.QPen(QtGui.QColor("#2b3642")))
            qp.drawRoundedRect(QtCore.QRectF(x0, y + 3, w, 12), 3, 3)
            if self.valid:
                # Full bar = twice the limit, so "just over" still shows a
                # gap rather than pinning to the end.
                frac = min(val / (2.0 * lim), 1.0) if lim > 0 else 0.0
                col = C_OK if val <= lim else (C_WARN if val <= 2 * lim else C_BAD)
                qp.setBrush(QtGui.QBrush(QtGui.QColor(col)))
                qp.setPen(QtCore.Qt.PenStyle.NoPen)
                qp.drawRoundedRect(QtCore.QRectF(x0 + 1, y + 4,
                                                 max(2.0, (w - 2) * frac), 10), 2, 2)
            qp.setPen(QtGui.QPen(QtGui.QColor("#8fa0b4"), 1,
                                 QtCore.Qt.PenStyle.DashLine))
            qp.drawLine(QtCore.QPointF(x0 + w / 2.0, y + 1),
                        QtCore.QPointF(x0 + w / 2.0, y + 17))
        qp.end()


# ============================================================================
# Sources
# ============================================================================

class DemoSerial:
    """A board that is not there: acts out a whole calibration session.

    Only used by --demo. Rests, then repeatedly rotates to a random new
    attitude and holds it, with a bias and scale error built in so the
    solve has something to find.

    The magnetometer it streams carries the three errors the magnetometer
    calibration is meant to find: a hard iron, a soft iron, and a triad
    turned a few degrees against the IMU. The IMU itself is turned again
    inside the "housing", so the housing alignment has something to
    measure too (in the demo the only level placement available is
    whatever attitude the acted-out session happens to be in, so that part
    is exercised rather than verified).

    It also streams a fixed NAV-PVT position, so the GNSS button next to
    Position has something to fill in without a receiver attached."""

    RATE_HZ = 100.0
    MAG_DIV = 10                    # 10 Hz, as the firmware sends it
    NAV_DIV = 100                   # NAV-PVT once a second, a receiver's usual rate
    # Where the demo pretends to be: same point tools/README.md's WMM example
    # uses, so a session run with --latlon left blank fills in to a number
    # that is checkable against that example.
    LAT_DEG = 48.137
    LON_DEG = 11.575
    HEIGHT_M = 520.0                # ellipsoidal, so the gravity lookup has one
    _NAV_PVT_FMT = "<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6sihH"
    REST_SEC = 12.0
    HOLD_SEC = 3.5
    TURN_SEC = 1.5
    M_ACC = np.array([[1.011, -0.008, 0.005],
                      [0.0, 0.994, -0.011],
                      [0.0, 0.0, 1.007]])
    B_ACC = np.array([0.075, -0.052, 0.110])
    M_GYR = np.array([[1.021, -0.006, 0.009],
                      [0.004, 0.988, -0.007],
                      [-0.010, 0.003, 1.005]])
    B_GYR = np.radians([0.9, -0.4, 0.25])
    G = 9.80665
    # Local field: 48.5 uT at 64 deg dip, 3 deg declination, NED.
    FIELD_UT = 48.5
    DIP_DEG = 64.0
    DECL_DEG = 3.0
    A_MAG = np.array([[1.07, 0.03, -0.02],     # soft iron, symmetric
                      [0.03, 0.94, 0.02],
                      [-0.02, 0.02, 1.01]])
    B_MAG = np.array([9.0, -5.5, 13.0])        # hard iron [uT]
    RPY_MAG = (1.5, -2.5, 3.5)                 # mag triad vs. the IMU [deg]

    def __init__(self):
        self.t0 = time.monotonic()
        self.n = 0
        self.buf = bytearray()
        self.rng = np.random.default_rng(11)
        self.rot = np.eye(3)            # world -> body
        self.axis = np.array([1.0, 0.0, 0.0])
        self.rate = 0.0
        self._phase_end = self.REST_SEC
        self._turning = False
        self._inv_acc = np.linalg.inv(self.M_ACC)
        self._inv_gyr = np.linalg.inv(self.M_GYR)
        dip, dec = math.radians(self.DIP_DEG), math.radians(self.DECL_DEG)
        self._field_n = self.FIELD_UT * np.array(
            [math.cos(dip) * math.cos(dec), math.cos(dip) * math.sin(dec),
             math.sin(dip)])
        self._r_mag = fa.rodrigues(np.radians(self.RPY_MAG))
        self._inv_mag = np.linalg.inv(self.A_MAG)

    def _nav_pvt_payload(self):
        """A minimal but valid UBX-NAV-PVT payload: 3D fix, fixed position.

        Only the fields the calibration window actually reads (fixType,
        the gnssFixOk flag bit, numSV, lon, lat, height) carry real
        values; the
        rest of the 92-byte payload (tools/insrcv.c NAV_PVT_PAYLOAD_LEN)
        is zeroed, which pyubx2 decodes without complaint."""
        return struct.pack(
            self._NAV_PVT_FMT,
            0, 0, 0, 0, 0, 0, 0, 0,      # iTOW, date/time, valid
            0, 0,                        # tAcc, nano
            3, 0x01, 0, 14,              # fixType=3D, flags=gnssFixOk, flags2, numSV
            int(round(self.LON_DEG * 1e7)), int(round(self.LAT_DEG * 1e7)),
            int(round(self.HEIGHT_M * 1000.0)), 0,   # height (mm), hMSL
            0, 0,                        # hAcc, vAcc
            0, 0, 0, 0, 0,               # velN, velE, velD, gSpeed, headMot
            0, 0,                        # sAcc, headAcc
            0, b"\x00" * 6,               # pDOP, reserved1
            0, 0, 0)                     # headVeh, magDec, magAcc

    def _step(self, t):
        if t >= self._phase_end:
            self._turning = not self._turning
            self._phase_end = t + (self.TURN_SEC if self._turning
                                   else self.HOLD_SEC)
            if self._turning:
                a = self.rng.normal(size=3)
                self.axis = a / np.linalg.norm(a)
                self.total = self.rng.uniform(0.7, 2.8)
                self.turn_t0 = t
        if self._turning:
            frac = (t - self.turn_t0) / self.TURN_SEC
            w = (self.total * math.pi / self.TURN_SEC
                 * math.sin(math.pi * min(frac, 1.0)) / 2.0)
        else:
            w = 0.0
        omega = self.axis * w
        rot = self.rot
        # Specific force at rest is the reaction to gravity, so it points
        # UP: level and upright it reads (0, 0, -g), the convention ins.c
        # levels with.
        f_body = rot @ np.array([0.0, 0.0, -self.G])
        self.rot = fa.rodrigues(self.axis * (-w / self.RATE_HZ)) @ self.rot
        return f_body, omega, rot @ self._field_n

    def _generate(self):
        due = int((time.monotonic() - self.t0) * self.RATE_HZ)
        while self.n < due:
            t = self.n / self.RATE_HZ
            acc_true, gyr_true, mag_true = self._step(t)
            acc = (self._inv_acc @ acc_true + self.B_ACC
                   + self.rng.normal(0, 0.010, 3)) / self.G
            gyr = np.degrees(self._inv_gyr @ gyr_true + self.B_GYR
                             + self.rng.normal(0, math.radians(0.03), 3))
            payload = struct.pack(IMU_FMT, int(t * 1e6),
                                  acc[0], acc[1], acc[2],
                                  gyr[0], gyr[1], gyr[2],
                                  build_imu_status(
                                      26.5 + 0.4 * math.sin(self.n / 3000.0)),
                                  self.n & 0xFFFF)
            self.buf += ubx_frame(CLS_INSLIB, ID_IMU, payload)
            if self.n % self.MAG_DIV == 0:
                # Inverse of the model the solver fits: the sensor reads
                # its own axes, through the soft iron, offset by the hard
                # iron.
                mag = (self._inv_mag @ (self._r_mag.T @ mag_true) + self.B_MAG
                       + self.rng.normal(0, 0.15, 3))
                self.buf += ubx_frame(CLS_INSLIB, ID_MAG, struct.pack(
                    MAG_FMT, int(t * 1e6), mag[0], mag[1], mag[2], 25.0))
            if self.n % self.NAV_DIV == 0:
                self.buf += ubx_frame(CLASS_NAV, ID_NAV_PVT,
                                      self._nav_pvt_payload())
            self.n += 1

    @property
    def in_waiting(self):
        self._generate()
        return len(self.buf)

    def read(self, n):
        self._generate()
        if not self.buf:
            time.sleep(0.005)
            self._generate()
        out = bytes(self.buf[:n])
        del self.buf[:len(out)]
        return out

    def reset_input_buffer(self):
        self._generate()
        self.buf.clear()

    def close(self):
        pass


# ============================================================================
# Workers
# ============================================================================

class ImuWorker(QtCore.QThread):
    """Owns the port: streams continuously, records on request, and
    re-runs the static-pose detector on what it has so far."""

    sig_live = QtCore.pyqtSignal(dict)
    sig_poses = QtCore.pyqtSignal(dict)
    sig_capture = QtCore.pyqtSignal(dict)
    sig_error = QtCore.pyqtSignal(str)

    def __init__(self, source, parent=None):
        super().__init__(parent)
        self.source = source
        self.stream = calib.ImuStream(source)
        self.lock = threading.Lock()
        self.rec = None                 # calib.Recording while recording
        self.init_sec = calib.DEFAULT_INIT_SEC
        self.pose_sec = calib.DEFAULT_POSE_SEC
        self._stop = threading.Event()
        self._start_req = False
        self._live = calib.AxisStats(), calib.AxisStats()
        self._live_t0 = 0.0
        self._live_first_us = None
        self._live_last_us = None
        self._rate_hz = 0.0
        self._last_emit = 0.0
        self._last_scan = 0.0
        self._last = None
        self._last_mag = None
        self._mag_hist = []             # recent samples, for the live sphere
        self._n_mag = 0
        self._last_gnss = None          # calib.GnssFix, if the link carries one
        # One housing placement being averaged: (deadline, AxisStats pair,
        # accumulated accel). None when nothing is being captured.
        self._capture = None
        self.t_started = 0.0
        # Configuration replies from the board, waiting to be collected
        # by whoever sent the request (see cfg_send/cfg_take).
        self._cfg_rx = []
        # The newest state message, which is broadcast rather than
        # requested and therefore belongs to nobody in particular.
        self._board_info = None

    # ------------------------------------------------------------------
    # Configuration channel
    #
    # The board answers on the port the request arrived on, and this
    # thread owns reading it. Sending goes the other way and is done by
    # the caller's thread: a serial port is full duplex, and routing the
    # write through here as well would mean a queue and a wakeup for no
    # gain.
    # ------------------------------------------------------------------
    def can_configure(self):
        """Whether this source can carry a request at all.

        A UDP fan-out from the hub is one-way and the demo source is not
        a board, so the upload has to be offered only where it can work
        rather than failing once pressed."""
        return hasattr(self.source, "write") and not getattr(
            self.source, "datagram", False) and not isinstance(self.source, DemoSerial)

    def cfg_send(self, frame):
        self.source.write(frame)
        flush = getattr(self.source, "flush", None)
        if flush:
            flush()

    def cfg_take(self):
        """Every configuration reply seen since the last call."""
        with self.lock:
            out, self._cfg_rx = self._cfg_rx, []
        return out

    def cfg_clear(self):
        with self.lock:
            self._cfg_rx = []

    def stop(self):
        self._stop.set()

    def start_capture(self, seconds=CAPTURE_SEC):
        """Average the next `seconds` of accelerometer samples.

        The mean of a whole window rather than one live sample: a
        placement is worth 0.05 deg of tilt if it is averaged and rather
        less if it is not, and the tilt is the entire measurement here."""
        with self.lock:
            self._capture = {"until": time.monotonic() + seconds,
                             "acc": calib.AxisStats(),
                             "gyr": calib.AxisStats()}

    def start_recording(self, init_sec, pose_sec):
        with self.lock:
            self.init_sec, self.pose_sec = init_sec, pose_sec
            self._start_req = True

    def stop_recording(self):
        with self.lock:
            rec, self.rec = self.rec, None
        return rec

    def snapshot(self):
        with self.lock:
            return self.rec

    # ------------------------------------------------------------------
    def _note(self, s):
        g, a = self._live
        now = time.monotonic()
        if now - self._live_t0 > 0.5:
            if (self._live_first_us is not None and g.n > 1
                    and self._live_last_us > self._live_first_us):
                span = (self._live_last_us - self._live_first_us) / 1e6
                self._rate_hz = (g.n - 1) / span
            self._live = calib.AxisStats(), calib.AxisStats()
            self._live_t0 = now
            self._live_first_us = None
            g, a = self._live
        if self._live_first_us is None:
            self._live_first_us = s.t_us
        self._live_last_us = s.t_us
        g.add(s.gyr_rps)
        a.add(s.acc_mps2)
        self._last = s
        cap = self._capture
        if cap is not None:
            cap["acc"].add(s.acc_mps2)
            cap["gyr"].add(s.gyr_rps)

    def _note_mag(self, m):
        self._last_mag = m
        self._n_mag += 1
        self._mag_hist.append(m.mag_ut)
        if len(self._mag_hist) > MAG_HIST:
            del self._mag_hist[:len(self._mag_hist) - MAG_HIST]

    def _finish_capture(self):
        """Emit the averaged placement once its window is over."""
        cap = self._capture
        if cap is None or time.monotonic() < cap["until"]:
            return
        with self.lock:
            self._capture = None
        if cap["acc"].n < 10:
            self.sig_capture.emit({"ok": False, "reason": "no samples"})
            return
        self.sig_capture.emit({
            "ok": True,
            "acc": tuple(cap["acc"].mean()),
            "n": cap["acc"].n,
            "gyr_std_dps": max(cap["gyr"].std()) * DEG,
            "acc_std": max(cap["acc"].std()),
        })

    def _emit_live(self):
        now = time.monotonic()
        if now - self._last_emit < 1.0 / UI_HZ or self._last is None:
            return
        self._last_emit = now
        g, a = self._live
        rec = self.rec
        cap = self._capture
        self.sig_live.emit({
            "acc": self._last.acc_mps2,
            "gyr": self._last.gyr_rps,
            "temp": self._last.temp_c,
            "norm": math.sqrt(sum(v * v for v in self._last.acc_mps2)),
            "gyr_std_dps": (max(g.std()) * DEG) if g.n > 5 else 0.0,
            "acc_std": max(a.std()) if a.n > 5 else 0.0,
            "valid": g.n > 5,
            "rate_hz": self._rate_hz,
            "recording": rec is not None,
            "elapsed": (now - self.t_started) if rec is not None else 0.0,
            "samples": len(rec) if rec is not None else 0,
            "mag": self._last_mag.mag_ut if self._last_mag else None,
            "mag_n": self._n_mag,
            "mag_samples": len(rec.mag) if rec is not None else 0,
            "capturing": 0.0 if cap is None else max(0.0,
                                                     cap["until"] - now),
            "gnss": self._last_gnss,
        })

    def _mag_dirs(self):
        """Where the field has been pointing lately, as unit directions.

        The hard iron is not known yet while the session is running, so the
        cloud is centred on its own mean. That is a poor bias estimate and
        a perfectly good coverage display: what the sphere has to answer is
        whether the unit has been turned through all directions, and a
        mis-centred cloud still shows the empty parts."""
        if len(self._mag_hist) < 20:
            return np.zeros((0, 3))
        m = np.asarray(self._mag_hist, dtype=float)
        step = max(1, len(m) // 400)        # the sphere draws every point
        d = m[::step] - m.mean(axis=0)
        n = np.linalg.norm(d, axis=1, keepdims=True)
        return d[n[:, 0] > 1e-6] / n[n[:, 0] > 1e-6]

    def _scan_poses(self):
        """Static poses found so far, plus their gravity directions."""
        now = time.monotonic()
        if now - self._last_scan < POSE_SCAN_SEC:
            return
        self._last_scan = now
        self.sig_poses.emit({"mag_dirs": self._mag_dirs()})
        rec = self.rec
        if rec is None:
            return
        ivals, acc = rec.static_poses(self.init_sec, self.pose_sec)
        if len(acc) == 0:
            return
        dirs = np.array([acc[s:e + 1].mean(axis=0) for (s, e) in ivals]) \
            if ivals else np.zeros((0, 3))
        if len(dirs):
            dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        self.sig_poses.emit({"count": len(ivals), "dirs": dirs})

    def _note_cfginfo(self, frames):
        """Keep the newest 0x40/0x07 the board sent.

        It arrives once a second whether or not anyone asked, and it is
        the only way to learn what the BOARD is doing to the stream --
        whether it applies its own calibration, and whether it carries a
        housing rotation. Without it the window can only see what
        config.yaml says, which is a statement about the file and not
        about the samples arriving."""
        for mid, payload in frames:
            if mid != ubx.ID_CFGINFO:
                continue
            info = ubx.parse_cfginfo(payload)
            if info is not None:
                with self.lock:
                    self._board_info = info

    def board_info(self):
        """The board's last state message, or None if it sends none."""
        with self.lock:
            return self._board_info

    def run(self):
        try:
            while not self._stop.is_set():
                with self.lock:
                    if self._start_req:
                        self._start_req = False
                        self.rec = calib.Recording()
                        self.t_started = time.monotonic()
                for s in self.stream.read_samples():
                    self._note(s)
                    if self.rec is not None:
                        self.rec.add(s)
                for m in self.stream.read_mag():
                    self._note_mag(m)
                    if self.rec is not None:
                        self.rec.add_mag(m)
                cfg = self.stream.read_cfg()
                if cfg:
                    self._note_cfginfo(cfg)
                    with self.lock:
                        self._cfg_rx += cfg
                for fix in self.stream.read_gnss():
                    self._last_gnss = fix
                self._finish_capture()
                self._emit_live()
                self._scan_poses()
        except Exception:
            self.sig_error.emit(traceback.format_exc())


class SolveWorker(QtCore.QThread):
    """The fit takes a second or two; it does not belong on the UI thread."""

    sig_done = QtCore.pyqtSignal(object, object)
    sig_log = QtCore.pyqtSignal(str)
    sig_error = QtCore.pyqtSignal(str)

    def __init__(self, rec, gravity, init_sec, misalignment=True,
                 with_mag=True, mag_field_ut=0.0, mag_field_source="",
                 parent=None):
        super().__init__(parent)
        self.rec = rec
        self.gravity = gravity
        self.init_sec = init_sec
        self.misalignment = misalignment
        self.with_mag = with_mag
        self.mag_field_ut = mag_field_ut
        self.mag_field_source = mag_field_source

    def run(self):
        try:
            cal, magcal = calib.solve_session(
                self.rec, self.gravity, self.init_sec,
                estimate_misalignment=self.misalignment,
                with_mag=self.with_mag, field_ut=self.mag_field_ut,
                field_source=self.mag_field_source, log=self.sig_log.emit)
        except Exception as e:
            self.sig_error.emit(str(e))
            return
        self.sig_done.emit(cal, magcal)


# ============================================================================
# Main window
# ============================================================================

# Two nodes closer together than this are the same operating point
# measured twice. The new measurement replaces the old one rather than
# crowding the table with a pair the interpolation cannot tell apart.
NODE_MERGE_TOL_C = 1.0

# Records per CFG-VALSET. A node is 58 bytes on the wire and the board
# accepts about 500, so eight fit with room to spare.
NODES_PER_FRAME = 8

CAL_GROUP_OF = {"acc": ubx.GRP_CAL_ACC, "gyr": ubx.GRP_CAL_GYR,
                "mag": ubx.GRP_CAL_MAG}
CAL_LABEL = {"acc": "accelerometer", "gyr": "gyroscope",
             "mag": "magnetometer"}

# NAV-PVT fixType, u-blox: 0 none, 1 DR only, 2 2D, 3 3D, 4 GNSS+DR, 5 time
# only (same table tools/inslib_hub.py's status line uses).
FIX_TYPE_TEXT = {0: "NoFix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}


class BoardRequest(QtCore.QThread):
    """A configuration exchange with the board, off the GUI thread.

    The reader thread owns the port and collects the replies (cfg_take),
    so everything that asks the board something needs the same small
    request/response loop. It lives here once rather than in every
    worker that wants to write one key."""

    sig_log = QtCore.pyqtSignal(str)
    sig_done = QtCore.pyqtSignal(bool, str)

    REQUEST_TIMEOUT = 1.5
    # A persisted write can hit a flash erase, and the board stops for a
    # few hundred milliseconds while that runs.
    SAVE_TIMEOUT = 4.0

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self._buf = []

    # --- request/response over the worker's port ---
    def _pop(self, ids):
        """A reply of one of these ids, or (None, None) on timeout.

        Replies that arrived in the same batch as the wanted one stay
        buffered: a multi-frame answer comes back to back and dropping
        the tail would lose most of the table."""
        deadline = time.monotonic() + self.REQUEST_TIMEOUT
        while True:
            for i, (mid, payload) in enumerate(self._buf):
                if mid in ids:
                    del self._buf[:i + 1]
                    return mid, payload
            self._buf.clear()
            if time.monotonic() >= deadline:
                return None, None
            self._buf += self.worker.cfg_take()
            if not self._buf:
                self.msleep(20)

    def _request(self, frame, ids, timeout=None):
        self._buf = []
        self.worker.cfg_clear()
        self.worker.cfg_send(frame)
        saved, self.REQUEST_TIMEOUT = self.REQUEST_TIMEOUT, timeout or self.REQUEST_TIMEOUT
        try:
            return self._pop(ids)
        finally:
            self.REQUEST_TIMEOUT = saved

    def _valget(self, keys):
        mid, payload = self._request(ubx.build_valget(keys),
                                     (ubx.ID_VALGET_R, ubx.ID_CFGACK))
        if mid is None:
            raise RuntimeError("the board did not answer. Is this the sensor "
                               "board and not a bare receiver?")
        if mid == ubx.ID_CFGACK:
            ack = ubx.parse_cfgack(payload)
            raise RuntimeError("the board refused the request: %s"
                               % (ack["text"] if ack else "malformed answer"))
        values = {}
        while True:
            got = ubx.parse_valget(payload)
            if got is None:
                raise RuntimeError("malformed value response")
            values.update(got["values"])
            if not got["more"]:
                return values
            mid, payload = self._pop((ubx.ID_VALGET_R,))
            if mid is None:
                raise RuntimeError("the answer stopped part way through")

    def _valset(self, items, layers, timeout=None):
        mid, payload = self._request(ubx.build_valset(items, layers),
                                     (ubx.ID_CFGACK,), timeout)
        if mid is None:
            raise RuntimeError("no acknowledgement from the board")
        ack = ubx.parse_cfgack(payload)
        if ack is None:
            raise RuntimeError("malformed acknowledgement")
        if not ack["ok"]:
            where = ("" if ack["key"] == 0
                     else " at %s" % ubx.cfg_key_name(ack["key"]))
            raise RuntimeError("the board refused the write%s: %s"
                               % (where, ack["text"]))
        return ack


class UploadWorker(BoardRequest):
    """Writes this session's calibration into the board as ONE
    temperature node per sensor.

    The board keeps a table of nodes and interpolates between them, so a
    session does not replace what is stored: it contributes the operating
    point it was measured at. The table is read back first, this
    session's node is merged into it in temperature order, and only what
    actually changed is written.

    Two things are deliberate. A node within NODE_MERGE_TOL_C of an
    existing one REPLACES it, because the same operating point measured
    twice is a better measurement and not a second point. And the bias
    polynomial is retired on every upload, since the fit that was made
    over the previous nodes no longer describes the table and would go on
    overriding the node biases if it were left in place.

    The housing rotation does NOT go into the nodes. It is one key of its
    own (CFG-FRAME-HOUSING) that the board premultiplies, which is what
    lets a node measured next winter go up without a housing measurement
    beside it -- and keeps a rotation out of a quantity that gets
    interpolated over temperature.

    The node counts are written last and in one frame, which is what
    makes a new calibration take effect in a single step rather than node
    by node (inslib_protocol.md)."""

    def __init__(self, worker, groups, persist, housing=None, parent=None):
        """groups: {kind: (matrix_rowmajor, bias_stream_units, temp_c)}

        `housing` is the mounting rotation as a row-major 3x3, or None to
        leave whatever the board has alone. It travels as its own key
        rather than folded into the nodes, so it can be measured and
        rewritten without touching the temperature table."""
        super().__init__(worker, parent)
        self.groups = groups
        self.persist = persist
        self.housing = housing

    # --- the table ---
    def _read_nodes(self, group):
        """The live node records of one group, in index order."""
        values = self._valget([ubx.group_wildcard(group)])
        n_raw = values.get(ubx.cal_npts_key(group))
        n = ubx.scalar_from_bytes(n_raw) if n_raw else 0
        nodes = []
        for i in range(min(n, ubx.CAL_PTS_MAX)):
            raw = values.get(ubx.cal_point_key(group, i))
            if raw is None or len(raw) != ubx.CAL_POINT_LEN:
                # The count promises a node that is not there. The board
                # rejects such a table as a whole, so there is nothing to
                # preserve and starting over is the honest repair.
                self.sig_log.emit("node %d is missing on the board, the "
                                  "stored table was incomplete" % i)
                return []
            nodes.append(raw)
        return nodes

    @staticmethod
    def _merge(nodes, record, temp_c):
        """(new node list, description of what happened)."""
        temps = [ubx.unpack_cal_point(r)[0] for r in nodes]
        for i, t in enumerate(temps):
            if abs(t - temp_c) <= NODE_MERGE_TOL_C:
                out = list(nodes)
                out[i] = record
                return out, "replaced node %d (was %.1f degC)" % (i, t)
        pos = len([t for t in temps if t < temp_c])
        out = list(nodes)
        out.insert(pos, record)
        if len(out) > ubx.CAL_PTS_MAX:
            raise RuntimeError(
                "the board already holds %d nodes, which is the maximum. "
                "Clear the table first (inslib_cfg.py reset --flash)."
                % ubx.CAL_PTS_MAX)
        return out, "inserted as node %d of %d" % (pos, len(out))

    def run(self):
        try:
            counts = []
            for kind in ("acc", "gyr", "mag"):
                if kind not in self.groups:
                    continue
                matrix, bias, temp_c = self.groups[kind]
                group = CAL_GROUP_OF[kind]
                record = ubx.pack_cal_point(temp_c, matrix, bias)

                old = self._read_nodes(group)
                new, what = self._merge(old, record, temp_c)
                self.sig_log.emit("%s at %.1f degC: %s"
                                  % (CAL_LABEL[kind], temp_c, what))

                changed = [(ubx.cal_point_key(group, i), r)
                           for i, r in enumerate(new)
                           if i >= len(old) or old[i] != r]
                for i in range(0, len(changed), NODES_PER_FRAME):
                    self._valset(changed[i:i + NODES_PER_FRAME], ubx.LAYER_RAM)
                # The previous fit was made over the previous nodes.
                self._valset([(ubx.cal_biaspoly_key(group), ubx.no_bias_poly())],
                             ubx.LAYER_RAM)
                counts.append((ubx.cal_npts_key(group), len(new)))

            # The rotation rides in the same frame as the node counts,
            # so a table and the frame it is expressed in take effect
            # together rather than one sample apart.
            last = list(counts)
            if self.housing is not None:
                last.append((ubx.housing_key(),
                             ubx.pack_housing(self.housing)))
                rpy = ubx.housing_rpy_deg(self.housing)
                self.sig_log.emit("housing rotation: roll %+.2f, pitch "
                                  "%+.2f, yaw %+.2f deg" % tuple(rpy))
            if not last:
                self.sig_done.emit(False, "nothing to upload")
                return

            layers = ubx.LAYER_RAM | (ubx.LAYER_FLASH if self.persist else 0)
            self._valset(last, layers, timeout=self.SAVE_TIMEOUT)

            parts = ["%s %d nodes" % (CAL_LABEL[k], n)
                     for (_key, n), k in
                     zip(counts, [k for k in ("acc", "gyr", "mag")
                                  if k in self.groups])]
            if self.housing is not None:
                parts.append("the housing rotation")
            summary = ", ".join(parts)
            self.sig_done.emit(
                True,
                "The board now holds %s.\n\n%s" %
                (summary,
                 "Stored, so it survives a power cycle."
                 if self.persist else
                 "Live only. Without persisting it is gone at the next "
                 "power cycle."))
        except Exception as e:                        # noqa: BLE001
            self.sig_done.emit(False, str(e))


class ApplyCalWorker(BoardRequest):
    """Clears the APPLY_CAL switches it is given, so the board stops
    correcting those streams.

    The live layer only. A calibration recording wants the raw sensor,
    and so does a housing capture that is meant to be an absolute
    measurement rather than what is left over of an earlier one, but
    neither is a reason to change what the unit comes up as.

    Which switches go off is the caller's, because the two are separate
    keys on the board and mean different things here: a housing capture
    reads the accelerometer and has no business turning the magnetometer
    off, while a recording that solves a magnetometer calibration has to
    have both."""

    def __init__(self, worker, keys, parent=None):
        super().__init__(worker, parent)
        self.keys = list(keys)

    def run(self):
        try:
            self._valset([(ubx.CFG_KEYS[k], 0) for k in self.keys],
                         ubx.LAYER_RAM)
            self.sig_done.emit(True, "")
        except Exception as e:                        # noqa: BLE001
            self.sig_done.emit(False, str(e))


class CalibWindow(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("inslib calib — IMU, magnetometer and housing")
        self.resize(1420, 900)
        self.setStyleSheet(STYLESHEET)
        pg.setConfigOptions(antialias=True)

        self.args = args
        self.worker = None
        self.solver = None
        self.uploader = None
        self.source = None
        # Magnetometer die temperature over the last recording. The board
        # looks the magnetometer calibration up by that and not by the
        # IMU's, so a node written against the wrong thermometer is a
        # node at the wrong temperature.
        self.mag_temp_c = None
        self.out_path = os.path.abspath(args.out)
        self.existing = None
        self.cal = None
        self.magcal = None
        # Housing placements captured so far, as (placement key, mean RAW
        # accel). Raw on purpose: the solve applies whatever IMU
        # calibration is current, so a capture stays valid when the
        # calibration under it changes.
        self.captures = []
        self.housing = None
        self._pending_capture = None
        self.pose_count = 0
        self.trace_t = []
        self.trace_acc = [[], [], []]
        self.trace_gyr = [[], [], []]
        self.trace_anorm_raw = []
        self.trace_anorm_cal = []
        self.trace_mag = [[], [], []]
        self.trace_mnorm_raw = []
        self.trace_mnorm_cal = []
        # Readout smoothing only, see ReadoutFilter.
        self.f_anorm = ReadoutFilter(READOUT_TAU_SEC)
        self.f_anorm_cal = ReadoutFilter(READOUT_TAU_SEC)
        self.f_wnorm = ReadoutFilter(READOUT_TAU_SEC)
        self.f_wnorm_cal = ReadoutFilter(READOUT_TAU_SEC)
        self.f_gyr = [ReadoutFilter(READOUT_TAU_SEC) for _ in range(3)]
        self.f_mnorm = ReadoutFilter(READOUT_TAU_SEC)
        self.f_mnorm_cal = ReadoutFilter(READOUT_TAU_SEC)
        self.f_temp = ReadoutFilter(READOUT_TEMP_TAU_SEC)
        # (acc_M, acc_bias, gyr_M, gyr_bias) once one is known. The
        # housing rotation is NOT folded in here: it is applied on top in
        # _apply_live_cal, so re-solving it never compounds.
        self.live_cal = None
        self._field_source = ""
        self._wmm_dip = None
        self._gnss_fix = None           # calib.GnssFix, the latest 3D fix seen
        self._gnss_autofilled = False   # only offer to overwrite a typed value once
        self._board_said = False        # the board's state is reported once
        self.live_cal_src = ""          # where live_cal came from, in words
        self.applyoff = None            # ApplyCalWorker while one is running
        self._record_after_off = False  # start recording once it is clear
        # Share of the last recording that arrived already corrected, out
        # of the per sample bit rather than out of the switch: the switch
        # says what the board was told, the bit says what reached the
        # sample. NaN before a recording has been solved.
        self.rec_cal_frac = float("nan")
        self._hint_state = None         # what the housing panel last said
        self._t0 = time.monotonic()

        self._build_ui()
        self._refresh_place_combo()
        self._refresh_house_steps()
        self._refresh_housing_hints(force=True)
        self._set_connected(False)
        if args.latlon:
            self._on_latlon()
        if args.port or args.udp or args.demo:
            QtCore.QTimer.singleShot(200, self.on_connect)

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)
        root.addWidget(self._build_topbar())

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(8)
        body.addWidget(self._build_settings_panel(), 0)
        body.addWidget(self._build_center_panel(), 3)
        body.addWidget(self._build_plots_panel(), 2)
        root.addLayout(body, 1)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(118)
        self.log.setStyleSheet(MONO)
        root.addWidget(self.log)

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._tick_traces)
        self.ui_timer.start(int(1000 / UI_HZ))

    def _build_topbar(self):
        bar = QtWidgets.QFrame()
        bar.setObjectName("panel")
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(10, 8, 10, 8)

        h.addWidget(QtWidgets.QLabel("Source"))
        self.cb_source = QtWidgets.QComboBox()
        self.cb_source.addItems(["Serial port", "Hub fan-out (UDP)"])
        self.cb_source.setToolTip(
            "inslib_hub.py owns the serial port while it runs. Point it at "
            "a second fan-out destination and read that instead, and the "
            "capture plus insrcv keep running through the calibration:\n"
            "  inslib_hub.py COM4 --fanout 127.0.0.1:29800,127.0.0.1:29801")
        self.cb_source.currentIndexChanged.connect(
            lambda i: self.src_stack.setCurrentIndex(i))
        h.addWidget(self.cb_source)

        self.src_stack = QtWidgets.QStackedWidget()
        self.src_stack.setMaximumHeight(32)
        w_serial = QtWidgets.QWidget()
        hs = QtWidgets.QHBoxLayout(w_serial)
        hs.setContentsMargins(0, 0, 0, 0)
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setMinimumWidth(140)
        self.cb_port.setEditable(True)
        hs.addWidget(self.cb_port)
        self.btn_rescan = QtWidgets.QPushButton("Rescan")
        self.btn_rescan.clicked.connect(self._scan_ports)
        hs.addWidget(self.btn_rescan)
        hs.addWidget(QtWidgets.QLabel("Baud"))
        self.sp_baud = QtWidgets.QSpinBox()
        self.sp_baud.setRange(9600, 4000000)
        self.sp_baud.setSingleStep(9600)
        self.sp_baud.setValue(self.args.baud)
        hs.addWidget(self.sp_baud)
        self.src_stack.addWidget(w_serial)

        w_udp = QtWidgets.QWidget()
        hu = QtWidgets.QHBoxLayout(w_udp)
        hu.setContentsMargins(0, 0, 0, 0)
        hu.addWidget(QtWidgets.QLabel("Listen on"))
        self.ed_udp = QtWidgets.QLineEdit(self.args.udp or "29801")
        self.ed_udp.setMaximumWidth(150)
        self.ed_udp.setToolTip("PORT, or HOST:PORT to bind one interface")
        hu.addWidget(self.ed_udp)
        hu.addStretch(1)
        self.src_stack.addWidget(w_udp)
        h.addWidget(self.src_stack)
        if self.args.udp:
            self.cb_source.setCurrentIndex(1)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.setObjectName("primary")
        self.btn_connect.clicked.connect(self.on_connect)
        h.addWidget(self.btn_connect)
        self.lbl_link = QtWidgets.QLabel("not connected")
        self.lbl_link.setStyleSheet(MONO + " color: %s;" % C_IDLE)
        h.addWidget(self.lbl_link)
        h.addStretch(1)

        h.addWidget(QtWidgets.QLabel("Output"))
        self.ed_out = QtWidgets.QLineEdit(self.out_path)
        self.ed_out.setMinimumWidth(280)
        self.ed_out.editingFinished.connect(self._on_out_changed)
        h.addWidget(self.ed_out)
        btn_browse = QtWidgets.QPushButton("...")
        btn_browse.setMaximumWidth(36)
        btn_browse.clicked.connect(self._pick_out)
        h.addWidget(btn_browse)

        self._scan_ports()
        if self.args.port:
            self.cb_port.setCurrentText(self.args.port)
        return bar

    def _build_settings_panel(self):
        # A scroll area, because the housing alignment grows a row per
        # placement and the panel must not push the sphere off the window.
        outer = QtWidgets.QScrollArea()
        outer.setWidgetResizable(True)
        outer.setFixedWidth(300)
        outer.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        panel = QtWidgets.QFrame()
        panel.setObjectName("panel")
        outer.setWidget(panel)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(10, 10, 10, 10)

        how = QtWidgets.QLabel(
            "<b>How</b><br>"
            "1. Put the unit down and leave it alone for the initial rest "
            "period.<br>"
            "2. Then: pick it up, turn it to <i>any</i> new attitude, set it "
            "down, hold.<br>"
            "3. Repeat until the sphere is well covered.<br><br>"
            "No fixture, no level surface, no known angles.")
        how.setWordWrap(True)
        how.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(how)

        grp = QtWidgets.QGroupBox("Session")
        f = QtWidgets.QFormLayout(grp)
        f.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        self.sp_init = QtWidgets.QDoubleSpinBox()
        self.sp_init.setRange(5.0, 300.0)
        self.sp_init.setValue(self.args.init_sec)
        self.sp_init.setSuffix(" s")
        self.sp_init.setToolTip("Initial undisturbed rest period: sets the "
                                "static threshold and the gyro bias")
        f.addRow("Rest", self.sp_init)
        self.sp_pose = QtWidgets.QDoubleSpinBox()
        self.sp_pose.setRange(1.0, 30.0)
        self.sp_pose.setValue(self.args.pose_sec)
        self.sp_pose.setSuffix(" s")
        f.addRow("Hold", self.sp_pose)
        self.sp_gravity = QtWidgets.QDoubleSpinBox()
        self.sp_gravity.setRange(9.7, 9.9)
        self.sp_gravity.setDecimals(5)
        self.sp_gravity.setSingleStep(0.001)
        self.sp_gravity.setValue(self.args.gravity)
        self.sp_gravity.setToolTip(
            "Local gravity; maps 1:1 into the accel scale factors.\nFilled "
            "in from Position below (WGS84 normal gravity, the same model\n"
            "the filter uses), so it only has to be typed when there is no "
            "position.")
        f.addRow("Gravity", self.sp_gravity)
        self.chk_misalign = QtWidgets.QCheckBox("Estimate misalignment")
        # Off unless asked for. The off-diagonal terms are the weakly
        # observable ones, so on a session that is not deliberately set
        # up for them they absorb scale and bias error rather than
        # measure anything, and the result is worse than leaving them
        # at identity.
        self.chk_misalign.setChecked(self.args.misalignment)
        self.chk_misalign.setToolTip(
            "Off: fit scale and bias only, leave the axis misalignment at "
            "identity.\nThe off-diagonal terms are the weakly observable "
            "ones -- with few or\npoorly spread poses they absorb scale and "
            "bias error instead of\nmeasuring anything. The before/after "
            "table says whether they earned\ntheir place in this session.")
        f.addRow(self.chk_misalign)
        v.addWidget(grp)

        self.lbl_poses = QtWidgets.QLabel("0")
        self.lbl_poses.setStyleSheet(
            "font-size: 40px; font-weight: bold; color: %s;" % C_IDLE)
        self.lbl_poses.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        v.addWidget(self.lbl_poses)
        cap = QtWidgets.QLabel("static poses accepted")
        cap.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        cap.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(cap)
        self.lbl_need = QtWidgets.QLabel("at least %d needed, 20+ is better"
                                         % calib.MIN_POSITIONS)
        self.lbl_need.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_need.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(self.lbl_need)

        v.addWidget(self._build_mag_group())
        v.addWidget(self._build_housing_group())
        v.addStretch(1)
        return outer

    def _build_mag_group(self):
        grp = QtWidgets.QGroupBox("Magnetometer")
        v = QtWidgets.QVBoxLayout(grp)
        self.chk_mag = QtWidgets.QCheckBox("Calibrate if present")
        self.chk_mag.setChecked(not self.args.no_mag)
        self.chk_mag.setToolTip(
            "Solve hard iron, soft iron and the rotation onto the IMU from "
            "the same session,\nand write the mag: keys. The turns between "
            "the poses are what covers the\nfield sphere, so nothing has to "
            "be done differently for it.")
        v.addWidget(self.chk_mag)
        f = QtWidgets.QFormLayout()
        f.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        self.sp_field = QtWidgets.QDoubleSpinBox()
        self.sp_field.setRange(0.0, 100.0)
        self.sp_field.setDecimals(1)
        self.sp_field.setSingleStep(0.5)
        self.sp_field.setValue(self.args.mag_field_ut)
        self.sp_field.setSuffix(" uT")
        self.sp_field.valueChanged.connect(lambda _v: self._sync_field_line())
        self.sp_field.setToolTip(
            "Local field strength to scale the calibration to. 0 keeps "
            "whatever the\nsensor measures, which points the same way but "
            "may not agree with the\nfield strength the filter gates the "
            "magnetometer on.")
        f.addRow("Field", self.sp_field)
        pos_row = QtWidgets.QHBoxLayout()
        pos_row.setContentsMargins(0, 0, 0, 0)
        self.ed_latlon = QtWidgets.QLineEdit(self.args.latlon)
        self.ed_latlon.setPlaceholderText("lat, lon[, height m]")
        self.ed_latlon.setToolTip(
            "Where this is being calibrated. Fills in BOTH reference "
            "magnitudes the\nfit is scaled to, each from INSLIB's own model "
            "and the same ones the\nfilter uses: the field strength from the "
            "WMM, and Gravity above from\nWGS84 normal gravity.\n"
            "The third number is ELLIPSOIDAL height in metres and may be left "
            "off;\nwithout it gravity comes out for sea level, which is "
            "3.1e-6 m/s2 per\nmetre too large.")
        self.ed_latlon.editingFinished.connect(self._on_latlon)
        pos_row.addWidget(self.ed_latlon, 1)
        self.btn_gnss = QtWidgets.QPushButton("GNSS")
        self.btn_gnss.setMaximumWidth(52)
        self.btn_gnss.setEnabled(False)
        self.btn_gnss.setToolTip(
            "Fill the position in from the receiver's own NAV-PVT fix, if "
            "one is\narriving on this link (tools/inslib_hub.py passes the "
            "GNSS receiver's\noutput through alongside the IMU frames). "
            "Needs a 3D fix.")
        self.btn_gnss.clicked.connect(self._use_gnss_fix)
        pos_row.addWidget(self.btn_gnss)
        f.addRow("Position", pos_row)
        v.addLayout(f)
        self.lbl_mag = QtWidgets.QLabel("no magnetometer frames yet")
        self.lbl_mag.setWordWrap(True)
        self.lbl_mag.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(self.lbl_mag)
        return grp

    def _build_housing_group(self):
        grp = QtWidgets.QGroupBox("Housing alignment")
        v = QtWidgets.QVBoxLayout(grp)
        # What each placement buys is on the step list below, so this says
        # only what the measurement IS. The panel has to stay short: it is
        # the last group in a scrolling column.
        how = QtWidgets.QLabel(
            "On a level surface: any tilt is the IMU sitting crooked in "
            "the box.")
        how.setWordWrap(True)
        how.setStyleSheet(DIM + " font-size: 11px;")
        how.setToolTip(
            "The correction is folded into the same 3x3 the scale factors "
            "live in, for the\naccelerometer, the gyroscope and the "
            "magnetometer alike.\n\n"
            "Gravity cannot see a rotation about itself, so one placement "
            "can only ever\ngive roll and pitch. A second face that is not "
            "parallel to the first pins the\nthird axis as well (level, "
            "then on its side). Two OPPOSITE faces, left and\nright, are "
            "parallel and do not.\n\n"
            "The faces are assumed square to one another. A housing where "
            "they are not\nshows up as a residual per placement, which "
            "nothing here can tell apart\nfrom a surface that was not level.")
        v.addWidget(how)

        # What the tilt is being measured THROUGH, which decides whether
        # the answer is the mounting error or something else entirely.
        self.lbl_house_src = QtWidgets.QLabel("")
        self.lbl_house_src.setWordWrap(True)
        self.lbl_house_src.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(self.lbl_house_src)

        # Shown only while the board corrects its own stream: then this
        # measures the leftover of a rotation that the upload replaces.
        self.lbl_house_warn = QtWidgets.QLabel("")
        self.lbl_house_warn.setWordWrap(True)
        self.lbl_house_warn.setStyleSheet(
            "color: %s; font-size: 11px; font-weight: bold;" % C_BAD)
        self.lbl_house_warn.setVisible(False)
        v.addWidget(self.lbl_house_warn)
        self.btn_applycal_off = QtWidgets.QPushButton(
            "Turn CFG-IMU-APPLY_CAL off (live)")
        self.btn_applycal_off.setToolTip(
            "Clears the switch in the live layer, so the board streams the "
            "raw sensor\nand this measures the mounting error itself. The "
            "stored image is untouched:\nthe unit comes up correcting "
            "again at the next power cycle.")
        self.btn_applycal_off.clicked.connect(self.on_applycal_off)
        self.btn_applycal_off.setVisible(False)
        v.addWidget(self.btn_applycal_off)

        ask = QtWidgets.QLabel("Which face is resting on the surface now?")
        ask.setWordWrap(True)
        ask.setStyleSheet("font-size: 11px; font-weight: bold;")
        v.addWidget(ask)
        self.cb_place = QtWidgets.QComboBox()
        for key, label, _ideal in fa.PLACEMENTS:
            self.cb_place.addItem(label, key)
        self.cb_place.setToolTip(
            "The face the unit is lying on RIGHT NOW, one capture at a "
            "time -- not a\nlist of what to measure. A capture filed under "
            "the wrong face is rejected\nby comparing it against gravity.")
        v.addWidget(self.cb_place)
        row = QtWidgets.QHBoxLayout()
        self.btn_capture = QtWidgets.QPushButton("Capture")
        self.btn_capture.clicked.connect(self.on_capture)
        row.addWidget(self.btn_capture)
        self.btn_clear_housing = QtWidgets.QPushButton("Clear")
        self.btn_clear_housing.clicked.connect(self.on_clear_housing)
        row.addWidget(self.btn_clear_housing)
        v.addLayout(row)

        # The outcome of the last capture, which is otherwise only in the
        # log -- and nobody reads a log while handling the unit.
        self.lbl_house_msg = QtWidgets.QLabel("")
        self.lbl_house_msg.setWordWrap(True)
        self.lbl_house_msg.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(self.lbl_house_msg)

        # Two steps and what each one buys, so the question "how many do
        # I need" is answered without opening a tooltip.
        self.lbl_house_steps = QtWidgets.QLabel("")
        self.lbl_house_steps.setStyleSheet(MONO + " font-size: 11px;")
        v.addWidget(self.lbl_house_steps)

        self.lst_place = QtWidgets.QListWidget()
        self.lst_place.setMaximumHeight(92)
        self.lst_place.setStyleSheet(MONO)
        v.addWidget(self.lst_place)
        self.lbl_housing = QtWidgets.QLabel("nothing captured")
        self.lbl_housing.setWordWrap(True)
        self.lbl_housing.setStyleSheet(DIM + " font-size: 11px;")
        v.addWidget(self.lbl_housing)
        return grp

    def _build_center_panel(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName("panel")
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(9)

        self.lbl_step = QtWidgets.QLabel("Connect to the unit to start")
        self.lbl_step.setStyleSheet("font-size: 17px; font-weight: bold;")
        v.addWidget(self.lbl_step)
        self.lbl_instr = QtWidgets.QLabel(
            "The poses may point anywhere; what matters is that they point "
            "in different directions and that the unit is really at rest.")
        self.lbl_instr.setWordWrap(True)
        self.lbl_instr.setStyleSheet(DIM)
        self.lbl_instr.setMinimumHeight(34)
        v.addWidget(self.lbl_instr)

        card = QtWidgets.QFrame()
        card.setObjectName("card")
        ch = QtWidgets.QHBoxLayout(card)
        ch.setContentsMargins(10, 10, 10, 10)
        self.sphere = PoseSphere()
        ch.addWidget(self.sphere, 1)
        readout = QtWidgets.QVBoxLayout()
        readout.setSpacing(3)
        self.lbl_live = {}
        for key, text in (("norm", "|a| raw"), ("norm_cal", "|a| cal"),
                          ("ax", "acc x"), ("ay", "acc y"), ("az", "acc z"),
                          ("wnorm", "|w| raw"), ("wnorm_cal", "|w| cal"),
                          ("gx", "gyr x"), ("gy", "gyr y"), ("gz", "gyr z"),
                          ("mnorm", "|m| raw"), ("mnorm_cal", "|m| cal"),
                          ("mref", "|m| ref"),
                          ("hdg", "hdg raw"), ("hdg_cal", "hdg cal"),
                          ("temp", "temp"), ("rate", "rate"),
                          ("samples", "samples")):
            row = QtWidgets.QHBoxLayout()
            name = QtWidgets.QLabel(text)
            name.setStyleSheet(MONO + DIM)
            name.setFixedWidth(58)
            val = QtWidgets.QLabel("--")
            val.setStyleSheet(MONO)
            val.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight
                             | QtCore.Qt.AlignmentFlag.AlignVCenter)
            row.addWidget(name)
            row.addWidget(val, 1)
            readout.addLayout(row)
            self.lbl_live[key] = val
        readout.addStretch(1)
        ch.addLayout(readout)
        v.addWidget(card, 1)

        self.meter = StillnessMeter()
        v.addWidget(self.meter)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setFormat("")
        v.addWidget(self.progress)

        row = QtWidgets.QHBoxLayout()
        self.btn_record = QtWidgets.QPushButton("Start recording")
        self.btn_record.setObjectName("primary")
        self.btn_record.clicked.connect(self.on_record)
        row.addWidget(self.btn_record)
        self.btn_solve = QtWidgets.QPushButton("Stop and solve")
        self.btn_solve.clicked.connect(self.on_solve)
        row.addWidget(self.btn_solve)
        self.btn_save = QtWidgets.QPushButton("Save config.yaml")
        self.btn_save.clicked.connect(self.on_save)
        row.addWidget(self.btn_save)
        # The board keeps its own calibration table and interpolates over
        # temperature, so a session contributes the operating point it
        # was measured at rather than replacing what is stored.
        self.btn_upload = QtWidgets.QPushButton("Upload to board")
        self.btn_upload.setToolTip(
            "Write this session as one temperature node into the board's "
            "own calibration table")
        self.btn_upload.clicked.connect(self.on_upload)
        row.addWidget(self.btn_upload)
        row.addStretch(1)
        v.addLayout(row)

        self.lbl_result = QtWidgets.QLabel("")
        self.lbl_result.setStyleSheet(MONO)
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        # Scrolled: the block is the IMU summary, the before/after table and
        # the magnetometer result together, and none of them may push
        # another one out of the window.
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setMinimumHeight(190)
        scroll.setWidget(self.lbl_result)
        v.addWidget(scroll)
        return panel

    def _build_plots_panel(self):
        panel = QtWidgets.QFrame()
        panel.setObjectName("panel")
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(8, 8, 8, 8)
        self.tabs = QtWidgets.QTabWidget()
        v.addWidget(self.tabs)

        self.plots = pg.GraphicsLayoutWidget()
        self.plots.setBackground(QtGui.QColor(15, 18, 22))
        self.tabs.addTab(self.plots, "Live")
        self.result_plots = pg.GraphicsLayoutWidget()
        self.result_plots.setBackground(QtGui.QColor(15, 18, 22))
        self.tabs.addTab(self.result_plots, "Before / after")
        self._build_result_plots()
        self.tabs.addTab(self._build_mag_tab(), "Magnetometer")

        self.p_acc = self.plots.addPlot(row=0, col=0)
        self.p_acc.setLabel("left", "accel", units="m/s²")
        self.p_acc.showGrid(x=True, y=True, alpha=0.2)
        self.p_acc.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.c_acc = [self.p_acc.plot(pen=p, name=n) for p, n in
                      ((PEN_X, "x"), (PEN_Y, "y"), (PEN_Z, "z"))]
        self.p_gyr = self.plots.addPlot(row=1, col=0)
        self.p_gyr.setLabel("left", "gyro", units="deg/s")
        self.p_gyr.setLabel("bottom", "t", units="s")
        self.p_gyr.showGrid(x=True, y=True, alpha=0.2)
        self.p_gyr.setXLink(self.p_acc)
        self.c_gyr = [self.p_gyr.plot(pen=p) for p in (PEN_X, PEN_Y, PEN_Z)]

        # The point of the whole exercise, live: at any attitude the
        # uncalibrated magnitude wanders, the calibrated one sits on |g|.
        # Empty until a calibration exists, from this session or from the
        # config.yaml being written to.
        self.p_norm = self.plots.addPlot(row=2, col=0)
        self.p_norm.setLabel("left", "|a|", units="m/s²")
        self.p_norm.setLabel("bottom", "t", units="s")
        self.p_norm.showGrid(x=True, y=True, alpha=0.2)
        self.p_norm.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.p_norm.setXLink(self.p_acc)
        self.c_norm_raw = self.p_norm.plot(
            pen=pg.mkPen("#e0b44a", width=1), name="uncalibrated")
        self.c_norm_cal = self.p_norm.plot(
            pen=pg.mkPen(C_OK, width=2), name="calibrated", connect="finite")
        self.l_norm_g = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#8fa0b4", width=1,
                         style=QtCore.Qt.PenStyle.DashLine))
        self.p_norm.addItem(self.l_norm_g)
        return panel

    def _build_mag_tab(self):
        """Field directions covered, and what the calibration did to |m|.

        The same reasoning as the gravity sphere: hard and soft iron are
        recovered from the SHAPE of the sample cloud, so a cloud that
        never left one plane cannot give them, however many samples went
        into it. The |m| trace is the check that needs no interpretation,
        because a calibrated magnetometer reads one number at rest no
        matter which way it points."""
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(6)
        self.mag_sphere = PoseSphere(caption="%d field directions",
                                     colour="#c07ad8")
        self.mag_sphere.setMinimumHeight(210)
        v.addWidget(self.mag_sphere, 1)
        plots = pg.GraphicsLayoutWidget()
        plots.setBackground(QtGui.QColor(15, 18, 22))
        v.addWidget(plots, 1)
        self.p_mag = plots.addPlot(row=0, col=0)
        self.p_mag.setLabel("left", "mag", units="uT")
        self.p_mag.showGrid(x=True, y=True, alpha=0.2)
        self.p_mag.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.c_mag = [self.p_mag.plot(pen=p, name=n) for p, n in
                      ((PEN_X, "x"), (PEN_Y, "y"), (PEN_Z, "z"))]
        self.p_mnorm = plots.addPlot(row=1, col=0)
        self.p_mnorm.setLabel("left", "|m|", units="uT")
        self.p_mnorm.setLabel("bottom", "t", units="s")
        self.p_mnorm.showGrid(x=True, y=True, alpha=0.2)
        self.p_mnorm.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.p_mnorm.setXLink(self.p_mag)
        self.c_mnorm_raw = self.p_mnorm.plot(
            pen=pg.mkPen("#e0b44a", width=1), name="uncalibrated")
        self.c_mnorm_cal = self.p_mnorm.plot(
            pen=pg.mkPen(C_OK, width=2), name="calibrated", connect="finite")
        self.l_mnorm_f = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#8fa0b4", width=1,
                         style=QtCore.Qt.PenStyle.DashLine))
        # The line |m| has to sit on, exactly as the |a| plot has one at
        # |g|. Hidden until a field strength is known (WMM lookup, typed
        # in, or measured by a solve): a reference line at zero is not a
        # reference.
        self.l_mnorm_f.setVisible(False)
        self.p_mnorm.addItem(self.l_mnorm_f)
        return w

    def _field_target(self):
        """What |m| should read [uT], or None while nothing knows.

        The WMM lookup (or a typed-in value) wins over the calibration's
        own scale, so the plot can show the deviation from the TRUE field
        before anything has been solved. Once a session has been solved
        without a reference, its own measured field is the target, which
        is what the calibration was scaled to."""
        if self.sp_field.value() > 0.0:
            return float(self.sp_field.value())
        if self.magcal is not None:
            return float(self.magcal.field_ut)
        return None

    def _sync_field_line(self):
        f = self._field_target()
        self.l_mnorm_f.setVisible(f is not None)
        if f is not None:
            self.l_mnorm_f.setPos(f)

    def _build_result_plots(self):
        """What the calibration did, one point per static pose.

        The accelerometer must read exactly |g| at rest and the gyro
        exactly zero, in every pose. Those two lines are the ground truth
        of the whole procedure, so plotting the poses against them shows
        the result without anybody having to trust a residual number."""
        self.rp_acc = self.result_plots.addPlot(row=0, col=0)
        self.rp_acc.setLabel("left", "|a| at rest", units="m/s²")
        self.rp_acc.showGrid(x=True, y=True, alpha=0.2)
        self.rp_acc.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.rc_acc_raw = self.rp_acc.plot(
            pen=pg.mkPen("#e0b44a", width=1, style=QtCore.Qt.PenStyle.DashLine),
            symbol="o", symbolSize=5, symbolBrush="#e0b44a",
            symbolPen=None, name="uncalibrated")
        self.rc_acc_cal = self.rp_acc.plot(
            pen=pg.mkPen(C_OK, width=2), symbol="o", symbolSize=5,
            symbolBrush=C_OK, symbolPen=None, name="calibrated")
        self.rl_g = pg.InfiniteLine(angle=0, movable=False,
                                    pen=pg.mkPen("#8fa0b4", width=1,
                                                 style=QtCore.Qt.PenStyle.DashLine))
        self.rp_acc.addItem(self.rl_g)

        self.rp_gyr = self.result_plots.addPlot(row=1, col=0)
        self.rp_gyr.setLabel("left", "rate at rest", units="deg/s")
        self.rp_gyr.setLabel("bottom", "static pose")
        self.rp_gyr.showGrid(x=True, y=True, alpha=0.2)
        self.rp_gyr.addLegend(offset=(8, 8), labelTextSize="8pt")
        self.rp_gyr.setXLink(self.rp_acc)
        dim = ("#7a4d4d", "#3f7a58", "#3b6b96")
        self.rc_gyr_raw = [
            self.rp_gyr.plot(pen=pg.mkPen(dim[i], width=1,
                                          style=QtCore.Qt.PenStyle.DashLine),
                             symbol="o", symbolSize=4, symbolBrush=dim[i],
                             symbolPen=None,
                             name=("uncalibrated" if i == 0 else None))
            for i in range(3)]
        self.rc_gyr_cal = [
            self.rp_gyr.plot(pen=p, symbol="o", symbolSize=4,
                             symbolBrush=c, symbolPen=None,
                             name=("calibrated x/y/z" if i == 0 else None))
            for i, (p, c) in enumerate(((PEN_X, "#e06060"), (PEN_Y, "#5fd08a"),
                                        (PEN_Z, "#54aaff")))]
        self.rp_gyr.addItem(pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#8fa0b4", width=1,
                         style=QtCore.Qt.PenStyle.DashLine)))

    def _show_result_plots(self, cal):
        n = len(cal.pose_acc_raw)
        if n == 0:
            return
        x = np.arange(1, n + 1)
        self.rc_acc_raw.setData(x, np.asarray(cal.pose_acc_raw))
        self.rc_acc_cal.setData(x, np.asarray(cal.pose_acc_cal))
        self.rl_g.setPos(cal.gravity)
        graw = np.asarray(cal.pose_gyr_raw) * DEG
        gcal = np.asarray(cal.pose_gyr_cal) * DEG
        for i in range(3):
            self.rc_gyr_raw[i].setData(x, graw[:, i])
            self.rc_gyr_cal[i].setData(x, gcal[:, i])
        self.tabs.setCurrentIndex(1)

    # ------------------------------------------------------------------
    def say(self, text):
        self.log.appendPlainText(text)
        self.log.verticalScrollBar().setValue(
            self.log.verticalScrollBar().maximum())

    def _scan_ports(self):
        current = self.cb_port.currentText()
        self.cb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cb_port.addItem(p.device)
        except ImportError:
            pass
        if current:
            self.cb_port.setCurrentText(current)

    def _on_latlon(self):
        """Fill both reference magnitudes in from the position.

        The fit scales the magnetometer to a field strength and the
        accelerometer to a |g|, and both come from INSLIB's own models
        here -- the WMM and WGS84 normal gravity. Same reason in both
        cases: these are the magnitudes the filter will later assume, so
        taking them from anywhere else turns a calibration into a
        disagreement between two models."""
        spec = self.ed_latlon.text().strip()
        if not spec:
            self._field_source = ""
            return
        try:
            vals = [float(x) for x in spec.replace(";", ",").split(",")
                    if x.strip()]
        except ValueError:
            vals = []
        if len(vals) not in (2, 3):
            self.say("[calib] '%s' is not 'lat, lon' or 'lat, lon, height_m'"
                     % spec)
            return
        lat, lon = vals[0], vals[1]
        # Ellipsoidal. Left out it reads as sea level, and the free air
        # term is 3.1e-6 m/s^2 for every metre that was not given.
        height = vals[2] if len(vals) == 3 else 0.0

        # Gravity before the field: it needs neither a magnetometer nor a
        # field strength, so a WMM lookup that comes back empty must not
        # take it down with it.
        self._set_gravity_from(lat, lon, height)

        year = (datetime.date.today().toordinal()
                - datetime.date(2000, 1, 1).toordinal()) / 365.25 + 2000.0
        ref = mag_calib.wmm_reference(lat, lon, year)
        if ref is None:
            self.say("[calib] no WMM lookup available (build the shared "
                     "library with 'make pylib'), enter the field strength "
                     "by hand")
            return
        field, decl, incl = ref
        self.sp_field.setValue(field)
        self._field_source = "WMM at %.3f %.3f, %.1f" % (lat, lon, year)
        self.say("[calib] WMM at %.3f %.3f: %.1f uT, declination %+.1f deg, "
                 "inclination %+.1f deg" % (lat, lon, field, decl, incl))
        self._wmm_dip = incl

    def _set_gravity_from(self, lat, lon, height):
        """Gravity from WGS84 normal gravity at this position.

        The default 9.80665 is right at about 45.5 degrees latitude at sea
        level and nowhere else. The cost function is |g| - ||M (a - b)||,
        so whatever this number is off by lands on the accelerometer scale
        factors one for one -- 2.6e-4 of them at 48.9 degrees, which is
        larger than the scale factor stability of a good part."""
        g = calib.local_gravity(lat, height)
        if g is None:
            self.say("[calib] no gravity lookup available (build the shared "
                     "library with 'make pylib'), Gravity left at %.5f"
                     % self.sp_gravity.value())
            return
        self.sp_gravity.setValue(g)
        self.say("[calib] normal gravity at %.3f %.3f, h=%.0f m (ellipsoidal)"
                 ": %.5f m/s2, against the standard %.5f"
                 % (lat, lon, height, g, calib.G_MPS2))
        # The |g| line in the norm plot is what a reader compares the live
        # trace against, so it has to move with the number.
        line = getattr(self, "l_norm_g", None)
        if line is not None:
            line.setPos(g)

    def _on_gnss(self, fix):
        """Track the receiver's own NAV-PVT, if the link carries one.

        Autofills Position the first time a 3D fix arrives and the field
        is still empty; a value already typed in is never overwritten,
        the GNSS button stays the way to pull a later fix in on top of it."""
        if fix is None:
            return
        self._gnss_fix = fix
        good = fix.fix_ok and fix.fix_type in (3, 4)
        self.btn_gnss.setEnabled(good)
        self.btn_gnss.setToolTip(
            "Fill the position in from the receiver's own NAV-PVT fix "
            "(%s, %d sat)" % (FIX_TYPE_TEXT.get(fix.fix_type, "fix"),
                              fix.num_sv))
        if good and not self._gnss_autofilled and not self.ed_latlon.text().strip():
            self._gnss_autofilled = True
            self._use_gnss_fix()

    def _use_gnss_fix(self):
        """Fill Position in from the receiver's own NAV-PVT, instead of it
        being typed in by hand."""
        fix = self._gnss_fix
        if fix is None:
            return
        # The height goes in too: gravity needs it, and NAV-PVT carries it
        # against the ellipsoid, which is the one the free air term wants.
        self.ed_latlon.setText("%.6f, %.6f, %.0f"
                               % (fix.lat_deg, fix.lon_deg, fix.height_m))
        self.say("[calib] position from NAV-PVT: %.6f, %.6f, h=%.0f m "
                 "(%d sat)" % (fix.lat_deg, fix.lon_deg, fix.height_m,
                               fix.num_sv))
        self._on_latlon()

    def _pick_out(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "config.yaml to write (an existing one is merged)",
            self.ed_out.text(), "YAML (*.yaml *.yml);;All files (*)")
        if path:
            self.ed_out.setText(path)
            self._on_out_changed()

    def _on_out_changed(self):
        self.out_path = self.ed_out.text().strip()
        # Load (and validate) now rather than after a whole session: a
        # merge that cannot work should say so while nothing is at stake.
        self.existing = None
        try:
            cfg = calib.read_config(self.out_path)
        except ValueError as e:
            self.say("[calib] %s" % e)
            return
        except OSError as e:
            self.say("[calib] cannot read %s: %s" % (self.out_path, e))
            return
        if cfg is None:
            return
        self.existing = cfg
        note = (", keeping its tuned noise model"
                if calib.has_tuned_noise(cfg) else "")
        self.say("[calib] will merge into the existing %s%s"
                 % (os.path.basename(self.out_path), note))
        # Its calibration drives the live view straight away, so an
        # earlier session can be checked without recording a new one.
        if self.cal is None:
            existing_cal = calib.calib_from_config(cfg)
            if existing_cal is not None:
                self.set_live_cal(existing_cal,
                                  "from " + os.path.basename(self.out_path))

    def _set_connected(self, on):
        self.btn_connect.setText("Disconnect" if on else "Connect")
        for w in (self.cb_source, self.cb_port, self.sp_baud, self.btn_rescan,
                  self.ed_udp):
            w.setEnabled(not on)
        self.btn_record.setEnabled(on)
        self.btn_capture.setEnabled(on)
        self.btn_solve.setEnabled(False)
        self._update_save()

    def _update_save(self):
        """Save is offered as soon as anything measured is worth writing.

        Not only after a full IMU solve: a housing alignment or a
        magnetometer calibration against an existing config.yaml is a
        complete piece of work on its own."""
        ok = self.cal is not None and self.cal.n_positions >= calib.MIN_POSITIONS
        self.btn_save.setEnabled(bool(ok or self.housing is not None
                                      or self.magcal is not None))
        # Uploading needs a solved calibration AND a link that can carry
        # a request back to the board. A hub fan-out is one-way and the
        # demo source is not a board, so the button stays off there
        # rather than failing once pressed.
        can_send = self.worker is not None and self.worker.can_configure()
        # A housing pass on its own is uploadable too, now that the board
        # keeps the rotation as its own key: it is one write and needs no
        # temperature node behind it.
        self.btn_upload.setEnabled(bool(can_send and self.uploader is None
                                        and (ok or self.magcal is not None
                                             or self.housing is not None)))

    def recording(self):
        return self.worker is not None and self.worker.snapshot() is not None

    # ------------------------------------------------------------------
    def on_connect(self):
        if self.worker is not None:
            self._disconnect()
            return
        if self.args.demo:
            self.source = DemoSerial()
            self.say("[calib] --demo: synthetic IMU stream, no board")
        elif self.cb_source.currentIndex() == 1:
            spec = self.ed_udp.text().strip()
            host, _, port = spec.rpartition(":")
            try:
                self.source = calib.UdpSource(host or "0.0.0.0", int(port))
            except ValueError:
                self.say("[calib] '%s' is not a port or HOST:PORT" % spec)
                return
            except OSError as e:
                self.say("[calib] cannot listen on %s: %s (is something else "
                         "bound to it?)" % (spec, e))
                return
            self.say("[calib] listening on udp:%s -- the hub needs a fan-out "
                     "to this port" % spec)
        else:
            port = self.cb_port.currentText().strip()
            if not port:
                self.say("[calib] pick a serial port first")
                return
            try:
                import serial
                self.source = serial.Serial(port, self.sp_baud.value(),
                                            timeout=0.05)
            except ImportError:
                self.say("[calib] pyserial is required: pip install pyserial")
                return
            except Exception as e:
                self.say("[calib] cannot open %s: %s" % (port, e))
                self.say("[calib] inslib_hub.py owns the port while it runs; "
                         "switch Source to the hub fan-out instead")
                return
            self.say("[calib] opened %s at %d baud" % (port, self.sp_baud.value()))

        self._gnss_fix = None
        self._gnss_autofilled = False
        self.btn_gnss.setEnabled(False)
        self.worker = ImuWorker(self.source)
        self.worker.sig_live.connect(self.on_live)
        self.worker.sig_poses.connect(self.on_poses)
        self.worker.sig_capture.connect(self.on_captured)
        self.worker.sig_error.connect(self.on_worker_error)
        self.worker.start()
        self._set_connected(True)
        self._on_out_changed()
        self.lbl_link.setStyleSheet(MONO + " color: %s;" % C_OK)
        self.lbl_link.setText("waiting for IMU frames")
        self.lbl_step.setText("Ready — press Start recording")
        self.lbl_instr.setText(
            "Put the unit down first: the recording opens with an "
            "undisturbed rest period that sets the static threshold and "
            "measures the gyro bias.")

    def _disconnect(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None
        self._board_said = False
        if self.source is not None:
            try:
                self.source.close()
            except Exception:
                pass
            self.source = None
        self._set_connected(False)
        self.lbl_link.setStyleSheet(MONO + " color: %s;" % C_IDLE)
        self.lbl_link.setText("not connected")
        self.say("[calib] disconnected")

    def on_record(self):
        """Start a recording, raw if the board can be talked out of it.

        A node solved from a corrected stream is the RESIDUAL of what is
        already stored, and the upload files it beside the absolute nodes
        as though it were one of them: interpolating between the two is
        worse than either. Here is the only place it can still be
        prevented, because it has to happen before the first sample."""
        if self.worker is None or self.applyoff is not None:
            return
        corrected = self._corrected_streams()
        if corrected:
            answer = self._ask_switch_off(corrected)
            if answer is None:
                return
            if answer:
                # Only once the write is actually on its way: a flag set
                # beside a switch-off that never started would fire on
                # the next one, whatever that one was for.
                self._record_after_off = self._start_applyoff(
                    [k for _label, k in corrected])
                return
            self.say("[calib] recording through the board's own "
                     "calibration: what this solves is the RESIDUAL of it, "
                     "and the upload files it as an absolute node")
        self._start_recording()

    def _ask_switch_off(self, corrected):
        """True to clear the switches, False to record anyway, None to
        cancel. Clearing is the default button, because it is the answer
        that makes the recording mean what the upload claims it does."""
        can_send = self.worker is not None and self.worker.can_configure()
        text = ("The board applies its own calibration to: %s.\n\n"
                "A recording taken through it solves what is LEFT OVER of "
                "the stored calibration, while the upload files the result "
                "beside the stored nodes as an absolute one. A second "
                "temperature point measured this way makes the table "
                "worse, not wider."
                % ", ".join(label for label, _k in corrected))
        if can_send:
            text += ("\n\nSwitching off touches the live layer only: "
                     "the stored image stays as it is and the unit comes "
                     "up correcting again at the next power cycle.")
        else:
            # Nothing to offer, so the dialog says where the switch is
            # instead of pretending this window can reach it.
            text += ("\n\nThis link cannot configure the board, so the "
                     "switches have to be cleared elsewhere:\n"
                     "  inslib_cfg.py --port ... set %s"
                     % " ".join("%s=0" % k for _l, k in corrected))

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("The board is correcting the stream")
        box.setText(text)
        b_off = None
        if can_send:
            b_off = box.addButton("Switch off and record",
                                  QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        b_on = box.addButton("Record anyway",
                             QtWidgets.QMessageBox.ButtonRole.DestructiveRole)
        box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(b_off if b_off is not None else b_on)
        box.exec()
        if b_off is not None and box.clickedButton() is b_off:
            return True
        if box.clickedButton() is b_on:
            return False
        return None

    def _start_recording(self):
        self.cal = None
        self.magcal = None
        self.rec_cal_frac = float("nan")
        self.pose_count = 0
        self.sphere.set_poses(np.zeros((0, 3)))
        self.lbl_result.setText("")
        self.worker.start_recording(self.sp_init.value(), self.sp_pose.value())
        self.btn_record.setEnabled(False)
        self.btn_solve.setEnabled(True)
        self._update_save()
        for w in (self.sp_init, self.sp_pose, self.chk_misalign):
            w.setEnabled(False)
        self.say("[calib] recording: keep the unit still for the first %.0f s"
                 % self.sp_init.value())

    def on_solve(self):
        if self.worker is None:
            return
        rec = self.worker.stop_recording()
        self.btn_record.setEnabled(True)
        self.btn_solve.setEnabled(False)
        for w in (self.sp_init, self.sp_pose, self.chk_misalign):
            w.setEnabled(True)
        if rec is None or len(rec) < 1000:
            self.say("[calib] nothing recorded")
            return
        # Kept for the board upload: the magnetometer calibration is
        # looked up by the magnetometer's own die temperature there.
        self.mag_temp_c = (sum(rec.mag_temp_c) / len(rec.mag_temp_c)
                           if rec.mag_temp_c else None)
        # The switch was checked before the recording started, this is
        # what actually reached the samples: it also catches the ones
        # still in flight when the switch went off, and a switch turned
        # back on halfway through.
        self.rec_cal_frac = rec.n_cal_applied / float(len(rec))
        if self.rec_cal_frac > 0.0:
            self.say("[calib] WARNING: %.0f%% of the recording arrived "
                     "already corrected by the board, so what this solves "
                     "is the residual of the stored calibration and not the "
                     "sensor" % (100.0 * self.rec_cal_frac))
        self.say("[calib] solving over %d samples (%.0f s) ..."
                 % (len(rec), len(rec) / max(rec.rate_hz(), 1.0)))
        self.lbl_step.setText("Solving ...")
        self.solver = SolveWorker(rec, self.sp_gravity.value(),
                                  self.sp_init.value(),
                                  self.chk_misalign.isChecked(),
                                  with_mag=self.chk_mag.isChecked(),
                                  mag_field_ut=self.sp_field.value(),
                                  mag_field_source=self._field_source)
        self.solver.sig_log.connect(lambda m: self.say("   " + m))
        self.solver.sig_done.connect(self.on_solved)
        self.solver.sig_error.connect(self.on_solve_failed)
        self.solver.start()

    def on_solved(self, cal, magcal):
        self.cal = cal
        self.magcal = magcal
        self.lbl_step.setText("Calibration ready")
        self.lbl_instr.setText(
            "Check the residuals below, then save. The numbers are only "
            "valid near the temperature range they were measured at.")
        lines = cal.summary() + (magcal.summary() if magcal else [])
        self.lbl_result.setText("\n".join(lines))
        self._show_result_plots(cal)
        self.set_live_cal((cal.acc_matrix, cal.acc_bias,
                           cal.gyr_matrix, cal.gyr_bias), "this session")
        if magcal is not None:
            self._sync_field_line()
            self.say("[calib] the live |m| now runs through this "
                     "calibration: it has to sit on the reference line in "
                     "every attitude, not only in this one")
        for line in lines:
            self.say("   " + line)
        ok = cal.n_positions >= calib.MIN_POSITIONS
        self._update_save()
        if not ok:
            self.say("[calib] only %d poses, need at least %d -- record more"
                     % (cal.n_positions, calib.MIN_POSITIONS))

    def on_solve_failed(self, msg):
        self.lbl_step.setText("Solve failed")
        self.lbl_result.setText(msg)
        self.say("[calib] solve failed: %s" % msg)

    # ------------------------------------------------------------------
    # Housing alignment
    # ------------------------------------------------------------------
    def on_capture(self):
        if self.worker is None:
            return
        self._pending_capture = self.cb_place.currentData()
        self.btn_capture.setEnabled(False)
        self.say("[calib] capturing %.1f s with the unit resting on: %s"
                 % (CAPTURE_SEC,
                    fa.PLACEMENT_LABEL.get(self._pending_capture, "?")))
        board = self._board_corrects_imu()
        if board and self.live_cal is not None:
            self.say("[calib] WARNING: corrected twice, by the board and by "
                     "config.yaml. The tilt this measures is not the "
                     "mounting")
        elif board:
            self.say("[calib] the board corrects the stream itself, so the "
                     "tilt is measured through a calibrated accelerometer")
        elif self.live_cal is None:
            self.say("[calib] no IMU calibration active anywhere, so the "
                     "tilt is measured through the raw accelerometer: an "
                     "uncorrected bias of 0.1 m/s^2 is 0.6 deg of it")
        self.worker.start_capture(CAPTURE_SEC)

    def on_captured(self, info):
        self.btn_capture.setEnabled(self.worker is not None)
        key, self._pending_capture = self._pending_capture, None
        self.lbl_step.setText("Placement captured")
        self.lbl_instr.setText(
            "Roll and pitch are covered by one placement. For the yaw as "
            "well, put the unit on a face that is not parallel to this one "
            "and capture that too.")
        if not info.get("ok") or key is None:
            self._note_capture("capture failed: %s"
                               % info.get("reason", "no placement selected"),
                               bad=True)
            return
        if info["gyr_std_dps"] > self.meter.gyr_lim:
            # A hand still on the box during the capture is the one error
            # this procedure cannot recover from later.
            self._note_capture("discarded: the unit was not still during "
                               "the capture (%.2f deg/s)"
                               % info["gyr_std_dps"], bad=True)
            return
        # Which face the unit ACTUALLY rested on. The solve cannot tell a
        # placement filed under the wrong face from a surface that was not
        # level: it fits whatever it is given and only the residual grows.
        # Gravity is the one thing that can say which of the two it was.
        try:
            off = fa.angle_between_deg(info["acc"], fa.PLACEMENT_IDEAL[key])
            near, _near_off = fa.nearest_placement(info["acc"])
        except ValueError:
            self._note_capture("discarded: the accelerometer read nothing "
                               "to take a direction from", bad=True)
            return
        if off > PLACEMENT_TOL_DEG:
            self._note_capture(
                "discarded: captured as %s, but gravity is %.0f deg away "
                "from that face. This looks like %s"
                % (fa.PLACEMENT_LABEL.get(key, key), off,
                   fa.PLACEMENT_LABEL.get(near, near)), bad=True)
            return
        self.captures.append((key, np.asarray(info["acc"], dtype=float)))
        self._note_capture("captured %s over %d samples, %.1f deg off that "
                           "face" % (fa.PLACEMENT_LABEL.get(key, key),
                                     info["n"], off))
        self._solve_housing()
        self._suggest_next_placement()

    def _note_capture(self, text, bad=False):
        """Say how the last capture went, where the hands are looking.

        The log holds the session, but nobody reads a log while turning
        the unit over, and a capture that was thrown away silently is
        indistinguishable from one that counted."""
        self.lbl_house_msg.setText(text)
        self.lbl_house_msg.setStyleSheet(
            "font-size: 11px; color: %s;" % (C_BAD if bad else C_OK))
        self.say("[calib] " + text)

    def _refresh_place_combo(self):
        """Tick off the faces that are already captured."""
        done = {k for k, _raw in self.captures}
        for i in range(self.cb_place.count()):
            key = self.cb_place.itemData(i)
            self.cb_place.setItemText(
                i, ("✓ " if key in done else "   ")
                + fa.PLACEMENT_LABEL.get(key, key))

    def _refresh_house_steps(self):
        """The two steps and what each one buys.

        A count of placements does not say how many are needed, and the
        answer is not a number but a geometry: one face for the tilt, a
        second non-parallel one for the yaw."""
        tilt = len(self.captures) >= 1
        yaw = self.housing is not None and not self.housing.tilt_only
        self.lbl_house_steps.setText(
            "[%s] 1. one face down      -> roll, pitch\n"
            "[%s] 2. a non-parallel one -> yaw"
            % ("x" if tilt else " ", "x" if yaw else " "))

    def _suggest_next_placement(self):
        """Point the box at a face that would still add something.

        After the first capture that is one which is NOT parallel to what
        is already there: parallel faces repeat roll and pitch and can
        never give the yaw. Once all three angles are pinned the choice
        is the operator's again and this leaves it alone."""
        done = [k for k, _raw in self.captures]
        if not done or (self.housing is not None
                        and not self.housing.tilt_only):
            return
        for i in range(self.cb_place.count()):
            key = self.cb_place.itemData(i)
            if key in done or any(fa.placements_parallel(key, d)
                                  for d in done):
                continue
            self.cb_place.setCurrentIndex(i)
            return

    def _refresh_housing_hints(self, force=False):
        """Keep the panel honest about what the tilt is measured through.

        Three states decide what a capture even means and none of them is
        visible otherwise: a board that corrects its own stream (then this
        measures what is LEFT OVER of a rotation that the upload
        replaces), a config.yaml correcting on top of that (corrected
        twice, and the numbers look fine either way), and nothing
        correcting at all (an accelerometer bias reads as tilt)."""
        corrects = self._board_corrects_imu()
        state = (corrects, self.live_cal is not None, self.live_cal_src)
        if state == self._hint_state and not force:
            return
        self._hint_state = state

        if corrects and self.live_cal is not None:
            text, colour = ("corrected TWICE: by the board, and again by "
                            "the calibration %s. What is measured here is "
                            "neither the mounting error nor what is left "
                            "of it" % self.live_cal_src, C_BAD)
        elif corrects:
            text, colour = ("measured through the board's own calibration",
                            C_WARN)
        elif self.live_cal is not None:
            text, colour = ("measured through the calibration %s"
                            % self.live_cal_src, C_OK)
        else:
            text, colour = ("measured through the RAW accelerometer: 0.1 "
                            "m/s² of bias reads as 0.6 deg of tilt. "
                            "Calibrate first, or point -o at a config.yaml "
                            "that carries one", C_WARN)
        self.lbl_house_src.setText(text)
        self.lbl_house_src.setStyleSheet("font-size: 11px; color: %s;"
                                         % colour)

        self.lbl_house_warn.setText(
            "The board corrects its own stream, so a capture measures only "
            "what is LEFT OVER of the mounting error -- while the upload "
            "REPLACES the rotation stored there. Switch it off for an "
            "absolute measurement.")
        self.lbl_house_warn.setVisible(bool(corrects))
        can_send = self.worker is not None and self.worker.can_configure()
        self.btn_applycal_off.setVisible(bool(corrects) and can_send)

    def on_applycal_off(self):
        # The housing capture is measured through the accelerometer, so
        # this is the accelerometer's switch and only that one. The
        # magnetometer's is cleared where a magnetometer calibration is
        # about to be recorded.
        self._start_applyoff(["CFG-IMU-APPLY_CAL"])

    def _start_applyoff(self, keys):
        """True when a switch-off is running, so a caller waiting on it
        knows whether to expect on_applycal_done()."""
        if self.worker is None or self.applyoff is not None:
            return False
        self.btn_applycal_off.setEnabled(False)
        self.applyoff = ApplyCalWorker(self.worker, keys)
        self.applyoff.sig_done.connect(self.on_applycal_done)
        self.applyoff.start()
        return True

    def on_applycal_done(self, ok, msg):
        # The same hold the upload needs: the signal comes out of run()
        # and the thread object has to outlive it.
        thread, self.applyoff = self.applyoff, None
        keys = ", ".join(thread.keys) if thread is not None else ""
        if thread is not None:
            thread.wait(2000)
        self.btn_applycal_off.setEnabled(True)
        # Whatever this was for, it is answered here: a recording that
        # waited on the switch must not be left waiting on a write that
        # failed, and must not start through a stream still corrected.
        start, self._record_after_off = self._record_after_off, False
        if ok:
            self.say("[calib] %s cleared in the live layer: the board "
                     "streams the raw sensor from now on. The stored image "
                     "is untouched, so it comes up correcting again after a "
                     "power cycle" % keys)
            # The panel follows the board's own state message rather than
            # this write, the same way it does for everything else it says
            # about the board.
            self._board_said = False
            if start:
                self._start_recording()
        else:
            self.say("[calib] could not clear %s: %s" % (keys, msg))
            if start:
                self.say("[calib] recording NOT started, the stream would "
                         "still arrive corrected")

    def on_clear_housing(self):
        self.captures = []
        self.housing = None
        self.lst_place.clear()
        self.lbl_housing.setText("nothing captured")
        self.lbl_house_msg.setText("")
        self._refresh_place_combo()
        self._refresh_house_steps()
        self._update_save()
        self.say("[calib] housing alignment cleared, the live view is back "
                 "on the sensor frame")

    def _solve_housing(self):
        """Re-solve from every placement captured so far.

        Always from scratch, and always through the CURRENT IMU
        calibration rather than the corrected live view: that way the
        rotation is a fresh absolute answer instead of a correction on top
        of a correction, and capturing one more placement can only improve
        it."""
        obs = []
        for key, raw in self.captures:
            a = raw
            if self.live_cal is not None:
                am, ab, _gm, _gb = self.live_cal
                a = np.asarray(am) @ (raw - np.asarray(ab))
            obs.append((key, a))
        try:
            self.housing = fa.housing_rotation(obs)
        except ValueError as e:
            self.housing = None
            self.lbl_housing.setText(str(e))
            self._refresh_place_combo()
            self._refresh_house_steps()
            self._update_save()
            return
        self.lst_place.clear()
        for (key, _raw), res in zip(self.captures, self.housing.residual_deg):
            self.lst_place.addItem("%-22s %5.2f deg left"
                                   % (fa.PLACEMENT_LABEL.get(key, key), res))
        self.lbl_housing.setText("\n".join(self.housing.summary()))
        for line in self.housing.summary():
            self.say("   " + line)
        self._refresh_place_combo()
        self._refresh_house_steps()
        self._update_save()

    def _housing_matrix(self):
        return None if self.housing is None else self.housing.as_matrix()

    # ------------------------------------------------------------------
    def _apply_live_cal(self, acc, gyr):
        """The live sample as the filter would see it, or None.

        The housing rotation sits on top of the calibration matrices, the
        same composition the config gets on save, so what is on screen is
        what the file will contain."""
        if self.live_cal is None:
            return None, None
        am, ab, gm, gb = self.live_cal
        r = self._housing_matrix()
        am, gm = np.asarray(am), np.asarray(gm)
        if r is not None:
            am, gm = r @ am, r @ gm
        return (imu_tk.apply_calib(acc, am, ab),
                imu_tk.apply_calib(gyr, gm, gb))

    def _apply_live_mag(self, mag):
        """The live magnetometer sample after this session's calibration."""
        if self.magcal is None or mag is None:
            return None
        m = self.magcal.rotated(self._housing_matrix())
        return mag_calib.apply_calib(mag, m.matrix, m.bias)

    def set_live_cal(self, cal, source):
        """Use `cal` (acc_M, acc_b, gyr_M, gyr_b) for the live view."""
        self.live_cal = cal
        self.live_cal_src = source if cal is not None else ""
        self._refresh_housing_hints(force=True)
        self.l_norm_g.setPos(self.sp_gravity.value())
        if cal is not None:
            self.say("[calib] live view now shows calibrated values (%s)"
                     % source)
        # The placements are stored RAW and corrected at solve time, so a
        # calibration arriving after them makes the rotation on screen
        # stale rather than wrong-by-construction. Re-solve: _solve_housing
        # promises the CURRENT calibration, and the captures are still the
        # measurements they were, so this costs nothing and removes the
        # only order that mattered inside a session.
        if self.captures:
            self.say("[calib] re-solving the housing alignment through the "
                     "calibration that just arrived")
            self._solve_housing()

    def _board_corrects_imu(self):
        """Whether the samples arriving are already corrected by the board.

        Two things have to hold: the APPLY_CAL switch is on AND there is
        something to apply, a valid node table or a housing rotation.
        None when the board sends no state message, which is the only
        honest answer for a link that carries none."""
        info = self.worker.board_info() if self.worker is not None else None
        if info is None:
            return None
        if not info["imu_cal_applied"]:
            return False
        return bool(info["cal_flags"] & (ubx.CAL_ACC_VALID | ubx.CAL_HOUSING))

    def _board_corrects_mag(self):
        """The magnetometer's switch, which is not the IMU's.

        Two keys, two answers: one stream can be raw while the other is
        corrected, and a magnetometer calibration solved from a corrected
        magnetometer is a residual dressed up as an absolute one exactly
        the way an IMU one is."""
        info = self.worker.board_info() if self.worker is not None else None
        if info is None:
            return None
        if not info["mag_cal_applied"]:
            return False
        return bool(info["cal_flags"] & (ubx.CAL_MAG_VALID | ubx.CAL_HOUSING))

    def _corrected_streams(self):
        """[(what it is in words, the switch to clear)] for every stream
        this recording would read through the board's own calibration.

        The magnetometer counts only while it is going to be solved for:
        turning a correction off that nothing here reads would be
        changing the unit for no reason."""
        out = []
        if self._board_corrects_imu():
            out.append(("the IMU", "CFG-IMU-APPLY_CAL"))
        if self.chk_mag.isChecked() and self._board_corrects_mag():
            out.append(("the magnetometer", "CFG-MAG-APPLY_CAL"))
        return out

    def _report_board_state(self):
        """Say once what the board is doing to the stream.

        The window otherwise only knows what config.yaml says, which is a
        statement about the file and not about the samples arriving. The
        two disagreeing is the expensive case: a board that corrects and
        a config.yaml that corrects again means every sample is corrected
        twice, and nothing about the numbers looks wrong."""
        if self._board_said:
            return
        info = self.worker.board_info() if self.worker is not None else None
        if info is None:
            return
        self._board_said = True

        corrects = self._board_corrects_imu()
        self.say("[calib] the board applies its own calibration: %s"
                 % ("yes, the stream arriving here is already corrected"
                    if corrects else
                    "no, the stream is raw" if not info["imu_cal_applied"]
                    else "switch is on but it has nothing stored to apply"))
        if self._board_corrects_mag():
            self.say("[calib] the magnetometer arrives corrected as well, "
                     "which is its own switch (CFG-MAG-APPLY_CAL)")
        if info["cal_flags"] & ubx.CAL_HOUSING_BAD:
            self.say("[calib] the board holds a housing rotation it REFUSED: "
                     "the stored record is not a rotation")
        elif info["cal_flags"] & ubx.CAL_HOUSING:
            self.say("[calib] the board carries a housing rotation of its "
                     "own, which this session will replace on upload")
        if corrects and self.live_cal is not None:
            self.say("[calib] WARNING: config.yaml ALSO carries an IMU "
                     "calibration and it is applied on top of the board's. "
                     "Every sample is being corrected twice -- comment the "
                     "imu matrices out, or clear CFG-IMU-APPLY_CAL")

    def on_live(self, snap):
        self._report_board_state()
        self._refresh_housing_hints()
        self._on_gnss(snap.get("gnss"))
        acc, gyr = snap["acc"], snap["gyr"]
        self.lbl_live["norm"].setText("%.2f m/s²" % self.f_anorm(snap["norm"]))
        wnorm = math.sqrt(sum(v * v for v in gyr)) * DEG
        self.lbl_live["wnorm"].setText("%.1f dps" % self.f_wnorm(wnorm))
        a_cal, w_cal = self._apply_live_cal(acc, gyr)
        # Both buffers are appended to on EVERY sample, NaN before a
        # calibration exists: they have to stay index-aligned with
        # trace_t, and a curve that starts mid-history would otherwise
        # never line up with the time axis again.
        n_cal = float("nan")
        if a_cal is None:
            self.f_anorm_cal.reset()
            self.f_wnorm_cal.reset()
            self.lbl_live["norm_cal"].setText("--")
            self.lbl_live["wnorm_cal"].setText("--")
        else:
            n_cal = float(np.linalg.norm(a_cal))
            self.lbl_live["norm_cal"].setText(
                "%.2f m/s²" % self.f_anorm_cal(n_cal))
            self.lbl_live["wnorm_cal"].setText(
                "%.1f dps"
                % self.f_wnorm_cal(float(np.linalg.norm(w_cal)) * DEG))
        self.trace_anorm_raw.append(snap["norm"])
        self.trace_anorm_cal.append(n_cal)
        for i, k in enumerate(("ax", "ay", "az")):
            self.lbl_live[k].setText("%+.3f" % acc[i])
        for i, k in enumerate(("gx", "gy", "gz")):
            self.lbl_live[k].setText(
                "%+.1f dps" % self.f_gyr[i](gyr[i] * DEG))
        self.lbl_live["temp"].setText("%.1f °C" % self.f_temp(snap["temp"]))
        self.lbl_live["rate"].setText("%.0f Hz" % snap["rate_hz"]
                                      if snap["rate_hz"] > 0 else "--")
        self.lbl_live["samples"].setText("%d" % snap["samples"])
        self.meter.set_values(snap["gyr_std_dps"], snap["acc_std"],
                              snap["valid"])
        self.sphere.set_live(np.asarray(acc))
        self._live_mag(snap, a_cal)

        if snap["capturing"] > 0.0:
            self.lbl_step.setText("Capturing placement — %.1f s left"
                                  % snap["capturing"])
            self.lbl_instr.setText(
                "Hands off. The mean over the whole window is what is used, "
                "so a touch anywhere in it costs the placement.")
        elif snap["recording"]:
            el, init = snap["elapsed"], self.sp_init.value()
            if el < init:
                self.progress.setValue(int(1000 * el / init))
                self.lbl_step.setText("Initial rest period — %.0f s left"
                                      % max(0.0, init - el))
                self.lbl_instr.setText(
                    "Do not touch the unit. This period sets the threshold "
                    "that decides what counts as static, and measures the "
                    "gyro bias.")
            else:
                self.progress.setValue(1000)
                self.lbl_step.setText("Recording — %d poses, %.0f s"
                                      % (self.pose_count, el))
                self.lbl_instr.setText(
                    "Pick it up, turn it to a new attitude, set it down and "
                    "hold ~%.0f s. Aim for directions that are still empty "
                    "on the sphere." % self.sp_pose.value())
        if self.lbl_link.text() == "waiting for IMU frames":
            self.lbl_link.setText("streaming")

        t = time.monotonic() - self._t0
        self.trace_t.append(t)
        for i in range(3):
            self.trace_acc[i].append(acc[i])
            self.trace_gyr[i].append(gyr[i] * DEG)
        while self.trace_t and self.trace_t[0] < t - TRACE_SEC:
            self.trace_t.pop(0)
            for i in range(3):
                self.trace_acc[i].pop(0)
                self.trace_gyr[i].pop(0)
            self.trace_anorm_raw.pop(0)
            self.trace_anorm_cal.pop(0)
            self.trace_mag[0].pop(0)
            self.trace_mag[1].pop(0)
            self.trace_mag[2].pop(0)
            self.trace_mnorm_raw.pop(0)
            self.trace_mnorm_cal.pop(0)

    def _live_mag(self, snap, a_cal=None):
        """Magnetometer read-out and traces, NaN-filled when there is none.

        Every buffer grows by one entry per live update whether a
        magnetometer sample arrived or not, so all of them stay
        index-aligned with trace_t (the field comes in at a tenth of the
        IMU rate, and a curve that skipped entries would drift away from
        the time axis)."""
        mag = snap.get("mag")
        m_cal = self._apply_live_mag(mag)
        field = self._field_target()
        raw_n = cal_n = float("nan")
        if mag is None:
            self.f_mnorm.reset()
            self.f_mnorm_cal.reset()
            self.lbl_live["mnorm"].setText("--")
            self.lbl_live["mnorm_cal"].setText("--")
        else:
            raw_n = math.sqrt(sum(v * v for v in mag))
            self.lbl_live["mnorm"].setText("%.1f uT" % self.f_mnorm(raw_n))
            if m_cal is None:
                self.f_mnorm_cal.reset()
                self.lbl_live["mnorm_cal"].setText("--")
            else:
                cal_n = float(np.linalg.norm(m_cal))
                self.lbl_live["mnorm_cal"].setText(
                    "%.1f uT" % self.f_mnorm_cal(cal_n))
        self.lbl_live["mref"].setText("%.1f uT" % field if field else "--")
        self._live_heading(snap["acc"], mag, a_cal, m_cal)
        for i in range(3):
            self.trace_mag[i].append(mag[i] if mag else float("nan"))
        self.trace_mnorm_raw.append(raw_n)
        self.trace_mnorm_cal.append(cal_n)
        self._sync_field_line()
        if snap["mag_n"]:
            self.lbl_mag.setText(self._mag_status(snap, field))

    def _live_heading(self, acc, mag, a_cal, m_cal):
        """Magnetic heading before and after this session's calibration.

        The raw row is the heading the board would fuse with no
        correction at all, so hard iron or a wrong axis table in the mag
        driver shows up here as a heading that simply sits somewhere
        else. The calibrated row is what the filter sees once the session
        is uploaded, and the difference between the two is what the
        calibration is worth.

        The calibrated row levels on the corrected accelerometer when
        there is one and falls back to the raw one otherwise: a few
        milli-g of accelerometer error move the heading far less than the
        magnetometer terms do, so the row is still worth showing."""
        raw = mag_heading_deg(mag, acc)
        cal = mag_heading_deg(m_cal, acc if a_cal is None else a_cal)
        self.lbl_live["hdg"].setText("--" if raw is None else "%.1f°" % raw)
        self.lbl_live["hdg_cal"].setText(
            "--" if cal is None else "%.1f°" % cal)

    def _mag_status(self, snap, field):
        """The magnetometer's version of the |a| against 9.81 check.

        A calibrated magnetometer reads ONE number whichever way the unit
        is turned, so the useful live quantity is not |m| itself but how
        far it is from the field that is supposed to be there, and how
        much that wanders over the last few seconds. The rms over the
        trace window is the second half of that: a hard iron shows up as
        an offset that CHANGES as the unit is turned, so a single sample
        can look perfect in one attitude and be badly off in the next."""
        if field is None:
            return ("%d frames, |m| %.1f uT. Enter a position or a field "
                    "strength above to compare it against the WMM"
                    % (snap["mag_n"],
                       self.trace_mnorm_raw[-1] if self.trace_mnorm_raw
                       else float("nan")))

        def stats(buf):
            v = np.asarray(buf, dtype=float)
            v = v[np.isfinite(v)]
            if v.size == 0:
                return None
            return (float(v[-1] - field),
                    float(np.sqrt(np.mean((v - field) ** 2))))

        raw = stats(self.trace_mnorm_raw)
        cal = stats(self.trace_mnorm_cal)
        if raw is None:
            return "%d frames, waiting for a sample" % snap["mag_n"]
        txt = ("%d frames\nraw %+.1f uT off, %.1f rms over %.0f s"
               % (snap["mag_n"], raw[0], raw[1], TRACE_SEC))
        if cal is not None:
            txt += "\ncalibrated %+.1f uT off, %.1f rms" % (cal[0], cal[1])
        return txt

    def on_poses(self, info):
        if "mag_dirs" in info:
            self.mag_sphere.set_poses(info["mag_dirs"])
            return
        self.pose_count = info["count"]
        self.sphere.set_poses(info["dirs"])
        self.lbl_poses.setText("%d" % self.pose_count)
        col = C_OK if self.pose_count >= 20 else (
            C_WARN if self.pose_count >= calib.MIN_POSITIONS else C_IDLE)
        self.lbl_poses.setStyleSheet(
            "font-size: 40px; font-weight: bold; color: %s;" % col)

    def _tick_traces(self):
        if not self.trace_t:
            return
        for i in range(3):
            self.c_acc[i].setData(self.trace_t, self.trace_acc[i])
            self.c_gyr[i].setData(self.trace_t, self.trace_gyr[i])
            self.c_mag[i].setData(self.trace_t, self.trace_mag[i],
                                  connect="finite")
        if len(self.trace_anorm_raw) == len(self.trace_t):
            self.c_norm_raw.setData(self.trace_t, self.trace_anorm_raw)
            self.c_norm_cal.setData(self.trace_t, self.trace_anorm_cal)
        if len(self.trace_mnorm_raw) == len(self.trace_t):
            self.c_mnorm_raw.setData(self.trace_t, self.trace_mnorm_raw,
                                     connect="finite")
            self.c_mnorm_cal.setData(self.trace_t, self.trace_mnorm_cal,
                                     connect="finite")

    def on_worker_error(self, text):
        self.say("[calib] worker failed:\n" + text)
        self._disconnect()

    # ------------------------------------------------------------------
    def on_save(self):
        if self.cal is None and self.housing is None and self.magcal is None:
            return
        try:
            kept = calib.write_calibration(self.out_path, self.cal,
                                           self.existing, magcal=self.magcal,
                                           housing=self.housing)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Write failed", str(e))
            self.say("[calib] writing %s failed: %s" % (self.out_path, e))
            return
        if kept and self.cal is not None:
            self.say("[calib] kept the existing noise model (measured here: "
                     "gyr_psd %.3e, acc_psd %.3e)"
                     % (self.cal.gyr_psd, self.cal.acc_psd))
        self.say("[calib] wrote %s" % self.out_path)
        # The housing rotation is now part of the matrices in the file, so
        # what is still on the screen would apply it a second time on the
        # next save. The captures are kept: they are still valid
        # measurements, and re-solving them against the new calibration
        # gives whatever is left over, which is what a second pass should
        # correct.
        if self.housing is not None and self.cal is None:
            # Only this path composes. With no solve behind it the write
            # goes through rotate_config_imu, which reads the matrices
            # BACK OUT of the file and premultiplies them, so repeating
            # it would apply the same rotation twice. With a solve the
            # write is absolute (build_imu_section takes the session's
            # own matrices), and repeating it is idempotent -- which is
            # why the rotation is retired here and kept there.
            self.housing = None
            self.lst_place.clear()
            self.lbl_housing.setText("folded into the saved calibration; "
                                     "capture again to check what is left")
        # The solve stays, in the SENSOR frame. It is what Upload to
        # board reads, and the board wants the nodes unrotated with the
        # housing rotation beside them as its own key -- so the copies
        # kept here must not be rotated. Saving again is idempotent:
        # with a solve present the matrices in the file are overwritten
        # and not composed.
        # Re-read the file, so a second Save merges into what was just
        # written instead of the state from before it.
        self._on_out_changed()
        self._update_save()
        QtWidgets.QMessageBox.information(
            self, "Saved",
            "Wrote %s\n\nUse it with:\n  insrcv --config %s\n  "
            "python3 python/replay.py <datadir>"
            % (self.out_path, os.path.basename(self.out_path)))

    # ------------------------------------------------------------------
    # Upload into the board's own calibration table
    # ------------------------------------------------------------------
    def _upload_groups(self):
        """{kind: (matrix, bias in stream units, node temperature)}.

        The nodes go up in the SENSOR frame, unrotated. Unlike the file,
        the board holds the housing rotation as its own key
        (CFG-FRAME-HOUSING) and premultiplies it, because its matrices
        are a table over temperature: a rotation folded into some nodes
        and not into others gets interpolated, and an interpolated
        rotation is not one. See cfg_keys.h.

        The bias converts out of SI, because the board corrects in the
        units of its own stream."""
        groups, t_imu = {}, None
        if self.cal is not None and self.cal.n_positions >= calib.MIN_POSITIONS:
            cal = self.cal
            t_imu = 0.5 * (cal.temp_lo + cal.temp_hi)
            groups["acc"] = (cal.acc_matrix,
                             ubx.cal_bias_to_stream_units(cal.acc_bias, "acc"),
                             t_imu)
            groups["gyr"] = (cal.gyr_matrix,
                             ubx.cal_bias_to_stream_units(cal.gyr_bias, "gyr"),
                             t_imu)
        if self.magcal is not None:
            mc = self.magcal
            t_mag = self.mag_temp_c if self.mag_temp_c is not None else t_imu
            if t_mag is not None:
                groups["mag"] = (mc.matrix,
                                 ubx.cal_bias_to_stream_units(mc.bias, "mag"),
                                 t_mag)
        return groups

    def on_upload(self):
        if self.worker is None or self.uploader is not None:
            return
        groups = self._upload_groups()
        housing = self._housing_matrix()
        if not groups and housing is None:
            return

        lines = ["This session goes into the board as one temperature node "
                 "per sensor. What is already stored is kept and "
                 "interpolated between, so the board ends up covering the "
                 "range you have measured across sessions.", ""]
        for kind in ("acc", "gyr", "mag"):
            if kind in groups:
                lines.append("  %-14s node at %.1f degC"
                             % (CAL_LABEL[kind], groups[kind][2]))
        if housing is not None:
            rpy = ubx.housing_rpy_deg(housing)
            lines.append("  %-14s roll %+.2f, pitch %+.2f, yaw %+.2f deg"
                         % ("housing", rpy[0], rpy[1], rpy[2]))
            lines.append("")
            lines.append("The housing rotation is one key of its own on the "
                         "board, not part of the nodes, so it replaces "
                         "whatever is stored there and leaves the "
                         "temperature table untouched.")
        if self.rec_cal_frac > 0.0:
            lines.append("")
            lines.append(
                "WARNING: %.0f%% of the recording arrived ALREADY CORRECTED "
                "by the board. The node about to be written is the residual "
                "of the stored calibration, and it goes into the table "
                "beside nodes that are not. Clear CFG-IMU-APPLY_CAL and "
                "record again." % (100.0 * self.rec_cal_frac))
        if self.cal is not None:
            span = self.cal.temp_hi - self.cal.temp_lo
            lines.append("")
            lines.append("The IMU moved %.1f degC during the recording."
                         % span)
            if span > 3.0:
                lines.append(
                    "That is wide for one node: it is being written at the "
                    "mid point, so the calibration is an average over the "
                    "range rather than a measurement at a temperature. Let "
                    "the unit settle thermally for a tighter one.")
        lines.append("")
        lines.append("Storing costs a flash write and stops the board for up "
                     "to a few hundred milliseconds, and its data streams "
                     "lose that much.")

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Upload to board")
        box.setText("\n".join(lines))
        b_store = box.addButton("Upload and store",
                                QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        b_live = box.addButton("Upload, live only",
                               QtWidgets.QMessageBox.ButtonRole.ActionRole)
        box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        box.exec()
        if box.clickedButton() not in (b_store, b_live):
            return
        persist = box.clickedButton() is b_store

        self.btn_upload.setEnabled(False)
        self.say("[calib] uploading %d node(s)%s to the board%s"
                 % (len(groups),
                    " and the housing rotation" if housing is not None else "",
                    " and storing them" if persist else ""))
        self.uploader = UploadWorker(self.worker, groups, persist, housing)
        self.uploader.sig_log.connect(lambda m: self.say("   " + m))
        self.uploader.sig_done.connect(self.on_uploaded)
        self.uploader.start()

    def on_uploaded(self, ok, msg):
        # sig_done is emitted from inside the worker's run(), and this slot
        # is queued, so it can run before run() has returned. Dropping the
        # last reference to the thread object at that moment destroys a
        # QThread that is still running, and Qt answers that with an abort
        # rather than an exception -- the window simply disappears mid
        # upload. Hold the object until it has left run().
        thread, self.uploader = self.uploader, None
        if thread is not None:
            thread.wait(2000)
        self._update_save()
        if ok:
            self.say("[calib] upload done")
            QtWidgets.QMessageBox.information(self, "Uploaded", msg)
            return
        self.say("[calib] upload failed: %s" % msg)
        QtWidgets.QMessageBox.warning(
            self, "Upload failed",
            "%s\n\nNothing on the board was left half written: the node "
            "counts are the last thing sent, so a table only takes effect "
            "once all of it is there." % msg)

    def closeEvent(self, ev):
        # Same reason as in on_uploaded: an upload still in flight owns a
        # running QThread, and letting the window take it down with it is
        # an abort rather than a clean exit.
        if self.uploader is not None:
            self.uploader.wait(5000)
            self.uploader = None
        self._disconnect()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="", help="serial device to preselect")
    ap.add_argument("--udp", default="", metavar="[HOST:]PORT",
                    help="start on the hub's UDP fan-out instead of a serial "
                         "port (hub: --fanout 127.0.0.1:29800,127.0.0.1:29801)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("-o", "--out", default="config.yaml",
                    help="config.yaml to write (existing one is merged)")
    ap.add_argument("--init-sec", type=float, default=calib.DEFAULT_INIT_SEC)
    ap.add_argument("--pose-sec", type=float, default=calib.DEFAULT_POSE_SEC)
    ap.add_argument("--gravity", type=float, default=calib.G_MPS2,
                    help="local gravity [m/s2] when no position is given. "
                         "A position, typed or from the receiver, overrides "
                         "it with WGS84 normal gravity (default: %(default)s)")
    ap.add_argument("--misalignment", action="store_true",
                    help="start with the misalignment checkbox on. Off "
                         "by default: the off-diagonal terms need poses "
                         "spread over the sphere to be observable at all")
    ap.add_argument("--no-mag", action="store_true",
                    help="start with the magnetometer checkbox off: ignore "
                         "the 0x40/0x06 frames even when the board sends them")
    ap.add_argument("--mag-field-ut", type=float, default=0.0,
                    help="local field strength [uT] to scale the "
                         "magnetometer calibration to (0: keep what the "
                         "sensor measures)")
    ap.add_argument("--latlon", default="", metavar="LAT,LON[,H]",
                    help="position in degrees, with an optional ellipsoidal "
                         "height in metres. Fills in the WMM field strength "
                         "and the local gravity from INSLIB's own models "
                         "instead of asking for either")
    ap.add_argument("--demo", action="store_true",
                    help="synthetic IMU stream instead of a serial port")
    ap.add_argument("--screenshot", metavar="PNG",
                    help="grab the window to PNG and exit (--demo smoke test)")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = CalibWindow(args)
    win.show()
    if args.screenshot:
        def grab():
            win.grab().save(args.screenshot)
            app.quit()
        QtCore.QTimer.singleShot(2500, grab)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
