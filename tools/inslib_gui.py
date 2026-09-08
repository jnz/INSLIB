#!/usr/bin/env python3
"""inslib_gui.py - the board's control room.

A window onto one INSLIB device: what it is configured to do, what it has
stored, and what it currently believes. For developers, integration
engineers and anyone who has to answer "is this unit set up the way I
think it is" without reading a 240x320 panel through a hatch.

Ten tabs, in the order the questions get asked. Overview is the page to
open first and the only one that answers "is this working" on its own.
Solution, Track, GNSS, Sensors, Vertical and Clock each take one part of
that apart: what the filter believes and how well it thinks it knows it,
where it is against the receiver, what the receiver itself is reporting
and how far the two disagree, what the raw triads are doing in time,
magnitude, spectrum and statistics, how the three vertical datums sit
against each other, and how fast the board's own counter runs against GPS
time. Calibration and Configuration are what is stored and what is set,
both editable, and Diagnostics is whether the box is keeping up.

This is deliberately NOT part of inslib_calib_gui.py. That tool drives one
workflow - record, solve, upload a calibration - and its window is built
around the steps of it. This one answers questions about a device that is
already built, which is a different job and a different layout.

What it shares with the rest of the toolbox is the parts that must not
disagree: the key definitions, the record layouts and the framing all come
from inslib_ubx.py, and the link is inslib_cfg.py's CfgLink - the same one
the command line tool uses, including its habit of reading past the sensor
stream rather than treating it as an error.

Two rules run through the whole file.

  - The BOARD is the authority on what a setting may be. This tool does
    not restate the ranges from the firmware's descriptor table; it sends
    what was typed and reports the refusal. A rule copied into a GUI is a
    rule that drifts, and a drifted range rejects a legal value or accepts
    an illegal one, silently, in the one place nobody looks.
  - Only what CHANGED is written. Every field remembers what the board
    reported, and Apply sends the difference. Rewriting a whole
    configuration to change one number is how a stored calibration
    disappears.

Usage:
    python3 tools/inslib_gui.py --port COM5
    python3 tools/inslib_gui.py            # pick the port in the window
    python3 tools/inslib_gui.py --theme light

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import struct
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import inslib_ubx as ux        # noqa: E402
import inslib_cfg as cfgtool   # noqa: E402

try:
    from PyQt6 import QtCore, QtGui, QtWidgets
except ImportError:  # noqa: BLE001
    raise SystemExit("PyQt6 is required: pip install PyQt6")

try:
    import pyqtgraph as pg
except ImportError:  # noqa: BLE001
    raise SystemExit("pyqtgraph is required: pip install pyqtgraph")


# ===========================================================================
# Look and feel
# ===========================================================================

# Two palettes, and every colour in this file comes out of whichever one is
# current. That is what makes a theme a rebind plus a repaint instead of a
# second copy of the window styling: a colour written down twice is a
# colour that later gets changed once.
DARK = {
    "bg": "#0f1216", "panel": "#171b21", "raised": "#1f2832",
    "edge": "#252c35", "line": "#2b3642", "field": "#12151a",
    "hover": "#283441", "box_edge": "#3a4654", "strong": "#ffffff",
    "text": "#d8dee6", "dim": "#8fa0b4",
    "primary": "#1d5c96", "primary_edge": "#2f7fc4",
    "primary_hover": "#246cae", "on_primary": "#ffffff",
    "danger": "#6b2626", "danger_edge": "#8c3434",
    "danger_hover": "#7d2c2c", "on_danger": "#ffffff",
    "ok": "#5fd08a", "warn": "#e0b44a", "bad": "#e06060",
    "accent": "#54aaff", "idle": "#5a6470",
    # A value that differs from the firmware's compiled-in default. Not a
    # fault - a configured device is supposed to differ - but it is the
    # first thing to look at when a unit behaves unlike its siblings.
    "changed": "#c58fe0",
    # The two halves of the artificial horizon. In the palette like
    # everything else, because a colour written down inside a paintEvent
    # is a colour that stays dark when the window goes light.
    "sky": "#1b4a73", "ground": "#5b4527",
    # x, y, z, in that order and nothing else. One triple for every
    # three-axis plot in the window, so a colour means the same axis
    # wherever it appears.
    "trace": ("#e06060", "#5fd08a", "#54aaff"),
}

# Not the dark one inverted. The traces that read well on near-black turn
# into glare on white, so they are darkened rather than lightened, and the
# greys are chosen for contrast against paper.
LIGHT = {
    "bg": "#fbfbfd", "panel": "#eceff4", "raised": "#e2e7ee",
    "edge": "#ccd3dd", "line": "#b6bfcc", "field": "#ffffff",
    "hover": "#d6dde7", "box_edge": "#9aa4b2", "strong": "#101418",
    "text": "#1b2027", "dim": "#5b6675",
    "primary": "#2f7fc4", "primary_edge": "#1d5c96",
    "primary_hover": "#3d8fd4", "on_primary": "#ffffff",
    "danger": "#c2504a", "danger_edge": "#9d3b36",
    "danger_hover": "#d05b55", "on_danger": "#ffffff",
    "ok": "#1c8a4e", "warn": "#8d6100", "bad": "#c0392b",
    "accent": "#1f6fb2", "idle": "#98a2b0",
    "changed": "#7d3c98",
    "sky": "#8fbde4", "ground": "#c0a377",
    "trace": ("#c0392b", "#1c8a4e", "#1f6fb2"),
}

THEMES = {"dark": DARK, "light": LIGHT}

# The palette in force. Read at CALL time everywhere - T["ok"], never a
# copy taken into a constant at import - which is what lets a switch reach
# the whole window.
T = DARK

MONO = "font-family: Consolas, 'DejaVu Sans Mono', monospace; font-size: 12px;"

# The window icon. It lives with the documentation rather than with the
# tools, so the path is taken relative to THIS FILE and not to the working
# directory - the window then has its icon whichever directory it was
# started from.
LOGO = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                    "doc", "figures", "inslib_logo.png")


def app_icon():
    """The application icon, or None when the file is not there.

    A missing logo is not a reason to refuse to start: this tool gets
    copied onto machines that do not carry the documentation tree."""
    if not os.path.isfile(LOGO):
        return None
    icon = QtGui.QIcon(LOGO)
    return None if icon.isNull() else icon


def taskbar_identity():
    """Tell Windows this process is its own application.

    Taskbar buttons are grouped by application id, and a script inherits
    the interpreter's: without this the button carries the Python icon
    however the window is set, and every Qt tool started from the same
    interpreter shares one button."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "inslib.gui")
    except Exception:                                     # noqa: BLE001
        # Cosmetic to begin with, and a shell that will not answer is not
        # a reason to stop.
        pass


def dim():
    """The caption colour, as a stylesheet fragment."""
    return "color: %s;" % T["dim"]


def stylesheet():
    return (
        "QMainWindow, QWidget { background: %(bg)s; color: %(text)s; } "
        "QGroupBox { border: 1px solid %(edge)s; border-radius: 6px; "
        "margin-top: 10px; padding-top: 14px; } "
        "QGroupBox::title { color: %(dim)s; left: 8px; "
        "subcontrol-origin: margin; } "
        "QPushButton { background: %(raised)s; border: 1px solid %(line)s; "
        "border-radius: 4px; padding: 6px 12px; color: %(text)s; } "
        "QPushButton:hover { background: %(hover)s; } "
        "QPushButton:disabled { color: %(idle)s; } "
        "QPushButton#primary { background: %(primary)s; "
        "border-color: %(primary_edge)s; color: %(on_primary)s; } "
        "QPushButton#primary:hover { background: %(primary_hover)s; } "
        "QPushButton#danger { background: %(danger)s; "
        "border-color: %(danger_edge)s; color: %(on_danger)s; } "
        "QPushButton#danger:hover { background: %(danger_hover)s; } "
        "QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit { "
        "background: %(field)s; border: 1px solid %(line)s; padding: 3px; "
        "color: %(text)s; } "
        "QCheckBox::indicator { width: 13px; height: 13px; border: 1px solid "
        "%(box_edge)s; border-radius: 3px; background: %(field)s; } "
        "QCheckBox::indicator:checked { background: %(accent)s; } "
        "QTableWidget { background: %(field)s; gridline-color: %(edge)s; "
        "color: %(text)s; } "
        "QHeaderView::section { background: %(panel)s; color: %(dim)s; "
        "border: 1px solid %(edge)s; padding: 2px 6px; } "
        "QTabWidget::pane { border: 1px solid %(edge)s; } "
        "QTabBar::tab { background: %(panel)s; padding: 6px 14px; "
        "border: 1px solid %(edge)s; border-bottom: none; } "
        "QTabBar::tab:selected { background: %(raised)s; color: %(strong)s; }"
    ) % T


# Everything whose colour is fixed at the moment a widget is built - an
# inline stylesheet, a pen, a symbol brush - registers a callable here.
# Half a repainted window is worse than none, and a pen created once
# survives a theme change unchanged unless somebody goes back for it.
_THEMED = []


def themed(fn):
    """Run fn now, and again after every theme change."""
    _THEMED.append(fn)
    fn()
    return fn


def set_theme(name, app=None):
    """Switch palette and repaint everything that registered a callable."""
    global T
    T = THEMES.get(name, DARK)
    pg.setConfigOptions(background=T["bg"], foreground=T["dim"])
    if app is not None:
        app.setStyleSheet(stylesheet())
    for fn in list(_THEMED):
        try:
            fn()
        except RuntimeError:
            # The Qt object behind it has been deleted. Nothing to
            # repaint, and nothing worth reporting either.
            _THEMED.remove(fn)


def trace_pen(i, width=1):
    return pg.mkPen(T["trace"][i % len(T["trace"])], width=width)


def style_plot(p, title, left, bottom):
    """Put a plot's frame - background, axes, labels, legend - on theme."""
    p.setBackground(T["bg"])
    fg = pg.mkColor(T["dim"])
    for name in ("left", "bottom", "right", "top"):
        ax = p.getAxis(name)
        ax.setPen(fg)
        ax.setTextPen(fg)
    if left:
        p.setLabel("left", left, color=T["dim"])
    if bottom:
        p.setLabel("bottom", bottom, color=T["dim"])
    if title:
        p.setTitle(title, color=T["dim"])
    lg = p.plotItem.legend
    if lg is not None and hasattr(lg, "setLabelTextColor"):
        lg.setLabelTextColor(fg)


class CountAxis(pg.AxisItem):
    """An axis for a quantity that only takes whole values.

    Left to itself pyqtgraph picks a tick spacing from the range alone,
    so a satellite count that sits on 13 gets labelled 13.0, 13.2, 13.4 -
    offering values the quantity cannot take, and reading like a
    measurement with a decimal place. Spacings are rounded up to one and
    the levels that then collide are dropped, which leaves the usual
    choice of major and minor level wherever the range is wide enough to
    have one.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # The prefix would put the ticks back on a fractional scale as
        # soon as the count reached four figures, and would label a
        # handful of satellites in multiples of a thousand.
        self.enableAutoSIPrefix(False)

    def tickSpacing(self, minVal, maxVal, size):
        out = []
        for spacing, offset in super().tickSpacing(minVal, maxVal, size):
            spacing = max(1.0, float(round(spacing)))
            if spacing not in [s for s, _ in out]:
                out.append((spacing, 0.0))
        return out

    def tickStrings(self, values, scale, spacing):
        return ["%d" % int(round(v)) for v in values]


def make_plot(title=None, left=None, bottom=None, legend=False, decimate=False,
              axes=None):
    """A pg.PlotWidget that follows the theme.

    `decimate` is for the streams that arrive by the thousand: pyqtgraph
    then draws what fits the view rather than every sample behind it,
    which is the difference between a long look-back and a stalled window.

    `axes` replaces individual AxisItems, for a plot whose quantity needs
    ticks of its own - a count, say, where CountAxis belongs.
    """
    p = pg.PlotWidget(title=title, axisItems=axes or {})
    p.showGrid(x=True, y=True, alpha=0.2)
    if legend:
        p.addLegend(offset=(-10, 10))
    if decimate:
        p.setDownsampling(auto=True, mode="peak")
        p.setClipToView(True)
    themed(lambda: style_plot(p, title, left, bottom))
    return p


def themed_curves(plot, names, width=1):
    """One curve per axis, coloured x/y/z out of the palette."""
    curves = [plot.plot(pen=trace_pen(i, width), name=n)
              for i, n in enumerate(names)]
    themed(lambda: [c.setPen(trace_pen(i, width))
                    for i, c in enumerate(curves)])
    return curves


def themed_curve(plot, key, name=None, width=1, symbol=None, size=7):
    """One curve whose colour is a palette KEY rather than a literal."""
    if symbol:
        c = plot.plot(pen=None, symbol=symbol, symbolSize=size,
                      symbolBrush=T[key], name=name)
        themed(lambda: c.setSymbolBrush(T[key]))
    else:
        c = plot.plot(pen=pg.mkPen(T[key], width=width), name=name)
        themed(lambda: c.setPen(pg.mkPen(T[key], width=width)))
    return c


def themed_overlay(plot, key, name=None, width=1, dashed=False):
    """A palette curve the view does not auto-range to.

    For the lines that are drawn FROM the data rather than being data -
    an arrow whose length is a fraction of the view, say. Auto-ranging to
    one of those has no fixed point to settle on: the view grows to fit
    the arrow, which makes the arrow longer, which grows the view.
    """
    def pen():
        p = pg.mkPen(T[key], width=width)
        if dashed:
            p.setStyle(QtCore.Qt.PenStyle.DashLine)
        return p

    c = pg.PlotDataItem(pen=pen(), name=name)
    plot.addItem(c, ignoreBounds=True)
    themed(lambda: c.setPen(pen()))
    return c


# How far back the live plots may look, in seconds. The rings below are
# sized for the largest of these; a shorter setting shows less of what is
# already held rather than throwing it away.
HISTORY_CHOICES = (10, 20, 60, 120, 300)
HISTORY_MAX_S = HISTORY_CHOICES[-1]
HISTORY_DEFAULT_S = 60

# Nominal rates, for ring sizing only. A stream faster than one of these
# is not an error: the ring then holds proportionally less time, which is
# what a fixed length ring is for.
#
# The navigation rate is the FASTEST the board can be set to and not the
# rate it ships at, which is 10 Hz. Sizing it for 10 would make the high
# rate mode (CFG-RATE-NAV_MS 5) hold three seconds of history where the
# window promises five minutes, and a plot that silently shortens by a
# factor of twenty is worse than the memory the full ring costs.
RATE_IMU_HZ = 800
RATE_MAG_HZ = 100
RATE_NAV_HZ = 200
RATE_PVT_HZ = 10

# Below this the direction of a horizontal velocity is noise: a track
# angle or a course over ground computed from it swings through the whole
# circle while the platform stands still, so both are blanked instead.
TRACK_MIN_SPEED_MPS = 0.5


# ===========================================================================
# The link, off the UI thread
# ===========================================================================

class LinkWorker(QtCore.QThread):
    """Owns the serial port and runs one job at a time.

    Everything that touches the port happens here: CfgLink is synchronous
    and a poll can wait a second for a CFGINFO frame, which on the UI
    thread is a frozen window. Jobs are plain callables taking the link,
    so the widgets below never learn what a UBX frame is.

    CfgLink reports failures by raising SystemExit, which is right for a
    command line tool and fatal for a window. It is caught here and turned
    into a signal.
    """

    connected = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal(str)
    finished_job = QtCore.pyqtSignal(str, object)
    failed_job = QtCore.pyqtSignal(str, str)
    # Sensor samples in batches rather than one signal per frame: the IMU
    # arrives 800 times a second and a cross-thread signal each time would
    # spend more on Qt than on the plot.
    samples = QtCore.pyqtSignal(object)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.jobs = queue.Queue()
        self._stop = False
        # Recording state, touched only from this thread: the start and
        # stop go through the job queue like everything else, so there is
        # no lock and no window in which a frame lands in a closed file.
        self._rec = None
        self._rec_path = None
        self._rec_bytes = 0
        self._rec_frames = 0
        self._rec_t0 = 0.0
        self._rec_resync0 = 0

    def submit(self, tag, fn):
        self.jobs.put((tag, fn))

    def stop(self):
        self._stop = True
        self.jobs.put((None, None))

    def run(self):
        try:
            link = cfgtool.CfgLink.open_serial(self.port, self.baud)
        except BaseException as exc:                      # noqa: BLE001
            self.disconnected.emit(str(exc))
            return
        self.connected.emit(self.port)
        try:
            while not self._stop:
                try:
                    tag, fn = self.jobs.get(timeout=0.02)
                except queue.Empty:
                    # Idle: read the sensor stream. One port, two
                    # consumers -- a configuration exchange and a live
                    # view -- and this is where they take turns. A
                    # request drops what is buffered (CfgLink.request), so
                    # the plots show a short gap while a read happens,
                    # which is the honest cost of sharing the link.
                    self._pump(link)
                    continue
                if tag is None:
                    break
                try:
                    self.finished_job.emit(tag, fn(link))
                except BaseException as exc:              # noqa: BLE001
                    # A refusal from the board and a bug in this file look
                    # the same from here, so the traceback goes to the
                    # console while the window gets the message.
                    traceback.print_exc()
                    self.failed_job.emit(tag, str(exc) or exc.__class__.__name__)
        finally:
            link.close()
            self.disconnected.emit("")

    # -- recording ---------------------------------------------------------

    def record_start(self, path):
        self.submit("record", lambda link: self._rec_open(path, link))

    def record_stop(self):
        self.submit("record", lambda link: self._rec_close(link))

    def _rec_open(self, path, link):
        self._rec_close(link)
        self._rec = open(path, "wb")
        self._rec_path = path
        self._rec_bytes = self._rec_frames = 0
        self._rec_t0 = time.monotonic()
        self._rec_resync0 = link.framer.n_resync
        return self.record_state(link)

    def _rec_close(self, link):
        if self._rec:
            self._rec.close()
            self._rec = None
        return self.record_state(link)

    def record_state(self, link):
        return {
            "on": self._rec is not None,
            "path": self._rec_path,
            "bytes": self._rec_bytes,
            "frames": self._rec_frames,
            "seconds": time.monotonic() - self._rec_t0 if self._rec else 0.0,
            # Bytes the framer threw away because they did not resolve
            # into a frame. Reported rather than hidden: a recording is
            # only worth having if it says where it is incomplete.
            "resync": link.framer.n_resync - self._rec_resync0,
        }

    def _pump(self, link):
        """Decode whatever is on the link into one batch.

        Decoding happens HERE and not in the window: at 800 Hz the
        difference between handing over parsed numbers and handing over
        raw frames is the difference between a plot that keeps up and one
        that does not."""
        try:
            link.poll()
        except BaseException:                             # noqa: BLE001
            return
        if not link.pending:
            return
        batch = {"imu": [], "mag": [], "baro": [], "satur": [], "pvt": [],
                 "timesync": [], "nav": None, "status": None, "health": None,
                 # Every framed byte in this read, whether or not the
                 # window has a use for the message inside it. That is
                 # what makes it a LINK figure: the throughput a tab
                 # derived from the frames it happens to plot would go
                 # down when a message is switched off that it never drew.
                 "bytes": 0, "t": time.monotonic()}
        for cls, mid, payload, raw in link.pending:
            batch["bytes"] += len(raw)
            if self._rec:
                # Every checksum-valid frame, in order, byte for byte -
                # the receiver's own messages included, because a
                # recording that dropped them could not be replayed.
                self._rec.write(raw)
                self._rec_bytes += len(raw)
                self._rec_frames += 1
            if cls == ux.CLASS_NAV and mid == ux.ID_NAV_PVT:
                fix = ux.parse_nav_pvt(payload)
                if fix:
                    batch["pvt"].append(fix)
                continue
            if cls != ux.CLS_INSLIB:
                continue
            if mid == ux.ID_IMU and len(payload) == ux.IMU_LEN:
                v = struct.unpack(ux.IMU_FMT, payload)
                batch["imu"].append((v[0], v[1:4], v[4:7], v[7], v[8]))
            elif mid == ux.ID_MAG and len(payload) == ux.MAG_LEN:
                batch["mag"].append(struct.unpack(ux.MAG_FMT, payload))
            elif mid == ux.ID_BARO and len(payload) == ux.BARO_LEN:
                batch["baro"].append(struct.unpack(ux.BARO_FMT, payload))
            elif mid == ux.ID_NAV:
                # Only the newest of a batch is kept: this is a state and
                # not an event, so an older one in the same read has
                # already been superseded.
                batch["nav"] = ux.parse_nav(payload) or batch["nav"]
            elif mid == ux.ID_STATUS:
                batch["status"] = ux.parse_status(payload) or batch["status"]
            elif mid == ux.ID_IMUHEALTH:
                batch["health"] = ux.parse_imuhealth(payload) or batch["health"]
            elif mid == ux.ID_TIMESYNC:
                # One per time pulse, and every one is an event: the pulse
                # count in it is what later says whether a pair slipped a
                # whole second, and a dropped edge leaves a gap in it.
                ts = ux.parse_timesync(payload)
                if ts:
                    batch["timesync"].append(ts)
            elif mid == ux.ID_SATUR:
                # Episodes ARE events, so every one is kept: each names a
                # run of samples that a recording has to treat as
                # unusable, and dropping one loses that run for good.
                ep = ux.parse_satur(payload)
                if ep:
                    batch["satur"].append(ep)
        link.pending.clear()
        if self._rec:
            self._rec.flush()
        if any(batch[k] for k in ("imu", "mag", "baro", "satur", "pvt",
                                  "timesync")) or                 batch["nav"] or batch["status"] or batch["health"]:
            batch["record"] = self.record_state(link)
            self.samples.emit(batch)


# ===========================================================================
# Field descriptors
# ===========================================================================

class Scalar:
    """One scalar configuration key and how to show it.

    `kind` is presentation only. The permitted RANGE is not stated here on
    purpose: the firmware's descriptor table owns it, and a copy in a GUI
    is a copy that drifts. The spin boxes are given room and the board
    does the refusing.
    """

    def __init__(self, key_name, label, kind, unit="", hint="", choices=None):
        self.key_name = key_name
        self.key = ux.CFG_KEYS[key_name]
        self.label = label
        self.kind = kind          # "bool", "int" or "choice"
        self.unit = unit
        self.hint = hint
        # "choice" only: [(value, label), ...]. A key whose permitted
        # values are a short list the firmware enumerates - a sensor rate,
        # a filter divider - is a list here too. A spin box would let
        # somebody type a number between two of them and learn from a
        # refused write that it does not exist.
        self.choices = choices or []
        self.widget = None
        self.label_widget = None
        self.mark = None          # the "differs from default" note
        self.differs = None       # None until the board has been read
        self.read_value = None    # what the board last reported
        self.default_value = None
        # Whether the firmware on the bench knows this key at all. A row
        # for a key it has never heard of is hidden AND excluded from the
        # write: sending one would have the board refuse the whole batch
        # at that key, taking every legitimate change after it down too.
        self.known = True


GROUPS = [
    ("Message output", [
        Scalar("CFG-MSGOUT-IMU", "IMU samples", "bool", "",
               "One frame per IMU sample, 0x40/0x01. This is 90 % of the "
               "link on its own."),
        Scalar("CFG-MSGOUT-BARO", "Barometer", "bool"),
        Scalar("CFG-MSGOUT-MAG", "Magnetometer", "bool"),
        Scalar("CFG-MSGOUT-STATUS", "Diagnostic counters", "bool", "",
               "0x40/0x04 every 5 s: dropped frames, IMU overruns, UART "
               "errors."),
        Scalar("CFG-MSGOUT-CFGINFO", "Configuration state", "bool", "",
               "0x40/0x07. Switching this off makes a recording unable to "
               "say which calibration it was taken with."),
        Scalar("CFG-MSGOUT-NAV", "Navigation solution", "bool", "",
               "0x40/0x0F, the filter's own attitude, velocity and position. "
               "The one message a recording cannot reconstruct afterwards."),
    ]),
    ("Rates", [
        Scalar("CFG-RATE-MAG_MS", "Magnetometer period", "int", "ms"),
        Scalar("CFG-RATE-CFGINFO_MS", "Configuration state period", "int", "ms"),
        Scalar("CFG-RATE-NAV_MS", "Navigation solution period", "int", "ms",
               "5 ms is 200 Hz and the fastest the board can pace, 100 ms "
               "is the rate it ships at. Anything below about 20 ms wants "
               "the IMU rate halved to pay for the link, which is what the "
               "output mode buttons above do as a pair."),
    ]),
    ("Sensor chain", [
        Scalar("CFG-IMU-ODR_HZ", "IMU output data rate", "choice", "",
               "The rate the sensor produces samples at, which is also the "
               "rate the filter propagates at: it runs one epoch per "
               "sample. Halving it halves the link the IMU stream needs.",
               choices=[(800, "800 Hz"), (400, "400 Hz")]),
        Scalar("CFG-IMU-LPF_DIV", "IMU low pass", "choice", "",
               "Corner of the sensor's own low pass, as a divider of the "
               "rate above. Stored as the divider because that is what the "
               "register holds: the corner follows the rate on its own.",
               choices=[(4, "ODR/4"), (8, "ODR/8"), (16, "ODR/16"),
                        (32, "ODR/32"), (64, "ODR/64"), (128, "ODR/128")]),
    ]),
    ("Calibration", [
        Scalar("CFG-IMU-APPLY_CAL", "Apply IMU calibration", "bool", "",
               "Clear this for a calibration recording, otherwise the "
               "solver fits a correction on top of a corrected stream. "
               "Without --flash it is back at the next power cycle."),
        Scalar("CFG-IMU-TEMP_TAU_MS", "IMU temperature filter", "int", "ms",
               "Time constant of the low pass on the die temperature that "
               "drives the correction."),
        Scalar("CFG-MAG-APPLY_CAL", "Apply magnetometer calibration", "bool"),
        Scalar("CFG-MAG-TEMP_TAU_MS", "Magnetometer temperature filter", "int", "ms"),
    ]),
]



# What a group of rows does NOT cover is as easy to misread as what it
# does, and the row labels have no room to say it.
GROUP_NOTES = {
    "Message output": "One switch per message: whether the board SENDS it, "
                      "not whether it is produced. The filter is fed every "
                      "IMU sample either way, so clearing IMU samples stops "
                      "the sensor plots in this window and changes nothing "
                      "about what the box computes. It is also most of the "
                      "link, which is what it is for.",
    "Rates": "Message PERIODS, in milliseconds: how often the board SENDS "
             "one, not how fast anything is sampled. The IMU has no period "
             "here because its samples go out as they are taken - its rate "
             "is under Sensor chain below, and the only choice here is "
             "whether the message is sent at all.",
    "Sensor chain": "What the IMU itself is set to, which is the one part "
                    "of this window that changes what the box COMPUTES "
                    "rather than what it sends. The filter runs one epoch "
                    "per sample and integrates whatever the low pass "
                    "passes, so both of these are in the answer and not "
                    "just in the picture of it. The frequency the two work "
                    "out to is under Stored image, on the imu line.",
    "Calibration": "Whether the STORED table is applied to the stream, and "
                   "how fast the temperature that drives it is filtered. "
                   "The table itself is on the Calibration tab.",
}


# The two settings a fast navigation output has to move together, and the
# reason they are offered as a pair rather than as two independent rows.
#
# One VCP carries everything. An IMU frame is 42 bytes, so the stream is
# 33.6 kB/s at 800 Hz and 16.8 at 400. A navigation frame is 128 bytes, so
# it is 1.3 kB/s at 10 Hz and 25.6 at 200. The link runs 92.2 kB/s at
# 921600 8N1 and the GNSS stream still wants its share, which leaves no
# room for 200 Hz on top of a full rate IMU: what pays for the solution is
# exactly the half of the IMU stream that is given up.
#
# The trade is real and worth stating: at 400 Hz the filter propagates half
# as often and the sensor's low pass halves with the rate. What it buys is
# a solution sampled fast enough to be compared against something external
# rather than watched.
OUTPUT_MODES = (
    ("Standard rate",
     (("CFG-IMU-ODR_HZ", 800), ("CFG-RATE-NAV_MS", 100)),
     "IMU 800 Hz, navigation solution 10 Hz. The rate the board ships at."),
    ("High rate",
     (("CFG-IMU-ODR_HZ", 400), ("CFG-RATE-NAV_MS", 5)),
     "Navigation solution 200 Hz, IMU halved to 400 Hz to pay for the "
     "link. The filter then propagates at 400 Hz as well."),
)


# ===========================================================================
# Configuration tab
# ===========================================================================

class ConfigTab(QtWidgets.QWidget):

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.scalars = []
        self.lever_read = None        # (x, y, z) or None for unset
        self.housing_read = None      # row-major 3x3 or None
        # What the spin boxes were SHOWING after the last read. "Changed"
        # has to mean "changed by the person at the keyboard", not
        # "differs from the stored bytes" -- the two are not the same:
        #   - an unset lever arm shows as 0.000, and writing that back
        #     would turn "nobody has measured the antenna" into "somebody
        #     measured it and it is zero", which is the exact distinction
        #     the firmware and the device screen go to trouble to keep;
        #   - a rotation stored as f32 and displayed to three decimals
        #     never compares equal to itself, so every Apply would rewrite
        #     the mounting with a slightly different one.
        self.lever_base = None
        self.housing_base = None
        self._build()

    # -- construction ------------------------------------------------------

    def _build(self):
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        left = QtWidgets.QVBoxLayout()
        right = QtWidgets.QVBoxLayout()
        outer.addLayout(left, 3)
        outer.addLayout(right, 4)

        left.addWidget(self._caption(
            "Type a value, then Apply below. Apply (live) takes effect at "
            "once and is gone at the next power cycle; Apply && store also "
            "writes it to flash. Only the rows somebody CHANGED are sent, "
            "so leaving this tab open changes nothing on the board."))
        # The rows are built first because the mode buttons above them
        # drive those rows and have to be able to find them.
        boxes = [self._scalar_group(title, items) for title, items in GROUPS]
        self.by_name = {sc.key_name: sc for sc in self.scalars}
        left.addWidget(self._mode_group())
        for box in boxes:
            left.addWidget(box)
        left.addStretch(1)

        odr = self.by_name.get("CFG-IMU-ODR_HZ")
        if odr is not None:
            odr.widget.currentIndexChanged.connect(self._refresh_lpf_labels)
        for name in ("CFG-IMU-ODR_HZ", "CFG-RATE-NAV_MS"):
            sc = self.by_name.get(name)
            if sc is None:
                continue
            signal = (sc.widget.currentIndexChanged if sc.kind == "choice"
                      else sc.widget.valueChanged)
            signal.connect(self._show_mode)
        self._refresh_lpf_labels()

        right.addWidget(self._frame_group())
        right.addWidget(self._store_group())
        right.addStretch(1)
        themed(self._restyle_marks)

    def _scalar_group(self, title, items):
        box = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(box)
        grid.setColumnStretch(1, 1)
        self._grids = getattr(self, "_grids", {})
        self._grids[title] = grid
        for s in items:
            self._add_scalar(grid, s)
        if title in GROUP_NOTES:
            grid.addWidget(self._caption(GROUP_NOTES[title]),
                           grid.rowCount(), 0, 1, 3)
        return box

    def _add_scalar(self, grid, s):
        row = grid.rowCount()
        name = QtWidgets.QLabel(s.label)
        name.setToolTip(s.hint or s.key_name)
        s.label_widget = name
        grid.addWidget(name, row, 0)

        if s.kind == "bool":
            w = QtWidgets.QCheckBox()
        elif s.kind == "choice":
            w = QtWidgets.QComboBox()
            for value, text in s.choices:
                w.addItem(text, value)
        else:
            w = QtWidgets.QSpinBox()
            # Wide on purpose. The board owns the real limits and says no;
            # a range restated here would be a second, drifting authority.
            w.setRange(0, 2 ** 31 - 1)
            w.setSuffix(" " + s.unit if s.unit else "")
            w.setGroupSeparatorShown(True)
        w.setToolTip(s.hint or s.key_name)
        w.setEnabled(False)
        s.widget = w
        grid.addWidget(w, row, 1)

        mark = QtWidgets.QLabel("")
        mark.setFixedWidth(90)
        themed(lambda w=mark: w.setStyleSheet(dim() + " font-size: 11px;"))
        s.mark = mark
        grid.addWidget(mark, row, 2)

        self.scalars.append(s)

    def _mode_group(self):
        box = QtWidgets.QGroupBox("Output mode")
        v = QtWidgets.QVBoxLayout(box)
        v.addWidget(self._caption(
            "Two settings that only make sense together: a navigation "
            "solution at 200 Hz needs 25.6 kB/s and the link does not have "
            "that on top of an 800 Hz IMU stream. These buttons fill in the "
            "IMU rate and the solution period below as a pair - nothing is "
            "sent until Apply."))
        row = QtWidgets.QHBoxLayout()
        self.mode_buttons = []
        for name, settings, note in OUTPUT_MODES:
            b = QtWidgets.QPushButton(name)
            b.setEnabled(False)
            b.setToolTip(note)
            b.clicked.connect(lambda _checked=False, st=settings:
                              self._set_mode(st))
            self.mode_buttons.append(b)
            row.addWidget(b)
        v.addLayout(row)
        self.mode_state = QtWidgets.QLabel("")
        self.mode_state.setWordWrap(True)
        themed(lambda: self.mode_state.setStyleSheet(
            dim() + " font-size: 11px;"))
        v.addWidget(self.mode_state)
        return box

    def _mode_of(self, values):
        """Name of the mode a {key_name: value} mapping is, or None.

        None is a real answer and not a failure: every value in between is
        reachable from the rows below, and a window that called those
        "standard rate" would be describing a board that is not."""
        for name, settings, _note in OUTPUT_MODES:
            if all(values.get(k) == v for k, v in settings):
                return name
        return None

    def _shown_modes(self):
        """(what the widgets show, what the board reported)."""
        shown, board = {}, {}
        for name, settings, _note in OUTPUT_MODES:
            for key_name, _v in settings:
                sc = self.by_name.get(key_name)
                if sc is None or not sc.known:
                    return None, None
                shown[key_name] = self._widget_value(sc)
                board[key_name] = (sc.read_value if sc.read_value is not None
                                   else sc.default_value)
        return self._mode_of(shown), self._mode_of(board)

    def _set_mode(self, settings):
        for key_name, value in settings:
            sc = self.by_name.get(key_name)
            if sc is None or not sc.known:
                continue
            self._set_widget_value(sc, value)
        self._show_mode()

    def _show_mode(self):
        """What the rows currently say, against what the board answered."""
        odr = self.by_name.get("CFG-IMU-ODR_HZ")
        if odr is None or not odr.known:
            # An older firmware without the rate key. The buttons stay off
            # rather than half working: setting the solution period alone
            # is the half of the pair that costs link and does not pay for
            # it, which is the one combination worth refusing.
            for b in self.mode_buttons:
                b.setEnabled(False)
            self.mode_state.setText(
                "this firmware has no IMU rate setting, so the pair cannot "
                "be moved - the solution period below is still yours to set")
            return
        shown, board = self._shown_modes()
        if shown and shown == board:
            self.mode_state.setText("board: %s" % shown.lower())
        elif shown:
            self.mode_state.setText(
                "showing %s, board is %s - Apply below to move it"
                % (shown.lower(), (board or "on neither preset").lower()))
        else:
            self.mode_state.setText(
                "rows are on neither preset%s"
                % ("" if board is None else
                   ", board is %s" % board.lower()))

    def _frame_group(self):
        box = QtWidgets.QGroupBox("Mounting")
        v = QtWidgets.QVBoxLayout(box)

        # -- lever arm
        v.addWidget(self._caption(
            "GNSS antenna lever arm, body frame FRD, metres from the IMU to "
            "the antenna phase centre. An antenna sitting on top of the IMU "
            "has a NEGATIVE down component. Type the three numbers and use "
            "Apply below: they go to the board as ONE record, because three "
            "separate writes leave a window in which the filter runs on two "
            "axes of the new arm and one of the old."))
        lev = QtWidgets.QHBoxLayout()
        self.lever = []
        for label in ("forward", "right", "down"):
            lev.addWidget(QtWidgets.QLabel(label))
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(-1000.0, 1000.0)     # the board bounds it at 100
            sb.setDecimals(3)
            sb.setSingleStep(0.01)
            sb.setSuffix(" m")
            sb.setEnabled(False)
            self.lever.append(sb)
            lev.addWidget(sb, 1)
        v.addLayout(lev)
        self.lever_state = QtWidgets.QLabel("")
        themed(lambda: self.lever_state.setStyleSheet(
            dim() + " font-size: 11px;"))
        v.addWidget(self.lever_state)

        v.addSpacing(8)

        # -- housing rotation
        v.addWidget(self._caption(
            "Housing rotation: where the board sits in its box. Edited as "
            "roll/pitch/yaw, stored as the nine numbers below - those are "
            "what the device uses."))
        hou = QtWidgets.QHBoxLayout()
        self.housing = []
        for label in ("roll", "pitch", "yaw"):
            hou.addWidget(QtWidgets.QLabel(label))
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(-180.0, 180.0)
            sb.setDecimals(3)
            sb.setSingleStep(0.1)
            sb.setSuffix(" deg")
            sb.setEnabled(False)
            sb.valueChanged.connect(self._housing_edited)
            self.housing.append(sb)
            hou.addWidget(sb, 1)
        v.addLayout(hou)

        self.housing_matrix = QtWidgets.QLabel("")
        themed(lambda: self.housing_matrix.setStyleSheet(MONO + dim()))
        v.addWidget(self.housing_matrix)

        self.housing_warn = QtWidgets.QLabel("")
        self.housing_warn.setWordWrap(True)
        themed(lambda: self.housing_warn.setStyleSheet(
            "color: %s; font-size: 11px;" % T["warn"]))
        v.addWidget(self.housing_warn)
        return box

    def _store_group(self):
        box = QtWidgets.QGroupBox("Stored image")
        v = QtWidgets.QVBoxLayout(box)
        self.store_text = QtWidgets.QLabel("not connected")
        self.store_text.setStyleSheet(MONO)
        self.store_text.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        v.addWidget(self.store_text)

        v.addSpacing(6)
        row1 = QtWidgets.QHBoxLayout()
        self.btn_read = QtWidgets.QPushButton("Read from board")
        self.btn_apply = QtWidgets.QPushButton("Apply (live)")
        self.btn_flash = QtWidgets.QPushButton("Apply && store")
        self.btn_flash.setObjectName("primary")
        for b in (self.btn_read, self.btn_apply, self.btn_flash):
            b.setEnabled(False)
            row1.addWidget(b)
        v.addLayout(row1)

        row2 = QtWidgets.QHBoxLayout()
        self.btn_reset = QtWidgets.QPushButton("Drop live values")
        self.btn_factory = QtWidgets.QPushButton("Erase store")
        self.btn_factory.setObjectName("danger")
        for b in (self.btn_reset, self.btn_factory):
            b.setEnabled(False)
            row2.addWidget(b)
        v.addLayout(row2)

        v.addWidget(self._caption(
            "Apply is live only and gone at the next power cycle - which is "
            "what a calibration recording wants. Erase store removes the "
            "calibration with everything else and cannot be undone."))

        self.btn_read.clicked.connect(self.read_all)
        self.btn_apply.clicked.connect(lambda: self.apply(flash=False))
        self.btn_flash.clicked.connect(lambda: self.apply(flash=True))
        self.btn_reset.clicked.connect(lambda: self.reset(flash=False))
        self.btn_factory.clicked.connect(lambda: self.reset(flash=True))
        return box

    @staticmethod
    def _caption(text):
        lab = QtWidgets.QLabel(text)
        lab.setWordWrap(True)
        themed(lambda: lab.setStyleSheet(dim() + " font-size: 11px;"))
        return lab

    # -- board conversation ------------------------------------------------

    def set_online(self, online):
        for s in self.scalars:
            s.widget.setEnabled(online)
        for sb in self.lever + self.housing:
            sb.setEnabled(online)
        for b in (self.btn_read, self.btn_apply, self.btn_flash,
                  self.btn_reset, self.btn_factory):
            b.setEnabled(online)
        for b in self.mode_buttons:
            b.setEnabled(online)
        if not online:
            self.store_text.setText("not connected")
            self.mode_state.setText("")

    def read_all(self):
        self.console.submit("read", self._job_read)

    @staticmethod
    def _job_read(link):
        """Everything the tab shows, in one trip.

        The live values and the compiled-in defaults are both read, so the
        window can say which settings this unit has been moved away from -
        the first question to ask about a box that behaves unlike its
        siblings."""
        groups = [ux.group_wildcard(g) for g in
                  (ux.GRP_MSGOUT, ux.GRP_RATE, ux.GRP_IMU, ux.GRP_MAG,
                   ux.GRP_FRAME)]
        live = link.valget(groups, ux.VALGET_RAM)
        try:
            default = link.valget(groups, ux.VALGET_DEFAULT)
        except BaseException:                             # noqa: BLE001
            default = {}          # older firmware, or nothing has defaults
        return {"live": live, "default": default, "info": link.info()}

    def on_read(self, data):
        live, default = data["live"], data["default"]

        for s in self.scalars:
            if s.key in live:
                s.read_value = ux.scalar_from_bytes(live[s.key])
            else:
                s.read_value = None
            s.default_value = (ux.scalar_from_bytes(default[s.key])
                               if s.key in default else None)
            # Every key in this window has a compiled-in default, so one
            # missing from the DEFAULT layer is a key this firmware has
            # never heard of. Hidden rather than shown as a dead control:
            # the tool is pointed at whatever is on the bench, and an
            # older board is not a broken one.
            s.known = (not default) or (s.default_value is not None)
            s.widget.setVisible(s.known)
            s.mark.setVisible(s.known)
            s.label_widget.setVisible(s.known)
            if s.known:
                self._show_scalar(s)

        self._refresh_lpf_labels()
        self._show_mode()

        raw = live.get(ux.leverarm_key())
        self.lever_read = ux.unpack_leverarm(raw) if raw else None
        for sb, v in zip(self.lever, self.lever_read or (0.0, 0.0, 0.0)):
            sb.blockSignals(True)
            sb.setValue(v)
            sb.blockSignals(False)
        self.lever_base = [sb.value() for sb in self.lever]
        if raw is None:
            self.lever_state.setText(
                "unset - the antenna is assumed to sit on the IMU. That is a "
                "real assumption, not a neutral one. Type a value to store "
                "one; leaving these at zero leaves the record absent.")
        elif self.lever_read is None:
            self.lever_state.setText("stored record is unreadable")
        else:
            self.lever_state.setText("configured")

        raw = live.get(ux.housing_key())
        self.housing_read = ux.unpack_housing(raw) if raw else None
        rpy = (ux.housing_rpy_deg(self.housing_read)
               if self.housing_read else (0.0, 0.0, 0.0))
        for sb, v in zip(self.housing, rpy):
            sb.blockSignals(True)
            sb.setValue(v)
            sb.blockSignals(False)
        self.housing_base = [sb.value() for sb in self.housing]
        self._housing_edited()

        self._show_info(data["info"])

    @staticmethod
    def _widget_value(s):
        if s.kind == "bool":
            return int(s.widget.isChecked())
        if s.kind == "choice":
            return s.widget.currentData()
        return int(s.widget.value())

    @staticmethod
    def _set_widget_value(s, value):
        w = s.widget
        if s.kind == "bool":
            w.setChecked(bool(value))
        elif s.kind == "choice":
            i = w.findData(int(value))
            if i < 0:
                # A value this window has no name for. Shown as the bare
                # number rather than snapped to a neighbour: the board is
                # the authority on what it accepts, and a silently changed
                # setting is the one thing a configuration tool must not do.
                w.addItem(str(int(value)), int(value))
                i = w.count() - 1
            w.setCurrentIndex(i)
        else:
            w.setValue(int(value))

    def _value_text(self, s, value):
        """A value as the row shows it, for the default marker."""
        if s.kind == "choice":
            for v, text in s.choices:
                if v == value:
                    return text
        return "%d" % value

    def _refresh_lpf_labels(self):
        """Put the corner frequency next to each low pass divider.

        The divider is what gets stored, but the number a person is after
        is the frequency, and it takes BOTH settings to know it. Reading it
        off the rate row keeps the two in step while the rate is being
        chosen, before anything has been sent."""
        lpf = self.by_name.get("CFG-IMU-LPF_DIV")
        odr = self.by_name.get("CFG-IMU-ODR_HZ")
        if lpf is None or odr is None:
            return
        rate = self._widget_value(odr) if odr.known else None
        for i in range(lpf.widget.count()):
            div = lpf.widget.itemData(i)
            if not div:
                continue
            lpf.widget.setItemText(
                i, "ODR/%d" % div + ("" if not rate
                                     else "   %.4g Hz" % (rate / float(div))))

    def _show_scalar(self, s):
        w = s.widget
        w.blockSignals(True)
        if s.read_value is None:
            # Not in the image: the board is using its compiled-in default,
            # which is what the widget then shows.
            value = s.default_value or 0
        else:
            value = s.read_value
        self._set_widget_value(s, value)
        w.blockSignals(False)

        if s.default_value is None:
            s.differs = None
            s.mark.setText("")
        elif value != s.default_value:
            s.differs = True
            s.mark.setText("default %s"
                           % self._value_text(s, s.default_value))
        else:
            s.differs = False
            s.mark.setText("default")
        self._style_mark(s)

    def _style_mark(self, s):
        s.mark.setStyleSheet(
            ("color: %s; font-size: 11px;" % T["changed"]) if s.differs
            else dim() + " font-size: 11px;")

    def _restyle_marks(self):
        for s in self.scalars:
            if s.mark is not None:
                self._style_mark(s)

    def _show_info(self, info):
        if not info:
            self.store_text.setText("no configuration state frame "
                                    "(CFG-MSGOUT-CFGINFO off?)")
            return
        cal = info["cal_flags"]
        lines = [
            "image     crc %08X   slot %d" % (info["cfg_crc"], info["cfg_seq"]),
            "state     %s%s" % (
                "stored" if info["stored"] else "NOTHING STORED",
                ", unsaved changes" if info["unsaved"] else ""),
            "apply     imu %s   mag %s" % (
                "on" if info["imu_cal_applied"] else "OFF",
                "on" if info["mag_cal_applied"] else "OFF"),
            "nodes     acc %d  gyr %d  mag %d" % (
                info["npts"]["acc"], info["npts"]["gyr"], info["npts"]["mag"]),
            "temp      imu %.1f C   mag %.1f C" % (
                info["temp_imu_c"], info["temp_mag_c"]),
        ]
        if cal & ux.CAL_HOUSING_BAD:
            lines.append("housing   STORED BUT REFUSED, not a rotation")
        elif cal & ux.CAL_HOUSING:
            lines.append("housing   applied")
        if "imu" in info:
            imu = info["imu"]
            lines.append("imu       %d Hz   +-%.4g g / +-%.4g dps   lpf %.4g Hz"
                         % (imu["odr_hz"], imu["accel_fs_g"],
                            imu["gyro_fs_dps"], imu["gyro_lpf_hz"]))
        self.store_text.setText("\n".join(lines))

    # -- housing helpers ---------------------------------------------------

    def _housing_edited(self):
        rpy = [sb.value() for sb in self.housing]
        rows = ux.housing_from_rpy_deg(*rpy)
        self.housing_matrix.setText("\n".join(
            "  [%s]" % "  ".join("%+9.6f" % v for v in row) for row in rows))
        # ZYX Euler angles are degenerate at +-90 degrees of pitch: roll and
        # yaw stop being separable there and only their difference survives.
        # A box mounted on its side really does sit at that pitch, so the
        # case is not academic and the window says so rather than quietly
        # storing a rotation that reads back as different angles.
        if abs(rpy[1]) > 85.0:
            self.housing_warn.setText(
                "pitch is near +-90 deg: roll and yaw are no longer separable "
                "there, so these three numbers will not read back unchanged. "
                "The nine above are the rotation that gets stored, and they "
                "are correct.")
        else:
            self.housing_warn.setText("")

    # -- writing -----------------------------------------------------------

    def _pending(self):
        """(key, value) for everything the user changed, and a description.

        Only differences are sent. Rewriting a whole configuration to
        change one number is how a stored calibration disappears."""
        items, what = [], []
        for s in self.scalars:
            if not s.known:
                continue
            value = self._widget_value(s)
            shown = s.read_value if s.read_value is not None else s.default_value
            if shown is None or value != shown:
                items.append((s.key, value))
                what.append("%s = %d" % (s.key_name, value))

        lever = tuple(sb.value() for sb in self.lever)
        if self.lever_base is not None and list(lever) != self.lever_base:
            items.append((ux.leverarm_key(), ux.pack_leverarm(lever)))
            what.append("CFG-FRAME-LEVERARM = %.3f %.3f %.3f m" % lever)

        rpy = tuple(sb.value() for sb in self.housing)
        if self.housing_base is not None and list(rpy) != self.housing_base:
            items.append((ux.housing_key(),
                          ux.pack_housing(ux.housing_from_rpy_deg(*rpy))))
            what.append("CFG-FRAME-HOUSING = roll %.3f pitch %.3f yaw %.3f deg"
                        % rpy)
        return items, what

    def apply(self, flash):
        try:
            items, what = self._pending()
        except ValueError as exc:
            # pack_leverarm refuses a units mistake before it reaches the
            # board, where the message can still say what the units are.
            self.console.status(str(exc), "bad")
            return
        if not items:
            self.console.status("nothing changed", "idle")
            return
        if flash and not self._confirm(
                "Store %d change(s) in the board's flash?\n\n%s"
                % (len(items), "\n".join(what))):
            return
        layers = ux.LAYER_RAM | (ux.LAYER_FLASH if flash else 0)
        self.console.submit(
            "apply", lambda link: link.valset(items, layers))

    def reset(self, flash):
        if flash:
            ok = self._confirm(
                "Erase the stored image?\n\nThis removes the CALIBRATION "
                "with everything else and cannot be undone.")
        else:
            ok = self._confirm(
                "Drop the live values back to the defaults?\n\nThe stored "
                "image is untouched and returns at the next power cycle. "
                "Until then, do not use Apply & store on anything: it would "
                "write the emptied image over the calibration.")
        if not ok:
            return
        layers = ux.LAYER_RAM | (ux.LAYER_FLASH if flash else 0)
        self.console.submit("reset", lambda link: link.reset(layers))

    def _confirm(self, text):
        m = QtWidgets.QMessageBox(self)
        m.setWindowTitle("Confirm")
        m.setText(text)
        m.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        m.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Yes |
                             QtWidgets.QMessageBox.StandardButton.Cancel)
        m.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        return m.exec() == QtWidgets.QMessageBox.StandardButton.Yes


# ===========================================================================
# Live history
# ===========================================================================

class Ring:
    """Fixed length history, oldest overwritten.

    A list that gets trimmed reallocates on every sample; at 800 Hz that
    is most of what this tab would cost. This does not.
    """

    def __init__(self, n, width):
        self.buf = np.zeros((n, width), dtype=np.float32)
        self.t = np.zeros(n, dtype=np.float64)
        self.n = n
        self.count = 0
        self.head = 0

    def add(self, t, values):
        self.t[self.head] = t
        self.buf[self.head] = values
        self.head = (self.head + 1) % self.n
        self.count = min(self.count + 1, self.n)

    def view(self):
        """(t, values) oldest first, or (None, None) while still empty."""
        if self.count == 0:
            return None, None
        if self.count < self.n:
            return self.t[:self.count], self.buf[:self.count]
        idx = np.r_[self.head:self.n, 0:self.head]
        return self.t[idx], self.buf[idx]


def window(t, vals, seconds):
    """The last `seconds` of a ring view, restamped as seconds before now.

    "Now" is the newest SAMPLE and not the wall clock, so a stream that
    has stopped leaves its last trace where it ended instead of sliding
    off the left edge as if it were still arriving.
    """
    keep = t >= t[-1] - seconds
    return t[keep] - t[-1], vals[keep]


def break_wraps(y, limit=180.0):
    """A heading series with its fold points blanked.

    A step of more than `limit` between two samples is the +-180 degree
    fold and not motion. The sample after it is blanked so that a curve
    drawn with connect="finite" lifts the pen there, instead of ruling a
    line across the whole plot that says the platform spun round.
    """
    out = np.asarray(y, dtype=np.float64).copy()
    if out.size > 1:
        out[1:][np.abs(np.diff(out)) > limit] = np.nan
    return out




# ===========================================================================
# Statistics, spectra, and the shapes they get drawn as
# ===========================================================================

def ring_window(ring, seconds):
    """The last `seconds` of a ring, or (None, None) while it is empty.

    The pair every plotted tab wants, in one place: a ring nobody has fed
    yet has no last sample to measure the look-back against, and that is
    a "nothing to draw" rather than an exception at the first index.
    """
    t, vals = ring.view()
    if t is None:
        return None, None
    return window(t, vals, seconds)


def sample_rate(t):
    """Hz from a timestamp vector, or 0.0 when it cannot be told."""
    if t is None or len(t) < 2:
        return 0.0
    span = float(t[-1] - t[0])
    return (len(t) - 1) / span if span > 0.0 else 0.0


# A covariance ellipse is not the contour most people assume it is.
# Scaled by 1 sigma it holds about 39 % of a bivariate normal sample and
# not 68 %: the 68 % of the one-dimensional rule is spent on ONE axis,
# and asking for two at once costs the rest. The 95 % contour needs
# sqrt(chi2(2, 0.95)) = 2.4477, and both are drawn precisely so that the
# difference is visible rather than assumed.
ELLIPSE_K95 = 2.4477


def cov_ellipse(x, y, k=1.0, points=72):
    """The k-scaled covariance ellipse of a 2D sample set.

    Returns (ex, ey, info), or (None, None, None) when there are too few
    points for a covariance to mean anything. It comes out of the SAMPLE
    covariance and not out of any filter: it says how the points that
    arrived are spread, which is a measurement of the last few minutes
    and not a prediction about the next one.
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 8:
        return None, None, None
    cx, cy = float(x.mean()), float(y.mean())
    cov = np.cov(np.vstack((x, y)))
    if not np.all(np.isfinite(cov)):
        return None, None, None
    # eigh and not eig: the matrix is symmetric, and eigh returns real
    # eigenvalues in ascending order, which is what makes the second one
    # the major axis without a sort.
    vals, vecs = np.linalg.eigh(cov)
    vals = np.clip(vals, 0.0, None)
    major, minor = math.sqrt(vals[1]), math.sqrt(vals[0])
    ang = math.atan2(vecs[1, 1], vecs[0, 1])
    th = np.linspace(0.0, 2.0 * math.pi, points)
    ca, sa = math.cos(ang), math.sin(ang)
    ex = cx + k * (major * np.cos(th) * ca - minor * np.sin(th) * sa)
    ey = cy + k * (major * np.cos(th) * sa + minor * np.sin(th) * ca)
    info = {
        "n": int(x.size), "cx": cx, "cy": cy,
        "major": major, "minor": minor, "angle_deg": math.degrees(ang),
        # DRMS is the radius that lands in every GNSS data sheet, so it
        # is reported as well: a number from this window can then be held
        # against one from a specification without somebody converting
        # the wrong way round on the way.
        "drms": math.hypot(float(x.std()), float(y.std())),
        "rms": float(np.sqrt(np.mean(x * x + y * y))),
        "p95": float(np.percentile(np.hypot(x - cx, y - cy), 95.0)),
    }
    return ex, ey, info


def circle_pts(cx, cy, r, points=72):
    """A circle as a polyline, for an accuracy radius drawn on a plot."""
    th = np.linspace(0.0, 2.0 * math.pi, points)
    return cx + r * np.cos(th), cy + r * np.sin(th)


def spectrum(x, fs, nseg=1024, max_samples=16384):
    """Amplitude spectral density of one channel, Welch style.

    Returns (f, asd) in units per sqrt(Hz), or (None, None) when there is
    not enough of it. A density and not a plain FFT magnitude, because
    the number then does not depend on how long a window happened to be
    averaged over, and two runs of different length can be compared.

    The mean goes out per segment: a gyro bias, or the 1 g an
    accelerometer axis carries while it is pointing down, is a real part
    of the signal but it is not vibration, and left in it buries the
    first few bins under a spike at DC.
    """
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    if fs <= 0.0 or x.size < 2 * nseg:
        return None, None
    if x.size > max_samples:
        x = x[-max_samples:]
    w = np.hanning(nseg)
    scale = 1.0 / (fs * np.sum(w * w))
    acc = np.zeros(nseg // 2 + 1)
    n = 0
    for start in range(0, x.size - nseg + 1, nseg // 2):
        seg = x[start:start + nseg]
        f = np.fft.rfft((seg - seg.mean()) * w)
        acc += f.real * f.real + f.imag * f.imag
        n += 1
    psd = acc / n * scale
    # One sided: every bin but DC and Nyquist carries its mirror image's
    # power as well.
    psd[1:-1] *= 2.0
    return np.fft.rfftfreq(nseg, 1.0 / fs), np.sqrt(psd)


# ===========================================================================
# Small instruments
# ===========================================================================

class StatTile(QtWidgets.QFrame):
    """One headline number, with a caption over it and a note under it.

    For the overview, where the question is what this box is doing and
    the answer has to survive being read from the other side of a bench.
    A field in a grid of twenty is not that, however exact it is.
    """

    def __init__(self, caption, hint=""):
        super().__init__()
        self.colour = "idle"
        if hint:
            self.setToolTip(hint)
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 6, 10, 6)
        v.setSpacing(1)
        self.cap = QtWidgets.QLabel(caption)
        self.val = QtWidgets.QLabel("--")
        self.sub = QtWidgets.QLabel("")
        for w in (self.cap, self.val, self.sub):
            v.addWidget(w)
        themed(self._restyle)

    def show_value(self, text, sub="", colour=None):
        self.val.setText(text)
        self.sub.setText(sub)
        self.colour = colour or "text"
        self._restyle()

    def clear(self):
        self.show_value("--", "", "idle")

    def _restyle(self):
        self.setStyleSheet("QFrame { background: %s; border: 1px solid %s; "
                           "border-radius: 6px; }" % (T["panel"], T["edge"]))
        self.cap.setStyleSheet("border: none; font-size: 10px; color: %s;"
                               % T["dim"])
        self.val.setStyleSheet("border: none; " + MONO
                               + " font-size: 18px; color: %s;" % T[self.colour])
        self.sub.setStyleSheet("border: none; font-size: 10px; color: %s;"
                               % T["dim"])


class HorizonWidget(QtWidgets.QWidget):
    """Roll and pitch as the instrument they are read off in a cockpit.

    The numbers are next to it and they are the exact answer. This is for
    the other question - is the box level, is it upside down, is the
    attitude moving the way the platform is - which a pair of digits
    walking about answers slowly and a picture answers at a glance.

    Nothing is drawn when there is no attitude. An artificial horizon
    holding its last picture is the one instrument failure that looks
    exactly like level flight.
    """

    SPAN_DEG = 45.0        # pitch from the centre of the disc to its rim

    def __init__(self):
        super().__init__()
        self.setMinimumSize(190, 190)
        self.rpy = None
        themed(self.update)

    def set_attitude(self, rpy):
        if rpy != self.rpy:
            self.rpy = rpy
            self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2.0 - 12.0
        cx, cy = w / 2.0, h / 2.0
        disc = QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r)

        if self.rpy is None:
            p.setBrush(QtGui.QColor(T["field"]))
            p.setPen(QtGui.QPen(QtGui.QColor(T["line"])))
            p.drawEllipse(disc)
            p.setPen(QtGui.QColor(T["dim"]))
            p.drawText(disc, QtCore.Qt.AlignmentFlag.AlignCenter, "no attitude")
            return

        roll, pitch = self.rpy[0], self.rpy[1]
        ppd = r / self.SPAN_DEG
        big = 3.0 * r
        p.save()
        clip = QtGui.QPainterPath()
        clip.addEllipse(disc)
        p.setClipPath(clip)
        p.translate(cx, cy)
        p.rotate(-roll)
        p.translate(0.0, pitch * ppd)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(T["sky"]))
        p.drawRect(QtCore.QRectF(-big, -big, 2 * big, big))
        p.setBrush(QtGui.QColor(T["ground"]))
        p.drawRect(QtCore.QRectF(-big, 0.0, 2 * big, big))
        pen = QtGui.QPen(QtGui.QColor(T["strong"]))
        pen.setWidthF(1.4)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(-big, 0.0), QtCore.QPointF(big, 0.0))

        font = p.font()
        font.setPointSizeF(7.0)
        p.setFont(font)
        pen.setWidthF(1.0)
        p.setPen(pen)
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = -deg * ppd
            half = 0.30 * r if deg % 20 == 0 else 0.20 * r
            p.drawLine(QtCore.QPointF(-half, y), QtCore.QPointF(half, y))
            p.drawText(QtCore.QRectF(half + 4, y - 7, 26, 14),
                       QtCore.Qt.AlignmentFlag.AlignLeft
                       | QtCore.Qt.AlignmentFlag.AlignVCenter, "%d" % abs(deg))
        p.restore()

        # The roll scale is fixed to the case and the pointer turns with
        # the platform, which is how the instrument works. A scale that
        # turned instead would read the right number the wrong way round.
        p.save()
        p.translate(cx, cy)
        p.setPen(QtGui.QPen(QtGui.QColor(T["dim"])))
        for deg in (-60, -30, -20, -10, 0, 10, 20, 30, 60):
            a = math.radians(90.0 + deg)
            ri = r - (9.0 if deg % 30 == 0 else 5.0)
            p.drawLine(QtCore.QPointF(ri * math.cos(a), -ri * math.sin(a)),
                       QtCore.QPointF(r * math.cos(a), -r * math.sin(a)))
        p.rotate(-roll)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(T["warn"]))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0.0, -(r - 10.0)),
                                       QtCore.QPointF(-6.0, -(r - 20.0)),
                                       QtCore.QPointF(6.0, -(r - 20.0))]))
        p.restore()

        # The aircraft symbol never moves: it is the case, and the world
        # is what turns behind it.
        p.save()
        p.translate(cx, cy)
        pen = QtGui.QPen(QtGui.QColor(T["warn"]))
        pen.setWidthF(2.0)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(-0.42 * r, 0.0), QtCore.QPointF(-0.12 * r, 0.0))
        p.drawLine(QtCore.QPointF(0.12 * r, 0.0), QtCore.QPointF(0.42 * r, 0.0))
        p.drawEllipse(QtCore.QRectF(-2.5, -2.5, 5.0, 5.0))
        p.restore()

        p.setPen(QtGui.QColor(T["text"]))
        font.setPointSizeF(8.0)
        p.setFont(font)
        p.drawText(QtCore.QRectF(0, h - 16, w / 2.0 - 4, 15),
                   QtCore.Qt.AlignmentFlag.AlignRight, "roll %+.1f" % roll)
        p.drawText(QtCore.QRectF(w / 2.0 + 4, h - 16, w / 2.0 - 4, 15),
                   QtCore.Qt.AlignmentFlag.AlignLeft, "pitch %+.1f" % pitch)


class CompassWidget(QtWidgets.QWidget):
    """Heading, and the direction the platform is actually travelling in.

    Two needles rather than one. Where the nose points and where the
    velocity goes are different quantities, and a heading that has
    drifted shows up as an angle between them long before it shows up as
    anything else. On a vehicle that cannot travel sideways, that angle
    IS the heading error.

    North up, with the platform turning inside the rose. A heading-up
    card reads better in a cockpit and worse on a bench, where the track
    on the next tab is drawn north up too and the two have to agree.
    """

    def __init__(self):
        super().__init__()
        self.setMinimumSize(190, 190)
        self.yaw = None
        self.track = None
        themed(self.update)

    def set_state(self, yaw_deg, track_deg=None):
        self.yaw, self.track = yaw_deg, track_deg
        self.update()

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2.0 - 12.0
        cx, cy = w / 2.0, h / 2.0
        p.setBrush(QtGui.QColor(T["field"]))
        p.setPen(QtGui.QPen(QtGui.QColor(T["line"])))
        p.drawEllipse(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r))
        if self.yaw is None:
            p.setPen(QtGui.QColor(T["dim"]))
            p.drawText(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r),
                       QtCore.Qt.AlignmentFlag.AlignCenter, "no heading")
            return

        p.save()
        p.translate(cx, cy)
        font = p.font()
        font.setPointSizeF(7.5)
        p.setFont(font)
        p.setPen(QtGui.QPen(QtGui.QColor(T["dim"])))
        for deg in range(0, 360, 15):
            a = math.radians(deg)
            ri = r - (10.0 if deg % 45 == 0 else 5.0)
            p.drawLine(QtCore.QPointF(ri * math.sin(a), -ri * math.cos(a)),
                       QtCore.QPointF(r * math.sin(a), -r * math.cos(a)))
        for deg, name in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            a = math.radians(deg)
            rt = r - 21.0
            p.setPen(QtGui.QColor(T["bad"] if deg == 0 else T["text"]))
            p.drawText(QtCore.QRectF(rt * math.sin(a) - 10,
                                     -rt * math.cos(a) - 8, 20, 16),
                       QtCore.Qt.AlignmentFlag.AlignCenter, name)

        if self.track is not None:
            a = math.radians(self.track)
            pen = QtGui.QPen(QtGui.QColor(T["accent"]))
            pen.setWidthF(2.0)
            pen.setStyle(QtCore.Qt.PenStyle.DashLine)
            p.setPen(pen)
            p.drawLine(QtCore.QPointF(0.0, 0.0),
                       QtCore.QPointF((r - 24.0) * math.sin(a),
                                      -(r - 24.0) * math.cos(a)))

        p.rotate(self.yaw)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(T["ok"]))
        p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0.0, -(r - 24.0)),
                                       QtCore.QPointF(-0.16 * r, 0.22 * r),
                                       QtCore.QPointF(0.0, 0.10 * r),
                                       QtCore.QPointF(0.16 * r, 0.22 * r)]))
        p.restore()

        p.setPen(QtGui.QColor(T["text"]))
        font.setPointSizeF(8.0)
        p.setFont(font)
        text = "yaw %.1f" % (self.yaw % 360.0)
        if self.track is not None:
            text += "   track %.1f   diff %+.1f" % (
                self.track % 360.0,
                (self.track - self.yaw + 180.0) % 360.0 - 180.0)
        p.drawText(QtCore.QRectF(0, h - 16, w, 15),
                   QtCore.Qt.AlignmentFlag.AlignCenter, text)


# ===========================================================================
# Sensors
# ===========================================================================

class SensorsTab(QtWidgets.QWidget):
    """What the sensors are doing right now.

    Plots the stream the board is already sending rather than asking for
    anything: those samples are on the link whether or not this window is
    open, so the tab costs the device nothing.

    Four views of the same three rings, because the questions asked of
    raw inertial data are not one question. The traces answer "is it
    moving and does it look sane". The norms answer "is the triad
    consistent with itself", which is the one thing a three axis sensor
    can be checked against without a reference. The spectrum answers
    "what is this platform vibrating at", which no time trace at 800 Hz
    shows. And the statistics put a number on the noise.
    """

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.acc = Ring(HISTORY_MAX_S * RATE_IMU_HZ, 3)
        self.gyr = Ring(HISTORY_MAX_S * RATE_IMU_HZ, 3)
        self.mag = Ring(HISTORY_MAX_S * RATE_MAG_HZ, 3)
        # Temperatures move over minutes, so one row per batch is more
        # than the signal needs and a great deal less than one per sample.
        self.temp = Ring(HISTORY_MAX_S * 50, 3)
        self.imu_n = 0
        self.imu_t0 = None
        self.last = {}
        self.mag_ref = None
        self._slow = 0
        self._build()

        self.timer = QtCore.QTimer(self)
        # Redrawing per batch would repaint twenty times a second for
        # motion the eye cannot follow. Ten is what a person reads.
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)

    # -- construction ------------------------------------------------------

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        self.state = QtWidgets.QLabel("waiting for samples")
        self.state.setStyleSheet(MONO)
        v.addWidget(self.state)

        self.inner = QtWidgets.QTabWidget()
        self.inner.addTab(self._page_series(), "Time series")
        self.inner.addTab(self._page_norms(), "Norms and temperature")
        self.inner.addTab(self._page_spectrum(), "Spectrum")
        self.inner.addTab(self._page_stats(), "Statistics")
        v.addWidget(self.inner, 1)

    def _page_series(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.curves = []
        first = None
        for title, unit in (("Accelerometer", "g"), ("Gyroscope", "deg/s"),
                            ("Magnetometer", "uT")):
            p = make_plot(title, unit, "seconds before now", legend=True,
                          decimate=True)
            self.curves.append(themed_curves(p, "xyz"))
            # Locked to one time axis: a step in the gyro that has nothing
            # under it in the accelerometer is a different fault from one
            # that has, and three plots scrolling independently cannot be
            # read that way.
            if first is None:
                first = p
            else:
                p.setXLink(first)
            v.addWidget(p, 1)
        return page

    def _page_norms(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)

        self.p_anorm = make_plot("Specific force magnitude", "g",
                                 None, decimate=True)
        self.c_anorm = themed_curve(self.p_anorm, "accent")
        self.l_g = self.p_anorm.addLine(y=1.0)
        self.p_mnorm = make_plot("Magnetic field magnitude", "uT", None,
                                 decimate=True)
        self.c_mnorm = themed_curve(self.p_mnorm, "accent")
        self.l_mref = self.p_mnorm.addLine(y=0.0)
        self.l_mref.setVisible(False)
        themed(lambda: [ln.setPen(pg.mkPen(
            T["warn"], style=QtCore.Qt.PenStyle.DashLine))
            for ln in (self.l_g, self.l_mref)])

        self.p_temp = make_plot("Temperature", "degC", "seconds before now",
                                legend=True)
        self.c_temp = themed_curves(self.p_temp, ("imu die", "mag", "baro"))
        for p in (self.p_anorm, self.p_mnorm, self.p_temp):
            v.addWidget(p, 1)
        self.p_mnorm.setXLink(self.p_anorm)
        self.p_temp.setXLink(self.p_anorm)

        note = QtWidgets.QLabel(
            "The magnitudes are the one check a three axis sensor offers "
            "against itself, without a reference and whatever the platform "
            "is doing: at rest the specific force is 1 g whichever way the "
            "box is turned, and the field magnitude is the same everywhere "
            "in a room. A magnitude that changes with orientation is a "
            "scale factor or a hard iron error, and it does that while all "
            "three axis traces still look entirely plausible. The dashed "
            "line on the field is the WMM reference the filter is using, "
            "when it is reporting one.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    def _page_spectrum(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.p_spec_a = make_plot("Accelerometer", "g/sqrt(Hz)", None,
                                  legend=True)
        self.p_spec_g = make_plot("Gyroscope", "dps/sqrt(Hz)", "Hz",
                                  legend=True)
        self.c_spec_a = themed_curves(self.p_spec_a, "xyz")
        self.c_spec_g = themed_curves(self.p_spec_g, "xyz")
        for p in (self.p_spec_a, self.p_spec_g):
            p.setLogMode(x=True, y=True)
            v.addWidget(p, 1)
        self.p_spec_g.setXLink(self.p_spec_a)

        note = QtWidgets.QLabel(
            "Amplitude spectral density over the look-back window, Welch "
            "averaged with a Hann window over 1024 sample segments. The "
            "mean is removed per segment, so a bias and the 1 g on a "
            "downward axis are not in it. A density rather than an FFT "
            "magnitude, so a longer window gives a smoother curve at the "
            "same height instead of a taller one. The flat part is the "
            "sensor's own noise; peaks above it are the airframe. Nothing "
            "above the anti alias filter, and nothing above half the "
            "sample rate, is real - a rotor line beyond that is folded "
            "back and appears at a frequency it does not have.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    STAT_ROWS = (("acc x", "g"), ("acc y", "g"), ("acc z", "g"),
                 ("|acc|", "g"),
                 ("gyr x", "deg/s"), ("gyr y", "deg/s"), ("gyr z", "deg/s"),
                 ("|gyr|", "deg/s"),
                 ("mag x", "uT"), ("mag y", "uT"), ("mag z", "uT"),
                 ("|mag|", "uT"))

    def _page_stats(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.stats = QtWidgets.QTableWidget(len(self.STAT_ROWS), 7)
        self.stats.setHorizontalHeaderLabels(
            ["channel", "unit", "mean", "std", "min", "max", "peak-peak"])
        self.stats.verticalHeader().setVisible(False)
        self.stats.horizontalHeader().setStretchLastSection(True)
        self.stats.setStyleSheet(MONO)
        for r, (name, unit) in enumerate(self.STAT_ROWS):
            for c, text in ((0, name), (1, unit)):
                self.stats.setItem(r, c, QtWidgets.QTableWidgetItem(text))
        v.addWidget(self.stats, 1)

        note = QtWidgets.QLabel(
            "Over the look-back window, which is what the toolbar's Look "
            "back sets. The standard deviation of a triad at rest is its "
            "noise, and it is the number to compare between two units or "
            "before and after a mounting change. It is NOT the same as the "
            "noise density on the spectrum page: this one gets smaller as "
            "the bandwidth does, so it only compares like with like.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    # -- data --------------------------------------------------------------

    def set_history(self, seconds):
        """How far back to plot. The rings hold HISTORY_MAX_S at the
        nominal rates; a faster stream simply fills them sooner, and what
        is not there is not drawn."""
        self.window_s = seconds

    def on_batch(self, batch):
        if self.imu_t0 is None:
            self.imu_t0 = batch["t"]
        for t_us, acc, gyr, status, seq in batch["imu"]:
            # Stamped with the DEVICE clock, not the arrival time: a batch
            # lands in a lump, and plotting against arrival would draw the
            # link's jitter instead of the sensor's signal.
            self.acc.add(t_us * 1e-6, acc)
            self.gyr.add(t_us * 1e-6, gyr)
            self.last["status"] = status
            self.last["seq"] = seq
        self.imu_n += len(batch["imu"])
        for m in batch["mag"]:
            self.mag.add(m[0] * 1e-6, m[1:4])
            self.last["mag_temp"] = m[4]
        for b in batch["baro"]:
            self.last["baro"] = (b[1], b[2])
        nav = batch.get("nav")
        if nav:
            self.mag_ref = nav["mag_ref_uT"]
        if batch["imu"] or batch["mag"] or batch["baro"]:
            st = self.last.get("status")
            self.temp.add(batch["t"], (
                ux.imu_status_temp_c(st) if st is not None else np.nan,
                self.last.get("mag_temp", np.nan),
                self.last["baro"][1] if "baro" in self.last else np.nan))

    # -- display -----------------------------------------------------------

    def refresh(self):
        page = self.inner.currentWidget()
        for ring, curves in ((self.acc, self.curves[0]),
                             (self.gyr, self.curves[1]),
                             (self.mag, self.curves[2])):
            t, vals = ring_window(ring, self.window_s)
            if t is None:
                continue
            for i, c in enumerate(curves):
                c.setData(t, vals[:, i])

        self._refresh_state()
        # Only the page in front, and the two that read the whole window
        # end to end at a fifth of the rate. At the longest look-back
        # those are a quarter of a million samples per channel, and
        # nobody reads a standard deviation ten times a second.
        self._slow = (self._slow + 1) % 5
        if page is self.inner.widget(1):
            self._refresh_norms()
        elif page is self.inner.widget(2):
            if self._slow == 0:
                self._refresh_spectrum()
        elif page is self.inner.widget(3):
            if self._slow == 0:
                self._refresh_stats()

    def _refresh_state(self):
        bits = []
        if self.imu_t0 is not None:
            el = time.monotonic() - self.imu_t0
            if el > 1.0:
                bits.append("%.0f Hz" % (self.imu_n / el))
        st = self.last.get("status")
        if st is not None:
            d = ux.decode_imu_status(st)
            bits.append("imu %5.1f C" % d["temp_c"])
            # Straight out of the sample's own status word: whether the
            # numbers being plotted are corrected is not something the
            # numbers can show.
            bits.append("cal %s" % ("applied" if d["cal_applied"] else "RAW"))
            if d["saturated"]:
                which = " ".join(w for w, on in (("acc", d["sat_acc"]),
                                                 ("gyr", d["sat_gyr"])) if on)
                bits.append("SATURATED %s" % which)
        if "baro" in self.last:
            bits.append("baro %.1f Pa  %.1f C" % self.last["baro"])
        if "mag_temp" in self.last:
            bits.append("mag %.1f C" % self.last["mag_temp"])
        self.state.setText("   ".join(bits) or "waiting for samples")

    def _refresh_norms(self):
        t, vals = ring_window(self.acc, self.window_s)
        if t is not None:
            self.c_anorm.setData(t, np.linalg.norm(vals, axis=1))
        t, vals = ring_window(self.mag, self.window_s)
        if t is not None:
            self.c_mnorm.setData(t, np.linalg.norm(vals, axis=1))
        if self.mag_ref is not None:
            self.l_mref.setValue(self.mag_ref)
            self.l_mref.setVisible(True)
        t, vals = ring_window(self.temp, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_temp):
                c.setData(t, vals[:, i], connect="finite")

    def _refresh_spectrum(self):
        for ring, curves in ((self.acc, self.c_spec_a),
                             (self.gyr, self.c_spec_g)):
            t, vals = ring_window(ring, self.window_s)
            fs = sample_rate(t)
            for i, c in enumerate(curves):
                f, asd = spectrum(vals[:, i], fs) if t is not None else (None, None)
                if f is None:
                    c.setData([], [])
                    continue
                # The first bin is DC, which the per segment mean removal
                # has already emptied, and a log x axis has no room for it.
                c.setData(f[1:], asd[1:])

    def _refresh_stats(self):
        rows = []
        for ring in (self.acc, self.gyr, self.mag):
            _t, vals = ring_window(ring, self.window_s)
            if vals is None:
                rows.extend([None] * 4)
                continue
            for i in range(3):
                rows.append(vals[:, i])
            rows.append(np.linalg.norm(vals, axis=1))
        for r, series in enumerate(rows):
            for c in range(2, 7):
                self.stats.setItem(r, c, QtWidgets.QTableWidgetItem("--"))
            if series is None or series.size == 0:
                continue
            s = np.asarray(series, dtype=np.float64)
            cells = ("%+.5f" % s.mean(), "%.5f" % s.std(),
                     "%+.5f" % s.min(), "%+.5f" % s.max(),
                     "%.5f" % (s.max() - s.min()))
            for c, text in enumerate(cells):
                self.stats.setItem(r, c + 2, QtWidgets.QTableWidgetItem(text))
        self.stats.resizeColumnsToContents()

    def set_online(self, online):
        if online:
            self.imu_n = 0
            self.imu_t0 = None
            self.timer.start()
        else:
            self.timer.stop()
            self.state.setText("not connected")


# ===========================================================================
# Calibration
# ===========================================================================

TRIADS = (("Accelerometer", ux.GRP_CAL_ACC, "acc", "g"),
          ("Gyroscope", ux.GRP_CAL_GYR, "gyr", "deg/s"),
          ("Magnetometer", ux.GRP_CAL_MAG, "mag", "uT"))

# Records per CFG-VALSET. A node is 58 bytes on the wire and the board
# accepts about 500, so eight fit with room to spare - the same chunk
# inslib_calib_gui.py uploads with.
NODES_PER_FRAME = 8

# Two nodes this close together have no defined winner in a table that is
# looked up by temperature, so a write that would produce them is refused
# here rather than stored.
NODE_MIN_GAP_C = 0.5


class CalibrationTab(QtWidgets.QWidget):
    """What is stored in the board, decoded, and editable.

    The table is the one the board USES, whole: three bias terms and all
    NINE matrix elements per temperature node. In use, not merely
    present, because NPTS is what decides that and a record can be
    replaced but never removed, so a cleared or a shrunk table leaves the
    old records behind. Showing those would say "calibrated" about a
    triad the firmware reads raw. What is left over is reported in the
    note beside the table instead. The off-diagonal elements are
    not a summary statistic - each one says how much of one raw axis ends
    up on another corrected axis, and which pair it is matters. Two units
    with the same largest off-diagonal and different signs on it are not
    two units with the same mounting.

    Editing is here for the case the solver cannot serve: a node whose
    numbers are known from elsewhere, a sign to be checked against the
    hardware, a bias to be nudged after a bench measurement. It is NOT a
    substitute for inslib_calib_gui.py, which records, fits and merges a
    node and is the only sound way to produce one from a device that is
    in front of you.

    Two things guard the write. It is behind a checkbox, because a
    diagnostic window that is left open should not turn a stray keystroke
    into a stored calibration. And the bias POLYNOMIAL is retired by every
    write, the way the uploader retires it: a fit made over the previous
    nodes no longer describes the table, and left in place it would keep
    overriding the bias terms that were just typed.
    """

    COLS = ("T [C]", "bias x", "bias y", "bias z",
            "Mxx", "Mxy", "Mxz", "Myx", "Myy", "Myz", "Mzx", "Mzy", "Mzz")

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.temps = {}
        # (temp_c, row-major 3x3, bias) per kind, in the order they are
        # shown, which is the order they were read in.
        self.read_nodes = {}
        self.npts = {}
        self.polys = {}
        self._loading = False
        self._build()

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        top = QtWidgets.QHBoxLayout()
        self.btn_read = QtWidgets.QPushButton("Read calibration")
        self.btn_read.setObjectName("primary")
        self.btn_read.setEnabled(False)
        self.btn_read.clicked.connect(self.read)
        top.addWidget(self.btn_read)

        self.editable = QtWidgets.QCheckBox("Allow editing")
        self.editable.setToolTip(
            "Unlocks the table. A node is normally SOLVED from a recording "
            "by inslib_calib_gui.py; typing one is for numbers that are "
            "known from elsewhere.")
        self.editable.setEnabled(False)
        self.editable.toggled.connect(self._editable_toggled)
        top.addWidget(self.editable)

        self.btn_write = QtWidgets.QPushButton("Write nodes (live)")
        self.btn_store = QtWidgets.QPushButton("Write && store")
        self.btn_revert = QtWidgets.QPushButton("Revert")
        self.btn_write.clicked.connect(lambda: self.write(persist=False))
        self.btn_store.clicked.connect(lambda: self.write(persist=True))
        self.btn_revert.clicked.connect(self.read)
        self.btn_clear = QtWidgets.QPushButton("Clear triad")
        self.btn_clear.setToolTip(
            "Stop using the stored table of the triad whose page is open, "
            "and leave\nthe other two alone. For a sensor that was "
            "remounted or whose driver\naxis table changed: its "
            "calibration was solved in a frame that no longer\nexists, "
            "and a wrong correction is worse than none.")
        self.btn_clear.clicked.connect(self.clear)
        for b in (self.btn_write, self.btn_store, self.btn_revert,
                  self.btn_clear):
            b.setEnabled(False)
            top.addWidget(b)

        self.summary = QtWidgets.QLabel("")
        themed(lambda: self.summary.setStyleSheet(dim()))
        top.addWidget(self.summary, 1)
        v.addLayout(top)

        hint = QtWidgets.QLabel(
            "The table the board uses, whole: the bias and all nine "
            "matrix elements per temperature node. The board corrects a sample as "
            "M (raw - bias), so element M(row, col) is how much of RAW "
            "axis col ends up on CORRECTED axis row - the diagonal is "
            "scale, the rest is mounting and non-orthogonality. A node is "
            "normally solved from a recording by inslib_calib_gui.py, "
            "which also knows how a new one merges into the table; "
            "unlocking the editor here writes exactly what is typed. Every "
            "write retires the bias polynomial, since a fit made over the "
            "previous nodes no longer describes the table.")
        hint.setWordWrap(True)
        themed(lambda: hint.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(hint)

        self.inner = QtWidgets.QTabWidget()
        self.tables, self.plots, self.notes, self.details = {}, {}, {}, {}
        for title, _grp, kind, unit in TRIADS:
            page = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(page)

            table = QtWidgets.QTableWidget(0, len(self.COLS))
            table.setHorizontalHeaderLabels(
                [c if not c.startswith("bias") else "%s [%s]" % (c, unit)
                 for c in self.COLS])
            table.setStyleSheet(MONO)
            table.itemChanged.connect(
                lambda item, k=kind: self._cell_changed(k, item))
            table.itemSelectionChanged.connect(
                lambda k=kind: self._show_detail(k))
            self.tables[kind] = table
            # The wider share: thirteen numeric columns against one small
            # scatter plot, and the columns are what this page is for.
            h.addWidget(table, 7)

            right = QtWidgets.QVBoxLayout()
            p = make_plot("Bias over temperature", unit,
                          "die temperature [C]")
            self.plots[kind] = p
            right.addWidget(p, 1)

            detail = QtWidgets.QLabel("")
            detail.setStyleSheet(MONO)
            detail.setTextInteractionFlags(
                QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            self.details[kind] = detail
            right.addWidget(detail)

            note = QtWidgets.QLabel("")
            note.setWordWrap(True)
            themed(lambda w=note: w.setStyleSheet(dim() + " font-size: 11px;"))
            self.notes[kind] = note
            right.addWidget(note)
            h.addLayout(right, 4)

            self.inner.addTab(page, title)
        v.addWidget(self.inner, 1)
        self.inner.currentChanged.connect(self._retitle_clear)
        self._retitle_clear(self.inner.currentIndex())

    # -- board conversation ------------------------------------------------

    def set_online(self, online):
        self.btn_read.setEnabled(online)
        self.editable.setEnabled(online)
        if not online:
            self.summary.setText("not connected")
            self.editable.setChecked(False)
        self._editable_toggled(self.editable.isChecked())

    def _retitle_clear(self, index):
        """The button says which triad it means, because that is the whole
        question being asked of it."""
        if 0 <= index < len(TRIADS):
            self.btn_clear.setText("Clear %s" % TRIADS[index][0].lower())

    def _editable_toggled(self, on):
        on = on and self.editable.isEnabled()
        for kind, table in self.tables.items():
            table.setEditTriggers(
                QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked
                | QtWidgets.QAbstractItemView.EditTrigger.EditKeyPressed
                if on else
                QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        for b in (self.btn_write, self.btn_store, self.btn_revert,
                  self.btn_clear):
            b.setEnabled(on)

    def read(self):
        self.console.submit("calibration", self._job)

    @staticmethod
    def _job(link):
        """One wildcard per triad.

        The BOARD expands the wildcard, so a firmware holding more nodes
        than this tool has heard of reports them anyway. That is the whole
        reason the protocol has one.
        """
        out = {}
        for _title, grp, kind, _unit in TRIADS:
            out[kind] = link.valget([ux.group_wildcard(grp)], ux.VALGET_RAM)
        out["info"] = link.info()
        return out

    def on_read(self, data):
        info = data.get("info")
        if info:
            self.temps = {"acc": info["temp_imu_c"], "gyr": info["temp_imu_c"],
                          "mag": info["temp_mag_c"]}
            self.summary.setText(
                "image crc %08X, slot %d    nodes  acc %d / gyr %d / mag %d"
                % (info["cfg_crc"], info["cfg_seq"], info["npts"]["acc"],
                   info["npts"]["gyr"], info["npts"]["mag"]))
        for _title, grp, kind, unit in TRIADS:
            self._show_triad(kind, unit, grp, data.get(kind, {}))

    def _show_triad(self, kind, unit, grp, values):
        npts_raw = values.get(ux.cal_npts_key(grp))
        npts = ux.scalar_from_bytes(npts_raw) if npts_raw else 0

        nodes = []
        for n in range(ux.CAL_PTS_MAX):
            raw = values.get(ux.cal_point_key(grp, n))
            if raw is None:
                continue
            got = ux.unpack_cal_point(raw)
            if got:
                nodes.append(got)
        nodes.sort(key=lambda p: p[0])
        # NPTS decides how many of the records are in service, so it also
        # decides what belongs on screen. Anything past it is inert and
        # is reported below rather than drawn as calibration.
        stored = len(nodes)
        nodes = nodes[:npts]
        self.read_nodes[kind] = nodes
        self.npts[kind] = npts

        table = self.tables[kind]
        # The signal is what marks a cell as edited by hand, and filling
        # the table fires it for every cell.
        self._loading = True
        table.setRowCount(len(nodes))
        for r, (temp_c, rows, bias) in enumerate(nodes):
            for c, text in enumerate(self._cells(temp_c, rows, bias)):
                table.setItem(r, c, QtWidgets.QTableWidgetItem(text))
        self._loading = False
        table.resizeColumnsToContents()
        self._show_detail(kind)

        plot = self.plots[kind]
        plot.clear()
        plot.setLabel("left", unit)
        note = []
        if not nodes:
            msg = ("nothing in use for this triad: its samples are the raw "
                   "sensor, whatever APPLY_CAL says.")
            if npts:
                msg += (" NPTS says %d node(s) and none of them are present, "
                        "which the board reports as a mismatch." % npts)
            elif stored:
                msg += (" %d node record(s) are still in the store, out of "
                        "service. Writing the count back puts them to work "
                        "again." % stored)
            self.notes[kind].setText(msg)
            return

        t = np.array([p[0] for p in nodes])
        b = np.array([p[2] for p in nodes])
        for i, axis in enumerate("xyz"):
            plot.plot(t, b[:, i], pen=None, symbol="o", symbolSize=7,
                      symbolBrush=T["trace"][i], name="%s measured" % axis)

        poly_raw = values.get(ux.cal_biaspoly_key(grp))
        poly = ux.unpack_bias_poly(poly_raw) if poly_raw else None
        self.polys[kind] = poly
        if poly and poly[1] != ux.CAL_POLY_NONE:
            t_ref, deg, coeffs = poly
            # Drawn only over the node range. Outside it the device does
            # not extrapolate either - it evaluates at the nearest node -
            # and a cubic running away past the last point would show
            # behaviour the firmware does not have.
            tt = np.linspace(float(t.min()), float(t.max()), 200)
            for i in range(3):
                c = coeffs[i][:deg + 1]
                y = sum(c[k] * (tt - t_ref) ** k for k in range(len(c)))
                plot.plot(tt, y, pen=trace_pen(i))
            note.append("bias from a degree %d polynomial about %.1f C; the "
                        "points are what was measured. It is what the board "
                        "uses, so editing a bias here has no effect until "
                        "the polynomial is retired - which a write does."
                        % (deg, t_ref))
        else:
            note.append("no polynomial stored: the bias is interpolated "
                        "between the nodes.")

        if npts > stored:
            note.append("NPTS says %d nodes and only %d are present. The "
                        "board reports that mismatch rather than using it, "
                        "so nothing is being corrected." % (npts, stored))
        elif stored > npts:
            note.append("%d node record(s) beyond the %d in use are still in "
                        "the store, out of service. Writing the count back "
                        "over them puts them to work again."
                        % (stored - npts, npts))

        now = self.temps.get(kind)
        if now is not None:
            plot.addLine(x=now, pen=pg.mkPen(
                T["warn"], style=QtCore.Qt.PenStyle.DashLine))
            if len(nodes) == 1:
                note.append("one node: the correction is constant, which is "
                            "the honest answer for a unit measured at one "
                            "temperature.")
            elif now < t.min() or now > t.max():
                note.append("die at %.1f C, OUTSIDE the calibrated range "
                            "%.1f..%.1f C - the correction is evaluated at "
                            "the nearest node, not extrapolated."
                            % (now, t.min(), t.max()))
            else:
                note.append("die at %.1f C, inside the calibrated range "
                            "%.1f..%.1f C." % (now, t.min(), t.max()))
        self.notes[kind].setText("  ".join(note))

    @staticmethod
    def _cells(temp_c, rows, bias):
        """One node as the thirteen strings the table shows.

        Enough digits that a value read back compares equal to the one
        that was typed: these are f32 on the wire, and a display that
        rounded would make every write look like a change."""
        return (["%.2f" % temp_c] + ["%+.6g" % v for v in bias]
                + ["%.6f" % rows[r][c] for r in range(3) for c in range(3)])

    # -- editing -----------------------------------------------------------

    def _cell_changed(self, kind, item):
        if self._loading:
            return
        nodes = self.read_nodes.get(kind) or []
        stored = (self._cells(*nodes[item.row()])[item.column()]
                  if item.row() < len(nodes) else None)
        changed = stored is None or item.text().strip() != stored
        item.setForeground(QtGui.QBrush(QtGui.QColor(
            T["changed"] if changed else T["text"])))
        self._show_detail(kind)

    def _show_detail(self, kind):
        """The selected node as a matrix rather than as thirteen columns."""
        table = self.tables[kind]
        rows = {i.row() for i in table.selectedItems()}
        if len(rows) != 1:
            self.details[kind].setText("")
            return
        try:
            temp_c, m, bias = self._node_from_row(table, rows.pop())
        except ValueError as exc:
            self.details[kind].setText(str(exc))
            return
        off = max(abs(m[i][j]) for i in range(3) for j in range(3) if i != j)
        text = ["node at %.2f C" % temp_c,
                "bias   %s" % "  ".join("%+.6g" % v for v in bias)]
        for r in range(3):
            text.append("  [ %s ]" % "  ".join("%+9.6f" % v for v in m[r]))
        text.append("scale  %s"
                    % "  ".join("%+.3f %%" % ((m[i][i] - 1.0) * 100.0)
                                for i in range(3)))
        # A small off-diagonal reads as an angle, and that is the number
        # somebody has in mind when they check a mounting. Only a reading,
        # not an identity: the same elements also carry scale error and
        # non-orthogonality, which no single angle separates.
        text.append("largest off-diagonal %.2e, about %.2f mrad if it were "
                    "misalignment alone" % (off, off * 1e3))
        self.details[kind].setText("\n".join(text))

    @staticmethod
    def _node_from_row(table, r):
        """(temp, row-major 3x3, bias) out of one table row."""
        vals = []
        for c in range(len(CalibrationTab.COLS)):
            item = table.item(r, c)
            text = "" if item is None else item.text().strip()
            try:
                v = float(text)
            except ValueError:
                raise ValueError("row %d, %s: %r is not a number"
                                 % (r + 1, CalibrationTab.COLS[c], text))
            if not math.isfinite(v):
                raise ValueError("row %d, %s: not finite"
                                 % (r + 1, CalibrationTab.COLS[c]))
            vals.append(v)
        m = [[vals[4 + r2 * 3 + c2] for c2 in range(3)] for r2 in range(3)]
        return vals[0], m, vals[1:4]

    def _pending(self):
        """(frames, description) for what the tables hold, or ([], []).

        Nodes are compared by INDEX after sorting by temperature, which is
        the order the board holds them in: editing a temperature can move
        a node past its neighbour, and writing it at the index it was
        displayed at would leave the table out of order."""
        frames, what, polys = [], [], []
        for _title, grp, kind, _unit in TRIADS:
            table = self.tables[kind]
            new = [self._node_from_row(table, r)
                   for r in range(table.rowCount())]
            new.sort(key=lambda p: p[0])
            for a, b in zip(new, new[1:]):
                if b[0] - a[0] < NODE_MIN_GAP_C:
                    raise ValueError(
                        "%s: two nodes at %.2f C and %.2f C. The table is "
                        "looked up by temperature and a pair that close has "
                        "no defined winner." % (kind, a[0], b[0]))
            old = self.read_nodes.get(kind) or []
            items = []
            for i, node in enumerate(new):
                packed = ux.pack_cal_point(node[0], node[1], node[2])
                if i < len(old) and packed == ux.pack_cal_point(*old[i]):
                    continue
                items.append((ux.cal_point_key(grp, i), packed))
            if not items:
                continue
            what.append("%s: %d of %d node(s)" % (kind, len(items), len(new)))
            for i in range(0, len(items), NODES_PER_FRAME):
                frames.append(items[i:i + NODES_PER_FRAME])
            polys.append((ux.cal_biaspoly_key(grp), ux.no_bias_poly()))
            if self.polys.get(kind) and self.polys[kind][1] != ux.CAL_POLY_NONE:
                what.append("  and retires the %s bias polynomial" % kind)
        if polys:
            # Last, so that the frame carrying the flash bit is the one
            # that finishes the change rather than one in the middle of it.
            frames.append(polys)
        return frames, what

    def clear(self):
        """Retire ONE triad's stored table and leave the other two alone.

        NPTS is what decides whether a table is used at all: the firmware
        reads it first and treats zero as no calibration (cfg_keys.h, and
        cal_validate in cal.c returns before touching a node). So setting
        it to zero is the whole operation. The POINT records stay in the
        store, because a key can be replaced but never removed, and
        leaving them costs nothing while making this reversible by
        writing the count back. The bias polynomial goes with it: a fit
        made over nodes that are no longer in use has nothing left to
        describe, and it would otherwise keep overriding the bias of the
        next table written here.

        This is the answer to a sensor that was remounted, or whose axis
        table in the driver changed: the stored calibration was solved in
        a frame that no longer exists, and a correction from the wrong
        frame is worse than no correction at all."""
        index = self.inner.currentIndex()
        if not 0 <= index < len(TRIADS):
            return
        title, grp, kind, _unit = TRIADS[index]
        npts = self.npts.get(kind, 0)
        if not npts:
            self.console.status(
                "%s: nothing stored to clear" % title.lower(), "idle")
            return

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Clear the %s calibration?" % title.lower())
        box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        box.setText(
            "Stop using the %d stored %s node(s). The other two triads are "
            "not touched, and neither is the housing rotation.\n\n"
            "The nodes themselves stay in the store and are only taken out "
            "of service, so writing the count back puts them to work again. "
            "The bias polynomial is retired, since it was fitted over "
            "them.\n\n"
            "Live only is gone at the next power cycle. Storing it means "
            "the unit comes up with this triad uncorrected."
            % (npts, title.lower()))
        live = box.addButton("Clear (live)",
                             QtWidgets.QMessageBox.ButtonRole.DestructiveRole)
        store = box.addButton("Clear && store",
                              QtWidgets.QMessageBox.ButtonRole.DestructiveRole)
        cancel = box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(cancel)
        box.exec()
        if box.clickedButton() not in (live, store):
            return

        # NPTS last in the frame, the way the firmware asks for it: the
        # table stops being used in one step rather than node by node.
        items = [(ux.cal_biaspoly_key(grp), ux.no_bias_poly()),
                 (ux.cal_npts_key(grp), 0)]
        layers = ux.LAYER_RAM | (ux.LAYER_FLASH
                                 if box.clickedButton() is store else 0)
        self.console.submit(
            "calclear", lambda link: self._job_write(link, [items], layers))

    def write(self, persist):
        try:
            frames, what = self._pending()
        except ValueError as exc:
            self.console.status(str(exc), "bad")
            return
        if not frames:
            self.console.status("nothing changed", "idle")
            return
        text = ("Write the edited nodes to the board%s?\n\n%s\n\n"
                "Every write retires the bias polynomial of the triad it "
                "touches: a fit made over the previous nodes no longer "
                "describes the table."
                % (" AND store them in flash" if persist else
                   " (live only, gone at the next power cycle)",
                   "\n".join(what)))
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Write calibration nodes?")
        box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        box.setText(text)
        box.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Yes
                               | QtWidgets.QMessageBox.StandardButton.Cancel)
        box.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        if box.exec() != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        last = ux.LAYER_RAM | (ux.LAYER_FLASH if persist else 0)
        self.console.submit(
            "calwrite", lambda link: self._job_write(link, frames, last))

    @staticmethod
    def _job_write(link, frames, last_layers):
        """Every frame in order, stopping at the first refusal.

        The board applies a VALSET in order and stops where it fails, so a
        refusal part way through leaves a table that is neither the old
        one nor the new one. The acknowledgement says how many values went
        in, and the caller reads the whole thing back rather than assuming
        anything about what is now in there."""
        for i, items in enumerate(frames):
            layers = last_layers if i == len(frames) - 1 else ux.LAYER_RAM
            ack = link.valset(items, layers)
            if ack and not ack["ok"]:
                return ack
        return {"ok": True, "text": "", "detail": 0, "key": 0}


# ===========================================================================
# Solution
# ===========================================================================

def _fmt(value, fmt="%.3f", absent="--"):
    """A number, or a mark that says the filter does not have one.

    Never a zero and never a blank. 0x40/0x0F pairs every value with a
    validity bit precisely so that "no answer" can be told from "the
    answer is zero", and a window that prints 0.000 for both throws that
    away at the last step.
    """
    if value is None:
        return absent
    if isinstance(value, (tuple, list)):
        return "  ".join(fmt % v for v in value)
    return fmt % value


class Field(QtWidgets.QLabel):
    """One monospaced readout that greys out when its value is absent.

    The colour is remembered as a palette KEY rather than as the colour
    itself, so a field that is not being refreshed - an offline window -
    still follows a theme change.
    """

    def __init__(self):
        super().__init__("--")
        self._present = False
        self._colour = None
        themed(self._restyle)

    def show_value(self, text, present=True, colour=None):
        self.setText(text)
        self._present, self._colour = present, colour
        self._restyle()

    def _restyle(self):
        key = self._colour or ("ok" if self._present else "idle")
        self.setStyleSheet(MONO + " color: %s;" % T[key])


class SolutionTab(QtWidgets.QWidget):
    """What the box believes, and what it admits it does not.

    Everything here comes from 0x40/0x0F. The absent values are as much
    the point as the present ones: an attitude-only mode has no position
    at all, and a console that quietly showed the last one it saw would
    be the reason somebody trusts it.
    """

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.att = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 3)
        self.sig = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 3)
        self.vel = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 3)
        self.pos = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 3)
        # What the receiver says about heading, for the yaw plot: vehicle
        # heading and course over ground, at the navigation message's own
        # much slower rate.
        self.head = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 2)
        self.fields = {}
        self.last = None
        self.last_t = None
        self._build()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)

        left = QtWidgets.QVBoxLayout()
        outer.addLayout(left, 2)

        self.mode = QtWidgets.QLabel("not connected")
        self.mode.setStyleSheet(MONO + " font-size: 15px;")
        left.addWidget(self.mode)

        for title, rows in (
                ("Attitude", (("roll  pitch  yaw", "rpy"),
                              ("1-sigma", "rpy_sigma"))),
                ("Velocity and position", (("velocity NED [m/s]", "vel"),
                                           ("ground speed / track", "speed"),
                                           ("position NED [m]", "pos"),
                                           ("latitude", "lat"),
                                           ("longitude", "lon"))),
                ("Height", (("above the NED origin [m]", "height"),
                            ("WGS84 ellipsoid [m]", "height_ell"),
                            ("ellipsoidal offset [m]", "ell_off"),
                            ("baro channel / its raw input [m]", "baro"),
                            ("vertical rate [m/s]", "vz"))),
                ("Aiding and mounting", (("zero updates", "zupd"),
                                         ("lever arm FRD [m]", "lever"),
                                         ("WMM field / declination", "mag")))):
            box = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout(box)
            grid.setColumnStretch(1, 1)
            for label, key in rows:
                r = grid.rowCount()
                cap = QtWidgets.QLabel(label)
                themed(lambda w=cap: w.setStyleSheet(dim()))
                grid.addWidget(cap, r, 0)
                f = Field()
                self.fields[key] = f
                grid.addWidget(f, r, 1)
            left.addWidget(box)

        self.note = QtWidgets.QLabel("")
        self.note.setWordWrap(True)
        themed(lambda: self.note.setStyleSheet(
            "color: %s; font-size: 11px;" % T["warn"]))
        left.addWidget(self.note)
        left.addStretch(1)

        right = QtWidgets.QVBoxLayout()
        outer.addLayout(right, 3)

        # Roll and pitch on one plot, yaw on another, because they do not
        # share a scale in any useful way: roll and pitch sit in a band a
        # few degrees wide while yaw walks the whole circle, and drawn
        # together the two that levelling is judged on are a flat line.
        att = QtWidgets.QTabWidget()
        p_rp = make_plot("Roll and pitch", "deg", "seconds before now",
                         legend=True)
        self.c_rp = themed_curves(p_rp, ("roll", "pitch"))
        att.addTab(p_rp, "Roll / pitch")

        p_yaw = make_plot("Yaw", "deg", "seconds before now", legend=True)
        self.c_yaw = themed_curve(p_yaw, "accent", "filter", width=2)
        # The receiver's two opinions about which way round the platform
        # is, drawn only where it actually has one. They are not the same
        # quantity as the filter's yaw and not the same as each other:
        # the COURSE is where the antenna is going, which equals a
        # heading only on something that cannot move sideways and is
        # noise at a standstill; the VEHICLE HEADING is where the
        # platform points, and a receiver without a heading source of its
        # own never sets its validity bit, which leaves that curve empty.
        self.c_head_veh = themed_curve(p_yaw, "ok", "receiver heading")
        self.c_head_mot = themed_curve(p_yaw, "idle", "receiver course")
        att.addTab(p_yaw, "Yaw")

        # The filter's own opinion of how well it knows the attitude, on
        # the same axis as the attitude itself. It is the trace to look at
        # first after an aiding change: a sigma that walks up and stays
        # there is a measurement the filter stopped getting, and neither
        # the angle nor the mode word says so.
        p_sig = make_plot("Attitude 1-sigma", "deg", "seconds before now",
                          legend=True)
        self.c_sig = themed_curves(p_sig, ("roll", "pitch", "yaw"))
        att.addTab(p_sig, "1-sigma")
        right.addWidget(att, 1)

        motion = QtWidgets.QTabWidget()
        p_vel = make_plot("Velocity NED", "m/s", "seconds before now",
                          legend=True)
        self.c_vel = themed_curves(p_vel, "ned")
        motion.addTab(p_vel, "Velocity NED")

        p_speed = make_plot("Ground speed", "m/s", "seconds before now")
        self.c_speed = themed_curve(p_speed, "accent", width=2)
        motion.addTab(p_speed, "Speed")

        p_pos = make_plot("Position NED", "m", "seconds before now",
                          legend=True)
        self.c_pos = themed_curves(p_pos, "ned")
        motion.addTab(p_pos, "Position NED")
        right.addWidget(motion, 1)

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self.mode.setText("not connected")

    def on_nav(self, d, t):
        self.last, self.last_t = d, t
        if d["rpy_deg"]:
            self.att.add(t, d["rpy_deg"])
        if d["vel_ned"]:
            self.vel.add(t, d["vel_ned"])
        if d["pos_ned"]:
            self.pos.add(t, d["pos_ned"])
        sig = d["rpy_sigma_deg"]
        if sig:
            # The yaw sigma has a validity bit of its own: the ARS free
            # integrates yaw, and a zero there would read as a perfectly
            # known heading. Blanked rather than plotted, which the curve
            # draws as a gap.
            yaw = d["yaw_sigma_deg"]
            self.sig.add(t, (sig[0], sig[1],
                             np.nan if yaw is None else yaw))

    def on_pvt(self, t, fixes):
        """The receiver's heading, for the yaw plot.

        A fix the receiver does not stand behind is not an opinion about
        anything, and a course computed from a velocity the platform does
        not have is a direction drawn out of noise - both are blanked
        rather than plotted, which the curve draws as a gap.
        """
        for fix in fixes:
            if not (fix["fix_ok"] and fix["fix_type"] >= 2):
                continue
            v = fix["vel_ned"]
            moving = math.hypot(v[0], v[1]) > TRACK_MIN_SPEED_MPS
            self.head.add(t, (fix["head_veh_deg"] if fix["head_veh_valid"]
                              else np.nan,
                              fix["head_mot_deg"] if moving else np.nan))

    def refresh(self):
        d = self.last
        if d is None:
            self.mode.setText("no navigation message - is CFG-MSGOUT-NAV on?")
            return

        src = d["att_src"]
        self.mode.setText("mode %s   attitude from %s   %s"
                          % (d["mode"].upper(), src.upper(),
                             "ready" if d["ready"] else "NOT READY"))
        self.mode.setStyleSheet(
            MONO + " font-size: 15px; color: %s;"
            % (T["ok"] if d["mode"] == "full" else T["warn"]))

        f = self.fields
        f["rpy"].show_value(_fmt(d["rpy_deg"], "%8.2f"), d["rpy_deg"] is not None)
        # The ARS free-integrates yaw, so its sigma is absent rather than
        # zero. Printing a zero there would read as a perfectly known
        # heading, which is the opposite of what it means.
        sig = d["rpy_sigma_deg"]
        f["rpy_sigma"].show_value(
            "%8.2f  %8.2f  %s" % (sig[0], sig[1], _fmt(d["yaw_sigma_deg"], "%.2f"))
            if sig else "--", sig is not None)
        f["vel"].show_value(_fmt(d["vel_ned"], "%8.2f"), d["vel_ned"] is not None)
        v = d["vel_ned"]
        if v is not None:
            speed = math.hypot(v[0], v[1])
            # Below walking pace the direction of a velocity is noise, and
            # a track angle printed from it swings through the whole
            # circle while the platform stands still.
            track = ("%6.1f deg" % (math.degrees(math.atan2(v[1], v[0])) % 360.0)
                     if speed > TRACK_MIN_SPEED_MPS else "--")
            f["speed"].show_value("%6.2f m/s   %s" % (speed, track))
        else:
            f["speed"].show_value("--", False)
        f["pos"].show_value(_fmt(d["pos_ned"], "%9.2f"), d["pos_ned"] is not None)
        f["lat"].show_value(_fmt(d["lat_deg"], "%.7f"), d["lat_deg"] is not None)
        f["lon"].show_value(_fmt(d["lon_deg"], "%.7f"), d["lon_deg"] is not None)
        f["height"].show_value(_fmt(d["height_m"], "%.2f"),
                               d["height_m"] is not None)
        f["height_ell"].show_value(_fmt(d["height_ell_m"], "%.1f"),
                                   d["height_ell_m"] is not None)
        # The datum offset the filter is holding, spelled out rather than
        # left as the difference of two numbers three rows apart. It is
        # the quantity that converges after a fix and the one that drifts
        # away from one, so it is worth seeing on its own.
        both = d["height_ell_m"] is not None and d["height_m"] is not None
        f["ell_off"].show_value(
            "%.2f" % (d["height_ell_m"] - d["height_m"]) if both else "--",
            both)
        f["baro"].show_value("%s  /  %s" % (_fmt(d["baro_alt_m"], "%.2f"),
                                            _fmt(d["baro_meas_m"], "%.1f")),
                             d["baro_alt_m"] is not None)
        f["vz"].show_value(_fmt(d["vz_mps"], "%.2f"), d["vz_mps"] is not None)

        active = [n for n, on in (("ZUPT", d["zupt"]), ("ZARU", d["zaru"]),
                                  ("VZUPT", d["vzupt"])) if on]
        f["zupd"].show_value(" ".join(active) or "none", bool(active))
        f["lever"].show_value(
            "%s   %s" % (_fmt(d["leverarm_frd"], "%6.3f"),
                         "BAD" if d["leverarm_bad"] else
                         ("configured" if d["leverarm_set"] else "unset")),
            d["leverarm_set"], "bad" if d["leverarm_bad"] else None)
        f["mag"].show_value("%s uT   %s deg" % (_fmt(d["mag_ref_uT"], "%.1f"),
                                                _fmt(d["mag_decl_deg"], "%.2f")),
                            d["mag_ref_uT"] is not None)

        notes = []
        if d["height_ell_m"] is not None and d["mode"] != "full":
            # Absolute only in the sense that it refers to the ellipsoid.
            # Away from a fresh fix it is the local height plus an
            # estimated offset, and that offset converges over tens of
            # seconds - a valid value can be a long way out early on.
            notes.append("the ellipsoidal height comes from an estimated "
                         "offset while the INS is not under fresh GNSS, and "
                         "that offset converges over tens of seconds.")
        if d["leverarm_bad"]:
            notes.append("a lever arm is stored and is not usable - the "
                         "filter is running on zero.")
        self.note.setText("  ".join(notes))

        t, vals = ring_window(self.att, self.window_s)
        if t is not None:
            self.c_rp[0].setData(t, vals[:, 0])
            self.c_rp[1].setData(t, vals[:, 1])
            # connect="finite" is what makes break_wraps() worth doing:
            # the blanked samples become gaps rather than zeros.
            self.c_yaw.setData(t, break_wraps(vals[:, 2]), connect="finite")

        t, vals = ring_window(self.head, self.window_s)
        if t is not None:
            self.c_head_veh.setData(t, break_wraps(vals[:, 0]),
                                    connect="finite")
            self.c_head_mot.setData(t, break_wraps(vals[:, 1]),
                                    connect="finite")

        t, vals = ring_window(self.sig, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_sig):
                c.setData(t, vals[:, i], connect="finite")

        t, vals = ring_window(self.vel, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_vel):
                c.setData(t, vals[:, i])
            self.c_speed.setData(t, np.hypot(vals[:, 0], vals[:, 1]))

        t, vals = ring_window(self.pos, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_pos):
                c.setData(t, vals[:, i])


# ===========================================================================
# Vertical channel
# ===========================================================================

class Lamp(QtWidgets.QLabel):
    """An indicator that is lit or not, and says what it stands for.

    Not a plot: whether an update is arming is a state with two values,
    and a trace of it would need as much room as the three that carry
    numbers while saying less than a word does.
    """

    def __init__(self, caption, hint=""):
        super().__init__()
        self.caption = caption
        self.on = False
        if hint:
            self.setToolTip(hint)
        themed(self._restyle)

    def set_on(self, on):
        if bool(on) != self.on:
            self.on = bool(on)
            self._restyle()

    def _restyle(self):
        self.setText("\u25cf %s" % self.caption)
        self.setStyleSheet(MONO + " color: %s;"
                           % T["ok" if self.on else "idle"])


class BaroAltTab(QtWidgets.QWidget):
    """The vertical channel, and the datums it is expressed against.

    Height and the rate it is changing at - the state of `baro_alt` - over
    the SAME axis and locked together, because what is asked of that
    filter is how the two line up.

    Behind the second page are the offsets between the three vertical
    datums in play: the local NED origin the filter counts from, the
    WGS84 ellipsoid the receiver reports against, and whatever pressure
    altitude the barometer happens to be on today. Those offsets are
    estimated states, they converge and they drift, and every one of the
    heights on the Solution tab is one of them added to something.

    The height plot never zooms in closer than ZOOM_MIN_M, and the rate
    plot no closer than ZOOM_MIN_VZ_MPS. Half a metre of barometric noise
    stretched over the full height of a plot reads as a unit that is
    bouncing, and at standstill that is precisely what an autoscaled axis
    draws - the rate does the same thing a thousand times smaller, where
    a millimetre per second of filter noise fills the axis and hides the
    metre-per-second excursion that actually matters.
    """

    ZOOM_MIN_M = 2.0
    ZOOM_MIN_VZ_MPS = 1.0    # full span, i.e. never tighter than +/- 0.5 m/s

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.alt = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 2)
        self.vz = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 1)
        # ellipsoidal offset, barometric offset
        self.off = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 2)
        # INS ellipsoidal height minus the receiver's, and the accuracy
        # the receiver claims for its own.
        self.gnss = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 2)
        self.last = None
        self._build()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        bar = QtWidgets.QHBoxLayout()
        self.state = QtWidgets.QLabel("waiting for a navigation message")
        self.state.setStyleSheet(MONO)
        bar.addWidget(self.state, 1)
        self.lamps = {}
        for key, caption, hint in (
                ("vzupt", "VZUPT", "Vertical zero velocity update: the one "
                                   "that fuses into this channel."),
                ("zupt", "ZUPT", "Zero velocity update: the filter is being "
                                 "told the platform is standing still."),
                ("zaru", "ZARU", "Zero angular rate update.")):
            lamp = Lamp(caption, hint)
            self.lamps[key] = lamp
            bar.addWidget(lamp)
        v.addLayout(bar)

        self.inner = QtWidgets.QTabWidget()
        self.inner.addTab(self._page_channel(), "Channel")
        self.inner.addTab(self._page_offsets(), "Datum offsets")
        v.addWidget(self.inner, 1)

    def _page_channel(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.p_alt = make_plot("Height, barometric channel", "m", None,
                               legend=True)
        self.c_alt = themed_curve(self.p_alt, "accent", "baro_alt", width=2)
        self.c_raw = themed_curve(self.p_alt, "idle", "its raw input")
        self.p_vz = make_plot("Vertical rate, positive up", "m/s",
                              "seconds before now")
        self.c_vz = themed_curve(self.p_vz, "ok", width=2)
        # A fixed +/-0.5 m/s span is dominated by the filter's own noise
        # at standstill; the auto SI prefix would then rescale the axis
        # into millimetres per second and label it in multiples of 0.001,
        # so it is switched off in favour of a plain m/s reading.
        self.p_vz.getAxis("left").enableAutoSIPrefix(False)
        for plot in (self.p_alt, self.p_vz):
            v.addWidget(plot, 1)
        # One axis for both. Reading where a step in one falls against
        # the other is the whole job here, and two plots that scroll
        # independently cannot be read that way.
        self.p_vz.setXLink(self.p_alt)
        return page

    def _page_offsets(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)

        self.off_fields = {}
        grid = QtWidgets.QGridLayout()
        for col, (label, key) in enumerate(
                (("ellipsoidal offset [m]", "ell"),
                 ("barometric offset [m]", "baro"),
                 ("INS minus receiver [m]", "resid"),
                 ("receiver vertical accuracy [m]", "vacc"))):
            cap = QtWidgets.QLabel(label)
            themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
            grid.addWidget(cap, 0, col)
            f = Field()
            self.off_fields[key] = f
            grid.addWidget(f, 1, col)
        v.addLayout(grid)

        self.p_off = make_plot("Datum offsets", "m", None, legend=True)
        self.c_off = themed_curves(self.p_off, ("ellipsoidal", "barometric"))
        v.addWidget(self.p_off, 1)

        self.p_resid = make_plot(
            "INS ellipsoidal height minus the receiver's", "m",
            "seconds before now", legend=True)
        self.c_resid = themed_curve(self.p_resid, "accent",
                                    "difference", width=2)
        self.c_vacc_hi = themed_curve(self.p_resid, "idle",
                                      "receiver 1-sigma")
        self.c_vacc_lo = themed_curve(self.p_resid, "idle")
        v.addWidget(self.p_resid, 1)
        self.p_resid.setXLink(self.p_off)

        note = QtWidgets.QLabel(
            "Three vertical datums and the offsets between them. The "
            "ELLIPSOIDAL offset is what the filter adds to its local "
            "height to get a WGS84 one: it is an estimated state, it "
            "converges over tens of seconds after a fix and it drifts "
            "away from one, and a height that looks absolute is that "
            "offset plus a local number. The BAROMETRIC offset is the raw "
            "pressure height minus the fused one, which is the weather "
            "moving under a platform that is not.\n"
            "The lower plot is the only external check available on the "
            "link: the receiver's height is against the ellipsoid too, so "
            "the difference should sit inside its own accuracy figure. A "
            "constant offset that survives a converged filter is usually "
            "the lever arm's down component - the antenna is not where "
            "the IMU is, and an unmeasured arm puts that distance here.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self.state.setText("not connected")

    def on_batch(self, batch):
        d = batch.get("nav")
        if d:
            self.last = d
            if d["baro_alt_m"] is not None:
                raw = d["baro_meas_m"]
                self.alt.add(batch["t"], (d["baro_alt_m"],
                                          np.nan if raw is None else raw))
            if d["vz_mps"] is not None:
                self.vz.add(batch["t"], (d["vz_mps"],))
            ell = (d["height_ell_m"] - d["height_m"]
                   if d["height_ell_m"] is not None and d["height_m"] is not None
                   else np.nan)
            baro = (d["baro_meas_m"] - d["baro_alt_m"]
                    if d["baro_meas_m"] is not None
                    and d["baro_alt_m"] is not None else np.nan)
            if np.isfinite(ell) or np.isfinite(baro):
                self.off.add(batch["t"], (ell, baro))

        if self.last and self.last["height_ell_m"] is not None:
            for fix in batch.get("pvt", ()):
                # Both heights are against the WGS84 ellipsoid, which is
                # what makes the difference meaningful. A fix the receiver
                # itself does not stand behind is not a reference for
                # anything, so it is dropped rather than differenced.
                if fix["fix_ok"] and fix["fix_type"] >= 3:
                    self.gnss.add(batch["t"],
                                  (self.last["height_ell_m"] - fix["height_m"],
                                   fix["vacc_m"]))

    def refresh(self):
        d = self.last
        for key, lamp in self.lamps.items():
            lamp.set_on(bool(d and d[key]))

        if d is None:
            self.state.setText("no navigation message - is CFG-MSGOUT-NAV on?")
        else:
            self.state.setText(
                "height %s m   rate %s m/s   raw baro %s m   %s"
                % (_fmt(d["baro_alt_m"], "%.2f"), _fmt(d["vz_mps"], "%.2f"),
                   _fmt(d["baro_meas_m"], "%.1f"),
                   "vertical channel running" if d["baro_alt_m"] is not None
                   else "NO vertical channel - is a barometer configured?"))

        t, vals = ring_window(self.alt, self.window_s)
        if t is not None:
            self.c_alt.setData(t, vals[:, 0])
            # The raw input is absent in some modes; blanked rather than
            # drawn at zero, and connect="finite" lifts the pen over it.
            self.c_raw.setData(t, vals[:, 1], connect="finite")
            self._zoom(self.p_alt, vals, self.ZOOM_MIN_M)
        t, vals = ring_window(self.vz, self.window_s)
        if t is not None:
            self.c_vz.setData(t, vals[:, 0])
            self._zoom(self.p_vz, vals, self.ZOOM_MIN_VZ_MPS)
        # Held rather than autoscaled, so that both plots show the
        # look-back that was asked for even when one of them is empty.
        self.p_alt.setXRange(-self.window_s, 0.0, padding=0.02)
        self._refresh_offsets()

    def _refresh_offsets(self):
        t, vals = ring_window(self.off, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_off):
                c.setData(t, vals[:, i], connect="finite")
            for key, col in (("ell", 0), ("baro", 1)):
                v = float(vals[-1, col])
                self.off_fields[key].show_value(
                    "%.2f" % v if np.isfinite(v) else "--", np.isfinite(v))

        t, vals = ring_window(self.gnss, self.window_s)
        if t is None:
            self.off_fields["resid"].show_value("--", False)
            self.off_fields["vacc"].show_value("--", False)
        else:
            self.c_resid.setData(t, vals[:, 0])
            self.c_vacc_hi.setData(t, vals[:, 1])
            self.c_vacc_lo.setData(t, -vals[:, 1])
            resid, vacc = float(vals[-1, 0]), float(vals[-1, 1])
            self.off_fields["resid"].show_value(
                "%+.2f" % resid, True,
                # Outside the receiver's own 1-sigma is not a fault by
                # itself - one sample in three is, by construction - but
                # a difference that sits outside it is the lever arm or
                # the datum, and neither goes away by waiting.
                "warn" if vacc > 0.0 and abs(resid) > vacc else None)
            self.off_fields["vacc"].show_value("%.2f" % vacc, True)
        self.p_off.setXRange(-self.window_s, 0.0, padding=0.02)

    def _zoom(self, plot, vals, min_span):
        """Centre one y axis on what is there, never closer than
        min_span."""
        finite = vals[np.isfinite(vals)]
        if finite.size == 0:
            return
        lo, hi = float(finite.min()), float(finite.max())
        span = max((hi - lo) * 1.1, min_span)
        mid = (lo + hi) / 2.0
        plot.setYRange(mid - span / 2.0, mid + span / 2.0, padding=0)


# ===========================================================================
# Diagnostics
# ===========================================================================

def themed_curve_keys(plot, keys, names):
    """One curve per palette KEY, for a plot with more series than the
    three-axis triple covers."""
    curves = [plot.plot(pen=pg.mkPen(T[k], width=1), name=n)
              for k, n in zip(keys, names)]
    themed(lambda: [c.setPen(pg.mkPen(T[k], width=1))
                    for k, c in zip(keys, curves)])
    return curves


class DiagnosticsTab(QtWidgets.QWidget):
    """Whether the box is keeping up, and what it has survived.

    Four different clocks feed this. The load and the fusion worst case
    ride the navigation message ten times a second; the counters arrive
    every five seconds; the acquisition diagnostics arrive only when
    something changes, plus a heartbeat. A saturation episode arrives when
    it happens and nowhere else, which is why they are kept as a list.
    The message rates are the host's own count of what came off the link,
    which is the only figure here the device does not supply.

    The counters answer "has it ever" and the plots answer "is it now".
    A worst case since boot cannot say whether the margin is being eaten
    into over an hour, and a load that touches its limit for one second
    in sixty does not move a counter at all.
    """

    MAX_EPISODES = 200

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.fields = {}
        self.nav = None
        self.status = None
        self.health = None
        self.load = Ring(HISTORY_MAX_S * RATE_NAV_HZ, 2)
        # One row a second: five message rates and the link throughput.
        self.rates = Ring(HISTORY_MAX_S + 4, 6)
        self._rate_t0 = None
        self._counts = np.zeros(6)
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        left = QtWidgets.QVBoxLayout()
        outer.addLayout(left, 2)

        for title, rows in (
                ("Timing", (("processor load", "cpu"),
                            ("fusion worst case", "wcet"),
                            ("IMU sample rate", "imu_hz"))),
                ("Link", (("throughput", "throughput"),
                          ("dropped to the host", "vcp_dropped"),
                          ("dropped TX frames", "tx_dropped"),
                          ("IMU overruns", "imu_overruns"),
                          ("loop passes / 5 s", "loop_count"))),
                ("GNSS link", (("bytes received", "gnss_rx_bytes"),
                               ("UART errors", "uart_errors"),
                               ("RX overflows", "gnss_rx_overflows"),
                               ("CRC errors", "gnss_crc_errors"),
                               ("dropped towards the F9P", "gnss_tx_dropped"))),
                ("IMU acquisition", (("stalls / re-inits / failed", "stalls"),
                                     ("SPI errors", "spi_errors"),
                                     ("first stall at", "at_ms"),
                                     ("registers then", "regs"))),
                ("Measurement range", (("axes ever clipped", "sat"),
                                       ("clipped samples", "sat_samples")))):
            box = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout(box)
            grid.setColumnStretch(1, 1)
            for label, key in rows:
                r = grid.rowCount()
                cap = QtWidgets.QLabel(label)
                themed(lambda w=cap: w.setStyleSheet(dim()))
                grid.addWidget(cap, r, 0)
                fld = Field()
                self.fields[key] = fld
                grid.addWidget(fld, r, 1)
            left.addWidget(box)
        left.addStretch(1)

        inner = QtWidgets.QTabWidget()
        inner.addTab(self._page_episodes(), "Saturation episodes")
        inner.addTab(self._page_load(), "Load")
        inner.addTab(self._page_link(), "Link")
        outer.addWidget(inner, 3)

    def _page_episodes(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.episodes = QtWidgets.QTableWidget(0, 4)
        self.episodes.setHorizontalHeaderLabels(
            ["first seq", "last seq", "samples", "axes"])
        self.episodes.horizontalHeader().setStretchLastSection(True)
        self.episodes.setStyleSheet(MONO)
        v.addWidget(self.episodes, 1)

        cap = QtWidgets.QLabel(
            "An episode is a run of consecutive samples in which an axis sat "
            "at the sensor end stop. Those samples are not measurements: the "
            "reading is the end of the range, it looks entirely plausible, "
            "and the filter fuses it as one. Empty here is the answer you "
            "want.")
        cap.setWordWrap(True)
        themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(cap)
        return page

    def _page_load(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.p_cpu = make_plot("Processor load", "%", None)
        self.c_cpu = themed_curve(self.p_cpu, "accent", width=2)
        self.p_cpu.setYRange(0.0, 100.0)
        self.l_cpu = self.p_cpu.addLine(y=80.0)

        self.p_wcet = make_plot("Worst fusion epoch", "us",
                                "seconds before now")
        self.c_wcet = themed_curve(self.p_wcet, "ok", width=2)
        # The budget one fusion epoch actually has: the interval between
        # two IMU samples. Set from the reported rate rather than assumed,
        # and hidden until the board has said what that rate is.
        self.l_budget = self.p_wcet.addLine(y=0.0)
        self.l_budget.setVisible(False)
        themed(lambda: [ln.setPen(pg.mkPen(
            T["warn"], style=QtCore.Qt.PenStyle.DashLine))
            for ln in (self.l_cpu, self.l_budget)])
        for p in (self.p_cpu, self.p_wcet):
            v.addWidget(p, 1)
        self.p_wcet.setXLink(self.p_cpu)

        cap = QtWidgets.QLabel(
            "Both ride the navigation message, so they stop when it does. "
            "The worst case is the board's own running maximum since boot "
            "and it therefore only ever climbs: a step in it is one epoch "
            "that took longer than every epoch before it, which is worth "
            "knowing about even though the load beside it never moved.")
        cap.setWordWrap(True)
        themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(cap)
        return page

    RATE_SERIES = (("IMU", "imu"), ("magnetometer", "mag"),
                   ("barometer", "baro"), ("navigation", "nav"),
                   ("receiver PVT", "pvt"))

    def _page_link(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.p_rates = make_plot("Messages arriving", "Hz", None, legend=True)
        self.c_rates = themed_curve_keys(
            self.p_rates, ("accent", "ok", "warn", "changed", "bad"),
            [n for n, _ in self.RATE_SERIES])
        self.p_bytes = make_plot("Link throughput", "kB/s",
                                 "seconds before now")
        self.c_bytes = themed_curve(self.p_bytes, "accent", width=2)
        for p in (self.p_rates, self.p_bytes):
            v.addWidget(p, 1)
        self.p_bytes.setXLink(self.p_rates)

        cap = QtWidgets.QLabel(
            "Counted on this host over one second windows, from framed "
            "messages only - bytes that did not resolve into a frame are "
            "not in the throughput. A rate that sags while the board says "
            "its load is fine is the link and not the filter. The IMU rate "
            "here is what ARRIVED, which is the sensor rate divided by "
            "whatever transmit decimation the firmware was built with, and "
            "it drops when a configuration exchange flushes the input "
            "buffer.")
        cap.setWordWrap(True)
        themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(cap)
        return page

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self._rate_t0 = None
            self._counts = np.zeros(6)

    def on_batch(self, batch):
        if batch["nav"]:
            self.nav = batch["nav"]
            self.load.add(batch["t"], (self.nav["cpu_pct"],
                                       self.nav["update_wcet_us"]))
        if batch["status"]:
            self.status = batch["status"]
        if batch["health"]:
            self.health = batch["health"]
        self._count(batch)
        for ep in batch["satur"]:
            r = self.episodes.rowCount()
            if r >= self.MAX_EPISODES:
                self.episodes.removeRow(0)
                r -= 1
            self.episodes.insertRow(r)
            cells = ("%d" % ep["first_seq"], "%d" % ep["last_seq"],
                     "%d%s" % (ep["samples"],
                               "  (earlier ones were lost)"
                               if ep["dropped_before"] else ""),
                     " ".join(ep["axes"]))
            for c, text in enumerate(cells):
                self.episodes.setItem(r, c, QtWidgets.QTableWidgetItem(text))
            self.episodes.scrollToBottom()

    def _count(self, batch):
        """Message counts into one second windows.

        The window is closed by the host clock and not by a sample count,
        so a stream that stops produces a rate of zero rather than a gap
        that the eye reads as "still going"."""
        if self._rate_t0 is None:
            self._rate_t0 = batch["t"]
        self._counts += (len(batch["imu"]), len(batch["mag"]),
                         len(batch["baro"]), 1 if batch["nav"] else 0,
                         len(batch["pvt"]), batch.get("bytes", 0))
        span = batch["t"] - self._rate_t0
        if span < 1.0:
            return
        row = self._counts / span
        row[5] /= 1000.0                     # bytes/s -> kB/s
        self.rates.add(batch["t"], row)
        self._rate_t0 = batch["t"]
        self._counts = np.zeros(6)

    def refresh(self):
        f = self.fields
        if self.nav:
            d = self.nav
            f["cpu"].show_value("%d %%" % d["cpu_pct"], True,
                                "bad" if d["cpu_pct"] >= 80 else None)
            # Held against the interval between two IMU samples, which is
            # the budget one fusion epoch actually has.
            budget = 1e6 / d["imu_hz"] if d["imu_hz"] else None
            wcet = d["update_wcet_us"]
            colour = None
            if budget:
                colour = ("bad" if wcet >= budget else
                          "warn" if wcet * 4 >= budget * 3 else None)
                self.l_budget.setValue(budget)
                self.l_budget.setVisible(True)
            f["wcet"].show_value(
                "%d us%s" % (wcet, "  of %d us" % budget if budget else ""),
                True, colour)
            f["imu_hz"].show_value("%d Hz" % d["imu_hz"], True)
            sat = d["sat_axes"]
            f["sat"].show_value(" ".join(sat) if sat else "none",
                                bool(sat), "bad" if sat else None)

        if self.status:
            s = self.status
            for key in ("vcp_dropped", "tx_dropped", "imu_overruns",
                        "loop_count", "gnss_rx_bytes", "uart_errors",
                        "gnss_rx_overflows", "gnss_crc_errors",
                        "gnss_tx_dropped"):
                v = s.get(key)
                if v is None:
                    continue
                # Zero is the good answer for every counter here except
                # the two that are supposed to grow.
                good = key in ("loop_count", "gnss_rx_bytes")
                f[key].show_value("%d" % v, v > 0 if good else True,
                                  None if (good or v == 0) else "bad")

        if self.health:
            h = self.health
            f["stalls"].show_value(
                "%d / %d / %d" % (h["stalls"], h["reinits"], h["reinit_fails"]),
                True, "bad" if h["stalls"] else None)
            f["spi_errors"].show_value(
                "%d" % h["spi_errors"], True,
                # The firmware recovers from a receive overrun by itself,
                # so the outage disappears and the counter does not. A
                # rising count with no stalls means the timing margin is
                # being eaten into while the recovery still holds.
                "warn" if h["spi_errors"] else None)
            if h["stalls"]:
                f["at_ms"].show_value("%.1f min uptime" % (h["at_ms"] / 60000.0))
                f["regs"].show_value(
                    "pwr %d  int1 %d  whoami %d  rc %d"
                    % (h["reg_pwr_mgmt0"], h["reg_int1_cfg0"],
                       h["reg_whoami"], h["reg_read_rc"]), True, "bad")
            else:
                f["at_ms"].show_value("never", False)
                f["regs"].show_value("--", False)
            if "sat_samples" in h:
                f["sat_samples"].show_value(
                    "%d" % h["sat_samples"], True,
                    "bad" if h["sat_samples"] else None)

        t, vals = ring_window(self.load, self.window_s)
        if t is not None:
            self.c_cpu.setData(t, vals[:, 0])
            self.c_wcet.setData(t, vals[:, 1])

        t, vals = ring_window(self.rates, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_rates):
                c.setData(t, vals[:, i])
            self.c_bytes.setData(t, vals[:, 5])
            f["throughput"].show_value("%.1f kB/s" % vals[-1, 5], True)


# ===========================================================================
# Track
# ===========================================================================

EARTH_R = 6378137.0

# OpenStreetMap asks every application that uses its tiles to identify
# itself and not to bulk download. Both are honoured here: the agent names
# this tool, tiles are fetched only for what is on screen, never ahead of
# it, at most TILE_BUDGET at a time, and everything fetched is kept on
# disk so a second look costs nothing.
# See https://operations.osmfoundation.org/policies/tiles/
TILE_URL = "https://tile.openstreetmap.org/%d/%d/%d.png"
TILE_AGENT = "inslib_gui/1.0 (INSLIB device console; local diagnostics)"
TILE_BUDGET = 24
TILE_CACHE = os.path.join(
    os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/.cache"),
    "inslib_gui", "tiles")


def merc(lat_deg, lon_deg):
    """WGS84 -> Web Mercator metres, the frame the tiles are cut in."""
    lat = max(-85.05112878, min(85.05112878, lat_deg))
    return (EARTH_R * math.radians(lon_deg),
            EARTH_R * math.log(math.tan(math.pi / 4.0
                                        + math.radians(lat) / 2.0)))


class TileFetcher(QtCore.QThread):
    """Fetches map tiles off the UI thread, cache first.

    One thread, a queue, and no prefetching: a map view that scrolled
    would otherwise turn into exactly the bulk download the tile policy
    asks applications not to do.
    """

    ready = QtCore.pyqtSignal(int, int, int, bytes)

    def __init__(self):
        super().__init__()
        self.q = queue.Queue()
        # What is IN FLIGHT, so the same square is not queued twice while
        # it is being fetched. Not a record of everything ever asked for:
        # a view that comes back to a tile has to be able to ask again.
        self.pending = set()
        self._stop = False

    def want(self, z, x, y):
        if (z, x, y) in self.pending:
            return
        self.pending.add((z, x, y))
        self.q.put((z, x, y))

    def stop(self):
        self._stop = True
        self.q.put(None)

    def run(self):
        import urllib.request
        while not self._stop:
            item = self.q.get()
            if item is None:
                break
            z, x, y = item
            path = os.path.join(TILE_CACHE, str(z), str(x), "%d.png" % y)
            try:
                if os.path.exists(path):
                    with open(path, "rb") as fh:
                        data = fh.read()
                else:
                    req = urllib.request.Request(
                        TILE_URL % (z, x, y),
                        headers={"User-Agent": TILE_AGENT})
                    with urllib.request.urlopen(req, timeout=8) as r:
                        data = r.read()
                    os.makedirs(os.path.dirname(path), exist_ok=True)
                    with open(path, "wb") as fh:
                        fh.write(data)
                self.ready.emit(z, x, y, data)
            except Exception:                             # noqa: BLE001
                # A missing tile is a blank square, not an error dialog:
                # the track underneath is the point of the view and it is
                # still there.
                pass
            finally:
                self.pending.discard((z, x, y))


class TrackTab(QtWidgets.QWidget):
    """Where the box thinks it is, and where the receiver thinks it is.

    Both against the same origin, in METRES, so the distance between them
    reads off the axes directly. That difference is the thing worth
    looking at: the receiver's position is an INPUT to the filter, and
    where the two part company is where the lever arm, the mounting or the
    aiding gates are wrong.

    The metres are on the Web Mercator sphere - radius 6378137 m, the one
    the tiles are cut on - with its 1/cos(latitude) stretch taken out at
    the origin. That is what lets one coordinate system carry an aligned
    map AND readable distances, and it is not free: against the WGS84
    ellipsoid at mid latitudes a north span comes out about 0.1 % long
    and an east span about 0.19 % short, so one to two metres per
    kilometre, with about 0.3 % anisotropy between the axes.

    The SEPARATION between the two tracks is unaffected by all of that,
    because both go through the same projection - which is fortunate,
    since the separation is what this view is for.

    Two arrows sit at the newest position: where the platform POINTS,
    out of the filter's yaw, and where it is GOING, out of its velocity.
    A line of positions is a record of the second one and says nothing
    about the first, so the angle between the arrows - the crab, printed
    next to them - is the one thing on this view that a track alone
    cannot show. Both are drawn a fixed fraction of the view long rather
    than a fixed number of metres, so they read the same at every zoom.

    The street map is optional and off until switched on. Tiles need a
    network, and asking for them tells somebody else's server which area
    is being looked at - which is to say where this device is. That is a
    decision for whoever is holding the box, not a default.
    """

    MAX_POINTS = 20000
    # Decoded tiles kept in memory. A zoom out and back in is two clicks
    # and would otherwise be two trips to the disk cache per square, so
    # keeping the pixels is what makes going back free. Well above
    # TILE_BUDGET, so what is on screen is never the thing that gets
    # dropped.
    TILE_KEEP = 96

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.origin = None
        self.origin_merc = None
        self.scale = 1.0        # local metres per Mercator metre at the origin
        self.ins = Ring(self.MAX_POINTS, 2)
        # east, north and the accuracy the receiver claims for that fix.
        self.gnss = Ring(self.MAX_POINTS, 3)
        self.fetcher = None
        self.tiles = {}         # (z, x, y) -> the item on the plot
        self.images = {}        # (z, x, y) -> the decoded pixels
        self.want = set()       # what the current view is made of
        # Where the platform points, and where it is going. Both from the
        # filter, both blanked rather than stale when it stops saying.
        self.yaw = None
        self.course = None
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        bar = QtWidgets.QHBoxLayout()
        self.info = QtWidgets.QLabel("waiting for a position")
        self.info.setStyleSheet(MONO)
        bar.addWidget(self.info, 1)
        self.map_on = QtWidgets.QCheckBox("OpenStreetMap background")
        self.map_on.setToolTip(
            "Fetches map tiles from tile.openstreetmap.org for the area on "
            "screen. That tells their server where you are looking.")
        self.map_on.toggled.connect(self._map_toggled)
        bar.addWidget(self.map_on)
        btn = QtWidgets.QPushButton("Clear")
        btn.clicked.connect(self.clear)
        bar.addWidget(btn)
        v.addLayout(bar)

        self.plot = make_plot(None, "north of the first fix [m]",
                              "east of the first fix [m]", legend=True)
        # Locked 1:1, or the shape of the track is a lie and the gap
        # between the two tracks cannot be read off the axes.
        self.plot.setAspectLocked(True)
        self.c_gnss = themed_curve(self.plot, "accent", "receiver",
                                   symbol="o", size=4)
        self.c_ins = themed_curve(self.plot, "ok", "filter", width=2)
        self.c_here = themed_curve(self.plot, "warn", "now",
                                   symbol="+", size=16)
        # The receiver's own horizontal accuracy, drawn where it applies
        # rather than printed as a number somewhere else. A separation
        # that stays inside this circle is the two agreeing; one that
        # leaves it is a disagreement the receiver did not expect either.
        self.c_hacc = themed_curve(self.plot, "idle",
                                   "receiver accuracy")
        # The current separation, as the segment it is. A distance in a
        # status line says how far apart they are and this says which way,
        # which is the half that names the cause.
        self.c_sep = themed_curve(self.plot, "bad", "separation", width=2)
        # Where the nose POINTS, drawn from where it points from. The
        # track behind it says where the platform has BEEN, and the two
        # are different quantities: the angle between them is the crab,
        # and none of it is visible in a line of positions alone.
        self.c_yaw = themed_overlay(self.plot, "changed", "heading", width=2)
        # Course over ground out of the filter's own velocity, dashed and
        # longer, the same pairing the compass on the Solution tab uses.
        self.c_course = themed_overlay(self.plot, "accent", "course",
                                       width=2, dashed=True)
        v.addWidget(self.plot, 1)

        # A zoom or a pan changes which tiles are on screen and how long
        # an arrow has to be to stay readable, and neither can wait for
        # the next refresh - with the link stopped there is no next
        # refresh at all. Coalesced, because the signal arrives once per
        # wheel notch and once per mouse move while dragging.
        self.view_timer = QtCore.QTimer(self)
        self.view_timer.setSingleShot(True)
        self.view_timer.setInterval(120)
        self.view_timer.timeout.connect(self._view_settled)
        self.plot.getPlotItem().sigRangeChanged.connect(
            lambda *_a: self.view_timer.start())

        self.note = QtWidgets.QLabel(
            "Metres east and north of the first position seen, on the Web "
            "Mercator sphere so that map tiles align. Absolute spans run "
            "1 to 2 m per km off against the WGS84 ellipsoid; the "
            "separation between the two tracks does not, since both go "
            "through the same projection.")
        self.note.setWordWrap(True)
        themed(lambda: self.note.setStyleSheet(
            dim() + " font-size: 11px;"))
        v.addWidget(self.note)

    # -- projection --------------------------------------------------------

    def _project(self, lat, lon):
        """(east, north) in metres against the first position seen."""
        if self.origin is None:
            self.origin = (lat, lon)
            self.origin_merc = merc(lat, lon)
            # Web Mercator stretches by 1/cos(latitude), isotropically over
            # a small area. Undoing that here is what lets one coordinate
            # system carry both true distances AND aligned tiles.
            self.scale = math.cos(math.radians(lat))
        mx, my = merc(lat, lon)
        return ((mx - self.origin_merc[0]) * self.scale,
                (my - self.origin_merc[1]) * self.scale)

    def _to_merc(self, east, north):
        return (self.origin_merc[0] + east / self.scale,
                self.origin_merc[1] + north / self.scale)

    def _view_rect(self):
        """What is on screen, in the plot's own metres.

        The box is named rather than left to the widget: a PlotWidget
        forwards viewRect() to its ViewBox, but it is also a GraphicsView
        with a viewRect() of its own that answers in scene pixels. Saying
        which of the two is meant costs one call and removes the question.
        """
        return self.plot.getViewBox().viewRect()

    def clear(self):
        self.origin = None
        self.ins = Ring(self.MAX_POINTS, 2)
        self.gnss = Ring(self.MAX_POINTS, 3)
        self.yaw = None
        self.course = None
        for c in (self.c_ins, self.c_gnss, self.c_here, self.c_hacc,
                  self.c_sep, self.c_yaw, self.c_course):
            c.setData([], [])
        # Every tile rect is measured against the origin, and the next
        # position seen sets a new one. Keeping the pictures would keep
        # them where the OLD origin put them.
        self._drop_tiles()
        self.images.clear()

    # -- data --------------------------------------------------------------

    def on_batch(self, batch):
        d = batch.get("nav")
        if d:
            rpy = d["rpy_deg"]
            self.yaw = rpy[2] if rpy else None
            vel = d["vel_ned"]
            # Below walking pace the direction of a velocity is noise and
            # a course drawn from it spins while the platform stands
            # still, so it is blanked rather than drawn.
            self.course = (
                math.degrees(math.atan2(vel[1], vel[0]))
                if vel and math.hypot(vel[0], vel[1]) > TRACK_MIN_SPEED_MPS
                else None)
        if d and d["lat_deg"] is not None:
            self.ins.add(batch["t"], self._project(d["lat_deg"], d["lon_deg"]))
        for fix in batch.get("pvt", ()):
            # Without a fix the receiver still reports a position: whatever
            # the last search left behind. Those look like measurements,
            # which is why they are dropped rather than plotted faintly.
            if fix["fix_ok"] and fix["fix_type"] >= 2:
                east, north = self._project(fix["lat_deg"], fix["lon_deg"])
                self.gnss.add(batch["t"], (east, north, fix["hacc_m"]))

    def refresh(self):
        here = None
        for ring, curve in ((self.gnss, self.c_gnss), (self.ins, self.c_ins)):
            _t, xy = ring.view()
            if xy is None:
                continue
            curve.setData(xy[:, 0], xy[:, 1])
            here = xy[-1]
        if here is not None:
            self.c_here.setData([here[0]], [here[1]])

        bits = []
        if self.origin:
            bits.append("origin %.7f %.7f" % self.origin)
        _t, g = self.gnss.view()
        _t, i = self.ins.view()
        bits.append("receiver %d" % (0 if g is None else len(g)))
        bits.append("filter %d" % (0 if i is None else len(i)))
        if g is not None:
            hacc = float(g[-1, 2])
            bits.append("accuracy %.2f m" % hacc)
            self.c_hacc.setData(*circle_pts(float(g[-1, 0]), float(g[-1, 1]),
                                            hacc))
        if g is not None and i is not None:
            sep = float(np.hypot(*(i[-1, :2] - g[-1, :2])))
            bits.append("separation %.2f m" % sep)
            self.c_sep.setData([g[-1, 0], i[-1, 0]], [g[-1, 1], i[-1, 1]])
        if self.yaw is not None:
            bits.append("heading %.1f deg" % (self.yaw % 360.0))
        if self.course is not None:
            bits.append("course %.1f deg" % (self.course % 360.0))
            if self.yaw is not None:
                # The crab angle. On a platform that cannot travel
                # sideways it is not a crab at all, it is the heading
                # error, which is why it is printed here between the two
                # arrows it is measured off.
                bits.append("crab %+.1f deg"
                            % ((self.course - self.yaw + 180.0) % 360.0
                               - 180.0))
        self.info.setText("   ".join(bits))

        self._draw_heading()
        if self.map_on.isChecked() and self.origin is not None:
            self._update_tiles()

    # -- heading -----------------------------------------------------------

    def _draw_heading(self):
        """The two arrows, at the newest position there is."""
        _t, xy = self.ins.view()
        if xy is None:
            _t, xy = self.gnss.view()
        here = None if xy is None else xy[-1]
        for curve, deg, frac in ((self.c_yaw, self.yaw, 0.11),
                                 (self.c_course, self.course, 0.15)):
            if here is None or deg is None:
                curve.setData([], [])
            else:
                curve.setData(*self._arrow(float(here[0]), float(here[1]),
                                           deg, frac))

    def _arrow(self, east, north, deg, frac):
        """A heading arrow as one polyline, in the plot's own metres.

        Sized against the view rather than against the world, so it reads
        the same at every zoom instead of covering the whole track at one
        and vanishing at the next.
        """
        rect = self._view_rect()
        length = max(abs(rect.width()), abs(rect.height())) * frac
        a = math.radians(deg)
        tip = (east + math.sin(a) * length, north + math.cos(a) * length)
        xs, ys = [east, tip[0]], [north, tip[1]]
        # Back down each barb and out to the tip again: one polyline is
        # one curve, and one curve is one entry in the legend.
        for barb in (150.0, -150.0):
            b = math.radians(deg + barb)
            xs += [tip[0] + math.sin(b) * length * 0.28, tip[0]]
            ys += [tip[1] + math.cos(b) * length * 0.28, tip[1]]
        return xs, ys

    def _view_settled(self):
        """After a zoom or a pan: new arrow length, new tiles."""
        self._draw_heading()
        if self.map_on.isChecked() and self.origin_merc is not None:
            self._update_tiles()

    # -- tiles -------------------------------------------------------------

    def _map_toggled(self, on):
        if on:
            m = QtWidgets.QMessageBox(self)
            m.setWindowTitle("Fetch map tiles?")
            m.setIcon(QtWidgets.QMessageBox.Icon.Question)
            m.setText(
                "This fetches map tiles from tile.openstreetmap.org for the "
                "area on screen.\n\nThat sends the area you are looking at - "
                "which is where this device is - to their server, and needs "
                "a network. Tiles are cached on disk and never fetched ahead "
                "of what is shown.")
            m.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Yes |
                                 QtWidgets.QMessageBox.StandardButton.Cancel)
            m.setDefaultButton(QtWidgets.QMessageBox.StandardButton.Cancel)
            if m.exec() != QtWidgets.QMessageBox.StandardButton.Yes:
                self.map_on.blockSignals(True)
                self.map_on.setChecked(False)
                self.map_on.blockSignals(False)
                return
            if self.fetcher is None:
                self.fetcher = TileFetcher()
                self.fetcher.ready.connect(self._tile_ready)
                self.fetcher.start()
            self.note.setText(
                "Map data (C) OpenStreetMap contributors, "
                "openstreetmap.org/copyright - ODbL. Metres east and north "
                "of the first position seen; absolute spans run 1 to 2 m per "
                "km off, the separation between the tracks does not.")
        else:
            self._drop_tiles()

    def _drop_tiles(self):
        for item in self.tiles.values():
            self.plot.removeItem(item)
        self.tiles.clear()
        self.want = set()

    def _update_tiles(self):
        rect = self._view_rect()
        z = self._zoom_for(rect)
        x0, x1, y0, y1 = self._tile_range(rect, z)
        self.want = {(z, x, y)
                     for x in range(x0, x1 + 1)
                     for y in range(y0, y1 + 1)}
        # Whatever the view has left behind goes: the previous zoom level
        # covers the same ground at a different resolution and two of
        # those on top of each other is not a map, and a pan that keeps
        # every square it has ever shown is a leak with a picture on it.
        # The pixels stay in self.images, so coming back is free.
        for key in [k for k in self.tiles if k not in self.want]:
            self.plot.removeItem(self.tiles.pop(key))
        # Only what is on screen: no prefetch, no bulk.
        for key in sorted(self.want):
            if key in self.tiles:
                continue
            if key in self.images:
                self._show_tile(key)
            elif self.fetcher is not None:
                self.fetcher.want(*key)

    def _tile_range(self, rect, z):
        """The inclusive tile index box a view rect covers.

        A QRectF counts y downwards and the view counts north upwards, so
        which corner gives which tile row depends on a convention that is
        easy to get backwards. Both corners, then min and max: the range
        is right whichever way round the rect came.
        """
        xa, ya = self._tile_of(rect.left(), rect.top(), z)
        xb, yb = self._tile_of(rect.right(), rect.bottom(), z)
        last = 2 ** z - 1
        x0, x1 = self._span(xa, xb, last)
        y0, y1 = self._span(ya, yb, last)
        return x0, x1, y0, y1

    @staticmethod
    def _span(a, b, last):
        """The inclusive tile index range two edges fall in, clamped to
        the world at this zoom."""
        lo = max(0, min(last, int(math.floor(min(a, b)))))
        hi = max(0, min(last, int(math.floor(max(a, b)))))
        return lo, hi

    def _zoom_for(self, rect):
        """The finest zoom whose visible tiles still fit the budget.

        Mercator metres, because that is what a tile measures - the view
        is in local metres, so the width is converted first. About two
        tiles across the view to begin with, then out a level at a time
        until the count fits: a tall window or a wide view then gets a
        coarser map instead of no map at all, which is what a hard cap on
        its own would give.
        """
        span = max(abs(rect.width()) / self.scale, 1.0)
        z = int(round(math.log2(2 * math.pi * EARTH_R * 2.0 / span)))
        z = max(1, min(19, z))
        while z > 1:
            x0, x1, y0, y1 = self._tile_range(rect, z)
            if (x1 - x0 + 1) * (y1 - y0 + 1) <= TILE_BUDGET:
                break
            z -= 1
        return z

    def _tile_of(self, east, north, z):
        mx, my = self._to_merc(east, north)
        n = 2 ** z
        half = math.pi * EARTH_R
        return ((mx + half) / (2 * half) * n,
                (half - my) / (2 * half) * n)

    def _tile_rect(self, z, x, y):
        """The tile's footprint in the plot's own metres."""
        n = 2 ** z
        half = math.pi * EARTH_R
        mx0 = x / n * 2 * half - half
        mx1 = (x + 1) / n * 2 * half - half
        my1 = half - y / n * 2 * half
        my0 = half - (y + 1) / n * 2 * half
        e0 = (mx0 - self.origin_merc[0]) * self.scale
        e1 = (mx1 - self.origin_merc[0]) * self.scale
        n0 = (my0 - self.origin_merc[1]) * self.scale
        n1 = (my1 - self.origin_merc[1]) * self.scale
        return QtCore.QRectF(e0, n0, e1 - e0, n1 - n0)

    def _tile_ready(self, z, x, y, data):
        if self.origin_merc is None:
            return
        img = QtGui.QImage()
        if not img.loadFromData(data):
            return
        key = (z, x, y)
        self._remember(key, _qimage_to_array(img))
        # The view may have moved on while this one was in flight. It is
        # kept either way and shown only if it is still wanted.
        if self.map_on.isChecked() and key in self.want:
            self._show_tile(key)

    def _remember(self, key, arr):
        """Keep the decoded pixels, bounded, on-screen ones kept last."""
        self.images.pop(key, None)
        self.images[key] = arr
        stale = [k for k in self.images if k not in self.tiles and k != key]
        for old in stale[:max(0, len(self.images) - self.TILE_KEEP)]:
            del self.images[old]

    def _show_tile(self, key):
        if key in self.tiles:
            return
        item = pg.ImageItem()
        item.setImage(self.images[key])
        item.setRect(self._tile_rect(*key))
        # Behind the tracks, and dimmed: a street map is context, and a
        # background at full contrast hides the two lines that matter.
        item.setZValue(-100)
        item.setOpacity(0.55)
        # ignoreBounds, or the tile just added enlarges the view, which asks
        # for tiles further out, which enlarge the view again: the map walks
        # away from the track it was fetched for.
        self.plot.addItem(item, ignoreBounds=True)
        self.tiles[key] = item

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()

    def shutdown(self):
        if self.fetcher:
            self.fetcher.stop()
            self.fetcher.wait(2000)
            self.fetcher = None


def _qimage_to_array(img):
    """QImage -> (h, w, 3) uint8, the orientation pyqtgraph draws upright."""
    img = img.convertToFormat(QtGui.QImage.Format.Format_RGB888)
    w, h = img.width(), img.height()
    ptr = img.constBits()
    ptr.setsize(h * img.bytesPerLine())
    arr = np.frombuffer(ptr, np.uint8).reshape(h, img.bytesPerLine())
    arr = arr[:, :w * 3].reshape(h, w, 3)
    # Tiles number their rows from the north down; the plot's y axis runs
    # the other way, so the image is flipped once here rather than by
    # inverting an axis and mirroring the track with it.
    #
    # The copy is the point of the last step and not a tidiness: everything
    # above is a VIEW into the QImage's own buffer, and that buffer is
    # released with the QImage when this function returns. What the plot
    # would keep is a pointer into freed memory, which paints as noise
    # until the process happens to fall over instead.
    return np.transpose(arr[::-1], (1, 0, 2)).copy()


# ===========================================================================
# The receiver
# ===========================================================================

# WGS84, for the one job the Mercator metres on the Track tab cannot do:
# a separation that has to be right to the centimetre rather than aligned
# with a map tile. Over the short baselines this is used on, the radii of
# curvature at the reference latitude ARE the answer, and they carry the
# ellipsoid's flattening that the sphere on the other tab leaves out.
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def ned_offset(lat0_deg, lon0_deg, lat_deg, lon_deg):
    """(east, north) metres of a position against a reference."""
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    r_meridian = WGS84_A * (1.0 - WGS84_E2) / (w * w * w)
    r_normal = WGS84_A / w
    north = math.radians(lat_deg - lat0_deg) * r_meridian
    east = math.radians(lon_deg - lon0_deg) * r_normal * math.cos(lat0)
    return east, north


FIX_TYPE = {0: "no fix", 1: "dead reckoning", 2: "2D", 3: "3D",
            4: "GNSS + DR", 5: "time only"}
CARR_SOLN = {0: "no carrier", 1: "RTK float", 2: "RTK fixed"}


class GnssTab(QtWidgets.QWidget):
    """The receiver, and what the filter makes of it.

    UBX-NAV-PVT comes past this window on its way through, so the
    receiver's own opinion is available next to the filter's without
    asking either of them for anything. Two things are worth doing with
    that. The first is watching the accuracy figures: they are the
    receiver saying how much it trusts itself, and a fix that degrades
    is visible there a long time before it is visible in a position.

    The second is the difference between the two positions, which is
    where this tab earns its place. That difference is not an error - the
    fix is an INPUT to the filter, and the filter is entitled to disagree
    with it while a lever arm, a delay or an aiding gate is doing its
    job. What it is, is the one quantity on the link that names a
    mounting mistake: an unmeasured antenna offset puts itself here, as a
    constant that turns with the platform.
    """

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        # hacc, vacc, sacc, satellites
        self.acc = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 4)
        # east, north and down of the filter's position against the fix
        self.sep = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 3)
        # receiver height, barometric channel and the datum origin, all
        # three on the WGS84 ellipsoid
        self.alt = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 3)
        self.fields = {}
        self.fix = None
        self.nav = None
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(300)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        self.state = QtWidgets.QLabel("waiting for a fix")
        self.state.setStyleSheet(MONO + " font-size: 15px;")
        v.addWidget(self.state)

        grid = QtWidgets.QGridLayout()
        for col, (label, key) in enumerate(
                (("satellites", "sv"), ("horizontal 1-sigma [m]", "hacc"),
                 ("vertical 1-sigma [m]", "vacc"),
                 ("speed 1-sigma [m/s]", "sacc"),
                 ("receiver speed [m/s]", "speed"))):
            cap = QtWidgets.QLabel(label)
            themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
            grid.addWidget(cap, 0, col)
            f = Field()
            self.fields[key] = f
            grid.addWidget(f, 1, col)
        v.addLayout(grid)

        self.inner = QtWidgets.QTabWidget()
        self.inner.addTab(self._page_accuracy(), "Reported accuracy")
        self.inner.addTab(self._page_heights(), "Heights")
        self.inner.addTab(self._page_separation(), "Against the filter")
        v.addWidget(self.inner, 1)

    def _page_accuracy(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)
        self.p_acc = make_plot("Reported 1-sigma position accuracy", "m", None,
                               legend=True)
        self.c_acc = themed_curve_keys(self.p_acc, ("accent", "warn"),
                                       ("horizontal", "vertical"))
        # Speed accuracy on an axis of its own: it is metres per second
        # against the position figures' metres, and the two share no
        # scale. It is also the figure that answers a different question
        # - a receiver can hold a good position while its velocity falls
        # apart, and the velocity is what the filter fuses hardest.
        self.p_sacc = make_plot("Reported 1-sigma speed accuracy", "m/s", None)
        self.c_sacc = themed_curve(self.p_sacc, "changed", width=2)
        self.p_sv = make_plot("Satellites used", "count",
                              "seconds before now",
                              axes={"left": CountAxis("left")})
        self.c_sv = themed_curve(self.p_sv, "ok", width=2)
        for p in (self.p_acc, self.p_sacc, self.p_sv):
            v.addWidget(p, 1)
        for p in (self.p_sacc, self.p_sv):
            p.setXLink(self.p_acc)

        note = QtWidgets.QLabel(
            "The receiver's own estimate of how well it is doing, which is "
            "not a measurement of anything: it is a model output, and it "
            "goes optimistic in exactly the places that hurt - under a "
            "canopy, next to a wall, on a roof with half the sky behind "
            "it. Read together with the satellite count, and treat a "
            "figure that improves while the count falls as the receiver "
            "guessing.\n"
            "The speed figure is the one to watch on a moving platform: "
            "it degrades on multipath before the position does, and it is "
            "what a velocity aiding gate rejects on.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    def _page_heights(self):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        v.setContentsMargins(0, 6, 0, 0)

        self.p_alt = make_plot("Height above the WGS84 ellipsoid", "m",
                               "seconds before now", legend=True)
        self.c_alt = themed_curve_keys(
            self.p_alt, ("accent", "ok", "idle"),
            ("receiver", "barometric channel", "local datum origin"))
        v.addWidget(self.p_alt, 1)

        note = QtWidgets.QLabel(
            "Three heights that are only comparable because they have "
            "been put on one datum first. The RECEIVER's is the ellipsoidal "
            "height out of UBX-NAV-PVT, not the mean-sea-level one next to "
            "it in the same message. The BAROMETRIC CHANNEL counts from the "
            "filter's local origin, so what is drawn is that height plus "
            "the ellipsoidal offset - and the offset is the third curve, "
            "the estimated ellipsoidal height of the origin itself.\n"
            "Which makes the gap between the first two the quantity worth "
            "reading, and the third curve the reason to distrust it: the "
            "offset is an estimated state that converges over tens of "
            "seconds after a fix and drifts away from one, so a barometric "
            "height that walks while its offset walks the same way has not "
            "moved at all. The barometer measures pressure, and weather "
            "moving over a platform that is not moving is metres.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)
        return page

    def _page_separation(self):
        page = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(page)
        h.setContentsMargins(0, 6, 0, 0)

        left = QtWidgets.QVBoxLayout()
        self.p_sep = make_plot("Filter minus receiver", "m",
                               "seconds before now", legend=True)
        self.c_sep = themed_curves(self.p_sep, ("north", "east", "down"))
        left.addWidget(self.p_sep, 1)
        note = QtWidgets.QLabel(
            "Metres of the filter's position against the receiver's, on "
            "the WGS84 ellipsoid at the current latitude. The two are not "
            "sampled at the same instant - the navigation message carries "
            "no timestamp - so at speed some of what is drawn here is the "
            "few tens of milliseconds between them, which is a metre at "
            "motorway speed and nothing at all standing still.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        left.addWidget(note)
        h.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        self.p_scatter = make_plot(None, "north [m]", "east [m]", legend=True)
        # Locked 1:1, or an ellipse drawn on it is not the ellipse the
        # numbers describe.
        self.p_scatter.setAspectLocked(True)
        self.c_pts = themed_curve(self.p_scatter, "accent", "separations",
                                  symbol="o", size=4)
        self.c_e1 = themed_curve(self.p_scatter, "ok", "1-sigma", width=2)
        self.c_e95 = themed_curve(self.p_scatter, "warn", "95 %", width=2)
        self.c_mean = themed_curve(self.p_scatter, "bad", "mean",
                                   symbol="+", size=16)
        right.addWidget(self.p_scatter, 1)

        self.ellipse_text = QtWidgets.QLabel("")
        self.ellipse_text.setStyleSheet(MONO)
        self.ellipse_text.setWordWrap(True)
        right.addWidget(self.ellipse_text)

        note = QtWidgets.QLabel(
            "The error ellipses come out of the SAMPLE covariance of the "
            "points on the plot, which makes them a statement about the "
            "last look-back window and not a prediction. Two of them, "
            "because a 1-sigma ellipse is the contour people misread most "
            "often: in two dimensions it holds about 39 % of the points "
            "and not 68 %, and the 95 % contour is 2.45 sigma rather than "
            "the 2 that the one-dimensional rule suggests.\n"
            "The CENTRE is the part that names a fault. Scatter about zero "
            "is the two solutions disagreeing at random, which is what "
            "they are supposed to do. A centre that sits somewhere else, "
            "and stays there, is a lever arm that has not been measured "
            "or has been measured with a sign the wrong way round.")
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        right.addWidget(note)
        h.addLayout(right, 4)
        return page

    # -- data --------------------------------------------------------------

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self.state.setText("not connected")

    def on_batch(self, batch):
        if batch.get("nav"):
            self.nav = batch["nav"]
        for fix in batch.get("pvt", ()):
            self.fix = fix
            if not (fix["fix_ok"] and fix["fix_type"] >= 2):
                # A receiver without a fix still reports a position and an
                # accuracy for it. Neither belongs on a plot: they are
                # what the last search left behind.
                continue
            self.acc.add(batch["t"], (fix["hacc_m"], fix["vacc_m"],
                                      fix["sacc_mps"], fix["num_sv"]))
            d = self.nav
            if d is not None:
                self._add_heights(batch["t"], fix, d)
            if d is None or d["lat_deg"] is None:
                continue
            east, north = ned_offset(fix["lat_deg"], fix["lon_deg"],
                                     d["lat_deg"], d["lon_deg"])
            down = np.nan
            if d["height_ell_m"] is not None and fix["fix_type"] >= 3:
                # Down positive, so that a filter sitting BELOW the fix
                # reads positive here the way it does in a NED velocity.
                down = fix["height_m"] - d["height_ell_m"]
            self.sep.add(batch["t"], (north, east, down))

    def _add_heights(self, t, fix, d):
        """The three heights of the Heights page, on one datum.

        Every one of them is blanked on its own rather than the sample
        being dropped: the receiver has no height without a 3D fix, and
        the other two are absent until the offset filter has something to
        say, and each of those is a gap in one curve and not in the rest.
        """
        ell_off = (d["height_ell_m"] - d["height_m"]
                   if d["height_ell_m"] is not None and d["height_m"] is not None
                   else np.nan)
        baro = (d["baro_alt_m"] + ell_off
                if d["baro_alt_m"] is not None and np.isfinite(ell_off)
                else np.nan)
        self.alt.add(t, (fix["height_m"] if fix["fix_type"] >= 3 else np.nan,
                         baro, ell_off))

    # -- display -----------------------------------------------------------

    def refresh(self):
        fix = self.fix
        if fix is None:
            self.state.setText("no UBX-NAV-PVT - is a receiver connected?")
        else:
            carrier = CARR_SOLN.get(fix["carr_soln"], "?")
            self.state.setText(
                "%s   %s   %s"
                % (FIX_TYPE.get(fix["fix_type"], "?").upper(),
                   "valid" if fix["fix_ok"] else "NOT VALID", carrier))
            good = fix["fix_ok"] and fix["fix_type"] >= 3
            self.state.setStyleSheet(
                MONO + " font-size: 15px; color: %s;"
                % T["ok" if good else "warn"])
            f = self.fields
            f["sv"].show_value("%d" % fix["num_sv"], fix["num_sv"] > 0)
            for key, value in (("hacc", fix["hacc_m"]), ("vacc", fix["vacc_m"]),
                               ("sacc", fix["sacc_mps"])):
                f[key].show_value("%.2f" % value, fix["fix_ok"])
            v = fix["vel_ned"]
            f["speed"].show_value("%.2f" % math.hypot(v[0], v[1]),
                                  fix["fix_ok"])

        t, vals = ring_window(self.acc, self.window_s)
        if t is not None:
            self.c_acc[0].setData(t, vals[:, 0])
            self.c_acc[1].setData(t, vals[:, 1])
            self.c_sacc.setData(t, vals[:, 2])
            self.c_sv.setData(t, vals[:, 3])
            # Held open to a few counts. A count that does not change sits
            # on a degenerate range, and an axis autoscaled onto it has
            # nowhere to put a whole-numbered tick.
            sv = vals[:, 3]
            lo, hi = float(sv.min()), float(sv.max())
            span = max(hi - lo + 2.0, 4.0)
            mid = (lo + hi) / 2.0
            self.p_sv.setYRange(mid - span / 2.0, mid + span / 2.0, padding=0)

        t, vals = ring_window(self.alt, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_alt):
                c.setData(t, vals[:, i], connect="finite")

        t, vals = ring_window(self.sep, self.window_s)
        if t is None:
            return
        for i, c in enumerate(self.c_sep):
            c.setData(t, vals[:, i], connect="finite")
        north, east = vals[:, 0], vals[:, 1]
        self.c_pts.setData(east, north)
        ex, ey, info = cov_ellipse(east, north, 1.0)
        if info is None:
            self.ellipse_text.setText(
                "%d separations - too few for an ellipse" % len(north))
            return
        self.c_e1.setData(ex, ey)
        ex, ey, _ = cov_ellipse(east, north, ELLIPSE_K95)
        self.c_e95.setData(ex, ey)
        self.c_mean.setData([info["cx"]], [info["cy"]])
        hacc = ""
        if self.fix is not None and self.fix["fix_ok"]:
            hacc = "   receiver 1-sigma %.2f m" % self.fix["hacc_m"]
        down = vals[np.isfinite(vals[:, 2]), 2]
        self.ellipse_text.setText(
            "%d points   mean %+.2f E %+.2f N%s m   axes %.2f / %.2f m at "
            "%.0f deg   DRMS %.2f m   95 %% %.2f m%s"
            % (info["n"], info["cx"], info["cy"],
               "" if down.size == 0 else " %+.2f D" % float(down.mean()),
               info["major"], info["minor"], info["angle_deg"],
               info["drms"], info["p95"], hacc))


# ===========================================================================
# Overview
# ===========================================================================

class OverviewTab(QtWidgets.QWidget):
    """One page that answers "what is this box doing" without a scroll.

    Nothing here is new: every number on it is on one of the other tabs,
    in more detail and with the plot that gives it context. What this
    page adds is the order they get read in. The question asked of a
    device on a bench is almost never "what is the yaw sigma" - it is
    "is this thing working, and if not, which part", and answering that
    out of six tabs means knowing in advance where to look.

    The warnings at the bottom are the same idea: the conditions that
    make a solution untrustworthy are spread across four messages, and
    every one of them is easy to miss while looking straight at it.
    """

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.nav = None
        self.fix = None
        self.imu_status = None
        # What the receiver claims, and how far the filter is from it.
        self.acc = Ring(HISTORY_MAX_S * RATE_PVT_HZ, 3)
        self.tiles = {}
        self.fields = {}
        self._rate_t0 = None
        self._rate_n = 0
        self._rate_bytes = 0
        self.imu_hz = None
        self.kbps = None
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(10)

        row = QtWidgets.QHBoxLayout()
        for key, caption, hint in (
                ("mode", "Filter mode", "What the filter is running as, "
                                        "from 0x40/0x0F."),
                ("att", "Attitude from", "Which of the three attitude "
                                         "solutions the answer came out of."),
                ("fix", "GNSS fix", "The receiver's own fix state, from "
                                    "UBX-NAV-PVT."),
                ("acc", "Position 1-sigma", "The receiver's horizontal "
                                            "accuracy figure."),
                ("cpu", "Processor load", "With the worst fusion epoch "
                                          "since boot beneath it."),
                ("link", "Link", "Frames arriving on this host, and what "
                                 "they cost in bytes.")):
            tile = StatTile(caption, hint)
            self.tiles[key] = tile
            row.addWidget(tile, 1)
        v.addLayout(row)

        middle = QtWidgets.QHBoxLayout()
        self.horizon = HorizonWidget()
        self.compass = CompassWidget()
        for inst in (self.horizon, self.compass):
            # Capped, or a maximised window spends half its height on two
            # dials that are no easier to read at 600 pixels than at 300.
            inst.setMaximumSize(340, 340)
            middle.addWidget(inst, 2)

        right = QtWidgets.QVBoxLayout()
        for title, rows in (
                ("Attitude", (("roll  pitch  yaw [deg]", "rpy"),
                              ("1-sigma [deg]", "sigma"))),
                ("Position", (("latitude / longitude", "latlon"),
                              ("ellipsoidal height / offset [m]", "height"),
                              ("filter minus receiver [m]", "sep"),
                              ("ground speed [m/s]", "speed"))),
                ("Sensors", (("die temperature [C]", "temp"),
                             ("calibration", "cal"),
                             ("measurement range", "sat")))):
            box = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout(box)
            grid.setColumnStretch(1, 1)
            for label, key in rows:
                r = grid.rowCount()
                cap = QtWidgets.QLabel(label)
                themed(lambda w=cap: w.setStyleSheet(dim()))
                grid.addWidget(cap, r, 0)
                f = Field()
                self.fields[key] = f
                grid.addWidget(f, r, 1)
            right.addWidget(box)

        lamps = QtWidgets.QHBoxLayout()
        self.lamps = {}
        for key, caption, hint in (
                ("ready", "READY", "The filter says its solution is usable."),
                ("zupt", "ZUPT", "Zero velocity update."),
                ("zaru", "ZARU", "Zero angular rate update."),
                ("vzupt", "VZUPT", "Vertical zero velocity update."),
                ("lever", "LEVER ARM", "An antenna offset is configured.")):
            lamp = Lamp(caption, hint)
            self.lamps[key] = lamp
            lamps.addWidget(lamp)
        lamps.addStretch(1)
        right.addLayout(lamps)
        right.addStretch(1)
        middle.addLayout(right, 3)
        v.addLayout(middle)

        # The one plot on this page, and the reason it is here rather than
        # on the three tabs its two halves come from: an accuracy the
        # receiver reports and a separation the filter is responsible for
        # only mean something against each other. A separation that grows
        # while the accuracy does not is the filter, and both growing
        # together is the sky.
        self.p_acc = make_plot("Accuracy and separation", "m",
                               "seconds before now", legend=True)
        self.c_acc = themed_curve_keys(
            self.p_acc, ("accent", "warn", "ok"),
            ("receiver horizontal 1-sigma", "receiver vertical 1-sigma",
             "filter minus receiver"))
        v.addWidget(self.p_acc, 1)

        self.warnings = QtWidgets.QLabel("")
        self.warnings.setWordWrap(True)
        self.warn_key = "dim"
        themed(lambda: self.warnings.setStyleSheet(
            MONO + " color: %s;" % T[self.warn_key]))
        v.addWidget(self.warnings)

    # -- data --------------------------------------------------------------

    def on_batch(self, batch):
        if batch.get("nav"):
            self.nav = batch["nav"]
        for fix in batch.get("pvt", ()):
            self.fix = fix
            if not (fix["fix_ok"] and fix["fix_type"] >= 2):
                continue
            sep = np.nan
            d = self.nav
            if d is not None and d["lat_deg"] is not None:
                sep = math.hypot(*ned_offset(fix["lat_deg"], fix["lon_deg"],
                                             d["lat_deg"], d["lon_deg"]))
            self.acc.add(batch["t"], (fix["hacc_m"], fix["vacc_m"], sep))
        if batch["imu"]:
            self.imu_status = batch["imu"][-1][3]
        if self._rate_t0 is None:
            self._rate_t0 = batch["t"]
        self._rate_n += len(batch["imu"])
        self._rate_bytes += batch.get("bytes", 0)
        span = batch["t"] - self._rate_t0
        if span >= 1.0:
            self.imu_hz = self._rate_n / span
            self.kbps = self._rate_bytes / span / 1000.0
            self._rate_t0, self._rate_n, self._rate_bytes = batch["t"], 0, 0

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self.nav = self.fix = None
            self.imu_hz = self.kbps = None
            self._rate_t0 = None
            for tile in self.tiles.values():
                tile.clear()
            self.horizon.set_attitude(None)
            self.compass.set_state(None)

    # -- display -----------------------------------------------------------

    def refresh(self):
        d, fix = self.nav, self.fix
        warn = []

        if d is None:
            self.tiles["mode"].show_value("--", "no 0x40/0x0F", "idle")
            self.tiles["att"].clear()
            warn.append("no navigation message: switch CFG-MSGOUT-NAV on, "
                        "or this page can only show the sensors.")
        else:
            self.tiles["mode"].show_value(
                d["mode"].upper(), "ready" if d["ready"] else "NOT READY",
                "ok" if d["mode"] == "full" and d["ready"] else "warn")
            self.tiles["att"].show_value(
                d["att_src"].upper(),
                "yaw 1-sigma %s deg" % _fmt(d["yaw_sigma_deg"], "%.2f"),
                "ok" if d["att_src"] == "ins" else "warn")

        if fix is None:
            self.tiles["fix"].show_value("--", "no receiver message", "idle")
            self.tiles["acc"].clear()
        else:
            good = fix["fix_ok"] and fix["fix_type"] >= 3
            self.tiles["fix"].show_value(
                FIX_TYPE.get(fix["fix_type"], "?").upper(),
                "%d satellites   %s" % (fix["num_sv"],
                                        CARR_SOLN.get(fix["carr_soln"], "")),
                "ok" if good else "warn")
            self.tiles["acc"].show_value(
                "%.2f m" % fix["hacc_m"] if fix["fix_ok"] else "--",
                "vertical %.2f m" % fix["vacc_m"] if fix["fix_ok"] else "",
                "ok" if fix["fix_ok"] else "idle")

        if d is not None:
            budget = 1e6 / d["imu_hz"] if d["imu_hz"] else None
            self.tiles["cpu"].show_value(
                "%d %%" % d["cpu_pct"],
                "worst epoch %d us%s" % (d["update_wcet_us"],
                                         " of %d" % budget if budget else ""),
                "bad" if d["cpu_pct"] >= 80 else "ok")
        if self.imu_hz is not None:
            self.tiles["link"].show_value(
                "%.0f Hz" % self.imu_hz, "%.1f kB/s IMU frames arriving"
                % (self.kbps or 0.0), "ok" if self.imu_hz > 1.0 else "warn")

        f = self.fields
        rpy = d["rpy_deg"] if d else None
        self.horizon.set_attitude(tuple(rpy) if rpy else None)
        vel = d["vel_ned"] if d else None
        track = None
        if vel is not None and math.hypot(vel[0], vel[1]) > 0.5:
            track = math.degrees(math.atan2(vel[1], vel[0]))
        self.compass.set_state(rpy[2] if rpy else None, track)

        f["rpy"].show_value(_fmt(rpy, "%8.2f"), rpy is not None)
        sig = d["rpy_sigma_deg"] if d else None
        f["sigma"].show_value(
            "%8.2f  %8.2f  %s" % (sig[0], sig[1],
                                  _fmt(d["yaw_sigma_deg"], "%.2f"))
            if sig else "--", sig is not None)
        latlon = (d["lat_deg"] is not None) if d else False
        f["latlon"].show_value(
            "%.7f  %.7f" % (d["lat_deg"], d["lon_deg"]) if latlon else "--",
            latlon)
        if d and d["height_ell_m"] is not None and d["height_m"] is not None:
            f["height"].show_value("%.2f   offset %.2f"
                                   % (d["height_ell_m"],
                                      d["height_ell_m"] - d["height_m"]))
        else:
            f["height"].show_value("--", False)
        if latlon and fix is not None and fix["fix_ok"] and fix["fix_type"] >= 2:
            east, north = ned_offset(fix["lat_deg"], fix["lon_deg"],
                                     d["lat_deg"], d["lon_deg"])
            sep = math.hypot(east, north)
            over = fix["hacc_m"] > 0.0 and sep > 3.0 * fix["hacc_m"]
            f["sep"].show_value("%+.2f N  %+.2f E   %.2f m"
                                % (north, east, sep), True,
                                "warn" if over else None)
            if over:
                warn.append("the filter and the receiver are %.1f m apart, "
                            "which is more than three times the accuracy the "
                            "receiver claims. Check the lever arm and the "
                            "housing rotation." % sep)
        else:
            f["sep"].show_value("--", False)
        if vel is not None:
            f["speed"].show_value("%.2f" % math.hypot(vel[0], vel[1]))
        else:
            f["speed"].show_value("--", False)

        if self.imu_status is not None:
            st = ux.decode_imu_status(self.imu_status)
            f["temp"].show_value("%.1f" % st["temp_c"])
            f["cal"].show_value("applied" if st["cal_applied"] else "RAW",
                                st["cal_applied"],
                                None if st["cal_applied"] else "warn")
            f["sat"].show_value(
                "SATURATED" if st["saturated"] else "inside range",
                not st["saturated"], "bad" if st["saturated"] else None)
            if st["saturated"]:
                warn.append("an axis is at its end stop right now. Those "
                            "samples are not measurements and the filter is "
                            "fusing them as if they were.")
        for key, on in (("ready", d and d["ready"]),
                        ("zupt", d and d["zupt"]), ("zaru", d and d["zaru"]),
                        ("vzupt", d and d["vzupt"]),
                        ("lever", d and d["leverarm_set"]
                         and not d["leverarm_bad"])):
            self.lamps[key].set_on(bool(on))

        if d is not None:
            if d["sat_axes"]:
                warn.append("axes that have clipped since boot: %s. See the "
                            "episode list on the Diagnostics tab."
                            % " ".join(d["sat_axes"]))
            if d["leverarm_bad"]:
                warn.append("a lever arm is stored and the board refused it. "
                            "The filter is running on zero.")
            elif not d["leverarm_set"] and fix is not None:
                warn.append("no lever arm is configured, so the antenna is "
                            "assumed to sit on the IMU. Under rotation that "
                            "offset does not average out - it couples "
                            "attitude into the fix and back again.")
            if d["cpu_pct"] >= 80:
                warn.append("processor load is at %d %%." % d["cpu_pct"])
        t, vals = ring_window(self.acc, self.window_s)
        if t is not None:
            for i, c in enumerate(self.c_acc):
                c.setData(t, vals[:, i], connect="finite")

        self.warn_key = "warn" if warn else "dim"
        self.warnings.setText(
            "\n".join("- " + w for w in warn) if warn
            else "nothing to report.")
        self.warnings.setStyleSheet(MONO + " color: %s;" % T[self.warn_key])


# ===========================================================================
# The board's clock against GPS
# ===========================================================================

# A time pulse a second, which is what the receiver is configured for and
# what the offline converter assumes as well. It is what makes the pulse
# COUNT a check on the pairing: GPS seconds and pulses advance together,
# so their difference is a constant, and a pair that slipped by a whole
# second is the one whose difference is not that constant.
PULSE_PERIOD_S = 1.0

# Pairs kept. At one a second this is two hours, which is longer than any
# rate estimate needs and short enough to fit in a fixed array.
CLOCK_MAX_PAIRS = 7200

# How long the rolling rate estimate looks back, in pairs. Short enough to
# follow a warm-up, long enough that the pairing jitter divided by the
# span is well under a ppm.
CLOCK_ROLL_PAIRS = 60


def clock_fit(t_s, gps_s):
    """Least squares gps = a * t + b over centred series.

    Returns (ppm, residual_us, rms_us) or None. CENTRED, and that is not a
    detail: the two series are around 1e5 and 1e9 seconds, a slope is
    wanted to a part in 1e12, and the normal equations formed on the raw
    numbers lose most of that to cancellation before the division.

    The sign is the one inslib_clock_error.py computes: a is GPS seconds
    per BOARD second, so a counter that runs fast makes a board second
    short, a smaller than one, and the ppm NEGATIVE.
    """
    if t_s.size < 3:
        return None
    x = t_s - t_s.mean()
    y = gps_s - gps_s.mean()
    sxx = float(np.dot(x, x))
    if sxx <= 0.0:
        return None
    a = float(np.dot(x, y)) / sxx
    res = (y - a * x) * 1e6
    return (a - 1.0) * 1e6, res, float(np.sqrt(np.mean(res * res)))


class ClockTab(QtWidgets.QWidget):
    """How fast the board's microsecond counter runs, in GPS's opinion.

    Every time pulse arrives as 0x40/0x05 with the MCU capture of the
    edge and the GPS instant the receiver announced for that same edge.
    A straight line through those pairs is the counter's rate error, and
    the scatter about the line is what the pairing itself is worth.

    Nothing here changes a solution. The rate scales every integrated
    quantity uniformly, and at a few tens of ppm that is nothing against
    a dead reckoning error. It matters where board time has to become GPS
    time - post processing raw observations, or stamping a GNSS epoch
    with its time of validity rather than its time of arrival - and it is
    the one number on the link that says whether that conversion is
    trustworthy over an hour.

    The message is not behind a CFG-MSGOUT switch: an edge is reported
    whether or not the receiver had an announcement for it, because a
    pulse that is alive with no time in it and no pulse at all are
    different faults.
    """

    def __init__(self, console):
        super().__init__()
        self.console = console
        self.window_s = HISTORY_DEFAULT_S
        self.fields = {}
        self._reset(0)
        self._build()
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.refresh)

    def _reset(self, segment):
        """Start a timeline. Everything before a counter restart belongs
        to a different time base and cannot share a fit with what comes
        after it."""
        self.segment = segment
        # Float64 throughout and deliberately not a Ring: those hold their
        # values as float32, and a GPS second in float32 is quantised to
        # about a hundred seconds.
        self.t = np.zeros(CLOCK_MAX_PAIRS)
        self.gps = np.zeros(CLOCK_MAX_PAIRS)
        self.key = np.zeros(CLOCK_MAX_PAIRS)
        self.n = 0
        self.last_t_us = None
        self.edges = 0
        self.unannounced = 0
        self.suspect = 0
        self.last_edge_host = None
        self.last_ts = None

    def _build(self):
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        self.state = QtWidgets.QLabel("waiting for a time pulse")
        self.state.setStyleSheet(MONO + " font-size: 15px;")
        v.addWidget(self.state)

        grid = QtWidgets.QGridLayout()
        for col, (label, key) in enumerate(
                (("rate over %d pulses [ppm]" % CLOCK_ROLL_PAIRS, "roll"),
                 ("rate over the segment [ppm]", "fit"),
                 ("drift [ms per hour]", "drift"),
                 ("fit residual [us rms]", "rms"),
                 ("pairs / span", "pairs"),
                 ("edges dropped", "dropped"))):
            cap = QtWidgets.QLabel(label)
            themed(lambda w=cap: w.setStyleSheet(dim() + " font-size: 11px;"))
            grid.addWidget(cap, 0, col)
            f = Field()
            self.fields[key] = f
            grid.addWidget(f, 1, col)
        v.addLayout(grid)

        self.p_ppm = make_plot("Counter rate error", "ppm", None, legend=True)
        self.c_int = themed_curve(self.p_ppm, "idle", "pulse to pulse",
                                  symbol="o", size=4)
        self.c_roll = themed_curve(self.p_ppm, "accent",
                                   "over %d pulses" % CLOCK_ROLL_PAIRS,
                                   width=2)
        self.l_fit = self.p_ppm.addLine(y=0.0)
        self.l_fit.setVisible(False)
        themed(lambda: self.l_fit.setPen(pg.mkPen(
            T["warn"], style=QtCore.Qt.PenStyle.DashLine)))

        self.p_res = make_plot("Residual against the segment fit", "us", None)
        self.c_res = themed_curve(self.p_res, "warn", symbol="o", size=3)

        self.p_off = make_plot("Board time minus GPS time", "ms",
                               "seconds before now")
        self.c_off = themed_curve(self.p_off, "ok", width=2)
        for p in (self.p_ppm, self.p_res, self.p_off):
            v.addWidget(p, 1)
        self.p_res.setXLink(self.p_ppm)
        self.p_off.setXLink(self.p_ppm)

        note = QtWidgets.QLabel(
            "One pair per time pulse: the MCU capture of the edge and the "
            "GPS instant the receiver announced for that same edge. "
            "POSITIVE ppm means the counter runs SLOW, a board "
            "microsecond being longer than a real one, which is the "
            "sign inslib_clock_error.py reports for the same pairs out "
            "of a hub session. The dashed line is the fit over this whole "
            "timeline and the trace is the same estimate over the last "
            "%d pulses, which follows a rate that moves.\n"
            "The residual says which of those two to believe: pairs "
            "carrying nothing but jitter scatter about the line at the "
            "size of that jitter, so a residual well above it is a rate "
            "that CHANGED during the segment - on a plain crystal, the "
            "warm-up, several ppm over the first minutes. Edges without "
            "an announcement, edges the firmware marked suspect, and "
            "pairs that slipped a whole second (caught by the pulse "
            "count, the way the offline converter does it) are counted "
            "and not fitted: one slipped pair in six hundred moves a "
            "least squares result by several ppm and leaves nothing "
            "that looks wrong." % CLOCK_ROLL_PAIRS)
        note.setWordWrap(True)
        themed(lambda: note.setStyleSheet(dim() + " font-size: 11px;"))
        v.addWidget(note)

    # -- data --------------------------------------------------------------

    def set_history(self, seconds):
        self.window_s = seconds

    def set_online(self, online):
        if online:
            self.timer.start()
        else:
            self.timer.stop()
            self.state.setText("not connected")

    def on_batch(self, batch):
        for ts in batch.get("timesync", ()):
            self.last_ts = ts
            self.edges += 1
            self.last_edge_host = batch["t"]
            t_us = ts["t_us"]
            if self.last_t_us is not None and t_us < self.last_t_us:
                # The counter went backwards, so the device restarted.
                # A cable that came loose does not do this; only a reset
                # does, and the pairs on either side of one are on
                # different time bases.
                self._reset(self.segment + 1)
            self.last_t_us = t_us
            if ts["suspect"]:
                self.suspect += 1
                continue
            if not ts["gps_valid"]:
                # The pulse is alive and carries no time. Worth counting,
                # since it is a different fault from a pulse that stopped.
                self.unannounced += 1
                continue
            i = self.n % CLOCK_MAX_PAIRS
            self.t[i] = t_us * 1e-6
            self.gps[i] = ts["gps_s"]
            self.key[i] = round(ts["gps_s"] / PULSE_PERIOD_S) - ts["count"]
            self.n += 1

    def _pairs(self):
        """The accepted pairs of this timeline, oldest first.

        The modal pulse-count key wins and the rest are slipped pairs, the
        same rule the offline converter applies. Recomputed rather than
        decided once: the first few pairs of a session are too few for a
        mode to mean anything."""
        if self.n == 0:
            return None, None, 0
        if self.n < CLOCK_MAX_PAIRS:
            t, gps, key = self.t[:self.n], self.gps[:self.n], self.key[:self.n]
        else:
            idx = np.r_[self.n % CLOCK_MAX_PAIRS:CLOCK_MAX_PAIRS,
                        0:self.n % CLOCK_MAX_PAIRS]
            t, gps, key = self.t[idx], self.gps[idx], self.key[idx]
        vals, counts = np.unique(key, return_counts=True)
        ok = key == vals[int(np.argmax(counts))]
        return t[ok], gps[ok], int(ok.size - ok.sum())

    # -- display -----------------------------------------------------------

    def refresh(self):
        t, gps, slipped = self._pairs()
        age = ("" if self.last_edge_host is None
               else "   last edge %.0f s ago"
               % (time.monotonic() - self.last_edge_host))
        if t is None or t.size < 3:
            self.state.setText(
                "%d time pulse(s), not enough for a rate%s"
                % (self.edges, age) if self.edges
                else "no time pulse yet - 0x40/0x05 arrives on the PPS edge, "
                     "so this needs a receiver with a fix")
            self.state.setStyleSheet(MONO + " font-size: 15px; color: %s;"
                                     % T["idle"])
            self.fields["dropped"].show_value(
                "%d unannounced  %d suspect" % (self.unannounced, self.suspect),
                bool(self.unannounced or self.suspect))
            return

        got = clock_fit(t, gps)
        if got is None:
            return
        ppm, res, rms = got
        span = float(t[-1] - t[0])
        self.state.setText(
            "%+.2f ppm over %.0f s from %d pulses   residual %.1f us rms%s"
            % (ppm, span, t.size, rms, age))
        self.state.setStyleSheet(MONO + " font-size: 15px; color: %s;"
                                 % T["ok"])

        f = self.fields
        f["fit"].show_value("%+.2f" % ppm)
        # What the rate costs in the only unit anybody acts on. The board
        # loses this much against GPS in an hour of its own running.
        f["drift"].show_value("%+.1f" % (ppm * 3.6))
        f["rms"].show_value("%.1f" % rms)
        f["pairs"].show_value("%d / %.0f s" % (t.size, span))
        f["dropped"].show_value(
            "%d unannounced  %d suspect  %d slipped"
            % (self.unannounced, self.suspect, slipped),
            bool(self.unannounced or self.suspect or slipped),
            "warn" if slipped else None)

        rel = t - t[-1]
        keep = rel >= -self.window_s
        # Pulse to pulse, which is two pairs differenced: it carries the
        # pairing jitter divided by one pulse interval, so it is a band a
        # few ppm wide around the answer rather than the answer.
        dt = np.diff(t)
        dg = np.diff(gps)
        with np.errstate(divide="ignore", invalid="ignore"):
            inter = np.where(dt > 0.0, (dg / dt - 1.0) * 1e6, np.nan)
        self.c_int.setData(rel[1:][keep[1:]], inter[keep[1:]])

        w = min(CLOCK_ROLL_PAIRS, t.size - 1)
        span_w = t[w:] - t[:-w]
        with np.errstate(divide="ignore", invalid="ignore"):
            roll = np.where(span_w > 0.0,
                            ((gps[w:] - gps[:-w]) / span_w - 1.0) * 1e6, np.nan)
        self.c_roll.setData(rel[w:][keep[w:]], roll[keep[w:]],
                            connect="finite")
        self.fields["roll"].show_value(
            "%+.2f" % roll[-1] if np.isfinite(roll[-1]) else "--",
            bool(np.isfinite(roll[-1])))
        self.l_fit.setValue(ppm)
        self.l_fit.setVisible(True)

        self.c_res.setData(rel[keep], res[keep])
        # Board minus GPS, against the oldest pair held: the accumulated
        # difference, which is what a timestamp converted with a nominal
        # clock would be out by.
        off = ((t - t[0]) - (gps - gps[0])) * 1e3
        self.c_off.setData(rel[keep], off[keep])
        self.p_ppm.setXRange(-self.window_s, 0.0, padding=0.02)

        if self.last_ts is not None and self.last_ts["gps_valid"]:
            # The receiver can only place an edge on its own clock grid and
            # reports the residual it knows it was off by. Sub nanosecond,
            # so it changes nothing here, but it belongs next to the pair
            # it qualifies rather than nowhere.
            self.state.setToolTip(
                "last pulse: quantisation error %d ps, %s time base"
                % (self.last_ts["q_err_ps"],
                   "UTC" if self.last_ts["utc_base"] else "GPS"))


# ===========================================================================
# Window
# ===========================================================================

class Console(QtWidgets.QMainWindow):

    def __init__(self, port, baud, settings=None):
        super().__init__()
        self.setWindowTitle("INSLIB console")
        icon = app_icon()
        if icon is not None:
            # On the window as well as on the application: a dialog
            # raised from here takes its icon from its parent window.
            self.setWindowIcon(icon)
        self.resize(1180, 760)
        self.worker = None
        self.baud = baud
        self.recording = False
        # The look-back and the theme are the two things a person sets
        # once and expects to find again, so they outlive the process.
        self.settings = settings or QtCore.QSettings("INSLIB", "inslib_gui")
        self.theme = "light" if T is LIGHT else "dark"
        self.history_s = HISTORY_DEFAULT_S
        try:
            stored = int(self.settings.value("history_s", HISTORY_DEFAULT_S))
        except (TypeError, ValueError):
            stored = HISTORY_DEFAULT_S
        if stored in HISTORY_CHOICES:
            self.history_s = stored

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central)
        v.setContentsMargins(8, 8, 8, 8)

        v.addLayout(self._port_bar(port))
        v.addLayout(self._record_bar())

        self.tabs = QtWidgets.QTabWidget()
        self.config = ConfigTab(self)
        self.overview = OverviewTab(self)
        self.solution = SolutionTab(self)
        self.sensors = SensorsTab(self)
        self.baro = BaroAltTab(self)
        self.calibration = CalibrationTab(self)
        self.diagnostics = DiagnosticsTab(self)
        self.track = TrackTab(self)
        self.gnss = GnssTab(self)
        self.clock = ClockTab(self)
        # Overview first, and the order after it is the order the
        # questions get asked in: what is it doing, where does it think it
        # is, what is the receiver saying, what are the sensors doing,
        # what is the vertical channel doing, what is stored, how is it
        # set up, and is it keeping up.
        self.tabs.addTab(self.overview, "Overview")
        self.tabs.addTab(self.solution, "Solution")
        self.tabs.addTab(self.track, "Track")
        self.tabs.addTab(self.gnss, "GNSS")
        self.tabs.addTab(self.sensors, "Sensors")
        self.tabs.addTab(self.baro, "Vertical")
        self.tabs.addTab(self.clock, "Clock")
        self.tabs.addTab(self.calibration, "Calibration")
        self.tabs.addTab(self.config, "Configuration")
        self.tabs.addTab(self.diagnostics, "Diagnostics")
        v.addWidget(self.tabs, 1)

        self._apply_history()

        self.status_label = QtWidgets.QLabel("")
        self.status_key = "idle"
        themed(lambda: self.status_label.setStyleSheet(
            "color: %s;" % T[self.status_key]))
        v.addWidget(self.status_label)

        if port:
            QtCore.QTimer.singleShot(0, self.connect)

    def _port_bar(self, port):
        h = QtWidgets.QHBoxLayout()
        h.addWidget(QtWidgets.QLabel("Port"))
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setEditable(True)
        self.port_box.setMinimumWidth(260)
        for name, desc in list_ports():
            self.port_box.addItem("%s  %s" % (name, desc), name)
        if port:
            self.port_box.setCurrentText(port)
        h.addWidget(self.port_box)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.setObjectName("primary")
        self.btn_connect.clicked.connect(self.toggle)
        h.addWidget(self.btn_connect)
        h.addStretch(1)

        self.link_label = QtWidgets.QLabel("offline")
        themed(lambda: self.link_label.setStyleSheet(
            "color: %s;" % T["ok" if self.worker else "idle"]))
        h.addWidget(self.link_label)
        return h

    def _record_bar(self):
        h = QtWidgets.QHBoxLayout()
        self.btn_record = QtWidgets.QPushButton("Record raw UBX")
        self.btn_record.setEnabled(False)
        self.btn_record.clicked.connect(self.toggle_record)
        h.addWidget(self.btn_record)
        self.record_label = QtWidgets.QLabel("not recording")
        themed(lambda: self.record_label.setStyleSheet(
            MONO + " color: %s;" % T["bad" if self.recording else "idle"]))
        h.addWidget(self.record_label, 1)

        h.addWidget(QtWidgets.QLabel("Look back"))
        self.hist_box = QtWidgets.QComboBox()
        self.hist_box.setToolTip(
            "How far back the live plots reach. The buffers behind them "
            "hold %d s at the nominal sample rates; a longer setting than "
            "the link has been up for shows what there is."
            % HISTORY_MAX_S)
        for sec in HISTORY_CHOICES:
            self.hist_box.addItem("%d s" % sec, sec)
        self.hist_box.setCurrentIndex(HISTORY_CHOICES.index(self.history_s))
        self.hist_box.currentIndexChanged.connect(self._history_changed)
        h.addWidget(self.hist_box)

        h.addWidget(QtWidgets.QLabel("Theme"))
        self.theme_box = QtWidgets.QComboBox()
        for name in ("dark", "light"):
            self.theme_box.addItem(name.capitalize(), name)
        self.theme_box.setCurrentIndex(1 if self.theme == "light" else 0)
        self.theme_box.currentIndexChanged.connect(self._theme_changed)
        h.addWidget(self.theme_box)
        return h

    def _apply_history(self):
        for tab in self._tabs():
            if hasattr(tab, "set_history"):
                tab.set_history(self.history_s)

    def _history_changed(self):
        self.history_s = self.hist_box.currentData()
        self.settings.setValue("history_s", self.history_s)
        self._apply_history()

    def _theme_changed(self):
        self.theme = self.theme_box.currentData()
        self.settings.setValue("theme", self.theme)
        set_theme(self.theme, QtWidgets.QApplication.instance())

    def toggle_record(self):
        if not self.worker:
            return
        if self.recording:
            self.worker.record_stop()
            return
        default = time.strftime("inslib_%Y%m%d_%H%M%S.ubx")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Record raw UBX to", default, "UBX capture (*.ubx)")
        if not path:
            return
        self.worker.record_start(path)

    def show_record(self, st):
        was, self.recording = self.recording, st["on"]
        if st["on"]:
            self.record_label.setText(
                "recording %s   %.0f s   %d frames   %.1f MB%s"
                % (os.path.basename(st["path"] or ""), st["seconds"],
                   st["frames"], st["bytes"] / 1e6,
                   "   %d bytes did not frame" % st["resync"]
                   if st["resync"] else ""))
            self.record_label.setStyleSheet(MONO + " color: %s;" % T["bad"])
            self.btn_record.setText("Stop recording")
        else:
            if was:
                self.record_label.setText(
                    "stopped: %d frames, %.1f MB in %s"
                    % (st["frames"], st["bytes"] / 1e6,
                       os.path.basename(st["path"] or "")))
            self.record_label.setStyleSheet(MONO + " color: %s;" % T["idle"])
            self.btn_record.setText("Record raw UBX")
        if was != self.recording:
            # A configuration exchange flushes the input buffer
            # (CfgLink.request), which would put a silent hole in the
            # recording. A log with holes nobody can see is worse than a
            # moment spent stopping it, so the board conversations are
            # locked out while one is running.
            for tab in (self.config, self.calibration):
                tab.set_online(not self.recording)
            self.status("recording: settings are locked out until it stops"
                        if self.recording else "recording stopped",
                        "warn" if self.recording else "ok")

    def current_port(self):
        data = self.port_box.currentData()
        text = self.port_box.currentText().strip()
        # The combo shows "COM5  ST-Link VCP"; typing a bare name has to
        # work too, so the description is cut off rather than assumed.
        return data if data and text.startswith(data) else text.split()[0] if text else ""

    def toggle(self):
        if self.worker:
            self.disconnect_board()
        else:
            self.connect()

    def connect(self):
        port = self.current_port()
        if not port:
            self.status("no port selected", "bad")
            return
        self.btn_connect.setEnabled(False)
        self.link_label.setText("connecting to %s" % port)
        self.worker = LinkWorker(port, self.baud)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.finished_job.connect(self.on_job)
        self.worker.failed_job.connect(self.on_job_failed)
        self.worker.samples.connect(self.on_samples)
        self.worker.start()

    def disconnect_board(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None

    def submit(self, tag, fn):
        if not self.worker:
            self.status("not connected", "bad")
            return
        self.status("%s ..." % tag, "idle")
        self.worker.submit(tag, fn)

    def status(self, text, colour=None):
        """`colour` is a PALETTE KEY, so the line survives a theme switch."""
        self.status_key = colour or "idle"
        self.status_label.setText(text)
        self.status_label.setStyleSheet("color: %s;" % T[self.status_key])

    # -- worker signals ----------------------------------------------------

    def on_connected(self, port):
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("Disconnect")
        self.link_label.setText("connected to %s" % port)
        self.link_label.setStyleSheet("color: %s;" % T["ok"])
        for tab in self._tabs():
            tab.set_online(True)
        self.btn_record.setEnabled(True)
        self.config.read_all()
        self.calibration.read()

    def on_disconnected(self, why):
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("Connect")
        self.link_label.setText("offline")
        self.link_label.setStyleSheet("color: %s;" % T["idle"])
        for tab in self._tabs():
            tab.set_online(False)
        self.btn_record.setEnabled(False)
        self.recording = False
        self.record_label.setText("not recording")
        self.btn_record.setText("Record raw UBX")
        self.worker = None
        if why:
            self.status(why, "bad")

    def on_job(self, tag, result):
        # PyQt6 aborts the PROCESS on an unhandled exception in a slot, so
        # a malformed frame or a bug in the display code below would not
        # produce a message -- it would make the window vanish, mid-task,
        # with whatever was on screen. Anything that goes wrong in here
        # becomes a status line instead.
        try:
            self._on_job(tag, result)
        except Exception as exc:                          # noqa: BLE001
            traceback.print_exc()
            self.status("%s: could not show the answer (%s)"
                        % (tag, exc.__class__.__name__), "bad")

    def _tabs(self):
        return (self.config, self.overview, self.solution, self.track,
                self.gnss, self.sensors, self.baro, self.clock,
                self.calibration, self.diagnostics)

    def on_samples(self, batch):
        # Same guard as _on_job: a decode surprise must not take the
        # window with it, and this one runs many times a second.
        try:
            for tab in (self.overview, self.sensors, self.diagnostics,
                        self.track, self.gnss, self.baro, self.clock):
                tab.on_batch(batch)
            if batch["nav"]:
                self.solution.on_nav(batch["nav"], batch["t"])
            if batch.get("pvt"):
                self.solution.on_pvt(batch["t"], batch["pvt"])
            if "record" in batch:
                self.show_record(batch["record"])
        except Exception:                                 # noqa: BLE001
            traceback.print_exc()

    def _on_job(self, tag, result):
        if tag == "read":
            self.config.on_read(result)
            self.status("read from the board", "ok")
        elif tag == "calibration":
            self.calibration.on_read(result)
            self.status("read the calibration", "ok")
        elif tag == "record":
            self.show_record(result)
        elif tag in ("calwrite", "calclear"):
            ack = result
            if ack and not ack["ok"]:
                where = ("" if not ack.get("key")
                         else " at %s" % ux.cfg_key_name(ack["key"]))
                self.status("calibration write refused%s: %s (%d applied "
                            "before it)" % (where, ack["text"], ack["detail"]),
                            "bad")
            else:
                self.status("calibration nodes written" if tag == "calwrite"
                            else "calibration cleared", "ok")
            # Read back rather than assume. A VALSET stops at the first
            # refusal, so a partial write leaves a table that is neither
            # the old one nor the new one, and the only way to know which
            # nodes are in there is to ask.
            self.calibration.read()
        elif tag in ("apply", "reset"):
            ack = result
            if ack and not ack["ok"]:
                where = ("" if not ack.get("key")
                         else " at %s" % ux.cfg_key_name(ack["key"]))
                self.status("refused%s: %s (%d applied before it)"
                            % (where, ack["text"], ack["detail"]), "bad")
            else:
                self.status("%s accepted" % tag, "ok")
            # Read back rather than assume: what the board holds after a
            # write is the only thing worth showing, and a partial apply
            # stops at the first refusal.
            self.config.read_all()

    def on_job_failed(self, tag, message):
        self.status("%s failed: %s" % (tag, message), "bad")

    def closeEvent(self, event):
        if self.recording and self.worker:
            self.worker.record_stop()
            self.worker.wait(500)
        self.track.shutdown()
        self.disconnect_board()
        super().closeEvent(event)


def list_ports():
    try:
        import serial.tools.list_ports as lp
        return [(p.device, p.description or "") for p in lp.comports()]
    except Exception:                                     # noqa: BLE001
        return []


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--theme", choices=sorted(THEMES),
                    help="colour scheme; remembered between runs")
    args = ap.parse_args(argv)

    taskbar_identity()
    app = QtWidgets.QApplication(sys.argv[:1])
    icon = app_icon()
    if icon is not None:
        app.setWindowIcon(icon)
    pg.setConfigOptions(antialias=False)
    settings = QtCore.QSettings("INSLIB", "inslib_gui")
    theme = args.theme or settings.value("theme", "dark")
    if theme not in THEMES:
        theme = "dark"
    if args.theme:
        settings.setValue("theme", theme)
    set_theme(theme, app)
    win = Console(args.port, args.baud, settings)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
