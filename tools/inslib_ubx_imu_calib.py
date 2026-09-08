#!/usr/bin/env python3
"""IMU calibration for the STM32 sensor board, without a fixture.

Records one continuous session in which the unit is put down in a number
of arbitrary static poses with rotations in between, then solves for the
accelerometer and gyroscope calibration and writes the REQ-NAV-037 keys
into a config.yaml that tools/insrcv.c --config and python/replay.py
consume directly:

    imu:
      acc_misalignment: [...]   # col-major 3x3, corrected = M*(raw-bias)
      gyr_misalignment: [...]   # col-major 3x3, same model
      acc_fixed_bias: [...]     # [m/s^2]
      gyr_fixed_bias: [...]     # [rad/s]

When the board also streams a magnetometer (0x40/0x06) the same recording
calibrates that too, into the REQ-NAV-039 keys:

    mag:
      misalignment: [...]       # col-major 3x3, soft iron + alignment
      fixed_bias: [...]         # hard iron [uT]

The method is imu_tk's (Tedaldi/Pretto/Menegatti, ICRA 2014); the maths
lives in inslib_imu_tk.py, which also explains why it needs no known
orientations: the accelerometer cost is |g| - ||M*(a-b)||, a magnitude,
so where a pose points never enters it. Put the unit down anywhere, leave
it for a few seconds, pick it up, turn it, put it down again. The static
periods are detected from a rolling variance and the one threshold
involved is swept and resolved by residual, so there is nothing to tune.

This replaces the 6-position tumble test that used to live here. That
method estimates the bias as (up+down)/2 per axis pair, which is exact
only when the two halves of a pair are exactly opposite one another: an
independent 1 deg placement error on each position puts ~0.11 m/s^2 into
the accelerometer bias, more than the bias of a decent MEMS part. It also
could not calibrate the gyroscope beyond its bias, for want of a rate
table. This one gets gyro scale and full misalignment from the same
recording, because between two static poses the accelerometer knows both
gravity directions and the integrated gyro has to carry one onto the
other.

What is measured:

  * Accelerometer misalignment, scale and bias: 9 parameters, fitted over
    every static pose in the session.
  * Gyroscope misalignment and scale: 9 parameters, fitted over the
    rotations between consecutive poses. Its bias comes from the initial
    rest period.
  * IMU noise (gyr_psd/acc_psd): from the initial rest period, as
    sigma^2 * dt (the per-sample convention replay.py's noise check
    uses). Only written when the config has no value yet.
  * Magnetometer hard iron, soft iron and the mounting rotation onto the
    IMU triad, when the board streams one. The maths and the reasoning
    are in inslib_mag_calib.py and inslib_frame_align.py: the turns
    between the poses give the field cloud its coverage, and the static
    poses give the alignment, so the session needs nothing added to it.

The housing alignment (where the IMU sits inside the BOX, as opposed to
where the sensor sits on the board) needs the unit placed on a level
surface face by face, which is a guided procedure rather than a recording:
tools/inslib_calib_gui.py has it.

The IMU temperature over the whole session is recorded and written as a
comment together with the date: MEMS biases drift with temperature, so
the calibration is only valid near that range.

If the output config.yaml already exists it is merged: the imu
calibration keys are replaced, everything else (and an existing tuned
noise model) is preserved. Merging needs PyYAML; hand-written comments
in the old file are regenerated, not kept.

Usage:
    python3 tools/inslib_ubx_imu_calib.py --port COM4
    python3 tools/inslib_ubx_imu_calib.py --udp 29801        # hub fan-out
    python3 tools/inslib_ubx_imu_calib.py --port COM4 --duration 240

--udp reads the stream from tools/inslib_hub.py's fan-out instead of
opening the serial port, so a running session keeps recording and feeding
insrcv while the unit is being calibrated (see UdpSource).

tools/inslib_calib_gui.py is a graphical front end for the same
measurement and solve code in this file.

Requires pyserial and numpy. PyYAML only when merging into an existing
config. Framing is UbxFramer from inslib_ubx.py, like every other tool
here: pyubx2's stream reader must not be pointed at this stream (see that
module's docstring, and the note on ImuStream below).

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import datetime
import math
import os
import socket
import struct
import sys
import time
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "datasets"))
import replay_format   # noqa: E402  (datasets/, shared config.yaml writer)
import inslib_imu_tk as imu_tk   # noqa: E402  (the calibration maths)
import inslib_frame_align as fa   # noqa: E402  (mounting rotations)
import inslib_mag_calib as mag_calib   # noqa: E402  (hard/soft iron)
from inslib_ubx import CLS_INSLIB as CLASS_CUSTOM   # noqa: E402
from inslib_ubx import (CLASS_NAV, ID_IMU, ID_MAG, ID_NAV_PVT,   # noqa: E402
                        IMU_FMT, MAG_FMT, UbxFramer, decode_imu_status)
import inslib_ubx as ubx   # noqa: E402

try:
    from pyubx2 import UBXReader
except ImportError:
    UBXReader = None   # GNSS auto-fill degrades to "not available"

# The board's answers to configuration requests, which ImuStream parks
# for whoever asked rather than letting them fall through the sample
# filter (inslib_protocol.md, "Configuration interface").
CFG_REPLY_IDS = (ubx.ID_CFGINFO, ubx.ID_VALGET_R, ubx.ID_CFGACK)

US_PER_SEC = 1e6
G_MPS2 = 9.80665            # g -> m/s^2
DEG2RAD = math.pi / 180.0   # deg/s -> rad/s


def local_gravity(lat_deg, height_m=0.0):
    """Normal gravity at a position [m/s^2], or None without the library.

    WGS84 normal gravity (Somigliana) plus the free-air correction, taken
    from INSLIB's own ins_gravity_ned() rather than from a formula
    repeated here. Same reason the field strength comes from INSLIB's own
    WMM (inslib_mag_calib.wmm_reference): |g| is the reference magnitude
    the accelerometer fit is scaled to, so it has to be the magnitude the
    filter will later assume, not a second opinion about it.

    G_MPS2 above is the standard value. It is right at about 45.5 degrees
    latitude at sea level and nowhere else: at 48.9 degrees and 160 m it
    is 2.6e-4 of its own size too small, and because the cost function is
    |g| - ||M (a - b)||, that error lands on the accelerometer scale
    factors one for one.

    height_m is ELLIPSOIDAL height, which is what the free-air term is
    defined against and what NAV-PVT's height field carries (hMSL is the
    other one and differs by the geoid undulation, tens of metres)."""
    try:
        import ctypes
        sys.path.insert(0, os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "..", "python"))
        from INSLIB._core import _lib as lib   # noqa: F401
        lib.ins_gravity_ned.argtypes = [ctypes.c_float, ctypes.c_float,
                                        ctypes.c_float * 3]
        lib.ins_gravity_ned.restype = None
        out = (ctypes.c_float * 3)()
        lib.ins_gravity_ned(float(lat_deg) * DEG2RAD, float(height_m), out)
    except Exception:
        return None
    # NED, so gravity is the down component. The bracket is a sanity
    # check on the whole path, not on the model: normal gravity runs from
    # 9.78 at the equator to 9.83 at the poles.
    g = float(out[2])
    return g if 9.7 < g < 9.9 else None

# Defaults of the recording session. The initial rest period sizes the
# static-detection threshold and seeds the gyro bias, so it has to be a
# genuinely undisturbed stretch, and longer than any pose that follows.
DEFAULT_INIT_SEC = 20.0
DEFAULT_DURATION_SEC = 200.0
DEFAULT_POSE_SEC = 4.0
# Below this the fit is not worth writing out (imu_tk's own floor).
MIN_POSITIONS = imu_tk.DEFAULT_MIN_INTERVALS


# --- small helpers ---------------------------------------------------------

class AxisStats:
    """Streaming per-axis mean/variance (Welford), for 3-vectors.

    Used for the live stillness read-out; the calibration itself works on
    the recorded arrays."""

    def __init__(self):
        self.n = 0
        self._mean = [0.0, 0.0, 0.0]
        self._m2 = [0.0, 0.0, 0.0]

    def add(self, v):
        self.n += 1
        for i in range(3):
            d = v[i] - self._mean[i]
            self._mean[i] += d / self.n
            self._m2[i] += d * (v[i] - self._mean[i])

    def mean(self):
        return tuple(self._mean)

    def var(self):
        if self.n < 2:
            return (0.0, 0.0, 0.0)
        return tuple(m2 / (self.n - 1) for m2 in self._m2)

    def std(self):
        return tuple(math.sqrt(v) for v in self.var())


def mat3_to_colmajor(m):
    """Row-major nested 3x3 -> flat column-major 9-list (config layout)."""
    return [float(m[i][j]) for j in range(3) for i in range(3)]


def psd_from_still(acc, gyr, rate_hz):
    """(gyr_psd, acc_psd) from a still window, averaged over the 3 axes.

    Per-sample variance times the sample interval, the same
    expect = sqrt(psd/dt) convention replay.py's noise check uses. A still
    window is the cleanest estimate this sensor will ever give."""
    dt = 1.0 / rate_hz
    return (float(np.var(gyr, axis=0, ddof=1).mean()) * dt,
            float(np.var(acc, axis=0, ddof=1).mean()) * dt)


def stillness_complaints(gyr_std_dps, acc_std, max_gyr_std_dps=0.5,
                         max_acc_std_mps2=0.3):
    """Human-readable reasons why a window was NOT still (empty = ok)."""
    out = []
    if gyr_std_dps > max_gyr_std_dps:
        out.append("gyro noise %.2f deg/s (limit %.2f) -- unit moved or "
                   "vibrating surface" % (gyr_std_dps, max_gyr_std_dps))
    if acc_std > max_acc_std_mps2:
        out.append("accel noise %.2f m/s^2 (limit %.2f) -- unit moved or "
                   "vibrating surface" % (acc_std, max_acc_std_mps2))
    return out


# --- UBX class-0x40 IMU decode ---------------------------------------------
# UbxFramer (inslib_ubx.py) does the framing/resync/checksum; the payload
# is this board's own format (tools/inslib_protocol.md), decoded here.

_IMU_STRUCT = struct.Struct(IMU_FMT)   # t_us, acc[3] g, gyr[3] dps, status, seq
_MAG_STRUCT = struct.Struct(MAG_FMT)   # t_us, mag[3] uT, temp


@dataclass
class ImuSample:
    t_us: int                       # u64 microseconds since power-up
    acc_mps2: tuple                 # (x, y, z) body frame [m/s^2]
    gyr_rps: tuple                  # (x, y, z) body frame [rad/s]
    temp_c: float                   # 0.1 degC resolution on the wire
    seq: int                        # monotonic 16-bit sample counter
    # At least one axis sat at the sensor end stop in this sample. The
    # values are then not a measurement: they are the end of the range,
    # and they look entirely plausible. Which axis is in 0x40/0x0E.
    saturated: bool = False
    # The stored calibration reached this sample. Not derivable from the
    # values, which is why it travels with them.
    cal_applied: bool = False


@dataclass
class MagSample:
    t_us: int                       # same MCU timer as ImuSample
    mag_ut: tuple                   # (x, y, z) body frame [uT]
    temp_c: float


def decode_imu(payload):
    """0x40/0x01 payload -> ImuSample, or None on a length mismatch.
    Firmware units are g / dps / degC; converted to SI here."""
    if len(payload) != _IMU_STRUCT.size:
        return None
    t_us, ax, ay, az, gx, gy, gz, status, seq = _IMU_STRUCT.unpack(payload)
    st = decode_imu_status(status)
    return ImuSample(
        t_us=t_us,
        acc_mps2=(ax * G_MPS2, ay * G_MPS2, az * G_MPS2),
        gyr_rps=(gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD),
        temp_c=st["temp_c"],
        seq=seq,
        saturated=st["saturated"],
        cal_applied=st["cal_applied"],
    )


def decode_mag(payload):
    """0x40/0x06 payload -> MagSample, or None on a length mismatch.

    Already in uT and already rotated into the same body frame as the IMU
    message by the firmware's mounting remap, so what is left for the host
    to find is hard iron, soft iron and the residual mounting rotation
    between the two parts (inslib_mag_calib.py)."""
    if len(payload) != _MAG_STRUCT.size:
        return None
    t_us, mx, my, mz, temp = _MAG_STRUCT.unpack(payload)
    return MagSample(t_us=t_us, mag_ut=(mx, my, mz), temp_c=temp)


# --- standard u-blox NAV-PVT decode (pyubx2) --------------------------------
# The receiver's own fix, arriving on the same link whenever the board
# passes it through (tools/inslib_hub.py). Only decoded so the GUI can
# offer "fill the position in from the receiver" instead of it having to
# be typed in by hand. Position feeds two lookups: the WMM field strength
# for the magnetometer and normal gravity for the accelerometer.

@dataclass
class GnssFix:
    lat_deg: float
    lon_deg: float
    fix_type: int    # u-blox fixType: 0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time
    fix_ok: bool      # NAV-PVT flags bit 0 (gnssFixOk)
    num_sv: int
    # Ellipsoidal, because that is the height normal gravity is defined
    # against. Defaulted, so a fix built without one still constructs.
    height_m: float = 0.0


def decode_nav_pvt(frame):
    """One complete, checksum-verified NAV-PVT frame -> GnssFix, or None.

    Decoded with pyubx2 rather than by hand (inslib_ubx.py's module
    docstring): it is the standard u-blox message the receiver sends, not
    something this project defines. None both when pyubx2 is not
    installed and on a decode fault -- a GNSS position is a convenience
    here, not something the calibration depends on."""
    if UBXReader is None:
        return None
    try:
        msg = UBXReader.parse(frame)
        return GnssFix(
            lat_deg=float(msg.lat), lon_deg=float(msg.lon),
            fix_type=int(getattr(msg, "fixType", 0)),
            fix_ok=bool(getattr(msg, "gnssFixOk", 0)),
            num_sv=int(getattr(msg, "numSV", 0)),
            # NAV-PVT height is millimetres above the ellipsoid, and
            # pyubx2 passes it through unscaled.
            height_m=float(getattr(msg, "height", 0)) / 1000.0)
    except Exception:
        return None


# --- input sources ---------------------------------------------------------

class UdpSource:
    """The hub's UDP fan-out instead of the serial port.

    tools/inslib_hub.py owns the device (one owner per port), so a
    calibration that opens the port itself cannot run while a session is
    being recorded or fed to insrcv. Reading the fan-out means the board
    keeps streaming to everything else while it is calibrated, which is
    the same reason insrcv does not open a serial port either
    (tools/README.md).

    Give the hub a second destination for it:

        python3 tools/inslib_hub.py COM4 \\
            --fanout 127.0.0.1:29800,127.0.0.1:29801

    A separate port, not a second listener on insrcv's: two UDP sockets
    bound to one port do not both get the traffic on Windows."""

    datagram = True     # a datagram carries whole frames, never a part of one

    def __init__(self, bind_addr="0.0.0.0", port=29801, timeout=0.05):
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_addr, port))
        self.sock.settimeout(timeout)

    def read_datagrams(self, max_n=256):
        """Wait up to the timeout for one datagram, then take whatever
        else is already queued without waiting again."""
        out = []
        try:
            out.append(self.sock.recv(65536))
        except OSError:
            return out
        self.sock.setblocking(False)
        try:
            while len(out) < max_n:
                try:
                    out.append(self.sock.recv(65536))
                except OSError:
                    break
        finally:
            self.sock.settimeout(self.timeout)
        return out

    def reset_input_buffer(self):
        self.sock.setblocking(False)
        try:
            while True:
                try:
                    self.sock.recv(65536)
                except OSError:
                    break
        finally:
            self.sock.settimeout(self.timeout)

    def close(self):
        self.sock.close()


def open_source(port=None, baud=921600, udp=None):
    """A serial port or a UDP fan-out listener, from the CLI's two flags.

    `udp` is "PORT" or "HOST:PORT"."""
    if udp:
        host, _, p = udp.rpartition(":")
        return UdpSource(host or "0.0.0.0", int(p))
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is required: pip install pyserial")
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"[calib] cannot open {port}: {e}\n"
                 f"        (tools/inslib_hub.py owns the port when it is "
                 f"running -- use --udp instead)")


class ImuStream:
    """Reads IMU samples off the live stream on demand, from a serial port
    or from a UdpSource.

    Framing is UbxFramer, NOT pyubx2's stream reader. Opening the port and
    every flush() below leave the reader in the middle of a frame, so it
    has to hunt for the next sync pair through raw float bytes. Two of
    those bytes can read as one of pyubx2's 21 two-byte NMEA headers
    ("$G", "$P", ...), and it then decodes the "sentence" as UTF-8 before
    the protocol filter ever runs. A float byte that is not valid UTF-8
    raises UnicodeDecodeError straight out of read(), which quitonerror
    does not cover because it is not one of pyubx2's own exception types.
    See inslib_ubx.py's module docstring: every other tool here already
    frames this way."""

    def __init__(self, source):
        self.source = source
        self.datagram = getattr(source, "datagram", False)
        self.framer = UbxFramer()
        self.temp_min = float("inf")    # whole-session range (config comment)
        self.temp_max = float("-inf")
        # Magnetometer frames arrive interleaved with the IMU ones at a
        # tenth of the rate. They are decoded in the same pass and parked
        # here rather than returned, so read_samples() keeps meaning "IMU
        # samples" for the callers that only want those.
        self._mag = []
        # The board's answers to configuration requests come back on the
        # same link. They are parked undecoded, because this class has no
        # business knowing what a caller asked for - only that these
        # frames belong to it and not in the sample stream.
        self._cfg = []
        # A GNSS receiver passed through the same link (tools/inslib_hub.py)
        # sends its own NAV-PVT alongside the IMU frames, parked here like
        # the magnetometer.
        self._gnss = []

    def flush(self):
        self.source.reset_input_buffer()
        # The partial frame held over from before the flush is not a
        # continuation of anything that follows it: splicing the two would
        # manufacture a frame that was never sent.
        self.framer.reset()

    def _chunks(self):
        """Whatever has arrived, blocking at most for the source timeout.

        Serial: read(1) waits for the first byte, in_waiting takes the
        rest that is already buffered."""
        if self.datagram:
            return self.source.read_datagrams()
        data = self.source.read(max(self.source.in_waiting, 1))
        return [data] if data else []

    def read_samples(self):
        """Every IMU sample that has arrived since the last call."""
        out = []
        for chunk in self._chunks():
            if self.datagram:
                # Same rule the other UDP consumers follow: a datagram
                # holds whole frames, so a leftover from the previous one
                # is the tail of a lost or reordered datagram rather than
                # a continuation of this one (inslib_protocol.md).
                self.framer.reset()
            for cls, mid, payload, frame in self.framer.feed(chunk):
                if cls == CLASS_NAV and mid == ID_NAV_PVT:
                    fix = decode_nav_pvt(frame)
                    if fix is not None:
                        self._gnss.append(fix)
                    continue
                if cls != CLASS_CUSTOM:
                    continue
                if mid == ID_MAG:
                    m = decode_mag(payload)
                    if m is not None:
                        self._mag.append(m)
                    continue
                if mid in CFG_REPLY_IDS:
                    self._cfg.append((mid, payload))
                    continue
                if mid != ID_IMU:
                    continue
                s = decode_imu(payload)
                if s is None:
                    continue
                self.temp_min = min(self.temp_min, s.temp_c)
                self.temp_max = max(self.temp_max, s.temp_c)
                out.append(s)
        return out

    def read_mag(self):
        """Every magnetometer sample decoded since the last call.

        Call it after read_samples(), which is what actually reads the
        port. A board without a magnetometer simply never returns any
        (inslib_protocol.md: the message is absent, not zero-filled)."""
        out, self._mag = self._mag, []
        return out

    def read_cfg(self):
        """Every configuration reply seen since the last call, as
        (msg_id, payload). Call it after read_samples(), which is what
        actually reads the port."""
        out, self._cfg = self._cfg, []
        return out

    def read_gnss(self):
        """Every NAV-PVT fix decoded since the last call.

        Call it after read_samples(). A board with no receiver attached,
        or one that has not passed pyubx2 a frame it can decode, simply
        never returns any."""
        out, self._gnss = self._gnss, []
        return out


class Recording:
    """One calibration session, accumulated sample by sample.

    The whole session is kept: the calibration is a global fit over every
    static pose in it, not a sequence of independent measurements, so
    there is nothing to discard as it goes. At 100 Hz an hour is 1.1 M
    samples, so a python list of tuples is not the problem it looks
    like."""

    def __init__(self):
        self.t_us = []
        self.acc = []
        self.gyr = []
        self.mag_t_us = []
        self.mag = []
        # The magnetometer's own die temperature, kept because the
        # firmware looks its calibration up by exactly that and not by
        # the IMU's (cal.c). A node written against the wrong thermometer
        # is a node at the wrong temperature.
        self.mag_temp_c = []
        self.temp_min = float("inf")
        self.temp_max = float("-inf")
        # Samples the board had already corrected when they arrived. A
        # calibration solved over those is the residual of the one the
        # board holds, which is not what the caller believes it has, and
        # nothing in the numbers says so: only this bit does.
        self.n_cal_applied = 0

    def add(self, s):
        self.t_us.append(s.t_us)
        self.acc.append(s.acc_mps2)
        self.gyr.append(s.gyr_rps)
        if s.cal_applied:
            self.n_cal_applied += 1
        self.temp_min = min(self.temp_min, s.temp_c)
        self.temp_max = max(self.temp_max, s.temp_c)

    def add_mag(self, s):
        self.mag_t_us.append(s.t_us)
        self.mag.append(s.mag_ut)
        self.mag_temp_c.append(s.temp_c)

    def has_mag(self):
        return len(self.mag) > 0

    def __len__(self):
        return len(self.t_us)

    def arrays(self):
        """(t_sec, acc, gyr) as numpy arrays, time relative to the start."""
        t = np.asarray(self.t_us, dtype=np.float64)
        t = (t - t[0]) / US_PER_SEC
        return t, np.asarray(self.acc), np.asarray(self.gyr)

    def mag_arrays(self):
        """(t_sec, mag) as numpy arrays, on the SAME clock arrays() uses.

        Both messages carry the one MCU timer (inslib_protocol.md), so the
        two streams can be lined up by time even though they run at
        different rates and are never sampled together."""
        if not self.mag or not self.t_us:
            return np.zeros(0), np.zeros((0, 3))
        t = (np.asarray(self.mag_t_us, dtype=np.float64)
             - float(self.t_us[0])) / US_PER_SEC
        return t, np.asarray(self.mag, dtype=float)

    def rate_hz(self):
        if len(self) < 2:
            return 0.0
        span = (self.t_us[-1] - self.t_us[0]) / US_PER_SEC
        return (len(self) - 1) / span if span > 0 else 0.0

    def pose_count(self, init_sec, pose_sec):
        """How many static poses the detector currently finds.

        The live counter during recording. Uses the middle threshold of
        the sweep the solve will run, so it is indicative rather than
        final -- the solve may find one or two more or fewer."""
        if len(self) < 500:
            return 0
        t, acc, _ = self.arrays()
        rate = self.rate_hz()
        if rate <= 0:
            return 0
        end = int(min(np.searchsorted(t, init_sec), len(t) - 1))
        if end < 100:
            return 0
        var = acc[:end + 1].var(axis=0, ddof=1)
        norm_th = float(np.sqrt((var * var).sum()))
        if not (norm_th > 0):
            return 0
        n_samp = max(int(pose_sec * rate * 0.5), 20)
        ivals = imu_tk.static_intervals(acc, 6.0 * norm_th)
        return sum(1 for (s, e) in ivals if e - s + 1 >= n_samp)


# --- config output ---------------------------------------------------------

def load_existing(path):
    """Existing config.yaml as a dict (to merge into), or None. Needs
    PyYAML only in the merge case -- fail before measuring, not after."""
    if not os.path.exists(path):
        return None
    try:
        import yaml
    except ImportError:
        sys.exit(f"{path} exists; merging the calibration into it needs "
                 f"PyYAML (pip install pyyaml). Or write to a fresh file "
                 f"with -o <path>.")
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        sys.exit(f"{path}: not a mapping at the top level, refusing to merge")
    return cfg


def build_sections(existing, updates):
    """Section list for replay_format.write_config: a fresh starter config
    around the calibration, or the existing config with the given sections
    updated (order and unrelated keys preserved).

    `updates` is {section name: {key: value}}, so one call covers the imu
    section, the mag section, or both."""
    if existing is None:
        base = [
            ("name", "imu_calib"),
            ("aiding", ("gnss", "F9P NAV-PVT fixes (for replay use)")),
            ("init", "auto"),
            ("baro", {"enable": 1, "stddev_m": 2.0}),
            ("imu", {}),
            ("gnss", {"leverarm_frd": [0.0, 0.0, 0.0],
                      "vel_stddev_fallback_mps": 0.2}),
            ("score", {"warmup_sec": 30.0}),
        ]
        existing_keys = [k for k, _v in base]
        sections = [(k, dict(updates[k]) if k in updates else v)
                    for k, v in base]
        sections = [(k, v) for k, v in sections if v != {}]
    else:
        existing_keys = list(existing.keys())
        sections = []
        for key, val in existing.items():
            if key in updates:
                sec = dict(val) if isinstance(val, dict) else {}
                sec.update(updates[key])
                sections.append((key, sec))
            else:
                sections.append((key, val))
    for key, val in updates.items():
        if key not in existing_keys:
            sections.append((key, dict(val)))
    return sections


def _aslist(a):
    return [] if a is None else np.asarray(a).tolist()


def _colmajor_to_mat3(flat):
    """Config layout (flat column-major 9) -> row-major nested 3x3.

    All-zero means identity, the same reading insrcv.c and replay.py
    apply to these keys (REQ-NAV-037)."""
    v = [float(x) for x in flat]
    if len(v) != 9 or not any(v):
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return [[v[j * 3 + i] for j in range(3)] for i in range(3)]


def calib_from_config(cfg):
    """The calibration already written in a config.yaml, as
    (acc_M, acc_bias, gyr_M, gyr_bias), or None if it carries none.

    Lets a live view apply an EXISTING calibration, so one can check what
    a previous session produced without recording a new one."""
    if not isinstance(cfg, dict):
        return None
    imu = cfg.get("imu")
    if not isinstance(imu, dict):
        return None
    keys = ("acc_misalignment", "acc_fixed_bias",
            "gyr_misalignment", "gyr_fixed_bias")
    if not any(k in imu for k in keys):
        return None
    zero = [0.0, 0.0, 0.0]
    return (_colmajor_to_mat3(imu.get("acc_misalignment", [])),
            [float(x) for x in imu.get("acc_fixed_bias", zero)],
            _colmajor_to_mat3(imu.get("gyr_misalignment", [])),
            [float(x) for x in imu.get("gyr_fixed_bias", zero)])


@dataclass
class Calibration:
    """Everything one session measured, already in config units.

    The single hand-off between measuring and writing, so the console
    tool and tools/inslib_calib_gui.py produce byte-identical files from
    the same numbers."""
    acc_matrix: list                # row-major nested 3x3, M = T*K
    acc_bias: list                  # [m/s^2]
    gyr_matrix: list                # row-major nested 3x3
    gyr_bias: list                  # [rad/s]
    gyr_psd: float                  # (rad/s)^2/Hz
    acc_psd: float                  # (m/s^2)^2/Hz
    rate_hz: float
    temp_lo: float
    temp_hi: float
    gravity: float
    n_positions: int = 0
    residual_rms: float = float("nan")      # |g| error after calib [m/s^2]
    gyr_residual_rms: float = float("nan")  # gravity-direction mismatch
    duration_sec: float = 0.0
    misalignment_estimated: bool = True
    acc_stats: dict = field(default_factory=dict)
    gyr_stats: dict = field(default_factory=dict)
    # Per static pose, for the before/after plot (see CalibResult).
    pose_acc_raw: list = field(default_factory=list)
    pose_acc_cal: list = field(default_factory=list)
    pose_gyr_raw: list = field(default_factory=list)
    pose_gyr_cal: list = field(default_factory=list)
    # Sample index ranges of those poses, so a second sensor recorded
    # alongside can be averaged over the same windows.
    intervals: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @classmethod
    def from_result(cls, res, gyr_psd, acc_psd, rate_hz, temp_lo, temp_hi,
                    gravity, duration_sec):
        """Wrap an inslib_imu_tk.CalibResult."""
        return cls(
            acc_matrix=[list(map(float, row)) for row in res.acc_matrix],
            acc_bias=[float(v) for v in res.acc_bias],
            gyr_matrix=[list(map(float, row)) for row in res.gyr_matrix],
            gyr_bias=[float(v) for v in res.gyr_bias],
            gyr_psd=gyr_psd, acc_psd=acc_psd, rate_hz=rate_hz,
            temp_lo=temp_lo, temp_hi=temp_hi, gravity=gravity,
            n_positions=res.n_positions, residual_rms=res.residual_rms,
            gyr_residual_rms=res.gyr_residual_rms,
            duration_sec=duration_sec,
            misalignment_estimated=res.misalignment_estimated,
            acc_stats=dict(res.acc_stats), gyr_stats=dict(res.gyr_stats),
            pose_acc_raw=_aslist(res.pose_acc_raw),
            pose_acc_cal=_aslist(res.pose_acc_cal),
            pose_gyr_raw=_aslist(res.pose_gyr_raw),
            pose_gyr_cal=_aslist(res.pose_gyr_cal),
            intervals=[(int(s), int(e)) for (s, e) in res.intervals],
            warnings=quality_warnings(res))

    def rotated(self, r):
        """The same calibration, expressed in a rotated body frame.

        How a housing alignment is folded in: corrected = M*(raw - bias)
        becomes R*M*(raw - bias), which is the same measurement written
        along the housing axes instead of the sensor's own. The bias is
        untouched, it sits on the raw side of the matrix."""
        if r is None:
            return self
        r = np.asarray(r, dtype=float)
        out = Calibration(**{k: getattr(self, k)
                             for k in self.__dataclass_fields__})
        out.acc_matrix = [[float(x) for x in row]
                          for row in r @ np.asarray(self.acc_matrix)]
        out.gyr_matrix = [[float(x) for x in row]
                          for row in r @ np.asarray(self.gyr_matrix)]
        return out

    def summary(self):
        """The human-readable result block, shared by both front ends."""
        acc_scale = [(math.sqrt(sum(self.acc_matrix[i][j] ** 2
                                    for i in range(3))) - 1.0) * 100.0
                     for j in range(3)]
        gyr_scale = [(math.sqrt(sum(self.gyr_matrix[i][j] ** 2
                                    for i in range(3))) - 1.0) * 100.0
                     for j in range(3)]
        lines = [
            "%d static poses over %.0f s at %.0f Hz"
            % (self.n_positions, self.duration_sec, self.rate_hz),
            "accel bias  [%s] m/s^2"
            % ", ".join("%+.4f" % b for b in self.acc_bias),
            "accel scale [%s] %%"
            % ", ".join("%+.2f" % s for s in acc_scale),
            "gyro  bias  [%s] deg/s"
            % ", ".join("%+.3f" % (b / DEG2RAD) for b in self.gyr_bias),
            "gyro  scale [%s] %%"
            % ", ".join("%+.2f" % s for s in gyr_scale),
            "IMU temperature %.1f .. %.1f degC" % (self.temp_lo, self.temp_hi),
        ]
        if not self.misalignment_estimated:
            lines.append("misalignment NOT estimated (diagonal model)")
        return lines + self.comparison() + ["! " + w for w in self.warnings]

    def comparison(self):
        """What the calibration bought, as a before/after table.

        Rows are cumulative: `raw` is the sensor as it comes, `+bias` adds
        only the zero offset, `+scale/misalign` is the full model. Read
        down the column to see which part of the calibration did the work
        -- if the last row barely improves on the middle one, the scale
        and misalignment terms were not observable in this session and a
        diagonal fit would have been the more honest answer."""
        if not self.acc_stats:
            return []
        out = ["",
               "what the calibration bought (over the fitted samples):",
               "                    |a| - |g| [m/s^2]                  "
               "gyro dir",
               "                      rms        mean         max"
               "          rms"]
        rows = [("raw", "raw"), ("+bias", "bias")]
        rows.append(("+scale/misalign" if self.misalignment_estimated
                     else "+scale", "full"))
        for label, key in rows:
            a = self.acc_stats.get(key)
            if a is None:
                continue
            g = self.gyr_stats.get(key, {}).get("rms", float("nan"))
            gtxt = ("%8.2f deg" % math.degrees(g)) if g == g else "        --"
            out.append("  %-16s %8.4f   %+8.4f   %9.4f   %s"
                       % (label, a["rms"], a["mean"], a["max"], gtxt))
        return out


def quality_warnings(res):
    """Results that are physically implausible for a working MEMS part."""
    out = []
    if res.init_static_frac < 0.9:
        # The gyro bias and the noise model both come from this period.
        # Only the clean part of it is used, but if most of it was not
        # clean there may not be enough left to average over.
        out.append("only %.0f %% of the initial rest period was actually "
                   "still -- the gyro bias comes from that part alone; "
                   "redo it with the unit untouched if it looks off"
                   % (100.0 * res.init_static_frac))
    scale = res.acc_scale_percent()
    if any(abs(s) > 10.0 for s in scale):
        out.append("an accel scale factor is off by >10 %, unusual -- "
                   "check that |g| is right for your location")
    if any(abs(b) > 1.0 for b in res.acc_bias):
        out.append("an accel bias exceeds 1 m/s^2, unusual for a modern part")
    if res.residual_rms > 0.05:
        out.append("|g| still %.3f m/s^2 rms off after the fit: the poses "
                   "were probably not all really static" % res.residual_rms)
    if res.n_positions < 20:
        out.append("only %d poses -- more (and more varied) positions make "
                   "the misalignment terms better conditioned"
                   % res.n_positions)
    if res.gyr_solved and res.gyr_residual_rms > 0.05:
        out.append("gyro direction mismatch %.2f deg: rotate more slowly, "
                   "or the gyro is saturating"
                   % math.degrees(res.gyr_residual_rms))
    return out


def has_tuned_noise(existing):
    """True if the config being merged into already carries a noise model.
    A measured still window is a decent estimate, but a value somebody
    tuned against a real dataset beats it and must not be overwritten."""
    return bool(existing is not None
                and isinstance(existing.get("imu"), dict)
                and existing["imu"].get("gyr_psd")
                and existing["imu"].get("acc_psd"))


def build_imu_section(cal, existing):
    """The imu: section for write_config, as (imu_new, kept_psd)."""
    imu_new = {
        "acc_misalignment": (mat3_to_colmajor(cal.acc_matrix),
                             "col-major 3x3, corrected = M*(raw-bias)"),
        "gyr_misalignment": (mat3_to_colmajor(cal.gyr_matrix),
                             "col-major 3x3, same model"),
        "acc_fixed_bias": ([round(b, 6) for b in cal.acc_bias],
                           "m/s^2, multi-position calibration"),
        "gyr_fixed_bias": ([round(b, 8) for b in cal.gyr_bias],
                           "rad/s, static avg (incl. earth rate)"),
    }
    kept_psd = has_tuned_noise(existing)
    if not kept_psd:
        imu_new = dict(
            [("gyr_psd", (cal.gyr_psd, "(rad/s)^2/Hz, measured still @ %.0f Hz"
                          % cal.rate_hz)),
             ("acc_psd", (cal.acc_psd, "(m/s^2)^2/Hz, measured still")),
             ("gyr_bias_rw", 2e-6), ("acc_bias_rw", 3e-4)],
            **imu_new)
    return imu_new, kept_psd


def build_mag_section(magcal):
    """The mag: section for write_config (REQ-NAV-039 keys).

    `enable` is deliberately not written: whether the filter should fuse
    the magnetometer is a decision about the platform, not something a
    calibration gets to make."""
    return {
        "misalignment": (mat3_to_colmajor(magcal.matrix),
                         "col-major 3x3, corrected = M*(raw-bias), soft "
                         "iron%s" % (" + alignment to the IMU"
                                     if magcal.align is not None else "")),
        "fixed_bias": ([round(b, 4) for b in magcal.bias],
                       "uT, hard iron"),
    }


def rotate_config_imu(existing, r):
    """The imu keys of an existing config with the 3x3s rotated into a new
    body frame, for a housing alignment captured on its own.

    A config that carries no calibration yet is treated as identity, so a
    housing alignment can be the first thing ever written to it."""
    cal = calib_from_config(existing) or (np.eye(3).tolist(), [0.0] * 3,
                                          np.eye(3).tolist(), [0.0] * 3)
    acc_m, acc_b, gyr_m, gyr_b = cal
    r = np.asarray(r, dtype=float)
    return {
        "acc_misalignment": (mat3_to_colmajor(r @ np.asarray(acc_m)),
                             "col-major 3x3, corrected = M*(raw-bias)"),
        "gyr_misalignment": (mat3_to_colmajor(r @ np.asarray(gyr_m)),
                             "col-major 3x3, same model"),
        "acc_fixed_bias": ([round(float(b), 6) for b in acc_b],
                           "m/s^2, unchanged"),
        "gyr_fixed_bias": ([round(float(b), 8) for b in gyr_b],
                           "rad/s, unchanged"),
    }


def rotate_config_mag(existing, r):
    """The mag keys of an existing config rotated into a new body frame.

    Counterpart to rotate_config_imu, and needed for the same reason. A
    housing alignment captured on its own moves the accelerometer and the
    gyroscope into the housing frame; a magnetometer calibration already
    in the file has to go with them. Left behind it stays in the sensor
    frame, and the heading it feeds the filter is then referenced to axes
    nothing else uses -- silently, because both files are well formed and
    every residual still looks right.

    Returns {} when the file carries no magnetometer calibration, which
    is the ordinary case and nothing to correct. The hard iron is a
    vector in the sensor frame the bias is subtracted in, so it does not
    rotate; only the 3x3 does, exactly as in MagCalibration.rotated."""
    mag = existing.get("mag") if isinstance(existing, dict) else None
    if not isinstance(mag, dict) or "misalignment" not in mag:
        return {}
    m = np.asarray(r, dtype=float) @ np.asarray(
        _colmajor_to_mat3(mag.get("misalignment", [])))
    return {
        "misalignment": (mat3_to_colmajor(m.tolist()),
                         "col-major 3x3, rotated into the frame measured "
                         "now"),
    }


def build_header(cal, merged, magcal=None, housing=None):
    """The comment block at the top of the generated config.yaml."""
    date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    if cal is not None:
        header = (
            "IMU calibration by tools/inslib_ubx_imu_calib.py\n"
            "method: multi-position, no fixture (imu_tk, Tedaldi et al. "
            "ICRA 2014)\n"
            "date: %s\n"
            "%d static poses over %.0f s, |g| residual %.4f m/s^2 rms\n"
            "IMU temperature during calibration: %.1f .. %.1f degC\n"
            "  (biases drift with temperature; recalibrate outside this "
            "range)\n"
            "gravity assumed: %g m/s^2\n"
            "%s"
            "model: corrected = M * (raw - fixed_bias)   (REQ-NAV-037)"
            % (date, cal.n_positions, cal.duration_sec, cal.residual_rms,
               cal.temp_lo, cal.temp_hi, cal.gravity,
               "" if cal.misalignment_estimated
               else "misalignment NOT estimated: scale and bias only\n"))
    else:
        header = ("calibration update by tools/inslib_calib_gui.py\n"
                  "date: %s\n"
                  "%s"
                  "model: corrected = M * (raw - fixed_bias)   (REQ-NAV-037)"
                  % (date,
                     "the IMU calibration below is the one that was already "
                     "in this file,\n  rotated into the frame measured now\n"
                     if housing is not None else
                     "the IMU calibration below is unchanged, only the "
                     "section(s) named\n  in this header were measured\n"))
    if housing is not None:
        header += (
            "\n\nhousing alignment: the 3x3 matrices above are premultiplied "
            "by the\n"
            "  rotation from the IMU axes onto the housing axes, measured "
            "from\n"
            "  %s\n"
            "  IMU in the housing: roll %+.2f, pitch %+.2f, yaw %+.2f deg%s"
            % (", ".join(fa.PLACEMENT_LABEL.get(p, p)
                         for p in housing.placements),
               housing.rpy_deg[0], housing.rpy_deg[1], housing.rpy_deg[2],
               "\n  (roll and pitch only, nothing measured the yaw)"
               if housing.tilt_only else ""))
    if magcal is not None:
        header += (
            "\n\nmagnetometer: hard and soft iron over %d samples, scaled to "
            "%.1f uT (%s)\n"
            "  |m| scatter %.3f -> %.3f uT%s"
            % (magcal.n_samples, magcal.field_ut, magcal.field_source,
               magcal.raw_rms_ut, magcal.residual_rms_ut,
               ("\n  aligned to the IMU over %d poses: roll %+.2f, pitch "
                "%+.2f, yaw %+.2f deg"
                % (magcal.align.n_poses, magcal.align.rpy_deg[0],
                   magcal.align.rpy_deg[1], magcal.align.rpy_deg[2]))
               if magcal.align is not None else
               "\n  NOT aligned to the IMU (soft and hard iron only)"))
    if merged:
        header += ("\n\nmerged into the existing config: other sections kept, "
                   "their comments regenerated")
    return header


def write_calibration(path, cal, existing, magcal=None, housing=None):
    """Write (or merge) one session's result. Returns kept_psd, so the
    caller can say whether the measured noise model was used.

    `cal` may be None when there is nothing new to say about the IMU
    itself, which is how a housing alignment or a magnetometer
    calibration is written on its own against an existing config.

    `housing` is an inslib_frame_align.HousingAlignment. Its rotation is
    folded into every 3x3 that leaves the sensor frame (accelerometer,
    gyroscope and magnetometer alike): they have to end up in ONE body
    frame, and correcting only some of them would silently rotate the
    magnetometer away from the attitude the filter is tracking. That
    holds for a magnetometer calibration this session did not measure
    too, which is why a housing-only pass rotates the one already in the
    file rather than leaving it behind (rotate_config_mag)."""
    r = None if housing is None else housing.as_matrix()
    if cal is not None:
        imu_new, kept_psd = build_imu_section(cal.rotated(r), existing)
    elif r is not None:
        imu_new, kept_psd = rotate_config_imu(existing, r), True
    else:
        imu_new, kept_psd = {}, True
    updates = {}
    if imu_new:
        updates["imu"] = imu_new
    if magcal is not None:
        updates["mag"] = build_mag_section(magcal.rotated(r))
    elif r is not None:
        # No new magnetometer solve, but the body frame moved. Whatever
        # magnetometer calibration the file already holds has to move
        # with it, or only half of the sensors end up in the new frame.
        mag_new = rotate_config_mag(existing, r)
        if mag_new:
            updates["mag"] = mag_new
    if not updates:
        raise ValueError("nothing to write")
    replay_format.write_config(
        path, build_sections(existing, updates),
        header=build_header(cal, existing is not None, magcal, housing))
    return kept_psd


# --- solving ---------------------------------------------------------------

def solve(rec, gravity=G_MPS2, init_sec=DEFAULT_INIT_SEC,
          estimate_misalignment=True, log=None):
    """Recording -> Calibration. Raises ValueError if the session is not
    usable (too few poses, no rest period, ...)."""
    if len(rec) < 1000:
        raise ValueError("only %d samples recorded, that is not a session"
                         % len(rec))
    t, acc, gyr = rec.arrays()
    rate = rec.rate_hz()
    res = imu_tk.calibrate(acc, gyr, t, g_mag=gravity,
                           init_static_sec=init_sec,
                           estimate_misalignment=estimate_misalignment,
                           log=log)
    end = int(min(np.searchsorted(t, init_sec), len(t) - 1))
    # The noise model comes from the same genuinely-static samples the
    # gyro bias did, not from the whole declared rest period: a second of
    # handling in there inflates the measured PSD by orders of magnitude
    # and that value would go straight into the filter tuning.
    mask = res.init_static_mask
    if mask is not None and mask.any():
        a_still, g_still = acc[:end + 1][mask], gyr[:end + 1][mask]
    else:
        a_still, g_still = acc[:end + 1], gyr[:end + 1]
    gyr_psd, acc_psd = psd_from_still(a_still, g_still, rate)
    return Calibration.from_result(res, gyr_psd, acc_psd, rate,
                                   rec.temp_min, rec.temp_max, gravity,
                                   float(t[-1]))


def mag_pose_pairs(rec, cal, min_samples=3):
    """One (calibrated accel, raw mag) mean pair per static pose.

    The alignment of the magnetometer to the IMU needs both sensors read
    while the unit was standing still, and the two streams run at
    different rates, so the magnetometer samples are selected by the TIME
    span of each pose rather than by index. The accelerometer side has the
    IMU calibration applied, because a leftover accelerometer bias tilts
    the gravity direction the alignment is measured against."""
    if not rec.has_mag() or not cal.intervals:
        return []
    t, acc, _gyr = rec.arrays()
    tm, mag = rec.mag_arrays()
    acc_cal = imu_tk.apply_calib(acc, np.asarray(cal.acc_matrix),
                                 np.asarray(cal.acc_bias))
    pairs = []
    for (s, e) in cal.intervals:
        sel = (tm >= t[s]) & (tm <= t[e])
        if int(sel.sum()) < min_samples:
            continue
        pairs.append((acc_cal[s:e + 1].mean(axis=0), mag[sel].mean(axis=0)))
    return pairs


def solve_mag(rec, cal, field_ut=0.0, field_source="", log=None):
    """Recording (+ its IMU calibration) -> MagCalibration, or None when
    the session carries no magnetometer data. Raises ValueError when there
    is some but it cannot support a fit."""
    if not rec.has_mag():
        return None
    _tm, mag = rec.mag_arrays()
    return mag_calib.solve(mag, mag_pose_pairs(rec, cal), field_ut=field_ut,
                           field_source=field_source, log=log)


# --- main ------------------------------------------------------------------

def record_session(stream, duration, init_sec, pose_sec, show=True):
    """Stream for `duration` seconds, printing what is going on."""
    rec = Recording()
    stream.flush()
    t0 = time.monotonic()
    next_line = t0
    live = AxisStats()
    live_t0 = t0
    poses = 0
    while True:
        now = time.monotonic()
        if now - t0 >= duration:
            break
        for s in stream.read_samples():
            rec.add(s)
            live.add(s.gyr_rps)
        for m in stream.read_mag():
            rec.add_mag(m)
        if now - live_t0 > 0.5:
            live_t0, live_std = now, max(live.std()) / DEG2RAD if live.n > 5 else 0.0
            still = live_std < 0.5
            live = AxisStats()
        if show and now >= next_line:
            next_line = now + 1.0
            if int(now - t0) % 5 == 0:
                poses = rec.pose_count(init_sec, pose_sec)
            elapsed = now - t0
            phase = ("keep it STILL (initial rest period)"
                     if elapsed < init_sec else
                     "move / place / hold, repeat")
            sys.stderr.write("\r  %5.0f/%.0f s  %6d samples  %2d poses  %-38s"
                             % (elapsed, duration, len(rec), poses, phase))
            sys.stderr.flush()
    if show:
        sys.stderr.write("\n")
    if show and rec.n_cal_applied:
        # Not derivable from the numbers, so it has to be said: a raw
        # stream and a corrected one look exactly alike here.
        sys.stderr.write(
            "  WARNING: %.0f%% of the recording arrived already "
            "corrected by the board.\n"
            "  What this solves is the RESIDUAL of the stored "
            "calibration. Clear the switch\n"
            "  (inslib_cfg.py set CFG-IMU-APPLY_CAL=0) and record "
            "again.\n"
            % (100.0 * rec.n_cal_applied / max(len(rec), 1)))
    return rec


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial device (e.g. COM4, /dev/ttyACM0)")
    src.add_argument("--udp", metavar="[HOST:]PORT",
                     help="listen on the hub's UDP fan-out instead of "
                          "opening the port, so a running inslib_hub.py "
                          "session keeps recording and feeding insrcv "
                          "(hub: --fanout 127.0.0.1:29800,127.0.0.1:29801)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate (irrelevant for USB-CDC; default 921600)")
    ap.add_argument("-o", "--out", default="config.yaml",
                    help="output config.yaml (default ./config.yaml; an "
                         "existing file is merged, not overwritten)")
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION_SEC,
                    help="length of the recording [s] (default %(default)s)")
    ap.add_argument("--init-sec", type=float, default=DEFAULT_INIT_SEC,
                    help="initial undisturbed rest period [s]; sizes the "
                         "static threshold and gives the gyro bias "
                         "(default %(default)s)")
    ap.add_argument("--pose-sec", type=float, default=DEFAULT_POSE_SEC,
                    help="how long to hold each pose [s], for the live "
                         "counter and the advice (default %(default)s)")
    ap.add_argument("--no-misalignment", action="store_true",
                    help="fit scale and bias only, leave the axis "
                         "misalignment at identity. The off-diagonal terms "
                         "are the weakly observable ones: with few or poorly "
                         "spread poses they absorb scale and bias error "
                         "instead of measuring anything")
    ap.add_argument("--gravity", type=float, default=G_MPS2,
                    help="local gravity [m/s^2] (default standard %(default)s; "
                         "differs up to ~0.3%% with latitude/altitude and "
                         "maps 1:1 into the accel scale factors)")
    ap.add_argument("--no-mag", action="store_true",
                    help="ignore the magnetometer even if the board streams "
                         "one (0x40/0x06). By default its hard iron, soft "
                         "iron and mounting rotation relative to the IMU are "
                         "solved from the same session and written as the "
                         "mag: keys")
    ap.add_argument("--mag-field-ut", type=float, default=0.0,
                    help="local field strength [uT] to scale the "
                         "magnetometer calibration to (0: keep what the "
                         "sensor measures). The heading is the same either "
                         "way, but the filter gates the magnetometer on the "
                         "field strength matching its own WMM lookup")
    args = ap.parse_args()

    existing = load_existing(args.out)  # fail early if merge is impossible
    source = open_source(args.port, args.baud, args.udp)
    stream = ImuStream(source)
    where = f"udp:{args.udp}" if args.udp else args.port

    n_poses = int((args.duration - args.init_sec) // (args.pose_sec + 2.0))
    print("Multi-position IMU calibration (no fixture needed)")
    print("  1. Put the unit down and leave it completely alone for the "
          "first %.0f s." % args.init_sec)
    print("  2. Then, until the %.0f s are up: pick it up, turn it to ANY "
          "new\n     orientation, put it down, hold ~%.0f s. Repeat."
          % (args.duration, args.pose_sec))
    print("     Roughly %d poses fit in the time; the more varied the "
          "angles, the\n     better conditioned the misalignment terms. "
          "They do NOT have to be\n     level or axis-aligned." % n_poses)
    print()

    probe = None
    t_probe = time.monotonic()
    while time.monotonic() - t_probe < 2.0:
        got = stream.read_samples()
        if got:
            probe = got[-1]
    if probe is None:
        sys.exit(f"[calib] no IMU frames on {where} -- wrong port, or the "
                 f"board is not streaming, or (with --udp) the hub has no "
                 f"fan-out to this port (see tools/inslib_protocol.md)")
    print("[calib] stream is alive, temperature %.1f degC" % probe.temp_c)

    input("Press Enter to start the recording ... ")
    rec = record_session(stream, args.duration, args.init_sec, args.pose_sec)
    print("[calib] recorded %d samples at %.0f Hz, temperature %.1f .. %.1f degC"
          % (len(rec), rec.rate_hz(), rec.temp_min, rec.temp_max))

    try:
        cal = solve(rec, args.gravity, args.init_sec,
                    estimate_misalignment=not args.no_misalignment,
                    log=lambda m: print("   " + m))
    except ValueError as e:
        sys.exit("[calib] %s" % e)

    magcal = None
    if rec.has_mag() and not args.no_mag:
        try:
            magcal = solve_mag(rec, cal, field_ut=args.mag_field_ut,
                               log=lambda m: print("   " + m))
        except ValueError as e:
            print("[calib] magnetometer not calibrated: %s" % e)
    elif not rec.has_mag():
        print("[calib] no magnetometer frames in the session (0x40/0x06), "
              "IMU only")

    print()
    print("Result:")
    for line in cal.summary():
        print("  " + line)
    if magcal is not None:
        for line in magcal.summary():
            print("  " + line)
    print()

    if cal.n_positions < MIN_POSITIONS:
        sys.exit("[calib] only %d poses, need at least %d -- nothing written"
                 % (cal.n_positions, MIN_POSITIONS))
    ans = input(f"Write calibration to {args.out}? [Y/n] ").strip().lower()
    if ans and ans not in ("y", "yes", "j", "ja"):
        sys.exit("[calib] aborted, nothing written")

    if write_calibration(args.out, cal, existing, magcal=magcal):
        print("[calib] keeping the existing noise model (measured here: "
              "gyr_psd %.3e, acc_psd %.3e)" % (cal.gyr_psd, cal.acc_psd))
    print(f"[calib] wrote {args.out}")
    print("Use it with:")
    print(f"  insrcv --udp-port 29800 --config {args.out}   (live receiver)")
    print(f"  python3 python/replay.py <datadir>            (offline replay)")


if __name__ == "__main__":
    main()
