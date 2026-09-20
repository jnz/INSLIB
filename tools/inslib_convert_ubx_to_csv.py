#!/usr/bin/env python3
"""Convert a sensor-board .ubx capture (see inslib_protocol.md) into the
INSLIB datasets replay format (config.yaml + CSVs, contract in
datasets/replay_format.py). Turns a recorded session into a dataset that
tools/replay.c and python/replay.py can score, with ref.csv and a fuller
config.yaml (automotive mode, score section) matching the
datasets/convert_*.py dataset layout.

Outputs (written to --outdir):
  - imu.csv, baro.csv, mag.csv
                  straight from the firmware's own class-0x40
                  messages (0x40/0x01, 0x40/0x02, 0x40/0x06) --
                  framing/checksum via inslib_ubx.py, the payload decode
                  is local (pyubx2 has no definition for this vendor
                  class). mag.csv is only written when the capture
                  actually carried magnetometer frames, and `mag: enable`
                  in the generated config.yaml follows the same fact:
                  the replay harnesses fail on `enable` without a file.
  - speed.csv     absolute ground speed from the host-produced odometry
                  frames (0x40/0x80, e.g. tools/inslib_obd_speed.py via
                  tools/inslib_hub.py). Only frames the hub already
                  stamped with an MCU t_us are convertible: the replay
                  format has one timebase and there is nothing here to
                  map a bare host clock onto. Written, and `speed:
                  enable` set, only when the capture carries such frames.
  - gnss.csv      from the passed-through receiver's NAV-PVT + NAV-COV
                  (0x01/0x07, 0x01/0x36) -- decoded with pyubx2 from
                  already checksum-verified frames. Only epochs with
                  fixType >= --min-fix-type are kept.
  - ref.csv       MVP placeholder: this tool has no independent ground
                  truth, so ref.csv is just the GNSS solution again (same
                  lat/lon/h/v_ned as gnss.csv) with roll/pitch/yaw
                  hardcoded to 0/0/0. Scoring ins position against this
                  ref.csv is close to circular (ins would be aided by the
                  same GNSS) and the attitude is not a real reference --
                  see the RAWX/SFRBX paragraph below for the planned fix.
                  config.yaml disables AHRS/ARS attitude scoring
                  accordingly (score: ahrs: 0).
  - config.yaml   replay configuration for tools/replay.c and
                  python/replay.py. Only written if the file does not
                  exist yet, so a hand-tuned config survives a re-run.
                  `mag: wmm_year` comes from the UTC date of the first
                  dated NAV-PVT: the replay harnesses build no reference
                  field without an epoch, and without one every
                  magnetometer sample is rejected. `speed: scale` is
                  fitted against gnss.csv's own velocity by
                  tools/inslib_speed_scale.py (used here as a library)
                  whenever that fit clears its own trust bar -- see
                  calibrate_speed_scale() below -- and left at the
                  library default (0.0 -> 1.0) with a comment explaining
                  why otherwise, never a number this tool cannot vouch for.
                  `automotive_mode` defaults to whatever this capture is:
                  on when it carries odometry, off otherwise -- a wheel/OBD
                  speed source is itself evidence of a road vehicle.
                  --automotive-mode/--no-automotive-mode overrides this.

Not produced yet (left as planned features -- see the comments at the
call sites below for where to hook them in):
  - A real, independent ref.csv: the receiver's RXM-RAWX (0x02/0x15) and
    RXM-SFRBX (0x02/0x13) passthrough messages carry raw pseudorange/
    carrier-phase observations and broadcast ephemeris -- everything
    a post-processing tool (e.g. RTKLIB's rnx2rtkp) needs for a
    post-processed (or at least a second, independent single-receiver)
    solution. Both already arrive at the pyubx2 decode step below
    (identity RXM-RAWX/RXM-SFRBX) but are only counted, not acted on.
    Once wired up: capture them into a RINEX obs/nav file (or RTCM3),
    run the post-processor, and write the result to ref.csv instead of
    the gnss.csv passthrough -- ideally against a nearby base station
    for an RTK-quality independent ground truth. The attitude
    columns would still need a real IMU-independent source (none exists
    yet); until then keep them at 0/0/0 or leave score:ahrs disabled.

Timebase: only the firmware's own class-0x40 messages carry the local
t_us clock (u64 microseconds since power-up, monotonic, never unwrapped
here -- inslib_protocol.md "Timebase"). NAV-PVT/NAV-COV do not, so each
GNSS epoch is stamped with the most recently seen IMU t_us. That folds the
USB/processing/passthrough latency into the fix time; config.yaml's
`gnss: delay_ms` is where that gets compensated at replay time
(REQ-VER-008). Absolute time comes from the 0x40/0x05 time sync frames,
which the firmware has already paired with the receiver's TIM-TP
announcement; --pos fits a linear clock model to them (see ClockFit).

Usage:
  inslib_convert_ubx_to_csv.py capture.ubx --outdir datasets/mydevice/trial1
  inslib_convert_ubx_to_csv.py capture.ubx --outdir out   # automotive_mode
                                                           # auto-detected from
                                                           # whether odometry
                                                           # is in the capture
  inslib_convert_ubx_to_csv.py capture.ubx --outdir out --no-automotive-mode

Requires pyubx2 (pip install pyubx2, listed in python/requirements.txt).

(c) Jan Zwiener (jan@zwiener.org)
"""
import argparse
import math
import os
import struct
import sys
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "..", "python"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "..", "datasets"))
from replay_format import (BARO_HEADER, GNSS_HEADER, IMU_HEADER_TEMP,    # noqa: E402
                           MAG_HEADER, REF_HEADER, SPEED_HEADER, gnss_row,
                           write_config)

from inslib_ubx import (BARO_FMT, ID_BARO, ID_IMU, ID_MAG,             # noqa: E402
                        ID_ODOMETRY, ID_STATUS, ID_TIMESYNC, IMU_FMT,
                        MAG_FMT, ODO_DIR_VALID, ODO_KIND_GROUND_SPEED,
                        ODO_T_DEGRADED, UbxFramer, decode_imu_status,
                        parse_odometry, parse_timesync)

try:
    from pyubx2 import UBXReader
except ImportError:
    sys.exit("pyubx2 is required to decode NAV-PVT/NAV-COV: pip install pyubx2"
              " (see python/requirements.txt)")

G0 = 9.80665     # standard gravity, for m/s^2 -> g in config comments

# --------------------------------------------------------------------------
# RTKLIB post-processed solution (.pos) as an INDEPENDENT reference
# --------------------------------------------------------------------------
# Without one, ref.csv is the receiver's own solution again, i.e. ins gets
# scored against the very data that aided it. A .pos from RTKPOST/rnx2rtkp
# against a base station breaks that circularity: same rover observations,
# but a second receiver and carrier-phase ambiguity resolution on top, so
# it is an order of magnitude better than the SPP fix in gnss.csv and
# arrived at independently of what the filter is fed.
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
POS_Q_TEXT = {1: "fix", 2: "float", 3: "sbas", 4: "dgps", 5: "single", 6: "ppp"}


@dataclass
class PosRow:
    gps_s: float                    # continuous GPS seconds since the epoch
    lat_deg: float
    lon_deg: float
    h_m: float
    q: int
    vel_ned: tuple                  # (vn, ve, vd) [m/s], or None
    sd_ned: tuple                   # (sdn, sde, sdu) [m]


def read_pos(path):
    """Parse an RTKLIB .pos. Velocity columns are optional in that format;
    rows without them get vel_ned = None."""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            c = line.split()
            if len(c) < 15:
                continue
            try:
                t = datetime.strptime(c[0] + " " + c[1], "%Y/%m/%d %H:%M:%S.%f")
                # The header states GPST, so this is already GPS time and no
                # leap-second correction applies.
                gps_s = (t.replace(tzinfo=timezone.utc) - GPS_EPOCH).total_seconds()
                vel = None
                if len(c) >= 18:
                    # vu is UP; the replay format is NED, hence the sign.
                    vel = (float(c[15]), float(c[16]), -float(c[17]))
                rows.append(PosRow(gps_s, float(c[2]), float(c[3]), float(c[4]),
                                   int(c[5]), vel,
                                   (float(c[7]), float(c[8]), float(c[9]))))
            except (ValueError, IndexError):
                continue
    return rows


def navpvt_leap_seconds(msg):
    """GPS - UTC in whole seconds, read out of one NAV-PVT.

    That message carries both iTOW (GPS time of week) and the UTC
    date/time of the SAME epoch, so their difference is the current
    leap-second count - no table, no expiry, no assumption about the year
    this runs in."""
    if not (getattr(msg, "validDate", 0) and getattr(msg, "validTime", 0)):
        return None
    try:
        utc = datetime(msg.year, msg.month, msg.day, msg.hour, msg.min,
                       msg.second, tzinfo=timezone.utc)
    except (ValueError, AttributeError):
        return None
    utc_s = (utc - GPS_EPOCH).total_seconds() + getattr(msg, "nano", 0) * 1e-9
    d = getattr(msg, "iTOW", 0) / 1e3 - (utc_s % 604800.0)
    # Fold a week boundary between the two representations back in.
    if d > 302400.0:
        d -= 604800.0
    elif d < -302400.0:
        d += 604800.0
    return int(round(d))


def navpvt_decimal_year(msg):
    """Capture epoch as a decimal year, the WMM's time argument.

    Read out of NAV-PVT rather than off the clock of whoever runs the
    conversion, so a capture converted a year later still gets the field
    that was actually there. Month granularity is plenty: secular
    variation moves the declination by about 0.1 deg per year."""
    if not getattr(msg, "validDate", 0):
        return None
    try:
        utc = datetime(msg.year, msg.month, msg.day, tzinfo=timezone.utc)
    except (ValueError, AttributeError):
        return None
    start = datetime(utc.year, 1, 1, tzinfo=timezone.utc)
    return round(utc.year + (utc - start).total_seconds() / (365.25 * 86400.0), 1)


class ClockFit:
    """Linear model gps_s = a + b * t_us, from 0x40/0x05 time sync frames.

    A single offset would not do: the MCU crystal is off by tens of ppm
    (64.6 ppm on the first real drive), which over a quarter of an hour is
    already tens of milliseconds - at 30 m/s that is metres of reference
    error, the same order as the quantity being measured. The slope
    absorbs it, and the fit also bridges pulses that were dropped."""

    def __init__(self):
        self.cand = []              # (t_us, gps_s, pulse_count)
        self.pairs = []             # the validated subset actually fitted
        self.n_rejected = 0
        self.n_no_gps = 0           # edges captured without an announcement
        self.n_suspect = 0
        self.a = self.b = None
        self.resid_rms_s = None
        # The receiver may report its time on the UTC scale instead of GPS
        # (TS_UTC_BASE). The .pos is GPST, so mixing the two puts the
        # whole reference out by the leap-second count - 18 s in 2026,
        # which at 100 km/h is half a kilometre of reference error. Both
        # facts are read from the stream: the base from the time sync
        # flags, the leap count from NAV-PVT (see navpvt_leap_seconds).
        self.utc_base = None
        self.leap_s = None

    def add(self, ts):
        """Take one parsed 0x40/0x05 frame.

        Two kinds are dropped here rather than fitted. An edge without an
        announcement carries no time at all. A TP_SUSPECT edge carries one
        the firmware could not confirm advanced by exactly one second, and
        a pair that is off by a whole second drags the slope far more than
        losing one pulse costs - at 1 Hz there are hundreds left."""
        if not ts["gps_valid"]:
            self.n_no_gps += 1
            return
        if self.utc_base is None:
            self.utc_base = ts["utc_base"]
        if ts["suspect"]:
            self.n_suspect += 1
            return
        self.cand.append((ts["t_us"], ts["gps_s"], ts["count"]))

    def _validate(self):
        """Second net under the firmware's own pairing.

        Both sequences advance by one per second, so for a correct pair
        (GPS time - pulse count) is a constant. The modal value is that
        constant, and anything else is a pair that slipped by a whole
        second. The firmware pairs the announcement with the edge itself
        now and flags what it cannot confirm, so this should find nothing;
        it is kept because a slipped pair is invisible in the result (the
        fit simply comes out wrong) and the check costs one pass."""
        if not self.cand:
            return
        keys = Counter(int(round(g)) - c for _, g, c in self.cand)
        key, _n = keys.most_common(1)[0]
        self.pairs = [(t, g) for t, g, c in self.cand if int(round(g)) - c == key]
        self.n_rejected = len(self.cand) - len(self.pairs)

    def solve(self):
        self._validate()
        if self.utc_base:
            if self.leap_s is None:
                raise SystemExit(
                    "The time sync frames report UTC (TS_UTC_BASE) but no NAV-PVT in the"
                    " capture carried a valid date/time to read the leap-second\n"
                    "count from. Converting anyway would put ref.csv out by that"
                    " count (18 s in 2026, ~500 m at 100 km/h), so this stops here.")
            self.pairs = [(t, g + self.leap_s) for t, g in self.pairs]
        n = len(self.pairs)
        if n < 2:
            return False
        # Referenced to the means so the normal equations stay conditioned:
        # t_us is ~1e9 while the span is only ~1e3 s.
        tm = sum(p[0] for p in self.pairs) / n
        gm = sum(p[1] for p in self.pairs) / n
        sxx = sum((p[0] - tm) ** 2 for p in self.pairs)
        if sxx <= 0:
            return False
        self.b = sum((p[0] - tm) * (p[1] - gm) for p in self.pairs) / sxx
        self.a = gm - self.b * tm
        self.resid_rms_s = math.sqrt(
            sum((self.a + self.b * t - g) ** 2 for t, g in self.pairs) / n)
        return True

    def gps_to_t_us(self, gps_s):
        return int(round((gps_s - self.a) / self.b))

    def ppm(self):
        """MCU timer rate error against GPS time [ppm], positive = fast."""
        return (1.0 / (self.b * 1e6) - 1.0) * 1e6


# --------------------------------------------------------------------------
# Firmware class-0x40 payload decode. pyubx2 handles the UBX framing/
# resync/checksum (it returns an "UNKNOWN" message for these, since it has
# no definition for this vendor class); the payload itself is this
# board's own format (tools/inslib_protocol.md), decoded here.
# --------------------------------------------------------------------------

CLASS_CUSTOM = 0x40
G_MPS2 = 9.80665            # g -> m/s^2
DEG2RAD = math.pi / 180.0   # deg/s -> rad/s

_IMU_STRUCT = struct.Struct(IMU_FMT)     # t_us, acc[3] g, gyr[3] dps, status, seq
_BARO_STRUCT = struct.Struct(BARO_FMT)   # t_us, pressure Pa, temp C
_MAG_STRUCT = struct.Struct(MAG_FMT)     # t_us, mag[3] uT, temp C


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
class BaroSample:
    t_us: int
    pressure_pa: float
    temp_c: float


@dataclass
class MagSample:
    t_us: int
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


def decode_baro(payload):
    """0x40/0x02 payload -> BaroSample, or None on a length mismatch."""
    if len(payload) != _BARO_STRUCT.size:
        return None
    t_us, pressure_pa, temp_c = _BARO_STRUCT.unpack(payload)
    return BaroSample(t_us=t_us, pressure_pa=pressure_pa, temp_c=temp_c)


def decode_mag(payload):
    """0x40/0x06 payload -> MagSample, or None on a length mismatch.

    uT already, and already in the same body frame as the IMU message
    (the driver's mounting remap), so unlike decode_imu there is nothing
    to convert. What the replay still needs is the mag: calibration
    (hard/soft iron), which tools/inslib_calib_gui.py measures."""
    if len(payload) != _MAG_STRUCT.size:
        return None
    t_us, mx, my, mz, temp_c = _MAG_STRUCT.unpack(payload)
    return MagSample(t_us=t_us, mag_ut=(mx, my, mz), temp_c=temp_c)


# --------------------------------------------------------------------------
# GNSS (standard u-blox passthrough) decode via pyubx2 -- same approach as
# GNSS epoch assembly, local to this tool (not imported from the
# library) because the converter must run without the built INSLIB and
# this converter must run without it.
# --------------------------------------------------------------------------

@dataclass
class GnssEpoch:
    itow_ms: int
    fix_type: int
    lat_deg: float
    lon_deg: float
    h_ell_m: float                  # WGS84 ellipsoid height (nav_suite's datum)
    vel_ned: tuple
    cov_pos6: tuple                 # NED (nn,ne,nd,ee,ed,dd) [m^2] or None
    cov_vel6: tuple                 # NED (nn,ne,nd,ee,ed,dd) [(m/s)^2] or None
    hacc_m: float                   # NAV-PVT scalar accuracies, used as a
    vacc_m: float                   # diagonal covariance fallback when
    sacc_mps: float                 # NAV-COV wasn't matched (see below)


class UbxGnssDecoder:
    """NAV-PVT -> GnssEpoch; NAV-COV's full NED covariance is buffered and
    attached to the fix of the same epoch (matched by iTOW, within one
    epoch's slop -- NAV-COV arrives just after its NAV-PVT). Other
    identities (RXM-RAWX, RXM-SFRBX, NAV-TIMEGPS, ...) are counted only,
    see the module docstring's "planned features" section. `msg` is
    already a parsed UBXMessage (pyubx2 did the framing/checksum/decode)."""

    def __init__(self):
        self.identities = Counter()
        self._last_cov = None   # (itow_ms, cov_pos6, cov_vel6)

    def decode(self, msg):
        self.identities[msg.identity] += 1
        if msg.identity == "NAV-COV":
            self._store_cov(msg)
            return None
        if msg.identity != "NAV-PVT":
            return None
        itow = getattr(msg, "iTOW", -1)
        cov_pos6, cov_vel6 = self._cov_for(itow)
        return GnssEpoch(
            itow_ms=itow, fix_type=getattr(msg, "fixType", 0),
            lat_deg=getattr(msg, "lat", float("nan")),
            lon_deg=getattr(msg, "lon", float("nan")),
            h_ell_m=getattr(msg, "height", 0) / 1000.0,   # mm -> m
            vel_ned=(getattr(msg, "velN", 0) / 1000.0,
                     getattr(msg, "velE", 0) / 1000.0,
                     getattr(msg, "velD", 0) / 1000.0),
            cov_pos6=cov_pos6, cov_vel6=cov_vel6,
            hacc_m=getattr(msg, "hAcc", 0) / 1000.0,
            vacc_m=getattr(msg, "vAcc", 0) / 1000.0,
            sacc_mps=getattr(msg, "sAcc", 0) / 1000.0,
        )

    def _store_cov(self, msg):
        if not (getattr(msg, "posCovValid", 0) and getattr(msg, "velCovValid", 0)):
            return
        cov_pos6 = (msg.posCovNN, msg.posCovNE, msg.posCovND,
                    msg.posCovEE, msg.posCovED, msg.posCovDD)
        cov_vel6 = (msg.velCovNN, msg.velCovNE, msg.velCovND,
                    msg.velCovEE, msg.velCovED, msg.velCovDD)
        self._last_cov = (getattr(msg, "iTOW", -1), cov_pos6, cov_vel6)

    def _cov_for(self, itow_ms):
        if self._last_cov is None:
            return None, None
        cov_itow, cov_pos6, cov_vel6 = self._last_cov
        if itow_ms < 0 or abs(itow_ms - cov_itow) <= 1000:
            return cov_pos6, cov_vel6
        return None, None


# --------------------------------------------------------------------------
# Conversion driver
# --------------------------------------------------------------------------

def _median(values):
    v = sorted(values)
    if not v:
        return 0.0
    mid = len(v) // 2
    return float(v[mid]) if len(v) % 2 else 0.5 * float(v[mid - 1] + v[mid])


def convert(ubx_path, outdir, min_fix_type=3, pos_rows=None, pos_max_q=2):
    os.makedirs(outdir, exist_ok=True)
    gnss_decoder = UbxGnssDecoder()
    clock = ClockFit()

    n = Counter()
    first_imu_t_us = None
    last_imu_t_us = None
    status_last = None
    speed_rows = []

    imu_f = open(os.path.join(outdir, "imu.csv"), "w", encoding="utf-8")
    baro_f = open(os.path.join(outdir, "baro.csv"), "w", encoding="utf-8")
    mag_path = os.path.join(outdir, "mag.csv")
    mag_f = open(mag_path, "w", encoding="utf-8")
    gnss_f = open(os.path.join(outdir, "gnss.csv"), "w", encoding="utf-8")
    ref_f = open(os.path.join(outdir, "ref.csv"), "w", encoding="utf-8")
    imu_f.write(IMU_HEADER_TEMP)
    baro_f.write(BARO_HEADER)
    mag_f.write(MAG_HEADER)
    gnss_f.write(GNSS_HEADER)
    gnss_f.write("# GNSS-only passthrough (NAV-PVT + NAV-COV), no RTKLIB"
                  " post-processing -- see inslib_convert_ubx_to_csv.py\n")
    ref_f.write(REF_HEADER)
    if pos_rows is None:
        ref_f.write("# MVP placeholder: this is the GNSS solution again, NOT an"
                    " independent reference. roll/pitch/yaw hardcoded to 0/0/0"
                    " -- see inslib_convert_ubx_to_csv.py module docstring\n")
    else:
        ref_f.write("# RTKLIB post-processed solution (--pos): carrier-phase against a"
                    " base station, independent of the SPP fix in gnss.csv that aids"
                    " the filter. roll/pitch/yaw stay 0/0/0 (no attitude reference"
                    " exists), so attitude must not be scored.\n")

    def _imu_status_is_legacy_f32(payload):
        """Does this 0x40/0x01 frame predate the status word?

        The status word replaced an f32 die temperature in the same 38
        bytes and nothing in the frame says which one it is, so an old
        capture decodes silently: temperature 0.0 and saturation/
        calibration flags read out of float mantissa bits.

        Bits 14..31 of the status word are reserved and sent as zero,
        so any frame with them set is either older than the word or
        newer than this decoder. The float test separates the two: a
        legacy temperature lands in a die-temperature band, while a
        reserved bit someone adds at 14 reinterprets as a denormal.
        Deliberately only reports -- the protocol requires a decoder to
        IGNORE reserved bits rather than reject the frame, and the
        accel/gyro/timestamp fields are identical in both layouts, so
        such a capture still converts into a usable dataset."""
        status = struct.unpack_from("<I", payload, 8 + 6 * 4)[0]
        if (status >> 14) == 0:
            return False
        as_f32 = struct.unpack("<f", struct.pack("<I", status))[0]
        return -60.0 <= as_f32 <= 150.0

    def handle_frame(cls, mid, payload, frame):
        nonlocal first_imu_t_us, last_imu_t_us, status_last
        if cls == CLASS_CUSTOM:
            if mid == ID_IMU:
                s = decode_imu(payload)
                if s is None:
                    n["imu_bad_len"] += 1
                    return
                if _imu_status_is_legacy_f32(payload):
                    n["imu_legacy_status"] += 1
                if first_imu_t_us is None:
                    first_imu_t_us = s.t_us
                last_imu_t_us = s.t_us
                # NOTE (assumption): the IMU is assumed to already output
                # body-FRD axes as-is (x fwd, y right, z down). If the
                # physical mounting differs, apply the rotation here.
                # The status word carries tenths of a degree, so %.1f is
                # the wire resolution exactly and the column is lossless.
                imu_f.write("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.1f\n" % (
                    s.t_us, s.gyr_rps[0], s.gyr_rps[1], s.gyr_rps[2],
                    s.acc_mps2[0], s.acc_mps2[1], s.acc_mps2[2], s.temp_c))
                n["imu"] += 1
            elif mid == ID_BARO:
                s = decode_baro(payload)
                if s is None:
                    n["baro_bad_len"] += 1
                    return
                baro_f.write("%d,%.6f\n" % (s.t_us, s.pressure_pa))
                n["baro"] += 1
            elif mid == ID_MAG:
                s = decode_mag(payload)
                if s is None:
                    n["mag_bad_len"] += 1
                    return
                # Same assumption as the IMU above: the firmware already
                # delivers body-FRD axes (inslib_protocol.md 0x40/0x06 says
                # the driver's mounting remap has run), so this is a
                # straight copy.
                mag_f.write("%d,%.6g,%.6g,%.6g\n" % (
                    s.t_us, s.mag_ut[0], s.mag_ut[1], s.mag_ut[2]))
                n["mag"] += 1
            elif mid == ID_TIMESYNC:
                # One exact (t_us, GPS time) pair per second, already
                # paired with its TIM-TP announcement by the firmware
                # (inslib_protocol.md 0x40/0x05). ClockFit decides which
                # of them are fit for the model.
                ts = parse_timesync(payload)
                if ts is None:
                    n["timesync_bad_len"] += 1
                    return
                clock.add(ts)
                n["timesync"] += 1
            elif mid == ID_STATUS:
                # Status counters (tx_dropped, imu_overruns, ...): keep
                # the raw payload, format not asserted here (firmware may
                # carry more fields than inslib_protocol.md's 6-counter
                # table without breaking this converter).
                status_last = payload
            elif mid == ID_ODOMETRY:
                # Host-produced ground speed -> speed.csv (REQ-NAV-068).
                # Buffered rather than streamed because it is the only
                # stream whose frames can reach the capture out of order:
                # they are injected by the hub on the host clock while
                # everything else is written in MCU order.
                o = parse_odometry(payload)
                if o is None:
                    n["odometry_bad_len"] += 1
                    return
                n["odometry"] += 1
                if o["kind"] != ODO_KIND_GROUND_SPEED:
                    # A different measurement MODEL (an airspeed would
                    # need a wind assumption), not another source of |v|.
                    n["odometry_other_kind"] += 1
                    return
                if not o["t_us_valid"]:
                    # The producer's half of a frame the hub never
                    # completed, typically recorded before any IMU sample
                    # gave it a clock to map onto.
                    n["odometry_no_t_us"] += 1
                    return
                if not (math.isfinite(o["speed_mps"]) and o["speed_mps"] >= 0.0):
                    n["odometry_bad_value"] += 1
                    return
                if o["flags"] & ODO_T_DEGRADED:
                    n["odometry_t_degraded"] += 1
                speed_rows.append(o)
            return

        # Standard u-blox: hand pyubx2 ONE complete, already checksum-
        # verified frame, so it never has to guess at a protocol from a
        # byte stream (and never mistakes a 0x24 in an IMU float for an
        # NMEA header).
        try:
            msg = UBXReader.parse(frame)
        except Exception:
            n["parse_errors"] += 1
            return

        if msg.identity == "NAV-PVT":
            if clock.leap_s is None:
                leap = navpvt_leap_seconds(msg)
                if leap is not None:
                    clock.leap_s = leap
                    n["leap_seconds"] = leap
            if not n["wmm_year"]:
                year = navpvt_decimal_year(msg)
                if year is not None:
                    n["wmm_year"] = year
            # falls through: NAV-PVT is also the GNSS epoch source below

        if msg.identity == "TIM-TP":
            # Not expected: the firmware consumes TIM-TP to build 0x40/0x05
            # and does not forward it. A capture that still carries it came
            # from older firmware and has no time sync frames to fit, which
            # the --pos path reports rather than failing on obscurely.
            n["tim_tp"] += 1
            return

        # Standard u-blox frame (GNSS passthrough).
        epoch = gnss_decoder.decode(msg)
        if epoch is None:
            return
        n["nav_pvt"] += 1
        if epoch.fix_type < min_fix_type:
            n["nav_pvt_rejected"] += 1
            return
        if last_imu_t_us is None:
            n["gnss_skipped_no_imu_yet"] += 1
            return
        t_us = last_imu_t_us
        # Prefer NAV-COV's full NED covariance; else a diagonal built from
        # NAV-PVT's own scalar accuracies (same fallback as the C receiver's
        # CsvCaptureHandler) rather than leaving it all-zero -- a NAV-COV-
        # less epoch still carries per-epoch accuracy in hAcc/vAcc/sAcc,
        # only the position/velocity cross-terms are unknown (0).
        def var(x):
            return x * x if math.isfinite(x) and x > 0.0 else 0.0
        if epoch.cov_pos6 is not None:
            cov_pos6 = epoch.cov_pos6
        else:
            cov_pos6 = (var(epoch.hacc_m), 0.0, 0.0,
                        var(epoch.hacc_m), 0.0, var(epoch.vacc_m))
        if epoch.cov_vel6 is not None:
            cov_vel6 = epoch.cov_vel6
        else:
            sv = var(epoch.sacc_mps)
            cov_vel6 = (sv, 0.0, 0.0, sv, 0.0, sv)
        vel_ok = all(math.isfinite(v) for v in epoch.vel_ned)
        gnss_f.write(gnss_row(t_us, epoch.lat_deg, epoch.lon_deg,
                              epoch.h_ell_m, cov_pos6, epoch.vel_ned,
                              cov_vel6, vel_ok))
        if pos_rows is None:
            ref_f.write("%d,%.10f,%.10f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n" % (
                t_us, epoch.lat_deg, epoch.lon_deg, epoch.h_ell_m,
                0.0, 0.0, 0.0, epoch.vel_ned[0], epoch.vel_ned[1], epoch.vel_ned[2]))
        n["gnss"] += 1

    framer = UbxFramer()
    with open(ubx_path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            for cls, mid, payload, frame in framer.feed(chunk):
                handle_frame(cls, mid, payload, frame)
    n["skipped_bytes"] = framer.n_resync

    # A capture with no usable fix (indoors, no sky view) leaves ref.csv
    # empty otherwise -- both replay harnesses need >= 1 row to even load
    # the dataset (they sys.exit on an empty ref.csv), which shows up as a
    # crash far from the real cause. lat/lon/h 0/0/0 is an obvious sentinel,
    # not a guessed position: consistent with this codebase's own "0 ->
    # unknown/disabled" convention elsewhere (e.g. mag: wmm_year). It only
    # ever fires when nothing else already wrote a row.
    if pos_rows is None and n["gnss"] == 0 and first_imu_t_us is not None:
        ref_f.write("# no GNSS fix anywhere in this capture -- one placeholder"
                    " row (lat/lon/h 0/0/0) so tools requiring a non-empty"
                    " ref.csv (e.g. python/replay.py, tools/replay.c) can load"
                    " this dataset. NOT a real position -- aiding: none /"
                    " free_inertial_start is the only sound way to score or"
                    " replay it.\n")
        ref_f.write("%d,%.10f,%.10f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n" % (
            first_imu_t_us, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        n["ref_placeholder"] = 1

    # ref.csv from the post-processed solution, once the whole capture has
    # been seen: the clock fit needs all the time sync pairs, and the .pos
    # is on GPS time while every CSV in this format is on the MCU counter.
    if pos_rows is not None:
        if not clock.solve():
            raise SystemExit(
                "--pos given, but the capture carries no usable 0x40/0x05 time sync"
                " frames to tie GPS time to the MCU counter.\n"
                + ("It does carry TIM-TP, so this was recorded with firmware that"
                   " still forwards it instead of pairing it: convert it with the"
                   " version of this tool that matches that firmware.\n"
                   if n["tim_tp"] else
                   "They only appear once the receiver has a time lock, so a capture"
                   " recorded without sky view will not have them.\n")
                + ("%d edge(s) were captured without an announcement, %d were flagged"
                   " suspect.\n" % (clock.n_no_gps, clock.n_suspect)
                   if clock.n_no_gps or clock.n_suspect else ""))
        n["clock_pairs"] = len(clock.pairs)
        n["clock_rejected"] = clock.n_rejected
        n["clock_no_gps"] = clock.n_no_gps
        n["clock_suspect"] = clock.n_suspect
        n["clock_utc_base"] = 1 if clock.utc_base else 0
        n["clock_ppm"] = clock.ppm()
        n["clock_resid_us"] = clock.resid_rms_s * 1e6
        t_lo, t_hi = min(p[0] for p in clock.pairs), max(p[0] for p in clock.pairs)
        for r in pos_rows:
            if r.q > pos_max_q:
                n["pos_rejected_q"] += 1
                continue
            if r.vel_ned is None:
                n["pos_no_velocity"] += 1
            t_us = clock.gps_to_t_us(r.gps_s)
            # Outside the pulse span the fit is extrapolation, not
            # interpolation; a reference epoch is worthless if its
            # timestamp is guessed, so drop rather than stretch.
            if not (t_lo <= t_us <= t_hi):
                n["pos_outside_clock_span"] += 1
                continue
            v = r.vel_ned or (0.0, 0.0, 0.0)
            ref_f.write("%d,%.10f,%.10f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n" % (
                t_us, r.lat_deg, r.lon_deg, r.h_m, 0.0, 0.0, 0.0, v[0], v[1], v[2]))
            n["ref_from_pos"] += 1

    imu_f.close()
    baro_f.close()
    mag_f.close()
    gnss_f.close()
    ref_f.close()

    # A capture from a board without a magnetometer would otherwise leave
    # a header-only mag.csv behind, which reads as "there is magnetometer
    # data here" to anyone looking at the directory and makes `mag:
    # enable` in a hand-edited config a puzzle rather than an error.
    if n["mag"] == 0:
        os.remove(mag_path)

    # speed.csv last, and only when there is something to put in it: same
    # reasoning as mag.csv above, an empty file reads as a stream that
    # exists. Sorted because the hub injects these frames on the host
    # clock, so capture order is not t_us order, and tools/replay.c reads
    # the rows in file order (python/replay.py sorts, the C harness does
    # not, and the two must not disagree about the same dataset).
    if speed_rows:
        speed_rows.sort(key=lambda o: o["t_us"])
        with open(os.path.join(outdir, "speed.csv"), "w",
                  encoding="utf-8") as speed_f:
            speed_f.write(SPEED_HEADER)
            speed_f.write("# from 0x40/0x80 odometry frames. Both replay"
                          " harnesses read the first two columns only; the"
                          " rest is the producer's own record: stddev_mps,"
                          " delay_ms, reverse (1 = travelling backwards,"
                          " blank = direction unknown)\n")
            for o in speed_rows:
                speed_f.write("%d,%.4f,%.4f,%d,%s\n" % (
                    o["t_us"], o["speed_mps"], o["stddev_mps"],
                    o["delay_ms"],
                    "1" if o["reverse"] else
                    ("0" if o["flags"] & ODO_DIR_VALID else "")))
        n["speed"] = len(speed_rows)
        # Constants for config.yaml: the format keeps uncertainty and delay
        # there rather than per row, so the dataset is described in one
        # place. Median rather than mean, so a handful of outliers (a
        # stalled OBD round trip) cannot move what every sample is charged.
        n["speed_stddev_mps"] = _median([o["stddev_mps"] for o in speed_rows])
        n["speed_delay_ms"] = _median([o["delay_ms"] for o in speed_rows])
        n["speed_reverse"] = sum(1 for o in speed_rows if o["reverse"])

    return n, status_last, gnss_decoder.identities


# --------------------------------------------------------------------------
# config.yaml
# --------------------------------------------------------------------------

SPEED_SCALE_FIXME = (
    0.0, "0 -> 1.0. Systematic and vehicle-specific, so calibrate it against"
        " GNSS: PID 0x0D is the ECU's own value, not the dashboard reading"
        " that type approval keeps on the high side")


def calibrate_speed_scale(outdir):
    """(scale, comment) for config.yaml's `speed: scale`, fitted from the
    gnss.csv/speed.csv just written to `outdir` -- reuses
    tools/inslib_speed_scale.py as a library rather than a second copy of
    its parsing/fit logic.

    Falls back to SPEED_SCALE_FIXME (the library default, 0.0 -> 1.0) with
    a comment saying why whenever the fit does not clear that script's own
    trust bar (inslib_speed_scale.trust_reason): a plausible-looking number
    baked into config.yaml unattended is worse than the FIXME it replaces,
    since nothing about a config file says "this one was never checked"."""
    import inslib_speed_scale as ss
    gnss_rows = ss.load_gnss(os.path.join(outdir, "gnss.csv"))
    speed_rows = ss.load_speed(os.path.join(outdir, "speed.csv"))
    if len(gnss_rows) < 2:
        return (0.0, SPEED_SCALE_FIXME[1] + " (this capture has no usable"
                " GNSS velocity fix, vel_ok=1, to calibrate against yet)")

    # gnss: delay_ms is written as 0.0 just below (see its own comment):
    # calibrate against the same event-time convention this config will
    # replay with, not against whatever a previous hand-tuned config used.
    samples = ss.collect_samples(gnss_rows, speed_rows, gnss_delay_ms=0.0,
                                 min_speed_mps=3.0, max_gap_s=0.5,
                                 max_accel_mps2=1.5, include_reverse=False)
    if len(samples) < ss.MIN_SAMPLES:
        return (0.0, SPEED_SCALE_FIXME[1] +
                " (only %d sample(s) against GNSS after filtering, need >="
                " %d -- not enough speed variation in this capture yet)"
                % (len(samples), ss.MIN_SAMPLES))

    fit = ss.fit_scale(samples)
    reason = ss.trust_reason(samples, fit)
    if reason:
        return (0.0, SPEED_SCALE_FIXME[1] +
                " (tools/inslib_speed_scale.py measured %.4f from this"
                " capture but flagged it -- %s; review with its --plot"
                " before applying it by hand)" % (fit["scale"], reason))
    return (round(fit["scale"], 4),
           "measured against GNSS by tools/inslib_speed_scale.py: %.4f +/-"
           " %s from %d samples, %.1f-%.1f m/s, RMS %.3f -> %.3f m/s."
           " Re-run it after a mechanical change (tires, gearing)"
           % (fit["scale"], ss.fmt_unc(fit["sigma_scale"]), fit["n"],
              fit["odo_min"], fit["odo_max"], fit["rms_raw"], fit["rms_fit"]))


# Tuning carried over from datasets/tunnel/config.yaml, which was tuned on
# a recording from this same board and receiver, so it is a far better
# starting point for a road vehicle than the library's generic MEMS
# fallbacks. Still worth re-checking per vehicle: the lever arm is the
# antenna position on THAT car, and delay_ms compensates the latency of
# stamping GNSS epochs with the last IMU t_us (see "Timebase" above).
AUTOMOTIVE_IMU = {
    "gyr_psd": 5.0e-07,
    "acc_psd": 1.0e-05,
    "gyr_bias_rw": 1.0e-05,
    "acc_bias_rw": 1.0e-04,
    "pos_pred_stddev_m_sqrts": 0.05,
    "vel_pred_stddev_mps_sqrts": 0.01,
    "rpy_pred_stddev_rad_sqrts": 0.0,
    "zero_vel_stddev_mps": 0.02,
    "zero_rot_stddev_deg": 0.5,
    "auto_zupt_static_gyr_stddev_deg": 0.8,
    "auto_zupt_static_acc_stddev_mps2": 0.25,
    "auto_zupt_static_gyr_deg": 4.0,
    "auto_zupt_static_acc_mps2": 0.5,
    "auto_zupt_max_vel_mps": 1.5,
    "auto_zupt_max_vel_stddev_mps": 0.5,
    "auto_zupt_dwell_sec": 1.0,
    "auto_zupt_min_interval_sec": 1.0,
}


def write_cfg(path_out, name, automotive_mode, leverarm_frd, from_pos=False,
              has_mag=False, wmm_year=0.0, has_speed=False,
              speed_stddev_mps=0.0, speed_delay_ms=0.0,
              speed_scale=SPEED_SCALE_FIXME, automotive_mode_reason=None):
    """Replay configuration for the converted dataset. IMU noise defaults
    are the library's generic MEMS fallback (src/sensor_defaults.h,
    src/ins.c INS_DEFAULT_ACC_*) -- replace with this board's actual IMU
    datasheet/Allan-variance figures once known. GNSS covariance comes
    straight from NAV-COV when present, so no pos/vel stddev fallback
    should normally be needed; small non-zero values are still set so a
    NAV-COV-less epoch (all-zero covariance) does not get treated as
    perfectly known."""
    gyr_arw_rps_sqrthz = 0.02 * (3.14159265358979323846 / 180.0)  # INS_DEFAULT_GYR_ARW_RPS_SQRTHZ
    acc_vrw_mps2_sqrthz = 350e-6 * G0                              # INS_DEFAULT_ACC_VRW_MPS2_SQRTHZ

    extra = []
    if automotive_mode:
        extra = [
            ("max_deadreckoning_sec",
             (40.0, "a road vehicle can lose GNSS for a long time"
                    " (tunnel, underpass)")),
            ("automotive_min_yaw_stddev_deg",
             (15.0, "course over ground is a noisy heading, do not let a fast"
                    " fix claim more than this")),
        ]

    write_config(path_out, [
        ("name", name),
        ("aiding", ("gnss", "NAV-PVT/NAV-COV passthrough, see gnss.csv header")),
        ("init", "auto"),
        ("automotive_mode", (1 if automotive_mode else 0,
                              "derive yaw from GNSS course over ground"
                              + (" (%s)" % automotive_mode_reason
                                 if automotive_mode_reason else ""))),
        ("imu", dict(AUTOMOTIVE_IMU, **{
            "gyr_psd": (AUTOMOTIVE_IMU["gyr_psd"],
                        "from datasets/tunnel (same board), not a datasheet figure"),
        }) if automotive_mode else {
            "gyr_psd": (gyr_arw_rps_sqrthz ** 2,
                        "FIXME: generic MEMS default, replace with datasheet/Allan figures"),
            "acc_psd": acc_vrw_mps2_sqrthz ** 2,
            "gyr_bias_rw": 2e-6,
            "acc_bias_rw": 3e-5,
        }),
        ("gnss", {
            "leverarm_frd": (list(leverarm_frd),
                              "FIXME: measure antenna vs. IMU on THIS vehicle"),
            "pos_stddev_fallback_m": ([0.8, 1.5] if automotive_mode else [5.0, 10.0],
                                       "only used for a NAV-COV-less epoch"),
            "vel_stddev_fallback_mps": 0.2 if automotive_mode else 0.5,
            # 0 is only a placeholder, it is usually wrong. Stamping each
            # GNSS epoch with the most recent IMU t_us removes the transport
            # delay, but not the group delay of the receiver's own
            # navigation filter, which depends on its dynamic model (about
            # 200 ms for u-blox F9P/X20P in Airborne 4g, possibly different
            # in Automotive). No default fits every receiver setup, so the
            # value has to be tuned per recording, e.g. by cross-correlating
            # GNSS vs baro_alt
            "delay_ms": (0.0, "TUNE ME: receiver filter group delay, e.g. ~200"
                              " for u-blox F9P/X20P in Airborne 4g"),
        }),
        ("mag", {
            # Only ever 1 when the capture really carried 0x40/0x06 frames:
            # the replay harnesses treat `enable` as "open mag.csv" and
            # exit when it is not there.
            "enable": (1 if has_mag else 0,
                        "0x40/0x06 frames in the capture" if has_mag
                        else "no magnetometer in this capture"),
            "stddev_ut": (0.0, "0 -> default; raise it for a platform whose"
                                " own field changes with what it is doing"),
            # The replay harnesses build no reference field without an
            # epoch, and with no reference field every magnetometer sample
            # is rejected and yaw runs unaided -- unlike the live receiver
            # (tools/insrcv.c), which falls back to GPS/host time.
            "wmm_year": (wmm_year,
                          "WMM epoch, from the capture's own NAV-PVT date"
                          if wmm_year > 0.0 else
                          "FIXME: no valid NAV-PVT date in the capture; 0"
                          " leaves the mag decoded but never fused"),
            "estimate_bias": (0, "1 -> 18-state hard-iron estimation on top"
                                  " of the fixed calibration below"),
            # Left at the identity/zero no-op on purpose: the hard and soft
            # iron belong to the unit AND its housing, so they have to be
            # measured on the assembled thing rather than guessed here.
            # tools/inslib_calib_gui.py writes both keys.
            "misalignment": ([0.0] * 9,
                              "col-major 3x3 soft iron + alignment to the IMU,"
                              " all-0 -> identity (tools/inslib_calib_gui.py)"),
            "fixed_bias": ([0.0, 0.0, 0.0], "hard iron [uT], same tool"),
        }),
        ("baro", {
            "enable": 1,
            "stddev_m": (0.8 if automotive_mode else 0.0,
                          "from datasets/tunnel" if automotive_mode
                          else "0 -> default"),
            "acc_bias_init_mps2": 0.08,
        }),
    ] + extra + [
        # Absolute speed aiding (REQ-NAV-068), from the capture's own
        # 0x40/0x80 odometry frames. Enabled on the same fact as `mag`
        # above: on when speed.csv was written, off (but visible, so the
        # knobs do not have to be looked up) when the capture carried no
        # convertible odometry. Every 0 means "library default".
        #
        # stddev_mps and delay_ms are the medians the producer reported
        # per sample, promoted to constants because that is where this
        # format keeps them. A dataset whose speed uncertainty really
        # varies (heavy braking widens it, see inslib_protocol.md
        # 0x40/0x80) is served worse by a median than by its own columns,
        # which stay in speed.csv for exactly that day.
        ("speed", {
            "enable": (1 if has_speed else 0,
                        "0x40/0x80 odometry frames in the capture"
                        if has_speed
                        else "1 + a speed.csv next to this file to use it"),
            "scale": speed_scale,
            "stddev_mps": (round(speed_stddev_mps, 4) if has_speed else 0.0,
                            "median of the per-sample 1-sigma the producer"
                            " reported" if has_speed
                            else "per-sample 1-sigma; 0 -> default (0.080,"
                                 " the 1 km/h quantisation of OBD-II PID 0x0D)"),
            "stddev_rel": (0.0, "speed-proportional 1-sigma; 0 -> default (3%),"
                                " covering the scale error left after `scale`"),
            "min_speed_mps": (0.0, "below this FILTERED speed a sample is"
                                   " skipped; 0 -> default"),
            "delay_ms": (round(speed_delay_ms) if has_speed else 0.0,
                          "median reported age of a sample at its timestamp"
                          if has_speed
                          else "how old a sample is at its timestamp"),
        }),
        ("score", {
            # The SAME lever arm as gnss.leverarm_frd above, and not
            # optional: ref.csv is the receiver's own solution, i.e. the
            # ANTENNA's position, while the filter reports the IMU's. Left
            # at the (0,0,0) default the harness compares two different
            # points on the vehicle and charges the whole lever arm to the
            # filter -- on a 0.5 m arm that is a 0.5 m floor under every
            # reported position error, an order of magnitude above what the
            # filter itself contributes on a good drive.
            "leverarm_frd": (list(leverarm_frd),
                              "project the estimate onto the antenna before"
                              " comparing: ref.csv IS the antenna"),
            "warmup_sec": 30,
            # replay.c refuses a config without it, and a bare epoch count
            # is also the cheapest guard against a truncated conversion.
            "min_epochs": 100,
            "ahrs": (0, "ref.csv attitude is a 0/0/0 placeholder, not a"
                        " real truth -- don't score against it"),
            "attitude": (0, "no attitude reference exists for this dataset"),
            # Deliberately loose: a limit of 0 would fail every first run and
            # teach nothing. Run once, read the observed numbers off the
            # output, then retune to ~2x them (the convention the
            # datasets/* datasets follow).
            "lim_pos_rms_m": (50.0, "FIXME: retune to ~2x a known-good run"),
            "lim_ellipsoid_rms_m": (50.0, "FIXME: retune to ~2x a known-good run"),
        }),
    ], header=("generated by inslib_convert_ubx_to_csv.py\n"
               + ("ref.csv is an RTKLIB post-processed solution (--pos):"
                  " carrier-phase against a base station, so it is independent"
                  " of the SPP fix in gnss.csv that aids the filter"
                  if from_pos else
                  "ref.csv is the GNSS solution again (MVP placeholder, not an"
                  " independent truth) -- see the script's module docstring")))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ubx", help=".ubx capture file (inslib_protocol.md)")
    ap.add_argument("--outdir", required=True, help="output directory")
    ap.add_argument("--name", default=None,
                    help="dataset name for config.yaml (default: outdir basename)")
    ap.add_argument("--automotive-mode", action=argparse.BooleanOptionalAction,
                    default=None,
                    help="automotive_mode (GNSS course-over-ground yaw) in the"
                         " generated config.yaml. Default: on when the capture"
                         " carries odometry (0x40/0x80) -- a wheel/OBD speed"
                         " source is itself evidence of a road vehicle -- off"
                         " otherwise. Pass --no-automotive-mode to force it off"
                         " even with odometry present")
    ap.add_argument("--leverarm-frd", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    metavar=("X", "Y", "Z"),
                    help="GNSS antenna lever arm, body FRD [m] (default 0 0 0)")
    ap.add_argument("--min-fix-type", type=int, default=3,
                    help="minimum NAV-PVT fixType accepted as a GNSS epoch"
                         " (default 3 = 3D fix)")
    ap.add_argument("--pos", metavar="FILE",
                    help="RTKLIB .pos solution to use as ref.csv instead of the"
                         " receiver's own fix. Requires 0x40/0x05 time sync frames"
                         " in the capture to tie GPS time to the MCU counter.")
    ap.add_argument("--pos-max-q", type=int, default=2,
                    help="worst RTKLIB quality accepted into ref.csv"
                         " (1 fix, 2 float, 4 dgps, 5 single; default 2)")
    args = ap.parse_args()

    name = args.name or os.path.basename(os.path.normpath(args.outdir))
    pos_rows = None
    if args.pos:
        pos_rows = read_pos(args.pos)
        if not pos_rows:
            sys.exit("no solution rows parsed from %s" % args.pos)
        qhist = Counter(r.q for r in pos_rows)
        print("%s: %d epochs (%s)"
              % (os.path.basename(args.pos), len(pos_rows),
                 ", ".join("%s %d" % (POS_Q_TEXT.get(q, "Q%d" % q), c)
                           for q, c in sorted(qhist.items()))))

    n, status_last, identities = convert(
        args.ubx, args.outdir, min_fix_type=args.min_fix_type,
        pos_rows=pos_rows, pos_max_q=args.pos_max_q)
    cfg_path = os.path.join(args.outdir, "config.yaml")
    # Never clobber an existing config: re-running the conversion on a new
    # capture is routine, throwing away a hand-tuned config.yaml is not.
    cfg_kept = os.path.exists(cfg_path)
    speed_scale = None
    automotive_mode = args.automotive_mode
    automotive_mode_reason = None
    if not cfg_kept:
        # Needs speed.csv/gnss.csv already on disk (just written by
        # convert() above) and at least one usable GNSS velocity fix.
        if n["speed"] > 0 and n["gnss"] > 0:
            speed_scale = calibrate_speed_scale(args.outdir)
        else:
            speed_scale = SPEED_SCALE_FIXME
        # A wheel/OBD speed source is itself evidence of a road vehicle, so
        # --automotive-mode/--no-automotive-mode left unset defaults to
        # whichever this capture is. Only when it is genuinely unset: an
        # explicit choice, either way, is never second-guessed.
        if automotive_mode is None:
            automotive_mode = n["speed"] > 0
            automotive_mode_reason = ("auto-enabled: odometry present"
                                      if automotive_mode else
                                      "auto-disabled: no odometry in this capture")
        write_cfg(cfg_path, name,
                  automotive_mode, args.leverarm_frd,
                  from_pos=pos_rows is not None, has_mag=n["mag"] > 0,
                  wmm_year=n["wmm_year"], has_speed=n["speed"] > 0,
                  speed_stddev_mps=n["speed_stddev_mps"],
                  speed_delay_ms=n["speed_delay_ms"],
                  speed_scale=speed_scale,
                  automotive_mode_reason=automotive_mode_reason)

    if pos_rows is None:
        print("imu.csv: %d samples, baro.csv: %d samples, gnss.csv/ref.csv: %d epochs"
              % (n["imu"], n["baro"], n["gnss"]))
        if n["ref_placeholder"]:
            print("ref.csv: no GNSS fix in this capture -- wrote one 0/0/0"
                  " placeholder row so the dataset still loads (aiding: none /"
                  " free_inertial_start is the only sound way to use it)")
    else:
        print("imu.csv: %d samples, baro.csv: %d samples, gnss.csv: %d epochs"
              % (n["imu"], n["baro"], n["gnss"]))
    if n["imu_legacy_status"]:
        print("WARNING: %d of %d IMU frame(s) carry an f32 die temperature where"
              " the status word belongs -- this capture predates it."
              " imu.csv's temperature column is 0.0 throughout and the"
              " saturation/calibration flags are meaningless. Accel, gyro and"
              " timestamps are unaffected. Re-record with current firmware to"
              " get the temperature."
              % (n["imu_legacy_status"], n["imu"]))
    if pos_rows is not None:
        print("clock:   %d time sync pairs fitted, MCU %+.1f ppm vs GPS,"
              " fit residual %.1f us RMS"
              % (n["clock_pairs"], n["clock_ppm"], n["clock_resid_us"]))
        dropped = [(n["clock_no_gps"], "without a GPS time"),
                   (n["clock_suspect"], "flagged suspect"),
                   (n["clock_rejected"], "rejected as slipped")]
        dropped = [(c, why) for c, why in dropped if c]
        if dropped:
            print("         %d pulse(s) not fitted: %s"
                  % (sum(c for c, _ in dropped),
                     ", ".join("%d %s" % (c, why) for c, why in dropped)))
        print("         receiver timebase %s, leap seconds %s"
              % ("UTC (corrected to GPST)" if n["clock_utc_base"] else "GPS",
                 n["leap_seconds"] if n["clock_utc_base"] else "n/a"))
        print("ref.csv: %d epochs from %s"
              % (n["ref_from_pos"], os.path.basename(args.pos)))
        if n["pos_rejected_q"]:
            print("         %d dropped, quality worse than Q=%d"
                  % (n["pos_rejected_q"], args.pos_max_q))
        if n["pos_outside_clock_span"]:
            print("         %d dropped, outside the pulse span (would need"
                  " extrapolation)" % n["pos_outside_clock_span"])
        if n["pos_no_velocity"]:
            print("         %d row(s) had no velocity columns -> written as 0"
                  % n["pos_no_velocity"])
    if n["mag"]:
        print("mag.csv: %d samples (mag: enable: 1, but its calibration keys"
              " are still a no-op until tools/inslib_calib_gui.py measures"
              " them)" % n["mag"])
        if not n["wmm_year"]:
            print("warning: no NAV-PVT with a valid date in the capture, so"
                  " mag: wmm_year is 0 -- fill it in by hand or the replay"
                  " builds no reference field and rejects every magnetometer"
                  " sample")
    if n["nav_pvt_rejected"]:
        print("note: %d NAV-PVT epoch(s) dropped, fixType < %d"
              % (n["nav_pvt_rejected"], args.min_fix_type))
    if n["gnss_skipped_no_imu_yet"]:
        print("warning: %d GNSS epoch(s) dropped, no IMU sample seen yet to"
              " timestamp them with" % n["gnss_skipped_no_imu_yet"])
    if n["parse_errors"]:
        print("pyubx2: %d frame(s) undecodable, skipped" % n["parse_errors"])
    if n["skipped_bytes"]:
        print("framer: %d byte(s) skipped to resynchronise (damaged stream?)"
              % n["skipped_bytes"])
    if n["speed"]:
        print("speed.csv: %d of %d odometry frame(s) (0x40/0x80), speed:"
              " enable: 1, stddev_mps %.3f / delay_ms %d from the medians"
              " the producer reported"
              % (n["speed"], n["odometry"], n["speed_stddev_mps"],
                 round(n["speed_delay_ms"])))
        if speed_scale is not None:
            if speed_scale[0]:
                print("         scale %.4f, %s" % speed_scale)
            else:
                print("         scale left at 0 (1.0): %s" % speed_scale[1])
        if automotive_mode_reason:
            print("         automotive_mode %s (%s)"
                  % ("on" if automotive_mode else "off", automotive_mode_reason))
        if n["speed_reverse"]:
            print("         %d sample(s) flagged reverse -- fused as |v|"
                  " either way, but a filter in automotive_mode derives"
                  " heading from course, which is 180 deg off there"
                  % n["speed_reverse"])
        if n["odometry_t_degraded"]:
            print("         %d sample(s) carry T_DEGRADED: the host/MCU"
                  " mapping behind their timestamp was stale or thin"
                  % n["odometry_t_degraded"])
    dropped_odo = [(n["odometry_no_t_us"], "never stamped with an MCU t_us by"
                    " the hub (nothing to place them on)"),
                   (n["odometry_other_kind"], "not a ground speed (kind != 0)"),
                   (n["odometry_bad_value"], "speed not finite or negative"),
                   (n["odometry_bad_len"], "wrong payload length")]
    dropped_odo = [(c, why) for c, why in dropped_odo if c]
    for c, why in dropped_odo:
        print("note: %d odometry frame(s) dropped, %s" % (c, why))
    if n["odometry"] and not n["speed"]:
        print("note: %d odometry frame(s) (0x40/0x80) in the capture, none"
              " convertible -- no speed.csv written" % n["odometry"])
    if status_last is not None:
        print("last 0x40/0x04 status payload (%d bytes): %s"
              % (len(status_last), status_last.hex()))
    other = {k: v for k, v in identities.items() if k not in ("NAV-PVT", "NAV-COV")}
    if other:
        print("other passthrough identities seen: "
              + ", ".join(f"{k}={v}" for k, v in sorted(other.items())))
    if cfg_kept:
        print("config.yaml already exists, kept as-is (delete it to"
              " regenerate)")
        # An older config, or one a calibration tool regenerated, can be
        # missing the epoch entirely, which costs the whole magnetometer.
        if n["mag"] and n["wmm_year"]:
            with open(cfg_path, encoding="utf-8") as f:
                kept = f.read()
            if "wmm_year" not in kept:
                print("         it has no mag: wmm_year, add"
                      " \"wmm_year: %.1f\" under `mag:` to let the replay"
                      " fuse the magnetometer" % n["wmm_year"])
    else:
        print("config.yaml written")
    if n["imu"] == 0:
        sys.exit("error: no IMU samples decoded, is this a valid .ubx capture?")


if __name__ == "__main__":
    main()
