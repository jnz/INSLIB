#!/usr/bin/env python3
"""
inslib hub - the one process that owns the sensor board's serial port.

A serial port has exactly one owner. That single fact shapes the whole
tool landscape: a receiver that opens the device makes recording the same
session impossible, and two recorders cannot coexist at all. So this tool
owns the device and everything else talks to it over UDP.

It does four things:

  1. Forwards UDP (corrections from str2str/strsvr, RTCM3 or SPARTN) to
     the serial link, byte for byte.
  2. Captures the return channel to a .ubx file, verbatim, before
     anything is parsed.
  3. Fans the return channel out over UDP, so tools/insrcv.c (or
     anything else) can process the session live while it is recorded.
  4. Accepts odometry on a second UDP port, completes it with the MCU
     timebase and injects it into both the capture and the fan-out.

Which of the four you get is a matter of flags and of what else you
start:

  python inslib_hub.py COM4                     record only
  python inslib_hub.py COM4                     ... plus corrections in,
                                                if str2str sends to :29797
  python inslib_hub.py COM4                     ... plus live processing,
    + insrcv --udp-port 29800                   if insrcv is running
  python inslib_hub.py COM4 --no-capture        live only, nothing written

The MCU firmware forwards incoming bytes on its serial link 1:1 to the
receiver, which makes the transport format-agnostic: RTCM3 from a base
station or NTRIP caster and SPARTN from a PPP-RTK service such as u-blox
PointPerfect travel the same path. The matching input protocol must be
enabled beforehand (CFG-*INPROT-RTCM3X resp. CFG-*INPROT-SPARTN, see
python/f9p_config.py and python/x20p_config.py). str2str/strsvr then send
with e.g. "-out udp://localhost:PORT" instead of directly to a COM port.

All UDP->serial bytes are additionally captured 1:1 to a file (the raw
correction stream, whatever format it is in). The stream is also parsed
into RTCM3 frames (preamble 0xD3, CRC24Q), for message 1005/1006 (base
station ARP) the station ID + ECEF position + derived lat/lon/height are
printed, in case you later want to know where the base station stood
(baseline length etc.). A SPARTN stream yields no frames here and is
simply passed through unexamined: its corrections are area based and
carry no base station to report.

The return channel is also decoded live for a status line every second:
IMU/baro/magnetometer packet rate (with the magnetometer's own
uncalibrated field magnitude, so a dead or miswired sensor is visible
without needing insrcv/PlotJuggler running), the raw-observable rates,
the age of the last correction frame (a rate would blink: an SSR service
publishes in bursts), the latest NAV-PVT fix type with satellite count,
RTK carrier solution and speed accuracy (sAcc), the return-channel
throughput and the odometry speed. It is
kept short enough to stay on one line, because a status line that wraps
is one nobody reads: everything that does not change from second to
second is printed once instead. That includes which of the u-blox's
correction inputs are actually enabled, per port and per protocol
(CFG-*INPROT-RTCM3X and CFG-*INPROT-SPARTN, polled via CFG-VALGET; a
"no answer" here usually means the poll never reached the GNSS receiver
through the MCU passthrough, not that the keys are disabled).

A post-processing (PPK) session needs more than the capture itself, so the
tool also reports whether the inputs for it are actually arriving and
writes a clock cross-reference:

  - RXM-RAWX / RXM-SFRBX rates in the status line, plus a one-shot verdict
    a few seconds in. Without raw observables and broadcast ephemeris the
    capture cannot be post-processed at all, and that is worth knowing on
    the driveway rather than after the drive.
  - the RTCM3 message types seen, so a correction stream that carries
    observations but no ephemeris (or no base ARP) is visible as such.
    Empty for SPARTN, which is not framed this way.
  - timesync.csv: once a minute, the MCU timer, the host clock and GPS
    time side by side. Three clocks are in play and only their pairing
    makes host-timestamped side channels alignable with the IMU stream
    afterwards.
  - speed.csv: the vehicle speed arriving on a SECOND UDP port as UBX
    0x40/0x80 (inslib_obd_speed.py), with both the host clock the producer
    stated and the MCU time derived from it. Never forwarded to the serial
    link: those frames have no business reaching the GNSS receiver. They
    are instead completed with t_us and injected into the capture and the
    fan-out (see inslib_protocol.md), so a consumer sees one stream on one
    clock and the odometry survives into post-processing rather than
    living in a side file that has to be merged by timestamp afterwards.

A dropped serial link does not end a recording. Reopening is retried fast
for the first seconds and then indefinitely at a calmer rate, while the
UDP side, the captures and the speed log keep running: a loose USB
connector costs the IMU stream for the duration of the outage and nothing
else. Each reconnect starts a new "segment", which both CSVs carry, and a
device that restarted during the gap is detected as such: t_us starts over
from zero, which ends the old timeline rather than continuing it.

The capture files default to timestamped names
(inslib_YYYYmmdd_HHMMSS.corr / .ubx / _timesync.csv / _speed.csv), one
set per run. Explicit names are still possible, but then a second run
overwrites the first: a recording is expensive to repeat and disk is not.

Ctrl-C stops. Requires pyserial and pyubx2 (see python/requirements.txt).
"""
import argparse, contextlib, os, sys, socket, struct, time, math, csv, collections
from datetime import datetime, timezone
import serial   # pip install pyserial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inslib_ubx import (BARO_LEN, ID_BARO, ID_IMU, ID_MAG, ID_TIMESYNC,
                        IMU_LEN, MAG_FMT, MAG_LEN, ODO_T_DEGRADED,
                        ODO_T_US_VALID, TimerRestartWatch, UbxFramer,
                        build_odometry, datagrams, parse_odometry_datagram,
                        parse_timesync)

try:
    from pyubx2 import POLL_LAYER_RAM, UBXMessage, UBXReader
except ImportError:
    sys.exit("pyubx2 is required: pip install pyubx2 (see python/requirements.txt)")


def _parse_args():
    ap = argparse.ArgumentParser(
        description="Own the sensor board's serial port: corrections in, "
                    "capture out, UDP fan-out, odometry injection.")
    ap.add_argument("port", help="serial device (COM4, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate (default %(default)s; USB-CDC ignores it, "
                         "an USB UART adapter on UART2 may want 460800)")
    ap.add_argument("--bind", default="0.0.0.0",
                    help="address the UDP inputs bind to (default %(default)s)")
    ap.add_argument("--rtcm-port", type=int, default=29797,
                    help="UDP port for corrections (RTCM3 or SPARTN), "
                         "forwarded to the serial link (default %(default)s)")
    ap.add_argument("--speed-port", type=int, default=29798,
                    help="UDP port for odometry (UBX 0x40/0x80), matching "
                         "inslib_obd_speed.py's default (default %(default)s)")
    ap.add_argument("--fanout", default="127.0.0.1:29800",
                    help="HOST:PORT the return channel is republished to, "
                         "comma-separated for several (default %(default)s); "
                         "an empty string disables the fan-out")
    ap.add_argument("--no-capture", action="store_true",
                    help="write no capture and no CSV; live fan-out only")
    ap.add_argument("--rtcm-log", help="correction capture path (default timestamped)")
    ap.add_argument("--ubx-log", help="return-channel capture path (default timestamped)")
    return ap.parse_args()


def _parse_fanout(text):
    """'host:port,host:port' -> [(host, port)]. A bare port means localhost."""
    out = []
    for item in (t.strip() for t in text.split(",")):
        if not item:
            continue
        if ":" in item:
            host, _, p = item.rpartition(":")
            out.append((host, int(p)))
        else:
            out.append(("127.0.0.1", int(item)))
    return out


args       = _parse_args()
_stamp     = datetime.now().strftime("%Y%m%d_%H%M%S")
port       = args.port
baud       = args.baud
bind_addr  = args.bind
udp_port   = args.rtcm_port
speed_port = args.speed_port
capture_on = not args.no_capture
fanout_dests = _parse_fanout(args.fanout)
rtcm_log   = args.rtcm_log or f"inslib_{_stamp}.corr"
ubx_log    = args.ubx_log or f"inslib_{_stamp}.ubx"
timesync_log = f"inslib_{_stamp}_timesync.csv"
speed_log = f"inslib_{_stamp}_speed.csv"

# --- Odometry side channel (UBX 0x40/0x80, see inslib_protocol.md) ----------
# A SECOND UDP port, deliberately not the correction port: everything that
# arrives on udp_port is forwarded to the serial link byte for byte, and an
# odometry frame has no business reaching the GNSS receiver. Frames here
# are parsed, logged, completed with t_us and re-emitted into the sensor
# stream, never forwarded to serial. The frame format and the framer live
# in inslib_ubx.py, shared with every other tool here.


# --- Odometry mapping quality -----------------------------------------------
# Below this many pairs the offset window has not plausibly seen a
# least-delayed packet yet, so its maximum is still climbing.
ODO_MIN_OFFSETS = 200
# An offset window whose IMU stream stopped this long ago describes a link
# that is no longer there. Two crystals drift a few ppm apart, which is
# tens of ms over an hour: tolerable for a few seconds, not for minutes.
ODO_IMU_STALE_S = 5.0

# A magnetometer reading older than this must not still be shown on the
# status line: it would read as "arriving fine" for a sensor that has
# actually gone quiet, same trap the odometry speed avoids with
# ODO_STALE_S below.
MAG_STALE_S = 3.0


class Fanout:
    """Republishes the return channel to zero or more UDP consumers.

    Frames, not bytes: a datagram carries whole UBX frames only
    (inslib_protocol.md), so a lost datagram costs exactly the frames
    inside it instead of also desynchronising the frame that straddled the
    boundary. That is the one thing UDP gives away for free and it costs
    nothing to keep, since the frames are already parsed for the status
    line.

    Best effort throughout. A consumer that is not running, is slow, or
    went away must never slow down or stop the recording, so a failed send
    is counted and dropped rather than raised."""

    def __init__(self, dests):
        self.dests = list(dests)
        self.sock = None
        self.n_datagrams = 0
        self.n_frames = 0
        self.n_errors = 0
        if self.dests:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @property
    def enabled(self):
        return self.sock is not None

    def send(self, frames):
        if not self.enabled or not frames:
            return
        self.n_frames += len(frames)
        for dgram in datagrams(frames):
            for dest in self.dests:
                try:
                    self.sock.sendto(dgram, dest)
                except OSError:
                    # On Windows an ICMP port-unreachable from a consumer
                    # that is not listening surfaces on a LATER send as
                    # WSAECONNRESET. It is not this socket's problem.
                    self.n_errors += 1
            self.n_datagrams += 1

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


# --- PPK input bookkeeping ---------------------------------------------------
# RTCM3 message groups that matter for post-processing. A correction stream
# with observations but no ephemeris still fixes nothing on its own, and one
# without the base ARP has no absolute reference to fix TO.
RTCM_EPH = {1019, 1020, 1041, 1042, 1044, 1045, 1046}
RTCM_ARP = {1005, 1006}


def rtcm_group(msg_num):
    if msg_num in RTCM_ARP:
        return "ARP"
    if msg_num in RTCM_EPH:
        return "eph"
    if 1071 <= msg_num <= 1230:
        return "obs"
    return "other"


# --- RTCM3 helpers (preamble 0xD3, 10-bit length, CRC24Q) -----------------
def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc

def rtcm3_frame(buf, j):
    """(total_len, msg_num, payload) for the CRC-checked frame at buf[j],
       "short" while it is still incomplete, None if the CRC rejects it."""
    if len(buf) < j + 3:
        return "short"
    length = ((buf[j + 1] & 0x03) << 8) | buf[j + 2]
    total = 3 + length + 3
    if len(buf) < j + total:
        return "short"
    frame = bytes(buf[j:j + total])
    if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
        return None
    payload = frame[3:3 + length]
    if len(payload) < 2:            # too short to carry a message number
        return None
    return total, (payload[0] << 4) | (payload[1] >> 4), payload


# --- SPARTN helpers (preamble 0x73, SPARTN 2.0) ---------------------------
# What a PPP-RTK service such as u-blox PointPerfect sends instead of RTCM3.
# There is no CRC here that is cheap to lean on the way RTCM3 has CRC24Q:
# the frame's own CRC is 4 bits wide with parameters not worth guessing at,
# and a wrong guess would reject every frame, which reads exactly like
# nothing arriving. So a frame is accepted once the byte behind it is the
# next preamble, which confirms the length its header claimed. That costs
# the last frame of a burst, held until the next one arrives, and nothing
# else.
SPARTN_TYPES  = {0: "OCB", 1: "HPAC", 2: "GAD", 3: "BPAC", 4: "EAS"}
SPARTN_CONST  = {0: "GPS", 1: "GLO", 2: "GAL", 3: "BDS", 4: "QZSS"}
SPARTN_CRCLEN = {0: 1, 1: 2, 2: 3, 3: 4}

def spartn_frame(buf, j):
    """(total_len, name, encrypted) for the frame at buf[j], "short" while
       the header or the chaining byte is missing, None if it is no frame."""
    if len(buf) < j + 5:
        return "short"
    b0, b1, b2, b4 = buf[j + 1], buf[j + 2], buf[j + 3], buf[j + 4]
    mtype   = b0 >> 1                            # TF002
    plen    = ((b0 & 0x01) << 9) | (b1 << 1) | (b2 >> 7)   # TF003
    enc     = (b2 >> 6) & 1                      # TF004 EAF
    crclen  = SPARTN_CRCLEN[(b2 >> 4) & 0x03]    # TF005
    # TF007 subtype in the high nibble, TF008 time tag type right below it:
    # a long time tag adds two bytes, encryption another two.
    varhdr  = (6 if (b4 >> 3) & 1 else 4) + (2 if enc else 0)
    total   = 4 + varhdr + plen + crclen
    if len(buf) < j + total + 1:                 # +1 for the chaining byte
        return "short"
    if buf[j + total] not in (0x73, 0xD3):
        return None
    name = SPARTN_TYPES.get(mtype, f"type{mtype}")
    if mtype in (0, 1):                          # OCB/HPAC are per constellation
        name += "-" + SPARTN_CONST.get(b4 >> 4, f"sub{b4 >> 4}")
    return total, name, enc


def corr_age_text(last_t, now):
    """Seconds since the last correction frame, for the status line.

       A rate would be the obvious choice and is the wrong one: an SSR
       service publishes in bursts, PointPerfect sends orbit/clock every 5 s
       and atmosphere every 30 s, so a per-second frame rate sits at zero
       most of the time and blinks. Age carries the same information without
       the blinking, and it is what actually decides whether the fix still
       has support: corrections that stop arriving age visibly."""
    if last_t is None:
        return "--"
    age = now - last_t
    return f"{age:.1f}s" if age < 99.5 else ">99s"


def corr_verdict(n_bytes, rtcm_types, spartn_types):
    """One line on whether the correction link delivers something the
       receiver can actually use.

       Bytes arriving is not the same as frames parsing, and the byte
       counter cannot tell the two apart: a wrong mount point, a stream
       that is silently truncated somewhere on the way, and a service
       sending a format nobody asked for all look like traffic. They
       differ here, where the framing either holds or does not."""
    if not n_bytes:
        return ("corrections: nothing arrived, the link is idle "
                "(NTRIP client running and pointed at this port?)")
    if not rtcm_types and not spartn_types:
        return (f"corrections: {n_bytes} bytes arrived but none of it framed "
                f"as RTCM3 or SPARTN")
    seen = []
    if rtcm_types:
        kinds = sorted({rtcm_group(n) for n in rtcm_types})
        seen.append("RTCM3 (" + ", ".join(kinds) + ")")
    if spartn_types:
        kinds = sorted({n.split("-")[0] for n in spartn_types})
        seen.append("SPARTN (" + ", ".join(kinds) + ")")
    return "corrections: valid " + " and ".join(seen) + " arriving"


def corr_frames(buf):
    """Splits the correction stream into frames, RTCM3 and SPARTN alike.
       Nothing tells the hub which service is feeding it, and a counter that
       knows only one of the two formats reads as a flat zero for the other,
       which is indistinguishable from a stream that never arrived.

       Returns (rtcm, spartn) as lists of (msg_num, payload) resp.
       (name, encrypted). buf is shortened in place by what was consumed."""
    rtcm, spartn = [], []
    i = 0
    while i < len(buf):
        head = buf[i]
        if head == 0xD3:
            got = rtcm3_frame(buf, i)
        elif head == 0x73:
            got = spartn_frame(buf, i)
        else:
            i += 1                  # neither preamble, resync one byte on
            continue
        if got == "short":
            break                   # incomplete: keep it, wait for more
        if got is None:
            i += 1                  # false preamble, resync one byte on
            continue
        if head == 0xD3:
            total, msg_num, payload = got
            rtcm.append((msg_num, payload))
        else:
            total, name, enc = got
            spartn.append((name, enc))
        i += total
    del buf[:i]
    return rtcm, spartn

class BitReader:
    def __init__(self, data):
        self.data, self.pos = data, 0
    def u(self, n):
        v = 0
        for _ in range(n):
            v = (v << 1) | ((self.data[self.pos >> 3] >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return v
    def s(self, n):
        v = self.u(n)
        return v - (1 << n) if v & (1 << (n - 1)) else v

def ecef2llh(x, y, z):
    """WGS84 ECEF -> geodetic lat/lon (deg) / height (m), Bowring iteration."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    p = math.hypot(x, y)
    lon = math.atan2(y, x)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(5):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h

def parse_base_station(payload):
    """1005/1006 -> (stn_id, x, y, z, height_or_None) in meters, else None."""
    br = BitReader(payload)
    msg_num = br.u(12)
    if msg_num not in (1005, 1006):
        return None
    stn_id = br.u(12)
    br.u(6)                            # ITRF realization year
    br.u(1); br.u(1); br.u(1); br.u(1)  # GPS/GLONASS/Galileo/ref-station indicator
    x = br.s(38) * 0.0001
    br.u(1); br.u(1)                    # single-Rx oscillator, reserved
    y = br.s(38) * 0.0001
    br.u(2)                            # quarter-cycle indicator
    z = br.s(38) * 0.0001
    height = br.u(16) * 0.0001 if msg_num == 1006 else None
    return stn_id, x, y, z, height


# --- return-channel status: IMU/baro/NAV-PVT/CFG-VALGET (pyubx2) ----------
# UbxFramer above does the framing/resync/checksum on the return channel
# and pyubx2 only decodes the individual standard u-blox frames handed to
# it; the firmware's own class-0x40 payloads have no pyubx2 definition and
# are decoded locally below (same format as tools/inslib_protocol.md).

FIX_TYPE_TEXT = {0: "NoFix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}
CARR_SOLN_TEXT = {0: "-", 1: "Float", 2: "Fixed"}
# Fix types that come with a velocity solution, and therefore with a
# meaningful sAcc: everything except no fix at all and a time-only fix.
FIX_TYPES_WITH_VELOCITY = (1, 2, 3, 4)
# Correction input protocols, polled together in ONE CFG-VALGET. RTCM3X
# comes from a base station or NTRIP caster, SPARTN from PPP-RTK services
# such as u-blox PointPerfect. Both can be enabled at the same time, so the
# check has to cover both: a correction stream that arrives but is dropped
# by the receiver looks exactly like one that never arrived.
CORR_PORTS  = ["UART1", "UART2", "USB"]
CORR_PROTOS = ["RTCM3X", "SPARTN"]
CORR_KEYS   = [f"CFG_{port}INPROT_{proto}"
               for port in CORR_PORTS for proto in CORR_PROTOS]
CORR_POLL_INTERVAL_S = 5.0    # retry until we get an answer

class RxStatus:
    """Tallies the return-channel status printed once a second: IMU/baro
    packet counts, the latest NAV-PVT fix, the CFG-VALGET correction-input
    check. Rates are computed by the caller from the wall-clock interval,
    same as the existing TX/RX KB/s line."""

    def __init__(self):
        self.imu_n = 0
        self.baro_n = 0
        self.mag_n = 0                 # 0x40/0x06 samples this interval
        self.mag_total = 0             # whole session, for the closing report
        self.last_mag_ut = None        # (x, y, z), last decoded sample
        self.last_mag_wall = None      # host clock at that sample
        self.rawx_n = 0               # RXM-RAWX, the PPK observables
        self.sfrbx_n = 0              # RXM-SFRBX, the broadcast ephemeris
        self.rawx_total = 0
        self.sfrbx_total = 0
        self.last_pvt = None          # (fixType, carrSoln, numSV, sAcc [m/s])
        self.corr_in = None           # dict key -> 0/1, once CFG-VALGET answers
        self.corr_poll_sent_t = None

        # Clock cross-reference (see inslib_protocol.md "Mapping t_us to
        # GPS time"). The firmware pairs the hardware pulse capture with
        # the TIM-TP that announced it and sends the result as 0x40/0x05,
        # so one exact (t_us, GPS time) pair arrives per second and the
        # host correlates nothing.
        self.imu_clock = TimerRestartWatch()
        self.imu_t_us = None          # last IMU sample, MCU timebase
        self.tp_pair = None           # (t_us, week, tow_s) of the last valid pulse
        self.tp_edges_n = 0           # every captured edge, valid or not
        self.tp_pairs_n = 0           # those carrying a GPS time
        self.tp_suspect_n = 0
        self.saw_tim_tp = False       # old firmware still forwards it

        # Host clock -> MCU timer, for stamping host-timestamped side
        # channels with an MCU time (REQ: the speed CSV below).
        #
        # Each IMU packet gives a pair (arrival on the host, timer value at
        # sampling). Arrival is late by a transport latency that varies but
        # has a floor, so offset = t_us - host_us is CONST - latency and the
        # cleanest estimate of the constant is the MAXIMUM over a window,
        # not the mean: the least-delayed packet is the most honest one.
        #
        # A single offset ignores the crystal drift between the two clocks
        # (tens of ms over an hour at a few ppm), which is why this is a
        # convenience index and not the authority. The raw host timestamp
        # stays in the CSV so the proper fit can be done offline from
        # timesync.csv, over the whole session and with bad pairs rejected.
        self.host_offsets = collections.deque(maxlen=2000)
        self.last_imu_wall = None     # host clock at the last IMU packet
        self.segment = 0              # bumped on every serial reconnect

    def on_reconnect(self):
        """The link came back. What survives and what does not.

        The offset window is dropped: it describes a transport that has
        just been re-established, and stale offsets would stamp the first
        samples after the gap with the old link's latency. Whether the
        DEVICE also restarted is a separate question, answered by the data
        (a backward step in t_us) rather than by the reconnect - a USB
        cable that came loose does not reset the MCU."""
        self.host_offsets.clear()
        self.last_imu_wall = None
        self.segment += 1

    def host_to_imu_t_us(self, host_unix_us):
        if not self.host_offsets:
            return None
        return int(host_unix_us + max(self.host_offsets))

    def odo_time_flags(self):
        """(mappable, flags) for an odometry frame stamped right now.

        The mapping is only as good as the window it rests on. Two things
        spoil it and both are visible from here: too few pairs to have
        found the least-delayed packet yet, and a window that describes a
        link which has since gone quiet (the offsets are then minutes old
        while the crystals have kept drifting apart). Neither is a reason
        to refuse the sample, but a consumer has to be able to tell a
        well-founded timestamp from a shaky one, which is what
        ODO_T_DEGRADED says."""
        if not self.host_offsets:
            return False, 0
        stale = (self.last_imu_wall is None
                 or time.time() - self.last_imu_wall > ODO_IMU_STALE_S)
        thin = len(self.host_offsets) < ODO_MIN_OFFSETS
        return True, (ODO_T_DEGRADED if (stale or thin) else 0)

    def handle_inslib(self, mid, payload, host_unix_us=None):
        """Firmware messages (class 0x40). Decoded here rather than through
        pyubx2, which has no definition for this class anyway."""
        if mid == ID_IMU and len(payload) == IMU_LEN:
            self.imu_n += 1
            self.imu_t_us = struct.unpack("<Q", payload[:8])[0]
            if self.imu_clock.feed(self.imu_t_us):
                # A new timeline: every offset in the window maps the host
                # clock onto the timeline that just ended, so keeping them
                # would stamp the next odometry sample with the previous
                # uptime.
                self.host_offsets.clear()
            if host_unix_us is not None:
                self.host_offsets.append(self.imu_t_us - host_unix_us)
                self.last_imu_wall = time.time()
        elif mid == ID_BARO and len(payload) == BARO_LEN:
            self.baro_n += 1
        elif mid == ID_MAG and len(payload) == MAG_LEN:
            self.mag_n += 1
            self.mag_total += 1
            _t_us, mx, my, mz, _temp_c = struct.unpack(MAG_FMT, payload)
            self.last_mag_ut = (mx, my, mz)
            self.last_mag_wall = time.time()
        elif mid == ID_TIMESYNC:
            ts = parse_timesync(payload)
            if ts is None:
                return
            self.tp_edges_n += 1
            if ts["suspect"]:
                self.tp_suspect_n += 1
            # An edge without an announcement still proves the pulse is
            # alive, but it is not a clock pair and must not be shown as
            # one.
            if ts["gps_valid"]:
                self.tp_pair = (ts["t_us"], ts["week"], ts["tow_s"])
                self.tp_pairs_n += 1

    def handle_ubx(self, msg):
        """Standard u-blox messages, already parsed by pyubx2 from ONE
        complete, checksum-verified frame."""
        if msg.identity == "TIM-TP":
            # Not expected any more: the firmware consumes TIM-TP to build
            # 0x40/0x05 and does not forward it (inslib_protocol.md "GNSS
            # passthrough content"). Seeing it means old firmware, and
            # then no 0x40/0x05 arrives either - which otherwise looks
            # exactly like a receiver without a time lock.
            self.saw_tim_tp = True
        elif msg.identity == "RXM-RAWX":
            self.rawx_n += 1
            self.rawx_total += 1
        elif msg.identity == "RXM-SFRBX":
            self.sfrbx_n += 1
            self.sfrbx_total += 1
        elif msg.identity == "NAV-PVT":
            self.last_pvt = (getattr(msg, "fixType", 0),
                             getattr(msg, "carrSoln", 0),
                             getattr(msg, "numSV", 0),
                             getattr(msg, "sAcc", 0) / 1000.0)
        elif msg.identity == "CFG-VALGET" and self.corr_in is None:
            if all(hasattr(msg, k) for k in CORR_KEYS):
                self.corr_in = {k: getattr(msg, k) for k in CORR_KEYS}

    def poll_corr_in(self, ser, now):
        """(Re-)send the CFG-VALGET poll if we still have no answer and
        the retry interval elapsed. A "no answer" after several retries
        usually means the poll never reached the receiver through the MCU
        passthrough (see f9p_config.py's connect(): that path is a one-
        way street for some MCU firmware versions), not that the
        correction inputs are actually disabled.

        Older firmware may not know the SPARTN keys at all and then answers
        with the RTCM3X ones only. That answer is discarded above (all keys
        or nothing), so the report degrades to "no answer" rather than to a
        half-truth about which protocols are live."""
        if self.corr_in is not None:
            return
        if (self.corr_poll_sent_t is not None
                and now - self.corr_poll_sent_t < CORR_POLL_INTERVAL_S):
            return
        ser.write(UBXMessage.config_poll(POLL_LAYER_RAM, 0, CORR_KEYS).serialize())
        self.corr_poll_sent_t = now

    def corr_in_text(self):
        """One field per port listing the protocols actually enabled on it,
        e.g. "corr-in UART1=RTCM3X+SPARTN UART2=off USB=SPARTN"."""
        if self.corr_in is None:
            return "corr-in ?" if self.corr_poll_sent_t is None else "corr-in no answer (passthrough?)"
        parts = []
        for port in CORR_PORTS:
            on = [p for p in CORR_PROTOS if self.corr_in[f"CFG_{port}INPROT_{p}"]]
            parts.append(f"{port}=" + ("+".join(on) if on else "off"))
        return "corr-in " + " ".join(parts)

    def pvt_text(self):
        """The fix for the status line, padded to a constant width so the
        fields behind it do not jump around as the fix changes.

        The speed accuracy (NAV-PVT sAcc, the 1-sigma the receiver claims
        for its velocity solution) is shown only from a fix that actually
        has a velocity solution: below that it is the receiver's "invalid"
        sentinel (999 m/s), which says nothing and reads as if it did."""
        if self.last_pvt is None:
            return "%-7s sv-- %-5s %-9s" % ("noPVT", "", "sa --")
        fix_type, carr_soln, num_sv, sacc_mps = self.last_pvt
        return ("%-7s sv%2d %-5s %-9s"
               % (FIX_TYPE_TEXT.get(fix_type, "?%d" % fix_type), num_sv,
                  CARR_SOLN_TEXT.get(carr_soln, "?%d" % carr_soln),
                  "sa%.2fm/s" % sacc_mps if fix_type in FIX_TYPES_WITH_VELOCITY
                  else "sa --"))

    def clock_flag(self):
        """One character for the time pulse, for the status line: T = a
        pulse was paired with a GPS time, p = pulses arrive but carry no
        time yet, - = no pulse at all."""
        if self.tp_pair is not None:
            return "T"
        return "p" if self.tp_edges_n else "-"

    def mag_text(self, now):
        """Field magnitude for the status line, or '--' once the last
        sample is stale (MAG_STALE_S) or none has ever arrived.

        Uncalibrated on purpose: this is an "is anything coming in at all,
        and is it a plausible field reading" check, not the calibrated
        value the filter fuses (tools/inslib_calib_gui.py owns that, and
        insrcv.c's console/PlotJuggler output shows the calibrated |m|
        once mag: calibration is in config.yaml). A raw MEMS magnetometer
        near Earth's surface reads on the order of tens of uT, so a
        number wildly outside that range still says "wiring/scale is
        wrong" even before any calibration exists."""
        if (self.last_mag_ut is None or self.last_mag_wall is None
                or now - self.last_mag_wall > MAG_STALE_S):
            return "--"
        mx, my, mz = self.last_mag_ut
        return "%.1fuT" % math.sqrt(mx * mx + my * my + mz * mz)

    def gps_time_text(self):
        """The GPS time itself, for the one-shot line when the pulse first
        pairs with one. The status line only carries clock_flag()."""
        if self.tp_pair is None:
            if self.tp_edges_n:
                # The pulse is alive but carries no announcement yet: the
                # receiver is pulsing without a usable time solution.
                return "GPS-time none (%d pulses)" % self.tp_edges_n
            return "GPS-time none"
        _t_us, week, tow_s = self.tp_pair
        return "GPS wk%d tow%.1f" % (week, tow_s)

    def ppk_verdict(self):
        """One-shot judgement on whether this capture will be
        post-processable, printed early enough to act on it.

        Both halves are needed and neither substitutes for the other:
        RXM-RAWX carries the pseudorange/carrier observables, RXM-SFRBX the
        broadcast navigation data. A capture missing either is a capture
        that cannot be turned into a PPK solution later, no matter how long
        the drive was."""
        have_rawx, have_sfrbx = self.rawx_total > 0, self.sfrbx_total > 0
        if have_rawx and have_sfrbx:
            return ("PPK inputs OK: RXM-RAWX and RXM-SFRBX are being logged", True)

        # SFRBX carries decoded subframes from TRACKED satellites, so with
        # none in view its absence says nothing about the configuration: it
        # is the expected reading indoors. Calling that a fault would train
        # the check to be ignored, which is worse than not having one.
        num_sv = self.last_pvt[2] if self.last_pvt else 0
        if have_rawx and not have_sfrbx and num_sv == 0:
            return ("PPK inputs: RXM-RAWX OK, RXM-SFRBX not seen - expected while 0"
                    " satellites are tracked."
                    "\n  Re-check with sky view: ephemeris only arrives with reception.",
                    True)

        missing = []
        if not have_rawx:
            missing.append("RXM-RAWX (raw observables)")
        if not have_sfrbx:
            missing.append("RXM-SFRBX (broadcast ephemeris)")
        return ("PPK inputs INCOMPLETE, missing " + " and ".join(missing)
                + (" while tracking %d satellites" % num_sv if num_sv else "")
                + "\n  This capture will not be post-processable. Enable the message(s)"
                  "\n  on the receiver before driving off"
                  "\n  (ephemeris can alternatively come from the base's RTCM 1019/1020/1042/1046).",
                False)


def write_timesync(writer, fh, status):
    """One row pairing the three clocks in play.

    Deliberately raw: the MCU counter, the host clock and the GPS time as
    observed, with no fitting and no interpolation. The fit belongs in
    post-processing, where the whole session is available and a bad pair
    can be rejected; a value already collapsed into a single number here
    could not be un-collapsed later."""
    if status.imu_t_us is None:
        return False
    week = tow = ""
    tp_t_us = ""
    if status.tp_pair is not None:
        tp_t_us, week, tow = status.tp_pair
        tow = "%.6f" % tow
    now = time.time()
    writer.writerow([datetime.now(timezone.utc).isoformat(), "%.6f" % now,
                     status.imu_t_us, tp_t_us, week, tow, status.tp_pairs_n,
                     status.segment, status.imu_clock.restarts])
    fh.flush()
    return True


# --- Serial link, with reconnect --------------------------------------------
# A loose USB connector must not end a recording. Reopening is attempted
# fast for the first seconds (a CDC device re-enumerates in well under a
# second, and a short IMU gap is survivable) and then indefinitely at a
# calmer rate: giving up after a fixed window is precisely the failure this
# exists to prevent, and retrying costs nothing while the drive continues.
# The UDP side, the capture files and the speed logging all keep running
# throughout, so an outage costs the IMU stream and nothing else.
RECONNECT_FAST_S = 5.0
RECONNECT_FAST_INTERVAL_S = 0.02
RECONNECT_SLOW_INTERVAL_S = 0.5
RECONNECT_PORTLIST_EVERY_S = 10.0


def open_serial():
    return serial.Serial(port, baud, timeout=1)


def open_serial_or_exit():
    """First open. A failure here is a user setup problem, not a link
    outage, so report it in one line instead of a stack trace."""
    try:
        return open_serial()
    except (serial.SerialException, OSError) as exc:
        try:
            from serial.tools import list_ports
            names = ", ".join(p.device for p in list_ports.comports()) or "none"
        except ImportError:
            names = "?"
        errno = getattr(exc, "errno", None)
        if errno == 13 or "PermissionError" in str(exc) or "Access is denied" in str(exc):
            hint = ("the port is already open in another program "
                    "(another hub instance, u-center, a terminal, a plotter)")
        elif errno == 2 or "FileNotFoundError" in str(exc) or "No such file" in str(exc):
            hint = "no such port. Ports present: %s" % names
        else:
            hint = "%s. Ports present: %s" % (exc, names)
        sys.exit("could not open %s at %d baud: %s" % (port, baud, hint))


def reconnect_serial(status):
    """Block until the port is back. Returns (ser, outage_s)."""
    t_lost = time.time()
    t_listed = 0.0
    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] serial link lost, reopening {port} ...")
    while True:
        waited = time.time() - t_lost
        try:
            new_ser = open_serial()
        except (serial.SerialException, OSError):
            # On Windows a re-enumerated device can come back under a
            # DIFFERENT COM number, in which case reopening this name never
            # succeeds. Show what is actually there so that is visible
            # rather than looking like a dead device.
            if waited - t_listed >= RECONNECT_PORTLIST_EVERY_S:
                t_listed = waited
                try:
                    from serial.tools import list_ports
                    names = ", ".join(p.device for p in list_ports.comports()) or "none"
                except ImportError:
                    names = "?"
                print(f"  still gone after {waited:.0f} s. Ports present: {names}"
                      f"  (a re-enumerated device may have changed name)")
            time.sleep(RECONNECT_FAST_INTERVAL_S if waited < RECONNECT_FAST_S
                       else RECONNECT_SLOW_INTERVAL_S)
            continue
        outage = time.time() - t_lost
        status.on_reconnect()
        print(f"  reopened after {outage:.2f} s (segment {status.segment})")
        return new_ser, outage


# --- Bridge -----------------------------------------------------------------
ser = open_serial_or_exit()
framer = UbxFramer()
rx_status = RxStatus()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((bind_addr, udp_port))
# This timeout is the whole loop's period whenever no RTCM3 arrives, and
# the loop is what forwards the return channel: everything the serial port
# buffered meanwhile goes out as ONE burst.
sock.settimeout(0.01)

speed_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
speed_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
speed_sock.bind((bind_addr, speed_port))
speed_sock.setblocking(False)   # drained inside the loop, never blocks it

fanout = Fanout(fanout_dests)

print(f"serial {port} @ {baud} baud  (Ctrl-C to stop)")
print(f"  corr in  : UDP {bind_addr}:{udp_port} -> serial")
print(f"  odometry : UDP {bind_addr}:{speed_port} -> 0x40/0x80 + MCU clock "
      f"(never forwarded to {port})")
if fanout.enabled:
    print("  fan-out  : serial -> UDP "
          + ", ".join(f"{h}:{p}" for h, p in fanout_dests))
else:
    print("  fan-out  : off (--fanout '')")
if capture_on:
    print(f"  capture  : {ubx_log}, {rtcm_log}, {timesync_log}, {speed_log}")
else:
    print("  capture  : off (--no-capture), nothing is written to disk")
# Legend once instead of labels every second: the status line below is
# overwritten in place and every character it spends on naming its own
# fields is a character closer to wrapping, which is what makes it
# unreadable.
print("  status   : rates in Hz (Mag@field = magnetometer rate and uncalibrated |m|,"
      " '--' once stale;\n"
      "             RAWX/SFRBX = raw observables/ephemeris,\n"
      "             CORR = age of the last correction frame, RTCM3 or SPARTN)\n"
      "             then fix/satellites/RTK/speed accuracy (sa, '--' without a\n"
      "             velocity solution), time pulse (T=GPS time, p=pulse only,"
      " -=none),\n"
      "             return-channel throughput and odometry speed")

n_speed, n_speed_bad = 0, 0
n_odo_tx, n_odo_unmapped, n_odo_degraded = 0, 0, 0
last_speed = None
last_speed_wall = None   # a speed that stopped arriving must not read as current
ODO_STALE_S = 3.0        # tolerant of a slow poll, not of a dead producer
n_outage, outage_total_s = 0, 0.0
n_parse_err = 0
first_parse_err = None      # reported once, see the decode guard below
n_pkt, n_tx, n_rx, t0, tp = 0, 0, 0, time.time(), time.time()
corr_total = 0          # valid correction frames, whole session
last_corr_t = None      # wall clock of the last one, for the age display
rtcm_types = collections.Counter()     # RTCM3 msg_num -> count, whole session
spartn_types = collections.Counter()   # SPARTN message name -> count
spartn_enc_n = 0                       # SPARTN frames with the EAF flag set
parse_buf = bytearray()
last_base = None

PPK_VERDICT_AFTER_S = 8.0   # long enough for a 1 Hz SFRBX to have shown up
ppk_reported = False
# Which correction inputs are enabled is a configuration fact, not a live
# value: it belongs in a line that is printed once, not in one that is
# repainted every second.
CORR_REPORT_AFTER_S = 3 * CORR_POLL_INTERVAL_S   # give the retries a chance
corr_reported = False
# A verdict on the stream itself, separate from the keys above: one says
# the receiver would accept corrections, the other that they are arriving.
# The wait covers a full SPARTN cycle (orbit/clock every 5 s, atmosphere
# and area definitions every 30), so the verdict names everything the
# service sends rather than only the fastest message. A link that delivers
# nothing at all is not worth sitting on that long.
CORR_VERDICT_AFTER_S = 35.0
CORR_SILENT_AFTER_S  = 12.0
corr_verdict_reported = False
gpstime_reported = False    # the moment the pulse first carries a GPS time
TIMESYNC_INTERVAL_S = 60.0
t_timesync = 0.0            # 0 -> write the first row as soon as there is one


class _NullSink:
    """Stands in for the capture files under --no-capture, so the loop
    below has no branch on whether it is recording."""

    def write(self, data): pass
    def flush(self): pass
    def writerow(self, row): pass


try:
    with contextlib.ExitStack() as stack:
        if capture_on:
            logf = stack.enter_context(open(rtcm_log, "wb"))
            rxf = stack.enter_context(open(ubx_log, "wb"))
            tsf = stack.enter_context(open(timesync_log, "w", newline=""))
            spf = stack.enter_context(open(speed_log, "w", newline=""))
            ts_writer = csv.writer(tsf)
            sp_writer = csv.writer(spf)
        else:
            logf = rxf = tsf = spf = ts_writer = sp_writer = _NullSink()

        # t_us columns are the firmware's u64 microseconds as received, not
        # unwrapped by anything here (inslib_protocol.md "Timebase"). The
        # one discontinuity left is a device restart, which is why its
        # running count is a column: a reader can tell a continuous
        # timeline from two of them without guessing.
        ts_writer.writerow(["utc_iso", "host_unix_s", "imu_t_us",
                            "timesync_t_us", "gps_week", "gps_tow_s",
                            "n_timesync_pairs", "segment", "mcu_restarts"])

        # Speed CSV. Both timebases per row on purpose: host_meas_unix_us is
        # what the source actually stated and stays authoritative, imu_t_us
        # is derived from a running offset and is a convenience index. If
        # the mapping later turns out to be off, it can be recomputed from
        # timesync.csv without repeating the drive - which is only possible
        # because the raw value was never thrown away.
        #
        # "kind" is what makes this extensible: ground speed is |v_n| and
        # fuses against the velocity states, whereas a future relative
        # airspeed is a different measurement model (it needs a wind
        # assumption), so the consumer has to be able to tell them apart
        # rather than infer it from the file name.
        sp_writer.writerow(["utc_iso", "host_recv_unix_us", "host_meas_unix_us",
                            "imu_t_us", "kind", "speed_mps", "stddev_mps",
                            "delay_ms", "flags", "segment"])
        while True:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                data = None
            if data:
                try:
                    ser.write(data)
                except (serial.SerialException, OSError):
                    ser, out_s = reconnect_serial(rx_status)
                    n_outage += 1
                    outage_total_s += out_s
                    ser.write(data)
                # Captured even if the write above had to be retried: the
                # file is the record of the correction stream, independent
                # of whether the link happened to be up.
                logf.write(data)
                n_pkt += 1
                n_tx += len(data)

                parse_buf += data
                rtcm_f, spartn_f = corr_frames(parse_buf)
                if rtcm_f or spartn_f:
                    last_corr_t = time.time()
                for name, enc in spartn_f:
                    corr_total += 1
                    spartn_types[name] += 1
                    spartn_enc_n += enc
                for msg_num, payload in rtcm_f:
                    corr_total += 1
                    rtcm_types[msg_num] += 1
                    base = parse_base_station(payload)
                    if base and base != last_base:
                        stn_id, x, y, z, height = base
                        lat, lon, h = ecef2llh(x, y, z)
                        print(f"\nBase station {stn_id}: ECEF=({x:.4f}, {y:.4f}, {z:.4f}) m  "
                              f"-> Lat={lat:.7f} Lon={lon:.7f} H={h:.2f} m"
                              + (f"  AntHeight={height:.3f} m" if height is not None else ""))
                        last_base = base

            # Speed side channel: drained every pass, logged, and re-emitted
            # onto the MCU timebase as 0x40/0x80. Never forwarded to serial.
            while True:
                try:
                    sdata, _saddr = speed_sock.recvfrom(2048)
                except (BlockingIOError, OSError):
                    break
                frame = parse_odometry_datagram(sdata)
                if frame is None:
                    n_speed_bad += 1
                    continue
                n_speed += 1
                last_speed = frame["speed_mps"]
                last_speed_wall = time.time()
                # The measurement is older than the frame by the delay the
                # producer measured; that, not the arrival, is the epoch to
                # stamp (and the one ins anchors its residual on).
                meas_us = frame["t_unix_us"] - frame["delay_ms"] * 1000
                imu_t = rx_status.host_to_imu_t_us(meas_us)
                sp_writer.writerow([
                    datetime.now(timezone.utc).isoformat(),
                    frame["t_unix_us"], meas_us,
                    "" if imu_t is None else imu_t,
                    "ground",
                    "%.4f" % frame["speed_mps"], "%.4f" % frame["stddev_mps"],
                    frame["delay_ms"], frame["flags"], rx_status.segment])

                # Complete the frame: everything the producer wrote is kept
                # verbatim, only t_us and the two hub-owned flag bits are
                # added. Keeping the raw host timestamp and delay is what
                # makes the capture self-sufficient -- the mapping can be
                # redone offline from timesync.csv without the CSV beside it.
                mappable, map_flags = rx_status.odo_time_flags()
                odo_flags = frame["flags"] | map_flags
                t_us = 0
                if mappable:
                    odo_flags |= ODO_T_US_VALID
                    t_us = imu_t
                    n_odo_tx += 1
                    if odo_flags & ODO_T_DEGRADED:
                        n_odo_degraded += 1
                else:
                    # No IMU stream yet, so nothing to map onto. Recorded
                    # unstamped rather than dropped: a live consumer ignores
                    # it (T_US_VALID is clear), an offline one can still
                    # place it once the whole session is available.
                    n_odo_unmapped += 1
                odo_frame = build_odometry(
                    frame["t_unix_us"], frame["speed_mps"], frame["stddev_mps"],
                    delay_ms=frame["delay_ms"], flags=odo_flags,
                    kind=frame["kind"], t_us=t_us)
                rxf.write(odo_frame)
                fanout.send([odo_frame])

            try:
                # Only ever ask for what is already there. read() blocks up
                # to the port timeout, and a device that goes quiet must not
                # stall the UDP side of the bridge for a second per pass.
                waiting = ser.in_waiting
                chunk = ser.read(waiting) if waiting else b""
            except (serial.SerialException, OSError):
                ser, out_s = reconnect_serial(rx_status)
                n_outage += 1
                outage_total_s += out_s
                chunk = b""

            if chunk:
                # Capture first, unconditionally. Nothing below this line
                # can cost bytes in the recording.
                rxf.write(chunk)
                n_rx += len(chunk)
                now_us = int(time.time() * 1e6)
                # The frames are needed for the status line anyway, so the
                # fan-out gets them for free and consumers never see a
                # datagram with half a frame in it.
                out_frames = []
                for cls, mid, payload, frame in framer.feed(chunk):
                    out_frames.append(frame)
                    if cls == 0x40:
                        rx_status.handle_inslib(mid, payload, now_us)
                        continue
                    # Standard u-blox: hand pyubx2 ONE complete, already
                    # checksum-verified frame, so it never has to guess at
                    # a protocol from the byte stream. UBXReader.parse is
                    # the static single-frame entry point (UBXMessage has
                    # no parse). Still guarded - a decoder fault must not
                    # end a recording either.
                    try:
                        rx_status.handle_ubx(UBXReader.parse(frame))
                    except Exception as exc:
                        n_parse_err += 1
                        if first_parse_err is None:
                            # Swallowing these silently hides the difference
                            # between a single corrupted frame and a decoder
                            # that rejects every frame there is, which looks
                            # exactly like a receiver sending nothing.
                            first_parse_err = f"{type(exc).__name__}: {exc}"
                            print(f"\n  first UBX decode failure: {first_parse_err}"
                                  f"\n  (capture unaffected; status line will show no"
                                  f" NAV-PVT/RAWX if this repeats)")
                fanout.send(out_frames)

            now = time.time()
            rx_status.poll_corr_in(ser, now)

            if not corr_reported and (rx_status.corr_in is not None
                                      or now - t0 >= CORR_REPORT_AFTER_S):
                print("\n  " + rx_status.corr_in_text())
                corr_reported = True

            if not corr_verdict_reported and (
                    now - t0 >= CORR_VERDICT_AFTER_S
                    or (now - t0 >= CORR_SILENT_AFTER_S and not corr_total)):
                print("\n  " + corr_verdict(n_tx, rtcm_types, spartn_types))
                corr_verdict_reported = True

            # The week/tow is worth seeing, but only when it changes from
            # "none" to a time: after that the status line's T says it.
            if not gpstime_reported and rx_status.tp_pair is not None:
                print("\n  time pulse paired: " + rx_status.gps_time_text())
                gpstime_reported = True

            # PPK readiness, once, early: the point is to be actionable
            # while the car is still parked.
            if not ppk_reported and now - t0 >= PPK_VERDICT_AFTER_S:
                verdict, ok = rx_status.ppk_verdict()
                print(("\n  " if ok else "\n\n  ") + verdict + ("" if ok else "\n"))
                ppk_reported = True

            if now - t_timesync >= TIMESYNC_INTERVAL_S:
                if write_timesync(ts_writer, tsf, rx_status):
                    t_timesync = now

            if now - tp >= 1.0:
                dt = now - tp
                # Flushed every second, not just on a clean Ctrl-C: a
                # recording tool that loses its last buffer to a hard kill
                # or a crashed laptop defeats the point of logging at all.
                # The binary captures need this more than the CSVs, not
                # less: a buffer holds a fixed number of BYTES, so the
                # slower the stream the longer it sits there. At 8 kB that
                # is a fifth of a second of return channel and minutes of a
                # correction stream. It also lets the files be read while
                # they are being written, which is the cheapest way to check
                # a recording without touching the running hub.
                logf.flush()
                rxf.flush()
                spf.flush()
                # One line, short enough not to wrap: what it has to answer
                # at a glance is whether each stream is flowing, and a rate
                # of 0.0 answers that as well as any wording would. Cumulative
                # byte counts, the correction packet count and the enabled
                # correction inputs are reported once at the end (or once
                # above) instead: a number that only grows does not need
                # repainting every second, and a wrapped line answers
                # nothing at all.
                print(f"\rIMU {rx_status.imu_n/dt:4.0f} Baro {rx_status.baro_n/dt:4.1f}"
                      f" Mag {rx_status.mag_n/dt:4.1f}@{rx_status.mag_text(now)}"
                      f" RAWX {rx_status.rawx_n/dt:4.1f} SFRBX {rx_status.sfrbx_n/dt:4.1f}"
                      f" CORR {corr_age_text(last_corr_t, now):>5}"
                      f" | {rx_status.pvt_text()} {rx_status.clock_flag()}"
                      f" | RX {n_rx/(now - t0)/1024:5.1f}KB/s {n_rx/1048576:5.1f}MB"
                      + (f" | {last_speed*3.6:5.1f}km/h"
                         if last_speed is not None
                         and now - last_speed_wall <= ODO_STALE_S
                         else " |   --     ")
                      + "  ",
                      end="")
                rx_status.imu_n = rx_status.baro_n = rx_status.mag_n = 0
                rx_status.rawx_n = rx_status.sfrbx_n = 0
                tp = now
except KeyboardInterrupt:
    if capture_on:
        print(f"\nDone: TX {n_pkt} pkt/{n_tx} bytes -> {rtcm_log}, RX {n_rx} bytes -> {ubx_log}")
        print(f"      clocks -> {timesync_log}")
    else:
        print(f"\nDone: TX {n_pkt} pkt/{n_tx} bytes, RX {n_rx} bytes (--no-capture, nothing written)")
    if fanout.enabled:
        print(f"      fan-out: {fanout.n_frames} frames in {fanout.n_datagrams} datagrams"
              + (f", {fanout.n_errors} send errors (consumer not listening?)"
                 if fanout.n_errors else ""))
    if n_outage:
        print(f"      serial link lost {n_outage}x, {outage_total_s:.1f} s total "
              f"(segments 0..{rx_status.segment})")
    if rx_status.mag_total:
        print(f"      magnetometer: {rx_status.mag_total} samples, "
              f"last |m| {rx_status.mag_text(time.time())} (uncalibrated)")
    else:
        print("      magnetometer: no 0x40/0x06 samples seen this session")
    if n_parse_err or framer.n_resync:
        print(f"      stream noise: {framer.n_resync} resyncs, {n_parse_err} undecodable "
              f"frames (skipped; the capture itself is unaffected)")
        if first_parse_err is not None:
            print(f"      first decode failure was: {first_parse_err}")
    if rx_status.imu_clock.restarts:
        print(f"      MCU timer restarted {rx_status.imu_clock.restarts}x "
              f"(device rebooted; t_us timeline is not continuous across that)")
    if rx_status.saw_tim_tp:
        print("      TIM-TP appeared in the stream: this firmware still forwards "
              "it instead of\n      building 0x40/0x05 from it. Expect no time "
              "sync frames and no GPS time here.")
    print(f"      speed  -> {speed_log if capture_on else '(not written)'}: {n_speed} samples"
          + (f", {n_speed_bad} malformed datagrams ignored" if n_speed_bad else ""))
    if n_speed:
        print(f"      odometry: {n_odo_tx} frames stamped with t_us"
              + (f", {n_odo_degraded} flagged T_DEGRADED" if n_odo_degraded else "")
              + (f", {n_odo_unmapped} recorded unstamped (no IMU stream to map onto)"
                 if n_odo_unmapped else ""))
    if n_speed and not rx_status.host_offsets:
        # Speed arrived but no IMU ever did, so nothing could be stamped.
        print("      (imu_t_us column is empty: no IMU stream to map the host clock onto)")
    print(f"      RXM-RAWX {rx_status.rawx_total}, RXM-SFRBX {rx_status.sfrbx_total}, "
          f"time sync {rx_status.tp_pairs_n}/{rx_status.tp_edges_n} pulses with GPS time"
          + (f", {rx_status.tp_suspect_n} flagged suspect"
             if rx_status.tp_suspect_n else ""))
    verdict, _ok = rx_status.ppk_verdict()
    print("      " + verdict.replace("\n  ", "\n      "))

    if rtcm_types:
        # Grouped rather than a bare list: what decides whether a correction
        # stream is usable is which KINDS are present, not how many types.
        groups = collections.defaultdict(list)
        for num, cnt in sorted(rtcm_types.items()):
            groups[rtcm_group(num)].append(f"{num}x{cnt}")
        print("      RTCM3 types: "
              + " | ".join(f"{g}: {' '.join(v)}" for g, v in sorted(groups.items())))
        missing = []
        if not any(n in RTCM_ARP for n in rtcm_types):
            missing.append("base position (1005/1006)")
        if not any(n in RTCM_EPH for n in rtcm_types):
            missing.append("ephemeris (1019/1020/1042/1046)")
        if missing:
            # Not necessarily a problem: the rover's own SFRBX covers the
            # ephemeris, and a stream fed by NTRIP normally carries the ARP.
            # Worth naming either way, since neither is visible afterwards.
            print("      RTCM3 stream carries no " + " and no ".join(missing)
                  + (" (rover SFRBX can supply the ephemeris instead)"
                     if "ephemeris (1019/1020/1042/1046)" in missing else ""))

    if corr_total:
        print(f"      corrections: {corr_total} frames forwarded")

    if spartn_types:
        # OCB (orbit/clock/bias) and HPAC (atmosphere) per constellation are
        # what a PPP-RTK stream lives on, GAD defines the areas HPAC refers
        # to. A stream with OCB but no HPAC converges far slower, and one
        # without GAD leaves the HPAC areas undefined, so name them.
        print("      SPARTN types: "
              + " ".join(f"{n}x{c}" for n, c in sorted(spartn_types.items())))
        absent = [k for k in ("OCB", "HPAC", "GAD")
                  if not any(n.split("-")[0] == k for n in spartn_types)]
        if absent:
            print("      SPARTN stream carries no " + " and no ".join(absent))
        # Corrections over NTRIP arrive unencrypted, so a set EAF flag means
        # the receiver needs keys that nothing here supplies.
        if spartn_enc_n:
            print(f"      WARNING: {spartn_enc_n} SPARTN frames are flagged "
                  f"encrypted, the receiver needs the matching keys")
