"""Shared UBX framing and class-0x40 payloads (see inslib_protocol.md).

Every host tool here needs the same three things: split a byte stream into
checksum-verified UBX frames, build a frame, and decode the firmware's own
class-0x40 payloads. They used to carry a copy each, which is how one of
them ended up with a framing bug the others did not have.

Deliberately NOT pyubx2's stream reader for the framing, for two reasons.

The capture must be written BEFORE anything is parsed. Reading through a
parser couples the recording to it, and then a parser that throws takes
the recording with it - which is exactly what happened once: the IMU
payloads are raw floats, so a 0x24 ('$') turns up in them several times a
second, pyubx2 reads that as an NMEA header and calls .decode() on the
following bytes, and a byte sequence that is not valid UTF-8 raises
straight out of read(). quitonerror only covers pyubx2's own exception
types, not that one.

Second, the stream is lossy by contract, so resynchronising on the sync
bytes and verifying the checksum is the documented behaviour rather than a
workaround.

pyubx2 is still the right tool for the STANDARD u-blox messages (NAV-PVT,
NAV-COV, RXM-*): hand it one complete, already checksum-verified frame via
UBXReader.parse() and it never has to guess at a protocol from a byte
stream.
"""
import math
import struct

UBX_SYNC = b"\xB5\x62"
UBX_MAX_PAYLOAD = 4096   # anything larger is a false sync, not a message

# --- class 0x40, the firmware's own sensors (and the host's own inputs) ---
CLS_INSLIB = 0x40
ID_IMU = 0x01
ID_BARO = 0x02
# 0x03 was the timepulse capture, retired: it carried only (t_us, count)
# and left the pairing with TIM-TP to the host. It must not be reused, so
# an old firmware cannot have a new payload read out of it.
ID_STATUS = 0x04
ID_TIMESYNC = 0x05
ID_MAG = 0x06
ID_IMUHEALTH = 0x0D    # device -> host, IMU acquisition diagnostics
ID_SATUR = 0x0E        # device -> host, one measurement range episode
ID_NAV = 0x0F          # device -> host, the navigation solution
ID_ODOMETRY = 0x80     # first host-produced id, see inslib_protocol.md

# t_us, acc[3] g, gyr[3] dps, status word, seq. The status word replaced a
# f32 die temperature: four bytes and 24 bits of mantissa for a number
# nobody reads past the first decimal, at 800 Hz, while the sample said
# nothing about itself. Same 38 bytes, see decode_imu_status.
IMU_FMT = "<Q6fIH"

# 0x40/0x01 status word, mirroring ubx.h. Bits 14..31 are reserved: they
# are sent as zero and a decoder must IGNORE them rather than reject a
# frame that has them set, which is what lets a field be added later
# without breaking every reader.
IMU_ST_TEMP_MASK = 0x7FF          # bits 0..10, signed, 0.1 degC
IMU_ST_TEMP_SIGN = 0x400
IMU_ST_SAT_ACC = 1 << 11
IMU_ST_SAT_GYR = 1 << 12
IMU_ST_CAL_APPLIED = 1 << 13
BARO_FMT = "<Qff"
# t_us, mag[3] uT, temp degC. Axes are already in the same body frame as
# the IMU message (the driver's mounting remap), inslib_protocol.md
# 0x40/0x06. The temperature is diagnostic only and not synchronous with
# the field on the MMC5603.
MAG_FMT = "<Qffff"
# t_us, count, tow_ms, tow_sub_ms, q_err_ps, week, flags: one hardware
# pulse capture already paired with the TIM-TP that announced it
# (inslib_protocol.md 0x40/0x05).
TIMESYNC_FMT = "<QIIIiHH"
# t_unix_us, t_us, speed, stddev, delay_ms, flags, kind. Filled in two
# stages: the producer writes everything but t_us, the hub adds t_us and
# sets ODO_T_US_VALID (inslib_protocol.md 0x40/0x80).
ODOMETRY_FMT = "<QQffHHH"

IMU_LEN = struct.calcsize(IMU_FMT)
BARO_LEN = struct.calcsize(BARO_FMT)
MAG_LEN = struct.calcsize(MAG_FMT)
TIMESYNC_LEN = struct.calcsize(TIMESYNC_FMT)
ODOMETRY_LEN = struct.calcsize(ODOMETRY_FMT)

# Time sync flag bits, inslib_protocol.md 0x40/0x05.
TS_GPS_VALID = 0x0001
TS_UTC_BASE = 0x0002
TS_UTC_AVAIL = 0x0004
TS_QERR_VALID = 0x0008
TS_TP_SUSPECT = 0x0010

# Odometry flag bits, inslib_protocol.md 0x40/0x80.
ODO_DIR_VALID = 0x0001
ODO_REVERSE = 0x0002
ODO_T_DEGRADED = 0x0004
ODO_DELAY_SUSPECT = 0x0008
ODO_T_US_VALID = 0x0010

ODO_KIND_GROUND_SPEED = 0

# Standard u-blox NAV-PVT (class 0x01, id 0x07), arriving on the same link
# as the class-0x40 frames whenever the board passes a GNSS receiver's
# output through. Just enough to pick the frame out of the generic framer;
# the payload itself is decoded with pyubx2, per this module's docstring.
CLASS_NAV = 0x01
ID_NAV_PVT = 0x07

GPS_SEC_PER_WEEK = 604800.0

# A datagram stays inside a typical 1500-byte MTU. Frames are packed up to
# this and never split across datagrams (inslib_protocol.md).
UDP_MAX_PAYLOAD = 1400


def ubx_checksum(data):
    """8-bit Fletcher over class, id, length and payload."""
    ck_a = ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def ubx_frame(msg_cls, msg_id, payload):
    """Build one complete UBX frame."""
    body = struct.pack("<BBH", msg_cls, msg_id, len(payload)) + payload
    ck_a, ck_b = ubx_checksum(body)
    return UBX_SYNC + body + bytes((ck_a, ck_b))


class UbxFramer:
    """Splits a byte stream into checksum-verified UBX frames.

    feed() returns a list of (cls, mid, payload, frame) for every complete
    frame, where `frame` is the raw bytes including sync and checksum -
    what pyubx2 or a forwarder wants. Bytes that do not resolve into a
    valid frame are skipped and counted in n_resync.
    """

    def __init__(self):
        self.buf = bytearray()
        self.n_resync = 0

    def reset(self):
        """Drop the partial frame held over from the previous feed().

        For a DATAGRAM source: a datagram never splits a frame, so a
        leftover is the tail of a lost or reordered one rather than a
        continuation, and splicing it onto the next datagram would
        manufacture a frame that was never sent."""
        self.n_resync += len(self.buf)
        self.buf.clear()

    def feed(self, data):
        self.buf += data
        out = []
        i, n = 0, len(self.buf)
        while True:
            j = self.buf.find(UBX_SYNC, i)
            if j < 0:
                # Keep one trailing byte: a sync pair may straddle reads.
                i = max(i, n - 1)
                break
            if j + 6 > n:
                i = j
                break
            cls, mid, ln = struct.unpack("<BBH", self.buf[j + 2:j + 6])
            if ln > UBX_MAX_PAYLOAD:
                self.n_resync += 1
                i = j + 2
                continue
            end = j + 6 + ln + 2
            if end > n:
                i = j
                break
            frame = bytes(self.buf[j:end])
            if ubx_checksum(frame[2:-2]) == (frame[-2], frame[-1]):
                out.append((cls, mid, frame[6:-2], frame))
                i = end
            else:
                self.n_resync += 1
                i = j + 2
        del self.buf[:i]
        return out


class TimerRestartWatch:
    """Watches an MCU timestamp stream for a device restart.

    t_us is monotonic u64 and must NOT be unwrapped (inslib_protocol.md
    "Timebase"), so a backward step no longer has two possible meanings:
    the only thing that resets the counter is the MCU starting over. What
    the host still has to know is that it happened, because the timeline
    before and after are two different ones and anything derived from the
    old one (a host-clock offset, a clock fit) describes a device that no
    longer exists.

    Feed it ONE stream - the IMU - whose timestamps are strictly
    increasing; mixing in the baro or the pulse capture would report their
    normal interleaving as a restart."""

    def __init__(self):
        self.last_t_us = None
        self.restarts = 0

    def feed(self, t_us):
        """True if this sample begins a new timeline."""
        restarted = self.last_t_us is not None and t_us < self.last_t_us
        if restarted:
            self.restarts += 1
        self.last_t_us = t_us
        return restarted


# Field names of 0x40/0x0D, in WIRE order, which is the order app.c fills
# the array in and not the declaration order of the C struct behind it.
# The message has grown once already, so it is decoded by the length that
# arrived rather than against a constant: a shorter frame from an older
# firmware yields the fields it carried, and a longer one from a newer
# firmware yields the ones this build knows plus the rest under "extra".
IMUHEALTH_FIELDS = (
    "stalls", "reinits", "reinit_fails", "spi_errors",
    "last_busy", "last_overruns", "last_spi_error", "last_spi_state",
    "last_exti_pend", "last_drdy_level", "last_fifo_used", "at_seq",
    "at_ms", "reg_whoami", "reg_pwr_mgmt0", "reg_int1_cfg0",
    "reg_read_rc", "sat_sticky", "sat_samples",
)

# Bit per body axis in sat_sticky and in the episode mask below. Body
# axes, i.e. the ones the IMU message itself reports in.
SAT_AXES = ("acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z")
SAT_EP_DROPPED = 0x01   # episodes were lost before this one

SATUR_FMT = "<3H2B"
SATUR_LEN = struct.calcsize(SATUR_FMT)


def sat_axis_names(mask):
    """The axes a saturation mask names, in bit order."""
    return [n for i, n in enumerate(SAT_AXES) if mask & (1 << i)]


def parse_imuhealth(payload):
    """0x40/0x0D -> dict, or None if the payload is not whole u32."""
    if not payload or len(payload) % 4:
        return None
    words = struct.unpack("<%dI" % (len(payload) // 4), payload)
    out = dict(zip(IMUHEALTH_FIELDS, words))
    if len(words) > len(IMUHEALTH_FIELDS):
        out["extra"] = list(words[len(IMUHEALTH_FIELDS):])
    if "sat_sticky" in out:
        out["sat_axes"] = sat_axis_names(out["sat_sticky"])
    return out


def parse_satur(payload):
    """0x40/0x0E -> dict, or None on a wrong length.

    One run of consecutive clipped samples. first_seq/last_seq are the
    IMU message's own seq, so the run can be marked in a recorded stream
    directly. samples is carried rather than derived from the two: a
    dropped sample leaves a gap in seq, and the bracket is then wider
    than the run inside it."""
    if len(payload) != SATUR_LEN:
        return None
    first, last, samples, mask, flags = struct.unpack(SATUR_FMT, payload)
    return {
        "first_seq": first,
        "last_seq": last,
        "samples": samples,
        "mask": mask,
        "axes": sat_axis_names(mask),
        "flags": flags,
        "dropped_before": bool(flags & SAT_EP_DROPPED),
    }


def imu_status_temp_c(status):
    """Die temperature out of the status word, degC."""
    t = status & IMU_ST_TEMP_MASK
    if t & IMU_ST_TEMP_SIGN:
        t -= (IMU_ST_TEMP_MASK + 1)
    return t / 10.0


def decode_imu_status(status):
    """The 0x40/0x01 status word as a dict.

    sat_acc/sat_gyr mean at least one axis of that sensor sat at its end
    stop in THIS sample. The reading is then not a measurement at all --
    it is the end of the range, which looks entirely plausible in the
    values next to it. Which axis it was is in 0x40/0x0E.

    cal_applied says the stored calibration reached this sample. It is
    not derivable from the values: a device streaming raw data looks
    exactly like a calibrated one whose corrections happen to be small."""
    return {
        "temp_c": imu_status_temp_c(status),
        "sat_acc": bool(status & IMU_ST_SAT_ACC),
        "sat_gyr": bool(status & IMU_ST_SAT_GYR),
        "saturated": bool(status & (IMU_ST_SAT_ACC | IMU_ST_SAT_GYR)),
        "cal_applied": bool(status & IMU_ST_CAL_APPLIED),
    }


def build_imu_status(temp_c, sat_acc=False, sat_gyr=False, cal_applied=False):
    """Inverse of decode_imu_status, for producers of IMU frames.

    Clamps rather than wraps, the same way the firmware does: the limits
    sit far outside anything the sensor survives, so hitting one means
    the reading is already meaningless, and a wrapped value would come
    back as a plausible temperature of the wrong sign."""
    t = temp_c
    if not math.isfinite(t):
        t = -102.4
    t = max(-102.4, min(102.3, t))
    q = int(math.floor(t * 10.0 + 0.5)) if t >= 0 else -int(math.floor(-t * 10.0 + 0.5))
    status = q & IMU_ST_TEMP_MASK
    if sat_acc:
        status |= IMU_ST_SAT_ACC
    if sat_gyr:
        status |= IMU_ST_SAT_GYR
    if cal_applied:
        status |= IMU_ST_CAL_APPLIED
    return status


# 0x40/0x0F. Field order is the firmware's declaration order; offsets are
# fixed and new fields go on the end. Generous with bytes on purpose: it
# goes out ten times a second, where 128 bytes is 1.3 kB/s, and packing a
# diagnostic message costs the clarity it exists to provide.
NAV_FMT = ("<"
           "BBBBBB"      # version, mode, att_src, cal_applied, sat_sticky, cpu_pct
           "HII"         # imu_hz, flags, update_wcet_us
           "6f"          # roll pitch yaw, then their 1-sigmas
           "3f3f"        # vel_ned, pos_ned
           "5f"          # height, height_ell, baro_alt, baro_meas, vz
           "3f"          # leverarm frd
           "dd"          # ins lat, lon
           "2f")         # mag ref uT, declination
NAV_LEN = struct.calcsize(NAV_FMT)

NAV_MODE = {0: "none", 1: "attitude", 2: "coasting", 3: "full"}
NAV_ATT_SRC = {0: "none", 1: "ins", 2: "ahrs", 3: "ars"}

# Every value is paired with a validity bit rather than with a sentinel: a
# filter that has no position must SAY so, because a zero, a NaN and a
# stale number all read as an answer.
NAV_F = {
    "ready": 1 << 0, "rpy": 1 << 1, "rpy_sigma": 1 << 2, "yaw_sigma": 1 << 3,
    "vel": 1 << 4, "pos": 1 << 5, "latlon": 1 << 6, "height": 1 << 7,
    "height_ell": 1 << 8, "baro_alt": 1 << 9, "baro_meas": 1 << 10,
    "vz": 1 << 11, "leverarm": 1 << 12, "leverarm_bad": 1 << 13,
    "zupt": 1 << 14, "zaru": 1 << 15, "vzupt": 1 << 16,
    "mag_ref": 1 << 17,
}


# 0x40/0x04, in the order app.c fills it. Counters, all u32, all since
# boot except loop_count which is per 5 s window.
STATUS_FIELDS = ("tx_dropped", "imu_overruns", "loop_count", "uart_errors",
                 "cfg_fail_mask", "vcp_dropped", "gnss_rx_bytes",
                 "gnss_tx_dropped", "gnss_rx_overflows", "gnss_crc_errors")
STATUS_LEN = 4 * len(STATUS_FIELDS)


# UBX-NAV-PVT (u-blox interface description), the fields a console needs.
# Offsets restated from the receiver's own spec, the same ones nav_glue.c
# reads on the device. This is a VENDOR message: neither side owns the
# layout, so both read it from the same document rather than from each
# other, and pyubx2 stays an optional convenience elsewhere instead of a
# dependency here.
NAV_PVT_LEN = 92


def parse_nav_pvt(payload):
    """0x01/0x07 -> dict, or None on a short payload.

    lat/lon are returned whatever the fix state; fix_ok and fix_type say
    whether to use them. Without a fix the receiver still sends the
    message, carrying whatever the last search left behind - printing
    those unfiltered is worse than printing nothing, because they look
    like measurements."""
    if len(payload) < NAV_PVT_LEN:
        return None
    flags = payload[21]
    u32 = lambda o: struct.unpack_from("<I", payload, o)[0]   # noqa: E731
    i32 = lambda o: struct.unpack_from("<i", payload, o)[0]   # noqa: E731
    return {
        "fix_type": payload[20],
        "fix_ok": bool(flags & 0x01),
        "carr_soln": (flags >> 6) & 0x03,
        "num_sv": payload[23],
        "lon_deg": i32(24) * 1e-7,
        "lat_deg": i32(28) * 1e-7,
        "height_m": i32(32) * 1e-3,
        "hacc_m": u32(40) * 1e-3,
        "vacc_m": u32(44) * 1e-3,
        "vel_ned": (i32(48) * 1e-3, i32(52) * 1e-3, i32(56) * 1e-3),
        "sacc_mps": u32(68) * 1e-3,
        # Course over ground: the direction the antenna is TRAVELLING,
        # which is only an attitude on a platform that cannot move
        # sideways, and is noise below walking pace whatever the
        # platform. Valid whenever the fix has a velocity at all.
        "head_mot_deg": i32(64) * 1e-5,
        # Vehicle heading: where the platform POINTS. A plain positioning
        # receiver has no way of knowing it and clears the bit; it is
        # filled by the receivers that carry a heading source of their
        # own, a second antenna or an inertial sensor. Read the bit, not
        # the number - the field is zero rather than absent when unset.
        "head_veh_deg": i32(84) * 1e-5,
        "head_veh_valid": bool(flags & 0x20),
    }


def parse_status(payload):
    """0x40/0x04 -> dict, or None if the payload is not whole u32.

    Decoded by the length that arrived rather than against a constant:
    this message is a list of counters and lists grow. Names run out when
    the frame is longer than this build knows, and the surplus is kept
    under "extra" instead of being dropped."""
    if not payload or len(payload) % 4:
        return None
    words = struct.unpack("<%dI" % (len(payload) // 4), payload)
    out = dict(zip(STATUS_FIELDS, words))
    if len(words) > len(STATUS_FIELDS):
        out["extra"] = list(words[len(STATUS_FIELDS):])
    return out


def parse_nav(payload):
    """0x40/0x0F -> dict, or None on a wrong length.

    Values whose validity bit is clear come back as None rather than as
    the number that happened to be in the frame."""
    if len(payload) != NAV_LEN:
        return None
    v = struct.unpack(NAV_FMT, payload)
    f = v[7]

    def opt(flag, value):
        return value if f & NAV_F[flag] else None

    return {
        "version": v[0],
        "mode": NAV_MODE.get(v[1], "?"),
        "mode_id": v[1],
        "att_src": NAV_ATT_SRC.get(v[2], "?"),
        "cal_applied": v[3],
        "sat_sticky": v[4],
        "sat_axes": sat_axis_names(v[4]),
        "cpu_pct": v[5],
        "imu_hz": v[6],
        "flags": f,
        "ready": bool(f & NAV_F["ready"]),
        "update_wcet_us": v[8],
        "rpy_deg": opt("rpy", (v[9], v[10], v[11])),
        "rpy_sigma_deg": opt("rpy_sigma", (v[12], v[13], v[14])),
        # The ARS free-integrates yaw, so its sigma is absent rather than
        # zero -- a zero there would read as a perfectly known heading.
        "yaw_sigma_deg": opt("yaw_sigma", v[14]),
        "vel_ned": opt("vel", (v[15], v[16], v[17])),
        "pos_ned": opt("pos", (v[18], v[19], v[20])),
        "height_m": opt("height", v[21]),
        # Absolute only in the sense that it refers to the ellipsoid. It
        # comes from the local height plus an estimated offset whenever
        # the INS is not running under fresh GNSS, and that offset
        # converges over tens of seconds -- so a valid value here can
        # still be a long way out, with nothing in the message saying so.
        "height_ell_m": opt("height_ell", v[22]),
        "baro_alt_m": opt("baro_alt", v[23]),
        "baro_meas_m": opt("baro_meas", v[24]),
        "vz_mps": opt("vz", v[25]),
        "leverarm_frd": (v[26], v[27], v[28]),
        "leverarm_set": bool(f & NAV_F["leverarm"]),
        "leverarm_bad": bool(f & NAV_F["leverarm_bad"]),
        "lat_deg": opt("latlon", v[29]),
        "lon_deg": opt("latlon", v[30]),
        # A declination of zero is a real value in places, so "not known
        # here yet" has to be said with a bit and not with the number.
        "mag_ref_uT": opt("mag_ref", v[31]),
        "mag_decl_deg": opt("mag_ref", v[32]),
        "zupt": bool(f & NAV_F["zupt"]),
        "zaru": bool(f & NAV_F["zaru"]),
        "vzupt": bool(f & NAV_F["vzupt"]),
    }


def parse_timesync(payload):
    """0x40/0x05 payload -> dict, or None on a wrong length.

    gps_s is the GPS instant of the pulse edge in continuous seconds, or
    None when the firmware had no announcement for this edge - the frame
    is then an "the pulse is alive" report and nothing else, and every
    time field in it is zero."""
    if len(payload) != TIMESYNC_LEN:
        return None
    t_us, count, tow_ms, tow_sub_ms, q_err_ps, week, flags = struct.unpack(
        TIMESYNC_FMT, payload)
    out = {"t_us": t_us, "count": count, "tow_ms": tow_ms,
           "tow_sub_ms": tow_sub_ms, "q_err_ps": q_err_ps, "week": week,
           "flags": flags,
           "gps_valid": bool(flags & TS_GPS_VALID),
           "utc_base": bool(flags & TS_UTC_BASE),
           "suspect": bool(flags & TS_TP_SUSPECT),
           "tow_s": None, "gps_s": None}
    if out["gps_valid"]:
        out["tow_s"] = (tow_ms + tow_sub_ms * (2.0 ** -32)) / 1e3
        gps_s = week * GPS_SEC_PER_WEEK + out["tow_s"]
        if flags & TS_QERR_VALID:
            # The receiver can only place the edge on its own clock grid
            # and reports the residual it knows it was off by. Sub-
            # nanosecond, so it changes nothing at vehicle speeds; applied
            # because it is free and the sign convention belongs in one
            # place rather than in every consumer.
            gps_s += q_err_ps * 1e-12
        out["gps_s"] = gps_s
    return out


def build_odometry(t_unix_us, speed_mps, stddev_mps, delay_ms=0, flags=0,
                   kind=ODO_KIND_GROUND_SPEED, t_us=0):
    """Build a 0x40/0x80 frame.

    A producer calls this with t_us left at 0 and ODO_T_US_VALID clear; the
    hub rebuilds the same frame with t_us filled in."""
    payload = struct.pack(ODOMETRY_FMT, int(t_unix_us) & 0xFFFFFFFFFFFFFFFF,
                          int(t_us) & 0xFFFFFFFFFFFFFFFF, float(speed_mps),
                          float(stddev_mps), int(delay_ms) & 0xFFFF,
                          int(flags) & 0xFFFF, int(kind) & 0xFFFF)
    return ubx_frame(CLS_INSLIB, ID_ODOMETRY, payload)


def parse_odometry(payload):
    """0x40/0x80 payload -> dict, or None on a wrong length."""
    if len(payload) != ODOMETRY_LEN:
        return None
    t_unix_us, t_us, speed_mps, stddev_mps, delay_ms, flags, kind = struct.unpack(
        ODOMETRY_FMT, payload)
    return {"t_unix_us": t_unix_us, "t_us": t_us, "speed_mps": speed_mps,
            "stddev_mps": stddev_mps, "delay_ms": delay_ms, "flags": flags,
            "kind": kind,
            "t_us_valid": bool(flags & ODO_T_US_VALID),
            "reverse": bool(flags & ODO_DIR_VALID) and bool(flags & ODO_REVERSE)}


def parse_odometry_datagram(data):
    """One whole UBX 0x40/0x80 datagram -> dict, or None if it is not one.

    Checksummed rather than trusted: this arrives on a socket anyone on
    the host can write to, and a malformed frame logged as a measurement
    is worse than a dropped one."""
    if len(data) != 8 + ODOMETRY_LEN or data[:2] != UBX_SYNC:
        return None
    cls, mid, ln = struct.unpack("<BBH", data[2:6])
    if (cls, mid, ln) != (CLS_INSLIB, ID_ODOMETRY, ODOMETRY_LEN):
        return None
    if ubx_checksum(data[2:-2]) != (data[-2], data[-1]):
        return None
    return parse_odometry(data[6:-2])


def datagrams(frames, max_payload=UDP_MAX_PAYLOAD):
    """Pack complete frames into datagram-sized chunks.

    A frame is never split across two datagrams, so a lost datagram costs
    exactly the frames inside it instead of also desynchronising the frame
    that straddled the boundary. A single frame larger than max_payload
    gets a datagram of its own rather than being dropped."""
    out, cur = [], b""
    for f in frames:
        if cur and len(cur) + len(f) > max_payload:
            out.append(cur)
            cur = b""
        cur += f
    if cur:
        out.append(cur)
    return out


# ---------------------------------------------------------------------------
# Configuration interface (class 0x40, ids 0x07..0x0C)
#
# See inslib_protocol.md "Configuration interface". These are the only
# messages the firmware consumes: everything else a host writes to the
# link is relayed to the u-blox receiver byte for byte.
#
# The encoding follows u-blox CFG-VALSET/VALGET, with one addition:
# storage size id 6 is a variable-length record whose value is preceded
# by a u16 length. That is what carries a calibration node as one key.
# ---------------------------------------------------------------------------

ID_CFGINFO = 0x07      # device -> host, store and calibration state
ID_VALSET = 0x08       # host   -> device
ID_VALGET = 0x09       # host   -> device, poll
ID_VALGET_R = 0x0A     # device -> host, poll response
ID_CFGACK = 0x0B       # device -> host
ID_CFGRESET = 0x0C     # host   -> device

# Storage size ids, bits 30..28 of a key.
SZ_BIT = 0x01
SZ_U1 = 0x02
SZ_U2 = 0x03
SZ_U4 = 0x04
SZ_U8 = 0x05
SZ_BLOB = 0x06

# Width a size id implies. None means the width travels with the value.
SZ_WIDTH = {SZ_BIT: 1, SZ_U1: 1, SZ_U2: 2, SZ_U4: 4, SZ_U8: 8, SZ_BLOB: None}

ITEM_WILDCARD = 0xFFFF

# VALSET layer bits. A write always lands in the live layer; the flash
# bit additionally persists the whole image (inslib_protocol.md).
LAYER_RAM = 0x01
LAYER_FLASH = 0x04

# VALGET layer, an enum rather than a mask.
VALGET_RAM = 0x00
VALGET_DEFAULT = 0x07

GRP_MSGOUT = 0x001
GRP_RATE = 0x002
GRP_IMU = 0x003
GRP_MAG = 0x004
GRP_FRAME = 0x005
GRP_CAL_ACC = 0x010
GRP_CAL_GYR = 0x011
GRP_CAL_MAG = 0x012

CAL_PTS_MAX = 16
CAL_POLY_MAX = 4               # coefficients per axis, degree 0..3
# A degree of this means no polynomial has been fitted over the current
# nodes. A record can be replaced but never removed, so without it a fit
# made over an older node set would stay in charge of the bias after a
# node is added.
CAL_POLY_NONE = 0xFF
CAL_POINT_LEN = 52             # temp_c, M[9] column major, b[3]
HOUSING_LEN = 36               # R[9] column major, the mounting rotation
LEVERARM_LEN = 12              # l[3] body frame FRD, metres, IMU -> antenna
# Mirrors INSLIB_CFG_LEVERARM_MAX_M in cfg_keys.h, which carries the
# reasoning: a sanity bound past the largest airframe, so that catching a
# unit mistake never costs a real installation.
LEVERARM_MAX_M = 100
CAL_BIASPOLY_LEN = 56          # t_ref, deg, pad[3], c[3][4] axis major

# Result codes of 0x40/0x0B, the numbering is part of the protocol.
CFG_OK = 0
CFG_E_KEY = 1
CFG_E_LEN = 2
CFG_E_RANGE = 3
CFG_E_FLASH = 4
CFG_E_FULL = 5
CFG_E_FRAME = 6
CFG_E_LAYER = 7
CFG_E_UNSET = 8

CFG_RESULT_TEXT = {
    CFG_OK: "accepted",
    CFG_E_KEY: "no such key",
    CFG_E_LEN: "value length does not match the key",
    CFG_E_RANGE: "value outside the key's permitted range",
    CFG_E_FLASH: "erase or program failed",
    CFG_E_FULL: "the live image has no room for this record",
    CFG_E_FRAME: "malformed message",
    CFG_E_LAYER: "layer mask empty or unsupported",
    CFG_E_UNSET: "the key is known but carries no value",
}

# 0x40/0x07 flags.
CFGINFO_STORED = 0x01
CFGINFO_UNSAVED = 0x02
CFGINFO_IMU_CAL = 0x04
CFGINFO_MAG_CAL = 0x08

# 0x40/0x07 cal_flags.
CAL_ACC_VALID = 0x01
CAL_GYR_VALID = 0x02
CAL_MAG_VALID = 0x04
CAL_ACC_CLAMPED = 0x08
CAL_GYR_CLAMPED = 0x10
CAL_MAG_CLAMPED = 0x20
CAL_HOUSING = 0x40             # a housing rotation is being applied
CAL_HOUSING_BAD = 0x80         # one is stored and is not a rotation

# The firmware reports the accelerometer in g and the consumers map it
# with this constant, so a bias expressed in m/s^2 converts with the same
# one and NOT with the local gravity the solver used (tools/insrcv.c:
# acc_mps2 = raw_g * INS_GRAVITY_NOMINAL).
GRAVITY_NOMINAL = 9.80665
RAD_TO_DEG = 57.29577951308232


def cfg_key(size, group, item):
    """Assemble a key from its three fields."""
    return ((size & 0x7) << 28) | ((group & 0xFFF) << 16) | (item & 0xFFFF)


def cfg_key_size(key):
    return (key >> 28) & 0x7


def cfg_key_group(key):
    return (key >> 16) & 0xFFF


def cfg_key_item(key):
    return key & 0xFFFF


def cal_point_key(group, n):
    return cfg_key(SZ_BLOB, group, 0x100 + n)


def cal_npts_key(group):
    return cfg_key(SZ_U1, group, 0x001)


def cal_biaspoly_key(group):
    return cfg_key(SZ_BLOB, group, 0x002)


def housing_key():
    return cfg_key(SZ_BLOB, GRP_FRAME, 0x001)


def leverarm_key():
    return cfg_key(SZ_BLOB, GRP_FRAME, 0x002)


def pack_leverarm(xyz):
    """GNSS antenna lever arm -> the 12 byte record.

    Body frame FRD (x forward, y right, z down), metres, IMU to antenna
    phase centre. Same quantity and sign as gnss.leverarm_frd in the host
    side config.yaml."""
    v = [float(c) for c in xyz]
    if len(v) != 3:
        raise ValueError("lever arm must be three numbers")
    for c in v:
        if not math.isfinite(c):
            raise ValueError("lever arm components must be finite")
        if abs(c) > LEVERARM_MAX_M:
            # Refused here as well as on the board, because this is where
            # a person types it and where the message can say what the
            # units are. The board would only be able to light a lamp.
            raise ValueError(
                "lever arm %g m exceeds %d m on one axis -- the value is in "
                "METRES from the IMU to the antenna, not millimetres"
                % (c, LEVERARM_MAX_M))
    return struct.pack("<3f", *v)


def unpack_leverarm(raw):
    """Inverse of pack_leverarm -> [x, y, z], or None on a bad length."""
    if len(raw) != LEVERARM_LEN:
        return None
    return list(struct.unpack("<3f", raw))


# Every key the firmware knows, by name. The names are what the command
# line tool spells out, so they are part of the user interface and not
# just a lookup table. The descriptor table in the firmware's cfg.c is
# the authority on defaults and ranges.
CFG_KEYS = {
    "CFG-MSGOUT-IMU": cfg_key(SZ_U1, GRP_MSGOUT, 0x001),
    "CFG-MSGOUT-BARO": cfg_key(SZ_U1, GRP_MSGOUT, 0x002),
    "CFG-MSGOUT-MAG": cfg_key(SZ_U1, GRP_MSGOUT, 0x003),
    "CFG-MSGOUT-STATUS": cfg_key(SZ_U1, GRP_MSGOUT, 0x004),
    "CFG-MSGOUT-CFGINFO": cfg_key(SZ_U1, GRP_MSGOUT, 0x005),
    "CFG-MSGOUT-NAV": cfg_key(SZ_U1, GRP_MSGOUT, 0x006),

    "CFG-RATE-MAG_MS": cfg_key(SZ_U2, GRP_RATE, 0x001),
    "CFG-RATE-CFGINFO_MS": cfg_key(SZ_U2, GRP_RATE, 0x002),
    "CFG-RATE-NAV_MS": cfg_key(SZ_U2, GRP_RATE, 0x003),

    "CFG-IMU-APPLY_CAL": cfg_key(SZ_U1, GRP_IMU, 0x001),
    "CFG-IMU-TEMP_TAU_MS": cfg_key(SZ_U2, GRP_IMU, 0x002),
    # Output data rate in Hz, and the UI low pass as the DIVIDER of it.
    # The low pass is a divider and not a frequency because that is what
    # the register holds: the corner follows the rate on its own, so a
    # stored 16 stays true when the rate is halved and a stored 50 Hz
    # would not. What it currently works out to is in 0x40/0x07.
    "CFG-IMU-ODR_HZ": cfg_key(SZ_U2, GRP_IMU, 0x003),
    "CFG-IMU-LPF_DIV": cfg_key(SZ_U2, GRP_IMU, 0x004),
    "CFG-MAG-APPLY_CAL": cfg_key(SZ_U1, GRP_MAG, 0x001),
    "CFG-MAG-TEMP_TAU_MS": cfg_key(SZ_U2, GRP_MAG, 0x002),

    "CFG-FRAME-HOUSING": cfg_key(SZ_BLOB, GRP_FRAME, 0x001),
    "CFG-FRAME-LEVERARM": cfg_key(SZ_BLOB, GRP_FRAME, 0x002),
}

CFG_GROUPS = {
    "CFG-MSGOUT": GRP_MSGOUT,
    "CFG-RATE": GRP_RATE,
    "CFG-IMU": GRP_IMU,
    "CFG-MAG": GRP_MAG,
    "CFG-FRAME": GRP_FRAME,
    "CFG-CALACC": GRP_CAL_ACC,
    "CFG-CALGYR": GRP_CAL_GYR,
    "CFG-CALMAG": GRP_CAL_MAG,
}

# The calibration groups are three identical sets of keys, so they are
# generated rather than spelled out three times.
CAL_GROUP_OF = {
    "CFG-CALACC": GRP_CAL_ACC,
    "CFG-CALGYR": GRP_CAL_GYR,
    "CFG-CALMAG": GRP_CAL_MAG,
}
for _name, _grp in CAL_GROUP_OF.items():
    CFG_KEYS["%s-NPTS" % _name] = cal_npts_key(_grp)
    CFG_KEYS["%s-BIASPOLY" % _name] = cal_biaspoly_key(_grp)
    for _n in range(CAL_PTS_MAX):
        CFG_KEYS["%s-POINT%d" % (_name, _n)] = cal_point_key(_grp, _n)
del _name, _grp, _n

CFG_KEY_NAMES = {v: k for k, v in CFG_KEYS.items()}

# Which triad a calibration group is, for the unit conversions below.
CAL_KIND_OF_GROUP = {GRP_CAL_ACC: "acc", GRP_CAL_GYR: "gyr", GRP_CAL_MAG: "mag"}


def cfg_key_name(key):
    """Human name of a key, or its hex id when it has none."""
    return CFG_KEY_NAMES.get(key, "0x%08X" % key)


def _encode_value(key, value):
    """Value bytes for the wire, from an int or a bytes-like record."""
    width = SZ_WIDTH.get(cfg_key_size(key))
    if width is None:
        blob = bytes(value)
        return struct.pack("<H", len(blob)) + blob
    if isinstance(value, (bytes, bytearray)):
        if len(value) != width:
            raise ValueError("key 0x%08X takes %d bytes, got %d"
                             % (key, width, len(value)))
        return bytes(value)
    return int(value).to_bytes(width, "little")


def build_valset(items, layers=LAYER_RAM):
    """0x40/0x08 from [(key, value), ...].

    Applied in order and stopped at the first failure, so the frame is
    not atomic: the acknowledgement says how many values went in."""
    payload = struct.pack("<BBBB", 0x01, layers, 0, 0)
    for key, value in items:
        payload += struct.pack("<I", key) + _encode_value(key, value)
    return ubx_frame(CLS_INSLIB, ID_VALSET, payload)


def build_valget(keys, layer=VALGET_RAM):
    """0x40/0x09. An item id of ITEM_WILDCARD asks for a whole group."""
    payload = struct.pack("<BBBB", 0x00, layer, 0, 0)
    for key in keys:
        payload += struct.pack("<I", key)
    return ubx_frame(CLS_INSLIB, ID_VALGET, payload)


def build_cfgreset(layers=LAYER_RAM):
    """0x40/0x0C. Without LAYER_FLASH only the live values are dropped
    and the stored image returns at the next power cycle."""
    return ubx_frame(CLS_INSLIB, ID_CFGRESET,
                     struct.pack("<BBBB", 0x01, layers, 0, 0))


def group_wildcard(group):
    """A key that asks a VALGET for every known item of one group."""
    return cfg_key(SZ_U1, group, ITEM_WILDCARD)


def parse_cfgack(payload):
    """0x40/0x0B -> dict, or None on a wrong length."""
    if len(payload) != 12:
        return None
    version, result, msg_id, _rsv, key, detail = struct.unpack("<BBBBII", payload)
    return {"version": version, "result": result, "msg_id": msg_id,
            "key": key, "detail": detail, "ok": result == CFG_OK,
            "text": CFG_RESULT_TEXT.get(result, "unknown result %d" % result)}


def parse_valget(payload):
    """0x40/0x0A -> dict, or None if malformed.

    Keys carrying no value are absent from the response rather than
    reported, so a caller learns what is set by comparing this with what
    it asked for (inslib_protocol.md).

    `more` is set while further frames of the same answer are still to
    come. Without it a response that happens to fill a frame would look
    exactly like the last one, and a wildcard over a calibration group
    fills several."""
    if len(payload) < 4:
        return None
    layer, more = payload[1], bool(payload[2])
    out, off, n = {}, 4, len(payload)
    while off < n:
        if n - off < 4:
            return None
        key, = struct.unpack("<I", payload[off:off + 4])
        off += 4
        width = SZ_WIDTH.get(cfg_key_size(key))
        if width is None:
            if n - off < 2:
                return None
            width, = struct.unpack("<H", payload[off:off + 2])
            off += 2
        if n - off < width:
            return None
        out[key] = bytes(payload[off:off + width])
        off += width
    return {"layer": layer, "more": more, "values": out}


# ... plus the IMU configuration from version 0x02 on: odr, reserved,
# accel/gyro full scale and low pass corner. Physical numbers rather than
# register codes, so no decode table has to be kept in step with the
# firmware, and at 1 Hz rather than on every sample, because they change
# at init and nowhere else.
CFGINFO_FMT = "<BBBBIIffBBBBIHHffff"
CFGINFO_LEN = struct.calcsize(CFGINFO_FMT)


def parse_cfginfo(payload):
    """0x40/0x07 -> dict, or None on a wrong length.

    cfg_crc identifies the calibration a recording was taken with, which
    is the whole reason to log this message."""
    if len(payload) != CFGINFO_LEN:
        return None
    (version, flags, cal_flags, _rsv, crc, seq, t_imu, t_mag,
     n_acc, n_gyr, n_mag, _rsv2, dropped,
     odr_hz, _rsv3, acc_fs_g, gyr_fs_dps,
     acc_lpf_hz, gyr_lpf_hz) = struct.unpack(CFGINFO_FMT, payload)
    return {
        "imu": {
            "odr_hz": odr_hz,
            # The full scales are the CLIP LEVEL: a reading at exactly
            # this value is a reading at the end stop, which is what
            # makes the saturation bits of 0x40/0x01 checkable against
            # the values beside them.
            "accel_fs_g": acc_fs_g,
            "gyro_fs_dps": gyr_fs_dps,
            "accel_lpf_hz": acc_lpf_hz,
            "gyro_lpf_hz": gyr_lpf_hz,
        },
        "version": version,
        "flags": flags,
        "cal_flags": cal_flags,
        "cfg_crc": crc,
        "cfg_seq": seq,
        "temp_imu_c": t_imu,
        "temp_mag_c": t_mag,
        "npts": {"acc": n_acc, "gyr": n_gyr, "mag": n_mag},
        "cmd_dropped": dropped,
        "stored": bool(flags & CFGINFO_STORED),
        "unsaved": bool(flags & CFGINFO_UNSAVED),
        "imu_cal_applied": bool(flags & CFGINFO_IMU_CAL),
        "mag_cal_applied": bool(flags & CFGINFO_MAG_CAL),
    }


def scalar_from_bytes(raw):
    """Little-endian integer out of a VALGET value, for a scalar key."""
    return int.from_bytes(raw, "little")


def pack_cal_point(temp_c, matrix_rowmajor, bias):
    """One calibration node record.

    matrix_rowmajor is the 3x3 the way the solver and config.yaml hold
    it, as nested rows; the wire wants it column major. bias must already
    be in the units of the stream, which is what
    cal_bias_to_stream_units() converts to."""
    m = [[float(v) for v in row] for row in matrix_rowmajor]
    if len(m) != 3 or any(len(row) != 3 for row in m):
        raise ValueError("matrix must be 3x3")
    colmajor = [m[r][c] for c in range(3) for r in range(3)]
    b = [float(v) for v in bias]
    if len(b) != 3:
        raise ValueError("bias must have 3 elements")
    return struct.pack("<13f", float(temp_c), *colmajor, *b)


def unpack_cal_point(raw):
    """Inverse of pack_cal_point -> (temp_c, row-major 3x3, bias)."""
    if len(raw) != CAL_POINT_LEN:
        return None
    v = struct.unpack("<13f", raw)
    temp_c, col, bias = v[0], v[1:10], list(v[10:13])
    rows = [[col[c * 3 + r] for c in range(3)] for r in range(3)]
    return temp_c, rows, bias


def pack_bias_poly(t_ref, deg, coeffs):
    """Bias polynomial record.

    coeffs is 3 axes x up to CAL_POLY_MAX, axis major, c[axis][k]
    multiplying (T - t_ref)**k. Unused coefficients are zero and deg says
    how many the device evaluates. Use no_bias_poly() to retire a fit
    rather than writing zeros, which would mean a bias of zero and
    override the node values with it."""
    if deg != CAL_POLY_NONE and not 0 <= deg < CAL_POLY_MAX:
        raise ValueError("degree must be 0..%d" % (CAL_POLY_MAX - 1))
    flat = []
    for axis in range(3):
        row = list(coeffs[axis])
        if len(row) > CAL_POLY_MAX:
            raise ValueError("at most %d coefficients per axis" % CAL_POLY_MAX)
        flat += [float(v) for v in row] + [0.0] * (CAL_POLY_MAX - len(row))
    return struct.pack("<fB3x12f", float(t_ref), int(deg), *flat)


def no_bias_poly():
    """The record that says no polynomial is fitted over these nodes.

    Written whenever the node table changes, because the fit that was
    made over the previous one no longer describes it and would keep
    overriding the node biases if left in place."""
    return pack_bias_poly(0.0, CAL_POLY_NONE, [[0.0]] * 3)


def unpack_bias_poly(raw):
    """Inverse of pack_bias_poly -> (t_ref, deg, 3 x CAL_POLY_MAX).

    A deg of CAL_POLY_NONE means the record is a retirement marker and
    the node biases are what the device uses."""
    if len(raw) != CAL_BIASPOLY_LEN:
        return None
    v = struct.unpack("<fB3x12f", raw)
    t_ref, deg, flat = v[0], v[1], v[2:]
    coeffs = [list(flat[a * CAL_POLY_MAX:(a + 1) * CAL_POLY_MAX])
              for a in range(3)]
    return t_ref, deg, coeffs


def pack_housing(matrix_rowmajor):
    """The mounting rotation record, 36 bytes.

    Held apart from the calibration nodes on purpose. Those are a table
    over temperature that the device interpolates, and a rotation folded
    into some nodes and not into others interpolates into a matrix that
    is no rotation at all -- the attitude would then turn with
    temperature. See cfg_keys.h, CFG-FRAME.

    Row-major in, column-major out, the same convention as
    pack_cal_point."""
    m = [[float(v) for v in row] for row in matrix_rowmajor]
    if len(m) != 3 or any(len(row) != 3 for row in m):
        raise ValueError("matrix must be 3x3")
    return struct.pack("<9f", *[m[r][c] for c in range(3) for r in range(3)])


def unpack_housing(raw):
    """Inverse of pack_housing -> row-major 3x3, or None on a bad length."""
    if len(raw) != HOUSING_LEN:
        return None
    col = struct.unpack("<9f", raw)
    return [[col[c * 3 + r] for c in range(3)] for r in range(3)]


def housing_from_rpy_deg(roll, pitch, yaw):
    """Tait-Bryan ZYX roll/pitch/yaw [deg] -> row-major 3x3.

    The exact inverse of housing_rpy_deg, so a rotation read from a board
    and written back unchanged produces the same nine numbers. Kept here
    beside its inverse rather than in the alignment solver: this one is
    needed to EDIT a stored mounting, which is a configuration job and
    not a measurement one, and it must not drift from the reader."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def housing_rpy_deg(rows):
    """ZYX Tait-Bryan roll/pitch/yaw [deg] of a mounting rotation.

    Same extraction as inslib_frame_align.dcm_to_rpy_deg and as the
    filter's ins_rotmat_to_rpy, repeated here only so that reading a
    board's settings does not pull numpy into a command line tool. The
    three numbers are how a person checks a rotation; the nine are not."""
    sp = max(-1.0, min(1.0, -float(rows[2][0])))
    pitch = math.asin(sp)
    if abs(sp) < 0.9999:
        roll = math.atan2(float(rows[2][1]), float(rows[2][2]))
        yaw = math.atan2(float(rows[1][0]), float(rows[0][0]))
    else:
        roll = math.atan2(-float(rows[1][2]), float(rows[1][1]))
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def cal_bias_to_stream_units(bias, kind):
    """Convert a solver bias into the units the device stores.

    The device corrects in the units of its own stream, so an
    accelerometer bias in m/s^2 becomes g and a gyroscope bias in rad/s
    becomes deg/s. The matrix is dimensionless and carries over
    unchanged. Getting this wrong is silent: the correction simply comes
    out about 9.8x or 57x too large."""
    if kind == "acc":
        return [float(b) / GRAVITY_NOMINAL for b in bias]
    if kind == "gyr":
        return [float(b) * RAD_TO_DEG for b in bias]
    if kind == "mag":
        return [float(b) for b in bias]
    raise ValueError("kind must be acc, gyr or mag")


def cal_bias_from_stream_units(bias, kind):
    """Inverse of cal_bias_to_stream_units, for reading a device back."""
    if kind == "acc":
        return [float(b) * GRAVITY_NOMINAL for b in bias]
    if kind == "gyr":
        return [float(b) / RAD_TO_DEG for b in bias]
    if kind == "mag":
        return [float(b) for b in bias]
    raise ValueError("kind must be acc, gyr or mag")
