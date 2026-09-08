#!/usr/bin/env python3
"""Tests for the configuration protocol layer in tools/inslib_ubx.py and
the command line tool tools/inslib_cfg.py.

Two different things are checked here, and it is worth being clear about
which is which.

The encoding half is checked against the firmware's own header: the key
scheme, the record layouts and the unit conversions have to agree with
embedded/stm32f429/Core/Inc/inslib/cfg_keys.h, and this file reads the
constants straight out of it rather than restating them. That is the one
place a silent mismatch would hurt, because a record of the wrong length
is refused loudly but a record of the right length with the fields in the
wrong order is not.

The tool half runs against a fake board written in Python. That fake is
not evidence about the firmware, which has its own test in
tests/test_cfg.c; it exists to exercise the parts of inslib_cfg.py that
only appear over a link - reassembling a multi-frame answer, reading past
the sensor stream the board keeps sending, and reporting a refusal.

Runs under pytest or standalone:

    python3 python/tests/test_cfg_protocol.py
"""

import os
import re
import struct
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inslib_ubx as ux       # noqa: E402
import inslib_cfg as cfgtool  # noqa: E402

# The upload path lives in the calibration window, which needs PyQt6 and
# pyqtgraph. Its node merge is plain arithmetic and worth testing even
# where those are not installed, so the import is optional and the tests
# that need it say when they are skipped rather than passing quietly.
try:
    import inslib_calib_gui as gui   # noqa: E402
except Exception:                    # noqa: BLE001
    gui = None


def _need_gui():
    if gui is not None:
        return True
    try:
        import pytest
        pytest.skip("PyQt6/pyqtgraph not installed")
    except ImportError:
        print("   (skipped: PyQt6/pyqtgraph not installed)")
    return False

CFG_KEYS_H = os.path.join(REPO, "embedded", "stm32f429", "Core", "Inc",
                          "inslib", "cfg_keys.h")
UBX_H = os.path.join(REPO, "embedded", "stm32f429", "Core", "Inc",
                     "inslib", "ubx.h")
APP_C = os.path.join(REPO, "embedded", "stm32f429", "Core", "Src",
                     "inslib_sensor", "app.c")


# ---------------------------------------------------------------------------
# Agreement with the firmware header
# ---------------------------------------------------------------------------

def _firmware_defines():
    """The #define constants of cfg_keys.h, as integers."""
    out = {}
    with open(CFG_KEYS_H, "r", encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r"\s*#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)U?\b", line)
            if m:
                out[m.group(1)] = int(m.group(2), 0)
    return out


def _firmware_key(name):
    """A key the header builds with the INSLIB_CFG_KEY macro, as an int.

    _firmware_defines only sees plain numeric defines, so the keys
    themselves -- which are the thing most worth pinning, since a wrong
    one writes into nothing -- would otherwise go unchecked."""
    fw = _firmware_defines()
    pat = re.compile(r"#define\s+" + re.escape(name) +
                     r"\s+INSLIB_CFG_KEY\((\w+),\s*(\w+),\s*(\w+)\)")
    with open(CFG_KEYS_H, "r", encoding="utf-8") as fh:
        m = pat.search(fh.read())
    assert m, "%s is not defined through INSLIB_CFG_KEY" % name

    def val(tok):
        return int(tok, 0) if re.match(r"^(0[xX])?[0-9A-Fa-f]+$", tok)             else fw[tok]

    return ux.cfg_key(val(m.group(1)), val(m.group(2)), val(m.group(3)))


def test_record_lengths_match_firmware():
    fw = _firmware_defines()
    assert ux.CAL_POINT_LEN == fw["INSLIB_CAL_POINT_LEN"]
    assert ux.CAL_BIASPOLY_LEN == fw["INSLIB_CAL_BIASPOLY_LEN"]
    assert ux.CAL_PTS_MAX == fw["INSLIB_CAL_PTS_MAX"]
    assert ux.CAL_POLY_MAX == fw["INSLIB_CAL_POLY_MAX"]
    # The packers have to produce exactly what the firmware reads.
    assert len(ux.pack_cal_point(0.0, [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                 [0, 0, 0])) == ux.CAL_POINT_LEN
    assert len(ux.pack_bias_poly(0.0, 0, [[0]] * 3)) == ux.CAL_BIASPOLY_LEN
    assert ux.HOUSING_LEN == fw["INSLIB_CAL_HOUSING_LEN"]
    assert len(ux.pack_housing([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))         == ux.HOUSING_LEN
    assert ux.LEVERARM_LEN == fw["INSLIB_CFG_LEVERARM_LEN"]
    assert len(ux.pack_leverarm([0.0, 0.0, 0.0])) == ux.LEVERARM_LEN
    assert ux.LEVERARM_MAX_M == fw["INSLIB_CFG_LEVERARM_MAX_M"]


def _header_defines(path):
    """The #define constants of a firmware header, as integers.

    Plain literals and the (1UL << n) form both, because a bit field is
    far more readable in the header written as a shift -- and a test that
    could only read literals would quietly skip exactly the constants
    whose bit position matters most."""
    out = {}
    lit = re.compile(r"\s*#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)U?L?")
    shift = re.compile(r"\s*#define\s+(\w+)\s+\(1U?L?\s*<<\s*(\d+)\s*\)")
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            m = lit.match(line)
            if m:
                out[m.group(1)] = int(m.group(2), 0)
                continue
            m = shift.match(line)
            if m:
                out[m.group(1)] = 1 << int(m.group(2))
    return out


def test_imu_status_bits_match_firmware():
    """The status word layout against ubx.h, not against a comment.

    Three flag bits sit directly above an eleven bit signed number. A
    layout that drifted by one would move the temperature under a flag,
    and every frame would still decode -- into plausible nonsense."""
    fw = _header_defines(UBX_H)
    assert ux.IMU_ST_TEMP_MASK == fw["UBX_IMU_ST_TEMP_MASK"]
    assert ux.IMU_ST_SAT_ACC == 1 << 11
    assert ux.IMU_ST_SAT_GYR == 1 << 12
    assert ux.IMU_ST_CAL_APPLIED == 1 << 13
    assert fw["UBX_IMU_ST_TEMP_BITS"] == 11
    # The message must not have changed length: the firmware writes into a
    # pl[38] and the tools all check IMU_LEN.
    assert ux.IMU_LEN == 38


def test_imu_status_round_trips():
    for t in (0.0, 25.3, -12.3, 85.0, -40.0, 102.3, -102.4):
        st = ux.build_imu_status(t)
        assert abs(ux.imu_status_temp_c(st) - t) < 0.05, t
    # Flags do not disturb the temperature and vice versa.
    st = ux.build_imu_status(-12.3, sat_acc=True, cal_applied=True)
    got = ux.decode_imu_status(st)
    assert abs(got["temp_c"] + 12.3) < 0.05
    assert got["sat_acc"] and not got["sat_gyr"]
    assert got["saturated"] and got["cal_applied"]
    # Reserved bits must be ignored, not rejected -- that is what lets a
    # field be added later without breaking this decoder.
    got = ux.decode_imu_status(st | 0xFFFFC000)
    assert abs(got["temp_c"] + 12.3) < 0.05
    assert got["sat_acc"] and got["cal_applied"]
    # Out of range clamps rather than wrapping into the wrong sign.
    assert ux.imu_status_temp_c(ux.build_imu_status(500.0)) > 100.0
    assert ux.imu_status_temp_c(ux.build_imu_status(-500.0)) < -100.0


def test_cfginfo_carries_the_imu_configuration():
    """0x40/0x07 gained the range, bandwidth and rate at version 2.

    They are numbers rather than register codes on purpose: a code needs
    a decode table here AND in the firmware, and a table that drifts
    yields plausible wrong data instead of an error."""
    fw = _header_defines(os.path.join(REPO, "embedded", "stm32f429", "Core",
                                      "Inc", "inslib", "cfg_ubx.h"))
    assert ux.CFGINFO_LEN == fw["INSLIB_CFGINFO_LEN"]
    raw = struct.pack(ux.CFGINFO_FMT, 2, 0x05, 0x07, 0, 0xDEADBEEF, 7,
                      31.5, 29.0, 3, 3, 2, 0, 0,
                      800, 0, 8.0, 2000.0, 50.0, 50.0)
    got = ux.parse_cfginfo(raw)
    assert got["version"] == 2
    assert got["imu"] == {"odr_hz": 800, "accel_fs_g": 8.0,
                          "gyro_fs_dps": 2000.0,
                          "accel_lpf_hz": 50.0, "gyro_lpf_hz": 50.0}
    # The fields that were there before must not have moved.
    assert got["cfg_crc"] == 0xDEADBEEF and got["npts"]["acc"] == 3


def test_nav_layout_matches_firmware():
    fw = _header_defines(UBX_H)
    assert ux.ID_NAV == fw["UBX_MSG_NAV"]
    assert ux.NAV_LEN == fw["UBX_NAV_LEN"]
    # The flag bits decide which fields are believed, so a shift by one
    # would hand back a position the filter does not have.
    for name, bit in (("ready", "READY"), ("rpy", "RPY"), ("vel", "VEL"),
                      ("pos", "POS"), ("latlon", "LATLON"),
                      ("height_ell", "HEIGHT_ELL"), ("leverarm", "LEVERARM"),
                      ("leverarm_bad", "LEVERARM_BAD"), ("zupt", "ZUPT"),
                      ("vzupt", "VZUPT"), ("mag_ref", "MAG_REF")):
        assert ux.NAV_F[name] == fw["UBX_NAV_F_%s" % bit], name


def test_nav_absent_values_decode_as_none():
    """A missing value must come back as None, never as the bytes.

    Zero, NaN and a stale number all read as an answer; only None reads
    as "the filter does not have this"."""
    body = struct.pack(ux.NAV_FMT,
                       1, 1, 2, 0x07, 0x00, 30,          # header bytes
                       803, ux.NAV_F["ready"] | ux.NAV_F["rpy"], 752,
                       1.5, -2.5, 213.0, 0.1, 0.1, 0.9,  # rpy and sigmas
                       9.0, 9.0, 9.0, 8.0, 8.0, 8.0,     # vel, pos (invalid)
                       7.0, 7.0, 7.0, 7.0, 7.0,          # heights, vz
                       0.0, 0.0, 0.0,                    # lever arm
                       48.9, 8.4,                        # lat, lon
                       48.6, 3.2)                        # mag ref
    d = ux.parse_nav(body)
    assert d["rpy_deg"] == (1.5, -2.5, 213.0)
    assert d["ready"] is True and d["mode"] == "attitude"
    assert d["att_src"] == "ahrs"
    for absent in ("vel_ned", "pos_ned", "lat_deg", "lon_deg", "height_m",
                   "height_ell_m", "baro_alt_m", "vz_mps",
                   "mag_ref_uT", "mag_decl_deg"):
        assert d[absent] is None, absent
    # rpy_sigma is not set either, even though rpy is.
    assert d["rpy_sigma_deg"] is None and d["yaw_sigma_deg"] is None
    assert ux.parse_nav(body + b"x") is None


def test_message_ids_match_firmware():
    fw = _header_defines(UBX_H)
    assert ux.ID_IMUHEALTH == fw["UBX_MSG_IMUHEALTH"]
    assert ux.ID_SATUR == fw["UBX_MSG_SATUR"]
    assert ux.SATUR_LEN == fw["UBX_SATUR_LEN"]


def test_imuhealth_decoder_knows_every_field_the_firmware_sends():
    """The decoder's field list against app.c's HEALTH_FIELDS.

    This message has already shipped once with more fields than its
    encoder buffer held, which dropped the last one silently. A decoder
    that knows fewer names than the board sends would hide the same class
    of mismatch again, from the other end."""
    with open(APP_C, "r", encoding="utf-8") as fh:
        m = re.search(r"enum\s*\{\s*HEALTH_FIELDS\s*=\s*(\d+)\s*\}", fh.read())
    assert m, "app.c no longer declares HEALTH_FIELDS the way this test reads it"
    sent = int(m.group(1))
    assert len(ux.IMUHEALTH_FIELDS) == sent, (
        "the board sends %d health fields, the decoder names %d"
        % (sent, len(ux.IMUHEALTH_FIELDS)))
    # And they have to fit the encoder, which clamps rather than errors.
    assert sent <= _header_defines(UBX_H)["UBX_IMUHEALTH_MAX_FIELDS"]


def test_imuhealth_decodes_short_and_long_payloads():
    words = list(range(len(ux.IMUHEALTH_FIELDS)))
    got = ux.parse_imuhealth(struct.pack("<%dI" % len(words), *words))
    assert got["stalls"] == 0
    assert got["reg_read_rc"] == 16
    assert got["sat_sticky"] == 17 and got["sat_samples"] == 18
    # An older firmware sends fewer: the fields it carried still decode.
    short = ux.parse_imuhealth(struct.pack("<4I", 1, 2, 3, 4))
    assert short == {"stalls": 1, "reinits": 2, "reinit_fails": 3, "spi_errors": 4}
    # A newer one sends more: the surplus is kept rather than dropped.
    longer = ux.parse_imuhealth(struct.pack("<%dI" % (len(words) + 1), *(words + [99])))
    assert longer["extra"] == [99]
    assert ux.parse_imuhealth(bytes(3)) is None


def test_satur_layout_and_axis_names():
    # Three u16 in a row transpose without changing the length, so the
    # order is the whole content of this message.
    raw = struct.pack(ux.SATUR_FMT, 0x1234, 0x5678, 0x009A, 0x2A, 0x01)
    ep = ux.parse_satur(raw)
    assert (ep["first_seq"], ep["last_seq"], ep["samples"]) == (0x1234, 0x5678, 0x9A)
    # 0x2A = bits 1, 3, 5 -> acc_y, gyr_x, gyr_z
    assert ep["axes"] == ["acc_y", "gyr_x", "gyr_z"]
    assert ep["dropped_before"] is True
    assert ux.parse_satur(raw + bytes(1)) is None


def test_satur_axis_bits_match_the_firmware_driver():
    """The mask bit order against imu_saturation.h, not against a comment.

    A mask whose bits are shuffled decodes without complaint and blames
    the wrong axis, which is worse than not decoding at all."""
    hdr = os.path.join(REPO, "embedded", "stm32f429", "Core", "Inc",
                       "inslib", "imu_saturation.h")
    fw = _header_defines(hdr)
    for i, name in enumerate(ux.SAT_AXES):
        assert fw["INSLIB_SAT_%s" % name.upper()] == 1 << i
    assert fw["INSLIB_SAT_EP_DROPPED"] == ux.SAT_EP_DROPPED


def test_groups_and_sizes_match_firmware():
    fw = _firmware_defines()
    assert ux.GRP_CAL_ACC == fw["INSLIB_CFG_GRP_CAL_ACC"]
    assert ux.GRP_CAL_GYR == fw["INSLIB_CFG_GRP_CAL_GYR"]
    assert ux.GRP_CAL_MAG == fw["INSLIB_CFG_GRP_CAL_MAG"]
    assert ux.GRP_MSGOUT == fw["INSLIB_CFG_GRP_MSGOUT"]
    assert ux.GRP_FRAME == fw["INSLIB_CFG_GRP_FRAME"]
    assert ux.housing_key() == _firmware_key("INSLIB_CFG_FRAME_HOUSING")
    assert ux.leverarm_key() == _firmware_key("INSLIB_CFG_FRAME_LEVERARM")
    assert ux.SZ_BLOB == fw["INSLIB_CFG_SZ_BLOB"]
    assert ux.SZ_U1 == fw["INSLIB_CFG_SZ_U1"]
    assert ux.SZ_U2 == fw["INSLIB_CFG_SZ_U2"]
    assert ux.ITEM_WILDCARD == fw["INSLIB_CFG_ITEM_WILDCARD"]


def test_key_encoding():
    key = ux.CFG_KEYS["CFG-CALACC-POINT3"]
    assert ux.cfg_key_size(key) == ux.SZ_BLOB
    assert ux.cfg_key_group(key) == ux.GRP_CAL_ACC
    assert ux.cfg_key_item(key) == 0x103
    assert ux.cfg_key_name(key) == "CFG-CALACC-POINT3"
    # Every name maps back to itself, so no two keys collide.
    assert len(ux.CFG_KEY_NAMES) == len(ux.CFG_KEYS)


def test_cal_point_is_column_major():
    """The wire is column major, the solver and config.yaml are not.

    Pinned with an asymmetric matrix, because the identity and any
    symmetric matrix survive the wrong convention untouched."""
    rows = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    raw = ux.pack_cal_point(25.0, rows, [0.1, 0.2, 0.3])
    v = struct.unpack("<13f", raw)
    assert v[0] == 25.0
    assert list(v[1:10]) == [1, 4, 7, 2, 5, 8, 3, 6, 9]
    assert [round(x, 6) for x in v[10:13]] == [0.1, 0.2, 0.3]

    temp_c, back, bias = ux.unpack_cal_point(raw)
    assert temp_c == 25.0
    assert back == [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    assert [round(b, 6) for b in bias] == [0.1, 0.2, 0.3]


def test_bias_poly_is_axis_major():
    coeffs = [[1.0, 2.0], [3.0], [4.0, 5.0, 6.0]]
    raw = ux.pack_bias_poly(20.0, 2, coeffs)
    t_ref, deg, back = ux.unpack_bias_poly(raw)
    assert (t_ref, deg) == (20.0, 2)
    assert back[0][:2] == [1.0, 2.0] and back[0][2:] == [0.0, 0.0]
    assert back[1][0] == 3.0 and back[1][1:] == [0.0, 0.0, 0.0]
    assert back[2][:3] == [4.0, 5.0, 6.0]


def test_bias_unit_conversion():
    """The device stores stream units, the solver works in SI.

    An accelerometer bias of one g must land as 1.0 and not as 9.80665,
    and the conversion must use the nominal gravity the firmware maps
    with rather than any local value."""
    assert ux.cal_bias_to_stream_units([ux.GRAVITY_NOMINAL, 0, 0], "acc")[0] == 1.0
    assert abs(ux.cal_bias_to_stream_units([1.0, 0, 0], "gyr")[0] - 57.29577951) < 1e-6
    assert ux.cal_bias_to_stream_units([12.5, 0, 0], "mag")[0] == 12.5
    for kind, value in (("acc", 0.37), ("gyr", -0.002), ("mag", 4.25)):
        there = ux.cal_bias_to_stream_units([value, 0, 0], kind)
        back = ux.cal_bias_from_stream_units(there, kind)
        assert abs(back[0] - value) < 1e-12


def test_valset_frame_layout():
    key = ux.CFG_KEYS["CFG-RATE-MAG_MS"]
    frame = ux.build_valset([(key, 250)], ux.LAYER_RAM | ux.LAYER_FLASH)
    assert frame[:2] == ux.UBX_SYNC
    assert frame[2] == ux.CLS_INSLIB and frame[3] == ux.ID_VALSET
    assert ux.ubx_checksum(frame[2:-2]) == (frame[-2], frame[-1])
    payload = frame[6:-2]
    assert payload[0] == 0x01
    assert payload[1] == ux.LAYER_RAM | ux.LAYER_FLASH
    assert struct.unpack("<I", payload[4:8])[0] == key
    assert struct.unpack("<H", payload[8:10])[0] == 250

    # A record key carries its own length, a scalar does not.
    blob = ux.pack_cal_point(0.0, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
    frame = ux.build_valset([(ux.CFG_KEYS["CFG-CALACC-POINT0"], blob)])
    payload = frame[6:-2]
    assert struct.unpack("<H", payload[8:10])[0] == ux.CAL_POINT_LEN
    assert payload[10:] == blob


# ---------------------------------------------------------------------------
# A fake board, for the command line tool
# ---------------------------------------------------------------------------

class FakeBoard:
    """Enough of the device to drive inslib_cfg.py over.

    Answers the three host messages and, between answers, keeps pushing
    IMU frames the way the real board does. The tool has to read past
    those to find its reply, and that is the point of them being here."""

    MAX_RESPONSE_VALUES = 3     # small, to force a multi-frame answer

    def __init__(self, values=None, refuse=None):
        self.values = dict(values or {})
        self.refuse = refuse            # a result code to answer VALSET with
        self.rx = bytearray()
        self.tx = bytearray()
        self.framer = ux.UbxFramer()
        self.saved = False
        self.reset_calls = []

    # --- the pyserial surface the tool uses ---
    @property
    def in_waiting(self):
        self._noise()
        return len(self.tx)

    def read(self, n):
        self._noise()
        out, self.tx = bytes(self.tx[:n]), self.tx[n:]
        return out

    def write(self, data):
        self.rx += data
        for cls, mid, payload, _f in self.framer.feed(bytes(data)):
            if cls == ux.CLS_INSLIB:
                self._handle(mid, payload)
        self.rx.clear()
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.tx.clear()

    # --- device behaviour ---
    def _noise(self):
        """One IMU frame, so the link is never quiet."""
        if len(self.tx) < 64:
            self.tx += ux.ubx_frame(
                ux.CLS_INSLIB, ux.ID_IMU,
                struct.pack(ux.IMU_FMT, 1, 0, 0, -1.0, 0, 0, 0,
                            ux.build_imu_status(25.0), 1))

    def _ack(self, msg_id, result, key=0, detail=0):
        self.tx += ux.ubx_frame(
            ux.CLS_INSLIB, ux.ID_CFGACK,
            struct.pack("<BBBBII", 0x01, result, msg_id, 0, key, detail))

    def _handle(self, mid, payload):
        if mid == ux.ID_VALSET:
            self._handle_valset(payload)
        elif mid == ux.ID_VALGET:
            self._handle_valget(payload)
        elif mid == ux.ID_CFGRESET:
            self.reset_calls.append(payload[1])
            self.values.clear()
            self._ack(ux.ID_CFGRESET, ux.CFG_OK)

    def _handle_valset(self, payload):
        if self.refuse is not None:
            self._ack(ux.ID_VALSET, self.refuse, key=0, detail=0)
            return
        layers, off, n_applied = payload[1], 4, 0
        while off < len(payload):
            key, = struct.unpack("<I", payload[off:off + 4])
            off += 4
            width = ux.SZ_WIDTH.get(ux.cfg_key_size(key))
            if width is None:
                width, = struct.unpack("<H", payload[off:off + 2])
                off += 2
            self.values[key] = bytes(payload[off:off + width])
            off += width
            n_applied += 1
        if layers & ux.LAYER_FLASH:
            self.saved = True
        self._ack(ux.ID_VALSET, ux.CFG_OK, detail=n_applied)

    def _expand(self, key):
        if ux.cfg_key_item(key) != ux.ITEM_WILDCARD:
            return [key]
        group = ux.cfg_key_group(key)
        return [k for k in sorted(self.values) if ux.cfg_key_group(k) == group]

    def _handle_valget(self, payload):
        layer = payload[1]
        wanted = []
        for off in range(4, len(payload), 4):
            wanted += self._expand(struct.unpack("<I", payload[off:off + 4])[0])
        present = [k for k in wanted if k in self.values]

        chunks = [present[i:i + self.MAX_RESPONSE_VALUES]
                  for i in range(0, len(present), self.MAX_RESPONSE_VALUES)] or [[]]
        for i, chunk in enumerate(chunks):
            more = 1 if i + 1 < len(chunks) else 0
            body = struct.pack("<BBBB", 0x01, layer, more, 0)
            for key in chunk:
                raw = self.values[key]
                body += struct.pack("<I", key)
                if ux.SZ_WIDTH.get(ux.cfg_key_size(key)) is None:
                    body += struct.pack("<H", len(raw))
                body += raw
            self.tx += ux.ubx_frame(ux.CLS_INSLIB, ux.ID_VALGET_R, body)

    def push_info(self, **over):
        f = dict(flags=ux.CFGINFO_STORED | ux.CFGINFO_IMU_CAL, cal_flags=0x03,
                 crc=0xDEADBEEF, seq=7, t_imu=31.5, t_mag=29.0,
                 n_acc=2, n_gyr=2, n_mag=0, dropped=0)
        f.update(over)
        self.tx += ux.ubx_frame(ux.CLS_INSLIB, ux.ID_CFGINFO, struct.pack(
            ux.CFGINFO_FMT, 0x02, f["flags"], f["cal_flags"], 0, f["crc"],
            f["seq"], f["t_imu"], f["t_mag"], f["n_acc"], f["n_gyr"],
            f["n_mag"], 0, f["dropped"],
            800, 0, 8.0, 2000.0, 50.0, 50.0))


def _run(board, argv):
    return cfgtool.main(argv, link=cfgtool.CfgLink(board))


# ---------------------------------------------------------------------------
# The tool over that fake board
# ---------------------------------------------------------------------------

def test_set_writes_the_key(capsys):
    board = FakeBoard()
    assert _run(board, ["set", "CFG-IMU-APPLY_CAL=0"]) == 0
    assert board.values[ux.CFG_KEYS["CFG-IMU-APPLY_CAL"]] == b"\x00"
    assert board.saved is False
    out = capsys.readouterr().out
    assert "gone at the next power cycle" in out


def test_leverarm_round_trips_through_the_record():
    # The axis order is the thing worth pinning: a record of the right
    # length with the axes swapped is accepted by the board and quietly
    # wrong in the solution.
    raw = ux.pack_leverarm([0.2, 0.6, -0.5])
    assert len(raw) == ux.LEVERARM_LEN
    assert [round(v, 6) for v in ux.unpack_leverarm(raw)] == [0.2, 0.6, -0.5]


def test_leverarm_accepts_an_airframe_sized_arm():
    """The bound is a units guard, not a description of a vehicle.

    A tail-fin antenna against an IMU at the centre of gravity is tens of
    metres on a large aircraft. A bound that refused those would be worse
    than none: the arm falls back to zero and the aircraft flies with the
    error the setting exists to remove."""
    for arm in ([30.0, 0.0, -4.0], [0.0, 0.0, -0.12], [-2.5, 1.25, -0.8]):
        assert [round(v, 6) for v in
                ux.unpack_leverarm(ux.pack_leverarm(arm))] == arm


def test_leverarm_refuses_a_units_mistake():
    # Metres typed as millimetres is the mistake worth catching, and the
    # message has to say which unit it wanted.
    try:
        ux.pack_leverarm([1200.0, 0.0, 0.0])
    except ValueError as exc:
        assert "METRES" in str(exc)
    else:
        raise AssertionError("1200 m on one axis was accepted")
    for bad in ([float("nan"), 0, 0], [0, float("inf"), 0]):
        try:
            ux.pack_leverarm(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("a non-finite lever arm was accepted")


def test_leverarm_writes_one_record_not_three_keys(capsys):
    board = FakeBoard()
    assert _run(board, ["leverarm", "0.2", "0.6", "-0.5"]) == 0
    assert list(board.values) == [ux.leverarm_key()]
    assert [round(v, 6) for v in
            ux.unpack_leverarm(board.values[ux.leverarm_key()])] == [0.2, 0.6, -0.5]
    assert board.saved is False
    assert "+0.600" in capsys.readouterr().out


def test_leverarm_unset_says_so_rather_than_printing_zeros(capsys):
    board = FakeBoard()
    assert _run(board, ["leverarm"]) == 0
    out = capsys.readouterr().out
    assert "unset" in out
    assert "0.000" not in out


def test_set_with_flash_persists(capsys):
    board = FakeBoard()
    assert _run(board, ["set", "CFG-RATE-MAG_MS=50", "--flash"]) == 0
    assert board.values[ux.CFG_KEYS["CFG-RATE-MAG_MS"]] == struct.pack("<H", 50)
    assert board.saved is True
    assert "stored" in capsys.readouterr().out


def test_set_accepts_spelled_out_flags():
    board = FakeBoard()
    _run(board, ["set", "CFG-MSGOUT-MAG=off"])
    assert board.values[ux.CFG_KEYS["CFG-MSGOUT-MAG"]] == b"\x00"
    _run(board, ["set", "CFG-MSGOUT-MAG=on"])
    assert board.values[ux.CFG_KEYS["CFG-MSGOUT-MAG"]] == b"\x01"


def test_set_reports_a_refusal(capsys):
    board = FakeBoard(refuse=ux.CFG_E_RANGE)
    assert _run(board, ["set", "CFG-RATE-MAG_MS=1"]) == 1
    assert "refused" in capsys.readouterr().out


def test_set_refuses_a_record_key(capsys):
    """A calibration record cannot be typed at a shell, and saying so is
    more use than a struct error further down."""
    board = FakeBoard()
    try:
        _run(board, ["set", "CFG-CALACC-POINT0=1"])
    except SystemExit as e:
        assert "calib_gui" in str(e)
    else:
        raise AssertionError("a record key must not be settable from here")


def test_get_reassembles_a_multi_frame_answer(capsys):
    """The board answers a wildcard in several frames, and the tool has
    to keep reading until the last one says so."""
    values = {}
    for n in range(5):
        values[ux.cal_point_key(ux.GRP_CAL_ACC, n)] = ux.pack_cal_point(
            10.0 * n, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.01 * n, 0, 0])
    values[ux.cal_npts_key(ux.GRP_CAL_ACC)] = b"\x05"
    board = FakeBoard(values)
    assert board.MAX_RESPONSE_VALUES < len(values)   # so it really splits

    assert _run(board, ["get", "CFG-CALACC"]) == 0
    out = capsys.readouterr().out
    for n in range(5):
        assert "CFG-CALACC-POINT%d" % n in out
    assert "CFG-CALACC-NPTS" in out


def test_get_decodes_a_calibration_node(capsys):
    key = ux.cal_point_key(ux.GRP_CAL_GYR, 0)
    board = FakeBoard({key: ux.pack_cal_point(
        42.5, [[1.5, 0, 0], [0, 1, 0], [0, 0, 1]], [0.25, 0, 0])})
    assert _run(board, ["get", "CFG-CALGYR-POINT0"]) == 0
    out = capsys.readouterr().out
    assert "42.50 degC" in out
    assert "+1.500000" in out            # the matrix, not hex
    assert "+0.250000" in out            # the bias
    assert "deg/s" in out                # in the unit the device stores


def test_get_renders_a_retired_polynomial_as_retired(capsys):
    """The marker is the normal state after a node was added.

    A record can be replaced but never removed, so a table whose fit has
    been retired carries a degree marker rather than losing the key.
    Printing that marker as a degree makes a healthy table look like a
    corrupt one."""
    key = ux.cal_biaspoly_key(ux.GRP_CAL_ACC)
    board = FakeBoard({key: ux.no_bias_poly()})
    assert _run(board, ["get", "CFG-CALACC-BIASPOLY"]) == 0
    out = capsys.readouterr().out
    assert "none fitted" in out
    assert "degree 255" not in out
    assert "dT^" not in out


def test_get_reports_an_empty_answer(capsys):
    board = FakeBoard()
    assert _run(board, ["get", "CFG-CALMAG"]) == 0
    assert "nothing set" in capsys.readouterr().out


def test_reset_passes_the_layers(capsys):
    board = FakeBoard({ux.CFG_KEYS["CFG-RATE-MAG_MS"]: b"\x32\x00"})
    assert _run(board, ["reset"]) == 0
    assert board.reset_calls == [ux.LAYER_RAM]
    assert _run(board, ["reset", "--flash"]) == 0
    assert board.reset_calls[-1] == ux.LAYER_RAM | ux.LAYER_FLASH
    assert "calibration is gone" in capsys.readouterr().out


def test_info_renders_the_state(capsys):
    board = FakeBoard()
    board.push_info(flags=ux.CFGINFO_STORED | ux.CFGINFO_UNSAVED,
                    cal_flags=ux.CAL_ACC_VALID | ux.CAL_ACC_CLAMPED)
    assert _run(board, ["info"]) == 0
    out = capsys.readouterr().out
    assert "UNSAVED CHANGES" in out
    assert "OUTSIDE the calibrated range" in out
    assert "sequence 7" in out


def test_info_separates_a_valid_table_from_the_apply_switch(capsys):
    """A stored table with APPLY_CAL off corrects nothing.

    Reporting the switch and the table on separate lines let that read as
    a working calibration, which is how a board ends up streaming raw
    magnetometer samples with a calibration sitting in its flash."""
    board = FakeBoard()
    board.push_info(flags=ux.CFGINFO_STORED | ux.CFGINFO_IMU_CAL,
                    cal_flags=ux.CAL_MAG_VALID, n_acc=0, n_gyr=0, n_mag=2)
    assert _run(board, ["info"]) == 0
    out = capsys.readouterr().out
    assert "mag off" in out
    assert "NOT applied" in out


def test_info_says_where_the_calibration_went_after_a_reset(capsys):
    """An emptied live image over an untouched store.

    That is what `reset` without --flash leaves behind, and without the
    note it reads as a board whose calibration was deleted."""
    board = FakeBoard()
    board.push_info(flags=ux.CFGINFO_STORED | ux.CFGINFO_UNSAVED,
                    cal_flags=0, crc=0, n_acc=0, n_gyr=0, n_mag=0)
    assert _run(board, ["info"]) == 0
    out = capsys.readouterr().out
    assert "loaded back at the next power cycle" in out
    assert "reset --flash" in out


def test_reset_warns_that_a_later_flash_write_persists_the_empty_image(capsys):
    """The store keeps its sequence and the live image is marked dirty,
    so the next persisted write of anything at all saves the emptied
    image over the calibration."""
    board = FakeBoard()
    assert _run(board, ["reset"]) == 0
    out = capsys.readouterr().out
    assert "NOT touched" in out
    assert "OVER the calibration" in out


def test_bare_invocation_prints_the_full_help(capsys):
    """No subcommand has to answer with the help, not a usage line.

    argparse's default for a required subparser is one line of usage and
    an exit, which is the least useful thing this tool can say to someone
    who typed its name to find out what it does."""
    assert cfgtool.main([]) == 0
    out = capsys.readouterr().out
    assert "Examples:" in out
    assert "reset --flash" in out           # the destructive one, spelled out
    assert "LIVE" in out and "STORED" in out


def test_unknown_name_is_refused():
    board = FakeBoard()
    try:
        _run(board, ["get", "CFG-NOPE"])
    except SystemExit as e:
        assert "unknown key or group" in str(e)
    else:
        raise AssertionError("an unknown name must not be accepted")


# ---------------------------------------------------------------------------
# The node merge behind the GUI's upload
# ---------------------------------------------------------------------------

def _node(temp_c, scale=1.0, bias_x=0.0):
    return ux.pack_cal_point(temp_c,
                             [[scale, 0, 0], [0, scale, 0], [0, 0, scale]],
                             [bias_x, 0, 0])


def _temps(nodes):
    return [round(ux.unpack_cal_point(r)[0], 3) for r in nodes]


def test_merge_into_an_empty_table():
    if not _need_gui():
        return
    out, what = gui.UploadWorker._merge([], _node(25.0), 25.0)
    assert _temps(out) == [25.0]
    assert "inserted" in what


def test_merge_keeps_the_table_in_temperature_order():
    if not _need_gui():
        return
    nodes = [_node(0.0), _node(40.0)]
    out, _ = gui.UploadWorker._merge(nodes, _node(20.0), 20.0)
    assert _temps(out) == [0.0, 20.0, 40.0]

    out, _ = gui.UploadWorker._merge(nodes, _node(-10.0), -10.0)
    assert _temps(out) == [-10.0, 0.0, 40.0]

    out, _ = gui.UploadWorker._merge(nodes, _node(70.0), 70.0)
    assert _temps(out) == [0.0, 40.0, 70.0]


def test_merge_replaces_a_node_at_the_same_operating_point():
    """The same temperature measured twice is a better measurement, not
    a second node: two nodes a fraction of a degree apart would make the
    interpolation between them meaningless."""
    if not _need_gui():
        return
    nodes = [_node(0.0), _node(25.3, scale=1.0), _node(40.0)]
    out, what = gui.UploadWorker._merge(nodes, _node(25.0, scale=2.0), 25.0)
    assert _temps(out) == [0.0, 25.0, 40.0]
    assert "replaced node 1" in what
    assert ux.unpack_cal_point(out[1])[1][0][0] == 2.0   # the new matrix won


def test_merge_refuses_to_overflow_the_table():
    if not _need_gui():
        return
    nodes = [_node(float(5 * i)) for i in range(ux.CAL_PTS_MAX)]
    try:
        gui.UploadWorker._merge(nodes, _node(-30.0), -30.0)
    except RuntimeError as e:
        assert "maximum" in str(e)
    else:
        raise AssertionError("a 17th node must not be accepted")


class _FakeBoardLink:
    """The cfg_send/cfg_take surface UploadWorker drives, over an empty
    board that acknowledges everything."""

    def __init__(self):
        self.written = {}
        self._out = []

    def can_configure(self):
        return True

    def cfg_clear(self):
        self._out = []

    def cfg_take(self):
        out, self._out = self._out, []
        return out

    def cfg_send(self, frame):
        mid, payload = frame[3], frame[6:-2]
        if mid == ux.ID_VALGET:
            self._out.append((ux.ID_VALGET_R, bytes([1, 0, 0, 0])))
        elif mid == ux.ID_VALSET:
            got = ux.parse_valget(bytes(4) + payload[4:])
            self.written.update(got["values"])
            self._out.append((ux.ID_CFGACK, struct.pack(
                "<BBBBII", 1, 0, ux.ID_VALSET, 0, 0, 1)))


def _upload(groups, housing):
    board = _FakeBoardLink()
    gui.UploadWorker(board, groups, persist=False, housing=housing).run()
    return board.written


def test_the_housing_rotation_is_its_own_key_not_part_of_the_nodes():
    """The board interpolates its node matrices over temperature.

    A rotation folded into some nodes and not into others interpolates
    into a matrix that is no rotation: the attitude turns with
    temperature and the scale goes with it. So the node goes up in the
    sensor frame and the rotation goes up beside it."""
    if not _need_gui():
        return
    m = [[1.01, 0.0, 0.0], [0.0, 0.99, 0.0], [0.0, 0.0, 1.0]]
    r = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]   # 90 deg yaw
    written = _upload({"acc": (m, [0.0, 0.0, 0.0], 25.0)}, r)

    node = written[ux.cal_point_key(ux.GRP_CAL_ACC, 0)]
    _temp, rows, _bias = ux.unpack_cal_point(node)
    assert abs(rows[0][0] - 1.01) < 1e-6, "the node was rotated on the way up"
    assert abs(rows[1][1] - 0.99) < 1e-6

    back = ux.unpack_housing(written[ux.housing_key()])
    assert [round(v, 3) for v in ux.housing_rpy_deg(back)] == [0.0, -0.0, 90.0]


def test_a_housing_pass_on_its_own_writes_one_key_and_no_nodes():
    """No temperature node behind it, and none needed.

    That is the whole point of holding the rotation apart: measuring
    where the box sits no longer means recording a calibration to carry
    it."""
    if not _need_gui():
        return
    r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    written = _upload({}, r)
    assert list(written) == [ux.housing_key()]


def test_an_upload_without_a_housing_pass_leaves_the_board_rotation_alone():
    """A node measured next winter must not clear the mounting.

    Absent means "nothing to say about it", not "identity": overwriting
    it with identity would silently undo a rotation nobody re-measured."""
    if not _need_gui():
        return
    m = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    written = _upload({"acc": (m, [0.0, 0.0, 0.0], 25.0)}, None)
    assert ux.housing_key() not in written


def test_retired_polynomial_is_not_zero_coefficients():
    """Zeros would mean a bias of zero and override the node values.

    The marker has to be a degree the firmware recognises as "no fit",
    which is the one thing this record has to get right."""
    raw = ux.no_bias_poly()
    assert len(raw) == ux.CAL_BIASPOLY_LEN
    _t_ref, deg, _coeffs = ux.unpack_bias_poly(raw)
    assert deg == ux.CAL_POLY_NONE
    assert deg >= ux.CAL_POLY_MAX          # so it can never read as a degree
    fw = _firmware_defines()
    assert ux.CAL_POLY_NONE == fw["INSLIB_CAL_POLY_NONE"]


# ---------------------------------------------------------------------------
# Standalone runner, for a checkout without pytest
# ---------------------------------------------------------------------------

class _Capsys:
    """The sliver of pytest's capsys the tests above use."""

    class _Out:
        def __init__(self, text):
            self.out = text

    def __init__(self):
        import io
        self._buf = io.StringIO()
        self._saved = None

    def __enter__(self):
        self._saved = sys.stdout
        sys.stdout = self._buf
        return self

    def __exit__(self, *_a):
        sys.stdout = self._saved
        return False

    def readouterr(self):
        text = self._buf.getvalue()
        self._buf.seek(0)
        self._buf.truncate(0)
        return self._Out(text)


def _main():
    fails = []
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        cap = _Capsys()
        try:
            with cap:
                if fn.__code__.co_argcount:
                    fn(cap)
                else:
                    fn()
        except Exception as e:                     # noqa: BLE001
            fails.append(name)
            print("FAIL %s: %s" % (name, e))
        else:
            print("ok   %s" % name)
    if fails:
        print("failed: %s" % ", ".join(fails))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
