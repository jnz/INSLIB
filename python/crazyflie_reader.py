#!/usr/bin/env python3
"""Live Crazyflie front-end for INSLIB.

Subscribes to a Crazyflie's onboard telemetry over the Crazyradio, feeds
it through the INSLIB navigation suite in real time, forwards the estimate
to PlotJuggler + MAVLink (znav3d), and logs every input stream to a
timestamped dataset directory in the standard datasets replay format --
so the same run can be reprocessed offline with python/replay.py
("falls was nicht stimmt").

    python3 python/crazyflie_reader.py --uri radio://0/80/2M/DABADA5503
    ...
    python3 python/replay.py datasets/crazyflie/2026_07_13_142530   # post-process

Two Crazyflie log configs are subscribed (each within the 26-byte CRTP
payload limit, all FP16):

  imu   100 Hz  acc.x/y/z [G], gyro.x/y/z [deg/s]
  est    10 Hz  stateEstimate.x/y/z [m], stateEstimate.qw/qx/qy/qz,
                kalman.varX/Y/Z [m^2], baro.asl [m]

The Crazyflie carries no GNSS. Its onboard EKF position (stateEstimate,
from the flow/UWB deck) anchors the local frame to a fake, configurable
lat/lon origin and is fed to INSLIB as a position fix with the reported
per-axis variance (kalman.var*), which is enough for the suite to
auto-initialize and run a full 3D solution. The heading from the
stateEstimate attitude quaternion is fed as a yaw measurement and baro.asl
drives the vertical channel. The
Crazyflie's own attitude/position estimate is also logged as the reference
(ref.csv) and published under `cf/*` so INSLIB can be compared against it
live in PlotJuggler.

Frames: the Crazyflie world frame is ENU, INSLIB is FRD body / NED nav.
  position  (n, e, d)_ned = ( y,  x, -z)_cf
  accel     (x, y, z)_frd = ( x, -y, -z)_cf         [G -> m/s^2]
  gyro      (p, q, r)_frd  = ( x, -y, -z)_cf         [deg/s -> rad/s]
  attitude  quaternion transform body-FLU/ENU -> body-FRD/NED (quat_cf_to_ned)
INSLIB uses init: auto and does not consume the reference attitude; the
yaw fed as a heading aid comes from the transformed NED quaternion.

Dataset directory (datasets/crazyflie/YYYY_MM_DD_HHMMSS/), all in the
datasets replay_format contract so python/replay.py reads it directly:
  config.yaml   aiding: gnss, init: auto, IMU noise model
  imu.csv       t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]
  gnss.csv      t_us, lat/lon/h, NED position covariance (position-only fix)
  ref.csv       t_us, lat/lon/h, roll/pitch/yaw [deg]  (Crazyflie's own estimate)
  baro.csv      t_us, static pressure [Pa]
  mag.csv       t_us, mag_frd_xyz [uT]  (synthetic: WMM field x CF attitude)

mag.csv is synthesized from the WMM reference field (INSLIB's own model)
rotated by the CF attitude, so replay.py -- whose replay_format has no yaw
stream -- recovers heading by fusing it against the same WMM reference.
Live navigation does not consume it (it uses the direct yaw aid).

Requires cflib (pip install cflib) and, for the MAVLink sink, pymavlink
(pip install pymavlink).

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import ctypes
import datetime
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                                   # python/
sys.path.insert(0, os.path.join(_HERE, "..", "datasets"))  # replay_format

from INSLIB import Navigator, Config, Telemetry, ecef_to_llh   # noqa: E402
from INSLIB._core import _lib as _inslib                       # noqa: E402
from replay_format import (GNSS_HEADER, REF_HEADER, IMU_HEADER,   # noqa: E402
                           MAG_HEADER, BARO_HEADER, gnss_row, write_config)


# World Magnetic Model reference field (INSLIB's own WMM LUT, so the
# synthesized mag.csv matches exactly what replay.py's mag fusion arms as
# its reference -- no declination mismatch). None if the shared library
# predates the export (rebuild: make pylib).
try:
    _inslib.magnetic_field_ned_uT.argtypes = [ctypes.c_float, ctypes.c_float,
                                              ctypes.c_float, ctypes.c_float * 3]
    _inslib.magnetic_field_ned_uT.restype = None

    def wmm_field_ned(lat_deg, lon_deg, year):
        """WMM reference field at a position [uT], NED."""
        out = (ctypes.c_float * 3)()
        _inslib.magnetic_field_ned_uT(float(lat_deg), float(lon_deg),
                                      float(year), out)
        return (out[0], out[1], out[2])
except AttributeError:                                          # pragma: no cover
    wmm_field_ned = None

US_PER_SEC = 1_000_000
G0 = 9.80665            # standard gravity, Crazyflie acc.* are in units of G
DEG2RAD = math.pi / 180.0

_WGS84_A = 6378137.0
_WGS84_E2 = 0.00669437999014

# International standard atmosphere (matches src/baro_alt.c's BARO_ALT_ISA_*),
# used to turn the Crazyflie's baro.asl [m] back into the static pressure [Pa]
# the vertical-channel filter actually consumes.
_ISA_P0_PA = 101325.0
_ISA_SCALE_M = 44330.0
_ISA_EXP = 5.255

# Default Crazyflie / BMI088 in-flight noise model. These are rough starting
# points dominated by motor vibration -- refine them per airframe from a real
# static/hover log with python/allan_variance.py or replay.py's noise check.
_IMU_NOISE = {
    "gyr_psd": (2.0e-5, "(rad/s)^2/Hz  approx, refine via allan_variance.py"),
    "acc_psd": (2.0e-3, "(m/s^2)^2/Hz  approx, refine via allan_variance.py"),
    "gyr_bias_rw": (1.0e-4, "rad/s/sqrt(s)  approx"),
    "acc_bias_rw": (1.0e-3, "m/s^2/sqrt(s)  approx"),
}

DEFAULT_URI = "radio://0/80/2M/DABADA5503"


def asl_to_pressure(asl_m):
    """Inverse ISA: barometric altitude [m] -> static pressure [Pa]."""
    return _ISA_P0_PA * (1.0 - asl_m / _ISA_SCALE_M) ** _ISA_EXP


# Attitude frame transform: the Crazyflie reports a Hamilton quaternion for
# body-FLU -> world-ENU; INSLIB wants body-FRD -> world-NED. Compose
# q_ned = q_enu_to_ned * q_cf * q_frd_to_flu entirely in quaternion space
# (no Euler round-trip, so no gimbal-lock degeneracy). q_enu_to_ned is a
# 180 deg rotation about (1,1,0)/sqrt2 -> (x,y,z)->(y,x,-z); q_frd_to_flu is
# 180 deg about x.
_Q_ENU_TO_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
_Q_FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)


def quat_multiply(a, b):
    """Hamilton product a*b, quaternions as (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_cf_to_ned(qw, qx, qy, qz):
    """Crazyflie attitude quaternion (body-FLU->world-ENU) -> INSLIB NED
    Hamilton quaternion (body-FRD->world-NED), unit-normalized."""
    q = quat_multiply(_Q_ENU_TO_NED, quat_multiply((qw, qx, qy, qz),
                                                   _Q_FRD_TO_FLU))
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def quat_to_rpy(q):
    """Hamilton quaternion (w,x,y,z) for R_b_to_n -> roll/pitch/yaw [rad]
    (ZYX Tait-Bryan), the inverse of INSLIB's rpy_to_quat."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rotate_ned_to_body(q, v_ned):
    """v_body = R_b_to_n' * v_ned, with q the Hamilton R_b_to_n quaternion.
    Used to express a nav-frame vector (e.g. the WMM field) in the body
    frame -- the synthetic magnetometer measurement."""
    w, x, y, z = q
    r = ((1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
         (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
         (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)))
    return tuple(sum(r[j][i] * v_ned[j] for j in range(3)) for i in range(3))


def decimal_year(dt=None):
    """Current UTC time as a decimal year (for the WMM epoch)."""
    dt = dt or datetime.datetime.now(datetime.timezone.utc)
    start = datetime.datetime(dt.year, 1, 1, tzinfo=datetime.timezone.utc)
    return dt.year + (dt - start).total_seconds() / (365.25 * 86400.0)


def llh_to_ecef(lat, lon, h):
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - _WGS84_E2) + h) * math.sin(lat))


def ned_to_ecef_rot(lat, lon):
    """R_n_to_e (NED -> ECEF), row-major 3x3 tuple."""
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return ((-sl * co, -so, -cl * co),
            (-sl * so,  co, -cl * so),
            ( cl,      0.0, -sl))


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

DEFAULTS = {
    "name": "crazyflie",
    "origin": {"lat_deg": 49.8728, "lon_deg": 8.6512, "h_m": 144.0},  # Darmstadt
    "aiding": {"pos_stddev_floor_hor_m": 0.05, "pos_stddev_floor_ver_m": 0.10,
               "yaw_stddev_deg": 5.0},
    # mag.csv is synthesized from the CF attitude + WMM reference field so
    # replay.py can recover heading (its replay_format has no yaw stream);
    # wmm_year is filled in at capture time.
    # min_delay_ms < 0 switches the fusion rate limit off: the library
    # throttles a real magnetometer because its disturbances are
    # time-correlated, which a stream derived from the CF's own attitude
    # simply is not.
    "mag": {"enable": 1, "stddev_ut": 0.5, "wmm_year": 0.0,
            "min_delay_ms": -1},
    "baro": {"enable": 1, "stddev_m": 1.0},
    "crazyflie": {"uri": DEFAULT_URI},
}


def write_session_config(path, spec):
    """Write config.yaml for a live session, in the datasets replay_format
    schema so python/replay.py reprocesses the dataset directly."""
    origin = spec["origin"]
    aid = spec["aiding"]
    write_config(path, [
        ("name", spec["name"]),
        ("aiding", ("gnss", "Crazyflie stateEstimate logged as position fixes")),
        ("init", "auto"),
        ("gyro_bias_window_sec",
         (3, "harness's historical implicit default, pinned explicitly (no library default exists)")),
        ("imu", dict(_IMU_NOISE)),
        ("gnss", {
            "leverarm_frd": [0.0, 0.0, 0.0],
            "pos_stddev_fallback_m": [aid["pos_stddev_floor_hor_m"],
                                      aid["pos_stddev_floor_ver_m"]],
            "vel_stddev_fallback_mps": 0.1,
        }),
        ("baro", {"enable": spec["baro"]["enable"],
                  "stddev_m": spec["baro"]["stddev_m"]}),
        ("mag", {"enable": spec["mag"]["enable"],
                 "stddev_ut": spec["mag"]["stddev_ut"],
                 "min_delay_ms": (spec["mag"]["min_delay_ms"],
                                  "no rate limit: not a real magnetometer"),
                 "wmm_year": (spec["mag"]["wmm_year"],
                              "synthesized from CF attitude + WMM")}),
        ("score", {"warmup_sec": 5, "leverarm_frd": [0.0, 0.0, 0.0]}),
        # Crazyflie-specific bookkeeping (ignored by replay.py): the fake
        # local-frame anchor and the radio URI this session was captured on.
        ("origin", {"lat_deg": (origin["lat_deg"], "fake local-frame anchor"),
                    "lon_deg": origin["lon_deg"], "h_m": origin["h_m"]}),
        ("crazyflie", {"uri": spec["crazyflie"]["uri"],
                       "yaw_stddev_deg": aid["yaw_stddev_deg"]}),
    ], header="generated by crazyflie_reader.py -- live Crazyflie capture\n"
              "IMU noise is an approximate BMI088/vibration starting point,"
              " refine per airframe from a real log")


# --------------------------------------------------------------------------
# INSLIB processing + logging pipeline
# --------------------------------------------------------------------------

class Pipeline:
    """Feeds Crazyflie samples into the INSLIB Navigator, logs every input
    stream in the replay_format, and fans the live estimate out to
    telemetry. The Crazyflie EKF position (est, ~10 Hz) is latched and
    attached to the next IMU epoch as a position fix + heading, mirroring
    replay.c/replay.py's "attach the most recent fix" structure so the
    logged dataset reproduces this run offline."""

    def __init__(self, spec, tele, logdir):
        self.tele = tele
        noise = spec["imu"]
        self.acc_var = (noise["acc_psd"],) * 3
        self.gyr_var = (noise["gyr_psd"],) * 3
        aid = spec["aiding"]
        self.var_floor = (aid["pos_stddev_floor_hor_m"] ** 2,
                          aid["pos_stddev_floor_hor_m"] ** 2,
                          aid["pos_stddev_floor_ver_m"] ** 2)
        self.yaw_stddev = aid["yaw_stddev_deg"] * DEG2RAD
        self.baro_enable = bool(spec["baro"]["enable"])
        self.baro_stddev = float(spec["baro"]["stddev_m"])

        # Synthetic magnetometer: the WMM reference field at the origin,
        # rotated into the body frame per epoch (see latch_est), so replay.py
        # can recover heading. Only for the logged mag.csv -- live navigation
        # uses the direct yaw aid instead.
        mag = spec["mag"]
        self.mag_ned = None
        self.mag_stddev = float(mag["stddev_ut"])
        if int(mag["enable"]):
            if wmm_field_ned is None:
                print("[cf] mag enabled but WMM lookup unavailable "
                      "(rebuild: make pylib) -- mag.csv disabled")
            else:
                year = float(mag["wmm_year"]) or decimal_year()
                self.mag_ned = wmm_field_ned(spec["origin"]["lat_deg"],
                                             spec["origin"]["lon_deg"], year)

        origin = spec["origin"]
        self.origin_lat = origin["lat_deg"] * DEG2RAD
        self.origin_lon = origin["lon_deg"] * DEG2RAD
        self.origin_ecef = llh_to_ecef(self.origin_lat, self.origin_lon,
                                       origin["h_m"])
        self.r_n_to_e = ned_to_ecef_rot(self.origin_lat, self.origin_lon)

        self.nav = Navigator(Config(
            lat_rad=self.origin_lat, lon_rad=self.origin_lon, h_m=origin["h_m"],
            auto_init=True,
            pos_init_stddev_m=1.0,
            vel_init_stddev_mps=0.5,
            rpy_init_stddev_rad=(3.0 * DEG2RAD,) * 3,
            acc_bias_init_stddev_mps2=0.05,
            gyr_bias_init_stddev_rps=0.5 * DEG2RAD,
            acc_bias_pred_stddev_mps2_sqrts=noise["acc_bias_rw"],
            gyr_bias_pred_stddev_rps_sqrts=noise["gyr_bias_rw"],
            kalman_update_dt_sec=0.01,
            max_prediction_time_sec=0.5,
        ))

        self._t_prev = None
        self._n_imu = 0
        self._n_est = 0
        self._fix = None          # pending (llh, var3, yaw_ned) for on_imu
        self._fix_new = False
        self._baro_pa = None
        self._baro_new = False
        self._cf = None           # latest cf pose for the overlay

        self._imu_f = open(os.path.join(logdir, "imu.csv"), "w", encoding="utf-8")
        self._imu_f.write(IMU_HEADER)
        self._gnss_f = open(os.path.join(logdir, "gnss.csv"), "w", encoding="utf-8")
        self._gnss_f.write(GNSS_HEADER)
        self._gnss_f.write("# Crazyflie stateEstimate position (flow/UWB deck)"
                           " as position-only fixes\n")
        self._ref_f = open(os.path.join(logdir, "ref.csv"), "w", encoding="utf-8")
        self._ref_f.write(REF_HEADER)
        self._ref_f.write("# Crazyflie onboard estimate (not independent truth)\n")
        self._baro_f = open(os.path.join(logdir, "baro.csv"), "w", encoding="utf-8")
        self._baro_f.write(BARO_HEADER)
        self._mag_f = None
        if self.mag_ned is not None:
            self._mag_f = open(os.path.join(logdir, "mag.csv"), "w",
                               encoding="utf-8")
            self._mag_f.write(MAG_HEADER)
            self._mag_f.write("# synthesized: WMM(%.4f,%.4f) field rotated by"
                              " the CF attitude\n" % (spec["origin"]["lat_deg"],
                                                      spec["origin"]["lon_deg"]))

    def _ned_to_llh(self, n, e, d):
        R = self.r_n_to_e
        ned = (n, e, d)
        ecef = tuple(self.origin_ecef[i] + sum(R[i][j] * ned[j] for j in range(3))
                     for i in range(3))
        lat, lon, h = ecef_to_llh(*ecef)
        return ecef, lat, lon, h

    # --- inputs -------------------------------------------------------------
    def latch_est(self, t_us, n, e, d, var3, q_ned):
        """A Crazyflie stateEstimate sample (position + NED attitude
        quaternion): log it as a GNSS fix + reference row (+ synthetic
        magnetometer), and stage it for the next IMU epoch."""
        self._n_est += 1
        roll_ned, pitch_ned, yaw_ned = quat_to_rpy(q_ned)
        var3 = tuple(max(var3[i], self.var_floor[i]) for i in range(3))
        ecef, lat, lon, h = self._ned_to_llh(n, e, d)
        self._fix = ((lat, lon, h), var3, yaw_ned)
        self._fix_new = True
        self._cf = {"pos_ned": (n, e, d),
                    "att_deg": (math.degrees(roll_ned), math.degrees(pitch_ned),
                                math.degrees(yaw_ned))}
        self._gnss_f.write(gnss_row(
            t_us, math.degrees(lat), math.degrees(lon), h,
            (var3[0], 0.0, 0.0, var3[1], 0.0, var3[2]),  # NED pos covariance
            (0.0, 0.0, 0.0), (0.0,) * 6, vel_ok=False))  # position-only fix
        self._ref_f.write("%d,%.10f,%.10f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n" % (
            t_us, math.degrees(lat), math.degrees(lon), h,
            math.degrees(roll_ned), math.degrees(pitch_ned), math.degrees(yaw_ned),
            0.0, 0.0, 0.0))
        if self._mag_f is not None:
            mb = rotate_ned_to_body(q_ned, self.mag_ned)
            self._mag_f.write("%d,%.4f,%.4f,%.4f\n" % (t_us, mb[0], mb[1], mb[2]))

    def latch_baro(self, t_us, pressure_pa, asl_m=None):
        self._baro_pa = pressure_pa
        self._baro_new = True
        if asl_m is not None and self._cf is not None:
            self._cf["asl_m"] = asl_m
        self._baro_f.write("%d,%.4f\n" % (t_us, pressure_pa))

    def on_imu(self, t_us, acc_frd, gyr_frd):
        dt = (t_us - self._t_prev) / US_PER_SEC if self._t_prev is not None else 0.0
        self._t_prev = t_us
        self.nav.imu(t_us, dt, acc_frd, gyr_frd, self.acc_var, self.gyr_var)
        if self._fix_new:
            llh, var3, yaw = self._fix
            self.nav.gnss_pos_llh(llh, var3)
            if yaw is not None and math.isfinite(yaw):
                self.nav.yaw(yaw, self.yaw_stddev)
            self._fix_new = False
        if self._baro_new and self.baro_enable:
            self.nav.baro(self._baro_pa, self.baro_stddev)
            self._baro_new = False
        self.nav.update()
        self._publish()
        self._imu_f.write("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n" % (
            t_us, gyr_frd[0], gyr_frd[1], gyr_frd[2],
            acc_frd[0], acc_frd[1], acc_frd[2]))
        self._n_imu += 1

    # --- outputs ------------------------------------------------------------
    def _publish(self):
        self.tele.publish(self.nav.state(), self.nav.stddev())
        if self._cf is not None:
            cf = {"pos_ned": {"n": self._cf["pos_ned"][0],
                              "e": self._cf["pos_ned"][1],
                              "d": self._cf["pos_ned"][2]},
                  "att_deg": {"roll": self._cf["att_deg"][0],
                              "pitch": self._cf["att_deg"][1],
                              "yaw": self._cf["att_deg"][2]}}
            if "asl_m" in self._cf:
                cf["baro"] = {"asl_m": self._cf["asl_m"]}
            self.tele.publish_extra({"cf": cf})

    def status(self, elapsed, imu_hz, est_hz):
        """One-line live status: incoming sensor rates + whether INSLIB is
        producing a solution (suite mode + current NED position/yaw)."""
        parts = [f"t={elapsed:4.0f}s",
                 f"imu {imu_hz:5.1f}Hz ({self._n_imu})",
                 f"est {est_hz:4.1f}Hz ({self._n_est})",
                 f"mode {self.nav.mode_name()}"
                 f"{'/ready' if self.nav.is_ready() else ''}"]
        pos = self.nav.position_local()
        if pos is not None:
            parts.append("NED %+.2f %+.2f %+.2f" % (pos[0], pos[1], pos[2]))
        rpy = self.nav.rpy()
        if rpy is not None:
            parts.append("yaw %+6.1f" % math.degrees(rpy[2]))
        if imu_hz == 0.0:
            parts.append("<-- no IMU data")
        return "  ".join(parts)

    def close(self):
        for f in (self._imu_f, self._gnss_f, self._ref_f, self._baro_f,
                  self._mag_f):
            if f is not None:
                f.close()
        self.nav.close()


# --------------------------------------------------------------------------
# Live capture
# --------------------------------------------------------------------------

def run_live(pipeline, uri, duration_sec):
    """Connect over the Crazyradio and drive the pipeline from log callbacks."""
    import cflib.crtp                                        # noqa: E402
    from cflib.crazyflie import Crazyflie                    # noqa: E402
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  # noqa: E402
    from cflib.crazyflie.log import LogConfig                # noqa: E402

    def imu_cb(ts_ms, data, _logconf):
        # G -> m/s^2, deg/s -> rad/s, FLU -> FRD (y -> -y, z -> -z).
        acc_frd = (data["acc.x"] * G0, -data["acc.y"] * G0, -data["acc.z"] * G0)
        gyr_frd = (data["gyro.x"] * DEG2RAD, -data["gyro.y"] * DEG2RAD,
                   -data["gyro.z"] * DEG2RAD)
        pipeline.on_imu(ts_ms * 1000, acc_frd, gyr_frd)

    def est_cb(ts_ms, data, _logconf):
        t_us = ts_ms * 1000
        # CF world -> local NED (n=y, e=x, d=-z); attitude via the quaternion
        # frame transform (body-FLU/ENU -> body-FRD/NED), then ZYX Euler.
        n, e, d = (data["stateEstimate.y"], data["stateEstimate.x"],
                   -data["stateEstimate.z"])
        q_ned = quat_cf_to_ned(data["stateEstimate.qw"], data["stateEstimate.qx"],
                               data["stateEstimate.qy"], data["stateEstimate.qz"])
        # NED position variance follows the axis swap: var_n = var(y_cf),
        # var_e = var(x_cf), var_d = var(z_cf). This is the measurement
        # covariance R the position fix is fused with (floored below).
        var3 = (data["kalman.varY"], data["kalman.varX"], data["kalman.varZ"])
        pipeline.latch_est(t_us, n, e, d, var3, q_ned)
        asl = data["baro.asl"]
        pipeline.latch_baro(t_us, asl_to_pressure(asl), asl)

    logconf_imu = LogConfig(name="inslib_imu", period_in_ms=10)
    for v in ("acc.x", "acc.y", "acc.z", "gyro.x", "gyro.y", "gyro.z"):
        logconf_imu.add_variable(v, "FP16")
    logconf_est = LogConfig(name="inslib_est", period_in_ms=100)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.qw", "stateEstimate.qx", "stateEstimate.qy",
              "stateEstimate.qz", "kalman.varX", "kalman.varY", "kalman.varZ",
              "baro.asl"):
        logconf_est.add_variable(v, "FP16")

    cflib.crtp.init_drivers()
    cache = os.path.join(_HERE, "..", "datasets", "crazyflie", ".cache")
    os.makedirs(cache, exist_ok=True)
    print(f"[cf] connecting to {uri} ...")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache)) as scf:
        scf.cf.log.add_config(logconf_imu)
        scf.cf.log.add_config(logconf_est)
        logconf_imu.data_received_cb.add_callback(imu_cb)
        logconf_est.data_received_cb.add_callback(est_cb)
        logconf_imu.start()
        logconf_est.start()
        print("[cf] logging started -- Ctrl-C to stop")
        pipeline.tele.statustext("INSLIB Crazyflie live", "info")
        t0 = last = time.time()
        last_imu = last_est = 0
        try:
            while duration_sec <= 0 or (time.time() - t0) < duration_sec:
                time.sleep(0.25)
                now = time.time()
                if now - last < 1.0:
                    continue
                imu_hz = (pipeline._n_imu - last_imu) / (now - last)
                est_hz = (pipeline._n_est - last_est) / (now - last)
                last, last_imu, last_est = now, pipeline._n_imu, pipeline._n_est
                print("[cf] " + pipeline.status(now - t0, imu_hz, est_hz))
        except KeyboardInterrupt:
            print("\n[cf] stopping")
        finally:
            logconf_imu.stop()
            logconf_est.stop()


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uri", default=DEFAULT_URI,
                    help=f"Crazyflie radio URI (default {DEFAULT_URI})")
    ap.add_argument("--outdir", default=os.path.join(_HERE, "..", "datasets",
                                                      "crazyflie"),
                    help="base directory for session logs")
    ap.add_argument("--origin", type=float, nargs=3,
                    metavar=("LAT_DEG", "LON_DEG", "H_M"),
                    help="fake local-frame origin (default %s %s %s, Darmstadt)"
                         % (DEFAULTS["origin"]["lat_deg"],
                            DEFAULTS["origin"]["lon_deg"],
                            DEFAULTS["origin"]["h_m"]))
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after N seconds (0 = until Ctrl-C)")
    ap.add_argument("--yaw-stddev-deg", type=float,
                    default=DEFAULTS["aiding"]["yaw_stddev_deg"],
                    help="1-sigma accuracy of the Crazyflie heading fed as a"
                         " yaw measurement (default %(default)s)")
    ap.add_argument("--pos-stddev-floor", type=float, nargs=2,
                    metavar=("HOR_M", "VER_M"),
                    help="floor on the kalman.var* position stddev before"
                         " fusion (the CF EKF covariance is often over-"
                         "confident; default %s %s)"
                         % (DEFAULTS["aiding"]["pos_stddev_floor_hor_m"],
                            DEFAULTS["aiding"]["pos_stddev_floor_ver_m"]))
    ap.add_argument("--no-mag", action="store_true",
                    help="do not synthesize mag.csv from the CF attitude + WMM")
    ap.add_argument("--wmm-year", type=float, default=0.0,
                    help="WMM epoch (decimal year) for the synthetic"
                         " magnetometer (default: current date)")
    ap.add_argument("--no-plotjuggler", action="store_true",
                    help="disable the PlotJuggler UDP/JSON sink")
    ap.add_argument("--no-mavlink", action="store_true",
                    help="disable the MAVLink (znav3d) sink")
    ap.add_argument("--pj-port", type=int, default=9870)
    ap.add_argument("--mav-ip", default="127.0.0.1")
    ap.add_argument("--mav-port", type=int, default=14550)
    args = ap.parse_args()

    spec = {k: (dict(v) if isinstance(v, dict) else v)
            for k, v in DEFAULTS.items()}
    spec["imu"] = {k: _IMU_NOISE[k][0] for k in _IMU_NOISE}
    spec["crazyflie"]["uri"] = args.uri
    spec["aiding"]["yaw_stddev_deg"] = args.yaw_stddev_deg
    spec["mag"]["enable"] = 0 if args.no_mag else 1
    spec["mag"]["wmm_year"] = args.wmm_year or decimal_year()
    if args.pos_stddev_floor:
        spec["aiding"]["pos_stddev_floor_hor_m"] = args.pos_stddev_floor[0]
        spec["aiding"]["pos_stddev_floor_ver_m"] = args.pos_stddev_floor[1]
    if args.origin:
        spec["origin"] = {"lat_deg": args.origin[0], "lon_deg": args.origin[1],
                          "h_m": args.origin[2]}

    logdir = os.path.join(args.outdir, time.strftime("%Y_%m_%d_%H%M%S"))
    os.makedirs(logdir, exist_ok=True)
    write_session_config(os.path.join(logdir, "config.yaml"), spec)
    print(f"[cf] logging to {logdir}")

    tele = Telemetry(plotjuggler=not args.no_plotjuggler,
                     mavlink=not args.no_mavlink,
                     pj_port=args.pj_port,
                     mav_ip=args.mav_ip, mav_port=args.mav_port)
    pipeline = Pipeline(spec, tele, logdir)
    try:
        run_live(pipeline, args.uri, args.duration)
    finally:
        pipeline.close()
        tele.close()
        print(f"[cf] {pipeline._n_imu} IMU epochs logged to {logdir}")


if __name__ == "__main__":
    main()
