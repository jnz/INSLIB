"""Telemetry forwarding for ins estimates.

Two independent sinks, either or both enabled:

  * PlotJugglerSender: UDP/JSON to PlotJuggler (default 127.0.0.1:9870),
    with an optional NDJSON flight log. NaN/Inf leaves are dropped so the
    JSON stays finite (PlotJuggler rejects datagrams otherwise).
  * MavlinkSender: rate-limited MAVLink (ATTITUDE_QUATERNION, ATTITUDE,
    LOCAL_POSITION_NED, GLOBAL_POSITION_INT, ALTITUDE, HIGHRES_IMU,
    EKF_STATUS_REPORT, NAMED_VALUE_FLOAT, STATUSTEXT) via pymavlink
    (default udpout:127.0.0.1:14550).

The Telemetry facade takes a ins ``State`` (see ins._core) and fans it
out. Adapted from a Crazyflie telemetry adapter; the Crazyflie/battery/
NMPC specifics were dropped and the wire data comes straight from INSLIB
(already NED / Hamilton, so there is NO frame conversion here).

Both python/replay.py (post-hoc replay) and a live receiver (off
the wire) drive this module, so a capture and its replay produce the
same PlotJuggler tree and the same MAVLink stream. The shared entry
point is :meth:`Telemetry.publish_suite`, which takes the Navigator
itself (not just its arbitrated State) and publishes:

  * the arbitrated best-available solution      -> INSLIB/*
  * every sub-filter's OWN output, side by side -> INSLIB/ars|ahrs|
    full3d|baroalt/* (see :func:`subfilter_overlay_tree`)
  * per-filter 1-sigma uncertainties            -> .../sigma/*
  * health/status bits and, when the 3D filter is not running, WHY
    (see :func:`suite_status`)                  -> INSLIB/status/*

pymavlink is an optional dependency: if it is missing, MavlinkSender is
simply unavailable and PlotJuggler-only telemetry still works.

(c) Jan Zwiener (jan@zwiener.org)
"""

import json
import math
import os
import socket
import time

_DROP = object()  # sentinel: a non-finite leaf to omit from the JSON entirely

_DEG = 180.0 / math.pi


def sanitize_for_json(data):
    """Recursively drop NaN/Inf leaves so the JSON is valid and finite."""
    if isinstance(data, dict):
        out = {}
        for k, v in data.items():
            sv = sanitize_for_json(v)
            if sv is not _DROP:
                out[k] = sv
        return out
    if isinstance(data, (list, tuple)):
        out = []
        for v in data:
            sv = sanitize_for_json(v)
            if sv is not _DROP:
                out.append(sv)
        return out
    if isinstance(data, float):
        if math.isnan(data) or math.isinf(data):
            return _DROP
        return data
    return data


# International standard atmosphere (matches src/baro_alt.c's
# BARO_ALT_ISA_*), for a convenient "sensor/baro/alt_m" alongside the
# filtered height -- baro_alt itself only ever sees pressure_pa.
_ISA_P0_PA = 101325.0
_ISA_SCALE_M = 44330.0
_ISA_EXP = 1.0 / 5.255


def isa_pressure_to_altitude(pressure_pa):
    """Non-positive pressure (a dropped/garbled live sample) has no
    altitude: return NaN rather than letting ** produce a complex number."""
    if not (pressure_pa > 0.0):
        return float("nan")
    return _ISA_SCALE_M * (1.0 - (pressure_pa / _ISA_P0_PA) ** _ISA_EXP)


# --------------------------------------------------------------------------
# Suite status / "why is the 3D filter not running?"
# --------------------------------------------------------------------------

# Numeric reason codes for the full 3D filter (ins). Published as
# INSLIB/status/blocked so the reason is a plottable step function, not
# just a log line. OK/COASTING mean the filter IS running.
BLOCKED_OK = 0             # ins ready, aiding fresh (mode FULL)
BLOCKED_COASTING = 1       # ins ready but dead-reckoning (aiding stale/gone)
BLOCKED_WARMUP = 2         # aiding accepted, filter not converged yet
BLOCKED_NO_AIDING = 3      # no absolute-position measurement has EVER arrived
BLOCKED_AIDING_REJECTED = 4  # aiding arrived but every fix failed the noise gate
BLOCKED_NO_ANCHOR = 5      # aiding arrived but there is no ECEF anchor for it

BLOCKED_TEXT = {
    BLOCKED_OK: "3D filter running (FULL)",
    BLOCKED_COASTING: "3D filter coasting: no fresh position aiding",
    BLOCKED_WARMUP: "3D filter converging: aiding accepted, not ready yet",
    BLOCKED_NO_AIDING: "3D filter off: no GNSS/position measurements at all",
    BLOCKED_AIDING_REJECTED: "3D filter off: all GNSS fixes rejected (accuracy gate)",
    BLOCKED_NO_ANCHOR: "3D filter off: GNSS seen but no ECEF anchor",
}


def suite_status(nav):
    """Health/status bits of the whole suite, plus the reason the full 3D
    filter is not producing a solution.

    Returns ``(tree, blocked_code, blocked_text)``. ``tree`` is the
    PlotJuggler sub-tree published under INSLIB/status: every entry is a
    float so it plots as a step function next to the estimates. The
    per-sub-filter ``*_running`` bits answer "which module is alive right
    now", and the ``gnss_*`` counters plus ``blocked`` answer "and if the
    3D filter is not, why not".

    The reason is derived from ins's own diagnostic counters (see
    Navigator.diag): a fix that never arrived, one rejected by the
    accuracy gate and one that arrived before the ECEF anchor existed are
    three different failures with three different fixes, and guessing
    between them from the outside is exactly what this avoids.
    """
    diag = nav.diag()
    ready = nav.is_ready()
    dr_ms = nav.deadreckoning_ms()
    mode = nav.mode_name()

    seen = int(diag["n_gnss_seen"])
    used = int(diag["n_gnss_used"])
    rejected = int(diag["n_gnss_rejected_noise"])
    no_anchor = int(diag["n_gnss_no_anchor"])

    if ready:
        blocked = BLOCKED_OK if mode == "FULL" else BLOCKED_COASTING
    elif used > 0:
        blocked = BLOCKED_WARMUP
    elif no_anchor > 0:
        blocked = BLOCKED_NO_ANCHOR
    elif rejected > 0:
        blocked = BLOCKED_AIDING_REJECTED
    elif seen == 0:
        blocked = BLOCKED_NO_AIDING
    else:
        # Fixes arrived and were neither used nor counted as rejected:
        # they are still queued / the filter has not consumed one yet.
        blocked = BLOCKED_WARMUP

    def bit(x):
        return 1.0 if x else 0.0

    tree = {
        "blocked": float(blocked),
        "mode": float(MODE_CODES.get(mode, 0)),
        "dr_ms": float(dr_ms),
        # Which sub-filter is producing output this epoch.
        "full3d_running": bit(ready),
        "ars_running": bit(nav.rpy_ars() is not None),
        "ahrs_running": bit(nav.rpy_ahrs() is not None),
        "baro_running": bit(nav.baro_alt() is not None),
        "baro_offset_locked": bit(nav.local_gnss_offset() is not None),
        # Aiding pipeline (cumulative, so a stalled counter is as
        # informative as a rising one).
        "gnss_seen": float(seen),
        "gnss_used": float(used),
        "gnss_rejected_noise": float(rejected),
        "gnss_no_anchor": float(no_anchor),
        "fuse_fail": float(diag["n_fuse_fail"]),
        "invalid_input": float(diag["n_invalid_input"]),
        # Timestamp health. Without these a restarted time source looks
        # exactly like a healthy filter that stopped moving: the tree keeps
        # arriving while every epoch behind it is dropped (REQ-NAV-070).
        "time_backward": float(diag["n_time_backward"]),
        "time_dropped": float(diag["n_time_dropped"]),
        "time_restart_reset": float(diag["n_time_restart_reset"]),
        # Stillness detectors (REQ-SUITE-009/-010/-015, REQ-AHRS-017).
        # vertical_zupt is what actually REACHED baro_alt, which is not
        # implied by any of the others: while ins is attitude-only its
        # own detector never fires and the ARS/AHRS fallback is the only
        # source the vertical channel has.
        "zupt_ins": bit(nav.auto_zupt_active()),
        "zaru_ars": bit(nav.ars_auto_zaru_active()),
        "zaru_ahrs": bit(nav.ahrs_auto_zaru_active()),
        "zaru_applied": bit(nav.zaru_active()),
        "vertical_zupt": bit(nav.vertical_zupt_active()),
    }

    oc = nav.overconfidence()
    tree["overconfident"] = {
        "full3d": bit(oc["tripped"]),
        "ars": bit(oc["ars"]["tripped"]),
        "ahrs": bit(oc["ahrs"]["tripped"]),
    }
    return tree, blocked, BLOCKED_TEXT[blocked]


# --------------------------------------------------------------------------
# Sub-filter breakdown
# --------------------------------------------------------------------------

def subfilter_overlay_tree(nav):
    """Per-sub-filter PlotJuggler view of a Navigator: what each module of
    the suite computes on its OWN, next to the arbitrated solution.

    INSLIB/* (from :func:`state_to_pj_tree`) is the *arbitrated* best
    available answer; this tree is the breakdown behind it, so a
    disagreement between the modules is visible directly instead of
    having to be inferred from a mode switch:

      * ``ars``    -- the roll/pitch ARS's own attitude, gyro bias and
        that bias's 1-sigma. Runs on gyro+accel alone and therefore keeps
        running when everything else has stopped.
      * ``ahrs``   -- same for the magnetometer-aided AHRS (absolute
        heading), independent of the ARS and of ins.
      * ``full3d`` -- the 15/18-state ESKF's own position/velocity/
        attitude, accel+gyro bias, and under ``sigma`` its own
        error-state 1-sigma for all of them. Present regardless of the
        suite-level mode arbitration.
      * ``baroalt``-- the baro/accel vertical filter's own height, climb
        rate, z-accel bias and their 1-sigma, plus (once a baro/GNSS pair
        has been seen) the baro-to-ellipsoid offset filter:
        ellipsoid height ~= barometric ISA altitude + gnss_offset_m.
        Independent of the height() arbitration INSLIB/global/alt_m may
        fall back to.
      * ``status`` -- :func:`suite_status`, including why ``full3d`` is
        not running when it is not.
      * ``zaru``   -- step functions to correlate ARS/AHRS gyro-bias
        corrections and the vertical zero-velocity update with detected
        stops (REQ-SUITE-009/-010/-015, REQ-AHRS-017).

    ``n_downweighted`` (REQ-SYS-006) sits alongside each sub-filter's
    other fields -- monotonic counters of chi2-downweighted fusions, for
    spotting outlier-heavy stretches live instead of only in a final
    stats printout.

    Heights are negated into the same NED-down convention as pos_ned
    everywhere else (baro_alt itself is positive-up), so baroalt/pos_ned/d
    overlays INSLIB/pos_ned/d directly.
    """
    def att_deg(rpy):
        return {"roll": rpy[0] * _DEG, "pitch": rpy[1] * _DEG,
                "yaw": rpy[2] * _DEG}

    def xyz(v):
        return {"x": v[0], "y": v[1], "z": v[2]}

    def gyr_bias_dps(bias_rps):
        return xyz([b * _DEG for b in bias_rps])

    dw = nav.downweight_counts()
    tree = {}

    # --- attitude-only sub-filters (ARS: roll/pitch, AHRS: + mag yaw) ---
    for name, rpy_fn, bias_fn, sigma_fn in (
            ("ars", nav.rpy_ars, nav.bias_gyr_ars, nav.gyr_bias_stddev_ars),
            ("ahrs", nav.rpy_ahrs, nav.bias_gyr_ahrs, nav.gyr_bias_stddev_ahrs)):
        rpy = rpy_fn()
        if rpy is None:
            continue
        sub = {"att_deg": att_deg(rpy), "n_downweighted": float(dw[name])}
        bias = bias_fn()
        if bias is not None:
            sub["gyr_bias_dps"] = gyr_bias_dps(bias)
        sigma = sigma_fn()
        if sigma is not None:
            sub["gyr_bias_sigma_dps"] = gyr_bias_dps(sigma)
        tree[name] = sub

    # --- the full 3D filter's own solution -----------------------------
    full3d = {}
    rpy = nav.rpy_ins()
    if rpy is not None:
        full3d["att_deg"] = att_deg(rpy)
    pos = nav.position_local()
    if pos is not None:
        full3d["pos_ned"] = {"n": pos[0], "e": pos[1], "d": pos[2]}
    vel = nav.velocity_ned()
    if vel is not None:
        full3d["vel_ned"] = {"n": vel[0], "e": vel[1], "d": vel[2]}
    gyr_bias = nav.bias_gyr()
    if gyr_bias is not None:
        full3d["gyr_bias_dps"] = gyr_bias_dps(gyr_bias)
    acc_bias = nav.bias_acc()
    if acc_bias is not None:
        full3d["acc_bias_mps2"] = xyz(acc_bias)
    sd = nav.stddev()
    if sd is not None:
        full3d["sigma"] = {
            "gyr_bias_dps": gyr_bias_dps(sd["gyr_bias"]),
            "acc_bias_mps2": xyz(sd["acc_bias"]),
            "pos_ned_m": {"n": sd["pos_ned"][0], "e": sd["pos_ned"][1],
                          "d": sd["pos_ned"][2]},
            "vel_ned_mps": {"n": sd["vel_ned"][0], "e": sd["vel_ned"][1],
                            "d": sd["vel_ned"][2]},
            "att_deg": att_deg(sd["rpy"]),
        }
    full3d["n_downweighted"] = float(nav.diag()["n_downweighted"])
    tree["full3d"] = full3d

    # --- the baro/accel vertical channel -------------------------------
    baro = nav.baro_alt()
    if baro is not None:
        h_m, v_mps = baro
        sub = {"pos_ned": {"d": -h_m}, "vel_ned": {"d": -v_mps},
               "height_m": h_m, "climb_mps": v_mps,
               "zupt_applied": 1.0 if nav.vertical_zupt_active() else 0.0,
               "n_downweighted": float(dw["baro_alt"])}
        acc_bias = nav.baro_acc_bias()
        if acc_bias is not None:
            sub["acc_bias_mps2"] = acc_bias
        sigma = nav.baro_stddev()
        if sigma is not None:
            sub["sigma"] = {"height_m": sigma[0], "climb_mps": sigma[1],
                            "acc_bias_mps2": sigma[2]}
        offset = nav.local_gnss_offset()
        if offset is not None:
            sub["gnss_offset_m"] = offset[0]
            sub["gnss_offset_sigma_m"] = offset[1]
            sub["gnss_offset_n_downweighted"] = float(dw["local_gnss"])
        tree["baroalt"] = sub

    status, _blocked, _text = suite_status(nav)
    tree["status"] = status
    tree["zaru"] = {
        "ins_active": status["zupt_ins"],
        "ars_active": status["zaru_ars"],
        "ahrs_active": status["zaru_ahrs"],
        "applied": status["zaru_applied"],
        "vertical_applied": status["vertical_zupt"],
    }
    return {"INSLIB": tree}


class PlotJugglerSender:
    """UDP/JSON sender for PlotJuggler, with an optional NDJSON flight log."""

    def __init__(self, ip="127.0.0.1", port=9870, log_path=None):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.t0 = time.perf_counter()
        self.log_file = None
        self._log_last_flush = 0.0
        if log_path:
            try:
                d = os.path.dirname(os.path.abspath(log_path))
                os.makedirs(d, exist_ok=True)
                self.log_file = open(log_path, "a", encoding="utf-8")
                print(f"[telemetry] flight log -> {log_path}")
            except OSError as e:
                print(f"[telemetry] flight log disabled: {e}")

    def send(self, data_dict):
        t_sec = time.perf_counter() - self.t0
        timestamp = time.time()
        clean = sanitize_for_json(data_dict)
        payload = {**clean, "t_sec": t_sec, "timestamp": timestamp}
        try:
            self.sock.sendto(json.dumps(payload).encode("utf-8"),
                             (self.ip, self.port))
        except OSError as e:
            print(f"[telemetry] UDP send error: {e}")
        if self.log_file is not None:
            try:
                self.log_file.write(json.dumps(payload) + "\n")
                if t_sec - self._log_last_flush >= 1.0:
                    self.log_file.flush()
                    self._log_last_flush = t_sec
            except OSError as e:
                print(f"[telemetry] flight-log write error: {e}")

    def close(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None


class MavlinkSender:
    """Rate-limited MAVLink sender for a ins ``State`` (NED, Hamilton)."""

    def __init__(self, ip="127.0.0.1", port=14550, subfilter_hz=5.0):
        # DEBUG_FLOAT_ARRAY etc. need MAVLink 2; force v2 before importing.
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil  # optional dependency
        self._mavutil = mavutil
        self.conn = mavutil.mavlink_connection(
            f"udpout:{ip}:{port}", source_system=1, source_component=1)
        self.start_time = time.time()
        self.last_heartbeat = 0.0
        self.intervals = {
            "local_pos": 1.0 / 50.0,
            "global_pos": 1.0 / 20.0,
            "altitude": 1.0 / 20.0,
            "attitude": 1.0 / 50.0,
            "imu": 1.0 / 50.0,
            "ekf_status": 1.0 / 10.0,
            "subfilter": 1.0 / max(subfilter_hz, 0.1),
        }
        self.last_sent = {k: 0.0 for k in self.intervals}
        self._last_blocked = None

    def _due(self, key, now):
        if now - self.last_sent[key] >= self.intervals[key]:
            self.last_sent[key] = now
            return True
        return False

    def send(self, st, stddev=None, status=None):
        mav = self.conn.mav
        now = time.time()
        time_boot_ms = int((now - self.start_time) * 1000)

        if now - self.last_heartbeat > 1.0:
            mav.heartbeat_send(self._mavutil.mavlink.MAV_TYPE_GENERIC,
                               self._mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                               0, 0, 0)
            self.last_heartbeat = now

        if self._due("local_pos", now) and math.isfinite(st.x_m):
            mav.local_position_ned_send(
                time_boot_ms, st.x_m, st.y_m, st.z_m,
                st.vx_mps, st.vy_mps, st.vz_mps)

        if self._due("global_pos", now) and math.isfinite(st.lat_rad):
            mav.global_position_int_send(
                time_boot_ms,
                int(math.degrees(st.lat_rad) * 1e7),
                int(math.degrees(st.lon_rad) * 1e7),
                int(st.alt_m * 1000.0),
                int(-st.z_m * 1000.0) if math.isfinite(st.z_m) else 0,
                int(st.vx_mps * 100.0) if math.isfinite(st.vx_mps) else 0,
                int(st.vy_mps * 100.0) if math.isfinite(st.vy_mps) else 0,
                int(st.vz_mps * 100.0) if math.isfinite(st.vz_mps) else 0,
                65535)

        # ALTITUDE carries the vertical channel on its own. Without it a
        # baro-only solution would never reach MAVLink at all: both
        # messages above are gated on a horizontal position, and while
        # the suite is in ATTITUDE_ONLY mode x/y and lat/lon are NaN even
        # though baro_alt is producing a perfectly good height.
        if self._due("altitude", now) and (math.isfinite(st.z_m)
                                           or math.isfinite(st.alt_m)):
            local_up = -st.z_m if math.isfinite(st.z_m) else float("nan")
            mav.altitude_send(
                int((now - self.start_time) * 1e6),
                local_up,          # altitude_monotonic
                st.alt_m,          # altitude_amsl (ellipsoid: no geoid model)
                local_up,          # altitude_local (NED-origin datum)
                local_up,          # altitude_relative
                float("nan"),      # altitude_terrain
                float("nan"))      # bottom_clearance

        if self._due("attitude", now):
            # Attitude as both a quaternion and Euler angles (ATTITUDE), so
            # consumers can pick whichever they prefer.
            if math.isfinite(st.qw):
                mav.attitude_quaternion_send(
                    time_boot_ms, st.qw, st.qx, st.qy, st.qz,
                    _f(st.roll_rate_rps), _f(st.pitch_rate_rps),
                    _f(st.yaw_rate_rps))
            if math.isfinite(st.roll_rad):
                mav.attitude_send(
                    time_boot_ms, st.roll_rad, st.pitch_rad, st.yaw_rad,
                    _f(st.roll_rate_rps), _f(st.pitch_rate_rps),
                    _f(st.yaw_rate_rps))

        if self._due("imu", now):
            values = [
                (st.ax_mps2, 1 << 0), (st.ay_mps2, 1 << 1), (st.az_mps2, 1 << 2),
                (st.roll_rate_rps, 1 << 3), (st.pitch_rate_rps, 1 << 4),
                (st.yaw_rate_rps, 1 << 5),
            ]
            fields_updated = 0
            wire = []
            for v, bit in values:
                if math.isfinite(v):
                    fields_updated |= bit
                    wire.append(v)
                else:
                    wire.append(0.0)
            if fields_updated:
                mav.highres_imu_send(
                    int((now - self.start_time) * 1e6),
                    wire[0], wire[1], wire[2], wire[3], wire[4], wire[5],
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, fields_updated)

        if self._due("ekf_status", now):
            self._send_ekf_status(stddev, st, status)

    def _send_ekf_status(self, stddev, st=None, status=None):
        """EKF_STATUS_REPORT with real flags and real variances.

        The flags are what the suite actually has this epoch, not a
        constant: a receiver can then tell "attitude only, no position"
        from "everything nominal" without parsing anything else. The
        variances come from the filter's own covariance (see
        Navigator.stddev()) whenever it is available.
        """
        m = self._mavutil.mavlink
        flags = 0
        if status is not None:
            if status["full3d_running"] or status["ars_running"] \
                    or status["ahrs_running"]:
                flags |= m.EKF_ATTITUDE
            if status["full3d_running"]:
                flags |= (m.EKF_VELOCITY_HORIZ | m.EKF_VELOCITY_VERT
                          | m.EKF_POS_HORIZ_REL)
                if status["blocked"] == float(BLOCKED_OK):
                    flags |= m.EKF_POS_HORIZ_ABS
            if status["baro_running"] or status["full3d_running"]:
                flags |= (m.EKF_POS_VERT_ABS | m.EKF_POS_VERT_AGL)
        else:
            # No Navigator handed in (plain publish()): fall back to what
            # the State itself proves is present.
            if st is not None and math.isfinite(st.roll_rad):
                flags |= m.EKF_ATTITUDE
            if st is not None and math.isfinite(st.x_m):
                flags |= (m.EKF_VELOCITY_HORIZ | m.EKF_VELOCITY_VERT
                          | m.EKF_POS_HORIZ_REL | m.EKF_POS_HORIZ_ABS)
            if st is not None and math.isfinite(st.z_m):
                flags |= m.EKF_POS_VERT_ABS

        if stddev is not None:
            vel_var = max(v * v for v in stddev["vel_ned"])
            pos_h_var = max(stddev["pos_ned"][0]**2, stddev["pos_ned"][1]**2)
            pos_v_var = stddev["pos_ned"][2]**2
            compass_var = stddev["rpy"][2]**2  # yaw variance [rad^2]
        else:
            vel_var = pos_h_var = pos_v_var = 0.01**2
            compass_var = 0.0
        self.conn.mav.ekf_status_report_send(flags, vel_var, pos_h_var,
                                             pos_v_var, compass_var, 0.0)

    def send_subfilters(self, nav, status=None, blocked=None, text=None):
        """Publish the sub-filter breakdown as NAMED_VALUE_FLOAT.

        NAMED_VALUE_FLOAT is the one MAVLink message every ground station
        and log analyser understands without a dialect extension, so the
        per-module outputs and their 1-sigma stay readable in QGC /
        MAVExplorer alongside the standard messages. Names are capped at
        MAVLink's 10-char limit.

        The blocked reason additionally goes out as a STATUSTEXT, but only
        when it CHANGES -- a reason repeated at the publish rate would
        just flood the message log.
        """
        now = time.time()
        if not self._due("subfilter", now):
            return
        if status is None:
            status, blocked, text = suite_status(nav)
        boot_ms = int((now - self.start_time) * 1000)

        def nv(name, value):
            if value is not None and math.isfinite(value):
                self.conn.mav.named_value_float_send(
                    boot_ms, name.encode("ascii")[:10], float(value))

        nv("mode", status["mode"])
        nv("blocked", status["blocked"])
        nv("dr_ms", status["dr_ms"])
        nv("vert_zupt", status["vertical_zupt"])

        baro = nav.baro_alt()
        if baro is not None:
            nv("ba_h", baro[0])
            nv("ba_vz", baro[1])
            sigma = nav.baro_stddev()
            if sigma is not None:
                nv("ba_sig_h", sigma[0])
                nv("ba_sig_v", sigma[1])
                nv("ba_sig_ab", sigma[2])
            acc_bias = nav.baro_acc_bias()
            if acc_bias is not None:
                nv("ba_accbia", acc_bias)
            offset = nav.local_gnss_offset()
            if offset is not None:
                nv("ba_offset", offset[0])
                nv("ba_off_sig", offset[1])

        for prefix, rpy_fn, bias_fn in (("ars", nav.rpy_ars, nav.bias_gyr_ars),
                                        ("ahr", nav.rpy_ahrs, nav.bias_gyr_ahrs)):
            rpy = rpy_fn()
            if rpy is None:
                continue
            nv(prefix + "_roll", rpy[0] * _DEG)
            nv(prefix + "_pitch", rpy[1] * _DEG)
            nv(prefix + "_yaw", rpy[2] * _DEG)
            bias = bias_fn()
            if bias is not None:
                nv(prefix + "_bz", bias[2] * _DEG)

        sd = nav.stddev()
        if sd is not None:
            nv("i3d_sig_h", max(sd["pos_ned"][0], sd["pos_ned"][1]))
            nv("i3d_sig_v", sd["pos_ned"][2])
            nv("i3d_sig_s", max(sd["vel_ned"]))
            nv("i3d_sig_y", sd["rpy"][2] * _DEG)

        if blocked is not None and blocked != self._last_blocked:
            self._last_blocked = blocked
            self.statustext(text, "info" if blocked <= BLOCKED_COASTING
                            else "warning")

    def statustext(self, text, level="info"):
        severity = {"emergency": 0, "alert": 1, "critical": 2, "error": 3,
                    "warning": 4, "notice": 5, "info": 6, "debug": 7
                    }.get(str(level).lower(), 6)
        payload = str(text).encode("ascii", "replace")[:50]
        self.conn.mav.statustext_send(severity, payload)


def _f(x):
    """MAVLink wants finite floats; substitute 0.0 for NaN/Inf."""
    return x if math.isfinite(x) else 0.0


MODE_CODES = {"NONE": 0, "ATTITUDE_ONLY": 1, "COASTING": 2, "FULL": 3}


def state_to_pj_tree(st, stddev=None):
    """Flatten a ins ``State`` into a nested dict for PlotJuggler.

    Angles are published in degrees (nicer axes in PlotJuggler); everything
    else keeps SI units. NaN leaves are dropped downstream by the sender.
    ``mode`` is emitted as a numeric code so it can be plotted.

    ``stddev`` is the optional ins.Navigator.stddev() dict (same one
    MAVLink's EKF_STATUS_REPORT already consumes). When given, its
    pos/vel/attitude 1-sigma values are added under "full3d/sigma"
    (ins's own error-state covariance, grouped separately from the
    arbitrated "best available" fields above it) so the filter's own
    uncertainty is visible in PlotJuggler, not just the point estimate.
    """
    tree = {
        "INSLIB": {
            "ready": 1.0 if st.ready else 0.0,
            "mode": float(MODE_CODES.get(getattr(st, "mode", "NONE"), 0)),
            "dr_ms": float(st.dr_ms),
            "pos_ned": {"n": st.x_m, "e": st.y_m, "d": st.z_m},
            "vel_ned": {"n": st.vx_mps, "e": st.vy_mps, "d": st.vz_mps},
            "global": {"lat_deg": math.degrees(st.lat_rad),
                       "lon_deg": math.degrees(st.lon_rad),
                       "alt_m": st.alt_m},
            "att_deg": {"roll": math.degrees(st.roll_rad),
                        "pitch": math.degrees(st.pitch_rad),
                        "yaw": math.degrees(st.yaw_rad)},
            "rate_dps": {"roll": math.degrees(st.roll_rate_rps),
                         "pitch": math.degrees(st.pitch_rate_rps),
                         "yaw": math.degrees(st.yaw_rate_rps)},
            "acc_n": {"x": st.ax_mps2, "y": st.ay_mps2, "z": st.az_mps2},
        }
    }
    if stddev is not None:
        pos_sd, vel_sd, rpy_sd = stddev["pos_ned"], stddev["vel_ned"], stddev["rpy"]
        tree["INSLIB"]["full3d"] = {"sigma": {
            "pos_ned_m": {"n": pos_sd[0], "e": pos_sd[1], "d": pos_sd[2]},
            "vel_ned_mps": {"n": vel_sd[0], "e": vel_sd[1], "d": vel_sd[2]},
            "att_deg": {"roll": math.degrees(rpy_sd[0]),
                        "pitch": math.degrees(rpy_sd[1]),
                        "yaw": math.degrees(rpy_sd[2])},
        }}
    return tree


class Telemetry:
    """Fan a ins ``State`` out to PlotJuggler and/or MAVLink."""

    def __init__(self, plotjuggler=True, mavlink=False,
                 pj_ip="127.0.0.1", pj_port=9870, pj_log=None,
                 mav_ip="127.0.0.1", mav_port=14550, mav_subfilter_hz=5.0):
        self.pj = (PlotJugglerSender(pj_ip, pj_port, pj_log)
                   if plotjuggler else None)
        self.mav = None
        if mavlink:
            try:
                self.mav = MavlinkSender(mav_ip, mav_port, mav_subfilter_hz)
            except ImportError:
                print("[telemetry] pymavlink not installed -- MAVLink disabled "
                      "(pip install pymavlink)")

    def publish(self, state, stddev=None):
        """``stddev`` is the optional ins.Navigator.stddev() dict; MAVLink
        uses it to fill EKF_STATUS_REPORT's variances with the filter's
        real 1-sigma instead of a placeholder."""
        if self.pj is not None:
            self.pj.send(state_to_pj_tree(state, stddev))
        if self.mav is not None:
            self.mav.send(state, stddev)

    def publish_suite(self, nav, state=None):
        """Publish EVERYTHING a Navigator knows, in one call.

        This is the entry point every publisher uses, so a
        live capture and its replay put the same data on the wire: the
        arbitrated solution, every sub-filter's own output and 1-sigma,
        and the status bits (including why the 3D filter is not running).

        ``state`` may be passed in when the caller already has one
        (nav.state() is not free); otherwise it is read here.
        """
        if state is None:
            state = nav.state()
        stddev = nav.stddev()
        status, blocked, text = suite_status(nav)
        if self.pj is not None:
            self.pj.send(state_to_pj_tree(state, stddev))
            self.pj.send(subfilter_overlay_tree(nav))
        if self.mav is not None:
            self.mav.send(state, stddev, status)
            self.mav.send_subfilters(nav, status, blocked, text)

    def publish_extra(self, tree):
        """Send an arbitrary nested dict to PlotJuggler only (e.g. a
        ground-truth 'ref' overlay or raw 'meas' measurements). No-op if
        PlotJuggler is disabled."""
        if self.pj is not None and tree:
            self.pj.send(tree)

    def statustext(self, text, level="info"):
        if self.mav is not None:
            self.mav.statustext(text, level)

    def close(self):
        if self.pj is not None:
            self.pj.close()
