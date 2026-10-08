#!/usr/bin/env python3
"""Shared replay core of tools/replay.py and tools/inspostgui.py.

One implementation of everything the two front ends have in common, so a
feature added to the replay reaches both and they cannot drift apart:

* load_inputs(): the reference (moved onto its time of validity), the
  aiding fixes (gnss.csv, synthesized from the reference, or none), the
  simulated GNSS outage cut, every optional sensor stream (mag, baro,
  speed, heading, ranges) and the initial gyro bias,
* make_navigator(): the Navigator with its Config and every runtime
  setter the config.yaml schema maps to,
* Replay: the IMU-driven feed loop (IMU epoch, the most recent fix,
  free-inertial declaration, mag, baro, speed, heading, ranges, then
  update), the scoring against the reference, the inputs of the Dr. INS
  findings, and per-epoch callbacks into observer objects,
* PlotRecorder, TrackRecorder, KmlRecorder: the recorders behind the
  ins_plots pages, the North-East map and the KML/map track,
* Replay.report() and friends: the summary text both front ends show.

What stays in the front ends is only what is specific to them: the
command line, telemetry, the CSV dumps and the plot/KML writers in
replay.py, the Qt thread, the live views and stop/pause in inspostgui.py.

tools/replay.c stays the regression reference, this module only has to
agree with it.

(c) Jan Zwiener (jan@zwiener.org)
"""

import contextlib
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "python"))
import replay as rp  # noqa: E402  (loaders, build_config, Stat, ...)
from INSLIB import Navigator, ecef_to_llh  # noqa: E402
from INSLIB.telemetry import isa_pressure_to_altitude, suite_status  # noqa: E402
from geodetic_toolbox import mag_heading  # noqa: E402

US_PER_SEC = 1_000_000


class ReplayError(Exception):
    """A dataset or option the replay cannot run with. replay.py turns it
    into its exit message, inspostgui.py into an error dialog."""


# ============================================================================
# Inputs
# ============================================================================

def parse_gnss_outages(specs):
    """START:DURATION strings (seconds from the first IMU sample) ->
    [(start, end)]. Each entry of `specs` may itself hold several windows
    separated by commas or whitespace, which is how the GUI's single text
    field passes them."""
    windows = []
    for spec in specs:
        for w in re.split(r"[,\s]+", spec.strip()):
            if not w:
                continue
            try:
                a_s, d_s = w.split(":")
                start, dur = float(a_s), float(d_s)
            except ValueError:
                raise ReplayError(
                    f"--gnss-outage: expected START:DURATION, got {w!r}") from None
            if dur <= 0:
                raise ReplayError(
                    f"--gnss-outage: duration must be > 0, got {w!r}")
            windows.append((start, start + dur))
    return windows


class Inputs:
    """Everything load_inputs() read for one replay. Attributes:

    imu_path, ref, ref_delay_ms, aiding, fixes, mags, baros, speeds,
    headings, ranges, first_imu_us, ref0, gyr_bias, bias_note,
    outage_windows, n_outage_cut."""


def load_inputs(spec, data_dir, gnss_outages=(), log=print):
    """Load every input stream of a dataset the way the replay feeds it.
    `gnss_outages` is a list of (start, end) windows in seconds from the
    first IMU sample (parse_gnss_outages()), whose fixes are cut out."""
    inp = Inputs()
    inp.imu_path = imu_path = rp.input_path(data_dir, spec, "imu")
    ref_path = rp.input_path(data_dir, spec, "ref")
    if not os.path.exists(imu_path):
        raise ReplayError(f"dataset missing under {data_dir}")

    # The reference is optional: without ref.csv nothing is scored and the
    # reference traces stay empty. Only the modes that take their state from
    # it (aiding/init: ref) cannot do without.
    if os.path.exists(ref_path):
        ref = rp.load_ref(ref_path)
        if not ref:
            raise ReplayError(f"no reference epochs in {ref_path}")
    else:
        if spec["aiding"] == "ref" or spec["init"] == "ref":
            raise ReplayError(f"{ref_path} missing, but aiding: ref / "
                              "init: ref take their state from it")
        ref = []
    inp.ref = ref

    # Move every reference row onto its own time of validity (REQ-VER-030),
    # the same shift tools/replay.c applies at load, so one config.yaml
    # scores the same in every harness.
    ref_delay_ms = float(spec["score"].get("ref_delay_ms", 0.0))
    inp.ref_delay_ms = ref_delay_ms
    if ref_delay_ms:
        if spec["aiding"] == "ref":
            raise ReplayError(f"{data_dir}: score: ref_delay_ms cannot be "
                              "used with aiding: ref")
        shift_us = int(ref_delay_ms * 1000.0)
        for r in ref:
            r["t_us"] -= shift_us
        log(f"reference time of validity: {ref_delay_ms:.0f} ms earlier "
            "than its timestamps (score: ref_delay_ms)")

    first = next(rp.iter_imu(imu_path), None)
    if first is None:
        raise ReplayError(f"no IMU samples in {imu_path}")
    inp.first_imu_us = first_imu_us = first[0]

    gnss_cfg = spec["gnss"]
    inp.aiding = aiding_mode = spec["aiding"]
    var_hor = gnss_cfg["pos_stddev_fallback_m"][0] ** 2
    var_ver = gnss_cfg["pos_stddev_fallback_m"][1] ** 2
    var_vel = gnss_cfg["vel_stddev_fallback_mps"] ** 2
    inp.outage_windows = list(gnss_outages)
    inp.n_outage_cut = 0
    if aiding_mode == "gnss":
        gnss_path = rp.input_path(data_dir, spec, "gnss")
        fixes = rp.load_gnss(gnss_path) if os.path.exists(gnss_path) else None
        if not fixes:
            raise ReplayError(f"aiding: gnss but no usable {gnss_path}")
        if inp.outage_windows:
            # Cut the fixes out rather than telling the filter to ignore
            # them. An outage is the ABSENCE of data, and anything the
            # filter would otherwise still learn from a "rejected" fix
            # (that one arrived at all, its covariance, its time) would
            # make the test easier than reality.
            #
            # Same origin as the evaluation window, which counts from the
            # first IMU sample. Anchoring the cut on the first FIX instead
            # would silently offset the two by however long the receiver
            # took to produce one, and the evaluation window would then
            # not be inside the outage it claims to measure.
            kept = [fx for fx in fixes
                    if not any(lo <= (fx["t_us"] - first_imu_us) / US_PER_SEC < hi
                               for lo, hi in inp.outage_windows)]
            inp.n_outage_cut = len(fixes) - len(kept)
            fixes = kept
            if not fixes:
                raise ReplayError("--gnss-outage removed every fix")
            log("simulated GNSS outage: %s -> %d of %d fixes dropped"
                % (", ".join("%.0f..%.0f s" % w for w in inp.outage_windows),
                   inp.n_outage_cut, inp.n_outage_cut + len(kept)))
        for fx in fixes:
            fx["cov_pos"] = rp.apply_fallback(rp.cov6_to_rows(fx["cov_pos"]),
                                              var_hor, var_ver)
            fx["cov_vel"] = (rp.apply_fallback(rp.cov6_to_rows(fx["cov_vel"]),
                                               var_vel, var_vel)
                             if fx["vel_ok"] else None)
    elif aiding_mode == "ref":
        # Synthesize the fix from the reference. Noise is
        # pos_stddev_fallback_m/vel_stddev_fallback_mps, the same fields a
        # real fix's zero/unknown covariance diagonals fall back to. This
        # synthesized fix never has a reported covariance either.
        pos_cov = [[var_hor, 0, 0], [0, var_hor, 0], [0, 0, var_ver]]
        vel_cov = [[var_vel if i == j else 0
                    for j in range(3)] for i in range(3)]
        fixes = [{
            "t_us": r["t_us"], "lat_rad": r["lat_rad"],
            "lon_rad": r["lon_rad"], "h_m": r["h_m"],
            "cov_pos": pos_cov, "vel_ned": r["vel_ned"],
            "cov_vel": vel_cov, "vel_ok": True,
        } for r in ref]
    elif aiding_mode == "none":
        # No absolute position aiding at all. ins never gets a fix, so it
        # stays uninitialized (nav_suite mode NONE/ATTITUDE_ONLY) forever,
        # but the ARS and baro_alt filters don't need one. They bootstrap
        # from the accelerometer/first barometer sample and keep running
        # regardless (see nav_suite.h).
        fixes = []
    else:
        raise ReplayError(f"{data_dir}: unknown aiding mode {aiding_mode!r} "
                          f"(expected gnss/ref/none)")
    inp.fixes = fixes

    def stream(section, name, loader):
        if not int(spec[section]["enable"]):
            return []
        rows = loader(rp.input_path(data_dir, spec, section))
        if not rows:
            raise ReplayError(f"{section}: enable but no usable {name} "
                              f"in {data_dir}")
        return rows

    inp.mags = stream("mag", "mag.csv", rp.load_txyz)
    inp.baros = stream("baro", "baro.csv", rp.load_baro)
    inp.speeds = stream("speed", "speed.csv", rp.load_speed)
    inp.headings = stream("heading", "heading.csv", rp.load_heading)
    inp.ranges = stream("ranges", "ranges.csv", rp.load_ranges)

    # init: ref uses the reference epoch aligned with the FIRST IMU sample,
    # not ref[0]. ins defers a prescribed init to its actual start epoch
    # and stamps the provided state THERE without propagating it
    # (REQ-NAV-033). Handing it an older truth epoch bakes a permanent
    # -v*dt position offset into a moving start, which then reads as
    # filter inaccuracy.
    if ref:
        inp.ref0 = next((r for r in ref if r["t_us"] >= first_imu_us), ref[0])
    else:
        # Neutral stand-in, only read by the aiding: none seed and by
        # init: ref (refused above)
        inp.ref0 = {"t_us": first_imu_us, "lat_rad": 0.0, "lon_rad": 0.0,
                    "h_m": 0.0, "roll_rad": 0.0, "pitch_rad": 0.0,
                    "yaw_rad": 0.0, "vel_ned": (0.0, 0.0, 0.0)}
    # Where the dataset was recorded, for the magnetic reference field
    site = ref[0] if ref else (fixes[0] if fixes else None)
    inp.site_llh = ((site["lat_rad"], site["lon_rad"]) if site else None)

    gyr_bias = rp.estimate_gyro_bias(imu_path, spec["gyro_bias_window_sec"])
    inp.bias_note = "from initial window" if gyr_bias else "n/a"
    if gyr_bias is not None and spec["init"] == "ref":
        # With init: ref the reference provides position, velocity AND
        # attitude, so the non-bias content of the window average can be
        # removed (see correct_initial_gyr_bias).
        gyr_bias = rp.correct_initial_gyr_bias(
            gyr_bias, ref, first_imu_us, spec["gyro_bias_window_sec"])
        inp.bias_note += ", ref motion + earth/transport rate removed"
    inp.gyr_bias = gyr_bias
    return inp


def log_startup(spec, inp, log=print):
    """The settings summary printed before the replay starts."""
    leverarm = list(spec["gnss"]["leverarm_frd"])
    gnss_delay_ms = int(spec["gnss"].get("delay_ms", 0.0))
    if not spec["gnss"]["enable"]:
        log("gnss: enable 0 - the fixes are read and counted, but none of "
            "them aids the filter")
    log(f"dataset {spec['name']}: aiding={spec['aiding']}, "
        f"init={spec['init']}, warmup {spec['score']['warmup_sec']:g} s, "
        f"leverarm FRD {leverarm}"
        f"{', automotive' if spec['automotive_mode'] else ''}"
        f"{', mag' if inp.mags else ''}{', baro' if inp.baros else ''}"
        f"{f', gnss delay {gnss_delay_ms} ms' if gnss_delay_ms else ''}")
    log(f"initial gyro bias: "
        f"{[round(math.degrees(b), 4) for b in (inp.gyr_bias or (0, 0, 0))]} deg/s "
        f"({inp.bias_note})")
    rp.print_chi2_gate(spec, out=log)
    notes = rp.notable_settings(spec)
    if notes:
        log("notable settings (non-default):")
        for n in notes:
            log(f"  - {n}")
    else:
        log("notable settings: none (all defaults)")


def make_navigator(spec, inp, log=print):
    """The Navigator for this replay: Config from build_config() plus the
    runtime setters, all before the first sample as nav_suite requires."""
    ref0 = inp.ref0
    # fixes[0] is just a provisional seed for auto_init (overridden by the
    # first real fix). With aiding: none there is none, so fall back to
    # the reference epoch. Harmless since ins then never leaves
    # is_collecting anyway.
    t0 = ref0 if (spec["init"] == "ref" or not inp.fixes) else inp.fixes[0]
    nav = Navigator(rp.build_config(spec, ref0, t0["t_us"], t0["lat_rad"],
                                    t0["lon_rad"], t0["h_m"], inp.gyr_bias))
    # baro_alt / ARS/AHRS noise model (0 -> each filter's own default, the
    # ARS/AHRS gyro terms first fall back to imu:, see effective_ahrs_cfg).
    # Has to be set before the first baro sample / ins_suite_update()
    # latches the respective template (nav_suite contract).
    baro_cfg = spec["baro"]
    ahrs_cfg = rp.effective_ahrs_cfg(spec)
    nav.set_baro_acc_bias_drift(float(baro_cfg.get("acc_bias_rw", 0.0)))
    nav.set_baro_acc_noise(float(baro_cfg.get("acc_noise_mps2_sqrthz", 0.0)))
    nav.set_baro_acc_bias_init_stddev(float(baro_cfg.get("acc_bias_init_mps2", 0.0)))
    nav.set_baro_h_process_noise(float(baro_cfg.get("h_process_noise", 0.0)))
    nav.set_local_gnss_rw_stddev(float(baro_cfg.get("local_gnss_rw_stddev_mps", 0.0)))
    nav.set_local_gnss_chi2_threshold(float(baro_cfg.get("local_gnss_chi2_threshold", 0.0)))
    nav.set_local_gnss_min_update_interval(
        float(baro_cfg.get("local_gnss_min_update_interval_sec", 0.0)))
    nav.set_local_gnss_stddev_inflation(
        float(baro_cfg.get("local_gnss_stddev_inflation_factor", 0.0)))
    nav.set_ahrs_gyr_noise(float(ahrs_cfg.get("gyr_noise_psd", 0.0)))
    nav.set_ahrs_acc_noise(float(ahrs_cfg.get("acc_noise_mps2", 0.0)))
    nav.set_ahrs_gyr_bias_rw(float(ahrs_cfg.get("gyr_bias_rw", 0.0)))
    nav.set_ahrs_rpy_pred_stddev(float(ahrs_cfg.get("rpy_pred_stddev_rad_sqrts", 0.0)))
    nav.set_ahrs_gyr_bias_init_stddev(
        math.radians(float(ahrs_cfg.get("gyr_bias_init_stddev_rps_deg", 0.0))))
    init_hint = spec["init_hint"]
    nav.set_init_att_hint(
        math.radians(init_hint["roll_deg"]), math.radians(init_hint["pitch_deg"]),
        math.radians(init_hint["rpy_stddev_deg"]), math.radians(init_hint["yaw_deg"]),
        math.radians(init_hint["yaw_stddev_deg"]))
    # No set_auto_zaru() here. The ARS/AHRS stillness fallback
    # (REQ-AHRS-017) is armed by default and configured through the same
    # imu.auto_zupt_* set as ins (REQ-SUITE-020, already in the Config
    # above). Calling the runtime override here would silently re-arm a
    # dataset that opted out with imu.auto_zupt_velocity_blind_disable.
    #
    # Armed by default is what a real capture needs. The ARS/AHRS ZARU and,
    # through it, baro_alt's vertical ZUPT are decided on the IMU alone,
    # which is available from the first samples, whereas the trigger
    # propagated from ins (REQ-SUITE-009) cannot answer until ins has
    # initialized.
    mag_cfg = spec["mag"]
    if inp.mags and float(mag_cfg["wmm_year"]) > 0 and inp.site_llh is None:
        log("  WARNING: no reference and no fix to locate the dataset, no"
            " magnetic reference field is built and the magnetometer will"
            " not be fused")
    elif inp.mags and float(mag_cfg["wmm_year"]) > 0:
        # Both attitude sources agree on true north (WMM declination).
        nav.set_magnetic_model(inp.site_llh[0], inp.site_llh[1],
                               float(mag_cfg["wmm_year"]))
    elif inp.mags:
        # Same up-front warning as tools/replay.c. Without an epoch there is
        # no reference field and every magnetometer sample is rejected.
        log("  WARNING: mag.enable is set but mag.wmm_year is missing, no"
            " magnetic reference field is built and the magnetometer will"
            " not be fused (inslib_convert_ubx_to_csv.py derives it"
            " from NAV-PVT)")
    return nav


# ============================================================================
# Replay loop
# ============================================================================

class Pacer:
    """Paces the replay to wall-clock time (x `speed`). A pause moves the
    wall-clock anchor along with it (shift()), so the replay picks up
    where it stopped instead of racing to catch up."""

    def __init__(self, speed=1.0):
        self.speed = max(speed, 1e-6)
        self.wall0 = time.perf_counter()

    def shift(self, sec):
        self.wall0 += sec

    def wait(self, t_rel_sec, should_stop=None):
        target = self.wall0 + t_rel_sec / self.speed
        if should_stop is None:
            sleep = target - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            return
        # Interruptible: a stop request is answered within 0.2 s even at
        # a very slow speed factor.
        while not should_stop():
            sleep = target - time.perf_counter()
            if sleep <= 0:
                break
            time.sleep(min(sleep, 0.2))


def ins_pos_in_replay_frame(nav, r):
    """nav.position_local() in the replay's local frame, the one latched
    once at r.origin_ecef that the reference and the fixes are drawn in.

    ins sets a new origin of its own local frame when it restarts (after a
    time jump, for example), its position_local() then starts again near
    zero. Drawn as it is, the whole estimate after the restart would sit
    shifted by the distance between the two origins (REQ-VER-041). The
    shift is formed with ref_to_local_ned(), the same mapping the
    reference gets. None while ins has no position."""
    pos = nav.position_local()
    if pos is None or r.origin_ecef is None:
        return pos
    o = nav.origin_ecef()
    if o is None or max(abs(o[i] - r.origin_ecef[i]) for i in range(3)) < 1e-3:
        return pos
    lat, lon, h = ecef_to_llh(*o)
    shift = rp.ref_to_local_ned({"lat_rad": lat, "lon_rad": lon, "h_m": h},
                                r.origin_ecef, r.origin_lat, r.origin_lon)
    return [pos[i] + shift[i] for i in range(3)]


class Observer:
    """Per-epoch callback interface. on_epoch() runs once per IMU epoch
    after nav.update() and the replay's own bookkeeping, with the Replay
    itself as the argument (its attributes are the epoch's state: t, a, g,
    fix_now, ref_now, mag_now, baro_now, last_ref, last_fix, last_mag,
    last_baro, origin_ecef/lat/lon/h, nav). on_pos_error() is called for
    every scored position error (ECEF, metres), on_finish() once after
    the loop."""

    def on_epoch(self, r):
        pass

    def on_pos_error(self, r, err_ecef):
        pass

    def on_finish(self, r):
        pass


class Replay:
    """One replay of a dataset through `nav`. Construct, run(), then read
    the results off the attributes or report()/findings()."""

    def __init__(self, spec, inp, nav, observers=(), eval_start=None,
                 eval_end=None, rate_bucket_sec=5.0):
        self.spec, self.inp, self.nav = spec, inp, nav
        self.observers = list(observers)
        self.eval_start, self.eval_end = eval_start, eval_end
        self.fixes, self.ref = inp.fixes, inp.ref
        self.site_llh = inp.site_llh
        self.mags, self.baros = inp.mags, inp.baros
        self.speeds, self.headings, self.ranges = inp.speeds, inp.headings, inp.ranges
        self.aiding = spec["aiding"]
        gnss_cfg = spec["gnss"]
        self.gnss_delay_ms = int(gnss_cfg.get("delay_ms", 0.0))
        self.leverarm = tuple(gnss_cfg["leverarm_frd"])
        self.score_la = tuple(spec["score"].get("leverarm_frd") or (0.0, 0.0, 0.0))
        mag_cfg = spec["mag"]
        # 0 is handed straight to the library, which reads an unset
        # variance as "use my magnetometer default" (src/sensor_defaults.h).
        # The standstill accuracy check is fed the same 0 on purpose: with
        # nothing configured there is no configured stddev to call
        # optimistic.
        self.mag_sd = float(mag_cfg["stddev_ut"])
        # The fixed calibration as the library gets it (the mag samples
        # themselves stay raw: nav.mag() is handed the raw sample and ins.c
        # calibrates it, applying it here as well would correct twice).
        # Anything the replay derives from the mag stream on its own goes
        # through mag_calibrate() with these.
        self.mag_misalign = rp.mounted_calibration(spec)[2]
        self.mag_bias_cfg = tuple(float(v) for v in mag_cfg["fixed_bias"])
        self.mag_cal_active = any(self.mag_misalign) or any(self.mag_bias_cfg)
        self.noise = spec["imu"]

        (self.n_imu_total, self.imu_duration, self.imu_hz, self.imu_max_gap,
         self.imu_max_gap_at, self.gyr_d2_stat, self.acc_d2_stat,
         self.gyr_d3_stat, self.acc_d3_stat,
         self.rate_t0, imu_rate_t, self.imu_rate_hz) = rp._imu_prepass(
             inp.imu_path, rate_bucket_sec)
        # Per-stream sampling rate over time for the sensor-rate page, all
        # sharing rate_t0 (the first IMU sample) as their time origin, the
        # same as the recorders' "t".
        self.sensor_rate = {"imu": (imu_rate_t, self.imu_rate_hz)}
        if self.aiding == "gnss" and self.fixes:
            self.sensor_rate["gnss"] = rp._bucketed_rate(
                [fx["t_us"] for fx in self.fixes], self.rate_t0, rate_bucket_sec)
        if self.mags:
            self.sensor_rate["mag"] = rp._bucketed_rate(
                [m[0] for m in self.mags], self.rate_t0, rate_bucket_sec)
        if self.baros:
            self.sensor_rate["baro"] = rp._bucketed_rate(
                [b[0] for b in self.baros], self.rate_t0, rate_bucket_sec)

        self.t_warmup_end = ((self.fixes[0] if self.fixes else inp.ref0)["t_us"]
                             + int(spec["score"]["warmup_sec"] * US_PER_SEC))
        self.e_roll, self.e_pitch, self.e_yaw = rp.Stat(), rp.Stat(), rp.Stat()
        self.e_pos = rp.Stat()
        self.e_height_ell = rp.Stat()  # nav.height_ellipsoid() vs ref_now["h_m"]

        # Epoch state, read by the observers.
        self.t = self.t0_us = None
        self.a = self.g = None
        self.fix_now = self.ref_now = self.mag_now = self.baro_now = None
        self.last_ref = self.last_fix = self.last_mag = self.last_baro = None
        self.origin_ecef = None
        self.origin_lat = self.origin_lon = self.origin_h = 0.0

        # Counters
        self.n_imu = 0
        self.ifix = 0
        self.n_static = 0
        self.n_speed_fused = 0
        self.heading_reasons = {}
        self.n_ranges_offered = self.n_ranges_carried = 0
        self.n_fi_offers = 0
        self.aborted = False

        # GNSS accuracy self-check: the GNSS fixes that land during an
        # ins-detected standstill, grouped into contiguous standstill
        # phases, to compare their empirical spread against the receiver's
        # own reported covariance (see gnss_standstill_accuracy). The same
        # for the raw baro/mag channels (channel_standstill_accuracy).
        self.gnss_static_phases, self._gnss_cur_phase = [], []
        self.baro_static_phases, self._baro_cur_phase = [], []
        self.mag_static_phases, self._mag_cur_phase = [], []

        # Filter self-consistency watchdog: the filter's OWN reported 1-sigma
        # (attitude + position) right after a fused GNSS fix, but only past
        # the warmup and while GNSS is streaming (the previous used fix is
        # still recent). So the uncertainty growth during initial
        # convergence, a GNSS outage, or the re-acquisition transient after
        # one is excluded, only steady, healthy operation is judged against
        # STDDEV_LIMITS.
        self.health_stats = {k: {"worst": 0.0, "t": 0.0, "n_exceed": 0}
                             for k in rp.STDDEV_LIMITS}
        self.health_n = 0
        # Streaming grace: the previous fix counts as "recent" within a few
        # nominal GNSS periods (>= 2 s), so a slow receiver isn't
        # permanently treated as gapped, and a real gap (>= this)
        # suppresses the sample.
        self._gnss_grace_us = 2 * US_PER_SEC
        if self.fixes and len(self.fixes) > 1:
            nominal_sec = ((self.fixes[-1]["t_us"] - self.fixes[0]["t_us"])
                           / US_PER_SEC / (len(self.fixes) - 1))
            self._gnss_grace_us = int(max(2.0, 3.0 * nominal_sec) * US_PER_SEC)

    # ------------------------------------------------------------------
    def run(self, progress=None, before_epoch=None, pacer=None,
            should_stop=None):
        """Replay the whole IMU stream. progress(n_imu) is called once per
        epoch, before_epoch() before each one (returning False aborts the
        run, self.aborted), pacer (a Pacer) paces it to wall-clock time
        and should_stop() lets its waits end early."""
        spec, nav = self.spec, self.nav
        fixes, ref = self.fixes, self.ref
        mags, baros, speeds = self.mags, self.baros, self.speeds
        headings, ranges = self.headings, self.ranges
        observers = self.observers
        noise = self.noise
        acc_var = (noise["acc_psd"],) * 3
        gyr_var = (noise["gyr_psd"],) * 3
        mag_var = (self.mag_sd ** 2,) * 3
        baro_sd = float(spec["baro"]["stddev_m"])
        speed_cfg = spec["speed"]
        speed_sd_cfg = float(speed_cfg["stddev_mps"])
        speed_delay_cfg = int(round(float(speed_cfg["delay_ms"])))
        heading_cfg = spec["heading"]
        heading_delay_cfg = int(round(float(heading_cfg["delay_ms"])))
        ranges_cfg = spec["ranges"]
        rng_scale = float(ranges_cfg["stddev_scale"]) or 1.0
        rng_min = float(ranges_cfg["stddev_min_m"])
        rng_lever = tuple(ranges_cfg["leverarm_frd"])
        gnss_delay_ms = self.gnss_delay_ms
        leverarm = self.leverarm
        score_la = self.score_la
        is_gnss = self.aiding == "gnss"
        heading_reasons = self.heading_reasons
        health_stats = self.health_stats

        # Optional ground-truth evaluation window, in microseconds from the
        # first IMU sample (t0_us, the printed-time origin). Only the
        # scoring below is gated, the filter still runs the whole trial.
        eval_lo_us = (int(self.eval_start * US_PER_SEC)
                      if self.eval_start is not None else None)
        eval_hi_us = (int(self.eval_end * US_PER_SEC)
                      if self.eval_end is not None else None)

        # free_inertial_start (see the offer inside the loop). Prepared
        # once: the ECEF of the declared point and the covariance of that
        # statement. gnss: enable 0 keeps the fixes loaded and counted, they
        # just never reach the filter (same key and same meaning as
        # tools/insrcv.c).
        gnss_enable = bool(spec["gnss"]["enable"])
        fi_spec = spec["free_inertial_start"]
        fi_llh = fi_cov = None
        fi_next_t_us = 0
        if fi_spec["enable"]:
            fi_llh = (math.radians(fi_spec["lat_deg"]),
                      math.radians(fi_spec["lon_deg"]),
                      float(fi_spec["height_m"]))
            fi_var = float(fi_spec["stddev_m"]) ** 2
            fi_cov = [fi_var, 0.0, 0.0, 0.0, fi_var, 0.0, 0.0, 0.0, fi_var]

        iref = imag = ibaro = ispeed = iheading = iranges = 0
        t_prev = None
        last_fix_used_us = None

        for t, g, a in rp.iter_imu(self.inp.imu_path):
            if before_epoch is not None and not before_epoch():
                self.aborted = True
                break
            if self.t0_us is None:
                self.t0_us = t
            t0_us = self.t0_us
            dt = (t - t_prev) / US_PER_SEC if t_prev is not None else 0.0
            t_prev = t
            self.n_imu += 1
            if progress is not None:
                progress(self.n_imu)
            self.t, self.a, self.g = t, a, g

            nav.imu(t, dt, a, g, acc_var, gyr_var)

            # Attach the most recent fix (real GNSS or reference-derived).
            fix_now = None
            while self.ifix < len(fixes) and fixes[self.ifix]["t_us"] <= t:
                fix_now = fixes[self.ifix]
                self.ifix += 1
            # free_inertial_start: the declared origin, offered as a
            # position measurement until ins has bootstrapped from it and
            # not one epoch longer. A source that kept repeating the same
            # point would pin the solution to it instead of dead reckoning.
            # Never in an epoch that already carries a real fix, a measured
            # position beats a declared one. Mirrors
            # fi_offer_start_position() in tools/insrcv.c.
            if (fi_llh is not None and fix_now is None
                    and nav.deadreckoning_ms() < 0 and t >= fi_next_t_us):
                fi_next_t_us = t + US_PER_SEC // 2   # 2 Hz: entry dwell wants >= 1
                nav.gnss_pos_llh(fi_llh, fi_cov)
                self.n_fi_offers += 1

            if (gnss_enable and fix_now is not None
                    and fix_now["cov_pos"] is not None):
                # REQ-VER-008: assumed fixed processing/telemetry latency,
                # applied uniformly regardless of aiding source, history-
                # anchors the fusion via ins's existing gnss_delay_ms.
                nav.gnss_pos_llh((fix_now["lat_rad"], fix_now["lon_rad"],
                                  fix_now["h_m"]),
                                 fix_now["cov_pos"], delay_ms=gnss_delay_ms)
                if fix_now["vel_ok"] and fix_now["cov_vel"] is not None:
                    nav.gnss_vel(fix_now["vel_ned"], fix_now["cov_vel"])
                nav.gnss_leverarm(leverarm)
            if fix_now is not None:
                self.last_fix = fix_now
            self.fix_now = fix_now

            # Attach the most recent magnetometer / barometer sample.
            mag_now = None
            while imag < len(mags) and mags[imag][0] <= t:
                mag_now = mags[imag]
                imag += 1
            if mag_now is not None:
                nav.mag(mag_now[1], mag_var)
                self.last_mag = mag_now
            self.mag_now = mag_now
            baro_now = None
            while ibaro < len(baros) and baros[ibaro][0] <= t:
                baro_now = baros[ibaro]
                ibaro += 1
            if baro_now is not None:
                nav.baro(baro_now[1], baro_sd)
                self.last_baro = baro_now
            self.baro_now = baro_now

            # Absolute speed (REQ-NAV-068). Only the newest sample of the
            # interval is fused. Re-fusing a value the filter has already
            # seen would count one measurement twice and make it
            # overconfident about the velocity, the same contract the
            # barometer above follows.
            speed_now = None
            while ispeed < len(speeds) and speeds[ispeed][0] <= t:
                speed_now = speeds[ispeed]
                ispeed += 1
            if speed_now is not None:
                nav.speed(speed_now[1], speed_sd_cfg, speed_delay_cfg)
                self.n_speed_fused += 1

            # Dual-antenna heading (REQ-NAV-010/087). Newest row only, same
            # contract as the speed above.
            heading_now = None
            while iheading < len(headings) and headings[iheading][0] <= t:
                heading_now = headings[iheading]
                iheading += 1
            if heading_now is not None:
                why, yaw_meas, sd_meas = rp.heading_measurement(
                    heading_cfg, heading_now, nav.rpy())
                heading_reasons[why] = heading_reasons.get(why, 0) + 1
                if why == rp.HEADING_OK:
                    nav.yaw(yaw_meas, sd_meas, heading_delay_cfg)

            # Ranges (REQ-NAV-082, REQ-VER-038): every row up to this epoch,
            # each anchored at its own age. What does not fit the filter's
            # per-epoch maximum is carried to the next epoch, as in
            # replay.c.
            k = 0
            while iranges < len(ranges) and ranges[iranges][0] <= t:
                t_r, aid, ecef, rng, sd = ranges[iranges]
                sd = sd * rng_scale
                if rng_min > 0.0 and sd < rng_min:
                    sd = rng_min
                if not nav.range(ecef, rng, sd, int((t - t_r) // 1000), aid):
                    self.n_ranges_carried += 1
                    break
                k += 1
                self.n_ranges_offered += 1
                iranges += 1
            if k:
                nav.range_leverarm(rng_lever)

            nav.update()

            standstill = nav.auto_zupt_active()
            if standstill:
                self.n_static += 1

            ref_now = None
            while iref < len(ref) and ref[iref]["t_us"] <= t:
                ref_now = ref[iref]
                iref += 1
            if ref_now is not None:
                self.last_ref = ref_now
            self.ref_now = ref_now

            # Cache the local-frame origin once the filter is up, so the
            # ground truth can be expressed in the same NED frame as the
            # estimate. A GNSS quality-loss re-bootstrap no longer moves the
            # origin, it is carried across (REQ-NAV-062). A time-jump reset
            # (REQ-NAV-016) or a bootstrap beyond the carry distance still
            # establish a new one, and refreshing the cache then would move
            # the truth frame mid-plot, which is worse. Scoring is
            # unaffected either way, it runs on absolute ECEF
            # (pos_error_ecef). The estimate is moved into this cached frame
            # instead, by ins_pos_in_replay_frame() (REQ-VER-041).
            if self.origin_ecef is None:
                origin = nav.origin_ecef()
                if origin is not None:
                    self.origin_ecef = origin
                    self.origin_lat, self.origin_lon, self.origin_h = ecef_to_llh(*origin)

            # File a fused GNSS fix that arrived this epoch under the
            # current standstill phase (ins-ZUPT-detected), or end the phase
            # when a fix arrives while moving. cov_pos None means the fix
            # was rejected and is not counted. The standstill flag is read
            # AFTER nav.update() so it reflects this epoch, and needs the
            # local origin to place the fix in NED.
            if is_gnss and fix_now is not None and fix_now["cov_pos"] is not None:
                if standstill:
                    if self.origin_ecef is not None:
                        cov_p = fix_now["cov_pos"]
                        cov_v = fix_now["cov_vel"]
                        has_vel = fix_now["vel_ok"] and cov_v is not None
                        self._gnss_cur_phase.append({
                            "pos_ned": rp.ref_to_local_ned(
                                fix_now, self.origin_ecef,
                                self.origin_lat, self.origin_lon),
                            "vel_ned": (list(fix_now["vel_ned"]) if has_vel
                                        else None),
                            "rep_pos_hor": math.sqrt((cov_p[0][0] + cov_p[1][1]) / 2.0),
                            "rep_pos_ver": math.sqrt(cov_p[2][2]),
                            "rep_vel_hor": (math.sqrt((cov_v[0][0] + cov_v[1][1]) / 2.0)
                                            if has_vel else None),
                            "rep_vel_ver": (math.sqrt(cov_v[2][2]) if has_vel
                                            else None),
                        })
                elif self._gnss_cur_phase:  # moving fix closes the phase
                    self.gnss_static_phases.append(self._gnss_cur_phase)
                    self._gnss_cur_phase = []

            # File the baro/mag samples that arrived this epoch under the
            # current standstill, or close the phase once the platform
            # moves. The mag field in body frame is only constant if the
            # platform isn't ROTATING, so it is gated on the zero-angular-
            # rate detector too.
            #
            # baro uses the WIDER vertical_zupt_active() rather than
            # auto_zupt_active(). Without absolute position aiding (aiding:
            # none) ins never initializes, so auto_zupt_active() stays false
            # for the whole replay and the baro accuracy check would never
            # fire even on a dataset that never moves.
            # vertical_zupt_active() folds in the ARS/AHRS velocity-blind
            # fallback, the only stillness detector available in that mode
            # (see nav_suite.c, REQ-SUITE-015).
            baro_standstill = nav.vertical_zupt_active()
            if baros:
                if baro_standstill and baro_now is not None:
                    self._baro_cur_phase.append(
                        (isa_pressure_to_altitude(baro_now[1]),))
                elif not baro_standstill and self._baro_cur_phase:
                    self.baro_static_phases.append(self._baro_cur_phase)
                    self._baro_cur_phase = []
            if mags:
                mag_still = standstill and nav.zaru_active()
                if mag_still and mag_now is not None:
                    self._mag_cur_phase.append(tuple(mag_now[1]))
                elif not mag_still and self._mag_cur_phase:
                    self.mag_static_phases.append(self._mag_cur_phase)
                    self._mag_cur_phase = []

            # Filter self-consistency watchdog: on a fused fix, judge the
            # filter's freshly-corrected 1-sigma against STDDEV_LIMITS, but
            # only past the warmup, while streaming (prev fix recent) and
            # while actually moving. Standstill is excluded because yaw is
            # unobservable there without a magnetometer, so a growing yaw
            # sigma is expected, not a fault.
            if fix_now is not None and fix_now["cov_pos"] is not None:
                streaming = (last_fix_used_us is not None
                             and (t - last_fix_used_us) <= self._gnss_grace_us)
                last_fix_used_us = t
                sd = nav.stddev() if nav.is_ready() else None
                if (streaming and t >= self.t_warmup_end and sd is not None
                        and not nav.auto_zupt_active()):
                    self.health_n += 1
                    t_sec = (t - t0_us) / US_PER_SEC
                    vals = {"roll": math.degrees(sd["rpy"][0]),
                            "pitch": math.degrees(sd["rpy"][1]),
                            "yaw": math.degrees(sd["rpy"][2]),
                            "pos": math.sqrt(sum(x * x for x in sd["pos_ned"]))}
                    for key, v in vals.items():
                        h = health_stats[key]
                        if v > h["worst"]:
                            h["worst"], h["t"] = v, t_sec
                        if v > rp.STDDEV_LIMITS[key]:
                            h["n_exceed"] += 1

            for obs in observers:
                obs.on_epoch(self)

            # Score against the ground truth after the warmup (same metrics
            # as the C harness: attitude mean/std, position rms), optionally
            # further restricted to the evaluation window.
            rel_us = t - t0_us
            in_eval_window = ((eval_lo_us is None or rel_us >= eval_lo_us)
                              and (eval_hi_us is None or rel_us <= eval_hi_us))
            if (ref_now is not None and nav.is_ready() and t >= self.t_warmup_end
                    and in_eval_window):
                err = rp.pos_error_ecef(nav, ref_now, score_la)
                if err is not None:
                    self.e_pos.add(math.sqrt(sum(e * e for e in err)))
                    for obs in observers:
                        obs.on_pos_error(self, err)
                rpy = nav.rpy_ins()
                if rpy is not None:
                    self.e_roll.add(math.degrees(rp.wrap_pi(rpy[0] - ref_now["roll_rad"])))
                    self.e_pitch.add(math.degrees(rp.wrap_pi(rpy[1] - ref_now["pitch_rad"])))
                    self.e_yaw.add(math.degrees(rp.wrap_pi(rpy[2] - ref_now["yaw_rad"])))
            # nav_suite_get_height_ellipsoid() vs ref_now["h_m"] directly
            # (absolute, like the C harness's ell_h). Deliberately NOT gated
            # on nav.is_ready() like the block above: the accessor still
            # succeeds during COASTING/ATTITUDE_ONLY (baro_alt + the offset
            # filter's estimate), which is exactly the case worth scoring.
            if ref_now is not None and t >= self.t_warmup_end and in_eval_window:
                h_ell = nav.height_ellipsoid()
                if h_ell is not None:
                    # REQ-VER-037: the reference point's height, not the IMU's
                    self.e_height_ell.add(h_ell - rp.ref_point_offset_ned(nav, score_la)[2]
                                          - ref_now["h_m"])

            if pacer is not None:
                pacer.wait((t - t0_us) / US_PER_SEC, should_stop)

        # Close the standstill phases still open at EOF.
        if self._gnss_cur_phase:
            self.gnss_static_phases.append(self._gnss_cur_phase)
            self._gnss_cur_phase = []
        if self._baro_cur_phase:
            self.baro_static_phases.append(self._baro_cur_phase)
            self._baro_cur_phase = []
        if self._mag_cur_phase:
            self.mag_static_phases.append(self._mag_cur_phase)
            self._mag_cur_phase = []
        for obs in observers:
            obs.on_finish(self)

    # ------------------------------------------------------------------
    # Results
    # ------------------------------------------------------------------
    def report(self, log=print):
        """The summary after the run: counts, sub-filter state, final
        1-sigma, accuracy vs. the reference and the data quality section."""
        spec, nav = self.spec, self.nav
        diag = nav.diag()
        mags, baros, speeds = self.mags, self.baros, self.speeds
        headings, ranges, fixes = self.headings, self.ranges, self.fixes
        log(f"\nreplayed {self.n_imu} IMU samples, {len(fixes)} fixes, "
            f"{len(self.ref)} reference epochs"
            f"{f', {len(mags)} mag' if mags else ''}"
            f"{f', {len(baros)} baro' if baros else ''}"
            f"{f', {len(speeds)} speed' if speeds else ''}"
            f"{f', {len(headings)} heading' if headings else ''}"
            f"{f', {len(ranges)} ranges' if ranges else ''}")
        if self.n_fi_offers:
            fi_spec = spec["free_inertial_start"]
            log(f"free-inertial start: {self.n_fi_offers} declaration(s) offered at "
                f"{fi_spec['lat_deg']:.7f} {fi_spec['lon_deg']:.7f} "
                f"h={fi_spec['height_m']:.1f} m +-{fi_spec['stddev_m']:g} m, "
                f"then the filter is on its own")
        if headings:
            hr = self.heading_reasons
            log(f"heading aiding: {len(headings)} rows, "
                f"{hr.get(rp.HEADING_OK, 0)} offered as yaw, "
                f"{hr.get(rp.HEADING_NOT_FIXED, 0)} not fixed, "
                f"{hr.get(rp.HEADING_BAD_STDDEV, 0)} bad 1-sigma, "
                f"{hr.get(rp.HEADING_BAD_GEOMETRY, 0)} refused by the "
                f"baseline geometry")
        if ranges:
            log(f"range aiding: {len(ranges)} rows, {self.n_ranges_offered} offered "
                f"({self.n_ranges_carried} epochs full), {diag.get('n_range_used', 0)} fused, "
                f"{diag.get('n_range_rejected', 0)} rejected, "
                f"{diag.get('n_range_skipped', 0)} skipped, "
                f"{diag.get('n_range_pos_aiding', 0)} epochs counted as position aiding")
        if speeds:
            log(f"speed aiding: {self.n_speed_fused} samples offered, "
                f"{diag.get('n_speed_used', 0)} fused, "
                f"{diag.get('n_speed_skipped', 0)} skipped below the speed gate, "
                f"last residual {diag.get('last_speed_residual_mps', 0.0):+.3f} m/s")
        log(f"ins: {diag['n_predict']} predicts, {diag['n_gnss_used']} gnss "
            f"fusions ({diag['n_gnss_seen']} seen), {diag['n_fuse_fail']} fuse "
            f"fails, {diag['n_auto_zupt']} auto-zupt, "
            f"{diag['n_downweighted']} downweighted")
        dw = nav.downweight_counts()
        log(f"downweighted (chi2 outlier): ars {dw['ars']}, "
            f"ahrs {dw['ahrs']}, baro_alt {dw['baro_alt']}, "
            f"local_gnss_offset {dw['local_gnss']}")
        # Which sub-filters were alive at the end, and, when the 3D filter
        # was not, the reason (same codes telemetry publishes live under
        # INSLIB/status, so a replay and a live session agree on the
        # verdict).
        st_tree, _blocked, blocked_text = suite_status(nav)
        running = [n for n, k in (("ars", "ars_running"), ("ahrs", "ahrs_running"),
                                  ("baro_alt", "baro_running"),
                                  ("full3d", "full3d_running"))
                   if st_tree[k] > 0.0]
        log(f"final state: mode {nav.mode_name()}, running "
            f"[{', '.join(running) if running else 'none'}] - {blocked_text}")
        # Final filter-reported 1-sigma of the ins error state (last
        # epoch): how uncertain ins believes its own solution is at the end
        # of the run (not a vs-truth error).
        sd = nav.stddev()
        if sd is not None:
            p, v, a = sd["pos_ned"], sd["vel_ned"], sd["rpy"]
            ab, gb = sd["acc_bias"], sd["gyr_bias"]
            log("final 1-σ (last epoch, filter-reported):")
            log(f"  pos NED   {p[0]:.3f} {p[1]:.3f} {p[2]:.3f} m")
            log(f"  vel NED   {v[0]:.3f} {v[1]:.3f} {v[2]:.3f} m/s")
            log(f"  att RPY   {math.degrees(a[0]):.3f} {math.degrees(a[1]):.3f} "
                f"{math.degrees(a[2]):.3f} deg")
            log(f"  acc bias  {ab[0]:.4f} {ab[1]:.4f} {ab[2]:.4f} m/s^2")
            log(f"  gyr bias  {math.degrees(gb[0]):.4f} {math.degrees(gb[1]):.4f} "
                f"{math.degrees(gb[2]):.4f} deg/s")
            if "mag_bias" in sd:
                mb = sd["mag_bias"]
                log(f"  mag bias  {mb[0]:.3f} {mb[1]:.3f} {mb[2]:.3f} uT")
        for line in rp.last_fix_accuracy_lines(self.last_fix) or ():
            log(line)
        for line in rp.growth_rate_lines(self.noise):
            log(line)
        for line in rp.baro_growth_rate_lines(spec["baro"]):
            log(line)
        for line in rp.ahrs_growth_rate_lines(rp.effective_ahrs_cfg(spec)):
            log(line)
        # Overconfidence / covariance-collapse watchdog (REQ-NAV-040): the
        # filter reporting a physically implausible accuracy means its
        # covariance has collapsed and it may be silently diverging while
        # looking confident.
        oc = nav.overconfidence()

        def _mm(x):
            return "-" if not math.isfinite(x) else f"{x:.2e}"
        log(f"covariance watchdog: best reported stddev pos {_mm(oc['min_pos_m'])} m, "
            f"vel {_mm(oc['min_vel_mps'])} m/s, att {_mm(oc['min_att_deg'])} deg")
        log(f"  ARS best att stddev {_mm(oc['ars']['min_att_deg'])} deg, "
            f"AHRS best att stddev {_mm(oc['ahrs']['min_att_deg'])} deg")
        for who, sub in (("INSLIB", oc), ("ARS", oc["ars"]), ("AHRS", oc["ahrs"])):
            if sub["tripped"]:
                log(f"  WARNING: {who} reported an implausibly small stddev at "
                    f"{sub['n']} epoch(s) - likely covariance collapse / divergence.")
        e_pos = self.e_pos
        if e_pos.n:
            window = ""
            if self.eval_start is not None or self.eval_end is not None:
                lo = f"{self.eval_start:g}" if self.eval_start is not None else "0"
                hi = f"{self.eval_end:g}" if self.eval_end is not None else "end"
                window = f", eval window [{lo}, {hi}] s"
            log(f"ins vs ground truth (n={e_pos.n}, "
                f"after {spec['score']['warmup_sec']:g} s warmup{window}):")
            # score.attitude = 0 means the dataset states it has no attitude
            # reference (ref.csv carries a 0/0/0 placeholder). Printing an
            # error against that reads as a 40 deg yaw failure of the
            # filter, which is not what the number means.
            if spec["score"]["attitude"]:
                for name, s in (("roll", self.e_roll), ("pitch", self.e_pitch),
                                ("yaw", self.e_yaw)):
                    log(f"  {name:5s} error: mean {s.mean():+7.3f}  "
                        f"std {s.std():6.3f}  max |{s.max_abs:6.3f}| deg")
            else:
                log("  attitude error: n/a (score.attitude = 0, no attitude "
                    "reference in this dataset)")
            log(f"  pos rms: {e_pos.rms():.3f} m  max {e_pos.max_abs:.3f} m")
        e_h = self.e_height_ell
        if e_h.n:
            log(f"nav_suite_get_height_ellipsoid() vs ground truth (absolute, "
                f"n={e_h.n}):")
            log(f"  height error: mean {e_h.mean():+.3f}  "
                f"std {e_h.std():.3f}  rms {e_h.rms():.3f}  "
                f"max |{e_h.max_abs:.3f}| m")
        log(f"final nav_suite mode: {nav.mode_name()}")
        self._report_data_quality(log)

    def _report_data_quality(self, log):
        spec = self.spec
        n_imu_total = self.n_imu_total
        log("\ndata quality summary:")
        log(f"  imu:  {self.imu_hz:6.1f} Hz avg (n={n_imu_total}, "
            f"{self.imu_duration:.1f} s), "
            f"max gap {self.imu_max_gap * 1000.0:.0f} ms at t={self.imu_max_gap_at:.1f} s")

        def _report_stream(label, timestamps_us):
            n, dur, hz, gap, gap_at = rp._stream_gap_stats(timestamps_us)
            if n < 2:
                log(f"  {label}: n/a")
                return
            log(f"  {label}: {hz:6.2f} Hz avg (n={n}, {dur:.1f} s), "
                f"max gap {gap:.2f} s at t={gap_at:.1f} s")

        _report_stream("gnss", [fx["t_us"] for fx in self.fixes])
        if self.mags:
            _report_stream("mag ", [row[0] for row in self.mags])
        if self.baros:
            _report_stream("baro", [row[0] for row in self.baros])

        if self.n_static:
            frac = 100.0 * self.n_static / n_imu_total if n_imu_total else 0.0
            log(f"  stationary epochs: {self.n_static}/{n_imu_total} "
                f"({frac:.1f}% of trial, ins auto-ZUPT/ZARU-detected)")
        else:
            log("  stationary epochs: none detected"
                f"{' (aiding: none - ins never runs auto-ZUPT/ZARU)' if self.aiding == 'none' else ''}")

        # Is the configured/reported GNSS accuracy realistic? Only for real
        # GNSS aiding (ref-synthesized fixes equal the truth).
        if self.aiding == "gnss":
            rp.print_gnss_standstill_accuracy(self.gnss_static_phases, out=log)
        # Same check for the raw baro/mag channels (any aiding mode).
        if self.baros:
            rp.print_channel_standstill_accuracy(
                "baro", "m", self.baro_static_phases,
                float(spec["baro"]["stddev_m"]), out=log)
        if self.mags:
            rp.print_channel_standstill_accuracy(
                "mag ", "uT", self.mag_static_phases, self.mag_sd, out=log)

        if self.imu_hz <= 0:
            return
        noise = self.noise
        dt_nominal = 1.0 / self.imu_hz

        # Per-sample sensor noise floor vs. the configured model, measured
        # by the whole-trial second difference (see _imu_prepass): it
        # cancels bias and smooth vehicle dynamics, keeps sensor noise +
        # vibration, and works even when the platform never stops.
        # Var(2nd diff) = 6*sigma^2 for white noise.
        nm = rp.noise_model_metrics(self.gyr_d2_stat, self.acc_d2_stat,
                                    self.gyr_d3_stat, self.acc_d3_stat,
                                    self.imu_hz, noise)
        for label, key, unit, to_unit in (("gyro ", "gyr", "deg/s", math.degrees),
                                          ("accel", "acc", "m/s^2", lambda x: x)):
            d2_stat = getattr(self, f"{key}_d2_stat")
            if d2_stat[0].n <= 1:
                continue
            expect = math.sqrt(noise[f"{key}_psd"] / dt_nominal)
            meas = math.sqrt(sum(s.std() ** 2 for s in d2_stat) / 3.0) / math.sqrt(6.0)
            ratio = meas / expect if expect > 0 else math.nan
            log(f"  {label} noise vs. configured {to_unit(expect):.4f} {unit}:")
            log(f"    measured on this recording: {to_unit(meas):8.4f} {unit}  "
                f"(ratio {ratio:.2f}, n={d2_stat[0].n})")
            if nm[f"{key}_bandlimited"]:
                log(f"    not comparable: the IMU low-pass filters its output (d3/d2"
                    f" variance ratio {nm[f'{key}_d3d2']:.1f}, white noise 3.3),")
                log("    so this per-sample value misses most of the noise")

        # A concrete config action only for a gross error in the dangerous
        # direction (see noise_model_metrics): a configured model above the
        # measurement is the normal case and is never talked down, and a
        # low-pass filtered IMU gets no number at all.
        sugg = [(f"{k}_psd", nm[f"{k}_psd_sugg"], unit) for k, unit in
                (("gyr", "(rad/s)^2/Hz"), ("acc", "(m/s^2)^2/Hz"))
                if nm[f"{k}_psd_sugg"] is not None]
        if sugg:
            log("  imu noise model far too optimistic, measured on this recording"
                " (paste into config.yaml `imu:`):")
            for term, val, unit in sugg:
                log(f"    {term}: {val:.1e}   # {unit}  (now {noise[term]:.1e},"
                    f" x{val / noise[term]:.0f})")
        else:
            log("  imu noise model: no change suggested (only a sensor noisier than"
                f" configured by more than x{rp.NOISE_MODEL_WARN_FACTOR:.0f} gets one)")
        # Neither the bias random walk (imu.*_bias_rw), a slow drift, nor the
        # white noise of a low-pass filtered IMU can be read off this
        # per-sample measure: both want Allan variance on a long static
        # recording.
        log("  for gyr_psd/acc_psd and gyr_bias_rw/acc_bias_rw from a long STATIC"
            " recording use tools/allan_variance.py")

    def findings(self, rec=None):
        """Dr. INS findings over everything above (insdoctor_findings()).
        `rec` is a PlotRecorder's history, None when none ran (the baro
        offset span is then unknown)."""
        spec, nav = self.spec, self.nav
        aiding = self.aiding
        diag = nav.diag()
        return rp.insdoctor_findings({
            "aiding_mode": aiding,
            "imu_hz": self.imu_hz,
            "imu_max_gap": self.imu_max_gap, "imu_max_gap_at": self.imu_max_gap_at,
            "imu_rate_hz": self.imu_rate_hz,
            "gnss": (rp._stream_gap_stats([fx["t_us"] for fx in self.fixes])
                     if self.fixes else None),
            "noise": rp.noise_model_metrics(self.gyr_d2_stat, self.acc_d2_stat,
                                            self.gyr_d3_stat, self.acc_d3_stat,
                                            self.imu_hz, self.noise),
            "gnss_acc": (rp.gnss_standstill_accuracy(self.gnss_static_phases)
                         if aiding == "gnss" else None),
            "gnss_vel": (rp.gnss_vel_accuracy_stats(self.fixes)
                         if aiding == "gnss" else None),
            "gnss_max_hor_vel_stddev_mps": spec["gnss"]["max_horizontal_vel_stddev_mps"],
            "baro_acc": (rp.channel_standstill_accuracy(
                self.baro_static_phases, float(spec["baro"]["stddev_m"]))
                if self.baros else None),
            "mag_acc": (rp.channel_standstill_accuracy(
                self.mag_static_phases, self.mag_sd) if self.mags else None),
            "health": {"n": self.health_n, "limits": rp.STDDEV_LIMITS,
                       "stats": self.health_stats},
            "overconfidence": nav.overconfidence(),
            "leverarm": {"gnss": self.leverarm, "score": self.score_la},
            "baro_height": {
                # n_baro_height_used > 0 <=> ins latched the barometric
                # height source at bootstrap (REQ-NAV-053/-054).
                "active": diag["n_baro_height_used"] > 0,
                "offset_span_m": (rp._span(rec["local_gnss_offset"])
                                  if rec is not None else None),
            },
            "diag": diag,
        })

    def gnss_delay_auto(self):
        """Whether a GNSS-delay estimate is meaningful on its own: real GNSS
        aiding (a ref-synthesized fix is time-aligned with the reference by
        construction and would trivially self-correlate at about 0 ms) with
        usable velocity, plus a barometer as the near-zero-latency vertical
        reference."""
        return (self.aiding == "gnss" and bool(self.baros)
                and any(fx.get("vel_ok") and fx.get("cov_vel") is not None
                        for fx in self.fixes))

    def plot_extras(self):
        """Whole-run inputs of the magnetometer and ellipsoid altitude pages
        of ins_plots, to merge into the PlotRecorder history."""
        mags, t0_us = self.mags, self.t0_us
        extra = {
            # Raw magnetometer point cloud for the hard-iron sphere page.
            # Kept at full resolution (thinned in the plot itself). An
            # all-zero fixed_bias means "no fixed calibration".
            "mag_raw": [tuple(m[1]) for m in mags],
            "mag_fixed_bias": self.mag_bias_cfg if any(self.mag_bias_cfg) else None,
            # Calibrated field magnitude |B|(t) vs. the WMM total field F,
            # for the "deviation from WMM" page. The uncalibrated magnitude
            # comes along as a second trace to show what the calibration
            # bought. Times share the recorders' origin (t0_us).
            "wmm_field_uT": None,
            "mag_field_t": [],
            "mag_field_mag": [],
            "mag_field_mag_raw": [],
            "mag_field_xyz": [],
            # WGS84 ellipsoid height of nav's own (fixed) local-NED origin,
            # so the baro_alt page can convert ref_pos/fix_pos_d back to
            # absolute ellipsoid height.
            "origin_ellipsoid_h_m": (self.origin_h if self.origin_ecef is not None
                                     else math.nan),
        }
        wmm_year = float(self.spec["mag"]["wmm_year"])
        if (mags and wmm_year > 0 and t0_us is not None
                and self.site_llh is not None):
            from INSLIB import wmm_field_ned
            b_ned = wmm_field_ned(math.degrees(self.site_llh[0]),
                                  math.degrees(self.site_llh[1]), wmm_year)
            extra["wmm_field_uT"] = math.sqrt(sum(c * c for c in b_ned))
            extra["mag_field_t"] = [(m[0] - t0_us) / US_PER_SEC for m in mags]
            # Calibrated vectors: the plot subtracts the 18-state bias
            # estimate from them.
            extra["mag_field_xyz"] = [
                tuple(rp.mag_calibrate(m[1], self.mag_misalign,
                                       self.mag_bias_cfg))
                for m in mags]
            extra["mag_field_mag"] = [math.sqrt(sum(c * c for c in v))
                                      for v in extra["mag_field_xyz"]]
            # Only worth a second trace when a calibration is actually
            # configured, otherwise it would sit exactly on the first one.
            if self.mag_cal_active:
                extra["mag_field_mag_raw"] = [math.sqrt(sum(c * c for c in m[1]))
                                              for m in mags]
        return extra


def gnss_delay_lines(curve, configured_ms):
    """The GNSS-delay estimate from gnss_delay_correlation_curve() as
    summary lines, with the config key that consumes it (REQ-VER-008)."""
    if curve is None:
        return ["gnss delay estimate: not enough data (need baro_alt "
                "running and at least one GNSS fix)"]
    delay_ms, corr = max(curve, key=lambda row: row[1])
    quality = ("good" if corr > 0.7 else
               "weak - probably not enough vertical motion in "
               "this trial to tell" if corr > 0.3 else
               "poor - do not trust this number")
    lines = [f"gnss delay estimate: {delay_ms:.0f} ms "
             f"(correlation {corr:.2f}, {quality}) - relative to "
             f"baro_alt, which is not necessarily latency-free "
             f"either (see --estimate-gnss-delay's help)"]
    if corr > 0.7:
        lines.append(f"  -> set `gnss: delay_ms: {delay_ms:.0f}` in "
                     f"config.yaml to compensate it "
                     f"(currently {configured_ms} ms)")
    else:
        lines.append(f"  (would go into config `gnss: delay_ms`, now "
                     f"{configured_ms} ms - but the correlation is too low "
                     f"to trust this value)")
    return lines


# ============================================================================
# Recorders
# ============================================================================

REC_KEYS = (
    "t", "pos", "pos_sigma", "vel", "vel_sigma",
    "rpy_deg", "rpy_sigma_deg", "ref_pos",
    "ref_vel", "ref_rpy_deg", "pos_err_ned",
    "rpy_err_deg", "baro_vel_d", "gnss_vel_d",
    # most recent fused GNSS fix's OWN reported 1-sigma (receiver
    # covariance, after apply_fallback), for the PDF summary's "last GNSS
    # fix accuracy" line, side-by-side with the filter's final 1-sigma.
    "gnss_pos_sigma", "gnss_vel_sigma",
    # ins bias + 1-sigma (all NED/body, SI units)
    "acc_bias", "acc_bias_sigma", "gyr_bias", "gyr_bias_sigma",
    # ins magnetometer hard-iron bias + 1-sigma
    # (18-state mode only, mag: estimate_bias) [uT]
    "mag_bias", "mag_bias_sigma",
    # ARS/AHRS roll/pitch + own gyro bias/sigma
    "ars_rpy_deg", "ars_rpy_sigma_deg", "ars_gyr_bias", "ars_gyr_bias_sigma",
    "ahrs_rpy_deg", "ahrs_rpy_sigma_deg", "ahrs_gyr_bias", "ahrs_gyr_bias_sigma",
    # heading page: independent absolute-heading sources sampled every
    # tick, NaN when unavailable. mag_heading_deg is the tilt-compensated
    # raw-magnetometer compass (magnetic north), gnss_course_deg the GNSS
    # course-over-ground (atan2(vE,vN), the automotive_mode yaw source)
    # once moving fast enough.
    "mag_heading_deg", "gnss_course_deg",
    # baro_alt page: height/vel in the same NED-down convention as
    # pos/vel/ref_pos/ref_vel above (baro_alt itself is positive up,
    # negated to match), plus its own z-accel bias/sigma, plus the GNSS fix
    # in the same local frame as ref_pos/ref_vel.
    "baro_h_d", "baro_raw_d", "baro_h_sigma", "baro_acc_bias", "baro_acc_bias_sigma",
    "fix_pos_d",
    # baro-to-ellipsoid offset filter (nav_suite_get_height_ellipsoid)
    "local_gnss_offset", "local_gnss_offset_sigma",
    # nav_suite_get_height_ellipsoid() itself, i.e. h_local + the ESTIMATED
    # offset above (not the true origin_h ref_pos is anchored to). Reveals
    # the offset filter's own reconstruction error. Same NED-down
    # convention as the other *_d fields.
    "nav_height_ellipsoid_d",
    # REQ-VER-037: pos moved onto the score.leverarm_frd point, and the up
    # shift from the board to that point (0 without a lever arm) for the
    # board-point curves of the altitude pages.
    "pos_ref_pt", "ref_pt_up",
    # REQ-VER-039: R_b_to_n * score.leverarm_frd and R_b_to_n *
    # gnss.leverarm_frd [m, NED], rotated when the reference sample and the
    # fix arrived and held with them, subtracted from ref_pos/fix_pos_d to
    # move both onto the IMU point (inspostgui's lever arm compensation).
    "ref_pt_ned", "gnss_la_ned",
    # ins's own velocity-aware auto-ZUPT/ZARU detector (shaded on the plot
    # pages so stops can be correlated with bias jumps).
    "zupt_active",
    # The individual detectors zupt_active is OR'd together from, each
    # sampled separately for the dedicated ZUPT/ZARU timeline page. A
    # mismatch between them is invisible on the combined signal above.
    # *_zaru_applied, not *_auto_zaru_active (a much looser "stillness run
    # started" gate), see ars_zaru_applied()'s docstring.
    "auto_zupt_active", "vertical_zupt_active",
    "ars_zaru_applied", "ahrs_zaru_applied",
    # cumulative chi2-downweight counters, same source as
    # downweight_counts()/diag()['n_downweighted'] printed at the end, here
    # sampled every recorder tick so the outlier page can show WHEN.
    "dw_full3d", "dw_ars", "dw_ahrs", "dw_baro_alt", "dw_local_gnss",
)


class PlotRecorder(Observer):
    """State history + error samples at a fixed rate (sim time), the input
    of ins_plots.plot_results(). Every list stays the same length as
    rec["t"]: each group is recorded every tick regardless of ins's own
    readiness (ARS/AHRS/baro_alt run without it) and NaN-padded on its OWN
    availability. A row is appended under `lock`, so another thread can
    read a consistent prefix while the replay runs."""

    def __init__(self, rate_hz, lock=None):
        self.period_us = US_PER_SEC / max(rate_hz, 1e-3)
        self.lock = lock if lock is not None else contextlib.nullcontext()
        self.rec = {k: [] for k in REC_KEYS}
        self._last_us = None
        self._started = False
        self._ref_la_ned = None
        self._fix_la_ned = None

    def on_epoch(self, r):
        # REQ-VER-039: the rotated lever arms of the reference and the fix
        # are latched at the epoch each sample arrives, with the attitude
        # of that moment, and held with the sample. Rotated again on every
        # recorded tick, a 1 Hz reference held for a second would draw an
        # arc per second of turning.
        if r.ref_now is not None:
            self._ref_la_ned = list(rp.ref_point_offset_ned(r.nav, r.score_la))
        if r.fix_now is not None:
            self._fix_la_ned = list(rp.ref_point_offset_ned(r.nav, r.leverarm))
        if not self._started:
            self._started = True
            # A single ref.csv row (e.g. a placeholder ref.csv for a
            # capture with no real fix) gets forward-filled onto every
            # later epoch by last_ref, which would otherwise masquerade as
            # a full, flat "ground truth" trajectory to ins_plots.py. Record
            # the real epoch count so it can tell the two apart.
            with self.lock:
                self.rec["ref_epoch_count"] = len(r.ref)
        t = r.t
        if self._last_us is not None and (t - self._last_us) < self.period_us:
            return
        self._last_us = t
        nav = r.nav
        last_ref, last_fix = r.last_ref, r.last_fix
        origin_ecef = r.origin_ecef
        nan3 = [math.nan] * 3
        row = {"t": (t - r.t0_us) / US_PER_SEC}
        # OR'd with vertical_zupt_active() so the shading still shows
        # standstill phases for a dataset with no absolute position aiding,
        # where auto_zupt_active() alone stays false for the whole replay.
        auto_zupt = nav.auto_zupt_active()
        vert_zupt = nav.vertical_zupt_active()
        row["zupt_active"] = 1.0 if (auto_zupt or vert_zupt) else 0.0
        row["auto_zupt_active"] = 1.0 if auto_zupt else 0.0
        row["vertical_zupt_active"] = 1.0 if vert_zupt else 0.0
        row["ars_zaru_applied"] = 1.0 if nav.ars_zaru_applied() else 0.0
        row["ahrs_zaru_applied"] = 1.0 if nav.ahrs_zaru_applied() else 0.0
        dw_now = nav.downweight_counts()
        row["dw_full3d"] = float(nav.diag()["n_downweighted"])
        row["dw_ars"] = float(dw_now["ars"])
        row["dw_ahrs"] = float(dw_now["ahrs"])
        row["dw_baro_alt"] = float(dw_now["baro_alt"])
        row["dw_local_gnss"] = float(dw_now["local_gnss"])

        # ins's local NED position, moved into the frame latched once in
        # the replay loop, which the ground truth is converted with (see
        # the origin comment in Replay.run() and ins_pos_in_replay_frame()).
        pos = ins_pos_in_replay_frame(nav, r)
        vel = nav.velocity_ned()
        rpy = nav.rpy_ins()
        sd = nav.stddev()
        if pos is not None and vel is not None and rpy is not None and sd:
            row["pos"] = pos
            row["pos_sigma"] = sd["pos_ned"]
            row["vel"] = vel
            row["vel_sigma"] = sd["vel_ned"]
            row["rpy_deg"] = [math.degrees(x) for x in rpy]
            row["rpy_sigma_deg"] = [math.degrees(x) for x in sd["rpy"]]
            acc_bias = nav.bias_acc()
            row["acc_bias"] = list(acc_bias) if acc_bias else nan3[:]
            row["acc_bias_sigma"] = list(sd["acc_bias"])
            gyr_bias = nav.bias_gyr()
            row["gyr_bias"] = list(gyr_bias) if gyr_bias else nan3[:]
            row["gyr_bias_sigma"] = list(sd["gyr_bias"])
            # only present in 18-state mode (mag: estimate_bias), NaN
            # otherwise, same convention as the sub-filter fields below.
            mag_bias = nav.bias_mag()
            row["mag_bias"] = list(mag_bias) if mag_bias else nan3[:]
            row["mag_bias_sigma"] = (list(sd["mag_bias"]) if "mag_bias" in sd
                                     else nan3[:])
        else:
            for k in ("pos", "vel", "pos_sigma", "vel_sigma", "rpy_deg",
                      "rpy_sigma_deg", "acc_bias", "acc_bias_sigma",
                      "gyr_bias", "gyr_bias_sigma", "mag_bias", "mag_bias_sigma"):
                row[k] = nan3[:]
        # REQ-VER-037: the same position at the reference point, and the
        # height shift every board-point curve needs on the altitude pages
        # (baro_alt too, whose sensor sits on the board).
        la_n = rp.ref_point_offset_ned(nav, r.score_la)
        row["pos_ref_pt"] = [p + d for p, d in zip(row["pos"], la_n)]
        row["ref_pt_up"] = -la_n[2]
        row["ref_pt_ned"] = (self._ref_la_ned[:] if self._ref_la_ned is not None
                             else nan3[:])
        row["gnss_la_ned"] = (self._fix_la_ned[:] if self._fix_la_ned is not None
                              else nan3[:])

        # ARS/AHRS: their own roll/pitch (yaw omitted for the ARS,
        # free-running, not meaningful without a reference) and their own
        # independent gyro bias + 1-sigma.
        for pfx, rpy_fn, rpy_sd_fn, bias_fn, sd_fn in (
            ("ars", nav.rpy_ars, nav.rpy_stddev_ars,
             nav.bias_gyr_ars, nav.gyr_bias_stddev_ars),
            ("ahrs", nav.rpy_ahrs, nav.rpy_stddev_ahrs,
             nav.bias_gyr_ahrs, nav.gyr_bias_stddev_ahrs),
        ):
            sub_rpy = rpy_fn()
            row[f"{pfx}_rpy_deg"] = ([math.degrees(x) for x in sub_rpy]
                                     if sub_rpy else nan3[:])
            sub_rpy_sd = rpy_sd_fn()
            row[f"{pfx}_rpy_sigma_deg"] = ([math.degrees(x) for x in sub_rpy_sd]
                                           if sub_rpy_sd else nan3[:])
            sub_bias = bias_fn()
            row[f"{pfx}_gyr_bias"] = list(sub_bias) if sub_bias else nan3[:]
            sub_sd = sd_fn()
            row[f"{pfx}_gyr_bias_sigma"] = list(sub_sd) if sub_sd else nan3[:]

        # Independent absolute-heading sources for the heading page.
        # Magnetometer: tilt-compensate the last sample with the best
        # available roll/pitch (full3d if ready, else AHRS/ARS, both of
        # which run without GNSS), yaw is what is derived, so only leveling
        # matters. The sample is put through the configured fixed
        # calibration first, so the trace is the compass the filter sees.
        level_rpy = rpy or nav.rpy_ahrs() or nav.rpy_ars()
        if r.last_mag is not None and level_rpy is not None:
            m_cal = rp.mag_calibrate(r.last_mag[1], r.mag_misalign, r.mag_bias_cfg)
            row["mag_heading_deg"] = math.degrees(
                mag_heading(m_cal, level_rpy[0], level_rpy[1]))
        else:
            row["mag_heading_deg"] = math.nan
        # GNSS course over ground (the automotive_mode yaw source), gated
        # on horizontal speed so a standstill's noisy course doesn't
        # pollute the trace. Shown whenever GNSS velocity is present,
        # independent of whether automotive_mode is armed.
        row["gnss_course_deg"] = math.nan
        if last_fix is not None and last_fix.get("vel_ok"):
            v_n, v_e = last_fix["vel_ned"][0], last_fix["vel_ned"][1]
            if math.hypot(v_n, v_e) >= max(r.spec["automotive_min_speed_mps"], 2.0):
                row["gnss_course_deg"] = math.degrees(math.atan2(v_e, v_n))

        # baro_alt: own height/velocity (negated to the same NED-down
        # convention as pos/vel above) and its own z-accel bias/sigma.
        baro = nav.baro_alt()
        row["baro_h_d"] = -baro[0] if baro is not None else math.nan
        row["baro_raw_d"] = (-isa_pressure_to_altitude(r.last_baro[1])
                             if r.last_baro is not None else math.nan)
        row["baro_vel_d"] = -baro[1] if baro is not None else math.nan
        baro_bias = nav.baro_acc_bias()
        row["baro_acc_bias"] = baro_bias if baro_bias is not None else math.nan
        baro_sd = nav.baro_stddev()
        row["baro_h_sigma"] = baro_sd[0] if baro_sd is not None else math.nan
        row["baro_acc_bias_sigma"] = baro_sd[2] if baro_sd is not None else math.nan
        gnss_offset = nav.local_gnss_offset()
        row["local_gnss_offset"] = (gnss_offset[0] if gnss_offset is not None
                                    else math.nan)
        row["local_gnss_offset_sigma"] = (gnss_offset[1] if gnss_offset is not None
                                          else math.nan)
        nav_h_ell = nav.height_ellipsoid()
        row["nav_height_ellipsoid_d"] = (
            -(nav_h_ell - r.origin_h)
            if nav_h_ell is not None and origin_ecef is not None else math.nan)

        # GNSS fix, in the same local NED frame as ref_pos/ref_vel.
        row["gnss_vel_d"] = (last_fix["vel_ned"][2] if last_fix is not None
                             else math.nan)
        if last_fix is not None and origin_ecef is not None:
            row["fix_pos_d"] = rp.ref_to_local_ned(
                last_fix, origin_ecef, r.origin_lat, r.origin_lon)[2]
        else:
            row["fix_pos_d"] = math.nan
        if last_fix is not None and last_fix.get("cov_pos") is not None:
            cov_p = last_fix["cov_pos"]
            row["gnss_pos_sigma"] = [math.sqrt(cov_p[0][0]), math.sqrt(cov_p[1][1]),
                                     math.sqrt(cov_p[2][2])]
            cov_v = last_fix.get("cov_vel")
            row["gnss_vel_sigma"] = (
                [math.sqrt(cov_v[0][0]), math.sqrt(cov_v[1][1]),
                 math.sqrt(cov_v[2][2])]
                if last_fix.get("vel_ok") and cov_v is not None else nan3[:])
        else:
            row["gnss_pos_sigma"] = nan3[:]
            row["gnss_vel_sigma"] = nan3[:]

        if last_ref is not None and origin_ecef is not None:
            err_ecef = (rp.pos_error_ecef(nav, last_ref, r.score_la)
                        if nav.is_ready() else None)
            row["ref_pos"] = rp.ref_to_local_ned(
                last_ref, origin_ecef, r.origin_lat, r.origin_lon)
            row["ref_vel"] = list(last_ref["vel_ned"])
            row["ref_rpy_deg"] = [math.degrees(last_ref["roll_rad"]),
                                  math.degrees(last_ref["pitch_rad"]),
                                  math.degrees(last_ref["yaw_rad"])]
            row["pos_err_ned"] = (
                rp._matvec_rm_T(rp.ned_to_ecef_rot(r.origin_lat, r.origin_lon),
                                err_ecef)
                if err_ecef is not None else nan3[:])
            row["rpy_err_deg"] = (nan3[:] if rpy is None else [
                math.degrees(rp.wrap_pi(rpy[0] - last_ref["roll_rad"])),
                math.degrees(rp.wrap_pi(rpy[1] - last_ref["pitch_rad"])),
                math.degrees(rp.wrap_pi(rpy[2] - last_ref["yaw_rad"]))])
        else:
            for k in ("ref_pos", "ref_vel", "ref_rpy_deg", "pos_err_ned",
                      "rpy_err_deg"):
                row[k] = nan3[:]

        rec = self.rec
        with self.lock:
            for k in REC_KEYS:
                rec[k].append(row[k])


class TrackRecorder(Observer):
    """North-East ground track for the map page, at its own (higher) rate
    than PlotRecorder: two floats per sample instead of about sixty. A
    NaN pair rather than a skipped sample while ins has nothing, so the
    drawn line BREAKS instead of bridging the gap with a straight chord.
    The reference arrives at its own (usually slower) rate, so repeated
    reference points are dropped."""

    def __init__(self, rate_hz):
        self.period_us = US_PER_SEC / max(rate_hz, 1e-3)
        self.est, self.ref = [], []
        self._last_us = None

    def on_epoch(self, r):
        t = r.t
        if self._last_us is not None and (t - self._last_us) < self.period_us:
            return
        self._last_us = t
        nav = r.nav
        track_pos = ins_pos_in_replay_frame(nav, r)
        track_la = rp.ref_point_offset_ned(nav, r.score_la)  # REQ-VER-037
        self.est.append((track_pos[0] + track_la[0], track_pos[1] + track_la[1])
                        if track_pos is not None else (math.nan, math.nan))
        if r.last_ref is not None and r.origin_ecef is not None:
            p = rp.ref_to_local_ned(r.last_ref, r.origin_ecef,
                                    r.origin_lat, r.origin_lon)
            if not self.ref or (p[0], p[1]) != self.ref[-1]:
                self.ref.append((p[0], p[1]))


class KmlRecorder(Observer):
    """Geodetic track for ins_kml/ins_map_frames: the estimate with its
    attitude, the reference, and the raw fix with its North/East
    covariance."""

    def __init__(self, rate_hz):
        self.period_us = US_PER_SEC / max(rate_hz, 1e-3)
        self.est, self.ref, self.fix = [], [], []
        self._last_us = None

    def on_epoch(self, r):
        t = r.t
        if self._last_us is not None and (t - self._last_us) < self.period_us:
            return
        self._last_us = t
        nav = r.nav
        ecef = nav.position_ecef()
        rpy = nav.rpy_ins()
        if ecef is None or rpy is None:
            return
        t_rel = (t - r.t0_us) / US_PER_SEC
        lat, lon, alt = ecef_to_llh(*ecef)
        self.est.append((t_rel, math.degrees(lat), math.degrees(lon), alt,
                         math.degrees(rpy[0]), math.degrees(rpy[1]),
                         math.degrees(rpy[2])))
        if r.last_ref is not None:
            self.ref.append((math.degrees(r.last_ref["lat_rad"]),
                             math.degrees(r.last_ref["lon_rad"]),
                             r.last_ref["h_m"]))
        last_fix = r.last_fix
        if last_fix is not None:
            fx = (math.degrees(last_fix["lat_rad"]),
                  math.degrees(last_fix["lon_rad"]),
                  last_fix["h_m"])
            # North/East/(North-East) position covariance in m^2, for
            # --map-frames' error ellipses, NaN if this fix carries none.
            cov_p = last_fix.get("cov_pos")
            cov_ne = ((cov_p[0][0], cov_p[0][1], cov_p[1][1])
                      if cov_p is not None else (math.nan, math.nan, math.nan))
            # Dedup repeats (the fix rate is usually slower than this
            # recorder's) so a stale fix during an outage draws as one held
            # point. t_rel is kept even on a repeat (--map-frames uses it to
            # tell "still the last fix" from "no fix yet at all").
            if not self.fix or fx != self.fix[-1][1:4]:
                self.fix.append((t_rel,) + fx + cov_ne)
