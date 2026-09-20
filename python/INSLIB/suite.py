"""High-level Navigator: the best available navigation solution.

:class:`Navigator` wraps the C ``nav_suite`` (ins + two AHRS filters
with mode arbitration). You push whatever sensors you have each epoch and
read back a unified :class:`Solution`; the suite degrades gracefully:

* FULL: ins ready with recent absolute position aiding.
* COASTING: ins ready, dead-reckoning on IMU only (position drifts).
* ATTITUDE_ONLY: ins position gone (long outage, sustained bad GNSS
  quality, or not initialized yet), but attitude still comes from the
  AHRS fallback.
* NONE: nothing usable yet.

The vertical channel gets the same treatment: a barometer sample per
epoch (:meth:`Navigator.baro`) drives the baro/accel vertical filter, and
``Solution.height_m`` / ``height_ell_m`` always carry the best height
source (ins under fresh aiding, else baro, else coasting) in one
continuous datum.

Example::

    from INSLIB import Navigator, Config
    nav = Navigator(Config(auto_init=True))
    nav.imu(t_us, dt, acc, gyr)          # begins an epoch
    nav.gnss_pos_llh(llh, var_ned)       # optional aiding, add what you have
    nav.mag(mag_uT, mag_var)
    nav.baro(pressure_pa)
    nav.update()                         # runs the filter for this epoch
    sol = nav.solution()                 # pure reader, call it when you need it
    print(sol.mode, sol.roll, sol.pitch, sol.yaw, sol.height_m, sol.ready)

(c) Jan Zwiener (jan@zwiener.org)
"""

import ctypes
import math
from dataclasses import dataclass

from ._core import (_Base, _lib, _f3, State, MODE_NAMES, _fill_nav_state,
                    rpy_to_quat)


@dataclass
class Solution:
    """Unified per-epoch result of :meth:`Navigator.solution`."""
    mode: str = "NONE"          # FULL / COASTING / ATTITUDE_ONLY / NONE
    ready: bool = False         # position/velocity usable (ins ready)
    attitude_ok: bool = False   # some attitude available (ins or AHRS)
    dr_ms: int = -1             # ms since last absolute position aiding
    roll: float = math.nan      # best-available attitude [rad]
    pitch: float = math.nan
    yaw: float = math.nan
    pos_ecef: tuple = None      # WGS84 ECEF [m] or None
    pos_local: tuple = None     # local NED [m] or None
    vel_ned: tuple = None       # NED velocity [m/s] or None
    lat_rad: float = math.nan
    lon_rad: float = math.nan
    alt_m: float = math.nan     # ellipsoid height from INSLIB's geodetic anchor
    height_m: float = math.nan  # best height above the NED-origin datum
                                # [m, positive up]: ins under fresh aiding,
                                # else the baro filter, else coasting ins
    height_ell_m: float = math.nan  # best absolute (ellipsoid) height [m]:
                                # ins, else baro + estimated baro/GNSS offset
    baro_height_m: float = math.nan  # baro filter height (same datum) [m]
    baro_vz_mps: float = math.nan    # baro filter climb rate [m/s, up]


class Navigator(_Base):
    """nav_suite-backed navigator (the "best available solution")."""
    _prefix = "ins_suite"

    # --- attitude sources ---------------------------------------------------
    def mode(self) -> int:
        return int(_lib.ins_suite_get_mode(self._h))

    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode(), "NONE")

    def _rpy(self, fn):
        out = _f3()
        return list(out) if fn(self._h, out) else None

    def rpy(self):
        """Best available roll/pitch/yaw [rad] (ins, else AHRS, else ARS)."""
        return self._rpy(_lib.ins_suite_get_rpy)

    def rpy_ins(self):
        return self._rpy(_lib.ins_suite_get_rpy_ins)

    def rpy_ars(self):
        return self._rpy(_lib.ins_suite_get_rpy_ars)

    def rpy_ahrs(self):
        return self._rpy(_lib.ins_suite_get_rpy_ahrs)

    def rpy_stddev(self):
        """Best-available roll/pitch/yaw 1-sigma [rad] (ins, else AHRS,
        else ARS) -- the same source precedence as rpy(), so a caller that
        already fell back to AHRS/ARS for the attitude itself (ins not
        ready, e.g. ATTITUDE_ONLY) gets a matching stddev instead of None:
        ins's own stddev() only ever reports ins's covariance, which is
        unavailable exactly when rpy() has already fallen back."""
        if self.is_ready():
            sd = self.stddev()
            if sd is not None:
                return sd["rpy"]
        out = self.rpy_stddev_ahrs()
        if out is not None:
            return out
        return self.rpy_stddev_ars()

    def bias_gyr_ars(self):
        """ARS's own gyro bias estimate [rad/s] (independent of ins's
        and the AHRS's), or None if not initialized."""
        return self._rpy(_lib.ins_suite_get_bias_gyr_ars)

    def bias_gyr_ahrs(self):
        """Same as bias_gyr_ars() for the magnetometer AHRS."""
        return self._rpy(_lib.ins_suite_get_bias_gyr_ahrs)

    def gyr_bias_stddev_ars(self):
        """1-sigma uncertainty [rad/s] of bias_gyr_ars(), from the ARS's
        own error-state covariance."""
        return self._rpy(_lib.ins_suite_get_gyr_bias_stddev_ars)

    def gyr_bias_stddev_ahrs(self):
        """Same as gyr_bias_stddev_ars() for the magnetometer AHRS."""
        return self._rpy(_lib.ins_suite_get_gyr_bias_stddev_ahrs)

    def rpy_stddev_ars(self):
        """ARS's own 1-sigma roll/pitch/yaw uncertainty [rad] (independent
        of ins's and the AHRS's), or None if not initialized."""
        return self._rpy(_lib.ins_suite_get_rpy_stddev_ars)

    def rpy_stddev_ahrs(self):
        """Same as rpy_stddev_ars() for the magnetometer AHRS."""
        return self._rpy(_lib.ins_suite_get_rpy_stddev_ahrs)

    # --- zero-rotation update (REQ-SUITE-009/-010, REQ-AHRS-017) -------------
    def set_auto_zaru(self, ars=True, ahrs=True):
        """Arm the ARS's/AHRS's velocity-blind auto-ZARU fallback: for use
        when ins never initializes (e.g. "aiding: none"), so the normal
        velocity-aware trigger (see zaru_active()) never fires. Off by
        default; call once, right after construction, before the first
        update()/imu()."""
        self._c("set_auto_zaru")(self._h, 1 if ars else 0, 1 if ahrs else 0)

    def set_baro_acc_bias_drift(self, density_mps2_sqrthz):
        """Set the baro/accel vertical filter's acc-bias drift density
        sigma_b [m/s^2/sqrt(Hz)] (<=0 -> baro_alt's own default). Call once,
        right after construction, before the first baro()/update(): the
        template latches when the first barometer sample arrives."""
        self._c("set_baro_acc_bias_drift")(self._h, float(density_mps2_sqrthz))

    def set_baro_acc_noise(self, density_mps2_sqrthz):
        """Set the baro/accel vertical filter's own direct accel-noise
        density sigma_a [m/s^2/sqrt(Hz)] (<=0 -> baro_alt's own default).
        Same call-before-first-baro-sample timing as
        set_baro_acc_bias_drift()."""
        self._c("set_baro_acc_noise")(self._h, float(density_mps2_sqrthz))

    def set_baro_acc_bias_init_stddev(self, stddev_mps2):
        """Set baro_alt's own INITIAL accel-bias uncertainty [m/s^2]
        (<=0 -> baro_alt's own default). An INITIAL CONDITION, not a
        process-noise rate -- it propagates into height/velocity through
        the bias/height/velocity Phi coupling, growing roughly with
        T^4/T^2 respectively, and at the default size can dominate
        baro_alt's apparent growth for tens of seconds. Same call-before-
        first-baro-sample timing as set_baro_acc_bias_drift()."""
        self._c("set_baro_acc_bias_init_stddev")(self._h, float(stddev_mps2))

    def set_baro_h_init_stddev(self, stddev_m):
        """Set baro_alt's own INITIAL height uncertainty [m] (<=0 ->
        baro_alt default, 1.0). Same call-before-first-baro-sample timing
        as set_baro_acc_bias_drift()."""
        self._c("set_baro_h_init_stddev")(self._h, float(stddev_m))

    def set_baro_v_init_stddev(self, stddev_mps):
        """Set baro_alt's own INITIAL velocity uncertainty [m/s] (<=0 ->
        baro_alt default, 0.5). Same call-before-first-baro-sample timing
        as set_baro_acc_bias_drift()."""
        self._c("set_baro_v_init_stddev")(self._h, float(stddev_mps))

    def set_baro_h_process_noise(self, density_m_sqrthz):
        """Set baro_alt's own small direct height process-noise density
        sigma_h [m/sqrt(Hz)] (<=0 -> baro_alt's own default) -- a safety
        margin against discretization/model mismatch, not a primary noise
        source. Same call-before-first-baro-sample timing as
        set_baro_acc_bias_drift()."""
        self._c("set_baro_h_process_noise")(self._h, float(density_m_sqrthz))

    def set_local_gnss_rw_stddev(self, rw_stddev_mps):
        """Set the local-height/GNSS-ellipsoid offset filter's random walk
        [m/sqrt(s)] (<=0 -> local_gnss_alt's own default 0.03). Raise this
        for missions with large altitude excursions (e.g. a soaring glider
        gaining kilometres of altitude): the offset absorbs the ISA-model
        error, which grows with the excursion and can outrun the default
        random walk, tuned for a multi-metre swing. Call once, right after
        construction, before the offset filter's first local-height/GNSS
        pair (in practice, before the first update())."""
        self._c("set_local_gnss_rw_stddev")(self._h, float(rw_stddev_mps))

    def set_local_gnss_chi2_threshold(self, threshold):
        """Set the offset filter's chi2 outlier gate on the innovation
        (<=0 -> local_gnss_alt's own default, chi2inv(0.95,1)). Same
        call-before-first-pair timing as set_local_gnss_rw_stddev()."""
        self._c("set_local_gnss_chi2_threshold")(self._h, float(threshold))

    def set_local_gnss_min_update_interval(self, interval_sec):
        """Set the offset filter's minimum time between two fusions [s]
        (<=0 -> local_gnss_alt's own default 10). Same call-before-first-
        pair timing as set_local_gnss_rw_stddev()."""
        self._c("set_local_gnss_min_update_interval")(self._h, float(interval_sec))

    def set_local_gnss_stddev_inflation(self, factor):
        """Set the offset filter's stddev inflation factor applied to both
        sides of the pair before combining into the measurement variance
        (<=0 -> local_gnss_alt's own default 3). Same call-before-first-
        pair timing as set_local_gnss_rw_stddev()."""
        self._c("set_local_gnss_stddev_inflation")(self._h, float(factor))

    def set_ahrs_gyr_noise(self, density_rps_sqrthz):
        """Set the ARS's/AHRS's SHARED gyro noise density sigma_g
        [rad/s/sqrt(Hz)] (<=0 -> ahrs's own default) -- both sub-filters
        consume the same physical gyro, so this sets both templates at
        once. Call once, right after construction, before the first
        update(): the templates latch at auto-initialization."""
        self._c("set_ahrs_gyr_noise")(self._h, float(density_rps_sqrthz))

    def set_ahrs_acc_noise(self, stddev_mps2):
        """Set the ARS's/AHRS's SHARED accelerometer noise stddev [m/s^2]
        (<=0 -> ahrs's own default) -- both sub-filters consume the same
        physical accelerometer for leveling, so this sets both templates
        at once. Same call-before-first-update() timing contract as
        set_ahrs_gyr_noise()."""
        self._c("set_ahrs_acc_noise")(self._h, float(stddev_mps2))

    def set_ahrs_gyr_bias_rw(self, density_rps2_sqrthz):
        """Set the ARS's/AHRS's SHARED gyro bias random walk density
        [rad/s^2/sqrt(Hz)] (<=0 -> ahrs's own default); same timing
        contract as set_ahrs_gyr_noise()."""
        self._c("set_ahrs_gyr_bias_rw")(self._h, float(density_rps2_sqrthz))

    def set_init_att_hint(self, roll_rad=0.0, pitch_rad=0.0,
                          stddev_roll_pitch_rad=0.0, yaw_rad=0.0,
                          stddev_yaw_rad=0.0):
        """Known initial roll/pitch and/or yaw for ins's auto-init
        bootstrap -- the static "I just know it" case (e.g. a known
        heading with no mag/GNSS-course yaw aiding to derive it from).
        Call once, right after construction, before the first update().
        Roll/pitch are only used together (stddev_roll_pitch_rad <= 0 ->
        no roll/pitch hint); yaw is independent (stddev_yaw_rad <= 0 -> no
        yaw hint). Stops applying once ins first initializes."""
        self._c("set_init_att_hint")(self._h, float(roll_rad), float(pitch_rad),
                                     float(stddev_roll_pitch_rad), float(yaw_rad),
                                     float(stddev_yaw_rad))

    def set_ahrs_gyr_bias_init_stddev(self, stddev_rps):
        """Set the ARS's/AHRS's SHARED initial gyro-bias uncertainty
        [rad/s], all 3 axes (<=0 -> ahrs default, 1/1/5 deg/s xy/z). An
        INITIAL CONDITION, not a process-noise rate -- but it dominates
        yaw's apparent growth for a long time via the attitude/bias Phi
        coupling if left at the (generous) default. Same timing contract
        as set_ahrs_gyr_noise()."""
        self._c("set_ahrs_gyr_bias_init_stddev")(self._h, float(stddev_rps))

    def zaru_active(self):
        """True if a zero-rotation update was applied to the ARS/AHRS on
        the last update() (explicit flag OR'd with ins's own auto-ZUPT/
        ZARU detector, see auto_zupt_active()). Diagnostic/telemetry."""
        return bool(_lib.ins_suite_zaru_active(self._h))

    def ars_auto_zaru_active(self):
        """True if the ARS's own velocity-blind fallback (set_auto_zaru)
        has STARTED a stillness run (its loose gyro/accel magnitude gate
        is satisfied). NOT the same as "a ZARU actually fired": true for
        the whole run -- including e.g. genuine constant-velocity cruise,
        which this loose gate cannot tell from a stop -- well before the
        stricter windowed-variance criterion and dwell time have been
        satisfied. Use ars_zaru_applied() for "did it actually fire"."""
        return bool(_lib.ins_suite_ars_auto_zaru_active(self._h))

    def ahrs_auto_zaru_active(self):
        """Same as ars_auto_zaru_active() for the magnetometer AHRS."""
        return bool(_lib.ins_suite_ahrs_auto_zaru_active(self._h))

    def ars_zaru_applied(self):
        """True if the ARS's own last update actually saw a zero-rotation
        trigger fire: the loose gate above AND the windowed-variance
        criterion AND the dwell time, OR'd with whatever nav_suite passed
        down from ins (zaru_active()) -- the decision that actually
        mattered, unlike ars_auto_zaru_active() above."""
        return bool(_lib.ins_suite_ars_zaru_applied(self._h))

    def ahrs_zaru_applied(self):
        """Same as ars_zaru_applied() for the magnetometer AHRS."""
        return bool(_lib.ins_suite_ahrs_zaru_applied(self._h))

    def vertical_zupt_active(self):
        """True if a zero-velocity update actually reached the baro/accel
        vertical filter on the last update() (REQ-SUITE-015). Wider than
        zaru_active(): the ARS/AHRS velocity-blind fallback counts too,
        which is the only stillness source while ins is attitude-only.
        False whenever baro_alt did not run that epoch, so this answers
        "are the ZUPTs arriving?", not just "was a trigger present?"."""
        return bool(_lib.ins_suite_vertical_zupt_active(self._h))

    # --- vertical channel -----------------------------------------------------
    def baro_alt(self):
        """(height_m, climb_mps) from the baro/accel vertical filter (both
        positive up, height in the common NED-origin datum), or None while
        no barometer sample has been seen / the filter is unhealthy."""
        h = ctypes.c_float()
        v = ctypes.c_float()
        if _lib.ins_suite_get_baro_alt(self._h, ctypes.byref(h), ctypes.byref(v)):
            return (h.value, v.value)
        return None

    def baro_acc_bias(self):
        """baro_alt's own z-accel bias correction [m/s^2] (own error
        state, independent of ins's/AHRS's accel bias), or None while
        not initialized."""
        v = ctypes.c_float()
        return v.value if _lib.ins_suite_get_baro_acc_bias(self._h, ctypes.byref(v)) else None

    def baro_stddev(self):
        """1-sigma [h_m, v_mps, acc_bias_mps2] uncertainty of baro_alt's
        own 3-state error covariance, or None while not initialized."""
        return self._rpy(_lib.ins_suite_get_baro_stddev)

    def local_gnss_offset(self):
        """(offset_m, stddev_m) of the baro-to-ellipsoid offset filter:
        ellipsoid height ~= barometric ISA altitude + offset_m (see
        nav_suite_get_height_ellipsoid). None until a baro/GNSS pair has
        been seen."""
        offset = ctypes.c_float()
        stddev = ctypes.c_float()
        if _lib.ins_suite_get_local_gnss_offset(self._h, ctypes.byref(offset), ctypes.byref(stddev)):
            return (offset.value, stddev.value)
        return None

    # --- outlier-rejection diagnostics (REQ-SYS-006) --------------------------
    def downweight_counts(self):
        """Monotonic counters of chi2-downweighted fusions for the sub-
        filters that have no bare-ins equivalent (ins's own is
        diag()["n_downweighted"], REQ-NAV-036): ars, ahrs (REQ-AHRS-019),
        baro_alt, local_gnss (REQ-BARO-017). Diagnostic only."""
        return {
            "ars": _lib.ins_suite_get_ars_downweight_count(self._h),
            "ahrs": _lib.ins_suite_get_ahrs_downweight_count(self._h),
            "baro_alt": _lib.ins_suite_get_baro_downweight_count(self._h),
            "local_gnss": _lib.ins_suite_get_local_gnss_downweight_count(self._h),
        }

    def overconfidence(self):
        """Overconfidence / covariance-collapse watchdog (REQ-NAV-040 for
        ins, REQ-AHRS-020 for the attitude filters). Extends the base
        (ins pos/vel/att) dict with attitude-only ``ars`` and ``ahrs``
        sub-dicts ({tripped, n, min_att_deg}); the ARS covers roll/pitch,
        the AHRS roll/pitch/yaw (its yaw is magnetometer-aided)."""
        oc = super().overconfidence()

        def _att(fn):
            mn = ctypes.c_float()
            n = fn(self._h, ctypes.byref(mn))
            return {"tripped": n > 0, "n": int(n), "min_att_deg": mn.value}
        oc["ars"] = _att(_lib.ins_suite_get_ars_overconfidence)
        oc["ahrs"] = _att(_lib.ins_suite_get_ahrs_overconfidence)
        return oc

    def height(self):
        """Best available height above the NED-origin datum [m, up], or
        None. Continuous across source changes (ins <-> baro)."""
        h = ctypes.c_float()
        return h.value if _lib.ins_suite_get_height(self._h, ctypes.byref(h)) else None

    def height_ellipsoid(self):
        """Best available absolute (ellipsoid) height [m], or None."""
        h = ctypes.c_float()
        return (h.value
                if _lib.ins_suite_get_height_ellipsoid(self._h, ctypes.byref(h))
                else None)

    # --- unified result -----------------------------------------------------
    def solution(self) -> Solution:
        rpy = self.rpy()
        sol = Solution(mode=self.mode_name(), ready=self.is_ready(),
                       attitude_ok=rpy is not None,
                       dr_ms=self.deadreckoning_ms())
        if rpy:
            sol.roll, sol.pitch, sol.yaw = rpy
        sol.pos_ecef = tuple(self.position_ecef() or ()) or None
        sol.pos_local = tuple(self.position_local() or ()) or None
        sol.vel_ned = tuple(self.velocity_ned() or ()) or None
        # Straight from the filter's own anchor rather than from pos_ecef:
        # that one is built FROM these three, so converting it back would be
        # a round trip for a value already on hand.
        llh = self.position_llh()
        if llh:
            sol.lat_rad, sol.lon_rad, sol.alt_m = llh
        h = self.height()
        if h is not None:
            sol.height_m = h
        h = self.height_ellipsoid()
        if h is not None:
            sol.height_ell_m = h
        b = self.baro_alt()
        if b is not None:
            sol.baro_height_m, sol.baro_vz_mps = b
        return sol

    def state(self) -> State:
        """A telemetry-ready :class:`State` with the *best* attitude filled.

        In ATTITUDE_ONLY mode position/velocity stay NaN but roll/pitch/yaw
        (and a synthesized quaternion) come from the AHRS fallback, so a
        heading is always on the wire when one exists.
        """
        best = self.rpy()
        st = State(ready=self.is_ready(), mode=self.mode_name(),
                   dr_ms=self.deadreckoning_ms())
        _fill_nav_state(st, self, best)
        # If ins has no quaternion (attitude-only) but we have Euler angles,
        # synthesize one so the attitude is still published.
        if best is not None and not math.isfinite(st.qw):
            st.qw, st.qx, st.qy, st.qz = rpy_to_quat(*best)
        # z_m/alt_m: ALWAYS defer to the suite's own arbitration
        # (height()/height_ellipsoid(), REQ-SUITE-007/008: ins under fresh
        # aiding, else baro, matching Navigator's module docstring), not
        # just as a fallback for a NaN. _fill_nav_state() above already
        # populated z_m/alt_m straight from ins's raw local/geodetic
        # position, which stays finite from initialization onward
        # (position_local()/position_llh() only check is_initialized, not
        # is_ready() or a real WGS84 anchor) -- e.g. while merely coasting, or before any
        # GNSS fix has ever anchored the origin, ins's own value is not
        # NaN but is also not the best height the suite actually has. (A
        # GNSS quality loss is not one of those cases: that re-arms ins,
        # so is_initialized goes false and the accessors do report None.)
        # Overwriting unconditionally -- including resetting to NaN when
        # the suite has no answer either, so a stale ins-only value never
        # survives under a name that no longer means "the suite's best
        # height" -- is what makes this the same "best known height"
        # MAVLink and PlotJuggler are documented to carry.
        h = self.height()
        st.z_m = -h if h is not None else math.nan
        h = self.height_ellipsoid()
        st.alt_m = h if h is not None else math.nan
        return st
