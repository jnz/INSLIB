"""Low-level ctypes plumbing over libINSLIB (see csrc/ins_capi.h).

This module loads the shared library, mirrors the flat config struct,
binds both ABI families (``ins_core_*`` bare filter, ``ins_suite_*`` suite) and
exposes the lean :class:`Ins` wrapper plus the shared :class:`Config`
and :class:`State` types. The high-level :class:`~INSLIB.suite.Navigator`
(nav_suite-backed) lives in ``suite.py`` and reuses this plumbing.

Conventions are ins's: body frame FRD, nav frame NED, Hamilton
quaternion ``q=[w,x,y,z]``, time in int64 microseconds, angles in radians.

(c) Jan Zwiener (jan@zwiener.org)
"""

import ctypes
import math
import os
import sys
from dataclasses import dataclass, field

_HERE = os.path.dirname(os.path.abspath(__file__))
# This platform's own suffix first. Building for both Windows and WSL out of
# one checkout leaves both files side by side, and loading the foreign one
# fails with a thoroughly misleading message ("not a valid Win32
# application" for an ELF .so), so the order is not cosmetic.
if sys.platform == "win32":
    _LIBNAMES = ("libINSLIB.dll", "libINSLIB.so", "libINSLIB.dylib")
elif sys.platform == "darwin":
    _LIBNAMES = ("libINSLIB.dylib", "libINSLIB.so", "libINSLIB.dll")
else:
    _LIBNAMES = ("libINSLIB.so", "libINSLIB.dylib", "libINSLIB.dll")


def _load_library():
    tried = []
    for name in _LIBNAMES:
        path = os.path.join(_HERE, name)
        if not os.path.exists(path):
            continue
        try:
            return ctypes.CDLL(path)
        except OSError as exc:
            tried.append("%s: %s" % (name, exc))
    if tried:
        raise OSError(
            "libINSLIB found but not loadable:\n  " + "\n  ".join(tried)
            + "\nRebuild for this platform with:  make pylib")
    raise OSError(
        "libINSLIB shared library not found in the INSLIB package. "
        "Build it first with:  make pylib"
    )


_lib = _load_library()

_f3 = ctypes.c_float * 3
_f4 = ctypes.c_float * 4
_f9 = ctypes.c_float * 9
_d3 = ctypes.c_double * 3
_u10 = ctypes.c_uint32 * 10

# Mirrors INS_CAPI_MAX_STATE (ins_capi.h) / INS_UNKNOWNS_MAG (ins.h).
MAX_STATE = 18
_fcov = ctypes.c_float * (MAX_STATE * MAX_STATE)

# Mirrors ins.h's INS_EPOCH_COV_PROPAGATED bit.
_INS_EPOCH_COV_PROPAGATED = 1 << 0


class _CfgStruct(ctypes.Structure):
    # Field order/types MUST mirror ins_cfg_t in csrc/ins_capi.h exactly.
    _fields_ = [
        ("time_us", ctypes.c_int64),
        ("lat_rad", ctypes.c_double),
        ("lon_rad", ctypes.c_double),
        ("h_m", ctypes.c_double),
        ("pos_init_stddev_m", ctypes.c_float),
        ("vel_init_stddev_mps", ctypes.c_float),
        ("rpy_init_stddev_rad", ctypes.c_float * 3),
        ("acc_bias_init_stddev_mps2", ctypes.c_float),
        ("gyr_bias_init_stddev_rps", ctypes.c_float),
        ("pos_pred_stddev_m_sqrts", ctypes.c_float),
        ("vel_pred_stddev_mps_sqrts", ctypes.c_float),
        ("rpy_pred_stddev_rad_sqrts", ctypes.c_float),
        ("acc_bias_pred_stddev_mps2_sqrts", ctypes.c_float),
        ("gyr_bias_pred_stddev_rps_sqrts", ctypes.c_float),
        ("zero_vel_stddev_mps", ctypes.c_float),
        ("zero_rot_stddev_rps", ctypes.c_float),
        ("gyr_bias_init_rps", ctypes.c_float * 3),
        ("magnetic_n", ctypes.c_float * 3),
        ("kalman_update_dt_sec", ctypes.c_float),
        ("max_prediction_time_sec", ctypes.c_float),
        ("gnss_max_horizontal_pos_stddev_m", ctypes.c_float),
        ("gnss_max_vertical_pos_stddev_m", ctypes.c_float),
        ("gnss_max_horizontal_vel_stddev_mps", ctypes.c_float),
        ("gnss_max_vertical_vel_stddev_mps", ctypes.c_float),
        ("magnetometer_min_delay_ms", ctypes.c_int32),
        ("auto_init", ctypes.c_int32),
        ("allow_unlimited_deadreckoning", ctypes.c_int32),
        ("rpy_init_rad", ctypes.c_float * 3),
        ("max_deadreckoning_sec", ctypes.c_float),
        ("auto_zupt_disable", ctypes.c_int32),
        ("mag_field_tolerance", ctypes.c_float),
        ("mag_field_check_disable", ctypes.c_int32),
        ("estimate_mag_bias", ctypes.c_int32),
        ("mag_bias_init_stddev_ut", ctypes.c_float),
        ("mag_bias_pred_stddev_ut_sqrts", ctypes.c_float),
        ("automotive_mode", ctypes.c_int32),
        ("automotive_min_speed_mps", ctypes.c_float),
        ("automotive_min_yaw_stddev", ctypes.c_float),
        ("chi2_disable", ctypes.c_int32),
        # IMU calibration (REQ-NAV-037) + GNSS covariance conditioning
        # (REQ-NAV-038). Appended to match ins_cfg_t's field order.
        ("imu_acc_misalignment", _f9),
        ("imu_gyr_misalignment", _f9),
        ("imu_acc_fixed_bias", _f3),
        ("imu_gyr_fixed_bias", _f3),
        ("gnss_pos_cov_scale", ctypes.c_float),
        ("gnss_pos_cov_scale_height", ctypes.c_float),
        ("gnss_vel_cov_scale", ctypes.c_float),
        ("gnss_pos_stddev_floor_hor_m", ctypes.c_float),
        ("gnss_pos_stddev_floor_ver_m", ctypes.c_float),
        ("gnss_vel_stddev_floor_hor_mps", ctypes.c_float),
        ("gnss_vel_stddev_floor_ver_mps", ctypes.c_float),
        ("mag_misalignment", _f9),
        ("mag_fixed_bias", _f3),
        ("init_vel_ned", _f3),
        ("chi2_reject_alpha", ctypes.c_float),
        ("auto_init_window_sec", ctypes.c_float),
        ("auto_zupt_static_gyr_rps", ctypes.c_float),
        ("auto_zupt_max_vel_mps", ctypes.c_float),
        # GNSS quality hysteresis around the 3D solution (REQ-NAV-051/052).
        ("gnss_start_max_horizontal_pos_stddev_m", ctypes.c_float),
        ("gnss_start_max_vertical_pos_stddev_m", ctypes.c_float),
        ("gnss_start_max_horizontal_vel_stddev_mps", ctypes.c_float),
        ("gnss_start_max_vertical_vel_stddev_mps", ctypes.c_float),
        ("gnss_stop_max_horizontal_pos_stddev_m", ctypes.c_float),
        ("gnss_stop_max_vertical_pos_stddev_m", ctypes.c_float),
        ("gnss_stop_max_horizontal_vel_stddev_mps", ctypes.c_float),
        ("gnss_stop_max_vertical_vel_stddev_mps", ctypes.c_float),
        ("gnss_init_dwell_sec", ctypes.c_float),
        ("gnss_init_dwell_disable", ctypes.c_int32),
        ("gnss_stop_dwell_sec", ctypes.c_float),
        ("gnss_stop_disable", ctypes.c_int32),
        ("baro_height_disable", ctypes.c_int32),
        # Rest of the stillness definition (REQ-SUITE-020), see the
        # auto_zupt_* fields on Config.
        ("auto_zupt_static_acc_mps2", ctypes.c_float),
        ("auto_zupt_max_vel_stddev_mps", ctypes.c_float),
        ("auto_zupt_static_gyr_stddev_rps", ctypes.c_float),
        ("auto_zupt_static_acc_stddev_mps2", ctypes.c_float),
        ("auto_zupt_dwell_sec", ctypes.c_float),
        ("auto_zupt_min_interval_sec", ctypes.c_float),
        ("auto_zupt_velocity_blind_disable", ctypes.c_int32),
        ("gnss_pos_decimation", ctypes.c_int32),
        ("speed_scale", ctypes.c_float),
        ("speed_stddev_rel", ctypes.c_float),
        ("speed_min_mps", ctypes.c_float),
        ("gnss_pos_stddev_cap_hor_m", ctypes.c_float),
        ("gnss_pos_stddev_cap_ver_m", ctypes.c_float),
        ("gnss_vel_stddev_cap_hor_mps", ctypes.c_float),
        ("gnss_vel_stddev_cap_ver_mps", ctypes.c_float),
        ("gnss_acc_envelope_tau_sec", ctypes.c_float),
        ("gnss_vel_noise_acc_scale_hor", ctypes.c_float),
        ("gnss_vel_noise_acc_scale_ver", ctypes.c_float),
        ("gnss_vel_noise_acc_window_sec", ctypes.c_float),
        ("gnss_min_delay_ms", ctypes.c_int32),
        # Non-holonomic lateral velocity constraint (REQ-NAV-077), appended
        # at the end to keep every offset above it stable.
        ("automotive_lateral_constraint", ctypes.c_int32),
        ("automotive_lateral_stddev_mps", ctypes.c_float),
        ("automotive_lateral_max_yaw_rate", ctypes.c_float),
        ("automotive_lateral_after_sec", ctypes.c_float),
        # Range aiding (REQ-NAV-082, REQ-NAV-085), appended at the end to
        # keep every offset above it stable.
        ("range_height_with_baro", ctypes.c_int32),
        ("range_aiding_max_hpos_stddev_m", ctypes.c_float),
    ]


# Solution modes (mirror INS_MODE_* / nav_suite_mode_t).
MODE_NONE, MODE_ATTITUDE_ONLY, MODE_COASTING, MODE_FULL = 0, 1, 2, 3
MODE_NAMES = {MODE_NONE: "NONE", MODE_ATTITUDE_ONLY: "ATTITUDE_ONLY",
              MODE_COASTING: "COASTING", MODE_FULL: "FULL"}

# Sensible IMU noise defaults (variance = PSD) so imu() is call-able without
# spelling out covariances for a quick start.
_DEF_ACC_VAR = (0.01, 0.01, 0.01)
_DEF_GYR_VAR = (1e-4, 1e-4, 1e-4)

_DIAG_KEYS = ("n_predict", "n_gnss_seen", "n_gnss_used",
              "n_gnss_rejected_noise", "n_gnss_no_anchor", "n_fuse_fail",
              "n_auto_zupt", "n_invalid_input", "n_downweighted",
              "n_baro_height_used")


def _bind_family(p):
    """Bind argtypes/restypes for one ABI prefix ('ins_core' or 'ins_suite')."""
    L = _lib
    g = lambda n: getattr(L, p + "_" + n)  # noqa: E731
    g("create").restype = ctypes.c_void_p
    g("destroy").argtypes = [ctypes.c_void_p]
    g("init").argtypes = [ctypes.c_void_p, ctypes.POINTER(_CfgStruct)]
    g("init").restype = ctypes.c_int
    g("set_imu").argtypes = [ctypes.c_void_p, ctypes.c_int64, ctypes.c_float,
                             _f3, _f3, _f3, _f3]
    g("set_gnss_pos_llh").argtypes = [ctypes.c_void_p, _d3, _f3]
    g("set_gnss_vel_ned").argtypes = [ctypes.c_void_p, _f3, _f3]
    g("set_gnss_pos_llh_cov").argtypes = [ctypes.c_void_p, _d3, _f9]
    g("set_gnss_vel_ned_cov").argtypes = [ctypes.c_void_p, _f3, _f9]
    g("set_gnss_pos_vel_cov").argtypes = [ctypes.c_void_p, _f9]
    g("set_gnss_leverarm_b").argtypes = [ctypes.c_void_p, _f3]
    g("set_mag").argtypes = [ctypes.c_void_p, _f3, _f3]
    g("set_yaw").argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    g("set_local_pos").argtypes = [ctypes.c_void_p, _f3, _f3, _f3]
    g("set_zupt").argtypes = [ctypes.c_void_p, ctypes.c_int]
    g("set_zaru").argtypes = [ctypes.c_void_p, ctypes.c_int]
    g("set_baro").argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    g("set_speed").argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float,
                               ctypes.c_int]
    g("get_speed_diag").argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32),
                                    ctypes.POINTER(ctypes.c_float)]
    g("get_time_diag").argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
    g("add_range").argtypes = [ctypes.c_void_p, _d3, ctypes.c_float, ctypes.c_float,
                               ctypes.c_int, ctypes.c_uint16]
    g("add_range").restype = ctypes.c_int
    g("set_range_leverarm").argtypes = [ctypes.c_void_p, _f3]
    g("get_range_diag").argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32),
                                    ctypes.POINTER(ctypes.c_float)]
    for n in ("set_gnss_delay_ms", "set_yaw_delay_ms",
              "set_local_pos_delay_ms"):
        g(n).argtypes = [ctypes.c_void_p, ctypes.c_int]
    for n in ("set_gnss_delay_us", "set_local_pos_delay_us", "set_yaw_delay_us",
              "set_speed_delay_us"):
        g(n).argtypes = [ctypes.c_void_p, ctypes.c_int32]
    g("set_magnetic_model_position").argtypes = [
        ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_float]
    g("update").argtypes = [ctypes.c_void_p]
    g("predict").argtypes = [ctypes.c_void_p, _fcov]
    g("predict").restype = ctypes.c_int
    g("correct").argtypes = [ctypes.c_void_p]
    for n in ("is_ready", "deadreckoning_ms"):
        g(n).argtypes = [ctypes.c_void_p]
        g(n).restype = ctypes.c_int
    for n in ("get_position_ecef", "get_origin_ecef", "get_latlonh"):
        g(n).argtypes = [ctypes.c_void_p, _d3]
        g(n).restype = ctypes.c_int
    for n in ("get_position_local", "get_velocity_ned", "get_omega_b_nb",
              "get_acc_n", "get_bias_acc", "get_bias_gyr", "get_bias_mag"):
        g(n).argtypes = [ctypes.c_void_p, _f3]
        g(n).restype = ctypes.c_int
    g("get_quaternion").argtypes = [ctypes.c_void_p, _f4]
    g("get_quaternion").restype = ctypes.c_int
    g("get_rotmat_b_to_n").argtypes = [ctypes.c_void_p, _f9]
    g("get_rotmat_b_to_n").restype = ctypes.c_int
    g("get_covariance").argtypes = [ctypes.c_void_p, _fcov]
    g("get_covariance").restype = ctypes.c_int
    g("get_diag").argtypes = [ctypes.c_void_p, _u10]
    g("get_overconfidence").argtypes = [ctypes.c_void_p, _f3]
    g("get_overconfidence").restype = ctypes.c_uint32
    g("auto_zupt_active").argtypes = [ctypes.c_void_p]
    g("auto_zupt_active").restype = ctypes.c_int


def _bind():
    _bind_family("ins_core")
    _bind_family("ins_suite")
    L = _lib
    # ins_core has get_rpy on the bare filter.
    L.ins_core_get_rpy.argtypes = [ctypes.c_void_p, _f3]
    L.ins_core_get_rpy.restype = ctypes.c_int
    # ins_suite adds mode + the per-source attitudes.
    L.ins_suite_get_mode.argtypes = [ctypes.c_void_p]
    L.ins_suite_get_mode.restype = ctypes.c_int
    for n in ("ins_suite_get_rpy", "ins_suite_get_rpy_ins", "ins_suite_get_rpy_ars",
              "ins_suite_get_rpy_ahrs", "ins_suite_get_bias_gyr_ars", "ins_suite_get_bias_gyr_ahrs",
              "ins_suite_get_gyr_bias_stddev_ars", "ins_suite_get_gyr_bias_stddev_ahrs",
              "ins_suite_get_rpy_stddev_ars", "ins_suite_get_rpy_stddev_ahrs"):
        getattr(L, n).argtypes = [ctypes.c_void_p, _f3]
        getattr(L, n).restype = ctypes.c_int
    # ns adds the vertical channel (baro filter + height arbitration).
    _pf = ctypes.POINTER(ctypes.c_float)
    L.ins_suite_get_baro_alt.argtypes = [ctypes.c_void_p, _pf, _pf]
    L.ins_suite_get_baro_alt.restype = ctypes.c_int
    L.ins_suite_get_baro_acc_bias.argtypes = [ctypes.c_void_p, _pf]
    L.ins_suite_get_baro_acc_bias.restype = ctypes.c_int
    L.ins_suite_get_baro_stddev.argtypes = [ctypes.c_void_p, _f3]
    L.ins_suite_get_baro_stddev.restype = ctypes.c_int
    L.ins_suite_get_local_gnss_offset.argtypes = [ctypes.c_void_p, _pf, _pf]
    L.ins_suite_get_local_gnss_offset.restype = ctypes.c_int
    for n in ("ins_suite_get_height", "ins_suite_get_height_ellipsoid"):
        getattr(L, n).argtypes = [ctypes.c_void_p, _pf]
        getattr(L, n).restype = ctypes.c_int
    # ns adds the velocity-blind auto-ZARU fallback (REQ-AHRS-017) and its
    # diagnostics. No bare-ins equivalent (ARS/AHRS only exist here).
    L.ins_suite_set_auto_zaru.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    # ns adds the baro/accel vertical channel: its acc-bias drift density has
    # no bare-ins equivalent (baro_alt only exists in the suite).
    L.ins_suite_set_baro_acc_bias_drift.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_baro_acc_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_baro_acc_bias_init_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_baro_h_init_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_baro_v_init_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_baro_h_process_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
    # ns adds the local-height/GNSS-ellipsoid offset filter's tuning: no
    # bare-ins equivalent (local_gnss_alt only exists in the suite).
    L.ins_suite_set_local_gnss_rw_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_local_gnss_chi2_threshold.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_local_gnss_min_update_interval.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_local_gnss_stddev_inflation.argtypes = [ctypes.c_void_p, ctypes.c_float]
    # ns adds the ARS/AHRS noise model: no bare-ins equivalent (ARS/AHRS
    # only exist in the suite); one call sets both sub-filters' template.
    L.ins_suite_set_ahrs_gyr_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_ahrs_acc_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_ahrs_gyr_bias_rw.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_ahrs_rpy_pred_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_ahrs_gyr_bias_init_stddev.argtypes = [ctypes.c_void_p, ctypes.c_float]
    L.ins_suite_set_ahrs_tuning.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 9
    L.ins_suite_set_init_att_hint.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                              ctypes.c_float, ctypes.c_float,
                                              ctypes.c_float, ctypes.c_float]
    for n in ("ins_suite_zaru_active", "ins_suite_ars_auto_zaru_active",
              "ins_suite_ahrs_auto_zaru_active", "ins_suite_ars_zaru_applied",
              "ins_suite_ahrs_zaru_applied", "ins_suite_vertical_zupt_active"):
        getattr(L, n).argtypes = [ctypes.c_void_p]
        getattr(L, n).restype = ctypes.c_int
    # ns adds downweight counters for the other sub-filters (REQ-AHRS-019,
    # REQ-BARO-017). No bare-ins equivalent; ins's own is in diag().
    for n in ("ins_suite_get_ars_downweight_count", "ins_suite_get_ahrs_downweight_count",
              "ins_suite_get_baro_downweight_count", "ins_suite_get_local_gnss_downweight_count"):
        getattr(L, n).argtypes = [ctypes.c_void_p]
        getattr(L, n).restype = ctypes.c_uint32
    # ns attitude overconfidence watchdog for ARS/AHRS (REQ-AHRS-020).
    for n in ("ins_suite_get_ars_overconfidence", "ins_suite_get_ahrs_overconfidence"):
        getattr(L, n).argtypes = [ctypes.c_void_p, _pf]
        getattr(L, n).restype = ctypes.c_uint32


_bind()


@dataclass
class Config:
    """Filter configuration (a friendly view of ``ins_cfg_t``).

    Every tuning field defaults to 0, which the C library resolves to its
    own default (ins.c, REQ-NAV-043/049). The binding deliberately keeps no
    numbers of its own: a Python-side copy would silently differ from the C
    filter the day either side is tuned. An empty ``Config()`` is a working
    setup, override only what you need, for example the initial position
    (ignored when ``auto_init`` bootstraps from the first fix) and the noise
    terms.
    """
    time_us: int = 0
    """Initialization time stamp [us]."""
    lat_rad: float = 0.0
    """Initial latitude [rad]. Overridden by ``auto_init``."""
    lon_rad: float = 0.0
    """Initial longitude [rad]. Overridden by ``auto_init``."""
    h_m: float = 0.0
    """Initial ellipsoidal height [m]. Overridden by ``auto_init``."""
    pos_init_stddev_m: float = 0.0
    """Initial position 1-sigma [m], 0 -> C default."""
    vel_init_stddev_mps: float = 0.0
    """Initial velocity 1-sigma [m/s], 0 -> C default."""
    rpy_init_stddev_rad: tuple = (0.0, 0.0, 0.0)
    """Initial roll/pitch/yaw 1-sigma [rad], each axis on its own (each 0 ->
    its own C default, no cross-axis fallback)."""
    acc_bias_init_stddev_mps2: float = 0.0
    """Initial accelerometer bias 1-sigma [m/s^2], 0 -> C default."""
    gyr_bias_init_stddev_rps: float = 0.0
    """Initial gyro bias 1-sigma [rad/s], 0 -> C default."""
    pos_pred_stddev_m_sqrts: float = 0.0
    """Position process noise [m/sqrt(s)], 0 -> C default."""
    vel_pred_stddev_mps_sqrts: float = 0.0
    """Velocity process noise [m/s/sqrt(s)], 0 -> C default."""
    rpy_pred_stddev_rad_sqrts: float = 0.0
    """Extra attitude process noise [rad/sqrt(s)], 0 -> C default."""
    acc_bias_pred_stddev_mps2_sqrts: float = 0.0
    """Accelerometer bias random walk [m/s^2/sqrt(s)], 0 -> C default."""
    gyr_bias_pred_stddev_rps_sqrts: float = 0.0
    """Gyro bias random walk [rad/s/sqrt(s)], 0 -> C default."""
    zero_vel_stddev_mps: float = 0.0
    """1-sigma of a zero-velocity update (ZUPT) [m/s], 0 -> C default."""
    zero_rot_stddev_rps: float = 0.0
    """1-sigma of a zero-rotation update (ZARU) [rad/s], 0 -> C default."""
    gyr_bias_init_rps: tuple = (0.0, 0.0, 0.0)
    """Initial gyro bias estimate [rad/s]. Only seeds the estimated bias
    state, unlike ``imu_gyr_fixed_bias``."""
    magnetic_n: tuple = (0.0, 0.0, 0.0)
    """Magnetic reference field in NED [uT] for magnetometer fusion.
    ``set_magnetic_model()`` fills it from the World Magnetic Model."""
    kalman_update_dt_sec: float = 0.0
    """Covariance prediction period [s], 0 -> C default. The strapdown
    itself still runs at the full IMU rate."""
    max_prediction_time_sec: float = 0.0
    """Largest allowed time step [s] (REQ-NAV-043), a longer one counts as
    a time jump, 0 -> C default."""
    gnss_max_horizontal_pos_stddev_m: float = 0.0
    """GNSS fusion gate: largest accepted horizontal position 1-sigma [m],
    0 -> C default."""
    gnss_max_vertical_pos_stddev_m: float = 0.0
    """GNSS fusion gate: largest accepted vertical position 1-sigma [m],
    0 -> C default."""
    gnss_max_horizontal_vel_stddev_mps: float = 0.0
    """GNSS fusion gate: largest accepted horizontal velocity 1-sigma [m/s],
    0 -> C default."""
    gnss_max_vertical_vel_stddev_mps: float = 0.0
    """GNSS fusion gate: largest accepted vertical velocity 1-sigma [m/s],
    0 -> C default."""
    magnetometer_min_delay_ms: int = 0
    """Minimum time between two fused magnetometer samples [ms], 0 -> C
    default, negative -> no rate limit (fuse every sample)."""
    auto_init: bool = True
    """Bootstrap position, velocity and attitude from the first usable
    measurements instead of the initial values given here."""
    allow_unlimited_deadreckoning: bool = False
    """Never expire the IMU-only coasting window. Lets the filter start and
    run without any position aiding."""
    rpy_init_rad: tuple = (0.0, 0.0, 0.0)
    """Initial roll/pitch/yaw [rad], used with ``auto_init=False``."""
    init_vel_ned: tuple = (0.0, 0.0, 0.0)
    """Initial NED velocity [m/s] for a non-stationary manual start
    (``auto_init=False``), 0 -> start at rest."""
    max_deadreckoning_sec: float = 0.0
    """IMU-only coasting budget [s] before ``ready`` degrades, 0 -> C
    default."""
    auto_zupt_disable: bool = False
    """Disable the built-in stillness detector (automatic ZUPT)."""
    mag_field_tolerance: float = 0.0
    """Largest deviation of the measured field strength from the World
    Magnetic Model, as a fraction, before the magnetometer is downweighted,
    0 -> C default. Needs the magnetometer in uT."""
    mag_field_check_disable: bool = False
    """Skip the field strength check, for an uncalibrated or unit-less
    magnetometer."""
    estimate_mag_bias: bool = False
    """18-state mode: estimate the magnetometer hard-iron bias."""
    mag_bias_init_stddev_ut: float = 0.0
    """Initial hard-iron bias 1-sigma [uT], 0 -> C default."""
    mag_bias_pred_stddev_ut_sqrts: float = 0.0
    """Hard-iron bias random walk [uT/sqrt(s)], 0 -> C default."""
    chi2_disable: bool = False
    """Disable the chi2 outlier downweighting library-wide (diagnostics
    only)."""
    automotive_mode: bool = False
    """Yaw from the GNSS course over ground."""
    automotive_min_speed_mps: float = 0.0
    """Ground speed floor [m/s] for ``automotive_mode``, 0 -> C default."""
    automotive_min_yaw_stddev: float = 0.0
    """Floor [rad] on the yaw 1-sigma that ``automotive_mode`` fuses, 0 -> C
    default. Raise it where the course-equals-heading assumption is itself
    looser than a car's (aircraft wind drift, for example)."""
    automotive_lateral_constraint: bool = False
    """Non-holonomic lateral velocity constraint (REQ-NAV-077), needs
    ``automotive_mode``."""
    automotive_lateral_stddev_mps: float = 0.0
    """1-sigma of the lateral velocity constraint [m/s], 0 -> C default."""
    automotive_lateral_max_yaw_rate: float = 0.0
    """|Yaw rate| [rad/s] above which the lateral constraint is skipped,
    where side slip breaks the assumption, 0 -> C default."""
    automotive_lateral_after_sec: float = 0.0
    """Hold the lateral constraint off until this long [s] without a GNSS
    fusion, 0 -> C default, negative -> no delay."""
    range_height_with_baro: bool = False
    """Let ranges correct the height also while the barometer is the height
    source."""
    range_aiding_max_hpos_stddev_m: float = 0.0
    """Horizontal 1-sigma [m] in its worst direction below which fused ranges
    count as position aiding, 0 -> C default."""
    imu_acc_misalignment: tuple = (0.0,) * 9
    """Accelerometer calibration matrix M, column-major 3x3 (all 0 ->
    identity). The calibration (REQ-NAV-037) is ``corrected = M * (raw -
    fixed_bias)``, applied to the raw samples at the measurement boundary."""
    imu_gyr_misalignment: tuple = (0.0,) * 9
    """Gyro calibration matrix M, column-major 3x3 (all 0 -> identity)."""
    imu_acc_fixed_bias: tuple = (0.0, 0.0, 0.0)
    """Accelerometer fixed bias [m/s^2], removed permanently, never
    estimated."""
    imu_gyr_fixed_bias: tuple = (0.0, 0.0, 0.0)
    """Gyro fixed bias [rad/s], removed permanently, never estimated (unlike
    ``gyr_bias_init_rps``, which only seeds the estimated bias)."""
    gnss_pos_cov_scale: float = 0.0
    """Factor on the reported GNSS position 1-sigma, 0 -> 1. The GNSS
    covariance conditioning (REQ-NAV-038) is ``stddev = max(scale *
    stddev_reported, floor)``, applied before the gates and the fusion."""
    gnss_pos_cov_scale_height: float = 0.0
    """Factor on the reported GNSS height 1-sigma, 0 -> 1."""
    gnss_vel_cov_scale: float = 0.0
    """Factor on the reported GNSS velocity 1-sigma, 0 -> 1."""
    gnss_pos_stddev_floor_hor_m: float = 0.0
    """Smallest horizontal GNSS position 1-sigma [m], 0 -> no floor."""
    gnss_pos_stddev_floor_ver_m: float = 0.0
    """Smallest vertical GNSS position 1-sigma [m], 0 -> no floor."""
    gnss_vel_stddev_floor_hor_mps: float = 0.0
    """Smallest horizontal GNSS velocity 1-sigma [m/s], 0 -> no floor."""
    gnss_vel_stddev_floor_ver_mps: float = 0.0
    """Smallest vertical GNSS velocity 1-sigma [m/s], 0 -> no floor."""
    mag_misalignment: tuple = (0.0,) * 9
    """Magnetometer soft-iron, scale and misalignment matrix M, column-major
    3x3 (all 0 -> identity), ``corrected = M * (raw - fixed_bias)``
    (REQ-NAV-039). The Navigator forwards it to the magnetometer AHRS too
    (REQ-SUITE-012)."""
    mag_fixed_bias: tuple = (0.0, 0.0, 0.0)
    """Magnetometer hard-iron offset [uT]. Independent of the estimated
    hard-iron bias of ``estimate_mag_bias``."""
    chi2_reject_alpha: float = 0.0
    """Global chi2 downweight significance level (REQ-NAV-046): 0 -> each
    channel keeps its own gate, a positive alpha sets one shared threshold
    ``chi2inv(1 - alpha, 1)`` for all channels."""
    auto_init_window_sec: float = 0.0
    """IMU leveling window [s] of the ``auto_init`` bootstrap, 0 -> C
    default. Raise it for a low IMU rate so the window still spans at least
    3 samples, otherwise the bootstrap silently never fires."""
    auto_zupt_static_gyr_rps: float = 0.0
    """Stillness detector (REQ-NAV-013), loose sanity bound on |gyro|
    [rad/s], 0 -> C default. For a Navigator this one set defines
    standstill for the whole suite, the ARS/AHRS and through them the
    vertical channel (REQ-SUITE-020). The window 1-sigma values below are
    the primary criterion, these magnitude bounds only sanity limits."""
    auto_zupt_static_acc_mps2: float = 0.0
    """Stillness detector, loose sanity bound on ||f| - g| [m/s^2], 0 -> C
    default."""
    auto_zupt_static_gyr_stddev_rps: float = 0.0
    """Stillness detector, primary criterion: per-axis RMS 1-sigma of the
    raw gyro over a short window [rad/s], 0 -> C default. Raising it far
    above any plausible sensor noise leaves the magnitude bounds in charge,
    which a platform with an inherently noisy standstill (a hovering
    multicopter) needs."""
    auto_zupt_static_acc_stddev_mps2: float = 0.0
    """Stillness detector, primary criterion: per-axis RMS 1-sigma of the
    raw accelerometer over a short window [m/s^2], 0 -> C default."""
    auto_zupt_max_vel_mps: float = 0.0
    """Largest GNSS speed [m/s] that still arms the automatic ZUPT. Applied
    only while a recent, precise enough fix exists, never against the
    filter's own velocity state, which would be circular."""
    auto_zupt_max_vel_stddev_mps: float = 0.0
    """Accuracy [m/s] the GNSS velocity has to report (1-sigma, per axis) to
    count as evidence of standstill."""
    auto_zupt_dwell_sec: float = 0.0
    """Stillness [s] required before an automatic ZUPT triggers."""
    auto_zupt_min_interval_sec: float = 0.0
    """Smallest time [s] between two automatic ZUPT triggers."""
    speed_scale: float = 0.0
    """Factor on the reported speed (odometer), 0 -> 1."""
    speed_stddev_rel: float = 0.0
    """Speed-proportional 1-sigma of the speed measurement, as a fraction of
    the speed, 0 -> C default."""
    speed_min_mps: float = 0.0
    """Filtered speed [m/s] below which a speed sample is skipped: the
    direction of motion is undefined at standstill and meaningless just
    above it, 0 -> C default."""
    gnss_pos_stddev_cap_hor_m: float = 0.0
    """Upper clamp of the horizontal GNSS position 1-sigma [m] (REQ-NAV-071),
    0 -> C default, negative -> not applied."""
    gnss_pos_stddev_cap_ver_m: float = 0.0
    """Upper clamp of the vertical GNSS position 1-sigma [m], 0 -> C
    default, negative -> not applied."""
    gnss_vel_stddev_cap_hor_mps: float = 0.0
    """Upper clamp of the horizontal GNSS velocity 1-sigma [m/s], 0 -> C
    default, negative -> not applied."""
    gnss_vel_stddev_cap_ver_mps: float = 0.0
    """Upper clamp of the vertical GNSS velocity 1-sigma [m/s], 0 -> C
    default, negative -> not applied."""
    gnss_acc_envelope_tau_sec: float = 0.0
    """Decay time constant [s] of the asymmetric GNSS accuracy envelope
    (REQ-NAV-072), 0 -> C default, negative -> not applied."""
    gnss_vel_noise_acc_scale_hor: float = 0.0
    """Manoeuvre-dependent GNSS velocity noise on N and E (REQ-NAV-073):
    extra velocity 1-sigma [m/s] per [m/s^2] of antenna acceleration (the
    body's plus the centripetal term of the lever arm), 0 -> off."""
    gnss_vel_noise_acc_scale_ver: float = 0.0
    """Same as ``gnss_vel_noise_acc_scale_hor``, on D."""
    gnss_vel_noise_acc_window_sec: float = 0.0
    """Averaging window [s] of the acceleration that the manoeuvre term
    reads (REQ-NAV-075), 0 -> C default, negative -> instantaneous."""
    gnss_min_delay_ms: int = 0
    """Smallest time between two fused GNSS epochs [ms] (REQ-NAV-074), 0 ->
    C default, negative -> no limit."""
    auto_zupt_velocity_blind_disable: bool = False
    """Opt only the ARS/AHRS's own stillness detector out (REQ-AHRS-017),
    the INS keeps deciding for everyone. For a platform dominated by
    constant-velocity cruise, which no velocity-blind detector can tell from
    a standstill."""
    gnss_start_max_horizontal_pos_stddev_m: float = 0.0
    """Entry gate of the 3D solution (REQ-NAV-051/052): largest horizontal
    position 1-sigma [m], 0 -> C default. Separate from the ``gnss_max_*``
    fusion gates: the strict ``start_*`` set decides whether the 3D solution
    may be entered, the loose ``stop_*`` set whether it has to be left
    (after ``gnss_stop_dwell_sec`` of nothing better). Leaving re-arms the
    filter: position and velocity go None until a new bootstrap through the
    entry gate, which carries only the IMU biases over."""
    gnss_start_max_vertical_pos_stddev_m: float = 0.0
    """Entry gate: largest vertical position 1-sigma [m], 0 -> C default."""
    gnss_start_max_horizontal_vel_stddev_mps: float = 0.0
    """Entry gate: largest horizontal velocity 1-sigma [m/s], 0 -> C
    default."""
    gnss_start_max_vertical_vel_stddev_mps: float = 0.0
    """Entry gate: largest vertical velocity 1-sigma [m/s], 0 -> C
    default."""
    gnss_stop_max_horizontal_pos_stddev_m: float = 0.0
    """Exit gate: horizontal position 1-sigma [m] above which the 3D
    solution is left, 0 -> C default."""
    gnss_stop_max_vertical_pos_stddev_m: float = 0.0
    """Exit gate: vertical position 1-sigma [m], 0 -> C default."""
    gnss_stop_max_horizontal_vel_stddev_mps: float = 0.0
    """Exit gate: horizontal velocity 1-sigma [m/s], 0 -> C default."""
    gnss_stop_max_vertical_vel_stddev_mps: float = 0.0
    """Exit gate: vertical velocity 1-sigma [m/s], 0 -> C default."""
    gnss_init_dwell_sec: float = 0.0
    """How long [s] the fixes have to stay at entry quality before the 3D
    solution is entered, 0 -> C default."""
    gnss_init_dwell_disable: bool = False
    """Enter the 3D solution on the first good fix."""
    gnss_stop_dwell_sec: float = 0.0
    """How long [s] the fixes have to stay worse than the exit gate before
    the 3D solution is left, 0 -> C default."""
    gnss_stop_disable: bool = False
    """Never leave the 3D solution because of GNSS quality."""
    baro_height_disable: bool = False
    """Never select the barometric height at bootstrap (REQ-NAV-053), even
    with a barometer present. For a GNSS-labelled source that is not really
    satellite GNSS and already reports better-than-barometric vertical
    accuracy."""
    gnss_pos_decimation: int = 0
    """Fuse the GNSS position only on every Nth epoch that offers a usable
    position and velocity, the velocity alone on the other N-1
    (REQ-NAV-063). A receiver's two blocks come out of one coupled solution
    whose cross-covariance it does not report, so fusing both counts the
    same information twice. N <= 1 -> off, 0 -> C default."""

    def _to_struct(self) -> _CfgStruct:
        s = _CfgStruct()
        _bools = ("auto_init", "allow_unlimited_deadreckoning", "range_height_with_baro",
                  "auto_zupt_disable", "mag_field_check_disable",
                  "estimate_mag_bias", "automotive_mode", "chi2_disable",
                  "automotive_lateral_constraint",
                  "gnss_init_dwell_disable", "gnss_stop_disable",
                  "baro_height_disable", "auto_zupt_velocity_blind_disable")
        _vec3 = ("rpy_init_stddev_rad", "gyr_bias_init_rps", "magnetic_n",
                 "rpy_init_rad", "init_vel_ned",
                 "imu_acc_fixed_bias", "imu_gyr_fixed_bias", "mag_fixed_bias")
        _vec9 = ("imu_acc_misalignment", "imu_gyr_misalignment",
                 "mag_misalignment")
        for f, _t in _CfgStruct._fields_:
            if f in _vec3:
                setattr(s, f, _f3(*getattr(self, f)))
            elif f in _vec9:
                setattr(s, f, _f9(*getattr(self, f)))
            elif f in _bools:
                setattr(s, f, 1 if getattr(self, f) else 0)
            else:
                setattr(s, f, getattr(self, f))
        return s


@dataclass
class State:
    """A snapshot of the estimate (NED / body FRD). Missing fields are NaN.

    Field names mirror the telemetry adapter so it can forward them
    without renaming. ``mode`` is only meaningful for the suite Navigator.
    """
    ready: bool = False
    mode: str = "NONE"
    dr_ms: int = -1
    x_m: float = math.nan
    y_m: float = math.nan
    z_m: float = math.nan
    vx_mps: float = math.nan
    vy_mps: float = math.nan
    vz_mps: float = math.nan
    lat_rad: float = math.nan
    lon_rad: float = math.nan
    alt_m: float = math.nan
    qw: float = math.nan
    qx: float = math.nan
    qy: float = math.nan
    qz: float = math.nan
    roll_rad: float = math.nan
    pitch_rad: float = math.nan
    yaw_rad: float = math.nan
    roll_rate_rps: float = math.nan
    pitch_rate_rps: float = math.nan
    yaw_rate_rps: float = math.nan
    ax_mps2: float = math.nan
    ay_mps2: float = math.nan
    az_mps2: float = math.nan


class _Base:
    """Shared input plumbing for a handle of a given ABI prefix."""
    _prefix = None  # 'ins_core' or 'ins_suite'

    def __init__(self, config: Config = None):
        p = self._prefix
        self._c = lambda n: getattr(_lib, p + "_" + n)
        self._h = self._c("create")()
        if not self._h:
            raise MemoryError(f"{p}_create failed")
        self._n_states = 15  # updated by init(); default matches the 15-state mode
        # Always initialized: a handle that never saw init() runs on an
        # all-zero setup (no IMU timeout, so every epoch reads as an IMU
        # loss) and never produces a solution. Navigator() therefore means
        # Navigator(Config()), init() may still be called again later.
        self.init(config if config is not None else Config())

    def init(self, config: Config):
        rc = self._c("init")(self._h, ctypes.byref(config._to_struct()))
        if rc != 0:
            raise ValueError(f"{self._prefix}_init failed (rc={rc})")
        self._n_states = 18 if getattr(config, "estimate_mag_bias", False) else 15
        return rc

    # --- per-epoch inputs ---------------------------------------------------
    def imu(self, t_us, dt_sec, acc, gyr, acc_var=_DEF_ACC_VAR,
            gyr_var=_DEF_GYR_VAR):
        """Begin an epoch and set the IMU sample (clears prior aiding)."""
        self._c("set_imu")(self._h, int(t_us), float(dt_sec),
                           _f3(*acc), _f3(*gyr), _f3(*acc_var), _f3(*gyr_var))

    @staticmethod
    def _cov9(cov):
        """Accept a 3x3 covariance as nested rows or a flat length-9
        sequence (row-major) and return the column-major flat _f9 the C
        side expects. Returns None if ``cov`` is a plain length-3
        diagonal."""
        rows = list(cov)
        if len(rows) == 3 and hasattr(rows[0], "__len__"):
            flat = [rows[i][j] for i in range(3) for j in range(3)]
        elif len(rows) == 9:
            flat = [float(v) for v in rows]
        else:
            return None
        # row-major (i,j)=flat[3i+j] -> column-major out[i+3j]=flat[3i+j]
        return _f9(*[flat[3 * i + j] for j in range(3) for i in range(3)])

    def gnss_pos_llh(self, llh, var_ned, delay_ms=0, delay_us=0):
        """GNSS position as the receiver reports it: latitude [rad],
        longitude [rad], height above the WGS84 ellipsoid [m].
        ``var_ned`` is the NED position variance, either a length-3
        diagonal or a full 3x3; ``delay_ms`` is the fix's age at the
        moment it is handed over, ``delay_us`` a further age in
        microseconds added to it (REQ-NAV-092), e.g. how much older the
        fix is than the IMU epoch it is attached to.

        This is the only position input: the filter builds its residual
        from the geodetic difference to its own anchor. A source that is
        natively ECEF converts first, e.g. with :func:`ecef_to_llh`, which
        keeps that cost out of the filter's own worst case."""
        q9 = self._cov9(var_ned)
        if q9 is not None:
            self._c("set_gnss_pos_llh_cov")(self._h, _d3(*llh), q9)
        else:
            self._c("set_gnss_pos_llh")(self._h, _d3(*llh), _f3(*var_ned))
        if delay_ms:
            self._c("set_gnss_delay_ms")(self._h, int(delay_ms))
        if delay_us:
            self._c("set_gnss_delay_us")(self._h, int(delay_us))

    def gnss_vel(self, vel_ned, var_ned):
        """GNSS NED velocity; ``var_ned`` diagonal (3) or full 3x3 like
        :meth:`gnss_pos_llh`."""
        q9 = self._cov9(var_ned)
        if q9 is not None:
            self._c("set_gnss_vel_ned_cov")(self._h, _f3(*vel_ned), q9)
        else:
            self._c("set_gnss_vel_ned")(self._h, _f3(*vel_ned), _f3(*var_ned))

    def gnss_pos_vel_cov(self, cov):
        """Optional position/velocity cross-covariance block: element
        (i,j) = cov(pos_ned_i, vel_ned_j) [m^2/s], nested rows or flat
        row-major length 9. With the two block covariances this forms
        the full 6x6 covariance of [pos; vel]. Only consumed when both
        gnss_pos and gnss_vel are set this epoch."""
        q9 = self._cov9(cov)
        if q9 is None:
            raise ValueError("expected a 3x3 covariance (nested or flat 9)")
        self._c("set_gnss_pos_vel_cov")(self._h, q9)

    def gnss_leverarm(self, lever_b):
        self._c("set_gnss_leverarm_b")(self._h, _f3(*lever_b))

    def mag(self, data, var):
        self._c("set_mag")(self._h, _f3(*data), _f3(*var))

    def yaw(self, yaw_rad, stddev_rad, delay_ms=0, delay_us=0):
        self._c("set_yaw")(self._h, float(yaw_rad), float(stddev_rad))
        if delay_ms:
            self._c("set_yaw_delay_ms")(self._h, int(delay_ms))
        if delay_us:
            self._c("set_yaw_delay_us")(self._h, int(delay_us))

    def local_pos(self, pos_ned, var_ned, leverarm_b=(0.0, 0.0, 0.0),
                  delay_ms=0, delay_us=0):
        self._c("set_local_pos")(self._h, _f3(*pos_ned), _f3(*var_ned),
                                 _f3(*leverarm_b))
        if delay_ms:
            self._c("set_local_pos_delay_ms")(self._h, int(delay_ms))
        if delay_us:
            self._c("set_local_pos_delay_us")(self._h, int(delay_us))

    def baro(self, pressure_pa, stddev_m=0.0):
        """Barometer static pressure [Pa] (stddev of the derived altitude
        [m]; 0 -> default). Feeds the suite's vertical channel filter.
        The bare Ins accepts but ignores it."""
        self._c("set_baro")(self._h, float(pressure_pa), float(stddev_m))

    def speed(self, speed_mps, stddev_mps=0.0, delay_ms=0, delay_us=0):
        """Absolute speed aiding (REQ-NAV-068): the scalar magnitude of the
        velocity, e.g. an OBD-II vehicle speed or a wheel-odometry rate.

        stddev_mps is the PER-SAMPLE 1-sigma only (0 -> default). The
        systematic scale error belongs in the config's speed_scale /
        speed_stddev_rel, where a constant bias can be removed instead of
        being disguised as noise. delay_ms is how old the sample is; it is
        history-anchored exactly like a delayed GNSS fix, delay_us adds a
        microsecond part (REQ-NAV-092)."""
        self._c("set_speed")(self._h, float(speed_mps), float(stddev_mps),
                             int(delay_ms))
        if delay_us:
            self._c("set_speed_delay_us")(self._h, int(delay_us))

    def range(self, anchor_ecef, range_m, stddev_m, delay_ms=0, anchor_id=0):
        """Range to an anchor at a known position (REQ-NAV-082): anchor in
        ECEF [m], the calibrated range [m] and its 1-sigma [m]. delay_ms is
        how old the range is, history-anchored like a delayed GNSS fix.
        Each call adds one entry to the pending epoch; returns False when
        the epoch already holds INS_RANGE_MAX of them."""
        return self._c("add_range")(self._h, _d3(*anchor_ecef), float(range_m),
                                    float(stddev_m), int(delay_ms),
                                    int(anchor_id) & 0xFFFF) == 0

    def range_leverarm(self, lever_b):
        """Ranging antenna lever arm, body FRD [m], for the pending epoch."""
        self._c("set_range_leverarm")(self._h, _f3(*lever_b))

    def zupt(self, on=True):
        self._c("set_zupt")(self._h, 1 if on else 0)

    def zaru(self, on=True):
        """Zero-rotation update: we know omega == 0 (e.g. clamped)."""
        self._c("set_zaru")(self._h, 1 if on else 0)

    def auto_zupt_active(self):
        """True if the automatic ZUPT/ZARU detector currently considers
        the filter stationary (diagnostic/telemetry; see diag()'s
        n_auto_zupt for the cumulative count instead)."""
        return bool(self._c("auto_zupt_active")(self._h))

    def update(self):
        """Run the filter for this epoch, consuming everything pushed since
        the last call. Returns nothing. Read the result back with
        :meth:`solution` / :meth:`state`, which are pure readers and can be
        called at whatever rate you publish at, not once per epoch."""
        self._c("update")(self._h)

    def predict(self):
        """Time-propagation half of :meth:`update`: strapdown/covariance
        prediction only, no fusion. Must be followed by exactly one
        :meth:`correct` call before the next :meth:`predict`/:meth:`update`.
        Lets a caller sample :meth:`covariance` between prediction and
        correction to get P(k|k-1) as well as P(k|k) -- e.g. for an
        offline RTS smoother, which also needs the returned Phi.

        Returns the discrete-time state transition matrix as nested rows
        (n x n, same n and state order as :meth:`covariance`), or None if
        the covariance was not propagated this call (throttled, or the
        epoch was dropped: non-finite input, a time jump, or the filter
        is not yet initialized -- see :meth:`correct`)."""
        buf = _fcov()
        status = self._c("predict")(self._h, buf)
        if not (status & _INS_EPOCH_COV_PROPAGATED):
            return None
        n = self._n_states
        return [[buf[i + j * MAX_STATE] for j in range(n)] for i in range(n)]

    def correct(self):
        """Fusion half of :meth:`update`. A no-op if the matching
        :meth:`predict` call dropped the epoch or was never called."""
        self._c("correct")(self._h)

    # --- runtime configuration -----------------------------------------------
    def set_magnetic_model(self, lat_rad, lon_rad, year):
        """Arm the World Magnetic Model from a position: the magnetometer
        reference becomes the full WMM NED vector (declination included),
        so yaw is estimated relative to TRUE north, and the field-strength
        disturbance gate is armed. Callable at any time, e.g. on the first
        GNSS fix. ``year`` is a decimal year (e.g. 2026.5). One call is
        enough: the filter keeps the model current at its own position as
        it travels, for the AHRS heading as well."""
        self._c("set_magnetic_model_position")(
            self._h, float(lat_rad), float(lon_rad), float(year))

    # --- outputs ------------------------------------------------------------
    def is_ready(self) -> bool:
        return bool(self._c("is_ready")(self._h))

    def deadreckoning_ms(self) -> int:
        return int(self._c("deadreckoning_ms")(self._h))

    def _get3f(self, name):
        out = _f3()
        return list(out) if self._c(name)(self._h, out) else None

    def position_ecef(self):
        out = _d3()
        return list(out) if self._c("get_position_ecef")(self._h, out) else None

    def position_llh(self):
        """Position as the filter holds it: [lat_rad, lon_rad, height_m]
        above the WGS84 ellipsoid, None until the filter is initialized.
        :meth:`position_ecef` is built FROM this, so take this one when
        degrees are what you want."""
        out = _d3()
        return list(out) if self._c("get_latlonh")(self._h, out) else None

    def origin_ecef(self):
        """ECEF of the local NED frame origin (fixed at init). None until
        the filter is initialized. Lets callers express external llh
        positions in the same local frame as :meth:`position_local`."""
        out = _d3()
        return list(out) if self._c("get_origin_ecef")(self._h, out) else None

    def position_local(self):
        return self._get3f("get_position_local")

    def velocity_ned(self):
        return self._get3f("get_velocity_ned")

    def omega_b_nb(self):
        return self._get3f("get_omega_b_nb")

    def acc_n(self):
        return self._get3f("get_acc_n")

    def bias_acc(self):
        """Estimated accelerometer bias [m/s^2] or None."""
        return self._get3f("get_bias_acc")

    def bias_gyr(self):
        """Estimated gyroscope bias [rad/s] or None."""
        return self._get3f("get_bias_gyr")

    def bias_mag(self):
        """Estimated magnetometer hard-iron bias [uT] or None. Only
        estimated with ``Config(estimate_mag_bias=True)`` (18-state)."""
        return self._get3f("get_bias_mag")

    def quaternion(self):
        out = _f4()
        return list(out) if self._c("get_quaternion")(self._h, out) else None

    def rotmat_b_to_n(self):
        """R_b_to_n as a flat length-9 list, COLUMN-major (element (i,j) is
        out[i + j*3])."""
        out = _f9()
        return list(out) if self._c("get_rotmat_b_to_n")(self._h, out) else None

    def covariance(self):
        """Full error-state covariance P = U*diag(d)*U' as nested rows (n x
        n, n = 15, or 18 with ``estimate_mag_bias``), or None before the
        filter is initialized. State order: pos_ned(3) vel_ned(3) rpy(3,
        small-angle NED attitude error [rad]) acc_bias(3) gyr_bias(3)
        [mag_bias(3)]. Read directly from the filter's own UDU factors, so
        this is exact (not an approximation)."""
        buf = _fcov()
        n = self._c("get_covariance")(self._h, buf)
        if n <= 0:
            return None
        return [[buf[i + j * MAX_STATE] for j in range(n)] for i in range(n)]

    def stddev(self):
        """1-sigma standard deviations of the error state (sqrt of the
        :meth:`covariance` diagonal), grouped like :class:`State`: pos_ned
        [m], vel_ned [m/s], rpy [rad], acc_bias [m/s^2], gyr_bias [rad/s]
        and (18-state mode only) mag_bias [uT]. None before the filter is
        initialized."""
        p = self.covariance()
        if p is None:
            return None
        d = [math.sqrt(max(p[i][i], 0.0)) for i in range(len(p))]
        out = {"pos_ned": tuple(d[0:3]), "vel_ned": tuple(d[3:6]),
              "rpy": tuple(d[6:9]), "acc_bias": tuple(d[9:12]),
              "gyr_bias": tuple(d[12:15])}
        if len(d) > 15:
            out["mag_bias"] = tuple(d[15:18])
        return out

    def diag(self):
        out = _u10()
        self._c("get_diag")(self._h, out)
        d = dict(zip(_DIAG_KEYS, out))
        # Speed aiding lives outside the fixed 10-slot array (REQ-NAV-068),
        # merged in here so callers see one dict.
        counts = (ctypes.c_uint32 * 3)()
        resid = ctypes.c_float(0.0)
        self._c("get_speed_diag")(self._h, counts, ctypes.byref(resid))
        d["n_speed_seen"], d["n_speed_used"], d["n_speed_skipped"] = list(counts)
        d["last_speed_residual_mps"] = resid.value
        # Range aiding, likewise outside the array (REQ-NAV-082/085).
        rcounts = (ctypes.c_uint32 * 5)()
        rresid = ctypes.c_float(0.0)
        self._c("get_range_diag")(self._h, rcounts, ctypes.byref(rresid))
        (d["n_range_seen"], d["n_range_used"], d["n_range_rejected"],
         d["n_range_skipped"], d["n_range_pos_aiding"]) = list(rcounts)
        d["last_range_residual_m"] = rresid.value
        # Timestamp health, same reason it lives outside the array
        # (REQ-NAV-016, REQ-NAV-070).
        tcounts = (ctypes.c_uint32 * 3)()
        self._c("get_time_diag")(self._h, tcounts)
        (d["n_time_backward"], d["n_time_dropped"],
         d["n_time_restart_reset"]) = list(tcounts)
        return d

    def overconfidence(self):
        """Overconfidence / covariance-collapse watchdog (REQ-NAV-040).

        Returns a dict: ``tripped`` (True if the filter reported an
        implausibly small 1-sigma at some epoch, a likely covariance
        collapse), ``n`` (how many epochs tripped a floor) and the smallest
        per-axis 1-sigma seen: ``min_pos_m``, ``min_vel_mps``,
        ``min_att_deg`` (``inf`` until the first post-init epoch)."""
        mn = _f3()
        n = self._c("get_overconfidence")(self._h, mn)
        return {"tripped": n > 0, "n": int(n),
                "min_pos_m": mn[0], "min_vel_mps": mn[1], "min_att_deg": mn[2]}

    # --- lifecycle ----------------------------------------------------------
    def close(self):
        if getattr(self, "_h", None):
            self._c("destroy")(self._h)
            self._h = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Ins(_Base):
    """The lean 15-state ESKF (no AHRS fallback). See Navigator for the
    "best available solution" wrapper."""
    _prefix = "ins_core"

    def rpy(self):
        out = _f3()
        return list(out) if _lib.ins_core_get_rpy(self._h, out) else None

    def state(self) -> State:
        st = State(ready=self.is_ready(), dr_ms=self.deadreckoning_ms())
        _fill_nav_state(st, self, self.rpy())
        return st


def _fill_nav_state(st: State, h, rpy):
    """Populate the shared NED/attitude fields of ``st`` from handle ``h``."""
    p = h.position_local()
    if p:
        st.x_m, st.y_m, st.z_m = p
    v = h.velocity_ned()
    if v:
        st.vx_mps, st.vy_mps, st.vz_mps = v
    llh = h.position_llh()
    if llh:
        st.lat_rad, st.lon_rad, st.alt_m = llh
    q = h.quaternion()
    if q:
        st.qw, st.qx, st.qy, st.qz = q
    if rpy:
        st.roll_rad, st.pitch_rad, st.yaw_rad = rpy
    w = h.omega_b_nb()
    if w:
        st.roll_rate_rps, st.pitch_rate_rps, st.yaw_rate_rps = w
    a = h.acc_n()
    if a:
        st.ax_mps2, st.ay_mps2, st.az_mps2 = a


# WGS84 ECEF -> geodetic (Bowring), so State.lat/lon/alt need no C round-trip.
_WGS84_A = 6378137.0
_WGS84_E2 = 0.00669437999014


def rpy_to_quat(roll, pitch, yaw):
    """Hamilton quaternion [w,x,y,z] for R_b_to_n (ZYX Tait-Bryan), matching
    ins_quat_from_rpy. Used to put an attitude on the wire when only Euler
    angles are available (e.g. the AHRS fallback in ATTITUDE_ONLY mode)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


def llh_to_ecef(lat, lon, h):
    """WGS84 geodetic latitude/longitude [rad] and ellipsoidal height [m]
    to ECEF [m]. Inverse of :func:`ecef_to_llh`. The filter itself never
    needs this -- :meth:`Navigator.gnss_pos_llh` takes geodetic input --
    so it is here for callers that have to talk ECEF to something else."""
    sl, cl = math.sin(lat), math.cos(lat)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
    x = (n + h) * cl * math.cos(lon)
    y = (n + h) * cl * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + h) * sl
    return x, y, z


_lib.magnetic_field_ned_uT.restype = None
_lib.magnetic_field_ned_uT.argtypes = [ctypes.c_float, ctypes.c_float,
                                       ctypes.c_float, _f3]


def wmm_field_ned(lat_deg, lon_deg, year):
    """World Magnetic Model reference field [uT] in NED for a location and
    epoch, as a 3-tuple (north, east, down). Its magnitude is the WMM total
    field strength F, handy for comparing a magnetometer's measured |B|
    against the model. ``year`` is a decimal year (e.g. 2026.5). Thin wrapper
    over the C ``magnetic_field_ned_uT`` lookup."""
    out = _f3()
    _lib.magnetic_field_ned_uT(float(lat_deg), float(lon_deg), float(year), out)
    return tuple(out)


_lib.ins_yaw_from_baseline_heading.restype = ctypes.c_bool
_lib.ins_yaw_from_baseline_heading.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, _f3,
    ctypes.POINTER(ctypes.c_float)]


def yaw_from_baseline_heading(heading_rad, roll_rad, pitch_rad, baseline_b):
    """Vehicle yaw [rad] from the measured azimuth of a body-fixed antenna
    baseline (dual-antenna GNSS heading), or None where the C
    ``ins_yaw_from_baseline_heading`` refuses (non-finite input, zero or
    near-vertical baseline). ``baseline_b`` points, in body FRD, from the
    antenna the azimuth is measured from to the one it points to. Thin
    wrapper, so the replay harnesses share one geometry with the firmware."""
    out = ctypes.c_float()
    ok = _lib.ins_yaw_from_baseline_heading(
        float(heading_rad), float(roll_rad), float(pitch_rad),
        _f3(*baseline_b), ctypes.byref(out))
    return out.value if ok else None


def ecef_to_llh(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    b = _WGS84_A * math.sqrt(1.0 - _WGS84_E2)
    theta = math.atan2(z * _WGS84_A, p * b)
    ep2 = (_WGS84_A**2 - b**2) / (b**2)
    lat = math.atan2(z + ep2 * b * math.sin(theta)**3,
                     p - _WGS84_E2 * _WGS84_A * math.cos(theta)**3)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat)**2)
    alt = p / math.cos(lat) - n if abs(math.cos(lat)) > 1e-12 else z - b
    return lat, lon, alt
