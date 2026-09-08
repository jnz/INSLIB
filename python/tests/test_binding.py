"""Smoke tests for the ins Python binding.

Runs under pytest (``pytest python/tests``) or standalone without any
framework (``python3 python/tests/test_binding.py``) -- the latter mirrors
the plain-C test style used elsewhere in the repo and needs no extra deps.

These exercise the ctypes wrapper end to end against the real libINSLIB;
they are NOT part of the C requirements/aerospace process (Python is out
of scope by design).

(c) Jan Zwiener (jan@zwiener.org)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from INSLIB import Ins, Navigator, Config, ecef_to_llh, rpy_to_quat  # noqa: E402

_G = 9.81
_LAT, _LON, _H = math.radians(48.783), math.radians(9.181), 300.0


def _level_imu_config(**kw):
    # These level-IMU configs carry no position aiding, so run ins in pure
    # dead reckoning: it then starts on the first IMU sample alone
    # (REQ-NAV-033, startup stream-coherence gate). Callers that add a fix or
    # want the limited dead-reckoning window override the flag explicitly.
    kw.setdefault("allow_unlimited_deadreckoning", True)
    return Config(lat_rad=_LAT, lon_rad=_LON, h_m=_H, auto_init=False, **kw)


def _run_level_zupt(handle, epochs=400, dt=0.01):
    """Feed a level, stationary IMU with a ZUPT each epoch."""
    t = 0
    for _ in range(epochs):
        t += int(dt * 1e6)
        handle.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
        handle.zupt(True)
        handle.update()
    return t


def test_ecef_roundtrip():
    """The package's WGS84 helper must invert llh<->ecef to sub-mm."""
    # A known ECEF for Stuttgart-ish llh, then back.
    from INSLIB._core import _WGS84_A, _WGS84_E2
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(_LAT) ** 2)
    x = (n + _H) * math.cos(_LAT) * math.cos(_LON)
    y = (n + _H) * math.cos(_LAT) * math.sin(_LON)
    z = (n * (1.0 - _WGS84_E2) + _H) * math.sin(_LAT)
    lat, lon, h = ecef_to_llh(x, y, z)
    assert abs(lat - _LAT) < 1e-10
    assert abs(lon - _LON) < 1e-10
    assert abs(h - _H) < 1e-4


def test_rpy_to_quat_identity():
    """Zero attitude -> identity quaternion; 90 deg yaw -> [c,0,0,s]."""
    q = rpy_to_quat(0.0, 0.0, 0.0)
    assert abs(q[0] - 1.0) < 1e-9 and max(abs(c) for c in q[1:]) < 1e-9
    q = rpy_to_quat(0.0, 0.0, math.pi / 2)
    assert abs(q[0] - math.cos(math.pi / 4)) < 1e-6
    assert abs(q[3] - math.sin(math.pi / 4)) < 1e-6


def test_ins_level_convergence():
    """A level, stationary run must keep roll/pitch ~0 and stay finite."""
    with Ins(_level_imu_config()) as nav:
        _run_level_zupt(nav)
        rpy = nav.rpy()
        assert rpy is not None
        assert abs(rpy[0]) < math.radians(0.5)
        assert abs(rpy[1]) < math.radians(0.5)
        assert nav.diag()["n_predict"] > 0


def test_predict_correct_equivalence():
    """update() must be exactly predict()+correct() (see ins.h's
    ins_predict_step()/ins_correct_step()): two identical filters fed the
    same epochs, one via update(), the other manually split, must end up
    bit-for-bit identical. predict() must also return a Phi matrix with
    the expected pos/vel coupling whenever it propagated the covariance."""
    dt = 0.01
    # Match the covariance-predict cadence to the IMU rate so every epoch
    # propagates the covariance and Phi's pos/vel coupling is exactly dt
    # (otherwise it accumulates to the (default, slower) throttled cadence).
    cfg = _level_imu_config(kalman_update_dt_sec=dt)
    with Ins(cfg) as nav1, Ins(cfg) as nav2:
        t = 0
        phi_seen = None
        for _ in range(400):
            t += int(dt * 1e6)
            acc, gyr = (0.0, 0.0, -_G), (0.0, 0.0, 0.0)

            nav1.imu(t, dt, acc, gyr)
            nav1.zupt(True)
            nav1.update()

            nav2.imu(t, dt, acc, gyr)
            nav2.zupt(True)
            phi = nav2.predict()
            nav2.correct()
            if phi is not None and phi_seen is None:
                phi_seen = phi

        assert phi_seen is not None
        n = len(phi_seen)
        assert n in (15, 18)
        # pos += vel*dt coupling: Phi[i][3+i] == dt for i in {N,E,D}.
        for i in range(3):
            assert abs(phi_seen[i][3 + i] - dt) < 1e-6

        assert nav1.position_local() == nav2.position_local()
        assert nav1.velocity_ned() == nav2.velocity_ned()
        assert nav1.rpy() == nav2.rpy()
        assert nav1.covariance() == nav2.covariance()


def test_ins_rpy_init_rad_passthrough():
    """Config.rpy_init_rad must reach ins (manual init with known yaw)."""
    yaw0 = math.radians(135.0)
    cfg = _level_imu_config(rpy_init_rad=(0.0, 0.0, yaw0),
                            rpy_init_stddev_rad=(math.radians(1.0),) * 3)
    with Ins(cfg) as nav:
        _run_level_zupt(nav, epochs=100)
        rpy = nav.rpy()
        assert rpy is not None
        # ZUPTs don't observe yaw: it must still be the configured value.
        assert abs(rpy[2] - yaw0) < math.radians(1.0)


def test_ins_rpy_init_stddev_yaw_override():
    """Config.rpy_init_stddev_rad[2] must let yaw start far less certain
    than roll/pitch: with a loose yaw prior, a single tight external yaw
    fix should pull yaw almost all the way to the fix. If the yaw slot
    were not wired through independently (yaw stuck using the tight
    roll/pitch value instead), the same fix would only partially move
    the estimate."""
    wrong_yaw = math.radians(10.0)
    fix_yaw = math.radians(100.0)
    cfg = _level_imu_config(rpy_init_rad=(0.0, 0.0, wrong_yaw),
                            rpy_init_stddev_rad=(math.radians(0.5),
                                                 math.radians(0.5),
                                                 math.radians(90.0)))
    with Ins(cfg) as nav:
        # The first coherent epoch starts the filter (REQ-NAV-033) and does
        # not fuse, so apply the yaw fix on the next epoch, where the loose
        # initial yaw prior is still live.
        nav.imu(10000, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
        nav.update()
        nav.imu(20000, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
        nav.yaw(fix_yaw, math.radians(1.0))
        nav.update()
        rpy = nav.rpy()
        assert rpy is not None
        err_to_fix = abs(((rpy[2] - fix_yaw + math.pi) % (2 * math.pi)) - math.pi)
        assert err_to_fix < math.radians(5.0)


def test_ins_gnss_position_pulls_in():
    """A GNSS fix offset from the init must move the ECEF estimate toward it."""
    cfg = _level_imu_config(pos_init_stddev_m=5.0)
    with Ins(cfg) as nav:
        # True antenna 3 m east of the init origin.
        from INSLIB._core import _WGS84_A, _WGS84_E2
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(_LAT) ** 2)
        # east unit vector in ECEF at (lat,lon): [-sin(lon), cos(lon), 0]
        e_hat = (-math.sin(_LON), math.cos(_LON), 0.0)
        base = ((n + _H) * math.cos(_LAT) * math.cos(_LON),
                (n + _H) * math.cos(_LAT) * math.sin(_LON),
                (n * (1.0 - _WGS84_E2) + _H) * math.sin(_LAT))
        fix = tuple(base[i] + 3.0 * e_hat[i] for i in range(3))
        t = 0
        for i in range(600):
            t += 10000
            nav.imu(t, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
            if i % 20 == 0:
                nav.gnss_pos(fix, (1.0, 1.0, 4.0))
            nav.update()
        assert nav.diag()["n_gnss_used"] > 0
        ecef = nav.position_ecef()
        assert ecef is not None
        # estimate should sit within ~1 m of the fix, not at the origin
        assert math.dist(ecef, fix) < 1.0


def test_navigator_attitude_fallback():
    """Without position aiding the suite must still provide an attitude
    (ATTITUDE_ONLY / COASTING), not go dark."""
    with Navigator(_level_imu_config()) as nav:
        _run_level_zupt(nav)
        sol = nav.solution()
        assert sol.mode in ("ATTITUDE_ONLY", "COASTING", "FULL")
        assert sol.attitude_ok
        assert abs(sol.roll) < math.radians(0.5)
        assert abs(sol.pitch) < math.radians(0.5)
        st = nav.state()               # telemetry snapshot must be finite
        assert math.isfinite(st.qw)    # synthesized from rpy in attitude-only


def test_unlimited_deadreckoning_stays_ready():
    """allow_unlimited_deadreckoning must keep ins ready without aiding."""
    cfg = _level_imu_config(allow_unlimited_deadreckoning=True)
    with Ins(cfg) as nav:
        # long IMU-only run; with the window disabled it must not degrade
        _run_level_zupt(nav, epochs=2000)
        assert nav.is_ready()


_P0 = 101325.0  # ISA sea-level pressure [Pa]


def _run_level_baro(nav, epochs, pressure_pa, t0=0, dt=0.01):
    t = t0
    for _ in range(epochs):
        t += int(dt * 1e6)
        nav.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
        nav.baro(pressure_pa)
        nav.zupt(True)
        nav.update()
    return t


def test_navigator_baro_height():
    """The baro vertical channel must anchor h=0 at the first pressure
    sample and report the best-available height through Solution.

    auto_zupt_disable is essential to the second phase: a perfectly level,
    perfectly static IMU keeps ins's auto-ZUPT/ZARU detector armed
    permanently, and REQ-SUITE-015 then feeds the vertical channel a
    zero-velocity update every epoch. That is correct behaviour -- a
    platform declared stationary has no climb rate -- but it makes
    "static IMU plus a 67 m pressure step" a self-contradictory scenario:
    the ZUPT pins the height and the barometer never gets to move it. The
    filter's ability to FOLLOW the barometer is what is under test here,
    so the stillness detector is switched off for it; the ZUPT path
    itself is covered by tests/test_baro.c:scenario_suite_zaru_drives_baro_zupt.
    """
    with Navigator(_level_imu_config(auto_zupt_disable=True)) as nav:
        _run_level_baro(nav, 300, _P0)
        sol = nav.solution()
        # anchored: first sample defines the datum -> height ~ 0
        assert sol.baro_height_m == sol.baro_height_m  # not NaN
        assert abs(sol.baro_height_m) < 0.5
        assert abs(sol.baro_vz_mps) < 0.2
        assert math.isfinite(sol.height_m)
        # ~8 hPa less is ~ +67 m ISA; the filter must follow (static IMU
        # resists a fast jump, so allow a generous band after settling).
        _run_level_baro(nav, 4000, _P0 - 800.0, t0=300 * 10000)
        sol = nav.solution()
        assert sol.baro_height_m > 20.0
        b = nav.baro_alt()
        assert b is not None and b[0] == sol.baro_height_m


def test_state_defers_to_suite_height():
    """Navigator.state() (the MAVLink/PlotJuggler telemetry snapshot) must
    always carry the suite's own arbitrated height/altitude
    (height()/height_ellipsoid(), REQ-SUITE-007/008), not ins's raw
    local/ECEF position.

    Regression: position_local()/position_ecef() only check
    is_initialized, not is_ready() or a real WGS84 anchor, so they stay
    finite from initialization onward -- here from the prescribed init
    lat/lon, with no GNSS fix ever presented to actually anchor it.
    state() used to only fall back to height()/height_ellipsoid() when
    its raw fields were NaN, so this un-anchored placeholder altitude
    leaked onto MAVLink/PlotJuggler as if it were real instead of
    height_ellipsoid() correctly reporting "no absolute height yet".
    """
    with Navigator(_level_imu_config(auto_zupt_disable=True)) as nav:
        _run_level_baro(nav, 300, _P0)
        st = nav.state()
        h = nav.height()
        assert h is not None and math.isfinite(h)
        assert math.isclose(st.z_m, -h, abs_tol=1e-6)
        # Never WGS84-anchored (no GNSS fix ever presented): the suite
        # correctly has no absolute height, and state() must not
        # substitute ins's un-anchored placeholder position for it.
        assert nav.height_ellipsoid() is None
        assert not math.isfinite(st.alt_m)


def test_bias_getters():
    """bias_acc/bias_gyr must be available on a running filter; bias_mag
    only in 18-state mode (estimate_mag_bias)."""
    with Ins(_level_imu_config()) as nav:
        _run_level_zupt(nav, epochs=100)
        assert nav.bias_acc() is not None
        assert nav.bias_gyr() is not None
        assert nav.bias_mag() is None      # 15-state: no mag bias states
    with Ins(_level_imu_config(estimate_mag_bias=True)) as nav:
        _run_level_zupt(nav, epochs=100)
        mb = nav.bias_mag()
        assert mb is not None and all(math.isfinite(v) for v in mb)


def test_wmm_and_zaru_smoke():
    """set_magnetic_model + zaru must be callable and keep the filter sane
    (yaw referenced to true north is covered by the C tests; here we only
    guard the binding plumbing)."""
    with Navigator(_level_imu_config()) as nav:
        nav.set_magnetic_model(_LAT, _LON, 2026.5)
        t = 0
        for _ in range(200):
            t += 10000
            nav.imu(t, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
            nav.mag((20.0, 1.0, 44.0), (1.0, 1.0, 1.0))
            nav.zupt(True)
            nav.zaru(True)
            nav.update()
        sol = nav.solution()
        assert sol.attitude_ok
        assert math.isfinite(sol.roll) and math.isfinite(sol.yaw)


def test_gnss_full_covariance():
    """gnss_pos/gnss_vel with a full 3x3 covariance (+ pos/vel cross
    block) must fuse like the diagonal path. A strongly anisotropic,
    correlated covariance must not break the UDU update (it is
    decorrelated internally)."""
    cfg = _level_imu_config(pos_init_stddev_m=5.0)
    with Ins(cfg) as nav:
        from INSLIB._core import _WGS84_A, _WGS84_E2
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(_LAT) ** 2)
        base = ((n + _H) * math.cos(_LAT) * math.cos(_LON),
                (n + _H) * math.cos(_LAT) * math.sin(_LON),
                (n * (1.0 - _WGS84_E2) + _H) * math.sin(_LAT))
        cov_pos = [[1.0, 0.3, 0.0],
                   [0.3, 1.0, 0.0],
                   [0.0, 0.0, 4.0]]
        cov_vel = [[0.04, 0.01, 0.0],
                   [0.01, 0.04, 0.0],
                   [0.0,  0.0,  0.09]]
        cross = [[0.01, 0.0, 0.0],
                 [0.0, 0.01, 0.0],
                 [0.0, 0.0, 0.02]]
        t = 0
        for i in range(400):
            t += 10000
            nav.imu(t, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
            if i % 20 == 0:
                nav.gnss_pos(base, cov_pos)
                nav.gnss_vel((0.0, 0.0, 0.0), cov_vel)
                nav.gnss_pos_vel_cov(cross)
            nav.update()
        assert nav.diag()["n_gnss_used"] > 0
        assert nav.diag()["n_fuse_fail"] == 0
        ecef = nav.position_ecef()
        assert ecef is not None and math.dist(ecef, base) < 1.0


def test_gnss_pos_cov_scale_height():
    """gnss_pos_cov_scale_height (REQ-NAV-041) wires through the binding and
    downweights the position height axis only: the DOWN posterior inflates
    while the NORTH posterior is essentially unchanged (unlike the isotropic
    pos scale)."""
    from INSLIB._core import _WGS84_A, _WGS84_E2

    def run(**kw):
        cfg = _level_imu_config(pos_init_stddev_m=5.0, **kw)
        with Ins(cfg) as nav:
            n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(_LAT) ** 2)
            base = ((n + _H) * math.cos(_LAT) * math.cos(_LON),
                    (n + _H) * math.cos(_LAT) * math.sin(_LON),
                    (n * (1.0 - _WGS84_E2) + _H) * math.sin(_LAT))
            t = 0
            for i in range(400):
                t += 10000
                nav.imu(t, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
                if i % 20 == 0:
                    nav.gnss_pos(base, (1.0, 1.0, 1.0))  # diagonal, unit stddev
                nav.update()
            assert nav.diag()["n_gnss_used"] > 0
            p = nav.covariance()
            return p[0][0], p[2][2]  # north, down position variance

    pnn0, pdd0 = run()                                  # no downweight
    pnnH, pddH = run(gnss_pos_cov_scale_height=5.0)     # height only
    pnnI, _ = run(gnss_pos_cov_scale=5.0)               # isotropic, for contrast

    assert pddH > pdd0 * 4.0                 # height=5 inflates the DOWN posterior
    assert abs(pnnH - pnn0) <= 1e-3 * pnn0   # ... but leaves NORTH ~unchanged
    assert pnnI > pnn0 * 2.0                 # isotropic scale DOES inflate NORTH


def test_gnss_delay_kwarg():
    """gnss_pos(..., delay_ms=) must still fuse (history-anchored)."""
    cfg = _level_imu_config(pos_init_stddev_m=5.0)
    with Ins(cfg) as nav:
        from INSLIB._core import _WGS84_A, _WGS84_E2
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(_LAT) ** 2)
        base = ((n + _H) * math.cos(_LAT) * math.cos(_LON),
                (n + _H) * math.cos(_LAT) * math.sin(_LON),
                (n * (1.0 - _WGS84_E2) + _H) * math.sin(_LAT))
        t = 0
        for i in range(400):
            t += 10000
            nav.imu(t, 0.01, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
            if i > 50 and i % 20 == 0:
                nav.gnss_pos(base, (1.0, 1.0, 4.0), delay_ms=100)
            nav.update()
        assert nav.diag()["n_gnss_used"] > 0


def test_automotive_yaw_from_gnss_course():
    """automotive_mode wires through and derives yaw from the GNSS velocity
    vector: a steady course over ground pulls the (wrong) initial heading to
    it. With the mode off the same velocity leaves yaw unobserved."""
    course = math.radians(40.0)
    speed = 10.0

    def run(automotive):
        # The manoeuvre-dependent velocity noise (REQ-NAV-073) is switched
        # off for the same reason auto_zupt is: this test feeds a stationary,
        # level IMU next to a 10 m/s GNSS velocity on purpose, so the
        # acceleration that term reads is an artefact of the contradiction
        # rather than a manoeuvre.
        cfg = _level_imu_config(automotive_mode=automotive,
                                auto_zupt_disable=True,
                                gnss_vel_noise_acc_scale_hor=-1.0,
                                gnss_vel_noise_acc_scale_ver=-1.0,
                                rpy_init_stddev_rad=(math.radians(5.0),
                                                     math.radians(5.0),
                                                     math.radians(20.0)))
        with Ins(cfg) as nav:
            t, dt = 0, 0.05
            for k in range(600):
                t += int(dt * 1e6)
                nav.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, 0.0))
                if k % 20 == 0:
                    nav.gnss_vel((speed * math.cos(course),
                                  speed * math.sin(course), 0.0),
                                 (0.09, 0.09, 0.09))
                nav.update()
            return nav.rpy()[2]

    yaw_on = run(True)
    err = abs(((yaw_on - course + math.pi) % (2 * math.pi)) - math.pi)
    assert err < math.radians(4.0)
    yaw_off = run(False)
    assert abs(yaw_off) < math.radians(3.0)  # unobserved -> stays at init 0


_TESTS = [
    test_ecef_roundtrip,
    test_rpy_to_quat_identity,
    test_ins_level_convergence,
    test_predict_correct_equivalence,
    test_ins_rpy_init_rad_passthrough,
    test_ins_rpy_init_stddev_yaw_override,
    test_ins_gnss_position_pulls_in,
    test_navigator_attitude_fallback,
    test_unlimited_deadreckoning_stays_ready,
    test_navigator_baro_height,
    test_state_defers_to_suite_height,
    test_bias_getters,
    test_wmm_and_zaru_smoke,
    test_gnss_full_covariance,
    test_gnss_pos_cov_scale_height,
    test_gnss_delay_kwarg,
    test_automotive_yaw_from_gnss_course,
]


def _main():
    fails = 0
    for t in _TESTS:
        try:
            t()
            print(f"ok    {t.__name__}")
        except Exception as e:  # noqa: BLE001
            fails += 1
            print(f"FAIL  {t.__name__}: {e!r}")
    print(f"\n{fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(_main())
