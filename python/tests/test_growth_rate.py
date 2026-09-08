"""Verifies replay.py's process-noise growth-rate closed forms
(process_noise_growth_rate/baro_alt_growth_rate/ahrs_growth_rate) against
the real filter's own reported covariance growth under free (unaided)
coasting.

Covariance PREDICTION in a Kalman filter is deterministic (it depends only
on Phi/Q, not on the actual measurement values), so a single noiseless run
of Ins.imu()+update() -- no GNSS/baro/mag/ZUPT at all -- reproduces the
exact same Riccati-style growth the closed forms model. No Monte Carlo /
ensemble averaging is needed; this compares one arithmetic to another.

Runs under pytest (``pytest python/tests``) or standalone
(``python3 python/tests/test_growth_rate.py``), same style as
test_binding.py.

(c) Jan Zwiener (jan@zwiener.org)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from INSLIB import Ins, Navigator, Config  # noqa: E402
import replay  # noqa: E402

_G = replay.STANDARD_GRAVITY_MPS2
_LAT, _LON, _H = math.radians(48.783), math.radians(9.181), 300.0
_DT = 0.01
_T_SEC = 60.0
GROWTH_HORIZON = replay.GROWTH_HORIZON_SEC

# A representative, non-tiny noise model (same order of magnitude as a real
# MEMS IMU dataset) so the growth over _T_SEC sits well above any initial-
# condition/discretization noise floor. pos/vel/rpy_pred_stddev_*_sqrts are
# set explicitly (matching Config's own dataclass defaults, which is what
# _free_coast's Config(...) call below actually sends the real filter,
# since it doesn't override them either) -- process_noise_growth_rate()
# deliberately returns None rather than guess these when left at 0, so the
# comparison here needs them spelled out, same as any real config.yaml
# would.
_NOISE = {
    "acc_psd": 3.846815369e-06,       # (m/s^2)^2/Hz
    "gyr_psd": 3.384637998e-11,       # (rad/s)^2/Hz
    "acc_bias_rw": 3.16227766e-04,    # m/s^2/sqrt(s)
    "gyr_bias_rw": 1.414213562e-06,   # rad/s/sqrt(s)
    "pos_pred_stddev_m_sqrts": 0.01,          # m/sqrt(s), Config's default
    "vel_pred_stddev_mps_sqrts": 0.05,        # m/s/sqrt(s), Config's default
    "rpy_pred_stddev_rad_sqrts": math.radians(0.01),  # rad/sqrt(s), ditto
}

# Explicit baro_alt/ahrs noise model for the Navigator (suite) tests below --
# arbitrary but distinguishable values (deliberately NOT matching whatever
# baro_alt.c's/ahrs.c's own current C-side defaults happen to be, so a test
# that passed only because these coincide with the default wouldn't hide a
# broken setter). ahrs's values are 10x the plausible-dataset size used
# elsewhere in this file: yaw's delta-variance signal over GROWTH_HORIZON is
# otherwise small enough to sit near the UDU/float32 discretization noise
# floor and inflate the comparison's apparent error well past a real filter/
# formula mismatch (confirmed by cross-checking a wider T1/T2 gap, which
# recovers agreement at any signal size -- this is a precision-floor
# artifact, not evidence of anything wrong).
_BARO_CFG = {"acc_noise_mps2_sqrthz": 0.3, "acc_bias_rw": 5e-4}
_AHRS_CFG = {"gyr_noise_psd": 3e-3, "gyr_bias_rw": 3e-5}


def _free_coast(noise, T_sec=_T_SEC, dt=_DT):
    """Run a level, non-rotating Ins with ~zero initial uncertainty through
    T_sec of pure IMU dead reckoning, return its final stddev() -- the
    ground truth the closed forms are checked against.

    auto_zupt_disable=True is essential: a perfectly level, zero-rotation
    feed would otherwise trip ins's own stillness detector (REQ-NAV, see
    ins_auto_zupt_detect) and silently apply a zero-velocity update, capping
    the very growth this test means to observe. pos/vel/rpy_pred_stddev_*
    are intentionally NOT passed here -- Config's own dataclass defaults
    (0.01/0.05/0.01deg) already match noise's explicit values above, so
    this stays the same real-filter behavior either way."""
    cfg = Config(
        lat_rad=_LAT, lon_rad=_LON, h_m=_H, auto_init=False,
        rpy_init_rad=(0.0, 0.0, 0.0),
        pos_init_stddev_m=1e-6, vel_init_stddev_mps=1e-6,
        rpy_init_stddev_rad=(1e-6, 1e-6, 1e-6),
        acc_bias_init_stddev_mps2=1e-9, gyr_bias_init_stddev_rps=1e-12,
        acc_bias_pred_stddev_mps2_sqrts=noise["acc_bias_rw"],
        gyr_bias_pred_stddev_rps_sqrts=noise["gyr_bias_rw"],
        allow_unlimited_deadreckoning=True,
        auto_zupt_disable=True,
    )
    acc_var = (noise["acc_psd"],) * 3
    gyr_var = (noise["gyr_psd"],) * 3
    n_steps = int(round(T_sec / dt))
    with Ins(cfg) as nav:
        t = 0
        for _ in range(n_steps):
            t += int(dt * 1e6)
            nav.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, 0.0), acc_var, gyr_var)
            nav.update()
        sd = nav.stddev()
        assert sd is not None
        return sd


def _suite_free_coast_samples(baro_cfg, ahrs_cfg, times_sec, dt=_DT):
    """Run the full nav_suite (Navigator) through free coasting, sampling
    AHRS's own yaw/gyro-bias stddev and baro_alt's own stddev at each
    requested elapsed time -- verifies baro_alt_growth_rate()/
    ahrs_growth_rate() (and the set_baro_acc_noise()/
    set_baro_acc_bias_drift()/set_ahrs_gyr_noise()/set_ahrs_gyr_bias_rw()
    API they depend on) against the real filter, the same way
    process_noise_growth_rate() is checked in _free_coast() above.

    Roll/pitch are continuously corrected by ARS's/AHRS's own accelerometer
    leveling fusion every epoch under a constant gravity vector (that's the
    point of an AHRS -- not a pure gyro integrator), so they don't grow
    freely and can't stand in for ahrs_growth_rate()'s uncorrected model.
    Yaw is the axis that DOES grow uncorrected here: ARS has no yaw state
    at all (5-state, "yaw free" per nav_suite.h), and AHRS's yaw gets fed
    exactly ONE magnetometer sample (bootstrap only, at the first epoch)
    and none afterward, so it free-runs on gyro alone from then on -- the
    same "worst-case, uncorrected" axis process_noise_growth_rate() targets
    for ins (see test_growth_rate_matches_filter_horizontal).

    baro_alt only starts predicting once bootstrapped by a pressure sample
    (nav_suite.h) -- one baro() call on the very first epoch does that,
    then none afterward, so it free-coasts just like the IMU-only path.
    Returns {t: (rpy_stddev_ahrs, gyr_bias_stddev_ahrs, baro_stddev)}."""
    cfg = Config(
        lat_rad=_LAT, lon_rad=_LON, h_m=_H, auto_init=False,
        rpy_init_rad=(0.0, 0.0, 0.0),
        pos_init_stddev_m=1e-6, vel_init_stddev_mps=1e-6,
        rpy_init_stddev_rad=(1e-6, 1e-6, 1e-6),
        acc_bias_init_stddev_mps2=1e-9, gyr_bias_init_stddev_rps=1e-12,
        magnetic_n=(20.0, 0.0, 40.0),  # plausible NED field, declination 0
        allow_unlimited_deadreckoning=True,
        auto_zupt_disable=True,
    )
    acc_var = (_NOISE["acc_psd"],) * 3
    gyr_var = (_NOISE["gyr_psd"],) * 3
    mag_var = (1.0,) * 3
    want = sorted(times_sec)
    out = {}
    nav = Navigator(cfg)
    try:
        nav.set_baro_acc_bias_drift(baro_cfg["acc_bias_rw"])
        nav.set_baro_acc_noise(baro_cfg["acc_noise_mps2_sqrthz"])
        nav.set_ahrs_gyr_noise(ahrs_cfg["gyr_noise_psd"])
        nav.set_ahrs_gyr_bias_rw(ahrs_cfg["gyr_bias_rw"])
        # ahrs's own initial gyro-bias uncertainty (1/1/5 deg/s xy/z by
        # default) is an INITIAL CONDITION, not process noise, but it
        # propagates into yaw through the attitude/bias Phi coupling and
        # at the default size dominates any gyr_noise_psd/gyr_bias_rw
        # signal for tens of seconds -- pin it negligible so this test
        # isolates the process-noise growth ahrs_growth_rate() models
        # (matches how _free_coast() above pins ins's OWN init stddevs
        # tiny for the same reason).
        nav.set_ahrs_gyr_bias_init_stddev(1e-9)
        # Same reasoning for baro_alt's own initial accel-bias uncertainty
        # (0.1 m/s^2 default): it propagates into height/velocity via the
        # bias/height/velocity Phi coupling, growing ~T^4/T^2, and at the
        # default size can trip the precision-restart watchdog well before
        # the process-noise signal this test wants is even measurable.
        nav.set_baro_acc_bias_init_stddev(1e-9)
        # Same reasoning for baro_alt's own initial height/velocity
        # uncertainty (1.0 m / 0.5 m/s defaults): they're a separate,
        # non-configurable-until-now baseline that would otherwise
        # dominate this test's signal the same way the accel-bias one did.
        nav.set_baro_h_init_stddev(1e-9)
        nav.set_baro_v_init_stddev(1e-9)
        t = 0
        wi = 0
        n_steps = int(round(max(want) / dt))
        # baro_alt doesn't bootstrap off a single sample: nav_suite.c
        # averages pressure over a NAV_SUITE_BARO_BOOT_SEC (0.3 s) window
        # and needs >=2 samples in it (REQ-SUITE-006, screens a single
        # glitched startup reading) -- feed baro for the first 0.5 s (with
        # margin), then stop so it free-coasts afterward like everything
        # else here.
        baro_boot_steps = int(round(0.5 / dt))
        for i in range(1, n_steps + 1):
            t += int(dt * 1e6)
            nav.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, 0.0), acc_var, gyr_var)
            if i <= baro_boot_steps:
                nav.baro(101325.0, 1.0)
            if i == 1:
                nav.mag((20.0, 0.0, 40.0), mag_var)  # bootstrap yaw only
            nav.update()
            tsec = i * dt
            while wi < len(want) and tsec >= want[wi] - 1e-9:
                out[want[wi]] = (nav.rpy_stddev_ahrs(),
                                 nav.gyr_bias_stddev_ahrs(), nav.baro_stddev())
                wi += 1
    finally:
        nav.close()
    assert len(out) == len(want)
    return out


def test_growth_rate_matches_filter_horizontal():
    """The closed-form pos_mps/vel_mps2/att_radps must land within a
    generous (discretization/float32-tolerant) band of the real filter's
    own reported 1-sigma growth over the same horizon, on the horizontal
    (N) channel the closed form targets."""
    sd = _free_coast(_NOISE)
    g = replay.process_noise_growth_rate(_NOISE, horizon_sec=_T_SEC)

    pos_n_actual = sd["pos_ned"][0]
    vel_n_actual = sd["vel_ned"][0]
    att_actual = sd["rpy"][1]  # pitch: what couples into velN

    pos_n_pred = g["pos_mps"] * _T_SEC
    vel_n_pred = g["vel_mps2"] * _T_SEC
    att_pred = g["att_radps"] * _T_SEC

    for label, actual, pred in (("pos", pos_n_actual, pos_n_pred),
                                ("vel", vel_n_actual, vel_n_pred),
                                ("att", att_actual, att_pred)):
        ratio = actual / pred
        assert 0.95 < ratio < 1.05, (
            f"{label}: filter reports {actual:.6g}, closed form predicts "
            f"{pred:.6g} (ratio {ratio:.3f}) -- formula/filter disagree "
            f"beyond float32/discretization tolerance")


def test_growth_rate_without_tilt_coupling_understates_it():
    """Regression guard for the bug this test suite was written to catch:
    a naive accel-noise-only model (_rate_state_growth_var/
    _position_state_growth_var alone, no g^2 * attitude term) must
    UNDERSTATE the real filter's velocity/position growth by a large
    margin here -- if it didn't, the tilt-coupling term would be
    pointless. Confirms the coupling is not a rounding-error-sized
    correction but the dominant effect it's documented to be."""
    sd = _free_coast(_NOISE)
    pos_n_actual = sd["pos_ned"][0]
    vel_n_actual = sd["vel_ned"][0]

    q1_acc, q2_acc = _NOISE["acc_psd"], _NOISE["acc_bias_rw"] ** 2
    naive_pos = math.sqrt(
        replay._position_state_growth_var(q1_acc, q2_acc, _T_SEC))
    naive_vel = math.sqrt(
        replay._rate_state_growth_var(q1_acc, q2_acc, _T_SEC))

    assert naive_pos < 0.5 * pos_n_actual
    assert naive_vel < 0.5 * vel_n_actual


def test_growth_rate_vertical_has_no_tilt_coupling():
    """Sanity check on the derivation: under level flight the down channel
    gets no g^2 attitude term (fnN=fnE=0 in ins_compute_Phi), so the
    filter's own vertical pos/vel 1-sigma must stay measurably smaller than
    the horizontal (N/E) ones, which do carry the coupling. Not a huge gap
    here: ins.c's own extra process-noise margin (INS_VEL/RPY_PRED_STDDEV_*,
    identical on all 3 axes) dominates this particular noise config, so the
    tilt-only difference is a modest fraction, not the whole signal."""
    sd = _free_coast(_NOISE)
    assert sd["pos_ned"][2] < 0.9 * sd["pos_ned"][0]
    assert sd["vel_ned"][2] < 0.9 * sd["vel_ned"][0]
    # N and E are statistically identical here (isotropic noise, symmetric
    # roll/pitch coupling) -- and covariance PREDICTION is deterministic
    # (no injected measurement noise), so they should match almost exactly.
    assert abs(sd["pos_ned"][0] - sd["pos_ned"][1]) < 0.02 * sd["pos_ned"][0]


def _check_unset_message(lines, *keys):
    """A *_growth_rate_lines() helper fed an unset (0) config must emit
    exactly one line that says the number is unavailable AND names the
    config.yaml keys the user has to set. Asserting on those parts rather
    than the full sentence keeps harmless rewording out of the test while
    still failing if the section silently disappears, stops being
    actionable or starts pointing at the wrong keys."""
    assert len(lines) == 1, f"expected a single line, got {lines}"
    line = lines[0]
    assert "N/A" in line, f"missing unavailable marker: {line}"
    assert "config.yaml" in line, f"missing config.yaml pointer: {line}"
    for key in keys:
        assert key in line, f"missing {key}: {line}"


def test_growth_rate_none_when_margin_unset():
    """process_noise_growth_rate() should not guess ins.c's built-in
    pos/vel/rpy_pred_stddev_*_sqrts defaults and return None.
    growth_rate_lines() must turn that None into an actionable
    message rather than crashing or silently omitting the section."""
    for key in ("pos_pred_stddev_m_sqrts", "vel_pred_stddev_mps_sqrts",
                "rpy_pred_stddev_rad_sqrts"):
        noise = dict(_NOISE)
        noise[key] = 0.0
        assert replay.process_noise_growth_rate(noise) is None, (
            f"expected None with {key}=0.0")
    lines = replay.growth_rate_lines({**_NOISE, "pos_pred_stddev_m_sqrts": 0.0})
    _check_unset_message(lines, "pos_pred_stddev_m_sqrts",
                         "vel_pred_stddev_mps_sqrts",
                         "rpy_pred_stddev_rad_sqrts")


def test_baro_alt_and_ahrs_growth_rate_match_filter():
    """baro_alt_growth_rate()/ahrs_growth_rate() checked against the REAL
    nav_suite's own reported covariance growth (AHRS's yaw/gyro-bias
    stddev, baro_alt's own stddev), via the set_baro_acc_noise()/
    set_baro_acc_bias_drift()/set_ahrs_gyr_noise()/set_ahrs_gyr_bias_rw()
    API -- this is the actual thing under test: does the new setter API
    correctly wire config.yaml's baro:/ahrs: values into the filter, and
    does the closed form correctly predict what comes out.

    Uses the DELTA between two sample times (T1, T2) rather than an
    absolute sigma: Var_measured(T) = Var_init + Var_closedform(T) for any
    T (Var_init is whatever ars_cfg/ahrs_cfg's own initial-uncertainty
    default happens to be, not independently pinned down here), so
    Var_measured(T2) - Var_measured(T1) cancels the unknown Var_init term
    exactly and isolates the part this test actually verifies."""
    T1, T2 = GROWTH_HORIZON, 2 * GROWTH_HORIZON
    samples = _suite_free_coast_samples(_BARO_CFG, _AHRS_CFG, (T1, T2))
    rpy1, gb1, baro1 = samples[T1]
    rpy2, gb2, baro2 = samples[T2]

    # Roll/pitch are continuously corrected by AHRS's own accelerometer
    # leveling fusion (that's the point of an AHRS/ARS -- not a pure gyro
    # integrator), so they don't grow freely under a constant, noiseless
    # gravity vector; yaw has no such correction after its one-time mag
    # bootstrap (see _suite_free_coast_samples) and behaves like the
    # uncorrected integrator ahrs_growth_rate() models -- same reasoning as
    # process_noise_growth_rate() picking the worst-case (uncorrected)
    # channel for ins.
    g_ahrs1 = replay.ahrs_growth_rate(_AHRS_CFG, horizon_sec=T1)
    g_ahrs2 = replay.ahrs_growth_rate(_AHRS_CFG, horizon_sec=T2)
    att_pred_delta_var = ((g_ahrs2["att_radps"] * T2) ** 2
                          - (g_ahrs1["att_radps"] * T1) ** 2)
    att_actual_delta_var = rpy2[2] ** 2 - rpy1[2] ** 2  # yaw
    ratio = att_actual_delta_var / att_pred_delta_var
    assert 0.9 < ratio < 1.1, (
        f"ahrs att: filter delta-var {att_actual_delta_var:.4g}, predicted "
        f"{att_pred_delta_var:.4g} (ratio {ratio:.3f})")

    # AHRS's own gyro-bias state is a plain n=1 random walk driven by
    # gyr_bias_rw alone (no attitude coupling into it) -- check it
    # independently of the att check above.
    q2_gyr = _AHRS_CFG["gyr_bias_rw"] ** 2
    bias_pred_delta_var = (replay._integrated_white_noise_var(q2_gyr, T2, 1)
                           - replay._integrated_white_noise_var(q2_gyr, T1, 1))
    bias_actual_delta_var = gb2[0] ** 2 - gb1[0] ** 2
    ratio = bias_actual_delta_var / bias_pred_delta_var
    assert 0.9 < ratio < 1.1, (
        f"ahrs gyr bias: filter delta-var {bias_actual_delta_var:.4g}, "
        f"predicted {bias_pred_delta_var:.4g} (ratio {ratio:.3f})")

    g_baro1 = replay.baro_alt_growth_rate(_BARO_CFG, horizon_sec=T1)
    g_baro2 = replay.baro_alt_growth_rate(_BARO_CFG, horizon_sec=T2)
    for label, idx, g1key, g2key in (("height", 0, "h_mps", "h_mps"),
                                     ("vel", 1, "v_mps2", "v_mps2")):
        pred_delta_var = ((g_baro2[g2key] * T2) ** 2
                          - (g_baro1[g1key] * T1) ** 2)
        actual_delta_var = baro2[idx] ** 2 - baro1[idx] ** 2
        ratio = actual_delta_var / pred_delta_var
        assert 0.9 < ratio < 1.1, (
            f"baro_alt {label}: filter delta-var {actual_delta_var:.4g}, "
            f"predicted {pred_delta_var:.4g} (ratio {ratio:.3f})")


def test_baro_alt_growth_rate_matches_closed_form():
    """baro_alt has no attitude coupling (a plain double/triple integrator,
    see baro_alt_predict's Phi) -- self-consistency check of the shared
    primitives against an explicit config (no filter round-trip needed
    here, unlike the end-to-end check above -- this just locks in that the
    formula matches its own building blocks as the file evolves)."""
    g = replay.baro_alt_growth_rate(_BARO_CFG, horizon_sec=_T_SEC)
    q1 = _BARO_CFG["acc_noise_mps2_sqrthz"] ** 2
    q2 = _BARO_CFG["acc_bias_rw"] ** 2
    assert math.isclose(
        g["h_mps"] * _T_SEC,
        math.sqrt(replay._position_state_growth_var(q1, q2, _T_SEC)),
        rel_tol=1e-9)
    assert math.isclose(
        g["v_mps2"] * _T_SEC,
        math.sqrt(replay._rate_state_growth_var(q1, q2, _T_SEC)),
        rel_tol=1e-9)


def test_baro_alt_and_ahrs_growth_rate_none_when_unset():
    """Same "refuse to guess" contract as process_noise_growth_rate() --
    baro_alt_growth_rate()/ahrs_growth_rate() must return None (and their
    _lines() helpers an actionable message) when left at 0, not a
    hardcoded copy of baro_alt.c's/ahrs.c's own default."""
    for key in ("acc_noise_mps2_sqrthz", "acc_bias_rw"):
        cfg = dict(_BARO_CFG)
        cfg[key] = 0.0
        assert replay.baro_alt_growth_rate(cfg) is None
    for key in ("gyr_noise_psd", "gyr_bias_rw"):
        cfg = dict(_AHRS_CFG)
        cfg[key] = 0.0
        assert replay.ahrs_growth_rate(cfg) is None

    baro_lines = replay.baro_growth_rate_lines(
        {**_BARO_CFG, "acc_noise_mps2_sqrthz": 0.0})
    _check_unset_message(baro_lines, "acc_noise_mps2_sqrthz", "acc_bias_rw")
    ahrs_lines = replay.ahrs_growth_rate_lines(
        {**_AHRS_CFG, "gyr_bias_rw": 0.0})
    _check_unset_message(ahrs_lines, "gyr_noise_psd", "gyr_bias_rw")


def _run(fn):
    try:
        fn()
        print(f"ok    {fn.__name__}")
        return True
    except AssertionError as e:
        print(f"FAIL  {fn.__name__}: {e}")
        return False


if __name__ == "__main__":
    tests = [test_growth_rate_matches_filter_horizontal,
             test_growth_rate_without_tilt_coupling_understates_it,
             test_growth_rate_vertical_has_no_tilt_coupling,
             test_growth_rate_none_when_margin_unset,
             test_baro_alt_and_ahrs_growth_rate_match_filter,
             test_baro_alt_growth_rate_matches_closed_form,
             test_baro_alt_and_ahrs_growth_rate_none_when_unset]
    ok = all([_run(t) for t in tests])
    print("\n0 failures" if ok else "\nFAILURES")
    sys.exit(0 if ok else 1)
