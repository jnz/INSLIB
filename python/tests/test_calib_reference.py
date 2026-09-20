#!/usr/bin/env python3
"""Regression tests for the two reference magnitudes a calibration is
scaled to: local gravity and the WMM field strength.

Both come out of INSLIB's own models through the shared library rather
than out of a formula repeated in Python, and both are thin ctypes
bindings, which is exactly the kind of code that fails silently. A
degrees/radians slip in the gravity call or a millimetre read as a metre
in the NAV-PVT height does not raise: it returns a plausible looking
number that lands straight on the accelerometer scale factors.

The models themselves are tested on the C side. What is pinned here is
the plumbing around them, held against values computed independently.

Runs under pytest or standalone:

    python3 python/tests/test_calib_reference.py
"""

import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inslib_mag_calib as mc          # noqa: E402
import inslib_imu_calib as calib       # noqa: E402

# Only the NAV-PVT test needs the calibration window, and only for the
# demo frame builder it happens to own. That window needs PyQt6 and
# pyqtgraph, so the import is optional the same way it is in
# test_cfg_protocol.py: the gravity tests are worth running without them.
try:
    import inslib_calib_gui as gui     # noqa: E402
except Exception:                      # noqa: BLE001
    gui = None

# WGS84 normal gravity at 48.91 deg N, 160 m above the ellipsoid, from an
# independent evaluation of Somigliana plus the free air term. The
# tolerance is what a float32 pass through the library costs, not slack.
LAT_DEG = 48.91
HEIGHT_M = 160.0
G_REF = 9.80918
G_TOL = 2e-4


def _no_library(value):
    """Both lookups degrade to None without the shared library, which is
    a documented state and not a failure. make pytest builds it first."""
    if value is None:
        print("   (shared library not built, skipped -- run 'make pylib')")
        return True
    return False


def test_local_gravity_matches_an_independent_evaluation():
    g = calib.local_gravity(LAT_DEG, HEIGHT_M)
    if _no_library(g):
        return
    assert abs(g - G_REF) < G_TOL, "%.6f vs %.6f" % (g, G_REF)


def test_local_gravity_takes_degrees_and_not_radians():
    """The library wants radians and the callers hold degrees.

    Passed straight through, 48.91 would be read as 48.91 rad, which
    wraps to some other latitude and still returns a number in the right
    range. Latitude is what separates the two: normal gravity rises
    monotonically from the equator to the pole, so the ordering below
    cannot hold for a mixed up angle."""
    g_eq = calib.local_gravity(0.0, 0.0)
    if _no_library(g_eq):
        return
    g_45 = calib.local_gravity(45.0, 0.0)
    g_pole = calib.local_gravity(90.0, 0.0)
    assert g_eq < g_45 < g_pole
    assert abs(g_eq - 9.7803) < 1e-3      # WGS84 gamma_e
    assert abs(g_pole - 9.8322) < 1e-3    # WGS84 gamma_p


def test_local_gravity_falls_with_height():
    """Free air, about -3.1e-6 m/s^2 per metre."""
    g0 = calib.local_gravity(LAT_DEG, 0.0)
    if _no_library(g0):
        return
    g1000 = calib.local_gravity(LAT_DEG, 1000.0)
    assert g1000 < g0
    assert abs((g0 - g1000) / 1000.0 - 3.086e-6) < 2e-7


def test_local_gravity_is_a_real_correction():
    """It has to be worth taking at all.

    The standard 9.80665 is right at about 45.5 degrees at sea level. If
    the difference at a European latitude were negligible, this whole
    lookup would be ceremony -- it is 2.6e-4 relative, larger than the
    scale factor stability of a good MEMS part."""
    g = calib.local_gravity(LAT_DEG, HEIGHT_M)
    if _no_library(g):
        return
    assert abs(g - calib.G_MPS2) / g > 1e-4


def test_nav_pvt_height_arrives_in_metres():
    """NAV-PVT carries the ellipsoidal height in millimetres.

    Forwarded unscaled it would be 520000 m, which puts the gravity
    lookup outside its sanity bracket and silently fills in nothing."""
    if gui is None:
        print("   (PyQt6/pyqtgraph not installed, skipped)")
        return
    payload = gui.DemoSerial._nav_pvt_payload(gui.DemoSerial())
    frame = gui.ubx_frame(gui.CLASS_NAV, gui.ID_NAV_PVT, payload)
    fix = calib.decode_nav_pvt(frame)
    if fix is None:
        print("   (pyubx2 not installed, skipped)")
        return
    assert abs(fix.height_m - gui.DemoSerial.HEIGHT_M) < 1e-3
    assert calib.local_gravity(fix.lat_deg, fix.height_m) is not None


def test_wmm_reference_returns_the_local_field():
    """The magnetometer's half of the same idea, over Munich."""
    ref = mc.wmm_reference(48.137, 11.575, 2025.5)
    if _no_library(ref):
        return
    field_ut, decl_deg, incl_deg = ref
    assert 45.0 < field_ut < 52.0
    assert 0.0 < decl_deg < 8.0
    assert 60.0 < incl_deg < 68.0


def _main():
    fails = []
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok   %s" % name)
            except Exception as e:                  # noqa: BLE001
                fails.append(name)
                print("FAIL %s: %s" % (name, e))
    if fails:
        print("failed: %s" % ", ".join(fails))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
