"""Smoke test of an INSTALLED INSLIB wheel (cibuildwheel's test-command).

Deliberately no sys.path tweak: it must import the installed package, not
the checkout, so it proves the wheel carries a loadable libINSLIB for its
platform. Not a pytest module (no test_ prefix), the full suite runs
against the checkout in `make pytest`.

(c) Jan Zwiener (jan@zwiener.org)
"""

import math
import os
import sys

import INSLIB
from INSLIB import Config, Navigator

_G = 9.81


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(os.path.abspath(INSLIB.__file__))
    checkout = os.path.join(os.path.dirname(here), "INSLIB")

    def norm(p):
        return os.path.normcase(os.path.realpath(p))

    # A plain comparison, commonpath() raises for paths on different drives.
    if norm(pkg) == norm(checkout):
        print("FAIL: imported the checkout %s, not the wheel" % pkg)
        return 1

    cfg = Config(lat_rad=math.radians(48.783), lon_rad=math.radians(9.181),
                 h_m=300.0, auto_init=False,
                 allow_unlimited_deadreckoning=True)
    nav = Navigator(cfg)
    t, dt = 0, 0.01
    for _ in range(200):
        t += int(dt * 1e6)
        # Tiny alternating rate: a constant one reads as a frozen sensor.
        w = 1e-5 if (t // 10000) % 2 else -1e-5
        nav.imu(t, dt, (0.0, 0.0, -_G), (0.0, 0.0, w))
        nav.update()
    sol = nav.solution()
    if not (math.isfinite(sol.roll) and abs(sol.roll) < math.radians(1.0)):
        print("FAIL: implausible roll %r" % (sol.roll,))
        return 1
    print("OK: INSLIB %s from %s, mode %s" % (INSLIB.__version__, pkg, sol.mode))
    return 0


if __name__ == "__main__":
    sys.exit(main())
