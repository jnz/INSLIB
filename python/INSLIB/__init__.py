"""ins: Python bindings and telemetry for the INSLIB navigation library.

Two levels of API:

* :class:`Navigator`: the high-level "best available solution". Push
  whatever sensors you have each epoch and read a unified :class:`Solution`
  (nav_suite mode arbitration with AHRS attitude fallback).
* :class:`Ins`: the lean 15-state ESKF alone.

Plus :class:`Telemetry` to stream the estimate to PlotJuggler (JSON) and
MAVLink.

    from INSLIB import Navigator, Config, Telemetry
    nav = Navigator(Config(auto_init=True))
    tele = Telemetry(plotjuggler=True)
    nav.imu(t_us, dt, acc, gyr); nav.gnss_pos_llh(llh, var_ned)
    nav.baro(pressure_pa); nav.mag(mag_uT, mag_var)
    nav.update()                         # run the epoch, then read it back
    sol = nav.solution(); tele.publish(nav.state())
    sol.mode, sol.yaw, sol.height_m      # best available, always

The shared library (built by ``make pylib``) is loaded from inside the
package. Conventions: body FRD, nav NED, Hamilton quaternion [w,x,y,z],
int64 microsecond time, radians.

(c) Jan Zwiener (jan@zwiener.org)
"""

from ._core import (Config, State, Ins, ecef_to_llh, llh_to_ecef,
                    rpy_to_quat, wmm_field_ned)
from .suite import Navigator, Solution
from .telemetry import Telemetry
from . import telemetry

__all__ = [
    "Config", "State", "Ins", "Navigator", "Solution", "Telemetry",
    "telemetry", "ecef_to_llh", "llh_to_ecef", "rpy_to_quat",
    "wmm_field_ned",
]
__version__ = "0.1.0"
