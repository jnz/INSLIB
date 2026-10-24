"""Python binding of the INSLIB navigation filter (``pip install inslib``).

The compiled C library ships inside the package, this is a thin ``ctypes``
layer over it. The entry point is :class:`Navigator`: feed it your sensor
samples each epoch and read the best available solution back. Every epoch
is the same three moves:

    from INSLIB import Navigator

    nav = Navigator()
    nav.imu(t_us, dt, acc, gyr)       # 1. start the epoch with the IMU sample
    nav.gnss_pos_llh(llh, var_ned)    #    plus whatever else you have
    nav.update()                      # 2. run the filter
    sol = nav.solution()              # 3. read the result

* :class:`Navigator` runs the full suite: the INS plus the ARS/AHRS
  attitude filters and the barometric vertical channel, with automatic
  fallback (``sol.mode``: FULL, COASTING, ATTITUDE_ONLY, NONE).
* :class:`Ins` is the bare 15/18-state error-state Kalman filter alone.
* :class:`Config` holds the setup, an empty ``Config()`` works.
* :class:`Telemetry` streams the estimate to PlotJuggler and MAVLink.

Conventions: body frame FRD (x forward, y right, z down), navigation frame
NED, Hamilton quaternion ``[w, x, y, z]``, time in int64 microseconds,
angles in radians, accelerometer in m/s^2, gyro in rad/s.

Getting started: the `Python tutorial
<https://github.com/jnz/INSLIB/blob/main/tutorial/python_tutorial.md>`_.
Source, datasets and the post-processing GUI:
`github.com/jnz/INSLIB <https://github.com/jnz/INSLIB>`_.

(c) Jan Zwiener (jan@zwiener.org)
"""

from ._core import (Config, State, Ins, ecef_to_llh, llh_to_ecef,
                    rpy_to_quat, wmm_field_ned,
                    yaw_from_baseline_heading)
from .suite import Navigator, Solution
from .telemetry import Telemetry
from . import telemetry

__all__ = [
    "Config", "State", "Ins", "Navigator", "Solution", "Telemetry",
    "telemetry", "ecef_to_llh", "llh_to_ecef", "rpy_to_quat",
    "wmm_field_ned", "yaw_from_baseline_heading",
]
__version__ = "1.4.1"
