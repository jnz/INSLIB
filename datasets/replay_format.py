"""Shared writers for the dataset-neutral replay format (REQ-VER-003).

Every converted dataset directory carries the same files, consumed by
both tools/replay.c and python/replay.py:

  config.yaml  lever arms, IMU noise model + optional IMU/magnetometer
               calibration (misalignment/fixed bias, REQ-NAV-037/039),
               aiding/init mode, optional GNSS covariance conditioning
               (scale/floor, REQ-NAV-038), warmup, regression limits (flat
               two-level YAML subset -- the C harness parses only sections
               + scalar/inline-list values). An optional `inputs:` section
               overrides the CSV filenames below per stream (imu/ref/gnss/
               mag/baro), each relative to config.yaml's directory -- unset
               keeps the conventional <stream>.csv name, so one stream can
               be swapped (e.g. gnss: gnss_f9p.csv) with the rest shared.
  imu.csv      t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2], and
               optionally imu_temp_c [degC] (see IMU_HEADER_TEMP)
  ref.csv      t_us, lat_deg, lon_deg, h_m, roll/pitch/yaw [deg], v_ned
  gnss.csv     t_us, lat_deg, lon_deg, h_m, full NED position covariance
               (nn,ne,nd,ee,ed,dd) [m^2], v_ned [m/s], full NED velocity
               covariance (nn,ne,nd,ee,ed,dd) [(m/s)^2], vel_ok.
               Zero diagonal entries mean "unknown" and are replaced by
               the config fallback stddevs at replay time.
  mag.csv      t_us, mag_frd_xyz [uT]                (optional)
  baro.csv     t_us, static pressure [Pa]            (optional)
  speed.csv    t_us, scalar ground speed [m/s]      (optional)

Converters import these helpers so the format cannot drift apart.

(c) Jan Zwiener (jan@zwiener.org)
"""

GNSS_HEADER = ("# t_us, lat_deg, lon_deg, h_m,"
               " cov_pos_ned nn,ne,nd,ee,ed,dd [m^2],"
               " vn, ve, vd [m/s],"
               " cov_vel_ned nn,ne,nd,ee,ed,dd [(m/s)^2], vel_ok\n")

IMU_HEADER = ("# t_us, gyr_frd_x [rad/s], gyr_frd_y [rad/s], gyr_frd_z [rad/s],"
              " acc_frd_x [m/s^2], acc_frd_y [m/s^2], acc_frd_z [m/s^2]\n")
# The same rows plus the IMU die temperature. A trailing column rather than
# a file of its own because it is per-SAMPLE and synchronous with the values
# it explains: a MEMS gyro's zero-rate output moves with die temperature,
# and that drift is a function of temperature rather than of time, so no
# process-noise model can stand in for it. Both harnesses read only the
# first seven fields, so a producer may write either header and every
# dataset written before this column existed stays valid unchanged.
IMU_HEADER_TEMP = ("# t_us, gyr_frd_x [rad/s], gyr_frd_y [rad/s], gyr_frd_z [rad/s],"
                   " acc_frd_x [m/s^2], acc_frd_y [m/s^2], acc_frd_z [m/s^2],"
                   " imu_temp_c [degC]\n")
REF_HEADER = ("# t_us, lat_deg, lon_deg, h_m, roll_deg, pitch_deg,"
              " yaw_deg, vn_mps, ve_mps, vd_mps\n")
MAG_HEADER = "# t_us, mag_frd_x [uT], mag_frd_y [uT], mag_frd_z [uT]\n"
BARO_HEADER = "# t_us, static pressure [Pa]\n"
# The per-sample uncertainty and the delay are config.yaml constants, not
# columns: one place describes the dataset. A producer may keep further
# columns of its own here, both harnesses read only the first two.
SPEED_HEADER = "# t_us, speed_mps\n"


def gnss_row(t_us, lat_deg, lon_deg, h_m, cov_pos6, vel_ned, cov_vel6,
             vel_ok):
    """Format one gnss.csv line. cov_pos6/cov_vel6 are the 6 unique
    elements (nn, ne, nd, ee, ed, dd) of the symmetric NED covariance;
    pass zeros for unknown entries."""
    return ("%d,%.10f,%.10f,%.4f,%s,%.4f,%.4f,%.4f,%s,%d\n" % (
        t_us, lat_deg, lon_deg, h_m,
        ",".join("%.6g" % c for c in cov_pos6),
        vel_ned[0], vel_ned[1], vel_ned[2],
        ",".join("%.6g" % c for c in cov_vel6),
        1 if vel_ok else 0))


def _fmt_float(x):
    """%.10g, but YAML-1.1-safe: PyYAML only recognizes scientific
    notation as a float when the mantissa has a decimal point
    ("1e-05" parses as a STRING, "1.0e-05" as a float)."""
    s = "%.10g" % x
    if ("e" in s or "E" in s) and "." not in s.split("e")[0].split("E")[0]:
        mant, _, exp = s.replace("E", "e").partition("e")
        s = mant + ".0e" + exp
    return s


def _fmt(v):
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt_float(x) for x in v) + "]"
    if isinstance(v, float):
        return _fmt_float(v)
    return str(v)


def write_config(path, sections, header=""):
    """Write the config.yaml subset shared by replay.c and replay.py.

    `sections` is a list of (key, value) pairs; a dict value becomes a
    section with 2-space-indented scalar keys, everything else a
    top-level scalar. Values may be str/int/float/bool or a flat list
    (emitted inline as [a, b, c]). Line comments can be embedded by
    passing a (value, comment) tuple."""
    def emit(f, key, val, indent):
        comment = ""
        if isinstance(val, tuple) and len(val) == 2 and isinstance(val[1], str):
            val, comment = val[0], "    # " + val[1]
        f.write("%s%s: %s%s\n" % (indent, key, _fmt(val), comment))

    with open(path, "w", encoding="utf-8") as f:
        if header:
            for line in header.rstrip("\n").split("\n"):
                f.write("# %s\n" % line if line else "#\n")
        for key, val in sections:
            if isinstance(val, dict):
                f.write("\n%s:\n" % key)
                for k, v in val.items():
                    emit(f, k, v, "  ")
            else:
                emit(f, key, val, "")
