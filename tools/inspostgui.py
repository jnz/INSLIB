#!/usr/bin/env python3
"""inspostgui -- INSLIB post-processing GUI.

Interactive Qt front end for tools/replay.py: open a config YAML of a
converted dataset (the "project file": it sits in a directory with the
dataset-neutral CSVs, contract: datasets/replay_format.py, and names them
through its optional inputs: section, so one directory can carry e.g. a
config.yaml next to a config_experimental.yaml), inspect/edit/create it,
replay it through the ins filter (in-process, on the replay core
replay.py runs on, tools/replay_core.py, so every input stream, the feed
order, the scoring and the summary are the same) and evaluate the
result:

* live 3D trajectory view with a speed-colored trail, ground-truth
  overlay, GNSS fix dots, the previous run of the same dataset as a grey
  ghost trail (all switchable) and an attitude-driven vehicle model /
  error ellipsoid (visualization lifted from znav3d),
* live attitude / position / altitude / acceleration panels,
* post-run error plots (estimate vs. ground truth with the filter's own
  1-sigma band), a North-East map view, bias convergence and outlier
  counters,
* a Map tab: the same track over an optional OpenStreetMap background
  (tiles cached on disk and shared with inslib_gui.py's Track tab),
* the same accuracy / data-quality summary replay.py prints,
* a simulated GNSS outage (toolbar field, start:duration windows in
  seconds from the first IMU sample), the same cut as replay.py
  --gnss-outage,
* multi-page PDF export via ins_plots and Google Earth KML export via
  ins_kml,
* a dark and a light colour theme (toolbar, or --theme), same palettes
  as inslib_gui.py, see ins_gui_theme.py.

The config editor loads the dataset's config.yaml into a form (the
replay.py/replay.c schema minus the score.lim_* regression gates, which
are CI-only knobs for `make datasets`/`make simulated` and not
meaningful for interactive GUI use), can create a config for a dataset
directory that has none yet, and preserves keys it does not know about
(e.g. the foreign origin:/crazyflie: sections and the score.lim_* gates
themselves) when saving. NOTE: YAML comments are lost on save -- prefer
"Save As" for generated configs.

Usage:
    make pylib
    python3 tools/inspostgui.py
    python3 tools/inspostgui.py datasets/simulated/profile_1_car
    python3 tools/inspostgui.py datasets/fog/config.yaml
    python3 tools/inspostgui.py --theme light
    python3 tools/inspostgui.py --batch datasets/simulated/profile_1_car

--batch replays the dataset headless (no window) and prints the summary,
it exists mainly as a smoke test of the replay worker. --gnss-outage
START:DURATION (repeatable) applies the outage cut there.

Dependencies (pip install -r python/requirements.txt):
PyQt6, pyqtgraph, PyOpenGL, numpy, PyYAML. Optional: matplotlib (PDF
export), simplekml (KML export), numpy-stl (3D vehicle model, a simple
box is drawn without it).
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import sys
import textwrap
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "python"))
import replay  # noqa: E402  (loaders, DEFAULTS, build_config, Stat, ...)
import replay_core  # noqa: E402  (the replay loop shared with replay.py)
import ins_rts  # noqa: E402  (the offline RTS smoother)
import ins_solution  # noqa: E402  (solution files: CSV, RTKLIB .pos)
from INSLIB import ecef_to_llh  # noqa: E402
from INSLIB._core import rpy_to_quat  # noqa: E402

US_PER_SEC = replay.US_PER_SEC

# ============================================================================
# Config schema (mirrors replay.py's DEFAULTS / tools/replay.c)
# ============================================================================

BOOL, FLOAT, INT, STR, CHOICE, VEC = (
    "bool", "float", "int", "str", "choice", "vec")

TOOLTIP_WIDTH = 64  # characters per tooltip line

# (path, label, kind, extra, tooltip), extra: CHOICE -> options, VEC -> length
CONFIG_SECTIONS = [
    ("Inputs (optional CSV overrides)", [
        (("inputs", "imu"), "IMU CSV", STR, None,
         "File name relative to the config's directory. Blank → imu.csv"),
        (("inputs", "ref"), "Reference CSV", STR, None,
         "blank → ref.csv. Optional: without it nothing is scored"),
        (("inputs", "gnss"), "GNSS CSV", STR, None,
         "blank → gnss.csv, e.g. gnss_f9p.csv to A/B a receiver"),
        (("inputs", "mag"), "Magnetometer CSV", STR, None,
         "blank → mag.csv"),
        (("inputs", "baro"), "Barometer CSV", STR, None,
         "blank → baro.csv"),
        (("inputs", "heading"), "GNSS heading CSV", STR, None,
         "blank → heading.csv"),
        (("inputs", "speed"), "Speed CSV", STR, None,
         "blank → speed.csv"),
        (("inputs", "ranges"), "Ranges CSV", STR, None,
         "blank → ranges.csv"),
    ]),
    ("General", [
        (("name",), "Name", STR, None,
         "Dataset label used in plots and the summary"),
        (("aiding",), "Aiding", CHOICE, ("gnss", "ref", "none"),
         "gnss: real fixes from gnss.csv, ref: fix synthesized from the "
         "ground truth, none: no absolute position aiding (ARS/baro only)"),
        (("init",), "Init", CHOICE, ("auto", "ref"),
         "auto: ins auto-initialization, ref: known initial state from "
         "the first epoch of the reference file"),
        (("automotive_mode",), "Automotive mode", BOOL, None,
         "Derives yaw from the GNSS course over ground. Only for road "
         "vehicles, not for aircraft."),
        (("automotive_min_speed_mps",), "Automotive min speed [m/s]", FLOAT,
         None, "Minimum horizontal ground speed for the yaw-from-course "
         "fusion. Below it the course over ground is noise-dominated "
         "(parking, reversing) and not used. 0 → default"),
        (("automotive_min_yaw_stddev_deg",), "Automotive min yaw stddev "
         "[°]", FLOAT, None,
         "Lower limit of the yaw uncertainty assigned to the course "
         "measurement. Raise it to cap how much the course is trusted, "
         "regardless of speed. 0 → default"),
        (("automotive_lateral_constraint",),
         "Lateral velocity constraint",
         BOOL, None,
         "Tells the filter that the vehicle does not slide sideways. This "
         "holds roll and yaw while GNSS is out. Needs automotive mode. Only "
         "the lateral direction is used, the vertical one would pull the "
         "mounting pitch into the pitch estimate."),
        (("automotive_lateral_stddev_mps",),
         "Lateral constraint stddev [m/s]",
         FLOAT, None,
         "How strictly \"no sideways motion\" is enforced. Choose it from the "
         "vehicle's lateral mounting offset, not from the scatter of the "
         "residual. 0 → default"),
        (("automotive_lateral_max_yaw_rate_deg",),
         "Lateral constraint max yaw rate [°/s]",
         FLOAT, None,
         "The constraint is skipped above this yaw rate, where the vehicle "
         "starts to slide. 0 → default"),
        (("automotive_lateral_after_sec",),
         "Lateral constraint after [s]",
         FLOAT, None,
         "Wait this long without a GNSS fusion before the constraint starts. "
         "0 → default, negative → no delay. Next to live GNSS velocity it "
         "adds little and can feed a mounting error into the state."),
        (("chi2_disable",), "Disable chi2 downweighting", BOOL, None,
         "Diagnostics only: turns the outlier downweighting off."),
        (("chi2_reject_alpha",), "Chi2 reject alpha", FLOAT, None,
         "Outlier gate for all measurements (GNSS, magnetometer, local "
         "position, yaw). Larger → stricter, smaller → more tolerant. 0 → "
         "default"),
        (("gyro_bias_window_sec",), "Gyro-bias window [s]", FLOAT, None,
         "Average the gyro over the first seconds of the recording and start "
         "the filters from that as gyro bias, with a tighter initial "
         "uncertainty. Only if the platform stands still for the whole "
         "window, any motion in it ends up in the bias. ZARU stays active. "
         "Replay only, the library has no such option. 0 → off"),
        (("auto_init_window_sec",),
         "Auto-init leveling window [s]",
         FLOAT, None,
         "Window the filter levels itself over at the start. Raise it for a "
         "low IMU rate so it holds enough samples. 0 → default"),
        (("baro_height_disable",), "Never pick barometric height", BOOL, None,
         "1: take the height from the GNSS position instead of the barometer "
         "at the start. For position sources that are vertically better than "
         "a barometer, e.g. a Lighthouse or UWB rig."),
        (("max_deadreckoning_sec",), "Max dead-reckoning [s]", FLOAT, None,
         "How long the filter coasts on the IMU alone before it reports not "
         "ready and waits for the next fix. 0 → default. Ignored when "
         "unlimited dead-reckoning is on."),
        (("allow_unlimited_deadreckoning",),
         "Allow unlimited dead-reckoning",
         BOOL, None,
         "Never give up coasting, however long the outage. The position "
         "drifts without bound."),
        (("kalman_update_dt_sec",), "INS kalman_update_dt [s]", FLOAT, None,
         "Covariance prediction period of the navigation filter, the "
         "strapdown integration still runs at the full IMU rate. The IMU "
         "sample period predicts at the IMU rate: more accurate in turns, "
         "but costly on a microcontroller. 0 → default"),
    ]),
    ("IMU noise model (required)", [
        (("imu", "gyr_psd"), "gyr_psd [(rad/s)²/Hz]", FLOAT, None,
         "Gyro noise density from an Allan variance analysis. Required, must "
         "be > 0"),
        (("imu", "acc_psd"), "acc_psd [(m/s²)²/Hz]", FLOAT, None,
         "Accelerometer noise density. Required, must be > 0"),
        (("imu", "gyr_bias_rw"), "gyr_bias_rw [rad/s/√s]", FLOAT, None,
         "Gyro bias random walk"),
        (("imu", "acc_bias_rw"), "acc_bias_rw [m/s²/√s]", FLOAT, None,
         "Accelerometer bias random walk"),
    ]),
    ("IMU calibration (optional)", [
        (("imu", "acc_misalignment"), "acc misalignment (3x3 col-major)",
         VEC, 9, "corrected = M*(raw - bias), all-zero → identity"),
        (("imu", "gyr_misalignment"), "gyr misalignment (3x3 col-major)",
         VEC, 9, "corrected = M*(raw - bias), all-zero → identity"),
        (("imu", "mount_rpy_deg"), "Mounting roll/pitch/yaw [°]", VEC, 3,
         "Rotation from board axes to vehicle axes (ZYX). Lever arms and the "
         "heading baseline are then given in vehicle axes."),
        (("imu", "acc_fixed_bias"), "acc fixed bias [m/s²]", VEC, 3,
         "Removed permanently, never estimated"),
        (("imu", "gyr_fixed_bias"), "gyr fixed bias [rad/s]", VEC, 3,
         "Removed permanently, never estimated"),
    ]),
    ("Magnetometer", [
        (("mag", "enable"), "Enable (needs mag.csv)", BOOL, None, None),
        (("mag", "stddev_ut"), "Stddev [µT]", FLOAT, None,
         "0 → default. Yaw noise is about stddev / horizontal field, so a "
         "large value makes the magnetometer a slow, long-term yaw anchor "
         "instead of a per-epoch yaw sensor."),
        (("mag", "min_delay_ms"), "Min fusion delay [ms]", INT, None,
         "Minimum time between two fusions. 0 → default, negative → fuse "
         "every sample"),
        (("mag", "wmm_year"), "WMM year (e.g. 2024.5)", FLOAT, None,
         "0: magnetic north. > 0: apply the WMM declination for this year → "
         "true north"),
        (("mag", "estimate_bias"),
         "Estimate hard-iron bias (18-state)",
         BOOL, None,
         "Estimates the hard-iron bias online (18-state filter). Needs "
         "attitude changes to converge."),
        (("mag", "misalignment"), "Soft-iron (3x3 col-major)", VEC, 9,
         "all-zero → identity"),
        (("mag", "fixed_bias"), "Hard-iron fixed bias [µT]", VEC, 3,
         "Hard-iron offset that is removed permanently, never estimated."),
    ]),
    ("Barometer", [
        (("baro", "enable"), "Enable (needs baro.csv)", BOOL, None, None),
        (("baro", "stddev_m"), "Stddev [m]", FLOAT, None,
         "Raw sensor accuracy, used by baro_alt and by the local/GNSS height "
         "offset check. See the Baro_Alt group for baro_alt's own process "
         "noise. 0 → default"),
    ]),
    ("GNSS", [
        (("gnss", "enable"), "Fuse GNSS fixes", BOOL, None,
         "0: gnss.csv is still read and scored against, but no fix aids the "
         "filter (inertial-only A/B run on the same recording)."),
        (("gnss", "leverarm_frd"), "Lever arm FRD [m]", VEC, 3,
         "GNSS antenna vs. IMU, body FRD"),
        (("gnss", "pos_stddev_fallback_m"),
         "Pos stddev fallback (hor, ver) [m]",
         VEC, 2,
         "Used instead of zero (unknown) entries in the fix's position "
         "covariance."),
        (("gnss", "vel_stddev_fallback_mps"),
         "Vel stddev fallback [m/s]",
         FLOAT, None,
         "Used instead of zero (unknown) entries in the fix's velocity "
         "covariance."),
        (("gnss", "max_horizontal_pos_stddev_m"),
         "Max hor pos stddev [m]",
         FLOAT, None,
         "Fixes that report a worse horizontal position accuracy than this "
         "are rejected, not just downweighted. 0 → default"),
        (("gnss", "max_vertical_pos_stddev_m"),
         "Max ver pos stddev [m]",
         FLOAT, None,
         "Fixes that report a worse vertical position accuracy than this are "
         "rejected, not just downweighted. 0 → default"),
        (("gnss", "max_horizontal_vel_stddev_mps"),
         "Max hor vel stddev [m/s]",
         FLOAT, None,
         "Fixes that report a worse horizontal velocity accuracy than this "
         "are rejected, not just downweighted. 0 → default"),
        (("gnss", "max_vertical_vel_stddev_mps"),
         "Max ver vel stddev [m/s]",
         FLOAT, None,
         "Fixes that report a worse vertical velocity accuracy than this are "
         "rejected, not just downweighted. 0 → default"),
        (("gnss", "start_max_horizontal_pos_stddev_m"),
         "3D entry: max hor pos stddev [m]",
         FLOAT, None,
         "Entering the 3D solution needs better fixes than fusing one does: "
         "this and the three limits below must hold for the entry dwell "
         "time. 0 → default"),
        (("gnss", "start_max_vertical_pos_stddev_m"),
         "3D entry: max ver pos stddev [m]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "start_max_horizontal_vel_stddev_mps"),
         "3D entry: max hor vel stddev [m/s]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "start_max_vertical_vel_stddev_mps"),
         "3D entry: max ver vel stddev [m/s]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "init_dwell_sec"), "3D entry dwell [s]", FLOAT, None,
         "How long the fixes must stay below the entry limits before the 3D "
         "solution starts. 0 → default"),
        (("gnss", "init_dwell_disable"), "3D entry dwell off", BOOL, None,
         "1: start the 3D solution with the first good fix."),
        (("gnss", "stop_max_horizontal_pos_stddev_m"),
         "3D exit: max hor pos stddev [m]",
         FLOAT, None,
         "The 3D solution is left once every fix of the exit dwell time is "
         "worse than this or one of the three limits below. The filter keeps "
         "running. 0 → default"),
        (("gnss", "stop_max_vertical_pos_stddev_m"),
         "3D exit: max ver pos stddev [m]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "stop_max_horizontal_vel_stddev_mps"),
         "3D exit: max hor vel stddev [m/s]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "stop_max_vertical_vel_stddev_mps"),
         "3D exit: max ver vel stddev [m/s]", FLOAT, None,
         "0 → built-in default"),
        (("gnss", "stop_dwell_sec"), "3D exit dwell [s]", FLOAT, None,
         "How long only bad fixes must arrive before the 3D solution is "
         "left. 0 → default"),
        (("gnss", "stop_disable"), "3D exit off", BOOL, None,
         "1: never leave the 3D solution because of GNSS quality alone."),
        (("gnss", "pos_decimation"), "Pos decimation (every Nth)", INT, None,
         "Fuse position and velocity together only every Nth epoch and the "
         "velocity alone otherwise. Receiver position and velocity are "
         "correlated, so fusing both every time counts the same information "
         "twice. 1 → off, 0 → default"),
        (("gnss", "delay_ms"), "Fix delay [ms]", FLOAT, None,
         "Assumed fixed latency of the fixes. The GNSS delay estimate in the "
         "summary helps to pick a value."),
        (("gnss", "pos_cov_scale"), "Pos cov scale", FLOAT, None,
         "Multiplies the reported position stddev. 0 → 1"),
        (("gnss", "pos_cov_scale_height"),
         "Pos cov scale (height)",
         FLOAT, None,
         "Extra downweighting of the height axis only. 0 → 1"),
        (("gnss", "vel_cov_scale"), "Vel cov scale", FLOAT, None,
         "Multiplies the reported velocity stddev. 0 → 1"),
        (("gnss", "pos_stddev_floor_hor_m"),
         "Pos stddev floor hor [m]",
         FLOAT, None,
         "Lower bound for the horizontal position stddev: stddev = max(scale "
         "· reported, floor)"),
        (("gnss", "pos_stddev_floor_ver_m"),
         "Pos stddev floor ver [m]",
         FLOAT, None,
         "Lower bound for the vertical position stddev."),
        (("gnss", "vel_stddev_floor_hor_mps"),
         "Vel stddev floor hor [m/s]",
         FLOAT, None,
         "Lower bound for the horizontal velocity stddev."),
        (("gnss", "vel_stddev_floor_ver_mps"),
         "Vel stddev floor ver [m/s]",
         FLOAT, None,
         "Lower bound for the vertical velocity stddev."),
        (("gnss", "pos_stddev_cap_hor_m"),
         "Pos stddev cap hor [m]",
         FLOAT, None,
         "Upper bound applied after the floors. 0 → default, negative → off"),
        (("gnss", "pos_stddev_cap_ver_m"), "Pos stddev cap ver [m]", FLOAT,
         None, "0 → built-in default, negative → off"),
        (("gnss", "vel_stddev_cap_hor_mps"), "Vel stddev cap hor [m/s]",
         FLOAT, None, "0 → built-in default, negative → off"),
        (("gnss", "vel_stddev_cap_ver_mps"), "Vel stddev cap ver [m/s]",
         FLOAT, None, "0 → built-in default, negative → off"),
        (("gnss", "acc_envelope_tau_sec"),
         "Accuracy envelope tau [s]",
         FLOAT, None,
         "Time constant of the accuracy tracking: a worse reported accuracy "
         "applies at once, a better one only slowly. 0 → default, negative → "
         "off"),
        (("gnss", "vel_noise_acc_scale_hor"),
         "Vel noise per accel hor",
         FLOAT, None,
         "Extra GNSS velocity noise during manoeuvres, per m/s² of antenna "
         "acceleration. 0 → default, negative → off"),
        (("gnss", "vel_noise_acc_scale_ver"), "Vel noise per accel ver",
         FLOAT, None, "0 → built-in default, negative → off"),
        (("gnss", "vel_noise_acc_window_sec"),
         "Vel noise accel window [s]",
         FLOAT, None,
         "Averaging window of that acceleration. 0 → default, negative → off"),
        (("gnss", "min_delay_ms"), "Min time between fusions [ms]", INT, None,
         "Rate limit of the GNSS fusion. 0 → default"),
    ]),
    ("GNSS heading (dual antenna)", [
        (("heading", "enable"), "Enable (needs heading.csv)", BOOL, None,
         "Azimuth of a moving-base baseline, e.g. NAV-RELPOSNED from the "
         "converter that wrote the dataset."),
        (("heading", "baseline_frd"), "Baseline base → rover (FRD)", VEC, 3,
         "Direction from the base antenna to the rover antenna in the body "
         "frame, any length. [1, 0, 0] means the azimuth is the yaw."),
        (("heading", "require_fixed"), "Only carrier-phase fixed", BOOL, None,
         "Use only carrier-phase fixed headings. A float heading can be "
         "degrees off while claiming better."),
        (("heading", "stddev_scale"), "1σ scale", FLOAT, None,
         "Multiplies the receiver's per-row 1σ, 0 → 1.0"),
        (("heading", "stddev_min_deg"), "1σ floor [°]", FLOAT, None,
         "Applied after scaling, 0 → no floor"),
        (("heading", "delay_ms"), "Delay [ms]", INT, None,
         "Age of a row at its timestamp. Start from the GNSS delay."),
    ]),
    ("Absolute speed", [
        (("speed", "enable"), "Enable (needs speed.csv)", BOOL, None, None),
        (("speed", "scale"), "Scale", FLOAT, None,
         "Speed sensor scale correction; 0 → 1.0"),
        (("speed", "stddev_mps"), "Stddev [m/s]", FLOAT, None,
         "Per-sample 1σ; 0 → library default"),
        (("speed", "stddev_rel"), "Relative stddev", FLOAT, None,
         "Speed-proportional 1σ; 0 → library default"),
        (("speed", "min_speed_mps"), "Min speed [m/s]", FLOAT, None,
         "A sample is skipped while the filtered speed is below this. 0 → "
         "default"),
        (("speed", "delay_ms"), "Delay [ms]", FLOAT, None,
         "Age of a sample at its timestamp."),
    ]),
    ("Ranges to anchors", [
        (("ranges", "enable"), "Enable (needs ranges.csv)", BOOL, None,
         "Ranges to anchors at known ECEF positions, one row per range with "
         "its own 1σ, e.g. two-way time-of-flight radio ranging."),
        (("ranges", "leverarm_frd"), "Ranging antenna lever arm (FRD) [m]",
         VEC, 3, "Ranging antenna position relative to the IMU, body frame"),
        (("ranges", "stddev_scale"), "1σ scale", FLOAT, None,
         "Multiplies each row's 1σ, 0 → 1.0"),
        (("ranges", "stddev_min_m"), "1σ floor [m]", FLOAT, None,
         "Applied after scaling, 0 → no floor"),
        (("ranges", "height_with_baro"),
         "Ranges also correct the height",
         BOOL, None,
         "Off: with a barometer the ranges leave the height to it. On: they "
         "correct it too, which pays off only with anchors spread in height."),
        (("ranges", "aiding_max_hpos_stddev_m"),
         "Position-aiding limit [m]",
         FLOAT, None,
         "An epoch with fused ranges counts as position aiding (resets the "
         "coasting window) once the horizontal 1σ in its worst direction is "
         "at or below this. 0 → default"),
    ]),
    ("Auto-ZUPT/ZARU - one set for ins, ARS/AHRS and baro_alt "
     "(optional, 0 = default)", [
        (("imu", "zero_vel_stddev_mps"), "Zero-vel stddev [m/s]", FLOAT, None,
         "How much a detected standstill's zero velocity is trusted. Tighter "
         "→ faster bias and attitude convergence while parked, but riskier "
         "on a false stop. Also used by baro_alt. 0 → default"),
        (("imu", "zero_rot_stddev_deg"), "Zero-rot stddev [°/s]", FLOAT, None,
         "Same for the zero-rotation update, used by ins and both AHRS "
         "instances. 0 → this tool's default, which differs from the "
         "library's."),
        (("imu", "auto_zupt_static_gyr_stddev_deg"),
         "Window stddev gyro [°/s]",
         FLOAT, None,
         "Main stillness criterion: maximum per-axis noise of the raw gyro "
         "over a short window. Independent of the gyro bias."),
        (("imu", "auto_zupt_static_acc_stddev_mps2"),
         "Window stddev accel [m/s²]",
         FLOAT, None,
         "Same for the accelerometer."),
        (("imu", "auto_zupt_static_gyr_deg"),
         "Bound |gyro| [°/s]",
         FLOAT, None,
         "Additional loose limit on the gyro magnitude."),
        (("imu", "auto_zupt_static_acc_mps2"),
         "Bound ||f|-g| [m/s²]",
         FLOAT, None,
         "Additional loose limit on the deviation of the specific force from "
         "gravity."),
        (("imu", "auto_zupt_max_vel_mps"), "Max |GNSS vel| [m/s]", FLOAT, None,
         "A standstill is only accepted below this GNSS speed. Only used "
         "while a recent, precise fix exists, never the filter's own "
         "velocity. Not used by ARS/AHRS."),
        (("imu", "auto_zupt_max_vel_stddev_mps"),
         "Max GNSS vel 1σ [m/s]",
         FLOAT, None,
         "The GNSS velocity must report at least this accuracy to count as "
         "evidence of a standstill."),
        (("imu", "auto_zupt_dwell_sec"), "Dwell [s]", FLOAT, None,
         "How long the platform must look still before a trigger"),
        (("imu", "auto_zupt_min_interval_sec"), "Min interval [s]",
         FLOAT, None, "Minimum time between auto-triggers"),
        (("imu", "auto_zupt_disable"), "Disable everywhere (0/1)",
         FLOAT, None, "1 → no stillness detection anywhere in the suite"),
        (("imu", "auto_zupt_velocity_blind_disable"),
         "Disable ARS/AHRS detector (0/1)",
         FLOAT, None,
         "1: switch off only the own detector of ARS/AHRS, which cannot tell "
         "constant-velocity cruise from standing still. ins keeps deciding "
         "for all filters."),
    ]),
    ("Initial attitude hint (auto-init only, optional)", [
        (("init_hint", "roll_deg"), "Roll [°]", FLOAT, None,
         "Known initial roll, used together with the pitch (both stddevs "
         "must be > 0). Usually not needed: gravity already levels the "
         "filter."),
        (("init_hint", "pitch_deg"), "Pitch [°]", FLOAT, None,
         "Known initial pitch, see roll above"),
        (("init_hint", "rpy_stddev_deg"), "Roll/pitch stddev [°]", FLOAT, None,
         "0 → no roll/pitch hint, the filter levels itself from gravity."),
        (("init_hint", "yaw_deg"), "Yaw [°]", FLOAT, None,
         "Known initial yaw, e.g. a known launch heading, for starts without "
         "magnetometer or GNSS-course aiding."),
        (("init_hint", "yaw_stddev_deg"), "Yaw stddev [°]", FLOAT, None,
         "0 → no yaw hint, yaw stays unobservable until aiding arrives."),
    ]),
    ("Initial-state stddev overrides (0 = built-in)", [
        (("init_stddev", "pos_init_stddev_m"), "Position [m]",
         FLOAT, None, "Initial position std.-dev."),
        (("init_stddev", "vel_init_stddev_mps"), "Velocity [m/s]",
         FLOAT, None, "Initial velocity std.-dev."),
        (("init_stddev", "rpy_init_stddev_rad_deg"), "Attitude [°]",
         FLOAT, None, "Initial attitude std.-dev."),
        (("init_stddev", "yaw_init_stddev_rad_deg"), "Yaw [°]", FLOAT, None,
         "Initial yaw std.-dev. 0 → same as the attitude value"),
        (("init_stddev", "acc_bias_init_stddev_mps2"), "Acc bias [m/s²]",
         FLOAT, None, "Initial accelerometer bias std.-dev."),
        (("init_stddev", "gyr_bias_init_stddev_rps_deg"), "Gyr bias [°/s]",
         FLOAT, None, "Initial gyroscope bias std.-dev."),
    ]),
    ("Free inertial start (IMU dead reckoning only)", [
        (("free_inertial_start", "enable"), "Enable", BOOL, None,
         "Offers the start position below as a one-time position fix, then "
         "dead reckons. Also lifts the dead-reckoning limit and the "
         "GNSS-quality 3D exit."),
        (("free_inertial_start", "lat_deg"), "Latitude [°]", FLOAT, None,
         "Start position, REQUIRED when enabled"),
        (("free_inertial_start", "lon_deg"), "Longitude [°]", FLOAT, None,
         "Start position, REQUIRED when enabled"),
        (("free_inertial_start", "height_m"), "Height [m]", FLOAT, None,
         "Ellipsoidal height of the start position"),
        (("free_inertial_start", "stddev_m"), "Stddev [m]", FLOAT, None,
         "1σ of the declared position, must pass the 3D entry gate"),
    ]),
    ("INS process noise margin (ins only, optional, 0 = ins.c default)", [
        (("imu", "pos_pred_stddev_m_sqrts"),
         "pos_pred_stddev [m/√s]",
         FLOAT, None,
         "Extra process noise on top of the one derived from the "
         "accelerometer and gyro noise. 0 → default"),
        (("imu", "vel_pred_stddev_mps_sqrts"),
         "vel_pred_stddev [m/s/√s]",
         FLOAT, None,
         "Extra velocity process noise. 0 → default. Often dominates the "
         "error growth when the IMU noise is set tightly, see \"process noise "
         "growth rate\" in the summary."),
        (("imu", "rpy_pred_stddev_rad_sqrts"),
         "rpy_pred_stddev [rad/√s]",
         FLOAT, None,
         "Extra attitude process noise of the INS only. ARS/AHRS have their "
         "own value under \"ARS/AHRS noise model\" below. 0 → default"),
    ]),
    ("ARS/AHRS noise model (optional, 0 = ahrs.c default)", [
        (("ahrs", "gyr_noise_psd"), "gyr_noise_psd [rad/s/√Hz]", FLOAT, None,
         "Shared by ARS and AHRS (same physical gyro). 0 → √(imu.gyr_psd)"),
        (("ahrs", "gyr_bias_rw"), "gyr_bias_rw [rad/s²/√Hz]", FLOAT, None,
         "Shared by ARS and AHRS. 0 → imu.gyr_bias_rw"),
        (("ahrs", "rpy_pred_stddev_rad_sqrts"), "rpy_pred_stddev [rad/√s]",
         FLOAT, None,
         "Extra attitude process noise of ARS and AHRS only, independent of "
         "the INS value under \"INS process noise margin\". Tuning knob for "
         "model errors (scale factor, misalignment, mounting), needed when "
         "imu: holds a good sensor's own figure. 0 → none"),
        (("ahrs", "acc_noise_mps2"), "acc_noise [m/s²]", FLOAT, None,
         "Shared by ARS and AHRS (same accelerometer, used for leveling). 0 "
         "→ default"),
        (("ahrs", "gyr_bias_init_stddev_rps_deg"),
         "gyr_bias_init stddev [°/s]",
         FLOAT, None,
         "Initial gyro bias uncertainty of both filters. 0 → stddev of the "
         "static-window seed (or the default if the window found nothing)"),
    ]),
    ("ARS/AHRS leveling and heading tuning (optional, 0 = ahrs.c default)", [
        (("ahrs", "acc_freq_hz"), "acc_freq [Hz]", FLOAT, None,
         "Max. rate of accelerometer leveling corrections of ARS and AHRS. "
         "0 → default"),
        (("ahrs", "acc_cutoff_freq_hz"), "acc_cutoff_freq [Hz]", FLOAT, None,
         "Accelerometer low-pass cut-off: tolerance against vibration at the "
         "cost of delay. 0 → default"),
        (("ahrs", "gravity_diff_penalty"), "gravity_diff_penalty", FLOAT, None,
         "Extra distrust of the leveling update per m/s² of |acc| − g "
         "(higher = more resistant to acceleration phases). 0 → default, "
         "< 0 → off"),
        (("ahrs", "acc_reject_gravity_mps2"), "acc_reject_gravity [m/s²]",
         FLOAT, None,
         "Hard-reject the leveling update while the low-passed | |acc| − g | "
         "exceeds this. 0 → default, < 0 → never reject"),
        (("ahrs", "chi2_threshold"), "chi2_threshold", FLOAT, None,
         "χ² outlier gate of the accelerometer update, per scalar row. "
         "0 → default"),
        (("ahrs", "kalman_update_dt_sec"), "kalman_update_dt [s]", FLOAT,
         None,
         "Covariance prediction period, the attitude integration still runs "
         "at the full IMU rate. 0 → default"),
        (("ahrs", "mag_yaw_stddev_deg"), "mag_yaw_stddev [°]", FLOAT, None,
         "Stddev of the magnetometer heading measurement (AHRS only). "
         "0 → default"),
        (("ahrs", "mag_freq_hz"), "mag_freq [Hz]", FLOAT, None,
         "Max. rate of magnetometer heading corrections (AHRS only). "
         "0 → default"),
        (("ahrs", "mag_chi2_threshold"), "mag_chi2_threshold", FLOAT, None,
         "χ² outlier gate of the heading update (AHRS only). "
         "0 → default"),
    ]),
    ("Baro_Alt (process noise, optional, 0 = baro_alt.c default)", [
        (("baro", "acc_noise_mps2_sqrthz"),
         "acc_noise [m/s²/√Hz]",
         FLOAT, None,
         "baro_alt's own accelerometer noise density. 0 → default"),
        (("baro", "acc_bias_rw"), "acc_bias_rw [m/s²/√Hz]", FLOAT, None,
         "baro_alt's accelerometer bias drift density. 0 → default"),
        (("baro", "acc_bias_init_mps2"), "acc_bias_init [m/s²]", FLOAT, None,
         "baro_alt's initial accelerometer bias uncertainty. It is an "
         "initial condition, not a noise rate, and can dominate the error "
         "growth for tens of seconds. 0 → default"),
        (("baro", "h_process_noise"), "h_process_noise [m/√Hz]", FLOAT, None,
         "Small process noise on baro_alt's height state, a margin against "
         "discretization and model errors. 0 → default"),
        (("baro", "ins_acc_bias_disable"), "Don't use the ins accel bias",
         BOOL, None,
         "By default baro_alt gets the accelerometer bias of ins while ins "
         "is ready and keeps only the rest in its own correction. Tick to "
         "keep baro_alt independent of ins, e.g. as a cross-check."),
    ]),
    ("Baro/GNSS offset filter (optional, 0 = local_gnss_alt.c default)", [
        (("baro", "local_gnss_rw_stddev_mps"), "rw_stddev [m/√s]", FLOAT, None,
         "Random walk of the offset between local height and ellipsoid. "
         "Raise it for large altitude excursions (e.g. a glider gaining "
         "kilometres), where the offset would otherwise lag behind. 0 → "
         "default"),
        (("baro", "local_gnss_chi2_threshold"), "chi2_threshold", FLOAT, None,
         "Outlier gate on the offset innovation. It downweights and does not "
         "drop. 0 → default"),
        (("baro", "local_gnss_min_update_interval_sec"),
         "min_update_interval [s]",
         FLOAT, None,
         "Minimum time between two offset fusions. 0 → default"),
        (("baro", "local_gnss_stddev_inflation_factor"),
         "stddev_inflation_factor",
         FLOAT, None,
         "Inflation applied to both sides of the baro/GNSS pair before they "
         "are combined into the measurement variance. 0 → default"),
    ]),
    ("Scoring (evaluation only, never touches the filter)", [
        (("score", "leverarm_frd"), "Scoring lever arm FRD [m]", VEC, 3,
         "Ground-truth reference point vs. IMU"),
        (("score", "warmup_sec"), "Warmup [s]", FLOAT, None,
         "Scoring starts this long after the first fix."),
        (("score", "ahrs"), "Score the attitude filters too", BOOL, None,
         "1: also score the ARS/AHRS and their regression gates"),
        (("score", "attitude"), "Score attitude", BOOL, None,
         "0: ref.csv has no attitude (placeholder zeros), so the "
         "roll/pitch/yaw error is not reported."),
        (("score", "min_epochs"), "Min scored epochs", INT, None,
         "Minimum number of scored epochs a run must produce. 0 → not gated"),
        (("score", "coast_gap_min_sec"),
         "Coasting gap threshold [s]",
         FLOAT, None,
         "Reports the position error at the first reference epoch after "
         "every aiding gap longer than this. 0 → off. The whole-run RMS "
         "rewards a filter that gives up coasting, this number does not."),
        (("score", "ref_delay_ms"), "Reference delay [ms]", FLOAT, None,
         "Reference rows stamped T describe T minus this. Mirror gnss: "
         "delay_ms when ref.csv comes from the same receiver output, "
         "otherwise the score penalizes correct delay compensation. 0 → "
         "independent reference"),
        (("score", "lim_coast_exit_err_m"),
         "Limit coasting exit error [m]",
         FLOAT, None,
         "Regression gate on that error. 0 → not gated. No solution when "
         "aiding returns counts as a failure."),
    ]),
]


def _get_path(d, path, default=None):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def _set_path(d, path, value):
    for k in path[:-1]:
        d = d.setdefault(k, {})
    d[path[-1]] = value


def _schema_default(path, kind, extra):
    """Default value for a field: replay.DEFAULTS, else a zero of its kind."""
    val = _get_path(replay.DEFAULTS, path)
    if val is None:
        val = {BOOL: 0, FLOAT: 0.0, INT: 0, STR: "",
               CHOICE: extra[0] if extra else "",
               VEC: (0.0,) * (extra or 3)}[kind]
    return val


def merge_spec(raw):
    """Same DEFAULTS merge as replay.load_config(), from an in-memory dict."""
    spec = {}
    for key, dflt in replay.DEFAULTS.items():
        if isinstance(dflt, dict):
            merged = dict(dflt)
            merged.update(raw.get(key) or {})
            spec[key] = merged
        else:
            spec[key] = raw.get(key, dflt)
    spec["imu"].setdefault("gyr_bias_rw", 0.0)
    spec["imu"].setdefault("acc_bias_rw", 0.0)
    return spec


def _value_sigma_rows(label, vals, sigmas, dec, width, label_w=None):
    """Values and, right under them, their 1-sigma as two monospace rows
    with the same decimals and field width, so each decimal point sits
    below its value's. `label` heads the values row inline when `label_w`
    is given, else it is a line of its own. The second row starts with
    the ± in the label's place."""
    sigmas = sigmas or (math.nan,) * len(vals)
    v_row = " ".join(f"{v:+{width}.{dec}f}" for v in vals)
    s_row = " ".join(f"{x:{width}.{dec}f}" for x in sigmas)
    if label_w is None:
        return f"{label}\n {v_row}\n±{s_row}"
    return f"{label:<{label_w}} {v_row}\n{'±':>{label_w}} {s_row}"


def load_raw_config(path):
    """Load config.yaml WITHOUT the defaults merge (the editor needs the raw
    keys to preserve unknown ones). `path` may be a dataset directory or the
    YAML file itself. Returns (raw_dict, cfg_path, data_dir)."""
    import yaml
    if os.path.isdir(path):
        cfg_path = os.path.join(path, "config.yaml")
        data_dir = path
    else:
        cfg_path = path
        data_dir = os.path.dirname(path) or "."
    raw = {}
    if os.path.exists(cfg_path):
        with open(cfg_path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if raw is None:
            raw = {}
        if not isinstance(raw, dict):
            raise ValueError(f"{cfg_path} is not an INSLIB config (top "
                             f"level is a {type(raw).__name__}, expected "
                             "key: value sections)")
    return raw, cfg_path, data_dir


def looks_like_config(raw):
    """False for a YAML mapping that shares no top-level key with the
    config schema (a docker-compose file, a CI workflow, ...). An empty
    file is fine, it is a config yet to be written."""
    known = set(replay.DEFAULTS) | set(replay.FOREIGN_SECTIONS)
    return not raw or any(k in known for k in raw)


def validate_spec(spec, data_dir):
    """Pre-run checks, returns a list of error strings (empty = ok)."""
    errors = []
    if not spec["imu"].get("gyr_psd") or not spec["imu"].get("acc_psd"):
        errors.append("imu: gyr_psd/acc_psd missing or zero (required "
                      "noise model)")
    fi = spec["free_inertial_start"]
    if int(fi["enable"]) and (fi["lat_deg"] is None
                              or fi["lon_deg"] is None):
        errors.append("free_inertial_start needs lat_deg and lon_deg: the "
                      "whole point is the origin you supply")
    need = {"imu": True,
            "ref": spec["aiding"] == "ref" or spec["init"] == "ref",
            "gnss": spec["aiding"] == "gnss",
            "mag": bool(int(spec["mag"]["enable"])),
            "baro": bool(int(spec["baro"]["enable"])),
            "heading": bool(int(spec["heading"]["enable"])),
            "speed": bool(int(spec["speed"]["enable"])),
            "ranges": bool(int(spec["ranges"]["enable"]))}
    for stream, needed in need.items():
        path = replay.input_path(data_dir, spec, stream)
        if needed and not os.path.exists(path):
            errors.append(f"{os.path.basename(path)} missing in {data_dir}")
    return errors


def is_config_file(name):
    return name.lower().endswith((".yaml", ".yml"))


def config_label(cfg_path, base=None):
    """Combo box label: the dataset directory, plus the file name unless it
    is the conventional config.yaml."""
    d, name = os.path.split(os.path.abspath(cfg_path))
    label = os.path.relpath(d, base) if base else d
    return label if name == "config.yaml" else f"{label}  [{name}]"


# ============================================================================
# Replay worker (replay_core.Replay on a Qt thread)
# ============================================================================

from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: E402
import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
import pyqtgraph.opengl as gl  # noqa: E402
from OpenGL import GL  # noqa: E402  (pyqtgraph.opengl's own dependency)

from ins_map_view import MapView  # noqa: E402
import ins_gui_theme as theme  # noqa: E402

# The window icon. It lives with the documentation rather than with the
# tools, so the path is taken relative to THIS FILE and not to the working
# directory - the window then has its icon whichever directory it was
# started from. In a PyInstaller bundle the same file is shipped under the
# bundle's own root (see scripts/inspostgui.spec).
_RES_ROOT = (getattr(sys, "_MEIPASS", os.path.dirname(sys.executable))
             if getattr(sys, "frozen", False)
             else os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOGO = os.path.join(_RES_ROOT, "doc", "figures", "inslib_logo.png")


def app_icon():
    """The application icon, or None when the file is not there.

    A missing logo is not a reason to refuse to start: this tool gets
    copied onto machines that do not carry the documentation tree."""
    if not os.path.isfile(LOGO):
        return None
    icon = QtGui.QIcon(LOGO)
    return None if icon.isNull() else icon


def taskbar_identity():
    """Tell Windows this process is its own application.

    Taskbar buttons are grouped by application id, and a script inherits
    the interpreter's: without this the button carries the Python icon
    however the window is set, and every Qt tool started from the same
    interpreter shares one button."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "inslib.inspostgui")
    except Exception:                                     # noqa: BLE001
        # Cosmetic to begin with, and a shell that will not answer is not
        # a reason to stop.
        pass


class ReplayWorker(QtCore.QThread):
    """Runs the dataset through ins on a worker thread, with the replay
    core replay.py runs on (replay_core.Replay), so both see the same
    inputs, feed order, scoring and summary. Records the same per-sample
    history (self.rec, guarded by self.lock) so ins_plots.plot_results can
    consume it unchanged, plus a low-rate live snapshot (self.live) for the
    UI."""

    sig_status = QtCore.pyqtSignal(str)
    sig_progress = QtCore.pyqtSignal(int)          # percent 0..100
    sig_finished = QtCore.pyqtSignal(dict)         # results (see _replay)
    sig_error = QtCore.pyqtSignal(str, str)        # summary, details

    def __init__(self, spec, data_dir, realtime=False, speed=1.0,
                 rec_hz=10.0, gnss_outages=(), smooth=False, parent=None):
        super().__init__(parent)
        self.spec = spec
        # Run the RTS smoother after the replay (see _smooth()).
        self.smooth = smooth
        self.data_dir = data_dir
        self.realtime = realtime
        self.speed = max(speed, 1e-6)
        # (start, end) windows in seconds from the first IMU sample, see
        # replay_core.parse_gnss_outages()
        self.gnss_outages = list(gnss_outages)
        self.lock = threading.Lock()
        self._recorder = replay_core.PlotRecorder(rec_hz, lock=self.lock)
        self.rec = self._recorder.rec
        # Every fix of the aiding stream in the local NED frame, one entry
        # per epoch (rec only samples the last one at rec_hz), and next to
        # it R_b_to_n * gnss.leverarm_frd at that epoch, which moves the fix
        # from the antenna onto the IMU point (REQ-VER-039).
        self.fix_ned = []
        self.fix_la_ned = []
        # ins's local NED origin once known (ECEF), for the UI to move the
        # previous run's ghost trail into this run's frame.
        self.origin_ecef = None
        # Anchors the filter was given ranges to so far: id -> local NED.
        self.anchor_ned = {}
        self.live = {}
        self._stop = threading.Event()
        self._pause = threading.Event()

    def stop(self):
        self._stop.set()

    def set_paused(self, paused):
        if paused:
            self._pause.set()
        else:
            self._pause.clear()

    def run(self):
        try:
            self.sig_finished.emit(self._replay())
        except replay_core.ReplayError as e:
            self.sig_error.emit(str(e), "")
        except Exception as e:
            self.sig_error.emit(f"{type(e).__name__}: {e}",
                                traceback.format_exc())

    # ------------------------------------------------------------------
    def _replay(self):
        spec, data_dir = self.spec, self.data_dir
        lines = []
        self.sig_status.emit("Loading dataset ...")
        inp = replay_core.load_inputs(spec, data_dir, self.gnss_outages,
                                      log=lines.append)
        replay_core.log_startup(spec, inp, log=lines.append)
        nav = replay_core.make_navigator(spec, inp, log=lines.append)

        self.sig_status.emit("IMU pre-pass (gaps / rates / noise) ...")
        # North-East map page track at replay.py's --plot-track-hz default,
        # the KML/Map tab track at its --kml-hz default.
        track = replay_core.TrackRecorder(50.0)
        kml = replay_core.KmlRecorder(2.0)
        filter_score = ins_rts.EpochScore() if self.smooth else None
        # The filter's solution for Export solution.
        solution = ins_solution.SolutionRecorder(SOLUTION_HZ)
        r = replay_core.Replay(spec, inp, nav, observers=(
            self._recorder, track, kml, _LiveTap(self), solution)
            + ((filter_score,) if filter_score is not None else ()))
        n_total = r.n_imu_total
        self.sig_status.emit(f"Replaying {spec['name'] or data_dir} "
                             f"({n_total} IMU samples) ...")

        pacer = replay_core.Pacer(self.speed) if self.realtime else None
        last_pct = [-1]

        def progress(n):
            pct = int(100 * n / n_total) if n_total else 100
            if pct != last_pct[0]:
                last_pct[0] = pct
                self.sig_progress.emit(pct)

        def before_epoch():
            while self._pause.is_set() and not self._stop.is_set():
                time.sleep(0.05)
                if pacer is not None:
                    pacer.shift(0.05)  # keep realtime pacing across the pause
            return not self._stop.is_set()

        r.run(progress=progress, before_epoch=before_epoch, pacer=pacer,
              should_stop=self._stop.is_set)

        if r.aborted:
            lines.append(f"*** STOPPED by user after {r.n_imu}/{n_total} "
                         f"IMU samples, partial results below ***")
        r.report(log=lines.append)
        rec = self.rec
        with self.lock:
            insdoctor = r.findings(rec)
        lines.append("")
        lines += replay.insdoctor_lines(insdoctor)

        final_mode = nav.mode_name()
        nav.close()

        gnss_delay_curve = None
        if r.gnss_delay_auto():
            with self.lock:
                gnss_delay_curve = replay.gnss_delay_correlation_curve(
                    rec["t"], rec["baro_vel_d"], rec["gnss_vel_d"])
            lines += replay_core.gnss_delay_lines(gnss_delay_curve,
                                                  r.gnss_delay_ms)

        # Whole-run inputs of the map, magnetometer and ellipsoid altitude
        # pages, the same as replay.py's --plot block.
        extra = r.plot_extras()
        extra["track_ne"] = track.est
        extra["track_ref_ne"] = track.ref
        with self.lock:
            rec.update(extra)

        rts = None
        if self.smooth and not r.aborted:
            last_pct[0] = -1
            rts = self._smooth(spec, inp, r, filter_score, lines, progress,
                               before_epoch)

        return {
            "text": "\n".join(lines),
            "rec": rec,
            "gnss_delay_ms": r.gnss_delay_ms,
            "sensor_rate": r.sensor_rate,
            "gnss_delay_curve": gnss_delay_curve,
            "findings": insdoctor,
            "growth_rate": replay.process_noise_growth_rate(spec["imu"]),
            "baro_growth_rate": replay.baro_alt_growth_rate(spec["baro"]),
            "ahrs_growth_rate": replay.ahrs_growth_rate(replay.effective_ahrs_cfg(spec)),
            "name": spec["name"] or os.path.basename(
                os.path.normpath(data_dir)),
            "warmup_sec": float(spec["score"]["warmup_sec"]),
            "warmup_end_sec": ((r.t_warmup_end - r.t0_us) / US_PER_SEC
                               if r.t0_us is not None else 0.0),
            "kml_est": kml.est,
            "origin_ecef": (tuple(r.origin_ecef) if r.origin_ecef is not None
                            else None),
            "kml_ref": kml.ref,
            "kml_fix": kml.fix,
            "fix_latlon": [(math.degrees(fx["lat_rad"]),
                            math.degrees(fx["lon_rad"]))
                           for fx in inp.fixes[:r.ifix]],
            "aborted": r.aborted,
            "mode": final_mode,
            "pos_rms_m": r.e_pos.rms(),
            "pos_max_m": r.e_pos.max_abs,
            "att_rms_deg": ((r.e_roll.rms(), r.e_pitch.rms(), r.e_yaw.rms())
                            if r.e_yaw.n else None),
            "scored_epochs": r.e_pos.n,
            "rts": rts,
            "solution": solution.solution(),
            "gps_clock": inp.gps_clock,
        }

    def _smooth(self, spec, inp, r, filter_score, lines, progress,
                before_epoch):
        """The RTS smoother over the replay just done (ins_rts.run_smoother:
        its own forward pass without a coasting limit, then the backward
        pass). Appends its summary to `lines`, adds the rts_* series to
        self.rec (ins_rts.add_rec_series, plus rec["rts_lines"], the
        summary lines, for the PDF) and returns the results entry: the 3D
        trail (NED points + breaks), the KML/map track and the scores, or
        None if it was stopped."""
        self.sig_status.emit("RTS smoother: forward pass with unlimited "
                             "dead reckoning ...")
        t_start = time.perf_counter()
        smr = ins_rts.run_smoother(spec, inp, progress=progress,
                                   before_epoch=before_epoch)
        if smr is None:
            lines += ["", "RTS smoother: stopped by user, no smoothed solution"]
            return None
        self.sig_status.emit("RTS smoother: preparing the results ...")
        rts = smr.rts
        score_la = tuple(spec["score"].get("leverarm_frd") or (0.0, 0.0, 0.0))
        f_score = ins_rts.score_filter(filter_score)
        s_score = ins_rts.score_rts(smr.score, score_la)
        lines.append("")
        n_before = len(lines)
        lines += ins_rts.score_lines(f_score, s_score, rts.n_cuts)
        lines.append(f"  {time.perf_counter() - t_start:.1f} s, "
                     f"{len(rts.k_epoch)} knots, {rts.memory_mb():.0f} MB recorded")

        frame = (r.origin_ecef, r.origin_lat, r.origin_lon)
        trail, breaks = ins_rts.trail(smr, *frame)
        with self.lock:
            ins_rts.add_rec_series(self.rec, smr, r.t0_us, *frame, score_la)
            self.rec["rts_lines"] = lines[n_before:]
        return {"trail": trail, "breaks": breaks,
                "kml_est": ins_rts.kml_track(smr, r.t0_us),
                "solution": ins_rts.solution(smr, SOLUTION_HZ),
                "filter": f_score, "rts": s_score, "cuts": rts.n_cuts}

    def _update_live(self, nav, t, t0_us, duration, acc, gyr, run=None):
        rpy = nav.rpy()  # best available: ins, else AHRS, else ARS
        # In the replay's frame, where the trail and the reference are.
        pos = (replay_core.ins_pos_in_replay_frame(nav, run) if run is not None
               else nav.position_local())
        vel = nav.velocity_ned()
        sd = nav.stddev()
        ecef = nav.position_ecef()
        llh = None
        if ecef is not None:
            lat, lon, alt = ecef_to_llh(*ecef)
            llh = (math.degrees(lat), math.degrees(lon), alt)
        dw = nav.downweight_counts()
        acc_bias = nav.bias_acc()
        gyr_bias = nav.bias_gyr()
        baro = nav.baro_alt()
        baro_sd = nav.baro_stddev()
        gnss_offset = nav.local_gnss_offset()
        live = {
            "t_sec": (t - t0_us) / US_PER_SEC,
            "duration": duration,
            "mode": nav.mode_name(),
            "ready": nav.is_ready(),
            "zupt": nav.auto_zupt_active(),
            "pos": pos,
            "vel": vel,
            "rpy": rpy,
            "quat": rpy_to_quat(*rpy) if rpy is not None else None,
            "llh": llh,
            "baro_h_d": -baro[0] if baro is not None else None,
            "baro_h_sigma": baro_sd[0] if baro_sd is not None else None,
            "local_gnss_offset": (gnss_offset[0]
                                  if gnss_offset is not None else None),
            "local_gnss_offset_sigma": (gnss_offset[1]
                                        if gnss_offset is not None else None),
            "pos_sigma": sd["pos_ned"] if sd else None,
            "vel_sigma": sd["vel_ned"] if sd else None,
            "rpy_sigma": nav.rpy_stddev(),  # best available, matches "rpy"
            "acc_bias": acc_bias,
            "acc_bias_sigma": sd["acc_bias"] if sd else None,
            "gyr_bias": gyr_bias,
            "gyr_bias_sigma": sd["gyr_bias"] if sd else None,
            "acc": acc,
            "gyr": gyr,
            "dw": {"full3d": nav.diag()["n_downweighted"], **dw},
        }
        with self.lock:
            self.live = live


class _LiveTap(replay_core.Observer):
    """The worker's per-epoch view of the replay: every fix in the local
    frame for the 3D view, the local origin once known, and the live
    snapshot at about 30 Hz of sim time."""

    def __init__(self, worker):
        self.w = worker
        self._last_live_us = None
        self._live_period_us = US_PER_SEC / 30.0
        self._i_range = 0

    def on_epoch(self, r):
        w = self.w
        if w.origin_ecef is None and r.origin_ecef is not None:
            w.origin_ecef = tuple(r.origin_ecef)
        if r.fix_now is not None and r.origin_ecef is not None:
            p_fix = replay.ref_to_local_ned(r.fix_now, r.origin_ecef,
                                            r.origin_lat, r.origin_lon)
            la_n = replay.ref_point_offset_ned(r.nav, r.leverarm)
            with w.lock:
                w.fix_ned.append(p_fix)
                w.fix_la_ned.append(la_n)
        ranges = r.ranges
        if ranges and r.origin_ecef is not None:
            # Rows up to now, so an anchor appears with its first range.
            while (self._i_range < len(ranges)
                   and ranges[self._i_range][0] <= r.t):
                _, aid, ecef, _, _ = ranges[self._i_range]
                self._i_range += 1
                if aid not in w.anchor_ned:
                    lat, lon, alt = ecef_to_llh(*ecef)
                    p = replay.ref_to_local_ned(
                        {"lat_rad": lat, "lon_rad": lon, "h_m": alt},
                        r.origin_ecef, r.origin_lat, r.origin_lon)
                    with w.lock:
                        w.anchor_ned[aid] = p
        if (self._last_live_us is None
                or (r.t - self._last_live_us) >= self._live_period_us):
            self._last_live_us = r.t
            w._update_live(r.nav, r.t, r.t0_us, r.imu_duration, r.a, r.g, r)


# ============================================================================
# 3D views (lifted from znav3d.py, MAVLink specifics removed, ground-truth
# trail added)
# ============================================================================

# Colours come from ins_gui_theme. The GL lines/points blend by alpha
# instead of pyqtgraph's default 'additive', which adds up to white on a
# light background and makes them vanish. Like 'additive' (and unlike
# 'translucent') without the depth test: the origin axes lie in the grid
# plane and would z-fight with it, so draw order decides.
GL_BLEND = {
    GL.GL_DEPTH_TEST: False,
    GL.GL_BLEND: True,
    'glBlendFunc': (GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA),
}
TRAIL_MAX_POINTS = 100_000
# Speed [m/s] at the red end of the trail colour ramp before a faster
# sample widens it (auto mode).
AUTO_SPEED_SCALE = 5.0
ANTIALIAS = False


def _move_ned_origin(points_ned, origin_from, origin_to):
    """NED points relative to ECEF origin_from -> relative to origin_to
    (both tangent frames at their own origin)."""
    if not points_ned:
        return []
    o1 = np.asarray(origin_from, dtype=float)
    o2 = np.asarray(origin_to, dtype=float)
    lat1, lon1, _ = ecef_to_llh(*o1)
    lat2, lon2, _ = ecef_to_llh(*o2)
    r1 = np.asarray(replay.ned_to_ecef_rot(lat1, lon1), dtype=float)
    r2 = np.asarray(replay.ned_to_ecef_rot(lat2, lon2), dtype=float)
    ecef = o1 + np.asarray(points_ned, dtype=float) @ r1.T
    return [tuple(p) for p in (ecef - o2) @ r2]


def _speeds_to_rgba(speeds, scale, value=1.0):
    """Vectorized speed [m/s] -> RGBA (blue = slow, red = fast). `value`
    darkens the hues (HSV value) for a light background."""
    v = np.clip(np.asarray(speeds, dtype=np.float32) / max(scale, 1e-6),
                0.0, 1.0)
    h = 0.66 * (1.0 - v) * 6.0  # HSV hue sector, S=V=1
    i = np.floor(h).astype(int) % 6
    f = h - np.floor(h)
    out = np.empty((len(v), 4), dtype=np.float32)
    q, tt = 1.0 - f, f
    r = np.choose(i, [np.ones_like(f), q, np.zeros_like(f),
                      np.zeros_like(f), tt, np.ones_like(f)])
    g = np.choose(i, [tt, np.ones_like(f), np.ones_like(f), q,
                      np.zeros_like(f), np.zeros_like(f)])
    b = np.choose(i, [np.zeros_like(f), np.zeros_like(f), tt,
                      np.ones_like(f), np.ones_like(f), q])
    out[:, 0], out[:, 1], out[:, 2] = r * value, g * value, b * value
    out[:, 3] = 0.9
    return out


def _make_box_mesh(sx=1.0, sy=0.6, sz=0.2):
    v = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ], dtype=np.float32)
    f = np.array([
        [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0],
    ], dtype=np.int32)
    c = np.array([
        [0.3, 0.4, 0.5, 1], [0.3, 0.4, 0.5, 1], [0.6, 0.7, 0.8, 1],
        [0.6, 0.7, 0.8, 1], [0.4, 0.5, 0.6, 1], [0.4, 0.5, 0.6, 1],
        [0.9, 0.3, 0.3, 1], [0.9, 0.3, 0.3, 1], [0.4, 0.5, 0.6, 1],
        [0.4, 0.5, 0.6, 1], [0.35, 0.45, 0.55, 1], [0.35, 0.45, 0.55, 1],
    ], dtype=np.float32)
    return v, f, c


def _load_vehicle_mesh(scale=5.0):
    """vehicle.stl next to this script or in the cwd (needs numpy-stl),
    falls back to a simple oriented box."""
    for cand in (os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "vehicle.stl"), "vehicle.stl"):
        if not os.path.exists(cand):
            continue
        try:
            from stl import mesh
            vehicle = mesh.Mesh.from_file(cand)
            verts = vehicle.points.reshape(-1, 3) * scale
            faces = np.arange(verts.shape[0]).reshape(-1, 3)
            colors = np.ones((faces.shape[0], 4), dtype=np.float32)
            colors[:, 0:3] = [0.9, 0.9, 0.9]
            return verts, faces, colors
        except Exception as e:
            print(f"could not load {cand}: {e}")
    return _make_box_mesh()


def _quat_to_rotmat(q):
    """(w,x,y,z) body FRD -> NED, row-major np 3x3."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def _rotate_ned_mesh(base_verts, R):
    """Rotate NED-frame mesh vertices for pyqtgraph's display frame
    (x north, y = -east, z = -down), same double-flip as znav3d."""
    v = base_verts.copy()
    v[:, 1] *= -1
    v[:, 2] *= -1
    v = v @ R.T
    v[:, 1] *= -1
    v[:, 2] *= -1
    return v


def _break_segments(points_ned, breaks):
    """'lines' vertex pairs (display frame x, -y, -z) for NED points with
    append_trail()'s break flags, or None with nothing to draw."""
    if len(points_ned) < 2:
        return None
    pts = np.array([(p[0], -p[1], -p[2]) for p in points_ned],
                   dtype=np.float32)
    end = np.nonzero(~np.array(breaks[1:], dtype=bool))[0] + 1
    if not len(end):
        return None
    seg = np.empty((2 * len(end), 3), dtype=np.float32)
    seg[0::2], seg[1::2] = pts[end - 1], pts[end]
    return seg


def _nice_spacing(raw):
    """Smallest 1-2-5 step (1, 2, 5, 10, ...) that is >= raw."""
    decade = 10.0 ** math.floor(math.log10(raw))
    for mant in (1.0, 2.0, 5.0, 10.0):
        if mant * decade >= raw:
            return mant * decade
    return 10.0 * decade


class Position3DView(gl.GLViewWidget):
    manual_pan_requested = QtCore.pyqtSignal()
    grid_spacing_changed = QtCore.pyqtSignal(float)  # [m] now on screen

    def __init__(self, parent=None):
        super().__init__(parent)
        self.opts['distance'] = 50
        self.opts['elevation'] = 35
        self.opts['azimuth'] = 170
        self.orbit_label = QtWidgets.QLabel(self)
        self.orbit_label.setAttribute(
            QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.orbit_label.setStyleSheet(
            MONO + " background: rgba(0, 0, 0, 140); color: #ddd;"
            " padding: 2px 6px; border-radius: 3px;")
        self.orbit_label.hide()

        self.grid = gl.GLGridItem()
        self._grid_fixed = None  # user's spacing [m], None = automatic
        self._grid_spacing = None
        self._grid_cells = None
        self._grid_origin = None
        self.addItem(self.grid)
        self._update_grid()
        L = 20
        for end, col in (([L, 0, 0], (1, 0.3, 0.3, 1)),
                         ([0, -L, 0], (0.3, 0.8, 0.3, 1)),
                         ([0, 0, L], (0.3, 0.6, 1, 1))):
            self.addItem(gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], end]), color=col, width=2,
                glOptions=GL_BLEND))

        # The previous run's trail, drawn behind this one for an A/B look
        # at a config change (display frame, 'lines' pairs like the trail).
        self.ghost_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32), width=1.5,
            antialias=ANTIALIAS, mode='lines', glOptions=GL_BLEND)
        self.ghost_item.setVisible(False)
        self.addItem(self.ghost_item)
        self._ghost_wanted = True
        self._ghost_has_data = False

        # The RTS smoother's solution of this run, drawn once the run is
        # done (same 'lines' pairs as the trail).
        self.rts_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32), width=1.5,
            antialias=ANTIALIAS, mode='lines', glOptions=GL_BLEND)
        self.rts_item.setVisible(False)
        self.addItem(self.rts_item)
        self._rts_wanted = True
        self._rts_has_data = False

        self.trail_points = []      # display frame (x, -y, -z)
        self.trail_speeds = []
        # Per point: does the trail START here (no segment drawn from the
        # previous point)? Set when the filter published nothing in
        # between, e.g. while it is re-arming after an outage. Drawn as
        # 'lines' rather than 'line_strip' for exactly this reason: a strip
        # would bridge the gap with a straight chord, which looks like a
        # trajectory the vehicle never flew.
        self.trail_break = []
        self._speed_fixed = None  # user's red end [m/s], None = automatic
        self._speed_scale = AUTO_SPEED_SCALE
        self._last_obj_pos = (0.0, 0.0, 0.0)
        self.trail_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32),
            color=np.zeros((2, 4), dtype=np.float32),
            width=1.5, antialias=ANTIALIAS, mode='lines', glOptions=GL_BLEND)
        self.addItem(self.trail_item)

        self.ref_points = []
        self.ref_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32),
            width=1.0, antialias=ANTIALIAS, mode='line_strip',
            glOptions=GL_BLEND)
        self.addItem(self.ref_item)

        # The aiding fixes as dots: 1 Hz fixes joined by lines would draw
        # chords the vehicle never drove.
        self.fix_points = []
        self.fix_item = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3), dtype=np.float32), size=4, pxMode=True,
            glOptions=GL_BLEND)
        self.fix_item.setVisible(False)
        self.addItem(self.fix_item)
        self._fix_wanted = True

        # Where on the trail a ZUPT/ZARU was applied, one dot per recorder
        # tick, so a standstill shows as a cluster.
        self.zupt_points = []
        self.zupt_item = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3), dtype=np.float32), size=5, pxMode=True,
            glOptions=GL_BLEND)
        self.zupt_item.setVisible(False)
        self.addItem(self.zupt_item)
        self._zupt_wanted = False

        # Ranging anchors: a dot and the anchor id each.
        self.anchor_ids = []
        self.anchor_points = []
        self.anchor_item = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3), dtype=np.float32), size=11, pxMode=True,
            glOptions=GL_BLEND)
        self.anchor_item.setVisible(False)
        self.addItem(self.anchor_item)
        self.anchor_labels = []
        self._anchor_wanted = True

        self.position_item = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]), size=10, glOptions=GL_BLEND)
        self.addItem(self.position_item)

        self.ellipsoid_item = None
        (self._ellipsoid_base_verts,
         self._ellipsoid_faces) = self._make_sphere_mesh(20, 20)

        verts, faces, colors = _load_vehicle_mesh(scale=5.0)
        self.vehicle_base_verts = verts.copy()
        self.vehicle_meshdata = gl.MeshData(vertexes=verts, faces=faces,
                                            faceColors=colors)
        self.vehicle_item = gl.GLMeshItem(meshdata=self.vehicle_meshdata,
                                          smooth=False, shader='shaded',
                                          glOptions='opaque')
        self.vehicle_item.setVisible(False)
        self.addItem(self.vehicle_item)
        theme.themed(self._restyle)

    def set_grid_spacing(self, spacing):
        """Fixed grid spacing [m], None for automatic (follows the zoom)."""
        self._grid_fixed = spacing
        self._update_grid()
        self.update()

    def grid_spacing(self):
        return self._grid_spacing

    def _update_grid(self):
        """Size the grid to the zoom: spacing from the camera distance (or
        the user's value), edge length a few distances, centred under the
        view and snapped to the spacing so the lines stay put on pan."""
        dist = max(float(self.opts['distance']), 1e-3)
        sp = self._grid_fixed or _nice_spacing(dist / GRID_TARGET_CELLS)
        cells = int(min(GRID_MAX_CELLS,
                        math.ceil(GRID_SPAN_FACTOR * dist / sp)))
        cells = max(2, cells + cells % 2)
        c = self.opts['center']
        origin = (round(c.x() / sp) * sp, round(c.y() / sp) * sp)
        if (sp, cells) != (self._grid_spacing, self._grid_cells):
            self.grid.setSize(x=cells * sp, y=cells * sp)
            self.grid.setSpacing(x=sp, y=sp)
            self._grid_cells = cells
        if origin != self._grid_origin:
            self.grid.resetTransform()
            self.grid.translate(origin[0], origin[1], 0)
            self._grid_origin = origin
        if sp != self._grid_spacing:
            self._grid_spacing = sp
            self.grid_spacing_changed.emit(sp)

    def paintGL(self, *args, **kwargs):
        self._update_grid()
        super().paintGL(*args, **kwargs)

    def _restyle(self):
        self.setBackgroundColor(theme.T["gl_bg"])
        grid = pg.mkColor(theme.T["gl_grid"])
        grid.setAlpha(150)
        self.grid.setColor(grid)
        self.ref_item.setData(color=theme.rgba("ref_trail", 0.6))
        self.ghost_item.setData(color=theme.rgba("ghost", 0.8))
        self.rts_item.setData(color=theme.rgba("rts", 0.9))
        self.fix_item.setData(color=theme.rgba("fix", 0.9))
        self.zupt_item.setData(color=theme.rgba("zupt", 0.9))
        self.anchor_item.setData(color=theme.rgba("anchor", 1.0))
        for lab in self.anchor_labels:
            lab.setData(color=pg.mkColor(theme.T["anchor"]))
        self.position_item.setData(color=np.array([theme.rgba("position")]))
        if self.ellipsoid_item is not None:
            self.ellipsoid_item.setColor(theme.rgba("ellipsoid", 0.16))
        self._redraw_trail()

    @staticmethod
    def _make_sphere_mesh(rows, cols):
        verts = []
        for i in range(rows + 1):
            theta = math.pi * i / rows
            for j in range(cols):
                phi = 2 * math.pi * j / cols
                verts.append([math.sin(theta) * math.cos(phi),
                              math.sin(theta) * math.sin(phi),
                              math.cos(theta)])
        verts = np.array(verts, dtype=np.float32)
        faces = []
        for i in range(rows):
            for j in range(cols):
                a = i * cols + j
                b = i * cols + (j + 1) % cols
                c = (i + 1) * cols + j
                d = (i + 1) * cols + (j + 1) % cols
                faces.extend([[a, b, c], [b, d, c]])
        return verts, np.array(faces, dtype=np.int32)

    # ------------------------------------------------------------------
    def append_trail(self, points_ned, speeds, breaks):
        """Batch-append estimate positions (NED) with per-point speed.

        breaks[i] True starts a new run at that point instead of
        connecting it to the previous one."""
        if not points_ned:
            return
        for p, s, b in zip(points_ned, speeds, breaks):
            self.trail_points.append((p[0], -p[1], -p[2]))
            self.trail_speeds.append(s)
            self.trail_break.append(b)
        overflow = len(self.trail_points) - TRAIL_MAX_POINTS
        if overflow > 0:
            del self.trail_points[:overflow]
            del self.trail_speeds[:overflow]
            del self.trail_break[:overflow]
        smax = max(self.trail_speeds[-len(points_ned):], default=0.0)
        if self._speed_fixed is None and smax > self._speed_scale:
            self._speed_scale = smax * 1.2  # recolor everything below
        self._redraw_trail()

    def _redraw_trail(self):
        if len(self.trail_points) >= 2:
            pts = np.array(self.trail_points, dtype=np.float32)
            col = _speeds_to_rgba(self.trail_speeds, self._speed_scale,
                                  theme.T["trail_value"])
            # One segment per consecutive pair that is not split by a break.
            end = np.nonzero(~np.array(self.trail_break[1:], dtype=bool))[0] + 1
            seg = np.empty((2 * len(end), 3), dtype=np.float32)
            seg[0::2], seg[1::2] = pts[end - 1], pts[end]
            seg_col = np.empty((2 * len(end), 4), dtype=np.float32)
            seg_col[0::2], seg_col[1::2] = col[end - 1], col[end]
            self.trail_item.setData(pos=seg, color=seg_col)

    def append_ref(self, points_ned):
        if not points_ned:
            return
        for p in points_ned:
            self.ref_points.append((p[0], -p[1], -p[2]))
        if len(self.ref_points) >= 2:
            self.ref_item.setData(
                pos=np.array(self.ref_points, dtype=np.float32))

    def append_fix(self, points_ned):
        if not points_ned:
            return
        for p in points_ned:
            self.fix_points.append((p[0], -p[1], -p[2]))
        overflow = len(self.fix_points) - TRAIL_MAX_POINTS
        if overflow > 0:
            del self.fix_points[:overflow]
        self.fix_item.setData(pos=np.array(self.fix_points, dtype=np.float32))
        self.fix_item.setVisible(self._fix_wanted)

    def append_zupt(self, points_ned):
        if not points_ned:
            return
        for p in points_ned:
            self.zupt_points.append((p[0], -p[1], -p[2]))
        overflow = len(self.zupt_points) - TRAIL_MAX_POINTS
        if overflow > 0:
            del self.zupt_points[:overflow]
        self.zupt_item.setData(pos=np.array(self.zupt_points,
                                            dtype=np.float32))
        self.zupt_item.setVisible(self._zupt_wanted)

    def set_anchors(self, anchors):
        """anchors: {id: NED}. Redraws only when an anchor is new."""
        if len(anchors) == len(self.anchor_ids):
            return False
        for lab in self.anchor_labels:
            self.removeItem(lab)
        self.anchor_labels = []
        self.anchor_ids = sorted(anchors)
        self.anchor_points = [(anchors[i][0], -anchors[i][1], -anchors[i][2])
                              for i in self.anchor_ids]
        if self.anchor_points:
            self.anchor_item.setData(
                pos=np.array(self.anchor_points, dtype=np.float32))
        for aid, p in zip(self.anchor_ids, self.anchor_points):
            lab = gl.GLTextItem(pos=np.array(p, dtype=np.float32),
                                text=f" {aid}",
                                color=pg.mkColor(theme.T["anchor"]))
            self.addItem(lab)
            self.anchor_labels.append(lab)
        self._apply_anchor_visible()
        return True

    def _apply_anchor_visible(self):
        on = self._anchor_wanted and bool(self.anchor_points)
        self.anchor_item.setVisible(on)
        for lab in self.anchor_labels:
            lab.setVisible(on)

    def set_anchor_visible(self, on):
        self._anchor_wanted = on
        self._apply_anchor_visible()

    def clear_anchors(self):
        return self.set_anchors({})

    def set_zupt_visible(self, on):
        self._zupt_wanted = on
        # Stays hidden while empty: the placeholder point sits at the origin.
        self.zupt_item.setVisible(on and bool(self.zupt_points))

    def set_ref_visible(self, on):
        self.ref_item.setVisible(on)

    def clear_ref_fix(self):
        """Drop the reference line and the fix dots, to draw them again at
        another point of the vehicle (lever arm compensation switched)."""
        self.ref_points.clear()
        self.fix_points.clear()
        self.fix_item.setVisible(False)
        self.ref_item.setData(pos=np.zeros((2, 3), dtype=np.float32))

    def speed_scale(self):
        """Speed [m/s] at the red end of the trail colour ramp."""
        return self._speed_scale

    def set_speed_scale(self, vmax):
        """Pin the red end of the trail colour ramp to vmax [m/s], None
        for automatic: it starts at AUTO_SPEED_SCALE and widens to the
        fastest speed seen."""
        self._speed_fixed = vmax
        if vmax is not None:
            self._speed_scale = vmax
        else:
            self._speed_scale = max(
                AUTO_SPEED_SCALE, max(self.trail_speeds, default=0.0) * 1.2)
        self._redraw_trail()

    def trail_snapshot(self):
        """(NED points, breaks) of the trail drawn so far."""
        return ([(x, -y, -z) for x, y, z in self.trail_points],
                list(self.trail_break))

    def set_ghost(self, points_ned, breaks):
        """Draw a previous run's trail, NED points with the same break
        flags as append_trail(). Empty clears it."""
        seg = _break_segments(points_ned, breaks)
        self._ghost_has_data = seg is not None
        if seg is not None:
            self.ghost_item.setData(pos=seg, color=theme.rgba("ghost", 0.8))
        self.ghost_item.setVisible(self._ghost_wanted and self._ghost_has_data)

    def set_rts(self, points_ned, breaks):
        """Draw the RTS smoother's trail, NED points with the same break
        flags as append_trail(). Empty clears it."""
        seg = _break_segments(points_ned, breaks)
        self._rts_has_data = seg is not None
        if seg is not None:
            self.rts_item.setData(pos=seg, color=theme.rgba("rts", 0.9))
        self.rts_item.setVisible(self._rts_wanted and self._rts_has_data)

    def has_rts(self):
        return self._rts_has_data

    def set_rts_visible(self, on):
        self._rts_wanted = on
        self.rts_item.setVisible(on and self._rts_has_data)

    def set_ghost_visible(self, on):
        self._ghost_wanted = on
        self.ghost_item.setVisible(on and self._ghost_has_data)

    def set_fix_visible(self, on):
        self._fix_wanted = on
        # Stays hidden while empty: the placeholder point sits at the origin.
        self.fix_item.setVisible(on and bool(self.fix_points))

    def set_pose(self, pos_ned, quat, sigma3, show_model, follow):
        dx, dy, dz = pos_ned[0], -pos_ned[1], -pos_ned[2]
        self._last_obj_pos = (dx, dy, dz)
        self.position_item.setData(pos=np.array([[dx, dy, dz]]))
        if show_model and quat is not None:
            self.vehicle_item.setVisible(True)
            if self.ellipsoid_item:
                self.ellipsoid_item.setVisible(False)
            v = _rotate_ned_mesh(self.vehicle_base_verts,
                                 _quat_to_rotmat(quat))
            v[:, 0] += dx
            v[:, 1] += dy
            v[:, 2] += dz
            self.vehicle_meshdata.setVertexes(v)
            self.vehicle_item.meshDataChanged()
        else:
            self.vehicle_item.setVisible(False)
            if sigma3 is not None:
                self._update_ellipsoid(dx, dy, dz, 3 * sigma3[0],
                                       3 * sigma3[1], 3 * sigma3[2])
        if follow:
            self.setCameraPosition(pos=pg.Vector(dx, dy, dz))

    def _update_ellipsoid(self, cx, cy, cz, rx, ry, rz):
        if rx < 0.01 or ry < 0.01 or rz < 0.01:
            if self.ellipsoid_item:
                self.ellipsoid_item.setVisible(False)
            return
        verts = self._ellipsoid_base_verts.copy()
        verts[:, 0] = verts[:, 0] * rx + cx
        verts[:, 1] = verts[:, 1] * ry + cy
        verts[:, 2] = verts[:, 2] * rz + cz
        mesh_data = gl.MeshData(vertexes=verts, faces=self._ellipsoid_faces)
        if self.ellipsoid_item is None:
            self.ellipsoid_item = gl.GLMeshItem(
                meshdata=mesh_data, smooth=True,
                color=theme.rgba("ellipsoid", 0.16),
                shader='shaded', glOptions='translucent')
            self.addItem(self.ellipsoid_item)
        else:
            self.ellipsoid_item.setMeshData(meshdata=mesh_data)
            self.ellipsoid_item.setVisible(True)

    def focus_on_object(self):
        x, y, z = self._last_obj_pos
        self.setCameraPosition(pos=pg.Vector(x, y, z), distance=40)

    def fit_trail(self):
        pts = (self.trail_points or self.ref_points or self.fix_points
               or self.anchor_points)
        if not pts:
            return
        arr = np.array(pts, dtype=np.float32)
        lo, hi = arr.min(axis=0), arr.max(axis=0)
        center = (lo + hi) / 2
        extent = float(np.linalg.norm(hi - lo))
        self.setCameraPosition(pos=pg.Vector(*center),
                               distance=max(extent * 0.8, 20.0))

    def reset_trail(self):
        self.trail_points.clear()
        self.trail_speeds.clear()
        self.trail_break.clear()
        self.ref_points.clear()
        self.fix_points.clear()
        self.fix_item.setVisible(False)
        self.zupt_points.clear()
        self.zupt_item.setVisible(False)
        self._speed_scale = self._speed_fixed or AUTO_SPEED_SCALE
        z2 = np.zeros((2, 3), dtype=np.float32)
        self.trail_item.setData(pos=z2,
                                color=np.zeros((2, 4), dtype=np.float32))
        self.ref_item.setData(pos=z2)
        self.set_rts([], [])

    def mousePressEvent(self, ev):
        # Middle-button drag is GLViewWidget's own pan gesture: honour the
        # user's manual pan instead of snapping the camera back to the
        # followed object on the next update tick.
        if ev.button() == QtCore.Qt.MouseButton.MiddleButton:
            self.manual_pan_requested.emit()
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        super().mouseMoveEvent(ev)
        if (ev.buttons() == QtCore.Qt.MouseButton.LeftButton
                and not ev.modifiers()
                & QtCore.Qt.KeyboardModifier.ControlModifier):
            self._show_orbit_readout()

    def mouseReleaseEvent(self, ev):
        self.orbit_label.hide()
        super().mouseReleaseEvent(ev)

    def _show_orbit_readout(self):
        """Azimuth/elevation of the camera, only while orbiting."""
        self.orbit_label.setText(
            f"Azimuth: {self.opts['azimuth'] % 360.0:.0f}°   "
            f"Elevation: {self.opts['elevation']:.0f}°")
        self.orbit_label.adjustSize()
        self._place_orbit_label()
        self.orbit_label.show()

    def _place_orbit_label(self):
        lab = self.orbit_label
        lab.move(self.width() - lab.width() - 8,
                 self.height() - lab.height() - 8)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._place_orbit_label()

    def pan(self, dx, dy, dz, relative='global'):
        if relative != 'view-upright':
            return super().pan(dx, dy, dz, relative)
        # pyqtgraph derives the horizontal axes from cross(z, view), which
        # is zero looking straight down or up (elevation +-90) and the pan
        # dies there. The azimuth alone gives the same axes at any elevation.
        cPos = self.cameraPosition()
        dist = (self.opts['center'] - cPos).length()
        scale = (dist * 2.0 * math.tan(0.5 * math.radians(self.opts['fov']))
                 / self.width())
        az = math.radians(self.opts['azimuth'])
        c, s = math.cos(az), math.sin(az)
        self.opts['center'] = self.opts['center'] + QtGui.QVector3D(
            s * dx - c * dy, -c * dx - s * dy, dz) * scale
        self.update()

class AttitudeView(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        theme.themed(lambda: self.setBackgroundColor(theme.T["gl_bg"]))
        self.opts['distance'] = 4
        self.opts['elevation'] = 25
        self.opts['azimuth'] = 170
        for end, col in (([1.5, 0, 0], (1, 0.3, 0.3, 1)),
                         ([0, -1.5, 0], (0.3, 0.8, 0.3, 1)),
                         ([0, 0, 1.5], (0.3, 0.6, 1, 1))):
            self.addItem(gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], end]), color=col, width=2,
                glOptions=GL_BLEND))
        verts, faces, colors = _load_vehicle_mesh(scale=20.0) \
            if os.path.exists("vehicle.stl") else _make_box_mesh()
        self._base_verts = verts.copy()
        self.box_meshdata = gl.MeshData(vertexes=verts, faces=faces,
                                        faceColors=colors)
        self.box_item = gl.GLMeshItem(meshdata=self.box_meshdata,
                                      smooth=False, shader='shaded',
                                      glOptions='opaque')
        self.addItem(self.box_item)

    def update_attitude(self, q):
        self.box_meshdata.setVertexes(
            _rotate_ned_mesh(self._base_verts, _quat_to_rotmat(q)))
        self.box_item.meshDataChanged()


class GBubbleWidget(QtWidgets.QWidget):
    """Body x/y acceleration as a bubble. The outer ring is not a fixed
    range: a car or a walking pedestrian moves the bubble by a few tenths
    of a g, so the ring snaps to the smallest of SCALES_G that holds the
    recent peak, widens at once and narrows again slowly."""
    FILTER_ALPHA = 0.15
    SCALES_G = (0.2, 0.5, 1.0, 2.0, 4.0, 8.0)
    HEADROOM = 1.15      # the peak sits below the ring
    PEAK_DECAY = 0.995   # per update, so a few seconds

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(80, 80)
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.scale_g = self.SCALES_G[0]
        self._peak_g = 0.0
        theme.themed(self.update)

    def reset(self):
        """Back to the centre, e.g. when the replay is over: the bubble
        otherwise stays where the file happened to end."""
        self.acc_x = self.acc_y = self._peak_g = 0.0
        self.scale_g = self.SCALES_G[0]
        self.update()

    def set_acceleration(self, ax_g, ay_g):
        a = self.FILTER_ALPHA
        self.acc_x += a * (ax_g - self.acc_x)
        self.acc_y += a * (ay_g - self.acc_y)
        self._peak_g = max(math.hypot(self.acc_x, self.acc_y),
                           self._peak_g * self.PEAK_DECAY)
        need = self._peak_g * self.HEADROOM
        self.scale_g = next((x for x in self.SCALES_G if x >= need),
                            self.SCALES_G[-1])
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = (min(w, h) / 2 - 4) * 0.95
        p.setPen(QtGui.QPen(QtGui.QColor(theme.T["bubble_ring"]), 1))
        for frac in (0.33, 0.66, 1.0):
            p.drawEllipse(QtCore.QPointF(cx, cy), r * frac, r * frac)
        p.drawLine(QtCore.QPointF(cx - r, cy), QtCore.QPointF(cx + r, cy))
        p.drawLine(QtCore.QPointF(cx, cy - r), QtCore.QPointF(cx, cy + r))
        bx = cx + max(-1, min(1, self.acc_y / self.scale_g)) * r
        by = cy - max(-1, min(1, self.acc_x / self.scale_g)) * r
        p.setPen(QtGui.QPen(QtGui.QColor(theme.T["bubble_edge"]), 2))
        fill = QtGui.QColor(theme.T["bubble_fill"])
        fill.setAlpha(180)
        p.setBrush(QtGui.QBrush(fill))
        p.drawEllipse(QtCore.QPointF(bx, by), 5, 5)
        # The outer ring's value, since it changes.
        p.setPen(QtGui.QColor(theme.T["dim"]))
        p.drawText(QtCore.QRectF(0, 0, w - 2, h),
                   QtCore.Qt.AlignmentFlag.AlignRight
                   | QtCore.Qt.AlignmentFlag.AlignBottom,
                   f"{self.scale_g:g} g")


class TrailLegend(QtWidgets.QWidget):
    """Key to the 3D view: the speed colour ramp of the trail plus one
    swatch for every optional overlay that is switched on."""

    # kind: "line" or "dot", palette key
    OVERLAYS = {
        "ref": ("Reference", "line", "ref_trail"),
        "ghost": ("Previous run", "line", "ghost"),
        "rts": ("RTS smoother", "line", "rts"),
        "fix": ("GNSS fix", "dot", "fix"),
        "zupt": ("ZUPT/ZARU", "dot", "zupt"),
        "anchor": ("Anchor", "dot", "anchor"),
    }
    RAMP_W = 110

    def __init__(self, parent=None):
        super().__init__(parent)
        self._vmax = 5.0
        self._shown = []
        self.setFixedHeight(22)
        theme.themed(self.update)

    def set_scale(self, vmax):
        if vmax != self._vmax:
            self._vmax = vmax
            self.update()

    def set_overlays(self, keys):
        """Keys of OVERLAYS to list, in drawing order."""
        self._shown = [k for k in self.OVERLAYS if k in keys]
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        font = p.font()
        font.setPixelSize(11)
        p.setFont(font)
        fm = p.fontMetrics()
        text = QtGui.QColor(theme.T["dim"])
        mid = self.height() / 2
        x = 4.0

        def label(txt):
            nonlocal x
            p.setPen(text)
            w = fm.horizontalAdvance(txt)
            p.drawText(QtCore.QPointF(x, mid + fm.ascent() / 2 - 1), txt)
            x += w + 5

        # The overlays sit on the left, the speed ramp is right-aligned so
        # it ends up next to the "Red at" combo box that scales it.
        range_txt = f"0 … {self._vmax:.1f} m/s"
        for key in self._shown:
            name, kind, pal = self.OVERLAYS[key]
            col = QtGui.QColor(theme.T[pal])
            if kind == "line":
                p.setPen(QtGui.QPen(col, 2))
                p.drawLine(QtCore.QPointF(x, mid), QtCore.QPointF(x + 14, mid))
                x += 19
            else:
                p.setPen(QtCore.Qt.PenStyle.NoPen)
                p.setBrush(col)
                p.drawEllipse(QtCore.QPointF(x + 4, mid), 3, 3)
                x += 12
            label(name)
            x += 10
        speed_w = (fm.horizontalAdvance("Speed") + 5 + self.RAMP_W + 5
                   + fm.horizontalAdvance(range_txt))
        x = max(x, self.width() - speed_w - 4)
        label("Speed")
        ramp = QtCore.QRectF(x, mid - 4, self.RAMP_W, 8)
        grad = QtGui.QLinearGradient(ramp.left(), 0, ramp.right(), 0)
        stops = 9
        cols = _speeds_to_rgba(np.linspace(0.0, 1.0, stops), 1.0,
                               theme.T["trail_value"])
        for i, c in enumerate(cols):
            grad.setColorAt(i / (stops - 1), QtGui.QColor.fromRgbF(*c))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QBrush(grad))
        p.drawRoundedRect(ramp, 2, 2)
        x = ramp.right() + 5
        label(range_txt)


# ============================================================================
# Config editor
# ============================================================================

class _NoWheelComboBox(QtWidgets.QComboBox):
    """Combo box that never reacts to the mouse wheel, so scrolling through
    the config form cannot silently change a choice under the cursor."""

    def wheelEvent(self, event):
        event.ignore()  # lets the enclosing scroll area handle it


# Subtle tint behind the label of a field that differs from its default.
# Translucent so it works on light and dark themes.
NON_DEFAULT_STYLE = ("background-color: rgba(255, 170, 0, 55);"
                     " border-radius: 3px; padding: 0 3px;")


class ConfigEditor(QtWidgets.QScrollArea):
    """Schema-driven form over config.yaml. Keys outside the schema (e.g.
    the foreign origin:/crazyflie: sections, truth:) are preserved
    untouched."""

    changed = QtCore.pyqtSignal()   # the USER edited a field

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._raw = {}
        self._baseline = {}  # collect() right after the last load()
        self._widgets = {}  # path -> (kind, extra, widget)
        self._labels = {}  # path -> QLabel
        body = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(body)
        for section, fields in CONFIG_SECTIONS:
            box = QtWidgets.QGroupBox(section)
            form = QtWidgets.QFormLayout(box)
            for path, label, kind, extra, tip in fields:
                w = self._make_widget(kind, extra)
                lbl = QtWidgets.QLabel(label)
                if tip:
                    # Qt does not wrap plain-text tooltips, so break the
                    # line here to keep long ones on the screen
                    tip = textwrap.fill(tip, TOOLTIP_WIDTH)
                    w.setToolTip(tip)
                    lbl.setToolTip(tip)
                form.addRow(lbl, w)
                self._widgets[path] = (kind, extra, w)
                self._labels[path] = lbl
                # The user-only signals: load() sets values programmatically
                # and must not count as an edit.
                if kind == BOOL:
                    w.clicked.connect(self.changed)
                elif kind == CHOICE:
                    w.activated.connect(self.changed)
                else:
                    w.textEdited.connect(self.changed)
            lay.addWidget(box)
        lay.addStretch()
        self.setWidget(body)
        self.changed.connect(self._update_marks)
        self.load({})  # schema defaults, and the baseline for is_dirty()

    @staticmethod
    def _make_widget(kind, extra):
        if kind == BOOL:
            return QtWidgets.QCheckBox()
        if kind == CHOICE:
            cb = _NoWheelComboBox()
            cb.addItems(list(extra))
            return cb
        e = QtWidgets.QLineEdit()
        if kind in (FLOAT, INT):
            e.setMaximumWidth(180)
        return e

    @staticmethod
    def _fmt(val):
        if isinstance(val, float):
            return repr(val)  # shortest round-trip form, no precision loss
        return str(val)

    def load(self, raw):
        self._raw = copy.deepcopy(raw)
        for path, (kind, extra, w) in self._widgets.items():
            val = _get_path(raw, path)
            if val is None:
                val = _schema_default(path, kind, extra)
            if kind == BOOL:
                w.setChecked(bool(int(val)))
            elif kind == CHOICE:
                idx = w.findText(str(val))
                w.setCurrentIndex(max(idx, 0))
            elif kind == VEC:
                seq = list(val) if isinstance(val, (list, tuple)) else []
                if len(seq) != extra:
                    seq = [0.0] * extra
                w.setText(", ".join(self._fmt(float(x)) for x in seq))
            elif kind == INT:
                w.setText(str(int(val)))
            else:
                w.setText(self._fmt(val))
        self._baseline = self.collect()[0]
        self._update_marks()

    @staticmethod
    def _parse(kind, extra, w):
        """Widget content as a config value, ValueError if malformed."""
        if kind == BOOL:
            return 1 if w.isChecked() else 0
        if kind == CHOICE:
            return w.currentText()
        txt = w.text().strip()
        if kind == STR:
            return txt
        if kind == INT:
            return int(float(txt)) if txt else 0
        if kind == VEC:
            parts = [p for p in w.text().replace(",", " ").split() if p]
            val = [float(p) for p in parts] if parts else [0.0] * extra
            if len(val) != extra:
                raise ValueError(f"expected {extra} values")
            return val
        return float(txt) if txt else 0.0

    @staticmethod
    def _differs(path, kind, extra, val):
        dflt = _schema_default(path, kind, extra)
        if isinstance(dflt, (list, tuple)):
            return list(val) != [float(x) for x in dflt]
        if kind == BOOL:
            return val != int(dflt)
        return val != dflt

    def _update_marks(self):
        """Tint the label of every field that deviates from its default."""
        for path, (kind, extra, w) in self._widgets.items():
            try:
                differs = self._differs(path, kind, extra,
                                        self._parse(kind, extra, w))
            except ValueError:
                differs = True
            lbl = self._labels[path]
            lbl.setStyleSheet(NON_DEFAULT_STYLE if differs else "")

    def is_dirty(self):
        """Does the form differ from what load() put in?"""
        raw, errors = self.collect()
        return bool(errors) or raw != self._baseline

    def collect(self):
        """Returns (raw_config, errors). Values equal to the schema default
        are only written if the key was already present in the file."""
        raw = copy.deepcopy(self._raw)
        errors = []
        for path, (kind, extra, w) in self._widgets.items():
            name = ".".join(path)
            try:
                val = self._parse(kind, extra, w)
            except ValueError as e:
                errors.append(f"{name}: {e}")
                continue
            differs = self._differs(path, kind, extra, val)
            if differs or _get_path(self._raw, path) is not None:
                _set_path(raw, path, val)
        return raw, errors


# ============================================================================
# Post-run plots
# ============================================================================

def populate_plots(glw, rec, warmup_end_sec, leverarm_comp=False):
    """Fill a GraphicsLayoutWidget with the post-run evaluation plots, in
    the current theme's colours (a theme switch calls it again).
    leverarm_comp moves the reference and the GNSS fixes of the map and
    the altitude profile onto the IMU point the estimate refers to
    (REQ-VER-039), otherwise they are drawn where they were taken."""
    glw.clear()
    glw.setBackground(theme.T["bg"])
    PEN_N, PEN_E, PEN_D = (theme.pen("trace", 1.5, index=i) for i in range(3))
    PEN_REF = theme.pen("ref", 1.5, theme.DASH)
    PEN_SIG = theme.pen("sigma", 1.0, theme.DOT)
    PEN_WARM = theme.pen("warm", 1.0, theme.DASH)
    PEN_EXTRA = theme.pen("extra", 1.5)
    PEN_RTS = theme.pen("rts", 1.5)
    t = np.asarray(rec["t"], dtype=float)
    if t.size == 0:
        glw.addLabel("no recorded samples (filter never initialized?)")
        return

    def arr(key):
        return np.asarray(rec[key], dtype=float)

    pos_err = arr("pos_err_ned")
    pos_sig = arr("pos_sigma")
    rpy_err = arr("rpy_err_deg")
    rpy_sig = arr("rpy_sigma_deg")
    pos = arr("pos")
    ref_pos = arr("ref_pos")
    fix_d = arr("fix_pos_d")
    if leverarm_comp:
        ref_pos = ref_pos - arr("ref_pt_ned")
        fix_d = fix_d - arr("gnss_la_ned")[:, 2]
    gyr_bias = arr("gyr_bias")
    acc_bias = arr("acc_bias")
    gyr_bias_sig = arr("gyr_bias_sigma")
    acc_bias_sig = arr("acc_bias_sigma")
    # The RTS smoother's series (ReplayWorker._smooth), drawn next to the
    # filter's where present.
    has_rts = ("rts_pos" in rec and len(rec["rts_pos"]) == len(t)
               and np.isfinite(arr("rts_pos")).any())

    time_plots = []

    def add_time_plot(title, ylabel):
        p = glw.addPlot(title=title)
        p.showGrid(x=True, y=True, alpha=0.2)
        p.setLabel('left', ylabel)
        if time_plots:
            p.setXLink(time_plots[0])
        if warmup_end_sec > 0:
            p.addItem(pg.InfiniteLine(pos=warmup_end_sec, angle=90,
                                      pen=PEN_WARM))
        time_plots.append(p)
        return p

    # Row 1: position error + 1-sigma band
    for i, (axis, pen) in enumerate((("N", PEN_N), ("E", PEN_E),
                                     ("D", PEN_D))):
        p = add_time_plot(f"pos error {axis} [m]", "m")
        if has_rts and i == 0:
            p.addLegend(offset=(10, 10))
        p.plot(t, pos_err[:, i], pen=pen, connect="finite", name="filter")
        p.plot(t, pos_sig[:, i], pen=PEN_SIG, connect="finite")
        p.plot(t, -pos_sig[:, i], pen=PEN_SIG, connect="finite")
        if has_rts:
            p.plot(t, arr("rts_pos_err_ned")[:, i], pen=PEN_RTS,
                   connect="finite", name="RTS")
    glw.nextRow()
    # Row 2: attitude error + 1-sigma band
    for i, (axis, pen) in enumerate((("roll", PEN_N), ("pitch", PEN_E),
                                     ("yaw", PEN_D))):
        p = add_time_plot(f"{axis} error [°]", "°")
        p.plot(t, rpy_err[:, i], pen=pen, connect="finite")
        p.plot(t, rpy_sig[:, i], pen=PEN_SIG, connect="finite")
        p.plot(t, -rpy_sig[:, i], pen=PEN_SIG, connect="finite")
        if has_rts:
            p.plot(t, arr("rts_rpy_err_deg")[:, i], pen=PEN_RTS,
                   connect="finite")
    glw.nextRow()

    # Row 3: NE map, altitude profile, downweight counters
    map_plot = glw.addPlot(title="North-East map [m]")
    map_plot.showGrid(x=True, y=True, alpha=0.2)
    map_plot.setAspectLocked(True)
    map_plot.setLabel('left', 'North [m]')
    map_plot.setLabel('bottom', 'East [m]')
    map_plot.plot(ref_pos[:, 1], ref_pos[:, 0], pen=PEN_REF,
                  connect="finite", name="truth")
    map_plot.plot(pos[:, 1], pos[:, 0], pen=PEN_N, connect="finite",
                  name="estimate")
    if has_rts:
        rts_pos = arr("rts_pos")
        map_plot.plot(rts_pos[:, 1], rts_pos[:, 0], pen=PEN_RTS,
                      connect="finite", name="RTS")

    alt = add_time_plot("altitude (-D) [m]", "m")
    alt.addLegend(offset=(10, 10))
    alt_est = -pos[:, 2]
    alt_sig = pos_sig[:, 2]
    alt.plot(t, alt_est, pen=PEN_N, connect="finite", name="ins")
    alt.plot(t, alt_est + alt_sig, pen=PEN_SIG, connect="finite")
    alt.plot(t, alt_est - alt_sig, pen=PEN_SIG, connect="finite")
    alt.plot(t, -ref_pos[:, 2], pen=PEN_REF, connect="finite", name="truth")
    if has_rts:
        alt.plot(t, -arr("rts_pos")[:, 2], pen=PEN_RTS, connect="finite",
                 name="RTS")
    baro_h = arr("baro_h_d")
    if np.isfinite(baro_h).any():
        alt.plot(t, -baro_h, pen=PEN_E, connect="finite", name="baro_alt")
    if np.isfinite(fix_d).any():
        alt.plot(t, -fix_d, pen=theme.pen("extra", 1.0),
                 connect="finite", name="gnss")

    dwp = add_time_plot("chi2 downweight counters", "count")
    dwp.addLegend(offset=(10, 10))
    for key, pen, name in (("dw_full3d", PEN_N, "ins"),
                           ("dw_ars", PEN_E, "ars"),
                           ("dw_ahrs", PEN_D, "ahrs"),
                           ("dw_baro_alt", PEN_EXTRA, "baro_alt")):
        dwp.plot(t, arr(key), pen=pen, connect="finite", name=name)
    glw.nextRow()

    # Row 4: biases + 1-sigma band
    gb = add_time_plot("ins gyro bias [°/s]", "°/s")
    gb.addLegend(offset=(10, 10))
    for i, (axis, pen) in enumerate((("x", PEN_N), ("y", PEN_E),
                                     ("z", PEN_D))):
        center = np.degrees(gyr_bias[:, i])
        sigma = np.degrees(gyr_bias_sig[:, i])
        gb.plot(t, center, pen=pen, connect="finite", name=axis)
        gb.plot(t, center + sigma, pen=PEN_SIG, connect="finite")
        gb.plot(t, center - sigma, pen=PEN_SIG, connect="finite")
        if has_rts:
            gb.plot(t, np.degrees(arr("rts_gyr_bias")[:, i]),
                    pen=theme.pen("trace", 1.0, theme.DASH, index=i),
                    connect="finite")
    ab = add_time_plot("ins accel bias [m/s²]", "m/s²")
    ab.addLegend(offset=(10, 10))
    for i, (axis, pen) in enumerate((("x", PEN_N), ("y", PEN_E),
                                     ("z", PEN_D))):
        center = acc_bias[:, i]
        sigma = acc_bias_sig[:, i]
        ab.plot(t, center, pen=pen, connect="finite", name=axis)
        ab.plot(t, center + sigma, pen=PEN_SIG, connect="finite")
        ab.plot(t, center - sigma, pen=PEN_SIG, connect="finite")
        if has_rts:
            ab.plot(t, arr("rts_acc_bias")[:, i],
                    pen=theme.pen("trace", 1.0, theme.DASH, index=i),
                    connect="finite")
    vel = arr("vel")
    ref_vel = arr("ref_vel")
    sp = add_time_plot("horizontal speed [m/s]", "m/s")
    sp.addLegend(offset=(10, 10))
    sp.plot(t, np.hypot(vel[:, 0], vel[:, 1]), pen=PEN_N,
            connect="finite", name="ins")
    sp.plot(t, np.hypot(ref_vel[:, 0], ref_vel[:, 1]), pen=PEN_REF,
            connect="finite", name="truth")
    if has_rts:
        rts_vel = arr("rts_vel")
        sp.plot(t, np.hypot(rts_vel[:, 0], rts_vel[:, 1]), pen=PEN_RTS,
                connect="finite", name="RTS")

    # Row 5: GNSS fix quality, transport latency, satellites and receiver
    # integrity, where the data has them (gnss.csv's trailing columns,
    # timesync.csv), see ins_plots._gnss_quality_page
    q_t = np.asarray(rec.get("gnss_q_t") or [], dtype=float)
    q = np.asarray(rec.get("gnss_q") or [], dtype=float)
    lat_t = np.asarray(rec.get("gnss_lat_t") or [], dtype=float)
    lat = np.asarray(rec.get("gnss_lat_ms") or [], dtype=float)
    if (q.size or lat.size or rec.get("gnss_sv")
            or any(any(v) for v in (rec.get("gnss_ind") or {}).values())):
        glw.nextRow()
    if q.size:
        qp = add_time_plot("GNSS fix quality (1 fix, 2 float, 4 dgnss, 5 single)",
                           "Q")
        qp.plot(q_t, q, pen=None, symbol="o", symbolSize=3,
                symbolPen=None, symbolBrush=theme.T["fix"])
        qp.getViewBox().invertY(True)   # best (1) on top
    if lat.size:
        lp = add_time_plot("GNSS transport latency [ms]", "ms")
        lp.plot(lat_t, lat, pen=None, symbol="o", symbolSize=3,
                symbolPen=None, symbolBrush=theme.T["trace"][0])
        delay = rec.get("gnss_delay_ms")
        if delay is not None:
            lp.addItem(pg.InfiniteLine(pos=delay, angle=0, pen=PEN_REF))
    # Satellites used and the receiver's interference/authentication
    # states (gnss.csv num_sv, jamming, spoofing, sig_auth, nav_auth)
    sv_t = np.asarray(rec.get("gnss_sv_t") or [], dtype=float)
    sv = np.asarray(rec.get("gnss_sv") or [], dtype=float)
    ind = {k: v for k, v in (rec.get("gnss_ind") or {}).items() if any(v)}
    if sv.size:
        svp = add_time_plot("GNSS satellites used", "n")
        svp.plot(sv_t, sv, pen=PEN_N)
    if ind:
        ip = add_time_plot("GNSS integrity (jamming/spoofing 1 none, 2 warning,"
                           " 3 critical, auth 1 no, 2 yes, 3 failed)", "state")
        ip.addLegend(offset=(10, 10))
        ind_t = np.asarray(rec.get("gnss_ind_t") or [], dtype=float)
        for i, (key, vals) in enumerate(ind.items()):
            ip.plot(ind_t, np.asarray(vals, dtype=float), name=key,
                    pen=theme.pen("trace", 1.0, index=i))

    for p in time_plots:
        p.setLabel('bottom', 't [s]')


# ============================================================================
# Main window
# ============================================================================

UI_UPDATE_HZ = 30
SOLUTION_HZ = 25.0  # Export solution rate [Hz of sim time]
SPEED_SCALE_PRESETS = (1, 2, 5, 10, 20, 40, 100)  # m/s, trail colour ramp
GRID_SPACING_PRESETS = (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000)  # m
GRID_TARGET_CELLS = 8  # auto: cells between the camera and the centre
GRID_SPAN_FACTOR = 6.0  # grid edge length in camera distances
GRID_MAX_CELLS = 400  # per axis, bounds the line count at fine spacings
# Time constant of the low-pass on the attitude shown (wall clock, so the
# smoothing does not depend on how often the UI timer manages to fire).
ATTITUDE_SMOOTH_TAU_S = 0.08
GRAVITY = 9.80665

MONO = "font-family: Consolas, 'DejaVu Sans Mono', monospace; font-size: 12px;"


def findings_digest(findings, min_sev=replay.SEV_INFO):
    """(worst severity, text) for an insdoctor findings list, counting
    only findings of at least min_sev, e.g. (SEV_WARN, "1 warning, 2 info")."""
    names = ((replay.SEV_CRIT, "critical", "critical"),
             (replay.SEV_WARN, "warning", "warnings"),
             (replay.SEV_INFO, "info", "info"))
    parts = []
    for sev, one, many in names:
        n = sum(1 for s, _ in findings if s == sev)
        if n and sev >= min_sev:
            parts.append(f"{n} {one if n == 1 else many}")
    worst = max((s for s, _ in findings), default=replay.SEV_OK)
    return worst, ", ".join(parts) or "all nominal"


class MainWindow(QtWidgets.QMainWindow):
    TAB_REPLAY, TAB_CONFIG, TAB_PLOTS, TAB_MAP, TAB_SUMMARY = range(5)

    def __init__(self, settings=None):
        super().__init__()
        self.setWindowTitle("inspostgui — INSLIB post-processing")
        self.resize(1700, 980)
        pg.setConfigOptions(antialias=ANTIALIAS)

        self.settings = settings or QtCore.QSettings("zwiener", "inspostgui")
        # Before anything is built: every widget takes its colours from the
        # palette in force when it registers with theme.themed().
        theme.set_theme(self.settings.value("theme", "dark"))
        theme.themed(lambda: self.setStyleSheet(theme.stylesheet()))
        self.data_dir = None
        self.cfg_path = None
        self.worker = None
        self.results = None
        self._drained = 0
        self._fix_drained = 0
        # First rec sample / fix still on screen after "Clear".
        self._shown_from = 0
        self._fix_shown_from = 0
        self._alt_t, self._alt_est, self._alt_ref = [], [], []
        self._alt_has_finite = False
        self._trail_gap = False
        self._smooth_quat = None
        # The previous run, kept as a ghost for the next one (see _on_run).
        self._ghost = None
        self._ghost_placed = False
        self._run_dir = None
        self._run_cfg = None
        self._run_unsaved = False
        # Replays stopped by opening another dataset, still winding down.
        self._stale_workers = []
        self._smooth_t = None
        self._title_name = None

        self._build_toolbar()
        self._build_central()
        self._build_statusbar()
        self._build_shortcuts()
        self._populate_datasets()
        geometry = self.settings.value("geometry")
        if isinstance(geometry, QtCore.QByteArray):
            self.restoreGeometry(geometry)

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._update_ui)
        self.ui_timer.start(int(1000 / UI_UPDATE_HZ))

    # ------------------------------------------------------------------
    def _build_toolbar(self):
        tb = QtWidgets.QToolBar("main")
        tb.setMovable(False)
        self.addToolBar(tb)
        tb.addWidget(QtWidgets.QLabel(" Dataset: "))
        self.dataset_combo = QtWidgets.QComboBox()
        self.dataset_combo.setMinimumWidth(320)
        self.dataset_combo.activated.connect(self._on_dataset_selected)
        tb.addWidget(self.dataset_combo)
        browse = QtWidgets.QPushButton("Browse…")
        browse.setToolTip("Open a dataset config (Ctrl+O)")
        browse.clicked.connect(self._on_browse)
        tb.addWidget(browse)
        tb.addSeparator()

        self.run_btn = QtWidgets.QPushButton("▶  Run")
        self.run_btn.setObjectName("primary")  # green, see ins_gui_theme
        self.run_btn.setToolTip("Replay the dataset (F5)")
        self.run_btn.clicked.connect(self._on_run)
        tb.addWidget(self.run_btn)
        self.pause_btn = QtWidgets.QPushButton("⏸ Pause")
        self.pause_btn.setToolTip("Pause / resume the replay (Space)")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setEnabled(False)
        self.pause_btn.toggled.connect(self._on_pause)
        tb.addWidget(self.pause_btn)
        self.stop_btn = QtWidgets.QPushButton("■ Stop")
        self.stop_btn.setToolTip("Stop the replay, keeps the partial "
                                 "results (Esc)")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._on_stop)
        tb.addWidget(self.stop_btn)
        tb.addSeparator()

        self.chk_realtime = QtWidgets.QCheckBox("Realtime")
        self.chk_realtime.setToolTip("Pace the replay to wall-clock time "
                                     "(x speed factor); off = full speed")
        tb.addWidget(self.chk_realtime)
        # Fixed factors in a combo box: the stylesheet's frame on a spin box
        # drops the native up/down buttons, which then overlap the value.
        self.speed_combo = QtWidgets.QComboBox()
        self.speed_combo.setToolTip("Realtime speed factor")
        for f in (0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0):
            self.speed_combo.addItem(f"x{f:g}", f)
        self.speed_combo.setCurrentIndex(self.speed_combo.findData(1.0))
        self.speed_combo.setSizeAdjustPolicy(
            QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        self.speed_combo.setEnabled(False)
        self.chk_realtime.toggled.connect(self.speed_combo.setEnabled)
        tb.addWidget(self.speed_combo)
        tb.addSeparator()

        tb.addWidget(QtWidgets.QLabel(" GNSS outage: "))
        self.outage_edit = QtWidgets.QLineEdit()
        self.outage_edit.setPlaceholderText("start:duration")
        self.outage_edit.setToolTip(
            "Simulated GNSS outage, like replay.py --gnss-outage: every fix "
            "in START .. START+DURATION seconds (counted from the first IMU "
            "sample) is cut from the input before the run. Several windows "
            "separated by commas, e.g. 60:30, 200:45. Empty → no outage. "
            "Only with aiding: gnss")
        self.outage_edit.setMaximumWidth(140)
        tb.addWidget(self.outage_edit)
        tb.addSeparator()

        self.chk_smooth = QtWidgets.QCheckBox("RTS smoother")
        self.chk_smooth.setToolTip(
            "After the replay, run the offline RTS smoother "
            "(tools/ins_rts.py, like replay.py --smooth): a second forward "
            "pass in which ins never stops dead reckoning, so the smoother "
            "can bridge an outage the real-time filter gives up on, then "
            "the backward pass. Shown as its own trail, in the plots and in "
            "the summary. Its forward pass records every epoch and takes "
            "several times as long as the replay itself.")
        # Not remembered: it costs several times the replay, so every start
        # begins with it off and a user has to ask for it.
        tb.addWidget(self.chk_smooth)
        tb.addSeparator()

        self.pdf_btn = QtWidgets.QPushButton("Export PDF")
        self.pdf_btn.setToolTip("Multi-page ins_plots PDF (same pages as "
                                "replay.py --plot --plot-out)")
        self.pdf_btn.setEnabled(False)
        self.pdf_btn.clicked.connect(self._on_export_pdf)
        tb.addWidget(self.pdf_btn)
        self.kml_btn = QtWidgets.QPushButton("Export KML")
        self.kml_btn.setToolTip("Google Earth KMZ via ins_kml (needs "
                                "simplekml)")
        self.kml_btn.setEnabled(False)
        self.kml_btn.clicked.connect(self._on_export_kml)
        tb.addWidget(self.kml_btn)
        self.sol_btn = QtWidgets.QPushButton("Export solution")
        self.sol_btn.setToolTip(
            "The filter's solution as a file (like replay.py --dump-solution): "
            "CSV in the ref.csv format plus 1-sigma columns, or an RTKLIB "
            ".pos (GPS time from the dataset's timesync.csv, without one "
            "guessed from the system clock, the file says so). "
            "With the RTS smoother on, its solution goes next to it as "
            "<name>_rts.")
        self.sol_btn.setEnabled(False)
        self.sol_btn.clicked.connect(self._on_export_solution)
        tb.addWidget(self.sol_btn)
        tb.addSeparator()

        tb.addWidget(QtWidgets.QLabel(" Theme: "))
        self.theme_combo = QtWidgets.QComboBox()
        for name in ("dark", "light"):
            self.theme_combo.addItem(name.capitalize(), name)
        self.theme_combo.setCurrentIndex(
            self.theme_combo.findData(theme.name()))
        self.theme_combo.currentIndexChanged.connect(self._on_theme_changed)
        tb.addWidget(self.theme_combo)
        tb.addSeparator()
        help_btn = QtWidgets.QPushButton("?")
        help_btn.setToolTip("Keyboard and mouse shortcuts (F1)")
        help_btn.setFixedWidth(28)
        help_btn.clicked.connect(self._on_help)
        tb.addWidget(help_btn)

    def _build_central(self):
        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        # --- Replay tab ---
        replay_w = QtWidgets.QWidget()
        root = QtWidgets.QHBoxLayout(replay_w)
        root.setContentsMargins(6, 6, 6, 6)
        left_w = QtWidgets.QWidget()
        left = QtWidgets.QVBoxLayout(left_w)
        left.setContentsMargins(0, 0, 0, 0)
        self.pos_view = Position3DView()
        left.addWidget(self.pos_view, 1)
        legend_row = QtWidgets.QHBoxLayout()
        self.legend = TrailLegend()
        legend_row.addWidget(self.legend, 1)
        legend_row.addWidget(QtWidgets.QLabel("Red at:"))
        # Editable, so any value can be typed besides the presets. A combo
        # box rather than a spin box for the reason given at speed_combo.
        self.scale_combo = QtWidgets.QComboBox()
        self.scale_combo.setEditable(True)
        self.scale_combo.setInsertPolicy(
            QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.scale_combo.setToolTip(
            "Speed that the trail colours as red (blue is standstill). "
            "auto: starts small and widens to the fastest speed seen. "
            "Type a number [m/s] for a fixed scale, e.g. 2 for a small "
            "UAV or 40 for a car.")
        self.scale_combo.addItems(
            ["auto"] + [f"{v:g} m/s" for v in SPEED_SCALE_PRESETS])
        self.scale_combo.setMinimumWidth(100)
        self.scale_combo.activated.connect(self._on_speed_scale_edited)
        self.scale_combo.lineEdit().editingFinished.connect(
            self._on_speed_scale_edited)
        legend_row.addWidget(self.scale_combo)
        legend_row.addWidget(QtWidgets.QLabel("Grid:"))
        self.grid_combo = QtWidgets.QComboBox()
        self.grid_combo.setToolTip(
            "Spacing of the 3D view's ground grid. auto: follows the zoom "
            "in 1-2-5 steps. A fixed value keeps the lines at that "
            "distance, e.g. to measure by eye.")
        self.grid_combo.addItems(
            ["auto"] + [f"{v:g} m" for v in GRID_SPACING_PRESETS])
        self.grid_combo.activated.connect(self._on_grid_spacing_selected)
        legend_row.addWidget(self.grid_combo)
        self.lbl_grid = QtWidgets.QLabel("")
        self.lbl_grid.setMinimumWidth(60)
        self.pos_view.grid_spacing_changed.connect(
            lambda sp: self.lbl_grid.setText(f"{sp:g} m"))
        legend_row.addWidget(self.lbl_grid)
        left.addLayout(legend_row)
        saved_grid = self.settings.value("grid_spacing", "auto")
        self.grid_combo.setCurrentText(
            saved_grid if self.grid_combo.findText(saved_grid) >= 0
            else "auto")
        self._on_grid_spacing_selected()
        saved = self.settings.value("speed_scale", "auto")
        self.scale_combo.setCurrentText(
            saved if self._parse_speed_scale(saved) is not None
            or saved == "auto" else "auto")
        self._on_speed_scale_edited()
        bottom = QtWidgets.QHBoxLayout()
        self.chk_follow = QtWidgets.QCheckBox("Follow")
        self.chk_follow.setChecked(True)
        bottom.addWidget(self.chk_follow)
        self.chk_model = QtWidgets.QCheckBox("Show 3D model")
        bottom.addWidget(self.chk_model)
        self.pos_view.manual_pan_requested.connect(
            lambda: self.chk_follow.setChecked(False))
        self.chk_ref = QtWidgets.QCheckBox("Reference")
        self.chk_ref.setToolTip("ref.csv (or inputs: ref) as the green "
                                "line: the ground truth the score uses. Also on the Map tab.")
        self.chk_ghost = QtWidgets.QCheckBox("Previous run")
        self.chk_ghost.setToolTip("The previous run of this dataset as a "
                                  "grey trail, to compare a config change "
                                  "against")
        self.chk_rts = QtWidgets.QCheckBox("RTS")
        self.chk_rts.setToolTip("The RTS smoother's solution of this run "
                                "(toolbar: RTS smoother) as its own trail, "
                                "here and on the Map tab")
        self.chk_fix = QtWidgets.QCheckBox("GNSS fixes")
        self.chk_fix.setToolTip("gnss.csv (or inputs: gnss) as yellow dots, "
                                "one per fix, whether the filter fused it "
                                "or not. With aiding: ref these are the "
                                "fixes synthesized from the reference. Also on the Map tab.")
        self.chk_zupt = QtWidgets.QCheckBox("ZUPT/ZARU")
        self.chk_zupt.setToolTip("Dots on the trail wherever a zero-velocity "
                                 "or zero-angular-rate update was applied "
                                 "(INS auto-ZUPT, baro_alt vertical ZUPT, "
                                 "ARS/AHRS ZARU). A standstill shows as a "
                                 "cluster. Needs the 3D solution, the trail "
                                 "is the ins position.")
        self.chk_anchor = QtWidgets.QCheckBox("Anchors")
        self.chk_anchor.setToolTip("The ranging anchors (ranges.csv) as dots "
                                   "with their id, shown from the first "
                                   "range to the anchor on.")
        # The previous run is off until switched on: a grey trail nobody
        # asked for reads as part of the current result. It has its own
        # settings key, so a remembered "on" from before the default
        # changed does not carry over.
        for chk, key, default, apply in (
                (self.chk_ref, "show_ref", True,
                 self._set_ref_visible),
                (self.chk_fix, "show_fix", True,
                 self._set_fix_visible),
                (self.chk_zupt, "show_zupt", False,
                 self.pos_view.set_zupt_visible),
                (self.chk_anchor, "show_anchors", True,
                 self.pos_view.set_anchor_visible),
                (self.chk_ghost, "show_previous_run", False,
                 self._set_ghost_visible),
                (self.chk_rts, "show_rts", True,
                 self._set_rts_visible)):
            on = self.settings.value(key, default, type=bool)
            chk.setChecked(on)
            apply(on)
            chk.toggled.connect(apply)
            chk.toggled.connect(
                lambda c, k=key: self.settings.setValue(k, c))
            chk.toggled.connect(self._update_legend)
            bottom.addWidget(chk)
        self._update_legend()
        self.chk_leverarm = QtWidgets.QCheckBox("Lever arm compensated")
        self.chk_leverarm.setToolTip(
            "Draw the reference and the GNSS fixes at the IMU point, where "
            "the estimate is: the reference shifted back by "
            "score.leverarm_frd, each fix by gnss.leverarm_frd, rotated "
            "with the filter's attitude. Off: both where they were taken. "
            "The position error and the score always compensate.")
        self.chk_leverarm.setChecked(
            self.settings.value("leverarm_comp", True, type=bool))
        self.chk_leverarm.toggled.connect(
            lambda c: self.settings.setValue("leverarm_comp", c))
        self.chk_leverarm.toggled.connect(self._on_leverarm_comp_toggled)
        bottom.addWidget(self.chk_leverarm)
        # GNSS vs. scoring lever arm of the config on screen, see
        # _update_leverarm_check.
        self.lbl_leverarm = QtWidgets.QLabel("")
        theme.themed(self._style_leverarm_check)
        bottom.addWidget(self.lbl_leverarm)
        bottom.addStretch()
        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.setToolTip("Clear the trail, the reference, the GNSS fixes "
                             "and the ZUPT/ZARU dots of the 3D view and the "
                             "altitude profile. A running replay keeps "
                             "drawing from here on.")
        clear_btn.clicked.connect(self._on_clear_trail)
        bottom.addWidget(clear_btn)
        fit_btn = QtWidgets.QPushButton("Fit trail")
        fit_btn.clicked.connect(self.pos_view.fit_trail)
        bottom.addWidget(fit_btn)
        focus_btn = QtWidgets.QPushButton("Focus object")
        focus_btn.clicked.connect(self.pos_view.focus_on_object)
        bottom.addWidget(focus_btn)
        left.addLayout(bottom)
        # The side panel's width is the user's call: the 3D view takes
        # what is left, the divider position is remembered.
        self.replay_split = QtWidgets.QSplitter(
            QtCore.Qt.Orientation.Horizontal)
        self.replay_split.setChildrenCollapsible(False)
        self.replay_split.addWidget(left_w)
        self.replay_split.addWidget(self._build_side_panel())
        self.replay_split.setStretchFactor(0, 1)
        self.replay_split.setStretchFactor(1, 0)
        self.replay_split.setSizes([1280, 420])
        state = self.settings.value("replay_splitter")
        if isinstance(state, QtCore.QByteArray):
            self.replay_split.restoreState(state)
        root.addWidget(self.replay_split)
        self.tabs.addTab(replay_w, "Replay")

        # --- Config tab ---
        cfg_w = QtWidgets.QWidget()
        cfg_lay = QtWidgets.QVBoxLayout(cfg_w)
        row = QtWidgets.QHBoxLayout()
        self.cfg_label = QtWidgets.QLabel("no config loaded")
        theme.themed(lambda: self.cfg_label.setStyleSheet(theme.dim()))
        row.addWidget(self.cfg_label, 1)
        for text, slot in (("Reload", self._on_cfg_reload),
                           ("New…", self._on_cfg_new),
                           ("Save", self._on_cfg_save),
                           ("Save As…", self._on_cfg_save_as)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(slot)
            row.addWidget(b)
        cfg_lay.addLayout(row)
        self.files_label = QtWidgets.QLabel("")
        theme.themed(lambda: self.files_label.setStyleSheet(
            theme.dim() + " " + MONO))
        cfg_lay.addWidget(self.files_label)
        self.editor = ConfigEditor()
        self.editor.changed.connect(self._refresh_cfg_ui)
        cfg_lay.addWidget(self.editor, 1)
        self.tabs.addTab(cfg_w, "Config")

        # --- Plots tab ---
        plots_scroll = QtWidgets.QScrollArea()
        plots_scroll.setWidgetResizable(True)
        self.plots_widget = pg.GraphicsLayoutWidget()
        theme.themed(
            lambda: self.plots_widget.setBackground(theme.T["bg"]))
        self.plots_widget.setMinimumHeight(1100)
        plots_scroll.setWidget(self.plots_widget)
        self.tabs.addTab(plots_scroll, "Plots")

        # --- Map tab ---
        self.map_view = MapView()
        self.map_view.set_ref_visible(self.chk_ref.isChecked())
        self.map_view.set_fix_visible(self.chk_fix.isChecked())
        self.map_view.set_ghost_visible(self.chk_ghost.isChecked())
        self.map_view.set_rts_visible(self.chk_rts.isChecked())
        self.tabs.addTab(self.map_view, "Map")

        # --- Summary tab ---
        self.summary_text = QtWidgets.QPlainTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setStyleSheet(MONO)
        self.tabs.addTab(self.summary_text, "Summary")

    def _build_side_panel(self):
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(340)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)

        def frame(title, stretch=0):
            f = QtWidgets.QFrame()
            f.setObjectName("panel")
            v = QtWidgets.QVBoxLayout(f)
            h = QtWidgets.QLabel(title)
            theme.themed(lambda h=h: h.setStyleSheet(
                theme.dim() + " font-size: 11px;"))
            v.addWidget(h)
            lay.addWidget(f, stretch)
            return v, f

        v, _ = frame("Status")
        self.lbl_status = QtWidgets.QLabel("idle")
        self.lbl_status.setStyleSheet(MONO)
        self.lbl_status.setWordWrap(True)
        v.addWidget(self.lbl_status)
        self.lbl_mode = QtWidgets.QLabel("")
        self.lbl_mode.setStyleSheet(MONO)
        v.addWidget(self.lbl_mode)

        # The outcome of the last run. Own frame: the Status labels above
        # are rewritten by the live view on every tick.
        v, self.result_frame = frame("Last run")
        self.lbl_result = QtWidgets.QLabel()
        self.lbl_result.setStyleSheet(MONO)
        self.lbl_result.setWordWrap(True)
        v.addWidget(self.lbl_result)
        self.lbl_findings = QtWidgets.QLabel()
        self.lbl_findings.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.lbl_findings.setToolTip("insdoctor findings, click for the "
                                     "Summary tab")
        self.lbl_findings.mousePressEvent = (
            lambda _ev: self.tabs.setCurrentIndex(self.TAB_SUMMARY))
        v.addWidget(self.lbl_findings)
        theme.themed(self._style_findings)
        self.result_frame.setVisible(False)

        v, _ = frame("Attitude / Bias (± 1σ)")
        h = QtWidgets.QHBoxLayout()
        num = QtWidgets.QVBoxLayout()
        self.lbl_roll = QtWidgets.QLabel()
        self.lbl_pitch = QtWidgets.QLabel()
        self.lbl_yaw = QtWidgets.QLabel()
        self.lbl_gyr_bias = QtWidgets.QLabel()
        self.lbl_acc_bias = QtWidgets.QLabel()
        for lbl in (self.lbl_roll, self.lbl_pitch, self.lbl_yaw):
            lbl.setStyleSheet(MONO)
            num.addWidget(lbl)
        num.addSpacing(6)
        for lbl in (self.lbl_gyr_bias, self.lbl_acc_bias):
            lbl.setStyleSheet(MONO)
            num.addWidget(lbl)
        num.addStretch()
        h.addLayout(num, 0)
        # Reserve the size of the filled-in text up front: empty labels
        # would let the 3D view start large and shrink on the first tick,
        # and "n/a" would shrink it again. The placeholder is measured,
        # then cleared, so no NaNs show before the first result.
        nan = math.nan
        placeholder = {
            self.lbl_roll: f"Roll:  {nan:+7.1f}° ± {nan:5.2f}°",
            self.lbl_gyr_bias: _value_sigma_rows(
                "Gyro [°/s]", (nan,) * 3, None, 3, 6),
            self.lbl_acc_bias: _value_sigma_rows(
                "Accel [m/s²]", (nan,) * 3, None, 3, 6)}
        for lbl, text in placeholder.items():
            lbl.setText(text)
            lbl.setMinimumSize(lbl.sizeHint())
            lbl.setText("")
        for lbl in (self.lbl_pitch, self.lbl_yaw):
            lbl.setMinimumSize(self.lbl_roll.minimumSize())
        self.attitude_view = AttitudeView()
        self.attitude_view.setMinimumSize(140, 160)
        h.addWidget(self.attitude_view, 1)
        v.addLayout(h)

        v, _ = frame("Position / Velocity (± 1σ)")
        self.lbl_pos = QtWidgets.QLabel()
        self.lbl_llh = QtWidgets.QLabel()
        self.lbl_baro = QtWidgets.QLabel()
        self.lbl_vel = QtWidgets.QLabel()
        for lbl in (self.lbl_pos, self.lbl_llh, self.lbl_baro, self.lbl_vel):
            lbl.setStyleSheet(MONO)
            lbl.setWordWrap(True)
            v.addWidget(lbl)

        v, _ = frame("Altitude (-D) [m]", stretch=1)
        self.alt_plot = pg.PlotWidget()
        theme.themed(lambda: theme.style_plot(self.alt_plot))
        self.alt_plot.showGrid(x=True, y=True, alpha=0.2)
        self.alt_plot.setMinimumHeight(140)
        self.alt_curve_est = self.alt_plot.plot()
        self.alt_curve_ref = self.alt_plot.plot()
        theme.themed(lambda: (
            self.alt_curve_est.setPen(theme.pen("est", 2)),
            self.alt_curve_ref.setPen(theme.pen("ref", 1.5, theme.DASH))))
        v.addWidget(self.alt_plot)

        v, _ = frame("Acceleration (body)")
        h = QtWidgets.QHBoxLayout()
        col = QtWidgets.QVBoxLayout()
        self.lbl_acc = QtWidgets.QLabel()
        self.lbl_dw = QtWidgets.QLabel()
        self.lbl_dw.setWordWrap(True)
        for lbl in (self.lbl_acc, self.lbl_dw):
            lbl.setStyleSheet(MONO)
            col.addWidget(lbl)
        self.chk_dynacc = QtWidgets.QCheckBox("Gravity removed")
        self.chk_dynacc.setToolTip(
            "On: the motion's acceleration, the specific force with the "
            "gravity the attitude estimate predicts taken out and the "
            "accelerometer bias corrected. Level and unaccelerated it "
            "reads zero. Off: the raw accelerometer, which also shows the "
            "tilt (1 g times sin of pitch or roll) and reads 1 g at rest.")
        self.chk_dynacc.setChecked(
            self.settings.value("bubble_dynamic", True, type=bool))
        self.chk_dynacc.toggled.connect(
            lambda c: self.settings.setValue("bubble_dynamic", c))
        self.chk_dynacc.toggled.connect(lambda _c: self.g_bubble.reset())
        col.addWidget(self.chk_dynacc)
        col.addStretch()
        h.addLayout(col, 1)
        self.g_bubble = GBubbleWidget()
        h.addWidget(self.g_bubble)
        v.addLayout(h)

        return panel

    def _build_statusbar(self):
        self.progress = QtWidgets.QProgressBar()
        self.progress.setMaximumWidth(300)
        self.progress.setRange(0, 100)
        self.statusBar().addPermanentWidget(self.progress)
        self.statusBar().showMessage("ready")

    def _build_shortcuts(self):
        def add(keys, slot):
            sc = QtGui.QShortcut(QtGui.QKeySequence(keys), self)
            sc.activated.connect(slot)

        add("F5", self._on_run)
        add("Space", self._on_space)
        add("Esc", self._on_esc)
        add("Ctrl+O", self._on_browse)
        add("Ctrl+S", self._on_cfg_save)
        add("Ctrl+Shift+S", self._on_cfg_save_as)
        add("F1", self._on_help)
        for i in range(self.tabs.count()):
            add(f"Ctrl+{i + 1}", lambda i=i: self.tabs.setCurrentIndex(i))

    @staticmethod
    def _motion_acc(f, rpy, bias):
        """Body acceleration without gravity from the accelerometer's
        specific force f: a = f - bias + g_body, where g_body is gravity
        (down in NED) rotated into the body frame by the estimated roll and
        pitch. None without an attitude."""
        if rpy is None:
            return None
        sr, cr = math.sin(rpy[0]), math.cos(rpy[0])
        sp, cp = math.sin(rpy[1]), math.cos(rpy[1])
        b = bias or (0.0, 0.0, 0.0)
        return (f[0] - b[0] - GRAVITY * sp,
                f[1] - b[1] + GRAVITY * sr * cp,
                f[2] - b[2] + GRAVITY * cr * cp)

    def _on_help(self):
        """Keyboard and mouse cheat sheet."""
        tab_rows = tuple((f"Ctrl+{i + 1}", f"Tab {self.tabs.tabText(i)}")
                         for i in range(self.tabs.count()))

        def table(title, rows):
            body = "".join(f"<tr><td style='padding-right:18px'>"
                           f"<b>{k}</b></td><td>{d}</td></tr>"
                           for k, d in rows)
            return f"<h4>{title}</h4><table>{body}</table>"

        html = (
            table("Keyboard", (
                ("F5", "Run the replay"),
                ("Space", "Pause / resume"),
                ("Esc", "Stop, keeps the partial results"),
                ("Ctrl+O", "Open a config"),
                ("Ctrl+S / Ctrl+Shift+S", "Save the config / save as"),
                ) + tab_rows + (("F1", "This help"),)) +
            table("3D view (Replay tab)", (
                ("Left drag", "Orbit, azimuth and elevation show while "
                              "dragging"),
                ("Ctrl + left drag", "Pan in the screen plane"),
                ("Middle drag", "Pan over the ground (switches Follow off)"),
                ("Ctrl + middle drag", "Pan horizontally and up/down"),
                ("Wheel", "Zoom"),
                ("Ctrl + wheel", "Field of view"))) +
            table("Map and plots", (
                ("Left drag", "Pan"),
                ("Wheel", "Zoom"),
                ("Right drag", "Zoom each axis separately"),
                ("Right click", "Plot menu, e.g. export the plot"),
                ("A (button, lower left)", "Auto-range, show everything"))))
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Shortcuts")
        lay = QtWidgets.QVBoxLayout(dlg)
        head = QtWidgets.QHBoxLayout()
        logo = QtGui.QPixmap(LOGO)
        if not logo.isNull():
            pic = QtWidgets.QLabel()
            pic.setPixmap(logo.scaledToHeight(
                72, QtCore.Qt.TransformationMode.SmoothTransformation))
            head.addWidget(pic)
        about = QtWidgets.QLabel(
            "<h2 style='margin:0'>INSLIB</h2>Jan Zwiener<br>"
            "<a href='https://github.com/jnz/INSLIB'>"
            "github.com/jnz/INSLIB</a>")
        about.setTextFormat(QtCore.Qt.TextFormat.RichText)
        about.setOpenExternalLinks(True)
        head.addWidget(about)
        head.addStretch()
        lay.addLayout(head)
        lbl = QtWidgets.QLabel(html)
        lbl.setTextFormat(QtCore.Qt.TextFormat.RichText)
        lay.addWidget(lbl)
        close = QtWidgets.QPushButton("Close")
        close.clicked.connect(dlg.accept)
        lay.addWidget(close, 0, QtCore.Qt.AlignmentFlag.AlignRight)
        dlg.exec()

    def _on_space(self):
        # The shortcut swallows the key everywhere, so hand it on to what
        # has the focus: a button is pressed, a combo box opens.
        fw = QtWidgets.QApplication.focusWidget()
        if isinstance(fw, QtWidgets.QComboBox):
            fw.showPopup()
        elif (isinstance(fw, QtWidgets.QAbstractButton)
              and fw is not self.pause_btn):
            fw.click()
        elif self.pause_btn.isEnabled():
            self.pause_btn.toggle()

    def _on_esc(self):
        if self.stop_btn.isEnabled():
            self._on_stop()

    @staticmethod
    def _parse_speed_scale(text):
        """Speed [m/s] out of the combo's text ("12", "12 m/s"), None for
        "auto" and for anything that is not a positive number."""
        try:
            v = float(str(text).replace("m/s", "").replace(",", ".").strip())
        except ValueError:
            return None
        return v if math.isfinite(v) and v > 0.0 else None

    def _on_grid_spacing_selected(self, *_):
        text = self.grid_combo.currentText()
        self.settings.setValue("grid_spacing", text)
        self.pos_view.set_grid_spacing(
            None if text == "auto" else float(text.replace("m", "")))
        self.lbl_grid.setText(f"{self.pos_view.grid_spacing():g} m")

    def _on_speed_scale_edited(self, *_):
        text = self.scale_combo.currentText().strip()
        vmax = self._parse_speed_scale(text)
        if vmax is None and text.lower() != "auto":
            self.scale_combo.setCurrentText("auto")  # unusable input
            text = "auto"
        elif vmax is not None:
            self.scale_combo.setCurrentText(f"{vmax:g} m/s")
        self.settings.setValue("speed_scale",
                               "auto" if vmax is None else f"{vmax:g} m/s")
        self.pos_view.set_speed_scale(vmax)
        self.legend.set_scale(self.pos_view.speed_scale())

    def _update_legend(self, *_):
        shown = [key for key, chk in (("ref", self.chk_ref),
                                      ("ghost", self.chk_ghost),
                                      ("rts", self.chk_rts),
                                      ("fix", self.chk_fix),
                                      ("zupt", self.chk_zupt),
                                      ("anchor", self.chk_anchor))
                 if chk.isChecked()
                 and (key != "anchor" or self.pos_view.anchor_points)
                 and (key != "rts" or self.pos_view.has_rts())]
        self.legend.set_overlays(shown)

    def _style_findings(self):
        sev = getattr(self, "_findings_sev", replay.SEV_OK)
        colour = (theme.T["trace"][0] if sev >= replay.SEV_CRIT
                  else theme.T["warm"] if sev == replay.SEV_WARN
                  else theme.T["dim"])
        self.lbl_findings.setStyleSheet(
            f"{MONO} color: {colour}; text-decoration: underline;")

    # ------------------------------------------------------------------
    def _populate_datasets(self):
        self.dataset_combo.clear()
        for path in self._recents():
            self.dataset_combo.addItem(config_label(path), path)
        self.dataset_combo.setCurrentIndex(-1)

    def _recents(self):
        """Recently opened config files that still exist. Entries from
        before the GUI opened files were directories, read as their
        config.yaml."""
        out = []
        for path in self.settings.value("recents", [], type=list):
            if os.path.isdir(path):
                path = os.path.join(path, "config.yaml")
            if os.path.isfile(path) and path not in out:
                out.append(path)
        return out

    def _remember_recent(self, cfg_path):
        recents = self._recents()
        cfg_path = os.path.abspath(cfg_path)
        if cfg_path in recents:
            recents.remove(cfg_path)
        recents.insert(0, cfg_path)
        self.settings.setValue("recents", recents[:10])

    def _select_in_combo(self, cfg_path):
        cfg_path = os.path.abspath(cfg_path)
        idx = next((i for i in range(self.dataset_combo.count())
                    if os.path.abspath(self.dataset_combo.itemData(i) or "")
                    == cfg_path), -1)
        if idx < 0:
            self.dataset_combo.addItem(config_label(cfg_path), cfg_path)
            idx = self.dataset_combo.count() - 1
        self.dataset_combo.setCurrentIndex(idx)

    def open_dataset(self, path):
        """`path`: a config YAML (it need not exist yet, see New…) or a
        dataset directory, meaning its config.yaml. False if nothing was
        opened: unreadable, or the user kept the unsaved edits."""
        if not self._confirm_discard("Open another config anyway?"):
            return False
        try:
            raw, cfg_path, data_dir = load_raw_config(path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error",
                                           f"Could not load config:\n{e}")
            return False
        if not looks_like_config(raw):
            keys = ", ".join(str(k) for k in list(raw)[:6])
            if QtWidgets.QMessageBox.question(
                    self, "Not an INSLIB config?",
                    f"{cfg_path}\nhas none of the config sections "
                    f"(top-level keys: {keys}).\nOpen it anyway?",
                    QtWidgets.QMessageBox.StandardButton.Yes
                    | QtWidgets.QMessageBox.StandardButton.No,
                    QtWidgets.QMessageBox.StandardButton.No
                    ) != QtWidgets.QMessageBox.StandardButton.Yes:
                return False
        # Another config of the same dataset keeps the run on screen: it
        # becomes the ghost the next run is compared against.
        if (self.data_dir is not None
                and os.path.normpath(data_dir)
                != os.path.normpath(self.data_dir)):
            self._clear_for_other_dataset()
        self.data_dir = data_dir
        self.cfg_path = cfg_path
        self.editor.load(raw)
        self._title_name = (raw.get("name")
                            or os.path.basename(os.path.normpath(data_dir)))
        self._refresh_cfg_ui()
        self._update_files_label(raw)
        if os.path.exists(cfg_path):
            self._remember_recent(cfg_path)
        self.statusBar().showMessage(
            f"loaded {self._title_name} ({cfg_path})")
        self._select_in_combo(cfg_path)
        return True

    def _refresh_cfg_ui(self):
        """Config label, Config tab and window title: path, NEW and the
        unsaved-changes marker."""
        if not self.cfg_path:
            return
        self._update_leverarm_check()
        dirty = self.editor.is_dirty()
        exists = os.path.exists(self.cfg_path)
        self.cfg_label.setText(
            f"{self.cfg_path}{'' if exists else '  (NEW, not saved yet)'}"
            f"{'  *modified*' if dirty else ''}")
        self.tabs.setTabText(self.TAB_CONFIG,
                             "Config *" if dirty else "Config")
        self.setWindowTitle(
            f"inspostgui — {self._title_name} "
            f"[{os.path.basename(self.cfg_path)}]{' *' if dirty else ''}")

    def _update_leverarm_check(self):
        """Next to the compensation checkbox: does the scoring lever arm
        match the GNSS one (REQ-VER-039)? Same verdict as the insdoctor
        finding, but from the config on screen, before any run."""
        raw, errors = self.editor.collect()
        if errors:
            self.lbl_leverarm.setText("")
            self.lbl_leverarm.setToolTip("")
            return
        spec = merge_spec(raw)
        g = list(spec["gnss"]["leverarm_frd"])
        sc = list(spec["score"]["leverarm_frd"])
        text, tip, warn = {
            "none": ("", "", False),
            "same": ("lever arms: GNSS = scoring", "", False),
            "score_unset": (
                "⚠ scoring lever arm not set",
                f"gnss.leverarm_frd is {g} but score.leverarm_frd is zero: "
                "the reference is taken as the IMU point. Set it to the GNSS "
                "lever arm when ref.csv is the receiver's own solution, "
                "otherwise the whole lever arm counts as position error.",
                True),
            "gnss_unset": (
                "scoring lever arm only",
                f"score.leverarm_frd is {sc}, gnss.leverarm_frd is zero: "
                "the fixes are fused as if taken at the IMU.", False),
            "differ": (
                "⚠ lever arms: GNSS ≠ scoring",
                f"gnss.leverarm_frd {g} vs. score.leverarm_frd {sc}. Right "
                "when ref.csv refers to another point than the GNSS antenna, "
                "a typo when it is the receiver's own solution.", True),
        }[replay.leverarm_relation(g, sc)]
        self.lbl_leverarm.setText(text)
        self.lbl_leverarm.setToolTip(tip)
        self._leverarm_warn = warn
        self._style_leverarm_check()

    def _style_leverarm_check(self):
        self.lbl_leverarm.setStyleSheet(
            f"color: {theme.T['warm']};"
            if getattr(self, "_leverarm_warn", False) else theme.dim())

    def _on_leverarm_comp_toggled(self, *_):
        """Redraw the reference and the fixes of the 3D view and the
        altitude profile, and the post-run plots, at the newly chosen
        point."""
        w = self.worker
        self.pos_view.clear_ref_fix()
        if w is not None:
            with w.lock:
                ref = w.rec["ref_pos"][self._shown_from:self._drained]
                ref_la = w.rec["ref_pt_ned"][self._shown_from:self._drained]
                fix = w.fix_ned[self._fix_shown_from:self._fix_drained]
                fix_la = w.fix_la_ned[self._fix_shown_from:self._fix_drained]
            shown = [self._ref_shown(r, d) for r, d in zip(ref, ref_la)]
            self.pos_view.append_ref([p for p in shown if p[0] == p[0]])
            self.pos_view.append_fix([self._ref_shown(p, d)
                                      for p, d in zip(fix, fix_la)])
            self._alt_ref = [-p[2] if p[2] == p[2] else math.nan
                             for p in shown]
            if self._alt_has_finite:
                self.alt_curve_ref.setData(self._alt_t, self._alt_ref,
                                           connect="finite")
        if self.results:
            populate_plots(self.plots_widget, self.results["rec"],
                           self.results.get("warmup_end_sec", 0.0),
                           self.chk_leverarm.isChecked())

    def _on_clear_trail(self):
        """Empty the 3D view (ghost aside) and the altitude profile, a
        running replay keeps drawing from the next sample on."""
        self.pos_view.reset_trail()
        self._update_legend()
        self.legend.set_scale(self.pos_view.speed_scale())
        self._shown_from = self._drained
        self._fix_shown_from = self._fix_drained
        self._alt_t, self._alt_est, self._alt_ref = [], [], []
        self._alt_has_finite = False
        self._trail_gap = False
        self.alt_curve_est.clear()
        self.alt_curve_ref.clear()

    def _ref_shown(self, p, la_n):
        """A reference or fix position (NED) where the 3D view draws it:
        moved by its rotated lever arm la_n onto the IMU point while the
        compensation is on, as taken otherwise."""
        if not self.chk_leverarm.isChecked():
            return p
        return [a - b for a, b in zip(p, la_n)]

    def _confirm_discard(self, question):
        """True if it is fine to drop the config form's state: nothing
        edited, or the user saved or discarded the edits."""
        if not self.editor.is_dirty():
            return True
        Btn = QtWidgets.QMessageBox.StandardButton
        ret = QtWidgets.QMessageBox.question(
            self, "Unsaved config changes",
            f"The config has unsaved changes.\n{question}",
            Btn.Save | Btn.Discard | Btn.Cancel, Btn.Cancel)
        if ret == Btn.Save:
            self._on_cfg_save()
            return not self.editor.is_dirty()  # cancelled or invalid
        return ret == Btn.Discard

    def _update_files_label(self, raw):
        """The CSVs the config points at: inputs: overrides resolved against
        the config's directory, like the replay does."""
        if not self.data_dir:
            self.files_label.setText("")
            return
        spec = {"inputs": raw.get("inputs") or {}}
        parts = []
        for stream in ("imu", "ref", "gnss", "heading", "mag", "baro",
                       "speed", "ranges"):
            p = replay.input_path(self.data_dir, spec, stream)
            fname = os.path.basename(p)
            if os.path.exists(p):
                parts.append(f"{fname} ({os.path.getsize(p) // 1024} kB)")
            else:
                parts.append(f"{fname} —")
        self.files_label.setText("dataset files:  " + "   ".join(parts))

    def _on_dataset_selected(self, idx):
        path = self.dataset_combo.itemData(idx)
        if path and not self.open_dataset(path):
            # the combo already moved on to the pick: put it back
            if self.cfg_path:
                self._select_in_combo(self.cfg_path)
            else:
                self.dataset_combo.setCurrentIndex(-1)

    def _on_browse(self):
        start = self.data_dir or os.getcwd()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open dataset config (YAML next to the CSVs)", start,
            "Config YAML (*.yaml *.yml);;All files (*)")
        if path:
            self.open_dataset(path)

    # --- config buttons ------------------------------------------------
    def _on_cfg_reload(self):
        if self.cfg_path:
            self.open_dataset(self.cfg_path)

    def _on_cfg_new(self):
        start = os.path.join(self.data_dir or os.getcwd(), "config.yaml")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "New config (in the dataset directory with the CSVs)",
            start, "Config YAML (*.yaml *.yml)",
            options=QtWidgets.QFileDialog.Option.DontConfirmOverwrite)
        if not path:
            return
        if os.path.exists(path):
            QtWidgets.QMessageBox.information(
                self, "Exists", f"{os.path.basename(path)} already exists "
                "-- loading it instead.")
        self.open_dataset(path)

    def _save_config(self, path):
        raw, errors = self.editor.collect()
        if errors:
            QtWidgets.QMessageBox.warning(self, "Invalid values",
                                          "\n".join(errors))
            return
        import yaml
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(raw, f, sort_keys=False)
        self.cfg_path = path
        # The config's own directory is where its CSVs are looked up, so a
        # Save As into another directory moves the dataset with it.
        self.data_dir = os.path.dirname(os.path.abspath(path))
        self.editor.load(raw)  # new baseline for "already present" keys
        self._refresh_cfg_ui()
        self._update_files_label(raw)
        self._remember_recent(path)
        self._select_in_combo(path)
        self.statusBar().showMessage(f"saved {path}")

    def _on_cfg_save(self):
        if not self.cfg_path:
            self._on_cfg_save_as()
            return
        if os.path.exists(self.cfg_path):
            ret = QtWidgets.QMessageBox.question(
                self, "Overwrite?",
                f"Overwrite {self.cfg_path}?\nYAML comments in the file "
                f"will be lost (generated configs carry documentation).",
                QtWidgets.QMessageBox.StandardButton.Yes
                | QtWidgets.QMessageBox.StandardButton.No)
            if ret != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        self._save_config(self.cfg_path)

    def _on_cfg_save_as(self):
        start = self.cfg_path or os.path.join(self.data_dir or ".",
                                              "config.yaml")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save config as", start, "YAML files (*.yaml *.yml)")
        if path:
            self._save_config(path)

    # --- run control ----------------------------------------------------
    def _on_run(self):
        if self.worker is not None and self.worker.isRunning():
            return
        if not self.data_dir:
            QtWidgets.QMessageBox.information(self, "No dataset",
                                              "Select a dataset first.")
            return
        raw, errors = self.editor.collect()
        if errors:
            QtWidgets.QMessageBox.warning(self, "Invalid config values",
                                          "\n".join(errors))
            return
        spec = merge_spec(raw)
        errors = validate_spec(spec, self.data_dir)
        outages = []
        try:
            outages = replay_core.parse_gnss_outages([self.outage_edit.text()])
        except replay_core.ReplayError as e:
            errors.append(str(e).replace("--gnss-outage", "GNSS outage"))
        if errors:
            QtWidgets.QMessageBox.warning(self, "Cannot run",
                                          "\n".join(errors))
            return

        self._run_unsaved = self.editor.is_dirty()
        self._keep_ghost()
        self._reset_run_view()

        self.worker = ReplayWorker(spec, self.data_dir,
                                   realtime=self.chk_realtime.isChecked(),
                                   speed=self.speed_combo.currentData(),
                                   gnss_outages=outages,
                                   smooth=self.chk_smooth.isChecked())
        self.worker.sig_status.connect(self.statusBar().showMessage)
        self.worker.sig_progress.connect(self.progress.setValue)
        self.worker.sig_finished.connect(self._on_finished)
        self.worker.sig_error.connect(self._on_worker_error)
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.pause_btn.setEnabled(True)
        self.pause_btn.setChecked(False)
        self.worker.start()

    def _reset_run_view(self):
        """Empty everything a run draws (the ghost aside, see _keep_ghost):
        3D trail, anchors, altitude profile, plots, summary, exports."""
        self.results = None
        self._drained = 0
        self._fix_drained = 0
        # First rec sample / fix still on screen after "Clear".
        self._shown_from = 0
        self._fix_shown_from = 0
        self._alt_t, self._alt_est, self._alt_ref = [], [], []
        self._alt_has_finite = False
        self._trail_gap = False
        self._smooth_quat = None
        self._smooth_t = None
        self.pos_view.reset_trail()
        self.pos_view.clear_anchors()
        self._update_legend()
        self.legend.set_scale(self.pos_view.speed_scale())
        self.plots_widget.clear()
        self.summary_text.setPlainText("")
        self.result_frame.setVisible(False)
        self.tabs.setTabText(self.TAB_SUMMARY, "Summary")
        self.pdf_btn.setEnabled(False)
        self.kml_btn.setEnabled(False)
        self.sol_btn.setEnabled(False)
        self.progress.setValue(0)

    def _clear_for_other_dataset(self):
        """A different dataset was opened: nothing on screen belongs to it.
        A replay still running is stopped and cut loose, else it would go
        on drawing the old dataset into the emptied views."""
        w = self.worker
        if w is not None and w.isRunning():
            w.stop()
            for sig in (w.sig_status, w.sig_progress, w.sig_finished,
                        w.sig_error):
                sig.disconnect()
            self._stale_workers = [x for x in self._stale_workers
                                   if x.isRunning()] + [w]
            self._run_ended()
            self.progress.setValue(0)
        self.worker = None
        self._ghost = None
        self._run_dir = None
        self.pos_view.set_ghost([], [])
        self._reset_run_view()
        self.alt_curve_est.clear()
        self.alt_curve_ref.clear()
        self.map_view.clear()

    def _keep_ghost(self):
        """Turn the run on screen into the ghost of the one about to start.
        Only within one dataset directory (config variants included): a
        trail of another dataset is no comparison."""
        if self._run_dir != self.data_dir:
            self._ghost = None
        elif self.results is not None:
            points, breaks = self.pos_view.trail_snapshot()
            self._ghost = {
                "points": points, "breaks": breaks,
                "origin": self.results.get("origin_ecef"),
                "rms": (self.results["pos_rms_m"]
                        if self.results["scored_epochs"] else None),
                "cfg": self._run_cfg,
                "est_latlon": [(r[1], r[2]) for r in self.results["kml_est"]],
            }
        # else: the last run failed, the ghost before it stays.
        self._run_dir = self.data_dir
        self._run_cfg = self.cfg_path
        self._ghost_placed = False
        if self._ghost:
            self.pos_view.set_ghost(self._ghost["points"],
                                    self._ghost["breaks"])
        else:
            self.pos_view.set_ghost([], [])

    def _place_ghost(self, origin_ecef):
        """Move the ghost into this run's local frame once its origin is
        known (a config change can move the bootstrap point)."""
        self._ghost_placed = True
        g = self._ghost
        if not g or g["origin"] is None:
            return
        if max(abs(a - b) for a, b in zip(g["origin"], origin_ecef)) < 1e-3:
            return
        g["points"] = _move_ned_origin(g["points"], g["origin"], origin_ecef)
        g["origin"] = tuple(origin_ecef)
        self.pos_view.set_ghost(g["points"], g["breaks"])

    def _set_ref_visible(self, on):
        self.pos_view.set_ref_visible(on)
        if hasattr(self, "map_view"):  # built after the Replay tab
            self.map_view.set_ref_visible(on)

    def _set_fix_visible(self, on):
        self.pos_view.set_fix_visible(on)
        if hasattr(self, "map_view"):  # built after the Replay tab
            self.map_view.set_fix_visible(on)

    def _set_rts_visible(self, on):
        self.pos_view.set_rts_visible(on)
        if hasattr(self, "map_view"):  # built after the Replay tab
            self.map_view.set_rts_visible(on)

    def _set_ghost_visible(self, on):
        self.pos_view.set_ghost_visible(on)
        if hasattr(self, "map_view"):  # built after the Replay tab
            self.map_view.set_ghost_visible(on)

    def _on_pause(self, checked):
        if self.worker is not None:
            self.worker.set_paused(checked)
            if checked:
                self.statusBar().showMessage("paused")

    def _on_stop(self):
        if self.worker is not None:
            self.worker.stop()
            self.pause_btn.setChecked(False)

    def _on_finished(self, results):
        self.results = results
        self._run_ended()
        self.summary_text.setPlainText(results["text"])
        populate_plots(self.plots_widget, results["rec"],
                       results.get("warmup_end_sec", 0.0),
                       self.chk_leverarm.isChecked())
        rts = results.get("rts")
        self.map_view.set_tracks(
            [(row[1], row[2]) for row in results["kml_est"]],
            [(row[0], row[1]) for row in results["kml_ref"]],
            results["fix_latlon"],
            self._ghost["est_latlon"] if self._ghost else (),
            [(row[1], row[2]) for row in rts["kml_est"]] if rts is not None else ())
        rts = results.get("rts")
        if rts is not None:
            self.pos_view.set_rts(rts["trail"], rts["breaks"])
            self._update_legend()
        self.pdf_btn.setEnabled(True)
        self.kml_btn.setEnabled(bool(results["kml_est"]))
        self.sol_btn.setEnabled(len(results["solution"]) > 0)
        rms = results["pos_rms_m"]
        prev = self._ghost["rms"] if self._ghost else None
        worst, digest = findings_digest(results["findings"])
        msg = (f"done: {results['name']}, mode {results['mode']}"
               + (f", pos rms {rms:.2f} m ({results['scored_epochs']} "
                  f"scored epochs)" if results["scored_epochs"] else "")
               + (f", previous run {prev:.2f} m"
                  + (f" ({os.path.basename(self._ghost['cfg'])})"
                     if self._ghost["cfg"] != self.cfg_path else "")
                  if prev is not None else "")
               + (f", RTS pos rms {rts['rts']['pos_rms']:.2f} m"
                  if rts is not None and rts["rts"].get("n") else "")
               + (f", findings: {digest}" if worst > replay.SEV_OK else "")
               + (" [STOPPED]" if results["aborted"] else ""))
        self.statusBar().showMessage(msg)
        self._show_run_result(results, prev, worst, digest)
        if worst >= replay.SEV_WARN:
            _, bad = findings_digest(results["findings"], replay.SEV_WARN)
            self.tabs.setTabText(self.TAB_SUMMARY, f"Summary ({bad})")
        self.tabs.setCurrentIndex(self.TAB_REPLAY)

    def _show_run_result(self, results, prev, worst, digest):
        """The "Last run" panel: score, previous run, findings."""
        lines = []
        if results["scored_epochs"]:
            lines.append(f"pos rms {results['pos_rms_m']:.2f} m   "
                         f"max {results['pos_max_m']:.2f} m   "
                         f"({results['scored_epochs']} epochs)")
            if results["att_rms_deg"] is not None:
                r, p, y = results["att_rms_deg"]
                lines.append(f"att rms r/p/y {r:.2f} / {p:.2f} / {y:.2f}\u00b0")
        else:
            lines.append("no scored epochs (no reference)")
        rts = results.get("rts")
        if rts is not None and rts["rts"].get("n"):
            s = rts["rts"]
            lines.append(f"RTS pos rms {s['pos_rms']:.2f} m   "
                         f"max {s['pos_max']:.2f} m   ({s['n']} epochs)")
            r, p, y = (math.hypot(*s[k][:2]) for k in ("roll", "pitch", "yaw"))
            lines.append(f"RTS att rms r/p/y {r:.2f} / {p:.2f} / {y:.2f}°")
        if prev is not None and results["scored_epochs"]:
            delta = results["pos_rms_m"] - prev
            other = (f" ({os.path.basename(self._ghost['cfg'])})"
                     if self._ghost["cfg"] != self.cfg_path else "")
            lines.append(f"previous {prev:.2f} m{other}   "
                         f"\u0394 {delta:+.2f} m")
        if results["aborted"]:
            lines.append("STOPPED, partial results")
        if self._run_unsaved:
            lines.append("ran with unsaved config edits")
        self.lbl_result.setText("\n".join(lines))
        self._findings_sev = worst
        self.lbl_findings.setText(f"findings: {digest}")
        self._style_findings()
        self.result_frame.setVisible(True)

    def _on_worker_error(self, text, details):
        self._run_ended()
        self.statusBar().showMessage("replay failed")
        box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Icon.Critical,
                                    "Replay failed", text,
                                    QtWidgets.QMessageBox.StandardButton.Ok,
                                    self)
        if details:
            box.setDetailedText(details)  # the traceback, on demand
        box.exec()

    def _run_ended(self):
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.pause_btn.setEnabled(False)
        self.progress.setValue(100)
        self.g_bubble.reset()

    # --- exports ---------------------------------------------------------
    def _on_export_pdf(self):
        if not self.results:
            return
        default = os.path.join(self.data_dir or ".",
                               f"{self.results['name']}_plots.pdf")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export ins_plots PDF", default, "PDF files (*.pdf)")
        if not path:
            return
        try:
            QtWidgets.QApplication.setOverrideCursor(
                QtCore.Qt.CursorShape.WaitCursor)
            from ins_plots import plot_results
            plot_results(self.results["rec"], self.results["name"],
                         self.results["warmup_sec"], out_path=path,
                         gnss_delay_curve=self.results["gnss_delay_curve"],
                         sensor_rate=self.results["sensor_rate"],
                         findings=self.results["findings"],
                         configured_gnss_delay_ms=self.results["gnss_delay_ms"],
                         growth_rate=self.results["growth_rate"],
                         baro_growth_rate=self.results["baro_growth_rate"],
                         ahrs_growth_rate=self.results["ahrs_growth_rate"])
            self.statusBar().showMessage(f"wrote {path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "PDF export failed", str(e))
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

    def _on_export_kml(self):
        if not self.results:
            return
        default = os.path.join(self.data_dir or ".",
                               f"{self.results['name']}.kml")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Google Earth KML", default, "KML files (*.kml)")
        if not path:
            return
        try:
            from ins_kml import write_kml
            rts = self.results.get("rts")
            write_kml(path, self.results["kml_est"], self.results["kml_ref"],
                      self.results["kml_fix"], self.results["name"],
                      rts_track=rts["kml_est"] if rts is not None else ())
            self.statusBar().showMessage(f"wrote {path}")
        except ImportError:
            QtWidgets.QMessageBox.warning(
                self, "Missing dependency",
                "KML export needs simplekml: pip install simplekml")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "KML export failed", str(e))

    def _on_export_solution(self):
        if not self.results:
            return
        default = os.path.join(self.data_dir or ".",
                               f"{self.results['name']}_solution.csv")
        path, chosen = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export solution", default,
            "CSV, ref.csv format + 1-sigma (*.csv);;RTKLIB solution (*.pos)")
        if not path:
            return
        # The filter picks the format when the name has no extension of
        # its own.
        if not os.path.splitext(path)[1]:
            path += ".pos" if "*.pos" in chosen else ".csv"
        try:
            written = self.write_solution(path)
            self.statusBar().showMessage("wrote " + ", ".join(written))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Solution export failed", str(e))

    def write_solution(self, path):
        """Write the last run's solution to `path` (format by extension,
        ins_solution.write), and the RTS smoother's next to it as
        <name>_rts when it ran. Returns the paths written."""
        res = self.results
        clock = res.get("gps_clock")
        rts = res.get("rts")
        if clock is None:
            # The same guess for both files, so they stay comparable, from
            # whichever of the two has an epoch.
            firsts = [sol.t_us[0] for sol in (res["solution"],
                                               rts["solution"] if rts else None)
                      if sol is not None and len(sol)]
            if firsts:
                clock = ins_solution.guessed_clock(min(firsts))
        ins_solution.write(path, res["solution"], clock,
                           comment="INSLIB nav_suite solution, inspostgui.py")
        written = [path]
        if rts is not None:
            rts_path = ins_solution.companion_path(path, "_rts")
            ins_solution.write(rts_path, rts["solution"], clock,
                               comment="INSLIB RTS smoother solution, inspostgui.py")
            written.append(rts_path)
        return written

    def _on_theme_changed(self):
        name = self.theme_combo.currentData()
        self.settings.setValue("theme", name)
        theme.set_theme(name)
        # The post-run plots are built with the palette of their moment,
        # rebuilding them is simpler than chasing every curve.
        if self.results:
            populate_plots(self.plots_widget, self.results["rec"],
                           self.results.get("warmup_end_sec", 0.0),
                           self.chk_leverarm.isChecked())

    # --- live UI ----------------------------------------------------------
    def _update_ui(self):
        w = self.worker
        if w is None:
            return
        with w.lock:
            n = len(w.rec["t"])
            new_t = w.rec["t"][self._drained:n]
            new_pos = w.rec["pos"][self._drained:n]
            new_vel = w.rec["vel"][self._drained:n]
            new_ref = [self._ref_shown(r, d) for r, d in zip(
                w.rec["ref_pos"][self._drained:n],
                w.rec["ref_pt_ned"][self._drained:n])]
            new_fix = [self._ref_shown(p, d) for p, d in zip(
                w.fix_ned[self._fix_drained:],
                w.fix_la_ned[self._fix_drained:])]
            new_zupt = [bool(a or b or c) for a, b, c in zip(
                w.rec["zupt_active"][self._drained:n],
                w.rec["ars_zaru_applied"][self._drained:n],
                w.rec["ahrs_zaru_applied"][self._drained:n])]
            live = dict(w.live)
            origin_now = w.origin_ecef
            anchors = dict(w.anchor_ned)
        self._drained = n
        self._fix_drained += len(new_fix)
        self.pos_view.append_fix(new_fix)
        if self.pos_view.set_anchors(anchors):
            self._update_legend()
        if not self._ghost_placed and origin_now is not None:
            self._place_ghost(origin_now)

        est_pts, est_speeds, est_breaks, ref_pts = [], [], [], []
        zupt_pts = []
        for ts, p, v, r, z in zip(new_t, new_pos, new_vel, new_ref,
                                  new_zupt):
            if p[0] == p[0]:  # not NaN
                est_pts.append(p)
                if z:
                    zupt_pts.append(p)
                est_speeds.append(math.sqrt(sum(x * x for x in v))
                                  if v[0] == v[0] else 0.0)
                # A run of unavailable samples (the filter re-arming after
                # an outage, say) separates two runs of the trail. The
                # altitude curve gets the same treatment for free through
                # its connect="finite" below, which is why the NaN is
                # appended there instead of being skipped.
                est_breaks.append(self._trail_gap)
                self._trail_gap = False
                self._alt_t.append(ts)
                self._alt_est.append(-p[2])
                self._alt_ref.append(-r[2] if r[2] == r[2] else math.nan)
                self._alt_has_finite = True
            else:
                self._trail_gap = True
                self._alt_t.append(ts)
                self._alt_est.append(math.nan)
                self._alt_ref.append(-r[2] if r[2] == r[2] else math.nan)
            if r[0] == r[0]:
                ref_pts.append(r)
        self.pos_view.append_trail(est_pts, est_speeds, est_breaks)
        self.legend.set_scale(self.pos_view.speed_scale())
        self.pos_view.append_zupt(zupt_pts)
        self.pos_view.append_ref(ref_pts)
        # Guarded on ever having seen a finite sample, not just on new_t:
        # a curve made of nothing but leading NaN (e.g. the pre-bootstrap
        # stretch, replayed faster than the UI drains it) has crashed the
        # GPU-accelerated PlotWidget backend on some drivers.
        if new_t and self._alt_has_finite:
            self.alt_curve_est.setData(self._alt_t, self._alt_est,
                                       connect="finite")
            self.alt_curve_ref.setData(self._alt_t, self._alt_ref,
                                       connect="finite")

        if not live:
            return
        if live.get("quat") is not None:
            q = live["quat"]
            now = time.monotonic()
            if self._smooth_quat is None or self._smooth_t is None:
                self._smooth_quat = q
            else:
                alpha = 1.0 - math.exp(-(now - self._smooth_t)
                                       / ATTITUDE_SMOOTH_TAU_S)
                self._smooth_quat = self._slerp(self._smooth_quat, q, alpha)
            self._smooth_t = now
            self.attitude_view.update_attitude(self._smooth_quat)
        if live.get("pos") is not None:
            self.pos_view.set_pose(live["pos"],
                                   self._smooth_quat,
                                   live.get("pos_sigma"),
                                   self.chk_model.isChecked(),
                                   self.chk_follow.isChecked())
        self.lbl_status.setText(
            f"t = {live['t_sec']:7.1f} / {live['duration']:.1f} s"
            f"{'   [ZUPT]' if live['zupt'] else ''}")
        self.lbl_mode.setText(f"mode: {live['mode']}"
                              f"{'  (ready)' if live['ready'] else ''}")
        rpy, rpy_sigma = live.get("rpy"), live.get("rpy_sigma")
        if rpy is not None:
            rs = rpy_sigma or (math.nan, math.nan, math.nan)
            self.lbl_roll.setText(f"Roll:  {math.degrees(rpy[0]):+7.1f}° "
                                  f"± {math.degrees(rs[0]):5.2f}°")
            self.lbl_pitch.setText(f"Pitch: {math.degrees(rpy[1]):+7.1f}° "
                                   f"± {math.degrees(rs[1]):5.2f}°")
            self.lbl_yaw.setText(f"Yaw:   {math.degrees(rpy[2]):+7.1f}° "
                                 f"± {math.degrees(rs[2]):5.2f}°")
        gyr_bias = live.get("gyr_bias")
        if gyr_bias is not None:
            self.lbl_gyr_bias.setText(_value_sigma_rows(
                "Gyro [°/s]", [math.degrees(x) for x in gyr_bias],
                [math.degrees(x) for x in live.get("gyr_bias_sigma") or ()],
                3, 6))
        else:
            self.lbl_gyr_bias.setText("Gyro: n/a")
        acc_bias = live.get("acc_bias")
        if acc_bias is not None:
            self.lbl_acc_bias.setText(_value_sigma_rows(
                "Accel [m/s²]", acc_bias, live.get("acc_bias_sigma"), 3, 6))
        else:
            self.lbl_acc_bias.setText("Accel: n/a")
        pos, vel, llh = live.get("pos"), live.get("vel"), live.get("llh")
        if pos is not None:
            self.lbl_pos.setText(_value_sigma_rows(
                "NED [m]", pos, live.get("pos_sigma"), 2, 9, label_w=9))
        if llh is not None:
            self.lbl_llh.setText(f"Lat {llh[0]:11.7f}°  Lon {llh[1]:11.7f}°"
                                 f"  Alt {llh[2]:7.1f} m")
        baro_h = live.get("baro_h_d")
        baro_h_sigma = live.get("baro_h_sigma")
        gnss_off = live.get("local_gnss_offset")
        gnss_off_sigma = live.get("local_gnss_offset_sigma")
        # Ellipsoid height of the datum origin against the local height
        # reference: the barometer, else ins under local position aiding.
        # nav_suite runs no offset filter under GNSS alone (N/A then).
        off_label = "Baro→ell." if baro_h is not None else "Local→ell."
        rows = []
        for label, val, sig in (("Baro alt", baro_h, baro_h_sigma),
                                (off_label, gnss_off, gnss_off_sigma)):
            if val is None:
                rows.append(f"{label + ':':<13}N/A")
            else:
                rows.append(f"{label + ':':<13}{val:+8.2f} ± {sig:.2f} m")
        self.lbl_baro.setText("\n".join(rows))
        if vel is not None:
            speed = math.sqrt(sum(x * x for x in vel))
            self.lbl_vel.setText(
                _value_sigma_rows("Vel [m/s]", vel, live.get("vel_sigma"),
                                  2, 7, label_w=9) + "\n"
                f"|v| = {speed:5.2f} m/s  ({speed * 3.6:5.1f} km/h)")
        acc = live.get("acc")
        if acc is not None:
            if self.chk_dynacc.isChecked():
                acc = self._motion_acc(acc, live.get("rpy"),
                                       live.get("acc_bias"))
            if acc is not None:
                g = math.sqrt(sum(x * x for x in acc)) / GRAVITY
                self.lbl_acc.setText(f"|a| = {g:5.2f} g")
                self.g_bubble.set_acceleration(acc[0] / GRAVITY,
                                               acc[1] / GRAVITY)
        dw = live.get("dw")
        if dw:
            self.lbl_dw.setText(f"downweighted: ins {dw['full3d']}, "
                                f"ars {dw['ars']}, ahrs {dw['ahrs']}, "
                                f"baro {dw['baro_alt']}")

    @staticmethod
    def _slerp(q0, q1, t):
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = tuple(-x for x in q1)
            dot = -dot
        if dot > 0.9995:
            q = tuple(a + t * (b - a) for a, b in zip(q0, q1))
            mag = math.sqrt(sum(x * x for x in q))
            return tuple(x / mag for x in q)
        theta0 = math.acos(min(dot, 1.0))
        theta = theta0 * t
        s0 = math.cos(theta) - dot * math.sin(theta) / math.sin(theta0)
        s1 = math.sin(theta) / math.sin(theta0)
        return tuple(s0 * a + s1 * b for a, b in zip(q0, q1))

    def closeEvent(self, event):
        if not self._confirm_discard("Quit anyway?"):
            event.ignore()
            return
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("replay_splitter",
                               self.replay_split.saveState())
        for w in [self.worker] + self._stale_workers:
            if w is not None and w.isRunning():
                w.stop()
                # The dataset load has no stop check. Destroying a QThread
                # that still runs aborts the process, so as a last resort cut
                # it off.
                if not w.wait(3000):
                    w.terminate()
                    w.wait(1000)
        self.map_view.shutdown()
        super().closeEvent(event)


# ============================================================================
# Entry point
# ============================================================================

def run_batch(dataset, gnss_outages=()):
    """Headless replay + summary printout (worker smoke test)."""
    app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
    raw, cfg_path, data_dir = load_raw_config(dataset)
    if not os.path.exists(cfg_path):
        sys.exit(f"{cfg_path} missing")
    spec = merge_spec(raw)
    errors = validate_spec(spec, data_dir)
    if errors:
        sys.exit("cannot run:\n  " + "\n  ".join(errors))
    worker = ReplayWorker(spec, data_dir, gnss_outages=gnss_outages)
    done = {}
    worker.sig_status.connect(print)
    worker.sig_error.connect(
        lambda msg, details: done.update(error=details or msg))
    worker.sig_finished.connect(lambda res: done.update(results=res))
    worker.run()  # synchronous, current thread
    app.processEvents()
    if "error" in done:
        sys.exit(done["error"])
    print()
    print(done["results"]["text"])
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", nargs="?", default=None,
                    help="config YAML to open at startup, or a dataset "
                         "directory (opens its config.yaml)")
    ap.add_argument("--batch", action="store_true",
                    help="headless replay + summary printout, no GUI")
    ap.add_argument("--gnss-outage", action="append", default=[],
                    metavar="START:DURATION",
                    help="with --batch: drop every GNSS fix in this window, "
                         "like replay.py --gnss-outage. Repeatable")
    ap.add_argument("--theme", choices=sorted(theme.THEMES),
                    help="colour theme (remembered for the next start, "
                         "also switchable in the toolbar)")
    args = ap.parse_args()
    if args.theme:
        QtCore.QSettings("zwiener", "inspostgui").setValue("theme",
                                                            args.theme)

    if args.batch:
        if not args.dataset:
            sys.exit("--batch needs a dataset argument")
        try:
            outages = replay_core.parse_gnss_outages(args.gnss_outage)
        except replay_core.ReplayError as e:
            sys.exit(str(e))
        sys.exit(run_batch(args.dataset, outages))

    taskbar_identity()
    app = QtWidgets.QApplication(sys.argv)
    icon = app_icon()
    if icon is not None:
        app.setWindowIcon(icon)
    win = MainWindow()
    if args.dataset:
        win.open_dataset(args.dataset)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    main()
