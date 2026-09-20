#!/usr/bin/env python3
"""inspostgui -- INSLIB post-processing GUI.

Interactive Qt front end for python/replay.py: pick a converted dataset
(a directory with config.yaml + the dataset-neutral CSVs, contract:
datasets/replay_format.py), inspect/edit/create its config.yaml, replay
it through the ins filter (in-process, same INSLIB ctypes wrapper and the
same feed order as replay.py's main loop) and evaluate the result:

* live 3D trajectory view with a speed-colored trail, ground-truth
  overlay, attitude-driven vehicle model / error ellipsoid and a
  free-look fly mode (visualization lifted from znav3d),
* live attitude / position / altitude / acceleration panels,
* post-run error plots (estimate vs. ground truth with the filter's own
  1-sigma band), a North-East map view, bias convergence and outlier
  counters,
* a Map tab: the same track over an optional OpenStreetMap background
  (tiles cached on disk and shared with tools/inslib_gui.py's Track tab),
* the same accuracy / data-quality summary replay.py prints,
* multi-page PDF export via ins_plots and Google Earth KML export via
  ins_kml.

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
    python3 python/inspostgui.py
    python3 python/inspostgui.py datasets/simulated/profile_1_car
    python3 python/inspostgui.py --batch datasets/simulated/profile_1_car

--batch replays the dataset headless (no window) and prints the summary,
it exists mainly as a smoke test of the replay worker.

Dependencies (pip install -r python/requirements-inspostgui.txt):
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
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay  # noqa: E402  (loaders, DEFAULTS, build_config, Stat, ...)
from INSLIB import Navigator, ecef_to_llh  # noqa: E402
from INSLIB._core import rpy_to_quat  # noqa: E402

US_PER_SEC = replay.US_PER_SEC
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ============================================================================
# Config schema (mirrors replay.py's DEFAULTS / tools/replay.c)
# ============================================================================

BOOL, FLOAT, INT, STR, CHOICE, VEC = (
    "bool", "float", "int", "str", "choice", "vec")

# (path, label, kind, extra, tooltip), extra: CHOICE -> options, VEC -> length
CONFIG_SECTIONS = [
    ("General", [
        (("name",), "Name", STR, None,
         "Dataset label used in plots and the summary"),
        (("aiding",), "Aiding", CHOICE, ("gnss", "ref", "none"),
         "gnss: real fixes from gnss.csv, ref: fix synthesized from the "
         "ground truth, none: no absolute position aiding (ARS/baro only)"),
        (("init",), "Init", CHOICE, ("auto", "ref"),
         "auto: ins auto-initialization, ref: known initial state from "
         "the first truth epoch (KF-GINS protocol)"),
        (("automotive_mode",), "Automotive mode", BOOL, None,
         "Derive yaw from the GNSS course over ground, "
         "vehicle-mounted non-holonomic platforms only, NOT aircraft"),
        (("automotive_min_speed_mps",), "Automotive min speed [m/s]", FLOAT,
         None, "0 -> default"),
        (("automotive_min_yaw_stddev_deg",), "Automotive min yaw stddev "
         "[deg]", FLOAT, None, "0 -> default"),
        (("automotive_lateral_constraint",), "Lateral velocity constraint",
         BOOL, None,
         "Non-holonomic constraint (REQ-NAV-077): fuse the body-frame "
         "LATERAL velocity against a truth of zero. Needs automotive mode. "
         "Its value is the attitude row, whose gain is the ground speed, so "
         "it holds roll and yaw while GNSS is out. Only the lateral row "
         "exists: the vertical one states v_D + pitch*v_fwd == 0 rather "
         "than level travel and drives any mounting pitch into the pitch "
         "estimate"),
        (("automotive_lateral_stddev_mps",), "Lateral constraint stddev "
         "[m/s]", FLOAT, None,
         "Measurement noise of the lateral constraint; 0 -> default. Set it "
         "by the vehicle's lateral mounting offset rather than by the "
         "residual's scatter: a bound costs what its systematic part costs"),
        (("automotive_lateral_max_yaw_rate_deg",), "Lateral constraint max "
         "yaw rate [deg/s]", FLOAT, None,
         "Skip the constraint above this |yaw rate|, where side slip breaks "
         "the assumption; 0 -> default"),
        (("automotive_lateral_after_sec",), "Lateral constraint after [s]",
         FLOAT, None,
         "Hold the constraint off until this long without a GNSS fusion; "
         "0 -> default, negative -> no delay. Next to a live GNSS velocity "
         "it adds little and risks feeding a mounting error into the state"),
        (("chi2_disable",), "Disable chi2 downweighting", BOOL, None,
         "diagnostics/analysis only"),
        (("chi2_reject_alpha",), "Chi2 reject alpha", FLOAT, None,
         "Global chi2 outlier-gate significance level, shared by every "
         "channel (GNSS, magnetometer, local-position, yaw); 0 -> built-in "
         "default. Larger = stricter/more downweighting (e.g. 0.2 -> gate "
         "at 1.64), smaller = more tolerant (e.g. 0.01 -> gate at 6.63)"),
        (("gyro_bias_window_sec",), "Gyro-bias window [s]", FLOAT, None,
         "Initial parked-phase gyro averaging window"),
        (("auto_init_window_sec",), "Auto-init leveling window [s]", FLOAT,
         None, "IMU window ins's auto-init levels over (median of the "
         "specific force); 0 -> built-in default. Raise it for a low IMU "
         "rate so the window still holds enough samples"),
        (("baro_height_disable",), "Never pick barometric height", BOOL,
         None, "1: keep the GNSS position's vertical row as the height "
         "source at bootstrap even with a barometer present. For a "
         "gnss-labelled stream that is not satellite GNSS and is more "
         "accurate vertically than a barometer (e.g. a Lighthouse/UWB rig)"),
        (("max_deadreckoning_sec",), "Max dead-reckoning [s]", FLOAT, None,
         "IMU-only coasting budget before is_ready() degrades and the "
         "filter freezes (waits for the next fix); 0 -> built-in default "
         "(10 s). Ignored when allow_unlimited_deadreckoning is set"),
        (("allow_unlimited_deadreckoning",), "Allow unlimited "
         "dead-reckoning", BOOL, None,
         "Never expire the coasting window -- stay is_ready() through "
         "arbitrarily long IMU-only outages (position drifts unbounded)"),
    ]),
    ("IMU noise model (required)", [
        (("imu", "gyr_psd"), "gyr_psd [(rad/s)^2/Hz]", FLOAT, None,
         "Gyro noise PSD (Allan), REQUIRED, must be > 0"),
        (("imu", "acc_psd"), "acc_psd [(m/s^2)^2/Hz]", FLOAT, None,
         "Accelerometer noise PSD, REQUIRED, must be > 0"),
        (("imu", "gyr_bias_rw"), "gyr_bias_rw [rad/s/sqrt(s)]", FLOAT, None,
         "Gyro bias random walk"),
        (("imu", "acc_bias_rw"), "acc_bias_rw [m/s^2/sqrt(s)]", FLOAT, None,
         "Accelerometer bias random walk"),
    ]),
    ("Process noise margin (optional, 0 = ins.c default)", [
        (("imu", "pos_pred_stddev_m_sqrts"), "pos_pred_stddev [m/sqrt(s)]",
         FLOAT, None, "Extra margin added on top of the acc/gyro-driven "
         "pos/vel/rpy noise above, 0 -> default"),
        (("imu", "vel_pred_stddev_mps_sqrts"), "vel_pred_stddev [m/s/sqrt(s)]",
         FLOAT, None, "0 -> default. Often dominates growth for "
         "a tight (well-characterized) imu noise config -- see the "
         "\"process noise growth rate\" summary lines"),
        (("imu", "rpy_pred_stddev_rad_sqrts"), "rpy_pred_stddev [rad/sqrt(s)]",
         FLOAT, None, "0 -> default"),
    ]),
    ("Auto-ZUPT/ZARU - one set for ins, ARS/AHRS and baro_alt "
     "(optional, 0 = default)", [
        (("imu", "zero_vel_stddev_mps"), "Zero-vel stddev [m/s]", FLOAT,
         None, "How hard the filter trusts a detected stop's zero "
         "velocity; tighter -> faster bias/attitude convergence while "
         "parked but more sensitive to a false-positive stop. Also "
         "becomes baro_alt's vertical ZUPT stddev. "
         "0 -> 0.05 m/s (NOT ins.c's own built-in default, though they "
         "happen to be equal here)"),
        (("imu", "zero_rot_stddev_deg"), "Zero-rot stddev [deg/s]",
         FLOAT, None, "Same, for the zero-rotation (ZARU) update, ins "
         "and both AHRS instances. "
         "0 -> 0.1 deg/s -- this tool's own default, tighter than ins.c's "
         "own built-in default of 0.5 deg/s"),
        (("imu", "auto_zupt_static_gyr_stddev_deg"),
         "Window stddev gyro [deg/s]", FLOAT, None,
         "PRIMARY stillness criterion: max per-axis RMS stddev of the raw "
         "gyro over a short window (bias-invariant)"),
        (("imu", "auto_zupt_static_acc_stddev_mps2"),
         "Window stddev accel [m/s^2]", FLOAT, None,
         "Same, for the accelerometer"),
        (("imu", "auto_zupt_static_gyr_deg"), "Bound |gyro| [deg/s]",
         FLOAT, None, "Loose magnitude bound beside the window stddev"),
        (("imu", "auto_zupt_static_acc_mps2"), "Bound ||f|-g| [m/s^2]",
         FLOAT, None, "Loose magnitude bound beside the window stddev"),
        (("imu", "auto_zupt_max_vel_mps"), "Max |GNSS vel| [m/s]", FLOAT,
         None, "Velocity gate. Applied only while a recent, precise "
         "enough fix exists; the filter's own velocity state is never "
         "used (that would be circular). Not used by the ARS/AHRS, "
         "which have no velocity state"),
        (("imu", "auto_zupt_max_vel_stddev_mps"),
         "Max GNSS vel 1-sigma [m/s]", FLOAT, None,
         "Accuracy the GNSS velocity must itself report to count as "
         "evidence of standstill"),
        (("imu", "auto_zupt_dwell_sec"), "Dwell [s]", FLOAT, None,
         "How long the platform must look still before a trigger"),
        (("imu", "auto_zupt_min_interval_sec"), "Min interval [s]",
         FLOAT, None, "Minimum time between auto-triggers"),
        (("imu", "auto_zupt_disable"), "Disable everywhere (0/1)",
         FLOAT, None, "1 -> no stillness detection anywhere in the suite"),
        (("imu", "auto_zupt_velocity_blind_disable"),
         "Disable ARS/AHRS detector (0/1)", FLOAT, None,
         "1 -> only the ARS/AHRS's own velocity-blind detector off (it "
         "cannot tell constant-velocity cruise from a standstill); ins "
         "keeps deciding for all filters"),
    ]),
    ("IMU calibration (optional)", [
        (("imu", "acc_misalignment"), "acc misalignment (3x3 col-major)",
         VEC, 9, "corrected = M*(raw - bias), all-zero -> identity"),
        (("imu", "gyr_misalignment"), "gyr misalignment (3x3 col-major)",
         VEC, 9, "corrected = M*(raw - bias), all-zero -> identity"),
        (("imu", "acc_fixed_bias"), "acc fixed bias [m/s^2]", VEC, 3,
         "Removed permanently, never estimated"),
        (("imu", "gyr_fixed_bias"), "gyr fixed bias [rad/s]", VEC, 3,
         "Removed permanently, never estimated"),
    ]),
    ("GNSS", [
        (("gnss", "leverarm_frd"), "Lever arm FRD [m]", VEC, 3,
         "GNSS antenna vs. IMU, body FRD"),
        (("gnss", "pos_stddev_fallback_m"), "Pos stddev fallback "
         "(hor, ver) [m]", VEC, 2,
         "Replaces zero (unknown) diagonal covariance entries"),
        (("gnss", "vel_stddev_fallback_mps"), "Vel stddev fallback [m/s]",
         FLOAT, None, "Replaces zero velocity covariance entries"),
        (("gnss", "max_horizontal_pos_stddev_m"), "Max hor pos stddev [m]",
         FLOAT, None, "Fixes with a reported horizontal position stddev "
         "above this are rejected outright (not just downweighted); "
         "0 -> built-in default"),
        (("gnss", "max_vertical_pos_stddev_m"), "Max ver pos stddev [m]",
         FLOAT, None, "Fixes with a reported vertical position stddev "
         "above this are rejected outright (not just downweighted); "
         "0 -> built-in default"),
        (("gnss", "max_horizontal_vel_stddev_mps"), "Max hor vel stddev "
         "[m/s]", FLOAT, None, "Fixes with a reported horizontal velocity "
         "stddev above this are rejected outright (not just "
         "downweighted); 0 -> built-in default"),
        (("gnss", "max_vertical_vel_stddev_mps"), "Max ver vel stddev "
         "[m/s]", FLOAT, None, "Fixes with a reported vertical velocity "
         "stddev above this are rejected outright (not just "
         "downweighted); 0 -> built-in default"),
        (("gnss", "start_max_horizontal_pos_stddev_m"),
         "3D entry: max hor pos stddev [m]", FLOAT, None,
         "Entering the 3D solution needs better fixes than fusing one "
         "does: below this (and the three thresholds after it) for "
         "init_dwell_sec; 0 -> built-in default"),
        (("gnss", "start_max_vertical_pos_stddev_m"),
         "3D entry: max ver pos stddev [m]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "start_max_horizontal_vel_stddev_mps"),
         "3D entry: max hor vel stddev [m/s]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "start_max_vertical_vel_stddev_mps"),
         "3D entry: max ver vel stddev [m/s]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "init_dwell_sec"), "3D entry dwell [s]", FLOAT, None,
         "How long the fix stream must stay above the entry gate before "
         "the 3D solution is entered; 0 -> built-in default"),
        (("gnss", "init_dwell_disable"), "3D entry dwell off", BOOL, None,
         "1: enter the 3D solution on the first good fix"),
        (("gnss", "stop_max_horizontal_pos_stddev_m"),
         "3D exit: max hor pos stddev [m]", FLOAT, None,
         "Once every fix for stop_dwell_sec is worse than this (or the "
         "three thresholds after it), the 3D solution is left while the "
         "filter keeps running; 0 -> built-in default"),
        (("gnss", "stop_max_vertical_pos_stddev_m"),
         "3D exit: max ver pos stddev [m]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "stop_max_horizontal_vel_stddev_mps"),
         "3D exit: max hor vel stddev [m/s]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "stop_max_vertical_vel_stddev_mps"),
         "3D exit: max ver vel stddev [m/s]", FLOAT, None,
         "0 -> built-in default"),
        (("gnss", "stop_dwell_sec"), "3D exit dwell [s]", FLOAT, None,
         "How long only-bad fixes must arrive before the 3D solution is "
         "left; 0 -> built-in default"),
        (("gnss", "stop_disable"), "3D exit off", BOOL, None,
         "1: never leave the 3D solution on GNSS quality alone"),
        (("gnss", "pos_decimation"), "Pos decimation (every Nth)", INT, None,
         "Fuse the position on every Nth epoch that offers a usable "
         "position AND velocity, the velocity alone on the other N-1: a "
         "receiver reports both out of one coupled solution but not the "
         "correlation between them, so fusing both counts the same "
         "information twice; 1 -> off, 0 -> built-in default"),
        (("gnss", "delay_ms"), "Fix delay [ms]", FLOAT, None,
         "assumed fixed latency, the "
         "GNSS-delay estimate in the summary helps to pick a value"),
        (("gnss", "pos_cov_scale"), "Pos cov scale", FLOAT, None,
         "multiplies the reported pos stddev (0 -> 1)"),
        (("gnss", "pos_cov_scale_height"), "Pos cov scale (height)", FLOAT, None,
         "extra downweight of the pos height axis only (0 -> 1)"),
        (("gnss", "vel_cov_scale"), "Vel cov scale", FLOAT, None,
         "Multiplies the reported vel stddev (0 -> 1)"),
        (("gnss", "pos_stddev_floor_hor_m"), "Pos stddev floor hor [m]",
         FLOAT, None, "stddev = max(scale*reported, floor)"),
        (("gnss", "pos_stddev_floor_ver_m"), "Pos stddev floor ver [m]",
         FLOAT, None, None),
        (("gnss", "vel_stddev_floor_hor_mps"), "Vel stddev floor hor [m/s]",
         FLOAT, None, None),
        (("gnss", "vel_stddev_floor_ver_mps"), "Vel stddev floor ver [m/s]",
         FLOAT, None, None),
    ]),
    ("Initial-state stddev overrides (0 = built-in)", [
        (("init_stddev", "pos_init_stddev_m"), "Position [m]",
         FLOAT, None, "Initial position std.-dev."),
        (("init_stddev", "vel_init_stddev_mps"), "Velocity [m/s]",
         FLOAT, None, "Initial velocity std.-dev."),
        (("init_stddev", "rpy_init_stddev_rad_deg"), "Attitude [deg]",
         FLOAT, None, "Initial attitude std.-dev."),
        (("init_stddev", "yaw_init_stddev_rad_deg"), "Yaw [deg]",
         FLOAT, None, "Initial yaw std.-dev. 0 -> falls back to the attitude value"),
        (("init_stddev", "acc_bias_init_stddev_mps2"), "Acc bias [m/s^2]",
         FLOAT, None, "Initial accelerometer bias std.-dev."),
        (("init_stddev", "gyr_bias_init_stddev_rps_deg"), "Gyr bias [deg/s]",
         FLOAT, None, "Initial gyroscope bias std.-dev."),
    ]),
    ("Initial attitude hint (auto-init only, optional)", [
        (("init_hint", "roll_deg"), "Roll [deg]", FLOAT, None,
         "Known initial roll, used together with pitch below (both "
         "stddevs must be > 0). Usually unneeded -- leveling is already "
         "observable from gravity without any hint"),
        (("init_hint", "pitch_deg"), "Pitch [deg]", FLOAT, None,
         "Known initial pitch, see roll above"),
        (("init_hint", "rpy_stddev_deg"), "Roll/pitch stddev [deg]", FLOAT,
         None, "0 -> no roll/pitch hint (the default: let auto-init level "
         "from gravity as usual)"),
        (("init_hint", "yaw_deg"), "Yaw [deg]", FLOAT, None,
         "Known initial yaw/heading, e.g. from a known launch heading, "
         "for a start with no magnetometer/GNSS-course yaw aiding"),
        (("init_hint", "yaw_stddev_deg"), "Yaw stddev [deg]", FLOAT, None,
         "0 -> no yaw hint (the default: yaw stays unobservable until "
         "aiding arrives)"),
    ]),
    ("Magnetometer", [
        (("mag", "enable"), "Enable (needs mag.csv)", BOOL, None, None),
        (("mag", "stddev_ut"), "Stddev [uT]", FLOAT, None,
         "0 -> library default. Yaw noise is roughly stddev / horizontal "
         "field, so a large value is what makes the mag a long-term "
         "anchor instead of a per-epoch yaw sensor"),
        (("mag", "min_delay_ms"), "Min fusion delay [ms]", INT, None,
         "0 -> library default, negative -> fuse every sample"),
        (("mag", "wmm_year"), "WMM year (e.g. 2024.5)", FLOAT, None,
         "0: magnetic north, > 0: WMM declination -> true north"),
        (("mag", "estimate_bias"), "Estimate hard-iron bias (18-state)",
         BOOL, None, "Needs attitude changes "
         "to converge"),
        (("mag", "misalignment"), "Soft-iron (3x3 col-major)", VEC, 9,
         "all-zero -> identity"),
        (("mag", "fixed_bias"), "Hard-iron fixed bias [uT]", VEC, 3, None),
    ]),
    ("Barometer", [
        (("baro", "enable"), "Enable (needs baro.csv)", BOOL, None, None),
        (("baro", "stddev_m"), "Stddev [m]", FLOAT, None,
         "Raw sensor accuracy: feeds baro_alt AND the local/GNSS vertical "
         "datum-offset cross-check, not just baro_alt's "
         "own filter - see the Baro_Alt group below for baro_alt's own "
         "process-noise tuning. 0 -> built-in"),
    ]),
    ("Baro_Alt (process noise, optional, 0 = baro_alt.c default)", [
        (("baro", "acc_noise_mps2_sqrthz"), "acc_noise [m/s^2/sqrt(Hz)]",
         FLOAT, None, "baro_alt's own direct accel-noise density, "
         "0 -> baro_alt built-in default"),
        (("baro", "acc_bias_rw"), "acc_bias_rw [m/s^2/sqrt(Hz)]", FLOAT,
         None, "baro_alt's accel-bias drift density, 0 -> baro_alt "
         "built-in default"),
        (("baro", "acc_bias_init_mps2"), "acc_bias_init [m/s^2]", FLOAT,
         None, "baro_alt's own INITIAL accel-bias uncertainty, an initial "
         "condition (not a process-noise rate) that can dominate the "
         "apparent growth for tens of seconds, 0 -> baro_alt built-in "
         "default"),
        (("baro", "h_process_noise"), "h_process_noise [m/sqrt(Hz)]", FLOAT,
         None, "small direct process-noise density on baro_alt's height "
         "state, a safety margin against discretization/model mismatch, "
         "0 -> baro_alt built-in default"),
    ]),
    ("Baro/GNSS offset filter (optional, 0 = local_gnss_alt.c default)", [
        (("baro", "local_gnss_rw_stddev_mps"), "rw_stddev [m/sqrt(s)]",
         FLOAT, None, "Random walk of the local-height-to-ellipsoid "
         "offset. Raise for missions with large altitude excursions "
         "(e.g. a soaring glider gaining kilometres of altitude): the "
         "offset absorbs the ISA-model error, which grows with the "
         "excursion and can outrun the default (tuned for a multi-metre "
         "swing), leaving the offset permanently lagging. 0 -> built-in "
         "default 0.03"),
        (("baro", "local_gnss_chi2_threshold"), "chi2_threshold", FLOAT,
         None, "Chi2 outlier gate on the offset innovation (downweights, "
         "does not drop), 0 -> built-in default (chi2inv(0.95,1))"),
        (("baro", "local_gnss_min_update_interval_sec"),
         "min_update_interval [s]", FLOAT, None,
         "Minimum time between two offset fusions, 0 -> built-in "
         "default 10"),
        (("baro", "local_gnss_stddev_inflation_factor"),
         "stddev_inflation_factor", FLOAT, None,
         "Factor applied to both sides of the pair before combining "
         "into the measurement variance, 0 -> built-in default 3"),
    ]),
    ("ARS/AHRS noise model (optional, 0 = ahrs.c default)", [
        (("ahrs", "gyr_noise_psd"), "gyr_noise_psd [rad/s/sqrt(Hz)]", FLOAT,
         None, "SHARED by both ARS and AHRS (same physical gyro) "
         "0 -> ahrs built-in default"),
        (("ahrs", "gyr_bias_rw"), "gyr_bias_rw [rad/s^2/sqrt(Hz)]", FLOAT,
         None, "SHARED by both ARS and AHRS, 0 -> ahrs built-in default"),
        (("ahrs", "acc_noise_mps2"), "acc_noise [m/s^2]", FLOAT, None,
         "SHARED by both ARS and AHRS (same physical accelerometer, "
         "used for leveling), 0 -> ahrs built-in default"),
        (("ahrs", "gyr_bias_init_stddev_rps_deg"), "gyr_bias_init stddev "
         "[deg/s]", FLOAT, None,
         "Initial gyro-bias uncertainty of both sub-filters, 0 -> the "
         "static-window seed's own stddev (or ahrs's built-in default if "
         "the window found nothing)"),
    ]),
    ("Scoring (evaluation only, never touches the filter)", [
        (("score", "leverarm_frd"), "Scoring lever arm FRD [m]", VEC, 3,
         "Ground-truth reference point vs. IMU"),
        (("score", "warmup_sec"), "Warmup [s]", FLOAT, None,
         "Scoring starts this long after the first fix"),
        (("score", "ahrs"), "Score the attitude filters too", BOOL, None,
         "1: also score the ARS/AHRS and their regression gates"),
        (("score", "min_epochs"), "Min scored epochs", INT, None,
         "Minimum number of scored epochs a run must produce; 0 -> not "
         "gated"),
        (("score", "coast_gap_min_sec"), "Coasting gap threshold [s]", FLOAT,
         None,
         "Report the position error at the first reference epoch after every "
         "aiding gap longer than this (REQ-VER-029); 0 -> off. On a dataset "
         "with a real outage this is the number that means something: the "
         "whole-run RMS scores a filter that gives the coasting up BETTER "
         "than one that coasts through, because it contributes no epochs "
         "where the error is large"),
        (("score", "ref_delay_ms"), "Reference delay [ms]", FLOAT, None,
         "The reference row timestamped T actually describes T minus this "
         "(REQ-VER-030). Set it to mirror gnss: delay_ms when ref.csv comes "
         "out of the same receiver output as the aiding, otherwise the score "
         "penalises the filter by speed times delay for compensating "
         "correctly. 0 for an independent reference"),
        (("score", "lim_coast_exit_err_m"), "Limit coasting exit error [m]",
         FLOAT, None,
         "Regression gate on that error; 0 -> not gated. Having no solution "
         "when aiding returns counts as a failure, not as a skipped epoch"),
    ]),
    ("Inputs (optional CSV overrides)", [
        (("inputs", "imu"), "IMU CSV", STR, None,
         "Relative to the config's directory, blank -> imu.csv"),
        (("inputs", "ref"), "Reference CSV", STR, None,
         "blank -> ref.csv"),
        (("inputs", "gnss"), "GNSS CSV", STR, None,
         "blank -> gnss.csv, e.g. gnss_f9p.csv to A/B a receiver"),
        (("inputs", "mag"), "Magnetometer CSV", STR, None,
         "blank -> mag.csv"),
        (("inputs", "baro"), "Barometer CSV", STR, None,
         "blank -> baro.csv"),
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
            raw = yaml.safe_load(f) or {}
    return raw, cfg_path, data_dir


def validate_spec(spec, data_dir):
    """Pre-run checks, returns a list of error strings (empty = ok)."""
    errors = []
    if not spec["imu"].get("gyr_psd") or not spec["imu"].get("acc_psd"):
        errors.append("imu: gyr_psd/acc_psd missing or zero (required "
                      "noise model)")
    for stream in ("imu", "ref"):
        path = replay.input_path(data_dir, spec, stream)
        if not os.path.exists(path):
            errors.append(f"{os.path.basename(path)} missing in {data_dir}")
    need = {"gnss": spec["aiding"] == "gnss",
            "mag": bool(int(spec["mag"]["enable"])),
            "baro": bool(int(spec["baro"]["enable"]))}
    for stream, needed in need.items():
        path = replay.input_path(data_dir, spec, stream)
        if needed and not os.path.exists(path):
            errors.append(f"{os.path.basename(path)} missing in {data_dir}")
    return errors


def discover_datasets():
    """(label, dir) for every config.yaml anywhere under datasets/."""
    found = []
    base = os.path.join(REPO_ROOT, "datasets")
    if not os.path.isdir(base):
        return found
    for root, dirs, files in os.walk(base):
        dirs[:] = sorted(d for d in dirs if d != "raw")
        if "config.yaml" in files:
            found.append((os.path.relpath(root, base), root))
            dirs[:] = []  # a dataset dir has no nested datasets
    return sorted(found)


# ============================================================================
# Replay worker (in-process port of replay.py's main loop)
# ============================================================================

REC_KEYS = (
    "t", "pos", "pos_sigma", "vel", "vel_sigma",
    "rpy_deg", "rpy_sigma_deg", "ref_pos", "ref_vel", "ref_rpy_deg",
    "pos_err_ned", "rpy_err_deg", "baro_vel_d", "gnss_vel_d",
    "gnss_pos_sigma", "gnss_vel_sigma",
    "acc_bias", "acc_bias_sigma", "gyr_bias", "gyr_bias_sigma",
    "mag_bias", "mag_bias_sigma",
    "ars_rpy_deg", "ars_rpy_sigma_deg", "ars_gyr_bias", "ars_gyr_bias_sigma",
    "ahrs_rpy_deg", "ahrs_rpy_sigma_deg", "ahrs_gyr_bias", "ahrs_gyr_bias_sigma",
    "baro_h_d", "baro_raw_d", "baro_h_sigma", "baro_acc_bias", "baro_acc_bias_sigma",
    "fix_pos_d", "local_gnss_offset", "local_gnss_offset_sigma",
    "zupt_active",
    "dw_full3d", "dw_ars", "dw_ahrs", "dw_baro_alt", "dw_local_gnss",
)

from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: E402
import numpy as np  # noqa: E402
import pyqtgraph as pg  # noqa: E402
import pyqtgraph.opengl as gl  # noqa: E402

from ins_map_view import MapView  # noqa: E402

# The window icon. It lives with the documentation rather than with the
# tools, so the path is taken relative to THIS FILE and not to the working
# directory - the window then has its icon whichever directory it was
# started from.
LOGO = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                    "doc", "figures", "inslib_logo.png")


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
    """Runs the dataset through ins on a worker thread. Mirrors
    replay.py's main() feed order exactly (IMU epoch, then the most recent
    fix/mag/baro, then update()), records the same per-sample history dict
    (self.rec, guarded by self.lock) so ins_plots.plot_results can consume
    it unchanged, plus a low-rate live snapshot (self.live) for the UI."""

    sig_status = QtCore.pyqtSignal(str)
    sig_progress = QtCore.pyqtSignal(int)          # percent 0..100
    sig_finished = QtCore.pyqtSignal(dict)         # results (see _replay)
    sig_error = QtCore.pyqtSignal(str)

    def __init__(self, spec, data_dir, realtime=False, speed=1.0,
                 rec_hz=10.0, parent=None):
        super().__init__(parent)
        self.spec = spec
        self.data_dir = data_dir
        self.realtime = realtime
        self.speed = max(speed, 1e-6)
        self.rec_hz = rec_hz
        self.lock = threading.Lock()
        self.rec = {k: [] for k in REC_KEYS}
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
        except Exception:
            self.sig_error.emit(traceback.format_exc())

    # ------------------------------------------------------------------
    def _replay(self):
        spec, data_dir = self.spec, self.data_dir
        imu_path = replay.input_path(data_dir, spec, "imu")
        ref = replay.load_ref(replay.input_path(data_dir, spec, "ref"))
        if not ref:
            raise RuntimeError("no reference epochs in ref.csv")

        gnss_cfg = spec["gnss"]
        gnss_delay_ms = int(gnss_cfg.get("delay_ms", 0.0))
        aiding = spec["aiding"]
        var_hor = gnss_cfg["pos_stddev_fallback_m"][0] ** 2
        var_ver = gnss_cfg["pos_stddev_fallback_m"][1] ** 2
        var_vel = gnss_cfg["vel_stddev_fallback_mps"] ** 2
        if aiding == "gnss":
            gnss_path = replay.input_path(data_dir, spec, "gnss")
            fixes = (replay.load_gnss(gnss_path)
                     if os.path.exists(gnss_path) else None)
            if not fixes:
                raise RuntimeError(f"aiding: gnss but no usable {gnss_path}")
            for fx in fixes:
                fx["cov_pos"] = replay.apply_fallback(
                    replay.cov6_to_rows(fx["cov_pos"]), var_hor, var_ver)
                fx["cov_vel"] = (replay.apply_fallback(
                    replay.cov6_to_rows(fx["cov_vel"]), var_vel, var_vel)
                    if fx["vel_ok"] else None)
        elif aiding == "ref":
            # Noise is pos_stddev_fallback_m/vel_stddev_fallback_mps, same
            # fields a real fix's zero/unknown covariance diagonals fall
            # back to: this synthesized fix never has a reported covariance
            # either.
            pos_cov = [[var_hor, 0, 0], [0, var_hor, 0], [0, 0, var_ver]]
            vel_cov = [[var_vel if i == j else 0
                        for j in range(3)] for i in range(3)]
            fixes = [{"t_us": r["t_us"], "lat_rad": r["lat_rad"],
                      "lon_rad": r["lon_rad"], "h_m": r["h_m"],
                      "cov_pos": pos_cov, "vel_ned": r["vel_ned"],
                      "cov_vel": vel_cov, "vel_ok": True} for r in ref]
        elif aiding == "none":
            fixes = []
        else:
            raise RuntimeError(f"unknown aiding mode {aiding!r}")

        mag_cfg = spec["mag"]
        mags = []
        if int(mag_cfg["enable"]):
            mags = replay.load_txyz(replay.input_path(data_dir, spec, "mag"))
            if not mags:
                raise RuntimeError("mag: enable but no usable mag.csv")
        mag_sd = float(mag_cfg["stddev_ut"])  # 0 -> library default
        mag_var = (mag_sd ** 2,) * 3

        baro_cfg = spec["baro"]
        baros = []
        if int(baro_cfg["enable"]):
            baros = replay.load_baro(replay.input_path(data_dir, spec, "baro"))
            if not baros:
                raise RuntimeError("baro: enable but no usable baro.csv")

        leverarm = tuple(gnss_cfg["leverarm_frd"])
        score_la = tuple(spec["score"].get("leverarm_frd") or (0.0,) * 3)

        self.sig_status.emit("Estimating initial gyro bias ...")
        gyr_bias = replay.estimate_gyro_bias(imu_path,
                                             spec["gyro_bias_window_sec"])

        # init: ref uses the reference epoch aligned with the first IMU
        # sample (same rationale as replay.py: ins stamps a prescribed
        # init at its actual start epoch without propagating it,
        # REQ-NAV-033 -- an older truth epoch bakes in a -v*dt offset).
        first_imu_us = next(replay.iter_imu(imu_path))[0]
        ref0 = next((r for r in ref if r["t_us"] >= first_imu_us), ref[0])
        if gyr_bias is not None and spec["init"] == "ref":
            # De-contaminate the initial-window average from the modeled
            # non-bias content (reference motion + earth/transport rate),
            # like replay.py -- see replay.correct_initial_gyr_bias.
            gyr_bias = replay.correct_initial_gyr_bias(
                gyr_bias, ref, first_imu_us, spec["gyro_bias_window_sec"])
        t0 = ref0 if (spec["init"] == "ref" or not fixes) else fixes[0]
        nav = Navigator(replay.build_config(
            spec, ref0, t0["t_us"], t0["lat_rad"], t0["lon_rad"],
            t0["h_m"], gyr_bias))
        # baro_alt / ARS/AHRS noise model (0 -> each filter's own default),
        # must be set before the first baro sample / update() latches the
        # respective template (nav_suite contract) -- mirrors replay.py's
        # main().
        ahrs_cfg = spec["ahrs"]
        nav.set_baro_acc_bias_drift(float(baro_cfg.get("acc_bias_rw", 0.0)))
        nav.set_baro_acc_noise(float(baro_cfg.get("acc_noise_mps2_sqrthz", 0.0)))
        nav.set_baro_acc_bias_init_stddev(float(baro_cfg.get("acc_bias_init_mps2", 0.0)))
        nav.set_baro_h_process_noise(float(baro_cfg.get("h_process_noise", 0.0)))
        nav.set_local_gnss_rw_stddev(
            float(baro_cfg.get("local_gnss_rw_stddev_mps", 0.0)))
        nav.set_local_gnss_chi2_threshold(
            float(baro_cfg.get("local_gnss_chi2_threshold", 0.0)))
        nav.set_local_gnss_min_update_interval(
            float(baro_cfg.get("local_gnss_min_update_interval_sec", 0.0)))
        nav.set_local_gnss_stddev_inflation(
            float(baro_cfg.get("local_gnss_stddev_inflation_factor", 0.0)))
        nav.set_ahrs_gyr_noise(float(ahrs_cfg.get("gyr_noise_psd", 0.0)))
        nav.set_ahrs_acc_noise(float(ahrs_cfg.get("acc_noise_mps2", 0.0)))
        nav.set_ahrs_gyr_bias_rw(float(ahrs_cfg.get("gyr_bias_rw", 0.0)))
        init_hint = spec["init_hint"]
        nav.set_init_att_hint(
            math.radians(init_hint["roll_deg"]), math.radians(init_hint["pitch_deg"]),
            math.radians(init_hint["rpy_stddev_deg"]), math.radians(init_hint["yaw_deg"]),
            math.radians(init_hint["yaw_stddev_deg"]))
        # No set_auto_zaru() here: the ARS/AHRS fallback is armed by default
        # and configured through the same imu.auto_zupt_* set as ins
        # (REQ-SUITE-020, already in build_config's Config above). Calling
        # the runtime override would silently re-arm a config that opted out
        # via imu.auto_zupt_velocity_blind_disable.
        if mags and float(mag_cfg["wmm_year"]) > 0:
            nav.set_magnetic_model(ref[0]["lat_rad"], ref[0]["lon_rad"],
                                   float(mag_cfg["wmm_year"]))

        noise = spec["imu"]
        acc_var = (noise["acc_psd"],) * 3
        gyr_var = (noise["gyr_psd"],) * 3

        self.sig_status.emit("IMU pre-pass (gaps / rates / noise) ...")
        (n_imu_total, imu_duration, imu_hz, imu_max_gap, imu_max_gap_at,
         gyr_diff_stat, acc_diff_stat, rate_t0, imu_rate_t,
         imu_rate_hz) = replay._imu_prepass(imu_path)

        sensor_rate = {"imu": (imu_rate_t, imu_rate_hz)}
        if aiding == "gnss" and fixes:
            sensor_rate["gnss"] = replay._bucketed_rate(
                [fx["t_us"] for fx in fixes], rate_t0, 5.0)
        if mags:
            sensor_rate["mag"] = replay._bucketed_rate(
                [m[0] for m in mags], rate_t0, 5.0)
        if baros:
            sensor_rate["baro"] = replay._bucketed_rate(
                [b[0] for b in baros], rate_t0, 5.0)

        self.sig_status.emit(f"Replaying {spec['name'] or data_dir} "
                             f"({n_imu_total} IMU samples) ...")

        rec = self.rec
        iref = ifix = imag = ibaro = 0
        t_prev = None
        n_imu = 0
        last_ref = last_fix = last_baro = None
        origin_ecef = None
        origin_lat = origin_lon = 0.0
        t0_us = None
        last_rec_us = None
        rec_period_us = US_PER_SEC / max(self.rec_hz, 1e-3)
        last_live_us = None
        live_period_us = US_PER_SEC / 30.0
        last_kml_us = None
        kml_period_us = US_PER_SEC / 2.0
        kml_est, kml_ref, kml_fix = [], [], []
        wall0 = time.perf_counter()
        t_warmup_end = ((fixes[0] if fixes else ref[0])["t_us"]
                        + int(spec["score"]["warmup_sec"] * US_PER_SEC))
        Stat = replay.Stat
        e_roll, e_pitch, e_yaw, e_pos = Stat(), Stat(), Stat(), Stat()
        n_static = 0
        gyr_noise_stat = [Stat(), Stat(), Stat()]
        acc_noise_stat = [Stat(), Stat(), Stat()]
        last_pct = -1
        aborted = False

        # Dr. INS findings inputs (mirrors replay.py's main() loop): GNSS/
        # baro/mag standstill-spread phases (gnss_standstill_accuracy/
        # channel_standstill_accuracy) and the filter's own post-warmup,
        # streaming, moving 1-sigma vs. STDDEV_LIMITS.
        gnss_static_phases, gnss_cur_phase = [], []
        baro_static_phases, baro_cur_phase = [], []
        mag_static_phases, mag_cur_phase = [], []
        health_stats = {k: {"worst": 0.0, "t": 0.0, "n_exceed": 0}
                        for k in replay.STDDEV_LIMITS}
        health_n = 0
        last_fix_used_us = None
        gnss_grace_us = 2 * US_PER_SEC
        if fixes and len(fixes) > 1:
            nominal_sec = ((fixes[-1]["t_us"] - fixes[0]["t_us"]) / US_PER_SEC
                          / (len(fixes) - 1))
            gnss_grace_us = int(max(2.0, 3.0 * nominal_sec) * US_PER_SEC)

        for t, g, a in replay.iter_imu(imu_path):
            if self._stop.is_set():
                aborted = True
                break
            while self._pause.is_set() and not self._stop.is_set():
                time.sleep(0.05)
                wall0 += 0.05  # keep realtime pacing across the pause
            if t0_us is None:
                t0_us = t
            dt = (t - t_prev) / US_PER_SEC if t_prev is not None else 0.0
            t_prev = t
            n_imu += 1
            pct = int(100 * n_imu / n_imu_total) if n_imu_total else 100
            if pct != last_pct:
                last_pct = pct
                self.sig_progress.emit(pct)

            nav.imu(t, dt, a, g, acc_var, gyr_var)

            fix_now = None
            while ifix < len(fixes) and fixes[ifix]["t_us"] <= t:
                fix_now = fixes[ifix]
                ifix += 1
            if fix_now is not None and fix_now["cov_pos"] is not None:
                nav.gnss_pos_llh((fix_now["lat_rad"], fix_now["lon_rad"],
                                  fix_now["h_m"]),
                                 fix_now["cov_pos"], delay_ms=gnss_delay_ms)
                if fix_now["vel_ok"] and fix_now["cov_vel"] is not None:
                    nav.gnss_vel(fix_now["vel_ned"], fix_now["cov_vel"])
                nav.gnss_leverarm(leverarm)
            if fix_now is not None:
                last_fix = fix_now

            mag_now = None
            while imag < len(mags) and mags[imag][0] <= t:
                mag_now = mags[imag]
                imag += 1
            if mag_now is not None:
                nav.mag(mag_now[1], mag_var)
            baro_now = None
            while ibaro < len(baros) and baros[ibaro][0] <= t:
                baro_now = baros[ibaro]
                ibaro += 1
            if baro_now is not None:
                nav.baro(baro_now[1], float(baro_cfg["stddev_m"]))
                last_baro = baro_now

            nav.update()

            if nav.auto_zupt_active():
                n_static += 1
                bg = nav.bias_gyr()
                ba = nav.bias_acc()
                if bg is not None and ba is not None:
                    for k in range(3):
                        gyr_noise_stat[k].add(g[k] - bg[k])
                        acc_noise_stat[k].add(a[k] - ba[k])

            ref_now = None
            while iref < len(ref) and ref[iref]["t_us"] <= t:
                ref_now = ref[iref]
                iref += 1
            if ref_now is not None:
                last_ref = ref_now

            if origin_ecef is None:
                origin_ecef = nav.origin_ecef()
                if origin_ecef is not None:
                    origin_lat, origin_lon, _ = ecef_to_llh(*origin_ecef)

            # Dr. INS findings inputs (see init above, mirrors replay.py's
            # main() loop verbatim): GNSS/baro/mag standstill phases and the
            # filter self-consistency watchdog vs. STDDEV_LIMITS.
            if (aiding == "gnss" and fix_now is not None
                    and fix_now["cov_pos"] is not None):
                if nav.auto_zupt_active():
                    if origin_ecef is not None:
                        cov_p = fix_now["cov_pos"]
                        cov_v = fix_now["cov_vel"]
                        has_vel = fix_now["vel_ok"] and cov_v is not None
                        gnss_cur_phase.append({
                            "pos_ned": replay.ref_to_local_ned(
                                fix_now, origin_ecef, origin_lat, origin_lon),
                            "vel_ned": (list(fix_now["vel_ned"]) if has_vel
                                       else None),
                            "rep_pos_hor": math.sqrt((cov_p[0][0] + cov_p[1][1]) / 2.0),
                            "rep_pos_ver": math.sqrt(cov_p[2][2]),
                            "rep_vel_hor": (math.sqrt((cov_v[0][0] + cov_v[1][1]) / 2.0)
                                           if has_vel else None),
                            "rep_vel_ver": (math.sqrt(cov_v[2][2]) if has_vel
                                           else None),
                        })
                elif gnss_cur_phase:
                    gnss_static_phases.append(gnss_cur_phase)
                    gnss_cur_phase = []

            standstill = nav.auto_zupt_active()
            if baros:
                if standstill and baro_now is not None:
                    baro_cur_phase.append(
                        (replay.isa_pressure_to_altitude(baro_now[1]),))
                elif not standstill and baro_cur_phase:
                    baro_static_phases.append(baro_cur_phase)
                    baro_cur_phase = []
            if mags:
                mag_still = standstill and nav.zaru_active()
                if mag_still and mag_now is not None:
                    mag_cur_phase.append(tuple(mag_now[1]))
                elif not mag_still and mag_cur_phase:
                    mag_static_phases.append(mag_cur_phase)
                    mag_cur_phase = []

            if fix_now is not None and fix_now["cov_pos"] is not None:
                streaming = (last_fix_used_us is not None
                            and (t - last_fix_used_us) <= gnss_grace_us)
                last_fix_used_us = t
                sd_now = nav.stddev() if nav.is_ready() else None
                if (streaming and t >= t_warmup_end and sd_now is not None
                        and not nav.auto_zupt_active()):
                    health_n += 1
                    t_sec = (t - t0_us) / US_PER_SEC
                    vals = {"roll": math.degrees(sd_now["rpy"][0]),
                           "pitch": math.degrees(sd_now["rpy"][1]),
                           "yaw": math.degrees(sd_now["rpy"][2]),
                           "pos": math.sqrt(sum(x * x for x in sd_now["pos_ned"]))}
                    for k, v in vals.items():
                        h = health_stats[k]
                        if v > h["worst"]:
                            h["worst"], h["t"] = v, t_sec
                        if v > replay.STDDEV_LIMITS[k]:
                            h["n_exceed"] += 1

            if last_rec_us is None or (t - last_rec_us) >= rec_period_us:
                last_rec_us = t
                self._record(rec, nav, t, t0_us, last_ref, last_fix,
                             last_baro, origin_ecef, origin_lat, origin_lon,
                             score_la)

            if last_live_us is None or (t - last_live_us) >= live_period_us:
                last_live_us = t
                self._update_live(nav, t, t0_us, imu_duration, a, g)

            if last_kml_us is None or (t - last_kml_us) >= kml_period_us:
                last_kml_us = t
                ecef = nav.position_ecef()
                rpy = nav.rpy_ins()
                if ecef is not None and rpy is not None:
                    lat, lon, alt = ecef_to_llh(*ecef)
                    kml_est.append(((t - t0_us) / US_PER_SEC,
                                    math.degrees(lat), math.degrees(lon),
                                    alt, math.degrees(rpy[0]),
                                    math.degrees(rpy[1]),
                                    math.degrees(rpy[2])))
                    if last_ref is not None:
                        kml_ref.append((math.degrees(last_ref["lat_rad"]),
                                        math.degrees(last_ref["lon_rad"]),
                                        last_ref["h_m"]))
                    if last_fix is not None:
                        fx = (math.degrees(last_fix["lat_rad"]),
                             math.degrees(last_fix["lon_rad"]),
                             last_fix["h_m"])
                        cov_p = last_fix.get("cov_pos")
                        cov_ne = ((cov_p[0][0], cov_p[0][1], cov_p[1][1])
                                 if cov_p is not None
                                 else (math.nan, math.nan, math.nan))
                        if not kml_fix or fx != kml_fix[-1][1:4]:
                            kml_fix.append(
                                ((t - t0_us) / US_PER_SEC,) + fx + cov_ne)

            if ref_now is not None and nav.is_ready() and t >= t_warmup_end:
                err = replay.pos_error_ecef(nav, ref_now, score_la)
                if err is not None:
                    e_pos.add(math.sqrt(sum(e * e for e in err)))
                rpy = nav.rpy_ins()
                if rpy is not None:
                    e_roll.add(math.degrees(replay.wrap_pi(
                        rpy[0] - ref_now["roll_rad"])))
                    e_pitch.add(math.degrees(replay.wrap_pi(
                        rpy[1] - ref_now["pitch_rad"])))
                    e_yaw.add(math.degrees(replay.wrap_pi(
                        rpy[2] - ref_now["yaw_rad"])))

            if self.realtime:
                target = wall0 + (t - t0_us) / US_PER_SEC / self.speed
                while not self._stop.is_set():
                    sleep = target - time.perf_counter()
                    if sleep <= 0:
                        break
                    time.sleep(min(sleep, 0.2))

        if gnss_cur_phase:  # close a standstill phase still open at EOF
            gnss_static_phases.append(gnss_cur_phase)
        if baro_cur_phase:
            baro_static_phases.append(baro_cur_phase)
        if mag_cur_phase:
            mag_static_phases.append(mag_cur_phase)

        # ---- summary (same content replay.py prints) ---------------------
        lines = []
        lines.append(f"dataset {spec['name'] or data_dir}: "
                     f"aiding={aiding}, init={spec['init']}, "
                     f"warmup {spec['score']['warmup_sec']:g} s, "
                     f"leverarm FRD {list(leverarm)}"
                     f"{', automotive' if spec['automotive_mode'] else ''}"
                     f"{', mag' if mags else ''}"
                     f"{', baro' if baros else ''}"
                     f"{f', gnss delay {gnss_delay_ms} ms' if gnss_delay_ms else ''}")
        lines.append(
            f"initial gyro bias: "
            f"{[round(math.degrees(b), 4) for b in (gyr_bias or (0, 0, 0))]}"
            f" deg/s ({'from parked phase' if gyr_bias else 'n/a'})")
        if aborted:
            lines.append(f"*** STOPPED by user after {n_imu}/{n_imu_total} "
                         f"IMU samples -- partial results below ***")
        diag = nav.diag()
        lines.append(f"replayed {n_imu} IMU samples, {len(fixes)} fixes, "
                     f"{len(ref)} reference epochs"
                     f"{f', {len(mags)} mag' if mags else ''}"
                     f"{f', {len(baros)} baro' if baros else ''}")
        lines.append(f"ins: {diag['n_predict']} predicts, "
                     f"{diag['n_gnss_used']} gnss fusions "
                     f"({diag['n_gnss_seen']} seen), "
                     f"{diag['n_fuse_fail']} fuse fails, "
                     f"{diag['n_auto_zupt']} auto-zupt, "
                     f"{diag['n_downweighted']} downweighted")
        dw = nav.downweight_counts()
        lines.append(f"downweighted (chi2 outlier): ars {dw['ars']}, "
                     f"ahrs {dw['ahrs']}, baro_alt {dw['baro_alt']}, "
                     f"local_gnss_offset {dw['local_gnss']}")
        # Which sub-filters were alive at the end, and -- when the 3D filter
        # was not -- the reason (same codes telemetry publishes live).
        st_tree, _blocked, blocked_text = replay.suite_status(nav)
        running = [n for n, k in (("ars", "ars_running"), ("ahrs", "ahrs_running"),
                                  ("baro_alt", "baro_running"),
                                  ("full3d", "full3d_running"))
                   if st_tree[k] > 0.0]
        lines.append(f"final state: mode {nav.mode_name()}, running "
                     f"[{', '.join(running) if running else 'none'}] -- "
                     f"{blocked_text}")
        sd_final = nav.stddev()
        if sd_final is not None:
            p, v, a = (sd_final["pos_ned"], sd_final["vel_ned"],
                      sd_final["rpy"])
            ab, gb = sd_final["acc_bias"], sd_final["gyr_bias"]
            lines.append("final 1-sigma (last epoch, filter-reported):")
            lines.append(f"  pos NED   {p[0]:.3f} {p[1]:.3f} {p[2]:.3f} m")
            lines.append(f"  vel NED   {v[0]:.3f} {v[1]:.3f} {v[2]:.3f} m/s")
            lines.append(f"  att RPY   {math.degrees(a[0]):.3f} "
                         f"{math.degrees(a[1]):.3f} {math.degrees(a[2]):.3f} "
                         f"deg")
            lines.append(f"  acc bias  {ab[0]:.4f} {ab[1]:.4f} {ab[2]:.4f} "
                         f"m/s^2")
            lines.append(f"  gyr bias  {math.degrees(gb[0]):.4f} "
                         f"{math.degrees(gb[1]):.4f} {math.degrees(gb[2]):.4f}"
                         f" deg/s")
            if "mag_bias" in sd_final:
                mb = sd_final["mag_bias"]
                lines.append(f"  mag bias  {mb[0]:.3f} {mb[1]:.3f} "
                             f"{mb[2]:.3f} uT")
        fix_acc_lines = replay.last_fix_accuracy_lines(last_fix)
        if fix_acc_lines is not None:
            lines += fix_acc_lines
        lines += replay.growth_rate_lines(noise)
        lines += replay.baro_growth_rate_lines(baro_cfg)
        lines += replay.ahrs_growth_rate_lines(ahrs_cfg)
        oc = nav.overconfidence()

        def _mm(x):
            return "-" if not math.isfinite(x) else f"{x:.2e}"
        lines.append(f"covariance watchdog: best reported stddev pos "
                     f"{_mm(oc['min_pos_m'])} m, vel {_mm(oc['min_vel_mps'])}"
                     f" m/s, att {_mm(oc['min_att_deg'])} deg")
        lines.append(f"  ARS best att stddev {_mm(oc['ars']['min_att_deg'])} "
                     f"deg, AHRS best att stddev "
                     f"{_mm(oc['ahrs']['min_att_deg'])} deg")
        for who, sub in (("INSLIB", oc), ("ARS", oc["ars"]),
                         ("AHRS", oc["ahrs"])):
            if sub["tripped"]:
                lines.append(f"  WARNING: {who} reported an implausibly "
                             f"small stddev at {sub['n']} epoch(s) -- likely "
                             f"covariance collapse / divergence.")
        if e_pos.n:
            lines.append(f"ins vs ground truth (n={e_pos.n}, after "
                         f"{spec['score']['warmup_sec']:g} s warmup):")
            for nm, s in (("roll", e_roll), ("pitch", e_pitch),
                          ("yaw", e_yaw)):
                lines.append(f"  {nm:5s} error: mean {s.mean():+7.3f}  "
                             f"std {s.std():6.3f} deg")
            lines.append(f"  pos rms: {e_pos.rms():.3f} m")
        lines.append(f"final nav_suite mode: {nav.mode_name()}")

        lines.append("")
        lines.append("data quality summary:")
        lines.append(f"  imu:  {imu_hz:6.1f} Hz avg (n={n_imu_total}, "
                     f"{imu_duration:.1f} s), max gap "
                     f"{imu_max_gap * 1000.0:.0f} ms at "
                     f"t={imu_max_gap_at:.1f} s")

        def _stream_line(label, timestamps_us):
            n, dur, hz, gap, gap_at = replay._stream_gap_stats(timestamps_us)
            if n < 2:
                return f"  {label}: n/a"
            return (f"  {label}: {hz:6.2f} Hz avg (n={n}, {dur:.1f} s), "
                    f"max gap {gap:.2f} s at t={gap_at:.1f} s")
        lines.append(_stream_line("gnss", [fx["t_us"] for fx in fixes]))
        if mags:
            lines.append(_stream_line("mag ", [m[0] for m in mags]))
        if baros:
            lines.append(_stream_line("baro", [b[0] for b in baros]))
        if n_static:
            frac = 100.0 * n_static / n_imu_total if n_imu_total else 0.0
            lines.append(f"  stationary epochs: {n_static}/{n_imu_total} "
                         f"({frac:.1f}% of trial, ins auto-ZUPT/ZARU)")
        else:
            lines.append("  stationary epochs: none detected")

        def _noise_lines(label, unit, static_stat, diff_stat, expect,
                         to_unit):
            out = [f"  {label} noise vs. configured "
                   f"{to_unit(expect):.4f} {unit}:"]
            if static_stat[0].n > 1:
                meas = math.sqrt(sum(s.std() ** 2
                                     for s in static_stat) / 3.0)
                ratio = meas / expect if expect > 0 else math.nan
                out.append(f"    static (stationary epochs):        "
                           f"{to_unit(meas):8.4f} {unit}  (ratio "
                           f"{ratio:.2f}, n={static_stat[0].n})")
            else:
                out.append("    static (stationary epochs):        n/a")
            if diff_stat[0].n > 1:
                meas = (math.sqrt(sum(s.std() ** 2 for s in diff_stat) / 3.0)
                        / math.sqrt(2.0))
                ratio = meas / expect if expect > 0 else math.nan
                out.append(f"    overall (consecutive-sample diff): "
                           f"{to_unit(meas):8.4f} {unit}  (ratio "
                           f"{ratio:.2f}, n={diff_stat[0].n})")
            return out
        if imu_hz > 0:
            dt_nom = 1.0 / imu_hz
            lines += _noise_lines("gyro ", "deg/s", gyr_noise_stat,
                                  gyr_diff_stat,
                                  math.sqrt(noise["gyr_psd"] / dt_nom),
                                  math.degrees)
            lines += _noise_lines("accel", "m/s^2", acc_noise_stat,
                                  acc_diff_stat,
                                  math.sqrt(noise["acc_psd"] / dt_nom),
                                  lambda x: x)

        # --- Dr. INS findings: prioritized health check over everything
        # above (see replay.insdoctor_findings) -- same digest as the CLI's
        # console output and the --plot PDF's cover page.
        insdoctor = replay.insdoctor_findings({
            "aiding_mode": aiding,
            "imu_hz": imu_hz,
            "imu_max_gap": imu_max_gap, "imu_max_gap_at": imu_max_gap_at,
            "imu_rate_hz": imu_rate_hz,
            "gnss": (replay._stream_gap_stats([fx["t_us"] for fx in fixes])
                    if fixes else None),
            "noise": (replay.noise_model_metrics(
                gyr_diff_stat, acc_diff_stat, imu_hz, noise)
                if imu_hz > 0 else None),
            "gnss_acc": (replay.gnss_standstill_accuracy(gnss_static_phases)
                        if aiding == "gnss" else None),
            "gnss_vel": (replay.gnss_vel_accuracy_stats(fixes)
                        if aiding == "gnss" else None),
            "gnss_max_hor_vel_stddev_mps": gnss_cfg["max_horizontal_vel_stddev_mps"],
            "baro_acc": (replay.channel_standstill_accuracy(
                baro_static_phases, float(baro_cfg["stddev_m"]))
                if baros else None),
            "mag_acc": (replay.channel_standstill_accuracy(
                mag_static_phases, mag_sd) if mags else None),
            "health": {"n": health_n, "limits": replay.STDDEV_LIMITS,
                      "stats": health_stats},
            "overconfidence": oc,
            "diag": diag,
        })
        lines.append("")
        lines += replay.insdoctor_lines(insdoctor)

        final_mode = nav.mode_name()
        nav.close()

        do_gnss_delay = (aiding == "gnss" and bool(baros)
                         and any(fx.get("vel_ok")
                                 and fx.get("cov_vel") is not None
                                 for fx in fixes))
        gnss_delay_curve = None
        if do_gnss_delay:
            with self.lock:
                gnss_delay_curve = replay.gnss_delay_correlation_curve(
                    rec["t"], rec["baro_vel_d"], rec["gnss_vel_d"])
            if gnss_delay_curve is None:
                lines.append("gnss delay estimate: not enough data")
            else:
                delay_ms, corr = max(gnss_delay_curve, key=lambda r: r[1])
                quality = ("good" if corr > 0.7 else
                           "weak -- probably not enough vertical motion"
                           if corr > 0.3 else
                           "poor -- do not trust this number")
                lines.append(f"gnss delay estimate: {delay_ms:.0f} ms "
                             f"(correlation {corr:.2f}, {quality}) -- "
                             f"relative to baro_alt (which has its own "
                             f"group delay)")

        return {
            "text": "\n".join(lines),
            "rec": rec,
            "sensor_rate": sensor_rate,
            "gnss_delay_curve": gnss_delay_curve,
            "findings": insdoctor,
            "growth_rate": replay.process_noise_growth_rate(noise),
            "baro_growth_rate": replay.baro_alt_growth_rate(baro_cfg),
            "ahrs_growth_rate": replay.ahrs_growth_rate(ahrs_cfg),
            "name": spec["name"] or os.path.basename(
                os.path.normpath(data_dir)),
            "warmup_sec": float(spec["score"]["warmup_sec"]),
            "warmup_end_sec": (t_warmup_end - t0_us) / US_PER_SEC
            if t0_us is not None else 0.0,
            "kml_est": kml_est,
            "kml_ref": kml_ref,
            "kml_fix": kml_fix,
            "aborted": aborted,
            "mode": final_mode,
            "pos_rms_m": e_pos.rms(),
            "scored_epochs": e_pos.n,
        }

    # ------------------------------------------------------------------
    def _record(self, rec, nav, t, t0_us, last_ref, last_fix, last_baro,
                origin_ecef, origin_lat, origin_lon, score_la):
        """One --plot-recorder tick, identical to replay.py's rec block
        (all lists stay the same length as rec['t'], NaN-padded per
        group's own availability)."""
        nan3 = [math.nan] * 3
        row = {}
        row["t"] = (t - t0_us) / US_PER_SEC
        row["zupt_active"] = 1.0 if nav.auto_zupt_active() else 0.0
        dw_now = nav.downweight_counts()
        row["dw_full3d"] = float(nav.diag()["n_downweighted"])
        row["dw_ars"] = float(dw_now["ars"])
        row["dw_ahrs"] = float(dw_now["ahrs"])
        row["dw_baro_alt"] = float(dw_now["baro_alt"])
        row["dw_local_gnss"] = float(dw_now["local_gnss"])

        # ins's own local NED frame, straight out of the filter, plotted
        # against a ground truth converted with the origin latched once in
        # the replay loop -- so the two only agree as long as that origin
        # stays put. It does across a GNSS quality-loss re-arm: ins carries
        # it (REQ-NAV-062) and the wrapper no longer re-aligns a carried
        # origin onto the barometric datum (REQ-SUITE-007), the two halves
        # of the case that used to move it mid-run. It can still change on
        # a time-jump reset (REQ-NAV-016), a bootstrap beyond the carry
        # distance, or the wrapper's datum shift onto a freshly anchored
        # origin -- but ins goes uninitialized first in all of those, so
        # the samples in between are NaN and the trail breaks there rather
        # than drawing a line into the new frame.
        pos = nav.position_local()
        vel = nav.velocity_ned()
        rpy = nav.rpy_ins()
        sd = nav.stddev()
        if pos is not None and vel is not None and rpy is not None and sd:
            row["pos"] = list(pos)
            row["pos_sigma"] = list(sd["pos_ned"])
            row["vel"] = list(vel)
            row["vel_sigma"] = list(sd["vel_ned"])
            row["rpy_deg"] = [math.degrees(x) for x in rpy]
            row["rpy_sigma_deg"] = [math.degrees(x) for x in sd["rpy"]]
            acc_bias = nav.bias_acc()
            row["acc_bias"] = list(acc_bias) if acc_bias else nan3[:]
            row["acc_bias_sigma"] = list(sd["acc_bias"])
            gyr_bias = nav.bias_gyr()
            row["gyr_bias"] = list(gyr_bias) if gyr_bias else nan3[:]
            row["gyr_bias_sigma"] = list(sd["gyr_bias"])
            mag_bias = nav.bias_mag()
            row["mag_bias"] = list(mag_bias) if mag_bias else nan3[:]
            row["mag_bias_sigma"] = (list(sd["mag_bias"])
                                     if "mag_bias" in sd else nan3[:])
        else:
            for k in ("pos", "vel", "pos_sigma", "vel_sigma", "rpy_deg",
                      "rpy_sigma_deg", "acc_bias", "acc_bias_sigma",
                      "gyr_bias", "gyr_bias_sigma", "mag_bias",
                      "mag_bias_sigma"):
                row[k] = nan3[:]

        for pfx, rpy_fn, rpy_sd_fn, bias_fn, sd_fn in (
                ("ars", nav.rpy_ars, nav.rpy_stddev_ars, nav.bias_gyr_ars,
                 nav.gyr_bias_stddev_ars),
                ("ahrs", nav.rpy_ahrs, nav.rpy_stddev_ahrs, nav.bias_gyr_ahrs,
                 nav.gyr_bias_stddev_ahrs)):
            sub_rpy = rpy_fn()
            row[f"{pfx}_rpy_deg"] = ([math.degrees(x) for x in sub_rpy]
                                     if sub_rpy else nan3[:])
            sub_rpy_sd = rpy_sd_fn()
            row[f"{pfx}_rpy_sigma_deg"] = ([math.degrees(x) for x in sub_rpy_sd]
                                           if sub_rpy_sd else nan3[:])
            sub_bias = bias_fn()
            row[f"{pfx}_gyr_bias"] = (list(sub_bias) if sub_bias
                                      else nan3[:])
            sub_sd = sd_fn()
            row[f"{pfx}_gyr_bias_sigma"] = (list(sub_sd) if sub_sd
                                            else nan3[:])

        baro = nav.baro_alt()
        row["baro_h_d"] = -baro[0] if baro is not None else math.nan
        row["baro_vel_d"] = -baro[1] if baro is not None else math.nan
        row["baro_raw_d"] = (-replay.isa_pressure_to_altitude(last_baro[1])
                             if last_baro is not None else math.nan)
        baro_bias = nav.baro_acc_bias()
        row["baro_acc_bias"] = (baro_bias if baro_bias is not None
                                else math.nan)
        baro_sd = nav.baro_stddev()
        row["baro_h_sigma"] = baro_sd[0] if baro_sd is not None else math.nan
        row["baro_acc_bias_sigma"] = (baro_sd[2] if baro_sd is not None
                                      else math.nan)
        gnss_offset = nav.local_gnss_offset()
        row["local_gnss_offset"] = (gnss_offset[0]
                                   if gnss_offset is not None else math.nan)
        row["local_gnss_offset_sigma"] = (gnss_offset[1]
                                         if gnss_offset is not None
                                         else math.nan)
        row["gnss_vel_d"] = (last_fix["vel_ned"][2]
                             if last_fix is not None else math.nan)
        if last_fix is not None and origin_ecef is not None:
            row["fix_pos_d"] = replay.ref_to_local_ned(
                last_fix, origin_ecef, origin_lat, origin_lon)[2]
        else:
            row["fix_pos_d"] = math.nan
        if last_fix is not None and last_fix.get("cov_pos") is not None:
            cov_p = last_fix["cov_pos"]
            row["gnss_pos_sigma"] = [math.sqrt(cov_p[0][0]),
                                     math.sqrt(cov_p[1][1]),
                                     math.sqrt(cov_p[2][2])]
            cov_v = last_fix.get("cov_vel")
            row["gnss_vel_sigma"] = (
                [math.sqrt(cov_v[0][0]), math.sqrt(cov_v[1][1]),
                 math.sqrt(cov_v[2][2])]
                if last_fix.get("vel_ok") and cov_v is not None else nan3[:])
        else:
            row["gnss_pos_sigma"] = nan3[:]
            row["gnss_vel_sigma"] = nan3[:]

        if last_ref is not None and origin_ecef is not None:
            err_ecef = (replay.pos_error_ecef(nav, last_ref, score_la)
                        if nav.is_ready() else None)
            row["ref_pos"] = replay.ref_to_local_ned(
                last_ref, origin_ecef, origin_lat, origin_lon)
            row["ref_vel"] = list(last_ref["vel_ned"])
            row["ref_rpy_deg"] = [math.degrees(last_ref["roll_rad"]),
                                  math.degrees(last_ref["pitch_rad"]),
                                  math.degrees(last_ref["yaw_rad"])]
            row["pos_err_ned"] = (replay._matvec_rm_T(
                replay.ned_to_ecef_rot(origin_lat, origin_lon), err_ecef)
                if err_ecef is not None else nan3[:])
            rpy = nav.rpy_ins()
            row["rpy_err_deg"] = (nan3[:] if rpy is None else [
                math.degrees(replay.wrap_pi(rpy[0] - last_ref["roll_rad"])),
                math.degrees(replay.wrap_pi(rpy[1] - last_ref["pitch_rad"])),
                math.degrees(replay.wrap_pi(rpy[2] - last_ref["yaw_rad"]))])
        else:
            for k in ("ref_pos", "ref_vel", "ref_rpy_deg", "pos_err_ned",
                      "rpy_err_deg"):
                row[k] = nan3[:]

        with self.lock:
            for k in REC_KEYS:
                rec[k].append(row[k])

    def _update_live(self, nav, t, t0_us, duration, acc, gyr):
        rpy = nav.rpy()  # best available: ins, else AHRS, else ARS
        pos = nav.position_local()
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


# ============================================================================
# 3D views (lifted from znav3d.py, MAVLink specifics removed, ground-truth
# trail added)
# ============================================================================

COLOR_BG = (18, 20, 24)
COLOR_GRID = (60, 65, 75)
COLOR_POSITION = (255, 90, 90)
COLOR_ELLIPSOID = (100, 200, 255, 40)
COLOR_REF_TRAIL = (140, 240, 140, 140)
TRAIL_MAX_POINTS = 100_000
FLY_MOVE_SPEED = 8.0
FLY_SPEED_BOOST = 4.0
FLY_MOUSE_SENS = 0.15
FLY_TICK_MS = 16
ANTIALIAS = False


def _speeds_to_rgba(speeds, scale):
    """Vectorized speed [m/s] -> RGBA (blue = slow, red = fast)."""
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
    out[:, 0], out[:, 1], out[:, 2], out[:, 3] = r, g, b, 0.9
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


class Position3DView(gl.GLViewWidget):
    fly_exit_requested = QtCore.pyqtSignal()
    manual_pan_requested = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundColor(*COLOR_BG)
        self.opts['distance'] = 50
        self.opts['elevation'] = 35
        self.opts['azimuth'] = 170

        self.grid = gl.GLGridItem(color=(*COLOR_GRID, 150))
        self.grid.setSize(x=2000, y=2000)
        self.grid.setSpacing(x=50, y=50)
        self.addItem(self.grid)
        L = 20
        self.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [L, 0, 0]]),
                                       color=(1, 0.3, 0.3, 1), width=2))
        self.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, -L, 0]]),
                                       color=(0.3, 1, 0.3, 1), width=2))
        self.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, L]]),
                                       color=(0.3, 0.6, 1, 1), width=2))

        self.trail_points = []      # display frame (x, -y, -z)
        self.trail_speeds = []
        # Per point: does the trail START here (no segment drawn from the
        # previous point)? Set when the filter published nothing in
        # between, e.g. while it is re-arming after an outage. Drawn as
        # 'lines' rather than 'line_strip' for exactly this reason: a strip
        # would bridge the gap with a straight chord, which looks like a
        # trajectory the vehicle never flew.
        self.trail_break = []
        self._speed_scale = 5.0
        self._last_obj_pos = (0.0, 0.0, 0.0)
        self.trail_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32),
            color=np.zeros((2, 4), dtype=np.float32),
            width=1.5, antialias=ANTIALIAS, mode='lines')
        self.addItem(self.trail_item)

        self.ref_points = []
        self.ref_item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=np.float32),
            color=tuple(c / 255 for c in COLOR_REF_TRAIL),
            width=1.0, antialias=ANTIALIAS, mode='line_strip')
        self.addItem(self.ref_item)

        self.position_item = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]),
            color=np.array([[*[c / 255 for c in COLOR_POSITION], 1.0]]),
            size=10)
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

        # --- fly / WASD free-look (znav3d) ---
        self.fly_enabled = False
        self.invert_mouse = False
        self._fly_keys = set()
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self._fly_timer = QtCore.QTimer(self)
        self._fly_timer.setInterval(FLY_TICK_MS)
        self._fly_timer.timeout.connect(self._fly_step)
        self._last_fly_time = None
        self._last_mouse_pos = None
        self._can_warp_cursor = (
            QtGui.QGuiApplication.platformName() != "wayland")

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
        if smax > self._speed_scale:
            self._speed_scale = smax * 1.2  # recolor everything below
        if len(self.trail_points) >= 2:
            pts = np.array(self.trail_points, dtype=np.float32)
            col = _speeds_to_rgba(self.trail_speeds, self._speed_scale)
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
                color=(*[c / 255 for c in COLOR_ELLIPSOID[:3]],
                       COLOR_ELLIPSOID[3] / 255),
                shader='shaded', glOptions='translucent')
            self.addItem(self.ellipsoid_item)
        else:
            self.ellipsoid_item.setMeshData(meshdata=mesh_data)
            self.ellipsoid_item.setVisible(True)

    def focus_on_object(self):
        x, y, z = self._last_obj_pos
        self.setCameraPosition(pos=pg.Vector(x, y, z), distance=40)

    def fit_trail(self):
        pts = self.trail_points or self.ref_points
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
        self._speed_scale = 5.0
        z2 = np.zeros((2, 3), dtype=np.float32)
        self.trail_item.setData(pos=z2,
                                color=np.zeros((2, 4), dtype=np.float32))
        self.ref_item.setData(pos=z2)

    # --- fly mode (verbatim from znav3d) ------------------------------
    def set_fly_enabled(self, enabled):
        self.fly_enabled = enabled
        self.setMouseTracking(enabled)
        self._fly_keys.clear()
        if enabled:
            self.setFocus()
            self.setCursor(QtCore.Qt.CursorShape.BlankCursor)
            self._last_fly_time = None
            self._last_mouse_pos = None
            self._recenter_cursor()
            self._fly_timer.start()
        else:
            self._fly_timer.stop()
            self._last_mouse_pos = None
            self.unsetCursor()

    def _recenter_cursor(self):
        if not self._can_warp_cursor:
            return
        center = self.rect().center()
        QtGui.QCursor.setPos(self.mapToGlobal(center))
        self._last_mouse_pos = center

    def keyPressEvent(self, ev):
        if self.fly_enabled:
            if ev.key() == QtCore.Qt.Key.Key_Escape:
                self.fly_exit_requested.emit()
                ev.accept()
                return
            if not ev.isAutoRepeat():
                self._fly_keys.add(ev.key())
            ev.accept()
            return
        super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev):
        if self.fly_enabled:
            if not ev.isAutoRepeat():
                self._fly_keys.discard(ev.key())
            ev.accept()
            return
        super().keyReleaseEvent(ev)

    def mousePressEvent(self, ev):
        # Middle-button drag is GLViewWidget's own pan gesture: honour the
        # user's manual pan instead of snapping the camera back to the
        # followed object on the next update tick.
        if (not self.fly_enabled
                and ev.button() == QtCore.Qt.MouseButton.MiddleButton):
            self.manual_pan_requested.emit()
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if not self.fly_enabled:
            super().mouseMoveEvent(ev)
            return
        pos = ev.position().toPoint()
        if self._last_mouse_pos is None:
            self._last_mouse_pos = pos
            ev.accept()
            return
        dx = pos.x() - self._last_mouse_pos.x()
        dy = pos.y() - self._last_mouse_pos.y()
        self._last_mouse_pos = pos
        if dx or dy:
            self.opts['azimuth'] = (self.opts['azimuth']
                                    + dx * FLY_MOUSE_SENS) % 360
            pitch = dy if self.invert_mouse else -dy
            elev = self.opts['elevation'] + pitch * FLY_MOUSE_SENS
            self.opts['elevation'] = max(-89.9, min(89.9, elev))
            self.update()
            rect = self.rect()
            margin = 0.25
            if (pos.x() < rect.width() * margin
                    or pos.x() > rect.width() * (1 - margin)
                    or pos.y() < rect.height() * margin
                    or pos.y() > rect.height() * (1 - margin)):
                self._recenter_cursor()
        ev.accept()

    def _fly_step(self):
        now = time.perf_counter()
        dt = (now - self._last_fly_time) if self._last_fly_time else 0.0
        self._last_fly_time = now
        if not self._fly_keys or dt <= 0.0:
            return
        K = QtCore.Qt.Key
        az = math.radians(self.opts['azimuth'])
        fwd = np.array([-math.cos(az), -math.sin(az), 0.0])
        right = np.array([-math.sin(az), math.cos(az), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        move = np.zeros(3)
        if K.Key_W in self._fly_keys:
            move += fwd
        if K.Key_S in self._fly_keys:
            move -= fwd
        if K.Key_D in self._fly_keys:
            move += right
        if K.Key_A in self._fly_keys:
            move -= right
        if K.Key_E in self._fly_keys or K.Key_Space in self._fly_keys:
            move += up
        if K.Key_Q in self._fly_keys:
            move -= up
        norm = np.linalg.norm(move)
        if norm == 0.0:
            return
        speed = FLY_MOVE_SPEED * (FLY_SPEED_BOOST
                                  if K.Key_Shift in self._fly_keys else 1.0)
        delta = move / norm * speed * dt
        c = self.opts['center']
        self.opts['center'] = pg.Vector(c.x() + delta[0], c.y() + delta[1],
                                        c.z() + delta[2])
        self.update()


class AttitudeView(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackgroundColor(*COLOR_BG)
        self.opts['distance'] = 4
        self.opts['elevation'] = 25
        self.opts['azimuth'] = 170
        self.addItem(gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [1.5, 0, 0]]),
            color=(1, 0.3, 0.3, 1), width=2))
        self.addItem(gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, -1.5, 0]]),
            color=(0.3, 1, 0.3, 1), width=2))
        self.addItem(gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, 1.5]]),
            color=(0.3, 0.6, 1, 1), width=2))
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
    FILTER_ALPHA = 0.15

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(80, 80)
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.scale_g = 1.5

    def set_acceleration(self, ax_g, ay_g):
        a = self.FILTER_ALPHA
        self.acc_x += a * (ax_g - self.acc_x)
        self.acc_y += a * (ay_g - self.acc_y)
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r = (min(w, h) / 2 - 4) * 0.95
        p.setPen(QtGui.QPen(QtGui.QColor(80, 90, 100), 1))
        for frac in (0.33, 0.66, 1.0):
            p.drawEllipse(QtCore.QPointF(cx, cy), r * frac, r * frac)
        p.drawLine(QtCore.QPointF(cx - r, cy), QtCore.QPointF(cx + r, cy))
        p.drawLine(QtCore.QPointF(cx, cy - r), QtCore.QPointF(cx, cy + r))
        bx = cx + max(-1, min(1, self.acc_y / self.scale_g)) * r
        by = cy - max(-1, min(1, self.acc_x / self.scale_g)) * r
        p.setPen(QtGui.QPen(QtGui.QColor(255, 220, 100), 2))
        p.setBrush(QtGui.QBrush(QtGui.QColor(255, 180, 60, 180)))
        p.drawEllipse(QtCore.QPointF(bx, by), 5, 5)


# ============================================================================
# Config editor
# ============================================================================

class ConfigEditor(QtWidgets.QScrollArea):
    """Schema-driven form over config.yaml. Keys outside the schema (e.g.
    the foreign origin:/crazyflie: sections, truth:) are preserved
    untouched."""

    changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._raw = {}
        self._widgets = {}  # path -> (kind, extra, widget)
        body = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(body)
        for section, fields in CONFIG_SECTIONS:
            box = QtWidgets.QGroupBox(section)
            form = QtWidgets.QFormLayout(box)
            for path, label, kind, extra, tip in fields:
                w = self._make_widget(kind, extra)
                if tip:
                    w.setToolTip(tip)
                form.addRow(label, w)
                self._widgets[path] = (kind, extra, w)
            lay.addWidget(box)
        lay.addStretch()
        self.setWidget(body)

    @staticmethod
    def _make_widget(kind, extra):
        if kind == BOOL:
            return QtWidgets.QCheckBox()
        if kind == CHOICE:
            cb = QtWidgets.QComboBox()
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

    def collect(self):
        """Returns (raw_config, errors). Values equal to the schema default
        are only written if the key was already present in the file."""
        raw = copy.deepcopy(self._raw)
        errors = []
        for path, (kind, extra, w) in self._widgets.items():
            name = ".".join(path)
            try:
                if kind == BOOL:
                    val = 1 if w.isChecked() else 0
                elif kind == CHOICE:
                    val = w.currentText()
                elif kind == STR:
                    val = w.text().strip()
                elif kind == INT:
                    txt = w.text().strip()
                    val = int(float(txt)) if txt else 0
                elif kind == VEC:
                    parts = [p for p in
                             w.text().replace(",", " ").split() if p]
                    if not parts:
                        val = [0.0] * extra
                    else:
                        val = [float(p) for p in parts]
                    if len(val) != extra:
                        raise ValueError(f"expected {extra} values")
                else:
                    txt = w.text().strip()
                    val = float(txt) if txt else 0.0
            except ValueError as e:
                errors.append(f"{name}: {e}")
                continue
            dflt = _schema_default(path, kind, extra)
            if isinstance(dflt, (list, tuple)):
                differs = list(val) != [float(x) for x in dflt]
            elif kind == BOOL:
                differs = val != int(dflt)
            else:
                differs = val != dflt
            if differs or _get_path(self._raw, path) is not None:
                _set_path(raw, path, val)
        return raw, errors


# ============================================================================
# Post-run plots
# ============================================================================

PEN_N = pg.mkPen('#ff6a6a', width=1.5)
PEN_E = pg.mkPen('#5af08a', width=1.5)
PEN_D = pg.mkPen('#54aaff', width=1.5)
PEN_REF = pg.mkPen('#e8e8e8', width=1.5, style=QtCore.Qt.PenStyle.DashLine)
PEN_SIG = pg.mkPen('#b0b0b0', width=1.0, style=QtCore.Qt.PenStyle.DotLine)
PEN_WARM = pg.mkPen('#ffb454', width=1.0, style=QtCore.Qt.PenStyle.DashLine)


def populate_plots(glw, rec, warmup_end_sec):
    """Fill a GraphicsLayoutWidget with the post-run evaluation plots."""
    glw.clear()
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
    gyr_bias = arr("gyr_bias")
    acc_bias = arr("acc_bias")
    gyr_bias_sig = arr("gyr_bias_sigma")
    acc_bias_sig = arr("acc_bias_sigma")

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
        p.plot(t, pos_err[:, i], pen=pen, connect="finite")
        p.plot(t, pos_sig[:, i], pen=PEN_SIG, connect="finite")
        p.plot(t, -pos_sig[:, i], pen=PEN_SIG, connect="finite")
    glw.nextRow()
    # Row 2: attitude error + 1-sigma band
    for i, (axis, pen) in enumerate((("roll", PEN_N), ("pitch", PEN_E),
                                     ("yaw", PEN_D))):
        p = add_time_plot(f"{axis} error [deg]", "deg")
        p.plot(t, rpy_err[:, i], pen=pen, connect="finite")
        p.plot(t, rpy_sig[:, i], pen=PEN_SIG, connect="finite")
        p.plot(t, -rpy_sig[:, i], pen=PEN_SIG, connect="finite")
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

    alt = add_time_plot("altitude (-D) [m]", "m")
    alt.addLegend(offset=(10, 10))
    alt_est = -pos[:, 2]
    alt_sig = pos_sig[:, 2]
    alt.plot(t, alt_est, pen=PEN_N, connect="finite", name="ins")
    alt.plot(t, alt_est + alt_sig, pen=PEN_SIG, connect="finite")
    alt.plot(t, alt_est - alt_sig, pen=PEN_SIG, connect="finite")
    alt.plot(t, -ref_pos[:, 2], pen=PEN_REF, connect="finite", name="truth")
    baro_h = arr("baro_h_d")
    if np.isfinite(baro_h).any():
        alt.plot(t, -baro_h, pen=PEN_E, connect="finite", name="baro_alt")
    fix_d = arr("fix_pos_d")
    if np.isfinite(fix_d).any():
        alt.plot(t, -fix_d, pen=pg.mkPen('#c080ff', width=1.0),
                 connect="finite", name="gnss")

    dwp = add_time_plot("chi2 downweight counters", "count")
    dwp.addLegend(offset=(10, 10))
    for key, pen, name in (("dw_full3d", PEN_N, "ins"),
                           ("dw_ars", PEN_E, "ars"),
                           ("dw_ahrs", PEN_D, "ahrs"),
                           ("dw_baro_alt",
                            pg.mkPen('#c080ff', width=1.5), "baro_alt")):
        dwp.plot(t, arr(key), pen=pen, connect="finite", name=name)
    glw.nextRow()

    # Row 4: biases + 1-sigma band
    gb = add_time_plot("ins gyro bias [deg/s]", "deg/s")
    gb.addLegend(offset=(10, 10))
    for i, (axis, pen) in enumerate((("x", PEN_N), ("y", PEN_E),
                                     ("z", PEN_D))):
        center = np.degrees(gyr_bias[:, i])
        sigma = np.degrees(gyr_bias_sig[:, i])
        gb.plot(t, center, pen=pen, connect="finite", name=axis)
        gb.plot(t, center + sigma, pen=PEN_SIG, connect="finite")
        gb.plot(t, center - sigma, pen=PEN_SIG, connect="finite")
    ab = add_time_plot("ins accel bias [m/s^2]", "m/s^2")
    ab.addLegend(offset=(10, 10))
    for i, (axis, pen) in enumerate((("x", PEN_N), ("y", PEN_E),
                                     ("z", PEN_D))):
        center = acc_bias[:, i]
        sigma = acc_bias_sig[:, i]
        ab.plot(t, center, pen=pen, connect="finite", name=axis)
        ab.plot(t, center + sigma, pen=PEN_SIG, connect="finite")
        ab.plot(t, center - sigma, pen=PEN_SIG, connect="finite")
    vel = arr("vel")
    ref_vel = arr("ref_vel")
    sp = add_time_plot("horizontal speed [m/s]", "m/s")
    sp.addLegend(offset=(10, 10))
    sp.plot(t, np.hypot(vel[:, 0], vel[:, 1]), pen=PEN_N,
            connect="finite", name="ins")
    sp.plot(t, np.hypot(ref_vel[:, 0], ref_vel[:, 1]), pen=PEN_REF,
            connect="finite", name="truth")

    for p in time_plots:
        p.setLabel('bottom', 't [s]')


# ============================================================================
# Main window
# ============================================================================

UI_UPDATE_HZ = 30
GRAVITY = 9.80665

STYLESHEET = (
    "QMainWindow, QWidget { background: #0f1216; color: #d8dee6; } "
    "QFrame#panel { background: #171b21; border: 1px solid #252c35; "
    "border-radius: 6px; } "
    "QGroupBox { border: 1px solid #252c35; border-radius: 6px; "
    "margin-top: 8px; padding-top: 12px; } "
    "QGroupBox::title { color: #8fa0b4; left: 8px; } "
    "QPushButton { background: #1f2832; border: 1px solid #2b3642; "
    "border-radius: 4px; padding: 6px 12px; color: #d8dee6; } "
    "QPushButton:hover { background: #283441; } "
    "QPushButton:disabled { color: #5a6470; } "
    "QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit { "
    "background: #12151a; border: 1px solid #2b3642; padding: 3px; "
    "color: #d8dee6; } "
    "QCheckBox::indicator { width: 13px; height: 13px; border: 1px solid "
    "#3a4654; border-radius: 3px; background: #12151a; } "
    "QCheckBox::indicator:checked { background: #54aaff; } "
    "QTabWidget::pane { border: 1px solid #252c35; } "
    "QTabBar::tab { background: #171b21; padding: 6px 14px; } "
    "QTabBar::tab:selected { background: #283441; }"
)

MONO = "font-family: Consolas, 'DejaVu Sans Mono', monospace; font-size: 12px;"


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("inspostgui — INSLIB post-processing")
        self.resize(1700, 980)
        self.setStyleSheet(STYLESHEET)
        pg.setConfigOptions(antialias=ANTIALIAS)

        self.settings = QtCore.QSettings("zwiener", "inspostgui")
        self.data_dir = None
        self.cfg_path = None
        self.worker = None
        self.results = None
        self._drained = 0
        self._alt_t, self._alt_est, self._alt_ref = [], [], []
        self._alt_has_finite = False
        self._trail_gap = False
        self._smooth_quat = None

        self._build_toolbar()
        self._build_central()
        self._build_statusbar()
        self._populate_datasets()

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
        browse.clicked.connect(self._on_browse)
        tb.addWidget(browse)
        tb.addSeparator()

        self.run_btn = QtWidgets.QPushButton("▶ Run")
        self.run_btn.clicked.connect(self._on_run)
        tb.addWidget(self.run_btn)
        self.pause_btn = QtWidgets.QPushButton("⏸ Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setEnabled(False)
        self.pause_btn.toggled.connect(self._on_pause)
        tb.addWidget(self.pause_btn)
        self.stop_btn = QtWidgets.QPushButton("■ Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._on_stop)
        tb.addWidget(self.stop_btn)
        tb.addSeparator()

        self.chk_realtime = QtWidgets.QCheckBox("Realtime")
        self.chk_realtime.setToolTip("Pace the replay to wall-clock time "
                                     "(x speed factor); off = full speed")
        tb.addWidget(self.chk_realtime)
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 100.0)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setSingleStep(0.5)
        self.speed_spin.setPrefix("x")
        self.speed_spin.setMaximumWidth(80)
        tb.addWidget(self.speed_spin)
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

        self.allan_btn = QtWidgets.QPushButton("Estimate bias RW")
        self.allan_btn.setToolTip("Allan variance on a long STATIC imu.csv "
                                  "(ideally a few hours, the bias random walk "
                                  "only shows up at long averaging times) -> "
                                  "suggests imu.gyr_bias_rw / acc_bias_rw for "
                                  "config.yaml (needs numpy)")
        self.allan_btn.clicked.connect(self._on_estimate_bias_rw)
        tb.addWidget(self.allan_btn)

    def _build_central(self):
        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        # --- Replay tab ---
        replay_w = QtWidgets.QWidget()
        root = QtWidgets.QHBoxLayout(replay_w)
        root.setContentsMargins(6, 6, 6, 6)
        left = QtWidgets.QVBoxLayout()
        self.pos_view = Position3DView()
        left.addWidget(self.pos_view, 1)
        bottom = QtWidgets.QHBoxLayout()
        self.chk_follow = QtWidgets.QCheckBox("Follow")
        self.chk_follow.setChecked(True)
        bottom.addWidget(self.chk_follow)
        self.chk_model = QtWidgets.QCheckBox("Show 3D model")
        bottom.addWidget(self.chk_model)
        self.chk_fly = QtWidgets.QCheckBox("Fly (WASD)")
        self.chk_fly.setToolTip("WASD = move, Q/E or Space = down/up, "
                                "mouse = look, Shift = faster, Esc = exit")
        self.chk_fly.toggled.connect(self._on_fly_toggled)
        self.pos_view.fly_exit_requested.connect(
            lambda: self.chk_fly.setChecked(False))
        self.pos_view.manual_pan_requested.connect(
            lambda: self.chk_follow.setChecked(False))
        bottom.addWidget(self.chk_fly)
        self.chk_invert = QtWidgets.QCheckBox("Invert mouse")
        self.chk_invert.toggled.connect(
            lambda c: setattr(self.pos_view, "invert_mouse", c))
        bottom.addWidget(self.chk_invert)
        bottom.addStretch()
        fit_btn = QtWidgets.QPushButton("Fit trail")
        fit_btn.clicked.connect(self.pos_view.fit_trail)
        bottom.addWidget(fit_btn)
        focus_btn = QtWidgets.QPushButton("Focus object")
        focus_btn.clicked.connect(self.pos_view.focus_on_object)
        bottom.addWidget(focus_btn)
        left.addLayout(bottom)
        root.addLayout(left, 3)
        root.addWidget(self._build_side_panel())
        self.tabs.addTab(replay_w, "Replay")

        # --- Config tab ---
        cfg_w = QtWidgets.QWidget()
        cfg_lay = QtWidgets.QVBoxLayout(cfg_w)
        row = QtWidgets.QHBoxLayout()
        self.cfg_label = QtWidgets.QLabel("no config loaded")
        self.cfg_label.setStyleSheet("color: #8fa0b4;")
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
        self.files_label.setStyleSheet("color: #8fa0b4; " + MONO)
        cfg_lay.addWidget(self.files_label)
        self.editor = ConfigEditor()
        cfg_lay.addWidget(self.editor, 1)
        self.tabs.addTab(cfg_w, "Config")

        # --- Plots tab ---
        plots_scroll = QtWidgets.QScrollArea()
        plots_scroll.setWidgetResizable(True)
        self.plots_widget = pg.GraphicsLayoutWidget()
        self.plots_widget.setBackground(QtGui.QColor(*COLOR_BG))
        self.plots_widget.setMinimumHeight(1100)
        plots_scroll.setWidget(self.plots_widget)
        self.tabs.addTab(plots_scroll, "Plots")

        # --- Map tab ---
        self.map_view = MapView()
        self.tabs.addTab(self.map_view, "Map")

        # --- Summary tab ---
        self.summary_text = QtWidgets.QPlainTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setStyleSheet(MONO)
        self.tabs.addTab(self.summary_text, "Summary")

    def _build_side_panel(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(420)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)

        def frame(title):
            f = QtWidgets.QFrame()
            f.setObjectName("panel")
            v = QtWidgets.QVBoxLayout(f)
            h = QtWidgets.QLabel(title)
            h.setStyleSheet("color: #8fa0b4; font-size: 11px;")
            v.addWidget(h)
            lay.addWidget(f)
            return v

        v = frame("Status")
        self.lbl_status = QtWidgets.QLabel("idle")
        self.lbl_status.setStyleSheet(MONO)
        self.lbl_status.setWordWrap(True)
        v.addWidget(self.lbl_status)
        self.lbl_mode = QtWidgets.QLabel("")
        self.lbl_mode.setStyleSheet(MONO)
        v.addWidget(self.lbl_mode)

        v = frame("Attitude (± 1-sigma)")
        h = QtWidgets.QHBoxLayout()
        num = QtWidgets.QVBoxLayout()
        self.lbl_roll = QtWidgets.QLabel()
        self.lbl_pitch = QtWidgets.QLabel()
        self.lbl_yaw = QtWidgets.QLabel()
        for lbl in (self.lbl_roll, self.lbl_pitch, self.lbl_yaw):
            lbl.setStyleSheet(MONO)
            num.addWidget(lbl)
        num.addStretch()
        h.addLayout(num, 1)
        self.attitude_view = AttitudeView()
        self.attitude_view.setMinimumSize(160, 160)
        h.addWidget(self.attitude_view, 1)
        v.addLayout(h)

        v = frame("Bias estimates (± 1-sigma)")
        self.lbl_gyr_bias = QtWidgets.QLabel()
        self.lbl_acc_bias = QtWidgets.QLabel()
        for lbl in (self.lbl_gyr_bias, self.lbl_acc_bias):
            lbl.setStyleSheet(MONO)
            lbl.setWordWrap(True)
            v.addWidget(lbl)

        v = frame("Position / Velocity (± 1-sigma)")
        self.lbl_pos = QtWidgets.QLabel()
        self.lbl_llh = QtWidgets.QLabel()
        self.lbl_baro = QtWidgets.QLabel()
        self.lbl_vel = QtWidgets.QLabel()
        for lbl in (self.lbl_pos, self.lbl_llh, self.lbl_baro, self.lbl_vel):
            lbl.setStyleSheet(MONO)
            lbl.setWordWrap(True)
            v.addWidget(lbl)

        v = frame("Altitude (-D) [m]")
        self.alt_plot = pg.PlotWidget()
        self.alt_plot.setBackground(QtGui.QColor(*COLOR_BG))
        self.alt_plot.showGrid(x=True, y=True, alpha=0.2)
        self.alt_plot.setMinimumHeight(140)
        self.alt_curve_est = self.alt_plot.plot(
            pen=pg.mkPen((100, 200, 255), width=2))
        self.alt_curve_ref = self.alt_plot.plot(pen=PEN_REF)
        v.addWidget(self.alt_plot)

        v = frame("Acceleration (body)")
        h = QtWidgets.QHBoxLayout()
        col = QtWidgets.QVBoxLayout()
        self.lbl_acc = QtWidgets.QLabel()
        self.lbl_dw = QtWidgets.QLabel()
        self.lbl_dw.setWordWrap(True)
        for lbl in (self.lbl_acc, self.lbl_dw):
            lbl.setStyleSheet(MONO)
            col.addWidget(lbl)
        col.addStretch()
        h.addLayout(col, 1)
        self.g_bubble = GBubbleWidget()
        h.addWidget(self.g_bubble)
        v.addLayout(h)

        lay.addStretch()
        return panel

    def _build_statusbar(self):
        self.progress = QtWidgets.QProgressBar()
        self.progress.setMaximumWidth(300)
        self.progress.setRange(0, 100)
        self.statusBar().addPermanentWidget(self.progress)
        self.statusBar().showMessage("ready")

    # ------------------------------------------------------------------
    def _populate_datasets(self):
        self.dataset_combo.clear()
        seen = set()
        for label, path in discover_datasets():
            self.dataset_combo.addItem(label, path)
            seen.add(os.path.abspath(path))
        recents = self.settings.value("recents", [], type=list)
        for path in recents:
            if os.path.abspath(path) not in seen and os.path.isdir(path):
                self.dataset_combo.addItem(path, path)
        self.dataset_combo.setCurrentIndex(-1)

    def _remember_recent(self, path):
        recents = self.settings.value("recents", [], type=list)
        path = os.path.abspath(path)
        if path in recents:
            recents.remove(path)
        recents.insert(0, path)
        self.settings.setValue("recents", recents[:10])

    def open_dataset(self, path):
        try:
            raw, cfg_path, data_dir = load_raw_config(path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error",
                                           f"Could not load config:\n{e}")
            return
        self.data_dir = data_dir
        self.cfg_path = cfg_path
        self.editor.load(raw)
        exists = os.path.exists(cfg_path)
        self.cfg_label.setText(
            f"{cfg_path}{'' if exists else '  (NEW, not saved yet)'}")
        self._update_files_label()
        self._remember_recent(data_dir)
        name = raw.get("name") or os.path.basename(os.path.normpath(data_dir))
        self.statusBar().showMessage(f"loaded {name}")
        self.setWindowTitle(f"inspostgui — {name}")
        idx = self.dataset_combo.findData(data_dir)
        if idx < 0:
            self.dataset_combo.addItem(data_dir, data_dir)
            idx = self.dataset_combo.count() - 1
        self.dataset_combo.setCurrentIndex(idx)

    def _update_files_label(self):
        if not self.data_dir:
            self.files_label.setText("")
            return
        parts = []
        for fname in ("imu.csv", "ref.csv", "gnss.csv", "mag.csv",
                      "baro.csv"):
            p = os.path.join(self.data_dir, fname)
            if os.path.exists(p):
                parts.append(f"{fname} ({os.path.getsize(p) // 1024} kB)")
            else:
                parts.append(f"{fname} —")
        self.files_label.setText("dataset files:  " + "   ".join(parts))

    def _on_dataset_selected(self, idx):
        path = self.dataset_combo.itemData(idx)
        if path:
            self.open_dataset(path)

    def _on_browse(self):
        start = self.data_dir or os.path.join(REPO_ROOT, "datasets")
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select dataset directory (with config.yaml + the CSVs)", start)
        if path:
            self.open_dataset(path)

    # --- config buttons ------------------------------------------------
    def _on_cfg_reload(self):
        if self.data_dir:
            self.open_dataset(self.data_dir)

    def _on_cfg_new(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Dataset directory for the new config", REPO_ROOT)
        if not path:
            return
        if os.path.exists(os.path.join(path, "config.yaml")):
            QtWidgets.QMessageBox.information(
                self, "Exists", "This directory already has a config.yaml "
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
        self.editor.load(raw)  # new baseline for "already present" keys
        self.cfg_label.setText(path)
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
        if errors:
            QtWidgets.QMessageBox.warning(self, "Cannot run",
                                          "\n".join(errors))
            return

        self.results = None
        self._drained = 0
        self._alt_t, self._alt_est, self._alt_ref = [], [], []
        self._alt_has_finite = False
        self._trail_gap = False
        self._smooth_quat = None
        self.pos_view.reset_trail()
        self.plots_widget.clear()
        self.summary_text.setPlainText("")
        self.pdf_btn.setEnabled(False)
        self.kml_btn.setEnabled(False)
        self.progress.setValue(0)

        self.worker = ReplayWorker(spec, self.data_dir,
                                   realtime=self.chk_realtime.isChecked(),
                                   speed=self.speed_spin.value())
        self.worker.sig_status.connect(self.statusBar().showMessage)
        self.worker.sig_progress.connect(self.progress.setValue)
        self.worker.sig_finished.connect(self._on_finished)
        self.worker.sig_error.connect(self._on_worker_error)
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.pause_btn.setEnabled(True)
        self.pause_btn.setChecked(False)
        self.worker.start()

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
                       results.get("warmup_end_sec", 0.0))
        self.map_view.set_tracks(
            [(row[1], row[2]) for row in results["kml_est"]],
            [(row[0], row[1]) for row in results["kml_ref"]])
        self.pdf_btn.setEnabled(True)
        self.kml_btn.setEnabled(bool(results["kml_est"]))
        rms = results["pos_rms_m"]
        msg = (f"done: {results['name']}, mode {results['mode']}"
               + (f", pos rms {rms:.2f} m ({results['scored_epochs']} "
                  f"scored epochs)" if results["scored_epochs"] else "")
               + (" [STOPPED]" if results["aborted"] else ""))
        self.statusBar().showMessage(msg)
        self.lbl_status.setText(msg)
        self.tabs.setCurrentIndex(0)

    def _on_worker_error(self, text):
        self._run_ended()
        self.statusBar().showMessage("replay failed")
        QtWidgets.QMessageBox.critical(self, "Replay failed", text)

    def _run_ended(self):
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.pause_btn.setEnabled(False)
        self.progress.setValue(100)

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
            write_kml(path, self.results["kml_est"], self.results["kml_ref"],
                      self.results["kml_fix"], self.results["name"])
            self.statusBar().showMessage(f"wrote {path}")
        except ImportError:
            QtWidgets.QMessageBox.warning(
                self, "Missing dependency",
                "KML export needs simplekml: pip install simplekml")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "KML export failed", str(e))

    def _on_estimate_bias_rw(self):
        # Allan variance needs a LONG STATIC recording, which is usually a
        # separate log from the (moving) dataset -- let the user pick the
        # imu.csv, defaulting to the current dataset's directory.
        start = self.data_dir or os.path.join(REPO_ROOT, "datasets")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Pick a long STATIC imu.csv, ideally a few hours "
                 "(Allan / bias random walk)",
            start, "IMU CSV (imu*.csv *.csv);;All files (*)")
        if not path:
            return
        try:
            QtWidgets.QApplication.setOverrideCursor(
                QtCore.Qt.CursorShape.WaitCursor)
            import allan_variance
            result = allan_variance.analyze(path)
            cfg_path = getattr(self, "cfg_path", None)
            cfg_imu = (allan_variance._load_config_imu(cfg_path)
                       if cfg_path and os.path.exists(cfg_path) else {})
            report = allan_variance.format_report(result, cfg_imu)
        except SystemExit as e:  # analyze() rejects bad/short input via sys.exit
            QtWidgets.QApplication.restoreOverrideCursor()
            QtWidgets.QMessageBox.warning(self, "Bias RW estimation", str(e))
            return
        except Exception as e:
            QtWidgets.QApplication.restoreOverrideCursor()
            QtWidgets.QMessageBox.critical(self, "Bias RW estimation failed", str(e))
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Allan variance — bias random walk")
        box.setIcon(QtWidgets.QMessageBox.Icon.Information)
        box.setText("Estimated IMU noise model (paste bias_rw into "
                    "config.yaml `imu:`):")
        box.setDetailedText(report)
        # Monospace so the aligned columns line up in the detail view.
        box.setStyleSheet("QTextEdit { font-family: monospace; min-width: 620px; }")
        box.exec()

    # --- live UI ----------------------------------------------------------
    def _on_fly_toggled(self, checked):
        if checked:
            self.chk_follow.setChecked(False)
        self.chk_follow.setEnabled(not checked)
        self.pos_view.set_fly_enabled(checked)

    def _update_ui(self):
        w = self.worker
        if w is None:
            return
        with w.lock:
            n = len(w.rec["t"])
            new_t = w.rec["t"][self._drained:n]
            new_pos = w.rec["pos"][self._drained:n]
            new_vel = w.rec["vel"][self._drained:n]
            new_ref = w.rec["ref_pos"][self._drained:n]
            live = dict(w.live)
        self._drained = n

        est_pts, est_speeds, est_breaks, ref_pts = [], [], [], []
        for ts, p, v, r in zip(new_t, new_pos, new_vel, new_ref):
            if p[0] == p[0]:  # not NaN
                est_pts.append(p)
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
            self._smooth_quat = (q if self._smooth_quat is None else
                                 self._slerp(self._smooth_quat, q, 0.35))
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
        gyr_bias_sigma = live.get("gyr_bias_sigma")
        if gyr_bias is not None:
            gs = gyr_bias_sigma or (math.nan, math.nan, math.nan)
            self.lbl_gyr_bias.setText(
                f"Gyro [deg/s]:\n"
                f"  ({math.degrees(gyr_bias[0]):+6.3f}, "
                f"{math.degrees(gyr_bias[1]):+6.3f}, "
                f"{math.degrees(gyr_bias[2]):+6.3f})\n"
                f"± ({math.degrees(gs[0]):.3f}, {math.degrees(gs[1]):.3f}, "
                f"{math.degrees(gs[2]):.3f})")
        else:
            self.lbl_gyr_bias.setText("Gyro: n/a")
        acc_bias = live.get("acc_bias")
        acc_bias_sigma = live.get("acc_bias_sigma")
        if acc_bias is not None:
            accs = acc_bias_sigma or (math.nan, math.nan, math.nan)
            self.lbl_acc_bias.setText(
                f"Accel [m/s^2]:\n"
                f"  ({acc_bias[0]:+6.3f}, {acc_bias[1]:+6.3f}, "
                f"{acc_bias[2]:+6.3f})\n"
                f"± ({accs[0]:.3f}, {accs[1]:.3f}, {accs[2]:.3f})")
        else:
            self.lbl_acc_bias.setText("Accel: n/a")
        pos, vel, llh = live.get("pos"), live.get("vel"), live.get("llh")
        pos_sigma, vel_sigma = live.get("pos_sigma"), live.get("vel_sigma")
        if pos is not None:
            ps = pos_sigma or (math.nan, math.nan, math.nan)
            self.lbl_pos.setText(f"NED: ({pos[0]:+9.1f}, {pos[1]:+9.1f}, "
                                 f"{pos[2]:+7.1f}) m\n"
                                 f"± ({ps[0]:.2f}, {ps[1]:.2f}, {ps[2]:.2f}) m")
        if llh is not None:
            self.lbl_llh.setText(f"Lat {llh[0]:11.7f}°  Lon {llh[1]:11.7f}°"
                                 f"  Alt {llh[2]:7.1f} m")
        baro_h = live.get("baro_h_d")
        baro_h_sigma = live.get("baro_h_sigma")
        gnss_off = live.get("local_gnss_offset")
        gnss_off_sigma = live.get("local_gnss_offset_sigma")
        if baro_h is not None:
            baro_txt = f"{baro_h:7.1f} ± {baro_h_sigma:.2f} m"
        else:
            baro_txt = "N/A"
        if gnss_off is not None:
            offset_txt = f"{gnss_off:+6.2f} ± {gnss_off_sigma:.2f} m"
        else:
            offset_txt = "N/A"
        self.lbl_baro.setText(f"Baro alt: {baro_txt}\n"
                              f"GNSS offset: {offset_txt}")
        if vel is not None:
            speed = math.sqrt(sum(x * x for x in vel))
            vs = vel_sigma or (math.nan, math.nan, math.nan)
            self.lbl_vel.setText(f"Vel NED: ({vel[0]:+6.2f}, {vel[1]:+6.2f},"
                                 f" {vel[2]:+6.2f}) m/s "
                                 f"± ({vs[0]:.2f}, {vs[1]:.2f}, {vs[2]:.2f})\n"
                                 f"|v| = {speed:5.2f} m/s  "
                                 f"({speed * 3.6:5.1f} km/h)")
        acc = live.get("acc")
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
        if self.worker is not None and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(3000)
        self.map_view.shutdown()
        super().closeEvent(event)


# ============================================================================
# Entry point
# ============================================================================

def run_batch(dataset):
    """Headless replay + summary printout (worker smoke test)."""
    app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
    raw, cfg_path, data_dir = load_raw_config(dataset)
    if not os.path.exists(cfg_path):
        sys.exit(f"{cfg_path} missing")
    spec = merge_spec(raw)
    errors = validate_spec(spec, data_dir)
    if errors:
        sys.exit("cannot run:\n  " + "\n  ".join(errors))
    worker = ReplayWorker(spec, data_dir)
    done = {}
    worker.sig_status.connect(print)
    worker.sig_error.connect(lambda txt: done.update(error=txt))
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
                    help="dataset directory (with config.yaml) or a "
                         "config YAML path to open at startup")
    ap.add_argument("--batch", action="store_true",
                    help="headless replay + summary printout, no GUI")
    args = ap.parse_args()

    if args.batch:
        if not args.dataset:
            sys.exit("--batch needs a dataset argument")
        sys.exit(run_batch(args.dataset))

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
