#!/usr/bin/env python3
"""Python replay of the datasets datasets through INS, with optional live
telemetry (PlotJuggler JSON via --plotjuggler and/or MAVLink via --mavlink,
both off by default).

This is the visualization/telemetry counterpart to the C harness
(tools/replay.c — that one is the regression evaluation tool for
`make datasets`). Both consume the same dataset directory: a generated
config.yaml plus the dataset-neutral CSVs (imu/ref/gnss/mag/baro.csv,
see datasets/replay_format.py). replay.py drives the same INS
filter via the ctypes wrapper.

Usage:
    make pylib
    python3 python/replay.py datasets/some/dataset  # dir
    python3 python/replay.py datasets/some/dataset/config.yaml
    python3 python/replay.py datasets/some/dataset --realtime
    python3 python/replay.py datasets/some/dataset --plotjuggler
    python3 python/replay.py datasets/some/dataset --mavlink
    python3 python/replay.py datasets/some/dataset --plot
    python3 python/replay.py datasets/some/dataset \\
        --plot --plot-out /tmp/plots.pdf  # save a multi-page PDF instead
    python3 python/replay.py datasets/some/dataset \\
        --kml /tmp/flight.kml           # Google Earth output
    python3 python/replay.py datasets/some/dataset \\
        --map-frames /tmp/frames        # OpenStreetMap PNG frame sequence

--plot draws the filter's state history (position/velocity/attitude) with
a 1-σ band from its own error-state covariance, the ground truth overlaid on
top, and (reference from ref.csv) an error-vs-truth plot with
the same band as a consistency check. Also included: a North-East position (map
view) page drawn from a separate, faster ground-track recorder
(--plot-track-hz) and thinned to the scale of the view rather than to a fixed
sample count, a dedicated altitude-profile page
(INSLIB/baro_alt/ground-truth/raw GNSS together), the INSLIB/ARS/AHRS bias
pages (mag hard-iron bias too, 18-state mode), an outlier-rejection page, the
cumulative chi2-downweight counters per sub-filter over time, and a sensor
sampling-rate page, each input stream's Hz bucketed over time (bucket width
--plot-rate-bucket-sec).

--kml writes a Google Earth KMZ (see ins_kml.py): the estimate as a
time-animated gx:Track (drag Google Earth's time slider to fly the replay)
with a small 3D model banking/pitching along the real roll/pitch/yaw, plus
static ground-track lines for the estimate (draped down to the ground so
climbs/descents are visible), the ground truth and the raw GNSS fix.

--map-frames writes a portrait PNG frame sequence (see ins_map_frames.py):
raw GNSS fix and INSLIB estimate revealed over time on an OpenStreetMap
background, with a band marking the real tunnel/underpass geometry pulled
from the Overpass API -- meant for a stretch of a drive a Google Earth
flyover cannot show (no hollow tunnel interior there). Prints the ffmpeg
command to stitch the frames into a video; does not run ffmpeg itself.

The GNSS-delay estimate cross-correlates baro_alt's own
down-velocity (assumed ~zero latency) against the held GNSS down-velocity
to estimate the fix's processing/telemetry latency, printed at the end -
a way to find a value for config `gnss: delay_ms` instead of
guessing it. Runs automatically whenever it would be meaningful (aiding:
gnss with usable velocity, plus a barometer), --estimate-gnss-delay forces
it on for other cases (e.g. a sanity check against aiding: ref, which
should come out near 0 ms since the fix is synthesized from the same
reference used for scoring). Needs a trial with real vertical motion
(climbs/descents) to give a meaningful (not just noise-fit) correlation
peak.

Requires the converted dataset (datasets/), PyYAML,
pymavlink for --mavlink, matplotlib for --plot, simplekml for --kml
(plus optional pyproj for its real-altitude geoid correction), and
matplotlib/Pillow for --map-frames (plus network access for its
OpenStreetMap tiles and tunnel geometry -- degrades gracefully without).
--estimate-gnss-delay needs no extra dependency.

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from INSLIB import Navigator, Config, Telemetry, ecef_to_llh   # noqa: E402
# The sub-filter breakdown, the status bits and the ISA conversion live in
# the shared telemetry module, so a live receiver and this replay put the
# identical tree on the wire (see INSLIB/telemetry.py).
from INSLIB.telemetry import (isa_pressure_to_altitude,        # noqa: E402
                              subfilter_overlay_tree, suite_status)
from geodetic_toolbox import mag_heading   # noqa: E402

US_PER_SEC = 1_000_000

_WGS84_A = 6378137.0
_WGS84_E2 = 0.00669437999014

# Harness defaults, mirroring tools/replay.c load_config().
DEFAULTS = {
    "name": "",
    "aiding": "gnss",
    "init": "auto",
    "automotive_mode": 0,
    "automotive_min_speed_mps": 0.0,
    "automotive_min_yaw_stddev_deg": 0.0,
    # Non-holonomic lateral velocity constraint (REQ-NAV-077). A filter
    # option, so this harness applies it rather than only accepting it:
    # one config.yaml has to mean the same thing to both.
    "automotive_lateral_constraint": 0,
    "automotive_lateral_stddev_mps": 0.0,
    "automotive_lateral_max_yaw_rate_deg": 0.0,
    "automotive_lateral_after_sec": 0.0,
    "chi2_disable": 0,  # REQ-SYS-015/REQ-VER-011: disable chi2 outlier
                        # downweighting library-wide (diagnostics only)
    "chi2_reject_alpha": 0.0,  # REQ-NAV-046: global chi2 gate significance;
                        # 0 -> per-channel historical gates, positive ->
                        # shared chi2inv(1-alpha, 1) for all channels
    "allow_unlimited_deadreckoning": 0,  # 1: never expire the coasting
                        # window - stay is_ready() through arbitrarily long
                        # IMU-only outages (position drifts unbounded)
    "max_deadreckoning_sec": 0.0,  # IMU-only coasting budget before
                        # is_ready() degrades; 0 -> C default.
                        # Ignored when allow_unlimited_deadreckoning is set
    "baro_height_disable": 0,  # REQ-NAV-053: 1 -> never select barometric
                        # height at bootstrap, even with a barometer present.
                        # Set this for a GNSS-labelled source that is not
                        # really satellite GNSS and already reports
                        # better-than-barometric vertical accuracy (e.g.
                        # crazyflie's Lighthouse position)

    # Static window at the start of the trial to seed the initial gyro bias:
    # a harness-only heuristic, no ins.h equivalent to fall back to. 0 -> no
    # seed (estimate_gyro_bias() needs >= 10 samples inside the window and 0
    # never collects that many). Schema shared with tools/replay.c -- a
    # dataset that relies on the historical 3.0 s window states it in its
    # own config.yaml instead of this file assuming it for everyone.
    "gyro_bias_window_sec": 0.0,
    "auto_init_window_sec": 0.0,  # IMU leveling window for ins's auto-init
                        # bootstrap [s] (0 -> ins's built-in default); raise
                        # for a low IMU rate so the window still contains
                        # enough samples (schema shared with
                        # tools/replay.c, REQ-VER-017)
    # The optional calibration keys below (REQ-NAV-037) default to no-op: an
    # all-zero misalignment means identity, a zero fixed bias removes nothing.
    "imu": {
        # Required from the dataset yaml -- listed with a 0 default only so
        # they are part of the known-key schema (REQ-VER-025); a config that
        # leaves them at 0 is rejected by the noise-model check below.
        "gyr_psd": 0.0,
        "acc_psd": 0.0,
        "gyr_bias_rw": 0.0,
        "acc_bias_rw": 0.0,
        "acc_misalignment": (0.0,) * 9,  # col-major 3x3 (all-0 -> identity)
        "gyr_misalignment": (0.0,) * 9,
        "acc_fixed_bias": (0.0, 0.0, 0.0),  # [m/s^2], removed permanently
        "gyr_fixed_bias": (0.0, 0.0, 0.0),  # [rad/s]
        # Extra process-noise margin on top of the physically-derived acc/
        # gyro noise (ins.h's Qxx_noise_diag fields) - 0 -> ins.c's own
        # INS_DEFAULT_POS/VEL/RPY_PRED_STDDEV_* (REQ-NAV-049 beginner-
        # friendly default). Left at 0 here on purpose so build_config()
        # doesn't duplicate that default as a second, driftable copy of the
        # same number - override per-dataset in config.yaml instead of
        # editing this file.
        "pos_pred_stddev_m_sqrts": 0.0,
        "vel_pred_stddev_mps_sqrts": 0.0,
        "rpy_pred_stddev_rad_sqrts": 0.0,
        # Auto-ZUPT/ZARU pseudo-measurement stddev (ins.h's zero_vel_stddev_mps
        # / zero_rot_stddev_rps): how hard the filter trusts its own
        # "stationary" detection once ins's velocity-aware auto-ZUPT/ZARU
        # flags a stop - tighter -> faster bias/attitude convergence while
        # parked but more sensitive to a false-positive stop; looser -> the
        # opposite. 0 -> this harness's own default (0.05 m/s, 0.1 deg/s),
        # NOT ins.c's own built-in default (0.05 m/s, 0.5 deg/s for the
        # rotation channel) - these two tools have deliberately tuned the
        # rotation stddev tighter than the library default for years, so 0
        # preserves that instead of silently loosening it.
        "zero_vel_stddev_mps": 0.0,
        "zero_rot_stddev_deg": 0.0,
        # Stillness detection: ONE set for the whole suite (REQ-SUITE-020).
        # Forwarded to ins's auto_zupt_*, from where nav_suite_init()
        # propagates it to the ARS/AHRS fallback (REQ-AHRS-017) and, through
        # that, to baro_alt's vertical ZUPT (REQ-SUITE-015). Each 0 -> the
        # library's own default. Schema shared with tools/replay.c.
        #
        # The window stddevs are the PRIMARY criterion (per-axis RMS of the
        # raw IMU over a short window); the magnitude bounds are only loose
        # sanity limits next to them. max_vel is applied only while a recent
        # GNSS velocity exists that is itself precise enough
        # (max_vel_stddev), never against the filter's own velocity state -
        # gating on that would be circular.
        "auto_zupt_static_gyr_deg": 0.0,
        "auto_zupt_static_acc_mps2": 0.0,
        "auto_zupt_static_gyr_stddev_deg": 0.0,
        "auto_zupt_static_acc_stddev_mps2": 0.0,
        "auto_zupt_max_vel_mps": 0.0,
        "auto_zupt_max_vel_stddev_mps": 0.0,
        "auto_zupt_dwell_sec": 0.0,
        "auto_zupt_min_interval_sec": 0.0,
        "auto_zupt_disable": 0,
        # Narrower opt-out: only the ARS/AHRS's own velocity-blind detector,
        # ins keeps deciding for everyone. Set for a dataset the IMU alone
        # cannot tell from a standstill - a noiseless synthetic constant-rate
        # climb or constant-velocity leg has zero sample variance and a
        # gravity-only specific force.
        "auto_zupt_velocity_blind_disable": 0,
    },
    "gnss": {
        # 0 -> the fixes are still read, counted and available for the
        # score, but none of them is fed to the filter. One key instead of
        # gates set to reject everything, for a run that is deliberately
        # GNSS-free (an inertial-only test, an A/B against the same
        # recording with GNSS on). tools/insrcv.c has the same key.
        "enable": 1,
        "leverarm_frd": (0.0, 0.0, 0.0),
        # Replaces a zero (unknown) diagonal entry in a real gnss.csv fix's
        # reported covariance, AND (aiding: ref only) is the noise assigned
        # to the whole reference-synthesized fix, which never has a reported
        # covariance to begin with. REQUIRED (no built-in default) under
        # aiding: ref - see the validation in load_config() below.
        "pos_stddev_fallback_m": (0.0, 0.0),
        "vel_stddev_fallback_mps": 0.0,
        # GNSS fusion thresholds (REQ-NAV-043): a fix with a larger reported
        # stddev is not fused at all. 0 -> ins.c's own INS_DEFAULT_GNSS_MAX_*
        # (schema shared with tools/replay.c). Left at 0 here on purpose,
        # like pos_pred_stddev_m_sqrts above - a dataset whose GNSS is coarser
        # than that default (e.g. an unlogged-accuracy receiver falling back
        # to pos_stddev_fallback_m) must open the gate explicitly in its own
        # config.yaml instead of this file silently loosening it for everyone.
        "max_horizontal_pos_stddev_m": 0.0,
        "max_vertical_pos_stddev_m": 0.0,
        "max_horizontal_vel_stddev_mps": 0.0,
        "max_vertical_vel_stddev_mps": 0.0,
        "delay_ms": 0.0,  # REQ-VER-008: assumed fixed processing/telemetry
                          # latency, applied uniformly to every fix
        # REQ-NAV-038 covariance conditioning (all 0 -> no-op): scale the
        # reported covariance and floor the per-axis stddev.
        "pos_cov_scale": 0.0,             # multiplies GNSS pos stddev (0 -> 1)
        "pos_cov_scale_height": 0.0,      # extra downweight of pos HEIGHT axis (0 -> 1)
        "vel_cov_scale": 0.0,             # multiplies GNSS vel stddev (0 -> 1)
        "pos_stddev_floor_hor_m": 0.0,    # min horiz. pos stddev [m]
        "pos_stddev_floor_ver_m": 0.0,    # min vert.  pos stddev [m]
        "vel_stddev_floor_hor_mps": 0.0,  # min horiz. vel stddev [m/s]
        "vel_stddev_floor_ver_mps": 0.0,  # min vert.  vel stddev [m/s]
        # REQ-NAV-051/052 solution-mode gates (all 0 -> ins's own defaults),
        # separate from the max_* fusion gates above: start_* is the strict
        # set that admits the 3D solution, stop_* the loose set that gives it
        # up after stop_dwell_sec of nothing better.
        "start_max_horizontal_pos_stddev_m": 0.0,    # [m]
        "start_max_vertical_pos_stddev_m": 0.0,      # [m]
        "start_max_horizontal_vel_stddev_mps": 0.0,  # [m/s]
        "start_max_vertical_vel_stddev_mps": 0.0,    # [m/s]
        "stop_max_horizontal_pos_stddev_m": 0.0,     # [m]
        "stop_max_vertical_pos_stddev_m": 0.0,       # [m]
        "stop_max_horizontal_vel_stddev_mps": 0.0,   # [m/s]
        "stop_max_vertical_vel_stddev_mps": 0.0,     # [m/s]
        "init_dwell_sec": 0.0,      # entry dwell [s] (REQ-NAV-045)
        "init_dwell_disable": 0,    # 1 -> enter on the first good fix
        "stop_dwell_sec": 0.0,      # exit dwell [s]
        "stop_disable": 0,          # 1 -> never leave 3D on GNSS quality
        # REQ-NAV-063: fuse the position on every Nth epoch that offers a
        # usable position AND velocity, the velocity alone on the other
        # N-1. A receiver's two blocks come out of one coupled solution
        # whose cross-covariance it does not report, so fusing both counts
        # the same information twice. <= 1 -> off, 0 -> ins's own default.
        "pos_decimation": 0,
        "min_delay_ms": 0,
        # REQ-NAV-071 covariance caps (0 -> ins's own default, <0 -> off
        # for that axis group), applied after the floors above.
        "pos_stddev_cap_hor_m": 0.0,
        "pos_stddev_cap_ver_m": 0.0,
        "vel_stddev_cap_hor_mps": 0.0,
        "vel_stddev_cap_ver_mps": 0.0,
        # REQ-NAV-072: asymmetric tracking of the reported GNSS accuracy,
        # decay time constant [s] (0 -> default, <0 -> off).
        "acc_envelope_tau_sec": 0.0,
        # REQ-NAV-073/075/076: manoeuvre-dependent GNSS velocity noise (0 ->
        # default, <0 -> off for that axis group) and its averaging
        # window [s] (0 -> default, <0 -> off). The acceleration read is the
        # antenna's, so a lever arm makes rotation count too.
        "vel_noise_acc_scale_hor": 0.0,
        "vel_noise_acc_scale_ver": 0.0,
        "vel_noise_acc_window_sec": 0.0,
    },
    # REQ-VER-010: initial-state uncertainty, each 0 -> the harness's
    # built-in default. This YAML schema is shared with the C replay
    # harness (tools/replay.c) and kept stable across both - see
    # build_config() below for the translation onto ins.Config's own
    # (harmonized) field names. Applies to BOTH init modes (init: ref AND
    # init: auto) - ins's shared finalize step uses these for the
    # initial covariance regardless of which one supplied the state itself.
    "init_stddev": {"pos_init_stddev_m": 0.0,
                    "vel_init_stddev_mps": 0.0,
                    "rpy_init_stddev_rad_deg": 0.0,
                    "yaw_init_stddev_rad_deg": 0.0,
                    "acc_bias_init_stddev_mps2": 0.0,
                    "gyr_bias_init_stddev_rps_deg": 0.0},
    # A known initial roll/pitch and/or yaw for ins's auto-init bootstrap
    # (init: auto only - init: ref already gets its state from the truth
    # epoch and never runs auto-init), e.g. a known heading with no
    # magnetometer/GNSS-course yaw aiding to derive it from otherwise.
    # roll/pitch are only used together (rpy_stddev_deg 0 -> no roll/pitch
    # hint); yaw is independent (yaw_stddev_deg 0 -> no yaw hint, the
    # typical case: leveling is already observable from gravity, only yaw
    # needs the operator's knowledge). See Navigator.set_init_att_hint().
    "init_hint": {"roll_deg": 0.0, "pitch_deg": 0.0, "rpy_stddev_deg": 0.0,
                  "yaw_deg": 0.0, "yaw_stddev_deg": 0.0},
    # The operator's declared origin, offered to the filter until it
    # bootstraps from it. Same key, same meaning and the same forced
    # options as tools/insrcv.c and tools/replay.c: one config.yaml has to
    # describe one experiment whether it is replayed or driven live.
    "free_inertial_start": {"enable": 0, "lat_deg": None, "lon_deg": None,
                            "height_m": 0.0, "stddev_m": 2.0},
    "mag": {"enable": 0, "stddev_ut": 0.0, "wmm_year": 0.0,
            # Fusion rate limit: 0 -> ins default, negative -> fuse every
            # sample. Same key as tools/insrcv.c and tools/replay.c.
            "min_delay_ms": 0,
            "estimate_bias": 0,  # 18-state hard-iron bias
            # REQ-NAV-039 fixed calibration (all-0 -> no-op):
            "misalignment": (0.0,) * 9,  # col-major 3x3 (all-0 -> identity)
            "fixed_bias": (0.0, 0.0, 0.0)},  # hard-iron [uT]
    "baro": {"enable": 0, "stddev_m": 0.0, "acc_bias_rw": 0.0,
             # baro_alt's own direct accel-noise density [m/s^2/sqrt(Hz)];
             # 0 -> baro_alt's own default (Navigator.set_baro_acc_noise()).
             "acc_noise_mps2_sqrthz": 0.0,
             # baro_alt's own INITIAL accel-bias uncertainty [m/s^2]; 0 ->
             # baro_alt default (Navigator.set_baro_acc_bias_init_stddev()).
             "acc_bias_init_mps2": 0.0,
             # baro_alt's own small direct height process-noise density
             # [m/sqrt(Hz)]; 0 -> baro_alt default
             # (Navigator.set_baro_h_process_noise()), schema shared with
             # tools/replay.c.
             "h_process_noise": 0.0,
             # local-height/GNSS-ellipsoid offset filter tuning (all 0 ->
             # local_gnss_alt's own defaults). Raise local_gnss_rw_stddev_mps
             # further for missions with extreme altitude excursions (e.g. a
             # soaring glider): the offset absorbs the ISA-model error, which
             # grows with the excursion and can outrun even the default random
             # walk. Schema shared with tools/replay.c.
             "local_gnss_rw_stddev_mps": 0.0,
             "local_gnss_chi2_threshold": 0.0,
             "local_gnss_min_update_interval_sec": 0.0,
             "local_gnss_stddev_inflation_factor": 0.0},
    # ARS/AHRS noise model (see nav_suite.h: ars_cfg/ahrs_cfg are only
    # ever wired to THESE values (never to imu) so this is the
    # only way to move them off the C library's own fixed default, one
    # value each, since both sub-filters consume the same physical gyro/
    # accelerometer). 0 -> ahrs's own default. gyr_noise_psd is named after
    # ahrs.h's own (slightly misleading) field: despite "psd" in the name
    # it's a density [rad/s/sqrt(Hz)], not a raw PSD (see ahrs.h, it gets
    # squared internally, same convention as gyr_bias_rw). acc_noise_mps2
    # is a plain stddev (not a density): ahrs.c fuses it as a discrete
    # leveling correction, same convention as ahrs.h's own field.
    # gyr_bias_init_stddev_rps_deg [deg/s]: initial gyro-bias uncertainty
    # for the ARS/AHRS, independent of gyro_bias_window_sec's parked-phase
    # seed - 0 -> Navigator.set_ahrs_gyr_bias_init_stddev() not called,
    # ahrs.c's own xy/z default stays in effect.
    "ahrs": {"gyr_noise_psd": 0.0, "gyr_bias_rw": 0.0, "acc_noise_mps2": 0.0,
             "gyr_bias_init_stddev_rps_deg": 0.0},
    # score: this harness scores on leverarm_frd/warmup_sec only. The pass/fail
    # limits below are consumed by tools/replay.c, which is the gating
    # harness (make datasets), and lim_groves_pos_rms_factor by
    # datasets/check_simulated.py; they are listed here so one dataset
    # config.yaml validates against both schemas (REQ-VER-025) instead of
    # forcing a dataset to choose a harness.
    "score": {
        "leverarm_frd": (0.0, 0.0, 0.0), "warmup_sec": 60.0,
        "ahrs": 0, "attitude": 1, "min_epochs": 0,
        "lim_att_bias_deg": 0.0, "lim_att_std_deg": 0.0,
        "lim_yaw_bias_deg": 0.0, "lim_yaw_std_deg": 0.0,
        "lim_pos_rms_m": 0.0,
        "lim_baro_rms_m": 0.0, "lim_baro_bias_m": 0.0, "lim_baro_max_m": 0.0,
        "lim_ellipsoid_rms_m": 0.0, "lim_ellipsoid_bias_m": 0.0,
        "lim_ellipsoid_max_m": 0.0,
        "lim_ars_att_bias_deg": 0.0, "lim_ars_roll_std_deg": 0.0,
        "lim_ars_pitch_std_deg": 0.0, "lim_ars_yaw_drift_deg_min": 0.0,
        # Coasting re-acquisition (REQ-VER-029). Scored by replay.c, which
        # owns the regression gates; this harness only has to accept the
        # keys so one config.yaml stays readable by both (REQ-VER-025).
        "coast_gap_min_sec": 0.0, "lim_coast_exit_err_m": 0.0,
        # Time of validity of a reference row (REQ-VER-030).
        "ref_delay_ms": 0.0,
        # check_simulated.py: "ins pos rms <= this x the Groves textbook
        # filter's own rms". Needs ref_groves_kf_sol.csv, which only the
        # simulated datasets carry, so this harness has nothing to do with it
        # and only has to accept it.
        "lim_groves_pos_rms_factor": 0.0,
    },
    # Optional per-stream CSV filename overrides, each relative to the
    # config's own directory. Empty -> the conventional <stream>.csv name,
    # so a dataset with no inputs: section behaves exactly as before. Lets
    # one stream be swapped (e.g. gnss: gnss_f9p.csv) while the other CSVs
    # stay shared - no directory duplication for A/B runs.
    # Absolute speed aiding (REQ-NAV-068), e.g. an OBD-II vehicle speed.
    # Every value is a plain constant applied to every sample: speed.csv
    # carries only the timestamp and the speed, so a dataset is described
    # here rather than half here and half in the CSV. Each 0 falls back to
    # the library's own default (REQ-NAV-043's convention), so a bare
    # "speed: {enable: 1}" is already a working configuration.
    "speed": {"enable": 0,
              "scale": 0.0,           # 0 -> 1.0, no correction
              "stddev_mps": 0.0,      # per-sample 1-sigma; 0 -> default
              "stddev_rel": 0.0,      # speed-proportional; 0 -> default (3%)
              "min_speed_mps": 0.0,   # 0 -> default
              "delay_ms": 0.0},       # how old a sample is at its timestamp
    "inputs": {"imu": "imu.csv", "ref": "ref.csv", "gnss": "gnss.csv",
               "mag": "mag.csv", "baro": "baro.csv", "speed": "speed.csv"},
}

# Sections of the config.yaml schema that belong to a DIFFERENT consumer of
# the same file. One dataset directory is read by more than one tool, and
# REQ-VER-025 makes the schema shared, not the interpretation: a section this
# harness has no use for is still a valid part of the file, so the unknown-key
# check below skips these wholesale instead of rejecting the dataset. The C
# parser carries the identical list (tools/replay.c, cfg_set tail).
#
# Rejecting them would force a dataset to pick one of its tools; deleting them
# from the file to satisfy this parser would silently disarm the other tool.
#
# A section only earns a place here once a tool actually reads it: something
# no tool reads is not a foreign section, it is documentation, and belongs in
# a YAML comment where no parser has to know about it at all (which is where
# the simulated datasets' injected-bias numbers now live).
#
#   origin      local-frame anchor (lat/lon/h) for a platform that has no
#               absolute position source, consumed by python/crazyflie_reader.py.
#   crazyflie   radio URI and other link settings for that same reader.
FOREIGN_SECTIONS = ("origin", "crazyflie")


def input_path(data_dir, spec, stream):
    """Absolute path of one input CSV, honouring the config's optional
    `inputs:` filename overrides (schema shared with tools/replay.c).
    Falls back to the conventional <stream>.csv when unset."""
    name = (spec.get("inputs") or {}).get(stream) or f"{stream}.csv"
    return os.path.join(data_dir, name)


def llh_to_ecef(lat, lon, h):
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + h) * math.cos(lat) * math.cos(lon)
    y = (n + h) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + h) * math.sin(lat)
    return x, y, z


def ned_to_ecef_rot(lat, lon):
    """R_n_to_e (NED -> ECEF), row-major 3x3 tuple."""
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return ((-sl * co, -so, -cl * co),
            (-sl * so,  co, -cl * so),
            ( cl,      0.0, -sl))


def ecef_to_ned_rot(lat, lon):
    """R_e_to_n, the transpose of ned_to_ecef_rot, row-major 3x3."""
    m = ned_to_ecef_rot(lat, lon)
    return ((m[0][0], m[1][0], m[2][0]),
            (m[0][1], m[1][1], m[2][1]),
            (m[0][2], m[1][2], m[2][2]))


def _matvec_cm(r9, v):
    """flat COLUMN-major 3x3 (r9) times v -> list (element (i,j)=r9[i+3j])."""
    return [r9[i] * v[0] + r9[i + 3] * v[1] + r9[i + 6] * v[2] for i in range(3)]


def _matvec_rm(m, v):
    """row-major 3x3 (nested) times v -> list."""
    return [m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3)]


def _matvec_rm_T(m, v):
    """row-major 3x3 transposed times v -> list (M^T @ v)."""
    return [m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2] for i in range(3)]


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


_WGS84_OMEGA = 7.2921151467e-5  # earth rotation rate [rad/s]


def _rotmat_from_rpy(rpy):
    """R_b_to_n from roll/pitch/yaw [rad] (ZYX), row-major nested tuple."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return ((cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
            (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
            (-sp, sr * cp, cr * cp))


def _rotvec_from_rotmat(R):
    """Rotation vector [rad] of a row-major 3x3 rotation (log map)."""
    s_vec = ((R[2][1] - R[1][2]) * 0.5, (R[0][2] - R[2][0]) * 0.5,
             (R[1][0] - R[0][1]) * 0.5)
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) * 0.5))
    theta = math.acos(c)
    sin_t = math.sin(theta)
    if sin_t < 1e-9:  # ~identity (theta ~ 0; 180 deg cannot occur here)
        return list(s_vec)
    k = theta / sin_t
    return [k * x for x in s_vec]


def nav_rotation_rate_body(lat_rad, h_m, vel_ned, rpy):
    """Rotation rate of the navigation frame w.r.t. inertial, expressed in
    the BODY frame [rad/s]: earth rotation plus transport rate, rotated by
    the given attitude (roll/pitch/yaw rad, ZYX). This is what an
    error-free gyro triad measures ON TOP of its bias while the platform
    holds attitude. Mirrors src/geodetic_toolbox.c ins_calc_omega_n_in()."""
    sl, cl = math.sin(lat_rad), math.cos(lat_rad)
    denom = 1.0 - _WGS84_E2 * sl * sl
    rn = _WGS84_A * (1.0 - _WGS84_E2) / (denom * math.sqrt(denom))
    re = _WGS84_A / math.sqrt(denom)
    tl = sl / cl if abs(cl) > 1e-6 else 0.0  # pole guard, like the C side
    w_n = (_WGS84_OMEGA * cl + vel_ned[1] / (re + h_m),
           -vel_ned[0] / (rn + h_m),
           -_WGS84_OMEGA * sl - vel_ned[1] * tl / (re + h_m))
    return _matvec_rm_T(_rotmat_from_rpy(rpy), w_n)  # R' * w_n -> body


def correct_initial_gyr_bias(gyr_bias, ref, t_first_imu_us, window_sec):
    """Remove the modeled non-bias content from the initial-window gyro
    average (init: ref only - needs the reference): the navigation-frame
    rotation (earth rate, ~15 deg/hr at mid latitudes, plus transport
    rate, comparable at aircraft speeds) and the platform's own mean
    rotation over the window, derived from the reference attitude (log
    map of the relative rotation / window length). What remains is the
    sensor bias - valid for a parked start, a start that begins moving
    inside the window (the Groves car pitches up 0.9 deg while
    accelerating: 0.31 deg/s of very real y rate in a 3 s window), and a
    moving start (aircraft in cruise). Leaving these in poisons the seed
    in a way a tight gyr_bias_init_stddev_rps can never recover from, and
    drives the ARS's unobservable z-bias directly."""
    t_b_us = t_first_imu_us + int(window_sec * US_PER_SEC)
    r_a = next((r for r in ref if r["t_us"] >= t_first_imu_us), ref[0])
    r_b = next((r for r in reversed(ref) if r["t_us"] <= t_b_us), ref[-1])
    w_in_b = nav_rotation_rate_body(
        r_a["lat_rad"], r_a["h_m"], r_a["vel_ned"],
        (r_a["roll_rad"], r_a["pitch_rad"], r_a["yaw_rad"]))
    out = [gyr_bias[i] - w_in_b[i] for i in range(3)]
    span = (r_b["t_us"] - r_a["t_us"]) / US_PER_SEC
    if span > 0.5:  # enough baseline for the attitude-delta rate
        R0 = _rotmat_from_rpy((r_a["roll_rad"], r_a["pitch_rad"],
                               r_a["yaw_rad"]))
        R1 = _rotmat_from_rpy((r_b["roll_rad"], r_b["pitch_rad"],
                               r_b["yaw_rad"]))
        # body-frame rotation over the window: R_rel = R0' * R1
        r_rel = tuple(tuple(sum(R0[k][i] * R1[k][j] for k in range(3))
                            for j in range(3)) for i in range(3))
        phi = _rotvec_from_rotmat(r_rel)
        for i in range(3):
            out[i] -= phi[i] / span
    return tuple(out)


def load_config(path):
    """Load the dataset config.yaml (generated by datasets/convert_*.py,
    schema shared with tools/replay.c). `path` may be the YAML file
    or a dataset directory containing config.yaml. Returns (spec,
    data_dir)."""
    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML is required: pip install pyyaml")
    if os.path.isdir(path):
        cfg_path = os.path.join(path, "config.yaml")
        data_dir = path
    else:
        cfg_path = path
        data_dir = os.path.dirname(path) or "."
    if not os.path.exists(cfg_path):
        sys.exit(f"{cfg_path} missing")
    with open(cfg_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    # An unknown key is an error, never a silently ignored one: a mistyped
    # or renamed tuning parameter is otherwise indistinguishable from a
    # parameter the library ignored, and the run happily reports numbers
    # produced with a configuration nobody asked for. Same rule as
    # tools/replay.c's parser (REQ-VER-025).
    unknown = [k for k in cfg if k not in DEFAULTS and k not in FOREIGN_SECTIONS]
    for key, dflt in DEFAULTS.items():
        if isinstance(dflt, dict) and isinstance(cfg.get(key), dict):
            unknown += [f"{key}.{k}" for k in cfg[key] if k not in dflt]
    if unknown:
        sys.exit(f"{cfg_path}: unknown config key(s): {', '.join(sorted(unknown))}")

    spec = {}
    for key, dflt in DEFAULTS.items():
        if isinstance(dflt, dict):
            merged = dict(dflt)
            merged.update(cfg.get(key) or {})
            spec[key] = merged
        else:
            spec[key] = cfg.get(key, dflt)
    if not spec["imu"].get("gyr_psd") or not spec["imu"].get("acc_psd"):
        sys.exit(f"{cfg_path}: missing imu noise model (imu: gyr_psd/acc_psd)")
    gd = spec["gnss"]
    fi = spec["free_inertial_start"]
    if fi["enable"] and (fi["lat_deg"] is None or fi["lon_deg"] is None):
        sys.exit(f"{cfg_path}: free_inertial_start needs lat_deg and lon_deg: "
                 "the whole point is the origin you supply.")

    if spec["aiding"] == "ref" and not (
            gd["pos_stddev_fallback_m"][0] > 0.0 and
            gd["pos_stddev_fallback_m"][1] > 0.0 and
            gd["vel_stddev_fallback_mps"] > 0.0):
        sys.exit(f"{cfg_path}: aiding: ref requires gnss: "
                 "pos_stddev_fallback_m/vel_stddev_fallback_mps (the "
                 "reference-synthesized fix never has a reported covariance "
                 "of its own, so this is the only noise it gets; no "
                 "built-in default)")
    return spec, data_dir


def pos_error_ecef(nav, ref_now, score_leverarm_frd):
    """Estimate-minus-reference position error in ECEF [m]. The scoring
    lever arm maps the filter position onto the ground-truth point
    (zero when the truth refers to the IMU center). Returns the
    3-vector, or None if the estimate is unavailable."""
    fecef = nav.position_ecef()
    rbn = nav.rotmat_b_to_n()
    if not (fecef and rbn):
        return None
    recef = llh_to_ecef(ref_now["lat_rad"], ref_now["lon_rad"], ref_now["h_m"])
    la_n = _matvec_cm(rbn, score_leverarm_frd)
    la_e = _matvec_rm(ned_to_ecef_rot(ref_now["lat_rad"], ref_now["lon_rad"]),
                      la_n)
    return [fecef[i] - recef[i] + la_e[i] for i in range(3)]


def ref_to_local_ned(ref_now, origin_ecef, origin_lat, origin_lon):
    """Ground-truth position in the estimate's LOCAL NED frame (same origin
    and convention as INSLIB/pos_ned), so ref and estimate can be overlaid
    directly.

    Uses the local-tangent "flat-Earth with local curvature radii" mapping
    (dN = dlat*(M+h), dE = dlon*(N+h)*cos, dD = -dh), which is ins's own
    pos_local convention - NOT a rigid ECEF->NED rotation at the origin.
    Over a long baseline the two disagree by hundreds of metres of purely
    COSMETIC overlay drift (the ellipsoid curving away from the origin's
    tangent plane, ~d^2/2R ~ 500 m in D at 80 km, plus meridian convergence
    in N/E), which made the overlay look wildly wrong even when the estimate
    tracked truth to ~cm. The true error is unaffected: it is computed in
    ECEF (see pos_error_ecef / rec['pos_err_ned'])."""
    _, _, origin_h = ecef_to_llh(*origin_ecef)
    slat = math.sin(origin_lat)
    denom = 1.0 - _WGS84_E2 * slat * slat
    r_meridian = _WGS84_A * (1.0 - _WGS84_E2) / (denom * math.sqrt(denom))
    r_transverse = _WGS84_A / math.sqrt(denom)
    dlat = ref_now["lat_rad"] - origin_lat
    dlon = ref_now["lon_rad"] - origin_lon
    n = dlat * (r_meridian + origin_h)
    e = dlon * (r_transverse + origin_h) * math.cos(origin_lat)
    d = -(ref_now["h_m"] - origin_h)
    return [n, e, d]


def ref_overlay_tree(ref_now, ref_local):
    """PlotJuggler overlay of the ground truth in the SAME frames as the
    estimate, so you can drop ref_* on top of INSLIB/* directly (overlay
    the error yourself in PlotJuggler if you want it):

      ref/pos_ned   -> overlay on INSLIB/pos_ned  (N/E/D in the local frame)
      ref/att_deg   -> overlay on INSLIB/att_deg  (roll/pitch/yaw)
      ref/vel_ned   -> overlay on INSLIB/vel_ned
      ref/global    -> overlay on INSLIB/global   (lat/lon/alt)
    """
    ref = {
        "att_deg": {"roll": math.degrees(ref_now["roll_rad"]),
                    "pitch": math.degrees(ref_now["pitch_rad"]),
                    "yaw": math.degrees(ref_now["yaw_rad"])},
        "global": {"lat_deg": math.degrees(ref_now["lat_rad"]),
                   "lon_deg": math.degrees(ref_now["lon_rad"]),
                   "alt_m": ref_now["h_m"]},
        "vel_ned": {"n": ref_now["vel_ned"][0], "e": ref_now["vel_ned"][1],
                    "d": ref_now["vel_ned"][2]},
    }
    if ref_local is not None:
        ref["pos_ned"] = {"n": ref_local[0], "e": ref_local[1],
                          "d": ref_local[2]}
    return {"ref": ref}


def meas_overlay_tree(acc_mps2, gyr_rps, fix, mag, baro,
                      origin_ecef, origin_lat, origin_lon):
    """PlotJuggler view of the raw input measurements (everything fed to
    nav.imu()/gnss_pos_llh()/mag()/baro() this epoch), alongside the estimate
    and ground truth published in the same publish() call:

      meas/imu    -> raw acc/gyro this epoch (vs. INSLIB/acc_n, INSLIB/rate_dps)
      meas/gnss   -> last fix, in the SAME local NED frame as INSLIB/pos_ned
      meas/mag    -> last magnetometer sample [uT]
      meas/baro   -> last barometer sample (pressure + derived ISA altitude,
                     vs. INSLIB/global/alt_m)

    fix/mag/baro are the "last seen" sample (held between updates, like
    ref_overlay_tree's ref_now) so the trace stays continuous in
    PlotJuggler even though these arrive slower than the IMU."""
    meas = {
        "imu": {"acc_mps2": {"x": acc_mps2[0], "y": acc_mps2[1], "z": acc_mps2[2]},
               "gyr_dps": {"x": math.degrees(gyr_rps[0]),
                           "y": math.degrees(gyr_rps[1]),
                           "z": math.degrees(gyr_rps[2])}},
    }
    if fix is not None and origin_ecef is not None:
        pos_local = ref_to_local_ned(fix, origin_ecef, origin_lat, origin_lon)
        gnss = {"pos_ned": {"n": pos_local[0], "e": pos_local[1], "d": pos_local[2]},
               "global": {"lat_deg": math.degrees(fix["lat_rad"]),
                          "lon_deg": math.degrees(fix["lon_rad"]),
                          "alt_m": fix["h_m"]}}
        if fix.get("vel_ok", True):
            gnss["vel_ned"] = {"n": fix["vel_ned"][0], "e": fix["vel_ned"][1],
                               "d": fix["vel_ned"][2]}
        meas["gnss"] = gnss
    if mag is not None:
        meas["mag"] = {"x": mag[1][0], "y": mag[1][1], "z": mag[1][2]}
    if baro is not None:
        meas["baro"] = {"pressure_pa": baro[1],
                        "alt_m": isa_pressure_to_altitude(baro[1])}
    return {"meas": meas}


def load_ref(path):
    """Return list of dicts: t_us, lat_rad, lon_rad, h_m, vel_ned, rpy_rad."""
    ref = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if len(p) < 10:
                continue
            ref.append({
                "t_us": int(p[0]),
                "lat_rad": math.radians(float(p[1])),
                "lon_rad": math.radians(float(p[2])),
                "h_m": float(p[3]),
                "roll_rad": math.radians(float(p[4])),
                "pitch_rad": math.radians(float(p[5])),
                "yaw_rad": math.radians(float(p[6])),
                "vel_ned": (float(p[7]), float(p[8]), float(p[9])),
            })
    return ref


def cov6_to_rows(c6):
    """(nn, ne, nd, ee, ed, dd) -> symmetric 3x3 nested rows."""
    nn, ne, nd, ee, ed, dd = c6
    return [[nn, ne, nd], [ne, ee, ed], [nd, ed, dd]]


def apply_fallback(cov, var_hor, var_ver):
    """Replace zero (unknown) diagonal entries by the fallback variance;
    returns None if the result still has a non-positive diagonal
    (measurement unusable)."""
    if cov[0][0] <= 0.0:
        cov[0][0] = var_hor
    if cov[1][1] <= 0.0:
        cov[1][1] = var_hor
    if cov[2][2] <= 0.0:
        cov[2][2] = var_ver
    if cov[0][0] <= 0.0 or cov[1][1] <= 0.0 or cov[2][2] <= 0.0:
        return None
    return cov


GROWTH_HORIZON_SEC = 10.0  # reference free-inertial coasting window

# ars_cfg/ahrs_cfg in nav_suite.c are only ever wired to config.yaml's ahrs:
# section (Navigator.set_ahrs_gyr_noise()/set_ahrs_gyr_bias_rw(), see
# main()) - never to imu: above, since ARS/AHRS are physically-separate
# filter instances from ins (nav_suite.h: "edit s->ars_cfg / s->ahrs_cfg /
# s->baro_cfg" is the documented tuning contract). Likewise baro_alt's
# direct accel-noise term follows baro: acc_noise_mps2_sqrthz
# (Navigator.set_baro_acc_noise()). Left at 0 -> each filter's own C-side
# default: this module does NOT hardcode a copy of those constants (same
# reasoning as process_noise_growth_rate() - a python-side copy of a C
# #define silently goes stale the day someone retunes the C default and
# forgets this file exists).
STANDARD_GRAVITY_MPS2 = 9.80665  # INS_GRAVITY_NOMINAL, geodetic_toolbox.h


def _integrated_white_noise_var(q, T, n):
    """Var(y_n(T)) of the n-fold time integral of zero-mean white noise with
    PSD q, starting from zero covariance at t=0 (standard closed form for a
    chain of n ideal integrators driven by white noise):
        Var(y_n(T)) = q * T^(2n-1) / ((2n-1) * ((n-1)!)^2)
    n=1 -> q*T (a plain random walk); n=2 -> q*T^3/3; n=3 -> q*T^5/20;
    n=4 -> q*T^7/252; etc."""
    return q * T ** (2 * n - 1) / ((2 * n - 1) * math.factorial(n - 1) ** 2)


def _rate_state_growth_var(q1, q2, T):
    """Var(T) of a state directly driven by white noise q1 (1 integral)
    PLUS a bias state (itself driven by white noise q2) feeding it via one
    more integral (2 integrals total for the q2 path) - the shape shared by
    ins/baro_alt velocity and ins/ahrs attitude:
        Var(T) = y1(q1) + y2(q2) = q1*T + q2*T^3/3"""
    return _integrated_white_noise_var(q1, T, 1) + _integrated_white_noise_var(q2, T, 2)


def _position_state_growth_var(q1, q2, T):
    """Var(T) of the further integral of a _rate_state_growth_var state
    (ins/baro_alt position/height): 2 integrals of q1, 3 of q2.
        Var(T) = y2(q1) + y3(q2) = q1*T^3/3 + q2*T^5/20"""
    return _integrated_white_noise_var(q1, T, 2) + _integrated_white_noise_var(q2, T, 3)


def _tilt_coupled_velocity_growth_var(q1_acc, q2_acc, q1_gyr, q2_gyr, T,
                                      g=STANDARD_GRAVITY_MPS2):
    """Var(T) of a HORIZONTAL velocity error under free coasting, including
    the attitude-tilt-into-velocity coupling that a plain
    _rate_state_growth_var(q1_acc, q2_acc, T) misses.

    ins_compute_Phi (ins.c) couples velocity to attitude via the specific-
    force cross product: Phi[VEL,RPY] += [f_n]_x*dt. Under level,
    non-accelerating flight f_n=[0,0,-g], so d(velN)/dt gets an extra
    -g*pitch_err term (d(velE)/dt: +g*roll_err) - gravity leaking through
    a tilt error into a spurious horizontal acceleration. This is the
    classical INS free-inertial error mechanism and is typically FAR bigger
    than the direct accelerometer-noise term for any real gyro (a mere 1
    deg tilt error already implies ~0.17 m/s^2 of phantom accel, vs. a
    typical acc_psd noise floor orders of magnitude smaller) - omitting it
    understates real-world velocity/position drift, which is exactly what
    process_noise_growth_rate() did before this term was added.

    attitude (pitch/roll) has NO feedback from velocity in ins_compute_Phi
    (no Earth-rotation/transport-rate term: a deliberate flat-Earth/local-
    tangent-plane simplification) - it evolves autonomously, so the
    coupling is one-way and superposes exactly, no cross-covariance term:
    the tilt error over [0,T] is pitch(t) = -y2(gyro bias path) + y1(gyro
    white-noise path) (see process_noise_growth_rate's "att"), and the
    velocity contribution is -g times its running integral, i.e. one more
    integration of that same pair -> the SAME shape as
    _position_state_growth_var(q1_gyr, q2_gyr, T), scaled by g^2 (variance,
    so g not g^2... squared because Var(g*X) = g^2*Var(X)):
        Var(T) = [direct accel path] + g^2 * [attitude "position"]
               = _rate_state_growth_var(q1_acc, q2_acc, T)
                 + g^2 * _position_state_growth_var(q1_gyr, q2_gyr, T)

    The vertical (down) channel has no such coupling under level flight
    (fnN=fnE=0 there), so this models the (worse) horizontal channel -
    the representative number to show, matching the isotropic acc_var/
    gyr_var simplification used everywhere else in this file."""
    accel_path = _rate_state_growth_var(q1_acc, q2_acc, T)
    tilt_path = g ** 2 * _position_state_growth_var(q1_gyr, q2_gyr, T)
    return accel_path + tilt_path


def _tilt_coupled_position_growth_var(q1_acc, q2_acc, q1_gyr, q2_gyr, T,
                                      g=STANDARD_GRAVITY_MPS2):
    """Var(T) of HORIZONTAL position error under free coasting: position is
    one more integral of _tilt_coupled_velocity_growth_var's velocity, so
    each path there gains one more integration order (n -> n+1):
        Var(T) = [direct accel path, unchanged from before this term
                  existed] + g^2 * [one more integral of the attitude
                  "position" term]
               = _position_state_growth_var(q1_acc, q2_acc, T)
                 + g^2 * (y3(q1_gyr) + y4(q2_gyr))
    See _tilt_coupled_velocity_growth_var for the derivation this extends."""
    accel_path = _position_state_growth_var(q1_acc, q2_acc, T)
    tilt_path = g ** 2 * (_integrated_white_noise_var(q1_gyr, T, 3)
                         + _integrated_white_noise_var(q2_gyr, T, 4))
    return accel_path + tilt_path


def process_noise_growth_rate(noise, horizon_sec=GROWTH_HORIZON_SEC):
    """Theoretical 1-σ uncertainty GROWTH RATE of the ins's own
    position/velocity/attitude under free inertial coasting (no aiding at
    all), driven purely by the configured imu process noise (gyr_psd/
    acc_psd white noise plus gyr_bias_rw/acc_bias_rw random walk) -
    independent of any particular replay/filter run, a pure function of the
    noise model.

    Attitude has no process-noise coupling FROM anywhere else (see
    _rate_state_growth_var) - driven by gyro white noise + gyro bias random
    walk, no feedback from velocity/position in ins_compute_Phi. It DOES,
    however, feed velocity/position: besides their own direct accel-noise/
    accel-bias path (_rate_state_growth_var/_position_state_growth_var),
    ins_compute_Phi couples them to the (independently growing) attitude
    error via the specific-force cross product - gravity leaking through a
    tilt error into a spurious horizontal acceleration. See
    _tilt_coupled_velocity_growth_var/_tilt_coupled_position_growth_var for
    that derivation; modeled for the horizontal channel (worse case than
    vertical, which has no tilt coupling under level flight), matching the
    isotropic acc_var/gyr_var simplification used elsewhere in this file.

    On top of all of the above, ins.c ALWAYS adds its own extra process-
    noise margin directly to pos/vel/rpy (config.yaml's imu: pos/vel/
    rpy_pred_stddev_*_sqrts) - it enters each state's OWN derivative
    exactly like a direct sensor-noise term (n=1 in
    _integrated_white_noise_var's terms), so it folds additively into
    q1_acc (vel's direct-noise level) and q1_gyr (attitude's), and once
    folded in there it correctly rides along through the same tilt-coupling
    math into velocity/position too; pos gets its own extra term directly
    (nothing downstream depends on position, so it doesn't propagate
    further).

    Returns None if any of pos/vel/rpy_pred_stddev_*_sqrts is left at 0 in
    `noise` - that means the dataset defers to ins.c's own built-in
    default (INS_DEFAULT_POS/VEL/RPY_PRED_STDDEV_*), and this module
    deliberately does NOT hardcode a copy of those C-side constants to work
    around it: a second, python-side copy of a C #define is exactly the
    kind of thing that silently goes stale the day someone tunes the C
    default and forgets this file exists (this whole function originally
    shipped broken for exactly that class of reason - see
    tests/test_growth_rate.py). Set the three fields explicitly in
    config.yaml's imu: section to get an estimate here.

    None of these are constant slopes in the strict calculus sense (σ
    grows with sqrt(t)/t^1.5, not linearly) - "growth rate" here means the
    AVERAGE rate over `horizon_sec` (σ(horizon)/horizon), which is why
    position/velocity/attitude come out in m/s, m/s^2 and rad/s
    respectively (matching each quantity's own unit divided by seconds).
    Read it as: "coasting blind for `horizon_sec`, my uncertainty grows by
    about this much per second, on average over that window" - NOT a
    per-sqrt(s) ARW/VRW density (see gyr_bias_rw/acc_bias_rw docs for
    that)."""
    pos_margin = noise.get("pos_pred_stddev_m_sqrts", 0.0)
    vel_margin = noise.get("vel_pred_stddev_mps_sqrts", 0.0)
    rpy_margin = noise.get("rpy_pred_stddev_rad_sqrts", 0.0)
    if not (pos_margin and vel_margin and rpy_margin):
        return None

    T = horizon_sec
    # Effective q1's: the physically-modeled sensor noise PLUS ins.c's own
    # always-on extra margin, which enters at the exact same level (see
    # docstring above).
    q1_acc = noise["acc_psd"] + vel_margin ** 2
    q2_acc = noise.get("acc_bias_rw", 0.0) ** 2
    q1_gyr = noise["gyr_psd"] + rpy_margin ** 2
    q2_gyr = noise.get("gyr_bias_rw", 0.0) ** 2
    q_pos_extra = pos_margin ** 2

    var_pos = (_tilt_coupled_position_growth_var(q1_acc, q2_acc, q1_gyr, q2_gyr, T)
              + q_pos_extra * T)
    var_vel = _tilt_coupled_velocity_growth_var(q1_acc, q2_acc, q1_gyr, q2_gyr, T)
    var_att = _rate_state_growth_var(q1_gyr, q2_gyr, T)

    return {
        "horizon_sec": T,
        "pos_mps": math.sqrt(var_pos) / T,
        "vel_mps2": math.sqrt(var_vel) / T,
        "att_radps": math.sqrt(var_att) / T,
    }


def baro_alt_growth_rate(baro_cfg, horizon_sec=GROWTH_HORIZON_SEC):
    """Same free-coasting growth-rate idea as process_noise_growth_rate(),
    for baro_alt's own height/vertical-velocity/accel-bias states (baro_alt
    always runs as part of nav_suite, whether or not a barometer is
    actually configured - it's what dead-reckons the vertical channel
    between/without baro samples).

    Returns None if either baro: acc_noise_mps2_sqrthz or acc_bias_rw is
    left at 0 in `baro_cfg` - see process_noise_growth_rate()'s docstring
    for why this deliberately does not fall back to a hardcoded copy of
    baro_alt.c's own default."""
    acc_noise = baro_cfg.get("acc_noise_mps2_sqrthz", 0.0)
    bias_rw = baro_cfg.get("acc_bias_rw", 0.0)
    if not (acc_noise and bias_rw):
        return None

    T = horizon_sec
    q1 = acc_noise ** 2
    q2 = bias_rw ** 2

    return {
        "horizon_sec": T,
        "bias_rw_mps2_sqrths": bias_rw,
        "h_mps": math.sqrt(_position_state_growth_var(q1, q2, T)) / T,
        "v_mps2": math.sqrt(_rate_state_growth_var(q1, q2, T)) / T,
        "bias_mps2_per_s": math.sqrt(q2 * T) / T,
    }


def ahrs_growth_rate(ahrs_cfg, horizon_sec=GROWTH_HORIZON_SEC):
    """Same free-coasting growth-rate idea as process_noise_growth_rate(),
    for the ARS/AHRS sub-filters' own attitude state - both share one
    number (nav_suite.h: ars_cfg/ahrs_cfg are set from the SAME ahrs:
    config, see Navigator.set_ahrs_gyr_noise()); there is no separate
    position/velocity state to report (ars/ahrs carry only attitude + gyro
    bias).

    Returns None if either ahrs: gyr_noise_psd or gyr_bias_rw is left at 0
    in `ahrs_cfg` - see process_noise_growth_rate()'s docstring for why
    this deliberately does not fall back to a hardcoded copy of ahrs.c's
    own default."""
    gyr_noise = ahrs_cfg.get("gyr_noise_psd", 0.0)
    bias_rw = ahrs_cfg.get("gyr_bias_rw", 0.0)
    if not (gyr_noise and bias_rw):
        return None

    T = horizon_sec
    q1 = gyr_noise ** 2
    q2 = bias_rw ** 2
    return {
        "horizon_sec": T,
        "att_radps": math.sqrt(_rate_state_growth_var(q1, q2, T)) / T,
    }


def growth_rate_lines(noise):
    """process_noise_growth_rate(), formatted as the plain-text lines shared
    by replay.py's console summary and inspostgui.py's GUI summary."""
    g = process_noise_growth_rate(noise)
    if g is None:
        return [
            "ins process noise growth rate: N/A - set "
            "imu.pos_pred_stddev_m_sqrts / vel_pred_stddev_mps_sqrts / "
            "rpy_pred_stddev_rad_sqrts explicitly in config.yaml (currently "
            "0 -> default)",
        ]
    return [
        f"ins process noise growth rate (avg over {g['horizon_sec']:.0f} s "
        f"free-inertial coasting, no aiding):",
        f"  pos   {g['pos_mps']:.4f} m/s",
        f"  vel   {g['vel_mps2']:.4f} m/s^2",
        f"  att   {math.degrees(g['att_radps']):.4f} deg/s",
    ]


def baro_growth_rate_lines(baro_cfg):
    """baro_alt_growth_rate(), formatted as plain-text lines."""
    g = baro_alt_growth_rate(baro_cfg)
    if g is None:
        return [
            "baro_alt process noise growth rate: N/A - set "
            "baro.acc_noise_mps2_sqrthz / acc_bias_rw explicitly in "
            "config.yaml (currently 0 -> baro_alt's default)",
        ]
    return [
        f"baro_alt process noise growth rate (avg over {g['horizon_sec']:.0f} s "
        f"free vertical coasting, no baro):",
        f"  height    {g['h_mps']:.4f} m/s",
        f"  vel (-D)  {g['v_mps2']:.4f} m/s^2",
        f"  acc bias  {g['bias_mps2_per_s']:.6f} m/s^2/s  "
        f"(bias_rw {g['bias_rw_mps2_sqrths']:.2e} m/s^2/sqrt(Hz))",
    ]


def ahrs_growth_rate_lines(ahrs_cfg):
    """ahrs_growth_rate(), formatted as plain-text lines."""
    g = ahrs_growth_rate(ahrs_cfg)
    if g is None:
        return [
            "ars/ahrs process noise growth rate: N/A - set "
            "ahrs.gyr_noise_psd / gyr_bias_rw explicitly in config.yaml "
            "(currently 0 -> ahrs's own built-in default)",
        ]
    return [
        f"ars/ahrs process noise growth rate (avg over {g['horizon_sec']:.0f} s "
        f"free coasting):",
        f"  att   {math.degrees(g['att_radps']):.4f} deg/s",
    ]


def last_fix_accuracy_lines(last_fix):
    """Format the most recent fused GNSS fix's OWN reported 1-σ accuracy
    (receiver covariance, after apply_fallback), for a quick side-by-side
    against the filter's final 1-σ. None if no fix was ever usable
    (cov_pos None - rejected/unavailable)."""
    if last_fix is None or last_fix.get("cov_pos") is None:
        return None
    cov_p = last_fix["cov_pos"]
    lines = ["last GNSS fix accuracy (receiver-reported 1-σ):"]
    lines.append(f"  pos NED   {math.sqrt(cov_p[0][0]):.3f} "
                f"{math.sqrt(cov_p[1][1]):.3f} {math.sqrt(cov_p[2][2]):.3f} m")
    cov_v = last_fix.get("cov_vel")
    if last_fix.get("vel_ok") and cov_v is not None:
        lines.append(f"  vel NED   {math.sqrt(cov_v[0][0]):.3f} "
                    f"{math.sqrt(cov_v[1][1]):.3f} {math.sqrt(cov_v[2][2]):.3f} m/s")
    return lines


def load_gnss(path):
    """gnss.csv (datasets/replay_format.py): t_us, lat/lon/h, full NED
    position covariance (6 unique elements), NED velocity, full NED
    velocity covariance, vel_ok."""
    fixes = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if len(p) < 20:
                continue
            v = [float(x) for x in p]
            fixes.append({
                "t_us": int(p[0]),
                "lat_rad": math.radians(v[1]),
                "lon_rad": math.radians(v[2]),
                "h_m": v[3],
                "cov_pos": v[4:10],
                "vel_ned": (v[10], v[11], v[12]),
                "cov_vel": v[13:19],
                "vel_ok": int(v[19]) != 0,
            })
    return fixes


def load_txyz(path):
    """mag.csv: t_us + 3 values per line."""
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if len(p) < 4:
                continue
            rows.append((int(p[0]), (float(p[1]), float(p[2]), float(p[3]))))
    return rows


def mag_calibrate(v, misalignment, fixed_bias):
    """One raw magnetometer sample [uT] through the config.yaml fixed
    calibration, mirroring ins.c's ins_imu_calibrate(): corrected =
    M * (raw - fixed_bias), an all-zero M meaning identity. The library
    applies this internally to every fused sample, so replay has to repeat it
    to show/derive anything from the same field the filter actually sees."""
    c = [v[i] - fixed_bias[i] for i in range(3)]
    if any(misalignment):
        return _matvec_cm(misalignment, c)
    return c


def load_baro(path):
    """baro.csv: t_us, static pressure [Pa]."""
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if len(p) < 2:
                continue
            rows.append((int(p[0]), float(p[1])))
    return rows


def load_speed(path):
    """speed.csv -> [(t_us, speed_mps)].

    Two spellings are accepted, because the file has two origins. The
    logger (tools/inslib_udp_to_serial.py) writes a NAMED header carrying
    both timebases and more besides, and reading it directly means a drive
    can be replayed without a conversion step. A converter may instead
    emit the plain dataset form "t_us, speed_mps, stddev_mps[, delay_ms]"
    behind a '#' comment header, like every other stream here.

    Only the timestamp and the speed are read. Uncertainty and delay come
    from config.yaml as constants, so one place describes the dataset -
    the producer's own per-sample columns stay in the file as a record of
    what it measured, and can be promoted to inputs later if a dataset
    ever needs them to vary.

    Rows without an MCU timestamp are dropped: they arrived before the
    host/MCU clock relation was established and there is nothing to place
    them on. Only kind == "ground" is fused - a relative airspeed would be
    a different measurement model, not another source of |v|."""
    rows = []
    with open(path, encoding="utf-8") as f:
        first = f.readline()
        named = first.lower().startswith("utc_iso") or "imu_t_us" in first.lower()
        idx = {}
        if named:
            cols = [c.strip().lower() for c in first.strip().split(",")]
            idx = {name: i for i, name in enumerate(cols)}
            for need in ("imu_t_us", "speed_mps"):
                if need not in idx:
                    raise SystemExit("%s: named header without a '%s' column"
                                     % (path, need))
        else:
            f.seek(0)
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            p = line.strip().split(",")
            try:
                if named:
                    if idx.get("kind") is not None and p[idx["kind"]].strip() != "ground":
                        continue
                    t_raw = p[idx["imu_t_us"]].strip()
                    if not t_raw:
                        continue
                    t_us = int(t_raw)
                    speed = float(p[idx["speed_mps"]])
                else:
                    if len(p) < 2:
                        continue
                    t_us = int(p[0])
                    speed = float(p[1])
            except (ValueError, IndexError):
                continue
            rows.append((t_us, speed))
    rows.sort(key=lambda r: r[0])
    return rows


def iter_imu(path):
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if len(p) < 7:
                continue
            yield (int(p[0]),
                   (float(p[1]), float(p[2]), float(p[3])),   # gyro
                   (float(p[4]), float(p[5]), float(p[6])))   # acc


def estimate_gyro_bias(imu_path, window_sec):
    total = [0.0, 0.0, 0.0]
    n = 0
    t0 = None
    for t, g, _a in iter_imu(imu_path):
        if t0 is None:
            t0 = t
        if (t - t0) > window_sec * US_PER_SEC:
            break
        total = [total[i] + g[i] for i in range(3)]
        n += 1
    if n < 10:
        return None
    return tuple(total[i] / n for i in range(3))


def build_config(spec, ref0, t0_us, lat0, lon0, h0, gyr_bias):
    have_bias = gyr_bias is not None
    noise = spec["imu"]
    gnss = spec["gnss"]
    init_ref = spec["init"] == "ref"
    # REQ-VER-010: config init_stddev.* overrides the built-in default
    # (each field individually, 0 -> unchanged). The init_stddev.* YAML
    # keys are a stable schema shared with the C replay harness
    # (tools/replay.c), mirroring ins.Config's own field names.
    # yaw_init_stddev_rad_deg has no built-in default of its own to
    # override - it maps onto rpy_init_stddev_rad[2] independently of
    # roll/pitch, so it's only passed through when the config actually
    # asks for it (0 -> falls back to the roll/pitch value below).
    init_sd = spec["init_stddev"]
    rp_stddev_rad = math.radians(
        init_sd["rpy_init_stddev_rad_deg"] or (1.0 if init_ref else 3.0))
    yaw_stddev_rad = (math.radians(init_sd["yaw_init_stddev_rad_deg"])
                      if init_sd["yaw_init_stddev_rad_deg"] else rp_stddev_rad)
    cfg = Config(
        time_us=t0_us,
        lat_rad=lat0, lon_rad=lon0, h_m=h0,
        # free_inertial_start: the origin IS the declared point, so the
        # uncertainty of that statement is the initial position uncertainty
        # by construction (tools/replay.c and tools/insrcv.c do the same).
        # An explicit init_stddev.pos_init_stddev_m still wins.
        pos_init_stddev_m=(init_sd["pos_init_stddev_m"]
                           or (spec["free_inertial_start"]["stddev_m"]
                               if spec["free_inertial_start"]["enable"] else None)
                           or (0.1 if init_ref else 1.0)),
        vel_init_stddev_mps=(init_sd["vel_init_stddev_mps"]
                             or (0.1 if init_ref else 0.5)),
        rpy_init_stddev_rad=(rp_stddev_rad, rp_stddev_rad, yaw_stddev_rad),
        acc_bias_init_stddev_mps2=init_sd["acc_bias_init_stddev_mps2"] or 0.05,
        gyr_bias_init_stddev_rps=math.radians(
            init_sd["gyr_bias_init_stddev_rps_deg"] or (0.2 if have_bias else 0.5)),
        pos_pred_stddev_m_sqrts=noise["pos_pred_stddev_m_sqrts"],
        vel_pred_stddev_mps_sqrts=noise["vel_pred_stddev_mps_sqrts"],
        rpy_pred_stddev_rad_sqrts=noise["rpy_pred_stddev_rad_sqrts"],
        acc_bias_pred_stddev_mps2_sqrts=noise["acc_bias_rw"],
        gyr_bias_pred_stddev_rps_sqrts=noise["gyr_bias_rw"],
        zero_vel_stddev_mps=noise["zero_vel_stddev_mps"] or 0.05,
        zero_rot_stddev_rps=math.radians(noise["zero_rot_stddev_deg"] or 0.1),
        # Stillness detection (REQ-SUITE-020): one set, reaching ins, both
        # AHRS instances and baro_alt.
        auto_zupt_static_gyr_rps=math.radians(noise["auto_zupt_static_gyr_deg"]),
        auto_zupt_static_acc_mps2=noise["auto_zupt_static_acc_mps2"],
        auto_zupt_static_gyr_stddev_rps=math.radians(
            noise["auto_zupt_static_gyr_stddev_deg"]),
        auto_zupt_static_acc_stddev_mps2=noise["auto_zupt_static_acc_stddev_mps2"],
        auto_zupt_max_vel_mps=noise["auto_zupt_max_vel_mps"],
        auto_zupt_max_vel_stddev_mps=noise["auto_zupt_max_vel_stddev_mps"],
        auto_zupt_dwell_sec=noise["auto_zupt_dwell_sec"],
        auto_zupt_min_interval_sec=noise["auto_zupt_min_interval_sec"],
        auto_zupt_disable=bool(int(noise["auto_zupt_disable"])),
        auto_zupt_velocity_blind_disable=bool(
            int(noise["auto_zupt_velocity_blind_disable"])),
        gyr_bias_init_rps=gyr_bias if have_bias else (0.0, 0.0, 0.0),
        # kalman_update_dt_sec left at 0 -> default
        max_prediction_time_sec=0.5,
        gnss_max_horizontal_pos_stddev_m=gnss["max_horizontal_pos_stddev_m"],
        gnss_max_vertical_pos_stddev_m=gnss["max_vertical_pos_stddev_m"],
        gnss_max_horizontal_vel_stddev_mps=gnss["max_horizontal_vel_stddev_mps"],
        gnss_max_vertical_vel_stddev_mps=gnss["max_vertical_vel_stddev_mps"],
        # REQ-NAV-051/052: solution-mode entry/exit gates + their dwells.
        gnss_start_max_horizontal_pos_stddev_m=gnss["start_max_horizontal_pos_stddev_m"],
        gnss_start_max_vertical_pos_stddev_m=gnss["start_max_vertical_pos_stddev_m"],
        gnss_start_max_horizontal_vel_stddev_mps=gnss["start_max_horizontal_vel_stddev_mps"],
        gnss_start_max_vertical_vel_stddev_mps=gnss["start_max_vertical_vel_stddev_mps"],
        gnss_stop_max_horizontal_pos_stddev_m=gnss["stop_max_horizontal_pos_stddev_m"],
        gnss_stop_max_vertical_pos_stddev_m=gnss["stop_max_vertical_pos_stddev_m"],
        gnss_stop_max_horizontal_vel_stddev_mps=gnss["stop_max_horizontal_vel_stddev_mps"],
        gnss_stop_max_vertical_vel_stddev_mps=gnss["stop_max_vertical_vel_stddev_mps"],
        gnss_init_dwell_sec=gnss["init_dwell_sec"],
        gnss_init_dwell_disable=bool(gnss["init_dwell_disable"]),
        gnss_stop_dwell_sec=gnss["stop_dwell_sec"],
        gnss_stop_disable=bool(gnss["stop_disable"])
                          or bool(spec["free_inertial_start"]["enable"]),
        gnss_pos_decimation=int(gnss["pos_decimation"]),
        gnss_min_delay_ms=int(gnss["min_delay_ms"]),
        magnetometer_min_delay_ms=int(spec["mag"]["min_delay_ms"]),
        auto_init=not init_ref,
        # free_inertial_start is developer mode: coasting is the intent, so
        # neither the dead-reckoning budget nor the 3D exit gate may end the
        # run (tools/insrcv.c forces the same two).
        allow_unlimited_deadreckoning=bool(spec["allow_unlimited_deadreckoning"])
                                      or bool(spec["free_inertial_start"]["enable"]),
        max_deadreckoning_sec=float(spec["max_deadreckoning_sec"]),
        baro_height_disable=bool(spec["baro_height_disable"]),
        automotive_mode=bool(spec["automotive_mode"]),
        automotive_min_speed_mps=spec["automotive_min_speed_mps"],
        automotive_min_yaw_stddev=math.radians(spec["automotive_min_yaw_stddev_deg"]),
        automotive_lateral_constraint=bool(spec["automotive_lateral_constraint"]),
        automotive_lateral_stddev_mps=spec["automotive_lateral_stddev_mps"],
        automotive_lateral_max_yaw_rate=math.radians(
            spec["automotive_lateral_max_yaw_rate_deg"]),
        automotive_lateral_after_sec=spec["automotive_lateral_after_sec"],
        chi2_disable=bool(spec["chi2_disable"]),
        chi2_reject_alpha=float(spec["chi2_reject_alpha"]),
        # Absolute speed aiding calibration (REQ-NAV-068). 0 keeps the
        # library defaults, so a config without a speed: section is a no-op.
        speed_scale=float(spec["speed"]["scale"]),
        speed_stddev_rel=float(spec["speed"]["stddev_rel"]),
        speed_min_mps=float(spec["speed"]["min_speed_mps"]),
        estimate_mag_bias=bool(spec["mag"]["estimate_bias"]),
        # IMU calibration (REQ-NAV-037) + GNSS covariance conditioning
        # (REQ-NAV-038); .get() keeps older configs (no such keys) working.
        imu_acc_misalignment=tuple(noise.get("acc_misalignment", (0.0,) * 9)),
        imu_gyr_misalignment=tuple(noise.get("gyr_misalignment", (0.0,) * 9)),
        imu_acc_fixed_bias=tuple(noise.get("acc_fixed_bias", (0.0, 0.0, 0.0))),
        imu_gyr_fixed_bias=tuple(noise.get("gyr_fixed_bias", (0.0, 0.0, 0.0))),
        gnss_pos_cov_scale=gnss.get("pos_cov_scale", 0.0),
        gnss_pos_cov_scale_height=gnss.get("pos_cov_scale_height", 0.0),
        gnss_vel_cov_scale=gnss.get("vel_cov_scale", 0.0),
        gnss_pos_stddev_floor_hor_m=gnss.get("pos_stddev_floor_hor_m", 0.0),
        gnss_pos_stddev_floor_ver_m=gnss.get("pos_stddev_floor_ver_m", 0.0),
        gnss_vel_stddev_floor_hor_mps=gnss.get("vel_stddev_floor_hor_mps", 0.0),
        gnss_vel_stddev_floor_ver_mps=gnss.get("vel_stddev_floor_ver_mps", 0.0),
        # Accuracy caps (REQ-NAV-071), envelope (REQ-NAV-072) and the
        # manoeuvre-dependent velocity noise (REQ-NAV-073).
        gnss_pos_stddev_cap_hor_m=gnss.get("pos_stddev_cap_hor_m", 0.0),
        gnss_pos_stddev_cap_ver_m=gnss.get("pos_stddev_cap_ver_m", 0.0),
        gnss_vel_stddev_cap_hor_mps=gnss.get("vel_stddev_cap_hor_mps", 0.0),
        gnss_vel_stddev_cap_ver_mps=gnss.get("vel_stddev_cap_ver_mps", 0.0),
        gnss_acc_envelope_tau_sec=gnss.get("acc_envelope_tau_sec", 0.0),
        gnss_vel_noise_acc_scale_hor=gnss.get("vel_noise_acc_scale_hor", 0.0),
        gnss_vel_noise_acc_scale_ver=gnss.get("vel_noise_acc_scale_ver", 0.0),
        gnss_vel_noise_acc_window_sec=gnss.get("vel_noise_acc_window_sec", 0.0),
        mag_misalignment=tuple(spec["mag"].get("misalignment", (0.0,) * 9)),
        mag_fixed_bias=tuple(spec["mag"].get("fixed_bias", (0.0, 0.0, 0.0))),
        auto_init_window_sec=float(spec.get("auto_init_window_sec", 0.0)),
    )
    if init_ref:
        # Known initial state from the reference: attitude AND velocity, so a
        # non-stationary start (e.g. an aircraft cruising at 200 m/s) begins
        # at the right velocity instead of 0 and having to catch up on GNSS.
        cfg.rpy_init_rad = (ref0["roll_rad"], ref0["pitch_rad"], ref0["yaw_rad"])
        cfg.init_vel_ned = tuple(ref0["vel_ned"])
    return cfg


class Stat:
    def __init__(self):
        self.sum = self.sum2 = 0.0
        self.max_abs = 0.0
        self.n = 0

    def add(self, e):
        self.sum += e
        self.sum2 += e * e
        if abs(e) > self.max_abs:
            self.max_abs = abs(e)
        self.n += 1

    def mean(self):
        return self.sum / self.n if self.n else 0.0

    def std(self):
        if self.n < 2:
            return 0.0
        m = self.mean()
        return math.sqrt(max(self.sum2 / self.n - m * m, 0.0))

    def rms(self):
        return math.sqrt(self.sum2 / self.n) if self.n else 0.0


def _stream_gap_stats(timestamps_us):
    """(n, duration_sec, avg_hz, max_gap_sec, max_gap_at_sec) for a list
    of non-decreasing int microsecond timestamps. avg_hz is (n-1)/span,
    not a count/duration ratio, so a single outage doesn't quietly
    lower it - that's what max_gap_sec is for. n < 2 -> everything but
    n is 0.0."""
    n = len(timestamps_us)
    if n < 2:
        return n, 0.0, 0.0, 0.0, 0.0
    duration = (timestamps_us[-1] - timestamps_us[0]) / US_PER_SEC
    avg_hz = (n - 1) / duration if duration > 0 else 0.0
    max_gap = 0.0
    max_gap_at = 0.0
    prev = timestamps_us[0]
    for ts in timestamps_us[1:]:
        gap = (ts - prev) / US_PER_SEC
        if gap > max_gap:
            max_gap = gap
            max_gap_at = (ts - timestamps_us[0]) / US_PER_SEC
        prev = ts
    return n, duration, avg_hz, max_gap, max_gap_at


def _bucketed_rate(timestamps_us, t0_us, bucket_sec):
    """Sample rate [Hz] over time for an already in-memory timestamp list
    (GNSS/mag/baro): count per fixed-width `bucket_sec`
    window since `t0_us` (the same origin as rec["t"] and the IMU rate
    from _imu_prepass, so all sensor-rate lines share one time axis).
    Returns (bucket_center_sec, hz) parallel lists; empty if there are
    fewer than 2 timestamps."""
    if len(timestamps_us) < 2:
        return [], []
    counts = []
    for t in timestamps_us:
        idx = int((t - t0_us) / US_PER_SEC / bucket_sec)
        if idx < 0:
            continue
        while len(counts) <= idx:
            counts.append(0)
        counts[idx] += 1
    hz = [c / bucket_sec for c in counts]
    t_center = [(i + 0.5) * bucket_sec for i in range(len(counts))]
    return t_center, hz


def _imu_prepass(imu_path, bucket_sec=5.0):
    """Same shape as _stream_gap_stats, but streamed (O(1) memory)
    instead of list-based: doubles as the progress bar's sample count
    (n_imu_total), the IMU gap stats, a whole-trial noise estimate, and
    a bucketed rate-over-time series, in the one extra
    full pass that was already needed for the former - the IMU stream
    can be too large (tens of millions of samples on a long trial) to
    hold as a raw timestamp list for post-hoc bucketing the way
    _bucketed_rate() does for the lower-rate streams.

    The whole-trial noise estimate (gyr_d2_stat/acc_d2_stat) uses SECOND-
    difference sampling: d2[n] = x[n] - 2*x[n-1] + x[n-2]. This cancels a
    constant bias AND a constant first derivative (steady rate / constant
    angular acceleration), so smooth vehicle dynamics - which a plain
    first difference still leaks in as alpha*dt - drop out, leaving
    jerk+noise. It stays a high-pass, so real vibration is kept: exactly
    the split a gyro/accel NOISE model wants (sensor noise + vibration, not
    vehicle motion). σ = stddev(d2)/sqrt(6) (Var = 6*σ^2 for white
    noise). It works even when the platform never stops - unlike the
    stationary-epoch measure (see main()), which needs the platform to
    actually stand still at some point. Only accumulated when BOTH
    bracketing intervals are contiguous (no gap), so one sensor dropout
    can't fabricate a huge spurious jerk sample."""
    n = 0
    t0 = prev = None
    max_gap = max_gap_at = 0.0
    prev_g = prev_a = None
    prev2_g = prev2_a = None
    prev_gap_ok = False
    gyr_d2_stat = [Stat(), Stat(), Stat()]
    acc_d2_stat = [Stat(), Stat(), Stat()]
    rate_counts = []
    for t, g, a in iter_imu(imu_path):
        if t0 is None:
            t0 = t
        elif t > prev:
            gap = (t - prev) / US_PER_SEC
            if gap > max_gap:
                max_gap = gap
                max_gap_at = (t - t0) / US_PER_SEC
            gap_ok = gap < 0.1
            if gap_ok and prev_gap_ok and prev2_g is not None:
                for k in range(3):
                    gyr_d2_stat[k].add(g[k] - 2.0 * prev_g[k] + prev2_g[k])
                    acc_d2_stat[k].add(a[k] - 2.0 * prev_a[k] + prev2_a[k])
            prev_gap_ok = gap_ok
        bidx = int((t - t0) / US_PER_SEC / bucket_sec)
        while len(rate_counts) <= bidx:
            rate_counts.append(0)
        rate_counts[bidx] += 1
        prev2_g, prev2_a = prev_g, prev_a
        prev_g, prev_a = g, a
        prev = t
        n += 1
    duration = (prev - t0) / US_PER_SEC if n > 1 else 0.0
    avg_hz = (n - 1) / duration if duration > 0 else 0.0
    rate_hz = [c / bucket_sec for c in rate_counts]
    rate_t = [(i + 0.5) * bucket_sec for i in range(len(rate_counts))]
    return (n, duration, avg_hz, max_gap, max_gap_at,
           gyr_d2_stat, acc_d2_stat, t0, rate_t, rate_hz)


def estimate_gnss_delay_ms(t_sec, baro_vel_d, gnss_vel_d, max_lag_sec=3.0):
    """Cross-correlate baro_alt's down-velocity (assumed ~zero latency)
    against the held GNSS down-velocity to estimate the GNSS fix's
    processing/telemetry latency: a positive result means
    GNSS lags baro. `t_sec` must be uniformly sampled (see the --plot-hz
    recorder); NaN samples (baro/GNSS not yet seen) are dropped first.

    Returns (delay_ms, correlation) where correlation is the best
    normalized dot product in [-1, 1] - close to 1 means a clean,
    unambiguous match; a low or flat-topped result means the trial
    doesn't have enough vertical motion (climbs/descents) to tell.
    Returns None if there isn't enough data.

    CAVEAT: baro_alt is a Kalman-filtered estimate, not a raw sensor -
    "assumed ~zero latency" is an approximation. A high-correlation
    result is the RELATIVE delay between the two signals; it cannot by
    itself tell you whether that delay is GNSS's, baro_alt's own group
    delay, or both. See the module docstring for an empirical example."""
    curve = gnss_delay_correlation_curve(t_sec, baro_vel_d, gnss_vel_d, max_lag_sec)
    if curve is None:
        return None
    return max(curve, key=lambda row: row[1])


def gnss_delay_correlation_curve(t_sec, baro_vel_d, gnss_vel_d, max_lag_sec=3.0):
    """Shared computation behind estimate_gnss_delay_ms(), also used to
    plot the correlation-vs-lag curve (see ins_plots.py). Returns a
    list of (lag_ms, correlation) covering [-max_lag_sec, +max_lag_sec],
    or None if there isn't enough data."""
    rows = [(t, b, g) for t, b, g in zip(t_sec, baro_vel_d, gnss_vel_d)
           if b == b and g == g]  # drop NaN (b != b iff NaN)
    n = len(rows)
    if n < 20:
        return None
    ts = [r[0] for r in rows]
    dt = (ts[-1] - ts[0]) / (n - 1)
    if dt <= 0:
        return None
    b = [r[1] for r in rows]
    g = [r[2] for r in rows]
    bmean = sum(b) / n
    gmean = sum(g) / n
    b = [x - bmean for x in b]
    g = [x - gmean for x in g]

    max_lag = max(1, int(round(max_lag_sec / dt)))
    curve = []
    for lag in range(-max_lag, max_lag + 1):
        # Compare gnss[i + lag] against baro[i]: lag > 0 means gnss
        # reports now what baro already saw `lag` samples ago (GNSS
        # lagging baro, the normal case for a real receiver).
        if lag >= 0:
            bb, gg = b[: n - lag], g[lag:]
        else:
            bb, gg = b[-lag:], g[: n + lag]
        m = len(bb)
        if m < 10:
            continue
        num = sum(x * y for x, y in zip(bb, gg))
        nb = math.sqrt(sum(x * x for x in bb))
        ng = math.sqrt(sum(y * y for y in gg))
        if nb < 1e-9 or ng < 1e-9:
            continue
        curve.append((lag * dt * 1000.0, num / (nb * ng)))
    return curve or None


def _pooled_within_phase_std(per_phase_axis_lists):
    """Pooled within-phase stddev: remove each phase's per-axis mean (so an
    unknown constant offset - position, field bias - doesn't inflate the
    estimate) and pool the residual deviations across all phases/axes
    (dof = sum of n-1). Returns (stddev, dof), or (None, 0) if too few
    samples. Shared by the GNSS/baro/mag standstill checks."""
    ss = 0.0
    dof = 0
    for vals in per_phase_axis_lists:
        n = len(vals)
        if n < 2:
            continue
        m = sum(vals) / n
        ss += sum((v - m) ** 2 for v in vals)
        dof += n - 1
    return (math.sqrt(ss / dof), dof) if dof > 0 else (None, 0)


def gnss_standstill_accuracy(phases, min_seg_fixes=4):
    """Compare the empirical spread of the GNSS fixes DURING STANDSTILL
    against the accuracy the receiver itself reported for those same
    fixes: the gnss.csv covariance, or the config fallback
    stddev where the csv gave none - whichever was actually fused (both
    already merged into each fix's cov_pos/cov_vel by apply_fallback), so
    the "from the csv, else from config.yaml" cases are handled uniformly.

    During a true standstill the platform position is constant and its
    velocity is zero, so the per-phase spread of the fixes IS the GNSS
    measurement noise - a direct check of whether the reported 1-σ is
    realistic (measured > reported => the receiver's covariance is too
    optimistic; measured < reported => too pessimistic).

    `phases` is a list of standstill phases (each a list of per-fix sample
    dicts, see main()'s collector). Only phases with at least
    `min_seg_fixes` static fixes are used (need a few samples for a
    meaningful stddev), and the per-phase mean is removed per axis so an
    unknown constant position (or a small velocity bias) doesn't inflate
    the estimate - the within-phase deviations are pooled across all
    phases (dof = sum of n-1 per phase per axis). N and E are pooled into
    one horizontal figure, D is the vertical one, matching the config's
    (horizontal, vertical) stddev split. Returns None if there aren't
    enough standstill fixes to tell - a trial that never stops simply
    can't be checked this way."""
    used = [ph for ph in phases if len(ph) >= min_seg_fixes]
    n_fix = sum(len(ph) for ph in used)
    if not used:
        return None

    hor_pos = ([[s["pos_ned"][0] for s in ph] for ph in used]
               + [[s["pos_ned"][1] for s in ph] for ph in used])
    ver_pos = [[s["pos_ned"][2] for s in ph] for ph in used]
    hor_vel = ([[s["vel_ned"][0] for s in ph if s["vel_ned"]] for ph in used]
               + [[s["vel_ned"][1] for s in ph if s["vel_ned"]] for ph in used])
    ver_vel = [[s["vel_ned"][2] for s in ph if s["vel_ned"]] for ph in used]

    def rep_rms(key):
        # Typical reported 1-σ over the static fixes: variances add,
        # so RMS the per-fix stddevs (they are ~constant during a stop).
        vals = [s[key] for ph in used for s in ph if s[key] is not None]
        return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else None

    out = {"n_phases": len(used), "n_fix": n_fix, "min_seg_fixes": min_seg_fixes}
    for name, meas_lists, rep_key in (("pos_hor", hor_pos, "rep_pos_hor"),
                                      ("pos_ver", ver_pos, "rep_pos_ver"),
                                      ("vel_hor", hor_vel, "rep_vel_hor"),
                                      ("vel_ver", ver_vel, "rep_vel_ver")):
        meas, dof = _pooled_within_phase_std(meas_lists)
        rep = rep_rms(rep_key)
        out[name] = {"meas": meas, "rep": rep, "dof": dof,
                     "ratio": (meas / rep if meas is not None and rep else None)}
    return out


def print_gnss_standstill_accuracy(phases):
    """Print the gnss_standstill_accuracy() result as a data-quality line,
    or note that there weren't enough standstill fixes."""
    acc = gnss_standstill_accuracy(phases)
    if acc is None:
        n_fix = sum(len(ph) for ph in phases)
        print(f"  gnss accuracy check: not enough standstill fixes "
              f"({n_fix} seen) - skipped (needs the platform to stand still)")
        return
    print(f"  gnss accuracy check (measured spread during {acc['n_phases']} "
          f"standstill phase(s), {acc['n_fix']} fixes, vs receiver-reported "
          f"1-σ):")
    for label, key, unit in (("horizontal pos", "pos_hor", "m"),
                             ("vertical pos  ", "pos_ver", "m"),
                             ("horizontal vel", "vel_hor", "m/s"),
                             ("vertical vel  ", "vel_ver", "m/s")):
        d = acc[key]
        if d["meas"] is None or d["rep"] is None:
            print(f"    {label}: n/a (no usable samples)")
            continue
        r = d["ratio"]
        verdict = ("optimistic" if r > 1.3 else
                   "pessimistic" if r < 0.77 else "consistent")
        print(f"    {label}: measured {d['meas']:8.3f} {unit:3s} vs reported "
              f"{d['rep']:8.3f} {unit:3s} -> {verdict} (x{r:.2f})")
    print("    (measured > reported => GNSS covariance too optimistic, "
          "< => too pessimistic)")


def gnss_vel_accuracy_stats(fixes):
    """Median receiver-reported 1-σ GNSS velocity accuracy (horizontal and
    vertical) over every fix with usable velocity (cov_vel not None),
    regardless of whether the filter actually fused it (unlike
    gnss_standstill_accuracy, this needs no standstill phase - it reads
    the receiver's own claimed accuracy straight off gnss.csv). A
    consistently large reported stddev means weak GNSS velocity aiding for
    the whole trial (a receiver/environment property - few satellites,
    poor DOP, no RTK fix - not a filter tuning issue): less velocity-error
    correction, and in automotive_mode weaker yaw observability from the
    GNSS course over ground. Returns None if no fix has usable velocity."""
    hor = []
    ver = []
    for fx in fixes:
        cov_v = fx.get("cov_vel")
        if not fx.get("vel_ok") or cov_v is None:
            continue
        hor.append(math.sqrt(0.5 * (cov_v[0][0] + cov_v[1][1])))
        ver.append(math.sqrt(cov_v[2][2]))
    if not hor:
        return None
    hor.sort()
    ver.sort()
    n = len(hor)
    return {"hor_mps": hor[n // 2], "ver_mps": ver[n // 2], "n": n}


def channel_standstill_accuracy(phases, reported_stddev, min_seg_samples=4):
    """Same standstill-spread idea as gnss_standstill_accuracy(), but for a
    single sensor channel whose configured accuracy is one scalar 1-σ:
    the barometer (altitude, one axis) or the magnetometer (field, three
    axes pooled like GNSS pools N and E).

    During a true standstill the quantity is constant - the baro altitude
    (zero vertical velocity) and, with zero rotation, the body-frame mag
    field - so the within-phase spread IS the sensor's measurement noise.
    Comparing it to the configured stddev tells you whether that stddev is
    realistic: measured > reported => the model is too optimistic (the
    filter over-trusts the sensor); measured < reported => too pessimistic
    (accuracy discarded).

    `phases` is a list of standstill phases; each phase is a list of
    per-sample tuples (one value per axis: baro (alt,), mag (x, y, z)). The
    per-phase per-axis mean is removed and the residuals pooled across all
    phases and axes. Only phases with at least `min_seg_samples` samples are
    used. Returns None if there aren't enough standstill samples to tell."""
    used = [ph for ph in phases if len(ph) >= min_seg_samples]
    if not used:
        return None
    n_samples = sum(len(ph) for ph in used)
    n_axes = len(used[0][0])
    axis_lists = [[s[ax] for s in ph] for ph in used for ax in range(n_axes)]
    meas, dof = _pooled_within_phase_std(axis_lists)
    rep = reported_stddev if reported_stddev and reported_stddev > 0 else None
    return {"n_phases": len(used), "n_samples": n_samples, "dof": dof,
            "meas": meas, "rep": rep,
            "ratio": (meas / rep if meas is not None and rep else None)}


def print_channel_standstill_accuracy(label, unit, phases, reported_stddev):
    """Print channel_standstill_accuracy() as a data-quality line (baro/mag),
    or note that there weren't enough standstill samples."""
    acc = channel_standstill_accuracy(phases, reported_stddev)
    if acc is None:
        n = sum(len(ph) for ph in phases)
        print(f"  {label} accuracy check: not enough standstill samples "
              f"({n} seen) - skipped (needs the platform to stand still)")
        return
    d = acc
    if d["meas"] is None or d["rep"] is None:
        print(f"  {label} accuracy check: n/a "
              f"({'no configured stddev' if d['rep'] is None else 'no usable samples'})")
        return
    r = d["ratio"]
    verdict = ("optimistic" if r > 1.3 else
               "pessimistic" if r < 0.77 else "consistent")
    print(f"  {label} accuracy check (measured spread during {d['n_phases']} "
          f"standstill phase(s), {d['n_samples']} samples, vs configured "
          f"1-σ):")
    print(f"    measured {d['meas']:8.3f} {unit:3s} vs configured "
          f"{d['rep']:8.3f} {unit:3s} -> {verdict} (x{r:.2f})")
    print(f"    (measured > configured => {label} stddev too optimistic, "
          "< => too pessimistic)")


def noise_model_metrics(gyr_d2_stat, acc_d2_stat, imu_hz, noise):
    """Measured (maneuver-robust, whole-trial second-difference - see
    _imu_prepass) IMU noise floor vs. the configured gyr_psd/acc_psd, plus
    the psd each would suggest. The pure computation behind main()'s printed
    noise-model section and insdoctor_findings()'s "noise" input, factored
    out so inspostgui.py can feed the same check without duplicating it.
    None if imu_hz <= 0 (rate unknown)."""
    if imu_hz <= 0:
        return None
    dt_nominal = 1.0 / imu_hz

    def meas_floor(d2_stat):
        return (math.sqrt(sum(s.std() ** 2 for s in d2_stat) / 3.0) / math.sqrt(6.0)
                if d2_stat[0].n > 1 else None)

    def suggest_psd(d2_stat):
        if d2_stat[0].n < 2:
            return None
        var = sum(s.std() ** 2 for s in d2_stat) / 3.0 / 6.0
        return var * dt_nominal

    gyr_expect = math.sqrt(noise["gyr_psd"] / dt_nominal)
    acc_expect = math.sqrt(noise["acc_psd"] / dt_nominal)
    gyr_meas, acc_meas = meas_floor(gyr_d2_stat), meas_floor(acc_d2_stat)
    return {
        "gyr_ratio": (gyr_meas / gyr_expect if gyr_meas and gyr_expect > 0 else None),
        "acc_ratio": (acc_meas / acc_expect if acc_meas and acc_expect > 0 else None),
        "gyr_psd_sugg": suggest_psd(gyr_d2_stat),
        "acc_psd_sugg": suggest_psd(acc_d2_stat),
    }


# --- "Dr. INS" diagnostic findings ----------------------------------------
# Turn the raw data-quality metrics already computed for the summary into a
# prioritized, human-readable health check: it names the actual problems
# (GNSS outages, irregular IMU sampling, a noise model that doesn't match the
# sensor, an over-optimistic GNSS covariance, covariance collapse, ...) and
# what to do about them, colored by severity so the important ones stand out.
# Filter self-consistency limits: the largest the filter's OWN reported
# 1-σ may plausibly get during healthy, steady operation. Past these,
# the filter is effectively telling you it doesn't know where/how it is
# pointing - something is wrong (bad leveling, a noise model that lets the
# state run away, no yaw observability, ...). Evaluated only past the warmup
# and while GNSS is streaming (see main()'s sampler), so the expected growth
# during initial convergence or a GNSS outage is NOT flagged.
STDDEV_LIMITS = {"roll": 3.0, "pitch": 3.0, "yaw": 10.0, "pos": 20.0}  # deg/deg/deg/m

# How far a measured quantity may sit from its configured/reported model
# before the DIGEST calls it a finding. Deliberately far looser than the
# "consistent / optimistic / pessimistic" verdicts printed next to the
# numbers in the detail tables above: those describe the measurement, this
# decides whether to interrupt the reader. A noise model within an order of
# magnitude is a normal, working setup - a datasheet figure, a generic MEMS
# default, an Allan fit at a different temperature all land there - and
# flagging it buries the findings that matter. The suggested values are
# printed unconditionally either way, so nothing is lost by staying quiet.
NOISE_MODEL_WARN_FACTOR = 100.0  # IMU psd: measured vs. configured
GNSS_COV_WARN_FACTOR    = 10.0   # standstill spread vs. receiver-reported 1-sigma

# A receiver whose Doppler velocity is genuinely too weak to aid with.
# Consumer receivers report well under this; past it, course-over-ground
# yaw and velocity-error correction stop being worth much.
GNSS_WEAK_HVEL_STDDEV_MPS = 0.5

# How far the local/GNSS offset filter may wander over a run before the
# barometric height source is called out. Weather alone moves an ISA altitude
# by a few metres per hour; a cabin-mounted sensor in a vehicle moves tens of
# metres in minutes, and that whole excursion lands in the position solution
# because the GNSS vertical is not fused while the barometric source holds.
BARO_HEIGHT_OFFSET_SPAN_WARN_M = 10.0

SEV_OK, SEV_INFO, SEV_WARN, SEV_CRIT = 0, 1, 2, 3
_SEV_TAG = {SEV_CRIT: "CRIT", SEV_WARN: "WARN", SEV_INFO: "INFO", SEV_OK: " OK "}
_SEV_COLOR = {SEV_CRIT: "\033[1;31m", SEV_WARN: "\033[1;33m",
              SEV_INFO: "\033[36m", SEV_OK: "\033[32m"}
_RESET = "\033[0m"


def _span(series):
    """Peak-to-peak of a series, ignoring NaN. None if it never had two
    finite samples."""
    finite = [v for v in series if v == v]
    if len(finite) < 2:
        return None
    return max(finite) - min(finite)


def insdoctor_findings(m):
    """Build the list of (severity, text) findings from the metrics dict
    `m` assembled in main(). Pure/inspectable - all thresholds live here,
    no I/O. Each finding is one issue (or an explicit "nominal" all-clear
    so the user sees the check ran)."""
    f = []

    # --- IMU stream: dropouts and rate stability ---------------------------
    imu_hz = m["imu_hz"]
    gap, gap_at = m["imu_max_gap"], m["imu_max_gap_at"]
    # gap_at is the elapsed time (since the FIRST imu sample) at which the
    # gap ends; gap_start is when it began. A gap starting right at the
    # very beginning of the file (a handful of samples, then silence) isn't
    # a mid-recording dropout - it means the real data only starts at
    # gap_at (e.g. a logger/receiver enumeration delay before streaming
    # settles), so word it as a late start instead of a "dropout" that
    # implies otherwise-good data got interrupted.
    gap_start = gap_at - gap
    late_start = gap_start < 2.0
    if gap >= 0.5:  # exceeds ins's default max_prediction_time_sec
        if late_start:
            f.append((SEV_WARN,
                      f"IMU stream starts late: only {gap_start * 1000:.0f} ms of data "
                      f"before a {gap * 1000:.0f} ms gap - real data begins at "
                      f"t={gap_at:.0f} s, not a mid-recording dropout"))
        else:
            f.append((SEV_WARN, f"IMU dropout: {gap * 1000:.0f} ms gap at t={gap_at:.0f} s "
                                "(> max_prediction_time 0.5 s - the filter stops "
                                "predicting across it)"))
    elif gap >= 0.1:
        f.append((SEV_INFO, f"IMU minor dropout: {gap * 1000:.0f} ms gap at t={gap_at:.0f} s"))
    else:
        f.append((SEV_OK, f"IMU stream continuous ({imu_hz:.0f} Hz, worst gap {gap * 1000:.0f} ms)"))

    rates = m["imu_rate_hz"]
    core = rates[:-1] if len(rates) >= 3 else rates  # drop partial last bucket
    if core:
        lo, hi = min(core), max(core)
        med = sorted(core)[len(core) // 2]
        spread = (hi - lo) / med if med > 0 else 0.0
        if spread > 0.25:
            f.append((SEV_WARN, f"IMU rate varies {lo:.1f}..{hi:.1f} Hz (median {med:.1f}) "
                                "- irregular sampling, check the logger clock/timestamps "
                                "(the noise model assumes a fixed rate)"))
        elif spread > 0.05:
            f.append((SEV_INFO, f"IMU rate wobbles {lo:.1f}..{hi:.1f} Hz (median {med:.1f}) "
                                "- mild sampling jitter"))

    # --- GNSS coverage -----------------------------------------------------
    g = m["gnss"]
    if g and g[0] >= 2:
        _, _, ghz, ggap, ggap_at = g
        if ggap >= 10.0:
            f.append((SEV_CRIT, f"GNSS outage: {ggap:.0f} s with no fix at t={ggap_at:.0f} s "
                                "- long dead-reckoning stretch, expect drift there"))
        elif ggap >= 2.0:
            f.append((SEV_WARN, f"GNSS gap: {ggap:.1f} s at t={ggap_at:.0f} s "
                                f"(nominal {ghz:.1f} Hz) - brief coasting on the IMU"))
        else:
            f.append((SEV_OK, f"GNSS coverage good ({ghz:.1f} Hz, worst gap {ggap:.1f} s)"))
    elif m["aiding_mode"] == "gnss":
        f.append((SEV_INFO, "GNSS: fewer than 2 fixes - coverage not assessable"))

    # --- IMU noise model vs. measured floor --------------------------------
    nm = m["noise"]
    if nm:
        for label, ratio, term, sugg in (
                ("gyro", nm["gyr_ratio"], "gyr_psd", nm["gyr_psd_sugg"]),
                ("accel", nm["acc_ratio"], "acc_psd", nm["acc_psd_sugg"])):
            if ratio is None:
                continue
            if ratio > NOISE_MODEL_WARN_FACTOR or ratio < 1.0 / NOISE_MODEL_WARN_FACTOR:
                verb = ("too optimistic (sensor noisier than configured)" if ratio > 1
                        else "too pessimistic (sensor quieter than configured)")
                txt = f"{label} noise model {verb}: measured x{ratio:.2f} of configured"
                if sugg is not None:
                    txt += f" -> try {term}: {sugg:.2e}"
                f.append((SEV_WARN, txt))
            else:
                f.append((SEV_OK, f"{label} noise model consistent (measured x{ratio:.2f} of configured)"))

    # --- GNSS accuracy vs. its own reported covariance (standstill) --------
    ga = m["gnss_acc"]
    if ga is None:
        if m["aiding_mode"] == "gnss":
            f.append((SEV_INFO, "GNSS accuracy vs. reported covariance: not checked "
                                "(no standstill phase in this trial)"))
    else:
        opt, pes = [], []
        for label, key in (("horiz-pos", "pos_hor"), ("vert-pos", "pos_ver"),
                           ("horiz-vel", "vel_hor"), ("vert-vel", "vel_ver")):
            d = ga[key]
            if d["ratio"] is None:
                continue
            if d["ratio"] > GNSS_COV_WARN_FACTOR:
                opt.append(f"{label} x{d['ratio']:.1f}")
            elif d["ratio"] < 1.0 / GNSS_COV_WARN_FACTOR:
                pes.append(f"{label} x{d['ratio']:.2f}")
        if opt:
            f.append((SEV_WARN, "GNSS covariance too OPTIMISTIC in standstill: "
                                f"{', '.join(opt)} (receiver claims better than measured "
                                "-> the filter over-trusts it, scale it up via GNSS "
                                "pos/vel_cov_scale or a stddev floor)"))
        if pes:
            f.append((SEV_INFO, "GNSS covariance conservative (pessimistic) in standstill: "
                                f"{', '.join(pes)} (safe, but you discard accuracy the "
                                "receiver actually has)"))
        if not opt and not pes:
            f.append((SEV_OK, "GNSS accuracy consistent with its reported covariance (standstill check)"))

    # --- GNSS reported velocity accuracy (receiver quality, no standstill --
    # needed, unlike the check above) ---------------------------------------
    gv = m["gnss_vel"]
    if gv is not None:
        # The configured fusion gate, when the config actually sets one. A 0
        # there means "use the library default", NOT "reject everything": the
        # raw config value is unusable as a threshold, and comparing a median
        # against it made this fire on every receiver, however good. The
        # default it resolves to is the shutdown gate (tens of m/s), which is
        # about when to stop fusing, not about aiding quality - so quality is
        # judged against its own threshold and the configured gate only
        # tightens it.
        cfg_lim = m["gnss_max_hor_vel_stddev_mps"]
        lim = min(GNSS_WEAK_HVEL_STDDEV_MPS, cfg_lim) if cfg_lim > 0.0             else GNSS_WEAK_HVEL_STDDEV_MPS
        if gv["hor_mps"] > lim:
            f.append((SEV_WARN,
                      f"GNSS velocity aiding weak: median reported horizontal "
                      f"velocity stddev {gv['hor_mps']:.2f} m/s (> {lim:.2f} m/s) "
                      f"over {gv['n']} fixes - poor GNSS velocity support "
                      "(weaker yaw observability from course-over-ground in "
                      "automotive_mode, less velocity-error correction), "
                      "check antenna sky view / satellite count / RTK status"))
        else:
            f.append((SEV_OK, "GNSS velocity accuracy nominal (median reported "
                              f"horizontal stddev {gv['hor_mps']:.2f} m/s, limit "
                              f"{lim:.2f} m/s, over {gv['n']} fixes)"))
    elif m["aiding_mode"] == "gnss":
        f.append((SEV_INFO, "GNSS velocity accuracy: not checked (no fix "
                            "reported usable velocity)"))

    # --- Baro / mag stddev vs. its measured standstill spread --------------
    # Same idea as the GNSS check, but for the configured baro/mag 1-σ
    # (channel_standstill_accuracy). Only reported when a standstill actually
    # occurred (else the metric is None and nothing is said).
    for chan, unit, cfg_key in (("baro", "m", "baro stddev_m"),
                                ("mag", "uT", "mag stddev_ut")):
        ca = m.get(chan + "_acc")
        if not ca or ca["ratio"] is None:
            continue
        r = ca["ratio"]
        if r > 1.3:
            f.append((SEV_WARN, f"{chan} stddev too OPTIMISTIC in standstill "
                                f"(measured x{r:.1f} the configured spread) - the "
                                f"filter over-trusts the {chan}; raise {cfg_key}"))
        elif r < 0.77:
            f.append((SEV_INFO, f"{chan} stddev conservative (pessimistic) in "
                                f"standstill (measured x{r:.2f}) - safe, but you "
                                f"discard accuracy the {chan} actually has"))
        else:
            f.append((SEV_OK, f"{chan} accuracy consistent with its configured "
                              "stddev (standstill check)"))

    # --- Filter self-reported uncertainty stayed sane? ---------------------
    # Sampled only after a fused GNSS fix, past warmup, with GNSS streaming
    # (see main()) - so the growth during warmup/GNSS gaps is excluded.
    hp = m["health"]
    if hp and hp["n"] > 0:
        lim, st = hp["limits"], hp["stats"]
        rp_exc = st["roll"]["n_exceed"] + st["pitch"]["n_exceed"]
        if rp_exc > 0:
            worse = "roll" if st["roll"]["worst"] >= st["pitch"]["worst"] else "pitch"
            w = st[worse]
            f.append((SEV_WARN, "Filter roll/pitch uncertainty too high in steady operation: "
                                f"{worse} stddev reached {w['worst']:.1f} deg (limit "
                                f"{lim['roll']:.0f}) at t={w['t']:.0f} s, {rp_exc} sample(s) - past "
                                "warmup & outside GNSS gaps, so the filter isn't leveling "
                                "(bad IMU noise model / vibration / mounting?)"))
        for key, label, unit in (("yaw", "yaw", "deg"), ("pos", "position", "m")):
            s = st[key]
            if s["n_exceed"] > 0:
                extra = (" - no yaw observability? (no mag, little maneuvering, "
                         "automotive_mode off?)" if key == "yaw" else
                         " - the filter has effectively lost its position fix")
                f.append((SEV_WARN, f"Filter {label} uncertainty too high in steady operation: "
                                    f"stddev reached {s['worst']:.1f} {unit} (limit "
                                    f"{lim[key]:.0f} {unit}) at t={s['t']:.0f} s, "
                                    f"{s['n_exceed']} sample(s){extra}"))
        if rp_exc == 0 and st["yaw"]["n_exceed"] == 0 and st["pos"]["n_exceed"] == 0:
            f.append((SEV_OK, "Filter-reported uncertainty stayed within bounds "
                              f"(roll/pitch<{lim['roll']:.0f} deg, yaw<{lim['yaw']:.0f} deg, "
                              f"pos<{lim['pos']:.0f} m) over {hp['n']} post-warmup fixes"))

    # --- Filter-health flags from the run ----------------------------------
    oc = m["overconfidence"]
    tripped = [who for who, sub in (("INSLIB", oc), ("ARS", oc["ars"]),
                                   ("AHRS", oc["ahrs"])) if sub["tripped"]]
    if tripped:
        f.append((SEV_CRIT, f"Covariance collapse: {', '.join(tripped)} reported an "
                            "implausibly small stddev - likely divergence while looking "
                            "confident"))

    diag = m["diag"]
    if diag["n_fuse_fail"] > 0:
        f.append((SEV_WARN, f"{diag['n_fuse_fail']} Kalman fusion failure(s) - numerical "
                            "trouble in the update step"))
    seen, used = diag["n_gnss_seen"], diag["n_gnss_used"]
    if seen > 0:
        rej = seen - used
        if rej > 0 and rej / seen > 0.2:
            f.append((SEV_WARN, f"{rej}/{seen} GNSS fixes ({100.0 * rej / seen:.0f}%) not used "
                                "- gated out by the gnss_max_*_stddev limits, check them "
                                "against the data"))
        # diag.n_downweighted counts EVERY ins_fuse() call whose chi2 test
        # tripped - GNSS position/velocity, but also the barometric height
        # (REQ-NAV-054), the magnetometer and the zero-velocity updates. Scale
        # it by the GNSS fusion count for a rate, and say plainly that the
        # numerator is not GNSS-only, so a barometer fighting the filter is
        # not read as street canyon multipath.
        # --- scoring point vs. fusion point ---------------------------------
        # ins reports the IMU's position; a fix is taken at the ANTENNA, which
        # is why gnss.leverarm_frd exists. score.leverarm_frd is the same
        # correction on the way back out, applied before the estimate is
        # compared against ref.csv. Left at zero while the fusion arm is not,
        # the two sides sit on different points of the vehicle and the whole
        # lever arm is charged to the filter as position error - a floor under
        # every reported number, and on a good drive an order of magnitude
        # above what the filter itself contributes.
        la = m.get("leverarm")
        if la:
            fuse_arm = max(abs(v) for v in la["gnss"])
            score_arm = max(abs(v) for v in la["score"])
            if fuse_arm > 0.0 and score_arm == 0.0:
                f.append((SEV_WARN,
                          f"gnss.leverarm_frd is set ({la['gnss']}) but score.leverarm_frd "
                          "is not - the position error is scored between the IMU and the "
                          "reference point, so it carries the whole lever arm. Set "
                          "score.leverarm_frd to the same value when ref.csv is the "
                          "receiver's own solution"))

        # --- barometric height source vs. the GNSS vertical it replaced ----
        # ins latches the height source once, at bootstrap (REQ-NAV-053): a
        # barometer streaming at that moment wins, and the GNSS vertical is
        # dropped from the position fusion for the rest of the run. That is
        # the right call for a platform whose GNSS height is the weak axis,
        # and the wrong one for a barometer sitting in a car cabin, where
        # ventilation and speed move the static pressure by tens of metres.
        # The offset filter (local_gnss_alt) measures exactly that: it tracks
        # baro-vs-ellipsoid, so its excursion over the run IS the height error
        # the position solution carries.
        bh = m.get("baro_height")
        if bh and bh["active"] and bh["offset_span_m"] is not None:
            span = bh["offset_span_m"]
            if span >= BARO_HEIGHT_OFFSET_SPAN_WARN_M:
                f.append((SEV_WARN,
                          f"barometric height source drifted {span:.0f} m against the GNSS "
                          "ellipsoid over the run - ins is not using the GNSS vertical at "
                          "all (REQ-NAV-053, latched at bootstrap because a barometer was "
                          "streaming). Set baro_height_disable: 1 if the GNSS vertical is "
                          "the better absolute reference here"))
            else:
                f.append((SEV_OK,
                          f"barometric height source tracks the GNSS ellipsoid "
                          f"(offset span {span:.1f} m over the run)"))

        dwt = diag["n_downweighted"]
        if used > 0 and dwt / used > 0.15:
            f.append((SEV_INFO, f"{dwt} chi2-downweighted ins fusions "
                                f"({100.0 * dwt / used:.0f}% of the GNSS fusion count; the "
                                "counter also covers baro height, mag and ZUPT) - "
                                "outlier-heavy input, check the outlier page for which "
                                "sub-filter it is"))

    return f


def insdoctor_lines(findings, use_color=False):
    """Format the insdoctor_findings() list as plain text lines (most-severe
    first, header with severity counts), the shared rendering behind both
    print_insdoctor() (console) and inspostgui.py's GUI summary text."""
    n_crit = sum(1 for s, _ in findings if s == SEV_CRIT)
    n_warn = sum(1 for s, _ in findings if s == SEV_WARN)
    n_info = sum(1 for s, _ in findings if s == SEV_INFO)
    counts = ", ".join(p for p in (
        f"{n_crit} critical" if n_crit else "",
        f"{n_warn} warning" + ("s" if n_warn != 1 else "") if n_warn else "",
        f"{n_info} info" if n_info else "") if p) or "all nominal"
    lines = [f"Findings ({counts}):"]
    for sev, text in sorted(findings, key=lambda x: -x[0]):
        tag = _SEV_TAG[sev]
        label = f"{_SEV_COLOR[sev]}[{tag}]{_RESET}" if use_color else f"[{tag}]"
        lines.append(f"  {label} {text}")
    return lines


def print_insdoctor(findings, use_color=None):
    """Print the insdoctor_findings() list, most-severe first, colored by
    severity on a TTY (auto-off when piped/redirected)."""
    if use_color is None:
        use_color = sys.stdout.isatty()
    print()
    for line in insdoctor_lines(findings, use_color):
        print(line)


def _progress_reporter(total, label="imu"):
    """Returns (update, done) for a lightweight progress indicator over
    the main replay loop, which otherwise just hangs silently on the
    console for large datasets. On a TTY: a live \\r-overwritten bar +
    percentage + ETA, throttled to ~10 Hz wall-clock so it doesn't slow
    the replay down. Redirected to a file (not a TTY): a plain line
    every 10% instead - no carriage-return spam in a log, but a batch
    run still shows some advancement over time."""
    is_tty = sys.stdout.isatty()
    start = time.perf_counter()
    last_print = [0.0]
    last_decile = [-1]

    def update(i):
        now = time.perf_counter()
        if now - last_print[0] < 0.1:
            return
        last_print[0] = now
        pct = 100.0 * i / total if total else 100.0
        elapsed = now - start
        if is_tty:
            bar_len = 30
            filled = int(bar_len * pct / 100.0)
            bar = "#" * filled + "-" * (bar_len - filled)
            eta = elapsed / i * (total - i) if i > 0 else 0.0
            sys.stdout.write(f"\r[{bar}] {pct:5.1f}%  {i}/{total} {label}  "
                            f"elapsed {elapsed:5.0f}s  eta {eta:5.0f}s")
            sys.stdout.flush()
        else:
            decile = int(pct // 10)
            if decile != last_decile[0] and 0 < decile <= 10:
                last_decile[0] = decile
                print(f"  {decile * 10:3d}%  ({i}/{total} {label}, "
                     f"{elapsed:.0f}s elapsed)")

    def done():
        if is_tty:
            sys.stdout.write("\r" + " " * 90 + "\r")
            sys.stdout.flush()

    return update, done


def _vec_nonzero(v):
    return any(abs(float(x)) > 0.0 for x in v)


def print_chi2_gate(spec):
    """Print the active chi2 outlier gate as a confidence level (1-alpha) at
    startup, so it's obvious at a glance whether outliers are gated at ~95%
    or ~99%. Each ins reference is fused scalar-row-wise, so the gate is
    1-DOF and the confidence is P(X <= thr) = erf(sqrt(thr/2)). The default
    thresholds mirror src/ins.c INS_CHI2_GNSS/LOCAL_POS (10) and
    INS_CHI2_MAG/YAW (9); a positive chi2_reject_alpha overrides them with
    one shared chi2inv(1-alpha, 1) gate."""
    if spec["chi2_disable"]:
        print("chi2 outlier gate: DISABLED (chi2_disable=1)")
        return
    alpha = float(spec["chi2_reject_alpha"])
    if alpha > 0.0:
        conf = 1.0 - alpha
        print(f"chi2 outlier gate: shared chi2inv(1-alpha, 1), "
              f"1-alpha = {100.0 * conf:.2f}% (alpha {100.0 * alpha:.2f}%)")
        return
    parts = []
    for label, thr in (("GNSS/local-pos", 10.0), ("mag/yaw", 9.0)):
        conf = math.erf(math.sqrt(thr / 2.0))
        parts.append(f"{label} thr {thr:.1f} -> {100.0 * conf:.2f}% "
                     f"(alpha {100.0 * (1.0 - conf):.2f}%)")
    print("chi2 outlier gate (per-channel defaults, 1 DOF): " + "; ".join(parts))


def notable_settings(spec):
    """Human-readable list of settings that materially change filter
    behaviour and are easy to overlook - surfaced at startup so a run is
    never silently using e.g. a downweighted GNSS height, a fixed sensor
    bias, or disabled outlier gating. Only non-default (non-no-op) settings
    appear; an all-defaults run reports none."""
    notes = []
    g = spec["gnss"]
    gd = DEFAULTS["gnss"]

    if spec["automotive_mode"]:
        thr = []
        if spec["automotive_min_speed_mps"]:
            thr.append(f"min speed {spec['automotive_min_speed_mps']:g} m/s")
        if spec["automotive_min_yaw_stddev_deg"]:
            thr.append(f"min yaw stddev {spec['automotive_min_yaw_stddev_deg']:g} deg")
        detail = f" ({', '.join(thr)})" if thr else ""
        notes.append(
            "automotive mode ON - yaw from GNSS course-over-ground" + detail)
    if spec["chi2_disable"]:
        notes.append("chi2 outlier downweighting DISABLED (no robust gating)")

    if g["pos_cov_scale"]:
        notes.append(f"GNSS pos covariance scaled x{g['pos_cov_scale']:g} "
                     "(pos_cov_scale)")
    if g["pos_cov_scale_height"]:
        notes.append(
            f"GNSS height downweighted x{g['pos_cov_scale_height']:g} "
            "(pos_cov_scale_height)")
    if g["vel_cov_scale"]:
        notes.append(f"GNSS vel covariance scaled x{g['vel_cov_scale']:g} "
                     "(vel_cov_scale)")

    floors = []
    if g["pos_stddev_floor_hor_m"]:
        floors.append(f"pos hor {g['pos_stddev_floor_hor_m']:g} m")
    if g["pos_stddev_floor_ver_m"]:
        floors.append(f"pos ver {g['pos_stddev_floor_ver_m']:g} m")
    if g["vel_stddev_floor_hor_mps"]:
        floors.append(f"vel hor {g['vel_stddev_floor_hor_mps']:g} m/s")
    if g["vel_stddev_floor_ver_mps"]:
        floors.append(f"vel ver {g['vel_stddev_floor_ver_mps']:g} m/s")
    if floors:
        notes.append("GNSS min stddev floor: " + ", ".join(floors))

    if _vec_nonzero(g["pos_stddev_fallback_m"]) or g["vel_stddev_fallback_mps"]:
        fb = g["pos_stddev_fallback_m"]
        notes.append(
            "GNSS fallback stddev (used only when reported cov is 0): "
            f"pos [{fb[0]:g}, {fb[1]:g}] m, "
            f"vel {g['vel_stddev_fallback_mps']:g} m/s")

    gates = []
    for key, unit in (("max_horizontal_pos_stddev_m", "m"),
                      ("max_vertical_pos_stddev_m", "m"),
                      ("max_horizontal_vel_stddev_mps", "m/s"),
                      ("max_vertical_vel_stddev_mps", "m/s")):
        if g[key] != gd[key]:
            gates.append(f"{key}={g[key]:g} {unit} (default {gd[key]:g})")
    if gates:
        notes.append("GNSS aiding-quality gate changed: " + ", ".join(gates))

    mode_gates = []
    for key, unit in (("start_max_horizontal_pos_stddev_m", "m"),
                      ("start_max_vertical_pos_stddev_m", "m"),
                      ("start_max_horizontal_vel_stddev_mps", "m/s"),
                      ("start_max_vertical_vel_stddev_mps", "m/s"),
                      ("stop_max_horizontal_pos_stddev_m", "m"),
                      ("stop_max_vertical_pos_stddev_m", "m"),
                      ("stop_max_horizontal_vel_stddev_mps", "m/s"),
                      ("stop_max_vertical_vel_stddev_mps", "m/s"),
                      ("init_dwell_sec", "s"),
                      ("stop_dwell_sec", "s")):
        if g[key] != gd[key]:
            mode_gates.append(f"{key}={g[key]:g} {unit}")
    if g["init_dwell_disable"]:
        mode_gates.append("init_dwell_disable=1 (enter on the first good fix)")
    if g["stop_disable"]:
        mode_gates.append("stop_disable=1 (never leave 3D on GNSS quality)")
    if mode_gates:
        notes.append("GNSS 3D entry/exit gate changed: " + ", ".join(mode_gates))

    imu = spec["imu"]
    if _vec_nonzero(imu["gyr_fixed_bias"]) or _vec_nonzero(imu["acc_fixed_bias"]):
        gb, ab = imu["gyr_fixed_bias"], imu["acc_fixed_bias"]
        notes.append(
            "IMU fixed bias removed: gyro "
            f"{[round(math.degrees(x), 4) for x in gb]} deg/s, accel "
            f"{[round(x, 4) for x in ab]} m/s^2")
    if _vec_nonzero(imu["gyr_misalignment"]) or _vec_nonzero(imu["acc_misalignment"]):
        notes.append("IMU misalignment/scale calibration active")

    mag = spec["mag"]
    if mag["estimate_bias"]:
        notes.append("magnetometer 18-state hard-iron bias estimation ON ")
    if mag["enable"] and (_vec_nonzero(mag["fixed_bias"])
                          or _vec_nonzero(mag["misalignment"])):
        notes.append("magnetometer fixed calibration active")

    if spec["init"] == "ref":
        notes.append("initial state from reference truth (init: ref)")
    overrides = [k for k, v in spec["init_stddev"].items() if v]
    if overrides:
        notes.append("initial-state stddev override(s): " + ", ".join(overrides))

    return notes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset",
                    help="dataset directory (with config.yaml) or a "
                         "config YAML path")
    ap.add_argument("--realtime", action="store_true",
                    help="pace the replay to wall-clock time")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="realtime speed multiplier (with --realtime)")
    ap.add_argument("--mavlink", action="store_true",
                    help="also forward MAVLink (needs pymavlink)")
    ap.add_argument("--plotjuggler", action="store_true",
                    help="stream live telemetry as PlotJuggler JSON UDP "
                         "(off by default; --pj-port sets the port)")
    # Deprecated no-op: PlotJuggler is now opt-in (off by default), so
    # --no-plotjuggler is redundant. Kept hidden so old invocations don't
    # error out.
    ap.add_argument("--no-plotjuggler", action="store_true",
                    help=argparse.SUPPRESS)
    ap.add_argument("--pj-port", type=int, default=9870)
    ap.add_argument("--mav-port", type=int, default=14550)
    ap.add_argument("--flight-log", default=None,
                    help="write an NDJSON flight log to this path")
    ap.add_argument("--telemetry-hz", type=float, default=50.0,
                    help="max telemetry publish rate (in sim time)")
    ap.add_argument("--plot", action="store_true",
                    help="matplotlib state history (1-σ band) + error "
                         "plots at the end (needs matplotlib)")
    ap.add_argument("--plot-out", default=None,
                    help="save the --plot figures as pages of one "
                         "multi-page PDF at this path instead of showing "
                         "them interactively (implies --plot)")
    ap.add_argument("--plot-hz", type=float, default=5.0,
                    help="sampling rate for the --plot recorder (in sim "
                         "time; independent of --telemetry-hz)")
    ap.add_argument("--plot-track-hz", type=float, default=50.0,
                    help="sampling rate for the --plot North-East map "
                         "track (in sim time). Deliberately higher than "
                         "--plot-hz: a track spanning a couple of metres "
                         "shows every missing sample as a chord across "
                         "the real path, and the map page thins its "
                         "vertices geometrically anyway, so the extra "
                         "samples only cost page size where the track "
                         "actually curves. 0 -> reuse the --plot-hz "
                         "samples for the map too")
    ap.add_argument("--plot-rate-bucket-sec", type=float, default=5.0,
                    help="bucket width for the --plot sensor sampling-rate "
                         "page")
    ap.add_argument("--kml", default=None,
                    help="write a Google Earth KML file to this path: the "
                         "estimate as a time-animated gx:Track (drag the "
                         "time slider) plus static ground-track lines for "
                         "the estimate and the ground truth")
    ap.add_argument("--kml-hz", type=float, default=2.0,
                    help="sampling rate for the --kml/--map-frames track "
                         "(in sim time); raise it for smoother --map-frames "
                         "motion at a high --map-fps")
    ap.add_argument("--map-frames", default=None,
                    help="write a portrait PNG frame sequence to this "
                         "directory: raw GNSS fix (red) and INSLIB estimate "
                         "(cyan) revealed over time on an OpenStreetMap "
                         "background, with real tunnel geometry (Overpass "
                         "API) shaded in -- see ins_map_frames.py. Prints "
                         "the ffmpeg command to stitch the frames into a "
                         "video")
    ap.add_argument("--map-fps", type=float, default=20.0,
                    help="frame rate for --map-frames")
    ap.add_argument("--map-t-start", type=float, default=None,
                    help="--map-frames: seconds into the replay to start "
                         "at (default: the start of the recorded track)")
    ap.add_argument("--map-t-end", type=float, default=None,
                    help="--map-frames: seconds into the replay to end at "
                         "(default: the end of the recorded track)")
    ap.add_argument("--estimate-gnss-delay", action="store_true",
                    help="force the GNSS-delay estimate on "
                         "even outside the auto-detected case (aiding: "
                         "gnss with usable velocity + a barometer, which "
                         "already runs this automatically). Uses the "
                         "--plot-hz recorder (--plot not required). CAVEAT: "
                         "the result is the delay of GNSS RELATIVE TO "
                         "baro_alt, which has its own (not necessarily "
                         "zero) group delay -- see the module docstring")
    ap.add_argument("--eval-start", type=float, default=None, metavar="SEC",
                    help="restrict the ground-truth evaluation to epochs at "
                         "or after this trial time (seconds from the first IMU "
                         "sample, the same origin as the printed t=... times). "
                         "The filter still runs over the WHOLE trial (so it "
                         "stays converged) -- only the scored error stats and "
                         "the summary JSON are limited to the window. Useful to "
                         "exclude a bad startup segment or focus on one leg.")
    ap.add_argument("--eval-end", type=float, default=None, metavar="SEC",
                    help="upper bound of the --eval-start window (trial "
                         "seconds). Combine with --eval-start for a closed "
                         "interval; either bound may be given alone.")
    ap.add_argument("--dump-errors", default=None, metavar="FILE",
                    help="write per-epoch position error to CSV: t_s,"
                         " horizontal_m, down_m, north_m, east_m. Separates the"
                         " horizontal solution from a baro-driven height, which"
                         " the scored 3D magnitude mixes together.")
    ap.add_argument("--dump-solution", default=None, metavar="FILE",
                    help="write the filter's own solution to CSV in the"
                         " dataset ref format (t_us, lat, lon, h, roll, pitch,"
                         " yaw, v_ned), so it can be replayed, plotted or"
                         " compared against another estimator without rerunning"
                         " this script. Unlike --dump-errors this needs no"
                         " reference at all.")
    ap.add_argument("--dump-solution-hz", type=float, default=25.0,
                    help="rate of --dump-solution (0 -> every IMU epoch)")
    ap.add_argument("--gnss-outage", action="append", default=[],
                    metavar="START:DURATION",
                    help="drop every GNSS fix in this window (seconds, relative"
                         " to the first IMU sample - the same origin"
                         " --eval-start/--eval-end use), simulating a tunnel."
                         " Repeatable."
                         " Combine with --eval-start/--eval-end to score the"
                         " drift inside it rather than diluting it over the"
                         " whole run.")
    ap.add_argument("--summary-json", default=None,
                    help="write the accuracy summary (scored-epoch count, "
                         "ins position RMS and roll/pitch/yaw error "
                         "mean/std vs truth, warmup window) as JSON to this "
                         "path. Machine-readable form of the summary already "
                         "printed at the end - consumed by the simulated-"
                         "dataset regression harness (datasets/"
                         "check_simulated.py), but with no gates "
                         "or pass/fail of its own; this stays a pure analysis "
                         "tool.")
    args = ap.parse_args()
    # Asking for a PDF path obviously means "make the plots": save the user
    # from having to pass --plot as well (a --plot-out without --plot would
    # otherwise silently produce nothing).
    if args.plot_out:
        args.plot = True
    if (args.eval_start is not None and args.eval_end is not None
            and args.eval_end <= args.eval_start):
        ap.error("--eval-end must be greater than --eval-start")

    print(f"ins replay.py - loading {args.dataset} ...")

    spec, data_dir = load_config(args.dataset)
    imu_path = input_path(data_dir, spec, "imu")
    ref_path = input_path(data_dir, spec, "ref")
    if not (os.path.exists(imu_path) and os.path.exists(ref_path)):
        sys.exit(f"dataset missing under {data_dir}")

    ref = load_ref(ref_path)
    if not ref:
        sys.exit(f"no reference epochs in {ref_path}")

    # Move every reference row onto its own time of validity (REQ-VER-030),
    # the same shift tools/replay.c applies at load, so one config.yaml scores
    # the same in both harnesses.
    ref_delay_ms = float(spec["score"].get("ref_delay_ms", 0.0))
    if ref_delay_ms:
        if spec["aiding"] == "ref":
            sys.exit(f"{data_dir}: score: ref_delay_ms cannot be used with "
                     "aiding: ref")
        shift_us = int(ref_delay_ms * 1000.0)
        for r in ref:
            r["t_us"] -= shift_us
        print(f"reference time of validity: {ref_delay_ms:.0f} ms earlier "
              "than its timestamps (score: ref_delay_ms)")

    gnss_cfg = spec["gnss"]
    gnss_delay_ms = int(gnss_cfg.get("delay_ms", 0.0))
    aiding_mode = spec["aiding"]
    var_hor = gnss_cfg["pos_stddev_fallback_m"][0] ** 2
    var_ver = gnss_cfg["pos_stddev_fallback_m"][1] ** 2
    var_vel = gnss_cfg["vel_stddev_fallback_mps"] ** 2
    if aiding_mode == "gnss":
        gnss_path = input_path(data_dir, spec, "gnss")
        fixes = load_gnss(gnss_path) if os.path.exists(gnss_path) else None
        if not fixes:
            sys.exit(f"aiding: gnss but no usable {gnss_path}")
        if args.gnss_outage:
            # Cut the fixes out rather than telling the filter to ignore
            # them: an outage is the ABSENCE of data, and anything the
            # filter would otherwise still learn from a "rejected" fix
            # (that one arrived at all, its covariance, its time) would
            # make the test easier than reality.
            windows = []
            for w in args.gnss_outage:
                try:
                    a_s, d_s = w.split(":")
                    start, dur = float(a_s), float(d_s)
                except ValueError:
                    sys.exit(f"--gnss-outage: expected START:DURATION, got {w!r}")
                if dur <= 0:
                    sys.exit(f"--gnss-outage: duration must be > 0, got {w!r}")
                windows.append((start, start + dur))
            # Same origin as --eval-start/--eval-end, which count from the
            # first IMU sample: anchoring the cut on the first FIX instead
            # would silently offset the two by however long the receiver
            # took to produce one (55.6 s on the first real drive), and the
            # evaluation window would then not be inside the outage it
            # claims to measure.
            base_us = None
            for _t, _g, _a in iter_imu(imu_path):
                base_us = _t
                break
            if base_us is None:
                sys.exit("--gnss-outage: no IMU samples to anchor the window on")
            kept = [fx for fx in fixes
                    if not any(lo <= (fx["t_us"] - base_us) / US_PER_SEC < hi
                               for lo, hi in windows)]
            n_cut = len(fixes) - len(kept)
            fixes = kept
            if not fixes:
                sys.exit("--gnss-outage removed every fix")
            print("simulated GNSS outage: %s -> %d of %d fixes dropped"
                  % (", ".join("%.0f..%.0f s" % w for w in windows),
                     n_cut, n_cut + len(kept)))
        for fx in fixes:
            fx["cov_pos"] = apply_fallback(cov6_to_rows(fx["cov_pos"]),
                                           var_hor, var_ver)
            fx["cov_vel"] = (apply_fallback(cov6_to_rows(fx["cov_vel"]),
                                            var_vel, var_vel)
                             if fx["vel_ok"] else None)
    elif aiding_mode == "ref":
        # Synthesize the fix from the reference. Noise is
        # pos_stddev_fallback_m/vel_stddev_fallback_mps, same fields a real
        # fix's zero/unknown covariance diagonals fall back to: this
        # synthesized fix never has a reported covariance either.
        pos_cov = [[var_hor, 0, 0], [0, var_hor, 0], [0, 0, var_ver]]
        vel_cov = [[var_vel if i == j else 0
                    for j in range(3)] for i in range(3)]
        fixes = [{
            "t_us": r["t_us"], "lat_rad": r["lat_rad"],
            "lon_rad": r["lon_rad"], "h_m": r["h_m"],
            "cov_pos": pos_cov, "vel_ned": r["vel_ned"],
            "cov_vel": vel_cov, "vel_ok": True,
        } for r in ref]
    elif aiding_mode == "none":
        # No absolute position aiding at all: ins never gets a fix, so
        # it stays uninitialized (nav_suite mode NONE/ATTITUDE_ONLY)
        # forever - but the ARS and baro_alt filters don't need one,
        # they bootstrap from the accelerometer/first barometer sample
        # and keep running regardless (see nav_suite.h). Use this to
        # exercise just those two, e.g. when there is no position source
        # at all.
        fixes = []
    else:
        sys.exit(f"{args.dataset}: unknown aiding mode {aiding_mode!r} "
                 f"(expected gnss/ref/none)")

    mag_cfg = spec["mag"]
    mags = []
    if int(mag_cfg["enable"]):
        mags = load_txyz(input_path(data_dir, spec, "mag"))
        if not mags:
            sys.exit(f"mag: enable but no usable mag.csv in {data_dir}")
    # 0 is handed straight to the library, which reads an unset variance as
    # "use my magnetometer default" (src/sensor_defaults.h). The standstill
    # accuracy check below is fed the same 0 on purpose: with nothing
    # configured there is no configured stddev to call optimistic.
    mag_sd = float(mag_cfg["stddev_ut"])
    mag_var = (mag_sd ** 2,) * 3
    # The fixed calibration as the library gets it (mags itself stays raw:
    # nav.mag() is handed the raw sample and ins.c calibrates it, applying it
    # here as well would correct twice). Anything replay derives from the mag
    # stream on its own goes through mag_calibrate() with these.
    mag_misalign = tuple(float(v) for v in mag_cfg["misalignment"])
    mag_bias_cfg = tuple(float(v) for v in mag_cfg["fixed_bias"])
    mag_cal_active = any(mag_misalign) or any(mag_bias_cfg)

    baro_cfg = spec["baro"]
    ahrs_cfg = spec["ahrs"]
    baros = []
    if int(baro_cfg["enable"]):
        baros = load_baro(input_path(data_dir, spec, "baro"))
        if not baros:
            sys.exit(f"baro: enable but no usable baro.csv in {data_dir}")

    speed_cfg = spec["speed"]
    speeds = []
    if int(speed_cfg["enable"]):
        speeds = load_speed(input_path(data_dir, spec, "speed"))
        if not speeds:
            sys.exit(f"speed: enable but no usable speed.csv in {data_dir}")

    leverarm = tuple(gnss_cfg["leverarm_frd"])
    score_la = tuple(spec["score"].get("leverarm_frd") or (0.0, 0.0, 0.0))

    # init: ref uses the reference epoch aligned with the FIRST IMU
    # sample, not ref[0]: ins defers a prescribed init to its actual
    # start epoch and stamps the provided state THERE without propagating
    # it (REQ-NAV-033). Handing it an older truth epoch bakes a permanent
    # -v*dt position offset into a moving start (2 m East on the Groves
    # aircraft profile: one 10-ms IMU period at 200 m/s East), which then
    # reads as filter inaccuracy.
    first_imu_us = next(iter_imu(imu_path))[0]
    ref0 = next((r for r in ref if r["t_us"] >= first_imu_us), ref[0])

    gyr_bias = estimate_gyro_bias(imu_path, spec["gyro_bias_window_sec"])
    bias_note = "from initial window" if gyr_bias else "n/a"
    if gyr_bias is not None and spec["init"] == "ref":
        # With init: ref the reference provides position, velocity AND
        # attitude, so the non-bias content of the window average can be
        # removed (see correct_initial_gyr_bias).
        gyr_bias = correct_initial_gyr_bias(
            gyr_bias, ref, first_imu_us, spec["gyro_bias_window_sec"])
        bias_note += ", ref motion + earth/transport rate removed"
    if not spec["gnss"]["enable"]:
        print("gnss: enable 0 - the fixes are read and counted, but none of "
              "them aids the filter")
    print(f"dataset {spec['name']}: aiding={spec['aiding']}, "
          f"init={spec['init']}, warmup {spec['score']['warmup_sec']:g} s, "
          f"leverarm FRD {list(leverarm)}"
          f"{', automotive' if spec['automotive_mode'] else ''}"
          f"{', mag' if mags else ''}{', baro' if baros else ''}"
          f"{f', gnss delay {gnss_delay_ms} ms' if gnss_delay_ms else ''}")
    print(f"initial gyro bias: "
          f"{[round(math.degrees(b), 4) for b in (gyr_bias or (0, 0, 0))]} deg/s "
          f"({bias_note})")
    print_chi2_gate(spec)
    notes = notable_settings(spec)
    if notes:
        print("notable settings (non-default):")
        for n in notes:
            print(f"  - {n}")
    else:
        print("notable settings: none (all defaults)")

    # fixes[0] is just a provisional seed for auto_init (overridden by the
    # first real fix); with aiding: none there is none, so fall back to
    # the reference epoch - harmless since ins then never leaves
    # is_collecting anyway.
    t0 = ref0 if (spec["init"] == "ref" or not fixes) else fixes[0]
    nav = Navigator(build_config(spec, ref0, t0["t_us"], t0["lat_rad"],
                                 t0["lon_rad"], t0["h_m"], gyr_bias))
    # baro_alt / ARS/AHRS noise model (0 -> each filter's own default); must
    # be set before the first baro sample / ins_suite_update() latches the
    # respective template (nav_suite contract).
    nav.set_baro_acc_bias_drift(float(baro_cfg.get("acc_bias_rw", 0.0)))
    nav.set_baro_acc_noise(float(baro_cfg.get("acc_noise_mps2_sqrthz", 0.0)))
    nav.set_baro_acc_bias_init_stddev(float(baro_cfg.get("acc_bias_init_mps2", 0.0)))
    nav.set_baro_h_process_noise(float(baro_cfg.get("h_process_noise", 0.0)))
    nav.set_local_gnss_rw_stddev(float(baro_cfg.get("local_gnss_rw_stddev_mps", 0.0)))
    nav.set_local_gnss_chi2_threshold(float(baro_cfg.get("local_gnss_chi2_threshold", 0.0)))
    nav.set_local_gnss_min_update_interval(
        float(baro_cfg.get("local_gnss_min_update_interval_sec", 0.0)))
    nav.set_local_gnss_stddev_inflation(
        float(baro_cfg.get("local_gnss_stddev_inflation_factor", 0.0)))
    nav.set_ahrs_gyr_noise(float(ahrs_cfg.get("gyr_noise_psd", 0.0)))
    nav.set_ahrs_acc_noise(float(ahrs_cfg.get("acc_noise_mps2", 0.0)))
    nav.set_ahrs_gyr_bias_rw(float(ahrs_cfg.get("gyr_bias_rw", 0.0)))
    nav.set_ahrs_gyr_bias_init_stddev(
        math.radians(float(ahrs_cfg.get("gyr_bias_init_stddev_rps_deg", 0.0))))
    init_hint = spec["init_hint"]
    nav.set_init_att_hint(
        math.radians(init_hint["roll_deg"]), math.radians(init_hint["pitch_deg"]),
        math.radians(init_hint["rpy_stddev_deg"]), math.radians(init_hint["yaw_deg"]),
        math.radians(init_hint["yaw_stddev_deg"]))
    # No set_auto_zaru() here: the ARS/AHRS stillness fallback (REQ-AHRS-017)
    # is armed by default and configured through the same imu.auto_zupt_* set
    # as ins (REQ-SUITE-020, already in the Config above). Calling the runtime
    # override here would silently re-arm a dataset that opted out with
    # imu.auto_zupt_velocity_blind_disable.
    #
    # Armed by default is what a real capture needs: the ARS/AHRS ZARU and,
    # through it, baro_alt's vertical ZUPT are decided on the IMU alone
    # (short-window sample stddev), which is available from the first samples,
    # whereas the trigger propagated from ins (REQ-SUITE-009) cannot answer
    # until ins has initialized.
    if mags and float(mag_cfg["wmm_year"]) > 0:
        # Both attitude sources agree on true north (WMM declination).
        nav.set_magnetic_model(ref[0]["lat_rad"], ref[0]["lon_rad"],
                               float(mag_cfg["wmm_year"]))
    elif mags:
        # Same up-front warning as tools/replay.c: without an epoch there is
        # no reference field and every magnetometer sample is rejected.
        print("  WARNING: mag.enable is set but mag.wmm_year is missing, no"
              " magnetic reference field is built and the magnetometer will"
              " not be fused (tools/inslib_convert_ubx_to_csv.py derives it"
              " from NAV-PVT)")
    tele = Telemetry(plotjuggler=args.plotjuggler, mavlink=args.mavlink,
                     pj_port=args.pj_port, mav_port=args.mav_port,
                     pj_log=args.flight_log)

    noise = spec["imu"]
    acc_var = (noise["acc_psd"],) * 3
    gyr_var = (noise["gyr_psd"],) * 3

    (n_imu_total, imu_duration, imu_hz, imu_max_gap, imu_max_gap_at,
     gyr_d2_stat, acc_d2_stat, rate_t0, imu_rate_t,
     imu_rate_hz) = _imu_prepass(imu_path, args.plot_rate_bucket_sec)
    progress, progress_done = _progress_reporter(n_imu_total, "imu")

    # per-stream sampling rate over time for the --plot
    # sensor-rate page, all sharing rate_t0 (the first IMU sample) as
    # their time origin, same as rec["t"].
    sensor_rate = {"imu": (imu_rate_t, imu_rate_hz)}
    if aiding_mode == "gnss" and fixes:
        sensor_rate["gnss"] = _bucketed_rate(
            [fx["t_us"] for fx in fixes], rate_t0, args.plot_rate_bucket_sec)
    if mags:
        sensor_rate["mag"] = _bucketed_rate(
            [m[0] for m in mags], rate_t0, args.plot_rate_bucket_sec)
    if baros:
        sensor_rate["baro"] = _bucketed_rate(
            [b[0] for b in baros], rate_t0, args.plot_rate_bucket_sec)

    iref = 0
    ifix = 0
    imag = 0
    ibaro = 0
    ispeed = 0
    n_speed_fused = 0
    speed_sd_cfg = float(speed_cfg["stddev_mps"])
    speed_delay_cfg = int(round(float(speed_cfg["delay_ms"])))
    t_prev = None
    n_imu = 0
    last_ref = None
    last_fix = None
    last_mag = None
    last_baro = None
    origin_ecef = None
    origin_lat = origin_lon = origin_h = 0.0
    last_pub_us = None
    pub_period_us = US_PER_SEC / max(args.telemetry_hz, 1e-3)
    wall0 = time.perf_counter()
    err_dump = None
    if args.dump_errors:
        err_dump = open(args.dump_errors, "w", encoding="utf-8")
        err_dump.write("# t_s, horizontal_m, down_m, north_m, east_m\n")
    sol_dump = None
    last_sol_us = None
    sol_period_us = (US_PER_SEC / args.dump_solution_hz
                     if args.dump_solution_hz > 0 else 0.0)
    if args.dump_solution:
        # newline="" so the dataset CSVs stay LF on every platform, matching
        # datasets/replay_format.py's writers
        sol_dump = open(args.dump_solution, "w", encoding="utf-8", newline="\n")
        sol_dump.write("# t_us, lat_deg, lon_deg, h_m, roll_deg, pitch_deg,"
                       " yaw_deg, vn_mps, ve_mps, vd_mps\n")
        sol_dump.write("# INSLIB nav_suite solution, python/replay.py"
                       " --dump-solution\n")
    t0_us = None
    t_warmup_end = ((fixes[0] if fixes else ref[0])["t_us"]
                    + int(spec["score"]["warmup_sec"] * US_PER_SEC))
    e_roll, e_pitch, e_yaw, e_pos = Stat(), Stat(), Stat(), Stat()
    e_height_ell = Stat()  # nav_suite_get_height_ellipsoid() vs ref_now["h_m"]

    # Optional ground-truth evaluation window (--eval-start/--eval-end),
    # in microseconds from the first IMU sample (t0_us, the printed-time
    # origin). Only the scoring below is gated - the filter still runs the
    # whole trial. Resolved to absolute timestamps once t0_us is known.
    eval_lo_us = (int(args.eval_start * US_PER_SEC)
                  if args.eval_start is not None else None)
    eval_hi_us = (int(args.eval_end * US_PER_SEC)
                  if args.eval_end is not None else None)

    # Auto-detect whether a GNSS-delay estimate is meaningful:
    # real GNSS aiding (not a ref-synthesized fix, which is perfectly
    # time-aligned with the reference by construction and would just
    # trivially self-correlate at ~0 ms) with usable velocity, plus a
    # barometer as the near-zero-latency vertical reference. --estimate-
    # gnss-delay forces it on regardless (e.g. to sanity-check aiding: ref).
    auto_gnss_delay = (aiding_mode == "gnss" and bool(baros)
                      and any(fx.get("vel_ok") and fx.get("cov_vel") is not None
                              for fx in fixes))
    do_gnss_delay_estimate = args.estimate_gnss_delay or auto_gnss_delay

    rec = None
    last_rec_us = None
    rec_period_us = US_PER_SEC / max(args.plot_hz, 1e-3)
    if args.plot or do_gnss_delay_estimate:
        rec = {k: [] for k in (
            "t", "pos", "pos_sigma", "vel", "vel_sigma",
            "rpy_deg", "rpy_sigma_deg", "ref_pos",
            "ref_vel", "ref_rpy_deg", "pos_err_ned",
            "rpy_err_deg", "baro_vel_d", "gnss_vel_d",
            # most recent fused GNSS fix's OWN reported 1-σ (receiver
            # covariance, after apply_fallback) - for the PDF summary's
            # "last GNSS fix accuracy" line, side-by-side with the filter's
            # final 1-σ above.
            "gnss_pos_sigma", "gnss_vel_sigma",
            # ins bias + 1-σ (all NED/body, SI units)
            "acc_bias", "acc_bias_sigma", "gyr_bias", "gyr_bias_sigma",
            # ins magnetometer hard-iron bias + 1-σ
            # (18-state mode only, mag: estimate_bias) [uT]
            "mag_bias", "mag_bias_sigma",
            # ARS/AHRS roll/pitch + own gyro bias/sigma
            "ars_rpy_deg", "ars_rpy_sigma_deg", "ars_gyr_bias", "ars_gyr_bias_sigma",
            "ahrs_rpy_deg", "ahrs_rpy_sigma_deg", "ahrs_gyr_bias", "ahrs_gyr_bias_sigma",
            # heading page: independent absolute-heading sources sampled
            # every tick, NaN when unavailable. mag_heading_deg is the
            # tilt-compensated raw-magnetometer compass (magnetic north),
            # gnss_course_deg the GNSS course-over-ground (atan2(vE,vN),
            # the automotive_mode yaw source) once moving fast enough.
            "mag_heading_deg", "gnss_course_deg",
            # baro_alt page: height/vel in the same NED-down convention as
            # pos/vel/ref_pos/ref_vel above (baro_alt itself is positive
            # up, negated to match), plus its own z-accel bias/sigma, plus
            # the GNSS fix in the same local frame as ref_pos/ref_vel.
            "baro_h_d", "baro_raw_d", "baro_h_sigma", "baro_acc_bias", "baro_acc_bias_sigma",
            "fix_pos_d",
            # baro-to-ellipsoid offset filter (nav_suite_get_height_ellipsoid)
            "local_gnss_offset", "local_gnss_offset_sigma",
            # nav_suite_get_height_ellipsoid() itself, i.e. h_local + the
            # ESTIMATED offset above (not the true origin_h ref_pos is
            # anchored to) -- reveals the offset filter's own reconstruction
            # error, invisible when everything is compared against the true
            # origin_h instead. Same NED-down convention as the other *_d
            # fields, so it overlays directly on the altitude-profile page.
            "nav_height_ellipsoid_d",
            # ins's own velocity-aware auto-ZUPT/ZARU detector (shaded on
            # the plot pages so stops can be correlated with bias jumps).
            "zupt_active",
            # The individual detectors zupt_active is OR'd together from,
            # each sampled separately for the dedicated ZUPT/ZARU timeline
            # page (_zupt_page) -- a mismatch between them (e.g. the ARS
            # thinks it's still but ins doesn't) is invisible on the
            # combined signal above. *_zaru_applied, not *_auto_zaru_active
            # (a much looser "stillness run started" gate that stays true
            # through genuine constant-velocity cruise) -- see
            # ars_zaru_applied()'s docstring.
            "auto_zupt_active", "vertical_zupt_active",
            "ars_zaru_applied", "ahrs_zaru_applied",
            # cumulative chi2-downweight counters, same
            # source as downweight_counts()/diag()['n_downweighted']
            # printed at the end - here sampled every recorder tick so
            # the outlier page can show WHEN, not just how many.
            "dw_full3d", "dw_ars", "dw_ahrs", "dw_baro_alt", "dw_local_gnss",
        )}

    # --plot North-East map recorder: the ground track alone (two floats
    # per sample), sampled independently of - and by default faster than -
    # the full state history above, which carries ~60 fields per tick and
    # would blow up the PDF and the memory footprint at this rate. See
    # --plot-track-hz and _ne_page/_thin_track in ins_plots.py.
    do_track = args.plot and args.plot_track_hz > 0.0
    track_est = [] if do_track else None
    track_ref = [] if do_track else None
    last_track_us = None
    track_period_us = US_PER_SEC / max(args.plot_track_hz, 1e-3)

    # Shared by --kml and --map-frames: both consume the same recorded
    # geodetic track, just render it differently.
    do_kml_track = bool(args.kml) or bool(args.map_frames)
    kml_est_track = [] if do_kml_track else None
    kml_ref_track = [] if do_kml_track else None
    kml_fix_track = [] if do_kml_track else None
    last_kml_us = None
    kml_period_us = US_PER_SEC / max(args.kml_hz, 1e-3)

    # Stationary-epoch count (for the "stationary epochs" line below), from
    # ins's own (velocity-aware) auto-ZUPT/ZARU detector.
    n_static = 0

    # GNSS accuracy self-check: collect the GNSS fixes that land during
    # an ins-detected standstill, grouped into contiguous
    # standstill phases, to later compare their empirical spread against
    # the receiver's own reported covariance (see gnss_standstill_accuracy).
    # Only meaningful for real GNSS aiding - a ref-synthesized fix equals
    # the truth and would trivially show ~zero spread.
    gnss_static_phases = []
    gnss_cur_phase = []

    # Same standstill-spread self-check for the raw baro/mag channels (see
    # channel_standstill_accuracy). Unlike GNSS these are never
    # reference-synthesized, so the check is meaningful without GNSS aiding
    # too - baro truly in ANY aiding mode (nav.vertical_zupt_active()
    # below folds in the ARS/AHRS fallback), mag still needs ins's own
    # auto-ZUPT/ZARU. Each phase is a list of per-sample tuples: baro (ISA
    # altitude,), mag (x, y, z).
    baro_static_phases, baro_cur_phase = [], []
    mag_static_phases, mag_cur_phase = [], []

    # Filter self-consistency watchdog: sample the filter's OWN reported
    # 1-σ (attitude + position) right after a fused GNSS fix, but only
    # past the warmup and while GNSS is streaming (the previous used fix is
    # still recent), so the uncertainty growth during initial convergence,
    # a GNSS outage, or the re-acquisition transient after one is excluded
    # - only steady, healthy operation is judged against STDDEV_LIMITS.
    health_stats = {k: {"worst": 0.0, "t": 0.0, "n_exceed": 0}
                    for k in STDDEV_LIMITS}
    health_n = 0
    last_fix_used_us = None
    # Streaming grace: the previous fix counts as "recent" within a few
    # nominal GNSS periods (>=2 s), so a slow receiver isn't permanently
    # treated as gapped, and a real gap (>= this) suppresses the sample.
    gnss_grace_us = 2 * US_PER_SEC
    if fixes and len(fixes) > 1:
        nominal_sec = ((fixes[-1]["t_us"] - fixes[0]["t_us"]) / US_PER_SEC
                       / (len(fixes) - 1))
        gnss_grace_us = int(max(2.0, 3.0 * nominal_sec) * US_PER_SEC)

    # free_inertial_start (see the offer inside the loop). Prepared once:
    # the ECEF of the declared point and the covariance of that statement.
    # gnss: enable 0 - the fixes stay loaded and counted, they just never
    # reach the filter (same key and same meaning as tools/insrcv.c).
    gnss_enable = bool(spec["gnss"]["enable"])
    fi_spec = spec["free_inertial_start"]
    fi_llh = fi_cov = None
    fi_next_t_us = 0
    n_fi_offers = 0
    if fi_spec["enable"]:
        fi_llh = (math.radians(fi_spec["lat_deg"]),
                  math.radians(fi_spec["lon_deg"]),
                  float(fi_spec["height_m"]))
        fi_var = float(fi_spec["stddev_m"]) ** 2
        fi_cov = [fi_var, 0.0, 0.0, 0.0, fi_var, 0.0, 0.0, 0.0, fi_var]

    for t, g, a in iter_imu(imu_path):
        if t0_us is None:
            t0_us = t
        dt = (t - t_prev) / US_PER_SEC if t_prev is not None else 0.0
        t_prev = t
        n_imu += 1
        progress(n_imu)

        nav.imu(t, dt, a, g, acc_var, gyr_var)

        # Attach the most recent fix (real GNSS or reference-derived).
        fix_now = None
        while ifix < len(fixes) and fixes[ifix]["t_us"] <= t:
            fix_now = fixes[ifix]
            ifix += 1
        # free_inertial_start: the declared origin, offered as a position
        # measurement until ins has bootstrapped from it and not one epoch
        # longer - a source that kept repeating the same point would pin the
        # solution to it instead of dead reckoning. Never in an epoch that
        # already carries a real fix: a measured position beats a declared
        # one. Mirrors fi_offer_start_position() in tools/insrcv.c.
        if (fi_llh is not None and fix_now is None
                and nav.deadreckoning_ms() < 0 and t >= fi_next_t_us):
            fi_next_t_us = t + US_PER_SEC // 2   # 2 Hz: entry dwell wants >= 1
            nav.gnss_pos_llh(fi_llh, fi_cov)
            n_fi_offers += 1

        if (gnss_enable and fix_now is not None
                and fix_now["cov_pos"] is not None):
            # REQ-VER-008: assumed fixed processing/telemetry latency,
            # applied uniformly regardless of aiding source; history-
            # anchors the fusion via ins's existing gnss_delay_ms.
            nav.gnss_pos_llh((fix_now["lat_rad"], fix_now["lon_rad"],
                              fix_now["h_m"]),
                             fix_now["cov_pos"], delay_ms=gnss_delay_ms)
            if fix_now["vel_ok"] and fix_now["cov_vel"] is not None:
                nav.gnss_vel(fix_now["vel_ned"], fix_now["cov_vel"])
            nav.gnss_leverarm(leverarm)
        if fix_now is not None:
            last_fix = fix_now

        # Attach the most recent magnetometer / barometer sample.
        mag_now = None
        while imag < len(mags) and mags[imag][0] <= t:
            mag_now = mags[imag]
            imag += 1
        if mag_now is not None:
            nav.mag(mag_now[1], mag_var)
            last_mag = mag_now
        baro_now = None
        while ibaro < len(baros) and baros[ibaro][0] <= t:
            baro_now = baros[ibaro]
            ibaro += 1
        if baro_now is not None:
            nav.baro(baro_now[1], float(baro_cfg["stddev_m"]))
            last_baro = baro_now

        # Absolute speed (REQ-NAV-068). Only the newest sample of the
        # interval is fused: re-fusing a value the filter has already seen
        # would count one measurement twice and make it overconfident about
        # the velocity, the same contract the barometer above follows.
        speed_now = None
        while ispeed < len(speeds) and speeds[ispeed][0] <= t:
            speed_now = speeds[ispeed]
            ispeed += 1
        if speed_now is not None:
            nav.speed(speed_now[1], speed_sd_cfg, speed_delay_cfg)
            n_speed_fused += 1

        nav.update()

        if nav.auto_zupt_active():
            n_static += 1

        ref_now = None
        while iref < len(ref) and ref[iref]["t_us"] <= t:
            ref_now = ref[iref]
            iref += 1
        if ref_now is not None:
            last_ref = ref_now

        # Cache the local-frame origin once the filter is up, so the ground
        # truth can be expressed in the same NED frame as the estimate. A
        # GNSS quality-loss re-bootstrap no longer moves the origin, it is
        # carried across (REQ-NAV-062); a time-jump reset (REQ-NAV-016) or a
        # bootstrap beyond the carry distance still establish a new one, and
        # rec['pos'] comes straight from ins's pos_local, so it would
        # restart at zero in a frame that no longer matches the cached one.
        # Refreshing the cache instead would move the truth frame mid-plot,
        # which is worse. Scoring is unaffected either way, it runs on
        # absolute ECEF (pos_error_ecef); only plot overlays drawn in this
        # cached frame would show the offset.
        if origin_ecef is None:
            origin_ecef = nav.origin_ecef()
            if origin_ecef is not None:
                origin_lat, origin_lon, origin_h = ecef_to_llh(*origin_ecef)

        # File a fused GNSS fix that arrived this epoch under the current
        # standstill phase (ins-ZUPT-detected), or end the
        # phase when a fix arrives while moving. fix_now is the fix just
        # fused (cov_pos None means it was rejected -> not counted); the
        # standstill flag is read AFTER nav.update() so it reflects this
        # epoch, and needs the local origin to place the fix in NED.
        if (aiding_mode == "gnss" and fix_now is not None
                and fix_now["cov_pos"] is not None):
            if nav.auto_zupt_active():
                if origin_ecef is not None:
                    cov_p = fix_now["cov_pos"]
                    cov_v = fix_now["cov_vel"]
                    has_vel = fix_now["vel_ok"] and cov_v is not None
                    gnss_cur_phase.append({
                        "pos_ned": ref_to_local_ned(fix_now, origin_ecef,
                                                    origin_lat, origin_lon),
                        "vel_ned": (list(fix_now["vel_ned"]) if has_vel
                                    else None),
                        "rep_pos_hor": math.sqrt((cov_p[0][0] + cov_p[1][1]) / 2.0),
                        "rep_pos_ver": math.sqrt(cov_p[2][2]),
                        "rep_vel_hor": (math.sqrt((cov_v[0][0] + cov_v[1][1]) / 2.0)
                                        if has_vel else None),
                        "rep_vel_ver": (math.sqrt(cov_v[2][2]) if has_vel
                                        else None),
                    })
            elif gnss_cur_phase:  # moving fix -> close the standstill phase
                gnss_static_phases.append(gnss_cur_phase)
                gnss_cur_phase = []

        # File the baro/mag samples that arrived this epoch under the current
        # ins-detected standstill, or close the phase once the platform moves
        # (same machinery as the GNSS check, but driven by the ZUPT flag
        # directly so it works without GNSS aiding). baro_now/mag_now are the
        # samples read this epoch (None if none arrived); the standstill flag
        # is read AFTER nav.update() so it reflects this epoch. The mag field
        # in body frame is only constant if the platform isn't ROTATING, so
        # gate it on the zero-angular-rate detector too - an in-place turn
        # during a stop would otherwise inflate the measured spread.
        #
        # baro uses the WIDER vertical_zupt_active() rather than
        # auto_zupt_active(): without absolute position aiding (aiding:
        # none) ins never initializes, so auto_zupt_active() stays false
        # for the whole replay and the baro accuracy check below would
        # silently never fire even on a dataset that never moves -
        # vertical_zupt_active() folds in the ARS/AHRS velocity-blind
        # fallback, the only stillness detector available in that mode
        # (see nav_suite.c, REQ-SUITE-015).
        baro_standstill = nav.vertical_zupt_active()
        if baros:
            if baro_standstill and baro_now is not None:
                baro_cur_phase.append((isa_pressure_to_altitude(baro_now[1]),))
            elif not baro_standstill and baro_cur_phase:
                baro_static_phases.append(baro_cur_phase)
                baro_cur_phase = []
        standstill = nav.auto_zupt_active()
        if mags:
            mag_still = standstill and nav.zaru_active()
            if mag_still and mag_now is not None:
                mag_cur_phase.append(tuple(mag_now[1]))
            elif not mag_still and mag_cur_phase:
                mag_static_phases.append(mag_cur_phase)
                mag_cur_phase = []

        # Filter self-consistency watchdog: on a fused fix, judge the
        # filter's freshly-corrected 1-σ against STDDEV_LIMITS - but
        # only past the warmup, while streaming (prev fix recent) and while
        # actually moving. Excludes: initial convergence (warmup), GNSS
        # gaps + the post-gap transient (streaming), and standstill (where
        # yaw is unobservable without a magnetometer, so a growing yaw
        # σ is expected, not a fault). Only steady, observable
        # navigation is judged.
        if fix_now is not None and fix_now["cov_pos"] is not None:
            streaming = (last_fix_used_us is not None
                         and (t - last_fix_used_us) <= gnss_grace_us)
            last_fix_used_us = t
            sd = nav.stddev() if nav.is_ready() else None
            if (streaming and t >= t_warmup_end and sd is not None
                    and not nav.auto_zupt_active()):
                health_n += 1
                t_sec = (t - t0_us) / US_PER_SEC
                vals = {"roll": math.degrees(sd["rpy"][0]),
                        "pitch": math.degrees(sd["rpy"][1]),
                        "yaw": math.degrees(sd["rpy"][2]),
                        "pos": math.sqrt(sum(x * x for x in sd["pos_ned"]))}
                for k, v in vals.items():
                    h = health_stats[k]
                    if v > h["worst"]:
                        h["worst"], h["t"] = v, t_sec
                    if v > STDDEV_LIMITS[k]:
                        h["n_exceed"] += 1

        # Telemetry, throttled in sim time so fast replays don't flood.
        # Publishes the estimate (INSLIB/...), the individual sub-filter
        # outputs (INSLIB/ars|ahrs|full3d|baroalt/...), the ground truth
        # (ref/...) and the raw input measurements (meas/...), so
        # everything available can be overlaid in one PlotJuggler session
        # (overlay the error yourself if you want it - e.g. ref/pos_ned
        # vs. INSLIB/pos_ned).
        if last_pub_us is None or (t - last_pub_us) >= pub_period_us:
            last_pub_us = t
            tele.publish_suite(nav)
            if last_ref is not None:
                ref_local = (ref_to_local_ned(last_ref, origin_ecef,
                                              origin_lat, origin_lon)
                             if origin_ecef is not None else None)
                tele.publish_extra(ref_overlay_tree(last_ref, ref_local))
            tele.publish_extra(
                meas_overlay_tree(a, g, last_fix, last_mag, last_baro,
                                  origin_ecef, origin_lat, origin_lon))

        # --plot recorder: state history + error samples at a fixed rate,
        # independent of the telemetry throttle above (see ins_plots.py).
        # "t" (and everything below) is recorded every period regardless of
        # ins's own readiness - ARS/AHRS/baro_alt run independently of
        # it (e.g. "aiding: none", where ins never initializes at all)
        # and each group is NaN-padded on its OWN availability, not
        # ins's, so every rec[...] list stays the same length as "t".
        if rec is not None and (last_rec_us is None
                                or (t - last_rec_us) >= rec_period_us):
            last_rec_us = t
            rec["t"].append((t - t0_us) / US_PER_SEC)
            # OR'd with vertical_zupt_active() (see the baro_standstill
            # comment above) so the --plot-out shading still shows
            # standstill phases - e.g. on the altitude page - for a
            # dataset with no absolute position aiding, where
            # auto_zupt_active() alone stays false for the whole replay.
            rec["zupt_active"].append(
                1.0 if (nav.auto_zupt_active() or nav.vertical_zupt_active()) else 0.0)
            rec["auto_zupt_active"].append(1.0 if nav.auto_zupt_active() else 0.0)
            rec["vertical_zupt_active"].append(1.0 if nav.vertical_zupt_active() else 0.0)
            rec["ars_zaru_applied"].append(1.0 if nav.ars_zaru_applied() else 0.0)
            rec["ahrs_zaru_applied"].append(1.0 if nav.ahrs_zaru_applied() else 0.0)

            # cumulative downweight counters, sampled every
            # tick so the outlier page can plot WHEN outliers occurred.
            dw_now = nav.downweight_counts()
            rec["dw_full3d"].append(float(nav.diag()["n_downweighted"]))
            rec["dw_ars"].append(float(dw_now["ars"]))
            rec["dw_ahrs"].append(float(dw_now["ahrs"]))
            rec["dw_baro_alt"].append(float(dw_now["baro_alt"]))
            rec["dw_local_gnss"].append(float(dw_now["local_gnss"]))

            pos = nav.position_local()
            vel = nav.velocity_ned()
            rpy = nav.rpy_ins()
            sd = nav.stddev()
            if pos is not None and vel is not None and rpy is not None and sd:
                rec["pos"].append(pos)
                rec["pos_sigma"].append(sd["pos_ned"])
                rec["vel"].append(vel)
                rec["vel_sigma"].append(sd["vel_ned"])
                rec["rpy_deg"].append([math.degrees(x) for x in rpy])
                rec["rpy_sigma_deg"].append(
                    [math.degrees(x) for x in sd["rpy"]])
                acc_bias = nav.bias_acc()
                rec["acc_bias"].append(list(acc_bias) if acc_bias else [math.nan] * 3)
                rec["acc_bias_sigma"].append(list(sd["acc_bias"]))
                gyr_bias = nav.bias_gyr()
                rec["gyr_bias"].append(list(gyr_bias) if gyr_bias else [math.nan] * 3)
                rec["gyr_bias_sigma"].append(list(sd["gyr_bias"]))
                # only present in 18-state mode (mag:
                # estimate_bias) - NaN otherwise, same convention as the
                # sub-filter fields below.
                mag_bias = nav.bias_mag()
                rec["mag_bias"].append(list(mag_bias) if mag_bias else [math.nan] * 3)
                rec["mag_bias_sigma"].append(
                    list(sd["mag_bias"]) if "mag_bias" in sd else [math.nan] * 3)
            else:
                for k in ("pos", "vel", "pos_sigma", "vel_sigma", "rpy_deg",
                         "rpy_sigma_deg", "acc_bias", "acc_bias_sigma",
                         "gyr_bias", "gyr_bias_sigma", "mag_bias", "mag_bias_sigma"):
                    rec[k].append([math.nan] * 3)

            # ARS/AHRS: their own roll/pitch (yaw omitted for the ARS -
            # free-running, not meaningful without a reference) and their
            # own independent gyro bias + 1-σ.
            for pfx, rpy_fn, rpy_sd_fn, bias_fn, sd_fn in (
                ("ars", nav.rpy_ars, nav.rpy_stddev_ars,
                 nav.bias_gyr_ars, nav.gyr_bias_stddev_ars),
                ("ahrs", nav.rpy_ahrs, nav.rpy_stddev_ahrs,
                 nav.bias_gyr_ahrs, nav.gyr_bias_stddev_ahrs),
            ):
                sub_rpy = rpy_fn()
                rec[f"{pfx}_rpy_deg"].append(
                    [math.degrees(x) for x in sub_rpy] if sub_rpy else [math.nan] * 3)
                sub_rpy_sd = rpy_sd_fn()
                rec[f"{pfx}_rpy_sigma_deg"].append(
                    [math.degrees(x) for x in sub_rpy_sd] if sub_rpy_sd else [math.nan] * 3)
                sub_bias = bias_fn()
                rec[f"{pfx}_gyr_bias"].append(list(sub_bias) if sub_bias else [math.nan] * 3)
                sub_sd = sd_fn()
                rec[f"{pfx}_gyr_bias_sigma"].append(list(sub_sd) if sub_sd else [math.nan] * 3)

            # Independent absolute-heading sources for the heading page.
            # Magnetometer: tilt-compensate the last sample with the best
            # available roll/pitch (full3d if ready, else AHRS/ARS, both of
            # which run without GNSS) - yaw is what we derive, so only
            # leveling matters. The sample is put through the configured
            # fixed calibration first, so the trace is the compass the filter
            # sees, not one still carrying the hard/soft-iron error.
            # NaN until a mag sample exists.
            level_rpy = rpy or nav.rpy_ahrs() or nav.rpy_ars()
            if last_mag is not None and level_rpy is not None:
                m_cal = mag_calibrate(last_mag[1], mag_misalign, mag_bias_cfg)
                rec["mag_heading_deg"].append(math.degrees(
                    mag_heading(m_cal, level_rpy[0], level_rpy[1])))
            else:
                rec["mag_heading_deg"].append(math.nan)
            # GNSS course over ground (the automotive_mode yaw source),
            # gated on horizontal speed so a standstill's noisy course
            # doesn't pollute the trace. Shown whenever GNSS velocity is
            # present, independent of whether automotive_mode is armed.
            if last_fix is not None and last_fix.get("vel_ok"):
                v_n, v_e = last_fix["vel_ned"][0], last_fix["vel_ned"][1]
                if math.hypot(v_n, v_e) >= max(
                        spec["automotive_min_speed_mps"], 2.0):
                    rec["gnss_course_deg"].append(math.degrees(math.atan2(v_e, v_n)))
                else:
                    rec["gnss_course_deg"].append(math.nan)
            else:
                rec["gnss_course_deg"].append(math.nan)

            # baro_alt: own height/velocity (negated to the same NED-down
            # convention as pos/vel above) and its own z-accel bias/σ.
            baro = nav.baro_alt()
            rec["baro_h_d"].append(-baro[0] if baro is not None else math.nan)
            rec["baro_raw_d"].append(
                -isa_pressure_to_altitude(last_baro[1])
                if last_baro is not None else math.nan)
            rec["baro_vel_d"].append(-baro[1] if baro is not None else math.nan)
            baro_bias = nav.baro_acc_bias()
            rec["baro_acc_bias"].append(baro_bias if baro_bias is not None else math.nan)
            baro_sd = nav.baro_stddev()
            rec["baro_h_sigma"].append(baro_sd[0] if baro_sd is not None else math.nan)
            rec["baro_acc_bias_sigma"].append(baro_sd[2] if baro_sd is not None else math.nan)
            gnss_offset = nav.local_gnss_offset()
            rec["local_gnss_offset"].append(gnss_offset[0] if gnss_offset is not None else math.nan)
            rec["local_gnss_offset_sigma"].append(
                gnss_offset[1] if gnss_offset is not None else math.nan)
            nav_h_ell = nav.height_ellipsoid()
            rec["nav_height_ellipsoid_d"].append(
                -(nav_h_ell - origin_h) if nav_h_ell is not None and origin_ecef is not None
                else math.nan)

            # GNSS fix, in the same local NED frame as ref_pos/ref_vel.
            rec["gnss_vel_d"].append(
                last_fix["vel_ned"][2] if last_fix is not None else math.nan)
            if last_fix is not None and origin_ecef is not None:
                rec["fix_pos_d"].append(ref_to_local_ned(
                    last_fix, origin_ecef, origin_lat, origin_lon)[2])
            else:
                rec["fix_pos_d"].append(math.nan)
            if last_fix is not None and last_fix.get("cov_pos") is not None:
                cov_p = last_fix["cov_pos"]
                rec["gnss_pos_sigma"].append(
                    [math.sqrt(cov_p[0][0]), math.sqrt(cov_p[1][1]),
                     math.sqrt(cov_p[2][2])])
                cov_v = last_fix.get("cov_vel")
                rec["gnss_vel_sigma"].append(
                    [math.sqrt(cov_v[0][0]), math.sqrt(cov_v[1][1]),
                     math.sqrt(cov_v[2][2])]
                    if last_fix.get("vel_ok") and cov_v is not None
                    else [math.nan] * 3)
            else:
                rec["gnss_pos_sigma"].append([math.nan] * 3)
                rec["gnss_vel_sigma"].append([math.nan] * 3)

            if last_ref is not None and origin_ecef is not None:
                err_ecef = (pos_error_ecef(nav, last_ref, score_la)
                           if nav.is_ready() else None)
                rec["ref_pos"].append(ref_to_local_ned(
                    last_ref, origin_ecef, origin_lat, origin_lon))
                rec["ref_vel"].append(list(last_ref["vel_ned"]))
                rec["ref_rpy_deg"].append(
                    [math.degrees(last_ref["roll_rad"]),
                     math.degrees(last_ref["pitch_rad"]),
                     math.degrees(last_ref["yaw_rad"])])
                rec["pos_err_ned"].append(
                    _matvec_rm_T(ned_to_ecef_rot(origin_lat, origin_lon),
                                 err_ecef) if err_ecef is not None
                    else [math.nan] * 3)
                rec["rpy_err_deg"].append(
                    [math.nan] * 3 if rpy is None else [
                        math.degrees(wrap_pi(rpy[0] - last_ref["roll_rad"])),
                        math.degrees(wrap_pi(rpy[1] - last_ref["pitch_rad"])),
                        math.degrees(wrap_pi(rpy[2] - last_ref["yaw_rad"])),
                    ])
            else:
                for k in ("ref_pos", "ref_vel", "ref_rpy_deg",
                         "pos_err_ned", "rpy_err_deg"):
                    rec[k].append([math.nan] * 3)

        # --plot map recorder: North/East only, at its own rate.
        if track_est is not None and (last_track_us is None
                                      or (t - last_track_us) >= track_period_us):
            last_track_us = t
            track_pos = nav.position_local()
            # A NaN pair rather than a skipped sample, so the drawn line
            # BREAKS while ins has nothing instead of bridging the gap
            # with a straight chord.
            track_est.append((track_pos[0], track_pos[1])
                             if track_pos is not None
                             else (math.nan, math.nan))
            if last_ref is not None and origin_ecef is not None:
                r = ref_to_local_ned(last_ref, origin_ecef,
                                     origin_lat, origin_lon)
                # The reference arrives at its own (usually slower) rate,
                # so most ticks repeat the previous point - dropping the
                # repeats keeps the identical polyline at a fraction of
                # the memory.
                if not track_ref or (r[0], r[1]) != track_ref[-1]:
                    track_ref.append((r[0], r[1]))

        # --kml recorder: geodetic track for Google Earth (see ins_kml.py).
        if kml_est_track is not None and (last_kml_us is None
                                          or (t - last_kml_us) >= kml_period_us):
            last_kml_us = t
            ecef = nav.position_ecef()
            rpy = nav.rpy_ins()
            if ecef is not None and rpy is not None:
                lat, lon, alt = ecef_to_llh(*ecef)
                kml_est_track.append(((t - t0_us) / US_PER_SEC,
                                      math.degrees(lat), math.degrees(lon), alt,
                                      math.degrees(rpy[0]), math.degrees(rpy[1]),
                                      math.degrees(rpy[2])))
                if last_ref is not None:
                    kml_ref_track.append((math.degrees(last_ref["lat_rad"]),
                                          math.degrees(last_ref["lon_rad"]),
                                          last_ref["h_m"]))
                if last_fix is not None:
                    fx = (math.degrees(last_fix["lat_rad"]),
                         math.degrees(last_fix["lon_rad"]),
                         last_fix["h_m"])
                    # North/East/(North-East) position covariance in m^2,
                    # for --map-frames' error ellipses; NaN if this fix
                    # carries none (drawn as no ellipse there).
                    cov_p = last_fix.get("cov_pos")
                    cov_ne = ((cov_p[0][0], cov_p[0][1], cov_p[1][1])
                             if cov_p is not None
                             else (math.nan, math.nan, math.nan))
                    # Dedup repeats (fix rate is usually slower than
                    # --kml-hz) so a stale fix during an outage draws as
                    # one held point, not a cluster of identical vertices.
                    # t_rel is kept even on a repeat (--map-frames uses it to
                    # tell "still the last fix" from "no fix yet at all").
                    if not kml_fix_track or fx != kml_fix_track[-1][1:4]:
                        kml_fix_track.append(
                            ((t - t0_us) / US_PER_SEC,) + fx + cov_ne)

        # --dump-solution: the filter's own trajectory in the dataset ref
        # format. Independent of the reference (there may not be one), and of
        # the warmup, so the caller sees the whole run including the
        # convergence at its start.
        if sol_dump is not None and (last_sol_us is None
                                     or (t - last_sol_us) >= sol_period_us):
            ecef = nav.position_ecef()
            rpy = nav.rpy_ins()
            vel = nav.velocity_ned()
            if ecef is not None and rpy is not None and vel is not None:
                last_sol_us = t
                lat, lon, alt = ecef_to_llh(*ecef)
                sol_dump.write(
                    "%d,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
                        t, math.degrees(lat), math.degrees(lon), alt,
                        math.degrees(rpy[0]), math.degrees(rpy[1]),
                        math.degrees(rpy[2]), vel[0], vel[1], vel[2]))

        # Score against the ground truth after the warmup (same metrics
        # as the C harness: attitude mean/std, position rms), optionally
        # further restricted to the --eval-start/--eval-end window.
        rel_us = t - t0_us
        in_eval_window = ((eval_lo_us is None or rel_us >= eval_lo_us)
                          and (eval_hi_us is None or rel_us <= eval_hi_us))
        if (ref_now is not None and nav.is_ready() and t >= t_warmup_end
                and in_eval_window):
            err = pos_error_ecef(nav, ref_now, score_la)
            if err is not None:
                e_pos.add(math.sqrt(sum(e * e for e in err)))
                if err_dump is not None:
                    # Split into horizontal and vertical. The scored
                    # "pos error" is the 3D magnitude, and on a
                    # baro-driven vertical channel that number can be
                    # dominated by the height - which says nothing about
                    # how well the horizontal dead reckoning did.
                    ned = _matvec_rm(
                        ecef_to_ned_rot(ref_now["lat_rad"], ref_now["lon_rad"]),
                        err)
                    err_dump.write("%.3f,%.3f,%.3f,%.3f,%.3f\n" % (
                        rel_us / US_PER_SEC, math.hypot(ned[0], ned[1]),
                        ned[2], ned[0], ned[1]))
            rpy = nav.rpy_ins()
            if rpy is not None:
                e_roll.add(math.degrees(wrap_pi(rpy[0] - ref_now["roll_rad"])))
                e_pitch.add(math.degrees(wrap_pi(rpy[1] - ref_now["pitch_rad"])))
                e_yaw.add(math.degrees(wrap_pi(rpy[2] - ref_now["yaw_rad"])))
        # nav_suite_get_height_ellipsoid() vs ref_now["h_m"] directly
        # (absolute, like the C harness's ell_h): deliberately NOT gated on
        # nav.is_ready() like the block above -- the accessor still succeeds
        # during COASTING/ATTITUDE_ONLY (baro_alt + the offset filter's
        # estimate), which is exactly the case worth scoring, unlike e_pos.
        if ref_now is not None and t >= t_warmup_end and in_eval_window:
            h_ell = nav.height_ellipsoid()
            if h_ell is not None:
                e_height_ell.add(h_ell - ref_now["h_m"])

        if args.realtime:
            target = wall0 + (t - t0_us) / US_PER_SEC / max(args.speed, 1e-6)
            sleep = target - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

    progress_done()
    if gnss_cur_phase:  # close a standstill phase still open at EOF
        gnss_static_phases.append(gnss_cur_phase)
    if baro_cur_phase:
        baro_static_phases.append(baro_cur_phase)
    if mag_cur_phase:
        mag_static_phases.append(mag_cur_phase)
    if sol_dump is not None:
        sol_dump.close()
        print(f"solution written to {args.dump_solution}")
    diag = nav.diag()
    print(f"\nreplayed {n_imu} IMU samples, {len(fixes)} fixes, "
          f"{len(ref)} reference epochs"
          f"{f', {len(mags)} mag' if mags else ''}"
          f"{f', {len(baros)} baro' if baros else ''}"
          f"{f', {len(speeds)} speed' if speeds else ''}")
    if n_fi_offers:
        print(f"free-inertial start: {n_fi_offers} declaration(s) offered at "
              f"{fi_spec['lat_deg']:.7f} {fi_spec['lon_deg']:.7f} "
              f"h={fi_spec['height_m']:.1f} m +-{fi_spec['stddev_m']:g} m, "
              f"then the filter is on its own")
    if speeds:
        d = diag
        print(f"speed aiding: {n_speed_fused} samples offered, "
              f"{d.get('n_speed_used', 0)} fused, "
              f"{d.get('n_speed_skipped', 0)} skipped below the speed gate, "
              f"last residual {d.get('last_speed_residual_mps', 0.0):+.3f} m/s")
    print(f"ins: {diag['n_predict']} predicts, {diag['n_gnss_used']} gnss "
          f"fusions ({diag['n_gnss_seen']} seen), {diag['n_fuse_fail']} fuse "
          f"fails, {diag['n_auto_zupt']} auto-zupt, "
          f"{diag['n_downweighted']} downweighted")
    dw = nav.downweight_counts()
    print(f"downweighted (chi2 outlier): ars {dw['ars']}, "
          f"ahrs {dw['ahrs']}, baro_alt {dw['baro_alt']}, "
          f"local_gnss_offset {dw['local_gnss']}")
    # Which sub-filters were alive at the end, and - when the 3D filter
    # was not - the reason (same codes telemetry publishes live under
    # INSLIB/status, so a replay and a live session agree on the verdict).
    st_tree, _blocked, blocked_text = suite_status(nav)
    running = [n for n, k in (("ars", "ars_running"), ("ahrs", "ahrs_running"),
                              ("baro_alt", "baro_running"),
                              ("full3d", "full3d_running"))
               if st_tree[k] > 0.0]
    print(f"final state: mode {nav.mode_name()}, running "
          f"[{', '.join(running) if running else 'none'}] - {blocked_text}")
    # Final filter-reported 1-σ of the ins error state (last epoch): the
    # exact sqrt of the covariance diagonal, i.e. how uncertain ins believes
    # its own solution is at the end of the run (not a vs-truth error).
    sd = nav.stddev()
    if sd is not None:
        p, v, a = sd["pos_ned"], sd["vel_ned"], sd["rpy"]
        ab, gb = sd["acc_bias"], sd["gyr_bias"]
        print("final 1-σ (last epoch, filter-reported):")
        print(f"  pos NED   {p[0]:.3f} {p[1]:.3f} {p[2]:.3f} m")
        print(f"  vel NED   {v[0]:.3f} {v[1]:.3f} {v[2]:.3f} m/s")
        print(f"  att RPY   {math.degrees(a[0]):.3f} {math.degrees(a[1]):.3f} "
              f"{math.degrees(a[2]):.3f} deg")
        print(f"  acc bias  {ab[0]:.4f} {ab[1]:.4f} {ab[2]:.4f} m/s^2")
        print(f"  gyr bias  {math.degrees(gb[0]):.4f} {math.degrees(gb[1]):.4f} "
              f"{math.degrees(gb[2]):.4f} deg/s")
        if "mag_bias" in sd:
            mb = sd["mag_bias"]
            print(f"  mag bias  {mb[0]:.3f} {mb[1]:.3f} {mb[2]:.3f} uT")
    fix_acc_lines = last_fix_accuracy_lines(last_fix)
    if fix_acc_lines is not None:
        for line in fix_acc_lines:
            print(line)
    for line in growth_rate_lines(noise):
        print(line)
    for line in baro_growth_rate_lines(baro_cfg):
        print(line)
    for line in ahrs_growth_rate_lines(ahrs_cfg):
        print(line)
    # Overconfidence / covariance-collapse watchdog (REQ-NAV-040): the filter
    # reporting a physically implausible accuracy means its covariance has
    # collapsed and it may be silently diverging while looking confident.
    oc = nav.overconfidence()

    def _mm(x):
        return "-" if not math.isfinite(x) else f"{x:.2e}"
    print(f"covariance watchdog: best reported stddev pos {_mm(oc['min_pos_m'])} m, "
          f"vel {_mm(oc['min_vel_mps'])} m/s, att {_mm(oc['min_att_deg'])} deg")
    print(f"  ARS best att stddev {_mm(oc['ars']['min_att_deg'])} deg, "
          f"AHRS best att stddev {_mm(oc['ahrs']['min_att_deg'])} deg")
    for who, sub in (("INSLIB", oc), ("ARS", oc["ars"]), ("AHRS", oc["ahrs"])):
        if sub["tripped"]:
            print(f"  WARNING: {who} reported an implausibly small stddev at "
                  f"{sub['n']} epoch(s) - likely covariance collapse / divergence.")
    if e_pos.n:
        window = ""
        if args.eval_start is not None or args.eval_end is not None:
            lo = f"{args.eval_start:g}" if args.eval_start is not None else "0"
            hi = f"{args.eval_end:g}" if args.eval_end is not None else "end"
            window = f", eval window [{lo}, {hi}] s"
        print(f"ins vs ground truth (n={e_pos.n}, "
              f"after {spec['score']['warmup_sec']:g} s warmup{window}):")
        # score.attitude = 0 means the dataset states it has no attitude
        # reference (ref.csv carries a 0/0/0 placeholder). Printing an error
        # against that reads as a 40 deg yaw failure of the filter, which is
        # not what the number means.
        if spec["score"]["attitude"]:
            for name, s in (("roll", e_roll), ("pitch", e_pitch), ("yaw", e_yaw)):
                print(f"  {name:5s} error: mean {s.mean():+7.3f}  "
                      f"std {s.std():6.3f}  max |{s.max_abs:6.3f}| deg")
        else:
            print("  attitude error: n/a (score.attitude = 0, no attitude "
                  "reference in this dataset)")
        print(f"  pos rms: {e_pos.rms():.3f} m  max {e_pos.max_abs:.3f} m")
    if e_height_ell.n:
        print(f"nav_suite_get_height_ellipsoid() vs ground truth (absolute, "
              f"n={e_height_ell.n}):")
        print(f"  height error: mean {e_height_ell.mean():+.3f}  "
              f"std {e_height_ell.std():.3f}  rms {e_height_ell.rms():.3f}  "
              f"max |{e_height_ell.max_abs:.3f}| m")
    print(f"final nav_suite mode: {nav.mode_name()}")

    # --- data quality summary ----------------------------------------------
    print("\ndata quality summary:")
    print(f"  imu:  {imu_hz:6.1f} Hz avg (n={n_imu_total}, {imu_duration:.1f} s), "
         f"max gap {imu_max_gap * 1000.0:.0f} ms at t={imu_max_gap_at:.1f} s")

    def _report_stream(label, timestamps_us):
        n, dur, hz, gap, gap_at = _stream_gap_stats(timestamps_us)
        if n < 2:
            print(f"  {label}: n/a")
            return
        print(f"  {label}: {hz:6.2f} Hz avg (n={n}, {dur:.1f} s), "
             f"max gap {gap:.2f} s at t={gap_at:.1f} s")

    _report_stream("gnss", [fx["t_us"] for fx in fixes])
    if mags:
        _report_stream("mag ", [row[0] for row in mags])
    if baros:
        _report_stream("baro", [row[0] for row in baros])

    if n_static:
        frac = 100.0 * n_static / n_imu_total if n_imu_total else 0.0
        print(f"  stationary epochs: {n_static}/{n_imu_total} "
             f"({frac:.1f}% of trial, ins auto-ZUPT/ZARU-detected)")
    else:
        print("  stationary epochs: none detected"
             f"{' (aiding: none - ins never runs auto-ZUPT/ZARU)' if aiding_mode == 'none' else ''}")

    # Is the configured/reported GNSS accuracy realistic? Only for real
    # GNSS aiding (ref-synthesized fixes equal the truth).
    if aiding_mode == "gnss":
        print_gnss_standstill_accuracy(gnss_static_phases)

    # Same check for the raw baro/mag channels (any aiding mode).
    if baros:
        print_channel_standstill_accuracy(
            "baro", "m", baro_static_phases, float(baro_cfg["stddev_m"]))
    if mags:
        print_channel_standstill_accuracy(
            "mag ", "uT", mag_static_phases, mag_sd)

    def _noise_report(label, unit, motion_stat, expect, to_unit):
        # Per-sample sensor noise floor vs. the configured model, measured
        # in-motion: whole-trial second difference (see _imu_prepass) cancels
        # bias and smooth vehicle dynamics, keeps sensor noise + vibration,
        # and works even when the platform never stops. Divide by sqrt(6):
        # Var(2nd diff) = 6*σ^2 for white noise. This is the basis for
        # the suggested model below.
        if motion_stat[0].n <= 1:
            return
        meas = math.sqrt(sum(s.std() ** 2 for s in motion_stat) / 3.0) / math.sqrt(6.0)
        ratio = meas / expect if expect > 0 else math.nan
        print(f"  {label} noise vs. configured {to_unit(expect):.4f} {unit}:")
        print(f"    in-motion (no dynamics): {to_unit(meas):8.4f} {unit}  "
             f"(ratio {ratio:.2f}, n={motion_stat[0].n})")

    def _suggest_psd(d2_stat, dt_nominal):
        # Invert the maneuver-robust second-difference noise estimate back
        # into the PSD the filter's noise model wants: expect = sqrt(psd/dt)
        # -> psd = σ^2 * dt. σ is the per-sample per-axis stddev,
        # averaged over the 3 axes (the config carries one scalar
        # gyr_psd/acc_psd applied to all axes). For the second difference
        # Var = 6*σ^2 (vs 2 for the first), so divide by 6 - this is
        # what drops smooth vehicle dynamics while keeping vibration+noise.
        if d2_stat[0].n < 2:
            return None
        var = sum(s.std() ** 2 for s in d2_stat) / 3.0 / 6.0
        return var * dt_nominal

    noise_metrics = None  # measured/expected ratios for the Dr. INS check
    if imu_hz > 0:
        dt_nominal = 1.0 / imu_hz
        gyr_expect = math.sqrt(noise["gyr_psd"] / dt_nominal)
        acc_expect = math.sqrt(noise["acc_psd"] / dt_nominal)
        _noise_report("gyro ", "deg/s", gyr_d2_stat, gyr_expect, math.degrees)
        _noise_report("accel", "m/s^2", acc_d2_stat, acc_expect, lambda x: x)

        def _meas_floor(d2_stat):
            return (math.sqrt(sum(s.std() ** 2 for s in d2_stat) / 3.0) / math.sqrt(6.0)
                    if d2_stat[0].n > 1 else None)
        gyr_meas = _meas_floor(gyr_d2_stat)
        acc_meas = _meas_floor(acc_d2_stat)

        # Turn the measurement into a concrete config action: which noise
        # term to set, and to what. Based on the maneuver-robust
        # second-difference estimate ON PURPOSE - the goal is the effective
        # sensor noise floor under real flight conditions (white noise +
        # vibration), NOT vehicle dynamics (which the second difference
        # cancels), and not the datasheet/lab or stationary-epoch value.
        gyr_psd_sugg = _suggest_psd(gyr_d2_stat, dt_nominal)
        acc_psd_sugg = _suggest_psd(acc_d2_stat, dt_nominal)
        if gyr_psd_sugg is not None and acc_psd_sugg is not None:
            print("  suggested imu noise model for real flight conditions "
                 "(paste into config.yaml `imu:`):")
            print(f"    gyr_psd: {gyr_psd_sugg:.3e}   # (rad/s)^2/Hz  "
                 f"(now {noise['gyr_psd']:.3e}, x{gyr_psd_sugg / noise['gyr_psd']:.1f})")
            print(f"    acc_psd: {acc_psd_sugg:.3e}   # (m/s^2)^2/Hz  "
                 f"(now {noise['acc_psd']:.3e}, x{acc_psd_sugg / noise['acc_psd']:.1f})")
            print("    (from the in-motion (no-dynamics) line above: sensor noise + "
                 "vibration,")
            print("     not vehicle motion.")
            # The bias random walk (imu.*_bias_rw) is a slow drift we can't
            # read off a moving trial - it needs Allan variance on a long
            # static recording. Point the user at the dedicated tool.
            print("  bias random walk (imu.gyr_bias_rw / acc_bias_rw) is NOT "
                 "estimated here -")
            print("  it needs a long STATIC recording, use "
                 "allan_variance.py for those.")

        noise_metrics = {
            "gyr_ratio": (gyr_meas / gyr_expect if gyr_meas and gyr_expect > 0 else None),
            "acc_ratio": (acc_meas / acc_expect if acc_meas and acc_expect > 0 else None),
            "gyr_psd_sugg": gyr_psd_sugg, "acc_psd_sugg": acc_psd_sugg,
        }

    # --- Dr. INS findings: prioritized health check over everything above ---
    # Computed once, printed here and (below) rendered as the --plot PDF's
    # cover page.
    insdoctor = insdoctor_findings({
        "aiding_mode": aiding_mode,
        "imu_hz": imu_hz,
        "imu_max_gap": imu_max_gap, "imu_max_gap_at": imu_max_gap_at,
        "imu_rate_hz": imu_rate_hz,
        "gnss": _stream_gap_stats([fx["t_us"] for fx in fixes]) if fixes else None,
        "noise": noise_metrics,
        "gnss_acc": (gnss_standstill_accuracy(gnss_static_phases)
                     if aiding_mode == "gnss" else None),
        "gnss_vel": (gnss_vel_accuracy_stats(fixes)
                    if aiding_mode == "gnss" else None),
        "gnss_max_hor_vel_stddev_mps": gnss_cfg["max_horizontal_vel_stddev_mps"],
        "baro_acc": (channel_standstill_accuracy(
            baro_static_phases, float(baro_cfg["stddev_m"])) if baros else None),
        "mag_acc": (channel_standstill_accuracy(
            mag_static_phases, mag_sd) if mags else None),
        "health": {"n": health_n, "limits": STDDEV_LIMITS, "stats": health_stats},
        "overconfidence": oc,
        "leverarm": {"gnss": leverarm, "score": score_la},
        "baro_height": {
            # n_baro_height_used > 0 <=> ins latched the barometric height
            # source at bootstrap (REQ-NAV-053/-054).
            "active": diag["n_baro_height_used"] > 0,
            # rec is only populated when the plot/telemetry recorder runs
            # (--plot/--plotjuggler/...); without it the span is unknown.
            "offset_span_m": (_span(rec["local_gnss_offset"])
                              if rec is not None else None),
        },
        "diag": diag,
    })
    print_insdoctor(insdoctor)

    nav.close()
    tele.close()

    # Computed once (--plot-hz recorder), shared by the printed summary
    # below and the --plot PDF's GNSS-delay page.
    gnss_delay_curve = (gnss_delay_correlation_curve(rec["t"], rec["baro_vel_d"],
                                                     rec["gnss_vel_d"])
                       if do_gnss_delay_estimate else None)

    if args.plot:
        # High-rate ground track for the North-East map page (empty when
        # --plot-track-hz is 0, which makes _ne_page fall back to the
        # --plot-hz state history). Its own length, unrelated to rec["t"].
        rec["track_ne"] = track_est or []
        rec["track_ref_ne"] = track_ref or []
        # Raw magnetometer point cloud for the hard-iron sphere page. Kept at
        # full resolution (thinned in the plot itself); an all-zero fixed_bias
        # means "no fixed calibration" and is passed through as None.
        rec["mag_raw"] = [tuple(m[1]) for m in mags]
        rec["mag_fixed_bias"] = mag_bias_cfg if any(mag_bias_cfg) else None
        # Calibrated field magnitude |B|(t) vs. the WMM total field F, for the
        # "deviation from WMM" page: the calibration the filter applies is
        # applied here too, so the page scores the field the filter fuses. The
        # uncalibrated magnitude comes along as a second trace to show what the
        # calibration bought. Times share rec["t"]'s origin (t0_us).
        rec["wmm_field_uT"] = None
        rec["mag_field_t"] = []
        rec["mag_field_mag"] = []
        rec["mag_field_mag_raw"] = []
        # WGS84 ellipsoid height of nav's own (fixed) local-NED origin, so
        # the baro_alt page can convert its ref_pos/fix_pos_d (already
        # expressed relative to that origin, see ref_to_local_ned above)
        # back to absolute ellipsoid height and overlay it against
        # baro_h_d + local_gnss_offset -- the same origin_ecef this
        # replay loop cached earlier for ref_to_local_ned.
        rec["origin_ellipsoid_h_m"] = origin_h if origin_ecef is not None else math.nan
        if mags and float(mag_cfg["wmm_year"]) > 0:
            from INSLIB import wmm_field_ned
            b_ned = wmm_field_ned(math.degrees(ref[0]["lat_rad"]),
                                  math.degrees(ref[0]["lon_rad"]),
                                  float(mag_cfg["wmm_year"]))
            rec["wmm_field_uT"] = math.sqrt(sum(c * c for c in b_ned))
            rec["mag_field_t"] = [(m[0] - t0_us) / US_PER_SEC for m in mags]
            rec["mag_field_mag"] = [
                math.sqrt(sum(c * c for c in
                              mag_calibrate(m[1], mag_misalign, mag_bias_cfg)))
                for m in mags]
            # Only worth a second trace when a calibration is actually
            # configured, otherwise it would sit exactly on the first one.
            if mag_cal_active:
                rec["mag_field_mag_raw"] = [math.sqrt(sum(c * c for c in m[1]))
                                            for m in mags]
        from ins_plots import plot_results
        plot_results(rec, spec["name"] or args.dataset, spec["score"]["warmup_sec"],
                    out_path=args.plot_out, gnss_delay_curve=gnss_delay_curve,
                    sensor_rate=sensor_rate, findings=insdoctor,
                    configured_gnss_delay_ms=gnss_delay_ms,
                    growth_rate=process_noise_growth_rate(noise),
                    baro_growth_rate=baro_alt_growth_rate(baro_cfg),
                    ahrs_growth_rate=ahrs_growth_rate(ahrs_cfg))

    if args.kml:
        from ins_kml import write_kml
        write_kml(args.kml, kml_est_track, kml_ref_track, kml_fix_track,
                 spec["name"] or args.dataset)

    if args.map_frames:
        from ins_map_frames import write_frames
        write_frames(args.map_frames, kml_est_track, kml_fix_track,
                    spec["name"] or args.dataset,
                    fps=args.map_fps, t_start=args.map_t_start,
                    t_end=args.map_t_end)

    if do_gnss_delay_estimate:
        if gnss_delay_curve is None:
            print("gnss delay estimate: not enough data (need baro_alt "
                 "running and at least one GNSS fix)")
        else:
            delay_ms, corr = max(gnss_delay_curve, key=lambda row: row[1])
            quality = ("good" if corr > 0.7 else
                      "weak - probably not enough vertical motion in "
                      "this trial to tell" if corr > 0.3 else
                      "poor - do not trust this number")
            print(f"gnss delay estimate: {delay_ms:.0f} ms "
                 f"(correlation {corr:.2f}, {quality}) - relative to "
                 f"baro_alt, which is not necessarily latency-free "
                 f"either (see --estimate-gnss-delay's help)")
            # Point at the knob that consumes this number (REQ-VER-008):
            # config `gnss: delay_ms` history-anchors the fix by this much.
            if corr > 0.7:
                print(f"  -> set `gnss: delay_ms: {delay_ms:.0f}` in "
                     f"config.yaml to compensate it "
                     f"(currently {gnss_delay_ms} ms)")
            else:
                print(f"  (would go into config `gnss: delay_ms`, now "
                     f"{gnss_delay_ms} ms - but the correlation is too low "
                     f"to trust this value)")

    # --- machine-readable accuracy summary ---------------------------------
    # Dump the same accuracy numbers just printed above as JSON, for a
    # downstream consumer that wants them without parsing console text.
    # E.g. a regression harness (datasets/check_simulated.py, REQ-VER-016).
    if args.summary_json:
        summary = {
            "dataset": os.path.basename(os.path.normpath(data_dir)),
            "name": spec["name"],
            "warmup_sec": float(spec["score"]["warmup_sec"]),
            "t_warmup_end_us": int(t_warmup_end),
            "eval_start_sec": args.eval_start,
            "eval_end_sec": args.eval_end,
            "scored_epochs": e_pos.n,
            "pos_rms_m": e_pos.rms(),
            "pos_max_m": e_pos.max_abs,
            "att_err_deg": {
                "roll": {"mean": e_roll.mean(), "std": e_roll.std(), "max_abs": e_roll.max_abs},
                "pitch": {"mean": e_pitch.mean(), "std": e_pitch.std(), "max_abs": e_pitch.max_abs},
                "yaw": {"mean": e_yaw.mean(), "std": e_yaw.std(), "max_abs": e_yaw.max_abs},
            },
        }
        with open(args.summary_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        print(f"\nwrote accuracy summary to {args.summary_json}")


if __name__ == "__main__":
    # Some summary lines use non-ASCII characters (sigma etc.); the default
    # Windows console codepage (cp1252) can't encode them and would crash
    # the print, unlike UTF-8 terminals (Linux/macOS/WSL).
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    main()
