/** @file replay.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Post processing of data: runs the nav_suite (ins + the
 * two AHRS filters + baro) on `.csv` inputs and
 * scores the estimates against the ground truth.
 * Dataset-specific settings are stored in a
 * config.yaml file next to the input CSVs.
 * (REQ-VER-003, REQ-VER-008, REQ-VER-010)
 *
 * Inputs for a dataset:
 *   config.yaml  lever arms, IMU noise model, aiding/init mode, warmup,
 *                regression limits, etc.
 *   imu.csv      t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]
 *                (an eighth column, the IMU die temperature, is written by
 *                some converters and ignored here -- the sscanf below stops
 *                after seven fields)
 *   ref.csv      t_us, lat/lon/h, roll/pitch/yaw [deg], v_ned [m/s]
 *   gnss.csv     t_us, lat/lon/h, full NED pos covariance (6), v_ned,
 *                full NED vel covariance (6), vel_ok
 *   mag.csv      t_us, mag_frd_xyz [uT]          (optional)
 *   baro.csv     t_us, static pressure [Pa]      (optional)
 *   speed.csv    t_us, ground speed [m/s]        (optional)
 *
 * Usage: replay <config.yaml | datadir> [errdump.csv]
 *        (a datadir implies datadir/config.yaml, the errdump carries
 *        per-epoch errors for plotting)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h> /* isatty/fileno for the interactive progress bar */

#include "nav_suite.h"
#include "geodetic_toolbox.h"
#include "linalg.h"
#include "mini_yaml.h"

/* All streams are loaded in full at their native rate. The per-stream buffers
 * grow on demand (realloc doubling). If the machine runs out of memory the
 * loader aborts loudly. */
#define US_PER_SEC 1000000LL

static int fails = 0;

#define CHECK_LIMIT(val, limit, msg)                                                               \
    do {                                                                                           \
        if (!((val) < (limit)))                                                                    \
        {                                                                                          \
            printf("  FAIL  %-44s: %8.3f (limit %g)\n", msg, (double)(val), (double)(limit));      \
            fails++;                                                                               \
        }                                                                                          \
        else { printf("  ok    %-44s: %8.3f (limit %g)\n", msg, (double)(val), (double)(limit)); } \
    } while (0)

/* ---------------------------------------------------------------------------
 * Dataset configuration (config.yaml)
 * ------------------------------------------------------------------------- */

typedef struct
{
    char name[64];
    char aiding[16]; /* "gnss" | "ref" | "none" */
    /* free_inertial_start: the operator's declared origin, offered to the
       filter until it bootstraps from it. Same key, same meaning and the
       same forced options as tools/insrcv.c - one config.yaml has to
       describe one experiment whether it is replayed or driven live. */
    int    fi_enable;
    int    fi_have_lat, fi_have_lon;
    double fi_lat_deg, fi_lon_deg, fi_height_m;
    float  fi_stddev_m;
    char init[16];   /* "auto" | "ref" */

    /* inputs: optional per-stream CSV filename overrides, each relative to
       the config's own directory. Empty -> the conventional <stream>.csv */
    char in_imu[256];
    char in_ref[256];
    char in_gnss[256];
    char in_mag[256];
    char in_baro[256];
    char in_speed[256];

    int   automotive_mode;               /* 0/1: yaw from GNSS course over ground (REQ-NAV-034) */
    float automotive_min_speed_mps;      /* 0 -> default */
    float automotive_min_yaw_stddev_deg; /* 0 -> default */
    int   automotive_lateral_constraint;   /* 0/1: NHC lateral (REQ-NAV-077) */
    float automotive_lateral_stddev_mps;   /* 0 -> default */
    float automotive_lateral_max_yaw_rate_deg; /* 0 -> default */
    float automotive_lateral_after_sec;    /* 0 -> default, <0 -> no delay */

    float auto_init_window_sec; /* IMU leveling window for ins's auto-init
                                    bootstrap [s] (0 -> ins's built-in default,
                                    REQ-VER-017); raise for a low IMU rate so
                                    the window still contains enough samples */

    int chi2_disable;        /* 0/1: disable chi2 outlier downweighting library-wide */
    float chi2_reject_alpha; /* outlier: global chi2 gate significance threshold */

    int allow_unlimited_deadreckoning; /* 0/1: never expire the IMU-only
                                          coasting window (position drifts) */
    float max_deadreckoning_sec;       /* IMU-only coasting budget [s] */

    int baro_height_disable; /* 0/1: never select the barometric height source
                                 at bootstrap (REQ-NAV-053), even with a
                                 barometer present */

    double gyro_bias_window_sec; /* static window at the start of the trial to
                                     seed the initial gyro bias (harness-only
                                     heuristic, no library equivalent); 0 ->
                                     no seed (estimate_gyro_bias() needs >= 10
                                     samples inside the window and 0 never
                                     collects that many) */

    /* imu: */
    float gyr_psd, acc_psd;         /* (rad/s)^2/Hz, (m/s^2)^2/Hz */
    float gyr_bias_rw, acc_bias_rw; /* rad/s/sqrt(s), m/s^2/sqrt(s) */
    /* IMU calibration (REQ-NAV-037), each optional: col-major 3x3
       misalignment (all-0 -> identity) + permanent fixed bias. */
    float acc_misalignment[9]; /* accel 3x3, col-major */
    float gyr_misalignment[9]; /* gyro  3x3, col-major */
    float acc_fixed_bias[3];   /* [m/s^2] */
    float gyr_fixed_bias[3];   /* [rad/s] */
    /* Extra process-noise margin on top of the physically-derived acc/gyro
       noise (0 -> default, not zero!) */
    float pos_pred_stddev_m_sqrts;   /* [m/sqrt(s)] */
    float vel_pred_stddev_mps_sqrts; /* [m/s/sqrt(s)] */
    float rpy_pred_stddev_rad_sqrts; /* [rad/sqrt(s)] */
    /* Auto-ZUPT/ZARU pseudo-measurement stddev, 0 -> this harness's own
       default (0.05 m/s, 0.1 deg/s -- NOT ins.c's own built-in default of
       0.05 m/s, 0.5 deg/s for the rotation channel; shared with
       python/replay.py's DEFAULTS for years, see build_config() there). */
    float zero_vel_stddev_mps;
    float zero_rot_stddev_deg;
    /* Stillness detection, ONE set for the whole suite (REQ-SUITE-020,
       REQ-VER-024): forwarded to ins_options_t.auto_zupt_*, from where
       nav_suite_init() propagates it to the ARS/AHRS auto-ZARU fallback
       and, through that, to baro_alt's vertical ZUPT. Every field 0 ->
       the library's built-in default.

       Magnitude bounds: how still the IMU and a recent GNSS velocity
       observation must look before the filter injects its own
       zero-velocity/zero-rotation update. Without a recent-enough GNSS
       velocity the max_vel gate is not applied at all (never the filter's
       own state estimate -- gating on that would be circular, see
       ins_auto_zupt_velocity_gate_ok in ins.c). */
    float auto_zupt_static_gyr_deg;  /* [deg/s] */
    float auto_zupt_static_acc_mps2; /* [m/s^2] */
    float auto_zupt_max_vel_mps;     /* [m/s] */
    /* Accuracy the GNSS velocity observation above must itself meet
       (1-sigma, per axis) to be trusted for that gate: a low reported
       speed with a large uncertainty is not evidence of standstill. */
    float auto_zupt_max_vel_stddev_mps; /* [m/s] */
    /* Variance criterion of the same detector (the PRIMARY stillness
       statement, the magnitude bounds above are only loose bounds next
       to it). */
    float auto_zupt_static_gyr_stddev_deg;  /* [deg/s] */
    float auto_zupt_static_acc_stddev_mps2; /* [m/s^2] */
    /* Timing: how long the platform must look still before a trigger,
       and how often triggers may repeat. */
    float auto_zupt_dwell_sec;
    float auto_zupt_min_interval_sec;
    /* Turn the whole thing off (ins, both AHRS instances and, since it
       has no detector of its own, baro_alt). */
    int auto_zupt_disable;
    /* Narrower opt-out: only the ARS/AHRS's own velocity-blind detector
       (REQ-AHRS-017), ins keeps deciding for everyone. Set for a dataset
       whose profile the IMU alone cannot tell from a standstill --
       noiseless synthetic trajectories in particular: a constant-rate
       climb or a constant-velocity leg has zero sample variance and a
       gravity-only specific force, so the fallback fires and flattens
       exactly the trajectory under test. */
    int auto_zupt_velocity_blind_disable;
    /* Magnetometer calibration (REQ-NAV-039), optional. */
    float mag_misalignment[9]; /* mag 3x3, col-major (all-0 -> identity) */
    float mag_fixed_bias[3];   /* hard-iron [uT] */

    /* gnss: */
    float gnss_leverarm_frd[3];     /* antenna, FRD [m] */
    float pos_stddev_fallback_m[2]; /* hor/ver; replaces a zero (unknown)
                                        diagonal entry in a real gnss.csv fix,
                                        AND (aiding: ref only) is the noise
                                        assigned to the whole reference-
                                        synthesized fix, which never has a
                                        reported covariance to begin with */
    float vel_stddev_fallback_mps;  /* same, for velocity */
    float max_horizontal_pos_stddev_m; /* ins noise-shutdown gates */
    float max_vertical_pos_stddev_m;   /* (0 -> ins's own default) */
    float gnss_delay_ms;               /* assumed fixed processing/telemetry latency of
                                           every fix, applied uniformly regardless of
                                           aiding source (0 -> none, REQ-VER-008) */
    float max_horizontal_vel_stddev_mps; /* 0 -> ins's own default */
    float max_vertical_vel_stddev_mps;   /* 0 -> ins's own default */
    /* Solution-mode gates (REQ-NAV-051/052), each 0 -> ins's own default:
       the strict set that admits the 3D solution and the loose set that
       gives it up, plus their dwells. */
    float start_max_horizontal_pos_stddev_m;
    float start_max_vertical_pos_stddev_m;
    float start_max_horizontal_vel_stddev_mps;
    float start_max_vertical_vel_stddev_mps;
    float stop_max_horizontal_pos_stddev_m;
    float stop_max_vertical_pos_stddev_m;
    float stop_max_horizontal_vel_stddev_mps;
    float stop_max_vertical_vel_stddev_mps;
    float init_dwell_sec;
    int   init_dwell_disable;
    float stop_dwell_sec;
    int   stop_disable;
    /* GNSS position decimation (REQ-NAV-063): fuse the position on every
       Nth epoch that offers a usable position AND velocity, the velocity
       alone on the other N-1. <= 1 -> off, 0 -> ins's own default. */
    int   pos_decimation;
    int   min_delay_ms;             /* min time between two fused GNSS epochs [ms]
                                       (0 -> default, < 0 -> no limit) */
    /* GNSS covariance conditioning (REQ-NAV-038), all optional. */
    /* 0 -> the fixes are still read, counted and available for the score,
       but none of them is fed to the filter. One key instead of gates set
       to reject everything, for a run that is deliberately GNSS-free (an
       inertial-only test, an A/B against the same recording with GNSS on).
       tools/insrcv.c has the same key. */
    int   gnss_enable;
    float pos_cov_scale;            /* multiplies GNSS pos stddev (0 -> 1) */
    float pos_cov_scale_height;     /* multiplies GNSS pos *height* stddev (0 -> 1), REQ-NAV-041 */
    float vel_cov_scale;            /* multiplies GNSS vel stddev (0 -> 1) */
    float pos_stddev_floor_hor_m;   /* min horiz. pos stddev [m]   (0 -> default, <0 -> off) */
    float pos_stddev_floor_ver_m;   /* min vert.  pos stddev [m]   (0 -> default, <0 -> off) */
    float vel_stddev_floor_hor_mps; /* min horiz. vel stddev [m/s] (0 -> default, <0 -> off) */
    float vel_stddev_floor_ver_mps; /* min vert.  vel stddev [m/s] (0 -> default, <0 -> off) */
    float pos_stddev_cap_hor_m;     /* max horiz. pos stddev [m]   (0 -> default, <0 -> off) */
    float pos_stddev_cap_ver_m;     /* max vert.  pos stddev [m]   (0 -> default, <0 -> off) */
    float vel_stddev_cap_hor_mps;   /* max horiz. vel stddev [m/s] (0 -> default, <0 -> off) */
    float vel_stddev_cap_ver_mps;   /* max vert.  vel stddev [m/s] (0 -> default, <0 -> off) */
    float acc_envelope_tau_sec;     /* accuracy-envelope decay [s] (0 -> default, <0 -> off) */
    /* Manoeuvre vel noise, per [m/s^2] of ANTENNA acceleration (REQ-NAV-076:
       body acceleration plus the centripetal term of the lever arm). */
    float vel_noise_acc_scale_hor;  /* manoeuvre vel noise on N,E [m/s per m/s^2] (0 -> off) */
    float vel_noise_acc_scale_ver;  /* manoeuvre vel noise on D   [m/s per m/s^2] (0 -> off) */
    float vel_noise_acc_window_sec; /* averaging window for it [s] (0 -> default, <0 -> off) */

    /* init_stddev: initial-state uncertainty (ins_init_t's
       *_init_stddev_*), each 0 -> the built-in default (REQ-VER-010).
       Applies to BOTH init modes: ins_finalize_init() (ins.c) is the
       shared covariance setup for manual (init: ref) AND auto-init --
       the auto-init leveling/fix window only supplies the initial
       state itself (position/attitude), not its uncertainty. Key names
       mirror ins_init_t's own field names (same convention as
       python/examples/runner.yaml's `filter:` section, not an
       abbreviated scheme of their own). */
    float init_pos_stddev_m;
    float init_vel_stddev_mps;
    float init_rpy_stddev_deg;
    float init_yaw_stddev_deg; /* 0 -> falls back to init_rpy_stddev_deg,
                                   like ins's own rpy_init_stddev_rad[0..1]/[2] */
    float init_acc_bias_stddev_mps2;
    float init_gyr_bias_stddev_deg;

    /* mag: */
    int    mag_enable;
    float  mag_stddev_ut;     /* ins mag fusion noise (0 -> ins default) */
    int    mag_min_delay_ms;  /* fusion rate limit, 0 -> ins default,
                                 negative -> fuse every sample */
    double wmm_year;          /* magnetic model epoch (0 -> no model) */
    int    mag_estimate_bias; /* 0/1: ins 18-state mode, online hard-iron
                                  bias estimate (REQ-NAV-029) */
    /* The two knobs of that estimate, both 0 -> ins default. Exposed
       because they are what decides whether the 18-state mode helps on a
       given recording: the initial sigma says how much hard iron the
       filter should be prepared to find, and the random walk how long the
       estimate keeps listening once it has settled. Neither has a value
       that is right everywhere -- a road recording whose magnetometer was
       calibrated in place wants a different prior from a box just bolted
       into an unknown vehicle -- so they belong in the dataset's own
       config rather than in a compiled-in constant. */
    float  mag_bias_init_ut;      /* [uT] initial hard-iron 1-sigma      */
    float  mag_bias_rw_ut_sqrts;  /* [uT/sqrt(s)] hard-iron random walk  */

    /* speed: scalar ground speed (REQ-NAV-068), e.g. an OBD-II vehicle
       speed. Uncertainty and delay are constants from the config rather
       than per-sample columns, so one place describes the dataset. */
    int   speed_enable;
    float speed_scale;       /* multiplies the reported speed (0 -> 1.0) */
    float speed_stddev_mps;  /* per-sample 1-sigma [m/s] (0 -> default) */
    float speed_stddev_rel;  /* speed-proportional 1-sigma (0 -> default) */
    float speed_min_mps;     /* below this FILTERED speed a sample is
                                skipped (0 -> default) */
    int   speed_delay_ms;    /* how old a sample is at its timestamp */

    /* baro: */
    int   baro_enable;
    float baro_stddev_m;              /* 0 -> consumer default */
    float baro_acc_bias_rw;           /* baro_alt acc-bias drift density
                                         [m/s^2/sqrt(Hz)] (0 -> baro_alt default) */
    float baro_acc_bias_init_mps2;    /* baro_alt initial acc-bias stddev [m/s^2]
                                    (0 -> baro_alt default) */
    float baro_h_process_noise;       /* baro_alt direct height process-noise
                                       density [m/sqrt(Hz)] (0 -> baro_alt default) */
    float baro_acc_noise_mps2_sqrthz; /* baro_alt's own direct accel-noise
                                 density [m/s^2/sqrt(Hz)] (0 -> baro_alt default) */
    /* local-height/GNSS-ellipsoid offset filter tuning (all 0 ->
       local_gnss_alt defaults). Raise local_gnss_rw_stddev_mps for
       missions with large altitude excursions (e.g. a soaring glider):
       the offset absorbs the ISA-model error, which grows with the
       excursion and can outrun the default random walk, tuned for a
       multi-metre swing. */
    float local_gnss_rw_stddev_mps;
    float local_gnss_chi2_threshold;
    float local_gnss_min_update_interval_sec;
    float local_gnss_stddev_inflation_factor;

    /* ahrs: SHARED ARS/AHRS noise model override (0 -> derived from imu:
       above, this harness's own long-standing default, see below). */
    float ahrs_gyr_noise_psd;            /* [rad/s/sqrt(Hz)] */
    float ahrs_gyr_bias_rw;              /* [rad/s^2/sqrt(Hz)] */
    float ahrs_acc_noise_mps2;           /* [m/s^2] */
    float ahrs_gyr_bias_init_stddev_deg; /* [deg/s], 0 -> the static-window
                                   seed's own stddev (or ahrs.c's built-in
                                   default if the window found nothing) */

    /* init_hint: known initial roll/pitch and/or yaw for ins's auto-init
       bootstrap (init: auto only), see nav_suite_set_init_att_hint().
       Roll/pitch used together, yaw independent; each stddev 0 -> no hint. */
    float init_hint_roll_deg;
    float init_hint_pitch_deg;
    float init_hint_rpy_stddev_deg;
    float init_hint_yaw_deg;
    float init_hint_yaw_stddev_deg;

    /* score: */
    float  score_leverarm_frd[3]; /* truth point vs. IMU, FRD [m] */
    int    score_ahrs;            /* 1: also score the ARS + its gates */
    int    score_attitude;        /* 0: ref.csv carries no attitude truth ->
                                     report neither the ins attitude errors
                                     nor their gates. Not the same as a wide
                                     limit: a number scored against a
                                     placeholder reference reads as a passed
                                     check no matter how large the limit is. */
    double warmup_sec;            /* scoring starts this long after the
                                      first fix; used ONLY for scoring,
                                      not by the filter itself */
    int    min_epochs;
    double lim_att_bias_deg, lim_att_std_deg;
    double lim_yaw_bias_deg, lim_yaw_std_deg;
    double lim_pos_rms_m;
    double lim_baro_rms_m;  /* gate on baro rel-height rms;      0 -> not gated */
    double lim_baro_bias_m; /* gate on baro rel-height |mean|;   0 -> not gated */
    double lim_baro_max_m;  /* gate on baro rel-height max |err|; 0 -> not gated */
    double lim_ellipsoid_rms_m;  /* gate on nav_suite_get_height_ellipsoid()
                                     rms vs ref (absolute, REQ-NAV-053ff);
                                     0 -> not gated */
    double lim_ellipsoid_bias_m; /* same, |mean|; 0 -> not gated */
    double lim_ellipsoid_max_m;  /* same, max |err|; 0 -> not gated */
    double lim_ars_att_bias_deg;
    double lim_ars_roll_std_deg, lim_ars_pitch_std_deg;
    double lim_ars_yaw_drift_deg_min;
    /* Coasting re-acquisition (REQ-VER-029): every gap in the aiding file
       longer than coast_gap_min_sec is scored by the position error at the
       first reference epoch after it. 0 -> not measured / not gated. */
    double coast_gap_min_sec;
    double lim_coast_exit_err_m;
    /* Time of validity of a reference row (REQ-VER-030): the row timestamped
       T describes the state at T - ref_delay_ms. Set it when ref.csv comes
       out of the same receiver output as the aiding and that output carries
       a latency the filter is told about via gnss: delay_ms. */
    double ref_delay_ms;
} replay_cfg_t;

/* Apply one "section.key: value" pair. Returns -1 if the key is not
   known, which load_config turns into a hard error (REQ-VER-025): a
   silently ignored key is indistinguishable from a key that had no
   effect, and a mistyped or renamed tuning parameter then looks like the
   library ignoring the configuration. Returns -2 for a known key whose
   value is not the list it has to be (REQ-VER-028), the same argument one
   level down: a half-read lever arm or misalignment matrix would leave
   the harness running on numbers nobody wrote. */
static int cfg_set(replay_cfg_t* c, const char* sec, const char* key, const char* val)
{
    char full[128];
    snprintf(full, sizeof(full), "%s%s%s", sec, sec[0] ? "." : "", key);
    const double d = strtod(val, (char**)0);

    if (!strcmp(full, "name")) { snprintf(c->name, sizeof(c->name), "%s", val); }
    else if (!strcmp(full, "aiding")) { snprintf(c->aiding, sizeof(c->aiding), "%s", val); }
    else if (!strcmp(full, "free_inertial_start.enable")) { c->fi_enable = (int)d; }
    else if (!strcmp(full, "free_inertial_start.lat_deg"))
    {
        c->fi_lat_deg  = d;
        c->fi_have_lat = 1;
    }
    else if (!strcmp(full, "free_inertial_start.lon_deg"))
    {
        c->fi_lon_deg  = d;
        c->fi_have_lon = 1;
    }
    else if (!strcmp(full, "free_inertial_start.height_m")) { c->fi_height_m = d; }
    else if (!strcmp(full, "free_inertial_start.stddev_m"))
    {
        if (d > 0.0) { c->fi_stddev_m = (float)d; }
    }
    else if (!strcmp(full, "init")) { snprintf(c->init, sizeof(c->init), "%s", val); }
    else if (!strcmp(full, "inputs.imu")) { snprintf(c->in_imu, sizeof(c->in_imu), "%s", val); }
    else if (!strcmp(full, "inputs.ref")) { snprintf(c->in_ref, sizeof(c->in_ref), "%s", val); }
    else if (!strcmp(full, "inputs.gnss")) { snprintf(c->in_gnss, sizeof(c->in_gnss), "%s", val); }
    else if (!strcmp(full, "inputs.mag")) { snprintf(c->in_mag, sizeof(c->in_mag), "%s", val); }
    else if (!strcmp(full, "inputs.baro")) { snprintf(c->in_baro, sizeof(c->in_baro), "%s", val); }
    else if (!strcmp(full, "inputs.speed")) { snprintf(c->in_speed, sizeof(c->in_speed), "%s", val); }
    else if (!strcmp(full, "automotive_mode")) { c->automotive_mode = (int)d; }
    else if (!strcmp(full, "automotive_min_speed_mps")) { c->automotive_min_speed_mps = (float)d; }
    else if (!strcmp(full, "automotive_lateral_constraint"))
    {
        c->automotive_lateral_constraint = (int)d;
    }
    else if (!strcmp(full, "automotive_lateral_stddev_mps"))
    {
        c->automotive_lateral_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "automotive_lateral_max_yaw_rate_deg"))
    {
        c->automotive_lateral_max_yaw_rate_deg = (float)d;
    }
    else if (!strcmp(full, "automotive_lateral_after_sec"))
    {
        c->automotive_lateral_after_sec = (float)d;
    }
    else if (!strcmp(full, "automotive_min_yaw_stddev_deg"))
    {
        c->automotive_min_yaw_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "auto_init_window_sec")) { c->auto_init_window_sec = (float)d; }
    else if (!strcmp(full, "chi2_disable")) { c->chi2_disable = (int)d; }
    else if (!strcmp(full, "chi2_reject_alpha")) { c->chi2_reject_alpha = (float)d; }
    else if (!strcmp(full, "allow_unlimited_deadreckoning"))
    {
        c->allow_unlimited_deadreckoning = (int)d;
    }
    else if (!strcmp(full, "max_deadreckoning_sec")) { c->max_deadreckoning_sec = (float)d; }
    else if (!strcmp(full, "baro_height_disable")) { c->baro_height_disable = (int)d; }
    else if (!strcmp(full, "gyro_bias_window_sec")) { c->gyro_bias_window_sec = d; }
    else if (!strcmp(full, "imu.gyr_psd")) { c->gyr_psd = (float)d; }
    else if (!strcmp(full, "imu.acc_psd")) { c->acc_psd = (float)d; }
    else if (!strcmp(full, "imu.gyr_bias_rw")) { c->gyr_bias_rw = (float)d; }
    else if (!strcmp(full, "imu.acc_bias_rw")) { c->acc_bias_rw = (float)d; }
    else if (!strcmp(full, "imu.acc_misalignment")) { if (mini_yaml_list(val, c->acc_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "imu.gyr_misalignment")) { if (mini_yaml_list(val, c->gyr_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "imu.acc_fixed_bias")) { if (mini_yaml_list(val, c->acc_fixed_bias, 3) != 0) return -2; }
    else if (!strcmp(full, "imu.gyr_fixed_bias")) { if (mini_yaml_list(val, c->gyr_fixed_bias, 3) != 0) return -2; }
    else if (!strcmp(full, "imu.pos_pred_stddev_m_sqrts"))
    {
        c->pos_pred_stddev_m_sqrts = (float)d;
    }
    else if (!strcmp(full, "imu.vel_pred_stddev_mps_sqrts"))
    {
        c->vel_pred_stddev_mps_sqrts = (float)d;
    }
    else if (!strcmp(full, "imu.rpy_pred_stddev_rad_sqrts"))
    {
        c->rpy_pred_stddev_rad_sqrts = (float)d;
    }
    else if (!strcmp(full, "imu.zero_vel_stddev_mps")) { c->zero_vel_stddev_mps = (float)d; }
    else if (!strcmp(full, "imu.zero_rot_stddev_deg")) { c->zero_rot_stddev_deg = (float)d; }
    else if (!strcmp(full, "imu.auto_zupt_static_gyr_deg"))
    {
        c->auto_zupt_static_gyr_deg = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_static_acc_mps2"))
    {
        c->auto_zupt_static_acc_mps2 = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_max_vel_mps")) { c->auto_zupt_max_vel_mps = (float)d; }
    else if (!strcmp(full, "imu.auto_zupt_max_vel_stddev_mps"))
    {
        c->auto_zupt_max_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_static_gyr_stddev_deg"))
    {
        c->auto_zupt_static_gyr_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_static_acc_stddev_mps2"))
    {
        c->auto_zupt_static_acc_stddev_mps2 = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_dwell_sec")) { c->auto_zupt_dwell_sec = (float)d; }
    else if (!strcmp(full, "imu.auto_zupt_min_interval_sec"))
    {
        c->auto_zupt_min_interval_sec = (float)d;
    }
    else if (!strcmp(full, "imu.auto_zupt_disable")) { c->auto_zupt_disable = (int)d; }
    else if (!strcmp(full, "imu.auto_zupt_velocity_blind_disable"))
    {
        c->auto_zupt_velocity_blind_disable = (int)d;
    }
    else if (!strcmp(full, "gnss.leverarm_frd")) { if (mini_yaml_list(val, c->gnss_leverarm_frd, 3) != 0) return -2; }
    else if (!strcmp(full, "gnss.pos_stddev_fallback_m"))
    {
        if (mini_yaml_list(val, c->pos_stddev_fallback_m, 2) != 0) return -2;
    }
    else if (!strcmp(full, "gnss.vel_stddev_fallback_mps"))
    {
        c->vel_stddev_fallback_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.delay_ms")) { c->gnss_delay_ms = (float)d; }
    else if (!strcmp(full, "gnss.max_horizontal_pos_stddev_m"))
    {
        c->max_horizontal_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.max_vertical_pos_stddev_m"))
    {
        c->max_vertical_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.max_horizontal_vel_stddev_mps"))
    {
        c->max_horizontal_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.max_vertical_vel_stddev_mps"))
    {
        c->max_vertical_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.start_max_horizontal_pos_stddev_m"))
    {
        c->start_max_horizontal_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.start_max_vertical_pos_stddev_m"))
    {
        c->start_max_vertical_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.start_max_horizontal_vel_stddev_mps"))
    {
        c->start_max_horizontal_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.start_max_vertical_vel_stddev_mps"))
    {
        c->start_max_vertical_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.stop_max_horizontal_pos_stddev_m"))
    {
        c->stop_max_horizontal_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.stop_max_vertical_pos_stddev_m"))
    {
        c->stop_max_vertical_pos_stddev_m = (float)d;
    }
    else if (!strcmp(full, "gnss.stop_max_horizontal_vel_stddev_mps"))
    {
        c->stop_max_horizontal_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.stop_max_vertical_vel_stddev_mps"))
    {
        c->stop_max_vertical_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.init_dwell_sec")) { c->init_dwell_sec = (float)d; }
    else if (!strcmp(full, "gnss.init_dwell_disable")) { c->init_dwell_disable = (int)d; }
    else if (!strcmp(full, "gnss.stop_dwell_sec")) { c->stop_dwell_sec = (float)d; }
    else if (!strcmp(full, "gnss.stop_disable")) { c->stop_disable = (int)d; }
    else if (!strcmp(full, "gnss.pos_decimation")) { c->pos_decimation = (int)d; }
    else if (!strcmp(full, "gnss.min_delay_ms")) { c->min_delay_ms = (int)d; }
    else if (!strcmp(full, "gnss.enable")) { c->gnss_enable = (int)d; }
    else if (!strcmp(full, "gnss.pos_cov_scale")) { c->pos_cov_scale = (float)d; }
    else if (!strcmp(full, "gnss.pos_cov_scale_height")) { c->pos_cov_scale_height = (float)d; }
    else if (!strcmp(full, "gnss.vel_cov_scale")) { c->vel_cov_scale = (float)d; }
    else if (!strcmp(full, "gnss.pos_stddev_floor_hor_m")) { c->pos_stddev_floor_hor_m = (float)d; }
    else if (!strcmp(full, "gnss.pos_stddev_floor_ver_m")) { c->pos_stddev_floor_ver_m = (float)d; }
    else if (!strcmp(full, "gnss.vel_stddev_floor_hor_mps"))
    {
        c->vel_stddev_floor_hor_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_stddev_floor_ver_mps"))
    {
        c->vel_stddev_floor_ver_mps = (float)d;
    }
    else if (!strcmp(full, "gnss.pos_stddev_cap_hor_m")) { c->pos_stddev_cap_hor_m = (float)d; }
    else if (!strcmp(full, "gnss.pos_stddev_cap_ver_m")) { c->pos_stddev_cap_ver_m = (float)d; }
    else if (!strcmp(full, "gnss.vel_stddev_cap_hor_mps")) { c->vel_stddev_cap_hor_mps = (float)d; }
    else if (!strcmp(full, "gnss.vel_stddev_cap_ver_mps")) { c->vel_stddev_cap_ver_mps = (float)d; }
    else if (!strcmp(full, "gnss.acc_envelope_tau_sec")) { c->acc_envelope_tau_sec = (float)d; }
    else if (!strcmp(full, "gnss.vel_noise_acc_scale_hor"))
    {
        c->vel_noise_acc_scale_hor = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_noise_acc_scale_ver"))
    {
        c->vel_noise_acc_scale_ver = (float)d;
    }
    else if (!strcmp(full, "gnss.vel_noise_acc_window_sec"))
    {
        c->vel_noise_acc_window_sec = (float)d;
    }
    else if (!strcmp(full, "init_stddev.pos_init_stddev_m")) { c->init_pos_stddev_m = (float)d; }
    else if (!strcmp(full, "init_stddev.vel_init_stddev_mps"))
    {
        c->init_vel_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "init_stddev.rpy_init_stddev_rad_deg"))
    {
        c->init_rpy_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "init_stddev.yaw_init_stddev_rad_deg"))
    {
        c->init_yaw_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "init_stddev.acc_bias_init_stddev_mps2"))
    {
        c->init_acc_bias_stddev_mps2 = (float)d;
    }
    else if (!strcmp(full, "init_stddev.gyr_bias_init_stddev_rps_deg"))
    {
        c->init_gyr_bias_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "mag.enable")) { c->mag_enable = (int)d; }
    else if (!strcmp(full, "mag.stddev_ut")) { c->mag_stddev_ut = (float)d; }
    /* Unclamped: ins reads 0 as "use my default" and negative as "no rate
       limit", and both have to survive the config. */
    else if (!strcmp(full, "mag.min_delay_ms")) { c->mag_min_delay_ms = (int)d; }
    else if (!strcmp(full, "mag.misalignment")) { if (mini_yaml_list(val, c->mag_misalignment, 9) != 0) return -2; }
    else if (!strcmp(full, "mag.fixed_bias")) { if (mini_yaml_list(val, c->mag_fixed_bias, 3) != 0) return -2; }
    else if (!strcmp(full, "mag.wmm_year")) { c->wmm_year = d; }
    else if (!strcmp(full, "mag.estimate_bias")) { c->mag_estimate_bias = (int)d; }
    else if (!strcmp(full, "mag.bias_init_ut")) { c->mag_bias_init_ut = (float)d; }
    else if (!strcmp(full, "mag.bias_rw_ut_sqrts")) { c->mag_bias_rw_ut_sqrts = (float)d; }
    else if (!strcmp(full, "speed.enable")) { c->speed_enable = (int)d; }
    else if (!strcmp(full, "speed.scale")) { c->speed_scale = (float)d; }
    else if (!strcmp(full, "speed.stddev_mps")) { c->speed_stddev_mps = (float)d; }
    else if (!strcmp(full, "speed.stddev_rel")) { c->speed_stddev_rel = (float)d; }
    else if (!strcmp(full, "speed.min_speed_mps")) { c->speed_min_mps = (float)d; }
    else if (!strcmp(full, "speed.delay_ms")) { c->speed_delay_ms = (int)d; }
    else if (!strcmp(full, "baro.enable")) { c->baro_enable = (int)d; }
    else if (!strcmp(full, "baro.stddev_m")) { c->baro_stddev_m = (float)d; }
    else if (!strcmp(full, "baro.acc_bias_rw")) { c->baro_acc_bias_rw = (float)d; }
    else if (!strcmp(full, "baro.acc_bias_init_mps2")) { c->baro_acc_bias_init_mps2 = (float)d; }
    else if (!strcmp(full, "baro.h_process_noise")) { c->baro_h_process_noise = (float)d; }
    else if (!strcmp(full, "baro.acc_noise_mps2_sqrthz"))
    {
        c->baro_acc_noise_mps2_sqrthz = (float)d;
    }
    else if (!strcmp(full, "baro.local_gnss_rw_stddev_mps"))
    {
        c->local_gnss_rw_stddev_mps = (float)d;
    }
    else if (!strcmp(full, "baro.local_gnss_chi2_threshold"))
    {
        c->local_gnss_chi2_threshold = (float)d;
    }
    else if (!strcmp(full, "baro.local_gnss_min_update_interval_sec"))
    {
        c->local_gnss_min_update_interval_sec = (float)d;
    }
    else if (!strcmp(full, "baro.local_gnss_stddev_inflation_factor"))
    {
        c->local_gnss_stddev_inflation_factor = (float)d;
    }
    else if (!strcmp(full, "ahrs.gyr_noise_psd")) { c->ahrs_gyr_noise_psd = (float)d; }
    else if (!strcmp(full, "ahrs.gyr_bias_rw")) { c->ahrs_gyr_bias_rw = (float)d; }
    else if (!strcmp(full, "ahrs.acc_noise_mps2")) { c->ahrs_acc_noise_mps2 = (float)d; }
    else if (!strcmp(full, "ahrs.gyr_bias_init_stddev_rps_deg"))
    {
        c->ahrs_gyr_bias_init_stddev_deg = (float)d;
    }
    else if (!strcmp(full, "init_hint.roll_deg")) { c->init_hint_roll_deg = (float)d; }
    else if (!strcmp(full, "init_hint.pitch_deg")) { c->init_hint_pitch_deg = (float)d; }
    else if (!strcmp(full, "init_hint.rpy_stddev_deg")) { c->init_hint_rpy_stddev_deg = (float)d; }
    else if (!strcmp(full, "init_hint.yaw_deg")) { c->init_hint_yaw_deg = (float)d; }
    else if (!strcmp(full, "init_hint.yaw_stddev_deg")) { c->init_hint_yaw_stddev_deg = (float)d; }
    else if (!strcmp(full, "score.leverarm_frd")) { if (mini_yaml_list(val, c->score_leverarm_frd, 3) != 0) return -2; }
    else if (!strcmp(full, "score.ahrs")) { c->score_ahrs = (int)d; }
    else if (!strcmp(full, "score.attitude")) { c->score_attitude = (int)d; }
    else if (!strcmp(full, "score.warmup_sec")) { c->warmup_sec = d; }
    else if (!strcmp(full, "score.min_epochs")) { c->min_epochs = (int)d; }
    else if (!strcmp(full, "score.lim_att_bias_deg")) { c->lim_att_bias_deg = d; }
    else if (!strcmp(full, "score.lim_att_std_deg")) { c->lim_att_std_deg = d; }
    else if (!strcmp(full, "score.lim_yaw_bias_deg")) { c->lim_yaw_bias_deg = d; }
    else if (!strcmp(full, "score.lim_yaw_std_deg")) { c->lim_yaw_std_deg = d; }
    else if (!strcmp(full, "score.lim_pos_rms_m")) { c->lim_pos_rms_m = d; }
    else if (!strcmp(full, "score.lim_baro_rms_m")) { c->lim_baro_rms_m = d; }
    else if (!strcmp(full, "score.lim_baro_bias_m")) { c->lim_baro_bias_m = d; }
    else if (!strcmp(full, "score.lim_baro_max_m")) { c->lim_baro_max_m = d; }
    else if (!strcmp(full, "score.lim_ellipsoid_rms_m")) { c->lim_ellipsoid_rms_m = d; }
    else if (!strcmp(full, "score.lim_ellipsoid_bias_m")) { c->lim_ellipsoid_bias_m = d; }
    else if (!strcmp(full, "score.lim_ellipsoid_max_m")) { c->lim_ellipsoid_max_m = d; }
    else if (!strcmp(full, "score.lim_ars_att_bias_deg")) { c->lim_ars_att_bias_deg = d; }
    else if (!strcmp(full, "score.lim_ars_roll_std_deg")) { c->lim_ars_roll_std_deg = d; }
    else if (!strcmp(full, "score.lim_ars_pitch_std_deg")) { c->lim_ars_pitch_std_deg = d; }
    else if (!strcmp(full, "score.lim_ars_yaw_drift_deg_min")) { c->lim_ars_yaw_drift_deg_min = d; }
    else if (!strcmp(full, "score.coast_gap_min_sec")) { c->coast_gap_min_sec = d; }
    else if (!strcmp(full, "score.ref_delay_ms")) { c->ref_delay_ms = d; }
    else if (!strcmp(full, "score.lim_coast_exit_err_m")) { c->lim_coast_exit_err_m = d; }
    /* Known to the schema, owned by a DIFFERENT consumer of the SAME
       config.yaml. One dataset directory is read by more than one tool, and
       REQ-VER-025 makes the schema shared, not the interpretation - so these
       are accepted and ignored here rather than rejected. Rejecting them
       would force a dataset to pick one of its tools; deleting them from the
       file to satisfy this parser would silently disarm the other tool.
       python/replay.py carries the identical list (FOREIGN_SECTIONS + the
       score key below).

         score.lim_groves_pos_rms_factor  datasets/check_simulated.py's gate
                                          "ins pos rms <= this x the Groves
                                          textbook filter's own rms"; needs
                                          ref_groves_kf_sol.csv, which only
                                          the simulated datasets carry.
         origin.*                         local-frame anchor for a platform
                                          with no absolute position source,
         crazyflie.*                      link settings for the same reader.
                                          Both python/crazyflie_reader.py.

       A section only earns a place here once a tool actually reads it:
       something no tool reads is not a foreign section, it is documentation,
       and belongs in a YAML comment no parser has to know about. */
    else if (!strcmp(full, "score.lim_groves_pos_rms_factor")) { /* check_simulated.py */ }
    else if (!strcmp(sec, "origin") || !strcmp(sec, "crazyflie"))
    {
        /* foreign section, see above */
    }
    else { return -1; }
    return 0;
}

typedef struct
{
    replay_cfg_t* c;
    const char*   path;
} cfg_parse_ctx_t;

/* mini_yaml_parse() callback: apply one key, and turn an unknown key into
 * the hard error REQ-VER-025 requires (see cfg_set's own comment). */
static int cfg_set_cb(void* ctx_, const char* sec, const char* key, const char* val)
{
    cfg_parse_ctx_t* ctx = (cfg_parse_ctx_t*)ctx_;
    const int        rc  = cfg_set(ctx->c, sec, key, val);
    if (rc == -2)
    {
        fprintf(stderr, "%s: config key '%s%s%s' is not a list of numbers: %s\n", ctx->path, sec,
                sec[0] ? "." : "", key, val);
        return -1;
    }
    if (rc != 0)
    {
        fprintf(stderr, "%s: unknown config key '%s%s%s'\n", ctx->path, sec, sec[0] ? "." : "", key);
        return -1;
    }
    return 0;
}

static int load_config(const char* path, replay_cfg_t* c)
{
    memset(c, 0, sizeof(*c));
    /* Harness defaults (overridden by the file). */
    snprintf(c->aiding, sizeof(c->aiding), "gnss");
    c->gnss_enable = 1;
    c->fi_stddev_m = 2.0f; /* as in insrcv: at the order of the 3D entry gate */
    snprintf(c->init, sizeof(c->init), "auto");
    snprintf(c->in_imu, sizeof(c->in_imu), "imu.csv");
    snprintf(c->in_ref, sizeof(c->in_ref), "ref.csv");
    snprintf(c->in_gnss, sizeof(c->in_gnss), "gnss.csv");
    snprintf(c->in_mag, sizeof(c->in_mag), "mag.csv");
    snprintf(c->in_baro, sizeof(c->in_baro), "baro.csv");
    snprintf(c->in_speed, sizeof(c->in_speed), "speed.csv");
    c->warmup_sec     = 60.0;
    c->score_attitude = 1; /* opt OUT, so every existing dataset keeps its
                              attitude gates without touching its config */
    /* gyro_bias_window_sec, gnss.pos_stddev_fallback_m/vel_stddev_fallback_mps
       (REQUIRED under aiding: ref, see the validation below) and
       max_horizontal/vertical_pos/vel_stddev_m(ps) are all deliberately left
       at 0 (memset above): none of them has a value this harness can invent
       on a dataset's behalf (see the comment on gnss_max_horizontal_pos_stddev_m
       further down) -- a dataset that relies on one states it explicitly in
       its own config.yaml. */

    cfg_parse_ctx_t ctx = {c, path};
    if (mini_yaml_parse(path, &ctx, cfg_set_cb) != 0) return -1;

    if (!(c->gyr_psd > 0.0f && c->acc_psd > 0.0f))
    {
        fprintf(stderr, "%s: missing imu noise model (imu: gyr_psd/acc_psd)\n", path);
        return -1;
    }
    if (!(c->warmup_sec > 0.0) || c->min_epochs <= 0)
    {
        fprintf(stderr, "%s: missing score: warmup_sec or score: min_epochs\n", path);
        return -1;
    }
    if ((c->ref_delay_ms > 0.0 || c->ref_delay_ms < 0.0) && !strcmp(c->aiding, "ref"))
    {
        /* There the reference IS the aiding, so a latency belongs in
           gnss: delay_ms and shifting the reference would move the
           measurements with it (REQ-VER-030). */
        fprintf(stderr, "%s: score: ref_delay_ms cannot be used with aiding: ref\n", path);
        return -1;
    }
    if (c->fi_enable && !(c->fi_have_lat && c->fi_have_lon))
    {
        fprintf(stderr, "%s: free_inertial_start needs lat_deg and lon_deg:"
                        " the whole point is the origin you supply.\n", path);
        return -1;
    }
    if (!strcmp(c->aiding, "ref") &&
        !(c->pos_stddev_fallback_m[0] > 0.0f && c->pos_stddev_fallback_m[1] > 0.0f &&
          c->vel_stddev_fallback_mps > 0.0f))
    {
        fprintf(stderr,
                "%s: aiding: ref requires gnss: pos_stddev_fallback_m/vel_stddev_fallback_mps "
                "(the reference-synthesized fix never has a reported covariance of its own, "
                "so this is the only noise it gets; no built-in default)\n",
                path);
        return -1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Input CSVs
 * ------------------------------------------------------------------------- */

typedef struct
{
    int64_t t_us;
    double  lat_rad, lon_rad, h_m;
    float   roll_rad, pitch_rad, yaw_rad;
    float   vel_ned[3];
} ref_epoch_t;

typedef struct
{
    int64_t t_us;
    double  lat_rad, lon_rad, h_m;
    float   Qpos_ned[3 * 3]; /* full covariance, column-major [m^2] */
    float   vel_ned[3];
    float   Qvel_ned[3 * 3]; /* full covariance, column-major [(m/s)^2] */
    int     vel_ok;
} gnss_epoch_t;

typedef struct
{
    int64_t t_us;
    float   mag_ut[3]; /* body FRD [uT] */
} mag_epoch_t;

typedef struct
{
    int64_t t_us;
    float   pressure_pa;
} baro_epoch_t;

typedef struct
{
    int64_t t_us;
    float   speed_mps;
} speed_epoch_t;

typedef struct
{
    double sum, sum2, max_abs;
    int    n;
} stat_t;

static void stat_add(stat_t* s, double err)
{
    s->sum += err;
    s->sum2 += err * err;
    if (fabs(err) > s->max_abs) s->max_abs = fabs(err);
    s->n++;
}

static double stat_mean(const stat_t* s) { return s->n ? s->sum / s->n : 0.0; }
static double stat_rms(const stat_t* s) { return s->n ? sqrt(s->sum2 / s->n) : 0.0; }
static double stat_std(const stat_t* s)
{
    if (s->n < 2) return 0.0;
    const double m = stat_mean(s);
    return sqrt(s->sum2 / s->n - m * m);
}

static double wrap_pi_d(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static void stat_print(const char* name, const stat_t* s, const char* unit)
{
    printf("  %-24s mean %+8.3f  std %7.3f  rms %7.3f  max %7.3f  %s"
           "  (n=%d)\n",
           name, stat_mean(s), stat_std(s), stat_rms(s), s->max_abs, unit, s->n);
}

/* Ensure a per-stream buffer can hold at least n+1 elements of `elem` bytes,
 * doubling its capacity (*cap) when full. Returns the (possibly moved) buffer.
 * Aborts the program on allocation failure -- replay is a host-side test
 * harness (not the embedded library), so bailing out is fine. */
static void* grow_or_die(void* arr, int n, int* cap, size_t elem, const char* path)
{
    if (n < *cap) { return arr; }
    const int newcap = *cap ? *cap * 2 : 4096;
    void*     p      = realloc(arr, (size_t)newcap * elem);
    if (!p)
    {
        fprintf(stderr, "out of memory loading %s (%d epochs)\n", path, n);
        exit(EXIT_FAILURE);
    }
    *cap = newcap;
    return p;
}

static int load_ref(const char* path, ref_epoch_t** out)
{
    *out    = NULL;
    FILE* f = fopen(path, "r");
    if (!f) { return -1; }
    char         line[512];
    int          n   = 0;
    int          cap = 0;
    ref_epoch_t* ref = NULL;
    while (fgets(line, sizeof(line), f))
    {
        long long t;
        double    lat, lon, h, r, p, y;
        float     vn, ve, vd;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%f,%f,%f", &t, &lat, &lon, &h, &r, &p, &y,
                   &vn, &ve, &vd) != 10)
        {
            continue;
        }
        ref               = grow_or_die(ref, n, &cap, sizeof(*ref), path);
        ref[n].t_us       = (int64_t)t;
        ref[n].lat_rad    = lat * M_PI / 180.0;
        ref[n].lon_rad    = lon * M_PI / 180.0;
        ref[n].h_m        = h;
        ref[n].roll_rad   = (float)DEG2RAD(r);
        ref[n].pitch_rad  = (float)DEG2RAD(p);
        ref[n].yaw_rad    = (float)DEG2RAD(y);
        ref[n].vel_ned[0] = vn;
        ref[n].vel_ned[1] = ve;
        ref[n].vel_ned[2] = vd;
        n++;
    }
    fclose(f);
    *out = ref;
    return n;
}

/* Fill a column-major symmetric 3x3 from the 6 unique elements
 * (nn, ne, nd, ee, ed, dd). */
static void cov6_to_mat3(const float c6[6], float Q[3 * 3])
{
    Q[0] = c6[0];
    Q[1] = c6[1];
    Q[2] = c6[2];
    Q[3] = c6[1];
    Q[4] = c6[3];
    Q[5] = c6[4];
    Q[6] = c6[2];
    Q[7] = c6[4];
    Q[8] = c6[5];
}

/* gnss.csv: t_us, lat_deg, lon_deg, h_m, pos covariance (nn,ne,nd,ee,
 * ed,dd) [m^2], v_ned [m/s], vel covariance (nn,ne,nd,ee,ed,dd)
 * [(m/s)^2], vel_ok. Zeros on a covariance diagonal mean "unknown" and
 * are replaced by the config fallback stddevs at attach time. */
static int load_gnss(const char* path, gnss_epoch_t** out)
{
    *out    = NULL;
    FILE* f = fopen(path, "r");
    if (!f) { return -1; }
    char          line[1024];
    int           n   = 0;
    int           cap = 0;
    gnss_epoch_t* g   = NULL;
    while (fgets(line, sizeof(line), f))
    {
        long long t;
        double    lat, lon, h;
        float     cp[6], v[3], cv[6];
        int       ok;
        if (line[0] == '#') continue;
        if (sscanf(line,
                   "%lld,%lf,%lf,%lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", //
                   &t, &lat, &lon, &h,                                                 //
                   &cp[0], &cp[1], &cp[2], &cp[3], &cp[4], &cp[5],                     //
                   &v[0], &v[1], &v[2],                                                //
                   &cv[0], &cv[1], &cv[2], &cv[3], &cv[4], &cv[5], &ok) != 20)
        {
            continue;
        }
        g            = grow_or_die(g, n, &cap, sizeof(*g), path);
        g[n].t_us    = (int64_t)t;
        g[n].lat_rad = lat * M_PI / 180.0;
        g[n].lon_rad = lon * M_PI / 180.0;
        g[n].h_m     = h;
        cov6_to_mat3(cp, g[n].Qpos_ned);
        g[n].vel_ned[0] = v[0];
        g[n].vel_ned[1] = v[1];
        g[n].vel_ned[2] = v[2];
        cov6_to_mat3(cv, g[n].Qvel_ned);
        g[n].vel_ok = ok;
        n++;
    }
    fclose(f);
    *out = g;
    return n;
}

static int load_mag(const char* path, mag_epoch_t** out)
{
    *out    = NULL;
    FILE* f = fopen(path, "r");
    if (!f) { return -1; }
    char         line[256];
    int          n   = 0;
    int          cap = 0;
    mag_epoch_t* m   = NULL;
    while (fgets(line, sizeof(line), f))
    {
        long long t;
        float     b[3];
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f,%f,%f", &t, &b[0], &b[1], &b[2]) != 4) { continue; }
        m              = grow_or_die(m, n, &cap, sizeof(*m), path);
        m[n].t_us      = (int64_t)t;
        m[n].mag_ut[0] = b[0];
        m[n].mag_ut[1] = b[1];
        m[n].mag_ut[2] = b[2];
        n++;
    }
    fclose(f);
    *out = m;
    return n;
}

static int load_baro(const char* path, baro_epoch_t** out)
{
    *out    = NULL;
    FILE* f = fopen(path, "r");
    if (!f) { return -1; }
    char          line[256];
    int           n   = 0;
    int           cap = 0;
    baro_epoch_t* b   = NULL;
    while (fgets(line, sizeof(line), f))
    {
        long long t;
        float     p;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f", &t, &p) != 2) { continue; }
        b                = grow_or_die(b, n, &cap, sizeof(*b), path);
        b[n].t_us        = (int64_t)t;
        b[n].pressure_pa = p;
        n++;
    }
    fclose(f);
    *out = b;
    return n;
}

/* speed.csv: "t_us, speed_mps" plus any further columns the producer chose
   to keep. Only those two are read here -- uncertainty and delay come from
   the config as constants, matching python/replay.py's load_speed(). */
static int load_speed(const char* path, speed_epoch_t** out)
{
    *out    = NULL;
    FILE* f = fopen(path, "r");
    if (!f) { return -1; }
    char           line[256];
    int            n   = 0;
    int            cap = 0;
    speed_epoch_t* s   = NULL;
    while (fgets(line, sizeof(line), f))
    {
        long long t;
        float     v;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f", &t, &v) != 2) { continue; }
        s              = grow_or_die(s, n, &cap, sizeof(*s), path);
        s[n].t_us      = (int64_t)t;
        s[n].speed_mps = v;
        n++;
    }
    fclose(f);
    *out = s;
    return n;
}

/* ---------------------------------------------------------------------------
 * Coasting re-acquisition metric (REQ-VER-029)
 *
 * A dataset with a real outage cannot be scored inside it: the reference
 * comes from the same receiver as the aiding, so no fixes means no truth.
 * The one question the data does answer is how far out the filter is when
 * aiding returns, and that is what this measures.
 *
 * The gaps are taken from the aiding FILE, not from what the filter fused,
 * so the number stays comparable across configurations that gate fixes
 * differently.
 * ------------------------------------------------------------------------- */

#define REPLAY_MAX_COAST_GAPS 16
/* Furthest a reference row may lie from the sampled IMU epoch and still score
   a gap, and never more than half the gap itself (REQ-VER-029). */
#define REPLAY_COAST_REF_MAX_DT_US 500000

typedef struct
{
    int64_t t_beg_us;  /* timestamp of the last fix before the gap */
    int64_t t_end_us;  /* timestamp of the first fix after the gap */
    double  gap_sec;   /* length of the gap */
    double  chord_m;   /* straight-line distance across it */
    double  err_m;     /* position error at the first reference epoch after it */
    int     have_err;  /* 0 -> the filter had no solution there, which fails */
    int     resolved;  /* 0 -> still waiting for that reference epoch */
} coast_gap_t;

/* Collect every gap longer than min_sec that ends after t_warmup_end. */
static int coast_gaps_scan(const gnss_epoch_t* fix, int n_fix, double min_sec,
                           int64_t t_warmup_end, coast_gap_t* out, int max_out)
{
    int n = 0;
    int i;
    if (min_sec <= 0.0 || n_fix < 2) return 0;
    for (i = 1; i < n_fix && n < max_out; ++i)
    {
        const double gap = (double)(fix[i].t_us - fix[i - 1].t_us) * 1e-6;
        double       dllh[3];
        float        dned_f[3];
        if (gap < min_sec) continue;
        if (fix[i].t_us < t_warmup_end) continue;
        dllh[0] = fix[i].lat_rad - fix[i - 1].lat_rad;
        dllh[1] = fix[i].lon_rad - fix[i - 1].lon_rad;
        dllh[2] = fix[i].h_m - fix[i - 1].h_m;
        ins_dlatlonh_to_dned(dllh, fix[i - 1].lat_rad, fix[i - 1].h_m, dned_f);
        out[n].t_beg_us = fix[i - 1].t_us;
        out[n].t_end_us = fix[i].t_us;
        out[n].gap_sec  = gap;
        out[n].chord_m =
            sqrt((double)dned_f[0] * dned_f[0] + (double)dned_f[1] * dned_f[1]);
        out[n].err_m    = 0.0;
        out[n].have_err = 0;
        out[n].resolved = 0;
        n++;
        if (n == max_out && i + 1 < n_fix)
        {
            /* Say so rather than dropping the rest quietly: a dataset with
               more outages than this would otherwise be gated on a subset
               nobody chose. */
            fprintf(stderr, "replay: more than %d aiding gaps over %g s, only the first %d "
                            "are scored\n",
                    max_out, min_sec, max_out);
        }
    }
    return n;
}

static nav_suite_t    g_suite; /* too large for the stack */
static ref_epoch_t*   g_ref;   /* per-stream buffers, grown on demand at load */
static gnss_epoch_t*  g_gnss;
static mag_epoch_t*   g_mag;
static baro_epoch_t*  g_baro;
static speed_epoch_t* g_speed;

/* 3D position error of the ins solution against one reference epoch, with the
 * scoring lever arm mapping the filter position onto the truth point. Returns
 * 0 when the filter has no position to offer, which is an answer of its own
 * for the coasting metric (REQ-VER-029). */
static int nav_pos_error_m(const ref_epoch_t* ref, const float leverarm_frd[3], double* err_m)
{
    double llh[3];
    double dllh[3];
    float  R[9];
    float  la_n[3] = {0.0f, 0.0f, 0.0f};
    float  dned_f[3];
    double dn, de, dd;
    if (!ins_is_ready(&g_suite.ins)) return 0;
    /* The anchor as the filter holds it (REQ-NAV-078): going out through
       ins_get_position_ecef() and back would run two conversions to land on
       the same three numbers. */
    if (!ins_get_latlonh(&g_suite.ins, llh)) return 0;
    if (ins_get_rotmat_b_to_n(&g_suite.ins, R))
    {
        la_n[0] = R[0] * leverarm_frd[0] + R[3] * leverarm_frd[1] + R[6] * leverarm_frd[2];
        la_n[1] = R[1] * leverarm_frd[0] + R[4] * leverarm_frd[1] + R[7] * leverarm_frd[2];
        la_n[2] = R[2] * leverarm_frd[0] + R[5] * leverarm_frd[1] + R[8] * leverarm_frd[2];
    }
    dllh[0] = llh[0] - ref->lat_rad;
    dllh[1] = llh[1] - ref->lon_rad;
    dllh[2] = llh[2] - ref->h_m;
    ins_dlatlonh_to_dned(dllh, ref->lat_rad, ref->h_m, dned_f);
    dn     = dned_f[0] + la_n[0];
    de     = dned_f[1] + la_n[1];
    dd     = dned_f[2] + la_n[2];
    *err_m = sqrt(dn * dn + de * de + dd * dd);
    return 1;
}

/* Reference epoch nearest to IMU epoch t for the coasting metric
 * (REQ-VER-029). iref is the index of the first row after t, so the only
 * candidates are the rows either side of it. Returns NULL when neither lies
 * within max_dt_us: a reference that does not cover the end of the gap cannot
 * score it. */
static const ref_epoch_t* coast_ref_near(int64_t t, int iref, int n_ref, int64_t max_dt_us)
{
    const ref_epoch_t* best    = (const ref_epoch_t*)0;
    int64_t            best_dt = 0;
    int                k;
    for (k = iref - 1; k <= iref; ++k)
    {
        int64_t dt;
        if (k < 0 || k >= n_ref) continue;
        dt = g_ref[k].t_us - t;
        if (dt < 0) dt = -dt;
        if (best == (const ref_epoch_t*)0 || dt < best_dt)
        {
            best    = &g_ref[k];
            best_dt = dt;
        }
    }
    if (best_dt > max_dt_us) { best = (const ref_epoch_t*)0; }
    return best;
}

/* Mean gyro over the first seconds of the (stationary) trial = initial
 * gyro bias; same idea as pyahrs.py initialGyroBiasHeuristics().
 * Rewinds the file afterwards. */
static int estimate_gyro_bias(FILE* fimu, double window_sec, float bias[3])
{
    char    line[512];
    double  sum[3] = {0.0, 0.0, 0.0};
    int     n      = 0;
    int64_t t0     = -1;

    while (fgets(line, sizeof(line), fimu))
    {
        long long t;
        float     g[3], a[3];
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f,%f,%f,%f,%f,%f", &t, &g[0], &g[1], &g[2], &a[0], &a[1], &a[2]) !=
            7)
        {
            continue;
        }
        if (t0 < 0) t0 = (int64_t)t;
        if ((double)((int64_t)t - t0) > window_sec * US_PER_SEC) { break; }
        sum[0] += g[0];
        sum[1] += g[1];
        sum[2] += g[2];
        n++;
    }
    (void)fseek(fimu, 0, SEEK_SET);
    if (n < 10) { return -1; }
    bias[0] = (float)(sum[0] / n);
    bias[1] = (float)(sum[1] / n);
    bias[2] = (float)(sum[2] / n);
    return 0;
}

/* Timestamp of the first IMU sample (-1 if none); rewinds the stream. */
static int64_t first_imu_time_us(FILE* fimu)
{
    char    line[512];
    int64_t t = -1;

    while (fgets(line, sizeof(line), fimu))
    {
        long long tt;
        float     g[3], a[3];
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f,%f,%f,%f,%f,%f", &tt, &g[0], &g[1], &g[2], &a[0], &a[1], &a[2]) ==
            7)
        {
            t = (int64_t)tt;
            break;
        }
    }
    (void)fseek(fimu, 0, SEEK_SET);
    return t;
}

/* Replace zero (unknown) diagonal entries by the fallback variance;
 * returns 0 if the result still has a non-positive diagonal entry
 * (measurement unusable). */
static int cov_apply_fallback(float Q[3 * 3], float var_hor, float var_ver)
{
    if (Q[0] <= 0.0f) { Q[0] = var_hor; }
    if (Q[4] <= 0.0f) { Q[4] = var_hor; }
    if (Q[8] <= 0.0f) { Q[8] = var_ver; }
    return (Q[0] > 0.0f && Q[4] > 0.0f && Q[8] > 0.0f);
}

int main(int argc, char** argv)
{
    printf("ins replay\n");
    if (argc < 2)
    {
        fprintf(stderr, "usage: replay <config.yaml | datadir> [errdump.csv]\n");
        return 1;
    }

    /* Resolve the config file and the data directory next to it. */
    replay_cfg_t cfg;
    char         cfg_path[512], datadir[512];
    /* datadir + '/' + the longest file name a config can carry + NUL
       (every cfg.in_* field is the same size). Sized from those two so
       the joins below cannot truncate: a truncated path is not just a
       confusing error message, it could name a different existing
       file. */
    char path[sizeof(datadir) + 1 + sizeof(cfg.in_imu)];
    if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 5, ".yaml"))
    {
        snprintf(cfg_path, sizeof(cfg_path), "%s", argv[1]);
        snprintf(datadir, sizeof(datadir), "%s", argv[1]);
        char* slash = strrchr(datadir, '/');
#ifdef _WIN32
        char* bslash = strrchr(datadir, '\\');
        if (bslash > slash) slash = bslash;
#endif
        if (slash) { *slash = '\0'; }
        else { snprintf(datadir, sizeof(datadir), "."); }
    }
    else
    {
        snprintf(datadir, sizeof(datadir), "%s", argv[1]);
        snprintf(cfg_path, sizeof(cfg_path), "%s/config.yaml", argv[1]);
    }

    if (load_config(cfg_path, &cfg) != 0)
    {
        fprintf(stderr, "cannot load %s (run the datasets/fetch_*.sh script?)\n", cfg_path);
        return 1;
    }
    const int aiding_gnss = (strcmp(cfg.aiding, "gnss") == 0);
    const int aiding_ref  = (strcmp(cfg.aiding, "ref") == 0);
    const int aiding_none = (strcmp(cfg.aiding, "none") == 0);
    const int init_ref    = (strcmp(cfg.init, "ref") == 0);
    if (!aiding_gnss && !aiding_ref && !aiding_none)
    {
        fprintf(stderr, "%s: unknown aiding mode '%s' (expected gnss/ref/none)\n", cfg_path,
                cfg.aiding);
        return 1;
    }

    FILE* fdump = (FILE*)0;
    if (argc > 2)
    {
        fdump = fopen(argv[2], "w");
        if (fdump)
        {
            fprintf(fdump, "# t_sec, speed_mps, nav_roll_err, nav_pitch_err,"
                           " nav_yaw_err, nav_pos_err, ars_roll_err,"
                           " ars_pitch_err, ars_yaw_err [deg/m]\n");
        }
    }

    snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_ref);
    const int n_ref = load_ref(path, &g_ref);
    if (n_ref <= 0)
    {
        fprintf(stderr, "cannot load %s\n", path);
        return 1;
    }

    /* Move every reference row onto its own time of validity (REQ-VER-030).
       Done here rather than at each comparison so that everything downstream
       -- scoring, the error dump, the warmup window -- reads one consistent
       timeline, and so a row's timestamp means what it says from this point
       on. */
    if (cfg.ref_delay_ms > 0.0 || cfg.ref_delay_ms < 0.0)
    {
        const int64_t shift = (int64_t)(cfg.ref_delay_ms * 1000.0);
        int           i;
        for (i = 0; i < n_ref; ++i) { g_ref[i].t_us -= shift; }
        printf("reference time of validity: %.0f ms earlier than its timestamps "
               "(score: ref_delay_ms)\n",
               cfg.ref_delay_ms);
    }

    int n_gnss = 0;
    if (aiding_gnss)
    {
        snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_gnss);
        n_gnss = load_gnss(path, &g_gnss);
        if (n_gnss <= 0)
        {
            fprintf(stderr, "aiding: gnss but no usable %s\n", path);
            return 1;
        }
    }

    int n_mag = 0;
    if (cfg.mag_enable)
    {
        snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_mag);
        n_mag = load_mag(path, &g_mag);
        if (n_mag <= 0)
        {
            fprintf(stderr, "mag: enable but no usable %s\n", path);
            return 1;
        }
    }

    int n_baro = 0;
    if (cfg.baro_enable)
    {
        snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_baro);
        n_baro = load_baro(path, &g_baro);
        if (n_baro <= 0)
        {
            fprintf(stderr, "baro: enable but no usable %s\n", path);
            return 1;
        }
    }

    int n_speed = 0;
    if (cfg.speed_enable)
    {
        snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_speed);
        n_speed = load_speed(path, &g_speed);
        if (n_speed <= 0)
        {
            fprintf(stderr, "speed: enable but no usable %s\n", path);
            return 1;
        }
    }

    snprintf(path, sizeof(path), "%s/%s", datadir, cfg.in_imu);
    FILE* fimu = fopen(path, "r");
    if (!fimu)
    {
        fprintf(stderr, "cannot load %s\n", path);
        return 1;
    }

    float      gyr_bias0[3] = {0.0f, 0.0f, 0.0f};
    const bool have_bias0   = (estimate_gyro_bias(fimu, cfg.gyro_bias_window_sec, gyr_bias0) == 0);
    bool       bias0_corrected = false; /* nav-frame rotation removed (init: ref) */

    /* --- filter setup ---------------------------------------------------- */
    ins_init_t init;
    memset(&init, 0, sizeof(init));
    if (init_ref)
    {
        /* Known initial state from the reference: position, attitude AND
           velocity. The start need not be stationary (e.g. an aircraft
           cruising at 200 m/s), so the reference NED velocity is carried
           over as-is: ins_init takes the velocity in the n-frame and the
           position geodetically (REQ-NAV-081), which is how the reference
           states both. Everything after t0 is aided by GNSS only.

           Use the reference epoch aligned with the FIRST IMU sample, not
           g_ref[0]: ins defers a prescribed init to its actual start
           epoch and stamps the provided state THERE without propagating
           it (REQ-NAV-033). Handing it an older truth epoch bakes a
           permanent -v*dt position offset into a moving start (2 m East
           on the Groves aircraft profile: one 10-ms IMU period at
           200 m/s East), which then reads as filter inaccuracy. */
        const int64_t t_imu0 = first_imu_time_us(fimu);
        int           i0     = 0;
        while (i0 + 1 < n_ref && g_ref[i0].t_us < t_imu0) { i0++; }
        const ref_epoch_t* r0 = &g_ref[i0];
        init.time             = r0->t_us;
        /* Position and velocity as the reference holds them, which is also
           what the filter works in (REQ-NAV-081). */
        init.llh[0] = r0->lat_rad;
        init.llh[1]           = r0->lon_rad;
        init.llh[2]           = r0->h_m;
        init.rpy_init_rad[0]  = r0->roll_rad;
        init.rpy_init_rad[1]  = r0->pitch_rad;
        init.rpy_init_rad[2]  = r0->yaw_rad;
        init.vel_ned[0]       = r0->vel_ned[0];
        init.vel_ned[1]       = r0->vel_ned[1];
        init.vel_ned[2]       = r0->vel_ned[2];
        if (have_bias0)
        {
            /* Remove the modeled non-bias content from the initial-window
               gyro average -- the reference provides position, velocity
               AND attitude here (init: ref):
                 - the navigation-frame rotation (earth rate, ~15 deg/hr
                   at mid latitudes, plus transport rate, comparable at
                   aircraft speeds), and
                 - the platform's own mean rotation over the window,
                   derived from the reference attitude (log map of the
                   relative rotation / window length) -- the Groves car
                   pitches up 0.9 deg while accelerating INSIDE the 3 s
                   window (0.31 deg/s of very real y rate).
               What remains is the sensor bias. Leaving these in poisons
               the seed in a way a tight gyr_bias_init_stddev_rps can
               never recover from, and drives the ARS's unobservable
               z-bias directly. */
            const int64_t t_b = t_imu0 + (int64_t)(cfg.gyro_bias_window_sec * US_PER_SEC);
            int           i1  = i0;
            while (i1 + 1 < n_ref && g_ref[i1 + 1].t_us <= t_b) { i1++; }
            const ref_epoch_t* r1 = &g_ref[i1];

            float w_in_n[3], q[4], R0[9], R1[9];
            ins_calc_omega_n_in(r0->lat_rad, r0->h_m, r0->vel_ned, w_in_n, (float*)0, (float*)0);
            ins_quat_from_rpy(r0->roll_rad, r0->pitch_rad, r0->yaw_rad, q);
            ins_quat_to_rotmat(q, R0);
            int k;
            for (k = 0; k < 3; ++k) /* w_b = R0' * w_n (R column-major) */
            {
                gyr_bias0[k] -= R0[3 * k + 0] * w_in_n[0] + R0[3 * k + 1] * w_in_n[1] +
                                R0[3 * k + 2] * w_in_n[2];
            }

            const float span = (float)(r1->t_us - r0->t_us) / (float)US_PER_SEC;
            if (span > 0.5f) /* enough baseline for the attitude-delta rate */
            {
                ins_quat_from_rpy(r1->roll_rad, r1->pitch_rad, r1->yaw_rad, q);
                ins_quat_to_rotmat(q, R1);
                float Rrel[9]; /* body rotation over the window: R0' * R1 */
                int   rr, cc;
                for (rr = 0; rr < 3; ++rr)
                {
                    for (cc = 0; cc < 3; ++cc)
                    {
                        Rrel[rr + 3 * cc] = R0[0 + 3 * rr] * R1[0 + 3 * cc] +
                                            R0[1 + 3 * rr] * R1[1 + 3 * cc] +
                                            R0[2 + 3 * rr] * R1[2 + 3 * cc];
                    }
                }
                /* rotation vector (log map), then mean rate = phi / span */
                float s_vec[3] = {0.5f * (Rrel[2 + 3 * 1] - Rrel[1 + 3 * 2]),
                                  0.5f * (Rrel[0 + 3 * 2] - Rrel[2 + 3 * 0]),
                                  0.5f * (Rrel[1 + 3 * 0] - Rrel[0 + 3 * 1])};
                float c        = 0.5f * (Rrel[0] + Rrel[4] + Rrel[8] - 1.0f);
                if (c > 1.0f) { c = 1.0f; }
                if (c < -1.0f) { c = -1.0f; }
                const float theta = acosf(c);
                const float sin_t = sinf(theta);
                const float scale = (sin_t > 1e-9f) ? theta / sin_t : 1.0f;
                for (k = 0; k < 3; ++k) { gyr_bias0[k] -= scale * s_vec[k] / span; }
            }
            bias0_corrected = true;
        }
    }
    else if (aiding_gnss)
    {
        init.time = g_gnss[0].t_us;
        init.llh[0] = g_gnss[0].lat_rad; /* replaced by auto-init */
        init.llh[1]           = g_gnss[0].lon_rad;
        init.llh[2]           = g_gnss[0].h_m;
    }
    else if (cfg.fi_enable)
    {
        /* The declared position IS the origin here, rather than something
           the first fix will replace. */
        init.time = first_imu_time_us(fimu);
        init.llh[0] = DEG2RAD(cfg.fi_lat_deg);
        init.llh[1]           = DEG2RAD(cfg.fi_lon_deg);
        init.llh[2]           = cfg.fi_height_m;
    }
    else
    {
        init.time = g_ref[0].t_us;
        init.llh[0] = g_ref[0].lat_rad; /* replaced by auto-init */
        init.llh[1]           = g_ref[0].lon_rad;
        init.llh[2]           = g_ref[0].h_m;
    }
    if (have_bias0)
    {
        init.gyr_bias_init_rps[0] = gyr_bias0[0];
        init.gyr_bias_init_rps[1] = gyr_bias0[1];
        init.gyr_bias_init_rps[2] = gyr_bias0[2];
    }
    /* REQ-VER-010: config init_stddev.* overrides the built-in default
       (each field individually, 0 -> unchanged). yaw_init_stddev_rad_deg has
       no built-in default of its own to override -- like ins's own
       field, 0 just means "fall back to rpy_init_stddev_rad_deg", so it's
       only set here when the config actually asks for it. */
    /* free_inertial_start: the origin IS the declared point, so the
       uncertainty of that statement is the initial position uncertainty
       by construction - stating 1 cm and starting the filter at a 10 m
       prior would throw the statement away. An explicit
       init_stddev.pos_init_stddev_m still wins: it is the more specific
       thing to have written down. */
    init.pos_init_stddev_m =
        (cfg.init_pos_stddev_m > 0.0f)
            ? cfg.init_pos_stddev_m
            : (cfg.fi_enable ? cfg.fi_stddev_m : (init_ref ? 0.1f : 1.0f));
    init.vel_init_stddev_mps =
        (cfg.init_vel_stddev_mps > 0.0f) ? cfg.init_vel_stddev_mps : (init_ref ? 0.1f : 0.5f);
    {
        const float rp_stddev_rad   = (cfg.init_rpy_stddev_deg > 0.0f)
                                          ? DEG2RAD(cfg.init_rpy_stddev_deg)
                                          : (init_ref ? DEG2RAD(1.0f) : DEG2RAD(3.0f));
        init.rpy_init_stddev_rad[0] = rp_stddev_rad;
        init.rpy_init_stddev_rad[1] = rp_stddev_rad;
    }
    if (cfg.init_yaw_stddev_deg > 0.0f)
    {
        /* cppcheck-suppress internalAstError
         * cppcheck 2.13 cannot build the AST for _Generic(x, ...) when x is
         * a struct member expression (reproduced in isolation outside this
         * repo); DEG2RAD()/RAD2DEG() are fine on plain variables. */
        init.rpy_init_stddev_rad[2] = DEG2RAD(cfg.init_yaw_stddev_deg);
    }
    init.acc_bias_init_stddev_mps2 =
        (cfg.init_acc_bias_stddev_mps2 > 0.0f) ? cfg.init_acc_bias_stddev_mps2 : 0.05f;
    init.gyr_bias_init_stddev_rps = (cfg.init_gyr_bias_stddev_deg > 0.0f)
                                        ? DEG2RAD(cfg.init_gyr_bias_stddev_deg)
                                        : (have_bias0 ? DEG2RAD(0.2f) : DEG2RAD(0.5f));
    /* Extra process-noise margin (0 -> ins.c's own default, verbatim). */
    init.pos_pred_stddev_m_sqrts   = cfg.pos_pred_stddev_m_sqrts;
    init.vel_pred_stddev_mps_sqrts = cfg.vel_pred_stddev_mps_sqrts;
    init.rpy_pred_stddev_rad_sqrts = cfg.rpy_pred_stddev_rad_sqrts;
    /* IMU bias random walk (imu: gyr_bias_rw/acc_bias_rw, already SI):
       ins's own bias process noise, not to be confused with the ARS/AHRS
       template derived from the same two keys below. */
    init.acc_bias_pred_stddev_mps2_sqrts = cfg.acc_bias_rw;
    init.gyr_bias_pred_stddev_rps_sqrts  = cfg.gyr_bias_rw;
    /* Auto-ZUPT/ZARU pseudo-measurement stddev: this harness's own default
       (0.05 m/s, 0.1 deg/s), NOT ins.c's own built-in default (0.05 m/s,
       0.5 deg/s) -- shared with python/replay.py's build_config() for
       years, see the replay_cfg_t comment above. */
    init.zero_vel_stddev_mps = (cfg.zero_vel_stddev_mps > 0.0f) ? cfg.zero_vel_stddev_mps : 0.05f;
    init.zero_rot_stddev_rps =
        (cfg.zero_rot_stddev_deg > 0.0f) ? DEG2RAD(cfg.zero_rot_stddev_deg) : DEG2RAD(0.1f);

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    /* kalman_update_dt_sec left at 0 -> default */
    opt.max_prediction_time_sec            = 0.5f;
    /* Noise-shutdown gates, 0 -> ins's own default (verbatim, no harness default). */
    opt.gnss_max_horizontal_pos_stddev_m   = cfg.max_horizontal_pos_stddev_m;
    opt.gnss_max_vertical_pos_stddev_m     = cfg.max_vertical_pos_stddev_m;
    opt.gnss_max_horizontal_vel_stddev_mps = cfg.max_horizontal_vel_stddev_mps;
    opt.gnss_max_vertical_vel_stddev_mps   = cfg.max_vertical_vel_stddev_mps;
    /* Solution-mode gates (REQ-NAV-051/052), 0 -> ins's own defaults. */
    opt.gnss_start_max_horizontal_pos_stddev_m   = cfg.start_max_horizontal_pos_stddev_m;
    opt.gnss_start_max_vertical_pos_stddev_m     = cfg.start_max_vertical_pos_stddev_m;
    opt.gnss_start_max_horizontal_vel_stddev_mps = cfg.start_max_horizontal_vel_stddev_mps;
    opt.gnss_start_max_vertical_vel_stddev_mps   = cfg.start_max_vertical_vel_stddev_mps;
    opt.gnss_stop_max_horizontal_pos_stddev_m    = cfg.stop_max_horizontal_pos_stddev_m;
    opt.gnss_stop_max_vertical_pos_stddev_m      = cfg.stop_max_vertical_pos_stddev_m;
    opt.gnss_stop_max_horizontal_vel_stddev_mps  = cfg.stop_max_horizontal_vel_stddev_mps;
    opt.gnss_stop_max_vertical_vel_stddev_mps    = cfg.stop_max_vertical_vel_stddev_mps;
    opt.gnss_init_dwell_sec                      = cfg.init_dwell_sec;
    opt.gnss_init_dwell_disable                  = (cfg.init_dwell_disable != 0);
    opt.gnss_stop_dwell_sec                      = cfg.stop_dwell_sec;
    opt.gnss_stop_disable                        = (cfg.stop_disable != 0);
    opt.gnss_pos_decimation                      = cfg.pos_decimation;
    opt.gnss_min_delay_ms                        = cfg.min_delay_ms;
    opt.auto_init                                = !init_ref;
    opt.auto_init_window_sec                     = cfg.auto_init_window_sec;
    opt.automotive_mode                          = cfg.automotive_mode != 0;
    opt.automotive_min_speed_mps                 = cfg.automotive_min_speed_mps;
    opt.automotive_min_yaw_stddev                = (cfg.automotive_min_yaw_stddev_deg > 0.0f)
                                                       ? DEG2RAD(cfg.automotive_min_yaw_stddev_deg)
                                                       : 0.0f;
    opt.automotive_lateral_constraint            = cfg.automotive_lateral_constraint != 0;
    opt.automotive_lateral_stddev_mps            = cfg.automotive_lateral_stddev_mps;
    opt.automotive_lateral_max_yaw_rate =
        (cfg.automotive_lateral_max_yaw_rate_deg > 0.0f)
            ? DEG2RAD(cfg.automotive_lateral_max_yaw_rate_deg)
            : 0.0f;
    opt.automotive_lateral_after_sec             = cfg.automotive_lateral_after_sec;
    /* Stillness detection (REQ-VER-024): set once here, nav_suite_init()
       hands it to the ARS/AHRS and baro_alt (REQ-SUITE-020). */
    opt.auto_zupt_static_gyr_rps =
        (cfg.auto_zupt_static_gyr_deg > 0.0f) ? DEG2RAD(cfg.auto_zupt_static_gyr_deg) : 0.0f;
    opt.auto_zupt_static_acc_mps2        = cfg.auto_zupt_static_acc_mps2;
    opt.auto_zupt_max_vel_mps            = cfg.auto_zupt_max_vel_mps;
    opt.auto_zupt_max_vel_stddev_mps     = cfg.auto_zupt_max_vel_stddev_mps;
    opt.auto_zupt_static_gyr_stddev_rps  = (cfg.auto_zupt_static_gyr_stddev_deg > 0.0f)
                                               ? DEG2RAD(cfg.auto_zupt_static_gyr_stddev_deg)
                                               : 0.0f;
    opt.auto_zupt_static_acc_stddev_mps2 = cfg.auto_zupt_static_acc_stddev_mps2;
    opt.auto_zupt_dwell_sec              = cfg.auto_zupt_dwell_sec;
    opt.auto_zupt_min_interval_sec       = cfg.auto_zupt_min_interval_sec;
    opt.auto_zupt_disable                = cfg.auto_zupt_disable != 0;
    opt.auto_zupt_velocity_blind_disable = cfg.auto_zupt_velocity_blind_disable != 0;
    opt.chi2_disable                     = cfg.chi2_disable != 0;
    opt.chi2_reject_alpha                = cfg.chi2_reject_alpha;
    opt.allow_unlimited_deadreckoning    = cfg.allow_unlimited_deadreckoning != 0;
    if (cfg.fi_enable)
    {
        /* Developer mode, identical to the live receiver's: coasting is
           the intent, so neither the dead-reckoning budget nor the 3D
           exit gate may end the run. The entry gate stays, and every fix
           is still judged on its own accuracy before it may fuse. */
        opt.allow_unlimited_deadreckoning = true;
        opt.gnss_stop_disable             = true;
    }
    opt.max_deadreckoning_sec            = cfg.max_deadreckoning_sec;
    opt.baro_height_disable              = cfg.baro_height_disable != 0;
    opt.speed_scale                      = cfg.speed_scale;
    opt.speed_stddev_rel                 = cfg.speed_stddev_rel;
    opt.speed_min_mps                    = cfg.speed_min_mps;
    opt.estimate_mag_bias                = cfg.mag_estimate_bias != 0;
    opt.magnetometer_min_delay_ms        = cfg.mag_min_delay_ms;
    /* Both are 0 unless the config names them, which is the library's own
       "use the default" sentinel -- so a dataset that says nothing about
       the hard-iron states replays exactly as it did before these keys
       existed. */
    init.mag_bias_init_stddev_ut         = cfg.mag_bias_init_ut;
    init.mag_bias_pred_stddev_ut_sqrts   = cfg.mag_bias_rw_ut_sqrts;

    /* IMU calibration (REQ-NAV-037) and GNSS covariance conditioning
       (REQ-NAV-038): copied through verbatim (all-0 / 0 sentinels are the
       library's own no-op defaults). */
    memcpy(opt.imu_acc_misalignment, cfg.acc_misalignment, sizeof(opt.imu_acc_misalignment));
    memcpy(opt.imu_gyr_misalignment, cfg.gyr_misalignment, sizeof(opt.imu_gyr_misalignment));
    memcpy(opt.imu_acc_fixed_bias, cfg.acc_fixed_bias, sizeof(opt.imu_acc_fixed_bias));
    memcpy(opt.imu_gyr_fixed_bias, cfg.gyr_fixed_bias, sizeof(opt.imu_gyr_fixed_bias));
    opt.gnss_pos_cov_scale            = cfg.pos_cov_scale;
    opt.gnss_pos_cov_scale_height     = cfg.pos_cov_scale_height;
    opt.gnss_vel_cov_scale            = cfg.vel_cov_scale;
    opt.gnss_pos_stddev_floor_hor_m   = cfg.pos_stddev_floor_hor_m;
    opt.gnss_pos_stddev_floor_ver_m   = cfg.pos_stddev_floor_ver_m;
    opt.gnss_vel_stddev_floor_hor_mps = cfg.vel_stddev_floor_hor_mps;
    opt.gnss_vel_stddev_floor_ver_mps = cfg.vel_stddev_floor_ver_mps;
    opt.gnss_pos_stddev_cap_hor_m     = cfg.pos_stddev_cap_hor_m;
    opt.gnss_pos_stddev_cap_ver_m     = cfg.pos_stddev_cap_ver_m;
    opt.gnss_vel_stddev_cap_hor_mps   = cfg.vel_stddev_cap_hor_mps;
    opt.gnss_vel_stddev_cap_ver_mps   = cfg.vel_stddev_cap_ver_mps;
    opt.gnss_acc_envelope_tau_sec     = cfg.acc_envelope_tau_sec;
    opt.gnss_vel_noise_acc_scale_hor  = cfg.vel_noise_acc_scale_hor;
    opt.gnss_vel_noise_acc_scale_ver  = cfg.vel_noise_acc_scale_ver;
    opt.gnss_vel_noise_acc_window_sec = cfg.vel_noise_acc_window_sec;
    memcpy(opt.mag_misalignment, cfg.mag_misalignment, sizeof(opt.mag_misalignment));
    memcpy(opt.mag_fixed_bias, cfg.mag_fixed_bias, sizeof(opt.mag_fixed_bias));

    memset(&g_suite, 0, sizeof(g_suite));
    if (nav_suite_init(&g_suite, &init, &opt) != 0)
    {
        fprintf(stderr, "nav_suite_init failed\n");
        return 1;
    }
    nav_suite_set_init_att_hint(
        &g_suite, DEG2RAD(cfg.init_hint_roll_deg), DEG2RAD(cfg.init_hint_pitch_deg),
        DEG2RAD(cfg.init_hint_rpy_stddev_deg), DEG2RAD(cfg.init_hint_yaw_deg),
        DEG2RAD(cfg.init_hint_yaw_stddev_deg));

    g_suite.ars_cfg.gyr_noise_psd =
        (cfg.ahrs_gyr_noise_psd > 0.0f) ? cfg.ahrs_gyr_noise_psd : SQRTF(cfg.gyr_psd);
    g_suite.ars_cfg.gyr_bias_rw =
        (cfg.ahrs_gyr_bias_rw > 0.0f) ? cfg.ahrs_gyr_bias_rw : cfg.gyr_bias_rw;
    g_suite.ars_cfg.acc_noise_mps2 =
        (cfg.ahrs_acc_noise_mps2 > 0.0f) ? cfg.ahrs_acc_noise_mps2 : 0.25f;
    if (have_bias0)
    {
        int k;
        for (k = 0; k < 3; ++k)
        {
            g_suite.ars_cfg.gyr_bias_init_rps[k]        = gyr_bias0[k];
            g_suite.ars_cfg.gyr_bias_init_stddev_rps[k] = DEG2RAD(0.2f);
        }
    }
    if (cfg.ahrs_gyr_bias_init_stddev_deg > 0.0f)
    {
        int k;
        for (k = 0; k < 3; ++k)
        {
            g_suite.ars_cfg.gyr_bias_init_stddev_rps[k] =
                DEG2RAD(cfg.ahrs_gyr_bias_init_stddev_deg);
        }
    }

    {
        int k;
        g_suite.ahrs_cfg.gyr_noise_psd  = g_suite.ars_cfg.gyr_noise_psd;
        g_suite.ahrs_cfg.gyr_bias_rw    = g_suite.ars_cfg.gyr_bias_rw;
        g_suite.ahrs_cfg.acc_noise_mps2 = g_suite.ars_cfg.acc_noise_mps2;
        for (k = 0; k < 3; ++k)
        {
            g_suite.ahrs_cfg.gyr_bias_init_rps[k] = g_suite.ars_cfg.gyr_bias_init_rps[k];
            g_suite.ahrs_cfg.gyr_bias_init_stddev_rps[k] =
                g_suite.ars_cfg.gyr_bias_init_stddev_rps[k];
        }
    }

    /* baro_alt acc-bias drift density (0 -> baro_alt's own default). Set
       before the first baro sample latches g_suite.baro_cfg into the
       filter (nav_suite.h contract). */
    if (cfg.baro_acc_bias_rw > 0.0f)
    {
        g_suite.baro_cfg.acc_bias_drift_mps2_sqrthz = cfg.baro_acc_bias_rw;
    }
    if (cfg.baro_acc_bias_init_mps2 > 0.0f)
    {
        g_suite.baro_cfg.acc_bias_init_stddev_mps2 = cfg.baro_acc_bias_init_mps2;
    }
    if (cfg.baro_h_process_noise > 0.0f)
    {
        g_suite.baro_cfg.h_process_noise_m_sqrthz = cfg.baro_h_process_noise;
    }
    if (cfg.baro_acc_noise_mps2_sqrthz > 0.0f)
    {
        g_suite.baro_cfg.acc_noise_mps2_sqrthz = cfg.baro_acc_noise_mps2_sqrthz;
    }

    /* local-height/GNSS-ellipsoid offset filter tuning (0 ->
       local_gnss_alt's own defaults). Same latch-before-first-pair timing
       as g_suite.baro_cfg above. */
    if (cfg.local_gnss_rw_stddev_mps > 0.0f)
    {
        g_suite.local_gnss_cfg.rw_stddev_mps = cfg.local_gnss_rw_stddev_mps;
    }
    if (cfg.local_gnss_chi2_threshold > 0.0f)
    {
        g_suite.local_gnss_cfg.chi2_threshold = cfg.local_gnss_chi2_threshold;
    }
    if (cfg.local_gnss_min_update_interval_sec > 0.0f)
    {
        g_suite.local_gnss_cfg.min_update_interval_sec = cfg.local_gnss_min_update_interval_sec;
    }
    if (cfg.local_gnss_stddev_inflation_factor > 0.0f)
    {
        g_suite.local_gnss_cfg.stddev_inflation_factor = cfg.local_gnss_stddev_inflation_factor;
    }

    /* Magnetic model: both attitude sources must agree on the yaw datum
       (true north) - ins gets the full NED reference vector + field
       gate, the magnetometer AHRS its declination re-frame. */
    if (cfg.mag_enable && cfg.wmm_year > 0.0)
    {
        ins_set_magnetic_model_from_position(&g_suite.ins, g_ref[0].lat_rad, g_ref[0].lon_rad,
                                             (float)cfg.wmm_year);
        ahrs_set_position(&g_suite.ahrs, (float)g_ref[0].lat_rad, (float)g_ref[0].lon_rad,
                          (float)cfg.wmm_year);
    }
    else if (cfg.mag_enable)
    {
        /* Unlike insrcv this harness has no date source of its own, so a
           missing epoch means no reference field and every magnetometer
           sample gets rejected inside ins. Say so once up front instead of
           leaving it to a per-sample warning from the filter. */
        fprintf(stderr,
                "replay: WARNING mag.enable is set but mag.wmm_year is missing, no magnetic "
                "reference field is built and the magnetometer will not be fused "
                "(tools/inslib_convert_ubx_to_csv.py derives it from NAV-PVT)\n");
    }

    char delay_suffix[32];
    delay_suffix[0] = '\0';
    if (cfg.gnss_delay_ms > 0.0f)
    {
        snprintf(delay_suffix, sizeof(delay_suffix), ", gnss delay %g ms",
                 (double)cfg.gnss_delay_ms);
    }
    if (!cfg.gnss_enable)
    {
        printf("gnss: enable 0 - the fixes are read and counted, but none of them"
               " aids the filter\n");
    }
    printf("dataset %s: aiding=%s, init=%s, leverarm FRD [%.3f %.3f %.3f] m, "
           "warmup %g s%s%s%s%s\n",
           cfg.name, cfg.aiding, cfg.init, (double)cfg.gnss_leverarm_frd[0],
           (double)cfg.gnss_leverarm_frd[1], (double)cfg.gnss_leverarm_frd[2], cfg.warmup_sec,
           cfg.automotive_mode ? ", automotive" : "", cfg.mag_enable ? ", mag" : "",
           cfg.baro_enable ? ", baro" : "", delay_suffix);
    printf("initial gyro bias estimate: [%.4f %.4f %.4f] deg/s (%s)\n", RAD2DEG(gyr_bias0[0]),
           RAD2DEG(gyr_bias0[1]), RAD2DEG(gyr_bias0[2]),
           !have_bias0       ? "n/a"
           : bias0_corrected ? "from initial window, ref motion + earth/transport rate removed"
                             : "from initial window");

    /* Report the active chi2 outlier gate as a confidence level (1-alpha), so
       it's obvious at a glance whether outliers are gated at ~95% or ~99%.
       Each ins reference is fused scalar-row-wise, so the gate is 1-DOF and
       the confidence is P(X <= thr) = erf(sqrt(thr/2)). Mirrors src/ins.c
       INS_CHI2_GNSS/LOCAL_POS (10) and INS_CHI2_MAG/YAW (9). */
    if (cfg.chi2_disable) { printf("chi2 outlier gate: DISABLED (chi2_disable=1)\n"); }
    else if (cfg.chi2_reject_alpha > 0.0f)
    {
        double conf = 1.0 - (double)cfg.chi2_reject_alpha;
        printf("chi2 outlier gate: shared chi2inv(1-alpha, 1), 1-alpha = %.2f%% "
               "(alpha %.2f%%)\n",
               100.0 * conf, 100.0 * (double)cfg.chi2_reject_alpha);
    }
    else
    {
        double c_gnss = erf(sqrt(10.0 / 2.0));
        double c_mag  = erf(sqrt(9.0 / 2.0));
        printf("chi2 outlier gate (per-channel defaults, 1 DOF): "
               "GNSS/local-pos thr 10.0 -> %.2f%% (alpha %.2f%%); "
               "mag/yaw thr 9.0 -> %.2f%% (alpha %.2f%%)\n",
               100.0 * c_gnss, 100.0 * (1.0 - c_gnss), 100.0 * c_mag, 100.0 * (1.0 - c_mag));
    }

    /* --- replay ---------------------------------------------------------- */
    stat_t  nav_roll = {0}, nav_pitch = {0}, nav_yaw = {0}, nav_pos = {0};
    stat_t  ars_roll = {0}, ars_pitch = {0};
    stat_t  baro_h      = {0}; /* baro_alt height-above-start vs. ref height change */
    stat_t  baro_v      = {0}; /* baro_alt vertical velocity vs. ref (up positive) */
    double  baro_ref_h0 = 0.0, baro_bh0 = 0.0;
    int     baro_have_h0 = 0;
    /* nav_suite_get_height_ellipsoid() vs. ref->h_m directly (absolute, not
     * a since-anchor delta like baro_h above): reveals the offset filter's
     * own reconstruction error, which a since-anchor comparison would
     * cancel out. Populated whenever the accessor succeeds, independent of
     * cfg.baro_enable -- in FULL mode it just re-reads ins.latlonh[2], the
     * interesting case (a real divergence) only appears during
     * COASTING/ATTITUDE_ONLY with a barometer present. */
    stat_t ell_h = {0};
    double  ars_yaw_err0 = 0.0, ars_yaw_err_last = 0.0;
    int64_t ars_yaw_t0 = 0, ars_yaw_t_last = 0;
    int     ars_yaw_have0 = 0;

    const float var_pos_fallback_hor = cfg.pos_stddev_fallback_m[0] * cfg.pos_stddev_fallback_m[0];
    const float var_pos_fallback_ver = cfg.pos_stddev_fallback_m[1] * cfg.pos_stddev_fallback_m[1];
    const float var_vel_fallback     = cfg.vel_stddev_fallback_mps * cfg.vel_stddev_fallback_mps;

    const int64_t t_first_fix  = aiding_gnss ? g_gnss[0].t_us : g_ref[0].t_us;
    const int64_t t_warmup_end = t_first_fix + (int64_t)(cfg.warmup_sec * US_PER_SEC);

    /* Coasting re-acquisition (REQ-VER-029), aiding: gnss only -- the other
       aiding modes synthesize their fixes from the reference and have no
       outage of their own to measure. */
    coast_gap_t coast_gap[REPLAY_MAX_COAST_GAPS];
    const int   n_coast_gap =
        aiding_gnss ? coast_gaps_scan(g_gnss, n_gnss, cfg.coast_gap_min_sec, t_warmup_end,
                                        coast_gap, REPLAY_MAX_COAST_GAPS)
                      : 0;
    int i_coast_gap = 0; /* next gap still waiting for its reference epoch */

    int64_t       t_prev       = 0;
    int           iref = 0, ignss = 0, imag = 0, ibaro = 0, ispeed = 0;
    int64_t       fi_next_t_us = 0; /* next free_inertial_start offer */
    unsigned long n_fi_offers  = 0;
    long          n_imu = 0;
    char          line[512];

    /* Progress bar over the reference time span, on stderr and only when that
     * is an interactive terminal -- keeps piped/CI output (make test) clean. */
    const int64_t t_prog_beg    = g_ref[0].t_us;
    const int64_t t_prog_span   = g_ref[n_ref - 1].t_us - t_prog_beg;
    const int     show_progress = isatty(fileno(stderr));
    int           last_pct      = -1;

    while (fgets(line, sizeof(line), fimu))
    {
        long long t_ll;
        float     g[3], a[3];
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%f,%f,%f,%f,%f,%f", &t_ll, &g[0], &g[1], &g[2], &a[0], &a[1],
                   &a[2]) != 7)
        {
            continue;
        }
        const int64_t t = (int64_t)t_ll;
        n_imu++;

        if (show_progress && t_prog_span > 0)
        {
            int pct = (int)(100 * (t - t_prog_beg) / t_prog_span);
            if (pct < 0) { pct = 0; }
            if (pct > 100) { pct = 100; }
            if (pct != last_pct)
            {
                last_pct = pct;
                char bar[21];
                for (int k = 0; k < 20; ++k) { bar[k] = k < pct / 5 ? '#' : '-'; }
                bar[20] = '\0';
                fprintf(stderr, "\r  replay [%s] %3d%%", bar, pct);
                fflush(stderr);
            }
        }

        ins_measurements_t m;
        memset(&m, 0, sizeof(m));
        m.timestamp        = t;
        m.strapdown_dt_sec = (t_prev > 0) ? (float)(t - t_prev) / (float)US_PER_SEC : 0.0f;
        t_prev             = t;
        m.acc.is_valid     = true;
        m.gyr.is_valid     = true;
        int i;
        for (i = 0; i < 3; ++i)
        {
            m.acc.data[i]     = a[i];
            m.gyr.data[i]     = g[i];
            m.acc.Qll_diag[i] = cfg.acc_psd;
            m.gyr.Qll_diag[i] = cfg.gyr_psd;
        }

        /* --- position aiding --------------------------------------------- */
        const ref_epoch_t* ref = (const ref_epoch_t*)0;
        while (iref < n_ref && g_ref[iref].t_us <= t)
        {
            ref = &g_ref[iref];
            iref++;
        }

        /* Coasting re-acquisition (REQ-VER-029). Sampled HERE, before this
           epoch's measurements are fused: the whole point is the state the
           coasting left behind, and the first fix back would have snapped it
           onto the truth before the scoring block further down ever sees it.
           Taken at the first IMU epoch at or after the end of the gap, which
           is the epoch that fix is fed in, against the reference row nearest
           to it. Not gated on a reference row arriving in this epoch: shifted
           by ref_delay_ms (REQ-VER-030) the row for the gap end can land just
           before it, and waiting for the next row would sample after the
           returning fix has been fused. */
        if (i_coast_gap < n_coast_gap && t >= coast_gap[i_coast_gap].t_end_us)
        {
            coast_gap_t*       cg     = &coast_gap[i_coast_gap];
            int64_t            max_dt = (int64_t)(0.5 * cg->gap_sec * US_PER_SEC);
            const ref_epoch_t* ref_gap_end;
            if (max_dt > REPLAY_COAST_REF_MAX_DT_US) { max_dt = REPLAY_COAST_REF_MAX_DT_US; }
            ref_gap_end = coast_ref_near(t, iref, n_ref, max_dt);
            if (ref_gap_end != (const ref_epoch_t*)0)
            {
                cg->have_err = nav_pos_error_m(ref_gap_end, cfg.score_leverarm_frd, &cg->err_m);
                cg->resolved = 1;
            }
            i_coast_gap++;
        }

        if (aiding_gnss && cfg.gnss_enable)
        {
            /* Real GNSS epochs with the receiver's full covariance. */
            gnss_epoch_t* gm = (gnss_epoch_t*)0;
            while (ignss < n_gnss && g_gnss[ignss].t_us <= t)
            {
                gm = &g_gnss[ignss];
                ignss++;
            }
            if (gm != (gnss_epoch_t*)0 &&
                cov_apply_fallback(gm->Qpos_ned, var_pos_fallback_hor, var_pos_fallback_ver))
            {
                m.gnss_pos.is_valid = true;
                /* Handed over as the dataset holds it: the filter fuses the
                   geodetic difference, so this form needs no conversion on
                   either side (REQ-NAV-079). */
                m.gnss_pos.llh[0]       = gm->lat_rad;
                m.gnss_pos.llh[1]       = gm->lon_rad;
                m.gnss_pos.llh[2]       = gm->h_m;
                memcpy(m.gnss_pos.Qll_ned, gm->Qpos_ned, sizeof(m.gnss_pos.Qll_ned));
                if (gm->vel_ok &&
                    cov_apply_fallback(gm->Qvel_ned, var_vel_fallback, var_vel_fallback))
                {
                    /* In-motion alignment: without a velocity, yaw is
                       only weakly observable through the position
                       residuals. */
                    m.gnss_vel.is_valid = true;
                    memcpy(m.gnss_vel.vel_ned, gm->vel_ned, sizeof(m.gnss_vel.vel_ned));
                    memcpy(m.gnss_vel.Qll_ned, gm->Qvel_ned, sizeof(m.gnss_vel.Qll_ned));
                }
                for (i = 0; i < 3; ++i) { m.gnss_leverarm_b[i] = cfg.gnss_leverarm_frd[i]; }
            }
        }
        else if (aiding_ref && cfg.gnss_enable && ref != (const ref_epoch_t*)0)
        {
            /* Fix synthesized from the reference (NOT a
               real GNSS error profile). aiding_none skips this branch
               entirely: ins never gets a fix, so it stays
               uninitialized (mode NONE/ATTITUDE_ONLY) for the whole
               replay - the ARS and baro_alt filters don't need one and
               keep running regardless (see nav_suite.h). Noise is
               gnss.pos_stddev_fallback_m/vel_stddev_fallback_mps (same
               fields a real fix's zero/unknown covariance diagonals fall
               back to): this synthesized fix never has a reported
               covariance of its own either, so it is exactly that case. */
            m.gnss_pos.is_valid = true;
            m.gnss_pos.llh[0]       = ref->lat_rad;
            m.gnss_pos.llh[1]       = ref->lon_rad;
            m.gnss_pos.llh[2]       = ref->h_m;
            m.gnss_pos.Qll_ned[0] = var_pos_fallback_hor;
            m.gnss_pos.Qll_ned[4] = var_pos_fallback_hor;
            m.gnss_pos.Qll_ned[8] = var_pos_fallback_ver;
            m.gnss_vel.is_valid   = true;
            for (i = 0; i < 3; ++i)
            {
                m.gnss_vel.vel_ned[i]         = ref->vel_ned[i];
                m.gnss_vel.Qll_ned[i * 3 + i] = var_vel_fallback;
                m.gnss_leverarm_b[i]          = cfg.gnss_leverarm_frd[i];
            }
        }
        /* free_inertial_start: the declared origin, offered as a position
           measurement until ins has bootstrapped from it and not one epoch
           longer - a source that kept repeating the same point would pin
           the solution to it instead of dead reckoning. Never in an epoch
           that already carries a real fix: a measured position beats a
           declared one. Mirrors fi_offer_start_position() in insrcv.c, so
           the same config.yaml produces the same start either way. */
        if (cfg.fi_enable && !m.gnss_pos.is_valid && ins_deadreckoning_ms(&g_suite.ins) < 0 &&
            t >= fi_next_t_us)
        {
            fi_next_t_us = t + US_PER_SEC / 2; /* 2 Hz: the entry dwell wants >= 1 */
            m.gnss_pos.llh[0]       = DEG2RAD(cfg.fi_lat_deg);
            m.gnss_pos.llh[1]       = DEG2RAD(cfg.fi_lon_deg);
            m.gnss_pos.llh[2]       = cfg.fi_height_m;
            const float fi_var    = cfg.fi_stddev_m * cfg.fi_stddev_m;
            m.gnss_pos.Qll_ned[0] = fi_var;
            m.gnss_pos.Qll_ned[4] = fi_var;
            m.gnss_pos.Qll_ned[8] = fi_var;
            m.gnss_pos.is_valid   = true;
            n_fi_offers++;
        }

        /* Assumed fixed processing/telemetry latency (REQ-VER-008),
           regardless of aiding source: history-anchors the fusion via
           the existing gnss_delay_ms mechanism (ins.c). */
        if (m.gnss_pos.is_valid) { m.gnss_delay_ms = (int)cfg.gnss_delay_ms; }

        /* --- magnetometer / barometer ------------------------------------ */
        if (cfg.mag_enable)
        {
            const mag_epoch_t* mm = (const mag_epoch_t*)0;
            while (imag < n_mag && g_mag[imag].t_us <= t)
            {
                mm = &g_mag[imag];
                imag++;
            }
            if (mm != (const mag_epoch_t*)0)
            {
                /* 0 -> ins's own magnetometer default (sensor_defaults.h),
                   same as tools/insrcv.c: the number lives in one place. */
                const float var = cfg.mag_stddev_ut * cfg.mag_stddev_ut;
                m.mag.is_valid  = true;
                for (i = 0; i < 3; ++i)
                {
                    m.mag.data[i]     = mm->mag_ut[i];
                    m.mag.Qll_diag[i] = var;
                }
            }
        }
        if (cfg.baro_enable)
        {
            const baro_epoch_t* bm = (const baro_epoch_t*)0;
            while (ibaro < n_baro && g_baro[ibaro].t_us <= t)
            {
                bm = &g_baro[ibaro];
                ibaro++;
            }
            if (bm != (const baro_epoch_t*)0)
            {
                m.baro.is_valid    = true;
                m.baro.pressure_pa = bm->pressure_pa;
                m.baro.stddev_m    = cfg.baro_stddev_m;
            }
        }
        if (cfg.speed_enable)
        {
            const speed_epoch_t* sm = (const speed_epoch_t*)0;
            while (ispeed < n_speed && g_speed[ispeed].t_us <= t)
            {
                sm = &g_speed[ispeed];
                ispeed++;
            }
            if (sm != (const speed_epoch_t*)0)
            {
                m.speed.is_valid   = true;
                m.speed.speed_mps  = sm->speed_mps;
                m.speed.stddev_mps = cfg.speed_stddev_mps;
                m.speed_delay_ms   = cfg.speed_delay_ms;
            }
        }

        nav_suite_update(&g_suite, &m);

        /* --- scoring at reference epochs -------------------------------- */
        if (ref == (const ref_epoch_t*)0) { continue; }
        const int scored   = (t >= t_warmup_end);
        double    e_nav[4] = {0, 0, 0, 0}; /* roll,pitch,yaw [deg], pos [m] */
        double    e_ars[3] = {0, 0, 0};    /* roll,pitch,yaw [deg] */
        int       have_nav = 0, have_ars = 0;

        float roll, pitch, yaw;
        if (ins_is_ready(&g_suite.ins) && nav_suite_get_rpy_ins(&g_suite, &roll, &pitch, &yaw))
        {
            e_nav[0] = RAD2DEG(wrap_pi_d(roll - ref->roll_rad));
            e_nav[1] = RAD2DEG(wrap_pi_d(pitch - ref->pitch_rad));
            e_nav[2] = RAD2DEG(wrap_pi_d(yaw - ref->yaw_rad));
            /* The scoring lever arm maps the filter position onto the
               ground-truth point (zero when the truth refers to the IMU
               center). Same helper the coasting metric uses, so the two can
               never drift apart. */
            (void)nav_pos_error_m(ref, cfg.score_leverarm_frd, &e_nav[3]);
            have_nav = 1;
            if (scored)
            {
                stat_add(&nav_roll, e_nav[0]);
                stat_add(&nav_pitch, e_nav[1]);
                stat_add(&nav_yaw, e_nav[2]);
                stat_add(&nav_pos, e_nav[3]);
            }
        }

        if (cfg.baro_enable)
        {
            float bh, bv;
            if (nav_suite_get_baro_alt(&g_suite, &bh, &bv))
            {
                /* ref vertical velocity is NED-down; baro_alt is up. */
                if (scored) { stat_add(&baro_v, (double)bv - (-(double)ref->vel_ned[2])); }
                /* baro_alt reports height above its own anchor; score the
                   CHANGE in that height against the reference height change
                   since the anchor epoch (offset-free, so a nonzero anchor
                   height h_init does not bias the metric). */
                if (!baro_have_h0)
                {
                    baro_ref_h0  = ref->h_m;
                    baro_bh0     = bh;
                    baro_have_h0 = 1;
                }
                const double e = ((double)bh - baro_bh0) - (ref->h_m - baro_ref_h0);
                if (scored) { stat_add(&baro_h, e); }
            }
        }

        {
            float ell_h_m;
            if (scored && nav_suite_get_height_ellipsoid(&g_suite, &ell_h_m))
            {
                stat_add(&ell_h, (double)ell_h_m - ref->h_m);
            }
        }

        if (cfg.score_ahrs && ahrs_get_rpy(&g_suite.ars, &roll, &pitch, &yaw))
        {
            e_ars[0] = RAD2DEG(wrap_pi_d(roll - ref->roll_rad));
            e_ars[1] = RAD2DEG(wrap_pi_d(pitch - ref->pitch_rad));
            e_ars[2] = RAD2DEG(wrap_pi_d(yaw - ref->yaw_rad));
            have_ars = 1;
            if (scored)
            {
                stat_add(&ars_roll, e_ars[0]);
                stat_add(&ars_pitch, e_ars[1]);
                /* Free yaw: track the drift of the (arbitrary-offset)
                   yaw error over the scored period. */
                const double yerr = e_ars[2] * M_PI / 180.0;
                if (!ars_yaw_have0)
                {
                    ars_yaw_err0  = yerr;
                    ars_yaw_t0    = t;
                    ars_yaw_have0 = 1;
                }
                ars_yaw_err_last = yerr;
                ars_yaw_t_last   = t;
            }
        }

        if (fdump && (have_nav || have_ars))
        {
            const double speed = sqrt((double)ref->vel_ned[0] * ref->vel_ned[0] +
                                      (double)ref->vel_ned[1] * ref->vel_ned[1] +
                                      (double)ref->vel_ned[2] * ref->vel_ned[2]);
            fprintf(fdump, "%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    (double)(t - t_first_fix) / (double)US_PER_SEC, speed, e_nav[0], e_nav[1],
                    e_nav[2], e_nav[3], e_ars[0], e_ars[1], e_ars[2]);
        }
    }
    fclose(fimu);
    if (fdump) fclose(fdump);
    if (show_progress) { fprintf(stderr, "\r  replay [####################] 100%%\n"); }

    /* --- report ----------------------------------------------------------- */
    const ins_diag_t* diag = ins_get_diag(&g_suite.ins);
    printf("replayed %ld IMU samples, %d GNSS fixes, %d reference epochs"
           "%s%s (scored after %g s warmup)\n",
           n_imu, n_gnss, n_ref, cfg.mag_enable ? ", mag" : "", cfg.baro_enable ? ", baro" : "",
           cfg.warmup_sec);
    if (cfg.fi_enable)
    {
        printf("free-inertial start: %lu declaration(s) offered at %.7f %.7f h=%.1f m"
               " +-%g m, then the filter is on its own\n",
               n_fi_offers, cfg.fi_lat_deg, cfg.fi_lon_deg, cfg.fi_height_m,
               (double)cfg.fi_stddev_m);
    }
    printf("ins: %u predicts, %u gnss fusions (%u seen), %u fuse fails, "
           "%u auto-zupt, %u downweighted\n",
           diag->n_predict, diag->n_gnss_used, diag->n_gnss_seen, diag->n_fuse_fail,
           diag->n_auto_zupt, diag->n_downweighted);
    printf("downweighted (chi2 outlier): ars %u, ahrs %u, baro_alt %u, "
           "local_gnss_offset %u\n",
           g_suite.ars.n_downweighted, g_suite.ahrs.n_downweighted, g_suite.baro_alt.n_downweighted,
           g_suite.local_gnss.n_downweighted);
    printf("\nins vs ground truth:\n");
    if (cfg.score_attitude)
    {
        stat_print("roll error", &nav_roll, "deg");
        stat_print("pitch error", &nav_pitch, "deg");
        stat_print("yaw error", &nav_yaw, "deg");
    }
    else
    {
        printf("  (attitude not scored: this dataset declares no attitude reference)\n");
    }
    stat_print("pos error", &nav_pos, "m  ");

    if (cfg.baro_enable)
    {
        printf("\nbaro_alt vertical channel vs ground truth:\n");
        stat_print("rel-height error", &baro_h, "m  ");
        stat_print("vert-velocity error", &baro_v, "m/s");
    }
    if (ell_h.n > 0)
    {
        printf("\nnav_suite_get_height_ellipsoid() vs ground truth (absolute, "
               "not a since-anchor delta):\n");
        stat_print("ellipsoid height error", &ell_h, "m  ");
    }

    if (n_coast_gap > 0)
    {
        int k;
        printf("\ncoasting re-acquisition (aiding gaps > %.0f s, error when each one "
               "ends):\n",
               cfg.coast_gap_min_sec);
        for (k = 0; k < n_coast_gap; ++k)
        {
            const coast_gap_t* cg = &coast_gap[k];
            printf("  gap %d: %6.1f s, %7.0f m travelled ->  ", k + 1, cg->gap_sec, cg->chord_m);
            if (!cg->resolved) { printf("no reference epoch near its end\n"); }
            else if (!cg->have_err) { printf("NO SOLUTION (filter gave the coast up)\n"); }
            else
            {
                printf("%8.1f m", cg->err_m);
                if (cg->chord_m > 1.0) { printf("  (%.1f %% of the distance)", 100.0 * cg->err_m / cg->chord_m); }
                printf("\n");
            }
        }
    }

    double ars_drift_deg_min = 0.0;
    if (cfg.score_ahrs)
    {
        printf("\nARS (roll/pitch, IMU-only) vs ground truth:\n");
        stat_print("roll error", &ars_roll, "deg");
        stat_print("pitch error", &ars_pitch, "deg");
        if (ars_yaw_have0 && ars_yaw_t_last > ars_yaw_t0)
        {
            ars_drift_deg_min = RAD2DEG(wrap_pi_d(ars_yaw_err_last - ars_yaw_err0)) /
                                ((double)(ars_yaw_t_last - ars_yaw_t0) / (60.0 * US_PER_SEC));
        }
        printf("  %-24s %+8.3f deg/min (free yaw, offset arbitrary)\n", "yaw drift",
               ars_drift_deg_min);
    }
    if (g_suite.ahrs.is_initialized && !cfg.mag_enable)
    {
        printf("\nmag-AHRS ran (unexpected: dataset has no magnetometer)\n");
    }

    /* --- pass/fail gates --------------------------------------------------
     * Per-dataset limits from config.yaml: ~2x the values of a
     * known-good run (regression gates, not accuracy claims). Mean
     * attitude errors include the IMU<->vehicle mounting misalignment
     * of the dataset. */
    printf("\nchecks:\n");
    CHECK_LIMIT(nav_roll.n >= cfg.min_epochs ? 0.0 : 1.0, 0.5, "enough scored epochs");
    if (cfg.score_attitude)
    {
        CHECK_LIMIT(fabs(stat_mean(&nav_roll)), cfg.lim_att_bias_deg, "ins |roll bias| [deg]");
        CHECK_LIMIT(fabs(stat_mean(&nav_pitch)), cfg.lim_att_bias_deg, "ins |pitch bias| [deg]");
        CHECK_LIMIT(stat_std(&nav_roll), cfg.lim_att_std_deg, "ins roll stddev [deg]");
        CHECK_LIMIT(stat_std(&nav_pitch), cfg.lim_att_std_deg, "ins pitch stddev [deg]");
        CHECK_LIMIT(fabs(stat_mean(&nav_yaw)), cfg.lim_yaw_bias_deg, "ins |yaw bias| [deg]");
        CHECK_LIMIT(stat_std(&nav_yaw), cfg.lim_yaw_std_deg, "ins yaw stddev [deg]");
    }
    else
    {
        printf("  skip  %-44s: no attitude reference in this dataset\n", "ins attitude gates");
    }
    /* 0 -> not gated, the convention the manual states for every lim_* key
       and the one datasets/check_simulated.py already implements. This one
       used to be checked unconditionally, so a 0 meant "must be below zero"
       and failed every run: the way to leave it ungated was an arbitrary
       large number (datasets/fog/config_pyahrs.yaml still carries a 999). */
    if (cfg.lim_pos_rms_m > 0.0)
    {
        CHECK_LIMIT(stat_rms(&nav_pos), cfg.lim_pos_rms_m, "ins pos rms [m]");
    }
    if (cfg.lim_coast_exit_err_m > 0.0)
    {
        int k;
        for (k = 0; k < n_coast_gap; ++k)
        {
            const coast_gap_t* cg = &coast_gap[k];
            char               msg[64];
            snprintf(msg, sizeof(msg), "coast %.0f s: re-acquisition error [m]", cg->gap_sec);
            /* No solution at that epoch is a failure of its own, not a
               number above the limit (REQ-VER-029). */
            if (!cg->resolved || !cg->have_err)
            {
                printf("  FAIL  %-44s: no solution when aiding returned\n", msg);
                fails++;
            }
            else { CHECK_LIMIT(cg->err_m, cfg.lim_coast_exit_err_m, msg); }
        }
        if (n_coast_gap == 0)
        {
            printf("  FAIL  %-44s: no aiding gap longer than %g s\n",
                   "coast re-acquisition gate", cfg.coast_gap_min_sec);
            fails++;
        }
    }
    if (cfg.baro_enable && cfg.lim_baro_rms_m > 0.0)
    {
        CHECK_LIMIT(stat_rms(&baro_h), cfg.lim_baro_rms_m, "baro rel-height rms [m]");
    }
    if (cfg.baro_enable && cfg.lim_baro_bias_m > 0.0)
    {
        CHECK_LIMIT(fabs(stat_mean(&baro_h)), cfg.lim_baro_bias_m, "baro |rel-height bias| [m]");
    }
    if (cfg.baro_enable && cfg.lim_baro_max_m > 0.0)
    {
        CHECK_LIMIT(baro_h.max_abs, cfg.lim_baro_max_m, "baro rel-height max |err| [m]");
    }
    if (ell_h.n > 0 && cfg.lim_ellipsoid_rms_m > 0.0)
    {
        CHECK_LIMIT(stat_rms(&ell_h), cfg.lim_ellipsoid_rms_m, "ellipsoid height rms [m]");
    }
    if (ell_h.n > 0 && cfg.lim_ellipsoid_bias_m > 0.0)
    {
        CHECK_LIMIT(fabs(stat_mean(&ell_h)), cfg.lim_ellipsoid_bias_m, "ellipsoid |height bias| [m]");
    }
    if (ell_h.n > 0 && cfg.lim_ellipsoid_max_m > 0.0)
    {
        CHECK_LIMIT(ell_h.max_abs, cfg.lim_ellipsoid_max_m, "ellipsoid height max |err| [m]");
    }
    if (cfg.score_ahrs)
    {
        CHECK_LIMIT(fabs(stat_mean(&ars_roll)), cfg.lim_ars_att_bias_deg, "ars |roll bias| [deg]");
        CHECK_LIMIT(fabs(stat_mean(&ars_pitch)), cfg.lim_ars_att_bias_deg,
                    "ars |pitch bias| [deg]");
        CHECK_LIMIT(stat_std(&ars_roll), cfg.lim_ars_roll_std_deg, "ars roll stddev [deg]");
        CHECK_LIMIT(stat_std(&ars_pitch), cfg.lim_ars_pitch_std_deg, "ars pitch stddev [deg]");
        CHECK_LIMIT(fabs(ars_drift_deg_min), cfg.lim_ars_yaw_drift_deg_min,
                    "ars |yaw drift| [deg/min]");
    }

    printf("\n==== %d failures ====\n", fails);
    return fails == 0 ? 0 : 1;
}
