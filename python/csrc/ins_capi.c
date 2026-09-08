/** @file ins_capi.c
 * @author Jan Zwiener (jan@zwiener.org)
 *  Implementation of the flat C ABI shim (see ins_capi.h).
 *  Lives under python/ -- outside the src/ requirements process. */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ins.h"
#include "nav_suite.h"
#include "ahrs.h"             /* ahrs_set_position (WMM true north) */
#include "geodetic_toolbox.h" /* ins_latlonh_to_ecef */
#include "ins_capi.h"

_Static_assert(INS_CAPI_MAX_STATE == INS_UNKNOWNS_MAG,
               "INS_CAPI_MAX_STATE must mirror ins.h's INS_UNKNOWNS_MAG");

/* Each handle carries its filter plus one pending measurement, so the
 * setter functions can accumulate an epoch across several calls before
 * _update() fuses it. */
typedef struct
{
    ins_t              filter;
    ins_measurements_t meas;
} ins_core_ctx_t;
typedef struct
{
    nav_suite_t        suite;
    ins_measurements_t meas;
} ins_suite_ctx_t;

/* ---- shared config + measurement helpers (used by both families) ------- */

static void cfg_to_init_opt(const ins_cfg_t* cfg, ins_init_t* init, ins_options_t* opt)
{
    memset(init, 0, sizeof(*init));
    init->time = cfg->time_us;
    ins_latlonh_to_ecef(cfg->lat_rad, cfg->lon_rad, cfg->h_m, init->x_ecef);
    /* Initial velocity (NED -> ECEF) for a non-stationary manual start. */
    {
        float R_n_to_e[9];
        int   k;
        ins_rotmat_n_to_e(cfg->lat_rad, cfg->lon_rad, R_n_to_e);
        for (k = 0; k < 3; ++k)
        {
            init->xdot_ecef[k] = (double)R_n_to_e[k] * (double)cfg->init_vel_ned[0] +
                                 (double)R_n_to_e[k + 3] * (double)cfg->init_vel_ned[1] +
                                 (double)R_n_to_e[k + 6] * (double)cfg->init_vel_ned[2];
        }
    }
    init->pos_init_stddev_m               = cfg->pos_init_stddev_m;
    init->vel_init_stddev_mps             = cfg->vel_init_stddev_mps;
    init->rpy_init_stddev_rad[0]          = cfg->rpy_init_stddev_rad[0];
    init->rpy_init_stddev_rad[1]          = cfg->rpy_init_stddev_rad[1];
    init->rpy_init_stddev_rad[2]          = cfg->rpy_init_stddev_rad[2];
    init->acc_bias_init_stddev_mps2       = cfg->acc_bias_init_stddev_mps2;
    init->gyr_bias_init_stddev_rps        = cfg->gyr_bias_init_stddev_rps;
    init->pos_pred_stddev_m_sqrts         = cfg->pos_pred_stddev_m_sqrts;
    init->vel_pred_stddev_mps_sqrts       = cfg->vel_pred_stddev_mps_sqrts;
    init->rpy_pred_stddev_rad_sqrts       = cfg->rpy_pred_stddev_rad_sqrts;
    init->acc_bias_pred_stddev_mps2_sqrts = cfg->acc_bias_pred_stddev_mps2_sqrts;
    init->gyr_bias_pred_stddev_rps_sqrts  = cfg->gyr_bias_pred_stddev_rps_sqrts;
    init->zero_vel_stddev_mps             = cfg->zero_vel_stddev_mps;
    init->zero_rot_stddev_rps             = cfg->zero_rot_stddev_rps;
    init->gyr_bias_init_rps[0]            = cfg->gyr_bias_init_rps[0];
    init->gyr_bias_init_rps[1]            = cfg->gyr_bias_init_rps[1];
    init->gyr_bias_init_rps[2]            = cfg->gyr_bias_init_rps[2];
    init->magnetic_n[0]                   = cfg->magnetic_n[0];
    init->magnetic_n[1]                   = cfg->magnetic_n[1];
    init->magnetic_n[2]                   = cfg->magnetic_n[2];
    init->rpy_init_rad[0]                 = cfg->rpy_init_rad[0];
    init->rpy_init_rad[1]                 = cfg->rpy_init_rad[1];
    init->rpy_init_rad[2]                 = cfg->rpy_init_rad[2];

    memset(opt, 0, sizeof(*opt));
    opt->kalman_update_dt_sec               = cfg->kalman_update_dt_sec;
    opt->max_prediction_time_sec            = cfg->max_prediction_time_sec;
    opt->gnss_max_horizontal_pos_stddev_m   = cfg->gnss_max_horizontal_pos_stddev_m;
    opt->gnss_max_vertical_pos_stddev_m     = cfg->gnss_max_vertical_pos_stddev_m;
    opt->gnss_max_horizontal_vel_stddev_mps = cfg->gnss_max_horizontal_vel_stddev_mps;
    opt->gnss_max_vertical_vel_stddev_mps   = cfg->gnss_max_vertical_vel_stddev_mps;
    opt->magnetometer_min_delay_ms          = cfg->magnetometer_min_delay_ms;
    opt->auto_init                          = (cfg->auto_init != 0);
    opt->allow_unlimited_deadreckoning      = (cfg->allow_unlimited_deadreckoning != 0);
    opt->max_deadreckoning_sec              = cfg->max_deadreckoning_sec;
    opt->auto_zupt_disable                  = (cfg->auto_zupt_disable != 0);
    opt->mag_field_tolerance                = cfg->mag_field_tolerance;
    opt->mag_field_check_disable            = (cfg->mag_field_check_disable != 0);
    opt->estimate_mag_bias                  = (cfg->estimate_mag_bias != 0);
    init->mag_bias_init_stddev_ut           = cfg->mag_bias_init_stddev_ut;
    init->mag_bias_pred_stddev_ut_sqrts     = cfg->mag_bias_pred_stddev_ut_sqrts;
    opt->automotive_mode                    = (cfg->automotive_mode != 0);
    opt->automotive_min_speed_mps           = cfg->automotive_min_speed_mps;
    opt->automotive_min_yaw_stddev          = cfg->automotive_min_yaw_stddev;
    opt->chi2_disable                       = (cfg->chi2_disable != 0);
    opt->chi2_reject_alpha                  = cfg->chi2_reject_alpha;
    opt->auto_init_window_sec               = cfg->auto_init_window_sec;
    opt->baro_height_disable                = (cfg->baro_height_disable != 0);
    /* Stillness definition, forwarded as one set (REQ-SUITE-020): for a
       suite handle nav_suite_init() passes it on to the ARS/AHRS and
       baro_alt, for a bare ins handle only ins itself uses it. */
    opt->auto_zupt_static_gyr_rps         = cfg->auto_zupt_static_gyr_rps;
    opt->auto_zupt_static_acc_mps2        = cfg->auto_zupt_static_acc_mps2;
    opt->auto_zupt_max_vel_mps            = cfg->auto_zupt_max_vel_mps;
    opt->auto_zupt_max_vel_stddev_mps     = cfg->auto_zupt_max_vel_stddev_mps;
    opt->auto_zupt_static_gyr_stddev_rps  = cfg->auto_zupt_static_gyr_stddev_rps;
    opt->auto_zupt_static_acc_stddev_mps2 = cfg->auto_zupt_static_acc_stddev_mps2;
    opt->auto_zupt_dwell_sec              = cfg->auto_zupt_dwell_sec;
    opt->auto_zupt_min_interval_sec       = cfg->auto_zupt_min_interval_sec;
    opt->auto_zupt_velocity_blind_disable = (cfg->auto_zupt_velocity_blind_disable != 0);
    opt->speed_scale                      = cfg->speed_scale;
    opt->speed_stddev_rel                 = cfg->speed_stddev_rel;
    opt->speed_min_mps                    = cfg->speed_min_mps;
    opt->gnss_pos_stddev_cap_hor_m        = cfg->gnss_pos_stddev_cap_hor_m;
    opt->gnss_pos_stddev_cap_ver_m        = cfg->gnss_pos_stddev_cap_ver_m;
    opt->gnss_vel_stddev_cap_hor_mps      = cfg->gnss_vel_stddev_cap_hor_mps;
    opt->gnss_vel_stddev_cap_ver_mps      = cfg->gnss_vel_stddev_cap_ver_mps;
    opt->gnss_acc_envelope_tau_sec        = cfg->gnss_acc_envelope_tau_sec;
    opt->gnss_vel_noise_acc_scale_hor     = cfg->gnss_vel_noise_acc_scale_hor;
    opt->gnss_vel_noise_acc_scale_ver     = cfg->gnss_vel_noise_acc_scale_ver;
    opt->gnss_vel_noise_acc_window_sec    = cfg->gnss_vel_noise_acc_window_sec;

    /* IMU calibration (REQ-NAV-037) + GNSS covariance conditioning
       (REQ-NAV-038): forwarded verbatim (all-0 / 0 sentinels are the
       library's own no-op defaults). */
    memcpy(opt->imu_acc_misalignment, cfg->imu_acc_misalignment, sizeof(opt->imu_acc_misalignment));
    memcpy(opt->imu_gyr_misalignment, cfg->imu_gyr_misalignment, sizeof(opt->imu_gyr_misalignment));
    memcpy(opt->imu_acc_fixed_bias, cfg->imu_acc_fixed_bias, sizeof(opt->imu_acc_fixed_bias));
    memcpy(opt->imu_gyr_fixed_bias, cfg->imu_gyr_fixed_bias, sizeof(opt->imu_gyr_fixed_bias));
    opt->gnss_pos_cov_scale            = cfg->gnss_pos_cov_scale;
    opt->gnss_pos_cov_scale_height     = cfg->gnss_pos_cov_scale_height;
    opt->gnss_vel_cov_scale            = cfg->gnss_vel_cov_scale;
    opt->gnss_pos_stddev_floor_hor_m   = cfg->gnss_pos_stddev_floor_hor_m;
    opt->gnss_pos_stddev_floor_ver_m   = cfg->gnss_pos_stddev_floor_ver_m;
    opt->gnss_vel_stddev_floor_hor_mps = cfg->gnss_vel_stddev_floor_hor_mps;
    opt->gnss_vel_stddev_floor_ver_mps = cfg->gnss_vel_stddev_floor_ver_mps;
    memcpy(opt->mag_misalignment, cfg->mag_misalignment, sizeof(opt->mag_misalignment));
    memcpy(opt->mag_fixed_bias, cfg->mag_fixed_bias, sizeof(opt->mag_fixed_bias));

    /* GNSS quality hysteresis around the 3D solution (REQ-NAV-051/052). */
    opt->gnss_start_max_horizontal_pos_stddev_m   = cfg->gnss_start_max_horizontal_pos_stddev_m;
    opt->gnss_start_max_vertical_pos_stddev_m     = cfg->gnss_start_max_vertical_pos_stddev_m;
    opt->gnss_start_max_horizontal_vel_stddev_mps = cfg->gnss_start_max_horizontal_vel_stddev_mps;
    opt->gnss_start_max_vertical_vel_stddev_mps   = cfg->gnss_start_max_vertical_vel_stddev_mps;
    opt->gnss_stop_max_horizontal_pos_stddev_m    = cfg->gnss_stop_max_horizontal_pos_stddev_m;
    opt->gnss_stop_max_vertical_pos_stddev_m      = cfg->gnss_stop_max_vertical_pos_stddev_m;
    opt->gnss_stop_max_horizontal_vel_stddev_mps  = cfg->gnss_stop_max_horizontal_vel_stddev_mps;
    opt->gnss_stop_max_vertical_vel_stddev_mps    = cfg->gnss_stop_max_vertical_vel_stddev_mps;
    opt->gnss_init_dwell_sec                      = cfg->gnss_init_dwell_sec;
    opt->gnss_init_dwell_disable                  = (cfg->gnss_init_dwell_disable != 0);
    opt->gnss_stop_dwell_sec                      = cfg->gnss_stop_dwell_sec;
    opt->gnss_stop_disable                        = (cfg->gnss_stop_disable != 0);
    opt->gnss_pos_decimation                      = (int)cfg->gnss_pos_decimation;
    opt->gnss_min_delay_ms                        = (int)cfg->gnss_min_delay_ms;
}

static void meas_baro(ins_measurements_t* m, float pressure_pa, float stddev_m)
{
    m->baro.is_valid    = true;
    m->baro.pressure_pa = pressure_pa;
    m->baro.stddev_m    = stddev_m;
}

/* Absolute speed aiding (REQ-NAV-068): the scalar |v| of an OBD-II vehicle
   speed, wheel odometry or Doppler log. delay_ms states how old the sample
   is, anchored in the state history exactly like a delayed GNSS fix. */
static void meas_speed(ins_measurements_t* m, float speed_mps, float stddev_mps, int delay_ms)
{
    m->speed.is_valid   = true;
    m->speed.speed_mps  = speed_mps;
    m->speed.stddev_mps = stddev_mps;
    m->speed_delay_ms   = delay_ms;
}

static void meas_imu(ins_measurements_t* m, int64_t t_us, float dt, const float acc[3],
                     const float gyr[3], const float acc_var[3], const float gyr_var[3])
{
    memset(m, 0, sizeof(*m)); /* IMU begins a fresh epoch */
    m->timestamp        = t_us;
    m->strapdown_dt_sec = dt;
    m->acc.is_valid     = true;
    m->gyr.is_valid     = true;
    int i;
    for (i = 0; i < 3; ++i)
    {
        m->acc.data[i]     = acc[i];
        m->gyr.data[i]     = gyr[i];
        m->acc.Qll_diag[i] = acc_var[i];
        m->gyr.Qll_diag[i] = gyr_var[i];
    }
}

static void meas_gnss_pos(ins_measurements_t* m, const double ecef[3], const float var_ned[3])
{
    m->gnss_pos.is_valid    = true;
    m->gnss_pos.xyz_ecef[0] = ecef[0];
    m->gnss_pos.xyz_ecef[1] = ecef[1];
    m->gnss_pos.xyz_ecef[2] = ecef[2];
    m->gnss_pos.Qll_ned[0]  = var_ned[0];
    m->gnss_pos.Qll_ned[4]  = var_ned[1];
    m->gnss_pos.Qll_ned[8]  = var_ned[2];
}

static void meas_gnss_vel(ins_measurements_t* m, const float vel_ned[3], const float var_ned[3])
{
    m->gnss_vel.is_valid = true;
    int i;
    for (i = 0; i < 3; ++i)
    {
        m->gnss_vel.vel_ned[i]         = vel_ned[i];
        m->gnss_vel.Qll_ned[i * 3 + i] = var_ned[i];
    }
}

static void meas_gnss_pos_cov(ins_measurements_t* m, const double ecef[3], const float Qll_ned[9])
{
    m->gnss_pos.is_valid    = true;
    m->gnss_pos.xyz_ecef[0] = ecef[0];
    m->gnss_pos.xyz_ecef[1] = ecef[1];
    m->gnss_pos.xyz_ecef[2] = ecef[2];
    memcpy(m->gnss_pos.Qll_ned, Qll_ned, sizeof(float) * 9);
}

static void meas_gnss_vel_cov(ins_measurements_t* m, const float vel_ned[3], const float Qll_ned[9])
{
    m->gnss_vel.is_valid = true;
    memcpy(m->gnss_vel.vel_ned, vel_ned, sizeof(float) * 3);
    memcpy(m->gnss_vel.Qll_ned, Qll_ned, sizeof(float) * 9);
}

static void meas_mag(ins_measurements_t* m, const float mag[3], const float var[3])
{
    m->mag.is_valid = true;
    int i;
    for (i = 0; i < 3; ++i)
    {
        m->mag.data[i]     = mag[i];
        m->mag.Qll_diag[i] = var[i];
    }
}

static void meas_yaw(ins_measurements_t* m, float yaw_rad, float stddev_rad)
{
    m->yaw.is_valid   = true;
    m->yaw.yaw_rad    = yaw_rad;
    m->yaw.stddev_rad = stddev_rad;
}

static void meas_local_pos(ins_measurements_t* m, const float pos_ned[3], const float var_ned[3],
                           const float lever_b[3])
{
    m->local_pos.is_valid = true;
    int i;
    for (i = 0; i < 3; ++i)
    {
        m->local_pos.pos_ned[i]         = pos_ned[i];
        m->local_pos.Qll_ned[i * 3 + i] = var_ned[i];
        m->local_pos_leverarm_b[i]      = lever_b[i];
    }
}

/* The local NED frame origin (ECEF), valid once the filter is initialized. */
static int origin_of(const ins_t* f, double o[3])
{
    if (!f->is_initialized) return 0;
    o[0] = f->origin_ecef[0];
    o[1] = f->origin_ecef[1];
    o[2] = f->origin_ecef[2];
    return 1;
}

/* Marginal variance of every state from its UDU factors (U unit upper
 * triangular, column-major, leading dimension n): diag(P)_i = sum_{k=i}^{n-1}
 * U[i,k]^2 * d[k]. Shared by the smaller ahrs_t (5/6-state) and baro_alt_t
 * (3-state) accessors below, which only ever need the diagonal, not the
 * full off-diagonal covariance ins's own covariance_of() computes. */
static void diag_covariance_of(const float* U, const float* d, int n, float* diag_out)
{
    int i, k;
    for (i = 0; i < n; ++i)
    {
        float s = 0.0f;
        for (k = i; k < n; ++k) { s += U[i + k * n] * U[i + k * n] * d[k]; }
        diag_out[i] = s;
    }
}

/* Full covariance P = U*diag(d)*U' from the filter's UDU factors (U unit
 * upper triangular, leading dimension f->n -- mirrors ins.c's static
 * udu_get_diag(), just filled out to the full symmetric matrix instead of
 * only the diagonal). No new computation happens inside the filter; this
 * only reads state it already maintains every epoch. */
static int covariance_of(const ins_t* f, float P_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    memset(P_out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE);
    if (!f->is_initialized) return 0;
    const int n = f->n;
    int       i, j, k;
    for (i = 0; i < n; ++i)
    {
        for (j = i; j < n; ++j)
        {
            float s = 0.0f;
            for (k = j; k < n; ++k) { s += f->U[i + k * n] * f->d[k] * f->U[j + k * n]; }
            P_out[i + j * INS_CAPI_MAX_STATE] = s;
            P_out[j + i * INS_CAPI_MAX_STATE] = s;
        }
    }
    return n;
}

/* ============================= ins_core_* =================================== */

void* ins_core_create(void) { return calloc(1, sizeof(ins_core_ctx_t)); }
void  ins_core_destroy(void* h) { free(h); }

int ins_core_init(void* h, const ins_cfg_t* cfg)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c == (ins_core_ctx_t*)0 || cfg == (const ins_cfg_t*)0) return -1;
    ins_init_t    init;
    ins_options_t opt;
    cfg_to_init_opt(cfg, &init, &opt);
    memset(&c->meas, 0, sizeof(c->meas));
    return ins_init(&c->filter, &init, &opt);
}

void ins_core_set_imu(void* h, int64_t t_us, float dt_sec, const float acc[3], const float gyr[3],
                      const float acc_var[3], const float gyr_var[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_imu(&c->meas, t_us, dt_sec, acc, gyr, acc_var, gyr_var);
}
void ins_core_set_gnss_pos_ecef(void* h, const double ecef[3], const float var_ned[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_gnss_pos(&c->meas, ecef, var_ned);
}
void ins_core_set_gnss_vel_ned(void* h, const float vel_ned[3], const float var_ned[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_gnss_vel(&c->meas, vel_ned, var_ned);
}
void ins_core_set_gnss_pos_ecef_cov(void* h, const double ecef[3], const float Qll_ned[9])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_gnss_pos_cov(&c->meas, ecef, Qll_ned);
}
void ins_core_set_gnss_vel_ned_cov(void* h, const float vel_ned[3], const float Qll_ned[9])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_gnss_vel_cov(&c->meas, vel_ned, Qll_ned);
}
void ins_core_set_gnss_pos_vel_cov(void* h, const float Q_pos_vel_ned[9])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) memcpy(c->meas.gnss_Qll_pos_vel_ned, Q_pos_vel_ned, sizeof(float) * 9);
}
void ins_core_set_gnss_leverarm_b(void* h, const float lever_b[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c)
    {
        c->meas.gnss_leverarm_b[0] = lever_b[0];
        c->meas.gnss_leverarm_b[1] = lever_b[1];
        c->meas.gnss_leverarm_b[2] = lever_b[2];
    }
}
void ins_core_set_mag(void* h, const float mag[3], const float var[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_mag(&c->meas, mag, var);
}
void ins_core_set_yaw(void* h, float yaw_rad, float stddev_rad)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_yaw(&c->meas, yaw_rad, stddev_rad);
}
void ins_core_set_local_pos(void* h, const float pos_ned[3], const float var_ned[3],
                            const float lever_b[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_local_pos(&c->meas, pos_ned, var_ned, lever_b);
}
void ins_core_set_zupt(void* h, int on)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) c->meas.zero_velocity_update = (on != 0);
}
void ins_core_set_zaru(void* h, int on)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) c->meas.zero_rotation_update = (on != 0);
}
void ins_core_set_baro(void* h, float pressure_pa, float stddev_m)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_baro(&c->meas, pressure_pa, stddev_m);
}
void ins_core_set_speed(void* h, float speed_mps, float stddev_mps, int delay_ms)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) meas_speed(&c->meas, speed_mps, stddev_mps, delay_ms);
}
void ins_core_set_gnss_delay_ms(void* h, int ms)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) c->meas.gnss_delay_ms = ms;
}
void ins_core_set_yaw_delay_ms(void* h, int ms)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) c->meas.yaw_delay_ms = ms;
}
void ins_core_set_local_pos_delay_ms(void* h, int ms)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) c->meas.local_pos_delay_ms = ms;
}
void ins_core_update(void* h)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) ins_update(&c->filter, &c->meas);
}

/* ins_predict_step()'s phi_out is a tightly-packed n x n block (stride n);
 * re-stride it into the INS_CAPI_MAX_STATE-stride layout the rest of this
 * ABI uses for state matrices (see covariance_of), so callers index it the
 * same way as ins_core_get_covariance's output regardless of the runtime
 * state count. */
static void restride_phi(const float* tight, int n,
                         float out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    memset(out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE);
    int i, j;
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j < n; ++j) { out[i + j * INS_CAPI_MAX_STATE] = tight[i + j * n]; }
    }
}

int ins_core_predict(void* h, float phi_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (!c)
    {
        memset(phi_out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE);
        return 0;
    }
    float     phi_tight[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE];
    const int status = ins_predict_step(&c->filter, &c->meas, phi_tight);
    if (status & INS_EPOCH_COV_PROPAGATED) { restride_phi(phi_tight, c->filter.n, phi_out); }
    else { memset(phi_out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE); }
    return status;
}

void ins_core_correct(void* h)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) ins_correct_step(&c->filter);
}

void ins_core_set_magnetic_model_position(void* h, double lat_rad, double lon_rad, float year)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) ins_set_magnetic_model_from_position(&c->filter, lat_rad, lon_rad, year);
}

int ins_core_is_ready(void* h)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_is_ready(&c->filter);
}
int ins_core_deadreckoning_ms(void* h)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c ? ins_deadreckoning_ms(&c->filter) : -1;
}
int ins_core_get_position_ecef(void* h, double o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_position_ecef(&c->filter, o);
}
int ins_core_get_position_local(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_position_local(&c->filter, o);
}
int ins_core_get_velocity_ned(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_velocity_ned(&c->filter, o);
}
int ins_core_get_quaternion(void* h, float o[4])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_quaternion(&c->filter, o);
}
int ins_core_get_rotmat_b_to_n(void* h, float o[9])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_rotmat_b_to_n(&c->filter, o);
}
int ins_core_get_rpy(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_rpy(&c->filter, &o[0], &o[1], &o[2]);
}
int ins_core_get_omega_b_nb(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_omega_b_nb(&c->filter, o);
}
int ins_core_get_acc_n(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_acc_n(&c->filter, o);
}
int ins_core_get_origin_ecef(void* h, double o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && origin_of(&c->filter, o);
}
int ins_core_get_bias_acc(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_bias_acc(&c->filter, o);
}
int ins_core_get_bias_gyr(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_bias_gyr(&c->filter, o);
}
int ins_core_get_bias_mag(void* h, float o[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c && ins_get_bias_mag(&c->filter, o);
}
int ins_core_get_covariance(void* h, float P_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c ? covariance_of(&c->filter, P_out) : 0;
}

static void copy_diag(const ins_t* f, uint32_t out[10])
{
    memset(out, 0, sizeof(uint32_t) * 10);
    const ins_diag_t* d = ins_get_diag(f);
    if (!d) return;
    out[0] = d->n_predict;
    out[1] = d->n_gnss_seen;
    out[2] = d->n_gnss_used;
    out[3] = d->n_gnss_rejected_noise;
    out[4] = d->n_gnss_no_anchor;
    out[5] = d->n_fuse_fail;
    out[6] = d->n_auto_zupt;
    out[7] = d->n_invalid_input;
    out[8] = d->n_downweighted;     /* REQ-NAV-036 */
    out[9] = d->n_baro_height_used; /* REQ-NAV-054, 0 = GNSS height source */
}
void ins_core_get_diag(void* h, uint32_t out[10])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c)
        copy_diag(&c->filter, out);
    else
        memset(out, 0, sizeof(uint32_t) * 10);
}

/* Overconfidence watchdog snapshot (REQ-NAV-040): returns n_overconfident
 * (epochs that tripped a floor; 0 = never), and fills out_min_stddev with the
 * smallest position [m], velocity [m/s] and attitude [deg] per-axis 1-sigma
 * the filter has reported (INFINITY until the first post-init epoch). */
static uint32_t overconfidence_of(const ins_t* f, float out_min_stddev[3])
{
    const ins_diag_t* d = ins_get_diag(f);
    if (!d)
    {
        out_min_stddev[0] = out_min_stddev[1] = out_min_stddev[2] = 0.0f;
        return 0;
    }
    out_min_stddev[0] = d->min_pos_stddev_m;
    out_min_stddev[1] = d->min_vel_stddev_mps;
    out_min_stddev[2] = d->min_att_stddev_deg;
    return d->n_overconfident;
}
uint32_t ins_core_get_overconfidence(void* h, float out_min_stddev[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c) return overconfidence_of(&c->filter, out_min_stddev);
    out_min_stddev[0] = out_min_stddev[1] = out_min_stddev[2] = 0.0f;
    return 0;
}
int ins_core_auto_zupt_active(void* h)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    return c ? (ins_auto_zupt_active(&c->filter) ? 1 : 0) : 0;
}

/* ============================= ins_suite_* ==================================== */

void* ins_suite_create(void) { return calloc(1, sizeof(ins_suite_ctx_t)); }
void  ins_suite_destroy(void* h) { free(h); }

int ins_suite_init(void* h, const ins_cfg_t* cfg)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c == (ins_suite_ctx_t*)0 || cfg == (const ins_cfg_t*)0) return -1;
    ins_init_t    init;
    ins_options_t opt;
    cfg_to_init_opt(cfg, &init, &opt);
    memset(&c->meas, 0, sizeof(c->meas));
    return nav_suite_init(&c->suite, &init, &opt);
}

void ins_suite_set_imu(void* h, int64_t t_us, float dt_sec, const float acc[3], const float gyr[3],
                       const float acc_var[3], const float gyr_var[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_imu(&c->meas, t_us, dt_sec, acc, gyr, acc_var, gyr_var);
}
void ins_suite_set_gnss_pos_ecef(void* h, const double ecef[3], const float var_ned[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_gnss_pos(&c->meas, ecef, var_ned);
}
void ins_suite_set_gnss_vel_ned(void* h, const float vel_ned[3], const float var_ned[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_gnss_vel(&c->meas, vel_ned, var_ned);
}
void ins_suite_set_gnss_pos_ecef_cov(void* h, const double ecef[3], const float Qll_ned[9])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_gnss_pos_cov(&c->meas, ecef, Qll_ned);
}
void ins_suite_set_gnss_vel_ned_cov(void* h, const float vel_ned[3], const float Qll_ned[9])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_gnss_vel_cov(&c->meas, vel_ned, Qll_ned);
}
void ins_suite_set_gnss_pos_vel_cov(void* h, const float Q_pos_vel_ned[9])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) memcpy(c->meas.gnss_Qll_pos_vel_ned, Q_pos_vel_ned, sizeof(float) * 9);
}
void ins_suite_set_gnss_leverarm_b(void* h, const float lever_b[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c)
    {
        c->meas.gnss_leverarm_b[0] = lever_b[0];
        c->meas.gnss_leverarm_b[1] = lever_b[1];
        c->meas.gnss_leverarm_b[2] = lever_b[2];
    }
}
void ins_suite_set_mag(void* h, const float mag[3], const float var[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_mag(&c->meas, mag, var);
}
void ins_suite_set_yaw(void* h, float yaw_rad, float stddev_rad)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_yaw(&c->meas, yaw_rad, stddev_rad);
}
void ins_suite_set_local_pos(void* h, const float pos_ned[3], const float var_ned[3],
                             const float lever_b[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_local_pos(&c->meas, pos_ned, var_ned, lever_b);
}
void ins_suite_set_zupt(void* h, int on)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) c->meas.zero_velocity_update = (on != 0);
}
void ins_suite_set_zaru(void* h, int on)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) c->meas.zero_rotation_update = (on != 0);
}
void ins_suite_set_auto_zaru(void* h, int ars_on, int ahrs_on)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c)
    {
        c->suite.ars_cfg.auto_zaru_disable  = (ars_on == 0);
        c->suite.ahrs_cfg.auto_zaru_disable = (ahrs_on == 0);
    }
}
void ins_suite_set_baro(void* h, float pressure_pa, float stddev_m)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_baro(&c->meas, pressure_pa, stddev_m);
}
void ins_suite_set_speed(void* h, float speed_mps, float stddev_mps, int delay_ms)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) meas_speed(&c->meas, speed_mps, stddev_mps, delay_ms);
}
/* baro_alt acc-bias drift density [m/s^2/sqrt(Hz)] (<=0 -> baro_alt default).
   Must be called before the first baro sample latches baro_cfg into the
   filter (nav_suite.h contract); a no-op afterwards. */
void ins_suite_set_baro_acc_bias_drift(void* h, float density_mps2_sqrthz)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && density_mps2_sqrthz > 0.0f)
        c->suite.baro_cfg.acc_bias_drift_mps2_sqrthz = density_mps2_sqrthz;
}
/* baro_alt's own direct accel-noise density [m/s^2/sqrt(Hz)] (<=0 ->
   baro_alt default). Same latch-on-first-baro-sample timing as
   ins_suite_set_baro_acc_bias_drift(). */
void ins_suite_set_baro_acc_noise(void* h, float density_mps2_sqrthz)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && density_mps2_sqrthz > 0.0f)
        c->suite.baro_cfg.acc_noise_mps2_sqrthz = density_mps2_sqrthz;
}
/* baro_alt's own INITIAL accel-bias uncertainty [m/s^2] (<=0 -> baro_alt
   default, BARO_ALT_DEFAULT_AB_STDDEV_MPS2). An INITIAL CONDITION, not a
   process-noise rate -- but it propagates into height/velocity through
   the bias/height/velocity Phi coupling (baro_alt_predict's dt2h/dt
   terms), growing roughly with T^4/T^2 respectively through that double
   integration, and at the default size can dominate baro_alt's apparent
   growth for tens of seconds. Same latch-on-first-baro-sample timing as
   ins_suite_set_baro_acc_bias_drift(). */
void ins_suite_set_baro_acc_bias_init_stddev(void* h, float stddev_mps2)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && stddev_mps2 > 0.0f) c->suite.baro_cfg.acc_bias_init_stddev_mps2 = stddev_mps2;
}
/* baro_alt's own INITIAL height/velocity uncertainty ([m]/[m/s], <=0 ->
   baro_alt defaults 1.0/0.5) -- same INITIAL-CONDITION caveat as
   ins_suite_set_baro_acc_bias_init_stddev; these are what dominated this
   test suite's free-coasting checks before being pinned down (see
   tests/test_growth_rate.py). */
void ins_suite_set_baro_h_init_stddev(void* h, float stddev_m)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && stddev_m > 0.0f) c->suite.baro_cfg.h_init_stddev_m = stddev_m;
}
void ins_suite_set_baro_v_init_stddev(void* h, float stddev_mps)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && stddev_mps > 0.0f) c->suite.baro_cfg.v_init_stddev_mps = stddev_mps;
}
/* baro_alt's own small direct height process-noise density [m/sqrt(Hz)]
   (<=0 -> baro_alt default) -- a safety margin against discretization/model
   mismatch, not a primary noise source. Same latch-on-first-baro-sample
   timing as ins_suite_set_baro_acc_bias_drift(). */
void ins_suite_set_baro_h_process_noise(void* h, float density_m_sqrthz)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && density_m_sqrthz > 0.0f) c->suite.baro_cfg.h_process_noise_m_sqrthz = density_m_sqrthz;
}
/* Local-height/GNSS-ellipsoid offset filter's random walk [m/sqrt(s)]
   (<=0 -> local_gnss_alt default 0.03). Raise this for missions with large
   altitude excursions (e.g. a soaring glider gaining kilometres of
   altitude): the offset absorbs the ISA-model error, which grows with the
   excursion (REQ-BARO-011) and can outrun the default random walk, tuned
   for a multi-metre swing. Must be called before the offset filter's
   first local-height/GNSS pair (nav_suite_update_local_gnss's first
   call with both a fix and a usable local height) -- in practice, call
   right after construction, before the first ins_suite_update(). */
void ins_suite_set_local_gnss_rw_stddev(void* h, float rw_stddev_mps)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && rw_stddev_mps > 0.0f) c->suite.local_gnss_cfg.rw_stddev_mps = rw_stddev_mps;
}
/* Offset filter's chi2 outlier gate on the innovation (<=0 ->
   local_gnss_alt default, chi2inv(0.95,1)). Same latch timing as
   ins_suite_set_local_gnss_rw_stddev(). */
void ins_suite_set_local_gnss_chi2_threshold(void* h, float threshold)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && threshold > 0.0f) c->suite.local_gnss_cfg.chi2_threshold = threshold;
}
/* Offset filter's minimum time between two fusions [s] (<=0 ->
   local_gnss_alt default 10). Same latch timing as
   ins_suite_set_local_gnss_rw_stddev(). */
void ins_suite_set_local_gnss_min_update_interval(void* h, float interval_sec)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && interval_sec > 0.0f) c->suite.local_gnss_cfg.min_update_interval_sec = interval_sec;
}
/* Offset filter's stddev inflation factor applied to both sides of the
   pair before combining into the measurement variance (<=0 ->
   local_gnss_alt default 3). Same latch timing as
   ins_suite_set_local_gnss_rw_stddev(). */
void ins_suite_set_local_gnss_stddev_inflation(void* h, float factor)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && factor > 0.0f) c->suite.local_gnss_cfg.stddev_inflation_factor = factor;
}
/* ARS/AHRS SHARED gyro noise density [rad/s/sqrt(Hz)] (<=0 -> ahrs
   default): both sub-filters consume the same physical gyro, so one call
   sets both templates. Must be called before the first ins_suite_update()
   (nav_suite.h contract: ars_cfg/ahrs_cfg latch at auto-initialization). */
void ins_suite_set_ahrs_gyr_noise(void* h, float density_rps_sqrthz)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && density_rps_sqrthz > 0.0f)
    {
        c->suite.ars_cfg.gyr_noise_psd  = density_rps_sqrthz;
        c->suite.ahrs_cfg.gyr_noise_psd = density_rps_sqrthz;
    }
}
/* ARS/AHRS SHARED accelerometer noise stddev [m/s^2] (<=0 -> ahrs default,
   AHRS_DEFAULT_ACC_NOISE_MPS2): both sub-filters consume the same physical
   accelerometer for leveling, so one call sets both templates. Same timing
   contract as ins_suite_set_ahrs_gyr_noise(). */
void ins_suite_set_ahrs_acc_noise(void* h, float stddev_mps2)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && stddev_mps2 > 0.0f)
    {
        c->suite.ars_cfg.acc_noise_mps2  = stddev_mps2;
        c->suite.ahrs_cfg.acc_noise_mps2 = stddev_mps2;
    }
}
/* ARS/AHRS SHARED gyro bias random walk density [rad/s^2/sqrt(Hz)] (<=0 ->
   ahrs default); same timing contract as ins_suite_set_ahrs_gyr_noise(). */
void ins_suite_set_ahrs_gyr_bias_rw(void* h, float density_rps2_sqrthz)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && density_rps2_sqrthz > 0.0f)
    {
        c->suite.ars_cfg.gyr_bias_rw  = density_rps2_sqrthz;
        c->suite.ahrs_cfg.gyr_bias_rw = density_rps2_sqrthz;
    }
}
/* ARS/AHRS SHARED initial gyro-bias uncertainty [rad/s], all 3 axes (<=0 ->
   ahrs default, 1/1/5 deg/s xy/z) -- this is an INITIAL CONDITION, not a
   process-noise rate, but it matters a great deal for how fast yaw
   uncertainty appears to grow: yaw inherits it through the attitude/bias
   Phi coupling (ahrs_predict_covariance), and with the default 5 deg/s z
   bias uncertainty that inherited term dominates any gyr_noise_psd/
   gyr_bias_rw-driven growth for tens of seconds. Same timing contract as
   ins_suite_set_ahrs_gyr_noise(). */
void ins_suite_set_ahrs_gyr_bias_init_stddev(void* h, float stddev_rps)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c && stddev_rps > 0.0f)
    {
        int i;
        for (i = 0; i < 3; ++i)
        {
            c->suite.ars_cfg.gyr_bias_init_stddev_rps[i]  = stddev_rps;
            c->suite.ahrs_cfg.gyr_bias_init_stddev_rps[i] = stddev_rps;
        }
    }
}
void ins_suite_set_gnss_delay_ms(void* h, int ms)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) c->meas.gnss_delay_ms = ms;
}
void ins_suite_set_yaw_delay_ms(void* h, int ms)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) c->meas.yaw_delay_ms = ms;
}
void ins_suite_set_local_pos_delay_ms(void* h, int ms)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) c->meas.local_pos_delay_ms = ms;
}
/* A known initial roll/pitch and/or yaw (e.g. the vehicle's heading at
   the start of the log, with no magnetometer/GNSS-course yaw aiding to
   derive it from) for ins's auto-init bootstrap -- the static, "I just
   know it" case, as opposed to nav_suite's own per-epoch ARS/AHRS hint
   (REQ-SUITE-016), which still takes priority once available. Set once,
   right after construction, before the first update(); thin forward to
   nav_suite_set_init_att_hint (REQ-SUITE-017). */
void ins_suite_set_init_att_hint(void* h, float roll_rad, float pitch_rad,
                                 float stddev_roll_pitch_rad, float yaw_rad, float stddev_yaw_rad)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (!c) return;
    nav_suite_set_init_att_hint(&c->suite, roll_rad, pitch_rad, stddev_roll_pitch_rad, yaw_rad,
                                stddev_yaw_rad);
}
void ins_suite_update(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) nav_suite_update(&c->suite, &c->meas);
}

int ins_suite_predict(void* h, float phi_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (!c)
    {
        memset(phi_out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE);
        return 0;
    }
    float     phi_tight[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE];
    const int status = nav_suite_predict_step(&c->suite, &c->meas, phi_tight);
    if (status & INS_EPOCH_COV_PROPAGATED) { restride_phi(phi_tight, c->suite.ins.n, phi_out); }
    else { memset(phi_out, 0, sizeof(float) * INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE); }
    return status;
}

void ins_suite_correct(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) nav_suite_correct_step(&c->suite);
}

void ins_suite_set_magnetic_model_position(void* h, double lat_rad, double lon_rad, float year)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (!c) return;
    /* Both attitude sources must agree on the yaw datum (true north):
       ins gets the full NED reference vector + field gate, the
       magnetometer AHRS gets its declination re-frame. The ARS has no
       magnetometer, so its (free) yaw needs no declination. */
    ins_set_magnetic_model_from_position(&c->suite.ins, lat_rad, lon_rad, year);
    ahrs_set_position(&c->suite.ahrs, (float)lat_rad, (float)lon_rad, year);
}

int ins_suite_get_mode(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (!c) return 0;
    const nav_suite_mode_t mode = nav_suite_get_mode(&c->suite);
    return (int)mode;
}
int ins_suite_get_rpy(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_rpy(&c->suite, &o[0], &o[1], &o[2]);
}
int ins_suite_get_rpy_ins(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_rpy_ins(&c->suite, &o[0], &o[1], &o[2]);
}
int ins_suite_get_rpy_ars(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_rpy_ars(&c->suite, &o[0], &o[1], &o[2]);
}
int ins_suite_get_rpy_ahrs(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_rpy_ahrs(&c->suite, &o[0], &o[1], &o[2]);
}

int ins_suite_get_bias_gyr_ars(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_get_bias_gyr(&c->suite.ars, o);
}
int ins_suite_get_bias_gyr_ahrs(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_get_bias_gyr(&c->suite.ahrs, o);
}
/* 1-sigma gyro bias uncertainty (last 3 states of the 5/6-state error
 * vector, see ahrs.h) -- diagonal only, see diag_covariance_of(). */
static int ahrs_gyr_bias_stddev_of(const ahrs_t* a, float o[3])
{
    if (!a->is_initialized) return 0;
    float diag[AHRS_UNKNOWNS_MAX];
    diag_covariance_of(a->U, a->d, a->n, diag);
    const int off = a->n - 3;
    int       i;
    for (i = 0; i < 3; ++i) { o[i] = sqrtf(diag[off + i]); }
    return 1;
}
int ins_suite_get_gyr_bias_stddev_ars(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_gyr_bias_stddev_of(&c->suite.ars, o);
}
int ins_suite_get_gyr_bias_stddev_ahrs(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_gyr_bias_stddev_of(&c->suite.ahrs, o);
}
/* 1-sigma roll/pitch/yaw uncertainty (ahrs_get_rpy_stddev, ahrs.h) --
 * lets the growth-rate diagnostic (replay.ahrs_growth_rate()) be verified
 * against ARS's/AHRS's own reported attitude covariance growth, the same
 * way process_noise_growth_rate() is checked against ins (see
 * tests/test_growth_rate.py). */
int ins_suite_get_rpy_stddev_ars(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_get_rpy_stddev(&c->suite.ars, &o[0], &o[1], &o[2]);
}
int ins_suite_get_rpy_stddev_ahrs(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ahrs_get_rpy_stddev(&c->suite.ahrs, &o[0], &o[1], &o[2]);
}

int ins_suite_is_ready(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_is_ready(&c->suite.ins);
}
int ins_suite_deadreckoning_ms(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? ins_deadreckoning_ms(&c->suite.ins) : -1;
}
int ins_suite_get_position_ecef(void* h, double o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_position_ecef(&c->suite.ins, o);
}
int ins_suite_get_position_local(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_position_local(&c->suite.ins, o);
}
int ins_suite_get_velocity_ned(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_velocity_ned(&c->suite.ins, o);
}
int ins_suite_get_quaternion(void* h, float o[4])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_quaternion(&c->suite.ins, o);
}
int ins_suite_get_rotmat_b_to_n(void* h, float o[9])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_rotmat_b_to_n(&c->suite.ins, o);
}
int ins_suite_get_omega_b_nb(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_omega_b_nb(&c->suite.ins, o);
}
int ins_suite_get_acc_n(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_acc_n(&c->suite.ins, o);
}
int ins_suite_get_origin_ecef(void* h, double o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && origin_of(&c->suite.ins, o);
}
int ins_suite_get_bias_acc(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_bias_acc(&c->suite.ins, o);
}
int ins_suite_get_bias_gyr(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_bias_gyr(&c->suite.ins, o);
}
int ins_suite_get_bias_mag(void* h, float o[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && ins_get_bias_mag(&c->suite.ins, o);
}
int ins_suite_get_covariance(void* h, float P_out[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? covariance_of(&c->suite.ins, P_out) : 0;
}
static void copy_speed_diag(const ins_t* f, uint32_t out[3], float* resid)
{
    const ins_diag_t* d = ins_get_diag(f);
    out[0]              = d->n_speed_seen;
    out[1]              = d->n_speed_used;
    out[2]              = d->n_speed_skipped;
    if (resid) *resid = d->last_speed_residual_mps;
}
void ins_core_get_speed_diag(void* h, uint32_t out_counts[3], float* out_resid)
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c)
    {
        copy_speed_diag(&c->filter, out_counts, out_resid);
        return;
    }
    memset(out_counts, 0, sizeof(uint32_t) * 3);
    if (out_resid) *out_resid = 0.0f;
}
static void copy_time_diag(const ins_t* f, uint32_t out[3])
{
    const ins_diag_t* d = ins_get_diag(f);
    out[0]              = d->n_time_backward;
    out[1]              = d->n_time_dropped;
    out[2]              = d->n_time_restart_reset;
}
void ins_core_get_time_diag(void* h, uint32_t out_counts[3])
{
    ins_core_ctx_t* c = (ins_core_ctx_t*)h;
    if (c)
    {
        copy_time_diag(&c->filter, out_counts);
        return;
    }
    memset(out_counts, 0, sizeof(uint32_t) * 3);
}
void ins_suite_get_time_diag(void* h, uint32_t out_counts[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c)
    {
        copy_time_diag(&c->suite.ins, out_counts);
        return;
    }
    memset(out_counts, 0, sizeof(uint32_t) * 3);
}
void ins_suite_get_speed_diag(void* h, uint32_t out_counts[3], float* out_resid)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c)
    {
        copy_speed_diag(&c->suite.ins, out_counts, out_resid);
        return;
    }
    memset(out_counts, 0, sizeof(uint32_t) * 3);
    if (out_resid) *out_resid = 0.0f;
}
void ins_suite_get_diag(void* h, uint32_t out[10])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c)
        copy_diag(&c->suite.ins, out);
    else
        memset(out, 0, sizeof(uint32_t) * 10);
}
uint32_t ins_suite_get_overconfidence(void* h, float out_min_stddev[3])
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    if (c) return overconfidence_of(&c->suite.ins, out_min_stddev);
    out_min_stddev[0] = out_min_stddev[1] = out_min_stddev[2] = 0.0f;
    return 0;
}
/* Attitude overconfidence watchdog for the suite's ARS/AHRS instances
 * (REQ-AHRS-020): returns n_overconfident, fills the smallest attitude 1-sigma
 * [deg] seen. Attitude-only (these filters have no pos/vel). */
uint32_t ins_suite_get_ars_overconfidence(void* h, float* out_min_att_deg)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    *out_min_att_deg   = c ? c->suite.ars.min_att_stddev_deg : 0.0f;
    return c ? c->suite.ars.n_overconfident : 0;
}
uint32_t ins_suite_get_ahrs_overconfidence(void* h, float* out_min_att_deg)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    *out_min_att_deg   = c ? c->suite.ahrs.min_att_stddev_deg : 0.0f;
    return c ? c->suite.ahrs.n_overconfident : 0;
}

/* Downweight counters (REQ-NAV-036, REQ-AHRS-019, REQ-BARO-017) for the
 * suite's other sub-filters -- ins's own is already in ins_suite_get_diag()
 * above; these have no bare-core equivalent (ars/ahrs/baro only exist in
 * the suite). */
uint32_t ins_suite_get_ars_downweight_count(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? c->suite.ars.n_downweighted : 0;
}
uint32_t ins_suite_get_ahrs_downweight_count(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? c->suite.ahrs.n_downweighted : 0;
}
uint32_t ins_suite_get_baro_downweight_count(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? c->suite.baro_alt.n_downweighted : 0;
}
uint32_t ins_suite_get_local_gnss_downweight_count(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? c->suite.local_gnss.n_downweighted : 0;
}
int ins_suite_auto_zupt_active(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (ins_auto_zupt_active(&c->suite.ins) ? 1 : 0) : 0;
}
int ins_suite_zaru_active(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (nav_suite_get_zaru_active(&c->suite) ? 1 : 0) : 0;
}
int ins_suite_ars_auto_zaru_active(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (ahrs_auto_zaru_active(&c->suite.ars) ? 1 : 0) : 0;
}
int ins_suite_ahrs_auto_zaru_active(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (ahrs_auto_zaru_active(&c->suite.ahrs) ? 1 : 0) : 0;
}
int ins_suite_ars_zaru_applied(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (ahrs_zaru_applied(&c->suite.ars) ? 1 : 0) : 0;
}
int ins_suite_ahrs_zaru_applied(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (ahrs_zaru_applied(&c->suite.ahrs) ? 1 : 0) : 0;
}
int ins_suite_vertical_zupt_active(void* h)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c ? (nav_suite_get_vertical_zupt_active(&c->suite) ? 1 : 0) : 0;
}

int ins_suite_get_baro_alt(void* h, float* h_m, float* v_mps)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_baro_alt(&c->suite, h_m, v_mps);
}
int ins_suite_get_baro_acc_bias(void* h, float* acc_bias_mps2)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && baro_alt_get_acc_bias(&c->suite.baro_alt, acc_bias_mps2);
}
/* 1-sigma [h, v, acc_bias] uncertainty, see diag_covariance_of(). */
// cppcheck-suppress constParameterPointer
// (h stays void* to match every other handle getter in this API)
int ins_suite_get_baro_stddev(void* h, float o[3])
{
    const ins_suite_ctx_t* c = (const ins_suite_ctx_t*)h;
    if (!c || !c->suite.baro_alt.is_initialized) return 0;
    float diag[BARO_ALT_STATES];
    diag_covariance_of(c->suite.baro_alt.U, c->suite.baro_alt.d, BARO_ALT_STATES, diag);
    int i;
    for (i = 0; i < BARO_ALT_STATES; ++i) { o[i] = sqrtf(diag[i]); }
    return 1;
}
int ins_suite_get_local_gnss_offset(void* h, float* offset_m, float* stddev_m)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && local_gnss_alt_get(&c->suite.local_gnss, offset_m, stddev_m);
}
int ins_suite_get_height(void* h, float* h_m)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_height(&c->suite, h_m);
}
int ins_suite_get_height_ellipsoid(void* h, float* h_ell_m)
{
    ins_suite_ctx_t* c = (ins_suite_ctx_t*)h;
    return c && nav_suite_get_height_ellipsoid(&c->suite, h_ell_m);
}
