/** @file nav_suite.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief Wrapper running ins, two AHRS filters and a baro/accel
 *        vertical filter in parallel.
 *
 * See nav_suite.h for the overall design.
 */

#include <math.h>
#include <string.h>

#include "nav_suite.h"
#include "geodetic_toolbox.h"
#include "linalg.h" /* MAT_ELEM for the n-frame <-> ECEF rotation */
#include "log.h"

/* Diagnostics only (see log.h): how far the ARS's independently
   estimated gyro bias may drift from the 3D filter's own before it is
   flagged as a cross-check disagreement (both observe the same physical
   gyro, so a large gap usually means one of the two is not converged /
   has a problem), and how often that warning repeats while it persists. */
#define NAV_SUITE_LOG_GYR_BIAS_DIFF_WARN_DPS   (1.0f)
#define NAV_SUITE_LOG_GYR_BIAS_DIFF_REPEAT_SEC (240.0f)

/* Diagnostics only (see log.h): ins and baro_alt independently estimate
   vertical velocity from the same accelerometer, ins additionally
   corrected by its height aiding. A persistent gap flags a problem with
   one of the two vertical references rather than routine sensor noise. */
#define NAV_SUITE_LOG_VVEL_DIFF_WARN_MPS   (2.0f)
#define NAV_SUITE_LOG_VVEL_DIFF_REPEAT_SEC (240.0f)

/* Diagnostics only (REQ-SUITE-019, see log.h): the same cross-check on the
   HEIGHT rather than its derivative. Needed alongside the velocity check
   above because the two are sensitive to different failures: a slow
   divergence stays far below the velocity threshold while accumulating
   without bound in the height (a 0.2 m/s disagreement is invisible to a
   2 m/s gate and still 60 m after five minutes).
   Sized generously on purpose. The two filters are only expected to track
   the same physical quantity, not to agree closely: separate accelerometer
   bias states, separate process noise, and different aiding (ins is
   additionally corrected by GNSS while it has it). Below the barometric
   height source (REQ-NAV-053) they at least share the raw pressure stream
   and one datum, so a persistent gap of this size means one of them has a
   real problem, not that they are two estimators. */
#define NAV_SUITE_LOG_HEIGHT_DIFF_WARN_M     (10.0f)
#define NAV_SUITE_LOG_HEIGHT_DIFF_REPEAT_SEC (240.0f)

/* Diagnostics only (REQ-SUITE-023, see log.h): ins and the magnetometer AHRS
   independently estimate heading -- ins additionally corrected by position
   aiding, the AHRS referenced to the raw magnetometer. nav_suite_get_rpy()
   falls back from ins to the AHRS the moment ins leaves FULL/COASTING
   (REQ-SUITE-005), so a persistent gap between the two means the reported
   heading will STEP by exactly that amount at the next mode switch, the same
   situation REQ-SUITE-019 flags for height. Sized generously: a magnetic
   heading reference several degrees off an ins solution (residual hard/soft-
   iron error, declination) is routine, not itself a defect. */
#define NAV_SUITE_LOG_YAW_DIFF_WARN_DEG   (15.0f)
#define NAV_SUITE_LOG_YAW_DIFF_REPEAT_SEC (240.0f)

/* Initial attitude uncertainty for the AHRS auto-initialization: the
 * leveling/heading bootstrap uses a single (possibly moving) sample, so
 * start pessimistic and let the filters converge. */
#define NAV_SUITE_AHRS_INIT_RP_STDDEV  DEG2RAD(10.0f)
#define NAV_SUITE_AHRS_INIT_YAW_STDDEV DEG2RAD(20.0f)

/* How much less to trust the ARS/AHRS's own reported attitude/gyro-bias
 * uncertainty when handing it to ins as an auto-init/re-acquisition seed
 * (REQ-SUITE-016): the two filters process the same IMU stream but are
 * otherwise independent estimators, so the source's stddev at face value would
 * understate ins's actual uncertainty about a value it did not derive. */
#define NAV_SUITE_ATT_HINT_STDDEV_INFLATION (10.0f)

/* Heading carry-over across an ins re-initialization (REQ-SUITE-022).
 *
 * Latch gate: only a 3D heading this confident is worth carrying. Above it ins
 * never really resolved its own heading, and re-seeding a later bootstrap from
 * it would launder a guess into a prior.
 *
 * Use gate: the carried heading plus the ARS drift priced onto it stops being
 * worth having somewhere below the "heading unknown" prior ins would otherwise
 * start from. Past this the honest answer is that prior, which lets the first
 * aiding snap the heading in one step instead of unwinding a stale one. */
#define NAV_SUITE_YAW_CARRY_LATCH_MAX_STDDEV DEG2RAD(15.0f)
#define NAV_SUITE_YAW_CARRY_MAX_STDDEV       DEG2RAD(30.0f)

/* Averaged baro bootstrap window (REQ-SUITE-006): the datum anchor is the mean
 * of the plausible pressure samples over this window, so one glitched startup
 * reading cannot skew the vertical datum. Short enough that the vertical
 * channel is available quickly and the platform has barely moved. */
#define NAV_SUITE_BARO_BOOT_SEC (0.3f)

/* Position aiding older than this counts as "coasting" (mode arbitration only:
 * ins itself stays ready until its max_deadreckoning_sec window expires). Two
 * seconds covers a couple of missed epochs of a typical aiding source. */
#define NAV_SUITE_FRESH_AIDING_MS (2000)

/* Does ins still stand on the n-frame origin the vertical datum was established
 * for? A carried origin (REQ-NAV-062) is copied verbatim, so the difference is
 * exactly zero there; the tolerance only keeps this off exact float equality.
 * Only meaningful once an alignment has happened (vertical_datum_aligned),
 * which is what fills datum_origin_ecef; every caller checks that first. */
#define NAV_SUITE_DATUM_ORIGIN_EPS_M (1.0e-3)

static bool nav_suite_ins_origin_is_datum(const nav_suite_t* s)
{
    int i;
    for (i = 0; i < 3; ++i)
    {
        if (fabs(s->ins.origin_ecef[i] - s->datum_origin_ecef[i]) > NAV_SUITE_DATUM_ORIGIN_EPS_M)
        {
            return false;
        }
    }
    return true;
}

/* Relative variance improvement a pair must still deliver for the datum-offset
 * anchor to keep tracking the offset filter (REQ-SUITE-008). Just under 1: the
 * filter's variance falls steeply while it converges and then flattens as the
 * random-walk propagation balances the update, so anything short of a clear
 * improvement means it has settled and the anchor freezes. */
#define NAV_SUITE_DATUM_OFFSET_SETTLED (0.98f)

/* How many sigma of the drift estimate must be exceeded before any of it is
 * believed (REQ-SUITE-008). The correction is soft-thresholded by this much:
 * a drift inside the estimate's own noise is reported as no drift at all, and
 * a larger one is corrected less this margin. ins's absolute height is the
 * better estimate whenever the datum has NOT moved - which is most missions,
 * most of the time - so the burden of proof sits on the correction, and a
 * margin of one sigma keeps the offset filter's noise out of a height that
 * was already right. */
#define NAV_SUITE_DATUM_DRIFT_SIGMA (1.0f)

/* Record the origin the datum now belongs to. Called AFTER any shift, so
 * the latched value is the origin ins actually ends up on. */
static void nav_suite_latch_datum_origin(nav_suite_t* s)
{
    int i;
    for (i = 0; i < 3; ++i) { s->datum_origin_ecef[i] = s->ins.origin_ecef[i]; }
    s->vertical_datum_aligned = true;
    /* A datum just fixed has not drifted yet, whatever the previous one had
       done: the anchor is re-established against this one (REQ-SUITE-008). */
    s->datum_offset_valid   = false;
    s->datum_offset_var_min = 0.0f;
    s->datum_offset_t_last  = 0;
}

/* @satisfies REQ-SUITE-004 REQ-SUITE-011 REQ-SUITE-020 */
int nav_suite_init(nav_suite_t* s, const ins_init_t* init, const ins_options_t* opt)
{
    if (s == NULL || init == NULL || opt == NULL) { return -1; }

    memset(&s->ars, 0, sizeof(s->ars));
    memset(&s->ahrs, 0, sizeof(s->ahrs));
    memset(&s->baro_alt, 0, sizeof(s->baro_alt));
    memset(&s->local_gnss, 0, sizeof(s->local_gnss));

    /* Config templates: all noise/tuning fields 0 -> module defaults.
       May be tuned by the caller before the first update. */
    memset(&s->ars_cfg, 0, sizeof(s->ars_cfg));
    memset(&s->ahrs_cfg, 0, sizeof(s->ahrs_cfg));
    memset(&s->baro_cfg, 0, sizeof(s->baro_cfg));
    memset(&s->local_gnss_cfg, 0, sizeof(s->local_gnss_cfg));
    s->ars_cfg.mode  = AHRS_MODE_ARS;
    s->ahrs_cfg.mode = AHRS_MODE_AHRS;

    /* Global outlier-rejection override (REQ-SUITE-011): a single flag
       on ins's own options reaches every sub-filter's config, so the
       caller only has to set it once instead of on all four templates. */
    s->ars_cfg.chi2_disable        = opt->chi2_disable;
    s->ahrs_cfg.chi2_disable       = opt->chi2_disable;
    s->baro_cfg.chi2_disable       = opt->chi2_disable;
    s->local_gnss_cfg.chi2_disable = opt->chi2_disable;

    /* Stillness definition (REQ-SUITE-020): same reasoning as the chi2 flag
       above, for the question "is the platform standing still". ins, the two
       AHRS instances and baro_alt each answer it with their own code, and
       before this propagation each also had to be TUNED separately - a caller
       who tightened ins's gates got silently untouched AHRS gates. The
       auto_zupt_* set on ins_options_t is now the only place it is stated. The
       0 sentinels are forwarded as-is: both sides fall back to the same
       sensor_defaults.h constants.

       Not forwarded: auto_zupt_max_vel_mps / auto_zupt_max_vel_stddev_mps (the
       ARS/AHRS have no velocity state to gate on, REQ-AHRS-017) and
       auto_zupt_min_interval_sec (ahrs_fuse_zaru throttles its own fusion). */
    s->ars_cfg.auto_zaru_disable = opt->auto_zupt_disable || opt->auto_zupt_velocity_blind_disable;
    s->ars_cfg.auto_zaru_static_gyr_rps         = opt->auto_zupt_static_gyr_rps;
    s->ars_cfg.auto_zaru_static_acc_mps2        = opt->auto_zupt_static_acc_mps2;
    s->ars_cfg.auto_zaru_static_gyr_stddev_rps  = opt->auto_zupt_static_gyr_stddev_rps;
    s->ars_cfg.auto_zaru_static_acc_stddev_mps2 = opt->auto_zupt_static_acc_stddev_mps2;
    s->ars_cfg.auto_zaru_dwell_sec              = opt->auto_zupt_dwell_sec;
    s->ars_cfg.zero_rot_stddev_rps              = init->zero_rot_stddev_rps;

    s->ahrs_cfg.auto_zaru_disable                = s->ars_cfg.auto_zaru_disable;
    s->ahrs_cfg.auto_zaru_static_gyr_rps         = s->ars_cfg.auto_zaru_static_gyr_rps;
    s->ahrs_cfg.auto_zaru_static_acc_mps2        = s->ars_cfg.auto_zaru_static_acc_mps2;
    s->ahrs_cfg.auto_zaru_static_gyr_stddev_rps  = s->ars_cfg.auto_zaru_static_gyr_stddev_rps;
    s->ahrs_cfg.auto_zaru_static_acc_stddev_mps2 = s->ars_cfg.auto_zaru_static_acc_stddev_mps2;
    s->ahrs_cfg.auto_zaru_dwell_sec              = s->ars_cfg.auto_zaru_dwell_sec;
    s->ahrs_cfg.zero_rot_stddev_rps              = s->ars_cfg.zero_rot_stddev_rps;

    /* The vertical channel's zero-velocity pseudo-measurement answers the
       same "how well do I trust a detected standstill" question as ins's,
       restricted to the vertical axis, and it is fed from the very same
       trigger (REQ-SUITE-015). One configured value, one meaning. */
    s->baro_cfg.zupt_stddev_mps = init->zero_vel_stddev_mps;

    /* Manual ins init (opt.auto_init == false): the caller already knows the
       initial attitude (init.rpy_init_rad[2]). Seed the ARS's bootstrap yaw
       with it instead of the arbitrary 0 (REQ-SUITE-002), so the suite's
       best-available attitude reflects the true heading during the
       ATTITUDE_ONLY window. Roll/pitch stay accelerometer-derived. The
       magnetometer AHRS is unaffected. */
    s->ars_yaw_from_init = !opt->auto_init;
    if (s->ars_yaw_from_init) { s->ars_cfg.rpy_init_rad[2] = init->rpy_init_rad[2]; }

    const int rc = ins_init(&s->ins, init, opt);

    /* Manual init: the origin is fixed by the caller, nothing to align
       (baro_alt anchors to the NED height later), so latch it as the
       datum's origin right away. Auto-init: ins is still collecting; its
       origin is not established yet and gets aligned with the baro datum
       at bootstrap (see nav_suite_update). */
    s->vertical_datum_aligned = false;
    if (s->ins.is_initialized) { nav_suite_latch_datum_origin(s); }

    return rc;
}

/* Bootstrap an AHRS instance from the current epoch: roll/pitch from
 * accelerometer leveling; yaw from the magnetometer (AHRS mode), or (unless
 * keep_init_yaw) 0 (ARS mode without a known initial attitude). keep_init_yaw
 * preserves whatever is already in cfg->rpy_init_rad[2] (the ARS's known
 * initial yaw, see nav_suite_t.ars_yaw_from_init) and is only ever true for the
 * ARS call site, never for the magnetometer AHRS. */
/* @satisfies REQ-SUITE-002 */
static void nav_suite_ahrs_bootstrap(ahrs_t* a, ahrs_config_t* cfg, const ins_measurements_t* m,
                                     const float* mag_b, bool keep_init_yaw)
{
    float roll, pitch;
    ahrs_leveling_from_acc(m->acc.data, &roll, &pitch);

    cfg->rpy_init_rad[0] = roll;
    cfg->rpy_init_rad[1] = pitch;
    if (!keep_init_yaw)
    {
        cfg->rpy_init_rad[2] = (mag_b != NULL) ? ahrs_mag_heading(mag_b, roll, pitch) : 0.0f;
    }
    cfg->rpy_init_stddev_rad[0] = NAV_SUITE_AHRS_INIT_RP_STDDEV;
    cfg->rpy_init_stddev_rad[1] = NAV_SUITE_AHRS_INIT_RP_STDDEV;
    cfg->rpy_init_stddev_rad[2] = NAV_SUITE_AHRS_INIT_YAW_STDDEV;

    (void)ahrs_init(a, cfg, m->timestamp);
}

/* Attitude source for consumers that need a quaternion: same fallback order as
 * nav_suite_get_rpy (ins if ready, else the magnetometer AHRS, else the ARS).
 * Without nav_suite_get_rpy's heading carry-over (REQ-SUITE-022): the only
 * consumer is baro_alt's vertical channel, whose projection of the
 * accelerometer onto the down axis is yaw-invariant. */
static bool nav_suite_best_quaternion(const nav_suite_t* s, float q_bn[4])
{
    if (ins_is_ready(&s->ins) && ins_get_quaternion(&s->ins, q_bn)) { return true; }
    if (ahrs_get_quaternion(&s->ahrs, q_bn)) { return true; }
    return ahrs_get_quaternion(&s->ars, q_bn);
}

/* Vertical datum reconciliation for the GNSS/IMU case: if the baro was already
 * running when ins bootstrapped, the two disagree about where h = 0 is. The
 * datum is fixed by whoever was first (the baro here) and ins conforms to it:
 * shift the ins origin down so its local height matches the baro height. The
 * absolute solution is unchanged (see ins_shift_origin_down). The offset filter
 * is baro-anchored and the baro did NOT move, so it must NOT be touched here.
 *
 * This handles ONLY the non-external case. When a local positioning system owns
 * the n-frame (s->local_frame_external), ins is never shifted:
 * ins_shift_origin_down()'s contract requires the external frame to be shifted
 * along, which the wrapper cannot do. That case is reconciled at the source
 * instead: ins is fed local_pos already offset onto the datum.
 *
 * Re-arms per n-frame ORIGIN, not per ins instance: a re-bootstrap that
 * anchored a new origin has to re-join the datum, one that kept the origin
 * carried across the re-arm (REQ-NAV-062) is still standing on it. */
/* @satisfies REQ-SUITE-007 REQ-NAV-062 */
static void nav_suite_align_vertical_datum(nav_suite_t* s)
{
    if (!s->ins.is_initialized) { return; }
    if (s->vertical_datum_aligned && nav_suite_ins_origin_is_datum(s)) { return; }
    if (!s->local_frame_external)
    {
        float h_baro, pos_ned[3];
        if (baro_alt_get_height(&s->baro_alt, &h_baro) && ins_get_position_local(&s->ins, pos_ned))
        {
            const float dz = h_baro + pos_ned[2];
            if (fabsf(dz) > 0.01f)
            {
                LOG_INFO("nav_suite: aligning vertical datum, shifting ins origin by %.2f m "
                         "to match the baro_alt datum",
                         (double)dz);
            }
            /* Shifting the ins origin down by dz makes -pos_local[2] == h_baro. */
            ins_shift_origin_down(&s->ins, dz);
        }
    }
    nav_suite_latch_datum_origin(s);
}

/* The local height reference: the height above the NED origin from the source
 * that survives a GNSS outage. This is what the offset filter is fed with and
 * what the absolute height is evaluated against - both must use the SAME
 * source, otherwise the offset does not belong to the height it is added to.
 *
 * baro_alt comes first even when ins is running and better right now: the
 * offset has to absorb the barometer's drift against the ellipsoid while GNSS
 * is up, because the barometer is what remains when GNSS drops. Without a
 * barometer the ins local height is the reference, and the offset degenerates
 * to the datum's ellipsoid height. */
typedef struct
{
    float h_m;      /**< height above the NED origin [m], positive up */
    float v_up_mps; /**< climb rate of that source [m/s], positive up */
    float stddev_m; /**< 1-sigma of h_m [m]; 0 -> offset filter default */
    bool  has_v;    /**< is v_up_mps usable (delay extrapolation)? */
} nav_suite_local_ref_t;

/* @satisfies REQ-SUITE-008 */
static bool nav_suite_local_ref(const nav_suite_t* s, float baro_stddev_m, nav_suite_local_ref_t* r)
{
    memset(r, 0, sizeof(*r));

    if (baro_alt_get_height(&s->baro_alt, &r->h_m))
    {
        /* The barometer's own sample accuracy. The filtered height is better
           than a single sample, but the offset filter derates every pair anyway
           (stddev_inflation_factor). */
        r->stddev_m = baro_stddev_m;
        r->has_v    = baro_alt_get_velocity(&s->baro_alt, &r->v_up_mps);
        return true;
    }

    float pos_ned[3];
    if (ins_is_ready(&s->ins) && ins_get_position_local(&s->ins, pos_ned))
    {
        r->h_m = -pos_ned[2];
        /* stddev_m stays 0 -> the offset filter's configured default. ins's
           vertical position uncertainty is not reported here on purpose: with
           the stddev derating and the update decimation, the exact R would not
           earn back an accessor reaching into ins's covariance. */
        float vel_ned[3];
        if (ins_get_velocity_ned(&s->ins, vel_ned))
        {
            r->v_up_mps = -vel_ned[2];
            r->has_v    = true;
        }
        return true;
    }
    return false;
}

/* Offset filter: feed it from every epoch carrying both a GNSS position and a
 * local height reference. No IMU sample is needed this epoch: the reference is
 * then one epoch stale, far below the offset filter's decimation interval. */
/* @satisfies REQ-SUITE-008 */
static void nav_suite_update_local_gnss(nav_suite_t* s, const ins_measurements_t* m)
{
    if (!m->gnss_pos.is_valid) { return; }

    nav_suite_local_ref_t ref;
    if (!nav_suite_local_ref(s, m->baro.is_valid ? m->baro.stddev_m : 0.0f, &ref)) { return; }

    /* GNSS side: ellipsoid height from the fix, vertical stddev from the NED
       covariance (element (2,2)). This weights a fusion, so it takes the
       conditioned covariance ins fuses with (REQ-NAV-038). Pairs without a
       usable vertical accuracy are skipped. */
    float gnss_pos_Qll_fuse[3 * 3];
    ins_gnss_condition_pos_cov(&s->ins.opt, m->gnss_pos.Qll_ned, gnss_pos_Qll_fuse);
    const float var_v = gnss_pos_Qll_fuse[8];
    if (!(var_v > 0.0f) || !isfinite(var_v)) { return; }
    double lat, lon, h_ell;
    ins_ecef_to_latlonh(m->gnss_pos.xyz_ecef, &lat, &lon, &h_ell);

    /* If the GNSS measurement is delayed, evaluate the local height at
       the GNSS time of validity using the reference's climb rate: both
       sides of the pair must refer to the same instant. */
    float h_local = ref.h_m;
    if (m->gnss_delay_ms > 0 && ref.has_v)
    {
        h_local -= ref.v_up_mps * (float)m->gnss_delay_ms * 0.001f;
    }

    if (!s->local_gnss.is_initialized)
    {
        (void)local_gnss_alt_init(&s->local_gnss, &s->local_gnss_cfg, m->timestamp, h_local,
                                  ref.stddev_m, (float)h_ell, SQRTF(var_v));
    }
    else
    {
        local_gnss_alt_update(&s->local_gnss, m->timestamp, h_local, ref.stddev_m, (float)h_ell,
                              SQRTF(var_v));
    }

    /* Anchor the drift correction of REQ-SUITE-008: where the offset filter
       stood when ins's vertical datum was fixed. Everything the filter does
       from HERE is the datum's drift, so the difference is what the absolute
       height has to take back off ins. Anchored here rather than in the datum
       latch itself because the filter needs a GNSS pair to exist at all, and
       the latch can happen before the first one.

       The anchor tracks the estimate for as long as the filter is still
       CONVERGING, and freezes on the first pair that no longer improves its
       variance. A single snapshot would not do: taken while the filter is
       still settling it books the remaining convergence as drift that never
       happened, and since the anchor is a constant that error stays as a bias
       on every absolute height for the rest of the run. Tracking costs
       nothing - while the anchor follows the estimate the correction is zero,
       which is right, because a datum just fixed has not drifted yet. On a
       mission too short for the filter to settle it simply never freezes and
       ins's height is reported untouched, which is also right: nothing has
       been observed that would justify correcting it. */
    if (!s->datum_offset_valid && s->vertical_datum_aligned && s->ins.is_initialized &&
        s->ins.height_from_baro)
    {
        float off_now, sd_now;
        /* Only a pair the filter actually FUSED can move the variance; the
           decimated ones in between leave it exactly where it was and would
           read as "settled" on the very next epoch. */
        if (s->local_gnss.t_last != s->datum_offset_t_last &&
            local_gnss_alt_get(&s->local_gnss, &off_now, &sd_now))
        {
            const float var_now    = sd_now * sd_now;
            s->datum_offset_t_last = s->local_gnss.t_last;
            if (s->datum_offset_var_min <= 0.0f ||
                var_now < NAV_SUITE_DATUM_OFFSET_SETTLED * s->datum_offset_var_min)
            {
                s->datum_offset_var_min = var_now;
                s->datum_offset_m       = off_now;
            }
            else { s->datum_offset_valid = true; }
        }
    }
}

/* Reconcile an external local-positioning frame with the vertical datum BEFORE
 * ins sees the sample. A local system (lighthouse, ...) reports in its own
 * frame, whose origin is arbitrary; the datum is "0 at the start point" (or the
 * current baro height). A constant offset maps the reported frame onto the
 * datum and is subtracted from every local_pos, so ins always solves at
 * (0, 0, h) and never adopts the external frame's zero. That keeps the local
 * height continuous when the local system comes online after the baro.
 *
 * The offset is recomputed every epoch until ins locks the datum at its
 * bootstrap, then frozen as the fixed relation between the two frames, exposed
 * via nav_suite_get_local_pos_offset(). It stays frozen across ins resets.
 * Vertical uses the current baro height (0 if no baro yet), horizontal is
 * always zeroed.
 *
 * @satisfies REQ-SUITE-013 */
static const ins_measurements_t* nav_suite_apply_local_datum(nav_suite_t*              s,
                                                             const ins_measurements_t* m,
                                                             ins_measurements_t*       scratch)
{
    if (!m->local_pos.is_valid) { return m; }

    /* An external system owns the n-frame from its first sample onwards.
       Sticky across dropouts. */
    s->local_frame_external = true;

    if (!s->local_pos_datum_locked)
    {
        float h_datum = 0.0f;
        (void)baro_alt_get_height(&s->baro_alt, &h_datum);
        s->local_pos_offset_ned[0] = m->local_pos.pos_ned[0];
        s->local_pos_offset_ned[1] = m->local_pos.pos_ned[1];
        s->local_pos_offset_ned[2] = m->local_pos.pos_ned[2] + h_datum;
        s->local_pos_offset_valid  = true;
    }

    *scratch = *m;
    scratch->local_pos.pos_ned[0] -= s->local_pos_offset_ned[0];
    scratch->local_pos.pos_ned[1] -= s->local_pos_offset_ned[1];
    scratch->local_pos.pos_ned[2] -= s->local_pos_offset_ned[2];
    return scratch;
}

/* Refresh the heading pair that survives an ins shutdown (REQ-SUITE-022): the
 * 3D heading ins currently holds, together with the ARS yaw of the same epoch,
 * so a later re-init can restore the former by the amount the latter has turned
 * since. Called once per epoch after both filters have been updated; it simply
 * stops being refreshed when ins stops being ready, leaving the last good epoch
 * latched.
 *
 * Gated on ins having actually resolved its heading
 * (NAV_SUITE_YAW_CARRY_LATCH_MAX_STDDEV): a yaw ins itself is not sure of is
 * not a reference to come back to. That also covers the coasting case.
 *
 * @satisfies REQ-SUITE-022 */
static void nav_suite_update_yaw_carry(nav_suite_t* s, ins_time_us_t t)
{
    float ins_roll, ins_pitch, ins_yaw;
    float ins_roll_sd, ins_pitch_sd, ins_yaw_sd;
    float ars_roll, ars_pitch, ars_yaw;

    if (!ins_is_ready(&s->ins) || !s->ars.is_initialized) { return; }
    if (!ins_get_rpy(&s->ins, &ins_roll, &ins_pitch, &ins_yaw)) { return; }
    if (!ins_get_rpy_stddev(&s->ins, &ins_roll_sd, &ins_pitch_sd, &ins_yaw_sd)) { return; }
    if (!(ins_yaw_sd > 0.0f) || ins_yaw_sd > NAV_SUITE_YAW_CARRY_LATCH_MAX_STDDEV) { return; }
    if (!ahrs_get_rpy(&s->ars, &ars_roll, &ars_pitch, &ars_yaw)) { return; }

    s->yaw_carry.valid       = true;
    s->yaw_carry.yaw_ins_rad = ins_yaw;
    s->yaw_carry.yaw_ars_rad = ars_yaw;
    s->yaw_carry.stddev_rad  = ins_yaw_sd;
    s->yaw_carry.t           = t;
}

/* Resolve the carried-over heading for an ins re-init at time t
 * (REQ-SUITE-022): the latched 3D heading advanced by the yaw the ARS has
 * free-integrated since the latch. The ARS drifts in absolute terms, which is
 * why its yaw is never used as a heading itself (REQ-SUITE-016), but over an
 * outage it tracks the CHANGE well. Returns false if nothing usable exists.
 *
 * The uncertainty is ins's own 1-sigma at the latch plus what the ARS
 * accumulated since: its z gyro-bias 1-sigma integrated over the elapsed time,
 * inflated like every other ARS-derived hint value. That term is what
 * eventually retires the carry-over (NAV_SUITE_YAW_CARRY_MAX_STDDEV).
 *
 * @satisfies REQ-SUITE-022 */
static bool nav_suite_carried_yaw(const nav_suite_t* s, ins_time_us_t t, float* yaw_rad,
                                  float* stddev_rad)
{
    float ars_roll, ars_pitch, ars_yaw, gyr_bias_sd[3];

    if (!s->yaw_carry.valid || !s->ars.is_initialized) { return false; }
    if (!ahrs_get_rpy(&s->ars, &ars_roll, &ars_pitch, &ars_yaw)) { return false; }
    if (!ahrs_get_bias_gyr_stddev(&s->ars, gyr_bias_sd)) { return false; }

    float dt_sec = (float)(t - s->yaw_carry.t) * 1.0e-6f;
    if (dt_sec < 0.0f) { dt_sec = 0.0f; }

    const float drift_sd = NAV_SUITE_ATT_HINT_STDDEV_INFLATION * gyr_bias_sd[2] * dt_sec;
    const float sd = SQRTF(s->yaw_carry.stddev_rad * s->yaw_carry.stddev_rad + drift_sd * drift_sd);
    if (!(sd > 0.0f) || sd > NAV_SUITE_YAW_CARRY_MAX_STDDEV) { return false; }

    const float delta = ins_angle_diff(ars_yaw, s->yaw_carry.yaw_ars_rad);
    *yaw_rad          = ins_angle_diff(s->yaw_carry.yaw_ins_rad + delta, 0.0f);
    *stddev_rad       = sd;
    return true;
}

/* Attitude/gyro-bias seed for ins's own auto-init and re-acquisition
 * (REQ-NAV-048), built from the parallel ARS/AHRS's CURRENT state - one epoch
 * behind ins's own update, since ins runs first. That staleness is a single IMU
 * sample and is dwarfed by the stddev inflation applied here.
 *
 * Prefers the magnetometer AHRS (has a real yaw estimate) over the ARS
 * (gyro-only), falls back to the ARS, and leaves the hint invalid if neither is
 * initialized yet, so ins falls back to its own leveling.
 *
 * While ins is not initialized, two further fallbacks fill in what the live
 * filters cannot offer: the caller's static initial hint (REQ-SUITE-017) and,
 * for a filter that has run before, its own last converged heading advanced by
 * the ARS (REQ-SUITE-022). Both only ever ADD to the resolved hint.
 *
 * @satisfies REQ-SUITE-016 */
static void nav_suite_build_att_hint(const nav_suite_t* s, ins_time_us_t t,
                                     ins_meas_att_hint_t* hint)
{
    memset(hint, 0, sizeof(*hint));

    const ahrs_t* src = NULL;
    if (s->ahrs.is_initialized) { src = &s->ahrs; }
    else if (s->ars.is_initialized) { src = &s->ars; }

    if (src != NULL)
    {
        float roll, pitch, yaw, roll_sd, pitch_sd, yaw_sd, gb[3], gb_sd[3];
        if (ahrs_get_rpy(src, &roll, &pitch, &yaw) &&
            ahrs_get_rpy_stddev(src, &roll_sd, &pitch_sd, &yaw_sd) && ahrs_get_bias_gyr(src, gb) &&
            ahrs_get_bias_gyr_stddev(src, gb_sd))
        {
            int i;
            hint->is_valid         = true;
            hint->roll_rad         = roll;
            hint->pitch_rad        = pitch;
            hint->stddev_roll_rad  = NAV_SUITE_ATT_HINT_STDDEV_INFLATION * roll_sd;
            hint->stddev_pitch_rad = NAV_SUITE_ATT_HINT_STDDEV_INFLATION * pitch_sd;
            if (yaw_sd > 0.0f) /* AHRS only - the ARS's yaw free-integrates, no stddev */
            {
                hint->yaw_rad        = yaw;
                hint->stddev_yaw_rad = NAV_SUITE_ATT_HINT_STDDEV_INFLATION * yaw_sd;
            }
            for (i = 0; i < 3; ++i)
            {
                hint->gyr_bias_rps[i]        = gb[i];
                hint->stddev_gyr_bias_rps[i] = NAV_SUITE_ATT_HINT_STDDEV_INFLATION * gb_sd[i];
            }
        }
    }

    /* Static initial-hint fallback (nav_suite_set_init_att_hint), only while
       ins itself has not yet initialized - a stale "initial" hint must not bias
       a later re-acquisition (REQ-NAV-048). Yaw and roll/pitch fall back
       independently: the common case is a yaw-free ARS already leveled from
       gravity plus a caller who knows the initial heading, so the ARS's
       roll/pitch must still win and only the yaw needs the static hint. */
    if (!s->ins.is_initialized)
    {
        if (hint->stddev_yaw_rad <= 0.0f && s->init_att_hint.stddev_yaw_rad > 0.0f)
        {
            hint->is_valid       = true;
            hint->yaw_rad        = s->init_att_hint.yaw_rad;
            hint->stddev_yaw_rad = s->init_att_hint.stddev_yaw_rad;
        }
        if (src == NULL && s->init_att_hint.stddev_roll_rad > 0.0f &&
            s->init_att_hint.stddev_pitch_rad > 0.0f)
        {
            hint->is_valid         = true;
            hint->roll_rad         = s->init_att_hint.roll_rad;
            hint->pitch_rad        = s->init_att_hint.pitch_rad;
            hint->stddev_roll_rad  = s->init_att_hint.stddev_roll_rad;
            hint->stddev_pitch_rad = s->init_att_hint.stddev_pitch_rad;
        }

        /* Carried-over heading (REQ-SUITE-022): ins ran before, held a
           converged 3D heading, and was shut down. Restore it, advanced by the
           ARS's tracked yaw change. Last in line because a magnetometer AHRS
           measures the heading while this only reconstructs it. */
        if (hint->stddev_yaw_rad <= 0.0f)
        {
            float carried_yaw, carried_sd;
            if (nav_suite_carried_yaw(s, t, &carried_yaw, &carried_sd))
            {
                hint->is_valid       = true;
                hint->yaw_rad        = carried_yaw;
                hint->stddev_yaw_rad = carried_sd;
            }
        }
    }
}

/* @satisfies REQ-SUITE-002 REQ-SUITE-017 */
void nav_suite_set_init_att_hint(nav_suite_t* s, float roll_rad, float pitch_rad,
                                 float stddev_roll_pitch_rad, float yaw_rad, float stddev_yaw_rad)
{
    if (s == NULL) return;
    memset(&s->init_att_hint, 0, sizeof(s->init_att_hint));
    if (stddev_roll_pitch_rad > 0.0f)
    {
        s->init_att_hint.roll_rad         = roll_rad;
        s->init_att_hint.pitch_rad        = pitch_rad;
        s->init_att_hint.stddev_roll_rad  = stddev_roll_pitch_rad;
        s->init_att_hint.stddev_pitch_rad = stddev_roll_pitch_rad;
    }
    if (stddev_yaw_rad > 0.0f)
    {
        s->init_att_hint.yaw_rad        = yaw_rad;
        s->init_att_hint.stddev_yaw_rad = stddev_yaw_rad;
    }

    /* A known initial heading is a statement about the vehicle, not about one
       filter: the yaw-free ARS is the suite's attitude reference during the
       ATTITUDE_ONLY window, so it bootstraps from the same known yaw instead of
       the arbitrary 0 (REQ-SUITE-002). Only meaningful before the ARS
       bootstraps; once it runs, its yaw is the free-integrated continuation of
       that seed. The manual-init seed of nav_suite_init() stays untouched.
       Roll/pitch are NOT seeded: the ARS levels them from gravity on its first
       epoch. */
    if (!s->ars.is_initialized)
    {
        if (stddev_yaw_rad > 0.0f)
        {
            s->ars_cfg.rpy_init_rad[2] = yaw_rad;
            s->ars_yaw_from_init       = true;
        }
        else if (s->ins.opt.auto_init) { s->ars_yaw_from_init = false; }
    }
}

/* @satisfies REQ-SUITE-001 REQ-SUITE-003 REQ-SUITE-021 */
int nav_suite_predict_step(nav_suite_t* s, const ins_measurements_t* m, float* phi_out)
{
    /* A usable GNSS fix anchors the ins origin to WGS84: at bootstrap (origin =
       GNSS ECEF) or by later fusion. Latch it so the ellipsoid accessors know
       the absolute solution is real and not the prescribed init origin
       (REQ-SUITE-008). */
    if (m->gnss_pos.is_valid && m->gnss_pos.Qll_ned[8] > 0.0f && isfinite(m->gnss_pos.Qll_ned[8]))
    {
        s->wgs84_anchor_seen = true;
    }

    ins_measurements_t        mi;
    const ins_measurements_t* m_ins = nav_suite_apply_local_datum(s, m, &mi);

    /* Explicit local copy so the ARS/AHRS attitude hint can be attached
       without mutating the caller's struct (m_ins may alias m itself,
       see nav_suite_apply_local_datum). */
    ins_measurements_t m_hinted = *m_ins;
    nav_suite_build_att_hint(s, m->timestamp, &m_hinted.att_hint);
    const int ins_status = ins_predict_step(&s->ins, &m_hinted, phi_out);

    /* Hand off to nav_suite_correct_step(): the caller's original, unmodified
       measurement block. Everything ins-specific is already carried across by
       ins_predict_step() inside ins_t.step_ctx, nav_suite only needs to
       remember what it consumes afterwards. */
    s->step_ctx.m      = *m;
    s->step_ctx.active = true;
    return ins_status;
}

/* @satisfies REQ-SUITE-009 REQ-SUITE-012 REQ-SUITE-013 REQ-SUITE-016 REQ-SUITE-019
 * @satisfies REQ-SUITE-021 */
void nav_suite_correct_step(nav_suite_t* s)
{
    if (!s->step_ctx.active) return;
    s->step_ctx.active          = false;
    const ins_measurements_t* m = &s->step_ctx.m;

    ins_correct_step(&s->ins);

    /* The static initial-hint (nav_suite_set_init_att_hint, REQ-SUITE-017) is
       consumed exactly once: cleared as soon as ins has bootstrapped, so a
       later full re-bootstrap does not reuse a heading that may no longer hold.
       Checked via the stddev fields directly (same convention
       nav_suite_build_att_hint reads), not is_valid. */
    if (s->ins.is_initialized &&
        (s->init_att_hint.stddev_roll_rad > 0.0f || s->init_att_hint.stddev_yaw_rad > 0.0f))
    {
        memset(&s->init_att_hint, 0, sizeof(s->init_att_hint));
    }

    /* Diagnostics (see log.h): edge-triggered on ins losing its initialization
       (a GNSS quality-loss re-arm, a health-check or time-jump reset). Reports
       what the re-init will have to work with (REQ-SUITE-022). */
    if (s->log_state.last_ins_initialized && !s->ins.is_initialized && s->yaw_carry.valid)
    {
        LOG_INFO("nav_suite: ins shut down, heading carry-over armed at %.0f deg "
                 "(stddev %.1f deg), tracked by the ARS from here",
                 (double)RAD2DEG(s->yaw_carry.yaw_ins_rad),
                 (double)RAD2DEG(s->yaw_carry.stddev_rad));
    }
    s->log_state.last_ins_initialized = s->ins.is_initialized;

    /* Freeze the local-frame offset once ins has locked the datum at its
       bootstrap: from now on it is the fixed frame relation, not a moving
       target. */
    if (m->local_pos.is_valid && s->ins.is_initialized) { s->local_pos_datum_locked = true; }

    nav_suite_align_vertical_datum(s);
    nav_suite_update_local_gnss(s, m);

    /* Solution-mode transition diagnostics (see log.h): edge-triggered on
       nav_suite_get_mode() so a steady mode does not produce one line per
       epoch. Placed here so it still runs on the early-return path below. */
    {
        const nav_suite_mode_t mode = nav_suite_get_mode(s);
        if (mode != s->log_state.last_mode)
        {
#if LOG_LEVEL >= LOG_LEVEL_INFO
            /* Guarded (not just relying on LOG_INFO's own no-op expansion):
               otherwise this table would sit unused/warned-about in a
               LOG_LEVEL_NONE build, since it is not referenced anywhere
               except inside the macro's now-empty expansion. */
            static const char* const mode_name[] = {"NONE", "ATTITUDE_ONLY", "COASTING", "FULL"};
            LOG_INFO("nav_suite: mode %s -> %s", mode_name[s->log_state.last_mode],
                     mode_name[mode]);
#endif
            s->log_state.last_mode = mode;
        }
    }

    if (!m->acc.is_valid || !m->gyr.is_valid)
    {
        return; /* the AHRS filters are pure IMU(+mag) filters */
    }

    /* The ARS/AHRS/baro filters run the SAME physical sensors as ins, so they
       must see the same calibrated signal (REQ-SUITE-012). ins calibrates its
       own copy internally; the identical calibration is applied once to a local
       copy here to drive the parallel filters. */
    ins_measurements_t mc = *m;
    ins_apply_calibration(&s->ins.opt, &mc);
    m                  = &mc;
    const float* mag_b = m->mag.is_valid ? m->mag.data : NULL;

    /* Zero-rotation trigger for the ARS/AHRS (REQ-SUITE-009): the caller's
       explicit flag, OR'd with ins currently being inside an auto-ZUPT/ZARU
       stillness run (ins_auto_zupt_active), true for the run's whole duration.
       Persisted (REQ-SUITE-010) so a diagnostic caller can read back whether a
       zero-rotation update was applied this epoch. */
    const bool zaru_trigger = m->zero_rotation_update || ins_auto_zupt_active(&s->ins);
    s->last_zaru_trigger    = zaru_trigger;
    s->last_vertical_zupt   = false; /* set below iff the vertical filter is fed */

    if (!s->ars.is_initialized)
    {
        /* A (re-)bootstrapping ARS restarts its yaw from the config seed,
           so any difference against a yaw latched before it is a jump,
           not a tracked rotation: the carry-over loses its reference
           (REQ-SUITE-022). */
        s->yaw_carry.valid = false;
        nav_suite_ahrs_bootstrap(&s->ars, &s->ars_cfg, m, NULL, s->ars_yaw_from_init);
    }
    else { ahrs_update(&s->ars, m->timestamp, m->gyr.data, m->acc.data, NULL, zaru_trigger); }

    if (!s->ahrs.is_initialized)
    {
        /* The heading bootstrap needs a magnetometer sample. The
           magnetometer AHRS has its own independent heading reference,
           unaffected by ars_yaw_from_init (see nav_suite_ahrs_bootstrap). */
        if (mag_b != NULL) { nav_suite_ahrs_bootstrap(&s->ahrs, &s->ahrs_cfg, m, mag_b, false); }
    }
    else { ahrs_update(&s->ahrs, m->timestamp, m->gyr.data, m->acc.data, mag_b, zaru_trigger); }

    /* Both filters are up to date for this epoch: refresh the heading pair
       a later ins re-init reads back, then resolve it once so the const
       accessors can report it without a timestamp (REQ-SUITE-022). */
    nav_suite_update_yaw_carry(s, m->timestamp);
    {
        float carried_sd;
        s->yaw_carry.out_valid =
            nav_suite_carried_yaw(s, m->timestamp, &s->yaw_carry.out_yaw_rad, &carried_sd);
    }

    /* Cross-check diagnostic (see log.h): ins and the ARS estimate a gyro bias
       from the SAME physical gyro but are otherwise independent estimators (ins
       additionally uses position aiding). A large, persistent gap usually means
       one of them has not converged - exactly what this suite's
       independent-attitude-reference design (REQ-SYS-002) exists to catch.
       Throttled while it persists. */
    if (s->ins.is_initialized && s->ars.is_initialized)
    {
        int   i;
        float max_diff_rps = 0.0f;
        for (i = 0; i < 3; ++i)
        {
            const float d = fabsf(s->ins.state.gyr_bias[i] - s->ars.gyr_bias_rps[i]);
            if (d > max_diff_rps) { max_diff_rps = d; }
        }
        const float max_diff_dps = RAD2DEG(max_diff_rps);
        if (max_diff_dps >= NAV_SUITE_LOG_GYR_BIAS_DIFF_WARN_DPS)
        {
            const bool  first_warn = (s->log_state.t_last_gyr_bias_warn == 0);
            const float since_warn_sec =
                first_warn ? 0.0f
                           : (float)(m->timestamp - s->log_state.t_last_gyr_bias_warn) * 1.0e-6f;
            if (first_warn || since_warn_sec >= NAV_SUITE_LOG_GYR_BIAS_DIFF_REPEAT_SEC)
            {
                LOG_WARN("ins/ARS gyro bias disagreement %.2f deg/s - cross-check "
                         "mismatch",
                         (double)max_diff_dps);
                s->log_state.t_last_gyr_bias_warn = m->timestamp;
            }
        }
        /* No re-arm when the difference drops back below the bound: the
           throttle timestamp is what limits the repeat rate, and clearing it
           here would make the very next re-crossing take the first_warn path
           again. A quantity dithering around the bound would then print on
           every crossing, i.e. not be throttled at all. */
    }

    /* Cross-check diagnostic (see log.h): ins and the magnetometer AHRS
       independently estimate heading. nav_suite_get_rpy() falls back from ins
       to the AHRS the moment ins leaves FULL/COASTING (REQ-SUITE-005), so a
       persistent gap here is not an abstract inconsistency - it is the size
       of the step the reported heading will take at the next mode switch,
       the same argument REQ-SUITE-019 makes for height. */
    /* @satisfies REQ-SUITE-023 */
    if (s->ins.is_initialized && s->ahrs.is_initialized)
    {
        float ins_roll, ins_pitch, ins_yaw, ahrs_roll, ahrs_pitch, ahrs_yaw;
        if (ins_get_rpy(&s->ins, &ins_roll, &ins_pitch, &ins_yaw) &&
            ahrs_get_rpy(&s->ahrs, &ahrs_roll, &ahrs_pitch, &ahrs_yaw))
        {
            const float diff_deg = fabsf(RAD2DEG(ins_angle_diff(ins_yaw, ahrs_yaw)));
            if (diff_deg >= NAV_SUITE_LOG_YAW_DIFF_WARN_DEG)
            {
                const bool  first_warn = (s->log_state.t_last_yaw_warn == 0);
                const float since_warn_sec =
                    first_warn ? 0.0f
                               : (float)(m->timestamp - s->log_state.t_last_yaw_warn) * 1.0e-6f;
                if (first_warn || since_warn_sec >= NAV_SUITE_LOG_YAW_DIFF_REPEAT_SEC)
                {
                    LOG_WARN("nav_suite: ins/AHRS heading disagreement %.1f deg (ins %.1f, "
                             "ahrs %.1f) - cross-check mismatch, the reported heading will "
                             "step if the solution mode changes",
                             (double)diff_deg, (double)RAD2DEG(ins_yaw), (double)RAD2DEG(ahrs_yaw));
                    s->log_state.t_last_yaw_warn = m->timestamp;
                }
            }
            /* Deliberately not re-armed on the way back below the bound,
               same reason as the gyro-bias check above. */
        }
    }

    /* Baro/accel vertical channel: bootstrapped by the first valid pressure
       sample once an attitude reference is available (the anchor sample defines
       h = 0), then propagated every IMU epoch. The bootstrap condition re-arms
       if the filter trips its health check. */
    /* @satisfies REQ-SUITE-006 */
    float q_bn[4];
    if (nav_suite_best_quaternion(s, q_bn))
    {
        if (!s->baro_alt.is_initialized)
        {
            /* Anchor the datum on the MEAN of the plausible pressure
               samples over a short window, not a single sample, so one
               glitched startup reading cannot skew it (REQ-SUITE-006).
               Gross/non-finite samples are screened out of the mean. */
            if (m->baro.is_valid && baro_alt_pressure_plausible(m->baro.pressure_pa))
            {
                if (s->baro_boot_count == 0)
                {
                    s->baro_boot_t0    = m->timestamp;
                    s->baro_boot_p_sum = 0.0;
                }
                s->baro_boot_p_sum += (double)m->baro.pressure_pa;
                s->baro_boot_count++;

                const bool window_done = (m->timestamp - s->baro_boot_t0) >=
                                         (baro_alt_time_us_t)(NAV_SUITE_BARO_BOOT_SEC * 1.0e6f);
                if (window_done && s->baro_boot_count >= 2)
                {
                    /* Anchor at the current NED height if ins already provides
                       one, so both filters share the origin datum
                       (REQ-SUITE-007); otherwise the datum is the start point
                       (h_init = 0) and ins joins it at its bootstrap via the
                       origin shift.

                       Gated on is_initialized, NOT on ins_is_ready(): a datum
                       anchor needs the two filters to agree on a zero, not a
                       converged solution. Waiting for readiness would anchor at
                       0 while ins's local height had already moved away from
                       it, leaving a permanent datum offset. */
                    float h_init = 0.0f;
                    float pos_ned[3];
                    if (ins_get_position_local(&s->ins, pos_ned)) { h_init = -pos_ned[2]; }
                    const float p_mean = (float)(s->baro_boot_p_sum / (double)s->baro_boot_count);
                    if (baro_alt_init(&s->baro_alt, &s->baro_cfg, m->timestamp, p_mean, h_init,
                                      0.0f) == 0)
                    {
                        s->baro_boot_count = 0; /* reset for a future re-bootstrap */
                    }
                }
            }
        }
        else
        {
            s->baro_boot_count = 0; /* running: keep the accumulator idle */
            baro_alt_update(&s->baro_alt, m->timestamp, m->acc.data, q_bn, m->baro.pressure_pa,
                            m->baro.stddev_m, m->baro.is_valid);
            /* A zero-rotation update means the platform is standing still, and
               a platform standing still has no vertical velocity either: the
               same trigger that feeds the ARS/AHRS (REQ-SUITE-009) drives the
               vertical channel's zero-velocity update. Deliberately NOT gated
               on any velocity estimate.

               Read back from the ARS/AHRS (ahrs_zaru_applied) rather than from
               zaru_trigger alone, so the velocity-blind auto-ZARU fallback
               counts too: without an absolute position aid ins never
               initializes, and that fallback is then the suite's only
               stillness detector. */
            /* @satisfies REQ-SUITE-015 */
            s->last_vertical_zupt =
                zaru_trigger || ahrs_zaru_applied(&s->ars) || ahrs_zaru_applied(&s->ahrs);
            if (s->last_vertical_zupt) { baro_alt_zero_velocity_update(&s->baro_alt, 0.0f); }

            /* Cross-check diagnostic (see log.h): ins's own vertical channel is
               its weakest axis; baro_alt is an independent vertical estimator
               fed from the same accelerometer. A persistent gap usually means
               the GNSS height solution or the baro_alt datum has a problem. */
            if (s->ins.is_initialized)
            {
                float vel_ned[3], v_baro_up;
                if (ins_get_velocity_ned(&s->ins, vel_ned) &&
                    baro_alt_get_velocity(&s->baro_alt, &v_baro_up))
                {
                    const float diff_mps = fabsf(-vel_ned[2] - v_baro_up);
                    if (diff_mps >= NAV_SUITE_LOG_VVEL_DIFF_WARN_MPS)
                    {
                        const bool  first_warn = (s->log_state.t_last_vvel_warn == 0);
                        const float since_warn_sec =
                            first_warn
                                ? 0.0f
                                : (float)(m->timestamp - s->log_state.t_last_vvel_warn) * 1.0e-6f;
                        if (first_warn || since_warn_sec >= NAV_SUITE_LOG_VVEL_DIFF_REPEAT_SEC)
                        {
                            LOG_WARN("nav_suite: ins/baro_alt vertical velocity disagreement "
                                     "%.2f m/s (ins %.2f, baro_alt %.2f) - cross-check mismatch",
                                     (double)diff_mps, (double)(-vel_ned[2]), (double)v_baro_up);
                            s->log_state.t_last_vvel_warn = m->timestamp;
                        }
                    }
                    /* Deliberately not re-armed on the way back below the
                       bound, same reason as the gyro-bias check above. */
                }

                /* The same cross-check on the height itself (REQ-SUITE-019).
                   This is the one the architecture leans on: the height
                   accessors fall back to baro_alt whenever ins is not in FULL
                   mode, so a silent divergence surfaces as a jump in the
                   reported height at the moment the mode changes. The velocity
                   check above cannot see it. */
                float h_ins_ned[3], h_baro_m;
                if (ins_get_position_local(&s->ins, h_ins_ned) &&
                    baro_alt_get_height(&s->baro_alt, &h_baro_m))
                {
                    const float diff_m = fabsf(-h_ins_ned[2] - h_baro_m);
                    if (diff_m >= NAV_SUITE_LOG_HEIGHT_DIFF_WARN_M)
                    {
                        const bool  first_warn = (s->log_state.t_last_height_warn == 0);
                        const float since_warn_sec =
                            first_warn
                                ? 0.0f
                                : (float)(m->timestamp - s->log_state.t_last_height_warn) * 1.0e-6f;
                        if (first_warn || since_warn_sec >= NAV_SUITE_LOG_HEIGHT_DIFF_REPEAT_SEC)
                        {
                            LOG_WARN("nav_suite: ins/baro_alt height disagreement %.1f m "
                                     "(ins %.1f, baro_alt %.1f) - cross-check mismatch, the "
                                     "reported height will step if the solution mode changes",
                                     (double)diff_m, (double)(-h_ins_ned[2]), (double)h_baro_m);
                            s->log_state.t_last_height_warn = m->timestamp;
                        }
                    }
                    /* Deliberately not re-armed on the way back below the
                       bound, same reason as the gyro-bias check above. */
                }
            }
        }
    }
}

/* @satisfies REQ-SUITE-021 */
void nav_suite_update(nav_suite_t* s, const ins_measurements_t* m)
{
    nav_suite_predict_step(s, m, NULL);
    nav_suite_correct_step(s);
}

/* @satisfies REQ-SUITE-005 */
nav_suite_mode_t nav_suite_get_mode(const nav_suite_t* s)
{
    if (s == NULL) return NAV_SUITE_MODE_NONE;

    if (ins_is_ready(&s->ins))
    {
        const int dr_ms = ins_deadreckoning_ms(&s->ins);
        return (dr_ms >= 0 && dr_ms <= NAV_SUITE_FRESH_AIDING_MS) ? NAV_SUITE_MODE_FULL
                                                                  : NAV_SUITE_MODE_COASTING;
    }
    if (s->ahrs.is_initialized || s->ars.is_initialized) { return NAV_SUITE_MODE_ATTITUDE_ONLY; }
    return NAV_SUITE_MODE_NONE;
}

/* @satisfies REQ-SUITE-010 */
bool nav_suite_get_zaru_active(const nav_suite_t* s)
{
    if (s == NULL) return false;
    /* Both sources, the same expression the vertical channel keys off
       (REQ-SUITE-015): the trigger computed here, OR'd with what the
       ARS/AHRS's own velocity-blind fallback decided. Reading only the
       first would answer "no zero-rotation update" throughout a
       standstill that has no absolute position aiding -- ins never
       initializes there, so it contributes no trigger at all, while the
       fallback is fusing one into the ARS/AHRS the whole time. */
    return s->last_zaru_trigger || ahrs_zaru_applied(&s->ars) || ahrs_zaru_applied(&s->ahrs);
}

/* @satisfies REQ-SUITE-015 */
bool nav_suite_get_vertical_zupt_active(const nav_suite_t* s)
{
    return s != NULL && s->last_vertical_zupt;
}

/* @satisfies REQ-SUITE-018 */
void nav_suite_set_auto_zupt_zaru_disable(nav_suite_t* s, bool disable)
{
    if (s == NULL) return;
    ins_set_auto_zupt_disable(&s->ins, disable);
    ahrs_set_auto_zaru_disable(&s->ars, disable);
    ahrs_set_auto_zaru_disable(&s->ahrs, disable);
    /* Persist into the templates too: nav_suite_ahrs_bootstrap() consumes
       ars_cfg/ahrs_cfg fresh on every (re-)bootstrap, so without this the
       disable would silently lapse the next time either filter
       re-initializes. */
    s->ars_cfg.auto_zaru_disable  = disable;
    s->ahrs_cfg.auto_zaru_disable = disable;
}

/* @satisfies REQ-SUITE-005 REQ-SUITE-022 */
bool nav_suite_get_rpy(const nav_suite_t* s, float* roll, float* pitch, float* yaw)
{
    if (s == NULL) return false;
    if (ins_is_ready(&s->ins) && ins_get_rpy(&s->ins, roll, pitch, yaw)) { return true; }
    /* Fallback order: magnetometer AHRS (absolute heading) before the
       ARS (yaw only relative/drifting). */
    if (ahrs_get_rpy(&s->ahrs, roll, pitch, yaw)) return true;
    if (!ahrs_get_rpy(&s->ars, roll, pitch, yaw)) { return false; }

    /* The ARS's raw yaw is relative to whatever it bootstrapped from, so on its
       own it would make the suite's heading jump the moment ins drops out.
       Where a 3D heading was held before, report that heading carried forward
       by the ARS's own yaw change instead (REQ-SUITE-022) - the same
       reconstruction ins is re-initialized from. Roll/pitch stay the ARS's.
       nav_suite_get_rpy_ars() keeps reporting the raw yaw. */
    if (s->yaw_carry.out_valid) { *yaw = s->yaw_carry.out_yaw_rad; }
    return true;
}

bool nav_suite_get_rpy_ins(const nav_suite_t* s, float* roll, float* pitch, float* yaw)
{
    if (s == NULL) return false;
    return ins_get_rpy(&s->ins, roll, pitch, yaw);
}

bool nav_suite_get_rpy_ars(const nav_suite_t* s, float* roll, float* pitch, float* yaw)
{
    if (s == NULL) return false;
    return ahrs_get_rpy(&s->ars, roll, pitch, yaw);
}

bool nav_suite_get_rpy_ahrs(const nav_suite_t* s, float* roll, float* pitch, float* yaw)
{
    if (s == NULL) return false;
    return ahrs_get_rpy(&s->ahrs, roll, pitch, yaw);
}

/* @satisfies REQ-SUITE-006 */
bool nav_suite_get_baro_alt(const nav_suite_t* s, float* h_m, float* v_mps)
{
    if (s == NULL || !s->baro_alt.is_initialized) { return false; }
    if (h_m != NULL && !baro_alt_get_height(&s->baro_alt, h_m)) { return false; }
    if (v_mps != NULL && !baro_alt_get_velocity(&s->baro_alt, v_mps)) { return false; }
    return true;
}

/* @satisfies REQ-SUITE-008 */
bool nav_suite_get_height(const nav_suite_t* s, float* h_m)
{
    if (s == NULL) return false;

    float      pos_ned[3];
    const bool ins_h = ins_is_ready(&s->ins) && ins_get_position_local(&s->ins, pos_ned);

    /* Fresh absolute position aiding: ins is the best source. */
    if (ins_h && nav_suite_get_mode(s) == NAV_SUITE_MODE_FULL)
    {
        *h_m = -pos_ned[2];
        return true;
    }
    /* Coasting or no ins: the baro filter keeps measuring the
       vertical channel (same datum -> continuous output). */
    if (baro_alt_get_height(&s->baro_alt, h_m)) { return true; }
    if (ins_h)
    {
        *h_m = -pos_ned[2];
        return true;
    }
    return false;
}

/* Has a real GNSS fix ever anchored the ins solution to WGS84? Only then is
 * ins.latlonh a true ellipsoid position; without it the absolute anchor is just
 * the prescribed init origin (a rough indoor guess, a lighthouse frame), which
 * must not be passed off as an ellipsoid height. Latched in nav_suite_update
 * from the presented fix, so it covers a GNSS bootstrap as well as later
 * fusion and stays true through an outage. */
static bool nav_suite_wgs84_anchored(const nav_suite_t* s) { return s->wgs84_anchor_seen; }

/* @satisfies REQ-SUITE-008 */
bool nav_suite_get_height_ellipsoid(const nav_suite_t* s, float* h_ell_m)
{
    if (s == NULL) return false;

    const bool ins_ready = ins_is_ready(&s->ins);
    const bool wgs84     = nav_suite_wgs84_anchored(s);

    nav_suite_local_ref_t ref;
    float                 offset, offset_sd;
    const bool            have_offset = nav_suite_local_ref(s, 0.0f, &ref) &&
                             local_gnss_alt_get(&s->local_gnss, &offset, &offset_sd);

    /* ins carries a true ellipsoid height only once real GNSS has
       anchored it (REQ-SUITE-008): indoor local-position aiding leaves
       the absolute solution on the prescribed init origin, which is not
       an ellipsoid height. */
    if (wgs84 && ins_ready && nav_suite_get_mode(s) == NAV_SUITE_MODE_FULL)
    {
        *h_ell_m = (float)s->ins.latlonh[2];
        /* ins's height is excellent RELATIVE to its own vertical datum. What
           it cannot know is where that datum has gone: under the barometric
           height source it was anchored once at bootstrap and is never
           re-tied to GNSS (REQ-NAV-055), so the barometer's whole absolute
           error - weather, ISA model - accumulates in it and reaches an
           absolute height undiminished. The offset filter has been watching
           exactly that, so take its movement since the datum was fixed back
           off. Only the MOVEMENT: replacing the height with the filter's own
           ref.h_m + offset would hand the caller that filter's noise on
           every epoch, including the great majority where the datum has not
           drifted at all and ins was already right. */
        if (s->datum_offset_valid && have_offset && s->ins.height_from_baro)
        {
            const float drift = offset - s->datum_offset_m;
            /* Believe only the part of the drift that stands out of the
               estimate's own noise (both ends of the difference contribute).
               Below that, report ins untouched rather than hand the caller a
               correction indistinguishable from noise. */
            const float sd_drift = SQRTF(offset_sd * offset_sd + s->datum_offset_var_min);
            const float margin   = NAV_SUITE_DATUM_DRIFT_SIGMA * sd_drift;
            if (drift > margin) { *h_ell_m += drift - margin; }
            else if (drift < -margin) { *h_ell_m += drift + margin; }
        }
        return true;
    }
    /* Local height reference plus the estimated ellipsoid height of the
       datum origin: the offset was estimated against this very source
       (nav_suite_local_ref), so the two belong together. The offset
       filter only ever runs with GNSS, so this branch already implies a
       WGS84 anchor. */
    if (have_offset)
    {
        *h_ell_m = ref.h_m + offset;
        return true;
    }
    if (wgs84 && ins_ready)
    {
        *h_ell_m = (float)s->ins.latlonh[2];
        return true;
    }
    return false;
}

/* @satisfies REQ-SUITE-013 */
bool nav_suite_get_local_pos_offset(const nav_suite_t* s, float offset_ned[3])
{
    if (s == NULL || offset_ned == NULL || !s->local_pos_offset_valid) { return false; }
    offset_ned[0] = s->local_pos_offset_ned[0];
    offset_ned[1] = s->local_pos_offset_ned[1];
    offset_ned[2] = s->local_pos_offset_ned[2];
    return true;
}

/* @satisfies REQ-SUITE-014 */
bool nav_suite_local_to_wgs84(const nav_suite_t* s, const float ned[3], double* lat_rad,
                              double* lon_rad, double* h_m)
{
    if (s == NULL || ned == NULL) { return false; }
    if (!s->ins.is_initialized || !nav_suite_wgs84_anchored(s)) { return false; }

    /* ECEF = origin_ecef + R_n_to_e * ned. The local offset is small
       (metres..km), so accumulating the float rotation product into the
       double origin loses nothing that matters. */
    double ecef[3];
    for (int i = 0; i < 3; ++i)
    {
        const float d = MAT_ELEM(s->ins.R_n_to_e, i, 0, 3, 3) * ned[0] +
                        MAT_ELEM(s->ins.R_n_to_e, i, 1, 3, 3) * ned[1] +
                        MAT_ELEM(s->ins.R_n_to_e, i, 2, 3, 3) * ned[2];
        ecef[i] = s->ins.origin_ecef[i] + (double)d;
    }
    ins_ecef_to_latlonh(ecef, lat_rad, lon_rad, h_m);
    return true;
}

/* @satisfies REQ-SUITE-014 */
bool nav_suite_wgs84_to_local(const nav_suite_t* s, double lat_rad, double lon_rad, double h_m,
                              float ned[3])
{
    if (s == NULL || ned == NULL) { return false; }
    if (!s->ins.is_initialized || !nav_suite_wgs84_anchored(s)) { return false; }

    double ecef[3];
    ins_latlonh_to_ecef(lat_rad, lon_rad, h_m, ecef);
    /* NED = R_n_to_e' * (ecef - origin_ecef). The difference is taken in
       double (nearby points), then rotated by the float DCM. */
    const double de[3] = {ecef[0] - s->ins.origin_ecef[0], ecef[1] - s->ins.origin_ecef[1],
                          ecef[2] - s->ins.origin_ecef[2]};
    for (int i = 0; i < 3; ++i)
    {
        ned[i] = (MAT_ELEM(s->ins.R_n_to_e, 0, i, 3, 3) * (float)de[0] +
                  MAT_ELEM(s->ins.R_n_to_e, 1, i, 3, 3) * (float)de[1] +
                  MAT_ELEM(s->ins.R_n_to_e, 2, i, 3, 3) * (float)de[2]);
    }
    return true;
}
