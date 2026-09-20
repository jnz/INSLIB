/** @file magnetic_model.h
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * @brief World Magnetic Model (WMM) lookup: declination, inclination,
 *  field strength and the full NED reference field for a location/epoch.
 *
 * A compact interpolated stand-in for the full WMM
 * spherical-harmonics evaluation. The coefficients are stored in a
 * generated look-up table (wmm_lut.h, produced by generate_wmm_grid.py).
 * Declination is stored on a fine grid and interpolated bilinearly in space
 * and linearly in time between the two model epochs. Inclination and total
 * field strength are stored on a coarser grid and interpolated bilinearly in
 * space (the field strength changes slowly enough over a 5-year epoch that the
 * temporal term is omitted).
 *
 * Portable, heap-free, bounded execution time.
 */

/** @addtogroup magnetic_model
 *  @{ */

#ifndef MAGNETIC_MODEL_H
#define MAGNETIC_MODEL_H

#include <stdbool.h>

/** @brief Great-circle radius around a dip pole where the magnetic heading
 *  reference is rejected [deg]. Inside it the horizontal field is a few
 *  percent of the total, so both the sensor and the grid lose the heading
 *  information.
 *
 *  Carries ~2 deg of slack over the region that is strictly unusable, which
 *  absorbs the drift of the tabulated mid-epoch pole positions (under 1 deg
 *  towards either end of an epoch) and keeps the residual declination error
 *  outside the zone well clear of the steep flank at the boundary. */
#define MAGNETIC_DIP_POLE_EXCLUSION_DEG (7.0f)

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Magnetic declination (angle from true to magnetic north).
     *
     *  @param[in] lat_deg Latitude [deg] (-90..+90, clamped).
     *  @param[in] lon_deg Longitude [deg] (wrapped into -180..+180).
     *  @param[in] year Decimal year (e.g. 2027.5), extrapolated outside the
     *                  model epoch.
     *  @return Declination [deg], positive = East (add to a magnetic heading
     *          to obtain a true-north heading). */
    float magnetic_declination_deg(float lat_deg, float lon_deg, float year);

    /** @brief Magnetic inclination (dip) at the location.
     *  @param[in] lat_deg Latitude [deg] (-90..+90, clamped).
     *  @param[in] lon_deg Longitude [deg] (wrapped into -180..+180).
     *  @return Inclination [deg], positive = field points down (N hemisphere).
     */
    float magnetic_inclination_deg(float lat_deg, float lon_deg);

    /** @brief Total magnetic field strength at the location.
     *  @param[in] lat_deg Latitude [deg] (-90..+90, clamped).
     *  @param[in] lon_deg Longitude [deg] (wrapped into -180..+180).
     *  @return Field strength [uT]. */
    float magnetic_field_strength_uT(float lat_deg, float lon_deg);

    /** @brief Full magnetic reference field in the NED frame.
     *
     *  Built from declination D, inclination I and total field F as
     *  B = F * [cosI cosD, cosI sinD, sinI].
     *
     *  @param[in] lat_deg Latitude [deg].
     *  @param[in] lon_deg Longitude [deg].
     *  @param[in] year Decimal year.
     *  @param[out] b_ned_uT Reference field [uT], NED. */
    void magnetic_field_ned_uT(float lat_deg, float lon_deg, float year, float b_ned_uT[3]);

    /** @brief Great-circle distance to the nearest magnetic dip pole.
     *
     *  The positions come from the generated look-up table, tabulated at
     *  mid-epoch. Takes no year for the same reason the inclination and field
     *  strength queries do not: the drift over an epoch is smaller than the
     *  slack in MAGNETIC_DIP_POLE_EXCLUSION_DEG.
     *
     *  @param[in] lat_deg Latitude [deg].
     *  @param[in] lon_deg Longitude [deg].
     *  @return Distance [deg] in the range 0..180. */
    float magnetic_dip_pole_distance_deg(float lat_deg, float lon_deg);

    /** @brief Whether the model yields a usable magnetic heading reference.
     *
     *  False inside MAGNETIC_DIP_POLE_EXCLUSION_DEG of a dip pole. There the
     *  horizontal field vanishes, so the declination is ill-conditioned (the
     *  grid cannot resolve it and the errors reach tens of degrees) and a
     *  magnetometer carries almost no heading information either. Callers
     *  should suppress magnetic heading aiding while this is false, rather
     *  than trusting magnetic_declination_deg() there.
     *
     *  Inclination and total field strength stay usable inside the zone, so
     *  this does not invalidate the other queries.
     *
     *  @param[in] lat_deg Latitude [deg].
     *  @param[in] lon_deg Longitude [deg].
     *  @return true if the location is outside every exclusion zone. */
    bool magnetic_heading_reference_valid(float lat_deg, float lon_deg);

#ifdef __cplusplus
}
#endif

#endif /* MAGNETIC_MODEL_H */
/** @} */
