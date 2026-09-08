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

#ifdef __cplusplus
}
#endif

#endif /* MAGNETIC_MODEL_H */
/** @} */
