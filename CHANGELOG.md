# Changelog

INSLIB changes over time. Using:

- [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
- [Semantic Versioning](https://semver.org/).

## [1.1.0] - 2026-09-13 - Automotive Update

### Added

- Non-holonomic lateral velocity constraint for automotive mode during GNSS
  outages: cars typically don't move sideways at a high velocity, so this
  constraint bounds attitude/velocity drift while GNSS is unavailable.
  Example dataset: `datasets/tunnel_nhc`.
- Magnetic model safety: dip pole exclusion zones added to the World Magnetic
  Model lookup, since declination/inclination become unreliable close to the
  dip poles.
- `make stack`: static worst-case stack usage analysis added.
- `tools/inslib_speed_scale.py`: calibrate odometry speed from OBD2 dongles
  against GNSS speed.
- KML export: raw GNSS fix track added, MSL altitude (with geoid undulation),
  and a 3D attitude model for Google Earth playback.
- OpenStreetMap view for `inspostgui.py` added.
- `tools/inslib_imu_calib.py` (renamed from `inslib_ubx_imu_calib.py`):
  command line IMU/magnetometer calibration from `.csv` recordings of any
  IMU (`--csv`), no sensor board needed.
- Replay `config.yaml`: `mag.bias_init_ut` and `mag.bias_rw_ut_sqrts` expose
  the two hard-iron tuning knobs of the 18-state mode, so a dataset can state
  how much hard iron the filter should expect and how long the estimate keeps
  listening. Both default to the library values when absent.

### Performance compared to v1.0.0

- ~12% fewer instructions in the time critical Kalman update path.
- More than 30% reduction in worst-case stack memory usage.
- GNSS processing costs about half of what it did on a ARM Cortex-M4F MCU.

### Changed

- **API break:** `x_ecef`/`xdot_ecef` in `ins_init_t`
  are replaced by `llh` (latitude and longitude in rad,
  height over the WGS84 ellipsoid) and `vel_ned`.
- **API break:** `xyz_ecef` in `ins_meas_gnss_pos_t` is replaced by `llh`, the
  form receivers report and the filter fuses in. An ECEF-native source can
  convert with `ins_ecef_to_latlonh()`. In the Python binding,
  `Navigator.gnss_pos()` becomes `gnss_pos_llh()`.
- Post-Processing: Warn if magnetometer `.csv` data is present in a dataset but
  no WMM year is set in `config.yaml`.

### Security

- Added `SECURITY.md` (vulnerability disclosure policy) and `sbom.cdx.json`
  (CycloneDX software bill of materials) for EU Cyber Resilience Act (CRA).

### Fixed

- WMM: fixed a +/-180 degree declination wraparound bug in the interpolation.
- Fixed a YAML parser bug affecting vector definitions.
- `inslib_convert_ubx_to_csv.py`: fixed OBD2 speed odometry data export and
  post-processing.
- GNSS state machine exit criteria: coasting time is no longer counted
  against the bad-fix-time threshold.
- `inslib_decimate_dataset.py`: sampling rate is now derived correctly when
  the recording has gaps, instead of just total time span / sample count.
- `inslib_convert_ubx_to_csv.py`: no longer hardcodes a generic MEMS baro
  standard deviation guess, uses the library default instead.

## [1.0.0] - 2026-09-09

- Initial public release.

