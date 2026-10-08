# Changelog

## [Unreleased]

## [1.2.1] - 2026-10-08 - Bug fixes

### Added

- `allan_variance.py`: bias instability added, PDF output redesigned.
- `inspostgui`: "Lever arm compensated" checkbox, draws the reference and the
  GNSS fixes at the IMU point in the view ports.
- `replay.py`/`inspostgui`: `ref.csv` is now optional.
- `replay.py` PDF: the WMM page also shows |B| minus the 18-state hard-iron
  bias estimate.
- ARS/AHRS: `ahrs_config_t.rpy_pred_stddev_rad_sqrts` (config.yaml
  `ahrs: rpy_pred_stddev_rad_sqrts`), an extra attitude process noise on top
  of the gyro noise, the same is already in INS.
  Off by default, so a typical IMU behaves as before.
- `inspostgui`: ranging anchors are shown in the 3D view (dots with their id).

### Changed

- Python `Config` no longer has defaults of its own: every tuning field is 0,
  i.e. the C library's default.
- `nav_suite`: the ARS/AHRS bootstrap levels over the ins auto-init window
  instead of a single sample and widens roll/pitch if the platform moves.

### Fixed

- `replay.py`/`inspostgui`: the ARS/AHRS gyro noise and bias random walk now
  fall back to `imu: gyr_psd`/`gyr_bias_rw` when `ahrs:` leaves them at 0, as
  `replay.c` already did.
- IMU loss: a gap of 0.2 s or more in the IMU stream (`imu_loss_timeout_sec`)
  now stops INS, ARS, AHRS and baro_alt.
- INS: the magnetometer hard-iron bias (if estimated) is carried across a
  re-arm like the IMU biases.
- `inspostgui`: the mouse wheel no longer changes combo boxes in the config tab
  while scrolling the form.

## [1.2.0] - 2026-10-04 - GUI Tool

### Added

- Windows executable for: `inspostgui.exe`. Easier processing of `.csv` files.
- `inspostgui` can highlight ZUPT/ZARU epochs in trajectory view.
- `inspostgui`: "Last run" panel (position and attitude error, change against
  the previous run, insdoctor findings), a legend for the 3D view, keyboard
  shortcuts (F5 run, Space pause, Esc stop, Ctrl+O/S, Ctrl+1..5 tabs).
- `inspostgui`: adjustable speed for the red end of the 3D trail colours
  ("Red at", auto or a fixed value in m/s).

### Changed

- All executables now live in `tools/`: `replay.py`, `inspostgui.py`,
  and `allan_variance.py` moved over from `python/`,
  which keeps only the `INSLIB` package.
- Renamed `f9p_config.py` and `x20p_config.py` to `ublox_f9p_config.py` and
  `ublox_x20p_config.py`.

### Fixed

- Potential height jump when the 3D solution restarts after a GNSS outage with
  a barometer fixed.
- Fixed a bunch of parameters in different `config.yaml` files in the `datasets`
  directory.

### Removed

- The reference board's host tools (serial hub, control GUI, configuration,
  calibration window, capture converter) are no longer part of this
  repository. The protocol stays documented in `tools/inslib_protocol.md`.

## [1.1.1] - 2026-09-28 - Smaller fixes

### Added

- Dual-antenna GNSS heading post-processing: new `heading.csv` file format.
- `config.yaml` option `imu: mount_rpy_deg`: the sensor board's attitude in the
  body frame, corrects at once accelerometer, gyroscope and magnetometer
  attitude (without the need to set individual misalignment matrices).
- Range aiding to known anchors (no full tight coupling yet though).

### Changed

- `inspostgui.py` several small tweaks and bug fixes to improve quality of life.
- Post-processing: `score: leverarm_frd` fixed: a reference taken at the GNSS
  antenna no longer shows the lever arm as a height or position offset.
- Post-processing PDF output: The North-East map marks the start and end of the
  estimated track.

### Fixed

- `nav_suite`: the local-height/GNSS offset filter was fed the antenna's
  ellipsoid height against the IMU's local height, so it absorbed the
  vertical GNSS lever arm.
- `ins` auto-init bootstrapped the IMU at the GNSS antenna's position. It
  now starts at the fix minus the lever arm rotated with the bootstrap
  attitude (only its vertical part while the yaw is unknown).
- `ins` re-acquisition after an expired coasting window rotated the lever
  arm with the attitude frozen at the start of the outage.

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

