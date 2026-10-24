# Changelog

## [1.4.1] - 2026-10-24 - Tweaks

### Changed

- Lower worst-case epoch time: the Earth parameter refresh every 100 m of
  travel (curvature terms, gravity, magnetic model) no longer runs in an
  epoch with covariance prediction or a Kalman update. It waits for a free
  epoch (forced after 1000 m).
- IMU samples that are finite but physically impossible (a unit mix-up, a
  corrupt frame) are now dropped like non-finite ones and counted in
  `n_invalid_input`, by INS, AHRS/ARS, `baro_alt` and `nav_suite` alike
  (bounds in `sensor_defaults.h`). One such sample used to rotate the
  attitude by an arbitrary angle.
- Static/stillness checks compare squared norms and no longer take a
  square root per epoch.

### Fixed

- `nav_suite`: the magnetometer AHRS kept the World Magnetic Model at the
  position it was first given. It now follows the INS's own refresh as the
  vehicle travels, so one `nav_suite_set_magnetic_model_position()` call
  is enough.
- AHRS/ARS: a non-finite gyro bias estimate now shuts the filter down
  instead of corrupting the next epochs, and is never carried over into a
  restart.

- A gravity vector set at runtime with `ins_set_world_model()` was
  replaced by the computed normal gravity again after at most 100 m of
  travel, if the filter had been started with automatic gravity
  (`init.gravity_n` zero). The refresh now leaves vectors set there alone.

## [1.4.0] - 2026-10-18 - PyPI Package (pip install inslib)

### Added

- The Python binding is published on PyPI: `pip install inslib` installs a
  prebuilt wheel (Linux x86_64/aarch64/armv7l,32-bit Raspberry Pi OS,
  Windows x64/ARM64, macOS Intel/Apple silicon).
- API reference of the Python package.
- Python tutorial rewritten around `pip install inslib`: a
  minimal roll/pitch example (IMU only, ARS filter), GNSS, a barometer, a
  compass (magnetometer), the per-epoch loop.
- `nav_suite_set_magnetic_model_position()`: one call that sets the World
  Magnetic Model position for the INS and the magnetometer AHRS.
- More GNSS test scenarios added.
  (`make datasets-kfgins-faults`, part of `make test`): spikes, a wrong RTK
  fix, multipath, honest degradation, velocity spikes, lasting steps and
  500 m outliers, with RTK and with L1 single point accuracy, written
  reproducibly by the new `tools/gnss_faults.py`.

### Fixed

- Python binding: `Navigator()` and `Ins()` without a `Config` left the
  filter uninitialized, so every epoch was treated as an IMU loss and no
  solution ever came. Without an argument they now start from `Config()`.
- The magnetometer AHRS ignored the World Magnetic Model position whenever
  it arrived before the AHRS had started (for example Python's
  `set_magnetic_model()` right after `Navigator()`), and lost it on every
  restart, so its heading stayed on magnetic north, off by the local
  declination. The suite now keeps the position and applies it on every
  AHRS start.

## [1.3.0] - 2026-10-14 - RTS Smoother

### Added

- RTS (Rauch, Tung, Striebel) smoother added to Python post-processing.
  This will perform a forward and backwards processing of the data for
  better results (no change to the C code required for this feature).
- GUI quality of life improvements (e.g. better colors, highlight
  variables that have changed from the defaults, grid size)
- More ARS/AHRS tuning variables for `config.yaml`: `acc_freq_hz`,
  `acc_cutoff_freq_hz`, `gravity_diff_penalty`, `acc_reject_gravity_mps2`,
  `chi2_threshold`, `kalman_update_dt_sec`, `mag_yaw_stddev_deg`, `mag_freq_hz`
  and `mag_chi2_threshold`.
- `replay.py`/`inspostgui` PDF: attitude comparison page (INS, ARS and AHRS
  error against the reference attitude, with RMS, mean and maximum) and the
  same figures on the front page.
- Improved measurement delay compensation: Delayed measurement are now
  interpolated to the time of validity between the two surrounding history
  entries instead of just the nearest history entry.
- INS: top-level `config.yaml` key `kalman_update_dt_sec`, the covariance
  prediction interval of the INS can be tuned now.
- `allan_variance.py`: improvements, e.g. bias instability marked in the plot,
  IMU temperature curve added, PDF output tweaks.

### Changed (allan_variance.py)

- The first 30 s and the last 60 s of the recording are now skipped by
  default (`--skip-start 0 --skip-end 0` restores the old behaviour).

### Changed

- Improved PDF output with more plots and statistics (e.g. for RTS).
- Logging: the library-wide default `LOG_LEVEL` is now `LOG_LEVEL_WARN`
  (was INFO), so INFO messages are compiled out unless a build defines
  `-DLOG_LEVEL=LOG_LEVEL_INFO`.
- u-blox F9P/X20 configuration script: change to GPS time base instead
  of UTC.
- `nav_suite`: Under GNSS alone (no baro, no local position) the baro/GNSS
  offset filter now stays idle instead of re-estimating the datum's
  ellipsoid height (with a meaningless sigma). inspostgui shows N/A there.

### Fixed

- For GNSS receivers without velocity measurements (a bit of an edge case) and
  INS re-acquisition after a GNSS loss: The estimated velocity at the start of
  the outage was used at the re-acquisition.  This old velocity is outdated
  at the time of the re-acquisition and caused issues. Fixed (also for local
  position fixes).
- `nav_suite`: the ARS-tracked heading carry-over now also initializes an INS
  re-acquisition after an expired coasting window, not only a re-init.
  The ARS gyro heading can now support the full 3D INS re-init.
- Post-Processing Bug: `replay.c` (`init: ref`): with a GNSS delay the
  prescribed start state was taken at the first IMU sample instead of the later
  first fix where INS actually starts, offsetting a moving start.
- Auto-ZUPT/ZARU bug: the GNSS velocity test missed fixes on epochs the IMU
  already read as moving, so it could trigger during steady turns or climbs.
- `ins_auto_zupt_active()` reported stillness before the detector's dwell
  time had passed (REQ-NAV-013), so the ARS/AHRS received zero-rotation
  updates during short slow-speed phases the INS itself did not act on.
  It now applies the dwell time like the detector.

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
  The default (0 selects it) is sized so that, together with the new, smaller
  default gyro noise density, a caller leaving both at 0 gets the same total
  attitude noise as before. A caller who sets the gyro noise explicitly now
  gets the extra term on top, set a small positive value to avoid it.
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

