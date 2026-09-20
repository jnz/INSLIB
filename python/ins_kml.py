#!/usr/bin/env python3
"""Google Earth (KML) output for python/replay.py --kml: the ins
estimate as a time-animated gx:Track (drag Google Earth's time slider to
fly the replay) plus static ground-track LineStrings for the estimate, the
ground truth and the raw GNSS fix.

The recorded altitude is WGS84 ellipsoidal height (no geoid model
elsewhere in this project, see python/INSLIB/telemetry.py), while KML's
"absolute" altitudeMode expects height above the EGM96 geoid -- in
Germany that mismatch is around +45 m, enough to visibly float a
replay above the terrain if used uncorrected. If pyproj is installed
and can reach the network once (to fetch/cache PROJ's EGM96 grid), we
look up the local ellipsoidal-to-MSL offset a single time (it only
varies by centimeters per km, one sample is enough for any dataset
short of a long cross-country flight) and use real "absolute" altitude.
Without pyproj/network, we fall back to clampToGround/relativeToGround,
which ignores the altitude value entirely and glues the replay to
Google Earth's own terrain model instead -- fine for a ground track,
not a substitute for an actual vertical-accuracy visualization.

The animated track carries full attitude, not just heading: a small
COLLADA "dart" body (kml_assets/nav3d_dart.dae) is attached to the
gx:Track and driven by gx:angles (heading, tilt, roll) each sample, so
roll/pitch/yaw is visible as the dart banks and pitches while it flies
the route in Google Earth. Axis mapping:
  heading =  yaw                    (already 0-360 from true north)
  tilt    = -pitch, clamped to [-90, 90]  (pitch 0 = level = tilt 0,
            i.e. as authored -- NOT the <Camera>-style convention
            where 0 = nadir; a Model's tilt is a plain rotation about
            its local x axis with 0 = no rotation)
  roll    = -roll                   (already -180..180)
Visually cross-checked in Google Earth Pro: the previous "tilt = 90 -
pitch" (Camera-style) formula pointed the dart straight at the ground
during level driving, confirming Model tilt is not Camera tilt.
The negation on tilt/roll is because KML's <Orientation>/gx:angles
convention (see the KML reference's "Specifying <Orientation>
parameters" diagram) defines positive heading/tilt/roll as clockwise
when looking down the positive axis toward the origin, on a y=North,
x=East, z=Up frame -- the opposite handedness from this project's
right-hand-rule, NED/FRD roll and pitch. Heading needs no sign flip
only because yaw's clockwise-from-above sense already happens to
coincide with KML heading's clockwise-from-above sense.

Output is always a KMZ (a zip bundling the .kml plus the embedded
model), since a bare .kml cannot carry the model file with it; --kml's
path gets its extension corrected to .kmz if needed.

Built with simplekml (python/requirements-replay.txt) rather than
hand-rolled XML templates, so the KML stays well-formed as fields are
added.

Kept out of the INSLIB package proper (python/INSLIB/) since this is a
replay-tool convenience, not part of the reusable ctypes binding -- same
reasoning as ins_plots.py.

(c) Jan Zwiener (jan@zwiener.org)
"""

import datetime
import os

import simplekml

# KML <color> is aabbggrr (alpha, blue, green, red), not rrggbb.
_EST_COLOR = "ffffff00"   # opaque cyan
_REF_COLOR = "ff00ffff"   # opaque yellow
_FIX_COLOR = "ff0000ff"   # opaque red
_MODEL_TRACK_COLOR = "ff0080ff"  # opaque orange, distinct from the static
                                 # "ins estimate (path)" line's cyan
_EST_ICON_COLOR = "ffffff00"

_MODEL_PATH = os.path.join(os.path.dirname(__file__), "kml_assets", "nav3d_dart.dae")
_MODEL_SCALE = 15.0  # dart is a ~1-unit model; scale up to be visible in GE
# The dart's belly fin reaches ~0.2 * _MODEL_SCALE below the model origin,
# so the origin needs clearance from the ground to keep the fin from
# poking into the terrain (z-fighting). With real MSL altitude the origin
# is already close to true ground level, so only a small cosmetic nudge is
# needed; the clampToGround/relativeToGround fallback pins the origin
# exactly to GE's terrain, so it needs the fin's full depth cleared.
_MODEL_HOVER_ABSOLUTE_M = 1.0
_MODEL_HOVER_FALLBACK_M = 4.0


def _geoid_undulation_m(lat_deg, lon_deg):
    """Ellipsoidal-to-MSL offset N (m, WGS84 ellipsoidal = MSL + N) at one
    point, via PROJ's EGM96 grid (pyproj). N changes by only centimeters
    per km, so one lookup covers a whole dataset short of a long
    cross-country flight. Returns None (caller falls back to
    clampToGround) if pyproj is missing or the grid can't be fetched --
    PROJ_NETWORK fetches and caches the grid on first use, which needs
    internet access at least once."""
    try:
        import pyproj
        pyproj.network.set_network_enabled(True)
        t = pyproj.Transformer.from_crs("EPSG:4979", "EPSG:5773", always_xy=True)
        _, _, h_msl = t.transform(lon_deg, lat_deg, 0.0)
        return -h_msl
    except Exception as e:
        print(f"kml: no geoid correction available ({e}); "
              "falling back to clampToGround instead of real altitude")
        return None


def write_kml(path, est_track, ref_track, fix_track, name):
    """est_track: list of (t_rel_sec, lat_deg, lon_deg, alt_m, roll_deg,
    pitch_deg, yaw_deg) (t_rel_sec seconds since replay start -- only used
    to animate the gx:Track time slider, not a real calendar time).
    ref_track: list of (lat_deg, lon_deg, alt_m), or empty if no reference
    is available.
    fix_track: list of (t_rel_sec, lat_deg, lon_deg, alt_m, cov_nn, cov_ne,
    cov_ee) raw GNSS fixes (pre-fusion, pre-NHC; the trailing North/East
    position covariance in m^2 is for --map-frames' error ellipses, unused
    here), held flat while no fix arrives (e.g. a GNSS outage) so an
    outage shows as a frozen point followed by a jump, not a smooth
    interpolation -- or empty if no GNSS fix stream was recorded."""
    if not est_track:
        print("kml: no recorded samples (filter never initialized?)")
        return

    if not path.lower().endswith(".kmz"):
        path = os.path.splitext(path)[0] + ".kmz"

    t0 = datetime.datetime(2000, 1, 1, tzinfo=datetime.timezone.utc)  # arbitrary epoch

    _, lat0, lon0, _, _, _, _ = est_track[0]
    N = _geoid_undulation_m(lat0, lon0)
    line_mode = (simplekml.AltitudeMode.absolute if N is not None
                else simplekml.AltitudeMode.clamptoground)
    model_mode = (simplekml.AltitudeMode.absolute if N is not None
                 else simplekml.AltitudeMode.relativetoground)
    # MSL height plus the model's own ground-clearance hover; the lines have
    # no geometry below their coordinate so they only need the MSL part.
    line_alt = (lambda a: a - N) if N is not None else (lambda a: a)
    model_alt = ((lambda a: a - N + _MODEL_HOVER_ABSOLUTE_M) if N is not None
                else (lambda a: _MODEL_HOVER_FALLBACK_M))

    kml = simplekml.Kml(name=name)

    est_line = kml.newlinestring(
        name="ins estimate (path)",
        coords=[(lon, lat, line_alt(alt)) for _, lat, lon, alt, _, _, _ in est_track])
    est_line.altitudemode = line_mode
    est_line.extrude = 1 if N is not None else 0
    est_line.tessellate = 1
    est_line.linestyle.color = _EST_COLOR
    est_line.linestyle.width = 3

    if ref_track:
        ref_line = kml.newlinestring(
            name="ground truth (path)",
            coords=[(lon, lat, line_alt(alt)) for lat, lon, alt in ref_track])
        ref_line.altitudemode = line_mode
        ref_line.extrude = 1 if N is not None else 0
        ref_line.tessellate = 1
        ref_line.linestyle.color = _REF_COLOR
        ref_line.linestyle.width = 2

    if fix_track:
        fix_line = kml.newlinestring(
            name="raw GNSS fix (path)",
            coords=[(lon, lat, line_alt(alt))
                   for _, lat, lon, alt, *_ in fix_track])
        fix_line.altitudemode = line_mode
        fix_line.extrude = 1 if N is not None else 0
        fix_line.tessellate = 1
        fix_line.linestyle.color = _FIX_COLOR
        fix_line.linestyle.width = 2

    track = kml.newgxtrack(name="INSLIB (animated)")
    track.visibility = 0  # off by default, static "path" lines are the overview
    track.altitudemode = model_mode
    track.linestyle.color = _MODEL_TRACK_COLOR
    track.linestyle.width = 2
    # No separate point icon: the 3D model below is the only marker, the
    # default gx:Track pushpin icon would just duplicate it.
    track.iconstyle.icon.href = ""
    track.iconstyle.scale = 0

    model_href = kml.addfile(_MODEL_PATH)
    track.model.link.href = model_href
    track.model.altitudemode = model_mode
    track.model.scale.x = _MODEL_SCALE
    track.model.scale.y = _MODEL_SCALE
    track.model.scale.z = _MODEL_SCALE

    for t_rel, lat, lon, alt, roll, pitch, yaw in est_track:
        ts = t0 + datetime.timedelta(seconds=t_rel)
        tilt = max(-90.0, min(90.0, -pitch))
        # Microsecond precision: at typical --kml-hz sampling two
        # consecutive samples can otherwise round to the same whole
        # second, giving gx:Track a non-increasing <when> and a stutter.
        track.newwhen(ts.strftime("%Y-%m-%dT%H:%M:%S.%fZ"))
        track.newgxcoord([(lon, lat, model_alt(alt))])
        track.newgxangle([(yaw, tilt, -roll)])

    kml.savekmz(path)
    print(f"wrote {path} ({len(est_track)} estimate points, "
          f"{len(ref_track)} reference points, "
          f"{len(fix_track)} raw GNSS fix points)")
