#!/usr/bin/env python3
"""Google Earth (KML) output for python/replay.py --kml: the ins
estimate as a time-animated gx:Track (drag Google Earth's time slider to
fly the replay) plus static ground-track LineStrings for the estimate and
the ground truth, both draped from the actual altitude down to the
ground (extrude) so climbs/descents are visible, not just the 2D path.

The animated track carries full attitude, not just heading: a small
COLLADA "dart" body (kml_assets/nav3d_dart.dae) is attached to the
gx:Track and driven by gx:angles (heading, tilt, roll) each sample, so
roll/pitch/yaw is visible as the dart banks and pitches while it flies
the route in Google Earth. Axis mapping, since gx:angles' "tilt" follows
the same convention as <Camera>'s tilt (0 = pointing straight down, 180
= pointing straight up, 90 = level with the horizon):
  heading = yaw                     (already 0-360 from true north)
  tilt    = 90 - pitch, clamped to [0, 180]  (pitch 0 = level = tilt 90)
  roll    = roll                    (already -180..180)
This has not been visually cross-checked against a real Google Earth
render; if the dart's nose/belly look wrong (e.g. upside down, or roll
inverted), that pins down which sign/offset above needs flipping -- the
dart's shape is deliberately asymmetric (pointed nose, flat top, one
belly fin) so a wrong mapping is obvious at a glance.

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
_EST_ICON_COLOR = "ffffff00"

_MODEL_PATH = os.path.join(os.path.dirname(__file__), "kml_assets", "nav3d_dart.dae")
_MODEL_SCALE = 15.0  # dart is a ~1-unit model; scale up to be visible in GE


def write_kml(path, est_track, ref_track, name):
    """est_track: list of (t_rel_sec, lat_deg, lon_deg, alt_m, roll_deg,
    pitch_deg, yaw_deg) (t_rel_sec seconds since replay start -- only used
    to animate the gx:Track time slider, not a real calendar time).
    ref_track: list of (lat_deg, lon_deg, alt_m), or empty if no reference
    is available."""
    if not est_track:
        print("kml: no recorded samples (filter never initialized?)")
        return

    if not path.lower().endswith(".kmz"):
        path = os.path.splitext(path)[0] + ".kmz"

    t0 = datetime.datetime(2000, 1, 1, tzinfo=datetime.timezone.utc)  # arbitrary epoch

    kml = simplekml.Kml(name=name)

    est_line = kml.newlinestring(
        name="ins estimate (path)",
        coords=[(lon, lat, alt) for _, lat, lon, alt, _, _, _ in est_track])
    est_line.altitudemode = simplekml.AltitudeMode.absolute
    est_line.extrude = 1
    est_line.tessellate = 1
    est_line.linestyle.color = _EST_COLOR
    est_line.linestyle.width = 3

    if ref_track:
        ref_line = kml.newlinestring(
            name="ground truth (path)",
            coords=[(lon, lat, alt) for lat, lon, alt in ref_track])
        ref_line.altitudemode = simplekml.AltitudeMode.absolute
        ref_line.tessellate = 1
        ref_line.linestyle.color = _REF_COLOR
        ref_line.linestyle.width = 2

    track = kml.newgxtrack(name="ins estimate (animated)")
    track.altitudemode = simplekml.AltitudeMode.absolute
    track.linestyle.color = _EST_COLOR
    track.linestyle.width = 2
    track.iconstyle.color = _EST_ICON_COLOR
    track.iconstyle.scale = 1.0

    model_href = kml.addfile(_MODEL_PATH)
    track.model.link.href = model_href
    track.model.altitudemode = simplekml.AltitudeMode.absolute
    track.model.scale.x = _MODEL_SCALE
    track.model.scale.y = _MODEL_SCALE
    track.model.scale.z = _MODEL_SCALE

    for t_rel, lat, lon, alt, roll, pitch, yaw in est_track:
        ts = t0 + datetime.timedelta(seconds=t_rel)
        tilt = max(0.0, min(180.0, 90.0 - pitch))
        track.newwhen(ts.strftime("%Y-%m-%dT%H:%M:%SZ"))
        track.newgxcoord([(lon, lat, alt)])
        track.newgxangle([(yaw, tilt, roll)])

    kml.savekmz(path)
    print(f"wrote {path} ({len(est_track)} estimate points, "
          f"{len(ref_track)} reference points)")
