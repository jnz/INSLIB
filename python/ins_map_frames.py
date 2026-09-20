#!/usr/bin/env python3
"""PNG frame sequence for python/replay.py --map-frames: a portrait
(1080x1920) top-down OpenStreetMap view with the raw GNSS fix track (red)
and the INSLIB estimate (cyan) progressively revealed over time, plus a
gray band marking the real tunnel/underpass geometry pulled from OSM. Meant
to be stitched into a short video clip with ffmpeg (the exact command is
printed at the end) for the stretch of a drive that a Google Earth flyover
cannot show (Google Earth has no hollow tunnel interior -- flying "through"
one just enters solid terrain).

Two data sources beyond the recorded track:
  - OpenStreetMap raster tiles for the background (tile.openstreetmap.org,
    same disk cache and usage-policy-respecting fetch as tools/inslib_gui.py
    -- identifying User-Agent, on-screen budget in spirit even though this
    is a batch tool, cache-first).
  - The tunnel/underpass geometry itself via the Overpass API: every OSM
    way tagged tunnel=yes with a highway tag inside the track's bounding
    box, stitched into one polyline and drawn as a thick translucent band
    (not a precise buffered polygon -- a thick line is visually the same
    for a road-width band and needs no extra geometry library). Cached to
    disk next to the tile cache so a rerun does not hit Overpass again.

Both network dependencies degrade gracefully: no tiles -> plain background;
no tunnel geometry -> no band, just the two tracks.

Kept out of the INSLIB package proper (python/INSLIB/) for the same reason
as ins_kml.py/ins_plots.py: a replay-tool convenience, not part of the
reusable ctypes binding.

(c) Jan Zwiener (jan@zwiener.org)
"""

import io
import json
import math
import os
import urllib.parse
import urllib.request

EARTH_R = 6378137.0  # Web Mercator sphere radius (metres)


def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres -- only used to decide "is this OSM
    way actually near the driven route", so the sphere approximation is
    plenty."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2.0) ** 2)
    return 2.0 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))

# Same cache root and tile-policy-respecting fetch as tools/inslib_gui.py's
# TileFetcher: fetched tiles are shared between the interactive GUI and
# this batch tool instead of being downloaded twice.
_CACHE_ROOT = os.path.join(
    os.environ.get("LOCALAPPDATA") or os.path.expanduser("~/.cache"),
    "inslib_gui")
_TILE_URL = "https://tile.openstreetmap.org/%d/%d/%d.png"
_TILE_AGENT = "inslib_gui/1.0 (INSLIB device console; local diagnostics)"
_TILE_CACHE = os.path.join(_CACHE_ROOT, "tiles")
_OVERPASS_CACHE = os.path.join(_CACHE_ROOT, "overpass")
_OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]

_TUNNEL_COLOR = (0.35, 0.35, 0.35, 0.55)
_EST_COLOR = "#ff7f00"
_FIX_COLOR = "#0000ff"     # red, matches ins_kml.py's fix color family
_ELLIPSE_VISUAL_SCALE = 3.0  # exaggeration for video legibility -- a good
                             # (HAS) fix's true ellipse is a few cm, a few
                             # sub-pixel dots either way, so this only
                             # matters (and reads as intended) right where
                             # it is actually large


def _tile_xy(lat_deg, lon_deg, zoom):
    n = 2 ** zoom
    x = (lon_deg + 180.0) / 360.0 * n
    lat_rad = math.radians(lat_deg)
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad))
        / math.pi) / 2.0 * n
    return x, y


def _meters_per_pixel(lat_deg, zoom):
    """Web Mercator is conformal: local scale is the same in every
    direction at a given point, so one scalar covers both North and East
    error-ellipse axes -- it just depends on latitude and zoom."""
    return (2.0 * math.pi * EARTH_R * math.cos(math.radians(lat_deg))
           / (256.0 * 2 ** zoom))


def _error_ellipse_px(cov_nn, cov_ne, cov_ee, lat_deg, zoom):
    """1-sigma North/East position-covariance ellipse (matplotlib.patches.
    Ellipse width, height, angle_deg), in the pixel space _tile_xy/px use.
    None if the covariance is missing (NaN) or degenerate.

    Map "up" is North and "right" is East (standard north-up OSM tiles),
    and pixel y grows downward, so a real-world (dN, dE) direction is a
    pixel-space (dE, -dN) direction; the eigenvector-to-angle conversion
    below has not been cross-checked against a real anisotropic fix (every
    sample in datasets/ so far reports an isotropic circle), the same
    caveat ins_kml.py's dart orientation carried before it was checked."""
    if not (cov_nn == cov_nn and cov_ee == cov_ee and cov_ne == cov_ne):
        return None
    import numpy as np
    eigvals, eigvecs = np.linalg.eigh([[cov_nn, cov_ne], [cov_ne, cov_ee]])
    eigvals = np.clip(eigvals, 0.0, None)
    mpp = _meters_per_pixel(lat_deg, zoom)
    major_m, minor_m = math.sqrt(eigvals[1]), math.sqrt(eigvals[0])
    v_n, v_e = eigvecs[0, 1], eigvecs[1, 1]     # major-axis eigenvector
    angle_deg = math.degrees(math.atan2(-v_n, v_e))
    return 2.0 * major_m / mpp, 2.0 * minor_m / mpp, angle_deg


def _fetch_tile(z, x, y):
    """One 256x256 PNG tile, disk-cached, or None on any failure (offline,
    rate-limited, ...) -- the caller draws a plain background instead."""
    n = 2 ** z
    x, y = x % n, y % n
    path = os.path.join(_TILE_CACHE, str(z), str(x), "%d.png" % y)
    try:
        if os.path.exists(path):
            with open(path, "rb") as fh:
                return fh.read()
        req = urllib.request.Request(_TILE_URL % (z, x, y),
                                     headers={"User-Agent": _TILE_AGENT})
        with urllib.request.urlopen(req, timeout=8) as r:
            data = r.read()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            fh.write(data)
        return data
    except Exception as e:
        print(f"map-frames: tile {z}/{x}/{y} unavailable ({e})")
        return None


def _basemap(lat_min, lat_max, lon_min, lon_max, zoom, margin_frac=0.15,
            aspect=9.0 / 16.0):
    """Stitches the tiles covering [lat/lon]_min/max (plus a margin) into
    one RGB array, plus the pixel<->lat/lon mapping (a linear map in tile
    space: _tile_xy is affine in lon and near-affine in lat over the small
    span one dataset covers). Returns (image_array, to_px) where to_px(lat,
    lon) -> (px, py) in the stitched image. None, None if no tile loaded.

    The stitched canvas is then cropped (centered) to exactly the given
    width/height aspect in whole pixels, so the final array is pixel-exact
    portrait with no letterboxing -- padding the fetch box up front only
    gets the tile *count* close, integer tile rounding still leaves a
    fractional-tile mismatch that a plain imshow would otherwise show as a
    gap on one side (matplotlib's own aspect-fitting centers the box, but
    does not repaint the plot area with more image, so a mismatch surfaces
    as blank margin instead)."""
    import numpy as np
    from PIL import Image

    dlat, dlon = lat_max - lat_min, lon_max - lon_min
    lat_min -= dlat * margin_frac
    lat_max += dlat * margin_frac
    lon_min -= dlon * margin_frac
    lon_max += dlon * margin_frac

    x0f, y1f = _tile_xy(lat_min, lon_min, zoom)
    x1f, y0f = _tile_xy(lat_max, lon_max, zoom)
    x0f, x1f = min(x0f, x1f), max(x0f, x1f)
    y0f, y1f = min(y0f, y1f), max(y0f, y1f)

    w, h = x1f - x0f, y1f - y0f
    if w / h < aspect:
        new_w, cx = h * aspect, (x0f + x1f) / 2.0
        x0f, x1f = cx - new_w / 2.0, cx + new_w / 2.0
    else:
        new_h, cy = w / aspect, (y0f + y1f) / 2.0
        y0f, y1f = cy - new_h / 2.0, cy + new_h / 2.0

    x0, x1 = int(math.floor(x0f)), int(math.ceil(x1f))
    y0, y1 = int(math.floor(y0f)), int(math.ceil(y1f))

    canvas = Image.new("RGB", ((x1 - x0) * 256, (y1 - y0) * 256), (235, 235, 230))
    any_tile = False
    for ty in range(y0, y1):
        for tx in range(x0, x1):
            data = _fetch_tile(zoom, tx, ty)
            if data is None:
                continue
            any_tile = True
            tile = Image.open(io.BytesIO(data)).convert("RGB")
            canvas.paste(tile, ((tx - x0) * 256, (ty - y0) * 256))
    if not any_tile:
        return None, None

    w_px, h_px = canvas.size
    if w_px / h_px > aspect:
        new_w_px = int(round(h_px * aspect))
        crop_x, crop_y = (w_px - new_w_px) // 2, 0
        canvas = canvas.crop((crop_x, 0, crop_x + new_w_px, h_px))
    else:
        new_h_px = int(round(w_px / aspect))
        crop_x, crop_y = 0, (h_px - new_h_px) // 2
        canvas = canvas.crop((0, crop_y, w_px, crop_y + new_h_px))

    def to_px(lat_deg, lon_deg):
        tx, ty = _tile_xy(lat_deg, lon_deg, zoom)
        return (tx - x0) * 256 - crop_x, (ty - y0) * 256 - crop_y

    return np.asarray(canvas), to_px


_OVERPASS_GRID_DEG = 0.02  # ~2 km: snap the query bbox to this grid so a
                           # --map-t-start/--map-t-end window a bit
                           # different from a previous run still lands on
                           # the same cache entry, and Overpass sees one
                           # slightly larger query instead of many
                           # near-duplicate ones.


def _fetch_tunnel_geometry(lat_min, lat_max, lon_min, lon_max):
    """Highway ways tagged tunnel=yes in the bbox, as a list of polylines
    (each a list of (lat, lon)) -- a tunnel with multiple OSM ways (bores,
    named sections) comes back as several polylines, drawn independently.
    Cached to disk keyed by the (grid-snapped) bbox; None if Overpass is
    unreachable (both mirrors) or returns nothing."""
    g = _OVERPASS_GRID_DEG
    lat_min = math.floor(lat_min / g) * g
    lat_max = math.ceil(lat_max / g) * g
    lon_min = math.floor(lon_min / g) * g
    lon_max = math.ceil(lon_max / g) * g
    key = "%.4f_%.4f_%.4f_%.4f" % (lat_min, lat_max, lon_min, lon_max)
    cache_path = os.path.join(_OVERPASS_CACHE, key + ".json")
    if os.path.exists(cache_path):
        with open(cache_path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    else:
        query = ('[out:json][timeout:25];'
                 'way["tunnel"="yes"]["highway"](%f,%f,%f,%f);out geom;'
                 % (lat_min, lon_min, lat_max, lon_max))
        data = None
        for url in _OVERPASS_ENDPOINTS:
            try:
                body = urllib.parse.urlencode({"data": query}).encode()
                req = urllib.request.Request(
                    url, data=body,
                    headers={"Content-Type":
                            "application/x-www-form-urlencoded",
                            "User-Agent": _TILE_AGENT})
                with urllib.request.urlopen(req, timeout=25) as r:
                    data = json.loads(r.read().decode("utf-8"))
                break
            except Exception as e:
                print(f"map-frames: Overpass {url} failed ({e})")
        if data is not None:
            os.makedirs(_OVERPASS_CACHE, exist_ok=True)
            with open(cache_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh)
    if data is None:
        return None

    polylines = []
    for el in data.get("elements", []):
        geom = el.get("geometry")
        if not geom:
            continue
        polylines.append([(pt["lat"], pt["lon"]) for pt in geom])
    return polylines or None


def write_frames(out_dir, est_track, fix_track, name, fps=20.0,
                 t_start=None, t_end=None):
    """est_track: list of (t_rel_sec, lat_deg, lon_deg, alt_m, roll_deg,
    pitch_deg, yaw_deg), same recorder as ins_kml.py's write_kml.
    fix_track: list of (t_rel_sec, lat_deg, lon_deg, alt_m, cov_nn, cov_ne,
    cov_ee) raw GNSS fixes, held flat while no fix arrives (see
    ins_kml.py's write_kml docstring); drawn as individual points plus a
    1-sigma position error ellipse (not connected by a line -- a line
    would draw a straight chord across a GNSS outage same as the removed
    ground-truth line did, and a fix is a point measurement, not a path).
    t_start/t_end: seconds into the replay to render, default the whole
    est_track span. --kml-hz controls the recorder's sampling rate (shared
    with --kml); raise it for smoother motion at a high fps.

    No ground-truth line: ref.csv is, for several datasets, the raw GNSS
    solution again rather than an independent reference (see each
    dataset's own ref.csv header), and it is not time-synced here the way
    est_track/fix_track are, so a gap in it (e.g. the same GNSS outage
    fix_track shows frozen) would draw as a straight chord across the
    outage instead of breaking -- misleading in a tunnel/underpass shot.
    The OSM background already shows the road; raw GNSS vs. INSLIB is the
    contrast that matters here."""
    if not est_track:
        print("map-frames: no recorded samples (filter never initialized?)")
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse, Patch

    os.makedirs(out_dir, exist_ok=True)

    t0 = t_start if t_start is not None else est_track[0][0]
    t1 = t_end if t_end is not None else est_track[-1][0]

    # Framing comes from the [t0, t1] window only, not the whole dataset --
    # otherwise a short clip out of a long drive zooms out to fit the
    # entire route instead of the stretch actually being rendered.
    windowed = [(p[1], p[2]) for p in est_track if t0 <= p[0] <= t1] \
             + [(p[1], p[2]) for p in fix_track if t0 <= p[0] <= t1]
    if not windowed:
        windowed = [(p[1], p[2]) for p in est_track]
    lats = [p[0] for p in windowed]
    lons = [p[1] for p in windowed]
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)

    # Zoom high enough that the whole recorded extent still fits one
    # stitched image at a sane tile count (<= ~6x6 tiles).
    zoom = 17
    while zoom > 10:
        x0, _ = _tile_xy(lat_max, lon_min, zoom)
        x1, _ = _tile_xy(lat_min, lon_max, zoom)
        if (x1 - x0) <= 6:
            break
        zoom -= 1

    image, to_px = _basemap(lat_min, lat_max, lon_min, lon_max, zoom)
    tunnels = _fetch_tunnel_geometry(lat_min, lat_max, lon_min, lon_max)
    if tunnels:
        # The query bbox is grid-snapped for cache reuse (see
        # _OVERPASS_GRID_DEG), so it is usually bigger than the framed
        # area and pulls in unrelated tunnel=yes ways nearby (culverts,
        # footways, a different road entirely) -- keep only the ones the
        # actual driven route comes within ROUTE_TUNNEL_MAX_M of, so the
        # gray band (and its "Tunnel" legend entry) is the tunnel that was
        # actually driven, not some unrelated tagged way in the bbox.
        ROUTE_TUNNEL_MAX_M = 80.0
        route = windowed
        tunnels = [line for line in tunnels
                  if any(_haversine_m(rlat, rlon, tlat, tlon) < ROUTE_TUNNEL_MAX_M
                        for tlat, tlon in line for rlat, rlon in route)]

    fig = plt.figure(figsize=(9, 16), dpi=120)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor("#e8e8e0")
    if image is not None:
        ax.imshow(image, origin="upper")
        px = lambda lat, lon: to_px(lat, lon)                # noqa: E731
    else:
        # No tiles: plot directly in lon/lat (flipped y for screen-down).
        px = lambda lat, lon: (lon, -lat)                    # noqa: E731

    if tunnels:
        for line in tunnels:
            tx, ty = zip(*[px(lat, lon) for lat, lon in line])
            ax.plot(tx, ty, "-", color=_TUNNEL_COLOR, linewidth=14,
                   solid_capstyle="round", zorder=1)

    if image is not None:
        ax.set_xlim(0, image.shape[1])
        ax.set_ylim(image.shape[0], 0)
    ax.set_aspect("equal")
    ax.axis("off")

    fix_points = ax.scatter([], [], s=45, color=_FIX_COLOR, zorder=4,
                            label="raw GNSS")
    (est_line,) = ax.plot([], [], "-", color=_EST_COLOR, linewidth=3,
                          zorder=5, label="INSLIB")
    (est_dot,) = ax.plot([], [], "o", color=_EST_COLOR, markersize=9, zorder=6)
    legend_handles = [fix_points, est_line]
    if tunnels:
        legend_handles.append(Patch(facecolor=_TUNNEL_COLOR, label="Tunnel"))
    ax.legend(handles=legend_handles, loc="lower left", fontsize=40,
             framealpha=0.8, markerscale=2.5, handlelength=1.5, borderpad=0.8)
    title = ax.text(0.02, 0.98, name, transform=ax.transAxes, fontsize=14,
                    va="top", ha="left", color="white",
                    bbox=dict(facecolor="black", alpha=0.5, pad=6))
    no_gnss = ax.text(0.98, 0.98, "No GNSS!", transform=ax.transAxes,
                      fontsize=44, va="top", ha="right", color="white",
                      weight="bold", zorder=10, visible=False,
                      bbox=dict(facecolor="#c00000", alpha=0.85, pad=10))
    # A fix this stale counts as "no GNSS right now": real fixes update at
    # least a few times a second, so this is well above normal update
    # jitter but far below anything but a genuine outage.
    NO_GNSS_GAP_S = 2.0

    n_frames = max(1, int(round((t1 - t0) * fps)))
    est_i = fix_i = fix_ellipse_i = 0
    for frame in range(n_frames + 1):
        t = t0 + frame / fps
        while est_i + 1 < len(est_track) and est_track[est_i + 1][0] <= t:
            est_i += 1
        while fix_i + 1 < len(fix_track) and fix_track[fix_i + 1][0] <= t:
            fix_i += 1

        ex, ey = zip(*[px(lat, lon) for _, lat, lon, *_ in est_track[:est_i + 1]])
        est_line.set_data(ex, ey)
        est_dot.set_data([ex[-1]], [ey[-1]])

        has_fix_now = (bool(fix_track) and fix_track[0][0] <= t
                      and (t - fix_track[fix_i][0]) <= NO_GNSS_GAP_S)
        no_gnss.set_visible(not has_fix_now)

        if fix_track and fix_track[0][0] <= t:
            revealed = fix_track[:fix_i + 1]
            fix_points.set_offsets([px(lat, lon) for _, lat, lon, *_ in revealed])
            # Ellipses are patches, not a single updatable artist: add
            # each fix's just once, when it first becomes visible, rather
            # than rebuilding the whole set every frame.
            for _, lat, lon, _, cov_nn, cov_ne, cov_ee in \
                    fix_track[fix_ellipse_i:fix_i + 1]:
                ell = _error_ellipse_px(cov_nn, cov_ne, cov_ee, lat, zoom)
                if ell is not None:
                    w, h, angle = ell
                    w, h = w * _ELLIPSE_VISUAL_SCALE, h * _ELLIPSE_VISUAL_SCALE
                    ex_, ey_ = px(lat, lon)
                    ax.add_patch(Ellipse((ex_, ey_), w, h, angle=angle,
                                         facecolor=_FIX_COLOR, edgecolor="none",
                                         alpha=0.2, zorder=2))
            fix_ellipse_i = fix_i + 1

        title.set_text(f"{name}  t={t:6.1f}s")
        fig.savefig(os.path.join(out_dir, "frame_%05d.png" % frame))

    plt.close(fig)
    print(f"map-frames: wrote {n_frames + 1} frames to {out_dir}")
    print(f"ffmpeg -y -framerate {fps:g} -i "
          f"{os.path.join(out_dir, 'frame_%05d.png')} "
          f"-c:v libx264 -pix_fmt yuv420p {os.path.join(out_dir, name + '.mp4')}")
