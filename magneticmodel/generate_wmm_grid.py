"""(c) Jan Zwiener (jan@zwiener.org)"""

import math
import os
import pygeomag

# pip install pygeomag

# --- Config ---
STEP_DEG = 5            # Grid resolution grad (declination)
STEP_DEG_F = 15         # Lower resolution for field strength and inclination
ALTITUDE_KM = 0.0       # Altitude over sea level (0 km)
# Single source of truth: the generated LUT lives next to the C code in src/.
FILENAME = os.path.join(os.path.dirname(__file__), "..", "src", "wmm_lut.h")

DIP_SCAN_STEP_DEG = 2.0   # coarse global scan for dip pole candidates
DIP_POLE_MAX_NT = 1000.0  # a refined candidate above this is not a true zero
DIP_POLE_MERGE_DEG = 1.0  # candidates closer than this are the same pole
# Drift the fixed mid-epoch pole position is allowed to accumulate towards the
# epoch ends. MAGNETIC_DIP_POLE_EXCLUSION_DEG carries this as slack, so the
# generator refuses rather than silently letting the zone slip off the pole.
DIP_POLE_DRIFT_BUDGET_DEG = 2.0

def gc_distance_deg(lat_a, lon_a, lat_b, lon_b):
    """Great-circle distance [deg]. Near the poles a degree of longitude is a
    fraction of a degree on the ground, so distances must not be taken in
    lat/lon."""
    a, b = math.radians(lat_a), math.radians(lat_b)
    dl = math.radians(lon_a - lon_b)
    c = math.sin(a) * math.sin(b) + math.cos(a) * math.cos(b) * math.cos(dl)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def refine_dip_pole(geo_mag, year, lat_guess, lon_guess):
    """Locate a magnetic dip pole (the point where the horizontal field
    vanishes) by shrinking grid refinement around a starting guess.

    Dependency free and deterministic, converges to ~1e-3 deg. The dip poles
    are where the declination becomes meaningless, so magnetic_model.c ships
    their position to let callers exclude that neighbourhood.
    """
    lat, lon = float(lat_guess), float(lon_guess)
    span = 10.0
    while span > 1e-3:
        # Meridians converge near the pole, so a step of one degree in
        # longitude covers far less ground than one in latitude. Scaling the
        # longitude step keeps the search isotropic, otherwise it stalls
        # along the elongated valley and lands up to a degree off.
        lon_scale = 1.0 / max(0.05, math.cos(math.radians(lat)))
        best = None
        for i in range(-5, 6):
            for j in range(-5, 6):
                la = max(-89.99, min(89.99, lat + i * span / 5.0))
                lo = lon + j * span * lon_scale / 5.0
                h = geo_mag.calculate(glat=la, glon=lo, alt=ALTITUDE_KM, time=year).h
                if best is None or h < best[0]:
                    best = (h, la, lo)
        _, lat, lon = best
        span *= 0.4
    return lat, lon


def find_dip_poles(geo_mag, year):
    """Find every point where the horizontal field vanishes.

    A coarse global scan collects candidate minima, each is refined, and only
    those that actually reach zero survive (the South Atlantic has a broad
    weak-field minimum around 9000 nT that must not be mistaken for a pole).
    Scanning rather than starting from fixed guesses keeps this correct as the
    poles drift, and surfaces a change in their number instead of silently
    tracking the wrong spot. Today's field has two, but that is a property of
    the present dipole dominance, not an invariant.
    """
    lats = [-88.0 + i * DIP_SCAN_STEP_DEG
            for i in range(int(176.0 / DIP_SCAN_STEP_DEG) + 1)]
    lons = [-180.0 + j * DIP_SCAN_STEP_DEG
            for j in range(int(360.0 / DIP_SCAN_STEP_DEG))]
    field = [[geo_mag.calculate(glat=la, glon=lo, alt=ALTITUDE_KM, time=year).h
              for lo in lons] for la in lats]

    poles = []
    for i in range(1, len(lats) - 1):
        for j in range(len(lons)):
            h = field[i][j]
            # Local minimum against all eight neighbours, wrapping in longitude.
            if any(h > field[i + di][(j + dj) % len(lons)]
                   for di in (-1, 0, 1) for dj in (-1, 0, 1)):
                continue
            lat, lon = refine_dip_pole(geo_mag, year, lats[i], lons[j])
            if geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=year).h > DIP_POLE_MAX_NT:
                continue
            if any(gc_distance_deg(lat, lon, p[0], p[1]) < DIP_POLE_MERGE_DEG for p in poles):
                continue  # the coarse scan can hit one flat valley twice
            poles.append((lat, lon))

    # Stable order so a regenerated header diffs cleanly.
    return sorted(poles, reverse=True)


def generate_header():
    # pygeomag loads the active WMM (e.g. WMM2025)
    geo_mag = pygeomag.GeoMag()

    # The model knows its own validity range, so the epoch is never guessed
    # from the system clock. pygeomag rejects a year outside it outright.
    epoch_start, epoch_end = (float(v) for v in geo_mag.life_span)

    lat_count = (180 // STEP_DEG) + 1
    lon_count = (360 // STEP_DEG) + 1

    lat_count_f = (180 // STEP_DEG_F) + 1
    lon_count_f = (360 // STEP_DEG_F) + 1

    print(f"Generating WMM grids (epoch {epoch_start} to {epoch_end})...")
    print(f"  - Declination step: {STEP_DEG}°")
    print(f"  - Field strength / inclination step: {STEP_DEG_F}°")

    # C-Header Info
    lines = [
        "// AUTO-GENERATED FILE - DO NOT EDIT",
        "// Regenerate with magneticmodel/generate_wmm_grid.py (needs pygeomag).",
        "// Generator (c) Jan Zwiener (jan@zwiener.org); WMM data: NOAA/NCEI via pygeomag.",
        f"// Resolution: {STEP_DEG} Grad (declination), {STEP_DEG_F} Grad (field/inclination)",
        f"// Altitude: {ALTITUDE_KM} km (Mean Sea Level)",
        "// Format: int16_t. Angles in grad * 100 (e.g. 1234 = 12.34 Grad)",
        "// Lon (X): -180 to +180, Lat (Y): -90 to +90",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define WMM_STEP_DEG {STEP_DEG}",
        f"#define WMM_STEP_DEG_F {STEP_DEG_F}",
        f"#define WMM_LAT_MIN (-90)",
        f"#define WMM_LON_MIN (-180)",
        f"#define WMM_EPOCH_START {epoch_start}f",
        f"#define WMM_EPOCH_END {epoch_end}f",
        "",
    ]

    # Dip pole positions. Near these the horizontal field vanishes, the
    # declination is ill-conditioned and the grid cannot resolve it, so the
    # runtime offers a distance query to exclude the neighbourhood.
    #
    # Tabulated once at mid-epoch rather than per epoch: the poles drift about
    # 1.5 deg over five years, so a fixed mid-epoch position is at most ~0.8 deg
    # from the truth at either end, well inside the margin the exclusion radius
    # carries anyway. Interpolating in time would compute a correction smaller
    # than the uncertainty in choosing that radius, so the runtime does not
    # take a year at all, like the inclination and field strength queries.
    epoch_mid = 0.5 * (epoch_start + epoch_end)
    print(f"Scanning for dip poles at mid-epoch {epoch_mid}...")
    poles = find_dip_poles(geo_mag, epoch_mid)
    for la, lo in poles:
        print(f"  dip pole: lat {la:8.3f}, lon {lo:8.3f}")

    if not poles:
        raise SystemExit("no dip poles found, the scan or the model is wrong")

    # How far the fixed position sits from the truth at the epoch ends decides
    # whether the radius margin still covers the drift. Report it, and refuse
    # if it grew past what MAGNETIC_DIP_POLE_EXCLUSION_DEG budgets for.
    worst = 0.0
    for year in (epoch_start, epoch_end):
        for la, lo in poles:
            true_poles = find_dip_poles(geo_mag, year)
            if not true_poles:
                continue
            worst = max(worst, min(gc_distance_deg(la, lo, t[0], t[1]) for t in true_poles))
    print(f"  largest drift from the mid-epoch position: {worst:.3f} deg")
    if worst > DIP_POLE_DRIFT_BUDGET_DEG:
        raise SystemExit(
            f"dip poles drift {worst:.3f} deg from mid-epoch, more than the "
            f"{DIP_POLE_DRIFT_BUDGET_DEG} deg the exclusion radius budgets for. "
            "Raise MAGNETIC_DIP_POLE_EXCLUSION_DEG in magnetic_model.h or "
            "reintroduce a time-dependent pole position."
        )

    lines += [
        "// Magnetic dip poles (horizontal field = 0), located by a global scan",
        "// on the exact spherical-harmonics model. Each row is {lat, lon} in",
        "// degrees, taken at mid-epoch: the poles drift ~1.5 deg over the five",
        "// years, so this is at most ~0.8 deg off at either end, which the",
        "// exclusion radius absorbs. The count is what the scan found, it is",
        "// not a fixed property of the field.",
        f"#define WMM_DIP_POLE_COUNT {len(poles)}",
        f"const float wmm_dip_pole[{len(poles)}][2] = {{",
    ]
    for la, lo in poles:
        lines.append(f"    {{ {la:9.3f}f, {lo:9.3f}f }},")
    lines.append("};")

    lines += [
        "",
        f"// Declination values for Epoch {epoch_start}",
        f"const int16_t wmm_decl_start[{lat_count}][{lon_count}] = {{"
    ]

    # Generate Start Epoch
    for lat in range(-90, 90 + 1, STEP_DEG):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=epoch_start)
            dec_quantized = int(round(result.d * 100))
            row_values.append(f"{dec_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")
    lines.append("")

    # Generate End Epoch
    lines.append(f"// Declination values for Epoch {epoch_end}")
    lines.append(f"const int16_t wmm_decl_end[{lat_count}][{lon_count}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=epoch_end)
            dec_quantized = int(round(result.d * 100))
            row_values.append(f"{dec_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")
    lines.append("")

    # Absolute field strength (uint8_t micro-Tesla, coarse grid)
    lines.append(f"// ABSOLUTE FIELD STRENGTH micro-Tesla (uT) ({epoch_start})")
    lines.append(f"const uint8_t wmm_field_microTesla[{lat_count_f}][{lon_count_f}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG_F):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG_F):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=epoch_start)
            # Convert nano-Tesla (nT) to micro-Tesla (uT) for uint8_t
            f_quantized = int(round(result.f / 1000.0))
            row_values.append(f"{f_quantized:3d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")
    lines.append("")

    # Inclination (dip) in grad * 100, int16_t, coarse grid. Together with
    # the declination (horizontal direction) and the total field strength
    # this fully defines the NED reference field:
    #   B_ned = F * [cosI cosD, cosI sinD, sinI]
    lines.append(f"// INCLINATION (dip) grad*100 ({epoch_start})")
    lines.append(f"const int16_t wmm_incl_cdeg[{lat_count_f}][{lon_count_f}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG_F):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG_F):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=epoch_start)
            inc_quantized = int(round(result.i * 100))
            row_values.append(f"{inc_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")

    # File output. Explicit UTF-8 and LF: the degree signs in the row comments
    # and the repository's line endings must not follow the local Windows
    # locale, which would rewrite the whole file.
    with open(FILENAME, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")

    print(f"Success! Saved LUT to '{os.path.normpath(FILENAME)}'.")

if __name__ == "__main__":
    generate_header()
