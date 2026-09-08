"""(c) Jan Zwiener (jan@zwiener.org)"""

import os
import pygeomag
import datetime

# pip install pygeomag

# --- Config ---
STEP_DEG = 5            # Grid resolution grad (declination)
STEP_DEG_F = 15         # Lower resolution for field strength and inclination
ALTITUDE_KM = 0.0       # Altitude over sea level (0 km)
# Single source of truth: the generated LUT lives next to the C code in src/.
FILENAME = os.path.join(os.path.dirname(__file__), "..", "src", "wmm_lut.h")

current_year = datetime.date.today().year
EPOCH_START = float((current_year // 5) * 5)
EPOCH_END   = EPOCH_START + 5.0

def generate_header():
    # pygeomag loads the active WMM (e.g. WMM2025)
    geo_mag = pygeomag.GeoMag()

    lat_count = (180 // STEP_DEG) + 1
    lon_count = (360 // STEP_DEG) + 1

    lat_count_f = (180 // STEP_DEG_F) + 1
    lon_count_f = (360 // STEP_DEG_F) + 1

    print(f"Generating WMM grids (epoch {EPOCH_START} to {EPOCH_END})...")
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
        f"#define WMM_EPOCH_START {EPOCH_START}f",
        f"#define WMM_EPOCH_END {EPOCH_END}f",
        "",
        f"// Declination values for Epoch {EPOCH_START}",
        f"const int16_t wmm_decl_start[{lat_count}][{lon_count}] = {{"
    ]

    # Generate Start Epoch
    for lat in range(-90, 90 + 1, STEP_DEG):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=EPOCH_START)
            dec_quantized = int(round(result.d * 100))
            row_values.append(f"{dec_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")
    lines.append("")

    # Generate End Epoch
    lines.append(f"// Declination values for Epoch {EPOCH_END}")
    lines.append(f"const int16_t wmm_decl_end[{lat_count}][{lon_count}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=EPOCH_END)
            dec_quantized = int(round(result.d * 100))
            row_values.append(f"{dec_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")
    lines.append("")

    # Absolute field strength (uint8_t micro-Tesla, coarse grid)
    lines.append(f"// ABSOLUTE FIELD STRENGTH micro-Tesla (uT) ({EPOCH_START})")
    lines.append(f"const uint8_t wmm_field_microTesla[{lat_count_f}][{lon_count_f}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG_F):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG_F):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=EPOCH_START)
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
    lines.append(f"// INCLINATION (dip) grad*100 ({EPOCH_START})")
    lines.append(f"const int16_t wmm_incl_cdeg[{lat_count_f}][{lon_count_f}] = {{")
    for lat in range(-90, 90 + 1, STEP_DEG_F):
        row_values = []
        for lon in range(-180, 180 + 1, STEP_DEG_F):
            result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=EPOCH_START)
            inc_quantized = int(round(result.i * 100))
            row_values.append(f"{inc_quantized:6d}")
        lines.append(f"    {{ {', '.join(row_values)} }}, // Lat: {lat:3d}°")
    lines.append("};")

    # File output
    with open(FILENAME, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"Success! Saved LUT to '{os.path.normpath(FILENAME)}'.")

if __name__ == "__main__":
    generate_header()
