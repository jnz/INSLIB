"""(c) Jan Zwiener (jan@zwiener.org)"""

import os
import pygeomag
import datetime

# --- Configuration ---
FILENAME = os.path.join(os.path.dirname(__file__), "..", "src", "wmm_test_vectors.h")
ALTITUDE_KM = 0.0

# Automatic epoch detection (e.g. 2025.0 to 2030.0)
current_year = datetime.date.today().year
EPOCH_START = float((current_year // 5) * 5)
EPOCH_END   = EPOCH_START + 5.0

# Define specific test coordinates (Latitude, Longitude, Year)
TEST_POINTS = [
    # --- Standard Locations (Spatio-temporal Interpolation) ---
    (48.137, 11.575, EPOCH_START + 2.5),   # Munich (Mid-Epoch)
    (40.712, -74.006, EPOCH_START),        # New York (Start of Epoch)
    (35.689, 139.692, EPOCH_END - 0.1),    # Tokyo (Near end of Epoch)

    # --- Boundary Cases (Wrap-around / Clamp testing) ---
    (89.9, 0.0, EPOCH_START + 1.0),        # Near North Pole (Lat clamp test)
    (-89.9, 179.9, EPOCH_START + 1.0),     # Near South Pole
    (0.0, 180.0, EPOCH_START + 3.0),       # Equator / Date line East (Lon wrap test)
    (0.0, -180.0, EPOCH_START + 3.0),      # Equator / Date line West (Lon wrap test)
    (0.0, 370.0, EPOCH_START + 2.0),       # Invalid Longitude (Should wrap to 10.0)
]

def generate_test_header():
    geo_mag = pygeomag.GeoMag()

    lines = [
        "// AUTO-GENERATED TEST VECTORS - DO NOT EDIT",
        "// Generated using pygeomag (Exact Spherical Harmonics model)",
        "// Generator (c) Jan Zwiener (jan@zwiener.org); WMM data: NOAA/NCEI via pygeomag.",
        "// NOTE: Compare these expected values against the C interpolation",
        "// using an epsilon (e.g., +/- 0.6 degrees tolerance).",
        "#pragma once",
        "",
        "typedef struct {",
        "    float lat;",
        "    float lon;",
        "    float year;",
        "    float expected_decl;",
        "} wmm_test_vector_t;",
        "",
        f"const wmm_test_vector_t WMM_TEST_VECTORS[{len(TEST_POINTS)}] = {{"
    ]

    print(f"Generating WMM Test Vectors (Epoch {EPOCH_START} - {EPOCH_END})...")

    for lat, lon, year in TEST_POINTS:
        # Calculate exact truth using WMM Spherical Harmonics
        result = geo_mag.calculate(glat=lat, glon=lon, alt=ALTITUDE_KM, time=year)
        exact_declination = result.d

        # Format the C array entry
        line = f"    {{ {lat:8.3f}f, {lon:8.3f}f, {year:8.3f}f, {exact_declination:8.3f}f }},"
        lines.append(line)

    lines.append("};")
    lines.append(f"const int WMM_TEST_VECTOR_COUNT = {len(TEST_POINTS)};")

    # Write to file
    with open(FILENAME, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"Successfully generated {len(TEST_POINTS)} test vectors into '{FILENAME}'.")

if __name__ == "__main__":
    generate_test_header()
