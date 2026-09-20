"""(c) Jan Zwiener (jan@zwiener.org)

Error analysis for the compact WMM lookup table (src/wmm_lut.h) used by
src/magnetic_model.c.

magnetic_model.c does NOT evaluate the WMM spherical harmonics directly. It
bilinearly interpolates (and, for declination, linearly interpolates in time)
a coarse int16/uint8 grid that was pre-computed by generate_wmm_grid.py. This
script estimates the error that interpolation + quantization introduces,
relative to the exact spherical-harmonics model (pygeomag), by:

  1. Parsing the grids and constants straight out of src/wmm_lut.h (so the
     analysis always matches what is actually shipped, not what the
     generator script's config says).
  2. Re-implementing the exact interpolation from magnetic_model.c in numpy
     (same float32 arithmetic, same clamping/wrapping rules).
  3. Evaluating both the LUT model and pygeomag on a dense lat/lon/year grid
     and comparing declination, inclination, field strength and the full
     NED vector (magnitude + angular error).

Usage:
    pip install pygeomag
    python magneticmodel/wmm_error_analysis.py [--step 1] [--plot]

Requires: numpy, pygeomag, and matplotlib only for --plot.
"""

import argparse
import os
import re
import sys
import time

import numpy as np

LUT_PATH = os.path.join(os.path.dirname(__file__), "..", "src", "wmm_lut.h")
DEG2RAD = np.float32(np.pi / 180.0)


# --------------------------------------------------------------------------
# Parse src/wmm_lut.h (single source of truth: what is actually compiled in)
# --------------------------------------------------------------------------
def parse_defines(text):
    defines = {}
    for m in re.finditer(r"#define\s+(\w+)\s+\(?(-?\d+(?:\.\d+)?)f?\)?", text):
        defines[m.group(1)] = float(m.group(2))
    return defines


def parse_array(text, name, dtype=np.int64):
    idx = text.index(name + "[")
    start = text.index("{", idx)
    depth = 0
    end = None
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                end = i
                break
    if end is None:
        raise ValueError(f"unterminated array '{name}' in {LUT_PATH}")
    body = text[start + 1 : end]
    rows = re.findall(r"\{([^{}]*)\}", body)
    conv = float if dtype == np.float64 else int
    return np.array(
        [[conv(v.strip().rstrip("f")) for v in row.split(",") if v.strip()] for row in rows],
        dtype=dtype,
    )


def load_lut():
    with open(LUT_PATH, "r", encoding="utf-8") as f:
        text = f.read()
    d = parse_defines(text)
    lut = {
        "dip_pole": parse_array(text, "wmm_dip_pole", np.float64),
        "STEP_DEG": int(d["WMM_STEP_DEG"]),
        "STEP_DEG_F": int(d["WMM_STEP_DEG_F"]),
        "LAT_MIN": int(d["WMM_LAT_MIN"]),
        "LON_MIN": int(d["WMM_LON_MIN"]),
        "EPOCH_START": np.float32(d["WMM_EPOCH_START"]),
        "EPOCH_END": np.float32(d["WMM_EPOCH_END"]),
        "decl_start": parse_array(text, "wmm_decl_start"),
        "decl_end": parse_array(text, "wmm_decl_end"),
        "field_uT": parse_array(text, "wmm_field_microTesla"),
        "incl_cdeg": parse_array(text, "wmm_incl_cdeg"),
    }
    return lut


# --------------------------------------------------------------------------
# Re-implementation of magnetic_model.c, vectorized in numpy/float32
# --------------------------------------------------------------------------
def wmm_cell(lat_deg, lon_deg, step, lat_min, lon_min):
    """Mirrors wmm_cell() in src/magnetic_model.c exactly (float32)."""
    lat_deg = np.array(lat_deg, dtype=np.float32)
    lon_deg = np.array(lon_deg, dtype=np.float32)

    lat_deg = np.where(lat_deg >= np.float32(90.0), np.float32(89.999), lat_deg)
    lat_deg = np.where(lat_deg <= np.float32(-90.0), np.float32(-90.0), lat_deg)

    lon_deg = np.mod(lon_deg + np.float32(180.0), np.float32(360.0))
    lon_deg = np.where(lon_deg < np.float32(0.0), lon_deg + np.float32(360.0), lon_deg)
    lon_deg = lon_deg - np.float32(180.0)
    lon_deg = np.where(lon_deg >= np.float32(180.0), np.float32(179.999), lon_deg)

    s = np.float32(step)
    lat_off = lat_deg - np.float32(lat_min)
    lon_off = lon_deg - np.float32(lon_min)

    lat_idx = (lat_off / s).astype(np.int64)  # (int) cast in C == truncation; lat_off/s >= 0 here
    lon_idx = (lon_off / s).astype(np.int64)
    lat_frac = (lat_off - lat_idx.astype(np.float32) * s) / s
    lon_frac = (lon_off - lon_idx.astype(np.float32) * s) / s
    return lat_idx, lat_frac, lon_idx, lon_frac


def bilerp(grid, li, lf, oi, of):
    v00 = grid[li, oi].astype(np.float32)
    v01 = grid[li, oi + 1].astype(np.float32)
    v10 = grid[li + 1, oi].astype(np.float32)
    v11 = grid[li + 1, oi + 1].astype(np.float32)
    bottom = v00 + (v01 - v00) * of
    top = v10 + (v11 - v10) * of
    return bottom + (top - bottom) * lf


def wrap_cdeg(cdeg):
    """Fold an angle in centidegrees into [-18000, 18000), as in magnetic_model.c."""
    cdeg = np.mod(np.float32(cdeg) + np.float32(18000.0), np.float32(36000.0))
    cdeg = np.where(cdeg < np.float32(0.0), cdeg + np.float32(36000.0), cdeg)
    return cdeg - np.float32(18000.0)


def lut_declination_deg(lut, lat_deg, lon_deg, year):
    li, lf, oi, of = wmm_cell(lat_deg, lon_deg, lut["STEP_DEG"], lut["LAT_MIN"], lut["LON_MIN"])
    tf = (np.float32(year) - lut["EPOCH_START"]) / (lut["EPOCH_END"] - lut["EPOCH_START"])

    start = lut["decl_start"].astype(np.float32)
    end = lut["decl_end"].astype(np.float32)

    def node(i, j):
        s = start[i, j]
        e = end[i, j]
        return s + wrap_cdeg(e - s) * tf

    v00 = node(li, oi)
    v01 = node(li, oi + 1)
    v10 = node(li + 1, oi)
    v11 = node(li + 1, oi + 1)

    # Neighbours onto the same branch as v00 (the +/-180 cut runs along the
    # agonic line trailing each dip pole).
    v01 = v00 + wrap_cdeg(v01 - v00)
    v10 = v00 + wrap_cdeg(v10 - v00)
    v11 = v00 + wrap_cdeg(v11 - v00)

    bottom = v00 + (v01 - v00) * of
    top = v10 + (v11 - v10) * of
    return wrap_cdeg(bottom + (top - bottom) * lf) / np.float32(100.0)


def lut_inclination_deg(lut, lat_deg, lon_deg):
    li, lf, oi, of = wmm_cell(lat_deg, lon_deg, lut["STEP_DEG_F"], lut["LAT_MIN"], lut["LON_MIN"])
    return bilerp(lut["incl_cdeg"], li, lf, oi, of) / np.float32(100.0)


def lut_field_uT(lut, lat_deg, lon_deg):
    li, lf, oi, of = wmm_cell(lat_deg, lon_deg, lut["STEP_DEG_F"], lut["LAT_MIN"], lut["LON_MIN"])
    return bilerp(lut["field_uT"], li, lf, oi, of)


def dip_pole_distance_deg(lut, lat_deg, lon_deg):
    """Great-circle distance to the nearest dip pole, as magnetic_model.c computes it.

    Takes no year: the poles are tabulated at mid-epoch and not interpolated in
    time, because the drift over an epoch is smaller than the slack in the
    exclusion radius.
    """
    out = None
    for pla, plo in lut["dip_pole"]:
        a = np.radians(lat_deg)
        b = np.radians(pla)
        dl = np.radians(lon_deg - plo)
        c = np.sin(a) * np.sin(b) + np.cos(a) * np.cos(b) * np.cos(dl)
        d = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
        out = d if out is None else np.minimum(out, d)
    return out


def ned_from_dif(d_deg, i_deg, f_uT):
    d = d_deg * DEG2RAD
    i = i_deg * DEG2RAD
    n = f_uT * np.cos(i) * np.cos(d)
    e = f_uT * np.cos(i) * np.sin(d)
    dn = f_uT * np.sin(i)
    return n, e, dn


def wrap_deg(diff):
    """Wrap an angular difference into [-180, 180)."""
    return (diff + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------
# Ground truth via pygeomag (exact spherical harmonics)
# --------------------------------------------------------------------------
def truth_grid(lats, lons, year):
    import pygeomag

    gm = pygeomag.GeoMag()
    d = np.empty((len(lats), len(lons)), dtype=np.float64)
    i = np.empty_like(d)
    f = np.empty_like(d)
    for a, lat in enumerate(lats):
        for b, lon in enumerate(lons):
            r = gm.calculate(glat=float(lat), glon=float(lon), alt=0.0, time=float(year))
            d[a, b] = r.d
            i[a, b] = r.i
            f[a, b] = r.f / 1000.0  # nT -> uT
    return d, i, f


def stats(name, err, lats2d, lons2d, unit="deg"):
    abs_err = np.abs(err)
    weights = np.cos(np.deg2rad(lats2d))  # area weighting for the "global" RMS
    rms_w = np.sqrt(np.sum(weights * err**2) / np.sum(weights))
    idx = np.unravel_index(np.argmax(abs_err), abs_err.shape)
    print(f"{name:28s} max={abs_err.max():8.4f} {unit}  "
          f"rms(area-w)={rms_w:8.4f} {unit}  "
          f"p99={np.percentile(abs_err, 99):8.4f} {unit}  "
          f"@ lat={lats2d[idx]:7.2f} lon={lons2d[idx]:8.2f}")
    return abs_err.max()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--step", type=float, default=2.0, help="grid step in degrees (default 2)")
    ap.add_argument("--years", type=int, default=5, help="number of years sampled across the epoch (default 5)")
    ap.add_argument("--plot", action="store_true", help="save error heatmaps as PNG (needs matplotlib)")
    ap.add_argument("--exclusion-deg", type=float, default=7.0,
                     help="dip-pole exclusion radius to report against "
                          "(match MAGNETIC_DIP_POLE_EXCLUSION_DEG, default 5)")
    ap.add_argument("--pole-margin", type=float, default=0.5,
                     help="keep this many degrees away from +/-90 lat (pygeomag dip-pole singularity)")
    args = ap.parse_args()

    lut = load_lut()
    print(f"Loaded {LUT_PATH}")
    print(f"  declination grid : {lut['STEP_DEG']} deg step, epoch {lut['EPOCH_START']}-{lut['EPOCH_END']}")
    print(f"  incl/field grid  : {lut['STEP_DEG_F']} deg step")
    print()

    lats = np.arange(-90.0 + args.pole_margin, 90.0 - args.pole_margin + 1e-9, args.step)
    lons = np.arange(-180.0, 180.0 + 1e-9, args.step)
    years = np.linspace(float(lut["EPOCH_START"]), float(lut["EPOCH_END"]), args.years)

    lats2d, lons2d = np.meshgrid(lats, lons, indexing="ij")

    print(f"Grid: {len(lats)} x {len(lons)} points, {len(years)} epochs "
          f"({int(lats.size*lons.size*len(years))} pygeomag evaluations)")
    if args.step > 0.5:
        print(f"WARNING: --step {args.step} UNDERSTATES the maximum error. The worst points sit "
              f"on a thin ridge at the exclusion boundary, and a coarse grid steps over it "
              f"(1.0 deg reported 13.7 deg where 0.5 deg found 15.9 deg). Use --step 0.5 for a "
              f"number to quote or to set the exclusion radius by.")

    worst = {
        "decl": 0.0,
        "incl": 0.0,
        "field": 0.0,
        "vec_ang": 0.0,
        "vec_mag": 0.0,
        "decl_zoned": 0.0,
        "vec_ang_zoned": 0.0,
        "excluded_frac": 0.0,
    }
    t0 = time.time()

    last_year_maps = None  # for --plot: keep the mid-epoch maps around

    for k, year in enumerate(years):
        d_true, i_true, f_true = truth_grid(lats, lons, year)

        d_lut = np.asarray(lut_declination_deg(lut, lats2d, lons2d, year), dtype=np.float64)
        i_lut = np.asarray(lut_inclination_deg(lut, lats2d, lons2d), dtype=np.float64)
        f_lut = np.asarray(lut_field_uT(lut, lats2d, lons2d), dtype=np.float64)

        d_err = wrap_deg(d_lut - d_true)
        i_err = i_lut - i_true
        f_err = f_lut - f_true

        n_t, e_t, dn_t = ned_from_dif(d_true, i_true, f_true)
        n_l, e_l, dn_l = ned_from_dif(d_lut, i_lut, f_lut)
        vec_mag_err = np.sqrt((n_l - n_t) ** 2 + (e_l - e_t) ** 2 + (dn_l - dn_t) ** 2)
        dot = (n_l * n_t + e_l * e_t + dn_l * dn_t) / (f_lut * f_true)
        vec_ang_err = np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))

        print(f"\n--- year {year:.2f} ---")
        worst["decl"] = max(worst["decl"], stats("declination error", d_err, lats2d, lons2d))
        worst["incl"] = max(worst["incl"], stats("inclination error", i_err, lats2d, lons2d))
        worst["field"] = max(worst["field"], stats("field strength error", f_err, lats2d, lons2d, unit="uT"))
        worst["vec_ang"] = max(worst["vec_ang"], stats("NED vector angular error", vec_ang_err, lats2d, lons2d))
        worst["vec_mag"] = max(worst["vec_mag"], stats("NED vector magnitude error", vec_mag_err, lats2d, lons2d, unit="uT"))

        # Same contract magnetic_heading_reference_valid() offers at runtime:
        # inside the exclusion radius the declination is not claimed to be usable.
        outside = dip_pole_distance_deg(lut, lats2d, lons2d) >= args.exclusion_deg
        worst["decl_zoned"] = max(worst["decl_zoned"], np.abs(d_err[outside]).max())
        worst["vec_ang_zoned"] = max(worst["vec_ang_zoned"], vec_ang_err[outside].max())
        # Area, not grid points. An equirectangular grid packs far more points
        # per unit area near the poles, which is exactly where the zones are,
        # so counting points overstates the excluded surface several times over.
        cell_area = np.cos(np.radians(lats2d))
        worst["excluded_frac"] = cell_area[~outside].sum() / cell_area.sum()

        if args.plot and k == len(years) // 2:
            last_year_maps = (year, d_err, i_err, f_err, vec_ang_err)

    dt = time.time() - t0
    print(f"\nEvaluated in {dt:.1f} s")

    print("\n=== Worst case over full grid + all sampled epochs ===")
    print(f"declination            : {worst['decl']:.3f} deg")
    print(f"inclination            : {worst['incl']:.3f} deg")
    print(f"field strength         : {worst['field']:.3f} uT")
    print(f"NED vector angle       : {worst['vec_ang']:.3f} deg")
    print(f"NED vector magnitude   : {worst['vec_mag']:.3f} uT")

    print(f"\n=== Outside the {args.exclusion_deg:.0f} deg dip-pole exclusion zone "
          f"(magnetic_heading_reference_valid() == true) ===")
    print(f"excluded surface       : {100.0 * worst['excluded_frac']:.2f} %")
    print(f"declination            : {worst['decl_zoned']:.3f} deg")
    print(f"NED vector angle       : {worst['vec_ang_zoned']:.3f} deg")
    print("\nThe residual vector error is set by the coarse inclination grid (South "
          "Atlantic Anomaly), not by the dip poles, so no exclusion radius removes it.")

    if args.plot and last_year_maps is not None:
        plot_maps(lats, lons, *last_year_maps)


def plot_maps(lats, lons, year, d_err, i_err, f_err, vec_ang_err):
    import matplotlib.pyplot as plt

    out_dir = os.path.join(os.path.dirname(__file__), "error_analysis_out")
    os.makedirs(out_dir, exist_ok=True)

    maps = [
        ("declination_error_deg", d_err, "deg"),
        ("inclination_error_deg", i_err, "deg"),
        ("field_error_uT", f_err, "uT"),
        ("ned_vector_angle_error_deg", vec_ang_err, "deg"),
    ]
    for fname, data, unit in maps:
        fig, ax = plt.subplots(figsize=(10, 5))
        vmax = np.abs(data).max()
        im = ax.pcolormesh(lons, lats, data, cmap="RdBu_r", vmin=-vmax, vmax=vmax, shading="auto")
        ax.set_title(f"{fname} (epoch {year:.2f}), LUT - truth [{unit}]")
        ax.set_xlabel("longitude [deg]")
        ax.set_ylabel("latitude [deg]")
        fig.colorbar(im, ax=ax, label=unit)
        path = os.path.join(out_dir, f"{fname}.png")
        fig.savefig(path, dpi=120)
        plt.close(fig)
        print(f"saved {path}")


if __name__ == "__main__":
    if sys.version_info < (3, 8):
        raise SystemExit("Python 3.8+ required")
    main()
