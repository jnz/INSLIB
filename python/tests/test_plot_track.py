"""Self-test for ins_plots._thin_track, the geometric decimation behind the
North-East map page: unlike the per-axis min/max thinning the time-series
pages use, it must keep the SHAPE of a 2D ground track to a tolerance that
follows the track's own extent, so a 2 m track stays resolved to a
millimetre while a kilometre-scale one costs the same page size. Runs under
pytest or standalone

(c) Jan Zwiener (jan@zwiener.org)
(``python3 python/tests/test_plot_track.py``)."""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import ins_plots as ip  # noqa: E402


def _max_deviation(x, y, kx, ky):
    """Largest distance from an original vertex (x, y) to the simplified
    polyline (kx, ky) -- the error the reader actually sees on the page.
    Brute force over all segments, fine at these test sizes."""
    worst = 0.0
    for px, py in zip(x, y):
        best = float("inf")
        for i in range(len(kx) - 1):
            x0, y0, x1, y1 = kx[i], ky[i], kx[i + 1], ky[i + 1]
            dx, dy = x1 - x0, y1 - y0
            den = dx * dx + dy * dy
            u = 0.0 if den == 0.0 else ((px - x0) * dx + (py - y0) * dy) / den
            u = min(1.0, max(0.0, u))
            best = min(best, math.hypot(px - (x0 + u * dx), py - (y0 + u * dy)))
        worst = max(worst, best)
    return worst


def test_straight_line_collapses():
    """A straight run carries no shape: two endpoints are enough, however
    many samples it was recorded with."""
    x = [i * 0.01 for i in range(20000)]
    y = [3.0 * v for v in x]
    kx, ky = ip._thin_track(x, y)
    assert len(kx) == 2, len(kx)
    assert (kx[0], ky[0]) == (x[0], y[0]) and (kx[-1], ky[-1]) == (x[-1], y[-1])


def test_tolerance_scales_with_extent():
    """The same circle, drawn at 2 m and at 2 km: the vertex count must not
    depend on the scale (the tolerance is relative), while the ABSOLUTE
    deviation does -- sub-millimetre on the small one, which is the whole
    point of the map page at close range."""
    n = 20000
    counts = []
    for radius in (1.0, 1000.0):
        x = [radius * math.cos(2.0 * math.pi * i / n) for i in range(n)]
        y = [radius * math.sin(2.0 * math.pi * i / n) for i in range(n)]
        kx, ky = ip._thin_track(x, y)
        counts.append(len(kx))
        dev = _max_deviation(x[::37], y[::37], kx, ky)
        assert dev <= 2.0 * radius * ip._MAP_REL_TOL, (radius, dev)
    assert abs(counts[0] - counts[1]) <= 2, counts
    # And it is a real decimation, not a pass-through.
    assert counts[0] < n // 10, counts


def test_gaps_survive():
    """A NaN gap must stay a gap: the two halves are simplified apart and
    never bridged by a chord across the missing stretch."""
    x = [float(i) for i in range(100)] + [math.nan] + [float(i) for i in range(200, 300)]
    y = [0.0] * 100 + [math.nan] + [50.0] * 100
    kx, ky = ip._thin_track(x, y)
    assert sum(1 for v in kx if v != v) == 1, kx
    gap = [i for i, v in enumerate(kx) if v != v][0]
    assert ky[gap - 1] == 0.0 and ky[gap + 1] == 50.0


def test_pure_noise_is_capped():
    """A track that never moved is pure sensor noise at any tolerance
    derived from its (near-zero) extent -- the point budget still has to
    hold, otherwise a long static log would put every sample on the page."""
    rng = random.Random(7)
    n = 60000
    x = [rng.gauss(0.0, 0.01) for _ in range(n)]
    y = [rng.gauss(0.0, 0.01) for _ in range(n)]
    kx, _ = ip._thin_track(x, y)
    assert len(kx) <= ip._MAX_MAP_POINTS, len(kx)


def test_excursion_between_identical_endpoints_survives():
    """A closed loop leaves its two endpoints on top of each other. The
    chord between them is degenerate, and a splitter that only measures
    distance TO that chord would drop the entire loop."""
    n = 500
    x = [math.cos(2.0 * math.pi * i / (n - 1)) for i in range(n)]
    y = [math.sin(2.0 * math.pi * i / (n - 1)) for i in range(n)]
    x[-1], y[-1] = x[0], y[0]
    kx, _ = ip._thin_track(x, y)
    assert len(kx) > 20, len(kx)


if __name__ == "__main__":
    test_straight_line_collapses()
    test_tolerance_scales_with_extent()
    test_gaps_survive()
    test_pure_noise_is_capped()
    test_excursion_between_identical_endpoints_survives()
    print("ok  ins_plots._thin_track keeps track shape at view scale")
