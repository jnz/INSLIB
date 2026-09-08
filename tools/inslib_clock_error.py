#!/usr/bin/env python3
"""Estimate the sensor board's clock error against GPS time.

    python3 inslib_clock_error.py <capture>_timesync.csv [...]

The timesync CSV pairs the board's free-running microsecond counter with the
GPS time of a receiver time pulse. A straight line through those pairs gives
the counter's frequency error in ppm; the residual says how well the pairing
itself is behaving.

The fit is per segment: a counter restart (the mcu_restarts / segment columns)
breaks the time base, so pairs from either side of one must not share a fit.

The frequency error scales every integrated quantity uniformly, which at a few
tens of ppm is negligible for dead reckoning. It matters where board time has
to be converted to GPS time -- post-processing raw observations, or stamping
GNSS epochs with their true time of validity instead of their arrival time.

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import csv
import math
import sys

GPS_WEEK_SEC = 604800.0


def read_pairs(path):
    """[(segment, mcu_us, gps_seconds)] for rows that carry a time pulse."""
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if not row.get("timesync_t_us") or not row.get("gps_tow_s"):
                continue
            try:
                week = float(row.get("gps_week") or 0.0)
                out.append((int(float(row.get("segment") or 0)),
                            float(row["timesync_t_us"]),
                            week * GPS_WEEK_SEC + float(row["gps_tow_s"])))
            except ValueError:
                continue
    return out


def fit(pairs):
    """Least squares gps_seconds = a * mcu_us + b. Returns (ppm, rms, span)."""
    n = len(pairs)
    sx = sum(x for _, x, _ in pairs)
    sy = sum(y for _, _, y in pairs)
    sxx = sum(x * x for _, x, _ in pairs)
    sxy = sum(x * y for _, x, y in pairs)
    den = n * sxx - sx * sx
    if den == 0.0:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    res = [y - (a * x + b) for _, x, y in pairs]
    rms = math.sqrt(sum(r * r for r in res) / n)
    # a is [s GPS] per [us board]; nominal 1e-6. Positive ppm -> the counter
    # runs slow (a board microsecond is longer than a real one).
    return (a * 1e6 - 1.0) * 1e6, rms, (max(x for _, x, _ in pairs)
                                        - min(x for _, x, _ in pairs)) / 1e6


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("timesync", nargs="+", help="*_timesync.csv from a capture")
    ap.add_argument("--min-pairs", type=int, default=3,
                    help="skip a segment with fewer time pulses than this")
    args = ap.parse_args()

    worst = 0.0
    for path in args.timesync:
        pairs = read_pairs(path)
        segments = sorted({s for s, _, _ in pairs})
        if not pairs:
            print("%s: no time pulses" % path)
            continue
        for seg in segments:
            sel = [p for p in pairs if p[0] == seg]
            label = path if len(segments) == 1 else "%s [segment %d]" % (path, seg)
            if len(sel) < args.min_pairs:
                print("%s: only %d time pulse(s), need %d"
                      % (label, len(sel), args.min_pairs))
                continue
            ppm, rms, span = fit(sel)
            print("%s\n  %+8.2f ppm over %.0f s from %d pulses, "
                  "residual %.1f us rms" % (label, ppm, span, len(sel), rms * 1e6))
            worst = max(worst, abs(ppm))

    if worst:
        print("\n%.0f ppm is %.1f ms of drift per hour of board time"
              % (worst, worst * 3.6))


if __name__ == "__main__":
    main()
