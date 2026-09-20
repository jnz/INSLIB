#!/usr/bin/env python3
"""Regression tests for tools/inslib_decimate_dataset.py.

The tool used to take the input rate from the total span of imu.csv. One gap
in the stream (a real capture had five stray samples, then 441 s of nothing)
shrinks that estimate, so an 800 Hz stream came out at factor 3 and 175 Hz
instead of factor 4 and 200 Hz, and a block could average samples from both
sides of the gap.

Runs under pytest or standalone:

    python3 python/tests/test_decimate_dataset.py
"""

import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inslib_decimate_dataset as dec   # noqa: E402

DT_US = 1250  # 800 Hz


def _stream(t0_us, n, value=0.0):
    return [(t0_us + i * DT_US, [value + i, 2.0 * i]) for i in range(n)]


def test_contiguous_stream_gets_the_nominal_factor():
    out, rate, factor, n_gaps = dec.decimate_stream(_stream(0, 800), 200.0)
    assert abs(rate - 800.0) < 1e-6, rate
    assert factor == 4, factor
    assert n_gaps == 0
    assert len(out) == 200


def test_long_gap_does_not_change_the_factor():
    # The real capture: a handful of stray samples, a long pause, then the
    # contiguous stream. Estimated from the span this reads as ~ 1 Hz.
    rows = _stream(0, 5) + _stream(441_000_000, 8000, value=1000.0)
    out, rate, factor, n_gaps = dec.decimate_stream(rows, 200.0)
    assert abs(rate - 800.0) < 1e-6, rate
    assert factor == 4, factor
    assert n_gaps == 1, n_gaps
    # 5 stray samples give one full block (the tail sample is dropped), the
    # stream after the gap gives 2000.
    assert len(out) == 1 + 2000, len(out)


def test_no_block_straddles_a_gap():
    before = _stream(0, 6)                      # 6 samples, factor 4: 1 block + 2 left over
    after = _stream(10_000_000, 8, value=500.0)
    out, _, factor, _ = dec.decimate_stream(before + after, 200.0)
    assert factor == 4
    gap_start = before[-1][0]
    gap_end = after[0][0]
    for t, _ in out:
        assert not (gap_start < t < gap_end), t
    # The first block after the gap must be built from post-gap samples only.
    first_after = [v for t, v in out if t >= gap_end][0]
    expected = [sum(r[1][k] for r in after[:4]) / 4.0 for k in range(2)]
    assert first_after == expected, (first_after, expected)


def test_block_average_and_last_timestamp():
    rows = _stream(0, 8)
    out, _, factor, _ = dec.decimate_stream(rows, 200.0)
    assert factor == 4
    assert out[0][0] == rows[3][0]              # timestamp of the LAST sample in the block
    assert out[0][1] == [1.5, 3.0]              # mean of 0..3 and of 0,2,4,6


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print("ok    %s" % name)
        except AssertionError as e:
            failures += 1
            print("FAIL  %s: %s" % (name, e))
    print("==== %d failures ====" % failures)
    sys.exit(1 if failures else 0)
