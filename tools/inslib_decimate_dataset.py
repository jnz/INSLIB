#!/usr/bin/env python3
"""Reduce the IMU rate of a dataset directory (datasets/replay_format.py).

    python3 inslib_decimate_dataset.py <dataset> --hz 100 -o <out_dir>

A high-rate capture is mostly imu.csv, which makes a dataset expensive to
carry in a repository. Rates are BLOCK AVERAGED, not sampled: the mean rate
over a block times the block interval is the same delta-angle/delta-velocity
the full-rate stream integrates, so the strapdown sees the same motion.
Dropping every n-th sample instead would fold vibration into the signal band.

Each block keeps the timestamp of its LAST sample, so that dt times the
averaged rate spans exactly the interval the block covers.

The other streams are already low rate and are copied unchanged.

(c) Jan Zwiener (jan@zwiener.org)
"""

import argparse
import os
import shutil


def read_imu(path):
    header, rows = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                header.append(line.rstrip("\n"))
                continue
            if not line.strip():
                continue
            v = line.split(",")
            # Every column after the timestamp, not a fixed six: imu.csv
            # may carry the optional temperature (datasets/replay_format.py)
            # and block-averaging it alongside the rates is the right thing.
            rows.append((int(v[0]), [float(x) for x in v[1:]]))
    return header, rows


def decimate(rows, factor):
    out = []
    for i in range(0, len(rows) - factor + 1, factor):
        block = rows[i:i + factor]
        ncol = len(block[0][1])
        mean = [sum(r[1][k] for r in block) / factor for k in range(ncol)]
        out.append((block[-1][0], mean))
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--hz", type=float, required=True,
                    help="target IMU rate; the factor is rounded to an integer")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    src_imu = os.path.join(args.dataset, "imu.csv")
    header, rows = read_imu(src_imu)
    if len(rows) < 2:
        raise SystemExit("%s: too few samples" % src_imu)

    span = (rows[-1][0] - rows[0][0]) / 1e6
    rate = (len(rows) - 1) / span
    factor = max(int(round(rate / args.hz)), 1)
    out = decimate(rows, factor) if factor > 1 else rows

    dst_imu = os.path.join(args.out, "imu.csv")
    with open(dst_imu, "w", newline="\n") as fh:
        for line in header:
            fh.write(line + "\n")
        fh.write("# block-averaged from %.0f Hz by %d (inslib_decimate_dataset.py)\n"
                 % (rate, factor))
        for t, v in out:
            fh.write("%d,%s\n" % (t, ",".join("%.9g" % x for x in v)))

    for name in os.listdir(args.dataset):
        if name == "imu.csv" or os.path.isdir(os.path.join(args.dataset, name)):
            continue
        shutil.copy2(os.path.join(args.dataset, name),
                     os.path.join(args.out, name))

    before = os.path.getsize(src_imu)
    after = os.path.getsize(dst_imu)
    print("%s: %.0f Hz -> %.0f Hz (factor %d), %d -> %d samples, "
          "imu.csv %.1f -> %.1f MB"
          % (args.dataset, rate, rate / factor, factor, len(rows), len(out),
             before / 1048576.0, after / 1048576.0))


if __name__ == "__main__":
    main()
