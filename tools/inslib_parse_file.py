#!/usr/bin/env python3
"""
inslib offline parser - analyzes a .ubx file captured with
inslib_hub.py (or any other 1:1 serial capture): IMU rate, seq gaps
(drops), odometry, GNSS message counts.

If the firmware delivered cleanly AND the raw logger caught everything,
the IMU seq is gapless -> "IMU drops: 0". That proves the firmware path
independent of the live parser's speed.

Usage:      python inslib_parse_file.py inslib_raw.ubx

Standard library only: framing/resync/checksum come from inslib_ubx.py,
the class-0x40 payloads are decoded there too.
"""
import os
import sys
import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inslib_ubx import (CLS_INSLIB, ID_IMU, ID_ODOMETRY, IMU_FMT, IMU_LEN,
                        ODO_T_DEGRADED, UbxFramer, parse_odometry)

if len(sys.argv) < 2:
    print(__doc__); sys.exit(1)

data = open(sys.argv[1], "rb").read()

imu_n = imu_drops = resyncs = 0
last_seq = last_t = None
dur_us = 0
gnss = {}
odo_n = odo_degraded = 0
odo_speed_min = odo_speed_max = None

framer = UbxFramer()
for cls, mid, payload, _frame in framer.feed(data):
    if cls == CLS_INSLIB and mid == ID_IMU and len(payload) == IMU_LEN:
        imu_n += 1
        vals = struct.unpack(IMU_FMT, payload)
        t_us, seq = vals[0], vals[8]
        # Only count plausible jumps -- a checksum-valid-but-wrong frame
        # after a resync would otherwise blow up the statistic.
        if last_seq is not None:
            d = (seq - last_seq - 1) & 0xFFFF
            if d > 1000:
                resyncs += 1
            else:
                imu_drops += d
        if last_t is not None:
            # t_us is monotonic u64 and needs no unwrapping; a step that is
            # negative or over a second is a device restart or a gap in the
            # capture and would not be a sample interval either way.
            dt = t_us - last_t
            if 0 < dt < 1_000_000:
                dur_us += dt
        last_seq, last_t = seq, t_us
    elif cls == CLS_INSLIB and mid == ID_ODOMETRY:
        odo = parse_odometry(payload)
        if odo is not None:
            odo_n += 1
            if odo["flags"] & ODO_T_DEGRADED:
                odo_degraded += 1
            v = odo["speed_mps"]
            odo_speed_min = v if odo_speed_min is None else min(odo_speed_min, v)
            odo_speed_max = v if odo_speed_max is None else max(odo_speed_max, v)
    else:
        gnss[(cls, mid)] = gnss.get((cls, mid), 0) + 1

dur = dur_us / 1e6
print(f"File size        : {len(data)/1024:.1f} KB")
print(f"IMU frames       : {imu_n}")
if dur:
    print(f"IMU duration (t_us) : {dur:.2f} s  -> {imu_n/dur:.1f} Hz")
print(f"IMU drops (seq)  : {imu_drops}   <- should be 0")
print(f"Resync artifacts : {resyncs}   (implausible seq jumps, not counted as a drop)")
# Bytes the framer had to skip to find the next valid frame. On a clean
# capture this is 0 or the few bytes before the first sync; anything more
# means the byte stream itself was damaged.
print(f"Skipped bytes    : {framer.n_resync}   <- >0 = corrupt byte stream (GNSS line?)")
if odo_n:
    print(f"Odometry (0x80)  : {odo_n} frames"
          + (f", {odo_speed_min:.2f}..{odo_speed_max:.2f} m/s" if odo_n else "")
          + (f", {odo_degraded} with a degraded timestamp" if odo_degraded else ""))
print("GNSS/other:")
for (c, m), n in sorted(gnss.items()):
    print(f"  {c:02X}/{m:02X}: {n}" + (f"  ({n/dur:.1f} Hz)" if dur else ""))
