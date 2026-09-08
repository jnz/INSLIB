#!/usr/bin/env python3
"""Replay a recorded .ubx capture to a UDP port at its original pace.

The counterpart to inslib_hub.py's live fan-out: both put the same byte
stream on the same socket, so tools/insrcv.c cannot tell a replay from a
live session. That is the whole point - a capture taken once can be
replayed as often as you like, and what it exercises is the real framer,
the real decoders and the real filter rather than a test double.

    python inslib_replay_udp.py capture.ubx
    python inslib_replay_udp.py capture.ubx --udp 127.0.0.1:29800
    python inslib_replay_udp.py capture.ubx --speed 4      # 4x real time
    python inslib_replay_udp.py capture.ubx --speed 0      # as fast as possible
    python inslib_replay_udp.py capture.ubx --start 120 --duration 30

PACING. The capture carries no host timestamps, only the MCU's
free-running microsecond counter, so the counter IS the clock here: the
file is walked frame by frame and each frame goes out when its t_us says
it should. Frames without a t_us (the GNSS passthrough) inherit the most
recent one, which is where they sat in the stream anyway.

--speed above 1 compresses that timeline. Be aware of what it costs: a
consumer that throttles its own publishing on the host clock (insrcv
does) then sees fewer samples per unit of MCU time, so plots get sparser
the faster you replay. --speed 1 is the only setting where a replay
behaves exactly like the live session it came from.

Standard library only.
"""
import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inslib_ubx import (CLS_INSLIB, ID_BARO, ID_IMU, ID_TIMESYNC,
                        UDP_MAX_PAYLOAD, UbxFramer)

# The class-0x40 messages whose payload starts with the MCU t_us. Read
# straight off the front rather than fully decoded: pacing only needs the
# timestamp, and this way a new message type paces correctly the day the
# firmware gains it, as long as it follows that convention.
#
# 0x80 odometry does not follow it: its first u64 is the HOST wall clock
# (the MCU time sits behind it and is only valid with T_US_VALID), so it
# is deliberately absent here. Pacing on a Unix timestamp would schedule
# the whole capture decades into the future. Those frames inherit the
# preceding timestamp instead, which is where the hub injected them.
TIMED_IDS = (ID_IMU, ID_BARO, ID_TIMESYNC)

# A backward step in t_us bigger than this is a device restart; anything
# smaller is the sensor streams interleaving (the baro latching just before
# an IMU sample that was queued first).
RESTART_STEP_US = 1_000_000
RESTART_SEAM_US = 1_000_000     # replayed gap across a restart


def frame_t_us(cls, mid, payload):
    if cls != CLS_INSLIB or mid not in TIMED_IDS or len(payload) < 8:
        return None
    return struct.unpack("<Q", payload[:8])[0]


def parse_dest(text):
    if ":" in text:
        host, _, port = text.rpartition(":")
        return host, int(port)
    return "127.0.0.1", int(text)


def main():
    ap = argparse.ArgumentParser(
        description="Replay a .ubx capture to a UDP port at the original pace.")
    ap.add_argument("capture", help="raw .ubx file, as written by inslib_hub.py")
    ap.add_argument("--udp", default="127.0.0.1:29800",
                    help="destination HOST:PORT or just PORT (default %(default)s)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="pace multiplier; 0 = no pacing at all (default %(default)s)")
    ap.add_argument("--start", type=float, default=0.0,
                    help="skip this many seconds of capture time before sending")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds of capture time (0 = all)")
    ap.add_argument("--loop", action="store_true",
                    help="restart from the beginning when the file ends")
    ap.add_argument("--max-gap", type=float, default=0.0,
                    help="compress idle gaps longer than this many seconds "
                         "(0 = replay them faithfully, the default)")
    args = ap.parse_args()

    host, port = parse_dest(args.udp)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    with open(args.capture, "rb") as fh:
        data = fh.read()
    print(f"{args.capture}: {len(data)/1024:.1f} KB -> {host}:{port} "
          f"(speed {'max' if args.speed <= 0 else f'{args.speed}x'})")

    pass_n = 0
    try:
        while True:
            pass_n += 1
            stats = replay_once(sock, (host, port), data, args)
            print(f"\npass {pass_n}: {stats['frames']} frames in {stats['datagrams']} "
                  f"datagrams, {stats['resync']} resync bytes, "
                  f"{stats['span_s']:.1f} s of capture time"
                  + (f", {len(stats['gaps'])} gap(s) totalling "
                     f"{sum(stats['gaps']):.1f} s" if stats["gaps"] else ""))
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        sock.close()


def replay_once(sock, dest, data, args):
    framer = UbxFramer()
    # The capture is a serial byte stream, so a frame CAN straddle a read
    # boundary here; feeding it in one go avoids the question entirely and
    # a capture that does not fit in memory is not a case worth carrying.
    frames = framer.feed(data)

    t_us_first = t_us_prev = None
    t_offset = 0                # added to t_us to bridge a device restart
    batch, batch_len = [], 0
    n_frames = n_datagrams = 0
    t_wall0 = time.monotonic()
    pending_t_us = None
    span_s = 0.0
    # Time removed from the schedule by --max-gap, and the gaps seen. A
    # recording stitched across a USB dropout (or one that starts with a
    # stale device buffer) carries multi-second holes, and replaying one
    # faithfully looks exactly like a hung tool unless it is announced.
    skew_s = 0.0
    gaps = []
    prev_rel = None
    GAP_REPORT_S = 1.0

    def flush():
        nonlocal batch, batch_len, n_datagrams
        if batch:
            sock.sendto(b"".join(batch), dest)
            n_datagrams += 1
            batch, batch_len = [], 0

    for cls, mid, payload, frame in frames:
        t_us = frame_t_us(cls, mid, payload)
        if t_us is not None:
            # No wrap handling: t_us is monotonic u64 for the whole session
            # (inslib_protocol.md "Timebase"). What DOES step backwards is
            # a device restart mid-capture, and the two timelines then have
            # to be stitched: replaying the second one against the first
            # would either fire everything at once or wait out the previous
            # uptime. The seam is nominal, since how long the device was
            # away is not in the capture. Small backward steps are the
            # streams interleaving, not a restart.
            if t_us_prev is not None and t_us < t_us_prev - RESTART_STEP_US:
                t_offset += t_us_prev - t_us + RESTART_SEAM_US
                print(f"\n  device restart in the capture at t={span_s:.1f}s"
                      f", continuing {RESTART_SEAM_US/1e6:.0f}s later")
            t_us_prev = t_us
            pending_t_us = t_us + t_offset
            if t_us_first is None:
                t_us_first = pending_t_us

        if pending_t_us is None:
            continue    # leading GNSS frames before the first timestamp
        t_rel = (pending_t_us - t_us_first) * 1e-6
        if t_rel < args.start:
            continue
        if args.duration > 0.0 and t_rel > args.start + args.duration:
            break
        span_s = t_rel - args.start

        if prev_rel is not None and t_rel - prev_rel > GAP_REPORT_S:
            gap = t_rel - prev_rel
            if args.max_gap > 0.0 and gap > args.max_gap:
                skew_s += gap - args.max_gap
                note = f", compressed to {args.max_gap:.1f}s"
            else:
                note = ", waiting it out (--max-gap compresses it)"
            gaps.append(gap)
            print(f"\n  gap of {gap:.1f}s in the recording at t={prev_rel:.1f}s{note}")
        prev_rel = t_rel

        if args.speed > 0.0:
            due = t_wall0 + (t_rel - args.start - skew_s) / args.speed
            delay = due - time.monotonic()
            if delay > 0.0:
                # Anything buffered is due NOW, so it goes out before the
                # sleep rather than after it: holding it back would add the
                # whole sleep to its latency.
                flush()
                time.sleep(delay)

        if batch_len + len(frame) > UDP_MAX_PAYLOAD:
            flush()
        batch.append(frame)
        batch_len += len(frame)
        n_frames += 1

    flush()
    return {"frames": n_frames, "datagrams": n_datagrams,
            "resync": framer.n_resync, "span_s": span_s, "gaps": gaps}


if __name__ == "__main__":
    main()
