#!/usr/bin/env python3
"""
inslib OBD-II speed reader - polls vehicle speed from an ELM327-style
Bluetooth dongle, sends it as UBX 0x40/0x80 frames over UDP and
optionally logs everything to CSV.

The speed feeds ins's absolute-speed aiding (REQ-NAV-068), which fuses
h(x) = ||v_n|| as a single scalar row. That channel wants three things
per sample and this tool produces all three: the speed, a PER-SAMPLE
1-sigma, and how OLD the sample is when it lands.

Three link types, because "OBD dongle" is not one kind of device:

  --serial COM5      Bluetooth Classic (SPP). The OS pairs the dongle and
                     exposes it as a serial port (COMx on Windows,
                     /dev/rfcomm0 or /dev/tty.* elsewhere). Needs pyserial.
  --ble ADDRESS      Bluetooth Low Energy (GATT). Needs bleak.
  --wifi HOST[:PORT] WiFi. The dongle opens its own access point; join it
                     first, then connect by TCP (commonly 192.168.0.10:35000).
                     Standard library only.

Which one you have is largely a question of when it was built and whether
it was meant to work with an iPhone: Apple does not allow generic
Bluetooth Classic SPP for uncertified accessories, so dongles aimed at
iOS went BLE or WiFi, roughly from 2014 on, while Android-only ones
stayed on Classic.

Start with --scan if you are not sure: it lists BLE advertisers and the
serial ports present, so you can tell whether the dongle showed up as a
COM port (-> Classic/SPP) or only as a BLE advertiser (-> --ble). A WiFi
dongle appears in neither list, it is the one that shows up as a WLAN.

    python inslib_obd_speed.py --scan
    python inslib_obd_speed.py --wifi 192.168.0.10
    python inslib_obd_speed.py --serial COM5 --csv obd.csv
    python inslib_obd_speed.py --ble 11:22:33:44:55:66 --udp 127.0.0.1:29798
    python inslib_obd_speed.py --wifi 192.168.0.10 --no-udp     # log only

Ctrl-C stops.

Every run writes a CSV. Without --csv the name is timestamped
(inslib_obd_YYYYmmdd_HHMMSS.csv, one file per run), --csv picks a fixed
one, --no-csv turns it off. A drive is expensive to repeat and a CSV is
not, so the default errs towards keeping the data.

WHAT THE ACCURACY FIGURE MEANS. Only the per-sample part: PID 0x0D is
quantised to 1 km/h, so the floor is the uniform-quantisation sigma
1/sqrt(12) km/h = 0.080 m/s. Added to it in quadrature is what the delay
uncertainty costs, |dv/dt| * sigma_delay: misdating a constant speed is
free, so that term vanishes at cruise and dominates under braking. The
SYSTEMATIC error - a speedometer that by EU type approval may
never read low and in practice reads 2..5% high, plus tyre wear - is
deliberately NOT folded in here: it is a property of the vehicle, not of
the sample, and belongs in the filter's speed_scale / speed_stddev_rel
(see ins_options_t). Mixing the two would hide a constant bias inside a
per-sample noise figure.

WHAT THE DELAY FIGURE MEANS. The value is sampled somewhere between the
request going out and the response coming back, so the best estimate of
its age at response time is half the round trip. On top of that sits the
ECU's own update interval, which is not observable from outside and is
added as a constant (--ecu-delay-ms). Both are reported, so the receiver
can anchor the residual in ins's state history instead of pretending the
reading is current.

Requires pyserial (for --serial) and/or bleak (for --ble/--scan).
"""
import argparse
import collections
import csv
import math
import os
import socket
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inslib_ubx import ODO_DELAY_SUSPECT, build_odometry

# --- UBX 0x40/0x80 odometry --------------------------------------------------
# Same framing as the rest of the inslib protocol (tools/inslib_protocol.md),
# so one parser handles the whole stream.
#
# This tool fills the PRODUCER half of the frame: the host wall clock, the
# measurement and its uncertainty. t_us stays 0 and ODO_T_US_VALID stays
# clear, because this runs on the host and has no access to the MCU
# counter. inslib_hub.py fills those in, being the only process that
# observes both clocks.
#
# Message ids below 0x80 belong to the FIRMWARE (0x01 IMU, 0x02 baro,
# 0x04 status counters, 0x05 time sync). Host-produced side channels take
# 0x80 and up, so the two sources can never collide in a merged stream.

# Round trip this far above the session floor means latency the tool cannot
# account for, so the delay figure that goes with the sample is less
# trustworthy than usual. Deliberately not "the value did not change":
# a constant reading is the normal case at cruise, not a fault.
SPEED_FLAG_LINK_DEGRADED = ODO_DELAY_SUSPECT
RTT_DEGRADED_FACTOR = 3.0
RTT_WINDOW = 100      # samples the round-trip floor is taken over
ACCEL_WINDOW_S = 0.5  # baseline for the dv/dt estimate, long enough that the
                      # 1 km/h quantisation does not dominate it

# PID 0x0D resolution is 1 km/h; a uniformly quantised value has
# sigma = q/sqrt(12).
KMH_TO_MPS = 1.0 / 3.6
QUANT_SIGMA_MPS = (1.0 * KMH_TO_MPS) / math.sqrt(12.0)

# Common GATT profiles of ELM327 BLE clones. Tried in order; the first
# service present on the device wins. (write_uuid, notify_uuid)
BLE_PROFILES = [
    ("0000fff2-0000-1000-8000-00805f9b34fb", "0000fff1-0000-1000-8000-00805f9b34fb"),
    ("0000ffe1-0000-1000-8000-00805f9b34fb", "0000ffe1-0000-1000-8000-00805f9b34fb"),
    ("00002af1-0000-1000-8000-00805f9b34fb", "00002af0-0000-1000-8000-00805f9b34fb"),
    ("0000fff1-0000-1000-8000-00805f9b34fb", "0000fff2-0000-1000-8000-00805f9b34fb"),
]

# ELM327 bring-up. ATE0 matters most: the echo would otherwise have to be
# stripped from every reply and costs link time we are trying to measure.
ELM_INIT = ["ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"]

# Replies that are answers about the link rather than about the vehicle.
ELM_NON_DATA = ("NO DATA", "SEARCHING", "UNABLE TO CONNECT", "STOPPED",
                "CAN ERROR", "BUS INIT", "BUS ERROR", "ERROR", "?")


def speed_frame(t_unix_us, speed_mps, stddev_mps, delay_ms, flags):
    """The producer half of a 0x40/0x80 frame: t_us stays 0 and
    ODO_T_US_VALID stays clear, for inslib_hub.py to fill in."""
    return build_odometry(t_unix_us, speed_mps, stddev_mps,
                          delay_ms=delay_ms, flags=flags)


def parse_speed_reply(text):
    """ELM327 reply -> speed in km/h, or None if it carried no value.

    Expects the hex of a mode-01 PID-0D response, '41 0D xx', with spaces
    and headers already turned off. Anything else (NO DATA, SEARCHING,
    a prompt on its own) is not an error, just not a measurement."""
    cleaned = text.replace("\r", " ").replace("\n", " ").replace(">", " ")
    cleaned = cleaned.replace(" ", "").upper()
    if not cleaned:
        return None
    for marker in ELM_NON_DATA:
        if marker.replace(" ", "") in cleaned:
            return None
    idx = cleaned.find("410D")
    if idx < 0 or len(cleaned) < idx + 6:
        return None
    try:
        return int(cleaned[idx + 4:idx + 6], 16)
    except ValueError:
        return None


class ElmLink:
    """Common half of the two transports: an ELM327 speaks a
    request/response dialogue terminated by the '>' prompt, whichever way
    the bytes travel."""

    def __init__(self, timeout_s):
        self.timeout_s = timeout_s
        self._buf = ""

    def _write(self, data):
        raise NotImplementedError

    def _read_available(self):
        raise NotImplementedError

    def close(self):
        pass

    def command(self, cmd, timeout_s=None):
        """Send one command, wait for the prompt. Returns (reply, rtt_s).
        The reply excludes the prompt. On timeout the reply is whatever
        arrived, which parse_speed_reply then rejects.

        timeout_s overrides the link default for this one exchange, for the
        cases that are known to be slow (protocol search)."""
        self._buf = ""
        self._write((cmd + "\r").encode("ascii"))
        t0 = time.monotonic()
        deadline = t0 + (self.timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            chunk = self._read_available()
            if chunk:
                self._buf += chunk
                if ">" in self._buf:
                    break
            else:
                time.sleep(0.002)
        rtt = time.monotonic() - t0
        return self._buf.split(">")[0], rtt

    def init_elm(self, verbose=True):
        for cmd in ELM_INIT:
            reply, _ = self.command(cmd)
            if verbose:
                print(f"  {cmd:6s} -> {reply.strip().replace(chr(13), ' ')!r}")
            time.sleep(0.1)

    def probe_bus(self, timeout_s):
        """Get the vehicle bus talking before the timing loop starts.

        With ATSP0 the first request to the vehicle triggers the protocol
        search, which takes seconds - far longer than a poll timeout sized
        for the steady state. Doing it here, once, with a generous timeout
        keeps the search out of the RTT measurements, and separates "the
        car is not answering" from "this poll was slow": the former is a
        property of the link and is reported once, not 10 times a second.

        Returns True if the bus answered. Prints what came back either way,
        because the ELM327's own words (SEARCHING, UNABLE TO CONNECT, BUS
        INIT) say more about the cause than any status counter can."""
        reply, rtt = self.command("0100", timeout_s=timeout_s)
        shown = reply.strip().replace("\r", " ")
        print(f"  0100   -> {shown!r}  ({rtt * 1e3:.0f} ms)")
        ok = "4100" in reply.replace("\r", "").replace(" ", "").upper()
        if ok:
            proto, _ = self.command("ATDP")
            print(f"  ATDP   -> {proto.strip().replace(chr(13), ' ')!r}")
        return ok


class SerialLink(ElmLink):
    """Bluetooth Classic / SPP, or any wired ELM327 on a serial port."""

    def __init__(self, port, baud, timeout_s):
        super().__init__(timeout_s)
        try:
            import serial  # pip install pyserial
        except ImportError:
            sys.exit("pyserial is required for --serial: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0)

    def _write(self, data):
        self.ser.reset_input_buffer()
        self.ser.write(data)

    def _read_available(self):
        n = self.ser.in_waiting
        if not n:
            return ""
        return self.ser.read(n).decode("ascii", errors="replace")

    def close(self):
        self.ser.close()


class TcpLink(ElmLink):
    """WiFi ELM327. The dongle runs its own access point and exposes the
    same request/response dialogue on a raw TCP socket, so only the
    plumbing differs from the serial case.

    The socket is kept non-blocking for reads, matching _read_available's
    "return whatever is there" contract, and switched to a timeout for the
    send so a full transmit buffer cannot raise instead of blocking
    briefly."""

    DEFAULT_PORT = 35000

    def __init__(self, host, port, timeout_s):
        super().__init__(timeout_s)
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.sock.settimeout(0.0)

    def _write(self, data):
        # Drop anything still queued from a previous exchange, so a late
        # reply cannot be read as the answer to this request.
        try:
            while True:
                if not self.sock.recv(4096):
                    break
        except (BlockingIOError, OSError):
            pass
        self.sock.settimeout(self.timeout_s)
        try:
            self.sock.sendall(data)
        finally:
            self.sock.settimeout(0.0)

    def _read_available(self):
        try:
            data = self.sock.recv(4096)
        except (BlockingIOError, OSError):
            return ""
        return data.decode("ascii", errors="replace")

    def close(self):
        self.sock.close()


class BleLink(ElmLink):
    """BLE GATT. bleak is async, so a private event loop is run in this
    thread and each transfer is driven to completion on it - the polling
    loop above stays synchronous and identical for both transports."""

    def __init__(self, address, timeout_s):
        super().__init__(timeout_s)
        try:
            import asyncio
            from bleak import BleakClient
        except ImportError:
            sys.exit("bleak is required for --ble: pip install bleak")
        self._asyncio = asyncio
        self._loop = asyncio.new_event_loop()
        self._client = BleakClient(address)
        self._rx = []
        self._loop.run_until_complete(self._client.connect())

        services = self._client.services
        have = {c.uuid.lower() for s in services for c in s.characteristics}
        self._write_uuid = self._notify_uuid = None
        for write_uuid, notify_uuid in BLE_PROFILES:
            if write_uuid in have and notify_uuid in have:
                self._write_uuid, self._notify_uuid = write_uuid, notify_uuid
                break
        if self._write_uuid is None:
            self._loop.run_until_complete(self._client.disconnect())
            sys.exit("No known ELM327 BLE profile on this device. Characteristics found:\n  "
                     + "\n  ".join(sorted(have))
                     + "\nAdd the write/notify pair to BLE_PROFILES if you can identify them.")

        def on_notify(_handle, data):
            self._rx.append(bytes(data).decode("ascii", errors="replace"))

        self._loop.run_until_complete(
            self._client.start_notify(self._notify_uuid, on_notify))
        print(f"BLE profile: write {self._write_uuid}, notify {self._notify_uuid}")

    def _write(self, data):
        self._rx.clear()
        self._loop.run_until_complete(
            self._client.write_gatt_char(self._write_uuid, data, response=False))

    def _read_available(self):
        # Give the loop a slice so queued notifications are delivered.
        self._loop.run_until_complete(self._asyncio.sleep(0.002))
        if not self._rx:
            return ""
        out = "".join(self._rx)
        self._rx.clear()
        return out

    def close(self):
        try:
            self._loop.run_until_complete(self._client.disconnect())
        except Exception:
            pass


def do_scan():
    """List BLE advertisers and serial ports, so the link type can be
    identified without guessing."""
    print("Serial ports (a Bluetooth Classic/SPP dongle appears here once paired):")
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if not ports:
            print("  (none)")
        for p in ports:
            print(f"  {p.device:20s} {p.description}")
    except ImportError:
        print("  (pyserial not installed)")

    print("\nBLE advertisers (5 s scan):")
    try:
        import asyncio
        from bleak import BleakScanner
    except ImportError:
        print("  (bleak not installed: pip install bleak)")
        return
    devices = asyncio.new_event_loop().run_until_complete(BleakScanner.discover(timeout=5.0))
    if not devices:
        print("  (none)")
    for d in devices:
        print(f"  {d.address:20s} {d.name or '?'}")

    # A WiFi dongle is invisible to both lists above: it is an access point,
    # not a Bluetooth device or a serial port. Nothing to scan for from here.
    print("\nWiFi dongles appear in neither list - they run their own access point.\n"
          "Look for an OBD-ish WLAN, join it, then use\n"
          f"  --wifi 192.168.0.10          (port {TcpLink.DEFAULT_PORT} unless stated otherwise)")


def main():
    ap = argparse.ArgumentParser(
        description="Poll OBD-II vehicle speed and publish it as UBX 0x40/0x80 over UDP.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--serial", metavar="PORT",
                     help="serial port of a Bluetooth Classic/SPP (or wired) dongle")
    src.add_argument("--ble", metavar="ADDRESS", help="BLE address of a GATT dongle")
    src.add_argument("--wifi", metavar="HOST[:PORT]",
                     help="WiFi dongle reachable by TCP, join its access point first "
                          f"(port defaults to {TcpLink.DEFAULT_PORT})")
    src.add_argument("--scan", action="store_true",
                     help="list BLE advertisers and serial ports, then exit")
    ap.add_argument("--baud", type=int, default=38400,
                    help="serial baud (ignored over Bluetooth SPP, default 38400)")
    ap.add_argument("--udp", default="127.0.0.1:29798", metavar="HOST:PORT",
                    help="destination for the UBX speed frames (default 127.0.0.1:29798)")
    ap.add_argument("--no-udp", action="store_true", help="do not send, only log")
    ap.add_argument("--csv", metavar="FILE",
                    help="CSV to append to (default: a new timestamped file per run)")
    ap.add_argument("--no-csv", action="store_true",
                    help="do not log at all (a drive that was not logged is gone)")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="target poll rate in Hz (default 10, the link may be slower)")
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="per-command timeout in seconds (default 1.0)")
    ap.add_argument("--search-timeout", type=float, default=10.0,
                    help="timeout for the one-off bus handshake, which runs the "
                         "ATSP0 protocol search and is much slower than a poll "
                         "(default 10)")
    ap.add_argument("--ecu-delay-ms", type=float, default=50.0,
                    help="constant added to RTT/2 for the ECU's own update interval, "
                         "which is not observable from outside (default 50)")
    args = ap.parse_args()

    if args.scan:
        do_scan()
        return 0
    if not args.serial and not args.ble and not args.wifi:
        ap.error("one of --serial, --ble, --wifi or --scan is required")

    if args.serial:
        print(f"Opening {args.serial} @ {args.baud}")
        link = SerialLink(args.serial, args.baud, args.timeout)
    elif args.wifi:
        host, sep, port_txt = args.wifi.rpartition(":")
        if not sep:
            host, port = args.wifi, TcpLink.DEFAULT_PORT
        else:
            host, port = host, int(port_txt)
        print(f"Connecting to {host}:{port}")
        link = TcpLink(host, port, args.timeout)
    else:
        print(f"Connecting to BLE {args.ble}")
        link = BleLink(args.ble, args.timeout)

    print("ELM327 init:")
    link.init_elm()
    if not link.probe_bus(args.search_timeout):
        print("The dongle answers, the vehicle does not. Usual causes:\n"
              "  - ignition off: most ECUs only answer with the ignition on\n"
              "    (engine not needed, key/button to 'on' is enough)\n"
              "  - dongle not fully seated in the OBD socket\n"
              "  - protocol search still running: raise --search-timeout\n"
              "Polling anyway, in case the bus comes up.")

    sock = None
    dest = None
    if not args.no_udp:
        host, _, port = args.udp.rpartition(":")
        dest = (host, int(port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Sending UBX 0x40/0x80 to {dest[0]}:{dest[1]}")
    else:
        print("UDP output disabled (--no-udp)")

    # Logging is ON by default, like the RTCM/UBX captures in
    # inslib_udp_to_serial.py. A drive is expensive to repeat and a CSV is
    # not, so the failure mode worth avoiding is "went out, came back,
    # nothing was written". A fresh timestamped file per run rather than one
    # growing file: runs stay separable without anything being overwritten
    # or silently mixed.
    csv_file = csv_writer = None
    if not args.no_csv:
        csv_path = args.csv or datetime.now().strftime("inslib_obd_%Y%m%d_%H%M%S.csv")
        csv_file = open(csv_path, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(["utc_iso", "t_unix_us", "raw_reply", "speed_kmh",
                                 "speed_mps", "stddev_mps", "delay_ms", "rtt_ms", "flags"])
        print(f"Logging to {csv_path}")
    else:
        print("CSV logging disabled (--no-csv)")

    period = 1.0 / args.rate if args.rate > 0 else 0.0
    n_ok = n_bad = n_degraded = 0
    last_kmh = None
    last_bad = ""
    rtt_window = collections.deque(maxlen=RTT_WINDOW)
    speed_window = collections.deque(
        maxlen=max(2, int(ACCEL_WINDOW_S * max(args.rate, 1.0))))
    t_status = time.monotonic()
    print("Polling 010D (Ctrl-C to stop)")
    try:
        while True:
            t_cycle = time.monotonic()
            reply, rtt = link.command("010D")
            t_resp_unix_us = int(time.time() * 1e6)
            kmh = parse_speed_reply(reply)

            if kmh is None:
                n_bad += 1
                # Keep the dongle's own wording for the status line: an empty
                # reply (timeout) and a NO DATA mean different things and only
                # the raw text tells them apart.
                last_bad = reply.strip().replace("\r", " ").replace("\n", " ")
                last_bad = " ".join(last_bad.split()) or "(timeout)"
                if csv_writer:
                    csv_writer.writerow([datetime.now(timezone.utc).isoformat(),
                                         t_resp_unix_us, reply.strip(), "", "", "",
                                         "", f"{rtt * 1e3:.1f}", ""])
                # No frame on the wire: a missing sample must not look like
                # a measurement of anything.
            else:
                n_ok += 1
                # The value was sampled somewhere in the round trip, so its
                # best-estimate age at t_resp is half of it, plus whatever
                # the ECU itself sat on it (not observable, configurable).
                delay_ms = 0.5 * rtt * 1e3 + args.ecu_delay_ms
                last_kmh = kmh
                speed_mps = kmh * KMH_TO_MPS

                # Delay uncertainty, and what it actually costs.
                #
                # An unchanged reading was previously taken as evidence of a
                # stale cache. It is not: at a steady cruise the value sits
                # on one quantisation step for minutes, which is the normal
                # case, not a fault. And an ELM327 queries the bus per
                # request rather than serving a cache, so the failure that
                # heuristic was guarding against barely exists.
                #
                # What IS observable is the link: the round trip has a floor
                # set by the dongle and the bus, and excursions above it are
                # latency this tool cannot attribute. Treat that excess as
                # the 1-sigma of the delay estimate.
                #
                # A delay error only matters while the speed is changing -
                # misdating a constant speed costs nothing - so it enters
                # the sigma as |dv/dt| * sigma_delay, which vanishes at
                # cruise and grows under braking, exactly where it hurts.
                rtt_window.append(rtt)
                rtt_floor = min(rtt_window)
                sigma_delay_s = max(rtt - rtt_floor, 0.0)

                accel = 0.0
                speed_window.append((t_resp_unix_us * 1e-6, speed_mps))
                if len(speed_window) >= 2:
                    (t_a, v_a), (t_b, v_b) = speed_window[0], speed_window[-1]
                    # Over the whole window, so the 1 km/h quantisation is
                    # divided by a longer baseline instead of dominating.
                    if t_b > t_a:
                        accel = (v_b - v_a) / (t_b - t_a)

                flags = 0
                sigma = math.hypot(QUANT_SIGMA_MPS, accel * sigma_delay_s)
                if rtt > RTT_DEGRADED_FACTOR * rtt_floor and rtt - rtt_floor > 0.05:
                    flags |= SPEED_FLAG_LINK_DEGRADED
                    n_degraded += 1

                if sock:
                    sock.sendto(speed_frame(t_resp_unix_us, speed_mps, sigma,
                                            round(delay_ms), flags), dest)
                if csv_writer:
                    csv_writer.writerow([datetime.now(timezone.utc).isoformat(),
                                         t_resp_unix_us, reply.strip(), kmh,
                                         f"{speed_mps:.4f}", f"{sigma:.4f}",
                                         f"{delay_ms:.1f}", f"{rtt * 1e3:.1f}", flags])

            now = time.monotonic()
            if now - t_status >= 1.0:
                if csv_file:
                    csv_file.flush()
                speed_txt = "--" if last_kmh is None else f"{last_kmh:3d} km/h"
                bad_txt = f"  last {last_bad[:24]!r}" if n_bad else ""
                deg_txt = f"  degraded {n_degraded:5d}" if n_degraded else ""
                print(f"\r{speed_txt}  ok {n_ok:6d}  no-data {n_bad:5d}  "
                      f"rtt {rtt * 1e3:5.1f} ms{deg_txt}{bad_txt}   ", end="")
                t_status = now

            if period > 0:
                sleep_s = period - (time.monotonic() - t_cycle)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except KeyboardInterrupt:
        print(f"\nDone: {n_ok} samples, {n_bad} without data")
    finally:
        link.close()
        if csv_file:
            csv_file.close()
        if sock:
            sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
