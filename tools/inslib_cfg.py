#!/usr/bin/env python3
"""inslib_cfg -- read, write and reset the sensor board's stored settings.

Command line front end to the firmware's configuration interface
(tools/inslib_protocol.md, "Configuration interface"). The wire format
lives in inslib_ubx.py, which tools/inslib_calib_gui.py uses as well, so
the two cannot drift apart.

The usage, the two configurations a command can mean and the examples
live in DESCRIPTION and EPILOG further down, so that they reach a reader
in a terminal rather than only one with the file open:

    python3 tools/inslib_cfg.py

Kept here instead, because it is about this file rather than about using
it: a persisted write blocks the board. Most saves cost milliseconds, but
every fourth one has to erase a flash sector, and this part cannot fetch
instructions while its flash is erasing: the board stops for a few
hundred milliseconds and its GNSS and IMU streams lose whatever arrives
meanwhile. That is why the timeout below is generous and why nothing here
persists in a loop.

The board answers on the port the request arrived on, so this tool has to
own the serial port while it runs. inslib_hub.py holds it when it is
running; stop it first.

(c) Jan Zwiener (jan@zwiener.org)
"""

from __future__ import annotations

import argparse
import sys
import time

import inslib_ubx as ux

# A save that has to erase a sector stops the board for a few hundred
# milliseconds, and the acknowledgement only comes after it. Two seconds
# is comfortably past the worst case without hanging a script forever.
ACK_TIMEOUT = 2.0
# A poll is answered out of the run loop with no flash involved.
POLL_TIMEOUT = 1.0
# The state message is periodic rather than polled, so waiting for one
# means waiting out its interval.
INFO_TIMEOUT = 5.0


class CfgLink:
    """One request at a time over a serial port.

    Deliberately not pipelined: the firmware holds a single pending
    command per port and drops a second one that arrives while the first
    is still being answered (it says so in the cmd_dropped counter). One
    request, one answer, is also what makes a failure attributable."""

    def __init__(self, stream):
        # Any object with the pyserial read/write/in_waiting/
        # reset_input_buffer surface, so the tests can drive this against
        # a fake board instead of a port.
        self.ser = stream
        self.framer = ux.UbxFramer()
        # One read can carry several frames, and a multi-frame answer
        # often arrives in a single one. Frames that are parsed but not
        # yet wanted wait here rather than being discarded.
        self.pending = []

    @classmethod
    def open_serial(cls, port, baud):
        try:
            import serial
        except ImportError:
            raise SystemExit("pyserial is required: pip install pyserial")
        try:
            return cls(serial.Serial(port, baud, timeout=0.05))
        except Exception as e:
            raise SystemExit("cannot open %s: %s\n"
                             "(inslib_hub.py owns the port while it runs)"
                             % (port, e))

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _pump(self):
        data = self.ser.read(max(self.ser.in_waiting, 1))
        if data:
            self.pending += self.framer.feed(data)

    def poll(self):
        """Read whatever is on the link into `pending`, asking for nothing.

        For a caller that wants the sensor stream rather than an answer -
        inslib_gui.py plots it between configuration exchanges. It is
        a named method rather than a reach into _pump() so that the two
        uses of this class are both visible from here: one asks and
        waits, the other only listens."""
        self._pump()

    def wait_for(self, ids, timeout):
        """Next class-0x40 frame with one of these message ids.

        Everything else on the link - the IMU stream, the receiver's own
        traffic - is read past rather than treated as an error. Frames of
        interest that arrived in the same read as this one stay queued:
        dropping them would lose the tail of a multi-frame answer, which
        arrives back to back and so almost always shares a read."""
        deadline = time.monotonic() + timeout
        while True:
            for i, (cls, mid, payload, _frame) in enumerate(self.pending):
                if cls == ux.CLS_INSLIB and mid in ids:
                    del self.pending[:i + 1]
                    return mid, payload
            self.pending.clear()
            if time.monotonic() >= deadline:
                return None, None
            self._pump()

    def request(self, frame, ids, timeout):
        # Drop what is already buffered: an answer to the PREVIOUS
        # request sitting in the buffer would be read as the answer to
        # this one, which is the classic way to get a stale reply.
        self.ser.reset_input_buffer()
        self.framer.reset()
        self.pending.clear()
        self.ser.write(frame)
        self.ser.flush()
        return self.wait_for(ids, timeout)

    def valget(self, keys, layer=ux.VALGET_RAM):
        """Every value the board reports for these keys.

        Reads on until the board says no more frames follow, so a
        wildcard over a calibration group comes back whole."""
        mid, payload = self.request(ux.build_valget(keys, layer),
                                    (ux.ID_VALGET_R, ux.ID_CFGACK),
                                    POLL_TIMEOUT)
        if mid is None:
            raise SystemExit("no answer from the board (wrong port or baud?)")
        if mid == ux.ID_CFGACK:
            ack = ux.parse_cfgack(payload)
            raise SystemExit("board refused the request: %s"
                             % (ack["text"] if ack else "malformed answer"))

        values = {}
        while True:
            got = ux.parse_valget(payload)
            if got is None:
                raise SystemExit("malformed value response")
            values.update(got["values"])
            if not got["more"]:
                return values
            mid, payload = self.wait_for((ux.ID_VALGET_R,), POLL_TIMEOUT)
            if mid is None:
                raise SystemExit("the answer stopped part way through "
                                 "(%d values so far)" % len(values))

    def valset(self, items, layers):
        mid, payload = self.request(ux.build_valset(items, layers),
                                    (ux.ID_CFGACK,), ACK_TIMEOUT)
        if mid is None:
            raise SystemExit("no acknowledgement from the board")
        ack = ux.parse_cfgack(payload)
        if ack is None:
            raise SystemExit("malformed acknowledgement")
        return ack

    def reset(self, layers):
        mid, payload = self.request(ux.build_cfgreset(layers),
                                    (ux.ID_CFGACK,), ACK_TIMEOUT)
        if mid is None:
            raise SystemExit("no acknowledgement from the board")
        return ux.parse_cfgack(payload)

    def info(self):
        mid, payload = self.wait_for((ux.ID_CFGINFO,), INFO_TIMEOUT)
        if mid is None:
            return None
        return ux.parse_cfginfo(payload)


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------

def resolve_name(name):
    """A name -> list of keys to ask for.

    A group name expands to a wildcard, which the board turns into every
    item it knows of that group. That is better than the tool guessing
    the item list, because then a firmware with a new key reports it
    without this file having heard of it."""
    upper = name.upper()
    if upper in ux.CFG_KEYS:
        return [ux.CFG_KEYS[upper]]
    if upper in ux.CFG_GROUPS:
        return [ux.group_wildcard(ux.CFG_GROUPS[upper])]
    if upper.startswith("0X"):
        try:
            return [int(upper, 16)]
        except ValueError:
            pass
    raise SystemExit("unknown key or group '%s'\n"
                     "try: %s" % (name, ", ".join(sorted(ux.CFG_GROUPS))))


def _fmt_matrix(rows):
    return ["   [%s]" % "  ".join("%+9.6f" % v for v in row) for row in rows]


def format_value(key, raw):
    """One key rendered for a person to read.

    Calibration records are decoded rather than dumped as hex: the whole
    point of reading them back is to check the numbers."""
    name = ux.cfg_key_name(key)
    size = ux.cfg_key_size(key)
    group = ux.cfg_key_group(key)

    if size != ux.SZ_BLOB:
        return ["%-26s %d" % (name, ux.scalar_from_bytes(raw))]

    if group == ux.GRP_FRAME and len(raw) == ux.LEVERARM_LEN:
        lev = ux.unpack_leverarm(raw)
        return ["%-26s fwd %+.3f, right %+.3f, down %+.3f m"
                % (name, lev[0], lev[1], lev[2])]

    if group == ux.GRP_FRAME and len(raw) == ux.HOUSING_LEN:
        rows = ux.unpack_housing(raw)
        rpy = ux.housing_rpy_deg(rows)
        # The three angles first: nine numbers are what the device needs
        # and the three are what a person can check against the box.
        out = ["%-26s roll %+.2f, pitch %+.2f, yaw %+.2f deg"
               % (name, rpy[0], rpy[1], rpy[2])]
        return out + _fmt_matrix(rows)

    kind = ux.CAL_KIND_OF_GROUP.get(group)
    unit = {"acc": "g", "gyr": "deg/s", "mag": "uT"}.get(kind, "")

    if len(raw) == ux.CAL_POINT_LEN:
        temp_c, rows, bias = ux.unpack_cal_point(raw)
        out = ["%-26s %.2f degC" % (name, temp_c)]
        out += _fmt_matrix(rows)
        out.append("   bias [%s] %s"
                   % (", ".join("%+.6f" % b for b in bias), unit))
        return out

    if len(raw) == ux.CAL_BIASPOLY_LEN:
        t_ref, deg, coeffs = ux.unpack_bias_poly(raw)
        # A record can be replaced but never removed, so a retired fit is
        # a degree marker and not an absent key. Printing it as degree 255
        # with four coefficients reads like a corrupt record instead of
        # the normal state of a table the host has just added a node to.
        if deg == ux.CAL_POLY_NONE:
            return ["%-26s none fitted, the node biases carry the bias"
                    % name]
        out = ["%-26s degree %d about %.2f degC" % (name, deg, t_ref)]
        for axis, c in zip("xyz", coeffs):
            terms = " ".join("%+.6g*dT^%d" % (v, k)
                             for k, v in enumerate(c[:deg + 1]))
            out.append("   b_%s = %s   [%s]" % (axis, terms, unit))
        return out

    return ["%-26s %d bytes: %s" % (name, len(raw), raw.hex())]


def print_values(values):
    for key in sorted(values, key=lambda k: (ux.cfg_key_group(k),
                                             ux.cfg_key_item(k))):
        for line in format_value(key, values[key]):
            print(line)


def print_info(info):
    if info is None:
        print("no state message within %.0f s.\n"
              "The board sends it periodically, so either the link is quiet "
              "or CFG-MSGOUT-CFGINFO is off." % INFO_TIMEOUT)
        return
    print("stored image      : %s"
          % ("sequence %d" % info["cfg_seq"] if info["stored"] else "none"))
    print("live image CRC    : 0x%08X%s"
          % (info["cfg_crc"], "  (UNSAVED CHANGES)" if info["unsaved"] else ""))
    print("APPLY_CAL switch  : imu %s, mag %s"
          % ("on" if info["imu_cal_applied"] else "off",
             "on" if info["mag_cal_applied"] else "off"))
    print("temperature       : imu %.2f degC, mag %.2f degC"
          % (info["temp_imu_c"], info["temp_mag_c"]))

    # Two independent things decide whether a sample actually comes out
    # corrected: the node table has to hold up, and the APPLY_CAL switch
    # has to be on. Reported apart, a valid table with the switch off
    # reads as a working calibration, which is the one thing these lines
    # exist to answer.
    flags = info["cal_flags"]
    for label, kind, valid, clamped, switch in (
            ("accelerometer", "acc", ux.CAL_ACC_VALID, ux.CAL_ACC_CLAMPED,
             info["imu_cal_applied"]),
            ("gyroscope    ", "gyr", ux.CAL_GYR_VALID, ux.CAL_GYR_CLAMPED,
             info["imu_cal_applied"]),
            ("magnetometer ", "mag", ux.CAL_MAG_VALID, ux.CAL_MAG_CLAMPED,
             info["mag_cal_applied"])):
        if not flags & valid:
            state = "nothing applied"
        elif not switch:
            state = "valid, but NOT applied (APPLY_CAL is off)"
        else:
            state = "IN USE"
        if flags & clamped:
            state += ", temperature OUTSIDE the calibrated range"
        print("  %s: %d nodes, %s" % (label, info["npts"][kind], state))

    # The mounting rotation is device wide rather than per triad, so it
    # gets its own line instead of one inside each of the three above.
    if flags & ux.CAL_HOUSING_BAD:
        print("housing rotation  : STORED BUT REFUSED, the record is not a "
              "rotation")
    elif flags & ux.CAL_HOUSING:
        print("housing rotation  : applied (CFG-FRAME-HOUSING)")

    # The live image and the stored one are separate, and `reset` without
    # --flash only empties the live one. Saying so here is what keeps a
    # board that reports no calibration from looking like a board whose
    # calibration was deleted.
    if info["stored"] and not any(info["npts"].values()):
        print("note              : the live image carries no calibration, but "
              "the stored image")
        print("                    (sequence %d) is loaded back at the next "
              "power cycle." % info["cfg_seq"])
        print("                    'reset --flash' is what clears it for "
              "good.")
    if info["cmd_dropped"]:
        print("commands dropped  : %d (a request arrived while one was "
              "still being answered)" % info["cmd_dropped"])


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def parse_assignment(text):
    """KEY=VALUE, with on/off and true/false spelled out for the flags."""
    if "=" not in text:
        raise SystemExit("expected KEY=VALUE, got '%s'" % text)
    name, _, value = text.partition("=")
    keys = resolve_name(name.strip())
    if len(keys) != 1 or ux.cfg_key_item(keys[0]) == ux.ITEM_WILDCARD:
        raise SystemExit("'%s' is a group, and a group cannot be written "
                         "in one go" % name.strip())
    key = keys[0]
    if ux.cfg_key_size(key) == ux.SZ_BLOB:
        raise SystemExit(
            "%s is a calibration record, not a number.\n"
            "Records are written by tools/inslib_calib_gui.py, which has "
            "the measurement they come from." % ux.cfg_key_name(key))

    value = value.strip().lower()
    if value in ("on", "true", "yes"):
        number = 1
    elif value in ("off", "false", "no"):
        number = 0
    else:
        try:
            number = int(value, 0)
        except ValueError:
            raise SystemExit("'%s' is not a number" % value)
    return key, number


def cmd_info(link, _args):
    print_info(link.info())
    return 0


def cmd_get(link, args):
    layer = ux.VALGET_DEFAULT if args.default else ux.VALGET_RAM
    keys = []
    for name in args.names:
        keys += resolve_name(name)
    values = link.valget(keys, layer)
    if not values:
        print("nothing set. Keys with no value are left out of the answer, "
              "so an empty result means none of them carries one.")
        return 0
    print_values(values)
    return 0


def cmd_dump(link, args):
    layer = ux.VALGET_DEFAULT if args.default else ux.VALGET_RAM
    for name in sorted(ux.CFG_GROUPS):
        values = link.valget([ux.group_wildcard(ux.CFG_GROUPS[name])], layer)
        print("--- %s ---" % name)
        if values:
            print_values(values)
        else:
            print("   (nothing set)")
    return 0


def cmd_set(link, args):
    items = [parse_assignment(a) for a in args.assignments]
    layers = ux.LAYER_RAM | (ux.LAYER_FLASH if args.flash else 0)
    ack = link.valset(items, layers)
    if not ack["ok"]:
        where = ("" if ack["key"] == 0
                 else " at %s" % ux.cfg_key_name(ack["key"]))
        print("refused%s: %s (%d of %d applied before it)"
              % (where, ack["text"], ack["detail"], len(items)))
        return 1
    for key, value in items:
        print("%-26s = %d" % (ux.cfg_key_name(key), value))
    print("applied to the live configuration%s"
          % (" and stored" if args.flash else
             " only. It is gone at the next power cycle without --flash"))
    return 0


def cmd_leverarm(link, args):
    """Read or write CFG-FRAME-LEVERARM.

    Its own command rather than a KEY=VALUE assignment: 'set' takes one
    number per key, and this key is a vector that has to be written in
    one piece -- three separate writes leave a window in which the
    filter runs on two axes of the new arm and one of the old."""
    key = ux.leverarm_key()

    if not args.xyz:
        values = link.valget([key])
        raw = values.get(key)
        if raw is None:
            print("CFG-FRAME-LEVERARM         unset -- the antenna is assumed "
                  "to sit on the IMU")
            return 0
        for line in format_value(key, raw):
            print(line)
        return 0

    if len(args.xyz) != 3:
        raise SystemExit("expected three numbers: forward right down, in metres")
    try:
        record = ux.pack_leverarm(args.xyz)
    except ValueError as exc:
        # A units mistake is the expected way to get here, and it deserves
        # the same one line refusal every other bad input gets rather than
        # a traceback.
        raise SystemExit(str(exc))
    layers = ux.LAYER_RAM | (ux.LAYER_FLASH if args.flash else 0)
    ack = link.valset([(key, record)], layers)
    if not ack["ok"]:
        print("refused: %s" % ack["text"])
        return 1
    print("CFG-FRAME-LEVERARM         fwd %+.3f, right %+.3f, down %+.3f m"
          % tuple(args.xyz))
    print("applied to the live configuration%s"
          % (" and stored" if args.flash else
             " only. It is gone at the next power cycle without --flash"))
    return 0


def cmd_reset(link, args):
    layers = ux.LAYER_RAM | (ux.LAYER_FLASH if args.flash else 0)
    ack = link.reset(layers)
    if ack is None:
        raise SystemExit("malformed acknowledgement")
    if not ack["ok"]:
        print("refused: %s" % ack["text"])
        return 1
    if args.flash:
        print("store erased, the board is back to its defaults for good.\n"
              "The calibration is gone with it.")
    else:
        print("live values dropped: the live configuration is back to its "
              "defaults.")
        print("The stored image was NOT touched. It is loaded back at the "
              "next power cycle,")
        print("so the calibration returns with it; 'reset --flash' is what "
              "clears it for good.")
        print()
        print("Until that power cycle, do not use --flash on anything: a "
              "persisted write now")
        print("stores the emptied live image OVER the calibration.")
    return 0


def cmd_keys(_link, _args):
    for name in sorted(ux.CFG_KEYS):
        print("%-26s 0x%08X" % (name, ux.CFG_KEYS[name]))
    return 0


DESCRIPTION = """\
Read, write and reset the sensor board's stored settings.

The board holds TWO configurations, and which one a command means is the
thing to get right:

  LIVE    the image in RAM. What the board corrects its samples with, and
          what `info` and `dump` report.
  STORED  the image in flash. A snapshot of the live one, loaded back
          into it at every power cycle.

Every write lands in the live image. `--flash` additionally copies it to
the stored one, and nothing else reaches flash.
"""

EPILOG = """\
Examples:

  # what is the board doing right now: stored image, calibration, temperature
  python3 tools/inslib_cfg.py --port COM4 info

  # read it all back, calibration records decoded rather than as hex
  python3 tools/inslib_cfg.py --port COM4 dump
  python3 tools/inslib_cfg.py --port COM4 get CFG-CALMAG CFG-MAG-APPLY_CAL

  # stream raw for a calibration recording. Live only, so the correction
  # is back on by itself at the next power cycle
  python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=0
  python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=1

  # a setting that has to survive the power cycle
  python3 tools/inslib_cfg.py --port COM4 set CFG-RATE-MAG_MS=50 --flash

  # live values back to the defaults. The stored image is NOT touched and
  # returns at the next power cycle, calibration included
  python3 tools/inslib_cfg.py --port COM4 reset

  # the factory reset: erase the store. The calibration is gone for good
  python3 tools/inslib_cfg.py --port COM4 reset --flash

  # the key names, no board needed
  python3 tools/inslib_cfg.py keys

A name is a key or a whole group: CFG-RATE reads every item of the rate
group, CFG-CALACC the whole accelerometer calibration table.

After a `reset` without --flash the live image counts as changed, so any
persisted write at all stores the emptied image OVER the calibration.
Power cycle first, or mean `reset --flash` in the first place.

A --flash write blocks the board for up to a few hundred milliseconds and
its data streams lose that much.

This tool owns the serial port while it runs. inslib_hub.py holds it when
it is running; stop that first.
"""


def main(argv=None, link=None):
    p = argparse.ArgumentParser(
        description=DESCRIPTION,
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port, e.g. COM4 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=921600,
                   help="port speed (default: %(default)s)")
    # Not required: without a subcommand argparse prints a one line usage
    # and exits, which is the least useful thing this file can say. The
    # full help with the examples below is what someone typing the bare
    # command is after.
    sub = p.add_subparsers(dest="cmd")

    s = sub.add_parser("info", help="stored image, calibration and temperature")
    s.set_defaults(func=cmd_info)

    s = sub.add_parser("get", help="read keys or whole groups")
    s.add_argument("names", nargs="+", metavar="NAME")
    s.add_argument("--default", action="store_true",
                   help="what a reset would give instead of what is live")
    s.set_defaults(func=cmd_get)

    s = sub.add_parser("dump", help="read every group")
    s.add_argument("--default", action="store_true",
                   help="what a reset would give instead of what is live")
    s.set_defaults(func=cmd_dump)

    s = sub.add_parser("set", help="write KEY=VALUE")
    s.add_argument("assignments", nargs="+", metavar="KEY=VALUE")
    s.add_argument("--flash", action="store_true",
                   help="also persist, so the value survives a power cycle")
    s.set_defaults(func=cmd_set)

    s = sub.add_parser("leverarm",
                       help="read or write the GNSS antenna lever arm, body "
                            "frame FRD (forward right down) in metres, from "
                            "the IMU to the antenna phase centre")
    s.add_argument("xyz", nargs="*", type=float,
                   metavar="FORWARD RIGHT DOWN")
    s.add_argument("--flash", action="store_true",
                   help="also persist, so the value survives a power cycle")
    s.set_defaults(func=cmd_leverarm)

    # "back to the defaults" read as a factory reset, which is the one
    # thing this command does NOT do without --flash. Both halves of the
    # choice are spelled out here, because this is where it is made.
    s = sub.add_parser("reset",
                       help="drop the LIVE values back to the defaults. The "
                            "stored image is untouched and comes back at the "
                            "next power cycle")
    s.add_argument("--flash", action="store_true",
                   help="erase the stored image as well. This is the one that "
                        "clears the calibration, and it cannot be undone")
    s.set_defaults(func=cmd_reset)

    s = sub.add_parser("keys", help="list the known key names, no board needed")
    s.set_defaults(func=cmd_keys)

    args = p.parse_args(argv)

    if getattr(args, "func", None) is None:
        p.print_help()
        return 0

    if args.func is cmd_keys:
        return cmd_keys(None, args)
    if link is not None:                 # injected by the tests
        return args.func(link, args)
    if not args.port:
        raise SystemExit("--port is required for this command")

    link = CfgLink.open_serial(args.port, args.baud)
    try:
        return args.func(link, args)
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
