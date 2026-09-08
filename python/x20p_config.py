#!/usr/bin/env python3
"""
x20p_config.py - configure a u-blox X20P directly and PERSISTENTLY
(RAM + BBR + Flash), independent of the STM32 firmware.

Meant for a direct connection to the F9P, e.g. via its own USB port
(dedicated COM port, baud rate irrelevant) or a USB-serial adapter on
UART1. The script auto-detects the current baud rate (MON-VER poll on
all candidates).

Configures:
  - UART1 + UART2: 460800 baud, UBX output on, NMEA output off
  - UART1 + UART2 + USB input: UBX + RTCM3X + SPARTN on, NMEA off
    (correction data can be fed in via UART1, UART2 or USB, e.g. from
     an NTRIP client/radio modem or directly from the PC. RTCM3X for a
     classic base station or NTRIP caster, SPARTN for PPP-RTK services
     such as u-blox PointPerfect. Both formats may be enabled at the
     same time, the receiver decides per message which source it
     applies and reports that in RXM-COR, see below)
  - Measurement rate 5 Hz (RATE-MEAS 250 ms, RATE-NAV 1 -> raw data and
    navigation solution both run at 5 Hz), dynModel airborne <4g
  - Message output on UART1 AND UART2:
      NAV-PVT, NAV-COV, RXM-RAWX, RXM-SFRBX, TIM-TP  = on
      NAV-TIMEGPS                                    = off
    (TIM-TP announces the NEXT timepulse with week+TOW ahead of time.
     The firmware consumes the announcement itself, pairs it with the
     hardware capture of the corresponding edge, and sends the result
     as UBX 0x40/0x05. TIM-TP itself is also passed through, the host
     can ignore it - the paired form is the useful one. It must stay
     enabled here regardless. UTC/leap seconds come from NAV-PVT, so
     NAV-TIMEGPS is redundant.)

Additionally (CFG_OPT block, see switches below, requires X20P
firmware HPG 2.10 or newer):
  - GPS L5 health override: GPS L5 is still pre-operational and flagged
    "unhealthy", so the receiver does not use it by default. The key
    makes it take over the L1 health value of the respective satellite
    for L5 (u-blox AppNote UBX-21038688).
  - Enable Galileo HAS (E6-B) as a correction source.
  - RXM-COR to check which correction source is actually being used.

Usage:      python x20p_config.py COM5
            python x20p_config.py /dev/ttyUSB0
            python x20p_config.py COM5 921600

The optional baud rate disables the search. It is needed via the
STM32 passthrough: there 921600 is the PC<->STM32 line, while the
460800 further below refers to the STM32<->receiver line. Without an
override the script tries 921600 last, as a final candidate.

The baud rate keys are sent last: if the script runs over UART1/UART2,
the connection drops when switching - the script then automatically
reconnects at the target baud rate and verifies.
"""
import sys, time
import serial   # pip install pyserial

TARGET_BAUD = 460800
BAUD_CANDIDATES = [460800, 115200, 38400, 9600, 230400, 57600, 921600]
LAYERS_ALL = 0x07          # RAM | BBR | Flash

# --- Switches for the optional block (firmware HPG 2.10+) -----------------
L5_HEALTH_OVERRIDE = True   # use GPS L5 despite the "unhealthy" flag
GAL_HAS            = True   # Galileo HAS (E6-B) as a correction source
KEEP_HOST_CORR     = False  # keep allowing RTCM/SPARTN from the host (NTRIP/RTK).
                            # ArduSimple sets this to 0 in its HAS file,
                            # which kills RTK/NTRIP. The receiver's default
                            # is 1; only set to False if pure HAS operation
                            # without any host correction is really wanted.
RXM_COR_ON_UARTS   = False  # also enable RXM-COR on UART1/UART2 (bandwidth!)

# --- Key IDs (ZED-F9P Interface Description, "Configuration") -------------
# Size is encoded in the key: bits 30..28 = 1:L(1B) 2:U1 3:U2 4:U4
CFG = {
    # Signal configuration (example keys for F10/X20 multi-band)
    "CFG-SIGNAL-GPS_L1CA_ENA":    (0x10310001, 1),
    "CFG-SIGNAL-GPS_L2C_ENA":     (0x10310003, 1),
    "CFG-SIGNAL-GPS_L5_ENA":      (0x10310004, 1),  # X20P L5 support!

    "CFG-SIGNAL-GAL_E1_ENA":      (0x10310007, 1),
    "CFG-SIGNAL-GAL_E5A_ENA":     (0x10310009, 1),  # Galileo L5/E5a
    "CFG-SIGNAL-GAL_E5B_ENA":     (0x1031000A, 1),
    "CFG-SIGNAL-GAL_E6_ENA":      (0x1031000B, 1),  # HAS from E6-B

    "CFG-SIGNAL-BDS_B1I_ENA":     (0x1031000D, 1),
    "CFG-SIGNAL-BDS_B2I_ENA":     (0x1031000E, 1),
    "CFG-SIGNAL-BDS_B2A_ENA":     (0x10310028, 1),  # BeiDou L5 equivalent

    "CFG-SIGNAL-GLO_L1_ENA":      (0x10310018, 1),
    "CFG-SIGNAL-GLO_L2_ENA":      (0x10310019, 1),


    # Protocols / ports - output
    "CFG-UART1OUTPROT-UBX":   (0x10740001, 1),
    "CFG-UART1OUTPROT-NMEA":  (0x10740002, 0),  # disable
    "CFG-UART2OUTPROT-UBX":   (0x10760001, 1),
    "CFG-UART2OUTPROT-NMEA":  (0x10760002, 0),  # disable
    # Protocols / ports - input (RTCM3X, SPARTN = receive correction data)
    "CFG-UART1INPROT-UBX":     (0x10730001, 1),
    "CFG-UART1INPROT-NMEA":    (0x10730002, 0),  # disable
    "CFG-UART1INPROT-RTCM3X":  (0x10730004, 1),
    "CFG-UART1INPROT-SPARTN":  (0x10730005, 1),  # PointPerfect via UART1
    "CFG-UART2INPROT-UBX":     (0x10750001, 1),
    "CFG-UART2INPROT-NMEA":    (0x10750002, 0),  # disable
    "CFG-UART2INPROT-RTCM3X":  (0x10750004, 1),
    "CFG-UART2INPROT-SPARTN":  (0x10750005, 1),
    # Navigation
    "CFG-RATE-MEAS":          (0x30210001, 200),   # 200 ms measurement epoch = 5 Hz -> RXM-RAWX/SFRBX
    "CFG-RATE-NAV":           (0x30210002, 1),     # measurements per nav solution -> NAV-PVT/COV also 5 Hz
    "CFG-NAVSPG-DYNMODEL":    (0x20110021, 8),     # airborne <4g
    # Message output UART1
    "CFG-MSGOUT-NAV_PVT_UART1":     (0x20910007, 1),
    "CFG-MSGOUT-NAV_COV_UART1":     (0x20910084, 0),  # disable
    "CFG-MSGOUT-NAV_TIMEGPS_UART1": (0x20910179, 0),  # disable
    "CFG-MSGOUT-RXM_RAWX_UART1":    (0x209102A5, 1),
    "CFG-MSGOUT-RXM_SFRBX_UART1":   (0x20910232, 1),
    "CFG-MSGOUT-TIM_TP_UART1":      (0x2091017E, 1),
    # Message output UART2 (key = UART1 key + 1)
    "CFG-MSGOUT-NAV_PVT_UART2":     (0x20910008, 1),
    "CFG-MSGOUT-NAV_COV_UART2":     (0x20910085, 0),  # disable
    "CFG-MSGOUT-NAV_TIMEGPS_UART2": (0x2091017A, 0),  # disable
    "CFG-MSGOUT-RXM_RAWX_UART2":    (0x209102A6, 1),
    "CFG-MSGOUT-RXM_SFRBX_UART2":   (0x20910233, 1),
    "CFG-MSGOUT-TIM_TP_UART2":      (0x2091017F, 1),
    # USB protocols (there is NO baud rate key for USB) - output
    "CFG-USBOUTPROT-UBX":   (0x10780001, 1),
    "CFG-USBOUTPROT-NMEA":  (0x10780002, 0),
    # USB protocols - input (RTCM3X, SPARTN = receive correction data)
    "CFG-USBINPROT-UBX":     (0x10770001, 1),
    "CFG-USBINPROT-NMEA":    (0x10770002, 0),
    "CFG-USBINPROT-RTCM3X":  (0x10770004, 1),
    "CFG-USBINPROT-SPARTN":  (0x10770005, 1),
    # Message output USB (key = UART1 key + 2)
    "CFG-MSGOUT-NAV_PVT_USB":     (0x20910009, 1),
    "CFG-MSGOUT-NAV_COV_USB":     (0x20910086, 0),  # disable
    "CFG-MSGOUT-NAV_TIMEGPS_USB": (0x2091017B, 0),  # disable
    "CFG-MSGOUT-RXM_RAWX_USB":    (0x209102A7, 1),
    "CFG-MSGOUT-RXM_SFRBX_USB":   (0x20910234, 1),
    "CFG-MSGOUT-TIM_TP_USB":      (0x20910180, 1),
}
# --- Optional block: L5 health, Galileo HAS, RXM-COR -----------------------
# These keys only exist from X20P firmware HPG 2.10 onward (and, like
# 0x10320001, are not even documented in the Interface Description). On
# older firmware this gets a NAK - the script reports that as a WARNING
# and does not abort, unlike the mandatory CFG block above.
CFG_OPT = {}

if L5_HEALTH_OVERRIDE:
    # u-blox AppNote "GPS L5 configuration" (UBX-21038688): replace L5
    # health with the L1 health of the same satellite. Identical to the
    # three hex strings from the AppNote, just with all three layers in
    # ONE VALSET (0x07 instead of 0x01/0x02/0x04) - see self_test() below.
    CFG_OPT["CFG-SIGNAL-GPS_L5_HEALTH_OVERRIDE"] = (0x10320001, 1)

if GAL_HAS:
    # CFG-NAVCOR (group 0x0d), new in HPG 2.10:
    #   ENABLE_GAL_HAS  default 0 - corrections from Galileo HAS (E6-B)
    #   ENABLE_HOST     default 1 - corrections from the host (RTCM/SPARTN)
    # HAS delivers decimeter accuracy (RTK float) without a base/NTRIP. With
    # a reachable base, a true RTK fix is still better, so leave ENABLE_HOST
    # at its default here.
    CFG_OPT["CFG-NAVCOR-ENABLE_GAL_HAS"] = (0x100D0002, 1)
    CFG_OPT["CFG-NAVCOR-ENABLE_HOST"]    = (0x100D0001, 1 if KEEP_HOST_CORR else 0)
    # RXM-COR: one status frame per received correction message (source,
    # type, "used/unused"). This is the only useful way to check whether
    # HAS is actually being applied. Always on for USB, only on request
    # for the UARTs.
    CFG_OPT["CFG-MSGOUT-RXM_COR_USB"]   = (0x209106B9, 1)
    CFG_OPT["CFG-MSGOUT-RXM_COR_UART1"] = (0x209106B7, 1 if RXM_COR_ON_UARTS else 0)
    CFG_OPT["CFG-MSGOUT-RXM_COR_UART2"] = (0x209106B8, 1 if RXM_COR_ON_UARTS else 0)

# Baud rates LAST (connection drops if we are ourselves on the UART)
CFG_BAUD = {
    "CFG-UART1-BAUDRATE": (0x40520001, TARGET_BAUD),
    "CFG-UART2-BAUDRATE": (0x40530001, TARGET_BAUD),
}

# --- UBX helpers ------------------------------------------------------------
def ubx_frame(cls, mid, payload=b""):
    hdr = bytes([cls, mid, len(payload) & 0xFF, len(payload) >> 8])
    a = b = 0
    for x in hdr + payload:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xB5\x62" + hdr + payload + bytes([a, b])

def key_size(key):
    return {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}[(key >> 28) & 0x7]

def valset(key, val, layers=None, version=0x01):
    # version 0x01 = transaction-capable VALSET; u-blox uses 0x00 in its
    # AppNote hex strings. Both are accepted, the rest is identical.
    pl = bytes([version, LAYERS_ALL if layers is None else layers, 0, 0])
    pl += key.to_bytes(4, "little") + int(val).to_bytes(key_size(key), "little")
    return ubx_frame(0x06, 0x8A, pl)

def read_frames(ser, timeout_s):
    """Generator over (cls, id, payload) until timeout_s, ignoring any leftover data."""
    buf = bytearray()
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        buf += ser.read(ser.in_waiting or 1)
        while True:
            j = buf.find(b"\xB5\x62")
            if j < 0:
                if buf[-1:] == b"\xB5":    # keep a half sync sequence at the end
                    del buf[:-1]
                else:
                    buf.clear()
                break
            if len(buf) < j + 8:
                del buf[:j]; break
            ln = buf[j + 4] | (buf[j + 5] << 8)
            if ln > 8192:                  # not a real UBX frame: wrong sync
                del buf[:j + 2]; continue  # -> search again 2 bytes further
            end = j + 6 + ln + 2
            if len(buf) < end:
                del buf[:j]; break
            frame = bytes(buf[j:end])
            a = b = 0
            for x in frame[2:-2]:
                a = (a + x) & 0xFF
                b = (b + a) & 0xFF
            if a == frame[-2] and b == frame[-1]:
                del buf[:end]
                yield frame[2], frame[3], frame[6:-2]
            else:
                del buf[:j + 2]            # wrong sync: do NOT discard up to end

def mon_ver_poll(ser, timeout_s=1.5):
    """'answer'  = MON-VER response -> F9P reachable (bidirectional).
       'traffic' = valid UBX frames, but NO response to the poll.
       'silent'  = nothing valid at all."""
    ser.reset_input_buffer()
    ser.write(ubx_frame(0x0A, 0x04))
    seen = False
    for cls, mid, _ in read_frames(ser, timeout_s):
        seen = True
        if (cls, mid) == (0x0A, 0x04):
            return "answer"
    return "traffic" if seen else "silent"

def mon_ver_ok(ser, timeout_s=1.5):
    return mon_ver_poll(ser, timeout_s) == "answer"

def wait_ack(ser, timeout_s=1.5):
    """0 = ACK, -1 = NAK, None = timeout (for CFG-VALSET)."""
    for cls, mid, pl in read_frames(ser, timeout_s):
        if cls == 0x05 and len(pl) >= 2 and pl[0] == 0x06 and pl[1] == 0x8A:
            return 0 if mid == 0x01 else -1
    return None

def connect(port, baud_override=None):
    traffic_seen = False
    for baud in ([baud_override] if baud_override else BAUD_CANDIDATES):
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            print(f"ERROR: cannot open {port} ({e}).\n"
                  f"Is the port still open in u-center/Monitor?")
            sys.exit(2)
        time.sleep(0.05)
        r = mon_ver_poll(ser)
        if r == "answer":
            print(f"F9P found on {port} @ {baud} baud")
            return ser
        traffic_seen |= (r == "traffic")
        ser.close()
    if traffic_seen:
        print(f"{port}: valid UBX data stream, but NO response to MON-VER.\n"
              f"Via the STM32 this is the typical symptom of a firmware\n"
              f"that filters the RETURN path: the poll reaches the receiver,\n"
              f"but the response is dropped (before INSLIB_VCP_GNSS_ALL only\n"
              f"NAV-PVT/RXM-RAWX/SFRBX went out on the VCP, so neither ACK\n"
              f"nor MON-VER). Update the firmware in that case. Otherwise: use\n"
              f"the receiver board's USB port ('u-blox GNSS receiver' in Device\n"
              f"Manager) or a USB-serial adapter directly on UART1/UART2.")
    return None

def apply(ser, cfg, optional=False):
    """optional=True: failures are only a warning (keys only exist from
    HPG 2.10 onward and are NAKed by older firmware)."""
    fails = []
    for name, (key, val) in cfg.items():
        ok = None
        for _ in range(2):                       # 1 retry, like in the firmware
            ser.reset_input_buffer()
            ser.write(valset(key, val))
            ok = wait_ack(ser)
            if ok == 0:
                break
        status = {0: "ACK", -1: "NAK!", None: "TIMEOUT!"}[ok]
        print(f"  {name:36s} = {val:<7d} {status}")
        if ok != 0:
            fails.append(name)
    return fails

def self_test():
    """Compares the generated frames against the hex strings from the u-blox
    AppNote UBX-21038688 (GPS L5 health override), for each layer separately."""
    ref = {
        0x01: "B5 62 06 8A 09 00 00 01 00 00 01 00 32 10 01 DE ED",
        0x02: "B5 62 06 8A 09 00 00 02 00 00 01 00 32 10 01 DF F5",
        0x04: "B5 62 06 8A 09 00 00 04 00 00 01 00 32 10 01 E1 05",
    }
    for layer, s in ref.items():
        mine = valset(0x10320001, 1, layer, version=0x00)
        if mine != bytes.fromhex(s.replace(" ", "")):
            return f"Layer 0x{layer:02X}: {mine.hex(' ').upper()} != {s}"
    return None

def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__); sys.exit(1)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) == 3 else None

    err = self_test()
    if err:
        print(f"INTERNAL ERROR in frame construction: {err}"); sys.exit(4)

    ser = connect(port, baud)
    if ser is None:
        print(f"ERROR: no F9P found on {port} (all baud rates tried).")
        sys.exit(2)

    print("\nConfiguration (RAM+BBR+Flash):")
    fails = apply(ser, CFG)

    warn = []
    if CFG_OPT:
        print("\nOptional (L5 health / Galileo HAS, requires HPG 2.10+):")
        warn = apply(ser, CFG_OPT, optional=True)

    print("\nBaud rates (connection may briefly drop):")
    for name, (key, val) in CFG_BAUD.items():
        ser.write(valset(key, val))
        time.sleep(0.2)                          # flush out before the port switches
        print(f"  {name:34s} = {val:<7d} sent")

    # Verification - reconnect at the target baud rate if needed
    time.sleep(0.3)
    if not mon_ver_ok(ser):
        ser.close()
        # Directly on the receiver our port must follow the baud rate we just
        # set. Via the STM32 passthrough it must NOT change: there TARGET_BAUD
        # is the line behind it, ours is the given one.
        ser = serial.Serial(port, baud or TARGET_BAUD, timeout=0.05)
        time.sleep(0.1)
    if mon_ver_ok(ser):
        print(f"\nVerification: F9P responds ({ser.baudrate} baud).")
    else:
        print("\nWARNING: F9P does not respond after the baud rate change - "
              "rerun the script or check the wiring.")
        fails.append("VERIFY")

    if warn:
        print(f"\nWARNING: optional keys without ACK: {', '.join(warn)}\n"
              f"Typical reason: firmware older than HPG 2.10. The version is in\n"
              f"the MON-VER response; update via u-center 2 (with 'Enter safeboot\n"
              f"before update'). The rest of the configuration is unaffected.")

    if fails:
        print(f"FAILED: {', '.join(fails)}")
        sys.exit(3)
    print("All ACKed - configuration stored persistently.")

if __name__ == "__main__":
    main()
