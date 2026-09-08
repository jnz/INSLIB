# tools/ - host-side utilities

## The shape of it

A serial port has exactly one owner. That single fact decides how these
tools fit together: a receiver that opens the device makes recording the
same session impossible, and two recorders cannot coexist at all.

So one tool owns the device and everything else speaks UDP.

```
 corrections (str2str) ──UDP:29797─►┐────────────────────.
                                    │                    ├──► inslib_<stamp>.ubx
 odometry ─────────────UDP:29798───►│   inslib_hub.py    ├──► _timesync.csv, _speed.csv
 (inslib_obd_speed.py)              │  owns the COM port │
                                    │                    ├──UDP:29800──► insrcv ──► PlotJuggler
                                    │                    │                     └──► MAVLink (GCS)
                       serial ◄────►│    e.g. COM4       │
                       (corr out)   └────────────────────┘
                       (UBX in)
```

`inslib_replay_udp.py` puts a recorded `.ubx` on the same UDP port at the
original pace, so `insrcv` cannot tell a replay from a live session.

## The four jobs

| Job | Command |
|-----|---------|
| Record a session | `python inslib_hub.py COM4` |
| Send corrections and record | same, plus `str2str ... -out udp://localhost:29797` |
| Record **and** process live | same, plus `./build/insrcv --udp-port 29800` |
| Process live, write nothing | `python inslib_hub.py COM4 --no-capture` plus `./build/insrcv --udp-port 29800` |
| Replay a recording into the filter | `python inslib_replay_udp.py cap.ubx` plus `./build/insrcv --udp-port 29800` |
| Turn a recording into a dataset | `python inslib_convert_ubx_to_csv.py cap.ubx --outdir datasets/mydrive` |

## inslib_hub.py - the serial-port owner

Forwards UDP corrections to the MCU's serial link, captures the return
channel, republishes it over UDP, and injects odometry. The forwarding is
byte for byte and thus format-agnostic: RTCM3 from a base station or NTRIP
caster and SPARTN from a PPP-RTK service such as u-blox PointPerfect take
the same path, as long as the matching `CFG-*INPROT-*` key is enabled on
the receiver.

```sh
python inslib_hub.py COM4
python inslib_hub.py COM4 --no-capture                  # live only
python inslib_hub.py COM4 --fanout 127.0.0.1:29800,192.168.1.5:29800
python inslib_hub.py COM4 --fanout ''                   # no fan-out
python inslib_hub.py /dev/ttyACM0 --baud 460800         # USB UART on UART2
python inslib_hub.py --help
```

Per run, timestamped so a second run cannot overwrite the first:
`inslib_<stamp>.corr`, `.ubx`, `_timesync.csv`, `_speed.csv`.

**Fan-out.** The return channel is republished as UDP datagrams carrying
**whole UBX frames only** (see `inslib_protocol.md`). A lost datagram then
costs exactly the frames inside it rather than also desynchronising the
frame that straddled the boundary. Sending is best effort throughout: a
consumer that is not running, is slow, or goes away must never slow down
or stop the recording.

**Odometry completion.** `0x40/0x80` frames arriving on the second UDP
port carry the producer's half: host wall clock, speed, uncertainty,
delay. The hub adds `t_us` on the MCU timebase, sets `T_US_VALID`, and
injects the completed frame into both the capture and the fan-out.
Everything the producer wrote is kept verbatim, so the mapping can be
redone offline from `timesync.csv` without the CSV beside the capture.

The hub is the only process that sees both clocks -- it reads the IMU
stream and receives the host-stamped frames -- so a consumer gets one
stream on one clock. A sample that cannot be mapped yet (no IMU stream)
is recorded **unstamped** rather than dropped: `insrcv` ignores it,
an offline consumer can still place it.

**Status line.** One line, repainted every second:

```
IMU  804 Baro 25.0 RAWX  4.0 SFRBX  1.0 RTCM  8.0 | 3D      sv24 Float sa0.05m/s T | RX 682.7KB/s 400.0MB |  50.0km/h
```

The rates are per second (`RAWX`/`SFRBX` are the PPK (Post Processed
Kinematics) inputs, `RTCM` the valid correction frames going out to the
receiver), then the fix with satellite count, RTK carrier solution and the
receiver's speed accuracy (`sa`, NAV-PVT `sAcc`, `--` without a velocity
solution), the time-pulse state (`T` = a pulse carried a GPS time, `p` = pulses
without one, `-` = no pulse), the return-channel throughput and the odometry
speed (`--` once none has arrived for a few seconds). A stream that stopped
reads as `0.0`.  Everything constant -- the RTCM3X-input keys, the byte and
packet totals -- is printed once instead, on startup or in the closing summary.

**PPK check.** Eight seconds in, a one-shot verdict on whether `RXM-RAWX`
and `RXM-SFRBX` are actually arriving. Without both, the capture cannot be
post-processed at all -- worth learning on the driveway, not afterwards.
The RTCM3 message types seen are grouped (ARP / ephemeris / observations)
in the closing summary, so a correction stream missing the base position
or the ephemeris is visible as such.

**timesync.csv** pairs the three clocks in play once a minute: the MCU's
free-running timer (u64 microseconds, taken as received), the host clock,
and GPS time from the firmware's `0x40/0x05` time sync frames. Raw and
unfitted on purpose; the fit belongs in post-processing where the whole
session is available and bad pairs can be rejected.

**speed.csv** logs the vehicle speed with both the host timestamp the
producer stated *and* an MCU time derived from it. The derived value never
replaces the raw one, so a mapping that turns out to be off can be
recomputed from `timesync.csv` without repeating the drive.

**A dropped serial link does not end the recording.** Reopening is retried
fast for five seconds and then indefinitely at a calmer rate, while UDP,
the captures and the fan-out keep running -- a loose USB connector costs
the IMU stream for the duration and nothing else. Each reconnect starts a
new `segment` (a column in both CSVs). A device that *restarted* during
the gap is visible as such -- `t_us` starts over from zero -- and the
running restart count is a column too, so a reader can tell one continuous
timeline from two.

## insrcv.c - live receiver (C)

Binds a UDP port, drives `nav_suite`, publishes to PlotJuggler over
UDP/JSON and, on request, to MAVLink. Also the reference for porting the
host-side glue onto an embedded target.

```sh
make insrcv
./build/insrcv                                   # listen on 0.0.0.0:29800
./build/insrcv --udp-port 29800 --udp-bind 127.0.0.1
./build/insrcv --raw-only                        # raw sensors, no filter
./build/insrcv --config config.yaml              # noise model + IMU calibration
./build/insrcv --mavlink                         # + MAVLink 2 on udp:14550
./build/insrcv --help                            # all options
```

Then point PlotJuggler at UDP/JSON on 127.0.0.1:9870 (the default).

It never opens a serial port. Recording and live processing have to be
possible at the same time, and one owner per port means the receiver
cannot be that owner. It consumes the protocol in `inslib_protocol.md`
and nothing else.

**Status line.** Repainted once a second (`--stats-sec`), on one line,
built on the same rule as the hub's -- rates and current state live,
totals in the closing summary:

```
  632s | imu 804 baro 25.0 gnss 4.0 odo  4.0@13.9m/s ppsT | |a|9.88 gz+12 alt43.0 | 3D 24sv FIX 0.15/0.24m 0.05m/s ZUPT
```

Sensor rates are in Hz and counted over the interval, so a stream that
stopped reads `0` (the IMU rate is deliberately *not* the device-clock
rate, which would freeze at its last value on a dead link). Then the
receiver's fix: type, satellites, RTK carrier solution (`float` / `FIX`)
and the hAcc/vAcc/sAcc that gate GNSS fusion (position accuracy in
metres, then speed accuracy) -- omitted without a position fix, where
they are the receiver's "invalid" sentinel rather than a number. `|a|`, `alt` and `|m|` are the sensor sanity checks; `gz` appears
once the unit is actually turning.

Counters only appear once they are not zero -- `drop`, `gnss-off`,
`baro-off`, `odo-stale`, `decl`, `crc`/`resync`/`badlen` -- so anything
visible there is worth reading. Session totals (samples per sensor, the
measured device-clock IMU rate, framing faults) are printed once on
Ctrl-C.

**Magnetometer.** `0x40/0x06` frames are decoded, counted and published
always, and fused only when `mag: enable` says so. It is the one sensor
here that is opt-in, because its raw output is not a heading until it has
been calibrated against the platform it is bolted into: an uncalibrated
hard iron of 15 µT against a 48 µT field is tens of degrees of yaw.
`mag: misalignment` / `fixed_bias` (soft iron, hard iron and the
alignment onto the IMU, all measured by `inslib_calib_gui.py`),
`mag: stddev_ut` and `mag: estimate_bias` are read from the same
`config.yaml` the replay harnesses use.

The stats line and the raw overlay both carry the magnetometer's own
sanity check, the exact counterpart of `|a|` against 9.81:

```
|m|48/49
```

left is the calibrated magnitude, right is what the World Magnetic Model
expects at the receiver's position. A calibrated magnetometer reads one
number whichever way the unit is turned, so a left-hand number that moves
as the platform turns is a calibration that is not done (in PlotJuggler:
`sensor/mag/norm_ut`, `norm_raw_ut` and `expected_ut`).

The stats line also carries `hdgmag123`, a tilt-compensated heading off
the magnetometer, after the `mag:` calibration (hard/soft iron) and so
the direction the filter is handed. It is relative to magnetic north, no
declination applied, so against a phone compass set to true north it is
off by the local declination. Levelled with the latest IMU sample rather
than ins/ahrs, so it is there from the first magnetometer frame, before
either filter has initialised. The raw overlay carries the
calibrated/uncalibrated pair the same way it does for magnitude:
`sensor/mag/heading_deg` and `heading_raw_deg`.

**The magnetic model needs an epoch**, and takes the first it can get:
`mag: wmm_year` if the config states one, else GPS time from the
`0x40/0x05` pulses, else the UTC date NAV-PVT carries once the receiver
has decoded one (`validDate`), else the host clock. The last one is why a
session indoors, with no fix, no time pulse and no dated epoch, still gets
declination instead of silently referencing magnetic north. Which source
was used is in the log line:

```
[insrcv] magnetic model at 48.137 11.575 (from GNSS fixType 3), epoch 2026.61 (from host clock)
```

A better source arriving later re-applies the model right away rather than
waiting for the 25 km refresh, and logs the line again -- a receiver needs
tens of seconds to decode the date, and without that the whole session
would stay on whatever the host clock said.

**Odometry.** `0x40/0x80` frames are fused as `ins`'s absolute-speed
aiding (REQ-NAV-068). A sample's age at its epoch is derived from its own
`t_us`, so the residual is anchored that far back in the filter's state
history. A small *negative* age is normal and is fused at delay 0: the
odometry delay is measured against the host clock while the IMU stream
arrives with a transport latency of its own, so the effective age lands
near zero with jitter on both sides. The `REVERSE` flag is decoded,
published and counted but **not** acted on -- see `inslib_protocol.md`.

### MAVLink output (`--mavlink`)

Off by default, because nothing listens on `udp:14550` unless you started
something. With it on, any ground station or log analyser pointed at that
port gets the message set `python/replay.py --mavlink` produces, at the
same rates, plus `GPS_RAW_INT`:

| message | rate | carries |
|---|---|---|
| `HEARTBEAT` | 1 Hz | so a GCS keeps the system on screen |
| `ATTITUDE`, `ATTITUDE_QUATERNION` | 50 Hz | the arbitrated attitude, both spellings |
| `LOCAL_POSITION_NED` | 50 Hz | position and velocity in the filter's own NED frame |
| `GLOBAL_POSITION_INT` | 20 Hz | lat/lon/height |
| `GPS_RAW_INT` | 5 Hz | the receiver's own fix: satellite count, fix type, accuracies |
| `ALTITUDE` | 20 Hz | the vertical channel on its own |
| `HIGHRES_IMU` | 50 Hz | the nav-frame acceleration and the body rates |
| `EKF_STATUS_REPORT` | 10 Hz | real flags and real variances from the covariance |
| `NAMED_VALUE_FLOAT` | 5 Hz | the sub-filter breakdown and its 1-sigma |
| `STATUSTEXT` | on change | the blocked reason, in words |

```sh
./build/insrcv --mavlink                              # udp:14550 on this host
./build/insrcv --mavlink --mav-ip 192.168.1.20        # a GCS on another machine
./build/insrcv --mavlink --mav-subfilter-hz 1         # thin out NAMED_VALUE_FLOAT
```

`ALTITUDE` is the one that makes a baro-only run visible: the two
position messages need a horizontal solution, so in `ATTITUDE_ONLY` mode
they carry nothing while `baro_alt` is producing a perfectly good height.

`GPS_RAW_INT` is the receiver, not the filter: it is the last NAV-PVT
epoch as it arrived, so `satellites_visible` (NAV-PVT `numSV`) and
`fix_type` stay visible even while the filter refuses every fix, and they
keep flowing with `gnss.enable: 0`. `fix_type` is the u-blox `fixType`
mapped onto `MAV_GPS_FIX_TYPE`, with `carrSoln` folded in, so an RTK
float/fixed solution shows as one. An epoch with `gnssFixOK` clear goes
out as `NO_FIX` -- it carries a position, but not one anything should act
on, which is the verdict the filter's own gate reaches too. `fixType` 1
(dead reckoning only) and 5 (time only) have no MAVLink spelling and map
to `NO_FIX` as well. `eph`/`epv` are `hAcc`/`vAcc` in cm, saturating to
65535 ("unknown") where the receiver reports its own "no idea".

Whether the filter is actually **fusing** GNSS is a different question,
and it rides in the `NAMED_VALUE_FLOAT` block: `gnss_used` is 1 while
`ins` has accepted a fix within the last 3 s and 0 otherwise, so a run
whose fixes all fail the accuracy gate shows `GPS_RAW_INT.fix_type` 3 and
`gnss_used` 0. `gnss_nsat` repeats the satellite count there so it is
plottable without a `GPS_RAW_INT` decode. Both are live-only: the CSV
replay path carries no satellite count at all, which is why
`python/replay.py --mavlink` has neither them nor `GPS_RAW_INT`.

Two things a consumer should know. `GLOBAL_POSITION_INT.alt`,
`GPS_RAW_INT.alt` and `ALTITUDE.altitude_amsl` are declared as mean sea
level but carry the **ellipsoidal** height, because there is no geoid
model in this library. And `EKF_STATUS_REPORT` is an *ardupilotmega*
message rather than a common one, so a decoder needs that dialect
(pymavlink's default has it).

The framing is `tools/mini_mavlink.h`, a MAVLink 2 encoder for exactly
these eleven messages. The generated MAVLink C library would be a
submodule and a build dependency for a fixed, small message set, and the
point of `insrcv.c` is to be portable host-side glue.

### Config file (`--config`)

`--config` reads a `config.yaml` subset shared with `python/replay.py`.
The file is applied where it appears on the command line, later options
override it.

A key `insrcv` does not consume is **named on the console** and then
ignored -- a tuning value that silently has no effect is
indistinguishable from one that had no effect. It implements the filter
side of the shared schema in full: `imu:`, `gnss:` (including the
covariance conditioning), `baro: stddev_m` / `acc_bias_init_mps2`,
`init_stddev:`, `init_hint:`, `free_inertial_start:`, `mag:`, and the
top-level filter switches. Still replay-only, because this receiver has
no counterpart for them: `ahrs:` (its ARS/AHRS templates are derived from
`imu:`), the rest of `baro:`, `automotive_mode`, and the harness's own
bias/init windows. The keys that describe the replay harness rather than the filter
(`score:`, `inputs:`, and the `name`/`aiding`/`init` labels) pass
unremarked, so pointing `insrcv` at a dataset `config.yaml` does not
bury the real warnings. The count is repeated in the config summary line:

```
[insrcv] config: "imu.pos_pred_stddev_m_sqrts" not used by insrcv (typo, or a replay-only key)
[insrcv] config foo.yaml: gyr_psd=5e-07 acc_psd=1e-05 imu-calib=off, 1 key(s) ignored (see above)
```

### Starting the 3D filter without GNSS (`free_inertial_start`)

Normally the 3D solution waits for the first fix, because without one
there is no origin to navigate away from. This section lets you supply
that origin instead -- *"I am here, to within this much"* -- so the filter
comes up on the IMU alone and coasts on the auto-ZUPTs and the barometer:

```yaml
free_inertial_start:
  enable: 1
  lat_deg: 49.0069
  lon_deg: 8.4037
  height_m: 118.0
  stddev_m: 2.0     # how well you know that point, 1-sigma

gnss:
  enable: 0         # optional: exclude GNSS outright (see below)
  init_dwell_sec: 1.0
```

The position is **offered as a measurement only until the filter has
bootstrapped from it**, then the receiver goes quiet -- a source that kept
repeating the same point would pin the solution to it instead of dead
reckoning. The stats line counts the offers as `decl=N`, with a `*` while
they are still being made.

While the declaration stands, a GNSS fix **wider than it** is kept out of
the filter and counted as `decl=N(-M)`. Indoors the receiver keeps
producing fixes tens of metres wide, and such a fix does not just carry
less than the declaration -- it also **resets the 3D entry dwell** the
declaration is in the middle of earning, so letting it through means the
filter never starts at all. The first fix that is at least as good (judged
on NAV-COV, or NAV-PVT's `hAcc`) hands the job over for good:

```
free-inertial start: ignoring GNSS wider than the declared +-2.0 m while starting up
free-inertial start: 3D solution up, declared position withdrawn (3 offer(s))
   ...  decl=3(-1)
```

Those fixes are only held back *while starting up*. Once the filter is in
3D they reach it again and its own gates decide; a stream of wide fixes
will then keep it in `COASTING` rather than `FULL`.

**`enable: 1` is a developer mode and behaves like one: once the 3D
filter is up, it stays up.** Coasting without absolute aiding is not a
fault here, it *is* the mode, so both mechanisms that would otherwise end
the run are taken out of the loop and the receiver says so at startup:

- **`max_deadreckoning_sec` is ignored** (`allow_unlimited_deadreckoning`
  forced on). A budget would stop the filter mid-experiment for doing
  exactly what it was asked to do.
- **the 3D exit gate is off** (`gnss.stop_disable` forced on). A run of
  poor-but-valid fixes -- 20 m indoors, a canyon, a tunnel mouth -- makes
  `ins` conclude after `gnss.stop_dwell_sec` that its aiding degraded and
  re-arm (REQ-NAV-052), ending the run for no good reason.

What is *not* switched off: the 3D entry gate, and every fix is still
judged on its own accuracy before it may fuse. GNSS keeps its power to
**correct** the solution, it only loses its power to **end** it.

### Excluding GNSS outright (`gnss: enable: 0`)

Sometimes the point is to see what the IMU does *without* being corrected.
`gnss.enable: 0` keeps decoding, counting and publishing the receiver --
so the same recording stays comparable against a run with GNSS on -- but
nothing derived from it reaches the filter. Dropped fixes are counted as
`gnss-off=N` in the stats line.

That includes the magnetic model: with GNSS excluded it takes its anchor
from `free_inertial_start`'s declared position instead of from a fix. Its
*epoch* still comes from the receiver where the stream carries one -- the
`0x40/0x05` time pulse or NAV-PVT's UTC date -- because a date is a clock
reading and not a position measurement; `mag.wmm_year` and the host clock
remain the fallbacks.

### NAV-PVT fix quality: two consumers, two thresholds

`fixType` is read by two things that need very different amounts of it.

**The filter gets a 3D fix or nothing** (`fixType` 3, or 4 = GNSS+DR, and
`gnssFixOK` set). Everything else is counted as `nofix=N` in the stats
line and never reaches `ins`. Indoors an F9P keeps emitting NAV-PVT at the
full rate with `fixType 0`, `hAcc 0xFFFFFFFF` (4294967 m) and
`sAcc 999 m/s` -- that is the receiver saying *nothing*, not saying
*something imprecise* -- and a 2D fix has no height to give at all.
Handed to the filter as position measurements they used to trip the 3D
**exit** gate after `gnss.stop_dwell_sec`, so the filter concluded its
aiding had degraded and dropped out of 3D (REQ-NAV-052), ending a
dead-reckoning run that nothing had gone wrong with. The offline
converter has always filtered on `fixType` (`--min-fix-type`); this is
the live path catching up.

**The magnetic model takes any fix with a position** (`fixType` >= 2). It
only needs to know where on earth it is, to look up declination and the
NED reference field; those vary on the scale of degrees of latitude, so a
2D fix hundreds of metres out is worth exactly as much as an RTK one.
Refusing it would leave a magnetometer AHRS referencing *magnetic* north
-- a real heading error of a few degrees in Europe, far more further
north -- for no gain. The model is re-looked-up when the platform has
moved 25 km:

```
[insrcv] magnetic model at 49.007 8.404, epoch 2026.02 (fixType 2 is good enough for this)
```

Its epoch comes from the receiver too, because it knows the date better
than a config file does and a stale hand-typed year costs about 0.1 deg of
declination per year: GPS time from the stream (`0x40/0x05`) first, else
the UTC date in NAV-PVT itself, which is the only date a capture without
time pulses carries. `mag.wmm_year` overrides both for a session that gets
neither.

The blocked-reason line tells the two no-aiding cases apart:

```
3D filter off: no GNSS/position measurements at all             # nothing arrives - antenna? config?
3D filter off: the receiver has no 3D fix (NAV-PVT arrives without one)   # arrives, empty - go outside
```

Two limits are inherent, not incidental:

- **Heading is not observable.** Without GNSS course and without a
  magnetometer nothing measures yaw, and at standstill a MEMS gyro cannot
  find north either (earth rate is far below its bias stability). The
  track gets a shape, not a direction.
- **Horizontal position drifts** with nothing on board to correct it. The
  barometer holds the vertical channel and the ZUPTs hold velocity and the
  biases while the platform stands still -- that is what makes the run
  useful, and it is also all of it.

So `max_deadreckoning_sec` still applies: when the budget expires the
suite falls back to `ATTITUDE_ONLY`, and the first real fix re-acquires
normally. Combining this with `allow_unlimited_deadreckoning: 1` is
refused rather than silently resolved -- the two ask for opposite things.
Keep `stddev_m` at or below `gnss.start_max_horizontal_pos_stddev_m`,
otherwise the declaration cannot pass the 3D entry gate and the filter
never starts.

The same section works in `replay.c` and `replay.py` with identical
semantics, so a recording made this way replays the same way -- see the
design document's *Free-inertial mode* section.

### Building and debugging on Windows

The Makefile drives gcc, which typically Windows does not have on PATH.
Rather than failing with a bare "gcc: command not found" from inside a
recipe, `make` checks up front and prints how to put a MinGW-w64 / MSYS2
toolchain on PATH (many MinGW-w64 builds ship a `mingwvars.bat` /
`mingwvars.sh` for exactly that, MSYS2 has its "MinGW 64-bit" shell).

For VS Code: build tasks and the debugger inherit PATH when VS Code
starts, so add the gcc (MinGW) toolchain's `bin\` directory to your **user
PATH** once (Windows Settings > "Edit environment variables for your
account") and restart VS Code. Then:

- `.vscode/tasks.json` has **make insrcv**
- `.vscode/launch.json` has **Debug insrcv**, which runs that task first.
  Start `inslib_hub.py` (or `inslib_replay_udp.py`) alongside it to give
  it something to receive.

## inslib_replay_udp.py - replay a capture into the receiver

```sh
python inslib_replay_udp.py capture.ubx
python inslib_replay_udp.py capture.ubx --speed 4        # 4x real time
python inslib_replay_udp.py capture.ubx --start 120 --duration 30
python inslib_replay_udp.py capture.ubx --max-gap 2      # compress idle gaps
```

The capture carries no host timestamps, only the MCU's free-running
counter, so that counter is the clock: each frame goes out when its `t_us`
says it should. `--speed 1` is the only setting where a replay behaves
exactly like the live session it came from; faster compresses the timeline
and a consumer that throttles on the host clock (`insrcv` does) then plots
fewer samples per unit of capture time.

A recording stitched across a USB dropout carries multi-second holes, and
replaying one faithfully looks exactly like a hung tool. Gaps over a
second are therefore announced as they are reached, and `--max-gap`
compresses them.

## inslib_obd_speed.py - OBD-II vehicle speed

Polls speed (mode 01, PID 0x0D) from an ELM327-style dongle and publishes
it as UBX `0x40/0x80` over UDP to the hub, optionally logging every poll
to CSV. Feeds `ins`'s absolute-speed aiding (REQ-NAV-068).

"OBD dongle" is three different devices, and the tool handles all three.
Which one you have is mostly a question of whether it was built to work
with an iPhone: Apple does not allow generic Bluetooth Classic SPP for
uncertified accessories, so iOS-capable dongles went **BLE** or **WiFi**
from roughly 2014 on, while Android-only ones stayed on **Classic**.

```sh
python inslib_obd_speed.py --scan                      # which link type is it?
python inslib_obd_speed.py --wifi 192.168.0.10             # WiFi (TCP)
python inslib_obd_speed.py --serial COM5 --csv obd.csv     # Classic/SPP
python inslib_obd_speed.py --serial /dev/rfcomm0           # ... on Linux
python inslib_obd_speed.py --ble 11:22:33:44:55:66         # BLE/GATT
python inslib_obd_speed.py --wifi 192.168.0.10 --no-udp --csv obd.csv  # log only
```

`--scan` lists BLE advertisers and serial ports: a Classic/SPP dongle
appears as a serial port once the OS has paired it, a BLE one only as an
advertiser. A **WiFi** dongle appears in neither, because it is an access
point — join its WLAN, then `--wifi HOST[:PORT]` (port defaults to 35000,
the usual address is `192.168.0.10`).

Needs `pyserial` for `--serial` and `bleak` for `--ble`/`--scan`; `--wifi`
uses the standard library only.

Every sample carries a per-sample 1-sigma (the 1 km/h quantisation of PID
0x0D, 0.080 m/s) and an estimated age (half the measured round trip plus a
constant for the ECU's own update interval, `--ecu-delay-ms`). The
*systematic* speedometer scale error is deliberately not folded into that
sigma — it is a property of the vehicle, not of the sample, and belongs in
the filter's `speed_scale` / `speed_stddev_rel`.

Before polling starts the tool sends one `0100` handshake with a long
timeout (`--search-timeout`): with `ATSP0` the first request to the vehicle
runs the protocol search, which takes seconds and would otherwise expire
every poll timeout in turn. The reply is printed verbatim, so a silent bus
is visible immediately — `UNABLE TO CONNECT` or a timeout here almost
always means the ignition is off (the engine need not run) or the dongle
is not fully seated. During polling the last non-data reply is shown in
the status line for the same reason.

BLE dongles do not agree on a GATT profile. A few common write/notify
characteristic pairs are tried in order; if none match, the tool prints
the characteristics it found so the pair can be added to `BLE_PROFILES`.

## inslib_convert_ubx_to_csv.py - capture to replay dataset

Turns a recorded `.ubx` into the `datasets/` replay format (`config.yaml`
+ `imu.csv` / `baro.csv` / `mag.csv` / `gnss.csv` / `ref.csv`) that
`tools/replay.c` and `python/replay.py` score. With `--pos` it takes an
RTKLIB post-processed solution as an independent `ref.csv` instead of the
receiver's own fix.

`mag.csv` and `mag: enable: 1` appear only when the capture actually
carried `0x40/0x06` frames; a capture without them leaves no empty file
behind, since the harnesses read `enable` as "open mag.csv" and exit when
it is missing. The generated `mag:` section carries the calibration keys
as their no-op identity/zero: hard and soft iron belong to the assembled
unit and are measured by `inslib_calib_gui.py`, not guessed from a
recording.

`mag: wmm_year` is taken from the UTC date of the first NAV-PVT that
carries one, so the epoch comes from the capture rather than from the
clock of whoever converts it. It matters more than it looks: unlike the
live receiver (`insrcv`, which falls back to GPS time and then the host
clock), the replay harnesses build **no** reference field without an
epoch, and with no reference field every magnetometer sample is rejected
and yaw runs unaided. A capture that never got a dated fix leaves the key
at 0 and says so. When an existing `config.yaml` is kept and turns out to
have no `wmm_year` at all, the run prints the value to paste in.

Odometry (`0x40/0x80`) in the capture is counted and reported but not
converted: the replay format has no odometry stream and `replay.c` has no
input for one.

## inslib_parse_file.py - capture health check

```sh
python inslib_parse_file.py inslib_raw.ubx
```

IMU rate, seq gaps (drops), odometry, message counts, and how many bytes
the framer had to skip. If the firmware delivered cleanly and the logger
caught everything, `IMU drops` is 0 -- which proves the firmware path
independent of the live parser's speed.

## inslib_clock_error.py - MCU crystal rate from a session

```sh
python inslib_clock_error.py inslib_20260812_010548_timesync.csv
python inslib_clock_error.py *_timesync.csv --min-pairs 10
```

Reads the `_timesync.csv` a hub session writes and reports the board
timer's rate error in ppm. Each row of that file pairs the MCU counter
with the GPS instant of a receiver time pulse (`0x40/0x05`), raw and
unfitted, which is exactly what a straight line needs.

**Positive ppm means the counter runs slow** - a board microsecond is
longer than a real one. The last line turns the worst segment into the
unit anybody acts on: `1 ppm` is `3.6 ms` of drift per hour of board time.

One plain least squares fit per **segment**, since a broken time base must
not be fitted across:

```
inslib_20260812_010548_timesync.csv
    -13.42 ppm over 4380 s from 73 pulses, residual 8.1 us rms
```

The **residual** is what makes the number checkable. Pairs carrying
nothing but pairing jitter scatter about the line at the size of that
jitter, so a residual well above it is a rate that *moved* during the
segment - on a plain crystal, the warm-up, several ppm over the first
minutes. A plain fit cannot report that any other way: it returns one
slope for the whole span and looks equally confident either way.

Three things are worth knowing before trusting a number from this:

* The CSV is **thin**. The hub writes one row a minute, not one per
  pulse, so an hour's driving yields tens of pairs rather than thousands.
  Enough for a rate, not enough to watch one move. `--min-pairs` (default
  3) skips a segment with fewer than that.
* A pair that **slipped by a whole second** is not rejected here. That
  check lives in `inslib_convert_ubx_to_csv.py`, which has the pulse
  counts to do it with. Least squares has no opinion about any single
  point, so one slipped pair in a thin file moves the result by several
  ppm and leaves nothing that looks wrong.
* `segment` counts **serial reconnects**, not device restarts. The two are
  usually the same event, but they need not be: the VCP belongs to the
  ST-LINK and not to the target, so a board that resets while the port
  stays open keeps its segment number. That shows up in the file's
  `mcu_restarts` column, and a session where it grew has to be split by
  hand before the fit means anything.

Whether the pairs are on the GPS or the UTC scale does not matter: a
constant offset moves the intercept, not the slope, so the leap-second
question that stops `inslib_convert_ubx_to_csv.py` has no hold on a rate.

For the same quantity **live**, with the pulse-level pairs instead of a
row a minute and with slipped pairs, unannounced edges and restarts
handled as they arrive, use the **Clock** tab of `inslib_gui.py`.

The frequency error scales every integrated quantity uniformly, which at a
few tens of ppm is negligible for dead reckoning. It matters where board
time has to be converted to GPS time - post-processing raw observations,
or stamping GNSS epochs with their true time of validity instead of their
arrival time.

## inslib_ubx_imu_calib.py - IMU calibration (no fixture)

Records one session in which the unit is set down in a number of
**arbitrary** static poses with rotations in between, then writes the
REQ-NAV-037 keys into a `config.yaml` that `insrcv --config` and
`python/replay.py` consume directly.

```sh
python3 tools/inslib_ubx_imu_calib.py --port COM4
python3 tools/inslib_ubx_imu_calib.py --udp 29801     # via the hub fan-out
```

**A board that corrects its own stream has to be switched off first**, or
the recording measures what is left over of the calibration it already
holds rather than the sensor:

```sh
python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=0   # live only
```

Nothing in the numbers says which it was, so the recording counts the per
sample `cal_applied` bit instead and says at the end if any of it arrived
corrected. Clearing the switch this way touches the live layer only: the
stored image stays as it is and the unit comes up correcting again at the
next power cycle.

**Why the poses can be arbitrary.** The method is IMU-TK's
(Tedaldi/Pretto/Menegatti, ICRA 2014; the numpy port is
`inslib_imu_tk.py`). Its accelerometer cost is

    residual = |g| - || M * (a_raw - bias) ||

a *magnitude*, so where a pose points never enters it. Hold the unit, put
it down somewhere, leave it a few seconds, pick it up, turn it, repeat.
The static periods are found from a rolling variance and the single
threshold involved is swept and resolved by residual, so there is nothing
to tune. What matters is that the poses point in *different* directions
and that the unit is really at rest in each.

This replaced a 6-position tumble test. That method estimates the bias as
`(up + down)/2` per axis pair, which is exact only when the two halves of
a pair are exactly opposite one another — an independent 1 deg placement
error on each position puts ~0.11 m/s² into the accelerometer bias, more
than the bias of a decent MEMS part. It also could not calibrate the
gyroscope beyond its bias, for want of a rate table.

**The gyroscope gets scale and full misalignment too**, from the same
recording and still without a turntable: between two static poses the
accelerometer knows both gravity directions, and the gyro integrated over
the rotation between them has to carry one onto the other.

Measured on synthetic data with known truth: accelerometer `M` to 1.2e-4
and bias to 7e-4 m/s², gyro `M` to 7e-5 and bias to 0.0007 deg/s. On
IMU-TK's own Xsens recording (`testdata/`): 38 poses found automatically,
|g| 1.1 mm/s² rms across all of them. Both are gated by
`python/tests/test_imu_tk.py`.

**What did it buy?** Both front ends print a before/after table over the
very samples the fit used, so the answer is not a matter of faith:

```
what the calibration bought (over the fitted samples):
                    |a| - |g| [m/s^2]                  gyro dir
                      rms        mean         max          rms
  raw                0.1060    -0.0359      0.2382       2.38 deg
  +bias              0.0713    -0.0454      0.1411       1.44 deg
  +scale/misalign    0.0101    -0.0000      0.0314       0.01 deg
```

The rows are cumulative, so reading down the column separates what the
zero offset fixed from what the scale and misalignment added.

**`--misalignment`** (checkbox in the GUI, **off by default**) turns the
axis misalignment on; without it the fit is scale and bias only and the
off-diagonal terms stay at identity. They are the weakly observable ones:
with few or poorly spread poses they will happily absorb scale and bias
error instead of measuring anything, and a diagonal model is then the more
honest fit, which is why the checkbox starts off. The table is how you tell — if the last
row barely improves on the middle one, they were not observable. On a
session that genuinely has misalignment, switching it off costs a factor
of 2.7 on |g| and 50 on the gyro direction; on imu_tk's Xsens recording,
4x and 6x.

**The initial rest period is the one part that needs care**, because the
gyro bias and the noise model come from it and nothing else can check
them. Two defences: only the genuinely static samples in it are used (the
fraction is reported, and warned about below 90 %), and the bias is a
*median*. A gyro measures a rate, so vibration that starts and ends at
rest integrates to zero and a mean handles it fine; what a mean cannot
survive is a knock that leaves the unit slightly **turned**. With 5 such
knocks in a 20 s window a plain mean is 0.016 deg/s off, the masked mean
0.0009; at 40 knocks the masked mean degrades to 0.020 while the median
holds 0.0023. On a clean window the median costs 0.0003 deg/s.

**Which input.** `--port` opens the device itself, which only works when
nothing else has it. While `inslib_hub.py` is running it does, so use
`--udp` instead and give the hub a second fan-out destination — the
capture and `insrcv` keep running right through the calibration:

```sh
python3 tools/inslib_hub.py COM4 --fanout 127.0.0.1:29800,127.0.0.1:29801
```

A separate port, not a second listener on `insrcv`'s: two UDP sockets
bound to the same port do not both receive on Windows.

An existing `config.yaml` is **merged**, not overwritten: the calibration
keys are replaced and everything else is kept, including a noise model
somebody already tuned against a real dataset (a measured still window is
a decent estimate, but not better than that).

**Temperature.** MEMS biases drift with it, so the session's temperature
range goes into the generated file as a comment. Calibrate on a board
that has reached its working temperature; numbers taken while it is still
warming up describe a state it will not be in.

**The magnetometer rides along.** If the board streams one (`0x40/0x06`),
the same session calibrates it into the `mag:` keys, and nothing has to be
done differently for it: the turns between the poses are exactly the
coverage a magnetometer fit needs. `--no-mag` skips it, `--mag-field-ut`
scales the result to the local field strength instead of to whatever the
sensor reads on average. Details below and in `inslib_mag_calib.py`.

## Magnetometer: hard iron, soft iron, and where it is pointing

Turned through every attitude, a perfect magnetometer traces a sphere of
radius |F|, the local field strength. Two things spoil that. Permanently
magnetised material on the board adds a constant field in the body frame,
which moves the sphere off the origin (**hard iron**), and nearby ferrous
material bends the field, which turns it into an ellipsoid (**soft
iron**). Fitting the ellipsoid recovers both: its centre is `fixed_bias`,
and the matrix that maps it back onto a sphere is `misalignment`.

**What the ellipsoid fit cannot see is a rotation** — a sphere is a sphere
whichever way you turn it — so it pins down `M` only up to one, and the
fit here is deliberately the symmetric square root, the one that rotates
nothing. The rotation left over is the physical one: where the
magnetometer sits relative to the IMU. It is solved separately, from the
one quantity that a rigid pair of sensors cannot disagree about:

    the angle between the local field and gravity is a property of
    where on earth you are, not of how you are holding the unit

so it has to come out the same in every static pose, and the way it fails
to identifies the rotation (three well-spread poses are enough in
principle; a session has twenty). The final `mag: misalignment` is
`R_mag_to_imu * A_symmetric`, and the reported misalignment angles say how
far off the part was mounted. A magnetometer 3 deg off its board otherwise
contributes 3 deg of heading error to an otherwise perfect fusion.

```
magnetometer: 2000 samples, 48.5 uT field (WMM at 48.137 11.575, 2026.6)
hard iron   [+9.00, -5.51, +12.99] uT
soft iron   [-7.0, -0.8, +8.5] % (semi-axis spread)
|m| scatter 8.596 -> 0.151 uT, direction coverage 0.76
mag/IMU misalignment roll +1.43, pitch -2.55, yaw +3.48 deg (4.57 deg total)
measured dip 64.00 deg, scatter over 38 poses 2.26 -> 0.03 deg
```

Read the last line: the field/gravity angle wandered by 2.26 deg over the
poses before the alignment and by 0.03 deg after, which is what says the
rotation was real and not fitted noise. The measured dip can be held
against a WMM lookup for the same position, and `direction coverage` (1.0
is a cloud spread evenly over the sphere, 0 one confined to a plane) is
the number that catches a session where the unit was only ever yawed: the
hard iron across the thin axis of a flat cloud is guesswork, and no
residual will say so.

**Scale.** The fit forces `|corrected|` onto one number, which has to be
chosen. Give the tool a position (`--latlon`, or the Position field in the
window) and it looks the field strength up in INSLIB's own WMM, the same
model the filter arms its magnetometer fusion with. The same position also
fills in the local gravity, which is the accelerometer's half of exactly
this idea (below). Left at 0 the result
keeps whatever the sensor measures on average, which points the same way
but need not agree with the field strength the filter gates on. In the
window, the **GNSS** button next to Position fills the field in from the
receiver's own NAV-PVT fix instead of it being typed in by hand — it lights
up once a 3D fix is arriving on the link (the GNSS receiver's output passed
through by `inslib_hub.py`, above), and the first fix seen fills an empty
field in automatically.

**Watching it live.** Once a field strength is known, the Magnetometer tab
draws it as the line `|m|` has to sit on, and the panel reports how far
off the sensor is right now and over the last few seconds:

```
89 frames
raw +15.8 uT off, 15.6 rms over 6 s
calibrated +0.2 uT off, 0.3 rms
```

Same check the accelerometer gets against 9.81, and it has to be read the
same way: what matters is not one attitude but that the number stays put
while the unit is turned. A hard iron is an offset that *changes* with
attitude, so a magnetometer can look perfect lying on the desk and be 15
µT out upside down. The rms over the trace window is that, condensed.
`insrcv` shows the same pair on its stats line during a live run.

## Housing alignment: where the IMU sits in the box

The calibration above measures the sensor against itself. What it cannot
know is how the board was mounted, and a board 3 deg out of level makes
every attitude the filter reports 3 deg wrong.

So: put the unit on a level surface and press Capture (in
`inslib_calib_gui.py`). Any deviation from level is the mounting error,
and the rotation that removes it is folded into the same 3x3 matrices the
calibration already writes — for the accelerometer, the gyroscope and the
magnetometer alike, because they have to end up in one body frame.

**One placement gives roll and pitch only.** Gravity cannot see a rotation
about itself: a level box turned on the table still reads level. Capture a
second placement on a face that is **not parallel** to the first (level,
then on its side) and the yaw comes with it, because each placement is a
direction that is known in the housing frame and two non-parallel
directions determine a rotation completely (Wahba's problem, solved by
SVD). Note that two *opposite* faces, left and right, are parallel: they
pin the axis they turn about and say nothing about the rotation around it.
Level plus one side is the shortest recipe that gives all three angles.

```
housing alignment from 2 placement(s): bottom face down (normal, upright), left face down
IMU sits at roll +2.19, pitch -1.29, yaw +3.98 deg in the housing
total correction 4.74 deg
residual per placement [0.23, 0.23] deg
```

Two things this cannot do. It assumes the housing faces are square to one
another, and a housing where they are not shows up as the per-placement
residual — which is indistinguishable from a surface that was not level,
so a large residual says something is wrong without saying which. And it
aligns the IMU to the *housing*, not to the vehicle: if the box is then
bolted in crooked, that is a different rotation and only something that
knows the direction of travel (GNSS course over a straight run) can
measure it.

The capture is only as good as the accelerometer calibration under it: an
uncorrected bias of 0.1 m/s² is 0.6 deg of tilt, so calibrate the IMU
first, or point the window at a `config.yaml` that already carries one.
The window says so when there is none. Captures are averaged over the
whole window and rejected if the unit was not still, and the solve always
runs against the *current* calibration from scratch, so capturing one more
placement can only improve the answer and saving twice cannot apply the
rotation twice.

**On the board it is a parameter of its own.** `CFG-FRAME-HOUSING`, one
36-byte record for the whole device, which the board premultiplies onto
whatever its temperature table produces. So a housing pass alone is a
complete upload - **Upload to board** is offered for one, with no
recording behind it - and a temperature node uploaded next winter needs
no housing measurement beside it. Read it back with

```sh
python3 tools/inslib_cfg.py --port COM4 get CFG-FRAME
```

which prints it as roll/pitch/yaw, the three numbers a person can check
against the box, plus the matrix. In `config.yaml` it stays folded into
the 3x3 matrices: there is one of them per sensor there and nothing to
interpolate.

**It does not have to happen in the same session as the IMU calibration.**
Save is offered as soon as a housing alignment exists, with or without a
solve behind it: with no new solve the rotation is applied to the
matrices the `config.yaml` already carries - accelerometer, gyroscope and
the magnetometer section alike, because they all have to end up in the
same body frame. So the usual order works and so does the split one:

```sh
# in one go
record -> solve -> capture placements -> Save

# or separately, later, against the file the first pass wrote
inslib_calib_gui.py -o config.yaml     # point it at the existing file
capture placements -> Save
```

What matters is the order *within* one save, and the window handles it:
the rotation is folded into the file, and the copies still on screen move
with it. That is why the panel switches to "folded into the saved
calibration; capture again to check what is left" afterwards - the
captures stay valid measurements, and re-solving them against the new
calibration shows the leftover rather than the original error.

The one real ordering rule is the other one: **the capture is only as good
as the accelerometer calibration under it.** Doing the housing pass first,
against an uncalibrated unit, measures the accelerometer bias as if it
were a tilt. The panel says which of the three cases it is in, above the
face selector: measured through this session's calibration, through the
one in a `config.yaml`, or through the raw accelerometer.

**What the drop-down asks is which face is on the surface right now**, one
capture at a time - it is not a list of faces to measure. Picking the
wrong entry is the mistake the solve cannot see: it fits whatever it is
given and only the residual grows, which reads the same as a surface that
was not level. So the capture is compared against gravity first and
rejected when the unit is more than 25 deg away from the face it was filed
under, with the face it actually looks like named in the message. Captured
faces are ticked off in the list, and after the first one the selection
moves on to a face that is *not* parallel to it - the one that still has
something to add. The two steps and what each buys stand under the
buttons:

```
[x] 1. one face down      -> roll, pitch
[ ] 2. a non-parallel one -> yaw
```

**A board that corrects its own stream is the case to watch.** With
`CFG-IMU-APPLY_CAL` on and something stored to apply, the samples arriving
are already rotated by the housing key the board carries, so a capture
measures what is *left over* of the mounting error - while the upload
**replaces** that key rather than composing onto it. The panel says so and
offers to clear the switch in the live layer, which leaves the stored
image alone: the unit comes up correcting again at the next power cycle.
The same button is the answer to a `config.yaml` correcting on top of the
board, where every sample is corrected twice.

## inslib_calib_gui.py - the same calibration, guided

A window around the module above: a live count of the static poses the
detector has actually accepted, a sphere showing which gravity directions
are already covered and which part of the sky is still empty (a count
alone hides the case where twenty poses all sit near one attitude — it is
the *spread* that makes the misalignment terms observable), a stillness
meter that says whether the unit is calm enough to count right now, and
live traces. Same solve and the same writer — everything that computes a
number is imported, not reimplemented.

```sh
python3 tools/inslib_calib_gui.py                 # pick the source in the window
python3 tools/inslib_calib_gui.py --udp 29801
python3 tools/inslib_calib_gui.py --latlon 48.14,11.58
python3 tools/inslib_calib_gui.py --latlon 48.91,8.4656,160   # with height
python3 tools/inslib_calib_gui.py --demo          # no board, synthetic stream
```

**The position fills in both reference magnitudes.** The accelerometer
cost is `|g| - ||M (a - b)||`, so whatever `Gravity` is off by lands on the
accelerometer scale factors one for one. The standard 9.80665 is right at
about 45.5 degrees latitude at sea level and nowhere else - at 48.91 deg N
and 160 m it is 2.6e-4 short, which is more than the scale factor
stability of a good MEMS part. Typing a position (or pressing **GNSS**)
therefore sets `Gravity` from WGS84 normal gravity, Somigliana plus the
free air term, out of INSLIB's own `ins_gravity_ned()`:

```
[calib] position from NAV-PVT: 48.137000, 11.575000, h=520 m (14 sat)
[calib] normal gravity at 48.137 11.575, h=520 m (ellipsoidal): 9.80743 m/s2,
        against the standard 9.80665
[calib] WMM at 48.137 11.575: 48.6 uT, declination +4.3 deg, inclination +63.9 deg
```

The third number of the position is **ellipsoidal** height in metres and
may be left off; without it the lookup answers for sea level, which is
3.1e-6 m/s2 per metre too large. The GNSS button fills it in from
NAV-PVT's `height` field, which is the ellipsoidal one - `hMSL` is the
other and differs by the geoid undulation, tens of metres in places.
`--gravity` still sets the number directly and is what a session without
a position uses.

It also carries the two calibrations described above: a Magnetometer tab
with the field-direction sphere and |m| before/after, and the Housing
alignment panel, which is the only place the level-surface capture lives
(it is a guided procedure rather than a recording). Either can be written
against an existing `config.yaml` on its own — Save is offered as soon as
anything measured is worth writing, not only after a full IMU solve.

`--demo` acts out a whole session (rest, then poses with rotations
between them) on a generated stream, which is how the flow is exercised
without hardware. The synthetic magnetometer in it carries a known hard
iron, soft iron and mounting rotation, so the numbers the window reports
can be checked against the ones in `DemoSerial`.

Needs PyQt6 + pyqtgraph + numpy (`pip install -r python/requirements.txt`,
the same ones `python/inspostgui.py` uses).

### Upload to board — one temperature node per session

Next to Save there is **Upload to board**, which writes the session into
the board's own calibration store instead of into a file. The board keeps
a table of temperature nodes and interpolates between them, so a session
does not replace what is there: it contributes the operating point it was
measured at, and the coverage grows as you calibrate at more
temperatures. Let the unit settle at a temperature, record, solve,
upload; repeat somewhere else on the range.

**The second temperature point has to be recorded raw.** Once the first
node is stored and `CFG-IMU-APPLY_CAL` is on, the samples arriving at the
window are already corrected, and a calibration solved from them is the
*residual* of the stored one - which the upload then files beside the
absolute nodes as though it were one of them. Interpolating between the
two is worse than either. **Record** therefore asks first: it checks what
the board is doing to each stream it is about to read and offers to clear
the switches in the live layer (`CFG-IMU-APPLY_CAL`, and
`CFG-MAG-APPLY_CAL` as well when the magnetometer is being calibrated in
the same session, since that is a second switch and not the same one).
The stored image is untouched either way. *Record anyway* stays available
and says in the log what it costs, and on a link that cannot configure the
board the dialog names the `inslib_cfg.py` command instead of offering a
button that cannot work.

The switch is what the board was *told*. What actually reached the samples
is the per sample `cal_applied` bit, which the recording counts: a session
that was corrected for only part of its length - samples still in flight
when the switch went off, or a switch turned back on halfway through - is
reported at the solve and again in the upload dialog, rather than becoming
a node nothing marks as different.

What it does, in order: read the stored table back, merge this session's
node into it in temperature order, write the nodes that actually changed,
and write the node counts last so the new table takes effect in one step
rather than node by node. A node within a degree of one already stored
**replaces** it — the same operating point measured twice is a better
measurement, not a second point.

Three details that are easy to get wrong and are therefore handled here
rather than left to the operator:

* The **housing rotation travels as its own key** (`CFG-FRAME-HOUSING`),
  not folded into the nodes. The board premultiplies it. Folded in it
  would ride on a quantity that is interpolated over temperature, and a
  node added later without a housing measurement beside it would leave
  the table half rotated - which interpolates into a matrix that is no
  rotation, so the attitude turns with temperature. Written apart, a
  housing pass on its own is one key and needs no recording behind it,
  and a new temperature node needs no housing pass. (`config.yaml` still
  gets it folded into its matrices: there is only one of them there and
  nothing to interpolate.)
* The **bias converts out of SI**: the board corrects in the units of its
  own stream, so m/s² becomes g and rad/s becomes deg/s. Getting this
  wrong is silent and costs a factor of 9.8 or 57.
* The **bias polynomial is retired** on every upload. A fit made over the
  previous nodes no longer describes the table, and records can be
  replaced but not removed, so it would otherwise keep overriding the
  node biases.

The dialog offers *Upload and store* or *Upload, live only*. Live only is
gone at the next power cycle, which is what you want while checking a
calibration before committing it. Storing costs a flash write and stops
the board for up to a few hundred milliseconds.

The button stays disabled on a hub fan-out and in `--demo`: those links
carry no request back to the board, and offering an action that cannot
work is worse than not offering it.

## inslib_gui.py - the board's control room

One window onto one device: what it is configured to do, what is stored in
it, and what its sensors and its filter are doing right now. For the
questions that come up about a unit that is already built, rather than for
a workflow.

```sh
python3 tools/inslib_gui.py --port COM5
python3 tools/inslib_gui.py            # pick the port in the window
python3 tools/inslib_gui.py --theme light
```

Two settings in the toolbar are remembered between runs: **Look back**,
how far the live plots reach (10 s to 300 s, applied to every plotted tab
at once), and **Theme**, dark or light. The buffers behind the plots hold
the longest look-back at the nominal sample rates, so a shorter setting
shows less of what is already there rather than throwing it away, and a
setting longer than the link has been up for shows what there is. The
theme switch repaints the running window - traces, axes and all - rather
than needing a restart.

Ten tabs, in the order the questions get asked.

**Overview** is the page to open first, and the only one that answers "is
this working" on its own. Six tiles across the top - filter mode,
attitude source, GNSS fix, position accuracy, processor load, link - then
an artificial horizon and a north-up compass, the numbers that go with
them, the aiding lamps, and a list of warnings.

Nothing on it is new: every value is on one of the other tabs, in more
detail and with the plot that gives it context. What the page adds is the
order they get read in. The compass carries two needles, the heading and
the direction the platform is actually travelling in: on a vehicle that
cannot travel sideways, the angle between them *is* the heading error,
and it shows up there long before it shows up anywhere else. The warnings
are the same idea - the conditions that make a solution untrustworthy are
spread across four messages, and every one of them is easy to miss while
looking straight at it: an axis at its end stop, a lever arm the board
refused, no lever arm at all, a filter and a receiver more than three
accuracy radii apart, a processor at its limit.

**Solution** is what the filter believes, from `0x40/0x0F` ten times a
second: attitude with its 1-sigmas, velocity and position NED, ground
speed and track, latitude/longitude, the three heights with the
ellipsoidal offset between two of them, the vertical rate, which zero
updates are arming, and the lever arm actually in use. Roll and pitch
share a plot and yaw gets its own, behind three tabs with the attitude
1-sigma as the third: the two do not share a scale in any useful way,
since roll and pitch sit in a band a few degrees wide while yaw walks the
whole circle, and drawn together the two that levelling is judged on are
a flat line. The yaw trace is broken at the +-180 degree fold rather than
ruled across the plot.

The **1-sigma** plot is the one to look at after an aiding change: a
sigma that walks up and stays there is a measurement the filter stopped
getting, and neither the angle nor the mode word says so. Velocity NED,
ground speed and position NED sit behind three more tabs below it.

The **absent** values are as much the point as the present ones - an
attitude-only mode has no position at all, and every field that the
message marks invalid shows `--` rather than a zero or the last number
seen. A window that printed 0.000 for "no answer" would be the reason
somebody trusts it.

**Track** plots the filter's position and the receiver's own against the
same origin, in metres, so the distance between them reads off the axes.
That separation is the point: the receiver's position is an *input* to
the filter, and where the two part company is where the lever arm, the
mounting or the aiding gates are wrong. The current separation is drawn
as the segment it is, and the receiver's own horizontal accuracy as a
circle around its latest fix - a separation that stays inside that circle
is the two agreeing, one that leaves it is a disagreement the receiver
did not expect either. Fixes the receiver reports without a valid fix are
dropped rather than drawn faintly - without one it still sends a
position, whatever the last search left behind, and those look like
measurements.

An **OpenStreetMap background is optional and off until switched on**,
behind a confirmation. Tiles need a network, and asking for them tells
`tile.openstreetmap.org` which area is being looked at, which is to say
where the device is. That is a decision for whoever is holding the box.
When it is on, tiles are fetched only for what is on screen, never ahead
of it, capped per refresh and cached on disk, with the attribution the
licence requires shown under the plot.

The metres are on the Web Mercator sphere with its `1/cos(latitude)`
stretch removed at the origin - one coordinate system carrying both an
aligned map and readable distances. Against the WGS84 ellipsoid that
leaves absolute spans 1 to 2 m per km out, with about 0.3 % anisotropy
between the axes. The separation between the two tracks is unaffected,
since both go through the same projection.

**GNSS** is the receiver, and what the filter makes of it. UBX-NAV-PVT
comes past this window on its way through, so the receiver's own opinion
is available next to the filter's without asking either of them for
anything. The first page is what the receiver claims for itself -
horizontal and vertical 1-sigma, speed accuracy, satellites used - which
is a model output and not a measurement, and goes optimistic in exactly
the places that hurt. Read it together with the satellite count, and
treat a figure that improves while the count falls as the receiver
guessing.

The second page is the difference between the two positions, in metres on
the WGS84 ellipsoid at the current latitude (not the Mercator sphere the
Track tab draws on - here the numbers matter more than the tiles). North,
east and down as traces, and the horizontal pair as a scatter with **two
error ellipses** on it.

Two, because a 1-sigma ellipse is the contour that gets misread most
often: in two dimensions it holds about 39 % of the points and not 68 %,
and the 95 % contour is 2.45 sigma rather than the 2 the one-dimensional
rule suggests. Both come out of the sample covariance of the points on
screen, which makes them a statement about the last look-back window and
not a prediction. The line under the plot carries the mean, the two semi
axes with their orientation, DRMS and the 95 % radius, next to the
receiver's own accuracy figure for comparison.

The **centre** of that ellipse is the part that names a fault. Scatter
about zero is the two solutions disagreeing at random, which is what they
are supposed to do. A centre that sits somewhere else and stays there is
a lever arm that has not been measured, or has been measured with a sign
the wrong way round.

**Sensors** plots the stream the board is already sending, so the tab
costs the device nothing. The strip above is read out of each sample's
own status word: die temperature, whether the calibration is reaching the
data, and whether an axis has clipped. None of those three can be seen in
the plotted values themselves. Four pages, because the questions asked of
raw inertial data are not one question:

* **Time series** - the three triads on one shared time axis, so that a
  step in the gyro with nothing under it in the accelerometer is a
  different fault from one that has.
* **Norms and temperature** - the magnitude of the specific force and of
  the magnetic field, which is the one check a three axis sensor offers
  against itself without a reference and whatever the platform is doing.
  At rest the specific force is 1 g whichever way the box is turned, and
  the field magnitude is the same everywhere in a room. A magnitude that
  changes with orientation is a scale factor or a hard iron error, and it
  does that while all three axis traces still look entirely plausible.
  The dashed line on the field is the WMM reference the filter is using.
  Underneath, the three temperatures over time.
* **Spectrum** - amplitude spectral density, Welch averaged over 1024
  sample segments with the mean removed per segment, log-log. The flat
  part is the sensor's own noise and the peaks above it are the airframe.
  A density rather than an FFT magnitude, so a longer window gives a
  smoother curve at the same height instead of a taller one. Nothing
  above the anti-alias filter, and nothing above half the sample rate, is
  real: a rotor line beyond that is folded back and appears at a
  frequency it does not have.
* **Statistics** - mean, standard deviation, extremes and peak-to-peak
  per channel over the look-back window. The standard deviation of a
  triad at rest is its noise, and it is the number to compare between two
  units or before and after a mounting change. It is not the same as the
  density on the spectrum page, which is why both are here.

**Vertical** is the barometric channel and the datums it is expressed
against. The first page is height, the rate it is changing at and the
vertical acceleration on one time axis, three plots locked together
because what is asked of that filter is how the three line up - a step in
height with nothing under it in the acceleration is the barometer moving
and not the platform. Whether ZUPT, VZUPT and ZARU are arming is shown as
three lamps: a state with two values needs a word, not a trace.

The height axis never zooms in past 2 m. At standstill a barometer fills
anything tighter with its own noise, and a unit sitting on a bench then
looks like one that is bouncing. The acceleration is not in `0x40/0x0F`:
it is rebuilt from the IMU stream and the reported attitude the way
`baro_alt.c` builds it - body specific force rotated to NED, gravity
removed, sign flipped because NED z is down while height is up - and
averaged over blocks of samples, since 800 a second of it is a band
rather than a trace.

The second page is the **datum offsets**, and it is the answer to why
three heights that all say "metres" disagree. The *ellipsoidal* offset is
what the filter adds to its local height to get a WGS84 one: an estimated
state that converges over tens of seconds after a fix and drifts away
from one, so a height that looks absolute is that offset plus a local
number. The *barometric* offset is the raw pressure height minus the
fused one, which is the weather moving under a platform that is not.
Under both, the only external check the link offers: the filter's
ellipsoidal height minus the receiver's, with the receiver's own vertical
1-sigma drawn either side of zero. A constant offset that survives a
converged filter is usually the lever arm's down component - the antenna
is not where the IMU is, and an unmeasured arm puts that distance here.

**Clock** is how fast the board's microsecond counter runs, in GPS's
opinion. Every time pulse arrives as `0x40/0x05` carrying the MCU capture
of the edge and the GPS instant the receiver announced for that same edge,
and a straight line through those pairs is the counter's rate error. The
message is not behind a `CFG-MSGOUT` switch: an edge is reported whether
or not there was an announcement for it, because a pulse that is alive
with no time in it and no pulse at all are different faults.

**Positive ppm means the counter runs slow** - a board microsecond is
longer than a real one - which is the sign `inslib_clock_error.py`
computes on a recording of the same frames.

Three plots on one axis: the rate error (pulse to pulse as points, the
rolling estimate over the last sixty pulses as a trace, the fit over the
whole timeline as a dashed line), the residual against that fit, and the
accumulated board-minus-GPS difference in milliseconds, which is what a
timestamp converted with a nominal clock is actually out by.

The residual is the plot that says which of the two rate estimates to
believe. Pairs carrying nothing but pairing jitter scatter about the line
at the size of that jitter, so a residual well above it is a rate that
*changed* during the segment - on a plain crystal that is the warm-up,
several ppm over the first minutes, and the batch fit is then the one
number that cannot report it.

Three kinds of pair are counted and not fitted: an edge without an
announcement, an edge the firmware marked `TP_SUSPECT`, and a pair that
slipped by a whole second. The last is caught the way the offline
converter catches it - GPS seconds and pulse counts advance together, so
their difference is a constant and anything else has slipped. This matters
more than it sounds: least squares has no opinion about any single point,
and one slipped pair in six hundred moves the result by several ppm while
leaving nothing that looks wrong. A counter that runs backwards is a
device restart, which starts a new timeline and drops everything before
it.

Whether the receiver is on the GPS or the UTC scale does not matter here:
a constant offset moves the intercept and not the slope, so the
leap-second question that stops `inslib_convert_ubx_to_csv.py` has no hold
on a rate.

The rate scales every integrated quantity uniformly, and at a few tens of
ppm that is nothing against a dead reckoning error. It matters where board
time has to become GPS time - post-processing raw observations, or
stamping a GNSS epoch with its time of validity rather than its time of
arrival.

**Diagnostics** is whether the box is keeping up and what it has
survived: processor load and the worst fusion epoch since boot (held
against the interval between two IMU samples, which is the budget it
actually has), the link and GNSS counters from `0x40/0x04`, the
acquisition diagnostics from `0x40/0x0D`, and a list of saturation
episodes from `0x40/0x0E`. Empty in that list is the answer you want:
each row names a run of samples in which an axis sat at its end stop,
and those are not measurements - the reading is the end of the range, it
looks entirely plausible, and the filter fuses it as one.

The counters answer "has it ever" and two further pages answer "is it
now". **Load** plots the processor load and the worst fusion epoch over
time, with the epoch budget drawn in: a worst case since boot cannot say
whether the margin is being eaten into over an hour, and a load that
touches its limit for one second in sixty does not move a counter at all.
**Link** plots what actually arrived on this host - the rate of each
message and the throughput in kB/s, counted over one second windows from
framed messages only. A rate that sags while the board says its load is
fine is the link and not the filter.

**Recording** is in the toolbar and writes raw UBX: every checksum-valid
frame, in order, byte for byte, the receiver's own messages included, to
a `.ubx` file the rest of the toolbox already reads. Bytes that did not
resolve into a frame are counted and shown rather than hidden - a
recording is only worth having if it says where it is incomplete.

Configuration and calibration exchanges are **locked out while a
recording runs**. A request flushes the serial input buffer, which would
put a hole of a tenth of a second in the log with nothing to mark it, and
a log with invisible holes is worse than a moment spent stopping it.

**Configuration** shows every scalar key with an editor, plus the GNSS
lever arm and the housing rotation. Type a value, then *Apply (live)*,
which takes effect at once and is gone at the next power cycle, or *Apply
&& store*, which also writes it to flash. Two rules make it safe to leave
open:

* The **board** is the authority on what a setting may be. This tool does
  not restate the ranges from the firmware's descriptor table - it sends
  what was typed and reports the refusal. A rule copied into a GUI drifts,
  and a drifted range rejects a legal value or accepts an illegal one in
  the one place nobody looks.
* Only what **changed** is written, and "changed" means changed by the
  person at the keyboard - not "differs from the stored bytes". Those are
  not the same: an unset lever arm displays as `0.000`, and writing that
  back would turn "nobody has measured the antenna" into "somebody
  measured it and it is zero".

The **lever arm** is edited in the Mounting box, three numbers
forward/right/down in metres from the IMU to the antenna phase centre,
and goes to the board as one record - three separate writes would leave a
window in which the filter runs on two axes of the new arm and one of the
old. An antenna sitting on top of the IMU has a *negative* down
component.

The **Message output** group is one switch per message: whether the board
*sends* it, not whether it is produced. The filter is fed every IMU sample
either way, so clearing *IMU samples* stops the sensor plots in this
window and changes nothing about what the box computes - it buys back most
of the link, which is what it is for.

The **Rates** group is message *periods*, not sensor rates: how often the
board sends a message, in milliseconds. There is no period for the IMU
because its samples go out as they are taken. **The IMU output data rate
is not settable over the link at all** - it is fixed when the firmware is
built, along with the anti-alias filter that goes with it, and the board
reports what it came out as in the *Stored image* box, on the `imu` line:
the rate, the two full scales and the filter corner that unit was built
with. The only choice the link offers is whether the IMU message is sent
at all.

Every row also says whether it differs from the firmware's compiled-in
default, which is the first thing to look at on a unit that behaves unlike
its siblings. Keys the connected firmware does not know are hidden and
excluded from writes, so the tool works against an older board.

The housing rotation is edited as roll/pitch/yaw with the stored nine
numbers shown beneath. Near +-90 degrees of pitch the window says that the
three angles will not read back unchanged - ZYX Euler angles are
degenerate there, and a box mounted on its side really does sit at that
pitch.

**Calibration** decodes what is stored, whole: per temperature node the
three bias terms and **all nine matrix elements**, plus the bias
polynomial drawn over the node range only, because outside it the device
does not extrapolate either. A dashed line marks the current die
temperature, and the note below says whether the unit is being run inside
the range it was measured over.

The board corrects a sample as `M (raw - bias)`, so element `M(row, col)`
is how much of **raw** axis `col` ends up on **corrected** axis `row`: the
diagonal is scale, the rest is mounting and non-orthogonality. The
off-diagonals are shown individually rather than as their largest
absolute value, because which pair an element belongs to is the whole
information in it - two units with the same largest off-diagonal and
opposite signs on it are not two units with the same mounting. Selecting a
row prints that node as a matrix next to the plot, with the diagonal
restated as a scale error in percent and the largest off-diagonal as the
angle it would be if it were misalignment alone.

The table is **editable**, behind an *Allow editing* checkbox so that a
window left open cannot turn a stray keystroke into a stored calibration.
*Write nodes (live)* and *Write && store* send only the nodes that
actually changed, and both ask first. This is for the case the solver
cannot serve - a node known from elsewhere, a sign to be checked against
the hardware, a bias to be nudged after a bench measurement. It is **not**
a substitute for `inslib_calib_gui.py`, which records, fits and merges a
node and is the only sound way to produce one from a device in front of
you.

Three things the write does on its own, because leaving them to the person
typing would be a trap:

* The **bias polynomial is retired**, the way the uploader retires it. A
  fit made over the previous nodes no longer describes the table, and left
  in place it keeps overriding the bias terms that were just typed - the
  note under the plot says so while a polynomial is stored.
* Nodes are **re-sorted by temperature** before the comparison, since the
  board holds them in that order and an edited temperature can move a node
  past its neighbour. What gets written is the indices whose contents
  changed after that sort.
* A pair of nodes closer together than 0.5 degC is **refused** rather than
  stored: a table looked up by temperature has no defined winner between
  them.

Whether a stored table reaches the samples at all is `CFG-IMU-APPLY_CAL` /
`CFG-MAG-APPLY_CAL` on the Configuration tab, and *Erase store* there is
what removes a table - with everything else in the image.

This tool owns the serial port while it runs, like the others below.

## inslib_cfg.py - the board's stored settings

Command line access to the same configuration interface, for everything
that is not a calibration: what the board streams, at what rate, and
whether it corrects its samples at all.

```sh
python3 tools/inslib_cfg.py --port COM4 info
python3 tools/inslib_cfg.py --port COM4 dump
python3 tools/inslib_cfg.py --port COM4 get CFG-RATE CFG-CALACC
python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=0
python3 tools/inslib_cfg.py --port COM4 set CFG-RATE-MAG_MS=50 --flash
python3 tools/inslib_cfg.py --port COM4 leverarm 0.20 0.60 -0.50 --flash
python3 tools/inslib_cfg.py --port COM4 reset --flash
python3 tools/inslib_cfg.py keys            # no board needed
```

`leverarm` is its own command rather than a `set` assignment because
`CFG-FRAME-LEVERARM` is a vector: three numbers, forward/right/down in
metres from the IMU to the GNSS antenna phase centre, body frame FRD -
the same quantity and sign as `gnss.leverarm_frd` in `config.yaml`.
Written in one record, because three separate writes leave a window in
which the filter runs on two axes of the new arm and one of the old.
With no arguments it reads the stored one back, and it distinguishes
`unset` from a configured `[0, 0, 0]`: both fuse identically, but only
one of them is a decision somebody made. The board's status page makes
the same distinction, in the `LEV` line.

An unmeasured lever arm is not a harmless default. Under rotation the
offset does not average out: it couples attitude into the position fix
and, through the fix, back into the attitude.

Values above 100 m on an axis are refused, here and on the board. That
bound is a units guard - metres typed as millimetres - and is set past
the largest airframe on purpose, because a bound that refused a real
tail-fin antenna would silently drop the arm to zero, which is the error
it was supposed to prevent. A record the board refuses shows as `BAD` on
its status page rather than as `unset`.

A name is either a key or a whole group, and `get` prints calibration
records decoded rather than as hex — reading the table back is the point
of having it.

`info` is the one to reach for first: it says whether an image is stored,
whether the live one differs from it, what temperature the board thinks it
is at, and per triad how many nodes there are and whether they are
actually correcting anything.

Two separate things decide that last part, and a board can sit in the
combination where they disagree:

```
APPLY_CAL switch  : imu on, mag off
  magnetometer : 2 nodes, valid, but NOT applied (APPLY_CAL is off)
```

That is a stored, sound magnetometer calibration that corrects nothing,
which is what a recording session leaves behind when `CFG-MAG-APPLY_CAL=0`
was written with `--flash` instead of live only.

**A write is live immediately; `--flash` also persists it.** Without
`--flash` the change is gone at the next power cycle, which is exactly
what a calibration recording wants when it clears `CFG-IMU-APPLY_CAL` for
the duration of the session:

```sh
python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=0   # record raw
# ... record and solve ...
python3 tools/inslib_cfg.py --port COM4 set CFG-IMU-APPLY_CAL=1
```

`reset --flash` erases the store, calibration included.

`reset` **without** `--flash` is a different operation and an easy one to
misread: it drops the live configuration back to the defaults and leaves
the store alone, so `info` and `dump` report no calibration while the
board still has one and loads it back at the next power cycle. Until that
power cycle the live image counts as changed, so any persisted write at
all - `set CFG-RATE-MAG_MS=50 --flash`, an upload from the GUI - stores
the emptied image over the calibration. Power cycle first, or use
`reset --flash` if clearing it is what was meant.

This tool owns the serial port while it runs, so stop `inslib_hub.py`
first. The board answers on the port the request arrived on, and a
one-way UDP fan-out cannot carry one.

## inslib_imu_tk.py - the calibration maths (library, not a command)

numpy port of IMU-TK (BSD, Tedaldi/Pretto/Menegatti, ICRA 2014):
static-interval detection, the two least-squares problems, and a
Levenberg-Marquardt in place of Ceres — which for 9 and 12 parameters is
not a sacrifice worth the dependency. No I/O, so it runs against a
recording from anywhere.

The original C++ is not vendored here; what is kept is the reference
*recording* it ships with, in `testdata/`, because that is the only check
the port has against real hardware — see that directory's README and
`python/tests/test_imu_tk.py`.

## inslib_mag_calib.py, inslib_frame_align.py - the other two maths (libraries)

`inslib_mag_calib.py` is the ellipsoid fit (hard iron, soft iron, the
outlier rejection a magnetometer needs because it picks up whatever passed
close to it) plus the WMM lookup for the local field strength.

`inslib_frame_align.py` is the rotations: the magnetometer against the IMU
from the constancy of the dip angle, and the IMU against the housing from
placements on a level surface, together with the rotation toolbox they
share (Kabsch, the minimal rotation between two directions, the ZYX
extraction `ins_rotmat_to_rpy` uses). No I/O in either, and both are gated
by `python/tests/test_mag_calib.py`, which puts a known hard iron, soft
iron, magnetometer rotation and housing rotation into synthetic data and
requires them all back out.

## testdata/ - reference recordings

Input data for the tests of the tools above, not INSLIB replay datasets
(those live in `datasets/`). Gzipped; `numpy.loadtxt` reads `.gz`
directly. Provenance and licences in `testdata/README.md`.

## inslib_ubx.py - shared framing (library, not a command)

UBX framing/resync/checksum, frame building, the class-0x40 payload
formats, the configuration interface (keys, the VALSET/VALGET/ACK
messages, the calibration record layouts and the SI-to-stream unit
conversions), the device-restart watch, and the datagram packing rule.
Imported by the tools above.

`python/tests/test_cfg_protocol.py` pins the configuration half against
the firmware's own `cfg_keys.h`, reading the record lengths and group ids
straight out of it. A record of the wrong length is refused loudly by the
board; a record of the right length with its fields in the wrong order is
not, which is why that agreement is checked rather than assumed.

Deliberately not pyubx2's stream reader for the *framing*: a capture must
be written before anything is parsed, and reading through a parser couples
the recording to it. The IMU payloads are raw floats, so a `0x24` (`$`)
turns up in them several times a second; pyubx2 read that as an NMEA
header, called `.decode()` on bytes that were not valid UTF-8, and raised
straight out of `read()` — taking the recording with it. pyubx2 is still
the right tool for the *standard* u-blox messages, handed one complete,
already checksum-verified frame at a time.
