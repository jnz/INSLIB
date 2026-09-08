# inslib Data Protocol

UBX-based protocol for INSLIB

## Transport

An INSLIB stream carries two kinds of content, in both transports below:

1. Custom INSLIB messages start with UBX class `0x40`
2. The unmodified output of a u-blox receiver, passed through byte for byte
   (classes `0x01` NAV, `0x02` RXM, `0x05` ACK, `0x0D` TIM and whatever else
   the receiver is configured to send)

A parser must treat the stream as lossy and resynchronize on the sync bytes.

## UBX frame format

All frames use the standard u-blox UBX framing:

| Offset | Size | Content |
|-------:|-----:|---------|
| 0 | 2 | Sync bytes `0xB5 0x62` |
| 2 | 1 | Message class |
| 3 | 1 | Message id |
| 4 | 2 | Payload length, u16 little-endian |
| 6 | n | Payload |
| 6+n | 2 | Checksum `ck_a ck_b` (8-bit Fletcher over class, id, length and payload) |

All multi-byte fields in this document are little-endian. Floating point
fields are IEEE 754 single precision (f32).

## Timebase

All `t_us` fields are **u64 microseconds** since e.g. power-up.
The value is monotonic and never wraps in a session:
2^64 microseconds is about 584,000 years.

## Firmware messages (class 0x40)

### 0x40 / 0x01 IMU sample

Rate: one frame per IMU sample.

Payload, 38 bytes, Python struct format `<Q6fIH`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `t_us` | u64 | µs | Time of sample (value e.g. latched in the data-ready interrupt) |
| `accel_x`, `accel_y`, `accel_z` | 3 x f32 | g | Specific force |
| `gyro_x`, `gyro_y`, `gyro_z` | 3 x f32 | deg/s | Angular rate |
| `status` | u32 | | temperature and per-sample state, see below |
| `seq` | u16 | | Sample counter, increments by 1 per acquired sample, wraps at 65535 |

#### The status word

| Bits | Field | Description |
|---|---|---|
| 0-10 | `temp` | sensor temperature, **signed**, 0.1 °C per step, -102.4 to +102.3 °C |
| 11 | `SAT_ACC` | at least one accelerometer axis sat at its end stop in this sample |
| 12 | `SAT_GYR` | same for gyroscope |
| 13 | `CAL_APPLIED` | calibration has been applied to this sample |
| 14-31 | reserved | sent as zero |

**Reserved bits must be IGNORED, not rejected.** A decoder that refuses a
frame with one of them set makes every future addition a breaking change.
Anything that does not fit a spare bit grows the message instead.

`SAT_ACC`/`SAT_GYR` matter because a clipped reading is
**indistinguishable from a real one** in the values beside them: an axis
at its stop reports the end of its range, which is a perfectly plausible
measurement, and a filter fuses it as one. Which axis it was, and how
long the run lasted, is in `0x40/0x0E`. `CAL_APPLIED` is not derivable
from the values either - a device streaming raw data looks exactly like a
calibrated one whose corrections happen to be small.

The measurement range, the low-pass corner and the output data rate are
**not** here. They are in `0x40/0x07` at 1 Hz instead, as physical numbers.

Values are already scaled to physical units in the firmware. A change of the
full-scale ranges requires no host change. A gap in `seq` means samples were
acquired but dropped on the link. The sensor data typically passes a hardware
low-pass filter. For example if the hardware low pass filter is set at 40 Hz,
the physical event would belong roughly 5 to 6 ms before `t_us` (filter group
delay).

### 0x40 / 0x02 Barometer sample

Payload, 16 bytes, format `<Qff`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `t_us` | u64 | µs | Timer value at readout |
| `pressure` | f32 | Pa | Static pressure |
| `temp` | f32 | °C | Sensor temperature |

The message is absent when no barometer is connected or its init failed.

The sensor runs at a much higher internal rate (240 Hz) than this message is
emitted (25 Hz). That is deliberate: the internal rate bounds how *old* a
sample is at readout (≤ 4.2 ms), it is not the output rate. A further ~4.2 ms
comes from the sensor's IIR filter, so the pressure applies roughly 8 ms
before `t_us`.

### 0x40 / 0x06 Magnetometer sample

Payload, 24 bytes, format `<Qffff`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `t_us` | u64 | µs | Time value at readout time |
| `mag_x` | f32 | µT | Magnetic field, body frame |
| `mag_y` | f32 | µT | Magnetic field, body frame |
| `mag_z` | f32 | µT | Magnetic field, body frame |
| `temp` | f32 | °C | Sensor temperature |

### 0x40 / 0x05 Time sync

One frame per rising edge of the GNSS receiver timepulse signal, captured in
hardware by the input capture pin. This is the **only** link between the MCU
timebase and absolute time.

The receiver announces the GPS time of the *next* pulse in `UBX-TIM-TP`
before that pulse occurs. The firmware reads that announcement and pairs it
with the hardware capture of the matching edge. The host therefore receives one exact
(`t_us`, GPS time) pair per second and never has to correlate two
asynchronous streams.

Payload, 28 bytes, format `<QIIIiHH`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `t_us` | u64 | µs | Timer value latched by hardware at the pulse edge |
| `count` | u32 | | Pulse counter since power-up |
| `tow_ms` | u32 | ms | GPS time of week of this edge |
| `tow_sub_ms` | u32 | 2^-32 ms | Sub-millisecond part of the same instant |
| `q_err_ps` | i32 | ps | Receiver's quantisation error for this pulse |
| `week` | u16 | | GPS week number |
| `flags` | u16 | | See below |

`flags`:

| Bit | Name | Meaning |
|----:|------|---------|
| 0 | `GPS_VALID` | An announcement was available for this edge. **When clear, `tow_ms`, `tow_sub_ms`, `q_err_ps` and `week` are all 0 and carry no information** |
| 1 | `UTC_BASE` | The receiver's timebase for this pulse was UTC rather than GNSS time |
| 2 | `UTC_AVAIL` | The receiver has UTC parameters |
| 3 | `QERR_VALID` | `q_err_ps` is usable |
| 4 | `TP_SUSPECT` | The announcement did not advance by exactly one second from the previous one, so this pairing is less trustworthy |

A frame with `GPS_VALID` clear is emitted deliberately rather than dropped:
the receiver pulses only once it has a time solution, but the edge itself is
still evidence that PPS is alive, and keeping every edge makes `count`
gap-free. A gap in `count` means an edge was captured but the frame was
dropped on the link.

The GPS instant of the edge is
`week * 604800 + tow_ms / 1000 + tow_sub_ms / 2^32 / 1000` seconds of GPS
time. Add `q_err_ps` when it is valid: the receiver can only place the edge
on its internal clock grid, and this field is the residual it knows it was
off by.

`TP_SUSPECT` is a warning, not a rejection. It fires when an announcement
was lost on the GNSS UART, when the receiver restarted its time solution,
or at a genuine discontinuity. Treat such pairs as outliers in the clock
fit rather than discarding the frame.

### 0x40 / 0x0D IMU health

Diagnostics for the IMU acquisition chain: 19 u32 in the order below, all
little endian, 76 byte payload. Decode by the `length` field of the frame
rather than by a constant: this message has grown once and may again, and
the fields below never move. Sent every 5 s as a heartbeat and again
within 250 ms of any counter changing. The heartbeat matters: without it a
recording that contains no health message is ambiguous between "nothing
went wrong" and "the message was never sent", and those call for very
different conclusions.

| offset | field | meaning |
|---|---|---|
| 0 | `stalls` | acquisition stopped for >100 ms, count of episodes |
| 4 | `reinits` | full sensor re-initialisations |
| 8 | `reinit_fails` | of those, ones that did not come back |
| 12 | `spi_errors` | Bus errors |
| 16 | `last_busy` | driver's transfer-in-flight flag at the first stall |
| 20 | `last_overruns` | data-ready edges dropped during a transfer |
| 24 | `last_spi_error` | SPI error code, `4` = receive overrun |
| 28 | `last_spi_state` | SPI state, `1` = ready |
| 32 | `last_exti_pend` | `EXTI->PR` at that moment (STM32F4-specific) |
| 36 | `last_drdy_level` | data-ready pin level at that moment |
| 40 | `last_fifo_used` | sample queue depth at the first stall |
| 44 | `at_seq` | `seq` of the IMU message at that moment (u16 range) |
| 48 | `at_ms` | uptime in ms at that moment |
| 52 | `reg_whoami` | sensor `WHO_AM_I` read back then |
| 56 | `reg_pwr_mgmt0` | sensor `PWR_MGMT0`, `0x0F` configured / `0x00` reset default |
| 60 | `reg_int1_cfg0` | sensor `INT1_CONFIG0`, `0x04` configured / `0x80` reset default |
| 64 | `reg_read_rc` | 0 if those three reads succeeded |
| 68 | `sat_sticky` | axes that have EVER reached the sensor end stop, sticky since power up: bit 0..2 accel X/Y/Z, bit 3..5 gyro X/Y/Z, body axes |
| 72 | `sat_samples` | samples in which at least one axis was at the end stop |

**What to read it for.** `spi_errors` is the one to watch on a healthy
system: the firmware recovers from a receive overrun by itself, so the
outage disappears but the counter does not. A rising `spi_errors` with
`stalls` at zero means the underlying timing margin is being eaten into
while the recovery still holds.

Should a stall occur, `last_busy` separates the two failure shapes: `1`
means an SPI/DMA transfer never completed, `0` means data-ready edges
stopped arriving. `reg_pwr_mgmt0` and `reg_int1_cfg0` separate a sensor
that lost its configuration (supply dip) from one that still holds it.

`sat_sticky` is the odd one out here: it is not a fault of the
acquisition chain at all. The samples keep arriving, on time, and every
one of them is wrong - a clipped 8 g reading is indistinguishable from a
real one, so the filter fuses it as a measurement and the whole solution
follows it. It is latched rather than sampled because saturation on a
vehicle lasts a handful of samples at the moment of the shock; a change
in it also forces this message out within 250 ms, so a recording carries
the moment it happened. Use `sat_samples` to tell one jolt from a
measurement range that is simply too small for what the device is bolted
to.

### 0x40 / 0x0E IMU measurement range

One run of consecutive samples in which at least one axis sat at the
sensor's end stop. 8 bytes, little endian:

| offset | type | field | meaning |
|---|---|---|---|
| 0 | u16 | `first_seq` | `seq` of the first clipped sample of the run |
| 2 | u16 | `last_seq` | `seq` of the last one |
| 4 | u16 | `samples` | clipped samples in the run |
| 6 | u1 | `mask` | axes involved: bit 0..2 accel X/Y/Z, bit 3..5 gyro X/Y/Z, body axes |
| 7 | u1 | `flags` | bit 0: episodes were lost before this one |

`first_seq`/`last_seq` are the `seq` of the `0x40/0x01` IMU message, so a
recording marks the affected samples by a field it already has. `samples`
is carried rather than derived from the bracket: a dropped sample leaves
a gap in `seq`, and the bracket is then wider than the run inside it.

**Why this exists at all.** A clipped reading is indistinguishable from a
real one in the stream - a saturated gyro arrives as the end of its own
range, 1999.94 dps at the +-2000 dps setting, which is a perfectly
plausible measurement, and the filter fuses it as one. Nothing downstream can recover that: while an axis is at
its stop the rate error is however far past the range the vehicle
actually went, and it integrates straight into the attitude.

**An episode and not a flag per sample.** At 800 Hz a manoeuvre that clips
would put 800 messages a second on the link at the moment the link is
least idle. A run costs one message. The run is closed when a clean
sample arrives, or forced out once a second while the clipping continues,
so a sensor stuck at its end stop reports itself rather than disappearing
inside one endless episode.

Detection happens on the raw burst before the sample queue, so a sample
that the queue had no room for is still counted: saturation is a property
of what the sensor measured, not of whether the firmware managed to keep
it.

`flags` bit 0 says the queue to the main loop overflowed and episodes
before this one were lost. Without it a host cannot tell an incomplete
record from a complete one. The cumulative `sat_sticky` and `sat_samples`
in `0x40/0x0D` stay correct either way - they are counted at detection,
not at queueing, so they are the authority on whether it happened and
these messages are the detail of where.

### 0x40 / 0x0F Navigation solution

What the filter currently believes. 120 bytes, rate-controlled by
`CFG-RATE-NAV_MS` (default 100 ms) and gated by `CFG-MSGOUT-NAV`.

The period bottoms out at 5 ms, i.e. 200 Hz, which is as fine as a
period counted in milliseconds can be paced. At 128 bytes a frame that
is 25.6 kB/s, which the link only has room for once the IMU stream has
been halved through `CFG-IMU-ODR_HZ` - the two are set as a pair.

| offset | type | field |
|---|---|---|
| 0 | u1 | `version`, `0x01` |
| 1 | u1 | `mode`: 0 none, 1 attitude only, 2 coasting, 3 full |
| 2 | u1 | `att_src`: 0 none, 1 INS, 2 magnetometer AHRS, 3 ARS |
| 3 | u1 | `cal_applied`, bit 0..2 = accel/gyro/mag |
| 4 | u1 | `sat_sticky`, axes ever clipped, same bits as `0x40/0x0E` |
| 5 | u1 | `cpu_pct`, estimated processor load |
| 6 | u2 | `imu_hz`, measured sample rate |
| 8 | u4 | `flags`, see below |
| 12 | u4 | `update_wcet_us`, longest fusion epoch since boot |
| 16 | 6 x f32 | roll, pitch, yaw, then their 1-sigmas, degrees |
| 40 | 3 x f32 | velocity NED, m/s |
| 52 | 3 x f32 | position NED against the origin, m |
| 64 | f32 | height above the NED origin, m |
| 68 | f32 | ellipsoidal height, m |
| 72 | f32 | baro/accel vertical channel, m |
| 76 | f32 | the barometer reading it last fused, same datum, m |
| 80 | f32 | vertical velocity, positive up, m/s |
| 84 | 3 x f32 | GNSS antenna lever arm in use, body FRD, m |
| 96 | 2 x f64 | latitude, longitude of the INS solution, degrees |
| 112 | 2 x f32 | WMM field strength (uT) and declination (deg) here |

`flags`: bit 0 `READY`, 1 `RPY`, 2 `RPY_SIGMA`, 3 `YAW_SIGMA`, 4 `VEL`,
5 `POS`, 6 `LATLON`, 7 `HEIGHT`, 8 `HEIGHT_ELL`, 9 `BARO_ALT`,
10 `BARO_MEAS`, 11 `VZ`, 12 `LEVERARM` (one came from the store),
13 `LEVERARM_BAD` (one is stored and unusable), 14 `ZUPT`, 15 `ZARU`,
16 `VZUPT`, 17 `MAG_REF`. Bits above 17 are reserved, sent as zero, and
must be **ignored** rather than rejected.

**Every value is paired with a validity bit, and a value whose bit is
clear means nothing.** Zero, NaN and a stale number all read as an
answer; only the bit says "the filter does not have this". Attitude-only
modes have no velocity, no position and no latitude/longitude, and say
so. `YAW_SIGMA` is separate from `RPY_SIGMA` because the ARS
free-integrates yaw: its uncertainty is absent, not zero.

**Why this exists.** Until it did, the solution was visible only on a
240x320 panel on the device. Every other message here can be replayed - a
host with the IMU, barometer, magnetometer and raw GNSS stream can
compute its own answer - but what the BOX computed is not recoverable
afterwards, and comparing the two is the whole of integration work.

**`HEIGHT_ELL` is absolute only in the sense that it refers to the
ellipsoid.** When the INS is not running under fresh GNSS it comes from
the local height plus an estimated offset, and that offset converges over
tens of seconds: a valid value can still be a long way out early on, with
nothing in this message saying so. Read it together with `mode`.

Generous with bytes where `0x40/0x01` is not, on purpose: at ten a second
120 bytes is 1.3 kB/s - 1.4 % of a 921600 baud link - and packing a
diagnostic message costs exactly the clarity it exists to provide.

## Configuration interface

The firmware keeps its settings, and above all its sensor calibration, in
its own flash. They are read and written over the same link as everything
else, with messages modelled on u-blox `CFG-VALSET` and `CFG-VALGET` so
that tooling written against those idioms carries over.

**These are the only messages the firmware consumes.** Everything else a
host sends is relayed to the u-blox receiver byte for byte, which is what
makes u-center and an NTRIP client work through this board. The firmware
holds bytes back only from the point where a frame looks like one of its
own, and only takes it out of the stream once the checksum has proven
that it is: a run of bytes inside an RTCM message that happens to start
like a UBX frame is delayed by the length of that frame and then
forwarded unchanged, never swallowed.

### Keys

A key is a `u32`:

| Bits | Meaning |
|------|---------|
| 31 | 0 |
| 30..28 | storage size id |
| 27..16 | group id |
| 15..0 | item id |

Storage size ids follow u-blox (`1` one bit, `2` one byte, `3` two bytes,
`4` four bytes, `5` eight bytes) with one addition: **`6` is a
variable-length record**, whose value is preceded by a `u16` byte count
on the wire. That is what lets one calibration temperature node travel as
a single key instead of thirteen scalar ones per node and per sensor.

Item id `0xFFFF` is a wildcard in `CFG-VALGET` and expands to every known
item of that group.

| Group | Contents |
|------:|----------|
| `0x001` | message output enables, one `U1` per firmware message |
| `0x002` | rates |
| `0x003` | IMU: `APPLY_CAL` (`U1`), `TEMP_TAU_MS` (`U2`), `ODR_HZ` (`U2`), `LPF_DIV` (`U2`) |
| `0x004` | magnetometer, same two items |
| `0x005` | frame: `HOUSING` (record, item `0x001`), `LEVERARM` (record, item `0x002`) |
| `0x010` | accelerometer calibration |
| `0x011` | gyroscope calibration |
| `0x012` | magnetometer calibration |

Each calibration group holds `NPTS` (`U1`, item `0x001`), `BIASPOLY`
(record, item `0x002`) and up to 16 nodes `POINT_n` (record, items
`0x100 + n`).

The authoritative list, with the default and the permitted range of every
key, is the descriptor table in
`embedded/stm32f429/Core/Src/inslib_sensor/cfg.c`. A key with no entry
there is rejected rather than ignored, so a typo in a host tool is
reported instead of silently writing into nothing.

A few keys accept a **list of values rather than an interval**, because
what they select is a hardware step that has nothing in between:
`CFG-IMU-ODR_HZ` takes 400 or 800, and `CFG-IMU-LPF_DIV` takes 4, 8, 16,
32, 64 or 128. A value inside the bounds but off the list comes back as
`E_RANGE`. It is deliberately not rounded to a neighbour: a host that
learns it asked for the impossible is in a better position than one that
believes it got what it asked for.

`CFG-IMU-ODR_HZ` is the one setting in the store that changes what the
device **computes** and not only what it sends - the filter runs one
epoch per IMU sample. `CFG-IMU-LPF_DIV` is the sensor's own low pass,
given as the divider of that rate: the corner follows the rate on its
own, so a stored `16` stays true when the rate is halved where a stored
`50 Hz` would not. The frequency the two work out to is reported in
`0x40/0x07` as `accel_lpf_hz` / `gyro_lpf_hz`.

### Layers

`CFG-VALSET` takes a bitmask: bit 0 is the live configuration, bit 2 is
flash. **A write always lands in the live layer**, and the flash bit
additionally persists the whole image afterwards. There is no write that
skips the live layer, because the stored image is by definition a
snapshot of it. Whether the snapshot is current is reported by the
`UNSAVED` flag of `0x40/0x07`.

`CFG-VALGET` takes a single value: `0` live, `7` default. There is no
flash layer to read, for the same reason.

A persisted write blocks the device for as long as the flash controller
takes. That is milliseconds for most saves, but every few saves the store
has to erase a sector, and this part cannot fetch instructions while its
flash is erasing: the CPU stops for a few hundred milliseconds and the
GNSS and IMU streams lose whatever arrives meanwhile. Allow several
seconds for the acknowledgement, and do not persist in a loop.

### 0x40 / 0x08 CFG-VALSET

Host to device.

| Field | Type | Description |
|-------|------|-------------|
| `version` | u8 | `0x01` |
| `layers` | u8 | bit 0 live, bit 2 flash |
| `reserved` | 2 x u8 | |
| repeated | | `key` (u32) followed by its value |

The value width comes from the key's size field. For a record key the
value is `u16 length` followed by that many bytes.

### 0x40 / 0x09 CFG-VALGET

Host to device.

| Field | Type | Description |
|-------|------|-------------|
| `version` | u8 | `0x00` |
| `layer` | u8 | `0` live, `7` default |
| `reserved` | 2 x u8 | |
| repeated | | `key` (u32) |

### 0x40 / 0x0A CFG-VALGET response

Device to host, same shape as `CFG-VALSET` with `version` `0x01` and the
values filled in, so a response can be fed straight back as a `VALSET`.

**Keys that carry no value are absent from the response rather than
reported as an error.** A host learns what is set by comparing what came
back with what it asked for.

An answer too large for one frame arrives as several, in request order.
The **first reserved byte is set while further frames of the same answer
are still to come** and clear on the last one. Read until it is clear: a
response that happens to fill a frame is otherwise indistinguishable from
the final one, and a wildcard over a calibration group fills several.

### 0x40 / 0x0C CFG-RESET

Host to device.

| Field | Type | Description |
|-------|------|-------------|
| `version` | u8 | `0x01` |
| `layers` | u8 | bit 0 live, bit 2 flash |
| `reserved` | 2 x u8 | |

Without the flash bit only the live values are dropped and the stored
image returns at the next power cycle. With it the store is erased and
the device is back to its defaults for good.

### 0x40 / 0x0B CFG-ACK

Device to host, the result of a `VALSET` or a `RESET`, and of a malformed
`VALGET`.

| Field | Type | Description |
|-------|------|-------------|
| `version` | u8 | `0x01` |
| `result` | u8 | see below |
| `msg_id` | u8 | the message being answered |
| `reserved` | u8 | |
| `key` | u32 | the key that failed, where one is to blame, else 0 |
| `detail` | u32 | values applied, on a successful `VALSET` |

| `result` | Meaning |
|---------:|---------|
| 0 | accepted |
| 1 | no such key |
| 2 | value length does not match the key |
| 3 | value outside the key's permitted range |
| 4 | erase or program failed |
| 5 | the live image has no room for this record |
| 6 | malformed message |
| 7 | layer mask empty or unsupported |
| 8 | the key is known but carries no value |

A `VALSET` is applied in order and **stops at the first failure**, so
`detail` says how many values went in before it. The frame is not atomic.

### 0x40 / 0x07 Configuration state

Device to host, periodically and rate-controlled by its own key.

Payload, 48 bytes, format `<BBBBIIffBBBBIHHffff`. Version `0x02` appended
the IMU configuration; `0x01` ended after `cmd_dropped` at 28 bytes.

| Field | Type | Description |
|-------|------|-------------|
| `version` | u8 | `0x01` |
| `flags` | u8 | see below |
| `cal_flags` | u8 | see below |
| `reserved` | u8 | |
| `cfg_crc` | u32 | CRC32 of the live image |
| `cfg_seq` | u32 | sequence number of the stored image, 0 if none |
| `temp_imu` | f32 | filtered IMU die temperature, °C |
| `temp_mag` | f32 | filtered magnetometer die temperature, °C |
| `npts_acc` | u8 | calibration nodes in use, accelerometer |
| `npts_gyr` | u8 | gyroscope |
| `npts_mag` | u8 | magnetometer |
| `reserved2` | u8 | |
| `cmd_dropped` | u32 | host commands dropped because one was still pending |
| `imu_odr_hz` | u16 | nominal IMU output data rate, see `CFG-IMU-ODR_HZ` |
| `reserved3` | u16 | |
| `accel_fs_g` | f32 | accelerometer full scale, ± this, g |
| `gyro_fs_dps` | f32 | gyroscope full scale, ± this, deg/s |
| `accel_lpf_hz` | f32 | configured accelerometer low-pass corner, Hz |
| `gyro_lpf_hz` | f32 | gyroscope low-pass corner, Hz |

The full scales are the **clip level**: a reading at exactly this value is
a reading at the end stop, which is what makes the saturation bits of
`0x40/0x01` checkable against the values beside them.

Physical numbers and not register codes, deliberately. A code needs a
decode table in the firmware AND in every host tool, and a table that
drifts produces plausible-looking wrong data rather than an error - the
one failure mode this protocol is worst at surfacing. A number needs no
table, and a different sensor fits without touching the protocol.

`flags`:

| Bit | Name | Meaning |
|----:|------|---------|
| 0 | `STORED` | an image exists in flash |
| 1 | `UNSAVED` | the live image differs from it |
| 2 | `IMU_CAL` | the IMU correction is being applied |
| 3 | `MAG_CAL` | the magnetometer correction likewise |

`cal_flags`:

| Bit | Meaning |
|----:|---------|
| 0..2 | accelerometer, gyroscope, magnetometer calibration is valid |
| 3..5 | the same three, temperature outside the calibrated range |

`cfg_crc` is what ties a recording to the calibration it was taken with.
Log this message and there is no later question about which correction
the samples went through.

### Calibration model

Per triad, and the same model the host side tools write into
`config.yaml`:

    corrected = M(T) * (raw - b(T))

`M` carries misalignment and scale together and is **linearly
interpolated** between the temperature nodes. For `b` a stored
**polynomial** in `(T - t_ref)` wins, and where none is stored the bias
values held at the nodes are interpolated instead.

The polynomial is the better answer where it exists. Interpolating the
bias puts a kink into it at every node, and a kink in the gyroscope bias
is a step that the navigation filter reads as a disturbance rather than
as a slowly varying parameter. The misalignment barely moves with
temperature, and a table cannot run away just outside the range it was
fitted on the way a cubic can.

Each node carries the bias **measured at that temperature** even so, and
that is what makes a node-by-node upload work. The device remembers what
was measured where, so a host can read the whole table back with a
wildcard `VALGET`, fit the polynomial over it, and write that back,
without keeping a file of its own between sessions. A device with one or
two nodes corrects its bias from them in the meantime rather than waiting
for a fit that needs more points than exist yet.

Outside the node range **nothing is extrapolated**: both parts are
evaluated at the nearest node temperature and the clamp bit in
`cal_flags` says so. A single node therefore means a constant correction,
which is the honest answer when the unit was calibrated at one
temperature.

`T` is the sensor's own die temperature, low pass filtered with
`TEMP_TAU_MS`. Unfiltered it would carry its own noise into the
correction, which is a way of adding noise to a sample in the name of
removing error.

**Units are the units of this protocol, not SI**: accelerometer bias in g,
gyroscope bias in deg/s, magnetometer bias in µT. `M` is dimensionless
and therefore identical to the host side matrices. A host holding the
bias in m/s² or rad/s has to convert. That is deliberate: correcting in
the sample's own unit keeps the per-sample path free of two unit
conversions it would otherwise carry at the IMU rate.

`POINT_n`, 52 bytes, format `<13f`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `temp_c` | f32 | °C | node temperature |
| `M` | 9 x f32 | | misalignment and scale, **column major** |
| `b` | 3 x f32 | g, deg/s, µT | bias measured at this node |

`BIASPOLY`, 56 bytes, format `<fB3x12f`:

| Field | Type | Description |
|-------|------|-------------|
| `t_ref` | f32 | the polynomial is evaluated in `(T - t_ref)` |
| `deg` | u8 | degree, 0 to 3, or `0xFF` for "no fit, use the node biases" |
| `reserved` | 3 x u8 | |
| `c` | 12 x f32 | coefficients, **axis major**: `c[axis][k]` for `(T - t_ref)^k`, four per axis regardless of `deg` |

A calibration is activated only once the whole of it checks out: every
node `NPTS` promises is present, the node temperatures rise strictly, and
no value is a NaN. A half uploaded table is rejected as a whole, never
applied in part. **Write `NPTS` last** and the new calibration takes
effect in one step.

`BIASPOLY` may be absent, and then the node biases carry the correction
on their own. Fit it once the table has enough nodes to be worth fitting
over, which for a quadratic means at least three well separated ones.

**Retire a fit with `deg = 0xFF` whenever the node table changes.** A
record can be replaced but never removed, so a polynomial fitted over an
older set of nodes would otherwise stay in charge of the bias after a
node is added. Writing zero coefficients is not the same thing: that is a
bias of zero, and it would override the node values with it.

### `CFG-FRAME-HOUSING` - where the board sits in its box

One record of 36 bytes, `f32 R[9]` column major: the rotation from the
sensor axes onto the ones the housing is built along. The full correction
is

    corrected = R * M(T) * (raw - b(T))

so the bias is untouched - it sits on the raw side of the matrix.

**One key for the whole device**, not one per triad. It describes the
mounting rather than a sensor, and three copies of the same number could
drift apart while still describing one piece of hardware. The
accelerometer, the gyroscope and the magnetometer all have to end up in
the same body frame.

**Kept out of the node matrices on purpose.** Those are a table over
temperature that the device interpolates between. A rotation folded into
some nodes and not into others - which is what happens as soon as a node
is added later without a housing measurement beside it - interpolates
into a matrix that is no rotation at all: the attitude then turns with
temperature and the scale goes with it, 0.4 % for a 10 degree mounting
error. Held apart, a node stays a measurement of the sensor and this
stays a property of the box, and either can be rewritten without touching
the other.

The record has to be a rotation. One that is not orthonormal, or that is
a reflection, is **refused and reported** (`cal_flags` bit `0x80`) rather
than applied: a matrix that also scales would change the very magnitude
the calibration exists to get right, and every attitude would still look
plausible. An absent record means identity.

It applies whether or not a node table exists, so a unit that has only
ever been measured in its housing still corrects for that. `cal_flags`
bit `0x40` says one is in effect.

A host that writes a new node without having measured the housing simply
leaves this key alone: absent from a `CFG-VALSET` means "nothing to say
about it", never "identity".

### `CFG-FRAME-LEVERARM` - where the GNSS antenna sits

One record of 12 bytes, `f32 l[3]` little endian: the vector from the IMU
to the antenna phase centre, in the **body frame FRD** (x forward, y
right, z down), metres. Same quantity and same sign convention as
`gnss.leverarm_frd` in the host side `config.yaml`, so a value measured
once is usable in both places unchanged.

The device hands it to the filter with every GNSS fix, where it corrects
the position and velocity residuals for the fact that the antenna is not
where the IMU is. Under rotation an uncorrected arm does not average out:
it couples attitude into the position fix and, through the fix, back into
the attitude.

An absent record means zero, i.e. antenna and IMU coincide. That is a
real assumption rather than a neutral one, and the device says which of
the two it is using on its status page (`cfg` against `unset`) - a
configured `[0, 0, 0]` and a never-measured one fuse identically but are
not the same statement.

**One record, not three `R4` keys.** Three separate writes have a window
in which the filter runs on two axes of the new arm and one of the old. A
vector is set as a vector.

A stored record that is not finite, or whose magnitude exceeds
`INSLIB_CFG_LEVERARM_MAX_M` (100 m) on any axis, is refused and reported
as such - the status page says `BAD` rather than showing the arm, and the
filter runs on zero. That is deliberately distinct from `unset`: an
absent record means nobody has measured the antenna, a refused one means
somebody did and it is not being used, and a rejection that read as
absence is the one way this could go wrong with nobody seeing it.

The bound is a units guard and not a description of a vehicle, which is
why it sits past the largest airframe an IMU could be bolted into - an
An-225 is 84 m long and spans 88 m. A tail-fin antenna against an IMU at
the centre of gravity is tens of metres on aircraft of that size, and a
bound that refused it would be worse than no bound at all: the arm falls
back to zero and the aircraft flies with exactly the error this setting
exists to remove. At 100 m it still catches a value entered in
millimetres for anything above 10 cm. It does not catch centimetres for
metres, and no bound can without rejecting real hardware - what catches
that is the number being on the screen, in metres, next to the vehicle it
describes.

### Recording a calibration

The correction is applied before either the on-board fusion or this
stream sees a sample, so a recording made for solving a new calibration
has to switch it off first, otherwise the solver fits a correction on top
of an already corrected stream. Clear `APPLY_CAL` **in the live layer
only**, so that a power cycle brings the device back to normal whatever
happens to the session.

### 0x40 / 0x80 Odometry

Absolute ground speed (e.g. from an OBD-II dongle, but not
OBD-specific: a wheel-odometry rate or a Doppler log fits the same frame).

The frame is **filled in two stages by two different processes**, which is
why it carries two timestamps:

1. The **producer** potentially runs on a host PC and has no access to the MCU counter,
   and fills `t_unix_us`, `delay_ms`, `speed_mps`, `stddev_mps`, `kind`
   and the direction bits. It leaves `t_us` at 0 and `T_US_VALID` clear.
2. The **hub** (`tools/inslib_hub.py`) is the only process that observes
   both clocks: it reads the IMU stream (MCU timer) and receives these
   frames (host wall clock), so it can pair them. It fills `t_us`, sets
   `T_US_VALID`, and emits the completed frame into the sensor stream.

Payload, 30 bytes, format `<QQffHHH`:

| Field | Type | Unit | Written by | Description |
|-------|------|------|------------|-------------|
| `t_unix_us` | u64 | µs | producer | Host wall clock (UTC, since the Unix epoch) when the value was RECEIVED |
| `t_us` | u64 | µs | hub | MCU timer at the measurement epoch, i.e. `t_unix_us - delay_ms` mapped onto the MCU clock. 0 while `T_US_VALID` is clear |
| `speed_mps` | f32 | m/s | producer | Ground speed, unsigned |
| `stddev_mps` | f32 | m/s | producer | PER-SAMPLE 1-sigma (see below) |
| `delay_ms` | u16 | ms | producer | Estimated age of the value at `t_unix_us` |
| `flags` | u16 | | both | See below |
| `kind` | u16 | | producer | 0: ground speed. Other values reserved |

`flags`:

| Bit | Name | Set by | Meaning |
|----:|------|--------|---------|
| 0 | `DIR_VALID` | producer | The producer knows the direction of travel. When clear, bit 1 carries no information |
| 1 | `REVERSE` | producer | The vehicle is moving backwards |
| 2 | `T_DEGRADED` | hub | `t_us` was mapped with a stale or thin host/MCU offset window, so it is less reliable than usual |
| 3 | `DELAY_SUSPECT` | producer | The round trip was well above this session's floor, so `delay_ms` is less trustworthy than usual |
| 4 | `T_US_VALID` | hub | `t_us` has been filled in. **A consumer that needs `t_us` must check this bit and ignore the frame otherwise** |

A frame with `T_US_VALID` clear may legitimately appear in a capture: when
no IMU stream has been seen yet there is nothing to map onto, and the hub
records the sample unstamped rather than dropping it. `tools/insrcv.c`
ignores such frames; an offline consumer can still map them.

`stddev_mps` is the per-sample uncertainty **only** — for OBD-II PID 0x0D
the 1 km/h quantisation, i.e. 1/sqrt(12) km/h = 0.080 m/s. The systematic
speedometer scale error (EU type approval forbids reading low, so
production speedometers read 2..5% high; tyre wear adds to it) is a
property of the vehicle rather than of the sample and is deliberately not
folded in here. It belongs in the filter's `speed_scale` /
`speed_stddev_rel`, which is where a constant bias can be removed instead
of being disguised as noise.

`delay_ms` is half the measured request/response round trip plus a
constant for the ECU's own update interval, which is not observable from
outside the dongle. The consumer feeds it to `speed_delay_ms` so the
residual is anchored in the filter's state history rather than against
the current state.

What is left of that estimate after the round trip has been accounted for
does show up in `stddev_mps`, but only where it costs anything:
misdating a CONSTANT speed is free, so the delay uncertainty enters as
`|dv/dt| * sigma_delay` and is added in quadrature to the quantisation
term. At a steady cruise it vanishes; under braking it dominates.

**A small negative age is normal.** `t_us` is derived from the host clock
minus `delay_ms`, whereas the IMU stream reaches the host with a transport
latency of its own (order 100 ms over USB CDC). The age of an odometry
sample measured against the IMU epoch being processed is therefore
`delay_ms` minus that latency, which lands near zero with jitter on both
sides. A consumer that rejects everything stamped ahead of the current
epoch throws away about half of a healthy stream; one that accepts a
sample stamped *seconds* ahead is ignoring a broken mapping.
`tools/insrcv.c` accepts a lead of up to 500 ms and fuses it at delay 0.

`kind` separates measurement *models*, not devices. Ground speed fuses as
`h(x) = ||v_n||`; a relative airspeed would need a wind assumption and is
a different model, so a consumer must be able to tell them apart rather
than infer it from context.

**On `REVERSE`.** It does not affect the speed fusion: `||v_n||` has no
sign, so driving backwards at 5 m/s and forwards at 5 m/s are the same
measurement. It exists for the *course-over-ground* assumption, which
breaks completely in reverse — heading and course differ by 180 degrees
there, and a filter deriving yaw from GNSS course would take a heading
error of exactly that size. `ins` carries that assumption in
`ins_options_t.automotive_mode`, which is configured once rather than per
epoch, so nothing in the library consumes this bit today.
`tools/insrcv.c` decodes it, publishes it and counts it, and does not act
on it. Recording it now means a capture taken today stays usable when
that changes.

## Message id ranges

Ids below `0x80` are the **firmware's**. Documented above are `0x01`, `0x02`,
`0x05` and `0x06` as sensor data, `0x0D` as IMU diagnostics, and `0x07`
through `0x0C` as the configuration interface; `0x04` is additionally
emitted as a status-counter block (tx_dropped, imu_overruns and similar,
see `tools/inslib_convert_ubx_to_csv.py`), and further ids may be added
there. `0x03` is retired and must not be reused.

`0x08`, `0x09` and `0x0C` travel from host to device and are the only ids
in the whole protocol the firmware consumes. Everything else a host sends
reaches the u-blox receiver unchanged.

Ids from `0x80` up are produced by **host-side tools** and never by the
firmware, so the two can share one stream without an id ever meaning two
things. Currently `0x80` (odometry) is the only one.

A host-produced message is filled by whichever host process holds the
information, which may be more than one: `0x80` carries a producer half
and a hub half (see above). That is a property of the message, not of its
id — an id still names exactly one measurement.

## GNSS passthrough content

With the intended receiver configuration the stream additionally contains:

| Class/Id | Message | Rate |
|----------|---------|------|
| 0x01/0x07 | NAV-PVT, position, velocity, UTC time, fix flags | per navigation solution |
| 0x01/0x36 | NAV-COV, position and velocity covariance | per navigation solution |
| 0x02/0x15 | RXM-RAWX, raw pseudorange and carrier phase | per measurement epoch |
| 0x02/0x13 | RXM-SFRBX, broadcast navigation subframes | per subframe |

The rates are the receiver's own and are set on the host, not by the firmware,
so this document does not name a number for them. Read the current one from
`CFG-RATE-MEAS` and `CFG-RATE-NAV` rather than assuming.

Payload formats of these messages are defined in the u-blox UBX Interface
Description, not here.

## Mapping `t_us` to GPS time

Each `0x40/0x05` frame with `GPS_VALID` set is one exact
(`t_us`, GPS time) pair, without rounding and without any pairing work on
the host — the firmware has already done it.

For continuous timestamps, fit a linear clock model `t_gps = a + b * t_us`
over a sliding window of pairs. The slope `b` absorbs the MCU crystal error
(some ppm) and the fit bridges missing pulses. Feed `q_err_ps` into the pair
where it is valid, and down-weight or reject pairs carrying `TP_SUSPECT`.

Because `t_us` is monotonic u64, the fit needs no wrap handling and a
single model stays valid for the whole session.

## Computing the checksum

The 8-bit Fletcher checksum runs over class, id, length and payload -
everything in the frame except the two sync bytes and the checksum
itself.

Python (`tools/inslib_ubx.py`):

```python
def ubx_checksum(data):
    """8-bit Fletcher over class, id, length and payload."""
    ck_a = ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b
```

C (`tools/insrcv.c`):

```c
static void ubx_checksum(const uint8_t* data, size_t n, uint8_t* ck_a, uint8_t* ck_b)
{
    uint8_t a = 0, b = 0;
    size_t  i;
    for (i = 0; i < n; ++i)
    {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}
```

Both take the frame starting at `class` (offset 2), running through the end
of the payload (offset `6+n`, exclusive) - the sync bytes are never part of
the sum, and the checksum bytes obviously aren't either.
