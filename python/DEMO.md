# Live demo: ins replay in PlotJuggler

A step-by-step walkthrough to watch the ins solution converge live and
compare it against the ground truth, using the committed `fog` dataset
replay (MEMS ADAHRS vs. an independent FOG strapdown attitude
reference, see `datasets/fog/config.yaml`).

You get three groups of signals in PlotJuggler:

* `INSLIB/…`  — the estimate (pos, vel, attitude, rates, mode)
* `ref/…`    — the ground truth, in the **same frames** as the estimate, so
  you can drop `ref/*` straight on top of the matching `INSLIB/*`
* `error/…`  — `pos_m`, a single scalar drift-vs-truth number

---

## 1. One-time setup

```sh
# a) build the shared library the Python binding loads
make pylib

# b) install PlotJuggler (any recent 3.x)
#    Ubuntu/Debian:  sudo apt install plotjuggler
#    or the AppImage / snap / flatpak from https://plotjuggler.io
```

The `fog` dataset is already committed under `datasets/fog/` — no
download step needed.

Optional: `pip install pymavlink` if you also want the MAVLink output.

---

## 2. Start the PlotJuggler UDP/JSON receiver

1. Launch **PlotJuggler**.
2. Left panel → **Streaming** → choose **UDP Server** in the dropdown →
   click **Start**.
3. In the dialog set:
   * **Port**: `9870`
   * **Message Protocol**: `JSON`
   * tick **"use field as timestamp"** and enter `timestamp`
     (the payload carries both `timestamp` (wall clock) and `t_sec`).
4. Click **OK**. PlotJuggler now listens; the signal tree fills up once
   the replay starts sending.

---

## 3. Run the replay (real time)

```sh
python3 python/replay.py datasets/fog --realtime            # wall-clock paced
# or faster, to not wait the full ~16 min:
python3 python/replay.py datasets/fog --realtime --speed 10
```

Leave `--realtime` off to run as fast as possible (useful for a quick
batch check; PlotJuggler still receives everything, just compressed in
time). The console prints the final `pos rms` and suite `mode` at the end.

---

## 4. Suggested layout (what to plot)

Drag signals from the left tree onto the plotting area. Because `ref/*`
is in the same frame as `INSLIB/*`, just put both on the same plot to
overlay estimate and truth:

| Plot | Signals | Shows |
|------|---------|-------|
| Heading | `INSLIB/att_deg/yaw` **+** `ref/att_deg/yaw` | yaw estimate vs. truth |
| Roll/pitch | `INSLIB/att_deg/roll,pitch` **+** `ref/att_deg/roll,pitch` | leveling vs. truth |
| Position N/E/D | `INSLIB/pos_ned/n,e,d` **+** `ref/pos_ned/n,e,d` | local-NED position vs. truth |
| Altitude | `INSLIB/global/alt_m` **+** `ref/global/alt_m` | height vs. truth |
| Velocity | `INSLIB/vel_ned/n,e,d` **+** `ref/vel_ned/n,e,d` | velocity tracking |
| Drift | `error/pos_m` | scalar position error vs. truth [m] |
| Mode | `INSLIB/mode` | 3=FULL, 2=COASTING, 1=ATTITUDE_ONLY, 0=NONE |
| Ready | `INSLIB/ready` | 1 when ins position is usable |

**Top-down trajectory (XY plot):** right-click the plot area → *Add XY
plot* (or drag two series into an XY panel). Estimate track: `INSLIB/pos_ned/e`
(X) vs `INSLIB/pos_ned/n` (Y). Add `ref/pos_ned/e` vs `ref/pos_ned/n` to the
same XY plot for the ground-truth track on top.

Tip: PlotJuggler's toolbar has a *Save layout* button — store this
arrangement once and reload it for the next run.

---

## 5. Reading the demo

* Early on, `INSLIB/mode` sits at `1` (ATTITUDE_ONLY) / `2` (COASTING)
  during warm-up, then settles at `3` (FULL) once GNSS fixes come in.
  In ATTITUDE_ONLY the position signals go quiet but the attitude keeps
  coming from the AHRS fallback — that is the "best available solution"
  behaviour.
* `ref.csv` attitude is an **independent** truth here: it comes from the
  FOG strapdown solution, a different sensor from the ADAHRS feeding
  `imu.csv`. Overlaying `INSLIB/att_deg/roll,pitch` on `ref/att_deg/*`
  should track within roughly a third of a degree bias and a few tenths
  of a degree spread; yaw settles within about a degree bias, a little
  over a degree spread (a known-good replay's numbers, printed at the
  end of the run — retune your expectations to whatever your own run
  reports).
* `ref.csv` **position/velocity is NOT independent** — it is the same
  GPS solution that also feeds `gnss.csv` (the aiding). `error/pos_m`
  therefore measures self-consistency with that GPS solution, not
  absolute accuracy against a survey-grade truth.

---

## 6. MAVLink (optional)

```sh
python3 python/replay.py datasets/fog --realtime --mavlink   # udp:14550
```

Point any MAVLink GCS (QGroundControl, MAVProxy, a pymavlink script) at
`udp:14550`. It receives `ATTITUDE` (Euler) and `ATTITUDE_QUATERNION`,
`LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `HIGHRES_IMU` and
`EKF_STATUS_REPORT`.

Record a flight log alongside the live stream:

```sh
python3 python/replay.py datasets/fog --realtime --flight-log /tmp/ins_flight.json
```

That writes newline-delimited JSON (one record per telemetry tick,
NaN-free) which PlotJuggler can also open directly as a file.

---

## Troubleshooting

* **No series appear:** confirm the UDP port matches on both sides
  (PlotJuggler `9870`, replay default `9870`; override with `--pj-port`).
  Check no firewall blocks localhost UDP.
* **`libINSLIB … not found`:** run `make pylib` first.
* **Flooded / laggy plots:** lower the publish rate with
  `--telemetry-hz 20` (default 50).
