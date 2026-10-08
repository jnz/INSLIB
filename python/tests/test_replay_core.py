#!/usr/bin/env python3
"""The replay core shared by tools/replay.py and tools/inspostgui.py
(tools/replay_core.py).

The GNSS outage option on its own, then one whole replay through both
front ends: the simulated A_ideal flight with ranges to four anchors and a
GNSS outage, run through tools/replay.py and through the GUI's worker
thread object. Both have to produce the same solution and the same
summary. Finally the GUI's config editor has to offer every key of the
config.yaml schema, so a new key cannot be silently missing there.

The GUI part needs PyQt6 and is skipped without it.

Runs under pytest or standalone:

    python3 python/tests/test_replay_core.py
"""

import json
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "python"))
sys.path.insert(0, os.path.join(REPO, "tools"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import replay                 # noqa: E402
import replay_core            # noqa: E402
import test_range_stream      # noqa: E402  (its A_ideal + ranges dataset)

OUTAGE = "25:20"


def _gui():
    """inspostgui, or None without PyQt6 (the GUI part is then skipped)."""
    try:
        import PyQt6  # noqa: F401
    except ImportError:
        return None
    import inspostgui
    return inspostgui


def test_gnss_outage_windows_parse():
    assert replay_core.parse_gnss_outages([]) == []
    assert replay_core.parse_gnss_outages([""]) == []
    assert replay_core.parse_gnss_outages(["60:30"]) == [(60.0, 90.0)]
    # The GUI hands its whole text field over as one entry.
    assert replay_core.parse_gnss_outages(["60:30, 200:45"]) == [
        (60.0, 90.0), (200.0, 245.0)]
    assert replay_core.parse_gnss_outages(["1:2", "3:4"]) == [(1.0, 3.0), (3.0, 7.0)]
    for bad in ("60", "a:b", "60:0", "60:-5"):
        try:
            replay_core.parse_gnss_outages([bad])
        except replay_core.ReplayError:
            continue
        raise AssertionError(f"{bad!r} accepted")


def test_gnss_outage_cut_is_anchored_on_the_first_imu_sample():
    spec, data_dir = replay.load_config(test_range_stream.A_IDEAL)
    full = replay_core.load_inputs(spec, data_dir, log=lambda _l: None)
    t0 = full.first_imu_us
    windows = replay_core.parse_gnss_outages([OUTAGE])
    cut = replay_core.load_inputs(spec, data_dir, windows, log=lambda _l: None)
    lo, hi = windows[0]
    inside = [fx for fx in full.fixes
              if lo <= (fx["t_us"] - t0) / 1e6 < hi]
    assert inside and cut.n_outage_cut == len(inside)
    assert len(cut.fixes) == len(full.fixes) - len(inside)
    assert not any(lo <= (fx["t_us"] - t0) / 1e6 < hi for fx in cut.fixes)


def test_replay_without_reference_csv():
    import shutil
    with tempfile.TemporaryDirectory() as d:
        ds = os.path.join(d, "noref")
        shutil.copytree(test_range_stream.A_IDEAL, ds)
        os.remove(os.path.join(ds, "ref.csv"))
        cfg = os.path.join(ds, "config.yaml")
        with open(cfg, encoding="utf-8", newline="") as f:
            txt = f.read()
        # init: ref cannot work without the reference, and is refused
        p = subprocess.run([sys.executable, os.path.join(REPO, "tools", "replay.py"),
                            ds], capture_output=True, text=True, cwd=REPO,
                           encoding="utf-8", errors="replace")
        assert p.returncode != 0 and "init: ref" in p.stdout + p.stderr
        with open(cfg, "w", encoding="utf-8", newline="") as f:
            f.write(re.sub(r"(?m)^init:\s*ref", "init: auto", txt))
        summary_json = os.path.join(d, "summary.json")
        p = subprocess.run([sys.executable, os.path.join(REPO, "tools", "replay.py"),
                            ds, "--summary-json", summary_json],
                           capture_output=True, text=True, cwd=REPO,
                           encoding="utf-8", errors="replace")
        assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
        with open(summary_json, encoding="utf-8") as f:
            assert json.load(f)["scored_epochs"] == 0
        gui = _gui()
        if gui is None:
            return
        raw, _cfg_path, data_dir = gui.load_raw_config(ds)
        spec = gui.merge_spec(raw)
        assert gui.validate_spec(spec, data_dir) == []


def test_estimate_stays_in_the_replay_frame_after_an_origin_reset():
    import math
    lat0, lon0, h0 = math.radians(48.9003), math.radians(8.4787), 387.0
    o0 = replay.llh_to_ecef(lat0, lon0, h0)
    # ins restarted 31 m south, 10 m west and 1 m below the first origin
    lat1 = lat0 - 31.0 / 6.37e6
    lon1 = lon0 - 10.0 / (6.39e6 * math.cos(lat0))
    h1 = h0 - 1.0
    o1 = replay.llh_to_ecef(lat1, lon1, h1)
    local = (2.0, -3.0, 0.5)   # ins position in its new frame

    class Nav:
        def __init__(self, origin, pos):
            self.o, self.p = origin, pos

        def origin_ecef(self):
            return list(self.o)

        def position_local(self):
            return None if self.p is None else list(self.p)

    class Run:
        origin_ecef = list(o0)
        origin_lat, origin_lon = lat0, lon0

    # Where that position is, independently: new origin plus the rotated
    # offset, then into the replay frame like the reference.
    r_n2e = replay.ned_to_ecef_rot(lat1, lon1)
    d = replay._matvec_rm(r_n2e, local)
    ecef = [o1[i] + d[i] for i in range(3)]
    lat, lon, h = replay.ecef_to_llh(*ecef)
    want = replay.ref_to_local_ned({"lat_rad": lat, "lon_rad": lon, "h_m": h},
                                   list(o0), lat0, lon0)
    got = replay_core.ins_pos_in_replay_frame(Nav(o1, local), Run())
    assert max(abs(got[i] - want[i]) for i in range(3)) < 0.01, (got, want)
    assert abs(got[0] - (-29.0)) < 0.2 and abs(got[1] - (-13.0)) < 0.2, got
    # Same origin: unchanged. No position: None.
    assert replay_core.ins_pos_in_replay_frame(Nav(o0, local), Run()) == list(local)
    assert replay_core.ins_pos_in_replay_frame(Nav(o1, None), Run()) is None


def _summary_block(text):
    """The summary from "replayed ..." to the end of the findings, without
    the library's own log lines and without blank lines."""
    lines = text.splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith("replayed "))
    out = []
    for ln in lines[start:]:
        if re.match(r"^\[(INFO|WARN|ERROR|DEBUG) *\]", ln) or not ln.strip():
            continue
        if ln.startswith("wrote accuracy summary") or ln.startswith("gnss delay estimate"):
            break
        out.append(ln)
    return out


def test_gui_worker_and_replay_py_agree():
    gui = _gui()
    if gui is None:
        print("  skip  PyQt6 not installed")
        return
    from PyQt6 import QtCore
    QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
    with tempfile.TemporaryDirectory() as d:
        # No early GNSS stop: the outage below is the only gap.
        ds = test_range_stream._dataset(d, stop_after_sec=1e6, with_ranges=True)
        summary_json = os.path.join(d, "summary.json")
        p = subprocess.run([sys.executable, os.path.join(REPO, "tools", "replay.py"),
                            ds, "--gnss-outage", OUTAGE,
                            "--summary-json", summary_json],
                           capture_output=True, text=True, cwd=REPO,
                           encoding="utf-8", errors="replace")
        assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
        cli_text = p.stdout
        with open(summary_json, encoding="utf-8") as f:
            cli = json.load(f)

        raw, _cfg_path, data_dir = gui.load_raw_config(ds)
        spec = gui.merge_spec(raw)
        assert gui.validate_spec(spec, data_dir) == []
        worker = gui.ReplayWorker(spec, data_dir, gnss_outages=
                                  replay_core.parse_gnss_outages([OUTAGE]))
        done = {}
        worker.sig_error.connect(lambda msg, details: done.update(error=details or msg))
        worker.sig_finished.connect(lambda res: done.update(results=res))
        worker.run()  # synchronous, this thread
        assert "error" not in done, done.get("error")
        res = done["results"]

    assert "simulated GNSS outage" in res["text"]
    assert "range aiding: " in res["text"]
    # The same solution: the scored error, to the last bit, and the same
    # summary line for line (counts, final 1-sigma, data quality, findings).
    assert res["scored_epochs"] == cli["scored_epochs"] > 0
    assert res["pos_rms_m"] == cli["pos_rms_m"], (res["pos_rms_m"], cli["pos_rms_m"])
    gui_block = _summary_block(res["text"])
    cli_block = _summary_block(cli_text)
    assert len(cli_block) > 30 and any(ln.startswith("range aiding") for ln in cli_block)
    assert gui_block == cli_block, "\n".join(
        f"{a!r}\n{b!r}" for a, b in zip(gui_block, cli_block) if a != b)
    # The recorder history ins_plots draws from is complete.
    rec = res["rec"]
    assert len(rec["t"]) > 100
    assert all(len(rec[k]) == len(rec["t"]) for k in replay_core.REC_KEYS)
    assert rec["ref_epoch_count"] > 1 and rec["track_ne"]


def test_gui_config_editor_covers_the_schema():
    gui = _gui()
    if gui is None:
        print("  skip  PyQt6 not installed")
        return
    have = {tuple(field[0]) for _title, fields in gui.CONFIG_SECTIONS
            for field in fields}

    def leaves(d, path=()):
        for k, v in d.items():
            if isinstance(v, dict):
                yield from leaves(v, path + (k,))
            else:
                yield path + (k,)

    schema = set(leaves(replay.DEFAULTS))
    # The score.lim_* regression gates are CI knobs of `make datasets`,
    # left out of the editor on purpose (inspostgui.py's docstring).
    missing = sorted(p for p in schema - have
                     if not (p[0] == "score" and p[-1].startswith("lim_")))
    assert not missing, f"config keys without an editor field: {missing}"
    assert not have - schema, f"editor fields outside the schema: {sorted(have - schema)}"


if __name__ == "__main__":
    fails = 0
    for fn in (test_gnss_outage_windows_parse,
               test_gnss_outage_cut_is_anchored_on_the_first_imu_sample,
               test_estimate_stays_in_the_replay_frame_after_an_origin_reset,
               test_gui_worker_and_replay_py_agree,
               test_gui_config_editor_covers_the_schema):
        try:
            fn()
            print("  ok    %s" % fn.__name__)
        except AssertionError as e:
            fails += 1
            print("  FAIL  %s\n%s" % (fn.__name__, e))
    print("==== %d failures ====" % fails)
    sys.exit(1 if fails else 0)
