#!/usr/bin/env python3
"""score: leverarm_frd applied wherever the estimate meets the reference
(REQ-VER-037): replay.ref_point_offset_ned(), which the ellipsoid height
score and the --plot recorders of tools/replay.py and tools/inspostgui.py
add to the IMU-point quantities, and ins_plots._ref_pt_up(), which shifts
the board curves of the altitude pages. And inspostgui.py's lever arm
compensation (REQ-VER-039): replay.leverarm_relation(), the GNSS vs.
scoring lever arm check, and the rotated lever arms the GUI worker records
next to every reference sample and every fix (the replay part needs PyQt6
and is skipped without it).

Runs under pytest or standalone:

    python3 python/tests/test_score_leverarm.py
"""

import math
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "python"))
sys.path.insert(0, os.path.join(REPO, "tools"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import replay             # noqa: E402
import ins_plots          # noqa: E402
import test_range_stream  # noqa: E402  (its A_ideal dataset)


class _Nav:
    def __init__(self, rpy):
        self._rpy = rpy
        self.calls = 0

    def rpy(self):
        self.calls += 1
        return self._rpy


def test_height_projection_matches_rotated_lever_arm():
    la = (-2.0, 0.1, -1.3)  # antenna behind, right of and above the IMU
    for rpy_deg in ((0.0, 0.0, 0.0), (5.0, -10.0, 0.0), (5.0, -10.0, 135.0),
                    (-20.0, 15.0, -60.0)):
        rpy = tuple(math.radians(v) for v in rpy_deg)
        d = replay.ref_point_offset_ned(_Nav(rpy), la)
        # Same rotation as ins: R_b_to_n from ZYX roll/pitch/yaw.
        R = replay._rotmat_from_rpy(rpy)
        want = [sum(R[r][k] * la[k] for k in range(3)) for r in range(3)]
        assert all(abs(a - b) < 1e-12 for a, b in zip(d, want)), (rpy_deg, d, want)
        # The height part depends on roll and pitch only.
        r, p = rpy[0], rpy[1]
        down = (-math.sin(p) * la[0] + math.sin(r) * math.cos(p) * la[1]
                + math.cos(r) * math.cos(p) * la[2])
        assert abs(d[2] - down) < 1e-12, (rpy_deg, d[2], down)
    # Level: the antenna 1.3 m above the IMU is 1.3 m up.
    assert abs(replay.ref_point_offset_ned(_Nav((0.0, 0.0, 0.0)), la)[2] + 1.3) < 1e-12
    # No attitude yet: treated as level, so the height is still right.
    assert abs(replay.ref_point_offset_ned(_Nav(None), la)[2] + 1.3) < 1e-12
    # The altitude pages shift every board curve by -down, per sample.
    up, shifted = ins_plots._ref_pt_up({"ref_pt_up": [1.3, float("nan"), 1.2]}, 4)
    assert shifted and up == [1.3, 0.0, 1.2, 0.0], up


def test_zero_lever_arm_is_a_noop():
    nav = _Nav((0.3, -0.2, 1.0))
    assert replay.ref_point_offset_ned(nav, (0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert nav.calls == 0
    up, shifted = ins_plots._ref_pt_up({"ref_pt_up": [0.0, 0.0]}, 2)
    assert not shifted and up == [0.0, 0.0]
    up, shifted = ins_plots._ref_pt_up({}, 3)
    assert not shifted and up == [0.0, 0.0, 0.0]


def test_leverarm_relation_classifies_gnss_vs_score():
    z = (0.0, 0.0, 0.0)
    ant = (-2.0, 0.1, -1.3)
    assert replay.leverarm_relation(z, z) == "none"
    assert replay.leverarm_relation(ant, ant) == "same"
    # Within LEVERARM_MATCH_TOL_M per axis: still the same point.
    near = (ant[0] + 0.5 * replay.LEVERARM_MATCH_TOL_M, ant[1], ant[2])
    assert replay.leverarm_relation(ant, near) == "same"
    far = (ant[0] + 2.0 * replay.LEVERARM_MATCH_TOL_M, ant[1], ant[2])
    assert replay.leverarm_relation(ant, far) == "differ"
    assert replay.leverarm_relation(ant, z) == "score_unset"
    assert replay.leverarm_relation(z, ant) == "gnss_unset"


def test_gui_records_rotated_lever_arms_for_ref_and_fix():
    try:
        import PyQt6  # noqa: F401
    except ImportError:
        print("  skip  PyQt6 not installed")
        return
    from PyQt6 import QtCore
    import inspostgui as gui
    QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
    gnss_la = (0.4, -0.2, -0.9)
    score_la = (-2.0, 0.1, -1.3)
    with tempfile.TemporaryDirectory() as d:
        ds = test_range_stream._dataset(d, stop_after_sec=1e6, with_ranges=False)
        # A 1 Hz reference, so the 10 Hz recorder holds each sample.
        ref_path = os.path.join(ds, next(n for n in os.listdir(ds)
                                         if n.lower() == "ref.csv"))
        with open(ref_path, encoding="utf-8") as f:
            lines = f.readlines()
        kept, last_s = [], None
        for ln in lines:
            if not ln.startswith("#"):
                s = int(ln.split(",")[0]) // 1000000
                if s == last_s:
                    continue
                last_s = s
            kept.append(ln)
        with open(ref_path, "w", encoding="utf-8", newline="") as f:
            f.writelines(kept)
        raw, _cfg_path, data_dir = gui.load_raw_config(ds)
        spec = gui.merge_spec(raw)
        spec["gnss"]["leverarm_frd"] = list(gnss_la)
        spec["score"]["leverarm_frd"] = list(score_la)
        worker = gui.ReplayWorker(spec, data_dir)
        done = {}
        worker.sig_error.connect(lambda msg, details: done.update(error=details or msg))
        worker.sig_finished.connect(lambda res: done.update(results=res))
        worker.run()  # synchronous, this thread
        assert "error" not in done, done.get("error")
    rec = done["results"]["rec"]
    # The two arms differ: reported as info, not as the unset-arm warning.
    la_findings = [(sev, text) for sev, text in done["results"]["findings"]
                   if "leverarm_frd" in text]
    assert [sev for sev, _ in la_findings] == [replay.SEV_INFO], la_findings
    assert "differ" in la_findings[0][1]

    def norm(v):
        return math.sqrt(sum(x * x for x in v))

    # One rotated GNSS lever arm per fix, a rotation of gnss.leverarm_frd.
    assert len(worker.fix_ned) > 10
    assert len(worker.fix_la_ned) == len(worker.fix_ned)
    assert all(abs(norm(v) - norm(gnss_la)) < 1e-9 for v in worker.fix_la_ned)
    # Per recorded sample: the rotated scoring and GNSS lever arms, latched
    # when the reference sample and the fix arrived. A reference sample held
    # over several ticks keeps its shift, so a slower reference is not
    # bent into an arc per sample while the vehicle turns.
    assert len(rec["ref_pt_ned"]) == len(rec["gnss_la_ned"]) == len(rec["t"]) > 100
    n_held = 0
    prev = None
    for ref, la_n, g_n in zip(rec["ref_pos"], rec["ref_pt_ned"], rec["gnss_la_ned"]):
        if ref[0] != ref[0]:
            continue
        assert abs(norm(la_n) - norm(score_la)) < 1e-9
        if g_n[0] == g_n[0]:
            assert abs(norm(g_n) - norm(gnss_la)) < 1e-9
        if prev is not None and ref == prev[0]:
            n_held += 1
            assert la_n == prev[1]
        prev = (ref, la_n)
    assert n_held > 50
    # The last fix's shift is the one the 3D view moves that fix by.
    assert rec["gnss_la_ned"][-1] == list(worker.fix_la_ned[-1])


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except Exception as e:     # noqa: BLE001
                failed += 1
                print("FAIL", name, repr(e))
    sys.exit(1 if failed else 0)
