#!/usr/bin/env python3
"""Generate tests/udu_takasu_vectors.h: reference results for the UDU filter.

Two kinds of case. Prediction cases: P+ = Phi P Phi' + G diag(Q) G' and
x+ = Phi x in double, against which the Thornton routine is compared.
Update cases: each is one Kalman measurement update with a FULLY correlated
measurement covariance R (dense, no zero off-diagonals), computed in double
precision with the Takasu/Cholesky formulation from KFCore/python (the
independent reference). The Joseph-form vanilla filter from the same package
is run alongside as a cross-check of the reference itself, the generator
aborts if the two disagree.

tests/test_ins_core.c:scenario_udu_vs_takasu feeds the same inputs through
the C pipeline ins.c uses for correlated fixes (decorrelate + kalman_udu in
float) and compares the result with the numbers written here. The vectors
are committed, so nothing needs Python at test time. Rerun via
`make udu-vectors` only when the cases below change.

Storage convention: all matrices are flattened the way the C code reads them
(column-major), except H, which is written row by row. The C routines take
Ht (n x m, column-major), whose memory layout is exactly H row-major.

(c) Jan Zwiener (jan@zwiener.org)
"""

import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "KFCore", "python"))

from kfcore.kalman_takasu import kalman_takasu  # noqa: E402
from kfcore.kalman_vanilla import kalman_vanilla  # noqa: E402

OUT = os.path.join(ROOT, "tests", "udu_takasu_vectors.h")
N_MAX = 15
M_MAX = 6
Q_MAX = 15  # process noise columns


def _cholesky_upper_shim():
    """KFCore calls np.linalg.cholesky(S, upper=True), numpy >= 2.0 only."""
    try:
        np.linalg.cholesky(np.eye(1), upper=True)
        return
    except TypeError:
        pass
    plain = np.linalg.cholesky

    def shim(a, upper=False):
        low = plain(a)
        return low.T if upper else low

    np.linalg.cholesky = shim


def make_state_cov(rng, n, sig_hi=10.0, sig_lo=1e-3):
    """Random SPD covariance with a wide scale spread (position metres down
    to bias terms), dense and strongly cross-correlated like a real filter."""
    scale = np.geomspace(sig_hi, sig_lo, n)  # 1-sigma, default 10 ... 1e-3
    B = rng.standard_normal((n, n))
    C = B @ B.T / n + 0.2 * np.eye(n)
    d = 1.0 / np.sqrt(np.diag(C))
    C = C * d[:, None] * d[None, :]  # unit diagonal
    return C * scale[:, None] * scale[None, :]


def make_meas_cov(rng, m, kind, sigma, rho=0.9):
    if kind == "equicorr":  # every pair of measurements correlated by rho
        C = rho * np.ones((m, m)) + (1.0 - rho) * np.eye(m)
    else:  # random dense correlation, condition number a few hundred
        B = rng.standard_normal((m, m))
        C = B @ B.T + 0.02 * m * np.eye(m)
        d = 1.0 / np.sqrt(np.diag(C))
        C = C * d[:, None] * d[None, :]
    return C * sigma[:, None] * sigma[None, :]


def make_case(seed, n, m, kind, sig_lo, sig_hi, rho, p_hi, p_lo):
    rng = np.random.default_rng(seed)
    P = make_state_cov(rng, n, p_hi, p_lo)
    H = rng.standard_normal((m, n))
    R = make_meas_cov(rng, m, kind, np.geomspace(sig_lo, sig_hi, m), rho)
    x = np.zeros(n)  # error-state filter: the state starts at zero
    # Residual: a draw from the predicted innovation distribution, so the
    # update is a realistic size and not a pathological one.
    S = H @ P @ H.T + R
    dz = np.linalg.cholesky(S) @ rng.standard_normal(m)

    xt, Pt = kalman_takasu(x, P, dz, R, H)
    xv, Pv = kalman_vanilla(x, P, dz, R, H)
    err_x = np.max(np.abs(xt - xv)) / max(np.max(np.abs(xv)), 1e-30)
    err_P = np.max(np.abs(Pt - Pv)) / np.max(np.abs(Pv))
    assert err_x < 1e-9 and err_P < 1e-9, f"reference disagrees: {err_x} {err_P}"
    return dict(n=n, m=m, P=P, H=H, R=R, dz=dz, x=xt, Pp=Pt)


CASES = [
    # name, seed, n, m, R structure, sigma range, rho, P sigma hi, P sigma lo,
    # test tolerance (relative), pipeline may refuse the update (0 or 1)
    ("INS15 6 meas equicorr", 11, 15, 6, "equicorr", 0.5, 2.0, 0.9, 10.0, 1e-3, 1e-4, 0),
    ("INS15 6 meas random", 12, 15, 6, "random", 0.1, 3.0, 0.9, 10.0, 1e-3, 1e-4, 0),
    ("AHRS6 3 meas equicorr", 13, 6, 3, "equicorr", 0.05, 0.5, 0.9, 10.0, 1e-3, 1e-4, 0),
    # P standard deviations over seven decades (variances over fourteen)
    ("INS15 ill-conditioned P", 14, 15, 6, "equicorr", 0.5, 2.0, 0.9, 1e3, 1e-4, 1e-3, 0),
    # R close to singular: condition number of the correlation ~ 6e4 and 6e6
    ("INS15 near-singular R 1e-4", 15, 15, 6, "equicorr", 0.5, 2.0, 1.0 - 1e-4, 10.0, 1e-3, 1e-4, 0),
    ("INS15 near-singular R 1e-6", 16, 15, 6, "equicorr", 0.5, 2.0, 1.0 - 1e-6, 10.0, 1e-3, 1e-3, 1),
]


def make_predict_case(seed, n, r, sparse_g):
    """One covariance prediction P+ = Phi P Phi' + G diag(Q) G' in double."""
    rng = np.random.default_rng(seed)
    P = make_state_cov(rng, n)
    F = rng.standard_normal((n, n))
    dt = 0.1
    Phi = np.eye(n) + dt * F + 0.5 * dt * dt * (F @ F)  # dense, well conditioned
    G = rng.standard_normal((n, r))
    if sparse_g:  # block structure like the INS: noise enters a few states only
        mask = np.zeros((n, r))
        for j in range(r):
            mask[j % n, j] = 1.0
            mask[(j + 3) % n, j] = 1.0
        G = G * mask
    Q = np.geomspace(1e-4, 1e-1, r)
    x = rng.standard_normal(n)
    Pp = Phi @ P @ Phi.T + G @ np.diag(Q) @ G.T
    Pp = 0.5 * (Pp + Pp.T)
    return dict(n=n, r=r, P=P, Phi=Phi, G=G, Q=Q, x=x, xp=Phi @ x, Pp=Pp)


PREDICT_CASES = [
    # name,                          seed, n,  r, sparse G
    ("INS15 dense G, r = n", 21, 15, 15, False),
    ("INS15 sparse G, r < n", 22, 15, 12, True),
    ("AHRS6 dense G", 23, 6, 6, False),
]


def arr(name, a):
    flat = ", ".join(repr(float(v)) for v in a)
    return f"        .{name} = {{{flat}}},"


def pad(v, size):
    out = np.zeros(size)
    out[: v.size] = v
    return out


def main():
    _cholesky_upper_shim()
    lines = [
        "// AUTO-GENERATED TEST VECTORS - DO NOT EDIT",
        "// Generator: scripts/generate_udu_takasu_vectors.py (make udu-vectors)",
        "// Reference: Takasu/Cholesky update in double (KFCore/python), cross-",
        "// checked against the Joseph-form vanilla filter by the generator.",
        "#pragma once",
        "",
        f"#define UDU_TAKASU_N_MAX {N_MAX}",
        f"#define UDU_TAKASU_M_MAX {M_MAX}",
        f"#define UDU_TAKASU_Q_MAX {Q_MAX}",
        "",
        "typedef struct",
        "{",
        "    const char* name;",
        "    int         n, m;",
        "    double      tol_rel;    /* accepted relative error of the float pipeline */",
        "    int         may_reject; /* 1: refusing the update (rc != 0) is acceptable */",
        "    /* column-major, n x n and m x m */",
        "    double P0[UDU_TAKASU_N_MAX * UDU_TAKASU_N_MAX];",
        "    double R[UDU_TAKASU_M_MAX * UDU_TAKASU_M_MAX];",
        "    /* H row by row (m x n) = Ht (n x m) column-major */",
        "    double Ht[UDU_TAKASU_N_MAX * UDU_TAKASU_M_MAX];",
        "    double dz[UDU_TAKASU_M_MAX];",
        "    /* reference result: state after the update, covariance after it */",
        "    double x_post[UDU_TAKASU_N_MAX];",
        "    double P_post[UDU_TAKASU_N_MAX * UDU_TAKASU_N_MAX];",
        "} udu_takasu_case_t;",
        "",
        "static const udu_takasu_case_t udu_takasu_cases[] = {",
    ]
    for name, seed, n, m, kind, lo, hi, rho, p_hi, p_lo, tol, rej in CASES:
        c = make_case(seed, n, m, kind, lo, hi, rho, p_hi, p_lo)
        lines += [
            "    {",
            f'        .name = "{name}",',
            f"        .n = {n}, .m = {m}, .tol_rel = {tol!r}, .may_reject = {rej},",
            arr("P0", pad(c["P"].flatten(order="F"), N_MAX * N_MAX)),
            arr("R", pad(c["R"].flatten(order="F"), M_MAX * M_MAX)),
            arr("Ht", pad(c["H"].flatten(order="C"), N_MAX * M_MAX)),
            arr("dz", pad(c["dz"], M_MAX)),
            arr("x_post", pad(c["x"], N_MAX)),
            arr("P_post", pad(c["Pp"].flatten(order="F"), N_MAX * N_MAX)),
            "    },",
        ]
    lines += [
        "};",
        "",
        "/* Covariance prediction: P_post = Phi P0 Phi' + G diag(Q) G',",
        "   x_post = Phi x0 (all column-major, G is n x r) */",
        "typedef struct",
        "{",
        "    const char* name;",
        "    int         n, r;",
        "    double      P0[UDU_TAKASU_N_MAX * UDU_TAKASU_N_MAX];",
        "    double      Phi[UDU_TAKASU_N_MAX * UDU_TAKASU_N_MAX];",
        "    double      G[UDU_TAKASU_N_MAX * UDU_TAKASU_Q_MAX];",
        "    double      Q[UDU_TAKASU_Q_MAX];",
        "    double      x0[UDU_TAKASU_N_MAX];",
        "    double      x_post[UDU_TAKASU_N_MAX];",
        "    double      P_post[UDU_TAKASU_N_MAX * UDU_TAKASU_N_MAX];",
        "} udu_takasu_predict_case_t;",
        "",
        "static const udu_takasu_predict_case_t udu_takasu_predict_cases[] = {",
    ]
    for name, seed, n, r, sparse in PREDICT_CASES:
        c = make_predict_case(seed, n, r, sparse)
        lines += [
            "    {",
            f'        .name = "{name}",',
            f"        .n = {n}, .r = {r},",
            arr("P0", pad(c["P"].flatten(order="F"), N_MAX * N_MAX)),
            arr("Phi", pad(c["Phi"].flatten(order="F"), N_MAX * N_MAX)),
            arr("G", pad(c["G"].flatten(order="F"), N_MAX * Q_MAX)),
            arr("Q", pad(c["Q"], Q_MAX)),
            arr("x0", pad(c["x"], N_MAX)),
            arr("x_post", pad(c["xp"], N_MAX)),
            arr("P_post", pad(c["Pp"].flatten(order="F"), N_MAX * N_MAX)),
            "    },",
        ]
    lines += ["};", ""]
    with open(OUT, "w", newline="\n") as fh:
        fh.write("\n".join(lines))
    print(f"wrote {os.path.normpath(OUT)} ({len(CASES)} cases)")


if __name__ == "__main__":
    main()
