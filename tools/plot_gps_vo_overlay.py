#!/usr/bin/env python3
"""GPS vs Sim2-aligned VO overlay (TUM)."""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

GREEN = "#2ca02c"
BLUE = "#1f77b4"


def load_xy(path: Path):
    a = np.loadtxt(path, comments="#")
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a[:, 0], a[:, 1:3]


def associate(t_a, p_a, t_b, p_b, max_dt=0.05):
    idx = np.clip(np.searchsorted(t_b, t_a), 1, len(t_b) - 1)
    pick = np.where(np.abs(t_b[idx] - t_a) < np.abs(t_b[idx - 1] - t_a), idx, idx - 1)
    ok = np.abs(t_b[pick] - t_a) < max_dt
    return p_a[ok], p_b[pick[ok]], t_a[ok]


def umeyama(src, dst, with_scale: bool):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    cov = d.T @ s / max(len(s), 1)
    u, sv, vt = np.linalg.svd(cov)
    w = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        w[1, 1] = -1
    r = u @ w @ vt
    scale = float(np.trace(np.diag(sv) @ w) / max((s ** 2).sum(), 1e-12) * len(s)) if with_scale else 1.0
    t = mu_d - scale * r @ mu_s
    return r, t, scale


def path_len(xy):
    if len(xy) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    tv, pv = load_xy(args.vins)
    tg, pg = load_xy(args.gt)
    ev, eg, _ = associate(tv, pv, tg, pg)
    if len(ev) < 10:
        raise SystemExit(f"too few matches: {len(ev)}")

    re, te, _ = umeyama(ev, eg, False)
    se2 = (re @ ev.T).T + te
    r, t, scale = umeyama(ev, eg, True)
    sim2 = scale * (r @ ev.T).T + t

    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    ax.plot(eg[:, 0], eg[:, 1], color=GREEN, lw=1.7, label="GPS (эталон)")
    ax.plot(sim2[:, 0], sim2[:, 1], color=BLUE, lw=1.2, label="VINS")
    ax.plot(eg[0, 0], eg[0, 1], marker="s", ms=9, color="#8e44ad", zorder=5,
            markeredgecolor="white", markeredgewidth=0.6, label="старт")
    ax.plot(eg[-1, 0], eg[-1, 1], marker="*", ms=14, color=GREEN, zorder=5, label="конец GPS")
    ax.plot(sim2[-1, 0], sim2[-1, 1], marker="*", ms=14, color=BLUE, zorder=5, label="конец VINS")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend()
    if args.title:
        ax.set_title(args.title)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=140)
    plt.close(fig)

    ratio = path_len(ev) / max(path_len(eg), 1e-6)
    med = float(np.median(np.linalg.norm(se2 - eg, axis=1)))
    end_dist = float(np.linalg.norm(sim2[-1] - eg[-1]))
    print(f"path_ratio {ratio:.3f}")
    print(f"SE2 median {med:.3f}")
    print(f"Sim2 scale {scale:.3f}")
    print(f"end dist {end_dist:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
