#!/usr/bin/env python3
"""Наложение траектории VINS на эталон RTK в плане + диагностика ухода курса.

ATE после SE(3)-выравнивания не различает две разные болезни: неверный масштаб
и накопление курсовой ошибки. Здесь они разделены явно — Sim2 даёт масштаб,
SE2 (без масштаба) даёт остаточную форму, а график курса показывает, растёт
ли рассогласование линейно со временем.

    python3 tools/plot_xy_vs_rtk.py \
        --vins results/eval_mars_long_b10/vins.tum \
        --gt data/mars/mars_hkairport03_livox_gt.tum \
        --out results/eval_mars_long_b10/xy_vs_rtk.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    a = np.loadtxt(path, comments="#")
    return a[:, 0], a[:, 1:4]


def associate(t_a, p_a, t_b, p_b, max_dt=0.05):
    idx = np.searchsorted(t_b, t_a)
    idx = np.clip(idx, 1, len(t_b) - 1)
    pick = np.where(np.abs(t_b[idx] - t_a) < np.abs(t_b[idx - 1] - t_a), idx, idx - 1)
    ok = np.abs(t_b[pick] - t_a) < max_dt
    return p_a[ok], p_b[pick[ok]], t_a[ok]


def umeyama_2d(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    """Оптимальные поворот вокруг Z, сдвиг и (опционально) масштаб."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    cov = d.T @ s / len(s)
    u, sv, vt = np.linalg.svd(cov)
    w = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        w[1, 1] = -1
    r = u @ w @ vt
    scale = float(np.trace(np.diag(sv) @ w) / (s**2).sum() * len(s)) if with_scale else 1.0
    t = mu_d - scale * r @ mu_s
    return r, t, scale


def path_len(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, required=True)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    t_v, p_v = load_tum(args.vins)
    t_g, p_g = load_tum(args.gt)
    pv, pg, tt = associate(t_v, p_v, t_g, p_g)
    if len(pv) < 10:
        raise SystemExit(f"Слишком мало общих меток времени: {len(pv)}")

    r_s, t_s, scale = umeyama_2d(pv[:, :2], pg[:, :2], with_scale=True)
    r_e, t_e, _ = umeyama_2d(pv[:, :2], pg[:, :2], with_scale=False)
    xy_sim = (scale * (r_s @ pv[:, :2].T).T) + t_s
    xy_se2 = (r_e @ pv[:, :2].T).T + t_e

    err_se2 = np.linalg.norm(xy_se2 - pg[:, :2], axis=1)
    rel = tt - tt[0]

    # Курс по направлению движения: растущий разрыв = уход по yaw.
    def course(p, win=10):
        d = p[win:] - p[:-win]
        return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))

    c_v, c_g = course(xy_se2), course(pg[:, :2])
    n = min(len(c_v), len(c_g))
    dyaw = np.degrees(np.unwrap(c_v[:n] - c_g[:n]))
    dyaw -= dyaw[0]

    end_dist = float(np.linalg.norm(pv[-1, :2] - pg[-1, :2]))

    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ax[0].plot(pg[:, 0], pg[:, 1], "k-", lw=2, label="RTK (эталон)")
    ax[0].plot(pv[:, 0], pv[:, 1], "-", color="tab:blue", lw=1.2, label="VINS (raw)")
    ax[0].plot(
        xy_sim[:, 0], xy_sim[:, 1], "--", color="tab:cyan", lw=1.0, alpha=0.85,
        label=f"VINS Sim2 (SE2 med {np.median(err_se2):.0f} м)",
    )
    ax[0].plot(pg[0, 0], pg[0, 1], "go", ms=9, label="старт")
    ax[0].plot(pg[-1, 0], pg[-1, 1], marker="*", ms=14, color="black", zorder=5,
               label="конец RTK")
    ax[0].plot(pv[-1, 0], pv[-1, 1], marker="*", ms=14, color="tab:blue", zorder=5,
               label=f"конец VINS ({end_dist:.1f} м)")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("X, м")
    ax[0].set_ylabel("Y, м")
    ax[0].legend(loc="best", fontsize=9)
    ax[0].grid(alpha=0.3)
    ax[0].set_title(args.title or args.vins.parent.name)

    ax[1].plot(rel[: len(dyaw)], dyaw, color="tab:purple")
    ax[1].set_xlabel("время, с")
    ax[1].set_ylabel("рассогласование курса, °")
    ax[1].grid(alpha=0.3)
    ax[1].set_title("уход курса относительно RTK")
    ax1b = ax[1].twinx()
    ax1b.plot(rel, err_se2, color="tab:orange", alpha=0.5)
    ax1b.set_ylabel("ошибка положения (SE2), м", color="tab:orange")

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)

    raw_step = np.linalg.norm(np.diff(pv[:, :2], axis=0), axis=1)
    frac_15 = float(np.mean(np.abs(raw_step - 1.5) < 5e-4)) if len(raw_step) else 0.0

    print(f"точек {len(pv)}, длительность {rel[-1]:.0f} с")
    print(f"путь VINS {path_len(pv):.1f} м, RTK {path_len(pg):.1f} м, "
          f"отношение {path_len(pv) / path_len(pg):.3f}")
    print(f"шагов XY ровно 1.500 м: {100 * frac_15:.1f}%")
    print(f"масштаб Sim2 {scale:.3f}")
    print(f"ошибка XY после SE2: медиана {np.median(err_se2):.1f} м, "
          f"RMS {np.sqrt((err_se2**2).mean()):.1f} м, max {err_se2.max():.1f} м")
    print(f"уход курса: конец {dyaw[-1]:+.1f}°, скорость {dyaw[-1] / rel[len(dyaw) - 1] * 60:+.1f} °/мин")
    print(f"концы (raw): RTK ({pg[-1, 0]:.1f}, {pg[-1, 1]:.1f}), "
          f"VINS ({pv[-1, 0]:.1f}, {pv[-1, 1]:.1f}), dist {end_dist:.1f} м")
    print(f"график: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
