#!/usr/bin/env python3
"""Графики как в отчёте: 3 модели VO vs RTK (оверлей + ошибка + панели).

RTK только для рисунка и RMSE. В оценку не подмешивается.

    python3 tools/plot_vo_like_report.py \\
        --gt data/mars/mars_hkairport03_livox_gt.tum \\
        --planimetry results/eval_vo_field/planimetry.tum \\
        --homography results/eval_vo_field/homography.tum \\
        --essential results/eval_vo_field/essential.tum \\
        --out results/eval_vo_field \\
        --title "HKairport03: VO без коррекции по GPS, курс AHRS DJI"
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import numpy as np

plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["axes.unicode_minus"] = False

NAMES = (
    ("planimetry", "планиметрия", "#e67e22"),
    ("homography", "гомография", "#1e8449"),
    ("essential", "существенная", "#c0392b"),
)


def load_xy(path: Path) -> tuple[np.ndarray, np.ndarray]:
    a = np.loadtxt(path, comments="#")
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a[:, 0], a[:, 1:3]


def interp_xy(t_q: np.ndarray, t: np.ndarray, xy: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(t_q, t, xy[:, j]) for j in range(2)])


def path_len(xy: np.ndarray) -> float:
    if len(xy) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def rmse_xy(est: np.ndarray, gt: np.ndarray) -> float:
    err = np.linalg.norm(est - gt, axis=1)
    return float(np.sqrt(np.mean(err ** 2)))


def err_series(est: np.ndarray, gt: np.ndarray) -> np.ndarray:
    return np.linalg.norm(est - gt, axis=1)


def turn_spans(gt: np.ndarray, t: np.ndarray, rate_min: float = 0.18, min_s: float = 8.0):
    """Интервалы разворотов по курсу RTK (только для заливки графика)."""
    if len(gt) < 30 or len(t) < 30:
        return []
    v = np.diff(gt, axis=0)
    heading = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    # окно ~3 с, чтобы шум RTK не заливал весь график
    dt_med = float(np.median(np.diff(t))) if len(t) > 2 else 0.1
    win = max(8, int(round(3.0 / max(dt_med, 1e-3))))
    if len(heading) <= win:
        return []
    dpsi = np.abs(heading[win:] - heading[:-win])
    t_rate = t[1 : 1 + len(dpsi)]
    high = dpsi > 0.55  # ~31° за окно
    spans = []
    i = 0
    while i < len(high):
        if not high[i]:
            i += 1
            continue
        j = i
        while j < len(high) and high[j]:
            j += 1
        t0, t1 = float(t_rate[i]), float(t_rate[min(j, len(t_rate) - 1)])
        if t1 - t0 >= min_s * 0.35:
            pad = 4.0
            spans.append((t0 - pad, t1 + pad))
        i = j
    merged = []
    for a, b in spans:
        if merged and a <= merged[-1][1] + 8.0:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    out = [(a, b) for a, b in merged if b - a >= min_s]
    if not out:
        return []
    cover = sum(b - a for a, b in out)
    dur = float(t[-1] - t[0])
    if dur > 1 and cover > 0.45 * dur:
        return []
    return out


def _start_arrow(ax, gt: np.ndarray):
    if len(gt) < 8:
        return
    p0 = gt[0]
    step = min(25, len(gt) - 1)
    d = gt[step] - p0
    n = np.linalg.norm(d)
    if n < 1.0:
        return
    d = d / n * max(40.0, 0.12 * (gt.max(0) - gt.min(0)).max())
    ax.plot(p0[0], p0[1], "kD", ms=5, zorder=5)
    arr = FancyArrowPatch(
        p0, p0 + d, arrowstyle="-|>", mutation_scale=12,
        color="black", lw=1.2, zorder=5,
    )
    ax.add_patch(arr)


def plot_figure2(
    out: Path,
    t: np.ndarray,
    gt: np.ndarray,
    trajs: dict[str, np.ndarray],
    stats: dict[str, dict],
    title: str,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.6),
                             gridspec_kw={"width_ratios": [1.15, 1.0]})
    ax, ax_e = axes
    t0 = float(t[0])
    t_rel = t - t0
    pg = stats["homography"]["path_gt"] if "homography" in stats else path_len(gt)

    ax.plot(gt[:, 0], gt[:, 1], color="black", lw=1.8, zorder=3,
            label=f"RTK (эталон), {pg:.0f} м")
    for key, ru, col in NAMES:
        e = trajs[key]
        m = stats[key]
        ax.plot(e[:, 0], e[:, 1], color=col, lw=1.15,
                label=f"{ru}, RMS {m['rmse']:.0f} м")
    _start_arrow(ax, gt)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")
    ax.grid(True, alpha=0.28)
    ax.legend(loc="best", fontsize=7.5, framealpha=0.92)
    ax.set_title(title, fontsize=9)

    for key, ru, col in NAMES:
        err = err_series(trajs[key], gt)
        ax_e.plot(t_rel, err, color=col, lw=1.05, label=ru)
    for a, b in turn_spans(gt, t_rel):
        ax_e.axvspan(a, b, color="#d6eaf8", zorder=0)
    ax_e.set_xlabel("время, с")
    ax_e.set_ylabel("ошибка, м")
    ax_e.set_title("Ошибка положения во времени (синим — развороты)", fontsize=9)
    ax_e.grid(True, alpha=0.28)
    ax_e.legend(loc="upper right", fontsize=7.5, framealpha=0.92)
    ax_e.set_ylim(bottom=0)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    fig.savefig(out.with_suffix(".pdf"))
    plt.close(fig)


def plot_figure3(
    out: Path,
    gt: np.ndarray,
    trajs: dict[str, np.ndarray],
    stats: dict[str, dict],
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.4, 4.15), sharex=False, sharey=False)
    for ax, (key, ru, col) in zip(axes, NAMES):
        e = trajs[key]
        m = stats[key]
        ax.plot(gt[:, 0], gt[:, 1], color="black", lw=1.6, label="RTK")
        ax.plot(e[:, 0], e[:, 1], color=col, lw=1.1, label="одометрия")
        ax.set_title(
            f"{ru}\nRMS {m['rmse']:.0f} м, путь {m['path_ratio']:.2f} от эталона",
            fontsize=9,
        )
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("восток, м")
        ax.set_ylabel("север, м")
        ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    fig.savefig(out.with_suffix(".pdf"))
    plt.close(fig)


def stats_of(est: np.ndarray, gt: np.ndarray) -> dict:
    return {
        "rmse": rmse_xy(est, gt),
        "path_est": path_len(est),
        "path_gt": path_len(gt),
        "path_ratio": path_len(est) / max(path_len(gt), 1e-6),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--planimetry", type=Path, required=True)
    ap.add_argument("--homography", type=Path, required=True)
    ap.add_argument("--essential", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", type=str, default="VO vs RTK")
    args = ap.parse_args()

    tg, gxy = load_xy(args.gt)
    paths = {
        "planimetry": args.planimetry,
        "homography": args.homography,
        "essential": args.essential,
    }
    t0, _ = load_xy(args.homography)
    gti = interp_xy(t0, tg, gxy)
    trajs, stats = {}, {}
    for key, p in paths.items():
        tt, xy = load_xy(p)
        trajs[key] = interp_xy(t0, tt, xy)
        stats[key] = stats_of(trajs[key], gti)
        print(f"{key:12s} RMSE {stats[key]['rmse']:.1f} м  "
              f"путь {stats[key]['path_est']:.0f}/{stats[key]['path_gt']:.0f} "
              f"({stats[key]['path_ratio']:.2f})")

    args.out.mkdir(parents=True, exist_ok=True)
    plot_figure2(args.out / "vo_overlay_error.png", t0, gti, trajs, stats, args.title)
    plot_figure3(args.out / "vo_three_panels.png", gti, trajs, stats)
    print(f"записано {args.out / 'vo_overlay_error.png'}")
    print(f"записано {args.out / 'vo_three_panels.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
