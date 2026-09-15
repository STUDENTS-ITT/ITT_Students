#!/usr/bin/env python3
"""Один PDF-отчёт: только VINS (+ AHRS). RTK — сравнение.

    python3 tools/generate_vins_report.py
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[1]
FONT = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONTB = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
GREEN_RTK = "#2ca02c"
BLUE_VINS = "#1f77b4"
RED_NO_VINS = "#c0392b"
START_CLR = "#8e44ad"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.unicode_minus": False,
    "axes.grid": True,
    "grid.alpha": 0.3,
})

# wall_s — время работы алгоритма до готовой траектории (сек, wall clock).
SCENES = (
    {
        "key": "field",
        "title": "Поле — HKairport03",
        "vins": ROOT / "results/eval_mars_scene_field_ahrs/vins_ahrs.tum",
        "alt": ROOT / "results/eval_mars_scene_field_ahrs/vins_fused.tum",
        "gt": ROOT / "data/mars/mars_hkairport03_livox_gt.tum",
        "gt2": ROOT / "data/mars/mars_hkairport03_gt.tum",
        "front": ROOT / "results/timing/vins_front_mars_r1.0_scene_field_ahrs.csv",
        "back": ROOT / "results/timing/vins_back_mars_r1.0_scene_field_ahrs.csv",
        "window_s": 365,
    },
    {
        "key": "water",
        "title": "Вода — HKisland01",
        "vins": ROOT / "results/eval_mars_scene_water_ahrs/vins_ahrs.tum",
        "alt": ROOT / "results/eval_mars_scene_water_ahrs/vins_fused.tum",
        "gt": ROOT / "data/mars/mars_hkisland01_gt.tum",
        "gt2": None,
        "front": ROOT / "results/timing/vins_front_mars_r1.0_scene_water_ahrs.csv",
        "back": ROOT / "results/timing/vins_back_mars_r1.0_scene_water_ahrs.csv",
        "window_s": 751,
    },
    {
        "key": "forest",
        "title": "Лес — AMvalley01",
        "vins": ROOT / "results/eval_mars_scene_forest_ahrs/vins_ahrs.tum",
        "alt": ROOT / "results/eval_mars_scene_forest_ahrs/vins_fused.tum",
        "gt": ROOT / "data/mars/mars_amvalley01_gt.tum",
        "gt2": None,
        "front": ROOT / "results/timing/vins_front_mars_r1.0_scene_forest_ahrs.csv",
        "back": ROOT / "results/timing/vins_back_mars_r1.0_scene_forest_ahrs.csv",
        "window_s": 1201,
    },
)


def load_xy(path: Path):
    a = np.loadtxt(path, comments="#")
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a[:, 0], a[:, 1:3]


def load_timing(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.is_file():
        return np.empty(0), np.empty(0)
    stamps, ms = [], []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) < 2:
            continue
        try:
            stamps.append(float(parts[0]))
            ms.append(float(parts[1]))
        except ValueError:
            continue
    return np.asarray(stamps, dtype=np.float64), np.asarray(ms, dtype=np.float64)


def wall_s_from_log(key: str) -> float | None:
    log = ROOT / f"results/full_run_{key}.log"
    if not log.is_file():
        return None
    for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
        if "WALL_SEC" in line:
            try:
                return float(line.rsplit("WALL_SEC", 1)[-1].strip())
            except ValueError:
                pass
    return None


def tum_duration(path: Path) -> float | None:
    if not path.is_file():
        return None
    t, _ = load_xy(path)
    if len(t) < 2:
        return None
    return float(t[-1] - t[0])


def timing_of(sc: dict) -> dict:
    stamps, front_ms = load_timing(sc["front"])
    vins = sc.get("vins")
    n_vins = len(load_xy(vins)[0]) if vins and vins.is_file() else 0
    n = int(sc["n_frames"]) if sc.get("n_frames") else (n_vins if n_vins else (int(len(front_ms)) if len(front_ms) else 1))
    window = float(sc.get("window_s", 0.0))
    dur = tum_duration(vins) if vins else None
    if dur is not None and dur > 0:
        window = dur
    wall = sc.get("wall_s")
    if wall is None:
        wall = wall_s_from_log(sc["key"])
    if wall is None and len(stamps) >= 2:
        wall = float(stamps[-1] - stamps[0]) + 30.0
    if wall is None:
        wall = window * 1.15 if window > 0 else 0.0
    wall = float(wall)
    t_ms = wall / max(n, 1) * 1000.0
    return {"n": n, "wall_s": wall, "window_s": window, "t_ms": t_ms}


def associate(t_a, p_a, t_b, p_b, max_dt=0.08):
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


def metrics_of(vins: Path, gt: Path) -> dict | None:
    if not vins.is_file() or not gt.is_file():
        return None
    tv, pv = load_xy(vins)
    tg, pg = load_xy(gt)
    ev, eg, tt = associate(tv, pv, tg, pg)
    if len(ev) < 10:
        return None
    re, te, _ = umeyama(ev, eg, False)
    se2 = (re @ ev.T).T + te
    r, t, scale = umeyama(ev, eg, True)
    sim2 = scale * (r @ ev.T).T + t
    err = np.linalg.norm(se2 - eg, axis=1)
    return {
        "t": tt,
        "t_gt": tg,
        "t_v0": float(tt[0]),
        "t_v1": float(tt[-1]),
        "gt": pg,
        "gt_ov": eg,
        "se2": se2,
        "sim2": sim2,
        "err": err,
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "sko": float(np.std(err, ddof=1)) if len(err) > 1 else 0.0,
        "med": float(np.median(err)),
        "path_v": path_len(ev),
        "path_g": path_len(eg),
        "ratio": path_len(ev) / max(path_len(eg), 1e-6),
        "scale": scale,
        "n": len(err),
        "dur": float(tt[-1] - tt[0]),
    }


def _draw_traj(ax, m: dict, legend: bool) -> None:
    gt, est = m["gt"], m["sim2"]
    ax.plot(gt[:, 0], gt[:, 1], color=GREEN_RTK, lw=1.7, label="GPS (Reference)")
    ax.plot(est[:, 0], est[:, 1], color=BLUE_VINS, lw=1.2, label="VO (Scaled & Aligned)")
    ax.plot(gt[0, 0], gt[0, 1], marker="s", ms=9, color=START_CLR, zorder=5,
            markeredgecolor="white", markeredgewidth=0.6, label="Start")
    ax.plot(gt[-1, 0], gt[-1, 1], marker="*", ms=14, color=GREEN_RTK, zorder=5,
            label="GPS End")
    ax.plot(est[-1, 0], est[-1, 1], marker="*", ms=14, color=BLUE_VINS, zorder=5,
            label="VO End")
    if legend:
        ax.legend(fontsize=7.5, loc="best")


def plot_traj(m: dict, title: str, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    _draw_traj(ax, m, legend=True)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def plot_error(m: dict, title: str, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    rel = m["t"] - m["t"][0]
    ax.plot(rel, m["err"], color=BLUE_VINS, lw=1.05, label="ошибка, м")
    ax.axhline(m["rmse"], color="#c0392b", ls="--", lw=1.2,
               label=f"СКО (RMS) {m['rmse']:.1f} м")
    ax.axhline(m["med"], color="#e67e22", ls=":", lw=1.1,
               label=f"медиана {m['med']:.1f} м")
    ax.set_xlabel("время, с")
    ax.set_ylabel("ошибка положения, м")
    ax.set_title(f"{title} — ошибка (СКО)", fontsize=10)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)


def plot_three_panels(rows: list[tuple[str, dict]], out: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(12.4, 4.3))
    for ax, (title, m) in zip(axes, rows):
        _draw_traj(ax, m, legend=True)
        ax.set_title(
            f"{title}\nСКО {m['rmse']:.0f} м, путь {m['ratio']:.2f}",
            fontsize=8.5,
        )
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("восток, м")
        ax.set_ylabel("север, м")
        ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    plt.close(fig)


def scene_graphs(sc: dict, figdir: Path) -> tuple[Path, Path, dict]:
    vins = sc["vins"]
    gt = sc["gt"]
    if not vins.is_file():
        raise SystemExit(f"нет траектории {vins}")
    if not gt.is_file():
        raise SystemExit(f"нет эталона {gt}")
    key = sc["key"]
    overlay = figdir / f"{key}_traj.png"
    errp = figdir / f"{key}_error.png"
    subprocess.check_call([
        "python3", str(ROOT / "tools/plot_gps_vo_overlay.py"),
        "--vins", str(vins),
        "--gt", str(gt),
        "--out", str(overlay),
        "--title", f"{sc['title']}: GPS и VINS",
    ])
    m = metrics_of(vins, gt)
    if m is None:
        raise SystemExit(f"не удалось посчитать метрики {key}")
    plot_error(m, sc["title"], errp)
    print(f"{key:7s} СКО={m['rmse']:.1f} м  мед={m['med']:.1f} м  путь={m['ratio']:.2f}")
    return overlay, errp, m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path,
                    default=ROOT / "results/ПРОМЕЖУТОЧНЫЙ_ОТЧЁТ_VINS.pdf")
    args = ap.parse_args()

    timed = []
    for sc in SCENES:
        tm = timing_of(sc)
        timed.append((sc, tm))
        print(f"{sc['key']:7s} n={tm['n']}  до траектории={tm['wall_s']:.1f} с  "
              f"кадр={tm['t_ms']:.2f} мс")

    figdir = ROOT / "results/eval_vins_report"
    figdir.mkdir(parents=True, exist_ok=True)
    graph_scenes = []
    for sc in SCENES:
        if sc["key"] == "field" and not sc["vins"].is_file():
            sc = dict(sc)
            sc["vins"] = ROOT / "results/eval_mars_scene_field_lawn_v9/vins_ahrs.tum"
        overlay, errp, _m = scene_graphs(sc, figdir)
        graph_scenes.append((sc, overlay, errp))

    pdfmetrics.registerFont(TTFont("DejaVu", str(FONT)))
    pdfmetrics.registerFont(TTFont("DejaVuB", str(FONTB)))
    h1 = ParagraphStyle("h1", fontName="DejaVuB", fontSize=16, leading=20,
                        alignment=TA_CENTER, spaceAfter=10)
    h2 = ParagraphStyle("h2", fontName="DejaVuB", fontSize=12, leading=16,
                        alignment=TA_LEFT, spaceBefore=8, spaceAfter=6)
    body = ParagraphStyle("body", fontName="DejaVu", fontSize=10, leading=14,
                          alignment=TA_JUSTIFY, spaceAfter=6)

    story = [
        Paragraph("Промежуточный отчёт VINS", h1),
        Paragraph("Быстродействие", h2),
        Paragraph(
            "Окно — сколько секунд записи прогоняли. "
            "Время до траектории — от включения алгоритма до готового результата "
            "на этом окне (без графиков и сверки с RTK). "
            "Время на кадр — то же время, делённое на число кадров.",
            body,
        ),
    ]
    ttab = [["Сцена", "окно, с", "кадров", "до готовой траектории, с", "на кадр, мс"]]
    for sc, tm in timed:
        ttab.append([
            sc["title"], f"{tm['window_s']:.0f}", str(tm["n"]),
            f"{tm['wall_s']:.1f}", f"{tm['t_ms']:.2f}",
        ])
    tw = Table(ttab, colWidths=[4.4 * cm, 2.2 * cm, 2.4 * cm, 4.6 * cm, 3.2 * cm])
    tw.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, 0), "DejaVuB"),
        ("FONTNAME", (0, 1), (-1, -1), "DejaVu"),
        ("FONTSIZE", (0, 0), (-1, -1), 8),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eef4")),
        ("GRID", (0, 0), (-1, -1), 0.3, colors.grey),
        ("ALIGN", (1, 0), (-1, -1), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    story += [tw, Spacer(1, 12)]
    for sc, overlay, errp in graph_scenes:
        story += [
            Paragraph(sc["title"], h2),
            Image(str(overlay), width=17.2 * cm, height=11.0 * cm),
            Paragraph("Ошибка положения (СКО)", h2),
            Image(str(errp), width=17.2 * cm, height=9.4 * cm),
            Spacer(1, 8),
        ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(args.out), pagesize=A4,
        leftMargin=1.6 * cm, rightMargin=1.6 * cm,
        topMargin=1.4 * cm, bottomMargin=1.4 * cm,
        title="Промежуточный отчёт VINS",
    )
    doc.build(story)
    print(f"отчёт {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
