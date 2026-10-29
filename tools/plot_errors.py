"""Графики ошибок по итогам подбора (дополняют best_comparison.png).

Три PNG в каталоге, заданном --out-dir:

  best_errors.png   — ошибки «Калман − эталон» по J-метрике: |dN|, |dE|,
                      |pos 2D|, |vel| (горизонтальная), |hdg| по времени,
                      с горизонтальными порогами P95 (5 м / 0.5 м/с / 1°),
                      вертикальной границей train/valid (TRAIN_FRAC) и
                      отсечкой SKIP_FIRST_S (первые 30 с в J не входят).
  best_diffs.png    — разности result − reference по всем 9 панелям
                      (курс/тангаж/крен/долгота/широта/высота + vn/vh/ve).
  best_kf_errors.png — внутренние ошибки КФ из errors.txt (15 компонент):
                      координаты (рад→м), скорости, углы (→°), ba, bg.

Сопоставление result→reference — как в метрике J (compute_error в
tune_kalman_v2.py): ближайшая по времени строка result для каждого такта
reference (np.searchsorted). Формат result/reference: 10 колонок
time, lon, lat, alt, heading, pitch, roll, vn, vh, ve (градусы/м/м/с).

Запуск:
    python tools/plot_errors.py
    python tools/plot_errors.py --result tools/result.txt --reference tools/reference.txt \
           --errors tools/errors.txt --out-dir temp_tune_v2

Без аргументов берёт самые свежие result.txt/reference.txt из tools/, build/ и
корня проекта; errors.txt ищется рядом с result.txt.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

# Пороги J (должны совпадать с tune_kalman_v2.py).
TOL_POS = 5.0      # м
TOL_VEL = 0.5      # м/с
TOL_HDG = 1.0      # °
SKIP_FIRST_S = 30.0
TRAIN_FRAC = 0.6

# Столбцы result/reference.
C_LON, C_LAT = 1, 2
C_HDG, C_PITCH, C_ROLL = 4, 5, 6
C_VN, C_VH, C_VE = 7, 8, 9


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.grid": True,
            "grid.linestyle": "-",
            "grid.linewidth": 0.5,
            "grid.alpha": 0.35,
            "axes.linewidth": 0.8,
            "axes.edgecolor": "black",
            "axes.labelsize": 10,
            "axes.titlesize": 11,
            "font.size": 10,
            "legend.fontsize": 9,
            "legend.framealpha": 1.0,
            "legend.edgecolor": "0.7",
            "lines.linewidth": 0.9,
            "xtick.direction": "in",
            "ytick.direction": "in",
        }
    )


def load(path: Path) -> np.ndarray:
    with path.open(encoding="utf-8") as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        try:
            float(line.split()[0])
            return np.loadtxt(lines[i:], dtype=float)
        except (ValueError, IndexError):
            continue
    raise ValueError(f"Не удалось найти данные в {path}")


def find_result_pair(script_dir: Path) -> tuple[Path, Path]:
    """Самые свежие result.txt/reference.txt: tools/, build/, корень проекта."""
    project_dir = script_dir.parent
    search_dirs = [script_dir, project_dir / "build", project_dir]

    found: list[tuple[float, Path, Path]] = []
    for base in search_dirs:
        result_path = base / "result.txt"
        reference_path = base / "reference.txt"
        if result_path.is_file() and reference_path.is_file():
            found.append((result_path.stat().st_mtime, result_path, reference_path))

    if not found:
        raise FileNotFoundError(
            "Не найдены result.txt и reference.txt.\n"
            "Сначала запустите imitator — файлы пишутся в tools/.\n"
            "Либо укажите пути: --result <...> --reference <...>"
        )

    found.sort(key=lambda item: item[0], reverse=True)
    return found[0][1], found[0][2]


def nearest_result_index(t_res: np.ndarray, t_ref: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(t_res, t_ref)
    idx = np.clip(idx, 0, len(t_res) - 1)
    idx_prev = np.clip(idx - 1, 0, len(t_res) - 1)
    closer = np.abs(t_res[idx] - t_ref) <= np.abs(t_res[idx_prev] - t_ref)
    return np.where(closer, idx, idx_prev)


def errors_j_metric(res: np.ndarray, ref: np.ndarray):
    """Ошибки на тактах reference, как в compute_error.

    Возвращает (tt, d_north, d_east, vel_err, hdg_err).
    """
    idx = nearest_result_index(res[:, 0], ref[:, 0])
    r = res[idx]

    d_north = (r[:, C_LAT] - ref[:, C_LAT]) * 111000.0
    d_east = (r[:, C_LON] - ref[:, C_LON]) * 111000.0 * np.cos(np.deg2rad(ref[:, C_LAT]))
    vel_err = np.hypot(r[:, C_VN] - ref[:, C_VN], r[:, C_VE] - ref[:, C_VE])
    hdg = np.mod(r[:, C_HDG] - ref[:, C_HDG] + 180.0, 360.0) - 180.0

    return ref[:, 0], d_north, d_east, vel_err, np.abs(hdg)


def plot_errors(tt, d_north, d_east, vel_err, hdg_err, save_path: Path,
                skip_first_s: float = SKIP_FIRST_S, train_frac: float = TRAIN_FRAC) -> None:
    """best_errors.png — ошибки по J-метрике + пороги P95 и train/valid."""
    configure_style()
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Ошибки Калман − эталон (J-метрика)", fontsize=13, fontweight="bold")

    pos2d = np.hypot(d_north, d_east)

    def draw(ax, y, color, ylabel, tol):
        ax.plot(tt, y, color=color, label="ошибка", linewidth=0.8)
        ax.axhline(tol, color="tab:red", ls="--", linewidth=1.0,
                   label=f"P95 (допуск {tol:g})")
        ax.set_ylabel(ylabel)

    draw(axes[0], pos2d, "#0072bd", "|pos 2D|, м", TOL_POS)
    axes[0].plot(tt, np.abs(d_north), color="#4dbeee", linewidth=0.6, alpha=0.7, label="|dN|")
    axes[0].plot(tt, np.abs(d_east), color="#77ac30", linewidth=0.6, alpha=0.7, label="|dE|")
    draw(axes[1], vel_err, "#d95319", "|vel|, м/с", TOL_VEL)
    draw(axes[2], hdg_err, "#7e2f8e", "|hdg|, °", TOL_HDG)

    for ax in axes:
        ax.axvline(skip_first_s, color="gray", ls=":", linewidth=1.2)
        split = tt[0] + (tt[-1] - tt[0]) * train_frac
        ax.axvline(split, color="gray", ls="--", linewidth=1.2)
        ax.legend(loc="upper right", fontsize=8)
    axes[0].set_title("закрашено: P95-порог; серые линии — skip=30 с и граница train/valid")

    axes[-1].set_xlabel("Время, с")
    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_diffs(res: np.ndarray, ref: np.ndarray, save_path: Path,
               skip_first_s: float = SKIP_FIRST_S, train_frac: float = TRAIN_FRAC) -> None:
    """best_diffs.png — разности result − reference по 6 панелям."""
    configure_style()
    idx = nearest_result_index(res[:, 0], ref[:, 0])
    r = res[idx]
    tt = ref[:, 0]

    diffs = [
        (r[:, C_HDG] - ref[:, C_HDG], "Курс, °"),
        (r[:, C_PITCH] - ref[:, C_PITCH], "Тангаж, °"),
        (r[:, C_ROLL] - ref[:, C_ROLL], "Крен, °"),
        (r[:, C_LON] - ref[:, C_LON], "Долгота, °"),
        (r[:, C_LAT] - ref[:, C_LAT], "Широта, °"),
        (r[:, 3] - ref[:, 3], "Высота, м"),
        (r[:, C_VN] - ref[:, C_VN], "vn, м/с"),
        (r[:, C_VH] - ref[:, C_VH], "vh, м/с"),
        (r[:, C_VE] - ref[:, C_VE], "ve, м/с"),
    ]

    fig, axes = plt.subplots(3, 3, figsize=(14, 10.2), sharex=True)
    fig.suptitle("Разности Калман − эталон (все каналы)", fontsize=13, fontweight="bold")
    for ax, (d, ylabel) in zip(axes.ravel(), diffs):
        ax.plot(tt, d, color="#4dbeee", linewidth=0.8)
        ax.axhline(0.0, color="black", linewidth=0.5)
        ax.axvline(skip_first_s, color="gray", ls=":", linewidth=1.0)
        split = tt[0] + (tt[-1] - tt[0]) * train_frac
        ax.axvline(split, color="gray", ls="--", linewidth=1.0)
        ax.set_ylabel(ylabel)
    for ax in axes[2]:
        ax.set_xlabel("Время, с")
    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_kf_errors(err: np.ndarray, save_path: Path,
                   skip_first_s: float = SKIP_FIRST_S, train_frac: float = TRAIN_FRAC) -> None:
    """best_kf_errors.png — вектор ошибок КФ из errors.txt (15 компонент)."""
    configure_style()
    tt = err[:, 0]
    # x0=широта(рад), x1=долгота(рад), x2=высота(м) — радианные → метры.
    lat_m = err[:, 1] * 111000.0
    lon_m = err[:, 2] * 111000.0
    panels = [
        (np.column_stack([lat_m, lon_m, err[:, 3]]),
         "Ошибки координат КФ, м (широта, долгота, высота)", ["δφ·R", "δλ·R", "δh"]),
        (err[:, [4, 5, 6]], "Ошибки скорости КФ, м/с (N, H, E)", ["δVn", "δVh", "δVe"]),
        (np.degrees(err[:, [7, 8, 9]]), "Ошибки ориентации КФ, ° (ψ, θ, φ)", ["δψ", "δθ", "δφ"]),
        (err[:, [10, 11, 12]], "Смещение акселя КФ, м/с²", ["δba_x", "δba_y", "δba_z"]),
        (err[:, [13, 14, 15]] * 1e9, "Смещение гиро КФ, нрад/с (×1e9)", ["δbg_x", "δbg_y", "δbg_z"]),
    ]

    fig, axes = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    fig.suptitle("Внутренние ошибки КФ (errors.txt)", fontsize=13, fontweight="bold")
    colors = ["#0072bd", "#d95319", "#77ac30"]
    for ax, (rows, title, labels) in zip(axes, panels):
        ax.set_title(title, fontsize=9)
        for k in range(3):
            ax.plot(tt, rows[:, k], color=colors[k], linewidth=0.8, label=labels[k])
        ax.axvline(skip_first_s, color="gray", ls=":", linewidth=1.0)
        split = tt[0] + (tt[-1] - tt[0]) * train_frac
        ax.axvline(split, color="gray", ls="--", linewidth=1.0)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Время, с")
    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Графики ошибок после подбора КФ")
    parser.add_argument("--result", default=None,
                        help="путь к result.txt (по умолчанию — самый свежий в tools/)")
    parser.add_argument("--reference", default=None,
                        help="путь к reference.txt (по умолчанию — рядом с result.txt)")
    parser.add_argument("--errors", default=None,
                        help="путь к errors.txt (по умолчанию — рядом с result.txt)")
    parser.add_argument("--out-dir", default=None,
                        help="каталог для PNG (по умолчанию — рядом с result.txt)")
    parser.add_argument("--skip-first", type=float, default=SKIP_FIRST_S,
                        help="отсечка начала (по умолчанию 30 с)")
    parser.add_argument("--train-frac", type=float, default=TRAIN_FRAC,
                        help="доля train (по умолчанию 0.6)")
    args = parser.parse_args()

    skip_first_s, train_frac = args.skip_first, args.train_frac

    if args.result and args.reference:
        result_path = Path(args.result).resolve()
        reference_path = Path(args.reference).resolve()
    else:
        if args.result or args.reference:
            print("Указан только один путь из пары — ищу свежие файлы автоматически.")
        result_path, reference_path = find_result_pair(script_dir)

    out_dir = Path(args.out_dir).resolve() if args.out_dir else result_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    errors_path = Path(args.errors).resolve() if args.errors else result_path.parent / "errors.txt"

    print(f"result:    {result_path}")
    print(f"reference: {reference_path}")
    print(f"errors:    {errors_path if errors_path.exists() else '(нет)'}")
    print(f"вывод:     {out_dir}")

    res = load(result_path)
    ref = load(reference_path)

    tt, d_n, d_e, ve, he = errors_j_metric(res, ref)
    plot_errors(tt, d_n, d_e, ve, he, out_dir / "best_errors.png",
                skip_first_s, train_frac)
    print(f"Сохранено: {out_dir / 'best_errors.png'}")

    plot_diffs(res, ref, out_dir / "best_diffs.png", skip_first_s, train_frac)
    print(f"Сохранено: {out_dir / 'best_diffs.png'}")

    if errors_path.exists():
        err = load(errors_path)
        plot_kf_errors(err, out_dir / "best_kf_errors.png", skip_first_s, train_frac)
        print(f"Сохранено: {out_dir / 'best_kf_errors.png'}")
    else:
        print(f"(нет {errors_path.name} — график ошибок КФ пропущен)")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1)