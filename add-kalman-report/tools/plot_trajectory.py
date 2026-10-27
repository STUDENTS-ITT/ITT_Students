"""Сравнение решения Калмана с эталоном.

Запуск (двойной клик или из IDE):
    python plot_trajectory.py

Ищет самые свежие result.txt и reference.txt в tools/, build/ и корне проекта.
По умолчанию открывает интерактивное окно (как figure в MATLAB) с панелью
масштабирования, перемещения и сохранения. PNG пишется рядом автоматически.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

_SAVE_ONLY = "--save-only" in sys.argv or "--no-open" in sys.argv
if _SAVE_ONLY:
    matplotlib.use("Agg")
else:
    for _backend in ("TkAgg", "Qt5Agg", "QtAgg", "WXAgg"):
        try:
            matplotlib.use(_backend, force=True)
            break
        except (ImportError, ValueError):
            continue

import matplotlib.pyplot as plt
import numpy as np



def configure_matlab_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "figure.edgecolor": "white",
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
            "Сначала пересоберите и запустите imitator — файлы пишутся в tools/."
        )

    found.sort(key=lambda item: item[0], reverse=True)
    return found[0][1], found[0][2]


def downsample_result(result: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Урезает плотный result (400 Гц) для отзывчивого интерактивного окна."""
    if len(result) <= 5 * len(reference):
        return result
    step = max(1, len(result) // (2 * len(reference)))
    return result[::step]


def plot(result: np.ndarray, reference: np.ndarray, save_path: Path) -> plt.Figure:
    configure_matlab_style()

    result_plot = downsample_result(result, reference)
    t_r = result_plot[:, 0]
    t_s = reference[:, 0]

    fig, axes = plt.subplots(2, 3, figsize=(14, 7.5), sharex=True, num="Калман vs эталон")
    fig.suptitle("Калман vs эталон", fontsize=13, fontweight="bold")

    panels = [
        (axes[0, 0], 4, 4, "Курс, град"),
        (axes[0, 1], 5, 5, "Тангаж, град"),
        (axes[0, 2], 6, 6, "Крен, град"),
        (axes[1, 0], 1, 1, "Долгота, град"),
        (axes[1, 1], 2, 2, "Широта, град"),
        (axes[1, 2], 3, 3, "Высота, м"),
    ]

    kalman_color = "#0072bd"
    ref_color = "#d95319"

    for ax, col_r, col_s, ylabel in panels:
        ax.plot(t_r, result_plot[:, col_r], label="Калман", color=kalman_color, linewidth=0.9)
        ax.plot(t_s, reference[:, col_s], label="Эталон", color=ref_color, linewidth=0.9, alpha=0.85)
        ax.set_ylabel(ylabel)
        ax.legend(loc="best")
        ax.grid(True)
        ax.set_box_aspect(None)

    for ax in axes[1]:
        ax.set_xlabel("Время, с")

    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


def main() -> int:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="График Калман vs эталон")
    parser.add_argument("result", nargs="?", default=None, help="Путь к result.txt")
    parser.add_argument("reference", nargs="?", default=None, help="Путь к reference.txt")
    parser.add_argument(
        "-o",
        "--output",
        default=str(script_dir / "result_comparison.png"),
        help="Путь к PNG",
    )
    parser.add_argument(
        "--save-only",
        action="store_true",
        help="Только сохранить PNG, без интерактивного окна",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()

    if args.result and args.reference:
        result_path = Path(args.result).resolve()
        reference_path = Path(args.reference).resolve()
    else:
        result_path, reference_path = find_result_pair(script_dir)

    save_path = Path(args.output).resolve()
    interactive = not (args.save_only or args.no_open)

    print(f"result:    {result_path}")
    print(f"reference: {reference_path}")

    result = load(result_path)
    reference = load(reference_path)
    fig = plot(result, reference, save_path)

    print(f"Сохранено: {save_path}")
    if interactive:
        print("Интерактивное окно: масштаб, перемещение, домой — на панели инструментов.")
        plt.show()
    else:
        plt.close(fig)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        input("Нажмите Enter...")
        raise SystemExit(1)
