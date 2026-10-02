"""Графики вектора состояния фильтра Калмана (15 компонент).

Читает kalman15_line2.txt (столбцы 19–35) или старый errors.txt:
  dlat, dlon, dh, dVn, dVh, dVe, dpsi, dtheta, dphi, ba[3], bg[3], dN_m, dE_m

Использование:
    python tools/plot_kalman_state.py [путь_к_файлу]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

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

R_EARTH = 6371000.0
RAD_TO_DEG = 180.0 / np.pi
LAT0_RAD = 47.641468 * np.pi / 180.0
KF_OFFSET = 19


def load(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            start = 1 if parts[0].count(":") == 2 else 0
            try:
                rows.append([float(x) for x in parts[start:]])
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"Не удалось найти данные в {path}")
    return np.asarray(rows, dtype=float)


def find_state_file(script_dir: Path) -> Path:
    project_dir = script_dir.parent
    names = ("kalman15_line2.txt", "errors.txt", "kalman_state.txt")
    search_dirs = [script_dir, project_dir / "build", project_dir]
    found: list[tuple[float, Path]] = []
    for base in search_dirs:
        for name in names:
            path = base / name
            if path.is_file():
                found.append((path.stat().st_mtime, path))
    if not found:
        raise FileNotFoundError(
            "Не найден kalman15_line2.txt.\n"
            "Сначала запустите imitator."
        )
    found.sort(key=lambda item: item[0], reverse=True)
    return found[0][1]


def kf_block(data: np.ndarray) -> tuple[np.ndarray, int]:
    if data.shape[1] >= KF_OFFSET + 17:
        return data, KF_OFFSET
    return data, 1


def north_east(data: np.ndarray, off: int) -> tuple[np.ndarray, np.ndarray]:
    dN_col = off + 15
    dE_col = off + 16
    if data.shape[1] > dE_col:
        return data[:, dN_col], data[:, dE_col]
    dN = data[:, off] * R_EARTH
    dE = data[:, off + 1] * R_EARTH * np.cos(LAT0_RAD)
    return dN, dE


def plot_state(data: np.ndarray, save_path: Path) -> plt.Figure:
    off = kf_block(data)[1]
    t = data[:, 0]
    dN, dE = north_east(data, off)
    dh = data[:, off + 2]
    dVn, dVh, dVe = data[:, off + 3], data[:, off + 4], data[:, off + 5]
    dpsi = data[:, off + 6] * RAD_TO_DEG
    dtheta = data[:, off + 7] * RAD_TO_DEG
    dphi = data[:, off + 8] * RAD_TO_DEG
    ba = data[:, off + 9 : off + 12]
    bg = data[:, off + 12 : off + 15]

    fig, axes = plt.subplots(
        5, 3, figsize=(16, 14), sharex=True, num="Вектор состояния фильтра Калмана"
    )
    fig.suptitle("Вектор состояния фильтра Калмана (x)", fontsize=14)

    panels = [
        (axes[0, 0], dN, "dN (ошибка север), м"),
        (axes[0, 1], dE, "dE (ошибка восток), м"),
        (axes[0, 2], dh, "dh (ошибка высоты), м"),
        (axes[1, 0], dVn, "dVn, м/с"),
        (axes[1, 1], dVh, "dVh, м/с"),
        (axes[1, 2], dVe, "dVe, м/с"),
        (axes[2, 0], dpsi, "dpsi (курс), град"),
        (axes[2, 1], dtheta, "dtheta (тангаж), град"),
        (axes[2, 2], dphi, "dphi (крен), град"),
        (axes[3, 0], ba[:, 0], "ba_x, м/с²"),
        (axes[3, 1], ba[:, 1], "ba_y, м/с²"),
        (axes[3, 2], ba[:, 2], "ba_z, м/с²"),
        (axes[4, 0], bg[:, 0], "bg_x, рад/с"),
        (axes[4, 1], bg[:, 1], "bg_y, рад/с"),
        (axes[4, 2], bg[:, 2], "bg_z, рад/с"),
    ]

    for ax, y, ylabel in panels:
        ax.plot(t, y, color="C0", linewidth=0.8)
        ax.axhline(0.0, color="0.5", linewidth=0.6)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3)

    for ax in axes[4]:
        ax.set_xlabel("Время, с")

    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    print(f"Сохранено: {save_path}")
    return fig


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="График вектора состояния Калмана")
    parser.add_argument("path", nargs="?", default=None, help="Путь к kalman15_line2.txt")
    parser.add_argument("--save-only", action="store_true", help="Только PNG, без окна")
    parser.add_argument("--no-open", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    path = Path(args.path).resolve() if args.path else find_state_file(script_dir)
    interactive = not (args.save_only or args.no_open)
    print(f"state: {path}")

    data = load(path)
    if data.ndim != 2 or data.shape[1] < 16:
        raise ValueError(f"Ожидалось >= 16 столбцов, получено {data.shape}")

    save_path = path.with_name(path.stem + "_plots.png")
    fig = plot_state(data, save_path)

    if interactive:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1)
