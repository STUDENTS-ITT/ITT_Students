"""Сравнение решения Калмана с эталоном GPS/СНС.

Читает один файл kalman15_line2.txt:
  timestamp (ЧЧ:ММ:СС), time (с), 9 столбцов БИНС, 9 столбцов СНС, 15 ошибок Калмана, dN, dE.

Запуск:
    python plot_trajectory.py
    python plot_trajectory.py tools/kalman15_line2.txt
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

COL_TIME = 0
COL_LON = 1
COL_LAT = 2
COL_ALT = 3
COL_HDG = 4
COL_PITCH = 5
COL_ROLL = 6
COL_VN = 7
COL_VH = 8
COL_VE = 9
COL_LON_GPS = 10
COL_LAT_GPS = 11
COL_ALT_GPS = 12
COL_HDG_GPS = 13
COL_PITCH_GPS = 14
COL_ROLL_GPS = 15
COL_VN_GPS = 16
COL_VH_GPS = 17
COL_VE_GPS = 18
N_COMBINED = 19


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


def find_output_file(script_dir: Path) -> Path:
    project_dir = script_dir.parent
    names = ("kalman15_line2.txt",)
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
            "Сначала пересоберите и запустите imitator — файл пишется в tools/."
        )
    found.sort(key=lambda item: item[0], reverse=True)
    return found[0][1]


def wrap_deg(angle: np.ndarray) -> np.ndarray:
    return (angle + 180.0) % 360.0 - 180.0


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def downsample(data: np.ndarray, max_points: int) -> np.ndarray:
    if len(data) <= max_points:
        return data
    step = max(1, len(data) // max_points)
    return data[::step]


def gps_update_mask(data: np.ndarray) -> np.ndarray:
    gps = data[:, COL_LON_GPS : COL_VE_GPS + 1]
    changed = np.ones(len(data), dtype=bool)
    if len(data) > 1:
        changed[1:] = np.any(np.abs(gps[1:] - gps[:-1]) > 1e-12, axis=1)
    return changed


def errors(data: np.ndarray) -> dict[str, np.ndarray]:
    dlat = np.deg2rad(data[:, COL_LAT] - data[:, COL_LAT_GPS])
    dlon = np.deg2rad(data[:, COL_LON] - data[:, COL_LON_GPS])
    north = dlat * R_EARTH
    east = dlon * R_EARTH * np.cos(np.deg2rad(data[:, COL_LAT_GPS]))
    return {
        "lon": data[:, COL_LON] - data[:, COL_LON_GPS],
        "lat": data[:, COL_LAT] - data[:, COL_LAT_GPS],
        "alt": data[:, COL_ALT] - data[:, COL_ALT_GPS],
        "north_m": north,
        "east_m": east,
        "horiz_m": np.hypot(north, east),
        "vn": data[:, COL_VN] - data[:, COL_VN_GPS],
        "vh": data[:, COL_VH] - data[:, COL_VH_GPS],
        "ve": data[:, COL_VE] - data[:, COL_VE_GPS],
        "hdg": wrap_deg(data[:, COL_HDG] - data[:, COL_HDG_GPS]),
        "pitch": data[:, COL_PITCH] - data[:, COL_PITCH_GPS],
        "roll": data[:, COL_ROLL] - data[:, COL_ROLL_GPS],
    }


def print_stats(err: dict[str, np.ndarray]) -> None:
    rows = [
        ("Север, м", err["north_m"]),
        ("Восток, м", err["east_m"]),
        ("Горизонт, м", err["horiz_m"]),
        ("Высота, м", err["alt"]),
        ("Vn, м/с", err["vn"]),
        ("Vh, м/с", err["vh"]),
        ("Ve, м/с", err["ve"]),
        ("Курс, град", err["hdg"]),
        ("Тангаж, град", err["pitch"]),
        ("Крен, град", err["roll"]),
    ]
    print("RMSE  Kalman - GPS")
    print(f"{'величина':<16} {'RMSE':>12} {'макс |e|':>12}")
    for name, values in rows:
        print(f"{name:<16} {rmse(values):12.4f} {np.max(np.abs(values)):12.4f}")


def plot_comparison(data: np.ndarray, gps: np.ndarray, save_path: Path) -> plt.Figure:
    result_plot = downsample(data, 40000)
    t_r = result_plot[:, COL_TIME]
    t_s = gps[:, COL_TIME]

    fig, axes = plt.subplots(
        3, 3, figsize=(16, 11), sharex=True, num="Калман (БИНС) и GPS"
    )
    fig.suptitle("Калман (БИНС) и GPS на одних графиках", fontsize=14)

    panels = [
        (axes[0, 0], COL_LON, COL_LON_GPS, "Долгота, град"),
        (axes[0, 1], COL_LAT, COL_LAT_GPS, "Широта, град"),
        (axes[0, 2], COL_ALT, COL_ALT_GPS, "Высота, м"),
        (axes[1, 0], COL_VN, COL_VN_GPS, "Скорость север, м/с"),
        (axes[1, 1], COL_VH, COL_VH_GPS, "Скорость вертикаль, м/с"),
        (axes[1, 2], COL_VE, COL_VE_GPS, "Скорость восток, м/с"),
        (axes[2, 0], COL_HDG, COL_HDG_GPS, "Курс, град"),
        (axes[2, 1], COL_PITCH, COL_PITCH_GPS, "Тангаж, град"),
        (axes[2, 2], COL_ROLL, COL_ROLL_GPS, "Крен, град"),
    ]

    kalman_color = "#0072bd"
    gps_color = "#d95319"

    for ax, col_kf, col_gps, ylabel in panels:
        ax.plot(t_r, result_plot[:, col_kf], label="Калман", color=kalman_color, linewidth=1.2)
        ax.plot(t_s, gps[:, col_gps], label="GPS", color=gps_color, linewidth=1.0, alpha=0.85)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best")

    for ax in axes[2]:
        ax.set_xlabel("Время, с")

    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    print(f"Сохранено: {save_path}")
    return fig


def plot_errors(t: np.ndarray, err: dict[str, np.ndarray], save_path: Path) -> plt.Figure:
    fig, axes = plt.subplots(3, 3, figsize=(16, 11), sharex=True, num="Ошибка Калман - GPS")
    fig.suptitle("Ошибка Калман - GPS", fontsize=14)

    panels = [
        (axes[0, 0], err["north_m"], "Ошибка север, м"),
        (axes[0, 1], err["east_m"], "Ошибка восток, м"),
        (axes[0, 2], err["alt"], "Ошибка высоты, м"),
        (axes[1, 0], err["vn"], "Ошибка Vn, м/с"),
        (axes[1, 1], err["vh"], "Ошибка Vh, м/с"),
        (axes[1, 2], err["ve"], "Ошибка Ve, м/с"),
        (axes[2, 0], err["hdg"], "Ошибка курса, град"),
        (axes[2, 1], err["pitch"], "Ошибка тангажа, град"),
        (axes[2, 2], err["roll"], "Ошибка крена, град"),
    ]

    for ax, values, ylabel in panels:
        ax.plot(t, values, color="C3", linewidth=1.0)
        ax.axhline(0.0, color="0.4", linewidth=0.8)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    for ax in axes[2]:
        ax.set_xlabel("Время, с")

    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    print(f"Сохранено: {save_path}")
    return fig


def plot_map(data: np.ndarray, gps: np.ndarray, save_path: Path) -> plt.Figure:
    result_plot = downsample(data, 40000)
    fig, ax = plt.subplots(figsize=(8, 8), num="Горизонтальная траектория")
    ax.plot(
        gps[:, COL_LON_GPS],
        gps[:, COL_LAT_GPS],
        label="GPS",
        color="#d95319",
        linewidth=1.4,
    )
    ax.plot(
        result_plot[:, COL_LON],
        result_plot[:, COL_LAT],
        label="Калман",
        color="#0072bd",
        linewidth=1.1,
    )
    ax.set_xlabel("Долгота, град")
    ax.set_ylabel("Широта, град")
    ax.set_title("Горизонтальная траектория")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    print(f"Сохранено: {save_path}")
    return fig


def main() -> int:
    script_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Графики Калман vs GPS")
    parser.add_argument("path", nargs="?", default=None, help="Путь к kalman15_line2.txt")
    parser.add_argument(
        "-o",
        "--output-dir",
        default=None,
        help="Каталог для PNG (по умолчанию рядом с файлом)",
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

    path = Path(args.path).resolve() if args.path else find_output_file(script_dir)
    out_dir = Path(args.output_dir).resolve() if args.output_dir else path.parent
    stem = out_dir / path.stem
    interactive = not (args.save_only or args.no_open)

    print(f"file: {path}")

    data = load(path)
    if data.ndim != 2 or data.shape[1] < N_COMBINED:
        raise ValueError(
            f"kalman15_line2.txt: ожидалось >= {N_COMBINED} столбцов, получено {data.shape}"
        )

    gps = data[gps_update_mask(data)]
    err = errors(gps)
    print_stats(err)

    configure_matlab_style()
    figs = [
        plot_comparison(data, gps, Path(str(stem) + "_comparison.png")),
        plot_errors(gps[:, COL_TIME], err, Path(str(stem) + "_errors.png")),
        plot_map(data, gps, Path(str(stem) + "_map.png")),
    ]

    if interactive:
        print("Интерактивные окна: масштаб, перемещение, домой — на панели инструментов.")
        plt.show()
    else:
        for fig in figs:
            plt.close(fig)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        input("Нажмите Enter...")
        raise SystemExit(1)
