"""Сравнение решения Калмана с эталоном.

Читает result.txt и reference.txt и строит 6 графиков:
  - Курс, тангаж, крен: Калман vs эталон
  - Долгота, широта, высота: Калман vs эталон

Использование:
    python tools/plot_trajectory.py [путь_к_result.txt] [путь_к_reference.txt]
"""

import sys
import numpy as np
import matplotlib.pyplot as plt


def load(path: str) -> np.ndarray:
    with open(path) as f:
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


def plot(result: np.ndarray, reference: np.ndarray, save_path: str | None = None):
    t_r = result[:, 0]
    t_s = reference[:, 0]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    fig.suptitle("Калман vs эталон", fontsize=14)

    # result: time, lon, lat, alt, heading, pitch, roll, vn, vh, ve
    # reference: time, lon, lat, alt, heading, pitch, roll, vn, vh, ve
    panels = [
        (axes[0, 0], 4,  4, "Курс, град"),
        (axes[0, 1], 5,  5, "Тангаж, град"),
        (axes[0, 2], 6,  6, "Крен, град"),
        (axes[1, 0], 1,  1, "Долгота, град"),
        (axes[1, 1], 2,  2, "Широта, град"),
        (axes[1, 2], 3,  3, "Высота, м"),
    ]

    for ax, col_r, col_s, ylabel in panels:
        ax.plot(t_r, result[:, col_r], label="Калман", linewidth=0.8)
        ax.plot(t_s, reference[:, col_s], label="Эталон", linewidth=0.8, alpha=0.7)
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for ax in axes[1]:
        ax.set_xlabel("Время, с")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Сохранено: {save_path}")
    plt.show()


if __name__ == "__main__":
    result_path = sys.argv[1] if len(sys.argv) > 1 else "result.txt"
    reference_path = sys.argv[2] if len(sys.argv) > 2 else "reference.txt"
    result = load(result_path)
    reference = load(reference_path)
    save_path = result_path.rsplit(".", 1)[0] + "_comparison.png"
    plot(result, reference, save_path)
