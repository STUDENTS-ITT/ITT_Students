"""Сравнение решения Калмана с эталоном.

Читает kalman15_line2.txt и строит 6 графиков:
  - Курс, тангаж, крен: Калман vs эталон
  - Долгота, широта, высота: Калман vs эталон

Использование:
    python tools/plot_trajectory.py [путь_к_файлу]
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


def plot(data: np.ndarray, save_path: str | None = None):
    t = data[:, 0]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    fig.suptitle("Калман vs эталон", fontsize=14)

    panels = [
        (axes[0, 0], 3,  13, "Курс, град"),
        (axes[0, 1], 4,  15, "Тангаж, град"),
        (axes[0, 2], 5,  14, "Крен, град"),
        (axes[1, 0], 1,  10, "Долгота, град"),
        (axes[1, 1], 2,  11, "Широта, град"),
        (axes[1, 2], 9,  12, "Высота, м"),
    ]

    for ax, col_bins, col_sns, ylabel in panels:
        ax.plot(t, data[:, col_bins], label="Калман", linewidth=0.8)
        ax.plot(t, data[:, col_sns], label="Эталон", linewidth=0.8, alpha=0.7)
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
    path = sys.argv[1] if len(sys.argv) > 1 else "kalman15_line2.txt"
    data = load(path)
    save_path = path.rsplit(".", 1)[0] + "_comparison.png"
    plot(data, save_path)
