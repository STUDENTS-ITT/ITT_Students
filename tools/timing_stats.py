#!/usr/bin/env python3
"""Сводная статистика по логам времени обработки кадра.

На вход подаются CSV, созданные врезкой patch_timing.py: две колонки
"метка_времени,миллисекунды", без заголовка.

    python3 timing_stats.py ~/Downloads/cursor/Projects/fire-dron/results/timing/*.csv --out summary.md

Публикуются медиана и 95-й перцентиль, а не только среднее: у обоих
алгоритмов распределение бимодальное (обычный кадр против кейфрейма с
полной оптимизацией окна), и среднее по такому распределению
мало о чём говорит.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def load(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    return np.array(rows, dtype=float) if rows else np.empty((0, 2))


def describe(name: str, data: np.ndarray) -> dict:
    ms = data[:, 1]
    stamps = data[:, 0]
    span = float(stamps.max() - stamps.min()) if len(stamps) > 1 else 0.0
    return {
        "name": name,
        "n": len(ms),
        "mean": float(np.mean(ms)),
        "median": float(np.median(ms)),
        "p95": float(np.percentile(ms, 95)),
        "max": float(np.max(ms)),
        "std": float(np.std(ms)),
        # Сколько кадров в секунду алгоритм способен переварить при таком
        # медианном времени. Сравнивается с частотой камеры.
        "fps_capable": 1000.0 / float(np.median(ms)) if np.median(ms) > 0 else 0.0,
        "span_s": span,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, help="сохранить таблицу в Markdown")
    ap.add_argument("--pair", action="store_true",
                    help="сложить парные логи vins_front_* и vins_back_*")
    args = ap.parse_args()

    stats = []
    loaded: dict[str, np.ndarray] = {}

    for path in args.files:
        if not path.is_file():
            print(f"пропускаю {path}: файла нет", file=sys.stderr)
            continue
        data = load(path)
        if len(data) == 0:
            print(f"пропускаю {path}: нет замеров", file=sys.stderr)
            continue
        loaded[path.stem] = data
        stats.append(describe(path.stem, data))

    if args.pair:
        for stem, data in list(loaded.items()):
            if not stem.startswith("vins_front_"):
                continue
            back_stem = stem.replace("vins_front_", "vins_back_", 1)
            back = loaded.get(back_stem)
            if back is None:
                continue
            # Бэкенд работает не на каждом кадре, поэтому суммируем по
            # общей длительности: средняя суммарная нагрузка на кадр.
            n = len(data)
            total = np.concatenate([data[:, 1], back[:, 1]])
            merged = describe(stem.replace("vins_front_", "vins_TOTAL_", 1),
                              np.column_stack([np.arange(len(total)), total]))
            merged["n"] = n
            merged["mean"] = (float(np.sum(data[:, 1])) + float(np.sum(back[:, 1]))) / n
            stats.append(merged)

    if not stats:
        print("Нет данных.", file=sys.stderr)
        return 1

    header = ("| Лог | Кадров | Среднее, мс | Медиана, мс | 95 %, мс "
              "| Максимум, мс | СКО, мс | Потолок, к/с |")
    sep = "|---|---:|---:|---:|---:|---:|---:|---:|"
    lines = [header, sep]
    for s in sorted(stats, key=lambda x: x["name"]):
        lines.append(
            f"| `{s['name']}` | {s['n']} | {s['mean']:.2f} | {s['median']:.2f} "
            f"| {s['p95']:.2f} | {s['max']:.2f} | {s['std']:.2f} "
            f"| {s['fps_capable']:.1f} |"
        )

    table = "\n".join(lines)
    print(table)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(
            "# Время обработки кадра\n\n" + table + "\n\n"
            "«Потолок, к/с» — сколько кадров в секунду алгоритм способен\n"
            "обработать при медианном времени. Сравнивается с частотой камеры:\n"
            "EuRoC 20 Гц, MARS-LVIG 10 Гц.\n",
            encoding="utf-8",
        )
        print(f"\nСохранено в {args.out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
