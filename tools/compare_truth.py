"""Сравнение reference.txt (измерения СНС) с truth.txt (истинная траектория).

Оба файла имеют 10 колонок: time lon lat alt heading pitch roll vn vh ve.
Так как сетки времени не совпадают, truth интерполируется линейно на моменты
reference. Метрики: RMSE, среднее абсолютное, максимум — по lat/lon/alt в метрах
и по heading/pitch/roll/vn/vh/ve в их единицах; плюс горизонтальная ошибка.

Использование:
    python compare_truth.py <dir> [dir2 ...]
        dir — каталог с truth.txt и tools/reference.txt (как у dataset_shum),
        либо каталог, содержащий оба файла напрямую.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

COL_TIME, COL_LON, COL_LAT, COL_ALT, COL_HDG, COL_PITCH, COL_ROLL, COL_VN, COL_VH, COL_VE = range(10)
HEADER = ["time", "lon", "lat", "alt", "heading", "pitch", "roll", "vn", "vh", "ve"]

# WGS84 приближение: 1 градус широты ≈ 111.32 км.
METERS_PER_DEG_LAT = 111320.0

NAMES_RAD = ["heading", "pitch", "roll"]  # в truth/reference хранятся уже в градусах


def norm_angle(e: np.ndarray) -> np.ndarray:
    return np.mod(e + 180.0, 360.0) - 180.0


def load_table(path: Path) -> np.ndarray:
    rows = []
    # latin-1: наборы могут содержать не-ASCII байты в хвосте; числа всё равно
    # читаются из ASCII-префикса каждой строки.
    with open(path, "r", encoding="latin-1") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 10 or parts[0] == "time":
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                # Строка содержит бинарный мусор (в наборах dataset_shum есть
                # повреждённые участки) — пропускаем её, как делает и читатель.
                continue
    if not rows:
        raise ValueError(f"no data in {path}")
    return np.array(rows, dtype=float)


def interp_truth(truth: np.ndarray, ref_time: np.ndarray) -> np.ndarray:
    rows = []
    for col in range(1, truth.shape[1]):
        rows.append(np.interp(ref_time, truth[:, COL_TIME], truth[:, col]))
    return np.vstack(rows).T  # (n, 9) без времени


def report(set_name: str, truth: np.ndarray, ref: np.ndarray) -> None:
    # Общее перекрытие по времени: ограничиваем обоих по диапазону reference.
    t0, t1 = ref[0, COL_TIME], ref[-1, COL_TIME]
    mask = (truth[:, COL_TIME] >= t0) & (truth[:, COL_TIME] <= t1)
    truth_i = truth[mask]
    if truth_i.shape[0] < 2:
        print(f"{set_name}: truth window empty in [{t0:.3f}, {t1:.3f}], skip")
        return

    ref_v = interp_truth(truth_i, ref[:, COL_TIME])  # эталон на моменты reference
    n = ref.shape[0]

    # Ошибки. Знаковые ошибки: ref - truth (измерение минус эталон).
    lat_err = ref[:, COL_LAT] - ref_v[:, COL_LAT - 1]
    lon_err = ref[:, COL_LON] - ref_v[:, COL_LON - 1]
    alt_err = ref[:, COL_ALT] - ref_v[:, COL_ALT - 1]

    # В метры.
    lat_deg = ref[:, COL_LAT]
    lat_m = lat_err * METERS_PER_DEG_LAT
    lon_m = lon_err * METERS_PER_DEG_LAT * np.cos(np.radians(lat_deg))
    alt_m = alt_err
    horizontal = np.sqrt(lon_m**2 + lat_m**2)

    cols = {
        "lat[sm]": lat_m,
        "lon[sm]": lon_m,
        "alt[m]": alt_m,
        "h[sm]": horizontal,
        "heading[deg]": norm_angle(ref[:, COL_HDG] - ref_v[:, COL_HDG - 1]),
        "pitch[deg]": norm_angle(ref[:, COL_PITCH] - ref_v[:, COL_PITCH - 1]),
        "roll[deg]": norm_angle(ref[:, COL_ROLL] - ref_v[:, COL_ROLL - 1]),
        "vn[m/s]": ref[:, COL_VN] - ref_v[:, COL_VN - 1],
        "vh[m/s]": ref[:, COL_VH] - ref_v[:, COL_VH - 1],
        "ve[m/s]": ref[:, COL_VE] - ref_v[:, COL_VE - 1],
    }

    print(f"{set_name}: n={n} t={t0:.3f}..{t1:.3f}")
    print(f"  {'column':<14}{'mean':>10}{'abs':>10}{'rmse':>10}{'max':>10}")
    for name, e in cols.items():
        rmse = float(np.sqrt(np.mean(e**2)))
        mx = float(np.max(np.abs(e)))
        mean = float(np.mean(e))
        abv = float(np.mean(np.abs(e)))
        print(f"  {name:<14}{mean:10.4f}{abv:10.4f}{rmse:10.4f}{mx:10.4f}")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    for arg in sys.argv[1:]:
        d = Path(arg)
        truth = d / "truth.txt"
        ref = d / "reference.txt"
        if truth.is_file() and ref.is_file():
            set_name = d.parent.name
        else:
            truth = d / "truth.txt"
            ref = d / "tools" / "reference.txt"
            set_name = d.name

        if not truth.is_file() or not ref.is_file():
            print(f"{d}: truth.txt/reference.txt not found, skip")
            continue
        try:
            report(set_name, load_table(truth), load_table(ref))
        except (ValueError, OSError) as e:
            print(f"{d}: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())