#!/usr/bin/env python3
"""
Построение графиков сравнения ВО vs ВО+ИНС vs RTK для отчёта.

Создаёт:
1. fusion_comparison.png — два графика ошибок (абсолютная + накопленная)
2. trajectory_comparison.png — траектории на одном поле
3. metrics_summary.json — таблица метрик для PDF
"""

import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Без GUI
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

ROOT = Path(__file__).parent.parent

# Фазы и развороты — как в run_all_HKairport03.py (секунды от старта).
PHASE_MARKS = ((60, "конец подъёма"), (95, "начало маршрута"), (310, "начало снижения"))
TURNS = ((105, 120), (140, 155), (185, 200), (232, 247), (275, 292))

def load_csv(path):
    """Загрузить CSV в список словарей"""
    with open(path, encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))

def imu_time_origin():
    """Первая метка IMU — та же нулевая точка, что у prepare_vo_inputs и result.txt."""
    imu_path = ROOT / "imu" / "dji_osdk_ros_imu.csv"
    with open(imu_path, encoding="utf-8-sig") as f:
        row = next(csv.DictReader(f))
    return float(row["header.stamp.sec"]) + float(row["header.stamp.nanosec"]) * 1e-9

def load_trajectory_vo(csv_path):
    """Загрузить траекторию ВО: time, X, Y, Z"""
    rows = load_csv(csv_path)
    rows = [r for r in rows if r.get("Status", "").startswith(("ok", "origin"))]

    if rows and rows[0].get("Time", "").strip():
        t = np.array([float(r["Time"]) for r in rows])
    else:
        cam = load_csv(ROOT / "camera" / "timestamps.csv")
        cam_t = np.array([
            float(r["stamp_sec"]) + float(r["stamp_nsec"]) * 1e-9 for r in cam
        ])
        frames = np.array([int(r["Frame"]) - 1 for r in rows])
        t = cam_t[frames] - imu_time_origin()
    x = np.array([float(r["X"]) for r in rows])
    y = np.array([float(r["Y"]) for r in rows])
    z = np.array([float(r.get("Altitude", r.get("Z", 0))) for r in rows])
    
    return {"time": t, "X": x, "Y": y, "Z": z, "name": "ВО (только камера)"}

def load_trajectory_ins(txt_path):
    """Загрузить result.txt от БИНС: lon, lat, alt → локальная ENU"""
    rows = []
    with open(txt_path, encoding='utf-8') as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) < 10:
                continue
            try:
                rows.append({
                    "time": float(parts[0]),
                    "lon": float(parts[1]),
                    "lat": float(parts[2]),
                    "alt": float(parts[3]),
                })
            except ValueError:
                continue
    
    if not rows:
        raise ValueError(f"Нет данных в {txt_path}")
    
    # Преобразовать lat/lon в локальную ENU относительно первой точки
    lat0, lon0, alt0 = rows[0]["lat"], rows[0]["lon"], rows[0]["alt"]
    lat0_rad = math.radians(lat0)
    
    a = 6378137.0  # WGS-84
    e2 = 6.69437999014e-3
    s = math.sin(lat0_rad)
    rn = a / math.sqrt(1 - e2 * s * s)
    rm = rn * (1 - e2) / (1 - e2 * s * s)
    
    t, x, y, z = [], [], [], []
    for r in rows:
        t.append(r["time"])
        E = math.radians(r["lon"] - lon0) * rn * math.cos(lat0_rad)
        N = math.radians(r["lat"] - lat0) * rm
        x.append(E)
        y.append(N)
        z.append(r["alt"] - alt0)
    
    return {
        "time": np.array(t),
        "X": np.array(x),
        "Y": np.array(y),
        "Z": np.array(z),
        "name": "VO+INS+Kalman"
    }

def load_rtk_reference():
    """Загрузить RTK эталон: gps/dji_osdk_ros_rtk_position.csv"""
    rtk_path = ROOT / "gps" / "dji_osdk_ros_rtk_position.csv"
    if not rtk_path.exists():
        return None
    
    rows = load_csv(rtk_path)
    t = np.array([float(r["header.stamp.sec"]) + float(r["header.stamp.nanosec"]) * 1e-9
                  for r in rows])
    lat = np.array([float(r["latitude"]) for r in rows])
    lon = np.array([float(r["longitude"]) for r in rows])
    alt = np.array([float(r["altitude"]) for r in rows])
    
    # Та же нулевая точка, что у result.txt (первый отсчёт IMU)
    t -= imu_time_origin()
    
    # Преобразовать в локальную ENU
    lat0, lon0, alt0 = lat[0], lon[0], alt[0]
    lat0_rad = math.radians(lat0)
    
    a = 6378137.0
    e2 = 6.69437999014e-3
    s = math.sin(lat0_rad)
    rn = a / math.sqrt(1 - e2 * s * s)
    rm = rn * (1 - e2) / (1 - e2 * s * s)
    
    E = np.radians(lon - lon0) * rn * math.cos(lat0_rad)
    N = np.radians(lat - lat0) * rm
    Z = alt - alt0
    
    return {"time": t, "X": E, "Y": N, "Z": Z, "name": "RTK (эталон)"}

def step_errors(traj, rtk):
    """Ошибка каждого шага относительно RTK на той же паре моментов."""
    tx = np.interp(traj["time"], rtk["time"], rtk["X"])
    ty = np.interp(traj["time"], rtk["time"], rtk["Y"])
    step = np.hypot(np.diff(traj["X"]) - np.diff(tx), np.diff(traj["Y"]) - np.diff(ty))
    return np.r_[0.0, step]

def align_trajectory(traj, rtk):
    """
    Выравнивание траектории к RTK по первым 10% или до t=105s (как в make_report.py).
    Возвращает выровненную траекторию (X, Y).
    """
    # Интерполировать RTK на моменты времени траектории
    rx = np.interp(traj["time"], rtk["time"], rtk["X"])
    ry = np.interp(traj["time"], rtk["time"], rtk["Y"])
    
    # Найти индекс для выравнивания (первые 10% пути RTK или t <= 105s)
    path_rtk = np.cumsum(np.hypot(np.diff(rx), np.diff(ry)))
    path_rtk = np.r_[0.0, path_rtk]
    cap = 0.10 * path_rtk[-1]
    i1 = np.searchsorted(path_rtk, cap)
    
    # Если есть точки с t <= 105s, ограничить по времени
    cut = np.searchsorted(traj["time"], 105.0)
    if cut >= 20 and cut < i1:
        i1 = cut
    
    # Центроиды
    cx_vo = np.mean(traj["X"][:i1])
    cy_vo = np.mean(traj["Y"][:i1])
    cx_rtk = np.mean(rx[:i1])
    cy_rtk = np.mean(ry[:i1])
    
    # SVD для поворота (Kabsch alignment)
    A = np.c_[traj["X"][:i1] - cx_vo, traj["Y"][:i1] - cy_vo]
    B = np.c_[rx[:i1] - cx_rtk, ry[:i1] - cy_rtk]
    
    # Найти оптимальный поворот через SVD
    dot = np.dot(A.T, B)
    cross = np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0])
    angle = np.arctan2(cross, np.trace(dot))
    
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    
    # Применить трансформацию ко всей траектории
    centered = np.c_[traj["X"] - cx_vo, traj["Y"] - cy_vo]
    aligned = (R @ centered.T).T + np.array([cx_rtk, cy_rtk])
    
    return aligned[:, 0], aligned[:, 1], i1


def compute_metrics(traj, rtk):
    """RMS, макс, финиш ошибки"""
    # Интерполировать RTK на моменты времени траектории
    rx = np.interp(traj["time"], rtk["time"], rtk["X"])
    ry = np.interp(traj["time"], rtk["time"], rtk["Y"])
    
    err = np.hypot(traj["X"] - rx, traj["Y"] - ry)
    
    return {
        "rms": float(np.sqrt(np.mean(err**2))),
        "max": float(np.max(err)),
        "end": float(err[-1]),
        "mean": float(np.mean(err)),
    }

def mark_flight_phases(ax):
    """Развороты синим, границы фаз — серым пунктиром. Как HK_01_models."""
    for a, b in TURNS:
        ax.axvspan(a, b, color="tab:blue", alpha=0.10, zorder=0)
    ymin, ymax = ax.get_ylim()
    for moment, label in PHASE_MARKS:
        ax.axvline(moment, color="gray", ls="--", lw=1, zorder=1)
        ax.text(
            moment + 3,
            ymax * 0.95,
            label,
            rotation=90,
            fontsize=8,
            color="gray",
            va="top",
        )
    ax.set_ylim(ymin, ymax)


def plot_comparison(vo_traj, ins_traj, rtk_traj, out_dir):
    """Построить графики сравнения"""
    out_dir.mkdir(exist_ok=True)
    
    # Выровнять ВО траекторию к RTK (как в make_report.py)
    vo_aligned_x, vo_aligned_y, align_idx = align_trajectory(vo_traj, rtk_traj)
    print(f"  ВО выровнена к RTK по первым {align_idx} точкам (до t={vo_traj['time'][align_idx]:.1f}s)")
    
    # Обновить координаты ВО
    vo_traj_aligned = vo_traj.copy()
    vo_traj_aligned["X"] = vo_aligned_x
    vo_traj_aligned["Y"] = vo_aligned_y
    
    # ---------------------------------------------------------------------
    # График 1: Абсолютное отклонение от RTK vs время
    # ---------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9))
    
    # Интерполировать RTK на моменты ВО и ИНС
    vo_rx = np.interp(vo_traj_aligned["time"], rtk_traj["time"], rtk_traj["X"])
    vo_ry = np.interp(vo_traj_aligned["time"], rtk_traj["time"], rtk_traj["Y"])
    vo_err = np.hypot(vo_traj_aligned["X"] - vo_rx, vo_traj_aligned["Y"] - vo_ry)
    
    ins_rx = np.interp(ins_traj["time"], rtk_traj["time"], rtk_traj["X"])
    ins_ry = np.interp(ins_traj["time"], rtk_traj["time"], rtk_traj["Y"])
    ins_err = np.hypot(ins_traj["X"] - ins_rx, ins_traj["Y"] - ins_ry)
    
    ax1.plot(ins_traj["time"], ins_err, label="VO+INS+Kalman", color='#27ae60', lw=1.5)
    mark_flight_phases(ax1)
    ax1.set_ylabel('ошибка, м', fontsize=11)
    ax1.set_xlabel('время, с', fontsize=11)
    ax1.set_title('Ошибка положения во времени (синим - развороты)', fontsize=13, fontweight='bold')
    ax1.legend(fontsize=10, loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # ---------------------------------------------------------------------
    # График 2: Накопленная ошибка шагов
    # ---------------------------------------------------------------------
    vo_steps = step_errors(vo_traj_aligned, rtk_traj)
    ins_steps = step_errors(ins_traj, rtk_traj)
    vo_cum = np.cumsum(vo_steps)
    ins_cum = np.cumsum(ins_steps)
    
    ax2.plot(ins_traj["time"], ins_cum, label="VO+INS+Kalman", color='#27ae60', lw=1.5)
    ax2.set_ylabel('Накопленная ошибка шагов, м', fontsize=11)
    ax2.set_xlabel('Время полёта, с', fontsize=11)
    ax2.set_title('Накопленная ошибка шагов (монотонный рост)', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(out_dir / "fusion_comparison.png", dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Сохранён: {out_dir / 'fusion_comparison.png'}")
    
    # ---------------------------------------------------------------------
    # График 2: Траектории на одном поле
    # ---------------------------------------------------------------------
    # СК: X=север, Z=восток, Y=вверх.
    # На поле: снизу Z (восток), слева X (север).
    # В данных ENU: "X" = восток, "Y" = север.
    fig, ax = plt.subplots(figsize=(10, 10))
    
    ax.plot(rtk_traj["X"], rtk_traj["Y"], label="RTK",
            color='#34495e', lw=2, ls='--', alpha=0.7)
    ax.plot(ins_traj["X"], ins_traj["Y"], label="VO+INS+Kalman",
            color='#27ae60', lw=1.5)
    
    # Отметить старт красным кружком
    ax.add_patch(Circle((rtk_traj["X"][0], rtk_traj["Y"][0]), 8, color='red', zorder=10))
    ax.text(rtk_traj["X"][0] + 15, rtk_traj["Y"][0] + 15, 'СТАРТ', 
            fontsize=11, fontweight='bold', color='red')
    
    ax.set_xlabel('восток, м', fontsize=11)
    ax.set_ylabel('север, м', fontsize=11)
    ax.set_title('Сравнение траекторий (X — север, Z — восток, Y — вверх)', fontsize=13, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig(out_dir / "trajectory_comparison.png", dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Сохранён: {out_dir / 'trajectory_comparison.png'}")
    
    # ---------------------------------------------------------------------
    # Метрики в JSON
    # ---------------------------------------------------------------------
    vo_metrics = compute_metrics(vo_traj_aligned, rtk_traj)
    ins_metrics = compute_metrics(ins_traj, rtk_traj)
    
    improvement = {
        "rms": (vo_metrics["rms"] - ins_metrics["rms"]) / vo_metrics["rms"] * 100,
        "max": (vo_metrics["max"] - ins_metrics["max"]) / vo_metrics["max"] * 100,
        "end": (vo_metrics["end"] - ins_metrics["end"]) / vo_metrics["end"] * 100,
    }
    
    summary = {
        "vo_only": vo_metrics,
        "vo_ins_kalman": ins_metrics,
        "improvement_pct": improvement,
    }
    
    with open(out_dir / "metrics_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"  ✓ Сохранён: {out_dir / 'metrics_summary.json'}")
    
    # Вывод в консоль
    print(f"""
╔══════════════════════════════════════════════════════╗
║  Метрики качества                                    ║
╚══════════════════════════════════════════════════════╝

                     ВО (только)  ВО+ИНС+Калман  Улучшение
────────────────────────────────────────────────────────
RMS ошибка, м        {vo_metrics['rms']:8.2f}      {ins_metrics['rms']:8.2f}     {improvement['rms']:+6.1f}%
Макс. ошибка, м      {vo_metrics['max']:8.2f}      {ins_metrics['max']:8.2f}     {improvement['max']:+6.1f}%
Ошибка на финише, м  {vo_metrics['end']:8.2f}      {ins_metrics['end']:8.2f}     {improvement['end']:+6.1f}%
────────────────────────────────────────────────────────
""")

def main():
    parser = argparse.ArgumentParser(description="Построение графиков сравнения ВО vs ВО+ИНС vs RTK")
    parser.add_argument(
        "--vo-csv",
        type=Path,
        default=ROOT / "results_HKairport03" / "homography" / "visual_odometry" / "FAST-FREAK" / "trajectory.csv",
        help="Путь к trajectory.csv видеоодометрии"
    )
    parser.add_argument(
        "--ins-txt",
        type=Path,
        default=ROOT / "results_fusion" / "result.txt",
        help="Путь к result.txt БИНС (imitator.exe)"
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=ROOT / "results_fusion",
        help="Директория для сохранения графиков и metrics_summary.json"
    )
    args = parser.parse_args()
    
    print("\n[Построение графиков сравнения ВО vs ВО+ИНС]")
    
    vo_csv = args.vo_csv
    ins_txt = args.ins_txt
    
    if not vo_csv.exists():
        print(f"ОШИБКА: Траектория ВО не найдена: {vo_csv}")
        return
    
    if not ins_txt.exists():
        print(f"ОШИБКА: Результат БИНС не найден: {ins_txt}")
        print("Запустите сначала БИНС: ./add-kalman-report/build/imitator.exe")
        return
    
    # Загрузка данных
    print("  Загрузка траекторий...")
    vo_traj = load_trajectory_vo(vo_csv)
    ins_traj = load_trajectory_ins(ins_txt)
    rtk_traj = load_rtk_reference()
    
    if rtk_traj is None:
        print("ПРЕДУПРЕЖДЕНИЕ: RTK эталон не найден, сравнение невозможно!")
        return
    
    print(f"  ✓ Загружено: ВО ({len(vo_traj['time'])} точек), "
          f"ИНС ({len(ins_traj['time'])} точек), RTK ({len(rtk_traj['time'])} точек)")
    
    # Построение графиков
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_comparison(vo_traj, ins_traj, rtk_traj, out_dir)
    
    print(f"\n✓ Графики сохранены в: {out_dir}")

if __name__ == "__main__":
    main()
