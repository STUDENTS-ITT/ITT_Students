#!/usr/bin/env python3
"""
Полный конвейер слияния ВО + БИНС + Калман для HKairport03.

Этапы:
1. Запуск VO (vo_final) → trajectory.csv
2. Преобразование VO → INS формат (gps.dat, angle.dat)
3. Запуск БИНС (imitator.exe) → result.txt
4. Сравнение: VO vs VO+INS vs RTK
5. Построение графиков и PDF отчёта
"""

import subprocess
import sys
import os
from pathlib import Path
import shutil

ROOT = Path(__file__).parent.parent
VO_CPP = ROOT / "vo_cpp"
INS = ROOT / "add-kalman-report"
DATASET = ROOT  # HKairport03 в корне проекта

def run_cmd(cmd, cwd=None, desc=""):
    """Запуск команды с логированием"""
    print(f"\n{'='*60}")
    print(f"[{desc}]" if desc else f"[CMD]")
    print(f"  {' '.join(str(c) for c in cmd)}")
    print(f"  cwd: {cwd or os.getcwd()}")
    print('='*60)
    result = subprocess.run(cmd, cwd=cwd, capture_output=False, text=True)
    if result.returncode != 0:
        print(f"ОШИБКА: команда завершилась с кодом {result.returncode}")
        sys.exit(1)
    return result

def main():
    print("""
╔══════════════════════════════════════════════════════════╗
║  Конвейер слияния ВО + БИНС + Калман (HKairport03)      ║
╚══════════════════════════════════════════════════════════╝
""")
    
    # ---------------------------------------------------------------------
    # Шаг 1: Прогон ВО (если ещё не прогнан)
    # ---------------------------------------------------------------------
    vo_result = DATASET / "results_HKairport03" / "homography" / "visual_odometry" / "FAST-FREAK"
    vo_traj = vo_result / "trajectory.csv"
    
    if not vo_traj.exists():
        print("\n[1/5] Запуск видеоодометрии (vo_final)...")
        vo_exe = VO_CPP / "build" / "vo_final.exe"
        if not vo_exe.exists():
            print(f"ОШИБКА: {vo_exe} не найден. Соберите vo_cpp:")
            print(f"  cmake --build {VO_CPP / 'build'}")
            sys.exit(1)
        
        run_cmd([
            str(vo_exe),
            "--input", str(DATASET / "camera" / "images"),
            "--calibration", str(DATASET / "calibration" / "camera.json"),
            "--altitude-file", str(DATASET / "gps" / "altitude_per_frame.csv"),
            "--motion-model", "homography",
            "--algorithms", "FAST-FREAK",
            "--no-show-video",
        ], cwd=DATASET, desc="Видеоодометрия")
    else:
        print(f"\n[1/5] Траектория ВО уже существует: {vo_traj}")
    
    # ---------------------------------------------------------------------
    # Шаг 2: Преобразование VO → INS формат
    # ---------------------------------------------------------------------
    print("\n[2/5] Преобразование ВО → формат БИНС (prepare_vo_inputs.py)...")
    ins_data_dir = INS / "data" / "vo_hk03"
    run_cmd([
        sys.executable,
        str(INS / "tools" / "prepare_vo_inputs.py")
    ], cwd=ROOT, desc="Конвертация VO → INS")
    
    # Проверка, что файлы созданы
    required_files = ["imu.dat", "gps.dat", "angle.dat", "StartupNav.ini", "params.ini"]
    for f in required_files:
        fpath = ins_data_dir / f
        if not fpath.exists():
            print(f"ОШИБКА: файл {fpath} не создан!")
            sys.exit(1)
    print(f"  ✓ Созданы: {', '.join(required_files)}")
    
    # ---------------------------------------------------------------------
    # Шаг 3: Запуск БИНС + Калман
    # ---------------------------------------------------------------------
    print("\n[3/5] Запуск БИНС + фильтр Калмана (imitator.exe)...")
    imitator_exe = INS / "build" / "imitator.exe"
    if not imitator_exe.exists():
        print(f"ОШИБКА: {imitator_exe} не найден. Соберите add-kalman-report:")
        print(f"  cmake -B {INS / 'build'} -G Ninja && cmake --build {INS / 'build'}")
        sys.exit(1)
    
    run_cmd([
        str(imitator_exe),
        str(ins_data_dir),
        str(ins_data_dir / "params.ini")
    ], cwd=INS / "build", desc="БИНС + Калман")
    
    # Копировать результаты в общую папку
    tools_dir = INS.parent / "tools"
    for fname in ["result.txt", "reference.txt", "errors.txt"]:
        src = tools_dir / fname
        dst = DATASET / "results_fusion" / fname
        dst.parent.mkdir(exist_ok=True)
        if src.exists():
            shutil.copy2(src, dst)
            print(f"  ✓ Скопирован: {fname}")
    
    # ---------------------------------------------------------------------
    # Шаг 4: Построение графиков сравнения
    # ---------------------------------------------------------------------
    print("\n[4/5] Построение графиков сравнения...")
    run_cmd([
        sys.executable,
        str(ROOT / "tools" / "plot_fusion_results.py")
    ], cwd=ROOT, desc="Графики VO vs VO+INS")
    
    # ---------------------------------------------------------------------
    # Шаг 5: Генерация PDF отчёта
    # ---------------------------------------------------------------------
    print("\n[5/5] Генерация PDF отчёта...")
    run_cmd([
        sys.executable,
        str(ROOT / "report" / "make_fusion_report.py")
    ], cwd=ROOT, desc="PDF отчёт")
    
    print(f"""
╔══════════════════════════════════════════════════════════╗
║  ✓ Конвейер завершён успешно!                           ║
╚══════════════════════════════════════════════════════════╝

Результаты:
  • Траектория ВО:         {vo_result}
  • Результат ВО+ИНС:      {DATASET / 'results_fusion'}
  • Графики:               {DATASET / 'results_fusion/fusion_comparison.png'}
  • PDF отчёт:             {DATASET / 'report/VO_INS_Fusion_HKairport03.pdf'}
""")

if __name__ == "__main__":
    main()
