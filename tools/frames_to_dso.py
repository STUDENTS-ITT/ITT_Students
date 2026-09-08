#!/usr/bin/env python3
"""Подготовка папки с кадрами к запуску DSO.

Создаёт структуру, которую ждёт dso_dataset:

    <out>/
      images/       кадры в градациях серого, 8 бит
      times.txt     "id  метка_времени  экспозиция"  <- в РОДИТЕЛЬСКОМ каталоге images
      camera.txt    калибровка

Запуск после подготовки:

    ~/dso/build/bin/dso_dataset files=<out>/images calib=<out>/camera.txt \\
        preset=0 mode=1 nogui=1 playbackSpeed=0

Пример для набора «Поворот_коптер» (1920x1080, калибровки нет):

    python3 frames_to_dso.py \\
        --frames "/media/$USER/DISK/Датасет/Datasets/Поворот_коптер/frames_1" \\
        --out data/povorot_kopter --fps 10 --hfov 73 --scale 0.5

Про калибровку. Реальных внутренних параметров камеры нет, поэтому fx
оценивается из горизонтального угла обзора: fx = (W/2) / tan(HFOV/2),
дисторсия принимается нулевой. Это заметное упрощение, и в отчёте оно
должно быть оговорено как источник погрешности. Если найдёте паспорт
камеры или сможете отснять шахматную доску — калибруйте нормально.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, required=True, help="папка с кадрами")
    ap.add_argument("--out", type=Path, required=True, help="куда сложить результат")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="частота кадров для синтеза times.txt")
    ap.add_argument("--stamps", type=Path,
                    help="CSV с настоящими метками времени (первая колонка, нс) — "
                         "если есть, используется вместо --fps")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="коэффициент уменьшения кадра, например 0.5")

    grp = ap.add_argument_group("калибровка")
    grp.add_argument("--hfov", type=float, default=None,
                     help="горизонтальный угол обзора в градусах (приближённая калибровка)")
    grp.add_argument("--fx", type=float, default=None)
    grp.add_argument("--fy", type=float, default=None)
    grp.add_argument("--cx", type=float, default=None)
    grp.add_argument("--cy", type=float, default=None)
    grp.add_argument("--dist", nargs=4, type=float, default=None,
                     metavar=("K1", "K2", "P1", "P2"),
                     help="коэффициенты дисторсии RadTan")
    ap.add_argument("--limit", type=int, default=None, help="взять только N первых кадров")
    args = ap.parse_args()

    files = sorted(p for p in args.frames.iterdir()
                   if p.is_file() and p.suffix.lower() in EXTS)
    if not files:
        raise SystemExit(f"В {args.frames} нет изображений")
    if args.limit:
        files = files[: args.limit]

    probe = cv2.imread(str(files[0]), cv2.IMREAD_UNCHANGED)
    if probe is None:
        raise SystemExit(f"Не читается {files[0]}")
    h_in, w_in = probe.shape[:2]

    # DSO любит размеры, кратные 8.
    w_out = int(round(w_in * args.scale)) // 8 * 8
    h_out = int(round(h_in * args.scale)) // 8 * 8
    sx, sy = w_out / w_in, h_out / h_in

    print(f"Кадров: {len(files)}")
    print(f"Вход:   {w_in}x{h_in}")
    print(f"Выход:  {w_out}x{h_out} (масштаб {sx:.4f} x {sy:.4f})")

    img_dir = args.out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    for i, src in enumerate(files):
        im = cv2.imread(str(src), cv2.IMREAD_GRAYSCALE)
        if im is None:
            print(f"  пропускаю нечитаемый {src.name}", file=sys.stderr)
            continue
        if (w_out, h_out) != (w_in, h_in):
            im = cv2.resize(im, (w_out, h_out), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(img_dir / f"{i:06d}.png"), im)
        if (i + 1) % 100 == 0:
            print(f"  {i + 1}/{len(files)}")

    # ---- times.txt ------------------------------------------------------
    if args.stamps and args.stamps.is_file():
        stamps = []
        with args.stamps.open(encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line or line[0].isalpha() or line.startswith("#"):
                    continue
                try:
                    stamps.append(float(line.split(",")[0]) / 1e9)
                except ValueError:
                    continue
        stamps = stamps[: len(files)]
        print(f"Метки времени: из {args.stamps.name}, {len(stamps)} шт.")
    else:
        stamps = [i / args.fps for i in range(len(files))]
        print(f"Метки времени: синтезированы из {args.fps:g} Гц. "
              "На геометрию траектории это не влияет (DSO не использует время "
              "в оптимизации), но любые скоростные величины будут условными.")

    # DSO читает times.txt из РОДИТЕЛЬСКОГО каталога папки с кадрами.
    times_path = args.out / "times.txt"
    with times_path.open("w", encoding="utf-8") as f:
        for i, t in enumerate(stamps):
            f.write(f"{i} {t:.9f} 0\n")

    # ---- camera.txt -----------------------------------------------------
    if args.fx is not None:
        fx, fy = args.fx * sx, (args.fy if args.fy else args.fx) * sy
        cx = (args.cx if args.cx is not None else w_in / 2) * sx
        cy = (args.cy if args.cy is not None else h_in / 2) * sy
        note = "по заданным параметрам"
    elif args.hfov is not None:
        fx_full = (w_in / 2.0) / math.tan(math.radians(args.hfov) / 2.0)
        fx = fx_full * sx
        fy = fx_full * sy
        cx, cy = w_out / 2.0, h_out / 2.0
        note = f"ПРИБЛИЖЁННАЯ, из HFOV={args.hfov:g}°"
    else:
        raise SystemExit("Задайте либо --hfov, либо --fx/--fy/--cx/--cy")

    dist = args.dist if args.dist else [0.0, 0.0, 0.0, 0.0]

    cam_path = args.out / "camera.txt"
    with cam_path.open("w", encoding="utf-8") as f:
        f.write(f"RadTan {fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f} "
                f"{dist[0]:.8f} {dist[1]:.8f} {dist[2]:.8f} {dist[3]:.8f}\n")
        f.write(f"{w_out} {h_out}\n")
        f.write("crop\n")
        f.write(f"{w_out} {h_out}\n")

    print(f"\nКалибровка ({note}):")
    print(f"  fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f} dist={dist}")
    print(f"\nГотово: {args.out}")
    print("\nЗапуск:")
    print(f"  export DSO_TIMING_LOG=~/Downloads/cursor/Projects/fire-dron/results/timing/dso_{args.out.name}.csv")
    print(f"  ~/dso/build/bin/dso_dataset files={img_dir} calib={cam_path} \\")
    print("      preset=0 mode=1 nogui=1 quiet=1 playbackSpeed=0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
