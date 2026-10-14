# validate_presets.py — независимая валидация пресета: прогон imitator с пресетом
# и расчёт метрик в точности как в tune_kalman.compute_error.
#
# Использование:
#   python validate_presets.py --build-dir <build/src/console> --presets-dir <presets> \
#       --data-root <dataset_shum> --out-dir <scratch>
#
# Для каждого набора (конфиг tuner_config.<name>.json) запускает
#   imitator.exe <data_dir> <preset.ini>
# в изолированном каталоге и печатает:
#   name  n_ref  p95_pos  p99.9_pos@200Гц  max_pos@200Гц
# p95_pos — по ближайшим строкам result к времени reference (протокол тюнера);
# p99.9 и max — по интерполяции reference на сетку 200 Гц (v3-tail).

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np


def load_text(path):
    """Числовая матрица с пропуском заголовка (как _load_skip_header)."""
    try:
        return np.loadtxt(path, skiprows=1)
    except ValueError:
        return None


def pos_hist_near(result, ref, skip_first_s):
    """p95 позиционной ошибки: для каждого времени reference — ближайшая
    строка result (протокол tune_kalman.compute_error, строки 216-229)."""
    t_ref = ref[:, 0]
    t_res = result[:, 0]
    idx = np.searchsorted(t_res, t_ref)
    idx = np.clip(idx, 0, len(t_res) - 1)
    idx_prev = np.clip(idx - 1, 0, len(t_res) - 1)
    closer = np.abs(t_res[idx] - t_ref) <= np.abs(t_res[idx_prev] - t_ref)
    idx = np.where(closer, idx, idx_prev)
    r = result[idx]
    d_north = (r[:, 2] - ref[:, 2]) * 111000.0
    d_east = (r[:, 1] - ref[:, 1]) * 111000.0 * np.cos(np.deg2rad(ref[:, 2]))
    pos = np.hypot(d_north, d_east)
    keep = t_ref >= skip_first_s
    return pos[keep]


def pos_hist_200(result, ref, skip_first_s):
    """Вектор ошибки на каждом такте 200 Гц: интерполяция reference по lat/lon
    (v3-tail из tune_kalman.compute_error, строки 272-281)."""
    t_ref = ref[:, 0]
    lat_ref_i = np.interp(result[:, 0], t_ref, ref[:, 2])
    lon_ref_i = np.interp(result[:, 0], t_ref, ref[:, 1])
    d_n = (result[:, 2] - lat_ref_i) * 111000.0
    d_e = (result[:, 1] - lon_ref_i) * 111000.0 * np.cos(np.deg2rad(lat_ref_i))
    pos = np.hypot(d_n, d_e)
    return pos[result[:, 0] >= skip_first_s]


def run_imitator(exe, data_dir, preset, workdir):
    """Консольный прогон с записью result/reference в <workdir>/tools."""
    tools = os.path.join(workdir, "tools")
    os.makedirs(tools, exist_ok=True)
    subprocess.run([exe, data_dir, preset], cwd=workdir, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return os.path.join(tools, "result.txt"), os.path.join(tools, "reference.txt")


def pct(x, p):
    return float(np.percentile(x, p)) if len(x) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--presets-dir", required=True)
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--tuner-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--preset-file", default=None,
                    help="альтернативный пресет для всех наборов (например дефолтный tuned.ini)")
    args = ap.parse_args()

    exe = os.path.join(args.build_dir, "imitator.exe")
    names = []
    for cfg_name in sorted(os.listdir(args.tuner_dir)):
        if not (cfg_name.startswith("tuner_config.") and cfg_name.endswith(".json")):
            continue
        names.append(cfg_name[len("tuner_config."):-len(".json")])
    names = [n for n in names if n and n != "smoke"]

    os.makedirs(args.out_dir, exist_ok=True)
    print("%-24s %8s %10s %14s %10s" % ("set", "n_ref", "p95_pos", "p99.9@200Hz", "max@200"))
    for name in names:
        cfg_path = os.path.join(args.tuner_dir, "tuner_config.%s.json" % name)
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        preset = os.path.join(args.presets_dir, os.path.basename(cfg["output"]["preset"]))
        if args.preset_file:
            preset = args.preset_file
        skip = float(cfg["horizon"]["skip_first_s"])
        data_dir = cfg["data_dir"]

        with tempfile.TemporaryDirectory(prefix="val_%s_" % name, dir=args.out_dir) as wd:
            result_path, ref_path = run_imitator(exe, data_dir, preset, wd)
            res = load_text(result_path)
            ref = load_text(ref_path)
            if res is None or ref is None or len(res) == 0 or len(ref) == 0:
                print("%-24s  ПРОГОН РАЗОШЁЛСЯ (файлы повреждены)" % name)
                continue
            p95h = pos_hist_near(res, ref, skip)
            p200 = pos_hist_200(res, ref, skip)
            print("%-24s %8d %10.3f %14.3f %10.3f" % (
                name, len(p95h), pct(p95h, 95), pct(p200, 99.9), float(np.max(p200)) if len(p200) else float("nan")))


if __name__ == "__main__":
    sys.exit(main())