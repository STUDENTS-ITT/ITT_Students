# compare_presets_stats.py — сравнение двух наборов пресетов по ошибке
# положения относительно наземной истины (truth.txt): максимум, число точек
# выше порогов, P95/P99.9 (этап 13A, доп. таблица отчёта).
#
# Протокол совпадает с tune_kalman.compute_error: для каждого такта truth
# берётся ближайшая строка result; горизонт отбрасывается (skip_first_s);
# метрики считаются на валид-окне (train_frac..1).
#
# Использование:
#   python compare_presets_stats.py --build-dir <...> --presets-a <dirA> \
#       --presets-b <dirB> --tuner-dir <...> --out-dir <scratch>

import argparse
import json
import os
import subprocess
import tempfile

import numpy as np

TUNER_DIR = os.path.dirname(os.path.abspath(__file__))
THRESHOLDS = (0.5, 1.0, 2.0, 5.0)
SKIP_NAMES = ("smoke", "square_noisefree")


def run_imitator(exe, data_dir, preset, workdir):
    tools = os.path.join(workdir, "tools")
    os.makedirs(tools, exist_ok=True)
    subprocess.run([exe, data_dir, preset], cwd=workdir,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   check=True)
    return os.path.join(tools, "result.txt")


def load_text(path):
    try:
        return np.loadtxt(path, skiprows=1)
    except (ValueError, OSError):
        return None


def pos_near_valid(res, truth, skip_first_s, train_frac):
    """Планарная ошибка («ближайшая строка») на валид-окне."""
    t_ref, t_res = truth[:, 0], res[:, 0]
    idx = np.searchsorted(t_res, t_ref)
    idx = np.clip(idx, 0, len(t_res) - 1)
    idx_prev = np.clip(idx - 1, 0, len(t_res) - 1)
    closer = np.abs(t_res[idx] - t_ref) <= np.abs(t_res[idx_prev] - t_ref)
    idx = np.where(closer, idx, idx_prev)
    r = res[idx]
    lat_ref, lon_ref = truth[:, 2], truth[:, 1]
    d_north = (r[:, 2] - lat_ref) * 111000.0
    d_east = (r[:, 1] - lon_ref) * 111000.0 * np.cos(np.deg2rad(lat_ref))
    pos = np.hypot(d_north, d_east)
    keep = t_ref >= skip_first_s
    tt, pe = t_ref[keep], pos[keep]
    if len(tt) < 20:
        return None
    split = tt[0] + (tt[-1] - tt[0]) * train_frac
    return pe[tt > split]


def stat_one(exe, data_dir, preset, skip_first_s, train_frac):
    wd = tempfile.mkdtemp(prefix="cmp_", dir=None)
    res_p = run_imitator(exe, data_dir, preset, wd)
    res = load_text(res_p)
    truth = load_text(os.path.join(data_dir, "truth.txt"))
    if res is None or truth is None:
        return None
    pos = pos_near_valid(res, truth, skip_first_s, train_frac)
    if pos is None or len(pos) == 0:
        return None
    return {
        "n": int(len(pos)),
        "p95": float(np.percentile(pos, 95)),
        "p999": float(np.percentile(pos, 99.9)),
        "max": float(np.max(pos)),
        "counts": {t: int(np.sum(pos > t)) for t in THRESHOLDS},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--presets-a", required=True)
    ap.add_argument("--presets-b", required=True)
    ap.add_argument("--tuner-dir", default=TUNER_DIR)
    args = ap.parse_args()

    exe = os.path.join(args.build_dir, "imitator.exe")
    names = []
    for fn in sorted(os.listdir(args.tuner_dir)):
        if fn.startswith("tuner_config.") and fn.endswith(".json"):
            n = fn[len("tuner_config."):-len(".json")]
            if n and n not in SKIP_NAMES:
                names.append(n)

    print("%-22s %6s %10s %10s %8s | %s" % (
        "set", "n", "max_A", "max_B", "dmax",
        "n>0.5m A/B   n>1m A/B   n>2m A/B   n>5m A/B"))
    for name in names:
        cfg = json.load(open(os.path.join(TUNER_DIR, "tuner_config.%s.json" % name),
                             encoding="utf-8"))
        data_dir = cfg["data_dir"]
        skip = float(cfg["horizon"]["skip_first_s"])
        tf = float(cfg["horizon"]["train_frac"])
        base = os.path.basename(cfg["output"]["preset"])
        if not os.path.exists(os.path.join(data_dir, "truth.txt")):
            print("%-22s  нет truth.txt" % name)
            continue
        a = stat_one(exe, data_dir, os.path.join(args.presets_a, base), skip, tf)
        b = stat_one(exe, data_dir, os.path.join(args.presets_b, base), skip, tf)
        if a is None or b is None:
            print("%-22s  ошибка расчёта" % name)
            continue
        counts = "  ".join("%4d/%-4d" % (a["counts"][t], b["counts"][t])
                           for t in THRESHOLDS)
        print("%-22s %6d %10.2f %10.2f %8.2f | %s" % (
            name, a["n"], a["max"], b["max"], b["max"] - a["max"], counts))
        print("    P95 A=%.3f B=%.3f | P99.9 A=%.3f B=%.3f" % (
            a["p95"], b["p95"], a["p999"], b["p999"]))


if __name__ == "__main__":
    main()
