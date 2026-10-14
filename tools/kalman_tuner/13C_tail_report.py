# 13C_tail_report.py — робастность хвоста: оконный квантиль (окна ~5 с),
# доля выбросов, куртозис распределения ошибки позиции (200 Гц), и локализация
# одиночного артефакта (время максимума). Этап 13C (только метрика тюнера).
#
# Протокол оценки совпадает с compare_presets_stats.pos_near_valid и
# tune_kalman.compute_error: truth интерполируется на времена тактов result;
# горизонт отбрасывается (skip_first_s); метрики — на валид-окне (train_frac..1).
#
# Использование:
#   python 13C_tail_report.py --build-dir <...> --presets <dir> [--out <md>]

import argparse
import json
import os
import subprocess
import tempfile

import numpy as np

TUNER_DIR = os.path.dirname(os.path.abspath(__file__))
THRESHOLDS = (0.5, 1.0, 2.0, 5.0)
SKIP_NAMES = ("smoke", "square_noisefree")
WINDOW_S = 5.0
TAIL_Q = 99.9
SETS = ["static300", "static_lownoise_align", "square650", "square_harsh",
        "straight_10ms", "figure8_8ms", "spiral_climb", "high_dynamics"]


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


def diag(exe, data_dir, preset, skip_first_s, train_frac, win_s=WINDOW_S):
    wd = tempfile.mkdtemp(prefix="c13_")
    res = load_text(run_imitator(exe, data_dir, preset, wd))
    truth = load_text(os.path.join(data_dir, "truth.txt"))
    if res is None or truth is None:
        return None
    t_res = res[:, 0]
    # Интерполяция truth на каждый такт result (200 Гц).
    lat_i = np.interp(t_res, truth[:, 0], truth[:, 2])
    lon_i = np.interp(t_res, truth[:, 0], truth[:, 1])
    d_n = (res[:, 2] - lat_i) * 111000.0
    d_e = (res[:, 1] - lon_i) * 111000.0 * np.cos(np.deg2rad(lat_i))
    pos = np.hypot(d_n, d_e)

    keep = t_res >= skip_first_s
    tt, pp = t_res[keep], pos[keep]
    if len(tt) < 20:
        return None
    split = tt[0] + (tt[-1] - tt[0]) * train_frac
    tv, pv = tt[tt > split], pp[tt > split]
    if len(tv) < 10:
        return None

    # Глобальный максимум (локализация артефакта).
    imax = int(np.argmax(pv))
    # Оконный квантиль TAIL_Q.
    widx = ((tv - tv[0]) // win_s).astype(int)
    worst_q, worst_t, win_qs = 0.0, tv[0], []
    for w in np.unique(widx):
        seg = pv[widx == w]
        if len(seg) >= 5:
            q = float(np.percentile(seg, TAIL_Q))
            win_qs.append(q)
            if q > worst_q:
                worst_q, worst_t = q, float(tv[widx == w][0])
    mu, sd = float(np.mean(pv)), float(np.std(pv))
    kurt = float(np.mean(((pv - mu) / sd) ** 4)) if sd > 0 else 0.0
    return {
        "n": int(len(pv)),
        "p95": float(np.percentile(pv, 95)),
        "p999": float(np.percentile(pv, TAIL_Q)),
        "max": float(pv[imax]),
        "max_t": float(tv[imax]),
        "win_q": worst_q,
        "win_t": worst_t,
        "n_win": len(win_qs),
        "kurt": kurt,
        "counts": {t: int(np.sum(pv > t)) for t in THRESHOLDS},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--presets", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    exe = os.path.join(args.build_dir, "imitator.exe")
    tag = args.tag or os.path.basename(os.path.normpath(args.presets))

    lines = []
    lines.append("## 13C — робастность хвоста (%s)" % tag)
    lines.append("")
    lines.append("Окна %.0f с, квантиль P%g@200Гц; выбросы считаются по тактам "
                 "result (200 Гц) на валид-окне." % (WINDOW_S, TAIL_Q))
    lines.append("")
    lines.append("| Набор | P95 | P99.9 | max | max@t,с | окн. P%g | окн.@t,с | >0.5 | >1 | >2 | >5 | куртозис |"
                 % TAIL_Q)
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name in SETS:
        cf = os.path.join(TUNER_DIR, "tuner_config.%s.json" % name)
        if not os.path.exists(cf):
            continue
        cfg = json.load(open(cf, encoding="utf-8"))
        data_dir = cfg["data_dir"]
        skip = float(cfg["horizon"]["skip_first_s"])
        tf = float(cfg["horizon"]["train_frac"])
        base = os.path.basename(cfg["output"]["preset"])
        if not os.path.exists(os.path.join(data_dir, "truth.txt")):
            lines.append("| %s | нет truth.txt |" % name)
            continue
        d = diag(exe, data_dir, os.path.join(args.presets, base), skip, tf)
        if d is None:
            lines.append("| %s | ошибка расчёта |" % name)
            continue
        cnt = " | ".join(str(d["counts"][t]) for t in THRESHOLDS)
        lines.append("| %s | %.3f | %.3f | %.3f | %.1f | %.3f | %.1f | %s | %.2f |" % (
            name, d["p95"], d["p999"], d["max"], d["max_t"],
            d["win_q"], d["win_t"], cnt, d["kurt"]))
        print("%-22s win_q=%.3f@%.1fs max=%.3f@%.1fs kurt=%.2f" % (
            name, d["win_q"], d["win_t"], d["max"], d["max_t"], d["kurt"]), flush=True)

    text = "\n".join(lines) + "\n"
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
