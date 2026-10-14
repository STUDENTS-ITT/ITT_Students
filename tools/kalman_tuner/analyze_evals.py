# analyze_evals.py — диагностика полных прогонов подбора (этап 13, шаг 13.0).
#
# Читает артефакты eval.json из каталогов temp_tune_<набор>/ (создаются
# tune_kalman.py для каждой оценки) и печатает по каждому набору:
#   * число оценок, лучшую точку (min J_train);
#   * для каждого sigma — положение лучшей точки в u-координате, «упор» в
#     стенку, корреляцию u с J_train и долю оценок у стенки.
# Дополнительно — сводку: сколько наборов держат ось у стенки.
#
# Использование:
#   python analyze_evals.py [--tuner-dir .]

import argparse
import glob
import json
import os

import numpy as np


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def find_configs(tuner_dir):
    configs = {}
    for path in sorted(glob.glob(os.path.join(tuner_dir, "tuner_config.*.json"))):
        name = os.path.basename(path)[len("tuner_config."):-len(".json")]
        if name and name != "smoke":
            configs[name] = path
    return configs


def collect_evals(tuner_dir, work_root):
    records = []
    root = os.path.join(tuner_dir, work_root)
    for path in glob.glob(os.path.join(root, "**", "eval.json"), recursive=True):
        try:
            data = load_json(path)
        except Exception:
            continue
        if "u" not in data or "J_train" not in data:
            continue
        records.append({
            "u": np.asarray(data["u"], dtype=float),
            "J_train": float(data["J_train"]),
            "J_valid": float(data.get("J_valid", float("nan"))),
        })
    return records


def corr(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 3 or np.std(x[m]) == 0.0 or np.std(y[m]) == 0.0:
        return float("nan")
    return float(np.corrcoef(x[m], y[m])[0, 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tuner-dir", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--wall-eps", type=float, default=0.02)
    args = ap.parse_args()

    tuner_dir = args.tuner_dir
    configs = find_configs(tuner_dir)

    print("=== Diagnostika eval.json (etap 13, shag 13.0) ===")
    print("wall-eps = %.3f\n" % args.wall_eps)

    wall_count = {}
    param_names = None
    global_best = []

    for name in sorted(configs):
        cfg = load_json(configs[name])
        params = cfg["params"]
        names = [p["name"] for p in params]
        span = np.array([float(p["u_span"]) for p in params], dtype=float)
        param_names = names

        work_root = cfg.get("work_root", "temp_tune_" + name)
        recs = collect_evals(tuner_dir, work_root)
        if not recs:
            print("%s: no eval.json (dir %s)\n" % (name, work_root))
            continue

        U = np.vstack([r["u"] for r in recs])
        J = np.array([r["J_train"] for r in recs], dtype=float)
        Jv = np.array([r["J_valid"] for r in recs], dtype=float)

        ibest = int(np.nanargmin(J))
        ubest = U[ibest]
        rel = np.abs(ubest) / span

        print("SET %-22s n=%-4d  J_train best=%.4f  J_valid@best=%.4f"
              % (name, len(recs), J[ibest], Jv[ibest]))
        print("  %-9s %6s %8s %9s %8s %10s"
              % ("param", "span", "u@best", "|u|/span", "corr", "wall_frac"))

        nwall_best = 0
        for i, pn in enumerate(names):
            wall_frac = float(np.mean(np.abs(U[:, i]) >= span[i] - args.wall_eps))
            c = corr(U[:, i], J)
            at_wall = rel[i] >= 1.0 - 1e-6
            if at_wall:
                nwall_best += 1
                wall_count[pn] = wall_count.get(pn, 0) + 1
            print("  %-9s %6.2f %8.3f %9.3f %8.3f %10.2f %s"
                  % (pn, span[i], ubest[i], rel[i], c, wall_frac,
                     "<-- WALL" if at_wall else ""))
        print("  best at wall: %d/%d\n" % (nwall_best, len(names)))
        global_best.append((J[ibest], name, ubest, span))

    print("=== Wall summary (best point |u|==span) ===")
    if param_names:
        for pn in param_names:
            print("  %-9s %d sets" % (pn, wall_count.get(pn, 0)))


if __name__ == "__main__":
    main()
