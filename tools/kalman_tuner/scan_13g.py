#!/usr/bin/env python3
"""scan_13g.py — координатный профиль J для этапа 13G.

Строит зависимость метрики J (без регуляризации и штрафа за стенку) от
АБСОЛЮТНОГО значения одного коэффициента КФ вдоль выбранной оси, при
фиксированных остальных коэффициентах. Служит для проверки гипотезы 13G:

  * если профиль J(σ) имеет ВНУТРЕННИЙ минимум (не на краю диапазона и с
    положительной кривизной) — ось ИДЕНТИФИЦИРУЕМА, её можно оставлять в
    подборе;
  * если профиль плоский или улучшается к краю — ось НЕ идентифицируема,
    значение следует ОСОЗНАННО зафиксировать (паспорт/пол).

Профиль считается по методике 13A/13B: сравнение с наземной истиной
(truth.txt), горизонт skip_first_s, train/valid-разбиение.

Запуск:
  python -u scan_13g.py --config=truth_cfg/cfg_static300.json \\
      --axis=sigma_bg --grid=1e-8,1e-3,9 \\
      --base=../presets/truth_13b/tuned_static300.ini --out=full_runs/_13G_scan_static300_sigma_bg.csv
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tune_kalman as tk  # noqa: E402

# Соответствие ключа пресета (params.ini [filter]) имени оси тюнера.
INI_TO_TUNER = {
    "sigma_g": "sigma_g", "sigma_a": "sigma_a",
    "sigma_bg": "sigma_bg", "sigma_ba": "sigma_ba",
    "sigma_pos": "sigma_pos", "sigma_h": "sigma_h",
    "sigma_v": "sigma_v", "sigma_hdg": "sigma_ang",
}


def read_preset(path):
    """Читает presets/*.ini ([filter]) → словарь имя_тюнера → σ."""
    sig = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.strip()
        if line.startswith(("#", "[")) or "=" not in line:
            continue
        key, val = [x.strip() for x in line.split("=", 1)]
        if key in INI_TO_TUNER:
            sig[INI_TO_TUNER[key]] = float(val)
    return sig


def base_sigma(config_path, preset_path):
    """Базовый вектор σ: из пресета, иначе — σ_ref из config."""
    if preset_path and os.path.exists(preset_path):
        sig = read_preset(preset_path)
        missing = [n for n, _, _ in tk.PARAMS if n not in sig]
        if missing:
            raise SystemExit(f"В пресете {preset_path} нет ключей: {missing}")
        return sig
    return {n: ref for n, ref, _ in tk.PARAMS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="tuner_config*.json (задаёт DATA_DIR, PARAMS, горизонт)")
    ap.add_argument("--axis", required=True, help="имя оси: sigma_g|sigma_a|sigma_bg|sigma_ba|sigma_pos|sigma_h|sigma_v|sigma_ang")
    ap.add_argument("--grid", required=True, help="lo,hi,n — абсолютные σ через запятую (лог-равномерно)")
    ap.add_argument("--base", default="", help="presets/*.ini с фиксированными остальными σ")
    ap.add_argument("--out", default="", help="CSV для сохранения профиля")
    ap.add_argument("--work-parent", default="", help="каталог для временных прогонов")
    args = ap.parse_args()

    tk.load_config(os.path.abspath(args.config))
    # Профиль — «чистая» метрика: без reg и штрафа за стенку.
    tk.REG_LAMBDA = 0.0
    tk.WALL_PENALTY_W = 0.0
    os.environ["TUNE_REF_TRUTH"] = "1"

    names = [n for n, _, _ in tk.PARAMS]
    if args.axis not in names:
        raise SystemExit(f"--axis={args.axis} не из {names}")
    axis_i = names.index(args.axis)
    refs = {n: ref for n, ref, _ in tk.PARAMS}

    lo, hi, n = (float(x) for x in args.grid.split(","))
    grid = np.logspace(np.log10(lo), np.log10(hi), int(n))

    sig0 = base_sigma(args.config, args.base)
    work_parent = args.work_parent or os.path.join(tk.WORK_ROOT, "scan_13g")
    os.makedirs(work_parent, exist_ok=True)

    print(f"13G профиль: набор={os.path.basename(tk.DATA_DIR)}  ось={args.axis}")
    print(f"  база: {args.base or '(σ_ref из конфига)'}")
    print(f"  остальные σ: " + ", ".join(f"{k}={sig0[k]:.4g}" for k in names if k != args.axis))
    print(f"  сетка {args.axis}: {grid[0]:.3e} .. {grid[-1]:.3e} ({len(grid)} точек)")
    print(f"  {'σ':>12} {'u':>8} {'J_train':>9} {'J_valid':>9} "
          f"{'P95pos_v':>9} {'P99.9_v':>9} {'t,с':>6}")

    rows = []
    t_all = time.monotonic()
    for k, s in enumerate(grid):
        sig = dict(sig0)
        sig[args.axis] = float(s)
        u = [np.log10(sig[nm] / refs[nm]) for nm in names]
        work = os.path.join(work_parent, f"{args.axis}_{k:02d}")
        t0 = time.monotonic()
        Jt, Jv, summary, status = tk.run_eval(u, work)
        dt = time.monotonic() - t0
        ok = summary is not None
        p95 = summary.get("p95_pos_valid", float("nan")) if ok else float("nan")
        p999 = summary.get("tail_pos_valid", float("nan")) if ok else float("nan")
        rows.append({"sigma": float(s), "u": u[axis_i], "J_train": Jt,
                     "J_valid": Jv, "p95_pos_valid": p95,
                     "tail_pos_valid": p999, "status": status or "ok"})
        print(f"  {s:12.4e} {u[axis_i]:8.3f} {Jt:9.4f} {Jv:9.4f} "
              f"{p95:9.4f} {p999:9.4f} {dt:6.1f}"
              + ("" if ok else f"  [{status}]"))

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"  CSV: {os.path.abspath(args.out)}")

    ok_rows = [r for r in rows if r["status"] == "ok"]
    if len(ok_rows) == len(rows) and len(rows) >= 3:
        jv = np.array([r["J_valid"] for r in rows])
        kmin = int(np.argmin(jv))
        interior = 0 < kmin < len(rows) - 1
        print(f"\n  минимум J_valid при {args.axis}={rows[kmin]['sigma']:.4e} "
              f"(точка {kmin + 1}/{len(rows)}, "
              f"{'внутренний' if interior else 'КРАЙНИЙ — ось не идентифицируема'})")
    print(f"  всего {time.monotonic() - t_all:.1f} с")


if __name__ == "__main__":
    main()
