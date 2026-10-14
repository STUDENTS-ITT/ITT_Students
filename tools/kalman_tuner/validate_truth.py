# validate_truth.py — сравнение метрики «по бортовому эталону» (reference.txt)
# и «по наземной истине» (truth.txt) в оптимуме текущих per-set пресетов
# (этап 13A, диагностика смещения цели подбора).
#
# Для каждого набора запускает imitator с presets/tuned_<набор>.ini и считает
# J против reference.txt и против truth.txt одной и той же функцией
# tune_kalman.compute_error.
#
# Использование:
#   python validate_truth.py --build-dir ... --presets-dir ... --tuner-dir ...
#                            [--out-dir ...]

import argparse
import importlib.util
import json
import os
import tempfile

TUNER_DIR = os.path.dirname(os.path.abspath(__file__))


def _load(modname, filename):
    spec = importlib.util.spec_from_file_location(
        modname, os.path.join(TUNER_DIR, filename))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--presets-dir", required=True)
    ap.add_argument("--tuner-dir", default=TUNER_DIR)
    ap.add_argument("--out-dir",
                    default=os.path.join(tempfile.gettempdir(), "etap13_truth"))
    args = ap.parse_args()

    tk = _load("tk", "tune_kalman.py")
    vp = _load("vp", "validate_presets.py")
    exe = os.path.join(args.build_dir, "imitator.exe")
    os.makedirs(args.out_dir, exist_ok=True)

    names = []
    for fn in sorted(os.listdir(args.tuner_dir)):
        if fn.startswith("tuner_config.") and fn.endswith(".json"):
            n = fn[len("tuner_config."):-len(".json")]
            if n and n != "smoke":
                names.append(n)

    print("%-24s %14s %14s %10s %10s %8s" % (
        "set", "P95_ref", "P95_truth", "Jref_tr", "Jtru_tr", "ratio"))
    for name in names:
        with open(os.path.join(args.tuner_dir, "tuner_config.%s.json" % name),
                  "r", encoding="utf-8") as f:
            cfg = json.load(f)
        data_dir = cfg["data_dir"]
        skip = float(cfg["horizon"]["skip_first_s"])
        train_frac = float(cfg["horizon"]["train_frac"])
        preset = os.path.join(args.presets_dir,
                              os.path.basename(cfg["output"]["preset"]))
        truth = os.path.join(data_dir, "truth.txt")
        if not os.path.exists(truth):
            print("%-24s  нет truth.txt" % name)
            continue

        wd = tempfile.mkdtemp(prefix="vt_%s_" % name, dir=args.out_dir)
        res_p, ref_p = vp.run_imitator(exe, data_dir, preset, wd)
        jr_tr, jr_va, sr = tk.compute_error(res_p, ref_p, skip, train_frac)
        jt_tr, jt_va, st = tk.compute_error(res_p, truth, skip, train_frac)
        p95r = sr["p95_pos_valid"] if sr else float("nan")
        p95t = st["p95_pos_valid"] if st else float("nan")
        ratio = (p95t / p95r) if (sr and st and p95r) else float("nan")
        print("%-24s %14.3f %14.3f %10.3f %10.3f %8.2f" % (
            name, p95r, p95t, jr_tr, jt_tr, ratio))


if __name__ == "__main__":
    main()
