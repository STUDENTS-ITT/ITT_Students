# gen_truth_configs.py — генерирует конфиги подбора по truth-метрике (этап 13A).
# Копирует tuner_config.<set>.json, переключая work_root и выходной пресет в
# отдельные каталоги, чтобы не перетереть штатные presets/tuned_<set>.ini.

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT_CFG = os.path.join(HERE, "truth_cfg")
OUT_PRESET_DIR = os.path.join(REPO, "presets", "truth")

SETS = [
    "static300",
    "static_lownoise_align",
    "square650",
    "square_harsh",
    "straight_10ms",
    "figure8_8ms",
    "spiral_climb",
    "high_dynamics",
]


def main():
    os.makedirs(OUT_CFG, exist_ok=True)
    os.makedirs(OUT_PRESET_DIR, exist_ok=True)
    for s in SETS:
        src = os.path.join(HERE, "tuner_config.%s.json" % s)
        with open(src, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        truth = os.path.join(cfg["data_dir"], "truth.txt")
        if not os.path.exists(truth):
            raise SystemExit("нет truth.txt для %s: %s" % (s, truth))
        cfg["work_root"] = "temp_tune_truth_%s" % s
        cfg["output"]["preset"] = (
            OUT_PRESET_DIR.replace("\\", "/") + "/tuned_%s.ini" % s)
        dst = os.path.join(OUT_CFG, "cfg_%s.json" % s)
        with open(dst, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print("wrote", dst, "->", cfg["output"]["preset"])


if __name__ == "__main__":
    main()
