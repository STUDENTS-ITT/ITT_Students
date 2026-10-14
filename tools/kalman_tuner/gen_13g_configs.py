# gen_13g_configs.py - генерация конфигов для 13G: фиксированные УНИВЕРСАЛЬНЫЕ
# симметричные u-интервалы (спаны 13A) + штраф за стенку. В отличие от 13B
# асимметричные u_lo/u_hi, выведенные из оптимумов имеющихся наборов, не
# используются (принцип универсальности интервалов, 2026-10-12).
#
# sigma_ref берётся из 13A-конфигов (глобальный шаблон, одинаков для всех
# наборов). sigma_bg/sigma_ba НЕ фиксируются (директива: смещения ИМУ
# неизвестны, подбираются все коэффициенты).

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT_CFG = os.path.join(HERE, "truth_cfg_13g")
OUT_PRESET_DIR = os.path.join(REPO, "presets", "truth_13g")

SETS = [
    "static300", "static_lownoise_align", "square650", "square_harsh",
    "straight_10ms", "figure8_8ms", "spiral_climb", "high_dynamics",
]


def main():
    os.makedirs(OUT_CFG, exist_ok=True)
    os.makedirs(OUT_PRESET_DIR, exist_ok=True)
    for s in SETS:
        src = os.path.join(HERE, "truth_cfg", f"cfg_{s}.json")  # база — 13A
        with open(src, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        params = cfg.get("params", [])
        for p in params:
            # Универсальные симметричные интервалы: убираем любые асимметричные
            # границы и «фиксации», оставляя только u_span (спаны 13A).
            p.pop("u_lo", None)
            p.pop("u_hi", None)
            p.pop("u_bounds", None)
            p.pop("fixed", None)
        cfg["work_root"] = f"temp_tune_13g_{s}"
        cfg["output"]["preset"] = os.path.join(
            OUT_PRESET_DIR, f"tuned_{s}.ini").replace("\\", "/")
        # Штраф за близость к стенке (механизм оптимизатора, не выводится из
        # оптимумов) — как в 13B.
        cfg["optim"]["wall_penalty"] = {"eps_frac": 0.05, "w": 0.1}
        dst = os.path.join(OUT_CFG, f"cfg_{s}.json")
        with open(dst, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print("wrote", dst, "->", cfg["output"]["preset"])


if __name__ == "__main__":
    main()
