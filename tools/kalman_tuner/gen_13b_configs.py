# gen_13b_configs.py - генерация конфигов для 13B с асимметричными интервалами
# и возможным wall penalty. sigma_ref берём из текущих truth-конфигов (по params.json).

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT_CFG = os.path.join(HERE, "truth_cfg_13b")
OUT_PRESET_DIR = os.path.join(REPO, "presets", "truth_13b")

# Предложенные асимметричные u-границы (лог10)
U_BOUNDS = {
  "sigma_g":   (-0.51, 1.62),
  "sigma_a":   (-0.91, 2.09),
  "sigma_bg":  (-0.7, 0.72),
  "sigma_ba":  (-0.59, 1.95),
  "sigma_pos": (0.05, 3.19),
  "sigma_h":   (-1.55, 0.48),
  "sigma_v":   (-1.12, 1.03),
  "sigma_ang": (-0.49, 0.75),
}

SETS = [
  "static300","static_lownoise_align","square650","square_harsh",
  "straight_10ms","figure8_8ms","spiral_climb","high_dynamics",
]

def main():
  os.makedirs(OUT_CFG, exist_ok=True)
  os.makedirs(OUT_PRESET_DIR, exist_ok=True)
  for s in SETS:
    src = os.path.join(HERE, "truth_cfg", f"cfg_{s}.json")  # базируемся на 13A-конфиге
    if not os.path.exists(src):
      src = os.path.join(HERE, f"tuner_config.{s}.json")
    with open(src, 'r', encoding='utf-8') as f:
      cfg = json.load(f)
    # обновим интервалы
    params = cfg.get('params', [])
    for p in params:
      name = p['name']
      if name in U_BOUNDS:
        lo, hi = U_BOUNDS[name]
        p['u_lo'], p['u_hi'] = lo, hi
    cfg['work_root'] = f"temp_tune_13b_{s}"
    cfg['output']['preset'] = os.path.join(OUT_PRESET_DIR, f"tuned_{s}.ini").replace('\\','/')
    # пометка для 13B
    cfg['optim']['wall_penalty'] = {"eps_frac": 0.05, "w": 0.1}
    dst = os.path.join(OUT_CFG, f"cfg_{s}.json")
    with open(dst, 'w', encoding='utf-8') as f:
      json.dump(cfg, f, ensure_ascii=False, indent=2)
      f.write('\n')
    print('wrote', dst)
if __name__=='__main__':
  main()
