# 13g_report.py — сводка этапа 13G: сравнение J_train/J_valid по вариантам
# 13A/13B/13G и контроль «стенок» (u у границы интервала) по best_params.txt.
#
# Только чтение артефактов тюнера — прогоны не запускает.
#
# Использование:
#   python 13g_report.py            # таблица на экран (markdown-ish)
#   python 13g_report.py --md out.md

import argparse
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))

SETS = [
    "static300", "static_lownoise_align", "square650", "square_harsh",
    "straight_10ms", "figure8_8ms", "spiral_climb", "high_dynamics",
]

# вариант: (метка, шаблон каталога прогона, шаблон конфига)
VARIANTS = [
    ("13A", "temp_tune_truth_{s}", "truth_cfg/cfg_{s}.json"),
    ("13B", "temp_tune_13b_{s}", "truth_cfg_13b/cfg_{s}.json"),
    ("13G", "temp_tune_13g_{s}", "truth_cfg_13g/cfg_{s}.json"),
]

SIG_LINE = re.compile(r"^\s*(sigma_\w+)\s+([0-9.eE+-]+)\s+([-0-9.]+)\s*$")


def parse_best(path):
    """Читает best_params.txt: J_train/J_valid и u по каждому sigma."""
    if not os.path.exists(path):
        return None
    txt = open(path, encoding="utf-8", errors="replace").read()
    out = {"J_train": None, "J_valid": None, "u": {}}
    m = re.search(r"J_train\s*:\s*([0-9.]+)", txt)
    if m:
        out["J_train"] = float(m.group(1))
    m = re.search(r"J_valid\s*:\s*([0-9.]+)", txt)
    if m:
        out["J_valid"] = float(m.group(1))
    in_sigma = False
    for line in txt.splitlines():
        if "sigma (СИ)" in line:
            in_sigma = True
            continue
        if in_sigma:
            if line.strip().startswith("---"):
                break
            mm = SIG_LINE.match(line)
            if mm:
                out["u"][mm.group(1)] = float(mm.group(3))
    return out


def bounds_of(cfg_path):
    """u-границы по конфигу: u_span (симметр.) или u_lo/u_hi; fixed → (u,u)."""
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    b = {}
    for p in cfg.get("params", []):
        n = p["name"]
        if "fixed" in p:
            b[n] = (0.0, 0.0)
        elif "u_lo" in p and "u_hi" in p:
            b[n] = (float(p["u_lo"]), float(p["u_hi"]))
        else:
            s = float(p.get("u_span", 1.0))
            b[n] = (-s, s)
    return b


def walls(u, bounds, eps_frac=0.05, margin=0.10):
    """Список осей, «прилипших» к стенке: u у границы (по eps_frac ширины)."""
    bad = []
    for n, x in u.items():
        lo, hi = bounds.get(n, (None, None))
        if lo is None or hi is None:
            continue
        width = hi - lo
        if width <= 0:
            continue
        eps = max(eps_frac * width, margin)
        if x <= lo + eps:
            bad.append(f"{n}↓{x:+.2f}")
        elif x >= hi - eps:
            bad.append(f"{n}↑{x:+.2f}")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", default="", help="записать markdown-таблицу в файл")
    args = ap.parse_args()

    lines = []
    lines.append("| Набор | J_valid 13A | J_valid 13B | J_valid 13G | Δ(13G-13B) | стенки 13G |")
    lines.append("|---|---|---|---|---|---|")
    all_rows = []
    n_wall = 0
    for s in SETS:
        row = {"set": s}
        for tag, wd_t, cfg_t in VARIANTS:
            best = parse_best(os.path.join(HERE, wd_t.format(s=s), "best_params.txt"))
            row[tag] = best
        b13g = row["13G"]
        w = []
        if b13g:
            bounds = bounds_of(os.path.join(HERE, VARIANTS[2][2].format(s=s)))
            w = walls(b13g["u"], bounds)
        n_wall += len(w)
        jv = {t: (row[t]["J_valid"] if row[t] else None) for t in ("13A", "13B", "13G")}
        d = (jv["13G"] - jv["13B"]) if (jv["13G"] is not None and jv["13B"] is not None) else None
        fmt = lambda x: f"{x:.4f}" if x is not None else "—"
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            s, fmt(jv["13A"]), fmt(jv["13B"]), fmt(jv["13G"]),
            (f"{d:+.4f}" if d is not None else "—"),
            (", ".join(w) if w else "нет")))
        all_rows.append(row)

    lines.append("")
    lines.append(f"Всего стенок в 13G: {n_wall}")
    text = "\n".join(lines)
    print(text)

    if args.md:
        with open(args.md, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"\nзаписано: {os.path.abspath(args.md)}")


if __name__ == "__main__":
    main()
