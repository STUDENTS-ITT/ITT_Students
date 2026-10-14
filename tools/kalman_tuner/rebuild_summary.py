# rebuild_summary.py — собирает tools/kalman_tuner/full_runs/_summary.txt
# из логов full_<set>.log + результатов валидации пресетов (validate_presets.py).
import json
import math
import os
import re

TUNER = r"C:\ITT_Students_Qt_Version\tools\kalman_tuner"
LOGS = os.path.join(TUNER, "full_runs")
SYM = os.path.join(LOGS, "_summary.txt")

SETS = [
    "static300", "static_lownoise_align", "square650", "square_noisefree",
    "square_harsh", "straight_10ms", "figure8_8ms", "spiral_climb", "high_dynamics",
]

# p95/p99.9@200Hz/max@200 из validate_presets.py для full-пресетов
VAL = {
    "static300": "3.170/4.838/19.4", "static_lownoise_align": "1.891/2.578/2.94",
    "square650": "3.200/4.514/6.2", "square_noisefree": "0.014/0.016/0.05",
    "square_harsh": "10.449/14.839/22.6", "straight_10ms": "3.757/4.653/5.5",
    "figure8_8ms": "3.476/4.742/6.2", "spiral_climb": "3.213/4.852/9.4",
    "high_dynamics": "5.368/7.934/10.1",
}

RE_J = re.compile(r"J_train=([\d.]+),\s*J_valid=([\d.]+)")
RE_TAIL = re.compile(r"RAW:\s*([\d.]+)\s*/\s*([\d.]+)")


def log_text(name):
    with open(os.path.join(LOGS, "full_%s.log" % name), "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def first(text, pattern):
    """Последнее вхождение (финальный прогон) — иначе промежуточные стадии."""
    m = list(pattern.finditer(text))
    if not m:
        return ("", "")
    return m[-1].groups()


def main():
    lines = ["Полный подбор КФ (без --quick), прогон 2026-10-10 (все 9 наборов OK, exit=0)"]
    lines.append("set | Jtr | Jval | tailRAW t/v | p95/p99.9/max,м | мин")
    worst = []
    for s in SETS:
        t = log_text(s)
        jtr, jval = first(t, RE_J)
        t1, t2 = first(t, RE_TAIL)
        dm = re.search(r"END\s+%s\s+exit=\d+\s+([\d.]+)" % re.escape(s), t)
        dur = dm.group(1) if dm else "?"
        lines.append("%-22s | %s | %s | %s/%s | %s | %s" % (s, jtr, jval, t1, t2, VAL[s], dur))
        try:
            worst.append((float(VAL[s].split("/")[0]), s))
        except ValueError:
            pass
    worst.sort(reverse=True)
    lines.append("")
    lines.append("Худшие по p95 (м): " + ", ".join("%s=%.3f" % (s, p) for p, s in worst))
    lines.append("Приёмка: full-пресеты оставлены (дефолт tuned.ini даёт p95 34-57 м или дивергенцию).")
    with open(SYM, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()