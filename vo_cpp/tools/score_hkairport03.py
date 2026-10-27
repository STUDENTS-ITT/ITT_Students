"""Honest HKairport03 metrics: prefix SE(2), s=1, cut t=105s."""
import csv
import sys
from math import atan2, hypot, cos, sin, sqrt, degrees


def score(vo_path):
    root = r"D:\Cursor_projects\VO"
    gps_path = root + r"\gps_reference.csv"
    cam_path = root + r"\camera\timestamps.csv"
    cam_t = []
    with open(cam_path, newline="") as f:
        for row in csv.DictReader(f):
            cam_t.append(float(row["stamp_sec"]) + float(row["stamp_nsec"]) * 1e-9)
    t0 = cam_t[0]
    cam_t = [t - t0 for t in cam_t]
    vo = []
    with open(vo_path, newline="") as f:
        for r in csv.DictReader(f):
            fr = int(float(r["Frame"])) - 1
            vo.append((cam_t[fr], float(r["X"]), float(r["Y"])))
    rtk = []
    with open(gps_path, newline="") as f:
        for r in csv.DictReader(f):
            rtk.append((float(r["time_s"]), float(r["east_m"]), float(r["north_m"])))

    def at(series, t):
        lo, hi = 0, len(series) - 1
        if t <= series[0][0]:
            return series[0]
        if t >= series[-1][0]:
            return series[-1]
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if series[mid][0] < t:
                lo = mid
            else:
                hi = mid
        a, b = series[lo], series[hi]
        w = (t - a[0]) / (b[0] - a[0]) if b[0] != a[0] else 0.0
        return (t, a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]))

    truth = [at(rtk, p[0]) for p in vo]
    s = [0.0]
    for i in range(1, len(truth)):
        s.append(s[-1] + hypot(truth[i][1] - truth[i - 1][1], truth[i][2] - truth[i - 1][2]))
    cap = 0.10 * s[-1]
    i1 = next((i for i, v in enumerate(s) if v > cap), len(s) - 1)
    cut = next((i for i in range(i1) if vo[i][0] >= 105), i1)
    if cut >= 20:
        i1 = cut
    n = float(i1)
    ct = [sum(p[1] for p in vo[:i1]) / n, sum(p[2] for p in vo[:i1]) / n]
    cg = [sum(p[1] for p in truth[:i1]) / n, sum(p[2] for p in truth[:i1]) / n]
    dot = cross = 0.0
    for i in range(i1):
        ax, ay = vo[i][1] - ct[0], vo[i][2] - ct[1]
        bx, by = truth[i][1] - cg[0], truth[i][2] - cg[1]
        dot += ax * bx + ay * by
        cross += ax * by - ay * bx
    ang = atan2(cross, dot)
    c, sn = cos(ang), sin(ang)
    fitted = []
    err = []
    for i in range(len(vo)):
        x, y = vo[i][1] - ct[0], vo[i][2] - ct[1]
        fx, fy = c * x - sn * y + cg[0], sn * x + c * y + cg[1]
        fitted.append((fx, fy))
        err.append(hypot(fx - truth[i][1], fy - truth[i][2]))
    rms = sqrt(sum(e * e for e in err) / len(err))
    path_vo = sum(
        hypot(fitted[i][0] - fitted[i - 1][0], fitted[i][1] - fitted[i - 1][1])
        for i in range(1, len(fitted))
    )
    path_rtk = s[-1]

    def idx_at(t):
        return min(range(len(vo)), key=lambda i: abs(vo[i][0] - t))

    def seg_delta(t0s, t1s):
        i, j = idx_at(t0s), idx_at(t1s)
        av = degrees(atan2(fitted[j][1] - fitted[i][1], fitted[j][0] - fitted[i][0]))
        ag = degrees(atan2(truth[j][2] - truth[i][2], truth[j][1] - truth[i][1]))
        return (av - ag + 180) % 360 - 180

    return {
        "ate": rms,
        "max": max(err),
        "end": err[-1],
        "path_vo": path_vo,
        "path_rtk": path_rtk,
        "ratio": path_vo / path_rtk,
        "leg2": seg_delta(120, 170),
        "leg3": seg_delta(175, 220),
    }


if __name__ == "__main__":
    m = score(sys.argv[1])
    print(
        "ATE %.2f max %.2f end %.2f path %.1f ratio %.3f leg2 %.1f leg3 %.1f"
        % (m["ate"], m["max"], m["end"], m["path_vo"], m["ratio"], m["leg2"], m["leg3"])
    )
