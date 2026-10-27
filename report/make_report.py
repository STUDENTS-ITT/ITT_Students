# -*- coding: utf-8 -*-
"""Figures + PDF report for the kept HKairport03 VO result (eval_arc)."""
from __future__ import annotations

import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from fpdf import FPDF

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAJ = os.path.join(
    ROOT,
    r"results_HKairport03\homography\visual_odometry\FAST-FREAK\trajectory.csv",
)
GPS = os.path.join(ROOT, "gps_reference.csv")
CAM = os.path.join(ROOT, r"camera\timestamps.csv")
FIG = os.path.join(ROOT, r"report\figures")
OUT_PDF = os.path.join(ROOT, r"report\VO_HKairport03.pdf")
VIDEO = os.path.join(ROOT, r"report\tracking_takeoff_small.mp4")
if not os.path.exists(VIDEO):
    VIDEO = os.path.join(ROOT, r"report\tracking_takeoff.mp4")
os.makedirs(FIG, exist_ok=True)

plt.rcParams["font.family"] = "DejaVu Sans"


def load_cam():
    t = []
    with open(CAM, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["stamp_sec"]) + float(row["stamp_nsec"]) * 1e-9)
    t0 = t[0]
    return [v - t0 for v in t]


def load_gps():
    series = []
    with open(GPS, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            series.append((float(row["time_s"]), float(row["east_m"]), float(row["north_m"])))
    return series


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


def load_vo(cam_t, traj_path=TRAJ):
    vo = []
    with open(traj_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            fr = int(float(r["Frame"])) - 1
            vo.append((cam_t[fr], float(r["X"]), float(r["Y"])))
    return vo


def eval_track(cam_t, traj_path):
    gps = load_gps()
    vo = load_vo(cam_t, traj_path)
    truth = [at(gps, p[0]) for p in vo]
    fitted, s_gt, i1 = align(vo, truth)
    err = [math.hypot(fitted[i][0] - truth[i][1], fitted[i][1] - truth[i][2]) for i in range(len(vo))]
    rms = math.sqrt(sum(e * e for e in err) / len(err))
    path_vo = sum(
        math.hypot(fitted[i][0] - fitted[i - 1][0], fitted[i][1] - fitted[i - 1][1]) for i in range(1, len(fitted))
    )
    step_e = [0.0]
    for i in range(1, len(fitted)):
        dvx = fitted[i][0] - fitted[i - 1][0]
        dvy = fitted[i][1] - fitted[i - 1][1]
        dtx = truth[i][1] - truth[i - 1][1]
        dty = truth[i][2] - truth[i - 1][2]
        step_e.append(math.hypot(dvx - dtx, dvy - dty))
    cum = np.cumsum(step_e)
    t = [p[0] for p in vo]
    return {
        "rms": rms,
        "max": max(err),
        "end": err[-1],
        "path_vo": path_vo,
        "path_gt": s_gt[-1],
        "ratio": path_vo / s_gt[-1],
        "cum_end": float(cum[-1]),
        "step_mean": float(np.mean(step_e[1:])),
        "t": t,
        "err": err,
        "cum": cum,
        "truth": truth,
        "fitted": fitted,
        "i1": i1,
    }


def align(vo, truth):
    s = [0.0]
    for i in range(1, len(truth)):
        s.append(s[-1] + math.hypot(truth[i][1] - truth[i - 1][1], truth[i][2] - truth[i - 1][2]))
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
    ang = math.atan2(cross, dot)
    c, sn = math.cos(ang), math.sin(ang)
    fitted = []
    for i in range(len(vo)):
        x, y = vo[i][1] - ct[0], vo[i][2] - ct[1]
        fitted.append((c * x - sn * y + cg[0], sn * x + c * y + cg[1]))
    return fitted, s, i1


def plot_track_bundle(track, fig_prefix, title_track):
    truth, fitted, t, err, cum = (
        track["truth"],
        track["fitted"],
        track["t"],
        track["err"],
        track["cum"],
    )
    path_gt = track["path_gt"]
    path_vo = track["path_vo"]
    rms = track["rms"]

    # СК: X=север, Z=восток, Y=вверх.
    # На поле: снизу Z (восток), слева X (север).
    # truth: (time, east, north), fitted: (east, north)
    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    ax.plot([p[1] for p in truth], [p[2] for p in truth], color="black", lw=1.6, label=f"GPS, {path_gt:.0f} м")
    ax.plot(
        [p[0] for p in fitted],
        [p[1] for p in fitted],
        color="#1b7f3a",
        lw=1.3,
        label=f"трек, {path_vo:.0f} м",
    )
    ax.scatter([fitted[0][0]], [fitted[0][1]], s=90, c="#d00000", zorder=5, label="старт")
    ax.annotate(
        "старт",
        (fitted[0][0], fitted[0][1]),
        textcoords="offset points",
        xytext=(8, 8),
        color="#d00000",
        fontsize=11,
        fontweight="bold",
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")
    ax.set_title(f"{title_track}, RMS {rms:.1f} м")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout(pad=0.6)
    p_xy = os.path.join(FIG, f"{fig_prefix}_trajectory_xy.png")
    fig.savefig(p_xy, dpi=140, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    ax.plot(t, err, color="#1b7f3a", lw=1.5, label="одометрия")
    for a, b in ((105, 120), (140, 155), (185, 200), (232, 247), (275, 292)):
        ax.axvspan(a, b, color="tab:blue", alpha=0.10, zorder=0)
    ymin, ymax = ax.get_ylim()
    for moment, label in ((60, "конец подъёма"), (95, "начало маршрута"), (310, "начало снижения")):
        ax.axvline(moment, color="gray", ls="--", lw=1, zorder=1)
        ax.text(moment + 3, ymax * 0.95, label, rotation=90, fontsize=8, color="gray", va="top")
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("время, с")
    ax.set_ylabel("ошибка, м")
    ax.set_title("Ошибка положения во времени (синим - развороты)")
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=9, loc="upper right")
    fig.tight_layout(pad=0.8)
    p_abs = os.path.join(FIG, f"{fig_prefix}_abs_error.png")
    fig.savefig(p_abs, dpi=140, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.0, 5.2))
    ax.plot(t, cum, color="#b00020", lw=2.0)
    ax.set_xlabel("время полёта, с")
    ax.set_ylabel("накопленная ошибка, м")
    ax.set_title("Накопленная ошибка за время полёта")
    ax.grid(True, alpha=0.35)
    fig.tight_layout(pad=0.8)
    p_cum = os.path.join(FIG, f"{fig_prefix}_cumulative.png")
    fig.savefig(p_cum, dpi=140, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    return p_xy, p_abs, p_cum


def main():
    cam_t = load_cam()
    vo_track = eval_track(cam_t, TRAJ)
    p_xy, p_abs, p_cum = plot_track_bundle(vo_track, "vo", "Траектория одометрии")

    # stills from video
    import cv2

    src = os.path.join(ROOT, r"report\tracking_takeoff.mp4")
    cap = cv2.VideoCapture(src)
    nfr = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 1)
    stills = []
    for k, frac in enumerate((0.08, 0.35, 0.62, 0.88)):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(frac * max(nfr - 1, 0)))
        ok, frame = cap.read()
        if not ok:
            continue
        path = os.path.join(FIG, f"still_{k}.png")
        cv2.imwrite(path, frame)
        stills.append(path)
    cap.release()

    metrics = {
        "rms": vo_track["rms"],
        "max": vo_track["max"],
        "end": vo_track["end"],
        "path_vo": vo_track["path_vo"],
        "path_gt": vo_track["path_gt"],
        "ratio": vo_track["ratio"],
        "cum_end": vo_track["cum_end"],
        "step_mean": vo_track["step_mean"],
        "n": len(vo_track["t"]),
        "i1": vo_track["i1"],
    }
    write_pdf(metrics, p_xy, p_abs, p_cum, stills)
    print(
        "VO RMS %.2f max %.2f end %.2f path %.1f / %.1f"
        % (metrics["rms"], metrics["max"], metrics["end"], metrics["path_vo"], metrics["path_gt"])
    )
    print("PDF", OUT_PDF)


def write_pdf(m, p_xy, p_abs, p_cum, stills):
    pdf = FPDF(format="A4", unit="mm")
    pdf.set_auto_page_break(auto=True, margin=16)
    pdf.add_font("Arial", "", r"C:\Windows\Fonts\arial.ttf")
    pdf.add_font("Arial", "B", r"C:\Windows\Fonts\arialbd.ttf")

    def h1(text):
        pdf.set_x(pdf.l_margin)
        pdf.set_font("Arial", "B", 16)
        pdf.multi_cell(0, 8, text)

    def h2(text):
        pdf.set_x(pdf.l_margin)
        pdf.set_font("Arial", "B", 13)
        pdf.multi_cell(0, 7, text)

    def body(text):
        pdf.set_x(pdf.l_margin)
        pdf.set_font("Arial", "", 11)
        pdf.multi_cell(0, 5.6, text)

    def figure(path, width=180):
        pdf.set_x(pdf.l_margin)
        pdf.image(path, w=width)
        pdf.ln(2)

    # --- 1. суть и цифры ---
    pdf.add_page()
    h1("Визуальная одометрия HKairport03")
    body("Камера смотрит вниз. Позиция считается по картинке, GPS в координаты не подставляется.")
    pdf.ln(2)
    h2("Метод")
    body(
        "Кадр 2448×2048, 10 кадров в секунду, фокус 1449 пикселей, 3657 кадров. "
        "Высота берётся из барометра. На развороте шаг идёт по дуге, а не срезает угол прямой. "
        "Для сравнения с GPS трек один раз поворачивается и сдвигается по началу полёта, "
        "до первого разворота (до 105 с). Масштаб не подгоняется."
    )
    pdf.ln(2)
    h2("Метрики")
    rows = [
        ("RMS отклонения от GPS", f"{m['rms']:.1f} м"),
        ("Максимум / конец", f"{m['max']:.1f} м / {m['end']:.1f} м"),
        ("Длина пути / GPS", f"{m['path_vo']:.0f} м / {m['path_gt']:.0f} м ({m['ratio']:.2f}×)"),
        ("Средняя ошибка шага", f"{m['step_mean']:.2f} м"),
        ("Сумма ошибок шагов", f"{m['cum_end']:.0f} м"),
        ("Время на i7-8750H", "118 с, 668 шагов, 5.65 шаг/с"),
        ("Длительность полёта", "366 с"),
    ]
    for a, b in rows:
        pdf.set_x(pdf.l_margin)
        pdf.set_font("Arial", "", 11)
        pdf.cell(110, 8, "  " + a, border=1)
        pdf.set_font("Arial", "B", 11)
        pdf.cell(70, 8, b, border=1, new_x="LMARGIN", new_y="NEXT")
    pdf.ln(3)
    body("Дальше траектория, два графика ошибки и кадры, на которых видно, как берутся точки.")

    # --- 2. траектория ---
    pdf.add_page()
    h1("Траектория")
    body("Чёрная линия — GPS, зелёная — одометрия. Красная точка — старт.")
    pdf.ln(2)
    figure(p_xy, 175)

    # --- 3. абсолютное отклонение ---
    pdf.add_page()
    h1("График ошибки 1")
    h2("Абсолютное отклонение от GPS")
    body(
        "На каждом шаге — расстояние от точки одометрии до GPS в этот же момент. "
        "На замкнутом маршруте линия то растёт, то падает: обратный участок снова проходит "
        "рядом с GPS, и расстояние уменьшается. Это не сброс расчёта."
    )
    pdf.ln(2)
    figure(p_abs, 180)

    # --- 4. накопленная ошибка ---
    pdf.add_page()
    h1("График ошибки 2")
    h2("Накопленная ошибка за время полёта")
    body(
        "Ошибка каждого шага только прибавляется к сумме. За время полёта кривая растёт "
        "и вниз не возвращается. На прямой рост медленнее, на развороте быстрее."
    )
    pdf.ln(2)
    figure(p_cum, 180)

    # --- 5. время ---
    pdf.add_page()
    h1("Время и Jetson")
    body(
        "Полный прогон на Intel Core i7-8750H (6 ядер, 2.2 ГГц): 668 шагов за 118 с, 5.65 шаг/с. "
        "Полёт длится 366 с, то есть на этом ноутбуке расчёт быстрее реального времени примерно в 3.1 раза. "
        "Полный кадр 2448×2048 считается не каждый: обычно берётся каждый третий."
    )
    pdf.ln(2)
    body(
        "На Jetson Orin Nano или Orin NX тот же режим, без записи картинок соответствий, успевает за 10 кадров "
        "в секунду. Считает процессор, видеокарта не используется. Старый Jetson Nano на полном кадре не успевает: "
        "кадр надо уменьшать примерно до 960 пикселей по ширине либо ставить Orin."
    )
    pdf.ln(2)
    body(
        "Ролик — первые 1 мин 20 с полёта: взлёт и начало маршрута. "
        "Зелёные кружки — найденные точки, оранжевые отрезки — одни и те же точки на соседних кадрах."
    )

    # --- 6. кадры ---
    pdf.add_page()
    h1("Как берутся точки")
    body("Четыре кадра из ролика: начало, набор высоты и уже идущий маршрут.")
    pdf.ln(2)
    if len(stills) >= 4:
        x0 = pdf.l_margin
        y0 = pdf.get_y()
        w, h = 88, 66
        pdf.image(stills[0], x=x0, y=y0, w=w)
        pdf.image(stills[1], x=x0 + w + 6, y=y0, w=w)
        pdf.image(stills[2], x=x0, y=y0 + h + 4, w=w)
        pdf.image(stills[3], x=x0 + w + 6, y=y0 + h + 4, w=w)
        pdf.set_y(y0 + 2 * h + 10)
    if os.path.exists(VIDEO):
        pdf.embed_file(VIDEO, desc="tracking, 80s")

    pdf.output(OUT_PDF)


if __name__ == "__main__":
    main()
