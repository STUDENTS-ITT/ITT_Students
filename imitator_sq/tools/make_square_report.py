"""Отчёт Word: пролёт по квадрату, Калман и СНС."""

from __future__ import annotations

import sys
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt

R_EARTH = 6371000.0


def load(path: Path) -> tuple[dict[str, int], np.ndarray]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    header = lines[0].split()[1:]
    rows = []
    for line in lines[1:]:
        parts = line.split()
        if not parts or parts[0].count(":") != 2:
            continue
        try:
            rows.append([float(x) for x in parts[1:]])
        except ValueError:
            continue
    return {name: i for i, name in enumerate(header)}, np.asarray(rows, dtype=float)


def gps_updates(idx: dict[str, int], data: np.ndarray) -> np.ndarray:
    block = data[:, idx["lon_sns"] : idx["ve_sns"] + 1]
    changed = np.ones(len(data), dtype=bool)
    if len(data) > 1:
        changed[1:] = np.any(np.abs(np.diff(block, axis=0)) > 1e-12, axis=1)
    return data[changed]


def wrap_deg(angle: np.ndarray) -> np.ndarray:
    return (angle + 180.0) % 360.0 - 180.0


def rmse(v: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(v))))


def errors(idx: dict[str, int], data: np.ndarray) -> dict[str, np.ndarray]:
    dlat = np.deg2rad(data[:, idx["lat"]] - data[:, idx["lat_sns"]])
    dlon = np.deg2rad(data[:, idx["lon"]] - data[:, idx["lon_sns"]])
    north = dlat * R_EARTH
    east = dlon * R_EARTH * np.cos(np.deg2rad(data[:, idx["lat_sns"]]))
    return {
        "north_m": north,
        "east_m": east,
        "horiz_m": np.hypot(north, east),
        "alt": data[:, idx["alt"]] - data[:, idx["alt_sns"]],
        "vn": data[:, idx["vn"]] - data[:, idx["vn_sns"]],
        "vh": data[:, idx["vh"]] - data[:, idx["vh_sns"]],
        "ve": data[:, idx["ve"]] - data[:, idx["ve_sns"]],
        "hdg": wrap_deg(data[:, idx["heading"]] - data[:, idx["heading_sns"]]),
        "pitch": wrap_deg(data[:, idx["pitch"]] - data[:, idx["pitch_sns"]]),
        "roll": wrap_deg(data[:, idx["roll"]] - data[:, idx["roll_sns"]]),
    }


def stats_rows(err: dict[str, np.ndarray]) -> list[tuple[str, str, str, str]]:
    items = [
        ("Север, м", err["north_m"]),
        ("Восток, м", err["east_m"]),
        ("Горизонт, м", err["horiz_m"]),
        ("Высота, м", err["alt"]),
        ("Vn, м/с", err["vn"]),
        ("Vh, м/с", err["vh"]),
        ("Ve, м/с", err["ve"]),
        ("Курс, °", err["hdg"]),
        ("Тангаж, °", err["pitch"]),
        ("Крен, °", err["roll"]),
    ]
    out = []
    for name, v in items:
        out.append((name, f"{rmse(v):.3f}", f"{float(np.mean(v)):.3f}", f"{float(np.max(np.abs(v))):.3f}"))
    return out


def set_run_font(run, size=12, bold=False):
    run.font.name = "Times New Roman"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "Times New Roman")
    run.font.size = Pt(size)
    run.bold = bold


def add_heading(doc, text, level=1):
    p = doc.add_heading(text, level=level)
    for run in p.runs:
        set_run_font(run, size=16 if level == 1 else 14, bold=True)


def add_para(doc, text):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.space_after = Pt(8)
    pf.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
    pf.first_line_indent = Cm(1.25)
    run = p.add_run(text)
    set_run_font(run)
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY


def shade_header(cell):
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "D9E2F3")
    shd.set(qn("w:val"), "clear")
    tcPr.append(shd)


def add_table(doc, headers, rows):
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.style = "Table Grid"
    for i, h in enumerate(headers):
        cell = table.rows[0].cells[i]
        cell.text = ""
        run = cell.paragraphs[0].add_run(h)
        set_run_font(run, size=11, bold=True)
        cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
        shade_header(cell)
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = table.rows[ri + 1].cells[ci]
            cell.text = ""
            run = cell.paragraphs[0].add_run(str(val))
            set_run_font(run, size=11)
            cell.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.LEFT if ci == 0 else WD_ALIGN_PARAGRAPH.CENTER
    doc.add_paragraph()


def add_picture(doc, path: Path, caption: str):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.first_line_indent = Cm(0)
    p.add_run().add_picture(str(path), width=Cm(16.0))
    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.first_line_indent = Cm(0)
    run = cap.add_run(caption)
    set_run_font(run, size=11)
    run.italic = True


def plot_map(idx, gps, path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(gps[:, idx["lon_sns"]], gps[:, idx["lat_sns"]], label="СНС", color="#d95319", linewidth=1.6)
    ax.plot(gps[:, idx["lon"]], gps[:, idx["lat"]], label="Калман", color="#0072bd", linewidth=1.2)
    ax.set_xlabel("Долгота, °")
    ax.set_ylabel("Широта, °")
    ax.set_title("Квадрат в плане")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_comparison(idx, gps, path: Path) -> None:
    t = gps[:, idx["time"]]
    fig, axes = plt.subplots(3, 3, figsize=(16, 11), sharex=True)
    fig.suptitle("Калман и СНС на квадрате", fontsize=14)
    panels = [
        (axes[0, 0], "lon", "lon_sns", "Долгота, °"),
        (axes[0, 1], "lat", "lat_sns", "Широта, °"),
        (axes[0, 2], "alt", "alt_sns", "Высота, м"),
        (axes[1, 0], "vn", "vn_sns", "Vn, м/с"),
        (axes[1, 1], "vh", "vh_sns", "Vh, м/с"),
        (axes[1, 2], "ve", "ve_sns", "Ve, м/с"),
        (axes[2, 0], "heading", "heading_sns", "Курс, °"),
        (axes[2, 1], "pitch", "pitch_sns", "Тангаж, °"),
        (axes[2, 2], "roll", "roll_sns", "Крен, °"),
    ]
    for ax, a, b, ylab in panels:
        ax.plot(t, gps[:, idx[a]], label="Калман", color="#0072bd", linewidth=1.15)
        ax.plot(t, gps[:, idx[b]], label="СНС", color="#d95319", linewidth=1.0, alpha=0.9)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    for ax in axes[2]:
        ax.set_xlabel("Время, с")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_errors(t, err, path: Path) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(16, 11), sharex=True)
    fig.suptitle("Ошибка Калман − СНС на квадрате", fontsize=14)
    panels = [
        (axes[0, 0], err["north_m"], "Δ север, м"),
        (axes[0, 1], err["east_m"], "Δ восток, м"),
        (axes[0, 2], err["alt"], "Δ высота, м"),
        (axes[1, 0], err["vn"], "Δ Vn, м/с"),
        (axes[1, 1], err["vh"], "Δ Vh, м/с"),
        (axes[1, 2], err["ve"], "Δ Ve, м/с"),
        (axes[2, 0], err["hdg"], "Δ курс, °"),
        (axes[2, 1], err["pitch"], "Δ тангаж, °"),
        (axes[2, 2], err["roll"], "Δ крен, °"),
    ]
    for ax, y, ylab in panels:
        ax.plot(t, y, color="#c0392b", linewidth=1.0)
        ax.axhline(0.0, color="0.45", linewidth=0.8)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    for ax in axes[2]:
        ax.set_xlabel("Время, с")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def square_legs(idx, gps) -> list[tuple[str, float, float, float]]:
    t = gps[:, idx["time"]]
    vn = gps[:, idx["vn_sns"]]
    ve = gps[:, idx["ve_sns"]]
    spd = np.hypot(vn, ve)
    moving = spd > 2.0
    names_cycle = ("Север", "Восток", "Юг", "Запад")
    legs = []
    i = 0
    while i < len(moving):
        if not moving[i]:
            i += 1
            continue
        j = i
        while j < len(moving) and moving[j]:
            j += 1
        if t[j - 1] - t[i] > 5.0:
            dlat = np.deg2rad(gps[j - 1, idx["lat_sns"]] - gps[i, idx["lat_sns"]])
            dlon = np.deg2rad(gps[j - 1, idx["lon_sns"]] - gps[i, idx["lon_sns"]])
            dn = dlat * R_EARTH
            de = dlon * R_EARTH * np.cos(np.deg2rad(gps[i, idx["lat_sns"]]))
            legs.append((names_cycle[len(legs) % 4], float(t[i]), float(t[j - 1]), float(np.hypot(dn, de))))
        i = j
    return legs


def main() -> int:
    root = Path(__file__).resolve().parent
    idx, data = load(root / "kalman15_line2.txt")
    gps_all = gps_updates(idx, data)
    legs = square_legs(idx, gps_all)
    if len(legs) < 4:
        raise SystemExit(f"Ожидались 4 стороны квадрата, найдено {len(legs)}")
    t0, t1 = legs[0][1], legs[3][2]
    gps = gps_all[(gps_all[:, idx["time"]] >= t0) & (gps_all[:, idx["time"]] <= t1)]
    err = errors(idx, gps)
    rows = stats_rows(err)

    leg_rows = []
    for name, a, b, length in legs[:4]:
        part = gps[(gps[:, idx["time"]] >= a) & (gps[:, idx["time"]] <= b)]
        e = errors(idx, part)
        leg_rows.append(
            (
                name,
                f"{a:.0f}–{b:.0f}",
                f"{length:.0f}",
                f"{rmse(e['horiz_m']):.2f}",
                f"{rmse(e['hdg']):.2f}",
                f"{rmse(e['vn']):.2f}",
                f"{rmse(e['ve']):.2f}",
            )
        )

    fig_dir = root / "report_square_figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_map(idx, gps, fig_dir / "square_map.png")
    plot_comparison(idx, gps, fig_dir / "square_comparison.png")
    plot_errors(gps[:, idx["time"]], err, fig_dir / "square_errors.png")

    side = float(np.mean([length for _, _, _, length in legs[:4]]))
    speed = float(np.mean(np.hypot(gps[:, idx["vn_sns"]], gps[:, idx["ve_sns"]])))

    doc = Document()
    add_heading(doc, "Отчёт о пролёте по квадрату")
    add_para(
        doc,
        f"Источник: kalman15_line2.txt. Квадрат выделен по участкам, где горизонтальная скорость СНС "
        f"больше 2 м/с. Четыре стороны идут подряд: {t0:.0f}–{t1:.0f} с, точек СНС {len(gps)}. "
        f"Длина стороны около {side:.0f} м, средняя скорость на этих участках {speed:.2f} м/с. "
        f"Ошибка ниже — решение БИНС с фильтром Калмана минус СНС. "
        f"Курс эталона записан как yaw + 16°. Если тангаж и крен эталона оба больше 90°, "
        f"они сворачиваются в ±90°, а курс сдвигается на 180° только когда так он ближе к направлению скорости.",
    )

    add_heading(doc, "1. Стороны квадрата", 2)
    add_table(
        doc,
        ("Сторона", "Время, с", "Длина, м", "RMSE горизонт, м", "RMSE курс, °", "RMSE Vn, м/с", "RMSE Ve, м/с"),
        leg_rows,
    )

    add_heading(doc, "2. Ошибки на всём квадрате", 2)
    add_table(doc, ("Величина", "RMSE", "Среднее", "max |e|"), rows)
    leg_text = ", ".join(
        f"{name.lower()} — курс {hdg}°, горизонт {horiz} м" for name, _, _, horiz, hdg, _, _ in leg_rows
    )
    add_para(
        doc,
        f"По сторонам RMSE такие: {leg_text}. "
        f"Наибольшие отклонения курса приходятся на развороты между сторонами, "
        f"где замер курса отключён и угол короткое время ведёт гироскоп. "
        f"На прямых участках курс, тангаж и крен остаются рядом с эталоном.",
    )

    add_heading(doc, "3. Траектория", 2)
    add_picture(doc, fig_dir / "square_map.png", "Рис. 1 — квадрат в плане, Калман и СНС")

    add_heading(doc, "4. Сравнение со СНС", 2)
    add_picture(doc, fig_dir / "square_comparison.png", "Рис. 2 — координаты, скорости и углы")

    add_heading(doc, "5. Графики ошибок", 2)
    add_picture(
        doc,
        fig_dir / "square_errors.png",
        "Рис. 3 — ошибка Калман − СНС",
    )

    horiz = next(r for r in rows if r[0].startswith("Горизонт"))
    hdg = next(r for r in rows if r[0].startswith("Курс"))
    pitch = next(r for r in rows if r[0].startswith("Тангаж"))
    roll = next(r for r in rows if r[0].startswith("Крен"))
    add_heading(doc, "6. Кратко", 2)
    add_para(
        doc,
        f"Контур квадрата повторяется, сторона около {side:.0f} м. "
        f"Горизонтальная ошибка Калмана на всём облёте: RMSE {horiz[1]} м, максимум {horiz[3]} м. "
        f"Курс: RMSE {hdg[1]}°, максимум {hdg[3]}°. "
        f"Тангаж: RMSE {pitch[1]}°, крен: RMSE {roll[1]}°.",
    )

    out = root / "Отчет_пролет_по_квадрату.docx"
    doc.save(out)
    print(f"квадрат {t0:.1f}–{t1:.1f} с, сторона ~{side:.0f} м, точек {len(gps)}")
    for row in leg_rows:
        print(" ", row)
    for row in rows:
        print(f"{row[0]:<14} RMSE={row[1]:>8} mean={row[2]:>8} max={row[3]:>8}")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
