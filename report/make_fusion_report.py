# -*- coding: utf-8 -*-
"""PDF report: VO + INS + Kalman fusion comparison (HKairport03)."""
from __future__ import annotations

import json
import re
from pathlib import Path

from fpdf import FPDF
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
FUSION_DIR = ROOT / "results_fusion"
FIG_DIR = ROOT / "report" / "figures"
OUT_PDF = ROOT / "report" / "VO_INS_Fusion_HKairport03.pdf"

VO_CONSOLE = (
    ROOT
    / "results_HKairport03"
    / "homography"
    / "visual_odometry"
    / "FAST-FREAK"
    / "console_log.txt"
)

PAGE_MARGIN_MM = 15.0
FONT_REG = r"C:\Windows\Fonts\arial.ttf"
FONT_BOLD = r"C:\Windows\Fonts\arialbd.ttf"


def require_inputs() -> tuple[Path, Path, Path, Path]:
    traj = FUSION_DIR / "trajectory_comparison.png"
    fusion = FUSION_DIR / "fusion_comparison.png"
    metrics = FUSION_DIR / "metrics_summary.json"
    missing = [p for p in (traj, fusion, metrics) if not p.is_file()]
    if missing:
        lines = "\n".join(f"  • {p}" for p in missing)
        raise SystemExit(
            "Не найдены входные файлы слияния. Сначала выполните:\n"
            "  python tools/plot_fusion_results.py\n"
            "или полный конвейер:\n"
            "  python tools/run_fusion_pipeline.py\n\n"
            f"Отсутствуют:\n{lines}"
        )
    return traj, fusion, metrics


def split_fusion_panels(fusion_png: Path, out_dir: Path) -> tuple[Path, Path]:
    """Upper / lower subplot from fusion_comparison.png (separate PDF pages)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    top_path = out_dir / "fusion_panel_abs.png"
    bot_path = out_dir / "fusion_panel_cum.png"
    with Image.open(fusion_png) as im:
        w, h = im.size
        mid = h // 2
        im.crop((0, 0, w, mid)).save(top_path)
        im.crop((0, mid, w, h)).save(bot_path)
    return top_path, bot_path


def load_metrics(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def parse_vo_timing(console_path: Path) -> dict | None:
    if not console_path.is_file():
        return None
    text = console_path.read_text(encoding="utf-8", errors="replace")
    m = re.search(
        r"Обработано шагов:\s*(\d+)\s*за\s*([\d.]+)\s*с\s*\(([\d.]+)\s*шаг/с\)",
        text,
    )
    if not m:
        return None
    steps = int(m.group(1))
    sec = float(m.group(2))
    rate = float(m.group(3))
    return {"steps": steps, "sec": sec, "steps_per_sec": rate, "source": str(console_path)}


def load_timing() -> dict:
    """VO from console_log; INS from results_fusion/timing.json if present."""
    timing_path = FUSION_DIR / "timing.json"
    data: dict = {}
    if timing_path.is_file():
        with open(timing_path, encoding="utf-8") as f:
            data.update(json.load(f))

    vo = parse_vo_timing(VO_CONSOLE)
    if vo:
        data.setdefault("vo_sec", vo["sec"])
        data.setdefault("vo_steps", vo["steps"])
        data.setdefault("vo_steps_per_sec", vo["steps_per_sec"])

    return data


def format_timing_row(label: str, sec: float | None, extra: str = "") -> str:
    if sec is None:
        return f"{label}: нет данных"
    tail = f", {extra}" if extra else ""
    return f"{label}: {sec:.1f} с{tail}"


def conclusions_text(metrics: dict, timing: dict) -> str:
    vo = metrics["vo_only"]
    fused = metrics["vo_ins_kalman"]
    imp = metrics.get("improvement_pct", {})

    parts = [
        "Сравнение выполнено относительно RTK в локальной плоскости (восток–север). "
        "Траектория «только ВО» строится по камере; «ВО+БИНС+Калман» дополняет её инерциальными "
        "измерениями и периодической коррекцией по псевдо-GPS от одометрии.",
        "",
        f"RMS отклонения от RTK снизилась с {vo['rms']:.2f} м до {fused['rms']:.2f} м "
        f"({imp.get('rms', 0):+.1f} %). Максимальная ошибка: {vo['max']:.2f} → {fused['max']:.2f} м "
        f"({imp.get('max', 0):+.1f} %). На финише маршрута: {vo['end']:.2f} → {fused['end']:.2f} м "
        f"({imp.get('end', 0):+.1f} %).",
    ]

    rms_imp = imp.get("rms", 0)
    if rms_imp > 5:
        parts.append(
            "Комплексирование заметно стабилизирует трек: пики на разворотах и участках "
            "слабой текстуры камеры сглаживаются за счёт инерциальной модели."
        )
    elif rms_imp > 0:
        parts.append(
            "Улучшение умеренное: основной выигрыш ожидается на длинных участках и при "
            "кратковременных провалах сопоставления признаков ВО."
        )
    else:
        parts.append(
            "По метрикам относительно RTK выигрыш не выражен — проверьте синхронизацию времени, "
            "масштаб ВО и параметры фильтра Калмана."
        )

    vo_sec = timing.get("vo_sec")
    ins_sec = timing.get("ins_sec")
    if vo_sec is not None and ins_sec is not None:
        ratio = vo_sec / ins_sec if ins_sec > 0 else 0.0
        parts.append(
            f"По времени расчёта ВО заняла {vo_sec:.1f} с, прогон БИНС+Калман — {ins_sec:.1f} с "
            f"(ВО примерно в {ratio:.1f} раз дольше). Инерциальный контур отрабатывает быстрее "
            "полного цикла обработки кадров; узкое место остаётся на стороне видеоодометрии."
        )
    elif vo_sec is not None:
        parts.append(
            f"Видеоодометрия на этом прогоне заняла {vo_sec:.1f} с. Время БИНС сохраните в "
            f"{FUSION_DIR / 'timing.json'} (поле ins_sec) после запуска imitator."
        )

    return "\n".join(parts)


class FusionReportPDF(FPDF):
    def __init__(self) -> None:
        super().__init__(format="A4", unit="mm")
        self.set_auto_page_break(auto=True, margin=PAGE_MARGIN_MM)
        self.set_margins(PAGE_MARGIN_MM, PAGE_MARGIN_MM, PAGE_MARGIN_MM)
        self.add_font("Arial", "", FONT_REG)
        self.add_font("Arial", "B", FONT_BOLD)

    def h1(self, text: str) -> None:
        self.set_x(self.l_margin)
        self.set_font("Arial", "B", 16)
        self.multi_cell(0, 8, text)

    def h2(self, text: str) -> None:
        self.set_x(self.l_margin)
        self.set_font("Arial", "B", 13)
        self.multi_cell(0, 7, text)

    def body(self, text: str) -> None:
        self.set_x(self.l_margin)
        self.set_font("Arial", "", 11)
        self.multi_cell(0, 5.6, text)

    def figure_fit(self, path: Path, caption: str | None = None) -> None:
        if caption:
            self.body(caption)
            self.ln(2)
        avail_w = self.w - self.l_margin - self.r_margin
        y0 = self.get_y()
        avail_h = self.h - y0 - PAGE_MARGIN_MM
        with Image.open(path) as im:
            iw, ih = im.size
        aspect = ih / iw
        w = avail_w
        h = w * aspect
        if h > avail_h:
            h = avail_h
            w = h / aspect
        x = self.l_margin + (avail_w - w) / 2
        self.image(str(path), x=x, y=y0, w=w, h=h)
        self.set_y(y0 + h + 2)


def write_pdf(
    traj_png: Path,
    abs_png: Path,
    cum_png: Path,
    metrics: dict,
    timing: dict,
) -> None:
    vo = metrics["vo_only"]
    fused = metrics["vo_ins_kalman"]
    imp = metrics.get("improvement_pct", {})

    pdf = FusionReportPDF()

    # --- 1. Title ---
    pdf.add_page()
    pdf.ln(55)
    pdf.set_font("Arial", "B", 20)
    pdf.multi_cell(
        0,
        10,
        "Комплексирование ВО + БИНС + Калман (HKairport03)",
        align="C",
    )
    pdf.ln(8)
    pdf.set_font("Arial", "", 12)
    pdf.multi_cell(
        0,
        6,
        "Сравнение траекторий и ошибок относительно RTK.\n"
        "Данные: results_fusion/, видеоодометрия FAST-FREAK + homography.",
        align="C",
    )

    # --- 2. Trajectories ---
    pdf.add_page()
    pdf.h1("Траектории")
    pdf.figure_fit(
        traj_png,
        "RTK и VO+INS+Kalman в одной локальной СК.",
    )

    # --- 3. Absolute error (top panel) ---
    pdf.add_page()
    pdf.h1("Абсолютное отклонение")
    pdf.h2("Ошибка положения во времени")
    pdf.figure_fit(
        abs_png,
        "Синие полосы — развороты. Пунктир: конец подъёма, начало маршрута, начало снижения.",
    )

    # --- 4. Cumulative step error (bottom panel) ---
    pdf.add_page()
    pdf.h1("Накопленная ошибка шагов")
    pdf.h2("Монотонная сумма ошибок приращений")
    pdf.figure_fit(
        cum_png,
        "Нижний график из fusion_comparison.png: накопленная ошибка шагов.",
    )

    # --- 5. Metrics table ---
    pdf.add_page()
    pdf.h1("Метрики качества")
    pdf.ln(2)
    headers = ("Метрика", "Только ВО", "ВО+ИНС+Калман", "Улучшение")
    rows = [
        ("RMS отклонения от RTK, м", vo["rms"], fused["rms"], imp.get("rms")),
        ("Максимум, м", vo["max"], fused["max"], imp.get("max")),
        ("Ошибка на финише, м", vo["end"], fused["end"], imp.get("end")),
    ]
    col_w = (70, 38, 38, 34)
    pdf.set_font("Arial", "B", 10)
    for i, h in enumerate(headers):
        pdf.cell(col_w[i], 8, h, border=1, align="C")
    pdf.ln()
    pdf.set_font("Arial", "", 10)
    for label, a, b, pct in rows:
        pdf.cell(col_w[0], 8, label, border=1)
        pdf.cell(col_w[1], 8, f"{a:.2f}", border=1, align="C")
        pdf.cell(col_w[2], 8, f"{b:.2f}", border=1, align="C")
        imp_s = f"{pct:+.1f} %" if pct is not None else "—"
        pdf.cell(col_w[3], 8, imp_s, border=1, align="C")
        pdf.ln()
    pdf.ln(4)
    pdf.body(
        "Улучшение в процентах: (метрика ВО − метрика слияния) / метрика ВО × 100. "
        "Положительное значение означает снижение ошибки."
    )

    # --- 6. Runtime ---
    pdf.add_page()
    pdf.h1("Время выполнения")
    vo_sec = timing.get("vo_sec")
    ins_sec = timing.get("ins_sec")
    vo_steps = timing.get("vo_steps")
    pdf.body(format_timing_row("Видеоодометрия (ВО)", vo_sec, f"{vo_steps} шагов" if vo_steps else ""))
    pdf.ln(1)
    pdf.body(format_timing_row("БИНС + фильтр Калмана", ins_sec))
    pdf.ln(1)
    if vo_sec is not None and ins_sec is not None and ins_sec > 0:
        pdf.body(
            f"Суммарно оба этапа: {vo_sec + ins_sec:.1f} с. "
            f"Доля ВО в общем времени: {100.0 * vo_sec / (vo_sec + ins_sec):.0f} %."
        )
        pdf.ln(2)
        pdf.body(
            "ВО обрабатывает изображения и сопоставляет признаки; БИНС интегрирует IMU "
            "на частоте сотен Гц и выполняет коррекции — обычно это на порядки быстрее одного "
            "полного прогона одометрии по всем кадрам."
        )
    else:
        pdf.ln(2)
        pdf.body(
            "Чтобы добавить время БИНС в отчёт, сохраните JSON "
            f'{{"ins_sec": <секунды>}} в {FUSION_DIR / "timing.json"} '
            "(imitator печатает длительность в консоль в конце прогона)."
        )

    # --- 7. Conclusions ---
    pdf.add_page()
    pdf.h1("Выводы")
    pdf.body(conclusions_text(metrics, timing))

    OUT_PDF.parent.mkdir(parents=True, exist_ok=True)
    pdf.output(str(OUT_PDF))


def main() -> None:
    traj_png, fusion_png, metrics_path = require_inputs()
    metrics = load_metrics(metrics_path)
    timing = load_timing()

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    abs_png, cum_png = split_fusion_panels(fusion_png, FIG_DIR)

    write_pdf(traj_png, abs_png, cum_png, metrics, timing)
    print(f"PDF: {OUT_PDF}")


if __name__ == "__main__":
    main()
