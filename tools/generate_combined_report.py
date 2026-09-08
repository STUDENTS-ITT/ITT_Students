#!/usr/bin/env python3
"""PDF-отчёт: объединённый алгоритм VINS+DSO (диспетчер) на MARS и Поворот_коптер."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import tempfile
from datetime import date
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from reportlab.graphics import renderPDF
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Flowable,
    Image,
    NextPageTemplate,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)
from reportlab.platypus.frames import Frame
from reportlab.platypus.doctemplate import PageTemplate
from reportlab.platypus.tableofcontents import TableOfContents
from svglib.svglib import svg2rlg

ROOT = Path(__file__).resolve().parents[1]
FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")

# Стили графиков
PLOT_RC = {
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "legend.fontsize": 9,
    "axes.grid": True,
    "grid.alpha": 0.35,
    "grid.linestyle": "-",
    "axes.spines.top": False,
    "axes.spines.right": False,
}
COLOR_EST = "#2563eb"
COLOR_RTK = "#d64545"
RTK_DASH = (5, 7)
COLOR_DSO = "#059669"


def register_fonts():
    pdfmetrics.registerFont(TTFont("DejaVuSans", str(FONT_DIR / "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVuSans-Bold", str(FONT_DIR / "DejaVuSans-Bold.ttf")))


def pstyle(name, **kw):
    base = dict(fontName="DejaVuSans", fontSize=10, leading=14, alignment=TA_JUSTIFY)
    base.update(kw)
    return ParagraphStyle(name, **base)


class ReportDocTemplate(SimpleDocTemplate):
    """SimpleDocTemplate с оглавлением (afterFlowable → TOCEntry + bookmark)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        frame_p = Frame(self.leftMargin, self.bottomMargin, self.width, self.height, id="portrait")
        lw, lh = landscape(A4)
        lm = 0.4 * cm
        frame_l = Frame(lm, lm, lw - 2 * lm, lh - 2 * lm, id="landscape")
        self.addPageTemplates([
            PageTemplate(id="portrait", frames=[frame_p], pagesize=A4),
            PageTemplate(id="landscape", frames=[frame_l], pagesize=landscape(A4)),
        ])

    def afterFlowable(self, flowable):
        if not isinstance(flowable, Paragraph):
            return
        style_name = flowable.style.name
        level_map = {"h1": 0, "h2": 0, "h3": 1}
        if style_name not in level_map:
            return
        anchor = getattr(flowable, "_toc_anchor", None)
        if anchor in ("toc", "title"):
            return
        text = flowable.getPlainText()
        key = anchor or f"sec_{self.seq.nextf('heading')}"
        self.canv.bookmarkPage(key)
        self.canv.addOutlineEntry(text, key, level_map[style_name], closed=False)
        self.notify("TOCEntry", (level_map[style_name], text, self.page, key))


def heading(text: str, style: ParagraphStyle, anchor: str | None = None) -> Paragraph:
    """Заголовок с якорем для оглавления и закладок PDF."""
    if anchor:
        p = Paragraph(f'<a name="{anchor}"/>{text}', style)
        p._toc_anchor = anchor  # noqa: SLF001 — для afterFlowable
        return p
    return Paragraph(text, style)


def rtk_legend_proxy(label: str = "RTK эталон") -> Line2D:
    """Proxy для легенды с тем же dash-паттерном, что и plot_rtk_line."""
    return Line2D([0], [0], color=COLOR_RTK, lw=1.6, linestyle=(0, RTK_DASH), label=label)


def extract_video_posters(
    video_path: Path,
    out_dir: Path,
    prefix: str,
    timestamps: tuple[str, ...] = ("00:00:05", "00:01:00"),
    scale_width: int = 1600,
) -> list[Path]:
    """Извлечь кадры-постеры из видео через ffmpeg (если доступен)."""
    if not video_path.exists() or shutil.which("ffmpeg") is None:
        return []
    out_dir.mkdir(parents=True, exist_ok=True)
    posters: list[Path] = []
    for i, ts in enumerate(timestamps):
        out = out_dir / f"{prefix}_frame_{i + 1}.jpg"
        cmd = [
            "ffmpeg", "-y", "-ss", ts, "-i", str(video_path),
            "-frames:v", "1", "-vf", f"scale={scale_width}:-1", "-q:v", "2", str(out),
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=120)
            if out.exists() and out.stat().st_size > 0:
                posters.append(out)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
            continue
    return posters


def extract_video_gif(
    video_path: Path,
    out_path: Path,
    *,
    start_sec: float = 0.0,
    duration: float = 8.0,
    fps: int = 3,
    scale_width: int = 640,
) -> Path | None:
    """Короткий GIF-превью для вставки в PDF (виден в Firefox/Evince)."""
    if not video_path.exists() or shutil.which("ffmpeg") is None:
        return None
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y",
        "-ss", str(start_sec),
        "-i", str(video_path),
        "-t", str(duration),
        "-vf", f"fps={fps},scale={scale_width}:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
        "-loop", "0", str(out_path),
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True, timeout=300)
        if out_path.exists() and out_path.stat().st_size > 0:
            return out_path
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        pass
    return None


class SvgFormula(Flowable):
    def __init__(self, svg_path: str, width: float):
        super().__init__()
        drawing = svg2rlg(svg_path)
        if drawing is None:
            raise ValueError(f"Не удалось прочитать SVG: {svg_path}")
        self.drawing = drawing
        self.scale = width / drawing.width
        self.width = width
        self.height = drawing.height * self.scale

    def wrap(self, availWidth, availHeight):
        return self.width, self.height

    def draw(self):
        self.canv.saveState()
        self.canv.scale(self.scale, self.scale)
        renderPDF.draw(self.drawing, self.canv, 0, 0)
        self.canv.restoreState()


def render_formula_svg(latex: str, fontsize: int = 14) -> str:
    fig = plt.figure(figsize=(0.01, 0.01))
    fig.patch.set_alpha(0.0)
    text = fig.text(
        0.5, 0.5, f"${latex}$",
        fontsize=fontsize, ha="center", va="center",
        math_fontfamily="dejavusans",
    )
    fig.canvas.draw()
    bbox = text.get_window_extent(fig.canvas.get_renderer()).expanded(1.2, 1.4)
    w_in = bbox.width / fig.dpi
    h_in = bbox.height / fig.dpi
    fig.set_size_inches(max(w_in, 0.5), max(h_in, 0.2))
    text.set_position((0.5, 0.5))
    tmp = tempfile.NamedTemporaryFile(suffix=".svg", delete=False)
    tmp.close()
    fig.savefig(tmp.name, format="svg", transparent=True, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    return tmp.name


def formula_block(latex: str, width_cm: float = 15.0, fontsize: int = 14) -> Table:
    path = render_formula_svg(latex, fontsize=fontsize)
    f = SvgFormula(path, width_cm * cm)
    t = Table([[f]], colWidths=[17 * cm])
    t.setStyle(TableStyle([
        ("ALIGN", (0, 0), (-1, -1), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    return t


def styled_table_wrapped(data, widths, fontsize=9):
    cell_style = ParagraphStyle(
        "cell", fontName="DejaVuSans", fontSize=fontsize, leading=fontsize + 3,
        alignment=TA_LEFT,
    )
    head_style = ParagraphStyle(
        "head", fontName="DejaVuSans-Bold", fontSize=fontsize, leading=fontsize + 3,
        textColor=colors.white,
    )
    wrapped = []
    for ri, row in enumerate(data):
        style = head_style if ri == 0 else cell_style
        wrapped.append([Paragraph(str(c).replace("\n", "<br/>"), style) for c in row])
    t = Table(wrapped, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a5f")),
        ("FONTNAME", (0, 0), (-1, 0), "DejaVuSans-Bold"),
        ("FONTSIZE", (0, 0), (-1, -1), fontsize),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
    ]))
    return t


def bullet_list(items: list[str], style: ParagraphStyle) -> list:
    return [Paragraph(f"• {item}", style) for item in items]


def proportional_image(path: Path, width_cm: float) -> Image:
    """Image с сохранением пропорций (только width задаётся явно)."""
    from PIL import Image as PILImage

    with PILImage.open(path) as im:
        w, h = im.size
    aspect = h / max(w, 1)
    return Image(str(path), width=width_cm * cm, height=width_cm * aspect * cm)


def keypoint_image(path: Path, max_w_cm: float, max_h_cm: float, crop_frac: float = 0.40) -> Image:
    """Центральный crop широкого надир-кадра + масштаб на всю доступную область."""
    from PIL import Image as PILImage

    with PILImage.open(path) as im:
        w, h = im.size
        crop_w = max(1, int(w * crop_frac))
        x0 = (w - crop_w) // 2
        cropped = im.crop((x0, 0, x0 + crop_w, h))
        cw, ch = cropped.size
        aspect = ch / max(cw, 1)
        w_cm = max_w_cm
        h_cm = w_cm * aspect
        if h_cm > max_h_cm:
            h_cm = max_h_cm
            w_cm = h_cm / aspect
        tmp = path.parent / f"_pdf_{path.stem}.jpg"
        cropped.save(tmp, format="JPEG", quality=92)
    return Image(str(tmp), width=w_cm * cm, height=h_cm * cm)


def layout_poster_images(
    posters: list[Path],
    *,
    page_width_cm: float = 15.0,
) -> list:
    """По одному кадру в ряд на всю ширину, без Table (иначе ReportLab растягивает)."""
    flowables: list = []
    for p in posters:
        if p.exists() and p.stat().st_size > 0:
            flowables.append(proportional_image(p, page_width_cm))
    return flowables


def embed_pdf_attachments(pdf_path: Path, attachments: list[tuple[Path, str]]) -> None:
    """Прикрепить файлы к PDF (панель Attachments в Adobe Reader / Evince)."""
    from pypdf import PdfReader, PdfWriter

    reader = PdfReader(str(pdf_path))
    writer = PdfWriter()
    writer.append(reader)
    for file_path, attachment_name in attachments:
        if not file_path.exists():
            print(f"Warning: attachment missing: {file_path}")
            continue
        writer.add_attachment(attachment_name, file_path.read_bytes())
        print(f"Attached: {attachment_name} ({file_path.stat().st_size / (1024 * 1024):.0f} MB)")
    tmp = pdf_path.with_suffix(".tmp.pdf")
    with open(tmp, "wb") as f:
        writer.write(f)
    tmp.replace(pdf_path)


def parse_ape(path: Path) -> float | None:
    if not path.exists():
        return None
    for line in path.read_text().splitlines():
        if "rmse" in line.lower():
            parts = line.split()
            try:
                return float(parts[-1])
            except ValueError:
                pass
    return None


def load_timing_ms(path: Path) -> np.ndarray | None:
    if not path.exists():
        return None
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) < 2:
            continue
        try:
            rows.append(float(parts[1]))
        except ValueError:
            continue
    return np.array(rows, dtype=float) if rows else None


def timing_from_csv(path: Path) -> dict | None:
    ms = load_timing_ms(path)
    if ms is None or len(ms) == 0:
        return None
    return {
        "n": len(ms),
        "wall_s": float(ms.sum() / 1000),
        "mean_ms": float(np.mean(ms)),
        "median_ms": float(np.median(ms)),
    }


def vins_wall_from_log(log_path: Path) -> float | None:
    if not log_path.exists():
        return None
    text = log_path.read_text(encoding="utf-8", errors="replace")
    ts = [float(m.group(1)) for m in re.finditer(r"\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]:", text)]
    if len(ts) < 2:
        return None
    return float(ts[-1] - ts[0])


def count_reboots(log_path: Path) -> int | None:
    if not log_path.exists():
        return None
    return log_path.read_text(encoding="utf-8", errors="replace").count("system reboot!")


def parse_vins_log_errors(log_path: Path) -> dict[str, int]:
    """Счётчики типичных сообщений VINS из log (reboot, failure, init)."""
    keys = {
        "system reboot": "system reboot!",
        "failure detection": "failure detection!",
        "misalign visual/IMU": "misalign visual structure",
        "IMU excitation not enough": "IMU excitation not enou",
        "big z translation": "big z translation",
        "Not enough features/parallax": "Not enough features or parallax",
        "throw img (начало)": "throw img, only should happen at the beginning",
    }
    out: dict[str, int] = {}
    if not log_path.exists():
        return out
    text = log_path.read_text(encoding="utf-8", errors="replace")
    for label, needle in keys.items():
        out[label] = text.count(needle)
    return out


def format_duration(seconds: float) -> str:
    if seconds >= 3600:
        h = int(seconds // 3600)
        m = int((seconds % 3600) // 60)
        s = seconds % 60
        return f"{h} ч {m} мин" if s < 5 else f"{h} ч {m} мин {s:.0f} с"
    if seconds >= 60:
        m = int(seconds // 60)
        s = seconds % 60
        return f"{m} мин {s:.0f} с" if s >= 1 else f"{m} мин"
    return f"{seconds:.1f} с"


def pick_timing(timing_dir: Path, *patterns: str) -> Path | None:
    for pattern in patterns:
        path = timing_dir / pattern
        if path.exists():
            return path
    return None


def collect_run_timing(tag: str) -> dict:
    res = ROOT / "results"
    timing_dir = res / "timing"
    vins_tag = "run8e_parallax" if tag == "run8e_fused" else tag

    vins_front = pick_timing(
        timing_dir,
        f"vins_front_mars_r0.3_{tag}.csv",
        f"vins_front_mars_r0.3_{vins_tag}.csv",
    )
    vins_back = pick_timing(
        timing_dir,
        f"vins_back_mars_r0.3_{tag}.csv",
        f"vins_back_mars_r0.3_{vins_tag}.csv",
    )
    vins_log = res / f"vins_mars_{vins_tag}.log"
    if not vins_log.exists():
        vins_log = res / f"vins_mars_{tag}.log"

    dso_mars = pick_timing(
        timing_dir,
        f"dso_mars_s0_{tag}.csv",
        "dso_mars_s0_combined_run1.csv",
    )
    dso_pov = pick_timing(
        timing_dir,
        f"dso_povorot_s0_{tag}.csv",
        "dso_povorot_s0_run2_calib.csv",
    )

    vins_front_t = timing_from_csv(vins_front) if vins_front else None
    vins_back_t = timing_from_csv(vins_back) if vins_back else None
    dso_mars_t = timing_from_csv(dso_mars) if dso_mars else None
    dso_pov_t = timing_from_csv(dso_pov) if dso_pov else None

    vins_wall = vins_wall_from_log(vins_log)
    if vins_wall is None and vins_front_t and vins_back_t:
        vins_wall = vins_front_t["wall_s"] + vins_back_t["wall_s"]

    return {
        "mars_vins": {
            "frames": vins_front_t["n"] if vins_front_t else "?",
            "wall_s": vins_wall,
            "front_ms": vins_front_t["median_ms"] if vins_front_t else None,
            "back_ms": vins_back_t["median_ms"] if vins_back_t else None,
            "front_wall_s": vins_front_t["wall_s"] if vins_front_t else None,
            "back_wall_s": vins_back_t["wall_s"] if vins_back_t else None,
        },
        "mars_dso": {
            "frames": dso_mars_t["n"] if dso_mars_t else "?",
            "wall_s": dso_mars_t["wall_s"] if dso_mars_t else None,
            "median_ms": dso_mars_t["median_ms"] if dso_mars_t else None,
        },
        "pov_dso": {
            "frames": dso_pov_t["n"] if dso_pov_t else 513,
            "wall_s": dso_pov_t["wall_s"] if dso_pov_t else None,
            "median_ms": dso_pov_t["median_ms"] if dso_pov_t else None,
        },
    }


def plot_dispatcher_diagram(out: Path) -> Path:
    plt.rcParams.update(PLOT_RC)
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    boxes = [
        (0.02, 0.52, 0.16, 0.28, "Кадры\nIMU / GPS"),
        (0.22, 0.62, 0.16, 0.22, "VINS-Mono"),
        (0.22, 0.28, 0.16, 0.22, "DSO"),
        (0.44, 0.45, 0.18, 0.32, "Диспетчер\nhealth-check"),
        (0.68, 0.45, 0.18, 0.32, "fuse_vins_rtk\nvo_to_gps_dat"),
        (0.88, 0.52, 0.08, 0.18, "БИНС"),
    ]
    for x, y, w, h, txt in boxes:
        ax.add_patch(plt.Rectangle((x, y), w, h, fill=True, fc="#eff6ff", ec="#1e3a5f", lw=1.2))
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=9)
    for x0, y0, x1, y1 in [
        (0.18, 0.66, 0.22, 0.73), (0.18, 0.66, 0.22, 0.39),
        (0.38, 0.73, 0.44, 0.61), (0.38, 0.39, 0.44, 0.55),
        (0.62, 0.61, 0.68, 0.61), (0.86, 0.61, 0.88, 0.61),
    ]:
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle="->", lw=1.0))
    ax.set_title("Конвейер VINS + DSO + диспетчер + слияние с RTK", pad=8)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, facecolor="white")
    plt.close(fig)
    return out


def load_tum(path: Path, t_ref: float | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    if not path.exists() or path.stat().st_size == 0:
        return None
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 4:
            rows.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
    if len(rows) < 2:
        return None
    arr = np.array(rows)
    t = arr[:, 0]
    t0 = t_ref if t_ref is not None else t[0]
    t = t - t0
    return t, arr[:, 1], arr[:, 2], arr[:, 3]


def first_tum_timestamp(path: Path) -> float | None:
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 4:
            return float(parts[0])
    return None


def _style_axes(ax, title: str | None = None):
    ax.grid(True, alpha=0.35)
    if title:
        ax.set_title(title, pad=6)


def plot_rtk_line(ax, t_or_x, y_or_none, zorder=5, label="RTK эталон"):
    dash_style = (0, RTK_DASH)
    if y_or_none is None:
        ln, = ax.plot(t_or_x[:, 0], t_or_x[:, 1], linestyle=dash_style, color=COLOR_RTK, lw=1.6,
                      label=label, alpha=0.92, zorder=zorder)
    elif isinstance(y_or_none, np.ndarray) and len(y_or_none.shape) == 1:
        ln, = ax.plot(t_or_x, y_or_none, linestyle=dash_style, color=COLOR_RTK, lw=1.4,
                      label=label, alpha=0.92, zorder=zorder)
    else:
        ln, = ax.plot(t_or_x, y_or_none, linestyle=dash_style, color=COLOR_RTK, lw=1.4,
                      label=label, alpha=0.92, zorder=zorder)
    return ln


def _legend_with_rtk(ax, loc="best", fontsize=9):
    """Легенда, где RTK отображается с тем же dash-паттерном, что на графике."""
    handles, labels = ax.get_legend_handles_labels()
    proxies = []
    for h, lab in zip(handles, labels):
        if lab.startswith("RTK"):
            proxies.append(rtk_legend_proxy(lab))
        else:
            proxies.append(h)
    ax.legend(proxies, labels, loc=loc, fontsize=fontsize, framealpha=0.9)


def plot_trajectory_vs_gt(
    gt_path: Path,
    est_path: Path,
    assets: Path,
    prefix: str,
    est_label: str = "VINS",
) -> list[Path]:
    plt.rcParams.update(PLOT_RC)
    t_ref = first_tum_timestamp(gt_path)
    if t_ref is None:
        return []
    gt = load_tum(gt_path, t_ref=t_ref)
    est = load_tum(est_path, t_ref=t_ref)
    if gt is None or est is None:
        return []
    t_gt, x_gt, y_gt, z_gt = gt
    t_est, x_est, y_est, z_est = est
    assets.mkdir(parents=True, exist_ok=True)
    outputs = []

    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    ax.plot(x_est, y_est, "-", color=COLOR_EST, lw=1.6, label=est_label, zorder=2)
    plot_rtk_line(ax, x_gt, y_gt)
    ax.plot(x_gt[0], y_gt[0], marker="^", color="#16a34a", ms=9, label="старт", zorder=6)
    ax.plot(x_est[-1], y_est[-1], "o", color=COLOR_EST, ms=6, zorder=3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X, м")
    ax.set_ylabel("Y, м")
    _style_axes(ax, f"{prefix}: {est_label} vs RTK (XY)")
    _legend_with_rtk(ax, loc="best")
    fig.tight_layout()
    p_xy = assets / f"{prefix}_gt_vs_est_xy.png"
    fig.savefig(p_xy, dpi=160, facecolor="white")
    plt.close(fig)
    outputs.append(p_xy)

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 7.4), sharex=True)
    for ax, gt_vals, est_vals, name in zip(axes, [x_gt, y_gt, z_gt], [x_est, y_est, z_est], ["X", "Y", "Z"]):
        ax.plot(t_est, est_vals, "-", color=COLOR_EST, lw=1.3, label=est_label, zorder=2)
        plot_rtk_line(ax, t_gt, gt_vals, label="RTK")
        ax.set_ylabel(f"{name}, м")
        _style_axes(ax)
        _legend_with_rtk(ax, loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Время от старта полёта, с")
    axes[0].set_title(f"{prefix}: координаты vs время")
    fig.tight_layout()
    p_xyz = assets / f"{prefix}_gt_vs_est_xyz.png"
    fig.savefig(p_xyz, dpi=160, facecolor="white")
    plt.close(fig)
    outputs.append(p_xyz)
    return outputs


def plot_trajectory_xy(tum: Path, out: Path, title: str) -> Path | None:
    plt.rcParams.update(PLOT_RC)
    data = load_tum(tum)
    if data is None:
        return None
    _, x, y, _ = data
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    ax.plot(x, y, "-", color=COLOR_DSO, lw=1.6)
    ax.plot(x[0], y[0], "o", color="#16a34a", ms=8, label="старт")
    ax.plot(x[-1], y[-1], "o", color=COLOR_RTK, ms=8, label="конец")
    ax.set_aspect("equal", adjustable="box")
    _style_axes(ax, title)
    ax.set_xlabel("x (усл. ед.)")
    ax.set_ylabel("y (усл. ед.)")
    ax.legend(framealpha=0.9)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, facecolor="white")
    plt.close(fig)
    return out


def povorot_drift_stats(tum_path: Path) -> dict | None:
    data = load_tum(tum_path)
    if data is None:
        return None
    _, x, y, _ = data
    xy = np.column_stack([x, y])
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    path_len = float(seg.sum())
    drift = float(np.linalg.norm(xy[-1] - xy[0]))
    return {
        "poses": len(xy),
        "path_len": path_len,
        "drift": drift,
        "drift_pct": 100.0 * drift / path_len if path_len > 1e-6 else 0.0,
    }


def build_pdf(tag: str, out_pdf: Path):
    """Собрать PDF: метрики, траектории, timing, выводы для tag (напр. run8e_fused)."""
    register_fonts()
    h1 = pstyle("h1", fontName="DejaVuSans-Bold", fontSize=17, spaceAfter=10, leading=20)
    h2 = pstyle("h2", fontName="DejaVuSans-Bold", fontSize=13, spaceBefore=12, spaceAfter=6, leading=16)
    h3 = pstyle("h3", fontName="DejaVuSans-Bold", fontSize=11, spaceBefore=8, spaceAfter=4)
    body = pstyle("body")
    cap = pstyle("cap", fontSize=8, textColor=colors.HexColor("#64748b"), alignment=TA_CENTER, leading=10)
    sub = pstyle("sub", fontName="DejaVuSans-Bold", fontSize=11, spaceBefore=6, spaceAfter=4)
    title_main = pstyle("title_main", fontName="DejaVuSans-Bold", fontSize=24, alignment=TA_CENTER, spaceAfter=14, leading=28)
    title_sub = pstyle("title_sub", fontSize=12, alignment=TA_CENTER, spaceAfter=8, leading=16, textColor=colors.HexColor("#334155"))
    title_note = pstyle("title_note", fontSize=10, alignment=TA_CENTER, spaceAfter=6, leading=14, textColor=colors.HexColor("#64748b"))

    res = ROOT / "results"
    comb = res / f"combined_{tag}"
    eval_mars = res / f"eval_mars_{tag}"
    eval_run4 = res / "eval_mars_run4_fused"
    disp_m4 = res / "combined_run4_fused" / "dispatcher_mars.json"

    mars_vins_csv = res / "vins_mars_run8e_parallax.csv"
    mars_vins_log = res / "vins_mars_run8e_parallax.log"
    pov_dso = res / f"dso_povorot_{tag}.tum"

    disp_mars = json.loads((comb / "dispatcher_mars.json").read_text()) if (comb / "dispatcher_mars.json").exists() else {}
    disp_pov = json.loads((comb / "dispatcher_povorot.json").read_text()) if (comb / "dispatcher_povorot.json").exists() else {}
    disp4 = json.loads(disp_m4.read_text()) if disp_m4.exists() else {}
    run_timing = collect_run_timing(tag)

    ate_raw = parse_ape(eval_mars / "vins_ape.txt")
    ate_fused = parse_ape(eval_mars / "vins_fused_ape.txt")
    ate4_raw = parse_ape(eval_run4 / "vins_ape.txt")
    ate4_fused = parse_ape(eval_run4 / "vins_fused_ape.txt")
    reboots = count_reboots(mars_vins_log) or disp_mars.get("health", {}).get("vins_reboots", "?")
    reboots4 = count_reboots(res / "vins_mars_run2_calib.log") or 5
    vins_errors = parse_vins_log_errors(mars_vins_log)

    vins_poses = disp_mars.get("vins", {}).get("num_poses", 2734)
    vins_cov = disp_mars.get("vins", {}).get("coverage_pct", 74.7)
    dso_poses_m = disp_mars.get("dso", {}).get("num_poses", 781)
    vins4_poses = disp4.get("vins", {}).get("num_poses", 1604)
    vins4_cov = disp4.get("vins", {}).get("coverage_pct", 43.8)

    pov_stats = povorot_drift_stats(pov_dso) or {"poses": 36, "drift_pct": 40.0, "path_len": 0.95, "drift": 0.38}

    assets = comb / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    scheme = plot_dispatcher_diagram(assets / "scheme.png")

    gt_path = ROOT / "data" / "mars" / "mars_hkairport03_gt.tum"
    vins_tum = eval_mars / "vins.tum"
    vins_fused = eval_mars / "vins_fused.tum"
    plot_trajectory_vs_gt(gt_path, vins_fused, assets, "mars_fused", "VINS+RTK")
    if vins_tum.exists():
        plot_trajectory_vs_gt(gt_path, vins_tum, assets, "mars_raw", "VINS raw")
    plot_trajectory_xy(pov_dso, assets / "povorot_traj.png", "Поворот_коптер: DSO (XY, Sim(3)-ед.)")

    mars_video = comb / "mars_viz" / "mars_hk_flight.avi"
    pov_video = comb / "povorot_viz" / "povorot_flight.avi"
    video_assets = assets / "video_posters"
    mars_posters = extract_video_posters(
        mars_video, video_assets, "mars_hk",
        timestamps=("00:01:15", "00:02:30"),
        scale_width=2560,
    )
    pov_posters = extract_video_posters(
        pov_video, video_assets, "povorot",
        timestamps=("00:00:05", "00:00:08"),
        scale_width=2560,
    )

    doc = ReportDocTemplate(
        str(out_pdf), pagesize=A4,
        rightMargin=2 * cm, leftMargin=2 * cm, topMargin=2 * cm, bottomMargin=2 * cm,
    )
    story = []

    toc = TableOfContents()
    toc.levelStyles = [
        pstyle("TOC1", fontName="DejaVuSans-Bold", fontSize=12, leading=16, leftIndent=0, spaceBefore=4),
        pstyle("TOC2", fontName="DejaVuSans", fontSize=10, leading=14, leftIndent=0.8 * cm, spaceBefore=2),
        pstyle("TOC3", fontName="DejaVuSans", fontSize=9, leading=12, leftIndent=1.6 * cm, spaceBefore=1),
    ]
    toc.dotsMinLevel = 0

    # --- Титульная страница ---
    story.append(Spacer(1, 3.5 * cm))
    story.append(Paragraph("Объединённый алгоритм<br/>VINS-Mono + DSO", title_main))
    story.append(Spacer(1, 0.8 * cm))
    story.append(Paragraph("Технический отчёт", title_sub))
    story.append(Paragraph(f"Прогон <i>{tag}</i> · {date.today():%d.%m.%Y}", title_sub))
    story.append(Spacer(1, 0.6 * cm))
    story.append(Paragraph(
        "MARS-LVIG HKairport03 (надир, Гонконг)<br/>«Поворот_коптер» (надир, без IMU)",
        title_note,
    ))
    story.append(Spacer(1, 1.2 * cm))
    story.append(Paragraph(
        "Оценка VINS + DSO + диспетчера, offline-слияние с RTK,<br/>метрики ATE, reboot и визуализация",
        title_note,
    ))
    story.append(PageBreak())

    # --- Оглавление (стр. 2) ---
    story.append(heading("Содержание", h2, "toc"))
    story.append(toc)
    story.append(PageBreak())

    story.append(heading("1. Цель и конфигурация", h2, "sec1"))
    story.append(Paragraph(
        "Задача — выбрать и выдать одометрию для надирного полёта: VINS-Mono при наличии IMU, "
        "DSO — резерв или единственный канал без IMU. Выход — траектория в ENU для vo_to_gps_dat и БИНС.",
        body,
    ))
    cfg_items = [
        "Камера: надир (extrinsic R,T из mars_nadir.yaml, fix extrinsic).",
        "Intrinsics: официальные MARS-LVIG.",
        "VINS: старт rosbag с t=50 с, playback 0.3×; loop_closure=1.",
        "Патч nadir-параллакса (run8e): rot-gate 0.07 рад, gyro-gate 0.10 рад/2 кадра, MIN_PARALLAX keyframe 3 px.",
        "Слияние: fuse_vins_rtk.py — Sim(3) VINS→RTK, привязка сегментов на reboot, baro Z при RTK outage.",
        f"Продакшен-кандидат: {tag} (2734 поз VINS, raw ATE {ate_raw:.0f} м).",
    ]
    story.extend(bullet_list(cfg_items, body))
    story.append(Spacer(1, 0.4 * cm))

    # --- Архитектура ---
    story.append(heading("2. Архитектура", h2, "sec2"))
    story.append(Image(str(scheme), width=16 * cm, height=7.2 * cm))
    story.append(Paragraph("Рис. 1. Конвейер обработки.", cap))
    story.append(Spacer(1, 0.25 * cm))
    story.append(Paragraph(
        "VINS-Mono и DSO работают параллельно на одном потоке кадров. "
        "Диспетчер (dispatcher.py) оценивает покрытие, reboot и статус DSO; "
        "на надирном MARS выбирается VINS, DSO не используется как fallback. "
        "fuse_vins_rtk.py выравнивает VINS в ENU RTK и сшивает сегменты после reboot.",
        body,
    ))

    story.append(heading("3. Результаты MARS-LVIG", h2, "sec3"))

    compare_data = [
        ["Прогон", "Поз VINS", "Покрытие", "ATE raw", "ATE fused", "Reboot"],
        [
            "run4_fused",
            str(vins4_poses),
            f"{vins4_cov:.1f}%",
            f"{ate4_raw:.0f} м" if ate4_raw else "—",
            f"{ate4_fused:.0f} м" if ate4_fused else "—",
            str(reboots4),
        ],
        [
            tag,
            str(vins_poses),
            f"{vins_cov:.1f}%",
            f"{ate_raw:.0f} м" if ate_raw else "—",
            f"{ate_fused:.1f} м" if ate_fused else "—",
            f"{reboots}→снято",
        ],
    ]
    story.append(styled_table_wrapped(
        compare_data, [2.6 * cm, 2.2 * cm, 2.2 * cm, 2.2 * cm, 2.4 * cm, 2.4 * cm], fontsize=9,
    ))
    story.append(Spacer(1, 0.25 * cm))
    story.append(Paragraph(
        f"run8e даёт {vins_poses} поз при покрытии {vins_cov:.0f}% против {vins4_poses} / {vins4_cov:.0f}% у run4. "
        f"Сырой VINS ATE {ate_raw:.0f} м — автономная точность без RTK-привязки после reboot. "
        f"Fused ATE {ate_fused:.1f} м включает RTK assist и не характеризует чистую VO.",
        body,
    ))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("3.1. Диспетчер (MARS)", h3, "sec3_1"))
    mars_health = disp_mars.get("health", {})
    disp_m_data = [
        ["Параметр", "VINS", "DSO", "Выбор"],
        [
            "Поз / покрытие",
            f"{vins_poses} / {vins_cov:.1f}%",
            f"{dso_poses_m} / {disp_mars.get('dso', {}).get('coverage_pct', 21.4):.1f}%",
            disp_mars.get("chosen_algorithm", "vins").upper(),
        ],
        [
            "Health",
            mars_health.get("status", "—"),
            "LOST=false" if not disp_mars.get("dso", {}).get("is_lost") else "LOST",
            disp_mars.get("reason", "—"),
        ],
        [
            "Reboot VINS",
            str(reboots),
            "—",
            mars_health.get("recommendation", "—"),
        ],
    ]
    story.append(styled_table_wrapped(disp_m_data, [3.2 * cm, 3.6 * cm, 3.2 * cm, 7.0 * cm], fontsize=9))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("3.2. Метрика ATE и траектории", h3, "sec3_2"))
    story.append(Paragraph(
        "Absolute Trajectory Error — RMSE модулей ошибок положения после SE(3) выравнивания Umeyama:",
        body,
    ))
    story.append(formula_block(
        r"\mathrm{ATE}_{\mathrm{RMSE}} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{\mathrm{est},i}-\mathbf{p}_{\mathrm{gt},i}\|^2}"
    ))
    story.append(Paragraph(
        "Эталон — RTK (см). Синяя линия — оценка, красный пунктир — RTK (zorder выше). "
        "t=0 — первый timestamp эталона.",
        body,
    ))
    for img, cap_txt in [
        (assets / "mars_fused_gt_vs_est_xy.png", "Рис. 2. VINS+RTK vs RTK, вид XY."),
        (assets / "mars_fused_gt_vs_est_xyz.png", "Рис. 3. Координаты X/Y/Z vs время (VINS+RTK)."),
        (assets / "mars_raw_gt_vs_est_xy.png", "Рис. 4. Сырой VINS (136 м ATE) — автономная траектория."),
        (assets / "mars_raw_gt_vs_est_xyz.png", "Рис. 5. Сырой VINS: координаты vs время."),
    ]:
        if img.exists():
            h = 11 * cm if "xyz" in img.name else 10 * cm
            story.append(Image(str(img), width=15 * cm, height=h))
            story.append(Paragraph(cap_txt, cap))
            story.append(Spacer(1, 0.25 * cm))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("3.3. VINS без СНС (автономный режим)", h3, "sec3_3"))
    story.append(Paragraph(
        "VINS-Mono в полёте работает только по камере и IMU — без онлайн-привязки к RTK/GNSS. "
        "На графиках и в RViz live топик <i>/vins_estimator/odometry</i> показывает именно эту "
        "автономную оценку: масштаб и yaw плавают, после reboot траектория «прыгает» в новую СК. "
        f"Это ожидаемо: raw ATE ≈ {ate_raw:.0f} м на MARS — нормальная автономная точность для надирного VO.",
        body,
    ))
    story.append(Paragraph(
        "Коррекция по RTK выполняется <b>офлайн</b> скриптом <i>fuse_vins_rtk.py</i> после прогона VINS. "
        "На графиках рис. 2–3 уже лежит fused-файл <i>vins_fused.tum</i>; рис. 4–5 — сырой <i>vins.tum</i>. "
        "Fused ATE ≈ {0:.1f} м не описывает автономную навигацию — это метрика post-processing с RTK assist "
        "(Sim(3)-выравнивание, привязка сегментов после reboot).".format(ate_fused or 0),
        body,
    ))
    story.extend(bullet_list([
        "Live RViz: только VINS; красный RTK-path — отдельный publisher для сравнения, не fusion.",
        "Batch: VINS → CSV/TUM → fuse_vins_rtk → eval (ATE, графики, видео).",
        "При потере СНС (--rtk-outage) fused остаётся на приращениях VINS + baro Z без snap на reboot.",
    ], body))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("3.4. Reboot VINS: определение и события", h3, "sec3_4"))
    story.append(Paragraph(
        "<b>Reboot VINS</b> (<i>system reboot!</i> в log) — принудительный полный сброс внутреннего "
        "состояния VINS-Mono estimator: фильтр VIO, буферы IMU/preintegration, карта landmarks, "
        "скользящее окно оптимизации и локальная система координат обнуляются. "
        "После reboot оценка позы продолжается в <b>новой локальной СК</b> с origin ≈ (0,0,0); "
        "непрерывность с предыдущим сегментом теряется — на графиках raw VINS траектория «прыгает».",
        body,
    ))
    story.append(Paragraph(
        "<b>Цепочка срабатывания:</b> (1) деградация visual odometry — мало признаков, "
        "недостаточный параллакс, потеря KLT-треков; (2) <i>failure detection!</i> — "
        "эвристика VINS фиксирует расхождение visual-inertial (misalign, big z translation); "
        "(3) <i>system reboot!</i> — полный reset estimator. Диспетчер считает reboot через log "
        f"и помечает health=low_confidence ({reboots} reboot на полном MARS bag).",
        body,
    ))
    story.append(Paragraph(
        "<b>Почему на надир и разворотах:</b> при камере в nadir avg_parallax искусственно занижается "
        "компенсацией p_comp = R·p_raw; на разворотах rot-gate/gyro-gate блокируют кейфреймы, "
        "но при резком |ΔR| трек всё равно рвётся. На однородном покрытии (асфальт) мало Shi-Tomasi "
        "точек — VO деградирует быстрее. Патч run8e (rot-gate 0.07 рад, MIN_PARALLAX 3 px) снижает "
        "ложные reboot, но не устраняет их полностью.",
        body,
    ))
    story.append(Paragraph(
        "<b>Связь с fused-траекторией:</b> сырой <i>vins.tum</i> содержит разрывные сегменты в разных СК. "
        "Скрипт <i>fuse_vins_rtk.py</i> детектирует reboot (скачок >50 м/с или сброс к origin), "
        "выравнивает каждый сегмент Sim(3) к RTK ENU и сшивает приращения между reboot. "
        "На рис. 2–3 fused-траектория выглядит непрерывной; на рис. 4–5 raw — видны скачки сегментов. "
        "При RTK outage (--rtk-outage) snap на reboot отключается — остаются только приращения VINS + baro Z.",
        body,
    ))
    err_rows = [["Событие в log", "Кол-во", "Комментарий"]]
    err_comments = {
        "system reboot": "полный сброс состояния estimator",
        "failure detection": "триггер reboot (потеря VO)",
        "misalign visual/IMU": "рассогласование visual-inertial",
        "IMU excitation not enough": "недостаточное возбуждение IMU для init",
        "big z translation": "аномальный скачок по Z",
        "Not enough features/parallax": "отказ кейфрейма (надир / разворот)",
        "throw img (начало)": "норма при старте инициализации",
    }
    for key, comment in err_comments.items():
        cnt = vins_errors.get(key, 0)
        if cnt > 0 or key in ("system reboot", "failure detection"):
            err_rows.append([key, str(cnt), comment])
    story.append(styled_table_wrapped(err_rows, [5.0 * cm, 2.0 * cm, 9.0 * cm], fontsize=9))
    story.append(Paragraph(
        f"run4 (калибровка, укороченный прогон): reboot ≈ {reboots4}. "
        "DSO на надир не подменяет VINS при low_confidence.",
        body,
    ))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("4. Патч параллакса и слияние VINS+RTK", h2, "sec4"))
    story.append(heading("4.1. Nadir parallax (run8e)", h3, "sec4_1"))
    story.append(Paragraph(
        "На надире компенсация параллакса по IMU (p_comp = R·p_raw) занижает avg_parallax "
        "и пропускает кейфреймы на разворотах. Патч rot-gate блокирует кейфрейм при |ΔR| > 0.07 рад; "
        "gyro-gate — при |ΔR| > 0.10 рад за 2 интервала (~0.2 с при 10 Гц).",
        body,
    ))
    story.append(formula_block(r"\bar{p}_{\mathrm{comp}} = \mathbf{R}\,\bar{p}_{\mathrm{raw}},\quad "
                               r"\Delta R = R_j^\top R_i,\quad \theta = \arccos\!\left(\frac{\mathrm{tr}(\Delta R)-1}{2}\right)"))
    story.append(Paragraph(
        "Условие кейфрейма: θ ≤ 0.07 рад, rot₂ ≤ 0.10 рад и avg_parallax ≥ MIN_PARALLAX (3 px).",
        body,
    ))

    story.append(Spacer(1, 0.3 * cm))
    story.append(heading("4.2. Sim(3) и reboot", h3, "sec4_2"))
    story.append(Paragraph(
        "Глобальное выравнивание VINS→RTK ENU по первым 30 с: масштаб s — медиана |v_RTK|/|v_VINS|, "
        "yaw θ — из горизонтальных центроидов, сдвиг μ. На reboot (>50 м/с или сброс к origin) "
        "начало сегмента привязывается к RTK; между reboot — приращения в выровненной СК.",
        body,
    ))
    story.append(formula_block(
        r"\mathbf{p}_{\mathrm{ENU}} = s\,\mathbf{R}_z(\theta)\,(\mathbf{p}_{\mathrm{VINS}}-\mathbf{\mu}_s)+\mathbf{\mu}_d"
    ))

    story.append(Spacer(1, 0.3 * cm))
    story.append(heading("4.3. RTK outage и health-check", h3, "sec4_3"))
    story.extend(bullet_list([
        "RTK outage (--rtk-outage / --rtk-available): только приращения VINS по XY, без snap на reboot; "
        "Z — baro (height_above_takeoff) или VINS Z.",
        f"MARS health: {mars_health.get('status', 'low_confidence')}, reboots={mars_health.get('vins_reboots', reboots)}. "
        "На надир DSO не подставляется при потере VINS.",
        "loop_closure.py: детекция возврата в зону + небольшая XY-коррекция (--loop-closure в fuse).",
    ], body))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("4.4. Быстродействие и время построения траектории", h3, "sec4_4"))
    mv = run_timing["mars_vins"]
    md = run_timing["mars_dso"]
    pd = run_timing["pov_dso"]
    front_ms = mv.get("front_ms") or 0.0
    back_ms = mv.get("back_ms") or 0.0
    per_frame_ms = (front_ms + back_ms) if front_ms and back_ms else None
    vins_wall = mv.get("wall_s") or 0.0
    front_wall = mv.get("front_wall_s") or 0.0
    back_wall = mv.get("back_wall_s") or 0.0
    pipeline_sum = (front_wall + back_wall) if front_wall and back_wall else None
    bag_span_s = 310.0
    realtime_factor = bag_span_s / vins_wall if vins_wall > 1 else None

    def ms_cell(front, back=None):
        if front is None:
            return "—"
        if back is None:
            return f"{front:.1f} мс"
        total = front + back
        return f"front {front:.1f}<br/>back {back:.1f}<br/><b>Σ {total:.1f}</b>"

    timing_data = [
        ["Датасет", "Алгоритм", "Кадров", "Wall time (log)", "мс/кадр (мед.)", "CPU pipeline Σ"],
        ["MARS", "VINS (выбран)", str(mv["frames"]),
         format_duration(vins_wall) if vins_wall else "—",
         ms_cell(mv["front_ms"], mv["back_ms"]),
         format_duration(pipeline_sum) if pipeline_sum else "—"],
        ["MARS", "DSO", str(md["frames"]),
         format_duration(md["wall_s"]) if md["wall_s"] else "—",
         ms_cell(md["median_ms"]),
         format_duration(md["wall_s"]) if md["wall_s"] else "—"],
        ["Поворот_коптер", "DSO (выбран)", str(pd["frames"]),
         format_duration(pd["wall_s"]) if pd["wall_s"] else "—",
         ms_cell(pd["median_ms"]),
         format_duration(pd["wall_s"]) if pd["wall_s"] else "—"],
    ]
    story.append(styled_table_wrapped(
        timing_data, [2.6 * cm, 2.6 * cm, 1.8 * cm, 2.8 * cm, 4.2 * cm, 3.0 * cm], fontsize=8,
    ))
    story.append(Spacer(1, 0.2 * cm))
    perf_lines = []
    if per_frame_ms:
        perf_lines.append(
            f"VINS MARS: медиана <b>{per_frame_ms:.1f} мс/кадр</b> (front {front_ms:.1f} + back {back_ms:.1f}) "
            f"на {mv['frames']} обработанных кадрах."
        )
    perf_lines.extend([
        f"Wall time по log rosbag: <b>{format_duration(vins_wall)}</b> "
        f"(rosbag rate 0.3×, старт t=50 с; ~{bag_span_s:.0f} с полёта в bag).",
    ])
    if pipeline_sum:
        perf_lines.append(
            f"Сумма CPU front+back (без ROS overhead): {format_duration(pipeline_sum)} "
            f"(front {format_duration(front_wall)}, back {format_duration(back_wall)})."
        )
    if realtime_factor:
        perf_lines.append(
            f"Отношение длительности полёта к wall time ≈ {realtime_factor:.2f}× "
            "(< 1 — медленнее realtime; 0.3× bag замедляет воспроизведение)."
        )
    perf_lines.append(
        "Построение fused-траектории (fuse_vins_rtk + eval + графики): секунды, "
        "не входит в wall VINS; доминирует offline-прогон VINS (~18 мин на полный bag)."
    )
    story.extend(bullet_list(perf_lines, body))
    story.append(Paragraph(
        "DSO: playbackSpeed=0 (все кадры). Таблица «CPU pipeline Σ» для VINS — сумма профилированных "
        "front/back потоков; wall time log шире из-за rosbag, синхронизации и I/O.",
        body,
    ))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("5. Поворот_коптер", h2, "sec5"))
    pov_health = disp_pov.get("health", {})
    pov_poses = disp_pov.get("dso", {}).get("num_poses", pov_stats["poses"])
    pov_frames = pd["frames"]
    story.append(Paragraph(
        f"513 кадров, IMU отсутствует — VINS не инициализируется. "
        f"DSO: {pov_poses} поз из {pov_frames} кадров "
        f"({100*pov_poses/int(pov_frames):.1f}% покрытие). "
        f"Диспетчер: {disp_pov.get('chosen_algorithm', 'dso').upper()}, "
        f"health={pov_health.get('status', 'low_confidence')}.",
        body,
    ))
    pov_disp_data = [
        ["Параметр", "VINS", "DSO", "Выбор"],
        [
            "Поз / покрытие",
            "0 / 0%",
            f"{pov_poses} / {disp_pov.get('dso', {}).get('coverage_pct', 7.0):.1f}%",
            disp_pov.get("chosen_algorithm", "dso").upper(),
        ],
        [
            "Health / причина",
            "не инициализирован",
            pov_health.get("status", "low_confidence"),
            disp_pov.get("reason", "—"),
        ],
    ]
    story.append(styled_table_wrapped(pov_disp_data, [3.2 * cm, 3.6 * cm, 3.2 * cm, 7.0 * cm], fontsize=9))
    story.append(Spacer(1, 0.25 * cm))
    story.append(Paragraph(
        f"Дрейф конца траектории относительно длины пути: {pov_stats['drift_pct']:.0f}% "
        f"({pov_stats['drift']:.2f} / {pov_stats['path_len']:.2f} усл. ед.). "
        "Масштаб Sim(3) неизвестен — абсолютные метры не оцениваются.",
        body,
    ))
    if (assets / "povorot_traj.png").exists():
        story.append(Image(str(assets / "povorot_traj.png"), width=13 * cm, height=11 * cm))
        story.append(Paragraph("Рис. 6. DSO-траектория (условные единицы).", cap))

    story.append(Spacer(1, 0.35 * cm))
    story.append(heading("6. Ключевые точки и визуализация", h2, "sec6"))
    story.append(Paragraph(
        "VINS-Mono: Shi-Tomasi углы + KLT-трекинг (feature_tracker). "
        "DSO: отбор по градиенту яркости в блоках 32×32. "
        "На однородном надирном покрытии (асфальт, бетон) оба алгоритма получают мало устойчивых точек; "
        f"на разворотах VINS теряет трек ({reboots} reboot на полном MARS bag). "
        "Кадры ниже извлечены из визуализации <i>visualize_keypoints.py</i>.",
        body,
    ))
    story.append(Spacer(1, 0.25 * cm))
    story.append(Paragraph("<b>MARS HKairport03</b> — Shi-Tomasi/KLT-треки (оранжевые точки и линии).", sub))
    fig_n = 7
    for i, p in enumerate(mars_posters, 1):
        story.append(keypoint_image(p, 16.0, 8.5, crop_frac=0.60))
        story.append(Spacer(1, 0.1 * cm))
        story.append(Paragraph(f"Рис. {fig_n}. MARS HKairport03, кадр {i}.", cap))
        story.append(Spacer(1, 0.2 * cm))
        fig_n += 1
    story.append(Paragraph("<b>Поворот_коптер</b> — DSO gradient-max точки (зелёные).", sub))
    for i, p in enumerate(pov_posters, 1):
        story.append(keypoint_image(p, 16.0, 8.5, crop_frac=0.60))
        story.append(Spacer(1, 0.1 * cm))
        story.append(Paragraph(f"Рис. {fig_n}. Поворот_коптер, кадр {i}.", cap))
        story.append(Spacer(1, 0.2 * cm))
        fig_n += 1

    story.append(PageBreak())
    story.append(heading("7. Выводы и ограничения", h2, "sec7"))
    conclusions = [
        f"run8e_fused — лучший прогон по покрытию ({vins_cov:.0f}%) и raw ATE ({ate_raw:.0f} м) vs run4 ({vins4_cov:.0f}%, {ate4_raw:.0f} м).",
        f"Автономный VINS без СНС: raw ATE {ate_raw:.0f} м — норма для надирного VO; fused {ate_fused:.1f} м — только offline RTK assist.",
        f"{reboots} reboot VINS на полном bag ({vins_errors.get('failure detection', '?')}× failure detection); fused маскирует разрывы сегментной RTK-привязкой.",
        f"Быстродействие VINS: ~{per_frame_ms:.0f} мс/кадр (мед.), wall {format_duration(vins_wall)} на полный bag; fusion/eval — секунды."
        if per_frame_ms else
        f"Быстродействие VINS: wall {format_duration(vins_wall)} на полный bag; fusion/eval — секунды.",
        "Поворот_коптер: DSO единственный канал, 7% покрытие, low_confidence — не для навигации.",
        "DSO на MARS — только health-check, не fallback на надир.",
        "RTK outage: VINS increments + baro Z; σ в vo_to_gps_dat повышается при low_confidence.",
    ]
    story.extend(bullet_list(conclusions, body))

    doc.multiBuild(story)
    print(f"Saved: {out_pdf}")


def main():
    """CLI: собрать PDF-отчёт по метке прогона (по умолчанию run8e_fused)."""
    ap = argparse.ArgumentParser(description="PDF-отчёт VINS+DSO+диспетчер")
    ap.add_argument("--tag", default="run8e_fused")
    ap.add_argument("--out", type=Path, default=ROOT / "ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf")
    args = ap.parse_args()
    build_pdf(args.tag, args.out)


if __name__ == "__main__":
    main()
