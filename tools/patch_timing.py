#!/usr/bin/env python3
"""Врезка счётчиков времени обработки кадра в VINS-Mono и DSO.

Замеряется:
  VINS-Mono фронтенд  -- feature_tracker_node.cpp, весь img_callback
  VINS-Mono бэкенд    -- estimator_node.cpp, processImage со сборкой измерений
  DSO                 -- FullSystem::addActiveFrame, трекинг вместе с картой

Пути к логам берутся из переменных окружения VINS_TIMING_LOG,
VINS_BACKEND_LOG и DSO_TIMING_LOG. Формат строки: "метка_времени,миллисекунды".

Скрипт идемпотентен: повторный запуск обнаруживает маркер и ничего не делает.

    python3 patch_timing.py --vins ~/catkin_ws/src/VINS-Mono --dso ~/dso
    python3 patch_timing.py --vins ... --dso ... --check
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-TIMING]"

VINS_HELPER = """
// {marker} врезка замеров времени, добавлено tools/patch_timing.py
#include <fstream>
#include <iomanip>
#include <cstdlib>
namespace vnav_timing
{{
inline void log_ms(const char *env_var, const char *fallback, double stamp, double ms)
{{
    static std::ofstream out;
    if (!out.is_open())
    {{
        const char *p = std::getenv(env_var);
        out.open(p ? p : fallback, std::ios::app);
    }}
    out << std::fixed << std::setprecision(9) << stamp << ","
        << std::setprecision(4) << ms << "\\n";
    out.flush();
}}
}} // namespace vnav_timing
""".format(marker=MARKER)

DSO_HELPER = """
// {marker} врезка замеров времени, добавлено tools/patch_timing.py
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <chrono>
namespace
{{
// Замер живёт до конца области видимости, поэтому корректно отрабатывает
// все точки выхода из addActiveFrame, включая ранние return.
struct VnavFrameTimer
{{
    std::chrono::steady_clock::time_point t0;
    double stamp;
    explicit VnavFrameTimer(double s)
        : t0(std::chrono::steady_clock::now()), stamp(s) {{}}
    ~VnavFrameTimer()
    {{
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        static std::ofstream out;
        if (!out.is_open())
        {{
            const char *p = std::getenv("DSO_TIMING_LOG");
            out.open(p ? p : "/tmp/dso_timing.csv", std::ios::app);
        }}
        out << std::fixed << std::setprecision(9) << stamp << ","
            << std::setprecision(4) << ms << "\\n";
        out.flush();
    }}
}};
}} // namespace
""".format(marker=MARKER)


def insert_after_includes(text: str, block: str) -> str:
    """Вставить блок после последней #include в ШАПКЕ файла.

    Поиск ограничен началом файла: если в середине исходника попадётся
    ещё один #include, вставка туда могла бы попасть внутрь namespace
    или класса и сломать сборку.
    """
    lines = text.splitlines(keepends=True)
    head_limit = min(len(lines), 150)

    last_end = None
    pos = 0
    for i, line in enumerate(lines):
        if i >= head_limit:
            break
        pos_end = pos + len(line)
        if re.match(r"\s*#include\s+[<\"]", line):
            last_end = pos_end
        pos = pos_end

    if last_end is None:
        return block + text
    return text[:last_end] + block + text[last_end:]


def patch_file(path: Path, edit, dry_run: bool = False) -> str:
    if not path.is_file():
        return f"НЕТ ФАЙЛА  {path}"

    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    if MARKER in text:
        return f"уже пропатчен  {path.name}"

    new_text, note = edit(text)
    if new_text is None:
        return f"ЯКОРЬ НЕ НАЙДЕН  {path.name}: {note}"

    if dry_run:
        return f"будет пропатчен  {path.name} ({note})"

    shutil.copy2(path, path.with_suffix(path.suffix + ".vnav.bak"))
    path.write_text(new_text, encoding="utf-8", errors="surrogateescape")
    return f"пропатчен  {path.name} ({note})"


def append_after_line(text: str, pattern: str, extra: str):
    """Дописать строку сразу после первой строки, подходящей под regex.

    Отступ берётся у найденной строки, чтобы вставка не ломала форматирование.
    """
    rx = re.compile(pattern)
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if rx.search(line):
            indent = re.match(r"[ \t]*", line).group(0)
            eol = "\r\n" if line.endswith("\r\n") else "\n"
            lines.insert(i + 1, f"{indent}{extra}{eol}")
            return "".join(lines)
    return None


def edit_feature_tracker(text: str):
    # Строка ROS_INFO в конце img_callback; параметр функции img_msg в области видимости.
    new = append_after_line(
        text,
        r"whole feature tracker processing costs",
        'vnav_timing::log_ms("VINS_TIMING_LOG", "/tmp/vins_front.csv", '
        'img_msg->header.stamp.toSec(), t_r.toc());',
    )
    if new is None:
        return None, 'не найдена строка "whole feature tracker processing costs"'
    return insert_after_includes(new, VINS_HELPER), "фронтенд"


def edit_estimator(text: str):
    # Строка "double whole_t = t_s.toc();" сразу после estimator.processImage.
    new = append_after_line(
        text,
        r"double\s+whole_t\s*=\s*t_s\.toc\(\)\s*;",
        'vnav_timing::log_ms("VINS_BACKEND_LOG", "/tmp/vins_back.csv", '
        'img_msg->header.stamp.toSec(), whole_t);',
    )
    if new is None:
        return None, "не найдена строка double whole_t = t_s.toc();"
    return insert_after_includes(new, VINS_HELPER), "бэкенд"


def edit_dso(text: str):
    # Сигнатура может быть записана с разными пробелами, ищем гибко.
    m = re.search(
        r"void\s+FullSystem::addActiveFrame\s*\([^)]*\)\s*\n?\s*\{",
        text,
    )
    if m is None:
        return None, "не найдена FullSystem::addActiveFrame"
    pos = m.end()
    call = "\n    VnavFrameTimer __vnav_timer(image->timestamp);\n"
    text = text[:pos] + call + text[pos:]
    return insert_after_includes(text, DSO_HELPER), "addActiveFrame"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vins", type=Path, help="корень репозитория VINS-Mono")
    ap.add_argument("--dso", type=Path, help="корень репозитория DSO")
    ap.add_argument("--check", action="store_true",
                    help="только показать, что будет сделано")
    args = ap.parse_args()

    if not args.vins and not args.dso:
        ap.error("укажите --vins и/или --dso")

    jobs = []
    if args.vins:
        root = args.vins.expanduser()
        jobs.append((root / "feature_tracker/src/feature_tracker_node.cpp",
                     edit_feature_tracker))
        jobs.append((root / "vins_estimator/src/estimator_node.cpp",
                     edit_estimator))
    if args.dso:
        root = args.dso.expanduser()
        jobs.append((root / "src/FullSystem/FullSystem.cpp", edit_dso))

    print("=== Врезка счётчиков времени ===")
    failed = False
    for path, edit in jobs:
        msg = patch_file(path, edit, dry_run=args.check)
        print("  " + msg)
        if "НЕ НАЙДЕН" in msg or "НЕТ ФАЙЛА" in msg:
            failed = True

    if failed:
        print("\nЧасть якорей не найдена — вероятно, версия исходников отличается.")
        print("Резервные копии файлов лежат рядом с расширением .vnav.bak")
        return 1

    if not args.check:
        print("\nТеперь обязательна пересборка:")
        print("  cd ~/catkin_ws && catkin_make -j$(nproc)")
        print("  cd ~/dso/build && make -j$(nproc)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
