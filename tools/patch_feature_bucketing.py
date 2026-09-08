#!/usr/bin/env python3
"""Сеточный отбор точек в VINS-Mono feature_tracker (блоки 32×32, как DSO).

Заменяет глобальный goodFeaturesToTrack на detectFeaturesGrid:
  - по одной (или нескольким) Shi-Tomasi точке на ячейку 32×32;
  - fallback: максимум градиента в ячейке для тёмных/однородных участков.

Идемпотентен: повторный запуск видит маркер [VNAV-GRID-BUCKET] и ничего не делает.

    python3 patch_feature_bucketing.py --vins ~/catkin_ws/src/VINS-Mono
    python3 patch_feature_bucketing.py --vins ... --revert
    python3 patch_feature_bucketing.py --vins ... --check
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MARKER = "[VNAV-GRID-BUCKET]"

HEADER_DECL = """
    void detectFeaturesGrid(const cv::Mat &img, int need_cnt);
"""

DETECT_METHOD = """
// {marker} сеточный отбор точек (блоки 32×32, как DSO)
void FeatureTracker::detectFeaturesGrid(const cv::Mat &img, int need_cnt)
{{
    n_pts.clear();
    if (need_cnt <= 0)
        return;

    const int GRID_CELL = 32;
    const int grid_cols = (COL + GRID_CELL - 1) / GRID_CELL;
    const int grid_rows = (ROW + GRID_CELL - 1) / GRID_CELL;
    const int cell_min_dist = std::max(3, std::min(MIN_DIST, GRID_CELL / 4));

    cv::Mat eig;
    cv::cornerMinEigenVal(img, eig, 3, 3);

    std::vector<std::pair<float, cv::Point2f>> candidates;
    candidates.reserve(grid_cols * grid_rows);

    for (int gy = 0; gy < grid_rows; ++gy)
    {{
        for (int gx = 0; gx < grid_cols; ++gx)
        {{
            const int x0 = gx * GRID_CELL;
            const int y0 = gy * GRID_CELL;
            const int cw = std::min(GRID_CELL, COL - x0);
            const int ch = std::min(GRID_CELL, ROW - y0);
            if (cw < 5 || ch < 5)
                continue;

            const cv::Rect roi(x0, y0, cw, ch);
            cv::Mat cell_mask = mask(roi);
            if (cv::countNonZero(cell_mask) == 0)
                continue;

            cv::Mat cell_img = img(roi);
            std::vector<cv::Point2f> cell_pts;
            cv::goodFeaturesToTrack(cell_img, cell_pts, 1, 0.01, cell_min_dist, cell_mask);

            cv::Point2f best_pt;
            float best_score = -1.f;
            if (!cell_pts.empty())
            {{
                best_pt = cell_pts[0];
                const int ix = std::min(std::max(cvRound(best_pt.x) + x0, 0), COL - 1);
                const int iy = std::min(std::max(cvRound(best_pt.y) + y0, 0), ROW - 1);
                best_score = eig.at<float>(iy, ix);
            }}
            else
            {{
                cv::Mat gx, gy, grad;
                cv::Sobel(cell_img, gx, CV_32F, 1, 0, 3);
                cv::Sobel(cell_img, gy, CV_32F, 0, 1, 3);
                cv::magnitude(gx, gy, grad);
                grad.setTo(0, cell_mask == 0);
                double max_val = 0;
                cv::Point max_loc;
                cv::minMaxLoc(grad, nullptr, &max_val, nullptr, &max_loc);
                if (max_val <= 2.0)
                    continue;
                best_pt = cv::Point2f(max_loc.x, max_loc.y);
                const int ix = std::min(std::max(max_loc.x + x0, 0), COL - 1);
                const int iy = std::min(std::max(max_loc.y + y0, 0), ROW - 1);
                best_score = static_cast<float>(max_val);
            }}

            candidates.emplace_back(best_score, cv::Point2f(best_pt.x + x0, best_pt.y + y0));
        }}
    }}

    std::sort(candidates.begin(), candidates.end(),
              [](const std::pair<float, cv::Point2f> &a, const std::pair<float, cv::Point2f> &b)
              {{ return a.first > b.first; }});

    const int keep = std::min(need_cnt, static_cast<int>(candidates.size()));
    for (int i = 0; i < keep; ++i)
        n_pts.push_back(candidates[i].second);
}}
""".format(marker=MARKER)

OLD_CALL = "cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);"
NEW_CALL = "detectFeaturesGrid(forw_img, n_max_cnt);"


def patch_cpp(text: str) -> str:
    if MARKER in text:
        return text
    if OLD_CALL not in text:
        raise SystemExit(f"Не найден вызов goodFeaturesToTrack в feature_tracker.cpp")
    if "void FeatureTracker::detectFeaturesGrid" in text:
        text = text.replace(OLD_CALL, NEW_CALL)
        return text
    anchor = "void FeatureTracker::addPoints()\n{\n    for (auto &p : n_pts)\n    {\n        forw_pts.push_back(p);\n        ids.push_back(-1);\n        track_cnt.push_back(1);\n    }\n}"
    if anchor not in text:
        raise SystemExit("Не найдена функция addPoints() — формат файла изменился")
    text = text.replace(anchor, anchor + DETECT_METHOD)
    text = text.replace(OLD_CALL, NEW_CALL)
    return text


def patch_h(text: str) -> str:
    if "detectFeaturesGrid" in text:
        return text
    anchor = "    void setMask();\n\n    void addPoints();"
    if anchor not in text:
        raise SystemExit("Не найден setMask/addPoints в feature_tracker.h")
    return text.replace(anchor, "    void setMask();\n\n    void detectFeaturesGrid(const cv::Mat &img, int need_cnt);\n\n    void addPoints();")


def revert_cpp(text: str) -> str:
    if MARKER not in text and OLD_CALL in text:
        return text
    text = text.replace(NEW_CALL, OLD_CALL)
    start = text.find(f"// {MARKER}")
    if start >= 0:
        end = text.find("\nvoid FeatureTracker::readImage", start)
        if end < 0:
            end = text.find("\nvoid FeatureTracker::rejectWithF", start)
        if end >= 0:
            text = text[:start] + text[end + 1 :]
    return text


def revert_h(text: str) -> str:
    return text.replace(
        "    void setMask();\n\n    void detectFeaturesGrid(const cv::Mat &img, int need_cnt);\n\n    void addPoints();",
        "    void setMask();\n\n    void addPoints();",
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vins", type=Path, required=True, help="Путь к VINS-Mono")
    ap.add_argument("--revert", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    cpp = args.vins / "feature_tracker/src/feature_tracker.cpp"
    h = args.vins / "feature_tracker/src/feature_tracker.h"
    for p in (cpp, h):
        if not p.is_file():
            print(f"Нет файла {p}", file=sys.stderr)
            return 1

    if args.check:
        text = cpp.read_text()
        ok = MARKER in text and NEW_CALL in text
        print("PATCHED" if ok else "NOT PATCHED")
        return 0 if ok else 1

    if args.revert:
        for src, fn in ((cpp, revert_cpp), (h, revert_h)):
            bak = src.with_suffix(src.suffix + ".orig")
            if bak.is_file():
                shutil.copy2(bak, src)
                print(f"restored {src} from {bak.name}")
            else:
                text = fn(src.read_text())
                src.write_text(text)
                print(f"reverted {src}")
        return 0

    for src, fn in ((cpp, patch_cpp), (h, patch_h)):
        bak = src.with_suffix(src.suffix + ".orig")
        if not bak.is_file():
            shutil.copy2(src, bak)
        text = fn(src.read_text())
        src.write_text(text)
        print(f"patched {src}")

    print("OK: grid bucketing applied. Пересоберите: catkin_make --pkg feature_tracker")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
