# AGENTS.md (repo-specific)

## Repo map
- `src/video/vo_final2.py` - main Python monocular VO (downward camera). Config + pipeline in-file (section 1). Heavy on OpenCV/numpy.
- `tools/scenarios/run_all_HKairport03.py` - prepares HKairport03 inputs (camera.json, data/raw/HKairport03/reference/altitude_per_frame.csv, gps_reference.csv) and runs `src/video/vo_final2.py` for `similarity|homography|essential`, benchmarks, plots vs RTK.
- `src/video/vo/vo_cpp/` - C++17 port (simplified): only `FAST-FREAK` + `homography`. Apps: `vo_final`, `run_all_hkairport03`. Uses OpenCV 4.7+ with `opencv_contrib` (FREAK). MSYS2 UCRT64 notes in `src/video/vo/vo_cpp/README.md`.
- `tools/reporting/make_report.py` - builds `docs/reports/VO_HKairport03.pdf` + figures from `results/HKairport03/` and `gps_reference.csv`. `ROOT` is hardcoded to `D:\Cursor_projects\VO`.
- `calibration/`, `camera/`, `gps/`, `imu/`, `lidar/`, `time_sync/`, `other/` - datasets/modules.
- `opencode.json` - MCP `testsprite` configured.
- Kalman/INS fusion is intentionally absent (removed until revisited); do not expect `_kalman_branch/`, `fusion/`, or `--closed-loop-kalman`.

## Critical facts (verified)
- Python VO is single-file with a huge config block (in `src/video/vo_final2.py`). Treat `src/video/vo_final2.py` as source of truth for defaults/behavior; CLI overrides exist.
- Motion model selection: `MOTION_MODEL` in `src/video/vo_final2.py`. `auto` not recommended for nadir. For nadir/rigid camera `homography` is default/typical; `similarity` if stabilized gimbal; `essential` only with real depth/parallax.
- Features: `ALGORITHMS` comma list (e.g. `FAST-FREAK`). `ORB-ORB` can be expensive; heavy descriptors (SIFT/KAZE/BRISK) rarely worth it for nadir constant-height.
- Frame selection: `FRAME_SELECT='adaptive'` with `TARGET_SHIFT_PCT` (default ~10%) is preferred over fixed `FRAME_STEP` for varying speed/height.
- Height is mandatory for metric scale: `ALTITUDE_SOURCE` `const` or `file`. Wrong height/focal gives scale error.
- Camera calibration: `CALIBRATION_FILE` JSON/NPZ/MAT (MAT needs numeric vars). If missing, uses rough approx `focal = APPROXIMATE_FOCAL_FACTOR * max(h,w)` (scale will be off).
- OSD masking: `OSD_MASK` may need filling if telemetry/overlay present (otherwise it causes bias).
- HKairport03 pipeline specifics (from `tools/scenarios/run_all_HKairport03.py`/`src/video/vo/vo_cpp/README.md`): RTK used for altitude per frame and for alignment/eval (first 10% up to ~105s, no global Kabsch to whole track). In C++ port body-yaw/arc integration used; Python path differs.

## Run commands (exact)
Python VO:
```bash
python src/video/vo_final2.py --input video.mp4 --altitude 100
python src/video/vo_final2.py --input frames/ --calibration configs/calibration/camera.json --altitude-file height.csv
python src/video/vo_final2.py --input video.mp4 --motion-model homography
python src/video/vo_final2.py --list
python src/video/vo_final2.py --mode trajectories  # all listed algorithms
```
HKairport03 full run (Python):
```bash
python tools/scenarios/run_all_HKairport03.py  # uses paths relative to repo/HKairport03
python tools/reporting/make_report.py   # needs matplotlib, fpdf, opencv
```
C++ (`src/video/vo/vo_cpp`), MSYS2 UCRT64 on Windows:
```powershell
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/vo_final -i data/raw/HKairport03/camera/images --calibration configs/calibration/camera.json --altitude-file data/raw/HKairport03/reference/altitude_per_frame.csv --out-dir results --no-show-video
./build/run_all_hkairport03 --dataset /path/to/HKairport03 --out .
```

## Gotchas
- `src/video/vo_final2.py` is very long; read config section first before changing behavior.
- For nadir, don't rely on `auto` motion model. Explicitly set `homography` (or `similarity` only if gimbal).
- Metric scale breaks if altitude source wrong or calibration missing.
- OSD overlay in frames will corrupt matching unless `OSD_MASK` set.
- C++ `src/video/vo/vo_cpp` is simplified (only FAST-FREAK+homography) vs full Python `src/video/vo_final2.py`.
- HKairport03 eval is not global scale/pose fit to whole track (alignment on first 10% up to t<=105s) - don't "fix" that.
- OpenCV contrib required for FREAK in C++ build.
