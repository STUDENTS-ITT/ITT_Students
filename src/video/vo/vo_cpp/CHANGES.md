# Changes Summary - INS Export Feature

## Date: October 26, 2026

## Task
Modify vo_cpp to export data in a format compatible with add-kalman-report.

## Completed Tasks

### ✅ 1. Created tools/export_for_ins.cpp
- Implemented `export_for_ins()` function
- Exports trajectory.csv data to gps.dat format
- Exports altitude and timestamps
- Exports heading angles to angle.dat format

### ✅ 2. Modified src/pipeline.cpp
- Added `#include "vo/ins_export.hpp"`
- Added export call after `write_gps_track()` in `run_body()`
- Export is conditionally executed based on `cfg.ins_export.enable`

### ✅ 3. Added InsExportCfg to config.hpp
- Added `InsExportCfg` struct with:
  - `bool enable` (default: false)
  - `std::string output_dir` (default: "ins_output")
  - `double frame_rate` (default: 25.0)
- Added `InsExportCfg ins_export` field to `Config` struct

### ✅ 4. Updated apps/vo_final.cpp
- Added CLI flags:
  - `--export-ins`: Enable INS export
  - `--ins-output-dir DIR`: Specify output directory
  - `--ins-fps F`: Specify frame rate for timestamps
- Added argument parsing logic
- Frame rate defaults to `--fps` value if `--ins-fps` not specified

### ✅ 5. Updated CMakeLists.txt
- Added `tools/export_for_ins.cpp` to `VO_SOURCES`

### ✅ 6. Build and Test
- Successfully built with MSYS2 UCRT64
- Tested with sample data from camera/images
- Verified output format:
  - `gps.dat`: timestamp, x, y, altitude (space-separated)
  - `angle.dat`: timestamp, heading (space-separated)

## Files Created
1. `include/vo/ins_export.hpp` - Header file
2. `tools/export_for_ins.cpp` - Implementation
3. `INS_EXPORT_README.md` - User documentation
4. `CHANGES.md` - This file

## Files Modified
1. `include/vo/config.hpp` - Added InsExportCfg struct
2. `src/pipeline.cpp` - Added export call and include
3. `apps/vo_final.cpp` - Added CLI flags
4. `CMakeLists.txt` - Added source file to build

## Testing Performed

```bash
./build/vo_final --input ../camera/images \
                 --altitude 50 \
                 --max-frames 10 \
                 --export-ins \
                 --ins-output-dir ./test_ins_output \
                 --no-show-video
```

**Results:**
- ✅ Build successful (no errors)
- ✅ Files created: gps.dat, angle.dat
- ✅ Correct format: space-separated with headers
- ✅ Correct data: timestamps, coordinates, altitude, heading
- ✅ Console output confirms export completion

## Example Usage

**As specified in requirements:**
```bash
./build/vo_final --export-ins \
                 --ins-output-dir ../add-kalman-report/data/vo_test \
                 --input camera/images \
                 --calibration calibration/camera.json \
                 --altitude-file altitude_per_frame.csv
```

## Output Format

### gps.dat
```
# timestamp x y altitude
0.000000 0.000000 0.000000 50.000000
0.040000 1.234567 2.345678 50.100000
```

### angle.dat
```
# timestamp heading
0.000000 90.000000
0.040000 91.234567
```

## Build System
- **Compiler:** MSYS2 UCRT64 g++
- **Build Type:** Release
- **CMake Generator:** Ninja
- **OpenCV:** 5.0.0 with xfeatures2d

## Notes
- INS export is disabled by default
- Must use `--export-ins` flag to enable
- Output directory is created automatically
- Compatible with add-kalman-report format requirements
