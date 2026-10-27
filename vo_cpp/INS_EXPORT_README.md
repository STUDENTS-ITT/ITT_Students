# INS Export Feature for vo_cpp

## Overview

Added functionality to export Visual Odometry data in a format compatible with `add-kalman-report` for INS/Kalman filter integration.

## Changes Made

### 1. New Files Created
- **`include/vo/ins_export.hpp`**: Header for INS export functionality
- **`tools/export_for_ins.cpp`**: Implementation of export functions
- **`INS_EXPORT_README.md`**: This documentation file

### 2. Modified Files
- **`include/vo/config.hpp`**: Added `InsExportCfg` struct to configuration
- **`src/pipeline.cpp`**: Added export call after trajectory computation
- **`apps/vo_final.cpp`**: Added CLI flags for INS export
- **`CMakeLists.txt`**: Added export source to build

## Output Format

### gps.dat
Space-separated values with header:
```
# timestamp x y altitude
0.000000 0.000000 0.000000 50.000000
0.040000 1.234567 2.345678 50.100000
...
```
- **timestamp**: Time in seconds (frame_id / fps)
- **x**: East coordinate in meters
- **y**: North coordinate in meters
- **altitude**: Altitude in meters

### angle.dat
Space-separated values with header:
```
# timestamp heading
0.000000 90.000000
0.040000 91.234567
...
```
- **timestamp**: Time in seconds (frame_id / fps)
- **heading**: Heading angle in degrees (0° = North, clockwise)

## Usage

### Command Line Flags

```bash
--export-ins              # Enable INS export
--ins-output-dir DIR      # Output directory (default: ins_output)
--ins-fps F               # Frame rate for timestamps (default: from --fps or 25.0)
```

### Example Commands

**Basic usage:**
```bash
./build/vo_final --input video.mp4 --altitude 100 --export-ins
```

**Full example (as per requirements):**
```bash
./build/vo_final --input camera/images \
                 --calibration calibration/camera.json \
                 --altitude-file altitude_per_frame.csv \
                 --export-ins \
                 --ins-output-dir ../add-kalman-report/data/vo_test \
                 --fps 25 \
                 --out-dir results
```

**With custom output directory:**
```bash
./build/vo_final --input frames/ \
                 --altitude 50 \
                 --export-ins \
                 --ins-output-dir ./my_ins_data \
                 --ins-fps 30
```

## Configuration Structure

### InsExportCfg (in config.hpp)
```cpp
struct InsExportCfg {
    bool enable = false;                    // Enable INS export
    std::string output_dir = "ins_output";  // Output directory for gps.dat and angle.dat
    double frame_rate = 25.0;               // Frame rate for timestamp calculation
};
```

This configuration is part of the main `Config` struct as `ins_export`.

## API Usage

The export function can be called programmatically:

```cpp
#include "vo/ins_export.hpp"

vo::export_for_ins(trajectory_rows, positions, frame_ids, config);
```

**Parameters:**
- `trajectory_rows`: Vector of TrajectoryRow with frame data
- `positions`: Vector of cv::Vec3d with 3D positions (X, Y, Z)
- `frame_ids`: Vector of int with 0-based frame indices
- `config`: Config struct with ins_export settings

## Coordinate System

**Internal ENU (for compatibility):**
- Internal calculations use East-North-Up (ENU) convention
- East stored as `east`, North stored as `north`

**Output Convention (X=North, Z=East, Y=Up):**
- **X-axis**: North (positive = northward)
- **Z-axis**: East (positive = eastward)
- **Y-axis**: Up (altitude relative to first frame)
- **Heading**: 0° = North, increases clockwise (90° = East, 180° = South, 270° = West)

**Plot layout**: bottom axis is Z (east), left axis is X (north). Data stay in ENU (`east`, `north`); only the axis names change.

## Build Instructions

The feature is automatically built with the main project:

```bash
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Testing

A simple test:

```bash
./build/vo_final --input ../camera/images \
                 --altitude 50 \
                 --max-frames 10 \
                 --export-ins \
                 --ins-output-dir ./test_ins_output \
                 --no-show-video
```

Check output:
```bash
cat ./test_ins_output/gps.dat
cat ./test_ins_output/angle.dat
```

## Integration with add-kalman-report

The exported files (`gps.dat` and `angle.dat`) are compatible with `add-kalman-report` data format and can be used directly for INS/Kalman filter fusion experiments.

Place the exported files in the appropriate directory expected by your Kalman filter pipeline:
```bash
../add-kalman-report/data/vo_test/gps.dat
../add-kalman-report/data/vo_test/angle.dat
```

## Notes

- Export is triggered automatically after trajectory computation if `--export-ins` flag is set
- Frame rate defaults to the value from `--fps` if `--ins-fps` is not specified
- Output directory is created automatically if it doesn't exist
- Files are overwritten if they already exist
- The export includes all successfully processed frames from the trajectory
