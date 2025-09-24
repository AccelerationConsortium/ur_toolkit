# Camera Calibration Workflow

This directory contains everything needed for camera calibration to obtain intrinsic parameters for accurate AprilTag pose estimation.

## 📁 Directory Contents

```
camera_calibration/
├── README.md                               # This file
├── Calibration chessboard (US Letter).pdf  # Chessboard pattern for calibration
├── capture_calibration_photos.py           # Step 1: Capture calibration photos
├── calculate_camera_intrinsics.py          # Step 2: Calculate intrinsics
├── camera_calibration.yaml                 # Generated camera intrinsics (after calibration)
└── calibration_photos/                     # Directory for captured photos
    ├── calib_00.jpg                        # Clean calibration images
    ├── calib_01.jpg
    ├── calib_02.jpg
    ├── failed_validation/                  # Failed validation images (for review)
    │   ├── failed_00_20250825_120005.jpg   # Images where corners weren't detected
    │   └── ...
    └── ...
```

## 🚀 Quick Start

### Step 1: Print Chessboard Pattern

1. Print `Calibration chessboard (US Letter).pdf` at **100% scale**
2. Mount on rigid surface (cardboard, clipboard, or foam board)
3. Verify printed square size is **30mm** (measure with ruler)

**Chessboard Source:**
- Pattern from [ARToolKit5 calibration patterns](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(US%20Letter).pdf)
- Standard computer vision calibration pattern
- Optimized for accurate corner detection

**Chessboard Specifications:**
- **8x6 external corners** (squares with corners)
- **7x5 internal corners** (detected by algorithm)
- **30mm square size** (verify after printing)

### Step 2: Capture Calibration Photos

```bash
cd camera_calibration
python capture_calibration_photos.py
```

**Instructions:**
- Hold chessboard at various positions and angles
- Capture **10 photos** with different perspectives
- Ensure good lighting and all corners are visible
- Keep chessboard flat and fully in frame

**Photo Requirements:**
- Different distances (close, medium, far)
- Different angles (tilted, rotated)
- Different positions (center, corners, edges)
- All corners clearly visible
- Good lighting, no glare

### Step 3: Calculate Camera Intrinsics

```bash
python calculate_camera_intrinsics.py
```

This will:
- Process all photos in `calibration_photos/`
- Detect chessboard corners
- Calculate camera matrix and distortion coefficients
- Save results to `# This creates camera_calibration.yaml in the camera_calibration directory`

## 📊 Output

The calibration produces `camera_calibration.yaml` in the camera_calibration directory:

```yaml
calibration_successful: true
camera_matrix:
  - [fx, 0, cx]
  - [0, fy, cy]
  - [0, 0, 1]
distortion_coefficients: [k1, k2, p1, p2, k3]
reprojection_error: 0.245
calibration_quality: "Good"
focal_length_x: 1500.2
focal_length_y: 1498.7
principal_point_x: 960.1
principal_point_y: 540.3
# ... additional metadata
```

## 🎯 Quality Assessment

**Reprojection Error Guidelines:**
- **< 0.5 pixels**: Excellent calibration
- **0.5-1.0 pixels**: Good calibration
- **1.0-2.0 pixels**: Fair calibration
- **> 2.0 pixels**: Poor - recalibrate

**Tips for Better Calibration:**
- Use more photos (15-20 for best results)
- Vary positions and angles more
- Ensure chessboard is perfectly flat
- Use better lighting
- Print chessboard on rigid material

## 🔧 Troubleshooting

### Failed Validation Images

The capture script saves ALL images, even those where chessboard corners couldn't be detected:

- **Valid images**: Saved as `calib_XX.jpg` in `calibration_photos/` 
- **Failed images**: Saved in `calibration_photos/failed_validation/`

**Review failed images to understand issues:**
- Poor lighting or shadows
- Chessboard not fully visible
- Pattern not flat or distorted
- Wrong focus or motion blur

**Clean up after review:**
```bash
# Delete failed validation images when done reviewing
rm -rf calibration_photos/failed_validation/
```

### "No corners detected"
- Check lighting (avoid shadows/glare)
- Ensure chessboard is flat
- Verify entire pattern is visible
- Check focus and image quality

### "Poor calibration quality"
- Capture more photos at different angles
- Ensure chessboard measurements are accurate
- Use better lighting conditions
- Keep chessboard perfectly flat

### "Insufficient valid photos"
- Capture more photos with better conditions
- Vary positions and angles more
- Ensure all corners are visible in each photo

## 🎲 Usage with AprilTag Detection

Once calibrated, use the intrinsics for pose estimation:

```bash
cd ..
python test_apriltag_detection.py --calibration camera_calibration/camera_calibration.yaml
```

The calibration enables:
- Accurate 6DOF pose estimation
- Distance measurements
- 3D coordinate transforms
- Pose visualization

## 📝 Technical Details

### Chessboard Pattern
- **Format**: Standard OpenCV chessboard calibration pattern
- **Source**: [ARToolKit5 calibration patterns](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(US%20Letter).pdf)
- **Layout**: Alternating black and white squares in checkerboard arrangement
- **External corners**: 8 columns × 6 rows = 48 corner points
- **Internal corners**: 7 × 5 = 35 detected corner points (intersections of black/white squares)
- **Square size**: 30mm (measured after printing at 100% scale)
- **Paper size**: US Letter (8.5" × 11")
- **Print settings**: Must use 100% scale, no auto-fit or scaling
- **Material**: Should be printed on matte paper to reduce glare
- **Mounting**: Mount on rigid backing (foam board, cardboard, clipboard) to prevent bending

### Camera Model
- **Pinhole camera model** with radial and tangential distortion
- **Intrinsic parameters**: fx, fy (focal lengths), cx, cy (principal point)
- **Distortion coefficients**: k1, k2, k3 (radial), p1, p2 (tangential)

### Calibration Algorithm
- **OpenCV calibrateCamera()** function
- **Corner detection**: findChessboardCorners() + cornerSubPix()
- **Optimization**: Levenberg-Marquardt algorithm
- **Error metric**: RMS reprojection error in pixels

## 🔄 Recalibration

Recalibrate if:
- Camera lens changes or is refocused
- Camera resolution changes
- Reprojection error > 1.0 pixels
- AprilTag poses seem inaccurate

Simply delete `calibration_photos/` contents and run the workflow again.
