# Quality Metrics and Pose Interpretation Guide

## 📊 Camera Calibration Quality

### Reprojection Error (Pixels)
The reprojection error tells you how accurately your camera calibration can predict where 3D points should appear in 2D images.

| Error Range | Quality | Real-World Accuracy | Use Case |
|-------------|---------|-------------------|----------|
| **< 0.3px** | Excellent | Sub-millimeter precision | High-precision robotics, measurement |
| **0.3-0.5px** | Very Good | ~1-2mm accuracy | General robotics, navigation |
| **0.5-1.0px** | Good | ~2-5mm accuracy | Object detection, rough positioning |
| **1.0-2.0px** | Fair | ~5-10mm accuracy | Basic vision tasks |
| **> 2.0px** | Poor | >10mm error | Unreliable - recalibrate |

**What it means:**
- Lower values = more accurate pose estimation
- Affects distance measurements and 3D positioning
- Critical for precise robot manipulation

## 🏷️ AprilTag Detection Quality

### Decision Margin
Measures how confident the detector is that it found a real AprilTag (vs. false positive).

| Margin Range | Quality | Reliability |
|--------------|---------|-------------|
| **> 50** | Excellent | Very reliable detection |
| **20-50** | Good | Reliable for most uses |
| **10-20** | Fair | Acceptable for basic tasks |
| **< 10** | Poor | May be false positive |

### Hamming Distance
Number of bit errors in the detected tag code.

| Hamming | Quality | Meaning |
|---------|---------|---------|
| **0** | Perfect | No bit errors - most reliable |
| **1-2** | Good | Minor errors - still reliable |
| **> 2** | Poor | Multiple errors - use with caution |

## 📍 Pose Information Explained

### Position (Translation Vector)
Position of the AprilTag relative to the camera in millimeters:

- **X-axis**: Left(-) / Right(+) from camera center
- **Y-axis**: Up(-) / Down(+) from camera center  
- **Z-axis**: Forward(+) / Backward(-) from camera

**Example:**
```
X: +150.0mm  → Tag is 150mm to the right
Y: -50.0mm   → Tag is 50mm above camera center
Z: +500.0mm  → Tag is 500mm in front of camera
```

### Orientation (Rotation)
How the AprilTag is rotated relative to the camera:

- **Roll**: Rotation around X-axis (tilting left/right)
- **Pitch**: Rotation around Y-axis (tilting up/down)
- **Yaw**: Rotation around Z-axis (rotating clockwise/counterclockwise)

**Example:**
```
Roll:  +15.0°  → Tag tilted 15° to the right
Pitch: -10.0°  → Tag tilted 10° upward
Yaw:   +45.0°  → Tag rotated 45° clockwise
```

### Distance
Straight-line distance from camera to AprilTag center:
```
Distance = √(X² + Y² + Z²)
```

## 🎯 Quality Assessment Guidelines

### Excellent Pose Quality
- Hamming = 0
- Decision margin > 50
- **Use for**: Precise robot manipulation, measurement

### Good Pose Quality  
- Hamming = 0
- Decision margin > 20
- **Use for**: Navigation, object tracking, general robotics

### Fair Pose Quality
- Hamming ≤ 1
- Decision margin > 10
- **Use for**: Basic positioning, rough estimates

### Poor Pose Quality
- Hamming > 1 OR decision margin < 10
- **Use with caution**: May have significant errors

## 🔧 Improving Quality

### For Better Calibration:
1. **More photos**: Use 15-20 calibration images
2. **Better conditions**: Good lighting, no shadows
3. **Varied poses**: Different angles and distances
4. **Flat chessboard**: Mount on rigid surface

### For Better AprilTag Detection:
1. **Good lighting**: Even illumination, avoid glare
2. **Sharp focus**: Clear tag edges
3. **Appropriate size**: Tag should be 50-200 pixels wide
4. **Stable mounting**: Minimize motion blur
5. **Clean tags**: High-contrast printing, no damage

## 📐 Coordinate System Reference

```
Camera Coordinate System:
    Y (up)
    ↑
    |
    |
    └──→ X (right)
   ╱
  ╱
 Z (forward into scene)

AprilTag Standard Orientation:
- Tag lying flat on table
- Camera looking down
- Tag's "top" pointing toward camera's +Y axis
```

## 🎲 Practical Applications

### Robot Navigation
- Use Z-distance for approach planning
- Use X,Y for lateral positioning
- Use Yaw for heading alignment

### Object Manipulation
- High-quality poses (Hamming=0, margin>50) for precise grasping
- Position accuracy depends on calibration quality
- Consider tag size vs. distance for optimal precision

### Quality Monitoring
- Log detection metrics over time
- Alert when quality drops below thresholds
- Automatic recalibration triggers
