# Hand-Eye Calibration for UR Toolkit

This document explains how to perform hand-eye calibration using the improved approach based on Zivid's proven methodology.

## Why Hand-Eye Calibration is Important

Your previous visual servoing issues were caused by incorrect coordinate frame transformations. Without proper hand-eye calibration, the system was using:

```python
robot_correction = -tag_error  # Wrong - assumes aligned coordinate frames
```

This approach fails because:
1. **Camera and robot coordinate frames are not aligned**
2. **Eye-in-hand mounting introduces complex transformations**
3. **Direct error negation doesn't account for the camera-to-robot relationship**

## The Solution: Proper Hand-Eye Calibration

Hand-eye calibration solves the equation **AX = XB** where:
- **A** = Robot pose changes
- **B** = Camera observation changes  
- **X** = Hand-eye transformation (camera to end-effector)

This gives us the **precise transformation matrix** between camera and robot coordinates.

## Usage

### 1. Run Hand-Eye Calibration

```bash
cd /path/to/ur_toolkit
python run_hand_eye_calibration.py --robot-ip 192.168.1.100 --tag-ids 0 --num-poses 15
```

**Parameters:**
- `--robot-ip`: Your robot's IP address
- `--tag-ids`: AprilTag IDs to use (default: [0])
- `--num-poses`: Number of calibration poses (default: 15)
- `--dry-run`: Test connections without moving robot

### 2. Safety Considerations

⚠️ **IMPORTANT SAFETY NOTES:**
- Ensure workspace is clear before starting
- Robot will move to 15 different poses automatically
- AprilTag must be visible from all poses
- Keep emergency stop accessible
- Verify robot joint limits and collision avoidance

### 3. What Happens During Calibration

1. **Pose Generation**: Creates 15 diverse robot poses around current position
2. **Data Collection**: 
   - Moves robot to each pose
   - Captures image and detects AprilTag
   - Records robot pose and tag detection
3. **Calibration**: Solves AX = XB equation using OpenCV's robust algorithm
4. **Validation**: Computes residuals to assess calibration quality
5. **Storage**: Saves calibration matrix to `src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json`

### 4. Calibration Quality Assessment

**Excellent Quality:**
- Translation error < 5mm
- Rotation error < 2°

**Good Quality:**
- Translation error < 10mm  
- Rotation error < 5°

**Poor Quality (Recalibrate):**
- Translation error > 10mm
- Rotation error > 5°

### 5. Using Calibration in Visual Servoing

Once calibrated, your visual servoing system automatically:

1. **Loads hand-eye calibration** on startup
2. **Transforms tag errors correctly** using the calibration matrix
3. **Applies proper coordinate transformations** instead of direct error negation

**Before (Problematic):**
```python
robot_correction = -tag_error  # Wrong coordinate frame
```

**After (Correct):**
```python
robot_correction = self._transform_tag_error_to_robot_correction(tag_error)
# Uses hand-eye calibration matrix for proper transformation
```

## Troubleshooting

### "Calibration quality may be poor"
- **Cause**: Insufficient pose diversity or detection quality
- **Solution**: Increase `--num-poses` to 20, ensure good lighting
- **Check**: AprilTag clearly visible and well-lit from all poses

### "Failed to detect AprilTag"
- **Cause**: Poor lighting, tag occluded, or wrong tag ID
- **Solution**: Improve lighting, check tag visibility, verify tag IDs
- **Check**: Test detection manually with `debug_apriltag.py`

### "Pose not reachable"
- **Cause**: Generated pose exceeds robot limits
- **Solution**: Start calibration from a more central robot position
- **Check**: Current robot pose allows ±100mm movement in all directions

### "Robot connection failed"
- **Cause**: Network issues or robot not in remote control mode
- **Solution**: Check IP address, ensure robot is in remote control
- **Check**: Can you ping the robot? Is UR software running?

## Expected Results

After successful calibration, your visual servoing should show:

✅ **Improved convergence** - No more oscillation or divergence
✅ **Accurate positioning** - Tag reaches target pose reliably  
✅ **Stable operation** - Consistent results across different starting poses
✅ **Faster settling** - Reduced correction iterations needed

## Files Created

- `src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json` - Main calibration file
- Calibration images and poses (temporary, for debugging)

The calibration file contains:
- 4x4 transformation matrix
- Calibration date and metadata
- Per-pose residuals for quality assessment
- Tag IDs used for calibration

## Next Steps

1. **Run the calibration** following the steps above
2. **Test visual servoing** with your existing workflows
3. **Compare results** - You should see dramatically improved performance
4. **Recalibrate periodically** if camera mount changes or accuracy degrades

The hand-eye calibration approach is based on Zivid's proven methodology and should resolve your visual servoing issues with proper coordinate transformations.