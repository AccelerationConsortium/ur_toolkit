# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
  - Updated default AprilTag family from `tag36h11` to `tagStandard41h12` (recommended by AprilRobotics)
  - Added `tagStandard41h12` to supported families in AprilTag detection and argument parser
  - Updated all documentation and examples to use the new recommended family
  - Backward compatibility maintained for existing tag families
  - `HandEyeCalibrator` class with automated dataset collection and calibration solving
  - Support for AprilTag-based calibration markers  
  - Automatic pose generation with safety validation
  - Quality assessment with translation/rotation residuals
  - Integrated with visual servoing engine for proper coordinate transformations
  - `run_hand_eye_calibration.py` script for automated calibration workflow
  - Comprehensive documentation in `HAND_EYE_CALIBRATION_GUIDE.md`
  - `PoseCorrectionEngine` using OpenCV solvePnP for stable pose estimation instead of direct AprilTag pose
  - `RobustPoseEstimator` with RANSAC outlier rejection for consistent measurements  
  - `PoseKalmanFilter` for pose smoothing to eliminate noise and oscillations
  - Replaces problematic direct error negation with proper pose transformation mathematics
  - Test script `test_pose_correction.py` for validation
  - Example workflow `pose_correction_test.yaml` demonstrating new approach
  - `linked_position_correction.yaml` workflow specification for observe/grasp operations
  - `run_linked_position_workflow.py` executor script with comprehensive error handling
  - 8-step workflow including validation, movement, pose correction, and linked position updates
  - Integrated with corrected pose correction engine for reliable convergence
  - Support for spatial relationship preservation between linked positions

### Changed
  - Moved all source code to `src/ur_toolkit/`
  - Moved executable scripts to `scripts/`
  - Moved configuration files to `config/`
  - Moved documentation to `docs/` (renamed from `documentation/`)
  - Moved workflow examples to `examples/workflows/`
  - Updated all import paths to use `ur_toolkit.` prefix
  - Created proper `pyproject.toml` for modern Python packaging
  - Updated README.md with new structure and corrected paths
  - Replaced direct error negation with proper hand-eye calibration transformations
  - Added fallback coordinate mapping for systems without calibration
  - Improved pose correction accuracy and stability

### Fixed
  - Added `_apply_coordinate_frame_correction()` method to PoseCorrectionEngine
  - Implements 180° rotation around Y-axis to align tag coordinate frame with expected orientation
  - Applied to both current pose measurements and target pose references
  - Reduced pose correction errors from 200-300° to normal 10-25° range
  - Achieved convergence in 2 iterations vs previous divergence issues
  - Improved success rate from 0% to 20% with excellent stability
  - Previously used `robot_correction = -tag_error` which assumes aligned coordinate frames
  - Now uses proper hand-eye calibration matrix for coordinate transformations
  - Addresses the "metre off" calibration issues mentioned in previous attempts

### Added (Previous)
  - Added `simple_mode` configuration option to enable XY-only corrections
  - Reduced correction complexity to match stable legacy approach from archive/sdl6
  - Fixed proportional gain (0.4) for translation corrections, zero rotation corrections
  - Applied to both direct and observation-based visual servoing methods
  - Improved diagnostic output to clearly show applied corrections in simple mode
  - **Achieved stable convergence** - System now converges reliably even when equipment is moved

### Fixed
  - Changed `update_stored_pose` default parameter to `False` 
  - Disabled equipment-wide position updates that were causing drift
  - Added reset test workflow (`reset_test_no_servo.yaml`) to verify original position accuracy
  - System now maintains consistent taught positions without algorithmic corrections

  - Added `transform_camera_to_robot_correction()` function for proper frame mapping
  - Camera frame (X-right, Y-down, Z-forward) → Robot TCP frame (X-forward, Y-left, Z-up)
  - Replaced simple negation mapping with proper coordinate transformation
  - Robot now moves toward equipment instead of toward base when tag detected further away
  - Eliminated rotation error explosions (6+ radians) through proper frame alignment
  - Enabled proper IBVS with standard PID tuning instead of simplified approach

### Changed

### Removed

### Removed

### Fixed

### Changed

### Improved

### Removed
- handeye_calibration/ directory and all hand-eye calibration scripts: collect_handeye_data.py, calculate_handeye_calibration.py, coordinate_transformer.py, etc.
- handeye_rework/ directory and experimental calibration approaches
- Individual config files (`camera_client_config.yaml`, `robots/ur/robot_config.yaml`) - now consolidated

- AprilTag detection works effectively without requiring camera-to-robot transformation
- Simplified codebase focuses on core functionality: camera capture, AprilTag detection, and robot control as separate components

## [2025-09-12] - Camera Coordinate Frame Correction for Hand-Eye Calibration

### Added
- `tests/test_camera_coordinate_frame.py` - Empirical camera frame mapping test with automatic robot movement
- Coordinate frame correction in `calculate_handeye_calibration.py` 
- Camera-to-robot coordinate transformation matrix based on empirical testing

### Changed
- Updated hand-eye calibration to account for OpenCV camera frame vs robot frame differences
- Robot X+ (RIGHT) → Camera X+, Robot Y+ (BACK) → Camera Z+, Robot Z+ (UP) → Camera Y+
- Applied coordinate transformation: Camera [X,Y,Z] → Robot [X,Z,-Y]

### Fixed
- Hand-eye calibration offset issue (~0.95m → expected 10-30cm) by correcting coordinate frame mismatch
- Camera pose data now properly transformed from OpenCV convention to robot frame before calibration

## [2025-09-11] - Code Organization and Testing Utilities

### Added
- `tests/live_robot_monitor.py` - Real-time robot pose monitoring with quaternion/matrix display
- `tests/rotations_cli.py` - CLI utility for rotation vector analysis and comparison
- `tests/debug_coordinate_frames.py` - Coordinate frame debugging utility

### Changed
- Moved debugging and testing utilities from main directory to `tests/`
- Improved code organization by separating core functionality from testing tools

## [2025-01-20] - TCP Pose Accuracy and Configuration Management

### Added
- Centralized robot configuration via `robots/ur/robot_config.yaml`
- TCP pose sign verification utility (`robots/ur/test_pose_signs.py`)
- Command-line IP override support for all robot scripts
- Configuration loading with fallback to defaults

### Fixed
- Removed incorrect Ry/Rz sign corrections - raw RTDE readings now match teach pendant
- Eliminated hardcoded robot IPs throughout codebase
- TCP pose reading accuracy for reliable hand-eye calibration

### Changed
- `ur_robot_interface.py`: Now loads robot IP and settings from YAML config
- `test_robot_pose.py`: Added config file support with command-line override
- `collect_handeye_data.py`: Added config file support with command-line override
- All robot scripts now use centralized configuration management

### Previous

### Added
- Comprehensive AprilTag pick-and-place workflow documentation in README
- TCP pose reading accuracy verification utility (`robots/ur/test_robot_pose.py`)
- Read-only mode for safer hand-eye calibration data collection
- Improved calibration file path handling to save in proper directories

### Fixed
- Hand-eye calibration TCP pose reading accuracy issues
- Calibration data file paths now save to correct handeye_calibration directory

### Changed
- Hand-eye calibration data collection now uses read-only mode by default
- Updated workflow documentation with immediate testing after each setup step

## [2025-09-05] - Hand-Eye Calibration Improvements

### Summary
- Addressed TCP pose reading accuracy that was affecting hand-eye calibration quality
- Improved safety and workflow for calibration data collection
- Added comprehensive documentation for complete AprilTag workflow

### Next Steps
- Verify TCP pose accuracy using the new test utility
- Re-collect hand-eye calibration data with corrected pose readings
- Continue refinement of calibration quality metrics