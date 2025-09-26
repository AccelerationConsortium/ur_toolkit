# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- **🎯 Hand-Eye Calibration System** - Implemented robust hand-eye calibration based on Zivid's proven methodology
  - `HandEyeCalibrator` class with automated dataset collection and calibration solving
  - Support for AprilTag-based calibration markers  
  - Automatic pose generation with safety validation
  - Quality assessment with translation/rotation residuals
  - Integrated with visual servoing engine for proper coordinate transformations
  - `run_hand_eye_calibration.py` script for automated calibration workflow
  - Comprehensive documentation in `HAND_EYE_CALIBRATION_GUIDE.md`

### Changed
- **🏗️ Major Project Restructure** - Migrated to `src/` layout for better packaging and development
  - Moved all source code to `src/ur_toolkit/`
  - Moved executable scripts to `scripts/`
  - Moved configuration files to `config/`
  - Moved documentation to `docs/` (renamed from `documentation/`)
  - Moved workflow examples to `examples/workflows/`
  - Updated all import paths to use `ur_toolkit.` prefix
  - Created proper `pyproject.toml` for modern Python packaging
  - Updated README.md with new structure and corrected paths
- **Pi Camera Server** - Kept as separate deployment component outside src/ structure
- **🔧 Visual Servoing Improvements** - Fixed coordinate transformation issues
  - Replaced direct error negation with proper hand-eye calibration transformations
  - Added fallback coordinate mapping for systems without calibration
  - Improved pose correction accuracy and stability

### Fixed
- **Visual Servoing Coordinate Frame Issues** - Resolved the core problem causing pose correction failures
  - Previously used `robot_correction = -tag_error` which assumes aligned coordinate frames
  - Now uses proper hand-eye calibration matrix for coordinate transformations
  - Addresses the "metre off" calibration issues mentioned in previous attempts

### Added (Previous)
- **Comprehensive Code Cleanup** - Fixed all flake8 linting errors across the entire codebase (reduced from 336 to 0 errors)
- **Simple Mode Visual Servoing** - Implemented legacy-style translation-only correction mode for improved stability
  - Added `simple_mode` configuration option to enable XY-only corrections
  - Reduced correction complexity to match stable legacy approach from archive/sdl6
  - Fixed proportional gain (0.4) for translation corrections, zero rotation corrections
  - Applied to both direct and observation-based visual servoing methods
  - Improved diagnostic output to clearly show applied corrections in simple mode
  - **Achieved stable convergence** - System now converges reliably even when equipment is moved

### Fixed
- **Disabled Visual Servo Position Updates** - Reverted to always use original taught positions for stability
  - Changed `update_stored_pose` default parameter to `False` 
  - Disabled equipment-wide position updates that were causing drift
  - Added reset test workflow (`reset_test_no_servo.yaml`) to verify original position accuracy
  - System now maintains consistent taught positions without algorithmic corrections

- **Implemented Proper Coordinate Frame Transformation** - Fixed root cause of incorrect correction directions
  - Added `transform_camera_to_robot_correction()` function for proper frame mapping
  - Camera frame (X-right, Y-down, Z-forward) → Robot TCP frame (X-forward, Y-left, Z-up)
  - Replaced simple negation mapping with proper coordinate transformation
  - Robot now moves toward equipment instead of toward base when tag detected further away
  - Eliminated rotation error explosions (6+ radians) through proper frame alignment
  - Enabled proper IBVS with standard PID tuning instead of simplified approach
- **File-by-File Code Cleanup** - Systematically cleaned up individual files including ur_controller.py, config_manager.py, and entire visual_servo/ folder (config.py, detection_filter.py, pose_history.py, visual_servo_engine.py)
- **Import Path Stability** - Added .env file for PYTHONPATH configuration and VS Code settings for consistent imports
- **Flake8 Configuration** - Created .flake8 config file to ignore style-only errors while maintaining functional code quality
- **File Organization** - Deleted all unnecessary debugging/test files and __pycache__ directories
- **Duplicate File Removal** - Removed obsolete ur_robot_interface.py duplicate of ur_controller.py
- **PID-Based Visual Servoing Stability** - Implemented conservative PID controllers for eye-in-hand visual servoing with automatic gain reduction
- **Stability Testing Framework** - Added test script to verify PID controller stability and error tracking behavior
- **Automatic Equipment Association** - All position teaching now automatically prompts for equipment association and observation pose setup
- **Enhanced Position Teaching Workflow** - Unified approach for both regular and freedrive position teaching with equipment linking
- **Improved Equipment Management** - Automatic offset calculation between work positions and observation poses for visual servoing
- **Clean YAML Structure** - Removed irrelevant fields (timestamps, freedrive flags) and AprilTag data from work positions
- **Dual-Purpose Positions** - Work positions can also serve as observation poses when they have AprilTag view and no other observation pose exists
- **Smart AprilTag Data Management** - Only observation poses store AprilTag data; work positions use observation poses for visual servoing
- **Intelligent Prompting** - When AprilTag is detected during position teaching, defaults to dual-purpose position with negative confirmation prompt
- **Comprehensive File Organization** - Moved CHANGELOG.md to documentation/, renamed config/ to setup/, moved requirements.txt and setup_venv.sh to setup/
- **Enhanced Directory Organization** - Created `positions/` directory for position files and moved teach_positions.py there
- **Project Structure Organization** - Created `workflow/` directory for workflow-related files and `workflow/examples/` for YAML examples
- **Path Resolution Fix** - Updated import paths in workflow files to work from subdirectory structure
- **URController Refactor** - Renamed `URRobotInterface` to `URController` and file to `ur_controller.py` for cleaner, more intuitive naming
- **moveJ Action** - Added joint move (`moveJ`) alongside linear move (`moveL`) for different movement types
- **Condensed Workflow YAML** - Removed explicit step names, auto-generate descriptive names, replaced `move_to_position` with `moveL`
- **Step-by-Step Workflow Execution** - Interactive mode with user prompts before each step and automatic delay skipping
- **Improved YAML Formatting** - Inline arrays for coordinates/joints, rounded values, spacing between entries, removed usage tracking
- **Safe Position Reachability Testing** - Automatic movement test after creating safe offset positions to verify accessibility
- **Enhanced Interactive Menu** - Support for both number (1-8) and keyword inputs (teach, list, move, etc.)
- **Simplified Safe Offset System** - Automatic prompting for safe positions after teaching with simple direction/distance input
- **Remote Freedrive Teaching** - Simplified position teaching with blocking input (no timer/threading)
- **Streamlined Interactive Menu** - Removed legacy manual teaching option, focused on remote freedrive workflow
- **Robot Workflow System** - YAML-based workflow execution for sequential robot operations

### Changed
- **Visual Servoing Stability** - Reduced PID gains (kp: 2.0→0.5), removed integral/derivative terms, added conservative output limits
- **Visual Servo Configuration** - Reduced max_iterations (5→3) and damping_factor (0.7→0.3) for enhanced stability
- **Error Tracking** - Added automatic gain reduction when error increases to prevent oscillation and overshoot

### Removed
- **Debugging Test Files** - Deleted test_freedrive_focused.py, test_freedrive_remote.py, test_remote_freedrive_teacher.py
- **Legacy Actions** - Removed `home` and `move_to_position` actions in favor of `moveL`/`moveJ` for cleaner, standardized API
- **URRobotInterface** - Replaced with shorter, more intuitive `URController` class name

### Removed
- **Usage Tracking** - Removed usage_count and last_used fields for cleaner position files
- **Workflow Actions** - Move to position, offset moves, gripper control, delays, position verification
- **Sample Workflows** - Pick and place demo, inspection routines, and simple test workflows
- **Workflow Runner** - Easy-to-use script for executing workflows with safety checks
- **Position Overwrite Confirmation** - Safety prompts when reteaching existing positions to prevent accidental data loss
- **Rich Position Metadata Collection** - Interactive prompts for position description, equipment, type, priority, and safety notes
- **Enhanced Position Teaching Workflow** - Automatic prompting to teach observation poses when no AprilTags detected
- **Observation Pose Auto-Teaching** - Streamlined workflow to teach observation poses immediately after grasp positions
- **Smart Position Linking** - Automatic linking of positions to newly created observation poses
- **Equipment Association** - Mandatory equipment linking for observation poses with validation
- **Manual Position Teaching** - Position teaching using teach pendant freedrive with RTDE pose reading (no remote control commands)
- **Two-Pose Teaching System** - Work poses and observation poses for positions with/without AprilTag visibility
- **Position Linking Commands** - CLI and interactive commands to link positions to observation poses (`link` command)
- **Clean YAML Format** - Human-readable position storage with essential data only (TCP pose, joint angles, AprilTag ID)
- **Read-Only Robot Interface** - Safe RTDE connection that only reads poses, no movement commands
- **Interactive Teaching Mode** - CLI interface for real-time position teaching and management
- **Robotiq Gripper Control** - Socket-based gripper control via URCap port 63352 following RTDE documentation
- Position teaching configuration in unified `config.yaml`
- Position verification using AprilTag visibility detection
- Position management (teach, observe, link, move, verify, list, delete)
- Comprehensive position teaching documentation in `documentation/POSITION_TEACHING.md`
- Enhanced position teaching workflow documentation in `documentation/ENHANCED_POSITION_TEACHING.md`
- Unified configuration system with central `config.yaml` file in project root
- `config_manager.py` module for centralized configuration access
- Configuration guide in `documentation/CONFIGURATION_GUIDE.md`
- Support for environment-based configuration overrides
- Path resolution utilities for robust file path handling
- Convenience functions for common configuration access patterns

### Fixed
- Fixed AprilTag detector initialization error with undefined tag family variable
- Fixed robot interface disconnect method call (should be `close()` not `disconnect()`)
- Improved error handling and validation in AprilTag detector initialization
- Added better diagnostics for AprilTag detector failures
- Fixed camera calibration file loading in AprilTag detector
- Resolved import errors for camera module (`picam`) in CLI and detection modules
- Updated `camera/picam/__init__.py` to properly expose `PiCam` and `PiCamConfig` classes
- Fixed import paths in `apriltag_detection.py` and `teach_positions.py` to use proper Python package structure
- Corrected sys.path management for cross-module imports

### Changed
- Updated `robots/ur/ur_robot_interface.py` to use centralized configuration
- Updated `apriltag_detection.py` to use centralized configuration  
- All modules now import from `config_manager` instead of individual config files
- Configuration values now support dot notation access (e.g., `robot.ip_address`)
- Command-line arguments now override config file values consistently

### Improved
- Simplified configuration management - single source of truth
- Better path handling - no more broken references when moving files
- Consistent configuration access patterns across all modules
- Built-in defaults and graceful fallbacks
- Configuration validation and debugging capabilities

### Removed
- handeye_calibration/ directory and all hand-eye calibration scripts: collect_handeye_data.py, calculate_handeye_calibration.py, coordinate_transformer.py, etc.
- handeye_rework/ directory and experimental calibration approaches
- Hand-eye calibration related files from root: handeye_result.yaml, handeye_samples.json, analyze_motion.py, check_distances.py, test_close_distance.py
- Hand-eye calibration sections from README.md, simplified to focus on pure AprilTag detection workflow
- Individual config files (`camera_client_config.yaml`, `robots/ur/robot_config.yaml`) - now consolidated

### Rationale
- Hand-eye calibration was not producing realistic results
- AprilTag detection works effectively without requiring camera-to-robot transformation
- Simplified codebase focuses on core functionality: camera capture, AprilTag detection, and robot control as separate components
- handeye_rework/collect_samples.py: implemented real robot/camera/apriltag hooks using existing interfaces
- Configuration consolidation eliminates path issues and simplifies system management

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