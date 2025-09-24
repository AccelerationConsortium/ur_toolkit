# Visual Servoing Module

This module provides dynamic visual servoing capabilities for robot pick and place operations using AprilTag detection.

## Components

- `visual_servo_engine.py` - Core visual servoing engine with iterative correction
- `pose_history.py` - Pose history management and tracking
- `detection_filter.py` - Noise filtering and median detection algorithms
- `config.py` - Visual servoing configuration and safety limits

## Features

- Iterative pose correction with configurable cycles
- Median filtering for noise reduction
- Dynamic pose storage with history tracking
- Configurable safety limits
- Integration with existing workflow system