# UR Robot Interface

This directory contains all Universal Robots (UR) related code and configuration.

## Files

- `ur_robot_interface.py` - Main robot interface using RTDE
- `test_ur_robot.py` - Test script for robot connection and movement
- `robot_config.yaml` - Robot configuration settings
- `__init__.py` - Python package initialization

## Usage

1. Set up virtual environment:
   ```bash
   ./setup_venv.sh
   ```

2. Activate environment:
   ```bash
   source venv/bin/activate
   ```

3. Test robot connection:
   ```bash
   python robots/ur/test_ur_robot.py --robot-ip 192.168.0.10
   ```

## Configuration

Update `robot_config.yaml` with your robot's IP address and settings.

## Requirements

- UR robot with RTDE enabled
- Robot in Remote Control mode
- Network connectivity to robot
