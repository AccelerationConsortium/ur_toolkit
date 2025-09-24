# AprilTag Detection Workflow

Simple workflow for using AprilTag detection with robot systems without requiring hand-eye calibration.

## Overview

This workflow provides AprilTag detection in camera coordinates, which can be used for robot control through various approaches:

1. **Direct visual servoing** - Move robot based on tag position in camera view
2. **Fixed camera setup** - Use known camera position relative to robot workspace
3. **Manual coordinate mapping** - Use simple offsets for your specific setup

## Core Components

### 1. Camera System
- Pi Camera server on Raspberry Pi
- Camera calibration for accurate pose estimation
- Remote image capture from robot control PC

### 2. AprilTag Detection
- Robust tag detection using `pupil-apriltags`
- 6DOF pose estimation (position + orientation)
- Distance and quality metrics

### 3. Robot Control
- UR robot interface via RTDE
- Independent pose control
- No coordinate transformation required

## Workflow Steps

### 1. Setup Camera System

```bash
# On Raspberry Pi
curl -sSL https://raw.githubusercontent.com/kelvinchow23/robot_system_tools/master/pi_cam_server/install.sh | bash

# On control PC
python tests/test_camera_capture.py
```

### 2. Calibrate Camera

```bash
cd camera_calibration
python capture_calibration_photos.py
python calculate_camera_intrinsics.py
```

### 3. Test AprilTag Detection

```bash
python tests/test_apriltag_detection.py
```

### 4. Robot Vision Integration

```python
from robots.ur.ur_controller import URController
from camera.picam.picam import PiCam, PiCamConfig
from apriltag_detection import AprilTagDetector
import cv2

# Initialize systems
robot = URController('192.168.0.10')
camera = PiCam(PiCamConfig.from_yaml('camera_client_config.yaml'))
detector = AprilTagDetector(
    tag_family='tag36h11',
    tag_size=0.023,
    camera_calibration_file='camera_calibration/camera_calibration.yaml'
)

# Capture and detect
photo_path = camera.capture_photo()
image = cv2.imread(photo_path)
detections = detector.detect_tags(image)

# Use detection results
for detection in detections:
    tag_id = detection['tag_id']
    distance = detection['distance']
    position = detection['pose']['tvec']  # [x, y, z] in camera frame
    
    print(f"Tag {tag_id}: {distance:.3f}m away")
    print(f"Camera coordinates: {position}")
    
    # Implement your control logic here
    # Examples:
    # - Move closer if distance > threshold
    # - Center tag in view if position[0] > threshold
    # - Approach tag using visual servoing
```

## Control Strategies

### Visual Servoing
Move robot to center AprilTag in camera view:

```python
def center_tag_in_view(robot, detection):
    """Move robot to center AprilTag in camera view"""
    x_offset = detection['pose']['tvec'][0]  # Lateral offset
    
    if abs(x_offset) > 0.01:  # 1cm tolerance
        current_pose = robot.get_tcp_pose()
        # Move robot based on camera offset
        new_pose = current_pose.copy()
        new_pose[1] += x_offset * 0.5  # Scale factor
        robot.move_to_pose(new_pose)
```

### Distance-Based Approach
Move robot closer to or further from target:

```python
def approach_tag(robot, detection, target_distance=0.2):
    """Move robot to maintain target distance from AprilTag"""
    current_distance = detection['distance']
    distance_error = current_distance - target_distance
    
    if abs(distance_error) > 0.02:  # 2cm tolerance
        current_pose = robot.get_tcp_pose()
        new_pose = current_pose.copy()
        new_pose[0] -= distance_error * 0.5  # Move toward/away
        robot.move_to_pose(new_pose)
```

### Fixed Offset Method
Use known camera position relative to robot:

```python
def camera_to_robot_approximate(camera_pos, camera_offset):
    """Convert camera coordinates to robot coordinates using fixed offset"""
    # camera_offset = [dx, dy, dz] from robot TCP to camera
    robot_pos = [
        camera_pos[0] + camera_offset[0],
        camera_pos[1] + camera_offset[1], 
        camera_pos[2] + camera_offset[2]
    ]
    return robot_pos

# Usage
camera_offset = [0.2, 0.0, 0.0]  # Camera is 20cm forward from TCP
robot_position = camera_to_robot_approximate(detection['pose']['tvec'], camera_offset)
```

## Advantages

1. **Simpler setup** - No complex calibration procedure
2. **More robust** - No calibration drift or errors
3. **Easier debugging** - Direct camera coordinates
4. **Flexible** - Works with any camera mounting
5. **Faster development** - Immediate results

## Applications

- **Pick and place** with visual servoing
- **Object tracking** and following
- **Inspection tasks** with distance control
- **Assembly** with tag-guided positioning
- **Navigation** using tag waypoints