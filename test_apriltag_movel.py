#!/usr/bin/env python3
"""
AprilTag-based moveL Test Script
Tests AprilTag detection and moveL to detected tag position
"""

import sys
import time
import numpy as np
import cv2
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

import rtde_control
import rtde_receive
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import (
    config,
    get_apriltag_family,
    get_apriltag_size,
    get_camera_calibration_file,
    get_camera_host,
    get_camera_port,
    get_photos_directory,
)


robot_ip = config.get("robot.ip_address", "192.168.0.10")

print("Connecting to UR robot at", robot_ip)

rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
print("RTDE connection established")

# Movement parameters
speed = 0.05  # m/s
accel = 0.1  # m/s²

# Initialize camera
camera_config = PiCamConfig(
    hostname=get_camera_host(),
    port=get_camera_port(),
    download_dir=str(get_photos_directory()),
)
camera = PiCam(camera_config)

if not camera.test_connection():
    print("Cannot connect to camera server")
    exit(1)

print("Camera connected")

# Initialize detector
detector = AprilTagDetector(
    tag_family=get_apriltag_family(),
    tag_size=get_apriltag_size(),
    calibration_file=get_camera_calibration_file(),
)

print("Detector initialized")

# Get current robot pose (this will be our reference)
reference_pose = np.array(rtde_r.getActualTCPPose())
print("Reference pose:", reference_pose)

# Capture image and detect AprilTag
photo_path = camera.capture_photo()
if not photo_path:
    print("Failed to capture photo")
    exit(1)

print("Captured photo:", photo_path)

image = cv2.imread(photo_path)
detections = detector.detect_tags(image)

if not detections:
    print("No AprilTags detected")
    exit(1)

print("Detected", len(detections), "AprilTags")

# Use first detection
tag = detections[0]
print("Using Tag ID", tag["tag_id"])

if not tag["pose"]:
    print("No pose estimation available")
    exit(1)

# Get tag pose in camera frame
tvec = np.array(tag["pose"]["translation_vector"])
distance = np.linalg.norm(tvec)
print("Tag tvec:", tvec)
print("Tag distance:", distance * 1000, "mm")

# Calculate movement direction (toward the tag)
# Note: tvec is position of camera in tag frame, so -tvec points from camera to tag
direction = -tvec / distance  # Unit vector toward tag
print("Direction (unit vector):", direction)
move_distance = 0.02  # 2cm toward tag

# Calculate position adjustment in camera frame
camera_adjustment = direction * move_distance  # Move toward tag

# Try to load empirical camera->robot mapping saved by calibration script
mapping_file = Path.cwd() / "camera_to_robot_mapping.npy"
if mapping_file.exists():
    try:
        M = np.load(mapping_file)
        # Apply mapping: robot_delta = M @ cam_delta
        # If sign/handedness is inverted, try negative mapping
        robot_adjustment_xyz = -M.dot(camera_adjustment)
        print(f"Loaded mapping from {mapping_file} (applied with sign flip)")
    except Exception as e:
        print(f"Failed to load mapping ({e}), using fallback transform")
        robot_adjustment_xyz = np.array(
            [camera_adjustment[1], -camera_adjustment[0], camera_adjustment[2]]
        )
else:
    # Fallback heuristic (previous ad-hoc 90° rotation)
    robot_adjustment_xyz = np.array(
        [camera_adjustment[1], -camera_adjustment[0], camera_adjustment[2]]
    )

camera_pose_adjustment = np.concatenate([robot_adjustment_xyz, np.zeros(3)])

# Simple transformation to robot frame (camera is close to gripper, minimal offset)
camera_to_robot_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No offset needed
robot_adjustment = camera_pose_adjustment + camera_to_robot_offset

# Calculate target pose (move toward tag)
target_pose = reference_pose + robot_adjustment
print(f"Moving {move_distance * 100:.0f}cm toward tag...")
print("Target pose:", target_pose)

# Execute move toward tag
success = rtde_c.moveL(target_pose.tolist(), speed, accel)

if success:
    print("Move toward tag completed successfully")
else:
    print("Move toward tag returned False")

# Capture new image to verify movement
print("Capturing new image to verify movement...")
new_photo_path = camera.capture_photo()
if new_photo_path:
    new_image = cv2.imread(new_photo_path)
    if new_image is not None:
        new_detections = detector.detect_tags(new_image)
        if new_detections:
            new_tag = new_detections[0]  # Assume same tag
            if new_tag["pose"]:
                new_tvec = np.array(new_tag["pose"]["translation_vector"])
                new_distance = np.linalg.norm(new_tvec)
                print(f"New distance to tag: {new_distance * 1000:.1f}mm")
                distance_change = distance - new_distance
                if distance_change > 0:
                    print(
                        f"✅ Successfully moved closer by {distance_change * 1000:.1f}mm!"
                    )
                else:
                    print(f"❌ Moved farther away by {-distance_change * 1000:.1f}mm")
                # Debug: compute observed movement projection along commanded camera->tag direction
                # tvec is camera position in tag frame, so camera->tag vector is -tvec
                old_cam_to_tag = -tvec
                new_cam_to_tag = -new_tvec
                observed_move_cam = new_cam_to_tag - old_cam_to_tag
                proj_along_cmd = np.dot(observed_move_cam, direction)
                print(
                    f"Observed movement along commanded direction: {proj_along_cmd * 1000:.1f}mm (positive = moved closer)"
                )
            else:
                print("No pose estimation in new image")
        else:
            print("No AprilTags detected in new image")
    else:
        print("Failed to load new image")
else:
    print("Failed to capture new image")

# Check position after move
time.sleep(0.5)
moved_pose = np.array(rtde_r.getActualTCPPose())
print("Pose after move:", moved_pose)

# Move back to reference position
print("Moving back to reference position...")
success = rtde_c.moveL(reference_pose.tolist(), speed, accel)

if success:
    print("Move back completed successfully")
else:
    print("Move back returned False")

# Final check
time.sleep(0.5)
final_pose = np.array(rtde_r.getActualTCPPose())
print("Final pose:", final_pose)

displacement = np.linalg.norm(final_pose[:3] - reference_pose[:3])
print("Total displacement from reference:", displacement * 1000, "mm")

rtde_c.disconnect()
rtde_r.disconnect()
print("Disconnected from robot")
