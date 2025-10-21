#!/usr/bin/env python3
"""
Center the camera on the first detected AprilTag by cancelling lateral X/Y offsets.

This issues a single conservative moveL based on the empirical mapping `camera_to_robot_mapping.npy`.
"""

import sys
import time
from pathlib import Path
import numpy as np

# Add src
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import rtde_control
import rtde_receive
import cv2
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import (
    config,
    get_apriltag_family,
    get_apriltag_size,
    get_camera_calibration_file,
)


def main():
    robot_ip = config.get("robot.ip_address", "192.168.0.10")
    print("Connecting to robot at", robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    # Camera
    cam_cfg = PiCamConfig(
        hostname=config.get("camera.server.host"), port=config.get("camera.server.port")
    )
    camera = PiCam(cam_cfg)
    if not camera.test_connection():
        print("Cannot connect to camera server")
        return

    detector = AprilTagDetector(
        tag_family=get_apriltag_family(),
        tag_size=get_apriltag_size(),
        calibration_file=get_camera_calibration_file(),
    )

    mapping_file = Path.cwd() / "camera_to_robot_mapping.npy"
    if not mapping_file.exists():
        print("Mapping file missing, run calibration first")
        return
    M = np.load(mapping_file)

    # Get reference pose
    reference_pose = np.array(rtde_r.getActualTCPPose())
    print("Reference pose:", reference_pose)

    # Capture and detect
    photo = camera.capture_photo()
    if not photo:
        print("Failed to capture photo")
        return
    img = cv2.imread(photo)
    dets = detector.detect_tags(img)
    if not dets:
        print("No tags detected")
        return

    tag = dets[0]
    if not tag["pose"]:
        print("No pose available for tag")
        return

    tvec = np.array(tag["pose"]["translation_vector"])  # meters
    print("Tag tvec (camera frame):", tvec)

    # Want to cancel lateral offsets so tag is centered: move camera by -tvec_xy in camera frame
    camera_adjustment = np.array([-tvec[0], -tvec[1], 0.0])
    print("Camera adjustment (m):", camera_adjustment)

    # Map to robot TCP adjustment using empirical mapping (use same sign flip as tests)
    robot_adjustment_xyz = -M.dot(camera_adjustment)
    print("Robot adjustment (m):", robot_adjustment_xyz)

    # Build full 6-DOF adjustment
    camera_pose_adjustment = np.concatenate([robot_adjustment_xyz, np.zeros(3)])
    target_pose = reference_pose + camera_pose_adjustment

    print("Moving to center tag (conservative)...")
    ok = rtde_c.moveL(target_pose.tolist(), 0.02, 0.05)
    print("moveL returned:", ok)

    time.sleep(0.5)
    new_tvec = None
    photo2 = camera.capture_photo()
    if photo2:
        img2 = cv2.imread(photo2)
        dets2 = detector.detect_tags(img2)
        if dets2 and dets2[0]["pose"]:
            new_tvec = np.array(dets2[0]["pose"]["translation_vector"])
            print("New tag tvec:", new_tvec)
            print("New tag distance (mm):", np.linalg.norm(new_tvec) * 1000)

    rtde_c.disconnect()
    rtde_r.disconnect()


if __name__ == "__main__":
    main()
