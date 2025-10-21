#!/usr/bin/env python3
"""
Rotate the robot TCP so the camera faces the detected AprilTag while keeping the same XYZ position.

This computes the desired robot-frame direction from the camera->tag unit vector using the empirical
mapping `camera_to_robot_mapping.npy` (same sign convention as other scripts), then computes a rotation
that aligns the tool Z-axis with that direction and issues a conservative moveL to update orientation.
"""

import sys
from pathlib import Path
import time
import numpy as np
import cv2

# Add src
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import rtde_control
import rtde_receive
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import (
    config,
    get_apriltag_family,
    get_apriltag_size,
    get_camera_calibration_file,
)


def axis_angle_to_rmat(axis, angle):
    axis = axis / np.linalg.norm(axis)
    rvec = axis * angle
    R, _ = cv2.Rodrigues(rvec)
    return R


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

    # Reference pose
    reference_pose = np.array(rtde_r.getActualTCPPose())
    ref_xyz = reference_pose[:3].copy()
    ref_rvec = np.array(reference_pose[3:])
    print("Reference pose:", reference_pose)

    # Capture tag
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
        print("No pose available")
        return

    tvec = np.array(tag["pose"]["translation_vector"])  # camera in tag frame
    cam_to_tag = -tvec
    dist = np.linalg.norm(cam_to_tag)
    if dist < 1e-6:
        print("Tag too close or invalid")
        return
    unit_cam_dir = cam_to_tag / dist
    print("Camera->tag unit vector (camera frame):", unit_cam_dir)

    # Map camera direction to robot-frame direction using empirical mapping.
    # Use same sign flip as earlier scripts.
    robot_dir = -M.dot(unit_cam_dir)
    robot_dir = robot_dir / np.linalg.norm(robot_dir)
    print("Desired robot-frame forward (unit):", robot_dir)

    # Current tool rotation matrix from rotation vector
    R_current, _ = cv2.Rodrigues(ref_rvec)
    tool_z = R_current[:, 2]
    print("Current tool Z (robot frame):", tool_z)

    # Compute rotation to align tool_z -> robot_dir
    dot = np.dot(tool_z, robot_dir)
    dot = np.clip(dot, -1.0, 1.0)
    angle = np.arccos(dot)
    if np.isclose(angle, 0.0):
        print("Already aligned (angle ~ 0). No rotation needed.")
        return

    axis = np.cross(tool_z, robot_dir)
    if np.linalg.norm(axis) < 1e-6:
        print("Axis too small, cannot compute rotation reliably")
        return

    R_delta = axis_angle_to_rmat(axis, angle)
    R_new = R_delta.dot(R_current)
    new_rvec, _ = cv2.Rodrigues(R_new)
    new_rvec = new_rvec.flatten()

    target_pose = np.concatenate([ref_xyz, new_rvec])
    print("Target pose (xyz + rvec):", target_pose)

    # Conservative move
    ok = rtde_c.moveL(target_pose.tolist(), 0.02, 0.05)
    print("moveL result:", ok)

    rtde_c.disconnect()
    rtde_r.disconnect()


if __name__ == "__main__":
    main()
