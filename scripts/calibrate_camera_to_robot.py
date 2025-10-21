#!/usr/bin/env python3
"""
Simple empirical calibration: map camera-frame observed deltas (from AprilTag) to robot TCP deltas.

Creates a 3x3 linear mapping M where robot_delta ≈ M @ camera_delta

Usage: python scripts/calibrate_camera_to_robot.py
"""

import sys
import time
from pathlib import Path

import numpy as np
import cv2

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import rtde_control
import rtde_receive
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import (
    get_camera_host,
    get_camera_port,
    get_camera_calibration_file,
    get_apriltag_family,
    get_apriltag_size,
    config,
)


def capture_first_tvec(camera, detector, attempts=3, delay=0.5):
    for _ in range(attempts):
        photo = camera.capture_photo()
        if not photo:
            time.sleep(delay)
            continue
        img = cv2.imread(photo)
        dets = detector.detect_tags(img)
        if dets and dets[0]["pose"]:
            return np.array(dets[0]["pose"]["translation_vector"])
        time.sleep(delay)
    return None


def main():
    robot_ip = config.get("robot.ip_address", "192.168.0.10")
    print("Connecting to robot at", robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    speed = 0.03
    accel = 0.08

    cam_cfg = PiCamConfig(hostname=get_camera_host(), port=get_camera_port())
    camera = PiCam(cam_cfg)

    if not camera.test_connection():
        print("Cannot connect to camera server")
        return

    detector = AprilTagDetector(
        tag_family=get_apriltag_family(),
        tag_size=get_apriltag_size(),
        calibration_file=get_camera_calibration_file(),
    )

    print("Collecting baseline pose and tag observation...")
    reference_pose = np.array(rtde_r.getActualTCPPose())
    base_tvec = capture_first_tvec(camera, detector, attempts=8)
    if base_tvec is None:
        print("Failed to detect tag for baseline - abort")
        return

    print("Baseline tvec:", base_tvec)

    # Define small moves in robot TCP frame (meters)
    step = 0.01  # 1cm steps
    deltas = [
        np.array([step, 0.0, 0.0]),
        np.array([-step, 0.0, 0.0]),
        np.array([0.0, step, 0.0]),
        np.array([0.0, -step, 0.0]),
        np.array([0.0, 0.0, step]),
        np.array([0.0, 0.0, -step]),
    ]

    cam_deltas = []
    robot_deltas = []

    for d in deltas:
        target = reference_pose.copy()
        target[:3] += d

        print(f"Moving TCP by {d} m")
        rtde_c.moveL(target.tolist(), speed, accel)
        time.sleep(0.6)

        moved_pose = np.array(rtde_r.getActualTCPPose())
        robot_delta = moved_pose[:3] - reference_pose[:3]

        new_tvec = capture_first_tvec(camera, detector, attempts=6)
        if new_tvec is None:
            print("Warning: tag not seen after move; skipping sample")
            # move back then continue
            rtde_c.moveL(reference_pose.tolist(), speed, accel)
            time.sleep(0.4)
            continue

        # observed camera->tag movement (camera->tag = -tvec)
        cam_delta = base_tvec - new_tvec

        print("  robot_delta:", robot_delta)
        print("  cam_delta:", cam_delta)

        cam_deltas.append(cam_delta)
        robot_deltas.append(robot_delta)

        # Move back to reference
        rtde_c.moveL(reference_pose.tolist(), speed, accel)
        time.sleep(0.4)

    if len(cam_deltas) < 3:
        print("Not enough samples for calibration")
        return

    C = np.vstack(cam_deltas)  # N x 3
    R = np.vstack(robot_deltas)  # N x 3

    # Solve R = C @ M.T  -> M.T = lstsq(C, R)
    sol, *_ = np.linalg.lstsq(C, R, rcond=None)
    M = sol.T

    print("Computed mapping M (robot_delta ≈ M @ cam_delta):")
    print(M)

    # Show residuals
    residuals = R - (C @ M.T)
    err_norm = np.linalg.norm(residuals, axis=1)
    print("Per-sample residual norms (m):", err_norm)

    # Save to file
    out = Path.cwd() / "camera_to_robot_mapping.npy"
    np.save(out, M)
    print("Saved mapping to:", out)


if __name__ == "__main__":
    main()
