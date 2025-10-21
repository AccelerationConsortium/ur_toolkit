#!/usr/bin/env python3
"""
Visual Servoing speedL Test Script
Tests closed-loop visual servoing using speedL commands
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
)
import logging
from pathlib import Path as _Path


robot_ip = config.get("robot.ip_address", "192.168.0.10")

print("Connecting to UR robot at", robot_ip)

rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
print("RTDE connection established")

# Initialize camera (use configured PiCam server)
camera_config = PiCamConfig(
    hostname=config.get("camera.server.host"),
    port=config.get("camera.server.port"),
    download_dir=str(
        config.resolve_path(config.get("camera.client.download_directory", "photos"))
    ),
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

# Load empirical camera->robot mapping (required)
mapping_file = Path.cwd() / "camera_to_robot_mapping.npy"
if not mapping_file.exists():
    print(
        f"Required mapping file not found: {mapping_file}\nRun scripts/calibrate_camera_to_robot.py first"
    )
    exit(1)

M = np.load(mapping_file)
print(f"Loaded camera->robot mapping from: {mapping_file}")

# Setup logging
logs_dir = _Path.cwd() / "logs"
logs_dir.mkdir(parents=True, exist_ok=True)
log_file = logs_dir / "visual_servo_last.log"
logger = logging.getLogger("visual_servo")
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(str(log_file), mode="w")
fmt = logging.Formatter("%(asctime)s %(levelname)s: %(message)s")
fh.setFormatter(fmt)
logger.addHandler(fh)
logger.info("Started visual servo script")
logger.info(f"Loaded mapping: {mapping_file}")

# Control parameters (safer defaults)
gain = 0.06  # Velocity gain (further reduced)
max_speed = 0.006  # Maximum speed (m/s) (~6 mm/s)
tolerance = 0.005  # 5mm tolerance
max_iterations = 40
speedl_duration = 0.15  # seconds per speedL command (short bursts)

print("Control parameters:")
print("Gain:", gain)
print("Max speed:", max_speed * 1000, "mm/s")
print("Tolerance:", tolerance * 1000, "mm")
print("Max iterations:", max_iterations)
logger.info(
    f"Control params: gain={gain}, max_speed={max_speed}, tolerance={tolerance}, max_iter={max_iterations}"
)

# Get initial pose
initial_pose = np.array(rtde_r.getActualTCPPose())
print("Initial pose:", initial_pose)
logger.info(f"Initial pose: {initial_pose}")

print("Starting visual servoing...")

# --- Pre-step: move to `pose-B-observe` to simulate a different starting pose ---
# Coordinates taken from src/ur_toolkit/positions/taught_positions.yaml
pose_B_observe = [0.025, -0.422, 0.353, -0.529, -2.253, 1.63]
logger.info("Moving to pose-B-observe to simulate unknown start (slow)")
logger.debug(f"pose-B-observe: {pose_B_observe}")
ok = rtde_c.moveL(pose_B_observe, 0.005, 0.01)
logger.info(f"moveL to B-observe returned: {ok}")
if not ok:
    print("Move to pose-B-observe failed; aborting visual servo test")
    rtde_c.disconnect()
    rtde_r.disconnect()
    exit(1)
time.sleep(0.5)
print("At pose-B-observe, starting visual servo loop toward the AprilTag")

for iteration in range(max_iterations):
    logger.info(f"Iteration {iteration + 1}/{max_iterations}")

    # Capture image
    photo_path = camera.capture_photo()
    if not photo_path:
        print("Failed to capture photo")
        continue

    # Load and detect
    image = cv2.imread(photo_path)
    if image is None:
        print("Failed to load image")
        continue

    detections = detector.detect_tags(image)
    if not detections:
        print("No AprilTags detected")
        continue

    # Use first detection
    tag = detections[0]
    if not tag["pose"]:
        print("No pose estimation")
        continue

    # Get tag pose in camera frame
    tvec = np.array(tag["pose"]["translation_vector"])
    logger.info(f"Tag distance (mm): {np.linalg.norm(tvec) * 1000:.3f}")

    # Check if we're close enough
    if np.linalg.norm(tvec) < tolerance:
        print("Converged to target!")
        break

    # Simple control in camera frame: move toward the tag (camera->tag == -tvec)
    # tvec from detector is tag->camera, so camera->tag = -tvec
    camera_velocity = -gain * tvec

    # Map camera-frame velocity to robot TCP velocity using empirical mapping
    # Apply sign flip consistent with calibration usage
    robot_velocity_xyz = -M.dot(camera_velocity[:3])

    velocity_cmd = np.zeros(6)
    velocity_cmd[:3] = robot_velocity_xyz

    # Limit velocity (clamp robot TCP linear speed)
    lin_speed = np.linalg.norm(velocity_cmd[:3])
    if lin_speed > max_speed:
        velocity_cmd[:3] *= max_speed / lin_speed
    # Also clamp each axis to avoid a single-axis runaway
    for i in range(3):
        if abs(velocity_cmd[i]) > max_speed:
            velocity_cmd[i] = np.sign(velocity_cmd[i]) * max_speed

    # Only control position, keep orientation fixed
    velocity_cmd[3:] = 0.0

    logger.info(f"Velocity command (robot TCP mm/s): {velocity_cmd[:3] * 1000}")

    # Send velocity command in a short burst; catch RTDE errors
    try:
        success = rtde_c.speedL(velocity_cmd.tolist(), speedl_duration, 0.1)
        logger.info(f"speedL returned: {success}")
    except Exception:
        logger.exception("speedL raised exception")
        success = False

    if not success:
        logger.warning("speedL failed or returned False; stopping visual servo loop")
        try:
            rtde_c.speedStop()
        except Exception:
            logger.exception("Error while calling speedStop")
        break

    time.sleep(0.5)  # Wait for movement

# Stop movement
rtde_c.speedStop()
time.sleep(0.2)

# Check final pose
final_pose = np.array(rtde_r.getActualTCPPose())
print("Final pose:", final_pose)

displacement = np.linalg.norm(final_pose[:3] - initial_pose[:3])
print("Total displacement:", displacement * 1000, "mm")
logger.info(f"Final pose (before final move): {final_pose}")
logger.info(f"Total displacement (mm): {displacement * 1000}")

# After convergence, attempt a final precise move to pose-A-observe
pose_A_observe = [-0.135, -0.588, 0.269, 0.006, -2.253, 2.139]
logger.info("Attempting final moveL to pose-A-observe (very slow)")
print("Performing final moveL to pose-A-observe for exact placement (very slow)")
try:
    ok = rtde_c.moveL(pose_A_observe, 0.005, 0.01)
    logger.info(f"moveL to A-observe returned: {ok}")
except Exception:
    logger.exception("Final moveL raised exception")
    ok = False

if not ok:
    logger.warning(
        "Final move to pose-A-observe failed; attempting reconnect and retry"
    )
    try:
        rtde_c.disconnect()
    except Exception:
        logger.exception("Error during disconnect")
    try:
        rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        ok = rtde_c.moveL(pose_A_observe, 0.005, 0.01)
        logger.info(f"retry moveL to A-observe returned: {ok}")
        if ok:
            logger.info("Final move to pose-A-observe succeeded after reconnect")
        else:
            logger.warning("Final move to pose-A-observe still failed after reconnect")
    except Exception:
        logger.exception("Reconnect + moveL attempt failed")

try:
    rtde_c.disconnect()
except Exception:
    logger.exception("Error during final disconnect of control")
try:
    rtde_r.disconnect()
except Exception:
    logger.exception("Error during final disconnect of receive")

logger.info("Disconnected from robot")
print(f"Wrote detailed log to: {log_file}")
