#!/usr/bin/env python3
"""
Basic moveL Test Script
Tests fundamental RTDE moveL commands with UR robot
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

import rtde_control
import rtde_receive
from ur_toolkit.config_manager import config


robot_ip = config.get("robot.ip_address", "192.168.0.10")

print("Connecting to UR robot at", robot_ip)

rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
print("RTDE connection established")

# Get current pose
current_pose = np.array(rtde_r.getActualTCPPose())
print("Current pose:", current_pose)

# Define test poses (small movements from current position)
test_poses = [
    current_pose + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0]),  # +5cm in X
    current_pose + np.array([0.0, 0.05, 0.0, 0.0, 0.0, 0.0]),  # +5cm in Y
    current_pose + np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0]),  # +5cm in Z
    current_pose,  # Back to start
]

print("Moving to test positions...")

for i, target_pose in enumerate(test_poses):
    print("Move", i + 1, "Target:", target_pose)

    success = rtde_c.moveL(target_pose.tolist(), 0.1, 0.2)
    if success:
        print("Move completed successfully")
    else:
        print("Move returned False")

    # Check final pose
    time.sleep(0.5)
    final_pose = np.array(rtde_r.getActualTCPPose())
    print("Final pose:", final_pose)

    time.sleep(1)  # Pause between moves

print("Basic moveL test complete")

rtde_c.disconnect()
rtde_r.disconnect()
print("Disconnected from robot")
