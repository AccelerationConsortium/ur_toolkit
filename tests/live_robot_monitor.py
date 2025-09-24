#!/usr/bin/env python3
"""
Live Robot Data Monitor
Continuously displays robot joint positions and TCP pose while in freedrive mode
"""

import numpy as np
import time
import sys
import os
from datetime import datetime
sys.path.append(os.path.join(os.path.dirname(__file__), 'robots', 'ur'))
from robots.ur.ur_controller import URController
from scipy.spatial.transform import Rotation as R

# Import our rotation utilities
from archive.rotations_cli import rotvec_to_matrix, rotvec_to_quat


def format_pose_detailed(pose):
    """Format pose with rotation vector, Euler angles, quaternion, and rotation matrix"""
    pos = pose[:3]
    rvec = pose[3:6]

    # Convert to Euler angles (using scipy)
    r = R.from_rotvec(rvec)
    xyz_euler = r.as_euler('xyz', degrees=True)
    zyx_euler = r.as_euler('zyx', degrees=True)

    # Convert using our rotation utilities
    quat = rotvec_to_quat(rvec)  # (x, y, z, w) format
    rot_matrix = rotvec_to_matrix(rvec)

    result = f"Position: [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}] m\n"
    result += f"Rotation Vec: [{rvec[0]:7.4f}, {rvec[1]:7.4f}, {rvec[2]:7.4f}] rad\n"
    result += f"XYZ Euler:    [{xyz_euler[0]:7.2f}, {xyz_euler[1]:7.2f}, {xyz_euler[2]:7.2f}] deg\n"
    result += f"ZYX Euler:    [{zyx_euler[0]:7.2f}, {zyx_euler[1]:7.2f}, {zyx_euler[2]:7.2f}] deg\n"
    result += f"Quaternion:   [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] (x, y, z, w)\n"
    result += "Rotation Matrix:\n"
    for i in range(3):
        result += f"  [{rot_matrix[i, 0]:8.5f}, {rot_matrix[i, 1]:8.5f}, {rot_matrix[i, 2]:8.5f}]\n"

    return result.rstrip()


def format_joints(joints):
    """Format joint positions in radians"""
    return f"[{joints[0]:6.3f}, {joints[1]:6.3f}, {joints[2]:6.3f}, {joints[3]:6.3f}, {joints[4]:6.3f}, {joints[5]:6.3f}] rad"


def clear_screen():
    """Clear terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')


def monitor_robot_live(update_rate=5.0):
    """
    Monitor robot data in real-time

    Args:
        update_rate: Updates per second (Hz)
    """

    print("🤖 Live Robot Data Monitor")
    print("=" * 60)
    print("Put robot in FREEDRIVE mode and move it around!")
    print("Press Ctrl+C to stop monitoring")
    print("=" * 60)

    try:
        # Connect to robot (read-only)
        robot = URController(read_only=True)

        print("\n✅ Connected! Monitoring robot data...")
        print("💡 Try different orientations to see how values change")

        # Store initial values for comparison
        initial_joints = robot.get_joint_positions()
        initial_pose = robot.get_tcp_pose()

        print("\n📍 Initial State:")
        print(f"Joints: {format_joints(initial_joints)}")
        print(f"{format_pose_detailed(initial_pose)}")

        time.sleep(2)

        # Main monitoring loop
        update_interval = 1.0 / update_rate

        while True:
            try:
                # Get current data
                joints = robot.get_joint_positions()
                pose = robot.get_tcp_pose()
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                # Calculate changes from initial
                joint_changes = joints - initial_joints
                pose_changes = pose - initial_pose

                # Clear and update display
                clear_screen()

                print(f"🤖 Live Robot Monitor - {timestamp}")
                print("=" * 60)
                print("📍 Current State:")
                print(f"Joints: {format_joints(joints)}")
                print(f"{format_pose_detailed(pose)}")

                print("\n📊 Changes from Initial:")
                print(f"Joint Δ: [{joint_changes[0]:6.3f}, {joint_changes[1]:6.3f}, {joint_changes[2]:6.3f}, {joint_changes[3]:6.3f}, {joint_changes[4]:6.3f}, {joint_changes[5]:6.3f}] rad")
                print(f"Position Δ: [{pose_changes[0]:7.4f}, {pose_changes[1]:7.4f}, {pose_changes[2]:7.4f}] m")
                print(f"RotVec Δ:   [{pose_changes[3]:7.4f}, {pose_changes[4]:7.4f}, {pose_changes[5]:7.4f}] rad")

                # Calculate angular difference from initial
                if np.linalg.norm(pose_changes[3:6]) > 1e-6:
                    from archive.rotations_cli import rotation_angle_between
                    angular_diff = rotation_angle_between(initial_pose[3:6], pose[3:6], degrees=True)
                    print(f"Angular Δ:  {angular_diff:.2f} degrees")

                print("\n💡 Tips:")
                print("   - Put robot in FREEDRIVE mode")
                print("   - Move TCP to see live changes")
                print("   - Compare rotation matrix with teach pendant")
                print("   - Quaternion values should be consistent")
                print("   - Press Ctrl+C to stop")

                time.sleep(update_interval)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"\n❌ Error reading robot data: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        return

    print("\n👋 Monitoring stopped")


def record_pose_snapshot():
    """Record a single pose snapshot with detailed breakdown"""

    print("📸 Robot Pose Snapshot")
    print("=" * 40)

    try:
        robot = URController(read_only=True)

        joints = robot.get_joint_positions()
        pose = robot.get_tcp_pose()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        print(f"⏰ Timestamp: {timestamp}")
        print("\n🔧 Joint Positions:")
        print(f"   {format_joints(joints)}")
        print(f"   [{joints[0]:7.4f}, {joints[1]:7.4f}, {joints[2]:7.4f}, {joints[3]:7.4f}, {joints[4]:7.4f}, {joints[5]:7.4f}] rad")

        print("\n📍 TCP Pose (Full Analysis):")
        print(f"   {format_pose_detailed(pose)}")

        # Also show rotation vector details
        rvec_magnitude = np.linalg.norm(pose[3:6])
        if rvec_magnitude > 0:
            rvec_axis = pose[3:6] / rvec_magnitude
            print("\n🔄 Rotation Vector Analysis:")
            print(f"   Magnitude: {rvec_magnitude:.4f} rad ({np.degrees(rvec_magnitude):.2f} deg)")
            print(f"   Axis: [{rvec_axis[0]:6.3f}, {rvec_axis[1]:6.3f}, {rvec_axis[2]:6.3f}]")

        # Show comparison format for teach pendant
        print("\n📱 For Teach Pendant Comparison:")
        print(f"   Position (mm): [{pose[0] * 1000:.1f}, {pose[1] * 1000:.1f}, {pose[2] * 1000:.1f}]")
        print(f"   Rotation Vec:  [{pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}] rad")

    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Monitor robot data in real-time")
    parser.add_argument("--snapshot", action="store_true", help="Take single snapshot instead of continuous monitoring")
    parser.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")

    args = parser.parse_args()

    if args.snapshot:
        record_pose_snapshot()
    else:
        monitor_robot_live(args.rate)
