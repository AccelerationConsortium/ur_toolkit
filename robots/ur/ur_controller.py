#!/usr/bin/env python3
"""
Universal Robots Interface for Robot Vision Applications
Provides clean interface for UR robot control via RTDE
"""

import numpy as np
import time
import socket
import rtde_control
import rtde_receive
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Try importing Dashboard Client for freedrive functionality
try:
    import dashboard_client
    import script_client
    DASHBOARD_AVAILABLE = True
except ImportError:
    DASHBOARD_AVAILABLE = False
    print("⚠️  Dashboard client not available - freedrive mode disabled")

# Import centralized configuration
import sys
sys.path.append(str(Path(__file__).parent.parent.parent))
import sys
from pathlib import Path

# Add parent directories to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "setup"))

from config_manager import config


class URController:
    """Universal Robots interface using RTDE"""

    def __init__(self, robot_ip=None, speed=None, acceleration=None, read_only=False, gripper_ip=None, gripper_port=None):
        """
        Initialize UR robot interface

        Args:
            robot_ip: IP address of UR robot (overrides config if provided)
            speed: Default linear speed (m/s) (overrides config if provided)
            acceleration: Default acceleration (m/s²) (overrides config if provided)
            read_only: If True, only connect receive interface (no remote control needed)
            gripper_ip: IP address of gripper for socket-based control (optional)
            gripper_port: Port for gripper socket communication (default 502)
        """
        self.robot_ip = robot_ip or config.get('robot.ip_address', '192.168.0.10')
        self.speed = speed or config.get('robot.default_speed', 0.05)
        self.acceleration = acceleration or config.get('robot.default_acceleration', 0.2)
        self.read_only = read_only

        # Gripper socket configuration (Robotiq URCap port 63352)
        self.gripper_ip = gripper_ip or config.get('gripper.ip_address', self.robot_ip)
        self.gripper_port = gripper_port or config.get('gripper.port', 63352)
        self.gripper_timeout = config.get('gripper.timeout', 5.0)
        self.gripper_socket = None

        print(f"🤖 Connecting to UR robot at {self.robot_ip}...")
        print(f"📋 Using config: speed={self.speed}m/s, accel={self.acceleration}m/s²")
        if self.gripper_ip:
            print(f"🤏 Gripper socket: {self.gripper_ip}:{self.gripper_port}")

        try:
            if not read_only:
                self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            else:
                self.rtde_c = None
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)

            # Initialize Dashboard and Script Clients for freedrive functionality
            if DASHBOARD_AVAILABLE and not read_only:
                try:
                    self.dashboard = dashboard_client.DashboardClient(self.robot_ip)
                    self.dashboard.connect()

                    # Script client requires major/minor version - use common UR5e/UR10e versions
                    # Most modern UR robots use control version 5.x
                    self.script_client = script_client.ScriptClient(self.robot_ip, 5, 11)  # UR 5.11
                    self.script_client.connect()
                    print("✅ Dashboard and Script clients connected")
                except Exception as e:
                    print(f"⚠️  Dashboard/Script client connection failed: {e}")
                    self.dashboard = None
                    self.script_client = None
            else:
                self.dashboard = None
                self.script_client = None

            print("✅ Connected to UR robot")
        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            raise

        # Get initial position for reference
        self.home_pose = self.get_tcp_pose()
        print(f"📍 Current TCP pose: {self.format_pose(self.home_pose)}")

    def set_calibration_speed(self):
        """Set safe speeds for calibration movements to prevent protective stop"""
        self.speed = config.get('robot.calibration_speed', 0.02)
        self.acceleration = config.get('robot.calibration_acceleration', 0.1)
        print(f"🐌 Calibration speeds set: {self.speed * 1000:.0f}mm/s, {self.acceleration * 1000:.0f}mm/s²")

    def get_tcp_pose(self):
        """
        Get current TCP pose in base frame

        Returns:
            np.array: [x, y, z, rx, ry, rz] in meters and radians
        """
        pose = np.array(self.rtde_r.getActualTCPPose())
        return pose

    def get_joint_positions(self):
        """
        Get current joint positions

        Returns:
            np.array: Joint angles in radians
        """
        return np.array(self.rtde_r.getActualQ())

    def joints_to_pose(self, joint_positions):
        """
        Convert joint positions to TCP pose (forward kinematics)

        Args:
            joint_positions: Joint angles in radians [j0, j1, j2, j3, j4, j5]

        Returns:
            np.array: TCP pose [x, y, z, rx, ry, rz] in meters and radians
        """
        if self.rtde_c is None:
            print("❌ Forward kinematics requires control interface (read_only=False)")
            return None

        try:
            pose = np.array(self.rtde_c.getForwardKinematics(joint_positions.tolist()))
            return pose
        except Exception as e:
            print(f"❌ Forward kinematics failed: {e}")
            return None

    def pose_to_joints(self, pose, current_joints=None):
        """
        Convert TCP pose to joint positions (inverse kinematics)

        Args:
            pose: TCP pose [x, y, z, rx, ry, rz] in meters and radians
            current_joints: Current joint positions for closest solution (optional)

        Returns:
            np.array: Joint angles in radians, or None if no solution
        """
        if self.rtde_c is None:
            print("❌ Inverse kinematics requires control interface (read_only=False)")
            return None

        try:
            if current_joints is None:
                current_joints = self.get_joint_positions()

            joints = self.rtde_c.getInverseKinematics(pose.tolist(), current_joints.tolist())

            if joints is None:
                print("❌ No inverse kinematics solution found")
                return None

            return np.array(joints)
        except Exception as e:
            print(f"❌ Inverse kinematics failed: {e}")
            return None

    def move_to_pose(self, pose, speed=None, acceleration=None, wait=True):
        """
        Move robot to specified TCP pose

        Args:
            pose: Target pose [x, y, z, rx, ry, rz]
            speed: Linear speed (m/s), uses default if None
            acceleration: Acceleration (m/s²), uses default if None
            wait: Whether to wait for movement completion

        Returns:
            bool: True if move was successful
        """
        if speed is None:
            speed = self.speed
        if acceleration is None:
            acceleration = self.acceleration

        try:
            self.rtde_c.moveL(pose.tolist(), speed, acceleration, not wait)
            if wait:
                time.sleep(0.1)  # Small delay for stability
            return True
        except Exception as e:
            print(f"❌ Failed to move to pose: {e}")
            return False

    def move_to_joints(self, joint_positions, speed=None, acceleration=None, wait=True):
        """
        Move robot to specified joint positions

        Args:
            joint_positions: Target joint angles in radians
            speed: Joint speed (rad/s), uses default if None
            acceleration: Joint acceleration (rad/s²), uses default if None
            wait: Whether to wait for movement completion

        Returns:
            bool: True if move was successful
        """
        if speed is None:
            speed = self.speed * 2  # Joint moves typically faster
        if acceleration is None:
            acceleration = self.acceleration * 2

        try:
            self.rtde_c.moveJ(joint_positions.tolist(), speed, acceleration, not wait)
            if wait:
                time.sleep(0.1)  # Small delay for stability
            return True
        except Exception as e:
            print(f"❌ Failed to move to joints: {e}")
            return False

    def get_pose_matrix(self, pose=None):
        """
        Convert pose to 4x4 transformation matrix

        Args:
            pose: [x, y, z, rx, ry, rz] pose, uses current if None

        Returns:
            np.array: 4x4 transformation matrix
        """
        if pose is None:
            pose = self.get_tcp_pose()

        # Extract translation and rotation
        translation = pose[:3]
        rotation_vector = pose[3:]

        # Create transformation matrix
        rotation_matrix = R.from_rotvec(rotation_vector).as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = translation

        return transform

    def matrix_to_pose(self, transform_matrix):
        """
        Convert 4x4 transformation matrix to pose

        Args:
            transform_matrix: 4x4 transformation matrix

        Returns:
            np.array: [x, y, z, rx, ry, rz] pose
        """
        translation = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        rotation_vector = R.from_matrix(rotation_matrix).as_rotvec()

        return np.concatenate([translation, rotation_vector])

    def is_at_pose(self, target_pose, position_tolerance=None, rotation_tolerance=None):
        """
        Check if robot is at target pose within tolerance

        Args:
            target_pose: Target pose [x, y, z, rx, ry, rz]
            position_tolerance: Position tolerance in meters (uses config default if None)
            rotation_tolerance: Rotation tolerance in radians (uses config default if None)

        Returns:
            bool: True if at target pose
        """
        if position_tolerance is None:
            position_tolerance = config.get('robot.position_tolerance', 0.001)
        if rotation_tolerance is None:
            rotation_tolerance = config.get('robot.rotation_tolerance', 0.01)

        current_pose = self.get_tcp_pose()

        position_diff = np.linalg.norm(current_pose[:3] - target_pose[:3])
        rotation_diff = np.linalg.norm(current_pose[3:] - target_pose[3:])

        return (position_diff < position_tolerance
                and rotation_diff < rotation_tolerance)

    def stop_motion(self):
        """Emergency stop robot motion"""
        try:
            self.rtde_c.stopL()
            print("🛑 Robot motion stopped")
            return True
        except Exception as e:
            print(f"❌ Failed to stop robot: {e}")
            return False

    def go_home(self):
        """Return robot to home pose"""
        print("🏠 Returning to home pose...")
        return self.move_to_pose(self.home_pose)

    def test_connection(self):
        """
        Test robot connection and basic functionality

        Returns:
            bool: True if connection is working
        """
        try:
            pose = self.get_tcp_pose()
            joints = self.get_joint_positions()

            print("🔍 Connection test:")
            print(f"   TCP Pose: {self.format_pose(pose)}")
            print(f"   Joints: {joints.round(3)} rad")

            return True
        except Exception as e:
            print(f"❌ Connection test failed: {e}")
            return False

    def format_pose(self, pose):
        """Format pose for nice printing"""
        return (f"[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}, "
                f"{pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}]")

    def enable_freedrive(self):
        """
        Enable freedrive mode - allows manual positioning of robot arm

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.dashboard_client:
            print("❌ Dashboard client not available - cannot enable freedrive")
            return False

        try:
            # Use Dashboard Client to unlock protective stop and enable freedrive
            # First make sure robot is not in protective stop
            unlock_result = self.dashboard_client.unlockProtectiveStop()
            print(f"🔓 Unlock protective stop result: {unlock_result}")

            # Enable freedrive mode through Dashboard
            freedrive_result = self.dashboard_client.brakeRelease()
            print(f"🤲 Brake release result: {freedrive_result}")

            print("🤲 Freedrive mode enabled - you can now manually move the robot")
            print("   Use disable_freedrive() when done positioning")
            return True

        except Exception as e:
            print(f"❌ Failed to enable freedrive: {e}")
            return False

    def disable_freedrive(self):
        """
        Disable freedrive mode - returns robot to normal operation

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.dashboard_client:
            print("❌ Dashboard client not available - cannot disable freedrive")
            return False

        try:
            # Use Dashboard Client to close safety popup and re-engage brakes
            close_popup_result = self.dashboard_client.closeSafetyPopup()
            print(f"🔒 Close safety popup result: {close_popup_result}")

            print("🔒 Freedrive mode disabled - robot returned to normal operation")
            print("⚠️  You may need to restart the robot program from the teach pendant")
            return True

        except Exception as e:
            print(f"❌ Failed to disable freedrive: {e}")
            return False

    def manual_position_teaching(self, position_name):
        """
        Interactive manual position teaching using freedrive

        Args:
            position_name: Name for the position being taught

        Returns:
            bool: True if position taught successfully, False otherwise
        """
        print(f"\n🎯 Manual position teaching: {position_name}")
        print("=" * 50)

        if not self.enable_freedrive():
            return False

        try:
            # Wait for user to position robot
            print("📍 Position the robot arm manually to the desired location")
            print("   Press ENTER when the robot is in the correct position...")
            input()

            # Get current position
            current_pose = self.get_tcp_pose()
            current_joints = self.get_joint_positions()

            print("📊 Current position recorded:")
            print(f"   TCP Pose: {self.format_pose(current_pose)}")
            print(f"   Joints: {current_joints.round(3)} rad")

            # Confirm position
            confirm = input("💾 Save this position? (y/N): ").lower().strip()
            if confirm != 'y':
                print("❌ Position teaching cancelled")
                return False

            # Disable freedrive before returning
            self.disable_freedrive()

            return {
                'pose': current_pose,
                'joints': current_joints,
                'name': position_name
            }

        except KeyboardInterrupt:
            print("\n⚠️  Position teaching interrupted")
            self.disable_freedrive()
            return False
        except Exception as e:
            print(f"❌ Position teaching failed: {e}")
            self.disable_freedrive()
            return False

    def close(self):
        """Close robot connection"""
        try:
            if self.rtde_c:
                self.rtde_c.disconnect()
            if self.rtde_r:
                self.rtde_r.disconnect()
            if self.dashboard:
                self.dashboard.disconnect()
            if self.script_client:
                self.script_client.disconnect()

            # Disconnect gripper socket
            self.gripper_disconnect()

            print("🔌 Disconnected from robot")
        except Exception:
            pass

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()

    # ========== Robotiq Gripper Control (Socket-based) ==========

    def gripper_connect(self):
        """Connect to gripper via socket (Robotiq URCap port 63352)"""
        if not self.gripper_ip:
            print("❌ No gripper IP configured")
            return False

        try:
            if self.gripper_socket:
                self.gripper_socket.close()

            self.gripper_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gripper_socket.settimeout(self.gripper_timeout)
            self.gripper_socket.connect((self.gripper_ip, self.gripper_port))
            print(f"✅ Connected to gripper at {self.gripper_ip}:{self.gripper_port}")
            return True

        except Exception as e:
            print(f"❌ Failed to connect to gripper socket: {e}")
            self.gripper_socket = None
            return False

    def gripper_disconnect(self):
        """Disconnect from gripper socket"""
        if self.gripper_socket:
            try:
                self.gripper_socket.close()
                self.gripper_socket = None
                print("🔌 Gripper socket disconnected")
            except Exception:
                pass

    def gripper_send_command(self, command):
        """
        Send command to gripper via socket

        Args:
            command: Command string to send

        Returns:
            str: Response from gripper, or None if failed
        """
        if not self.gripper_socket:
            if not self.gripper_connect():
                return None

        try:
            # Send command
            if isinstance(command, str):
                command = command.encode('utf-8')

            self.gripper_socket.send(command)
            print(f"📤 Sent gripper command: {command}")

            # Read response
            try:
                response = self.gripper_socket.recv(1024).decode('utf-8').strip()
                print(f"📥 Gripper response: {response}")
                return response
            except socket.timeout:
                print("⏱️  No response from gripper (timeout)")
                return None

        except Exception as e:
            print(f"❌ Failed to send gripper command: {e}")
            self.gripper_disconnect()
            return None

    def gripper_activate(self):
        """Activate gripper (complete initialization sequence)"""
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        print("🤖 Activating gripper with full sequence...")

        # Complete Robotiq activation sequence
        # 1. Reset first
        response1 = self.gripper_send_command("SET ACT 0\n")
        if not response1:
            return False
        time.sleep(0.5)  # Reduced from 1.0

        # 2. Activate
        response2 = self.gripper_send_command("SET ACT 1\n")
        if not response2:
            return False
        time.sleep(0.5)  # Reduced from 1.0

        # 3. Set gripper mode (required)
        response3 = self.gripper_send_command("SET GTM 1\n")
        if not response3:
            return False
        time.sleep(0.5)  # Reduced from 1.0

        # 4. Enable Go To command (CRITICAL - enables position commands)
        response4 = self.gripper_send_command("SET GTO 1\n")
        if not response4:
            return False
        time.sleep(1.0)  # Reduced from 2.0

        print("✅ Gripper fully activated and ready for position commands")
        return True

    def gripper_open(self):
        """Open gripper fully"""
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        print("🤲 Opening gripper...")
        # Robotiq open command (position 0)
        response = self.gripper_send_command("SET POS 0\n")
        if response:
            print("✅ Gripper opened")
            return True
        return False

    def gripper_close(self):
        """Close gripper fully"""
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        print("✊ Closing gripper...")
        # Robotiq close command (position 255)
        response = self.gripper_send_command("SET POS 255\n")
        if response:
            print("✅ Gripper closed")
            return True
        return False

    def gripper_set_position(self, position):
        """
        Set gripper to specific position

        Args:
            position: Position (0-255), 0=open, 255=closed
        """
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        position = max(0, min(255, int(position)))
        print(f"🎯 Setting gripper position to {position}...")

        response = self.gripper_send_command(f"SET POS {position}\n")
        if response:
            print(f"✅ Gripper position set to {position}")
            return True
        return False

    def gripper_set_force(self, force):
        """
        Set gripper force

        Args:
            force: Force (0-255)
        """
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        force = max(0, min(255, int(force)))
        print(f"💪 Setting gripper force to {force}...")

        response = self.gripper_send_command(f"SET FOR {force}\n")
        if response:
            print(f"✅ Gripper force set to {force}")
            return True
        return False

    def gripper_set_speed(self, speed):
        """
        Set gripper speed

        Args:
            speed: Speed (0-255)
        """
        if self.read_only:
            print("⚠️  Cannot control gripper in read-only mode")
            return False

        speed = max(0, min(255, int(speed)))
        print(f"⚡ Setting gripper speed to {speed}...")

        response = self.gripper_send_command(f"SET SPE {speed}\n")
        if response:
            print(f"✅ Gripper speed set to {speed}")
            return True
        return False

    def gripper_get_status(self):
        """
        Get gripper status

        Returns:
            dict: Gripper status information
        """
        print("📊 Getting gripper status...")
        response = self.gripper_send_command("GET POS\n")

        if response:
            try:
                # Parse response - format may vary by gripper model
                status = {'raw_response': response}
                return status
            except Exception as e:
                print(f"❌ Failed to parse gripper status: {e}")
                return None
        return None

    def gripper_is_moving(self):
        """Check if gripper is currently moving"""
        status = self.gripper_get_status()
        if status:
            # Implementation depends on gripper response format
            return False  # Placeholder
        return False


def main():
    """Test the UR robot interface"""
    import argparse

    parser = argparse.ArgumentParser(description='UR Robot Interface Test')
    parser.add_argument('--robot-ip', default=None,
                        help='Robot IP address (overrides config)')
    parser.add_argument('--test-move', action='store_true',
                        help='Perform small test movement')
    parser.add_argument('--test-gripper', action='store_true',
                        help='Test gripper functionality')

    args = parser.parse_args()

    print("🤖 UR Robot Interface Test")
    print("=" * 50)

    try:
        robot_ip = args.robot_ip or config.get('robot.ip_address', '192.168.0.10')

        with URController(robot_ip) as robot:
            if not robot.test_connection():
                return

            if args.test_move:
                print("\n🔄 Performing test movement...")

                current_pose = robot.get_tcp_pose()
                test_pose = current_pose.copy()
                test_pose[2] += 0.01

                print(f"Moving to: {robot.format_pose(test_pose)}")
                if robot.move_to_pose(test_pose):
                    time.sleep(1)

                    print("Returning to original position...")
                    robot.move_to_pose(current_pose)
                    print("✅ Test movement completed")
            else:
                print("❌ Test movement failed")

        # Test gripper if requested
        if args.test_gripper:
            print("\n🤲 Testing gripper functionality...")

            # Test activation
            if robot.gripper_activate():
                print("✅ Gripper activation successful")

                # Test open
                time.sleep(1)
                if robot.gripper_open():
                    print("✅ Gripper open successful")

                    # Test close
                    time.sleep(2)
                    if robot.gripper_close():
                        print("✅ Gripper close successful")

                        # Test status
                        time.sleep(1)
                        status = robot.gripper_get_status()
                        if status:
                            print(f"✅ Gripper status: {status}")

                        # Open again for safety
                        time.sleep(1)
                        robot.gripper_open()
                        print("✅ Gripper test completed - gripper opened for safety")
                    else:
                        print("❌ Gripper close failed")
                else:
                    print("❌ Gripper open failed")
            else:
                print("❌ Gripper activation failed")

        print("\n🎉 UR Robot interface test completed!")

    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")


if __name__ == "__main__":
    main()
