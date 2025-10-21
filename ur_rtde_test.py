#!/usr/bin/env python3
"""
UR Robot RTDE Test Script
Tests basic moveL commands and AprilTag-based positioning

This script provides a progression from basic RTDE commands to
AprilTag-assisted positioning and visual servoing:

1. Basic moveL testing with RTDE
2. AprilTag pose detection and moveL to detected pose
3. Visual servoing with speedL for precise positioning

Usage:
    python ur_rtde_test.py --robot-ip 192.168.0.10 --test-basic
    python ur_rtde_test.py --robot-ip 192.168.0.10 --test-apriltag
    python ur_rtde_test.py --robot-ip 192.168.0.10 --test-visual-servo
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

try:
    import rtde_control
    import rtde_receive

    RTDE_AVAILABLE = True
except ImportError:
    print("❌ RTDE libraries not available. Install UR RTDE:")
    print("   pip install ur-rtde")
    RTDE_AVAILABLE = False

try:
    from pupil_apriltags import Detector

    APRILTAG_AVAILABLE = True
except ImportError:
    print("❌ pupil-apriltags not available. Install:")
    print("   pip install pupil-apriltags")
    APRILTAG_AVAILABLE = False

# Import camera and detection modules
try:
    from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
    from ur_toolkit.apriltag_detection import AprilTagDetector
    from ur_toolkit.config_manager import (
        get_apriltag_family,
        get_apriltag_size,
        get_camera_calibration_file,
    )

    FULL_SYSTEM_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  Full system not available: {e}")
    FULL_SYSTEM_AVAILABLE = False
    PiCam = None
    PiCamConfig = None
    AprilTagDetector = None

from config_manager import config


class URRTDETest:
    """Test class for UR robot RTDE operations"""

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.rtde_c = None
        self.rtde_r = None

        # Camera parameters for AprilTag detection
        self.camera_params = [500, 500, 320, 240]  # fx, fy, cx, cy
        self.tag_size = 0.023  # 23mm AprilTag

        # Initialize AprilTag detector if available
        self.camera = None
        self.detector = None
        if FULL_SYSTEM_AVAILABLE:
            try:
                # Initialize camera
                camera_config = PiCamConfig()
                self.camera = PiCam(camera_config)

                # Initialize detector
                self.detector = AprilTagDetector(
                    tag_family=get_apriltag_family(),
                    tag_size=get_apriltag_size(),
                    calibration_file=get_camera_calibration_file(),
                )
                print("✅ Camera and detector initialized")
            except Exception as e:
                print(f"⚠️  Camera/detector initialization failed: {e}")
        else:
            print("⚠️  Full system not available - AprilTag features disabled")

    def connect(self):
        """Connect to UR robot via RTDE"""
        if not RTDE_AVAILABLE:
            raise RuntimeError("RTDE libraries not available")

        print(f"🤖 Connecting to UR robot at {self.robot_ip}...")

        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            print("✅ RTDE connection established")
            return True
        except Exception as e:
            print(f"❌ RTDE connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot"""
        if self.rtde_c:
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()
        print("🔌 Disconnected from robot")

    def get_current_pose(self):
        """Get current TCP pose"""
        if not self.rtde_r:
            return None
        pose = self.rtde_r.getActualTCPPose()
        return np.array(pose)

    def print_pose(self, pose, label="Pose"):
        """Print pose in readable format"""
        if pose is None:
            print(f"{label}: None")
            return
        print(
            f"{label}: [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}, "
            f"{pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}]"
        )

    def test_basic_movel(self):
        """Test basic moveL commands"""
        print("\n🧪 Testing Basic moveL Commands")
        print("=" * 40)

        # Get current pose
        current_pose = self.get_current_pose()
        self.print_pose(current_pose, "Current pose")

        # Define test poses (small movements from current position)
        test_poses = [
            current_pose + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0]),  # +5cm in X
            current_pose + np.array([0.0, 0.05, 0.0, 0.0, 0.0, 0.0]),  # +5cm in Y
            current_pose + np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0]),  # +5cm in Z
            current_pose,  # Back to start
        ]

        print("\n📍 Moving to test positions...")

        for i, target_pose in enumerate(test_poses):
            print(f"\nMove {i+1}: Target pose")
            self.print_pose(target_pose)

            try:
                # moveL with default speed/accel
                success = self.rtde_c.moveL(target_pose.tolist(), 0.1, 0.2)
                if success:
                    print("✅ Move completed successfully")
                else:
                    print("⚠️ Move returned False")

                # Wait a moment and check final pose
                time.sleep(0.5)
                final_pose = self.get_current_pose()
                self.print_pose(final_pose, "Final pose")

                # Calculate error but don't use it (for now)
                _ = np.linalg.norm(final_pose - target_pose)

            except Exception as e:
                print(f"❌ Move failed: {e}")

            time.sleep(1)  # Pause between moves

        print("\n✅ Basic moveL test complete")

    def test_apriltag_movel(self):
        """Test AprilTag-based moveL positioning"""
        print("\n🏷️ Testing AprilTag-based moveL")
        print("=" * 40)

        if not FULL_SYSTEM_AVAILABLE or not self.camera or not self.detector:
            print("❌ AprilTag system not available")
            return

        # Test camera connection
        if not self.camera.test_connection():
            print("❌ Cannot connect to camera server")
            return

        print("📷 Capturing image...")
        photo_path = self.camera.capture_photo()
        if not photo_path:
            print("❌ Failed to capture photo")
            return

        # Load image
        import cv2

        image = cv2.imread(photo_path)
        if image is None:
            print("❌ Failed to load image")
            return

        print("🔍 Detecting AprilTags...")
        detections = self.detector.detect_tags(image)

        if not detections:
            print("❌ No AprilTags detected")
            return

        print(f"✅ Detected {len(detections)} AprilTags")

        # Use first detection
        tag = detections[0]
        print(f"🎯 Using Tag ID {tag['tag_id']}")

        if not tag["pose"]:
            print("❌ No pose estimation available (missing calibration)")
            return

        # Get tag pose in camera frame
        tvec = np.array(tag["pose"]["translation_vector"])
        rvec = np.array(tag["pose"]["rotation_vector"])

        print("📍 Tag pose in camera frame:")
        print(
            f"   X: {tvec[0]*1000:.1f}mm, Y: {tvec[1]*1000:.1f}mm, Z: {tvec[2]*1000:.1f}mm"
        )
        # Simple transformation: assume camera is roughly aligned with robot
        # For now, just offset in Z direction (camera looks down)
        # TODO: Use proper hand-eye calibration matrix
        camera_to_robot_offset = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])  # 10cm offset

        tag_pose_camera = np.concatenate([tvec, rvec])
        tag_pose_robot = tag_pose_camera + camera_to_robot_offset

        print("🤖 Tag pose in robot frame (simple offset):")
        self.print_pose(tag_pose_robot)

        # Move to a position above the tag
        approach_offset = np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0])  # 5cm above
        target_pose = tag_pose_robot + approach_offset

        print("🎯 Target pose (5cm above tag):")
        self.print_pose(target_pose)

        # Confirm movement
        response = input("🤖 Execute moveL to tag position? (y/N): ")
        if response.lower() != "y":
            print("❌ Movement cancelled")
            return

        try:
            print("🚀 Moving to tag position...")
            success = self.rtde_c.moveL(target_pose.tolist(), 0.05, 0.1)
            if success:
                print("✅ Move completed successfully")
            else:
                print("⚠️ Move returned False")

            # Check final pose
            time.sleep(0.5)
            final_pose = self.get_current_pose()
            self.print_pose(final_pose, "Final pose")

        except Exception as e:
            print(f"❌ Move failed: {e}")

        print("\n✅ AprilTag-based moveL test complete")

    def test_visual_servo_speedl(self):
        """Test visual servoing with speedL"""
        print("\n👁️ Testing Visual Servoing with speedL")
        print("=" * 40)

        if not FULL_SYSTEM_AVAILABLE or not self.camera or not self.detector:
            print("❌ AprilTag system not available")
            return

        # Test camera connection
        if not self.camera.test_connection():
            print("❌ Cannot connect to camera server")
            return

        print("🎯 Starting visual servoing test...")
        print("   This will move the robot using speedL based on AprilTag detection")

        # Get initial pose
        initial_pose = self.get_current_pose()
        self.print_pose(initial_pose, "Initial pose")

        # Control parameters
        gain = 0.3  # Velocity gain
        max_speed = 0.02  # Maximum speed (m/s)
        tolerance = 0.005  # 5mm tolerance
        max_iterations = 20

        print("🎛️  Control parameters:")
        print(f"   Gain: {gain}")
        print(f"   Max speed: {max_speed*1000:.0f}mm/s")
        print(f"   Tolerance: {tolerance*1000:.0f}mm")
        print(f"   Max iterations: {max_iterations}")

        # Confirm start
        response = input("🤖 Start visual servoing? (y/N): ")
        if response.lower() != "y":
            print("❌ Visual servoing cancelled")
            return

        try:
            for iteration in range(max_iterations):
                print(f"\n🔄 Iteration {iteration + 1}/{max_iterations}")

                # Capture image
                photo_path = self.camera.capture_photo()
                if not photo_path:
                    print("❌ Failed to capture photo")
                    continue

                # Load and detect
                import cv2

                image = cv2.imread(photo_path)
                if image is None:
                    print("❌ Failed to load image")
                    continue

                detections = self.detector.detect_tags(image)
                if not detections:
                    print("❌ No AprilTags detected")
                    continue

                # Use first detection
                tag = detections[0]
                if not tag["pose"]:
                    print("❌ No pose estimation")
                    continue

                # Get tag pose in camera frame
                tvec = np.array(tag["pose"]["translation_vector"])

                print(f"   Tag distance: {np.linalg.norm(tvec)*1000:.1f}mm")
                # For visual servoing, we want to move toward the tag
                # Simple control: move in direction opposite to tag position
                # (negative feedback control)
                velocity_cmd = -gain * tvec

                # Limit velocity
                speed = np.linalg.norm(velocity_cmd[:3])
                if speed > max_speed:
                    velocity_cmd[:3] *= max_speed / speed

                # Only control position, keep orientation fixed
                velocity_cmd[3:] = 0.0

                print(f"   Velocity command: {velocity_cmd[:3]*1000}")
                # Check if we're close enough
                if np.linalg.norm(tvec) < tolerance:
                    print("✅ Converged to target!")
                    break

                # Send velocity command
                print("🚀 Sending speedL command...")
                success = self.rtde_c.speedL(
                    velocity_cmd.tolist(), 1.0, 0.1
                )  # 1s duration

                if not success:
                    print("⚠️ speedL command failed")
                    break

                time.sleep(0.5)  # Wait for movement

            # Stop movement
            print("🛑 Stopping movement...")
            self.rtde_c.speedStop()
            time.sleep(0.2)

            # Check final pose
            final_pose = self.get_current_pose()
            self.print_pose(final_pose, "Final pose")

            displacement = np.linalg.norm(final_pose[:3] - initial_pose[:3])
            print(f"   Total displacement: {displacement*1000:.1f}mm")
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
            self.rtde_c.speedStop()
        except Exception as e:
            print(f"❌ Visual servoing failed: {e}")
            try:
                self.rtde_c.speedStop()
            except Exception:
                pass

        print("\n✅ Visual servoing test complete")

    def run_test(self, test_type):
        """Run specified test"""
        if not self.connect():
            return False

        try:
            if test_type == "basic":
                self.test_basic_movel()
            elif test_type == "apriltag":
                self.test_apriltag_movel()
            elif test_type == "visual-servo":
                self.test_visual_servo_speedl()
            else:
                print(f"❌ Unknown test type: {test_type}")
                return False

        finally:
            self.disconnect()

        return True


def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description="UR Robot RTDE Test Script")
    parser.add_argument(
        "--robot-ip", default=None, help="Robot IP address (overrides config)"
    )
    parser.add_argument(
        "--test-basic", action="store_true", help="Test basic moveL commands"
    )
    parser.add_argument(
        "--test-apriltag", action="store_true", help="Test AprilTag-based positioning"
    )
    parser.add_argument(
        "--test-visual-servo",
        action="store_true",
        help="Test visual servoing with speedL",
    )

    args = parser.parse_args()

    # Get robot IP
    robot_ip = args.robot_ip or config.get("robot.ip_address", "192.168.0.10")

    print("=" * 60)
    print("UR Robot RTDE Test Script")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}")
    print()

    # Determine test type
    test_type = None
    if args.test_basic:
        test_type = "basic"
    elif args.test_apriltag:
        test_type = "apriltag"
    elif args.test_visual_servo:
        test_type = "visual-servo"

    if not test_type:
        print("❌ Please specify a test type:")
        print("   --test-basic        : Test basic moveL commands")
        print("   --test-apriltag     : Test AprilTag positioning")
        print("   --test-visual-servo : Test visual servoing")
        return

    # Run test
    tester = URRTDETest(robot_ip)
    success = tester.run_test(test_type)

    if success:
        print("\n✅ Test completed successfully")
    else:
        print("\n❌ Test failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
