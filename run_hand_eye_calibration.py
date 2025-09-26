#!/usr/bin/env python3
"""
Hand-Eye Calibration Script for UR Toolkit

This script performs automated hand-eye calibration using AprilTags
following Zivid's proven methodology.

Usage:
    python run_hand_eye_calibration.py --robot-ip 192.168.1.100 --tag-ids 0 1 2
"""

import argparse
import sys
from pathlib import Path

# Add src and setup to path for imports
sys.path.append(str(Path(__file__).parent / "src"))
sys.path.append(str(Path(__file__).parent / "setup"))

from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.hand_eye_calibration.hand_eye_calibrator import HandEyeCalibrator
from config_manager import (get_apriltag_family, get_apriltag_size,
                            get_camera_calibration_file)


def main():
    parser = argparse.ArgumentParser(description="Run hand-eye calibration")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--tag-ids", nargs="+", type=int, default=[0], 
                       help="AprilTag IDs to use for calibration")
    parser.add_argument("--num-poses", type=int, default=15,
                       help="Number of calibration poses")
    parser.add_argument("--dry-run", action="store_true",
                       help="Test setup without actually moving robot")
    
    args = parser.parse_args()
    
    print("🤖 UR Toolkit Hand-Eye Calibration")
    print("=" * 50)
    print(f"Robot IP: {args.robot_ip}")
    print(f"AprilTag IDs: {args.tag_ids}")
    print(f"Number of poses: {args.num_poses}")
    
    try:
        # Initialize components
        print("\n🔧 Initializing components...")
        
        # Initialize robot
        robot = URController(args.robot_ip)
        
        # Initialize camera using the same pattern as apriltag_detection.py
        from config_manager import config
        host = config.get('camera.server.host')
        port = config.get('camera.server.port')
        print(f"🔗 Using camera config: {host}:{port}")
        
        camera_config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(camera_config)
        
        # Test camera connection
        if not camera.test_connection():
            print("❌ Failed to connect to camera server")
            return False
        print("✅ Connected to camera server")
        
        # Initialize detector using the same pattern as apriltag_detection.py
        tag_family = get_apriltag_family()
        tag_size = get_apriltag_size()  # Already in meters
        calibration_file = get_camera_calibration_file()
        
        detector = AprilTagDetector(
            tag_family=tag_family,
            tag_size=tag_size,
            calibration_file=calibration_file
        )
        
        print("🏷️  Detector initialized:")
        print(f"   Family: {tag_family}")
        print(f"   Tag size: {tag_size*1000:.1f}mm")
        print(f"   Pose estimation: {'✅' if detector.pose_estimation_enabled else '❌'}")
        
        # Test robot connection (robot connects automatically in constructor)
        print("\n📡 Testing robot connection...")
        try:
            current_pose = robot.get_tcp_pose()
            print(f"✅ Robot connected - Current pose: {current_pose}")
        except Exception as e:
            print(f"❌ Failed to get robot pose: {e}")
            return False
            
        print("✅ All components initialized successfully!")
        
        if args.dry_run:
            print("🧪 Dry run completed - robot connections verified")
            return True
            
        # Create calibrator
        calibrator = HandEyeCalibrator(
            robot_controller=robot,
            camera=camera,
            apriltag_detector=detector,
            tag_ids=args.tag_ids
        )
        
        # Ask for confirmation
        print("\n⚠️  SAFETY WARNING:")
        print(f"   The robot will move to {args.num_poses} different poses")
        print("   Make sure the workspace is clear and safe")
        print(f"   AprilTag(s) {args.tag_ids} should be visible from all poses")
        
        response = input("\nProceed with calibration? (y/N): ").strip().lower()
        if response != 'y':
            print("Calibration cancelled")
            return False
            
        # Run calibration
        success = calibrator.run_automatic_calibration(args.num_poses)
        
        if success:
            print("\n🎉 Hand-eye calibration completed successfully!")
            print("\nThe calibration file has been saved and can now be used")
            print("in your visual servoing system for accurate pose corrections.")
        else:
            print("\n❌ Hand-eye calibration failed")
            
        return success
        
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted by user")
        return False
        
    except Exception as e:
        print(f"\n❌ Calibration failed with error: {e}")
        return False
        
    finally:
        # Cleanup
        try:
            robot.close()
        except Exception:
            pass


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)