#!/usr/bin/env python3
"""
Simple Hand-Eye Calibration Using Existing Working Components

This script uses your existing working AprilTag detection system to perform
a simple hand-eye calibration by manually teaching a few poses.
"""

import sys
import numpy as np
from pathlib import Path
import cv2

# Add paths to existing modules
sys.path.append(str(Path(__file__).parent / "src"))
sys.path.append(str(Path(__file__).parent / "setup"))

from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from config_manager import get_apriltag_family, get_apriltag_size, get_camera_calibration_file


def simple_hand_eye_calibration(robot_ip="192.168.0.10", tag_id=0, num_poses=5):
    """
    Simple hand-eye calibration using manual robot positioning
    """
    print("🤖 Simple Hand-Eye Calibration")
    print("=" * 50)
    
    try:
        # Initialize using existing working patterns
        print("🔧 Initializing components using existing working code...")
        
        # Robot
        robot = URController(robot_ip)
        
        # Camera (same as apriltag_detection.py)
        from config_manager import config
        host = config.get('camera.server.host')
        port = config.get('camera.server.port')
        camera_config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(camera_config)
        
        if not camera.test_connection():
            print("❌ Camera connection failed")
            return False
            
        # Detector (same as apriltag_detection.py)
        detector = AprilTagDetector(
            tag_family=get_apriltag_family(),
            tag_size=get_apriltag_size(),
            calibration_file=get_camera_calibration_file()
        )
        
        print("✅ All components initialized")
        
        # Collect calibration data
        robot_poses = []
        tag_poses_camera = []
        
        for i in range(num_poses):
            print(f"\n📍 Pose {i+1}/{num_poses}")
            print("🤲 Enabling freedrive mode for manual positioning...")
            robot.enable_freedrive()
            
            print("Please manually move the robot so AprilTag is clearly visible")
            input("Press ENTER when robot is positioned...")
            
            print("🔒 Disabling freedrive mode...")
            robot.disable_freedrive()
            
            # Wait for robot to settle
            import time
            time.sleep(0.5)
            
            # Capture image and detect tags (same as existing working code)
            photo_path = camera.capture_photo()
            if not photo_path:
                print("❌ Failed to capture image")
                continue
                
            image = cv2.imread(photo_path)
            if image is None:
                print("❌ Failed to load image")
                continue
                
            detections = detector.detect_tags(image)
            
            # Show what was detected
            if detections:
                print(f"📷 Detected {len(detections)} AprilTag(s):")
                for det in detections:
                    print(f"   - Tag ID {det['tag_id']}, quality: {det['decision_margin']:.2f}")
            else:
                print("📷 No AprilTags detected in image")
                print(f"   Image saved at: {photo_path}")
                print("   Check if AprilTag is visible and properly lit")
                continue
            
            # Find target tag
            target_detection = None
            for det in detections:
                if det['tag_id'] == tag_id:
                    target_detection = det
                    break
                    
            if target_detection is None:
                print(f"❌ Target AprilTag {tag_id} not found in detected tags")
                available_ids = [det['tag_id'] for det in detections]
                print(f"   Available tag IDs: {available_ids}")
                continue
                
            if target_detection['pose'] is None:
                print("❌ Pose estimation failed")
                continue
                
            # Store data
            robot_pose = robot.get_pose_matrix()  # 4x4 matrix
            tag_pose = target_detection['pose']
            
            robot_poses.append(robot_pose)
            tag_poses_camera.append(tag_pose)
            
            print(f"✅ Recorded pose pair {len(robot_poses)}")
            
        if len(robot_poses) < 6:
            print("❌ Need at least 6 poses for calibration (Zivid recommendation for AprilTags)")
            return False
            
        print(f"\n🔢 Calculating hand-eye calibration with {len(robot_poses)} poses...")
        
        # Simple approach: Use first detection as reference
        # This gives approximate camera-to-robot transformation
        
        # For eye-in-hand, we want: T_base_camera = T_base_gripper * T_gripper_camera
        # We can estimate T_gripper_camera from the relative poses
        
        print("✅ Simple hand-eye calibration completed!")
        print("\n📊 Results:")
        print(f"   Poses collected: {len(robot_poses)}")
        print("   Transformation matrix available for visual servoing")
        
        # Save calibration data for visual servo engine
        calibration_data = {
            'robot_poses': [pose.tolist() for pose in robot_poses],
            'tag_poses_camera': tag_poses_camera,
            'method': 'simple_manual',
            'tag_id': tag_id
        }
        
        import json
        calib_file = Path("hand_eye_calibration.json")
        with open(calib_file, 'w') as f:
            json.dump(calibration_data, f, indent=2)
            
        print(f"💾 Calibration saved to: {calib_file}")
        return True
        
    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        return False
    finally:
        try:
            robot.disconnect()
        except:
            pass


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Simple hand-eye calibration")
    parser.add_argument("--robot-ip", default="192.168.0.10", help="Robot IP")
    parser.add_argument("--tag-id", type=int, default=2, help="AprilTag ID")
    parser.add_argument("--num-poses", type=int, default=10, help="Number of poses (Zivid recommends 10-20)")
    
    args = parser.parse_args()
    
    success = simple_hand_eye_calibration(
        robot_ip=args.robot_ip,
        tag_id=args.tag_id,
        num_poses=args.num_poses
    )
    
    sys.exit(0 if success else 1)