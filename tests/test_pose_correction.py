#!/usr/bin/env python3
"""
Test Robust Pose Correction Engine
Tests the new visual servoing approach with robust pose estimation and filtering
"""

import argparse
import sys
from pathlib import Path

# Add parent directories for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.pose_correction.pose_correction_engine import PoseCorrectionEngine
from ur_toolkit.config_manager import config, get_camera_host, get_camera_port


def test_pose_correction(robot_ip: str, tag_id: int = 0, dry_run: bool = False):
    """
    Test the robust pose correction system
    
    Args:
        robot_ip: Robot IP address
        tag_id: AprilTag ID to use for testing
        dry_run: If True, only test detection without robot movement
    """
    print("🎯 Testing Robust Pose Correction Engine")
    print("=" * 50)
    
    # Initialize components
    print("\n🔌 Initializing systems...")
    
    try:
        # Initialize robot
        if not dry_run:
            robot = URController(robot_ip)
            if not robot.test_connection():
                print("❌ Robot connection failed")
                return False
            print("✅ Robot connected")
        else:
            robot = None
            print("🔍 Dry run mode - no robot connection")
            
        # Initialize camera
        host = get_camera_host()
        port = get_camera_port()
        camera_config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(camera_config)
        
        if not camera.test_connection():
            print("❌ Camera connection failed")
            return False
        print("✅ Camera connected")
        
        # Initialize AprilTag detector
        detector = AprilTagDetector()
        if not detector.pose_estimation_enabled:
            print("❌ Camera calibration required for pose estimation")
            return False
        print("✅ AprilTag detector initialized with pose estimation")
        
        # Initialize pose correction engine
        if not dry_run:
            pose_engine = PoseCorrectionEngine(robot, camera, detector)
            print("✅ Pose correction engine initialized")
        
        # Test 1: Basic detection and pose estimation
        print(f"\n🔍 Test 1: Detecting AprilTag {tag_id}...")
        image_path = camera.capture_photo()
        if not image_path:
            print("❌ Failed to capture test image")
            return False
            
        import cv2
        image = cv2.imread(image_path)
        detections = detector.detect_tags(image)
        
        target_detection = None
        for detection in detections:
            if detection['tag_id'] == tag_id:
                target_detection = detection
                break
                
        if target_detection is None:
            print(f"❌ Tag {tag_id} not found in image")
            print(f"   Available tags: {[d['tag_id'] for d in detections]}")
            return False
            
        print(f"✅ Tag {tag_id} detected")
        print(f"   Basic pose: {target_detection['pose']['translation_vector']}")
        
        # Test 2: Robust pose estimation
        print("\n🎯 Test 2: Robust pose estimation...")
        from ur_toolkit.pose_correction.robust_estimator import RobustPoseEstimator
        
        robust_estimator = RobustPoseEstimator(
            camera_matrix=detector.camera_matrix,
            dist_coeffs=detector.dist_coeffs,
            tag_size=detector.tag_size
        )
        
        pose_result = robust_estimator.estimate_pose_ransac(target_detection)
        if pose_result is None:
            print("❌ Robust pose estimation failed")
            return False
            
        rvec, tvec, num_inliers = pose_result
        pose_6dof = robust_estimator.pose_to_6dof(rvec, tvec)
        
        print(f"✅ Robust pose estimated with {num_inliers}/4 inliers")
        print(f"   Position: [{pose_6dof[0]:.4f}, {pose_6dof[1]:.4f}, {pose_6dof[2]:.4f}]m")
        print(f"   Rotation: [{pose_6dof[3]:.4f}, {pose_6dof[4]:.4f}, {pose_6dof[5]:.4f}]rad")
        
        # Test 3: Kalman filtering
        print("\n🔧 Test 3: Kalman filtering...")
        from ur_toolkit.pose_correction.kalman_filter import PoseKalmanFilter
        import numpy as np
        
        kalman = PoseKalmanFilter()
        filtered_pose1 = kalman.update(pose_6dof)
        
        # Simulate second measurement with small noise
        noisy_pose = pose_6dof + np.random.normal(0, 0.001, 6)
        filtered_pose2 = kalman.update(noisy_pose)
        
        print(f"✅ Kalman filtering working")
        print(f"   Raw pose:      [{pose_6dof[0]:.4f}, {pose_6dof[1]:.4f}, {pose_6dof[2]:.4f}]")
        print(f"   Filtered pose: [{filtered_pose2[0]:.4f}, {filtered_pose2[1]:.4f}, {filtered_pose2[2]:.4f}]")
        
        if dry_run:
            print("\n✅ Dry run completed successfully")
            print("   All components working correctly")
            print("   Ready for full pose correction test")
            return True
            
        # Test 4: Full pose correction (if not dry run)
        print("\n🎯 Test 4: Setting target pose...")
        if not pose_engine.set_target_from_current_detection(tag_id):
            print("❌ Failed to set target pose")
            return False
            
        print("✅ Target pose set from current detection")
        
        # Ask user if they want to proceed with movement
        print("\n⚠️  The robot will now attempt to correct its pose")
        print("   Make sure the workspace is clear and robot is in a safe position")
        response = input("   Proceed with pose correction? (y/N): ").lower().strip()
        
        if response != 'y':
            print("🛑 Test cancelled by user")
            return True
            
        print("\n🚀 Test 5: Pose correction...")
        success, metrics = pose_engine.correct_to_target(tag_id)
        
        if success:
            print("✅ Pose correction successful!")
            print(f"   Converged in {metrics['iterations']} iterations")
            print(f"   Final error: {metrics['final_error']:.4f}")
        else:
            print("❌ Pose correction failed")
            print(f"   Iterations: {metrics['iterations']}")
            print(f"   Final error: {metrics.get('final_error', 'unknown')}")
            
        return success
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False
    finally:
        # Clean up
        if 'robot' in locals() and robot is not None:
            robot.close()


def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description='Test Robust Pose Correction Engine')
    parser.add_argument('--robot-ip', default=None,
                        help='Robot IP address (overrides config)')
    parser.add_argument('--tag-id', type=int, default=2,
                        help='AprilTag ID to use for testing (default: 2)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Test detection only, no robot movement')
    
    args = parser.parse_args()
    
    robot_ip = args.robot_ip or config.get('robot.ip_address', '192.168.1.100')
    
    print(f"Robot IP: {robot_ip}")
    print(f"Tag ID: {args.tag_id}")
    print(f"Dry run: {args.dry_run}")
    
    success = test_pose_correction(robot_ip, args.tag_id, args.dry_run)
    
    if success:
        print("\n🎉 All tests passed!")
        sys.exit(0)
    else:
        print("\n❌ Tests failed")
        sys.exit(1)


if __name__ == '__main__':
    main()