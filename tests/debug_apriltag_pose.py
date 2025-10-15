#!/usr/bin/env python3
"""
Debug AprilTag Pose Measurements
Observe pose readings without corrections to identify coordinate frame issues
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.pose_correction.robust_estimator import RobustPoseEstimator
from ur_toolkit.config_manager import config, get_camera_host, get_camera_port

def rad_to_deg(rad_array):
    """Convert radians to degrees"""
    return np.rad2deg(rad_array)

def unwrap_angles_deg(angles_deg):
    """
    Unwrap angles to handle ±180° wraparound issues
    For angles near ±180°, convert them to a consistent negative range
    """
    unwrapped = np.copy(angles_deg)
    for i in range(len(unwrapped)):
        # If angle is > 90°, wrap to negative equivalent  
        if unwrapped[i] > 90:
            unwrapped[i] = unwrapped[i] - 360
    return unwrapped

def debug_pose_measurements(robot_ip="192.168.0.10", tag_id=2, num_measurements=10):
    """
    Take multiple pose measurements without moving robot to check stability
    """
    print("🔍 AprilTag Pose Measurement Diagnostic")
    print("=" * 50)
    
    # Initialize systems
    print("🔌 Connecting to robot...")
    robot = URController(robot_ip)
    # Robot auto-connects during initialization
    
    print("📷 Connecting to camera...")
    try:
        host = get_camera_host()
        port = get_camera_port()
        print(f"🔗 Camera target: {host}:{port}")
        
        cam_config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(cam_config)
        
        if not camera.test_connection():
            print("❌ Failed to connect to camera")
            return
        print("✅ Camera connected successfully")
    except Exception as e:
        print(f"❌ Camera connection failed: {e}")
        return
    
    print("🏷️  Initializing detector...")
    detector = AprilTagDetector()
    if not detector.pose_estimation_enabled:
        print("❌ Pose estimation not available")
        return
    
    # Initialize robust estimator
    estimator = RobustPoseEstimator(
        camera_matrix=detector.camera_matrix,
        dist_coeffs=detector.dist_coeffs,
        tag_size=detector.tag_size
    )
    
    print(f"\n📍 Current robot TCP pose: {robot.get_tcp_pose()}")
    print(f"🎯 Looking for AprilTag {tag_id}")
    print(f"📊 Taking {num_measurements} measurements...\n")
    
    poses = []
    unwrapped_rotations = []  # Store unwrapped rotation angles for better statistics
    
    for i in range(num_measurements):
        print(f"📸 Measurement {i+1}/{num_measurements}")
        
        # Capture image
        image_path = camera.capture_photo()
        if not image_path:
            print("❌ Failed to capture image")
            continue
            
        # Detect tags
        import cv2
        image = cv2.imread(image_path)
        detections = detector.detect_tags(image)
        
        # Find target tag
        target_detection = None
        for detection in detections:
            if detection['tag_id'] == tag_id:
                target_detection = detection
                break
        
        if target_detection is None:
            print(f"⚠️  Tag {tag_id} not detected")
            continue
        
        # Get pose estimate
        pose_result = estimator.estimate_pose_ransac(target_detection)
        if pose_result is None:
            print("❌ Pose estimation failed")
            continue
            
        rvec, tvec, num_inliers = pose_result
        pose_6dof = estimator.pose_to_6dof(rvec, tvec)
        poses.append(pose_6dof)
        
        # Print current measurement
        pos = pose_6dof[:3]
        rot_deg = rad_to_deg(pose_6dof[3:])
        rot_deg_unwrapped = unwrap_angles_deg(rot_deg)
        unwrapped_rotations.append(rot_deg_unwrapped)  # Store for statistics
        
        print(f"   Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]m")
        print(f"   Rotation: [{rot_deg_unwrapped[0]:.1f}, {rot_deg_unwrapped[1]:.1f}, {rot_deg_unwrapped[2]:.1f}]deg")
        print(f"   Inliers: {num_inliers}/4")
        
        # Check for concerning values
        if np.any(np.abs(rot_deg_unwrapped) > 45):
            print("   ⚠️  Large rotation detected!")
        if pos[2] < 0.1 or pos[2] > 2.0:
            print("   ⚠️  Unusual distance detected!")
            
        print()
        time.sleep(1)  # Brief pause between measurements
    
    # Analysis
    if len(poses) == 0:
        print("❌ No valid poses detected")
        return
        
    poses = np.array(poses)
    print("\n📊 ANALYSIS:")
    print("=" * 30)
    
    # Calculate statistics using unwrapped rotations for better accuracy
    mean_pose = np.mean(poses, axis=0)
    std_pose = np.std(poses, axis=0)
    
    # Calculate rotation statistics from unwrapped angles
    unwrapped_rotations_array = np.array(unwrapped_rotations)
    mean_rot_unwrapped = np.mean(unwrapped_rotations_array, axis=0)
    std_rot_unwrapped = np.std(unwrapped_rotations_array, axis=0)
    
    print(f"📈 Mean pose over {len(poses)} measurements:")
    print(f"   Position: [{mean_pose[0]:.4f}, {mean_pose[1]:.4f}, {mean_pose[2]:.4f}]m")
    print(f"   Rotation: [{mean_rot_unwrapped[0]:.1f}, {mean_rot_unwrapped[1]:.1f}, {mean_rot_unwrapped[2]:.1f}]deg")
    
    print(f"\n📏 Standard deviation (stability):")
    print(f"   Position: [{std_pose[0]:.4f}, {std_pose[1]:.4f}, {std_pose[2]:.4f}]m")
    print(f"   Rotation: [{std_rot_unwrapped[0]:.1f}, {std_rot_unwrapped[1]:.1f}, {std_rot_unwrapped[2]:.1f}]deg")
    
    # Identify issues
    print(f"\n🔍 POTENTIAL ISSUES:")
    if np.any(np.abs(mean_rot_unwrapped) > 45):
        print("❌ Large mean rotations suggest coordinate frame mismatch")
    if np.any(std_rot_unwrapped > 10):
        print("❌ High rotation variability suggests unstable pose estimation")
    if np.any(std_pose[:3] > 0.01):
        print("❌ High position variability suggests detection issues")
    if mean_pose[2] < 0.2:
        print("❌ Very close distance might cause perspective issues")
    if mean_pose[2] > 1.5:
        print("❌ Very far distance might cause detection issues")
        
    # Expected reasonable ranges
    print(f"\n✅ EXPECTED RANGES:")
    print("   Position X: -0.2 to +0.2m (left/right of camera center)")
    print("   Position Y: -0.2 to +0.2m (up/down from camera center)")  
    print("   Position Z: 0.3 to 1.2m (distance from camera)")
    print("   Rotation: -45° to +45° in each axis for normal tag orientation")
    
    try:
        robot.disconnect()
    except:
        pass

if __name__ == "__main__":
    debug_pose_measurements()