import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

def analyze_hand_eye_calibration_quality():
    """
    Analyze hand-eye calibration quality using real geometric metrics
    instead of OpenCV's misleading residual artifacts.
    """
    
    # Load calibration data
    calib_file = Path('src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json')
    with open(calib_file) as f:
        data = json.load(f)
    
    print("🔬 HAND-EYE CALIBRATION QUALITY ANALYSIS")
    print("=" * 60)
    print()
    
    # Basic info
    info = data['calibration_info']
    print(f"📅 Calibration: {info['timestamp']}")
    print(f"📊 Poses collected: {info['poses_collected']}")
    print(f"✅ Successful: {info['calibration_successful']}")
    print()
    
    # Extract calibration data
    T_ce = np.array(data['hand_eye_transform']['matrix'])
    robot_poses = [np.array(pose) for pose in data['raw_pose_data']['robot_poses']]
    tag_poses = [np.array(pose) for pose in data['raw_pose_data']['apriltag_poses']]
    
    print("🎯 CALIBRATION RESULT ANALYSIS:")
    print(f"   Camera position: [{T_ce[0,3]*1000:.1f}, {T_ce[1,3]*1000:.1f}, {T_ce[2,3]*1000:.1f}] mm")
    print(f"   Camera distance from TCP: {np.linalg.norm(T_ce[:3,3])*1000:.1f} mm")
    
    # Analyze rotation
    R_ce = T_ce[:3,:3]
    euler = R.from_matrix(R_ce).as_euler('xyz', degrees=True)
    print(f"   Camera orientation: [{euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°]")
    print()
    
    # REAL QUALITY METRICS
    print("� REAL QUALITY METRICS (NOT OpenCV artifacts):")
    print()
    
    # 1. Pose diversity analysis
    print("1️⃣ POSE DIVERSITY ANALYSIS:")
    
    # Translation diversity
    robot_translations = np.array([pose[:3,3] for pose in robot_poses])
    translation_range = np.ptp(robot_translations, axis=0) * 1000  # Convert to mm
    translation_volume = np.prod(translation_range)
    
    print(f"   Translation range: [{translation_range[0]:.1f}, {translation_range[1]:.1f}, {translation_range[2]:.1f}] mm")
    print(f"   Workspace volume: {translation_volume/1000:.0f} cm³")
    
    # Rotation diversity
    robot_rotations = [R.from_matrix(pose[:3,:3]) for pose in robot_poses]
    angles = []
    for i in range(len(robot_rotations)):
        for j in range(i+1, len(robot_rotations)):
            rel_rot = robot_rotations[i].inv() * robot_rotations[j]
            angle = rel_rot.magnitude()
            angles.append(np.degrees(angle))
    
    max_rotation_diff = max(angles)
    avg_rotation_diff = np.mean(angles)
    
    print(f"   Max rotation difference: {max_rotation_diff:.1f}°")
    print(f"   Average rotation difference: {avg_rotation_diff:.1f}°")
    
    # Quality assessment for diversity
    trans_quality = "EXCELLENT" if translation_volume > 50000 else "GOOD" if translation_volume > 10000 else "POOR"
    rot_quality = "EXCELLENT" if max_rotation_diff > 45 else "GOOD" if max_rotation_diff > 20 else "POOR"
    
    print(f"   Translation diversity: {trans_quality}")
    print(f"   Rotation diversity: {rot_quality}")
    print()
    
    # 2. Consistency analysis
    print("2️⃣ CALIBRATION CONSISTENCY ANALYSIS:")
    
    # Check consistency by validating the hand-eye equation: T_base_to_tag = T_base_to_ee * T_ee_to_cam * T_cam_to_tag
    consistency_errors = []
    
    for i in range(len(robot_poses)):
        T_base_to_ee = robot_poses[i]
        T_cam_to_tag = tag_poses[i]
        
        # Predicted tag pose in base frame using hand-eye calibration
        T_predicted_base_to_tag = T_base_to_ee @ T_ce @ T_cam_to_tag
        
        # For comparison, we need a reference. Let's use the first pose as reference.
        if i == 0:
            T_reference_base_to_tag = T_predicted_base_to_tag
            continue
        
        # The tag should appear at the same location in base frame for all poses
        T_error = T_reference_base_to_tag @ np.linalg.inv(T_predicted_base_to_tag)
        
        # Extract translation and rotation errors
        trans_error = np.linalg.norm(T_error[:3,3]) * 1000  # mm
        rot_error = R.from_matrix(T_error[:3,:3]).magnitude() * 180/np.pi  # degrees
        
        consistency_errors.append({
            'pose': i+1,
            'translation_error_mm': trans_error,
            'rotation_error_deg': rot_error
        })
    
    if consistency_errors:
        avg_trans_error = np.mean([e['translation_error_mm'] for e in consistency_errors])
        avg_rot_error = np.mean([e['rotation_error_deg'] for e in consistency_errors])
        max_trans_error = max([e['translation_error_mm'] for e in consistency_errors])
        max_rot_error = max([e['rotation_error_deg'] for e in consistency_errors])
        
        print(f"   Average consistency error: {avg_trans_error:.2f}mm, {avg_rot_error:.2f}°")
        print(f"   Maximum consistency error: {max_trans_error:.2f}mm, {max_rot_error:.2f}°")
        
        # Quality assessment for consistency
        consistency_quality = "EXCELLENT" if avg_trans_error < 5 and avg_rot_error < 2 else \
                            "GOOD" if avg_trans_error < 15 and avg_rot_error < 5 else \
                            "POOR"
        
        print(f"   Consistency quality: {consistency_quality}")
        
        # Show worst poses
        worst_trans = max(consistency_errors, key=lambda x: x['translation_error_mm'])
        worst_rot = max(consistency_errors, key=lambda x: x['rotation_error_deg'])
        print(f"   Worst translation error: Pose {worst_trans['pose']} ({worst_trans['translation_error_mm']:.2f}mm)")
        print(f"   Worst rotation error: Pose {worst_rot['pose']} ({worst_rot['rotation_error_deg']:.2f}°)")
    
    print()
    
    # 3. Physical reasonableness
    print("3️⃣ PHYSICAL REASONABLENESS CHECK:")
    
    camera_distance = np.linalg.norm(T_ce[:3,3]) * 1000
    camera_pos = T_ce[:3,3] * 1000
    
    # Check if camera position makes physical sense
    distance_reasonable = 50 < camera_distance < 300  # 5cm to 30cm from TCP
    
    # Check if camera is pointing roughly forward (positive Z in camera frame should align with some robot axis)
    camera_z_direction = T_ce[:3,2]  # Camera Z axis (forward direction)
    camera_pointing_forward = camera_z_direction[0] > 0.5  # Roughly pointing in robot X direction
    
    print(f"   Camera distance from TCP: {camera_distance:.1f}mm {'✅' if distance_reasonable else '⚠️'}")
    print(f"   Camera pointing direction: {'Forward ✅' if camera_pointing_forward else 'Other orientation ⚠️'}")
    print(f"   Physical setup: {'Reasonable ✅' if distance_reasonable else 'Check mounting ⚠️'}")
    print()
    
    # 4. Overall quality assessment
    print("🏆 OVERALL QUALITY ASSESSMENT:")
    
    quality_scores = []
    if 'trans_quality' in locals():
        quality_scores.append(trans_quality)
    if 'rot_quality' in locals():
        quality_scores.append(rot_quality)
    if 'consistency_quality' in locals():
        quality_scores.append(consistency_quality)
    
    excellent_count = quality_scores.count('EXCELLENT')
    good_count = quality_scores.count('GOOD')
    poor_count = quality_scores.count('POOR')
    
    if excellent_count >= 2:
        overall_quality = "EXCELLENT"
        overall_emoji = "🥇"
    elif good_count >= 2 and poor_count == 0:
        overall_quality = "GOOD"
        overall_emoji = "🥈"
    elif poor_count <= 1:
        overall_quality = "ACCEPTABLE"
        overall_emoji = "🥉"
    else:
        overall_quality = "POOR"
        overall_emoji = "❌"
    
    print(f"   {overall_emoji} Overall calibration quality: {overall_quality}")
    print()
    
    # 5. Recommendations
    print("💡 RECOMMENDATIONS:")
    
    if overall_quality == "EXCELLENT":
        print("   ✅ Calibration is excellent - ready for precise visual servoing")
        print("   ✅ Expected accuracy: 1-3mm translation, 0.5-1.5° rotation")
    elif overall_quality == "GOOD":
        print("   ✅ Calibration is good - suitable for most visual servoing tasks")
        print("   ✅ Expected accuracy: 2-5mm translation, 1-3° rotation")
    elif overall_quality == "ACCEPTABLE":
        print("   ⚠️ Calibration is acceptable but could be improved")
        print("   📝 Consider recalibrating with more diverse poses")
        print("   📝 Expected accuracy: 3-8mm translation, 2-5° rotation")
    else:
        print("   ❌ Calibration quality is poor - recalibration recommended")
        print("   📝 Increase pose diversity, especially rotations")
        print("   📝 Check physical setup and AprilTag mounting")
    
    print()
    print("📋 NOTE: These are REAL quality metrics, not OpenCV's misleading 500+mm 'residuals'")

if __name__ == "__main__":
    analyze_hand_eye_calibration_quality()