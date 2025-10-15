#!/usr/bin/env python3
"""
Fix 180° Ambiguity in Taught Positions
Corrects AprilTag pose ambiguity in camera_to_tag values and observation offsets
"""

import yaml
import numpy as np
from pathlib import Path


def correct_apriltag_180_ambiguity(pose_vector):
    """Correct 180° rotation ambiguity in AprilTag pose estimation.
    
    AprilTags can have 180° ambiguity around Y-axis due to coordinate frame conventions.
    This function detects and corrects poses that are ~180° off from expected orientation.
    
    Args:
        pose_vector: [x, y, z, rx, ry, rz] pose vector
        
    Returns:
        Corrected pose vector
    """
    corrected_pose = pose_vector.copy()
    
    # Check if Y-rotation is close to ±180° (π radians)
    ry = pose_vector[4]  # Y-rotation component
    
    # If Y-rotation magnitude is > 90° (π/2), likely 180° ambiguity
    if abs(ry) > np.pi / 2:
        # Correct by flipping around Y-axis
        if ry > 0:
            corrected_pose[4] = ry - np.pi  # Subtract 180°
        else:
            corrected_pose[4] = ry + np.pi  # Add 180°
            
        # Also flip X and Z rotations to maintain consistency
        corrected_pose[3] = -corrected_pose[3]  # Flip RX
        corrected_pose[5] = -corrected_pose[5]  # Flip RZ
        
        print(f"🔄 Corrected 180° ambiguity: Y-rot {np.degrees(ry):.1f}° → {np.degrees(corrected_pose[4]):.1f}°")
    
    return corrected_pose


def fix_taught_positions_ambiguity(input_file: str, output_file: str):
    """
    Fix 180° ambiguity in taught positions file
    
    Args:
        input_file: Path to original taught positions YAML file
        output_file: Path to save corrected taught positions YAML file
    """
    print(f"🔧 Loading taught positions from: {input_file}")
    
    # Load the YAML file
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)
    
    corrections_made = 0
    positions_processed = 0
    
    print("\n📊 Analyzing positions for 180° ambiguity...")
    
    # Process each position
    for position_name, position_data in data.get('positions', {}).items():
        positions_processed += 1
        position_corrected = False
        
        # Check camera_to_tag poses
        if 'camera_to_tag' in position_data and position_data['camera_to_tag']:
            original_pose = np.array(position_data['camera_to_tag'])
            ry_original = original_pose[4]
            
            # Check if Y-rotation has 180° ambiguity
            if abs(ry_original) > np.pi / 2:
                print(f"\n🎯 Position '{position_name}':")
                print(f"   Original camera_to_tag Y-rotation: {np.degrees(ry_original):.1f}°")
                
                corrected_pose = correct_apriltag_180_ambiguity(original_pose)
                position_data['camera_to_tag'] = corrected_pose.tolist()
                
                corrections_made += 1
                position_corrected = True
                
                print(f"   ✅ Corrected camera_to_tag Y-rotation: {np.degrees(corrected_pose[4]):.1f}°")
        
        # Check observation_offset poses (if they exist)
        if 'observation_offset' in position_data and position_data['observation_offset']:
            offset = np.array(position_data['observation_offset'])
            if len(offset) >= 6:  # Has rotation components
                ry_offset = offset[4]
                
                # Check if Y-rotation offset has large values that might indicate ambiguity
                if abs(ry_offset) > np.pi:  # Larger threshold for offsets
                    if not position_corrected:
                        print(f"\n🎯 Position '{position_name}':")
                    
                    print(f"   Original observation_offset Y-rotation: {np.degrees(ry_offset):.1f}°")
                    
                    corrected_offset = correct_apriltag_180_ambiguity(offset)
                    position_data['observation_offset'] = corrected_offset.tolist()
                    
                    if not position_corrected:
                        corrections_made += 1
                    
                    print(f"   ✅ Corrected observation_offset Y-rotation: {np.degrees(corrected_offset[4]):.1f}°")
    
    print(f"\n📈 Summary:")
    print(f"   Positions processed: {positions_processed}")
    print(f"   Positions with 180° ambiguity corrected: {corrections_made}")
    
    # Save corrected file
    print(f"\n💾 Saving corrected positions to: {output_file}")
    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, indent=2)
    
    print("✅ 180° ambiguity correction completed!")
    print(f"📁 Compare files:")
    print(f"   Original: {input_file}")
    print(f"   Corrected: {output_file}")


def main():
    """Main function to fix taught positions ambiguity"""
    input_file = "src/ur_toolkit/positions/taught_positions.yaml"
    output_file = "src/ur_toolkit/positions/taught_positions_corrected.yaml"
    
    # Convert to absolute paths
    input_path = Path(input_file).resolve()
    output_path = Path(output_file).resolve()
    
    if not input_path.exists():
        print(f"❌ Input file not found: {input_path}")
        return
    
    print("🎯 180° Ambiguity Correction Tool for Taught Positions")
    print("=" * 60)
    
    fix_taught_positions_ambiguity(str(input_path), str(output_path))
    
    print(f"\n🔍 Next steps:")
    print(f"1. Review the differences between the files")
    print(f"2. If satisfied, replace the original file:")
    print(f"   cp '{output_file}' '{input_file}'")


if __name__ == "__main__":
    main()