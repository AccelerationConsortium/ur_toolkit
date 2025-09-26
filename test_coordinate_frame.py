#!/usr/bin/env python3
"""
Coordinate Frame Diagnostic Tool
Helps determine the correct camera-to-robot coordinate transformation
"""

import sys
sys.path.insert(0, 'src')

from ur_toolkit.config.config_manager import ConfigManager
from ur_toolkit.robots.ur.ur_controller import URController


def test_coordinate_mapping():
    """
    Test robot movements in each axis to understand coordinate frame
    """
    print("🧪 Coordinate Frame Diagnostic Test")
    print("=====================================")
    
    config = ConfigManager()
    robot = URController(config)
    
    try:
        robot.connect()
        print("✅ Connected to robot")
        
        # Get current position
        current_pose = robot.get_current_pose()
        print(f"📍 Current pose: {current_pose}")
        
        print("\n🔍 Testing robot coordinate directions:")
        print("Watch the robot and note which direction it moves for each test")
        
        # Test X direction
        print("\n1️⃣ Testing Robot +X direction (should be FORWARD)")
        input("Press Enter to move robot +X (10mm)...")
        test_pose = current_pose.copy()
        test_pose[0] += 0.01  # +10mm in X
        robot.move_to_pose(test_pose, speed=0.05)
        direction = input("Which direction did robot move? (forward/back/left/right/up/down): ")
        print(f"   Robot +X = {direction}")
        
        # Return to start
        robot.move_to_pose(current_pose, speed=0.05)
        
        # Test Y direction
        print("\n2️⃣ Testing Robot +Y direction (should be LEFT)")
        input("Press Enter to move robot +Y (10mm)...")
        test_pose = current_pose.copy()
        test_pose[1] += 0.01  # +10mm in Y
        robot.move_to_pose(test_pose, speed=0.05)
        direction = input("Which direction did robot move? (forward/back/left/right/up/down): ")
        print(f"   Robot +Y = {direction}")
        
        # Return to start
        robot.move_to_pose(current_pose, speed=0.05)
        
        # Test Z direction
        print("\n3️⃣ Testing Robot +Z direction (should be UP)")
        input("Press Enter to move robot +Z (10mm)...")
        test_pose = current_pose.copy()
        test_pose[2] += 0.01  # +10mm in Z
        robot.move_to_pose(test_pose, speed=0.05)
        direction = input("Which direction did robot move? (forward/back/left/right/up/down): ")
        print(f"   Robot +Z = {direction}")
        
        # Return to start
        robot.move_to_pose(current_pose, speed=0.05)
        
        print("\n✅ Coordinate frame test complete!")
        print("Use this information to determine correct camera-to-robot mapping")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        robot.close()


if __name__ == "__main__":
    test_coordinate_mapping()