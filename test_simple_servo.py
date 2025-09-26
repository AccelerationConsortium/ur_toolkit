#!/usr/bin/env python3

import sys
sys.path.insert(0, 'src')

from ur_toolkit.config.config_manager import ConfigManager
from ur_toolkit.visual_servo.visual_servo_engine import VisualServoEngine
from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.positions.position_manager import PositionManager
import numpy as np

def test_simple_servo():
    print("🧪 Testing simple mode visual servoing...")
    
    # Load config
    config = ConfigManager()
    print(f"✅ Simple mode enabled: {getattr(config, 'simple_mode', False)}")
    
    # Initialize components
    position_manager = PositionManager(config)
    robot = URController(config)
    visual_servo = VisualServoEngine(config, robot)
    
    print("🤖 Connecting to robot...")
    robot.connect()
    
    try:
        print("📍 Moving to observation pose...")
        obs_pose = position_manager.get_position('right-column-observe')
        robot.move_to_pose(obs_pose['coordinates'])
        
        print("👁️ Testing single visual servo iteration...")
        success, result = visual_servo.visual_servo_to_position('grasp-B', max_iterations=1)
        
        print(f"✅ Result: {success}")
        if result and 'corrections_applied' in result:
            corrections = result['corrections_applied']
            if corrections:
                last_correction = corrections[-1]
                print(f"📊 Applied correction: {last_correction}")
        
    finally:
        robot.close()

if __name__ == "__main__":
    test_simple_servo()