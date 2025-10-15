#!/usr/bin/env python3
"""
Dynamic Pose Correction Test
Tests continuous pose correction as AprilTag or robot moves
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent directories for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.pose_correction.pose_correction_engine import PoseCorrectionEngine
from ur_toolkit.config_manager import config, get_camera_host, get_camera_port


def test_dynamic_pose_correction(robot_ip: str, tag_id: int = 2, cycles: int = 5):
    """Test dynamic pose correction with multiple cycles"""
    
    print(f"🔄 Dynamic Pose Correction Test")
    print(f"Robot IP: {robot_ip}")
    print(f"Tag ID: {tag_id}")
    print(f"Test cycles: {cycles}")
    print("=" * 50)

    try:
        # Initialize robot
        print("🔌 Connecting to robot...")
        robot = URController(robot_ip)
        print("✅ Robot connected")
        
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
        print("✅ AprilTag detector initialized")
        
        # Initialize pose correction engine
        pose_engine = PoseCorrectionEngine(robot, camera, detector)
        print("✅ Pose correction engine initialized")
        
        # Set initial target pose
        print(f"\n🎯 Setting initial target pose from tag {tag_id}...")
        if not pose_engine.set_target_from_current_detection(tag_id):
            print("❌ Failed to set initial target pose")
            return False
        print("✅ Initial target pose set")
        
        print("\n⚠️  DYNAMIC TEST INSTRUCTIONS:")
        print("   1. The robot will perform multiple correction cycles")
        print("   2. Between cycles, you can:")
        print("      - Move the AprilTag slightly (2-3cm)")
        print("      - Manually jog the robot with teach pendant")
        print("      - Leave everything stationary to test stability")
        print("   3. Each cycle will attempt to correct any detected errors")
        print("   4. Press Ctrl+C to stop the test early")
        
        input("\n📍 Press ENTER to start dynamic testing...")
        
        # Run correction cycles
        successful_cycles = 0
        
        for cycle in range(1, cycles + 1):
            print(f"\n🔄 === Cycle {cycle}/{cycles} ===")
            
            # Small delay to allow for manual adjustments
            print("⏳ 3 second pause for adjustments...")
            time.sleep(3)
            
            # Perform pose correction
            print(f"🚀 Starting pose correction cycle {cycle}...")
            success, metrics = pose_engine.correct_to_target(tag_id)
            
            if success:
                print(f"✅ Cycle {cycle} successful!")
                print(f"   Converged in {metrics['iterations']} iterations")
                print(f"   Final error: {metrics['final_error']:.4f}")
                successful_cycles += 1
            else:
                print(f"❌ Cycle {cycle} failed")
                print(f"   Iterations: {metrics['iterations']}")
                print(f"   Final error: {metrics.get('final_error', 'unknown')}")
            
            # Brief pause between cycles
            if cycle < cycles:
                print("⏸️  Brief pause before next cycle...")
                time.sleep(2)
        
        # Results summary
        print(f"\n🏁 Dynamic Test Complete!")
        print(f"   Successful cycles: {successful_cycles}/{cycles}")
        print(f"   Success rate: {successful_cycles/cycles*100:.1f}%")
        
        return successful_cycles == cycles
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return False
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False
    finally:
        # Clean up
        if 'robot' in locals():
            robot.close()
            print("🔌 Disconnected from robot")


def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description='Test Dynamic Pose Correction')
    parser.add_argument('--robot-ip', default='192.168.0.10', help='Robot IP address')
    parser.add_argument('--tag-id', type=int, default=2, help='AprilTag ID to track')
    parser.add_argument('--cycles', type=int, default=5, help='Number of correction cycles')
    
    args = parser.parse_args()
    
    # Run dynamic test
    success = test_dynamic_pose_correction(args.robot_ip, args.tag_id, args.cycles)
    
    if success:
        print("\n🎉 All dynamic tests passed!")
        sys.exit(0)
    else:
        print("\n💥 Dynamic tests failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()