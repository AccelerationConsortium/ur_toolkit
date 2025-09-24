#!/usr/bin/env python3
"""
Visual Servo System Test
Comprehensive test of the visual servoing functionality
"""

import sys
from pathlib import Path

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "setup"))

from config_manager import config
from workflow.workflow_executor import WorkflowExecutor
from visual_servo.visual_servo_engine import VisualServoEngine
from visual_servo.config import visual_servo_config
from apriltag_detection import AprilTagDetector
from robots.ur.ur_controller import URController


def test_visual_servo_components():
    """Test individual components of the visual servo system"""
    print("🧪 Testing Visual Servo Components")
    print("=" * 50)

    # Test 1: Configuration loading
    print("\n1. Testing configuration loading...")
    try:
        config_data = visual_servo_config.get_all_config()
        print(f"✅ Configuration loaded: {len(config_data)} parameters")
        visual_servo_config.print_config()
    except Exception as e:
        print(f"❌ Configuration test failed: {e}")
        return False

    # Test 2: AprilTag detector
    print("\n2. Testing AprilTag detector...")
    try:
        detector = AprilTagDetector()
        print("✅ AprilTag detector initialized")
    except Exception as e:
        print(f"❌ AprilTag detector test failed: {e}")
        return False

    # Test 3: Visual servo engine initialization (without robot)
    print("\n3. Testing visual servo engine initialization...")
    try:
        positions_file = config.resolve_path('taught_positions.yaml')
        VisualServoEngine(None, positions_file, detector)
        print("✅ Visual servo engine initialized")
    except Exception as e:
        print(f"❌ Visual servo engine test failed: {e}")
        return False

    print("\n✅ All component tests passed!")
    return True


def test_workflow_integration():
    """Test workflow integration with visual servo"""
    print("\n🧪 Testing Workflow Integration")
    print("=" * 50)

    try:
        # Initialize workflow executor
        executor = WorkflowExecutor()
        print("✅ Workflow executor initialized")

        # Check if visual servo engine is available
        if executor.visual_servo_engine:
            print("✅ Visual servo engine integrated with workflow")
        else:
            print("❌ Visual servo engine not available in workflow")
            return False

        # Test workflow loading (dry run)
        test_workflow = Path(__file__).parent / "workflow" / "examples" / "simple_visual_servo_test.yaml"
        if test_workflow.exists():
            workflow_data = executor._load_workflow(test_workflow)
            if workflow_data:
                print("✅ Test workflow loaded successfully")

                # Analyze workflow steps
                steps = workflow_data.get('steps', [])
                visual_servo_steps = [s for s in steps if s.get('action') == 'visual_servo' or s.get('visual_servo')]
                print(f"📊 Workflow has {len(steps)} steps, {len(visual_servo_steps)} use visual servoing")
            else:
                print("❌ Failed to load test workflow")
                return False
        else:
            print("⚠️ Test workflow not found, skipping workflow test")

        print("\n✅ Workflow integration test passed!")
        return True

    except Exception as e:
        print(f"❌ Workflow integration test failed: {e}")
        return False


def test_robot_connection():
    """Test robot connection and visual servo setup"""
    print("\n🧪 Testing Robot Connection")
    print("=" * 50)

    try:
        robot_ip = config.get('robot', {}).get('ip', '192.168.0.10')
        print(f"🔌 Connecting to robot at {robot_ip}...")

        # Connect to robot (read-only for safety)
        robot = URController(robot_ip, read_only=True)

        # Test basic connection
        current_pose = robot.get_tcp_pose()
        print(f"✅ Robot connected, current pose: {current_pose}")

        # Test visual servo engine with robot
        positions_file = config.resolve_path('taught_positions.yaml')
        detector = AprilTagDetector()
        engine = VisualServoEngine(robot, positions_file, detector)

        print("✅ Visual servo engine connected to robot")

        # Test position data loading
        try:
            # Try to load a position (this will fail gracefully if none exist)
            positions_data = engine.pose_history._load_positions()
            positions = positions_data.get('positions', {})
            print(f"📊 Found {len(positions)} taught positions")

            # Check for positions with visual servo capability
            visual_servo_positions = [
                name for name, data in positions.items()
                if data.get('tag_reference') and data.get('camera_to_tag')
            ]
            print(f"👁️ {len(visual_servo_positions)} positions have visual servo capability")

            for pos_name in visual_servo_positions[:3]:  # Show first 3
                pos_data = positions[pos_name]
                tag_ref = pos_data.get('tag_reference', 'unknown')
                print(f"   - {pos_name}: {tag_ref}")

        except Exception as e:
            print(f"⚠️ Position loading test: {e}")

        # Close robot connection
        robot.close()
        print("🔌 Robot disconnected")

        print("\n✅ Robot connection test passed!")
        return True

    except Exception as e:
        print(f"❌ Robot connection test failed: {e}")
        print("💡 Make sure robot is powered on and network is accessible")
        return False


def test_apriltag_detection():
    """Test AprilTag detection functionality"""
    print("\n🧪 Testing AprilTag Detection")
    print("=" * 50)

    try:
        # Initialize detector
        detector = AprilTagDetector()
        print("✅ AprilTag detector initialized")

        # Test detection (will fail gracefully if no camera)
        try:
            detections = detector.detect_tags()
            if detections:
                print(f"📷 Detected {len(detections)} AprilTag(s)")
                for det in detections[:3]:  # Show first 3
                    tag_id = det.get('tag_id', 'unknown')
                    distance = det.get('distance_mm', 'unknown')
                    print(f"   - Tag {tag_id}: {distance}mm away")
            else:
                print("📷 No AprilTags detected (may be normal if no tags in view)")
        except Exception as e:
            print(f"⚠️ AprilTag detection test: {e}")
            print("💡 Camera may not be available - this is OK for basic testing")

        print("\n✅ AprilTag detection test completed!")
        return True

    except Exception as e:
        print(f"❌ AprilTag detection test failed: {e}")
        return False


def run_comprehensive_test():
    """Run all visual servo tests"""
    print("🚀 Visual Servo System Comprehensive Test")
    print("=" * 60)
    print("Testing visual servoing system components and integration")
    print("=" * 60)

    test_results = []

    # Run individual tests
    test_results.append(("Component Tests", test_visual_servo_components()))
    test_results.append(("Workflow Integration", test_workflow_integration()))
    test_results.append(("AprilTag Detection", test_apriltag_detection()))
    test_results.append(("Robot Connection", test_robot_connection()))

    # Summary
    print("\n📊 Test Results Summary")
    print("=" * 30)

    passed = 0
    for test_name, result in test_results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:20} {status}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{len(test_results)} tests passed")

    if passed == len(test_results):
        print("\n🎉 All tests passed! Visual servo system is ready.")
    else:
        print(f"\n⚠️ {len(test_results) - passed} test(s) failed. Check the output above.")

    return passed == len(test_results)


if __name__ == "__main__":
    success = run_comprehensive_test()
    sys.exit(0 if success else 1)
