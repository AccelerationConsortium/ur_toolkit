#!/usr/bin/env python3
"""
Robot Workflow Execution System
Executes sequential robot operations from YAML workflow definitions
"""

import yaml
import time
import numpy as np
from datetime import datetime
from pathlib import Path
import sys

# Add parent directory and setup directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "setup"))

# Import robot control modules
from robots.ur.ur_controller import URController
from config_manager import config

# Import visual servoing
try:
    from visual_servo.visual_servo_engine import VisualServoEngine
    VISUAL_SERVO_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  Visual servoing not available: {e}")
    VISUAL_SERVO_AVAILABLE = False


class WorkflowExecutor:
    """Execute robot workflows defined in YAML format"""

    def __init__(self, taught_positions_file=None):
        """
        Initialize workflow executor

        Args:
            taught_positions_file: Path to taught positions YAML file
        """
        self.robot = None
        self.taught_positions = {}
        self.workflow_history = []
        self.visual_servo_engine = None

        # Load taught positions
        if taught_positions_file is None:
            taught_positions_file = Path(__file__).parent.parent / "positions" / "taught_positions.yaml"

        self.positions_file = Path(taught_positions_file)
        self.load_taught_positions()

        # Initialize visual servo engine
        self.init_visual_servo_engine()

        print("🤖 Workflow Executor initialized")
        print(f"📍 Loaded {len(self.taught_positions.get('positions', {}))} taught positions")

    def load_taught_positions(self):
        """Load taught positions from YAML file"""
        try:
            if self.positions_file.exists():
                with open(self.positions_file, 'r') as f:
                    self.taught_positions = yaml.safe_load(f) or {}
                print(f"✅ Loaded positions from: {self.positions_file}")
            else:
                print(f"⚠️  Positions file not found: {self.positions_file}")
                self.taught_positions = {'positions': {}}
        except Exception as e:
            print(f"❌ Error loading positions: {e}")
            self.taught_positions = {'positions': {}}

    def init_visual_servo_engine(self):
        """Initialize visual servo engine if available"""
        try:
            # Import AprilTag detector
            from apriltag_detection import AprilTagDetector

            # Initialize visual servo engine without robot (robot set later)
            detector = AprilTagDetector()
            self.visual_servo_engine = VisualServoEngine(None, self.positions_file, detector)
            print("✅ Visual servo engine initialized")
        except Exception as e:
            print(f"⚠️  Visual servo engine unavailable: {e}")
            self.visual_servo_engine = None

    def connect_robot(self):
        """Connect to the robot"""
        try:
            robot_ip = config.get('robot', {}).get('ip', '192.168.0.10')
            # URController connects automatically in __init__
            self.robot = URController(robot_ip, read_only=False)

            # Check if connection was successful by testing basic functionality
            try:
                _ = self.robot.get_tcp_pose()
                print(f"🔌 Connected to robot at {robot_ip}")

                # Initialize visual servo engine now that robot is connected
                if self.visual_servo_engine:
                    self.visual_servo_engine.set_robot_controller(self.robot)

                return True
            except Exception as e:
                print(f"❌ Robot not responding: {e}")
                return False

        except Exception as e:
            print(f"❌ Robot connection error: {e}")
            return False

    def disconnect_robot(self):
        """Disconnect from robot"""
        if self.robot:
            self.robot.close()  # URController uses close() method
            print("🔌 Disconnected from robot")

    def execute_workflow(self, workflow_file, step_mode=False):
        """
        Execute a workflow from YAML file

        Args:
            workflow_file: Path to workflow YAML file
            step_mode: If True, prompt user before each step
        """
        workflow_path = Path(workflow_file)
        if not workflow_path.exists():
            print(f"❌ Workflow file not found: {workflow_path}")
            return False

        try:
            with open(workflow_path, 'r') as f:
                workflow = yaml.safe_load(f)
        except Exception as e:
            print(f"❌ Error loading workflow: {e}")
            return False

        return self.execute_workflow_dict(workflow, step_mode)

    def execute_workflow_dict(self, workflow, step_mode=False):
        """
        Execute a workflow from dictionary

        Args:
            workflow: Workflow dictionary
            step_mode: If True, prompt user before each step
        """
        if not self.robot:
            print("❌ Robot not connected. Call connect_robot() first.")
            return False

        workflow_name = workflow.get('name', 'Unnamed Workflow')
        description = workflow.get('description', 'No description')
        steps = workflow.get('steps', [])

        print(f"\n🚀 Executing Workflow: {workflow_name}")
        print(f"📝 Description: {description}")
        print(f"📋 Steps: {len(steps)}")
        if step_mode:
            print("🐾 Step-by-step mode: Press ENTER to continue each step, 'q' to quit")
        print("=" * 60)

        start_time = datetime.now()
        success_count = 0

        for i, step in enumerate(steps, 1):
            # Auto-generate step name based on action and parameters
            action = step.get('action', 'unknown')
            step_name = self._generate_step_name(step, i)

            # Auto-skip delays in step-through mode
            if step_mode and action == 'delay':
                duration = step.get('duration', 0)
                print(f"\n⏩ Step {i}/{len(steps)}: {step_name}")
                print(f"⏭️ Auto-skipping delay ({duration}s) in step-through mode")
                success_count += 1
                print(f"✅ Step {i} completed successfully")
                continue

            # Step-by-step prompting for non-delay steps
            if step_mode:
                print(f"\n🔍 Next step {i}/{len(steps)}: {step_name}")
                if action in ['movel', 'movej']:
                    position = step.get('position', 'unknown')
                    move_type = "Joint move" if action == 'movej' else "Linear move"
                    print(f"   Action: {move_type} to position '{position}'")
                elif action == 'gripper_open':
                    print("   Action: Open gripper")
                elif action == 'gripper_close':
                    print("   Action: Close gripper")
                elif action == 'offset_move':
                    offset = step.get('offset', [0, 0, 0, 0, 0, 0])
                    print(f"   Action: Offset move {offset}")
                else:
                    print(f"   Action: {action}")

                try:
                    user_input = input("Continue? (ENTER/q to quit/s to skip): ").lower().strip()
                    if user_input == 'q' or user_input == 'quit':
                        print("🛑 Workflow stopped by user")
                        break
                    elif user_input == 's' or user_input == 'skip':
                        print(f"⏭️  Skipping step {i}")
                        continue
                except (EOFError, KeyboardInterrupt):
                    print("\n🛑 Workflow interrupted by user")
                    break

            print(f"\n⏩ Step {i}/{len(steps)}: {step_name}")

            if self.execute_step(step, step_mode):
                success_count += 1
                print(f"✅ Step {i} completed successfully")
            else:
                print(f"❌ Step {i} failed - stopping workflow")
                break

        end_time = datetime.now()
        duration = (end_time - start_time).total_seconds()

        print("\n" + "=" * 60)
        print(f"🏁 Workflow Complete: {success_count}/{len(steps)} steps successful")
        print(f"⏱️  Duration: {duration:.2f} seconds")

        # Record workflow execution
        self.workflow_history.append({
            'name': workflow_name,
            'start_time': start_time.isoformat(),
            'end_time': end_time.isoformat(),
            'duration': duration,
            'steps_total': len(steps),
            'steps_successful': success_count,
            'success': success_count == len(steps)
        })

        return success_count == len(steps)

    def execute_step(self, step, step_mode=False):
        """Execute a single workflow step"""
        action = step.get('action', '').lower()

        try:
            if action in ['movel', 'movej']:
                return self._move_to_position(step)
            elif action == 'offset_move':
                return self._offset_move(step)
            elif action == 'gripper_open':
                return self._gripper_open(step)
            elif action == 'gripper_close':
                return self._gripper_close(step)
            elif action == 'gripper_activate':
                return self._gripper_activate(step)
            elif action == 'gripper_position':
                return self._gripper_position(step)
            elif action == 'gripper_params':
                return self._gripper_params(step)
            elif action == 'delay':
                return self._delay(step, step_mode)
            elif action == 'verify_position':
                return self._verify_position(step)
            elif action == 'visual_servo':
                return self._visual_servo_action(step)
            else:
                print(f"❌ Unknown action: {action}")
                return False

        except Exception as e:
            print(f"❌ Step execution error: {e}")
            return False

    def _generate_step_name(self, step, step_number):
        """Generate a descriptive step name based on action and parameters"""
        action = step.get('action', 'unknown').lower()  # Convert to lowercase for consistency
        name = step.get('name')  # Use explicit name if provided

        if name:
            return name

        if action in ['movel', 'movej']:
            position = step.get('position', 'unknown')
            move_type = "Joint move" if action == 'movej' else "Move"
            return f"{move_type} to {position}"
        elif action == 'offset_move':
            offset = step.get('offset', [0, 0, 0, 0, 0, 0])
            # Format offset nicely - show only non-zero values
            offset_str = []
            labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
            for i, val in enumerate(offset[:6]):
                if val != 0:
                    if i < 3:  # Position
                        offset_str.append(f"{labels[i]}:{val * 1000:.0f}mm")
                    else:  # Rotation
                        offset_str.append(f"{labels[i]}:{val:.3f}rad")
            return f"Offset move ({', '.join(offset_str) if offset_str else 'no offset'})"
        elif action == 'gripper_open':
            return "Open gripper"
        elif action == 'gripper_close':
            return "Close gripper"
        elif action == 'gripper_activate':
            return "Activate gripper"
        elif action == 'delay':
            duration = step.get('duration', 0)
            return f"Wait {duration}s"
        elif action == 'gripper_position':
            pos = step.get('position', 0)
            return f"Gripper to {pos}"
        elif action == 'verify_position':
            position = step.get('position', 'unknown')
            return f"Verify at {position}"
        elif action == 'visual_servo':
            position = step.get('position', 'unknown')
            return f"Visual servo to {position}"
        else:
            return f"Step {step_number} ({action})"

    def _move_to_position(self, step):
        """Move to a taught position with optional visual servoing"""
        position_name = step.get('position')
        speed = step.get('speed', 0.1)  # Default 10% speed
        action = step.get('action', '').lower()
        use_visual_servo = step.get('visual_servo', False)

        if not position_name:
            print("❌ No position specified")
            return False

        positions = self.taught_positions.get('positions', {})
        if position_name not in positions:
            print(f"❌ Position '{position_name}' not found")
            return False

        position_data = positions[position_name]
        coordinates = position_data['coordinates']

        # Determine movement type
        if action == 'movej':
            print(f"🔄 Joint move to position: {position_name}")
        else:
            print(f"🎯 Linear move to position: {position_name}")

        # Check if visual servoing is requested
        if use_visual_servo and self.visual_servo_engine:
            print(f"�️ Visual servoing enabled for {position_name}")

            # Use visual servo engine to get corrected pose
            try:
                corrected_pose = self.visual_servo_engine.get_corrected_pose(position_name, coordinates)
                if corrected_pose is not None:
                    target_pose = corrected_pose
                    print("✅ Using visual servo corrected pose")
                else:
                    target_pose = np.array(coordinates)
                    print("⚠️ Visual servo correction failed, using original pose")
            except Exception as e:
                print(f"⚠️ Visual servo error: {e}, using original pose")
                target_pose = np.array(coordinates)
        else:
            target_pose = np.array(coordinates)

        print(f"📍 Target coordinates: {[f'{x:.3f}' for x in target_pose]}")

        # Use joint move or linear move based on action
        if action == 'movej':
            # For now, joint moves use the same function (could be extended in URController)
            # In a full implementation, this would use moveJ commands in the robot controller
            return self.robot.move_to_pose(target_pose, speed=speed)
        else:
            # Linear move (default) - uses moveL commands
            return self.robot.move_to_pose(target_pose, speed=speed)

    def _offset_move(self, step):
        """Move by an offset from current position"""
        offset = step.get('offset', [0, 0, 0, 0, 0, 0])
        speed = step.get('speed', 0.1)
        coordinate_system = step.get('coordinate_system', 'tcp')  # 'tcp' or 'base'

        if len(offset) != 6:
            print("❌ Offset must be [x, y, z, rx, ry, rz]")
            return False

        print(f"📐 Offset move: {[f'{x:.3f}' for x in offset]} ({coordinate_system})")

        # Get current position
        current_pose = self.robot.get_tcp_pose()

        # Apply offset
        if coordinate_system == 'tcp':
            # For TCP coordinate system, we need to transform the offset
            # For simplicity, we'll just add the offset to current pose
            target_pose = current_pose + np.array(offset)
        else:  # base coordinate system
            target_pose = current_pose + np.array(offset)

        return self.robot.move_to_pose(target_pose, speed=speed)

    def _gripper_open(self, step):
        """Open the gripper"""
        print("✋ Opening gripper...")
        success = self.robot.gripper_open()
        if success:
            # Wait for movement to complete
            wait_time = step.get('wait_time', 1.0)  # Reduced default from 2.0
            print(f"⏳ Waiting {wait_time}s for gripper to open...")
            time.sleep(wait_time)
        return success

    def _gripper_close(self, step):
        """Close the gripper"""
        print("✊ Closing gripper...")
        success = self.robot.gripper_close()
        if success:
            # Wait for movement to complete
            wait_time = step.get('wait_time', 1.0)  # Reduced default from 2.0
            print(f"⏳ Waiting {wait_time}s for gripper to close...")
            time.sleep(wait_time)
        return success

    def _gripper_activate(self, step):
        """Activate the gripper (required before first use)"""
        print("🤖 Activating gripper...")
        success = self.robot.gripper_activate()
        if success:
            # Wait for activation to complete
            wait_time = step.get('wait_time', 2.0)  # Reduced default from 3.0
            print(f"⏳ Waiting {wait_time}s for gripper activation...")
            time.sleep(wait_time)

            # Set default force and speed after activation
            print("🔧 Setting gripper parameters...")
            try:
                self.robot.gripper_set_force(50)  # Default force
                time.sleep(0.2)  # Reduced from 0.5
                self.robot.gripper_set_speed(100)  # Default speed
                time.sleep(0.2)  # Reduced from 0.5
            except Exception as e:
                print(f"⚠️ Warning: Could not set gripper parameters: {e}")

            # Check status
            try:
                status = self.robot.gripper_get_status()
                if status:
                    print(f"📊 Gripper status: {status.get('raw_response', 'Ready')}")
            except Exception as e:
                print(f"⚠️ Warning: Could not get gripper status: {e}")

        return success

    def _delay(self, step, step_mode=False):
        """Wait for specified time"""
        duration = step.get('duration', 1.0)
        if step_mode:
            print(f"⏭️ Skipping delay ({duration}s) in step-through mode")
        else:
            print(f"⏳ Waiting {duration} seconds...")
            time.sleep(duration)
        return True

    def _verify_position(self, step):
        """Verify current position matches expected position"""
        position_name = step.get('position')
        tolerance = step.get('tolerance', 0.01)  # 1cm tolerance

        if not position_name:
            print("❌ No position specified for verification")
            return False

        positions = self.taught_positions.get('positions', {})
        if position_name not in positions:
            print(f"❌ Position '{position_name}' not found")
            return False

        expected_pose = np.array(positions[position_name]['coordinates'])
        current_pose = self.robot.get_tcp_pose()

        # Check position difference (translation only)
        pos_diff = np.linalg.norm(current_pose[:3] - expected_pose[:3])

        print(f"🔍 Verifying position: {position_name}")
        print(f"📏 Position difference: {pos_diff:.4f}m (tolerance: {tolerance}m)")

        if pos_diff <= tolerance:
            print("✅ Position verification passed")
            return True
        else:
            print("❌ Position verification failed - difference too large")
            return False

    def _visual_servo_action(self, step):
        """Execute visual servoing to correct position based on AprilTag detection"""
        position_name = step.get('position')

        if not position_name:
            print("❌ No position specified for visual servo")
            return False

        if not self.visual_servo_engine:
            print("❌ Visual servo engine not available")
            return False

        positions = self.taught_positions.get('positions', {})
        if position_name not in positions:
            print(f"❌ Position '{position_name}' not found")
            return False

        print(f"👁️ Starting visual servo to position: {position_name}")

        try:
            # Execute visual servoing
            success, result_data = self.visual_servo_engine.visual_servo_to_position(
                position_name,
                update_stored_pose=True
            )

            if success:
                print("✅ Visual servo completed successfully")
                return True
            else:
                print("❌ Visual servo failed")
                return False

        except Exception as e:
            print(f"❌ Visual servo error: {e}")
            return False

    def get_workflow_history(self):
        """Get workflow execution history"""
        return self.workflow_history

    def list_available_positions(self):
        """List all available taught positions"""
        positions = self.taught_positions.get('positions', {})
        print(f"\n📋 Available Positions ({len(positions)}):")
        print("=" * 50)

        for name, data in positions.items():
            pos_type = data.get('pose_type', 'unknown')
            description = data.get('description', 'No description')
            print(f"🎯 {name} ({pos_type})")
            if description:
                print(f"   📝 {description}")

    def _gripper_position(self, step):
        """Set gripper to specific position"""
        position = step.get('position', 128)  # Default to half-closed
        print(f"🎯 Setting gripper position to {position}...")
        success = self.robot.gripper_set_position(position)
        if success:
            # Wait for movement to complete
            wait_time = step.get('wait_time', 1.0)  # Reduced default from 2.0
            print(f"⏳ Waiting {wait_time}s for gripper to move to position...")
            time.sleep(wait_time)
        return success

    def _gripper_params(self, step):
        """Set gripper force and/or speed parameters"""
        force = step.get('force')
        speed = step.get('speed')

        success = True
        if force is not None:
            print(f"💪 Setting gripper force to {force}...")
            if not self.robot.gripper_set_force(force):
                success = False
            time.sleep(0.5)

        if speed is not None:
            print(f"⚡ Setting gripper speed to {speed}...")
            if not self.robot.gripper_set_speed(speed):
                success = False
            time.sleep(0.5)

        return success


def main():
    """Main function for standalone execution"""
    import argparse

    parser = argparse.ArgumentParser(description='Robot Workflow Executor')
    parser.add_argument('workflow', help='Workflow YAML file to execute')
    parser.add_argument('--positions', help='Taught positions YAML file')
    parser.add_argument('--list-positions', action='store_true', help='List available positions')

    args = parser.parse_args()

    # Create executor
    executor = WorkflowExecutor(args.positions)

    if args.list_positions:
        executor.list_available_positions()
        return

    # Connect to robot
    if not executor.connect_robot():
        return

    try:
        # Execute workflow
        success = executor.execute_workflow(args.workflow)

        if success:
            print("\n🎉 Workflow executed successfully!")
        else:
            print("\n💥 Workflow execution failed!")

    finally:
        executor.disconnect_robot()


if __name__ == "__main__":
    main()
